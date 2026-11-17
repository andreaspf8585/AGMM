#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <utility>

// ============================================================
// HeatRev
// ============================================================
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// HeatRev is a 12 HP hybrid reverb/echo effect built from three layers:
//   1. A stereo delay/echo stage with feedback and L/R time spread.
//   2. A "Cathedral" reverb: pre-delay, early reflections, input diffusion
//      and an 8-line feedback delay network (FDN) with an orthonormal Hadamard
//      feedback matrix for guaranteed stability.
//   3. A 2D "heat grid" diffusion network that adds a slowly moving, warm
//      resonant texture to the tail.
//
// The display shows the heat grid as a smooth, true-aspect thermal map with
// the injection edges marked, plus level meters and a history
// trace of the grid energy.
//
// ROBUSTNESS (why this version cannot "scream" after a bad start)
//   - Inputs are sanitised (NaN/inf -> 0, hard limit) before they enter any
//     feedback path, so a glitching upstream module at patch load cannot poison
//     the echo loop.
//   - The echo loop itself is clamped and finite-checked.
//   - A watchdog looks at the internal levels every sample; if anything runs
//     away, ALL state (echo, reverb, grid, filters) is flushed and the wet path
//     fades back in. The old code only flushed on NaN and never flushed the echo.
//   - All delay-line indexing is integer based and bounds-safe at any sample
//     rate (the old fractional read could index one past the end).
//   - Buffers are allocated once (no allocation on the audio thread).
//
// File structure:
//   1. DSP building blocks: SimpleLPF, StereoDelay, Cathedral, HeatGrid.
//   2. Layout constants (12 HP = 180x380 px).
//   3. HeatRev module: state, config, display snapshot and process().
//   4. Anonymous-namespace UI: HRPanelBackground and HRDisplayWidget.
//   5. HeatRevWidget: places knobs, jacks and the display.
//   6. Model registration.
// -----------------------------------------------------------------------------

// ------------------------------------------------------------
// Safety constants
// ------------------------------------------------------------
static const float HR_IN_LIM = 24.f;      // input guard, volts
static const float HR_LOOP_LIM = 128.f;   // clamp inside the echo loop
static const float HR_RUNAWAY = 5000.f;   // summed internal level that counts as a runaway

// ------------------------------------------------------------
// DSP
// ------------------------------------------------------------

// Replaces NaN/inf by 0 and clamps to +/-lim.
static inline float hrSanitize(float x, float lim)
{
    if (!std::isfinite(x))
        return 0.f;
    return x > lim ? lim : (x < -lim ? -lim : x);
}

// One-pole low-pass
// Simple one-pole low-pass filter used to tame the heat-grid taps.
struct SimpleLPF
{
    float y = 0.0f;
    float a = 0.0f;

    // Sets the coefficient for a true cutoff of `fc` Hz at sample rate `sr`.
    void setHz(float fc, float sr)
    {
        a = 1.0f - std::exp(-2.0f * float(M_PI) * fc / sr);
        a = rack::math::clamp(a, 0.0f, 1.0f);
    }

    // Processes one sample through the one-pole low-pass.
    float process(float x)
    {
        y = y + a * (x - y);
        return y;
    }
};

// Stereo delay with feedback and fractional delay
// Independent L/R delay times, smoothed so turning TIME glides instead of
// clicking, linear-interpolated reads, buffers allocated once per sample rate.
struct StereoDelay
{
    std::vector<float> bufferL, bufferR;
    int writePos = 0;
    int bufferSize = 0;
    float sampleRate = 48000.0f;
    float targetL = 0.f, targetR = 0.f; // samples
    float curL = 0.f, curR = 0.f;       // smoothed samples
    float smCoef = 0.001f;
    float feedback = 0.0f;
    bool snap = true;

    // Allocates both buffers (1.05 s, enough for the 1000 ms clamp) for the
    // given sample rate. Called from prepare(), never from the audio loop.
    void init(float sr)
    {
        sampleRate = sr;
        bufferSize = (int)(sr * 1.05f) + 8;
        bufferL.assign(bufferSize, 0.f);
        bufferR.assign(bufferSize, 0.f);
        writePos = 0;
        smCoef = 1.f - std::exp(-1.f / (0.05f * sr)); // ~50 ms glide
        snap = true;
    }

    // Silences the buffers.
    void clear()
    {
        std::fill(bufferL.begin(), bufferL.end(), 0.f);
        std::fill(bufferR.begin(), bufferR.end(), 0.f);
        writePos = 0;
        snap = true;
    }

    // Sets the L/R delay times in milliseconds.
    void setDelayMs(float msL, float msR)
    {
        const float maxS = (float)(bufferSize - 4);
        targetL = rack::math::clamp(msL * 0.001f * sampleRate, 1.f, maxS);
        targetR = rack::math::clamp(msR * 0.001f * sampleRate, 1.f, maxS);
        if (snap)
        {
            curL = targetL;
            curR = targetR;
            snap = false;
        }
    }

    // Sets the feedback amount, clamped to 0..0.95 for stability.
    void setFeedback(float fb) { feedback = rack::math::clamp(fb, 0.0f, 0.95f); }

    // Processes one stereo sample: reads the delayed L/R values, writes the
    // input plus feedback into the buffers, and returns the delayed pair.
    std::pair<float, float> process(float inL, float inR)
    {
        curL += (targetL - curL) * smCoef;
        curR += (targetR - curR) * smCoef;

        const float readL = readBuffer(bufferL, curL);
        const float readR = readBuffer(bufferR, curR);

        float writeL = inL + feedback * readL;
        float writeR = inR + feedback * readR;
        if (!std::isfinite(writeL))
            writeL = 0.f;
        if (!std::isfinite(writeR))
            writeR = 0.f;
        bufferL[writePos] = rack::math::clamp(writeL, -HR_LOOP_LIM, HR_LOOP_LIM);
        bufferR[writePos] = rack::math::clamp(writeR, -HR_LOOP_LIM, HR_LOOP_LIM);
        if (++writePos >= bufferSize)
            writePos = 0;

        return {readL, readR};
    }

private:
    // Reads `d` samples behind the write head, integer + fraction so the index
    // can never leave the buffer (the old float-position version could land on
    // bufferSize through rounding).
    float readBuffer(const std::vector<float> &buf, float d) const
    {
        const int di = (int)d;
        const float fr = d - (float)di;
        int i0 = writePos - di;
        if (i0 < 0)
            i0 += bufferSize;
        int i1 = i0 - 1;
        if (i1 < 0)
            i1 += bufferSize;
        return buf[i0] + fr * (buf[i1] - buf[i0]);
    }
};

// ------------------------------------------------------------
// Cathedral: pre-delay + early reflections + input diffusion + 8-line FDN
//
// Stability by construction: the feedback matrix is an orthonormal Hadamard
// matrix and every line has a loop gain below 1 (set from the RT60), so the
// tail can only ever decay. Input level is normalised against the decay time so
// long tails do not get louder.
// ------------------------------------------------------------
static const int CATH_N = 8;
static const int CATH_BASE[CATH_N] = {1031, 1327, 1621, 1933, 2269, 2663, 3109, 3571}; // samples @ 48 kHz
static const int CATH_AP_L[4] = {113, 173, 307, 443};
static const int CATH_AP_R[4] = {127, 191, 331, 461};
static const float CATH_ER_MS_L[7] = {7.f, 19.f, 31.f, 47.f, 67.f, 89.f, 113.f};
static const float CATH_ER_MS_R[7] = {11.f, 23.f, 37.f, 53.f, 73.f, 97.f, 127.f};
static const float CATH_ER_G_L[7] = {0.80f, -0.65f, 0.55f, -0.45f, 0.38f, -0.30f, 0.24f};
static const float CATH_ER_G_R[7] = {-0.78f, 0.62f, -0.52f, 0.43f, -0.36f, 0.29f, -0.23f};

// Cathedral reverb engine: pre-delay, early reflections, four input diffusers
// per channel, and an 8-line FDN with per-line damping and Hadamard mixing.
struct Cathedral
{
    enum
    {
        N = CATH_N,
        LINE = 32768,
        LMASK = LINE - 1,
        PD = 131072,
        PDMASK = PD - 1,
        AP = 2048
    };

    std::vector<float> line[N];
    std::vector<float> pdL, pdR;
    std::vector<float> apL[4], apR[4];
    int w[N];
    int pdW = 0;
    int apLi[4], apRi[4];
    float lp[N], lfoPh[N], g[N];
    float inGain = 0.3f, scaleSm = 1.f, lastScale = -1.f, lastRT = -1.f, lastDamp = -1.f, b = 0.5f;
    float hpL = 0.f, hpR = 0.f;

    // Allocates every delay buffer to its final size and resets the engine.
    void init()
    {
        for (int i = 0; i < N; ++i)
            line[i].assign(LINE, 0.f);
        pdL.assign(PD, 0.f);
        pdR.assign(PD, 0.f);
        for (int k = 0; k < 4; ++k)
        {
            apL[k].assign(AP, 0.f);
            apR[k].assign(AP, 0.f);
        }
        reset();
    }

    // Clears all delay buffers, filter states, LFO phases and cached parameter
    // values so the reverb starts from silence.
    void reset()
    {
        for (int i = 0; i < N; ++i)
        {
            std::fill(line[i].begin(), line[i].end(), 0.f);
            w[i] = 0;
            lp[i] = 0.f;
            g[i] = 0.9f;
            lfoPh[i] = 0.8f * i;
        }
        std::fill(pdL.begin(), pdL.end(), 0.f);
        std::fill(pdR.begin(), pdR.end(), 0.f);
        for (int k = 0; k < 4; ++k)
        {
            std::fill(apL[k].begin(), apL[k].end(), 0.f);
            std::fill(apR[k].begin(), apR[k].end(), 0.f);
            apLi[k] = apRi[k] = 0;
        }
        pdW = 0;
        hpL = hpR = 0.f;
        lastScale = lastRT = lastDamp = -1.f;
    }

    // Runs one sample through a fixed-length all-pass delay line with gain
    // `gain`, returning the all-pass output. `len` is clamped to the buffer.
    static float allpass(std::vector<float> &buf, int &idx, int len, float gain, float x)
    {
        if (len < 2)
            len = 2;
        if (len > (int)buf.size())
            len = (int)buf.size();
        if (idx >= len)
            idx = 0;
        const float d = buf[idx];
        const float wv = x + gain * d;
        buf[idx] = wv;
        if (++idx >= len)
            idx = 0;
        return d - gain * wv;
    }

    // Reads the pre-delay buffer at a fractional delay `d` behind the write
    // head using linear interpolation, with power-of-two wrapping.
    float readPD(const std::vector<float> &buf, float d) const
    {
        d = std::min(d, (float)(PD - 4));
        const float pos = (float)pdW - d;
        int i0 = (int)std::floor(pos);
        const float fr = pos - (float)i0;
        i0 &= PDMASK;
        const int i1 = (i0 + 1) & PDMASK;
        return buf[i0] + fr * (buf[i1] - buf[i0]);
    }

    // fdn = diffuse tail, er = early reflections
    // Runs the full Cathedral reverb for one stereo sample.
    void process(float inL, float inR, float sr, float rt60, float damping,
                 float scaleTarget, float diffusion, float space,
                 float &fdnL, float &fdnR, float &erL, float &erR)
    {
        const float srk = sr / 48000.f;
        const float twoPi = 6.2831853f;
        rt60 = std::max(rt60, 0.1f);
        const float maxLen = (float)(LINE - 16) - 8.f * srk; // leaves room for the LFO depth

        if (lastScale < 0.f)
            scaleSm = scaleTarget;
        scaleSm += (scaleTarget - scaleSm) * 0.0001f; // slow, so SIZE changes don't click

        // Per-line loop gain from RT60 (-60 dB after rt60 seconds)
        if (lastScale < 0.f || std::fabs(scaleSm - lastScale) > 1e-4f || rt60 != lastRT)
        {
            lastScale = scaleSm;
            lastRT = rt60;
            float gm2 = 0.f;
            for (int i = 0; i < N; ++i)
            {
                const float len = std::min((float)CATH_BASE[i] * srk * scaleSm, maxLen);
                g[i] = std::pow(10.f, -3.f * len / (rt60 * sr));
                gm2 += g[i] * g[i];
            }
            // keeps reverb loudness roughly constant whatever the decay time
            inGain = std::sqrt(std::max(0.02f, 1.f - gm2 / (float)N));
        }

        // Damping: stone and air eat the highs on every trip round the loop
        if (damping != lastDamp)
        {
            lastDamp = damping;
            float fc = 14000.f * std::pow(0.0857f, damping); // 14 kHz .. 1.2 kHz
            fc = std::min(fc, 0.45f * sr);
            b = std::exp(-twoPi * fc / sr);
        }

        // Gentle low cut so long tails don't build up rumble
        const float hpc = 1.f - std::exp(-twoPi * 60.f / sr);
        hpL += (inL - hpL) * hpc;
        hpR += (inR - hpR) * hpc;
        const float xl = inL - hpL, xr = inR - hpR;

        // Pre-delay + early reflections
        pdL[pdW] = xl;
        pdR[pdW] = xr;
        const float pre = (0.010f + 0.060f * space) * sr;
        float dl = readPD(pdL, pre);
        float dr = readPD(pdR, pre);

        const float tscale = 0.6f * scaleSm * 0.001f * sr;
        float eL = 0.f, eR = 0.f;
        for (int k = 0; k < 7; ++k)
        {
            eL += CATH_ER_G_L[k] * readPD(pdL, pre + CATH_ER_MS_L[k] * tscale);
            eR += CATH_ER_G_R[k] * readPD(pdR, pre + CATH_ER_MS_R[k] * tscale);
        }
        pdW = (pdW + 1) & PDMASK;

        // Input diffusion
        const float ag = 0.25f + 0.5f * diffusion;
        for (int k = 0; k < 4; ++k)
        {
            dl = allpass(apL[k], apLi[k], (int)(CATH_AP_L[k] * srk), ag, dl);
            dr = allpass(apR[k], apRi[k], (int)(CATH_AP_R[k] * srk), ag, dr);
        }

        // Read the 8 delay lines (slowly modulated to avoid metallic ringing)
        const float depth = (1.f + 7.f * space) * srk;
        float x[N];
        for (int i = 0; i < N; ++i)
        {
            float len = std::min((float)CATH_BASE[i] * srk * scaleSm, maxLen) + depth * std::sin(lfoPh[i]);
            len = rack::math::clamp(len, 4.f, (float)(LINE - 4));
            lfoPh[i] += twoPi * (0.11f + 0.037f * (float)i) / sr;
            if (lfoPh[i] > twoPi)
                lfoPh[i] -= twoPi;

            const float pos = (float)w[i] - len;
            int i0 = (int)std::floor(pos);
            const float fr = pos - (float)i0;
            i0 &= LMASK;
            const int i1 = (i0 + 1) & LMASK;
            x[i] = line[i][i0] + fr * (line[i][i1] - line[i][i0]);
        }

        // Damping filter + loop gain
        float y[N];
        for (int i = 0; i < N; ++i)
        {
            lp[i] = (1.f - b) * x[i] + b * lp[i];
            y[i] = lp[i] * g[i];
        }

        // Orthonormal 8x8 Hadamard mix (energy preserving)
        for (int h = 1; h < N; h <<= 1)
        {
            for (int i = 0; i < N; i += (h << 1))
            {
                for (int j = i; j < i + h; ++j)
                {
                    const float a = y[j], c = y[j + h];
                    y[j] = a + c;
                    y[j + h] = a - c;
                }
            }
        }

        // Write back, injecting left into even lines and right into odd lines
        const float k8 = 0.35355339f;
        for (int i = 0; i < N; ++i)
        {
            const float inj = (i & 1) ? dr : dl;
            float v = y[i] * k8 + inj * inGain;
            v = rack::math::clamp(v, -64.f, 64.f); // belt and braces
            line[i][w[i]] = v;
            w[i] = (w[i] + 1) & LMASK;
        }

        fdnL = 0.5f * (x[0] + x[2] + x[4] + x[6]);
        fdnR = 0.5f * (x[1] + x[3] + x[5] + x[7]);
        erL = eL;
        erR = eR;
    }
};

// ------------------------------------------------------------
// HeatGrid: 2D diffusion plate
//
// Same maths as before (5-point Laplacian, reflecting edges, decay), but stored
// with a one-cell ghost border so the inner loop has no per-cell branches. That
// makes it vectorisable (several times faster at 64x64) and it is allocated
// once for the maximum size, so changing SIZE never allocates on the audio
// thread.
// ------------------------------------------------------------
struct HeatGrid
{
    enum
    {
        MAXN = 64,
        MAXP = MAXN + 2
    };
    int n = 48, stride = 50;
    std::vector<float> a, b;

    HeatGrid()
    {
        a.assign(MAXP * MAXP, 0.f);
        b.assign(MAXP * MAXP, 0.f);
        setSize(48);
    }

    void setSize(int newN)
    {
        n = rack::math::clamp(newN, 4, (int)MAXN);
        stride = n + 2;
        clear();
    }

    void clear()
    {
        std::fill(a.begin(), a.end(), 0.f);
        std::fill(b.begin(), b.end(), 0.f);
    }

    float &at(int x, int y) { return a[(y + 1) * stride + (x + 1)]; }
    float get(int x, int y) const { return a[(y + 1) * stride + (x + 1)]; }

    // One diffusion step with decay: next = (u + k * laplacian(u)) * fb.
    void step(float k, float fb)
    {
        const int s = stride;
        float *p = a.data();
        // Reflecting boundary: the ghost cell mirrors the neighbour one cell in.
        for (int y = 0; y < n; ++y)
        {
            float *row = p + (y + 1) * s;
            row[0] = row[2];
            row[n + 1] = row[n - 1];
        }
        std::memcpy(p, p + 2 * s, (size_t)s * sizeof(float));
        std::memcpy(p + (n + 1) * s, p + (n - 1) * s, (size_t)s * sizeof(float));

        float *q = b.data();
        for (int y = 0; y < n; ++y)
        {
            const float *c = p + (y + 1) * s + 1;
            const float *u = c - s;
            const float *d = c + s;
            float *o = q + (y + 1) * s + 1;
            for (int x = 0; x < n; ++x)
                o[x] = (c[x] + k * (c[x - 1] + c[x + 1] + u[x] + d[x] - 4.f * c[x])) * fb;
        }
        a.swap(b);
    }

    // Samples a cell at normalised coordinates (fx, fy).
    float tap(float fx, float fy) const
    {
        const int tx = rack::math::clamp((int)(fx * n), 0, n - 1);
        const int ty = rack::math::clamp((int)(fy * n), 0, n - 1);
        return get(tx, ty);
    }
};

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int VIZ_N = 64; // display field resolution (max grid size)

static const float HR_PANEL_W = RACK_GRID_WIDTH * 12; // 180
static const float HR_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it)
static const float HR_DISP_W = 160.f;
static const float HR_DISP_H = 78.f;
static const float HR_DISP_X = (HR_PANEL_W - HR_DISP_W) / 2.f;
static const float HR_DISP_Y = 26.f;

// Inside the display: square thermal plate on the left, instruments on the right
static const float HR_PLATE_X = 4.f, HR_PLATE_Y = 4.f, HR_PLATE = 70.f;
static const float HR_RX = 80.f, HR_RW = 76.f; // right column

// Every control sits in a plate with its label ABOVE it.
static const float HR_KX3[3] = {30.f, 90.f, 150.f};
static const float HR_KX4[4] = {27.f, 69.f, 111.f, 153.f};
static const float HR_W3 = 50.f, HR_W4 = 38.f;

static const float HR_REV_HEAD_Y = 116.f;
static const float HR_ROW1_TOP = 124.f;
static const float HR_ROW2_TOP = 178.f;
static const float HR_DLY_HEAD_Y = 238.f;
static const float HR_ROW3_TOP = 246.f;
static const float HR_KNOB_H = 50.f; // label +10, knob centre +32
static const float HR_IO_HEAD_Y = 306.f;
static const float HR_IO_TOP = 314.f, HR_IO_H = 44.f; // label +10, jack centre +29

static const float HR_ROW1 = HR_ROW1_TOP + 32.f; // knob centres
static const float HR_ROW2 = HR_ROW2_TOP + 32.f;
static const float HR_ROW3 = HR_ROW3_TOP + 32.f;
static const float HR_PORTY = HR_IO_TOP + 29.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct HeatRev : Module
{
    enum ParamId
    {
        DIFFUSION_PARAM,
        DECAY_PARAM,
        SIZE_PARAM,
        MIX_PARAM,
        DAMPING_PARAM,
        LEVEL_PARAM,
        DELAY_TIME_PARAM,
        DELAY_FEEDBACK_PARAM,
        DELAY_LEVEL_PARAM,
        STEREO_SPREAD_PARAM,
        SPACE_PARAM, // appended last so old patches keep their settings
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_L,
        INPUT_R,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_L,
        OUTPUT_R,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // Heat grid (warmth layer + display)
    HeatGrid grid;
    float sampleRate = 0.f; // 0 = not prepared yet
    SimpleLPF lpL, lpR;
    float dcL = 0.f, dcR = 0.f, dcCoef = 0.f;

    StereoDelay delay;
    Cathedral cathedral;

    // Wet-path fade-in after load / reset / sample-rate change / watchdog flush
    float fade = 0.f, fadeStep = 0.001f;
    int flushCount = 0;

    // Cached decay-derived values
    float lastDecayP = -1.f, rt60 = 7.6f, gridFb = 0.9998f, tailComp = 1.f;

    // Snapshot for the display (written by the audio thread, read by the UI
    // thread; a torn read is only cosmetic)
    float viz[VIZ_N][VIZ_N] = {};
    int vizN = 48;
    float vizNorm = 1.f;
    int vizCounter = 0;
    float vizEnergy = 0.f, vizPkL = 0.f, vizPkR = 0.f, vizRt60 = 7.6f;
    float pkL = 0.f, pkR = 0.f, pkDecay = 0.9999f;

    // Constructor: configures all parameters, inputs, outputs and bypass and
    // allocates every buffer once.
    HeatRev()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(DIFFUSION_PARAM, 0.f, 1.f, 0.7f, "Diffusion", "%", 0.f, 100.f);
        configParam(DECAY_PARAM, 0.f, 1.f, 0.8f, "Decay (RT60 0.5 - 15 s)", "%", 0.f, 100.f);
        configParam(SIZE_PARAM, 4.f, 64.f, 48.f, "Size", " cells");
        configParam(MIX_PARAM, 0.f, 1.f, 0.7f, "Mix", "%", 0.f, 100.f);
        configParam(DAMPING_PARAM, 0.f, 1.f, 0.55f, "Damping (higher = darker)", "%", 0.f, 100.f);
        configParam(LEVEL_PARAM, 0.f, 1.f, 0.8f, "Level", "%", 0.f, 100.f);
        configParam(DELAY_TIME_PARAM, 10.f, 500.f, 80.f, "Delay time", " ms");
        configParam(DELAY_FEEDBACK_PARAM, 0.f, 0.95f, 0.4f, "Delay feedback", "%", 0.f, 100.f);
        configParam(DELAY_LEVEL_PARAM, 0.f, 1.f, 0.7f, "Delay level", "%", 0.f, 100.f);
        configParam(STEREO_SPREAD_PARAM, 0.f, 50.f, 15.f, "Stereo spread", " ms");
        configParam(SPACE_PARAM, 0.f, 1.f, 0.7f, "Space (pre-delay, reflections, shimmer)", "%", 0.f, 100.f);
        getParamQuantity(SIZE_PARAM)->snapEnabled = true;

        configInput(INPUT_L, "Left");
        configInput(INPUT_R, "Right");
        configOutput(OUTPUT_L, "Left");
        configOutput(OUTPUT_R, "Right");
        configBypass(INPUT_L, OUTPUT_L);
        configBypass(INPUT_R, OUTPUT_R);

        cathedral.init();
        delay.init(48000.f); // real rate is applied on the first process()
    }

    // (Re)initialises everything that depends on the sample rate and silences
    // all state. Runs on the first process() call and whenever the rate changes.
    void prepare(float sr)
    {
        sampleRate = sr;
        delay.init(sr);
        cathedral.reset();
        grid.clear();
        lpL.setHz(4000.f, sr);
        lpR.setHz(4000.f, sr);
        lpL.y = lpR.y = 0.f;
        dcCoef = 1.0f - std::exp(-2.0f * float(M_PI) * 40.0f / sr);
        dcL = dcR = 0.f;
        fadeStep = 1.f / (0.08f * sr); // 80 ms wet fade-in
        fade = 0.f;
        pkDecay = std::exp(-1.f / (0.25f * sr));
        pkL = pkR = 0.f;
        vizCounter = 0;
        lastDecayP = -1.f;
    }

    // Silences all state (echo, reverb, grid, filters) and restarts the fade-in.
    void flushAll()
    {
        cathedral.reset();
        grid.clear();
        delay.clear();
        lpL.y = lpR.y = 0.f;
        dcL = dcR = 0.f;
        pkL = pkR = 0.f;
        fade = 0.f;
    }

    // Rack reset hook (Initialize).
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        if (sampleRate > 0.f)
            flushAll();
    }

    // Publishes the grid to the display: full-resolution field, auto-scaled
    // with a slow release so tails visibly fade, plus a few readouts.
    void publishViz()
    {
        const int n = grid.n;
        float peak = 0.f, ss = 0.f;
        for (int y = 0; y < n; ++y)
        {
            for (int x = 0; x < n; ++x)
            {
                const float v = grid.get(x, y);
                peak = std::max(peak, std::fabs(v));
                ss += v * v;
            }
        }
        vizEnergy = std::sqrt(ss / (float)(n * n));
        // fast attack, ~3 s release: the map stays readable but a decaying tail visibly dims
        vizNorm += (std::max(peak, 0.02f) - vizNorm) * (peak > vizNorm ? 0.6f : 0.004f);
        const float k = 1.f / vizNorm;
        for (int y = 0; y < n; ++y)
            for (int x = 0; x < n; ++x)
                viz[y][x] = grid.get(x, y) * k;
        vizN = n;
        vizPkL = pkL;
        vizPkR = pkR;
        vizRt60 = rt60;
    }

    // Main audio callback.
    void process(const ProcessArgs &args) override
    {
        if (args.sampleRate != sampleRate)
            prepare(args.sampleRate);

        const float diffusion = params[DIFFUSION_PARAM].getValue();
        const float decayP = params[DECAY_PARAM].getValue();
        const float mix = params[MIX_PARAM].getValue();
        const float damping = params[DAMPING_PARAM].getValue();
        const float level = params[LEVEL_PARAM].getValue();
        const float space = params[SPACE_PARAM].getValue();
        const float sizeP = params[SIZE_PARAM].getValue();

        // Decay-derived values (cached: pow() per sample is wasted work)
        if (decayP != lastDecayP)
        {
            lastDecayP = decayP;
            rt60 = 0.5f * std::pow(30.0f, decayP); // 0.5 s .. 15 s (7.6 s default)
            gridFb = 1.0f - 0.01f * std::pow(1.0f - decayP, 2.5f);
            gridFb = rack::math::clamp(gridFb, 0.99f, 0.99995f);
            tailComp = std::min(1.0f, std::sqrt((1.0f - gridFb) / 6e-4f));
        }
        const float scaleTarget = 0.6f + 1.4f * (sizeP - 4.0f) / 60.0f; // room scale 0.6 .. 2.0
        const float diffK = rack::math::clamp(diffusion, 0.f, 1.f) * 0.25f;

        const int newSize = rack::math::clamp((int)std::round(sizeP), 4, 64);
        if (newSize != grid.n)
            grid.setSize(newSize);

        // Delay / Echo
        const float delayTimeMs = params[DELAY_TIME_PARAM].getValue();
        const float spreadMs = params[STEREO_SPREAD_PARAM].getValue();
        const float delayTimeL = rack::math::clamp(delayTimeMs - spreadMs * 0.5f, 5.0f, 1000.0f);
        const float delayTimeR = rack::math::clamp(delayTimeMs + spreadMs * 0.5f, 5.0f, 1000.0f);
        delay.setDelayMs(delayTimeL, delayTimeR);
        delay.setFeedback(params[DELAY_FEEDBACK_PARAM].getValue());

        // Input guard: NaN/inf from an upstream module (typical at patch load)
        // never reaches a feedback path.
        const float inL_raw = hrSanitize(inputs[INPUT_L].getVoltage(), HR_IN_LIM);
        const float inR_raw = hrSanitize(inputs[INPUT_R].getVoltage(), HR_IN_LIM);
        const std::pair<float, float> delayed = delay.process(inL_raw, inR_raw);
        const float delayLevel = params[DELAY_LEVEL_PARAM].getValue();

        // The reverb hears the dry signal plus the echoes
        const float reverbInputL = inL_raw + delayed.first * delayLevel;
        const float reverbInputR = inR_raw + delayed.second * delayLevel;

        // ---- Cathedral ----
        float fdnL, fdnR, erL, erR;
        cathedral.process(reverbInputL, reverbInputR, sampleRate, rt60, damping,
                          scaleTarget, diffusion, space, fdnL, fdnR, erL, erR);

        // ---- Heat grid: gentle excitation, bounded output ----
        const float injectionGain = 0.3f * tailComp;
        const int n = grid.n;
        for (int y = 0; y < n; ++y)
            grid.at(0, y) += reverbInputL * injectionGain;
        for (int x = 0; x < n; ++x)
            grid.at(x, n - 1) += reverbInputR * injectionGain;
        grid.step(diffK, gridFb);

        const float rawL = (grid.tap(0.28f, 0.62f) + grid.tap(0.50f, 0.50f) + grid.tap(0.72f, 0.35f)) * (1.0f / 3.0f);
        const float rawR = (grid.tap(0.62f, 0.28f) + grid.tap(0.50f, 0.50f) + grid.tap(0.35f, 0.72f)) * (1.0f / 3.0f);
        dcL += (rawL - dcL) * dcCoef;
        dcR += (rawR - dcR) * dcCoef;
        float heatL = lpL.process(rawL - dcL);
        float heatR = lpR.process(rawR - dcR);
        heatL = 3.0f * std::tanh(heatL * 0.3f / 3.0f); // hard-bounded to +/-3 V
        heatR = 3.0f * std::tanh(heatR * 0.3f / 3.0f);

        // ---- Watchdog: anything runaway or non-finite flushes EVERYTHING ----
        const float chk = std::fabs(fdnL) + std::fabs(fdnR) + std::fabs(erL) + std::fabs(erR) +
                          std::fabs(delayed.first) + std::fabs(delayed.second) +
                          std::fabs(heatL) + std::fabs(heatR) + 0.f * (rawL + rawR);
        const bool runaway = !(chk < HR_RUNAWAY) || !std::isfinite(rawL + rawR);

        float wetL = 0.f, wetR = 0.f;
        if (runaway)
        {
            flushAll();
            ++flushCount;
        }
        else
        {
            // ---- Wet mix ----
            const float erGain = 0.35f * space;
            wetL = (fdnL + erL * erGain + 0.35f * heatL) * level;
            wetR = (fdnR + erR * erGain + 0.35f * heatR) * level;

            // Smooth limiter on the wet path: loud input can never blow up the output
            wetL = 8.0f * std::tanh(wetL / 8.0f);
            wetR = 8.0f * std::tanh(wetR / 8.0f);

            if (!std::isfinite(wetL) || !std::isfinite(wetR))
            {
                flushAll();
                ++flushCount;
                wetL = wetR = 0.f;
            }

            // Fade the wet path in after (re)start so any start-up transient is masked
            if (fade < 1.f)
                fade = std::min(1.f, fade + fadeStep);
            wetL *= fade;
            wetR *= fade;

            pkL = std::max(std::fabs(heatL), pkL * pkDecay);
            pkR = std::max(std::fabs(heatR), pkR * pkDecay);
        }

        float outL = (1.0f - mix) * inL_raw + mix * wetL;
        float outR = (1.0f - mix) * inR_raw + mix * wetR;

        // Soft-clips the output above +/-9 V with a gentle 0.3 slope.
        auto softClip = [](float x)
        {
            const float limit = 9.0f;
            if (x > limit)
                return limit + (x - limit) * 0.3f;
            if (x < -limit)
                return -limit + (x + limit) * 0.3f;
            return x;
        };
        outL = rack::math::clamp(softClip(outL), -10.0f, 10.0f);
        outR = rack::math::clamp(softClip(outR), -10.0f, 10.0f);

        outputs[OUTPUT_L].setVoltage(outL);
        outputs[OUTPUT_R].setVoltage(outR);

        // Refresh the display snapshot ~90 times per second at 48 kHz
        if (++vizCounter >= 512)
        {
            vizCounter = 0;
            publishViz();
        }
    }
};

// ------------------------------------------------------------
// UI (anonymous namespace: names cannot clash with other module files)
// ------------------------------------------------------------
namespace
{

    static const int HR_SPARK_N = 76;

    // Maps a level to 0..1 on a dB scale: 0 at -rangeDb below `top`, 1 at `top`.
    static inline float hrMeter(float v, float top, float rangeDb)
    {
        const float db = 20.f * std::log10(std::max(v / top, 1e-6f));
        return rack::math::clamp((db + rangeDb) / rangeDb, 0.f, 1.f);
    }

    // Diverging thermal colour map. u is the signed, normalised field value
    // (about +/-1 at the display peak). Positive: black -> deep teal -> cyan ->
    // white-hot. Negative: black -> indigo -> violet -> pale lilac.
    static inline void hrHeatColor(float u, unsigned char *o)
    {
        static const float POS[5][3] = {{7, 11, 19}, {6, 62, 90}, {24, 150, 196}, {61, 224, 255}, {228, 251, 255}};
        static const float NEG[5][3] = {{7, 11, 19}, {52, 18, 98}, {110, 56, 205}, {160, 110, 255}, {238, 226, 255}};
        static const float STOP[5] = {0.f, 0.22f, 0.5f, 0.78f, 1.f};

        if (!(u == u))
            u = 0.f;
        // Gamma 0.5 lifts the faint interior of the plate (heat that has diffused
        // away from the injection edges is far weaker than the edges themselves)
        // so structure stays visible instead of the map looking empty.
        float m = std::sqrt(std::fabs(u));
        m = m > 1.f ? 1.f : m;
        const float(*C)[3] = (u >= 0.f) ? POS : NEG;
        int s = 0;
        while (s < 3 && m > STOP[s + 1])
            ++s;
        const float t = (m - STOP[s]) / (STOP[s + 1] - STOP[s]);
        for (int c = 0; c < 3; ++c)
            o[c] = (unsigned char)(C[s][c] + (C[s + 1][c] - C[s][c]) * t + 0.5f);
        o[3] = 255;
    }

    // Static artwork: base panel, display bezel, the REVERB section (three
    // knobs in row 1, four in row 2), the DELAY section, and the I/O row with
    // tinted output jacks.
    struct HRPanelBackground : TransparentWidget
    {
        HRPanelBackground() { box.size = Vec(HR_PANEL_W, HR_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, HR_PANEL_W, HR_PANEL_H, "HEATREV");
            agmm::bezel(vg, HR_DISP_X, HR_DISP_Y, HR_DISP_W, HR_DISP_H);

            // ---- reverb: row 1 has three knobs, row 2 four ----
            agmm::sectionHeader(vg, HR_PANEL_W, HR_REV_HEAD_Y, "REVERB", 10.f);
            const char *rev1[3] = {"DIFFUSION", "DECAY", "SIZE"};
            const char *rev2[4] = {"MIX", "DAMPING", "LEVEL", "SPACE"};
            for (int i = 0; i < 3; ++i)
                agmm::cell(vg, HR_KX3[i], HR_ROW1_TOP, HR_W3, HR_KNOB_H, rev1[i], false, 7.f);
            for (int i = 0; i < 4; ++i)
                agmm::cell(vg, HR_KX4[i], HR_ROW2_TOP, HR_W4, HR_KNOB_H, rev2[i], false, 7.f);

            // ---- delay ----
            agmm::sectionHeader(vg, HR_PANEL_W, HR_DLY_HEAD_Y, "DELAY", 10.f);
            const char *dly[4] = {"TIME", "FDBK", "LEVEL", "SPREAD"};
            for (int i = 0; i < 4; ++i)
                agmm::cell(vg, HR_KX4[i], HR_ROW3_TOP, HR_W4, HR_KNOB_H, dly[i], false, 7.f);

            // ---- I/O: inputs plain, outputs tinted ----
            agmm::sectionHeader(vg, HR_PANEL_W, HR_IO_HEAD_Y, "I / O", 10.f);
            const char *io[4] = {"IN L", "IN R", "OUT L", "OUT R"};
            for (int i = 0; i < 4; ++i)
                agmm::cell(vg, HR_KX4[i], HR_IO_TOP, HR_W4, HR_IO_H, io[i], i >= 2, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Thermal display
    //
    //  +-----------------+  THERMAL FIELD
    //  |                 |  GRID            48x48
    //  |   heat plate    |  RT60             7.6 s
    //  |   (true square, |  L  ======----
    //  |   bilinear GPU  |  R  ====------
    //  |   texture)      |  - [violet|cyan] +
    //  +-----------------+  [ energy history ]
    //
    // The plate is a 64x64 RGBA texture, filtered by the GPU, so it is smooth at
    // any zoom level. The UI eases the field toward the latest snapshot every
    // frame, so motion looks fluid instead of stepping at the 90 Hz data rate.
    // ------------------------------------------------------------
    struct HRDisplayWidget : TransparentWidget
    {
        HeatRev *module = nullptr;

        float disp[VIZ_N][VIZ_N] = {};
        unsigned char pix[VIZ_N * VIZ_N * 4] = {};
        int img = 0;
        NVGcontext *imgVg = nullptr;

        float meterL = 0.f, meterR = 0.f;
        float spark[HR_SPARK_N] = {};
        int sparkPos = 0, sparkTick = 0;
        int gridN = 48;
        float rt60 = 7.6f;
        float demoT = 0.f;

        HRDisplayWidget(HeatRev *m) : module(m)
        {
            box.pos = Vec(HR_DISP_X, HR_DISP_Y);
            box.size = Vec(HR_DISP_W, HR_DISP_H);
            for (int i = 0; i < VIZ_N * VIZ_N; ++i)
                hrHeatColor(0.f, &pix[i * 4]);
        }

        ~HRDisplayWidget()
        {
            if (img > 0 && imgVg && APP && APP->window && APP->window->vg == imgVg)
                nvgDeleteImage(imgVg, img);
        }

        // Per-frame (UI thread): ease the displayed field toward the newest
        // snapshot, resampling the n x n grid to the fixed 64x64 texture with
        // bilinear interpolation, then colour-map it.
        void step() override
        {
            TransparentWidget::step();
            const int N = VIZ_N;

            if (module)
            {
                const int n = rack::math::clamp(module->vizN, 4, N);
                const float sc = (float)n / (float)N;
                for (int y = 0; y < N; ++y)
                {
                    float fy = rack::math::clamp((y + 0.5f) * sc - 0.5f, 0.f, (float)(n - 1));
                    const int y0 = (int)fy;
                    const int y1 = std::min(y0 + 1, n - 1);
                    const float ty = fy - (float)y0;
                    for (int x = 0; x < N; ++x)
                    {
                        float fx = rack::math::clamp((x + 0.5f) * sc - 0.5f, 0.f, (float)(n - 1));
                        const int x0 = (int)fx;
                        const int x1 = std::min(x0 + 1, n - 1);
                        const float tx = fx - (float)x0;
                        const float a = module->viz[y0][x0] + (module->viz[y0][x1] - module->viz[y0][x0]) * tx;
                        const float b = module->viz[y1][x0] + (module->viz[y1][x1] - module->viz[y1][x0]) * tx;
                        float v = a + (b - a) * ty;
                        if (!std::isfinite(v))
                            v = 0.f;
                        disp[y][x] += (v - disp[y][x]) * 0.45f;
                    }
                }
                meterL += (hrMeter(module->vizPkL, 3.f, 48.f) - meterL) * 0.3f;
                meterR += (hrMeter(module->vizPkR, 3.f, 48.f) - meterR) * 0.3f;
                gridN = n;
                rt60 = module->vizRt60;
                if (++sparkTick >= 6) // ~10 Hz -> 7.6 s of history
                {
                    sparkTick = 0;
                    spark[sparkPos] = rack::math::clamp((20.f * std::log10(module->vizEnergy + 1e-6f) + 70.f) / 90.f, 0.f, 1.f);
                    sparkPos = (sparkPos + 1) % HR_SPARK_N;
                }
            }
            else
            {
                // Module browser preview: a static, good-looking demo field
                for (int y = 0; y < N; ++y)
                    for (int x = 0; x < N; ++x)
                    {
                        const float fx = x / 63.f, fy = y / 63.f;
                        disp[y][x] = 0.95f * std::sin(fx * 13.f) * std::cos(fy * 9.f) * std::exp(-1.6f * fx);
                    }
                meterL = 0.62f;
                meterR = 0.45f;
                gridN = 48;
                rt60 = 7.6f;
                for (int i = 0; i < HR_SPARK_N; ++i)
                    spark[i] = 0.25f + 0.6f * std::exp(-0.05f * i) * (0.7f + 0.3f * std::sin(i * 0.7f));
                sparkPos = 0;
            }

            for (int y = 0; y < N; ++y)
                for (int x = 0; x < N; ++x)
                    hrHeatColor(disp[y][x], &pix[(y * N + x) * 4]);
        }

        // Static layer: screen, plate recess, labels, legend and frames.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, HR_DISP_W, HR_DISP_H);

            // Plate recess + frame
            nvgBeginPath(vg);
            nvgRoundedRect(vg, HR_PLATE_X - 1.f, HR_PLATE_Y - 1.f, HR_PLATE + 2.f, HR_PLATE + 2.f, 3.f);
            nvgFillColor(vg, nvgRGBA(4, 7, 12, 255));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 38));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            const NVGcolor dim = nvgRGBA(138, 160, 184, 210);
            agmm::text(vg, HR_RX, 8.f, "THERMAL FIELD", 6.5f, dim, agmm::ALIGN_L, 0.8f);

            nvgBeginPath(vg);
            nvgMoveTo(vg, HR_RX, 14.5f);
            nvgLineTo(vg, HR_RX + HR_RW, 14.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            agmm::text(vg, HR_RX, 22.f, "GRID", 6.5f, dim, agmm::ALIGN_L, 0.6f);
            agmm::text(vg, HR_RX, 32.f, "RT60", 6.5f, dim, agmm::ALIGN_L, 0.6f);
            agmm::text(vg, HR_RX, 42.f, "L", 6.5f, agmm::accent(), agmm::ALIGN_L, 0.6f);
            agmm::text(vg, HR_RX, 50.f, "R", 6.5f, agmm::violet(), agmm::ALIGN_L, 0.6f);
            agmm::text(vg, HR_RX, 58.f, "-", 7.f, agmm::violet(), agmm::ALIGN_L, 0.f);
            agmm::text(vg, HR_RX + HR_RW, 58.f, "+", 7.f, agmm::accent(), agmm::ALIGN_R, 0.f);

            // Meter tracks
            for (int i = 0; i < 2; ++i)
            {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, HR_RX + 10.f, (i ? 48.f : 40.f), HR_RW - 10.f, 4.f, 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 16));
                nvgFill(vg);
            }

            // Legend bar track and history frame
            nvgBeginPath(vg);
            nvgRoundedRect(vg, HR_RX + 8.f, 56.f, HR_RW - 16.f, 4.f, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 10));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgRoundedRect(vg, HR_RX + 0.5f, 63.5f, HR_RW - 1.f, 11.f, 2.f);
            nvgFillColor(vg, nvgRGBA(4, 7, 12, 255));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Self-lit layer: thermal texture, overlays, meters, values, history.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float PX = HR_PLATE_X, PY = HR_PLATE_Y, PS = HR_PLATE;

            // ---- texture upload (re-create if the GL context changed) ----
            if (img <= 0 || imgVg != vg)
            {
                img = nvgCreateImageRGBA(vg, VIZ_N, VIZ_N, 0, pix);
                imgVg = vg;
            }
            else
            {
                nvgUpdateImage(vg, img, pix);
            }

            if (img > 0)
            {
                NVGpaint p = nvgImagePattern(vg, PX, PY, PS, PS, 0.f, img, 1.f);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, PX, PY, PS, PS, 2.f);
                nvgFillPaint(vg, p);
                nvgFill(vg);
            }

            // ---- plate overlays ----
            // faint 8x8 reference grid
            nvgBeginPath(vg);
            for (int i = 1; i < 8; ++i)
            {
                const float o = PS * i / 8.f;
                nvgMoveTo(vg, PX + o, PY);
                nvgLineTo(vg, PX + o, PY + PS);
                nvgMoveTo(vg, PX, PY + o);
                nvgLineTo(vg, PX + PS, PY + o);
            }
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 10));
            nvgStrokeWidth(vg, 0.5f);
            nvgStroke(vg);

            // vignette
            {
                NVGpaint v = nvgRadialGradient(vg, PX + PS * 0.5f, PY + PS * 0.5f, PS * 0.35f, PS * 0.78f,
                                               nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 120));
                nvgBeginPath(vg);
                nvgRoundedRect(vg, PX, PY, PS, PS, 2.f);
                nvgFillPaint(vg, v);
                nvgFill(vg);
            }
            // glass highlight
            {
                NVGpaint gl = nvgLinearGradient(vg, 0.f, PY, 0.f, PY + PS * 0.45f,
                                                nvgRGBA(255, 255, 255, 24), nvgRGBA(255, 255, 255, 0));
                nvgBeginPath(vg);
                nvgRoundedRect(vg, PX, PY, PS, PS * 0.45f, 2.f);
                nvgFillPaint(vg, gl);
                nvgFill(vg);
            }

            // injection edges: left channel enters on the left edge, right channel on the bottom edge
            nvgBeginPath(vg);
            nvgRect(vg, PX, PY + 1.f, 3.f, PS - 2.f);
            nvgFillColor(vg, agmm::withA(agmm::accent(), 0.22f));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, PX, PY + 1.f, 1.5f, PS - 2.f);
            nvgFillColor(vg, agmm::withA(agmm::accent(), 0.95f));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgRect(vg, PX + 1.f, PY + PS - 3.f, PS - 2.f, 3.f);
            nvgFillColor(vg, agmm::withA(agmm::violet(), 0.22f));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, PX + 1.f, PY + PS - 1.5f, PS - 2.f, 1.5f);
            nvgFillColor(vg, agmm::withA(agmm::violet(), 0.95f));
            nvgFill(vg);

            // frame on top
            nvgBeginPath(vg);
            nvgRoundedRect(vg, PX + 0.25f, PY + 0.25f, PS - 0.5f, PS - 0.5f, 2.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 46));
            nvgStrokeWidth(vg, 0.8f);
            nvgStroke(vg);

            // ---- right column: values ----
            char buf[32];
            snprintf(buf, sizeof(buf), "%dx%d", gridN, gridN);
            agmm::text(vg, HR_RX + HR_RW, 22.f, buf, 7.5f, agmm::accent(), agmm::ALIGN_R, 0.6f);
            if (rt60 < 10.f)
                snprintf(buf, sizeof(buf), "%.1f s", rt60);
            else
                snprintf(buf, sizeof(buf), "%.0f s", rt60);
            agmm::text(vg, HR_RX + HR_RW, 32.f, buf, 7.5f, nvgRGBA(228, 238, 248, 255), agmm::ALIGN_R, 0.6f);

            // level meters (heat-layer output, dB scale)
            const float mw = HR_RW - 10.f;
            for (int i = 0; i < 2; ++i)
            {
                const float lvl = i ? meterR : meterL;
                const float y = i ? 48.f : 40.f;
                const NVGcolor c = i ? agmm::violet() : agmm::accent();
                if (lvl > 0.01f)
                {
                    NVGpaint mp = nvgLinearGradient(vg, HR_RX + 10.f, 0.f, HR_RX + 10.f + mw, 0.f,
                                                    agmm::withA(c, 0.35f), agmm::withA(c, 1.f));
                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, HR_RX + 10.f, y, std::max(2.f, mw * lvl), 4.f, 2.f);
                    nvgFillPaint(vg, mp);
                    nvgFill(vg);
                }
            }

            // sign legend: violet -> dark -> cyan
            {
                const float lx = HR_RX + 8.f, lw = HR_RW - 16.f;
                NVGpaint a = nvgLinearGradient(vg, lx, 0.f, lx + lw * 0.5f, 0.f,
                                               agmm::withA(agmm::violet(), 0.95f), nvgRGBA(7, 11, 19, 255));
                nvgBeginPath(vg);
                nvgRoundedRect(vg, lx, 56.f, lw * 0.5f + 0.5f, 4.f, 2.f);
                nvgFillPaint(vg, a);
                nvgFill(vg);
                NVGpaint b = nvgLinearGradient(vg, lx + lw * 0.5f, 0.f, lx + lw, 0.f,
                                               nvgRGBA(7, 11, 19, 255), agmm::withA(agmm::accent(), 0.95f));
                nvgBeginPath(vg);
                nvgRoundedRect(vg, lx + lw * 0.5f, 56.f, lw * 0.5f, 4.f, 2.f);
                nvgFillPaint(vg, b);
                nvgFill(vg);
            }

            // energy history (oldest -> newest), dB scale
            {
                const float x0 = HR_RX + 1.5f, w = HR_RW - 3.f;
                const float yb = 73.f, hh = 8.f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x0, yb);
                for (int i = 0; i < HR_SPARK_N; ++i)
                {
                    const float v = spark[(sparkPos + i) % HR_SPARK_N];
                    nvgLineTo(vg, x0 + w * i / (float)(HR_SPARK_N - 1), yb - hh * v);
                }
                nvgLineTo(vg, x0 + w, yb);
                nvgClosePath(vg);
                NVGpaint fp = nvgLinearGradient(vg, 0.f, yb - hh, 0.f, yb,
                                                agmm::withA(agmm::accent(), 0.45f), agmm::withA(agmm::accent(), 0.02f));
                nvgFillPaint(vg, fp);
                nvgFill(vg);

                nvgBeginPath(vg);
                for (int i = 0; i < HR_SPARK_N; ++i)
                {
                    const float v = spark[(sparkPos + i) % HR_SPARK_N];
                    const float px = x0 + w * i / (float)(HR_SPARK_N - 1);
                    const float py = yb - hh * v;
                    if (i == 0)
                        nvgMoveTo(vg, px, py);
                    else
                        nvgLineTo(vg, px, py);
                }
                nvgStrokeColor(vg, agmm::withA(agmm::accent(), 0.95f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget: adds the static panel, screws, the thermal display, the
// reverb knobs, the delay knobs and the two in / two out jacks.
// ------------------------------------------------------------
struct HeatRevWidget : ModuleWidget
{
    HeatRevWidget(HeatRev *module)
    {
        setModule(module);
        box.size = Vec(HR_PANEL_W, HR_PANEL_H);

        addChild(new HRPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new HRDisplayWidget(module));

        // Reverb row 1
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX3[0], HR_ROW1), module, HeatRev::DIFFUSION_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX3[1], HR_ROW1), module, HeatRev::DECAY_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX3[2], HR_ROW1), module, HeatRev::SIZE_PARAM));

        // Reverb row 2 (four knobs)
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX4[0], HR_ROW2), module, HeatRev::MIX_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX4[1], HR_ROW2), module, HeatRev::DAMPING_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX4[2], HR_ROW2), module, HeatRev::LEVEL_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX4[3], HR_ROW2), module, HeatRev::SPACE_PARAM));

        // Delay
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX4[0], HR_ROW3), module, HeatRev::DELAY_TIME_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX4[1], HR_ROW3), module, HeatRev::DELAY_FEEDBACK_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX4[2], HR_ROW3), module, HeatRev::DELAY_LEVEL_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(HR_KX4[3], HR_ROW3), module, HeatRev::STEREO_SPREAD_PARAM));

        // I/O
        addInput(createInputCentered<DarkPJ301MPort>(Vec(HR_KX4[0], HR_PORTY), module, HeatRev::INPUT_L));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(HR_KX4[1], HR_PORTY), module, HeatRev::INPUT_R));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(HR_KX4[2], HR_PORTY), module, HeatRev::OUTPUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(HR_KX4[3], HR_PORTY), module, HeatRev::OUTPUT_R));
    }
};

Model *modelHeatRev = createModel<HeatRev, HeatRevWidget>("HeatRev");