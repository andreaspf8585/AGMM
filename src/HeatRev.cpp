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
// The module exposes reverb controls (Diffusion, Decay, Size, Mix, Damping,
// Level, Space) and delay controls (Time, Feedback, Level, Spread), and shows
// a live heat-map of the grid plus a readout row with the grid size and the
// cyan/violet sign legend.
//
// File structure:
//   1. DSP building blocks: SimpleLPF, StereoDelay, Cathedral.
//   2. Layout constants (12 HP = 180x380 px).
//   3. HeatRev module: state, config, display snapshot and process().
//   4. Anonymous-namespace UI: HRPanelBackground and HRDisplayWidget.
//   5. HeatRevWidget: places knobs, jacks and the display.
//   6. Model registration.
// -----------------------------------------------------------------------------

// ------------------------------------------------------------
// DSP
// ------------------------------------------------------------

// One-pole low-pass
// Simple one-pole low-pass filter used to tame the heat-grid taps.
struct SimpleLPF
{
    float y = 0.0f;
    float a = 0.0f;

    // Sets the filter coefficient from a normalised cutoff (0..1, where 1 is
    // Nyquist). Values at or below 0 mute the filter; at or above 1 pass the
    // input through unchanged.
    void setCutoff(float normCutoff)
    {
        if (normCutoff <= 0.0f)
            a = 0.0f;
        else if (normCutoff >= 1.0f)
            a = 1.0f;
        else
            a = 1.0f - std::exp(-2.0f * float(M_PI) * normCutoff);
    }

    // Processes one sample through the one-pole low-pass and returns the
    // filtered value.
    float process(float x)
    {
        y = y + a * (x - y);
        return y;
    }
};

// Stereo delay with feedback and fractional delay
// Stereo delay line with independent L/R delay times, feedback and
// linear-interpolated fractional reads.
struct StereoDelay
{
    std::vector<float> bufferL, bufferR;
    int writePos = 0;
    int bufferSize = 0;
    float sampleRate = 48000.0f;
    float delayTimeL = 0.0f;
    float delayTimeR = 0.0f;
    float feedback = 0.0f;

    // Stores the current sample rate used to convert milliseconds to samples.
    void setSampleRate(float sr) { sampleRate = sr; }

    // Allocates (or reallocates) both delay buffers to hold maxDelaySamples+2
    // samples and resets the write position. No-op if the size is unchanged.
    void resize(int maxDelaySamples)
    {
        if (maxDelaySamples != bufferSize)
        {
            bufferSize = maxDelaySamples + 2;
            bufferL.assign(bufferSize, 0.0f);
            bufferR.assign(bufferSize, 0.0f);
            writePos = 0;
        }
    }

    // Sets the L/R delay times in milliseconds and grows the buffers if the
    // requested delay is longer than the current allocation.
    void setDelayMs(float msL, float msR)
    {
        delayTimeL = msL * 0.001f * sampleRate;
        delayTimeR = msR * 0.001f * sampleRate;
        int maxSamples = static_cast<int>(std::max(delayTimeL, delayTimeR)) + 2;
        if (maxSamples > bufferSize)
            resize(maxSamples);
    }

    // Sets the feedback amount, clamped to 0..0.95 for stability.
    void setFeedback(float fb) { feedback = rack::math::clamp(fb, 0.0f, 0.95f); }

    // Processes one stereo sample: reads the delayed L/R values, writes the
    // input plus feedback into the buffers, and returns the delayed pair.
    std::pair<float, float> process(float inL, float inR)
    {
        float readL = readBuffer(bufferL, delayTimeL);
        float readR = readBuffer(bufferR, delayTimeR);

        float writeL = inL + feedback * readL;
        float writeR = inR + feedback * readR;

        bufferL[writePos] = writeL;
        bufferR[writePos] = writeR;
        writePos = (writePos + 1) % bufferSize;

        return {readL, readR};
    }

private:
    // Reads one buffer at `delaySamples` behind the write head using linear
    // interpolation. Returns 0 for non-positive delays.
    float readBuffer(const std::vector<float> &buf, float delaySamples)
    {
        if (delaySamples <= 0.0f)
            return 0.0f;
        float readPos = static_cast<float>(writePos) - delaySamples;
        if (readPos < 0.0f)
            readPos += bufferSize;
        int idx1 = static_cast<int>(readPos);
        int idx2 = (idx1 + 1) % bufferSize;
        float frac = readPos - idx1;
        return buf[idx1] + frac * (buf[idx2] - buf[idx1]);
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
    // `gain`, returning the all-pass output.
    static float allpass(std::vector<float> &buf, int &idx, int len, float gain, float x)
    {
        if (len < 2)
            len = 2;
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
        const float pos = (float)pdW - d;
        int i0 = (int)std::floor(pos);
        const float fr = pos - (float)i0;
        i0 &= PDMASK;
        const int i1 = (i0 + 1) & PDMASK;
        return buf[i0] + fr * (buf[i1] - buf[i0]);
    }

    // fdn = diffuse tail, er = early reflections
    // Runs the full Cathedral reverb for one stereo sample. `rt60`, `damping`,
    // `scaleTarget`, `diffusion` and `space` control the tail; outputs are the
    // FDN tail (fdnL/fdnR) and the early reflections (erL/erR).
    void process(float inL, float inR, float sr, float rt60, float damping,
                 float scaleTarget, float diffusion, float space,
                 float &fdnL, float &fdnR, float &erL, float &erR)
    {
        const float srk = sr / 48000.f;
        const float twoPi = 6.2831853f;

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
                const float len = (float)CATH_BASE[i] * srk * scaleSm;
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
            const float len = (float)CATH_BASE[i] * srk * scaleSm + depth * std::sin(lfoPh[i]);
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
// Layout constants
// ------------------------------------------------------------
static const int VIZ_COLS = 40;
static const int VIZ_ROWS = 16;
static const float VIZ_CELL = 4.f;

static const float HR_PANEL_W = RACK_GRID_WIDTH * 12; // 180
static const float HR_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it): heat map on top, readout row below it
static const float HR_VIZ_W = VIZ_COLS * VIZ_CELL; // 160
static const float HR_VIZ_H = VIZ_ROWS * VIZ_CELL; // 64
static const float HR_DISP_W = HR_VIZ_W;
static const float HR_DISP_H = HR_VIZ_H + 12.f; // 76
static const float HR_DISP_X = (HR_PANEL_W - HR_DISP_W) / 2.f;
static const float HR_DISP_Y = 26.f;
static const float HR_FOOT_Y = HR_VIZ_H + 6.f; // readout row (inside display, under the map)

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
    std::vector<float> state, next;
    int size;
    float sampleRate;
    SimpleLPF lpL, lpR;
    float dcL = 0.f, dcR = 0.f;

    StereoDelay delay;
    Cathedral cathedral;

    // Downsampled snapshot of the grid for the display (written by the audio thread,
    // read by the UI thread; a torn read is only cosmetic)
    float viz[VIZ_ROWS][VIZ_COLS] = {};
    float vizNorm = 1.f;
    int vizCounter = 0;

    // Constructor: configures all parameters, inputs, outputs and bypass,
    // allocates the heat grid, and initialises the delay and Cathedral DSP.
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

        size = 48;
        state.resize(size * size, 0.f);
        next.resize(size * size, 0.f);
        sampleRate = 48000.0f;
        delay.setSampleRate(sampleRate);
        cathedral.init();
    }

    // Rack reset hook: clears the heat grid, the Cathedral engine and the
    // DC-blocker state.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        std::fill(state.begin(), state.end(), 0.f);
        std::fill(next.begin(), next.end(), 0.f);
        cathedral.reset();
        dcL = dcR = 0.f;
    }

    // Downsamples the current heat grid into the `viz` snapshot for display
    // and auto-scales it so the map stays readable at any signal level.
    void updateViz()
    {
        float peak = 0.f;
        for (int r = 0; r < VIZ_ROWS; ++r)
        {
            int sy = clamp((r * size) / VIZ_ROWS + size / (2 * VIZ_ROWS), 0, size - 1);
            for (int c = 0; c < VIZ_COLS; ++c)
            {
                int sx = clamp((c * size) / VIZ_COLS + size / (2 * VIZ_COLS), 0, size - 1);
                float s = state[sy * size + sx];
                viz[r][c] = s;
                peak = std::max(peak, std::fabs(s));
            }
        }
        // Auto-scale so the display stays readable at any level
        vizNorm += (std::max(peak, 0.01f) - vizNorm) * (peak > vizNorm ? 0.6f : 0.03f);
        const float k = 4.f / vizNorm; // the widget applies tanh(v * 0.25)
        for (int r = 0; r < VIZ_ROWS; ++r)
            for (int c = 0; c < VIZ_COLS; ++c)
                viz[r][c] *= k;
    }

    // Main audio callback: reads the controls, runs the delay, Cathedral
    // reverb and heat-grid diffusion, mixes dry/wet, applies soft limiting and
    // safety checks, writes the stereo outputs, and refreshes the display
    // snapshot periodically.
    void process(const ProcessArgs &args) override
    {
        sampleRate = args.sampleRate;
        delay.setSampleRate(sampleRate);

        const float diffusion = params[DIFFUSION_PARAM].getValue();
        const float decayP = params[DECAY_PARAM].getValue();
        const float mix = params[MIX_PARAM].getValue();
        const float damping = params[DAMPING_PARAM].getValue();
        const float level = params[LEVEL_PARAM].getValue();
        const float space = params[SPACE_PARAM].getValue();
        const float sizeP = params[SIZE_PARAM].getValue();

        // Cathedral settings
        const float rt60 = 0.5f * std::pow(30.0f, decayP);              // 0.5 s .. 15 s (7.6 s default)
        const float scaleTarget = 0.6f + 1.4f * (sizeP - 4.0f) / 60.0f; // room scale 0.6 .. 2.0

        // Heat-grid settings
        float diffK = diffusion * 0.25f;
        float gridFb = 1.0f - 0.01f * std::pow(1.0f - decayP, 2.5f);
        gridFb = rack::math::clamp(gridFb, 0.99f, 0.99995f);
        lpL.setCutoff(rack::math::clamp(4000.0f / (sampleRate * 0.5f), 0.0f, 1.0f));
        lpR.setCutoff(rack::math::clamp(4000.0f / (sampleRate * 0.5f), 0.0f, 1.0f));

        int newSize = static_cast<int>(std::round(sizeP));
        newSize = rack::math::clamp(newSize, 4, 64);
        if (newSize != size)
        {
            size = newSize;
            state.assign(size * size, 0.f);
            next.assign(size * size, 0.f);
        }

        // Delay / Echo
        float delayTimeMs = params[DELAY_TIME_PARAM].getValue();
        float spreadMs = params[STEREO_SPREAD_PARAM].getValue();
        float delayTimeL = delayTimeMs - spreadMs * 0.5f;
        float delayTimeR = delayTimeMs + spreadMs * 0.5f;
        delayTimeL = rack::math::clamp(delayTimeL, 5.0f, 1000.0f);
        delayTimeR = rack::math::clamp(delayTimeR, 5.0f, 1000.0f);
        delay.setDelayMs(delayTimeL, delayTimeR);
        delay.setFeedback(params[DELAY_FEEDBACK_PARAM].getValue());

        float inL_raw = inputs[INPUT_L].getVoltage();
        float inR_raw = inputs[INPUT_R].getVoltage();
        std::pair<float, float> delayed = delay.process(inL_raw, inR_raw);
        float delayLevel = params[DELAY_LEVEL_PARAM].getValue();

        // The reverb hears the dry signal plus the echoes
        float reverbInputL = inL_raw + delayed.first * delayLevel;
        float reverbInputR = inR_raw + delayed.second * delayLevel;

        // ---- Cathedral ----
        float fdnL, fdnR, erL, erR;
        cathedral.process(reverbInputL, reverbInputR, sampleRate, rt60, damping,
                          scaleTarget, diffusion, space, fdnL, fdnR, erL, erR);

        // ---- Heat grid: gentle excitation, bounded output ----
        float tailComp = std::min(1.0f, std::sqrt((1.0f - gridFb) / 6e-4f));
        float injectionGain = 0.3f * tailComp;
        for (int y = 0; y < size; ++y)
            state[y * size + 0] += reverbInputL * injectionGain;
        for (int x = 0; x < size; ++x)
            state[(size - 1) * size + x] += reverbInputR * injectionGain;

        for (int y = 0; y < size; ++y)
        {
            for (int x = 0; x < size; ++x)
            {
                int idx = y * size + x;
                float u = state[idx];
                int leftX = (x == 0) ? 1 : x - 1;
                int rightX = (x == size - 1) ? size - 2 : x + 1;
                int upY = (y == 0) ? 1 : y - 1;
                int downY = (y == size - 1) ? size - 2 : y + 1;
                float u_left = state[y * size + leftX];
                float u_right = state[y * size + rightX];
                float u_up = state[upY * size + x];
                float u_down = state[downY * size + x];
                float laplacian = (u_left + u_right + u_up + u_down) - 4.0f * u;
                next[idx] = u + diffK * laplacian;
            }
        }
        for (int i = 0; i < size * size; ++i)
            next[i] *= gridFb;
        state.swap(next);

        // Samples a heat-grid cell at normalised coordinates (fx, fy).
        auto tap = [&](float fx, float fy)
        {
            int tx = rack::math::clamp((int)(fx * size), 0, size - 1);
            int ty = rack::math::clamp((int)(fy * size), 0, size - 1);
            return state[ty * size + tx];
        };
        float rawL = (tap(0.28f, 0.62f) + tap(0.50f, 0.50f) + tap(0.72f, 0.35f)) * (1.0f / 3.0f);
        float rawR = (tap(0.62f, 0.28f) + tap(0.50f, 0.50f) + tap(0.35f, 0.72f)) * (1.0f / 3.0f);
        float dcCoef = 1.0f - std::exp(-2.0f * float(M_PI) * 40.0f / sampleRate);
        dcL += (rawL - dcL) * dcCoef;
        dcR += (rawR - dcR) * dcCoef;
        float heatL = lpL.process(rawL - dcL);
        float heatR = lpR.process(rawR - dcR);
        heatL = 3.0f * std::tanh(heatL * 0.3f / 3.0f); // hard-bounded to +/-3 V
        heatR = 3.0f * std::tanh(heatR * 0.3f / 3.0f);

        // ---- Wet mix ----
        const float erGain = 0.35f * space;
        float wetL = (fdnL + erL * erGain + 0.35f * heatL) * level;
        float wetR = (fdnR + erR * erGain + 0.35f * heatR) * level;

        // Smooth limiter on the wet path: loud input can never blow up the output
        wetL = 8.0f * std::tanh(wetL / 8.0f);
        wetR = 8.0f * std::tanh(wetR / 8.0f);

        // Safety net: if anything ever goes non-finite, flush the reverb instead of screaming
        if (!std::isfinite(wetL) || !std::isfinite(wetR))
        {
            cathedral.reset();
            std::fill(state.begin(), state.end(), 0.f);
            std::fill(next.begin(), next.end(), 0.f);
            wetL = wetR = 0.f;
        }

        float outL = (1.0f - mix) * inL_raw + mix * wetL;
        float outR = (1.0f - mix) * inR_raw + mix * wetR;

        // Soft-clips the output above ±9 V with a gentle 0.3 slope.
        auto softClip = [](float x)
        {
            const float limit = 9.0f;
            if (x > limit)
                return limit + (x - limit) * 0.3f;
            if (x < -limit)
                return -limit + (x + limit) * 0.3f;
            return x;
        };
        outL = softClip(outL);
        outR = softClip(outR);
        outL = rack::math::clamp(outL, -10.0f, 10.0f);
        outR = rack::math::clamp(outR, -10.0f, 10.0f);

        outputs[OUTPUT_L].setVoltage(outL);
        outputs[OUTPUT_R].setVoltage(outR);

        // Refresh the display snapshot ~90 times per second at 48 kHz
        if (++vizCounter >= 512)
        {
            vizCounter = 0;
            updateViz();
        }
    }
};

// ------------------------------------------------------------
// UI (anonymous namespace: names cannot clash with other module files)
// ------------------------------------------------------------
namespace
{

    // Static artwork: base panel, display bezel, the REVERB section (three
    // knobs in row 1, four in row 2), the DELAY section, and the I/O row with
    // tinted output jacks.
    struct HRPanelBackground : TransparentWidget
    {
        HRPanelBackground() { box.size = Vec(HR_PANEL_W, HR_PANEL_H); }

        // Draws all static panel art: base panel, bezel, REVERB/DELAY section
        // headers and their knob cells, and the I/O row with four jack cells
        // (outputs tinted).
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
    // Heat-map display: map on top, readout row under it
    // ------------------------------------------------------------
    struct HRDisplayWidget : TransparentWidget
    {
        HeatRev *module = nullptr;

        HRDisplayWidget(HeatRev *m) : module(m)
        {
            box.pos = Vec(HR_DISP_X, HR_DISP_Y);
            box.size = Vec(HR_DISP_W, HR_DISP_H);
        }

        // Static layer: fills the display screen, draws the faint grid over
        // the heat-map area, and adds the divider between the map and the
        // readout row.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, HR_DISP_W, HR_DISP_H);

            // Faint grid over the map area only
            nvgBeginPath(vg);
            for (int i = 0; i <= VIZ_COLS; i += 4)
            {
                nvgMoveTo(vg, i * VIZ_CELL + 0.5f, 0);
                nvgLineTo(vg, i * VIZ_CELL + 0.5f, HR_VIZ_H);
            }
            for (int i = 0; i <= VIZ_ROWS; i += 4)
            {
                nvgMoveTo(vg, 0, i * VIZ_CELL + 0.5f);
                nvgLineTo(vg, HR_VIZ_W, i * VIZ_CELL + 0.5f);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Divider between the map and the readout row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, HR_VIZ_H + 0.5f);
            nvgLineTo(vg, HR_DISP_W, HR_VIZ_H + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the heat-map cells (cyan positive, violet
        // negative, opacity by magnitude) and the readout row with the grid
        // size and the cyan/violet sign legend. Uses a demo pattern in the
        // module browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            // Cells: cyan = positive, violet = negative, opacity = magnitude (flat fills)
            for (int r = 0; r < VIZ_ROWS; ++r)
            {
                for (int c = 0; c < VIZ_COLS; ++c)
                {
                    float v;
                    if (module)
                        v = module->viz[r][c];
                    else
                        v = 8.f * std::sin(c * 0.33f) * std::cos(r * 0.55f) * std::exp(-0.025f * c);

                    float n = std::tanh(v * 0.25f);
                    float a = std::fabs(n);
                    if (a < 0.03f)
                        continue;

                    nvgBeginPath(vg);
                    nvgRect(vg, c * VIZ_CELL + 0.4f, r * VIZ_CELL + 0.4f, VIZ_CELL - 0.8f, VIZ_CELL - 0.8f);
                    nvgFillColor(vg, agmm::withA(n > 0.f ? agmm::accent() : agmm::violet(), 0.2f + 0.8f * a));
                    nvgFill(vg);
                }
            }

            // ---- readout row (own row, never over the map) ----
            char buf[32];
            int sz = module ? module->size : 48;
            snprintf(buf, sizeof(buf), "GRID %dx%d", sz, sz);
            agmm::text(vg, 4.f, HR_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);

            const float negW = agmm::textWidth(vg, "VIOLET -", 7.f, 0.5f);
            agmm::text(vg, HR_DISP_W - 4.f, HR_FOOT_Y, "VIOLET -", 7.f, agmm::violet(), agmm::ALIGN_R, 0.5f);
            agmm::text(vg, HR_DISP_W - 4.f - negW - 6.f, HR_FOOT_Y, "CYAN +", 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget: adds the static panel, screws, the heat-map display, the
// reverb knobs, the delay knobs and the two in / two out jacks.
// ------------------------------------------------------------
struct HeatRevWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, live display, the
    // three reverb row-1 knobs, the four reverb row-2 knobs, the four delay
    // knobs, and the I/O jacks.
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