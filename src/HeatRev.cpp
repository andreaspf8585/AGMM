#include "plugin.hpp"
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <utility>

// ------------------------------------------------------------
// DSP
// ------------------------------------------------------------

// One-pole low-pass
struct SimpleLPF {
    float y = 0.0f;
    float a = 0.0f;

    void setCutoff(float normCutoff) {
        if (normCutoff <= 0.0f) a = 0.0f;
        else if (normCutoff >= 1.0f) a = 1.0f;
        else a = 1.0f - std::exp(-2.0f * float(M_PI) * normCutoff);
    }

    float process(float x) {
        y = y + a * (x - y);
        return y;
    }
};

// Stereo delay with feedback and fractional delay
struct StereoDelay {
    std::vector<float> bufferL, bufferR;
    int writePos = 0;
    int bufferSize = 0;
    float sampleRate = 48000.0f;
    float delayTimeL = 0.0f;
    float delayTimeR = 0.0f;
    float feedback = 0.0f;

    void setSampleRate(float sr) { sampleRate = sr; }

    void resize(int maxDelaySamples) {
        if (maxDelaySamples != bufferSize) {
            bufferSize = maxDelaySamples + 2;
            bufferL.assign(bufferSize, 0.0f);
            bufferR.assign(bufferSize, 0.0f);
            writePos = 0;
        }
    }

    void setDelayMs(float msL, float msR) {
        delayTimeL = msL * 0.001f * sampleRate;
        delayTimeR = msR * 0.001f * sampleRate;
        int maxSamples = static_cast<int>(std::max(delayTimeL, delayTimeR)) + 2;
        if (maxSamples > bufferSize) resize(maxSamples);
    }

    void setFeedback(float fb) { feedback = rack::math::clamp(fb, 0.0f, 0.95f); }

    std::pair<float, float> process(float inL, float inR) {
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
    float readBuffer(const std::vector<float>& buf, float delaySamples) {
        if (delaySamples <= 0.0f) return 0.0f;
        float readPos = static_cast<float>(writePos) - delaySamples;
        if (readPos < 0.0f) readPos += bufferSize;
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
static const int   CATH_N = 8;
static const int   CATH_BASE[CATH_N] = {1031, 1327, 1621, 1933, 2269, 2663, 3109, 3571};   // samples @ 48 kHz
static const int   CATH_AP_L[4] = {113, 173, 307, 443};
static const int   CATH_AP_R[4] = {127, 191, 331, 461};
static const float CATH_ER_MS_L[7] = {7.f, 19.f, 31.f, 47.f, 67.f, 89.f, 113.f};
static const float CATH_ER_MS_R[7] = {11.f, 23.f, 37.f, 53.f, 73.f, 97.f, 127.f};
static const float CATH_ER_G_L[7]  = { 0.80f, -0.65f,  0.55f, -0.45f,  0.38f, -0.30f,  0.24f};
static const float CATH_ER_G_R[7]  = {-0.78f,  0.62f, -0.52f,  0.43f, -0.36f,  0.29f, -0.23f};

struct Cathedral {
    enum { N = CATH_N, LINE = 32768, LMASK = LINE - 1, PD = 131072, PDMASK = PD - 1, AP = 2048 };

    std::vector<float> line[N];
    std::vector<float> pdL, pdR;
    std::vector<float> apL[4], apR[4];
    int   w[N];
    int   pdW = 0;
    int   apLi[4], apRi[4];
    float lp[N], lfoPh[N], g[N];
    float inGain = 0.3f, scaleSm = 1.f, lastScale = -1.f, lastRT = -1.f, lastDamp = -1.f, b = 0.5f;
    float hpL = 0.f, hpR = 0.f;

    void init() {
        for (int i = 0; i < N; ++i) line[i].assign(LINE, 0.f);
        pdL.assign(PD, 0.f);
        pdR.assign(PD, 0.f);
        for (int k = 0; k < 4; ++k) { apL[k].assign(AP, 0.f); apR[k].assign(AP, 0.f); }
        reset();
    }

    void reset() {
        for (int i = 0; i < N; ++i) {
            std::fill(line[i].begin(), line[i].end(), 0.f);
            w[i] = 0; lp[i] = 0.f; g[i] = 0.9f; lfoPh[i] = 0.8f * i;
        }
        std::fill(pdL.begin(), pdL.end(), 0.f);
        std::fill(pdR.begin(), pdR.end(), 0.f);
        for (int k = 0; k < 4; ++k) {
            std::fill(apL[k].begin(), apL[k].end(), 0.f);
            std::fill(apR[k].begin(), apR[k].end(), 0.f);
            apLi[k] = apRi[k] = 0;
        }
        pdW = 0; hpL = hpR = 0.f;
        lastScale = lastRT = lastDamp = -1.f;
    }

    static float allpass(std::vector<float>& buf, int& idx, int len, float gain, float x) {
        if (len < 2) len = 2;
        if (idx >= len) idx = 0;
        const float d  = buf[idx];
        const float wv = x + gain * d;
        buf[idx] = wv;
        if (++idx >= len) idx = 0;
        return d - gain * wv;
    }

    float readPD(const std::vector<float>& buf, float d) const {
        const float pos = (float) pdW - d;
        int i0 = (int) std::floor(pos);
        const float fr = pos - (float) i0;
        i0 &= PDMASK;
        const int i1 = (i0 + 1) & PDMASK;
        return buf[i0] + fr * (buf[i1] - buf[i0]);
    }

    // fdn = diffuse tail, er = early reflections
    void process(float inL, float inR, float sr, float rt60, float damping,
                 float scaleTarget, float diffusion, float space,
                 float& fdnL, float& fdnR, float& erL, float& erR)
    {
        const float srk = sr / 48000.f;
        const float twoPi = 6.2831853f;

        if (lastScale < 0.f) scaleSm = scaleTarget;
        scaleSm += (scaleTarget - scaleSm) * 0.0001f;       // slow, so SIZE changes don't click

        // Per-line loop gain from RT60 (-60 dB after rt60 seconds)
        if (lastScale < 0.f || std::fabs(scaleSm - lastScale) > 1e-4f || rt60 != lastRT) {
            lastScale = scaleSm;
            lastRT = rt60;
            float gm2 = 0.f;
            for (int i = 0; i < N; ++i) {
                const float len = (float) CATH_BASE[i] * srk * scaleSm;
                g[i] = std::pow(10.f, -3.f * len / (rt60 * sr));
                gm2 += g[i] * g[i];
            }
            // keeps reverb loudness roughly constant whatever the decay time
            inGain = std::sqrt(std::max(0.02f, 1.f - gm2 / (float) N));
        }

        // Damping: stone and air eat the highs on every trip round the loop
        if (damping != lastDamp) {
            lastDamp = damping;
            float fc = 14000.f * std::pow(0.0857f, damping);        // 14 kHz .. 1.2 kHz
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
        for (int k = 0; k < 7; ++k) {
            eL += CATH_ER_G_L[k] * readPD(pdL, pre + CATH_ER_MS_L[k] * tscale);
            eR += CATH_ER_G_R[k] * readPD(pdR, pre + CATH_ER_MS_R[k] * tscale);
        }
        pdW = (pdW + 1) & PDMASK;

        // Input diffusion
        const float ag = 0.25f + 0.5f * diffusion;
        for (int k = 0; k < 4; ++k) {
            dl = allpass(apL[k], apLi[k], (int) (CATH_AP_L[k] * srk), ag, dl);
            dr = allpass(apR[k], apRi[k], (int) (CATH_AP_R[k] * srk), ag, dr);
        }

        // Read the 8 delay lines (slowly modulated to avoid metallic ringing)
        const float depth = (1.f + 7.f * space) * srk;
        float x[N];
        for (int i = 0; i < N; ++i) {
            const float len = (float) CATH_BASE[i] * srk * scaleSm + depth * std::sin(lfoPh[i]);
            lfoPh[i] += twoPi * (0.11f + 0.037f * (float) i) / sr;
            if (lfoPh[i] > twoPi) lfoPh[i] -= twoPi;

            const float pos = (float) w[i] - len;
            int i0 = (int) std::floor(pos);
            const float fr = pos - (float) i0;
            i0 &= LMASK;
            const int i1 = (i0 + 1) & LMASK;
            x[i] = line[i][i0] + fr * (line[i][i1] - line[i][i0]);
        }

        // Damping filter + loop gain
        float y[N];
        for (int i = 0; i < N; ++i) {
            lp[i] = (1.f - b) * x[i] + b * lp[i];
            y[i] = lp[i] * g[i];
        }

        // Orthonormal 8x8 Hadamard mix (energy preserving)
        for (int h = 1; h < N; h <<= 1) {
            for (int i = 0; i < N; i += (h << 1)) {
                for (int j = i; j < i + h; ++j) {
                    const float a = y[j], c = y[j + h];
                    y[j] = a + c;
                    y[j + h] = a - c;
                }
            }
        }

        // Write back, injecting left into even lines and right into odd lines
        const float k8 = 0.35355339f;
        for (int i = 0; i < N; ++i) {
            const float inj = (i & 1) ? dr : dl;
            float v = y[i] * k8 + inj * inGain;
            v = rack::math::clamp(v, -64.f, 64.f);              // belt and braces
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
static const int   VIZ_COLS = 40;
static const int   VIZ_ROWS = 16;
static const float VIZ_CELL = 4.f;

static const float HR_PANEL_W = RACK_GRID_WIDTH * 12;   // 180
static const float HR_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float HR_DISP_W = VIZ_COLS * VIZ_CELL;     // 160
static const float HR_DISP_H = VIZ_ROWS * VIZ_CELL;     // 64
static const float HR_DISP_X = (HR_PANEL_W - HR_DISP_W) / 2.f;
static const float HR_DISP_Y = 52.f;

static const float HR_KX3[3] = {30.f, 90.f, 150.f};
static const float HR_KX4[4] = {27.f, 69.f, 111.f, 153.f};

static const float HR_ROW1  = 152.f;
static const float HR_ROW2  = 196.f;
static const float HR_ROW3  = 254.f;
static const float HR_PORTY = 324.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct HeatRev : Module {
    enum ParamId {
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
        SPACE_PARAM,            // appended last so old patches keep their settings
        PARAMS_LEN
    };
    enum InputId { INPUT_L, INPUT_R, INPUTS_LEN };
    enum OutputId { OUTPUT_L, OUTPUT_R, OUTPUTS_LEN };
    enum LightId { LIGHTS_LEN };

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

    HeatRev() {
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

        size = 48;
        state.resize(size * size, 0.f);
        next.resize(size * size, 0.f);
        sampleRate = 48000.0f;
        delay.setSampleRate(sampleRate);
        cathedral.init();
    }

    void onReset() override {
        std::fill(state.begin(), state.end(), 0.f);
        std::fill(next.begin(), next.end(), 0.f);
        cathedral.reset();
        dcL = dcR = 0.f;
    }

    void updateViz() {
        float peak = 0.f;
        for (int r = 0; r < VIZ_ROWS; ++r) {
            int sy = clamp((r * size) / VIZ_ROWS + size / (2 * VIZ_ROWS), 0, size - 1);
            for (int c = 0; c < VIZ_COLS; ++c) {
                int sx = clamp((c * size) / VIZ_COLS + size / (2 * VIZ_COLS), 0, size - 1);
                float s = state[sy * size + sx];
                viz[r][c] = s;
                peak = std::max(peak, std::fabs(s));
            }
        }
        // Auto-scale so the display stays readable at any level
        vizNorm += (std::max(peak, 0.01f) - vizNorm) * (peak > vizNorm ? 0.6f : 0.03f);
        const float k = 4.f / vizNorm;      // the widget applies tanh(v * 0.25)
        for (int r = 0; r < VIZ_ROWS; ++r)
            for (int c = 0; c < VIZ_COLS; ++c)
                viz[r][c] *= k;
    }

    void process(const ProcessArgs& args) override {
        sampleRate = args.sampleRate;
        delay.setSampleRate(sampleRate);

        const float diffusion = params[DIFFUSION_PARAM].getValue();
        const float decayP    = params[DECAY_PARAM].getValue();
        const float mix       = params[MIX_PARAM].getValue();
        const float damping   = params[DAMPING_PARAM].getValue();
        const float level     = params[LEVEL_PARAM].getValue();
        const float space     = params[SPACE_PARAM].getValue();
        const float sizeP     = params[SIZE_PARAM].getValue();

        // Cathedral settings
        const float rt60        = 0.5f * std::pow(30.0f, decayP);          // 0.5 s .. 15 s (7.6 s default)
        const float scaleTarget = 0.6f + 1.4f * (sizeP - 4.0f) / 60.0f;    // room scale 0.6 .. 2.0

        // Heat-grid settings
        float diffK = diffusion * 0.25f;
        float gridFb = 1.0f - 0.01f * std::pow(1.0f - decayP, 2.5f);
        gridFb = rack::math::clamp(gridFb, 0.99f, 0.99995f);
        lpL.setCutoff(rack::math::clamp(4000.0f / (sampleRate * 0.5f), 0.0f, 1.0f));
        lpR.setCutoff(rack::math::clamp(4000.0f / (sampleRate * 0.5f), 0.0f, 1.0f));

        int newSize = static_cast<int>(std::round(sizeP));
        newSize = rack::math::clamp(newSize, 4, 64);
        if (newSize != size) {
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

        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                int idx = y * size + x;
                float u = state[idx];
                int leftX  = (x == 0) ? 1 : x - 1;
                int rightX = (x == size - 1) ? size - 2 : x + 1;
                int upY    = (y == 0) ? 1 : y - 1;
                int downY  = (y == size - 1) ? size - 2 : y + 1;
                float u_left  = state[y * size + leftX];
                float u_right = state[y * size + rightX];
                float u_up    = state[upY * size + x];
                float u_down  = state[downY * size + x];
                float laplacian = (u_left + u_right + u_up + u_down) - 4.0f * u;
                next[idx] = u + diffK * laplacian;
            }
        }
        for (int i = 0; i < size * size; ++i) next[i] *= gridFb;
        state.swap(next);

        auto tap = [&](float fx, float fy) {
            int tx = rack::math::clamp((int) (fx * size), 0, size - 1);
            int ty = rack::math::clamp((int) (fy * size), 0, size - 1);
            return state[ty * size + tx];
        };
        float rawL = (tap(0.28f, 0.62f) + tap(0.50f, 0.50f) + tap(0.72f, 0.35f)) * (1.0f / 3.0f);
        float rawR = (tap(0.62f, 0.28f) + tap(0.50f, 0.50f) + tap(0.35f, 0.72f)) * (1.0f / 3.0f);
        float dcCoef = 1.0f - std::exp(-2.0f * float(M_PI) * 40.0f / sampleRate);
        dcL += (rawL - dcL) * dcCoef;
        dcR += (rawR - dcR) * dcCoef;
        float heatL = lpL.process(rawL - dcL);
        float heatR = lpR.process(rawR - dcR);
        heatL = 3.0f * std::tanh(heatL * 0.3f / 3.0f);          // hard-bounded to +/-3 V
        heatR = 3.0f * std::tanh(heatR * 0.3f / 3.0f);

        // ---- Wet mix ----
        const float erGain = 0.35f * space;
        float wetL = (fdnL + erL * erGain + 0.35f * heatL) * level;
        float wetR = (fdnR + erR * erGain + 0.35f * heatR) * level;

        // Smooth limiter on the wet path: loud input can never blow up the output
        wetL = 8.0f * std::tanh(wetL / 8.0f);
        wetR = 8.0f * std::tanh(wetR / 8.0f);

        // Safety net: if anything ever goes non-finite, flush the reverb instead of screaming
        if (!std::isfinite(wetL) || !std::isfinite(wetR)) {
            cathedral.reset();
            std::fill(state.begin(), state.end(), 0.f);
            std::fill(next.begin(), next.end(), 0.f);
            wetL = wetR = 0.f;
        }

        float outL = (1.0f - mix) * inL_raw + mix * wetL;
        float outR = (1.0f - mix) * inR_raw + mix * wetR;

        auto softClip = [](float x) {
            const float limit = 9.0f;
            if (x > limit) return limit + (x - limit) * 0.3f;
            if (x < -limit) return -limit + (x + limit) * 0.3f;
            return x;
        };
        outL = softClip(outL);
        outR = softClip(outR);
        outL = rack::math::clamp(outL, -10.0f, 10.0f);
        outR = rack::math::clamp(outR, -10.0f, 10.0f);

        outputs[OUTPUT_L].setVoltage(outL);
        outputs[OUTPUT_R].setVoltage(outR);

        // Refresh the display snapshot ~90 times per second at 48 kHz
        if (++vizCounter >= 512) {
            vizCounter = 0;
            updateViz();
        }
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in CellAutoGate.cpp.
namespace {

// ------------------------------------------------------------
// Theme (same palette as CellAutoGate)
// ------------------------------------------------------------
NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }

void setFont(NVGcontext* vg) {
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font)
        nvgFontFaceId(vg, font->handle);
}

void drawText(NVGcontext* vg, float x, float y, const char* s, float size, NVGcolor col,
              int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f) {
    setFont(vg);
    nvgFontSize(vg, size);
    nvgTextLetterSpacing(vg, spacing);
    nvgTextAlign(vg, align);
    nvgFillColor(vg, col);
    nvgText(vg, x, y, s, NULL);
}

// ------------------------------------------------------------
// AGMM logo
// ------------------------------------------------------------
void drawLogo(NVGcontext* vg, float cx, float cy) {
    const float w = 64.f, h = 18.f;
    float x = cx - w / 2.f, y = cy - h / 2.f;

    // Glow
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
    nvgFill(vg);

    // Pill body
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 7.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                       nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
    nvgFill(vg);
    nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
                                         nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
    nvgStrokeWidth(vg, 1.2f);
    nvgStroke(vg);

    // Tiny cell triangle (a nod to cellular automata)
    const float s = 2.2f;
    float tx = x + 7.f, ty = y + h / 2.f - 3.f;
    nvgBeginPath(vg);
    nvgRect(vg, tx + s,       ty,         s, s);
    nvgRect(vg, tx,           ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);

    // Wordmark
    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
             NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct HRPanelBackground : TransparentWidget {
    HRPanelBackground() { box.size = Vec(HR_PANEL_W, HR_PANEL_H); }

    void label(NVGcontext* vg, float x, float y, const char* txt) {
        drawText(vg, x, y, txt, 8.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = HR_PANEL_W - 14.f, cx = HR_PANEL_W / 2.f;
        float w = 10.f + (float) strlen(txt) * 7.f;
        drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - w / 2.f, y);
        nvgMoveTo(vg, cx + w / 2.f, y);
        nvgLineTo(vg, x2, y);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void ring(NVGcontext* vg, float cx, float cy) {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, 20.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, HR_PANEL_W, HR_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, HR_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) HR_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, HR_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, HR_PANEL_W - 1.f, HR_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, HR_PANEL_W / 2.f, 15.f);
        drawText(vg, HR_PANEL_W / 2.f, 34.f, "HEATREV", 14.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = HR_DISP_X - 6.f, by = HR_DISP_Y - 6.f;
        float bw = HR_DISP_W + 12.f, bh = HR_DISP_H + 12.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // Sections
        section(vg, 130.f, "REVERB");
        section(vg, 232.f, "DELAY");
        section(vg, 290.f, "I / O");

        // Knob rings: row 1 has three knobs, row 2 four (Space added)
        for (int i = 0; i < 3; ++i) ring(vg, HR_KX3[i], HR_ROW1);
        for (int i = 0; i < 4; ++i) { ring(vg, HR_KX4[i], HR_ROW2); ring(vg, HR_KX4[i], HR_ROW3); }

        // Knob labels
        const char* rev1[3] = {"DIFFUSION", "DECAY", "SIZE"};
        const char* rev2[4] = {"MIX", "DAMPING", "LEVEL", "SPACE"};
        for (int i = 0; i < 3; ++i)
            label(vg, HR_KX3[i], HR_ROW1 + 26.f, rev1[i]);
        for (int i = 0; i < 4; ++i)
            label(vg, HR_KX4[i], HR_ROW2 + 26.f, rev2[i]);
        const char* dly[4] = {"TIME", "FDBK", "LEVEL", "SPREAD"};
        for (int i = 0; i < 4; ++i)
            label(vg, HR_KX4[i], HR_ROW3 + 26.f, dly[i]);

        // Port plates + labels
        const char* io[4] = {"IN L", "IN R", "OUT L", "OUT R"};
        for (int i = 0; i < 4; ++i) {
            float cx = HR_KX4[i];
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 18.f, HR_PORTY - 18.f, 36.f, 36.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            label(vg, cx, HR_PORTY + 28.f, io[i]);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Heat-map display
// ------------------------------------------------------------
struct HRDisplayWidget : TransparentWidget {
    HeatRev* module = nullptr;

    HRDisplayWidget(HeatRev* m) : module(m) {
        box.pos = Vec(HR_DISP_X, HR_DISP_Y);
        box.size = Vec(HR_DISP_W, HR_DISP_H);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, HR_DISP_W, HR_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, HR_DISP_W / 2.f, HR_DISP_H / 2.f, 20.f, HR_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        // Faint grid
        nvgBeginPath(vg);
        for (int i = 0; i <= VIZ_COLS; i += 4) {
            nvgMoveTo(vg, i * VIZ_CELL + 0.5f, 0);
            nvgLineTo(vg, i * VIZ_CELL + 0.5f, HR_DISP_H);
        }
        for (int i = 0; i <= VIZ_ROWS; i += 4) {
            nvgMoveTo(vg, 0, i * VIZ_CELL + 0.5f);
            nvgLineTo(vg, HR_DISP_W, i * VIZ_CELL + 0.5f);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        // Cells: cyan = positive, violet = negative, brightness = magnitude
        for (int r = 0; r < VIZ_ROWS; ++r) {
            for (int c = 0; c < VIZ_COLS; ++c) {
                float v;
                if (module)
                    v = module->viz[r][c];
                else
                    v = 8.f * std::sin(c * 0.33f) * std::cos(r * 0.55f) * std::exp(-0.025f * c);

                float n = std::tanh(v * 0.25f);
                float a = std::fabs(n);
                if (a < 0.03f) continue;

                float x = c * VIZ_CELL, y = r * VIZ_CELL;
                bool pos = n > 0.f;

                // Glow
                nvgBeginPath(vg);
                nvgRect(vg, x - 1.f, y - 1.f, VIZ_CELL + 2.f, VIZ_CELL + 2.f);
                nvgFillColor(vg, pos ? nvgRGBAf(0.24f, 0.88f, 1.f, 0.12f * a)
                                     : nvgRGBAf(0.69f, 0.35f, 1.f, 0.12f * a));
                nvgFill(vg);

                // Crisp cell
                nvgBeginPath(vg);
                nvgRect(vg, x + 0.4f, y + 0.4f, VIZ_CELL - 0.8f, VIZ_CELL - 0.8f);
                if (pos)
                    nvgFillColor(vg, nvgRGBAf(0.24f + 0.6f * a, 0.88f + 0.1f * a, 1.f, 0.2f + 0.8f * a));
                else
                    nvgFillColor(vg, nvgRGBAf(0.69f + 0.25f * a, 0.35f + 0.3f * a, 1.f, 0.2f + 0.8f * a));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) HR_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, HR_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, HR_DISP_W, HR_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, HR_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readout
        char buf[32];
        int sz = module ? module->size : 48;
        snprintf(buf, sizeof(buf), "GRID %dx%d", sz, sz);
        drawText(vg, 4.f, HR_DISP_H - 6.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 170),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct HeatRevWidget : ModuleWidget {
    HeatRevWidget(HeatRev* module) {
        setModule(module);
        box.size = Vec(HR_PANEL_W, HR_PANEL_H);

        addChild(new HRPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

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

Model* modelHeatRev = createModel<HeatRev, HeatRevWidget>("HeatRev");