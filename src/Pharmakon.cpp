#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <string>
#include <functional>

using namespace rack;

// ============================================================
//  Constants & helpers
// ============================================================
static constexpr float PH_PI     = 3.14159265358979323846f;
static constexpr float PH_TWO_PI = 2.0f * PH_PI;

static inline float phClamp(float v, float lo, float hi)
{
    return std::max(lo, std::min(v, hi));
}

static inline float phMapFreq(float t, float minFreq, float maxFreq)
{
    return minFreq * std::pow(maxFreq / minFreq, t);
}

// Transparent up to 6 V, smooth knee towards a hard ceiling of 10 V
static inline float phSoftLimit(float x)
{
    const float k = 6.f, r = 4.f;
    float a = std::fabs(x);
    if (a <= k)
        return x;
    float y = k + r * std::tanh((a - k) / r);
    return x < 0.f ? -y : y;
}

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   PH_SCOPE_N    = 160;     // waveform columns
static const int   PH_FFT_N      = 1024;    // FFT size
static const int   PH_FFT_RING   = 4096;    // audio -> UI ring buffer (power of 2)
static const int   PH_BARS       = 24;      // analyzer bars
static const int   PH_SPEC_COLS  = 150;     // spectrum curve resolution
static const int   PH_NUM_CTRL   = 16;      // knobs with CV + attenuverter
static const float PH_FMIN       = 30.f;    // analyzer lowest frequency
static const float PH_FMAX       = 18000.f; // analyzer highest frequency
static const float PH_DB_FLOOR   = -72.f;   // analyzer floor (dB re 5 V)
static const float PH_TILT       = 3.f;     // dB/oct display tilt (flattens pink noise)

static const float PH_PANEL_W = RACK_GRID_WIDTH * 26;   // 390 px
static const float PH_PANEL_H = RACK_GRID_HEIGHT;       // 380 px

static const float PH_DISP_W = 350.f;
static const float PH_DISP_H = 38.f;
static const float PH_DISP_X = (PH_PANEL_W - PH_DISP_W) / 2.f;
static const float PH_DISP_Y = 46.f;

static const float PH_CELL_W = 60.f;
static const float PH_CELL_H = 70.f;
static const float PH_ROW[3] = {112.f, 183.f, 254.f};   // knob centre of each row
static const float PH_PORT_Y = 348.f;

// 6 columns
static float phColX(int col) { return PH_PANEL_W / 12.f * (2 * col + 1); }

// ------------------------------------------------------------
// Control table (grid order: 6 columns x 3 rows, 16 knobs + FREEZE + CV RANGE)
// ------------------------------------------------------------
struct PHCtlSpec
{
    const char* name;    // panel label
    const char* label;   // tooltip label
    float lo, hi, def;
    const char* unit;
    float dispBase, dispMul;
};

static const PHCtlSpec PH_CTL[PH_NUM_CTRL] = {
    {"IN GAIN",    "Input gain",        0.f,    2.f,    1.f,  "%",   0.f, 100.f},
    {"DRY/WET",    "Dry/wet",           0.f,    1.f,    0.5f, "%",   0.f, 100.f},
    {"DELAY TIME", "Delay time",        0.005f, 2.f,    0.3f, " s",  0.f, 1.f},
    {"FEEDBACK",   "Feedback",          0.f,    0.98f,  0.5f, "%",   0.f, 100.f},
    {"DETUNE",     "Detune depth",      0.f,    1.f,    0.3f, " ms", 0.f, 5.f},
    {"DISTORT",    "Distortion",        0.f,    1.f,    0.f,  "%",   0.f, 100.f},
    {"LO GAIN",    "Low shelf gain",   -12.f,   12.f,   0.f,  " dB", 0.f, 1.f},
    {"HI GAIN",    "High shelf gain",  -12.f,   12.f,   0.f,  " dB", 0.f, 1.f},
    {"BRIGHT",     "Brightness",        0.f,    1.f,    0.7f, "%",   0.f, 100.f},
    {"LPF",        "LPF cutoff",        0.f,    1.f,    1.f,  "%",   0.f, 100.f},
    {"MIX",        "Delay/reverb mix",  0.f,    1.f,    0.5f, "%",   0.f, 100.f},
    {"REV DECAY",  "Reverb decay",      0.f,    0.95f,  0.5f, "%",   0.f, 100.f},
    {"SPACY MOD",  "Spacy mod",         0.f,    1.f,    0.f,  "%",   0.f, 100.f},
    {"SPACY RATE", "Spacy rate",        0.05f,  5.f,    0.5f, " Hz", 0.f, 1.f},
    {"WIDTH",      "Stereo width",      0.f,    2.f,    1.f,  "%",   0.f, 100.f},
    {"OUT LEVEL",  "Output level",      0.f,    2.f,    1.f,  "%",   0.f, 100.f},
};

// ------------------------------------------------------------
//  Delay line with linear interpolation (std::vector, bounds-safe)
// ------------------------------------------------------------
struct PHDelay
{
    std::vector<float> buf;
    int size = 0;
    int writePos = 0;
    float sampleRate = 44100.f;

    void initSamples(int samples, float sr)
    {
        sampleRate = sr;
        size = std::max(samples, 8);
        buf.assign(size, 0.f);
        writePos = 0;
    }

    void init(float maxDelaySec, float sr)
    {
        initSamples((int) (maxDelaySec * sr) + 4, sr);
    }

    void clear()
    {
        std::fill(buf.begin(), buf.end(), 0.f);
        writePos = 0;
    }

    void write(float s)
    {
        if (size <= 1) return;
        buf[writePos] = s;
        if (++writePos >= size) writePos = 0;
    }

    // delay in samples; minimum is 1 sample because we read before we write
    float readSamples(float d) const
    {
        if (size <= 1) return 0.f;
        d = phClamp(d, 1.f, (float) (size - 2));
        float rp = (float) writePos - d;
        if (rp < 0.f) rp += (float) size;
        int idx = (int) rp;
        float frac = rp - (float) idx;
        if (idx >= size) idx -= size;
        int next = idx + 1;
        if (next >= size) next = 0;
        return buf[idx] * (1.f - frac) + buf[next] * frac;
    }

    float read(float delaySec) const { return readSamples(delaySec * sampleRate); }
};

// ------------------------------------------------------------
//  One-pole lowpass
// ------------------------------------------------------------
struct PHOnePole
{
    float z = 0.f;
    float coeff = 1.f;

    void setCutoff(float freq, float sr)
    {
        if (freq < 1.f) freq = 1.f;
        if (freq > sr * 0.45f) freq = sr * 0.45f;
        coeff = 1.f - std::exp(-PH_TWO_PI * freq / sr);
    }
    float process(float in)
    {
        z += coeff * (in - z);
        return z;
    }
};

// ------------------------------------------------------------
//  Biquad shelving filter (RBJ cookbook, slope S = 1)
// ------------------------------------------------------------
struct PHBiquad
{
    float b0 = 1.f, b1 = 0.f, b2 = 0.f, a1 = 0.f, a2 = 0.f;
    float z1 = 0.f, z2 = 0.f;

    float process(float x)
    {
        float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }

    void copyCoeffs(const PHBiquad& o)
    {
        b0 = o.b0; b1 = o.b1; b2 = o.b2; a1 = o.a1; a2 = o.a2;
    }

    void setLowShelf(float freq, float dB, float sr)
    {
        freq = phClamp(freq, 10.f, sr * 0.45f);
        float A = std::pow(10.f, dB / 40.f);
        float w0 = PH_TWO_PI * freq / sr;
        float cw = std::cos(w0), sw = std::sin(w0);
        float alpha = sw / std::sqrt(2.f);
        float tsa = 2.f * std::sqrt(A) * alpha;
        float a0 = (A + 1.f) + (A - 1.f) * cw + tsa;
        b0 = (A * ((A + 1.f) - (A - 1.f) * cw + tsa)) / a0;
        b1 = (2.f * A * ((A - 1.f) - (A + 1.f) * cw)) / a0;
        b2 = (A * ((A + 1.f) - (A - 1.f) * cw - tsa)) / a0;
        a1 = (-2.f * ((A - 1.f) + (A + 1.f) * cw)) / a0;
        a2 = ((A + 1.f) + (A - 1.f) * cw - tsa) / a0;
    }

    void setHighShelf(float freq, float dB, float sr)
    {
        freq = phClamp(freq, 10.f, sr * 0.45f);
        float A = std::pow(10.f, dB / 40.f);
        float w0 = PH_TWO_PI * freq / sr;
        float cw = std::cos(w0), sw = std::sin(w0);
        float alpha = sw / std::sqrt(2.f);
        float tsa = 2.f * std::sqrt(A) * alpha;
        float a0 = (A + 1.f) - (A - 1.f) * cw + tsa;
        b0 = (A * ((A + 1.f) + (A - 1.f) * cw + tsa)) / a0;
        b1 = (-2.f * A * ((A - 1.f) + (A + 1.f) * cw)) / a0;
        b2 = (A * ((A + 1.f) + (A - 1.f) * cw - tsa)) / a0;
        a1 = (2.f * ((A - 1.f) - (A + 1.f) * cw)) / a0;
        a2 = ((A + 1.f) - (A - 1.f) * cw - tsa) / a0;
    }
};

// ------------------------------------------------------------
//  Stereo reverb (4 combs + 2 allpasses per side)
// ------------------------------------------------------------
struct PHReverb
{
    struct Comb
    {
        PHDelay delay;
        int len = 8;
        float feedback = 0.f;
        PHOnePole lpf;

        void init(int samples, float sr)
        {
            len = std::max(samples, 8);
            delay.initSamples(len + 4, sr);
            lpf.setCutoff(16000.f, sr);
            lpf.z = 0.f;
        }
        float process(float in)
        {
            float out = delay.readSamples((float) len);
            float damped = lpf.process(out);
            delay.write(in + damped * feedback);
            return out;
        }
    };

    struct Allpass
    {
        PHDelay delay;
        int len = 8;
        float gain = 0.7f;

        void init(int samples, float sr)
        {
            len = std::max(samples, 8);
            delay.initSamples(len + 4, sr);
        }
        float process(float in)
        {
            float dly = delay.readSamples((float) len);
            float out = -gain * in + dly;
            delay.write(in + gain * dly);
            return out;
        }
    };

    Comb combL[4], combR[4];
    Allpass allpassL[2], allpassR[2];
    float sampleRate = 44100.f;

    void init(float sr)
    {
        sampleRate = sr;
        const int combBase[4] = {1116, 1188, 1277, 1356};
        const int apBase[2]   = {556, 441};
        float k = sr / 44100.f;
        for (int i = 0; i < 4; ++i)
        {
            combL[i].init((int) (combBase[i] * k + 0.5f), sr);
            combR[i].init((int) ((combBase[i] + 23) * k + 0.5f), sr);
            combL[i].feedback = combR[i].feedback = 0.8f;
        }
        for (int i = 0; i < 2; ++i)
        {
            allpassL[i].init((int) (apBase[i] * k + 0.5f), sr);
            allpassR[i].init((int) ((apBase[i] + 11) * k + 0.5f), sr);
        }
    }

    void setDecay(float decay, float brightness)
    {
        float fb = 0.6f + decay * 0.38f;
        float cutoff = phMapFreq(brightness, 1500.f, 16000.f);
        for (int i = 0; i < 4; ++i)
        {
            combL[i].feedback = combR[i].feedback = fb;
            combL[i].lpf.setCutoff(cutoff, sampleRate);
            combR[i].lpf.setCutoff(cutoff, sampleRate);
        }
    }

    void process(float inL, float inR, float& outL, float& outR)
    {
        float wetL = 0.f, wetR = 0.f;
        for (int i = 0; i < 4; ++i)
        {
            wetL += combL[i].process(inL);
            wetR += combR[i].process(inR);
        }
        wetL *= 0.2f;
        wetR *= 0.2f;
        for (int i = 0; i < 2; ++i)
        {
            wetL = allpassL[i].process(wetL);
            wetR = allpassR[i].process(wetR);
        }
        outL = wetL;
        outR = wetR;
    }
};

// ------------------------------------------------------------
//  Small radix-2 FFT (used only by the display, on the UI thread)
// ------------------------------------------------------------
struct PHFFT
{
    static const int N = PH_FFT_N;
    static const int LOG2N = 10;
    float cosT[N / 2], sinT[N / 2];
    float window[N];
    int rev[N];

    PHFFT()
    {
        static_assert((1 << LOG2N) == N, "FFT size mismatch");
        for (int k = 0; k < N / 2; ++k)
        {
            cosT[k] = std::cos(PH_TWO_PI * k / N);
            sinT[k] = -std::sin(PH_TWO_PI * k / N);
        }
        for (int i = 0; i < N; ++i)
        {
            window[i] = 0.5f - 0.5f * std::cos(PH_TWO_PI * i / (N - 1));
            int r = 0;
            for (int b = 0; b < LOG2N; ++b)
                if ((i >> b) & 1)
                    r |= 1 << (LOG2N - 1 - b);
            rev[i] = r;
        }
    }

    void forward(const float* in, float* re, float* im) const
    {
        for (int i = 0; i < N; ++i)
        {
            re[rev[i]] = in[i] * window[i];
            im[rev[i]] = 0.f;
        }
        for (int len = 2; len <= N; len <<= 1)
        {
            int half = len / 2, step = N / len;
            for (int i = 0; i < N; i += len)
            {
                for (int j = 0; j < half; ++j)
                {
                    float wr = cosT[j * step], wi = sinT[j * step];
                    int a = i + j, b = i + j + half;
                    float vr = re[b] * wr - im[b] * wi;
                    float vi = re[b] * wi + im[b] * wr;
                    re[b] = re[a] - vr;
                    im[b] = im[a] - vi;
                    re[a] += vr;
                    im[a] += vi;
                }
            }
        }
    }
};

// ------------------------------------------------------------
//  Module
// ------------------------------------------------------------
struct Pharmakon : Module
{
    enum ParamId
    {
        // knobs (order matches PH_CTL)
        PARAM_IN_GAIN,
        PARAM_DRYWET,
        PARAM_DELAY_TIME,
        PARAM_FEEDBACK,
        PARAM_DETUNE,
        PARAM_DISTORTION,
        PARAM_LO_GAIN,
        PARAM_HI_GAIN,
        PARAM_BRIGHTNESS,
        PARAM_LPF,
        PARAM_MIX,
        PARAM_REV_DECAY,
        PARAM_SPACY_MOD,
        PARAM_SPACY_RATE,
        PARAM_WIDTH,
        PARAM_OUT_LEVEL,
        // attenuverters (same order)
        ATTN_FIRST,
        ATTN_LAST = ATTN_FIRST + PH_NUM_CTRL - 1,
        // switches
        PARAM_PINGPONG,
        PARAM_BYPASS,
        PARAM_FREEZE,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_LEFT,
        INPUT_RIGHT,
        CV_FIRST,
        CV_LAST = CV_FIRST + PH_NUM_CTRL - 1,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_LEFT,
        OUTPUT_RIGHT,
        OUTPUTS_LEN
    };
    enum LightId { LIGHTS_LEN };

    static_assert(PARAM_OUT_LEVEL == PH_NUM_CTRL - 1, "knob count mismatch");

    enum CVRange { BIPOLAR_M5_5, UNIPOLAR_0_10 };
    enum VisMode { VIS_WAVE, VIS_BARS, VIS_SPECTRUM, VIS_COUNT };

    // ---- settings (saved with the patch) ----
    CVRange cvRangeMode = BIPOLAR_M5_5;
    int   visMode  = VIS_SPECTRUM;
    bool  eqPost   = false;     // false: EQ on wet signal, true: EQ on whole output
    bool  softClip = true;      // output soft limiter
    float loFreq   = 200.f;
    float hiFreq   = 4000.f;

    // ---- DSP ----
    PHDelay delayL, delayR;
    PHOnePole brightL, brightR, outLpfL, outLpfR;
    PHReverb reverb;
    PHBiquad loL, loR, hiL, hiR;

    float dspSr = 0.f;
    bool  needSnap = true;
    int   ctrlCounter = 0;

    float lfoPhase = 0.f, spacyPhase = 0.f;
    float dcR = 0.999f, dcXL = 0.f, dcYL = 0.f, dcXR = 0.f, dcYR = 0.f;
    float kSmooth = 0.01f, kDelay = 0.001f;

    // smoothed parameters
    float sIn = 1.f, sWet = 0.5f, sDelay = 0.3f, sLo = 0.f, sHi = 0.f;
    float sWidth = 1.f, sOut = 1.f, sDist = 0.f;
    float sBypass = 0.f, sFreeze = 0.f, sPing = 0.f;

    // ---- display data (audio thread writes, UI thread reads; torn reads are cosmetic) ----
    float scopeMin[PH_SCOPE_N] = {};
    float scopeMax[PH_SCOPE_N] = {};
    int   scopeHead = 0;
    int   scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;
    float fftRing[PH_FFT_RING] = {};
    int   fftHead = 0;
    float dispDelay = 0.3f, dispWet = 0.5f, dispLo = 0.f, dispHi = 0.f;
    bool  dispBypass = false, dispFreeze = false;
    float clipTimer = 0.f;

    Pharmakon()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        for (int i = 0; i < PH_NUM_CTRL; ++i)
        {
            const PHCtlSpec& s = PH_CTL[i];
            configParam(i, s.lo, s.hi, s.def, s.label, s.unit, s.dispBase, s.dispMul);
            configParam(ATTN_FIRST + i, -1.f, 1.f, 0.f,
                        std::string(s.label) + " CV amount", "%", 0.f, 100.f);
            configInput(CV_FIRST + i, std::string(s.label) + " CV");
        }

        configSwitch(PARAM_PINGPONG, 0.f, 1.f, 0.f, "Ping-pong", {"Off", "On"});
        configSwitch(PARAM_BYPASS,   0.f, 1.f, 0.f, "Bypass",    {"Off", "On"});
        configSwitch(PARAM_FREEZE,   0.f, 1.f, 0.f, "Freeze",    {"Off", "On"});

        configInput(INPUT_LEFT,  "L in");
        configInput(INPUT_RIGHT, "R in");
        configOutput(OUTPUT_LEFT,  "L out");
        configOutput(OUTPUT_RIGHT, "R out");
        configBypass(INPUT_LEFT,  OUTPUT_LEFT);
        configBypass(INPUT_RIGHT, OUTPUT_RIGHT);
    }

    // (Re)build everything that depends on the sample rate. Called from process().
    void setupDSP(float sr)
    {
        dspSr = sr;
        delayL.init(2.5f, sr);
        delayR.init(2.5f, sr);
        reverb.init(sr);
        brightL.setCutoff(18000.f, sr);  brightL.z = 0.f;
        brightR.setCutoff(18000.f, sr);  brightR.z = 0.f;
        outLpfL.setCutoff(20000.f, sr);  outLpfL.z = 0.f;
        outLpfR.setCutoff(20000.f, sr);  outLpfR.z = 0.f;
        loL = PHBiquad(); loR = PHBiquad(); hiL = PHBiquad(); hiR = PHBiquad();
        dcR = 1.f - PH_TWO_PI * 10.f / sr;
        dcXL = dcYL = dcXR = dcYR = 0.f;
        kSmooth = 1.f - std::exp(-1.f / (0.010f * sr));
        kDelay  = 1.f - std::exp(-1.f / (0.050f * sr));
        ctrlCounter = 0;
        needSnap = true;
    }

    float getCtl(int i)
    {
        const PHCtlSpec& s = PH_CTL[i];
        float knob = params[i].getValue();
        float cv = inputs[CV_FIRST + i].getVoltage();
        float attn = params[ATTN_FIRST + i].getValue();

        float cvScaled = (cvRangeMode == UNIPOLAR_0_10) ? cv / 10.f : cv / 5.f;
        cvScaled = phClamp(cvScaled, -1.f, 1.f);

        // CV sweeps the whole range of the parameter at full attenuverter
        return phClamp(knob + cvScaled * attn * (s.hi - s.lo), s.lo, s.hi);
    }

    json_t* dataToJson() override
    {
        json_t* root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        json_object_set_new(root, "visMode",     json_integer(visMode));
        json_object_set_new(root, "eqPost",      json_boolean(eqPost));
        json_object_set_new(root, "softClip",    json_boolean(softClip));
        json_object_set_new(root, "loFreq",      json_real(loFreq));
        json_object_set_new(root, "hiFreq",      json_real(hiFreq));
        return root;
    }

    void dataFromJson(json_t* root) override
    {
        if (json_t* j = json_object_get(root, "cvRangeMode"))
            cvRangeMode = (CVRange) json_integer_value(j);
        if (json_t* j = json_object_get(root, "visMode"))
            visMode = (int) phClamp((float) json_integer_value(j), 0.f, (float) (VIS_COUNT - 1));
        if (json_t* j = json_object_get(root, "eqPost"))
            eqPost = json_boolean_value(j);
        if (json_t* j = json_object_get(root, "softClip"))
            softClip = json_boolean_value(j);
        if (json_t* j = json_object_get(root, "loFreq"))
            loFreq = phClamp((float) json_number_value(j), 40.f, 1000.f);
        if (json_t* j = json_object_get(root, "hiFreq"))
            hiFreq = phClamp((float) json_number_value(j), 1000.f, 12000.f);
    }

    void process(const ProcessArgs& args) override
    {
        const float sr = args.sampleRate;
        if (sr != dspSr)
            setupDSP(sr);

        // ---- parameters ----
        float c[PH_NUM_CTRL];
        for (int i = 0; i < PH_NUM_CTRL; ++i)
            c[i] = getCtl(i);

        const float tBypass = params[PARAM_BYPASS].getValue()   > 0.5f ? 1.f : 0.f;
        const float tFreeze = params[PARAM_FREEZE].getValue()   > 0.5f ? 1.f : 0.f;
        const float tPing   = params[PARAM_PINGPONG].getValue() > 0.5f ? 1.f : 0.f;

        if (needSnap)
        {
            sIn = c[PARAM_IN_GAIN];     sWet = c[PARAM_DRYWET];
            sDelay = c[PARAM_DELAY_TIME]; sLo = c[PARAM_LO_GAIN];
            sHi = c[PARAM_HI_GAIN];     sWidth = c[PARAM_WIDTH];
            sOut = c[PARAM_OUT_LEVEL];  sDist = c[PARAM_DISTORTION];
            sBypass = tBypass; sFreeze = tFreeze; sPing = tPing;
            needSnap = false;
        }

        sIn    += (c[PARAM_IN_GAIN]     - sIn)    * kSmooth;
        sWet   += (c[PARAM_DRYWET]      - sWet)   * kSmooth;
        sDelay += (c[PARAM_DELAY_TIME]  - sDelay) * kDelay;
        sLo    += (c[PARAM_LO_GAIN]     - sLo)    * kSmooth;
        sHi    += (c[PARAM_HI_GAIN]     - sHi)    * kSmooth;
        sWidth += (c[PARAM_WIDTH]       - sWidth) * kSmooth;
        sOut   += (c[PARAM_OUT_LEVEL]   - sOut)   * kSmooth;
        sDist  += (c[PARAM_DISTORTION]  - sDist)  * kSmooth;
        sBypass += (tBypass - sBypass) * kSmooth;
        sFreeze += (tFreeze - sFreeze) * kSmooth;
        sPing   += (tPing   - sPing)   * kSmooth;

        // ---- LFOs ----
        lfoPhase += PH_TWO_PI * 0.5f / sr;
        if (lfoPhase >= PH_TWO_PI) lfoPhase -= PH_TWO_PI;
        float lfoSinL = std::sin(lfoPhase);
        float lfoSinR = std::cos(lfoPhase);   // 90 degrees apart for stereo movement

        spacyPhase += PH_TWO_PI * c[PARAM_SPACY_RATE] / sr;
        if (spacyPhase >= PH_TWO_PI) spacyPhase -= PH_TWO_PI;
        float spacyLfo = (std::sin(spacyPhase) + 1.f) * 0.5f;

        // ---- control-rate coefficient updates (every 16 samples) ----
        if ((ctrlCounter++ & 15) == 0)
        {
            float bright = c[PARAM_BRIGHTNESS];
            float bc = phMapFreq(bright, 200.f, 18000.f);
            brightL.setCutoff(bc, sr);
            brightR.setCutoff(bc, sr);
            reverb.setDecay(c[PARAM_REV_DECAY], bright);

            float lpfC = phMapFreq(c[PARAM_LPF], 50.f, 20000.f);
            float sm = c[PARAM_SPACY_MOD];
            if (sm > 0.01f)
                lpfC = phClamp(lpfC * (0.5f + spacyLfo * sm * 1.5f), 30.f, 20000.f);
            outLpfL.setCutoff(lpfC, sr);
            outLpfR.setCutoff(lpfC, sr);

            loL.setLowShelf(loFreq, sLo, sr);   loR.copyCoeffs(loL);
            hiL.setHighShelf(hiFreq, sHi, sr);  hiR.copyCoeffs(hiL);
        }

        // ---- inputs (mono normalled to both sides) ----
        const bool lc = inputs[INPUT_LEFT].isConnected();
        const bool rc = inputs[INPUT_RIGHT].isConnected();
        float rawL = lc ? inputs[INPUT_LEFT].getVoltageSum() : 0.f;
        float rawR = rc ? inputs[INPUT_RIGHT].getVoltageSum() : 0.f;
        if (lc && !rc) rawR = rawL;
        if (rc && !lc) rawL = rawR;

        float dryL = rawL * sIn;
        float dryR = rawR * sIn;

        // DC blocker on the effect path
        float fxL = dryL - dcXL + dcR * dcYL;  dcXL = dryL;  dcYL = fxL;
        float fxR = dryR - dcXR + dcR * dcYR;  dcXR = dryR;  dcYR = fxR;

        // ---- distortion (level preserving, blended in) ----
        const float drive = 1.f + sDist * 10.f;
        const float satAmt = phClamp(sDist * 10.f, 0.f, 1.f);
        float dsL = 5.f * std::tanh(fxL * 0.2f * drive);
        float dsR = 5.f * std::tanh(fxR * 0.2f * drive);
        float dL = fxL + (dsL - fxL) * satAmt;
        float dR = fxR + (dsR - fxR) * satAmt;

        // ---- delay with feedback ----
        float dOutL = delayL.read(sDelay);
        float dOutR = delayR.read(sDelay);
        float filtL = brightL.process(dOutL);
        float filtR = brightR.process(dOutR);

        float fb = c[PARAM_FEEDBACK] + (0.995f - c[PARAM_FEEDBACK]) * sFreeze;
        float inScale = 1.f - sFreeze;
        float cL = phSoftLimit(filtL * fb);
        float cR = phSoftLimit(filtR * fb);

        // normal stereo feedback vs ping-pong (mono in -> L, R fed only from L)
        float wNL = dL * inScale + cL;
        float wNR = dR * inScale + cR;
        float mono = 0.5f * (dL + dR) * inScale;
        float wPL = mono + cR;
        float wPR = cL;
        delayL.write(wNL + (wPL - wNL) * sPing);
        delayR.write(wNR + (wPR - wNR) * sPing);

        // detune: chorus-like modulated tap blended with the straight tap
        float detMs = c[PARAM_DETUNE] * 5.f;
        float tModL = phClamp(sDelay + detMs * lfoSinL * 0.001f, 0.002f, 2.4f);
        float tModR = phClamp(sDelay + detMs * lfoSinR * 0.001f, 0.002f, 2.4f);
        float dlyL = 0.5f * (dOutL + delayL.read(tModL));
        float dlyR = 0.5f * (dOutR + delayR.read(tModR));

        // ---- reverb ----
        float revL, revR;
        reverb.process(dlyL, dlyR, revL, revR);

        float mix = c[PARAM_MIX];
        float wetL = dlyL * (1.f - mix) + revL * mix;
        float wetR = dlyR * (1.f - mix) + revR * mix;

        // ---- stereo width (M/S) ----
        float mid  = 0.5f * (wetL + wetR);
        float side = 0.5f * (wetL - wetR) * sWidth;
        wetL = mid + side;
        wetR = mid - side;

        // ---- EQ on wet path ----
        if (!eqPost)
        {
            wetL = hiL.process(loL.process(wetL));
            wetR = hiR.process(loR.process(wetR));
        }

        // ---- spacy lowpass ----
        wetL = outLpfL.process(wetL);
        wetR = outLpfR.process(wetR);

        // ---- dry/wet ----
        float outL = dryL * (1.f - sWet) + wetL * sWet;
        float outR = dryR * (1.f - sWet) + wetR * sWet;

        // ---- EQ on whole output ----
        if (eqPost)
        {
            outL = hiL.process(loL.process(outL));
            outR = hiR.process(loR.process(outR));
        }

        outL *= sOut;
        outR *= sOut;

        // ---- click-free bypass (raw input, tails keep running) ----
        outL = outL * (1.f - sBypass) + rawL * sBypass;
        outR = outR * (1.f - sBypass) + rawR * sBypass;

        // ---- output protection ----
        if (std::fabs(outL) > 10.f || std::fabs(outR) > 10.f)
            clipTimer = 0.3f;
        else
            clipTimer = std::max(0.f, clipTimer - args.sampleTime);

        if (softClip)
        {
            outL = phSoftLimit(outL);
            outR = phSoftLimit(outR);
        }

        outputs[OUTPUT_LEFT].setVoltage(outL);
        outputs[OUTPUT_RIGHT].setVoltage(outR);

        // ---- display data ----
        dispDelay = c[PARAM_DELAY_TIME];
        dispWet = c[PARAM_DRYWET];
        dispLo = c[PARAM_LO_GAIN];
        dispHi = c[PARAM_HI_GAIN];
        dispBypass = tBypass > 0.5f;
        dispFreeze = tFreeze > 0.5f;

        float decay = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(outL), peakL * decay);
        peakR = std::max(std::fabs(outR), peakR * decay);

        float monoOut = 0.5f * (outL + outR);

        fftRing[fftHead] = monoOut;
        fftHead = (fftHead + 1) & (PH_FFT_RING - 1);

        if (scopeCount == 0)
            scopeBMin = scopeBMax = monoOut;
        else
        {
            scopeBMin = std::min(scopeBMin, monoOut);
            scopeBMax = std::max(scopeBMax, monoOut);
        }
        int bucket = std::max(1, (int) (sr * 0.0025f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % PH_SCOPE_N;
        }
    }
};

// ============================================================
//  UI (anonymous namespace so helper names cannot clash with
//  same-named helpers in other module files)
// ============================================================
namespace {

// ------------------------------------------------------------
// Theme (same palette as the other AGMM modules)
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

    nvgBeginPath(vg);
    nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 7.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                       nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
    nvgFill(vg);
    nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
                                         nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
    nvgStrokeWidth(vg, 1.2f);
    nvgStroke(vg);

    const float s = 2.2f;
    float tx = x + 7.f, ty = y + h / 2.f - 3.f;
    nvgBeginPath(vg);
    nvgRect(vg, tx + s,       ty,            s, s);
    nvgRect(vg, tx,           ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);

    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
             NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct PHPanelBackground : TransparentWidget {
    PHPanelBackground() { box.size = Vec(PH_PANEL_W, PH_PANEL_H); }

    void plate(NVGcontext* vg, float cx, float ry) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - PH_CELL_W / 2.f, ry - 17.f, PH_CELL_W, PH_CELL_H, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = PH_PANEL_W - 14.f, cx = PH_PANEL_W / 2.f;
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

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PH_PANEL_W, PH_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PH_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) PH_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, PH_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, PH_PANEL_W - 1.f, PH_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, PH_PANEL_W / 2.f, 13.f);
        drawText(vg, PH_PANEL_W / 2.f, 31.f, "PHARMAKON", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = PH_DISP_X - 5.f, by = PH_DISP_Y - 5.f;
        float bw = PH_DISP_W + 10.f, bh = PH_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // Control cells: 16 knobs + FREEZE + CV RANGE (6 columns x 3 rows)
        for (int i = 0; i < 18; ++i) {
            int row = i / 6, col = i % 6;
            float cx = phColX(col), ry = PH_ROW[row];
            plate(vg, cx, ry);
            if (i < PH_NUM_CTRL)
                drawText(vg, cx, ry + 21.f, PH_CTL[i].name, 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            else if (i == PH_NUM_CTRL)   // FREEZE
                drawText(vg, cx, ry + 44.f, "FREEZE", 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // I/O strip
        section(vg, 314.f, "I / O");
        const char* io[6] = {"L IN", "R IN", "PING-PONG", "BYPASS", "L OUT", "R OUT"};
        for (int i = 0; i < 6; ++i) {
            float cx = phColX(i);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 28.f, 322.f, 56.f, 42.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, cx, 330.f, io[i], 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Display: WAVE / BARS / SPECTRUM (click to cycle)
// ------------------------------------------------------------
const char* PH_VIS_NAMES[3] = {"WAVE", "BARS", "SPECTRUM"};

struct PHDisplayWidget : Widget {
    Pharmakon* module = nullptr;

    PHFFT fft;
    float frame[PH_FFT_N];
    float re[PH_FFT_N], im[PH_FFT_N];
    float binDb[PH_FFT_N / 2];      // smoothed magnitude per bin (dB re 5 V)
    float barLvl[PH_BARS];
    float barPeak[PH_BARS];

    PHDisplayWidget(Pharmakon* m) : module(m) {
        box.pos = Vec(PH_DISP_X, PH_DISP_Y);
        box.size = Vec(PH_DISP_W, PH_DISP_H);
        for (int i = 0; i < PH_FFT_N; ++i) frame[i] = re[i] = im[i] = 0.f;
        for (int i = 0; i < PH_FFT_N / 2; ++i) binDb[i] = -120.f;
        for (int i = 0; i < PH_BARS; ++i) barLvl[i] = barPeak[i] = 0.f;
    }

    static float scopeW() { return PH_DISP_W - 16.f; }

    void onButton(const ButtonEvent& e) override {
        if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            module->visMode = (module->visMode + 1) % Pharmakon::VIS_COUNT;
            e.consume(this);
            return;
        }
        Widget::onButton(e);
    }

    // ---- analysis ----
    float sampleRate() const { return (module && module->dspSr > 0.f) ? module->dspSr : 48000.f; }

    void updateSpectrum() {
        const int half = PH_FFT_N / 2;
        if (module) {
            int start = (module->fftHead - PH_FFT_N + PH_FFT_RING) & (PH_FFT_RING - 1);
            for (int i = 0; i < PH_FFT_N; ++i)
                frame[i] = module->fftRing[(start + i) & (PH_FFT_RING - 1)];
            fft.forward(frame, re, im);
            for (int k = 1; k < half; ++k) {
                float mag = std::sqrt(re[k] * re[k] + im[k] * im[k]);
                float amp = 4.f * mag / (float) PH_FFT_N;               // Hann-window corrected
                float db = 20.f * std::log10(amp / 5.f + 1e-7f);
                db = phClamp(db, -120.f, 12.f);
                float s = binDb[k];
                // fast attack, slower release
                binDb[k] = (db > s) ? s + (db - s) * 0.6f : s + (db - s) * 0.14f;
            }
        } else {
            // preview data for the module browser
            float binHz = 48000.f / PH_FFT_N;
            for (int k = 1; k < half; ++k) {
                float f = k * binHz;
                float db = -14.f - 17.f * std::log10(1.f + f / 150.f)
                           - 5.f * std::fabs(std::sin(f * 0.0017f)) ;
                binDb[k] = db - PH_TILT * std::log2(std::max(f, 30.f) / 1000.f) * 0.3f;
            }
        }
    }

    float levelAt(float fLo, float fHi, float binHz) const {
        const int maxB = PH_FFT_N / 2 - 2;
        float b0 = fLo / binHz, b1 = fHi / binHz;
        if (b1 - b0 < 1.f) {
            float cc = phClamp(0.5f * (b0 + b1), 1.f, (float) maxB);
            int i = (int) cc;
            float frac = cc - (float) i;
            return binDb[i] * (1.f - frac) + binDb[i + 1] * frac;
        }
        int i0 = std::max(1, (int) std::ceil(b0));
        int i1 = std::min(maxB, (int) std::floor(b1));
        if (i1 < i0) i1 = i0;
        float m = -200.f;
        for (int i = i0; i <= i1; ++i)
            m = std::max(m, binDb[i]);
        return m;
    }

    float bandNorm(float fLo, float fHi, float binHz) const {
        float fc = std::sqrt(fLo * fHi);
        float db = levelAt(fLo, fHi, binHz) + PH_TILT * std::log2(fc / 1000.f);
        return phClamp((db - PH_DB_FLOOR) / (-PH_DB_FLOOR), 0.f, 1.f);
    }

    // ---- views ----
    void drawWave(NVGcontext* vg, float sw) {
        const float mid = PH_DISP_H / 2.f;
        const float amp = mid - 2.f;
        const float colW = sw / (float) PH_SCOPE_N;

        for (int pass = 0; pass < 2; ++pass) {   // 0: glow, 1: crisp
            nvgBeginPath(vg);
            for (int i = 0; i < PH_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % PH_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / PH_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - phClamp(vmax / 5.f, -1.f, 1.f) * amp;
                float y1 = mid - phClamp(vmin / 5.f, -1.f, 1.f) * amp;
                float h = std::max(y1 - y0, 1.f);
                float x = i * colW;
                if (pass == 0)
                    nvgRect(vg, x - 0.5f, y0 - 1.5f, colW + 1.f, h + 3.f);
                else
                    nvgRect(vg, x, y0, std::max(colW - 0.4f, 1.f), h);
            }
            nvgFillColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 32) : nvgRGB(0x8a, 0xee, 0xff));
            nvgFill(vg);
        }
    }

    void drawFreqGrid(NVGcontext* vg, float sw) {
        const float fs[3] = {100.f, 1000.f, 10000.f};
        const char* ls[3] = {"100", "1K", "10K"};
        const float lr = std::log(PH_FMAX / PH_FMIN);
        nvgBeginPath(vg);
        for (int i = 0; i < 3; ++i) {
            float x = std::floor(sw * std::log(fs[i] / PH_FMIN) / lr) + 0.5f;
            nvgMoveTo(vg, x, 0);
            nvgLineTo(vg, x, PH_DISP_H);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        for (int i = 0; i < 3; ++i) {
            float x = sw * std::log(fs[i] / PH_FMIN) / lr;
            drawText(vg, x + 2.f, PH_DISP_H - 5.f, ls[i], 6.f, nvgRGBA(0x7a, 0x85, 0x94, 200),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.5f);
        }
    }

    void drawBars(NVGcontext* vg, float sw) {
        const float binHz = sampleRate() / PH_FFT_N;
        const float ratio = PH_FMAX / PH_FMIN;
        const float slot = sw / (float) PH_BARS;
        const float barW = slot - 2.f;
        const float H = PH_DISP_H;

        drawFreqGrid(vg, sw);

        nvgBeginPath(vg);
        for (int b = 0; b < PH_BARS; ++b) {
            float fLo = PH_FMIN * std::pow(ratio, (float) b / PH_BARS);
            float fHi = PH_FMIN * std::pow(ratio, (float) (b + 1) / PH_BARS);
            float n = bandNorm(fLo, fHi, binHz);
            barLvl[b] = (n > barLvl[b]) ? n : barLvl[b] * 0.86f + n * 0.14f;
            barPeak[b] = std::max(barLvl[b], barPeak[b] - 0.010f);
            float h = barLvl[b] * (H - 3.f);
            if (h > 0.5f)
                nvgRect(vg, b * slot + 1.f, H - 1.f - h, barW, h);
        }
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, H, 0, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // peak caps
        nvgBeginPath(vg);
        for (int b = 0; b < PH_BARS; ++b) {
            float y = H - 1.f - barPeak[b] * (H - 3.f);
            if (barPeak[b] > 0.02f)
                nvgRect(vg, b * slot + 1.f, y - 1.f, barW, 1.5f);
        }
        nvgFillColor(vg, nvgRGBA(0xd8, 0xf8, 0xff, 230));
        nvgFill(vg);
    }

    void drawSpectrum(NVGcontext* vg, float sw) {
        const float binHz = sampleRate() / PH_FFT_N;
        const float ratio = PH_FMAX / PH_FMIN;
        const float H = PH_DISP_H;
        float xs[PH_SPEC_COLS], ys[PH_SPEC_COLS];

        drawFreqGrid(vg, sw);

        for (int c = 0; c < PH_SPEC_COLS; ++c) {
            float fLo = PH_FMIN * std::pow(ratio, (float) c / PH_SPEC_COLS);
            float fHi = PH_FMIN * std::pow(ratio, (float) (c + 1) / PH_SPEC_COLS);
            float n = bandNorm(fLo, fHi, binHz);
            xs[c] = (c + 0.5f) * sw / PH_SPEC_COLS;
            ys[c] = H - 1.f - n * (H - 3.f);
        }

        // filled area
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0.f, H);
        nvgLineTo(vg, 0.f, ys[0]);
        for (int c = 0; c < PH_SPEC_COLS; ++c)
            nvgLineTo(vg, xs[c], ys[c]);
        nvgLineTo(vg, sw, ys[PH_SPEC_COLS - 1]);
        nvgLineTo(vg, sw, H);
        nvgClosePath(vg);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H,
                                           nvgRGBA(0x3d, 0xe0, 0xff, 120), nvgRGBA(0xb0, 0x5a, 0xff, 8)));
        nvgFill(vg);

        // glow + crisp outline
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, ys[0]);
            for (int c = 0; c < PH_SPEC_COLS; ++c)
                nvgLineTo(vg, xs[c], ys[c]);
            nvgLineTo(vg, sw, ys[PH_SPEC_COLS - 1]);
            nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 50) : nvgRGB(0x8a, 0xee, 0xff));
            nvgStrokeWidth(vg, pass == 0 ? 3.f : 1.1f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
        }
    }

    // ---- main ----
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PH_DISP_W, PH_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, PH_DISP_W / 2.f, PH_DISP_H / 2.f, 20.f, PH_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        // Faint grid
        nvgBeginPath(vg);
        for (float x = 0.f; x <= scopeW(); x += 28.f) {
            nvgMoveTo(vg, x + 0.5f, 0);
            nvgLineTo(vg, x + 0.5f, PH_DISP_H);
        }
        nvgMoveTo(vg, 0, PH_DISP_H / 2.f + 0.5f);
        nvgLineTo(vg, scopeW(), PH_DISP_H / 2.f + 0.5f);
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
        const float sw = scopeW();
        int mode = module ? module->visMode : (int) Pharmakon::VIS_SPECTRUM;

        if (mode != Pharmakon::VIS_WAVE)
            updateSpectrum();

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, PH_DISP_H);
        if (mode == Pharmakon::VIS_WAVE)
            drawWave(vg, sw);
        else if (mode == Pharmakon::VIS_BARS)
            drawBars(vg, sw);
        else
            drawSpectrum(vg, sw);
        nvgResetScissor(vg);
        nvgRestore(vg);

        // Level meters (L / R)
        for (int ch = 0; ch < 2; ++ch) {
            float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 3.f : 2.4f);
            float lvl = std::min(peak / 10.f, 1.f);
            float x = sw + 3.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 1.f, 4.f, PH_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (PH_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, PH_DISP_H - 1.f - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, PH_DISP_H, 0, 0,
                                                   nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) PH_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, PH_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PH_DISP_W, PH_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PH_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[48];
        float dly = module ? module->dispDelay : 0.3f;
        float dw  = module ? module->dispWet : 0.5f;
        float lo  = module ? module->dispLo : 0.f;
        float hi  = module ? module->dispHi : 0.f;
        NVGcolor ro = nvgRGBA(0x3d, 0xe0, 0xff, 190);

        snprintf(buf, sizeof(buf), "DLY %4.0f MS", dly * 1000.f);
        drawText(vg, 4.f, 7.f, buf, 7.f, ro, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "LO %+.1f  HI %+.1f", lo, hi);
        drawText(vg, sw / 2.f, 7.f, buf, 7.f, ro, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "WET %3.0f%%", dw * 100.f);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, ro, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        // Mode name (bottom-left)
        drawText(vg, 4.f, PH_DISP_H - 5.f, PH_VIS_NAMES[mode], 6.f,
                 nvgRGBA(0xb0, 0x5a, 0xff, 220), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

        // Status flags (bottom-right)
        if (module) {
            char st[32] = "";
            if (module->dispFreeze) strcat(st, "FRZ ");
            if (module->dispBypass) strcat(st, "BYP ");
            if (module->clipTimer > 0.f) strcat(st, "CLIP");
            if (st[0])
                drawText(vg, sw - 4.f, PH_DISP_H - 5.f, st, 6.f,
                         module->clipTimer > 0.f ? nvgRGBA(0xff, 0x50, 0x60, 255) : nvgRGBA(0xff, 0xc8, 0x50, 230),
                         NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        }

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// CV range cell (click to toggle)
// ------------------------------------------------------------
struct PHCvRangeWidget : Widget {
    Pharmakon* module = nullptr;

    PHCvRangeWidget(Pharmakon* m, float cx, float ry) : module(m) {
        box.pos = Vec(cx - PH_CELL_W / 2.f, ry - 17.f);
        box.size = Vec(PH_CELL_W, PH_CELL_H);
    }

    void onButton(const ButtonEvent& e) override {
        if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            module->cvRangeMode = (module->cvRangeMode == Pharmakon::BIPOLAR_M5_5)
                                      ? Pharmakon::UNIPOLAR_0_10
                                      : Pharmakon::BIPOLAR_M5_5;
            e.consume(this);
            return;
        }
        Widget::onButton(e);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        bool bipolar = module ? (module->cvRangeMode == Pharmakon::BIPOLAR_M5_5) : true;

        drawText(vg, box.size.x / 2.f, 21.f + 17.f, "CV RANGE", 8.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        float sx = 6.f, sy = 10.f, sw = box.size.x - 12.f, sh = 24.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, sx, sy, sw, sh, 4.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        drawText(vg, box.size.x / 2.f, sy + sh / 2.f, bipolar ? "-5/+5V" : "0/+10V", 10.f,
                 colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, sx + 4.f, 44.f, sw - 8.f, 2.f, 1.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgFill(vg);
        nvgBeginPath(vg);
        if (bipolar)
            nvgRoundedRect(vg, box.size.x / 2.f - 12.f, 44.f, 24.f, 2.f, 1.f);
        else
            nvgRoundedRect(vg, sx + 4.f, 44.f, 24.f, 2.f, 1.f);
        nvgFillColor(vg, nvgRGB(0xb0, 0x5a, 0xff));
        nvgFill(vg);

        drawText(vg, box.size.x / 2.f, 58.f, "CLICK", 7.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// Context-menu helper
// ------------------------------------------------------------
struct PHChoiceItem : MenuItem {
    std::function<bool()> isOn;
    std::function<void()> action;
    void onAction(const ActionEvent& e) override { action(); }
    void step() override {
        rightText = isOn() ? "✔" : "";
        MenuItem::step();
    }
};

void addChoice(Menu* menu, const std::string& label,
               std::function<bool()> isOn, std::function<void()> action) {
    PHChoiceItem* item = createMenuItem<PHChoiceItem>(label);
    item->isOn = isOn;
    item->action = action;
    menu->addChild(item);
}

} // namespace

// ============================================================
//  Module widget
// ============================================================
struct PharmakonWidget : ModuleWidget {
    PharmakonWidget(Pharmakon* module) {
        setModule(module);
        box.size = Vec(PH_PANEL_W, PH_PANEL_H);

        addChild(new PHPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PHDisplayWidget(module));

        // 16 controls: knob on top, CV jack (left) and attenuverter (right) below
        for (int i = 0; i < PH_NUM_CTRL; ++i) {
            int row = i / 6, col = i % 6;
            float cx = phColX(col), ry = PH_ROW[row];
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, ry), module, i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 16.f, ry + 40.f), module,
                                                         Pharmakon::CV_FIRST + i));
            addParam(createParamCentered<Trimpot>(Vec(cx + 16.f, ry + 40.f), module,
                                                  Pharmakon::ATTN_FIRST + i));
        }

        // Row 3, col 4: FREEZE switch.  Row 3, col 5: CV range selector.
        addParam(createParamCentered<CKSS>(Vec(phColX(4), PH_ROW[2] + 12.f), module, Pharmakon::PARAM_FREEZE));
        addChild(new PHCvRangeWidget(module, phColX(5), PH_ROW[2]));

        // I/O strip
        addInput(createInputCentered<DarkPJ301MPort>(Vec(phColX(0), PH_PORT_Y), module, Pharmakon::INPUT_LEFT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(phColX(1), PH_PORT_Y), module, Pharmakon::INPUT_RIGHT));
        addParam(createParamCentered<CKSS>(Vec(phColX(2), PH_PORT_Y + 3.f), module, Pharmakon::PARAM_PINGPONG));
        addParam(createParamCentered<CKSS>(Vec(phColX(3), PH_PORT_Y + 3.f), module, Pharmakon::PARAM_BYPASS));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(phColX(4), PH_PORT_Y), module, Pharmakon::OUTPUT_LEFT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(phColX(5), PH_PORT_Y), module, Pharmakon::OUTPUT_RIGHT));
    }

    void appendContextMenu(Menu* menu) override {
        Pharmakon* m = dynamic_cast<Pharmakon*>(this->module);
        assert(m);

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));
        addChoice(menu, "-5V to +5V (bipolar)",
                  [m]() { return m->cvRangeMode == Pharmakon::BIPOLAR_M5_5; },
                  [m]() { m->cvRangeMode = Pharmakon::BIPOLAR_M5_5; });
        addChoice(menu, "0V to +10V (unipolar)",
                  [m]() { return m->cvRangeMode == Pharmakon::UNIPOLAR_0_10; },
                  [m]() { m->cvRangeMode = Pharmakon::UNIPOLAR_0_10; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Display"));
        for (int v = 0; v < Pharmakon::VIS_COUNT; ++v)
            addChoice(menu, PH_VIS_NAMES[v],
                      [m, v]() { return m->visMode == v; },
                      [m, v]() { m->visMode = v; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("EQ Position"));
        addChoice(menu, "Wet signal only",
                  [m]() { return !m->eqPost; },
                  [m]() { m->eqPost = false; });
        addChoice(menu, "Whole output (dry + wet)",
                  [m]() { return m->eqPost; },
                  [m]() { m->eqPost = true; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Low Shelf Frequency"));
        const float loF[4] = {100.f, 200.f, 300.f, 500.f};
        for (int i = 0; i < 4; ++i) {
            float f = loF[i];
            addChoice(menu, std::to_string((int) f) + " Hz",
                      [m, f]() { return m->loFreq == f; },
                      [m, f]() { m->loFreq = f; });
        }

        menu->addChild(createMenuLabel("High Shelf Frequency"));
        const float hiF[4] = {2500.f, 4000.f, 6000.f, 8000.f};
        for (int i = 0; i < 4; ++i) {
            float f = hiF[i];
            addChoice(menu, std::to_string((int) f) + " Hz",
                      [m, f]() { return m->hiFreq == f; },
                      [m, f]() { m->hiFreq = f; });
        }

        menu->addChild(new MenuSeparator);
        addChoice(menu, "Output soft limiter (10 V)",
                  [m]() { return m->softClip; },
                  [m]() { m->softClip = !m->softClip; });
    }
};

Model* modelPharmakon = createModel<Pharmakon, PharmakonWidget>("Pharmakon");