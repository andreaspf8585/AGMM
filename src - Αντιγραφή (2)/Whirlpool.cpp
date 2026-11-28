#include "GitLoad.hpp"
// ============================================================================
// Whirlpool — FDN reverb (20 HP Rack module)
// ----------------------------------------------------------------------------
// Overview:
//   * A stereo / polyphonic reverb built around an 8-line Feedback Delay
//     Network (FDN) with a Hadamard feedback matrix. Each voice owns its
//     own independent engine (up to MAX_CHANNELS = 16).
//   * Five controls, each with a CV jack and attenuverter:
//       REPLACE    shortens the tail (crossfades toward smaller RT60)
//       BRIGHTNESS damping cutoff (one-pole lowpass in each line)
//       DETUNE     modulation depth of the read positions (chorusing/wow)
//       BIGNESS    room size + decay (drives delay length + RT60)
//       MIX        dry/wet (equal-power)
//   * IN/OUT stereo switches; poly otherwise. CLEAR fades the wet path
//     out, resets every engine, then fades back in.
//   * Display modes: WAVE / BARS / SPECTRUM (click tabs / plot, or use the
//     context menu). BARS shows the 8 FDN line levels with delay times.
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include "Knob_arcs.hpp"
#include <vector>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int WH_SCOPE_N = 222;              // waveform history bins (1 px each)
static const int WH_SPEC_N = 4096;              // FFT size
static const int WH_BANDS = 111;                // spectrum bands (2 px each)
static const float WH_W = RACK_GRID_WIDTH * 20; // 300
static const float WH_H = RACK_GRID_HEIGHT;     // 380
// Display (260 x 124) sits just under the title; the bezel adds 4 px around it.
static const float WH_DX = 20.f, WH_DY = 26.f, WH_DW = 260.f, WH_DH = 124.f;
static float whCol(int i) { return 38.f + 56.f * (float)i; }
static const float WH_CELL_W = 52.f; // every cell is 52 wide (4 px gaps)
// Row A: 5 knob cells, 52 x 88. Label +10, knob +33, CV jack / attenuverter +59, captions +80.
static const float WH_RA = 162.f, WH_RAH = 88.f;
static const float WH_KNOB_DY = 33.f, WH_SUB_DY = 59.f, WH_SUB_DX = 13.f, WH_CAP_DY = 80.f;
// Row B: stereo switches + clear, 52 x 34 (control at the left, caption to its right).
static const float WH_RB = 258.f, WH_RBH = 34.f;
static const float WH_BTN_DX = -14.f;
// Row C: jack cells, 52 x 46, label +10, jack +30 (ends at y = 346).
static const float WH_JTOP = 300.f, WH_JH = 46.f, WH_JDY = 30.f;

// Plot area inside the display (display-local coordinates).
static const float WD_PX = 8.f, WD_PY = 18.f, WD_PW = 222.f, WD_PH = 90.f;
static const float WD_TABX[3] = {6.f, 46.f, 86.f};
static const float WD_TABW[3] = {36.f, 36.f, 46.f};

// ------------------------------------------------------------
// Reverb engine: 8-line feedback delay network (FDN)
//   stereo input -> early reflections + 4-stage allpass diffusion per side
//   -> 8 modulated delay lines (4-point Hermite read) with per-line RT60 gain
//      and one-pole damping, mixed by an orthonormal 8x8 Hadamard matrix
//   -> decorrelated stereo taps + DC blocker
//
// The engine is deliberately small and constant-time per sample; all
// per-sample control data is passed via `Shared`, computed once by the
// module and reused for every voice.
// ------------------------------------------------------------
namespace whengine
{

    static const int N = 8;
    // Prime-ish base delay lengths (samples @ 48 kHz) => mutually incommensurate
    // modal density, scaled by the current `sizeS`.
    static const float BASE[N] = {1117.f, 1327.f, 1601.f, 1861.f, 2143.f, 2423.f, 2713.f, 2999.f};
    // Walsh-pattern L / R output tap signs: orthogonal => decorrelated L / R.
    static const float PL[N] = {1.f, 1.f, -1.f, -1.f, 1.f, 1.f, -1.f, -1.f};
    static const float PR[N] = {1.f, -1.f, 1.f, -1.f, -1.f, 1.f, -1.f, 1.f};
    // Early reflection tap times (ms) and gains, alternating L / R.
    static const float ER_MS[N] = {4.3f, 8.9f, 13.7f, 17.9f, 23.5f, 29.1f, 36.7f, 43.3f};
    static const float ER_G[N] = {0.80f, -0.70f, 0.60f, -0.55f, 0.45f, -0.40f, 0.32f, -0.28f};
    // Allpass diffuser lengths, slightly different for L and R.
    static const int AP_L[4] = {89, 131, 223, 337};
    static const int AP_R[4] = {97, 149, 241, 359};

    // Per-sample control data shared by every voice (identical for all channels).
    struct Shared
    {
        float delay[N] = {};   // current delay lengths (samples)
        float mod[N] = {};     // current modulation offsets (samples)
        float gain[N] = {};    // per-line feedback gain from RT60
        float erTap[N] = {};   // early reflection tap times (samples)
        float lpCoef = 0.5f;   // damping one-pole coefficient
        float inGain = 0.3f;   // input injection gain (depends on RT60)
        float erLevel = 0.28f; // early reflections blend level
    };

    // Simple 4-sample allpass diffuser.
    struct Allpass
    {
        std::vector<float> buf;
        int pos = 0;
        void resize(int n)
        {
            buf.assign(n, 0.f);
            pos = 0;
        }
        void clear()
        {
            std::fill(buf.begin(), buf.end(), 0.f);
            pos = 0;
        }
        float process(float x, float g)
        {
            float d = buf[pos];
            float v = x - g * d;
            buf[pos] = v;
            if (++pos >= (int)buf.size())
                pos = 0;
            return d + g * v;
        }
    };

    // 4-point Hermite interpolation for a fractional delay read from a ring
    // buffer (`mask` must be size-1). Delays are smoothly modulated so the
    // read never "snaps" between samples.
    inline float readHermite(const std::vector<float> &b, int w, float d, int mask)
    {
        float pos = (float)w - d;
        int i0 = (int)std::floor(pos);
        float f = pos - (float)i0;
        float ym1 = b[(i0 - 1) & mask];
        float y0 = b[i0 & mask];
        float y1 = b[(i0 + 1) & mask];
        float y2 = b[(i0 + 2) & mask];
        float c1 = 0.5f * (y1 - ym1);
        float c2 = ym1 - 2.5f * y0 + 2.f * y1 - 0.5f * y2;
        float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
        return ((c3 * f + c2) * f + c1) * f + y0;
    }

    // In-place 8-point Hadamard transform, scaled by 1/sqrt(8) so the matrix
    // is orthonormal (energy preserving => stable feedback).
    inline void hadamard8(float *x)
    {
        for (int i = 0; i < 8; i += 2)
        {
            float a = x[i], b = x[i + 1];
            x[i] = a + b;
            x[i + 1] = a - b;
        }
        for (int i = 0; i < 8; i += 4)
            for (int j = 0; j < 2; ++j)
            {
                float a = x[i + j], b = x[i + j + 2];
                x[i + j] = a + b;
                x[i + j + 2] = a - b;
            }
        for (int j = 0; j < 4; ++j)
        {
            float a = x[j], b = x[j + 4];
            x[j] = a + b;
            x[j + 4] = a - b;
        }
        for (int i = 0; i < 8; ++i)
            x[i] *= 0.35355339f;
    }

    // One reverb voice (independent state).
    struct Engine
    {
        std::vector<float> line[N]; // 8 delay lines (power-of-two size ring)
        int mask = 0, w = 0;        // ring mask and write head
        float lp[N] = {};           // per-line one-pole damping states
        float lineAbs[N] = {};      // |y| per line (peaks for the display)

        std::vector<float> er; // early reflection ring
        int erMask = 0, erW = 0;

        Allpass apL[4], apR[4]; // input diffusion

        float dcXL = 0.f, dcYL = 0.f, dcXR = 0.f, dcYR = 0.f; // DC blockers

        // --------------------------------------------------------
        // Allocate ring buffers for a given sample rate. Buffers are
        // rounded up to the next power of two so the ring mask trick
        // works; all voices must use the same `sr`.
        // --------------------------------------------------------
        void init(float sr)
        {
            float srf = sr / 48000.f;
            int need = (int)(2999.f * 2.0f * srf + 20.f * srf + 64);
            int len = 1;
            while (len < need)
                len <<= 1;
            mask = len - 1;
            for (int i = 0; i < N; ++i)
                line[i].assign(len, 0.f);

            int erNeed = (int)(0.13f * sr);
            int erLen = 1;
            while (erLen < erNeed)
                erLen <<= 1;
            erMask = erLen - 1;
            er.assign(erLen, 0.f);

            for (int i = 0; i < 4; ++i)
            {
                apL[i].resize(std::max(8, (int)(AP_L[i] * srf)));
                apR[i].resize(std::max(8, (int)(AP_R[i] * srf)));
            }
            reset();
        }

        // --------------------------------------------------------
        // Wipe all engine state (tails, filters, delay lines).
        // --------------------------------------------------------
        void reset()
        {
            for (int i = 0; i < N; ++i)
            {
                std::fill(line[i].begin(), line[i].end(), 0.f);
                lp[i] = 0.f;
                lineAbs[i] = 0.f;
            }
            std::fill(er.begin(), er.end(), 0.f);
            for (int i = 0; i < 4; ++i)
            {
                apL[i].clear();
                apR[i].clear();
            }
            w = 0;
            erW = 0;
            dcXL = dcYL = dcXR = dcYR = 0.f;
        }

        // --------------------------------------------------------
        // Process one stereo frame:
        //   1. read the early-reflection taps (alternating L / R)
        //   2. run input diffusion through 4 allpasses per side
        //   3. read the 8 delay lines (fractional, modulated),
        //      damp them, apply the RT60 gain and accumulate decorrelated
        //      L / R outputs (PL / PR)
        //   4. apply the Hadamard feedback matrix, add new input, write
        //   5. mix taps + early reflections, then DC-block both outputs
        // --------------------------------------------------------
        void process(const Shared &s, float inL, float inR, float &wetL, float &wetR)
        {
            // early reflections (alternating L / R taps)
            er[erW] = 0.5f * (inL + inR);
            float erL = 0.f, erR = 0.f;
            for (int t = 0; t < N; ++t)
            {
                float v = er[(erW - (int)s.erTap[t]) & erMask] * ER_G[t];
                if (t & 1)
                    erR += v;
                else
                    erL += v;
            }
            erW = (erW + 1) & erMask;

            // input diffusion
            float dl = inL, dr = inR;
            for (int i = 0; i < 4; ++i)
            {
                dl = apL[i].process(dl, 0.6f);
                dr = apR[i].process(dr, 0.6f);
            }

            // read, damp, apply RT60 gain
            float y[N];
            float oL = 0.f, oR = 0.f;
            for (int i = 0; i < N; ++i)
            {
                float r = readHermite(line[i], w, s.delay[i] + s.mod[i], mask);
                lp[i] += s.lpCoef * (r - lp[i]);
                y[i] = lp[i] * s.gain[i];
                lineAbs[i] = std::fabs(y[i]);
                oL += PL[i] * y[i];
                oR += PR[i] * y[i];
            }

            // feedback matrix + injection
            float f[N];
            for (int i = 0; i < N; ++i)
                f[i] = y[i];
            hadamard8(f);
            for (int i = 0; i < N; ++i)
            {
                float v = f[i] + (dl * PL[i] + dr * PR[i]) * s.inGain;
                line[i][w] = rack::math::clamp(v, -20.f, 20.f);
            }
            w = (w + 1) & mask;

            float wl = oL * 0.35f + erL * s.erLevel;
            float wr = oR * 0.35f + erR * s.erLevel;

            // DC blocker
            float hl = wl - dcXL + 0.995f * dcYL;
            dcXL = wl;
            dcYL = hl;
            float hr = wr - dcXR + 0.995f * dcYR;
            dcXR = wr;
            dcYR = hr;
            wetL = hl;
            wetR = hr;
        }
    };

} // namespace whengine

// ------------------------------------------------------------
// Module — polyphonic FDN reverb.
//
// Owns MAX_CHANNELS engines, one per voice. The module computes a
// shared `Shared` struct once per sample and feeds every engine the
// same data; engines only differ in their delay line contents.
// All parameter smoothing is done here so the engine is stateless
// per-sample.
// ------------------------------------------------------------
struct Whirlpool : Module
{
    enum ParamId
    {
        REPLACE_PARAM,
        REPLACE_ATTEN,
        BRIGHTNESS_PARAM,
        BRIGHTNESS_ATTEN,
        DETUNE_PARAM,
        DETUNE_ATTEN,
        BIGNESS_PARAM,
        BIGNESS_ATTEN,
        MIX_PARAM,
        MIX_ATTEN,
        IN_STEREO_BUTTON_PARAM,
        OUT_STEREO_BUTTON_PARAM,
        CLEAR_BUTTON_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        IN_L_INPUT,
        IN_R_INPUT,
        REPLACE_CV_INPUT,
        BRIGHTNESS_CV_INPUT,
        DETUNE_CV_INPUT,
        BIGNESS_CV_INPUT,
        MIX_CV_INPUT,
        CLEAR_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        CLEAR_BUTTON_LIGHT,
        LIGHTS_LEN
    };

    static constexpr int MAX_CHANNELS = 16;
    whengine::Engine engines[MAX_CHANNELS];
    whengine::Shared sh;

    float allocSr = 0.f;                                     // sample rate the engines were init'd with
    float kSmooth = 0.001f, kSize = 0.0005f;                 // one-pole coefficients (control / size)
    float sizeS = 1.f, rtS = 4.f, fcS = 6000.f, detS = 0.2f; // smoothed control values
    float lfoPhase[8] = {};                                  // per-line modulation LFO phases

    // Clear trigger handling.
    dsp::SchmittTrigger clearButtonTrigger;
    dsp::SchmittTrigger clearInputTrigger;
    bool clearRequested = false;
    float clearFade = 1.0f;
    bool clearActive = false;
    int limiterRecovery = 0;

    // --------------------------------------------------------
    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic).
    // --------------------------------------------------------
    int dispMode = 0; // 0 wave, 1 bars, 2 spectrum (saved with the patch)
    int dispCh = 1;
    float dispVal[5] = {};     // replace, brightness, detune, bigness, mix (with CV)
    float dispLine[8] = {};    // FDN line levels (voice 0)
    float dispDelayMs[8] = {}; // FDN line delay times in ms
    float dispRt60 = 4.f;
    float dispSr = 48000.f;
    float meterOutL = 0.f, meterOutR = 0.f;

    // Waveform history: min/max per bin (1 bin per pixel).
    float scopeMinL[WH_SCOPE_N] = {}, scopeMaxL[WH_SCOPE_N] = {};
    float scopeMinR[WH_SCOPE_N] = {}, scopeMaxR[WH_SCOPE_N] = {};
    float scopeMinI[WH_SCOPE_N] = {}, scopeMaxI[WH_SCOPE_N] = {};
    int scopeHead = 0, scopeCount = 0, scopeBinSamples = 864;
    float accMinL = 1e9f, accMaxL = -1e9f, accMinR = 1e9f, accMaxR = -1e9f, accMinI = 1e9f, accMaxI = -1e9f;

    // FFT ring buffer (power-of-two).
    float specBuf[WH_SPEC_N] = {};
    int specPos = 0;

    // --------------------------------------------------------
    // Constructor — configure params, inputs, outputs, lights.
    // --------------------------------------------------------
    Whirlpool()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(REPLACE_PARAM, 0.f, 1.f, 0.f, "Replace (shortens the tail)", "%", 0.f, 100.f);
        configParam(REPLACE_ATTEN, -1.f, 1.f, 0.f, "Replace CV attenuverter", "%", 0.f, 100.f);
        configParam(BRIGHTNESS_PARAM, 0.f, 1.f, 0.5f, "Brightness (damping)", "%", 0.f, 100.f);
        configParam(BRIGHTNESS_ATTEN, -1.f, 1.f, 0.f, "Brightness CV attenuverter", "%", 0.f, 100.f);
        configParam(DETUNE_PARAM, 0.f, 1.f, 0.2f, "Detune (modulation)", "%", 0.f, 100.f);
        configParam(DETUNE_ATTEN, -1.f, 1.f, 0.f, "Detune CV attenuverter", "%", 0.f, 100.f);
        configParam(BIGNESS_PARAM, 0.f, 1.f, 0.6f, "Bigness (room size and decay)", "%", 0.f, 100.f);
        configParam(BIGNESS_ATTEN, -1.f, 1.f, 0.f, "Bigness CV attenuverter", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/wet", "%", 0.f, 100.f);
        configParam(MIX_ATTEN, -1.f, 1.f, 0.f, "Mix CV attenuverter", "%", 0.f, 100.f);

        configSwitch(IN_STEREO_BUTTON_PARAM, 0.f, 1.f, 0.f, "Stereo input (L/R into one reverb)", {"Off (mono / poly)", "On"});
        configSwitch(OUT_STEREO_BUTTON_PARAM, 0.f, 1.f, 0.f, "Stereo output (voices summed to L/R)", {"Off (mono / poly)", "On"});
        configButton(CLEAR_BUTTON_PARAM, "Clear");

        configInput(IN_L_INPUT, "Left / poly");
        configInput(IN_R_INPUT, "Right");
        configInput(REPLACE_CV_INPUT, "Replace CV (10 V = 100%)");
        configInput(BRIGHTNESS_CV_INPUT, "Brightness CV (10 V = 100%)");
        configInput(DETUNE_CV_INPUT, "Detune CV (10 V = 100%)");
        configInput(BIGNESS_CV_INPUT, "Bigness CV (10 V = 100%)");
        configInput(MIX_CV_INPUT, "Mix CV (10 V = 100%)");
        configInput(CLEAR_INPUT, "Clear trigger");

        configOutput(OUT_L_OUTPUT, "Left / poly");
        configOutput(OUT_R_OUTPUT, "Right");
        configLight(CLEAR_BUTTON_LIGHT, "Clear");

        configBypass(IN_L_INPUT, OUT_L_OUTPUT);
        configBypass(IN_R_INPUT, OUT_R_OUTPUT);

        // Stagger the modulation LFOs so the 8 lines aren't in phase.
        for (int i = 0; i < 8; ++i)
            lfoPhase[i] = 0.125f * i;
    }

    // --------------------------------------------------------
    // Reset all engines and clear the transient clear/limiter state.
    // --------------------------------------------------------
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        for (int c = 0; c < MAX_CHANNELS; ++c)
            engines[c].reset();
        clearRequested = false;
        clearFade = 1.0f;
        clearActive = false;
        limiterRecovery = 0;
    }

    // --------------------------------------------------------
    // Persist the display mode; everything else is a param.
    // --------------------------------------------------------
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "displayMode", json_integer(dispMode));
        return root;
    }

    // --------------------------------------------------------
    // Restore the display mode (clamped to a valid index).
    // --------------------------------------------------------
    void dataFromJson(json_t *root) override
    {
        json_t *m = json_object_get(root, "displayMode");
        if (m)
            dispMode = rack::math::clamp((int)json_integer_value(m), 0, 2);
    }

    // --------------------------------------------------------
    // (Re)initialise every engine for `sr`, and update the smoothing
    // coefficients + wave-bin cadence for that sample rate.
    // --------------------------------------------------------
    void allocate(float sr)
    {
        for (int c = 0; c < MAX_CHANNELS; ++c)
            engines[c].init(sr);
        allocSr = sr;
        dispSr = sr;
        // One-pole smoothing times: ~50 ms (controls), ~120 ms (size).
        kSmooth = 1.f - std::exp(-1.f / (0.05f * sr));
        kSize = 1.f - std::exp(-1.f / (0.12f * sr));
        scopeBinSamples = std::max(1, (int)(sr / 55.5f)); // 222 bins ~ 4 s of history
    }

    // --------------------------------------------------------
    // Combine one (param, attenuverter, CV) triple: CV / 10 V,
    // then knob + atten * cv, clamped to 0..1.
    // --------------------------------------------------------
    float getParam(int paramId, int attenId, int cvId)
    {
        float knob = params[paramId].getValue();
        float atten = params[attenId].getValue();
        float cv = inputs[cvId].getVoltageSum() / 10.f;
        float value = knob + atten * cv;
        return rack::math::clamp(value, 0.0f, 1.0f);
    }

    // --------------------------------------------------------
    // Main per-sample processing:
    //  1. Detect channel configuration (mono/poly/stereo in/out).
    //  2. Read all 5 (param, atten, CV) tuples.
    //  3. Compute the per-sample engine Shared struct from the
    //     smoothed controls (delay lengths, gains, damping, mod).
    //  4. Handle the clear fade/trigger sequence.
    //  5. Run each active voice, sum dry + wet (equal-power), with
    //     NaN/limiter safety.
    //  6. Feed the display: peaks, scope bins, FFT buffer.
    // --------------------------------------------------------
    void process(const ProcessArgs &args) override
    {
        const float sr = args.sampleRate;
        const float sampleTime = 1.0f / sr;
        const float srf = sr / 48000.f;

        if (sr != allocSr)
        {
            allocate(sr);
            sizeS = 0.35f + 1.65f * params[BIGNESS_PARAM].getValue();
            rtS = 0.35f * std::pow(60.f, params[BIGNESS_PARAM].getValue());
            fcS = 800.f * std::pow(24.f, params[BRIGHTNESS_PARAM].getValue());
            detS = params[DETUNE_PARAM].getValue();
        }

        // ---------------- channel handling ----------------
        int inLch = inputs[IN_L_INPUT].getChannels();
        int inRch = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getChannels() : 0;
        bool useStereoIn = params[IN_STEREO_BUTTON_PARAM].getValue() > 0.5f;
        bool useStereoOut = params[OUT_STEREO_BUTTON_PARAM].getValue() > 0.5f;

        int numChannels = useStereoIn ? 1 : rack::math::clamp(std::max(inLch, inRch), 1, MAX_CHANNELS);
        dispCh = numChannels;

        if (useStereoOut)
        {
            outputs[OUT_L_OUTPUT].setChannels(1);
            outputs[OUT_R_OUTPUT].setChannels(1);
        }
        else
        {
            outputs[OUT_L_OUTPUT].setChannels(numChannels);
            outputs[OUT_R_OUTPUT].setChannels(numChannels);
        }

        // ---------------- parameters ----------------
        float replace = getParam(REPLACE_PARAM, REPLACE_ATTEN, REPLACE_CV_INPUT);
        float brightness = getParam(BRIGHTNESS_PARAM, BRIGHTNESS_ATTEN, BRIGHTNESS_CV_INPUT);
        float detune = getParam(DETUNE_PARAM, DETUNE_ATTEN, DETUNE_CV_INPUT);
        float bigness = getParam(BIGNESS_PARAM, BIGNESS_ATTEN, BIGNESS_CV_INPUT);
        float mix = getParam(MIX_PARAM, MIX_ATTEN, MIX_CV_INPUT);
        dispVal[0] = replace;
        dispVal[1] = brightness;
        dispVal[2] = detune;
        dispVal[3] = bigness;
        dispVal[4] = mix;

        // Targets, smoothed so that no knob move or CV step ever clicks.
        float sizeT = 0.35f + 1.65f * bigness;                                 // delay length scale
        float rtT = 0.35f * std::pow(60.f, bigness) * (1.f - 0.92f * replace); // RT60 in seconds
        rtT = std::max(rtT, 0.05f);
        float fcT = std::min(800.f * std::pow(24.f, brightness), 0.45f * sr); // damping cutoff
        sizeS += kSize * (sizeT - sizeS);
        rtS += kSmooth * (rtT - rtS);
        fcS += kSmooth * (fcT - fcS);
        detS += kSmooth * (detune - detS);
        dispRt60 = rtS;

        // ---------------- shared per-sample engine state ----------------
        // Compute delay lengths (from size), RT60 gains, and LFO modulation
        // once per sample; every voice uses the same values.
        float gSum = 0.f;
        float depth = detS * 16.f * srf;
        for (int i = 0; i < whengine::N; ++i)
        {
            float d = whengine::BASE[i] * srf * sizeS;
            sh.delay[i] = d;
            float g = std::exp(-6.9078f * d / (sr * rtS)); // -60 dB after RT60
            g = std::min(g, 0.9995f);
            sh.gain[i] = g;
            gSum += g;

            // Free-running LFO; stagger speed per line for gentle chorusing.
            lfoPhase[i] += (0.12f + 0.065f * i + 0.6f * detS) * sampleTime;
            if (lfoPhase[i] >= 1.f)
                lfoPhase[i] -= 1.f;
            sh.mod[i] = std::sin(2.f * (float)M_PI * lfoPhase[i]) * depth;

            sh.erTap[i] = whengine::ER_MS[i] * 0.001f * sr * (0.4f + 0.8f * sizeS);
            dispDelayMs[i] = d / sr * 1000.f;
        }
        sh.lpCoef = 1.f - std::exp(-2.f * (float)M_PI * fcS * sampleTime);
        // Normalise injection so louder RT60s don't overdrive the network.
        float gAvg = gSum / (float)whengine::N;
        sh.inGain = 0.35f * (0.55f + 0.45f * std::sqrt(std::max(0.f, 1.f - gAvg * gAvg)));
        sh.erLevel = 0.28f;

        // ---------------- clear with fade ----------------
        // Trigger from button or jack; while `clearRequested` the wet path
        // fades down, then all engines reset and the wet path fades back in.
        bool clearPressed = params[CLEAR_BUTTON_PARAM].getValue() > 0.5f;
        bool clearTrig = clearButtonTrigger.process(clearPressed ? 1.f : 0.f) ||
                         clearInputTrigger.process(inputs[CLEAR_INPUT].getVoltage());
        if (clearTrig)
            clearRequested = true;

        if (clearRequested)
        {
            clearFade -= sampleTime * 10.f;
            if (clearFade <= 0.0f)
            {
                for (int i = 0; i < MAX_CHANNELS; ++i)
                    engines[i].reset();
                clearFade = 0.0f;
                clearRequested = false;
                clearActive = true;
            }
        }
        else if (clearActive)
        {
            clearFade += sampleTime * 10.f;
            if (clearFade >= 1.0f)
            {
                clearFade = 1.0f;
                clearActive = false;
            }
        }
        else
        {
            clearFade = 1.0f;
        }
        lights[CLEAR_BUTTON_LIGHT].setSmoothBrightness(clearPressed ? 1.0f : 0.0f, sampleTime);

        // ---------------- voices ----------------
        // Equal-power dry/wet crossfade; note that (unlike Vortex) the
        // module uses raw `mix` rather than a smoothed value (the CV is
        // already smoothed by hand movement).
        const float cd = std::cos(mix * 0.5f * (float)M_PI); // equal-power dry / wet
        const float sw = std::sin(mix * 0.5f * (float)M_PI);
        const float md = 1.f - 5.f * sampleTime; // peak decay

        float sumL = 0.f, sumR = 0.f, monIn = 0.f;
        for (int c = 0; c < numChannels; ++c)
        {
            float dryL, dryR;
            if (useStereoIn)
            {
                dryL = inputs[IN_L_INPUT].getVoltage();
                dryR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getVoltage() : dryL;
            }
            else
            {
                dryL = inputs[IN_L_INPUT].getPolyVoltage(c);
                dryR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getPolyVoltage(c) : dryL;
            }
            if (!std::isfinite(dryL))
                dryL = 0.f;
            if (!std::isfinite(dryR))
                dryR = 0.f;

            float wetL, wetR;
            engines[c].process(sh, dryL, dryR, wetL, wetR);

            // Wet path is scaled by the clear fade; dry stays untouched.
            float outL = (dryL * cd + wetL * sw) * clearFade;
            float outR = (dryR * cd + wetR * sw) * clearFade;

            // Safety: never let a NaN or runaway value out.
            if (!std::isfinite(outL) || !std::isfinite(outR))
            {
                outL = outR = 0.f;
                if (limiterRecovery == 0)
                {
                    for (int i = 0; i < MAX_CHANNELS; ++i)
                        engines[i].reset();
                    limiterRecovery = (int)(sr * 0.1f);
                }
            }
            if (limiterRecovery > 0)
            {
                outL = outR = 0.f;
                limiterRecovery--;
            }
            outL = rack::math::clamp(outL, -12.f, 12.f);
            outR = rack::math::clamp(outR, -12.f, 12.f);

            sumL += outL;
            sumR += outR;
            monIn += 0.5f * (dryL + dryR);

            // Voice 0 also feeds the display with per-line peak levels.
            if (c == 0)
            {
                for (int i = 0; i < whengine::N; ++i)
                    dispLine[i] = std::max(engines[0].lineAbs[i], dispLine[i] * md);
            }

            if (!useStereoOut)
            {
                outputs[OUT_L_OUTPUT].setVoltage(outL, c);
                outputs[OUT_R_OUTPUT].setVoltage(outR, c);
            }
        }
        // Stereo out: sum all voices to L/R.
        if (useStereoOut)
        {
            sumL = rack::math::clamp(sumL, -12.f, 12.f);
            sumR = rack::math::clamp(sumR, -12.f, 12.f);
            outputs[OUT_L_OUTPUT].setVoltage(sumL, 0);
            outputs[OUT_R_OUTPUT].setVoltage(sumR, 0);
        }

        // ---------------- display feeds ----------------
        meterOutL = std::max(std::fabs(sumL), meterOutL * md);
        meterOutR = std::max(std::fabs(sumR), meterOutR * md);

        specBuf[specPos] = 0.5f * (sumL + sumR);
        specPos = (specPos + 1) & (WH_SPEC_N - 1);

        // Wave history: accumulate min/max per channel, commit one bin
        // every `scopeBinSamples` samples.
        accMinL = std::min(accMinL, sumL);
        accMaxL = std::max(accMaxL, sumL);
        accMinR = std::min(accMinR, sumR);
        accMaxR = std::max(accMaxR, sumR);
        accMinI = std::min(accMinI, monIn);
        accMaxI = std::max(accMaxI, monIn);
        if (++scopeCount >= scopeBinSamples)
        {
            scopeCount = 0;
            scopeHead = (scopeHead + 1) % WH_SCOPE_N;
            scopeMinL[scopeHead] = accMinL;
            scopeMaxL[scopeHead] = accMaxL;
            scopeMinR[scopeHead] = accMinR;
            scopeMaxR[scopeHead] = accMaxR;
            scopeMinI[scopeHead] = accMinI;
            scopeMaxI[scopeHead] = accMaxI;
            accMinL = accMinR = accMinI = 1e9f;
            accMaxL = accMaxR = accMaxI = -1e9f;
        }
    }
};

// ============================================================
// Drawing helpers (anonymous namespace — UI-only)
// ------------------------------------------------------------
// Shared flat panel kit (agmm_ui.hpp): flat fills and hairline strokes only.
// ============================================================
namespace
{

    namespace ag = agmm;

    // ------------------------------------------------------------
    // WHPanel — static panel chrome:
    //   * panel base, bezel
    //   * row A: 5 knob cells (label, knob, CV jack + attenuverter with captions)
    //   * row B: stereo switches + clear button cells
    //   * row C: 5 I/O jack cells (outputs tinted)
    // The actual widgets are placed by WhirlpoolWidget at the same positions.
    // ------------------------------------------------------------
    struct WHPanel : TransparentWidget
    {
        WHPanel() { box.size = Vec(WH_W, WH_H); }
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, WH_W, WH_H, "WHIRLPOOL");
            ag::bezel(vg, WH_DX, WH_DY, WH_DW, WH_DH);

            // Row A: 5 knob cells.
            const char *names[5] = {"REPLACE", "BRIGHT", "DETUNE", "BIGNESS", "MIX"};
            for (int i = 0; i < 5; ++i)
            {
                float cx = whCol(i);
                ag::cell(vg, cx, WH_RA, WH_CELL_W, WH_RAH, names[i]);
                ag::text(vg, cx - WH_SUB_DX, WH_RA + WH_CAP_DY, "CV", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
                ag::text(vg, cx + WH_SUB_DX, WH_RA + WH_CAP_DY, "ATT", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            }

            // Row B: stereo switches + clear, control on the left of each cell, caption to its right.
            const char *b[3] = {"ST IN", "ST OUT", "CLEAR"};
            for (int i = 0; i < 3; ++i)
            {
                float cx = whCol(i + 1);
                ag::plate(vg, cx, WH_RB, WH_CELL_W, WH_RBH, false);
                ag::text(vg, cx + 9.f, WH_RB + WH_RBH / 2.f, b[i], 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            }

            // Row C: I/O jack cells (inputs, then tinted outputs).
            const char *io[5] = {"L IN", "R IN", "CLR IN", "OUT L", "OUT R"};
            for (int i = 0; i < 5; ++i)
                ag::cell(vg, whCol(i), WH_JTOP, WH_CELL_W, WH_JH, io[i], i >= 3);
            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // WHDisplay — live 260x124 display with three modes:
    //   WAVE     : scrolling min/max envelope (L cyan, R violet, dry white)
    //   BARS     : the 8 FDN line levels + delay times in ms
    //   SPECTRUM : log-frequency FFT with smoothing + peak-hold line
    // Interaction: click a tab (top row) to switch modes, or click the
    // plot area to cycle; also set from the context menu.
    // If module == nullptr (browser preview) synthetic demo data is drawn.
    // ------------------------------------------------------------
    struct WHDisplay : OpaqueWidget
    {
        Whirlpool *module = nullptr;

        // UI-side caches (mains thread only).
        float barPeak[8] = {};
        float specSm[WH_BANDS], specPk[WH_BANDS];
        std::vector<float> win, twC, twS, re, im, pw;
        float specDb[WH_BANDS]; // latest spectrum in dB (per widget instance)

        // Per-draw copy of the wave history (avoids racing the audio thread).
        float scMinL[WH_SCOPE_N], scMaxL[WH_SCOPE_N], scMinR[WH_SCOPE_N], scMaxR[WH_SCOPE_N];
        float scMinI[WH_SCOPE_N], scMaxI[WH_SCOPE_N];

        // --------------------------------------------------------
        // Precompute Hann window + twiddle factors; init smoothing state.
        // --------------------------------------------------------
        WHDisplay(Whirlpool *m) : module(m)
        {
            box.pos = Vec(WH_DX, WH_DY);
            box.size = Vec(WH_DW, WH_DH);

            win.resize(WH_SPEC_N);
            twC.resize(WH_SPEC_N / 2);
            twS.resize(WH_SPEC_N / 2);
            re.assign(WH_SPEC_N, 0.f);
            im.assign(WH_SPEC_N, 0.f);
            pw.assign(WH_SPEC_N / 2, 0.f);
            // Hann window.
            for (int i = 0; i < WH_SPEC_N; ++i)
                win[i] = 0.5f - 0.5f * std::cos(2.f * (float)M_PI * i / (float)WH_SPEC_N); // Hann
            // Twiddle factors for the FFT.
            for (int i = 0; i < WH_SPEC_N / 2; ++i)
            {
                twC[i] = std::cos(2.f * (float)M_PI * i / (float)WH_SPEC_N);
                twS[i] = std::sin(2.f * (float)M_PI * i / (float)WH_SPEC_N);
            }
            for (int b = 0; b < WH_BANDS; ++b)
            {
                specSm[b] = -90.f;
                specPk[b] = -90.f;
                specDb[b] = -90.f;
            }
        }

        // --------------------------------------------------------
        // Click handling: top-row tab select or plot-area cycle.
        // --------------------------------------------------------
        void onButton(const ButtonEvent &e) override
        {
            if (e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS)
            {
                if (module)
                {
                    if (e.pos.y < 15.f)
                    {
                        for (int i = 0; i < 3; ++i)
                            if (e.pos.x >= WD_TABX[i] && e.pos.x < WD_TABX[i] + WD_TABW[i])
                                module->dispMode = i;
                    }
                    else if (e.pos.y < WD_PY + WD_PH + 2.f)
                    {
                        module->dispMode = (module->dispMode + 1) % 3;
                    }
                }
                e.consume(this);
            }
        }

        // --------------------------------------------------------
        // In-place iterative radix-2 FFT on `re`/`im` (power-of-two size).
        // --------------------------------------------------------
        void fft()
        {
            const int N = WH_SPEC_N;
            // Bit-reversal permutation.
            for (int i = 1, j = 0; i < N; ++i)
            {
                int bit = N >> 1;
                for (; j & bit; bit >>= 1)
                    j ^= bit;
                j ^= bit;
                if (i < j)
                {
                    std::swap(re[i], re[j]);
                    std::swap(im[i], im[j]);
                }
            }
            // Butterfly passes.
            for (int len = 2; len <= N; len <<= 1)
            {
                int half = len >> 1, step = N / len;
                for (int i = 0; i < N; i += len)
                {
                    for (int k = 0; k < half; ++k)
                    {
                        float wr = twC[k * step], wi = -twS[k * step];
                        int a = i + k, b = a + half;
                        float tr = re[b] * wr - im[b] * wi;
                        float ti = re[b] * wi + im[b] * wr;
                        re[b] = re[a] - tr;
                        im[b] = im[a] - ti;
                        re[a] += tr;
                        im[a] += ti;
                    }
                }
            }
        }

        // --------------------------------------------------------
        // Fill dbOut[WH_BANDS] (dB re 5 V peak) from the module's
        // output history:
        //   * window + FFT the most recent WH_SPEC_N samples
        //   * bin into WH_BANDS log-spaced bands from 20 Hz to Nyquist
        //   * single-bin bands use linear interpolation between bins,
        //     wider bands average their included bins
        //   * convert to dB and normalise so 0 dB = 5 V amplitude
        // --------------------------------------------------------
        void computeSpectrum(float sr, float *dbOut)
        {
            int pos = module->specPos;
            // Window the most recent samples and zero the imaginary part.
            for (int i = 0; i < WH_SPEC_N; ++i)
            {
                re[i] = module->specBuf[(pos + i) & (WH_SPEC_N - 1)] * win[i];
                im[i] = 0.f;
            }
            fft();
            // Magnitude -> power spectrum for the first half.
            for (int k = 0; k < WH_SPEC_N / 2; ++k)
            {
                float a = std::sqrt(re[k] * re[k] + im[k] * im[k]) * 4.f / (float)WH_SPEC_N; // amplitude in volts
                pw[k] = a * a;
            }
            // Log-band aggregation.
            const float fMin = 20.f, fMax = std::min(20000.f, sr * 0.5f);
            const float binHz = sr / (float)WH_SPEC_N;
            const float ratio = fMax / fMin;
            for (int b = 0; b < WH_BANDS; ++b)
            {
                float f0 = fMin * std::pow(ratio, (float)b / WH_BANDS);
                float f1 = fMin * std::pow(ratio, (float)(b + 1) / WH_BANDS);
                float lo = f0 / binHz, hi = f1 / binHz;
                float p;
                if (hi - lo < 1.f)
                {
                    // Narrow band: linear interpolation between two bins.
                    float posb = 0.5f * (lo + hi);
                    int i0 = rack::math::clamp((int)posb, 0, WH_SPEC_N / 2 - 2);
                    float fr = posb - (float)i0;
                    p = pw[i0] * (1.f - fr) + pw[i0 + 1] * fr;
                }
                else
                {
                    // Wider band: average of the included bins.
                    int i0 = rack::math::clamp((int)std::ceil(lo), 0, WH_SPEC_N / 2 - 1);
                    int i1 = rack::math::clamp((int)std::floor(hi), i0, WH_SPEC_N / 2 - 1);
                    float s = 0.f;
                    for (int k = i0; k <= i1; ++k)
                        s += pw[k];
                    p = s / (float)(i1 - i0 + 1);
                }
                dbOut[b] = 10.f * std::log10(p / 25.f + 1e-12f); // 0 dB = 5 V amplitude
            }
        }

        // ---- static background (flat screen + plot frame) ----
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, WH_DW, WH_DH);
            ag::plotFrame(vg, WD_PX - 1.f, WD_PY - 1.f, WD_PW + 2.f, WD_PH + 2.f);
            Widget::draw(args);
        }

        // ---- live content ----
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = WH_DW, H = WH_DH;
            const float PX = WD_PX, PY = WD_PY, PW = WD_PW, PH = WD_PH;

            // ---------- gather data (or demo data when previewing) ----------
            int mode = 0, ch = 1;
            float pv[5] = {0.2f, 0.5f, 0.3f, 0.6f, 0.5f};
            float mL = 2.6f, mR = 2.2f, fade = 1.f, rt = 4.1f, sr = 48000.f;
            float lineLv[8], lineMs[8];
            bool sIn = false, sOut = false;

            if (module)
            {
                mode = rack::math::clamp(module->dispMode, 0, 2);
                ch = module->dispCh;
                for (int i = 0; i < 5; ++i)
                    pv[i] = module->dispVal[i];
                mL = module->meterOutL;
                mR = module->meterOutR;
                fade = module->clearFade;
                rt = module->dispRt60;
                sr = module->dispSr;
                sIn = module->params[Whirlpool::IN_STEREO_BUTTON_PARAM].getValue() > 0.5f;
                sOut = module->params[Whirlpool::OUT_STEREO_BUTTON_PARAM].getValue() > 0.5f;
                for (int i = 0; i < 8; ++i)
                {
                    lineLv[i] = module->dispLine[i];
                    lineMs[i] = module->dispDelayMs[i];
                }
                if (mode == 0)
                {
                    // Copy the wave history in chronological order (oldest first).
                    int head = module->scopeHead;
                    for (int i = 0; i < WH_SCOPE_N; ++i)
                    {
                        int idx = (head + 1 + i) % WH_SCOPE_N;
                        scMinL[i] = module->scopeMinL[idx];
                        scMaxL[i] = module->scopeMaxL[idx];
                        scMinR[i] = module->scopeMinR[idx];
                        scMaxR[i] = module->scopeMaxR[idx];
                        scMinI[i] = module->scopeMinI[idx];
                        scMaxI[i] = module->scopeMaxI[idx];
                    }
                }
                else if (mode == 2)
                {
                    computeSpectrum(sr, specDb);
                }
            }
            else
            {
                // module browser preview: synthetic data
                mode = 2;
                for (int i = 0; i < 8; ++i)
                {
                    lineLv[i] = 3.f * (0.35f + 0.65f * std::fabs(std::sin((float)i * 1.3f + 0.4f)));
                    lineMs[i] = whengine::BASE[i] / 48.f * 1.3f;
                }
                for (int b = 0; b < WH_BANDS; ++b)
                    specDb[b] = -18.f - 58.f * std::pow((float)b / (float)WH_BANDS, 0.75f) + 5.f * std::sin((float)b * 0.45f);
            }

            // ---------- plot (clipped to the frame) ----------
            nvgSave(vg);
            nvgScissor(vg, PX - 1.f, PY - 1.f, PW + 2.f, PH + 2.f);
            nvgLineCap(vg, NVG_BUTT);

            if (mode == 0)
            {
                // ===== WAVE: scrolling min/max envelope (L cyan, R violet), dry input in white =====
                const float mid = PY + PH / 2.f, half = PH / 2.f - 2.f, sc = half / 5.f;

                // Grid (vertical 1/8 divisions, horizontal quarter marks).
                nvgBeginPath(vg);
                for (int k = 1; k < 4; ++k)
                {
                    float x = PX + PW - (float)k * (PW / 4.f) + 0.5f;
                    nvgMoveTo(vg, x, PY);
                    nvgLineTo(vg, x, PY + PH);
                }
                nvgMoveTo(vg, PX, mid + 0.5f);
                nvgLineTo(vg, PX + PW, mid + 0.5f);
                nvgMoveTo(vg, PX, mid - half * 0.5f + 0.5f);
                nvgLineTo(vg, PX + PW, mid - half * 0.5f + 0.5f);
                nvgMoveTo(vg, PX, mid + half * 0.5f + 0.5f);
                nvgLineTo(vg, PX + PW, mid + half * 0.5f + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // One envelope (min..max per bin) in 4 flat alpha bands (older = fainter).
                const float step = PW / (float)WH_SCOPE_N;
                auto env = [&](const float *mn, const float *mx, NVGcolor col, float baseA)
                {
                    for (int q = 0; q < 4; ++q)
                    {
                        int i0 = q * WH_SCOPE_N / 4, i1 = (q + 1) * WH_SCOPE_N / 4;
                        nvgBeginPath(vg);
                        for (int i = i0; i < i1; ++i)
                        {
                            if (mx[i] < mn[i])
                                continue; // empty bin
                            float hi = rack::math::clamp(mx[i] * sc, -half, half);
                            float lo = rack::math::clamp(mn[i] * sc, -half, half);
                            float hh = std::max(hi - lo, 1.f);
                            nvgRect(vg, PX + (float)i * step, mid - hi, std::max(step, 1.f), hh);
                        }
                        nvgFillColor(vg, ag::withA(col, baseA * (0.35f + 0.65f * (float)(q + 1) / 4.f)));
                        nvgFill(vg);
                    }
                };
                env(scMinI, scMaxI, nvgRGB(255, 255, 255), 0.22f); // dry input (white)
                env(scMinR, scMaxR, ag::violet(), 0.75f);          // right
                env(scMinL, scMaxL, ag::accent(), 0.75f);          // left

                // Legend.
                const char *lg[3] = {"L", "R", "IN"};
                NVGcolor lc[3] = {ag::accent(), ag::violet(), nvgRGB(255, 255, 255)};
                for (int i = 0; i < 3; ++i)
                {
                    float x = PX + 6.f + (float)i * 24.f;
                    nvgBeginPath(vg);
                    nvgRect(vg, x, PY + 5.f, 4.f, 4.f);
                    nvgFillColor(vg, ag::withA(lc[i], 0.9f));
                    nvgFill(vg);
                    ag::text(vg, x + 7.f, PY + 7.f, lg[i], 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
                }
                ag::text(vg, PX + 4.f, PY + PH - 6.f, "-4 S", 7.f, ag::dim(), ag::ALIGN_L, 0.f);
                ag::text(vg, PX + PW - 4.f, PY + PH - 6.f, "NOW", 7.f, ag::dim(), ag::ALIGN_R, 0.f);
            }
            else if (mode == 1)
            {
                // ===== BARS: the 8 FDN line levels with peak hold and delay length =====
                const float cell = PW / 8.f, barW = 17.f;
                const float base = PY + PH - 12.f, maxH = PH - 24.f; // bars stay below the caption

                // Horizontal guides.
                nvgBeginPath(vg);
                for (int k = 1; k <= 3; ++k)
                {
                    float y = base - maxH * (float)k / 4.f + 0.5f;
                    nvgMoveTo(vg, PX, y);
                    nvgLineTo(vg, PX + PW, y);
                }
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // One flat bar per FDN line, coloured along the cyan-violet ramp.
                char buf[32];
                for (int i = 0; i < 8; ++i)
                {
                    float h = rack::math::clamp(std::sqrt(lineLv[i] / 4.f), 0.f, 1.f);
                    barPeak[i] = std::max(h, barPeak[i] - 0.007f);
                    float x = PX + (float)i * cell + (cell - barW) / 2.f;
                    float bh = std::max(h * maxH, 1.f);
                    NVGcolor c0 = nvgLerpRGBA(ag::accent(), ag::violet(), (float)i / 7.f);

                    nvgBeginPath(vg);
                    nvgRect(vg, x, base - bh, barW, bh);
                    nvgFillColor(vg, ag::withA(c0, 0.7f));
                    nvgFill(vg);
                    // Peak-hold line.
                    float py = base - barPeak[i] * maxH;
                    nvgBeginPath(vg);
                    nvgRect(vg, x, py - 1.f, barW, 1.5f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 220));
                    nvgFill(vg);

                    // Delay time label under the bar.
                    snprintf(buf, sizeof(buf), "%.0f", lineMs[i]);
                    ag::text(vg, x + barW / 2.f, base + 6.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.f);
                }

                ag::text(vg, PX + 6.f, PY + 7.f, "FDN LINES - DELAY MS", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            }
            else
            {
                // ===== SPECTRUM: log-frequency FFT of the output =====
                const float fMin = 20.f, fMax = std::min(20000.f, sr * 0.5f);
                auto xOf = [&](float f)
                { return PX + PW * std::log(f / fMin) / std::log(fMax / fMin); };
                auto yOf = [&](float db)
                { return PY + PH - rack::math::clamp((db + 90.f) / 90.f, 0.f, 1.f) * PH; };

                // Grid: log-frequency verticals and dB horizontals.
                nvgBeginPath(vg);
                const float fl[9] = {50.f, 100.f, 200.f, 500.f, 1000.f, 2000.f, 5000.f, 10000.f, 20000.f};
                for (int i = 0; i < 9; ++i)
                {
                    if (fl[i] >= fMax)
                        continue;
                    float x = xOf(fl[i]) + 0.5f;
                    nvgMoveTo(vg, x, PY);
                    nvgLineTo(vg, x, PY + PH);
                }
                for (int d = -20; d >= -80; d -= 20)
                {
                    float y = yOf((float)d) + 0.5f;
                    nvgMoveTo(vg, PX, y);
                    nvgLineTo(vg, PX + PW, y);
                }
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Smoothing (fast attack, slow release) + peak-hold decay.
                for (int b = 0; b < WH_BANDS; ++b)
                {
                    float v = rack::math::clamp(specDb[b], -90.f, 0.f);
                    specSm[b] += (v > specSm[b] ? 0.55f : 0.14f) * (v - specSm[b]);
                    specPk[b] = std::max(specSm[b], specPk[b] - 0.35f);
                }

                // Filled spectrum (flat area under the smoothed curve).
                const float bw = PW / (float)WH_BANDS;
                nvgBeginPath(vg);
                nvgMoveTo(vg, PX, PY + PH);
                for (int b = 0; b < WH_BANDS; ++b)
                    nvgLineTo(vg, PX + ((float)b + 0.5f) * bw, yOf(specSm[b]));
                nvgLineTo(vg, PX + PW, PY + PH);
                nvgClosePath(vg);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.18f));
                nvgFill(vg);

                // Peak-hold line.
                nvgBeginPath(vg);
                for (int b = 0; b < WH_BANDS; ++b)
                {
                    float x = PX + ((float)b + 0.5f) * bw, y = yOf(specPk[b]);
                    if (b == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, ag::withA(ag::violet(), 0.55f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Main curve: a single hairline.
                nvgBeginPath(vg);
                for (int b = 0; b < WH_BANDS; ++b)
                {
                    float x = PX + ((float)b + 0.5f) * bw, y = yOf(specSm[b]);
                    if (b == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, ag::bright());
                nvgStrokeWidth(vg, 1.f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                // Frequency / dB labels.
                const char *fn[9] = {"50", "100", "200", "500", "1K", "2K", "5K", "10K", "20K"};
                for (int i = 0; i < 9; ++i)
                {
                    if (fl[i] >= fMax)
                        continue;
                    ag::text(vg, xOf(fl[i]), PY + PH - 5.f, fn[i], 7.f, ag::dim(), ag::ALIGN_C, 0.f);
                }
                char buf[16];
                for (int d = -20; d >= -80; d -= 20)
                {
                    snprintf(buf, sizeof(buf), "%d", d);
                    ag::text(vg, PX + 3.f, yOf((float)d) - 5.f, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.f);
                }
            }

            // Clear flash across the plot while a clear is in progress.
            if (fade < 0.99f)
            {
                nvgBeginPath(vg);
                nvgRect(vg, PX, PY, PW, PH);
                nvgFillColor(vg, ag::withA(ag::violet(), 0.28f * (1.f - fade)));
                nvgFill(vg);
            }
            nvgResetScissor(vg);
            nvgRestore(vg);

            // ---------- stereo output meters (right of the plot) ----------
            ag::meter(vg, PX + PW + 8.f, PY, 6.f, PH, mL / 5.f);
            ag::meter(vg, PX + PW + 17.f, PY, 6.f, PH, mR / 5.f);

            // ---------- header: tabs + status ----------
            const char *tn[3] = {"WAVE", "BARS", "SPECTRUM"};
            for (int i = 0; i < 3; ++i)
            {
                bool on = (i == mode);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, WD_TABX[i], 3.f, WD_TABW[i], 11.f, 3.f);
                if (on)
                {
                    nvgFillColor(vg, ag::withA(ag::accent(), 0.22f));
                    nvgFill(vg);
                    nvgStrokeColor(vg, ag::withA(ag::accent(), 0.9f));
                }
                else
                {
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                }
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                ag::text(vg, WD_TABX[i] + WD_TABW[i] / 2.f, 8.8f, tn[i], 7.f, on ? ag::accent() : ag::dim(),
                         ag::ALIGN_C, 0.5f);
            }
            char buf[64];
            // Status.
            if (fade < 0.99f)
            {
                ag::text(vg, W - 6.f, 8.5f, "CLEARING", 7.f, ag::violet(), ag::ALIGN_R, 0.5f);
            }
            else
            {
                snprintf(buf, sizeof(buf), "%s%sCH %d RT %.1fS", sIn ? "ST IN " : "", sOut ? "ST OUT " : "", ch, rt);
                ag::text(vg, W - 6.f, 8.5f, buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            }

            // ---------- footer: parameter readouts ----------
            const char *nm[5] = {"REPL", "BRGT", "DTUN", "BIG", "MIX"};
            const float cw = (W - 16.f) / 5.f;
            for (int i = 0; i < 5; ++i)
            {
                float x = 8.f + (float)i * cw;
                ag::text(vg, x, H - 7.f, nm[i], 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
                snprintf(buf, sizeof(buf), "%.0f%%", rack::math::clamp(pv[i], 0.f, 1.f) * 100.f);
                ag::text(vg, x + cw - 6.f, H - 7.f, buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            }

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// WhirlpoolWidget — Rack widget.
//   * panel background + display + screws
//   * row A: 5 knobs with CV jack (left) and attenuverter (right)
//   * row B: ST IN / ST OUT switches + CLEAR button with LED
//   * row C: 5 I/O jacks (L IN / R IN / CLR IN / OUT L / OUT R)
//   * context menu: display mode
// ------------------------------------------------------------
struct WhirlpoolWidget : ModuleWidget
{
    WhirlpoolWidget(Whirlpool *module)
    {
        setModule(module);
        box.size = Vec(WH_W, WH_H);

        addChild(new WHPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new WHDisplay(module));

        // ---- row A: knob on top, CV jack (left) and attenuverter (right) below ----
        // Params are stored in pairs (param, atten) per cell; the +2*i
        // offsets step the enum to each successive cell.
        for (int i = 0; i < 5; ++i)
        {
            float cx = whCol(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, WH_RA + WH_KNOB_DY), module, Whirlpool::REPLACE_PARAM + 2 * i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - WH_SUB_DX, WH_RA + WH_SUB_DY), module, Whirlpool::REPLACE_CV_INPUT + i));
            addParam(createParamCentered<Trimpot>(Vec(cx + WH_SUB_DX, WH_RA + WH_SUB_DY), module, Whirlpool::REPLACE_ATTEN + 2 * i));
        }

        // ---- row B: stereo switches + clear ----
        const float by = WH_RB + WH_RBH / 2.f;
        addParam(createParamCentered<CKSS>(Vec(whCol(1) + WH_BTN_DX, by), module, Whirlpool::IN_STEREO_BUTTON_PARAM));
        addParam(createParamCentered<CKSS>(Vec(whCol(2) + WH_BTN_DX, by), module, Whirlpool::OUT_STEREO_BUTTON_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(whCol(3) + WH_BTN_DX, by), module, Whirlpool::CLEAR_BUTTON_PARAM));
        addChild(createLightCentered<MediumLight<ag::CyanLight>>(Vec(whCol(3) + WH_BTN_DX, by), module, Whirlpool::CLEAR_BUTTON_LIGHT));

        // ---- row C: I/O jacks ----
        const float jy = WH_JTOP + WH_JDY;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(whCol(0), jy), module, Whirlpool::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(whCol(1), jy), module, Whirlpool::IN_R_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(whCol(2), jy), module, Whirlpool::CLEAR_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(whCol(3), jy), module, Whirlpool::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(whCol(4), jy), module, Whirlpool::OUT_R_OUTPUT));
        agmm::addKnobArcs(this, 2.f);
    }

    // Add a "Display" submenu (Wave / Bars / Spectrum).
    void appendContextMenu(Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        Whirlpool *module = dynamic_cast<Whirlpool *>(this->module);
        if (!module)
            return;
        menu->addChild(new MenuSeparator);
        menu->addChild(createSubmenuItem("Display", "", [=](Menu *sub)
                                         {
            const char* names[3] = {"Wave", "Bars", "Spectrum"};
            for (int i = 0; i < 3; ++i)
                sub->addChild(createCheckMenuItem(names[i], "",
                    [=]() { return module->dispMode == i; },
                    [=]() { module->dispMode = i; })); }));
    }
};

Model *modelWhirlpool = createModel<Whirlpool, WhirlpoolWidget>("Whirlpool");