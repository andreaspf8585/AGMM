#include "plugin.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <vector>
#include <memory>
#include <cstdio>

// =============================================================================
// Dark Hall Granular
// =============================================================================
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// DarkHallGranular is an 18 HP stereo "galaxy-style" reverb. A shared input
// diffusion chain feeds two cross-coupled feedback tanks, each containing a
// long delay, a one-pole damping filter and two modulated all-passes. The
// two tanks cross-feed each other, producing a dense, slowly evolving tail.
// Controls: Replace, Brightness, Detune and Bigness (each with its own CV
// input and bipolar attenuverter), plus input gain, dry/wet and output gain.
// A RESET input flushes the reverb tail. The display shows a scope of the
// mono output plus L/R level meters.
//
// File structure:
//   1. DSP helpers: clampT, sanitize, softClip, DelayLine.
//   2. DarkHallGranular module: delay-line tanks, params/ports config, CV
//      scaling, reset, JSON persistence and process().
//   3. Layout constants (18 HP = 270x380 px).
//   4. Anonymous-namespace theme/drawing helpers and the two panel widgets:
//        - DhgPanelBackground: static artwork
//        - DhgDisplayWidget:  live scope + meters (layer 1)
//   5. DarkHallGranularWidget: places widgets and builds the CV-range menu.
//   6. Model registration.
// -----------------------------------------------------------------------------
static const int DHG_SCOPE_N = 96;

// =============================================================================
// Galaxy-style stereo reverb — DSP preserved from original
// =============================================================================

// Clamps v into [lo, hi].
template <typename T>
static inline T clampT(T v, T lo, T hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Replaces non-finite values with 0 and flushes denormals (|x| < 1e-25) to 0.
static inline float sanitize(float x)
{
    if (!std::isfinite(x))
        return 0.f;
    if (std::fabs(x) < 1e-25f)
        return 0.f;
    return x;
}

// Soft-clips x to approximately ±limit using tanh.
static inline float softClip(float x, float limit)
{
    return limit * std::tanh(x / limit);
}

// Circular delay line with cubic-interpolated fractional reads and an
// all-pass helper. Used for both the diffusion and the tank delays.
struct DelayLine
{
    std::vector<float> buf;
    int size = 0;
    int write = 0;

    // Allocates the buffer with n samples and clears it.
    void init(int n)
    {
        size = n;
        buf.assign(n, 0.f);
        write = 0;
    }
    // Zeroes the buffer contents and resets the write pointer.
    void clear()
    {
        std::fill(buf.begin(), buf.end(), 0.f);
        write = 0;
    }
    // Writes one sample at the write head and advances it (wrapping).
    inline void push(float x)
    {
        buf[write] = x;
        if (++write >= size)
            write = 0;
    }
    // Reads the buffer at a fractional `delay` (in samples) using 4-point
    // cubic interpolation. The delay is clamped to keep the read indices
    // inside the buffer.
    inline float read(float delay) const
    {
        delay = clampT(delay, 2.f, (float)(size - 4));
        float pos = (float)write - 1.f - delay + 1.f;
        pos -= 1.f;
        while (pos < 0.f)
            pos += (float)size;
        int i1 = (int)pos;
        float t = pos - (float)i1;
        if (i1 >= size)
            i1 -= size;
        int i0 = i1 - 1;
        if (i0 < 0)
            i0 += size;
        int i2 = i1 + 1;
        if (i2 >= size)
            i2 -= size;
        int i3 = i1 + 2;
        if (i3 >= size)
            i3 -= size;
        float y0 = buf[i0], y1 = buf[i1], y2 = buf[i2], y3 = buf[i3];
        float a = -0.5f * y0 + 1.5f * y1 - 1.5f * y2 + 0.5f * y3;
        float b = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
        float c = -0.5f * y0 + 0.5f * y2;
        return ((a * t + b) * t + c) * t + y1;
    }
    // Processes one sample through the line as a Schroeder all-pass with
    // gain g at the given delay, pushing the sanitised input and returning
    // the all-pass output.
    inline float allpass(float in, float delay, float g)
    {
        float d = read(delay);
        float w = in - g * d;
        push(sanitize(w));
        return d + g * w;
    }
};

// =============================================================================
// DarkHallGranular module
// =============================================================================
struct DarkHallGranular : Module
{

    // Fixed diffusion all-pass delays (samples at 32 kHz reference).
    static constexpr float AP_IN[4] = {150.f, 214.f, 319.f, 527.f};
    static constexpr float DAP1A = 2182.f, DAP1B = 2690.f, DEL1 = 4501.f;
    static constexpr float DAP2A = 2525.f, DAP2B = 2197.f, DEL2 = 6312.f;

    DelayLine apIn[4];
    DelayLine dap1a, dap1b, del1;
    DelayLine dap2a, dap2b, del2;

    float lp1 = 0.f, lp2 = 0.f;
    float lfo1 = 0.f, lfo2 = 0.25f, lfo3 = 0.5f, lfo4 = 0.75f;
    float bigSm = 0.7f;
    float dcInL = 0.f, dcInR = 0.f, dcOutL = 0.f, dcOutR = 0.f;

    bool wasResetHigh = false;
    bool wasConnected = false;

    // Selectable CV interpretation: bipolar ±5 V or unipolar 0..10 V.
    enum CVRange
    {
        BIPOLAR_M5_5,
        UNIPOLAR_0_10
    };
    CVRange cvRangeMode = BIPOLAR_M5_5;

    // ---- display state ----
    float scopeMin[DHG_SCOPE_N] = {};
    float scopeMax[DHG_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    enum ParamId
    {
        REPLACE_PARAM,
        BRIGHTNESS_PARAM,
        DETUNE_PARAM,
        BIGNESS_PARAM,
        DRYWET_PARAM,
        INPUT_GAIN_PARAM,
        OUTPUT_GAIN_PARAM,
        REPLACE_ATTEN,
        BRIGHTNESS_ATTEN,
        DETUNE_ATTEN,
        BIGNESS_ATTEN,
        PARAMS_LEN
    };
    enum InputId
    {
        IN_L,
        IN_R,
        REPLACE_CV,
        BRIGHTNESS_CV,
        DETUNE_CV,
        BIGNESS_CV,
        RESET_IN,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUT_L,
        OUT_R,
        OUTPUTS_LEN
    };

    // Constructor: configures all parameters, inputs, outputs, stereo
    // bypass, and allocates every delay line to its final length.
    DarkHallGranular()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);

        configParam(REPLACE_PARAM, 0.f, 1.f, 0.20f, "Replace", "%", 0.f, 100.f);
        configParam(BRIGHTNESS_PARAM, 0.f, 1.f, 0.45f, "Brightness", "%", 0.f, 100.f);
        configParam(DETUNE_PARAM, 0.f, 1.f, 0.50f, "Detune", "%", 0.f, 100.f);
        configParam(BIGNESS_PARAM, 0.f, 1.f, 0.70f, "Bigness", "%", 0.f, 100.f);
        configParam(DRYWET_PARAM, 0.f, 1.f, 0.50f, "Dry/Wet", "%", 0.f, 100.f);
        configParam(INPUT_GAIN_PARAM, 0.f, 2.f, 1.0f, "Input gain", "%", 0.f, 100.f);
        configParam(OUTPUT_GAIN_PARAM, 0.f, 2.f, 1.0f, "Output gain", "%", 0.f, 100.f);

        configParam(REPLACE_ATTEN, -1.f, 1.f, 0.f, "Replace CV amount", "%", 0.f, 100.f);
        configParam(BRIGHTNESS_ATTEN, -1.f, 1.f, 0.f, "Brightness CV amount", "%", 0.f, 100.f);
        configParam(DETUNE_ATTEN, -1.f, 1.f, 0.f, "Detune CV amount", "%", 0.f, 100.f);
        configParam(BIGNESS_ATTEN, -1.f, 1.f, 0.f, "Bigness CV amount", "%", 0.f, 100.f);

        configInput(IN_L, "Left");
        configInput(IN_R, "Right");
        configInput(REPLACE_CV, "Replace CV");
        configInput(BRIGHTNESS_CV, "Brightness CV");
        configInput(DETUNE_CV, "Detune CV");
        configInput(BIGNESS_CV, "Bigness CV");
        configInput(RESET_IN, "Reset (clears reverb tail)");

        configOutput(OUT_L, "Left");
        configOutput(OUT_R, "Right");

        configBypass(IN_L, OUT_L);
        configBypass(IN_R, OUT_R);

        for (int i = 0; i < 4; ++i)
            apIn[i].init(4096);
        dap1a.init(32768);
        dap1b.init(32768);
        dap2a.init(32768);
        dap2b.init(32768);
        del1.init(131072);
        del2.init(131072);
    }

    // Clears every delay line and resets the filter, LFO, smoothing and
    // DC-blocker state. Used on reset and when the reset input is patched.
    void resetState()
    {
        for (int i = 0; i < 4; ++i)
            apIn[i].clear();
        dap1a.clear();
        dap1b.clear();
        del1.clear();
        dap2a.clear();
        dap2b.clear();
        del2.clear();
        lp1 = lp2 = 0.f;
        lfo1 = 0.f;
        lfo2 = 0.25f;
        lfo3 = 0.5f;
        lfo4 = 0.75f;
        dcInL = dcInR = dcOutL = dcOutR = 0.f;
    }

    // Rack reset hook: flushes the reverb tail.
    void onReset() override { resetState(); }

    // Serialises the chosen CV range mode so patches reopen with the same
    // CV interpretation.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        return root;
    }
    // Restores the CV range mode from a saved patch, defaulting to bipolar.
    void dataFromJson(json_t *root) override
    {
        json_t *mode = json_object_get(root, "cvRangeMode");
        if (mode)
        {
            int m = (int)json_integer_value(mode);
            cvRangeMode = (m == UNIPOLAR_0_10) ? UNIPOLAR_0_10 : BIPOLAR_M5_5;
        }
    }

    // Returns the effective value of `param` after adding the CV input
    // `cvIn` scaled by the attenuverter `atten`, clamped to 0..1. The CV is
    // normalised according to the selected CV range mode.
    float cvParam(int param, int cvIn, int atten)
    {
        float cv = sanitize(inputs[cvIn].getVoltage());
        float scaled;
        if (cvRangeMode == UNIPOLAR_0_10)
            scaled = cv / 10.f;
        else
            scaled = clampT(cv / 5.f, -1.f, 1.f);
        return clampT(params[param].getValue() + scaled * params[atten].getValue(), 0.f, 1.f);
    }

    // Main audio callback: handles reset/connection state, reads the
    // controls and CVs, runs the diffusion + two cross-coupled tanks,
    // mixes dry/wet, writes the outputs and updates the scope/meter data.
    void process(const ProcessArgs &args) override
    {
        if (args.sampleTime <= 0.f)
            return;

        bool resetHigh = inputs[RESET_IN].getVoltage() > 1.f;
        if (resetHigh && !wasResetHigh)
            resetState();
        wasResetHigh = resetHigh;

        if (!outputs[OUT_L].isConnected() && !outputs[OUT_R].isConnected())
        {
            outputs[OUT_L].setVoltage(0.f);
            outputs[OUT_R].setVoltage(0.f);
            return;
        }
        bool inputConnected = inputs[IN_L].isConnected() || inputs[IN_R].isConnected();
        if (!inputConnected)
        {
            if (wasConnected)
            {
                resetState();
                wasConnected = false;
            }
            outputs[OUT_L].setVoltage(0.f);
            outputs[OUT_R].setVoltage(0.f);
            // gentle decay of meter peaks for UI
            float decayRate = 1.f - 4.f * args.sampleTime;
            peakL *= decayRate;
            peakR *= decayRate;
            return;
        }
        wasConnected = true;

        // ---- Parameters ----------------------------------------------------
        float replace = cvParam(REPLACE_PARAM, REPLACE_CV, REPLACE_ATTEN);
        float bright = cvParam(BRIGHTNESS_PARAM, BRIGHTNESS_CV, BRIGHTNESS_ATTEN);
        float detune = cvParam(DETUNE_PARAM, DETUNE_CV, DETUNE_ATTEN);
        float bigness = cvParam(BIGNESS_PARAM, BIGNESS_CV, BIGNESS_ATTEN);
        float dryWet = clampT(params[DRYWET_PARAM].getValue(), 0.f, 1.f);
        float inGain = params[INPUT_GAIN_PARAM].getValue();
        float outGain = params[OUTPUT_GAIN_PARAM].getValue();

        // ---- Input ---------------------------------------------------------
        float inL = softClip(sanitize(inputs[IN_L].getVoltage()), 8.f) * inGain;
        float inR = inputs[IN_R].isConnected()
                        ? softClip(sanitize(inputs[IN_R].getVoltage()), 8.f) * inGain
                        : inL;
        if (!inputs[IN_L].isConnected())
            inL = inR;

        // ---- Derived values ------------------------------------------------
        // Scale the fixed delay times to the current sample rate and smoothly
        // track the Bigness control so size changes glide.
        const float srScale = args.sampleRate / 32000.f;

        float bigTgt = 0.4f + bigness * 1.2f;
        bigSm += (bigTgt - bigSm) * 0.00015f;
        const float k = srScale * bigSm;

        float krt = 0.40f + 0.585f * (1.f - replace);
        float lpCoef = 0.08f + 0.90f * bright * bright;

        const float kap = 0.625f;
        const float inDiff = 0.75f;

        float modDepth = detune * detune * 55.f * srScale;
        float apModDepth = detune * 14.f * srScale;

        // Four free-running LFOs (different rates) for the modulated delays.
        lfo1 += args.sampleTime * 0.50f;
        lfo2 += args.sampleTime * 0.31f;
        lfo3 += args.sampleTime * 0.23f;
        lfo4 += args.sampleTime * 0.37f;
        if (lfo1 > 1.f)
            lfo1 -= 1.f;
        if (lfo2 > 1.f)
            lfo2 -= 1.f;
        if (lfo3 > 1.f)
            lfo3 -= 1.f;
        if (lfo4 > 1.f)
            lfo4 -= 1.f;
        const float twoPi = 2.f * (float)M_PI;
        float m1 = std::sin(twoPi * lfo1);
        float m2 = std::sin(twoPi * lfo2);
        float m3 = std::sin(twoPi * lfo3);
        float m4 = std::sin(twoPi * lfo4);

        // ---- Input diffusion -----------------------------------------------
        float x = (inL + inR) * 0.35f;
        for (int i = 0; i < 4; ++i)
            x = apIn[i].allpass(x, AP_IN[i] * srScale, inDiff);
        float apOut = sanitize(x);

        // ---- Tank 1 (-> left) -----------------------------------------------
        // A long delay from tank 2, damped, through two modulated all-passes
        // and back into tank 1's delay.
        float wet = apOut + del2.read(DEL2 * k + m3 * modDepth) * krt;
        lp1 += lpCoef * (wet - lp1);
        lp1 = sanitize(lp1);
        wet = lp1;
        wet = dap1a.allpass(wet, DAP1A * k + m1 * apModDepth, -kap);
        wet = dap1b.allpass(wet, DAP1B * k, kap);
        wet = softClip(wet, 8.f);
        del1.push(sanitize(wet));
        float revL = wet;

        // ---- Tank 2 (-> right) ----------------------------------------------
        // Mirror of tank 1, reading tank 1's delay for the cross-feed.
        wet = apOut + del1.read(DEL1 * k + m4 * modDepth) * krt;
        lp2 += lpCoef * (wet - lp2);
        lp2 = sanitize(lp2);
        wet = lp2;
        wet = dap2a.allpass(wet, DAP2A * k + m2 * apModDepth, -kap);
        wet = dap2b.allpass(wet, DAP2B * k, kap);
        wet = softClip(wet, 8.f);
        del2.push(sanitize(wet));
        float revR = wet;

        // ---- Output: DC block, equal-power mix ------------------------------
        float wetL = revL - dcInL + 0.995f * dcOutL;
        float wetR = revR - dcInR + 0.995f * dcOutR;
        dcInL = revL;
        dcOutL = sanitize(wetL);
        dcInR = revR;
        dcOutR = sanitize(wetR);

        const float wetBoost = 1.6f;
        wetL *= wetBoost;
        wetR *= wetBoost;

        float dryG = std::cos(dryWet * (float)M_PI_2);
        float wetG = std::sin(dryWet * (float)M_PI_2);
        float outL = dryG * inL + wetG * wetL;
        float outR = dryG * inR + wetG * wetR;

        outL = softClip(outL, 8.f) * outGain;
        outR = softClip(outR, 8.f) * outGain;
        outL = clampT(outL, -10.f, 10.f);
        outR = clampT(outR, -10.f, 10.f);

        float vL = sanitize(outL);
        float vR = sanitize(outR);
        outputs[OUT_L].setVoltage(vL);
        outputs[OUT_R].setVoltage(vR);

        // ---- display data ---------------------------------------------------
        // Peak envelopes with a short decay for the L/R meters.
        float decayRate = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(vL), peakL * decayRate);
        peakR = std::max(std::fabs(vR), peakR * decayRate);

        // Scope ring buffer: min/max of the mono output, bucketed to ~2.5 ms.
        float mono = 0.5f * (vL + vR);
        if (scopeCount == 0)
            scopeBMin = scopeBMax = mono;
        else
        {
            scopeBMin = std::min(scopeBMin, mono);
            scopeBMax = std::max(scopeBMax, mono);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % DHG_SCOPE_N;
        }
    }
};

// Out-of-line definition required for the static constexpr array.
constexpr float DarkHallGranular::AP_IN[4];

// =============================================================================
// Layout (18 HP = 270 x 380 px)
//
//   y   0-15   screws
//   y  10-20   title
//   y  26-66   display bezel (scope + meters)
//   y  78      section: TONE
//   y  88-188  4 cells: label / knob / [CV jack + atten] / tiny labels
//   y 202      section: LEVELS
//   y 212-280  3 cells: label / knob
//   y 292      section: I/O
//   y 302-352  5 cells: label / jack
//   y 362-372  maker tag (between the bottom screws)
//
// Every control lives inside its plate with clear margins; no label shares
// space with a widget.
// =============================================================================
static const float DHG_PANEL_W = RACK_GRID_WIDTH * 18; // 270
static const float DHG_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float DHG_DISP_W = 230.f;
static const float DHG_DISP_H = 32.f;
static const float DHG_DISP_X = (DHG_PANEL_W - DHG_DISP_W) / 2.f; // 20
static const float DHG_DISP_Y = 30.f;

static const float DHG_TITLE_Y = 15.f;
static const float DHG_MAKER_Y = 367.f;

// TONE: 4 cells with CV jack + attenuverter
static const float DHG_TONE_SEC_Y = 78.f;
static const float DHG_TONE_CX[4] = {45.f, 105.f, 165.f, 225.f};
static const float DHG_TONE_PLATE_TOP = 88.f, DHG_TONE_PLATE_H = 100.f, DHG_TONE_PLATE_W = 56.f;
static const float DHG_TONE_LABEL_Y = 98.f;
static const float DHG_TONE_KNOB_Y = 124.f;
static const float DHG_TONE_CV_Y = 162.f;
static const float DHG_TONE_SMALL_Y = 181.f;

// LEVELS: 3 knobs
static const float DHG_LVL_SEC_Y = 202.f;
static const float DHG_LVL_CX[3] = {55.f, 135.f, 215.f};
static const float DHG_LVL_PLATE_TOP = 212.f, DHG_LVL_PLATE_H = 68.f, DHG_LVL_PLATE_W = 70.f;
static const float DHG_LVL_LABEL_Y = 222.f;
static const float DHG_LVL_KNOB_Y = 250.f;

// I/O: 5 jacks (3 in, 2 out)
static const float DHG_IO_SEC_Y = 292.f;
static const float DHG_IO_CX[5] = {35.f, 85.f, 135.f, 185.f, 235.f};
static const float DHG_IO_PLATE_TOP = 302.f, DHG_IO_PLATE_H = 50.f, DHG_IO_PLATE_W = 46.f;
static const float DHG_IO_LABEL_Y = 312.f;
static const float DHG_IO_JACK_Y = 336.f;

// =============================================================================
// Theme + drawing helpers (anonymous namespace)
// =============================================================================
namespace
{

    // Panel background gradient start colour (top).
    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    // Panel background gradient end colour (bottom).
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    // Default body text colour.
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    // Muted colour for secondary labels.
    NVGcolor colDim() { return nvgRGB(0x8a, 0x95, 0xa4); }
    // Primary accent colour (cyan).
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    // Secondary accent colour (violet), used for meter gradients.
    NVGcolor colAccent2() { return nvgRGB(0xb0, 0x5a, 0xff); }

    // Loads the AGMM monospace font and makes it the current NanoVG font face.
    // Returns false if there is no window or the asset is missing.
    bool setFont(NVGcontext *vg)
    {
        if (!APP || !APP->window)
            return false;
        std::shared_ptr<window::Font> font =
            APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (!font)
            return false;
        nvgFontFaceId(vg, font->handle);
        return true;
    }

    // Draws a string at (x, y) with the given size, colour, alignment and
    // letter spacing.
    void drawText(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
                  int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f)
    {
        if (!setFont(vg))
            return;
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Describes one of the four TONE cells: label plus the three module IDs
    // (knob parameter, CV attenuverter, CV input) that it hosts.
    struct DhgToneDef
    {
        const char *name;
        int param, attn, cv;
    };
    const DhgToneDef DHG_TONE_CELLS[4] = {
        {"REPLACE", DarkHallGranular::REPLACE_PARAM, DarkHallGranular::REPLACE_ATTEN, DarkHallGranular::REPLACE_CV},
        {"BRIGHT", DarkHallGranular::BRIGHTNESS_PARAM, DarkHallGranular::BRIGHTNESS_ATTEN, DarkHallGranular::BRIGHTNESS_CV},
        {"DETUNE", DarkHallGranular::DETUNE_PARAM, DarkHallGranular::DETUNE_ATTEN, DarkHallGranular::DETUNE_CV},
        {"BIGNESS", DarkHallGranular::BIGNESS_PARAM, DarkHallGranular::BIGNESS_ATTEN, DarkHallGranular::BIGNESS_CV},
    };
    // Describes one of the three LEVELS cells: label plus the knob parameter.
    struct DhgLevelDef
    {
        const char *name;
        int param;
    };
    const DhgLevelDef DHG_LEVEL_CELLS[3] = {
        {"IN GAIN", DarkHallGranular::INPUT_GAIN_PARAM},
        {"MIX", DarkHallGranular::DRYWET_PARAM},
        {"OUT GAIN", DarkHallGranular::OUTPUT_GAIN_PARAM},
    };
    const char *DHG_IO_NAMES[5] = {"L IN", "R IN", "RESET", "L OUT", "R OUT"};

    // ------------------------------------------------------------
    // Panel background (static artwork only; no widgets overlap its text)
    // ------------------------------------------------------------
    struct DhgPanelBackground : TransparentWidget
    {
        DhgPanelBackground() { box.size = Vec(DHG_PANEL_W, DHG_PANEL_H); }

        // Draws a rounded control plate centred on cx with its top edge at `top`.
        // `output` tints the plate in accent cyan to mark output jacks.
        void plate(NVGcontext *vg, float cx, float top, float w, float h, bool output = false)
        {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - w / 2.f, top, w, h, 5.f);
            nvgFillColor(vg, output ? nvgRGBA(0x3d, 0xe0, 0xff, 22) : nvgRGBA(0, 0, 0, 55));
            nvgFill(vg);
            nvgStrokeColor(vg, output ? nvgRGBA(0x3d, 0xe0, 0xff, 60) : nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Section title with hairlines that stop a measured distance from the text
        void section(NVGcontext *vg, float y, const char *txt)
        {
            const float x1 = 14.f, x2 = DHG_PANEL_W - 14.f, cx = DHG_PANEL_W / 2.f;
            if (!setFont(vg))
                return;
            nvgFontSize(vg, 8.f);
            nvgTextLetterSpacing(vg, 2.f);
            float b[4];
            nvgTextBounds(vg, 0.f, 0.f, txt, NULL, b);
            float tw = b[2] - b[0];
            float gap = tw / 2.f + 6.f;

            drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x1, y);
            nvgLineTo(vg, cx - gap, y);
            nvgMoveTo(vg, cx + gap, y);
            nvgLineTo(vg, x2, y);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Draws the full static panel: background gradient, title, maker tag,
        // display bezel, the TONE section with its four cells, the LEVELS
        // section with three cells, and the I/O section with five jack cells.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            // Base gradient
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, DHG_PANEL_W, DHG_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DHG_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            // Title + maker
            drawText(vg, DHG_PANEL_W / 2.f, DHG_TITLE_Y, "DARK HALL GRANULAR", 12.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
            drawText(vg, DHG_PANEL_W / 2.f, DHG_MAKER_Y, "AGMM", 9.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

            // Display bezel
            nvgBeginPath(vg);
            nvgRoundedRect(vg, DHG_DISP_X - 4.f, DHG_DISP_Y - 4.f,
                           DHG_DISP_W + 8.f, DHG_DISP_H + 8.f, 5.f);
            nvgFillColor(vg, nvgRGB(0x0b, 0x0c, 0x0e));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // ---- TONE ----
            section(vg, DHG_TONE_SEC_Y, "TONE");
            for (int i = 0; i < 4; ++i)
            {
                float cx = DHG_TONE_CX[i];
                plate(vg, cx, DHG_TONE_PLATE_TOP, DHG_TONE_PLATE_W, DHG_TONE_PLATE_H);
                drawText(vg, cx, DHG_TONE_LABEL_Y, DHG_TONE_CELLS[i].name, 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
                drawText(vg, cx - 13.f, DHG_TONE_SMALL_Y, "CV", 7.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
                drawText(vg, cx + 13.f, DHG_TONE_SMALL_Y, "ATT", 7.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            }

            // ---- LEVELS ----
            section(vg, DHG_LVL_SEC_Y, "LEVELS");
            for (int i = 0; i < 3; ++i)
            {
                float cx = DHG_LVL_CX[i];
                plate(vg, cx, DHG_LVL_PLATE_TOP, DHG_LVL_PLATE_W, DHG_LVL_PLATE_H);
                drawText(vg, cx, DHG_LVL_LABEL_Y, DHG_LEVEL_CELLS[i].name, 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            }

            // ---- I/O (inputs left, outputs right, outputs tinted) ----
            section(vg, DHG_IO_SEC_Y, "I / O");
            for (int i = 0; i < 5; ++i)
            {
                float cx = DHG_IO_CX[i];
                bool isOutput = (i >= 3);
                plate(vg, cx, DHG_IO_PLATE_TOP, DHG_IO_PLATE_W, DHG_IO_PLATE_H, isOutput);
                drawText(vg, cx, DHG_IO_LABEL_Y, DHG_IO_NAMES[i], 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            }

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope display + L/R level meters (no text drawn over the waveform)
    // ------------------------------------------------------------
    struct DhgDisplayWidget : TransparentWidget
    {
        DarkHallGranular *module = nullptr;

        DhgDisplayWidget(DarkHallGranular *m) : module(m)
        {
            box.pos = Vec(DHG_DISP_X, DHG_DISP_Y);
            box.size = Vec(DHG_DISP_W, DHG_DISP_H);
        }

        // Width of the scope area (the display width minus room for meters).
        static float scopeW() { return DHG_DISP_W - 16.f; }

        // Draws the dark display fill and its grid (vertical ticks and a centre
        // line). This runs on layer 0; the live waveform is drawn in drawLayer().
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, DHG_DISP_W, DHG_DISP_H);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
            nvgFill(vg);

            // Grid
            nvgBeginPath(vg);
            for (float x = 0.f; x <= scopeW(); x += 28.f)
            {
                nvgMoveTo(vg, x + 0.5f, 0);
                nvgLineTo(vg, x + 0.5f, DHG_DISP_H);
            }
            nvgMoveTo(vg, 0, DHG_DISP_H / 2.f + 0.5f);
            nvgLineTo(vg, scopeW(), DHG_DISP_H / 2.f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the scope waveform from the module's min/max
        // ring buffer (or a demo burst in the module browser) and the L/R level
        // meters with a cyan-to-violet gradient. Other layers just forward.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            const float mid = DHG_DISP_H / 2.f;
            const float amp = mid - 2.f;
            const float sw = scopeW();
            const float colW = sw / (float)DHG_SCOPE_N;

            // Waveform
            nvgSave(vg);
            nvgScissor(vg, 0, 0, sw, DHG_DISP_H);
            nvgBeginPath(vg);
            for (int i = 0; i < DHG_SCOPE_N; ++i)
            {
                float vmin, vmax;
                if (module)
                {
                    int idx = (module->scopeHead + i) % DHG_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                }
                else
                {
                    float t = (float)i / DHG_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - clampT(vmax / 5.f, -1.f, 1.f) * amp;
                float y1 = mid - clampT(vmin / 5.f, -1.f, 1.f) * amp;
                float h = std::max(y1 - y0, 1.f);
                nvgRect(vg, i * colW, y0, std::max(colW - 0.4f, 1.f), h);
            }
            nvgFillColor(vg, nvgRGB(0x8a, 0xee, 0xff));
            nvgFill(vg);
            nvgResetScissor(vg);
            nvgRestore(vg);

            // L / R level meters (right of the scope, never overlapping it)
            for (int ch = 0; ch < 2; ++ch)
            {
                float peak = module ? (ch == 0 ? module->peakL : module->peakR)
                                    : (ch == 0 ? 3.f : 2.4f);
                float lvl = std::min(peak / 10.f, 1.f);
                float x = sw + 4.f + ch * 6.f;
                nvgBeginPath(vg);
                nvgRect(vg, x, 1.f, 4.f, DHG_DISP_H - 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
                if (lvl > 0.01f)
                {
                    float h = lvl * (DHG_DISP_H - 2.f);
                    nvgBeginPath(vg);
                    nvgRect(vg, x, DHG_DISP_H - 1.f - h, 4.f, h);
                    nvgFillPaint(vg, nvgLinearGradient(vg, 0, DHG_DISP_H, 0, 0,
                                                       colAccent(), colAccent2()));
                    nvgFill(vg);
                }
            }

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// =============================================================================
// Module widget: adds the panel background, screws, the display widget, the
// four TONE controls (knob + CV jack + attenuverter), the three LEVELS
// knobs, the five I/O jacks, and the CV-range context menu.
// =============================================================================
struct DarkHallGranularWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, live display,
    // TONE cells, LEVELS knobs and I/O jacks.
    DarkHallGranularWidget(DarkHallGranular *module)
    {
        setModule(module);
        box.size = Vec(DHG_PANEL_W, DHG_PANEL_H);

        addChild(new DhgPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new DhgDisplayWidget(module));

        // TONE: knob, then CV jack (left) + attenuverter (right) below it
        for (int i = 0; i < 4; ++i)
        {
            float cx = DHG_TONE_CX[i];
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(cx, DHG_TONE_KNOB_Y), module, DHG_TONE_CELLS[i].param));
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(cx - 13.f, DHG_TONE_CV_Y), module, DHG_TONE_CELLS[i].cv));
            addParam(createParamCentered<Trimpot>(
                Vec(cx + 13.f, DHG_TONE_CV_Y), module, DHG_TONE_CELLS[i].attn));
        }

        // LEVELS: in gain, mix, out gain
        for (int i = 0; i < 3; ++i)
        {
            addParam(createParamCentered<RoundBlackKnob>(
                Vec(DHG_LVL_CX[i], DHG_LVL_KNOB_Y), module, DHG_LEVEL_CELLS[i].param));
        }

        // I/O: L IN, R IN, RESET | L OUT, R OUT
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(DHG_IO_CX[0], DHG_IO_JACK_Y), module, DarkHallGranular::IN_L));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(DHG_IO_CX[1], DHG_IO_JACK_Y), module, DarkHallGranular::IN_R));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(DHG_IO_CX[2], DHG_IO_JACK_Y), module, DarkHallGranular::RESET_IN));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(DHG_IO_CX[3], DHG_IO_JACK_Y), module, DarkHallGranular::OUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(DHG_IO_CX[4], DHG_IO_JACK_Y), module, DarkHallGranular::OUT_R));
    }

    // Adds the right-click "CV Range" submenu with two mutually exclusive,
    // checkmarked options that set the module's CV interpretation.
    void appendContextMenu(Menu *menu) override
    {
        DarkHallGranular *module = dynamic_cast<DarkHallGranular *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem
        {
            DarkHallGranular *module;
            DarkHallGranular::CVRange range;
            // Sets the module's CV range mode to this item's range.
            void onAction(const ActionEvent &e) override
            {
                module->cvRangeMode = range;
            }
            // Shows a checkmark when this item's range is the active mode.
            void step() override
            {
                rightText = CHECKMARK(module->cvRangeMode == range);
                MenuItem::step();
            }
        };

        CVRangeItem *bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = DarkHallGranular::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);

        CVRangeItem *unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar)");
        unipolarItem->module = module;
        unipolarItem->range = DarkHallGranular::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);
    }
};

Model *modelDarkHallGranular =
    createModel<DarkHallGranular, DarkHallGranularWidget>("DarkHallGranular");