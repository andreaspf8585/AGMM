#include "plugin.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <vector>
#include <memory>
#include <cstdio>

static const int DHG_SCOPE_N = 96;

// =============================================================================
// Galaxy-style stereo reverb — DSP preserved from original
// =============================================================================

template<typename T>
static inline T clampT(T v, T lo, T hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float sanitize(float x) {
    if (!std::isfinite(x)) return 0.f;
    if (std::fabs(x) < 1e-25f) return 0.f;
    return x;
}

static inline float softClip(float x, float limit) {
    return limit * std::tanh(x / limit);
}

struct DelayLine {
    std::vector<float> buf;
    int size  = 0;
    int write = 0;

    void init(int n) {
        size = n;
        buf.assign(n, 0.f);
        write = 0;
    }
    void clear() {
        std::fill(buf.begin(), buf.end(), 0.f);
        write = 0;
    }
    inline void push(float x) {
        buf[write] = x;
        if (++write >= size) write = 0;
    }
    inline float read(float delay) const {
        delay = clampT(delay, 2.f, (float)(size - 4));
        float pos = (float)write - 1.f - delay + 1.f;
        pos -= 1.f;
        while (pos < 0.f) pos += (float)size;
        int i1 = (int)pos;
        float t = pos - (float)i1;
        if (i1 >= size) i1 -= size;
        int i0 = i1 - 1; if (i0 < 0) i0 += size;
        int i2 = i1 + 1; if (i2 >= size) i2 -= size;
        int i3 = i1 + 2; if (i3 >= size) i3 -= size;
        float y0 = buf[i0], y1 = buf[i1], y2 = buf[i2], y3 = buf[i3];
        float a = -0.5f*y0 + 1.5f*y1 - 1.5f*y2 + 0.5f*y3;
        float b =      y0 - 2.5f*y1 + 2.0f*y2 - 0.5f*y3;
        float c = -0.5f*y0             + 0.5f*y2;
        return ((a*t + b)*t + c)*t + y1;
    }
    inline float allpass(float in, float delay, float g) {
        float d = read(delay);
        float w = in - g * d;
        push(sanitize(w));
        return d + g * w;
    }
};

struct DarkHallGranular : Module {

    static constexpr float AP_IN[4]  = { 150.f, 214.f, 319.f, 527.f };
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

    enum CVRange { BIPOLAR_M5_5, UNIPOLAR_0_10 };
    CVRange cvRangeMode = BIPOLAR_M5_5;

    // ---- display state ----
    float scopeMin[DHG_SCOPE_N] = {};
    float scopeMax[DHG_SCOPE_N] = {};
    int   scopeHead = 0;
    int   scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    enum ParamId {
        REPLACE_PARAM, BRIGHTNESS_PARAM, DETUNE_PARAM, BIGNESS_PARAM,
        DRYWET_PARAM,
        INPUT_GAIN_PARAM, OUTPUT_GAIN_PARAM,
        REPLACE_ATTEN, BRIGHTNESS_ATTEN, DETUNE_ATTEN, BIGNESS_ATTEN,
        PARAMS_LEN
    };
    enum InputId {
        IN_L, IN_R,
        REPLACE_CV, BRIGHTNESS_CV, DETUNE_CV, BIGNESS_CV,
        RESET_IN,
        INPUTS_LEN
    };
    enum OutputId { OUT_L, OUT_R, OUTPUTS_LEN };

    DarkHallGranular() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);

        configParam(REPLACE_PARAM,     0.f, 1.f, 0.20f, "Replace");
        configParam(BRIGHTNESS_PARAM,  0.f, 1.f, 0.45f, "Brightness");
        configParam(DETUNE_PARAM,      0.f, 1.f, 0.50f, "Detune");
        configParam(BIGNESS_PARAM,     0.f, 1.f, 0.70f, "Bigness");
        configParam(DRYWET_PARAM,      0.f, 1.f, 0.50f, "Dry/Wet");
        configParam(INPUT_GAIN_PARAM,  0.f, 2.f, 1.0f,  "Input Gain");
        configParam(OUTPUT_GAIN_PARAM, 0.f, 2.f, 1.0f,  "Output Gain");

        configParam(REPLACE_ATTEN,    -1.f, 1.f, 0.f, "Replace CV");
        configParam(BRIGHTNESS_ATTEN, -1.f, 1.f, 0.f, "Brightness CV");
        configParam(DETUNE_ATTEN,     -1.f, 1.f, 0.f, "Detune CV");
        configParam(BIGNESS_ATTEN,    -1.f, 1.f, 0.f, "Bigness CV");

        configInput(IN_L, "Left In");
        configInput(IN_R, "Right In");
        configInput(REPLACE_CV,    "Replace CV");
        configInput(BRIGHTNESS_CV, "Brightness CV");
        configInput(DETUNE_CV,     "Detune CV");
        configInput(BIGNESS_CV,    "Bigness CV");
        configInput(RESET_IN,      "Reset");

        configOutput(OUT_L, "Left Out");
        configOutput(OUT_R, "Right Out");

        for (int i = 0; i < 4; ++i) apIn[i].init(4096);
        dap1a.init(32768); dap1b.init(32768);
        dap2a.init(32768); dap2b.init(32768);
        del1.init(131072); del2.init(131072);
    }

    void resetState() {
        for (int i = 0; i < 4; ++i) apIn[i].clear();
        dap1a.clear(); dap1b.clear(); del1.clear();
        dap2a.clear(); dap2b.clear(); del2.clear();
        lp1 = lp2 = 0.f;
        lfo1 = 0.f; lfo2 = 0.25f; lfo3 = 0.5f; lfo4 = 0.75f;
        dcInL = dcInR = dcOutL = dcOutR = 0.f;
    }

    void onReset() override { resetState(); }

    json_t *dataToJson() override {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        return root;
    }
    void dataFromJson(json_t *root) override {
        json_t *mode = json_object_get(root, "cvRangeMode");
        if (mode) cvRangeMode = (CVRange)json_integer_value(mode);
    }

    float cvParam(int param, int cvIn, int atten) {
        float cv = sanitize(inputs[cvIn].getVoltage());
        float scaled;
        if (cvRangeMode == UNIPOLAR_0_10)
            scaled = cv / 10.f;
        else
            scaled = clampT(cv / 5.f, -1.f, 1.f);
        return clampT(params[param].getValue()
                      + scaled * params[atten].getValue(), 0.f, 1.f);
    }

    void process(const ProcessArgs &args) override {
        if (args.sampleTime <= 0.f) return;

        bool resetHigh = inputs[RESET_IN].getVoltage() > 1.f;
        if (resetHigh && !wasResetHigh) resetState();
        wasResetHigh = resetHigh;

        if (!outputs[OUT_L].isConnected() && !outputs[OUT_R].isConnected()) {
            outputs[OUT_L].setVoltage(0.f);
            outputs[OUT_R].setVoltage(0.f);
            return;
        }
        bool inputConnected = inputs[IN_L].isConnected() || inputs[IN_R].isConnected();
        if (!inputConnected) {
            if (wasConnected) { resetState(); wasConnected = false; }
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
        float replace = cvParam(REPLACE_PARAM,    REPLACE_CV,    REPLACE_ATTEN);
        float bright  = cvParam(BRIGHTNESS_PARAM, BRIGHTNESS_CV, BRIGHTNESS_ATTEN);
        float detune  = cvParam(DETUNE_PARAM,     DETUNE_CV,     DETUNE_ATTEN);
        float bigness = cvParam(BIGNESS_PARAM,    BIGNESS_CV,    BIGNESS_ATTEN);
        float dryWet  = clampT(params[DRYWET_PARAM].getValue(), 0.f, 1.f);
        float inGain  = params[INPUT_GAIN_PARAM].getValue();
        float outGain = params[OUTPUT_GAIN_PARAM].getValue();

        // ---- Input ---------------------------------------------------------
        float inL = softClip(sanitize(inputs[IN_L].getVoltage()), 8.f) * inGain;
        float inR = inputs[IN_R].isConnected()
                  ? softClip(sanitize(inputs[IN_R].getVoltage()), 8.f) * inGain
                  : inL;
        if (!inputs[IN_L].isConnected()) inL = inR;

        // ---- Derived values ------------------------------------------------
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

        lfo1 += args.sampleTime * 0.50f;
        lfo2 += args.sampleTime * 0.31f;
        lfo3 += args.sampleTime * 0.23f;
        lfo4 += args.sampleTime * 0.37f;
        if (lfo1 > 1.f) lfo1 -= 1.f;
        if (lfo2 > 1.f) lfo2 -= 1.f;
        if (lfo3 > 1.f) lfo3 -= 1.f;
        if (lfo4 > 1.f) lfo4 -= 1.f;
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
        dcInL = revL;  dcOutL = sanitize(wetL);
        dcInR = revR;  dcOutR = sanitize(wetR);

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
        float decayRate = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(vL), peakL * decayRate);
        peakR = std::max(std::fabs(vR), peakR * decayRate);

        float mono = 0.5f * (vL + vR);
        if (scopeCount == 0) scopeBMin = scopeBMax = mono;
        else {
            scopeBMin = std::min(scopeBMin, mono);
            scopeBMax = std::max(scopeBMax, mono);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket) {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % DHG_SCOPE_N;
        }
    }
};

constexpr float DarkHallGranular::AP_IN[4];

// =============================================================================
// Layout constants (18 HP = 270 px)
// =============================================================================
static const float DHG_PANEL_W = RACK_GRID_WIDTH * 18;   // 270
static const float DHG_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float DHG_DISP_W = 226.f;
static const float DHG_DISP_H = 34.f;
static const float DHG_DISP_X = (DHG_PANEL_W - DHG_DISP_W) / 2.f;
static const float DHG_DISP_Y = 46.f;

static const float DHG_MAIN_ROW_Y = 145.f;
static const float DHG_GAIN_ROW_Y = 250.f;
static const float DHG_IO_ROW_Y   = 320.f;

// Column X positions
static const float DHG_MAIN_CX[5] = {27.f, 81.f, 135.f, 189.f, 243.f};
static const float DHG_GAIN_CX[2] = {90.f, 180.f};

// =============================================================================
// Theme + drawing helpers (anonymous namespace)
// =============================================================================
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colAccent2()  { return nvgRGB(0xb0, 0x5a, 0xff); }

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

struct DhgCellDef { const char* name; int param, attn, cv; };
const DhgCellDef DHG_MAIN_CELLS[5] = {
    {"REPLACE", DarkHallGranular::REPLACE_PARAM,    DarkHallGranular::REPLACE_ATTEN,    DarkHallGranular::REPLACE_CV},
    {"BRIGHT",  DarkHallGranular::BRIGHTNESS_PARAM, DarkHallGranular::BRIGHTNESS_ATTEN, DarkHallGranular::BRIGHTNESS_CV},
    {"DETUNE",  DarkHallGranular::DETUNE_PARAM,     DarkHallGranular::DETUNE_ATTEN,     DarkHallGranular::DETUNE_CV},
    {"BIGNESS", DarkHallGranular::BIGNESS_PARAM,    DarkHallGranular::BIGNESS_ATTEN,    DarkHallGranular::BIGNESS_CV},
    {"MIX",     DarkHallGranular::DRYWET_PARAM,     -1, -1},
};
struct DhgGainDef { const char* name; int param; };
const DhgGainDef DHG_GAIN_CELLS[2] = {
    {"IN GAIN",  DarkHallGranular::INPUT_GAIN_PARAM},
    {"OUT GAIN", DarkHallGranular::OUTPUT_GAIN_PARAM},
};

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct DhgPanelBackground : TransparentWidget {
    DhgPanelBackground() { box.size = Vec(DHG_PANEL_W, DHG_PANEL_H); }

    void plate(NVGcontext* vg, float cx, float ry, float w, float h) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - w / 2.f, ry - h / 2.f, w, h, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = DHG_PANEL_W - 14.f, cx = DHG_PANEL_W / 2.f;
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
        nvgRect(vg, 0, 0, DHG_PANEL_W, DHG_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DHG_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) DHG_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, DHG_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, DHG_PANEL_W - 1.f, DHG_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, DHG_PANEL_W / 2.f, 13.f);
        drawText(vg, DHG_PANEL_W / 2.f, 31.f, "DARK HALL GRAN", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // Display bezel
        float bx = DHG_DISP_X - 5.f, by = DHG_DISP_Y - 5.f;
        float bw = DHG_DISP_W + 10.f, bh = DHG_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- Main section (5 cells with CV + atten) ----
        section(vg, 100.f, "MAIN");
        for (int i = 0; i < 5; ++i) {
            float cx = DHG_MAIN_CX[i];
            float ry = DHG_MAIN_ROW_Y;
            plate(vg, cx, ry, 50.f, 70.f);
            drawText(vg, cx, ry + 21.f, DHG_MAIN_CELLS[i].name, 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // ---- Gains section (2 knobs, no CV) ----
        section(vg, 215.f, "GAINS");
        for (int i = 0; i < 2; ++i) {
            float cx = DHG_GAIN_CX[i];
            float ry = DHG_GAIN_ROW_Y;
            plate(vg, cx, ry, 60.f, 50.f);
            drawText(vg, cx, ry + 18.f, DHG_GAIN_CELLS[i].name, 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // ---- I/O section (5 jacks) ----
        section(vg, 295.f, "I / O");
        const char* io[5] = {"L IN", "R IN", "L OUT", "R OUT", "RESET"};
        for (int i = 0; i < 5; ++i) {
            float cx = DHG_MAIN_CX[i];
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 20.f, 300.f, 40.f, 40.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, cx, 308.f, io[i], 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Scope display + L/R level meters
// ------------------------------------------------------------
struct DhgDisplayWidget : TransparentWidget {
    DarkHallGranular* module = nullptr;

    DhgDisplayWidget(DarkHallGranular* m) : module(m) {
        box.pos = Vec(DHG_DISP_X, DHG_DISP_Y);
        box.size = Vec(DHG_DISP_W, DHG_DISP_H);
    }

    static float scopeW() { return DHG_DISP_W - 16.f; }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DHG_DISP_W, DHG_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, DHG_DISP_W / 2.f, DHG_DISP_H / 2.f, 20.f,
                                           DHG_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        nvgBeginPath(vg);
        for (float x = 0.f; x <= scopeW(); x += 28.f) {
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

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        const float mid  = DHG_DISP_H / 2.f;
        const float amp  = mid - 2.f;
        const float sw   = scopeW();
        const float colW = sw / (float) DHG_SCOPE_N;

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, DHG_DISP_H);
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < DHG_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % DHG_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / DHG_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - std::min(std::max(vmax / 5.f, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / 5.f, -1.f), 1.f) * amp;
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
        nvgResetScissor(vg);
        nvgRestore(vg);

        // L / R level meters
        for (int ch = 0; ch < 2; ++ch) {
            float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 3.f : 2.4f);
            float lvl = std::min(peak / 10.f, 1.f);
            float x = sw + 3.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 1.f, 4.f, DHG_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (DHG_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, DHG_DISP_H - 1.f - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, DHG_DISP_H, 0, 0,
                                                   colAccent(), colAccent2()));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) DHG_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, DHG_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DHG_DISP_W, DHG_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DHG_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16),
                                           nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[32];
        float big = module ? module->params[DarkHallGranular::BIGNESS_PARAM].getValue() : 0.7f;
        float mx  = module ? module->params[DarkHallGranular::DRYWET_PARAM].getValue()  : 0.5f;
        snprintf(buf, sizeof(buf), "BIG %3.0f%%", big * 100.f);
        drawText(vg, 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "MIX %3.0f%%", mx * 100.f);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// =============================================================================
// Module widget
// =============================================================================
struct DarkHallGranularWidget : ModuleWidget {
    DarkHallGranularWidget(DarkHallGranular* module) {
        setModule(module);
        box.size = Vec(DHG_PANEL_W, DHG_PANEL_H);

        addChild(new DhgPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new DhgDisplayWidget(module));

        // Main section: 5 knob cells with CV jack + atten
        for (int i = 0; i < 5; ++i) {
            float cx = DHG_MAIN_CX[i], ry = DHG_MAIN_ROW_Y;
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, ry), module,
                DHG_MAIN_CELLS[i].param));
            if (DHG_MAIN_CELLS[i].cv >= 0) {
                addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 14.f, ry + 40.f), module,
                    DHG_MAIN_CELLS[i].cv));
                addParam(createParamCentered<Trimpot>(Vec(cx + 14.f, ry + 40.f), module,
                    DHG_MAIN_CELLS[i].attn));
            }
        }

        // Gains section: 2 knobs
        for (int i = 0; i < 2; ++i) {
            float cx = DHG_GAIN_CX[i], ry = DHG_GAIN_ROW_Y;
            addParam(createParamCentered<RoundBlackKnob>(Vec(cx, ry), module,
                DHG_GAIN_CELLS[i].param));
        }

        // I/O section: 5 jacks in one row
        const int inIds[3]  = {DarkHallGranular::IN_L, DarkHallGranular::IN_R, DarkHallGranular::RESET_IN};
        const int outIds[2] = {DarkHallGranular::OUT_L, DarkHallGranular::OUT_R};

        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(DHG_MAIN_CX[0], DHG_IO_ROW_Y), module, inIds[0]));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(DHG_MAIN_CX[1], DHG_IO_ROW_Y), module, inIds[1]));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(DHG_MAIN_CX[2], DHG_IO_ROW_Y), module, outIds[0]));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(DHG_MAIN_CX[3], DHG_IO_ROW_Y), module, outIds[1]));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(DHG_MAIN_CX[4], DHG_IO_ROW_Y), module, inIds[2]));
    }

    void appendContextMenu(Menu* menu) override {
        DarkHallGranular* module = dynamic_cast<DarkHallGranular*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem {
            DarkHallGranular* module;
            DarkHallGranular::CVRange range;
            void onAction(const ActionEvent& e) override {
                module->cvRangeMode = range;
            }
            void step() override {
                rightText = (module->cvRangeMode == range) ? "✔" : "";
                MenuItem::step();
            }
        };

        CVRangeItem* bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = DarkHallGranular::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);

        CVRangeItem* unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar)");
        unipolarItem->module = module;
        unipolarItem->range = DarkHallGranular::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);
    }
};

Model *modelDarkHallGranular =
    createModel<DarkHallGranular, DarkHallGranularWidget>("DarkHallGranular");