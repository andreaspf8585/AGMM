#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

static const int NUM_DELAYS = 6;
static const int MAX_DELAY = 96000;
static const int PRE_DELAY_MAX = 44100;
static const int BH_SCOPE_N = 96;

template <typename T>
static T clamp(T value, T min, T max)
{
    return std::fmax(min, std::fmin(max, value));
}

// ============================================================
// Module – DSP preserved, scope tracking + CV range added
// ============================================================
struct BlackHoleReverb : Module
{
    float delayL[NUM_DELAYS][MAX_DELAY] = {};
    float delayR[NUM_DELAYS][MAX_DELAY] = {};

    int writeIndex = 0;

    float preDelayBuffer[PRE_DELAY_MAX] = {};
    int preWrite = 0;

    float modPhase[NUM_DELAYS] = {};
    float filterStoreL[NUM_DELAYS] = {};
    float filterStoreR[NUM_DELAYS] = {};

    enum CVRange { BIPOLAR_M5_5, UNIPOLAR_0_10 };
    CVRange cvRangeMode = BIPOLAR_M5_5;

    // ---- display state ----
    float scopeMin[BH_SCOPE_N] = {};
    float scopeMax[BH_SCOPE_N] = {};
    int   scopeHead = 0;
    int   scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    enum ParamId
    {
        MIX_PARAM,
        FEEDBACK_PARAM,
        DAMP_PARAM,
        MOD_PARAM,
        MIX_ATTEN,
        FEEDBACK_ATTEN,
        DAMP_ATTEN,
        MOD_ATTEN,
        PARAMS_LEN
    };

    enum InputId
    {
        IN_L,
        IN_R,
        MIX_CV,
        FEEDBACK_CV,
        DAMP_CV,
        MOD_CV,
        INPUTS_LEN
    };

    enum OutputId
    {
        OUT_L,
        OUT_R,
        OUTPUTS_LEN
    };

    BlackHoleReverb()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);

        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Mix");
        configParam(FEEDBACK_PARAM, 0.f, 0.98f, 0.50f, "Feedback");
        configParam(DAMP_PARAM, 0.f, 1.f, 0.7f, "Darkness");
        configParam(MOD_PARAM, 0.f, 1.f, 0.5f, "Modulation");

        configParam(MIX_ATTEN, 0.f, 1.f, 0.f, "Mix CV Attenuator");
        configParam(FEEDBACK_ATTEN, 0.f, 1.f, 0.f, "Feedback CV Attenuator");
        configParam(DAMP_ATTEN, 0.f, 1.f, 0.f, "Darkness CV Attenuator");
        configParam(MOD_ATTEN, 0.f, 1.f, 0.f, "Modulation CV Attenuator");

        configInput(IN_L, "Left In");
        configInput(IN_R, "Right In");
        configInput(MIX_CV, "Mix CV");
        configInput(FEEDBACK_CV, "Feedback CV");
        configInput(DAMP_CV, "Darkness CV");
        configInput(MOD_CV, "Modulation CV");

        configOutput(OUT_L, "Left Out");
        configOutput(OUT_R, "Right Out");
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *mode = json_object_get(root, "cvRangeMode");
        if (mode)
            cvRangeMode = (CVRange)json_integer_value(mode);
    }

    float scaleCV(float v) const
    {
        if (cvRangeMode == UNIPOLAR_0_10)
            return v / 10.f;
        return clamp(v / 5.f, -1.f, 1.f);
    }

    void process(const ProcessArgs &args) override
    {
        if (args.sampleTime <= 0.0f)
            return;

        auto safe = [](float x) -> float
        {
            return std::isnormal(x) ? x : 0.0f;
        };

        float inL = safe(inputs[IN_L].getVoltage());
        float inR = safe(inputs[IN_R].getVoltage());

        float mixKnob = params[MIX_PARAM].getValue();
        float fbKnob = params[FEEDBACK_PARAM].getValue();
        float dampKnob = params[DAMP_PARAM].getValue();
        float modKnob = params[MOD_PARAM].getValue();

        float mixCV  = safe(scaleCV(inputs[MIX_CV].getVoltage()));
        float fbCV   = safe(scaleCV(inputs[FEEDBACK_CV].getVoltage()));
        float dampCV = safe(scaleCV(inputs[DAMP_CV].getVoltage()));
        float modCV  = safe(scaleCV(inputs[MOD_CV].getVoltage()));

        float mixAtten  = params[MIX_ATTEN].getValue();
        float fbAtten   = params[FEEDBACK_ATTEN].getValue();
        float dampAtten = params[DAMP_ATTEN].getValue();
        float modAtten  = params[MOD_ATTEN].getValue();

        float mix      = clamp(mixKnob  + mixCV  * mixAtten,  0.f, 1.f);
        float feedback = clamp(fbKnob   + fbCV   * fbAtten,   0.f, 0.98f);
        float damp     = clamp(dampKnob + dampCV * dampAtten, 0.f, 1.f);
        float modAmount = clamp(modKnob + modCV  * modAtten,  0.f, 1.f);

        int preDelaySamples = (int)(0.08f * args.sampleRate);
        preDelaySamples = clamp(preDelaySamples, 0, PRE_DELAY_MAX - 1);
        int preRead = preWrite - preDelaySamples;
        if (preRead < 0)
            preRead += PRE_DELAY_MAX;
        float preDelayed = preDelayBuffer[preRead];
        preDelayBuffer[preWrite] = (inL + inR) * 0.5f;
        preWrite = (preWrite + 1) % PRE_DELAY_MAX;

        float revInL = preDelayed;
        float revInR = preDelayed;

        float dampSq = damp * damp;

        static const int baseDelays[NUM_DELAYS] = {
            2741, 3347, 3823, 4421, 5101, 5897};

        float wetL = 0.f, wetR = 0.f;

        for (int i = 0; i < NUM_DELAYS; i++)
        {
            modPhase[i] += args.sampleTime * (0.08f + 0.04f * i);
            if (modPhase[i] > 1.f)
                modPhase[i] -= 1.f;
            float mod = std::sin(2.f * M_PI * modPhase[i]);
            int delay = baseDelays[i] + (int)(mod * 240.f * modAmount);
            int readIndex = writeIndex - delay;
            if (readIndex < 0)
                readIndex += MAX_DELAY;

            float dl = delayL[i][readIndex];
            filterStoreL[i] = dl * (1.f - dampSq) + filterStoreL[i] * dampSq;
            float fbL = filterStoreL[i];
            float cross = delayR[i][readIndex] * 0.4f;
            delayL[i][writeIndex] = revInL + (fbL + cross) * feedback;
            wetL += dl;

            float dr = delayR[i][readIndex];
            filterStoreR[i] = dr * (1.f - dampSq) + filterStoreR[i] * dampSq;
            float fbR = filterStoreR[i];
            float crossR = delayL[i][readIndex] * 0.4f;
            delayR[i][writeIndex] = revInR + (fbR + crossR) * feedback;
            wetR += dr;
        }

        writeIndex++;
        if (writeIndex >= MAX_DELAY)
            writeIndex = 0;

        wetL /= NUM_DELAYS;
        wetR /= NUM_DELAYS;

        const float wetBoost = 1.4f;
        wetL *= wetBoost;
        wetR *= wetBoost;

        float outL = (1.f - mix) * inL + mix * wetL;
        float outR = (1.f - mix) * inR + mix * wetR;

        const float clipLevel = 10.f;
        outL = clipLevel * std::tanh(outL / clipLevel);
        outR = clipLevel * std::tanh(outR / clipLevel);

        outputs[OUT_L].setVoltage(safe(outL));
        outputs[OUT_R].setVoltage(safe(outR));

        // ---- display data ----
        float decay = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(outL), peakL * decay);
        peakR = std::max(std::fabs(outR), peakR * decay);

        float mono = 0.5f * (outL + outR);
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
            scopeHead = (scopeHead + 1) % BH_SCOPE_N;
        }
    }
};

// ============================================================
// Layout constants (12 HP = 180 px)
// ============================================================
static const float BH_PANEL_W = RACK_GRID_WIDTH * 12;   // 180
static const float BH_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float BH_DISP_W = 150.f;
static const float BH_DISP_H = 34.f;
static const float BH_DISP_X = (BH_PANEL_W - BH_DISP_W) / 2.f;
static const float BH_DISP_Y = 46.f;

static const float BH_CELL_W = 60.f;
static const float BH_CELL_H = 70.f;
static const float BH_ROW[2] = {125.f, 197.f};
static const float BH_COL[2] = {45.f, 135.f};
static const float BH_PORT_Y = 304.f;

// ============================================================
// Theme + drawing helpers (anonymous namespace)
// ============================================================
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

struct BhCellDef { const char* name; int param, attn, cv; };
const BhCellDef BH_CELLS[4] = {
    {"MIX",   BlackHoleReverb::MIX_PARAM,      BlackHoleReverb::MIX_ATTEN,      BlackHoleReverb::MIX_CV},
    {"FEED",  BlackHoleReverb::FEEDBACK_PARAM, BlackHoleReverb::FEEDBACK_ATTEN, BlackHoleReverb::FEEDBACK_CV},
    {"DAMP",  BlackHoleReverb::DAMP_PARAM,     BlackHoleReverb::DAMP_ATTEN,     BlackHoleReverb::DAMP_CV},
    {"MOD",   BlackHoleReverb::MOD_PARAM,      BlackHoleReverb::MOD_ATTEN,      BlackHoleReverb::MOD_CV},
};

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct BhPanelBackground : TransparentWidget {
    BhPanelBackground() { box.size = Vec(BH_PANEL_W, BH_PANEL_H); }

    void plate(NVGcontext* vg, float cx, float ry) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - BH_CELL_W / 2.f, ry - 17.f, BH_CELL_W, BH_CELL_H, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = BH_PANEL_W - 14.f, cx = BH_PANEL_W / 2.f;
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
        nvgRect(vg, 0, 0, BH_PANEL_W, BH_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, BH_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) BH_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, BH_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, BH_PANEL_W - 1.f, BH_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, BH_PANEL_W / 2.f, 13.f);
        drawText(vg, BH_PANEL_W / 2.f, 31.f, "BLACK HOLE", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // Display bezel
        float bx = BH_DISP_X - 5.f, by = BH_DISP_Y - 5.f;
        float bw = BH_DISP_W + 10.f, bh = BH_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // 4 control cells
        for (int i = 0; i < 4; ++i) {
            int row = i / 2, col = i % 2;
            float cx = BH_COL[col], ry = BH_ROW[row];
            plate(vg, cx, ry);
            drawText(vg, cx, ry + 21.f, BH_CELLS[i].name, 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // I/O section
        section(vg, 270.f, "I / O");
        const char* io[4] = {"L IN", "R IN", "L OUT", "R OUT"};
        for (int i = 0; i < 4; ++i) {
            float cx = 22.5f + i * 45.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 20.f, 278.f, 40.f, 42.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, cx, 286.f, io[i], 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Scope display + L/R level meters
// ------------------------------------------------------------
struct BhDisplayWidget : TransparentWidget {
    BlackHoleReverb* module = nullptr;

    BhDisplayWidget(BlackHoleReverb* m) : module(m) {
        box.pos = Vec(BH_DISP_X, BH_DISP_Y);
        box.size = Vec(BH_DISP_W, BH_DISP_H);
    }

    static float scopeW() { return BH_DISP_W - 16.f; }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, BH_DISP_W, BH_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, BH_DISP_W / 2.f, BH_DISP_H / 2.f, 20.f,
                                           BH_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        nvgBeginPath(vg);
        for (float x = 0.f; x <= scopeW(); x += 28.f) {
            nvgMoveTo(vg, x + 0.5f, 0);
            nvgLineTo(vg, x + 0.5f, BH_DISP_H);
        }
        nvgMoveTo(vg, 0, BH_DISP_H / 2.f + 0.5f);
        nvgLineTo(vg, scopeW(), BH_DISP_H / 2.f + 0.5f);
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

        const float mid  = BH_DISP_H / 2.f;
        const float amp  = mid - 2.f;
        const float sw   = scopeW();
        const float colW = sw / (float) BH_SCOPE_N;

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, BH_DISP_H);
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < BH_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % BH_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / BH_SCOPE_N;
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
            nvgRect(vg, x, 1.f, 4.f, BH_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (BH_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, BH_DISP_H - 1.f - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, BH_DISP_H, 0, 0,
                                                   colAccent(), colAccent2()));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) BH_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, BH_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, BH_DISP_W, BH_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, BH_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16),
                                           nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[32];
        float mx = module ? module->params[BlackHoleReverb::MIX_PARAM].getValue()      : 0.5f;
        float fb = module ? module->params[BlackHoleReverb::FEEDBACK_PARAM].getValue() : 0.5f;
        snprintf(buf, sizeof(buf), "MIX %3.0f%%", mx * 100.f);
        drawText(vg, 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "FB %3.0f%%", fb * 100.f);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ============================================================
// Module widget
// ============================================================
struct BlackHoleReverbWidget : ModuleWidget {
    BlackHoleReverbWidget(BlackHoleReverb* module) {
        setModule(module);
        box.size = Vec(BH_PANEL_W, BH_PANEL_H);

        addChild(new BhPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new BhDisplayWidget(module));

        // 4 knob cells (2x2)
        for (int i = 0; i < 4; ++i) {
            int row = i / 2, col = i % 2;
            float cx = BH_COL[col], ry = BH_ROW[row];
            const BhCellDef& c = BH_CELLS[i];
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, ry), module, c.param));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 16.f, ry + 40.f), module, c.cv));
            addParam(createParamCentered<Trimpot>(Vec(cx + 16.f, ry + 40.f), module, c.attn));
        }

        // I/O – 4 jacks in a row: L IN, R IN, L OUT, R OUT
        const int ioIds[4] = {
            BlackHoleReverb::IN_L,
            BlackHoleReverb::IN_R,
            BlackHoleReverb::OUT_L,
            BlackHoleReverb::OUT_R,
        };
        for (int i = 0; i < 4; ++i) {
            float cx = 22.5f + i * 45.f;
            if (i < 2)
                addInput(createInputCentered<DarkPJ301MPort>(Vec(cx, BH_PORT_Y), module, ioIds[i]));
            else
                addOutput(createOutputCentered<DarkPJ301MPort>(Vec(cx, BH_PORT_Y), module, ioIds[i]));
        }
    }

    void appendContextMenu(Menu* menu) override {
        BlackHoleReverb* module = dynamic_cast<BlackHoleReverb*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem {
            BlackHoleReverb* module;
            BlackHoleReverb::CVRange range;
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
        bipolarItem->range = BlackHoleReverb::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);

        CVRangeItem* unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar)");
        unipolarItem->module = module;
        unipolarItem->range = BlackHoleReverb::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);
    }
};

Model *modelBlackHoleReverb = createModel<BlackHoleReverb, BlackHoleReverbWidget>("BlackHoleReverb");