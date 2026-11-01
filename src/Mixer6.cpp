#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <string>

static const int MX_SCOPE_N = 96;

// ============================================================
// Stereo 6-channel mixer – per-channel L/R inputs, L/R outputs
// ============================================================
struct Mixer6 : Module {
    enum CVRange { BIPOLAR_M5_5, UNIPOLAR_0_10 };
    CVRange cvRangeMode = UNIPOLAR_0_10; // preserve original (cv/10) behavior

    // Display state
    float scopeMin[MX_SCOPE_N] = {};
    float scopeMax[MX_SCOPE_N] = {};
    int   scopeHead = 0;
    int   scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    enum ParamId {
        PARAM_CH1_LEVEL, PARAM_CH2_LEVEL, PARAM_CH3_LEVEL,
        PARAM_CH4_LEVEL, PARAM_CH5_LEVEL, PARAM_CH6_LEVEL,
        PARAM_CH1_ATTV,  PARAM_CH2_ATTV,  PARAM_CH3_ATTV,
        PARAM_CH4_ATTV,  PARAM_CH5_ATTV,  PARAM_CH6_ATTV,
        PARAM_MASTER_GAIN,
        PARAMS_LEN
    };
    enum InputId {
        // Interleaved so INPUT_CH1_L + ch*2 gives (L,R) for channel ch
        INPUT_CH1_L, INPUT_CH1_R,
        INPUT_CH2_L, INPUT_CH2_R,
        INPUT_CH3_L, INPUT_CH3_R,
        INPUT_CH4_L, INPUT_CH4_R,
        INPUT_CH5_L, INPUT_CH5_R,
        INPUT_CH6_L, INPUT_CH6_R,
        INPUT_CH1_CV, INPUT_CH2_CV, INPUT_CH3_CV,
        INPUT_CH4_CV, INPUT_CH5_CV, INPUT_CH6_CV,
        INPUTS_LEN
    };
    enum OutputId { OUTPUT_L, OUTPUT_R, OUTPUTS_LEN };
    enum LightId  { LIGHTS_LEN };

    Mixer6() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        for (int ch = 0; ch < 6; ++ch)
            configParam(PARAM_CH1_LEVEL + ch, 0.f, 1.f, 0.f,
                        "Channel " + std::to_string(ch+1) + " level");

        for (int ch = 0; ch < 6; ++ch)
            configParam(PARAM_CH1_ATTV + ch, -1.f, 1.f, 0.f,
                        "Channel " + std::to_string(ch+1) + " CV attenuverter");

        configParam(PARAM_MASTER_GAIN, 0.f, 2.f, 1.f, "Master gain");

        for (int ch = 0; ch < 6; ++ch) {
            std::string n = std::to_string(ch + 1);
            configInput(INPUT_CH1_L + ch * 2,     "Channel " + n + " L audio");
            configInput(INPUT_CH1_L + ch * 2 + 1, "Channel " + n + " R audio");
            configInput(INPUT_CH1_CV + ch,        "Channel " + n + " level CV");
        }

        configOutput(OUTPUT_L, "Mix L output");
        configOutput(OUTPUT_R, "Mix R output");
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        return root;
    }
    void dataFromJson(json_t* root) override {
        json_t* mode = json_object_get(root, "cvRangeMode");
        if (mode) cvRangeMode = (CVRange)json_integer_value(mode);
    }

    float scaleCV(float v) const {
        if (cvRangeMode == BIPOLAR_M5_5)
            return std::max(-1.f, std::min(v / 5.f, 1.f));
        return v / 10.f;  // UNIPOLAR_0_10 – original behavior
    }

    void process(const ProcessArgs& args) override {
        float mixL = 0.f;
        float mixR = 0.f;

        for (int ch = 0; ch < 6; ++ch) {
            float levelKnob    = params[PARAM_CH1_LEVEL + ch].getValue();
            float attenuverter = params[PARAM_CH1_ATTV + ch].getValue();

            float cv       = inputs[INPUT_CH1_CV + ch].getVoltage();
            float cvOffset = scaleCV(cv) * attenuverter;

            float gain = levelKnob + cvOffset;
            if (gain < 0.f) gain = 0.f;
            if (gain > 1.f) gain = 1.f;

            float inL = inputs[INPUT_CH1_L + ch * 2].getVoltage();
            float inR = inputs[INPUT_CH1_L + ch * 2 + 1].getVoltage();

            mixL += inL * gain;
            mixR += inR * gain;
        }

        float masterGain = params[PARAM_MASTER_GAIN].getValue();
        float outL = mixL * masterGain;
        float outR = mixR * masterGain;

        outputs[OUTPUT_L].setVoltage(outL);
        outputs[OUTPUT_R].setVoltage(outR);

        // ---- display tracking ----
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
            scopeHead = (scopeHead + 1) % MX_SCOPE_N;
        }
    }
};

// ============================================================
// Layout constants (26 HP = 390 px)
// ============================================================
static const float MX_PANEL_W = RACK_GRID_WIDTH * 26;   // 390
static const float MX_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float MX_DISP_W = 340.f;
static const float MX_DISP_H = 34.f;
static const float MX_DISP_X = (MX_PANEL_W - MX_DISP_W) / 2.f;  // 25
static const float MX_DISP_Y = 46.f;

// 6 channel columns, 60px apart
static const float MX_COL_X[6] = {45.f, 105.f, 165.f, 225.f, 285.f, 345.f};
static const float MX_JACK_OFFSET = 15.f; // L at cx-15, R at cx+15

// Per-channel Y positions (same as original mono layout)
static const float MX_CH_LABEL_Y  = 112.f;
static const float MX_AUDIO_Y     = 132.f;
static const float MX_AUDIO_LBL_Y = 148.f;
static const float MX_LEVEL_Y     = 168.f;
static const float MX_LEVEL_LBL_Y = 188.f;
static const float MX_CV_Y        = 205.f;
static const float MX_CV_LBL_Y    = 222.f;
static const float MX_ATTV_Y      = 238.f;
static const float MX_ATTV_LBL_Y  = 256.f;

// Master row Y positions
static const float MX_MASTER_DIV_Y  = 278.f;
static const float MX_GAIN_Y        = 305.f;
static const float MX_GAIN_LBL_Y    = 328.f;
static const float MX_OUTPUT_Y      = 352.f;
static const float MX_OUTPUT_LBL_Y  = 372.f;

static const float MX_MASTER_OUT_OFFSET = 45.f; // L/R outputs from center

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

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct MxPanelBackground : TransparentWidget {
    MxPanelBackground() { box.size = Vec(MX_PANEL_W, MX_PANEL_H); }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = MX_PANEL_W - 14.f, cx = MX_PANEL_W / 2.f;
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
        nvgRect(vg, 0, 0, MX_PANEL_W, MX_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, MX_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) MX_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, MX_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, MX_PANEL_W - 1.f, MX_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, MX_PANEL_W / 2.f, 13.f);
        drawText(vg, MX_PANEL_W / 2.f, 31.f, "MIXER 6 STEREO", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // Display bezel
        float bx = MX_DISP_X - 5.f, by = MX_DISP_Y - 5.f;
        float bw = MX_DISP_W + 10.f, bh = MX_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- CHANNELS section ----
        section(vg, 92.f, "CHANNELS");

        // Soft plate behind all channels
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 8.f, 100.f, MX_PANEL_W - 16.f, 165.f, 8.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 30));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 14));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Per-channel labels
        char buf[16];
        for (int ch = 0; ch < 6; ++ch) {
            float cx = MX_COL_X[ch];
            float lx = cx - MX_JACK_OFFSET;
            float rx = cx + MX_JACK_OFFSET;

            snprintf(buf, sizeof(buf), "CH %d", ch + 1);
            drawText(vg, cx, MX_CH_LABEL_Y, buf, 8.f, colAccent(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

            // L / R under the stereo input jack pair
            drawText(vg, lx, MX_AUDIO_LBL_Y, "L", 7.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            drawText(vg, rx, MX_AUDIO_LBL_Y, "R", 7.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

            drawText(vg, cx, MX_LEVEL_LBL_Y, "LEVEL", 6.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            drawText(vg, cx, MX_CV_LBL_Y, "CV", 6.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            drawText(vg, cx, MX_ATTV_LBL_Y, "ATT", 6.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // ---- MASTER section ----
        section(vg, MX_MASTER_DIV_Y, "MASTER");

        drawText(vg, MX_PANEL_W / 2.f, MX_GAIN_LBL_Y, "GAIN", 6.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        // L / R output labels
        float lOutX = MX_PANEL_W / 2.f - MX_MASTER_OUT_OFFSET;
        float rOutX = MX_PANEL_W / 2.f + MX_MASTER_OUT_OFFSET;
        drawText(vg, lOutX, MX_OUTPUT_LBL_Y, "L OUT", 7.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, rOutX, MX_OUTPUT_LBL_Y, "R OUT", 7.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Scope display + L/R level meters
// ------------------------------------------------------------
struct MxDisplayWidget : TransparentWidget {
    Mixer6* module = nullptr;

    MxDisplayWidget(Mixer6* m) : module(m) {
        box.pos = Vec(MX_DISP_X, MX_DISP_Y);
        box.size = Vec(MX_DISP_W, MX_DISP_H);
    }

    static float scopeW() { return MX_DISP_W - 20.f; }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, MX_DISP_W, MX_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, MX_DISP_W / 2.f, MX_DISP_H / 2.f, 20.f,
                                           MX_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        nvgBeginPath(vg);
        for (float x = 0.f; x <= scopeW(); x += 28.f) {
            nvgMoveTo(vg, x + 0.5f, 0);
            nvgLineTo(vg, x + 0.5f, MX_DISP_H);
        }
        nvgMoveTo(vg, 0, MX_DISP_H / 2.f + 0.5f);
        nvgLineTo(vg, scopeW(), MX_DISP_H / 2.f + 0.5f);
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

        const float mid  = MX_DISP_H / 2.f;
        const float amp  = mid - 2.f;
        const float sw   = scopeW();
        const float colW = sw / (float) MX_SCOPE_N;
        const float range = 10.f;

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, MX_DISP_H);
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < MX_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % MX_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / MX_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - std::min(std::max(vmax / range, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / range, -1.f), 1.f) * amp;
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
            float lvl = std::min(peak / range, 1.f);
            float x = sw + 3.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 1.f, 4.f, MX_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (MX_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, MX_DISP_H - 1.f - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, MX_DISP_H, 0, 0,
                                                   colAccent(), colAccent2()));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) MX_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, MX_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, MX_DISP_W, MX_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, MX_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16),
                                           nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[32];
        float gain = module ? module->params[Mixer6::PARAM_MASTER_GAIN].getValue() : 1.f;
        float peak = module ? std::max(module->peakL, module->peakR) : 0.f;
        snprintf(buf, sizeof(buf), "PK %4.1fV", peak);
        drawText(vg, 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "GAIN %.2f", gain);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ============================================================
// Module widget
// ============================================================
struct Mixer6Widget : ModuleWidget {
    Mixer6Widget(Mixer6* module) {
        setModule(module);
        box.size = Vec(MX_PANEL_W, MX_PANEL_H);

        addChild(new MxPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new MxDisplayWidget(module));

        // 6 stereo channel columns
        for (int ch = 0; ch < 6; ++ch) {
            float cx = MX_COL_X[ch];
            float lx = cx - MX_JACK_OFFSET;
            float rx = cx + MX_JACK_OFFSET;

            // Stereo input pair
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(lx, MX_AUDIO_Y), module,
                (Mixer6::InputId)(Mixer6::INPUT_CH1_L + ch * 2)));
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(rx, MX_AUDIO_Y), module,
                (Mixer6::InputId)(Mixer6::INPUT_CH1_L + ch * 2 + 1)));

            // Level knob (controls both L and R of this channel)
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(cx, MX_LEVEL_Y), module,
                (Mixer6::ParamId)(Mixer6::PARAM_CH1_LEVEL + ch)));

            // CV input + attenuverter
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(cx, MX_CV_Y), module,
                (Mixer6::InputId)(Mixer6::INPUT_CH1_CV + ch)));
            addParam(createParamCentered<Trimpot>(
                Vec(cx, MX_ATTV_Y), module,
                (Mixer6::ParamId)(Mixer6::PARAM_CH1_ATTV + ch)));
        }

        // Master gain
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(MX_PANEL_W / 2.f, MX_GAIN_Y), module, Mixer6::PARAM_MASTER_GAIN));

        // Stereo outputs
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(MX_PANEL_W / 2.f - MX_MASTER_OUT_OFFSET, MX_OUTPUT_Y),
            module, Mixer6::OUTPUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(MX_PANEL_W / 2.f + MX_MASTER_OUT_OFFSET, MX_OUTPUT_Y),
            module, Mixer6::OUTPUT_R));
    }

    void appendContextMenu(Menu* menu) override {
        Mixer6* module = dynamic_cast<Mixer6*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem {
            Mixer6* module;
            Mixer6::CVRange range;
            void onAction(const ActionEvent& e) override {
                module->cvRangeMode = range;
            }
            void step() override {
                rightText = (module->cvRangeMode == range) ? "✔" : "";
                MenuItem::step();
            }
        };

        CVRangeItem* unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar, default)");
        unipolarItem->module = module;
        unipolarItem->range = Mixer6::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);

        CVRangeItem* bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = Mixer6::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);
    }
};

Model* modelMixer6 = createModel<Mixer6, Mixer6Widget>("Mixer6");