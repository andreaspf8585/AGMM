#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   ST_N  = 8;     // channels / register stages
static const int   ST_HN = 24;    // history columns

static const float ST_PANEL_W = RACK_GRID_WIDTH * 24;   // 360
static const float ST_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Display
static const float ST_DISP_W = 228.f;
static const float ST_DISP_H = 140.f;
static const float ST_DISP_X = (ST_PANEL_W - ST_DISP_W) / 2.f;   // 66
static const float ST_DISP_Y = 44.f;

// Plot geometry inside the display (display-local coordinates)
static const float ST_PX0     = 20.f;     // left edge of bars / heat map
static const float ST_PITCH   = 25.f;     // column pitch (8 x 25 = 200)
static const float ST_BAR_CY  = 41.f;     // zero line of the bars
static const float ST_BAR_H   = 24.f;     // half height (10 V)
static const float ST_BAR_W   = 14.f;
static const float ST_LBL_Y   = 72.f;     // stage numbers + chain arrows
static const float ST_LOOP_Y  = 81.f;     // loop bus
static const float ST_HEAT_Y  = 85.f;     // heat map top
static const float ST_ROW_H   = 5.75f;
static const float ST_CAP_Y   = 136.5f;   // bottom caption

// Side knobs: SLEW, PROB (left) / LENGTH, LEVEL (right)
static const float ST_KNOB_X[4] = {28.f, 28.f, 332.f, 332.f};
static const float ST_KNOB_Y[4] = {84.f, 150.f, 84.f, 150.f};

// Control row
static const float ST_CTRL_SEC_Y = 204.f;
static const float ST_CTRL_Y     = 240.f;
static const float ST_CTRL_X[5]  = {60.f, 120.f, 180.f, 240.f, 300.f};

// Channel rows
static const float ST_CH_SEC_Y = 268.f;
static const float ST_IN_Y     = 300.f;
static const float ST_OUT_Y    = 342.f;
static const float ST_CH_X0    = 47.f;
static const float ST_CH_DX    = 38.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Stasis : Module {
    enum ParamId {
        SLEW_PARAM,
        PROB_PARAM,
        LENGTH_PARAM,
        LEVEL_PARAM,
        LOOP_PARAM,
        STEP_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        CLOCK_INPUT,
        RESET_INPUT,
        ENUMS(CH_INPUT, 8),
        INPUTS_LEN
    };
    enum OutputId {
        ENUMS(CH_OUTPUT, 8),
        POLY_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LOOP_LIGHT,
        STEP_LIGHT,
        LIGHTS_LEN
    };

    // Register contents (what the S&H stages hold) and smoothed outputs
    float held[ST_N]  = {};
    float outv[ST_N]  = {};

    // Display data
    float flash[ST_N] = {};       // sampled this clock
    float skip[ST_N]  = {};       // skipped by probability
    bool  linked[ST_N] = {};      // input normalled (not patched)
    float clockFlash = 0.f;
    float hist[ST_N][ST_HN] = {};
    bool  histUpd[ST_N][ST_HN] = {};
    int   histHead = 0;
    int   histCount = 0;
    uint32_t stepCount = 0;
    int   vLen = 8;
    float vProb = 1.f;
    float vTau = 0.f;

    bool loopOn = false;

    dsp::SchmittTrigger clockTrig, stepTrig, resetTrig, loopTrig;

    Stasis() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(SLEW_PARAM, 0.f, 1.f, 0.f, "Slew", "%", 0.f, 100.f);
        configParam(PROB_PARAM, 0.f, 1.f, 1.f, "Sample probability", "%", 0.f, 100.f);
        configParam(LENGTH_PARAM, 1.f, 8.f, 8.f, "Loop length", " steps");
        paramQuantities[LENGTH_PARAM]->snapEnabled = true;
        configParam(LEVEL_PARAM, -1.f, 1.f, 1.f, "Output level", "%", 0.f, 100.f);
        configButton(LOOP_PARAM, "Loop (last active stage feeds stage 1)");
        configButton(STEP_PARAM, "Manual clock step");

        configInput(CLOCK_INPUT, "Clock");
        configInput(RESET_INPUT, "Reset");
        for (int i = 0; i < ST_N; ++i) {
            if (i == 0)
                configInput(CH_INPUT + i, "Channel 1 input (normalled to noise / loop)");
            else
                configInput(CH_INPUT + i, string::f("Channel %d input (normalled to channel %d)", i + 1, i));
            configOutput(CH_OUTPUT + i, string::f("Channel %d", i + 1));
        }
        configOutput(POLY_OUTPUT, "Polyphonic (8 channels)");

        doReset();
    }

    void doReset() {
        std::memset(held, 0, sizeof(held));
        std::memset(flash, 0, sizeof(flash));
        std::memset(skip, 0, sizeof(skip));
        std::memset(hist, 0, sizeof(hist));
        std::memset(histUpd, 0, sizeof(histUpd));
        histHead = 0;
        histCount = 0;
        stepCount = 0;
    }

    void onReset() override {
        doReset();
        loopOn = false;
    }

    void doClock(int len, float prob) {
        float old[ST_N];
        for (int i = 0; i < ST_N; ++i) old[i] = held[i];

        for (int i = 0; i < ST_N; ++i) {
            bool conn = inputs[CH_INPUT + i].isConnected();
            float src;
            if (conn) {
                src = inputs[CH_INPUT + i].getVoltage();
            }
            else if (i == 0) {
                // stage 1: loop tap, or internal white noise
                src = loopOn ? old[len - 1] : (random::uniform() * 10.f - 5.f);
            }
            else {
                // normalled chain: take the previous stage's held value
                src = old[i - 1];
            }
            linked[i] = !conn;

            bool take = (prob >= 0.999f) || (random::uniform() < prob);
            if (take) {
                held[i] = src;
                flash[i] = 1.f;
            } else {
                skip[i] = 1.f;
            }
            hist[i][histHead] = held[i];
            histUpd[i][histHead] = take;
        }
        histHead = (histHead + 1) % ST_HN;
        if (histCount < ST_HN) histCount++;
        stepCount++;
        clockFlash = 1.f;
    }

    void process(const ProcessArgs& args) override {
        const float dt = args.sampleTime;

        float slewK = params[SLEW_PARAM].getValue();
        float prob  = params[PROB_PARAM].getValue();
        int   len   = rack::math::clamp((int)std::round(params[LENGTH_PARAM].getValue()), 1, ST_N);
        float level = params[LEVEL_PARAM].getValue();

        if (loopTrig.process(params[LOOP_PARAM].getValue(), 0.1f, 2.f))
            loopOn = !loopOn;

        if (resetTrig.process(inputs[RESET_INPUT].getVoltage(), 0.1f, 2.f))
            doReset();

        bool clk = clockTrig.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f);
        bool stp = stepTrig.process(params[STEP_PARAM].getValue(), 0.1f, 2.f);
        if (clk || stp) doClock(len, prob);

        // Slew: 1 ms .. 2 s time constant
        float tau = (slewK < 0.002f) ? 0.f : 0.001f * std::pow(2000.f, slewK);
        float a = (tau > 0.f) ? (1.f - std::exp(-dt / tau)) : 1.f;

        outputs[POLY_OUTPUT].setChannels(ST_N);
        for (int i = 0; i < ST_N; ++i) {
            outv[i] += (held[i] - outv[i]) * a;
            float v = outv[i] * level;
            outputs[CH_OUTPUT + i].setVoltage(v);
            outputs[POLY_OUTPUT].setVoltage(v, i);

            flash[i] = std::max(0.f, flash[i] - dt * 6.f);
            skip[i]  = std::max(0.f, skip[i]  - dt * 4.f);
        }
        clockFlash = std::max(0.f, clockFlash - dt * 8.f);

        lights[LOOP_LIGHT].setBrightness(loopOn ? 1.f : 0.f);
        lights[STEP_LIGHT].setBrightness(clockFlash);

        vLen = len;
        vProb = prob;
        vTau = tau;
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "loop", json_boolean(loopOn));
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* j = json_object_get(root, "loop");
        if (j) loopOn = json_is_true(j);
    }
};

// ------------------------------------------------------------
// Theme + drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x25, 0x28, 0x30); }
NVGcolor colBgBottom() { return nvgRGB(0x13, 0x15, 0x19); }
NVGcolor colText()     { return nvgRGB(0xd6, 0xdc, 0xe6); }
NVGcolor colDim()      { return nvgRGB(0x8a, 0x95, 0xa5); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colBright()   { return nvgRGB(0x7a, 0xec, 0xff); }
NVGcolor colWarm()     { return nvgRGB(0xff, 0x9c, 0x6a); }
//NVGcolor colPurple()   { return nvgRGB(0xb0, 0x5a, 0xff); }

NVGcolor withA(NVGcolor c, float a) { return nvgRGBAf(c.r, c.g, c.b, a); }

void setFont(NVGcontext* vg) {
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font) nvgFontFaceId(vg, font->handle);
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

// Custom light for the button LEDs
template <typename TBase = GrayModuleLightWidget>
struct TSTCyanLight : TBase {
    TSTCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using STCyanLight = TSTCyanLight<>;

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

// Tick arc around a knob (n ticks across the 270 degree sweep)
void drawKnobTicks(NVGcontext* vg, float cx, float cy, float r0, float r1, int n = 11) {
    nvgLineCap(vg, NVG_ROUND);
    for (int i = 0; i < n; ++i) {
        float a = (-0.75f + 1.5f * (float)i / (float)(n - 1)) * (float)M_PI;
        bool major = (i == 0 || i == n - 1 || (n % 2 == 1 && i == (n - 1) / 2));
        float ra = major ? r0 - 1.5f : r0;
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + std::sin(a) * ra, cy - std::cos(a) * ra);
        nvgLineTo(vg, cx + std::sin(a) * r1, cy - std::cos(a) * r1);
        nvgStrokeColor(vg, major ? nvgRGBA(0x3d, 0xe0, 0xff, 150) : nvgRGBA(255, 255, 255, 60));
        nvgStrokeWidth(vg, major ? 1.2f : 0.9f);
        nvgStroke(vg);
    }
}

float colX(int i) { return ST_PX0 + ST_PITCH * ((float)i + 0.5f); }

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct STPanelBackground : TransparentWidget {
    STPanelBackground() { box.size = Vec(ST_PANEL_W, ST_PANEL_H); }

    void label(NVGcontext* vg, float x, float y, const char* txt) {
        drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.0f);
    }

    void plate(NVGcontext* vg, float x, float y, float w, float h) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 1.f, y - 1.f, w + 2.f, h + 3.f, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 6.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                           nvgRGBA(0, 0, 0, 105), nvgRGBA(0, 0, 0, 60)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void ring(NVGcontext* vg, float cx, float cy, float r) {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, r * 0.3f, r,
                                           nvgRGBA(0, 0, 0, 40), nvgRGBA(0, 0, 0, 110)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 12.f, x2 = ST_PANEL_W - 12.f, cx = ST_PANEL_W / 2.f;
        float w = 14.f + (float) strlen(txt) * 7.f;
        drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - w / 2.f, y);
        nvgStrokePaint(vg, nvgLinearGradient(vg, x1, y, cx - w / 2.f, y,
                                             nvgRGBA(0x3d, 0xe0, 0xff, 0), nvgRGBA(0x3d, 0xe0, 0xff, 100)));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + w / 2.f, y);
        nvgLineTo(vg, x2, y);
        nvgStrokePaint(vg, nvgLinearGradient(vg, cx + w / 2.f, y, x2, y,
                                             nvgRGBA(0x3d, 0xe0, 0xff, 100), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // kind: 0 = input (orange), 1 = output (cyan), 2 = control (purple)
    void cell(NVGcontext* vg, float cx, float cy, float w, float top, float h, float labelDy,
              const char* txt, int kind) {
        float x = cx - w / 2.f, y = cy + top;
        plate(vg, x, y, w, h);

        NVGcolor c = (kind == 0) ? nvgRGBA(0xff, 0x9c, 0x6a, 170)
                   : (kind == 1) ? nvgRGBA(0x3d, 0xe0, 0xff, 170)
                                 : nvgRGBA(0xb0, 0x5a, 0xff, 170);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x + 6.f, y + 1.5f, w - 12.f, 1.8f, 0.9f);
        nvgFillColor(vg, c);
        nvgFill(vg);

        label(vg, cx, cy + labelDy, txt);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, ST_PANEL_W, ST_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, ST_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) ST_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, ST_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Corner vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, ST_PANEL_W, ST_PANEL_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, ST_PANEL_W / 2.f, ST_PANEL_H * 0.4f,
                                           ST_PANEL_H * 0.35f, ST_PANEL_H * 0.85f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
        nvgFill(vg);

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, ST_PANEL_W - 1.f, ST_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Top accent line
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, ST_PANEL_W, 2.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, ST_PANEL_W, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // Logo + title
        drawLogo(vg, ST_PANEL_W / 2.f, 15.f);
        drawText(vg, ST_PANEL_W / 2.f, 32.f, "STASIS", 12.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // ---- Display bezel (metallic, recessed) ----
        float bx = ST_DISP_X - 5.f, by = ST_DISP_Y - 5.f;
        float bw = ST_DISP_W + 10.f, bh = ST_DISP_H + 10.f;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 4.f, by - 3.f, bw + 8.f, bh + 9.f, 10.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x4a, 0x50, 0x5a), nvgRGB(0x14, 0x16, 0x1a)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 2.f, by - 2.f, bw + 4.f, bh + 4.f, 7.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 36));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- Side knobs ----
        const char* knobL[4] = {"SLEW", "PROB", "LENGTH", "LEVEL"};
        for (int k = 0; k < 4; ++k) {
            ring(vg, ST_KNOB_X[k], ST_KNOB_Y[k], 17.f);
            drawKnobTicks(vg, ST_KNOB_X[k], ST_KNOB_Y[k], 21.f, 24.f, (k == 2) ? 8 : 11);
            label(vg, ST_KNOB_X[k], ST_KNOB_Y[k] + 29.f, knobL[k]);
        }

        // ---- Control row ----
        section(vg, ST_CTRL_SEC_Y, "CONTROL");
        const char* ctl[5] = {"CLOCK", "RESET", "LOOP", "STEP", "POLY"};
        const int   ctlKind[5] = {0, 0, 2, 2, 1};
        for (int i = 0; i < 5; ++i)
            cell(vg, ST_CTRL_X[i], ST_CTRL_Y, 30.f, -27.f, 44.f, -19.f, ctl[i], ctlKind[i]);

        // ---- Channels ----
        section(vg, ST_CH_SEC_Y, "CHANNELS");
        for (int i = 0; i < ST_N; ++i) {
            char buf[8];
            float cx = ST_CH_X0 + ST_CH_DX * (float)i;
            snprintf(buf, sizeof(buf), "IN %d", i + 1);
            cell(vg, cx, ST_IN_Y, 33.f, -21.f, 38.f, -14.f, buf, 0);
            snprintf(buf, sizeof(buf), "OUT %d", i + 1);
            cell(vg, cx, ST_OUT_Y, 33.f, -21.f, 38.f, -14.f, buf, 1);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Live value arcs around the knobs
// ------------------------------------------------------------
struct STKnobArcs : TransparentWidget {
    Stasis* module = nullptr;

    STKnobArcs(Stasis* m) : module(m) { box.size = Vec(ST_PANEL_W, ST_PANEL_H); }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float R = 17.2f;
        const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
        const float a1 =  0.75f * (float)M_PI - (float)M_PI / 2.f;

        for (int k = 0; k < 4; ++k) {
            float cx = ST_KNOB_X[k], cy = ST_KNOB_Y[k];

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, a0, a1, NVG_CW);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 2.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStroke(vg);

            if (!module) continue;
            float v = module->paramQuantities[k]->getScaledValue();   // 0..1
            bool bipolar = (k == Stasis::LEVEL_PARAM);
            float from = bipolar ? 0.5f : 0.f;
            float lo = std::min(from, v), hi = std::max(from, v);
            if (hi - lo < 0.004f) continue;

            float s = a0 + (a1 - a0) * lo;
            float e = a0 + (a1 - a0) * hi;
            NVGcolor col = (bipolar && v < 0.5f) ? colWarm() : colAccent();

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, s, e, NVG_CW);
            nvgStrokeColor(vg, withA(col, 0.18f));
            nvgStrokeWidth(vg, 5.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, s, e, NVG_CW);
            nvgStrokeColor(vg, withA(col, 0.95f));
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
        }
    }
};

// ------------------------------------------------------------
// Register display: bars + chain + loop bus + history heat map
// ------------------------------------------------------------
struct STDisplayWidget : TransparentWidget {
    Stasis* module = nullptr;

    STDisplayWidget(Stasis* m) : module(m) {
        box.pos = Vec(ST_DISP_X, ST_DISP_Y);
        box.size = Vec(ST_DISP_W, ST_DISP_H);
    }

    // ---- static background ----
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float plotW = ST_PITCH * ST_N;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0, 0, ST_DISP_W, ST_DISP_H, 4.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, ST_DISP_H,
                                           nvgRGB(0x0a, 0x16, 0x1f), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0.5f, 0.5f, ST_DISP_W - 1.f, ST_DISP_H - 1.f, 4.f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Bar slots
        for (int i = 0; i < ST_N; ++i) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, colX(i) - ST_BAR_W / 2.f - 1.f, ST_BAR_CY - ST_BAR_H,
                           ST_BAR_W + 2.f, 2.f * ST_BAR_H, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 6));
            nvgFill(vg);
        }

        // Voltage grid: +-10 V, +-5 V, 0 V
        const float gy[5] = {-ST_BAR_H, -ST_BAR_H / 2.f, 0.f, ST_BAR_H / 2.f, ST_BAR_H};
        for (int i = 0; i < 5; ++i) {
            float y = ST_BAR_CY + gy[i];
            nvgBeginPath(vg);
            nvgMoveTo(vg, ST_PX0 - 2.f, y + 0.5f);
            nvgLineTo(vg, ST_PX0 + plotW + 2.f, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, i == 2 ? 50 : 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }
        NVGcolor gl = nvgRGBA(0x8a, 0x95, 0xa5, 170);
        drawText(vg, ST_PX0 - 4.f, ST_BAR_CY - ST_BAR_H, "+10", 6.f, gl, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, ST_PX0 - 4.f, ST_BAR_CY,            "0",   6.f, gl, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, ST_PX0 - 4.f, ST_BAR_CY + ST_BAR_H, "-10", 6.f, gl, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);

        // Heat map frame + row labels
        nvgBeginPath(vg);
        nvgRoundedRect(vg, ST_PX0 - 1.f, ST_HEAT_Y - 1.f, plotW + 2.f, ST_ROW_H * ST_N + 2.f, 2.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 5));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 26));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        for (int r = 0; r < ST_N; ++r) {
            char b[4];
            snprintf(b, sizeof(b), "%d", r + 1);
            drawText(vg, ST_PX0 - 4.f, ST_HEAT_Y + ((float)r + 0.5f) * ST_ROW_H, b, 5.5f, gl,
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.f);
        }

        Widget::draw(args);
    }

    // ---- live content ----
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float plotW = ST_PITCH * ST_N;

        float held[ST_N], outv[ST_N], flash[ST_N], skip[ST_N];
        bool  linked[ST_N];
        float hist[ST_N][ST_HN];
        bool  upd[ST_N][ST_HN];
        bool  loopOn;
        int   len, head, count;
        float prob, tau, clockFlash;
        uint32_t steps;

        if (module) {
            for (int i = 0; i < ST_N; ++i) {
                held[i] = module->held[i];
                outv[i] = module->outv[i];
                flash[i] = module->flash[i];
                skip[i] = module->skip[i];
                linked[i] = module->linked[i];
                for (int c = 0; c < ST_HN; ++c) {
                    hist[i][c] = module->hist[i][c];
                    upd[i][c] = module->histUpd[i][c];
                }
            }
            loopOn = module->loopOn;
            len = module->vLen;
            prob = module->vProb;
            tau = module->vTau;
            clockFlash = module->clockFlash;
            head = module->histHead;
            count = module->histCount;
            steps = module->stepCount;
        } else {
            // demo values for the module browser
            for (int i = 0; i < ST_N; ++i) {
                held[i] = 6.f * std::sin(0.9f * (float)i + 0.6f);
                outv[i] = held[i] * 0.9f;
                flash[i] = (i == 2) ? 0.8f : 0.f;
                skip[i] = (i == 5) ? 0.7f : 0.f;
                linked[i] = (i != 4);
                for (int c = 0; c < ST_HN; ++c) {
                    hist[i][c] = 7.f * std::sin(0.5f * (float)(c - i) + 0.4f * (float)i);
                    upd[i][c] = ((c + i) % 3) != 0;
                }
            }
            loopOn = true;
            len = 6;
            prob = 0.8f;
            tau = 0.12f;
            clockFlash = 0.6f;
            head = 0;
            count = ST_HN;
            steps = 128;
        }
        len = rack::math::clamp(len, 1, ST_N);

        nvgSave(vg);
        nvgScissor(vg, 0, 0, ST_DISP_W, ST_DISP_H);

        // ================= bars =================
        for (int i = 0; i < ST_N; ++i) {
            float cx = colX(i);
            float t = rack::math::clamp(held[i] / 10.f, -1.f, 1.f);
            float h = t * ST_BAR_H;
            bool pos = (t >= 0.f);
            NVGcolor col = pos ? colAccent() : colWarm();
            NVGcolor hi  = pos ? colBright() : nvgRGB(0xff, 0xc8, 0xa8);
            float dim = (loopOn && i >= len) ? 0.45f : 1.f;

            float yTip = ST_BAR_CY - h;
            float bh = std::max(std::fabs(h), 1.f);
            float y0 = pos ? (ST_BAR_CY - bh) : ST_BAR_CY;
            if (pos) yTip = y0;
            else     yTip = y0 + bh;

            // glow
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - ST_BAR_W / 2.f - 2.f, y0 - 2.f, ST_BAR_W + 4.f, bh + 4.f, 3.f);
            nvgFillColor(vg, withA(col, (0.10f + 0.25f * flash[i]) * dim));
            nvgFill(vg);

            // body
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - ST_BAR_W / 2.f, y0, ST_BAR_W, bh, 2.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, ST_BAR_CY, 0, yTip,
                                               withA(col, 0.22f * dim), withA(col, 0.95f * dim)));
            nvgFill(vg);

            // flash overlay
            if (flash[i] > 0.01f) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, cx - ST_BAR_W / 2.f, y0, ST_BAR_W, bh, 2.f);
                nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, flash[i] * 0.55f));
                nvgFill(vg);
            }

            // tip cap
            nvgBeginPath(vg);
            nvgRect(vg, cx - ST_BAR_W / 2.f - 1.f, yTip - 0.75f, ST_BAR_W + 2.f, 1.5f);
            nvgFillColor(vg, withA(hi, 0.95f * dim));
            nvgFill(vg);

            // skipped-by-probability outline
            if (skip[i] > 0.01f) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, cx - ST_BAR_W / 2.f - 1.5f, ST_BAR_CY - ST_BAR_H - 0.5f,
                               ST_BAR_W + 3.f, 2.f * ST_BAR_H + 1.f, 2.f);
                nvgStrokeColor(vg, withA(colWarm(), skip[i] * 0.85f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // slewed-output marker (small pointer on the right of the bar)
            float yo = ST_BAR_CY - rack::math::clamp(outv[i] / 10.f, -1.f, 1.f) * ST_BAR_H;
            float mx = cx + ST_BAR_W / 2.f + 1.5f;
            nvgBeginPath(vg);
            nvgMoveTo(vg, mx, yo);
            nvgLineTo(vg, mx + 3.5f, yo - 2.2f);
            nvgLineTo(vg, mx + 3.5f, yo + 2.2f);
            nvgClosePath(vg);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(200 * dim)));
            nvgFill(vg);
        }

        // ================= stage numbers, chain arrows, loop bus =================
        for (int i = 0; i < ST_N; ++i) {
            char b[4];
            snprintf(b, sizeof(b), "%d", i + 1);
            bool special = loopOn && (i == 0 || i == len - 1);
            float cx = colX(i);

            if (special) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, cx - 5.5f, ST_LBL_Y - 4.5f, 11.f, 9.f, 2.f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 150));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            drawText(vg, cx, ST_LBL_Y, b, 7.f,
                     special ? colBright() : nvgRGBA(0x8a, 0x95, 0xa5, 220),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

            // link between stage i and i+1
            if (i < ST_N - 1) {
                float ax = ST_PX0 + ST_PITCH * (float)(i + 1);
                if (linked[i + 1]) {
                    float a = 0.50f + 0.50f * flash[i + 1];
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, ax - 1.8f, ST_LBL_Y - 2.6f);
                    nvgLineTo(vg, ax + 1.4f, ST_LBL_Y);
                    nvgLineTo(vg, ax - 1.8f, ST_LBL_Y + 2.6f);
                    nvgLineCap(vg, NVG_ROUND);
                    nvgLineJoin(vg, NVG_ROUND);
                    nvgStrokeColor(vg, withA(colAccent(), a));
                    nvgStrokeWidth(vg, 1.2f);
                    nvgStroke(vg);
                } else {
                    // chain broken by a patched input
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, ax, ST_LBL_Y - 2.6f);
                    nvgLineTo(vg, ax, ST_LBL_Y + 2.6f);
                    nvgStrokeColor(vg, nvgRGBA(0xff, 0x9c, 0x6a, 150));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
            }
        }

        if (loopOn) {
            float xt = colX(len - 1), xh = colX(0);
            float yTop = ST_LBL_Y + 4.8f;
            if (len == 1) { xt = xh + 5.f; xh = xh - 5.f; }

            nvgBeginPath(vg);
            nvgMoveTo(vg, xt, yTop);
            nvgLineTo(vg, xt, ST_LOOP_Y);
            nvgLineTo(vg, xh, ST_LOOP_Y);
            nvgLineTo(vg, xh, yTop + 3.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStrokeColor(vg, withA(colAccent(), 0.18f + 0.3f * clockFlash));
            nvgStrokeWidth(vg, 3.5f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, xt, yTop);
            nvgLineTo(vg, xt, ST_LOOP_Y);
            nvgLineTo(vg, xh, ST_LOOP_Y);
            nvgLineTo(vg, xh, yTop + 3.f);
            nvgStrokeColor(vg, withA(colBright(), 0.85f));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // arrow head into stage 1
            nvgBeginPath(vg);
            nvgMoveTo(vg, xh, yTop - 0.3f);
            nvgLineTo(vg, xh - 2.3f, yTop + 3.4f);
            nvgLineTo(vg, xh + 2.3f, yTop + 3.4f);
            nvgClosePath(vg);
            nvgFillColor(vg, withA(colBright(), 0.95f));
            nvgFill(vg);
        }

        // ================= history heat map =================
        const float cw = plotW / (float)ST_HN;
        for (int c = 0; c < ST_HN; ++c) {
            if (c < ST_HN - count) continue;
            int idx = ((head - ST_HN + c) % ST_HN + ST_HN) % ST_HN;
            float x = ST_PX0 + (float)c * cw;
            for (int r = 0; r < ST_N; ++r) {
                float v = hist[r][idx];
                float t = rack::math::clamp(v / 10.f, -1.f, 1.f);
                float m = std::pow(std::fabs(t), 0.8f);
                NVGcolor col = (t >= 0.f) ? colAccent() : colWarm();
                float y = ST_HEAT_Y + (float)r * ST_ROW_H;

                nvgBeginPath(vg);
                nvgRect(vg, x + 0.5f, y + 0.5f, cw - 1.f, ST_ROW_H - 1.f);
                nvgFillColor(vg, withA(col, 0.10f + 0.85f * m));
                nvgFill(vg);

                if (upd[r][idx]) {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 0.5f, y + 0.5f, 1.f, ST_ROW_H - 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 110));
                    nvgFill(vg);
                }
            }
        }
        // newest column highlight
        {
            float x = ST_PX0 + (float)(ST_HN - 1) * cw;
            nvgBeginPath(vg);
            nvgRect(vg, x, ST_HEAT_Y - 0.5f, cw, ST_ROW_H * ST_N + 1.f);
            nvgStrokeColor(vg, withA(colBright(), 0.25f + 0.65f * clockFlash));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // ================= readouts =================
        {
            char buf[48];
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 220);
            NVGcolor cm = nvgRGBA(0xff, 0x9c, 0x6a, 230);
            NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 230);

            // clock lamp
            nvgBeginPath(vg);
            nvgCircle(vg, 8.f, 7.f, 5.f);
            nvgFillColor(vg, withA(colAccent(), 0.20f * clockFlash));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, 8.f, 7.f, 2.2f);
            nvgFillColor(vg, withA(colAccent(), 0.25f + 0.75f * clockFlash));
            nvgFill(vg);

            snprintf(buf, sizeof(buf), "LEN %d  PROB %.0f%%", len, prob * 100.f);
            drawText(vg, 15.f, 7.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

            drawText(vg, ST_DISP_W / 2.f + 14.f, 7.f, loopOn ? "LOOP" : "SHIFT", 7.f,
                     loopOn ? colBright() : cd, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

            if (tau <= 0.f)       snprintf(buf, sizeof(buf), "SLEW OFF");
            else if (tau < 1.f)   snprintf(buf, sizeof(buf), "SLEW %.0fms", tau * 1000.f);
            else                  snprintf(buf, sizeof(buf), "SLEW %.1fs", tau);
            drawText(vg, ST_DISP_W - 5.f, 7.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            drawText(vg, ST_PX0, ST_CAP_Y, "OLD", 6.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "STEP %u", (unsigned) steps);
            drawText(vg, ST_DISP_W / 2.f, ST_CAP_Y, buf, 6.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            drawText(vg, ST_PX0 + plotW, ST_CAP_Y, "NEW", 6.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        }

        // ================= scanlines, vignette, glass =================
        nvgBeginPath(vg);
        for (int y = 0; y < (int) ST_DISP_H; y += 3)
            nvgRect(vg, 0, y + 1, ST_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, ST_DISP_W, ST_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, ST_DISP_W / 2.f, ST_DISP_H / 2.f,
                                           ST_DISP_W * 0.45f, ST_DISP_W * 0.85f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 120)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, ST_DISP_W, ST_DISP_H * 0.30f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, ST_DISP_H * 0.30f,
                                           nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        nvgRestore(vg);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct StasisWidget : ModuleWidget {
    StasisWidget(Stasis* module) {
        setModule(module);
        box.size = Vec(ST_PANEL_W, ST_PANEL_H);

        addChild(new STPanelBackground());
        addChild(new STKnobArcs(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new STDisplayWidget(module));

        // Side knobs: SLEW, PROB, LENGTH, LEVEL
        for (int k = 0; k < 4; ++k) {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(ST_KNOB_X[k], ST_KNOB_Y[k]), module, k));
        }

        // Control row
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(ST_CTRL_X[0], ST_CTRL_Y), module, Stasis::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(ST_CTRL_X[1], ST_CTRL_Y), module, Stasis::RESET_INPUT));

        addParam(createParamCentered<LEDButton>(
            Vec(ST_CTRL_X[2], ST_CTRL_Y), module, Stasis::LOOP_PARAM));
        addChild(createLightCentered<MediumLight<STCyanLight>>(
            Vec(ST_CTRL_X[2], ST_CTRL_Y), module, Stasis::LOOP_LIGHT));

        addParam(createParamCentered<LEDButton>(
            Vec(ST_CTRL_X[3], ST_CTRL_Y), module, Stasis::STEP_PARAM));
        addChild(createLightCentered<MediumLight<STCyanLight>>(
            Vec(ST_CTRL_X[3], ST_CTRL_Y), module, Stasis::STEP_LIGHT));

        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(ST_CTRL_X[4], ST_CTRL_Y), module, Stasis::POLY_OUTPUT));

        // Channel inputs (top row) and outputs (bottom row)
        for (int i = 0; i < ST_N; ++i) {
            float cx = ST_CH_X0 + ST_CH_DX * (float)i;
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(cx, ST_IN_Y + 3.f), module, Stasis::CH_INPUT + i));
            addOutput(createOutputCentered<DarkPJ301MPort>(
                Vec(cx, ST_OUT_Y + 3.f), module, Stasis::CH_OUTPUT + i));
        }
    }
};

Model* modelStasis = createModel<Stasis, StasisWidget>("Stasis");