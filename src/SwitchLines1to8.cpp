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
static const int   SW_N  = 8;     // outputs
static const int   SW_HN = 16;    // history cells

static const float SW_PANEL_W = RACK_GRID_WIDTH * 24;   // 360
static const float SW_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Display
static const float SW_DISP_W = 228.f;
static const float SW_DISP_H = 140.f;
static const float SW_DISP_X = (SW_PANEL_W - SW_DISP_W) / 2.f;   // 66
static const float SW_DISP_Y = 44.f;

// Plot geometry inside the display (display-local coordinates)
static const float SW_ROW0   = 20.f;      // first output row
static const float SW_ROWP   = 12.5f;     // row pitch
static const float SW_BAR_X  = 8.f;       // input voltage bar
static const float SW_BAR_W  = 8.f;
static const float SW_INN_X  = 26.f;      // input node
static const float SW_HUB_X  = 54.f;      // junction
static const float SW_HUB_R  = 7.f;
static const float SW_NODE_X = 162.f;     // output nodes
static const float SW_MET_CX = 188.f;     // output voltage meters
static const float SW_MET_HW = 16.f;      // meter half width (10 V)
static const float SW_LBL_X  = 217.f;     // output numbers
static const float SW_HIST_X = 22.f;
static const float SW_HIST_Y = 114.f;
static const float SW_HIST_P = 11.5f;
static const float SW_HIST_W = 10.f;
static const float SW_HIST_H = 12.f;
static const float SW_CAP_Y  = 134.f;
static const float SW_BAR_HALF = 44.f;

static inline float swRowY(int i) { return SW_ROW0 + SW_ROWP * (float)i; }
static inline float swHubY()      { return SW_ROW0 + SW_ROWP * 3.5f; }

// Side knobs: LENGTH, START (left) / DIR, FADE (right)
static const float SW_KNOB_X[4] = {28.f, 28.f, 332.f, 332.f};
static const float SW_KNOB_Y[4] = {84.f, 150.f, 84.f, 150.f};

// Control row
static const float SW_CTRL_SEC_Y = 204.f;
static const float SW_CTRL_Y     = 240.f;
static const float SW_CTRL_X[5]  = {60.f, 120.f, 180.f, 240.f, 300.f};

// Channel rows
static const float SW_CH_SEC_Y = 268.f;
static const float SW_OUT_Y    = 300.f;
static const float SW_GATE_Y   = 342.f;
static const float SW_CH_X0    = 47.f;
static const float SW_CH_DX    = 38.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct SwitchLines1to8 : Module {
    enum ParamId {
        LENGTH_PARAM,
        START_PARAM,
        DIR_PARAM,
        FADE_PARAM,
        STEP_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        CLOCK_INPUT,
        RESET_INPUT,
        SIGNAL_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        EOC_OUTPUT,
        ENUMS(CH_OUTPUT, 8),
        ENUMS(GATE_OUTPUT, 8),
        OUTPUTS_LEN
    };
    enum LightId {
        STEP_LIGHT,
        LIGHTS_LEN
    };

    // Sequencer state
    int   pos = 0;            // position inside the active range
    int   dirSign = 1;        // pendulum direction
    int   curCh = 0;          // selected output
    int   prevCh = 0;         // output we are fading away from
    float gain[SW_N] = {1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    // Display data
    float inV = 0.f;
    bool  inLinked = false;
    float outV[SW_N] = {};
    bool  linked[SW_N] = {};  // output patched
    float pulse = 2.f;        // travelling pulse 0..1 (>1 = idle)
    float clockFlash = 0.f;
    float eocFlash = 0.f;
    int   hist[SW_HN] = {};
    int   histHead = 0;
    int   histCount = 0;
    uint32_t stepCount = 0;
    int   vLen = 8, vStart = 0, vDir = 0;
    float vFadeT = 0.f;

    dsp::SchmittTrigger clockTrig, stepTrig, resetTrig;
    dsp::PulseGenerator eocPulse;

    SwitchLines1to8() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(LENGTH_PARAM, 1.f, 8.f, 8.f, "Sequence length", " steps");
        paramQuantities[LENGTH_PARAM]->snapEnabled = true;
        configParam(START_PARAM, 1.f, 8.f, 1.f, "First output");
        paramQuantities[START_PARAM]->snapEnabled = true;
        configSwitch(DIR_PARAM, 0.f, 3.f, 0.f, "Direction",
                     {"Forward", "Reverse", "Pendulum", "Random"});
        configParam(FADE_PARAM, 0.f, 1.f, 0.f, "Crossfade");
        configButton(STEP_PARAM, "Manual clock step");

        configInput(CLOCK_INPUT, "Clock");
        configInput(RESET_INPUT, "Reset");
        configInput(SIGNAL_INPUT, "Signal");
        for (int i = 0; i < SW_N; ++i) {
            configOutput(CH_OUTPUT + i, string::f("Output %d", i + 1));
            configOutput(GATE_OUTPUT + i, string::f("Gate %d (output %d selected)", i + 1, i + 1));
        }
        configOutput(EOC_OUTPUT, "End of cycle trigger");
    }

    void onReset() override {
        pos = 0;
        dirSign = 1;
        curCh = prevCh = 0;
        for (int i = 0; i < SW_N; ++i) gain[i] = (i == 0) ? 1.f : 0.f;
        pulse = 2.f;
        std::memset(hist, 0, sizeof(hist));
        histHead = histCount = 0;
        stepCount = 0;
    }

    void doReset(int S, int L, int dir) {
        pos = (dir == 1) ? L - 1 : 0;
        dirSign = 1;
        prevCh = curCh = (S + pos) % SW_N;
        for (int i = 0; i < SW_N; ++i) gain[i] = (i == curCh) ? 1.f : 0.f;
        pulse = 2.f;
        stepCount = 0;
    }

    void doStep(int S, int L, int dir) {
        bool eoc = false;
        switch (dir) {
            case 0:
                pos++;
                if (pos >= L) { pos = 0; eoc = true; }
                break;
            case 1:
                pos--;
                if (pos < 0) { pos = L - 1; eoc = true; }
                break;
            case 2:
                if (L <= 1) { pos = 0; eoc = true; }
                else {
                    pos += dirSign;
                    if (pos >= L)     { pos = L - 2; dirSign = -1; }
                    else if (pos < 0) { pos = 1;     dirSign = 1; eoc = true; }
                }
                break;
            default:
                if (L > 1) {
                    int n = (int)(random::uniform() * (float)(L - 1));
                    if (n >= pos) n++;
                    pos = rack::math::clamp(n, 0, L - 1);
                } else pos = 0;
                if (pos == 0) eoc = true;
                break;
        }

        prevCh = curCh;
        curCh = (S + pos) % SW_N;
        pulse = 0.f;
        clockFlash = 1.f;
        stepCount++;

        hist[histHead] = curCh;
        histHead = (histHead + 1) % SW_HN;
        if (histCount < SW_HN) histCount++;

        if (eoc) {
            eocPulse.trigger(0.001f);
            eocFlash = 1.f;
        }
    }

    void process(const ProcessArgs& args) override {
        const float dt = args.sampleTime;

        int L   = rack::math::clamp((int)std::round(params[LENGTH_PARAM].getValue()), 1, SW_N);
        int S   = rack::math::clamp((int)std::round(params[START_PARAM].getValue()) - 1, 0, SW_N - 1);
        int dir = rack::math::clamp((int)std::round(params[DIR_PARAM].getValue()), 0, 3);
        float fade = params[FADE_PARAM].getValue();
        float fadeT = (fade < 0.01f) ? 0.f : (0.002f + fade * fade * 0.4f);

        if (pos >= L) pos = L - 1;

        if (resetTrig.process(inputs[RESET_INPUT].getVoltage(), 0.1f, 2.f))
            doReset(S, L, dir);

        bool clk = clockTrig.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f);
        bool stp = stepTrig.process(params[STEP_PARAM].getValue(), 0.1f, 2.f);
        if (clk || stp) doStep(S, L, dir);

        curCh = rack::math::clamp(curCh, 0, SW_N - 1);

        inLinked = inputs[SIGNAL_INPUT].isConnected();
        inV = inLinked ? inputs[SIGNAL_INPUT].getVoltage() : 0.f;

        // Per-output gain: the selected output fades in, the others fade out
        for (int i = 0; i < SW_N; ++i) {
            float target = (i == curCh) ? 1.f : 0.f;
            if (fadeT <= 0.f) {
                gain[i] = target;
            } else {
                float step = dt / fadeT;
                if (gain[i] < target) gain[i] = std::min(target, gain[i] + step);
                else                  gain[i] = std::max(target, gain[i] - step);
            }
            outV[i] = inV * gain[i];
            linked[i] = outputs[CH_OUTPUT + i].isConnected();

            outputs[CH_OUTPUT + i].setVoltage(outV[i]);
            outputs[GATE_OUTPUT + i].setVoltage(i == curCh ? 10.f : 0.f);
        }
        outputs[EOC_OUTPUT].setVoltage(eocPulse.process(dt) ? 10.f : 0.f);

        if (pulse < 2.f) pulse += dt * 5.f;
        clockFlash = std::max(0.f, clockFlash - dt * 8.f);
        eocFlash   = std::max(0.f, eocFlash   - dt * 4.f);
        lights[STEP_LIGHT].setBrightness(clockFlash);

        vLen = L;
        vStart = S;
        vDir = dir;
        vFadeT = fadeT;
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

// Custom light for the button LED
template <typename TBase = GrayModuleLightWidget>
struct TSWCyanLight : TBase {
    TSWCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using SWCyanLight = TSWCyanLight<>;

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

// Cubic Bezier point
void bezier(float t, float x0, float y0, float cx0, float cy0, float cx1, float cy1,
            float x1, float y1, float& x, float& y) {
    float u = 1.f - t;
    float b0 = u * u * u, b1 = 3.f * u * u * t, b2 = 3.f * u * t * t, b3 = t * t * t;
    x = b0 * x0 + b1 * cx0 + b2 * cx1 + b3 * x1;
    y = b0 * y0 + b1 * cy0 + b2 * cy1 + b3 * y1;
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct SWPanelBackground : TransparentWidget {
    SWPanelBackground() { box.size = Vec(SW_PANEL_W, SW_PANEL_H); }

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
        float x1 = 12.f, x2 = SW_PANEL_W - 12.f, cx = SW_PANEL_W / 2.f;
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
        nvgRect(vg, 0, 0, SW_PANEL_W, SW_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SW_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) SW_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, SW_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Corner vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SW_PANEL_W, SW_PANEL_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, SW_PANEL_W / 2.f, SW_PANEL_H * 0.4f,
                                           SW_PANEL_H * 0.35f, SW_PANEL_H * 0.85f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
        nvgFill(vg);

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, SW_PANEL_W - 1.f, SW_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Top accent line
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SW_PANEL_W, 2.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, SW_PANEL_W, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // Logo + title
        drawLogo(vg, SW_PANEL_W / 2.f, 15.f);
        drawText(vg, SW_PANEL_W / 2.f, 32.f, "SWITCH 1 > 8", 12.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // ---- Display bezel (metallic, recessed) ----
        float bx = SW_DISP_X - 5.f, by = SW_DISP_Y - 5.f;
        float bw = SW_DISP_W + 10.f, bh = SW_DISP_H + 10.f;

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
        const char* knobL[4] = {"LENGTH", "START", "DIR", "FADE"};
        const int   knobN[4] = {8, 8, 4, 11};
        for (int k = 0; k < 4; ++k) {
            ring(vg, SW_KNOB_X[k], SW_KNOB_Y[k], 17.f);
            drawKnobTicks(vg, SW_KNOB_X[k], SW_KNOB_Y[k], 21.f, 24.f, knobN[k]);
            label(vg, SW_KNOB_X[k], SW_KNOB_Y[k] + 29.f, knobL[k]);
        }

        // ---- Control row ----
        section(vg, SW_CTRL_SEC_Y, "CONTROL");
        const char* ctl[5] = {"CLOCK", "RESET", "STEP", "IN", "EOC"};
        const int   ctlKind[5] = {0, 0, 2, 0, 1};
        for (int i = 0; i < 5; ++i)
            cell(vg, SW_CTRL_X[i], SW_CTRL_Y, 30.f, -27.f, 44.f, -19.f, ctl[i], ctlKind[i]);

        // ---- Channels ----
        section(vg, SW_CH_SEC_Y, "CHANNELS");
        for (int i = 0; i < SW_N; ++i) {
            char buf[8];
            float cx = SW_CH_X0 + SW_CH_DX * (float)i;
            snprintf(buf, sizeof(buf), "OUT %d", i + 1);
            cell(vg, cx, SW_OUT_Y, 33.f, -21.f, 38.f, -14.f, buf, 1);
            snprintf(buf, sizeof(buf), "GATE %d", i + 1);
            cell(vg, cx, SW_GATE_Y, 33.f, -21.f, 38.f, -14.f, buf, 1);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Live value arcs around the knobs
// ------------------------------------------------------------
struct SWKnobArcs : TransparentWidget {
    SwitchLines1to8* module = nullptr;

    SWKnobArcs(SwitchLines1to8* m) : module(m) { box.size = Vec(SW_PANEL_W, SW_PANEL_H); }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float R = 17.2f;
        const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
        const float a1 =  0.75f * (float)M_PI - (float)M_PI / 2.f;

        for (int k = 0; k < 4; ++k) {
            float cx = SW_KNOB_X[k], cy = SW_KNOB_Y[k];

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, a0, a1, NVG_CW);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 2.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStroke(vg);

            if (!module) continue;
            float v = module->paramQuantities[k]->getScaledValue();   // 0..1
            if (v < 0.004f) continue;

            float e = a0 + (a1 - a0) * v;

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, a0, e, NVG_CW);
            nvgStrokeColor(vg, withA(colAccent(), 0.18f));
            nvgStrokeWidth(vg, 5.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, a0, e, NVG_CW);
            nvgStrokeColor(vg, withA(colAccent(), 0.95f));
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
        }
    }
};

// ------------------------------------------------------------
// Routing display: input -> junction -> 8 outputs, plus history
// ------------------------------------------------------------
struct SWDisplayWidget : TransparentWidget {
    SwitchLines1to8* module = nullptr;

    SWDisplayWidget(SwitchLines1to8* m) : module(m) {
        box.pos = Vec(SW_DISP_X, SW_DISP_Y);
        box.size = Vec(SW_DISP_W, SW_DISP_H);
    }

    // ---- static background ----
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0, 0, SW_DISP_W, SW_DISP_H, 4.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SW_DISP_H,
                                           nvgRGB(0x0a, 0x16, 0x1f), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0.5f, 0.5f, SW_DISP_W - 1.f, SW_DISP_H - 1.f, 4.f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Output meter slots (0 V centre line)
        for (int i = 0; i < SW_N; ++i) {
            float y = swRowY(i);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, SW_MET_CX - SW_MET_HW, y - 2.5f, 2.f * SW_MET_HW, 5.f, 1.5f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 7));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, SW_MET_CX + 0.5f, y - 3.5f);
            nvgLineTo(vg, SW_MET_CX + 0.5f, y + 3.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Input bar slot
        float hy = swHubY();
        nvgBeginPath(vg);
        nvgRoundedRect(vg, SW_BAR_X, hy - SW_BAR_HALF, SW_BAR_W, 2.f * SW_BAR_HALF, 2.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 7));
        nvgFill(vg);
        for (int k = -2; k <= 2; ++k) {
            float y = hy + (float)k * SW_BAR_HALF / 2.f;
            nvgBeginPath(vg);
            nvgMoveTo(vg, SW_BAR_X - 2.f, y + 0.5f);
            nvgLineTo(vg, SW_BAR_X + SW_BAR_W + 2.f, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, k == 0 ? 50 : 18));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }
        drawText(vg, SW_BAR_X + SW_BAR_W / 2.f, SW_HIST_Y + 6.f, "IN", 6.f,
                 nvgRGBA(0x8a, 0x95, 0xa5, 200), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        // History frame
        nvgBeginPath(vg);
        nvgRoundedRect(vg, SW_HIST_X - 2.f, SW_HIST_Y - 2.f,
                       SW_HIST_P * SW_HN + 0.5f, SW_HIST_H + 4.f, 2.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 5));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 26));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        Widget::draw(args);
    }

    // ---- live content ----
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        float gain[SW_N], outV[SW_N];
        bool  linked[SW_N];
        int   hist[SW_HN];
        int   cur, len, start, dir, head, count;
        float inV, pulse, clockFlash, eocFlash, fadeT;
        bool  inLinked;
        uint32_t steps;

        if (module) {
            for (int i = 0; i < SW_N; ++i) {
                gain[i] = module->gain[i];
                outV[i] = module->outV[i];
                linked[i] = module->linked[i];
            }
            for (int c = 0; c < SW_HN; ++c) hist[c] = module->hist[c];
            cur = module->curCh;
            len = module->vLen;       start = module->vStart;
            dir = module->vDir;       head = module->histHead;
            count = module->histCount;
            inV = module->inV;        inLinked = module->inLinked;
            pulse = module->pulse;    clockFlash = module->clockFlash;
            eocFlash = module->eocFlash;
            fadeT = module->vFadeT;
            steps = module->stepCount;
        } else {
            // demo values for the module browser
            inV = 6.f; inLinked = true;
            for (int i = 0; i < SW_N; ++i) {
                gain[i] = (i == 2) ? 0.6f : (i == 1) ? 0.4f : 0.f;
                outV[i] = inV * gain[i];
                linked[i] = (i != 5);
            }
            for (int c = 0; c < SW_HN; ++c) hist[c] = (c * 3) % SW_N;
            cur = 2; len = 6; start = 0; dir = 0;
            head = 0; count = SW_HN;
            pulse = 0.5f; clockFlash = 0.5f; eocFlash = 0.f;
            fadeT = 0.1f; steps = 64;
        }
        cur = rack::math::clamp(cur, 0, SW_N - 1);
        len = rack::math::clamp(len, 1, SW_N);

        const float hx = SW_HUB_X, hy = swHubY();
        NVGcolor sig = (inV >= 0.f) ? colAccent() : colWarm();

        nvgSave(vg);
        nvgScissor(vg, 0, 0, SW_DISP_W, SW_DISP_H);

        // ================= input bar + line to the junction =================
        {
            float t = rack::math::clamp(inV / 10.f, -1.f, 1.f);
            float h = t * SW_BAR_HALF;
            if (std::fabs(h) > 0.3f) {
                float y0 = (h >= 0.f) ? hy - h : hy;
                float hh = std::max(std::fabs(h), 1.f);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, SW_BAR_X - 2.f, y0 - 2.f, SW_BAR_W + 4.f, hh + 4.f, 3.f);
                nvgFillColor(vg, withA(sig, 0.12f + 0.2f * clockFlash));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, SW_BAR_X, y0, SW_BAR_W, hh, 2.f);
                float tipY = (h >= 0.f) ? y0 : y0 + hh;
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, hy, 0, tipY,
                                                   withA(sig, 0.25f), withA(sig, 0.95f)));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRect(vg, SW_BAR_X - 1.f, tipY - 0.75f, SW_BAR_W + 2.f, 1.5f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 220));
                nvgFill(vg);
            }

            // input node
            nvgBeginPath(vg);
            nvgCircle(vg, SW_INN_X, hy, 3.f);
            if (inLinked) {
                nvgFillColor(vg, withA(colBright(), 0.95f));
                nvgFill(vg);
            } else {
                nvgStrokeColor(vg, nvgRGBA(0x8a, 0x95, 0xa5, 170));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // line node -> hub
            nvgLineCap(vg, NVG_ROUND);
            nvgBeginPath(vg);
            nvgMoveTo(vg, SW_INN_X + 3.f, hy);
            nvgLineTo(vg, hx - SW_HUB_R, hy);
            nvgStrokeColor(vg, withA(sig, 0.18f + 0.25f * clockFlash));
            nvgStrokeWidth(vg, 4.5f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, SW_INN_X + 3.f, hy);
            nvgLineTo(vg, hx - SW_HUB_R, hy);
            nvgStrokeColor(vg, withA(colBright(), inLinked ? 0.85f : 0.3f));
            nvgStrokeWidth(vg, 1.5f);
            nvgStroke(vg);
        }

        // ================= output rows + curves =================
        // draw inactive first, active last
        for (int pass = 0; pass < 2; ++pass) {
            for (int i = 0; i < SW_N; ++i) {
                float w = rack::math::clamp(gain[i], 0.f, 1.f);
                bool act = (w > 0.01f);
                if ((pass == 0) == act) continue;

                float y = swRowY(i);
                bool inRange = (((i - start) % SW_N + SW_N) % SW_N) < len;
                float dim = inRange ? 1.f : 0.35f;

                float x0 = hx + SW_HUB_R, x1 = SW_NODE_X - 3.f;
                float cxm = (x0 + x1) * 0.5f;

                // curve
                nvgBeginPath(vg);
                nvgMoveTo(vg, x0, hy);
                nvgBezierTo(vg, cxm, hy, cxm, y, x1, y);
                nvgLineCap(vg, NVG_ROUND);
                if (!act) {
                    nvgStrokeColor(vg, nvgRGBA(0x8a, 0x95, 0xa5, (int)(46 * dim)));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                } else {
                    nvgStrokeColor(vg, withA(sig, 0.16f * w));
                    nvgStrokeWidth(vg, 5.f);
                    nvgStroke(vg);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, x0, hy);
                    nvgBezierTo(vg, cxm, hy, cxm, y, x1, y);
                    nvgStrokeColor(vg, withA(colBright(), 0.35f + 0.60f * w));
                    nvgStrokeWidth(vg, 1.5f);
                    nvgStroke(vg);
                }

                // travelling pulse on the selected line
                if (i == cur && pulse < 1.f) {
                    float px, py;
                    bezier(pulse, x0, hy, cxm, hy, cxm, y, x1, y, px, py);
                    nvgBeginPath(vg);
                    nvgCircle(vg, px, py, 5.f);
                    nvgFillColor(vg, withA(colBright(), 0.22f * (1.f - pulse)));
                    nvgFill(vg);
                    nvgBeginPath(vg);
                    nvgCircle(vg, px, py, 2.3f);
                    nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, 0.95f));
                    nvgFill(vg);
                }

                // node (filled = patched, hollow = unpatched)
                nvgBeginPath(vg);
                nvgCircle(vg, SW_NODE_X, y, act ? 3.f : 2.4f);
                if (linked[i]) {
                    nvgFillColor(vg, act ? withA(colBright(), 0.95f)
                                         : nvgRGBA(0x8a, 0x95, 0xa5, (int)(200 * dim)));
                    nvgFill(vg);
                } else {
                    nvgStrokeColor(vg, nvgRGBA(0x8a, 0x95, 0xa5, (int)(150 * dim)));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                if (act) {
                    nvgBeginPath(vg);
                    nvgCircle(vg, SW_NODE_X, y, 6.f);
                    nvgStrokeColor(vg, withA(colBright(), 0.55f * w));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }

                // output voltage meter
                float t = rack::math::clamp(outV[i] / 10.f, -1.f, 1.f);
                float mw = t * SW_MET_HW;
                NVGcolor mc = (outV[i] >= 0.f) ? colAccent() : colWarm();
                if (std::fabs(mw) > 0.2f) {
                    float mx0 = (mw >= 0.f) ? SW_MET_CX : SW_MET_CX + mw;
                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, mx0, y - 2.f, std::max(std::fabs(mw), 1.f), 4.f, 1.f);
                    nvgFillColor(vg, withA(mc, (0.45f + 0.5f * w) * dim));
                    nvgFill(vg);
                }

                // number
                char b[4];
                snprintf(b, sizeof(b), "%d", i + 1);
                drawText(vg, SW_LBL_X, y, b, 7.f,
                         act ? colBright() : nvgRGBA(0x8a, 0x95, 0xa5, (int)(220 * dim)),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
            }
        }

        // ================= junction =================
        {
            nvgBeginPath(vg);
            nvgCircle(vg, hx, hy, SW_HUB_R + 5.f);
            nvgFillColor(vg, withA(colAccent(), 0.08f + 0.22f * clockFlash));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgCircle(vg, hx, hy, SW_HUB_R);
            nvgFillPaint(vg, nvgLinearGradient(vg, hx, hy - SW_HUB_R, hx, hy + SW_HUB_R,
                                               nvgRGB(0x3a, 0x41, 0x4c), nvgRGB(0x12, 0x15, 0x1a)));
            nvgFill(vg);
            nvgStrokeColor(vg, withA(colBright(), 0.6f + 0.4f * clockFlash));
            nvgStrokeWidth(vg, 1.2f);
            nvgStroke(vg);

            char b[4];
            snprintf(b, sizeof(b), "%d", cur + 1);
            drawText(vg, hx, hy + 0.5f, b, 9.f, colBright(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        }

        // ================= history strip =================
        for (int c = 0; c < SW_HN; ++c) {
            if (c < SW_HN - count) continue;
            int idx = ((head - SW_HN + c) % SW_HN + SW_HN) % SW_HN;
            float x = SW_HIST_X + (float)c * SW_HIST_P;
            float age = (float)c / (float)(SW_HN - 1);        // 1 = newest
            float a = 0.10f + 0.45f * age * age;

            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, SW_HIST_Y, SW_HIST_W, SW_HIST_H, 2.f);
            nvgFillColor(vg, withA(colAccent(), a));
            nvgFill(vg);

            char b[4];
            snprintf(b, sizeof(b), "%d", hist[idx] + 1);
            drawText(vg, x + SW_HIST_W / 2.f, SW_HIST_Y + SW_HIST_H / 2.f + 0.5f, b, 6.5f,
                     nvgRGBA(255, 255, 255, (int)(110 + 120 * age)),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        }
        {
            float x = SW_HIST_X + (float)(SW_HN - 1) * SW_HIST_P;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x - 0.5f, SW_HIST_Y - 0.5f, SW_HIST_W + 1.f, SW_HIST_H + 1.f, 2.5f);
            nvgStrokeColor(vg, withA(colBright(), 0.30f + 0.65f * clockFlash));
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

            snprintf(buf, sizeof(buf), "IN > OUT %d", cur + 1);
            drawText(vg, 15.f, 7.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

            const char* dn[4] = {"FWD", "REV", "PEND", "RAND"};
            drawText(vg, SW_DISP_W / 2.f + 14.f, 7.f, dn[rack::math::clamp(dir, 0, 3)], 7.f,
                     colBright(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

            // end-of-cycle lamp
            nvgBeginPath(vg);
            nvgCircle(vg, SW_DISP_W - 32.f, 7.f, 2.f);
            nvgFillColor(vg, withA(colWarm(), 0.25f + 0.75f * eocFlash));
            nvgFill(vg);

            snprintf(buf, sizeof(buf), "LEN %d", len);
            drawText(vg, SW_DISP_W - 5.f, 7.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            drawText(vg, SW_HIST_X, SW_CAP_Y, "OLD", 6.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            if (fadeT > 0.f)
                snprintf(buf, sizeof(buf), "STEP %u  FADE %.0fms", (unsigned) steps, fadeT * 1000.f);
            else
                snprintf(buf, sizeof(buf), "STEP %u", (unsigned) steps);
            drawText(vg, SW_DISP_W / 2.f - 4.f, SW_CAP_Y, buf, 6.f, c1,
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            drawText(vg, SW_HIST_X + SW_HIST_P * SW_HN - 3.f, SW_CAP_Y, "NEW", 6.f, cd,
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        }

        // ================= scanlines, vignette, glass =================
        nvgBeginPath(vg);
        for (int y = 0; y < (int) SW_DISP_H; y += 3)
            nvgRect(vg, 0, y + 1, SW_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SW_DISP_W, SW_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, SW_DISP_W / 2.f, SW_DISP_H / 2.f,
                                           SW_DISP_W * 0.45f, SW_DISP_W * 0.85f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 120)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SW_DISP_W, SW_DISP_H * 0.30f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SW_DISP_H * 0.30f,
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
struct SwitchLines1to8Widget : ModuleWidget {
    SwitchLines1to8Widget(SwitchLines1to8* module) {
        setModule(module);
        box.size = Vec(SW_PANEL_W, SW_PANEL_H);

        addChild(new SWPanelBackground());
        addChild(new SWKnobArcs(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SWDisplayWidget(module));

        // Side knobs: LENGTH, START, DIR, FADE
        for (int k = 0; k < 4; ++k) {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(SW_KNOB_X[k], SW_KNOB_Y[k]), module, k));
        }

        // Control row
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(SW_CTRL_X[0], SW_CTRL_Y), module, SwitchLines1to8::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(SW_CTRL_X[1], SW_CTRL_Y), module, SwitchLines1to8::RESET_INPUT));

        addParam(createParamCentered<LEDButton>(
            Vec(SW_CTRL_X[2], SW_CTRL_Y), module, SwitchLines1to8::STEP_PARAM));
        addChild(createLightCentered<MediumLight<SWCyanLight>>(
            Vec(SW_CTRL_X[2], SW_CTRL_Y), module, SwitchLines1to8::STEP_LIGHT));

        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(SW_CTRL_X[3], SW_CTRL_Y), module, SwitchLines1to8::SIGNAL_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(SW_CTRL_X[4], SW_CTRL_Y), module, SwitchLines1to8::EOC_OUTPUT));

        // Channel outputs (top row) and gate outputs (bottom row)
        for (int i = 0; i < SW_N; ++i) {
            float cx = SW_CH_X0 + SW_CH_DX * (float)i;
            addOutput(createOutputCentered<DarkPJ301MPort>(
                Vec(cx, SW_OUT_Y + 3.f), module, SwitchLines1to8::CH_OUTPUT + i));
            addOutput(createOutputCentered<DarkPJ301MPort>(
                Vec(cx, SW_GATE_Y + 3.f), module, SwitchLines1to8::GATE_OUTPUT + i));
        }
    }
};

Model* modelSwitchLines1to8 = createModel<SwitchLines1to8, SwitchLines1to8Widget>("SwitchLines1to8");