#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <complex>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float PH_PANEL_W = RACK_GRID_WIDTH * 20;   // 300
static const float PH_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Display
static const float PH_DISP_W = 240.f;
static const float PH_DISP_H = 108.f;
static const float PH_DISP_X = (PH_PANEL_W - PH_DISP_W) / 2.f;   // 30
static const float PH_DISP_Y = 44.f;

// Plot area inside the display (display-local coordinates)
static const float PH_PX0 = 24.f,  PH_PX1 = 232.f;
static const float PH_PY0 = 17.f,  PH_PY1 = 77.f;
static const float PH_PW  = PH_PX1 - PH_PX0;
static const float PH_PH  = PH_PY1 - PH_PY0;
static const float PH_DB_TOP = 12.f, PH_DB_BOT = -36.f;

// LFO strip inside the display
static const float PH_LCY = 96.f;
static const float PH_LAMP = 6.f;

// Knobs (2 rows x 4)
static const float PH_KNOB_X[4]    = {45.f, 115.f, 185.f, 255.f};
static const float PH_ROW_Y[2]     = {204.f, 262.f};
static const float PH_SECTION_CTRL = 174.f;

// I/O row
static const float PH_SECTION_IO = 303.f;
static const float PH_IO_Y       = 340.f;
static const float PH_IO_X[8]    = {27.f, 60.f, 93.f, 126.f, 159.f, 192.f, 240.f, 273.f};

// ------------------------------------------------------------
// Shared helpers (DSP + display use the same maths)
// ------------------------------------------------------------
struct PhasmalViz {
    float fcL = 780.f, fcR = 780.f;
    float fcMin = 100.f, fcMax = 6000.f;
    float fb = 0.4f, mix = 0.5f, level = 1.f;
    float rate = 0.5f, depth = 0.6f, spread = 0.5f;
    float phase = 0.15f;
    int   stages = 6;
    int   shape = 0;
    float fs = 44100.f;
};

static inline float phLfo(float p, int shape) {
    p -= std::floor(p);
    if (shape == 1) {
        float q = p + 0.25f;
        q -= std::floor(q);
        return 1.f - 4.f * std::fabs(q - 0.5f);
    }
    return std::sin(2.f * (float)M_PI * p);
}

// First-order all-pass coefficient for a given corner frequency
static inline float phStageCoef(float fc, float fs) {
    float t = std::tan((float)M_PI * fc / fs);
    return (t - 1.f) / (t + 1.f);
}

// Exact magnitude response (dB) of the digital phaser at frequency f
static inline float phMagDb(float f, float fc, const PhasmalViz& v) {
    float fs = v.fs;
    f = std::min(f, 0.499f * fs);
    float w = 2.f * (float)M_PI * f / fs;
    std::complex<float> z1 = std::polar(1.f, -w);
    float a = phStageCoef(fc, fs);
    std::complex<float> A1 = (a + z1) / (1.f + a * z1);
    std::complex<float> A(1.f, 0.f);
    for (int i = 0; i < v.stages; ++i) A *= A1;
    std::complex<float> W = A / (1.f - v.fb * A * z1);
    float norm = 1.f - 0.5f * std::fabs(v.fb);
    std::complex<float> H = (1.f - v.mix) + v.mix * norm * W;
    return 20.f * std::log10(std::abs(H) + 1e-5f) + 20.f * std::log10(std::max(v.level, 1e-3f));
}

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Phasmal : Module {
    enum ParamId {
        RATE_PARAM,
        DEPTH_PARAM,
        FREQ_PARAM,
        FEEDBACK_PARAM,
        STAGES_PARAM,
        SPREAD_PARAM,
        MIX_PARAM,
        LEVEL_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        IN_L_INPUT,
        IN_R_INPUT,
        RATE_CV_INPUT,
        DEPTH_CV_INPUT,
        FREQ_CV_INPUT,
        FB_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    enum LfoShape { SHAPE_SINE, SHAPE_TRI, NUM_SHAPES };

    // DSP state
    float st[2][12] = {};
    float fbState[2] = {};
    float phase = 0.f;
    int   lastN = 6;
    LfoShape shape = SHAPE_SINE;

    // Read by the display
    PhasmalViz viz;

    Phasmal() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(RATE_PARAM, -5.f, 4.f, -1.f, "Rate", " Hz", 2.f, 1.f);
        configParam(DEPTH_PARAM, 0.f, 1.f, 0.6f, "Depth", "%", 0.f, 100.f);
        configParam(FREQ_PARAM, 6.64f, 12.3f, 9.6f, "Center frequency", " Hz", 2.f, 1.f);
        configParam(FEEDBACK_PARAM, -0.95f, 0.95f, 0.4f, "Feedback", "%", 0.f, 100.f);
        configSwitch(STAGES_PARAM, 0.f, 5.f, 2.f, "Stages", {"2", "4", "6", "8", "10", "12"});
        configParam(SPREAD_PARAM, 0.f, 1.f, 0.5f, "Stereo spread", " deg", 0.f, 180.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Mix", "%", 0.f, 100.f);
        configParam(LEVEL_PARAM, 0.f, 2.f, 1.f, "Output level", "%", 0.f, 100.f);

        configInput(IN_L_INPUT, "Left / mono");
        configInput(IN_R_INPUT, "Right (normalled to left)");
        configInput(RATE_CV_INPUT, "Rate CV (1 V/oct)");
        configInput(DEPTH_CV_INPUT, "Depth CV (+-5 V = +-50%)");
        configInput(FREQ_CV_INPUT, "Center frequency CV (1 V/oct)");
        configInput(FB_CV_INPUT, "Feedback CV (+-5 V = +-50%)");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");
    }

    void onReset() override {
        std::memset(st, 0, sizeof(st));
        std::memset(fbState, 0, sizeof(fbState));
        phase = 0.f;
    }

    void process(const ProcessArgs& args) override {
        const float fs = args.sampleRate;

        // ---- parameters + CV ----
        float rate = std::exp2(rack::math::clamp(
            params[RATE_PARAM].getValue() + inputs[RATE_CV_INPUT].getVoltage(), -7.f, 6.f));
        float depth = rack::math::clamp(
            params[DEPTH_PARAM].getValue() + inputs[DEPTH_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        float fb = rack::math::clamp(
            params[FEEDBACK_PARAM].getValue() + inputs[FB_CV_INPUT].getVoltage() * 0.1f, -0.95f, 0.95f);
        float fc0 = std::exp2(params[FREQ_PARAM].getValue() + inputs[FREQ_CV_INPUT].getVoltage());
        float spread = params[SPREAD_PARAM].getValue();
        float mix = params[MIX_PARAM].getValue();
        float level = params[LEVEL_PARAM].getValue();
        int N = 2 * ((int)std::round(params[STAGES_PARAM].getValue()) + 1);
        N = rack::math::clamp(N, 2, 12);

        if (N != lastN) {
            int from = std::min(N, lastN);
            for (int c = 0; c < 2; ++c)
                for (int i = from; i < 12; ++i)
                    st[c][i] = 0.f;
            lastN = N;
        }

        // ---- LFO ----
        phase += rate * args.sampleTime;
        phase -= std::floor(phase);
        float lfoL = phLfo(phase, shape);
        float lfoR = phLfo(phase + spread * 0.5f, shape);

        // ---- sweep ----
        const float sweep = depth * 3.f;                 // octaves each way
        const float fmax = std::min(18000.f, 0.45f * fs);
        float fcL = rack::math::clamp(fc0 * std::exp2(sweep * lfoL), 20.f, fmax);
        float fcR = rack::math::clamp(fc0 * std::exp2(sweep * lfoR), 20.f, fmax);

        // ---- audio ----
        float inL = inputs[IN_L_INPUT].getVoltage();
        float inR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getVoltage() : inL;

        const float norm = 1.f - 0.5f * std::fabs(fb);
        auto run = [&](int c, float in, float fc) {
            float a = phStageCoef(fc, fs);
            float x = in + fb * fbState[c];
            for (int i = 0; i < N; ++i) {
                float y = a * x + st[c][i];
                st[c][i] = x - a * y;
                x = y;
            }
            fbState[c] = 5.f * std::tanh(x * 0.2f);      // keep resonance under control
            return (in * (1.f - mix) + x * norm * mix) * level;
        };

        outputs[OUT_L_OUTPUT].setVoltage(run(0, inL, fcL));
        outputs[OUT_R_OUTPUT].setVoltage(run(1, inR, fcR));

        // ---- display data ----
        viz.fcL = fcL;  viz.fcR = fcR;
        viz.fcMin = rack::math::clamp(fc0 * std::exp2(-sweep), 20.f, fmax);
        viz.fcMax = rack::math::clamp(fc0 * std::exp2(sweep), 20.f, fmax);
        viz.fb = fb;  viz.mix = mix;  viz.level = level;
        viz.rate = rate;  viz.depth = depth;  viz.spread = spread;
        viz.phase = phase;
        viz.stages = N;
        viz.shape = (int)shape;
        viz.fs = fs;
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "shape", json_integer((int)shape));
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* j = json_object_get(root, "shape");
        if (j) {
            int s = (int)json_integer_value(j);
            shape = (LfoShape)rack::math::clamp(s, 0, (int)NUM_SHAPES - 1);
        }
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

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct PHPanelBackground : TransparentWidget {
    PHPanelBackground() { box.size = Vec(PH_PANEL_W, PH_PANEL_H); }

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
        float x1 = 12.f, x2 = PH_PANEL_W - 12.f, cx = PH_PANEL_W / 2.f;
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

    void jackCell(NVGcontext* vg, float cx, float cy, const char* txt, bool isOutput) {
        float w = 30.f;
        float x = cx - w / 2.f, y = cy - 27.f, h = 44.f;
        plate(vg, x, y, w, h);

        NVGcolor c = isOutput ? nvgRGBA(0x3d, 0xe0, 0xff, 170) : nvgRGBA(0xff, 0x9c, 0x6a, 170);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x + 6.f, y + 1.5f, w - 12.f, 1.8f, 0.9f);
        nvgFillColor(vg, c);
        nvgFill(vg);

        label(vg, cx, cy - 19.f, txt);
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

        // Corner vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PH_PANEL_W, PH_PANEL_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, PH_PANEL_W / 2.f, PH_PANEL_H * 0.4f,
                                           PH_PANEL_H * 0.35f, PH_PANEL_H * 0.85f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
        nvgFill(vg);

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, PH_PANEL_W - 1.f, PH_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Top accent line
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PH_PANEL_W, 2.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, PH_PANEL_W, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // Logo + title
        drawLogo(vg, PH_PANEL_W / 2.f, 15.f);
        drawText(vg, PH_PANEL_W / 2.f, 32.f, "PHASMAL", 12.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // ---- Display bezel (metallic, recessed) ----
        float bx = PH_DISP_X - 5.f, by = PH_DISP_Y - 5.f;
        float bw = PH_DISP_W + 10.f, bh = PH_DISP_H + 10.f;

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

        // ---- Knobs ----
        section(vg, PH_SECTION_CTRL, "CONTROL");
        const char* names[2][4] = {
            {"RATE", "DEPTH", "CENTER", "FEEDBACK"},
            {"STAGES", "SPREAD", "MIX", "LEVEL"}
        };
        for (int r = 0; r < 2; ++r) {
            for (int c = 0; c < 4; ++c) {
                float cx = PH_KNOB_X[c], cy = PH_ROW_Y[r];
                ring(vg, cx, cy, 17.f);
                bool stages = (r == 1 && c == 0);
                drawKnobTicks(vg, cx, cy, 21.f, 24.f, stages ? 6 : 11);
                label(vg, cx, cy + 29.f, names[r][c]);
            }
        }

        // ---- I/O ----
        section(vg, PH_SECTION_IO, "I / O");
        const char* io[8] = {"IN L", "IN R", "RATE", "DEPTH", "FREQ", "FDBK", "OUT L", "OUT R"};
        for (int i = 0; i < 8; ++i)
            jackCell(vg, PH_IO_X[i], PH_IO_Y, io[i], i >= 6);

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Live value arcs around the knobs
// ------------------------------------------------------------
struct PHKnobArcs : TransparentWidget {
    Phasmal* module = nullptr;

    PHKnobArcs(Phasmal* m) : module(m) { box.size = Vec(PH_PANEL_W, PH_PANEL_H); }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float R = 17.2f;
        const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;   // nvg angles
        const float a1 =  0.75f * (float)M_PI - (float)M_PI / 2.f;

        for (int k = 0; k < 8; ++k) {
            float cx = PH_KNOB_X[k % 4];
            float cy = PH_ROW_Y[k / 4];

            // track
            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, a0, a1, NVG_CW);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 2.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStroke(vg);

            if (!module) continue;
            float v = module->paramQuantities[k]->getScaledValue();   // 0..1
            bool bipolar = (k == Phasmal::FEEDBACK_PARAM);
            float from = bipolar ? 0.5f : 0.f;
            float lo = std::min(from, v), hi = std::max(from, v);
            if (hi - lo < 0.004f) continue;

            float s = a0 + (a1 - a0) * lo;
            float e = a0 + (a1 - a0) * hi;
            NVGcolor col = (bipolar && v < 0.5f) ? colWarm() : colAccent();

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, s, e, NVG_CW);
            nvgStrokeColor(vg, nvgRGBAf(col.r, col.g, col.b, 0.18f));
            nvgStrokeWidth(vg, 5.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, s, e, NVG_CW);
            nvgStrokeColor(vg, nvgRGBAf(col.r, col.g, col.b, 0.95f));
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
        }
    }
};

// ------------------------------------------------------------
// Phaser display: live frequency response + LFO scope
// ------------------------------------------------------------
struct PHDisplayWidget : TransparentWidget {
    Phasmal* module = nullptr;

    PHDisplayWidget(Phasmal* m) : module(m) {
        box.pos = Vec(PH_DISP_X, PH_DISP_Y);
        box.size = Vec(PH_DISP_W, PH_DISP_H);
    }

    static float xOfFreq(float f) {
        f = rack::math::clamp(f, 20.f, 20000.f);
        return PH_PX0 + (std::log10(f) - 1.30103f) / 3.f * PH_PW;
    }
    static float yOfDb(float db) {
        db = rack::math::clamp(db, PH_DB_BOT, PH_DB_TOP);
        return PH_PY0 + (PH_DB_TOP - db) / (PH_DB_TOP - PH_DB_BOT) * PH_PH;
    }

    // ---- static background: screen + grid ----
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0, 0, PH_DISP_W, PH_DISP_H, 4.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PH_DISP_H,
                                           nvgRGB(0x0a, 0x16, 0x1f), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0.5f, 0.5f, PH_DISP_W - 1.f, PH_DISP_H - 1.f, 4.f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Vertical frequency grid
        const float freqs[10] = {20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
        for (int i = 0; i < 10; ++i) {
            bool major = (freqs[i] == 100.f || freqs[i] == 1000.f || freqs[i] == 10000.f);
            float x = xOfFreq(freqs[i]);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x + 0.5f, PH_PY0);
            nvgLineTo(vg, x + 0.5f, PH_PY1);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, major ? 30 : 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }
        // Horizontal dB grid
        const float dbs[5] = {12.f, 0.f, -12.f, -24.f, -36.f};
        const char* dbl[5] = {"+12", "0", "-12", "-24", "-36"};
        for (int i = 0; i < 5; ++i) {
            float y = yOfDb(dbs[i]);
            nvgBeginPath(vg);
            nvgMoveTo(vg, PH_PX0, y + 0.5f);
            nvgLineTo(vg, PH_PX1, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, dbs[i] == 0.f ? 44 : 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, PH_PX0 - 3.f, y, dbl[i], 6.f, nvgRGBA(0x8a, 0x95, 0xa5, 170),
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);
        }
        // Frequency labels
        drawText(vg, xOfFreq(100.f),   PH_PY1 + 6.f, "100", 6.f, nvgRGBA(0x8a, 0x95, 0xa5, 170),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, xOfFreq(1000.f),  PH_PY1 + 6.f, "1k", 6.f, nvgRGBA(0x8a, 0x95, 0xa5, 170),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, xOfFreq(10000.f), PH_PY1 + 6.f, "10k", 6.f, nvgRGBA(0x8a, 0x95, 0xa5, 170),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        // LFO strip frame
        const float lx0 = PH_PX0, lw = PH_PW;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, lx0, PH_LCY - PH_LAMP - 3.f, lw, 2.f * PH_LAMP + 6.f, 3.f);
        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 8));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 26));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgMoveTo(vg, lx0 + 2.f, PH_LCY + 0.5f);
        nvgLineTo(vg, lx0 + lw - 2.f, PH_LCY + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 16));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        drawText(vg, PH_PX0 - 3.f, PH_LCY, "LFO", 6.f, nvgRGBA(0x8a, 0x95, 0xa5, 170),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);

        Widget::draw(args);
    }

    // Response curve (optionally filled)
    void drawCurve(NVGcontext* vg, const PhasmalViz& v, float fc, NVGcolor col,
                   bool fill, float widthCore, float alphaCore) {
        const int NP = 100;
        float xs[NP], ys[NP];
        for (int i = 0; i < NP; ++i) {
            float f = 20.f * std::pow(10.f, 3.f * (float)i / (float)(NP - 1));
            xs[i] = xOfFreq(f);
            ys[i] = yOfDb(phMagDb(f, fc, v));
        }

        if (fill) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, xs[0], PH_PY1);
            for (int i = 0; i < NP; ++i) nvgLineTo(vg, xs[i], ys[i]);
            nvgLineTo(vg, xs[NP - 1], PH_PY1);
            nvgClosePath(vg);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, PH_PY0, 0, PH_PY1,
                                               nvgRGBAf(col.r, col.g, col.b, 0.28f),
                                               nvgRGBAf(col.r, col.g, col.b, 0.02f)));
            nvgFill(vg);
        }

        nvgLineCap(vg, NVG_ROUND);
        nvgLineJoin(vg, NVG_ROUND);
        // glow
        nvgBeginPath(vg);
        nvgMoveTo(vg, xs[0], ys[0]);
        for (int i = 1; i < NP; ++i) nvgLineTo(vg, xs[i], ys[i]);
        nvgStrokeColor(vg, nvgRGBAf(col.r, col.g, col.b, 0.12f));
        nvgStrokeWidth(vg, widthCore + 3.f);
        nvgStroke(vg);
        // core
        nvgBeginPath(vg);
        nvgMoveTo(vg, xs[0], ys[0]);
        for (int i = 1; i < NP; ++i) nvgLineTo(vg, xs[i], ys[i]);
        nvgStrokeColor(vg, nvgRGBAf(col.r, col.g, col.b, alphaCore));
        nvgStrokeWidth(vg, widthCore);
        nvgStroke(vg);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        PhasmalViz v;                     // demo values in the module browser
        if (module) v = module->viz;

        nvgSave(vg);
        nvgScissor(vg, 0, 0, PH_DISP_W, PH_DISP_H);

        // ---- sweep range band ----
        float bx0 = xOfFreq(v.fcMin), bx1 = xOfFreq(v.fcMax);
        if (bx1 - bx0 < 1.5f) bx1 = bx0 + 1.5f;
        nvgBeginPath(vg);
        nvgRect(vg, bx0, PH_PY0, bx1 - bx0, PH_PH);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, PH_PY0, 0, PH_PY1,
                                           nvgRGBA(0x3d, 0xe0, 0xff, 26), nvgRGBA(0x3d, 0xe0, 0xff, 6)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgMoveTo(vg, bx0 + 0.5f, PH_PY0);  nvgLineTo(vg, bx0 + 0.5f, PH_PY1);
        nvgMoveTo(vg, bx1 + 0.5f, PH_PY0);  nvgLineTo(vg, bx1 + 0.5f, PH_PY1);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // ---- response curves (right channel underneath, only when it differs) ----
        bool showR = (v.spread > 0.02f && v.depth > 0.01f);
        if (showR) drawCurve(vg, v, v.fcR, colWarm(), false, 1.1f, 0.85f);
        drawCurve(vg, v, v.fcL, colBright(), true, 1.5f, 0.95f);

        // ---- centre-frequency markers ----
        float xl = xOfFreq(v.fcL), xr = xOfFreq(v.fcR);
        if (showR) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, xr + 0.5f, PH_PY0);  nvgLineTo(vg, xr + 0.5f, PH_PY1);
            nvgStrokeColor(vg, nvgRGBA(0xff, 0x9c, 0x6a, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, xr + 0.5f, PH_PY0 + 1.5f, 2.f);
            nvgFillColor(vg, nvgRGBA(0xff, 0x9c, 0x6a, 230));
            nvgFill(vg);
        }
        nvgBeginPath(vg);
        nvgMoveTo(vg, xl + 0.5f, PH_PY0);  nvgLineTo(vg, xl + 0.5f, PH_PY1);
        nvgStrokeColor(vg, nvgRGBA(0x7a, 0xec, 0xff, 110));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgCircle(vg, xl + 0.5f, PH_PY0 + 1.5f, 2.2f);
        nvgFillColor(vg, nvgRGBA(0xc8, 0xf6, 0xff, 240));
        nvgFill(vg);

        // ---- LFO scope ----
        {
            const float lx0 = PH_PX0 + 3.f, lw = PH_PW - 6.f;
            const int NP = 80;
            float off = v.spread * 0.5f;

            for (int ch = 1; ch >= 0; --ch) {
                if (ch == 1 && !showR) continue;
                nvgBeginPath(vg);
                for (int i = 0; i < NP; ++i) {
                    float p = (float)i / (float)(NP - 1);
                    float y = PH_LCY - phLfo(p + (ch ? off : 0.f), v.shape) * PH_LAMP;
                    float x = lx0 + p * lw;
                    if (i == 0) nvgMoveTo(vg, x, y); else nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, ch ? nvgRGBA(0xff, 0x9c, 0x6a, 150) : nvgRGBA(0x7a, 0xec, 0xff, 200));
                nvgStrokeWidth(vg, ch ? 1.f : 1.2f);
                nvgStroke(vg);
            }

            float cxp = lx0 + (v.phase - std::floor(v.phase)) * lw;
            nvgBeginPath(vg);
            nvgMoveTo(vg, cxp, PH_LCY - PH_LAMP - 2.f);
            nvgLineTo(vg, cxp, PH_LCY + PH_LAMP + 2.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 70));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            float yL = PH_LCY - phLfo(v.phase, v.shape) * PH_LAMP;
            float yR = PH_LCY - phLfo(v.phase + off, v.shape) * PH_LAMP;
            if (showR) {
                nvgBeginPath(vg);
                nvgCircle(vg, cxp, yR, 2.f);
                nvgFillColor(vg, nvgRGBA(0xff, 0x9c, 0x6a, 240));
                nvgFill(vg);
            }
            nvgBeginPath(vg);
            nvgCircle(vg, cxp, yL, 2.4f);
            nvgFillColor(vg, nvgRGBA(0xc8, 0xf6, 0xff, 250));
            nvgFill(vg);
        }

        // ---- readouts ----
        {
            char buf[48];
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 220);
            NVGcolor cm = nvgRGBA(0xff, 0x9c, 0x6a, 230);
            NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 230);

            if (v.rate < 10.f) snprintf(buf, sizeof(buf), "RATE %.2f Hz", v.rate);
            else               snprintf(buf, sizeof(buf), "RATE %.1f Hz", v.rate);
            drawText(vg, 5.f, 7.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

            snprintf(buf, sizeof(buf), "FC %.0f Hz  N%d", v.fcL, v.stages);
            drawText(vg, PH_DISP_W / 2.f, 7.f, buf, 7.f, cd, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

            snprintf(buf, sizeof(buf), "FB %+.0f%%  MIX %.0f%%", v.fb * 100.f, v.mix * 100.f);
            drawText(vg, PH_DISP_W - 5.f, 7.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        }

        // ---- scanlines, vignette, glass ----
        nvgBeginPath(vg);
        for (int y = 0; y < (int) PH_DISP_H; y += 3)
            nvgRect(vg, 0, y + 1, PH_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PH_DISP_W, PH_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, PH_DISP_W / 2.f, PH_DISP_H / 2.f,
                                           PH_DISP_W * 0.40f, PH_DISP_W * 0.80f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 130)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PH_DISP_W, PH_DISP_H * 0.30f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PH_DISP_H * 0.30f,
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
struct PhasmalWidget : ModuleWidget {
    PhasmalWidget(Phasmal* module) {
        setModule(module);
        box.size = Vec(PH_PANEL_W, PH_PANEL_H);

        addChild(new PHPanelBackground());
        addChild(new PHKnobArcs(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PHDisplayWidget(module));

        // Knobs: row 1 = RATE, DEPTH, CENTER, FEEDBACK; row 2 = STAGES, SPREAD, MIX, LEVEL
        for (int k = 0; k < 8; ++k) {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(PH_KNOB_X[k % 4], PH_ROW_Y[k / 4]), module, k));
        }

        // Inputs
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[0], PH_IO_Y), module, Phasmal::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[1], PH_IO_Y), module, Phasmal::IN_R_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[2], PH_IO_Y), module, Phasmal::RATE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[3], PH_IO_Y), module, Phasmal::DEPTH_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[4], PH_IO_Y), module, Phasmal::FREQ_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[5], PH_IO_Y), module, Phasmal::FB_CV_INPUT));

        // Outputs
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PH_IO_X[6], PH_IO_Y), module, Phasmal::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PH_IO_X[7], PH_IO_Y), module, Phasmal::OUT_R_OUTPUT));
    }

    void appendContextMenu(ui::Menu* menu) override {
        Phasmal* module = dynamic_cast<Phasmal*>(this->module);
        if (!module) return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("LFO shape", "",
            [=](ui::Menu* submenu) {
                const char* names[] = {"Sine", "Triangle"};
                for (int i = 0; i < Phasmal::NUM_SHAPES; ++i) {
                    submenu->addChild(createMenuItem(names[i],
                        ((int)module->shape == i) ? "✔" : "",
                        [=]() { module->shape = (Phasmal::LfoShape)i; }));
                }
            }
        ));
    }
};

Model* modelPhasmal = createModel<Phasmal, PhasmalWidget>("Phasmal");