#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// 1/11/2026

static const float GT_W = RACK_GRID_WIDTH * 24;   // 360 (double width)
static const float GT_H = RACK_GRID_HEIGHT;       // 380

static const float GT_DX = 30.f, GT_DY = 46.f, GT_DW = 300.f, GT_DH = 180.f;

// Inputs (row above display)
static const float GT_IN_Y = 260.f;
static float gtInJx(int i) { return GT_W / 2.f - 50.f + 100.f * i; }

// Time knob (below display, left)
static const float GT_KNOBX = 60.f;
static const float GT_KNOBY = 260.f;

// Outputs (row below the display / knob)
static const float GT_OUT_Y = 330.f;
static float gtOutJx(int i) { return 30.f + 46.f * i; }

// Log mapping helper: knob t (0..1) -> ms/screen in [5, 50000]
//   t=0   -> 50000 ms
//   t=0.5 ->   500 ms  (default)
//   t=1   ->     5 ms
static inline float timeKnobToMs(float t) {
    t = clamp(t, 0.f, 1.f);
    return 50000.f * std::pow(5.f / 50000.f, t);
}
static inline float msToTimeKnob(float ms) {
    if (ms <= 0.f) return 0.f;
    float t = std::log(ms / 50000.f) / std::log(5.f / 50000.f);
    return clamp(t, 0.f, 1.f);
}

struct TimeQuantity : ParamQuantity {
    float getDisplayValue() override {
        return timeKnobToMs(getValue());
    }
    void setDisplayValue(float v) override {
        setValue(msToTimeKnob(v));
    }
    std::string getDisplayValueString() override {
        float ms = getDisplayValue();
        char buf[32];
        if (ms >= 1000.f) snprintf(buf, sizeof(buf), "%.2f s/screen", ms / 1000.f);
        else              snprintf(buf, sizeof(buf), "%.1f ms/screen", ms);
        return std::string(buf);
    }
};

struct Gates : Module {
    enum ParamId { PARAM_TIME, PARAMS_LEN };
    enum InputId { INPUT_A, INPUT_B, INPUTS_LEN };
    enum OutputId {
        OUTPUT_AND, OUTPUT_OR, OUTPUT_NOT, OUTPUT_NAND,
        OUTPUT_NOR, OUTPUT_XOR, OUTPUT_XNOR, OUTPUTS_LEN
    };
    enum LightId {
        LIGHT_A, LIGHT_B,
        LIGHT_AND, LIGHT_OR, LIGHT_NOT, LIGHT_NAND,
        LIGHT_NOR, LIGHT_XOR, LIGHT_XNOR,
        LIGHTS_LEN
    };

    bool aNow = false;
    bool bNow = false;
    bool outNow[7] = {};

    // History for display (channel 0)
    static const int HIST = 512;
    bool histA[HIST] = {};
    bool histB[HIST] = {};
    bool histOut[7][HIST] = {};
    int histPos = 0;

    // TIME: ms per screen.
    float screenMs = 500.f;

    Gates() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        // Default 0.5 == exactly 500 ms/screen (log mapping)
        configParam<TimeQuantity>(PARAM_TIME, 0.f, 1.f, 0.5f, "Time");

        configInput(INPUT_A, "Input A");
        configInput(INPUT_B, "Input B");
        configOutput(OUTPUT_AND, "AND");
        configOutput(OUTPUT_OR, "OR");
        configOutput(OUTPUT_NOT, "NOT");
        configOutput(OUTPUT_NAND, "NAND");
        configOutput(OUTPUT_NOR, "NOR");
        configOutput(OUTPUT_XOR, "XOR");
        configOutput(OUTPUT_XNOR, "XNOR");

        configLight(LIGHT_A, "Input A active");
        configLight(LIGHT_B, "Input B active");
        configLight(LIGHT_AND, "AND active");
        configLight(LIGHT_OR, "OR active");
        configLight(LIGHT_NOT, "NOT active");
        configLight(LIGHT_NAND, "NAND active");
        configLight(LIGHT_NOR, "NOR active");
        configLight(LIGHT_XOR, "XOR active");
        configLight(LIGHT_XNOR, "XNOR active");
    }

    void process(const ProcessArgs& args) override {
        // Same mapping used by TimeQuantity::getDisplayValue()
        screenMs = timeKnobToMs(params[PARAM_TIME].getValue());

        int aCh = inputs[INPUT_A].isConnected() ? inputs[INPUT_A].getChannels() : 0;
        int bCh = inputs[INPUT_B].isConnected() ? inputs[INPUT_B].getChannels() : 0;
        int channels = std::max(std::max(aCh, bCh), 1);

        bool a0 = false, b0 = false;
        bool o0[7] = {};

        for (int c = 0; c < channels; ++c) {
            bool a = inputs[INPUT_A].getVoltage(c) > 1.f;
            bool b = inputs[INPUT_B].getVoltage(c) > 1.f;

            bool g_and  = (a && b);
            bool g_or   = (a || b);
            bool g_not  = !a;
            bool g_nand = !(a && b);
            bool g_nor  = !(a || b);
            bool g_xor  = (a != b);
            bool g_xnor = (a == b);

            outputs[OUTPUT_AND].setVoltage(g_and   ? 10.f : 0.f, c);
            outputs[OUTPUT_OR].setVoltage(g_or     ? 10.f : 0.f, c);
            outputs[OUTPUT_NOT].setVoltage(g_not   ? 10.f : 0.f, c);
            outputs[OUTPUT_NAND].setVoltage(g_nand ? 10.f : 0.f, c);
            outputs[OUTPUT_NOR].setVoltage(g_nor   ? 10.f : 0.f, c);
            outputs[OUTPUT_XOR].setVoltage(g_xor   ? 10.f : 0.f, c);
            outputs[OUTPUT_XNOR].setVoltage(g_xnor ? 10.f : 0.f, c);

            if (c == 0) {
                a0 = a; b0 = b;
                o0[0] = g_and; o0[1] = g_or; o0[2] = g_not;
                o0[3] = g_nand; o0[4] = g_nor; o0[5] = g_xor; o0[6] = g_xnor;
            }
        }

        for (int o = 0; o < OUTPUTS_LEN; ++o)
            outputs[o].setChannels(channels);

        aNow = a0;
        bNow = b0;
        for (int i = 0; i < 7; ++i) outNow[i] = o0[i];

        lights[LIGHT_A].setBrightness(a0 ? 1.f : 0.f);
        lights[LIGHT_B].setBrightness(b0 ? 1.f : 0.f);
        lights[LIGHT_AND].setBrightness(o0[0] ? 1.f : 0.f);
        lights[LIGHT_OR].setBrightness(o0[1] ? 1.f : 0.f);
        lights[LIGHT_NOT].setBrightness(o0[2] ? 1.f : 0.f);
        lights[LIGHT_NAND].setBrightness(o0[3] ? 1.f : 0.f);
        lights[LIGHT_NOR].setBrightness(o0[4] ? 1.f : 0.f);
        lights[LIGHT_XOR].setBrightness(o0[5] ? 1.f : 0.f);
        lights[LIGHT_XNOR].setBrightness(o0[6] ? 1.f : 0.f);

        // Sample history at a rate determined by screenMs.
        double slotSec = (screenMs / 1000.0) / (double) HIST;
        histAccum += args.sampleTime;
        while (histAccum >= slotSec) {
            histAccum -= slotSec;
            histA[histPos] = a0;
            histB[histPos] = b0;
            for (int i = 0; i < 7; ++i) histOut[i][histPos] = o0[i];
            histPos = (histPos + 1) % HIST;
        }
    }

    double histAccum = 0.0;
};

namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colHi()       { return nvgRGB(0x50, 0xff, 0x90); }
NVGcolor colLo()       { return nvgRGB(0x3a, 0x40, 0x4a); }

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
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h, nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
    nvgFill(vg);
    nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
    nvgStrokeWidth(vg, 1.2f);
    nvgStroke(vg);
    const float s = 2.2f;
    float tx = x + 7.f, ty = y + h / 2.f - 3.f;
    nvgBeginPath(vg);
    nvgRect(vg, tx + s, ty, s, s);
    nvgRect(vg, tx, ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);
    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

void drawPanelBase(NVGcontext* vg, float W, float H, const char* title) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
    nvgFill(vg);
    nvgStrokeWidth(vg, 1.f);
    for (int y = 0; y < (int) H; y += 3) {
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0, y + 0.5f);
        nvgLineTo(vg, W, y + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
        nvgStroke(vg);
    }
    nvgBeginPath(vg);
    nvgRect(vg, 0.5f, 0.5f, W - 1.f, H - 1.f);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
    nvgStroke(vg);
    drawLogo(vg, W / 2.f, 13.f);
    drawText(vg, W / 2.f, 31.f, title, 13.f, colText(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);
}

void drawBezel(NVGcontext* vg, float x, float y, float w, float h) {
    float bx = x - 5.f, by = y - 5.f, bw = w + 10.f, bh = h + 10.f;
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh, nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
    nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
    nvgFill(vg);
}

void plate(NVGcontext* vg, float x, float y, float w, float h) {
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 6.f);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 80));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

void ring(NVGcontext* vg, float cx, float cy, float r) {
    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, r);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

void label(NVGcontext* vg, float x, float y, const char* t, float size = 7.f) {
    drawText(vg, x, y, t, size, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
}

void screenBase(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 20.f, W * 0.75f, nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
    nvgFill(vg);
}

void screenOverlay(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    for (int y = 0; y < (int) H; y += 3) nvgRect(vg, 0, y + 1, W, 1);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H * 0.3f);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f, nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
    nvgFill(vg);
}

struct GTPanel : TransparentWidget {
    GTPanel() { box.size = Vec(GT_W, GT_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, GT_W, GT_H, "GATES");
        drawBezel(vg, GT_DX, GT_DY, GT_DW, GT_DH);

        // Input plates + labels (top)
        const char* inNames[2] = {"A", "B"};
        for (int i = 0; i < 2; ++i) {
            float x = gtInJx(i);
            plate(vg, x - 17.f, GT_IN_Y - 27.f, 34.f, 44.f);
            label(vg, x, GT_IN_Y - 19.f, inNames[i]);
        }

        // Time knob ring + label
        ring(vg, GT_KNOBX, GT_KNOBY, 18.f);
        label(vg, GT_KNOBX, GT_KNOBY + 25.f, "TIME");

        // Output labels under the jacks
        const char* outs[7] = {"AND", "OR", "NOT", "NAND", "NOR", "XOR", "XNOR"};
        for (int i = 0; i < 7; ++i)
            label(vg, gtOutJx(i), GT_OUT_Y + 26.f, outs[i], 6.f);

        Widget::draw(args);
    }
};

struct GTDisplay : TransparentWidget {
    Gates* module = nullptr;
    GTDisplay(Gates* m) : module(m) {
        box.pos = Vec(GT_DX, GT_DY);
        box.size = Vec(GT_DW, GT_DH);
    }

    static const int ROWS = 9;

    void drawRow(NVGcontext* vg, float x, float y, float w, float h,
                 const char* title, bool curHigh,
                 const bool* hist, int len, int pos, NVGcolor color) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 2.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
        nvgFill(vg);

        float traceW = w - 50.f;
        float traceX = x + 2.f;
        float hiY = y + 2.5f;
        float loY = y + h - 2.5f;

        if (hist && len > 1) {
            float step = traceW / (float)(len - 1);
            nvgBeginPath(vg);
            bool prev = false;
            for (int i = 0; i < len; ++i) {
                int idx = (pos + i) % len;
                bool v = hist[idx];
                float px = traceX + i * step;
                float py = v ? hiY : loY;
                if (i == 0) {
                    nvgMoveTo(vg, px, py);
                } else if (v != prev) {
                    float mx = traceX + (i - 0.5f) * step;
                    nvgLineTo(vg, mx, prev ? hiY : loY);
                    nvgLineTo(vg, mx, py);
                    nvgLineTo(vg, px, py);
                } else {
                    nvgLineTo(vg, px, py);
                }
                prev = v;
            }
            nvgStrokeColor(vg, color);
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
        }

        float dotX = traceX + traceW + 8.f;
        nvgBeginPath(vg);
        nvgCircle(vg, dotX, y + h * 0.5f, 3.f);
        nvgFillColor(vg, curHigh ? colHi() : colLo());
        nvgFill(vg);

        drawText(vg, x + w - 6.f, y + h * 0.5f, title, 7.f, colText(),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        screenBase(vg, GT_DW, GT_DH);
        screenOverlay(vg, GT_DW, GT_DH);

        const char* titles[ROWS] = {"A", "B", "AND", "OR", "NOT", "NAND", "NOR", "XOR", "XNOR"};
        NVGcolor colors[ROWS] = {
            colAccent(), colAccent(),
            colHi(), colHi(), colHi(), colHi(), colHi(), colHi(), colHi()
        };

        bool cur[ROWS] = {};
        const bool* hist[ROWS] = {};
        int len = 0, pos = 0;

        if (module) {
            cur[0] = module->aNow;
            cur[1] = module->bNow;
            for (int i = 0; i < 7; ++i) cur[2 + i] = module->outNow[i];

            hist[0] = module->histA;
            hist[1] = module->histB;
            for (int i = 0; i < 7; ++i) hist[2 + i] = module->histOut[i];

            len = Gates::HIST;
            pos = module->histPos;
        }

        const float topPad = 4.f;
        const float rowH = (GT_DH - topPad * 2.f) / (float)ROWS;
        const float rowGap = 1.f;

        for (int r = 0; r < ROWS; ++r) {
            float y = topPad + r * rowH + rowGap * 0.5f;
            float h = rowH - rowGap;
            drawRow(vg, 4.f, y, GT_DW - 8.f, h, titles[r], cur[r],
                    hist[r], len, pos, colors[r]);
        }

        // TIME readout
        if (module) {
            char buf[32];
            float ms = module->screenMs;
            if (ms >= 1000.f) snprintf(buf, sizeof(buf), "%.2f s/scr", ms / 1000.f);
            else              snprintf(buf, sizeof(buf), "%.1f ms/scr", ms);
            drawText(vg, GT_DW - 6.f, GT_DH - 6.f, buf, 6.f, colDim(),
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);
        }

        Widget::draw(args);
    }
};

} // namespace

struct GatesWidget : ModuleWidget {
    GatesWidget(Gates* module) {
        setModule(module);
        box.size = Vec(GT_W, GT_H);

        addChild(new GTPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new GTDisplay(module));

        // Inputs
        addInput(createInputCentered<DarkPJ301MPort>(Vec(gtInJx(0), GT_IN_Y), module, Gates::INPUT_A));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(gtInJx(1), GT_IN_Y), module, Gates::INPUT_B));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtInJx(0) - 30, GT_IN_Y), module, Gates::LIGHT_A));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtInJx(1) + 30, GT_IN_Y), module, Gates::LIGHT_B));

        // Time knob (below display, left)
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(GT_KNOBX, GT_KNOBY), module, Gates::PARAM_TIME));

        // Outputs below display
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(0), GT_OUT_Y), module, Gates::OUTPUT_AND));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(1), GT_OUT_Y), module, Gates::OUTPUT_OR));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(2), GT_OUT_Y), module, Gates::OUTPUT_NOT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(3), GT_OUT_Y), module, Gates::OUTPUT_NAND));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(4), GT_OUT_Y), module, Gates::OUTPUT_NOR));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(5), GT_OUT_Y), module, Gates::OUTPUT_XOR));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(6), GT_OUT_Y), module, Gates::OUTPUT_XNOR));

        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtOutJx(0), GT_OUT_Y - 22.f), module, Gates::LIGHT_AND));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtOutJx(1), GT_OUT_Y - 22.f), module, Gates::LIGHT_OR));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtOutJx(2), GT_OUT_Y - 22.f), module, Gates::LIGHT_NOT));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtOutJx(3), GT_OUT_Y - 22.f), module, Gates::LIGHT_NAND));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtOutJx(4), GT_OUT_Y - 22.f), module, Gates::LIGHT_NOR));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtOutJx(5), GT_OUT_Y - 22.f), module, Gates::LIGHT_XOR));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(gtOutJx(6), GT_OUT_Y - 22.f), module, Gates::LIGHT_XNOR));
    }
};

Model* modelGates = createModel<Gates, GatesWidget>("Gates");