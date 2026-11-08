#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout (24 HP = 360 x 380)
// Every control sits in a plate with its label ABOVE it.
// ------------------------------------------------------------
static const float GT_W = RACK_GRID_WIDTH * 24; // 360
static const float GT_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it). Rows, top to bottom:
//   y  9      header (channel, time per screen)
//   20..      9 trace rows, pitch 16, height 14:
//             label column | trace | state dot
static const float GT_DX = 30.f, GT_DY = 32.f, GT_DW = 300.f, GT_DH = 170.f;
static const float GT_ROW0 = 20.f, GT_ROW_PITCH = 16.f, GT_ROW_H = 14.f;
static const float GT_LABEL_W = 32.f; // label column inside a row
static const float GT_DOT_W = 18.f;   // state dot column inside a row

// Control row: A, B inputs (with LEDs) and the TIME knob
static const float GT_CTRL_HEAD_Y = 220.f;
static const float GT_CTRL_TOP = 230.f, GT_CTRL_H = 52.f, GT_CTRL_W = 60.f;
static const float GT_INX[2] = {75.f, 145.f};
static const float GT_KNOBX = 285.f;
static const float GT_IN_Y = GT_CTRL_TOP + 31.f;
static const float GT_KNOBY = GT_CTRL_TOP + 32.f;

// Output row: 7 jacks, each with its LED next to the label
static const float GT_OUT_HEAD_Y = 296.f;
static const float GT_OUT_TOP = 306.f, GT_OUT_H = 46.f, GT_OUT_W = 42.f;
static const float GT_OUT_Y = GT_OUT_TOP + 31.f;
static const float GT_OUT_LED_Y = GT_OUT_TOP + 10.f;
static float gtOutJx(int i) { return 42.f + 46.f * i; }

// Log mapping helper: knob t (0..1) -> ms/screen in [5, 50000]
//   t=0   -> 50000 ms
//   t=0.5 ->   500 ms  (default)
//   t=1   ->     5 ms
static inline float timeKnobToMs(float t)
{
    t = clamp(t, 0.f, 1.f);
    return 50000.f * std::pow(5.f / 50000.f, t);
}
static inline float msToTimeKnob(float ms)
{
    if (ms <= 0.f)
        return 0.f;
    float t = std::log(ms / 50000.f) / std::log(5.f / 50000.f);
    return clamp(t, 0.f, 1.f);
}

struct TimeQuantity : ParamQuantity
{
    float getDisplayValue() override
    {
        return timeKnobToMs(getValue());
    }
    void setDisplayValue(float v) override
    {
        setValue(msToTimeKnob(v));
    }
    std::string getDisplayValueString() override
    {
        float ms = getDisplayValue();
        char buf[32];
        if (ms >= 1000.f)
            snprintf(buf, sizeof(buf), "%.2f s/screen", ms / 1000.f);
        else
            snprintf(buf, sizeof(buf), "%.1f ms/screen", ms);
        return std::string(buf);
    }
};

// ------------------------------------------------------------
// Module (logic unchanged)
// ------------------------------------------------------------
struct Gates : Module
{
    enum ParamId
    {
        PARAM_TIME,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_A,
        INPUT_B,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_AND,
        OUTPUT_OR,
        OUTPUT_NOT,
        OUTPUT_NAND,
        OUTPUT_NOR,
        OUTPUT_XOR,
        OUTPUT_XNOR,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHT_A,
        LIGHT_B,
        LIGHT_AND,
        LIGHT_OR,
        LIGHT_NOT,
        LIGHT_NAND,
        LIGHT_NOR,
        LIGHT_XOR,
        LIGHT_XNOR,
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

    Gates()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        // Default 0.5 == exactly 500 ms/screen (log mapping)
        configParam<TimeQuantity>(PARAM_TIME, 0.f, 1.f, 0.5f, "Time");

        configInput(INPUT_A, "Input A");
        configInput(INPUT_B, "Input B");
        configOutput(OUTPUT_AND, "AND");
        configOutput(OUTPUT_OR, "OR");
        configOutput(OUTPUT_NOT, "NOT (A)");
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

    void process(const ProcessArgs &args) override
    {
        // Same mapping used by TimeQuantity::getDisplayValue()
        screenMs = timeKnobToMs(params[PARAM_TIME].getValue());

        int aCh = inputs[INPUT_A].isConnected() ? inputs[INPUT_A].getChannels() : 0;
        int bCh = inputs[INPUT_B].isConnected() ? inputs[INPUT_B].getChannels() : 0;
        int channels = std::max(std::max(aCh, bCh), 1);

        bool a0 = false, b0 = false;
        bool o0[7] = {};

        for (int c = 0; c < channels; ++c)
        {
            bool a = inputs[INPUT_A].getVoltage(c) > 1.f;
            bool b = inputs[INPUT_B].getVoltage(c) > 1.f;

            bool g_and = (a && b);
            bool g_or = (a || b);
            bool g_not = !a;
            bool g_nand = !(a && b);
            bool g_nor = !(a || b);
            bool g_xor = (a != b);
            bool g_xnor = (a == b);

            outputs[OUTPUT_AND].setVoltage(g_and ? 10.f : 0.f, c);
            outputs[OUTPUT_OR].setVoltage(g_or ? 10.f : 0.f, c);
            outputs[OUTPUT_NOT].setVoltage(g_not ? 10.f : 0.f, c);
            outputs[OUTPUT_NAND].setVoltage(g_nand ? 10.f : 0.f, c);
            outputs[OUTPUT_NOR].setVoltage(g_nor ? 10.f : 0.f, c);
            outputs[OUTPUT_XOR].setVoltage(g_xor ? 10.f : 0.f, c);
            outputs[OUTPUT_XNOR].setVoltage(g_xnor ? 10.f : 0.f, c);

            if (c == 0)
            {
                a0 = a;
                b0 = b;
                o0[0] = g_and;
                o0[1] = g_or;
                o0[2] = g_not;
                o0[3] = g_nand;
                o0[4] = g_nor;
                o0[5] = g_xor;
                o0[6] = g_xnor;
            }
        }

        for (int o = 0; o < OUTPUTS_LEN; ++o)
            outputs[o].setChannels(channels);

        aNow = a0;
        bNow = b0;
        for (int i = 0; i < 7; ++i)
            outNow[i] = o0[i];

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
        double slotSec = (screenMs / 1000.0) / (double)HIST;
        histAccum += args.sampleTime;
        while (histAccum >= slotSec)
        {
            histAccum -= slotSec;
            histA[histPos] = a0;
            histB[histPos] = b0;
            for (int i = 0; i < 7; ++i)
                histOut[i][histPos] = o0[i];
            histPos = (histPos + 1) % HIST;
        }
    }

    double histAccum = 0.0;
};

// ------------------------------------------------------------
// UI (anonymous namespace: names cannot clash with other module files)
// ------------------------------------------------------------
namespace
{

    struct GTPanel : TransparentWidget
    {
        GTPanel() { box.size = Vec(GT_W, GT_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, GT_W, GT_H, "GATES");
            agmm::bezel(vg, GT_DX, GT_DY, GT_DW, GT_DH);

            // ---- control row: inputs A / B and TIME ----
            agmm::sectionHeader(vg, GT_W, GT_CTRL_HEAD_Y, "CONTROL", 14.f);
            agmm::cell(vg, GT_INX[0], GT_CTRL_TOP, GT_CTRL_W, GT_CTRL_H, "IN A");
            agmm::cell(vg, GT_INX[1], GT_CTRL_TOP, GT_CTRL_W, GT_CTRL_H, "IN B");
            agmm::cell(vg, GT_KNOBX, GT_CTRL_TOP, GT_CTRL_W, GT_CTRL_H, "TIME");

            // ---- output row (tinted plates, label on top) ----
            agmm::sectionHeader(vg, GT_W, GT_OUT_HEAD_Y, "OUTPUT", 14.f);
            const char *outs[7] = {"AND", "OR", "NOT", "NAND", "NOR", "XOR", "XNOR"};
            for (int i = 0; i < 7; ++i)
                agmm::cell(vg, gtOutJx(i), GT_OUT_TOP, GT_OUT_W, GT_OUT_H, outs[i], true);

            Widget::draw(args);
        }
    };

    struct GTDisplay : TransparentWidget
    {
        Gates *module = nullptr;
        GTDisplay(Gates *m) : module(m)
        {
            box.pos = Vec(GT_DX, GT_DY);
            box.size = Vec(GT_DW, GT_DH);
        }

        static const int ROWS = 9;

        // One trace row: label column on the left, trace in the middle, state dot on the right
        void drawRow(NVGcontext *vg, float x, float y, float w, float h,
                     const char *title, bool curHigh,
                     const bool *hist, int len, int pos, NVGcolor color)
        {
            nvgBeginPath(vg);
            nvgRect(vg, x, y, w, h);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 10));
            nvgFill(vg);

            agmm::text(vg, x + 5.f, y + h * 0.5f + 0.5f, title, 7.f, agmm::textCol(), agmm::ALIGN_L, 0.3f);

            const float traceX = x + GT_LABEL_W;
            const float traceW = w - GT_LABEL_W - GT_DOT_W;
            const float hiY = y + 2.5f;
            const float loY = y + h - 2.5f;

            if (hist && len > 1)
            {
                float step = traceW / (float)(len - 1);
                nvgBeginPath(vg);
                bool prev = false;
                for (int i = 0; i < len; ++i)
                {
                    int idx = (pos + i) % len;
                    bool v = hist[idx];
                    float px = traceX + i * step;
                    float py = v ? hiY : loY;
                    if (i == 0)
                    {
                        nvgMoveTo(vg, px, py);
                    }
                    else if (v != prev)
                    {
                        float mx = traceX + (i - 0.5f) * step;
                        nvgLineTo(vg, mx, prev ? hiY : loY);
                        nvgLineTo(vg, mx, py);
                        nvgLineTo(vg, px, py);
                    }
                    else
                    {
                        nvgLineTo(vg, px, py);
                    }
                    prev = v;
                }
                nvgStrokeColor(vg, color);
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }

            nvgBeginPath(vg);
            nvgCircle(vg, x + w - GT_DOT_W / 2.f, y + h * 0.5f, 3.f);
            nvgFillColor(vg, curHigh ? color : nvgRGBA(255, 255, 255, 40));
            nvgFill(vg);
        }

        void draw(const DrawArgs &args) override
        {
            agmm::screen(args.vg, 0.f, 0.f, GT_DW, GT_DH);
            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            const char *titles[ROWS] = {"A", "B", "AND", "OR", "NOT", "NAND", "NOR", "XOR", "XNOR"};
            NVGcolor colors[ROWS] = {
                agmm::violet(), agmm::violet(),
                agmm::accent(), agmm::accent(), agmm::accent(), agmm::accent(),
                agmm::accent(), agmm::accent(), agmm::accent()};

            bool cur[ROWS] = {};
            const bool *hist[ROWS] = {};
            int len = 0, pos = 0;
            float ms = 500.f;

            if (module)
            {
                cur[0] = module->aNow;
                cur[1] = module->bNow;
                for (int i = 0; i < 7; ++i)
                    cur[2 + i] = module->outNow[i];

                hist[0] = module->histA;
                hist[1] = module->histB;
                for (int i = 0; i < 7; ++i)
                    hist[2 + i] = module->histOut[i];

                len = Gates::HIST;
                pos = module->histPos;
                ms = module->screenMs;
            }

            // ---- header row ----
            char buf[32];
            agmm::text(vg, 8.f, 9.f, "CHANNEL 1", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
            if (ms >= 1000.f)
                snprintf(buf, sizeof(buf), "%.2f S / SCREEN", ms / 1000.f);
            else
                snprintf(buf, sizeof(buf), "%.1f MS / SCREEN", ms);
            agmm::text(vg, GT_DW - 8.f, 9.f, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

            // ---- trace rows ----
            for (int r = 0; r < ROWS; ++r)
                drawRow(vg, 4.f, GT_ROW0 + r * GT_ROW_PITCH, GT_DW - 8.f, GT_ROW_H, titles[r], cur[r],
                        hist[r], len, pos, colors[r]);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

struct GatesWidget : ModuleWidget
{
    GatesWidget(Gates *module)
    {
        setModule(module);
        box.size = Vec(GT_W, GT_H);

        addChild(new GTPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new GTDisplay(module));

        // Inputs: jack left of centre, its activity LED on the right of the same plate
        addInput(createInputCentered<DarkPJ301MPort>(Vec(GT_INX[0] - 9.f, GT_IN_Y), module, Gates::INPUT_A));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(GT_INX[1] - 9.f, GT_IN_Y), module, Gates::INPUT_B));
        addChild(createLightCentered<SmallLight<agmm::CyanLight>>(Vec(GT_INX[0] + 17.f, GT_IN_Y), module, Gates::LIGHT_A));
        addChild(createLightCentered<SmallLight<agmm::CyanLight>>(Vec(GT_INX[1] + 17.f, GT_IN_Y), module, Gates::LIGHT_B));

        // Time knob
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(GT_KNOBX, GT_KNOBY), module, Gates::PARAM_TIME));

        // Outputs
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(0), GT_OUT_Y), module, Gates::OUTPUT_AND));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(1), GT_OUT_Y), module, Gates::OUTPUT_OR));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(2), GT_OUT_Y), module, Gates::OUTPUT_NOT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(3), GT_OUT_Y), module, Gates::OUTPUT_NAND));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(4), GT_OUT_Y), module, Gates::OUTPUT_NOR));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(5), GT_OUT_Y), module, Gates::OUTPUT_XOR));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(gtOutJx(6), GT_OUT_Y), module, Gates::OUTPUT_XNOR));

        // Output LEDs sit on the label row, to the right of the text
        const int ledIds[7] = {Gates::LIGHT_AND, Gates::LIGHT_OR, Gates::LIGHT_NOT, Gates::LIGHT_NAND,
                               Gates::LIGHT_NOR, Gates::LIGHT_XOR, Gates::LIGHT_XNOR};
        for (int i = 0; i < 7; ++i)
            addChild(createLightCentered<SmallLight<agmm::CyanLight>>(
                Vec(gtOutJx(i) + 16.f, GT_OUT_LED_Y), module, ledIds[i]));
    }
};

Model *modelGates = createModel<Gates, GatesWidget>("Gates");