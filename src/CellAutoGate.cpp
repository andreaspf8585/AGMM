#include "plugin.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <cstring>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   CELL_SIZE   = 4;
static const int   COLS        = 64;                 // fits exactly in a uint64_t
static const int   ROWS        = 64;
static const float DISP_W      = COLS * CELL_SIZE;   // 256
static const float DISP_H      = ROWS * CELL_SIZE;   // 256
static const float PANEL_W     = RACK_GRID_WIDTH * 32;   // 480
static const float PANEL_H     = RACK_GRID_HEIGHT;       // 380
static const float DISP_X      = (PANEL_W - DISP_W) / 2.f;
static const float DISP_Y      = 44.f;

static const float LEFT_X      = 56.f;
static const float RIGHT_X     = PANEL_W - 56.f;

// Palette
static NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
static NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
static NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
static NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
static NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct CellAutoGate : Module
{
    enum ParamId
    {
        RULE_PARAM,
        SPEED_PARAM,
        POLY_PARAM,
        RANDOM_PARAM,
        RESET_PARAM,
        PARAMS_LEN
    };
    enum InputId  { CLOCK_INPUT, INPUTS_LEN };
    enum OutputId { GATE_OUTPUT, OUTPUTS_LEN };
    enum LightId
    {
        GATE_LIGHT_1,
        LIGHTS_LEN = GATE_LIGHT_1 + 16
    };

    // Automaton state: one bit per column
    uint64_t currentRow = 0;
    int ruleset[8] = {0};

    // History ring buffer (read by the UI thread; a torn read is only cosmetic)
    std::array<uint64_t, ROWS> history{};
    int histHead  = 0;   // next write index
    int histCount = 0;

    // Timing / triggers
    float timeSinceLastStep = 0.f;
    dsp::SchmittTrigger randomTrigger, resetTrigger, clockTrigger;

    int polyChannels = 1;
    int lastRule = -1;
    bool clockActive = false;

    CellAutoGate()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(RULE_PARAM, 0.f, 255.f, 30.f, "Rule");
        configParam(SPEED_PARAM, 1.f, 160.f, 10.f, "Speed", " Hz");
        configParam(POLY_PARAM, 1.f, 16.f, 1.f, "Polyphony", " channels");
        getParamQuantity(RULE_PARAM)->snapEnabled = true;
        getParamQuantity(POLY_PARAM)->snapEnabled = true;
        getParamQuantity(SPEED_PARAM)->snapEnabled = true;

        configButton(RANDOM_PARAM, "Randomize");
        configButton(RESET_PARAM, "Reset");
        configInput(CLOCK_INPUT, "Clock");
        configOutput(GATE_OUTPUT, "Gates");

        for (int i = 0; i < 16; ++i)
            configLight(GATE_LIGHT_1 + i, string::f("Gate %d", i + 1));

        int rule = (int) params[RULE_PARAM].getValue();
        updateRuleset(rule);
        lastRule = rule;
        restart();
    }

    void onReset() override { restart(); }

    void updateRuleset(int rule)
    {
        for (int i = 0; i < 8; ++i)
            ruleset[i] = (rule >> i) & 1;
    }

    void pushHistory(uint64_t row)
    {
        history[histHead] = row;
        histHead = (histHead + 1) % ROWS;
        if (histCount < ROWS)
            histCount++;
    }

    // Row r (0 = oldest) for display
    uint64_t getHistoryRow(int r) const
    {
        int oldest = (histHead - histCount + ROWS * 2) % ROWS;
        return history[(oldest + r) % ROWS];
    }

    void restart()
    {
        currentRow = 1ULL << (COLS / 2);
        histHead = histCount = 0;
        pushHistory(currentRow);
    }

    void randomize()
    {
        uint64_t r = 0;
        for (int i = 0; i < COLS; ++i)
            if (random::uniform() > 0.5f)
                r |= (1ULL << i);
        currentRow = r;
        histHead = histCount = 0;
        pushHistory(currentRow);
    }

    void stepAutomaton()
    {
        uint64_t next = 0;
        for (int i = 0; i < COLS; ++i)
        {
            int left   = (currentRow >> ((i + COLS - 1) % COLS)) & 1;
            int center = (currentRow >> i) & 1;
            int right  = (currentRow >> ((i + 1) % COLS)) & 1;
            int index  = (left << 2) | (center << 1) | right;
            next |= (uint64_t) ruleset[index] << i;
        }
        currentRow = next;
        pushHistory(currentRow);
    }

    void process(const ProcessArgs &args) override
    {
        int rule = (int) params[RULE_PARAM].getValue();
        if (rule != lastRule)
        {
            updateRuleset(rule);
            restart();
            lastRule = rule;
        }

        int newChannels = clamp((int) params[POLY_PARAM].getValue(), 1, 16);
        if (newChannels != polyChannels || outputs[GATE_OUTPUT].getChannels() != newChannels)
        {
            polyChannels = newChannels;
            outputs[GATE_OUTPUT].setChannels(polyChannels);
        }

        if (randomTrigger.process(params[RANDOM_PARAM].getValue()))
            randomize();
        if (resetTrigger.process(params[RESET_PARAM].getValue()))
            restart();

        clockActive = inputs[CLOCK_INPUT].isConnected();
        if (clockActive)
        {
            if (clockTrigger.process(inputs[CLOCK_INPUT].getVoltage()))
                stepAutomaton();
            timeSinceLastStep = 0.f;
        }
        else
        {
            float interval = 1.f / params[SPEED_PARAM].getValue();
            timeSinceLastStep += args.sampleTime;
            if (timeSinceLastStep >= interval)
            {
                stepAutomaton();
                timeSinceLastStep -= interval;
                if (timeSinceLastStep > interval) // safety after long pauses
                    timeSinceLastStep = 0.f;
            }
        }

        int N = polyChannels;
        for (int ch = 0; ch < N; ++ch)
        {
            int pos = ((ch + 1) * COLS) / (N + 1);
            if (pos >= COLS) pos = COLS - 1;
            bool on = (currentRow >> pos) & 1;
            outputs[GATE_OUTPUT].setVoltage(on ? 10.f : 0.f, ch);
            lights[GATE_LIGHT_1 + ch].setSmoothBrightness(on ? 1.f : 0.f, args.sampleTime);
        }
        for (int ch = N; ch < 16; ++ch)
            lights[GATE_LIGHT_1 + ch].setBrightness(0.f);
    }
};

// ------------------------------------------------------------
// Drawing helpers
// ------------------------------------------------------------
static void setFont(NVGcontext *vg)
{
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font)
        nvgFontFaceId(vg, font->handle);
}

static void drawText(NVGcontext *vg, float x, float y, const char *s,
                     float size, NVGcolor col, int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE,
                     float spacing = 1.f)
{
    setFont(vg);
    nvgFontSize(vg, size);
    nvgTextLetterSpacing(vg, spacing);
    nvgTextAlign(vg, align);
    nvgFillColor(vg, col);
    nvgText(vg, x, y, s, NULL);
}

// ------------------------------------------------------------
// Panel background (all static artwork)
// ------------------------------------------------------------
struct PanelBackground : TransparentWidget
{
    PanelBackground() { box.size = Vec(PANEL_W, PANEL_H); }

    void label(NVGcontext *vg, float x, float y, const char *txt)
    {
        drawText(vg, x, y, txt, 9.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);
    }

    void section(NVGcontext *vg, float x1, float x2, float y, const char *txt)
    {
        // Line with a centered caption
        drawText(vg, (x1 + x2) / 2.f, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        float w = 4.f + strlen(txt) * 5.5f;
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y); nvgLineTo(vg, (x1 + x2) / 2.f - w / 2.f, y);
        nvgMoveTo(vg, (x1 + x2) / 2.f + w / 2.f, y); nvgLineTo(vg, x2, y);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void draw(const DrawArgs &args) override
    {
        NVGcontext *vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PANEL_W, PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Subtle brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) PANEL_H; y += 3)
        {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Outer edge highlights
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, PANEL_W - 1.f, PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Header
        drawText(vg, PANEL_W / 2.f, 16.f, "CELLAUTOGATE", 15.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, PANEL_W / 2.f - 70.f, 27.f);
        nvgLineTo(vg, PANEL_W / 2.f + 70.f, 27.f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 140));
        nvgStrokeWidth(vg, 1.5f);
        nvgStroke(vg);

        // Display bezel
        float bx = DISP_X - 8.f, by = DISP_Y - 8.f, bw = DISP_W + 16.f, bh = DISP_H + 16.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // Left / right control sections
        section(vg, 14.f, DISP_X - 22.f, 46.f - 22.f + 0.f, "CONTROL");
        section(vg, DISP_X + DISP_W + 22.f, PANEL_W - 14.f, 24.f, "TRIGGER");
        section(vg, DISP_X + DISP_W + 22.f, PANEL_W - 14.f, 252.f, "OUTPUT");

        // Knob / port labels
        label(vg, LEFT_X, 100.f, "RULE");
        label(vg, LEFT_X, 170.f, "SPEED");
        label(vg, LEFT_X, 240.f, "POLY");
        label(vg, LEFT_X, 322.f, "CLOCK");

        label(vg, RIGHT_X, 100.f, "RANDOM");
        label(vg, RIGHT_X, 170.f, "RESET");
        label(vg, RIGHT_X, 322.f, "GATES");

        // Port plates
        for (float cx : {LEFT_X, RIGHT_X})
        {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 22.f, 276.f, 44.f, 44.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // LED caption
        label(vg, PANEL_W / 2.f, 370.f, "POLY GATE ACTIVITY");

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Display widget (cells, rule glyphs, status readout)
// ------------------------------------------------------------
struct CellularDisplayWidget : TransparentWidget
{
    CellAutoGate *module = nullptr;

    CellularDisplayWidget(CellAutoGate *m) : module(m)
    {
        box.pos  = Vec(DISP_X, DISP_Y);
        box.size = Vec(DISP_W, DISP_H);
    }

    void draw(const DrawArgs &args) override
    {
        NVGcontext *vg = args.vg;

        // Screen background with vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DISP_W, DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, DISP_W / 2.f, DISP_H / 2.f, 40.f, DISP_W * 0.8f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        // Faint grid
        nvgBeginPath(vg);
        for (int i = 0; i <= COLS; i += 8)
        {
            nvgMoveTo(vg, i * CELL_SIZE + 0.5f, 0);
            nvgLineTo(vg, i * CELL_SIZE + 0.5f, DISP_H);
            nvgMoveTo(vg, 0, i * CELL_SIZE + 0.5f);
            nvgLineTo(vg, DISP_W, i * CELL_SIZE + 0.5f);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

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

        // Cells (self-illuminated, newest row brightest)
        int count = module ? module->histCount : 0;
        uint64_t demoRows[ROWS];
        if (!module)
        {
            // Browser preview: render rule 30 from a single seed
            uint64_t cur = 1ULL << (COLS / 2);
            for (int r = 0; r < ROWS; ++r)
            {
                demoRows[r] = cur;
                uint64_t next = 0;
                for (int i = 0; i < COLS; ++i)
                {
                    int l = (cur >> ((i + COLS - 1) % COLS)) & 1;
                    int c = (cur >> i) & 1;
                    int rr = (cur >> ((i + 1) % COLS)) & 1;
                    next |= (uint64_t) ((30 >> ((l << 2) | (c << 1) | rr)) & 1) << i;
                }
                cur = next;
            }
            count = ROWS;
        }

        const float inset = 0.4f;
        for (int pass = 0; pass < 2; ++pass)   // pass 0: glow, pass 1: crisp cell
        {
            for (int r = 0; r < count; ++r)
            {
                uint64_t bits = module ? module->getHistoryRow(r) : demoRows[r];
                if (!bits) continue;
                float age = (float) (r + 1) / (float) std::max(count, 1);  // 0..1 (1 = newest)
                float a = 0.25f + 0.75f * age * age;

                nvgBeginPath(vg);
                for (int x = 0; x < COLS; ++x)
                {
                    if (!((bits >> x) & 1)) continue;
                    if (pass == 0)
                        nvgRect(vg, x * CELL_SIZE - 1.f, r * CELL_SIZE - 1.f, CELL_SIZE + 2.f, CELL_SIZE + 2.f);
                    else
                        nvgRect(vg, x * CELL_SIZE + inset, r * CELL_SIZE + inset,
                                CELL_SIZE - 2 * inset, CELL_SIZE - 2 * inset);
                }
                if (pass == 0)
                    nvgFillColor(vg, nvgRGBAf(0.24f, 0.88f, 1.f, 0.10f * a));
                else
                    nvgFillColor(vg, nvgRGBAf(0.24f + 0.55f * age * age, 0.88f + 0.10f * age * age, 1.f, a));
                nvgFill(vg);
            }
        }

        // Newest-row marker line
        if (module && count > 0 && count < ROWS)
        {
            nvgBeginPath(vg);
            nvgRect(vg, 0, count * CELL_SIZE, DISP_W, 1.f);
            nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
            nvgFill(vg);
        }

        // Scanline overlay for a CRT feel
        nvgBeginPath(vg);
        for (int y = 0; y < (int) DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DISP_W, DISP_H * 0.35f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DISP_H * 0.35f,
                                           nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Status readout (above display, in panel space)
        int rule = module ? module->lastRule : 30;
        char status[96];
        if (module)
        {
            if (module->clockActive)
                snprintf(status, sizeof(status), "RULE %03d   |   EXT CLOCK   |   %d CH",
                         rule, module->polyChannels);
            else
                snprintf(status, sizeof(status), "RULE %03d   |   %3.0f HZ   |   %d CH",
                         rule, module->params[CellAutoGate::SPEED_PARAM].getValue(),
                         module->polyChannels);
        }
        else
            snprintf(status, sizeof(status), "RULE %03d   |   10 HZ   |   1 CH", rule);

        nvgSave(vg);
        nvgTranslate(vg, 0, -22.f);
        drawText(vg, DISP_W / 2.f, 0.f, status, 10.f, colAccent(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        nvgRestore(vg);

        // Rule lookup glyphs (8 neighbourhoods -> result)
        const float slotW = DISP_W / 8.f;
        const float gy = DISP_H + 22.f;
        const float cs = 6.f;
        for (int k = 0; k < 8; ++k)
        {
            int pattern = 7 - k;   // 111 ... 000
            int res = module ? module->ruleset[pattern] : ((30 >> pattern) & 1);
            float cx = k * slotW + slotW / 2.f;

            for (int j = 0; j < 3; ++j)
            {
                bool on = (pattern >> (2 - j)) & 1;
                float x = cx - 1.5f * cs - 1.f + j * (cs + 1.f);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, gy, cs, cs, 1.f);
                if (on)
                {
                    nvgFillColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 230));
                    nvgFill(vg);
                }
                else
                {
                    nvgStrokeColor(vg, nvgRGBA(0x7a, 0x85, 0x94, 200));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
            }
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - cs / 2.f, gy + cs + 4.f, cs, cs, 1.f);
            if (res)
            {
                nvgFillColor(vg, nvgRGB(0x3d, 0xe0, 0xff));
                nvgFill(vg);
                // glow
                nvgBeginPath(vg);
                nvgRoundedRect(vg, cx - cs / 2.f - 2.f, gy + cs + 2.f, cs + 4.f, cs + 4.f, 2.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                nvgFill(vg);
            }
            else
            {
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 120));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
        }

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// Custom cyan light (Rack has no built-in CyanLight)
// ------------------------------------------------------------
template <typename TBase = GrayModuleLightWidget>
struct TCyanLight : TBase
{
    TCyanLight()
    {
        this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff));
    }
};
using CyanLight = TCyanLight<>;

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct CellAutoGateWidget : ModuleWidget
{
    CellAutoGateWidget(CellAutoGate *module)
    {
        setModule(module);
        box.size = Vec(PANEL_W, PANEL_H);

        addChild(new PanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new CellularDisplayWidget(module));

        // Left column
        addParam(createParamCentered<RoundBigBlackKnob>(Vec(LEFT_X, 70.f),  module, CellAutoGate::RULE_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(LEFT_X, 140.f), module, CellAutoGate::SPEED_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(LEFT_X, 210.f), module, CellAutoGate::POLY_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(LEFT_X, 298.f), module, CellAutoGate::CLOCK_INPUT));

        // Right column
        addParam(createParamCentered<VCVButton>(Vec(RIGHT_X, 70.f),  module, CellAutoGate::RANDOM_PARAM));
        addParam(createParamCentered<VCVButton>(Vec(RIGHT_X, 140.f), module, CellAutoGate::RESET_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(RIGHT_X, 298.f), module, CellAutoGate::GATE_OUTPUT));

        // Gate LEDs
        float ledStartX = DISP_X + 8.f;
        float ledEndX   = DISP_X + DISP_W - 8.f;
        float ledY      = 354.f;
        float spacing   = (ledEndX - ledStartX) / 15.f;
        for (int i = 0; i < 16; ++i)
            addChild(createLightCentered<SmallLight<CyanLight>>(
                Vec(ledStartX + i * spacing, ledY), module, CellAutoGate::GATE_LIGHT_1 + i));
    }
};

Model *modelCellAutoGate = createModel<CellAutoGate, CellAutoGateWidget>("CellAutoGate");