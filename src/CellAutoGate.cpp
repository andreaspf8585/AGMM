#include "plugin.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <memory>

// =============================================================================
// Cell Auto Gate
// =============================================================================
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// CellAutoGate runs a one-dimensional elementary cellular automaton (Wolfram
// rules 0-255) on a 64-cell row and turns the cells into a 1-16 channel
// polyphonic gate signal. Each channel samples the automaton at an evenly
// spaced position along the row, so different channels fire different
// rhythms from the same rule.
//
// The module can be stepped either by its internal clock (SPEED knob) or by
// an external clock patch into CLOCK. RANDOM fills the row with random bits,
// RESET restarts from a single centre cell. The display shows the most recent
// 64 rows of history (newest brightest), with a status row above and a rule
// truth-table glyph row below.
//
// File structure:
//   1. Layout constants (32 HP = 480x380 px).
//   2. CellAutoGate module: automaton state, ruleset, history buffer,
//      clocking and poly gate outputs.
//   3. Anonymous-namespace UI helpers and widgets:
//        - theme colours / font / text / plate / cell helpers
//        - CAPanelBackground: static artwork
//        - CellularDisplayWidget: history display (static grid + live cells)
//        - CAStatusWidget: status text row above the display
//        - CARuleTableWidget: rule truth-table glyphs under the display
//        - TCaCyanLight: custom cyan LED widget
//   4. CellAutoGateWidget: places all controls and LEDs.
//   5. Model registration.
// -----------------------------------------------------------------------------

// =============================================================================
// Layout (32 HP = 480 x 380 px)
//
//   y   0-15   screws
//   y  15      title
//   y  32      status readout (own row above the display)
//   y  42-306  display bezel; display itself 256 x 256 at (112, 46)
//   y  46-296  left column: RULE / SPEED / POLY / CLOCK (label above control)
//   y  46-146  right column: RANDOM / RESET;  y 250-296: GATES output
//   y 314-330  rule table glyphs (own row under the display)
//   y 344      16 gate LEDs
//   y 356      LED caption
//   y 369      maker tag
//
// No text is ever drawn over the display; every control is inside its plate.
// =============================================================================
static const int CELL_SIZE = 4;
static const int COLS = 64; // fits exactly in a uint64_t
static const int ROWS = 64;
static const float DISP_W = COLS * CELL_SIZE;         // 256
static const float DISP_H = ROWS * CELL_SIZE;         // 256
static const float PANEL_W = RACK_GRID_WIDTH * 32;    // 480
static const float PANEL_H = RACK_GRID_HEIGHT;        // 380
static const float DISP_X = (PANEL_W - DISP_W) / 2.f; // 112
static const float DISP_Y = 46.f;

static const float CA_TITLE_Y = 15.f;
static const float CA_MAKER_Y = 369.f;
static const float CA_STATUS_Y = 32.f;
static const float CA_GLYPH_Y = 314.f; // top of the rule glyph row
static const float CA_LED_Y = 344.f;
static const float CA_LEDCAP_Y = 356.f;

static const float CA_LEFT_X = 56.f; // column centres (plates are 84 wide)
static const float CA_RIGHT_X = PANEL_W - 56.f;
static const float CA_COL_W = 84.f;

// Left column: plate tops / heights, label and control offsets from the top
static const float CA_RULE_TOP = 46.f, CA_RULE_H = 58.f;
static const float CA_SPEED_TOP = 114.f, CA_SPEED_H = 58.f;
static const float CA_POLY_TOP = 182.f, CA_POLY_H = 58.f;
static const float CA_CLOCK_TOP = 250.f, CA_CLOCK_H = 46.f;
// Right column
static const float CA_RAND_TOP = 46.f;
static const float CA_RESET_TOP = 100.f;
static const float CA_GATES_TOP = 250.f;
static const float CA_SMALL_H = 46.f;

static const float CA_LABEL_DY = 10.f;
static const float CA_KNOB_DY = 36.f; // knob plates (h 58)
static const float CA_JACK_DY = 31.f; // jack / button plates (h 46)

// =============================================================================
// Module
// =============================================================================
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
    enum InputId
    {
        CLOCK_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        GATE_OUTPUT,
        OUTPUTS_LEN
    };
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
    int histHead = 0; // next write index
    int histCount = 0;

    // Timing / triggers
    float timeSinceLastStep = 0.f;
    dsp::SchmittTrigger randomTrigger, resetTrigger, clockTrigger;

    int polyChannels = 1;
    int lastRule = -1;
    bool clockActive = false;

    // Constructor: configures params, input, output and lights, builds the
    // initial ruleset from the RULE knob and restarts the automaton.
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
        configInput(CLOCK_INPUT, "Clock (overrides the speed knob)");
        configOutput(GATE_OUTPUT, "Gates (polyphonic)");

        for (int i = 0; i < 16; ++i)
            configLight(GATE_LIGHT_1 + i, string::f("Gate %d", i + 1));

        int rule = (int)params[RULE_PARAM].getValue();
        updateRuleset(rule);
        lastRule = rule;
        restart();
    }

    // Resets the automaton state (same as pressing RESET).
    void onReset() override { restart(); }

    // Expands a Wolfram rule number (0-255) into the 8-entry lookup table
    // indexed by the 3-bit neighbourhood (left, centre, right).
    void updateRuleset(int rule)
    {
        for (int i = 0; i < 8; ++i)
            ruleset[i] = (rule >> i) & 1;
    }

    // Writes one row into the history ring buffer, advancing the head and
    // saturating the count at ROWS.
    void pushHistory(uint64_t row)
    {
        history[histHead] = row;
        histHead = (histHead + 1) % ROWS;
        if (histCount < ROWS)
            histCount++;
    }

    // Row r (0 = oldest) for display
    // Returns the r-th history row counting from the oldest stored row.
    uint64_t getHistoryRow(int r) const
    {
        int oldest = (histHead - histCount + ROWS * 2) % ROWS;
        return history[(oldest + r) % ROWS];
    }

    // Restarts the automaton from a single centre cell and clears history.
    void restart()
    {
        currentRow = 1ULL << (COLS / 2);
        histHead = histCount = 0;
        pushHistory(currentRow);
    }

    // Fills the current row with random bits (about 50% density) and clears
    // history to just that row.
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

    // Advances the automaton by one generation, wrapping at the row edges
    // (circular neighbourhood), and pushes the new row into history.
    void stepAutomaton()
    {
        uint64_t next = 0;
        for (int i = 0; i < COLS; ++i)
        {
            int left = (currentRow >> ((i + COLS - 1) % COLS)) & 1;
            int center = (currentRow >> i) & 1;
            int right = (currentRow >> ((i + 1) % COLS)) & 1;
            int index = (left << 2) | (center << 1) | right;
            next |= (uint64_t)ruleset[index] << i;
        }
        currentRow = next;
        pushHistory(currentRow);
    }

    // Main callback: rebuilds the ruleset if the rule changed, applies the
    // RANDOM/RESET buttons, advances the automaton on the external clock or
    // the internal SPEED timer, then writes the poly gate outputs and LEDs.
    void process(const ProcessArgs &args) override
    {
        int rule = (int)params[RULE_PARAM].getValue();
        if (rule != lastRule)
        {
            updateRuleset(rule);
            restart();
            lastRule = rule;
        }

        int newChannels = clamp((int)params[POLY_PARAM].getValue(), 1, 16);
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
            if (pos >= COLS)
                pos = COLS - 1;
            bool on = (currentRow >> pos) & 1;
            outputs[GATE_OUTPUT].setVoltage(on ? 10.f : 0.f, ch);
            lights[GATE_LIGHT_1 + ch].setSmoothBrightness(on ? 1.f : 0.f, args.sampleTime);
        }
        for (int ch = N; ch < 16; ++ch)
            lights[GATE_LIGHT_1 + ch].setBrightness(0.f);
    }
};

// =============================================================================
// Theme + drawing helpers
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

    // Centred text alignment preset.
    const int ALIGN_C = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE;

    // Loads the AGMM monospace font and makes it the current NanoVG font
    // face. Returns false if there is no window or the asset is missing.
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
                  int align = ALIGN_C, float spacing = 0.5f)
    {
        if (!setFont(vg))
            return;
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Control plate; tint = output jack
    void plate(NVGcontext *vg, float cx, float top, float w, float h, bool tint = false)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - w / 2.f, top, w, h, 5.f);
        nvgFillColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 22) : nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 60) : nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Draws a standard column cell: an 84-wide plate with its label near the
    // top. `tint` marks output jacks.
    void cell(NVGcontext *vg, float cx, float top, float h, const char *label, bool tint = false)
    {
        plate(vg, cx, top, CA_COL_W, h, tint);
        drawText(vg, cx, top + CA_LABEL_DY, label, 8.f, colDim(), ALIGN_C, 1.f);
    }

    // ------------------------------------------------------------
    // Panel background (static artwork only)
    // ------------------------------------------------------------
    struct CAPanelBackground : TransparentWidget
    {
        CAPanelBackground() { box.size = Vec(PANEL_W, PANEL_H); }

        // Draws the opaque background, title, maker tag, display bezel, the
        // left and right column cell plates, and the LED caption.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, PANEL_W, PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            drawText(vg, PANEL_W / 2.f, CA_TITLE_Y, "CELL AUTO GATE", 12.f, colText(), ALIGN_C, 3.f);
            drawText(vg, PANEL_W / 2.f, CA_MAKER_Y, "AGMM", 9.f, colDim(), ALIGN_C, 3.f);

            // Display bezel
            nvgBeginPath(vg);
            nvgRoundedRect(vg, DISP_X - 4.f, DISP_Y - 4.f, DISP_W + 8.f, DISP_H + 8.f, 5.f);
            nvgFillColor(vg, nvgRGB(0x0b, 0x0c, 0x0e));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Left column
            cell(vg, CA_LEFT_X, CA_RULE_TOP, CA_RULE_H, "RULE");
            cell(vg, CA_LEFT_X, CA_SPEED_TOP, CA_SPEED_H, "SPEED");
            cell(vg, CA_LEFT_X, CA_POLY_TOP, CA_POLY_H, "POLY");
            cell(vg, CA_LEFT_X, CA_CLOCK_TOP, CA_CLOCK_H, "CLOCK");

            // Right column (output tinted)
            cell(vg, CA_RIGHT_X, CA_RAND_TOP, CA_SMALL_H, "RANDOM");
            cell(vg, CA_RIGHT_X, CA_RESET_TOP, CA_SMALL_H, "RESET");
            cell(vg, CA_RIGHT_X, CA_GATES_TOP, CA_SMALL_H, "GATES", true);

            // LED caption
            drawText(vg, PANEL_W / 2.f, CA_LEDCAP_Y, "POLY GATE ACTIVITY", 7.f, colDim(), ALIGN_C, 1.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display widget: the automaton history only
    // ------------------------------------------------------------
    struct CellularDisplayWidget : TransparentWidget
    {
        CellAutoGate *module = nullptr;

        CellularDisplayWidget(CellAutoGate *m) : module(m)
        {
            box.pos = Vec(DISP_X, DISP_Y);
            box.size = Vec(DISP_W, DISP_H);
        }

        // Static layer: dark display fill and a faint 8-cell grid.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, DISP_W, DISP_H);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
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

        // Layer-1 overlay: draws the stored automaton history, one row per
        // generation, with the newest rows brightest. Falls back to a live
        // rule-30 preview in the module browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer == 1)
            {
                NVGcontext *vg = args.vg;

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
                            next |= (uint64_t)((30 >> ((l << 2) | (c << 1) | rr)) & 1) << i;
                        }
                        cur = next;
                    }
                    count = ROWS;
                }

                // Cells: newest row brightest
                const float inset = 0.4f;
                for (int r = 0; r < count; ++r)
                {
                    uint64_t bits = module ? module->getHistoryRow(r) : demoRows[r];
                    if (!bits)
                        continue;
                    float age = (float)(r + 1) / (float)std::max(count, 1); // 0..1 (1 = newest)
                    float a = 0.3f + 0.7f * age * age;

                    nvgBeginPath(vg);
                    for (int x = 0; x < COLS; ++x)
                    {
                        if (!((bits >> x) & 1))
                            continue;
                        nvgRect(vg, x * CELL_SIZE + inset, r * CELL_SIZE + inset,
                                CELL_SIZE - 2 * inset, CELL_SIZE - 2 * inset);
                    }
                    nvgFillColor(vg, nvgRGBAf(0.24f, 0.88f, 1.f, a));
                    nvgFill(vg);
                }

                // Newest-row marker line
                if (module && count > 0 && count < ROWS)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, 0, count * CELL_SIZE, DISP_W, 1.f);
                    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
                    nvgFill(vg);
                }
            }
            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Status row above the display
    // ------------------------------------------------------------
    struct CAStatusWidget : TransparentWidget
    {
        CellAutoGate *module = nullptr;

        CAStatusWidget(CellAutoGate *m) : module(m)
        {
            box.pos = Vec(DISP_X, CA_STATUS_Y - 6.f);
            box.size = Vec(DISP_W, 12.f);
        }

        // Layer-1 overlay: draws the status line (rule number, clock source,
        // and poly channel count) above the display, or demo values in the
        // module browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer == 1)
            {
                NVGcontext *vg = args.vg;
                int rule = module ? module->lastRule : 30;
                char status[96];
                if (module)
                {
                    if (module->clockActive)
                        snprintf(status, sizeof(status), "RULE %03d | EXT CLOCK | %d CH",
                                 rule, module->polyChannels);
                    else
                        snprintf(status, sizeof(status), "RULE %03d | %.0f HZ | %d CH",
                                 rule, module->params[CellAutoGate::SPEED_PARAM].getValue(),
                                 module->polyChannels);
                }
                else
                    snprintf(status, sizeof(status), "RULE %03d | 10 HZ | 1 CH", rule);

                drawText(vg, DISP_W / 2.f, 6.f, status, 8.f, colAccent(), ALIGN_C, 1.5f);
            }
            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Rule table under the display: 8 neighbourhoods -> result
    // ------------------------------------------------------------
    struct CARuleTableWidget : TransparentWidget
    {
        CellAutoGate *module = nullptr;

        CARuleTableWidget(CellAutoGate *m) : module(m)
        {
            box.pos = Vec(DISP_X, CA_GLYPH_Y);
            box.size = Vec(DISP_W, 16.f);
        }

        // Layer-1 overlay: draws the truth table for the current rule as 8
        // groups of three neighbourhood cells (filled = 1) above a result
        // cell (filled cyan = 1). Falls back to rule 30 in the browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer == 1)
            {
                NVGcontext *vg = args.vg;
                const float slotW = DISP_W / 8.f;
                const float cs = 6.f;
                for (int k = 0; k < 8; ++k)
                {
                    int pattern = 7 - k; // 111 ... 000
                    int res = module ? module->ruleset[pattern] : ((30 >> pattern) & 1);
                    float cx = k * slotW + slotW / 2.f;

                    for (int j = 0; j < 3; ++j)
                    {
                        bool on = (pattern >> (2 - j)) & 1;
                        float x = cx - 1.5f * cs - 1.f + j * (cs + 1.f);
                        nvgBeginPath(vg);
                        nvgRect(vg, x + 0.5f, 0.5f, cs - 1.f, cs - 1.f);
                        if (on)
                        {
                            nvgFillColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 230));
                            nvgFill(vg);
                        }
                        else
                        {
                            nvgStrokeColor(vg, nvgRGBA(0x8a, 0x95, 0xa4, 200));
                            nvgStrokeWidth(vg, 1.f);
                            nvgStroke(vg);
                        }
                    }
                    nvgBeginPath(vg);
                    nvgRect(vg, cx - cs / 2.f + 0.5f, cs + 4.5f, cs - 1.f, cs - 1.f);
                    if (res)
                    {
                        nvgFillColor(vg, colAccent());
                        nvgFill(vg);
                    }
                    else
                    {
                        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 140));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }
                }
            }
            Widget::drawLayer(args, layer);
        }
    };

    // Custom cyan light (Rack has no built-in CyanLight)
    // Cyan LED widget, templated on the base light class.
    template <typename TBase = GrayModuleLightWidget>
    struct TCaCyanLight : TBase
    {
        // Registers the cyan base colour with the underlying light widget.
        TCaCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
    };
    using CaCyanLight = TCaCyanLight<>;

} // namespace

// =============================================================================
// Module widget: adds the background, screws, the three display widgets,
// the left/right column controls and the 16 gate LEDs.
// =============================================================================
struct CellAutoGateWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, the history display,
    // status row, rule table, all controls and the gate LED row.
    CellAutoGateWidget(CellAutoGate *module)
    {
        setModule(module);
        box.size = Vec(PANEL_W, PANEL_H);

        addChild(new CAPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new CellularDisplayWidget(module));
        addChild(new CAStatusWidget(module));
        addChild(new CARuleTableWidget(module));

        // Left column
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(CA_LEFT_X, CA_RULE_TOP + CA_KNOB_DY), module, CellAutoGate::RULE_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(CA_LEFT_X, CA_SPEED_TOP + CA_KNOB_DY), module, CellAutoGate::SPEED_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(CA_LEFT_X, CA_POLY_TOP + CA_KNOB_DY), module, CellAutoGate::POLY_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(CA_LEFT_X, CA_CLOCK_TOP + CA_JACK_DY), module, CellAutoGate::CLOCK_INPUT));

        // Right column
        addParam(createParamCentered<VCVButton>(
            Vec(CA_RIGHT_X, CA_RAND_TOP + CA_JACK_DY), module, CellAutoGate::RANDOM_PARAM));
        addParam(createParamCentered<VCVButton>(
            Vec(CA_RIGHT_X, CA_RESET_TOP + CA_JACK_DY), module, CellAutoGate::RESET_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(CA_RIGHT_X, CA_GATES_TOP + CA_JACK_DY), module, CellAutoGate::GATE_OUTPUT));

        // Gate LEDs under the display
        float ledStartX = DISP_X + 8.f;
        float ledEndX = DISP_X + DISP_W - 8.f;
        float spacing = (ledEndX - ledStartX) / 15.f;
        for (int i = 0; i < 16; ++i)
            addChild(createLightCentered<SmallLight<CaCyanLight>>(
                Vec(ledStartX + i * spacing, CA_LED_Y), module, CellAutoGate::GATE_LIGHT_1 + i));
    }
};

Model *modelCellAutoGate = createModel<CellAutoGate, CellAutoGateWidget>("CellAutoGate");