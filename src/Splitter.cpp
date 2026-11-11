#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

// ============================================================================
// Splitter
// ============================================================================
//
// OVERVIEW
// --------
// Splitter is a small 12 HP utility that expands a single polyphonic input
// into sixteen monophonic outputs. Every output jack carries one channel of
// the incoming polyphonic signal (or 0 V if that channel isn't present), so
// the module can be used to break a polyphonic patch apart into individual
// voices, for example to route each channel through its own effect.
//
// The 132 x 56 display shows the current channel count as a large number plus
// a 16-segment bar whose lit segments indicate which channels are active.
//
// File structure:
//   1. Layout constants (12 HP = 180x380 px).
//   2. Splitter module: config, and a simple process() that copies each input
//      channel to its matching monophonic output.
//   3. Anonymous-namespace UI: SLPanelBackground (static artwork) and
//      SLDisplayWidget (channel count + segment bar).
//   4. SplitterWidget: places screws, the display, the poly input and the 16
//      output jacks.
//   5. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float SL_PANEL_W = RACK_GRID_WIDTH * 12;   // 180
static const float SL_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Channel display (same as Merger)
static const float SL_DISP_W = 132.f;
static const float SL_DISP_H = 56.f;
static const float SL_DISP_X = (SL_PANEL_W - SL_DISP_W) / 2.f;   // 24
static const float SL_DISP_Y = 28.f;

// Polyphonic input cell
static const float SL_SECTION_IN_Y = 98.f;
static const float SL_IN_X = SL_PANEL_W / 2.f;
static const float SL_IN_TOP = 106.f, SL_IN_W = 70.f, SL_IN_H = 44.f;

// Output grid (4 x 4 cells of 36 x 40, label at +10, jack at +27)
static const float SL_SECTION_OUT_Y = 164.f;
static const float SL_GRID_X[4] = {30.f, 70.f, 110.f, 150.f};
static const float SL_ROW_TOP[4] = {172.f, 215.f, 258.f, 301.f};
static const float SL_CELL_W = 36.f, SL_CELL_H = 40.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// Polyphonic-to-mono splitter: reads a polyphonic input and exposes up to 16
// monophonic outputs, one per input channel.
struct Splitter : Module {
    enum ParamId {
        PARAMS_LEN
    };
    enum InputId {
        POLY_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        ENUMS(SPLIT_OUTPUT, 16),
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    // Shown on the display
    int channels = 0;

    // Constructor: configures the single polyphonic input and the 16
    // monophonic outputs (each labelled "Channel N").
    Splitter() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configInput(POLY_INPUT, "Polyphonic");
        for (int i = 0; i < 16; ++i)
            configOutput(SPLIT_OUTPUT + i, string::f("Channel %d", i + 1));
    }

    // Main audio callback: reads the channel count of the poly input, then
    // writes channel i to output i (or 0 V when the input has fewer
    // channels). Each output is forced to mono and the count is saved for the
    // display.
    void process(const ProcessArgs& args) override {
        int ch = inputs[POLY_INPUT].isConnected() ? inputs[POLY_INPUT].getChannels() : 0;
        channels = ch;

        for (int i = 0; i < 16; ++i) {
            outputs[SPLIT_OUTPUT + i].setChannels(1);
            outputs[SPLIT_OUTPUT + i].setVoltage(i < ch ? inputs[POLY_INPUT].getVoltage(i) : 0.f);
        }
    }
};

// ------------------------------------------------------------
// UI (anonymous namespace: names cannot clash with other module files)
// ------------------------------------------------------------
namespace {

// Static artwork: base panel, display bezel, the INPUT section with its
// single poly-in cell, and the OUTPUTS section with a 4 x 4 grid of numbered
// jack cells (outputs tinted).
struct SLPanelBackground : TransparentWidget {
    SLPanelBackground() { box.size = Vec(SL_PANEL_W, SL_PANEL_H); }

    // Draws the base panel, display bezel, the INPUT section header and the
    // "POLY IN" cell, then the OUTPUTS section header and the 16 numbered
    // cells (each tinted as an output).
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        agmm::panelBase(vg, SL_PANEL_W, SL_PANEL_H, "SPLITTER");
        agmm::bezel(vg, SL_DISP_X, SL_DISP_Y, SL_DISP_W, SL_DISP_H);

        // ---- input ----
        agmm::sectionHeader(vg, SL_PANEL_W, SL_SECTION_IN_Y, "INPUT", 12.f);
        agmm::cell(vg, SL_IN_X, SL_IN_TOP, SL_IN_W, SL_IN_H, "POLY IN", false, 7.f);

        // ---- outputs ----
        agmm::sectionHeader(vg, SL_PANEL_W, SL_SECTION_OUT_Y, "OUTPUTS", 12.f);
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", r * 4 + c + 1);
                agmm::cell(vg, SL_GRID_X[c], SL_ROW_TOP[r], SL_CELL_W, SL_CELL_H, buf, true, 7.f);
            }
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Channel count display
// ------------------------------------------------------------
// Live display showing the current poly channel count as a large number plus
// a 16-segment bar (lit segments = active channels). Uses a demo value of 8
// in the module browser.
struct SLDisplayWidget : TransparentWidget {
    Splitter* module = nullptr;

    // Constructor: positions the display widget under its bezel.
    SLDisplayWidget(Splitter* m) : module(m) {
        box.pos = Vec(SL_DISP_X, SL_DISP_Y);
        box.size = Vec(SL_DISP_W, SL_DISP_H);
    }

    // Static layer: fills the screen and draws the divider between the
    // number row and the segment row.
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        agmm::screen(vg, 0.f, 0.f, SL_DISP_W, SL_DISP_H);

        // Divider between the number and the segment row
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0.f, 41.5f);
        nvgLineTo(vg, SL_DISP_W, 41.5f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        Widget::draw(args);
    }

    // Layer-1 overlay: draws the "CHANNELS" caption, the large channel count
    // (dimmed to 0 when no input is connected) and the 16-segment activity
    // bar.
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        // State (module is null in the module browser: show a demo)
        int ch = module ? module->channels : 8;
        ch = std::min(std::max(ch, 0), 16);

        // Caption row
        agmm::text(vg, SL_DISP_W / 2.f, 8.f, "CHANNELS", 7.f, agmm::dim(), agmm::ALIGN_C, 2.f);

        // Big number
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", ch);
        agmm::text(vg, SL_DISP_W / 2.f, 26.f, buf, 24.f,
                   ch > 0 ? agmm::bright() : agmm::withA(agmm::dim(), 0.6f), agmm::ALIGN_C, 1.f);

        // 16-segment bar: bright = channel present in the input, faint = unused
        const float pitch = 7.5f, segW = 5.5f, segH = 5.f;
        const float x0 = (SL_DISP_W - (16.f * pitch - (pitch - segW))) / 2.f;
        const float y0 = 46.f;
        for (int i = 0; i < 16; ++i) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x0 + i * pitch, y0, segW, segH, 1.f);
            nvgFillColor(vg, i < ch ? agmm::accent() : nvgRGBA(255, 255, 255, 14));
            nvgFill(vg);
        }

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
// Builds the widget tree: static panel, screws, live display, the polyphonic
// input jack and the 4 x 4 grid of monophonic output jacks.
struct SplitterWidget : ModuleWidget {
    // Constructor: places the panel background, the four silver screws, the
    // channel-count display, the polyphonic input and the 16 outputs (row by
    // row, left to right).
    SplitterWidget(Splitter* module) {
        setModule(module);
        box.size = Vec(SL_PANEL_W, SL_PANEL_H);

        addChild(new SLPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SLDisplayWidget(module));

        // Polyphonic input (jack at +29 inside its cell)
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(SL_IN_X, SL_IN_TOP + 29.f), module, Splitter::POLY_INPUT));

        // 16 outputs in a 4 x 4 grid (row by row, jack at +27 inside its cell)
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                addOutput(createOutputCentered<DarkPJ301MPort>(
                    Vec(SL_GRID_X[c], SL_ROW_TOP[r] + 27.f), module,
                    Splitter::SPLIT_OUTPUT + r * 4 + c));
            }
        }
    }
};

Model* modelSplitter = createModel<Splitter, SplitterWidget>("Splitter");