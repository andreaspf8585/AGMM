#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ============================================================================
// Range
// ============================================================================
//
// OVERVIEW
// --------
// Range is a 9 HP utility module that linearly rescales a control-voltage
// signal. It takes a bipolar input nominally in [-5 V, +5 V] and maps it onto
// an arbitrary output window [LOW, HIGH], both of which are user-adjustable
// via front-panel knobs (each spanning -10 V .. +10 V). The transfer is
// purely linear:
//
//   out = ((in + 5) / 10) * (HIGH - LOW) + LOW
//
// so an input of -5 V produces LOW, +5 V produces HIGH, and 0 V produces the
// midpoint. This makes the module useful for converting a bipolar LFO or
// envelope into a unipolar/attenuated/offset control signal.
//
// The display shows the live transfer curve, the current output range
// markers, and a moving dot for the actual input/output pair, along with a
// readout of the LOW/HIGH voltages underneath.
//
// File structure:
//   1. Layout constants (9 HP = 135x380 px).
//   2. Range module: parameter config, and a simple linear process().
//   3. Anonymous-namespace UI: RGPanelBackground (static art) and
//      RGDisplayWidget (transfer plot + live dot + readout).
//   4. RangeWidget: places screws, display, two knobs and two jacks.
//   5. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Layout constants (9 HP = 135 px wide)
//   display 96 x 64 at (19.5,34); readout row under it; two labelled knob cells;
//   an I/O row of two 54 x 50 cells
// ------------------------------------------------------------
static const float RG_PANEL_W = RACK_GRID_WIDTH * 9; // 135
static const float RG_PANEL_H = RACK_GRID_HEIGHT;    // 380

static const float RG_DISP_W = 96.f;
static const float RG_DISP_H = 64.f;
static const float RG_DISP_X = (RG_PANEL_W - RG_DISP_W) / 2.f;
static const float RG_DISP_Y = 34.f;

static const float RG_CX = RG_PANEL_W / 2.f;
static const float RG_SCALE_SEC_Y = 128.f;
static const float RG_KNOB_TOP[2] = {138.f, 208.f}; // LOW, HIGH cells
static const float RG_KNOB_CELL_W = 99.f, RG_KNOB_CELL_H = 62.f;
// Returns the knob centre y for cell i (0 = LOW, 1 = HIGH), 36 px below the
// cell's top edge.
static float rgKnobY(int i) { return RG_KNOB_TOP[i] + 36.f; }

static const float RG_IO_SEC_Y = 284.f;
static const float RG_IO_TOP = 294.f;
static const float RG_IO_CELL_W = 54.f, RG_IO_CELL_H = 50.f;
static const float RG_PORT_X[2] = {37.5f, 97.5f};
// Returns the jack centre y for the I/O row, 32 px below the cell's top edge.
static float rgPortY() { return RG_IO_TOP + 32.f; }

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// Linear CV range mapper: maps a [-5 V, +5 V] input onto a configurable
// [LOW, HIGH] output window.
struct Range : Module
{
    enum ParamId
    {
        LOW_PARAM,
        HIGH_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        SIGNAL_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        SIGNAL_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // For the display (written by the audio thread, read by the UI thread)
    float lastIn = 0.f;
    float lastOut = 0.f;
    bool active = false;

    // Constructor: configures the LOW and HIGH knobs (both spanning
    // -10 V..+10 V with sensible defaults), the signal input/output and the
    // input->output bypass.
    Range()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(LOW_PARAM, -10.f, 10.f, 0.f, "Low", " V");
        configParam(HIGH_PARAM, -10.f, 10.f, 10.f, "High", " V");
        configInput(SIGNAL_INPUT, "Signal");
        configOutput(SIGNAL_OUTPUT, "Output");
        configBypass(SIGNAL_INPUT, SIGNAL_OUTPUT);
    }

    // Main audio callback: reads the input voltage and the LOW/HIGH knobs,
    // applies the linear mapping out = ((in+5)/10)*(HIGH-LOW)+LOW, writes the
    // output, and stores the input/output pair for the display. When nothing
    // is patched the output is 0 V and the module is marked inactive.
    void process(const ProcessArgs &args) override
    {
        if (!inputs[SIGNAL_INPUT].isConnected())
        {
            outputs[SIGNAL_OUTPUT].setVoltage(0.f);
            active = false;
            return;
        }

        float in = inputs[SIGNAL_INPUT].getVoltage();
        float low = params[LOW_PARAM].getValue();
        float high = params[HIGH_PARAM].getValue();

        // Map input [-5V, +5V] to output [low, high]
        float out = ((in + 5.f) / 10.f) * (high - low) + low;

        outputs[SIGNAL_OUTPUT].setVoltage(out);

        lastIn = in;
        lastOut = out;
        active = true;
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{
    namespace ag = agmm;

    // ------------------------------------------------------------
    // Panel background: title, display frame, section headers, labelled plates
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the "SCALE" section with its
    // LOW and HIGH knob cells, and the "I/O" section with its IN and OUT
    // (tinted) jack cells.
    struct RGPanelBackground : TransparentWidget
    {
        RGPanelBackground() { box.size = Vec(RG_PANEL_W, RG_PANEL_H); }

        // Draws all static panel art: base panel, bezel, section headers, the
        // two knob cells and the two I/O cells.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, RG_PANEL_W, RG_PANEL_H, "RANGE");
            ag::bezel(vg, RG_DISP_X, RG_DISP_Y, RG_DISP_W, RG_DISP_H);

            ag::sectionHeader(vg, RG_PANEL_W, RG_SCALE_SEC_Y, "SCALE", 8.f);
            ag::cell(vg, RG_CX, RG_KNOB_TOP[0], RG_KNOB_CELL_W, RG_KNOB_CELL_H, "LOW");
            ag::cell(vg, RG_CX, RG_KNOB_TOP[1], RG_KNOB_CELL_W, RG_KNOB_CELL_H, "HIGH");

            ag::sectionHeader(vg, RG_PANEL_W, RG_IO_SEC_Y, "I / O", 8.f);
            ag::cell(vg, RG_PORT_X[0], RG_IO_TOP, RG_IO_CELL_W, RG_IO_CELL_H, "IN");
            ag::cell(vg, RG_PORT_X[1], RG_IO_TOP, RG_IO_CELL_W, RG_IO_CELL_H, "OUT", true);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Transfer-function display
    // ------------------------------------------------------------
    // Live display: draws the linear transfer curve, the LOW/HIGH range
    // markers, a moving dot for the current input/output, and a low/high
    // readout underneath the screen.
    struct RGDisplayWidget : TransparentWidget
    {
        Range *module = nullptr;

        // Constructor: positions the widget under the display bezel.
        RGDisplayWidget(Range *m) : module(m)
        {
            box.pos = Vec(RG_DISP_X, RG_DISP_Y);
            box.size = Vec(RG_DISP_W, RG_DISP_H);
        }

        // Voltage (-10..+10) to display coordinates
        // Maps a voltage to the x pixel inside the display (linear, -10 V at
        // the left, +10 V at the right).
        static float px(float v) { return (v + 10.f) / 20.f * RG_DISP_W; }
        // Maps a voltage to the y pixel inside the display (linear, -10 V at
        // the bottom, +10 V at the top).
        static float py(float v) { return RG_DISP_H - (v + 10.f) / 20.f * RG_DISP_H; }

        // Static layer: fills the screen, shades the nominal -5..+5 V input
        // window, draws a 5 V grid, the zero axes and the outer frame.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, RG_DISP_W, RG_DISP_H);

            // Nominal input window (-5V..+5V)
            nvgBeginPath(vg);
            nvgRect(vg, px(-5.f), 0, px(5.f) - px(-5.f), RG_DISP_H);
            nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 10));
            nvgFill(vg);

            // Grid every 5V
            nvgBeginPath(vg);
            for (int v = -10; v <= 10; v += 5)
            {
                nvgMoveTo(vg, px((float)v) + 0.5f, 0);
                nvgLineTo(vg, px((float)v) + 0.5f, RG_DISP_H);
                nvgMoveTo(vg, 0, py((float)v) + 0.5f);
                nvgLineTo(vg, RG_DISP_W, py((float)v) + 0.5f);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Zero axes
            nvgBeginPath(vg);
            nvgMoveTo(vg, px(0.f) + 0.5f, 0);
            nvgLineTo(vg, px(0.f) + 0.5f, RG_DISP_H);
            nvgMoveTo(vg, 0, py(0.f) + 0.5f);
            nvgLineTo(vg, RG_DISP_W, py(0.f) + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStroke(vg);

            // frame on top of the grid
            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, RG_DISP_W - 1.f, RG_DISP_H - 1.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the live transfer line, the LOW/HIGH output
        // range markers, a moving dot + crosshair for the current input/output
        // pair, and the "low / high" readout under the display. Uses demo
        // values when there is no module (module browser).
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            float low = module ? module->params[Range::LOW_PARAM].getValue() : 0.f;
            float high = module ? module->params[Range::HIGH_PARAM].getValue() : 10.f;
            float span = high - low;

            nvgSave(vg);
            nvgScissor(vg, 0, 0, RG_DISP_W, RG_DISP_H);

            // Transfer curve: out = ((in + 5) / 10) * (high - low) + low
            float x0 = -10.f, x1 = 10.f;
            float y0 = ((x0 + 5.f) / 10.f) * span + low;
            float y1 = ((x1 + 5.f) / 10.f) * span + low;

            nvgBeginPath(vg);
            nvgMoveTo(vg, px(x0), py(y0));
            nvgLineTo(vg, px(x1), py(y1));
            nvgLineCap(vg, NVG_ROUND);
            nvgStrokeColor(vg, ag::bright());
            nvgStrokeWidth(vg, 1.4f);
            nvgStroke(vg);

            // Output range markers (low / high)
            for (int i = 0; i < 2; ++i)
            {
                float v = (i == 0) ? low : high;
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, py(v));
                nvgLineTo(vg, 5.f, py(v));
                nvgMoveTo(vg, RG_DISP_W - 5.f, py(v));
                nvgLineTo(vg, RG_DISP_W, py(v));
                nvgStrokeColor(vg, ag::withA(ag::violet(), 0.8f));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }

            // Live signal marker
            bool active = module ? module->active : true;
            float in = module ? module->lastIn : 2.f;
            float out = module ? module->lastOut : ((2.f + 5.f) / 10.f) * span + low;
            if (active)
            {
                float cx = px(rack::math::clamp(in, -10.f, 10.f));
                float cy = py(rack::math::clamp(out, -10.f, 10.f));

                nvgBeginPath(vg);
                nvgMoveTo(vg, cx, cy);
                nvgLineTo(vg, cx, RG_DISP_H);
                nvgMoveTo(vg, cx, cy);
                nvgLineTo(vg, 0, cy);
                nvgStrokeColor(vg, ag::withA(ag::violet(), 0.35f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, 2.6f);
                nvgFillColor(vg, nvgRGB(0xf0, 0xe0, 0xff));
                nvgFill(vg);
                nvgStrokeColor(vg, ag::violet());
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            nvgRestore(vg);

            // Readout in its own row under the display
            char buf[32];
            snprintf(buf, sizeof(buf), "%+.1f / %+.1f V", low, high);
            ag::text(vg, RG_DISP_W / 2.f, RG_DISP_H + 14.f, buf, 7.f, ag::accent(), ag::ALIGN_C, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget: places the static panel, screws, the live transfer display,
// the two knobs (LOW/HIGH) and the input/output jacks.
// ------------------------------------------------------------
struct RangeWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, display, the two
    // knobs and the two I/O jacks.
    RangeWidget(Range *module)
    {
        setModule(module);
        box.size = Vec(RG_PANEL_W, RG_PANEL_H);

        addChild(new RGPanelBackground());

        // Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new RGDisplayWidget(module));

        // Knobs: control sits 36 px below its cell top
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(RG_CX, rgKnobY(0)), module, Range::LOW_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(RG_CX, rgKnobY(1)), module, Range::HIGH_PARAM));

        // Ports
        addInput(createInputCentered<DarkPJ301MPort>(Vec(RG_PORT_X[0], rgPortY()), module, Range::SIGNAL_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(RG_PORT_X[1], rgPortY()), module, Range::SIGNAL_OUTPUT));
    }
};

Model *modelRange = createModel<Range, RangeWidget>("Range");