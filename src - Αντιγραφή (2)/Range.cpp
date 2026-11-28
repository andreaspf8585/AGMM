#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include "Knob_arcs.hpp"

// ============================================================================
// Range
// ============================================================================
//
// OVERVIEW
// --------
// Range is a 9 HP utility module that linearly rescales a control-voltage
// signal. It maps the input window [LOW IN, HIGH IN] onto an output window
// [LOW, HIGH], where all four endpoints are user-adjustable via front-panel
// knobs (each spanning -10 V .. +10 V). The transfer is purely linear:
//
//   out = ((in - LOW_IN) / (HIGH_IN - LOW_IN)) * (HIGH - LOW) + LOW
//
// so an input of LOW_IN produces LOW, HIGH_IN produces HIGH, and the middle
// of the input window produces the midpoint. This makes the module useful for
// converting a bipolar LFO or envelope into a unipolar / attenuated / offset
// control signal.
//
// INPUT RANGE
// -----------
// The two left-hand knobs (LOW IN / HIGH IN) define the input window the
// module normalises from. They are the authoritative source of the input
// range and always take effect; the right-click "Input range presets" submenu
// is a convenience that snaps those two knobs to common values
// (-10..+10 V, -5..+5 V, 0..10 V, 0..5 V, -1..+1 V, 0..1 V). After picking a
// preset the user can still fine-tune the knobs. If LOW IN > HIGH IN the
// transfer is inverted (useful for inverting CV). The transfer is undefined
// if LOW IN == HIGH IN; a tiny epsilon guards against division by zero.
//
// "Clamp output to LOW..HIGH" (default on) keeps the result inside the output
// window even if the input overshoots its range. This setting is saved with
// the patch; the four knob positions are saved automatically by Rack.
//
// The display shows the live transfer curve, the current output range
// markers, and a moving dot for the actual input/output pair, along with a
// readout of the LOW/HIGH output voltages underneath and the current input
// window in the top-left corner.
//
// POLYPHONY
// ---------
// The module is polyphonic: it processes as many channels as the input cable
// carries (1..16) and the output cable carries the same number of channels.
// All four range endpoints are shared by all channels. The display shows
// channel 1 with a crosshair, the remaining channels as small dots on the
// same transfer line, and a "N CH" tag when more than one channel is present.
// Channels are processed four at a time with SIMD.
//
// File structure:
//   1. Layout constants (9 HP = 135x380 px).
//   2. Range module: parameter config, and a simple linear process().
//   3. Anonymous-namespace UI: RGPanelBackground (static art) and
//      RGDisplayWidget (transfer plot + live dot + readout).
//   4. RangeWidget: places screws, display, four knobs and two jacks.
//   5. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Layout constants (9 HP = 135 px wide)
//   display 96 x 64 at (19.5,34); readout row under it;
//   2x2 grid of labelled knob cells (left column = IN, right column = OUT);
//   an I/O row of two 54 x 50 cells.
// ------------------------------------------------------------
static const float RG_PANEL_W = RACK_GRID_WIDTH * 9; // 135
static const float RG_PANEL_H = RACK_GRID_HEIGHT;    // 380

static const float RG_DISP_W = 96.f;
static const float RG_DISP_H = 64.f;
static const float RG_DISP_X = (RG_PANEL_W - RG_DISP_W) / 2.f;
static const float RG_DISP_Y = 34.f;

static const float RG_SCALE_SEC_Y = 128.f;
static const float RG_KNOB_TOP[2] = {138.f, 208.f};      // row tops
static const float RG_KNOB_CELL_W = 55.f, RG_KNOB_CELL_H = 62.f;
static const float RG_KNOB_COL_X[2] = {37.5f, 97.5f};    // left col, right col
// Returns the knob centre y for row i (0 = top, 1 = bottom), 36 px below the
// row's top edge.
static float rgKnobY(int row) { return RG_KNOB_TOP[row] + 36.f; }

static const float RG_IO_SEC_Y = 284.f;
static const float RG_IO_TOP = 294.f;
static const float RG_IO_CELL_W = 54.f, RG_IO_CELL_H = 50.f;
static const float RG_PORT_X[2] = {37.5f, 97.5f};
// Returns the jack centre y for the I/O row, 32 px below the cell's top edge.
static float rgPortY() { return RG_IO_TOP + 32.f; }

// ------------------------------------------------------------
// Input range presets (selectable from the context menu)
// ------------------------------------------------------------
struct RgRange
{
    float lo, hi;
    const char *name;
};
static const int RG_RANGE_COUNT = 6;
static const RgRange RG_RANGES[RG_RANGE_COUNT] = {
    {-10.f, 10.f, "-10 .. +10 V"},
    {-5.f, 5.f, "-5 .. +5 V"},
    {0.f, 10.f, "0 .. 10 V"},
    {0.f, 5.f, "0 .. 5 V"},
    {-1.f, 1.f, "-1 .. +1 V"},
    {0.f, 1.f, "0 .. 1 V"},
};

// Transfer function shared by the audio code and the display.
// Maps `in` through the input window [inLo, inHi] onto [low, high]. A tiny
// epsilon guards against the degenerate inLo == inHi case. If clampOut is
// true the result is clamped into [min(low,high), max(low,high)].
static float rgMap(float in, float low, float high, float inLo, float inHi, bool clampOut)
{
    float denom = inHi - inLo;
    if (std::fabs(denom) < 1e-4f)
        denom = (denom >= 0.f ? 1e-4f : -1e-4f);
    float out = (in - inLo) / denom * (high - low) + low;
    if (clampOut)
        out = rack::math::clamp(out, std::min(low, high), std::max(low, high));
    return out;
}

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// Linear CV range mapper: maps an arbitrary [LOW IN, HIGH IN] input window
// onto a configurable [LOW, HIGH] output window.
struct Range : Module
{
    enum ParamId
    {
        LOW_IN_PARAM,   // input window low  (left col, top)
        HIGH_IN_PARAM,  // input window high (left col, bottom)
        LOW_PARAM,      // output window low (right col, top)
        HIGH_PARAM,     // output window high (right col, bottom)
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

    // For the display (written by the audio thread, read by the UI thread).
    // Index = polyphonic channel; `channels` is how many of them are live.
    float lastIn[16] = {};
    float lastOut[16] = {};
    int channels = 0;
    bool active = false;

    // Settings (context menu, saved with the patch). The four knob values are
    // auto-saved by Rack, so we only need to persist the clamp flag.
    bool clampOut = true;

    // Convenience accessors for the current input window (from the knobs).
    float inLo() { return params[LOW_IN_PARAM].getValue(); }
    float inHi() { return params[HIGH_IN_PARAM].getValue(); }

    // Constructor: configures the four range knobs (all spanning -10 V..+10 V
    // with sensible defaults), the signal input/output and the input->output
    // bypass.
    Range()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(LOW_IN_PARAM,  -10.f, 10.f, -10.f, "Low input",  " V");
        configParam(HIGH_IN_PARAM, -10.f, 10.f,  10.f, "High input", " V");
        configParam(LOW_PARAM,     -10.f, 10.f,   0.f, "Low output", " V");
        configParam(HIGH_PARAM,    -10.f, 10.f,  10.f, "High output"," V");
        configInput(SIGNAL_INPUT, "Signal");
        configOutput(SIGNAL_OUTPUT, "Output");
        configBypass(SIGNAL_INPUT, SIGNAL_OUTPUT);
    }

    // Main audio callback: reads every input channel (up to 16) and the four
    // range knobs, applies the linear mapping
    //   out = ((in - LOW_IN) / (HIGH_IN - LOW_IN)) * (HIGH - LOW) + LOW
    // to each channel (four at a time), writes the same number of output
    // channels, and stores the input/output pairs for the display. When
    // nothing is patched the output is a single 0 V channel and the module is
    // marked inactive.
    void process(const ProcessArgs &args) override
    {
        if (!inputs[SIGNAL_INPUT].isConnected())
        {
            outputs[SIGNAL_OUTPUT].setChannels(1);
            outputs[SIGNAL_OUTPUT].setVoltage(0.f);
            active = false;
            channels = 0;
            return;
        }

        const int n = std::max(1, std::min(inputs[SIGNAL_INPUT].getChannels(), 16));
        const float low   = params[LOW_PARAM].getValue();
        const float high  = params[HIGH_PARAM].getValue();
        const float inLo  = params[LOW_IN_PARAM].getValue();
        const float inHi  = params[HIGH_IN_PARAM].getValue();

        // Guard against LOW IN == HIGH IN (degenerate window).
        float denom = inHi - inLo;
        if (std::fabs(denom) < 1e-4f)
            denom = (denom >= 0.f ? 1e-4f : -1e-4f);
        const float scale = (high - low) / denom;
        const float wLo = std::min(low, high), wHi = std::max(low, high);

        outputs[SIGNAL_OUTPUT].setChannels(n);

        // Map the input range [LOW IN, HIGH IN] to output [low, high],
        // 4 channels per step. The arrays hold 16 entries, so the last block
        // (c = 12) stays in range.
        for (int c = 0; c < n; c += 4)
        {
            simd::float_4 in = inputs[SIGNAL_INPUT].getVoltageSimd<simd::float_4>(c);
            simd::float_4 out = (in - inLo) * scale + low;
            if (clampOut)
                out = simd::clamp(out, wLo, wHi);
            outputs[SIGNAL_OUTPUT].setVoltageSimd(out, c);
            in.store(&lastIn[c]);
            out.store(&lastOut[c]);
        }

        channels = n;
        active = true;
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "clampOut", json_boolean(clampOut));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        if (json_t *j = json_object_get(root, "clampOut"))
            clampOut = json_boolean_value(j);

        // Legacy migration: older patches stored an integer index into
        // RG_RANGES under "inRange". If present, snap the new LOW IN / HIGH IN
        // knobs to that preset (Rack loads params before dataFromJson, so this
        // override is safe and produces the same transfer as before).
        if (json_t *j = json_object_get(root, "inRange"))
        {
            int idx = rack::math::clamp((int)json_integer_value(j), 0, RG_RANGE_COUNT - 1);
            params[LOW_IN_PARAM].setValue(RG_RANGES[idx].lo);
            params[HIGH_IN_PARAM].setValue(RG_RANGES[idx].hi);
        }
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
    // 2x2 knob grid (left column = LOW IN / HIGH IN, right column = LOW / HIGH)
    // and the "I/O" section with its IN and OUT (tinted) jack cells.
    struct RGPanelBackground : TransparentWidget
    {
        RGPanelBackground() { box.size = Vec(RG_PANEL_W, RG_PANEL_H); }

        // Draws all static panel art.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, RG_PANEL_W, RG_PANEL_H, "RANGE");
            ag::bezel(vg, RG_DISP_X, RG_DISP_Y, RG_DISP_W, RG_DISP_H);

            // SCALE section: 2x2 grid.
            //   col 0 = input window, col 1 = output window
            //   row 0 = LOW, row 1 = HIGH
            ag::sectionHeader(vg, RG_PANEL_W, RG_SCALE_SEC_Y, "SCALE", 8.f);
            ag::cell(vg, RG_KNOB_COL_X[0], RG_KNOB_TOP[0], RG_KNOB_CELL_W, RG_KNOB_CELL_H, "LOW IN");
            ag::cell(vg, RG_KNOB_COL_X[1], RG_KNOB_TOP[0], RG_KNOB_CELL_W, RG_KNOB_CELL_H, "LOW");
            ag::cell(vg, RG_KNOB_COL_X[0], RG_KNOB_TOP[1], RG_KNOB_CELL_W, RG_KNOB_CELL_H, "HIGH IN");
            ag::cell(vg, RG_KNOB_COL_X[1], RG_KNOB_TOP[1], RG_KNOB_CELL_W, RG_KNOB_CELL_H, "HIGH");

            ag::sectionHeader(vg, RG_PANEL_W, RG_IO_SEC_Y, "I / O", 8.f);
            ag::cell(vg, RG_PORT_X[0], RG_IO_TOP, RG_IO_CELL_W, RG_IO_CELL_H, "IN");
            ag::cell(vg, RG_PORT_X[1], RG_IO_TOP, RG_IO_CELL_W, RG_IO_CELL_H, "OUT", true);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Transfer-function display
    // ------------------------------------------------------------
    // Live display: draws the linear transfer curve, the LOW/HIGH output range
    // markers, a moving dot for the current input/output, and a readout
    // underneath. All four endpoints come from the module knobs.
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
        static float px(float v) { return (v + 10.f) / 20.f * RG_DISP_W; }
        static float py(float v) { return RG_DISP_H - (v + 10.f) / 20.f * RG_DISP_H; }

        // Static layer: fills the screen, shades the current input window,
        // draws a 5 V grid, the zero axes and the outer frame.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, RG_DISP_W, RG_DISP_H);

            // Selected input window (sorted so the highlight is always a
            // proper rectangle even if the user inverted the window)
            const float inLo = module ? module->inLo() : -10.f;
            const float inHi = module ? module->inHi() :  10.f;
            const float xLo  = std::min(inLo, inHi);
            const float xHi  = std::max(inLo, inHi);
            nvgBeginPath(vg);
            nvgRect(vg, px(xLo), 0, px(xHi) - px(xLo), RG_DISP_H);
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

        // Layer-1 overlay: live transfer line, output range markers, moving
        // dot + crosshair for channel 1, small dots for the other channels,
        // and the readouts. Uses demo values when there is no module.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            const float low    = module ? module->params[Range::LOW_PARAM].getValue()      :  0.f;
            const float high   = module ? module->params[Range::HIGH_PARAM].getValue()     : 10.f;
            const float inLo   = module ? module->params[Range::LOW_IN_PARAM].getValue()   : -10.f;
            const float inHi   = module ? module->params[Range::HIGH_IN_PARAM].getValue()  :  10.f;
            const bool  clampOut = module ? module->clampOut : true;

            nvgSave(vg);
            nvgScissor(vg, 0, 0, RG_DISP_W, RG_DISP_H);

            // Transfer curve through the display edges and both ends of the
            // input window (flat outside the window when the output is
            // clamped). Sorted x positions keep the path monotonic even if the
            // window is inverted.
            const float xLo = std::min(inLo, inHi);
            const float xHi = std::max(inLo, inHi);
            const float xs[4] = {-10.f, xLo, xHi, 10.f};
            nvgBeginPath(vg);
            for (int i = 0; i < 4; ++i)
            {
                float y = rgMap(xs[i], low, high, inLo, inHi, clampOut);
                if (i == 0)
                    nvgMoveTo(vg, px(xs[i]), py(y));
                else
                    nvgLineTo(vg, px(xs[i]), py(y));
            }
            nvgLineJoin(vg, NVG_ROUND);
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

            // Live signal markers
            bool active = module ? module->active : true;
            const int nCh = module ? std::max(1, module->channels) : 1;
            if (active)
            {
                // Channels 2..N: small dots on the transfer line
                for (int c = nCh - 1; c >= 1; --c)
                {
                    float dx = px(rack::math::clamp(module->lastIn[c],  -10.f, 10.f));
                    float dy = py(rack::math::clamp(module->lastOut[c], -10.f, 10.f));
                    nvgBeginPath(vg);
                    nvgCircle(vg, dx, dy, 1.8f);
                    nvgFillColor(vg, ag::withA(ag::violet(), 0.75f));
                    nvgFill(vg);
                }

                // Channel 1: crosshair + bright dot
                float in  = module ? module->lastIn[0]  : 2.f;
                float out = module ? module->lastOut[0] : rgMap(in, low, high, inLo, inHi, clampOut);
                float cx = px(rack::math::clamp(in,  -10.f, 10.f));
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

            // Input window tag (top-left corner of the screen).
            char tag[32];
            snprintf(tag, sizeof(tag), "IN %+.0f..%+.0f", inLo, inHi);
            ag::text(vg, 4.f, 8.f, tag, 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            // Channel count tag
            if (nCh > 1)
            {
                char cb[16];
                snprintf(cb, sizeof(cb), "%d CH", nCh);
                ag::text(vg, 4.f, 17.f, cb, 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            }

            // Output readout in its own row under the display
            char buf[32];
            snprintf(buf, sizeof(buf), "%+.1f / %+.1f V", low, high);
            ag::text(vg, RG_DISP_W / 2.f, RG_DISP_H + 14.f, buf, 7.f, ag::accent(), ag::ALIGN_C, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget: places the static panel, screws, the live transfer display,
// the 2x2 grid of range knobs and the input/output jacks.
// ------------------------------------------------------------
struct RangeWidget : ModuleWidget
{
    // Builds the widget tree.
    RangeWidget(Range *module)
    {
        setModule(module);
        box.size = Vec(RG_PANEL_W, RG_PANEL_H);

        addChild(new RGPanelBackground());

        // Screws
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new RGDisplayWidget(module));

        // Knobs: left column = input window, right column = output window.
        // Controls sit 36 px below their cell top.
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(RG_KNOB_COL_X[0], rgKnobY(0)), module, Range::LOW_IN_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(RG_KNOB_COL_X[0], rgKnobY(1)), module, Range::HIGH_IN_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(RG_KNOB_COL_X[1], rgKnobY(0)), module, Range::LOW_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(RG_KNOB_COL_X[1], rgKnobY(1)), module, Range::HIGH_PARAM));
        agmm::addKnobArcs(this, 2.f);
        // Ports
        addInput(createInputCentered<DarkPJ301MPort>(Vec(RG_PORT_X[0], rgPortY()), module, Range::SIGNAL_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(RG_PORT_X[1], rgPortY()), module, Range::SIGNAL_OUTPUT));
    }

    // Right-click menu:
    //   * "Input range presets" snaps the LOW IN / HIGH IN knobs to a common
    //     window. The knobs remain authoritative and can be tweaked freely
    //     afterwards, so this is purely a convenience shortcut.
    //   * "Clamp output to LOW..HIGH" keeps the output inside the output
    //     window when the input overshoots the input window.
    void appendContextMenu(Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        Range *m = dynamic_cast<Range *>(module);
        if (!m)
            return;
        menu->addChild(new MenuSeparator);
        menu->addChild(createSubmenuItem("Input range presets", "", [m](Menu *sub)
                                         {
            for (int i = 0; i < RG_RANGE_COUNT; ++i)
                sub->addChild(createMenuItem(RG_RANGES[i].name, "", [m, i]()
                                             {
                    m->params[Range::LOW_IN_PARAM].setValue(RG_RANGES[i].lo);
                    m->params[Range::HIGH_IN_PARAM].setValue(RG_RANGES[i].hi); })); }));
        menu->addChild(createCheckMenuItem("Clamp output to LOW..HIGH", "",
                                           [m]() { return m->clampOut; },
                                           [m]() { m->clampOut = !m->clampOut; }));
    }
};

Model *modelRange = createModel<Range, RangeWidget>("Range");