#include "YouTubeLoad.hpp"
#include "GitLoad.hpp"
// ============================================================================
// SwitchLines1to8 — 1-to-8 sequential switch (24 HP Rack module)
// ----------------------------------------------------------------------------
// Overview:
//   * A single SIGNAL input is routed to one of eight outputs (CH_OUTPUT 1..8).
//   * A clocked step sequencer (CLOCK in / STEP button / RESET in) walks a
//     window of active outputs defined by LENGTH (1..8) and START (1..8).
//   * DIR selects Forward / Reverse / Pendulum / Random traversal.
//   * FADE controls a per-output crossfade: the newly selected output fades
//     in while the previously selected one fades out (complementary gains).
//   * One gate output per channel is high while that channel is selected;
//     an EOC trigger pulses at the end of each cycle.
//   * A custom 224x152 display shows the input bar, the routing fan-out,
//     per-output voltage meters, and a 16-cell history strip.
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout constants (24 HP = 360 px wide)
// ------------------------------------------------------------
static const int SW_N = 8;   // number of outputs
static const int SW_HN = 16; // history cells kept for the display

static const float SW_PANEL_W = RACK_GRID_WIDTH * 24; // 360
static const float SW_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (224 x 152): header row, routing diagram, history strip, footer row. Bezel adds 4 px.
static const float SW_DISP_W = 224.f;
static const float SW_DISP_H = 152.f;
static const float SW_DISP_X = (SW_PANEL_W - SW_DISP_W) / 2.f; // 68
static const float SW_DISP_Y = 26.f;

// Plot geometry inside the display (display-local coordinates)
static const float SW_HEAD_LINE = 15.5f; // hairline under the header row
static const float SW_ROW0 = 24.f;       // first output row (y)
static const float SW_ROWP = 10.f;       // row pitch (8 rows -> y 24..94)
static const float SW_BAR_X = 8.f;       // input voltage bar x
static const float SW_BAR_W = 8.f;       // bar width
static const float SW_BAR_HALF = 34.f;   // bar half height (maps ±10 V)
static const float SW_INN_X = 26.f;      // input node x
static const float SW_HUB_X = 54.f;      // central junction x
static const float SW_HUB_R = 8.f;       // junction radius
static const float SW_NODE_X = 162.f;    // output nodes x
static const float SW_MET_CX = 188.f;    // output voltage meters centre x
static const float SW_MET_HW = 16.f;     // meter half width (maps ±10 V)
static const float SW_LBL_X = 216.f;     // output-number label x
static const float SW_IN_LBL_Y = 101.f;  // "IN" caption y under the bar
static const float SW_HIST_X = 8.f;      // history strip origin
static const float SW_HIST_Y = 108.f;
static const float SW_HIST_P = 13.f;      // history cell pitch
static const float SW_HIST_W = 11.f;      // history cell width
static const float SW_HIST_H = 14.f;      // history cell height
static const float SW_FOOT_LINE = 136.5f; // hairline above the footer row
static const float SW_CAP_Y = 144.f;      // footer text centre line

// Row y for output i, and y of the central hub (midway between rows 4 and 5).
static inline float swRowY(int i) { return SW_ROW0 + SW_ROWP * (float)i; }
static inline float swHubY() { return SW_ROW0 + SW_ROWP * 3.5f; }

// Side knob cells: LENGTH, START (left) / DIR, FADE (right). 50 x 52, label at +10, knob at +32
static const float SW_SIDE_W = 50.f, SW_SIDE_H = 52.f, SW_SIDE_DY = 32.f;
static const float SW_SIDE_X[4] = {34.f, 34.f, 326.f, 326.f};
static const float SW_SIDE_TOP[4] = {40.f, 112.f, 40.f, 112.f};
static const float SW_KNOB_X[4] = {34.f, 34.f, 326.f, 326.f};
static const float SW_KNOB_Y[4] = {72.f, 144.f, 72.f, 144.f}; // SIDE_TOP + SIDE_DY

// Control row: five 52 x 48 cells, control at +29
static const float SW_CTRL_HDR = 192.f;
static const float SW_CTRL_TOP = 200.f;
static const float SW_CTRL_W = 52.f, SW_CTRL_H = 48.f;
static const float SW_CTRL_Y = SW_CTRL_TOP + 29.f;
static const float SW_CTRL_X[5] = {60.f, 120.f, 180.f, 240.f, 300.f};

// Channel rows: eight 36 x 42 cells per row, jack at +28
static const float SW_CH_HDR = 258.f;
static const float SW_OUT_TOP = 266.f;
static const float SW_GATE_TOP = 312.f;
static const float SW_CH_W = 36.f, SW_CH_H = 42.f, SW_CH_JACK_DY = 28.f;
static const float SW_CH_X0 = 40.f;
static const float SW_CH_DX = 40.f;

// ------------------------------------------------------------
// Module — DSP + state
// ------------------------------------------------------------
struct SwitchLines1to8 : Module
{
    enum ParamId
    {
        LENGTH_PARAM,
        START_PARAM,
        DIR_PARAM,
        FADE_PARAM,
        STEP_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        CLOCK_INPUT,
        RESET_INPUT,
        SIGNAL_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        EOC_OUTPUT,
        ENUMS(CH_OUTPUT, 8),
        ENUMS(GATE_OUTPUT, 8),
        OUTPUTS_LEN
    };
    enum LightId
    {
        STEP_LIGHT,
        LIGHTS_LEN
    };

    // --------------------------------------------------------
    // Sequencer state
    // --------------------------------------------------------
    int pos = 0;     // position inside the active range [0, L)
    int dirSign = 1; // pendulum direction (+1 / -1)
    int curCh = 0;   // currently selected output
    int prevCh = 0;  // previous selection (fading out)
    // Per-output gain 0..1 used for the crossfade. gain[curCh] tends
    // towards 1 while all other gains tend towards 0.
    float gain[SW_N] = {1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    // --------------------------------------------------------
    // Display data (mirrored for the UI widget; written by process())
    // --------------------------------------------------------
    float inV = 0.f;                    // measured SIGNAL input voltage
    bool inLinked = false;              // true if SIGNAL input is patched
    float outV[SW_N] = {};              // per-output voltages
    bool linked[SW_N] = {};             // true if output i is patched
    float pulse = 2.f;                  // travelling dot progress 0..1 (>1 = idle)
    float clockFlash = 0.f;             // clock LED / flash decay
    float eocFlash = 0.f;               // end-of-cycle flash decay
    int hist[SW_HN] = {};               // ring buffer of recent selections
    int histHead = 0;                   // ring buffer write head
    int histCount = 0;                  // number of valid entries
    uint32_t stepCount = 0;             // total steps taken (shown in the footer)
    int vLen = 8, vStart = 0, vDir = 0; // snapshots for the display
    float vFadeT = 0.f;                 // current fade time snapshot

    dsp::SchmittTrigger clockTrig, stepTrig, resetTrig;
    dsp::PulseGenerator eocPulse;

    // --------------------------------------------------------
    // Constructor — config params, inputs, outputs, light.
    // --------------------------------------------------------
    SwitchLines1to8()
    {
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
        for (int i = 0; i < SW_N; ++i)
        {
            configOutput(CH_OUTPUT + i, string::f("Output %d", i + 1));
            configOutput(GATE_OUTPUT + i, string::f("Gate %d (output %d selected)", i + 1, i + 1));
        }
        configOutput(EOC_OUTPUT, "End of cycle trigger");
    }

    // --------------------------------------------------------
    // Full reset: zero everything, route to output 1.
    // --------------------------------------------------------
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        pos = 0;
        dirSign = 1;
        curCh = prevCh = 0;
        for (int i = 0; i < SW_N; ++i)
            gain[i] = (i == 0) ? 1.f : 0.f;
        pulse = 2.f;
        std::memset(hist, 0, sizeof(hist));
        histHead = histCount = 0;
        stepCount = 0;
    }

    // --------------------------------------------------------
    // Reset the sequencer to the start of the active window and
    // jump the gains to the new selection (no crossfade on reset).
    // --------------------------------------------------------
    void doReset(int S, int L, int dir)
    {
        pos = (dir == 1) ? L - 1 : 0;
        dirSign = 1;
        prevCh = curCh = (S + pos) % SW_N;
        for (int i = 0; i < SW_N; ++i)
            gain[i] = (i == curCh) ? 1.f : 0.f;
        pulse = 2.f;
        stepCount = 0;
    }

    // --------------------------------------------------------
    // Advance the sequencer by one step according to direction.
    // Handles FWD, REV, pendulum (bounces between the ends) and
    // random (uniform choice excluding the current position).
    // Updates curCh/prevCh, pulse, history, EOC. Note that the
    // per-output gain ramps are handled in process().
    // --------------------------------------------------------
    void doStep(int S, int L, int dir)
    {
        bool eoc = false;
        switch (dir)
        {
        case 0: // Forward
            pos++;
            if (pos >= L)
            {
                pos = 0;
                eoc = true;
            }
            break;
        case 1: // Reverse
            pos--;
            if (pos < 0)
            {
                pos = L - 1;
                eoc = true;
            }
            break;
        case 2: // Pendulum (bounce)
            if (L <= 1)
            {
                pos = 0;
                eoc = true;
            }
            else
            {
                pos += dirSign;
                if (pos >= L)
                {
                    pos = L - 2;
                    dirSign = -1;
                }
                else if (pos < 0)
                {
                    pos = 1;
                    dirSign = 1;
                    eoc = true;
                }
            }
            break;
        default: // Random
            if (L > 1)
            {
                int n = (int)(random::uniform() * (float)(L - 1));
                if (n >= pos)
                    n++;
                pos = rack::math::clamp(n, 0, L - 1);
            }
            else
                pos = 0;
            if (pos == 0)
                eoc = true;
            break;
        }

        // Commit the new selection.
        prevCh = curCh;
        curCh = (S + pos) % SW_N;
        pulse = 0.f;
        clockFlash = 1.f;
        stepCount++;

        // Push the new selection into the history ring buffer.
        hist[histHead] = curCh;
        histHead = (histHead + 1) % SW_HN;
        if (histCount < SW_HN)
            histCount++;

        // Trigger the EOC pulse when a cycle ends.
        if (eoc)
        {
            eocPulse.trigger(0.001f);
            eocFlash = 1.f;
        }
    }

    // --------------------------------------------------------
    // Per-sample DSP entry point.
    //  1. Read params, clamp, derive fadeT from FADE knob.
    //  2. Handle RESET and clock/step edges -> doReset / doStep.
    //  3. Sample SIGNAL input; ramp each output's gain toward
    //     its target (1 for curCh, 0 otherwise) at rate 1/fadeT.
    //  4. Write each output as signal * gain, plus gates & EOC.
    //  5. Advance display timers and mirror state for the UI.
    // --------------------------------------------------------
    void process(const ProcessArgs &args) override
    {
        const float dt = args.sampleTime;

        int L = rack::math::clamp((int)std::round(params[LENGTH_PARAM].getValue()), 1, SW_N);
        int S = rack::math::clamp((int)std::round(params[START_PARAM].getValue()) - 1, 0, SW_N - 1);
        int dir = rack::math::clamp((int)std::round(params[DIR_PARAM].getValue()), 0, 3);
        float fade = params[FADE_PARAM].getValue();
        // Map the knob to seconds; very low values mean "no fade".
        float fadeT = (fade < 0.01f) ? 0.f : (0.002f + fade * fade * 0.4f);

        // Keep the internal position valid if LENGTH shrank.
        if (pos >= L)
            pos = L - 1;

        // Edge triggers: RESET and (CLOCK OR STEP button).
        if (resetTrig.process(inputs[RESET_INPUT].getVoltage(), 0.1f, 2.f))
            doReset(S, L, dir);

        bool clk = clockTrig.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f);
        bool stp = stepTrig.process(params[STEP_PARAM].getValue(), 0.1f, 2.f);
        if (clk || stp)
            doStep(S, L, dir);

        // Defensive clamp after parameter changes.
        curCh = rack::math::clamp(curCh, 0, SW_N - 1);

        // Read the single input; unpatched reads as 0 V.
        inLinked = inputs[SIGNAL_INPUT].isConnected();
        inV = inLinked ? inputs[SIGNAL_INPUT].getVoltage() : 0.f;

        // Per-output gains: selected output ramps toward 1, others toward 0.
        // The linear ramp converges in fadeT seconds.
        for (int i = 0; i < SW_N; ++i)
        {
            float target = (i == curCh) ? 1.f : 0.f;
            if (fadeT <= 0.f)
            {
                gain[i] = target;
            }
            else
            {
                float step = dt / fadeT;
                if (gain[i] < target)
                    gain[i] = std::min(target, gain[i] + step);
                else
                    gain[i] = std::max(target, gain[i] - step);
            }
            outV[i] = inV * gain[i];
            linked[i] = outputs[CH_OUTPUT + i].isConnected();

            outputs[CH_OUTPUT + i].setVoltage(outV[i]);
            outputs[GATE_OUTPUT + i].setVoltage(i == curCh ? 10.f : 0.f);
        }
        outputs[EOC_OUTPUT].setVoltage(eocPulse.process(dt) ? 10.f : 0.f);

        // Decay display timers and push state snapshots for the UI.
        if (pulse < 2.f)
            pulse += dt * 5.f;
        clockFlash = std::max(0.f, clockFlash - dt * 8.f);
        eocFlash = std::max(0.f, eocFlash - dt * 4.f);
        lights[STEP_LIGHT].setBrightness(clockFlash);

        vLen = L;
        vStart = S;
        vDir = dir;
        vFadeT = fadeT;
    }
};

// ------------------------------------------------------------
// Drawing helpers (anonymous namespace — UI-only)
// ------------------------------------------------------------
namespace
{

    namespace ag = agmm;

    // Evaluate a cubic Bezier at parameter t and write x/y.
    void bezier(float t, float x0, float y0, float cx0, float cy0, float cx1, float cy1,
                float x1, float y1, float &x, float &y)
    {
        float u = 1.f - t;
        float b0 = u * u * u, b1 = 3.f * u * u * t, b2 = 3.f * u * t * t, b3 = t * t * t;
        x = b0 * x0 + b1 * cx0 + b2 * cx1 + b3 * x1;
        y = b0 * y0 + b1 * cy0 + b2 * cy1 + b3 * y1;
    }

    // --------------------------------------------------------
    // SWPanelBackground — static panel chrome:
    //   * title + panel base
    //   * display bezel
    //   * labelled cells for side knobs, control row, channel rows
    // --------------------------------------------------------
    struct SWPanelBackground : TransparentWidget
    {
        SWPanelBackground() { box.size = Vec(SW_PANEL_W, SW_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, SW_PANEL_W, SW_PANEL_H, "SWITCH 1 > 8");
            ag::bezel(vg, SW_DISP_X, SW_DISP_Y, SW_DISP_W, SW_DISP_H);

            // Side knob cells: LENGTH, START / DIR, FADE
            const char *knobL[4] = {"LENGTH", "START", "DIR", "FADE"};
            for (int k = 0; k < 4; ++k)
                ag::cell(vg, SW_SIDE_X[k], SW_SIDE_TOP[k], SW_SIDE_W, SW_SIDE_H, knobL[k]);

            // Control row: CLOCK / RESET / STEP / IN / EOC (EOC output is tinted).
            ag::sectionHeader(vg, SW_PANEL_W, SW_CTRL_HDR, "CONTROL");
            const char *ctl[5] = {"CLOCK", "RESET", "STEP", "IN", "EOC"};
            for (int i = 0; i < 5; ++i)
                ag::cell(vg, SW_CTRL_X[i], SW_CTRL_TOP, SW_CTRL_W, SW_CTRL_H, ctl[i], i == 4);

            // Channels: signal outputs on top row, gates on bottom (all outputs, tinted).
            ag::sectionHeader(vg, SW_PANEL_W, SW_CH_HDR, "CHANNELS");
            for (int i = 0; i < SW_N; ++i)
            {
                char buf[8];
                float cx = SW_CH_X0 + SW_CH_DX * (float)i;
                snprintf(buf, sizeof(buf), "OUT %d", i + 1);
                ag::cell(vg, cx, SW_OUT_TOP, SW_CH_W, SW_CH_H, buf, true, 7.f);
                snprintf(buf, sizeof(buf), "GATE %d", i + 1);
                ag::cell(vg, cx, SW_GATE_TOP, SW_CH_W, SW_CH_H, buf, true, 7.f);
            }

            Widget::draw(args);
        }
    };

    // --------------------------------------------------------
    // SWKnobArcs — draws an arc track + a value arc around each side knob.
    // Non-interactive; purely visual feedback for LENGTH/START/DIR/FADE.
    // --------------------------------------------------------
    struct SWKnobArcs : TransparentWidget
    {
        SwitchLines1to8 *module = nullptr;

        SWKnobArcs(SwitchLines1to8 *m) : module(m) { box.size = Vec(SW_PANEL_W, SW_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float R = 16.f;
            // Arc from ~225° to ~315° (CCW around the knob face).
            const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
            const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

            for (int k = 0; k < 4; ++k)
            {
                float cx = SW_KNOB_X[k], cy = SW_KNOB_Y[k];

                // Track
                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, R, a0, a1, NVG_CW);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
                nvgStrokeWidth(vg, 2.f);
                nvgLineCap(vg, NVG_ROUND);
                nvgStroke(vg);

                // Value arc (skip when very small)
                if (!module)
                    continue;
                float v = module->paramQuantities[k]->getScaledValue(); // 0..1
                if (v < 0.004f)
                    continue;

                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, R, a0, a0 + (a1 - a0) * v, NVG_CW);
                nvgStrokeColor(vg, ag::accent());
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }
        }
    };

    // --------------------------------------------------------
    // SWDisplayWidget — the live 224x152 routing display.
    //   draw()       : static background (screen fill, separators,
    //                  meter slots, input-bar slot, history frame).
    //   drawLayer(1) : per-sample live content (input bar, fan-out
    //                  curves, travelling pulse, hub, per-output
    //                  meters, history, header/footer readouts).
    // If module == nullptr (browser preview) demo data is drawn.
    // --------------------------------------------------------
    struct SWDisplayWidget : TransparentWidget
    {
        SwitchLines1to8 *module = nullptr;

        SWDisplayWidget(SwitchLines1to8 *m) : module(m)
        {
            box.pos = Vec(SW_DISP_X, SW_DISP_Y);
            box.size = Vec(SW_DISP_W, SW_DISP_H);
        }

        // ---- static background ----
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, SW_DISP_W, SW_DISP_H);

            // Separators under the header and above the footer.
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, SW_HEAD_LINE);
            nvgLineTo(vg, SW_DISP_W - 8.f, SW_HEAD_LINE);
            nvgMoveTo(vg, 8.f, SW_FOOT_LINE);
            nvgLineTo(vg, SW_DISP_W - 8.f, SW_FOOT_LINE);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Output meter slots (0 V centre line).
            for (int i = 0; i < SW_N; ++i)
            {
                float y = swRowY(i);
                nvgBeginPath(vg);
                nvgRect(vg, SW_MET_CX - SW_MET_HW, y - 2.5f, 2.f * SW_MET_HW, 5.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 10));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, SW_MET_CX + 0.5f, y - 3.5f);
                nvgLineTo(vg, SW_MET_CX + 0.5f, y + 3.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // Input bar slot with +-10 V, +-5 V, 0 V grid lines.
            float hy = swHubY();
            nvgBeginPath(vg);
            nvgRect(vg, SW_BAR_X, hy - SW_BAR_HALF, SW_BAR_W, 2.f * SW_BAR_HALF);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 10));
            nvgFill(vg);
            for (int k = -2; k <= 2; ++k)
            {
                float y = hy + (float)k * SW_BAR_HALF / 2.f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, SW_BAR_X - 2.f, y + 0.5f);
                nvgLineTo(vg, SW_BAR_X + SW_BAR_W + 2.f, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, k == 0 ? 50 : 18));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            ag::text(vg, SW_BAR_X + SW_BAR_W / 2.f, SW_IN_LBL_Y, "IN", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            // History frame.
            ag::plotFrame(vg, SW_HIST_X - 2.f, SW_HIST_Y - 2.f,
                          SW_HIST_P * (float)SW_HN + 3.f, SW_HIST_H + 4.f);

            Widget::draw(args);
        }

        // ---- live content ----
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            // --------------------------------------------------------
            // Snapshot module state (or demo data) into local variables.
            // --------------------------------------------------------
            float gain[SW_N], outV[SW_N];
            bool linked[SW_N];
            int hist[SW_HN];
            int cur, len, start, dir, head, count;
            float inV, pulse, clockFlash, eocFlash, fadeT;
            bool inLinked;
            uint32_t steps;

            if (module)
            {
                for (int i = 0; i < SW_N; ++i)
                {
                    gain[i] = module->gain[i];
                    outV[i] = module->outV[i];
                    linked[i] = module->linked[i];
                }
                for (int c = 0; c < SW_HN; ++c)
                    hist[c] = module->hist[c];
                cur = module->curCh;
                len = module->vLen;
                start = module->vStart;
                dir = module->vDir;
                head = module->histHead;
                count = module->histCount;
                inV = module->inV;
                inLinked = module->inLinked;
                pulse = module->pulse;
                clockFlash = module->clockFlash;
                eocFlash = module->eocFlash;
                fadeT = module->vFadeT;
                steps = module->stepCount;
            }
            else
            {
                // demo values for the module browser
                inV = 6.f;
                inLinked = true;
                for (int i = 0; i < SW_N; ++i)
                {
                    gain[i] = (i == 2) ? 0.6f : (i == 1) ? 0.4f
                                                         : 0.f;
                    outV[i] = inV * gain[i];
                    linked[i] = (i != 5);
                }
                for (int c = 0; c < SW_HN; ++c)
                    hist[c] = (c * 3) % SW_N;
                cur = 2;
                len = 6;
                start = 0;
                dir = 0;
                head = 0;
                count = SW_HN;
                pulse = 0.5f;
                clockFlash = 0.5f;
                eocFlash = 0.f;
                fadeT = 0.1f;
                steps = 64;
            }
            // Defensive clamps (state may be transiently out of range).
            cur = rack::math::clamp(cur, 0, SW_N - 1);
            len = rack::math::clamp(len, 1, SW_N);
            start = rack::math::clamp(start, 0, SW_N - 1);

            const float hx = SW_HUB_X, hy = swHubY();
            NVGcolor sig = (inV >= 0.f) ? ag::accent() : ag::warm();

            nvgSave(vg);
            nvgScissor(vg, 0, 0, SW_DISP_W, SW_DISP_H);

            // ================= input bar + line to the junction =================
            {
                // Bipolar input voltage bar.
                float t = rack::math::clamp(inV / 10.f, -1.f, 1.f);
                float h = t * SW_BAR_HALF;
                if (std::fabs(h) > 0.3f)
                {
                    float hh = std::max(std::fabs(h), 1.f);
                    float y0 = (h >= 0.f) ? hy - hh : hy;
                    nvgBeginPath(vg);
                    nvgRect(vg, SW_BAR_X, y0, SW_BAR_W, hh);
                    nvgFillColor(vg, ag::withA(sig, 0.6f));
                    nvgFill(vg);
                    // Bright tip marker at the extreme end.
                    float tipY = (h >= 0.f) ? y0 : y0 + hh;
                    nvgBeginPath(vg);
                    nvgRect(vg, SW_BAR_X - 1.f, tipY - 0.75f, SW_BAR_W + 2.f, 1.5f);
                    nvgFillColor(vg, ag::bright());
                    nvgFill(vg);
                }

                // Input node: filled = patched, hollow = unpatched.
                nvgBeginPath(vg);
                nvgCircle(vg, SW_INN_X, hy, 3.f);
                if (inLinked)
                {
                    nvgFillColor(vg, ag::bright());
                    nvgFill(vg);
                }
                else
                {
                    nvgStrokeColor(vg, ag::dim());
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }

                // Line from the input node to the hub.
                nvgBeginPath(vg);
                nvgMoveTo(vg, SW_INN_X + 3.f, hy);
                nvgLineTo(vg, hx - SW_HUB_R, hy);
                nvgStrokeColor(vg, ag::withA(ag::bright(), inLinked ? 0.85f : 0.3f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // ================= output rows + curves =================
            // Two passes: draw inactive rows first, then active rows
            // (any output whose gain is > 0) on top.
            for (int pass = 0; pass < 2; ++pass)
            {
                for (int i = 0; i < SW_N; ++i)
                {
                    // w = how "active" this row is right now (0..1).
                    float w = rack::math::clamp(gain[i], 0.f, 1.f);
                    bool act = (w > 0.01f);
                    if ((pass == 0) == act)
                        continue;

                    float y = swRowY(i);
                    // Dim rows that fall outside the current window.
                    bool inRange = (((i - start) % SW_N + SW_N) % SW_N) < len;
                    float dm = inRange ? 1.f : 0.4f;

                    float x0 = hx + SW_HUB_R, x1 = SW_NODE_X - 3.f;
                    float cxm = (x0 + x1) * 0.5f;

                    // Curve from the hub to the output node.
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, x0, hy);
                    nvgBezierTo(vg, cxm, hy, cxm, y, x1, y);
                    if (!act)
                    {
                        nvgStrokeColor(vg, nvgRGBA(0x8a, 0x95, 0xa4, (int)(60 * dm)));
                    }
                    else
                    {
                        nvgStrokeColor(vg, ag::withA(ag::bright(), 0.35f + 0.60f * w));
                    }
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);

                    // Travelling pulse on the selected line.
                    if (i == cur && pulse < 1.f)
                    {
                        float px, py;
                        bezier(pulse, x0, hy, cxm, hy, cxm, y, x1, y, px, py);
                        nvgBeginPath(vg);
                        nvgCircle(vg, px, py, 2.f);
                        nvgFillColor(vg, nvgRGBA(255, 255, 255, 235));
                        nvgFill(vg);
                    }

                    // Node: filled = patched, hollow = unpatched.
                    nvgBeginPath(vg);
                    nvgCircle(vg, SW_NODE_X, y, act ? 3.f : 2.4f);
                    if (linked[i])
                    {
                        nvgFillColor(vg, act ? ag::bright()
                                             : nvgRGBA(0x8a, 0x95, 0xa4, (int)(200 * dm)));
                        nvgFill(vg);
                    }
                    else
                    {
                        nvgStrokeColor(vg, nvgRGBA(0x8a, 0x95, 0xa4, (int)(160 * dm)));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }

                    // Output voltage meter (bipolar, ±10 V).
                    float t = rack::math::clamp(outV[i] / 10.f, -1.f, 1.f);
                    float mw = t * SW_MET_HW;
                    NVGcolor mc = (outV[i] >= 0.f) ? ag::accent() : ag::warm();
                    if (std::fabs(mw) > 0.2f)
                    {
                        float mx0 = (mw >= 0.f) ? SW_MET_CX : SW_MET_CX + mw;
                        nvgBeginPath(vg);
                        nvgRect(vg, mx0, y - 2.f, std::max(std::fabs(mw), 1.f), 4.f);
                        nvgFillColor(vg, ag::withA(mc, (0.45f + 0.5f * w) * dm));
                        nvgFill(vg);
                    }

                    // Output number.
                    char b[4];
                    snprintf(b, sizeof(b), "%d", i + 1);
                    ag::text(vg, SW_LBL_X, y, b, 7.f,
                             act ? ag::bright() : nvgRGBA(0x8a, 0x95, 0xa4, (int)(230 * dm)),
                             ag::ALIGN_C, 0.f);
                }
            }

            // ================= junction =================
            {
                // Hub circle (flashes with the clock).
                nvgBeginPath(vg);
                nvgCircle(vg, hx, hy, SW_HUB_R);
                nvgFillColor(vg, nvgRGB(0x12, 0x16, 0x1b));
                nvgFill(vg);
                nvgStrokeColor(vg, ag::withA(ag::bright(), 0.6f + 0.4f * clockFlash));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Selected channel number inside the hub.
                char b[4];
                snprintf(b, sizeof(b), "%d", cur + 1);
                ag::text(vg, hx, hy + 0.5f, b, 9.f, ag::bright(), ag::ALIGN_C, 0.f);
            }

            // ================= history strip =================
            // 16 cells, oldest -> newest from left to right; newest is
            // brightest and gets a highlighted outline.
            for (int c = 0; c < SW_HN; ++c)
            {
                if (c < SW_HN - count)
                    continue;
                int idx = ((head - SW_HN + c) % SW_HN + SW_HN) % SW_HN;
                float x = SW_HIST_X + (float)c * SW_HIST_P;
                float age = (float)c / (float)(SW_HN - 1); // 1 = newest
                float a = 0.10f + 0.45f * age * age;

                nvgBeginPath(vg);
                nvgRect(vg, x, SW_HIST_Y, SW_HIST_W, SW_HIST_H);
                nvgFillColor(vg, ag::withA(ag::accent(), a));
                nvgFill(vg);

                char b[4];
                snprintf(b, sizeof(b), "%d", hist[idx] + 1);
                ag::text(vg, x + SW_HIST_W / 2.f, SW_HIST_Y + SW_HIST_H / 2.f + 0.5f, b, 7.f,
                         nvgRGBA(255, 255, 255, (int)(130 + 110 * age)), ag::ALIGN_C, 0.f);
            }
            {
                // Outline on the newest cell (pulses with the clock flash).
                float x = SW_HIST_X + (float)(SW_HN - 1) * SW_HIST_P;
                nvgBeginPath(vg);
                nvgRect(vg, x - 0.5f, SW_HIST_Y - 0.5f, SW_HIST_W + 1.f, SW_HIST_H + 1.f);
                nvgStrokeColor(vg, ag::withA(ag::bright(), 0.30f + 0.65f * clockFlash));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // ================= header and footer readouts =================
            {
                char buf[48];

                // Clock lamp.
                nvgBeginPath(vg);
                nvgCircle(vg, 8.f, 8.f, 2.5f);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.25f + 0.75f * clockFlash));
                nvgFill(vg);

                // "IN > OUT x"
                snprintf(buf, sizeof(buf), "IN > OUT %d", cur + 1);
                ag::text(vg, 15.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);

                // Direction indicator.
                const char *dn[4] = {"FWD", "REV", "PEND", "RAND"};
                ag::text(vg, 112.f, 8.f, dn[rack::math::clamp(dir, 0, 3)], 7.f,
                         ag::bright(), ag::ALIGN_C, 0.5f);

                // EOC lamp + caption.
                nvgBeginPath(vg);
                nvgCircle(vg, 150.f, 8.f, 2.5f);
                nvgFillColor(vg, ag::withA(ag::warm(), 0.25f + 0.75f * eocFlash));
                nvgFill(vg);
                ag::text(vg, 156.f, 8.f, "EOC", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);

                // Length readout.
                snprintf(buf, sizeof(buf), "LEN %d", len);
                ag::text(vg, SW_DISP_W - 8.f, 8.f, buf, 7.f, ag::warm(), ag::ALIGN_R, 0.5f);

                // Footer: OLD / step count (+ fade time) / NEW.
                const float histR = SW_HIST_X + SW_HIST_P * (float)SW_HN - 2.f;
                ag::text(vg, SW_HIST_X, SW_CAP_Y, "OLD", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
                if (fadeT > 0.f)
                    snprintf(buf, sizeof(buf), "STEP %u  FADE %.0fMS", (unsigned)steps, fadeT * 1000.f);
                else
                    snprintf(buf, sizeof(buf), "STEP %u", (unsigned)steps);
                ag::text(vg, SW_DISP_W / 2.f, SW_CAP_Y, buf, 7.f, ag::accent(), ag::ALIGN_C, 0.5f);
                ag::text(vg, histR, SW_CAP_Y, "NEW", 7.f, ag::dim(), ag::ALIGN_R, 0.5f);
            }

            nvgRestore(vg);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// SwitchLines1to8Widget — Rack widget.
//   * adds background / knob-arc / display widgets
//   * adds screws
//   * adds the four side knobs (LENGTH, START, DIR, FADE)
//   * adds the control row (CLOCK, RESET, STEP button + LED, SIGNAL, EOC)
//   * adds the eight signal output jacks and eight gate output jacks
// ------------------------------------------------------------
struct SwitchLines1to8Widget : ModuleWidget
{
    SwitchLines1to8Widget(SwitchLines1to8 *module)
    {
        setModule(module);
        box.size = Vec(SW_PANEL_W, SW_PANEL_H);

        addChild(new SWPanelBackground());
        addChild(new SWKnobArcs(module));

        // Screws (top row, bottom row).
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SWDisplayWidget(module));

        // Side knobs: LENGTH, START, DIR, FADE
        for (int k = 0; k < 4; ++k)
        {
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
        addChild(createLightCentered<MediumLight<ag::CyanLight>>(
            Vec(SW_CTRL_X[2], SW_CTRL_Y), module, SwitchLines1to8::STEP_LIGHT));

        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(SW_CTRL_X[3], SW_CTRL_Y), module, SwitchLines1to8::SIGNAL_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(SW_CTRL_X[4], SW_CTRL_Y), module, SwitchLines1to8::EOC_OUTPUT));

        // Channel outputs (upper row) and gate outputs (lower row)
        for (int i = 0; i < SW_N; ++i)
        {
            float cx = SW_CH_X0 + SW_CH_DX * (float)i;
            addOutput(createOutputCentered<DarkPJ301MPort>(
                Vec(cx, SW_OUT_TOP + SW_CH_JACK_DY), module, SwitchLines1to8::CH_OUTPUT + i));
            addOutput(createOutputCentered<DarkPJ301MPort>(
                Vec(cx, SW_GATE_TOP + SW_CH_JACK_DY), module, SwitchLines1to8::GATE_OUTPUT + i));
        }
    }

    void appendContextMenu(Menu *menu) override
    {
        agmm::addYouTubeLink(menu); // adds "Visit my youtube channel!"
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        // ... your other menu items ...
    }
};

Model *modelSwitchLines1to8 = createModel<SwitchLines1to8, SwitchLines1to8Widget>("SwitchLines1to8");