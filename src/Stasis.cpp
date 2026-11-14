#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <memory>

// ============================================================================
// Stasis
// ============================================================================
//
// OVERVIEW
// --------
// Stasis is a 24 HP 8-stage clocked sample-and-hold "shift register" with
// probability and slew. On each clock edge every stage samples a new value:
//   - Stage 1 takes its input from a patched jack, or (if unpatched) either
//     an internal white-noise source or, with LOOP on, the last active
//     stage's previous value (a feedback loop).
//   - Stages 2..8 take the previous stage's previous value (a shift register).
//   - A patched input at stage i breaks the chain at that point.
//   - With PROB below 100%, each stage independently decides whether to
//     sample or hold (and flashes a skipped outline).
//
// Outputs are per-stage jacks plus an 8-channel polyphonic output. A SLEW
// control smooths the outputs (1 ms .. 2 s time constant), and LEVEL scales
// all outputs from -100% to +100% (bipolar, so it can invert).
//
// Displays:
//   - Live bars per stage (violet for negative, cyan for positive), with a
//     tip cap, a flash overlay when a new value was sampled, a skipped
//     outline, and a slewed-output pointer beside each bar.
//   - Stage numbers with chain arrows, a LOOP bus when looping is on, and
//     stage 1 / last stage highlights.
//   - A 24-column history heat map (one row per stage) whose colour shows the
//     polarity and whose left edge marks the samples that actually landed.
//
// File structure:
//   1. Layout constants (24 HP = 360×380 px).
//   2. Stasis module: register state, clock/probability/slew logic, loop,
//      JSON persistence.
//   3. Anonymous-namespace UI: STPanelBackground, STKnobArcs, and
//      STDisplayWidget (bars + chain + history heat map).
//   4. StasisWidget: places knobs, buttons, jacks, LEDs and the display.
//   5. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Layout constants (24 HP = 360 px wide)
// ------------------------------------------------------------
static const int   ST_N  = 8;     // channels / register stages
static const int   ST_HN = 24;    // history columns

static const float ST_PANEL_W = RACK_GRID_WIDTH * 24;   // 360
static const float ST_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Display (224 x 152): header row, bars, chain, heat map, footer row. Bezel adds 4 px around it.
static const float ST_DISP_W = 224.f;
static const float ST_DISP_H = 152.f;
static const float ST_DISP_X = (ST_PANEL_W - ST_DISP_W) / 2.f;   // 68
static const float ST_DISP_Y = 26.f;

// Plot geometry inside the display (display-local coordinates)
static const float ST_HEAD_LINE = 15.5f;  // hairline under the header row
static const float ST_PX0     = 22.f;     // left edge of bars / heat map
static const float ST_PITCH   = 24.f;     // column pitch (8 x 24 = 192)
static const float ST_BAR_CY  = 42.f;     // zero line of the bars
static const float ST_BAR_H   = 22.f;     // half height (10 V)
static const float ST_BAR_W   = 14.f;
static const float ST_LBL_Y   = 72.f;     // stage numbers + chain arrows
static const float ST_LOOP_Y  = 81.f;     // loop bus
static const float ST_HEAT_Y  = 85.f;     // heat map top
static const float ST_ROW_H   = 6.f;      // 8 rows -> 48 px
static const float ST_FOOT_LINE = 136.5f; // hairline above the footer row
static const float ST_CAP_Y   = 144.f;    // footer text centre line

// Side knob cells: SLEW, PROB (left) / LENGTH, LEVEL (right). 50 x 52, label at +10, knob at +32
static const float ST_SIDE_W = 50.f, ST_SIDE_H = 52.f, ST_SIDE_DY = 32.f;
static const float ST_SIDE_X[4]   = {34.f, 34.f, 326.f, 326.f};
static const float ST_SIDE_TOP[4] = {40.f, 112.f, 40.f, 112.f};
static const float ST_KNOB_X[4]   = {34.f, 34.f, 326.f, 326.f};
static const float ST_KNOB_Y[4]   = {72.f, 144.f, 72.f, 144.f};   // SIDE_TOP + SIDE_DY

// Control row: five 52 x 48 cells, control at +29
static const float ST_CTRL_HDR = 192.f;
static const float ST_CTRL_TOP = 200.f;
static const float ST_CTRL_W = 52.f, ST_CTRL_H = 48.f;
static const float ST_CTRL_Y = ST_CTRL_TOP + 29.f;
static const float ST_CTRL_X[5] = {60.f, 120.f, 180.f, 240.f, 300.f};

// Channel rows: eight 36 x 42 cells per row, jack at +28
static const float ST_CH_HDR = 258.f;
static const float ST_IN_TOP = 266.f;
static const float ST_OUT_TOP = 312.f;
static const float ST_CH_W = 36.f, ST_CH_H = 42.f, ST_CH_JACK_DY = 28.f;
static const float ST_CH_X0 = 40.f;
static const float ST_CH_DX = 40.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// 8-stage clocked sample-and-hold shift register with probability, slew,
// loop-back and stereo/poly output.
struct Stasis : Module {
    enum ParamId {
        SLEW_PARAM,
        PROB_PARAM,
        LENGTH_PARAM,
        LEVEL_PARAM,
        LOOP_PARAM,
        STEP_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        CLOCK_INPUT,
        RESET_INPUT,
        ENUMS(CH_INPUT, 8),
        INPUTS_LEN
    };
    enum OutputId {
        ENUMS(CH_OUTPUT, 8),
        POLY_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LOOP_LIGHT,
        STEP_LIGHT,
        LIGHTS_LEN
    };

    // Register contents (what the S&H stages hold) and smoothed outputs
    float held[ST_N]  = {};
    float outv[ST_N]  = {};

    // Display data
    float flash[ST_N] = {};       // sampled this clock
    float skip[ST_N]  = {};       // skipped by probability
    bool  linked[ST_N] = {};      // input normalled (not patched)
    float clockFlash = 0.f;
    float hist[ST_N][ST_HN] = {};
    bool  histUpd[ST_N][ST_HN] = {};
    int   histHead = 0;
    int   histCount = 0;
    uint32_t stepCount = 0;
    int   vLen = 8;
    float vProb = 1.f;
    float vTau = 0.f;

    bool loopOn = false;

    dsp::SchmittTrigger clockTrig, stepTrig, resetTrig, loopTrig;

    // Constructor: configures all six parameters (with LENGTH snapped to
    // integer steps), the clock/reset inputs, the eight per-stage inputs and
    // outputs, the poly output, and initialises the register to silence.
    Stasis() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(SLEW_PARAM, 0.f, 1.f, 0.f, "Slew", "%", 0.f, 100.f);
        configParam(PROB_PARAM, 0.f, 1.f, 1.f, "Sample probability", "%", 0.f, 100.f);
        configParam(LENGTH_PARAM, 1.f, 8.f, 8.f, "Loop length", " steps");
        paramQuantities[LENGTH_PARAM]->snapEnabled = true;
        configParam(LEVEL_PARAM, -1.f, 1.f, 1.f, "Output level", "%", 0.f, 100.f);
        configButton(LOOP_PARAM, "Loop (last active stage feeds stage 1)");
        configButton(STEP_PARAM, "Manual clock step");

        configInput(CLOCK_INPUT, "Clock");
        configInput(RESET_INPUT, "Reset");
        for (int i = 0; i < ST_N; ++i) {
            if (i == 0)
                configInput(CH_INPUT + i, "Channel 1 input (normalled to noise / loop)");
            else
                configInput(CH_INPUT + i, string::f("Channel %d input (normalled to channel %d)", i + 1, i));
            configOutput(CH_OUTPUT + i, string::f("Channel %d", i + 1));
        }
        configOutput(POLY_OUTPUT, "Polyphonic (8 channels)");

        doReset();
    }

    // Clears the register, all flash/skip/history state and the step counter.
    void doReset() {
        std::memset(held, 0, sizeof(held));
        std::memset(flash, 0, sizeof(flash));
        std::memset(skip, 0, sizeof(skip));
        std::memset(hist, 0, sizeof(hist));
        std::memset(histUpd, 0, sizeof(histUpd));
        histHead = 0;
        histCount = 0;
        stepCount = 0;
    }

    // Rack reset hook: clears the register state and disables looping.
    void onReset(const ResetEvent& e) override {
        Module::onReset(e);
        doReset();
        loopOn = false;
    }

    // Performs one clock step over the first `len` stages: each stage samples
    // (with probability `prob`) either its patched input, a noise source (stage
    // 1 when loop is off), or the previous stage's previous value. The sampled
    // or skipped state is recorded in the history and flash arrays.
    void doClock(int len, float prob) {
        float old[ST_N];
        for (int i = 0; i < ST_N; ++i) old[i] = held[i];

        for (int i = 0; i < ST_N; ++i) {
            bool conn = inputs[CH_INPUT + i].isConnected();
            float src;
            if (conn) {
                src = inputs[CH_INPUT + i].getVoltage();
            }
            else if (i == 0) {
                // stage 1: loop tap, or internal white noise
                src = loopOn ? old[len - 1] : (random::uniform() * 10.f - 5.f);
            }
            else {
                // normalled chain: take the previous stage's held value
                src = old[i - 1];
            }
            linked[i] = !conn;

            bool take = (prob >= 0.999f) || (random::uniform() < prob);
            if (take) {
                held[i] = src;
                flash[i] = 1.f;
            } else {
                skip[i] = 1.f;
            }
            hist[i][histHead] = held[i];
            histUpd[i][histHead] = take;
        }
        histHead = (histHead + 1) % ST_HN;
        if (histCount < ST_HN) histCount++;
        stepCount++;
        clockFlash = 1.f;
    }

    // Main audio callback: reads controls, handles loop/reset/clock/step,
    // performs a clock step if triggered, applies slew to each stage's
    // output, writes the eight jacks and the poly output, and decays the
    // flash/skip timers for the display.
    void process(const ProcessArgs& args) override {
        const float dt = args.sampleTime;

        float slewK = params[SLEW_PARAM].getValue();
        float prob  = params[PROB_PARAM].getValue();
        int   len   = rack::math::clamp((int)std::round(params[LENGTH_PARAM].getValue()), 1, ST_N);
        float level = params[LEVEL_PARAM].getValue();

        if (loopTrig.process(params[LOOP_PARAM].getValue(), 0.1f, 2.f))
            loopOn = !loopOn;

        if (resetTrig.process(inputs[RESET_INPUT].getVoltage(), 0.1f, 2.f))
            doReset();

        bool clk = clockTrig.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f);
        bool stp = stepTrig.process(params[STEP_PARAM].getValue(), 0.1f, 2.f);
        if (clk || stp) doClock(len, prob);

        // Slew: 1 ms .. 2 s time constant
        float tau = (slewK < 0.002f) ? 0.f : 0.001f * std::pow(2000.f, slewK);
        float a = (tau > 0.f) ? (1.f - std::exp(-dt / tau)) : 1.f;

        outputs[POLY_OUTPUT].setChannels(ST_N);
        for (int i = 0; i < ST_N; ++i) {
            outv[i] += (held[i] - outv[i]) * a;
            float v = outv[i] * level;
            outputs[CH_OUTPUT + i].setVoltage(v);
            outputs[POLY_OUTPUT].setVoltage(v, i);

            flash[i] = std::max(0.f, flash[i] - dt * 6.f);
            skip[i]  = std::max(0.f, skip[i]  - dt * 4.f);
        }
        clockFlash = std::max(0.f, clockFlash - dt * 8.f);

        lights[LOOP_LIGHT].setBrightness(loopOn ? 1.f : 0.f);
        lights[STEP_LIGHT].setBrightness(clockFlash);

        vLen = len;
        vProb = prob;
        vTau = tau;
    }

    // Serialises the loop on/off state so a patch reopens with the same mode.
    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "loop", json_boolean(loopOn));
        return root;
    }

    // Restores the loop on/off state from a saved patch.
    void dataFromJson(json_t* root) override {
        json_t* j = json_object_get(root, "loop");
        if (j) loopOn = json_is_true(j);
    }
};

// ------------------------------------------------------------
// Drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace {

namespace ag = agmm;

// Returns the x centre of stage column i in display-local coordinates.
float colX(int i) { return ST_PX0 + ST_PITCH * ((float)i + 0.5f); }

// ------------------------------------------------------------
// Panel background: title, display frame, one labelled cell per control
// ------------------------------------------------------------
// Static artwork: base panel, display bezel, the four side knob cells, the
// CONTROL row (clock/reset/loop/step/poly), and the CHANNELS rows (inputs on
// the upper row, tinted outputs on the lower row).
struct STPanelBackground : TransparentWidget {
    STPanelBackground() { box.size = Vec(ST_PANEL_W, ST_PANEL_H); }

    // Draws all static panel art including section headers and cell plates.
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        ag::panelBase(vg, ST_PANEL_W, ST_PANEL_H, "STASIS");
        ag::bezel(vg, ST_DISP_X, ST_DISP_Y, ST_DISP_W, ST_DISP_H);

        // Side knob cells
        const char* knobL[4] = {"SLEW", "PROB", "LENGTH", "LEVEL"};
        for (int k = 0; k < 4; ++k)
            ag::cell(vg, ST_SIDE_X[k], ST_SIDE_TOP[k], ST_SIDE_W, ST_SIDE_H, knobL[k]);

        // Control row: inputs left, output right (tinted)
        ag::sectionHeader(vg, ST_PANEL_W, ST_CTRL_HDR, "CONTROL");
        const char* ctl[5] = {"CLOCK", "RESET", "LOOP", "STEP", "POLY"};
        for (int i = 0; i < 5; ++i)
            ag::cell(vg, ST_CTRL_X[i], ST_CTRL_TOP, ST_CTRL_W, ST_CTRL_H, ctl[i], i == 4);

        // Channels: inputs on the upper row, outputs (tinted) on the lower row
        ag::sectionHeader(vg, ST_PANEL_W, ST_CH_HDR, "CHANNELS");
        for (int i = 0; i < ST_N; ++i) {
            char buf[8];
            float cx = ST_CH_X0 + ST_CH_DX * (float)i;
            snprintf(buf, sizeof(buf), "IN %d", i + 1);
            ag::cell(vg, cx, ST_IN_TOP, ST_CH_W, ST_CH_H, buf);
            snprintf(buf, sizeof(buf), "OUT %d", i + 1);
            ag::cell(vg, cx, ST_OUT_TOP, ST_CH_W, ST_CH_H, buf, true);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Live value arcs around the knobs (track + one value arc)
// ------------------------------------------------------------
// Draws the four side-knob value arcs, with the bipolar LEVEL knob drawing
// its arc from the centre and using the warm colour when negative.
struct STKnobArcs : TransparentWidget {
    Stasis* module = nullptr;

    // Constructor: positions the widget over the whole panel.
    STKnobArcs(Stasis* m) : module(m) { box.size = Vec(ST_PANEL_W, ST_PANEL_H); }

    // Draws the track and value arcs for all four knobs.
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float R = 16.f;
        const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
        const float a1 =  0.75f * (float)M_PI - (float)M_PI / 2.f;

        for (int k = 0; k < 4; ++k) {
            float cx = ST_KNOB_X[k], cy = ST_KNOB_Y[k];

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, a0, a1, NVG_CW);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 2.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStroke(vg);

            if (!module) continue;
            float v = module->paramQuantities[k]->getScaledValue();   // 0..1
            bool bipolar = (k == Stasis::LEVEL_PARAM);
            float from = bipolar ? 0.5f : 0.f;
            float lo = std::min(from, v), hi = std::max(from, v);
            if (hi - lo < 0.004f) continue;

            float s = a0 + (a1 - a0) * lo;
            float e = a0 + (a1 - a0) * hi;
            NVGcolor col = (bipolar && v < 0.5f) ? ag::warm() : ag::accent();

            nvgBeginPath(vg);
            nvgArc(vg, cx, cy, R, s, e, NVG_CW);
            nvgStrokeColor(vg, col);
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
        }
    }
};

// ------------------------------------------------------------
// Register display: bars + chain + loop bus + history heat map
// ------------------------------------------------------------
// Live register display: draws the per-stage bars, chain arrows, loop bus,
// history heat map and readouts. Uses demo values in the module browser.
struct STDisplayWidget : TransparentWidget {
    Stasis* module = nullptr;

    // Constructor: positions the display widget under its bezel.
    STDisplayWidget(Stasis* m) : module(m) {
        box.pos = Vec(ST_DISP_X, ST_DISP_Y);
        box.size = Vec(ST_DISP_W, ST_DISP_H);
    }

    // ---- static background ----
    // Draws the display screen, header/footer hairlines, empty bar slots, the
    // voltage grid with labels, the heat map frame and its stage-number labels.
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float plotW = ST_PITCH * ST_N;

        ag::screen(vg, 0.f, 0.f, ST_DISP_W, ST_DISP_H);

        // separators under the header and above the footer
        nvgBeginPath(vg);
        nvgMoveTo(vg, 8.f, ST_HEAD_LINE);
        nvgLineTo(vg, ST_DISP_W - 8.f, ST_HEAD_LINE);
        nvgMoveTo(vg, 8.f, ST_FOOT_LINE);
        nvgLineTo(vg, ST_DISP_W - 8.f, ST_FOOT_LINE);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Bar slots
        for (int i = 0; i < ST_N; ++i) {
            nvgBeginPath(vg);
            nvgRect(vg, colX(i) - ST_BAR_W / 2.f - 1.f, ST_BAR_CY - ST_BAR_H,
                    ST_BAR_W + 2.f, 2.f * ST_BAR_H);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 8));
            nvgFill(vg);
        }

        // Voltage grid: +-10 V, +-5 V, 0 V
        const float gy[5] = {-ST_BAR_H, -ST_BAR_H / 2.f, 0.f, ST_BAR_H / 2.f, ST_BAR_H};
        for (int i = 0; i < 5; ++i) {
            float y = ST_BAR_CY + gy[i];
            nvgBeginPath(vg);
            nvgMoveTo(vg, ST_PX0 - 2.f, y + 0.5f);
            nvgLineTo(vg, ST_PX0 + plotW + 2.f, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, i == 2 ? 50 : 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }
        ag::text(vg, ST_PX0 - 4.f, ST_BAR_CY - ST_BAR_H, "+10", 7.f, ag::dim(), ag::ALIGN_R, 0.f);
        ag::text(vg, ST_PX0 - 4.f, ST_BAR_CY,            "0",   7.f, ag::dim(), ag::ALIGN_R, 0.f);
        ag::text(vg, ST_PX0 - 4.f, ST_BAR_CY + ST_BAR_H, "-10", 7.f, ag::dim(), ag::ALIGN_R, 0.f);

        // Heat map frame + row labels
        ag::plotFrame(vg, ST_PX0 - 1.f, ST_HEAT_Y - 1.f, plotW + 2.f, ST_ROW_H * ST_N + 2.f);
        for (int r = 0; r < ST_N; ++r) {
            char b[4];
            snprintf(b, sizeof(b), "%d", r + 1);
            ag::text(vg, ST_PX0 - 5.f, ST_HEAT_Y + ((float)r + 0.5f) * ST_ROW_H, b, 7.f, ag::dim(),
                     ag::ALIGN_R, 0.f);
        }

        Widget::draw(args);
    }

    // ---- live content ----
    // Layer-1 overlay: draws per-stage bars with flash/skip indicators and
    // slewed-output pointers, chain arrows, the loop bus, the history heat
    // map, and the header/footer readouts.
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float plotW = ST_PITCH * ST_N;

        // Snapshot module state (or demo values in the module browser)
        float held[ST_N], outv[ST_N], flash[ST_N], skip[ST_N];
        bool  linked[ST_N];
        float hist[ST_N][ST_HN];
        bool  upd[ST_N][ST_HN];
        bool  loopOn;
        int   len, head, count;
        float prob, tau, clockFlash;
        uint32_t steps;

        if (module) {
            for (int i = 0; i < ST_N; ++i) {
                held[i] = module->held[i];
                outv[i] = module->outv[i];
                flash[i] = module->flash[i];
                skip[i] = module->skip[i];
                linked[i] = module->linked[i];
                for (int c = 0; c < ST_HN; ++c) {
                    hist[i][c] = module->hist[i][c];
                    upd[i][c] = module->histUpd[i][c];
                }
            }
            loopOn = module->loopOn;
            len = module->vLen;
            prob = module->vProb;
            tau = module->vTau;
            clockFlash = module->clockFlash;
            head = module->histHead;
            count = module->histCount;
            steps = module->stepCount;
        } else {
            // demo values for the module browser
            for (int i = 0; i < ST_N; ++i) {
                held[i] = 6.f * std::sin(0.9f * (float)i + 0.6f);
                outv[i] = held[i] * 0.9f;
                flash[i] = (i == 2) ? 0.8f : 0.f;
                skip[i] = (i == 5) ? 0.7f : 0.f;
                linked[i] = (i != 4);
                for (int c = 0; c < ST_HN; ++c) {
                    hist[i][c] = 7.f * std::sin(0.5f * (float)(c - i) + 0.4f * (float)i);
                    upd[i][c] = ((c + i) % 3) != 0;
                }
            }
            loopOn = true;
            len = 6;
            prob = 0.8f;
            tau = 0.12f;
            clockFlash = 0.6f;
            head = 0;
            count = ST_HN;
            steps = 128;
        }
        len = rack::math::clamp(len, 1, ST_N);

        nvgSave(vg);
        nvgScissor(vg, 0, 0, ST_DISP_W, ST_DISP_H);

        // ================= bars =================
        // One bar per stage: filled in the polarity colour, with a flash
        // overlay when this clock sampled, a bright tip cap, an orange outline
        // when skipped, and a white pointer beside the bar for the slewed
        // output value.
        for (int i = 0; i < ST_N; ++i) {
            float cx = colX(i);
            float t = rack::math::clamp(held[i] / 10.f, -1.f, 1.f);
            float h = t * ST_BAR_H;
            bool pos = (t >= 0.f);
            NVGcolor col = pos ? ag::accent() : ag::warm();
            NVGcolor hi  = pos ? ag::bright() : nvgRGB(0xff, 0xc8, 0xa8);
            float dim = (loopOn && i >= len) ? 0.45f : 1.f;

            float bh = std::max(std::fabs(h), 1.f);
            float y0 = pos ? (ST_BAR_CY - bh) : ST_BAR_CY;
            float yTip = pos ? y0 : y0 + bh;

            // body (flat)
            nvgBeginPath(vg);
            nvgRect(vg, cx - ST_BAR_W / 2.f, y0, ST_BAR_W, bh);
            nvgFillColor(vg, ag::withA(col, 0.6f * dim));
            nvgFill(vg);

            // flash overlay
            if (flash[i] > 0.01f) {
                nvgBeginPath(vg);
                nvgRect(vg, cx - ST_BAR_W / 2.f, y0, ST_BAR_W, bh);
                nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, flash[i] * 0.5f));
                nvgFill(vg);
            }

            // tip cap
            nvgBeginPath(vg);
            nvgRect(vg, cx - ST_BAR_W / 2.f, yTip - 0.75f, ST_BAR_W, 1.5f);
            nvgFillColor(vg, ag::withA(hi, dim));
            nvgFill(vg);

            // skipped-by-probability outline
            if (skip[i] > 0.01f) {
                nvgBeginPath(vg);
                nvgRect(vg, cx - ST_BAR_W / 2.f - 1.5f, ST_BAR_CY - ST_BAR_H - 0.5f,
                        ST_BAR_W + 3.f, 2.f * ST_BAR_H + 1.f);
                nvgStrokeColor(vg, ag::withA(ag::warm(), skip[i] * 0.85f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // slewed-output marker (small pointer on the right of the bar)
            float yo = ST_BAR_CY - rack::math::clamp(outv[i] / 10.f, -1.f, 1.f) * ST_BAR_H;
            float mx = cx + ST_BAR_W / 2.f + 1.5f;
            nvgBeginPath(vg);
            nvgMoveTo(vg, mx, yo);
            nvgLineTo(vg, mx + 3.5f, yo - 2.2f);
            nvgLineTo(vg, mx + 3.5f, yo + 2.2f);
            nvgClosePath(vg);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(200 * dim)));
            nvgFill(vg);
        }

        // ================= stage numbers, chain arrows, loop bus =================
        // Numbers under each stage, with chain arrows between them (cyan when
        // normalled, orange when broken by a patched input). When loop is on,
        // the last active stage's number is boxed and a bus returns it to
        // stage 1.
        for (int i = 0; i < ST_N; ++i) {
            char b[4];
            snprintf(b, sizeof(b), "%d", i + 1);
            bool special = loopOn && (i == 0 || i == len - 1);
            float cx = colX(i);

            if (special) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, cx - 5.5f, ST_LBL_Y - 4.5f, 11.f, 9.f, 2.f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 150));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            ag::text(vg, cx, ST_LBL_Y, b, 7.f, special ? ag::bright() : ag::dim(), ag::ALIGN_C, 0.f);

            // link between stage i and i+1
            if (i < ST_N - 1) {
                float ax = ST_PX0 + ST_PITCH * (float)(i + 1);
                if (linked[i + 1]) {
                    float a = 0.50f + 0.50f * flash[i + 1];
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, ax - 1.8f, ST_LBL_Y - 2.6f);
                    nvgLineTo(vg, ax + 1.4f, ST_LBL_Y);
                    nvgLineTo(vg, ax - 1.8f, ST_LBL_Y + 2.6f);
                    nvgLineCap(vg, NVG_ROUND);
                    nvgLineJoin(vg, NVG_ROUND);
                    nvgStrokeColor(vg, ag::withA(ag::accent(), a));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                } else {
                    // chain broken by a patched input
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, ax, ST_LBL_Y - 2.6f);
                    nvgLineTo(vg, ax, ST_LBL_Y + 2.6f);
                    nvgStrokeColor(vg, nvgRGBA(0xff, 0x9c, 0x6a, 150));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
            }
        }

        if (loopOn) {
            float xt = colX(len - 1), xh = colX(0);
            float yTop = ST_LBL_Y + 4.8f;
            if (len == 1) { xt = xh + 5.f; xh = xh - 5.f; }

            nvgBeginPath(vg);
            nvgMoveTo(vg, xt, yTop);
            nvgLineTo(vg, xt, ST_LOOP_Y);
            nvgLineTo(vg, xh, ST_LOOP_Y);
            nvgLineTo(vg, xh, yTop + 3.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStrokeColor(vg, ag::withA(ag::bright(), 0.6f + 0.4f * clockFlash));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // arrow head into stage 1
            nvgBeginPath(vg);
            nvgMoveTo(vg, xh, yTop - 0.3f);
            nvgLineTo(vg, xh - 2.3f, yTop + 3.4f);
            nvgLineTo(vg, xh + 2.3f, yTop + 3.4f);
            nvgClosePath(vg);
            nvgFillColor(vg, ag::bright());
            nvgFill(vg);
        }

        // ================= history heat map =================
        // One column per clock step (oldest on the left, newest on the right),
        // one row per stage. Colour shows polarity, alpha scales with
        // magnitude, and the white left edge marks the samples that landed.
        const float cw = plotW / (float)ST_HN;
        for (int c = 0; c < ST_HN; ++c) {
            if (c < ST_HN - count) continue;
            int idx = ((head - ST_HN + c) % ST_HN + ST_HN) % ST_HN;
            float x = ST_PX0 + (float)c * cw;
            for (int r = 0; r < ST_N; ++r) {
                float v = hist[r][idx];
                float t = rack::math::clamp(v / 10.f, -1.f, 1.f);
                float m = std::pow(std::fabs(t), 0.8f);
                NVGcolor col = (t >= 0.f) ? ag::accent() : ag::warm();
                float y = ST_HEAT_Y + (float)r * ST_ROW_H;

                nvgBeginPath(vg);
                nvgRect(vg, x + 0.5f, y + 0.5f, cw - 1.f, ST_ROW_H - 1.f);
                nvgFillColor(vg, ag::withA(col, 0.10f + 0.85f * m));
                nvgFill(vg);

                if (upd[r][idx]) {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 0.5f, y + 0.5f, 1.f, ST_ROW_H - 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 110));
                    nvgFill(vg);
                }
            }
        }
        // newest column highlight
        {
            float x = ST_PX0 + (float)(ST_HN - 1) * cw;
            nvgBeginPath(vg);
            nvgRect(vg, x, ST_HEAT_Y - 0.5f, cw, ST_ROW_H * ST_N + 1.f);
            nvgStrokeColor(vg, ag::withA(ag::bright(), 0.25f + 0.65f * clockFlash));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // ================= header and footer readouts =================
        // Header: clock lamp, LEN/PROB, LOOP/SHIFT state, SLEW. Footer: OLD/
        // NEW markers and the step counter.
        {
            char buf[48];

            // clock lamp
            nvgBeginPath(vg);
            nvgCircle(vg, 8.f, 8.f, 2.5f);
            nvgFillColor(vg, ag::withA(ag::accent(), 0.25f + 0.75f * clockFlash));
            nvgFill(vg);

            snprintf(buf, sizeof(buf), "LEN %d  PROB %.0f%%", len, prob * 100.f);
            ag::text(vg, 15.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);

            ag::text(vg, ST_DISP_W / 2.f + 14.f, 8.f, loopOn ? "LOOP" : "SHIFT", 7.f,
                     loopOn ? ag::bright() : ag::dim(), ag::ALIGN_C, 0.5f);

            if (tau <= 0.f)       snprintf(buf, sizeof(buf), "SLEW OFF");
            else if (tau < 1.f)   snprintf(buf, sizeof(buf), "SLEW %.0fMS", tau * 1000.f);
            else                  snprintf(buf, sizeof(buf), "SLEW %.1fS", tau);
            ag::text(vg, ST_DISP_W - 6.f, 8.f, buf, 7.f, ag::warm(), ag::ALIGN_R, 0.5f);

            ag::text(vg, ST_PX0, ST_CAP_Y, "OLD", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "STEP %u", (unsigned) steps);
            ag::text(vg, ST_DISP_W / 2.f, ST_CAP_Y, buf, 7.f, ag::accent(), ag::ALIGN_C, 0.5f);
            ag::text(vg, ST_PX0 + plotW, ST_CAP_Y, "NEW", 7.f, ag::dim(), ag::ALIGN_R, 0.5f);
        }

        nvgRestore(vg);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
// Places the static panel, screws, live knob arcs, live display, the four
// side knobs, the CONTROL row (clock/reset inputs + loop/step buttons with
// LEDs + poly output) and the eight IN/OUT jack pairs.
struct StasisWidget : ModuleWidget {
    // Builds the widget tree.
    StasisWidget(Stasis* module) {
        setModule(module);
        box.size = Vec(ST_PANEL_W, ST_PANEL_H);

        addChild(new STPanelBackground());
        addChild(new STKnobArcs(module));

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new STDisplayWidget(module));  

        // Side knobs: SLEW, PROB, LENGTH, LEVEL
        for (int k = 0; k < 4; ++k) {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(ST_KNOB_X[k], ST_KNOB_Y[k]), module, k));
        }

        // Control row: inputs, buttons, output
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(ST_CTRL_X[0], ST_CTRL_Y), module, Stasis::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(ST_CTRL_X[1], ST_CTRL_Y), module, Stasis::RESET_INPUT));

        addParam(createParamCentered<LEDButton>(
            Vec(ST_CTRL_X[2], ST_CTRL_Y), module, Stasis::LOOP_PARAM));
        addChild(createLightCentered<MediumLight<ag::CyanLight>>(
            Vec(ST_CTRL_X[2], ST_CTRL_Y), module, Stasis::LOOP_LIGHT));

        addParam(createParamCentered<LEDButton>(
            Vec(ST_CTRL_X[3], ST_CTRL_Y), module, Stasis::STEP_PARAM));
        addChild(createLightCentered<MediumLight<ag::CyanLight>>(
            Vec(ST_CTRL_X[3], ST_CTRL_Y), module, Stasis::STEP_LIGHT));

        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(ST_CTRL_X[4], ST_CTRL_Y), module, Stasis::POLY_OUTPUT));

        // Channel inputs (upper row) and outputs (lower row)
        for (int i = 0; i < ST_N; ++i) {
            float cx = ST_CH_X0 + ST_CH_DX * (float)i;
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(cx, ST_IN_TOP + ST_CH_JACK_DY), module, Stasis::CH_INPUT + i));
            addOutput(createOutputCentered<DarkPJ301MPort>(
                Vec(cx, ST_OUT_TOP + ST_CH_JACK_DY), module, Stasis::CH_OUTPUT + i));
        }
    }
};

Model* modelStasis = createModel<Stasis, StasisWidget>("Stasis");