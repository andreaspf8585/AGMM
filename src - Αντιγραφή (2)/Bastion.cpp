#include "GitLoad.hpp"
#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <complex>
#include <memory>
#include <string>
#include <vector>
#include "Knob_arcs.hpp"

// ============================================================
// Bastion - Multi-Engine Filter Module
// Engines: Moog Ladder, TB-303 Diode, MS-20, SEM SVF,
//          Steiner-Parker, CEM3320, Roland OTA, EMS Diode,
//          EDP Wasp, Buchla LPG
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// Bastion is a single 16 HP filter module offering ten different classic
// filter topologies ("engines") selectable at runtime. The selected engine
// processes one summed audio input and produces LP, BP and HP outputs.
//
// File structure:
//   1. Engine enum / name / description tables.
//   2. DSP: per-engine state structs and processing functions, plus shared
//      helpers (one-pole LP, soft clippers, drive stage, zero-delay-feedback
//      prediction).
//   3. Panel layout constants (all positions in pixels for a 240x380 panel).
//   4. Bastion module class: parameter/port/light config and process().
//   5. UI helpers (palette, text, plates, screens, response estimation).
//   6. BastionPanel: static artwork plus a live "layer 1" overlay with the
//      response curve, scope, meters, engine name and readouts.
//   7. BastionEngineKnob: custom-drawn engine selector knob.
//   8. BastionWidget: places all widgets and the context menu.
//   9. Model registration.
//
// The design is flat fills and hairline strokes: no gradients are used for
// the panel art beyond the flat background, and the only "glow" is the
// per-engine accent colour used consistently for curve, scope, pointer, tick
// and engine name. Each engine also gets its own accent colour, which is what
// makes the module look different as you scroll through engines.
// ============================================================

// Engine IDs, in the order they appear on the selector knob.
enum BastionEngine
{
    ENGINE_MOOG = 0,
    ENGINE_TB303,
    ENGINE_MS20,
    ENGINE_SEM,
    ENGINE_STEINER,
    ENGINE_CEM3320,
    ENGINE_ROLAND_OTA,
    ENGINE_EMS_DIODE,
    ENGINE_WASP,
    ENGINE_BUCHLA_LPG,
    NUM_ENGINES
};

// Display names shown on the engine plate and in the context menu.
static const char *const ENGINE_NAMES[NUM_ENGINES] = {
    "Moog Ladder", "TB-303 Diode", "MS-20", "SEM SVF",
    "Steiner-Parker", "CEM3320", "Roland OTA",
    "EMS Diode", "EDP Wasp", "Buchla LPG"};

// Short technical descriptions shown under the engine name on the plate.
static const char *const ENGINE_DESCS[NUM_ENGINES] = {
    "4-POLE TRANSISTOR LADDER",
    "DIODE LADDER",
    "2-POLE + HP FEEDBACK",
    "2-POLE STATE VARIABLE",
    "2-POLE MULTIMODE",
    "4-POLE OTA LADDER",
    "4-POLE OTA CASCADE",
    "DIODE LADDER",
    "3-POLE DRIVEN",
    "VACTROL LOW-PASS GATE"};

// ---------- DSP ----------

// State shared by the ladder-style engines: four one-pole stages plus two
// extra states used by the MS-20 high-pass stages.
struct LadderState
{
    float s[4] = {0.f, 0.f, 0.f, 0.f};
    float h[2] = {0.f, 0.f}; // extra state used by the MS-20 high-pass stages
};

// One-pole low-pass with a zero-delay-feedback-friendly TPT form. `g` is the
// prewarped tan(pi*fc/fs) value; `state` is updated in place and the filtered
// output is returned.
static inline float onePoleLP(float in, float &state, float g)
{
    float G = g / (1.f + g);
    float v = (in - state) * G;
    float out = v + state;
    state = out + v;
    return out;
}

// Soft clippers that work in the +-5 V audio domain (linear for small signals)
// Tanh limiter scaled to `lim` volts; stays approximately linear for |v| << lim.
static inline float softClip(float v, float lim) { return lim * std::tanh(v / lim); }
// Saturator mapped to the +-5 V audio range.
static inline float sat5(float v) { return 5.f * std::tanh(v * 0.2f); }

// Drive stage: 0..1 -> 0..~25 dB of gain into an asymmetric tanh, with makeup
// gain compensation. Adds even + odd harmonics, getting harder as drive rises.
// `asym` adds a DC bias before the tanh, which increases even harmonics.
static inline float driveStage(float in, float d, float asym = 0.f)
{
    d = clamp(d, 0.f, 1.5f);
    float gain = std::exp(d * 2.9f); // 1x .. ~18x (more past 1.0)
    float bias = d * (0.10f + asym);
    float y = std::tanh(in * 0.2f * gain + bias) - std::tanh(bias);
    return y * 5.f / (1.f + d * 0.45f);
}

// Clamps a cutoff frequency into a safe range: at least 10 Hz and at most
// `frac` of the sample rate, to keep the tan() prewarp finite and stable.
static inline float fcLimit(float fc, float fs, float frac)
{
    return clamp(fc, 10.f, fs * frac);
}

// Zero-delay-feedback prediction for a chain of n one-pole stages with
// global negative feedback k. Returns the predicted output of the last stage,
// so the feedback can be applied without a unit-delay (stable up to high fc).
static inline float ladderPredict(const float *s, int n, float G, float x, float k)
{
    float a = 1.f - G;
    float S = 0.f, Gn = 1.f;
    for (int i = 0; i < n; ++i)
    {
        S = S * G + a * s[i];
        Gn *= G;
    }
    return (Gn * x + S) / (1.f + k * Gn);
}

// Shared 4-pole ladder engine. Used by Moog, TB-303 and EMS Diode; `diode`
// switches to the diode-ladder feedback saturation and slightly lower k.
static float processLadder(float in, LadderState &st, float fc, float res, float drive, float fs, bool diode)
{
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float k = clamp(res, 0.f, 1.f) * (diode ? 3.6f : 4.0f);
    float x = driveStage(in, drive, diode ? 0.08f : 0.f);
    float y4 = ladderPredict(st.s, 4, G, x, k);
    float fb = diode ? sat5(k * y4 * 0.9f) * 1.1f : sat5(k * y4);
    float u = x - fb;
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = onePoleLP(u, st.s[3], g);
    return u;
}

// MS-20 engine: two high-pass stages into two low-pass stages, with global
// negative feedback around the whole chain (the classic Korg topology).
// Uses an explicit ZDF prediction of the HP-HP-LP-LP cascade.
static float processMS20(float in, LadderState &st, float fc, float res, float drive, float fs)
{
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float x = driveStage(in, drive, 0.12f);
    float k = clamp(res, 0.f, 1.f) * 3.8f;
    float gHP = std::tan((float)M_PI * clamp(fcHz * 0.5f, 10.f, fs * 0.4f) / fs);
    float Gh = gHP / (1.f + gHP), ah = 1.f - Gh, a = 1.f - G;

    // zero-delay prediction of the output through HP-HP-LP-LP
    float P = G * G * ah * ah;
    float Ch = -ah * ah * st.h[0] - ah * st.h[1];
    float Cl = G * a * st.s[0] + a * st.s[1];
    float Q = G * G * Ch + Cl;
    float yEst = (P * x + Q) / (1.f + k * P);

    float hpIn = x - sat5(k * yEst);
    st.h[0] = onePoleLP(hpIn, st.h[0], gHP);
    float hpf = hpIn - st.h[0];
    st.h[1] = onePoleLP(hpf, st.h[1], gHP);
    hpf = hpf - st.h[1];
    float y = hpf;
    y = onePoleLP(y, st.s[0], g);
    y = onePoleLP(y, st.s[1], g);
    st.s[3] = y;
    return y;
}

// State for the state-variable filter (SEM engine): the three simultaneous
// outputs plus nothing else.
struct SVFState
{
    float lp = 0.f, bp = 0.f, hp = 0.f;
};

// SEM engine: a TPT/Chamberlin-style state-variable filter producing LP, BP
// and HP simultaneously. Resonance is mapped as damping q = 1 - res.
static void processSVF(float in, SVFState &st, float fc, float res, float drive, float fs)
{
    float fcHz = fcLimit(fc, fs, 0.2f);
    float f = 2.f * std::sin((float)M_PI * fcHz / fs);
    float q = clamp(1.f - res, 0.02f, 1.f);
    float x = driveStage(in, drive, 0.05f);
    float hp = x - st.lp - q * st.bp;
    float bp = softClip(f * hp + st.bp, 10.f); // analog-style limiting keeps resonance bounded
    float lp = softClip(f * bp + st.lp, 10.f);
    if (std::fabs(lp) < 1e-12f)
        lp = 0.f;
    if (std::fabs(bp) < 1e-12f)
        bp = 0.f;
    st.hp = hp;
    st.bp = bp;
    st.lp = lp;
}

// State for the Steiner-Parker engine: two one-pole stages plus the derived
// band-pass signal.
struct SPState
{
    float s1 = 0.f, s2 = 0.f, bp = 0.f;
};

// Steiner-Parker engine: a two-pole multimode core where the band-pass is
// formed as the difference of the input HP and the two LP stages, with
// feedback taken from the second stage.
static void processSteiner(float in, SPState &st, float fc, float res, float drive, float fs)
{
    float fcHz = fcLimit(fc, fs, 0.25f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float k = clamp(res, 0.f, 1.f) * 3.5f;
    float x = driveStage(in, drive, 0.06f);
    float fb = softClip(st.s2 * k * 0.9f, 5.f);
    float hp = x - st.s1 - fb;
    float lp1 = onePoleLP(hp, st.s1, g);
    float lp2 = onePoleLP(lp1, st.s2, g);
    st.bp = hp - lp1 - lp2;
}

// CEM3320 engine: 4-pole OTA ladder with a slightly softer feedback
// saturation and a final output saturator.
static float processCEM3320(float in, LadderState &st, float fc, float res, float drive, float fs)
{
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float k = clamp(res, 0.f, 1.f) * 3.9f;
    float x = driveStage(in, drive, 0.04f);
    float y4 = ladderPredict(st.s, 4, G, x, k);
    float u = x - sat5(k * y4 * 0.85f);
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = onePoleLP(u, st.s[3], g);
    return sat5(u * 1.1f);
}

// Roland OTA engine: 4-pole cascade with inter-stage saturation, which is
// what gives the OTA character its compressed resonance.
static float processRolandOTA(float in, LadderState &st, float fc, float res, float drive, float fs)
{
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float k = clamp(res, 0.f, 1.f) * 3.7f;
    float x = driveStage(in, drive, 0.02f);
    float y4 = ladderPredict(st.s, 4, G, x, k);
    float u = x - sat5(k * y4 * 0.9f);
    u = onePoleLP(u, st.s[0], g);
    u = sat5(u * 1.1f);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = sat5(u * 1.05f);
    u = onePoleLP(u, st.s[3], g);
    return u;
}

// EDP Wasp engine: a 3-pole driven ladder with extra drive and output
// saturation, giving the grittiest character of the set.
static float processWasp(float in, LadderState &st, float fc, float res, float drive, float fs)
{
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float k = clamp(res, 0.f, 1.f) * 4.2f;
    float x = driveStage(in, drive * 1.25f, 0.10f); // the Wasp is the grittiest engine
    float y3 = ladderPredict(st.s, 3, G, x, k);
    float u = sat5(x - sat5(k * y3));
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    return sat5(u * 1.2f);
}

// State for the Buchla low-pass gate: the lagging vactrol envelope and the
// one-pole low-pass state.
struct LPGState
{
    float vactrol = 0.f;
    float lp = 0.f;
};

// Buchla low-pass gate engine: `fc` (via the cutoff control) sets the vactrol
// target level, which lags with a slower release; the level then drives both
// a VCA and the low-pass cutoff. This emulates the combined VCA+LPG behaviour.
static float processLPG(float in, LPGState &st, float fc, float res, float drive, float fs)
{
    float level = clamp(fc / 1000.f, 0.f, 1.f);
    float target = level;
    float lag = 0.0015f + (1.f - level) * 0.03f;
    st.vactrol += (target - st.vactrol) * lag * (fs / 48000.f);
    if (st.vactrol < 0.f)
        st.vactrol = 0.f;
    if (st.vactrol > 1.f)
        st.vactrol = 1.f;
    float vca = driveStage(in, drive, 0.05f) * st.vactrol;
    float lpgFc = 200.f + st.vactrol * 12000.f;
    float g = std::tan((float)M_PI * clamp(lpgFc, 20.f, fs * 0.4f) / fs);
    st.lp = onePoleLP(vca, st.lp, g);
    return sat5(st.lp);
}

// ============================================================
// Panel layout (16 HP = 240 x 380 px)
//
//   y   0-15   screws
//   y  14      title
//   y  26-104  display: response | scope | level meters
//                rows: title (y+8) / plot (y+15..62) / axis labels (y+70)
//   y 111-157  engine knob + engine plate
//   y 166-280  3 cells: label / knob / readout / CV jack + attenuator
//   y 294      section: I / O
//   y 303-349  6 jack cells (3 inputs left, 3 outputs right)
//   y 368      maker tag (between the bottom screws)
//
// Every control lives inside its plate with clear margins; no label or
// readout is ever drawn over a widget or over a plot.
// ============================================================
static const float BS_W = RACK_GRID_WIDTH * 16; // 240
static const float BS_H = RACK_GRID_HEIGHT;     // 380

static const float BS_TITLE_Y = 14.f;
static const float BS_MAKER_Y = 368.f;

// ---- Display ----
static const float DISP_Y = 26.f;
static const float DISP_H = 78.f;
static const float RESP_X = 10.f, RESP_W = 140.f;
static const float SCOPE_X = 154.f, SCOPE_W = 50.f;
static const float METER_X = 208.f, METER_W = 22.f;

static const float DISP_TITLE_Y = 8.f; // relative to DISP_Y
static const float PLOT_TOP = 15.f;    // relative to DISP_Y
static const float PLOT_H = 47.f;
static const float DISP_AXIS_Y = 70.f; // relative to DISP_Y

static const float PLOT_Y0 = DISP_Y + PLOT_TOP;
static const float PLOT_Y1 = DISP_Y + PLOT_TOP + PLOT_H;
static const float RESP_PX0 = RESP_X + 20.f; // room for dB labels
static const float RESP_PX1 = RESP_X + RESP_W - 6.f;
static const float SCOPE_PX0 = SCOPE_X + 4.f;
static const float SCOPE_PX1 = SCOPE_X + SCOPE_W - 4.f;
static const float DB_MIN = -48.f, DB_MAX = 24.f;

// Maps a dB value to a y pixel inside the response plot (clamped to the plot).
static float dbToY(float db)
{
    float t = clamp((db - DB_MIN) / (DB_MAX - DB_MIN), 0.f, 1.f);
    return PLOT_Y1 - t * (PLOT_Y1 - PLOT_Y0);
}
// Maps a frequency in Hz to an x pixel inside the response plot using a
// 20 Hz..20 kHz logarithmic axis (each decade takes equal width).
static float hzToX(float f)
{
    return RESP_PX0 + std::log(std::max(f, 20.f) / 20.f) / std::log(1000.f) * (RESP_PX1 - RESP_PX0);
}

// ---- Engine strip ----
static const float ENG_X = 36.f;
static const float ENG_PLATE_X = 70.f, ENG_PLATE_Y = 111.f;
static const float ENG_PLATE_W = 160.f, ENG_PLATE_H = 46.f;
static const float ENG_Y = ENG_PLATE_Y + ENG_PLATE_H / 2.f; // 134

// ---- Main cells (cutoff / resonance / drive) ----
static const float CELL_W = 68.f, CELL_H = 114.f, CELL_TOP = 166.f;
static const float CELL_LABEL = 10.f; // offsets from the plate top
static const float CELL_KNOB = 36.f;
static const float CELL_READ = 65.f;
static const float CELL_JACK = 88.f;
static const float CELL_SUBLBL = 106.f;
static const float CELL_SUB_DX = 17.f; // CV jack left, attenuator right
// Horizontal centre of column i (0..2) for the three parameter cells.
static float colX(int i) { return 44.f + 76.f * i; } // 44, 120, 196

// ---- I/O ----
static const float IO_SEC_Y = 294.f;
static const float IO_TOP = 303.f, IO_H = 46.f, IO_W = 34.f;
static const float IO_LABEL = 10.f, IO_JACK = 30.f;
// Horizontal centre of I/O jack i (0..5): 3 inputs then 3 outputs.
static float ioX(int i) { return 27.f + 37.f * i; } // 27 .. 212

// Knob sweep (matches Rack's standard knobs)
static const float KNOB_MIN_ANGLE = -0.83f * (float)M_PI;
static const float KNOB_MAX_ANGLE = 0.83f * (float)M_PI;
// Converts a normalised knob position t (0..1) to a NanoVG angle
// (0 = +x axis), rotated so the sweep starts at the lower-left.
static float knobAngle(float t)
{ // t in 0..1 -> NanoVG angle (0 = +x axis)
    return -(float)M_PI / 2.f + KNOB_MIN_ANGLE + t * (KNOB_MAX_ANGLE - KNOB_MIN_ANGLE);
}

// ============================================================
// Bastion module: parameters, ports, lights, DSP state and process().
// Also holds the UI feedback values (display values, peaks, scope buffer)
// that the panel widget reads while drawing.
// ============================================================
struct Bastion : Module
{
    enum ParamId
    {
        ENGINE_PARAM,
        CUTOFF_PARAM,
        RES_PARAM,
        DRIVE_PARAM,
        CUTOFF_CV_PARAM,
        RES_CV_PARAM,
        DRIVE_CV_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        CUTOFF_INPUT,
        RES_INPUT,
        DRIVE_INPUT,
        LP_INPUT,
        BP_INPUT,
        HP_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        LP_OUTPUT,
        BP_OUTPUT,
        HP_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHT_ENGINE_A,
        LIGHT_ENGINE_B,
        LIGHTS_LEN
    };

    // Number of bins in the UI scope ring buffer.
    static const int SCOPE_N = 64;

    LadderState ladder;
    SVFState svf;
    SPState sp;
    LPGState lpg;

    int lastEngine = -1;
    float enginePulse = 0.f;

    // ---- Values shared with the UI (read-only from the widget) ----
    float dispCutoff = 632.f;
    float dispRes = 0.3f;
    float dispDrive = 0.f;
    float inPeak = 0.f;
    float outPeak = 0.f;
    float scopeMin[SCOPE_N] = {};
    float scopeMax[SCOPE_N] = {};
    int scopePos = 0;
    float scopeAccMin = 0.f, scopeAccMax = 0.f;
    int scopeCount = 0;

    // Constructor: configures all params, inputs, outputs, lights and the
    // bypass route (input 1 -> LP output).
    Bastion()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configSwitch(ENGINE_PARAM, 0.f, NUM_ENGINES - 1, 0.f, "Engine",
                     std::vector<std::string>(ENGINE_NAMES, ENGINE_NAMES + NUM_ENGINES));

        // Cutoff: 20 Hz * 1000^x, matching computeCutoff()
        configParam(CUTOFF_PARAM, 0.f, 1.f, 0.5f, "Cutoff", " Hz", 1000.f, 20.f);
        configParam(RES_PARAM, 0.f, 1.f, 0.3f, "Resonance", "%", 0.f, 100.f);
        configParam(DRIVE_PARAM, 0.f, 1.f, 0.f, "Drive", "%", 0.f, 100.f);
        configParam(CUTOFF_CV_PARAM, -1.f, 1.f, 0.f, "Cutoff CV amount", "%", 0.f, 100.f);
        configParam(RES_CV_PARAM, -1.f, 1.f, 0.f, "Resonance CV amount", "%", 0.f, 100.f);
        configParam(DRIVE_CV_PARAM, -1.f, 1.f, 0.f, "Drive CV amount", "%", 0.f, 100.f);

        configInput(CUTOFF_INPUT, "Cutoff CV");
        configInput(RES_INPUT, "Resonance CV");
        configInput(DRIVE_INPUT, "Drive CV");
        // The three signal inputs are summed into one filter input.
        configInput(LP_INPUT, "Input 1 (summed with inputs 2 and 3)");
        configInput(BP_INPUT, "Input 2 (summed with inputs 1 and 3)");
        configInput(HP_INPUT, "Input 3 (summed with inputs 1 and 2)");

        configOutput(LP_OUTPUT, "Low-pass");
        configOutput(BP_OUTPUT, "Band-pass");
        configOutput(HP_OUTPUT, "High-pass");

        configLight(LIGHT_ENGINE_A, "Engine change");
        configLight(LIGHT_ENGINE_B, "Signal activity");

        configBypass(LP_INPUT, LP_OUTPUT);
    }

    // Clears all DSP states and UI feedback values (used on reset and from
    // the context menu's "Reset State" item).
    void onReset() override
    {
        ladder = LadderState();
        svf = SVFState();
        sp = SPState();
        lpg = LPGState();
        lastEngine = -1;
        inPeak = outPeak = 0.f;
        for (int i = 0; i < SCOPE_N; ++i)
            scopeMin[i] = scopeMax[i] = 0.f;
    }

    // Maps the normalised cutoff knob (0..1) to Hz on a 20 Hz..20 kHz
    // exponential curve, then applies CV as an octave shift (0.2 V per volt
    // of CV into the CV amount), clamped to the audio range.
    float computeCutoff(float base, float cvAmt, float cv)
    {
        float norm = clamp(base, 0.f, 1.f);
        float hz = 20.f * std::pow(1000.f, norm);
        float cvShift = cvAmt * cv;
        hz *= std::pow(2.f, cvShift);
        return clamp(hz, 10.f, 20000.f);
    }

    // Main audio callback: reads controls + CV, runs the selected engine,
    // writes the three outputs, and updates the UI feedback (peaks and the
    // scope ring buffer).
    void process(const ProcessArgs &args) override
    {
        int engine = (int)std::round(params[ENGINE_PARAM].getValue());
        engine = clamp(engine, 0, NUM_ENGINES - 1);
        if (engine != lastEngine)
        {
            ladder = LadderState();
            svf = SVFState();
            sp = SPState();
            lpg = LPGState();
            lastEngine = engine;
            enginePulse = 1.f;
        }
        enginePulse *= 0.92f;

        float fs = args.sampleRate;

        float cvCut = 0.f, cvRes = 0.f, cvDrive = 0.f;
        if (inputs[CUTOFF_INPUT].isConnected())
            cvCut = inputs[CUTOFF_INPUT].getVoltage();
        if (inputs[RES_INPUT].isConnected())
            cvRes = inputs[RES_INPUT].getVoltage();
        if (inputs[DRIVE_INPUT].isConnected())
            cvDrive = inputs[DRIVE_INPUT].getVoltage();

        float cutoff = computeCutoff(
            params[CUTOFF_PARAM].getValue(),
            params[CUTOFF_CV_PARAM].getValue(),
            cvCut * 0.2f);

        float res = clamp(params[RES_PARAM].getValue() + params[RES_CV_PARAM].getValue() * cvRes * 0.2f, 0.f, 1.f);
        float drive = clamp(params[DRIVE_PARAM].getValue() + params[DRIVE_CV_PARAM].getValue() * cvDrive * 0.2f, 0.f, 1.f);

        dispCutoff = cutoff;
        dispRes = res;
        dispDrive = drive;

        float inL = 0.f, inB = 0.f, inH = 0.f;
        bool anyIn = false;
        if (inputs[LP_INPUT].isConnected())
        {
            inL = inputs[LP_INPUT].getVoltage();
            anyIn = true;
        }
        if (inputs[BP_INPUT].isConnected())
        {
            inB = inputs[BP_INPUT].getVoltage();
            anyIn = true;
        }
        if (inputs[HP_INPUT].isConnected())
        {
            inH = inputs[HP_INPUT].getVoltage();
            anyIn = true;
        }
        float input = anyIn ? (inL + inB + inH) : 0.f;

        float outLP = 0.f, outBP = 0.f, outHP = 0.f;

        switch (engine)
        {
        case ENGINE_MOOG:
        {
            outLP = processLadder(input, ladder, cutoff, res, drive, fs, false);
            outBP = ladder.s[1] - ladder.s[2];
            outHP = input - outLP - outBP;
        }
        break;
        case ENGINE_TB303:
        {
            outLP = processLadder(input, ladder, cutoff, res, drive, fs, true);
            outBP = ladder.s[1] - ladder.s[2];
            outHP = input - outLP;
        }
        break;
        case ENGINE_MS20:
        {
            outLP = processMS20(input, ladder, cutoff, res, drive, fs);
            outBP = ladder.s[0] - ladder.s[1];
            outHP = input - outLP;
        }
        break;
        case ENGINE_SEM:
        {
            processSVF(input, svf, cutoff, res, drive, fs);
            outLP = svf.lp;
            outBP = svf.bp;
            outHP = svf.hp;
        }
        break;
        case ENGINE_STEINER:
        {
            processSteiner(input, sp, cutoff, res, drive, fs);
            outLP = sp.s2;
            outBP = sp.bp;
            outHP = input - outLP - outBP;
        }
        break;
        case ENGINE_CEM3320:
        {
            outLP = processCEM3320(input, ladder, cutoff, res, drive, fs);
            outBP = ladder.s[1] - ladder.s[2];
            outHP = input - outLP;
        }
        break;
        case ENGINE_ROLAND_OTA:
        {
            outLP = processRolandOTA(input, ladder, cutoff, res, drive, fs);
            outBP = ladder.s[1] - ladder.s[2];
            outHP = input - outLP;
        }
        break;
        case ENGINE_EMS_DIODE:
        {
            outLP = processLadder(input, ladder, cutoff, res, drive, fs, true);
            outBP = ladder.s[1] - ladder.s[2];
            outHP = input - outLP;
        }
        break;
        case ENGINE_WASP:
        {
            outLP = processWasp(input, ladder, cutoff, res, drive, fs);
            outBP = ladder.s[0] - ladder.s[1];
            outHP = input - outLP;
        }
        break;
        case ENGINE_BUCHLA_LPG:
        {
            outLP = processLPG(input, lpg, cutoff, res, drive, fs);
            outBP = outLP * 0.7f;
            outHP = input - outLP;
        }
        break;
        }

        // Safety net: if any engine produced a non-finite value, reset all
        // states and output silence for this sample.
        if (!std::isfinite(outLP) || !std::isfinite(outBP) || !std::isfinite(outHP))
        {
            ladder = LadderState();
            svf = SVFState();
            sp = SPState();
            lpg = LPGState();
            outLP = outBP = outHP = 0.f;
        }
        outLP = clamp(outLP, -12.f, 12.f);
        outBP = clamp(outBP, -12.f, 12.f);
        outHP = clamp(outHP, -12.f, 12.f);

        outputs[LP_OUTPUT].setVoltage(outLP);
        outputs[BP_OUTPUT].setVoltage(outBP);
        outputs[HP_OUTPUT].setVoltage(outHP);

        // ---- UI feedback: meters ----
        float peakDecay = std::exp(-1.f / (fs * 0.18f));
        inPeak = std::max(std::fabs(input), inPeak * peakDecay);
        float outMax = std::max(std::fabs(outLP), std::max(std::fabs(outBP), std::fabs(outHP)));
        outPeak = std::max(outMax, outPeak * peakDecay);

        // ---- UI feedback: scope (LP output, ~30 ms window) ----
        // Accumulates min/max over a stride so each scope bin covers several
        // samples, giving a peak-preserving envelope of the LP output.
        int stride = std::max(1, (int)(fs * 0.03f / SCOPE_N));
        if (scopeCount == 0)
        {
            scopeAccMin = scopeAccMax = outLP;
        }
        scopeAccMin = std::min(scopeAccMin, outLP);
        scopeAccMax = std::max(scopeAccMax, outLP);
        if (++scopeCount >= stride)
        {
            scopeMin[scopePos] = scopeAccMin;
            scopeMax[scopePos] = scopeAccMax;
            scopePos = (scopePos + 1) % SCOPE_N;
            scopeCount = 0;
        }

        lights[LIGHT_ENGINE_A].setBrightness(enginePulse);
        lights[LIGHT_ENGINE_B].setBrightness(clamp(std::fabs(outLP) * 0.15f, 0.f, 1.f));
    }
};

// ============================================================
// UI helpers
// ============================================================
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
    // Primary accent colour (cyan) for headers and default highlights.
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }

    // Returns a copy of `c` with its alpha channel replaced by `a`.
    NVGcolor withAlpha(NVGcolor c, float a)
    {
        c.a = a;
        return c;
    }

    // One accent colour per engine (display curve, scope, pointer and name)
    NVGcolor engineColor(int e)
    {
        static const NVGcolor cols[NUM_ENGINES] = {
            nvgRGB(0x3d, 0xe0, 0xff), // Moog       - cyan
            nvgRGB(0xff, 0xb0, 0x3d), // TB-303     - amber
            nvgRGB(0xff, 0x5a, 0x6e), // MS-20      - red
            nvgRGB(0x50, 0xff, 0x90), // SEM        - green
            nvgRGB(0xb0, 0x5a, 0xff), // Steiner    - purple
            nvgRGB(0x5a, 0x9b, 0xff), // CEM3320    - blue
            nvgRGB(0xff, 0xe0, 0x4a), // Roland OTA - yellow
            nvgRGB(0xff, 0x8a, 0x5a), // EMS        - orange
            nvgRGB(0xe0, 0xff, 0x3d), // Wasp       - lime
            nvgRGB(0x3d, 0xff, 0xd0), // Buchla     - teal
        };
        return cols[clamp(e, 0, NUM_ENGINES - 1)];
    }

    // Loads the AGMM monospace font and makes it the current NanoVG font face.
    // Returns false if there is no window or the asset is missing.
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
                  int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 0.5f)
    {
        if (!setFont(vg))
            return;
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Text alignment presets local to this translation unit.
    const int ALIGN_L = NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE;
    const int ALIGN_C = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE;
    const int ALIGN_R = NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE;

    // Control plate; tint = output jacks
    void plate(NVGcontext *vg, float x, float y, float w, float h, bool tint = false)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 5.f);
        nvgFillColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 22) : nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 60) : nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Flat dark display screen with a rounded frame and faint hairline edge.
    void screen(NVGcontext *vg, float x, float y, float w, float h)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 4.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 38));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Translucent dark frame used to host a plot or scope, with a faint
    // cyan hairline border.
    void plotFrame(NVGcontext *vg, float x, float y, float w, float h)
    {
        nvgBeginPath(vg);
        nvgRect(vg, x, y, w, h);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Section title with hairlines that stop a measured distance from the text
    void sectionHeader(NVGcontext *vg, float y, const char *txt)
    {
        const float x1 = 10.f, x2 = BS_W - 10.f, cx = BS_W / 2.f;
        if (!setFont(vg))
            return;
        nvgFontSize(vg, 8.f);
        nvgTextLetterSpacing(vg, 2.f);
        float b[4];
        nvgTextBounds(vg, 0.f, 0.f, txt, NULL, b);
        float gap = (b[2] - b[0]) / 2.f + 6.f;

        drawText(vg, cx, y, txt, 8.f, colAccent(), ALIGN_C, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - gap, y);
        nvgMoveTo(vg, cx + gap, y);
        nvgLineTo(vg, x2, y);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Formats a frequency into `b` as "123 Hz" or "1.23 kHz".
    void fmtHz(char *b, size_t n, float hz)
    {
        if (hz >= 1000.f)
            snprintf(b, n, "%.2f kHz", hz / 1000.f);
        else
            snprintf(b, n, "%.0f Hz", hz);
    }

    // Approximate magnitude response (dB) of the LP output for each engine.
    // This is an analytic transfer-function estimate used only for drawing
    // the response curve; it is not used in the audio path.
    float responseDb(int engine, float f, float fc, float res)
    {
        typedef std::complex<float> C;
        float w = f / std::max(fc, 1.f);
        C s(0.f, w);
        C one(1.f, 0.f);
        C H(1.f, 0.f);
        switch (engine)
        {
        case ENGINE_MOOG:
        case ENGINE_TB303:
        case ENGINE_CEM3320:
        case ENGINE_ROLAND_OTA:
        case ENGINE_EMS_DIODE:
        {
            float kMax = (engine == ENGINE_MOOG) ? 4.0f : (engine == ENGINE_CEM3320)  ? 3.9f
                                                      : (engine == ENGINE_ROLAND_OTA) ? 3.7f
                                                                                      : 3.6f;
            float k = clamp(res, 0.f, 1.f) * kMax;
            C p = one + s;
            C p4 = p * p * p * p;
            H = one / (p4 + C(k, 0.f));
        }
        break;
        case ENGINE_WASP:
        {
            float k = clamp(res, 0.f, 1.f) * 4.2f;
            C p = one + s;
            C p3 = p * p * p;
            H = one / (p3 + C(k, 0.f));
        }
        break;
        case ENGINE_MS20:
        {
            float Q = 0.6f + clamp(res, 0.f, 1.f) * 10.f;
            H = one / (s * s + s * (1.f / Q) + one);
        }
        break;
        case ENGINE_SEM:
        {
            float q = clamp(1.f - res, 0.02f, 1.f);
            H = one / (s * s + s * q + one);
        }
        break;
        case ENGINE_STEINER:
        {
            float Q = 0.7f + clamp(res, 0.f, 1.f) * 6.f;
            H = one / (s * s + s * (1.f / Q) + one);
        }
        break;
        case ENGINE_BUCHLA_LPG:
        {
            float fcEff = 200.f + clamp(fc / 1000.f, 0.f, 1.f) * 12000.f;
            C s2(0.f, f / fcEff);
            H = one / (one + s2);
        }
        break;
        }
        float mag = std::abs(H);
        return 20.f * std::log10(mag + 1e-6f);
    }

} // namespace

// ============================================================
// Panel widget (static artwork + live displays)
// ============================================================
struct BastionPanel : TransparentWidget
{
    Bastion *bm = nullptr;

    BastionPanel() { box.size = Vec(BS_W, BS_H); }

    // ---------------- Static artwork ----------------

    // Draws the opaque background, title, maker tag and the static frames
    // for the display, engine strip, parameter cells and I/O row. This runs
    // on layer 0 and is only redrawn when the widget needs repainting.
    void draw(const DrawArgs &args) override
    {
        NVGcontext *vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, BS_W, BS_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, BS_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Title + maker
        drawText(vg, BS_W / 2.f, BS_TITLE_Y, "BASTION", 12.f, colText(), ALIGN_C, 4.f);
        drawText(vg, BS_W / 2.f, BS_MAKER_Y, "AGMM", 9.f, colDim(), ALIGN_C, 3.f);

        drawDisplayFrame(vg);
        drawEngineFrame(vg);
        drawCellFrames(vg);
        drawIoFrames(vg);

        Widget::draw(args);
    }

    // Draws the three screens (response, scope, meter) and their titles,
    // grids, axis labels and plot frames.
    void drawDisplayFrame(NVGcontext *vg)
    {
        screen(vg, RESP_X, DISP_Y, RESP_W, DISP_H);
        screen(vg, SCOPE_X, DISP_Y, SCOPE_W, DISP_H);
        screen(vg, METER_X, DISP_Y, METER_W, DISP_H);

        const float ty = DISP_Y + DISP_TITLE_Y;
        const float ay = DISP_Y + DISP_AXIS_Y;

        // Titles (above the plots)
        drawText(vg, RESP_X + 6.f, ty, "LP RESPONSE", 7.f, colDim(), ALIGN_L, 1.f);
        drawText(vg, SCOPE_X + 6.f, ty, "SCOPE", 7.f, colDim(), ALIGN_L, 1.f);
        drawText(vg, METER_X + METER_W / 2.f, ty, "LVL", 7.f, colDim(), ALIGN_C, 1.f);

        // ---- Response plot ----
        plotFrame(vg, RESP_PX0, PLOT_Y0, RESP_PX1 - RESP_PX0, PLOT_H);

        const float fGrid[3] = {100.f, 1000.f, 10000.f};
        const char *fLbl[3] = {"100", "1K", "10K"};
        nvgBeginPath(vg);
        for (int i = 0; i < 3; ++i)
        {
            float gx = hzToX(fGrid[i]);
            nvgMoveTo(vg, gx, PLOT_Y0);
            nvgLineTo(vg, gx, PLOT_Y1);
        }
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 16));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        for (int i = 0; i < 3; ++i)
            drawText(vg, hzToX(fGrid[i]), ay, fLbl[i], 7.f, colDim(), ALIGN_C, 0.f);

        // dB lines: 0 dB (brighter) and -24 dB, labelled in the left margin
        const float dbGrid[2] = {0.f, -24.f};
        const char *dbLbl[2] = {"0", "-24"};
        for (int i = 0; i < 2; ++i)
        {
            float gy = dbToY(dbGrid[i]);
            nvgBeginPath(vg);
            nvgMoveTo(vg, RESP_PX0, gy);
            nvgLineTo(vg, RESP_PX1, gy);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, i == 0 ? 34 : 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, RESP_PX0 - 3.f, gy, dbLbl[i], 7.f, colDim(), ALIGN_R, 0.f);
        }

        // ---- Scope plot ----
        plotFrame(vg, SCOPE_PX0, PLOT_Y0, SCOPE_PX1 - SCOPE_PX0, PLOT_H);
        float cy = (PLOT_Y0 + PLOT_Y1) / 2.f;
        nvgBeginPath(vg);
        nvgMoveTo(vg, SCOPE_PX0, cy);
        nvgLineTo(vg, SCOPE_PX1, cy);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        drawText(vg, SCOPE_X + SCOPE_W / 2.f, ay, "LP OUT", 7.f, colDim(), ALIGN_C, 0.5f);

        // ---- Meter tracks + labels ----
        const char *ml[2] = {"I", "O"};
        for (int i = 0; i < 2; ++i)
        {
            float x = METER_X + 4.5f + i * 8.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, PLOT_Y0, 5.f, PLOT_H);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgFill(vg);
            drawText(vg, x + 2.5f, ay, ml[i], 7.f, colDim(), ALIGN_C, 0.f);
        }
    }

    // Draws the engine selector tick marks and the engine name plate frame
    // with its static "SWITCH"/"SIGNAL" labels.
    void drawEngineFrame(NVGcontext *vg)
    {
        for (int i = 0; i < NUM_ENGINES; ++i)
        {
            float a = knobAngle(i / (float)(NUM_ENGINES - 1));
            nvgBeginPath(vg);
            nvgMoveTo(vg, ENG_X + std::cos(a) * 19.f, ENG_Y + std::sin(a) * 19.f);
            nvgLineTo(vg, ENG_X + std::cos(a) * 23.f, ENG_Y + std::sin(a) * 23.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 55));
            nvgStrokeWidth(vg, 1.2f);
            nvgStroke(vg);
        }

        plate(vg, ENG_PLATE_X, ENG_PLATE_Y, ENG_PLATE_W, ENG_PLATE_H);
        const float ly = ENG_PLATE_Y + 9.f;
        drawText(vg, ENG_PLATE_X + 100.f, ly, "SWITCH", 7.f, colDim(), ALIGN_R, 0.5f);
        drawText(vg, ENG_PLATE_X + 141.f, ly, "SIGNAL", 7.f, colDim(), ALIGN_R, 0.5f);
    }

    // Draws the three parameter cell frames with their titles and the
    // "CV"/"ATT" sub-labels.
    void drawCellFrames(NVGcontext *vg)
    {
        const char *titles[3] = {"CUTOFF", "RESONANCE", "DRIVE"};
        for (int i = 0; i < 3; ++i)
        {
            float cx = colX(i);
            plate(vg, cx - CELL_W / 2.f, CELL_TOP, CELL_W, CELL_H);
            drawText(vg, cx, CELL_TOP + CELL_LABEL, titles[i], 8.f, colText(), ALIGN_C, 0.5f);
            drawText(vg, cx - CELL_SUB_DX, CELL_TOP + CELL_SUBLBL, "CV", 7.f, colDim(), ALIGN_C, 0.5f);
            drawText(vg, cx + CELL_SUB_DX, CELL_TOP + CELL_SUBLBL, "ATT", 7.f, colDim(), ALIGN_C, 0.5f);
        }
    }

    // Draws the I/O section header and six jack cell frames: three inputs
    // (left) and three tinted outputs (right).
    void drawIoFrames(NVGcontext *vg)
    {
        sectionHeader(vg, IO_SEC_Y, "I / O");
        const char *names[6] = {"IN 1", "IN 2", "IN 3", "LP", "BP", "HP"};
        for (int i = 0; i < 6; ++i)
        {
            float cx = ioX(i);
            plate(vg, cx - IO_W / 2.f, IO_TOP, IO_W, IO_H, i >= 3);
            drawText(vg, cx, IO_TOP + IO_LABEL, names[i], 7.f, colDim(), ALIGN_C, 0.5f);
        }
    }

    // ---------------- Live (glowing) layer ----------------

    // Layer-1 hook: draws the live response curve, scope, meters, engine
    // plate and readouts on top of the static artwork, then forwards to the
    // base widget so child widgets are still drawn on other layers.
    void drawLayer(const DrawArgs &args, int layer) override
    {
        if (layer == 1)
            drawLive(args.vg);
        Widget::drawLayer(args, layer);
    }

    // Draws everything that changes with the module state: the response
    // curve, scope, meters, engine plate contents, engine tick highlight
    // and the three value readouts. Falls back to demo values when `bm` is
    // null (i.e. in the module browser).
    void drawLive(NVGcontext *vg)
    {
        // Pull values (or sensible demo values in the module browser)
        int engine = 0;
        float cutoff = 632.f, res = 0.3f, drive = 0.f;
        float inPk = 3.f, outPk = 4.f;
        if (bm)
        {
            engine = clamp((int)std::round(bm->params[Bastion::ENGINE_PARAM].getValue()), 0, NUM_ENGINES - 1);
            cutoff = bm->dispCutoff;
            res = bm->dispRes;
            drive = bm->dispDrive;
            inPk = bm->inPeak;
            outPk = bm->outPeak;
        }
        NVGcolor ec = engineColor(engine);

        drawResponse(vg, engine, cutoff, res, ec);
        drawScope(vg, ec);
        drawMeters(vg, inPk, outPk);
        drawEnginePlate(vg, engine, ec);

        // Engine tick highlight
        {
            float a = knobAngle(engine / (float)(NUM_ENGINES - 1));
            nvgBeginPath(vg);
            nvgMoveTo(vg, ENG_X + std::cos(a) * 18.f, ENG_Y + std::sin(a) * 18.f);
            nvgLineTo(vg, ENG_X + std::cos(a) * 24.f, ENG_Y + std::sin(a) * 24.f);
            nvgStrokeColor(vg, ec);
            nvgStrokeWidth(vg, 2.2f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStroke(vg);
        }

        // Value readouts (effective values, including CV)
        char buf[32];
        for (int i = 0; i < 3; ++i)
        {
            if (i == 0)
                fmtHz(buf, sizeof buf, cutoff);
            else if (i == 1)
                snprintf(buf, sizeof buf, "%.0f %%", res * 100.f);
            else
                snprintf(buf, sizeof buf, "%.0f %%", drive * 100.f);
            drawText(vg, colX(i), CELL_TOP + CELL_READ, buf, 7.f, colAccent(), ALIGN_C, 0.5f);
        }
    }

    // Draws the response curve for the current engine plus the cutoff
    // marker, clipped to the response plot rectangle. Uses responseDb() to
    // sample the analytic transfer function across the log frequency axis.
    void drawResponse(NVGcontext *vg, int engine, float fc, float res, NVGcolor ec)
    {
        const float x0 = RESP_PX0, x1 = RESP_PX1, w = x1 - x0;
        const float y0 = PLOT_Y0, y1 = PLOT_Y1;

        nvgSave(vg);
        nvgScissor(vg, x0, y0, w, y1 - y0);

        // cutoff marker
        float fcX = clamp(hzToX(fc), x0, x1);
        nvgBeginPath(vg);
        nvgMoveTo(vg, fcX, y0);
        nvgLineTo(vg, fcX, y1);
        nvgStrokeColor(vg, withAlpha(ec, 0.4f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // response curve (computed once, used for fill and stroke)
        const int N = 96;
        float px[N + 1], py[N + 1];
        for (int i = 0; i <= N; ++i)
        {
            float t = i / (float)N;
            float f = 20.f * std::pow(1000.f, t);
            px[i] = x0 + t * w;
            py[i] = dbToY(responseDb(engine, f, fc, res));
        }

        nvgBeginPath(vg);
        nvgMoveTo(vg, px[0], py[0]);
        for (int i = 1; i <= N; ++i)
            nvgLineTo(vg, px[i], py[i]);
        nvgLineTo(vg, x1, y1);
        nvgLineTo(vg, x0, y1);
        nvgClosePath(vg);
        nvgFillColor(vg, withAlpha(ec, 0.22f));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgMoveTo(vg, px[0], py[0]);
        for (int i = 1; i <= N; ++i)
            nvgLineTo(vg, px[i], py[i]);
        nvgStrokeColor(vg, ec);
        nvgStrokeWidth(vg, 1.4f);
        nvgLineJoin(vg, NVG_ROUND);
        nvgStroke(vg);

        nvgRestore(vg);
    }

    // Draws the scope as a filled min/max envelope from the module's ring
    // buffer, clipped to the scope plot. Uses a demo waveform when there is
    // no module (module browser).
    void drawScope(NVGcontext *vg, NVGcolor ec)
    {
        const int N = Bastion::SCOPE_N;
        const float x0 = SCOPE_PX0, x1 = SCOPE_PX1, w = x1 - x0;
        const float cy = (PLOT_Y0 + PLOT_Y1) / 2.f;
        const float amp = (PLOT_Y1 - PLOT_Y0) / 2.f - 1.f;

        nvgSave(vg);
        nvgScissor(vg, x0, PLOT_Y0, w, PLOT_Y1 - PLOT_Y0);

        float mn[Bastion::SCOPE_N], mx[Bastion::SCOPE_N];
        if (bm)
        {
            int pos = bm->scopePos;
            for (int i = 0; i < N; ++i)
            {
                int idx = (pos + i) % N;
                mn[i] = bm->scopeMin[idx];
                mx[i] = bm->scopeMax[idx];
            }
        }
        else
        {
            for (int i = 0; i < N; ++i)
            { // demo waveform for the module browser
                float v = 3.5f * std::sin(i * 0.45f);
                mn[i] = v - 0.3f;
                mx[i] = v + 0.3f;
            }
        }

        const float scale = 1.f / 6.f; // +-6 V full height
        nvgBeginPath(vg);
        for (int i = 0; i < N; ++i)
        {
            float x = x0 + w * i / (float)(N - 1);
            float y = cy - clamp(mx[i] * scale, -1.f, 1.f) * amp;
            if (i == 0)
                nvgMoveTo(vg, x, y);
            else
                nvgLineTo(vg, x, y);
        }
        for (int i = N - 1; i >= 0; --i)
        {
            float x = x0 + w * i / (float)(N - 1);
            float y = cy - clamp(mn[i] * scale, -1.f, 1.f) * amp;
            nvgLineTo(vg, x, y);
        }
        nvgClosePath(vg);
        nvgFillColor(vg, withAlpha(ec, 0.35f));
        nvgFill(vg);
        nvgStrokeColor(vg, ec);
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgRestore(vg);
    }

    // Draws the two vertical level meters (input "I" and output "O") as
    // flat fills rising from the bottom of each track. Turns red above 90%.
    void drawMeters(NVGcontext *vg, float inPk, float outPk)
    {
        const float h = PLOT_Y1 - PLOT_Y0;
        float levels[2] = {clamp(inPk / 10.f, 0.f, 1.f), clamp(outPk / 10.f, 0.f, 1.f)};
        for (int i = 0; i < 2; ++i)
        {
            float x = METER_X + 4.5f + i * 8.f;
            float lh = levels[i] * h;
            if (lh > 0.5f)
            {
                nvgBeginPath(vg);
                nvgRect(vg, x, PLOT_Y1 - lh, 5.f, lh);
                nvgFillColor(vg, levels[i] > 0.9f ? nvgRGB(0xff, 0x50, 0x50) : colAccent());
                nvgFill(vg);
            }
        }
    }

    // Counter, name and description (lights and their labels are static + widgets)
    // Draws the live engine plate contents: "ENGINE nn/NN", the engine name
    // in its accent colour, and the engine description underneath.
    void drawEnginePlate(NVGcontext *vg, int engine, NVGcolor ec)
    {
        char buf[24];
        snprintf(buf, sizeof buf, "ENGINE %02d/%02d", engine + 1, (int)NUM_ENGINES);
        drawText(vg, ENG_PLATE_X + 8.f, ENG_PLATE_Y + 9.f, buf, 7.f, colDim(), ALIGN_L, 0.5f);
        drawText(vg, ENG_PLATE_X + 8.f, ENG_PLATE_Y + 25.f, ENGINE_NAMES[engine], 11.f, ec, ALIGN_L, 0.8f);
        drawText(vg, ENG_PLATE_X + 8.f, ENG_PLATE_Y + 38.f, ENGINE_DESCS[engine], 7.f, colDim(), ALIGN_L, 0.5f);
    }
};

// ============================================================
// Engine selector knob - fully custom-drawn (no SVG required)
// Draws a flat knob body with an accent-coloured pointer; the colour comes
// from the currently selected engine.
// ============================================================
struct BastionEngineKnob : app::Knob
{
    // Fixed 30x30 px knob footprint.
    BastionEngineKnob()
    {
        box.size = Vec(30.f, 30.f);
    }

    // Paints the knob: drop shadow, radial-gradient body, inner cap, accent
    // pointer at the current engine's position, and a centre dot.
    void draw(const DrawArgs &args) override
    {
        NVGcontext *vg = args.vg;
        float cx = box.size.x / 2.f, cy = box.size.y / 2.f;
        float r = 14.f;

        float value = 0.f;
        if (engine::ParamQuantity *pq = getParamQuantity())
            value = pq->getValue();
        int engine = clamp((int)std::round(value), 0, NUM_ENGINES - 1);
        NVGcolor ec = engineColor(engine);

        // drop shadow
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy + 1.5f, r + 1.5f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 120));
        nvgFill(vg);

        // body
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx - 4.f, cy - 5.f, 1.f, r + 4.f,
                                           nvgRGB(0x4a, 0x50, 0x5c), nvgRGB(0x12, 0x14, 0x18)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 50));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // inner cap
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r * 0.62f);
        nvgFillPaint(vg, nvgLinearGradient(vg, cx, cy - r, cx, cy + r,
                                           nvgRGB(0x2a, 0x2e, 0x36), nvgRGB(0x0e, 0x10, 0x14)));
        nvgFill(vg);

        // pointer
        float t = value / (float)(NUM_ENGINES - 1);
        float a = knobAngle(clamp(t, 0.f, 1.f));
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + std::cos(a) * 3.f, cy + std::sin(a) * 3.f);
        nvgLineTo(vg, cx + std::cos(a) * 12.f, cy + std::sin(a) * 12.f);
        nvgStrokeColor(vg, ec);
        nvgStrokeWidth(vg, 2.4f);
        nvgLineCap(vg, NVG_ROUND);
        nvgStroke(vg);

        // centre dot
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, 1.6f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 70));
        nvgFill(vg);
    }
};

// ============================================================
// Module widget: creates the panel, screws and every control, and adds the
// context menu with an engine submenu and a "Reset State" item.
// ============================================================
struct BastionWidget : ModuleWidget
{
    // Places the panel, screws, engine knob/lights, the three parameter
    // cells (knob + CV jack + attenuator) and the six I/O jacks.
    BastionWidget(Bastion *module)
    {
        setModule(module);
        box.size = Vec(BS_W, BS_H); // 16 HP, panel is drawn entirely by BastionPanel

        BastionPanel *panel = new BastionPanel();
        panel->bm = module;
        addChild(panel);

        // ---- Screws ----
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // ---- Engine selector + indicator lights (on the engine plate) ----
        addParam(createParamCentered<BastionEngineKnob>(
            Vec(ENG_X, ENG_Y), module, Bastion::ENGINE_PARAM));
        addChild(createLightCentered<SmallLight<GreenLight>>(
            Vec(ENG_PLATE_X + 107.f, ENG_PLATE_Y + 9.f), module, Bastion::LIGHT_ENGINE_A));
        addChild(createLightCentered<SmallLight<GreenLight>>(
            Vec(ENG_PLATE_X + 148.f, ENG_PLATE_Y + 9.f), module, Bastion::LIGHT_ENGINE_B));

        // ---- Cutoff / resonance / drive: knob, then CV jack (left) + attenuator (right) ----
        const int knobIds[3] = {Bastion::CUTOFF_PARAM, Bastion::RES_PARAM, Bastion::DRIVE_PARAM};
        const int trimIds[3] = {Bastion::CUTOFF_CV_PARAM, Bastion::RES_CV_PARAM, Bastion::DRIVE_CV_PARAM};
        const int cvIds[3] = {Bastion::CUTOFF_INPUT, Bastion::RES_INPUT, Bastion::DRIVE_INPUT};
        for (int i = 0; i < 3; ++i)
        {
            float cx = colX(i);
            addParam(createParamCentered<RoundBlackKnob>(
                Vec(cx, CELL_TOP + CELL_KNOB), module, knobIds[i]));
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(cx - CELL_SUB_DX, CELL_TOP + CELL_JACK), module, cvIds[i]));
            addParam(createParamCentered<Trimpot>(
                Vec(cx + CELL_SUB_DX, CELL_TOP + CELL_JACK), module, trimIds[i]));
        }
        agmm::addKnobArcs(this, 6.f);

        // ---- I/O: IN 1, IN 2, IN 3 | LP, BP, HP ----
        const float jy = IO_TOP + IO_JACK;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(ioX(0), jy), module, Bastion::LP_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(ioX(1), jy), module, Bastion::BP_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(ioX(2), jy), module, Bastion::HP_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ioX(3), jy), module, Bastion::LP_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ioX(4), jy), module, Bastion::BP_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ioX(5), jy), module, Bastion::HP_OUTPUT));
    }

    // Adds the right-click menu: a separator, an "Engine" submenu with a
    // checkmark for the active engine, and a "Reset State" action.
    void appendContextMenu(Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        Bastion *module = dynamic_cast<Bastion *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);

        menu->addChild(createSubmenuItem("Engine", ENGINE_NAMES[clamp((int)std::round(module->params[Bastion::ENGINE_PARAM].getValue()), 0, NUM_ENGINES - 1)],
                                         [=](Menu *submenu)
                                         {
                                             for (int i = 0; i < NUM_ENGINES; ++i)
                                             {
                                                 submenu->addChild(createCheckMenuItem(ENGINE_NAMES[i], "", [=]()
                                                                                       { return (int)std::round(module->params[Bastion::ENGINE_PARAM].getValue()) == i; }, [=]()
                                                                                       { module->params[Bastion::ENGINE_PARAM].setValue((float)i); }));
                                             }
                                         }));

        menu->addChild(createMenuItem("Reset State", "", [=]()
                                      { module->onReset(); }));
    }
};

// ============================================================
// Module registration
// ============================================================
Model *modelBastion = createModel<Bastion, BastionWidget>("Bastion");