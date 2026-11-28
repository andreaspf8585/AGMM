#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include "Knob_arcs.hpp"

// =============================================================================
// MAGICBOX — a waveshaper with a live transfer curve, a harmonic bar display
// and two oscilloscope traces (input / output).
//
// Per channel:
//   in -> auto-level (divide by a slow input peak) -> drive -> soft clip to
//   [-1, 1] -> shaping function f(x) -> optional DC blocker -> rescale by the
//   same gain -> wet/dry mix -> output level -> soft limiter -> out
//
// The display mirrors exactly that chain: the transfer plot draws f() over the
// clipped input, the operating-point trail marks where the signal currently
// sits, the eight harmonic bars come from a DFT of f(sin) at the current drive,
// and the scopes show the captured input and output waveforms.
// =============================================================================

// ------------------------------------------------------------
// Layout (18 HP = 270 x 380)
// Every control sits in a plate with its label ABOVE it. Display rows:
//   header (function | status | trigger + time), then
//   left column: TRANSFER caption, plot, HARMONICS caption, bars, numbers
//   right column: IN caption, IN scope, OUT caption, OUT scope
// Captions and readouts have their own rows; nothing is drawn over a plot.
// ------------------------------------------------------------
static const int MB_NUM_FUNCS = 20;
static const int MB_COLS = 128;   // scope columns (min/max pairs)
static const int MB_TRAIL_N = 48; // transfer-plot trail

static const float MB_PANEL_W = RACK_GRID_WIDTH * 18; // 270
static const float MB_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display
static const float MB_DISP_W = 230.f;
static const float MB_DISP_H = 180.f;
static const float MB_DISP_X = (MB_PANEL_W - MB_DISP_W) / 2.f; // 20
static const float MB_DISP_Y = 26.f;
static const float MB_HDR_Y = 8.f;  // header text row centre
static const float MB_DIV_Y = 16.f; // divider under the header

// Transfer plot + harmonics (display-local)
static const float MB_CAP_TF_Y = 22.f;
static const float MB_TF_X0 = 6.f, MB_TF_Y0 = 28.f, MB_TF_S = 70.f;
static const float MB_CAP_HM_Y = 106.f;
static const float MB_HM_X0 = 6.f, MB_HM_W = 70.f;
static const float MB_HM_TOP = 114.f, MB_HM_BASE = 160.f, MB_HM_LBL_Y = 167.f;

// Scopes (display-local)
// Two identical frames on the right: IN on top, OUT below.
static const float MB_SC_X0 = 90.f, MB_SC_X1 = 222.f;
static const float MB_CAP_IN_Y = 22.f, MB_CAP_OUT_Y = 100.f;
static const float MB_SCI_Y0 = 28.f, MB_SCI_Y1 = 92.f;   // IN
static const float MB_SCO_Y0 = 106.f, MB_SCO_Y1 = 170.f; // OUT

// Controls: five knob plates (label +10, knob centre +32)
static const float MB_SECTION_CTRL = 220.f;
static const float MB_CTRL_TOP = 228.f, MB_CTRL_H = 52.f, MB_CTRL_W = 46.f;
static const float MB_KNOB_Y = MB_CTRL_TOP + 32.f;
static const float MB_KX[5] = {35.f, 85.f, 135.f, 185.f, 235.f};

// I/O: six jack plates (label +10, jack centre +29)
static const float MB_SECTION_IO = 292.f;
static const float MB_IO_TOP = 300.f, MB_IO_H = 44.f, MB_IO_W = 38.f;
static const float MB_IO_Y = MB_IO_TOP + 29.f;
static const float MB_IO_X[6] = {30.f, 72.f, 114.f, 156.f, 198.f, 240.f};

// ------------------------------------------------------------
// Shaping functions (shared by DSP and display)
// ------------------------------------------------------------
// Short names shown in the header and in the context menu.
static const char *const MB_NAMES[MB_NUM_FUNCS] = {
    "A^2", "A^3", "sin(A)", "cos(A)", "tan(A)", "sqrt(A)", "cbrt(A)", "exp(A)",
    "sinh(A)", "cosh(A)", "tanh(A)", "fold(A)", "T3(A)", "T5(A)", "|A|",
    "half(A)", "asin(A)", "atan(A)", "log(A)", "crush(A)"};

// One-line description of what each function does to the sound.
static const char *const MB_DESC[MB_NUM_FUNCS] = {
    "Square: rectifies, octave up",
    "Cube: odd harmonics, soft fuzz",
    "Sine: smooth saturation",
    "Cosine: inverted, octave doubling",
    "Tangent: bright mid boost",
    "Square root: gritty, expands quiet detail",
    "Cube root: milder expansion",
    "Exponential: asymmetric, even harmonics",
    "Hyperbolic sine: tight, expanding",
    "Hyperbolic cosine: rectifying, octave up",
    "Hyperbolic tangent: tube-style saturation",
    "Wavefolder: triangle fold",
    "Chebyshev 3rd: adds the 3rd harmonic",
    "Chebyshev 5th: adds the 5th harmonic",
    "Full-wave rectifier",
    "Half-wave rectifier",
    "Arcsine: pushes peaks outward",
    "Arctangent: soft clipper",
    "Logarithmic compressor",
    "Bit crusher: 9 levels"};

// Raw function on x in [-1, 1]
//
// Notes on the scaled arguments: sin/cos/tan and the inverse trig functions are
// fed half- or full-angle arguments so that the useful part of each curve fits
// the input range; the Chebyshev polynomials are the usual T3/T5.
static inline float mbRaw(float x, int f)
{
    switch (f)
    {
    case 0:
        // x^2: always positive -> full-wave rectification, octave up.
        return x * x;
    case 1:
        // x^3: odd symmetry, adds 3rd-order harmonics (soft fuzz).
        return x * x * x;
    case 2:
        // sin(pi/2 * x): smooth S-curve, saturates softly at +-1.
        return std::sin(1.5707963f * x);
    case 3:
        // cos(pi * x): even symmetry, rectifying with octave doubling.
        return std::cos(3.1415927f * x);
    case 4:
        // tan(1.2 * x): steeper than sin, bright mid emphasis.
        return std::tan(1.2f * x);
    case 5:
        // signed sqrt: keeps the sign, boosts quiet detail.
        return std::sqrt(std::fabs(x)) * (x < 0.f ? -1.f : 1.f);
    case 6:
        // cbrt: same idea but gentler.
        return std::cbrt(x);
    case 7:
        // exp: asymmetric (positive side grows, negative side shrinks).
        return std::exp(x);
    case 8:
        // sinh(2x): odd, expanding.
        return std::sinh(2.f * x);
    case 9:
        // cosh(2x): even, rectifying.
        return std::cosh(2.f * x);
    case 10:
        // tanh(3x): classic soft saturation.
        return std::tanh(3.f * x);
    case 11:
        // triangle wavefolder: asin(sin(2*pi*x)) folds the signal back on itself.
        return 0.63661977f * std::asin(rack::math::clamp(std::sin(6.2831853f * x), -1.f, 1.f));
    case 12:
        // Chebyshev T3: exactly the 3rd harmonic for a sine input.
        return 4.f * x * x * x - 3.f * x;
    case 13:
    {
        // Chebyshev T5: 5th harmonic.
        float x2 = x * x;
        return x * (16.f * x2 * x2 - 20.f * x2 + 5.f);
    }
    case 14:
        // |x|: full-wave rectifier.
        return std::fabs(x);
    case 15:
        // max(x, 0): half-wave rectifier.
        return std::max(x, 0.f);
    case 16:
        // asin: expands the ends, compresses the middle.
        return 0.63661977f * std::asin(rack::math::clamp(x, -1.f, 1.f));
    case 17:
        // atan(4x): gentle soft clipper.
        return std::atan(4.f * x);
    case 18:
        // signed log1p: logarithmic compressor.
        return std::log1p(8.f * std::fabs(x)) * (x < 0.f ? -1.f : 1.f);
    case 19:
        // 9-level quantiser (steps of 0.25): bit crusher.
        return std::round(4.f * x) * 0.25f;
    default:
        return x;
    }
}

// Every function is shifted so that f(0) = 0 (silence stays silent, no DC
// at rest) and scaled so that its largest excursion on [-1, 1] is exactly 1.
//
// Without the offset, exp() or cosh() would output a constant at rest; without
// the gain, each function would have a different loudness. Both are measured
// once on a 512-point grid and stored in a function-local static (thread-safe
// magic static, built on first use).
struct MBNorm
{
    float off[MB_NUM_FUNCS];
    float gain[MB_NUM_FUNCS];
    MBNorm()
    {
        for (int f = 0; f < MB_NUM_FUNCS; ++f)
        {
            off[f] = mbRaw(0.f, f);
            // Small floor so a theoretically zero range never divides by zero.
            float m = 1e-3f;
            for (int i = 0; i <= 512; ++i)
            {
                float x = -1.f + 2.f * (float)i / 512.f;
                m = std::max(m, std::fabs(mbRaw(x, f) - off[f]));
            }
            gain[f] = 1.f / m;
        }
    }
};

// Lazily-built shared table (one instance for the whole plugin).
static const MBNorm &mbNorm()
{
    static const MBNorm n;
    return n;
}

static inline float mbShape(float x, int f)
{
    const MBNorm &n = mbNorm();
    return (mbRaw(x, f) - n.off[f]) * n.gain[f];
}

// Keeps the function argument inside [-1, 1]; linear up to 0.9, then a smooth knee
//
// The knee is what makes DRIVE behave like a drive control: below ~0.9 the
// signal is untouched, above it the peaks are bent instead of clipped hard.
static inline float mbSoftClip(float x)
{
    float a = std::fabs(x);
    if (a <= 0.9f)
        return x;
    float y = 0.9f + 0.1f * std::tanh((a - 0.9f) * 10.f);
    return (x < 0.f) ? -y : y;
}

// Output limiter: transparent up to 8 V, then a soft knee that never passes 10 V
static inline float mbLimit(float x)
{
    float a = std::fabs(x);
    if (a <= 8.f)
        return x;
    float y = 8.f + 2.f * std::tanh((a - 8.f) * 0.5f);
    return (x < 0.f) ? -y : y;
}

// Rejects NaN/Inf and clamps to a sane voltage range at the input.
static inline float mbSanitize(float v)
{
    if (!std::isfinite(v))
        return 0.f;
    return rack::math::clamp(v, -30.f, 30.f);
}

// Time knob: 0..1 -> 5 ms .. 4 s per screen
//
// 800 = 4 s / 5 ms, so the mapping is exponential (constant ratio per unit of
// knob travel), which feels even across the whole range.
struct MBTimeQ : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override
    {
        float t = 0.005f * std::pow(800.f, getValue());
        char b[32];
        if (t < 0.1f)
            snprintf(b, sizeof(b), "%.1f ms", t * 1000.f);
        else if (t < 1.f)
            snprintf(b, sizeof(b), "%.0f ms", t * 1000.f);
        else
            snprintf(b, sizeof(b), "%.2f s", t);
        return std::string(b);
    }
};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct MagicBox : Module
{
    enum ParamId
    {
        FUNC_PARAM,
        TIME_PARAM,
        DRIVE_PARAM,
        MIX_PARAM,
        LEVEL_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        IN_L_INPUT,
        IN_R_INPUT,
        FUNC_CV_INPUT,
        TIME_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };
    enum CapState
    {
        CAP_RUN,  // filling columns
        CAP_ARMED // waiting for the next trigger
    };

    // Settings (saved)
    bool autoLevel = true; // normalise by the input envelope
    bool dcBlock = true;   // remove DC created by asymmetric functions

    // DSP state
    float peak[2] = {0.f, 0.f};
    // One-pole DC blocker state per channel (x = previous input, y = previous output).
    float dcX[2] = {0.f, 0.f}, dcY[2] = {0.f, 0.f};
    int funcCur = 0, funcPrev = 0;
    float xfade = 0.f;
    // Current operating point (normalised input, shaped output) for the plot.

    // Scope capture: channels 0 = IN L, 1 = IN R, 2 = OUT L, 3 = OUT R
    //
    // The capture is a sweep: each column stores the min/max of the samples
    // that fall into it, so the display is an envelope and never misses a peak.
    float scMin[4][MB_COLS] = {};
    float scMax[4][MB_COLS] = {};
    bool scValid[MB_COLS] = {};
    float accMin[4] = {}, accMax[4] = {};
    int capState = CAP_ARMED;
    int capCol = 0;
    float capCount = 0.f;
    float holdTimer = 0.f;
    bool trigLow = false;
    bool lastTrig = false;
    float sweepScale = 0.02f;

    // Display data
    float vTime = 0.02f, vDrive = 1.f, vMix = 1.f;
    float peakIn = 0.f, peakOut = 0.f, limitFlash = 0.f;
    bool vRConn = false, vLConn = false;
    float opN = 0.f, opY = 0.f;
    // Recent operating points, sampled at ~90 Hz, for the dot trail.
    float trN[MB_TRAIL_N] = {}, trY[MB_TRAIL_N] = {};
    int trHead = 0, trCount = 0;
    float trTimer = 0.f;

    MagicBox()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        // The function list doubles as the switch's display names.
        std::vector<std::string> names;
        for (int i = 0; i < MB_NUM_FUNCS; ++i)
            names.push_back(MB_NAMES[i]);
        configSwitch(FUNC_PARAM, 0.f, (float)(MB_NUM_FUNCS - 1), 2.f, "Function", names);

        configParam<MBTimeQ>(TIME_PARAM, 0.f, 1.f, 0.22f, "Time scale");
        // DRIVE is exponential: -2..+2 maps to 1/4x .. 4x.
        configParam(DRIVE_PARAM, -2.f, 2.f, 0.f, "Drive", "x", 2.f, 1.f);
        configParam(MIX_PARAM, 0.f, 1.f, 1.f, "Dry / wet", "%", 0.f, 100.f);
        configParam(LEVEL_PARAM, 0.f, 1.5f, 1.f, "Output level", "%", 0.f, 100.f);

        configInput(IN_L_INPUT, "Left / mono");
        configInput(IN_R_INPUT, "Right (normalled to left)");
        configInput(FUNC_CV_INPUT, "Function CV (0-10 V sweeps all functions)");
        configInput(TIME_CV_INPUT, "Time CV (0-10 V sweeps the full range)");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");

        configBypass(IN_L_INPUT, OUT_L_OUTPUT);
        configBypass(IN_R_INPUT, OUT_R_OUTPUT);
    }

    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        autoLevel = true;
        dcBlock = true;
        peak[0] = peak[1] = 0.f;
        dcX[0] = dcX[1] = dcY[0] = dcY[1] = 0.f;
        xfade = 0.f;
        std::memset(scMin, 0, sizeof(scMin));
        std::memset(scMax, 0, sizeof(scMax));
        std::memset(scValid, 0, sizeof(scValid));
        // Back to "waiting for trigger" so the display restarts cleanly.
        capState = CAP_ARMED;
        capCol = 0;
        capCount = 0.f;
        holdTimer = 0.f;
        trHead = trCount = 0;
        limitFlash = 0.f;
    }

    // ---------------- scope capture ----------------
    // Clears the screen and starts filling columns from column 0.
    void startSweep(const float *c)
    {
        capState = CAP_RUN;
        capCol = 0;
        capCount = 0.f;
        for (int i = 0; i < 4; ++i)
            accMin[i] = accMax[i] = c[i];
    }

    // c[0..3] = IN L, IN R, OUT L, OUT R (volts); trigV = trigger source
    //
    // A sweep is started either by a rising edge through zero (after going
    // negative) or by a timeout, so a silent input still produces a trace
    // instead of a frozen screen.
    void scopeStep(const float *c, float trigV, float dt, float fs, float timeScale)
    {
        // A big change of the time knob restarts the sweep with a clean screen
        if (std::fabs(timeScale - sweepScale) > 0.06f * sweepScale)
        {
            sweepScale = timeScale;
            std::memset(scValid, 0, sizeof(scValid));
            startSweep(c);
        }

        if (capState == CAP_ARMED)
        {
            holdTimer += dt;
            if (trigV < -0.05f)
                trigLow = true;
            bool trig = trigLow && trigV > 0.f;
            // Time-based fallback: shorter at fast time scales.
            float timeout = 0.03f + 0.1f * timeScale;
            if (trig || holdTimer > timeout)
            {
                lastTrig = trig;
                trigLow = false;
                startSweep(c);
            }
            else
            {
                return;
            }
        }

        // Track min/max for the current column.
        for (int i = 0; i < 4; ++i)
        {
            accMin[i] = std::min(accMin[i], c[i]);
            accMax[i] = std::max(accMax[i], c[i]);
        }
        // Samples per column; at least 0.05 so very fast sweeps still advance.
        float spc = std::max(timeScale * fs / (float)MB_COLS, 0.05f); // samples per column
        capCount += 1.f;
        while (capCount >= spc && capCol < MB_COLS)
        {
            for (int i = 0; i < 4; ++i)
            {
                scMin[i][capCol] = accMin[i];
                scMax[i][capCol] = accMax[i];
            }
            scValid[capCol] = true;
            capCol++;
            capCount -= spc;
            for (int i = 0; i < 4; ++i)
                accMin[i] = accMax[i] = c[i];
        }
        if (capCol >= MB_COLS)
        {
            // Screen full: stop and wait for the next trigger.
            capState = CAP_ARMED;
            holdTimer = 0.f;
        }
    }

    // ---------------- audio ----------------
    void process(const ProcessArgs &args) override
    {
        const float fs = args.sampleRate;
        const float dt = args.sampleTime;

        // ---- parameters + CV ----
        // Function CV sweeps all 20 functions over 0..10 V.
        float fv = params[FUNC_PARAM].getValue() + inputs[FUNC_CV_INPUT].getVoltage() * 0.1f * (float)(MB_NUM_FUNCS - 1);
        int func = rack::math::clamp((int)std::lround(fv), 0, MB_NUM_FUNCS - 1);
        if (func != funcCur)
        {
            funcPrev = funcCur;
            funcCur = func;
            xfade = 1.f; // short crossfade: no click when the function changes
        }

        // Time CV sweeps the full 5 ms .. 4 s range.
        float tp = rack::math::clamp(
            params[TIME_PARAM].getValue() + inputs[TIME_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        float timeScale = 0.005f * std::pow(800.f, tp);

        // exp2 maps -2..+2 to 0.25x..4x.
        float drive = std::exp2(params[DRIVE_PARAM].getValue());
        float mix = params[MIX_PARAM].getValue();
        float level = params[LEVEL_PARAM].getValue();

        // ---- inputs ----
        bool lConn = inputs[IN_L_INPUT].isConnected();
        bool rConn = inputs[IN_R_INPUT].isConnected();
        float in[2];
        in[0] = mbSanitize(inputs[IN_L_INPUT].getVoltage());
        in[1] = rConn ? mbSanitize(inputs[IN_R_INPUT].getVoltage()) : in[0];

        // Peak follower decay (~0.4 s) and DC-blocker coefficient (8 Hz).
        const float peakDecay = std::exp(-dt / 0.4f);
        const float dcR = rack::math::clamp(1.f - 6.2831853f * 8.f / fs, 0.9f, 0.99999f);

        float outv[2];
        bool limited = false;

        for (int ch = 0; ch < 2; ++ch)
        {
            float v = in[ch];
            float x = v * 0.2f; // 5 V -> 1

            float g = 1.f;
            if (autoLevel)
            {
                // Slow peak follower; the 0.02 floor prevents amplifying hiss
                // when the input is silent.
                peak[ch] = std::max(std::fabs(x), peak[ch] * peakDecay);
                g = std::max(peak[ch], 0.02f); // noise floor: never amplify hiss
            }
            float n = x / g;                  // normalised input
            float xs = mbSoftClip(n * drive); // always inside [-1, 1]

            float ys = mbShape(xs, funcCur);
            float w = ys;
            // Crossfade from the previous function over ~8 ms.
            if (xfade > 0.f)
                w += (mbShape(xs, funcPrev) - w) * xfade;

            if (dcBlock)
            {
                // Standard one-pole high-pass: y = w - x_prev + R * y_prev.
                float y = w - dcX[ch] + dcR * dcY[ch];
                dcX[ch] = w;
                dcY[ch] = y;
                w = y;
            }
            else
            {
                // Keep the state coherent in case DC blocking is switched on later.
                dcX[ch] = w;
                dcY[ch] = 0.f;
            }

            // Undo the auto-level gain: the output level follows the input.
            float wet = w * g * 5.f;
            float o = (v + (wet - v) * mix) * level;
            if (std::fabs(o) > 8.f)
                limited = true;
            o = mbLimit(o);
            if (!std::isfinite(o))
            { // last line of defence
                o = 0.f;
                dcX[ch] = dcY[ch] = 0.f;
                peak[ch] = 0.f;
            }
            outv[ch] = o;

            if (ch == 0)
            {
                // Operating point used by the transfer plot (left channel).
                opN = n;
                opY = ys;
            }
        }
        // 8 ms crossfade ramp.
        xfade = std::max(0.f, xfade - dt / 0.008f);

        outputs[OUT_L_OUTPUT].setVoltage(outv[0]);
        outputs[OUT_R_OUTPUT].setVoltage(outv[1]);

        // ---- scope ----
        float c[4] = {in[0], in[1], outv[0], outv[1]};
        // Trigger on the left input when it is patched, otherwise on the right.
        scopeStep(c, lConn ? in[0] : in[1], dt, fs, timeScale);

        // ---- display data ----
        vTime = timeScale;
        vDrive = drive;
        vMix = mix;
        vLConn = lConn;
        vRConn = rConn;

        // Slower peak decay (~0.6 s) for the readable PK numbers.
        float pk = std::exp(-dt / 0.6f);
        peakIn = std::max(std::max(std::fabs(in[0]), std::fabs(in[1])), peakIn * pk);
        peakOut = std::max(std::max(std::fabs(outv[0]), std::fabs(outv[1])), peakOut * pk);
        limitFlash = limited ? 1.f : std::max(0.f, limitFlash - dt * 3.f);

        // Transfer-plot trail: ~90 points per second.
        trTimer += dt;
        const float trStep = 1.f / 90.f;
        if (trTimer >= trStep)
        {
            trTimer -= trStep;
            if (trTimer > trStep)
                trTimer = 0.f;
            trN[trHead] = opN;
            trY[trHead] = opY;
            trHead = (trHead + 1) % MB_TRAIL_N;
            if (trCount < MB_TRAIL_N)
                trCount++;
        }
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "autoLevel", json_boolean(autoLevel));
        json_object_set_new(root, "dcBlock", json_boolean(dcBlock));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *j = json_object_get(root, "autoLevel");
        if (j)
            autoLevel = json_is_true(j);
        j = json_object_get(root, "dcBlock");
        if (j)
            dcBlock = json_is_true(j);
    }
};

// ------------------------------------------------------------
// UI (anonymous namespace: names cannot clash with other module files)
// ------------------------------------------------------------
namespace
{

    // Knob → parameter mapping, in visual order, with the label drawn above.
    struct MBKnob
    {
        int param;
        const char *label;
    };
    const MBKnob MB_KNOBS[5] = {
        {MagicBox::FUNC_PARAM, "FUNCTION"},
        {MagicBox::TIME_PARAM, "TIME"},
        {MagicBox::DRIVE_PARAM, "DRIVE"},
        {MagicBox::MIX_PARAM, "MIX"},
        {MagicBox::LEVEL_PARAM, "LEVEL"},
    };

    // Draws a dashed segment. Used for the identity line and for the two
    // dashed guides that connect the live operating point to the axes.
    void dashedLine(NVGcontext *vg, float x0, float y0, float x1, float y1,
                    float dash, float gap, NVGcolor col)
    {
        float dx = x1 - x0, dy = y1 - y0;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-3f)
            return;
        float ux = dx / len, uy = dy / len;
        nvgBeginPath(vg);
        for (float t = 0.f; t < len; t += dash + gap)
        {
            float t2 = std::min(t + dash, len);
            nvgMoveTo(vg, x0 + ux * t, y0 + uy * t);
            nvgLineTo(vg, x0 + ux * t2, y0 + uy * t2);
        }
        nvgStrokeColor(vg, col);
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // ------------------------------------------------------------
    // Panel background
    //
    // Pure artwork: panel base, the display bezel, the five knob plates and
    // the six jack plates. No module state is read here.
    // ------------------------------------------------------------
    struct MBPanelBackground : TransparentWidget
    {
        MBPanelBackground() { box.size = Vec(MB_PANEL_W, MB_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, MB_PANEL_W, MB_PANEL_H, "MAGICBOX");
            agmm::bezel(vg, MB_DISP_X, MB_DISP_Y, MB_DISP_W, MB_DISP_H);

            // ---- controls ----
            agmm::sectionHeader(vg, MB_PANEL_W, MB_SECTION_CTRL, "CONTROL", 12.f);
            for (int k = 0; k < 5; ++k)
                agmm::cell(vg, MB_KX[k], MB_CTRL_TOP, MB_CTRL_W, MB_CTRL_H, MB_KNOBS[k].label, false, 7.f);

            // ---- I/O: inputs plain, outputs tinted ----
            agmm::sectionHeader(vg, MB_PANEL_W, MB_SECTION_IO, "I / O", 12.f);
            const char *io[6] = {"IN L", "IN R", "FN CV", "TM CV", "OUT L", "OUT R"};
            for (int i = 0; i < 6; ++i)
                agmm::cell(vg, MB_IO_X[i], MB_IO_TOP, MB_IO_W, MB_IO_H, io[i], i >= 4, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: transfer curve, harmonics, IN / OUT scopes
    //
    // Layer 0: screen, frames, grids, captions and static numbers.
    // Layer 1: transfer curve + trail + live point, harmonic bars, scope traces.
    // ------------------------------------------------------------
    struct MBDisplayWidget : TransparentWidget
    {
        MagicBox *module = nullptr;

        // harmonic cache
        // Recomputed only when the function or the drive changes.
        int hFunc = -1;
        float hDrive = -1.f;
        float hMag[8] = {};

        MBDisplayWidget(MagicBox *m) : module(m)
        {
            box.pos = Vec(MB_DISP_X, MB_DISP_Y);
            box.size = Vec(MB_DISP_W, MB_DISP_H);
        }

        // Half-height inside a scope frame, with one pixel of inset.
        static float scopeHH(float y0, float y1) { return (y1 - y0) * 0.5f - 1.5f; }

        // ---- static scope frame + grid ----
        void scopeFrame(NVGcontext *vg, float x0, float y0, float x1, float y1)
        {
            float w = x1 - x0, yc = (y0 + y1) * 0.5f, hh = scopeHH(y0, y1);

            agmm::plotFrame(vg, x0, y0, w, y1 - y0);

            // Vertical quarter lines.
            for (int i = 1; i < 4; ++i)
            {
                float x = x0 + w * (float)i / 4.f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x + 0.5f, y0 + 1.f);
                nvgLineTo(vg, x + 0.5f, y1 - 1.f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            // Horizontal lines at +5 V, 0 V and -5 V (full scale here is 10 V).
            const float vs[3] = {5.f, 0.f, -5.f};
            for (int i = 0; i < 3; ++i)
            {
                float y = yc - vs[i] / 10.f * hh;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x0 + 1.f, y + 0.5f);
                nvgLineTo(vg, x1 - 1.f, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, vs[i] == 0.f ? 44 : 14));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            // limiter threshold (+-8 V)
            for (int s = -1; s <= 1; s += 2)
            {
                float y = yc - (float)s * 0.8f * hh;
                dashedLine(vg, x0 + 1.f, y + 0.5f, x1 - 1.f, y + 0.5f, 2.f, 3.f,
                           nvgRGBA(0xff, 0x9c, 0x6a, 46));
            }
        }

        // ---- static background ----
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            agmm::screen(vg, 0.f, 0.f, MB_DISP_W, MB_DISP_H);

            // Divider under the header row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, MB_DIV_Y + 0.5f);
            nvgLineTo(vg, MB_DISP_W, MB_DIV_Y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Transfer plot frame
            // x runs -1..1 (normalised input), y runs -1..1 (shaped output).
            {
                const float X0 = MB_TF_X0, Y0 = MB_TF_Y0, S = MB_TF_S;
                agmm::plotFrame(vg, X0, Y0, S, S);

                // Quarter grid, with a brighter centre cross.
                for (int i = 1; i < 4; ++i)
                {
                    float t = (float)i / 4.f * S;
                    bool mid = (i == 2);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, X0 + t + 0.5f, Y0 + 1.f);
                    nvgLineTo(vg, X0 + t + 0.5f, Y0 + S - 1.f);
                    nvgMoveTo(vg, X0 + 1.f, Y0 + t + 0.5f);
                    nvgLineTo(vg, X0 + S - 1.f, Y0 + t + 0.5f);
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, mid ? 40 : 13));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                // identity (no processing)
                dashedLine(vg, X0 + 1.f, Y0 + S - 1.f, X0 + S - 1.f, Y0 + 1.f, 3.f, 3.f,
                           nvgRGBA(255, 255, 255, 34));
            }

            // Harmonic grid and number row
            // Four horizontal lines = 0, 12, 24, 36 dB below full scale.
            {
                float hgt = MB_HM_BASE - MB_HM_TOP;
                for (int i = 0; i <= 4; ++i)
                {
                    float y = MB_HM_TOP + hgt * (float)i / 4.f;
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, MB_HM_X0, y + 0.5f);
                    nvgLineTo(vg, MB_HM_X0 + MB_HM_W, y + 0.5f);
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, i == 4 ? 40 : 12));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                // Harmonic numbers 1..8 under the bars.
                float pitch = MB_HM_W / 8.f;
                for (int k = 0; k < 8; ++k)
                {
                    char b[4];
                    snprintf(b, sizeof(b), "%d", k + 1);
                    agmm::text(vg, MB_HM_X0 + ((float)k + 0.5f) * pitch, MB_HM_LBL_Y, b, 7.f,
                               agmm::dim(), agmm::ALIGN_C, 0.f);
                }
            }

            // Captions (own rows above their plots)
            agmm::text(vg, MB_TF_X0, MB_CAP_TF_Y, "TRANSFER", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
            agmm::text(vg, MB_TF_X0, MB_CAP_HM_Y, "HARMONICS", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);

            scopeFrame(vg, MB_SC_X0, MB_SCI_Y0, MB_SC_X1, MB_SCI_Y1);
            scopeFrame(vg, MB_SC_X0, MB_SCO_Y0, MB_SC_X1, MB_SCO_Y1);

            Widget::draw(args);
        }

        // ---- harmonic analysis of a full-scale sine through the shaper ----
        //
        // A 128-point DFT is evaluated only at the first eight harmonics, which
        // is all the display needs. Cheaper than a full FFT and exact at the
        // bins we care about.
        void updateHarmonics(int func, float drive)
        {
            if (func == hFunc && std::fabs(drive - hDrive) < 1e-4f)
                return;
            hFunc = func;
            hDrive = drive;
            const int N = 128;
            float re[8] = {}, im[8] = {};
            for (int i = 0; i < N; ++i)
            {
                float th = 6.2831853f * (float)i / (float)N;
                float y = mbShape(mbSoftClip(drive * std::sin(th)), func);
                for (int k = 0; k < 8; ++k)
                {
                    re[k] += y * std::cos((float)(k + 1) * th);
                    im[k] += y * std::sin((float)(k + 1) * th);
                }
            }
            for (int k = 0; k < 8; ++k)
                hMag[k] = 2.f * std::sqrt(re[k] * re[k] + im[k] * im[k]) / (float)N;
        }

        // ---- one scope ----
        //
        // Draws channel chL (and chR when showR) as a min/max envelope per
        // column. `dimOld` + `cursor` split the screen into "already swept"
        // (bright) and "previous sweep" (dim), which makes the trigger visible.
        void drawScope(NVGcontext *vg, float x0, float y0, float x1, float y1,
                       float (*mn)[MB_COLS], float (*mx)[MB_COLS], const bool *valid,
                       int chL, int chR, bool showR, int cursor, bool dimOld)
        {
            const float w = x1 - x0, yc = (y0 + y1) * 0.5f, hh = scopeHH(y0, y1);

            nvgSave(vg);
            nvgScissor(vg, x0, y0, w, y1 - y0);

            // Volts -> pixels, clamped slightly beyond the +-10 V full scale.
            auto yMap = [&](float v)
            {
                v = rack::math::clamp(v, -10.5f, 10.5f);
                return yc - v / 10.f * hh;
            };
            auto xMap = [&](int i)
            { return x0 + ((float)i + 0.5f) / (float)MB_COLS * w; };

            auto trace = [&](int ch, int a, int b, NVGcolor col, float alpha)
            {
                a = std::max(a, 0);
                b = std::min(b, MB_COLS);
                if (b - a < 2)
                    return;

                // envelope
                // Fill the band between max and min: one closed path, max going
                // left-to-right and min coming back.
                int n = 0;
                nvgBeginPath(vg);
                for (int i = a; i < b; ++i)
                {
                    if (!valid[i])
                        continue;
                    float x = xMap(i), y = yMap(mx[ch][i]);
                    if (!n)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                    n++;
                }
                if (n < 2)
                    return;
                for (int i = b - 1; i >= a; --i)
                {
                    if (!valid[i])
                        continue;
                    nvgLineTo(vg, xMap(i), yMap(mn[ch][i]));
                }
                nvgClosePath(vg);
                nvgFillColor(vg, agmm::withA(col, 0.16f * alpha));
                nvgFill(vg);

                // Two crisp outlines: top edge (max) and bottom edge (min).
                nvgLineCap(vg, NVG_ROUND);
                nvgLineJoin(vg, NVG_ROUND);
                for (int pass = 0; pass < 2; ++pass)
                {
                    float (*arr)[MB_COLS] = (pass == 0) ? mx : mn;
                    nvgBeginPath(vg);
                    bool first = true;
                    for (int i = a; i < b; ++i)
                    {
                        if (!valid[i])
                            continue;
                        float x = xMap(i), y = yMap(arr[ch][i]);
                        if (first)
                        {
                            nvgMoveTo(vg, x, y);
                            first = false;
                        }
                        else
                            nvgLineTo(vg, x, y);
                    }
                    nvgStrokeColor(vg, agmm::withA(col, 0.95f * alpha));
                    nvgStrokeWidth(vg, 1.1f);
                    nvgStroke(vg);
                }
            };

            int split = dimOld ? cursor : MB_COLS;
            if (showR)
            {
                // Right channel first (drawn underneath), then the left one.
                trace(chR, 0, split, agmm::warm(), 0.9f);
                if (dimOld)
                    trace(chR, std::max(split - 1, 0), MB_COLS, agmm::warm(), 0.30f);
            }
            trace(chL, 0, split, agmm::bright(), 1.f);
            if (dimOld)
                trace(chL, std::max(split - 1, 0), MB_COLS, agmm::bright(), 0.32f);

            // sweep cursor
            // Vertical line at the write head while the sweep is running.
            if (dimOld && cursor < MB_COLS)
            {
                float x = x0 + (float)cursor / (float)MB_COLS * w;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x + 0.5f, y0 + 1.f);
                nvgLineTo(vg, x + 0.5f, y1 - 1.f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 90));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            nvgRestore(vg);
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

            // ----- snapshot of the state -----
            // Everything is copied once so the drawing code below never touches
            // module memory in the middle of a frame.
            int func = 2;
            float drive = 1.f, timeScale = 0.022f, peakIn = 4.5f, peakOut = 4.8f, limitF = 0.f;
            float opN = 0.5f, opY = 0.f;
            bool lConn = true, rConn = true, autoLvl = true, dcB = true, trig = true;
            int capState = MagicBox::CAP_ARMED, capCol = MB_COLS;
            int trHead = 0, trCount = 0;
            float mn[4][MB_COLS], mx[4][MB_COLS];
            bool valid[MB_COLS];
            float trN[MB_TRAIL_N], trY[MB_TRAIL_N];

            if (module)
            {
                func = module->funcCur;
                drive = module->vDrive;
                timeScale = module->vTime;
                peakIn = module->peakIn;
                peakOut = module->peakOut;
                limitF = module->limitFlash;
                opN = module->opN;
                opY = module->opY;
                lConn = module->vLConn;
                rConn = module->vRConn;
                autoLvl = module->autoLevel;
                dcB = module->dcBlock;
                trig = module->lastTrig;
                capState = module->capState;
                capCol = module->capCol;
                trHead = module->trHead;
                trCount = module->trCount;
                std::memcpy(mn, module->scMin, sizeof(mn));
                std::memcpy(mx, module->scMax, sizeof(mx));
                std::memcpy(valid, module->scValid, sizeof(valid));
                std::memcpy(trN, module->trN, sizeof(trN));
                std::memcpy(trY, module->trY, sizeof(trY));
            }
            else
            {
                // demo values for the module browser
                // A 3-cycle sine through the selected function, so the preview
                // looks like a working module without a running engine.
                for (int i = 0; i < MB_COLS; ++i)
                {
                    float t = (float)i / (float)(MB_COLS - 1);
                    float a = std::sin(6.2831853f * 3.f * t);
                    float b = std::sin(6.2831853f * 3.f * t + 0.7f);
                    mn[0][i] = mx[0][i] = 4.5f * a;
                    mn[1][i] = mx[1][i] = 4.5f * b;
                    mn[2][i] = mx[2][i] = 5.f * mbShape(mbSoftClip(a), func);
                    mn[3][i] = mx[3][i] = 5.f * mbShape(mbSoftClip(b), func);
                    valid[i] = true;
                }
                opN = 0.55f;
                opY = mbShape(mbSoftClip(opN), func);
                trCount = MB_TRAIL_N;
                for (int i = 0; i < MB_TRAIL_N; ++i)
                {
                    trN[i] = std::sin(0.22f * (float)i);
                    trY[i] = mbShape(mbSoftClip(trN[i]), func);
                }
            }
            func = rack::math::clamp(func, 0, MB_NUM_FUNCS - 1);
            updateHarmonics(func, drive);

            nvgSave(vg);
            nvgScissor(vg, 0, 0, MB_DISP_W, MB_DISP_H);

            // ================= transfer curve =================
            // f(x) for x in [-1, 1], after the same drive + soft clip the audio
            // path applies. The identity line behind it shows the unprocessed
            // response for comparison.
            {
                const float X0 = MB_TF_X0, Y0 = MB_TF_Y0, S = MB_TF_S;
                auto px = [&](float x)
                { return X0 + (rack::math::clamp(x, -1.1f, 1.1f) + 1.f) * 0.5f * S; };
                auto py = [&](float y)
                { return Y0 + S - (rack::math::clamp(y, -1.1f, 1.1f) + 1.f) * 0.5f * S; };

                nvgSave(vg);
                nvgScissor(vg, X0, Y0, S, S);

                // Sample the curve; 96 points is plenty at 70 px wide.
                const int N = 96;
                float cx[N], cy[N];
                for (int i = 0; i < N; ++i)
                {
                    float x = -1.f + 2.f * (float)i / (float)(N - 1);
                    cx[i] = px(x);
                    cy[i] = py(mbShape(mbSoftClip(x * drive), func));
                }

                // fill between the curve and the zero axis
                nvgBeginPath(vg);
                nvgMoveTo(vg, cx[0], py(0.f));
                for (int i = 0; i < N; ++i)
                    nvgLineTo(vg, cx[i], cy[i]);
                nvgLineTo(vg, cx[N - 1], py(0.f));
                nvgClosePath(vg);
                nvgFillColor(vg, agmm::withA(agmm::accent(), 0.10f));
                nvgFill(vg);

                // The curve itself.
                nvgLineCap(vg, NVG_ROUND);
                nvgLineJoin(vg, NVG_ROUND);
                nvgBeginPath(vg);
                nvgMoveTo(vg, cx[0], cy[0]);
                for (int i = 1; i < N; ++i)
                    nvgLineTo(vg, cx[i], cy[i]);
                nvgStrokeColor(vg, agmm::withA(agmm::bright(), 0.95f));
                nvgStrokeWidth(vg, 1.3f);
                nvgStroke(vg);

                // recent operating points (they all lie on the curve)
                // Older points are smaller/fainter (age^2 fade).
                for (int i = 0; i < trCount; ++i)
                {
                    int idx = ((trHead - trCount + i) % MB_TRAIL_N + MB_TRAIL_N) % MB_TRAIL_N;
                    float age = (float)(i + 1) / (float)trCount;
                    nvgBeginPath(vg);
                    nvgCircle(vg, px(trN[idx]), py(trY[idx]), 1.1f);
                    nvgFillColor(vg, agmm::withA(agmm::warm(), 0.55f * age * age));
                    nvgFill(vg);
                }

                // live point with guides
                // Dashed guides connect the point to both axes so its input and
                // output values can be read off the grid.
                float mxp = px(opN), myp = py(opY);
                dashedLine(vg, mxp + 0.5f, py(0.f), mxp + 0.5f, myp, 2.f, 2.f, agmm::withA(agmm::warm(), 0.5f));
                dashedLine(vg, px(0.f), myp + 0.5f, mxp, myp + 0.5f, 2.f, 2.f, agmm::withA(agmm::bright(), 0.5f));
                nvgBeginPath(vg);
                nvgCircle(vg, mxp, myp, 2.3f);
                nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, 0.97f));
                nvgFill(vg);

                nvgRestore(vg);
            }

            // ================= harmonic bars (flat) =================
            // Each bar is one harmonic level in dB, mapped to 0..-48 dB.
            // Odd harmonics cyan, even harmonics orange.
            {
                float pitch = MB_HM_W / 8.f, bw = pitch - 2.5f;
                float hgt = MB_HM_BASE - MB_HM_TOP;
                for (int k = 0; k < 8; ++k)
                {
                    float db = 20.f * std::log10(std::max(hMag[k], 1e-5f));
                    float f = rack::math::clamp((db + 48.f) / 48.f, 0.f, 1.f);
                    float h = f * hgt;
                    float x = MB_HM_X0 + (float)k * pitch + (pitch - bw) * 0.5f;
                    bool odd = (k % 2 == 0); // harmonics 1, 3, 5, 7
                    NVGcolor col = odd ? agmm::accent() : agmm::warm();
                    if (h < 0.5f)
                        continue;

                    nvgBeginPath(vg);
                    nvgRect(vg, x, MB_HM_BASE - h, bw, h);
                    nvgFillColor(vg, agmm::withA(col, 0.85f));
                    nvgFill(vg);
                }
            }

            // ================= scopes =================
            // Old sweeps are dimmed only at slow time scales, where the trace
            // would otherwise fill the whole screen between triggers.
            bool dimOld = (timeScale >= 0.1f);
            int cursor = (capState == MagicBox::CAP_RUN) ? capCol : MB_COLS;
            drawScope(vg, MB_SC_X0, MB_SCI_Y0, MB_SC_X1, MB_SCI_Y1, mn, mx, valid, 0, 1, rConn, cursor, dimOld);
            drawScope(vg, MB_SC_X0, MB_SCO_Y0, MB_SC_X1, MB_SCO_Y1, mn, mx, valid, 2, 3, rConn, cursor, dimOld);

            // scope captions: their own rows above each scope
            {
                char buf[32];
                agmm::text(vg, MB_SC_X0, MB_CAP_IN_Y, rConn ? "IN L/R" : "IN", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
                snprintf(buf, sizeof(buf), "PK %.1fV", peakIn);
                agmm::text(vg, MB_SC_X1, MB_CAP_IN_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

                agmm::text(vg, MB_SC_X0, MB_CAP_OUT_Y, rConn ? "OUT L/R" : "OUT", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
                snprintf(buf, sizeof(buf), "PK %.1fV", peakOut);
                agmm::text(vg, MB_SC_X1, MB_CAP_OUT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

                // limiter chip between the two texts
                // Lights up when the output stage hit the 8 V knee in the last
                // ~0.33 s.
                agmm::chip(vg, 140.f, MB_CAP_OUT_Y - 5.5f, 30.f, 11.f, "LIMIT", limitF > 0.02f, agmm::warm());
            }

            // ================= header row =================
            {
                char buf[48];
                // Left: active function name.
                agmm::text(vg, 6.f, MB_HDR_Y, MB_NAMES[func], 8.f, agmm::accent(), agmm::ALIGN_L, 0.5f);

                // Centre: level mode + DC blocker + connection state.
                snprintf(buf, sizeof(buf), "%s%s%s", autoLvl ? "AUTO" : "FIXED",
                         dcB ? "+DC" : "", lConn || rConn ? "" : " NO IN");
                agmm::text(vg, 100.f, MB_HDR_Y, buf, 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);

                // Right: sweep time in ms or s.
                if (timeScale < 0.1f)
                    snprintf(buf, sizeof(buf), "%.1f ms", timeScale * 1000.f);
                else if (timeScale < 1.f)
                    snprintf(buf, sizeof(buf), "%.0f ms", timeScale * 1000.f);
                else
                    snprintf(buf, sizeof(buf), "%.2f s", timeScale);
                agmm::text(vg, MB_DISP_W - 6.f, MB_HDR_Y, buf, 7.f, agmm::warm(), agmm::ALIGN_R, 0.5f);

                // trigger status left of the time
                agmm::text(vg, MB_DISP_W - 50.f, MB_HDR_Y, trig ? "TRIG" : "FREE", 7.f,
                           trig ? agmm::bright() : agmm::dim(), agmm::ALIGN_R, 0.5f);
            }

            nvgRestore(vg);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
//
// Order matters: panel background first, then the display, then controls.
// All positions come from the layout constants so artwork and widgets align.
// ------------------------------------------------------------
struct MagicBoxWidget : ModuleWidget
{
    MagicBoxWidget(MagicBox *module)
    {
        setModule(module);
        box.size = Vec(MB_PANEL_W, MB_PANEL_H);

        addChild(new MBPanelBackground());

        // Four rack screws, one per corner.
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new MBDisplayWidget(module));

        // Knobs
        // One loop for all five: order matches MB_KNOBS / MB_KX.
        for (int k = 0; k < 5; ++k)
        {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(MB_KX[k], MB_KNOB_Y), module, MB_KNOBS[k].param));
        }
        agmm::addKnobArcs(this, 2.f);
        // I/O
        addInput(createInputCentered<DarkPJ301MPort>(Vec(MB_IO_X[0], MB_IO_Y), module, MagicBox::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(MB_IO_X[1], MB_IO_Y), module, MagicBox::IN_R_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(MB_IO_X[2], MB_IO_Y), module, MagicBox::FUNC_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(MB_IO_X[3], MB_IO_Y), module, MagicBox::TIME_CV_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(MB_IO_X[4], MB_IO_Y), module, MagicBox::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(MB_IO_X[5], MB_IO_Y), module, MagicBox::OUT_R_OUTPUT));
    }

    void appendContextMenu(ui::Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        MagicBox *module = dynamic_cast<MagicBox *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        // Function list with a one-line description next to each name.
        menu->addChild(createSubmenuItem("Function", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             int cur = (int)std::lround(module->params[MagicBox::FUNC_PARAM].getValue());
                                             for (int i = 0; i < MB_NUM_FUNCS; ++i)
                                             {
                                                 std::string t = std::string(MB_NAMES[i]) + "  -  " + MB_DESC[i];
                                                 submenu->addChild(createMenuItem(t, CHECKMARK(cur == i),
                                                                                  [=]()
                                                                                  { module->params[MagicBox::FUNC_PARAM].setValue((float)i); }));
                                             }
                                         }));

        // Toggles that change how the shaper is fed / cleaned up.
        menu->addChild(createBoolPtrMenuItem("Auto level (normalise to input envelope)", "", &module->autoLevel));
        menu->addChild(createBoolPtrMenuItem("DC blocker (turn off for CV signals)", "", &module->dcBlock));
    }

    
};

Model *modelMagicBox = createModel<MagicBox, MagicBoxWidget>("MagicBox");