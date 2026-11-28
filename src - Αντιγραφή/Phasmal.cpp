// Phasmal.cpp
// AGMM-styled stereo phaser with feedback, LFO sweep, and a live
// frequency-response display.
//
// ============================================================================
// OVERVIEW 
// ============================================================================
// Phasmal is a 20 HP stereo phaser. Its core is a chain of first-order
// all-pass stages (2, 4, 6, 8, 10 or 12 stages) with global feedback, whose
// corner frequency is swept by an LFO (sine or triangle). The left and right
// channels can be swept with a phase offset (stereo spread), and the module
// provides CV control over rate, depth, center frequency and feedback. A
// small display shows the exact magnitude response of the current settings
// (computed from the same maths as the audio path), the current sweep range,
// and a small LFO scope.
//
// File structure:
//   1. Layout constants (20 HP = 300x380 px).
//   2. Shared maths helpers (phLfo, phStageCoef, phMagDb) used by both the
//      DSP and the display.
//   3. Phasmal module: parameters, CV handling, the phaser DSP, and the
//      display snapshot (PhasmalViz).
//   4. Anonymous-namespace UI: PHPanelBackground, PHKnobArcs, PHDisplayWidget.
//   5. PhasmalWidget: places knobs, jacks, the display and the context menu.
//   6. Model registration.
// ============================================================================

#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <complex>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float PH_PANEL_W = RACK_GRID_WIDTH * 20; // 300
static const float PH_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (260 x 116): header row, plot with dB column, frequency row, LFO strip
static const float PH_DISP_W = 260.f;
static const float PH_DISP_H = 116.f;
static const float PH_DISP_X = 20.f;
static const float PH_DISP_Y = 34.f;

// Plot area inside the display (display-local coordinates)
static const float PH_PX0 = 30.f, PH_PX1 = 252.f;
static const float PH_PY0 = 20.f, PH_PY1 = 72.f;
static const float PH_PW = PH_PX1 - PH_PX0;
static const float PH_PH = PH_PY1 - PH_PY0;
static const float PH_DB_TOP = 12.f, PH_DB_BOT = -36.f;

// LFO strip inside the display
static const float PH_LCY = 100.f;
static const float PH_LAMP = 6.f;

// Knob cells (2 rows x 4): plate top, knob centre is 33 px below it
static const float PH_KNOB_X[4] = {45.f, 115.f, 185.f, 255.f};
static const float PH_CELL_W = 62.f;
static const float PH_CELL_H = 58.f;
static const float PH_ROW_TOP[2] = {162.f, 224.f};
// Returns the knob centre y for control row `row` (0..1).
static float phKnobY(int row) { return PH_ROW_TOP[row] + 33.f; }

// Jack cells: IN (2) | CV (4) | OUT (2)
static const float PH_JACK_CAP_Y = 293.f;
static const float PH_JACK_TOP = 301.f;
static const float PH_JACK_H = 44.f;
static const float PH_JACK_W = 30.f;
static const float PH_IO_X[8] = {22.f, 55.f, 100.f, 133.f, 166.f, 199.f, 245.f, 278.f};
// Returns the y centre for the input/output jack row.
static float phJackY() { return PH_JACK_TOP + 28.f; }

// ------------------------------------------------------------
// Shared helpers (DSP + display use the same maths)
// ------------------------------------------------------------
// Snapshot of the phaser's parameters and state, shared between the audio
// thread (which fills it) and the UI thread (which renders it).
struct PhasmalViz
{
    float fcL = 780.f, fcR = 780.f;
    float fcMin = 100.f, fcMax = 6000.f;
    float fb = 0.4f, mix = 0.5f, level = 1.f;
    float rate = 0.5f, depth = 0.6f, spread = 0.5f;
    float phase = 0.15f;
    int stages = 6;
    int shape = 0;
    float fs = 44100.f;
};

// Evaluates the LFO waveform at normalised phase p (0..1). Shape 1 is a
// triangle (with a 90-degree offset) and shape 0 is a sine.
static inline float phLfo(float p, int shape)
{
    p -= std::floor(p);
    if (shape == 1)
    {
        float q = p + 0.25f;
        q -= std::floor(q);
        return 1.f - 4.f * std::fabs(q - 0.5f);
    }
    return std::sin(2.f * (float)M_PI * p);
}

// First-order all-pass coefficient for a given corner frequency
// Computes the all-pass coefficient a = (tan(pi*fc/fs) - 1) /
// (tan(pi*fc/fs) + 1) for the given corner frequency and sample rate.
static inline float phStageCoef(float fc, float fs)
{
    float t = std::tan((float)M_PI * fc / fs);
    return (t - 1.f) / (t + 1.f);
}

// Exact magnitude response (dB) of the digital phaser at frequency f
// Computes the exact magnitude response (in dB) of the full phaser
// (all-pass chain plus feedback plus wet/dry mix) at frequency f, using the
// same parameters as the DSP. Used only for drawing the response curve.
static inline float phMagDb(float f, float fc, const PhasmalViz &v)
{
    float fs = v.fs;
    f = std::min(f, 0.499f * fs);
    float w = 2.f * (float)M_PI * f / fs;
    std::complex<float> z1 = std::polar(1.f, -w);
    float a = phStageCoef(fc, fs);
    std::complex<float> A1 = (a + z1) / (1.f + a * z1);
    std::complex<float> A(1.f, 0.f);
    for (int i = 0; i < v.stages; ++i)
        A *= A1;
    std::complex<float> W = A / (1.f - v.fb * A * z1);
    float norm = 1.f - 0.5f * std::fabs(v.fb);
    std::complex<float> H = (1.f - v.mix) + v.mix * norm * W;
    return 20.f * std::log10(std::abs(H) + 1e-5f) + 20.f * std::log10(std::max(v.level, 1e-3f));
}

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Phasmal : Module
{
    enum ParamId
    {
        RATE_PARAM,
        DEPTH_PARAM,
        FREQ_PARAM,
        FEEDBACK_PARAM,
        STAGES_PARAM,
        SPREAD_PARAM,
        MIX_PARAM,
        LEVEL_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        IN_L_INPUT,
        IN_R_INPUT,
        RATE_CV_INPUT,
        DEPTH_CV_INPUT,
        FREQ_CV_INPUT,
        FB_CV_INPUT,
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

    enum LfoShape
    {
        SHAPE_SINE,
        SHAPE_TRI,
        NUM_SHAPES
    };

    // DSP state
    float st[2][12] = {};
    float fbState[2] = {};
    float phase = 0.f;
    int lastN = 6;
    LfoShape shape = SHAPE_SINE;

    // Read by the display
    PhasmalViz viz;

    // Constructor: configures all parameters (with sensible ranges and
    // units), CV inputs, stereo outputs, and stereo bypass.
    Phasmal()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(RATE_PARAM, -5.f, 4.f, -1.f, "Rate", " Hz", 2.f, 1.f);
        configParam(DEPTH_PARAM, 0.f, 1.f, 0.6f, "Depth", "%", 0.f, 100.f);
        configParam(FREQ_PARAM, 6.64f, 12.3f, 9.6f, "Center frequency", " Hz", 2.f, 1.f);
        configParam(FEEDBACK_PARAM, -0.95f, 0.95f, 0.4f, "Feedback", "%", 0.f, 100.f);
        configSwitch(STAGES_PARAM, 0.f, 5.f, 2.f, "Stages", {"2", "4", "6", "8", "10", "12"});
        configParam(SPREAD_PARAM, 0.f, 1.f, 0.5f, "Stereo spread", " deg", 0.f, 180.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Mix", "%", 0.f, 100.f);
        configParam(LEVEL_PARAM, 0.f, 2.f, 1.f, "Output level", "%", 0.f, 100.f);

        configInput(IN_L_INPUT, "Left / mono");
        configInput(IN_R_INPUT, "Right (normalled to left)");
        configInput(RATE_CV_INPUT, "Rate CV (1 V/oct)");
        configInput(DEPTH_CV_INPUT, "Depth CV (+-5 V = +-50%)");
        configInput(FREQ_CV_INPUT, "Center frequency CV (1 V/oct)");
        configInput(FB_CV_INPUT, "Feedback CV (+-5 V = +-50%)");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");
        configBypass(IN_L_INPUT, OUT_L_OUTPUT);
        configBypass(IN_R_INPUT, OUT_R_OUTPUT);
    }

    // Rack reset hook: clears the all-pass and feedback state and restarts
    // the LFO phase.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        std::memset(st, 0, sizeof(st));
        std::memset(fbState, 0, sizeof(fbState));
        phase = 0.f;
    }

    // Main audio callback: reads all parameters plus their CVs, advances the
    // LFO, computes per-channel sweep frequencies, runs the all-pass chain
    // with feedback for each channel, and writes the stereo outputs. Also
    // fills the PhasmalViz snapshot for the display.
    void process(const ProcessArgs &args) override
    {
        const float fs = args.sampleRate;

        // ---- parameters + CV ----
        float rate = std::exp2(rack::math::clamp(
            params[RATE_PARAM].getValue() + inputs[RATE_CV_INPUT].getVoltage(), -7.f, 6.f));
        float depth = rack::math::clamp(
            params[DEPTH_PARAM].getValue() + inputs[DEPTH_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        float fb = rack::math::clamp(
            params[FEEDBACK_PARAM].getValue() + inputs[FB_CV_INPUT].getVoltage() * 0.1f, -0.95f, 0.95f);
        float fc0 = std::exp2(params[FREQ_PARAM].getValue() + inputs[FREQ_CV_INPUT].getVoltage());
        float spread = params[SPREAD_PARAM].getValue();
        float mix = params[MIX_PARAM].getValue();
        float level = params[LEVEL_PARAM].getValue();
        int N = 2 * ((int)std::round(params[STAGES_PARAM].getValue()) + 1);
        N = rack::math::clamp(N, 2, 12);

        // Clear all-pass state beyond the new stage count when it decreases.
        if (N != lastN)
        {
            int from = std::min(N, lastN);
            for (int c = 0; c < 2; ++c)
                for (int i = from; i < 12; ++i)
                    st[c][i] = 0.f;
            lastN = N;
        }

        // ---- LFO ----
        phase += rate * args.sampleTime;
        phase -= std::floor(phase);
        float lfoL = phLfo(phase, shape);
        float lfoR = phLfo(phase + spread * 0.5f, shape);

        // ---- sweep ----
        const float sweep = depth * 3.f; // octaves each way
        const float fmax = std::min(18000.f, 0.45f * fs);
        float fcL = rack::math::clamp(fc0 * std::exp2(sweep * lfoL), 20.f, fmax);
        float fcR = rack::math::clamp(fc0 * std::exp2(sweep * lfoR), 20.f, fmax);

        // ---- audio ----
        float inL = inputs[IN_L_INPUT].getVoltage();
        float inR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getVoltage() : inL;

        const float norm = 1.f - 0.5f * std::fabs(fb);
        // Runs one channel's all-pass chain with feedback and mixes dry/wet,
        // applying the output level. `c` selects the channel state.
        auto run = [&](int c, float in, float fc)
        {
            float a = phStageCoef(fc, fs);
            float x = in + fb * fbState[c];
            for (int i = 0; i < N; ++i)
            {
                float y = a * x + st[c][i];
                st[c][i] = x - a * y;
                x = y;
            }
            fbState[c] = 5.f * std::tanh(x * 0.2f); // keep resonance under control
            return (in * (1.f - mix) + x * norm * mix) * level;
        };

        outputs[OUT_L_OUTPUT].setVoltage(run(0, inL, fcL));
        outputs[OUT_R_OUTPUT].setVoltage(run(1, inR, fcR));

        // ---- display data ----
        viz.fcL = fcL;
        viz.fcR = fcR;
        viz.fcMin = rack::math::clamp(fc0 * std::exp2(-sweep), 20.f, fmax);
        viz.fcMax = rack::math::clamp(fc0 * std::exp2(sweep), 20.f, fmax);
        viz.fb = fb;
        viz.mix = mix;
        viz.level = level;
        viz.rate = rate;
        viz.depth = depth;
        viz.spread = spread;
        viz.phase = phase;
        viz.stages = N;
        viz.shape = (int)shape;
        viz.fs = fs;
    }

    // Serialises the current LFO shape so a patch reopens with the same
    // waveform.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "shape", json_integer((int)shape));
        return root;
    }

    // Restores the LFO shape from a saved patch, clamping it to a valid
    // enum value.
    void dataFromJson(json_t *root) override
    {
        json_t *j = json_object_get(root, "shape");
        if (j)
        {
            int s = (int)json_integer_value(j);
            shape = (LfoShape)rack::math::clamp(s, 0, (int)NUM_SHAPES - 1);
        }
    }
};

// ------------------------------------------------------------
// UI (anonymous namespace)
// ------------------------------------------------------------
namespace
{
    namespace ag = agmm;

    // ------------------------------------------------------------
    // Panel background: title, display frame, labelled plates
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the 8 knob cell plates, the
    // jack group captions, and the 8 I/O jack cell plates (outputs tinted).
    struct PHPanelBackground : TransparentWidget
    {
        PHPanelBackground() { box.size = Vec(PH_PANEL_W, PH_PANEL_H); }

        // Draws all static panel art: base panel, bezel, the 8 knob cells
        // (2 rows x 4 columns), the INPUT/CV INPUT/OUTPUT captions, and the
        // 8 I/O jack cells (outputs tinted).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, PH_PANEL_W, PH_PANEL_H, "PHASMAL");
            ag::bezel(vg, PH_DISP_X, PH_DISP_Y, PH_DISP_W, PH_DISP_H);

            // knob cells
            const char *names[2][4] = {
                {"RATE", "DEPTH", "CENTER", "FEEDBACK"},
                {"STAGES", "SPREAD", "MIX", "LEVEL"}};
            for (int r = 0; r < 2; ++r)
                for (int c = 0; c < 4; ++c)
                    ag::cell(vg, PH_KNOB_X[c], PH_ROW_TOP[r], PH_CELL_W, PH_CELL_H, names[r][c]);

            // jack groups
            ag::text(vg, (PH_IO_X[0] + PH_IO_X[1]) / 2.f, PH_JACK_CAP_Y, "INPUT", 7.f, ag::accent(), ag::ALIGN_C, 1.5f);
            ag::text(vg, (PH_IO_X[2] + PH_IO_X[5]) / 2.f, PH_JACK_CAP_Y, "CV INPUT", 7.f, ag::accent(), ag::ALIGN_C, 1.5f);
            ag::text(vg, (PH_IO_X[6] + PH_IO_X[7]) / 2.f, PH_JACK_CAP_Y, "OUTPUT", 7.f, ag::accent(), ag::ALIGN_C, 1.5f);

            const char *io[8] = {"IN L", "IN R", "RATE", "DEPTH", "FREQ", "FDBK", "OUT L", "OUT R"};
            for (int i = 0; i < 8; ++i)
                ag::cell(vg, PH_IO_X[i], PH_JACK_TOP, PH_JACK_W, PH_JACK_H, io[i], i >= 6, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Live value arcs around the knobs (flat track + flat arc)
    // ------------------------------------------------------------
    // Draws a flat track arc around every knob plus a coloured arc showing
    // the current value (bipolar feedback shows the arc from centre in warm
    // colour when negative).
    struct PHKnobArcs : TransparentWidget
    {
        Phasmal *module = nullptr;

        PHKnobArcs(Phasmal *m) : module(m) { box.size = Vec(PH_PANEL_W, PH_PANEL_H); }

        // Draws the value arcs for all 8 knobs. Skips the value arc when the
        // change from the origin is negligible.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float R = 17.f;
            const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f; // nvg angles
            const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

            for (int k = 0; k < 8; ++k)
            {
                float cx = PH_KNOB_X[k % 4];
                float cy = phKnobY(k / 4);

                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, R, a0, a1, NVG_CW);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);

                if (!module)
                    continue;
                float v = module->paramQuantities[k]->getScaledValue(); // 0..1
                bool bipolar = (k == Phasmal::FEEDBACK_PARAM);
                float from = bipolar ? 0.5f : 0.f;
                float lo = std::min(from, v), hi = std::max(from, v);
                if (hi - lo < 0.004f)
                    continue;

                NVGcolor col = (bipolar && v < 0.5f) ? ag::warm() : ag::accent();
                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, R, a0 + (a1 - a0) * lo, a0 + (a1 - a0) * hi, NVG_CW);
                nvgStrokeColor(vg, col);
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }
        }
    };

    // ------------------------------------------------------------
    // Phaser display: live frequency response + LFO scope
    // ------------------------------------------------------------
    // Live display: draws the phaser's exact magnitude response (computed via
    // phMagDb), the current sweep range, the left/right center-frequency
    // markers, an LFO scope, and header readouts (rate, fc, stages, feedback,
    // mix). Uses demo values in the module browser.
    struct PHDisplayWidget : TransparentWidget
    {
        Phasmal *module = nullptr;

        PHDisplayWidget(Phasmal *m) : module(m)
        {
            box.pos = Vec(PH_DISP_X, PH_DISP_Y);
            box.size = Vec(PH_DISP_W, PH_DISP_H);
        }

        // Maps a frequency f (Hz) to an x pixel inside the plot (log scale
        // from 20 Hz to 20 kHz).
        static float xOfFreq(float f)
        {
            f = rack::math::clamp(f, 20.f, 20000.f);
            return PH_PX0 + (std::log10(f) - 1.30103f) / 3.f * PH_PW;
        }
        // Maps a dB value to a y pixel inside the plot (linear between
        // PH_DB_BOT and PH_DB_TOP).
        static float yOfDb(float db)
        {
            db = rack::math::clamp(db, PH_DB_BOT, PH_DB_TOP);
            return PH_PY0 + (PH_DB_TOP - db) / (PH_DB_TOP - PH_DB_BOT) * PH_PH;
        }

        // Draws a single-pixel vertical line at x between y0 and y1.
        static void vline(NVGcontext *vg, float x, float y0, float y1, NVGcolor c)
        {
            nvgBeginPath(vg);
            nvgMoveTo(vg, x + 0.5f, y0);
            nvgLineTo(vg, x + 0.5f, y1);
            nvgStrokeColor(vg, c);
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // ---- static background: screen + grid + labels ----
        // Draws the display screen, the header divider, the plot frame, the
        // log-frequency and dB grids with their labels, and the LFO strip
        // frame with its caption.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, PH_DISP_W, PH_DISP_H);

            // header divider
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, 14.5f);
            nvgLineTo(vg, PH_DISP_W - 8.f, 14.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, PH_PX0, PH_PY0, PH_PW, PH_PH);

            // Vertical frequency grid
            const float freqs[10] = {20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
            for (int i = 0; i < 10; ++i)
            {
                bool major = (freqs[i] == 100.f || freqs[i] == 1000.f || freqs[i] == 10000.f);
                vline(vg, xOfFreq(freqs[i]), PH_PY0 + 1.f, PH_PY1 - 1.f,
                      nvgRGBA(0x3d, 0xe0, 0xff, major ? 32 : 14));
            }
            // Horizontal dB grid, labels in their own column left of the plot
            const float dbs[5] = {12.f, 0.f, -12.f, -24.f, -36.f};
            const char *dbl[5] = {"+12", "0", "-12", "-24", "-36"};
            for (int i = 0; i < 5; ++i)
            {
                float y = yOfDb(dbs[i]);
                nvgBeginPath(vg);
                nvgMoveTo(vg, PH_PX0 + 1.f, y + 0.5f);
                nvgLineTo(vg, PH_PX1 - 1.f, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, dbs[i] == 0.f ? 44 : 16));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                ag::text(vg, PH_PX0 - 4.f, y, dbl[i], 7.f, ag::dim(), ag::ALIGN_R, 0.f);
            }
            // Frequency labels in their own row under the plot
            ag::text(vg, xOfFreq(100.f), PH_PY1 + 8.f, "100", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, xOfFreq(1000.f), PH_PY1 + 8.f, "1k", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, xOfFreq(10000.f), PH_PY1 + 8.f, "10k", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            // LFO strip: frame, centre line, caption
            const float lx0 = PH_PX0, lw = PH_PW;
            ag::plotFrame(vg, lx0, PH_LCY - PH_LAMP - 3.f, lw, 2.f * PH_LAMP + 6.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, lx0 + 2.f, PH_LCY + 0.5f);
            nvgLineTo(vg, lx0 + lw - 2.f, PH_LCY + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            ag::text(vg, PH_PX0 - 4.f, PH_LCY, "LFO", 7.f, ag::dim(), ag::ALIGN_R, 0.f);

            Widget::draw(args);
        }

        // Response curve (optionally filled)
        // Draws the magnitude response for a given center frequency `fc`,
        // sampling phMagDb() at NP log-spaced frequencies. Optionally fills
        // the area under the curve; `widthCore` sets the line width.
        void drawCurve(NVGcontext *vg, const PhasmalViz &v, float fc, NVGcolor col,
                       bool fill, float widthCore)
        {
            const int NP = 100;
            float xs[NP], ys[NP];
            for (int i = 0; i < NP; ++i)
            {
                float f = 20.f * std::pow(10.f, 3.f * (float)i / (float)(NP - 1));
                xs[i] = xOfFreq(f);
                ys[i] = yOfDb(phMagDb(f, fc, v));
            }

            if (fill)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, xs[0], PH_PY1);
                for (int i = 0; i < NP; ++i)
                    nvgLineTo(vg, xs[i], ys[i]);
                nvgLineTo(vg, xs[NP - 1], PH_PY1);
                nvgClosePath(vg);
                nvgFillColor(vg, ag::withA(col, 0.16f));
                nvgFill(vg);
            }

            nvgLineJoin(vg, NVG_ROUND);
            nvgBeginPath(vg);
            nvgMoveTo(vg, xs[0], ys[0]);
            for (int i = 1; i < NP; ++i)
                nvgLineTo(vg, xs[i], ys[i]);
            nvgStrokeColor(vg, col);
            nvgStrokeWidth(vg, widthCore);
            nvgStroke(vg);
        }

        // Layer-1 overlay: draws the sweep-range band, the L/R response
        // curves (R only if spread and depth are non-zero), the center-
        // frequency markers, the LFO scope with its current-position markers,
        // and the header readouts (rate, fc/stages, feedback/mix).
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            PhasmalViz v; // demo values in the module browser
            if (module)
                v = module->viz;

            nvgSave(vg);
            nvgScissor(vg, PH_PX0, PH_PY0, PH_PW, PH_PH);

            // ---- sweep range band ----
            float bx0 = xOfFreq(v.fcMin), bx1 = xOfFreq(v.fcMax);
            if (bx1 - bx0 < 1.5f)
                bx1 = bx0 + 1.5f;
            nvgBeginPath(vg);
            nvgRect(vg, bx0, PH_PY0, bx1 - bx0, PH_PH);
            nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
            nvgFill(vg);
            vline(vg, bx0, PH_PY0, PH_PY1, nvgRGBA(0x3d, 0xe0, 0xff, 60));
            vline(vg, bx1, PH_PY0, PH_PY1, nvgRGBA(0x3d, 0xe0, 0xff, 60));

            // ---- response curves (right channel underneath, only when it differs) ----
            bool showR = (v.spread > 0.02f && v.depth > 0.01f);
            if (showR)
                drawCurve(vg, v, v.fcR, ag::warm(), false, 1.1f);
            drawCurve(vg, v, v.fcL, ag::bright(), true, 1.5f);

            // ---- centre-frequency markers ----
            float xl = xOfFreq(v.fcL), xr = xOfFreq(v.fcR);
            if (showR)
            {
                vline(vg, xr, PH_PY0, PH_PY1, ag::withA(ag::warm(), 0.4f));
                nvgBeginPath(vg);
                nvgCircle(vg, xr + 0.5f, PH_PY0 + 2.5f, 2.f);
                nvgFillColor(vg, ag::warm());
                nvgFill(vg);
            }
            vline(vg, xl, PH_PY0, PH_PY1, ag::withA(ag::bright(), 0.45f));
            nvgBeginPath(vg);
            nvgCircle(vg, xl + 0.5f, PH_PY0 + 2.5f, 2.2f);
            nvgFillColor(vg, nvgRGB(0xc8, 0xf6, 0xff));
            nvgFill(vg);
            nvgRestore(vg);

            // ---- LFO scope ----
            {
                nvgSave(vg);
                nvgScissor(vg, PH_PX0, PH_LCY - PH_LAMP - 3.f, PH_PW, 2.f * PH_LAMP + 6.f);
                const float lx0 = PH_PX0 + 3.f, lw = PH_PW - 6.f;
                const int NP = 80;
                float off = v.spread * 0.5f;

                for (int ch = 1; ch >= 0; --ch)
                {
                    if (ch == 1 && !showR)
                        continue;
                    nvgBeginPath(vg);
                    for (int i = 0; i < NP; ++i)
                    {
                        float p = (float)i / (float)(NP - 1);
                        float y = PH_LCY - phLfo(p + (ch ? off : 0.f), v.shape) * PH_LAMP;
                        float x = lx0 + p * lw;
                        if (i == 0)
                            nvgMoveTo(vg, x, y);
                        else
                            nvgLineTo(vg, x, y);
                    }
                    nvgStrokeColor(vg, ch ? ag::withA(ag::warm(), 0.7f) : ag::bright());
                    nvgStrokeWidth(vg, ch ? 1.f : 1.2f);
                    nvgStroke(vg);
                }

                float cxp = lx0 + (v.phase - std::floor(v.phase)) * lw;
                vline(vg, cxp, PH_LCY - PH_LAMP - 2.f, PH_LCY + PH_LAMP + 2.f, nvgRGBA(255, 255, 255, 70));

                float yL = PH_LCY - phLfo(v.phase, v.shape) * PH_LAMP;
                float yR = PH_LCY - phLfo(v.phase + off, v.shape) * PH_LAMP;
                if (showR)
                {
                    nvgBeginPath(vg);
                    nvgCircle(vg, cxp, yR, 2.f);
                    nvgFillColor(vg, ag::warm());
                    nvgFill(vg);
                }
                nvgBeginPath(vg);
                nvgCircle(vg, cxp, yL, 2.4f);
                nvgFillColor(vg, nvgRGB(0xc8, 0xf6, 0xff));
                nvgFill(vg);
                nvgRestore(vg);
            }

            // ---- header readouts ----
            {
                char buf[48];
                if (v.rate < 10.f)
                    snprintf(buf, sizeof(buf), "RATE %.2f Hz", v.rate);
                else
                    snprintf(buf, sizeof(buf), "RATE %.1f Hz", v.rate);
                ag::text(vg, 8.f, 7.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.3f);

                snprintf(buf, sizeof(buf), "FC %.0f Hz  N%d", v.fcL, v.stages);
                ag::text(vg, PH_DISP_W / 2.f, 7.f, buf, 7.f, ag::textCol(), ag::ALIGN_C, 0.3f);

                snprintf(buf, sizeof(buf), "FB %+.0f%%  MIX %.0f%%", v.fb * 100.f, v.mix * 100.f);
                ag::text(vg, PH_DISP_W - 8.f, 7.f, buf, 7.f, ag::warm(), ag::ALIGN_R, 0.3f);
            }

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget: adds the static panel, the knob value arcs, screws, the
// display, the 8 knobs, the 8 I/O jacks, and the context menu (LFO shape).
// ------------------------------------------------------------
struct PhasmalWidget : ModuleWidget
{
    // Builds the widget tree: static background, knob value arcs, screws,
    // display, the 8 knobs (row-major: RATE/DEPTH/CENTER/FEEDBACK then
    // STAGES/SPREAD/MIX/LEVEL) and the 8 I/O jacks.
    PhasmalWidget(Phasmal *module)
    {
        setModule(module);
        box.size = Vec(PH_PANEL_W, PH_PANEL_H);

        addChild(new PHPanelBackground());
        addChild(new PHKnobArcs(module));

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PHDisplayWidget(module));

        // Knobs: row 1 = RATE, DEPTH, CENTER, FEEDBACK; row 2 = STAGES, SPREAD, MIX, LEVEL
        for (int k = 0; k < 8; ++k)
        {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(PH_KNOB_X[k % 4], phKnobY(k / 4)), module, k));
        }

        const float y = phJackY();
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[0], y), module, Phasmal::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[1], y), module, Phasmal::IN_R_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[2], y), module, Phasmal::RATE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[3], y), module, Phasmal::DEPTH_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[4], y), module, Phasmal::FREQ_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PH_IO_X[5], y), module, Phasmal::FB_CV_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PH_IO_X[6], y), module, Phasmal::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PH_IO_X[7], y), module, Phasmal::OUT_R_OUTPUT));
    }

    // Adds the right-click menu: an "LFO shape" submenu with Sine and
    // Triangle options and a checkmark on the current shape.
    void appendContextMenu(ui::Menu *menu) override
    {
        Phasmal *module = dynamic_cast<Phasmal *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("LFO shape", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *names[] = {"Sine", "Triangle"};
                                             for (int i = 0; i < Phasmal::NUM_SHAPES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(names[i],
                                                                                  CHECKMARK((int)module->shape == i),
                                                                                  [=]()
                                                                                  { module->shape = (Phasmal::LfoShape)i; }));
                                             }
                                         }));
    }
};

Model *modelPhasmal = createModel<Phasmal, PhasmalWidget>("Phasmal");