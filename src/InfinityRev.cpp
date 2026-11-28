#include "YouTubeLoad.hpp"
#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include "Knob_arcs.hpp"

// ============================================================================
//  InfinityRev - 8-line Feedback Delay Network reverb
//  Signal flow:
//    in -> pre-delay -> 4x allpass diffusion (per channel) -> 8-line FDN
//          (Hadamard feedback matrix, modulated delays, per-line damping,
//           RT60-based decay, freeze) -> decorrelated stereo taps -> width
//    + early reflections (6 taps) -> equal-power dry/wet mix -> DC block/limit
//
// ----------------------------------------------------------------------------
// OVERVIEW
// ----------------------------------------------------------------------------
// InfinityRev is a 24 HP stereo reverb with a fully parametric 8-line feedback
// delay network. The FDN uses an orthonormal Hadamard feedback matrix so the
// tail is energy-preserving when the per-line loop gains are 1, and the RT60
// control sets those gains for a target decay. A Freeze switch (and gate input)
// pushes the loop gains to just under 1 and disables damping so the tail can be
// held indefinitely. Input diffusion is provided by four Schroeder all-passes
// per channel, and six early-reflection taps are mixed in before the width
// stage. A live display shows the eight FDN line levels, the decay envelope,
// the damping response and input/output level meters.
//
// File structure:
//   1. DSP building blocks: IRDelayLine, IRAllpass.
//   2. Constants: base delay lengths, LFO rates, tap signs, diffuser and early
//      reflection delays.
//   3. Tooltip ParamQuantity helpers for Decay and Damping.
//   4. InfinityRev module: state, prepare(), onReset(), updateControl(), the
//      soft limiter and process().
//   5. Anonymous-namespace UI: IRPanel (static art) and IRDisplay (live
//      FDN/decay/damping/meter display).
//   6. InfinityRevWidget: places knobs, the freeze switch, jacks and LEDs.
//   7. Model registration.
// ----------------------------------------------------------------------------

// ------------------------------------------------------------
// Layout (24 HP = 360 x 380)
// Every control sits in a plate with its label ABOVE it.
// ------------------------------------------------------------
static const float IR_W = RACK_GRID_WIDTH * 24; // 360
static const float IR_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it). Rows, top to bottom:
//   y   9    readouts (size, RT60, mix)
//   y  22    captions (FDN lines | decay | meters)
//   28-98    FDN line bars | decay curve + damping curve | in / L / R meters
//   y 111    readouts (pre-delay, early/width, damping)
static const float IR_DX = 20.f, IR_DY = 32.f, IR_DW = 320.f, IR_DH = 120.f;

static const float IR_CAP_Y = 22.f;
static const float IR_FDN_X = 8.f, IR_FDN_BW = 8.f, IR_FDN_GAP = 3.f, IR_FDN_Y = 28.f, IR_FDN_H = 70.f;
static const float IR_FX = 108.f, IR_FW = 130.f;     // decay / damping plots
static const float IR_DEC_Y = 28.f, IR_DEC_H = 38.f; // decay field
static const float IR_DAMP_CAP_Y = 74.f;
static const float IR_DAMP_Y = 80.f, IR_DAMP_H = 18.f; // damping field
static const float IR_MET_X = 258.f, IR_MET_P = 20.f, IR_MET_W = 8.f;
static const float IR_FOOT_Y = 111.f;

// Controls: two rows of five knobs, then the I/O row
static const float IR_R1 = 170.f, IR_R2 = 228.f, IR_CH = 52.f, IR_CW = 64.f;
static const float IR_KNOB_DY = 32.f; // knob centre below the plate top
// Horizontal centre of knob column i (0..4).
static float irCol(int i) { return 40.f + 70.f * i; }
static const float IR_IO_HEAD_Y = 296.f;
static const float IR_IO_TOP = 306.f, IR_IO_H = 46.f, IR_IO_W = 38.f;
static const float IR_JY = IR_IO_TOP + 31.f; // jack centre (label above)
// Horizontal centre of I/O jack i (0..7).
static float irJx(int i) { return 31.f + 42.5f * i; }

// Circular delay line with linear + cubic (Hermite) fractional read.
struct IRDelayLine
{
    std::vector<float> buf;
    int w = 0, n = 0;

    // Allocates the buffer with `size` samples (minimum 16) and resets the
    // write head.
    void init(int size)
    {
        n = std::max(size, 16);
        buf.assign(n, 0.f);
        w = 0;
    }
    // Writes one sample at the write head and advances it (wrapping).
    void write(float x)
    {
        buf[w] = x;
        if (++w >= n)
            w = 0;
    }
    // Wraps an arbitrary integer index into [0, n).
    int wrap(int i) const
    {
        while (i < 0)
            i += n;
        while (i >= n)
            i -= n;
        return i;
    }
    // d >= 1
    // Reads the buffer at a fractional delay `d` (>= 1) using linear
    // interpolation.
    float readLinear(float d) const
    {
        float rp = (float)w - d;
        int i1 = (int)std::floor(rp);
        float f = rp - (float)i1;
        i1 = wrap(i1);
        int i2 = wrap(i1 + 1);
        return buf[i1] + f * (buf[i2] - buf[i1]);
    }
    // d >= 3
    // Reads the buffer at a fractional delay `d` (>= 3) using 4-point cubic
    // (Hermite) interpolation.
    float readCubic(float d) const
    {
        float rp = (float)w - d;
        int i1 = (int)std::floor(rp);
        float f = rp - (float)i1;
        i1 = wrap(i1);
        float xm1 = buf[wrap(i1 - 1)];
        float x0 = buf[i1];
        float x1 = buf[wrap(i1 + 1)];
        float x2 = buf[wrap(i1 + 2)];
        float c1 = 0.5f * (x1 - xm1);
        float c2 = xm1 - 2.5f * x0 + 2.f * x1 - 0.5f * x2;
        float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
        return ((c3 * f + c2) * f + c1) * f + x0;
    }
};

// Schroeder allpass (fixed length)
// Fixed-length Schroeder all-pass used for input diffusion.
struct IRAllpass
{
    std::vector<float> buf;
    int pos = 0;
    // Allocates the all-pass buffer to `len` samples and resets the read/write
    // position.
    void init(int len)
    {
        buf.assign(std::max(len, 1), 0.f);
        pos = 0;
    }
    // Processes one sample through the all-pass with gain `g` and returns the
    // all-pass output.
    float process(float x, float g)
    {
        float d = buf[pos];
        float v = x - g * d;
        buf[pos] = v;
        if (++pos >= (int)buf.size())
            pos = 0;
        return d + g * v;
    }
};

static const int IR_N = 8;

// Delay lengths (ms) at Size = 1.0 - mutually "inharmonic" to avoid ringing
static const float kBaseMs[IR_N] = {29.7f, 37.1f, 41.1f, 43.7f, 53.3f, 59.9f, 67.3f, 73.1f};
// Slow, different LFO rates per line (Hz)
static const float kLfoHz[IR_N] = {0.11f, 0.17f, 0.23f, 0.29f, 0.37f, 0.43f, 0.53f, 0.61f};
// Output tap signs (orthogonal Hadamard rows -> decorrelated L/R)
static const float kSignL[IR_N] = {1, -1, -1, 1, 1, -1, -1, 1};
static const float kSignR[IR_N] = {1, 1, -1, -1, -1, -1, 1, 1};
// Diffuser delays (ms)
static const float kApMsL[4] = {4.77f, 3.60f, 12.73f, 9.31f};
static const float kApMsR[4] = {5.13f, 3.91f, 11.89f, 8.67f};
// Early reflection taps
static const float kErMs[6] = {7.3f, 11.9f, 17.1f, 23.7f, 31.3f, 41.9f};
static const float kErGL[6] = {0.80f, -0.60f, 0.50f, -0.40f, 0.30f, -0.25f};
static const float kErGR[6] = {-0.70f, 0.70f, -0.45f, 0.45f, -0.30f, 0.28f};

// Tooltip helpers
// ParamQuantity for the Decay knob: the display string is the effective RT60
// in seconds, derived from the knob with the same 0.2 s .. 60 s mapping used
// by the DSP.
struct IRDecayQ : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f s", 0.2f * std::pow(300.f, getValue()));
        return std::string(buf);
    }
};
// ParamQuantity for the Damping knob: the display string is the effective
// cutoff in kHz, derived from the knob with the same 20 kHz .. 1 kHz mapping
// used by the DSP.
struct IRDampQ : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f kHz", 20.f * std::pow(0.05f, getValue()));
        return std::string(buf);
    }
};

// The InfinityRev module: holds all DSP state, configures params/ports/lights,
// prepares the delay lines, updates control-rate values, and runs the full
// reverb in process().
struct InfinityRev : Module
{
    enum ParamId
    {
        PREDELAY_PARAM,
        SIZE_PARAM,
        DECAY_PARAM,
        DAMP_PARAM,
        DIFFUSION_PARAM,
        MOD_PARAM,
        ER_PARAM,
        WIDTH_PARAM,
        MIX_PARAM,
        FREEZE_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_L,
        INPUT_R,
        DECAY_CV_INPUT,
        SIZE_CV_INPUT,
        MIX_CV_INPUT,
        FREEZE_GATE_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_L,
        OUTPUT_R,
        OUTPUTS_LEN
    };
    enum LightId
    {
        FREEZE_LIGHT,
        LIGHTS_LEN
    };

    float sr = 0.f;
    IRDelayLine pdL, pdR, er;
    IRAllpass apL[4], apR[4];
    IRDelayLine fdn[IR_N];
    float lp[IR_N] = {};
    float phase[IR_N] = {};
    float gain[IR_N] = {};
    float dampK = 0.5f;
    float apG = 0.6f;
    float sizeS = 1.f;
    float freezeS = 0.f;
    float smoothK = 0.001f;
    int ctr = 0;
    float dcInL = 0.f, dcOutL = 0.f, dcInR = 0.f, dcOutR = 0.f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float lineLevel[IR_N] = {};
    float meterIn = 0.f, meterOutL = 0.f, meterOutR = 0.f;
    float dispRt60 = 4.f, dispCut = 8000.f, dispMix = 0.35f;

    // Constructor: configures every parameter (with the Decay/Damping custom
    // ParamQuantity readouts), input, output and the freeze light, then calls
    // prepare() at 48 kHz.
    InfinityRev()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PREDELAY_PARAM, 0.f, 250.f, 20.f, "Pre-delay", " ms");
        configParam(SIZE_PARAM, 0.5f, 2.f, 1.f, "Size", "x");
        configParam<IRDecayQ>(DECAY_PARAM, 0.f, 1.f, 0.65f, "Decay (RT60)");
        configParam<IRDampQ>(DAMP_PARAM, 0.f, 1.f, 0.4f, "Damping (cutoff)");
        configParam(DIFFUSION_PARAM, 0.f, 1.f, 0.75f, "Diffusion", "%", 0.f, 100.f);
        configParam(MOD_PARAM, 0.f, 1.f, 0.3f, "Modulation", "%", 0.f, 100.f);
        configParam(ER_PARAM, 0.f, 1.f, 0.4f, "Early reflections", "%", 0.f, 100.f);
        configParam(WIDTH_PARAM, 0.f, 1.5f, 1.f, "Stereo width", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.35f, "Mix (dry/wet)", "%", 0.f, 100.f);
        configSwitch(FREEZE_PARAM, 0.f, 1.f, 0.f, "Freeze", {"Off", "On"});
        configInput(INPUT_L, "Left (mono if the right input is empty)");
        configInput(INPUT_R, "Right");
        configInput(DECAY_CV_INPUT, "Decay CV (10 V = full range)");
        configInput(SIZE_CV_INPUT, "Size CV (10 V adds 1.5x)");
        configInput(MIX_CV_INPUT, "Mix CV (10 V = 100%)");
        configInput(FREEZE_GATE_INPUT, "Freeze gate");
        configOutput(OUTPUT_L, "Left");
        configOutput(OUTPUT_R, "Right");
        configBypass(INPUT_L, OUTPUT_L);
        configBypass(INPUT_R, OUTPUT_R);
        configLight(FREEZE_LIGHT, "Freeze");
        prepare(48000.f);
    }

    // (Re)allocates every delay line and all-pass for the given sample rate,
    // resets filter/LFO/DC state, and recomputes the smoothing coefficient.
    void prepare(float newSr)
    {
        sr = newSr;
        pdL.init((int)(0.26f * sr));
        pdR.init((int)(0.26f * sr));
        er.init((int)(0.10f * sr));
        for (int i = 0; i < 4; ++i)
        {
            apL[i].init((int)std::round(kApMsL[i] * 0.001f * sr));
            apR[i].init((int)std::round(kApMsR[i] * 0.001f * sr));
        }
        for (int i = 0; i < IR_N; ++i)
        {
            fdn[i].init((int)(0.30f * sr));
            lp[i] = 0.f;
            phase[i] = (float)i / (float)IR_N; // spread LFO phases
            gain[i] = 0.9f;
        }
        smoothK = 1.f - std::exp(-1.f / (0.08f * sr));
        ctr = 0;
        dcInL = dcOutL = dcInR = dcOutR = 0.f;
    }

    // Rack reset hook: re-prepares the reverb at the current sample rate,
    // clearing all delay/filter state.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        prepare(sr);
    }

    // Control-rate update (every 32 samples)
    // Recomputes the per-line loop gains from the RT60 (including freeze),
    // the damping coefficient from the cutoff (including freeze), and the
    // diffusion all-pass gain. Caches the display values.
    void updateControl()
    {
        float dec = params[DECAY_PARAM].getValue() + inputs[DECAY_CV_INPUT].getVoltage() * 0.1f;
        dec = rack::math::clamp(dec, 0.f, 1.f);
        float rt60 = 0.2f * std::pow(300.f, dec); // 0.2 s .. 60 s
        dispRt60 = rt60;

        for (int i = 0; i < IR_N; ++i)
        {
            float delaySec = kBaseMs[i] * 0.001f * sizeS;
            float g = std::pow(10.f, -3.f * delaySec / rt60);
            gain[i] = g + (0.99995f - g) * freezeS; // freeze = infinite
        }

        float damp = params[DAMP_PARAM].getValue();
        float cutoff = 20000.f * std::pow(0.05f, damp); // 20k .. 1k
        cutoff = std::min(cutoff, 0.45f * sr);
        dispCut = cutoff;
        float k = 1.f - std::exp(-2.f * (float)M_PI * cutoff / sr);
        dampK = k + (1.f - k) * freezeS; // no damping while frozen

        apG = 0.1f + 0.65f * params[DIFFUSION_PARAM].getValue();
    }

    // Smooth limiter: linear up to ±8 V, then a tanh shoulder with an
    // asymptote at ±10 V. Applied to the final stereo output.
    static float softLimit(float x)
    {
        const float t = 8.f;
        float a = std::fabs(x);
        if (a <= t)
            return x;
        float over = a - t;
        float y = t + 2.f * std::tanh(over / 2.f); // asymptote at 10 V
        return x < 0.f ? -y : y;
    }

    // Main audio callback: reads inputs and controls, smooths size and freeze,
    // runs pre-delay, early reflections, diffusion, the 8-line FDN with
    // Hadamard feedback, output taps and width, equal-power dry/wet mix, DC
    // blocking and soft limiting, writes the outputs, and updates the display
    // meters and line levels.
    void process(const ProcessArgs &args) override
    {
        if (args.sampleRate != sr)
            prepare(args.sampleRate);

        float inL = inputs[INPUT_L].getVoltage();
        float inR = inputs[INPUT_R].isConnected() ? inputs[INPUT_R].getVoltage() : inL;

        // Smoothed size / freeze
        float sizeT = params[SIZE_PARAM].getValue() + inputs[SIZE_CV_INPUT].getVoltage() * 0.15f;
        sizeT = rack::math::clamp(sizeT, 0.5f, 2.f);
        sizeS += smoothK * (sizeT - sizeS);

        bool frz = params[FREEZE_PARAM].getValue() > 0.5f || inputs[FREEZE_GATE_INPUT].getVoltage() >= 1.f;
        freezeS += smoothK * ((frz ? 1.f : 0.f) - freezeS);
        lights[FREEZE_LIGHT].setBrightness(freezeS);

        if (--ctr <= 0)
        {
            ctr = 32;
            updateControl();
        }

        // ---- Pre-delay + early reflections ----
        float pdSamp = std::max(3.f, params[PREDELAY_PARAM].getValue() * 0.001f * sr);
        float pL = pdL.readLinear(pdSamp);
        float pR = pdR.readLinear(pdSamp);
        pdL.write(inL);
        pdR.write(inR);

        float erLevel = params[ER_PARAM].getValue();
        float erL = 0.f, erR = 0.f;
        for (int t = 0; t < 6; ++t)
        {
            float s = er.readLinear(kErMs[t] * 0.001f * sr);
            erL += s * kErGL[t];
            erR += s * kErGR[t];
        }
        er.write(0.5f * (inL + inR));

        // ---- Input diffusion ----
        float dL = pL, dR = pR;
        for (int k = 0; k < 4; ++k)
        {
            dL = apL[k].process(dL, apG);
            dR = apR[k].process(dR, apG);
        }
        float inGain = (1.f - freezeS) * 0.5f;
        dL *= inGain;
        dR *= inGain;

        // ---- FDN ----
        float modDepth = params[MOD_PARAM].getValue() * 0.0005f * sr; // up to ~0.5 ms
        float y[IR_N];
        float md = 1.f - 6.f * args.sampleTime;
        for (int i = 0; i < IR_N; ++i)
        {
            float lfo = std::sin(2.f * (float)M_PI * phase[i]);
            phase[i] += kLfoHz[i] / sr;
            if (phase[i] >= 1.f)
                phase[i] -= 1.f;

            float d = kBaseMs[i] * 0.001f * sr * sizeS + modDepth * lfo;
            d = rack::math::clamp(d, 4.f, (float)fdn[i].n - 4.f);
            float v = fdn[i].readCubic(d);
            lp[i] += dampK * (v - lp[i]);
            y[i] = lp[i] * gain[i];
            lineLevel[i] = std::max(std::fabs(y[i]), lineLevel[i] * md);
        }

        // Output taps (before the feedback matrix)
        float wL = 0.f, wR = 0.f;
        for (int i = 0; i < IR_N; ++i)
        {
            wL += kSignL[i] * y[i];
            wR += kSignR[i] * y[i];
        }
        wL *= 0.3f;
        wR *= 0.3f;

        // Fast Walsh-Hadamard transform (orthonormal feedback matrix)
        float m[IR_N];
        for (int i = 0; i < IR_N; ++i)
            m[i] = y[i];
        for (int len = 1; len < IR_N; len <<= 1)
            for (int i = 0; i < IR_N; i += len * 2)
                for (int j = i; j < i + len; ++j)
                {
                    float a = m[j], b = m[j + len];
                    m[j] = a + b;
                    m[j + len] = a - b;
                }
        const float hn = 0.35355339f; // 1/sqrt(8)
        for (int i = 0; i < IR_N; ++i)
            fdn[i].write(m[i] * hn + ((i & 1) ? dR : dL));

        // ---- Wet assembly ----
        wL += erL * erLevel * 0.35f;
        wR += erR * erLevel * 0.35f;

        float mid = 0.5f * (wL + wR);
        float side = 0.5f * (wL - wR) * params[WIDTH_PARAM].getValue();
        wL = mid + side;
        wR = mid - side;

        // ---- Equal-power mix ----
        float mix = params[MIX_PARAM].getValue() + inputs[MIX_CV_INPUT].getVoltage() * 0.1f;
        mix = rack::math::clamp(mix, 0.f, 1.f);
        dispMix = mix;
        float dryG = std::cos(mix * 0.5f * (float)M_PI);
        float wetG = std::sin(mix * 0.5f * (float)M_PI);
        float oL = dryG * inL + wetG * wL;
        float oR = dryG * inR + wetG * wR;

        // ---- DC block + limiter ----
        float hpL = oL - dcInL + 0.995f * dcOutL;
        dcInL = oL;
        dcOutL = hpL;
        float hpR = oR - dcInR + 0.995f * dcOutR;
        dcInR = oR;
        dcOutR = hpR;

        float finalL = softLimit(hpL), finalR = softLimit(hpR);
        outputs[OUTPUT_L].setVoltage(finalL);
        outputs[OUTPUT_R].setVoltage(finalR);

        meterIn = std::max(0.5f * (std::fabs(inL) + std::fabs(inR)), meterIn * md);
        meterOutL = std::max(std::fabs(finalL), meterOutL * md);
        meterOutR = std::max(std::fabs(finalR), meterOutR * md);
    }
};

// ------------------------------------------------------------
// UI (anonymous namespace: names cannot clash with other module files)
// ------------------------------------------------------------
namespace
{

    // Static artwork: base panel, display bezel, the two knob rows (five knobs
    // each) and the I/O row with eight jack cells (outputs tinted).
    struct IRPanel : TransparentWidget
    {
        IRPanel() { box.size = Vec(IR_W, IR_H); }
        // Draws all static panel art: base panel, bezel, the two rows of five
        // knob cells, and the I/O row with eight jack cells (outputs tinted).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, IR_W, IR_H, "INFINITYREV");
            agmm::bezel(vg, IR_DX, IR_DY, IR_DW, IR_DH);

            const char *r1[5] = {"PRE-DELAY", "SIZE", "DECAY", "DAMPING", "DIFFUSE"};
            const char *r2[5] = {"MOD", "EARLY", "WIDTH", "MIX", "FREEZE"};
            for (int i = 0; i < 5; ++i)
            {
                agmm::cell(vg, irCol(i), IR_R1, IR_CW, IR_CH, r1[i], false, 7.f);
                agmm::cell(vg, irCol(i), IR_R2, IR_CW, IR_CH, r2[i], false, 7.f);
            }

            agmm::sectionHeader(vg, IR_W, IR_IO_HEAD_Y, "I / O", 14.f);
            const char *io[8] = {"IN L", "IN R", "DEC CV", "SIZE CV", "MIX CV", "FREEZE", "OUT L", "OUT R"};
            for (int i = 0; i < 8; ++i)
                agmm::cell(vg, irJx(i), IR_IO_TOP, IR_IO_W, IR_IO_H, io[i], i >= 6, 7.f);
            Widget::draw(args);
        }
    };

    // Display: FDN line activity, decay curve, damping response, level meters
    // Live display: eight FDN line-level bars, a decay envelope plot, a damping
    // response plot, three level meters (in / L / R), plus header and footer
    // readouts.
    struct IRDisplay : TransparentWidget
    {
        InfinityRev *module = nullptr;
        IRDisplay(InfinityRev *m) : module(m)
        {
            box.pos = Vec(IR_DX, IR_DY);
            box.size = Vec(IR_DW, IR_DH);
        }

        // Static layer: fills the display screen, draws the decay and damping
        // plot frames with their faint grids, and lays out the caption row
        // above each plot and the meter captions.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, IR_DW, IR_DH);
            agmm::plotFrame(vg, IR_FX, IR_DEC_Y, IR_FW, IR_DEC_H);
            agmm::plotFrame(vg, IR_FX, IR_DAMP_Y, IR_FW, IR_DAMP_H);

            nvgBeginPath(vg);
            for (int t = 1; t < 4; ++t)
            {
                float x = IR_FX + IR_FW * t / 4.f + 0.5f;
                nvgMoveTo(vg, x, IR_DEC_Y);
                nvgLineTo(vg, x, IR_DEC_Y + IR_DEC_H);
            }
            for (int d = 1; d <= 3; ++d)
            { // 100 Hz, 1 kHz, 10 kHz
                float x = IR_FX + IR_FW * (std::log10(std::pow(10.f, (float)d) * 10.f / 20.f) / 3.f) + 0.5f;
                nvgMoveTo(vg, x, IR_DAMP_Y);
                nvgLineTo(vg, x, IR_DAMP_Y + IR_DAMP_H);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // captions sit above their plots, never inside them
            agmm::text(vg, IR_FDN_X, IR_CAP_Y, "FDN LINES", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
            agmm::text(vg, IR_FX, IR_CAP_Y, "DECAY 0-8S", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
            agmm::text(vg, IR_FX, IR_DAMP_CAP_Y, "DAMPING 20-20K", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
            const char *mc[3] = {"IN", "L", "R"};
            for (int k = 0; k < 3; ++k)
                agmm::text(vg, IR_MET_X + k * IR_MET_P + IR_MET_W / 2.f, IR_CAP_Y, mc[k], 7.f, agmm::dim(), agmm::ALIGN_C, 0.f);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the header readouts (size / RT60 / mix), the
        // eight FDN line bars, the decay envelope (with RT60 marker unless
        // frozen), the one-pole damping response (with cutoff marker), the
        // three meters, and the footer readouts (pre-delay / early + width /
        // damping). Uses plausible demo values when there is no module.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = IR_DW;
            const float FX = IR_FX, FW = IR_FW;

            float lvl[IR_N], mIn = 2.f, mL = 2.8f, mR = 2.4f, size = 1.f, rt60 = 4.f, cut = 8000.f;
            float frz = 0.f, pre = 20.f, er = 0.4f, wid = 1.f, mix = 0.35f;
            for (int i = 0; i < IR_N; ++i)
                lvl[i] = 0.4f + 1.6f * std::fabs(std::sin(i * 1.3f + 0.4f));
            if (module)
            {
                for (int i = 0; i < IR_N; ++i)
                    lvl[i] = module->lineLevel[i];
                mIn = module->meterIn;
                mL = module->meterOutL;
                mR = module->meterOutR;
                size = module->sizeS;
                rt60 = module->dispRt60;
                cut = module->dispCut;
                frz = module->freezeS;
                mix = module->dispMix;
                pre = module->params[InfinityRev::PREDELAY_PARAM].getValue();
                er = module->params[InfinityRev::ER_PARAM].getValue();
                wid = module->params[InfinityRev::WIDTH_PARAM].getValue();
            }
            rt60 = std::max(rt60, 0.01f);

            char buf[64];

            // ---- header row ----
            snprintf(buf, sizeof(buf), "SIZE %.2fX", size);
            agmm::text(vg, 8.f, 9.f, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            if (frz > 0.5f)
                snprintf(buf, sizeof(buf), "RT60 INF");
            else if (rt60 > 99.f)
                snprintf(buf, sizeof(buf), "RT60 >99S");
            else
                snprintf(buf, sizeof(buf), "RT60 %.1fS", rt60);
            agmm::text(vg, W / 2.f, 9.f, buf, 7.f, nvgRGB(0xff, 0xe6, 0x80), agmm::ALIGN_C, 0.5f);
            snprintf(buf, sizeof(buf), "MIX %3.0f%%", mix * 100.f);
            agmm::text(vg, W - 8.f, 9.f, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

            // ---- FDN line activity: 8 flat bars ----
            {
                NVGcolor bc = frz > 0.5f ? agmm::violet() : agmm::accent();
                for (int i = 0; i < IR_N; ++i)
                {
                    float x = IR_FDN_X + i * (IR_FDN_BW + IR_FDN_GAP);
                    float a = std::min(std::max(lvl[i] / 2.5f, 0.f), 1.f);
                    nvgBeginPath(vg);
                    nvgRect(vg, x, IR_FDN_Y, IR_FDN_BW, IR_FDN_H);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                    nvgFill(vg);
                    if (a > 0.01f)
                    {
                        nvgBeginPath(vg);
                        nvgRect(vg, x, IR_FDN_Y + IR_FDN_H * (1.f - a), IR_FDN_BW, IR_FDN_H * a);
                        nvgFillColor(vg, agmm::withA(bc, 0.45f + 0.55f * a));
                        nvgFill(vg);
                    }
                }
            }

            // ---- decay curve (0..8 s) ----
            {
                const float by = IR_DEC_Y + IR_DEC_H - 1.f, bh = IR_DEC_H - 2.f;
                float preS = pre * 0.001f;
                auto amp = [&](float t)
                {
                    float a = (t < preS) ? 0.f : std::exp(-6.9078f * (t - preS) / rt60);
                    float fl = (t < preS) ? 0.f : 1.f;
                    return a * (1.f - frz) + fl * frz;
                };
                nvgBeginPath(vg);
                nvgMoveTo(vg, FX, by);
                for (int i = 0; i <= (int)FW; ++i)
                    nvgLineTo(vg, FX + i, by - amp(8.f * i / FW) * bh);
                nvgLineTo(vg, FX + FW, by);
                nvgClosePath(vg);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                nvgFill(vg);

                nvgBeginPath(vg);
                for (int i = 0; i <= (int)FW; ++i)
                {
                    float x = FX + i, y = by - amp(8.f * i / FW) * bh;
                    if (i == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, agmm::bright());
                nvgStrokeWidth(vg, 1.3f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                if (frz < 0.5f && rt60 + preS < 8.f)
                { // RT60 marker
                    float mx = FX + FW * (rt60 + preS) / 8.f;
                    nvgBeginPath(vg);
                    for (float y = IR_DEC_Y + 1.f; y < IR_DEC_Y + IR_DEC_H - 1.f; y += 4.f)
                    {
                        nvgMoveTo(vg, mx + 0.5f, y);
                        nvgLineTo(vg, mx + 0.5f, y + 2.f);
                    }
                    nvgStrokeColor(vg, agmm::withA(agmm::violet(), 0.9f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
            }

            // ---- damping response (one-pole, log 20 Hz..20 kHz, 0..-36 dB) ----
            {
                float fc = cut * (1.f - frz) + 40000.f * frz;
                auto yAt = [&](int i)
                {
                    float f = 20.f * std::pow(1000.f, (float)i / FW);
                    float db = -10.f * std::log10(1.f + (f / fc) * (f / fc));
                    float t = std::min(std::max(-db / 36.f, 0.f), 1.f);
                    return IR_DAMP_Y + 1.f + t * (IR_DAMP_H - 2.f);
                };
                nvgBeginPath(vg);
                for (int i = 0; i <= (int)FW; ++i)
                {
                    if (i == 0)
                        nvgMoveTo(vg, FX + i, yAt(i));
                    else
                        nvgLineTo(vg, FX + i, yAt(i));
                }
                nvgStrokeColor(vg, nvgRGB(0xd8, 0xb0, 0xff));
                nvgStrokeWidth(vg, 1.2f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                float cxm = FX + FW * std::min(std::log10(std::max(fc, 20.f) / 20.f) / 3.f, 1.f);
                nvgBeginPath(vg);
                nvgRect(vg, cxm, IR_DAMP_Y, 1.f, IR_DAMP_H);
                nvgFillColor(vg, agmm::withA(agmm::accent(), 0.7f));
                nvgFill(vg);
            }

            // ---- meters: in, L, R ----
            const float lv[3] = {mIn / 5.f, mL / 5.f, mR / 5.f};
            for (int k = 0; k < 3; ++k)
                agmm::meter(vg, IR_MET_X + k * IR_MET_P, IR_FDN_Y, IR_MET_W, IR_FDN_H, lv[k]);

            // ---- footer row ----
            snprintf(buf, sizeof(buf), "PRE %3.0fMS", pre);
            agmm::text(vg, 8.f, IR_FOOT_Y, buf, 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "ER %2.0f%%  WID %3.0f%%", er * 100.f, wid * 100.f);
            agmm::text(vg, W / 2.f, IR_FOOT_Y, buf, 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            snprintf(buf, sizeof(buf), "DAMP %.1fK", cut / 1000.f);
            agmm::text(vg, W - 8.f, IR_FOOT_Y, buf, 7.f, agmm::dim(), agmm::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Module widget: adds the static panel, screws, the live display, the two
// rows of knobs, the freeze switch and light, and the eight I/O jacks.
// ============================================================
struct InfinityRevWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, live display, the
    // five row-1 knobs, the four row-2 knobs, the freeze switch plus its
    // light, and the eight I/O jacks.
    InfinityRevWidget(InfinityRev *module)
    {
        setModule(module);
        box.size = Vec(IR_W, IR_H);

        addChild(new IRPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new IRDisplay(module));

        const int row1[5] = {InfinityRev::PREDELAY_PARAM, InfinityRev::SIZE_PARAM, InfinityRev::DECAY_PARAM,
                             InfinityRev::DAMP_PARAM, InfinityRev::DIFFUSION_PARAM};
        const int row2[4] = {InfinityRev::MOD_PARAM, InfinityRev::ER_PARAM, InfinityRev::WIDTH_PARAM, InfinityRev::MIX_PARAM};
        for (int i = 0; i < 5; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(irCol(i), IR_R1 + IR_KNOB_DY), module, row1[i]));
        agmm::addKnobArcs(this, 2.f);
            for (int i = 0; i < 4; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(irCol(i), IR_R2 + IR_KNOB_DY), module, row2[i]));
        agmm::addKnobArcs(this, 2.f);
            addParam(createParamCentered<CKSS>(Vec(irCol(4) - 8.f, IR_R2 + 34.f), module, InfinityRev::FREEZE_PARAM));
        addChild(createLightCentered<SmallLight<agmm::VioletLight>>(Vec(irCol(4) + 14.f, IR_R2 + 34.f), module, InfinityRev::FREEZE_LIGHT));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(0), IR_JY), module, InfinityRev::INPUT_L));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(1), IR_JY), module, InfinityRev::INPUT_R));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(2), IR_JY), module, InfinityRev::DECAY_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(3), IR_JY), module, InfinityRev::SIZE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(4), IR_JY), module, InfinityRev::MIX_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(5), IR_JY), module, InfinityRev::FREEZE_GATE_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(irJx(6), IR_JY), module, InfinityRev::OUTPUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(irJx(7), IR_JY), module, InfinityRev::OUTPUT_R));
    }

    void appendContextMenu(Menu *menu) override
    {
        agmm::addYouTubeLink(menu); // adds "Visit my youtube channel!"
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        // ... your other menu items ...
    }
};

Model *modelInfinityRev = createModel<InfinityRev, InfinityRevWidget>("InfinityRev");