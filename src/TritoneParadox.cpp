#include "YouTubeLoad.hpp"
#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include "Knob_arcs.hpp"

// ============================================================================
// TritoneParadox - Shepard-tone pair generator for the tritone paradox
// ============================================================================
//
// OVERVIEW
// --------
// The tritone paradox (Deutsch, 1986) is an auditory illusion in which two
// Shepard tones separated by a tritone (6 semitones) are heard as ascending
// by some listeners and descending by others. The direction of perceived
// motion depends on the absolute pitch classes of the tones and on the
// position of the fixed spectral envelope that weights the octave-spaced
// partials.
//
// The spectral envelope is a Gaussian in log-frequency space, centred at a
// user-adjustable frequency. Shifting the envelope centre changes the
// perceptual bias. An optional context tone can be played before the pair
// (Chambers et al., 2017).
//
// Controls
//   PITCH       pitch class of the first tone (shown as a note name)
//   TRITONE     interval between the two tones (5..7 semitones)
//   PARTIALS    octave-spaced partials per tone (1..20)
//   ENV CENTER  centre frequency of the Gaussian envelope
//   ENV WIDTH   width of the envelope in octaves
//   SWEEP       depth of a slow automatic sweep of the envelope centre
//   MODE        Manual (trigger) or Auto (free-running cycle)
//   CONTEXT     Off / Lower / Higher context tone before the pair
//   RATE        cycle speed (0.1 .. 200 Hz, exponential)
//   FADE        cross-fade time between the tones
//   SPREAD      stereo spread of the partials
//   LEVEL       output level
//   PLAY        manual start button
//
// Inputs
//   TRIG        rising edge starts a sequence (Manual mode)
//   V/OCT       transposes the pair in semitone steps
//   ENV CV      1 V/oct modulation of the envelope centre
//   RATE CV     1 V/oct modulation of the cycle rate
//
// Outputs
//   OUT L / OUT R   stereo audio
//   GATE A / GATE B high while tone A / tone B sounds
//   EOC             trigger at the end of every A-B cycle
// ============================================================================

// ------------------------------------------------------------
// Layout constants (28 HP = 420 px wide)
// ------------------------------------------------------------
static const float TP_PANEL_W = RACK_GRID_WIDTH * 28; // 420
static const float TP_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (380 x 110), bezel adds 4 px on every side
static const float TP_DISP_W = 380.f;
static const float TP_DISP_H = 110.f;
static const float TP_DISP_X = (TP_PANEL_W - TP_DISP_W) / 2.f; // 20
static const float TP_DISP_Y = 24.f;

// Inside the display (local coordinates)
static const float TP_HDR_H = 18.f; // header strip
static const float TP_PLOT_Y = 22.f;
static const float TP_PLOT_H = 72.f;
static const float TP_ENV_X = 8.f; // envelope plot
static const float TP_ENV_W = 236.f;
static const float TP_CHR_X = 252.f; // chroma plot
static const float TP_CHR_W = 120.f;
static const float TP_SWEEP_PERIOD = 20.f; // seconds per envelope sweep cycle
static const float TP_LOG_MIN = 5.64386f;  // log2(50 Hz)
static const float TP_LOG_MAX = 11.96578f; // log2(4000 Hz)

// Control rows: every row spans x = 10 .. 410
static const float TP_ROW_X0 = 10.f;
static const float TP_ROW_W = 400.f;
static const float TP_CELL_H = 54.f;
static const float TP_KNOB_DY = 34.f; // control centre below the cell top

static const float TP_R1_HDR = 147.f; // SHEPARD TONES
static const float TP_R1_TOP = 155.f;
static const float TP_R2_HDR = 217.f; // SEQUENCE / OUTPUT
static const float TP_R2_TOP = 225.f;
static const float TP_R3_HDR = 287.f; // I / O
static const float TP_R3_TOP = 295.f;

// Centre of cell i of n equal cells in a row.
static inline float tpCellX(int i, int n)
{
    return TP_ROW_X0 + TP_ROW_W / (float)n * ((float)i + 0.5f);
}

// ------------------------------------------------------------
// Shared helpers
// ------------------------------------------------------------
namespace
{
    // Converts semitones above C4 to Hz (C4 = 261.625565 Hz).
    inline float tpSemisToHz(float semis)
    {
        return 261.625565f * std::pow(2.f, semis / 12.f);
    }

    // Formats a frequency as a note name with octave (e.g. "A#3").
    inline void tpFreqToNote(float hz, char *buf, size_t n)
    {
        if (hz <= 0.f)
        {
            snprintf(buf, n, "--");
            return;
        }
        static const char *names[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
        float midiNote = 69.f + 12.f * std::log2(hz / 440.f);
        int midiInt = (int)std::round(midiNote);
        int octave = (int)std::floor(midiInt / 12.0) - 1;
        int pc = midiInt % 12;
        if (pc < 0)
            pc += 12;
        snprintf(buf, n, "%s%d", names[pc], octave);
    }

    // Gaussian weight for a partial at frequency `hz`, given envelope
    // centre `centreHz` and width `sigma` (in octaves).
    inline float tpEnvelopeWeight(float hz, float centreHz, float sigma)
    {
        if (hz <= 0.f || centreHz <= 0.f)
            return 0.f;
        float log2Ratio = std::log2(hz / centreHz);
        return std::exp(-(log2Ratio * log2Ratio) / (2.f * sigma * sigma));
    }

    // Sine-only waveform (Shepard tones are traditionally pure sinusoids).
    inline float tpWaveSample(float ph)
    {
        return std::sin(2.f * (float)M_PI * ph);
    }
} // namespace

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct TritoneParadox : Module
{
    // New parameters, inputs and outputs are appended after the original ones
    // so that existing patches keep loading correctly.
    enum ParamId
    {
        PITCH_PARAM,
        ENV_CENTER_PARAM,
        ENV_WIDTH_PARAM,
        PARTIALS_PARAM,
        TRITONE_PARAM,
        MODE_PARAM, // 0 = Manual, 1 = Auto
        RATE_PARAM,
        CONTEXT_PARAM, // 0 = Off, 1 = Lower, 2 = Higher
        SWEEP_PARAM,
        FADE_PARAM,
        SPREAD_PARAM,
        LEVEL_PARAM,
        PLAY_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        TRIG_INPUT,
        VOCT_INPUT,
        ENV_CV_INPUT,
        RATE_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        GATE_A_OUTPUT,
        GATE_B_OUTPUT,
        EOC_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        PLAY_LIGHT,
        LIGHTS_LEN
    };

    static const int TP_MAX_PARTIALS = 20;
    static const int TP_CTRL_DIV = 16; // control-rate divider (samples)

    // ── Custom ParamQuantity for PITCH (shows note name) ──────────────
    struct PitchQuantity : ParamQuantity
    {
        std::string getDisplayValueString() override
        {
            float semis = getValue(); // semitones above C4
            float hz = tpSemisToHz(semis);
            char buf[32];
            tpFreqToNote(hz, buf, sizeof(buf));
            return buf;
        }
    };

    // ── Custom ParamQuantity for PARTIALS (integer) ───────────────────
    struct PartialsQuantity : ParamQuantity
    {
        std::string getDisplayValueString() override
        {
            int n = rack::math::clamp((int)std::round(getValue()), 1, TP_MAX_PARTIALS);
            return string::f("%d", n);
        }
    };

    // ── Custom ParamQuantity for RATE (exponential 0.1 .. 200 Hz) ─────
    struct RateQuantity : ParamQuantity
    {
        static float rawToHz(float raw)
        {
            return 0.1f * std::pow(2000.f, rack::math::clamp(raw, 0.f, 1.f));
        }
        static float hzToRaw(float hz)
        {
            return std::log(std::max(hz, 0.1f) / 0.1f) / std::log(2000.f);
        }
        std::string getDisplayValueString() override
        {
            return string::f("%.2f Hz", rawToHz(getValue()));
        }
        void setDisplayValueString(std::string s) override
        {
            float hz = std::atof(s.c_str());
            if (hz > 0.f)
                setValue(rack::math::clamp(hzToRaw(hz), 0.f, 1.f));
        }
    };

    // One Shepard tone: octave-spaced partials with their envelope weights
    // and constant-power pan gains. Rebuilt at control rate.
    struct ToneBank
    {
        float freq[TP_MAX_PARTIALS] = {};
        float amp[TP_MAX_PARTIALS] = {};
        float gl[TP_MAX_PARTIALS] = {};
        float gr[TP_MAX_PARTIALS] = {};
        float norm = 1.f;
    };

    // ── DSP state ────────────────────────────────────────────────────
    ToneBank bankA, bankB, bankC;
    float phasesA[TP_MAX_PARTIALS] = {};
    float phasesB[TP_MAX_PARTIALS] = {};
    float phasesC[TP_MAX_PARTIALS] = {}; // context tone

    // Sequencer state
    enum SeqState
    {
        SEQ_IDLE,
        SEQ_CONTEXT,
        SEQ_TONE_A,
        SEQ_TONE_B
    };
    SeqState seqState = SEQ_IDLE;
    float seqTimer = 0.f;
    bool lastTrig = false;

    // Smoothed per-state gains (anti-click crossfade between states).
    // Initialised to zero so the module fades in from silence.
    float gainA = 0.f;
    float gainB = 0.f;
    float gainCtx = 0.f;

    // Control-rate values
    int ctrlCount = 0;
    float rateHz = 1.f;
    float smoothK = 0.01f;
    float levelGain = 1.f;
    float sweepPhase = 0.f;

    dsp::PulseGenerator eocPulse;

    // Options (saved with the patch)
    bool monoOut = false;

    // Display snapshot (written by the audio thread, read by the display)
    float dispPitchSemis = 0.f;
    float dispEnvCenterHz = 370.f;
    float dispEnvWidth = 1.5f;
    int dispPartials = 6;
    float dispTritoneSemis = 6.f;
    float dispGainA = 0.f;
    float dispGainB = 0.f;
    int dispSeqState = SEQ_IDLE;
    int dispContextMode = 0;
    bool dispAuto = false;

    TritoneParadox()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        auto *pq = configParam<PitchQuantity>(PITCH_PARAM, -48.f, 48.f, 0.f, "Pitch");
        pq->snapEnabled = true;

        configParam(ENV_CENTER_PARAM, 65.f, 2000.f, 370.f, "Envelope centre", " Hz");
        configParam(ENV_WIDTH_PARAM, 0.5f, 3.f, 1.5f, "Envelope width", " oct");
        configParam<PartialsQuantity>(PARTIALS_PARAM, 1.f, (float)TP_MAX_PARTIALS, 6.f, "Partials", "", 0.f, 1.f);
        paramQuantities[PARTIALS_PARAM]->snapEnabled = true;
        configParam(TRITONE_PARAM, 5.f, 7.f, 6.f, "Tritone interval", " st");
        paramQuantities[TRITONE_PARAM]->snapEnabled = true;
        configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Mode", {"Manual", "Auto"});

        // RATE: exponential 0.1 .. 200 Hz (raw 0..1). Default raw 0.303 = about 1 Hz.
        configParam<RateQuantity>(RATE_PARAM, 0.f, 1.f, 0.303f, "Cycle rate");

        configSwitch(CONTEXT_PARAM, 0.f, 2.f, 0.f, "Context tone", {"Off", "Lower", "Higher"});

        configParam(SWEEP_PARAM, 0.f, 2.f, 0.f, "Envelope sweep depth", " oct");
        configParam(FADE_PARAM, 1.f, 50.f, 5.f, "Cross-fade time", " ms");
        configParam(SPREAD_PARAM, 0.f, 1.f, 0.55f, "Stereo spread", "%", 0.f, 100.f);
        configParam(LEVEL_PARAM, 0.f, 1.5f, 1.f, "Output level", "%", 0.f, 100.f);
        configButton(PLAY_PARAM, "Play sequence");

        configInput(TRIG_INPUT, "Trigger");
        configInput(VOCT_INPUT, "Pitch (1 V/oct, semitone steps)");
        configInput(ENV_CV_INPUT, "Envelope centre (1 V/oct)");
        configInput(RATE_CV_INPUT, "Cycle rate (1 V/oct)");
        configOutput(OUT_L_OUTPUT, "L out");
        configOutput(OUT_R_OUTPUT, "R out");
        configOutput(GATE_A_OUTPUT, "Tone A gate");
        configOutput(GATE_B_OUTPUT, "Tone B gate");
        configOutput(EOC_OUTPUT, "End of cycle");
    }

    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        seqState = SEQ_IDLE;
        seqTimer = 0.f;
        gainA = gainB = gainCtx = 0.f;
        sweepPhase = 0.f;
        ctrlCount = 0;
        for (int i = 0; i < TP_MAX_PARTIALS; ++i)
            phasesA[i] = phasesB[i] = phasesC[i] = 0.f;
    }

    // Builds one tone bank: partial frequencies, envelope weights, pan gains
    // and the RMS normalisation factor.
    static void buildBank(ToneBank &b, float baseSemis, float envCenterHz, float envWidth,
                          float spread, int partials, float sr)
    {
        const float base = tpSemisToHz(baseSemis);
        float sumSq = 0.f;
        for (int i = 0; i < TP_MAX_PARTIALS; ++i)
        {
            if (i >= partials)
            {
                b.freq[i] = 0.f;
                b.amp[i] = 0.f;
                b.gl[i] = 0.f;
                b.gr[i] = 0.f;
                continue;
            }
            float f = base * (float)(1 << i);
            float a = tpEnvelopeWeight(f, envCenterHz, envWidth);
            float fc = rack::math::clamp(f, 10.f, sr * 0.45f);

            float log2Ratio = std::log2(std::max(fc, 1.f) / envCenterHz);
            float pan = rack::math::clamp(log2Ratio / envWidth * spread, -1.f, 1.f);
            float ang = (pan + 1.f) * 0.25f * (float)M_PI;

            b.freq[i] = fc;
            b.amp[i] = a;
            b.gl[i] = std::cos(ang);
            b.gr[i] = std::sin(ang);
            if (a >= 1e-4f)
                sumSq += a * a;
        }
        // Normalise each tone independently so its RMS is consistent
        // across partial counts.
        b.norm = (sumSq > 0.001f) ? 1.8f / std::sqrt(sumSq) : 1.f;
    }

    // Adds one tone to the output pair.
    static void renderBank(const ToneBank &b, float *phase, float gain, float dt,
                           float &outL, float &outR)
    {
        if (gain < 1e-5f)
            return;
        float tL = 0.f, tR = 0.f;
        for (int i = 0; i < TP_MAX_PARTIALS; ++i)
        {
            float amp = b.amp[i];
            if (amp < 1e-4f)
                continue;
            phase[i] += b.freq[i] * dt;
            phase[i] -= std::floor(phase[i]);
            float s = tpWaveSample(phase[i]) * amp;
            tL += s * b.gl[i];
            tR += s * b.gr[i];
        }
        outL += tL * b.norm * gain;
        outR += tR * b.norm * gain;
    }

    // Everything that does not need to run every sample.
    void updateControl(float sr)
    {
        const float dt = 1.f / sr;

        // Pitch: knob plus V/OCT quantised to semitones.
        float pitchSemis = params[PITCH_PARAM].getValue();
        if (inputs[VOCT_INPUT].isConnected())
            pitchSemis += std::round(inputs[VOCT_INPUT].getVoltage() * 12.f);
        pitchSemis = rack::math::clamp(pitchSemis, -96.f, 96.f);

        // Envelope centre: knob, CV (1 V/oct) and automatic sweep.
        float envCv = 0.f;
        if (inputs[ENV_CV_INPUT].isConnected())
            envCv = rack::math::clamp(inputs[ENV_CV_INPUT].getVoltage(), -6.f, 6.f);
        float sweepOct = params[SWEEP_PARAM].getValue() * std::sin(2.f * (float)M_PI * sweepPhase);
        float envCenterHz = params[ENV_CENTER_PARAM].getValue() * std::pow(2.f, envCv + sweepOct);
        envCenterHz = rack::math::clamp(envCenterHz, 30.f, 6000.f);

        float envWidth = std::max(params[ENV_WIDTH_PARAM].getValue(), 0.1f);
        int partials = rack::math::clamp((int)std::round(params[PARTIALS_PARAM].getValue()), 1, TP_MAX_PARTIALS);
        float tritoneSemis = params[TRITONE_PARAM].getValue();
        int contextMode = rack::math::clamp((int)std::round(params[CONTEXT_PARAM].getValue()), 0, 2);
        float spread = params[SPREAD_PARAM].getValue();

        // Cycle rate: knob plus CV (1 V/oct).
        float rateCv = 0.f;
        if (inputs[RATE_CV_INPUT].isConnected())
            rateCv = rack::math::clamp(inputs[RATE_CV_INPUT].getVoltage(), -6.f, 6.f);
        rateHz = RateQuantity::rawToHz(params[RATE_PARAM].getValue()) * std::pow(2.f, rateCv);
        rateHz = rack::math::clamp(rateHz, 0.05f, 400.f);

        // Cross-fade smoother coefficient.
        float tau = std::max(params[FADE_PARAM].getValue(), 1.f) * 0.001f;
        smoothK = 1.f - std::exp(-dt / tau);

        levelGain = params[LEVEL_PARAM].getValue();

        // Tone banks
        buildBank(bankA, pitchSemis, envCenterHz, envWidth, spread, partials, sr);
        buildBank(bankB, pitchSemis + tritoneSemis, envCenterHz, envWidth, spread, partials, sr);
        if (contextMode > 0)
        {
            float shift = (contextMode == 1) ? -12.f : 12.f;
            buildBank(bankC, pitchSemis + shift, envCenterHz, envWidth, spread, partials, sr);
        }
        else
        {
            for (int i = 0; i < TP_MAX_PARTIALS; ++i)
                bankC.amp[i] = 0.f;
        }

        // Display snapshot
        dispPitchSemis = pitchSemis;
        dispEnvCenterHz = envCenterHz;
        dispEnvWidth = envWidth;
        dispPartials = partials;
        dispTritoneSemis = tritoneSemis;
        dispContextMode = contextMode;
        dispAuto = params[MODE_PARAM].getValue() > 0.5f;
        dispGainA = gainA;
        dispGainB = gainB;
        dispSeqState = (int)seqState;
    }

    void process(const ProcessArgs &args) override
    {
        const float dt = args.sampleTime;

        sweepPhase += dt / TP_SWEEP_PERIOD;
        if (sweepPhase >= 1.f)
            sweepPhase -= 1.f;

        if (--ctrlCount <= 0)
        {
            ctrlCount = TP_CTRL_DIV;
            updateControl(args.sampleRate);
        }

        bool autoMode = params[MODE_PARAM].getValue() > 0.5f;
        int contextMode = rack::math::clamp((int)std::round(params[CONTEXT_PARAM].getValue()), 0, 2);

        // ── Sequencer (timing only, gains handle the smooth transitions) ──
        bool trig = inputs[TRIG_INPUT].getVoltage() > 1.f || params[PLAY_PARAM].getValue() > 0.5f;
        if (trig && !lastTrig && seqState == SEQ_IDLE)
            startSequence();
        lastTrig = trig;

        if (autoMode && seqState == SEQ_IDLE)
            startSequence();

        if (seqState != SEQ_IDLE)
        {
            seqTimer += dt;
            float stateDuration = 1.f / rateHz;
            if (seqState == SEQ_CONTEXT)
                stateDuration *= 0.8f;
            if (seqTimer >= stateDuration)
            {
                seqTimer = 0.f;
                switch (seqState)
                {
                case SEQ_CONTEXT:
                    seqState = SEQ_TONE_A;
                    break;
                case SEQ_TONE_A:
                    seqState = SEQ_TONE_B;
                    break;
                case SEQ_TONE_B:
                    eocPulse.trigger(1e-3f);
                    if (contextMode == 0)
                        seqState = autoMode ? SEQ_TONE_A : SEQ_IDLE;
                    else
                        seqState = autoMode ? SEQ_CONTEXT : SEQ_IDLE;
                    break;
                default:
                    seqState = SEQ_IDLE;
                }
            }
        }

        // ── Anti-click: smoothed per-state gains ─────────────────────
        float targetA = (seqState == SEQ_TONE_A) ? 1.f : 0.f;
        float targetB = (seqState == SEQ_TONE_B) ? 1.f : 0.f;
        float targetCtx = (seqState == SEQ_CONTEXT && contextMode > 0) ? 1.f : 0.f;

        gainA += (targetA - gainA) * smoothK;
        gainB += (targetB - gainB) * smoothK;
        gainCtx += (targetCtx - gainCtx) * smoothK;

        // Snap tiny residuals to zero so we can skip work and avoid
        // denormal issues.
        if (gainA < 1e-5f)
            gainA = 0.f;
        if (gainB < 1e-5f)
            gainB = 0.f;
        if (gainCtx < 1e-5f)
            gainCtx = 0.f;

        // ── Render each active tone ──────────────────────────────────
        float outL = 0.f, outR = 0.f;
        renderBank(bankA, phasesA, gainA, dt, outL, outR);
        renderBank(bankB, phasesB, gainB, dt, outL, outR);
        if (contextMode > 0)
            renderBank(bankC, phasesC, gainCtx, dt, outL, outR);

        if (monoOut)
        {
            float m = 0.5f * (outL + outR);
            outL = m;
            outR = m;
        }

        // Soft saturation, then output level
        outL = 5.f * std::tanh(outL * 0.8f) * levelGain;
        outR = 5.f * std::tanh(outR * 0.8f) * levelGain;

        outputs[OUT_L_OUTPUT].setVoltage(outL);
        outputs[OUT_R_OUTPUT].setVoltage(outR);

        outputs[GATE_A_OUTPUT].setVoltage(seqState == SEQ_TONE_A ? 10.f : 0.f);
        outputs[GATE_B_OUTPUT].setVoltage(seqState == SEQ_TONE_B ? 10.f : 0.f);
        outputs[EOC_OUTPUT].setVoltage(eocPulse.process(dt) ? 10.f : 0.f);

        lights[PLAY_LIGHT].setBrightness(seqState != SEQ_IDLE ? 1.f : 0.f);
    }

    void startSequence()
    {
        seqTimer = 0.f;
        int contextMode = (int)std::round(params[CONTEXT_PARAM].getValue());
        seqState = (contextMode > 0) ? SEQ_CONTEXT : SEQ_TONE_A;
        // Note: phases and gains are NOT reset - the smoothed gains
        // automatically fade in from silence without a click.
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "monoOut", json_boolean(monoOut));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *m = json_object_get(root, "monoOut");
        if (m)
            monoOut = json_is_true(m);
    }
};

// ------------------------------------------------------------
// Panel background (static artwork)
// ------------------------------------------------------------
namespace
{
    namespace ag = agmm;

    struct TPPanel : TransparentWidget
    {
        TPPanel() { box.size = Vec(TP_PANEL_W, TP_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, TP_PANEL_W, TP_PANEL_H, "TRITONE PARADOX");
            ag::bezel(vg, TP_DISP_X, TP_DISP_Y, TP_DISP_W, TP_DISP_H);

            // Row 1: the tone pair and its spectral envelope
            ag::sectionHeader(vg, TP_PANEL_W, TP_R1_HDR, "SHEPARD TONES", 10.f);
            const char *row1[6] = {"PITCH", "TRITONE", "PARTIALS", "ENV CENTER", "ENV WIDTH", "SWEEP"};
            for (int i = 0; i < 6; ++i)
                ag::cell(vg, tpCellX(i, 6), TP_R1_TOP, 62.f, TP_CELL_H, row1[i]);

            // Row 2: sequencing and output
            ag::sectionHeader(vg, TP_PANEL_W, TP_R2_HDR, "SEQUENCE / OUTPUT", 10.f);
            const char *row2[7] = {"MODE", "CONTEXT", "RATE", "FADE", "SPREAD", "LEVEL", "PLAY"};
            for (int i = 0; i < 7; ++i)
                ag::cell(vg, tpCellX(i, 7), TP_R2_TOP, 54.f, TP_CELL_H, row2[i]);

            // Row 3: inputs on the left, outputs (tinted) on the right
            ag::sectionHeader(vg, TP_PANEL_W, TP_R3_HDR, "I / O", 10.f);
            const char *row3[9] = {"TRIG", "V/OCT", "ENV CV", "RATE CV", "OUT L", "OUT R", "GATE A", "GATE B", "EOC"};
            for (int i = 0; i < 9; ++i)
                ag::cell(vg, tpCellX(i, 9), TP_R3_TOP, 41.f, TP_CELL_H, row3[i], i >= 4, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: spectral envelope with partials, and pitch-class circle
    // ------------------------------------------------------------
    struct TPDisplay : TransparentWidget
    {
        TritoneParadox *module = nullptr;

        TPDisplay(TritoneParadox *m) : module(m)
        {
            box.pos = Vec(TP_DISP_X, TP_DISP_Y);
            box.size = Vec(TP_DISP_W, TP_DISP_H);
        }

        // Frequency to x position inside the envelope plot (display coordinates).
        static float mapX(float hz)
        {
            float t = (std::log2(std::max(hz, 1.f)) - TP_LOG_MIN) / (TP_LOG_MAX - TP_LOG_MIN);
            return TP_ENV_X + t * TP_ENV_W;
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, TP_DISP_W, TP_DISP_H);

            // Header separator
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, TP_HDR_H + 0.5f);
            nvgLineTo(vg, TP_DISP_W - 8.f, TP_HDR_H + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, TP_ENV_X, TP_PLOT_Y, TP_ENV_W, TP_PLOT_H);
            ag::plotFrame(vg, TP_CHR_X, TP_PLOT_Y, TP_CHR_W, TP_PLOT_H);

            // Octave grid and axis labels (labels sit below the plot frame)
            static const float gridHz[6] = {100.f, 200.f, 400.f, 800.f, 1600.f, 3200.f};
            static const char *gridTxt[6] = {"100", "200", "400", "800", "1.6k", "3.2k"};
            nvgBeginPath(vg);
            for (int i = 0; i < 6; ++i)
            {
                float x = std::floor(mapX(gridHz[i])) + 0.5f;
                nvgMoveTo(vg, x, TP_PLOT_Y + 1.f);
                nvgLineTo(vg, x, TP_PLOT_Y + TP_PLOT_H - 1.f);
            }
            float midY = std::floor(TP_PLOT_Y + TP_PLOT_H * 0.5f) + 0.5f;
            nvgMoveTo(vg, TP_ENV_X + 1.f, midY);
            nvgLineTo(vg, TP_ENV_X + TP_ENV_W - 1.f, midY);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            for (int i = 0; i < 6; ++i)
                ag::text(vg, mapX(gridHz[i]), TP_PLOT_Y + TP_PLOT_H + 9.f, gridTxt[i], 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, TP_ENV_X + TP_ENV_W, TP_PLOT_Y + TP_PLOT_H + 9.f, "Hz", 7.f, ag::dim(), ag::ALIGN_R, 0.f);

            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            // Snapshot (defaults give a sensible preview in the module browser)
            float pitchSemis = 0.f;
            float envCenterHz = 370.f;
            float envWidth = 1.5f;
            int partials = 6;
            float tritoneSemis = 6.f;
            float gainA = 1.f;
            float gainB = 0.35f;
            int seqState = TritoneParadox::SEQ_TONE_A;
            int contextMode = 0;
            bool autoMode = false;

            if (module)
            {
                pitchSemis = module->dispPitchSemis;
                envCenterHz = module->dispEnvCenterHz;
                envWidth = module->dispEnvWidth;
                partials = module->dispPartials;
                tritoneSemis = module->dispTritoneSemis;
                gainA = module->dispGainA;
                gainB = module->dispGainB;
                seqState = module->dispSeqState;
                contextMode = module->dispContextMode;
                autoMode = module->dispAuto;
            }
            partials = rack::math::clamp(partials, 1, TritoneParadox::TP_MAX_PARTIALS);
            seqState = rack::math::clamp(seqState, 0, 3);
            envWidth = std::max(envWidth, 0.1f);

            // ── Header: pitch pair, envelope and state chips ───────────
            char noteA[16], noteB[16], buf[64];
            tpFreqToNote(tpSemisToHz(pitchSemis), noteA, sizeof(noteA));
            tpFreqToNote(tpSemisToHz(pitchSemis + tritoneSemis), noteB, sizeof(noteB));
            snprintf(buf, sizeof(buf), "%s > %s   ENV %.0f Hz   W %.2f", noteA, noteB, envCenterHz, envWidth);
            ag::text(vg, 8.f, TP_HDR_H * 0.5f, buf, 9.f, ag::bright(), ag::ALIGN_L, 0.5f);

            const float chipY = 3.f, chipH = 12.f, gap = 4.f;
            float x = TP_DISP_W - 8.f;
            x -= 20.f;
            ag::chip(vg, x, chipY, 20.f, chipH, "B", seqState == TritoneParadox::SEQ_TONE_B, ag::warm());
            x -= gap + 20.f;
            ag::chip(vg, x, chipY, 20.f, chipH, "A", seqState == TritoneParadox::SEQ_TONE_A, ag::accent());
            x -= gap + 32.f;
            const char *ctxTxt = contextMode == 1 ? "CTX-" : (contextMode == 2 ? "CTX+" : "CTX");
            ag::chip(vg, x, chipY, 32.f, chipH, ctxTxt, seqState == TritoneParadox::SEQ_CONTEXT, ag::violet());
            x -= gap + 30.f;
            ag::chip(vg, x, chipY, 30.f, chipH, autoMode ? "AUTO" : "MAN", autoMode, ag::accent());

            // ── Envelope plot ──────────────────────────────────────────
            {
                nvgSave(vg);
                const float plotX = TP_ENV_X, plotY = TP_PLOT_Y, plotW = TP_ENV_W, plotH = TP_PLOT_H;
                const float bottom = plotY + plotH;
                const float scale = plotH - 6.f;
                nvgScissor(vg, plotX, plotY, plotW, plotH);

                // Envelope curve: flat translucent area plus a 1 px line
                const int N = 118;
                nvgBeginPath(vg);
                nvgMoveTo(vg, plotX, bottom);
                for (int k = 0; k <= N; ++k)
                {
                    float t = (float)k / (float)N;
                    float f = std::pow(2.f, TP_LOG_MIN + t * (TP_LOG_MAX - TP_LOG_MIN));
                    float w = tpEnvelopeWeight(f, envCenterHz, envWidth);
                    nvgLineTo(vg, plotX + t * plotW, bottom - 1.f - w * scale);
                }
                nvgLineTo(vg, plotX + plotW, bottom);
                nvgClosePath(vg);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.10f));
                nvgFill(vg);

                nvgBeginPath(vg);
                for (int k = 0; k <= N; ++k)
                {
                    float t = (float)k / (float)N;
                    float f = std::pow(2.f, TP_LOG_MIN + t * (TP_LOG_MAX - TP_LOG_MIN));
                    float w = tpEnvelopeWeight(f, envCenterHz, envWidth);
                    float px = plotX + t * plotW;
                    float py = bottom - 1.f - w * scale;
                    if (k == 0)
                        nvgMoveTo(vg, px, py);
                    else
                        nvgLineTo(vg, px, py);
                }
                nvgStrokeColor(vg, ag::withA(ag::bright(), 0.75f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Envelope centre marker
                float cxm = std::floor(mapX(envCenterHz)) + 0.5f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, cxm, plotY + 1.f);
                nvgLineTo(vg, cxm, bottom - 1.f);
                nvgStrokeColor(vg, ag::withA(ag::bright(), 0.30f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Partial bars: tone A cyan, tone B orange. A sounding tone
                // is drawn solid, a silent one dimmed.
                float baseA = tpSemisToHz(pitchSemis);
                float baseB = tpSemisToHz(pitchSemis + tritoneSemis);
                float alphaA = 0.30f + 0.70f * rack::math::clamp(gainA, 0.f, 1.f);
                float alphaB = 0.30f + 0.70f * rack::math::clamp(gainB, 0.f, 1.f);
                for (int i = 0; i < partials; ++i)
                {
                    float fA = baseA * (float)(1 << i);
                    float fB = baseB * (float)(1 << i);
                    float aA = tpEnvelopeWeight(fA, envCenterHz, envWidth);
                    float aB = tpEnvelopeWeight(fB, envCenterHz, envWidth);
                    if (aA > 0.005f)
                    {
                        float h = aA * scale;
                        nvgBeginPath(vg);
                        nvgRect(vg, mapX(fA) - 1.5f, bottom - 1.f - h, 3.f, h);
                        nvgFillColor(vg, ag::withA(ag::accent(), alphaA));
                        nvgFill(vg);
                    }
                    if (aB > 0.005f)
                    {
                        float h = aB * scale;
                        nvgBeginPath(vg);
                        nvgRect(vg, mapX(fB) - 1.5f, bottom - 1.f - h, 3.f, h);
                        nvgFillColor(vg, ag::withA(ag::warm(), alphaB));
                        nvgFill(vg);
                    }
                }

                // Baseline
                nvgBeginPath(vg);
                nvgMoveTo(vg, plotX + 1.f, bottom - 0.5f);
                nvgLineTo(vg, plotX + plotW - 1.f, bottom - 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                nvgRestore(vg);
            }

            // ── Pitch-class circle ─────────────────────────────────────
            {
                nvgSave(vg);
                const float cx = TP_CHR_X + TP_CHR_W * 0.5f;
                const float cy = TP_PLOT_Y + TP_PLOT_H * 0.5f;
                const float r = 21.f;

                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, r);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 40));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                auto pcAngle = [](float semis) -> float
                {
                    float pc = std::fmod(semis, 12.f);
                    if (pc < 0.f)
                        pc += 12.f;
                    return -(float)M_PI * 0.5f + (pc / 12.f) * 2.f * (float)M_PI;
                };

                // 12 pitch-class ticks
                for (int i = 0; i < 12; ++i)
                {
                    float a = pcAngle((float)i);
                    nvgBeginPath(vg);
                    nvgCircle(vg, cx + r * std::cos(a), cy + r * std::sin(a), i == 0 ? 2.f : 1.2f);
                    nvgFillColor(vg, i == 0 ? ag::withA(ag::textCol(), 0.7f) : ag::withA(ag::dim(), 0.7f));
                    nvgFill(vg);
                }

                float angA = pcAngle(pitchSemis);
                float angB = pcAngle(pitchSemis + tritoneSemis);
                float ax = cx + r * std::cos(angA), ay = cy + r * std::sin(angA);
                float bx = cx + r * std::cos(angB), by = cy + r * std::sin(angB);

                // Chord between the two pitch classes
                nvgBeginPath(vg);
                nvgMoveTo(vg, ax, ay);
                nvgLineTo(vg, bx, by);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 55));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Markers (solid when that tone sounds)
                nvgBeginPath(vg);
                nvgCircle(vg, ax, ay, 4.f);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.45f + 0.55f * rack::math::clamp(gainA, 0.f, 1.f)));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, bx, by, 4.f);
                nvgFillColor(vg, ag::withA(ag::warm(), 0.45f + 0.55f * rack::math::clamp(gainB, 0.f, 1.f)));
                nvgFill(vg);

                // Note names just outside the circle
                ag::text(vg, cx + (r + 9.f) * std::cos(angA), cy + (r + 9.f) * std::sin(angA), noteA, 7.f, ag::bright(), ag::ALIGN_C, 0.f);
                ag::text(vg, cx + (r + 9.f) * std::cos(angB), cy + (r + 9.f) * std::sin(angB), noteB, 7.f, ag::warm(), ag::ALIGN_C, 0.f);

                nvgRestore(vg);

                snprintf(buf, sizeof(buf), "INTERVAL %.0f ST", tritoneSemis);
                ag::text(vg, cx, TP_PLOT_Y + TP_PLOT_H + 9.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            }

            Widget::drawLayer(args, layer);
        }
    };
} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct TritoneParadoxWidget : ModuleWidget
{
    TritoneParadoxWidget(TritoneParadox *module)
    {
        setModule(module);
        box.size = Vec(TP_PANEL_W, TP_PANEL_H);

        addChild(new TPPanel());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new TPDisplay(module));

        // Row 1: PITCH, TRITONE, PARTIALS, ENV CENTER, ENV WIDTH, SWEEP
        const float y1 = TP_R1_TOP + TP_KNOB_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(0, 6), y1), module, TritoneParadox::PITCH_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(1, 6), y1), module, TritoneParadox::TRITONE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(2, 6), y1), module, TritoneParadox::PARTIALS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(3, 6), y1), module, TritoneParadox::ENV_CENTER_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(4, 6), y1), module, TritoneParadox::ENV_WIDTH_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(5, 6), y1), module, TritoneParadox::SWEEP_PARAM));

        // Row 2: MODE, CONTEXT, RATE, FADE, SPREAD, LEVEL, PLAY
        const float y2 = TP_R2_TOP + TP_KNOB_DY;
        addParam(createParamCentered<CKSS>(Vec(tpCellX(0, 7), y2), module, TritoneParadox::MODE_PARAM));
        addParam(createParamCentered<CKSSThree>(Vec(tpCellX(1, 7), y2), module, TritoneParadox::CONTEXT_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(2, 7), y2), module, TritoneParadox::RATE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(3, 7), y2), module, TritoneParadox::FADE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(4, 7), y2), module, TritoneParadox::SPREAD_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(tpCellX(5, 7), y2), module, TritoneParadox::LEVEL_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(tpCellX(6, 7), y2), module, TritoneParadox::PLAY_PARAM));
        addChild(createLightCentered<MediumLight<agmm::CyanLight>>(Vec(tpCellX(6, 7), y2), module, TritoneParadox::PLAY_LIGHT));

        // Row 3: inputs left, outputs right
        const float y3 = TP_R3_TOP + TP_KNOB_DY;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(tpCellX(0, 9), y3), module, TritoneParadox::TRIG_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(tpCellX(1, 9), y3), module, TritoneParadox::VOCT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(tpCellX(2, 9), y3), module, TritoneParadox::ENV_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(tpCellX(3, 9), y3), module, TritoneParadox::RATE_CV_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(tpCellX(4, 9), y3), module, TritoneParadox::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(tpCellX(5, 9), y3), module, TritoneParadox::OUT_R_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(tpCellX(6, 9), y3), module, TritoneParadox::GATE_A_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(tpCellX(7, 9), y3), module, TritoneParadox::GATE_B_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(tpCellX(8, 9), y3), module, TritoneParadox::EOC_OUTPUT));
        agmm::addKnobArcs(this, 2.f);
    }

    void appendContextMenu(Menu *menu) override
    {
        agmm::addYouTubeLink(menu); // adds "Visit my youtube channel!"
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        ModuleWidget::appendContextMenu(menu);
        TritoneParadox *m = dynamic_cast<TritoneParadox *>(module);
        if (!m)
            return;
        menu->addChild(new MenuSeparator);
        menu->addChild(createBoolPtrMenuItem("Mono output (sum L + R)", "", &m->monoOut));
    }
};

Model *modelTritoneParadox = createModel<TritoneParadox, TritoneParadoxWidget>("TritoneParadox");