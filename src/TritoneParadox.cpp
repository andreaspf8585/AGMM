#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>

// ============================================================================
// TritoneParadox — Shepard‑tone pair generator for the tritone paradox
// ============================================================================
//
// OVERVIEW
// --------
// The tritone paradox (Deutsch, 1986) is an auditory illusion in which two
// Shepard tones separated by a tritone (6 semitones) are heard as ascending
// by some listeners and descending by others. The direction of perceived
// motion depends on the absolute frequencies of the tones and on the position
// of the fixed spectral envelope that weights the octave‑spaced partials.
//
// This module generates exactly that stimulus: a pair of Shepard tones whose
// pitch classes differ by a tritone. The spectral envelope is a Gaussian in
// log‑frequency space, centred at a user‑adjustable frequency. Shifting the
// envelope centre changes the perceptual bias, allowing you to explore the
// paradox yourself.
//
// A context tone (optional) can be played before the pair, as in recent
// studies showing that a preceding lower or higher tone biases the perceived
// direction (Chambers et al., 2017).
//
// Controls:
//   PITCH      pitch class of the first tone (default C4). Shown as note name.
//   ENV CENTER centre frequency of the Gaussian spectral envelope (Hz).
//   ENV WIDTH  width (sigma) of the envelope in octaves.
//   PARTIALS   number of octave‑spaced partials per tone (1..20, default 6).
//   TRITONE    interval between the two tones (default 6 semitones).
//   MODE       Manual (trigger/gate) or Auto‑cycle.
//   RATE       speed of the auto‑cycle (0.1..200 Hz, exponential).
//   CONTEXT    Off / Lower / Higher – adds a context tone before the pair.
//
// Inputs:
//   TRIG       rising edge starts a sequence in Manual mode.
//
// Outputs:
//   OUT L / OUT R  stereo (partials panned by their position in the envelope).
//
// Display:
//   – Gaussian envelope curve over log frequency.
//   – Partials of Tone A (blue) and Tone B (orange) with their amplitudes.
//   – Chroma circle showing the two pitch classes.
// ============================================================================

// ------------------------------------------------------------
// Layout constants (24 HP = 360 px wide)
// ------------------------------------------------------------
static const float TP_PANEL_W = RACK_GRID_WIDTH * 24; // 360
static const float TP_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (320 x 140)
static const float TP_DISP_W = 320.f;
static const float TP_DISP_H = 140.f;
static const float TP_DISP_X = (TP_PANEL_W - TP_DISP_W) / 2.f; // 20
static const float TP_DISP_Y = 24.f;

// Row 1 (PITCH, ENV CENTER, ENV WIDTH, PARTIALS) — 4 knobs
static const float TP_CELL_W = 76.f;
static const float TP_CELL_H = 58.f;
static const float TP_ROW1_TOP = 174.f;
static const float TP_ROW1_Y = TP_ROW1_TOP + 32.f; // 206
static const float TP_ROW1_X[4] = {45.f, 135.f, 225.f, 315.f};

// Row 2 (TRITONE, MODE, RATE, CONTEXT) — 4 controls
static const float TP_ROW2_TOP = 248.f;
static const float TP_ROW2_Y = TP_ROW2_TOP + 32.f; // 280
static const float TP_ROW2_X[4] = {45.f, 135.f, 225.f, 315.f};

// I/O row (TRIG, L OUT, R OUT) — 3 jacks
static const float TP_IO_TOP = 324.f;
static const float TP_IO_H = 46.f;
static const float TP_IO_Y = TP_IO_TOP + 26.f; // 350
static const float TP_IO_X[3] = {90.f, 180.f, 270.f};

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
        int octave = midiInt / 12 - 1;
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

    // Sine‑only waveform (Shepard tones are traditionally pure sinusoids).
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
    enum ParamId
    {
        PITCH_PARAM,
        ENV_CENTER_PARAM,
        ENV_WIDTH_PARAM,
        PARTIALS_PARAM,
        TRITONE_PARAM,
        MODE_PARAM,    // 0 = Manual, 1 = Auto
        RATE_PARAM,
        CONTEXT_PARAM, // 0 = Off, 1 = Lower, 2 = Higher
        PARAMS_LEN
    };
    enum InputId
    {
        TRIG_INPUT,
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

    static const int TP_MAX_PARTIALS = 20;

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

    // ── DSP state ────────────────────────────────────────────────────
    // Phase accumulators for Tone A and Tone B (interleaved for simplicity)
    float phases[2 * TP_MAX_PARTIALS] = {};
    float phasesContext[TP_MAX_PARTIALS] = {}; // for context tone

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

    // Display snapshot
    float dispPitchSemis = 0.f;
    float dispEnvCenterHz = 370.f;
    float dispEnvWidth = 1.5f;
    int dispPartials = 6;
    float dispTritoneSemis = 6.f;
    float dispToneAFreq[TP_MAX_PARTIALS] = {};
    float dispToneAAmp[TP_MAX_PARTIALS] = {};
    float dispToneBFreq[TP_MAX_PARTIALS] = {};
    float dispToneBAmp[TP_MAX_PARTIALS] = {};
    int dispSeqState = SEQ_IDLE;
    float dispEnvCurve[64] = {};
    float dispEnvCurveFreq[64] = {};

    TritoneParadox()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        auto *pq = configParam<PitchQuantity>(PITCH_PARAM, -48.f, 48.f, 0.f, "Pitch", " st");
        pq->snapEnabled = true;
        paramQuantities[PITCH_PARAM]->snapEnabled = true;

        configParam(ENV_CENTER_PARAM, 65.f, 2000.f, 370.f, "Envelope centre", " Hz");
        configParam(ENV_WIDTH_PARAM, 0.5f, 3.f, 1.5f, "Envelope width", " oct");
        configParam<PartialsQuantity>(PARTIALS_PARAM, 1.f, (float)TP_MAX_PARTIALS, 6.f, "Partials", "", 0.f, 1.f);
        paramQuantities[PARTIALS_PARAM]->snapEnabled = true;
        configParam(TRITONE_PARAM, 5.f, 7.f, 6.f, "Tritone interval", " st");
        paramQuantities[TRITONE_PARAM]->snapEnabled = true;
        configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Mode", {"Manual", "Auto"});

        // RATE: exponential 0.1 .. 200 Hz (raw 0..1). Default raw 0.303 ≈ 1 Hz.
        configParam<RateQuantity>(RATE_PARAM, 0.f, 1.f, 0.303f, "Cycle rate");

        configSwitch(CONTEXT_PARAM, 0.f, 2.f, 0.f, "Context tone", {"Off", "Lower", "Higher"});

        configInput(TRIG_INPUT, "Trigger");
        configOutput(OUT_L_OUTPUT, "L out");
        configOutput(OUT_R_OUTPUT, "R out");
    }

    void process(const ProcessArgs &args) override
    {
        const float sr = args.sampleRate;

        // ── Read controls ────────────────────────────────────────────
        float pitchSemis = params[PITCH_PARAM].getValue();
        float envCenterHz = params[ENV_CENTER_PARAM].getValue();
        float envWidth = params[ENV_WIDTH_PARAM].getValue();
        int partials = rack::math::clamp((int)std::round(params[PARTIALS_PARAM].getValue()), 1, TP_MAX_PARTIALS);
        float tritoneSemis = params[TRITONE_PARAM].getValue();
        bool autoMode = params[MODE_PARAM].getValue() > 0.5f;
        float rate = RateQuantity::rawToHz(params[RATE_PARAM].getValue());
        int contextMode = (int)std::round(params[CONTEXT_PARAM].getValue());

        // ── Sequencer (timing only, gains handle the smooth transitions) ──
        bool trig = inputs[TRIG_INPUT].getVoltage() > 1.f;
        if (trig && !lastTrig && seqState == SEQ_IDLE)
            startSequence();
        lastTrig = trig;

        if (autoMode && seqState == SEQ_IDLE)
            startSequence();

        if (seqState != SEQ_IDLE)
        {
            seqTimer += args.sampleTime;
            float stateDuration = 1.f / rate;
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

        // ── Compute tone frequencies and amplitudes (also feeds display) ──
        float toneAFreq[TP_MAX_PARTIALS] = {};
        float toneAAmp[TP_MAX_PARTIALS] = {};
        float toneBFreq[TP_MAX_PARTIALS] = {};
        float toneBAmp[TP_MAX_PARTIALS] = {};
        float ctxFreq[TP_MAX_PARTIALS] = {};
        float ctxAmp[TP_MAX_PARTIALS] = {};

        for (int i = 0; i < partials; ++i)
        {
            float fA = tpSemisToHz(pitchSemis) * std::pow(2.f, (float)i);
            toneAFreq[i] = fA;
            toneAAmp[i] = tpEnvelopeWeight(fA, envCenterHz, envWidth);

            float fB = tpSemisToHz(pitchSemis + tritoneSemis) * std::pow(2.f, (float)i);
            toneBFreq[i] = fB;
            toneBAmp[i] = tpEnvelopeWeight(fB, envCenterHz, envWidth);

            if (contextMode > 0)
            {
                float contextShift = (contextMode == 1) ? -12.f : 12.f;
                float fC = tpSemisToHz(pitchSemis + contextShift) * std::pow(2.f, (float)i);
                ctxFreq[i] = fC;
                ctxAmp[i] = tpEnvelopeWeight(fC, envCenterHz, envWidth);
            }
        }

        // ── Anti-click: smoothed per-state gains ─────────────────────
        // Each state has a target gain (1 if active, 0 otherwise). A
        // one-pole smoother with a ~5 ms time constant crossfades
        // between states, eliminating discontinuities at transitions
        // and at rate changes mid-playback.
        float targetA = (seqState == SEQ_TONE_A) ? 1.f : 0.f;
        float targetB = (seqState == SEQ_TONE_B) ? 1.f : 0.f;
        float targetCtx = (seqState == SEQ_CONTEXT && contextMode > 0) ? 1.f : 0.f;

        const float fadeTau = 0.005f; // 5 ms
        const float kSmooth = 1.f - std::exp(-args.sampleTime / fadeTau);

        gainA += (targetA - gainA) * kSmooth;
        gainB += (targetB - gainB) * kSmooth;
        gainCtx += (targetCtx - gainCtx) * kSmooth;

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

        auto renderOneTone = [&](float *freqs, float *amps, float *phaseArr,
                                 int offset, float gain)
        {
            if (gain < 1e-5f)
                return;
            float tL = 0.f, tR = 0.f, sumSq = 0.f;
            for (int i = 0; i < partials; ++i)
            {
                float amp = amps[i];
                if (amp < 1e-4f)
                    continue;

                float fc = rack::math::clamp(freqs[i], 10.f, sr * 0.45f);
                phaseArr[offset + i] += fc * args.sampleTime;
                phaseArr[offset + i] -= std::floor(phaseArr[offset + i]);

                float s = tpWaveSample(phaseArr[offset + i]);

                float log2Ratio = std::log2(std::max(fc, 1.f) / envCenterHz);
                float pan = rack::math::clamp(log2Ratio / envWidth * 0.55f, -1.f, 1.f);
                float ang = (pan + 1.f) * 0.25f * (float)M_PI;

                tL += s * amp * std::cos(ang);
                tR += s * amp * std::sin(ang);
                sumSq += amp * amp;
            }
            // Normalise this tone independently so its RMS is consistent
            // across partial counts, then apply its smoothed gain.
            if (sumSq > 0.001f)
            {
                float inv = 1.8f / std::sqrt(sumSq);
                tL *= inv;
                tR *= inv;
            }
            outL += tL * gain;
            outR += tR * gain;
        };

        if (gainA > 0.f)
            renderOneTone(toneAFreq, toneAAmp, phases, 0, gainA);
        if (gainB > 0.f)
            renderOneTone(toneBFreq, toneBAmp, phases, TP_MAX_PARTIALS, gainB);
        if (gainCtx > 0.f && contextMode > 0)
            renderOneTone(ctxFreq, ctxAmp, phasesContext, 0, gainCtx);

        // Soft saturation
        outL = 5.f * std::tanh(outL * 0.8f);
        outR = 5.f * std::tanh(outR * 0.8f);

        outputs[OUT_L_OUTPUT].setVoltage(outL);
        outputs[OUT_R_OUTPUT].setVoltage(outR);

        // ── Display snapshot ────────────────────────────────────────
        dispPitchSemis = pitchSemis;
        dispEnvCenterHz = envCenterHz;
        dispEnvWidth = envWidth;
        dispPartials = partials;
        dispTritoneSemis = tritoneSemis;
        dispSeqState = (int)seqState;

        for (int i = 0; i < partials; ++i)
        {
            dispToneAFreq[i] = toneAFreq[i];
            dispToneAAmp[i] = toneAAmp[i];
            dispToneBFreq[i] = toneBFreq[i];
            dispToneBAmp[i] = toneBAmp[i];
        }
        for (int i = partials; i < TP_MAX_PARTIALS; ++i)
        {
            dispToneAFreq[i] = 0.f;
            dispToneAAmp[i] = 0.f;
            dispToneBFreq[i] = 0.f;
            dispToneBAmp[i] = 0.f;
        }

        for (int k = 0; k < 64; ++k)
        {
            float t = (float)k / 63.f;
            float logF = std::log2(50.f) + t * (std::log2(4000.f) - std::log2(50.f));
            float f = std::pow(2.f, logF);
            dispEnvCurveFreq[k] = f;
            dispEnvCurve[k] = tpEnvelopeWeight(f, envCenterHz, envWidth);
        }
    }

    void startSequence()
    {
        seqTimer = 0.f;
        int contextMode = (int)std::round(params[CONTEXT_PARAM].getValue());
        seqState = (contextMode > 0) ? SEQ_CONTEXT : SEQ_TONE_A;
        // Note: phases and gains are NOT reset — the smoothed gains
        // automatically fade in from silence without a click.
    }

    json_t *dataToJson() override
    {
        return json_object();
    }

    void dataFromJson(json_t *root) override
    {
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

            ag::sectionHeader(vg, TP_PANEL_W, 166.f, "SHEPARD TONES", 10.f);
            const char *row1Labels[4] = {"PITCH", "ENV CENTER", "ENV WIDTH", "PARTIALS"};
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, TP_ROW1_X[i], TP_ROW1_TOP, TP_CELL_W, TP_CELL_H, row1Labels[i]);

            ag::sectionHeader(vg, TP_PANEL_W, 240.f, "PARADOX", 10.f);
            const char *row2Labels[4] = {"TRITONE", "MODE", "RATE", "CONTEXT"};
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, TP_ROW2_X[i], TP_ROW2_TOP, TP_CELL_W, TP_CELL_H, row2Labels[i]);

            ag::sectionHeader(vg, TP_PANEL_W, 316.f, "I / O", 10.f);
            ag::cell(vg, TP_IO_X[0], TP_IO_TOP, TP_CELL_W, TP_IO_H, "TRIG");
            ag::cell(vg, TP_IO_X[1], TP_IO_TOP, TP_CELL_W, TP_IO_H, "OUT L", true);
            ag::cell(vg, TP_IO_X[2], TP_IO_TOP, TP_CELL_W, TP_IO_H, "OUT R", true);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: envelope, partials, chroma circle
    // ------------------------------------------------------------
    struct TPDisplay : TransparentWidget
    {
        TritoneParadox *module = nullptr;

        TPDisplay(TritoneParadox *m) : module(m)
        {
            box.pos = Vec(TP_DISP_X, TP_DISP_Y);
            box.size = Vec(TP_DISP_W, TP_DISP_H);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, TP_DISP_W, TP_DISP_H);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, 15.5f);
            nvgLineTo(vg, TP_DISP_W - 8.f, 15.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, 8.f, 20.f, 200.f, 100.f);
            ag::plotFrame(vg, 216.f, 20.f, 96.f, 100.f);

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

            float pitchSemis = 0.f;
            float envCenterHz = 370.f;
            float envWidth = 1.5f;
            int partials = 6;
            float tritoneSemis = 6.f;
            float toneAFreq[TritoneParadox::TP_MAX_PARTIALS] = {};
            float toneAAmp[TritoneParadox::TP_MAX_PARTIALS] = {};
            float toneBFreq[TritoneParadox::TP_MAX_PARTIALS] = {};
            float toneBAmp[TritoneParadox::TP_MAX_PARTIALS] = {};
            int seqState = TritoneParadox::SEQ_IDLE;
            float envCurve[64] = {};
            float envCurveFreq[64] = {};

            if (module)
            {
                pitchSemis = module->dispPitchSemis;
                envCenterHz = module->dispEnvCenterHz;
                envWidth = module->dispEnvWidth;
                partials = module->dispPartials;
                tritoneSemis = module->dispTritoneSemis;
                seqState = module->dispSeqState;
                for (int i = 0; i < TritoneParadox::TP_MAX_PARTIALS; ++i)
                {
                    toneAFreq[i] = module->dispToneAFreq[i];
                    toneAAmp[i] = module->dispToneAAmp[i];
                    toneBFreq[i] = module->dispToneBFreq[i];
                    toneBAmp[i] = module->dispToneBAmp[i];
                }
                for (int k = 0; k < 64; ++k)
                {
                    envCurve[k] = module->dispEnvCurve[k];
                    envCurveFreq[k] = module->dispEnvCurveFreq[k];
                }
            }
            else
            {
                envCenterHz = 370.f;
                envWidth = 1.5f;
                for (int k = 0; k < 64; ++k)
                {
                    float t = (float)k / 63.f;
                    float logF = std::log2(50.f) + t * (std::log2(4000.f) - std::log2(50.f));
                    envCurveFreq[k] = std::pow(2.f, logF);
                    envCurve[k] = tpEnvelopeWeight(envCurveFreq[k], envCenterHz, envWidth);
                }
            }

            char buf[64];
            char note[16];
            tpFreqToNote(tpSemisToHz(pitchSemis), note, sizeof(note));
            snprintf(buf, sizeof(buf), "%s  |  %.0f Hz  |  σ=%.2f oct", note, envCenterHz, envWidth);
            ag::text(vg, 8.f, 8.f, buf, 10.f, ag::bright(), ag::ALIGN_L, 0.5f);

            const char *stateNames[] = {"IDLE", "CONTEXT", "TONE A", "TONE B"};
            ag::text(vg, TP_DISP_W - 8.f, 8.f, stateNames[seqState], 8.f,
                     seqState == TritoneParadox::SEQ_TONE_A ? ag::accent() : ag::warm(),
                     ag::ALIGN_R, 0.5f);

            // ── Envelope + partials plot (left sub‑frame) ──────────────
            {
                nvgSave(vg);
                const float plotX = 8.f, plotY = 20.f, plotW = 200.f, plotH = 100.f;
                nvgScissor(vg, plotX, plotY, plotW, plotH);

                auto mapFreqToX = [&](float f) -> float
                {
                    float logF = std::log2(std::max(f, 1.f));
                    float logMin = std::log2(50.f);
                    float logMax = std::log2(4000.f);
                    float t = (logF - logMin) / (logMax - logMin);
                    return plotX + t * plotW;
                };

                nvgBeginPath(vg);
                for (int k = 0; k < 64; ++k)
                {
                    float x = mapFreqToX(envCurveFreq[k]);
                    float y = plotY + plotH - envCurve[k] * (plotH - 4.f);
                    if (k == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, ag::withA(ag::bright(), 0.5f));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);

                for (int i = 0; i < partials; ++i)
                {
                    if (toneAAmp[i] > 0.005f)
                    {
                        float x = mapFreqToX(toneAFreq[i]);
                        float h = toneAAmp[i] * (plotH - 4.f);
                        nvgBeginPath(vg);
                        nvgRect(vg, x - 1.f, plotY + plotH - 1.f - h, 2.f, h);
                        nvgFillColor(vg, ag::withA(ag::bright(), 0.8f));
                        nvgFill(vg);
                    }
                    if (toneBAmp[i] > 0.005f)
                    {
                        float x = mapFreqToX(toneBFreq[i]);
                        float h = toneBAmp[i] * (plotH - 4.f);
                        nvgBeginPath(vg);
                        nvgRect(vg, x - 1.f, plotY + plotH - 1.f - h, 2.f, h);
                        nvgFillColor(vg, ag::withA(ag::warm(), 0.8f));
                        nvgFill(vg);
                    }
                }

                nvgBeginPath(vg);
                nvgMoveTo(vg, plotX + 1.f, plotY + plotH - 1.f);
                nvgLineTo(vg, plotX + plotW - 1.f, plotY + plotH - 1.f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                nvgRestore(vg);
            }

            // ── Chroma circle (right sub‑frame) ────────────────────────
            {
                nvgSave(vg);
                const float cx = 216.f + 48.f;
                const float cy = 20.f + 50.f;
                const float r = 36.f;

                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, r);
                nvgStrokeColor(vg, ag::withA(ag::bright(), 0.3f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                auto pitchClassAngle = [&](float semis) -> float
                {
                    float pc = std::fmod(semis, 12.f);
                    if (pc < 0)
                        pc += 12.f;
                    return -M_PI / 2.f + (pc / 12.f) * 2.f * (float)M_PI;
                };

                float angleA = pitchClassAngle(pitchSemis);
                float angleB = pitchClassAngle(pitchSemis + tritoneSemis);

                auto drawRadius = [&](float angle, NVGcolor color)
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, cx, cy);
                    nvgLineTo(vg, cx + r * std::cos(angle), cy + r * std::sin(angle));
                    nvgStrokeColor(vg, color);
                    nvgStrokeWidth(vg, 2.f);
                    nvgStroke(vg);
                };

                drawRadius(angleA, ag::bright());
                drawRadius(angleB, ag::warm());

                auto drawMarker = [&](float angle, NVGcolor color)
                {
                    nvgBeginPath(vg);
                    nvgCircle(vg, cx + r * std::cos(angle), cy + r * std::sin(angle), 4.f);
                    nvgFillColor(vg, color);
                    nvgFill(vg);
                };
                drawMarker(angleA, ag::bright());
                drawMarker(angleB, ag::warm());

                char labelA[8], labelB[8];
                tpFreqToNote(tpSemisToHz(pitchSemis), labelA, sizeof(labelA));
                tpFreqToNote(tpSemisToHz(pitchSemis + tritoneSemis), labelB, sizeof(labelB));

                float bA[4] = {0.f, 0.f, 0.f, 0.f};
                float bB[4] = {0.f, 0.f, 0.f, 0.f};
                nvgTextBounds(vg, 0.f, 0.f, labelA, nullptr, bA);
                nvgTextBounds(vg, 0.f, 0.f, labelB, nullptr, bB);
                float wA = bA[2] - bA[0];
                float wB = bB[2] - bB[0];

                ag::text(vg, cx - wA * 0.5f, cy - r - 6.f,  labelA, 7.f, ag::bright(), ag::ALIGN_L, 0.f);
                ag::text(vg, cx - wB * 0.5f, cy + r + 10.f, labelB, 7.f, ag::warm(),   ag::ALIGN_L, 0.f);

                nvgRestore(vg);
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

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new TPDisplay(module));

        addParam(createParamCentered<RoundBlackKnob>(Vec(TP_ROW1_X[0], TP_ROW1_Y), module, TritoneParadox::PITCH_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(Vec(TP_ROW1_X[1], TP_ROW1_Y), module, TritoneParadox::ENV_CENTER_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(TP_ROW1_X[2], TP_ROW1_Y), module, TritoneParadox::ENV_WIDTH_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(TP_ROW1_X[3], TP_ROW1_Y), module, TritoneParadox::PARTIALS_PARAM));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(TP_ROW2_X[0], TP_ROW2_Y), module, TritoneParadox::TRITONE_PARAM));
        addParam(createParamCentered<CKSS>(Vec(TP_ROW2_X[1], TP_ROW2_Y), module, TritoneParadox::MODE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(TP_ROW2_X[2], TP_ROW2_Y), module, TritoneParadox::RATE_PARAM));
        addParam(createParamCentered<CKSSThree>(Vec(TP_ROW2_X[3], TP_ROW2_Y), module, TritoneParadox::CONTEXT_PARAM));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(TP_IO_X[0], TP_IO_Y), module, TritoneParadox::TRIG_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(TP_IO_X[1], TP_IO_Y), module, TritoneParadox::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(TP_IO_X[2], TP_IO_Y), module, TritoneParadox::OUT_R_OUTPUT));
    }
};

Model *modelTritoneParadox = createModel<TritoneParadox, TritoneParadoxWidget>("TritoneParadox");