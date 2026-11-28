#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <random>
#include <algorithm>
#include "Knob_arcs.hpp"

// ============================================================
// PsyDelay
// ============================================================
//
// OVERVIEW
// --------
// PsyDelay is a 36 HP psychedelic stereo delay with a rich set of
// self-modulation features. Its core is a modulated stereo delay line
// (up to 2 seconds) with feedback, reverse playback, ping-pong and phase-flip
// stereo routing, plus an input gate and a chain of character effects on the
// wet path:
//
//   in -> input gate -> delay (with time modulation, reverse, cross-mod,
//   glitch, ring mod, sample-rate reduction, bitcrush, feedback/howl,
//   freeze) -> drive -> colour LPF -> phase flip / ping-pong -> dry/wet mix
//   -> out.
//
// Every switch has a corresponding LED, and the central display shows the
// entire delay-line contents as a mirrored bar timeline, the current read
// tap position, the tap triangle and a "now" marker, along with state chips
// and numeric readouts.
//
// File structure:
//   1. Layout constants (36 HP = 540x380 px).
//   2. pdSoftLimit() feedback-path limiter.
//   3. PsyDelay module: buffer management, gate, delay/modulation, glitch,
//      ring mod, SRR, bitcrusher, feedback, freeze, drive, colour, phase/
//      ping-pong and dry/wet mix.
//   4. Anonymous-namespace UI: PDPanelBackground and PDDisplayWidget.
//   5. PsyDelayWidget: places all controls, jacks, switches and the display.
//   6. Model registration.
// ============================================================

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int PD_BINS = 130;                       // display overview (2 px per bin)
static const float PD_PANEL_W = RACK_GRID_WIDTH * 36; // 540
static const float PD_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (centre column): header, timeline, ruler, chips, footer
static const float PD_DISP_X = 124.f;
static const float PD_DISP_Y = 34.f;
static const float PD_DISP_W = 292.f;
static const float PD_DISP_H = 152.f;

// Side blocks: 2 columns x 3 rows of knob cells
static const float PD_LCOL[2] = {36.f, 88.f};
static const float PD_RCOL[2] = {452.f, 504.f};
static const float PD_KCELL_W = 48.f;
static const float PD_KCELL_H = 62.f;
static const float PD_KTOP[3] = {58.f, 126.f, 194.f};
static const float PD_CAP1_Y = 46.f; // DELAY / FX captions
// Returns the knob centre y for knob row `row` (0..2).
static float pdKnobY(int row) { return PD_KTOP[row] + 38.f; }

// Switch row under the display
static const float PD_SW_TOP = 200.f;
static const float PD_SW_H = 52.f;
static const float PD_SW_W = 45.f;
// Returns the switch centre y for the row of on/off switches.
static float pdSwY() { return PD_SW_TOP + 34.f; }

// Jack row
static const float PD_CAP2_Y = 274.f; // INPUT / CV INPUT / OUTPUT captions
static const float PD_JACK_TOP = 283.f;
static const float PD_JACK_H = 52.f;
// Returns the jack centre y for the input/CV/output row.
static float pdJackY() { return PD_JACK_TOP + 33.f; }
// Returns the horizontal centre of the six middle columns (CV/switch row).
static float pdCenterX(int i) { return 148.f + 48.8f * i; } // 6 centre columns

// ------------------------------------------------------------
// Soft limiter for the feedback path (keeps self-oscillation from blowing up)
// ------------------------------------------------------------
// Soft-limits a feedback sample: linear up to ±8 V, then a tanh shoulder
// that asymptotes to ±12 V. Non-finite values are replaced by 0.
static inline float pdSoftLimit(float x)
{
    if (!std::isfinite(x))
        return 0.f;
    float a = std::fabs(x);
    if (a <= 8.f)
        return x;
    float y = 8.f + 4.f * std::tanh((a - 8.f) / 4.f);
    return x < 0.f ? -y : y;
}

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct PsyDelay : Module
{
    enum ParamId
    {
        TIME_PARAM,
        FEEDBACK_PARAM,
        MIX_PARAM,
        COLOR_PARAM,
        DRIVE_PARAM,
        MOD_DEPTH_PARAM,
        REVERSE_SWITCH,
        CHAOS_SWITCH,
        PINGPONG_SWITCH,
        BITCRUSH_SWITCH,
        FREEZE_SWITCH,
        PHASE_FLIP_SWITCH,
        HOWL_PARAM,
        RING_MOD_PARAM,
        GLITCH_PARAM,
        GATE_THRESH_PARAM,
        SRR_PARAM,
        CROSS_MOD_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        LEFT_INPUT,
        RIGHT_INPUT,
        TIME_CV_INPUT,
        FREEZE_CV_INPUT,
        HOWL_CV_INPUT,
        RING_CV_INPUT,
        GLITCH_CV_INPUT,
        GATE_CV_INPUT,
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
        FREEZE_LIGHT,
        BITCRUSH_LIGHT,
        PINGPONG_LIGHT,
        REVERSE_LIGHT,
        CHAOS_LIGHT,
        PHASE_FLIP_LIGHT,
        LIGHTS_LEN
    };

    // Delay buffers
    std::vector<float> delayL, delayR;
    int writePos = 0;
    float sampleRate = 44100.0f;
    size_t bufferSize = 0;

    // Output filter state (for COLOR)
    float outFilterL = 0.0f, outFilterR = 0.0f;

    // LFO / Chaos
    float lfoPhase = 0.0f;
    float chaosState = 0.5f;

    // Ring modulator phase
    float ringPhase = 0.0f;

    // Glitch RNG
    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist{0.0f, 1.0f};

    // Sample rate reduction state
    float srrCounter = 0.0f;
    float lastWetL = 0.0f, lastWetR = 0.0f;

    // Glitch smoothing
    float glitchFade = 0.0f;
    float glitchOldL = 0.0f, glitchOldR = 0.0f;

    // Input gate
    float gateEnv = 0.f;
    float gateGain = 1.f;

    // Flag to initialise output filter
    bool filterInitialised = false;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float peakBins[PD_BINS] = {};
    int peakWriteBin = 0;
    int lastBin = -1;
    float dispReadAge = 0.2f; // 0 = "now", 1 = oldest sample
    float dispMod = 0.f;
    float glitchFlash = 0.f;
    float meterL = 0.f, meterR = 0.f;
    bool dispGateActive = false, dispGateOpen = true;

    // Constructor: configures all parameters (with display scaling),
    // switches, CV inputs, outputs, bypass, LEDs and initial buffer state.
    PsyDelay()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(TIME_PARAM, 0.f, 1.f, 0.5f, "Delay time", " s", 0.f, 1.99f, 0.01f);
        configParam(FEEDBACK_PARAM, 0.f, 1.2f, 0.8f, "Feedback", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/wet", "%", 0.f, 100.f);
        configParam(COLOR_PARAM, 0.01f, 0.5f, 0.2f, "Tone (low = darker)", "%", 0.f, 200.f);
        configParam(DRIVE_PARAM, 0.f, 3.f, 1.0f, "Drive (saturation)", "x", 0.f, 1.f, 1.f);
        configParam(MOD_DEPTH_PARAM, 0.f, 1.f, 0.2f, "Mod depth", "%", 0.f, 100.f);
        configSwitch(REVERSE_SWITCH, 0.f, 1.f, 0.f, "Reverse", {"Off", "On"});
        configSwitch(CHAOS_SWITCH, 0.f, 1.f, 0.f, "Chaos LFO", {"Off", "On"});
        configSwitch(PINGPONG_SWITCH, 0.f, 1.f, 0.f, "Ping-pong", {"Off", "On"});
        configSwitch(BITCRUSH_SWITCH, 0.f, 1.f, 0.f, "Bitcrusher", {"Off", "On"});
        configSwitch(FREEZE_SWITCH, 0.f, 1.f, 0.f, "Freeze", {"Off", "On"});
        configSwitch(PHASE_FLIP_SWITCH, 0.f, 1.f, 0.f, "Phase flip", {"Off", "On"});
        configParam(HOWL_PARAM, 0.f, 1.f, 0.f, "Howl", "%", 0.f, 100.f);
        configParam(RING_MOD_PARAM, 0.f, 1.f, 0.f, "Ring mod", "%", 0.f, 100.f);
        configParam(GLITCH_PARAM, 0.f, 1.f, 0.f, "Glitch", "%", 0.f, 100.f);
        configParam(GATE_THRESH_PARAM, 0.f, 1.f, 0.f, "Input gate threshold", " V", 0.f, 4.f);
        configParam(SRR_PARAM, 0.f, 1.f, 0.f, "Sample hold", "%", 0.f, 100.f);
        configParam(CROSS_MOD_PARAM, 0.f, 1.f, 0.f, "Cross-mod", "%", 0.f, 100.f);

        configInput(LEFT_INPUT, "Left (mono if the right input is empty)");
        configInput(RIGHT_INPUT, "Right");
        configInput(TIME_CV_INPUT, "Time CV (0 to 10 V)");
        configInput(FREEZE_CV_INPUT, "Freeze gate");
        configInput(HOWL_CV_INPUT, "Howl CV (0 to 10 V)");
        configInput(RING_CV_INPUT, "Ring mod CV (0 to 10 V)");
        configInput(GLITCH_CV_INPUT, "Glitch CV (0 to 10 V)");
        configInput(GATE_CV_INPUT, "Gate threshold CV (0 to 10 V)");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");
        configBypass(LEFT_INPUT, OUT_L_OUTPUT);
        configBypass(RIGHT_INPUT, OUT_R_OUTPUT);

        configLight(FREEZE_LIGHT, "Freeze");
        configLight(BITCRUSH_LIGHT, "Bitcrush");
        configLight(PINGPONG_LIGHT, "Ping-pong");
        configLight(REVERSE_LIGHT, "Reverse");
        configLight(CHAOS_LIGHT, "Chaos");
        configLight(PHASE_FLIP_LIGHT, "Phase flip");

        setSampleRate(44100.0f);
    }

    // Allocates the L/R delay buffers for the given sample rate (2 seconds
    // plus one sample) and resets all filter/LFO/gate/display state.
    void setSampleRate(float sr)
    {
        sampleRate = sr;
        bufferSize = (size_t)(sr * 2.0f) + 1;
        delayL.assign(bufferSize, 0.0f);
        delayR.assign(bufferSize, 0.0f);
        writePos = 0;
        outFilterL = outFilterR = 0.0f;
        lfoPhase = 0.0f;
        chaosState = 0.5f;
        ringPhase = 0.0f;
        srrCounter = 0.0f;
        lastWetL = lastWetR = 0.0f;
        glitchFade = 0.0f;
        glitchOldL = glitchOldR = 0.0f;
        filterInitialised = false;
        for (int i = 0; i < PD_BINS; ++i)
            peakBins[i] = 0.f;
        lastBin = -1;
    }

    // Rack sample-rate-change hook: reallocates the delay buffers to match
    // the engine's new sample rate.
    void onSampleRateChange() override
    {
        setSampleRate(APP->engine->getSampleRate());
    }

    // Main audio callback: applies the input gate, computes the modulated
    // delay time (with chaos LFO), reads the (optionally reversed and
    // cross-modulated) taps, applies glitch/ring mod/SRR/bitcrush to the wet
    // signal, computes the feedback (with howl), handles freeze, writes into
    // the delay line, runs drive + tone on the wet output, applies phase
    // flip/ping-pong and mixes dry/wet, then writes the stereo outputs and
    // updates the meters and display snapshot.
    void process(const ProcessArgs &args) override
    {
        float sr = args.sampleRate;
        if (sr != sampleRate)
            setSampleRate(sr);

        float inL = inputs[LEFT_INPUT].getVoltage();
        float inR = inputs[RIGHT_INPUT].isConnected() ? inputs[RIGHT_INPUT].getVoltage() : inL;

        // ===== 0. Input gate (threshold 0 = always open) =====
        float gateT = clamp(params[GATE_THRESH_PARAM].getValue() + inputs[GATE_CV_INPUT].getVoltage() / 10.0f, 0.0f, 1.0f) * 4.0f;
        float lvl = std::max(std::fabs(inL), std::fabs(inR));
        gateEnv = std::max(lvl, gateEnv * (1.0f - 10.0f * args.sampleTime));
        bool gateOpen = gateEnv >= gateT;
        gateGain += ((gateOpen ? 1.0f : 0.0f) - gateGain) * std::min(1.0f, 200.0f * args.sampleTime);
        dispGateActive = gateT > 0.001f;
        dispGateOpen = gateOpen;

        // ===== 1. Delay time & modulation =====
        float timeKnob = params[TIME_PARAM].getValue();
        float timeCv = inputs[TIME_CV_INPUT].getVoltage() / 10.0f;
        float time01 = clamp(timeKnob + timeCv, 0.0f, 1.0f);
        float maxDelaySec = (bufferSize - 1) / sr;
        float delaySec = 0.01f + time01 * (maxDelaySec - 0.01f);

        bool chaosMode = params[CHAOS_SWITCH].getValue() > 0.5f;
        lights[CHAOS_LIGHT].setBrightness(chaosMode ? 1.0f : 0.0f);
        float modDepth = params[MOD_DEPTH_PARAM].getValue();
        float modSignal;
        if (chaosMode)
        {
            chaosState = 3.9f * chaosState * (1.0f - chaosState);
            modSignal = (chaosState - 0.5f) * 2.0f;
        }
        else
        {
            lfoPhase += 0.5f * args.sampleTime;
            if (lfoPhase > 1.0f)
                lfoPhase -= 1.0f;
            modSignal = sinf(2.0f * M_PI * lfoPhase);
        }
        dispMod = modSignal * modDepth;
        float delayMod = modSignal * modDepth * 0.05f;
        float delaySamples = delaySec * sr * (1.0f + delayMod);
        delaySamples = clamp(delaySamples, 1.0f, (float)(bufferSize - 1));

        // ===== 2. Read position (with Reverse) =====
        bool reverse = params[REVERSE_SWITCH].getValue() > 0.5f;
        lights[REVERSE_LIGHT].setBrightness(reverse ? 1.0f : 0.0f);
        float readPos;
        if (reverse)
        {
            readPos = (float)writePos + delaySamples;
            while (readPos >= bufferSize)
                readPos -= bufferSize;
        }
        else
        {
            readPos = (float)writePos - delaySamples;
            while (readPos < 0)
                readPos += bufferSize;
        }
        {
            float age = (float)writePos - readPos;
            if (age < 0.f)
                age += (float)bufferSize;
            dispReadAge = age / (float)bufferSize;
        }

        // ===== 3. Cross-modulation (right channel offset) =====
        float crossMod = params[CROSS_MOD_PARAM].getValue();
        float crossOffset = crossMod * (delayL[writePos] * 0.002f * sr);
        float readPosR = readPos + crossOffset;
        while (readPosR >= bufferSize)
            readPosR -= bufferSize;
        while (readPosR < 0)
            readPosR += bufferSize;

        // Linear interpolation (left)
        int idxL1 = (int)readPos;
        int idxL2 = (idxL1 + 1) % bufferSize;
        float frac = readPos - idxL1;
        float wetRawL = (1.0f - frac) * delayL[idxL1] + frac * delayL[idxL2];

        // Linear interpolation (right)
        int idxR1 = (int)readPosR;
        int idxR2 = (idxR1 + 1) % bufferSize;
        float fracR = readPosR - idxR1;
        float wetRawR = (1.0f - fracR) * delayR[idxR1] + fracR * delayR[idxR2];

        // ===== 4. Glitch with smoothing =====
        float glitchAmount = params[GLITCH_PARAM].getValue();
        float glitchCv = inputs[GLITCH_CV_INPUT].getVoltage() / 10.0f;
        glitchAmount = clamp(glitchAmount + glitchCv, 0.0f, 1.0f);
        if (glitchAmount > 0.0f && dist(rng) < glitchAmount * 0.01f)
        {
            int jump = (int)(dist(rng) * bufferSize * glitchAmount);
            readPos = fmodf(readPos + jump, (float)bufferSize);
            if (readPos < 0)
                readPos += bufferSize;
            readPosR = fmodf(readPosR + jump, (float)bufferSize);
            if (readPosR < 0)
                readPosR += bufferSize;
            glitchOldL = wetRawL;
            glitchOldR = wetRawR;
            glitchFade = 1.0f;
            glitchFlash = 1.0f;
            idxL1 = (int)readPos;
            idxL2 = (idxL1 + 1) % bufferSize;
            frac = readPos - idxL1;
            wetRawL = (1.0f - frac) * delayL[idxL1] + frac * delayL[idxL2];
            idxR1 = (int)readPosR;
            idxR2 = (idxR1 + 1) % bufferSize;
            fracR = readPosR - idxR1;
            wetRawR = (1.0f - fracR) * delayR[idxR1] + fracR * delayR[idxR2];
        }
        // Crossfade over ~2 ms to avoid clicks
        if (glitchFade > 0.0f)
        {
            float fade = glitchFade;
            wetRawL = wetRawL * (1.0f - fade) + glitchOldL * fade;
            wetRawR = wetRawR * (1.0f - fade) + glitchOldR * fade;
            glitchFade -= args.sampleTime / 0.002f;
            if (glitchFade < 0.0f)
                glitchFade = 0.0f;
        }
        glitchFlash = std::max(0.f, glitchFlash - 4.f * args.sampleTime);

        // ===== 5. Ring modulation (softer carrier) =====
        float ringAmount = params[RING_MOD_PARAM].getValue();
        float ringCv = inputs[RING_CV_INPUT].getVoltage() / 10.0f;
        ringAmount = clamp(ringAmount + ringCv, 0.0f, 1.0f);
        if (ringAmount > 0.0f)
        {
            ringPhase += 2.0f * M_PI * 50.0f * args.sampleTime;
            if (ringPhase > 2.0f * M_PI)
                ringPhase -= 2.0f * M_PI;
            float carrier = sinf(ringPhase);
            wetRawL = wetRawL * (1.0f - ringAmount * 0.5f) + wetRawL * carrier * ringAmount;
            wetRawR = wetRawR * (1.0f - ringAmount * 0.5f) + wetRawR * carrier * ringAmount;
        }

        // ===== 6. Sample rate reduction (disabled by default) =====
        float srr = params[SRR_PARAM].getValue();
        if (srr > 0.0f)
        {
            float step = 1.0f / (1.0f + srr * 99.0f);
            srrCounter += args.sampleTime;
            if (srrCounter >= step)
            {
                srrCounter = 0.0f;
                lastWetL = wetRawL;
                lastWetR = wetRawR;
            }
            else
            {
                wetRawL = lastWetL;
                wetRawR = lastWetR;
            }
        }

        // ===== 7. Bitcrusher =====
        bool bitcrush = params[BITCRUSH_SWITCH].getValue() > 0.5f;
        lights[BITCRUSH_LIGHT].setBrightness(bitcrush ? 1.0f : 0.0f);
        if (bitcrush)
        {
            float bits = 6.0f;
            wetRawL = roundf(wetRawL * bits) / bits;
            wetRawR = roundf(wetRawR * bits) / bits;
        }

        // ===== 8. Feedback (using raw delay output) =====
        float feedback = params[FEEDBACK_PARAM].getValue();
        float howl = params[HOWL_PARAM].getValue();
        float howlCv = inputs[HOWL_CV_INPUT].getVoltage() / 10.0f;
        howl = clamp(howl + howlCv, 0.0f, 1.0f);
        float feedbackAmount = feedback + howl * 0.2f;
        float howlSignal = tanhf(wetRawL * howl * 2.0f);
        feedbackAmount += howlSignal * 0.1f;

        // ===== 9. Freeze =====
        bool freeze = (params[FREEZE_SWITCH].getValue() > 0.5f) || (inputs[FREEZE_CV_INPUT].getVoltage() > 1.0f);
        lights[FREEZE_LIGHT].setBrightness(freeze ? 1.0f : 0.0f);
        if (!freeze)
        {
            delayL[writePos] = pdSoftLimit(inL * gateGain + wetRawL * feedbackAmount);
            delayR[writePos] = pdSoftLimit(inR * gateGain + wetRawR * feedbackAmount);
        }
        else
        {
            delayL[writePos] = delayL[writePos] * 0.999f;
            delayR[writePos] = delayR[writePos] * 0.999f;
        }

        // Display overview of what is stored in the delay line
        {
            int bin = (int)((long long)writePos * PD_BINS / (long long)bufferSize);
            if (bin >= PD_BINS)
                bin = PD_BINS - 1;
            if (bin != lastBin)
            {
                peakBins[bin] = 0.f;
                lastBin = bin;
            }
            float wv = 0.5f * (std::fabs(delayL[writePos]) + std::fabs(delayR[writePos]));
            if (wv > peakBins[bin])
                peakBins[bin] = wv;
            peakWriteBin = bin;
        }

        writePos = (writePos + 1) % bufferSize;

        // ===== 10. Output processing: Drive (saturation) then Color (LPF) =====
        float drive = params[DRIVE_PARAM].getValue() + 1.0f;
        float color = params[COLOR_PARAM].getValue();
        float satL = tanhf(wetRawL * drive);
        float satR = tanhf(wetRawR * drive);

        if (!filterInitialised && (fabs(satL) > 0.001f || fabs(satR) > 0.001f))
        {
            outFilterL = satL;
            outFilterR = satR;
            filterInitialised = true;
        }
        else
        {
            outFilterL += color * (satL - outFilterL);
            outFilterR += color * (satR - outFilterR);
        }
        float processedL = outFilterL;
        float processedR = outFilterR;

        // ===== 11. Phase Flip & Ping-Pong =====
        bool phaseFlip = params[PHASE_FLIP_SWITCH].getValue() > 0.5f;
        lights[PHASE_FLIP_LIGHT].setBrightness(phaseFlip ? 1.0f : 0.0f);
        bool pingpong = params[PINGPONG_SWITCH].getValue() > 0.5f;
        lights[PINGPONG_LIGHT].setBrightness(pingpong ? 1.0f : 0.0f);

        float outL = processedL;
        float outR = processedR;
        if (pingpong)
            outR = -processedL;
        if (phaseFlip)
            outR = -outR;

        // ===== 12. Dry/Wet mix with 1% dry bleed at full wet =====
        float mix = params[MIX_PARAM].getValue();
        float dryAmount = 1.0f - mix;
        if (mix >= 0.999f)
            dryAmount = 0.01f;
        float finalL = inL * dryAmount + outL * mix;
        float finalR = inR * dryAmount + outR * mix;

        outputs[OUT_L_OUTPUT].setVoltage(finalL);
        outputs[OUT_R_OUTPUT].setVoltage(finalR);

        // Meters
        float md = 1.0f - 5.0f * args.sampleTime;
        meterL = std::max(std::fabs(finalL), meterL * md);
        meterR = std::max(std::fabs(finalR), meterR * md);
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{
    namespace ag = agmm;

    // ------------------------------------------------------------
    // Panel background: title, display frame, captions, labelled plates
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the left DELAY and right FX
    // knob blocks, the six-switch row, and the input/CV/output jack row with
    // its captions.
    struct PDPanelBackground : TransparentWidget
    {
        PDPanelBackground() { box.size = Vec(PD_PANEL_W, PD_PANEL_H); }

        // Section caption with hairlines, placed inside the block x0 .. x0 + w
        // Draws an agmm section header whose hairlines span only [x0, x0+w],
        // by translating the context.
        static void caption(NVGcontext *vg, float x0, float w, float y, const char *txt)
        {
            nvgSave(vg);
            nvgTranslate(vg, x0, 0.f);
            ag::sectionHeader(vg, w, y, txt, 0.f);
            nvgRestore(vg);
        }

        // Draws all static panel art: base panel, bezel, the two knob blocks,
        // the switch row plates and the input/CV/output jack row plates
        // (outputs tinted).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, PD_PANEL_W, PD_PANEL_H, "PSYDELAY");
            ag::bezel(vg, PD_DISP_X, PD_DISP_Y, PD_DISP_W, PD_DISP_H);

            // ---- knob blocks ----
            caption(vg, 12.f, 100.f, PD_CAP1_Y, "DELAY");
            caption(vg, 428.f, 100.f, PD_CAP1_Y, "FX");

            const char *leftNames[6] = {"TIME", "FEEDBACK", "MIX", "TONE", "DRIVE", "MOD"};
            const char *rightNames[6] = {"HOWL", "RING", "GLITCH", "GATE", "SRR", "CROSS"};
            for (int i = 0; i < 6; ++i)
            {
                ag::cell(vg, PD_LCOL[i % 2], PD_KTOP[i / 2], PD_KCELL_W, PD_KCELL_H, leftNames[i]);
                ag::cell(vg, PD_RCOL[i % 2], PD_KTOP[i / 2], PD_KCELL_W, PD_KCELL_H, rightNames[i]);
            }

            // ---- switch row ----
            const char *sw[6] = {"REVERSE", "CHAOS", "PINGPONG", "CRUSH", "PHASE", "FREEZE"};
            for (int i = 0; i < 6; ++i)
                ag::cell(vg, pdCenterX(i), PD_SW_TOP, PD_SW_W, PD_SW_H, sw[i]);

            // ---- jack row ----
            caption(vg, 12.f, 100.f, PD_CAP2_Y, "INPUT");
            caption(vg, PD_DISP_X, PD_DISP_W, PD_CAP2_Y, "CV INPUT");
            caption(vg, 428.f, 100.f, PD_CAP2_Y, "OUTPUT");

            ag::cell(vg, PD_LCOL[0], PD_JACK_TOP, PD_KCELL_W, PD_JACK_H, "L IN");
            ag::cell(vg, PD_LCOL[1], PD_JACK_TOP, PD_KCELL_W, PD_JACK_H, "R IN");
            const char *cv[6] = {"TIME CV", "HOWL CV", "RING CV", "GLT CV", "GATE CV", "FRZ GATE"};
            for (int i = 0; i < 6; ++i)
                ag::cell(vg, pdCenterX(i), PD_JACK_TOP, PD_SW_W, PD_JACK_H, cv[i]);
            ag::cell(vg, PD_RCOL[0], PD_JACK_TOP, PD_KCELL_W, PD_JACK_H, "OUT L", true);
            ag::cell(vg, PD_RCOL[1], PD_JACK_TOP, PD_KCELL_W, PD_JACK_H, "OUT R", true);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: the delay line as a timeline, with the read tap
    // Rows (display-local): header 0-15, timeline 22-102, ruler labels 113,
    // chips 121-134, footer text 146.
    // ------------------------------------------------------------
    // Live display: shows the entire delay-line contents as a mirrored bar
    // timeline, the current read tap, the tap triangle, a "now" marker, L/R
    // meters, state chips and numeric readouts. Uses demo data when there is
    // no module.
    struct PDDisplayWidget : TransparentWidget
    {
        PsyDelay *module = nullptr;

        static constexpr float X0 = 10.f, Y0 = 22.f, TW = 260.f, TH = 80.f;

        PDDisplayWidget(PsyDelay *m) : module(m)
        {
            box.pos = Vec(PD_DISP_X, PD_DISP_Y);
            box.size = Vec(PD_DISP_W, PD_DISP_H);
        }

        // Draws a horizontal divider line inside the display at y, spanning
        // the display width minus an 8 px margin.
        static void hairline(NVGcontext *vg, float y, float W)
        {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, y);
            nvgLineTo(vg, W - 8.f, y);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Static background: screen, timeline field, grid, ruler
        // Draws the display screen, the header/footer dividers, the timeline
        // frame, the 0.5 s grid and centre line, the time ruler with its
        // labels, and the L/R meter channel labels.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = PD_DISP_W, H = PD_DISP_H;
            const float mid = Y0 + TH / 2.f;

            ag::screen(vg, 0.f, 0.f, W, H);
            hairline(vg, 15.5f, W);
            hairline(vg, 139.5f, W);

            ag::plotFrame(vg, X0, Y0, TW, TH);

            // Grid every 0.5 s (the buffer holds 2 s, "now" is on the right)
            nvgBeginPath(vg);
            for (int k = 1; k < 4; ++k)
            {
                float x = X0 + TW - k * (TW / 4.f) + 0.5f;
                nvgMoveTo(vg, x, Y0 + 1.f);
                nvgLineTo(vg, x, Y0 + TH - 1.f);
            }
            nvgMoveTo(vg, X0 + 1.f, mid + 0.5f);
            nvgLineTo(vg, X0 + TW - 1.f, mid + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 18));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Ruler: ticks, then labels in their own row
            nvgBeginPath(vg);
            for (int k = 0; k <= 4; ++k)
            {
                float x = X0 + TW - k * (TW / 4.f) + 0.5f;
                nvgMoveTo(vg, x, Y0 + TH + 1.f);
                nvgLineTo(vg, x, Y0 + TH + 4.f);
            }
            nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 80));
            nvgStroke(vg);
            const char *tl[5] = {"0", "0.5", "1.0", "1.5", "2.0S"};
            for (int k = 0; k <= 4; ++k)
                ag::text(vg, X0 + TW - k * (TW / 4.f), Y0 + TH + 11.f, tl[k], 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            // Meter channel labels
            ag::text(vg, X0 + TW + 8.f, Y0 + TH + 11.f, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, X0 + TW + 14.f, Y0 + TH + 11.f, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the delay-line contents as mirrored bars,
        // the tap region, the tap line and triangle, the modulation wobble,
        // the "now" marker, glitch ring, L/R meters, state chips, and the
        // header/footer readouts.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = PD_DISP_W;
            const float mid = Y0 + TH / 2.f;

            // ---- gather state (live, or a sample scene in the module browser) ----
            float ord[PD_BINS];
            float readAge, mod, glf, mL, mR;
            float timeMs, fb, mix, drive, glitch, ring, srr;
            bool frz, rev, chs, png, crush, phs, gAct, gOpen;

            if (module)
            {
                int wb = module->peakWriteBin;
                for (int i = 0; i < PD_BINS; ++i)
                    ord[i] = module->peakBins[(wb + 1 + i) % PD_BINS];
                readAge = module->dispReadAge;
                mod = module->dispMod;
                glf = module->glitchFlash;
                mL = module->meterL;
                mR = module->meterR;
                float tk = module->params[PsyDelay::TIME_PARAM].getValue();
                timeMs = (0.01f + tk * 1.99f) * 1000.f;
                fb = module->params[PsyDelay::FEEDBACK_PARAM].getValue();
                mix = module->params[PsyDelay::MIX_PARAM].getValue();
                drive = module->params[PsyDelay::DRIVE_PARAM].getValue() + 1.f;
                glitch = module->params[PsyDelay::GLITCH_PARAM].getValue();
                ring = module->params[PsyDelay::RING_MOD_PARAM].getValue();
                srr = module->params[PsyDelay::SRR_PARAM].getValue();
                frz = module->params[PsyDelay::FREEZE_SWITCH].getValue() > 0.5f;
                rev = module->params[PsyDelay::REVERSE_SWITCH].getValue() > 0.5f;
                chs = module->params[PsyDelay::CHAOS_SWITCH].getValue() > 0.5f;
                png = module->params[PsyDelay::PINGPONG_SWITCH].getValue() > 0.5f;
                crush = module->params[PsyDelay::BITCRUSH_SWITCH].getValue() > 0.5f;
                phs = module->params[PsyDelay::PHASE_FLIP_SWITCH].getValue() > 0.5f;
                gAct = module->dispGateActive;
                gOpen = module->dispGateOpen;
            }
            else
            {
                for (int i = 0; i < PD_BINS; ++i)
                {
                    float age = (float)(PD_BINS - 1 - i);
                    ord[i] = 4.f * std::exp(-age * 0.022f) * (0.35f + 0.65f * std::fabs(std::sin(age * 0.33f)));
                }
                readAge = 0.25f;
                mod = 0.3f;
                glf = 0.f;
                mL = 3.f;
                mR = 2.4f;
                timeMs = 500.f;
                fb = 0.8f;
                mix = 0.5f;
                drive = 2.f;
                glitch = 0.f;
                ring = 0.f;
                srr = 0.f;
                frz = false;
                rev = false;
                chs = false;
                png = true;
                crush = false;
                phs = false;
                gAct = false;
                gOpen = true;
            }

            // ---- delay line contents (mirrored bars, newest on the right) ----
            nvgSave(vg);
            nvgScissor(vg, X0, Y0, TW, TH);
            const float amp = TH / 2.f - 2.f;
            nvgBeginPath(vg);
            for (int i = 0; i < PD_BINS; ++i)
            {
                float h = std::min(ord[i] / 5.f, 1.f) * amp;
                if (h < 0.4f)
                    continue;
                nvgRect(vg, X0 + i * 2.f, mid - h, 1.6f, 2.f * h);
            }
            nvgFillColor(vg, frz ? nvgRGB(0xd8, 0xb0, 0xff) : ag::bright());
            nvgFill(vg);

            // ---- tap region, tap marker, now marker ----
            float tx = X0 + (1.f - std::min(std::max(readAge, 0.f), 1.f)) * TW;
            nvgBeginPath(vg);
            nvgRect(vg, tx, Y0, X0 + TW - tx, TH);
            nvgFillColor(vg, ag::withA(ag::violet(), 0.12f));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, tx + 0.5f, Y0);
            nvgLineTo(vg, tx + 0.5f, Y0 + TH);
            nvgStrokeColor(vg, nvgRGB(0xd8, 0xb0, 0xff));
            nvgStrokeWidth(vg, 1.3f);
            nvgStroke(vg);

            if (glf > 0.02f)
            {
                nvgBeginPath(vg);
                nvgCircle(vg, tx + 0.5f, mid, 3.f + (1.f - glf) * 14.f);
                nvgStrokeColor(vg, ag::withA(ag::violet(), 0.9f * glf));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }

            // Modulation wobble under the tap
            {
                float mx = tx + 0.5f + std::min(std::max(mod, -1.f), 1.f) * 8.f;
                nvgBeginPath(vg);
                nvgCircle(vg, mx, Y0 + TH - 5.f, 2.f);
                nvgFillColor(vg, ag::violet());
                nvgFill(vg);
            }

            // Now marker
            nvgBeginPath(vg);
            nvgRect(vg, X0 + TW - 1.5f, Y0, 1.5f, TH);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 200));
            nvgFill(vg);
            nvgRestore(vg);

            // Tap triangle sits just above the plot, between header and timeline
            nvgBeginPath(vg);
            nvgMoveTo(vg, tx - 3.5f, Y0 - 4.f);
            nvgLineTo(vg, tx + 4.5f, Y0 - 4.f);
            nvgLineTo(vg, tx + 0.5f, Y0 + 1.f);
            nvgClosePath(vg);
            nvgFillColor(vg, ag::violet());
            nvgFill(vg);

            // ---- output meters (right of the timeline) ----
            ag::meter(vg, X0 + TW + 6.f, Y0, 4.f, TH, std::min(mL / 5.f, 1.f));
            ag::meter(vg, X0 + TW + 12.f, Y0, 4.f, TH, std::min(mR / 5.f, 1.f));

            // ---- state chips (own row under the ruler) ----
            const float cy = 121.f, ch = 13.f, cw = 34.f, pitch = 39.7f;
            ag::chip(vg, 10.f + pitch * 0, cy, cw, ch, "FRZ", frz, ag::violet());
            ag::chip(vg, 10.f + pitch * 1, cy, cw, ch, "REV", rev, ag::accent());
            ag::chip(vg, 10.f + pitch * 2, cy, cw, ch, "CHS", chs, ag::accent());
            ag::chip(vg, 10.f + pitch * 3, cy, cw, ch, "PNG", png, ag::accent());
            ag::chip(vg, 10.f + pitch * 4, cy, cw, ch, "CRSH", crush, ag::accent());
            ag::chip(vg, 10.f + pitch * 5, cy, cw, ch, "PHS", phs, ag::accent());
            ag::chip(vg, 10.f + pitch * 6, cy, cw, ch, "GATE", gAct && gOpen, ag::violet());

            // ---- header readouts ----
            char buf[64];
            snprintf(buf, sizeof(buf), "TIME %.0f MS", timeMs);
            ag::text(vg, 10.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "FB %.0f%%", fb * 100.f);
            ag::text(vg, W / 2.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_C, 0.5f);
            snprintf(buf, sizeof(buf), "MIX %.0f%%", mix * 100.f);
            ag::text(vg, W - 10.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);

            // ---- footer readouts ----
            snprintf(buf, sizeof(buf), "DRV %.1fX", drive);
            ag::text(vg, 10.f, 146.f, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "GLT %.0f%%  RNG %.0f%%", glitch * 100.f, ring * 100.f);
            ag::text(vg, W / 2.f, 146.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            snprintf(buf, sizeof(buf), "SRR %.0f%%", srr * 100.f);
            ag::text(vg, W - 10.f, 146.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Widget: adds the static panel, screws, the live display, the twelve knobs
// (six delay + six FX), the six switches with their LEDs, and the
// input/CV/output jacks.
// ------------------------------------------------------------
struct PsyDelayWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, display, the six
    // left delay knobs and six right FX knobs, the six switches each with a
    // status LED (violet for freeze, cyan otherwise), and the L/R inputs,
    // six CV/gate inputs, and L/R outputs.
    PsyDelayWidget(PsyDelay *module)
    {
        setModule(module);
        box.size = Vec(PD_PANEL_W, PD_PANEL_H);

        addChild(new PDPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PDDisplayWidget(module));

        // ---- left / right knobs ----
        const int leftIds[6] = {PsyDelay::TIME_PARAM, PsyDelay::FEEDBACK_PARAM, PsyDelay::MIX_PARAM,
                                PsyDelay::COLOR_PARAM, PsyDelay::DRIVE_PARAM, PsyDelay::MOD_DEPTH_PARAM};
        const int rightIds[6] = {PsyDelay::HOWL_PARAM, PsyDelay::RING_MOD_PARAM, PsyDelay::GLITCH_PARAM,
                                 PsyDelay::GATE_THRESH_PARAM, PsyDelay::SRR_PARAM, PsyDelay::CROSS_MOD_PARAM};
        for (int i = 0; i < 6; ++i)
        {
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PD_LCOL[i % 2], pdKnobY(i / 2)), module, leftIds[i]));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PD_RCOL[i % 2], pdKnobY(i / 2)), module, rightIds[i]));
            //agmm::addKnobArcs(this, 6.f);
        }
        agmm::addKnobArcs(this, 6.f);

        // ---- switches with LEDs ----
        const int swIds[6] = {PsyDelay::REVERSE_SWITCH, PsyDelay::CHAOS_SWITCH, PsyDelay::PINGPONG_SWITCH,
                              PsyDelay::BITCRUSH_SWITCH, PsyDelay::PHASE_FLIP_SWITCH, PsyDelay::FREEZE_SWITCH};
        const int ledIds[6] = {PsyDelay::REVERSE_LIGHT, PsyDelay::CHAOS_LIGHT, PsyDelay::PINGPONG_LIGHT,
                               PsyDelay::BITCRUSH_LIGHT, PsyDelay::PHASE_FLIP_LIGHT, PsyDelay::FREEZE_LIGHT};
        for (int i = 0; i < 6; ++i)
        {
            float cx = pdCenterX(i);
            addParam(createParamCentered<CKSS>(Vec(cx - 6.f, pdSwY()), module, swIds[i]));
            if (i == 5)
                addChild(createLightCentered<SmallLight<agmm::VioletLight>>(Vec(cx + 11.f, pdSwY()), module, ledIds[i]));
            else
                addChild(createLightCentered<SmallLight<agmm::CyanLight>>(Vec(cx + 11.f, pdSwY()), module, ledIds[i]));
        }

        // ---- jacks ----
        const float jy = pdJackY();
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PD_LCOL[0], jy), module, PsyDelay::LEFT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PD_LCOL[1], jy), module, PsyDelay::RIGHT_INPUT));
        const int cvIds[6] = {PsyDelay::TIME_CV_INPUT, PsyDelay::HOWL_CV_INPUT, PsyDelay::RING_CV_INPUT,
                              PsyDelay::GLITCH_CV_INPUT, PsyDelay::GATE_CV_INPUT, PsyDelay::FREEZE_CV_INPUT};
        for (int i = 0; i < 6; ++i)
            addInput(createInputCentered<DarkPJ301MPort>(Vec(pdCenterX(i), jy), module, cvIds[i]));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PD_RCOL[0], jy), module, PsyDelay::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PD_RCOL[1], jy), module, PsyDelay::OUT_R_OUTPUT));
        agmm::addKnobArcs(this, 6.f);
    }
};

Model *modelPsyDelay = createModel<PsyDelay, PsyDelayWidget>("PsyDelay");