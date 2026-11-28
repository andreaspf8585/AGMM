#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include "Knob_arcs.hpp"
#include <algorithm>
#include <random>
#include <chrono>

// ============================================================
// Dice On Steroids
// ============================================================
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// DiceOnSteroids is a 28 HP expanded version of the Dice module: a 16-voice
// probabilistic gate / random V/Oct generator with a large feature set. On
// each (optionally divided) clock edge it rolls per-channel dice, and voices
// that "win" open a gate and latch a new random voltage. The random voltage
// can be uniform, a random walk, a drunk walk, or Gaussian, and it can be
// quantized to any of 60+ scales. The module also supports:
//   - Glide (portamento) between successive pitches.
//   - A per-channel 64-step pattern recorder / playback loop (CH1 pattern is
//     displayed as a strip).
//   - A "train mode" where one random value shifts down the voices each
//     clock, like a delay line of pitches.
//   - CV control over probability, polyphony, gate time, range, scale,
//     transpose and glide.
//   - A seed input and menu to re-seed the RNG for repeatable randomness.
//
// File structure:
//   1. Layout constants (28 HP = 420x380 px).
//   2. Scale struct + module declaration (DiceOnSteroids) with all state,
//      config, RNG helpers, quantization, pattern recorder, JSON persistence
//      and process().
//   3. Static scale table definition.
//   4. Anonymous-namespace UI: dsBlockHeader helper, DSPanelBackground
//      (static art) and DSDisplayWidget (live plot/readouts/chips/meter).
//   5. DiceOnSteroidsWidget: places controls, LEDs and the context menu.
//   6. Model registration.
// -----------------------------------------------------------------------------

// ------------------------------------------------------------
// Layout (28 HP = 420 x 380)
// Every control sits in a plate with its label ABOVE it.
// ------------------------------------------------------------
static const float DS_PANEL_W = RACK_GRID_WIDTH * 28; // 420
static const float DS_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it)
static const float DS_DISP_X = 108.f;
static const float DS_DISP_Y = 34.f;
static const float DS_DISP_W = 212.f;
static const float DS_DISP_H = 166.f;
static const float DS_CH_X0 = 10.f; // first channel column inside the display
static const float DS_CH_W = 12.f;  // 16 x 12 = 192

// Left block: 2 columns x 4 rows of small knobs
static const float DS_BLOCK_HEAD_Y = 40.f;
static const float DS_LCOL[2] = {30.f, 72.f};
static const float DS_LTOP0 = 50.f, DS_LPITCH = 56.f;
static const float DS_LW = 38.f, DS_LH = 52.f;
// Top edge of left-block row r.
static float dsLTop(int r) { return DS_LTOP0 + DS_LPITCH * r; }
// Knob centre for left-block row r (plate top + 32 px).
static float dsLKnobY(int r) { return dsLTop(r) + 32.f; }

// Right block: outputs, then transport
static const float DS_RCOL[2] = {352.f, 392.f};
static const float DS_RW = 36.f;
static const float DS_ROUT_TOP[2] = {50.f, 100.f};
static const float DS_ROUT_H = 46.f;
static const float DS_TRANS_HEAD_Y = 160.f;
static const float DS_RSW_TOP[2] = {172.f, 230.f};
static const float DS_RSW_H = 54.f;

// LED strip under the display (numbers on top, LEDs below)
static const float DS_LED_TOP = 212.f, DS_LED_H = 34.f;
static const float DS_LED_NUM_Y = DS_LED_TOP + 10.f;
static const float DS_LED_Y = DS_LED_TOP + 24.f;

// Input row (10 jacks)
static const float DS_IN_HEAD_Y = 298.f;
static const float DS_IN_TOP = 308.f, DS_IN_H = 50.f, DS_IN_W = 38.f;
static const float DS_IN_Y = DS_IN_TOP + 31.f;
// Horizontal centre of input jack i (0..9).
static float dsInX(int i) { return 30.f + 40.f * i; }

// ------------------------------------------------------------
// Scale structure
// ------------------------------------------------------------
// One named quantization scale: a display name plus its semitone intervals
// from the root.
struct Scale
{
    std::string name;
    std::vector<float> intervals; // semitone intervals from root
};

// ------------------------------------------------------------
// Module definition
// ------------------------------------------------------------
struct DiceOnSteroids : Module
{
    enum ParamId
    {
        PARAM_POLYPHONY,
        PARAM_PROBABILITY,
        PARAM_GLIDE_TIME,
        PARAM_ROOT_SHIFT,
        PARAM_CLOCK_DIV,
        PARAM_PROB_SPREAD,
        PARAM_PROB_SMOOTH,
        PARAM_RECORD_ENABLE,
        PARAM_PLAYBACK_ENABLE,
        PARAM_LOOP_LENGTH,
        PARAM_TRAIN_MODE,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_CLOCK,
        INPUT_CV_PROB,
        INPUT_CV_POLY,
        INPUT_CV_GATE_TIME,
        INPUT_CV_RANGE,
        INPUT_CV_SCALE,
        INPUT_CV_TRANSPOSE,
        INPUT_CV_GLIDE,
        INPUT_RETRIGGER,
        INPUT_RESET,
        INPUT_RANDOM_SEED,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_GATES,
        OUTPUT_VOCT,
        OUTPUT_QUANTIZED,
        OUTPUT_TRIGGER_NEW,
        OUTPUTS_LEN
    };
    enum LightId
    {
        ENUMS(LIGHT_GATES, 16),
        LIGHTS_LEN
    };

    // ---------- State ----------
    float gateVoltages[16] = {};
    float voctVoltages[16] = {};
    float quantizedVoltages[16] = {};
    float targetVoct[16] = {};
    float currentVoct[16] = {};
    float gateTimers[16] = {};
    bool lastClockState = false;
    int clockCounter = 0;

    float gateTime = 0.01f;
    int randomRange = 0;
    int selectedScale = 0;

    float glideTime = 0.0f;
    float rootShift = 0.0f;
    int clockDiv = 1;
    float probSpread = 0.0f;
    float probSmooth = 0.0f;
    float filteredProbability = 0.5f;
    int randomMode = 0;
    bool trainMode = false;

    float glideStep[16] = {};
    float glideRemaining[16] = {};
    float lastVoct[16] = {};

    // Shift registers for train mode
    float shiftRegisterPitch[16] = {};
    float shiftRegisterGate[16] = {};

    static constexpr int MAX_PATTERN_LEN = 64;
    // One recorded step: the gate state, raw pitch and quantized pitch.
    struct PatternStep
    {
        float gate;
        float pitch;
        float quantized;
    };
    PatternStep pattern[16][MAX_PATTERN_LEN];
    int patternWritePos[16] = {};
    int patternReadPos[16] = {};
    int loopLength = 16;
    bool recordEnabled = false;
    bool playbackEnabled = false;
    bool wasRecordEnabled = false; // to detect rising edge of record switch

    float transposeOffset = 0.0f;

    uint32_t randomSeed = 0;
    std::mt19937 rng;
    bool useFixedSeed = false;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    int activePoly = 1;
    float flash[16] = {};
    bool prevGate[16] = {};

    // Static scales - accessible without a module instance
    static std::vector<Scale> scales;
    // Returns the shared static scale table (same list used by all instances).
    static const std::vector<Scale> &getScales() { return scales; }

    // Constructor: configures every parameter, input, output and light,
    // seeds the RNG from the clock, and clears the pattern buffers.
    DiceOnSteroids()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PARAM_POLYPHONY, 1.f, 16.f, 1.f, "Polyphony", " voices");
        paramQuantities[PARAM_POLYPHONY]->snapEnabled = true;
        configParam(PARAM_PROBABILITY, 0.f, 1.f, 0.5f, "Gate probability", "%", 0.f, 100.f);
        configParam(PARAM_GLIDE_TIME, 0.f, 1.f, 0.f, "Glide time", " s");
        configParam(PARAM_ROOT_SHIFT, -12.f, 12.f, 0.f, "Root shift", " semitones");
        configParam(PARAM_CLOCK_DIV, 1.f, 16.f, 1.f, "Clock division", " steps");
        paramQuantities[PARAM_CLOCK_DIV]->snapEnabled = true;
        configParam(PARAM_PROB_SPREAD, 0.f, 1.f, 0.f, "Probability spread", "%", 0.f, 100.f);
        configParam(PARAM_PROB_SMOOTH, 0.f, 1.f, 0.f, "Probability smoothing", "%", 0.f, 100.f);
        configSwitch(PARAM_RECORD_ENABLE, 0.f, 1.f, 0.f, "Record pattern", {"Off", "On"});
        configSwitch(PARAM_PLAYBACK_ENABLE, 0.f, 1.f, 0.f, "Playback pattern", {"Off", "On"});
        configParam(PARAM_LOOP_LENGTH, 1.f, MAX_PATTERN_LEN, 16.f, "Loop length", " steps");
        paramQuantities[PARAM_LOOP_LENGTH]->snapEnabled = true;
        configSwitch(PARAM_TRAIN_MODE, 0.f, 1.f, 0.f, "Train mode", {"Off", "On"});

        configInput(INPUT_CLOCK, "Clock");
        configInput(INPUT_CV_PROB, "CV Probability");
        configInput(INPUT_CV_POLY, "CV Polyphony");
        configInput(INPUT_CV_GATE_TIME, "CV Gate Time");
        configInput(INPUT_CV_RANGE, "CV Random Range");
        configInput(INPUT_CV_SCALE, "CV Scale Select");
        configInput(INPUT_CV_TRANSPOSE, "CV Transpose");
        configInput(INPUT_CV_GLIDE, "CV Glide Time");
        configInput(INPUT_RETRIGGER, "Retrigger");
        configInput(INPUT_RESET, "Reset");
        configInput(INPUT_RANDOM_SEED, "Seed Trigger");

        configOutput(OUTPUT_GATES, "Polyphonic Gates");
        configOutput(OUTPUT_VOCT, "Random V/Oct");
        configOutput(OUTPUT_QUANTIZED, "Quantized V/Oct");
        configOutput(OUTPUT_TRIGGER_NEW, "New Note Triggers");

        for (int i = 0; i < 16; ++i)
            configLight(LIGHT_GATES + i, string::f("Channel %d gate", i + 1));

        randomSeed = std::chrono::steady_clock::now().time_since_epoch().count();
        rng.seed(randomSeed);

        for (int ch = 0; ch < 16; ch++)
            for (int i = 0; i < MAX_PATTERN_LEN; i++)
                pattern[ch][i] = {0.f, 0.f, 0.f};
    }

    // Returns a uniformly distributed random float in [0, 1) from the
    // module's mt19937 generator.
    float randFloat()
    {
        return std::uniform_real_distribution<float>(0, 1)(rng);
    }

    // Generates a new random voltage according to the selected random mode
    // (uniform / walk / drunk / Gaussian) and range. `currentVoltage` is the
    // previous pitch, used by the walk/drunk modes to step from.
    float generateRandomVoltage(float currentVoltage = 0.f)
    {
        float voltage = 0.f;
        switch (randomMode)
        {
        case 0:
            switch (randomRange)
            {
            case 0:
                voltage = randFloat();
                break;
            case 1:
                voltage = randFloat() * 2.f - 1.f;
                break;
            case 2:
                voltage = randFloat() * 4.f - 2.f;
                break;
            case 3:
                voltage = randFloat() * 10.f - 5.f;
                break;
            }
            break;
        case 1:
        {
            float step = (randFloat() - 0.5f) * 0.2f;
            voltage = currentVoltage + step;
            float maxV = (randomRange == 0) ? 1.f : ((randomRange == 1) ? 1.f : ((randomRange == 2) ? 2.f : 5.f));
            float minV = (randomRange == 0) ? 0.f : ((randomRange == 1) ? -1.f : ((randomRange == 2) ? -2.f : -5.f));
            voltage = clamp(voltage, minV, maxV);
            break;
        }
        case 2:
            if (randFloat() < 0.1f)
            {
                switch (randomRange)
                {
                case 0:
                    voltage = randFloat();
                    break;
                case 1:
                    voltage = randFloat() * 2.f - 1.f;
                    break;
                case 2:
                    voltage = randFloat() * 4.f - 2.f;
                    break;
                case 3:
                    voltage = randFloat() * 10.f - 5.f;
                    break;
                }
            }
            else
            {
                float step = (randFloat() - 0.5f) * 0.05f;
                voltage = currentVoltage + step;
                float maxV = (randomRange == 0) ? 1.f : ((randomRange == 1) ? 1.f : ((randomRange == 2) ? 2.f : 5.f));
                float minV = (randomRange == 0) ? 0.f : ((randomRange == 1) ? -1.f : ((randomRange == 2) ? -2.f : -5.f));
                voltage = clamp(voltage, minV, maxV);
            }
            break;
        case 3:
        {
            float u1 = randFloat();
            float u2 = randFloat();
            float radius = std::sqrt(-2.f * std::log(u1));
            float theta = 2.f * M_PI * u2;
            float gaussian = radius * std::cos(theta);
            float rangeWidth = (randomRange == 0) ? 1.f : ((randomRange == 1) ? 2.f : ((randomRange == 2) ? 4.f : 10.f));
            float center = (randomRange == 0) ? 0.5f : 0.f;
            voltage = center + gaussian * (rangeWidth / 6.f);
            voltage = clamp(voltage, center - rangeWidth / 2, center + rangeWidth / 2);
            break;
        }
        }
        return voltage;
    }

    // Quantizes `voltage` (in V/Oct, plus `rootSemitone` offset) to the
    // nearest degree of `scale`, returning the quantized voltage with the
    // root offset removed. Works in semitone space and considers the wrap
    // into the next octave so notes near the top of a scale snap correctly.
    float quantizeToScale(float voltage, const Scale &scale, float rootSemitone)
    {
        float semitones = voltage * 12.0f + rootSemitone;
        int octave = (int)std::floor(semitones);
        float fractional = semitones - octave;
        float bestMatch = 0.f;
        float minDiff = 100.f;
        for (float interval : scale.intervals)
        {
            float degree = interval / 12.0f;
            float diff = std::abs(fractional - degree);
            if (diff < minDiff)
            {
                minDiff = diff;
                bestMatch = degree;
            }
            diff = std::abs((fractional + 1.0f) - degree);
            if (diff < minDiff)
            {
                minDiff = diff;
                bestMatch = degree - 1.0f;
            }
        }
        float outSemis = (octave + (int)std::floor(bestMatch + 0.5f)) + bestMatch;
        return (outSemis - rootSemitone) / 12.0f;
    }

    // Records the current gate/pitch/quantized state of `channel` into its
    // pattern buffer at the write head, then advances the head. No-op when
    // recording is off.
    void recordStep(int channel, float gate, float pitch, float quant)
    {
        if (!recordEnabled)
            return;
        pattern[channel][patternWritePos[channel]] = {gate, pitch, quant};
        patternWritePos[channel] = (patternWritePos[channel] + 1) % loopLength;
    }

    // Reads the next recorded step of `channel` from its pattern buffer into
    // gate/pitch/quant, then advances the read head. No-op when playback is
    // off.
    void playStep(int channel, float &gate, float &pitch, float &quant)
    {
        if (!playbackEnabled)
            return;
        const PatternStep &step = pattern[channel][patternReadPos[channel]];
        gate = step.gate;
        pitch = step.pitch;
        quant = step.quantized;
        patternReadPos[channel] = (patternReadPos[channel] + 1) % loopLength;
    }

    // Clears all per-channel state (gates, pitches, glide, shift registers,
    // pattern heads and the pattern buffers themselves) and resets the
    // clock/edge tracking. Called on RESET.
    void resetAll()
    {
        for (int c = 0; c < 16; ++c)
        {
            gateVoltages[c] = 0.f;
            voctVoltages[c] = 0.f;
            quantizedVoltages[c] = 0.f;
            gateTimers[c] = 0.f;
            lastVoct[c] = 0.f;
            currentVoct[c] = 0.f;
            targetVoct[c] = 0.f;
            glideRemaining[c] = 0.f;
            patternWritePos[c] = 0;
            patternReadPos[c] = 0;
            shiftRegisterPitch[c] = 0.f;
            shiftRegisterGate[c] = 0.f;
            // Clear pattern buffer to avoid garbage during playback
            for (int i = 0; i < MAX_PATTERN_LEN; ++i)
            {
                pattern[c][i] = {0.f, 0.f, 0.f};
            }
        }
        lastClockState = false;
        clockCounter = 0;
    }

    // Serialises all persistent settings (gate time, range, scale, glide,
    // random mode, clock division, seed, train mode) and the full 16x64
    // pattern buffer so a patch reopens with the same sequence.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "gateTime", json_real(gateTime));
        json_object_set_new(root, "randomRange", json_integer(randomRange));
        json_object_set_new(root, "selectedScale", json_integer(selectedScale));
        json_object_set_new(root, "glideTime", json_real(glideTime));
        json_object_set_new(root, "randomMode", json_integer(randomMode));
        json_object_set_new(root, "clockDiv", json_integer(clockDiv));
        json_object_set_new(root, "useFixedSeed", json_boolean(useFixedSeed));
        json_object_set_new(root, "randomSeed", json_integer(randomSeed));
        json_object_set_new(root, "trainMode", json_boolean(trainMode));
        json_t *patternArr = json_array();
        for (int ch = 0; ch < 16; ++ch)
        {
            json_t *chArr = json_array();
            for (int i = 0; i < MAX_PATTERN_LEN; ++i)
            {
                json_t *stepObj = json_object();
                json_object_set_new(stepObj, "gate", json_real(pattern[ch][i].gate));
                json_object_set_new(stepObj, "pitch", json_real(pattern[ch][i].pitch));
                json_object_set_new(stepObj, "quant", json_real(pattern[ch][i].quantized));
                json_array_append_new(chArr, stepObj);
            }
            json_array_append_new(patternArr, chArr);
        }
        json_object_set_new(root, "pattern", patternArr);
        return root;
    }

    // Restores all settings and the 16x64 pattern buffer from a saved patch,
    // clamping each scalar to its valid range and re-seeding the RNG.
    void dataFromJson(json_t *root) override
    {
        auto readFloat = [&](const char *key, float &out, float lo, float hi)
        {
            json_t *j = json_object_get(root, key);
            if (j && json_is_number(j))
                out = clamp((float)json_number_value(j), lo, hi);
        };
        auto readInt = [&](const char *key, int &out, int lo, int hi)
        {
            json_t *j = json_object_get(root, key);
            if (j && json_is_integer(j))
                out = clamp((int)json_integer_value(j), lo, hi);
        };
        readFloat("gateTime", gateTime, 0.001f, 1.f);
        readInt("randomRange", randomRange, 0, 3);
        readInt("selectedScale", selectedScale, 0, (int)scales.size() - 1);
        readFloat("glideTime", glideTime, 0.f, 1.f);
        readInt("randomMode", randomMode, 0, 3);
        readInt("clockDiv", clockDiv, 1, 16);
        json_t *ufs = json_object_get(root, "useFixedSeed");
        if (ufs)
            useFixedSeed = json_is_true(ufs);
        json_t *seed = json_object_get(root, "randomSeed");
        if (seed && json_is_integer(seed))
        {
            randomSeed = (uint32_t)json_integer_value(seed);
            rng.seed(randomSeed);
        }
        json_t *tm = json_object_get(root, "trainMode");
        if (tm)
            trainMode = json_is_true(tm);
        json_t *pat = json_object_get(root, "pattern");
        if (pat && json_is_array(pat))
        {
            size_t chCount = json_array_size(pat);
            for (size_t ch = 0; ch < 16 && ch < chCount; ++ch)
            {
                json_t *chArr = json_array_get(pat, ch);
                if (chArr && json_is_array(chArr))
                {
                    size_t stepCount = json_array_size(chArr);
                    for (size_t i = 0; i < MAX_PATTERN_LEN && i < stepCount; ++i)
                    {
                        json_t *stepObj = json_array_get(chArr, i);
                        if (stepObj && json_is_object(stepObj))
                        {
                            json_t *g = json_object_get(stepObj, "gate");
                            json_t *p = json_object_get(stepObj, "pitch");
                            json_t *q = json_object_get(stepObj, "quant");
                            pattern[ch][i].gate = (g && json_is_number(g)) ? (float)json_number_value(g) : 0.f;
                            pattern[ch][i].pitch = (p && json_is_number(p)) ? (float)json_number_value(p) : 0.f;
                            pattern[ch][i].quantized = (q && json_is_number(q)) ? (float)json_number_value(q) : 0.f;
                        }
                    }
                }
            }
        }
    }

    // Main audio callback: reads controls + CVs (probability, polyphony,
    // gate time, range, scale, transpose, glide), handles reset/retrigger/
    // seed, runs the clock-division, train-mode, record/playback and
    // independent random modes, applies glide and gate timers, writes the
    // four polyphonic outputs and updates the LEDs/flash envelopes.
    void process(const ProcessArgs &args) override
    {
        float polyKnob = params[PARAM_POLYPHONY].getValue();
        float polyCV = inputs[INPUT_CV_POLY].getVoltage() / 5.f;
        int polyphony = clamp((int)std::round(polyKnob + polyCV * 15.f), 1, 16);

        float probKnob = params[PARAM_PROBABILITY].getValue();
        float probCV = inputs[INPUT_CV_PROB].getVoltage() / 5.f;
        float rawProb = probKnob + probCV;
        float smoothCoeff = params[PARAM_PROB_SMOOTH].getValue();
        if (smoothCoeff > 0.f)
        {
            float alpha = args.sampleTime / (smoothCoeff * 0.1f + 0.001f);
            alpha = clamp(alpha, 0.f, 1.f);
            filteredProbability += alpha * (clamp(rawProb, 0.f, 1.f) - filteredProbability);
        }
        else
        {
            filteredProbability = clamp(rawProb, 0.f, 1.f);
        }
        float probability = filteredProbability;

        float gateCV = inputs[INPUT_CV_GATE_TIME].getVoltage() / 5.f;
        float curGateTime = clamp(gateTime + gateCV * 0.5f, 0.001f, 0.5f);

        // Range CV only if connected
        if (inputs[INPUT_CV_RANGE].isConnected())
        {
            float rangeCV = inputs[INPUT_CV_RANGE].getVoltage() / 5.f;
            int targetRange = clamp((int)(rangeCV * 3.9f), 0, 3);
            if (targetRange != randomRange)
                randomRange = targetRange;
        }

        // Scale CV only if connected (otherwise the menu selection is kept)
        if (inputs[INPUT_CV_SCALE].isConnected())
        {
            float scaleCV = inputs[INPUT_CV_SCALE].getVoltage() / 5.f;
            int newScale = clamp((int)(scaleCV * (scales.size() - 1)), 0, (int)scales.size() - 1);
            if (newScale != selectedScale)
                selectedScale = newScale;
        }

        float transpCV = inputs[INPUT_CV_TRANSPOSE].getVoltage();
        transposeOffset = transpCV;
        float totalRootShift = params[PARAM_ROOT_SHIFT].getValue() + transposeOffset;

        float glideKnob = params[PARAM_GLIDE_TIME].getValue();
        float glideCV = inputs[INPUT_CV_GLIDE].getVoltage() / 5.f;
        glideTime = clamp(glideKnob + glideCV * 0.5f, 0.f, 1.f);

        int effectiveDiv = (int)std::round(params[PARAM_CLOCK_DIV].getValue());
        effectiveDiv = clamp(effectiveDiv, 1, 16);
        clockDiv = effectiveDiv;

        float spread = params[PARAM_PROB_SPREAD].getValue();

        bool newRecordEnabled = params[PARAM_RECORD_ENABLE].getValue() > 0.5f;
        // If record was just turned on, reset write positions to start overwriting from beginning
        if (newRecordEnabled && !wasRecordEnabled)
        {
            for (int c = 0; c < 16; ++c)
            {
                patternWritePos[c] = 0;
            }
        }
        recordEnabled = newRecordEnabled;
        wasRecordEnabled = recordEnabled;

        playbackEnabled = params[PARAM_PLAYBACK_ENABLE].getValue() > 0.5f;
        loopLength = (int)params[PARAM_LOOP_LENGTH].getValue();
        trainMode = params[PARAM_TRAIN_MODE].getValue() > 0.5f;

        float clockVoltage = inputs[INPUT_CLOCK].getVoltage();
        bool clockHigh = clockVoltage >= 1.f;
        bool risingEdge = clockHigh && !lastClockState;
        lastClockState = clockHigh;

        bool resetTrigger = inputs[INPUT_RESET].getVoltage() > 1.f;
        bool retrigger = inputs[INPUT_RETRIGGER].getVoltage() > 1.f;
        bool seedTrigger = inputs[INPUT_RANDOM_SEED].getVoltage() > 1.f;

        if (resetTrigger)
            resetAll();
        if (seedTrigger && !useFixedSeed)
        {
            randomSeed = std::chrono::steady_clock::now().time_since_epoch().count();
            rng.seed(randomSeed);
        }

        // Manual retrigger (skip clock division)
        if (retrigger)
        {
            for (int c = 0; c < polyphony; ++c)
            {
                float channelProb = clamp(probability * (1.f + spread * ((float)c / polyphony * 2.f - 1.f)), 0.f, 1.f);
                if (randFloat() < channelProb)
                {
                    gateVoltages[c] = 10.f;
                    gateTimers[c] = curGateTime;
                    float rawV;
                    if (trainMode && c == 0)
                    {
                        rawV = generateRandomVoltage(lastVoct[c]);
                        shiftRegisterPitch[c] = rawV;
                        shiftRegisterGate[c] = 1.f;
                    }
                    else if (trainMode && c > 0)
                    {
                        rawV = shiftRegisterPitch[c - 1];
                    }
                    else
                    {
                        rawV = generateRandomVoltage(lastVoct[c]);
                    }
                    lastVoct[c] = rawV;
                    targetVoct[c] = rawV;
                    voctVoltages[c] = rawV;
                    quantizedVoltages[c] = quantizeToScale(rawV, scales[selectedScale], totalRootShift);
                    outputs[OUTPUT_TRIGGER_NEW].setVoltage(10.f, c);
                    if (recordEnabled)
                        recordStep(c, gateVoltages[c], rawV, quantizedVoltages[c]);
                }
                else
                {
                    gateVoltages[c] = 0.f;
                    outputs[OUTPUT_TRIGGER_NEW].setVoltage(0.f, c);
                    if (trainMode)
                        shiftRegisterGate[c] = 0.f;
                    if (recordEnabled)
                        recordStep(c, 0.f, voctVoltages[c], quantizedVoltages[c]);
                }
            }
        }

        // Clock edge with division
        if (risingEdge)
        {
            clockCounter = (clockCounter + 1) % effectiveDiv;
            bool clockFire = (clockCounter == 0);
            if (clockFire)
            {
                if (trainMode)
                {
                    // Shift registers: move values to higher channels
                    for (int c = polyphony - 1; c > 0; --c)
                    {
                        shiftRegisterPitch[c] = shiftRegisterPitch[c - 1];
                        shiftRegisterGate[c] = shiftRegisterGate[c - 1];
                    }
                    // Generate new pitch and gate for channel 1 (index 0)
                    float spreadFactor0 = 1.f + spread * ((0) / (float)polyphony * 2.f - 1.f);
                    float channelProb = clamp(probability * spreadFactor0, 0.f, 1.f);
                    shiftRegisterGate[0] = (randFloat() < channelProb) ? 1.f : 0.f;
                    shiftRegisterPitch[0] = generateRandomVoltage(shiftRegisterPitch[0]);

                    // Apply to all channels
                    for (int c = 0; c < polyphony; ++c)
                    {
                        if (playbackEnabled)
                        {
                            playStep(c, gateVoltages[c], voctVoltages[c], quantizedVoltages[c]);
                            gateTimers[c] = (gateVoltages[c] > 0.f) ? curGateTime : 0.f;
                        }
                        else
                        {
                            float gateOn = shiftRegisterGate[c];
                            if (gateOn > 0.5f)
                            {
                                gateVoltages[c] = 10.f;
                                gateTimers[c] = curGateTime;
                                float rawV = shiftRegisterPitch[c];
                                lastVoct[c] = rawV;
                                targetVoct[c] = rawV;
                                voctVoltages[c] = rawV;
                                quantizedVoltages[c] = quantizeToScale(rawV, scales[selectedScale], totalRootShift);
                                outputs[OUTPUT_TRIGGER_NEW].setVoltage(10.f, c);
                            }
                            else
                            {
                                gateVoltages[c] = 0.f;
                                outputs[OUTPUT_TRIGGER_NEW].setVoltage(0.f, c);
                            }
                        }
                        // Record the final state (whether playback or generated) on every clock tick
                        if (recordEnabled)
                        {
                            recordStep(c, gateVoltages[c], voctVoltages[c], quantizedVoltages[c]);
                        }
                    }
                }
                else
                {
                    // Normal independent mode
                    for (int c = 0; c < polyphony; ++c)
                    {
                        float spreadFactor = 1.f + spread * ((float)c / polyphony * 2.f - 1.f);
                        float channelProb = clamp(probability * spreadFactor, 0.f, 1.f);
                        if (playbackEnabled)
                        {
                            playStep(c, gateVoltages[c], voctVoltages[c], quantizedVoltages[c]);
                            gateTimers[c] = (gateVoltages[c] > 0.f) ? curGateTime : 0.f;
                        }
                        else
                        {
                            if (randFloat() < channelProb)
                            {
                                gateVoltages[c] = 10.f;
                                gateTimers[c] = curGateTime;
                                float rawV = generateRandomVoltage(lastVoct[c]);
                                lastVoct[c] = rawV;
                                targetVoct[c] = rawV;
                                voctVoltages[c] = rawV;
                                quantizedVoltages[c] = quantizeToScale(rawV, scales[selectedScale], totalRootShift);
                                outputs[OUTPUT_TRIGGER_NEW].setVoltage(10.f, c);
                            }
                            else
                            {
                                gateVoltages[c] = 0.f;
                                outputs[OUTPUT_TRIGGER_NEW].setVoltage(0.f, c);
                            }
                        }
                        // Record every clock tick
                        if (recordEnabled)
                        {
                            recordStep(c, gateVoltages[c], voctVoltages[c], quantizedVoltages[c]);
                        }
                    }
                }
            }
        }

        // Glide (portamento)
        for (int c = 0; c < 16; ++c)
        {
            if (glideTime > 0.f && glideRemaining[c] <= 0.f && targetVoct[c] != currentVoct[c])
            {
                float diff = targetVoct[c] - currentVoct[c];
                float steps = args.sampleRate * glideTime;
                glideStep[c] = diff / steps;
                glideRemaining[c] = steps;
            }
            if (glideRemaining[c] > 0.f)
            {
                currentVoct[c] += glideStep[c];
                glideRemaining[c] -= 1.f;
                if (glideRemaining[c] <= 0.f)
                    currentVoct[c] = targetVoct[c];
                voctVoltages[c] = currentVoct[c];
                quantizedVoltages[c] = quantizeToScale(currentVoct[c], scales[selectedScale], totalRootShift);
            }
        }

        // Gate timers
        for (int c = 0; c < 16; ++c)
        {
            if (gateTimers[c] > 0.f)
            {
                gateTimers[c] -= args.sampleTime;
                if (gateTimers[c] <= 0.f)
                    gateVoltages[c] = 0.f;
            }
        }

        // Polyphony cleanup
        for (int c = polyphony; c < 16; ++c)
        {
            gateVoltages[c] = 0.f;
            voctVoltages[c] = 0.f;
            quantizedVoltages[c] = 0.f;
            gateTimers[c] = 0.f;
        }

        // Outputs
        outputs[OUTPUT_GATES].setChannels(polyphony);
        outputs[OUTPUT_VOCT].setChannels(polyphony);
        outputs[OUTPUT_QUANTIZED].setChannels(polyphony);
        outputs[OUTPUT_TRIGGER_NEW].setChannels(polyphony);
        for (int c = 0; c < polyphony; ++c)
        {
            outputs[OUTPUT_GATES].setVoltage(gateVoltages[c], c);
            outputs[OUTPUT_VOCT].setVoltage(voctVoltages[c], c);
            outputs[OUTPUT_QUANTIZED].setVoltage(quantizedVoltages[c], c);
        }

        // LEDs and display flashes: each gate lights up and then fades,
        // so even a 5 ms gate is visible
        activePoly = polyphony;
        for (int c = 0; c < 16; ++c)
        {
            bool g = (c < polyphony) && (gateVoltages[c] > 0.f);
            if (g)
                flash[c] = 1.f;
            else
                flash[c] = std::max(0.f, flash[c] - 8.f * args.sampleTime);
            prevGate[c] = g;
            lights[LIGHT_GATES + c].setBrightness(flash[c]);
        }

        // Clear one-shot triggers
        for (int c = 0; c < polyphony; ++c)
        {
            if (outputs[OUTPUT_TRIGGER_NEW].getVoltage(c) > 0.f)
                outputs[OUTPUT_TRIGGER_NEW].setVoltage(0.f, c);
        }
    }
};

// Static scales definition (unchanged)
std::vector<Scale> DiceOnSteroids::scales = {
    {"Major", {0, 2, 4, 5, 7, 9, 11}},
    {"Minor (Natural)", {0, 2, 3, 5, 7, 8, 10}},
    {"Harmonic Minor", {0, 2, 3, 5, 7, 8, 11}},
    {"Melodic Minor", {0, 2, 3, 5, 7, 9, 11}},
    {"Chromatic", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}},
    {"Pentatonic Major", {0, 2, 4, 7, 9}},
    {"Pentatonic Minor", {0, 3, 5, 7, 10}},
    {"Blues", {0, 3, 5, 6, 7, 10}},
    {"Bebop", {0, 2, 4, 5, 7, 9, 10, 11}},
    {"Dorian", {0, 2, 3, 5, 7, 9, 10}},
    {"Phrygian", {0, 1, 3, 5, 7, 8, 10}},
    {"Lydian", {0, 2, 4, 6, 7, 9, 11}},
    {"Mixolydian", {0, 2, 4, 5, 7, 9, 10}},
    {"Aeolian", {0, 2, 3, 5, 7, 8, 10}},
    {"Locrian", {0, 1, 3, 5, 6, 8, 10}},
    {"Egyptian", {0, 2, 5, 7, 10}},
    {"Japanese (Hirajoshi)", {0, 2, 3, 7, 8}},
    {"Japanese (Kumoi)", {0, 2, 3, 7, 9}},
    {"Japanese (Iwato)", {0, 1, 5, 6, 10}},
    {"Hungarian Minor", {0, 2, 3, 6, 7, 8, 11}},
    {"Romanian Minor", {0, 2, 3, 6, 7, 9, 10}},
    {"Spanish (Phrygian)", {0, 1, 3, 5, 7, 8, 10}},
    {"Flamenco", {0, 1, 3, 4, 5, 7, 8, 10}},
    {"Whole Tone", {0, 2, 4, 6, 8, 10}},
    {"Diminished", {0, 2, 3, 5, 6, 8, 9, 11}},
    {"Augmented", {0, 3, 4, 7, 8, 11}},
    {"Enigmatic", {0, 1, 4, 6, 8, 10, 11}},
    {"Balinese", {0, 1, 3, 5, 7, 8, 10}},
    {"Pelog", {0, 1, 3, 4, 5, 7, 8, 10}},
    {"Slendro", {0, 3, 5, 7, 10}},
    {"Arabian", {0, 1, 4, 5, 7, 8, 11}},
    {"Persian", {0, 1, 4, 5, 6, 8, 11}},
    {"Byzantine", {0, 1, 4, 5, 7, 8, 11}},
    {"Egyptian Minor", {0, 2, 3, 6, 7, 8, 11}},
    {"Hindu", {0, 2, 4, 5, 7, 8, 10}},
    {"Oriental", {0, 1, 4, 5, 6, 9, 10}},
    {"Mysterious", {0, 1, 3, 5, 6, 7, 8, 10}},
    {"Prometheus", {0, 2, 4, 6, 9, 10}},
    {"Prometheus Neapolitan", {0, 1, 3, 6, 8, 10}},
    {"Neapolitan Major", {0, 1, 3, 5, 7, 9, 11}},
    {"Neapolitan Minor", {0, 1, 3, 5, 7, 8, 11}},
    {"Double Harmonic", {0, 1, 4, 5, 7, 8, 11}},
    {"Hungarian Gypsy", {0, 2, 3, 6, 7, 8, 11}},
    {"Ukrainian Dorian", {0, 2, 3, 6, 7, 9, 10}},
    {"Marva", {0, 1, 4, 6, 7, 9, 11}},
    {"Todi", {0, 1, 3, 6, 7, 8, 11}},
    {"Purvi", {0, 1, 4, 6, 7, 8, 11}},
    {"Bhairav", {0, 1, 4, 5, 7, 8, 11}},
    {"Bilawal", {0, 2, 4, 5, 7, 9, 11}},
    {"Kafi", {0, 2, 3, 5, 7, 9, 10}},
    {"Asavari", {0, 2, 3, 5, 7, 8, 10}},
    {"C Major Blues", {0, 2, 3, 5, 7, 8, 10}},
    {"C Minor Blues", {0, 3, 5, 6, 7, 10}},
    {"Major Locrian", {0, 2, 4, 5, 6, 8, 10}},
    {"Lydian Minor", {0, 2, 4, 6, 7, 8, 10}},
    {"Lydian Augmented", {0, 2, 4, 6, 8, 9, 11}},
    {"Phrygian Dominant", {0, 1, 4, 5, 7, 8, 10}},
    {"Spanish Gypsy", {0, 1, 4, 5, 7, 8, 10}},
    {"Algerian", {0, 2, 4, 5, 7, 8, 10}}};

// ------------------------------------------------------------
// UI (everything below is in an anonymous namespace so the names cannot
// clash with same-named helpers in other module files)
// ------------------------------------------------------------
namespace
{

    // Draws an agmm section header whose hairlines span only [x0, x0+w]
    // instead of the full panel width, by translating the context.
    void dsBlockHeader(NVGcontext *vg, float x0, float w, float y, const char *txt)
    {
        nvgSave(vg);
        nvgTranslate(vg, x0, 0.f);
        agmm::sectionHeader(vg, w, y, txt, 0.f);
        nvgRestore(vg);
    }

    // Static artwork: base panel, display bezel, the two control blocks,
    // output and transport blocks, the numbered LED strip plate, and the
    // input row.
    struct DSPanelBackground : TransparentWidget
    {
        DSPanelBackground() { box.size = Vec(DS_PANEL_W, DS_PANEL_H); }

        // Draws all static panel art: base panel, bezel, left CONTROL knobs,
        // right OUTPUT jacks, right TRANSPORT switches, the LED strip with
        // channel numbers, and the input row.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, DS_PANEL_W, DS_PANEL_H, "DICE ON STEROIDS");
            agmm::bezel(vg, DS_DISP_X, DS_DISP_Y, DS_DISP_W, DS_DISP_H);

            // ---- left block: 8 knobs, label above each ----
            dsBlockHeader(vg, 10.f, 82.f, DS_BLOCK_HEAD_Y, "CONTROL");
            static const char *kl[8] = {"POLY", "PROB", "GLIDE", "ROOT", "CLK DIV", "SPREAD", "SMOOTH", "LOOP LEN"};
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 2; ++c)
                    agmm::cell(vg, DS_LCOL[c], dsLTop(r), DS_LW, DS_LH, kl[r * 2 + c], false, 7.f);

            // ---- right block: outputs ----
            dsBlockHeader(vg, 334.f, 76.f, DS_BLOCK_HEAD_Y, "OUTPUT");
            agmm::cell(vg, DS_RCOL[0], DS_ROUT_TOP[0], DS_RW, DS_ROUT_H, "GATES", true);
            agmm::cell(vg, DS_RCOL[1], DS_ROUT_TOP[0], DS_RW, DS_ROUT_H, "RANDOM", true);
            agmm::cell(vg, DS_RCOL[0], DS_ROUT_TOP[1], DS_RW, DS_ROUT_H, "QUANT", true);
            agmm::cell(vg, DS_RCOL[1], DS_ROUT_TOP[1], DS_RW, DS_ROUT_H, "TRIG", true);

            // ---- right block: transport ----
            dsBlockHeader(vg, 334.f, 76.f, DS_TRANS_HEAD_Y, "TRANSPORT");
            agmm::cell(vg, DS_RCOL[0], DS_RSW_TOP[0], DS_RW, DS_RSW_H, "REC");
            agmm::cell(vg, DS_RCOL[1], DS_RSW_TOP[0], DS_RW, DS_RSW_H, "PLAY");
            agmm::cell(vg, DS_RCOL[0], DS_RSW_TOP[1], DS_RW, DS_RSW_H, "TRAIN");
            agmm::cell(vg, DS_RCOL[1], DS_RSW_TOP[1], DS_RW, DS_RSW_H, "SEED");

            // ---- gate LED strip: channel numbers above the LEDs ----
            agmm::plate(vg, DS_DISP_X + DS_DISP_W / 2.f, DS_LED_TOP, DS_DISP_W, DS_LED_H);
            for (int i = 0; i < 16; ++i)
            {
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", i + 1);
                agmm::text(vg, DS_DISP_X + DS_CH_X0 + (i + 0.5f) * DS_CH_W, DS_LED_NUM_Y, buf, 7.f,
                           agmm::dim(), agmm::ALIGN_C, 0.f);
            }

            // ---- input row ----
            agmm::sectionHeader(vg, DS_PANEL_W, DS_IN_HEAD_Y, "INPUTS", 14.f);
            static const char *in[10] = {"CLK", "RESET", "RETRIG", "CV PROB", "CV POLY",
                                         "CV GATE", "CV RANGE", "CV SCALE", "CV TRANS", "CV GLIDE"};
            for (int i = 0; i < 10; ++i)
                agmm::cell(vg, dsInX(i), DS_IN_TOP, DS_IN_W, DS_IN_H, in[i], false, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display rows (top to bottom):
    //   y  9  scale name / random mode
    //   y 21  root / range / gate time
    //   30-110  channel plot (no text inside)
    //   118  pattern strip (channel 1)
    //   134  status chips + loop readout
    //   156  probability meter
    // ------------------------------------------------------------
    struct DSDisplayWidget : TransparentWidget
    {
        DiceOnSteroids *module = nullptr;

        DSDisplayWidget(DiceOnSteroids *m) : module(m)
        {
            box.pos = Vec(DS_DISP_X, DS_DISP_Y);
            box.size = Vec(DS_DISP_W, DS_DISP_H);
        }

        static constexpr float PLOT_Y = 30.f, PLOT_H = 80.f;
        static constexpr float PAT_Y = 118.f, PAT_H = 8.f;
        static constexpr float CHIP_Y = 134.f, CHIP_H = 12.f;
        static constexpr float METER_Y = 156.f;

        // Static layer: fills the display area with the flat screen colour.
        void draw(const DrawArgs &args) override
        {
            agmm::screen(args.vg, 0.f, 0.f, DS_DISP_W, DS_DISP_H);
            Widget::draw(args);
        }

        // Layer-1 overlay: draws the readout rows, the 16-channel pitch plot
        // (quantized bars + raw ticks), the CH1 pattern strip with play/record
        // markers, the REC/PLAY/TRAIN chips with a loop/div readout, and the
        // probability meter. Falls back to a sample scene when there is no
        // module.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = DS_DISP_W;
            const float mid = PLOT_Y + PLOT_H / 2.f;
            const float amp = PLOT_H / 2.f - 3.f; // pixels for 5 V
            const float plotX0 = DS_CH_X0, plotW = 16.f * DS_CH_W;

            // ---- gather state (live, or a sample scene in the module browser) ----
            int poly, loopLen, readPos = 0, writePos = 0, scaleIdx, mode, range, div;
            float quant[16], raw[16], fl[16], gateMs, rootSemi, prob;
            float stepGate[DiceOnSteroids::MAX_PATTERN_LEN];
            bool rec, play, train;

            for (int c = 0; c < 16; ++c)
                quant[c] = raw[c] = fl[c] = 0.f;
            for (int i = 0; i < DiceOnSteroids::MAX_PATTERN_LEN; ++i)
                stepGate[i] = 0.f;

            if (module)
            {
                poly = module->activePoly;
                for (int c = 0; c < 16; ++c)
                {
                    quant[c] = module->quantizedVoltages[c];
                    raw[c] = module->voctVoltages[c];
                    fl[c] = module->flash[c];
                }
                loopLen = clamp(module->loopLength, 1, DiceOnSteroids::MAX_PATTERN_LEN);
                for (int i = 0; i < loopLen; ++i)
                    stepGate[i] = module->pattern[0][i].gate;
                readPos = module->patternReadPos[0];
                writePos = module->patternWritePos[0];
                scaleIdx = module->selectedScale;
                mode = module->randomMode;
                range = module->randomRange;
                div = module->clockDiv;
                gateMs = module->gateTime * 1000.f;
                rootSemi = module->params[DiceOnSteroids::PARAM_ROOT_SHIFT].getValue();
                prob = module->filteredProbability;
                rec = module->recordEnabled;
                play = module->playbackEnabled;
                train = module->trainMode;
            }
            else
            {
                poly = 10;
                for (int c = 0; c < 16; ++c)
                {
                    quant[c] = 3.4f * std::sin(c * 0.85f + 0.6f);
                    raw[c] = quant[c] + 0.4f * std::sin(c * 2.3f);
                    fl[c] = (c % 3 == 0) ? 1.f : (c % 5 == 0 ? 0.5f : 0.f);
                }
                loopLen = 16;
                for (int i = 0; i < loopLen; ++i)
                    stepGate[i] = ((i * 7) % 5 < 3) ? 10.f : 0.f;
                readPos = 5;
                scaleIdx = 0;
                mode = 0;
                range = 3;
                div = 1;
                gateMs = 10.f;
                rootSemi = 0.f;
                prob = 0.6f;
                rec = false;
                play = true;
                train = false;
            }
            poly = clamp(poly, 1, 16);
            scaleIdx = clamp(scaleIdx, 0, (int)DiceOnSteroids::scales.size() - 1);
            mode = clamp(mode, 0, 3);
            range = clamp(range, 0, 3);

            char buf[64];

            // ---- row 1 + 2: readouts ----
            static const char *modeNames[4] = {"UNIFORM", "WALK", "DRUNK", "GAUSS"};
            static const char *rangeNames[4] = {"0/+1V", "-1/+1V", "-2/+2V", "-5/+5V"};

            const float modeW = agmm::textWidth(vg, modeNames[mode], 7.f);
            agmm::text(vg, W - 8.f, 9.f, modeNames[mode], 7.f, agmm::accent(), agmm::ALIGN_R);
            std::string sname = DiceOnSteroids::scales[scaleIdx].name;
            const float maxW = W - 16.f - modeW - 8.f;
            while (sname.size() > 3 && agmm::textWidth(vg, sname.c_str(), 7.f) > maxW)
                sname.pop_back();
            agmm::text(vg, 8.f, 9.f, sname.c_str(), 7.f, agmm::accent(), agmm::ALIGN_L);

            snprintf(buf, sizeof(buf), "ROOT %+.1f", rootSemi);
            agmm::text(vg, 8.f, 21.f, buf, 7.f, agmm::dim(), agmm::ALIGN_L);
            snprintf(buf, sizeof(buf), "RNG %s", rangeNames[range]);
            agmm::text(vg, W / 2.f, 21.f, buf, 7.f, agmm::dim(), agmm::ALIGN_C);
            snprintf(buf, sizeof(buf), "GATE %.0fMS", gateMs);
            agmm::text(vg, W - 8.f, 21.f, buf, 7.f, agmm::dim(), agmm::ALIGN_R);

            // ---- channel plot ----
            agmm::plotFrame(vg, plotX0 - 4.f, PLOT_Y, plotW + 8.f, PLOT_H);
            for (int c = 0; c < 16; ++c)
            {
                nvgBeginPath(vg);
                nvgRect(vg, plotX0 + c * DS_CH_W + 1.5f, PLOT_Y + 1.f, DS_CH_W - 3.f, PLOT_H - 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 8));
                nvgFill(vg);
            }
            nvgBeginPath(vg);
            for (int k = -1; k <= 1; ++k)
            {
                float y = std::floor(mid - k * amp / 2.f) + 0.5f;
                nvgMoveTo(vg, plotX0 - 3.f, y);
                nvgLineTo(vg, plotX0 + plotW + 3.f, y);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 24));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            for (int c = 0; c < 16; ++c)
            {
                float x = plotX0 + c * DS_CH_W + 1.5f;
                float w = DS_CH_W - 3.f;
                if (c >= poly)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 2.f, mid, w - 4.f, 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 30));
                    nvgFill(vg);
                    continue;
                }
                float f = clamp(fl[c], 0.f, 1.f);

                // quantized pitch bar from the zero line
                float yq = mid - clamp(quant[c] / 5.f, -1.f, 1.f) * amp;
                float top = std::min(mid, yq);
                float h = std::max(std::fabs(mid - yq), 1.5f);
                nvgBeginPath(vg);
                nvgRect(vg, x, top, w, h);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (int)(90 + 150 * f)));
                nvgFill(vg);

                // random (unquantized) pitch: violet tick
                float yr = mid - clamp(raw[c] / 5.f, -1.f, 1.f) * amp;
                nvgBeginPath(vg);
                nvgRect(vg, x - 0.5f, yr - 0.5f, w + 1.f, 1.f);
                nvgFillColor(vg, agmm::violet());
                nvgFill(vg);
            }

            // ---- pattern strip (channel 1) ----
            const float sw = plotW / (float)loopLen;
            for (int i = 0; i < loopLen; ++i)
            {
                float x = plotX0 + i * sw;
                float cw = std::max(sw - (loopLen > 32 ? 0.6f : 1.4f), 1.f);
                bool on = stepGate[i] > 0.f;
                nvgBeginPath(vg);
                nvgRect(vg, x, PAT_Y, cw, PAT_H);
                nvgFillColor(vg, on ? nvgRGBA(0x3d, 0xe0, 0xff, 170) : nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
            }
            if (play && readPos >= 0 && readPos < loopLen)
            {
                // readPos points at the NEXT step to play, so mark the one before it
                int cur = (readPos + loopLen - 1) % loopLen;
                nvgBeginPath(vg);
                nvgRect(vg, plotX0 + cur * sw - 0.5f, PAT_Y - 2.f, std::max(sw, 2.f) + 0.5f, PAT_H + 4.f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 220));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            if (rec && writePos >= 0 && writePos < loopLen)
            {
                nvgBeginPath(vg);
                nvgRect(vg, plotX0 + writePos * sw, PAT_Y + PAT_H + 2.f, std::max(sw - 1.f, 2.f), 2.f);
                nvgFillColor(vg, agmm::violet());
                nvgFill(vg);
            }

            // ---- chip row: REC / PLAY / TRAIN, loop readout on the right ----
            float cx = 8.f;
            const char *chipTxt[3] = {"REC", "PLAY", "TRAIN"};
            bool chipOn[3] = {rec, play, train};
            NVGcolor chipCol[3] = {agmm::violet(), agmm::accent(), agmm::accent()};
            for (int i = 0; i < 3; ++i)
            {
                float cw = agmm::textWidth(vg, chipTxt[i], 7.f) + 10.f;
                agmm::chip(vg, cx, CHIP_Y, cw, CHIP_H, chipTxt[i], chipOn[i], chipCol[i]);
                cx += cw + 4.f;
            }
            snprintf(buf, sizeof(buf), "CH1 LOOP %d DIV %d", loopLen, div);
            agmm::text(vg, W - 8.f, CHIP_Y + CHIP_H / 2.f + 0.5f, buf, 7.f, agmm::dim(), agmm::ALIGN_R);

            // ---- probability meter row ----
            const float mx = 36.f, mw = W - 36.f - 40.f;
            agmm::text(vg, 8.f, METER_Y, "PROB", 7.f, agmm::dim(), agmm::ALIGN_L);
            nvgBeginPath(vg);
            nvgRect(vg, mx, METER_Y - 2.5f, mw, 5.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, mx, METER_Y - 2.5f, std::max(mw * clamp(prob, 0.f, 1.f), 1.f), 5.f);
            nvgFillColor(vg, agmm::accent());
            nvgFill(vg);
            snprintf(buf, sizeof(buf), "%.0f%%", prob * 100.f);
            agmm::text(vg, W - 8.f, METER_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_R);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Widget: adds the background, screws, live display, the left knob block,
// the right output/transport block, the input row, the 16 gate LEDs and the
// context menu (gate time, random mode, range, clock division, scale, seed).
// ------------------------------------------------------------
struct DiceOnSteroidsWidget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, display, the 8 control
    // knobs (row-major), the 4 output jacks, the 3 transport switches + seed
    // input, the 10 input jacks, and the 16 gate LEDs.
    DiceOnSteroidsWidget(DiceOnSteroids *module)
    {
        setModule(module);
        box.size = Vec(DS_PANEL_W, DS_PANEL_H);

        addChild(new DSPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new DSDisplayWidget(module));

        // ---- left block: knobs (row-major, same order as the labels) ----
        static const int knobIds[8] = {
            DiceOnSteroids::PARAM_POLYPHONY, DiceOnSteroids::PARAM_PROBABILITY,
            DiceOnSteroids::PARAM_GLIDE_TIME, DiceOnSteroids::PARAM_ROOT_SHIFT,
            DiceOnSteroids::PARAM_CLOCK_DIV, DiceOnSteroids::PARAM_PROB_SPREAD,
            DiceOnSteroids::PARAM_PROB_SMOOTH, DiceOnSteroids::PARAM_LOOP_LENGTH};
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 2; ++c)
                addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DS_LCOL[c], dsLKnobY(r)), module, knobIds[r * 2 + c]));

        agmm::addKnobArcs(this, 6.f);
                // ---- right block: outputs ----
        const float r0 = DS_RCOL[0], r1 = DS_RCOL[1];
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, DS_ROUT_TOP[0] + 28.f), module, DiceOnSteroids::OUTPUT_GATES));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, DS_ROUT_TOP[0] + 28.f), module, DiceOnSteroids::OUTPUT_VOCT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, DS_ROUT_TOP[1] + 28.f), module, DiceOnSteroids::OUTPUT_QUANTIZED));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, DS_ROUT_TOP[1] + 28.f), module, DiceOnSteroids::OUTPUT_TRIGGER_NEW));

        // ---- right block: transport ----
        addParam(createParamCentered<CKSS>(Vec(r0, DS_RSW_TOP[0] + 34.f), module, DiceOnSteroids::PARAM_RECORD_ENABLE));
        addParam(createParamCentered<CKSS>(Vec(r1, DS_RSW_TOP[0] + 34.f), module, DiceOnSteroids::PARAM_PLAYBACK_ENABLE));
        addParam(createParamCentered<CKSS>(Vec(r0, DS_RSW_TOP[1] + 34.f), module, DiceOnSteroids::PARAM_TRAIN_MODE));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(r1, DS_RSW_TOP[1] + 32.f), module, DiceOnSteroids::INPUT_RANDOM_SEED));

        // ---- input row ----
        const int inIds[10] = {
            DiceOnSteroids::INPUT_CLOCK,
            DiceOnSteroids::INPUT_RESET,
            DiceOnSteroids::INPUT_RETRIGGER,
            DiceOnSteroids::INPUT_CV_PROB,
            DiceOnSteroids::INPUT_CV_POLY,
            DiceOnSteroids::INPUT_CV_GATE_TIME,
            DiceOnSteroids::INPUT_CV_RANGE,
            DiceOnSteroids::INPUT_CV_SCALE,
            DiceOnSteroids::INPUT_CV_TRANSPOSE,
            DiceOnSteroids::INPUT_CV_GLIDE};
        for (int i = 0; i < 10; ++i)
            addInput(createInputCentered<DarkPJ301MPort>(Vec(dsInX(i), DS_IN_Y), module, inIds[i]));

        // ---- 16 gate LEDs, aligned with the display columns ----
        for (int i = 0; i < 16; ++i)
            addChild(createLightCentered<SmallLight<agmm::CyanLight>>(
                Vec(DS_DISP_X + DS_CH_X0 + (i + 0.5f) * DS_CH_W, DS_LED_Y),
                module, DiceOnSteroids::LIGHT_GATES + i));
    }

    // Adds the right-click menu: "Gate time", "Random mode", "V/Oct random
    // range", "Clock division", "Quantization scale" submenus and a
    // "Randomize seed" action, each with checkmarks on the active option.
    void appendContextMenu(Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        DiceOnSteroids *module = dynamic_cast<DiceOnSteroids *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);

        menu->addChild(createSubmenuItem("Gate time", "",
                                         [=](Menu *sub)
                                         {
                                             static const float T[7] = {0.005f, 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f};
                                             static const char *N[7] = {"5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms", "500 ms"};
                                             for (int i = 0; i < 7; ++i)
                                                 sub->addChild(createMenuItem(N[i], CHECKMARK(std::fabs(module->gateTime - T[i]) < 1e-6f),
                                                                              [=]()
                                                                              { module->gateTime = T[i]; }));
                                         }));

        menu->addChild(createSubmenuItem("Random mode", "",
                                         [=](Menu *sub)
                                         {
                                             static const char *N[4] = {"Uniform", "Random walk", "Drunk walk", "Gaussian"};
                                             for (int i = 0; i < 4; ++i)
                                                 sub->addChild(createMenuItem(N[i], CHECKMARK(module->randomMode == i),
                                                                              [=]()
                                                                              { module->randomMode = i; }));
                                         }));

        menu->addChild(createSubmenuItem("V/Oct random range", "",
                                         [=](Menu *sub)
                                         {
                                             static const char *N[4] = {"0 to +1 V", "-1 to +1 V", "-2 to +2 V", "-5 to +5 V"};
                                             for (int i = 0; i < 4; ++i)
                                                 sub->addChild(createMenuItem(N[i], CHECKMARK(module->randomRange == i),
                                                                              [=]()
                                                                              { module->randomRange = i; }));
                                         }));

        menu->addChild(createSubmenuItem("Clock division (1..16)", "",
                                         [=](Menu *sub)
                                         {
                                             for (int d = 1; d <= 16; ++d)
                                                 sub->addChild(createMenuItem(string::f("%d", d), CHECKMARK(module->clockDiv == d),
                                                                              [=]()
                                                                              { module->params[DiceOnSteroids::PARAM_CLOCK_DIV].setValue((float)d); }));
                                         }));

        menu->addChild(createSubmenuItem("Quantization scale", "",
                                         [=](Menu *sub)
                                         {
                                             const auto &sc = DiceOnSteroids::getScales();
                                             for (int i = 0; i < (int)sc.size(); ++i)
                                                 sub->addChild(createMenuItem(sc[i].name, CHECKMARK(module->selectedScale == i),
                                                                              [=]()
                                                                              { module->selectedScale = i; }));
                                         }));

        menu->addChild(createMenuItem("Randomize seed", "",
                                      [=]()
                                      {
                                          module->randomSeed = std::chrono::steady_clock::now().time_since_epoch().count();
                                          module->rng.seed(module->randomSeed);
                                          module->useFixedSeed = false;
                                      }));
    }
};

Model *modelDiceOnSteroids = createModel<DiceOnSteroids, DiceOnSteroidsWidget>("DiceOnSteroids");