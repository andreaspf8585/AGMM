#include "plugin.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <random>
#include <chrono>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float DS_PANEL_W = RACK_GRID_WIDTH * 28; // 420
static const float DS_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display: 16 channel columns, pattern strip, probability meter
static const float DS_DISP_X = 100.f;
static const float DS_DISP_Y = 46.f;
static const float DS_DISP_W = 220.f;
static const float DS_DISP_H = 170.f;
static const float DS_CH_X0 = 10.f; // first channel column inside the display
static const float DS_CH_W = 12.5f; // 16 x 12.5 = 200

// Left control block (2 columns x 4 rows)
static const float DS_LCOL[2] = {28.f, 68.f};
static const float DS_LROW[4] = {90.f, 138.f, 186.f, 234.f};

// Right block: outputs, transport switches
static const float DS_RCOL[2] = {352.f, 392.f};
static const float DS_ROUT[2] = {100.f, 150.f};
static const float DS_RSW[2] = {208.f, 258.f};

// LED strip under the display (aligned with the channel columns)
static const float DS_LED_Y = 241.f;

// Input row (10 jacks)
static const float DS_IN_Y = 340.f;
static float dsInX(int i) { return 30.f + 40.f * i; }

// ------------------------------------------------------------
// Scale structure
// ------------------------------------------------------------
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
    static const std::vector<Scale> &getScales() { return scales; }

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

    float randFloat()
    {
        return std::uniform_real_distribution<float>(0, 1)(rng);
    }

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

    void recordStep(int channel, float gate, float pitch, float quant)
    {
        if (!recordEnabled)
            return;
        pattern[channel][patternWritePos[channel]] = {gate, pitch, quant};
        patternWritePos[channel] = (patternWritePos[channel] + 1) % loopLength;
    }

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

    void dataFromJson(json_t *root) override
    {
        json_t *gt = json_object_get(root, "gateTime");
        if (gt)
            gateTime = json_real_value(gt);
        json_t *rr = json_object_get(root, "randomRange");
        if (rr)
            randomRange = json_integer_value(rr);
        json_t *ss = json_object_get(root, "selectedScale");
        if (ss)
            selectedScale = json_integer_value(ss);
        json_t *glide = json_object_get(root, "glideTime");
        if (glide)
            glideTime = json_real_value(glide);
        json_t *rm = json_object_get(root, "randomMode");
        if (rm)
            randomMode = json_integer_value(rm);
        json_t *cd = json_object_get(root, "clockDiv");
        if (cd)
            clockDiv = json_integer_value(cd);
        json_t *ufs = json_object_get(root, "useFixedSeed");
        if (ufs)
            useFixedSeed = json_is_true(ufs);
        json_t *seed = json_object_get(root, "randomSeed");
        if (seed)
        {
            randomSeed = json_integer_value(seed);
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
                        if (stepObj)
                        {
                            json_t *g = json_object_get(stepObj, "gate");
                            json_t *p = json_object_get(stepObj, "pitch");
                            json_t *q = json_object_get(stepObj, "quant");
                            pattern[ch][i].gate = g ? json_real_value(g) : 0.f;
                            pattern[ch][i].pitch = p ? json_real_value(p) : 0.f;
                            pattern[ch][i].quantized = q ? json_real_value(q) : 0.f;
                        }
                    }
                }
            }
        }
    }

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

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{

    // ------------------------------------------------------------
    // Theme (same palette as the other AGMM modules)
    // ------------------------------------------------------------
    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    NVGcolor colDim() { return nvgRGB(0x7a, 0x85, 0x94); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    NVGcolor colViolet() { return nvgRGB(0xb0, 0x5a, 0xff); }

    NVGcolor withAlpha(NVGcolor c, float a)
    {
        c.a = a;
        return c;
    }

    void setFont(NVGcontext *vg)
    {
        std::shared_ptr<window::Font> font =
            APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (font)
            nvgFontFaceId(vg, font->handle);
    }

    void drawText(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
                  int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f)
    {
        setFont(vg);
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Cyan LEDs (Rack has no built-in ones)
    template <typename TBase = GrayModuleLightWidget>
    struct TDSCyanLight : TBase
    {
        TDSCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
    };
    using DSCyanLight = TDSCyanLight<>;

    // ------------------------------------------------------------
    // AGMM logo
    // ------------------------------------------------------------
    void drawLogo(NVGcontext *vg, float cx, float cy)
    {
        const float w = 64.f, h = 18.f;
        float x = cx - w / 2.f, y = cy - h / 2.f;

        // Glow
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
        nvgFill(vg);

        // Pill body
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 7.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                           nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
        nvgFill(vg);
        nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
                                             nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgStrokeWidth(vg, 1.2f);
        nvgStroke(vg);

        // Tiny cell triangle
        const float s = 2.2f;
        float tx = x + 7.f, ty = y + h / 2.f - 3.f;
        nvgBeginPath(vg);
        nvgRect(vg, tx + s, ty, s, s);
        nvgRect(vg, tx, ty + s * 1.4f, s, s);
        nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
        nvgFillColor(vg, colAccent());
        nvgFill(vg);

        // Wordmark
        drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct DSPanelBackground : TransparentWidget
    {
        DSPanelBackground() { box.size = Vec(DS_PANEL_W, DS_PANEL_H); }

        void label(NVGcontext *vg, float x, float y, const char *txt)
        {
            drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        void plate(NVGcontext *vg, float x, float y, float w, float h)
        {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, y, w, h, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 80));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        void ring(NVGcontext *vg, float cx, float cy, float r)
        {
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, r);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Caption with an underline (side blocks)
        void caption(NVGcontext *vg, float cx, float y, const char *txt, NVGcolor col, float x1, float x2)
        {
            drawText(vg, cx, y, txt, 8.f, col, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x1, y + 8.f);
            nvgLineTo(vg, x2, y + 8.f);
            nvgStrokeColor(vg, withAlpha(col, 0.35f));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Full-width divider with a centered caption
        void section(NVGcontext *vg, float y, const char *txt)
        {
            float x1 = 14.f, x2 = DS_PANEL_W - 14.f, cx = DS_PANEL_W / 2.f;
            float w = 10.f + (float)strlen(txt) * 7.f;
            drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x1, y);
            nvgLineTo(vg, cx - w / 2.f, y);
            nvgMoveTo(vg, cx + w / 2.f, y);
            nvgLineTo(vg, x2, y);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Output / input cell: label on top, jack below
        void topCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
            label(vg, cx, cy - 19.f, txt);
        }
        // Control in the middle, label below
        void jackCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 17.f, 34.f, 34.f);
            label(vg, cx, cy + 25.f, txt);
        }
        void knobCell(NVGcontext *vg, float cx, float cy, float r, const char *txt)
        {
            ring(vg, cx, cy, r);
            label(vg, cx, cy + 25.f, txt);
        }
        void switchCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 20.f, 34.f, 40.f);
            label(vg, cx, cy + 25.f, txt);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            // Base gradient
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, DS_PANEL_W, DS_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DS_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            // Brushed texture
            nvgStrokeWidth(vg, 1.f);
            for (int y = 0; y < (int)DS_PANEL_H; y += 3)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, y + 0.5f);
                nvgLineTo(vg, DS_PANEL_W, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
                nvgStroke(vg);
            }

            // Edge highlight
            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, DS_PANEL_W - 1.f, DS_PANEL_H - 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Logo + title
            drawLogo(vg, DS_PANEL_W / 2.f, 13.f);
            drawText(vg, DS_PANEL_W / 2.f, 31.f, "DICE ON STEROIDS", 13.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

            // Display bezel
            float bx = DS_DISP_X - 5.f, by = DS_DISP_Y - 5.f;
            float bw = DS_DISP_W + 10.f, bh = DS_DISP_H + 10.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                               nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
            nvgFill(vg);

            // Block captions
            caption(vg, 48.f, 54.f, "CONTROL", colAccent(), 12.f, 84.f);
            caption(vg, 372.f, 54.f, "OUTPUT", colViolet(), 335.f, 409.f);
            caption(vg, 372.f, 176.f, "TRANSPORT", colAccent(), 335.f, 409.f);

            // ---- left block: 8 knobs ----
            const float l0 = DS_LCOL[0], l1 = DS_LCOL[1];
            knobCell(vg, l0, DS_LROW[0], 18.f, "POLY");
            knobCell(vg, l1, DS_LROW[0], 18.f, "PROB");
            knobCell(vg, l0, DS_LROW[1], 18.f, "GLIDE");
            knobCell(vg, l1, DS_LROW[1], 18.f, "ROOT");
            knobCell(vg, l0, DS_LROW[2], 18.f, "CLK DIV");
            knobCell(vg, l1, DS_LROW[2], 18.f, "SPREAD");
            knobCell(vg, l0, DS_LROW[3], 18.f, "SMOOTH");
            knobCell(vg, l1, DS_LROW[3], 18.f, "LOOP LEN");

            // ---- right block ----
            const float r0 = DS_RCOL[0], r1 = DS_RCOL[1];
            topCell(vg, r0, DS_ROUT[0], "GATES");
            topCell(vg, r1, DS_ROUT[0], "RANDOM");
            topCell(vg, r0, DS_ROUT[1], "QUANT");
            topCell(vg, r1, DS_ROUT[1], "TRIG");
            switchCell(vg, r0, DS_RSW[0], "REC");
            switchCell(vg, r1, DS_RSW[0], "PLAY");
            switchCell(vg, r0, DS_RSW[1], "TRAIN");
            jackCell(vg, r1, DS_RSW[1], "SEED");

            // ---- LED plate under the display ----
            plate(vg, DS_DISP_X, 229.f, DS_DISP_W, 36.f);
            for (int i = 0; i < 16; ++i)
            {
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", i + 1);
                drawText(vg, DS_DISP_X + DS_CH_X0 + (i + 0.5f) * DS_CH_W, 256.f, buf, 6.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
            }

            // ---- input row ----
            section(vg, 296.f, "INPUTS");
            const char *in[10] = {"CLK", "RESET", "RETRIG", "CV PROB", "CV POLY",
                                  "CV GATE", "CV RANGE", "CV SCALE", "CV TRANS", "CV GLIDE"};
            for (int i = 0; i < 10; ++i)
                topCell(vg, dsInX(i), DS_IN_Y, in[i]);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: channel columns, pattern strip, probability meter
    // ------------------------------------------------------------
    struct DSDisplayWidget : TransparentWidget
    {
        DiceOnSteroids *module = nullptr;

        DSDisplayWidget(DiceOnSteroids *m) : module(m)
        {
            box.pos = Vec(DS_DISP_X, DS_DISP_Y);
            box.size = Vec(DS_DISP_W, DS_DISP_H);
        }

        static constexpr float CH_Y0 = 32.f; // channel area top
        static constexpr float CH_H = 90.f;  // channel area height

        // Static background: screen, grid, channel tracks
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = DS_DISP_W, H = DS_DISP_H;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H);
            nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 30.f, W * 0.75f,
                                               nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
            nvgFill(vg);

            // Channel tracks
            const float mid = CH_Y0 + CH_H / 2.f;
            for (int c = 0; c < 16; ++c)
            {
                float x = DS_CH_X0 + c * DS_CH_W;
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x + 1.5f, CH_Y0, DS_CH_W - 3.f, CH_H, 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 9));
                nvgFill(vg);
            }

            // Voltage grid every 2.5 V (+-5 V range) and zero line
            nvgBeginPath(vg);
            for (int k = -2; k <= 2; ++k)
            {
                float y = mid - k * (CH_H / 2.f) / 2.f + 0.5f;
                nvgMoveTo(vg, DS_CH_X0, y);
                nvgLineTo(vg, DS_CH_X0 + 16.f * DS_CH_W, y);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, DS_CH_X0, mid + 0.5f);
            nvgLineTo(vg, DS_CH_X0 + 16.f * DS_CH_W, mid + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 45));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

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
            const float W = DS_DISP_W, H = DS_DISP_H;
            const float mid = CH_Y0 + CH_H / 2.f;
            const float amp = CH_H / 2.f - 2.f; // pixels for 5 V

            // ---- gather state (live, or a sample scene in the module browser) ----
            int poly, loopLen, readPos = 0, writePos = 0, scaleIdx, mode, range, div;
            float quant[16], raw[16], fl[16], gateMs, rootSemi, prob;
            float stepGate[DiceOnSteroids::MAX_PATTERN_LEN];
            bool rec, play, train;

            for (int c = 0; c < 16; ++c)
            {
                quant[c] = raw[c] = fl[c] = 0.f;
            }
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
                loopLen = std::min(std::max(module->loopLength, 1), DiceOnSteroids::MAX_PATTERN_LEN);
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
            poly = std::min(std::max(poly, 1), 16);
            scaleIdx = std::min(std::max(scaleIdx, 0), (int)DiceOnSteroids::scales.size() - 1);
            mode = std::min(std::max(mode, 0), 3);
            range = std::min(std::max(range, 0), 3);

            // ---- channel bars ----
            for (int c = 0; c < 16; ++c)
            {
                float x = DS_CH_X0 + c * DS_CH_W + 1.5f;
                float w = DS_CH_W - 3.f;
                if (c >= poly)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 2.f, mid, w - 4.f, 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 22));
                    nvgFill(vg);
                    continue;
                }
                float f = fl[c];

                // Random (unquantized) pitch tick
                float yr = mid - std::min(std::max(raw[c] / 5.f, -1.f), 1.f) * amp;
                nvgBeginPath(vg);
                nvgRect(vg, x - 0.5f, yr, w + 1.f, 1.f);
                nvgFillColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 150));
                nvgFill(vg);

                // Quantized pitch bar
                float yq = mid - std::min(std::max(quant[c] / 5.f, -1.f), 1.f) * amp;
                float top = std::min(mid, yq);
                float h = std::max(std::fabs(mid - yq), 1.5f);

                if (f > 0.02f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x - 1.5f, top - 1.5f, w + 3.f, h + 3.f);
                    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (int)(50.f * f)));
                    nvgFill(vg);
                }
                nvgBeginPath(vg);
                nvgRect(vg, x, top, w, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, mid + amp, 0, mid - amp,
                                                   nvgRGBA(0x1f, 0x9f, 0xc0, (int)(110 + 120 * f)),
                                                   nvgRGBA(0x8a, 0xee, 0xff, (int)(110 + 145 * f))));
                nvgFill(vg);

                // Cap at the pitch
                nvgBeginPath(vg);
                nvgRect(vg, x, yq - 1.f, w, 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(110 + 145 * f)));
                nvgFill(vg);
            }

            // ---- pattern strip (channel 1) ----
            const float py = 130.f, ph = 8.f;
            const float sw = (16.f * DS_CH_W) / (float)loopLen;
            for (int i = 0; i < loopLen; ++i)
            {
                float x = DS_CH_X0 + i * sw;
                float cw = std::max(sw - (loopLen > 32 ? 0.6f : 1.4f), 1.f);
                bool on = stepGate[i] > 0.f;
                nvgBeginPath(vg);
                nvgRect(vg, x, py, cw, ph);
                nvgFillColor(vg, on ? nvgRGBA(0x3d, 0xe0, 0xff, 170) : nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
            }
            if (play && readPos >= 0 && readPos < loopLen)
            {
                // readPos points at the NEXT step to play, so mark the one before it
                int cur = (readPos + loopLen - 1) % loopLen;
                nvgBeginPath(vg);
                nvgRect(vg, DS_CH_X0 + cur * sw - 0.5f, py - 2.f, std::max(sw, 2.f) + 0.5f, ph + 4.f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 220));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            if (rec && writePos >= 0 && writePos < loopLen)
            {
                nvgBeginPath(vg);
                nvgRect(vg, DS_CH_X0 + writePos * sw, py + ph + 1.f, std::max(sw - 1.f, 2.f), 2.f);
                nvgFillColor(vg, nvgRGB(0xb0, 0x5a, 0xff));
                nvgFill(vg);
            }

            // ---- status chips ----
            auto chip = [&](float x, float w, const char *txt, bool on, NVGcolor col)
            {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, 144.f, w, 11.f, 3.f);
                if (on)
                {
                    nvgFillColor(vg, withAlpha(col, 0.25f));
                    nvgFill(vg);
                    nvgStrokeColor(vg, withAlpha(col, 0.9f));
                }
                else
                {
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                }
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                drawText(vg, x + w / 2.f, 149.5f, txt, 6.f, on ? col : colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
            };
            chip(10.f, 26.f, "REC", rec, colViolet());
            chip(40.f, 28.f, "PLAY", play, colAccent());
            chip(72.f, 32.f, "TRAIN", train, colAccent());

            char buf[48];
            snprintf(buf, sizeof(buf), "LOOP %d  DIV %d", loopLen, div);
            drawText(vg, W - 10.f, 149.5f, buf, 7.f, colDim(), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            // ---- probability meter ----
            const float mx = 36.f, mw = W - 36.f - 44.f, my = 159.f;
            drawText(vg, 10.f, my + 1.f, "PROB", 6.f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, mx, my - 2.f, mw, 5.f, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, mx, my - 2.f, std::max(mw * prob, 2.f), 5.f, 2.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, mx, 0, mx + mw, 0, colAccent(), colViolet()));
            nvgFill(vg);
            snprintf(buf, sizeof(buf), "%3.0f%%", prob * 100.f);
            drawText(vg, W - 10.f, my + 1.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 200),
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            // ---- scanlines ----
            nvgBeginPath(vg);
            for (int y = 0; y < (int)H; y += 3)
                nvgRect(vg, 0, y + 1, W, 1);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
            nvgFill(vg);

            // ---- glass reflection ----
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H * 0.3f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f,
                                               nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
            nvgFill(vg);

            // ---- readouts (top) ----
            static const char *modeNames[4] = {"UNIFORM", "WALK", "DRUNK", "GAUSS"};
            static const char *rangeNames[4] = {"0/+1V", "-1/+1V", "-2/+2V", "-5/+5V"};
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
            NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 230);

            drawText(vg, 10.f, 9.f, DiceOnSteroids::scales[scaleIdx].name.c_str(), 7.f, c1,
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            drawText(vg, W - 10.f, 9.f, modeNames[mode], 7.f, c1,
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            snprintf(buf, sizeof(buf), "ROOT %+.1f", rootSemi);
            drawText(vg, 10.f, 21.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "RNG %s", rangeNames[range]);
            drawText(vg, W / 2.f, 21.f, buf, 7.f, cd, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "GATE %.0fMS", gateMs);
            drawText(vg, W - 10.f, 21.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Widget
// ------------------------------------------------------------
struct DiceOnSteroidsWidget : ModuleWidget
{
    DiceOnSteroidsWidget(DiceOnSteroids *module)
    {
        setModule(module);
        box.size = Vec(DS_PANEL_W, DS_PANEL_H);

        addChild(new DSPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new DSDisplayWidget(module));

        // ---- left block: knobs ----
        const float l0 = DS_LCOL[0], l1 = DS_LCOL[1];
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, DS_LROW[0]), module, DiceOnSteroids::PARAM_POLYPHONY));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l1, DS_LROW[0]), module, DiceOnSteroids::PARAM_PROBABILITY));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, DS_LROW[1]), module, DiceOnSteroids::PARAM_GLIDE_TIME));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l1, DS_LROW[1]), module, DiceOnSteroids::PARAM_ROOT_SHIFT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, DS_LROW[2]), module, DiceOnSteroids::PARAM_CLOCK_DIV));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l1, DS_LROW[2]), module, DiceOnSteroids::PARAM_PROB_SPREAD));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, DS_LROW[3]), module, DiceOnSteroids::PARAM_PROB_SMOOTH));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l1, DS_LROW[3]), module, DiceOnSteroids::PARAM_LOOP_LENGTH));

        // ---- right block ----
        const float r0 = DS_RCOL[0], r1 = DS_RCOL[1];
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, DS_ROUT[0]), module, DiceOnSteroids::OUTPUT_GATES));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, DS_ROUT[0]), module, DiceOnSteroids::OUTPUT_VOCT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, DS_ROUT[1]), module, DiceOnSteroids::OUTPUT_QUANTIZED));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, DS_ROUT[1]), module, DiceOnSteroids::OUTPUT_TRIGGER_NEW));

        addParam(createParamCentered<CKSS>(Vec(r0, DS_RSW[0]), module, DiceOnSteroids::PARAM_RECORD_ENABLE));
        addParam(createParamCentered<CKSS>(Vec(r1, DS_RSW[0]), module, DiceOnSteroids::PARAM_PLAYBACK_ENABLE));
        addParam(createParamCentered<CKSS>(Vec(r0, DS_RSW[1]), module, DiceOnSteroids::PARAM_TRAIN_MODE));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(r1, DS_RSW[1]), module, DiceOnSteroids::INPUT_RANDOM_SEED));

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
            addChild(createLightCentered<SmallLight<DSCyanLight>>(
                Vec(DS_DISP_X + DS_CH_X0 + (i + 0.5f) * DS_CH_W, DS_LED_Y),
                module, DiceOnSteroids::LIGHT_GATES + i));
    }

    void appendContextMenu(Menu *menu) override
    {
        DiceOnSteroids *module = dynamic_cast<DiceOnSteroids *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);

        // Gate Time submenu
        struct GateTimeMenu : MenuItem
        {
            DiceOnSteroids *module;
            Menu *createChildMenu() override
            {
                Menu *m = new Menu;
                std::vector<float> times = {0.005f, 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f};
                std::vector<std::string> names = {"5ms", "10ms", "20ms", "50ms", "100ms", "200ms", "500ms"};
                for (size_t i = 0; i < times.size(); ++i)
                {
                    struct GateTimeItem : MenuItem
                    {
                        DiceOnSteroids *module;
                        float time;
                        void onAction(const ActionEvent &e) override
                        {
                            if (module)
                                module->gateTime = time;
                        }
                        void step() override
                        {
                            if (!module)
                            {
                                rightText = "";
                                MenuItem::step();
                                return;
                            }
                            rightText = (module->gateTime == time) ? "✔" : "";
                            MenuItem::step();
                        }
                    };
                    auto item = new GateTimeItem;
                    item->text = names[i];
                    item->module = module;
                    item->time = times[i];
                    m->addChild(item);
                }
                return m;
            }
        };
        auto gateItem = new GateTimeMenu;
        gateItem->text = "Gate Time";
        gateItem->module = module;
        menu->addChild(gateItem);

        // Random Mode submenu
        struct RandomModeMenu : MenuItem
        {
            DiceOnSteroids *module;
            Menu *createChildMenu() override
            {
                Menu *m = new Menu;
                struct RandomModeItem : MenuItem
                {
                    DiceOnSteroids *module;
                    int mode;
                    void onAction(const ActionEvent &e) override
                    {
                        if (module)
                            module->randomMode = mode;
                    }
                    void step() override
                    {
                        if (!module)
                        {
                            rightText = "";
                            MenuItem::step();
                            return;
                        }
                        rightText = (module->randomMode == mode) ? "✔" : "";
                        MenuItem::step();
                    }
                };
                const std::vector<std::pair<int, std::string>> modes = {
                    {0, "Uniform"}, {1, "Random Walk"}, {2, "Drunk Walk"}, {3, "Gaussian"}};
                for (const auto &p : modes)
                {
                    auto item = new RandomModeItem;
                    item->text = p.second;
                    item->module = module;
                    item->mode = p.first;
                    m->addChild(item);
                }
                return m;
            }
        };
        auto randItem = new RandomModeMenu;
        randItem->text = "Random Mode";
        randItem->module = module;
        menu->addChild(randItem);

        // V/Oct Range submenu
        struct RangeMenu : MenuItem
        {
            DiceOnSteroids *module;
            Menu *createChildMenu() override
            {
                Menu *m = new Menu;
                struct RangeItem : MenuItem
                {
                    DiceOnSteroids *module;
                    int range;
                    void onAction(const ActionEvent &e) override
                    {
                        if (module)
                            module->randomRange = range;
                    }
                    void step() override
                    {
                        if (!module)
                        {
                            rightText = "";
                            MenuItem::step();
                            return;
                        }
                        rightText = (module->randomRange == range) ? "✔" : "";
                        MenuItem::step();
                    }
                };
                const std::vector<std::pair<int, std::string>> ranges = {
                    {0, "0 to +1V"},
                    {1, "-1 to +1V"},
                    {2, "-2 to +2V"},
                    {3, "-5 to +5V"}};
                for (const auto &p : ranges)
                {
                    auto item = new RangeItem;
                    item->text = p.second;
                    item->module = module;
                    item->range = p.first;
                    m->addChild(item);
                }
                return m;
            }
        };
        auto rangeItem = new RangeMenu;
        rangeItem->text = "V/Oct Random Range";
        rangeItem->module = module;
        menu->addChild(rangeItem);

        // Clock Division submenu (sets the CLK DIV knob)
        struct ClockDivMenu : MenuItem
        {
            DiceOnSteroids *module;
            Menu *createChildMenu() override
            {
                Menu *m = new Menu;
                struct ClockDivItem : MenuItem
                {
                    DiceOnSteroids *module;
                    int div;
                    void onAction(const ActionEvent &e) override
                    {
                        if (module)
                            module->params[DiceOnSteroids::PARAM_CLOCK_DIV].setValue((float)div);
                    }
                    void step() override
                    {
                        if (!module)
                        {
                            rightText = "";
                            MenuItem::step();
                            return;
                        }
                        rightText = (module->clockDiv == div) ? "✔" : "";
                        MenuItem::step();
                    }
                };
                for (int d = 1; d <= 16; ++d)
                {
                    auto item = new ClockDivItem;
                    item->text = string::f("%d", d);
                    item->module = module;
                    item->div = d;
                    m->addChild(item);
                }
                return m;
            }
        };
        auto clockItem = new ClockDivMenu;
        clockItem->text = "Clock Division (1..16)";
        clockItem->module = module;
        menu->addChild(clockItem);

        // Scale selection submenu
        struct ScaleMenu : MenuItem
        {
            DiceOnSteroids *module;
            Menu *createChildMenu() override
            {
                Menu *m = new Menu;
                const auto &scales = DiceOnSteroids::getScales();
                struct ScaleMenuItem : MenuItem
                {
                    DiceOnSteroids *module;
                    int index;
                    void onAction(const ActionEvent &e) override
                    {
                        if (module)
                            module->selectedScale = index;
                    }
                    void step() override
                    {
                        if (!module)
                        {
                            rightText = "";
                            MenuItem::step();
                            return;
                        }
                        rightText = (module->selectedScale == index) ? "✔" : "";
                        MenuItem::step();
                    }
                };
                for (size_t i = 0; i < scales.size(); ++i)
                {
                    auto item = new ScaleMenuItem;
                    item->text = scales[i].name;
                    item->module = module;
                    item->index = (int)i;
                    m->addChild(item);
                }
                return m;
            }
        };
        auto scaleItem = new ScaleMenu;
        scaleItem->text = "Quantization Scale";
        scaleItem->module = module;
        menu->addChild(scaleItem);

        // Randomize Seed
        struct RandomizeSeedItem : MenuItem
        {
            DiceOnSteroids *module;
            void onAction(const ActionEvent &e) override
            {
                if (module)
                {
                    module->randomSeed = std::chrono::steady_clock::now().time_since_epoch().count();
                    module->rng.seed(module->randomSeed);
                    module->useFixedSeed = false;
                }
            }
        };
        auto seedItem = new RandomizeSeedItem;
        seedItem->text = "Randomize Seed";
        seedItem->module = module;
        menu->addChild(seedItem);
    }
};

Model *modelDiceOnSteroids = createModel<DiceOnSteroids, DiceOnSteroidsWidget>("DiceOnSteroids");