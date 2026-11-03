#include "plugin.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float EA_W = RACK_GRID_WIDTH * 28;   // 420
static const float EA_H = RACK_GRID_HEIGHT;       // 380

// Display
static const float EA_DX = 75.f, EA_DY = 44.f, EA_DW = 270.f, EA_DH = 150.f;

// Rows below the display
static const float EA_CTRL_SEC_Y = 214.f;
static const float EA_CTRL_Y     = 246.f;
static const float EA_IO_SEC_Y   = 297.f;
static const float EA_IO_Y       = 337.f;
static float eaIoX(int i) { return 39.f + 38.f * (float)i; }

// Display geometry (display-local coordinates)
static const float EA_BX0 = 26.f;     // left edge of step columns
static const float EA_CW  = 15.f;     // column pitch (16 x 15 = 240)
static const float EA_BY0 = 20.f;     // top of bars
static const float EA_BH  = 58.f;     // bar area height
static const float EA_NOTE_Y = 84.5f; // note name row
static const float EA_GATE_Y = 90.f,  EA_GATE_H = 8.f;
static const float EA_LOCK_Y = 101.f, EA_LOCK_H = 8.f;
static const float EA_NUM_Y  = 114.5f;
static const float EA_KB_X0 = 8.f, EA_KB_X1 = 126.f, EA_KB_Y = 121.f;
static const float EA_KB_WH = 18.f, EA_KB_BH = 11.f, EA_KB_BW = 9.f;
static const float EA_CHIP_X = 136.f, EA_CHIP_P = 26.f, EA_CHIP_W = 24.f;
static const float EA_CHIP_Y = 121.f, EA_CHIP_H = 11.f;

static inline int eaMod12(int a) { return ((a % 12) + 12) % 12; }

static const char* const EA_NN[12] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

// ----------------------------------------------------------------------------
// Scale data: names and semitone intervals from the root
// ----------------------------------------------------------------------------
struct ScaleDef
{
    std::string name;
    std::vector<int> intervals; // semitone offsets from root, 0 = root
};

static const std::vector<ScaleDef> &scaleTable()
{
    static const std::vector<ScaleDef> table = {
        // ----- Common scales -----
        {"Major (Ionian)", {0, 2, 4, 5, 7, 9, 11}},
        {"Natural Minor (Aeolian)", {0, 2, 3, 5, 7, 8, 10}},
        {"Harmonic Minor", {0, 2, 3, 5, 7, 8, 11}},
        {"Melodic Minor (ascending)", {0, 2, 3, 5, 7, 9, 11}},
        {"Pentatonic Major", {0, 2, 4, 7, 9}},
        {"Pentatonic Minor", {0, 3, 5, 7, 10}},
        {"Blues", {0, 3, 5, 6, 7, 10}},
        {"Dorian", {0, 2, 3, 5, 7, 9, 10}},
        {"Phrygian", {0, 1, 3, 5, 7, 8, 10}},
        {"Lydian", {0, 2, 4, 6, 7, 9, 11}},
        {"Mixolydian", {0, 2, 4, 5, 7, 9, 10}},
        {"Locrian", {0, 1, 3, 5, 6, 8, 10}},

        // ----- Modes of harmonic minor -----
        {"Harmonic Minor Mode 2 (Locrian #6)", {0, 1, 3, 5, 6, 9, 10}},
        {"Harmonic Minor Mode 3 (Ionian #5)", {0, 2, 4, 5, 8, 9, 11}},
        {"Harmonic Minor Mode 4 (Dorian #11)", {0, 2, 3, 6, 7, 9, 10}},
        {"Harmonic Minor Mode 5 (Phrygian Dominant)", {0, 1, 4, 5, 7, 8, 10}},
        {"Harmonic Minor Mode 6 (Lydian #2)", {0, 3, 4, 6, 7, 9, 11}},
        {"Harmonic Minor Mode 7 (Ultralocrian)", {0, 1, 3, 4, 6, 8, 9}},

        // ----- Modes of melodic minor -----
        {"Melodic Minor Mode 2 (Dorian b2)", {0, 1, 3, 5, 7, 9, 10}},
        {"Melodic Minor Mode 3 (Lydian #5)", {0, 2, 4, 6, 8, 9, 11}},
        {"Melodic Minor Mode 4 (Lydian b7)", {0, 2, 4, 6, 7, 9, 10}},
        {"Melodic Minor Mode 5 (Mixolydian b6)", {0, 2, 4, 5, 7, 8, 10}},
        {"Melodic Minor Mode 6 (Aeolian b5)", {0, 2, 3, 5, 6, 8, 10}},
        {"Melodic Minor Mode 7 (Superlocrian)", {0, 1, 3, 4, 6, 8, 10}},

        // ----- Symmetrical -----
        {"Whole Tone", {0, 2, 4, 6, 8, 10}},
        {"Diminished (Half-Whole)", {0, 1, 3, 4, 6, 7, 9, 10}},
        {"Diminished (Whole-Half)", {0, 2, 3, 5, 6, 8, 9, 11}},
        {"Augmented", {0, 3, 4, 7, 8, 11}},

        // ----- Pentatonic modes -----
        {"Major Pentatonic (second mode)", {0, 2, 5, 7, 10}},
        {"Major Pentatonic (third mode)", {0, 3, 5, 8, 10}},
        {"Major Pentatonic (fourth mode)", {0, 2, 5, 7, 9}},
        {"Major Pentatonic (fifth mode)", {0, 3, 5, 7, 10}},
        {"Minor Pentatonic (second mode)", {0, 2, 4, 7, 9}},
        {"Minor Pentatonic (third mode)", {0, 2, 5, 7, 10}},
        {"Minor Pentatonic (fourth mode)", {0, 3, 5, 8, 10}},
        {"Minor Pentatonic (fifth mode)", {0, 2, 5, 7, 9}},

        // ----- Exotic / ethnic -----
        {"Hungarian Minor", {0, 2, 3, 6, 7, 8, 11}},
        {"Romanian Minor", {0, 2, 3, 6, 7, 9, 10}},
        {"Egyptian", {0, 2, 5, 7, 10}},
        {"Japanese (Hirajoshi)", {0, 2, 3, 7, 8}},
        {"Japanese (Kumoi)", {0, 2, 3, 7, 9}},
        {"Pelog", {0, 1, 3, 7, 8}},
        {"Slendro", {0, 2, 5, 7, 9}},
        {"Spanish (8-Note)", {0, 1, 3, 4, 5, 6, 8, 10}},
        {"Persian", {0, 1, 4, 5, 6, 8, 11}},
        {"Arabian (Double Harmonic)", {0, 1, 4, 5, 7, 8, 11}},
        {"Enigmatic", {0, 1, 4, 6, 8, 10, 11}},
        {"Neapolitan Major", {0, 1, 3, 5, 7, 9, 11}},
        {"Neapolitan Minor", {0, 1, 3, 5, 7, 8, 11}},
        {"Prometheus", {0, 2, 4, 6, 9, 10}},
        {"Prometheus Neapolitan", {0, 1, 4, 6, 9, 10}},
        {"Messiaen's Mode 1 (Whole Tone)", {0, 2, 4, 6, 8, 10}},
        {"Messiaen's Mode 2 (Octatonic)", {0, 1, 3, 4, 6, 7, 9, 10}},
        {"Messiaen's Mode 3", {0, 2, 3, 4, 6, 7, 8, 10, 11}},
        {"Messiaen's Mode 4", {0, 1, 2, 5, 6, 7, 8, 11}},
        {"Messiaen's Mode 5", {0, 1, 5, 6, 7, 11}},

        // ----- Extras -----
        {"Chromatic", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}},
        {"Bebop Dominant", {0, 2, 4, 5, 7, 9, 10, 11}},
        {"Bebop Major", {0, 2, 4, 5, 7, 8, 9, 11}},
        {"Bebop Dorian", {0, 2, 3, 4, 5, 7, 9, 10}},
        {"Harmonic Major", {0, 2, 4, 5, 7, 8, 11}},
        {"Major Locrian", {0, 2, 4, 5, 6, 8, 10}},
        {"Leading Whole Tone", {0, 2, 4, 6, 8, 10, 11}},
        {"Major Blues", {0, 2, 3, 4, 7, 9}},
        {"Hungarian Major", {0, 3, 4, 6, 7, 9, 10}},
        {"Hindu", {0, 2, 4, 5, 7, 8, 10}},
        {"Iwato", {0, 1, 5, 6, 10}},
        {"In Sen", {0, 1, 5, 7, 10}},
        {"Chinese", {0, 4, 6, 7, 11}},
        {"Six-Tone Symmetrical", {0, 1, 4, 5, 8, 9}},
        {"Tritone", {0, 1, 4, 6, 7, 10}},
    };
    return table;
}

// Tooltip helpers: the key knob shows the note, the scale knob shows the scale name
struct EARootQ : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override
    {
        int i = rack::math::clamp((int)std::round(getValue()), 0, 11);
        return std::string(EA_NN[i]) + "4";
    }
};

struct EAScaleQ : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override
    {
        int i = rack::math::clamp((int)std::round(getValue()), 0, (int)scaleTable().size() - 1);
        return scaleTable()[i].name;
    }
};

// ----------------------------------------------------------------------------
// Module
// ----------------------------------------------------------------------------
struct ExpressArpeggiator : Module
{
    enum ParamId
    {
        ROOT_NOTE_PARAM,
        SCALE_PARAM,
        STEPS_PARAM,
        DIRECTION_PARAM,
        MUTATION_PARAM,
        RANDOMIZE_BUTTON,
        OCTAVES_PARAM,
        GATE_LEN_PARAM,
        REST_PARAM,
        GLIDE_PARAM,
        TRANSPOSE_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        CLOCK_INPUT,
        RANDOM_TRIGGER_INPUT,
        RESET_INPUT,
        TRANSPOSE_CV_INPUT,
        MUT_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        VOCT_OUTPUT,
        CYCLE_GATE_OUTPUT,
        GATE_OUTPUT,
        STEP_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        RANDOM_LIGHT,
        LIGHTS_LEN
    };

    // ---- pattern (all 16 steps are stored; STEPS only sets how many play) ----
    int  noteSemi[16] = {};          // semitones relative to C4
    bool locked[16] = {};            // protected from randomize / mutation
    bool muted[16] = {};             // always a rest
    int  numSteps = 4;
    int  currentStep = 0;
    int  dirSign = 1;
    bool playing = false;            // false = next clock plays the first step
    bool curRest = false;
    uint32_t stepCounter = 0;

    int  lastRoot = -1, lastScale = -1, lastSpan = -1;
    uint16_t scaleMask = 0;          // absolute pitch classes in the scale

    // ---- audio state ----
    float targetV = 0.f, outV = 0.f;
    bool  outInit = false;
    float gateRemaining = 0.f;
    float sinceClock = 0.f, clockPeriod = 0.25f;
    bool  clockSeen = false;

    // ---- display data (torn reads are only cosmetic) ----
    float stepGlow[16] = {};
    float eocFlash = 0.f, restFlash = 0.f, mutFlash = 0.f, randFlash = 0.f;
    int   mutIdx = -1;
    int   playPc = -1;
    int   vTranspose = 0;

    dsp::SchmittTrigger clockTrig, randTrig, resetTrig, randBtnTrig;
    dsp::PulseGenerator eocPulse;

    const std::vector<ScaleDef> &scales = scaleTable();

    ExpressArpeggiator()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam<EARootQ>(ROOT_NOTE_PARAM, 0.f, 11.f, 0.f, "Root note");
        paramQuantities[ROOT_NOTE_PARAM]->snapEnabled = true;
        configParam<EAScaleQ>(SCALE_PARAM, 0.f, (float)(scaleTable().size() - 1), 0.f, "Scale");
        paramQuantities[SCALE_PARAM]->snapEnabled = true;
        configParam(STEPS_PARAM, 2.f, 16.f, 4.f, "Steps");
        paramQuantities[STEPS_PARAM]->snapEnabled = true;
        configSwitch(DIRECTION_PARAM, 0.f, 4.f, 0.f, "Direction",
                     {"Forward", "Backward", "Random", "Drunk", "Pendulum"});
        configParam(MUTATION_PARAM, 0.f, 1.f, 0.f, "Mutation probability", "%", 0.f, 100.f);
        configButton(RANDOMIZE_BUTTON, "Randomize pattern");
        configParam(OCTAVES_PARAM, 1.f, 4.f, 1.f, "Octave range", " oct");
        paramQuantities[OCTAVES_PARAM]->snapEnabled = true;
        configParam(GATE_LEN_PARAM, 0.05f, 1.f, 0.5f, "Gate length", "%", 0.f, 100.f);
        configParam(REST_PARAM, 0.f, 1.f, 0.f, "Rest probability", "%", 0.f, 100.f);
        configParam(GLIDE_PARAM, 0.f, 1.f, 0.f, "Glide", "%", 0.f, 100.f);
        configParam(TRANSPOSE_PARAM, -12.f, 12.f, 0.f, "Transpose", " semitones");
        paramQuantities[TRANSPOSE_PARAM]->snapEnabled = true;

        configInput(CLOCK_INPUT, "Clock");
        configInput(RANDOM_TRIGGER_INPUT, "Random trigger");
        configInput(RESET_INPUT, "Reset (next clock plays step 1)");
        configInput(TRANSPOSE_CV_INPUT, "Transpose CV (1 V/oct)");
        configInput(MUT_CV_INPUT, "Mutation CV (0-10 V)");
        configOutput(VOCT_OUTPUT, "V/Oct");
        configOutput(CYCLE_GATE_OUTPUT, "End of cycle");
        configOutput(GATE_OUTPUT, "Gate");
        configOutput(STEP_OUTPUT, "Step position (0-10 V)");

        initPattern();
    }

    // ---- helpers ----
    static int randInt(int n)
    {
        if (n <= 1)
            return 0;
        return rack::math::clamp((int)std::floor(rack::random::uniform() * (float)n), 0, n - 1);
    }

    int getRoot() { return rack::math::clamp((int)std::round(params[ROOT_NOTE_PARAM].getValue()), 0, 11); }
    int getSpan() { return rack::math::clamp((int)std::round(params[OCTAVES_PARAM].getValue()), 1, 4); }
    int getScaleIdx()
    {
        return rack::math::clamp((int)std::round(params[SCALE_PARAM].getValue()), 0, (int)scales.size() - 1);
    }
    const std::vector<int> &getIntervals() { return scales[getScaleIdx()].intervals; }

    static bool inScale(int semi, int root, const std::vector<int> &iv)
    {
        int r = eaMod12(semi - root);
        for (int v : iv)
            if (v == r)
                return true;
        return false;
    }

    // Nearest scale note, folded into [root, root + 12*span - 1]
    int fixNote(int s)
    {
        int root = getRoot(), span = getSpan();
        const auto &iv = getIntervals();
        int best = s;
        for (int d = 0; d <= 6; ++d)
        {
            if (inScale(s - d, root, iv)) { best = s - d; break; }
            if (inScale(s + d, root, iv)) { best = s + d; break; }
        }
        int lo = root, hi = root + 12 * span - 1;
        while (best < lo) best += 12;
        while (best > hi) best -= 12;
        return best;
    }

    int quantizeToScale(float semiF) { return fixNote((int)std::lround(semiF)); }

    int randomNote()
    {
        int root = getRoot(), span = getSpan();
        const auto &iv = getIntervals();
        int d = iv[randInt((int)iv.size())];
        int o = randInt(span);
        return root + d + 12 * o;
    }

    void updateScaleMask(int root, int scaleIdx)
    {
        uint16_t m = 0;
        for (int v : scales[scaleIdx].intervals)
            m |= (uint16_t)(1 << eaMod12(root + v));
        scaleMask = m;
    }

    void requantizeAll()
    {
        for (int i = 0; i < 16; ++i)
            noteSemi[i] = fixNote(noteSemi[i]);
    }

    // Whole new pattern: clears locks and mutes
    void initPattern()
    {
        lastRoot = getRoot();
        lastScale = getScaleIdx();
        lastSpan = getSpan();
        updateScaleMask(lastRoot, lastScale);
        for (int i = 0; i < 16; ++i)
        {
            locked[i] = false;
            muted[i] = false;
            noteSemi[i] = randomNote();
            stepGlow[i] = 0.f;
        }
        numSteps = rack::math::clamp((int)std::round(params[STEPS_PARAM].getValue()), 2, 16);
        currentStep = 0;
        dirSign = 1;
        playing = false;
        curRest = false;
        stepCounter = 0;
        gateRemaining = 0.f;
        targetV = (float)noteSemi[0] / 12.f;
        playPc = -1;
    }

    void onReset() override
    {
        initPattern();
        outInit = false;
    }

    // Randomize unlocked steps (keeps the playhead where it is)
    void randomizePattern()
    {
        for (int i = 0; i < 16; ++i)
        {
            if (locked[i])
                continue;
            noteSemi[i] = randomNote();
            stepGlow[i] = 0.8f;
        }
        randFlash = 1.f;
    }

    void maybeMutate(float p)
    {
        if (p <= 0.f || rack::random::uniform() >= p)
            return;
        int cand[16], nc = 0;
        for (int i = 0; i < numSteps; ++i)
            if (!locked[i])
                cand[nc++] = i;
        if (nc == 0)
            return;
        int idx = cand[randInt(nc)];
        noteSemi[idx] = randomNote();
        mutIdx = idx;
        mutFlash = 1.f;
    }

    // Returns true at the end of a cycle
    bool advance(int dir)
    {
        int n = numSteps;
        bool eoc = false;
        stepCounter++;
        switch (dir)
        {
        case 0:
            currentStep++;
            if (currentStep >= n) { currentStep = 0; eoc = true; }
            break;
        case 1:
            currentStep--;
            if (currentStep < 0) { currentStep = n - 1; eoc = true; }
            break;
        case 2:
            currentStep = randInt(n);
            eoc = (stepCounter % (uint32_t)n) == 0;
            break;
        case 3:
        {
            int delta = randInt(3) - 1;
            currentStep = (currentStep + delta + n) % n;
            eoc = (stepCounter % (uint32_t)n) == 0;
            break;
        }
        default:
            if (n <= 1) { currentStep = 0; eoc = true; }
            else
            {
                currentStep += dirSign;
                if (currentStep >= n)     { currentStep = n - 2; dirSign = -1; }
                else if (currentStep < 0) { currentStep = 1;     dirSign = 1; eoc = true; }
            }
            break;
        }
        return eoc;
    }

    void doStep(float restP, float gateLen, float mutP)
    {
        int dir = rack::math::clamp((int)std::round(params[DIRECTION_PARAM].getValue()), 0, 4);
        bool eoc = false;
        if (!playing)
        {
            playing = true;
            dirSign = 1;
            currentStep = (dir == 1) ? numSteps - 1 : 0;
        }
        else
        {
            eoc = advance(dir);
        }
        currentStep = rack::math::clamp(currentStep, 0, numSteps - 1);

        bool rest = muted[currentStep] || (restP > 0.f && rack::random::uniform() < restP);
        curRest = rest;
        stepGlow[currentStep] = 1.f;

        if (!rest)
        {
            playPc = eaMod12(noteSemi[currentStep]);
            gateRemaining = (gateLen >= 0.995f) ? 1e9f : std::max(0.001f, clockPeriod * gateLen);
        }
        else
        {
            gateRemaining = 0.f;
            restFlash = 1.f;
        }

        if (eoc)
        {
            eocPulse.trigger(1e-3f);
            eocFlash = 1.f;
        }
        maybeMutate(mutP);
    }

    void process(const ProcessArgs &args) override
    {
        const float dt = args.sampleTime;

        // ---- key / scale / range changes: re-fit the existing notes ----
        int root = getRoot(), span = getSpan(), sIdx = getScaleIdx();
        if (root != lastRoot || sIdx != lastScale || span != lastSpan)
        {
            lastRoot = root;
            lastScale = sIdx;
            lastSpan = span;
            updateScaleMask(root, sIdx);
            requantizeAll();
        }

        numSteps = rack::math::clamp((int)std::round(params[STEPS_PARAM].getValue()), 2, 16);
        if (currentStep >= numSteps)
            currentStep = 0;

        float restP = params[REST_PARAM].getValue();
        float gateLen = params[GATE_LEN_PARAM].getValue();
        float glide = params[GLIDE_PARAM].getValue();
        int trans = rack::math::clamp(
            (int)std::round(params[TRANSPOSE_PARAM].getValue() + inputs[TRANSPOSE_CV_INPUT].getVoltage() * 12.f),
            -48, 48);
        float mutP = rack::math::clamp(
            params[MUTATION_PARAM].getValue() + inputs[MUT_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);

        // ---- randomize ----
        if (randBtnTrig.process(params[RANDOMIZE_BUTTON].getValue(), 0.1f, 1.f))
            randomizePattern();
        if (randTrig.process(inputs[RANDOM_TRIGGER_INPUT].getVoltage(), 0.1f, 1.f))
            randomizePattern();

        // ---- reset: the next clock plays the first step ----
        if (resetTrig.process(inputs[RESET_INPUT].getVoltage(), 0.1f, 1.f))
        {
            playing = false;
            stepCounter = 0;
            dirSign = 1;
            gateRemaining = 0.f;
            curRest = false;
        }

        // ---- clock ----
        sinceClock += dt;
        if (clockTrig.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 1.f))
        {
            if (clockSeen && sinceClock > 0.001f && sinceClock < 10.f)
                clockPeriod = sinceClock;
            clockSeen = true;
            sinceClock = 0.f;
            doStep(restP, gateLen, mutP);
        }

        // ---- pitch ----
        if (playing && !curRest)
            targetV = (float)noteSemi[currentStep] / 12.f;
        float tgt = targetV + (float)trans / 12.f;
        if (!outInit)
        {
            outV = tgt;
            outInit = true;
        }
        float tau = (glide < 0.002f) ? 0.f : 0.002f * std::pow(250.f, glide);   // 2 ms .. 500 ms
        float a = (tau > 0.f) ? (1.f - std::exp(-dt / tau)) : 1.f;
        outV += (tgt - outV) * a;
        outputs[VOCT_OUTPUT].setVoltage(outV);

        // ---- gate ----
        bool g = gateRemaining > 0.f;
        if (g && gateRemaining < 1e8f)
            gateRemaining -= dt;
        outputs[GATE_OUTPUT].setVoltage(g ? 10.f : 0.f);
        outputs[CYCLE_GATE_OUTPUT].setVoltage(eocPulse.process(dt) ? 10.f : 0.f);
        outputs[STEP_OUTPUT].setVoltage(
            playing ? 10.f * (float)currentStep / (float)std::max(numSteps - 1, 1) : 0.f);

        // ---- display decay ----
        for (int i = 0; i < 16; ++i)
            stepGlow[i] = std::max(0.f, stepGlow[i] - 2.f * dt);
        eocFlash = std::max(0.f, eocFlash - 4.f * dt);
        restFlash = std::max(0.f, restFlash - 4.f * dt);
        mutFlash = std::max(0.f, mutFlash - 2.f * dt);
        randFlash = std::max(0.f, randFlash - 6.f * dt);
        lights[RANDOM_LIGHT].setBrightness(randFlash);
        vTranspose = trans;
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_t *n = json_array(), *l = json_array(), *m = json_array();
        for (int i = 0; i < 16; ++i)
        {
            json_array_append_new(n, json_integer(noteSemi[i]));
            json_array_append_new(l, json_boolean(locked[i]));
            json_array_append_new(m, json_boolean(muted[i]));
        }
        json_object_set_new(root, "notes", n);
        json_object_set_new(root, "locked", l);
        json_object_set_new(root, "muted", m);
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *n = json_object_get(root, "notes");
        json_t *l = json_object_get(root, "locked");
        json_t *m = json_object_get(root, "muted");
        for (int i = 0; i < 16; ++i)
        {
            if (n && json_is_array(n))
            {
                json_t *v = json_array_get(n, i);
                if (v)
                    noteSemi[i] = (int)json_integer_value(v);
            }
            if (l && json_is_array(l))
            {
                json_t *v = json_array_get(l, i);
                if (v)
                    locked[i] = json_is_true(v);
            }
            if (m && json_is_array(m))
            {
                json_t *v = json_array_get(m, i);
                if (v)
                    muted[i] = json_is_true(v);
            }
        }
        targetV = (float)noteSemi[rack::math::clamp(currentStep, 0, 15)] / 12.f;
        outInit = false;
    }
};

// ----------------------------------------------------------------------------
// Theme + drawing helpers (anonymous namespace)
// ----------------------------------------------------------------------------
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x25, 0x28, 0x30); }
NVGcolor colBgBottom() { return nvgRGB(0x13, 0x15, 0x19); }
NVGcolor colText()     { return nvgRGB(0xd6, 0xdc, 0xe6); }
NVGcolor colDim()      { return nvgRGB(0x8a, 0x95, 0xa5); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colBright()   { return nvgRGB(0x7a, 0xec, 0xff); }
NVGcolor colWarm()     { return nvgRGB(0xff, 0x9c, 0x6a); }
NVGcolor colViolet()   { return nvgRGB(0xb0, 0x5a, 0xff); }

NVGcolor withA(NVGcolor c, float a) { return nvgRGBAf(c.r, c.g, c.b, a); }

const int EA_WPC[7]  = {0, 2, 4, 5, 7, 9, 11};   // white keys
const int EA_BPC[5]  = {1, 3, 6, 8, 10};         // black keys
const int EA_BIDX[5] = {0, 1, 3, 4, 5};          // white key left of each black key

// Knob table: parameter, position, tick count, bipolar arc, label
struct EAKnob { int param; float x, y; int ticks; bool bipolar; const char* label; };
const EAKnob EA_KNOBS[10] = {
    {ExpressArpeggiator::ROOT_NOTE_PARAM,  36.f,  82.f, 12, false, "KEY"},
    {ExpressArpeggiator::SCALE_PARAM,      36.f, 152.f, 11, false, "SCALE"},
    {ExpressArpeggiator::STEPS_PARAM,     384.f,  82.f,  8, false, "STEPS"},
    {ExpressArpeggiator::DIRECTION_PARAM, 384.f, 152.f,  5, false, "MOVE"},
    {ExpressArpeggiator::MUTATION_PARAM,   50.f, 246.f, 11, false, "MUTATE"},
    {ExpressArpeggiator::OCTAVES_PARAM,   114.f, 246.f,  4, false, "RANGE"},
    {ExpressArpeggiator::GATE_LEN_PARAM,  178.f, 246.f, 11, false, "GATE"},
    {ExpressArpeggiator::REST_PARAM,      242.f, 246.f, 11, false, "REST"},
    {ExpressArpeggiator::GLIDE_PARAM,     306.f, 246.f, 11, false, "GLIDE"},
    {ExpressArpeggiator::TRANSPOSE_PARAM, 370.f, 246.f,  9, true,  "TRANS"},
};

void setFont(NVGcontext* vg) {
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font) nvgFontFaceId(vg, font->handle);
}

void drawText(NVGcontext* vg, float x, float y, const char* s, float size, NVGcolor col,
              int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f) {
    setFont(vg);
    nvgFontSize(vg, size);
    nvgTextLetterSpacing(vg, spacing);
    nvgTextAlign(vg, align);
    nvgFillColor(vg, col);
    nvgText(vg, x, y, s, NULL);
}

// Custom light for the button LED
template <typename TBase = GrayModuleLightWidget>
struct TEACyanLight : TBase {
    TEACyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using EACyanLight = TEACyanLight<>;

void drawLogo(NVGcontext* vg, float cx, float cy) {
    const float w = 64.f, h = 18.f;
    float x = cx - w / 2.f, y = cy - h / 2.f;

    nvgBeginPath(vg);
    nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 7.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                       nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
    nvgFill(vg);
    nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
                                         nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
    nvgStrokeWidth(vg, 1.2f);
    nvgStroke(vg);

    const float s = 2.2f;
    float tx = x + 7.f, ty = y + h / 2.f - 3.f;
    nvgBeginPath(vg);
    nvgRect(vg, tx + s,       ty,            s, s);
    nvgRect(vg, tx,           ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);

    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
             NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

// Tick arc around a knob (n ticks across the 270 degree sweep)
void drawKnobTicks(NVGcontext* vg, float cx, float cy, float r0, float r1, int n = 11) {
    nvgLineCap(vg, NVG_ROUND);
    for (int i = 0; i < n; ++i) {
        float a = (-0.75f + 1.5f * (float)i / (float)(n - 1)) * (float)M_PI;
        bool major = (i == 0 || i == n - 1 || (n % 2 == 1 && i == (n - 1) / 2));
        float ra = major ? r0 - 1.5f : r0;
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + std::sin(a) * ra, cy - std::cos(a) * ra);
        nvgLineTo(vg, cx + std::sin(a) * r1, cy - std::cos(a) * r1);
        nvgStrokeColor(vg, major ? nvgRGBA(0x3d, 0xe0, 0xff, 150) : nvgRGBA(255, 255, 255, 60));
        nvgStrokeWidth(vg, major ? 1.2f : 0.9f);
        nvgStroke(vg);
    }
}

// Small padlock glyph
void drawLock(NVGcontext* vg, float cx, float cy, bool on) {
    NVGcolor col = on ? colViolet() : colDim();
    float al = on ? 0.95f : 0.28f;
    if (on) {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy + 1.f, 5.5f);
        nvgFillColor(vg, withA(colViolet(), 0.16f));
        nvgFill(vg);
    }
    float sy = on ? cy - 0.5f : cy - 2.2f;
    nvgBeginPath(vg);
    nvgArc(vg, cx, sy, 2.1f, (float)M_PI, 2.f * (float)M_PI, NVG_CW);
    nvgStrokeColor(vg, withA(col, al));
    nvgStrokeWidth(vg, 1.1f);
    nvgStroke(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, cx - 3.f, cy - 0.5f, 6.f, 4.6f, 1.f);
    nvgFillColor(vg, withA(col, al * 0.9f));
    nvgFill(vg);
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct EAPanel : TransparentWidget {
    EAPanel() { box.size = Vec(EA_W, EA_H); }

    void label(NVGcontext* vg, float x, float y, const char* txt) {
        drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.0f);
    }

    void plate(NVGcontext* vg, float x, float y, float w, float h) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 1.f, y - 1.f, w + 2.f, h + 3.f, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 6.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                           nvgRGBA(0, 0, 0, 105), nvgRGBA(0, 0, 0, 60)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void ring(NVGcontext* vg, float cx, float cy, float r) {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, r * 0.3f, r,
                                           nvgRGBA(0, 0, 0, 40), nvgRGBA(0, 0, 0, 110)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 12.f, x2 = EA_W - 12.f, cx = EA_W / 2.f;
        float w = 14.f + (float) strlen(txt) * 7.f;
        drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - w / 2.f, y);
        nvgStrokePaint(vg, nvgLinearGradient(vg, x1, y, cx - w / 2.f, y,
                                             nvgRGBA(0x3d, 0xe0, 0xff, 0), nvgRGBA(0x3d, 0xe0, 0xff, 100)));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + w / 2.f, y);
        nvgLineTo(vg, x2, y);
        nvgStrokePaint(vg, nvgLinearGradient(vg, cx + w / 2.f, y, x2, y,
                                             nvgRGBA(0x3d, 0xe0, 0xff, 100), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // kind: 0 = input (orange), 1 = output (cyan), 2 = control (purple)
    void cell(NVGcontext* vg, float cx, float cy, const char* txt, int kind) {
        const float w = 32.f, top = -27.f, h = 44.f;
        float x = cx - w / 2.f, y = cy + top;
        plate(vg, x, y, w, h);

        NVGcolor c = (kind == 0) ? nvgRGBA(0xff, 0x9c, 0x6a, 170)
                   : (kind == 1) ? nvgRGBA(0x3d, 0xe0, 0xff, 170)
                                 : nvgRGBA(0xb0, 0x5a, 0xff, 170);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x + 6.f, y + 1.5f, w - 12.f, 1.8f, 0.9f);
        nvgFillColor(vg, c);
        nvgFill(vg);

        label(vg, cx, cy - 19.f, txt);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, EA_W, EA_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, EA_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) EA_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, EA_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Corner vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, EA_W, EA_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, EA_W / 2.f, EA_H * 0.4f,
                                           EA_H * 0.35f, EA_H * 0.95f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
        nvgFill(vg);

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, EA_W - 1.f, EA_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Top accent line
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, EA_W, 2.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, EA_W, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // Logo + title
        drawLogo(vg, EA_W / 2.f, 15.f);
        drawText(vg, EA_W / 2.f, 32.f, "EXPRESSARP", 12.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // ---- Display bezel (metallic, recessed) ----
        float bx = EA_DX - 5.f, by = EA_DY - 5.f;
        float bw = EA_DW + 10.f, bh = EA_DH + 10.f;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 4.f, by - 3.f, bw + 8.f, bh + 9.f, 10.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x4a, 0x50, 0x5a), nvgRGB(0x14, 0x16, 0x1a)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 2.f, by - 2.f, bw + 4.f, bh + 4.f, 7.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 36));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- Knobs: wells, tick scales, labels ----
        for (int k = 0; k < 10; ++k) {
            const EAKnob& kn = EA_KNOBS[k];
            ring(vg, kn.x, kn.y, 17.f);
            drawKnobTicks(vg, kn.x, kn.y, 21.f, 24.f, kn.ticks);
            label(vg, kn.x, kn.y + 29.f, kn.label);
        }

        // ---- Section headers ----
        section(vg, EA_CTRL_SEC_Y, "CONTROL");
        section(vg, EA_IO_SEC_Y, "I / O");

        // ---- I/O row ----
        const char* io[10] = {"CLOCK", "RESET", "RND IN", "TRN CV", "MUT CV",
                              "RANDOM", "V/OCT", "GATE", "EOC", "STEP"};
        const int kind[10] = {0, 0, 0, 0, 0, 2, 1, 1, 1, 1};
        for (int i = 0; i < 10; ++i)
            cell(vg, eaIoX(i), EA_IO_Y, io[i], kind[i]);

        char buf[64];
        snprintf(buf, sizeof(buf), "16 STEP  -  %d SCALES  -  GATE / V/OCT",
                 (int) scaleTable().size());
        drawText(vg, EA_W / 2.f, 368.f, buf, 6.f, nvgRGBA(0x8a, 0x95, 0xa5, 120),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Live value arcs around the knobs
// ------------------------------------------------------------
struct EAKnobArcs : TransparentWidget {
    ExpressArpeggiator* module = nullptr;

    EAKnobArcs(ExpressArpeggiator* m) : module(m) { box.size = Vec(EA_W, EA_H); }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float R = 17.2f;
        const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
        const float a1 =  0.75f * (float)M_PI - (float)M_PI / 2.f;

        for (int k = 0; k < 10; ++k) {
            const EAKnob& kn = EA_KNOBS[k];

            nvgBeginPath(vg);
            nvgArc(vg, kn.x, kn.y, R, a0, a1, NVG_CW);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 2.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStroke(vg);

            if (!module) continue;
            float v = module->paramQuantities[kn.param]->getScaledValue();   // 0..1
            float from = kn.bipolar ? 0.5f : 0.f;
            float lo = std::min(from, v), hi = std::max(from, v);
            if (hi - lo < 0.004f) continue;

            float s = a0 + (a1 - a0) * lo;
            float e = a0 + (a1 - a0) * hi;
            NVGcolor col = (kn.bipolar && v < 0.5f) ? colWarm() : colAccent();

            nvgBeginPath(vg);
            nvgArc(vg, kn.x, kn.y, R, s, e, NVG_CW);
            nvgStrokeColor(vg, withA(col, 0.18f));
            nvgStrokeWidth(vg, 5.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgArc(vg, kn.x, kn.y, R, s, e, NVG_CW);
            nvgStrokeColor(vg, withA(col, 0.95f));
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
        }
    }
};

// ------------------------------------------------------------
// Pattern display (interactive)
// ------------------------------------------------------------
enum { Z_NONE = 0, Z_BARS, Z_GATE, Z_LOCK, Z_NUM, Z_DIR };

struct EADisplay : Widget {
    ExpressArpeggiator* module = nullptr;
    int  hoverZone = Z_NONE, hoverIdx = -1;
    bool editing = false;
    Vec  dragPos;

    EADisplay(ExpressArpeggiator* m) : module(m) {
        box.pos = Vec(EA_DX, EA_DY);
        box.size = Vec(EA_DW, EA_DH);
    }

    // ---- hit testing ----
    int zoneAt(Vec p, int& idx) const {
        idx = -1;
        if (p.x >= EA_BX0 && p.x < EA_BX0 + 16.f * EA_CW) {
            int c = rack::math::clamp((int)std::floor((p.x - EA_BX0) / EA_CW), 0, 15);
            if (p.y >= EA_BY0 - 3.f && p.y <= EA_BY0 + EA_BH + 2.f) { idx = c; return Z_BARS; }
            if (p.y >= EA_GATE_Y - 1.f && p.y <= EA_GATE_Y + EA_GATE_H + 1.f) { idx = c; return Z_GATE; }
            if (p.y >= EA_LOCK_Y - 1.f && p.y <= EA_LOCK_Y + EA_LOCK_H + 1.f) { idx = c; return Z_LOCK; }
            if (p.y >= EA_NUM_Y - 3.5f && p.y <= EA_NUM_Y + 3.5f) { idx = c; return Z_NUM; }
        }
        if (p.y >= EA_CHIP_Y && p.y <= EA_CHIP_Y + EA_CHIP_H && p.x >= EA_CHIP_X) {
            float rel = p.x - EA_CHIP_X;
            int i = (int)std::floor(rel / EA_CHIP_P);
            if (i >= 0 && i < 5 && rel - (float)i * EA_CHIP_P <= EA_CHIP_W) { idx = i; return Z_DIR; }
        }
        return Z_NONE;
    }

    // ---- paint a note from the pointer position ----
    void applyEdit(Vec p) {
        if (!module) return;
        int c = rack::math::clamp((int)std::floor((p.x - EA_BX0) / EA_CW), 0, 15);
        if (c >= module->numSteps) return;
        float t = rack::math::clamp((EA_BY0 + EA_BH - p.y) / EA_BH, 0.f, 1.f);
        int root = module->getRoot(), span = module->getSpan();
        float semiF = (float)root + t * 12.f * (float)span - 1.f;
        module->noteSemi[c] = module->quantizeToScale(semiF);
        module->stepGlow[c] = 1.f;
    }

    // ---- mouse ----
    void onButton(const ButtonEvent& e) override {
        if (module && e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS) {
            int idx;
            int z = zoneAt(e.pos, idx);
            if (z != Z_NONE) {
                switch (z) {
                    case Z_BARS:
                        editing = true;
                        dragPos = e.pos;
                        applyEdit(dragPos);
                        break;
                    case Z_GATE:
                        module->muted[idx] = !module->muted[idx];
                        break;
                    case Z_LOCK:
                        module->locked[idx] = !module->locked[idx];
                        break;
                    case Z_NUM:
                        module->params[ExpressArpeggiator::STEPS_PARAM].setValue(
                            (float)std::max(2, idx + 1));
                        break;
                    case Z_DIR:
                        module->params[ExpressArpeggiator::DIRECTION_PARAM].setValue((float)idx);
                        break;
                }
                e.consume(this);
                return;
            }
        }
        Widget::onButton(e);
    }

    void onDragMove(const DragMoveEvent& e) override {
        if (!editing || !module) return;
        dragPos = dragPos.plus(e.mouseDelta.div(getAbsoluteZoom()));
        applyEdit(dragPos);
    }

    void onDragEnd(const DragEndEvent& e) override {
        editing = false;
        Widget::onDragEnd(e);
    }

    void onHover(const HoverEvent& e) override {
        int idx;
        int z = zoneAt(e.pos, idx);
        hoverZone = z;
        hoverIdx = idx;
        if (z != Z_NONE) e.consume(this);
        else Widget::onHover(e);
    }

    void onLeave(const LeaveEvent& e) override {
        hoverZone = Z_NONE;
        hoverIdx = -1;
        Widget::onLeave(e);
    }

    // ---- static background ----
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0, 0, EA_DW, EA_DH, 4.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, EA_DH,
                                           nvgRGB(0x0a, 0x16, 0x1f), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0.5f, 0.5f, EA_DW - 1.f, EA_DH - 1.f, 4.f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        NVGcolor rl = nvgRGBA(0x8a, 0x95, 0xa5, 150);
        drawText(vg, EA_BX0 - 3.f, EA_GATE_Y + EA_GATE_H / 2.f, "GATE", 4.8f, rl,
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, EA_BX0 - 3.f, EA_LOCK_Y + EA_LOCK_H / 2.f, "LOCK", 4.8f, rl,
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, EA_BX0 - 3.f, EA_NUM_Y, "STEP", 4.8f, rl,
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.5f);

        Widget::draw(args);
    }

    // ---- live content ----
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float W = EA_DW, H = EA_DH;

        // ----- snapshot of the state -----
        int n = 8, cur = 2, dir = 0, root = 0, scaleIdx = 0, span = 2;
        int mutIdx = -1, playPc = -1, trans = 0;
        bool playing = true, curRest = false, gateOn = true;
        int notes[16];
        bool lk[16], mt[16];
        float glow[16];
        float eocF = 0.f, restF = 0.f, mutF = 0.f, mutP = 0.2f, gateLen = 0.5f;
        uint16_t mask = 0xAB5;

        if (module) {
            n = module->numSteps;
            cur = module->currentStep;
            playing = module->playing;
            curRest = module->curRest;
            gateOn = module->gateRemaining > 0.f;
            for (int i = 0; i < 16; ++i) {
                notes[i] = module->noteSemi[i];
                lk[i] = module->locked[i];
                mt[i] = module->muted[i];
                glow[i] = module->stepGlow[i];
            }
            eocF = module->eocFlash;
            restF = module->restFlash;
            mutF = module->mutFlash;
            mutIdx = module->mutIdx;
            playPc = module->playPc;
            trans = module->vTranspose;
            mask = module->scaleMask;
            dir = (int) std::round(module->params[ExpressArpeggiator::DIRECTION_PARAM].getValue());
            root = (int) std::round(module->params[ExpressArpeggiator::ROOT_NOTE_PARAM].getValue());
            scaleIdx = (int) std::round(module->params[ExpressArpeggiator::SCALE_PARAM].getValue());
            span = (int) std::round(module->params[ExpressArpeggiator::OCTAVES_PARAM].getValue());
            mutP = module->params[ExpressArpeggiator::MUTATION_PARAM].getValue();
            gateLen = module->params[ExpressArpeggiator::GATE_LEN_PARAM].getValue();
        } else {
            // demo values for the module browser
            static const int demo[16] = {0, 4, 7, 12, 9, 5, 16, 11, 2, 7, 14, 9, 4, 0, 12, 7};
            for (int i = 0; i < 16; ++i) {
                notes[i] = demo[i];
                lk[i] = (i == 3);
                mt[i] = (i == 5);
                glow[i] = (i == 2) ? 0.7f : 0.f;
            }
            playPc = 7;
        }
        n = rack::math::clamp(n, 2, 16);
        cur = rack::math::clamp(cur, 0, n - 1);
        dir = rack::math::clamp(dir, 0, 4);
        root = rack::math::clamp(root, 0, 11);
        span = rack::math::clamp(span, 1, 4);
        scaleIdx = rack::math::clamp(scaleIdx, 0, (int) scaleTable().size() - 1);

        const float baseY = EA_BY0 + EA_BH;
        const float rangeSemi = 12.f * (float) span;

        nvgSave(vg);
        nvgScissor(vg, 0, 0, W, H);

        // ================= column slots =================
        for (int c = 0; c < 16; ++c) {
            bool active = (c < n);
            bool isCur = playing && c == cur;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, EA_BX0 + (float)c * EA_CW + 1.f, EA_BY0,
                           EA_CW - 2.f, EA_BH, 2.f);
            float a = active ? (isCur ? 0.05f + 0.08f * glow[c] + 0.05f : 0.035f) : 0.012f;
            nvgFillColor(vg, withA(isCur ? colAccent() : nvgRGB(255, 255, 255), a));
            nvgFill(vg);
        }

        // ================= pitch grid =================
        {
            int count = 0;
            for (int pc = 0; pc < 12; ++pc)
                if ((mask >> pc) & 1) count++;
            count *= span;

            // scale tone guides + edge ticks
            for (int s = root; s < root + 12 * span; ++s) {
                if (!((mask >> eaMod12(s)) & 1)) continue;
                float f = ((float)(s - root) + 1.f) / rangeSemi;
                float y = baseY - EA_BH * f;
                if (count <= 16) {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, EA_BX0, y + 0.5f);
                    nvgLineTo(vg, EA_BX0 + 16.f * EA_CW, y + 0.5f);
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 9));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                if (count <= 40) {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, EA_BX0 - 2.f, y + 0.5f);
                    nvgLineTo(vg, EA_BX0, y + 0.5f);
                    nvgStrokeColor(vg, withA(colAccent(), 0.5f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
            }

            // octave lines with note names
            for (int j = 0; j < span; ++j) {
                float f = (12.f * (float)j + 1.f) / rangeSemi;
                float y = baseY - EA_BH * f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, EA_BX0 - 2.f, y + 0.5f);
                nvgLineTo(vg, EA_BX0 + 16.f * EA_CW, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, j == 0 ? 42 : 26));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                int semi = root + 12 * j;
                char b[8];
                snprintf(b, sizeof(b), "%s%d", EA_NN[eaMod12(semi)], 4 + (int) std::floor((float) semi / 12.f));
                drawText(vg, EA_BX0 - 4.f, y - 3.f, b, 5.f, nvgRGBA(0xff, 0x9c, 0x6a, 190),
                         NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.3f);
            }

            // baseline
            nvgBeginPath(vg);
            nvgMoveTo(vg, EA_BX0, baseY + 0.5f);
            nvgLineTo(vg, EA_BX0 + 16.f * EA_CW, baseY + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // pattern end marker
            float ex = EA_BX0 + (float) n * EA_CW;
            nvgBeginPath(vg);
            nvgMoveTo(vg, ex + 0.5f, EA_BY0 - 2.f);
            nvgLineTo(vg, ex + 0.5f, EA_NUM_Y + 4.f);
            nvgStrokeColor(vg, withA(colViolet(), 0.35f));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // ================= bars =================
        float tipX[16], tipY[16];
        for (int c = 0; c < 16; ++c) {
            float x = EA_BX0 + (float) c * EA_CW + 1.5f, w = EA_CW - 3.f;
            float f = rack::math::clamp(((float)(notes[c] - root) + 1.f) / rangeSemi, 0.f, 1.f);
            float h = std::max(f * EA_BH, 2.f);
            float top = baseY - h;
            tipX[c] = x + w / 2.f;
            tipY[c] = top;

            if (c >= n) {
                nvgBeginPath(vg);
                nvgRect(vg, x + 1.f, baseY - 1.f, w - 2.f, 1.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 22));
                nvgFill(vg);
                continue;
            }

            bool isCur = playing && (c == cur);
            float g = glow[c];
            bool mutated = (c == mutIdx && mutF > 0.02f);
            bool isRootNote = (eaMod12(notes[c]) == root);
            bool isMuted = mt[c];

            NVGcolor hi = mutated ? nvgRGB(0xd8, 0xb0, 0xff) : nvgRGB(0x8a, 0xee, 0xff);
            NVGcolor lo = mutated ? nvgRGB(0x7a, 0x3a, 0xc8) : nvgRGB(0x1f, 0x9f, 0xc0);
            if (isMuted) { hi = nvgRGB(0x7a, 0x85, 0x94); lo = nvgRGB(0x30, 0x38, 0x42); }
            float a = isMuted ? 0.35f : (0.50f + 0.50f * std::max(g, isCur ? 0.5f : 0.f));

            if (!isMuted && (g > 0.02f || isCur)) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x - 2.f, top - 2.f, w + 4.f, h + 4.f, 3.f);
                nvgFillColor(vg, withA(mutated ? colViolet() : colAccent(), 0.10f + 0.28f * g));
                nvgFill(vg);
            }

            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, top, w, h, 1.5f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, baseY, 0, top, withA(lo, a), withA(hi, a)));
            nvgFill(vg);

            // tip cap: orange on the root, violet when locked
            NVGcolor cap = isRootNote ? colWarm() : (lk[c] ? colViolet() : nvgRGB(255, 255, 255));
            float capA = isMuted ? 0.3f : (isCur ? 0.95f : 0.6f);
            if (isRootNote || lk[c]) capA = std::max(capA, 0.85f);
            nvgBeginPath(vg);
            nvgRect(vg, x - 0.5f, top - 0.5f, w + 1.f, 1.8f);
            nvgFillColor(vg, withA(cap, capA));
            nvgFill(vg);
        }

        // pitch contour through the bar tips
        {
            nvgLineCap(vg, NVG_ROUND);
            nvgLineJoin(vg, NVG_ROUND);
            nvgBeginPath(vg);
            for (int c = 0; c < n; ++c) {
                if (c == 0) nvgMoveTo(vg, tipX[c], tipY[c]);
                else        nvgLineTo(vg, tipX[c], tipY[c]);
            }
            nvgStrokeColor(vg, withA(colBright(), 0.10f));
            nvgStrokeWidth(vg, 3.5f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            for (int c = 0; c < n; ++c) {
                if (c == 0) nvgMoveTo(vg, tipX[c], tipY[c]);
                else        nvgLineTo(vg, tipX[c], tipY[c]);
            }
            nvgStrokeColor(vg, withA(colBright(), 0.40f));
            nvgStrokeWidth(vg, 0.9f);
            nvgStroke(vg);

            for (int c = 0; c < n; ++c) {
                bool isCur = playing && (c == cur);
                nvgBeginPath(vg);
                nvgCircle(vg, tipX[c], tipY[c], isCur ? 2.3f : 1.3f);
                nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, isCur ? 0.97f : 0.55f));
                nvgFill(vg);
            }
        }

        // ================= note names =================
        for (int c = 0; c < n; ++c) {
            char b[8];
            int s = notes[c];
            snprintf(b, sizeof(b), "%s%d", EA_NN[eaMod12(s)], 4 + (int) std::floor((float) s / 12.f));
            bool isCur = playing && (c == cur);
            drawText(vg, EA_BX0 + ((float) c + 0.5f) * EA_CW, EA_NOTE_Y, b, 5.2f,
                     isCur ? colBright() : (mt[c] ? nvgRGBA(0x8a, 0x95, 0xa5, 110) : nvgRGBA(0xc9, 0xd1, 0xdc, 160)),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        }

        // ================= gate / mute row =================
        for (int c = 0; c < 16; ++c) {
            float x = EA_BX0 + (float) c * EA_CW + 1.5f, w = EA_CW - 3.f;
            bool active = (c < n);
            bool isCur = playing && (c == cur);

            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, EA_GATE_Y, w, EA_GATE_H, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, active ? 10 : 4));
            nvgFill(vg);

            if (!active) continue;

            if (mt[c]) {
                nvgBeginPath(vg);
                nvgMoveTo(vg, x + 2.f, EA_GATE_Y + EA_GATE_H - 1.5f);
                nvgLineTo(vg, x + w - 2.f, EA_GATE_Y + 1.5f);
                nvgStrokeColor(vg, withA(colWarm(), 0.7f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            } else {
                float fw = std::max(2.f, w * gateLen);
                float al = 0.40f + 0.45f * glow[c];
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, EA_GATE_Y, fw, EA_GATE_H, 2.f);
                if (isCur && curRest)
                    nvgFillColor(vg, withA(colWarm(), 0.75f));
                else
                    nvgFillColor(vg, withA(colAccent(), al));
                nvgFill(vg);

                if (isCur && gateOn && !curRest) {
                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, x - 0.5f, EA_GATE_Y - 0.5f, w + 1.f, EA_GATE_H + 1.f, 2.5f);
                    nvgStrokeColor(vg, withA(colBright(), 0.9f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
            }
            if (isCur && curRest && !mt[c])
                drawText(vg, x + w / 2.f, EA_GATE_Y + EA_GATE_H / 2.f + 0.5f, "R", 5.5f,
                         nvgRGBA(0x10, 0x10, 0x14, 230), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        }

        // ================= lock row =================
        for (int c = 0; c < n; ++c)
            drawLock(vg, EA_BX0 + ((float) c + 0.5f) * EA_CW, EA_LOCK_Y + 4.f, lk[c]);

        // ================= step numbers + playhead =================
        for (int c = 0; c < 16; ++c) {
            char b[4];
            snprintf(b, sizeof(b), "%d", c + 1);
            float cx = EA_BX0 + ((float) c + 0.5f) * EA_CW;
            bool active = (c < n);
            bool isCur = playing && (c == cur);

            if (isCur) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, cx - 5.5f, EA_NUM_Y - 3.5f, 11.f, 7.f, 2.f);
                nvgStrokeColor(vg, withA(colAccent(), 0.85f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                nvgBeginPath(vg);
                nvgMoveTo(vg, cx - 2.2f, EA_LOCK_Y + EA_LOCK_H + 0.2f);
                nvgLineTo(vg, cx + 2.2f, EA_LOCK_Y + EA_LOCK_H + 0.2f);
                nvgLineTo(vg, cx, EA_NUM_Y - 3.8f);
                nvgClosePath(vg);
                nvgFillColor(vg, withA(colBright(), 0.9f));
                nvgFill(vg);
            }
            drawText(vg, cx, EA_NUM_Y + 0.5f, b, 5.2f,
                     isCur ? colBright() : (active ? nvgRGBA(0x8a, 0x95, 0xa5, 220) : nvgRGBA(0x8a, 0x95, 0xa5, 70)),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        }

        // hover outline
        if (hoverZone != Z_NONE && hoverZone != Z_DIR && hoverIdx >= 0) {
            float x = EA_BX0 + (float) hoverIdx * EA_CW + 0.5f;
            float y0 = 0.f, h = 0.f;
            if (hoverZone == Z_BARS) { y0 = EA_BY0 - 1.f; h = EA_BH + 2.f; }
            else if (hoverZone == Z_GATE) { y0 = EA_GATE_Y - 1.f; h = EA_GATE_H + 2.f; }
            else if (hoverZone == Z_LOCK) { y0 = EA_LOCK_Y - 1.f; h = EA_LOCK_H + 2.f; }
            else { y0 = EA_NUM_Y - 4.f; h = 8.f; }
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, y0, EA_CW - 1.f, h, 2.f);
            nvgStrokeColor(vg, withA(colBright(), 0.55f));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // ================= mini keyboard =================
        {
            const float ww = (EA_KB_X1 - EA_KB_X0) / 7.f;

            nvgBeginPath(vg);
            nvgRoundedRect(vg, EA_KB_X0 - 2.f, EA_KB_Y - 2.f,
                           EA_KB_X1 - EA_KB_X0 + 4.f, EA_KB_WH + 4.f, 3.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
            nvgFill(vg);

            auto drawKey = [&](float x, float y, float w, float h, int pc, bool black) {
                bool in = (mask >> pc) & 1;
                bool isRoot = (pc == root);
                bool sounding = (playing && !curRest && pc == playPc);
                float gl = playing ? glow[cur] : 0.f;

                if (sounding) {
                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 3.f);
                    nvgFillColor(vg, withA(colAccent(), 0.14f + 0.28f * gl));
                    nvgFill(vg);
                }

                nvgBeginPath(vg);
                nvgRoundedRectVarying(vg, x, y, w, h, 1.f, 1.f, 3.f, 3.f);
                nvgFillColor(vg, black ? nvgRGB(0x08, 0x0a, 0x0e) : nvgRGB(0x16, 0x1c, 0x24));
                nvgFill(vg);

                if (in) {
                    NVGcolor tint = isRoot ? colWarm() : colAccent();
                    nvgBeginPath(vg);
                    nvgRoundedRectVarying(vg, x, y, w, h, 1.f, 1.f, 3.f, 3.f);
                    nvgFillPaint(vg, nvgLinearGradient(vg, 0, y, 0, y + h,
                                                       withA(tint, 0.10f), withA(tint, 0.42f)));
                    nvgFill(vg);
                }
                if (sounding) {
                    nvgBeginPath(vg);
                    nvgRoundedRectVarying(vg, x, y, w, h, 1.f, 1.f, 3.f, 3.f);
                    nvgFillColor(vg, nvgRGBAf(0.80f, 0.97f, 1.f, 0.30f + 0.40f * gl));
                    nvgFill(vg);
                }

                nvgBeginPath(vg);
                nvgRoundedRectVarying(vg, x, y, w, h, 1.f, 1.f, 3.f, 3.f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, black ? 34 : 44));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                if (isRoot) {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 2.f, y + h - 3.f, w - 4.f, 1.8f);
                    nvgFillColor(vg, withA(colWarm(), 0.95f));
                    nvgFill(vg);
                }
            };

            for (int i = 0; i < 7; ++i)
                drawKey(EA_KB_X0 + (float) i * ww + 0.5f, EA_KB_Y, ww - 1.f, EA_KB_WH, EA_WPC[i], false);
            for (int j = 0; j < 5; ++j) {
                float bx = EA_KB_X0 + (float)(EA_BIDX[j] + 1) * ww - EA_KB_BW / 2.f;
                drawKey(bx, EA_KB_Y, EA_KB_BW, EA_KB_BH, EA_BPC[j], true);
            }
        }

        // ================= direction chips =================
        {
            const char* dn[5] = {"FWD", "BWD", "RND", "DRK", "PND"};
            for (int i = 0; i < 5; ++i) {
                float x = EA_CHIP_X + (float) i * EA_CHIP_P;
                bool on = (i == dir);
                bool hv = (hoverZone == Z_DIR && hoverIdx == i);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, EA_CHIP_Y, EA_CHIP_W, EA_CHIP_H, 3.f);
                if (on) {
                    nvgFillColor(vg, withA(colAccent(), 0.25f));
                    nvgFill(vg);
                    nvgStrokeColor(vg, withA(colAccent(), 0.9f));
                } else {
                    nvgStrokeColor(vg, hv ? withA(colBright(), 0.7f) : nvgRGBA(255, 255, 255, 30));
                }
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                drawText(vg, x + EA_CHIP_W / 2.f, EA_CHIP_Y + EA_CHIP_H / 2.f + 0.5f, dn[i], 6.f,
                         on ? colAccent() : colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
            }
        }

        // ================= status chips =================
        {
            const char* sn[4] = {"GATE", "EOC", "REST", "MUT"};
            float lit[4] = {gateOn && playing && !curRest ? 1.f : 0.f, eocF, restF, mutF};
            NVGcolor sc[4] = {colAccent(), colViolet(), colWarm(), colViolet()};
            for (int i = 0; i < 4; ++i) {
                float x = EA_CHIP_X + (float) i * 32.7f;
                float y = 136.f, w = 30.f, h = 10.f;
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, y, w, h, 3.f);
                if (lit[i] > 0.02f) {
                    nvgFillColor(vg, withA(sc[i], 0.12f + 0.30f * lit[i]));
                    nvgFill(vg);
                    nvgStrokeColor(vg, withA(sc[i], 0.9f));
                } else {
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                }
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                drawText(vg, x + w / 2.f, y + h / 2.f + 0.5f, sn[i], 5.8f,
                         lit[i] > 0.02f ? sc[i] : colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
            }
        }

        // ================= readouts =================
        {
            char buf[64];
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 220);
            NVGcolor cm = nvgRGBA(0xff, 0xe6, 0x80, 220);
            NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 230);

            std::string name = scaleTable()[scaleIdx].name;
            if (name.size() > 20) name = name.substr(0, 18) + "..";
            drawText(vg, 6.f, 8.f, name.c_str(), 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

            snprintf(buf, sizeof(buf), "TRN %+d  %dOCT", trans, span);
            drawText(vg, W / 2.f + 14.f, 8.f, buf, 7.f, cd, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

            snprintf(buf, sizeof(buf), "%s4  MUT %.0f%%", EA_NN[root], mutP * 100.f);
            drawText(vg, W - 6.f, 8.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            // caption (help text while hovering)
            const char* help = nullptr;
            switch (hoverZone) {
                case Z_BARS: help = "DRAG: PAINT NOTES"; break;
                case Z_GATE: help = "CLICK: MUTE STEP"; break;
                case Z_LOCK: help = "CLICK: LOCK STEP"; break;
                case Z_NUM:  help = "CLICK: SET LENGTH"; break;
                case Z_DIR:  help = "CLICK: SET DIRECTION"; break;
                default: break;
            }
            if (help) {
                drawText(vg, EA_KB_X0, 144.5f, help, 5.5f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            } else {
                snprintf(buf, sizeof(buf), "STEP %02d/%02d", playing ? cur + 1 : 0, n);
                drawText(vg, EA_KB_X0, 144.5f, buf, 5.5f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            }
        }

        // ================= scanlines, vignette, glass =================
        nvgBeginPath(vg);
        for (int y = 0; y < (int) H; y += 3)
            nvgRect(vg, 0, y + 1, W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H);
        nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, W * 0.48f, W * 0.90f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 110)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H * 0.30f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.30f,
                                           nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        nvgRestore(vg);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ----------------------------------------------------------------------------
// Module widget
// ----------------------------------------------------------------------------
struct ExpressArpeggiatorWidget : ModuleWidget
{
    ExpressArpeggiatorWidget(ExpressArpeggiator *module)
    {
        setModule(module);
        box.size = Vec(EA_W, EA_H);

        addChild(new EAPanel());
        addChild(new EAKnobArcs(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new EADisplay(module));

        // Knobs
        for (int k = 0; k < 10; ++k) {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(EA_KNOBS[k].x, EA_KNOBS[k].y), module, EA_KNOBS[k].param));
        }

        // I/O row: 5 inputs, randomize button, 4 outputs
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(0), EA_IO_Y), module, ExpressArpeggiator::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(1), EA_IO_Y), module, ExpressArpeggiator::RESET_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(2), EA_IO_Y), module, ExpressArpeggiator::RANDOM_TRIGGER_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(3), EA_IO_Y), module, ExpressArpeggiator::TRANSPOSE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(4), EA_IO_Y), module, ExpressArpeggiator::MUT_CV_INPUT));

        addParam(createParamCentered<LEDButton>(Vec(eaIoX(5), EA_IO_Y), module, ExpressArpeggiator::RANDOMIZE_BUTTON));
        addChild(createLightCentered<MediumLight<EACyanLight>>(Vec(eaIoX(5), EA_IO_Y), module, ExpressArpeggiator::RANDOM_LIGHT));

        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(eaIoX(6), EA_IO_Y), module, ExpressArpeggiator::VOCT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(eaIoX(7), EA_IO_Y), module, ExpressArpeggiator::GATE_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(eaIoX(8), EA_IO_Y), module, ExpressArpeggiator::CYCLE_GATE_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(eaIoX(9), EA_IO_Y), module, ExpressArpeggiator::STEP_OUTPUT));
    }

    void appendContextMenu(ui::Menu *menu) override
    {
        ExpressArpeggiator *module = dynamic_cast<ExpressArpeggiator *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createMenuItem("Lock all steps", "", [=]() {
            for (int i = 0; i < 16; ++i) module->locked[i] = true;
        }));
        menu->addChild(createMenuItem("Unlock all steps", "", [=]() {
            for (int i = 0; i < 16; ++i) module->locked[i] = false;
        }));
        menu->addChild(createMenuItem("Unmute all steps", "", [=]() {
            for (int i = 0; i < 16; ++i) module->muted[i] = false;
        }));
        menu->addChild(createMenuItem("Randomize unlocked steps", "", [=]() {
            module->randomizePattern();
        }));
        menu->addChild(createMenuItem("New pattern (clears locks and mutes)", "", [=]() {
            module->initPattern();
        }));
    }
};
//...
Model *modelExpressArpeggiator = createModel<ExpressArpeggiator, ExpressArpeggiatorWidget>("ExpressArpeggiator");