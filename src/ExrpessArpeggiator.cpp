#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout (28 HP = 420 x 380)
// Every control sits in a plate with its label ABOVE it.
// ------------------------------------------------------------
static const float EA_W = RACK_GRID_WIDTH * 28; // 420
static const float EA_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it)
static const float EA_DX = 75.f, EA_DY = 32.f, EA_DW = 270.f, EA_DH = 176.f;

// Rows below the display
static const float EA_HEAD_Y = 40.f; // headers of the two side blocks
static const float EA_CTRL_SEC_Y = 224.f;
static const float EA_IO_SEC_Y = 302.f;
static const float EA_IO_TOP = 312.f, EA_IO_W = 34.f, EA_IO_H = 44.f;
static const float EA_IO_Y = EA_IO_TOP + 31.f; // jack centre (label above)
static float eaIoX(int i) { return 39.f + 38.f * (float)i; }

// Display geometry (display-local coordinates), top to bottom:
//   y  9      readouts
//   20-78     pitch bars (octave names in the left column)
//   87        note names
//   94-102    gate / mute row
//   105-113   lock row
//   122       step numbers
//   132-150   keyboard | direction chips (y 132) | status chips (y 148)
//   168       help / position caption
static const float EA_BX0 = 26.f;    // left edge of step columns
static const float EA_CW = 15.f;     // column pitch (16 x 15 = 240)
static const float EA_BY0 = 20.f;    // top of bars
static const float EA_BH = 58.f;     // bar area height
static const float EA_NOTE_Y = 87.f; // note name row
static const float EA_GATE_Y = 94.f, EA_GATE_H = 8.f;
static const float EA_LOCK_Y = 105.f, EA_LOCK_H = 8.f;
static const float EA_NUM_Y = 122.f;
static const float EA_KB_X0 = 8.f, EA_KB_X1 = 126.f, EA_KB_Y = 132.f;
static const float EA_KB_WH = 18.f, EA_KB_BH = 11.f, EA_KB_BW = 9.f;
static const float EA_CHIP_X = 136.f, EA_CHIP_P = 26.f, EA_CHIP_W = 24.f;
static const float EA_CHIP_Y = 132.f, EA_CHIP_H = 12.f;
static const float EA_STAT_Y = 148.f, EA_STAT_P = 33.f, EA_STAT_W = 30.f, EA_STAT_H = 12.f;
static const float EA_CAP_Y = 168.f;

static inline int eaMod12(int a) { return ((a % 12) + 12) % 12; }

static const char *const EA_NN[12] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

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
    int noteSemi[16] = {}; // semitones relative to C4
    bool locked[16] = {};  // protected from randomize / mutation
    bool muted[16] = {};   // always a rest
    int numSteps = 4;
    int currentStep = 0;
    int dirSign = 1;
    bool playing = false; // false = next clock plays the first step
    bool curRest = false;
    uint32_t stepCounter = 0;

    int lastRoot = -1, lastScale = -1, lastSpan = -1;
    uint16_t scaleMask = 0; // absolute pitch classes in the scale

    // ---- audio state ----
    float targetV = 0.f, outV = 0.f;
    bool outInit = false;
    float gateRemaining = 0.f;
    float sinceClock = 0.f, clockPeriod = 0.25f;
    bool clockSeen = false;

    // ---- display data (torn reads are only cosmetic) ----
    float stepGlow[16] = {};
    float eocFlash = 0.f, restFlash = 0.f, mutFlash = 0.f, randFlash = 0.f;
    int mutIdx = -1;
    int playPc = -1;
    int vTranspose = 0;

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
            if (inScale(s - d, root, iv))
            {
                best = s - d;
                break;
            }
            if (inScale(s + d, root, iv))
            {
                best = s + d;
                break;
            }
        }
        int lo = root, hi = root + 12 * span - 1;
        while (best < lo)
            best += 12;
        while (best > hi)
            best -= 12;
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

    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
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
            if (currentStep >= n)
            {
                currentStep = 0;
                eoc = true;
            }
            break;
        case 1:
            currentStep--;
            if (currentStep < 0)
            {
                currentStep = n - 1;
                eoc = true;
            }
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
            if (n <= 1)
            {
                currentStep = 0;
                eoc = true;
            }
            else
            {
                currentStep += dirSign;
                if (currentStep >= n)
                {
                    currentStep = n - 2;
                    dirSign = -1;
                }
                else if (currentStep < 0)
                {
                    currentStep = 1;
                    dirSign = 1;
                    eoc = true;
                }
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
        float tau = (glide < 0.002f) ? 0.f : 0.002f * std::pow(250.f, glide); // 2 ms .. 500 ms
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
                if (v && json_is_integer(v))
                    noteSemi[i] = rack::math::clamp((int)json_integer_value(v), 0, 59);
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
// UI (anonymous namespace: names cannot clash with other module files)
// ----------------------------------------------------------------------------
namespace
{

    const int EA_WPC[7] = {0, 2, 4, 5, 7, 9, 11}; // white keys
    const int EA_BPC[5] = {1, 3, 6, 8, 10};       // black keys
    const int EA_BIDX[5] = {0, 1, 3, 4, 5};       // white key left of each black key

    // Knob table: parameter, centre x, plate top, plate width, label.
    // The knob centre is always 34 px below the plate top (label sits above it).
    struct EAKnob
    {
        int param;
        float x, top, w;
        const char *label;
    };
    const float EA_KNOB_DY = 34.f, EA_KNOB_H = 56.f;
    const EAKnob EA_KNOBS[10] = {
        {ExpressArpeggiator::ROOT_NOTE_PARAM, 38.f, 50.f, 52.f, "KEY"},
        {ExpressArpeggiator::SCALE_PARAM, 38.f, 116.f, 52.f, "SCALE"},
        {ExpressArpeggiator::STEPS_PARAM, 384.f, 50.f, 52.f, "STEPS"},
        {ExpressArpeggiator::DIRECTION_PARAM, 384.f, 116.f, 52.f, "MOVE"},
        {ExpressArpeggiator::MUTATION_PARAM, 50.f, 234.f, 56.f, "MUTATE"},
        {ExpressArpeggiator::OCTAVES_PARAM, 114.f, 234.f, 56.f, "RANGE"},
        {ExpressArpeggiator::GATE_LEN_PARAM, 178.f, 234.f, 56.f, "GATE"},
        {ExpressArpeggiator::REST_PARAM, 242.f, 234.f, 56.f, "REST"},
        {ExpressArpeggiator::GLIDE_PARAM, 306.f, 234.f, 56.f, "GLIDE"},
        {ExpressArpeggiator::TRANSPOSE_PARAM, 370.f, 234.f, 56.f, "TRANS"},
    };

    // Section header for a block narrower than the panel
    void eaBlockHeader(NVGcontext *vg, float x0, float w, float y, const char *txt)
    {
        nvgSave(vg);
        nvgTranslate(vg, x0, 0.f);
        agmm::sectionHeader(vg, w, y, txt, 0.f);
        nvgRestore(vg);
    }

    // Small padlock glyph (flat)
    void drawLock(NVGcontext *vg, float cx, float cy, bool on)
    {
        NVGcolor col = on ? agmm::violet() : agmm::dim();
        float al = on ? 0.95f : 0.35f;
        float sy = on ? cy - 0.5f : cy - 2.2f;
        nvgBeginPath(vg);
        nvgArc(vg, cx, sy, 2.1f, (float)M_PI, 2.f * (float)M_PI, NVG_CW);
        nvgStrokeColor(vg, agmm::withA(col, al));
        nvgStrokeWidth(vg, 1.1f);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - 3.f, cy - 0.5f, 6.f, 4.6f, 1.f);
        nvgFillColor(vg, agmm::withA(col, al));
        nvgFill(vg);
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct EAPanel : TransparentWidget
    {
        EAPanel() { box.size = Vec(EA_W, EA_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, EA_W, EA_H, "EXPRESSARP");
            agmm::bezel(vg, EA_DX, EA_DY, EA_DW, EA_DH);

            // ---- side blocks and control row: plate + label above each knob ----
            eaBlockHeader(vg, 10.f, 56.f, EA_HEAD_Y, "PITCH");
            eaBlockHeader(vg, 356.f, 56.f, EA_HEAD_Y, "SEQ");
            agmm::sectionHeader(vg, EA_W, EA_CTRL_SEC_Y, "CONTROL", 12.f);
            for (int k = 0; k < 10; ++k)
            {
                const EAKnob &kn = EA_KNOBS[k];
                agmm::cell(vg, kn.x, kn.top, kn.w, EA_KNOB_H, kn.label, false, 8.f);
            }

            // ---- I/O row: inputs left, randomize button, outputs right (tinted) ----
            agmm::sectionHeader(vg, EA_W, EA_IO_SEC_Y, "I / O", 12.f);
            const char *io[10] = {"CLOCK", "RESET", "RND IN", "TRN CV", "MUT CV",
                                  "RANDOM", "V/OCT", "GATE", "EOC", "STEP"};
            for (int i = 0; i < 10; ++i)
                agmm::cell(vg, eaIoX(i), EA_IO_TOP, EA_IO_W, EA_IO_H, io[i], i >= 6, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Pattern display (interactive)
    // ------------------------------------------------------------
    enum
    {
        Z_NONE = 0,
        Z_BARS,
        Z_GATE,
        Z_LOCK,
        Z_NUM,
        Z_DIR
    };

    struct EADisplay : Widget
    {
        ExpressArpeggiator *module = nullptr;
        int hoverZone = Z_NONE, hoverIdx = -1;
        bool editing = false;
        Vec dragPos;

        EADisplay(ExpressArpeggiator *m) : module(m)
        {
            box.pos = Vec(EA_DX, EA_DY);
            box.size = Vec(EA_DW, EA_DH);
        }

        // ---- hit testing ----
        int zoneAt(Vec p, int &idx) const
        {
            idx = -1;
            if (p.x >= EA_BX0 && p.x < EA_BX0 + 16.f * EA_CW)
            {
                int c = rack::math::clamp((int)std::floor((p.x - EA_BX0) / EA_CW), 0, 15);
                if (p.y >= EA_BY0 - 3.f && p.y <= EA_BY0 + EA_BH + 2.f)
                {
                    idx = c;
                    return Z_BARS;
                }
                if (p.y >= EA_GATE_Y - 1.f && p.y <= EA_GATE_Y + EA_GATE_H + 1.f)
                {
                    idx = c;
                    return Z_GATE;
                }
                if (p.y >= EA_LOCK_Y - 1.f && p.y <= EA_LOCK_Y + EA_LOCK_H + 1.f)
                {
                    idx = c;
                    return Z_LOCK;
                }
                if (p.y >= EA_NUM_Y - 4.5f && p.y <= EA_NUM_Y + 4.5f)
                {
                    idx = c;
                    return Z_NUM;
                }
            }
            if (p.y >= EA_CHIP_Y && p.y <= EA_CHIP_Y + EA_CHIP_H && p.x >= EA_CHIP_X)
            {
                float rel = p.x - EA_CHIP_X;
                int i = (int)std::floor(rel / EA_CHIP_P);
                if (i >= 0 && i < 5 && rel - (float)i * EA_CHIP_P <= EA_CHIP_W)
                {
                    idx = i;
                    return Z_DIR;
                }
            }
            return Z_NONE;
        }

        // ---- paint a note from the pointer position ----
        void applyEdit(Vec p)
        {
            if (!module)
                return;
            int c = rack::math::clamp((int)std::floor((p.x - EA_BX0) / EA_CW), 0, 15);
            if (c >= module->numSteps)
                return;
            float t = rack::math::clamp((EA_BY0 + EA_BH - p.y) / EA_BH, 0.f, 1.f);
            int root = module->getRoot(), span = module->getSpan();
            float semiF = (float)root + t * 12.f * (float)span - 1.f;
            module->noteSemi[c] = module->quantizeToScale(semiF);
            module->stepGlow[c] = 1.f;
        }

        // ---- mouse ----
        void onButton(const ButtonEvent &e) override
        {
            if (module && e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS)
            {
                int idx;
                int z = zoneAt(e.pos, idx);
                if (z != Z_NONE)
                {
                    switch (z)
                    {
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

        void onDragMove(const DragMoveEvent &e) override
        {
            if (!editing || !module)
                return;
            dragPos = dragPos.plus(e.mouseDelta.div(getAbsoluteZoom()));
            applyEdit(dragPos);
        }

        void onDragEnd(const DragEndEvent &e) override
        {
            editing = false;
            Widget::onDragEnd(e);
        }

        void onHover(const HoverEvent &e) override
        {
            int idx;
            int z = zoneAt(e.pos, idx);
            hoverZone = z;
            hoverIdx = idx;
            if (z != Z_NONE)
                e.consume(this);
            else
                Widget::onHover(e);
        }

        void onLeave(const LeaveEvent &e) override
        {
            hoverZone = Z_NONE;
            hoverIdx = -1;
            Widget::onLeave(e);
        }

        // ---- static background: flat screen + row labels ----
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, EA_DW, EA_DH);

            // row labels live in the left column, never over the bars
            agmm::text(vg, EA_BX0 - 3.f, EA_GATE_Y + EA_GATE_H / 2.f + 0.5f, "GATE", 7.f, agmm::dim(), agmm::ALIGN_R, 0.3f);
            agmm::text(vg, EA_BX0 - 3.f, EA_LOCK_Y + EA_LOCK_H / 2.f + 0.5f, "LOCK", 7.f, agmm::dim(), agmm::ALIGN_R, 0.3f);
            agmm::text(vg, EA_BX0 - 3.f, EA_NUM_Y, "STEP", 7.f, agmm::dim(), agmm::ALIGN_R, 0.3f);

            Widget::draw(args);
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

            if (module)
            {
                n = module->numSteps;
                cur = module->currentStep;
                playing = module->playing;
                curRest = module->curRest;
                gateOn = module->gateRemaining > 0.f;
                for (int i = 0; i < 16; ++i)
                {
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
                dir = (int)std::round(module->params[ExpressArpeggiator::DIRECTION_PARAM].getValue());
                root = (int)std::round(module->params[ExpressArpeggiator::ROOT_NOTE_PARAM].getValue());
                scaleIdx = (int)std::round(module->params[ExpressArpeggiator::SCALE_PARAM].getValue());
                span = (int)std::round(module->params[ExpressArpeggiator::OCTAVES_PARAM].getValue());
                mutP = module->params[ExpressArpeggiator::MUTATION_PARAM].getValue();
                gateLen = module->params[ExpressArpeggiator::GATE_LEN_PARAM].getValue();
            }
            else
            {
                // demo values for the module browser
                static const int demo[16] = {0, 4, 7, 12, 9, 5, 16, 11, 2, 7, 14, 9, 4, 0, 12, 7};
                for (int i = 0; i < 16; ++i)
                {
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
            scaleIdx = rack::math::clamp(scaleIdx, 0, (int)scaleTable().size() - 1);

            const float baseY = EA_BY0 + EA_BH;
            const float rangeSemi = 12.f * (float)span;
            char buf[64];

            nvgSave(vg);
            nvgScissor(vg, 0, 0, W, H);

            // ================= readout row =================
            {
                snprintf(buf, sizeof(buf), "%s4  MUT %.0f%%", EA_NN[root], mutP * 100.f);
                const float rW = agmm::textWidth(vg, buf, 7.f, 0.5f);
                agmm::text(vg, W - 6.f, 9.f, buf, 7.f, nvgRGB(0xff, 0xe6, 0x80), agmm::ALIGN_R, 0.5f);

                char mid[32];
                snprintf(mid, sizeof(mid), "TRN %+d  %dOCT", trans, span);
                const float mW = agmm::textWidth(vg, mid, 7.f, 0.5f);
                const float mCx = W - 6.f - rW - 10.f - mW / 2.f;
                agmm::text(vg, mCx, 9.f, mid, 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);

                // scale name: shorten until it clears the centre readout
                std::string name = scaleTable()[scaleIdx].name;
                const float maxW = (mCx - mW / 2.f) - 6.f - 8.f;
                while (name.size() > 3 && agmm::textWidth(vg, name.c_str(), 7.f, 0.5f) > maxW)
                    name.pop_back();
                agmm::text(vg, 6.f, 9.f, name.c_str(), 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            }

            // ================= column slots =================
            for (int c = 0; c < 16; ++c)
            {
                bool active = (c < n);
                bool isCur = playing && c == cur;
                nvgBeginPath(vg);
                nvgRect(vg, EA_BX0 + (float)c * EA_CW + 1.f, EA_BY0, EA_CW - 2.f, EA_BH);
                float a = active ? (isCur ? 0.12f : 0.035f) : 0.012f;
                nvgFillColor(vg, agmm::withA(isCur ? agmm::accent() : nvgRGB(255, 255, 255), a));
                nvgFill(vg);
            }

            // ================= pitch grid =================
            {
                int count = 0;
                for (int pc = 0; pc < 12; ++pc)
                    if ((mask >> pc) & 1)
                        count++;
                count *= span;

                for (int s = root; s < root + 12 * span; ++s)
                {
                    if (!((mask >> eaMod12(s)) & 1))
                        continue;
                    float f = ((float)(s - root) + 1.f) / rangeSemi;
                    float y = baseY - EA_BH * f;
                    if (count <= 16)
                    {
                        nvgBeginPath(vg);
                        nvgMoveTo(vg, EA_BX0, y + 0.5f);
                        nvgLineTo(vg, EA_BX0 + 16.f * EA_CW, y + 0.5f);
                        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }
                    if (count <= 40)
                    {
                        nvgBeginPath(vg);
                        nvgMoveTo(vg, EA_BX0 - 2.f, y + 0.5f);
                        nvgLineTo(vg, EA_BX0, y + 0.5f);
                        nvgStrokeColor(vg, agmm::withA(agmm::accent(), 0.5f));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }
                }

                // octave lines; their note names sit in the left column
                for (int j = 0; j < span; ++j)
                {
                    float f = (12.f * (float)j + 1.f) / rangeSemi;
                    float y = baseY - EA_BH * f;
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, EA_BX0 - 2.f, y + 0.5f);
                    nvgLineTo(vg, EA_BX0 + 16.f * EA_CW, y + 0.5f);
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, j == 0 ? 50 : 30));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);

                    int semi = root + 12 * j;
                    snprintf(buf, sizeof(buf), "%s%d", EA_NN[eaMod12(semi)], 4 + (int)std::floor((float)semi / 12.f));
                    agmm::text(vg, EA_BX0 - 4.f, y - 4.f, buf, 7.f, agmm::warm(), agmm::ALIGN_R, 0.f);
                }

                // baseline
                nvgBeginPath(vg);
                nvgMoveTo(vg, EA_BX0, baseY + 0.5f);
                nvgLineTo(vg, EA_BX0 + 16.f * EA_CW, baseY + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // pattern end marker
                float ex = EA_BX0 + (float)n * EA_CW;
                nvgBeginPath(vg);
                nvgMoveTo(vg, ex + 0.5f, EA_BY0 - 2.f);
                nvgLineTo(vg, ex + 0.5f, EA_NUM_Y + 5.f);
                nvgStrokeColor(vg, agmm::withA(agmm::violet(), 0.4f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // ================= bars =================
            float tipX[16], tipY[16];
            for (int c = 0; c < 16; ++c)
            {
                float x = EA_BX0 + (float)c * EA_CW + 1.5f, w = EA_CW - 3.f;
                float f = rack::math::clamp(((float)(notes[c] - root) + 1.f) / rangeSemi, 0.f, 1.f);
                float h = std::max(f * EA_BH, 2.f);
                float top = baseY - h;
                tipX[c] = x + w / 2.f;
                tipY[c] = top;

                if (c >= n)
                {
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

                NVGcolor body = mutated ? agmm::violet() : agmm::accent();
                if (isMuted)
                    body = agmm::dim();
                float a = isMuted ? 0.30f : (0.45f + 0.50f * std::max(g, isCur ? 0.5f : 0.f));

                nvgBeginPath(vg);
                nvgRect(vg, x, top, w, h);
                nvgFillColor(vg, agmm::withA(body, a));
                nvgFill(vg);

                // tip cap: orange on the root, violet when locked
                NVGcolor cap = isRootNote ? agmm::warm() : (lk[c] ? agmm::violet() : nvgRGB(255, 255, 255));
                float capA = isMuted ? 0.3f : (isCur ? 0.95f : 0.6f);
                if (isRootNote || lk[c])
                    capA = std::max(capA, 0.85f);
                nvgBeginPath(vg);
                nvgRect(vg, x - 0.5f, top - 0.5f, w + 1.f, 1.8f);
                nvgFillColor(vg, agmm::withA(cap, capA));
                nvgFill(vg);
            }

            // pitch contour through the bar tips (single hairline)
            {
                nvgLineJoin(vg, NVG_ROUND);
                nvgBeginPath(vg);
                for (int c = 0; c < n; ++c)
                {
                    if (c == 0)
                        nvgMoveTo(vg, tipX[c], tipY[c]);
                    else
                        nvgLineTo(vg, tipX[c], tipY[c]);
                }
                nvgStrokeColor(vg, agmm::withA(agmm::bright(), 0.40f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                for (int c = 0; c < n; ++c)
                {
                    bool isCur = playing && (c == cur);
                    nvgBeginPath(vg);
                    nvgCircle(vg, tipX[c], tipY[c], isCur ? 2.3f : 1.3f);
                    nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, isCur ? 0.97f : 0.55f));
                    nvgFill(vg);
                }
            }

            // ================= note names =================
            for (int c = 0; c < n; ++c)
            {
                int s = notes[c];
                snprintf(buf, sizeof(buf), "%s%d", EA_NN[eaMod12(s)], 4 + (int)std::floor((float)s / 12.f));
                bool isCur = playing && (c == cur);
                agmm::text(vg, EA_BX0 + ((float)c + 0.5f) * EA_CW, EA_NOTE_Y, buf, 7.f,
                           isCur ? agmm::bright() : (mt[c] ? agmm::withA(agmm::dim(), 0.5f) : agmm::textCol()),
                           agmm::ALIGN_C, 0.f);
            }

            // ================= gate / mute row =================
            for (int c = 0; c < 16; ++c)
            {
                float x = EA_BX0 + (float)c * EA_CW + 1.5f, w = EA_CW - 3.f;
                bool active = (c < n);
                bool isCur = playing && (c == cur);

                nvgBeginPath(vg);
                nvgRect(vg, x, EA_GATE_Y, w, EA_GATE_H);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, active ? 10 : 4));
                nvgFill(vg);

                if (!active)
                    continue;

                if (mt[c])
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, x + 2.f, EA_GATE_Y + EA_GATE_H - 1.5f);
                    nvgLineTo(vg, x + w - 2.f, EA_GATE_Y + 1.5f);
                    nvgStrokeColor(vg, agmm::withA(agmm::warm(), 0.8f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                else
                {
                    float fw = std::max(2.f, w * gateLen);
                    float al = 0.40f + 0.45f * glow[c];
                    nvgBeginPath(vg);
                    nvgRect(vg, x, EA_GATE_Y, fw, EA_GATE_H);
                    if (isCur && curRest)
                        nvgFillColor(vg, agmm::withA(agmm::warm(), 0.75f));
                    else
                        nvgFillColor(vg, agmm::withA(agmm::accent(), al));
                    nvgFill(vg);

                    if (isCur && gateOn && !curRest)
                    {
                        nvgBeginPath(vg);
                        nvgRect(vg, x - 0.5f, EA_GATE_Y - 0.5f, w + 1.f, EA_GATE_H + 1.f);
                        nvgStrokeColor(vg, agmm::withA(agmm::bright(), 0.9f));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }
                }
                if (isCur && curRest && !mt[c])
                    agmm::text(vg, x + w / 2.f, EA_GATE_Y + EA_GATE_H / 2.f + 0.5f, "R", 7.f,
                               nvgRGB(0x10, 0x10, 0x14), agmm::ALIGN_C, 0.f);
            }

            // ================= lock row =================
            for (int c = 0; c < n; ++c)
                drawLock(vg, EA_BX0 + ((float)c + 0.5f) * EA_CW, EA_LOCK_Y + 4.f, lk[c]);

            // ================= step numbers + playhead =================
            for (int c = 0; c < 16; ++c)
            {
                float cx = EA_BX0 + ((float)c + 0.5f) * EA_CW;
                bool active = (c < n);
                bool isCur = playing && (c == cur);
                snprintf(buf, sizeof(buf), "%d", c + 1);

                if (isCur)
                {
                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, cx - 6.f, EA_NUM_Y - 4.5f, 12.f, 9.f, 2.f);
                    nvgStrokeColor(vg, agmm::withA(agmm::accent(), 0.85f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                agmm::text(vg, cx, EA_NUM_Y + 0.5f, buf, 7.f,
                           isCur ? agmm::bright() : (active ? agmm::dim() : agmm::withA(agmm::dim(), 0.35f)),
                           agmm::ALIGN_C, 0.f);
            }

            // hover outline
            if (hoverZone != Z_NONE && hoverZone != Z_DIR && hoverIdx >= 0)
            {
                float x = EA_BX0 + (float)hoverIdx * EA_CW + 0.5f;
                float y0 = 0.f, h = 0.f;
                if (hoverZone == Z_BARS)
                {
                    y0 = EA_BY0 - 1.f;
                    h = EA_BH + 2.f;
                }
                else if (hoverZone == Z_GATE)
                {
                    y0 = EA_GATE_Y - 1.f;
                    h = EA_GATE_H + 2.f;
                }
                else if (hoverZone == Z_LOCK)
                {
                    y0 = EA_LOCK_Y - 1.f;
                    h = EA_LOCK_H + 2.f;
                }
                else
                {
                    y0 = EA_NUM_Y - 5.f;
                    h = 10.f;
                }
                nvgBeginPath(vg);
                nvgRect(vg, x, y0, EA_CW - 1.f, h);
                nvgStrokeColor(vg, agmm::withA(agmm::bright(), 0.55f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // ================= mini keyboard (flat) =================
            {
                const float ww = (EA_KB_X1 - EA_KB_X0) / 7.f;

                auto drawKey = [&](float x, float y, float w, float h, int pc, bool black)
                {
                    bool in = (mask >> pc) & 1;
                    bool isRoot = (pc == root);
                    bool sounding = (playing && !curRest && pc == playPc);

                    nvgBeginPath(vg);
                    nvgRect(vg, x, y, w, h);
                    nvgFillColor(vg, black ? nvgRGB(0x08, 0x0a, 0x0e) : nvgRGB(0x16, 0x1c, 0x24));
                    nvgFill(vg);

                    if (in)
                    {
                        nvgBeginPath(vg);
                        nvgRect(vg, x, y, w, h);
                        nvgFillColor(vg, agmm::withA(isRoot ? agmm::warm() : agmm::accent(), 0.30f));
                        nvgFill(vg);
                    }
                    if (sounding)
                    {
                        nvgBeginPath(vg);
                        nvgRect(vg, x, y, w, h);
                        nvgFillColor(vg, nvgRGBAf(0.80f, 0.97f, 1.f, 0.65f));
                        nvgFill(vg);
                    }
                    nvgBeginPath(vg);
                    nvgRect(vg, x, y, w, h);
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, black ? 34 : 44));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);

                    if (isRoot)
                    {
                        nvgBeginPath(vg);
                        nvgRect(vg, x + 2.f, y + h - 3.f, w - 4.f, 1.8f);
                        nvgFillColor(vg, agmm::withA(agmm::warm(), 0.95f));
                        nvgFill(vg);
                    }
                };

                for (int i = 0; i < 7; ++i)
                    drawKey(EA_KB_X0 + (float)i * ww + 0.5f, EA_KB_Y, ww - 1.f, EA_KB_WH, EA_WPC[i], false);
                for (int j = 0; j < 5; ++j)
                {
                    float bx = EA_KB_X0 + (float)(EA_BIDX[j] + 1) * ww - EA_KB_BW / 2.f;
                    drawKey(bx, EA_KB_Y, EA_KB_BW, EA_KB_BH, EA_BPC[j], true);
                }
            }

            // ================= direction chips =================
            {
                const char *dn[5] = {"FWD", "BWD", "RND", "DRK", "PND"};
                for (int i = 0; i < 5; ++i)
                {
                    float x = EA_CHIP_X + (float)i * EA_CHIP_P;
                    bool on = (i == dir);
                    agmm::chip(vg, x, EA_CHIP_Y, EA_CHIP_W, EA_CHIP_H, dn[i], on, agmm::accent());
                    if (!on && hoverZone == Z_DIR && hoverIdx == i)
                    {
                        nvgBeginPath(vg);
                        nvgRoundedRect(vg, x, EA_CHIP_Y, EA_CHIP_W, EA_CHIP_H, 3.f);
                        nvgStrokeColor(vg, agmm::withA(agmm::bright(), 0.7f));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }
                }
            }

            // ================= status chips =================
            {
                const char *sn[4] = {"GATE", "EOC", "REST", "MUT"};
                float lit[4] = {gateOn && playing && !curRest ? 1.f : 0.f, eocF, restF, mutF};
                NVGcolor sc[4] = {agmm::accent(), agmm::violet(), agmm::warm(), agmm::violet()};
                for (int i = 0; i < 4; ++i)
                    agmm::chip(vg, EA_CHIP_X + (float)i * EA_STAT_P, EA_STAT_Y, EA_STAT_W, EA_STAT_H,
                               sn[i], lit[i] > 0.02f, sc[i]);
            }

            // ================= caption row (help while hovering) =================
            {
                const char *help = nullptr;
                switch (hoverZone)
                {
                case Z_BARS:
                    help = "DRAG: PAINT NOTES";
                    break;
                case Z_GATE:
                    help = "CLICK: MUTE STEP";
                    break;
                case Z_LOCK:
                    help = "CLICK: LOCK STEP";
                    break;
                case Z_NUM:
                    help = "CLICK: SET LENGTH";
                    break;
                case Z_DIR:
                    help = "CLICK: SET DIRECTION";
                    break;
                default:
                    break;
                }
                if (help)
                {
                    agmm::text(vg, EA_KB_X0, EA_CAP_Y, help, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "STEP %02d/%02d", playing ? cur + 1 : 0, n);
                    agmm::text(vg, EA_KB_X0, EA_CAP_Y, buf, 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
                }
            }

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

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new EADisplay(module));

        // Knobs: centre is EA_KNOB_DY below the plate top
        for (int k = 0; k < 10; ++k)
        {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(EA_KNOBS[k].x, EA_KNOBS[k].top + EA_KNOB_DY), module, EA_KNOBS[k].param));
        }

        // I/O row: 5 inputs, randomize button, 4 outputs
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(0), EA_IO_Y), module, ExpressArpeggiator::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(1), EA_IO_Y), module, ExpressArpeggiator::RESET_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(2), EA_IO_Y), module, ExpressArpeggiator::RANDOM_TRIGGER_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(3), EA_IO_Y), module, ExpressArpeggiator::TRANSPOSE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaIoX(4), EA_IO_Y), module, ExpressArpeggiator::MUT_CV_INPUT));

        addParam(createParamCentered<LEDButton>(Vec(eaIoX(5), EA_IO_Y), module, ExpressArpeggiator::RANDOMIZE_BUTTON));
        addChild(createLightCentered<MediumLight<agmm::CyanLight>>(Vec(eaIoX(5), EA_IO_Y), module, ExpressArpeggiator::RANDOM_LIGHT));

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

        menu->addChild(createMenuItem("Lock all steps", "", [=]()
                                      {
            for (int i = 0; i < 16; ++i) module->locked[i] = true; }));
        menu->addChild(createMenuItem("Unlock all steps", "", [=]()
                                      {
            for (int i = 0; i < 16; ++i) module->locked[i] = false; }));
        menu->addChild(createMenuItem("Unmute all steps", "", [=]()
                                      {
            for (int i = 0; i < 16; ++i) module->muted[i] = false; }));
        menu->addChild(createMenuItem("Randomize unlocked steps", "", [=]()
                                      { module->randomizePattern(); }));
        menu->addChild(createMenuItem("New pattern (clears locks and mutes)", "", [=]()
                                      { module->initPattern(); }));
    }
};

Model *modelExpressArpeggiator = createModel<ExpressArpeggiator, ExpressArpeggiatorWidget>("ExpressArpeggiator");