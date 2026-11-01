#include "plugin.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

static const float EA_W = RACK_GRID_WIDTH * 16;   // 240
static const float EA_H = RACK_GRID_HEIGHT;       // 380
static const float EA_DX = 20.f, EA_DY = 46.f, EA_DW = 200.f, EA_DH = 100.f;
static const float EA_R1 = 196.f, EA_R2 = 262.f, EA_IOY = 338.f;
static float eaCol(int i) { return 33.f + 58.f * i; }

// ----------------------------------------------------------------------------
// Scale data: 100+ scales with names and semitone intervals from root
// ----------------------------------------------------------------------------
struct ScaleDef
{
    std::string name;
    std::vector<int> intervals; // semitone offsets from root, 0 = root
};

static std::vector<ScaleDef> getScales()
{
    std::vector<ScaleDef> scales;

    auto addScale = [&](const std::string &name, const std::vector<int> &intervals)
    {
        scales.push_back({name, intervals});
    };

    // ----- Common scales (most popular first) -----
    addScale("Major (Ionian)", {0, 2, 4, 5, 7, 9, 11});
    addScale("Natural Minor (Aeolian)", {0, 2, 3, 5, 7, 8, 10});
    addScale("Harmonic Minor", {0, 2, 3, 5, 7, 8, 11});
    addScale("Melodic Minor (ascending)", {0, 2, 3, 5, 7, 9, 11});
    addScale("Pentatonic Major", {0, 2, 4, 7, 9});
    addScale("Pentatonic Minor", {0, 3, 5, 7, 10});
    addScale("Blues", {0, 3, 5, 6, 7, 10});
    addScale("Dorian", {0, 2, 3, 5, 7, 9, 10});
    addScale("Phrygian", {0, 1, 3, 5, 7, 8, 10});
    addScale("Lydian", {0, 2, 4, 6, 7, 9, 11});
    addScale("Mixolydian", {0, 2, 4, 5, 7, 9, 10});
    addScale("Locrian", {0, 1, 3, 5, 6, 8, 10});

    // Modes of harmonic minor
    addScale("Harmonic Minor Mode 2 (Locrian #6)", {0, 1, 3, 5, 6, 9, 10});
    addScale("Harmonic Minor Mode 3 (Ionian #5)", {0, 2, 4, 5, 8, 9, 11});
    addScale("Harmonic Minor Mode 4 (Dorian #11)", {0, 2, 3, 5, 7, 9, 11});
    addScale("Harmonic Minor Mode 5 (Phrygian Dominant)", {0, 1, 3, 5, 7, 8, 10});
    addScale("Harmonic Minor Mode 6 (Lydian #2)", {0, 2, 4, 6, 8, 9, 11});
    addScale("Harmonic Minor Mode 7 (Superlocrian)", {0, 1, 3, 4, 6, 8, 10});

    // Modes of melodic minor
    addScale("Melodic Minor Mode 2 (Dorian b2)", {0, 1, 3, 5, 7, 9, 10});
    addScale("Melodic Minor Mode 3 (Lydian #5)", {0, 2, 4, 6, 8, 9, 11});
    addScale("Melodic Minor Mode 4 (Lydian b7)", {0, 2, 4, 6, 7, 9, 10});
    addScale("Melodic Minor Mode 5 (Mixolydian b6)", {0, 2, 4, 5, 7, 8, 10});
    addScale("Melodic Minor Mode 6 (Aeolian b5)", {0, 2, 3, 5, 6, 8, 10});
    addScale("Melodic Minor Mode 7 (Superlocrian)", {0, 1, 3, 4, 6, 8, 10});

    // Other common scales
    addScale("Whole Tone", {0, 2, 4, 6, 8, 10});
    addScale("Diminished (Half-Whole)", {0, 1, 3, 4, 6, 7, 9, 10});
    addScale("Diminished (Whole-Half)", {0, 2, 3, 5, 6, 8, 9, 11});
    addScale("Augmented", {0, 3, 4, 7, 8, 11});
    addScale("Major Pentatonic (second mode)", {0, 2, 5, 7, 9});
    addScale("Major Pentatonic (third mode)", {0, 3, 5, 7, 10});
    addScale("Major Pentatonic (fourth mode)", {0, 2, 4, 7, 10});
    addScale("Major Pentatonic (fifth mode)", {0, 2, 5, 7, 11});
    addScale("Minor Pentatonic (second mode)", {0, 2, 3, 7, 9});
    addScale("Minor Pentatonic (third mode)", {0, 2, 5, 7, 8});
    addScale("Minor Pentatonic (fourth mode)", {0, 3, 5, 8, 10});
    addScale("Minor Pentatonic (fifth mode)", {0, 2, 5, 7, 10});

    // Exotic / ethnic scales
    addScale("Hungarian Minor", {0, 2, 3, 6, 7, 8, 11});
    addScale("Romanian Minor", {0, 2, 3, 6, 7, 9, 10});
    addScale("Egyptian", {0, 2, 5, 7, 10});
    addScale("Japanese (Hirajoshi)", {0, 2, 3, 7, 8});
    addScale("Japanese (Kumoi)", {0, 2, 4, 7, 8});
    addScale("Pelog", {0, 1, 3, 5, 7});
    addScale("Slendro", {0, 2, 4, 7, 9});
    addScale("Spanish Gypsy", {0, 1, 3, 4, 5, 7, 8, 10});
    addScale("Persian", {0, 1, 4, 5, 6, 8, 11});
    addScale("Arabian", {0, 1, 4, 5, 7, 8, 11});
    addScale("Enigmatic", {0, 1, 4, 6, 8, 10, 11});
    addScale("Neapolitan Major", {0, 1, 3, 5, 7, 9, 11});
    addScale("Neapolitan Minor", {0, 1, 3, 5, 7, 8, 11});
    addScale("Prometheus", {0, 2, 4, 6, 9, 10});
    addScale("Prometheus Neapolitan", {0, 1, 4, 6, 9, 10});
    addScale("Messiaen's Mode 1 (Whole Tone)", {0, 2, 4, 6, 8, 10});
    addScale("Messiaen's Mode 2 (Octatonic Half-Whole)", {0, 1, 3, 4, 6, 7, 9, 10});
    addScale("Messiaen's Mode 3", {0, 2, 3, 4, 6, 7, 8, 10});
    addScale("Messiaen's Mode 4", {0, 1, 2, 4, 5, 6, 8, 9, 10});
    addScale("Messiaen's Mode 5", {0, 1, 3, 5, 6, 7, 8, 10});

    // Fill to 100+ by repeating some with variations
    while (scales.size() < 100)
    {
        for (size_t i = 0; i < scales.size() && scales.size() < 100; ++i)
        {
            scales.push_back(scales[i]);
            scales.back().name += " (variant)";
        }
    }

    return scales;
}

static const std::vector<ScaleDef> &scaleTable()
{
    static const std::vector<ScaleDef> table = getScales();
    return table;
}

// Tooltip helpers: the root knob shows the note, the scale knob shows the scale name
struct EARootQ : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override
    {
        static const char *n[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
        int i = clamp((int)std::round(getValue()), 0, 11);
        return std::string(n[i]) + "4";
    }
};

struct EAScaleQ : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override
    {
        int i = clamp((int)std::round(getValue()), 0, (int)scaleTable().size() - 1);
        return scaleTable()[i].name;
    }
};

// ----------------------------------------------------------------------------
// Module implementation
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
        PARAMS_LEN
    };
    enum InputId
    {
        CLOCK_INPUT,
        RANDOM_TRIGGER_INPUT,
        RESET_INPUT,            // new
        INPUTS_LEN
    };
    enum OutputId
    {
        VOCT_OUTPUT,
        CYCLE_GATE_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // State (fixed arrays so the display can read them safely)
    float noteVoltages[16] = {};
    int numSteps = 0;
    int currentStep = 0;
    float lastClockVoltage = 0.0f;
    bool patternDirty = true;

    int lastRoot = -1;
    int lastScaleIdx = -1;
    int lastSteps = -1;

    bool lastRandomizeState = false;
    float lastRandomTriggerVoltage = 0.0f;
    float lastResetVoltage = 0.0f;

    int cyclePulseRemaining = 0;
    int stepCounter = 0;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float stepFlash = 0.f, eocFlash = 0.f, mutFlash = 0.f;
    int mutIdx = -1;

    const std::vector<ScaleDef> &scales = scaleTable();

    ExpressArpeggiator()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam<EARootQ>(ROOT_NOTE_PARAM, 0.f, 11.f, 0.f, "Root note");
        paramQuantities[ROOT_NOTE_PARAM]->snapEnabled = true;
        configParam<EAScaleQ>(SCALE_PARAM, 0.f, 99.f, 0.f, "Scale");
        paramQuantities[SCALE_PARAM]->snapEnabled = true;
        configParam(STEPS_PARAM, 2.f, 16.f, 4.f, "Steps");
        paramQuantities[STEPS_PARAM]->snapEnabled = true;
        configSwitch(DIRECTION_PARAM, 0.f, 3.f, 0.f, "Direction", {"Forward", "Backward", "Random", "Drunk"});
        configParam(MUTATION_PARAM, 0.f, 1.f, 0.f, "Mutation probability", "%", 0.f, 100.f);
        configButton(RANDOMIZE_BUTTON, "Randomize pattern");

        configInput(CLOCK_INPUT, "Clock");
        configInput(RANDOM_TRIGGER_INPUT, "Random trigger");
        configInput(RESET_INPUT, "Reset (back to step 1)");
        configOutput(VOCT_OUTPUT, "V/Oct");
        configOutput(CYCLE_GATE_OUTPUT, "End of cycle");

        regeneratePattern();
    }

    static int clampInt(int x, int minVal, int maxVal)
    {
        if (x < minVal)
            return minVal;
        if (x > maxVal)
            return maxVal;
        return x;
    }

    int rootParamToMidi(int rootParam) const
    {
        return 60 + rootParam;
    }

    const std::vector<int> &getCurrentScaleIntervals()
    {
        int idx = (int)params[SCALE_PARAM].getValue();
        idx = clampInt(idx, 0, (int)scales.size() - 1);
        return scales[idx].intervals;
    }

    float randomNoteFromScale()
    {
        const auto &intervals = getCurrentScaleIntervals();
        int pick = clampInt((int)std::floor(rack::random::uniform() * intervals.size()), 0, (int)intervals.size() - 1);
        int interval = intervals[pick];
        int rootMidi = rootParamToMidi((int)params[ROOT_NOTE_PARAM].getValue());
        int noteMidi = rootMidi + interval;
        return (noteMidi - 60) / 12.0f;
    }

    // Fully regenerate pattern: new random notes, reset step index
    void regeneratePattern()
    {
        int steps = (int)params[STEPS_PARAM].getValue();
        steps = clampInt(steps, 2, 16);
        numSteps = steps;
        for (int i = 0; i < steps; ++i)
            noteVoltages[i] = randomNoteFromScale();
        currentStep = 0;
        stepCounter = 0;
        patternDirty = false;
        lastRoot = (int)params[ROOT_NOTE_PARAM].getValue();
        lastScaleIdx = (int)params[SCALE_PARAM].getValue();
        lastSteps = steps;
    }

    // Randomize only the note sequence (same length, reset step index)
    void randomizePattern()
    {
        if (numSteps == 0)
            return;
        for (int i = 0; i < numSteps; ++i)
            noteVoltages[i] = randomNoteFromScale();
        currentStep = 0;
        stepCounter = 0;
    }

    void maybeMutate()
    {
        float p = params[MUTATION_PARAM].getValue();
        if (p > 0.0f && rack::random::uniform() < p)
        {
            if (numSteps > 0)
            {
                int idx = clampInt((int)std::floor(rack::random::uniform() * numSteps), 0, numSteps - 1);
                noteVoltages[idx] = randomNoteFromScale();
                mutIdx = idx;
                mutFlash = 1.f;
            }
        }
    }

    void advanceStep()
    {
        int steps = numSteps;
        if (steps == 0)
            return;
        int dir = (int)params[DIRECTION_PARAM].getValue();
        switch (dir)
        {
        case 0:
            currentStep = (currentStep + 1) % steps;
            break;
        case 1:
            currentStep = (currentStep - 1 + steps) % steps;
            break;
        case 2:
            currentStep = clampInt((int)std::floor(rack::random::uniform() * steps), 0, steps - 1);
            break;
        case 3:
        {
            int delta = (int)(rack::random::uniform() * 3) - 1;
            currentStep = (currentStep + delta + steps) % steps;
            break;
        }
        default:
            currentStep = (currentStep + 1) % steps;
            break;
        }
    }

    void process(const ProcessArgs &args) override
    {
        // Auto-regenerate if root/scale/steps changed
        int currentRoot = (int)params[ROOT_NOTE_PARAM].getValue();
        int currentScale = (int)params[SCALE_PARAM].getValue();
        int currentSteps = (int)params[STEPS_PARAM].getValue();
        if (currentRoot != lastRoot || currentScale != lastScaleIdx || currentSteps != lastSteps)
        {
            regeneratePattern();
        }

        // Randomize button (rising edge)
        bool randomizePressed = params[RANDOMIZE_BUTTON].getValue() > 0.5f;
        if (randomizePressed && !lastRandomizeState)
        {
            randomizePattern();
        }
        lastRandomizeState = randomizePressed;

        // External random trigger (rising edge)
        float randomTriggerIn = inputs[RANDOM_TRIGGER_INPUT].getVoltage();
        bool randomTriggerRising = (randomTriggerIn > 0.5f && lastRandomTriggerVoltage <= 0.5f);
        lastRandomTriggerVoltage = randomTriggerIn;
        if (randomTriggerRising)
        {
            randomizePattern();
        }

        // Reset input (rising edge): back to the first step
        float resetIn = inputs[RESET_INPUT].getVoltage();
        if (resetIn > 0.5f && lastResetVoltage <= 0.5f)
        {
            currentStep = 0;
            stepCounter = 0;
        }
        lastResetVoltage = resetIn;

        // Clock input detection
        float clockIn = inputs[CLOCK_INPUT].getVoltage();
        bool clockRising = (clockIn > 0.5f && lastClockVoltage <= 0.5f);
        lastClockVoltage = clockIn;

        if (clockRising && numSteps > 0)
        {
            advanceStep();
            stepCounter++;
            stepFlash = 1.f;

            // End of a full cycle (every N steps)
            if (stepCounter % numSteps == 0)
            {
                cyclePulseRemaining = static_cast<int>(args.sampleRate * 0.001f);
                eocFlash = 1.f;
            }

            outputs[VOCT_OUTPUT].setVoltage(noteVoltages[currentStep]);
            maybeMutate();
        }
        else
        {
            if (outputs[VOCT_OUTPUT].isConnected() && numSteps > 0)
            {
                outputs[VOCT_OUTPUT].setVoltage(noteVoltages[currentStep]);
            }
        }

        // Cycle gate output (one-shot pulse)
        if (cyclePulseRemaining > 0)
        {
            outputs[CYCLE_GATE_OUTPUT].setVoltage(10.0f);
            cyclePulseRemaining--;
        }
        else
        {
            outputs[CYCLE_GATE_OUTPUT].setVoltage(0.0f);
        }

        stepFlash = std::max(0.f, stepFlash - 5.f * args.sampleTime);
        eocFlash = std::max(0.f, eocFlash - 4.f * args.sampleTime);
        mutFlash = std::max(0.f, mutFlash - 2.f * args.sampleTime);
    }
};

namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colViolet()   { return nvgRGB(0xb0, 0x5a, 0xff); }
NVGcolor withAlpha(NVGcolor c, float a) { c.a = a; return c; }

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

void drawLogo(NVGcontext* vg, float cx, float cy) {
    const float w = 64.f, h = 18.f;
    float x = cx - w / 2.f, y = cy - h / 2.f;
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 7.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h, nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
    nvgFill(vg);
    nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
    nvgStrokeWidth(vg, 1.2f);
    nvgStroke(vg);
    const float s = 2.2f;
    float tx = x + 7.f, ty = y + h / 2.f - 3.f;
    nvgBeginPath(vg);
    nvgRect(vg, tx + s, ty, s, s);
    nvgRect(vg, tx, ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);
    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

void drawPanelBase(NVGcontext* vg, float W, float H, const char* title) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
    nvgFill(vg);
    nvgStrokeWidth(vg, 1.f);
    for (int y = 0; y < (int) H; y += 3) {
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0, y + 0.5f);
        nvgLineTo(vg, W, y + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
        nvgStroke(vg);
    }
    nvgBeginPath(vg);
    nvgRect(vg, 0.5f, 0.5f, W - 1.f, H - 1.f);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
    nvgStroke(vg);
    drawLogo(vg, W / 2.f, 13.f);
    drawText(vg, W / 2.f, 31.f, title, 13.f, colText(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);
}

void drawBezel(NVGcontext* vg, float x, float y, float w, float h) {
    float bx = x - 5.f, by = y - 5.f, bw = w + 10.f, bh = h + 10.f;
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh, nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
    nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
    nvgFill(vg);
}

void plate(NVGcontext* vg, float x, float y, float w, float h) {
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 6.f);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 80));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

void ring(NVGcontext* vg, float cx, float cy, float r) {
    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, r);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

void label(NVGcontext* vg, float x, float y, const char* t) {
    drawText(vg, x, y, t, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
}

void topCell(NVGcontext* vg, float cx, float cy, const char* t) {
    plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
    label(vg, cx, cy - 19.f, t);
}

void section(NVGcontext* vg, float W, float y, const char* txt) {
    float x1 = 14.f, x2 = W - 14.f, cx = W / 2.f;
    float w = 10.f + (float) strlen(txt) * 7.f;
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

void screenBase(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 20.f, W * 0.75f, nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
    nvgFill(vg);
}

void screenFinish(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    for (int y = 0; y < (int) H; y += 3) nvgRect(vg, 0, y + 1, W, 1);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H * 0.3f);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f, nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
    nvgFill(vg);
}

struct EAPanel : TransparentWidget {
    EAPanel() { box.size = Vec(EA_W, EA_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, EA_W, EA_H, "EXPRESSARP");
        drawBezel(vg, EA_DX, EA_DY, EA_DW, EA_DH);

        const char* r1[4] = {"KEY", "SCALE", "STEPS", "MOVE"};
        for (int i = 0; i < 4; ++i) {
            ring(vg, eaCol(i), EA_R1, 18.f);
            label(vg, eaCol(i), EA_R1 + 25.f, r1[i]);
        }
        ring(vg, eaCol(0), EA_R2, 18.f);
        label(vg, eaCol(0), EA_R2 + 25.f, "MUTATE");
        plate(vg, eaCol(1) - 17.f, EA_R2 - 17.f, 34.f, 34.f);
        label(vg, eaCol(1), EA_R2 + 25.f, "RANDOM");
        plate(vg, eaCol(2) - 17.f, EA_R2 - 17.f, 34.f, 34.f);
        label(vg, eaCol(2), EA_R2 + 25.f, "RAND IN");
        plate(vg, eaCol(3) - 17.f, EA_R2 - 17.f, 34.f, 34.f);
        label(vg, eaCol(3), EA_R2 + 25.f, "RESET");

        section(vg, EA_W, 300.f, "I / O");
        topCell(vg, eaCol(0), EA_IOY, "CLK");
        topCell(vg, eaCol(1), EA_IOY, "V/OCT");
        topCell(vg, eaCol(2), EA_IOY, "EOC");
        Widget::draw(args);
    }
};

// Display: the 16-step pattern as pitch bars, direction chips, scale and key
struct EADisplay : TransparentWidget {
    ExpressArpeggiator* module = nullptr;
    EADisplay(ExpressArpeggiator* m) : module(m) {
        box.pos = Vec(EA_DX, EA_DY);
        box.size = Vec(EA_DW, EA_DH);
    }

    static constexpr float CW = 12.5f, BY0 = 22.f, BH = 56.f;

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        screenBase(vg, EA_DW, EA_DH);
        for (int c = 0; c < 16; ++c) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, c * CW + 1.5f, BY0, CW - 3.f, BH, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 9));
            nvgFill(vg);
        }
        // grid at 0, 0.5, 1, 1.5 and 2 V
        nvgBeginPath(vg);
        for (int k = 0; k <= 4; ++k) {
            float y = BY0 + BH - k * (BH / 4.f) + 0.5f;
            nvgMoveTo(vg, 0, y);
            nvgLineTo(vg, EA_DW, y);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = EA_DW, H = EA_DH;

        int n = 8, cur = 3, dir = 0, root = 0, scaleIdx = 0, mutIdx = -1;
        float notes[16], flash = 0.6f, eoc = 0.f, mutF = 0.f, mut = 0.2f;
        for (int i = 0; i < 16; ++i) notes[i] = 0.f;
        if (module) {
            n = module->numSteps;
            cur = module->currentStep;
            for (int i = 0; i < 16; ++i) notes[i] = module->noteVoltages[i];
            flash = module->stepFlash; eoc = module->eocFlash; mutF = module->mutFlash;
            mutIdx = module->mutIdx;
            dir = (int) std::round(module->params[ExpressArpeggiator::DIRECTION_PARAM].getValue());
            root = (int) std::round(module->params[ExpressArpeggiator::ROOT_NOTE_PARAM].getValue());
            scaleIdx = (int) std::round(module->params[ExpressArpeggiator::SCALE_PARAM].getValue());
            mut = module->params[ExpressArpeggiator::MUTATION_PARAM].getValue();
        } else {
            static const float demo[8] = {0.f, 0.33f, 0.58f, 0.25f, 0.83f, 0.58f, 1.08f, 0.5f};
            for (int i = 0; i < 8; ++i) notes[i] = demo[i];
        }
        n = std::min(std::max(n, 0), 16);
        dir = std::min(std::max(dir, 0), 3);
        root = std::min(std::max(root, 0), 11);
        scaleIdx = std::min(std::max(scaleIdx, 0), (int) scaleTable().size() - 1);

        for (int c = 0; c < 16; ++c) {
            float x = c * CW + 1.5f, w = CW - 3.f;
            if (c >= n) {
                nvgBeginPath(vg);
                nvgRect(vg, x + 2.f, BY0 + BH - 1.f, w - 4.f, 1.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 22));
                nvgFill(vg);
                continue;
            }
            bool isCur = (c == cur);
            float h = std::max(std::min(std::max(notes[c] / 2.f, 0.f), 1.f) * BH, 2.f);
            float top = BY0 + BH - h;
            bool mutated = (c == mutIdx && mutF > 0.02f);
            NVGcolor hi = mutated ? nvgRGB(0xd8, 0xb0, 0xff) : nvgRGB(0x8a, 0xee, 0xff);
            NVGcolor lo = mutated ? nvgRGB(0x7a, 0x3a, 0xc8) : nvgRGB(0x1f, 0x9f, 0xc0);
            float a = isCur ? 1.f : 0.55f;

            if (isCur) {
                nvgBeginPath(vg);
                nvgRect(vg, x - 1.5f, top - 1.5f, w + 3.f, h + 3.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (int) (40 + 60 * flash)));
                nvgFill(vg);
            }
            nvgBeginPath(vg);
            nvgRect(vg, x, top, w, h);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, BY0 + BH, 0, BY0, withAlpha(lo, a), withAlpha(hi, a)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, x, top - 0.5f, w, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, isCur ? 235 : 130));
            nvgFill(vg);
        }

        // direction chips
        static const char* dirNames[4] = {"FWD", "BWD", "RND", "DRK"};
        for (int i = 0; i < 4; ++i) {
            float x = 8.f + i * 33.f;
            bool on = (i == dir);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, 84.f, 30.f, 11.f, 3.f);
            if (on) {
                nvgFillColor(vg, withAlpha(colAccent(), 0.25f));
                nvgFill(vg);
                nvgStrokeColor(vg, withAlpha(colAccent(), 0.9f));
            } else {
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            }
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, x + 15.f, 89.5f, dirNames[i], 6.f, on ? colAccent() : colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        }
        // EOC chip
        {
            float x = 142.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, 84.f, 24.f, 11.f, 3.f);
            if (eoc > 0.02f) {
                nvgFillColor(vg, withAlpha(colViolet(), 0.3f * eoc + 0.1f));
                nvgFill(vg);
                nvgStrokeColor(vg, withAlpha(colViolet(), 0.9f));
            } else {
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            }
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, x + 12.f, 89.5f, "EOC", 6.f, eoc > 0.02f ? colViolet() : colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        }

        screenFinish(vg, W, H);

        static const char* nn[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
        char buf[64];
        std::string name = scaleTable()[scaleIdx].name;
        if (name.size() > 18) name = name.substr(0, 16) + "..";
        drawText(vg, 6.f, 9.f, name.c_str(), 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "%s4  MUT %.0f%%", nn[root], mut * 100.f);
        drawText(vg, W - 6.f, 9.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "STEP %02d/%02d", n > 0 ? cur + 1 : 0, n);
        drawText(vg, W - 6.f, 89.5f, buf, 7.f, colDim(), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

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

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(eaCol(0), EA_R1), module, ExpressArpeggiator::ROOT_NOTE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(eaCol(1), EA_R1), module, ExpressArpeggiator::SCALE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(eaCol(2), EA_R1), module, ExpressArpeggiator::STEPS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(eaCol(3), EA_R1), module, ExpressArpeggiator::DIRECTION_PARAM));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(eaCol(0), EA_R2), module, ExpressArpeggiator::MUTATION_PARAM));
        addParam(createParamCentered<VCVButton>(Vec(eaCol(1), EA_R2), module, ExpressArpeggiator::RANDOMIZE_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaCol(2), EA_R2), module, ExpressArpeggiator::RANDOM_TRIGGER_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaCol(3), EA_R2), module, ExpressArpeggiator::RESET_INPUT));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(eaCol(0), EA_IOY), module, ExpressArpeggiator::CLOCK_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(eaCol(1), EA_IOY), module, ExpressArpeggiator::VOCT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(eaCol(2), EA_IOY), module, ExpressArpeggiator::CYCLE_GATE_OUTPUT));
    }
};

Model *modelExpressArpeggiator = createModel<ExpressArpeggiator, ExpressArpeggiatorWidget>("ExpressArpeggiator");