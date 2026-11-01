#include "plugin.hpp"
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float PS_PANEL_W = RACK_GRID_WIDTH * 38;   // 570
static const float PS_PANEL_H = RACK_GRID_HEIGHT;       // 380

// 8 x 8 step grid
static const float PS_GRID_X = 136.f;
static const float PS_GRID_Y = 55.f;
static const float PS_GRID_W = 316.f;
static const float PS_GRID_H = 300.f;
static float psCellX(int c) { return PS_GRID_X + c * 40.f + 18.f; }   // cell centre
static float psCellY(int r) { return PS_GRID_Y + r * 38.f + 17.f; }

// Left control block, status screen, right jack block
static const float PS_LCOL[2] = {30.f, 90.f};
static const float PS_LROW[3] = {86.f, 142.f, 198.f};
static const float PS_SCR_X = 14.f, PS_SCR_Y = 240.f, PS_SCR_W = 100.f, PS_SCR_H = 104.f;
static const float PS_RCOL[2] = {490.f, 530.f};
static const float PS_RROW[4] = {100.f, 154.f, 240.f, 294.f};

// ------------------------------------------------------------
// Custom ParamQuantity for wait cycles that snaps to discrete values 0, 4, 8, 16
// ------------------------------------------------------------
struct WaitCyclesParamQuantity : rack::engine::ParamQuantity {
    void setValue(float value) override {
        const std::vector<float> allowed = {0.f, 4.f, 8.f, 16.f};
        float nearest = allowed[0];
        float minDist = std::abs(value - nearest);
        for (size_t i = 1; i < allowed.size(); ++i) {
            float dist = std::abs(value - allowed[i]);
            if (dist < minDist) {
                minDist = dist;
                nearest = allowed[i];
            }
        }
        ParamQuantity::setValue(nearest);
    }
};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct PsySeq : Module
{
    enum ParamId
    {
        STEPS_PARAM,
        START_PARAM,
        DIRECTION_PARAM,
        RANDOM_VOLTAGE_PARAM,
        RANDOMIZE_BUTTON,
        KNOB_OFFSET,
        KNOB_MAX = KNOB_OFFSET + 64,
        MUTATION_PARAM = KNOB_MAX + 1,
        MUTATION_WAIT_PARAM,
        PARAMS_LEN = MUTATION_WAIT_PARAM + 1
    };
    enum InputId
    {
        CLOCK_INPUT,
        RESET_INPUT,
        RANDOMIZE_TRIGGER_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_OUT,
        CYCLE_GATE_OUTPUT,
        STEP_TRIGGER_OUTPUT,    // new
        INV_OUTPUT,             // new
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHT_OFFSET,
        LIGHT_MAX = LIGHT_OFFSET + 64,
        LIGHTS_LEN = LIGHT_MAX
    };

    // State
    int position = 0;
    float voltMin = -1.0f;
    float voltMax = 1.0f;
    float heldVoltage = 0.0f;
    int cycleCounter = 0;

    dsp::SchmittTrigger clockTrigger;
    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger randomizeTrigger;
    dsp::SchmittTrigger randomizeInputTrigger;

    dsp::PulseGenerator cyclePulse;
    dsp::PulseGenerator stepPulse;

    float lastSteps = 8.f;
    float lastStart = 1.f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    int curIndex = 0;
    float stepFlash = 0.f;
    float mutFlash = 0.f;

    PsySeq()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam<rack::engine::ParamQuantity>(STEPS_PARAM, 1.f, 64.f, 8.f, "Steps");
        paramQuantities[STEPS_PARAM]->snapEnabled = true;

        configParam<rack::engine::ParamQuantity>(START_PARAM, 1.f, 64.f, 1.f, "Start step");
        paramQuantities[START_PARAM]->snapEnabled = true;

        configSwitch(DIRECTION_PARAM, 0.f, 2.f, 0.f, "Direction", {"Forward", "Reverse", "Random"});
        configSwitch(RANDOM_VOLTAGE_PARAM, 0.f, 1.f, 0.f, "Random voltage per step", {"Off", "On"});

        configButton(RANDOMIZE_BUTTON, "Randomize all step voltages");

        configParam(MUTATION_PARAM, 0.f, 1.f, 0.f, "Mutation probability", "%", 0.f, 100.f);

        auto waitQuantity = configParam<WaitCyclesParamQuantity>(MUTATION_WAIT_PARAM, 0.f, 16.f, 0.f, "Wait cycles before mutation attempt");
        waitQuantity->snapEnabled = true;

        for (int i = 0; i < 64; i++)
        {
            configParam(KNOB_OFFSET + i, 0.f, 1.f, 0.f, string::f("Step %d", i + 1));
            configLight(LIGHT_OFFSET + i, string::f("Step %d", i + 1));
        }

        configInput(CLOCK_INPUT, "Clock");
        configInput(RESET_INPUT, "Reset");
        configInput(RANDOMIZE_TRIGGER_INPUT, "Randomize trigger");
        configOutput(OUTPUT_OUT, "CV out");
        configOutput(CYCLE_GATE_OUTPUT, "Cycle gate");
        configOutput(STEP_TRIGGER_OUTPUT, "Step trigger (every clock)");
        configOutput(INV_OUTPUT, "Inverted CV out");

        configBypass(CLOCK_INPUT, OUTPUT_OUT);
    }

    // Position inside the sequence -> knob index (0..63), honouring the start step
    int mapIndex(int pos)
    {
        int start = (int)params[START_PARAM].getValue();
        return (((start - 1 + pos) % 64) + 64) % 64;
    }

    float computeVoltageForStep(int stepIndex, bool randomMode)
    {
        if (randomMode)
        {
            return random::uniform() * (voltMax - voltMin) + voltMin;
        }
        else
        {
            float knobValue = params[KNOB_OFFSET + stepIndex].getValue();
            return voltMin + knobValue * (voltMax - voltMin);
        }
    }

    void updateHeldVoltage()
    {
        bool randomMode = params[RANDOM_VOLTAGE_PARAM].getValue() > 0.5f;
        heldVoltage = computeVoltageForStep(mapIndex(position), randomMode);
    }

    void setVoltageRange(float min, float max)
    {
        voltMin = min;
        voltMax = max;
        updateHeldVoltage();
    }

    void randomizeAllStepKnobs()
    {
        for (int i = 0; i < 64; i++)
        {
            params[KNOB_OFFSET + i].setValue(random::uniform());
        }
        updateHeldVoltage();
    }

    void clearAllStepKnobs()
    {
        for (int i = 0; i < 64; i++)
        {
            params[KNOB_OFFSET + i].setValue(0.f);
        }
        updateHeldVoltage();
    }

    json_t *dataToJson() override
    {
        json_t *rootJ = json_object();
        json_object_set_new(rootJ, "voltMin", json_real(voltMin));
        json_object_set_new(rootJ, "voltMax", json_real(voltMax));
        return rootJ;
    }

    void dataFromJson(json_t *rootJ) override
    {
        json_t *minJ = json_object_get(rootJ, "voltMin");
        json_t *maxJ = json_object_get(rootJ, "voltMax");
        if (minJ)
            voltMin = json_real_value(minJ);
        if (maxJ)
            voltMax = json_real_value(maxJ);
        updateHeldVoltage();
    }

    void process(const ProcessArgs &args) override
    {
        // Button randomize
        if (randomizeTrigger.process(params[RANDOMIZE_BUTTON].getValue()))
        {
            randomizeAllStepKnobs();
        }

        // External trigger randomize
        if (randomizeInputTrigger.process(inputs[RANDOMIZE_TRIGGER_INPUT].getVoltage()))
        {
            randomizeAllStepKnobs();
        }

        float stepsParam = params[STEPS_PARAM].getValue();
        float startParam = params[START_PARAM].getValue();
        float direction = params[DIRECTION_PARAM].getValue();

        int steps = (int)stepsParam;

        // Reset cycle counter when steps or start step changes
        if (stepsParam != lastSteps || startParam != lastStart)
        {
            if (position >= steps)
                position = steps - 1;
            if (position < 0)
                position = 0;
            lastSteps = stepsParam;
            lastStart = startParam;
            cycleCounter = 0;
            updateHeldVoltage();
        }

        // Reset input: reset position and cycle counter
        if (resetTrigger.process(inputs[RESET_INPUT].getVoltage()))
        {
            position = 0;
            cycleCounter = 0;
            updateHeldVoltage();
        }

        // Clock step
        if (clockTrigger.process(inputs[CLOCK_INPUT].getVoltage()))
        {
            bool cycleCompleted = false;

            if (direction < 0.5f)   // Forward
            {
                position++;
                if (position >= steps)
                {
                    position = 0;
                    cycleCompleted = true;
                }
            }
            else if (direction < 1.5f)   // Reverse
            {
                position--;
                if (position < 0)
                {
                    position = steps - 1;
                    cycleCompleted = true;
                }
            }
            else   // Random
            {
                if (steps > 0)
                {
                    position = (int)(random::uniform() * steps);
                    if (position >= steps)
                        position = steps - 1;
                }
                else
                {
                    position = 0;
                }
            }

            stepPulse.trigger(5e-3f);
            stepFlash = 1.f;

            // ---- PERIODIC MUTATION ATTEMPT (forward/reverse only) ----
            if (cycleCompleted)
            {
                cycleCounter++;

                float waitCycles = params[MUTATION_WAIT_PARAM].getValue();   // 0,4,8,16
                float mutationProb = params[MUTATION_PARAM].getValue();

                bool shouldAttempt = (waitCycles == 0.0f) || (cycleCounter >= (int)waitCycles);

                if (shouldAttempt)
                {
                    if (random::uniform() < mutationProb)
                    {
                        // Mutate all active steps
                        for (int i = 0; i < steps; i++)
                        {
                            params[KNOB_OFFSET + mapIndex(i)].setValue(random::uniform());
                        }
                        mutFlash = 1.f;
                    }
                    cycleCounter = 0;
                }

                cyclePulse.trigger(1e-3f);
            }

            updateHeldVoltage();
        }

        // Outputs
        outputs[OUTPUT_OUT].setVoltage(heldVoltage);
        outputs[INV_OUTPUT].setVoltage(voltMin + voltMax - heldVoltage);
        outputs[CYCLE_GATE_OUTPUT].setVoltage(cyclePulse.process(args.sampleTime) ? 10.0f : 0.0f);
        outputs[STEP_TRIGGER_OUTPUT].setVoltage(stepPulse.process(args.sampleTime) ? 10.0f : 0.0f);

        // LEDs and display data
        curIndex = mapIndex(position);
        stepFlash = std::max(0.f, stepFlash - 4.f * args.sampleTime);
        mutFlash = std::max(0.f, mutFlash - 2.f * args.sampleTime);
        for (int i = 0; i < 64; i++)
        {
            lights[LIGHT_OFFSET + i].setSmoothBrightness(i == curIndex ? 1.f : 0.f, args.sampleTime);
        }
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace {

// ------------------------------------------------------------
// Theme (same palette as the other AGMM modules)
// ------------------------------------------------------------
NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colViolet()   { return nvgRGB(0xb0, 0x5a, 0xff); }

NVGcolor withAlpha(NVGcolor c, float a) {
    c.a = a;
    return c;
}

void setFont(NVGcontext* vg) {
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font)
        nvgFontFaceId(vg, font->handle);
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

// Cyan LED (Rack has no built-in one)
template <typename TBase = GrayModuleLightWidget>
struct TPSCyanLight : TBase {
    TPSCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using PSCyanLight = TPSCyanLight<>;

// ------------------------------------------------------------
// AGMM logo
// ------------------------------------------------------------
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

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct PSPanelBackground : TransparentWidget {
    PSPanelBackground() { box.size = Vec(PS_PANEL_W, PS_PANEL_H); }

    void label(NVGcontext* vg, float x, float y, const char* txt) {
        drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
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

    void caption(NVGcontext* vg, float cx, float y, const char* txt, NVGcolor col, float x1, float x2) {
        drawText(vg, cx, y, txt, 8.f, col, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y + 8.f);
        nvgLineTo(vg, x2, y + 8.f);
        nvgStrokeColor(vg, withAlpha(col, 0.35f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void topCell(NVGcontext* vg, float cx, float cy, const char* txt) {
        plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
        label(vg, cx, cy - 19.f, txt);
    }
    void switchCell(NVGcontext* vg, float cx, float cy, const char* txt) {
        plate(vg, cx - 17.f, cy - 20.f, 34.f, 40.f);
        label(vg, cx, cy + 25.f, txt);
    }
    void knobCell(NVGcontext* vg, float cx, float cy, float r, const char* txt) {
        ring(vg, cx, cy, r);
        label(vg, cx, cy + 25.f, txt);
    }

    void bezel(NVGcontext* vg, float x, float y, float w, float h) {
        float bx = x - 5.f, by = y - 5.f, bw = w + 10.f, bh = h + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float W = PS_PANEL_W, H = PS_PANEL_H;

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
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        drawLogo(vg, W / 2.f, 13.f);
        drawText(vg, W / 2.f, 31.f, "PSYSEQ", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        bezel(vg, PS_GRID_X, PS_GRID_Y, PS_GRID_W, PS_GRID_H);
        bezel(vg, PS_SCR_X, PS_SCR_Y, PS_SCR_W, PS_SCR_H);

        // ---- left block ----
        caption(vg, 60.f, 54.f, "CONTROL", colAccent(), 12.f, 108.f);
        knobCell(vg, PS_LCOL[0], PS_LROW[0], 18.f, "STEPS");
        knobCell(vg, PS_LCOL[1], PS_LROW[0], 18.f, "START");
        switchCell(vg, PS_LCOL[0], PS_LROW[1], "DIRECTION");
        switchCell(vg, PS_LCOL[1], PS_LROW[1], "RAND V");
        knobCell(vg, PS_LCOL[0], PS_LROW[2], 18.f, "WAIT");
        knobCell(vg, PS_LCOL[1], PS_LROW[2], 18.f, "MUTATE");

        // ---- right block ----
        caption(vg, 510.f, 54.f, "OUTPUT", colViolet(), 473.f, 547.f);
        topCell(vg, PS_RCOL[0], PS_RROW[0], "CV OUT");
        topCell(vg, PS_RCOL[1], PS_RROW[0], "STEP");
        topCell(vg, PS_RCOL[0], PS_RROW[1], "CYCLE");
        topCell(vg, PS_RCOL[1], PS_RROW[1], "CV INV");
        caption(vg, 510.f, 190.f, "INPUT", colAccent(), 473.f, 547.f);
        topCell(vg, PS_RCOL[0], PS_RROW[2], "CLK");
        topCell(vg, PS_RCOL[1], PS_RROW[2], "RESET");
        topCell(vg, PS_RCOL[0], PS_RROW[3], "RAND IN");
        topCell(vg, PS_RCOL[1], PS_RROW[3], "RAND");

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Step grid: cells, active range, playhead, value bars
// ------------------------------------------------------------
struct PSGridWidget : TransparentWidget {
    PsySeq* module = nullptr;

    PSGridWidget(PsySeq* m) : module(m) {
        box.pos = Vec(PS_GRID_X, PS_GRID_Y);
        box.size = Vec(PS_GRID_W, PS_GRID_H);
    }

    // Static background: cell plates and step numbers
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        for (int i = 0; i < 64; ++i) {
            float x = (i % 8) * 40.f, y = (i / 8) * 38.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, y, 36.f, 34.f, 4.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + 34.f,
                                               nvgRGBA(255, 255, 255, 12), nvgRGBA(0, 0, 0, 60)));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", i + 1);
            drawText(vg, x + 3.5f, y + 5.5f, buf, 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.f);
        }
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        int steps, start, cur;
        float flash, mutFlash;
        bool randMode;
        float knob[64];
        if (module) {
            steps = (int) module->params[PsySeq::STEPS_PARAM].getValue();
            start = (int) module->params[PsySeq::START_PARAM].getValue();
            cur = module->curIndex;
            flash = module->stepFlash;
            mutFlash = module->mutFlash;
            randMode = module->params[PsySeq::RANDOM_VOLTAGE_PARAM].getValue() > 0.5f;
            for (int i = 0; i < 64; ++i)
                knob[i] = module->params[PsySeq::KNOB_OFFSET + i].getValue();
        } else {
            steps = 16; start = 1; cur = 5; flash = 0.6f; mutFlash = 0.f; randMode = false;
            for (int i = 0; i < 64; ++i)
                knob[i] = 0.5f + 0.5f * std::sin(i * 0.9f + 0.4f) * std::cos(i * 0.31f);
        }
        steps = std::min(std::max(steps, 1), 64);
        start = std::min(std::max(start, 1), 64);

        for (int i = 0; i < 64; ++i) {
            float x = (i % 8) * 40.f, y = (i / 8) * 38.f;
            int rel = (((i - (start - 1)) % 64) + 64) % 64;
            bool active = rel < steps;

            if (active) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, y, 36.f, 34.f, 4.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
                nvgFill(vg);
                if (mutFlash > 0.02f) {
                    nvgFillColor(vg, withAlpha(colViolet(), 0.35f * mutFlash));
                    nvgFill(vg);
                }
            } else {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, y, 36.f, 34.f, 4.f);
                nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
                nvgFill(vg);
            }

            // Value bar along the bottom
            float kv = std::min(std::max(knob[i], 0.f), 1.f);
            nvgBeginPath(vg);
            nvgRect(vg, x + 4.f, y + 28.f, 28.f, 2.5f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgFill(vg);
            if (kv > 0.005f) {
                nvgBeginPath(vg);
                nvgRect(vg, x + 4.f, y + 28.f, 28.f * kv, 2.5f);
                float a = randMode ? 0.18f : (active ? 0.95f : 0.3f);
                nvgFillPaint(vg, nvgLinearGradient(vg, x + 4.f, 0, x + 32.f, 0,
                                                   withAlpha(colAccent(), a), withAlpha(colViolet(), a)));
                nvgFill(vg);
            }

            // Start marker
            if (i == start - 1) {
                nvgBeginPath(vg);
                nvgMoveTo(vg, x + 1.f, y + 1.f);
                nvgLineTo(vg, x + 9.f, y + 1.f);
                nvgLineTo(vg, x + 1.f, y + 9.f);
                nvgClosePath(vg);
                nvgFillColor(vg, colViolet());
                nvgFill(vg);
            }

            // Playhead
            if (i == cur) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x - 2.f, y - 2.f, 40.f, 38.f, 6.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (int) (30 + 70 * flash)));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, y, 36.f, 34.f, 4.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (int) (25 + 45 * flash)));
                nvgFill(vg);
                nvgStrokeColor(vg, nvgRGB(0x8a, 0xee, 0xff));
                nvgStrokeWidth(vg, 1.4f);
                nvgStroke(vg);
            }
        }

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PS_GRID_W, PS_GRID_H * 0.25f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PS_GRID_H * 0.25f,
                                           nvgRGBA(255, 255, 255, 12), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// Status screen
// ------------------------------------------------------------
struct PSStatusWidget : TransparentWidget {
    PsySeq* module = nullptr;

    PSStatusWidget(PsySeq* m) : module(m) {
        box.pos = Vec(PS_SCR_X, PS_SCR_Y);
        box.size = Vec(PS_SCR_W, PS_SCR_H);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PS_SCR_W, PS_SCR_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, PS_SCR_W / 2.f, PS_SCR_H / 2.f, 15.f, PS_SCR_W * 0.8f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float W = PS_SCR_W, H = PS_SCR_H;

        int steps, pos, dir, wait, cyc;
        float volt, vmin, vmax, mut, flash;
        bool rnd;
        if (module) {
            steps = (int) module->params[PsySeq::STEPS_PARAM].getValue();
            pos = module->position;
            dir = (int) std::round(module->params[PsySeq::DIRECTION_PARAM].getValue());
            wait = (int) std::round(module->params[PsySeq::MUTATION_WAIT_PARAM].getValue());
            cyc = module->cycleCounter;
            volt = module->heldVoltage;
            vmin = module->voltMin; vmax = module->voltMax;
            mut = module->params[PsySeq::MUTATION_PARAM].getValue();
            rnd = module->params[PsySeq::RANDOM_VOLTAGE_PARAM].getValue() > 0.5f;
            flash = module->stepFlash;
        } else {
            steps = 16; pos = 5; dir = 0; wait = 8; cyc = 3;
            volt = 0.42f; vmin = -1.f; vmax = 1.f; mut = 0.4f; rnd = false; flash = 0.f;
        }
        dir = std::min(std::max(dir, 0), 2);
        steps = std::max(steps, 1);

        char buf[48];
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);

        drawText(vg, 6.f, 8.f, "STEP", 6.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "OF %d", steps);
        drawText(vg, W - 6.f, 8.f, buf, 6.5f, colDim(), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "%02d", pos + 1);
        drawText(vg, W / 2.f, 28.f, buf, 22.f, nvgRGBA(0x8a, 0xee, 0xff, (int) (190 + 65 * flash)),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        // Direction and random-voltage chips
        static const char* dirNames[3] = {"FWD", "REV", "RND"};
        auto chip = [&](float x, float w, const char* txt, bool on, NVGcolor col) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, 43.f, w, 11.f, 3.f);
            if (on) {
                nvgFillColor(vg, withAlpha(col, 0.25f));
                nvgFill(vg);
                nvgStrokeColor(vg, withAlpha(col, 0.9f));
            } else {
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            }
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, x + w / 2.f, 48.5f, txt, 6.f, on ? col : colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        };
        chip(6.f, 28.f, dirNames[dir], true, colAccent());
        chip(38.f, 28.f, "RAND V", rnd, colViolet());
        chip(70.f, 24.f, "MUT", mut > 0.001f, colViolet());

        // Voltage meter
        float t = (vmax > vmin) ? (volt - vmin) / (vmax - vmin) : 0.5f;
        t = std::min(std::max(t, 0.f), 1.f);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 8.f, 61.f, 84.f, 5.f, 2.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 8.f, 61.f, std::max(84.f * t, 2.f), 5.f, 2.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 8.f, 0, 92.f, 0, colAccent(), colViolet()));
        nvgFill(vg);
        if (vmin < 0.f && vmax > 0.f) {
            float zx = 8.f + 84.f * (-vmin / (vmax - vmin));
            nvgBeginPath(vg);
            nvgRect(vg, zx, 59.f, 1.f, 9.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 120));
            nvgFill(vg);
        }
        nvgBeginPath(vg);
        nvgRect(vg, 8.f + 84.f * t - 1.f, 58.f, 2.f, 11.f);
        nvgFillColor(vg, nvgRGB(255, 255, 255));
        nvgFill(vg);

        snprintf(buf, sizeof(buf), "%+.2fV", volt);
        drawText(vg, W / 2.f, 77.f, buf, 9.f, nvgRGBA(0xff, 0xe6, 0x80, 230),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "RNG %g/%gV", vmin, vmax);
        drawText(vg, W / 2.f, 89.f, buf, 6.5f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        snprintf(buf, sizeof(buf), "WAIT %d/%d  MUT %.0f%%", cyc, wait, mut * 100.f);
        drawText(vg, W / 2.f, 99.f, buf, 6.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) H; y += 3)
            nvgRect(vg, 0, y + 1, W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H * 0.3f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f,
                                           nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct PsySeqWidget : ModuleWidget
{
    PsySeqWidget(PsySeq *module)
    {
        setModule(module);
        box.size = Vec(PS_PANEL_W, PS_PANEL_H);

        addChild(new PSPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PSGridWidget(module));
        addChild(new PSStatusWidget(module));

        // ---- left block ----
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PS_LCOL[0], PS_LROW[0]), module, PsySeq::STEPS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PS_LCOL[1], PS_LROW[0]), module, PsySeq::START_PARAM));
        addParam(createParamCentered<CKSSThree>(Vec(PS_LCOL[0], PS_LROW[1]), module, PsySeq::DIRECTION_PARAM));
        addParam(createParamCentered<CKSS>(Vec(PS_LCOL[1], PS_LROW[1]), module, PsySeq::RANDOM_VOLTAGE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PS_LCOL[0], PS_LROW[2]), module, PsySeq::MUTATION_WAIT_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PS_LCOL[1], PS_LROW[2]), module, PsySeq::MUTATION_PARAM));

        // ---- right block ----
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PS_RCOL[0], PS_RROW[0]), module, PsySeq::OUTPUT_OUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PS_RCOL[1], PS_RROW[0]), module, PsySeq::STEP_TRIGGER_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PS_RCOL[0], PS_RROW[1]), module, PsySeq::CYCLE_GATE_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PS_RCOL[1], PS_RROW[1]), module, PsySeq::INV_OUTPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PS_RCOL[0], PS_RROW[2]), module, PsySeq::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PS_RCOL[1], PS_RROW[2]), module, PsySeq::RESET_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PS_RCOL[0], PS_RROW[3]), module, PsySeq::RANDOMIZE_TRIGGER_INPUT));
        addParam(createParamCentered<VCVButton>(Vec(PS_RCOL[1], PS_RROW[3]), module, PsySeq::RANDOMIZE_BUTTON));

        // ---- 8 x 8 step grid: LED in the corner, trimpot in the middle ----
        for (int i = 0; i < 64; ++i)
        {
            float cx = psCellX(i % 8), cy = psCellY(i / 8);
            addChild(createLightCentered<SmallLight<PSCyanLight>>(Vec(cx + 12.f, cy - 11.f), module, PsySeq::LIGHT_OFFSET + i));
            addParam(createParamCentered<Trimpot>(Vec(cx, cy - 1.f), module, PsySeq::KNOB_OFFSET + i));
        }
    }

    void appendContextMenu(ui::Menu *menu) override
    {
        PsySeq *module = dynamic_cast<PsySeq *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Voltage range", "",
            [=](ui::Menu* sub) {
                static const float R[7][2] = {{0, 1}, {-1, 1}, {-2, 2}, {-5, 5}, {0, 5}, {0, 10}, {-10, 10}};
                for (int i = 0; i < 7; ++i) {
                    bool cur = (module->voltMin == R[i][0] && module->voltMax == R[i][1]);
                    sub->addChild(createMenuItem(string::f("%g to %g V", R[i][0], R[i][1]),
                        cur ? "✔" : "",
                        [=]() { module->setVoltageRange(R[i][0], R[i][1]); }));
                }
            }
        ));

        menu->addChild(new ui::MenuSeparator);
        menu->addChild(createMenuItem("Randomize all steps", "", [=]() { module->randomizeAllStepKnobs(); }));
        menu->addChild(createMenuItem("Clear all steps", "", [=]() { module->clearAllStepKnobs(); }));
    }
};

Model *modelPsySeq = createModel<PsySeq, PsySeqWidget>("PsySeq");