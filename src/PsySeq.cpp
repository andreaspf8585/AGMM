#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <algorithm>

// ============================================================
// PsySeq
// ============================================================
//
// OVERVIEW
// --------
// PsySeq is a 38 HP 8x8 step sequencer with probabilistic node visiting
// and periodic mutation. Each of the 64 cells has:
//   - a voltage trimpot (0..1, scaled into a user-selectable voltage range)
//   - a probability trimpot (0..1) that decides whether the step is visited
//     on that clock (visited steps update the CV and fire the STEP trigger;
//     skipped steps hold the previous CV and do not fire).
// The sequencer supports forward, reverse and random directions, a
// configurable start step (mapped into the 64-cell grid), and a
// "random voltage per step" mode that ignores the voltage trimpots and
// generates a new random voltage on every visited step.
//
// In addition, when a forward or reverse cycle completes, the module
// periodically attempts to mutate all active-step voltages, controlled by a
// "mutation probability" knob and a "wait cycles" knob that snaps to
// 0/4/8/16 cycles (0 = every cycle).
//
// Voltage knobs accept typed input in their right-click field: plain volts
// ("2.5", "2.5v") or a note name with octave ("a#4v", "c3", "eb5"). Note
// entry uses C4 = 0 V (1 V/oct) and switches the voltage range to -10..10.
//
// Outputs: CV out, inverted CV out, a short cycle gate (on each completed
// forward/reverse cycle), and a short STEP trigger (on each visited clock).
// Inputs: clock, reset, and a randomize trigger (which randomizes all step
// voltages). A front-panel RANDOMIZE button does the same.
//
// A live 8x8 grid display shows, per cell: the step number, a voltage bar,
// a probability bar, a skip cross for zero-probability steps, a warm start
// marker, and a cyan/grey playhead (cyan when the node was visited, grey
// when it was skipped). A separate status screen shows the current step,
// HIT/SKIP flag, direction/chip states, current voltage, probability and
// mutation wait/mutation readouts.
//
// File structure:
//   1. Layout constants (38 HP = 570x380 px).
//   2. WaitCyclesParamQuantity (snaps the mutation wait knob to 0/4/8/16).
//   3. VoltageParamQuantity (volt/note entry for the step voltage knobs).
//   4. PsySeq module: state, mapping, randomization helpers, JSON
//      persistence, and process().
//   5. Anonymous-namespace UI: colour helpers, blockHeader, PSPanelBackground,
//      PSGridWidget, PSStatusWidget.
//   6. PsySeqWidget: places controls, jacks, the grid, the status screen,
//      the randomize button and the context menu.
//   7. Model registration.
// ============================================================

// ------------------------------------------------------------
// Layout constants (38 HP = 570 px wide)
// ------------------------------------------------------------
static const float PS_PANEL_W = RACK_GRID_WIDTH * 38; // 570
static const float PS_PANEL_H = RACK_GRID_HEIGHT;     // 380

// 8 x 8 step grid: cell 36 x 36, pitch 40 x 39
//   top row: step number (left), skip cross, LED (right)
//   middle:  voltage trimpot (left), probability trimpot (right)
//   bottom:  voltage bar (left), probability bar (right)
static const float PS_GRID_X = 136.f;
static const float PS_GRID_Y = 42.f;
static const float PS_GRID_W = 316.f;
static const float PS_GRID_H = 309.f;
// Returns the horizontal centre of grid cell column c (0..7).
static float psCellX(int c) { return PS_GRID_X + c * 40.f + 18.f; } // cell centre
// Returns the vertical centre of grid cell row r (0..7).
static float psCellY(int r) { return PS_GRID_Y + r * 39.f + 18.f; }

// Left control block: 2 columns x 3 rows of 54 x 56 cells, then the status screen
static const float PS_LCOL[2] = {36.f, 96.f};
static const float PS_LTOP[3] = {52.f, 112.f, 172.f};
static const float PS_LCELL_W = 54.f, PS_LCELL_H = 56.f;
static const float PS_SCR_X = 12.f, PS_SCR_Y = 242.f, PS_SCR_W = 108.f, PS_SCR_H = 104.f;

// Right jack block: 2 columns of 46 x 52 cells
static const float PS_RCOL[2] = {485.f, 535.f};
static const float PS_RTOP[4] = {52.f, 112.f, 202.f, 262.f};
static const float PS_RCELL_W = 46.f, PS_RCELL_H = 52.f;
static const float PS_OUT_HDR_Y = 40.f, PS_IN_HDR_Y = 190.f;

// ------------------------------------------------------------
// Custom ParamQuantity for wait cycles that snaps to 0, 4, 8, 16
// ------------------------------------------------------------
// ParamQuantity that snaps the mutation-wait parameter to one of the
// allowed values {0, 4, 8, 16}, picking the nearest allowed value.
struct WaitCyclesParamQuantity : rack::engine::ParamQuantity
{
    // Snaps the requested value to the nearest of {0, 4, 8, 16} before
    // storing it.
    void setValue(float value) override
    {
        static const float allowed[4] = {0.f, 4.f, 8.f, 16.f};
        float nearest = allowed[0];
        float minDist = std::abs(value - nearest);
        for (int i = 1; i < 4; ++i)
        {
            float dist = std::abs(value - allowed[i]);
            if (dist < minDist)
            {
                minDist = dist;
                nearest = allowed[i];
            }
        }
        ParamQuantity::setValue(nearest);
    }
};

// ------------------------------------------------------------
// Custom ParamQuantity for the step voltage knobs.
// Shows the knob in volts (using the module's current range) and accepts
// typed volts or note names ("a#4v"). Notes switch the range to -10..10.
// Member functions are defined after the PsySeq struct.
// ------------------------------------------------------------
struct VoltageParamQuantity : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override;
    void setDisplayValueString(std::string s) override;
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
        PROB_OFFSET, // 64 knobs: visit probability per cell
        PROB_MAX = PROB_OFFSET + 64,
        PARAMS_LEN = PROB_MAX
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
        STEP_TRIGGER_OUTPUT,
        INV_OUTPUT,
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
    bool curVisited = true; // did the current step pass its probability?
    float stepFlash = 0.f;
    float mutFlash = 0.f;

    // Constructor: configures all parameters (with snapping where needed),
    // the 64 voltage and probability trimpots, the 64 lights, the three
    // inputs, four outputs and the clock->CV bypass.
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

        auto waitQuantity = configParam<WaitCyclesParamQuantity>(
            MUTATION_WAIT_PARAM, 0.f, 16.f, 0.f, "Wait cycles before mutation attempt");
        waitQuantity->snapEnabled = true;

        for (int i = 0; i < 64; i++)
        {
            configParam<VoltageParamQuantity>(KNOB_OFFSET + i, 0.f, 1.f, 0.f,
                                              string::f("Step %d voltage", i + 1), " V");
            configLight(LIGHT_OFFSET + i, string::f("Step %d", i + 1));
            configParam(PROB_OFFSET + i, 0.f, 1.f, 1.f,
                        string::f("Step %d visit probability", i + 1), "%", 0.f, 100.f);
        }

        configInput(CLOCK_INPUT, "Clock");
        configInput(RESET_INPUT, "Reset");
        configInput(RANDOMIZE_TRIGGER_INPUT, "Randomize trigger");
        configOutput(OUTPUT_OUT, "CV out");
        configOutput(CYCLE_GATE_OUTPUT, "Cycle gate");
        configOutput(STEP_TRIGGER_OUTPUT, "Step trigger (every visited clock)");
        configOutput(INV_OUTPUT, "Inverted CV out");

        configBypass(CLOCK_INPUT, OUTPUT_OUT);
    }

    // Position inside the sequence -> knob index (0..63), honouring the start step
    // Converts a sequence position (0..63) into the absolute 64-cell grid
    // index, offset by the current start step (1-based, wrapped into 0..63).
    int mapIndex(int pos)
    {
        int start = (int)params[START_PARAM].getValue();
        return (((start - 1 + pos) % 64) + 64) % 64;
    }

    // Returns the voltage for the given grid cell: in random mode it is a
    // fresh random value inside the voltage range; otherwise it is the
    // cell's voltage trimpot scaled into the range.
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

    // Recomputes the held CV output from the current position, using the
    // current voltage range and random-voltage mode.
    void updateHeldVoltage()
    {
        bool randomMode = params[RANDOM_VOLTAGE_PARAM].getValue() > 0.5f;
        heldVoltage = computeVoltageForStep(mapIndex(position), randomMode);
    }

    // Sets the voltage range and immediately refreshes the held CV (called
    // from the context menu's "Voltage range" submenu and by note entry).
    void setVoltageRange(float min, float max)
    {
        voltMin = min;
        voltMax = max;
        updateHeldVoltage();
    }

    // Randomises the voltage trimpots of all 64 cells and refreshes the
    // held CV (used by the RANDOMIZE button, the randomize input, and the
    // context menu).
    void randomizeAllStepKnobs()
    {
        for (int i = 0; i < 64; i++)
            params[KNOB_OFFSET + i].setValue(random::uniform());
        updateHeldVoltage();
    }

    // Sets all voltage trimpots to zero and refreshes the held CV.
    void clearAllStepKnobs()
    {
        for (int i = 0; i < 64; i++)
            params[KNOB_OFFSET + i].setValue(0.f);
        updateHeldVoltage();
    }

    // Sets every cell's visit probability trimpot to `v`.
    void setAllProbabilities(float v)
    {
        for (int i = 0; i < 64; i++)
            params[PROB_OFFSET + i].setValue(v);
    }

    // Randomises every cell's visit probability trimpot.
    void randomizeAllProbabilities()
    {
        for (int i = 0; i < 64; i++)
            params[PROB_OFFSET + i].setValue(random::uniform());
    }

    // Serialises the voltage range so patches reopen with the same output
    // scaling.
    json_t *dataToJson() override
    {
        json_t *rootJ = json_object();
        json_object_set_new(rootJ, "voltMin", json_real(voltMin));
        json_object_set_new(rootJ, "voltMax", json_real(voltMax));
        return rootJ;
    }

    // Restores the voltage range from a patch, falling back to -1..1 if the
    // values are missing or invalid, then refreshes the held CV.
    void dataFromJson(json_t *rootJ) override
    {
        json_t *minJ = json_object_get(rootJ, "voltMin");
        json_t *maxJ = json_object_get(rootJ, "voltMax");
        if (minJ && json_is_number(minJ))
            voltMin = (float)json_number_value(minJ);
        if (maxJ && json_is_number(maxJ))
            voltMax = (float)json_number_value(maxJ);
        if (!(voltMax > voltMin))
        {
            voltMin = -1.f;
            voltMax = 1.f;
        }
        updateHeldVoltage();
    }

    // Main audio callback: handles the RANDOMIZE button and trigger, clamps
    // and reacts to STEPS/START changes, handles RESET, advances the
    // position on each clock according to direction, applies the per-cell
    // visit probability, fires the STEP trigger on visited steps, counts
    // completed cycles and periodically mutates the step voltages, then
    // updates the CV/inverted CV/cycle gate/step trigger outputs and the
    // grid LEDs.
    void process(const ProcessArgs &args) override
    {
        // Button randomize
        if (randomizeTrigger.process(params[RANDOMIZE_BUTTON].getValue()))
            randomizeAllStepKnobs();

        // External trigger randomize
        if (randomizeInputTrigger.process(inputs[RANDOMIZE_TRIGGER_INPUT].getVoltage()))
            randomizeAllStepKnobs();

        float stepsParam = params[STEPS_PARAM].getValue();
        float startParam = params[START_PARAM].getValue();
        float direction = params[DIRECTION_PARAM].getValue();

        int steps = (int)stepsParam;
        if (steps < 1)
            steps = 1;
        if (steps > 64)
            steps = 64;

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
            curVisited = true;
            updateHeldVoltage();
        }

        // Clock step
        if (clockTrigger.process(inputs[CLOCK_INPUT].getVoltage()))
        {
            bool cycleCompleted = false;

            if (direction < 0.5f) // Forward
            {
                position++;
                if (position >= steps)
                {
                    position = 0;
                    cycleCompleted = true;
                }
            }
            else if (direction < 1.5f) // Reverse
            {
                position--;
                if (position < 0)
                {
                    position = steps - 1;
                    cycleCompleted = true;
                }
            }
            else // Random
            {
                position = (int)(random::uniform() * steps);
                if (position >= steps)
                    position = steps - 1;
                if (position < 0)
                    position = 0;
            }

            // ---- probability gate: does the sequencer visit this node? ----
            int idx = mapIndex(position);
            float p = params[PROB_OFFSET + idx].getValue();
            bool visited = (p > 1e-4f) && (random::uniform() < p);

            curVisited = visited;

            if (visited)
            {
                stepPulse.trigger(5e-3f);
                stepFlash = 1.f;
            }

            // ---- PERIODIC MUTATION ATTEMPT (forward/reverse only) ----
            if (cycleCompleted)
            {
                cycleCounter++;

                float waitCycles = params[MUTATION_WAIT_PARAM].getValue(); // 0, 4, 8, 16
                float mutationProb = params[MUTATION_PARAM].getValue();

                bool shouldAttempt = (waitCycles == 0.0f) || (cycleCounter >= (int)waitCycles);

                if (shouldAttempt)
                {
                    if (random::uniform() < mutationProb)
                    {
                        for (int i = 0; i < steps; i++)
                            params[KNOB_OFFSET + mapIndex(i)].setValue(random::uniform());
                        mutFlash = 1.f;
                    }
                    cycleCounter = 0;
                }

                cyclePulse.trigger(1e-3f);
            }

            // Skipped nodes hold their CV (and emit no gate)
            if (visited)
                updateHeldVoltage();
        }

        curIndex = mapIndex(position);

        // Outputs
        outputs[OUTPUT_OUT].setVoltage(heldVoltage);
        outputs[INV_OUTPUT].setVoltage(voltMin + voltMax - heldVoltage);
        outputs[CYCLE_GATE_OUTPUT].setVoltage(cyclePulse.process(args.sampleTime) ? 10.0f : 0.0f);
        outputs[STEP_TRIGGER_OUTPUT].setVoltage(stepPulse.process(args.sampleTime) ? 10.0f : 0.0f);

        // LEDs and display data
        stepFlash = std::max(0.f, stepFlash - 4.f * args.sampleTime);
        mutFlash = std::max(0.f, mutFlash - 2.f * args.sampleTime);

        for (int i = 0; i < 64; i++)
        {
            float target = (i == curIndex && curVisited) ? 1.f : 0.f;
            lights[LIGHT_OFFSET + i].setSmoothBrightness(target, args.sampleTime);
        }
    }
};

// ------------------------------------------------------------
// VoltageParamQuantity implementation
// ------------------------------------------------------------
namespace
{
    // Parses a note name with octave such as "a#4", "c3", "eb5", "g-1"
    // (case-insensitive, optional trailing 'v', spaces ignored) into volts
    // with C4 = 0 V at 1 V/oct. Returns false if the text is not a note.
    bool psParseNote(const std::string &in, float &voltsOut)
    {
        std::string s;
        for (char ch : in)
        {
            if (ch == ' ' || ch == '\t')
                continue;
            s += (char)std::tolower((unsigned char)ch);
        }
        if (!s.empty() && s.back() == 'v')
            s.pop_back();
        if (s.size() < 2)
            return false;

        int semi;
        switch (s[0])
        {
        case 'c':
            semi = 0;
            break;
        case 'd':
            semi = 2;
            break;
        case 'e':
            semi = 4;
            break;
        case 'f':
            semi = 5;
            break;
        case 'g':
            semi = 7;
            break;
        case 'a':
            semi = 9;
            break;
        case 'b':
            semi = 11;
            break;
        default:
            return false;
        }

        size_t i = 1;
        if (i < s.size() && s[i] == '#')
        {
            semi += 1;
            ++i;
        }
        else if (i + 1 < s.size() && s[i] == 'b')
        {
            semi -= 1;
            ++i;
        }

        bool neg = false;
        if (i < s.size() && s[i] == '-')
        {
            neg = true;
            ++i;
        }
        if (i >= s.size())
            return false;
        int oct = 0;
        for (; i < s.size(); ++i)
        {
            if (s[i] < '0' || s[i] > '9')
                return false;
            oct = oct * 10 + (s[i] - '0');
            if (oct > 100)
                return false;
        }
        if (neg)
            oct = -oct;

        voltsOut = (float)(oct - 4) + (float)semi / 12.f;
        return true;
    }

    // Parses plain volts such as "2.5" or "2.5v".
    bool psParseVolts(const std::string &in, float &voltsOut)
    {
        std::string s;
        for (char ch : in)
        {
            if (ch == ' ' || ch == '\t')
                continue;
            s += (char)std::tolower((unsigned char)ch);
        }
        if (!s.empty() && s.back() == 'v')
            s.pop_back();
        if (s.empty())
            return false;
        char *end = nullptr;
        float v = std::strtof(s.c_str(), &end);
        if (end == s.c_str() || *end != '\0')
            return false;
        voltsOut = v;
        return true;
    }
} // namespace

// Shows the knob value as volts in the module's current range.
std::string VoltageParamQuantity::getDisplayValueString()
{
    PsySeq *m = dynamic_cast<PsySeq *>(module);
    if (!m)
        return ParamQuantity::getDisplayValueString();
    float v = m->voltMin + getValue() * (m->voltMax - m->voltMin);
    return string::f("%.3f", v);
}

// Accepts a note ("a#4v") or plain volts. Notes switch the range to
// -10..10 V first; the knob is then set so the output equals the typed
// voltage.
void VoltageParamQuantity::setDisplayValueString(std::string s)
{
    PsySeq *m = dynamic_cast<PsySeq *>(module);
    if (!m)
    {
        ParamQuantity::setDisplayValueString(s);
        return;
    }

    float volts;
    if (psParseNote(s, volts))
    {
        if (m->voltMin != -10.f || m->voltMax != 10.f)
            m->setVoltageRange(-10.f, 10.f);
    }
    else if (!psParseVolts(s, volts))
    {
        return; // unrecognised text: ignore
    }

    float range = m->voltMax - m->voltMin;
    float norm = (range > 0.f) ? (volts - m->voltMin) / range : 0.f;
    setValue(math::clamp(norm, 0.f, 1.f));
    m->updateHeldVoltage();
}

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{
    namespace ag = agmm;

    // Returns the colour used for probability markers and bars (violet).
    NVGcolor colProb() { return ag::violet(); }
    // Returns the colour used for the voltage bars and playhead (cyan).
    NVGcolor colHot() { return ag::accent(); }

    // Block title with hairlines that stop short of the text
    // Draws a centred block header whose hairlines span from cx-halfW to
    // cx+halfW, stopping a measured distance from the text.
    void blockHeader(NVGcontext *vg, float cx, float y, const char *txt, float halfW, NVGcolor col)
    {
        float gap = ag::textWidth(vg, txt, 8.f, 2.f) / 2.f + 6.f;
        ag::text(vg, cx, y, txt, 8.f, col, ag::ALIGN_C, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx - halfW, y);
        nvgLineTo(vg, cx - gap, y);
        nvgMoveTo(vg, cx + gap, y);
        nvgLineTo(vg, cx + halfW, y);
        nvgStrokeColor(vg, ag::withA(col, 0.35f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // ------------------------------------------------------------
    // Panel background: title, frames, labelled plates, grid legend
    // ------------------------------------------------------------
    // Static artwork: base panel, display and status-screen bezels, the
    // grid legend row, the left CONTROL block cells, and the right
    // OUTPUT/INPUT blocks with their jack cells (outputs tinted).
    struct PSPanelBackground : TransparentWidget
    {
        PSPanelBackground() { box.size = Vec(PS_PANEL_W, PS_PANEL_H); }

        // Draws all static panel art: base panel, bezels, the VOLTAGE/
        // PROBABILITY/START STEP grid legend, the six CONTROL cells, the
        // four OUTPUT jack cells (tinted), and the INPUT jack cells.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, PS_PANEL_W, PS_PANEL_H, "PSYSEQ");
            ag::bezel(vg, PS_GRID_X, PS_GRID_Y, PS_GRID_W, PS_GRID_H);
            ag::bezel(vg, PS_SCR_X, PS_SCR_Y, PS_SCR_W, PS_SCR_H);

            // ---- grid legend (own row above the grid) ----
            const float ly = 33.f;
            nvgBeginPath(vg);
            nvgRect(vg, PS_GRID_X + 2.f, ly - 2.5f, 5.f, 5.f);
            nvgFillColor(vg, colHot());
            nvgFill(vg);
            ag::text(vg, PS_GRID_X + 11.f, ly, "VOLTAGE", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);

            nvgBeginPath(vg);
            nvgRect(vg, PS_GRID_X + 82.f, ly - 2.5f, 5.f, 5.f);
            nvgFillColor(vg, colProb());
            nvgFill(vg);
            ag::text(vg, PS_GRID_X + 91.f, ly, "PROBABILITY", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);

            nvgBeginPath(vg);
            nvgRect(vg, PS_GRID_X + 176.f, ly - 2.5f, 5.f, 5.f);
            nvgFillColor(vg, ag::warm());
            nvgFill(vg);
            ag::text(vg, PS_GRID_X + 185.f, ly, "START STEP", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);

            // ---- left block ----
            ag::sectionHeader(vg, 132.f, 40.f, "CONTROL", 9.f);
            const char *lab[6] = {"STEPS", "START", "DIRECTION", "RAND V", "WAIT", "MUTATE"};
            for (int i = 0; i < 6; ++i)
                ag::cell(vg, PS_LCOL[i % 2], PS_LTOP[i / 2], PS_LCELL_W, PS_LCELL_H, lab[i]);

            // ---- right block ----
            blockHeader(vg, 510.f, PS_OUT_HDR_Y, "OUTPUT", 48.f, colProb());
            ag::cell(vg, PS_RCOL[0], PS_RTOP[0], PS_RCELL_W, PS_RCELL_H, "CV OUT", true);
            ag::cell(vg, PS_RCOL[1], PS_RTOP[0], PS_RCELL_W, PS_RCELL_H, "STEP", true);
            ag::cell(vg, PS_RCOL[0], PS_RTOP[1], PS_RCELL_W, PS_RCELL_H, "CYCLE", true);
            ag::cell(vg, PS_RCOL[1], PS_RTOP[1], PS_RCELL_W, PS_RCELL_H, "CV INV", true);

            blockHeader(vg, 510.f, PS_IN_HDR_Y, "INPUT", 48.f, ag::accent());
            ag::cell(vg, PS_RCOL[0], PS_RTOP[2], PS_RCELL_W, PS_RCELL_H, "CLK");
            ag::cell(vg, PS_RCOL[1], PS_RTOP[2], PS_RCELL_W, PS_RCELL_H, "RESET");
            ag::cell(vg, PS_RCOL[0], PS_RTOP[3], PS_RCELL_W, PS_RCELL_H, "RAND IN");
            ag::cell(vg, PS_RCOL[1], PS_RTOP[3], PS_RCELL_W, PS_RCELL_H, "RAND");

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Step grid: cells, active range, playhead, value + probability bars
    // ------------------------------------------------------------
    // Live 8x8 grid display: draws the 64 cell backgrounds with their step
    // numbers on layer 0, and on layer 1 the active-range tint, mutation
    // flash, voltage bar, probability bar, zero-probability cross, start
    // marker and playhead ring.
    struct PSGridWidget : TransparentWidget
    {
        PsySeq *module = nullptr;

        PSGridWidget(PsySeq *m) : module(m)
        {
            box.pos = Vec(PS_GRID_X, PS_GRID_Y);
            box.size = Vec(PS_GRID_W, PS_GRID_H);
        }

        // Static layer: draws the 64 rounded cell backgrounds and step
        // numbers.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            for (int i = 0; i < 64; ++i)
            {
                float x = (i % 8) * 40.f, y = (i / 8) * 39.f;
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, y, 36.f, 36.f, 4.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 8));
                nvgFill(vg);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                char buf[8];
                snprintf(buf, sizeof(buf), "%d", i + 1);
                ag::text(vg, x + 3.f, y + 6.f, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.f);
            }
            Widget::draw(args);
        }

        // Layer-1 overlay: for each cell, draws active-range tint and
        // mutation flash, the voltage and probability bars, a skip cross if
        // the probability is zero, the start marker, and the playhead ring
        // (cyan if visited, grey if skipped). Uses demo data when there is
        // no module.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            int steps, start, cur;
            float flash, mutFlash;
            bool randMode, visited;
            float knob[64], prob[64];
            if (module)
            {
                steps = (int)module->params[PsySeq::STEPS_PARAM].getValue();
                start = (int)module->params[PsySeq::START_PARAM].getValue();
                cur = module->curIndex;
                visited = module->curVisited;
                flash = module->stepFlash;
                mutFlash = module->mutFlash;
                randMode = module->params[PsySeq::RANDOM_VOLTAGE_PARAM].getValue() > 0.5f;
                for (int i = 0; i < 64; ++i)
                {
                    knob[i] = module->params[PsySeq::KNOB_OFFSET + i].getValue();
                    prob[i] = module->params[PsySeq::PROB_OFFSET + i].getValue();
                }
            }
            else
            {
                steps = 16;
                start = 1;
                cur = 5;
                visited = true;
                flash = 0.6f;
                mutFlash = 0.f;
                randMode = false;
                for (int i = 0; i < 64; ++i)
                {
                    knob[i] = 0.5f + 0.5f * std::sin(i * 0.9f + 0.4f) * std::cos(i * 0.31f);
                    prob[i] = (i % 5 == 0) ? 0.3f : 0.85f;
                }
            }
            steps = std::min(std::max(steps, 1), 64);
            start = std::min(std::max(start, 1), 64);

            for (int i = 0; i < 64; ++i)
            {
                float x = (i % 8) * 40.f, y = (i / 8) * 39.f;
                int rel = (((i - (start - 1)) % 64) + 64) % 64;
                bool active = rel < steps;

                // Cell tint: active range lightly tinted, inactive darkened
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x, y, 36.f, 36.f, 4.f);
                if (active)
                {
                    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
                    nvgFill(vg);
                    if (mutFlash > 0.02f)
                    {
                        nvgFillColor(vg, ag::withA(colProb(), 0.35f * mutFlash));
                        nvgFill(vg);
                    }
                }
                else
                {
                    nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
                    nvgFill(vg);
                }

                // ---- voltage bar (bottom left, under the voltage trimpot) ----
                float kv = std::min(std::max(knob[i], 0.f), 1.f);
                nvgBeginPath(vg);
                nvgRect(vg, x + 2.f, y + 31.f, 13.f, 2.5f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
                if (kv > 0.005f)
                {
                    float a = randMode ? 0.18f : (active ? 0.95f : 0.3f);
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 2.f, y + 31.f, 13.f * kv, 2.5f);
                    nvgFillColor(vg, ag::withA(colHot(), a));
                    nvgFill(vg);
                }

                // ---- probability bar (bottom right, under the probability trimpot) ----
                float pv = std::min(std::max(prob[i], 0.f), 1.f);
                nvgBeginPath(vg);
                nvgRect(vg, x + 21.f, y + 31.f, 13.f, 2.5f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
                if (pv > 0.005f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 21.f, y + 31.f, 13.f * pv, 2.5f);
                    nvgFillColor(vg, ag::withA(colProb(), active ? 0.95f : 0.35f));
                    nvgFill(vg);
                }

                // Zero-probability cell: small cross in the top row, between number and LED
                if (pv < 0.005f)
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, x + 16.5f, y + 3.5f);
                    nvgLineTo(vg, x + 21.5f, y + 8.5f);
                    nvgMoveTo(vg, x + 21.5f, y + 3.5f);
                    nvgLineTo(vg, x + 16.5f, y + 8.5f);
                    nvgStrokeColor(vg, nvgRGBA(0xff, 0x60, 0x60, 160));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }

                // Start marker: warm bar along the top edge
                if (i == start - 1)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 4.f, y + 1.f, 18.f, 1.5f);
                    nvgFillColor(vg, ag::warm());
                    nvgFill(vg);
                }

                // Playhead (cyan when the node was visited, grey when skipped)
                if (i == cur)
                {
                    NVGcolor ringCol = visited ? colHot() : nvgRGB(0x6a, 0x74, 0x82);
                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, x, y, 36.f, 36.f, 4.f);
                    nvgFillColor(vg, ag::withA(ringCol, visited ? (25 + 45 * flash) / 255.f : 0.12f));
                    nvgFill(vg);
                    nvgStrokeColor(vg, visited ? ag::bright() : nvgRGB(0x7a, 0x85, 0x94));
                    nvgStrokeWidth(vg, 1.4f);
                    nvgStroke(vg);
                }
            }

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Status screen
    // ------------------------------------------------------------
    // Live status screen: shows the current step / total, HIT/SKIP flag,
    // direction/RAND V/MUT chips, a voltage bar, the current voltage,
    // probability and the mutation wait/mutation readouts.
    struct PSStatusWidget : TransparentWidget
    {
        PsySeq *module = nullptr;

        PSStatusWidget(PsySeq *m) : module(m)
        {
            box.pos = Vec(PS_SCR_X, PS_SCR_Y);
            box.size = Vec(PS_SCR_W, PS_SCR_H);
        }

        // Static layer: fills the screen area with the flat screen colour.
        void draw(const DrawArgs &args) override
        {
            ag::screen(args.vg, 0.f, 0.f, PS_SCR_W, PS_SCR_H);
            Widget::draw(args);
        }

        // Layer-1 overlay: draws the status text and voltage bar. Uses demo
        // values in the module browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = PS_SCR_W;

            int steps, pos, dir, wait, cyc, curIdx;
            float volt, vmin, vmax, mut, flash, prob;
            bool rnd, visited;
            if (module)
            {
                steps = (int)module->params[PsySeq::STEPS_PARAM].getValue();
                pos = module->position;
                dir = (int)std::round(module->params[PsySeq::DIRECTION_PARAM].getValue());
                wait = (int)std::round(module->params[PsySeq::MUTATION_WAIT_PARAM].getValue());
                cyc = module->cycleCounter;
                volt = module->heldVoltage;
                vmin = module->voltMin;
                vmax = module->voltMax;
                mut = module->params[PsySeq::MUTATION_PARAM].getValue();
                rnd = module->params[PsySeq::RANDOM_VOLTAGE_PARAM].getValue() > 0.5f;
                flash = module->stepFlash;
                visited = module->curVisited;
                curIdx = module->curIndex;
                prob = module->params[PsySeq::PROB_OFFSET + curIdx].getValue();
            }
            else
            {
                steps = 16;
                pos = 5;
                dir = 0;
                wait = 8;
                cyc = 3;
                volt = 0.42f;
                vmin = -1.f;
                vmax = 1.f;
                mut = 0.4f;
                rnd = false;
                flash = 0.f;
                visited = true;
                prob = 0.75f;
            }
            dir = std::min(std::max(dir, 0), 2);
            steps = std::max(steps, 1);

            char buf[48];

            // Row 1: labels
            ag::text(vg, 6.f, 8.f, "STEP", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "OF %d", steps);
            ag::text(vg, W - 6.f, 8.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.5f);

            // Row 2: step number (left) and HIT / SKIP flag (right)
            snprintf(buf, sizeof(buf), "%02d", pos + 1);
            ag::text(vg, 6.f, 27.f, buf, 22.f,
                     ag::withA(ag::bright(), (190.f + 65.f * flash) / 255.f), ag::ALIGN_L, 1.f);
            ag::text(vg, W - 6.f, 27.f, visited ? "HIT" : "SKIP", 7.f,
                     visited ? colHot() : nvgRGB(0xff, 0x70, 0x70), ag::ALIGN_R, 0.5f);

            // Row 3: chips sized from their text
            static const char *dirNames[3] = {"FWD", "REV", "RND"};
            const char *chipTxt[3] = {dirNames[dir], "RAND V", "MUT"};
            const bool chipOn[3] = {true, rnd, mut > 0.001f};
            const NVGcolor chipCol[3] = {ag::accent(), colProb(), colProb()};
            float cx0 = 6.f;
            for (int k = 0; k < 3; ++k)
            {
                float w = ag::textWidth(vg, chipTxt[k], 7.f, 0.5f) + 8.f;
                ag::chip(vg, cx0, 41.f, w, 12.f, chipTxt[k], chipOn[k], chipCol[k]);
                cx0 += w + 4.f;
            }

            // Row 4: voltage meter (flat)
            float t = (vmax > vmin) ? (volt - vmin) / (vmax - vmin) : 0.5f;
            t = std::min(std::max(t, 0.f), 1.f);
            nvgBeginPath(vg);
            nvgRect(vg, 8.f, 61.f, W - 16.f, 4.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, 8.f, 61.f, std::max((W - 16.f) * t, 2.f), 4.f);
            nvgFillColor(vg, colHot());
            nvgFill(vg);

            // Rows 5-7: readouts
            snprintf(buf, sizeof(buf), "%+.2fV", volt);
            ag::text(vg, W / 2.f, 76.f, buf, 9.f, nvgRGBA(0xff, 0xe6, 0x80, 230), ag::ALIGN_C, 0.5f);

            snprintf(buf, sizeof(buf), "PROB %.0f%%", prob * 100.f);
            ag::text(vg, W / 2.f, 88.f, buf, 7.f, colProb(), ag::ALIGN_C, 0.5f);

            snprintf(buf, sizeof(buf), "WAIT %d/%d  MUT %.0f%%", cyc, wait, mut * 100.f);
            ag::text(vg, W / 2.f, 98.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget: adds the static panel, screws, the 8x8 grid and status
// screen, the six CONTROL knobs/switches, the eight right-block jacks/button
// and the 64 grid LEDs, plus the right-click menu.
// ------------------------------------------------------------
struct PsySeqWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, grid, status
    // screen, the six CONTROL cells, the eight right-block jacks and the
    // RANDOMIZE button, and the 64 per-cell LEDs plus voltage and
    // probability trimpots.
    PsySeqWidget(PsySeq *module)
    {
        setModule(module);
        box.size = Vec(PS_PANEL_W, PS_PANEL_H);

        addChild(new PSPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PSGridWidget(module));
        addChild(new PSStatusWidget(module));

        // ---- left block: control sits 35 px below each cell's top edge ----
        const float ky[3] = {PS_LTOP[0] + 35.f, PS_LTOP[1] + 35.f, PS_LTOP[2] + 35.f};
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PS_LCOL[0], ky[0]), module, PsySeq::STEPS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PS_LCOL[1], ky[0]), module, PsySeq::START_PARAM));
        addParam(createParamCentered<CKSSThree>(Vec(PS_LCOL[0], ky[1]), module, PsySeq::DIRECTION_PARAM));
        addParam(createParamCentered<CKSS>(Vec(PS_LCOL[1], ky[1]), module, PsySeq::RANDOM_VOLTAGE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PS_LCOL[0], ky[2]), module, PsySeq::MUTATION_WAIT_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PS_LCOL[1], ky[2]), module, PsySeq::MUTATION_PARAM));

        // ---- right block: jacks sit 32 px below each cell's top edge ----
        const float jy[4] = {PS_RTOP[0] + 32.f, PS_RTOP[1] + 32.f, PS_RTOP[2] + 32.f, PS_RTOP[3] + 32.f};
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PS_RCOL[0], jy[0]), module, PsySeq::OUTPUT_OUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PS_RCOL[1], jy[0]), module, PsySeq::STEP_TRIGGER_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PS_RCOL[0], jy[1]), module, PsySeq::CYCLE_GATE_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PS_RCOL[1], jy[1]), module, PsySeq::INV_OUTPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PS_RCOL[0], jy[2]), module, PsySeq::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PS_RCOL[1], jy[2]), module, PsySeq::RESET_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PS_RCOL[0], jy[3]), module, PsySeq::RANDOMIZE_TRIGGER_INPUT));
        addParam(createParamCentered<VCVButton>(Vec(PS_RCOL[1], jy[3]), module, PsySeq::RANDOMIZE_BUTTON));

        // ---- 8 x 8 grid: LED top-right, voltage trimpot left, probability trimpot right ----
        for (int i = 0; i < 64; ++i)
        {
            float cx = psCellX(i % 8), cy = psCellY(i / 8);
            addChild(createLightCentered<SmallLight<agmm::CyanLight>>(
                Vec(cx + 12.f, cy - 12.5f), module, PsySeq::LIGHT_OFFSET + i));
            addParam(createParamCentered<Trimpot>(Vec(cx - 9.25f, cy + 1.5f), module, PsySeq::KNOB_OFFSET + i));
            addParam(createParamCentered<Trimpot>(Vec(cx + 9.25f, cy + 1.5f), module, PsySeq::PROB_OFFSET + i));
        }
    }

    // Adds the right-click menu: the voltage range submenu, "randomize/clear
    // all step voltages", and "all probabilities 100% / randomize all
    // probabilities".
    void appendContextMenu(ui::Menu *menu) override
    {
        PsySeq *module = dynamic_cast<PsySeq *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Voltage range", "",
                                         [=](ui::Menu *sub)
                                         {
                                             static const float R[7][2] = {{0, 1}, {-1, 1}, {-2, 2}, {-5, 5}, {0, 5}, {0, 10}, {-10, 10}};
                                             for (int i = 0; i < 7; ++i)
                                             {
                                                 bool cur = (module->voltMin == R[i][0] && module->voltMax == R[i][1]);
                                                 sub->addChild(createMenuItem(string::f("%g to %g V", R[i][0], R[i][1]),
                                                                              CHECKMARK(cur),
                                                                              [=]()
                                                                              { module->setVoltageRange(R[i][0], R[i][1]); }));
                                             }
                                         }));

        menu->addChild(new ui::MenuSeparator);
        menu->addChild(createMenuItem("Randomize all step voltages", "", [=]()
                                      { module->randomizeAllStepKnobs(); }));
        menu->addChild(createMenuItem("Clear all step voltages", "", [=]()
                                      { module->clearAllStepKnobs(); }));

        menu->addChild(new ui::MenuSeparator);
        menu->addChild(createMenuItem("All probabilities 100%", "", [=]()
                                      { module->setAllProbabilities(1.f); }));
        menu->addChild(createMenuItem("Randomize all probabilities", "", [=]()
                                      { module->randomizeAllProbabilities(); }));
    }
};

Model *modelPsySeq = createModel<PsySeq, PsySeqWidget>("PsySeq");