#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ============================================================
// FancyADSR
// ============================================================
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// FancyADSR is a 20 HP envelope generator with per-stage CV control, selectable
// linear/exponential attack and release shapes, an optional looping mode, a
// selectable ADSR / AD mode, and a live scope that shows both the ideal curve
// implied by the current knobs (violet preview) and the actual envelope output
// history (cyan live trace).
//
// ADSR mode (default):
//   The envelope runs through ATTACK -> DECAY -> SUSTAIN -> RELEASE -> IDLE.
//   Attack and release are timed so that an exponential segment reaches within
//   0.1% of its target after exactly the configured duration; decay is always
//   exponential (the usual ADSR behaviour). An end-of-release (EOR) pulse fires
//   for 5 ms when the envelope reaches zero, and a LOOP switch makes the
//   envelope retrigger from the gate while it is held.
//
// AD mode:
//   The envelope runs ATTACK -> DECAY (down to zero) -> IDLE, using only the
//   ATTACK and DECAY times. SUSTAIN and RELEASE are ignored. The gate input
//   acts purely as a trigger: a rising edge starts the envelope (retriggering
//   from the current level if it is already running) and the falling edge is
//   ignored, so the envelope always plays through. The RETRIG input also
//   starts it. The EOR output pulses at the end of the decay. With LOOP on,
//   the envelope repeats for as long as the gate is held high.
//
// Each stage has its own CV input (5 V adds one second, or 100% for sustain),
// and the scope's time axis is user-selectable via the TIME knob.
//
// File structure:
//   1. Layout constants (20 HP = 300x380 px).
//   2. FancyADSR module: state, param helpers, envelope maths, process() and
//      the scope history buffer.
//   3. Anonymous-namespace UI: segCurve helper, FAPanelBackground (static
//      artwork) and FADisplayWidget (scope + readouts + chips).
//   4. FancyADSRWidget: places knobs, jacks, switches and the display.
//   5. Model registration.
// -----------------------------------------------------------------------------

// ------------------------------------------------------------
// Layout (20 HP = 300 x 380)
// Every control sits in a plate with its label ABOVE it.
// ------------------------------------------------------------
static const float FA_PANEL_W = RACK_GRID_WIDTH * 20; // 300
static const float FA_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it). Rows, top to bottom:
//   y  9      readouts (A D S R, output volts)
//   18-74     scope (voltage labels in the left column, level meter on the right)
//   83        legend row (preview / live / scope time)
//   92-104    stage chips
static const float FA_DISP_X = 30.f;
static const float FA_DISP_Y = 32.f;
static const float FA_DISP_W = 240.f;
static const float FA_DISP_H = 112.f;

static const float FA_SX0 = 18.f;
static const float FA_SY0 = 18.f;
static const float FA_SW = 200.f;
static const float FA_SH = 56.f;
static const float FA_LEGEND_Y = 83.f;
static const float FA_CHIP_Y = 92.f, FA_CHIP_H = 12.f;

// Rows below the display
static const float FA_ENV_HEAD_Y = 158.f;
static const float FA_ROWA_TOP = 166.f, FA_ROWA_H = 74.f; // envelope knobs + CV jacks
static const float FA_ROWB_TOP = 246.f, FA_ROWB_H = 48.f; // mode, shapes, loop, time
static const float FA_IO_HEAD_Y = 303.f;
static const float FA_ROWC_TOP = 311.f, FA_ROWC_H = 46.f; // I/O
static const float FA_CELL_W = 66.f;
static const float FA_CELL5_W = 54.f; // cell width for the five-column row B

// Horizontal centre of column i (0..3) for the four-column control rows.
static float faColX(int i) { return 37.5f + 75.f * i; }

// Horizontal centre of column i (0..4) for the five-column row B.
static float faColX5(int i) { return 30.f + 60.f * i; }

// ------------------------------------------------------------
// Fancy ADSR Module
// ------------------------------------------------------------
struct FancyADSR : Module
{
    enum ParamIds
    {
        ATTACK_PARAM,
        DECAY_PARAM,
        SUSTAIN_PARAM,
        RELEASE_PARAM,
        ATTACK_SHAPE_PARAM, // 0 = linear, 1 = exponential
        RELEASE_SHAPE_PARAM,
        LOOP_TOGGLE_PARAM,
        TIME_SCALE_PARAM, // seconds per full graph width
        MODE_PARAM,       // 0 = ADSR, 1 = AD (appended last: keeps old patches compatible)
        TEST_PARAM,       // momentary button: acts as a gate while held
        NUM_PARAMS
    };
    enum InputIds
    {
        GATE_INPUT,
        TRIGGER_INPUT,
        ATTACK_CV_INPUT,
        DECAY_CV_INPUT,
        SUSTAIN_CV_INPUT,
        RELEASE_CV_INPUT,
        NUM_INPUTS
    };
    enum OutputIds
    {
        ENV_OUTPUT,
        EOR_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds
    {
        LOOP_LIGHT,
        NUM_LIGHTS
    };

    // Envelope stages.
    enum Stage
    {
        STAGE_IDLE,
        STAGE_ATTACK,
        STAGE_DECAY,
        STAGE_SUSTAIN,
        STAGE_RELEASE
    };
    Stage stage = STAGE_IDLE;
    float env = 0.0f;
    float stageTime = 0.0f;
    float attackStart = 0.0f; // level the current attack rises from (0 in ADSR mode)
    float decayStart = 1.0f;  // level the current AD-mode decay falls from
    float releaseStart = 0.0f; // level the current release falls from (captured when it begins)
    float eorPulseRemaining = 0.0f;
    bool lastGateState = false;
    bool lastTriggerState = false;

    // Real-time graph history (0-10V)
    static const int HISTORY_SIZE = 400;
    float history[HISTORY_SIZE];
    int historyCount = 0;
    int historyWritePos = 0;
    float sampleAccumulator = 0.0f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float dispAttack = 0.01f, dispDecay = 0.1f, dispSustain = 0.8f, dispRelease = 0.5f;
    float dispAttackShape = 1.f, dispReleaseShape = 1.f;
    bool dispGate = false;
    bool dispAD = false;
    float eorFlash = 0.f;

    // Parameter helpers (with CV)
    // Attack time from the knob plus its CV, clamped to 1 ms .. 10 s.
    float getAttack() { return rack::math::clamp(params[ATTACK_PARAM].getValue() + inputs[ATTACK_CV_INPUT].getVoltage() / 5.0f, 0.001f, 10.0f); }
    // Decay time from the knob plus its CV, clamped to 1 ms .. 10 s.
    float getDecay() { return rack::math::clamp(params[DECAY_PARAM].getValue() + inputs[DECAY_CV_INPUT].getVoltage() / 5.0f, 0.001f, 10.0f); }
    // Sustain level from the knob plus its CV, clamped to 0..1.
    float getSustain() { return rack::math::clamp(params[SUSTAIN_PARAM].getValue() + inputs[SUSTAIN_CV_INPUT].getVoltage() / 5.0f, 0.0f, 1.0f); }
    // Release time from the knob plus its CV, clamped to 1 ms .. 10 s.
    float getRelease() { return rack::math::clamp(params[RELEASE_PARAM].getValue() + inputs[RELEASE_CV_INPUT].getVoltage() / 5.0f, 0.001f, 10.0f); }
    // True when the LOOP switch is on.
    bool getLoop() { return params[LOOP_TOGGLE_PARAM].getValue() > 0.5f; }
    // True when the MODE switch is set to AD.
    bool getAD() { return params[MODE_PARAM].getValue() > 0.5f; }
    // Scope time (full graph width in seconds), clamped to 50 ms .. 5 s.
    float getTimeScale() { return rack::math::clamp(params[TIME_SCALE_PARAM].getValue(), 0.05f, 5.0f); }

    // Constructor: configures every parameter, input, output and the loop
    // light, then resets the envelope and scope state.
    FancyADSR()
    {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configParam(ATTACK_PARAM, 0.001f, 10.0f, 0.01f, "Attack", " s");
        configParam(DECAY_PARAM, 0.001f, 10.0f, 0.1f, "Decay", " s");
        configParam(SUSTAIN_PARAM, 0.0f, 1.0f, 0.8f, "Sustain", "%", 0.f, 100.f);
        configParam(RELEASE_PARAM, 0.001f, 10.0f, 0.5f, "Release", " s");
        configSwitch(ATTACK_SHAPE_PARAM, 0.f, 1.f, 1.f, "Attack shape", {"Linear", "Exponential"});
        configSwitch(RELEASE_SHAPE_PARAM, 0.f, 1.f, 1.f, "Release shape", {"Linear", "Exponential"});
        configSwitch(LOOP_TOGGLE_PARAM, 0.f, 1.f, 0.f, "Loop mode", {"Off", "On"});
        configParam(TIME_SCALE_PARAM, 0.05f, 5.0f, 1.0f, "Scope time (full width)", " s");
        configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Envelope mode", {"ADSR", "AD (gate acts as trigger)"});
        configButton(TEST_PARAM, "Test gate (held = gate high)");

        configInput(GATE_INPUT, "Gate");
        configInput(TRIGGER_INPUT, "Retrigger");
        configInput(ATTACK_CV_INPUT, "Attack CV (5 V adds 1 s)");
        configInput(DECAY_CV_INPUT, "Decay CV (5 V adds 1 s)");
        configInput(SUSTAIN_CV_INPUT, "Sustain CV (5 V adds 100%)");
        configInput(RELEASE_CV_INPUT, "Release CV (5 V adds 1 s)");
        configOutput(ENV_OUTPUT, "Envelope (0 to 10 V)");
        configOutput(EOR_OUTPUT, "End of release (end of decay in AD mode)");

        configLight(LOOP_LIGHT, "Loop mode");

        resetState();
    }

    // Clears the envelope, stage, EOR pulse and scope history to a known
    // starting state.
    void resetState()
    {
        stage = STAGE_IDLE;
        env = 0.0f;
        stageTime = 0.0f;
        attackStart = 0.0f;
        decayStart = 1.0f;
        releaseStart = 0.0f;
        eorPulseRemaining = 0.0f;
        lastGateState = false;
        lastTriggerState = false;
        historyCount = 0;
        historyWritePos = 0;
        sampleAccumulator = 0.0f;
        for (int i = 0; i < HISTORY_SIZE; ++i)
            history[i] = 0.0f;
    }

    // Rack reset hook: clears envelope and scope state.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        resetState();
    }

    // Helper: exponential that reaches target (1 or sustain) after 'duration' seconds
    // shape: 0=linear, 1=exponential (with proper timing)
    // Computes the value of one envelope segment at time `t`: linear when
    // `shape` is 0, otherwise exponential with tau chosen so the value is
    // within 0.1% of `target` at t = duration.
    float runSegment(float start, float target, float duration, float t, float shape)
    {
        if (duration <= 0.0f)
            return target;
        float alpha = rack::math::clamp(t / duration, 0.0f, 1.0f);
        if (shape <= 0.5f) // linear
        {
            return start + (target - start) * alpha;
        }
        else // exponential: tau = duration / -ln(0.001) so that at t=duration, value is within 0.1% of target
        {
            const float eps = 0.001f;
            float tau = duration / -std::log(eps);
            float factor = 1.0f - std::exp(-t / tau);
            return start + (target - start) * factor;
        }
    }

    // Starts the attack stage. In ADSR mode it always rises from 0 (the
    // original behaviour); in AD mode it rises from the current level so a
    // retrigger mid-envelope does not snap to zero.
    void startAttack(bool fromCurrentLevel)
    {
        stage = STAGE_ATTACK;
        stageTime = 0.0f;
        attackStart = fromCurrentLevel ? env : 0.0f;
    }

    // Handles a gate transition.
    // ADSR mode: a rising edge starts (or restarts) the attack from
    // IDLE/RELEASE (and from SUSTAIN when looping); a falling edge starts the
    // release from ATTACK/DECAY/SUSTAIN.
    // AD mode: a rising edge starts the envelope from any stage; the falling
    // edge is ignored (the gate acts only as a trigger).
    void onGate(bool high)
    {
        if (getAD())
        {
            if (high)
                startAttack(true);
            return;
        }

        if (high)
        {
            if (stage == STAGE_IDLE || stage == STAGE_RELEASE)
            {
                startAttack(false);
            }
            else if (stage == STAGE_SUSTAIN && getLoop())
            {
                startAttack(false);
            }
        }
        else
        {
            if (stage == STAGE_ATTACK || stage == STAGE_DECAY || stage == STAGE_SUSTAIN)
            {
                stage = STAGE_RELEASE;
                stageTime = 0.0f;
                releaseStart = env; // capture once: the release falls from here
            }
        }
    }

    // Handles a trigger edge.
    // ADSR mode: while not idle, a rising trigger restarts the attack from
    // the beginning.
    // AD mode: a rising trigger starts the envelope from any stage.
    void onTrigger(bool high)
    {
        if (!high)
            return;
        if (getAD())
        {
            startAttack(true);
        }
        else if (stage != STAGE_IDLE)
        {
            startAttack(false);
        }
    }

    // Main audio callback: reads gate/trigger edges, evaluates the current
    // stage's envelope value, advances stages when their durations elapse,
    // writes the envelope and EOR outputs, updates the loop light, and
    // samples the envelope into the scope history at the chosen time base.
    void process(const ProcessArgs &args) override
    {
        float sampleTime = args.sampleTime;
        const bool adMode = getAD();

        // Gate / trigger edges
        // The TEST button acts as a gate for as long as it is held.
        bool gateHigh = inputs[GATE_INPUT].getVoltage() > 1.0f
                     || params[TEST_PARAM].getValue() > 0.5f;
        if (gateHigh && !lastGateState)
            onGate(true);
        else if (!gateHigh && lastGateState)
            onGate(false);
        lastGateState = gateHigh;

        bool triggerHigh = inputs[TRIGGER_INPUT].getVoltage() > 1.0f;
        if (triggerHigh && !lastTriggerState)
            onTrigger(true);
        lastTriggerState = triggerHigh;

        // Switching to AD mode while sitting in SUSTAIN: fall away to zero
        // through the decay stage instead of hanging forever.
        if (adMode && stage == STAGE_SUSTAIN)
        {
            stage = STAGE_DECAY;
            stageTime = 0.0f;
            decayStart = env;
        }

        float attackTime = getAttack();
        float decayTime = getDecay();
        float sustainLev = getSustain();
        float releaseTime = getRelease();
        float attackShape = params[ATTACK_SHAPE_PARAM].getValue();
        float releaseShape = params[RELEASE_SHAPE_PARAM].getValue();

        // Advance envelope
        switch (stage)
        {
        case STAGE_ATTACK:
            env = runSegment(attackStart, 1.0f, attackTime, stageTime, attackShape);
            stageTime += sampleTime;
            if (stageTime >= attackTime)
            {
                env = 1.0f;
                stage = STAGE_DECAY;
                stageTime = 0.0f;
                decayStart = 1.0f;
            }
            break;

        case STAGE_DECAY:
            // Decay is always exponential (typical ADSR), timing is correct.
            // ADSR mode falls to the sustain level; AD mode falls to zero.
            env = runSegment(adMode ? decayStart : 1.0f, adMode ? 0.0f : sustainLev,
                             decayTime, stageTime, 1.0f);
            stageTime += sampleTime;
            if (stageTime >= decayTime)
            {
                if (adMode)
                {
                    env = 0.0f;
                    eorPulseRemaining = 0.005f; // 5 ms pulse at the end of the decay
                    eorFlash = 1.f;
                    if (getLoop() && gateHigh)
                    {
                        startAttack(false); // loop: repeat while the gate is held
                    }
                    else
                    {
                        stage = STAGE_IDLE;
                        stageTime = 0.0f;
                    }
                }
                else
                {
                    env = sustainLev;
                    stage = STAGE_SUSTAIN;
                    stageTime = 0.0f;
                }
            }
            break;

        case STAGE_SUSTAIN:
            env = sustainLev;
            if (getLoop() && gateHigh)
            {
                startAttack(false);
            }
            break;

        case STAGE_RELEASE:
            // Fall from the level captured when the release began. (Using the
            // live `env` as the start made the curve compound every sample,
            // so the release finished far faster than the RELEASE knob.)
            env = runSegment(releaseStart, 0.0f, releaseTime, stageTime, releaseShape);
            stageTime += sampleTime;
            if (stageTime >= releaseTime)
            {
                env = 0.0f;
                stage = STAGE_IDLE;
                eorPulseRemaining = 0.005f; // 5 ms pulse
                eorFlash = 1.f;
            }
            break;

        default: // IDLE
            env = 0.0f;
            break;
        }

        float outVoltage = env * 10.0f;
        outputs[ENV_OUTPUT].setVoltage(outVoltage);

        // End-of-release trigger
        if (eorPulseRemaining > 0.0f)
        {
            outputs[EOR_OUTPUT].setVoltage(10.0f);
            eorPulseRemaining -= sampleTime;
            if (eorPulseRemaining <= 0.0f)
                outputs[EOR_OUTPUT].setVoltage(0.0f);
        }
        else
        {
            outputs[EOR_OUTPUT].setVoltage(0.0f);
        }

        lights[LOOP_LIGHT].setBrightness(getLoop() ? 1.0f : 0.0f);

        // ---- display data ----
        dispAttack = attackTime;
        dispDecay = decayTime;
        dispSustain = sustainLev;
        dispRelease = releaseTime;
        dispAttackShape = attackShape;
        dispReleaseShape = releaseShape;
        dispGate = gateHigh;
        dispAD = adMode;
        eorFlash = std::max(0.f, eorFlash - 5.f * sampleTime);

        // ----- Real-time graph sampling -----
        // Records the envelope output into a ring buffer at the current scope
        // time base (one sample every timeScale/HISTORY_SIZE seconds).
        float timeScale = getTimeScale();
        float sampleInterval = timeScale / HISTORY_SIZE;
        sampleAccumulator += sampleTime;
        while (sampleAccumulator >= sampleInterval)
        {
            history[historyWritePos] = outVoltage;
            historyWritePos = (historyWritePos + 1) % HISTORY_SIZE;
            if (historyCount < HISTORY_SIZE)
                historyCount++;
            sampleAccumulator -= sampleInterval;
        }
    }

    // For graph widget access
    // Returns the number of valid samples currently stored in the history
    // buffer (0 .. HISTORY_SIZE).
    int getHistorySize() const { return historyCount; }
    // Returns the history sample at `index`, where 0 is the oldest stored
    // sample and the newest is at index historyCount-1. Out-of-range indices
    // return 0.
    float getHistorySample(int index) const
    {
        if (index < 0 || index >= historyCount)
            return 0.0f;
        int oldest = (historyWritePos - historyCount + HISTORY_SIZE) % HISTORY_SIZE;
        return history[(oldest + index) % HISTORY_SIZE];
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with same-named helpers in the other module files.
namespace
{

    // Same segment maths as the module (used for the ideal-curve preview)
    // Mirrors FancyADSR::runSegment() so the preview drawn by the UI matches
    // the envelope the DSP will produce for the same parameters.
    float segCurve(float start, float target, float dur, float t, float shape)
    {
        if (dur <= 0.f)
            return target;
        float a = std::min(std::max(t / dur, 0.f), 1.f);
        if (shape <= 0.5f)
            return start + (target - start) * a;
        const float eps = 0.001f;
        float tau = dur / -std::log(eps);
        return start + (target - start) * (1.f - std::exp(-t / tau));
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct FAPanelBackground : TransparentWidget
    {
        FAPanelBackground() { box.size = Vec(FA_PANEL_W, FA_PANEL_H); }

        // Draws all static panel art: base panel, display bezel, the
        // ENVELOPE section with its four knob cells (and CV sub-labels), the
        // five mode/shape/loop/time cells with their state labels, and the
        // I/O row with its four tinted jack cells.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, FA_PANEL_W, FA_PANEL_H, "FANCYADSR");
            agmm::bezel(vg, FA_DISP_X, FA_DISP_Y, FA_DISP_W, FA_DISP_H);

            // ---- row A: knob on top, CV jack below it ----
            agmm::sectionHeader(vg, FA_PANEL_W, FA_ENV_HEAD_Y, "ENVELOPE", 14.f);
            const char *names[4] = {"ATTACK", "DECAY", "SUSTAIN", "RELEASE"};
            for (int i = 0; i < 4; ++i)
            {
                float cx = faColX(i);
                agmm::cell(vg, cx, FA_ROWA_TOP, FA_CELL_W, FA_ROWA_H, names[i]);
                agmm::text(vg, cx + 16.f, FA_ROWA_TOP + 58.f, "CV", 7.f, agmm::dim(), agmm::ALIGN_C);
            }

            // ---- row B: mode, shapes, loop, time (label above, state names beside the switches) ----
            const char *b[5] = {"MODE", "ATK SHAPE", "REL SHAPE", "LOOP", "TIME"};
            for (int i = 0; i < 5; ++i)
                agmm::cell(vg, faColX5(i), FA_ROWB_TOP, FA_CELL5_W, FA_ROWB_H, b[i]);
            const float by = FA_ROWB_TOP + 30.f;
            // MODE: switch up = AD (value 1), down = ADSR (value 0)
            agmm::text(vg, faColX5(0) + 13.f, by - 8.f, "AD", 7.f, agmm::dim(), agmm::ALIGN_C, 0.3f);
            agmm::text(vg, faColX5(0) + 13.f, by + 8.f, "ADSR", 7.f, agmm::dim(), agmm::ALIGN_C, 0.3f);
            for (int i = 1; i < 3; ++i)
            {
                agmm::text(vg, faColX5(i) + 13.f, by - 8.f, "EXP", 7.f, agmm::dim(), agmm::ALIGN_C, 0.3f);
                agmm::text(vg, faColX5(i) + 13.f, by + 8.f, "LIN", 7.f, agmm::dim(), agmm::ALIGN_C, 0.3f);
            }

            // ---- row C: I/O (outputs tinted) ----
            agmm::sectionHeader(vg, FA_PANEL_W, FA_IO_HEAD_Y, "I / O", 14.f);
            const char *io[4] = {"GATE", "RETRIG", "ENV OUT", "EOR"};
            for (int i = 0; i < 4; ++i)
                agmm::cell(vg, faColX(i), FA_ROWC_TOP, FA_CELL_W, FA_ROWC_H, io[i], i >= 2);
            // label for the TEST button that sits beside the GATE jack
            agmm::text(vg, faColX(0) + 17.f, FA_ROWC_TOP + 42.f, "TEST", 7.f, agmm::dim(), agmm::ALIGN_C, 0.3f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope display: live envelope trace + ideal-curve preview
    // ------------------------------------------------------------
    struct FADisplayWidget : TransparentWidget
    {
        FancyADSR *module = nullptr;

        FADisplayWidget(FancyADSR *m) : module(m)
        {
            box.pos = Vec(FA_DISP_X, FA_DISP_Y);
            box.size = Vec(FA_DISP_W, FA_DISP_H);
        }

        // Maps a voltage (0..10 V) to a y pixel inside the scope rectangle.
        static float vy(float v)
        { // volts (0..10) to display y
            return FA_SY0 + FA_SH - 1.f - std::min(std::max(v / 10.f, 0.f), 1.f) * (FA_SH - 2.f);
        }

        // Static background: screen, scope field, grid, voltage ticks
        // Draws the display screen, the scope frame, its 10x4 grid, and the
        // "0"/"10" voltage labels in the left column.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, FA_DISP_W, FA_DISP_H);
            agmm::plotFrame(vg, FA_SX0, FA_SY0, FA_SW, FA_SH);

            // Grid: 10 time divisions, 0 / 2.5 / 5 / 7.5 / 10 V
            nvgBeginPath(vg);
            for (int i = 1; i < 10; ++i)
            {
                float x = FA_SX0 + FA_SW * i / 10.f + 0.5f;
                nvgMoveTo(vg, x, FA_SY0);
                nvgLineTo(vg, x, FA_SY0 + FA_SH);
            }
            for (int k = 1; k < 4; ++k)
            {
                float y = FA_SY0 + FA_SH * k / 4.f + 0.5f;
                nvgMoveTo(vg, FA_SX0, y);
                nvgLineTo(vg, FA_SX0 + FA_SW, y);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Voltage ticks and labels in the left column (outside the scope)
            nvgBeginPath(vg);
            for (int k = 0; k <= 4; ++k)
            {
                float y = FA_SY0 + FA_SH - k * (FA_SH - 1.f) / 4.f + 0.5f;
                nvgMoveTo(vg, FA_SX0 - 4.f, y);
                nvgLineTo(vg, FA_SX0 - 1.f, y);
            }
            nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 80));
            nvgStroke(vg);
            agmm::text(vg, 7.f, FA_SY0 + 4.f, "10", 7.f, agmm::dim(), agmm::ALIGN_C, 0.f);
            agmm::text(vg, 7.f, FA_SY0 + FA_SH - 4.f, "0", 7.f, agmm::dim(), agmm::ALIGN_C, 0.f);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the readout row, the violet ideal-envelope
        // preview, the cyan live trace with a fill and end dot, the level
        // meter, the legend row, and the stage/state chips (with an EOR
        // flash ring). Falls back to a sample envelope in the module browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = FA_DISP_W;

            // ---- gather state (live, or a sample scene in the module browser) ----
            float A, D, S, R, aShape, rShape, envV, timeScale;
            bool gate, loop, ad;
            int stage;
            float eor;
            int n;
            float samples[FancyADSR::HISTORY_SIZE];

            if (module)
            {
                A = module->dispAttack;
                D = module->dispDecay;
                S = module->dispSustain;
                R = module->dispRelease;
                aShape = module->dispAttackShape;
                rShape = module->dispReleaseShape;
                envV = module->env * 10.f;
                timeScale = module->getTimeScale();
                gate = module->dispGate;
                ad = module->dispAD;
                loop = module->params[FancyADSR::LOOP_TOGGLE_PARAM].getValue() > 0.5f;
                stage = (int)module->stage;
                eor = module->eorFlash;
                n = std::min(std::max(module->getHistorySize(), 0), (int)FancyADSR::HISTORY_SIZE);
                for (int i = 0; i < n; ++i)
                    samples[i] = module->getHistorySample(i);
            }
            else
            {
                A = 0.05f;
                D = 0.3f;
                S = 0.6f;
                R = 0.8f;
                aShape = 1.f;
                rShape = 1.f;
                timeScale = 2.f;
                gate = false;
                loop = false;
                ad = false;
                stage = FancyADSR::STAGE_RELEASE;
                eor = 0.f;
                n = 300;
                float total = A + D + R + 0.25f * (A + D + R);
                for (int i = 0; i < n; ++i)
                {
                    float t = total * (float)i / (float)n;
                    float hold = A + D + 0.25f * (A + D + R);
                    float v;
                    if (t < A)
                        v = segCurve(0.f, 1.f, A, t, aShape);
                    else if (t < A + D)
                        v = segCurve(1.f, S, D, t - A, 1.f);
                    else if (t < hold)
                        v = S;
                    else
                        v = segCurve(S, 0.f, R, t - hold, rShape);
                    samples[i] = v * 10.f;
                }
                envV = samples[n - 1];
            }

            char buf[64];

            // ---- readout row ----
            if (ad)
                snprintf(buf, sizeof(buf), "A %.2f  D %.2f  S --  R --", A, D);
            else
                snprintf(buf, sizeof(buf), "A %.2f  D %.2f  S %.0f%%  R %.2f", A, D, S * 100.f, R);
            agmm::text(vg, 6.f, 9.f, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "%.1fV", envV);
            agmm::text(vg, W - 6.f, 9.f, buf, 7.f, nvgRGB(0xff, 0xe6, 0x80), agmm::ALIGN_R, 0.5f);

            // ---- ideal preview (the shape the current knobs describe) ----
            // ADSR: attack, decay to sustain, hold, release.
            // AD:   attack, decay to zero, short tail of silence.
            {
                float hold = ad ? 0.25f * (A + D) : 0.25f * (A + D + R);
                float total = ad ? (A + D + hold) : (A + D + hold + R);
                const int N = 100;
                nvgBeginPath(vg);
                for (int i = 0; i <= N; ++i)
                {
                    float t = total * (float)i / (float)N;
                    float v;
                    if (t < A)
                        v = segCurve(0.f, 1.f, A, t, aShape);
                    else if (t < A + D)
                        v = segCurve(1.f, ad ? 0.f : S, D, t - A, 1.f);
                    else if (t < A + D + hold)
                        v = ad ? 0.f : S;
                    else
                        v = ad ? 0.f : segCurve(S, 0.f, R, t - A - D - hold, rShape);
                    float x = FA_SX0 + FA_SW * (float)i / (float)N;
                    float y = vy(v * 10.f);
                    if (i == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, agmm::withA(agmm::violet(), 0.6f));
                nvgStrokeWidth(vg, 1.f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);
            }

            // ---- live trace (flat fill + 1.5 px line) ----
            nvgSave(vg);
            nvgScissor(vg, FA_SX0, FA_SY0, FA_SW, FA_SH);
            if (n >= 2)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, FA_SX0, FA_SY0 + FA_SH);
                for (int i = 0; i < n; ++i)
                {
                    float x = FA_SX0 + FA_SW * (float)i / (float)(FancyADSR::HISTORY_SIZE - 1);
                    nvgLineTo(vg, x, vy(samples[i]));
                }
                nvgLineTo(vg, FA_SX0 + FA_SW * (float)(n - 1) / (float)(FancyADSR::HISTORY_SIZE - 1), FA_SY0 + FA_SH);
                nvgClosePath(vg);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                nvgFill(vg);

                nvgBeginPath(vg);
                for (int i = 0; i < n; ++i)
                {
                    float x = FA_SX0 + FA_SW * (float)i / (float)(FancyADSR::HISTORY_SIZE - 1);
                    float y = vy(samples[i]);
                    if (i == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgLineJoin(vg, NVG_ROUND);
                nvgStrokeColor(vg, agmm::bright());
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);

                float hx = FA_SX0 + FA_SW * (float)(n - 1) / (float)(FancyADSR::HISTORY_SIZE - 1);
                nvgBeginPath(vg);
                nvgCircle(vg, hx, vy(samples[n - 1]), 2.f);
                nvgFillColor(vg, nvgRGB(255, 255, 255));
                nvgFill(vg);
            }
            nvgRestore(vg);

            // ---- level meter (own column right of the scope) ----
            agmm::meter(vg, FA_SX0 + FA_SW + 8.f, FA_SY0, 6.f, FA_SH, envV / 10.f);

            // ---- legend row: preview / live / scope time (no text over the scope) ----
            {
                const float y = FA_LEGEND_Y;
                float x = FA_SX0;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x, y);
                nvgLineTo(vg, x + 10.f, y);
                nvgStrokeColor(vg, agmm::withA(agmm::violet(), 0.8f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                agmm::text(vg, x + 14.f, y, "PREVIEW", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
                x += 14.f + agmm::textWidth(vg, "PREVIEW", 7.f, 0.5f) + 12.f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x, y);
                nvgLineTo(vg, x + 10.f, y);
                nvgStrokeColor(vg, agmm::bright());
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
                agmm::text(vg, x + 14.f, y, "LIVE", 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);

                snprintf(buf, sizeof(buf), "%.2fS FULL WIDTH", timeScale);
                agmm::text(vg, FA_SX0 + FA_SW, y, buf, 7.f, agmm::dim(), agmm::ALIGN_R, 0.5f);
            }

            // ---- stage / state chips (widths measured from the text) ----
            // SUS and REL are never active in AD mode; the EOR flash ring
            // moves to the DEC chip there, since AD ends at the end of decay.
            {
                const char *txt[6] = {"ATK", "DEC", "SUS", "REL", "GATE", "LOOP"};
                bool on[6] = {stage == FancyADSR::STAGE_ATTACK, stage == FancyADSR::STAGE_DECAY,
                              stage == FancyADSR::STAGE_SUSTAIN, stage == FancyADSR::STAGE_RELEASE, gate, loop};
                NVGcolor col[6] = {agmm::accent(), agmm::accent(), agmm::accent(), agmm::violet(),
                                   agmm::accent(), agmm::violet()};
                const int flashChip = ad ? 1 : 3;
                float x = 8.f;
                for (int i = 0; i < 6; ++i)
                {
                    float w = agmm::textWidth(vg, txt[i], 7.f, 0.5f) + 10.f;
                    agmm::chip(vg, x, FA_CHIP_Y, w, FA_CHIP_H, txt[i], on[i], col[i]);
                    if (i == flashChip && eor > 0.02f)
                    { // end-of-envelope flash ring around the REL (or DEC) chip
                        nvgBeginPath(vg);
                        nvgRoundedRect(vg, x - 1.5f, FA_CHIP_Y - 1.5f, w + 3.f, FA_CHIP_H + 3.f, 4.f);
                        nvgStrokeColor(vg, agmm::withA(agmm::violet(), 0.9f * eor));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }
                    x += w + (i == 3 ? 10.f : 4.f);
                }
            }

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module Widget: adds the static panel, screws, the live display, the four
// envelope knobs with their CV jacks, the mode/shape/loop/time controls, and
// the gate/retrigger inputs plus envelope/EOR outputs.
// ------------------------------------------------------------
struct FancyADSRWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, the scope display,
    // row A (four knobs + four CV jacks), row B (mode switch, two shape
    // switches, the loop switch plus its light, and the time knob), and row C
    // (two inputs + two outputs).
    FancyADSRWidget(FancyADSR *module)
    {
        setModule(module);
        box.size = Vec(FA_PANEL_W, FA_PANEL_H);

        addChild(new FAPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new FADisplayWidget(module));

        // ---- row A: knob on top, CV jack below ----
        const int knobIds[4] = {FancyADSR::ATTACK_PARAM, FancyADSR::DECAY_PARAM,
                                FancyADSR::SUSTAIN_PARAM, FancyADSR::RELEASE_PARAM};
        const int cvIds[4] = {FancyADSR::ATTACK_CV_INPUT, FancyADSR::DECAY_CV_INPUT,
                              FancyADSR::SUSTAIN_CV_INPUT, FancyADSR::RELEASE_CV_INPUT};
        for (int i = 0; i < 4; ++i)
        {
            float cx = faColX(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, FA_ROWA_TOP + 30.f), module, knobIds[i]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 8.f, FA_ROWA_TOP + 58.f), module, cvIds[i]));
        }

        // ---- row B: mode, shapes, loop, time ----
        const float by = FA_ROWB_TOP + 30.f;
        addParam(createParamCentered<CKSS>(Vec(faColX5(0) - 9.f, by), module, FancyADSR::MODE_PARAM));
        addParam(createParamCentered<CKSS>(Vec(faColX5(1) - 9.f, by), module, FancyADSR::ATTACK_SHAPE_PARAM));
        addParam(createParamCentered<CKSS>(Vec(faColX5(2) - 9.f, by), module, FancyADSR::RELEASE_SHAPE_PARAM));
        addParam(createParamCentered<CKSS>(Vec(faColX5(3) - 9.f, by), module, FancyADSR::LOOP_TOGGLE_PARAM));
        addChild(createLightCentered<SmallLight<agmm::CyanLight>>(Vec(faColX5(3) + 12.f, by), module, FancyADSR::LOOP_LIGHT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(faColX5(4), by), module, FancyADSR::TIME_SCALE_PARAM));

        // ---- row C: I/O ----
        const float cy = FA_ROWC_TOP + 31.f;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(faColX(0) - 10.f, cy), module, FancyADSR::GATE_INPUT));
        addParam(createParamCentered<TL1105>(Vec(faColX(0) + 17.f, cy - 3.f), module, FancyADSR::TEST_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(faColX(1), cy), module, FancyADSR::TRIGGER_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(faColX(2), cy), module, FancyADSR::ENV_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(faColX(3), cy), module, FancyADSR::EOR_OUTPUT));
    }
};

Model *modelFancyADSR = createModel<FancyADSR, FancyADSRWidget>("FancyADSR");