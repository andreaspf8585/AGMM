#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <random>
#include <limits>
#include "Knob_arcs.hpp"

// ============================================================
// PsyLFO
// ============================================================
//
// OVERVIEW
// --------
// PsyLFO is a 28 HP chaotic modulation source built from a real double
// pendulum physics simulation. Two coupled pendulum bobs swing under gravity
// with configurable mass/length, integrated with RK4 at the audio rate. The
// module exposes, per pendulum:
//   - X / Y position outputs (in volts, ±5 V, scaled to the total reach)
//   - a V/Oct output that latches a new random voltage on every zero
//     crossing of that pendulum's angle
//   - a gate output that fires for a selectable duration on the same zero
//     crossings
// Global controls are Speed (time-scaling), Gravity, Damping, plus Kick and
// Reset buttons and their CV inputs. A 200 px circular display draws the
// double pendulum in real time with fading trails, angle arcs, mass-sized
// bobs and gate flash rings.
//
// File structure:
//   1. Layout constants (28 HP = 420x380 px).
//   2. PsyLFO module: pendulum state, random V/Oct, gate timers, RK4 physics,
//      JSON persistence, and the display trail buffer.
//   3. Anonymous-namespace UI: colour helpers, drawRod/drawBob/drawHub/
//      drawTrail, PLPanelBackground, PLDisplayWidget.
//   4. PsyLFOWidget: places LEDs, knobs, outputs, the global control row,
//      and the context menu.
//   5. Model registration.
// ============================================================

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int PL_TRAIL_N = 120;      // trail points per bob
static const float PL_TRAIL_HZ = 150.f; // trail sample rate

static const float PL_PANEL_W = RACK_GRID_WIDTH * 28; // 420
static const float PL_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float PL_DISP_W = 200.f;
static const float PL_DISP_H = 200.f;
static const float PL_DISP_X = (PL_PANEL_W - PL_DISP_W) / 2.f; // 110
static const float PL_DISP_Y = 46.f;

// Side blocks: pendulum 1 on the left, pendulum 2 on the right (88 px wide each)
static const float PL_LBLOCK_X = 12.f, PL_RBLOCK_X = 320.f, PL_BLOCK_W = 88.f;
static const float PL_LCOL[2] = {34.f, 78.f};
static const float PL_RCOL[2] = {342.f, 386.f};
static const float PL_CAP_Y = 46.f;
static const float PL_LED_Y = 62.f;
static const float PL_CELL_W = 40.f;
static const float PL_KNOB_TOP = 72.f, PL_KNOB_H = 62.f; // MASS / LENGTH cells
static const float PL_OUT_TOP[2] = {142.f, 198.f};       // X Y / V/OCT GATE
static const float PL_OUT_H = 52.f;
// Returns the y centre of the MASS / LENGTH knob row.
static float plKnobY() { return PL_KNOB_TOP + 38.f; }
// Returns the y centre of the output jack row `row` (0 = X/Y, 1 = V/OCT/GATE).
static float plOutY(int row) { return PL_OUT_TOP[row] + 32.f; }

// Bottom control row: 8 cells
static const float PL_CTRL_CAP_Y = 268.f;
static const float PL_CTRL_TOP = 278.f;
static const float PL_CTRL_H = 52.f;
static const float PL_CTRL_W = 46.f;
// Returns the horizontal centre of global-control cell i (0..7).
static float plCtrlX(int i) { return 35.f + 50.f * (float)i; }
// Returns the y centre of the global-control row.
static float plCtrlY() { return PL_CTRL_TOP + 34.f; }

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct PsyLFO : Module
{
    enum ParamId
    {
        SPEED_PARAM,
        MASS1_PARAM,
        MASS2_PARAM,
        LEN1_PARAM,
        LEN2_PARAM,
        RESET_BUTTON,
        GRAVITY_PARAM, // new
        DAMPING_PARAM, // new
        KICK_BUTTON,   // new
        PARAMS_LEN
    };
    enum InputId
    {
        RESET_CV_INPUT,
        SPEED_CV_INPUT, // new
        KICK_INPUT,     // new
        INPUTS_LEN
    };
    enum OutputId
    {
        X1_OUTPUT,
        Y1_OUTPUT,
        X2_OUTPUT,
        Y2_OUTPUT,
        VOCT1_OUTPUT,
        VOCT2_OUTPUT,
        GATE1_OUTPUT,
        GATE2_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        MASS1_LIGHT,
        MASS2_LIGHT,
        LIGHTS_LEN
    };

    // Voct range selection
    enum VoctRange
    {
        RANGE_0_1,
        RANGE_N1_1,
        RANGE_N2_2,
        RANGE_N5_5,
        RANGE_0_5,
        NUM_RANGES
    };
    // Gate duration selection (in ms, except trigger which is 0.5 ms)
    enum GateDuration
    {
        GATE_TRIGGER,
        GATE_5MS,
        GATE_10MS,
        GATE_20MS,
        GATE_50MS,
        GATE_100MS,
        GATE_200MS,
        NUM_GATE_DURS
    };

    double theta1 = 0.0, theta2 = 0.0;
    double omega1 = 0.0, omega2 = 0.0;
    double mass1 = 1.0, mass2 = 1.0;
    double len1 = 1.0, len2 = 1.0;
    double speedFactor = 1.0;
    double g = 9.81;
    double damping = 0.0;
    const double eps = 1e-8;

    // Voct state
    double voct1Voltage = 0.0;
    double voct2Voltage = 0.0;
    double prevTheta1Sign = 1.0;
    double prevTheta2Sign = 1.0;

    // Gate pulse timers (sample count)
    int gate1Timer = 0;
    int gate2Timer = 0;

    // User settings
    VoctRange voct1Range = RANGE_N1_1;
    VoctRange voct2Range = RANGE_N1_1;
    GateDuration gate1Duration = GATE_5MS;
    GateDuration gate2Duration = GATE_5MS;

    bool lastResetState = false;
    bool lastKickState = false;
    std::mt19937 rng;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float trailX1[PL_TRAIL_N] = {};
    float trailY1[PL_TRAIL_N] = {};
    float trailX2[PL_TRAIL_N] = {};
    float trailY2[PL_TRAIL_N] = {};
    int trailHead = 0;
    int trailCount = 0;
    float trailTimer = 0.f;
    float flash1 = 0.f, flash2 = 0.f;

    // Constructor: configures every parameter (with units/ranges), the CV
    // inputs and outputs, seeds the RNG, randomises the initial pendulum
    // state, and generates the initial random V/Oct voltages.
    PsyLFO()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        // Speed is a time-scale factor: 0.2x .. 10x
        configParam(SPEED_PARAM, 0.f, 1.f, 0.3f, "Speed", "x", 0.f, 9.8f, 0.2f);
        configParam(MASS1_PARAM, 0.1f, 10.f, 1.f, "Mass 1", " kg");
        configParam(MASS2_PARAM, 0.1f, 10.f, 1.f, "Mass 2", " kg");
        configParam(LEN1_PARAM, 0.1f, 3.f, 1.f, "Length 1", " m");
        configParam(LEN2_PARAM, 0.1f, 3.f, 1.f, "Length 2", " m");
        configButton(RESET_BUTTON, "Reset");
        configParam(GRAVITY_PARAM, 1.f, 25.f, 9.81f, "Gravity", " m/s²");
        configParam(DAMPING_PARAM, 0.f, 1.f, 0.f, "Damping", "%", 0.f, 100.f);
        configButton(KICK_BUTTON, "Kick");

        configInput(RESET_CV_INPUT, "Reset gate");
        configInput(SPEED_CV_INPUT, "Speed CV (0 to 10 V)");
        configInput(KICK_INPUT, "Kick trigger");

        configOutput(X1_OUTPUT, "Mass 1 X");
        configOutput(Y1_OUTPUT, "Mass 1 Y");
        configOutput(X2_OUTPUT, "Mass 2 X");
        configOutput(Y2_OUTPUT, "Mass 2 Y");
        configOutput(VOCT1_OUTPUT, "V/Oct 1");
        configOutput(VOCT2_OUTPUT, "V/Oct 2");
        configOutput(GATE1_OUTPUT, "Gate 1");
        configOutput(GATE2_OUTPUT, "Gate 2");

        std::random_device rd;
        rng.seed(rd());
        resetState();

        // Initial random voltages
        generateRandomVoct1();
        generateRandomVoct2();
    }

    // Rack reset hook: randomises the pendulum state and clears display/gate
    // state.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        resetState();
    }

    // Randomise the pendulums and clear the display/gate state
    // Sets random angles and angular velocities for both pendulums, records
    // their starting signs, zeroes the gate timers and clears the trails.
    void resetState()
    {
        std::uniform_real_distribution<double> angleDist(-M_PI, M_PI);
        std::uniform_real_distribution<double> omegaDist(-2.0, 2.0);
        theta1 = angleDist(rng);
        theta2 = angleDist(rng);
        omega1 = omegaDist(rng);
        omega2 = omegaDist(rng);

        // Reset signs for zero-crossing detection
        prevTheta1Sign = (theta1 >= 0) ? 1.0 : -1.0;
        prevTheta2Sign = (theta2 >= 0) ? 1.0 : -1.0;

        gate1Timer = 0;
        gate2Timer = 0;

        trailHead = 0;
        trailCount = 0;
        trailTimer = 0.f;
        flash1 = flash2 = 0.f;
    }

    // Random angular impulse
    // Adds a random angular velocity to each pendulum (used by Kick), which
    // is enough to destabilise a near-still double pendulum.
    void kick()
    {
        std::uniform_real_distribution<double> d1(-4.0, 4.0);
        std::uniform_real_distribution<double> d2(-6.0, 6.0);
        omega1 += d1(rng);
        omega2 += d2(rng);
    }

    // Helper: map random value in [-1,1] to selected range
    // Scales a raw random value in [-1,1] into the voltage range selected
    // by the VoctRange enum.
    double scaleVoct(double raw, VoctRange range)
    {
        switch (range)
        {
        case RANGE_0_1:
            return (raw + 1.0) / 2.0; // 0..1
        case RANGE_N1_1:
            return raw; // -1..1
        case RANGE_N2_2:
            return raw * 2.0; // -2..2
        case RANGE_N5_5:
            return raw * 5.0; // -5..5
        case RANGE_0_5:
            return (raw + 1.0) / 2.0 * 5.0; // 0..5
        default:
            return raw;
        }
    }

    // Generates a new random V/Oct voltage for pendulum 1 in its selected
    // range.
    void generateRandomVoct1()
    {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        voct1Voltage = scaleVoct(raw, voct1Range);
    }

    // Generates a new random V/Oct voltage for pendulum 2 in its selected
    // range.
    void generateRandomVoct2()
    {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        voct2Voltage = scaleVoct(raw, voct2Range);
    }

    // Returns the length in seconds of the requested gate duration.
    double getGateDurationSeconds(GateDuration dur)
    {
        switch (dur)
        {
        case GATE_TRIGGER:
            return 0.0005; // 0.5 ms
        case GATE_5MS:
            return 0.005;
        case GATE_10MS:
            return 0.010;
        case GATE_20MS:
            return 0.020;
        case GATE_50MS:
            return 0.050;
        case GATE_100MS:
            return 0.100;
        case GATE_200MS:
            return 0.200;
        default:
            return 0.005;
        }
    }

    // Reads the global controls (Speed with CV, Mass, Length, Gravity,
    // Damping) into the simulation state, so the RK4 integration and the
    // output scaling use the current settings.
    void updateParameters()
    {
        float speedKnob = params[SPEED_PARAM].getValue();
        if (inputs[SPEED_CV_INPUT].isConnected())
            speedKnob = rack::math::clamp(speedKnob + inputs[SPEED_CV_INPUT].getVoltage() / 10.f, 0.f, 1.f);
        speedFactor = 0.2 + speedKnob * 9.8;

        mass1 = params[MASS1_PARAM].getValue();
        mass2 = params[MASS2_PARAM].getValue();
        len1 = params[LEN1_PARAM].getValue();
        len2 = params[LEN2_PARAM].getValue();
        g = params[GRAVITY_PARAM].getValue();
        damping = params[DAMPING_PARAM].getValue() * 2.0; // 1/s of simulated time
    }

    // Returns true when all four state variables are finite (guards against
    // numerical blow-ups from the RK4 integrator).
    bool isValid()
    {
        return std::isfinite(theta1) && std::isfinite(theta2) &&
               std::isfinite(omega1) && std::isfinite(omega2);
    }

    // Computes the angular accelerations of the double pendulum for the
    // given state, using the standard coupled rigid-body equations, plus a
    // simple linear damping term on each angular velocity.
    void computeDerivatives(double t1, double t2, double w1, double w2,
                            double &dw1, double &dw2)
    {
        double sin_d = sin(t2 - t1);
        double cos_d = cos(t2 - t1);
        double denom1 = (mass1 + mass2) * len1 - mass2 * len1 * cos_d * cos_d + eps;
        double denom2 = len2 * (mass1 + mass2 - mass2 * cos_d * cos_d) + eps;

        dw1 = (mass2 * len1 * w1 * w1 * sin_d * cos_d +
               mass2 * g * sin(t2) * cos_d +
               mass2 * len2 * w2 * w2 * sin_d -
               (mass1 + mass2) * g * sin(t1)) /
              denom1;

        dw2 = (-mass2 * len2 * w2 * w2 * sin_d * cos_d +
               (mass1 + mass2) * g * sin(t1) * cos_d -
               (mass1 + mass2) * len1 * w1 * w1 * sin_d -
               (mass1 + mass2) * g * sin(t2)) /
              denom2;

        // Pivot friction (damping = 0 leaves the original behaviour)
        dw1 -= damping * w1;
        dw2 -= damping * w2;
    }

    // Advances the double-pendulum state by dt seconds using the classical
    // 4th-order Runge-Kutta scheme, then wraps both angles into [-π, π].
    void rk4(double dt)
    {
        double k1_t1, k1_t2, k1_w1, k1_w2;
        double k2_t1, k2_t2, k2_w1, k2_w2;
        double k3_t1, k3_t2, k3_w1, k3_w2;
        double k4_t1, k4_t2, k4_w1, k4_w2;

        computeDerivatives(theta1, theta2, omega1, omega2, k1_w1, k1_w2);
        k1_t1 = omega1;
        k1_t2 = omega2;

        double t1 = theta1 + 0.5 * dt * k1_t1;
        double t2 = theta2 + 0.5 * dt * k1_t2;
        double w1 = omega1 + 0.5 * dt * k1_w1;
        double w2 = omega2 + 0.5 * dt * k1_w2;
        computeDerivatives(t1, t2, w1, w2, k2_w1, k2_w2);
        k2_t1 = w1;
        k2_t2 = w2;

        t1 = theta1 + 0.5 * dt * k2_t1;
        t2 = theta2 + 0.5 * dt * k2_t2;
        w1 = omega1 + 0.5 * dt * k2_w1;
        w2 = omega2 + 0.5 * dt * k2_w2;
        computeDerivatives(t1, t2, w1, w2, k3_w1, k3_w2);
        k3_t1 = w1;
        k3_t2 = w2;

        t1 = theta1 + dt * k3_t1;
        t2 = theta2 + dt * k3_t2;
        w1 = omega1 + dt * k3_w1;
        w2 = omega2 + dt * k3_w2;
        computeDerivatives(t1, t2, w1, w2, k4_w1, k4_w2);
        k4_t1 = w1;
        k4_t2 = w2;

        theta1 += dt * (k1_t1 + 2.0 * k2_t1 + 2.0 * k3_t1 + k4_t1) / 6.0;
        theta2 += dt * (k1_t2 + 2.0 * k2_t2 + 2.0 * k3_t2 + k4_t2) / 6.0;
        omega1 += dt * (k1_w1 + 2.0 * k2_w1 + 2.0 * k3_w1 + k4_w1) / 6.0;
        omega2 += dt * (k1_w2 + 2.0 * k2_w2 + 2.0 * k3_w2 + k4_w2) / 6.0;

        theta1 = fmod(theta1 + M_PI, 2.0 * M_PI);
        if (theta1 < 0)
            theta1 += 2.0 * M_PI;
        theta1 -= M_PI;
        theta2 = fmod(theta2 + M_PI, 2.0 * M_PI);
        if (theta2 < 0)
            theta2 += 2.0 * M_PI;
        theta2 -= M_PI;
    }

    // Main audio callback: reads parameters and the Reset / Kick buttons and
    // CVs, advances the physics, detects zero crossings of each pendulum to
    // latch new random V/Oct voltages and fire gate pulses, updates the
    // position / V/Oct / gate outputs and the spin LEDs, and samples the bob
    // positions into the display trail buffers.
    void process(const ProcessArgs &args) override
    {
        updateParameters();

        bool resetNow = (params[RESET_BUTTON].getValue() > 0.5f) ||
                        (inputs[RESET_CV_INPUT].getVoltage() > 1.0f);
        if (resetNow && !lastResetState)
            resetState();
        lastResetState = resetNow;

        bool kickNow = (params[KICK_BUTTON].getValue() > 0.5f) ||
                       (inputs[KICK_INPUT].getVoltage() > 1.0f);
        if (kickNow && !lastKickState)
            kick();
        lastKickState = kickNow;

        double dt = args.sampleTime * speedFactor;
        dt = std::min(dt, 0.001);
        rk4(dt);

        if (!isValid())
            resetState();

        // Detect zero crossings and generate random voltages + gate triggers
        double currentSign1 = (theta1 >= 0) ? 1.0 : -1.0;
        double currentSign2 = (theta2 >= 0) ? 1.0 : -1.0;

        if (currentSign1 != prevTheta1Sign && std::abs(theta1) > 1e-6)
        {
            generateRandomVoct1();
            double durSec = getGateDurationSeconds(gate1Duration);
            gate1Timer = static_cast<int>(args.sampleRate * durSec);
            flash1 = 1.f;
        }
        if (currentSign2 != prevTheta2Sign && std::abs(theta2) > 1e-6)
        {
            generateRandomVoct2();
            double durSec = getGateDurationSeconds(gate2Duration);
            gate2Timer = static_cast<int>(args.sampleRate * durSec);
            flash2 = 1.f;
        }

        prevTheta1Sign = currentSign1;
        prevTheta2Sign = currentSign2;

        // Decrement gate timers
        if (gate1Timer > 0)
            gate1Timer--;
        if (gate2Timer > 0)
            gate2Timer--;

        // Set outputs
        outputs[VOCT1_OUTPUT].setVoltage((float)voct1Voltage);
        outputs[VOCT2_OUTPUT].setVoltage((float)voct2Voltage);
        outputs[GATE1_OUTPUT].setVoltage(gate1Timer > 0 ? 10.0f : 0.0f);
        outputs[GATE2_OUTPUT].setVoltage(gate2Timer > 0 ? 10.0f : 0.0f);

        double x1 = len1 * sin(theta1);
        double y1 = -len1 * cos(theta1);
        double x2 = x1 + len2 * sin(theta2);
        double y2 = y1 - len2 * cos(theta2);

        outputs[X1_OUTPUT].setVoltage(rack::math::clamp((float)x1, -5.0f, 5.0f));
        outputs[Y1_OUTPUT].setVoltage(rack::math::clamp((float)y1, -5.0f, 5.0f));
        outputs[X2_OUTPUT].setVoltage(rack::math::clamp((float)x2, -5.0f, 5.0f));
        outputs[Y2_OUTPUT].setVoltage(rack::math::clamp((float)y2, -5.0f, 5.0f));

        // LEDs for angular speed
        float bright1 = std::min(1.0f, std::abs((float)omega1) / 10.0f);
        float bright2 = std::min(1.0f, std::abs((float)omega2) / 10.0f);
        lights[MASS1_LIGHT].setSmoothBrightness(bright1, args.sampleTime);
        lights[MASS2_LIGHT].setSmoothBrightness(bright2, args.sampleTime);

        // ---- display data ----
        flash1 = std::max(0.f, flash1 - 6.f * args.sampleTime);
        flash2 = std::max(0.f, flash2 - 6.f * args.sampleTime);

        const float trailStep = 1.f / PL_TRAIL_HZ;
        trailTimer += args.sampleTime;
        if (trailTimer >= trailStep)
        {
            trailTimer -= trailStep;
            if (trailTimer > trailStep)
                trailTimer = 0.f;
            float reach = (float)std::max(len1 + len2, 0.1);
            trailX1[trailHead] = (float)x1 / reach;
            trailY1[trailHead] = (float)y1 / reach;
            trailX2[trailHead] = (float)x2 / reach;
            trailY2[trailHead] = (float)y2 / reach;
            trailHead = (trailHead + 1) % PL_TRAIL_N;
            if (trailCount < PL_TRAIL_N)
                trailCount++;
        }
    }

    // JSON serialisation
    // Saves the four per-pendulum user settings: V/Oct range and gate
    // duration for each pendulum.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "voct1Range", json_integer(voct1Range));
        json_object_set_new(root, "voct2Range", json_integer(voct2Range));
        json_object_set_new(root, "gate1Duration", json_integer(gate1Duration));
        json_object_set_new(root, "gate2Duration", json_integer(gate2Duration));
        return root;
    }

    // Restores the four per-pendulum settings from a saved patch, clamping
    // each to its valid enum range.
    void dataFromJson(json_t *root) override
    {
        json_t *j;
        j = json_object_get(root, "voct1Range");
        if (j)
            voct1Range = (VoctRange)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_RANGES - 1);
        j = json_object_get(root, "voct2Range");
        if (j)
            voct2Range = (VoctRange)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_RANGES - 1);
        j = json_object_get(root, "gate1Duration");
        if (j)
            gate1Duration = (GateDuration)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_GATE_DURS - 1);
        j = json_object_get(root, "gate2Duration");
        if (j)
            gate2Duration = (GateDuration)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_GATE_DURS - 1);
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{
    namespace ag = agmm;

    // Returns pendulum 1's accent colour (cyan).
    NVGcolor colP1() { return nvgRGB(0x3d, 0xe0, 0xff); } // pendulum 1: cyan
    // Returns pendulum 2's accent colour (violet).
    NVGcolor colP2() { return nvgRGB(0xb0, 0x5a, 0xff); } // pendulum 2: violet

    // Metal rod: dark outline + flat steel core
    // Draws one pendulum rod as a dark outline underneath a flat steel core
    // line of width `w`.
    void drawRod(NVGcontext *vg, float x0, float y0, float x1, float y1, float w, NVGcolor steel)
    {
        nvgLineCap(vg, NVG_ROUND);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x0, y0);
        nvgLineTo(vg, x1, y1);
        nvgStrokeColor(vg, nvgRGB(0x14, 0x17, 0x1d));
        nvgStrokeWidth(vg, w + 1.6f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgMoveTo(vg, x0, y0);
        nvgLineTo(vg, x1, y1);
        nvgStrokeColor(vg, steel);
        nvgStrokeWidth(vg, w);
        nvgStroke(vg);
    }

    // Flat bob: filled disc with a darker rim
    // Draws one pendulum bob as a filled disc with a contrasting rim.
    void drawBob(NVGcontext *vg, float cx, float cy, float r, NVGcolor fill, NVGcolor edge)
    {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillColor(vg, fill);
        nvgFill(vg);
        nvgStrokeColor(vg, edge);
        nvgStrokeWidth(vg, 1.3f);
        nvgStroke(vg);
    }

    // Flat pivot bearing
    // Draws the central pivot bearing: a dark disc with a small metal
    // centre cap.
    void drawHub(NVGcontext *vg, float cx, float cy)
    {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, 10.f);
        nvgFillColor(vg, nvgRGB(0x24, 0x29, 0x31));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 60));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, 4.5f);
        nvgFillColor(vg, nvgRGB(0xc9, 0xd1, 0xdc));
        nvgFill(vg);
    }

    // Fading motion trail (older = thinner and fainter)
    // Draws a polyline through n points, broken into short segments so the
    // stroke width and alpha taper off toward the oldest points.
    void drawTrail(NVGcontext *vg, const float *xs, const float *ys, int n, NVGcolor col, float width, float alpha)
    {
        if (n < 2)
            return;
        nvgLineCap(vg, NVG_ROUND);
        nvgLineJoin(vg, NVG_ROUND);
        const int K = 4;
        for (int s = 0; s < n - 1; s += K)
        {
            int e = std::min(s + K, n - 1);
            float t = (float)(s + 1) / (float)n;
            nvgBeginPath(vg);
            nvgMoveTo(vg, xs[s], ys[s]);
            for (int i = s + 1; i <= e; ++i)
                nvgLineTo(vg, xs[i], ys[i]);
            nvgStrokeColor(vg, nvgRGBAf(col.r, col.g, col.b, alpha * t * t));
            nvgStrokeWidth(vg, width * (0.3f + 0.7f * t));
            nvgStroke(vg);
        }
    }

    // ------------------------------------------------------------
    // Panel background: title, display frame, labelled plates
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the two PENDULUM block
    // captions and their SPIN LED labels, the per-pendulum MASS/LENGTH knob
    // cells and X/Y/VOCT/GATE output cells (outputs tinted), and the global
    // CONTROL row.
    struct PLPanelBackground : TransparentWidget
    {
        PLPanelBackground() { box.size = Vec(PL_PANEL_W, PL_PANEL_H); }

        // Block caption with hairlines either side of the text
        // Draws a centred block caption with a hairline on each side, the
        // hairlines stopping a measured distance from the text.
        static void blockCaption(NVGcontext *vg, float x0, float w, float y, const char *txt, NVGcolor col)
        {
            float cx = x0 + w / 2.f;
            float gap = ag::textWidth(vg, txt, 8.f, 2.f) / 2.f + 6.f;
            ag::text(vg, cx, y, txt, 8.f, col, ag::ALIGN_C, 2.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x0, y);
            nvgLineTo(vg, std::max(x0, cx - gap), y);
            nvgMoveTo(vg, std::min(x0 + w, cx + gap), y);
            nvgLineTo(vg, x0 + w, y);
            nvgStrokeColor(vg, ag::withA(col, 0.35f));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Draws all static panel art: base panel, bezel, both PENDULUM
        // captions, the SPIN labels, the MASS/LENGTH knob cells, the X/Y/
        // V/OCT/GATE output cells (tinted), and the global CONTROL row.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, PL_PANEL_W, PL_PANEL_H, "PSYLFO");
            ag::bezel(vg, PL_DISP_X, PL_DISP_Y, PL_DISP_W, PL_DISP_H);

            // Side blocks
            blockCaption(vg, PL_LBLOCK_X, PL_BLOCK_W, PL_CAP_Y, "PENDULUM 1", colP1());
            blockCaption(vg, PL_RBLOCK_X, PL_BLOCK_W, PL_CAP_Y, "PENDULUM 2", colP2());

            // LED rows (the LED itself is a widget): label left of it
            ag::text(vg, PL_LBLOCK_X + 36.f, PL_LED_Y, "SPIN", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            ag::text(vg, PL_RBLOCK_X + 36.f, PL_LED_Y, "SPIN", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);

            for (int c = 0; c < 2; ++c)
            {
                const char *kn = c == 0 ? "MASS" : "LENGTH";
                ag::cell(vg, PL_LCOL[c], PL_KNOB_TOP, PL_CELL_W, PL_KNOB_H, kn);
                ag::cell(vg, PL_RCOL[c], PL_KNOB_TOP, PL_CELL_W, PL_KNOB_H, kn);
            }
            const char *outs[2][2] = {{"X", "Y"}, {"V/OCT", "GATE"}};
            for (int r = 0; r < 2; ++r)
                for (int c = 0; c < 2; ++c)
                {
                    ag::cell(vg, PL_LCOL[c], PL_OUT_TOP[r], PL_CELL_W, PL_OUT_H, outs[r][c], true);
                    ag::cell(vg, PL_RCOL[c], PL_OUT_TOP[r], PL_CELL_W, PL_OUT_H, outs[r][c], true);
                }

            // Global controls
            ag::sectionHeader(vg, PL_PANEL_W, PL_CTRL_CAP_Y, "CONTROL");
            const char *names[8] = {"SPEED", "SPD CV", "GRAVITY", "DAMPING", "KICK", "KICK IN", "RESET", "RESET IN"};
            for (int i = 0; i < 8; ++i)
                ag::cell(vg, plCtrlX(i), PL_CTRL_TOP, PL_CTRL_W, PL_CTRL_H, names[i]);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Pendulum display
    // ------------------------------------------------------------
    // Live 200x200 circular display that draws the double pendulum in real
    // time: dial face, degree ring, plumb line, angle arcs, fading trails,
    // rods, mass-sized bobs, gate flash rings and corner readouts.
    struct PLDisplayWidget : TransparentWidget
    {
        PsyLFO *module = nullptr;

        PLDisplayWidget(PsyLFO *m) : module(m)
        {
            box.pos = Vec(PL_DISP_X, PL_DISP_Y);
            box.size = Vec(PL_DISP_W, PL_DISP_H);
        }

        // Static background: screen, dial face, degree ring, reference lines
        // Draws the display screen, the circular dial face, the two
        // concentric range rings, the cross hair, the degree tick ring (with
        // longer ticks every 30°), the 0° mark and the dashed plumb line.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float cx = PL_DISP_W / 2.f, cy = PL_DISP_H / 2.f;

            ag::screen(vg, 0.f, 0.f, PL_DISP_W, PL_DISP_H);

            // Dial face
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, 97.f);
            nvgFillColor(vg, nvgRGB(0x07, 0x0f, 0x16));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 46));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Concentric range rings
            for (int k = 1; k <= 2; ++k)
            {
                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, 33.f * k);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, k == 2 ? 26 : 14));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // Cross hair
            nvgBeginPath(vg);
            nvgMoveTo(vg, cx + 0.5f, cy - 90.f);
            nvgLineTo(vg, cx + 0.5f, cy + 90.f);
            nvgMoveTo(vg, cx - 90.f, cy + 0.5f);
            nvgLineTo(vg, cx + 90.f, cy + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Degree ring: ticks every 10 deg, long every 30 deg. 0 deg = straight down.
            nvgBeginPath(vg);
            for (int i = 0; i < 36; ++i)
            {
                if (i % 3 == 0)
                    continue;
                float a = i * 10.f * (float)M_PI / 180.f;
                float dx = std::sin(a), dy = std::cos(a);
                nvgMoveTo(vg, cx + dx * 94.f, cy + dy * 94.f);
                nvgLineTo(vg, cx + dx * 90.f, cy + dy * 90.f);
            }
            nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 60));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            for (int i = 3; i < 36; i += 3)
            {
                float a = i * 10.f * (float)M_PI / 180.f;
                float dx = std::sin(a), dy = std::cos(a);
                nvgMoveTo(vg, cx + dx * 94.f, cy + dy * 94.f);
                nvgLineTo(vg, cx + dx * 86.f, cy + dy * 86.f);
            }
            nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 120));
            nvgStrokeWidth(vg, 1.2f);
            nvgStroke(vg);

            // Zero mark
            nvgBeginPath(vg);
            nvgMoveTo(vg, cx, cy + 94.f);
            nvgLineTo(vg, cx, cy + 83.f);
            nvgStrokeColor(vg, ag::accent());
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);

            // Dashed plumb line (the vertical reference)
            nvgBeginPath(vg);
            for (float y = 14.f; y < 80.f; y += 6.f)
            {
                nvgMoveTo(vg, cx + 0.5f, cy + y);
                nvgLineTo(vg, cx + 0.5f, cy + y + 3.f);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the live double pendulum — angle arcs,
        // fading trails for both bobs, the two rods, the gate flash rings,
        // the mass-sized bobs, the pivot, and the corner readouts (angles,
        // angular velocities, reach, gravity). Uses a sample pose when there
        // is no module (module browser).
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float cx = PL_DISP_W / 2.f, cy = PL_DISP_H / 2.f;
            const float Rpx = 66.f; // pixel radius of the full reach

            // ---- state (live, or a sample pose in the module browser) ----
            float th1, th2, w1, w2, l1, l2, m1, m2, fl1 = 0.f, fl2 = 0.f, grav;
            float t1x[PL_TRAIL_N], t1y[PL_TRAIL_N], t2x[PL_TRAIL_N], t2y[PL_TRAIL_N];
            int tn = 0;

            if (module)
            {
                th1 = (float)module->theta1;
                th2 = (float)module->theta2;
                w1 = (float)module->omega1;
                w2 = (float)module->omega2;
                l1 = (float)module->len1;
                l2 = (float)module->len2;
                m1 = (float)module->mass1;
                m2 = (float)module->mass2;
                fl1 = module->flash1;
                fl2 = module->flash2;
                grav = (float)module->g;
            }
            else
            {
                th1 = 0.7f;
                th2 = 2.2f;
                w1 = 3.f;
                w2 = -5.f;
                l1 = 1.2f;
                l2 = 1.f;
                m1 = 2.f;
                m2 = 1.f;
                grav = 9.81f;
            }

            float reach = std::max(l1 + l2, 0.1f);
            float scale = Rpx / reach;

            if (module)
            {
                int cnt = module->trailCount;
                int head = module->trailHead;
                tn = std::min(std::max(cnt, 0), PL_TRAIL_N);
                for (int i = 0; i < tn; ++i)
                {
                    int idx = ((head - tn + i) % PL_TRAIL_N + PL_TRAIL_N) % PL_TRAIL_N;
                    t1x[i] = cx + module->trailX1[idx] * Rpx;
                    t1y[i] = cy - module->trailY1[idx] * Rpx;
                    t2x[i] = cx + module->trailX2[idx] * Rpx;
                    t2y[i] = cy - module->trailY2[idx] * Rpx;
                }
            }
            else
            {
                tn = PL_TRAIL_N;
                for (int i = 0; i < tn; ++i)
                {
                    float k = (float)(i - (tn - 1)) * 0.03f;
                    float a1 = th1 + k * 0.9f;
                    float a2 = th2 + k * 1.7f + 0.5f * std::sin(k * 3.f);
                    t1x[i] = cx + l1 * std::sin(a1) * scale;
                    t1y[i] = cy + l1 * std::cos(a1) * scale;
                    t2x[i] = t1x[i] + l2 * std::sin(a2) * scale;
                    t2y[i] = t1y[i] + l2 * std::cos(a2) * scale;
                }
            }

            // Positions: theta = 0 hangs straight down, positive angles swing right
            float b1x = cx + l1 * std::sin(th1) * scale;
            float b1y = cy + l1 * std::cos(th1) * scale;
            float b2x = b1x + l2 * std::sin(th2) * scale;
            float b2y = b1y + l2 * std::cos(th2) * scale;

            float r1 = 4.5f + 3.4f * std::cbrt(m1);
            float r2 = 4.5f + 3.4f * std::cbrt(m2);

            // ---- angle arcs (measured from the vertical) ----
            auto arcAt = [&](float ax, float ay, float r, float th, NVGcolor col)
            {
                if (std::fabs(th) < 0.02f)
                    return;
                float a0 = (float)M_PI * 0.5f;
                float a1 = a0 - th;
                nvgBeginPath(vg);
                nvgArc(vg, ax, ay, r, a0, a1, th < 0.f ? NVG_CW : NVG_CCW);
                nvgStrokeColor(vg, col);
                nvgStrokeWidth(vg, 1.6f);
                nvgLineCap(vg, NVG_ROUND);
                nvgStroke(vg);
            };
            arcAt(cx, cy, 24.f, th1, nvgRGBA(0x3d, 0xe0, 0xff, 150));
            arcAt(b1x, b1y, 17.f, th2, nvgRGBA(0xb0, 0x5a, 0xff, 150));

            // Short vertical reference through bob 1
            nvgBeginPath(vg);
            nvgMoveTo(vg, b1x + 0.5f, b1y + r1 + 2.f);
            nvgLineTo(vg, b1x + 0.5f, b1y + r1 + 10.f);
            nvgStrokeColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 70));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // ---- trails ----
            drawTrail(vg, t1x, t1y, tn, colP1(), 1.4f, 0.55f);
            drawTrail(vg, t2x, t2y, tn, nvgRGB(0xd8, 0xa8, 0xff), 1.8f, 0.95f);

            // ---- rods ----
            drawRod(vg, cx, cy, b1x, b1y, 4.0f, nvgRGB(0x74, 0x9a, 0xab));
            drawRod(vg, b1x, b1y, b2x, b2y, 3.4f, nvgRGB(0x93, 0x84, 0xb0));

            // ---- gate flashes (expanding ring) ----
            auto flashRing = [&](float bx, float by, float r, NVGcolor col, float flash)
            {
                if (flash <= 0.02f)
                    return;
                nvgBeginPath(vg);
                nvgCircle(vg, bx, by, r + (1.f - flash) * 16.f);
                nvgStrokeColor(vg, ag::withA(col, 0.8f * flash));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            };
            flashRing(b1x, b1y, r1, colP1(), fl1);
            flashRing(b2x, b2y, r2, colP2(), fl2);

            // ---- bobs ----
            drawBob(vg, b1x, b1y, r1, nvgRGB(0x1f, 0x9f, 0xc0), nvgRGB(0xd8, 0xf9, 0xff));
            drawBob(vg, b2x, b2y, r2, nvgRGB(0x7a, 0x3a, 0xc8), nvgRGB(0xf1, 0xe3, 0xff));

            // ---- pivot ----
            drawHub(vg, cx, cy);

            // ---- readouts (in the corners, outside the dial) ----
            char buf[32];
            float deg1 = th1 * 180.f / (float)M_PI;
            float deg2 = th2 * 180.f / (float)M_PI;

            snprintf(buf, sizeof(buf), "A1 %+4.0f", deg1);
            ag::text(vg, 4.f, 8.f, buf, 7.f, colP1(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "W1 %+.1f", w1);
            ag::text(vg, 4.f, 18.f, buf, 7.f, colP1(), ag::ALIGN_L, 0.5f);

            snprintf(buf, sizeof(buf), "A2 %+4.0f", deg2);
            ag::text(vg, PL_DISP_W - 4.f, 8.f, buf, 7.f, colP2(), ag::ALIGN_R, 0.5f);
            snprintf(buf, sizeof(buf), "W2 %+.1f", w2);
            ag::text(vg, PL_DISP_W - 4.f, 18.f, buf, 7.f, colP2(), ag::ALIGN_R, 0.5f);

            snprintf(buf, sizeof(buf), "R %.2fM", reach);
            ag::text(vg, 4.f, PL_DISP_H - 8.f, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "G %.1f", grav);
            ag::text(vg, PL_DISP_W - 4.f, PL_DISP_H - 8.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget: adds the static panel, screws, the live pendulum display,
// the two MASS/LENGTH knobs and their SPIN LEDs, the 8 output jacks, and the
// global control row (speed, gravity, damping, kick, reset and their CVs).
// ------------------------------------------------------------
struct PsyLFOWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, display, the two
    // pendulums' LEDs, MASS/LENGTH knobs and X/Y/VOCT/GATE outputs, and the
    // global CONTROL row.
    PsyLFOWidget(PsyLFO *module)
    {
        setModule(module);
        box.size = Vec(PL_PANEL_W, PL_PANEL_H);

        addChild(new PLPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PLDisplayWidget(module));

        // Pendulum 1 (left)
        addChild(createLightCentered<MediumLight<agmm::CyanLight>>(Vec(PL_LBLOCK_X + 58.f, PL_LED_Y), module, PsyLFO::MASS1_LIGHT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_LCOL[0], plKnobY()), module, PsyLFO::MASS1_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_LCOL[1], plKnobY()), module, PsyLFO::LEN1_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_LCOL[0], plOutY(0)), module, PsyLFO::X1_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_LCOL[1], plOutY(0)), module, PsyLFO::Y1_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_LCOL[0], plOutY(1)), module, PsyLFO::VOCT1_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_LCOL[1], plOutY(1)), module, PsyLFO::GATE1_OUTPUT));

        // Pendulum 2 (right)
        addChild(createLightCentered<MediumLight<agmm::VioletLight>>(Vec(PL_RBLOCK_X + 58.f, PL_LED_Y), module, PsyLFO::MASS2_LIGHT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_RCOL[0], plKnobY()), module, PsyLFO::MASS2_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_RCOL[1], plKnobY()), module, PsyLFO::LEN2_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_RCOL[0], plOutY(0)), module, PsyLFO::X2_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_RCOL[1], plOutY(0)), module, PsyLFO::Y2_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_RCOL[0], plOutY(1)), module, PsyLFO::VOCT2_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_RCOL[1], plOutY(1)), module, PsyLFO::GATE2_OUTPUT));

        // Global controls: SPEED, SPD CV, GRAVITY, DAMPING, KICK, KICK IN, RESET, RESET IN
        const float y = plCtrlY();
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(plCtrlX(0), y), module, PsyLFO::SPEED_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(plCtrlX(1), y), module, PsyLFO::SPEED_CV_INPUT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(plCtrlX(2), y), module, PsyLFO::GRAVITY_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(plCtrlX(3), y), module, PsyLFO::DAMPING_PARAM));
        addParam(createParamCentered<VCVButton>(Vec(plCtrlX(4), y), module, PsyLFO::KICK_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(plCtrlX(5), y), module, PsyLFO::KICK_INPUT));
        addParam(createParamCentered<VCVButton>(Vec(plCtrlX(6), y), module, PsyLFO::RESET_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(plCtrlX(7), y), module, PsyLFO::RESET_CV_INPUT));
        agmm::addKnobArcs(this, 2.f);
    }

    // Adds the right-click menu: "Voct1 range" and "Voct2 range" submenus
    // (each with a checkmark on the active range, and a re-randomise on
    // selection), and "Gate1 duration" / "Gate2 duration" submenus (each with
    // a checkmark on the active duration).
    void appendContextMenu(ui::Menu *menu) override
    {
        PsyLFO *module = dynamic_cast<PsyLFO *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        // Voct1 range submenu
        menu->addChild(createSubmenuItem("Voct1 range", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                                             for (int i = 0; i < PsyLFO::NUM_RANGES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(rangeNames[i],
                                                                                  CHECKMARK(module->voct1Range == i),
                                                                                  [=]()
                                                                                  {
                                                                                      module->voct1Range = (PsyLFO::VoctRange)i;
                                                                                      module->generateRandomVoct1();
                                                                                  }));
                                             }
                                         }));

        // Voct2 range submenu
        menu->addChild(createSubmenuItem("Voct2 range", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                                             for (int i = 0; i < PsyLFO::NUM_RANGES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(rangeNames[i],
                                                                                  CHECKMARK(module->voct2Range == i),
                                                                                  [=]()
                                                                                  {
                                                                                      module->voct2Range = (PsyLFO::VoctRange)i;
                                                                                      module->generateRandomVoct2();
                                                                                  }));
                                             }
                                         }));

        // Gate1 duration submenu
        menu->addChild(createSubmenuItem("Gate1 duration", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                                             for (int i = 0; i < PsyLFO::NUM_GATE_DURS; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(gateNames[i],
                                                                                  CHECKMARK(module->gate1Duration == i),
                                                                                  [=]()
                                                                                  {
                                                                                      module->gate1Duration = (PsyLFO::GateDuration)i;
                                                                                  }));
                                             }
                                         }));

        // Gate2 duration submenu
        menu->addChild(createSubmenuItem("Gate2 duration", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                                             for (int i = 0; i < PsyLFO::NUM_GATE_DURS; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(gateNames[i],
                                                                                  CHECKMARK(module->gate2Duration == i),
                                                                                  [=]()
                                                                                  {
                                                                                      module->gate2Duration = (PsyLFO::GateDuration)i;
                                                                                  }));
                                             }
                                         }));
    }
};

Model *modelPsyLFO = createModel<PsyLFO, PsyLFOWidget>("PsyLFO");