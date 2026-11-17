#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <random>
#include <limits>
#include <memory>
#include <atomic>

// ============================================================================
// SpringPendulum
// ============================================================================
//
// OVERVIEW
// --------
// SpringPendulum is a 24 HP chaotic modulation source based on a physical
// spring-pendulum: a mass suspended by four linear springs anchored at the
// four compass points of a circular dial, with optional gravity, damping and
// a smooth random force. The mass is integrated in real time with RK4 at an
// adjustable simulation speed, and can be grabbed and dragged with the mouse.
//
// The module exposes the mass position as X/Y (in volts, ±5 V per anchor
// distance), plus a random V/Oct output that is re-rolled every time the mass
// crosses the vertical centre line (X = 0), together with a matching gate
// pulse. The Voct range and gate duration are configurable via the context
// menu.
//
// Controls:
//   SPEED       Simulation speed multiplier (0.2x .. 10x)
//   MASS        Mass of the bob (affects inertia and display radius)
//   REST LEN    Spring rest length (0.1 .. 0.9)
//   SPRING K    Spring constant (1 .. 100 N/m)
//   PUSH STR    Strength of the random push (m/s)
//   CHAOS       Amplitude of the random force (m/s²)
//   DAMP        Damping coefficient (1/s)
//   GRAVITY     Downward acceleration (m/s²)
//   PUSH/RESET  Buttons and CV inputs for random push and reset
//   ENERGY      LED showing current speed
//
// Display: a circular dial with the four anchor posts, the four coil springs,
// a fading motion trail, the bob and a header/footer row showing live
// position, velocity, spring constant, gravity, and a "GRAB" indicator.
//
// File structure:
//   1. Layout and physics constants (24 HP = 360×380 px).
//   2. SpringPendulum module: state, parameters, RK4 physics, output
//      generation, JSON persistence.
//   3. Anonymous-namespace drawing helpers: colSpring, mapToDial,
//      unmapRadius, drawSpring, drawBob, drawAnchor, drawTrail.
//   4. Anonymous-namespace UI: SPPanelBackground (static art) and
//      SPDisplayWidget (live dial + mouse grabbing).
//   5. SpringPendulumWidget: places all knobs, buttons, jacks and lights,
//      and builds the Voct-range and gate-duration menus.
//   6. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int SP_TRAIL_N = 120;      // trail points
static const float SP_TRAIL_HZ = 150.f; // trail sample rate

static const float SP_PANEL_W = RACK_GRID_WIDTH * 24; // 360
static const float SP_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display: header row (readouts) | dial | footer row (readouts). Bezel adds 4 px around it.
static const float SP_DISP_W = 180.f;
static const float SP_DISP_H = 188.f;
static const float SP_DISP_X = (SP_PANEL_W - SP_DISP_W) / 2.f; // 90
static const float SP_DISP_Y = 30.f;
static const float SP_HEAD_H = 16.f; // header row height
static const float SP_FOOT_H = 16.f; // footer row height

// Dial geometry inside the display (local coordinates)
static const float SP_DIAL_CX = SP_DISP_W / 2.f;                                       // 90
static const float SP_DIAL_CY = SP_HEAD_H + (SP_DISP_H - SP_HEAD_H - SP_FOOT_H) / 2.f; // 94
static const float SP_DIAL_R = 66.f;
// Max distance of the mass centre from the dial centre (bob radius <= ~9.3, so it never touches the rim)
static const float SP_MAXPX = 55.f;
// Soft-limit knee (fraction of SP_MAXPX where compression starts)
static const float SP_KNEE = 0.85f;

// Physical geometry: the four springs are anchored at distance SP_ANCHOR_D from the centre
static const double SP_ANCHOR_D = 1.0;
static const float SP_REACH = 1.1f; // display range, in units of SP_ANCHOR_D

// Side knob cells (50 x 52, label at top + 10, control at top + 32)
static const float SP_SIDE_W = 50.f;
static const float SP_SIDE_H = 52.f;
static const float SP_SIDE_LX = 34.f;
static const float SP_SIDE_RX = 326.f;
static const float SP_SIDE_TOP1 = 52.f;
static const float SP_SIDE_TOP2 = 124.f;
static const float SP_CTL_DY = 32.f;

// Control row (seven 44 x 50 cells)
static const float SP_CTRL_SECTION_Y = 234.f;
static const float SP_CTRL_TOP = 242.f;
static const float SP_CTRL_CELL_W = 44.f;
static const float SP_CTRL_CELL_H = 50.f;
static const float SP_CTRL_X0 = 34.f;
static const float SP_CTRL_DX = 49.f;

// I/O row (six 48 x 46 cells; jack at top + 30)
static const float SP_IO_SECTION_Y = 302.f;
static const float SP_IO_TOP = 310.f;
static const float SP_IO_CELL_W = 48.f;
static const float SP_IO_CELL_H = 46.f;
static const float SP_IO_X0 = 40.f;
static const float SP_IO_DX = 56.f;
static const float SP_IO_JACK_DY = 30.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// Spring-pendulum modulation source: integrates a mass on four springs with
// optional gravity, damping and a random force, exposing X/Y, random V/Oct
// and gate outputs.
struct SpringPendulum : Module
{
    enum ParamId
    {
        SPEED_PARAM,
        MASS_PARAM,
        LENGTH_PARAM,
        SPRING_K_PARAM,
        RESET_BUTTON,
        PUSH_STRENGTH_PARAM,
        PUSH_BUTTON,
        CHAOS_PARAM,
        DAMP_PARAM,
        GRAVITY_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        RESET_CV_INPUT,
        PUSH_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        X_OUTPUT,
        Y_OUTPUT,
        VOCT_OUTPUT,
        GATE_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        MASS_LIGHT,
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
    // Gate duration selection
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

    // A full physics state (position + velocity).
    struct State
    {
        double x, y, vx, vy;
    };

    // State variables (origin = centre of the dial, +y = up)
    double x = 0.0, y = 0.0;
    double vx = 0.0, vy = 0.0;
    double mass = 1.0;
    double restLen = 0.5;
    double springK = 10.0;
    double speedFactor = 1.0;
    double damping = 0.3;
    double chaos = 0.35;
    double g = 0.0;

    // Random force (Ornstein-Uhlenbeck noise, acceleration units)
    double noiseAx = 0.0, noiseAy = 0.0;

    // Mouse grab (written by the UI thread, read by the audio thread)
    std::atomic<bool> grabbed{false};
    std::atomic<double> grabX{0.0};
    std::atomic<double> grabY{0.0};
    // Audio-thread copies used by the physics
    bool handOn = false;
    double handX = 0.0, handY = 0.0;
    double handWn = 25.0;

    // Voct & gate state
    double voctVoltage = 0.0;
    double prevXSign = 1.0;
    int gateTimer = 0;

    // User settings
    VoctRange voctRange = RANGE_N1_1;
    GateDuration gateDuration = GATE_5MS;

    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger pushTrigger;
    bool lastResetState = false;
    std::mt19937 rng;
    std::normal_distribution<double> gauss{0.0, 1.0};

    // Display data
    float trailX[SP_TRAIL_N] = {};
    float trailY[SP_TRAIL_N] = {};
    int trailHead = 0;
    int trailCount = 0;
    float trailTimer = 0.f;

    // Constructor: configures all parameters (with units/ranges), the two
    // CV inputs and the four outputs, seeds the RNG, randomises the initial
    // state, and generates the initial random V/Oct value.
    SpringPendulum()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(SPEED_PARAM, 0.f, 1.f, 0.5f, "Speed", " Hz", 0.f, 10.f, 0.1f);
        configParam(MASS_PARAM, 0.1f, 10.f, 1.f, "Mass", " kg");
        configParam(LENGTH_PARAM, 0.1f, 0.9f, 0.5f, "Spring rest length", " m");
        configParam(SPRING_K_PARAM, 1.f, 100.f, 10.f, "Spring constant", " N/m");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset");
        configParam(PUSH_STRENGTH_PARAM, 0.f, 10.f, 2.f, "Push strength", " m/s");
        configParam(PUSH_BUTTON, 0.f, 1.f, 0.f, "Random push");
        configParam(CHAOS_PARAM, 0.f, 1.f, 0.35f, "Chaos (random force)", "%", 0.f, 100.f);
        configParam(DAMP_PARAM, 0.02f, 2.f, 0.3f, "Damping", " 1/s");
        configParam(GRAVITY_PARAM, 0.f, 40.f, 0.f, "Gravity", " m/s²");

        configInput(RESET_CV_INPUT, "Reset gate");
        configInput(PUSH_CV_INPUT, "Random push gate");

        configOutput(X_OUTPUT, "Mass X");
        configOutput(Y_OUTPUT, "Mass Y");
        configOutput(VOCT_OUTPUT, "V/Oct (random on X = 0 crossing)");
        configOutput(GATE_OUTPUT, "Gate (trigger on Voct change)");

        std::random_device rd;
        rng.seed(rd());
        resetState();
        generateRandomVoct();
    }

    // Rack reset hook: re-randomises the mass state and clears noise/gate/
    // trail data.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        resetState();
    }

    // Places the mass at a random polar position inside the springs (radius
    // 0.15..0.45 of the anchor distance), zeroes velocities and noise,
    // clears the gate timer and trail, and records the starting sign of x.
    void resetState()
    {
        std::uniform_real_distribution<double> angleDist(-M_PI, M_PI);
        std::uniform_real_distribution<double> radDist(0.15, 0.45);
        double angle = angleDist(rng);
        double r = SP_ANCHOR_D * radDist(rng);
        x = r * sin(angle);
        y = r * cos(angle);
        vx = 0.0;
        vy = 0.0;
        noiseAx = 0.0;
        noiseAy = 0.0;

        prevXSign = (x >= 0) ? 1.0 : -1.0;
        gateTimer = 0;

        trailHead = 0;
        trailCount = 0;
        trailTimer = 0.f;
    }

    // Scales a raw random value in [-1, 1] into the voltage range selected
    // by the VoctRange enum.
    double scaleVoct(double raw, VoctRange range)
    {
        switch (range)
        {
        case RANGE_0_1:
            return (raw + 1.0) / 2.0;
        case RANGE_N1_1:
            return raw;
        case RANGE_N2_2:
            return raw * 2.0;
        case RANGE_N5_5:
            return raw * 5.0;
        case RANGE_0_5:
            return (raw + 1.0) / 2.0 * 5.0;
        default:
            return raw;
        }
    }

    // Generates a new random V/Oct voltage inside the currently selected
    // range. Called on reset, from the menu when the range changes, and on
    // every X zero crossing.
    void generateRandomVoct()
    {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        voctVoltage = scaleVoct(raw, voctRange);
    }

    // Returns the length in seconds of the given gate duration setting.
    double getGateDurationSeconds(GateDuration dur)
    {
        switch (dur)
        {
        case GATE_TRIGGER:
            return 0.0005;
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

    // Reads the seven physical parameters and caches them in member fields
    // for the physics step.
    void updateParameters()
    {
        float speedKnob = params[SPEED_PARAM].getValue();
        speedFactor = 0.2f + speedKnob * 9.8f;
        mass = params[MASS_PARAM].getValue();
        restLen = params[LENGTH_PARAM].getValue();
        springK = params[SPRING_K_PARAM].getValue();
        chaos = params[CHAOS_PARAM].getValue();
        damping = params[DAMP_PARAM].getValue();
        g = params[GRAVITY_PARAM].getValue();
    }

    // Four springs: anchors at N (0,D), E (D,0), S (0,-D), W (-D,0)
    // Computes the time derivative of the state `s` into `d`: each of the
    // four springs contributes a linear restoring force toward its anchor,
    // then gravity, damping and the random force are added, plus an optional
    // critically damped spring pulling toward the mouse when grabbed.
    void deriv(const State &s, State &d) const
    {
        const double D = SP_ANCHOR_D;
        const double axs[4] = {0.0, D, 0.0, -D};
        const double ays[4] = {D, 0.0, -D, 0.0};

        double fx = 0.0, fy = 0.0;
        for (int i = 0; i < 4; ++i)
        {
            double dx = axs[i] - s.x;
            double dy = ays[i] - s.y;
            double r = std::sqrt(dx * dx + dy * dy);
            if (r < 1e-6)
                continue;
            double f = springK * (r - restLen) / r;
            fx += f * dx;
            fy += f * dy;
        }
        d.x = s.vx;
        d.y = s.vy;
        d.vx = fx / mass - damping * s.vx + noiseAx;
        d.vy = fy / mass - damping * s.vy - g + noiseAy;

        // Mouse "hand": critically damped pull towards the cursor
        if (handOn)
        {
            d.vx += handWn * handWn * (handX - s.x) - 2.0 * handWn * s.vx;
            d.vy += handWn * handWn * (handY - s.y) - 2.0 * handWn * s.vy;
        }
    }

    // Advances the full physics state by `dt` using the classical RK4
    // integrator with the derivative function `deriv`.
    void rk4(double dt)
    {
        State s{x, y, vx, vy};
        State k1, k2, k3, k4, t;

        // Computes an intermediate state t from the base state plus k * h.
        auto advance = [&](const State &k, double h)
        {
            t.x = s.x + h * k.x;
            t.y = s.y + h * k.y;
            t.vx = s.vx + h * k.vx;
            t.vy = s.vy + h * k.vy;
        };

        deriv(s, k1);
        advance(k1, 0.5 * dt);
        deriv(t, k2);
        advance(k2, 0.5 * dt);
        deriv(t, k3);
        advance(k3, dt);
        deriv(t, k4);

        x += dt * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x) / 6.0;
        y += dt * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y) / 6.0;
        vx += dt * (k1.vx + 2.0 * k2.vx + 2.0 * k3.vx + k4.vx) / 6.0;
        vy += dt * (k1.vy + 2.0 * k2.vy + 2.0 * k3.vy + k4.vy) / 6.0;
    }

    // Main audio callback: reads parameters and the reset/push buttons and
    // CVs, updates the mouse grab target, advances the Ornstein-Uhlenbeck
    // random force, integrates the physics with RK4, enforces the soft wall
    // and NaN guards, detects X zero crossings to re-roll V/Oct and fire the
    // gate, writes the outputs and updates the LED and trail buffer.
    void process(const ProcessArgs &args) override
    {
        updateParameters();

        bool resetNow = (params[RESET_BUTTON].getValue() > 0.5f) ||
                        (inputs[RESET_CV_INPUT].getVoltage() > 1.0f);
        if (resetNow && !lastResetState)
            resetState();
        lastResetState = resetNow;

        bool pushNow = (params[PUSH_BUTTON].getValue() > 0.5f) ||
                       (inputs[PUSH_CV_INPUT].getVoltage() > 1.0f);
        if (pushTrigger.process(pushNow))
        {
            float strength = params[PUSH_STRENGTH_PARAM].getValue();
            std::uniform_real_distribution<double> angleDist(0.0, 2.0 * M_PI);
            double angle = angleDist(rng);
            vx += strength * cos(angle);
            vy += strength * sin(angle);
        }

        double dt = args.sampleTime * speedFactor;
        dt = std::min(dt, 0.001);

        // Mouse grab: copy the target, and scale the hand stiffness with the
        // simulation speed so the ball follows the cursor in real time
        handOn = grabbed.load();
        handX = grabX.load();
        handY = grabY.load();
        handWn = 25.0 * speedFactor;

        // Random force: smooth (Ornstein-Uhlenbeck) noise on both axes
        {
            const double tau = 0.12;          // correlation time (s)
            const double sigma = chaos * 6.0; // m/s^2
            double a = std::exp(-dt / tau);
            double b = sigma * std::sqrt(1.0 - a * a);
            noiseAx = noiseAx * a + b * gauss(rng);
            noiseAy = noiseAy * a + b * gauss(rng);
        }

        rk4(dt);

        // Safety: NaN guard
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(vx) || !std::isfinite(vy))
            resetState();

        // Soft wall: the sphere can't travel further than 2.5x the anchor distance
        {
            const double rMax = 2.5 * SP_ANCHOR_D;
            double rr = std::sqrt(x * x + y * y);
            if (rr > rMax)
            {
                double nx = x / rr, ny = y / rr;
                x = nx * rMax;
                y = ny * rMax;
                double vr = vx * nx + vy * ny;
                if (vr > 0.0)
                {
                    vx -= vr * nx;
                    vy -= vr * ny;
                }
            }
        }

        // New random V/Oct + gate each time the sphere crosses x = 0
        double currentSign = (x >= 0) ? 1.0 : -1.0;
        if (currentSign != prevXSign && std::abs(x) > 1e-6)
        {
            generateRandomVoct();
            double durSec = getGateDurationSeconds(gateDuration);
            gateTimer = static_cast<int>(args.sampleRate * durSec);
        }
        prevXSign = currentSign;

        if (gateTimer > 0)
            gateTimer--;

        // +-5 V corresponds to the anchor distance
        outputs[X_OUTPUT].setVoltage(rack::math::clamp((float)(x / SP_ANCHOR_D * 5.0), -5.0f, 5.0f));
        outputs[Y_OUTPUT].setVoltage(rack::math::clamp((float)(y / SP_ANCHOR_D * 5.0), -5.0f, 5.0f));
        outputs[VOCT_OUTPUT].setVoltage((float)voctVoltage);
        outputs[GATE_OUTPUT].setVoltage(gateTimer > 0 ? 10.0f : 0.0f);

        float speed = std::sqrt((float)(vx * vx + vy * vy));
        float bright = std::min(1.0f, speed / 5.0f);
        lights[MASS_LIGHT].setBrightness(bright);

        // ---- display trail (stored normalised by display reach) ----
        const float trailStep = 1.f / SP_TRAIL_HZ;
        trailTimer += args.sampleTime;
        if (trailTimer >= trailStep)
        {
            trailTimer -= trailStep;
            if (trailTimer > trailStep)
                trailTimer = 0.f;
            float reach = (float)SP_ANCHOR_D * SP_REACH;
            trailX[trailHead] = (float)x / reach;
            trailY[trailHead] = (float)y / reach;
            trailHead = (trailHead + 1) % SP_TRAIL_N;
            if (trailCount < SP_TRAIL_N)
                trailCount++;
        }
    }

    // Serialises the user-chosen Voct range and gate duration.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "voctRange", json_integer(voctRange));
        json_object_set_new(root, "gateDuration", json_integer(gateDuration));
        return root;
    }

    // Restores the Voct range and gate duration from a saved patch,
    // clamping each to its valid range.
    void dataFromJson(json_t *root) override
    {
        json_t *j;
        j = json_object_get(root, "voctRange");
        if (j)
            voctRange = (VoctRange)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_RANGES - 1);
        j = json_object_get(root, "gateDuration");
        if (j)
            gateDuration = (GateDuration)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_GATE_DURS - 1);
    }
};

// ------------------------------------------------------------
// Drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace
{

    // Returns the base spring colour (cyan).
    NVGcolor colSpring() { return nvgRGB(0x5a, 0xe6, 0xff); }

    // Soft-limit: maps a normalised position (units of display reach) onto the dial so the
    // result ALWAYS lies within a circle of radius maxPx. 1:1 near the centre, smoothly
    // compressed near the rim, never crosses it.
    void mapToDial(float ux, float uy, float maxPx, float &sx, float &sy)
    {
        float r = std::sqrt(ux * ux + uy * uy);
        if (r < 1e-6f)
        {
            sx = 0.f;
            sy = 0.f;
            return;
        }
        float f = (r <= SP_KNEE) ? r : SP_KNEE + (1.f - SP_KNEE) * std::tanh((r - SP_KNEE) / (1.f - SP_KNEE));
        float k = f / r * maxPx;
        sx = ux * k;
        sy = uy * k;
    }

    // Inverse of the radial part of mapToDial: f = screen radius / maxPx (0..1) -> normalised radius
    float unmapRadius(float f)
    {
        if (f <= SP_KNEE)
            return f;
        float t = std::min((f - SP_KNEE) / (1.f - SP_KNEE), 0.97f);
        return SP_KNEE + (1.f - SP_KNEE) * std::atanh(t);
    }

    // Coil spring: dark outline + coloured core (flat, no glow or highlight)
    // Draws a zig-zag "spring" between (x0, y0) and (x1, y1) with `coils` loops
    // and lateral amplitude `amp`, using a dark outline under a coloured core.
    void drawSpring(NVGcontext *vg, float x0, float y0, float x1, float y1,
                    int coils, float amp, NVGcolor steel)
    {
        float dx = x1 - x0, dy = y1 - y0;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.f)
            return;

        float ux = dx / len, uy = dy / len;
        float nx = -uy, ny = ux;

        // Straight lead-in / lead-out so the coil looks mounted
        const float lead = std::min(8.f, len * 0.18f);

        // Draws the whole zig-zag with a given stroke width and colour.
        auto zigzag = [&](float width, NVGcolor col)
        {
            nvgBeginPath(vg);
            nvgMoveTo(vg, x0, y0);
            nvgLineTo(vg, x0 + ux * lead, y0 + uy * lead);
            int steps = coils * 2;
            float body = len - 2.f * lead;
            for (int i = 1; i < steps; ++i)
            {
                float t = (float)i / steps;
                float side = (i % 2 == 0) ? -1.f : 1.f;
                nvgLineTo(vg, x0 + ux * (lead + body * t) + nx * amp * side,
                          y0 + uy * (lead + body * t) + ny * amp * side);
            }
            nvgLineTo(vg, x1 - ux * lead, y1 - uy * lead);
            nvgLineTo(vg, x1, y1);
            nvgStrokeColor(vg, col);
            nvgStrokeWidth(vg, width);
            nvgLineCap(vg, NVG_ROUND);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
        };

        zigzag(3.8f, nvgRGB(0x05, 0x08, 0x0c)); // outline (same as display ground)
        zigzag(2.0f, steel);                    // core
    }

    // Flat mass disc with a rim
    // Draws the pendulum bob as a filled orange disc with a lighter rim.
    void drawBob(NVGcontext *vg, float cx, float cy, float r)
    {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillColor(vg, nvgRGB(0xd0, 0x50, 0x1a));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGB(0xff, 0xb0, 0x80));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Flat anchor post
    // Draws one spring anchor post as a filled grey circle with a hairline rim.
    void drawAnchor(NVGcontext *vg, float cx, float cy)
    {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, 5.f);
        nvgFillColor(vg, nvgRGB(0x4a, 0x52, 0x5e));
        nvgFill(vg);
        nvgStrokeColor(vg, agmm::dim());
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Fading motion trail (single pass, alpha ramps up towards the newest point)
    // Draws a polyline through the given points in short segments so the alpha
    // can taper toward older points.
    void drawTrail(NVGcontext *vg, const float *xs, const float *ys, int n,
                   NVGcolor col, float width, float alpha)
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
            nvgStrokeWidth(vg, width);
            nvgStroke(vg);
        }
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, four side knob cells, the
    // CONTROL row (seven cells), and the I/O row (six cells, outputs tinted).
    struct SPPanelBackground : TransparentWidget
    {
        SPPanelBackground() { box.size = Vec(SP_PANEL_W, SP_PANEL_H); }

        // Draws all static panel art: base panel, bezel, the side knob cells,
        // the CONTROL section with its seven cells, and the I/O section with its
        // six cells.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, SP_PANEL_W, SP_PANEL_H, "SPRING PENDULUM");

            agmm::bezel(vg, SP_DISP_X, SP_DISP_Y, SP_DISP_W, SP_DISP_H);

            // Side knob cells
            const float sx[2] = {SP_SIDE_LX, SP_SIDE_RX};
            const float st[2] = {SP_SIDE_TOP1, SP_SIDE_TOP2};
            const char *sl[2][2] = {{"SPEED", "MASS"}, {"REST LEN", "SPRING K"}};
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                    agmm::cell(vg, sx[i], st[j], SP_SIDE_W, SP_SIDE_H, sl[i][j]);

            // Control row
            agmm::sectionHeader(vg, SP_PANEL_W, SP_CTRL_SECTION_Y, "CONTROL");
            const char *cl[7] = {"PUSH STR", "CHAOS", "DAMP", "GRAVITY", "PUSH", "RESET", "ENERGY"};
            for (int i = 0; i < 7; ++i)
                agmm::cell(vg, SP_CTRL_X0 + SP_CTRL_DX * i, SP_CTRL_TOP, SP_CTRL_CELL_W, SP_CTRL_CELL_H, cl[i]);

            // I/O row: inputs left, outputs right (tinted)
            agmm::sectionHeader(vg, SP_PANEL_W, SP_IO_SECTION_Y, "I / O");
            const char *io[6] = {"RST IN", "PSH IN", "X", "Y", "V/OCT", "GATE"};
            for (int i = 0; i < 6; ++i)
                agmm::cell(vg, SP_IO_X0 + SP_IO_DX * i, SP_IO_TOP, SP_IO_CELL_W, SP_IO_CELL_H, io[i], i >= 2);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Spring display widget (also handles mouse grabbing)
    // ------------------------------------------------------------
    // Live dial widget: draws the dial face, rim, tick scale, reference rings,
    // anchor posts and struts on layer 0, and the springs, bob, trail, grab
    // indicator and readouts on layer 1. Handles mouse grabbing of the bob.
    struct SPDisplayWidget : TransparentWidget
    {
        SpringPendulum *module = nullptr;

        // Mouse interaction state
        bool dragging = false;
        Vec dragPos; // pointer position in display-local pixels
        Vec dragOff; // offset from pointer to ball centre at grab time

        // Constructor: positions the widget under the display bezel.
        SPDisplayWidget(SpringPendulum *m) : module(m)
        {
            box.pos = Vec(SP_DISP_X, SP_DISP_Y);
            box.size = Vec(SP_DISP_W, SP_DISP_H);
        }

        // ---- helpers ----
        // Returns the display range in physical units (anchor distance * reach).
        static float reachUnits() { return (float)SP_ANCHOR_D * SP_REACH; }

        // Returns the drawn radius of the bob for the current mass (cube-root
        // scaling, capped to always fit inside the dial).
        float bobRadius() const
        {
            return 4.5f + 2.2f * std::cbrt((float)(module ? module->mass : 1.5));
        }

        // Converts the current physical mass position into display-local pixel
        // coordinates.
        Vec ballPixel() const
        {
            float sx, sy;
            float reach = reachUnits();
            mapToDial((float)(module->x) / reach, (float)(module->y) / reach, SP_MAXPX, sx, sy);
            return Vec(SP_DIAL_CX + sx, SP_DIAL_CY - sy);
        }

        // Convert the pointer (plus grab offset) to a physical target and hand it to the module
        // Inverts the display mapping so a screen point becomes a physical
        // position for the mass, then writes it into the module's atomics.
        void updateGrab()
        {
            if (!module)
                return;
            Vec p = dragPos.plus(dragOff);
            float sx = p.x - SP_DIAL_CX;
            float sy = -(p.y - SP_DIAL_CY);
            float rs = std::sqrt(sx * sx + sy * sy);
            float ux = 0.f, uy = 0.f;
            if (rs > 1e-4f)
            {
                float f = std::min(rs / SP_MAXPX, 0.98f);
                float r = unmapRadius(f);
                ux = sx / rs * r;
                uy = sy / rs * r;
            }
            module->grabX = (double)(ux * reachUnits());
            module->grabY = (double)(uy * reachUnits());
        }

        // ---- mouse events ----
        // Left press: if the press is on/near the ball, grab it where it is;
        // otherwise, if inside the dial, pull the ball to the cursor.
        void onButton(const ButtonEvent &e) override
        {
            if (module && e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS)
            {
                Vec ball = ballPixel();
                float dBall = e.pos.minus(ball).norm();
                float dCentre = e.pos.minus(Vec(SP_DIAL_CX, SP_DIAL_CY)).norm();
                bool onBall = dBall <= bobRadius() + 12.f;

                // Click on / near the ball: grab it where it is.
                // Click elsewhere inside the dial: the ball is pulled to the cursor.
                if (onBall || dCentre <= SP_DIAL_R + 4.f)
                {
                    dragging = true;
                    dragPos = e.pos;
                    dragOff = onBall ? ball.minus(e.pos) : Vec(0.f, 0.f);
                    updateGrab();
                    module->grabbed = true;
                    e.consume(this);
                    return;
                }
            }
            TransparentWidget::onButton(e);
        }

        // Drag: moves the grab target with the pointer and updates the module.
        void onDragMove(const DragMoveEvent &e) override
        {
            if (!dragging || !module)
                return;
            dragPos = dragPos.plus(e.mouseDelta.div(getAbsoluteZoom()));
            updateGrab();
        }

        // Drag end: releases the grab; the mass keeps its current velocity.
        void onDragEnd(const DragEndEvent &e) override
        {
            dragging = false;
            if (module)
                module->grabbed = false; // ball is released and keeps its velocity
            TransparentWidget::onDragEnd(e);
        }

        // ---- drawing ----
        // Static layer: fills the display, draws the header/footer hairlines,
        // the dial face, the double rim, the tick scale, the reference rings
        // and the cross hair.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float cx = SP_DIAL_CX;
            const float cy = SP_DIAL_CY;

            agmm::screen(vg, 0.f, 0.f, SP_DISP_W, SP_DISP_H);

            // Header / footer separators
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, SP_HEAD_H - 0.5f);
            nvgLineTo(vg, SP_DISP_W, SP_HEAD_H - 0.5f);
            nvgMoveTo(vg, 0.f, SP_DISP_H - SP_FOOT_H + 0.5f);
            nvgLineTo(vg, SP_DISP_W, SP_DISP_H - SP_FOOT_H + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Dial face (flat)
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, SP_DIAL_R);
            nvgFillColor(vg, nvgRGB(0x09, 0x10, 0x17));
            nvgFill(vg);

            // Dial rim (double line)
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, SP_DIAL_R);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, SP_DIAL_R + 3.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Tick scale around the dial (every 5 deg, major every 30 deg)
            nvgLineCap(vg, NVG_BUTT);
            for (int i = 0; i < 72; ++i)
            {
                float a = (float)i * (float)M_PI / 36.f;
                bool major = (i % 6 == 0);
                float r0 = SP_DIAL_R + 5.f;
                float r1 = SP_DIAL_R + (major ? 9.5f : 7.5f);
                nvgBeginPath(vg);
                nvgMoveTo(vg, cx + std::sin(a) * r0, cy - std::cos(a) * r0);
                nvgLineTo(vg, cx + std::sin(a) * r1, cy - std::cos(a) * r1);
                nvgStrokeColor(vg, major ? nvgRGBA(0x3d, 0xe0, 0xff, 130) : nvgRGBA(255, 255, 255, 40));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // Range rings
            for (int k = 1; k <= 3; ++k)
            {
                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, SP_DIAL_R * (float)k / 4.f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, k % 2 ? 14 : 22));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // Cross hair
            nvgBeginPath(vg);
            nvgMoveTo(vg, cx, cy - SP_DIAL_R + 2.f);
            nvgLineTo(vg, cx, cy + SP_DIAL_R - 2.f);
            nvgMoveTo(vg, cx - SP_DIAL_R + 2.f, cy);
            nvgLineTo(vg, cx + SP_DIAL_R - 2.f, cy);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 20));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the anchor struts, trail, four springs (with
        // tension-based colour), anchor posts, bob, grab ring and the header/
        // footer readouts. Uses demo data when there is no module.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float cx = SP_DIAL_CX;
            const float cy = SP_DIAL_CY;

            // Read state
            double px = 0, py = 0, vx = 0, vy = 0, m = 1, K = 10, L = 0.5, g = 0.0;
            bool grabbedNow = false;
            if (module)
            {
                px = module->x;
                py = module->y;
                vx = module->vx;
                vy = module->vy;
                m = module->mass;
                K = module->springK;
                L = module->restLen;
                g = module->g;
                grabbedNow = module->grabbed.load();
            }
            else
            {
                px = 0.30;
                py = -0.18;
                vx = 1.5;
                vy = 0.6;
                m = 1.5;
                K = 20;
                L = 0.5;
                g = 0.0;
            }

            const float reach = reachUnits();

            // Mass radius (cube root of mass); max ~9.2 px, always fits in SP_MAXPX margin
            float bobR = 4.5f + 2.2f * std::cbrt((float)m);

            // Everything below is clipped to the display as a safety net
            nvgSave(vg);
            nvgScissor(vg, 0, 0, SP_DISP_W, SP_DISP_H);

            // Mass position on screen (soft-limited into the dial)
            float sx, sy;
            mapToDial((float)px / reach, (float)py / reach, SP_MAXPX, sx, sy);
            float mx = cx + sx;
            float my = cy - sy; // +y physical = up on screen

            // Anchor positions: N, E, S, W (same mapping, so springs end on the posts)
            const float dirX[4] = {0.f, 1.f, 0.f, -1.f};
            const float dirY[4] = {1.f, 0.f, -1.f, 0.f};
            float aX[4], aY[4];
            for (int i = 0; i < 4; ++i)
            {
                float ax, ay;
                mapToDial(dirX[i] * (float)SP_ANCHOR_D / reach,
                          dirY[i] * (float)SP_ANCHOR_D / reach, SP_MAXPX, ax, ay);
                aX[i] = cx + ax;
                aY[i] = cy - ay;
            }

            // Struts from the anchor posts out to the rim
            nvgLineCap(vg, NVG_ROUND);
            for (int i = 0; i < 4; ++i)
            {
                float ex = cx + dirX[i] * (SP_DIAL_R + 1.f);
                float ey = cy - dirY[i] * (SP_DIAL_R + 1.f);
                nvgBeginPath(vg);
                nvgMoveTo(vg, aX[i], aY[i]);
                nvgLineTo(vg, ex, ey);
                nvgStrokeColor(vg, nvgRGB(0x4a, 0x52, 0x5e));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }

            // Trail (same mapping)
            int tn = 0;
            float tx[SP_TRAIL_N], ty[SP_TRAIL_N];
            if (module)
            {
                tn = std::min(std::max(module->trailCount, 0), SP_TRAIL_N);
                for (int i = 0; i < tn; ++i)
                {
                    int idx = ((module->trailHead - tn + i) % SP_TRAIL_N + SP_TRAIL_N) % SP_TRAIL_N;
                    float ax, ay;
                    mapToDial(module->trailX[idx], module->trailY[idx], SP_MAXPX, ax, ay);
                    tx[i] = cx + ax;
                    ty[i] = cy - ay;
                }
            }
            else
            {
                tn = 90;
                for (int i = 0; i < tn; ++i)
                {
                    float k = (float)i / (float)(tn - 1);
                    float a1 = 6.0f * k;
                    float rr = 0.18f + 0.12f * std::sin(3.f * a1);
                    float ax, ay;
                    mapToDial(rr * std::cos(a1), rr * std::sin(a1), SP_MAXPX, ax, ay);
                    tx[i] = cx + ax;
                    ty[i] = cy - ay;
                }
            }
            drawTrail(vg, tx, ty, tn, colSpring(), 1.5f, 0.6f);

            // Four springs (colour shifts cyan -> orange with tension)
            for (int i = 0; i < 4; ++i)
            {
                double ddx = dirX[i] * SP_ANCHOR_D - px;
                double ddy = dirY[i] * SP_ANCHOR_D - py;
                float rr = (float)std::sqrt(ddx * ddx + ddy * ddy);
                float tension = std::min(1.f, std::fabs(rr - (float)L) / 0.6f);
                NVGcolor col = nvgLerpRGBA(colSpring(), agmm::warm(), tension);
                drawSpring(vg, aX[i], aY[i], mx, my, 11, 3.2f, col);
            }

            // Anchor posts on top of the spring ends
            for (int i = 0; i < 4; ++i)
                drawAnchor(vg, aX[i], aY[i]);

            // Mass
            drawBob(vg, mx, my, bobR);

            // Grab indicator: one ring around the ball while the mouse holds it
            if (grabbedNow)
            {
                nvgBeginPath(vg);
                nvgCircle(vg, mx, my, bobR + 4.f);
                nvgStrokeColor(vg, agmm::accent());
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // Readouts: header row (y = 8) and footer row, both outside the dial
            char buf[32];
            NVGcolor c1 = agmm::accent();
            NVGcolor cm = agmm::warm();
            NVGcolor cd = agmm::dim();
            const float hy = SP_HEAD_H / 2.f;
            const float fy = SP_DISP_H - SP_FOOT_H / 2.f;

            snprintf(buf, sizeof(buf), "X %+.2f  Y %+.2f", (float)px, (float)py);
            agmm::text(vg, 5.f, hy, buf, 7.f, c1, agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "VX %+.1f  VY %+.1f", (float)vx, (float)vy);
            agmm::text(vg, SP_DISP_W - 5.f, hy, buf, 7.f, cm, agmm::ALIGN_R, 0.5f);

            snprintf(buf, sizeof(buf), "K %.0f", (float)K);
            agmm::text(vg, 5.f, fy, buf, 7.f, cd, agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "G %.1f", (float)g);
            agmm::text(vg, SP_DISP_W - 5.f, fy, buf, 7.f, cd, agmm::ALIGN_R, 0.5f);
            if (grabbedNow)
                agmm::text(vg, SP_DISP_W / 2.f, fy, "GRAB", 7.f, c1, agmm::ALIGN_C, 1.f);

            nvgRestore(vg);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
// Places the static panel, screws, live spring display, the four side knobs,
// the seven CONTROL row items (four knobs, two buttons, one LED), and the
// six I/O jacks. Also builds the context menu with Voct-range and
// gate-duration submenus.
struct SpringPendulumWidget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, display, side knobs,
    // CONTROL row and I/O jacks.
    SpringPendulumWidget(SpringPendulum *module)
    {
        setModule(module);
        box.size = Vec(SP_PANEL_W, SP_PANEL_H);

        addChild(new SPPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SPDisplayWidget(module));

        // Side knobs
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_SIDE_LX, SP_SIDE_TOP1 + SP_CTL_DY), module, SpringPendulum::SPEED_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_SIDE_LX, SP_SIDE_TOP2 + SP_CTL_DY), module, SpringPendulum::MASS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_SIDE_RX, SP_SIDE_TOP1 + SP_CTL_DY), module, SpringPendulum::LENGTH_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_SIDE_RX, SP_SIDE_TOP2 + SP_CTL_DY), module, SpringPendulum::SPRING_K_PARAM));

        // Control row
        const float cy = SP_CTRL_TOP + SP_CTL_DY;
        auto cxAt = [](int i)
        { return SP_CTRL_X0 + SP_CTRL_DX * (float)i; };
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(cxAt(0), cy), module, SpringPendulum::PUSH_STRENGTH_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(cxAt(1), cy), module, SpringPendulum::CHAOS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(cxAt(2), cy), module, SpringPendulum::DAMP_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(cxAt(3), cy), module, SpringPendulum::GRAVITY_PARAM));
        addParam(createParamCentered<LEDButton>(
            Vec(cxAt(4), cy), module, SpringPendulum::PUSH_BUTTON));
        addParam(createParamCentered<LEDButton>(
            Vec(cxAt(5), cy), module, SpringPendulum::RESET_BUTTON));
        addChild(createLightCentered<MediumLight<agmm::CyanLight>>(
            Vec(cxAt(6), cy), module, SpringPendulum::MASS_LIGHT));

        // I/O row: inputs first, then outputs
        const float jy = SP_IO_TOP + SP_IO_JACK_DY;
        auto jx = [](int i)
        { return SP_IO_X0 + SP_IO_DX * (float)i; };
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(jx(0), jy), module, SpringPendulum::RESET_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(jx(1), jy), module, SpringPendulum::PUSH_CV_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(jx(2), jy), module, SpringPendulum::X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(jx(3), jy), module, SpringPendulum::Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(jx(4), jy), module, SpringPendulum::VOCT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(jx(5), jy), module, SpringPendulum::GATE_OUTPUT));
    }

    // Adds the right-click menu: a "Voct range" submenu (with re-roll on
    // selection) and a "Gate duration" submenu (with a checkmark on the
    // active option).
    void appendContextMenu(ui::Menu *menu) override
    {
        SpringPendulum *module = dynamic_cast<SpringPendulum *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Voct range", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                                             for (int i = 0; i < SpringPendulum::NUM_RANGES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(rangeNames[i],
                                                                                  CHECKMARK(module->voctRange == i),
                                                                                  [=]()
                                                                                  {
                                                                                      module->voctRange = (SpringPendulum::VoctRange)i;
                                                                                      module->generateRandomVoct();
                                                                                  }));
                                             }
                                         }));

        menu->addChild(createSubmenuItem("Gate duration", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                                             for (int i = 0; i < SpringPendulum::NUM_GATE_DURS; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(gateNames[i],
                                                                                  CHECKMARK(module->gateDuration == i),
                                                                                  [=]()
                                                                                  {
                                                                                      module->gateDuration = (SpringPendulum::GateDuration)i;
                                                                                  }));
                                             }
                                         }));
    }
};

Model *modelSpringPendulum = createModel<SpringPendulum, SpringPendulumWidget>("SpringPendulum");