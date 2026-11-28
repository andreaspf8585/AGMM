#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <random>
#include <limits>
#include <array>
#include "Knob_arcs.hpp"

// ============================================================================
// RandomWalk
// ============================================================================
//
// OVERVIEW
// --------
// RandomWalk is a 28 HP polyphonic random-voltage generator built from up to
// 16 independently wandering "walkers" confined to a ±10 V square arena. Each
// walker takes Gaussian-distributed random steps; the STEP knob sets the step
// magnitude and the SPEED knob scales how often those steps happen. A
// SMOOTHNESS control adds AR(1) momentum so that consecutive steps correlate,
// turning the pure white-noise walk into a meandering or drifting path.
//
// For every walker, the module exposes:
//   X, Y               its current position (bipolar, ±10 V)
//   GATE X, GATE Y     a short pulse whenever the walker crosses an axis
//   V/OCT X, V/OCT Y   a latched random voltage, regenerated on every axis
//                      crossing (range selectable in the context menu)
//
// Visual feedback:
//   - A 200×200 arena display shows each walker's fading trail, its current
//     position, wall-hit and axis-crossing flashes, and (when smoothness > 0)
//     a small tick showing its momentum direction.
//   - A row of 16 X/Y bar pairs shows each walker's current X and Y voltage.
//
// All outputs are polyphonic; the channel count equals the current number of
// walkers. X, Y, GATE X/Y and V/OCT X/Y are updated every sample; the trails
// are refreshed at a lower rate so they stay visible at any walk speed.
//
// File structure:
//   1. Layout constants (28 HP = 420×380 px).
//   2. RandomWalk module: parameter/CV handling, per-walker physics, gate
//      and V/Oct generation, JSON persistence.
//   3. Anonymous-namespace UI: colour/trail/header helpers, the static
//      panel, the arena display and the position-bars widget.
//   4. RandomWalkWidget: places all controls, jacks and displays, and
//      builds the context menu.
//   5. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int RW_N = 16;               // max walkers
static const int RW_TRAIL_N = 200;        // trail points per walker
static const float RW_TRAIL_DIST = 0.15f; // min distance (V) between trail points
static const float RW_TRAIL_HZ = 240.f;   // max trail sample rate

static const float RW_PANEL_W = RACK_GRID_WIDTH * 28; // 420
static const float RW_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (220 x 232): header row, 200 x 200 arena at (10,16), footer row.
// The arena is 9.5 px per volt with a 5 px rail, so +-10 V fills the playfield.
static const float RW_DISP_W = 220.f;
static const float RW_DISP_H = 232.f;
static const float RW_DISP_X = 100.f;
static const float RW_DISP_Y = 34.f;
static const float RW_ARENA_X = 10.f;
static const float RW_ARENA_Y = 16.f;
static const float RW_ARENA = 200.f;
static const float RW_SCALE = 9.5f;
static const float RW_MARGIN = 5.f;

// Left control block and right output block: 2 columns x 4 rows of 40 x 50 cells
static const float RW_LCOL[2] = {28.f, 72.f};
static const float RW_RCOL[2] = {348.f, 392.f};
static const float RW_ROW_TOP[4] = {56.f, 112.f, 168.f, 224.f};
static const float RW_CELL_W = 40.f, RW_CELL_H = 50.f;
// Returns the knob / jack centre y for a control row whose top edge is `top`
// (30 px below the top edge).
static float rwCtlY(float top) { return top + 30.f; } // knob / jack centre inside a cell

// Position strip: plate, header row, 16 bar pairs, number row
static const float RW_STRIP_TOP = 276.f;
static const float RW_STRIP_H = 80.f;
static const float RW_BARS_X = 40.f;
static const float RW_BARS_Y = 292.f;
static const float RW_BARS_W = 340.f;
static const float RW_BARS_H = 54.f;
static const float RW_BAR_H = 40.f; // bar area height inside the widget

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// Polyphonic random-walk generator: up to 16 walkers moving independently in
// a ±10 V arena, each producing position, gate and V/Oct outputs.
struct RandomWalk : Module
{
    enum ParamId
    {
        SPEED_PARAM,
        STEP_SIZE_PARAM,
        RESET_BUTTON,
        NUM_BALLS_PARAM,
        SMOOTHNESS_PARAM, // AR(1) momentum: 0 = none, 1 = constant direction
        PARAMS_LEN
    };
    enum InputId
    {
        RESET_CV_INPUT,
        SPEED_CV_INPUT,  // new
        STEP_CV_INPUT,   // new
        SMOOTH_CV_INPUT, // new
        BALLS_CV_INPUT,  // new
        INPUTS_LEN
    };
    enum OutputId
    {
        X_OUTPUT,
        Y_OUTPUT,
        GATE_X_OUTPUT,
        GATE_Y_OUTPUT,
        VOCT_X_OUTPUT,
        VOCT_Y_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // Selectable ranges for the random V/Oct outputs, exposed via the menu.
    enum VoctRange
    {
        RANGE_0_1,
        RANGE_N1_1,
        RANGE_N2_2,
        RANGE_N5_5,
        RANGE_0_5,
        NUM_RANGES
    };
    // Selectable gate durations, exposed via the menu.
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

    static constexpr int MAX_BALLS = 16;

    // Per-ball state
    std::array<double, MAX_BALLS> x{};
    std::array<double, MAX_BALLS> y{};
    std::array<double, MAX_BALLS> prevX{};
    std::array<double, MAX_BALLS> prevY{};
    std::array<double, MAX_BALLS> prevDeltaX{}; // previous step increment
    std::array<double, MAX_BALLS> prevDeltaY{};
    std::array<int, MAX_BALLS> gateXTimer{};
    std::array<int, MAX_BALLS> gateYTimer{};
    std::array<double, MAX_BALLS> voctX{};
    std::array<double, MAX_BALLS> voctY{};

    int numBalls = 1;

    double speedFactor = 1.0;
    double stepSize = 0.001;
    double smoothness = 0.0; // momentum

    VoctRange voctXRange = RANGE_N1_1, voctYRange = RANGE_N1_1;
    GateDuration gateXDuration = GATE_5MS, gateYDuration = GATE_5MS;

    std::mt19937 rng;
    dsp::SchmittTrigger resetTrigger;
    bool lastResetState = false;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float trailX[MAX_BALLS][RW_TRAIL_N] = {};
    float trailY[MAX_BALLS][RW_TRAIL_N] = {};
    int trailHead[MAX_BALLS] = {};
    int trailCount[MAX_BALLS] = {};
    float lastTX[MAX_BALLS] = {};
    float lastTY[MAX_BALLS] = {};
    float trailTimer = 0.f;
    float flashXc[MAX_BALLS] = {};   // X zero crossing (vertical axis)
    float flashYc[MAX_BALLS] = {};   // Y zero crossing (horizontal axis)
    float flashWall[MAX_BALLS] = {}; // reflection at a wall
    int wallSide[MAX_BALLS] = {};    // 0 left, 1 right, 2 top, 3 bottom
    float wallPos[MAX_BALLS] = {};   // position along that wall (V)
    float xcPos[MAX_BALLS] = {};     // Y value where X crossed zero
    float ycPos[MAX_BALLS] = {};     // X value where Y crossed zero

    // Constructor: configures all five parameters (with snapping where
    // useful), the five CV inputs, and the six polyphonic outputs. Seeds the
    // RNG, resets the walkers, and initialises each walker's random V/Oct
    // value plus a tiny momentum offset (so smoothness = 1 still moves).
    RandomWalk()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        // Speed is a time-scale factor: 0.1x .. 297x (matches speedFactor below)
        configParam(SPEED_PARAM, 0.f, 30.f, 0.2f, "Speed", "x", 0.f, 9.9f, 0.1f);
        configParam(STEP_SIZE_PARAM, 0.01f, 0.1f, 0.1f, "Step size", " V");
        configButton(RESET_BUTTON, "Reset");
        configParam(NUM_BALLS_PARAM, 1.f, 16.f, 1.f, "Number of balls");
        getParamQuantity(NUM_BALLS_PARAM)->snapEnabled = true;
        configParam(SMOOTHNESS_PARAM, 0.f, 1.f, 0.f, "Smoothness (momentum)", "%", 0.f, 100.f);

        configInput(RESET_CV_INPUT, "Reset gate");
        configInput(SPEED_CV_INPUT, "Speed CV (10 V adds 30 to the knob)");
        configInput(STEP_CV_INPUT, "Step size CV (10 V adds 0.09 V)");
        configInput(SMOOTH_CV_INPUT, "Smoothness CV (10 V adds 100%)");
        configInput(BALLS_CV_INPUT, "Number of balls CV (10 V adds 15 balls)");

        configOutput(X_OUTPUT, "X (-10V..10V, polyphonic)");
        configOutput(Y_OUTPUT, "Y (-10V..10V, polyphonic)");
        configOutput(GATE_X_OUTPUT, "Gate X (zero crossing, polyphonic)");
        configOutput(GATE_Y_OUTPUT, "Gate Y (zero crossing, polyphonic)");
        configOutput(VOCT_X_OUTPUT, "V/Oct X (random on zero crossing, polyphonic)");
        configOutput(VOCT_Y_OUTPUT, "V/Oct Y (random on zero crossing, polyphonic)");

        std::random_device rd;
        rng.seed(rd());
        resetWalkers();
        for (int i = 0; i < MAX_BALLS; ++i)
        {
            generateRandomVoctForBall(i, true);
            generateRandomVoctForBall(i, false);
            // give each ball a tiny random initial velocity so smoothness=1 still moves
            std::uniform_real_distribution<double> tiny(-1e-4, 1e-4);
            prevDeltaX[i] = tiny(rng);
            prevDeltaY[i] = tiny(rng);
        }
    }

    // Clears every walker's position, trails, and flash data. Intentionally
    // keeps prevDeltaX/Y so momentum survives a reset if the user wants it to.
    void resetWalkers()
    {
        for (int i = 0; i < MAX_BALLS; ++i)
        {
            x[i] = 0.0;
            y[i] = 0.0;
            prevX[i] = 0.0;
            prevY[i] = 0.0;
            // do NOT reset prevDelta here - keep small random values to avoid stuck at zero
            trailHead[i] = 0;
            trailCount[i] = 0;
            lastTX[i] = lastTY[i] = 0.f;
            flashXc[i] = flashYc[i] = flashWall[i] = 0.f;
        }
        trailTimer = 0.f;
    }

    // Rack reset hook: resets the walkers, leaving their momentum intact.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        resetWalkers();
    }

    // Generates a new random V/Oct value for walker `ball` on the X or Y
    // axis, scaled according to that axis's selected VoctRange.
    void generateRandomVoctForBall(int ball, bool forX)
    {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        VoctRange range = forX ? voctXRange : voctYRange;
        double &target = forX ? voctX[ball] : voctY[ball];
        switch (range)
        {
        case RANGE_0_1:
            target = (raw + 1.0) / 2.0;
            break;
        case RANGE_N1_1:
            target = raw;
            break;
        case RANGE_N2_2:
            target = raw * 2.0;
            break;
        case RANGE_N5_5:
            target = raw * 5.0;
            break;
        case RANGE_0_5:
            target = (raw + 1.0) / 2.0 * 5.0;
            break;
        default:
            target = raw;
            break;
        }
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

    // Reads the four main controls plus their CV inputs, computes the
    // effective speed / step / smoothness / ball-count values, and updates
    // the walker count (restarting the trails of any newly active walkers).
    void updateParameters()
    {
        float spd = params[SPEED_PARAM].getValue();
        if (inputs[SPEED_CV_INPUT].isConnected())
            spd = rack::math::clamp(spd + inputs[SPEED_CV_INPUT].getVoltage() * 3.f, 0.f, 30.f);
        speedFactor = 0.1 + spd * 9.9;

        float st = params[STEP_SIZE_PARAM].getValue();
        if (inputs[STEP_CV_INPUT].isConnected())
            st = rack::math::clamp(st + inputs[STEP_CV_INPUT].getVoltage() * 0.009f, 0.01f, 0.1f);
        stepSize = st;

        float sm = params[SMOOTHNESS_PARAM].getValue();
        if (inputs[SMOOTH_CV_INPUT].isConnected())
            sm = rack::math::clamp(sm + inputs[SMOOTH_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        smoothness = sm;

        float nb = params[NUM_BALLS_PARAM].getValue();
        if (inputs[BALLS_CV_INPUT].isConnected())
            nb += inputs[BALLS_CV_INPUT].getVoltage() * 1.5f;
        int newNum = rack::math::clamp((int)std::round(nb), 1, MAX_BALLS);
        if (newNum != numBalls)
        {
            // restart the trails of balls that join, so they don't draw a stale streak
            for (int i = numBalls; i < newNum; ++i)
            {
                trailHead[i] = 0;
                trailCount[i] = 0;
            }
            numBalls = newNum;
        }
    }

    // Main audio callback: reads parameters and CVs, handles the reset
    // button/gate, advances every walker with AR(1)-momentum Gaussian steps,
    // reflects them off the walls, detects axis zero crossings (firing gates
    // and re-rolling that axis's V/Oct), writes the six polyphonic outputs,
    // and samples the trail buffer at a fixed rate.
    void process(const ProcessArgs &args) override
    {
        updateParameters();

        bool resetNow = (params[RESET_BUTTON].getValue() > 0.5f) ||
                        (inputs[RESET_CV_INPUT].getVoltage() > 1.0f);
        if (resetNow && !lastResetState)
        {
            resetWalkers();
            // re-seed random initial deltas so smoothness=1 moves after reset
            std::uniform_real_distribution<double> tiny(-1e-4, 1e-4);
            for (int i = 0; i < MAX_BALLS; ++i)
            {
                prevDeltaX[i] = tiny(rng);
                prevDeltaY[i] = tiny(rng);
            }
        }
        lastResetState = resetNow;

        // Decay the visual flashes
        for (int i = 0; i < MAX_BALLS; ++i)
        {
            flashXc[i] = std::max(0.f, flashXc[i] - 4.f * args.sampleTime);
            flashYc[i] = std::max(0.f, flashYc[i] - 4.f * args.sampleTime);
            flashWall[i] = std::max(0.f, flashWall[i] - 5.f * args.sampleTime);
        }

        // Standard deviation per sample for a fresh random step
        double sigma = stepSize * std::sqrt(speedFactor * args.sampleTime);
        std::normal_distribution<double> noise(0.0, sigma);

        for (int i = 0; i < numBalls; ++i)
        {
            prevX[i] = x[i];
            prevY[i] = y[i];

            // Generate independent white noise increments
            double rawDeltaX = noise(rng);
            double rawDeltaY = noise(rng);

            // AR(1) momentum: delta = smoothness * prevDelta + (1-smoothness) * rawDelta
            double deltaX = smoothness * prevDeltaX[i] + (1.0 - smoothness) * rawDeltaX;
            double deltaY = smoothness * prevDeltaY[i] + (1.0 - smoothness) * rawDeltaY;

            // Update previous delta for next step
            prevDeltaX[i] = deltaX;
            prevDeltaY[i] = deltaY;

            // Add to position
            x[i] += deltaX;
            y[i] += deltaY;

            // Reflect at boundaries
            int side = -1;
            float wpos = 0.f;
            if (x[i] > 10.0)
            {
                x[i] = 20.0 - x[i];
                side = 1;
                wpos = (float)y[i];
            }
            if (x[i] < -10.0)
            {
                x[i] = -20.0 - x[i];
                side = 0;
                wpos = (float)y[i];
            }
            if (y[i] > 10.0)
            {
                y[i] = 20.0 - y[i];
                side = 2;
                wpos = (float)x[i];
            }
            if (y[i] < -10.0)
            {
                y[i] = -20.0 - y[i];
                side = 3;
                wpos = (float)x[i];
            }
            if (side >= 0)
            {
                flashWall[i] = 1.f;
                wallSide[i] = side;
                wallPos[i] = wpos;
            }

            // Zero crossing detection
            if ((prevX[i] <= 0 && x[i] > 0) || (prevX[i] >= 0 && x[i] < 0))
            {
                generateRandomVoctForBall(i, true);
                double dur = getGateDurationSeconds(gateXDuration);
                gateXTimer[i] = static_cast<int>(args.sampleRate * dur);
                flashXc[i] = 1.f;
                xcPos[i] = (float)y[i];
            }
            if ((prevY[i] <= 0 && y[i] > 0) || (prevY[i] >= 0 && y[i] < 0))
            {
                generateRandomVoctForBall(i, false);
                double dur = getGateDurationSeconds(gateYDuration);
                gateYTimer[i] = static_cast<int>(args.sampleRate * dur);
                flashYc[i] = 1.f;
                ycPos[i] = (float)x[i];
            }

            if (gateXTimer[i] > 0)
                gateXTimer[i]--;
            if (gateYTimer[i] > 0)
                gateYTimer[i]--;
        }

        // Set polyphonic outputs
        outputs[X_OUTPUT].setChannels(numBalls);
        outputs[Y_OUTPUT].setChannels(numBalls);
        outputs[GATE_X_OUTPUT].setChannels(numBalls);
        outputs[GATE_Y_OUTPUT].setChannels(numBalls);
        outputs[VOCT_X_OUTPUT].setChannels(numBalls);
        outputs[VOCT_Y_OUTPUT].setChannels(numBalls);

        for (int i = 0; i < numBalls; ++i)
        {
            outputs[X_OUTPUT].setVoltage(static_cast<float>(x[i]), i);
            outputs[Y_OUTPUT].setVoltage(static_cast<float>(y[i]), i);
            outputs[GATE_X_OUTPUT].setVoltage(gateXTimer[i] > 0 ? 10.0f : 0.0f, i);
            outputs[GATE_Y_OUTPUT].setVoltage(gateYTimer[i] > 0 ? 10.0f : 0.0f, i);
            outputs[VOCT_X_OUTPUT].setVoltage(static_cast<float>(voctX[i]), i);
            outputs[VOCT_Y_OUTPUT].setVoltage(static_cast<float>(voctY[i]), i);
        }

        // ---- trail data for the display ----
        // A point is stored when a walker has moved far enough, so the path stays
        // visible at any speed.
        trailTimer += args.sampleTime;
        if (trailTimer >= 1.f / RW_TRAIL_HZ)
        {
            trailTimer = 0.f;
            for (int i = 0; i < numBalls; ++i)
            {
                float px = (float)x[i], py = (float)y[i];
                float dx = px - lastTX[i], dy = py - lastTY[i];
                if (trailCount[i] == 0 || dx * dx + dy * dy >= RW_TRAIL_DIST * RW_TRAIL_DIST)
                {
                    trailX[i][trailHead[i]] = px;
                    trailY[i][trailHead[i]] = py;
                    trailHead[i] = (trailHead[i] + 1) % RW_TRAIL_N;
                    if (trailCount[i] < RW_TRAIL_N)
                        trailCount[i]++;
                    lastTX[i] = px;
                    lastTY[i] = py;
                }
            }
        }
    }

    // Serialises the V/Oct ranges, gate durations, ball count and smoothness
    // so a patch reopens with the same walker behaviour.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "voctXRange", json_integer(voctXRange));
        json_object_set_new(root, "voctYRange", json_integer(voctYRange));
        json_object_set_new(root, "gateXDuration", json_integer(gateXDuration));
        json_object_set_new(root, "gateYDuration", json_integer(gateYDuration));
        // the knob values (not the CV-modulated ones)
        json_object_set_new(root, "numBalls", json_integer((int)std::round(params[NUM_BALLS_PARAM].getValue())));
        json_object_set_new(root, "smoothness", json_real(params[SMOOTHNESS_PARAM].getValue()));
        return root;
    }

    // Restores the V/Oct ranges, gate durations, ball count and smoothness
    // from a saved patch, clamping each to its valid range.
    void dataFromJson(json_t *root) override
    {
        json_t *j;
        j = json_object_get(root, "voctXRange");
        if (j)
            voctXRange = (VoctRange)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_RANGES - 1);
        j = json_object_get(root, "voctYRange");
        if (j)
            voctYRange = (VoctRange)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_RANGES - 1);
        j = json_object_get(root, "gateXDuration");
        if (j)
            gateXDuration = (GateDuration)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_GATE_DURS - 1);
        j = json_object_get(root, "gateYDuration");
        if (j)
            gateYDuration = (GateDuration)rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_GATE_DURS - 1);
        j = json_object_get(root, "numBalls");
        if (j)
        {
            int nb = rack::math::clamp((int)json_integer_value(j), 1, MAX_BALLS);
            numBalls = nb;
            params[NUM_BALLS_PARAM].setValue((float)nb);
        }
        j = json_object_get(root, "smoothness");
        if (j)
        {
            smoothness = (double)rack::math::clamp((float)json_number_value(j), 0.f, 1.f);
            params[SMOOTHNESS_PARAM].setValue((float)smoothness);
        }
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{
    namespace ag = agmm;

    // ------------------------------------------------------------
    // Walker helpers
    // ------------------------------------------------------------

    // Colours run from cyan through violet to magenta, so all 16 stay distinct
    // and fit the theme. The colour depends on the walker index, not on the count.
    // Returns the HSL colour for walker `i` at the requested saturation and
    // lightness; the hue sweeps from 0.5 (cyan) toward 0.95 (magenta).
    NVGcolor ballColor(int i, float sat, float light)
    {
        float h = 0.5f + 0.45f * (float)i / 15.f;
        return nvgHSL(h, sat, light);
    }

    // Fading path (older = thinner and fainter). K points per stroked segment.
    // Draws a polyline through the given points, splitting it into K-point
    // segments so the stroke width and alpha can taper toward older points.
    void drawTrail(NVGcontext *vg, const float *xs, const float *ys, int n, NVGcolor col,
                   float width, float alpha, int K)
    {
        if (n < 2)
            return;
        nvgLineCap(vg, NVG_ROUND);
        nvgLineJoin(vg, NVG_ROUND);
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
    // Panel background: title, display frame, labelled plates, strip plate
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, CONTROL/OUTPUT headers, the
    // 4x2 left control cells, the 4x2 right output cells, and the position
    // strip plate with its captions.
    struct RWPanelBackground : TransparentWidget
    {
        RWPanelBackground() { box.size = Vec(RW_PANEL_W, RW_PANEL_H); }

        // Draws all static panel art: base panel, bezel, the two block
        // headers, the eight control/jack cells (left) and eight output/reset
        // cells (right, outputs tinted), and the walker-position strip plate.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, RW_PANEL_W, RW_PANEL_H, "RANDOMWALK");
            ag::bezel(vg, RW_DISP_X, RW_DISP_Y, RW_DISP_W, RW_DISP_H);

            blockHeader(vg, 50.f, 44.f, "CONTROL", 42.f, ag::accent());
            blockHeader(vg, 370.f, 44.f, "OUTPUT", 42.f, ag::violet());

            // ---- left block: knob cell + its CV jack cell on each row ----
            const char *knobL[4] = {"SPEED", "STEP", "SMOOTH", "BALLS"};
            const char *cvL[4] = {"SPD CV", "STEP CV", "SMTH CV", "BALL CV"};
            for (int r = 0; r < 4; ++r)
            {
                ag::cell(vg, RW_LCOL[0], RW_ROW_TOP[r], RW_CELL_W, RW_CELL_H, knobL[r], false, 7.f);
                ag::cell(vg, RW_LCOL[1], RW_ROW_TOP[r], RW_CELL_W, RW_CELL_H, cvL[r], false, 7.f);
            }

            // ---- right block (outputs tinted, reset cells plain) ----
            const char *outL[8] = {"X", "Y", "GATE X", "GATE Y", "V/OCT X", "V/OCT Y", "RESET", "RESET IN"};
            for (int i = 0; i < 8; ++i)
                ag::cell(vg, RW_RCOL[i % 2], RW_ROW_TOP[i / 2], RW_CELL_W, RW_CELL_H, outL[i], i < 6, 7.f);

            // ---- position strip ----
            ag::plate(vg, RW_PANEL_W / 2.f, RW_STRIP_TOP, 352.f, RW_STRIP_H);
            ag::text(vg, 46.f, RW_STRIP_TOP + 9.f, "WALKER POSITION", 7.f, ag::accent(), ag::ALIGN_L, 1.f);
            ag::text(vg, 374.f, RW_STRIP_TOP + 9.f, "X | Y", 7.f, ag::dim(), ag::ALIGN_R, 1.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Arena display
    // ------------------------------------------------------------
    // Live arena widget: draws the playfield, grid, axes and origin ring on
    // layer 0, and the walkers' trails, position markers, momentum ticks and
    // wall/crossing flashes on layer 1, along with header/footer readouts.
    // Uses a demo scene in the module browser.
    struct RWDisplayWidget : TransparentWidget
    {
        RandomWalk *module = nullptr;

        RWDisplayWidget(RandomWalk *m) : module(m)
        {
            box.pos = Vec(RW_DISP_X, RW_DISP_Y);
            box.size = Vec(RW_DISP_W, RW_DISP_H);
        }

        // Static background: screen, header/footer hairlines, rail, playfield, grid
        // Draws the display screen, header/footer dividers, the raised rail,
        // dark playfield, 2 V rail ticks, grid lines, axes and origin ring.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, RW_DISP_W, RW_DISP_H);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 6.f, 14.5f);
            nvgLineTo(vg, RW_DISP_W - 6.f, 14.5f);
            nvgMoveTo(vg, 6.f, 217.5f);
            nvgLineTo(vg, RW_DISP_W - 6.f, 217.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgSave(vg);
            nvgTranslate(vg, RW_ARENA_X, RW_ARENA_Y);
            const float A = RW_ARENA, cx = A / 2.f, cy = A / 2.f, m = RW_MARGIN;
            const float step = 2.f * RW_SCALE; // 2 V

            // Rail
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, A, A);
            nvgFillColor(vg, nvgRGB(0x1a, 0x1e, 0x24));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, A - 1.f, A - 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Playfield
            nvgBeginPath(vg);
            nvgRect(vg, m, m, A - 2.f * m, A - 2.f * m);
            nvgFillColor(vg, nvgRGB(0x05, 0x09, 0x0e));
            nvgFill(vg);

            // Rail ticks every 2 V
            nvgBeginPath(vg);
            for (int k = -5; k <= 5; ++k)
            {
                float p = cx + k * step + 0.5f;
                nvgMoveTo(vg, p, 1.f);
                nvgLineTo(vg, p, 3.5f);
                nvgMoveTo(vg, p, A - 3.5f);
                nvgLineTo(vg, p, A - 1.f);
                float q = cy + k * step + 0.5f;
                nvgMoveTo(vg, 1.f, q);
                nvgLineTo(vg, 3.5f, q);
                nvgMoveTo(vg, A - 3.5f, q);
                nvgLineTo(vg, A - 1.f, q);
            }
            nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Grid every 2 V
            nvgBeginPath(vg);
            for (int k = -4; k <= 4; ++k)
            {
                if (k == 0)
                    continue;
                float p = cx + k * step + 0.5f;
                nvgMoveTo(vg, p, m);
                nvgLineTo(vg, p, A - m);
                float q = cy + k * step + 0.5f;
                nvgMoveTo(vg, m, q);
                nvgLineTo(vg, A - m, q);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Axes (a gate fires whenever a walker crosses one)
            nvgBeginPath(vg);
            nvgMoveTo(vg, cx + 0.5f, m);
            nvgLineTo(vg, cx + 0.5f, A - m);
            nvgMoveTo(vg, m, cy + 0.5f);
            nvgLineTo(vg, A - m, cy + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 38));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Origin marker (where every walk starts)
            nvgBeginPath(vg);
            nvgCircle(vg, cx + 0.5f, cy + 0.5f, 3.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgRestore(vg);
            Widget::draw(args);
        }

        // Layer-1 overlay: draws the walkers' wall flashes, axis-crossing
        // rings, trails, momentum ticks and flat balls, plus the header
        // (WALKERS/SPD) and footer (STEP/SMOOTH) readouts. Uses a demo scene
        // with six synthetic walkers when there is no module.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float A = RW_ARENA, cx = A / 2.f, cy = A / 2.f;
            const float S = RW_SCALE, m = RW_MARGIN;
            const float R = 4.4f; // ball radius in pixels

            // ---- gather state (live, or a sample scene in the module browser) ----
            int n;
            float px[RW_N], py[RW_N], hx[RW_N], hy[RW_N];
            float fw[RW_N], fxc[RW_N], fyc[RW_N], wp[RW_N], xp[RW_N], yp[RW_N];
            int ws[RW_N];
            static const int TN = RW_TRAIL_N + 1;
            float tx[RW_N][TN], ty[RW_N][TN];
            int tn[RW_N];
            float spdFactor, stepV, smooth;

            for (int i = 0; i < RW_N; ++i)
            {
                px[i] = py[i] = hx[i] = hy[i] = 0.f;
                fw[i] = fxc[i] = fyc[i] = 0.f;
                wp[i] = xp[i] = yp[i] = 0.f;
                ws[i] = 0;
                tn[i] = 0;
            }

            if (module)
            {
                n = rack::math::clamp(module->numBalls, 1, RW_N);
                for (int i = 0; i < n; ++i)
                {
                    px[i] = (float)module->x[i];
                    py[i] = (float)module->y[i];
                    hx[i] = (float)module->prevDeltaX[i];
                    hy[i] = (float)module->prevDeltaY[i];
                    fw[i] = module->flashWall[i];
                    fxc[i] = module->flashXc[i];
                    fyc[i] = module->flashYc[i];
                    ws[i] = rack::math::clamp(module->wallSide[i], 0, 3);
                    wp[i] = module->wallPos[i];
                    xp[i] = module->xcPos[i];
                    yp[i] = module->ycPos[i];

                    int cnt = std::min(std::max(module->trailCount[i], 0), RW_TRAIL_N);
                    int head = module->trailHead[i];
                    for (int k = 0; k < cnt; ++k)
                    {
                        int idx = ((head - cnt + k) % RW_TRAIL_N + RW_TRAIL_N) % RW_TRAIL_N;
                        tx[i][k] = cx + module->trailX[i][idx] * S;
                        ty[i][k] = cy - module->trailY[i][idx] * S;
                    }
                    // attach the path to the walker itself
                    tx[i][cnt] = cx + px[i] * S;
                    ty[i][cnt] = cy - py[i] * S;
                    tn[i] = cnt + 1;
                }
                spdFactor = (float)module->speedFactor;
                stepV = (float)module->stepSize;
                smooth = (float)module->smoothness;
            }
            else
            {
                n = 6;
                const int pts = 140;
                for (int i = 0; i < n; ++i)
                {
                    for (int k = 0; k < pts; ++k)
                    {
                        float t = k * 0.09f + i * 10.f;
                        float x = 6.f * std::sin(0.7f * t + 1.3f * i) + 3.f * std::sin(1.9f * t + 2.1f * i);
                        float y = 6.f * std::sin(0.6f * t + 0.7f * i) + 3.f * std::cos(1.7f * t + (float)i);
                        x = std::max(-9.5f, std::min(9.5f, x));
                        y = std::max(-9.5f, std::min(9.5f, y));
                        tx[i][k] = cx + x * S;
                        ty[i][k] = cy - y * S;
                    }
                    tn[i] = pts;
                    px[i] = (tx[i][pts - 1] - cx) / S;
                    py[i] = (cy - ty[i][pts - 1]) / S;
                    hx[i] = tx[i][pts - 1] - tx[i][pts - 2];
                    hy[i] = ty[i][pts - 2] - ty[i][pts - 1];
                }
                fxc[2] = 0.8f;
                xp[2] = 3.f;
                fyc[4] = 0.6f;
                yp[4] = -4.f;
                fw[1] = 0.9f;
                ws[1] = 1;
                wp[1] = 2.f;
                spdFactor = 2.1f;
                stepV = 0.1f;
                smooth = 0.5f;
            }

            // ---- arena content, clipped to the arena ----
            nvgSave(vg);
            nvgTranslate(vg, RW_ARENA_X, RW_ARENA_Y);
            nvgScissor(vg, 0, 0, A, A);
            nvgLineCap(vg, NVG_ROUND);

            // wall reflection flashes on the rail
            for (int i = 0; i < n; ++i)
            {
                if (fw[i] < 0.02f)
                    continue;
                float f = fw[i];
                float a = (ws[i] <= 1) ? (cy - wp[i] * S) : (cx + wp[i] * S);
                float lo = std::max(m, a - 16.f), hi = std::min(A - m, a + 16.f);
                float x0, y0, x1, y1;
                if (ws[i] == 0)
                {
                    x0 = x1 = m + 0.5f;
                    y0 = lo;
                    y1 = hi;
                }
                else if (ws[i] == 1)
                {
                    x0 = x1 = A - m - 0.5f;
                    y0 = lo;
                    y1 = hi;
                }
                else if (ws[i] == 2)
                {
                    y0 = y1 = m + 0.5f;
                    x0 = lo;
                    x1 = hi;
                }
                else
                {
                    y0 = y1 = A - m - 0.5f;
                    x0 = lo;
                    x1 = hi;
                }
                nvgBeginPath(vg);
                nvgMoveTo(vg, x0, y0);
                nvgLineTo(vg, x1, y1);
                nvgStrokeColor(vg, ag::withA(ballColor(i, 0.9f, 0.85f), f));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }

            // axis crossing flashes (gate triggers)
            for (int i = 0; i < n; ++i)
            {
                for (int a = 0; a < 2; ++a)
                {
                    float f = (a == 0) ? fxc[i] : fyc[i];
                    if (f < 0.02f)
                        continue;
                    float ax = (a == 0) ? cx : cx + yp[i] * S;
                    float ay = (a == 0) ? cy - xp[i] * S : cy;
                    NVGcolor c = ballColor(i, 0.9f, 0.75f);
                    nvgBeginPath(vg);
                    nvgCircle(vg, ax, ay, 2.f + (1.f - f) * 9.f);
                    nvgStrokeColor(vg, ag::withA(c, 0.9f * f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                    nvgBeginPath(vg);
                    nvgCircle(vg, ax, ay, 1.8f);
                    nvgFillColor(vg, ag::withA(c, f));
                    nvgFill(vg);
                }
            }

            // paths (hairline-width, fading with age)
            for (int i = 0; i < n; ++i)
                drawTrail(vg, tx[i], ty[i], tn[i], ballColor(i, 0.85f, 0.6f), 1.2f, 0.65f, 5);

            // momentum ticks and flat balls
            for (int i = 0; i < n; ++i)
            {
                float bx = cx + px[i] * S, by = cy - py[i] * S;
                float flash = std::max(fxc[i], fyc[i]);

                // flash ring when a gate fires
                if (flash > 0.02f)
                {
                    nvgBeginPath(vg);
                    nvgCircle(vg, bx, by, R + 3.f);
                    nvgStrokeColor(vg, ag::withA(ballColor(i, 0.9f, 0.8f), flash));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }

                // Momentum tick: the direction the walker is carrying, visible with smoothness
                float mag = std::sqrt(hx[i] * hx[i] + hy[i] * hy[i]);
                if (smooth > 0.05f && mag > 1e-9f)
                {
                    float dx = hx[i] / mag, dy = -hy[i] / mag;
                    float len = 3.f + 9.f * smooth;
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, bx + dx * (R + 1.5f), by + dy * (R + 1.5f));
                    nvgLineTo(vg, bx + dx * (R + 1.5f + len), by + dy * (R + 1.5f + len));
                    nvgStrokeColor(vg, ag::withA(ballColor(i, 0.9f, 0.85f), 0.2f + 0.5f * smooth));
                    nvgStrokeWidth(vg, 1.2f);
                    nvgStroke(vg);
                }

                nvgBeginPath(vg);
                nvgCircle(vg, bx, by, R);
                nvgFillColor(vg, ballColor(i, 0.85f, 0.58f));
                nvgFill(vg);
                nvgStrokeColor(vg, ballColor(i, 0.9f, 0.85f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            nvgRestore(vg);

            // ---- readouts: header and footer rows, outside the arena ----
            char buf[32];
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 220);
            NVGcolor cd = ag::dim();
            snprintf(buf, sizeof(buf), "WALKERS %02d", n);
            ag::text(vg, 8.f, 7.f, buf, 7.f, c1, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "SPD %.1fX", spdFactor);
            ag::text(vg, RW_DISP_W - 8.f, 7.f, buf, 7.f, c1, ag::ALIGN_R, 0.5f);
            snprintf(buf, sizeof(buf), "STEP %.2fV", stepV);
            ag::text(vg, 8.f, 225.f, buf, 7.f, cd, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "SMOOTH %3.0f%%", smooth * 100.f);
            ag::text(vg, RW_DISP_W - 8.f, 225.f, buf, 7.f, cd, ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Per-walker position bars (X and Y voltage, bipolar)
    // ------------------------------------------------------------
    // Live strip of 16 X/Y bar pairs: the static layer draws the empty tracks
    // and their walker numbers, the layer-1 overlay fills each pair with the
    // walker's current X (left) and Y (right) voltage, plus gate flashes on
    // the zero line. Uses sample data in the module browser.
    struct RWBarsWidget : TransparentWidget
    {
        RandomWalk *module = nullptr;

        RWBarsWidget(RandomWalk *m) : module(m)
        {
            box.pos = Vec(RW_BARS_X, RW_BARS_Y);
            box.size = Vec(RW_BARS_W, RW_BARS_H);
        }

        // Returns the width of one slot (the strip is split evenly across 16).
        static float slot() { return RW_BARS_W / (float)RW_N; }

        // Static layer: draws the 16 empty X/Y bar tracks, their zero lines,
        // and the walker-number row underneath.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float mid = RW_BAR_H / 2.f;
            for (int i = 0; i < RW_N; ++i)
            {
                float cx = slot() * (i + 0.5f);

                nvgBeginPath(vg);
                nvgRect(vg, cx - 9.f, 0.f, 8.f, RW_BAR_H);
                nvgRect(vg, cx + 1.f, 0.f, 8.f, RW_BAR_H);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 10));
                nvgFill(vg);

                // Zero line
                nvgBeginPath(vg);
                nvgRect(vg, cx - 9.f, mid, 18.f, 1.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                nvgFill(vg);

                // walker number in its own row under the bars
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", i + 1);
                ag::text(vg, cx, RW_BAR_H + 8.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            }
            Widget::draw(args);
        }

        // Layer-1 overlay: fills each active walker's X and Y bars from the
        // zero line toward their current voltage, with white tip caps and
        // white flashes when an axis gate fires.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float mid = RW_BAR_H / 2.f;
            const float amp = mid - 2.f;

            int n = module ? rack::math::clamp(module->numBalls, 1, RW_N) : 6;

            // Draws one bar (left or right of the zero line) given a signed
            // voltage and the bar's colour.
            auto bar = [&](float xl, float v, NVGcolor col)
            {
                float h = std::min(std::fabs(v) / 10.f, 1.f) * amp;
                if (h < 1.f)
                    h = 1.f;
                float top = (v >= 0.f) ? mid - h : mid + 1.f;
                nvgBeginPath(vg);
                nvgRect(vg, xl, top, 8.f, h);
                nvgFillColor(vg, col);
                nvgFill(vg);
                // Cap at the tip
                float capY = (v >= 0.f) ? top : top + h - 1.5f;
                nvgBeginPath(vg);
                nvgRect(vg, xl, capY, 8.f, 1.5f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 170));
                nvgFill(vg);
            };

            for (int i = 0; i < RW_N; ++i)
            {
                float cx = slot() * (i + 0.5f);
                if (i >= n)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, cx - 3.f, mid, 6.f, 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 25));
                    nvgFill(vg);
                    continue;
                }

                float vx, vy, fx = 0.f, fy = 0.f;
                if (module)
                {
                    vx = (float)module->x[i];
                    vy = (float)module->y[i];
                    fx = module->flashXc[i];
                    fy = module->flashYc[i];
                }
                else
                {
                    vx = 8.f * std::sin(1.3f * i + 0.4f);
                    vy = 7.f * std::cos(1.9f * i + 0.8f);
                }

                bar(cx - 9.f, vx, ag::withA(ballColor(i, 0.9f, 0.62f), 0.95f));
                bar(cx + 1.f, vy, ag::withA(ballColor(i, 0.9f, 0.80f), 0.85f));

                // Gate flashes on the zero line
                if (fx > 0.02f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, cx - 9.f, mid - 1.f, 8.f, 3.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(230.f * fx)));
                    nvgFill(vg);
                }
                if (fy > 0.02f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, cx + 1.f, mid - 1.f, 8.f, 3.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(230.f * fy)));
                    nvgFill(vg);
                }
            }

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
// Places the static panel, screws, the live arena display, the position-bar
// strip, the four knob/jack pairs on the left, and the six outputs plus
// Reset button/input on the right. Also builds the right-click menu for the
// V/Oct ranges and gate durations.
struct RandomWalkWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, arena display,
    // position bars, four left-hand knob/jack pairs, six polyphonic outputs
    // and the Reset button/jack.
    RandomWalkWidget(RandomWalk *module)
    {
        setModule(module);
        box.size = Vec(RW_PANEL_W, RW_PANEL_H);

        addChild(new RWPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new RWDisplayWidget(module));
        addChild(new RWBarsWidget(module));

        // ---- left block: knob + its CV jack on each row ----
        const float l0 = RW_LCOL[0], l1 = RW_LCOL[1];
        const int knobs[4] = {RandomWalk::SPEED_PARAM, RandomWalk::STEP_SIZE_PARAM,
                              RandomWalk::SMOOTHNESS_PARAM, RandomWalk::NUM_BALLS_PARAM};
        const int cvs[4] = {RandomWalk::SPEED_CV_INPUT, RandomWalk::STEP_CV_INPUT,
                            RandomWalk::SMOOTH_CV_INPUT, RandomWalk::BALLS_CV_INPUT};
        for (int r = 0; r < 4; ++r)
        {
            float y = rwCtlY(RW_ROW_TOP[r]);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, y), module, knobs[r]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, y), module, cvs[r]));
        }
        agmm::addKnobArcs(this, 2.f);
        // ---- right block (all outputs are polyphonic) ----
        const float r0 = RW_RCOL[0], r1 = RW_RCOL[1];
        const float y0 = rwCtlY(RW_ROW_TOP[0]), y1 = rwCtlY(RW_ROW_TOP[1]);
        const float y2 = rwCtlY(RW_ROW_TOP[2]), y3 = rwCtlY(RW_ROW_TOP[3]);
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, y0), module, RandomWalk::X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, y0), module, RandomWalk::Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, y1), module, RandomWalk::GATE_X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, y1), module, RandomWalk::GATE_Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, y2), module, RandomWalk::VOCT_X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, y2), module, RandomWalk::VOCT_Y_OUTPUT));

        addParam(createParamCentered<VCVButton>(Vec(r0, y3), module, RandomWalk::RESET_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(r1, y3), module, RandomWalk::RESET_CV_INPUT));
    }

    // Adds the right-click menu: "Voct X range" and "Voct Y range" submenus
    // (each with a checkmark on the active range, and a re-roll on selection)
    // and "Gate X duration" / "Gate Y duration" submenus.
    void appendContextMenu(ui::Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        RandomWalk *module = dynamic_cast<RandomWalk *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Voct X range", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                                             for (int i = 0; i < RandomWalk::NUM_RANGES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(rangeNames[i],
                                                                                  CHECKMARK(module->voctXRange == i),
                                                                                  [=]()
                                                                                  {
                                                                                      module->voctXRange = (RandomWalk::VoctRange)i;
                                                                                      for (int b = 0; b < RandomWalk::MAX_BALLS; ++b)
                                                                                          module->generateRandomVoctForBall(b, true);
                                                                                  }));
                                             }
                                         }));
        menu->addChild(createSubmenuItem("Voct Y range", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                                             for (int i = 0; i < RandomWalk::NUM_RANGES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(rangeNames[i],
                                                                                  CHECKMARK(module->voctYRange == i),
                                                                                  [=]()
                                                                                  {
                                                                                      module->voctYRange = (RandomWalk::VoctRange)i;
                                                                                      for (int b = 0; b < RandomWalk::MAX_BALLS; ++b)
                                                                                          module->generateRandomVoctForBall(b, false);
                                                                                  }));
                                             }
                                         }));
        menu->addChild(createSubmenuItem("Gate X duration", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                                             for (int i = 0; i < RandomWalk::NUM_GATE_DURS; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(gateNames[i],
                                                                                  CHECKMARK(module->gateXDuration == i),
                                                                                  [=]()
                                                                                  { module->gateXDuration = (RandomWalk::GateDuration)i; }));
                                             }
                                         }));
        menu->addChild(createSubmenuItem("Gate Y duration", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                                             for (int i = 0; i < RandomWalk::NUM_GATE_DURS; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(gateNames[i],
                                                                                  CHECKMARK(module->gateYDuration == i),
                                                                                  [=]()
                                                                                  { module->gateYDuration = (RandomWalk::GateDuration)i; }));
                                             }
                                         }));
    }

    
};

Model *modelRandomWalk = createModel<RandomWalk, RandomWalkWidget>("RandomWalk");