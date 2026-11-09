#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <random>
#include <limits>

// =============================================================================
// Bouncing Balls
// =============================================================================
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// BouncingBalls is a 28 HP "chaos generator": up to 16 balls bounce inside a
// 20 V x 20 V arena, and their X/Y positions, zero-crossings, ball-ball
// collisions and wall hits are all exposed as polyphonic outputs (one channel
// per ball). Each ball also has its own random V/oct value, re-randomised on
// every zero-crossing of that axis, so the module can drive up to 16 voices
// of a synth.
//
// File structure:
//   1. Layout constants (28 HP = 420x380 px).
//   2. BouncingBalls module: state, config, physics, gates, trail buffer,
//      JSON persistence.
//   3. Anonymous-namespace UI helpers and widgets:
//        - theme colours / font / text / plate / cell / caption helpers
//        - TBBCyanLight / TBBVioletLight light widgets
//        - ballColor(), drawTrail()
//        - BBPanelBackground: static artwork (title, blocks, plates, bezel)
//        - BBDisplayWidget:   arena (static rail/grid + live balls/trails)
//        - BBReadoutWidget:   text row under the arena
//        - BBBarsWidget:      per-ball speed bars
//   4. BouncingBallsWidget: places all controls, LEDs and the menu.
//   5. Model registration.
// -----------------------------------------------------------------------------

// =============================================================================
// Layout (28 HP = 420 x 380 px)
//
//   y   0-15   screws
//   y  15      title
//   y  26-254  arena bezel; arena itself 220 x 220 at (100, 30)
//   y  36      block captions: CONTROL (left) / OUTPUT (right)
//   y  46-302  left block: 5 rows x 2 cells (label above control)
//   y  46-248  right block: 4 rows x 2 cells (label above jack)
//   y 264      readouts (own row under the arena)
//   y 274-308  LED plate (collision / wall-hit rows)
//   y 314-360  ball speed bars
//   y 369      maker tag
//
// No text is ever drawn over the arena; every control is inside its plate.
// =============================================================================
static const int BB_N = 16;            // max balls
static const int BB_TRAIL_N = 36;      // trail points per ball
static const float BB_TRAIL_HZ = 50.f; // trail sample rate

static const float BB_PANEL_W = RACK_GRID_WIDTH * 28; // 420
static const float BB_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float BB_TITLE_Y = 15.f;
static const float BB_MAKER_Y = 369.f;

// Arena: 10 px per volt, 6 px rail around the playfield
static const float BB_DISP_W = 220.f;
static const float BB_DISP_H = 220.f;
static const float BB_DISP_X = 100.f;
static const float BB_DISP_Y = 30.f;
static const float BB_SCALE = 10.f;
static const float BB_MARGIN = 6.f;
static const float BB_READ_Y = 264.f; // panel coordinates

// Side blocks
static const float BB_CAPTION_Y = 36.f;
static const float BB_CELL_W = 38.f, BB_CELL_H = 48.f;
static const float BB_CELL_LABEL = 8.f; // offsets from the plate top
static const float BB_CELL_CTRL = 30.f;
static const float BB_LCOL[2] = {29.f, 70.f};
static const float BB_RCOL[2] = {351.f, 392.f};
static const float BB_ROW_TOP[5] = {46.f, 98.f, 150.f, 202.f, 254.f};

// LED plate under the arena
static const float BB_LED_PLATE_Y = 274.f, BB_LED_PLATE_H = 34.f;
static const float BB_LED_X = BB_DISP_X + 44.f;
static const float BB_LED_DX = 11.f;
static const float BB_LED_Y1 = 285.f;
static const float BB_LED_Y2 = 297.f;

// Ball speed bars
static const float BB_SPEED_PLATE_Y = 314.f, BB_SPEED_PLATE_H = 46.f;
static const float BB_BARS_X = 40.f;
static const float BB_BARS_Y = 330.f;
static const float BB_BARS_W = 340.f;
static const float BB_BARS_H = 28.f;
static const float BB_BAR_H = 18.f; // bar area height inside the widget

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct BouncingBalls : Module
{
    enum ParamId
    {
        SPEED_PARAM,
        ANGLE_PARAM,
        NUM_BALLS_PARAM,
        RESET_BUTTON,
        ANGLE_ATTENUATOR_PARAM,
        COLLISIONS_SWITCH_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        RESET_CV_INPUT,
        ANGLE_CV_INPUT,
        SPEED_CV_INPUT,
        BALLS_CV_INPUT,
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
        COLLISION_GATE_OUTPUT, // collision gate (poly)
        WALL_HIT_GATE_OUTPUT,  // wall hit gate (poly)
        OUTPUTS_LEN
    };
    enum LightId
    {
        COLLISION_LED1,
        COLLISION_LED2,
        COLLISION_LED3,
        COLLISION_LED4,
        COLLISION_LED5,
        COLLISION_LED6,
        COLLISION_LED7,
        COLLISION_LED8,
        COLLISION_LED9,
        COLLISION_LED10,
        COLLISION_LED11,
        COLLISION_LED12,
        COLLISION_LED13,
        COLLISION_LED14,
        COLLISION_LED15,
        COLLISION_LED16,
        WALL_LED1,
        WALL_LED2,
        WALL_LED3,
        WALL_LED4,
        WALL_LED5,
        WALL_LED6,
        WALL_LED7,
        WALL_LED8,
        WALL_LED9,
        WALL_LED10,
        WALL_LED11,
        WALL_LED12,
        WALL_LED13,
        WALL_LED14,
        WALL_LED15,
        WALL_LED16,
        LIGHTS_LEN
    };

    // Range of the random per-ball V/oct value, selectable from the menu.
    enum VoctRange
    {
        RANGE_0_1,
        RANGE_N1_1,
        RANGE_N2_2,
        RANGE_N5_5,
        RANGE_0_5,
        NUM_RANGES
    };
    // Duration of the X/Y zero-crossing gates, selectable from the menu.
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
    static constexpr double BALL_RADIUS = 0.4;
    static constexpr double FLASH_DURATION_SEC = 0.02; // 20ms gate flash

    double posX[MAX_BALLS] = {};
    double posY[MAX_BALLS] = {};
    double velX[MAX_BALLS] = {};
    double velY[MAX_BALLS] = {};
    double prevX[MAX_BALLS] = {};
    double prevY[MAX_BALLS] = {};
    int gateXTimer[MAX_BALLS] = {};
    int gateYTimer[MAX_BALLS] = {};
    double voctX[MAX_BALLS] = {};
    double voctY[MAX_BALLS] = {};

    // Timers for collision / wall gates
    int collisionTimer[MAX_BALLS] = {};
    int wallHitTimer[MAX_BALLS] = {};

    // Current parameter values
    double speedBase = 5.0;
    double angleBase = 0.0; // radians
    int numBalls = 1;

    // Saved previous values to detect changes
    double prevSpeed = 5.0;
    double prevAngle = 0.0;

    VoctRange voctRange = RANGE_N1_1;
    GateDuration gateDuration = GATE_5MS;

    std::mt19937 rng;
    dsp::SchmittTrigger resetTrigger;
    bool lastResetState = false;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float trailX[MAX_BALLS][BB_TRAIL_N] = {};
    float trailY[MAX_BALLS][BB_TRAIL_N] = {};
    int trailHead = 0;
    int trailCount = 0;
    float trailTimer = 0.f;
    float flashCol[MAX_BALLS] = {};  // ball-ball collision
    float flashWall[MAX_BALLS] = {}; // wall hit
    float flashXc[MAX_BALLS] = {};   // X zero crossing (vertical axis)
    float flashYc[MAX_BALLS] = {};   // Y zero crossing (horizontal axis)
    int wallSide[MAX_BALLS] = {};    // 0 left, 1 right, 2 top, 3 bottom
    float wallPos[MAX_BALLS] = {};   // position along that wall (V)
    float xcPos[MAX_BALLS] = {};     // Y value where X crossed zero
    float ycPos[MAX_BALLS] = {};     // X value where Y crossed zero

    // Constructor: configures params, inputs, outputs, lights, then seeds the
    // RNG and resets the ball state to the initial scene.
    BouncingBalls()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(SPEED_PARAM, 0.f, 20.f, 5.f, "Base speed", " V/s");
        configParam(ANGLE_PARAM, -180.f, 180.f, 0.f, "Base angle", " deg");
        configParam(NUM_BALLS_PARAM, 1.f, 16.f, 1.f, "Number of balls");
        getParamQuantity(NUM_BALLS_PARAM)->snapEnabled = true;
        configButton(RESET_BUTTON, "Reset");
        configParam(ANGLE_ATTENUATOR_PARAM, -1.f, 1.f, 0.f, "Angle CV amount", "%", 0.f, 100.f);
        configSwitch(COLLISIONS_SWITCH_PARAM, 0.f, 1.f, 0.f, "Ball collisions", {"Off", "On"});

        configInput(RESET_CV_INPUT, "Reset gate");
        configInput(ANGLE_CV_INPUT, "Angle CV");
        configInput(SPEED_CV_INPUT, "Speed CV (0 to 10 V adds 0 to 20 V/s)");
        configInput(BALLS_CV_INPUT, "Number of balls CV (10 V adds 15 balls)");

        configOutput(X_OUTPUT, "X (polyphonic)");
        configOutput(Y_OUTPUT, "Y (polyphonic)");
        configOutput(GATE_X_OUTPUT, "Gate X (polyphonic)");
        configOutput(GATE_Y_OUTPUT, "Gate Y (polyphonic)");
        configOutput(VOCT_X_OUTPUT, "V/oct X (polyphonic)");
        configOutput(VOCT_Y_OUTPUT, "V/oct Y (polyphonic)");
        configOutput(COLLISION_GATE_OUTPUT, "Collision gate (polyphonic)");
        configOutput(WALL_HIT_GATE_OUTPUT, "Wall hit gate (polyphonic)");

        for (int i = 0; i < 16; ++i)
        {
            configLight(COLLISION_LED1 + i, string::f("Ball %d collision", i + 1));
            configLight(WALL_LED1 + i, string::f("Ball %d wall hit", i + 1));
        }

        std::random_device rd;
        rng.seed(rd());
        onReset();
    }

    // Resets all ball positions/velocities to a random scene, clears timers
    // and trails, and re-randomises the V/oct values. Called on construction,
    // on the Reset button/gate, and from the context menu if wired up.
    void onReset() override
    {
        updateParameters();
        std::uniform_real_distribution<double> posDist(-8.0, 8.0);
        std::uniform_real_distribution<double> angleOff(-M_PI / 4, M_PI / 4);
        std::uniform_real_distribution<double> speedOff(0.8, 1.2);
        for (int i = 0; i < numBalls; ++i)
        {
            posX[i] = posDist(rng);
            posY[i] = posDist(rng);
            double angleRad = angleBase + angleOff(rng);
            double speed = speedBase * speedOff(rng);
            velX[i] = speed * cos(angleRad);
            velY[i] = speed * sin(angleRad);
            prevX[i] = posX[i];
            prevY[i] = posY[i];
            gateXTimer[i] = 0;
            gateYTimer[i] = 0;
            collisionTimer[i] = 0;
            wallHitTimer[i] = 0;
            generateRandomVoct(i, true);
            generateRandomVoct(i, false);
        }
        prevSpeed = speedBase;
        prevAngle = angleBase;

        trailHead = 0;
        trailCount = 0;
        trailTimer = 0.f;
        for (int i = 0; i < MAX_BALLS; ++i)
        {
            flashCol[i] = flashWall[i] = flashXc[i] = flashYc[i] = 0.f;
        }
    }

    // Generates a new random V/oct value for one ball and one axis (X or Y),
    // scaled according to the currently selected V/oct range.
    void generateRandomVoct(int ball, bool forX)
    {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        double &target = forX ? voctX[ball] : voctY[ball];
        switch (voctRange)
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

    // Returns the selected zero-crossing gate length in seconds.
    double getGateDurationSeconds()
    {
        switch (gateDuration)
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

    // Reads the speed / angle / ball-count controls and CVs, updates the
    // cached values, creates any newly added balls, and smoothly reorients
    // and rescales existing velocities when speed or angle changes.
    void updateParameters()
    {
        float angleCV = inputs[ANGLE_CV_INPUT].getVoltage();
        float attenuverter = params[ANGLE_ATTENUATOR_PARAM].getValue();
        float angleOffsetDeg = attenuverter * angleCV * 36.0f;
        float effectiveAngleDeg = params[ANGLE_PARAM].getValue() + angleOffsetDeg;
        double newAngleBase = effectiveAngleDeg * M_PI / 180.0;

        float spd = params[SPEED_PARAM].getValue();
        if (inputs[SPEED_CV_INPUT].isConnected())
            spd = rack::math::clamp(spd + inputs[SPEED_CV_INPUT].getVoltage() * 2.f, 0.f, 20.f);
        speedBase = spd;

        float nb = params[NUM_BALLS_PARAM].getValue();
        if (inputs[BALLS_CV_INPUT].isConnected())
            nb += inputs[BALLS_CV_INPUT].getVoltage() * 1.5f;
        int newNum = rack::math::clamp((int)std::round(nb), 1, 16);

        if (newNum != numBalls)
        {
            std::uniform_real_distribution<double> posDist(-8.0, 8.0);
            std::uniform_real_distribution<double> angleOff(-M_PI / 4, M_PI / 4);
            std::uniform_real_distribution<double> speedOff(0.8, 1.2);
            for (int i = numBalls; i < newNum; ++i)
            {
                posX[i] = posDist(rng);
                posY[i] = posDist(rng);
                double angleRad = newAngleBase + angleOff(rng);
                double speed = speedBase * speedOff(rng);
                velX[i] = speed * cos(angleRad);
                velY[i] = speed * sin(angleRad);
                prevX[i] = posX[i];
                prevY[i] = posY[i];
                gateXTimer[i] = 0;
                gateYTimer[i] = 0;
                collisionTimer[i] = 0;
                wallHitTimer[i] = 0;
                generateRandomVoct(i, true);
                generateRandomVoct(i, false);
            }
            numBalls = newNum;
            trailCount = 0; // restart trails so new balls don't draw a stale streak
        }

        if (speedBase != prevSpeed || newAngleBase != prevAngle)
        {
            for (int i = 0; i < numBalls; ++i)
            {
                double oldSpeed = std::sqrt(velX[i] * velX[i] + velY[i] * velY[i]);
                double oldAngle = std::atan2(velY[i], velX[i]);
                if (oldSpeed > 0.01)
                {
                    double delta = oldAngle - prevAngle;
                    double newAngle = newAngleBase + delta;
                    double newSpeed = (prevSpeed > 1e-6) ? speedBase * (oldSpeed / prevSpeed) : speedBase;
                    velX[i] = newSpeed * cos(newAngle);
                    velY[i] = newSpeed * sin(newAngle);
                }
                else
                {
                    velX[i] = speedBase * cos(newAngleBase);
                    velY[i] = speedBase * sin(newAngleBase);
                }
            }
            prevSpeed = speedBase;
            prevAngle = newAngleBase;
        }
        angleBase = newAngleBase;
    }

    // Resolve collisions and record which balls collided
    // Runs two relaxation passes over all ball pairs, pushing overlapping
    // balls apart and exchanging the normal velocity component. If `record`
    // is true, `collided[i]` is set for each ball that actually bounced.
    void resolveCollisionsAndRecord(bool record, bool collided[MAX_BALLS])
    {
        for (int iter = 0; iter < 2; ++iter)
        {
            for (int i = 0; i < numBalls; ++i)
            {
                for (int j = i + 1; j < numBalls; ++j)
                {
                    double dx = posX[i] - posX[j];
                    double dy = posY[i] - posY[j];
                    double dist = std::sqrt(dx * dx + dy * dy);
                    double minDist = 2.0 * BALL_RADIUS;
                    if (dist < minDist)
                    {
                        double nx = dx / dist;
                        double ny = dy / dist;
                        double correction = (minDist - dist) * 0.5;
                        posX[i] += nx * correction;
                        posY[i] += ny * correction;
                        posX[j] -= nx * correction;
                        posY[j] -= ny * correction;
                        double vrelx = velX[i] - velX[j];
                        double vrely = velY[i] - velY[j];
                        double dot = vrelx * nx + vrely * ny;
                        if (dot < 0.0)
                        {
                            double imp = dot;
                            velX[i] -= imp * nx;
                            velY[i] -= imp * ny;
                            velX[j] += imp * nx;
                            velY[j] += imp * ny;
                            if (record)
                            {
                                collided[i] = true;
                                collided[j] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    // Main audio callback: applies parameter/CV changes, handles reset,
    // advances the balls (walls, zero-crossing gates, collisions), updates
    // the LEDs, writes the eight polyphonic outputs and samples the trails.
    void process(const ProcessArgs &args) override
    {
        updateParameters();

        bool resetNow = (params[RESET_BUTTON].getValue() > 0.5f) ||
                        (inputs[RESET_CV_INPUT].getVoltage() > 1.0f);
        if (resetNow && !lastResetState)
            onReset();
        lastResetState = resetNow;

        bool collisionsEnabled = params[COLLISIONS_SWITCH_PARAM].getValue() > 0.5f;
        double dt = args.sampleTime;
        int flashSamples = static_cast<int>(args.sampleRate * FLASH_DURATION_SEC);

        // Decay the visual flashes (LEDs and display)
        for (int i = 0; i < MAX_BALLS; ++i)
        {
            flashCol[i] = std::max(0.f, flashCol[i] - 6.f * args.sampleTime);
            flashWall[i] = std::max(0.f, flashWall[i] - 5.f * args.sampleTime);
            flashXc[i] = std::max(0.f, flashXc[i] - 4.f * args.sampleTime);
            flashYc[i] = std::max(0.f, flashYc[i] - 4.f * args.sampleTime);
        }

        // Move and handle wall collisions, record wall hits
        for (int i = 0; i < numBalls; ++i)
        {
            prevX[i] = posX[i];
            prevY[i] = posY[i];

            posX[i] += velX[i] * dt;
            posY[i] += velY[i] * dt;

            int side = -1;
            float wpos = 0.f;
            if (posX[i] > 10.0)
            {
                posX[i] = 20.0 - posX[i];
                velX[i] = -velX[i];
                side = 1;
                wpos = (float)posY[i];
            }
            if (posX[i] < -10.0)
            {
                posX[i] = -20.0 - posX[i];
                velX[i] = -velX[i];
                side = 0;
                wpos = (float)posY[i];
            }
            if (posY[i] > 10.0)
            {
                posY[i] = 20.0 - posY[i];
                velY[i] = -velY[i];
                side = 2;
                wpos = (float)posX[i];
            }
            if (posY[i] < -10.0)
            {
                posY[i] = -20.0 - posY[i];
                velY[i] = -velY[i];
                side = 3;
                wpos = (float)posX[i];
            }
            if (side >= 0)
            {
                wallHitTimer[i] = flashSamples;
                flashWall[i] = 1.f;
                wallSide[i] = side;
                wallPos[i] = wpos;
            }

            // Zero crossing for X
            if ((prevX[i] <= 0 && posX[i] > 0) || (prevX[i] >= 0 && posX[i] < 0))
            {
                generateRandomVoct(i, true);
                double dur = getGateDurationSeconds();
                gateXTimer[i] = static_cast<int>(args.sampleRate * dur);
                flashXc[i] = 1.f;
                xcPos[i] = (float)posY[i];
            }
            // Zero crossing for Y
            if ((prevY[i] <= 0 && posY[i] > 0) || (prevY[i] >= 0 && posY[i] < 0))
            {
                generateRandomVoct(i, false);
                double dur = getGateDurationSeconds();
                gateYTimer[i] = static_cast<int>(args.sampleRate * dur);
                flashYc[i] = 1.f;
                ycPos[i] = (float)posX[i];
            }

            if (gateXTimer[i] > 0)
                gateXTimer[i]--;
            if (gateYTimer[i] > 0)
                gateYTimer[i]--;
        }

        // Handle ball-ball collisions
        bool collided[MAX_BALLS] = {};
        if (collisionsEnabled)
        {
            resolveCollisionsAndRecord(true, collided);
            for (int i = 0; i < numBalls; ++i)
            {
                if (collided[i])
                {
                    collisionTimer[i] = flashSamples;
                    flashCol[i] = 1.f;
                }
            }
        }

        // Decrement timers for the extra gates
        for (int i = 0; i < numBalls; ++i)
        {
            if (collisionTimer[i] > 0)
                collisionTimer[i]--;
            if (wallHitTimer[i] > 0)
                wallHitTimer[i]--;
        }

        // LEDs (they fade out, so the short gates are still visible)
        for (int i = 0; i < MAX_BALLS; ++i)
        {
            bool on = (i < numBalls);
            lights[COLLISION_LED1 + i].setBrightness(on ? flashCol[i] : 0.f);
            lights[WALL_LED1 + i].setBrightness(on ? flashWall[i] : 0.f);
        }

        // Set polyphonic outputs
        int channels = numBalls;
        outputs[X_OUTPUT].setChannels(channels);
        outputs[Y_OUTPUT].setChannels(channels);
        outputs[GATE_X_OUTPUT].setChannels(channels);
        outputs[GATE_Y_OUTPUT].setChannels(channels);
        outputs[VOCT_X_OUTPUT].setChannels(channels);
        outputs[VOCT_Y_OUTPUT].setChannels(channels);
        outputs[COLLISION_GATE_OUTPUT].setChannels(channels);
        outputs[WALL_HIT_GATE_OUTPUT].setChannels(channels);

        for (int i = 0; i < channels; ++i)
        {
            outputs[X_OUTPUT].setVoltage((float)posX[i], i);
            outputs[Y_OUTPUT].setVoltage((float)posY[i], i);
            outputs[GATE_X_OUTPUT].setVoltage(gateXTimer[i] > 0 ? 10.0f : 0.0f, i);
            outputs[GATE_Y_OUTPUT].setVoltage(gateYTimer[i] > 0 ? 10.0f : 0.0f, i);
            outputs[VOCT_X_OUTPUT].setVoltage((float)voctX[i], i);
            outputs[VOCT_Y_OUTPUT].setVoltage((float)voctY[i], i);
            outputs[COLLISION_GATE_OUTPUT].setVoltage(collisionTimer[i] > 0 ? 10.0f : 0.0f, i);
            outputs[WALL_HIT_GATE_OUTPUT].setVoltage(wallHitTimer[i] > 0 ? 10.0f : 0.0f, i);
        }

        // ---- trail data for the display ----
        // Samples all ball positions at BB_TRAIL_HZ into a ring buffer that
        // the display widget reads to draw the fading motion trails.
        const float trailStep = 1.f / BB_TRAIL_HZ;
        trailTimer += args.sampleTime;
        if (trailTimer >= trailStep)
        {
            trailTimer -= trailStep;
            if (trailTimer > trailStep)
                trailTimer = 0.f;
            for (int i = 0; i < MAX_BALLS; ++i)
            {
                trailX[i][trailHead] = (float)posX[i];
                trailY[i][trailHead] = (float)posY[i];
            }
            trailHead = (trailHead + 1) % BB_TRAIL_N;
            if (trailCount < BB_TRAIL_N)
                trailCount++;
        }
    }

    // Serialises the V/oct range and gate duration menu choices.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "voctRange", json_integer(voctRange));
        json_object_set_new(root, "gateDuration", json_integer(gateDuration));
        return root;
    }

    // Restores the V/oct range and gate duration from a saved patch,
    // clamping the values to valid enum ranges.
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

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{

    // ------------------------------------------------------------
    // Theme (same palette as the other AGMM modules)
    // ------------------------------------------------------------
    // Panel background gradient start colour (top).
    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    // Panel background gradient end colour (bottom).
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    // Default body text colour.
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    // Muted colour for secondary labels.
    NVGcolor colDim() { return nvgRGB(0x8a, 0x95, 0xa4); }
    // Primary accent colour (cyan).
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    // Secondary accent colour (violet) for the output block and wall LEDs.
    NVGcolor colViolet() { return nvgRGB(0xb0, 0x5a, 0xff); }

    // Text alignment presets.
    const int ALIGN_L = NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE;
    const int ALIGN_C = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE;
    const int ALIGN_R = NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE;

    // Loads the AGMM monospace font and makes it the current NanoVG font
    // face. Returns false if there is no window or the asset is missing.
    bool setFont(NVGcontext *vg)
    {
        if (!APP || !APP->window)
            return false;
        std::shared_ptr<window::Font> font =
            APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (!font)
            return false;
        nvgFontFaceId(vg, font->handle);
        return true;
    }

    // Draws a string at (x, y) with the given size, colour, alignment and
    // letter spacing.
    void drawText(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
                  int align = ALIGN_C, float spacing = 0.5f)
    {
        if (!setFont(vg))
            return;
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Returns a copy of `c` with its alpha channel replaced by `a`.
    NVGcolor withAlpha(NVGcolor c, float a)
    {
        c.a = a;
        return c;
    }

    // Cyan / violet LEDs (Rack has no built-in ones)
    // Cyan LED widget, templated on the base light class.
    template <typename TBase = GrayModuleLightWidget>
    struct TBBCyanLight : TBase
    {
        // Registers the cyan base colour with the underlying light widget.
        TBBCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
    };
    using BBCyanLight = TBBCyanLight<>;

    // Violet LED widget, templated on the base light class.
    template <typename TBase = GrayModuleLightWidget>
    struct TBBVioletLight : TBase
    {
        // Registers the violet base colour with the underlying light widget.
        TBBVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
    };
    using BBVioletLight = TBBVioletLight<>;

    // Ball colours run from cyan through violet to magenta, so all 16 stay distinct.
    // The colour depends on the ball index, not on the ball count.
    // Returns the HSL colour for ball `i`, using the given saturation and
    // lightness; the hue sweeps from 0.5 (cyan) to ~0.95 (magenta).
    NVGcolor ballColor(int i, float sat, float light)
    {
        float h = 0.5f + 0.45f * (float)i / 15.f;
        return nvgHSL(h, sat, light);
    }

    // Fading motion trail (older = thinner and fainter)
    // Draws a polyline through n points, split into short segments so the
    // stroke width and alpha can taper off toward the oldest points.
    void drawTrail(NVGcontext *vg, const float *xs, const float *ys, int n, NVGcolor col, float width, float alpha)
    {
        if (n < 2)
            return;
        nvgLineCap(vg, NVG_ROUND);
        nvgLineJoin(vg, NVG_ROUND);
        const int K = 3;
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

    // Control plate; tint = outputs
    void plate(NVGcontext *vg, float cx, float top, float w, float h, bool tint = false)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - w / 2.f, top, w, h, 5.f);
        nvgFillColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 22) : nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 60) : nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Side-block cell: label above, control below (the control is a widget)
    void cell(NVGcontext *vg, float cx, float top, const char *txt, bool tint = false)
    {
        plate(vg, cx, top, BB_CELL_W, BB_CELL_H, tint);
        drawText(vg, cx, top + BB_CELL_LABEL, txt, 7.f, colDim(), ALIGN_C, 0.3f);
    }

    // Draws a centred, letter-spaced block caption (CONTROL / OUTPUT).
    void caption(NVGcontext *vg, float cx, float y, const char *txt, NVGcolor col)
    {
        drawText(vg, cx, y, txt, 8.f, col, ALIGN_C, 2.f);
    }

    // ------------------------------------------------------------
    // Panel background (static artwork only)
    // ------------------------------------------------------------
    struct BBPanelBackground : TransparentWidget
    {
        BBPanelBackground() { box.size = Vec(BB_PANEL_W, BB_PANEL_H); }

        // Draws the opaque background, title, maker tag, arena bezel, block
        // captions, the left and right cell plates, the LED plate labels and
        // the speed-bars plate.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, BB_PANEL_W, BB_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, BB_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            drawText(vg, BB_PANEL_W / 2.f, BB_TITLE_Y, "BOUNCING BALLS", 12.f, colText(), ALIGN_C, 3.f);
            drawText(vg, BB_PANEL_W / 2.f, BB_MAKER_Y, "AGMM", 9.f, colDim(), ALIGN_C, 3.f);

            // Arena bezel
            nvgBeginPath(vg);
            nvgRoundedRect(vg, BB_DISP_X - 4.f, BB_DISP_Y - 4.f,
                           BB_DISP_W + 8.f, BB_DISP_H + 8.f, 5.f);
            nvgFillColor(vg, nvgRGB(0x0b, 0x0c, 0x0e));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Block captions
            caption(vg, (BB_LCOL[0] + BB_LCOL[1]) / 2.f, BB_CAPTION_Y, "CONTROL", colAccent());
            caption(vg, (BB_RCOL[0] + BB_RCOL[1]) / 2.f, BB_CAPTION_Y, "OUTPUT", colViolet());

            // ---- left block (label above each control) ----
            const float l0 = BB_LCOL[0], l1 = BB_LCOL[1];
            cell(vg, l0, BB_ROW_TOP[0], "SPEED");
            cell(vg, l1, BB_ROW_TOP[0], "SPD CV");
            cell(vg, l0, BB_ROW_TOP[1], "ANGLE");
            cell(vg, l1, BB_ROW_TOP[1], "ANG ATT");
            cell(vg, l0, BB_ROW_TOP[2], "ANG CV");
            cell(vg, l1, BB_ROW_TOP[2], "COLLIDE");
            cell(vg, l0, BB_ROW_TOP[3], "BALLS");
            cell(vg, l1, BB_ROW_TOP[3], "BAL CV");
            cell(vg, l0, BB_ROW_TOP[4], "RESET");
            cell(vg, l1, BB_ROW_TOP[4], "RST IN");

            // ---- right block (outputs, tinted) ----
            const float r0 = BB_RCOL[0], r1 = BB_RCOL[1];
            cell(vg, r0, BB_ROW_TOP[0], "X", true);
            cell(vg, r1, BB_ROW_TOP[0], "Y", true);
            cell(vg, r0, BB_ROW_TOP[1], "GATE X", true);
            cell(vg, r1, BB_ROW_TOP[1], "GATE Y", true);
            cell(vg, r0, BB_ROW_TOP[2], "V/OCT X", true);
            cell(vg, r1, BB_ROW_TOP[2], "V/OCT Y", true);
            cell(vg, r0, BB_ROW_TOP[3], "COLL", true);
            cell(vg, r1, BB_ROW_TOP[3], "WALL", true);

            // ---- LED plate under the arena ----
            plate(vg, BB_DISP_X + BB_DISP_W / 2.f, BB_LED_PLATE_Y, BB_DISP_W, BB_LED_PLATE_H);
            drawText(vg, BB_DISP_X + 7.f, BB_LED_Y1, "COLL", 7.f, colAccent(), ALIGN_L, 0.5f);
            drawText(vg, BB_DISP_X + 7.f, BB_LED_Y2, "WALL", 7.f, colViolet(), ALIGN_L, 0.5f);

            // ---- speed bars plate ----
            plate(vg, BB_PANEL_W / 2.f, BB_SPEED_PLATE_Y, 352.f, BB_SPEED_PLATE_H);
            drawText(vg, 44.f, BB_SPEED_PLATE_Y + 8.f, "BALL SPEED", 7.f, colAccent(), ALIGN_L, 1.5f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Arena display (no text over the playfield)
    // ------------------------------------------------------------
    struct BBDisplayWidget : TransparentWidget
    {
        BouncingBalls *module = nullptr;

        BBDisplayWidget(BouncingBalls *m) : module(m)
        {
            box.pos = Vec(BB_DISP_X, BB_DISP_Y);
            box.size = Vec(BB_DISP_W, BB_DISP_H);
        }

        // Static layer: rail, playfield, grid, axes
        // Draws the arena's static artwork: raised rail, dark playfield,
        // 2 V grid lines and the two brighter centre axes.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = BB_DISP_W, H = BB_DISP_H, cx = W / 2.f, cy = H / 2.f, m = BB_MARGIN;

            // Rail
            nvgBeginPath(vg);
            nvgRoundedRect(vg, 0, 0, W, H, 3.f);
            nvgFillColor(vg, nvgRGB(0x1c, 0x20, 0x26));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Playfield
            nvgBeginPath(vg);
            nvgRect(vg, m, m, W - 2.f * m, H - 2.f * m);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 200));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Grid every 2 V
            nvgBeginPath(vg);
            for (int k = -5; k <= 5; ++k)
            {
                if (k == 0)
                    continue;
                float p = cx + k * 20.f + 0.5f;
                nvgMoveTo(vg, p, m);
                nvgLineTo(vg, p, H - m);
                float q = cy + k * 20.f + 0.5f;
                nvgMoveTo(vg, m, q);
                nvgLineTo(vg, W - m, q);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Axes (a gate fires whenever a ball crosses one)
            nvgBeginPath(vg);
            nvgMoveTo(vg, cx + 0.5f, m);
            nvgLineTo(vg, cx + 0.5f, H - m);
            nvgMoveTo(vg, m, cy + 0.5f);
            nvgLineTo(vg, W - m, cy + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 38));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Layer-1 hook: draws the live balls/trails/flashes on top of the
        // static arena, then forwards to the base widget.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer == 1)
                drawLive(args.vg);
            Widget::drawLayer(args, layer);
        }

        // Draws the live arena contents: wall-hit flashes on the rail, axis
        // crossing rings, fading trails, the balls with heading ticks, and
        // collision rings. Uses a sample scene when there is no module.
        void drawLive(NVGcontext *vg)
        {
            const float W = BB_DISP_W, H = BB_DISP_H, cx = W / 2.f, cy = H / 2.f;
            const float S = BB_SCALE, m = BB_MARGIN;
            const float R = 4.f; // ball radius in pixels

            // ---- gather state (live, or a sample scene in the module browser) ----
            int n;
            float px[BB_N], py[BB_N], vx[BB_N], vy[BB_N];
            float fw[BB_N], fc[BB_N], fxc[BB_N], fyc[BB_N], wp[BB_N], xp[BB_N], yp[BB_N];
            int ws[BB_N];
            float tx[BB_N][BB_TRAIL_N], ty[BB_N][BB_TRAIL_N];
            int tn = 0;

            for (int i = 0; i < BB_N; ++i)
            {
                px[i] = py[i] = vx[i] = vy[i] = 0.f;
                fw[i] = fc[i] = fxc[i] = fyc[i] = 0.f;
                wp[i] = xp[i] = yp[i] = 0.f;
                ws[i] = 0;
            }

            if (module)
            {
                n = module->numBalls;
                for (int i = 0; i < n; ++i)
                {
                    px[i] = (float)module->posX[i];
                    py[i] = (float)module->posY[i];
                    vx[i] = (float)module->velX[i];
                    vy[i] = (float)module->velY[i];
                    fw[i] = module->flashWall[i];
                    fc[i] = module->flashCol[i];
                    fxc[i] = module->flashXc[i];
                    fyc[i] = module->flashYc[i];
                    ws[i] = module->wallSide[i];
                    wp[i] = module->wallPos[i];
                    xp[i] = module->xcPos[i];
                    yp[i] = module->ycPos[i];
                }
                tn = std::min(std::max(module->trailCount, 0), BB_TRAIL_N);
                int head = module->trailHead;
                for (int i = 0; i < n; ++i)
                {
                    for (int k = 0; k < tn; ++k)
                    {
                        int idx = ((head - tn + k) % BB_TRAIL_N + BB_TRAIL_N) % BB_TRAIL_N;
                        tx[i][k] = cx + module->trailX[i][idx] * S;
                        ty[i][k] = cy - module->trailY[i][idx] * S;
                    }
                }
            }
            else
            {
                n = 8;
                for (int i = 0; i < n; ++i)
                {
                    px[i] = 8.f * std::sin(1.7f * i + 0.3f);
                    py[i] = 8.f * std::cos(2.3f * i + 0.9f);
                    float va = 0.9f * i + 0.5f;
                    float sp = 8.f + (float)(i % 4) * 3.f;
                    vx[i] = sp * std::cos(va);
                    vy[i] = sp * std::sin(va);
                }
                tn = BB_TRAIL_N;
                for (int i = 0; i < n; ++i)
                {
                    for (int k = 0; k < tn; ++k)
                    {
                        float t = (float)(k - (tn - 1)) * 0.012f;
                        float x = std::max(-10.f, std::min(10.f, px[i] + vx[i] * t));
                        float y = std::max(-10.f, std::min(10.f, py[i] + vy[i] * t));
                        tx[i][k] = cx + x * S;
                        ty[i][k] = cy - y * S;
                    }
                }
                fc[2] = 0.7f;
                fw[5] = 0.9f;
                ws[5] = 1;
                wp[5] = 2.f;
                fxc[3] = 0.8f;
                xp[3] = 3.f;
                fyc[6] = 0.6f;
                yp[6] = -4.f;
            }

            // ---- wall hit flashes on the rail ----
            nvgLineCap(vg, NVG_ROUND);
            for (int i = 0; i < n; ++i)
            {
                if (fw[i] < 0.02f)
                    continue;
                float f = fw[i];
                float a = (ws[i] <= 1) ? (cy - wp[i] * S) : (cx + wp[i] * S);
                float lo = std::max(m, a - 16.f), hi = std::min(H - m, a + 16.f);
                float x0, y0, x1, y1;
                if (ws[i] == 0)
                {
                    x0 = x1 = m + 0.5f;
                    y0 = lo;
                    y1 = hi;
                }
                else if (ws[i] == 1)
                {
                    x0 = x1 = W - m - 0.5f;
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
                    y0 = y1 = H - m - 0.5f;
                    x0 = lo;
                    x1 = hi;
                }
                nvgBeginPath(vg);
                nvgMoveTo(vg, x0, y0);
                nvgLineTo(vg, x1, y1);
                nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.8f), f));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }

            // ---- axis crossing flashes (gate triggers) ----
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
                    nvgStrokeColor(vg, withAlpha(c, 0.9f * f));
                    nvgStrokeWidth(vg, 1.2f);
                    nvgStroke(vg);
                }
            }

            // ---- trails ----
            for (int i = 0; i < n; ++i)
                drawTrail(vg, tx[i], ty[i], tn, ballColor(i, 0.85f, 0.6f), 1.5f, 0.5f);

            // ---- balls (flat) with a heading tick ----
            for (int i = 0; i < n; ++i)
            {
                float bx = cx + px[i] * S, by = cy - py[i] * S;
                float sp = std::sqrt(vx[i] * vx[i] + vy[i] * vy[i]);

                if (sp > 0.05f)
                {
                    float dx = vx[i] / sp, dy = -vy[i] / sp;
                    float len = 3.f + std::min(sp, 24.f) * 0.28f;
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, bx + dx * (R + 1.5f), by + dy * (R + 1.5f));
                    nvgLineTo(vg, bx + dx * (R + 1.5f + len), by + dy * (R + 1.5f + len));
                    nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.85f), 0.6f));
                    nvgStrokeWidth(vg, 1.2f);
                    nvgLineCap(vg, NVG_ROUND);
                    nvgStroke(vg);
                }

                nvgBeginPath(vg);
                nvgCircle(vg, bx, by, R);
                nvgFillColor(vg, ballColor(i, 0.85f, 0.58f));
                nvgFill(vg);
                nvgStrokeColor(vg, ballColor(i, 0.8f, 0.2f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // ---- collision rings ----
            for (int i = 0; i < n; ++i)
            {
                if (fc[i] < 0.02f)
                    continue;
                float bx = cx + px[i] * S, by = cy - py[i] * S;
                nvgBeginPath(vg);
                nvgCircle(vg, bx, by, R + (1.f - fc[i]) * 14.f);
                nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.85f), 0.8f * fc[i]));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }
        }
    };

    // ------------------------------------------------------------
    // Readout row under the arena (balls / speed / angle / collisions)
    // ------------------------------------------------------------
    struct BBReadoutWidget : TransparentWidget
    {
        BouncingBalls *module = nullptr;

        BBReadoutWidget(BouncingBalls *m) : module(m)
        {
            box.pos = Vec(BB_DISP_X, BB_READ_Y - 6.f);
            box.size = Vec(BB_DISP_W, 12.f);
        }

        // Layer-1 overlay: draws the four readouts (balls, speed, angle and
        // collision on/off) in four equal slots under the arena. Falls back
        // to sample values in the module browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer == 1)
            {
                NVGcontext *vg = args.vg;
                int n = module ? module->numBalls : 8;
                float spd = module ? (float)module->speedBase : 5.f;
                float ang = module ? (float)(module->angleBase * 180.0 / M_PI) : 0.f;
                bool coll = module ? module->params[BouncingBalls::COLLISIONS_SWITCH_PARAM].getValue() > 0.5f : true;

                const float y = 6.f, slot = BB_DISP_W / 4.f;
                char buf[32];
                snprintf(buf, sizeof(buf), "BALLS %02d", n);
                drawText(vg, slot * 0.5f, y, buf, 7.f, colAccent(), ALIGN_C, 0.5f);
                snprintf(buf, sizeof(buf), "SPD %.1f", spd);
                drawText(vg, slot * 1.5f, y, buf, 7.f, colAccent(), ALIGN_C, 0.5f);
                snprintf(buf, sizeof(buf), "ANG %+.0f", ang);
                drawText(vg, slot * 2.5f, y, buf, 7.f, colAccent(), ALIGN_C, 0.5f);
                drawText(vg, slot * 3.5f, y, coll ? "COLL ON" : "COLL OFF", 7.f,
                         coll ? colAccent() : colDim(), ALIGN_C, 0.5f);
            }
            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Per-ball speed bars
    // ------------------------------------------------------------
    struct BBBarsWidget : TransparentWidget
    {
        BouncingBalls *module = nullptr;

        BBBarsWidget(BouncingBalls *m) : module(m)
        {
            box.pos = Vec(BB_BARS_X, BB_BARS_Y);
            box.size = Vec(BB_BARS_W, BB_BARS_H);
        }

        // Width of one bar slot (the bars area is split evenly across 16).
        static float slot() { return BB_BARS_W / (float)BB_N; }

        // Static layer: the 16 empty bar tracks and their ball-number labels.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            for (int i = 0; i < BB_N; ++i)
            {
                float x = slot() * (i + 0.5f);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, x - 7.f, 0.f, 14.f, BB_BAR_H, 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 10));
                nvgFill(vg);

                char buf[8];
                snprintf(buf, sizeof(buf), "%d", i + 1);
                drawText(vg, x, BB_BAR_H + 6.f, buf, 7.f, colDim(), ALIGN_C, 0.f);
            }
            Widget::draw(args);
        }

        // Layer-1 overlay: fills each active ball's bar in proportion to its
        // speed (scaled so the bar tops out around 28 V/s). Uses sample
        // speeds in the module browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer == 1)
            {
                NVGcontext *vg = args.vg;
                int n = module ? module->numBalls : 8;
                for (int i = 0; i < BB_N; ++i)
                {
                    float x = slot() * (i + 0.5f);
                    if (i >= n)
                        continue;

                    float sp;
                    if (module)
                    {
                        float vx = (float)module->velX[i], vy = (float)module->velY[i];
                        sp = std::sqrt(vx * vx + vy * vy);
                    }
                    else
                    {
                        sp = 6.f + (float)((i * 5) % 9) * 2.2f;
                    }
                    float h = std::min(sp / 28.f, 1.f) * (BB_BAR_H - 2.f);
                    if (h < 1.f)
                        h = 1.f;

                    nvgBeginPath(vg);
                    nvgRect(vg, x - 6.f, BB_BAR_H - h, 12.f, h);
                    nvgFillColor(vg, ballColor(i, 0.85f, 0.55f));
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
struct BouncingBallsWidget : ModuleWidget
{
    // Builds the widget tree: background, screws, arena, readouts, speed
    // bars, the left control block, the right output block and both LED rows.
    BouncingBallsWidget(BouncingBalls *module)
    {
        setModule(module);
        box.size = Vec(BB_PANEL_W, BB_PANEL_H);

        addChild(new BBPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new BBDisplayWidget(module));
        addChild(new BBReadoutWidget(module));
        addChild(new BBBarsWidget(module));

        // ---- left block: control centre = plate top + BB_CELL_CTRL ----
        const float l0 = BB_LCOL[0], l1 = BB_LCOL[1];
        auto ly = [](int row)
        { return BB_ROW_TOP[row] + BB_CELL_CTRL; };

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, ly(0)), module, BouncingBalls::SPEED_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, ly(0)), module, BouncingBalls::SPEED_CV_INPUT));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, ly(1)), module, BouncingBalls::ANGLE_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(l1, ly(1)), module, BouncingBalls::ANGLE_ATTENUATOR_PARAM));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(l0, ly(2)), module, BouncingBalls::ANGLE_CV_INPUT));
        addParam(createParamCentered<CKSS>(Vec(l1, ly(2)), module, BouncingBalls::COLLISIONS_SWITCH_PARAM));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, ly(3)), module, BouncingBalls::NUM_BALLS_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, ly(3)), module, BouncingBalls::BALLS_CV_INPUT));

        addParam(createParamCentered<VCVButton>(Vec(l0, ly(4)), module, BouncingBalls::RESET_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, ly(4)), module, BouncingBalls::RESET_CV_INPUT));

        // ---- right block (all outputs are polyphonic) ----
        const float r0 = BB_RCOL[0], r1 = BB_RCOL[1];
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, ly(0)), module, BouncingBalls::X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, ly(0)), module, BouncingBalls::Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, ly(1)), module, BouncingBalls::GATE_X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, ly(1)), module, BouncingBalls::GATE_Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, ly(2)), module, BouncingBalls::VOCT_X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, ly(2)), module, BouncingBalls::VOCT_Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, ly(3)), module, BouncingBalls::COLLISION_GATE_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, ly(3)), module, BouncingBalls::WALL_HIT_GATE_OUTPUT));

        // ---- LED rows under the arena ----
        for (int i = 0; i < 16; ++i)
        {
            addChild(createLightCentered<SmallLight<BBCyanLight>>(
                Vec(BB_LED_X + i * BB_LED_DX, BB_LED_Y1), module, BouncingBalls::COLLISION_LED1 + i));
            addChild(createLightCentered<SmallLight<BBVioletLight>>(
                Vec(BB_LED_X + i * BB_LED_DX, BB_LED_Y2), module, BouncingBalls::WALL_LED1 + i));
        }
    }

    // Adds the right-click menu: a "V/oct range" submenu and a "Gate
    // duration" submenu, each with a checkmark on the active option.
    void appendContextMenu(ui::Menu *menu) override
    {
        BouncingBalls *module = dynamic_cast<BouncingBalls *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("V/oct range", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                                             for (int i = 0; i < BouncingBalls::NUM_RANGES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(rangeNames[i],
                                                                                  CHECKMARK(module->voctRange == i),
                                                                                  [=]()
                                                                                  { module->voctRange = (BouncingBalls::VoctRange)i; }));
                                             }
                                         }));
        menu->addChild(createSubmenuItem("Gate duration", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                                             for (int i = 0; i < BouncingBalls::NUM_GATE_DURS; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(gateNames[i],
                                                                                  CHECKMARK(module->gateDuration == i),
                                                                                  [=]()
                                                                                  { module->gateDuration = (BouncingBalls::GateDuration)i; }));
                                             }
                                         }));
    }
};

Model *modelBouncingBalls = createModel<BouncingBalls, BouncingBallsWidget>("BouncingBalls");