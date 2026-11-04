#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <random>
#include <limits>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int BB_N = 16;            // max balls
static const int BB_TRAIL_N = 36;      // trail points per ball
static const float BB_TRAIL_HZ = 50.f; // trail sample rate

static const float BB_PANEL_W = RACK_GRID_WIDTH * 28; // 420
static const float BB_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Arena display: 10 px per volt, 6 px rail around the playfield
static const float BB_DISP_W = 220.f;
static const float BB_DISP_H = 220.f;
static const float BB_DISP_X = 100.f;
static const float BB_DISP_Y = 46.f;
static const float BB_SCALE = 10.f;
static const float BB_MARGIN = 6.f;

// Left control block (2 columns x 5 rows) and right output block (2 x 4)
static const float BB_LCOL[2] = {28.f, 68.f};
static const float BB_LROW[5] = {90.f, 138.f, 186.f, 234.f, 282.f};
static const float BB_RCOL[2] = {352.f, 392.f};
static const float BB_ROUT[4] = {100.f, 150.f, 200.f, 250.f};

// LED rows under the display
static const float BB_LED_X = 112.f;
static const float BB_LED_DX = 11.6f;
static const float BB_LED_Y1 = 288.f;
static const float BB_LED_Y2 = 300.f;

// Ball speed bars (bottom strip)
static const float BB_BARS_X = 40.f;
static const float BB_BARS_Y = 330.f;
static const float BB_BARS_W = 340.f;
static const float BB_BARS_H = 34.f;
static const float BB_BAR_H = 22.f; // bar area height inside the widget

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
        SPEED_CV_INPUT, // new
        BALLS_CV_INPUT, // new
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

    enum VoctRange
    {
        RANGE_0_1,
        RANGE_N1_1,
        RANGE_N2_2,
        RANGE_N5_5,
        RANGE_0_5,
        NUM_RANGES
    };
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

    BouncingBalls()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(SPEED_PARAM, 0.f, 20.f, 5.f, "Base speed", " V/s");
        configParam(ANGLE_PARAM, -180.f, 180.f, 0.f, "Base angle", " deg");
        configParam(NUM_BALLS_PARAM, 1.f, 16.f, 1.f, "Number of balls");
        getParamQuantity(NUM_BALLS_PARAM)->snapEnabled = true;
        configButton(RESET_BUTTON, "Reset");
        configParam(ANGLE_ATTENUATOR_PARAM, -1.f, 1.f, 0.f, "Angle CV attenuverter", "%", 0.f, 100.f);
        configSwitch(COLLISIONS_SWITCH_PARAM, 0.f, 1.f, 0.f, "Ball collisions", {"Off", "On"});

        configInput(RESET_CV_INPUT, "Reset gate");
        configInput(ANGLE_CV_INPUT, "Angle CV");
        configInput(SPEED_CV_INPUT, "Speed CV (0 to 10 V adds 0 to 20 V/s)");
        configInput(BALLS_CV_INPUT, "Number of balls CV (10 V adds 15 balls)");

        configOutput(X_OUTPUT, "X (polyphonic)");
        configOutput(Y_OUTPUT, "Y (polyphonic)");
        configOutput(GATE_X_OUTPUT, "Gate X (polyphonic)");
        configOutput(GATE_Y_OUTPUT, "Gate Y (polyphonic)");
        configOutput(VOCT_X_OUTPUT, "Voct X (polyphonic)");
        configOutput(VOCT_Y_OUTPUT, "Voct Y (polyphonic)");
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

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "voctRange", json_integer(voctRange));
        json_object_set_new(root, "gateDuration", json_integer(gateDuration));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *j;
        j = json_object_get(root, "voctRange");
        if (j)
            voctRange = (VoctRange)json_integer_value(j);
        j = json_object_get(root, "gateDuration");
        if (j)
            gateDuration = (GateDuration)json_integer_value(j);
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{

    // ------------------------------------------------------------
    // Theme (same palette as the other AGMM modules)
    // ------------------------------------------------------------
    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    NVGcolor colDim() { return nvgRGB(0x7a, 0x85, 0x94); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    NVGcolor colViolet() { return nvgRGB(0xb0, 0x5a, 0xff); }

    void setFont(NVGcontext *vg)
    {
        std::shared_ptr<window::Font> font =
            APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (font)
            nvgFontFaceId(vg, font->handle);
    }

    void drawText(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
                  int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f)
    {
        setFont(vg);
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Cyan / violet LEDs (Rack has no built-in ones)
    template <typename TBase = GrayModuleLightWidget>
    struct TBBCyanLight : TBase
    {
        TBBCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
    };
    using BBCyanLight = TBBCyanLight<>;

    template <typename TBase = GrayModuleLightWidget>
    struct TBBVioletLight : TBase
    {
        TBBVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
    };
    using BBVioletLight = TBBVioletLight<>;

    // ------------------------------------------------------------
    // AGMM logo
    // ------------------------------------------------------------
    void drawLogo(NVGcontext *vg, float cx, float cy)
    {
        const float w = 64.f, h = 18.f;
        float x = cx - w / 2.f, y = cy - h / 2.f;

        // Glow
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
        nvgFill(vg);

        // Pill body
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 7.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                           nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
        nvgFill(vg);
        nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
                                             nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgStrokeWidth(vg, 1.2f);
        nvgStroke(vg);

        // Tiny cell triangle
        const float s = 2.2f;
        float tx = x + 7.f, ty = y + h / 2.f - 3.f;
        nvgBeginPath(vg);
        nvgRect(vg, tx + s, ty, s, s);
        nvgRect(vg, tx, ty + s * 1.4f, s, s);
        nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
        nvgFillColor(vg, colAccent());
        nvgFill(vg);

        // Wordmark
        drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
    }

    // ------------------------------------------------------------
    // Ball helpers
    // ------------------------------------------------------------

    // Ball colours run from cyan through violet to magenta, so all 16 stay distinct
    // and fit the theme. The colour depends on the ball index, not on the ball count.
    NVGcolor ballColor(int i, float sat, float light)
    {
        float h = 0.5f + 0.45f * (float)i / 15.f;
        return nvgHSL(h, sat, light);
    }

    NVGcolor withAlpha(NVGcolor c, float a)
    {
        c.a = a;
        return c;
    }

    // Shaded sphere with specular highlight and rim
    void drawBall(NVGcontext *vg, float cx, float cy, float r, NVGcolor hi, NVGcolor base, NVGcolor edge)
    {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx - r * 0.35f, cy - r * 0.4f, r * 0.1f, r * 1.25f, hi, base));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r - 0.4f);
        nvgStrokeColor(vg, edge);
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Bounce light on the lower right
        nvgBeginPath(vg);
        nvgCircle(vg, cx + r * 0.38f, cy + r * 0.42f, r * 0.3f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 22));
        nvgFill(vg);

        // Specular
        nvgBeginPath(vg);
        nvgCircle(vg, cx - r * 0.38f, cy - r * 0.42f, r * 0.22f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 215));
        nvgFill(vg);
    }

    // Fading motion trail (older = thinner and fainter)
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

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct BBPanelBackground : TransparentWidget
    {
        BBPanelBackground() { box.size = Vec(BB_PANEL_W, BB_PANEL_H); }

        void label(NVGcontext *vg, float x, float y, const char *txt)
        {
            drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        void plate(NVGcontext *vg, float x, float y, float w, float h)
        {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, y, w, h, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 80));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        void ring(NVGcontext *vg, float cx, float cy, float r)
        {
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, r);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Caption with an underline (side blocks)
        void caption(NVGcontext *vg, float cx, float y, const char *txt, NVGcolor col, float x1, float x2)
        {
            drawText(vg, cx, y, txt, 8.f, col, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x1, y + 8.f);
            nvgLineTo(vg, x2, y + 8.f);
            nvgStrokeColor(vg, withAlpha(col, 0.35f));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Output jack cell: label on top, jack below
        void outCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
            label(vg, cx, cy - 19.f, txt);
        }

        // Left block cells: control in the middle, label below
        void jackCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 17.f, 34.f, 34.f);
            label(vg, cx, cy + 25.f, txt);
        }
        void knobCell(NVGcontext *vg, float cx, float cy, float r, const char *txt)
        {
            ring(vg, cx, cy, r);
            label(vg, cx, cy + 25.f, txt);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            // Base gradient
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, BB_PANEL_W, BB_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, BB_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            // Brushed texture
            nvgStrokeWidth(vg, 1.f);
            for (int y = 0; y < (int)BB_PANEL_H; y += 3)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, y + 0.5f);
                nvgLineTo(vg, BB_PANEL_W, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
                nvgStroke(vg);
            }

            // Edge highlight
            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, BB_PANEL_W - 1.f, BB_PANEL_H - 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Logo + title
            drawLogo(vg, BB_PANEL_W / 2.f, 13.f);
            drawText(vg, BB_PANEL_W / 2.f, 31.f, "BOUNCINGBALLS", 13.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

            // Display bezel
            float bx = BB_DISP_X - 5.f, by = BB_DISP_Y - 5.f;
            float bw = BB_DISP_W + 10.f, bh = BB_DISP_H + 10.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                               nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
            nvgFill(vg);

            // Block captions
            caption(vg, 48.f, 54.f, "CONTROL", colAccent(), 12.f, 84.f);
            caption(vg, 372.f, 54.f, "OUTPUT", colViolet(), 335.f, 409.f);

            // ---- left block ----
            const float l0 = BB_LCOL[0], l1 = BB_LCOL[1];
            knobCell(vg, l0, BB_LROW[0], 18.f, "SPEED");
            jackCell(vg, l1, BB_LROW[0], "SPD CV");
            knobCell(vg, l0, BB_LROW[1], 18.f, "ANGLE");
            knobCell(vg, l1, BB_LROW[1], 13.f, "ANG ATT");
            jackCell(vg, l0, BB_LROW[2], "ANG CV");
            plate(vg, l1 - 17.f, BB_LROW[2] - 20.f, 34.f, 40.f); // collide switch
            label(vg, l1, BB_LROW[2] + 25.f, "COLLIDE");
            knobCell(vg, l0, BB_LROW[3], 18.f, "BALLS");
            jackCell(vg, l1, BB_LROW[3], "BALLS CV");
            knobCell(vg, l0, BB_LROW[4], 17.f, "RESET");
            jackCell(vg, l1, BB_LROW[4], "RESET IN");

            // ---- right block ----
            const float r0 = BB_RCOL[0], r1 = BB_RCOL[1];
            outCell(vg, r0, BB_ROUT[0], "X");
            outCell(vg, r1, BB_ROUT[0], "Y");
            outCell(vg, r0, BB_ROUT[1], "GATE X");
            outCell(vg, r1, BB_ROUT[1], "GATE Y");
            outCell(vg, r0, BB_ROUT[2], "V/OCT X");
            outCell(vg, r1, BB_ROUT[2], "V/OCT Y");
            outCell(vg, r0, BB_ROUT[3], "COLL");
            outCell(vg, r1, BB_ROUT[3], "WALL");

            // ---- LED plate under the display ----
            plate(vg, BB_DISP_X, 278.f, BB_DISP_W, 34.f);
            drawText(vg, 302.f, BB_LED_Y1, "COLL", 6.f, colAccent(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
            drawText(vg, 302.f, BB_LED_Y2, "WALL", 6.f, colViolet(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);

            // ---- speed bars plate ----
            plate(vg, 34.f, 316.f, 352.f, 48.f);
            drawText(vg, 44.f, 322.f, "BALL SPEED", 6.f, colAccent(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.5f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Arena display
    // ------------------------------------------------------------
    struct BBDisplayWidget : TransparentWidget
    {
        BouncingBalls *module = nullptr;

        BBDisplayWidget(BouncingBalls *m) : module(m)
        {
            box.pos = Vec(BB_DISP_X, BB_DISP_Y);
            box.size = Vec(BB_DISP_W, BB_DISP_H);
        }

        // Static background: metal rail, glass playfield, grid, rail ticks
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = BB_DISP_W, H = BB_DISP_H, cx = W / 2.f, cy = H / 2.f, m = BB_MARGIN;

            // Rail
            nvgBeginPath(vg);
            nvgRoundedRect(vg, 0, 0, W, H, 4.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H,
                                               nvgRGB(0x3a, 0x41, 0x4c), nvgRGB(0x14, 0x17, 0x1c)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, 0.5f, 0.5f, W - 1.f, H - 1.f, 4.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Playfield
            nvgBeginPath(vg);
            nvgRect(vg, m, m, W - 2.f * m, H - 2.f * m);
            nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, 20.f, 150.f,
                                               nvgRGB(0x0d, 0x1a, 0x24), nvgRGB(0x03, 0x06, 0x0a)));
            nvgFill(vg);

            // Inner shadow where the rail meets the glass
            nvgBeginPath(vg);
            nvgRect(vg, m + 0.5f, m + 0.5f, W - 2.f * m - 1.f, H - 2.f * m - 1.f);
            nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 200));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Rail ticks every 2 V and corner rivets
            nvgBeginPath(vg);
            for (int k = -5; k <= 5; ++k)
            {
                float p = cx + k * 20.f + 0.5f;
                nvgMoveTo(vg, p, 1.5f);
                nvgLineTo(vg, p, 4.f);
                nvgMoveTo(vg, p, H - 4.f);
                nvgLineTo(vg, p, H - 1.5f);
                float q = cy + k * 20.f + 0.5f;
                nvgMoveTo(vg, 1.5f, q);
                nvgLineTo(vg, 4.f, q);
                nvgMoveTo(vg, W - 4.f, q);
                nvgLineTo(vg, W - 1.5f, q);
            }
            nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgCircle(vg, 3.f, 3.f, 1.2f);
            nvgCircle(vg, W - 3.f, 3.f, 1.2f);
            nvgCircle(vg, 3.f, H - 3.f, 1.2f);
            nvgCircle(vg, W - 3.f, H - 3.f, 1.2f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 90));
            nvgFill(vg);

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

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = BB_DISP_W, H = BB_DISP_H, cx = W / 2.f, cy = H / 2.f;
            const float S = BB_SCALE, m = BB_MARGIN;
            const float R = 4.4f; // ball radius in pixels

            // ---- gather state (live, or a sample scene in the module browser) ----
            int n;
            float px[BB_N], py[BB_N], vx[BB_N], vy[BB_N];
            float fw[BB_N], fc[BB_N], fxc[BB_N], fyc[BB_N], wp[BB_N], xp[BB_N], yp[BB_N];
            int ws[BB_N];
            float tx[BB_N][BB_TRAIL_N], ty[BB_N][BB_TRAIL_N];
            int tn = 0;
            float spdKnob, angDeg;
            bool coll;

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
                spdKnob = (float)module->speedBase;
                angDeg = (float)(module->angleBase * 180.0 / M_PI);
                coll = module->params[BouncingBalls::COLLISIONS_SWITCH_PARAM].getValue() > 0.5f;
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
                spdKnob = 5.f;
                angDeg = 0.f;
                coll = true;
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
                for (int pass = 0; pass < 2; ++pass)
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, x0, y0);
                    nvgLineTo(vg, x1, y1);
                    if (pass == 0)
                    {
                        nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.6f), 0.28f * f));
                        nvgStrokeWidth(vg, 7.f);
                    }
                    else
                    {
                        nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.85f), f));
                        nvgStrokeWidth(vg, 1.8f);
                    }
                    nvgStroke(vg);
                }
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
                    nvgBeginPath(vg);
                    nvgCircle(vg, ax, ay, 1.8f);
                    nvgFillColor(vg, withAlpha(c, f));
                    nvgFill(vg);
                }
            }

            // ---- trails ----
            for (int i = 0; i < n; ++i)
            {
                NVGcolor c = ballColor(i, 0.85f, 0.6f);
                drawTrail(vg, tx[i], ty[i], tn, c, 4.f, 0.12f);
                drawTrail(vg, tx[i], ty[i], tn, c, 1.2f, 0.55f);
            }

            // ---- shadows ----
            nvgBeginPath(vg);
            for (int i = 0; i < n; ++i)
                nvgCircle(vg, cx + px[i] * S + 1.8f, cy - py[i] * S + 2.6f, R);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);

            // ---- heading ticks, halos and balls ----
            for (int i = 0; i < n; ++i)
            {
                float bx = cx + px[i] * S, by = cy - py[i] * S;
                float sp = std::sqrt(vx[i] * vx[i] + vy[i] * vy[i]);
                float energy = std::min(sp / 24.f, 1.f);
                NVGcolor c = ballColor(i, 0.85f, 0.6f);

                // Halo
                float ha = 0.10f + 0.22f * energy + 0.30f * fc[i];
                nvgBeginPath(vg);
                nvgCircle(vg, bx, by, R * 3.f);
                nvgFillPaint(vg, nvgRadialGradient(vg, bx, by, R * 0.6f, R * 3.f,
                                                   withAlpha(c, ha), withAlpha(c, 0.f)));
                nvgFill(vg);

                // Heading tick
                if (sp > 0.05f)
                {
                    float dx = vx[i] / sp, dy = -vy[i] / sp;
                    float len = 3.f + std::min(sp, 24.f) * 0.28f;
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, bx + dx * (R + 1.5f), by + dy * (R + 1.5f));
                    nvgLineTo(vg, bx + dx * (R + 1.5f + len), by + dy * (R + 1.5f + len));
                    nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.85f), 0.55f));
                    nvgStrokeWidth(vg, 1.2f);
                    nvgLineCap(vg, NVG_ROUND);
                    nvgStroke(vg);
                }

                drawBall(vg, bx, by, R,
                         ballColor(i, 0.9f, 0.86f),
                         ballColor(i, 0.8f, 0.42f),
                         ballColor(i, 0.8f, 0.16f));
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

            // ---- scanlines ----
            nvgBeginPath(vg);
            for (int y = 0; y < (int)H; y += 3)
                nvgRect(vg, 0, y + 1, W, 1);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
            nvgFill(vg);

            // ---- glass reflection ----
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H * 0.35f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.35f,
                                               nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
            nvgFill(vg);

            // ---- readouts ----
            char buf[32];
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 200);
            NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 220);
            snprintf(buf, sizeof(buf), "BALLS %02d", n);
            drawText(vg, 11.f, 15.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "SPD %4.1f", spdKnob);
            drawText(vg, W - 11.f, 15.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "ANG %+4.0f", angDeg);
            drawText(vg, 11.f, H - 15.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            drawText(vg, W - 11.f, H - 15.f, coll ? "COLL ON" : "COLL OFF", 7.f,
                     coll ? c1 : cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

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

        static float slot() { return BB_BARS_W / (float)BB_N; }

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
                drawText(vg, x, BB_BAR_H + 7.f, buf, 6.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            }
            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            int n = module ? module->numBalls : 8;
            for (int i = 0; i < BB_N; ++i)
            {
                float x = slot() * (i + 0.5f);
                if (i >= n)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x - 2.f, BB_BAR_H - 1.f, 4.f, 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 25));
                    nvgFill(vg);
                    continue;
                }

                float sp, flash = 0.f;
                if (module)
                {
                    float vx = (float)module->velX[i], vy = (float)module->velY[i];
                    sp = std::sqrt(vx * vx + vy * vy);
                    flash = module->flashCol[i];
                }
                else
                {
                    sp = 6.f + (float)((i * 5) % 9) * 2.2f;
                }
                float h = std::min(sp / 28.f, 1.f) * (BB_BAR_H - 2.f);
                if (h < 1.f)
                    h = 1.f;
                float top = BB_BAR_H - h;

                // Glow
                nvgBeginPath(vg);
                nvgRect(vg, x - 8.5f, top - 1.5f, 17.f, h + 3.f);
                nvgFillColor(vg, withAlpha(ballColor(i, 0.9f, 0.6f), 0.12f + 0.2f * flash));
                nvgFill(vg);

                // Bar
                nvgBeginPath(vg);
                nvgRect(vg, x - 6.f, top, 12.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, BB_BAR_H, 0, 0,
                                                   ballColor(i, 0.8f, 0.30f), ballColor(i, 0.9f, 0.66f)));
                nvgFill(vg);

                // Cap
                nvgBeginPath(vg);
                nvgRect(vg, x - 6.f, top, 12.f, 1.5f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 170));
                nvgFill(vg);
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
        addChild(new BBBarsWidget(module));

        // ---- left block ----
        const float l0 = BB_LCOL[0], l1 = BB_LCOL[1];
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, BB_LROW[0]), module, BouncingBalls::SPEED_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, BB_LROW[0]), module, BouncingBalls::SPEED_CV_INPUT));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, BB_LROW[1]), module, BouncingBalls::ANGLE_PARAM));
        addParam(createParamCentered<Trimpot>(Vec(l1, BB_LROW[1]), module, BouncingBalls::ANGLE_ATTENUATOR_PARAM));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(l0, BB_LROW[2]), module, BouncingBalls::ANGLE_CV_INPUT));
        addParam(createParamCentered<CKSS>(Vec(l1, BB_LROW[2]), module, BouncingBalls::COLLISIONS_SWITCH_PARAM));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, BB_LROW[3]), module, BouncingBalls::NUM_BALLS_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, BB_LROW[3]), module, BouncingBalls::BALLS_CV_INPUT));

        addParam(createParamCentered<VCVButton>(Vec(l0, BB_LROW[4]), module, BouncingBalls::RESET_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, BB_LROW[4]), module, BouncingBalls::RESET_CV_INPUT));

        // ---- right block (all outputs are polyphonic) ----
        const float r0 = BB_RCOL[0], r1 = BB_RCOL[1];
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, BB_ROUT[0]), module, BouncingBalls::X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, BB_ROUT[0]), module, BouncingBalls::Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, BB_ROUT[1]), module, BouncingBalls::GATE_X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, BB_ROUT[1]), module, BouncingBalls::GATE_Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, BB_ROUT[2]), module, BouncingBalls::VOCT_X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, BB_ROUT[2]), module, BouncingBalls::VOCT_Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, BB_ROUT[3]), module, BouncingBalls::COLLISION_GATE_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, BB_ROUT[3]), module, BouncingBalls::WALL_HIT_GATE_OUTPUT));

        // ---- LED rows under the display ----
        for (int i = 0; i < 16; ++i)
        {
            addChild(createLightCentered<SmallLight<BBCyanLight>>(
                Vec(BB_LED_X + i * BB_LED_DX, BB_LED_Y1), module, BouncingBalls::COLLISION_LED1 + i));
            addChild(createLightCentered<SmallLight<BBVioletLight>>(
                Vec(BB_LED_X + i * BB_LED_DX, BB_LED_Y2), module, BouncingBalls::WALL_LED1 + i));
        }
    }

    void appendContextMenu(ui::Menu *menu) override
    {
        BouncingBalls *module = dynamic_cast<BouncingBalls *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Voct range", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                                             for (int i = 0; i < BouncingBalls::NUM_RANGES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(rangeNames[i],
                                                                                  (module->voctRange == i) ? "✔" : "",
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
                                                                                  (module->gateDuration == i) ? "✔" : "",
                                                                                  [=]()
                                                                                  { module->gateDuration = (BouncingBalls::GateDuration)i; }));
                                             }
                                         }));
    }
};

Model *modelBouncingBalls = createModel<BouncingBalls, BouncingBallsWidget>("BouncingBalls");