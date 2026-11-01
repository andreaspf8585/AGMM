#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <random>
#include <limits>
#include <array>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   RW_N         = 16;      // max walkers
static const int   RW_TRAIL_N   = 200;     // trail points per walker
static const float RW_TRAIL_DIST = 0.15f;  // min distance (V) between trail points
static const float RW_TRAIL_HZ  = 240.f;   // max trail sample rate

static const float RW_PANEL_W = RACK_GRID_WIDTH * 28;   // 420
static const float RW_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Arena display: 10 px per volt, 6 px rail around the playfield
static const float RW_DISP_W  = 220.f;
static const float RW_DISP_H  = 220.f;
static const float RW_DISP_X  = 100.f;
static const float RW_DISP_Y  = 46.f;
static const float RW_SCALE   = 10.f;
static const float RW_MARGIN  = 6.f;

// Left control block (2 columns x 4 rows) and right output block (2 x 4)
static const float RW_LCOL[2]  = {28.f, 68.f};
static const float RW_LROW[4]  = {90.f, 138.f, 186.f, 234.f};
static const float RW_RCOL[2]  = {352.f, 392.f};
static const float RW_ROUT[4]  = {100.f, 150.f, 200.f, 250.f};

// Position bars (bottom strip)
static const float RW_BARS_X  = 40.f;
static const float RW_BARS_Y  = 298.f;
static const float RW_BARS_W  = 340.f;
static const float RW_BARS_H  = 64.f;
static const float RW_BAR_H   = 46.f;     // bar area height inside the widget

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct RandomWalk : Module {
    enum ParamId {
        SPEED_PARAM,
        STEP_SIZE_PARAM,
        RESET_BUTTON,
        NUM_BALLS_PARAM,
        SMOOTHNESS_PARAM,    // AR(1) momentum: 0 = none, 1 = constant direction
        PARAMS_LEN
    };
    enum InputId {
        RESET_CV_INPUT,
        SPEED_CV_INPUT,      // new
        STEP_CV_INPUT,       // new
        SMOOTH_CV_INPUT,     // new
        BALLS_CV_INPUT,      // new
        INPUTS_LEN
    };
    enum OutputId {
        X_OUTPUT,
        Y_OUTPUT,
        GATE_X_OUTPUT,
        GATE_Y_OUTPUT,
        VOCT_X_OUTPUT,
        VOCT_Y_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    enum VoctRange {
        RANGE_0_1,
        RANGE_N1_1,
        RANGE_N2_2,
        RANGE_N5_5,
        RANGE_0_5,
        NUM_RANGES
    };
    enum GateDuration {
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
    std::array<double, MAX_BALLS> prevDeltaX{};   // previous step increment
    std::array<double, MAX_BALLS> prevDeltaY{};
    std::array<int, MAX_BALLS> gateXTimer{};
    std::array<int, MAX_BALLS> gateYTimer{};
    std::array<double, MAX_BALLS> voctX{};
    std::array<double, MAX_BALLS> voctY{};

    int numBalls = 1;

    double speedFactor = 1.0;
    double stepSize = 0.001;
    double smoothness = 0.0;   // momentum

    VoctRange voctXRange = RANGE_N1_1, voctYRange = RANGE_N1_1;
    GateDuration gateXDuration = GATE_5MS, gateYDuration = GATE_5MS;

    std::mt19937 rng;
    dsp::SchmittTrigger resetTrigger;
    bool lastResetState = false;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float trailX[MAX_BALLS][RW_TRAIL_N] = {};
    float trailY[MAX_BALLS][RW_TRAIL_N] = {};
    int   trailHead[MAX_BALLS] = {};
    int   trailCount[MAX_BALLS] = {};
    float lastTX[MAX_BALLS] = {};
    float lastTY[MAX_BALLS] = {};
    float trailTimer = 0.f;
    float flashXc[MAX_BALLS] = {};      // X zero crossing (vertical axis)
    float flashYc[MAX_BALLS] = {};      // Y zero crossing (horizontal axis)
    float flashWall[MAX_BALLS] = {};    // reflection at a wall
    int   wallSide[MAX_BALLS] = {};     // 0 left, 1 right, 2 top, 3 bottom
    float wallPos[MAX_BALLS] = {};      // position along that wall (V)
    float xcPos[MAX_BALLS] = {};        // Y value where X crossed zero
    float ycPos[MAX_BALLS] = {};        // X value where Y crossed zero

    RandomWalk() {
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
        onReset();
        for (int i = 0; i < MAX_BALLS; ++i) {
            generateRandomVoctForBall(i, true);
            generateRandomVoctForBall(i, false);
            // give each ball a tiny random initial velocity so smoothness=1 still moves
            std::uniform_real_distribution<double> tiny(-1e-4, 1e-4);
            prevDeltaX[i] = tiny(rng);
            prevDeltaY[i] = tiny(rng);
        }
    }

    void onReset() override {
        for (int i = 0; i < MAX_BALLS; ++i) {
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

    void generateRandomVoctForBall(int ball, bool forX) {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        VoctRange range = forX ? voctXRange : voctYRange;
        double& target = forX ? voctX[ball] : voctY[ball];
        switch (range) {
            case RANGE_0_1:   target = (raw + 1.0) / 2.0; break;
            case RANGE_N1_1:  target = raw; break;
            case RANGE_N2_2:  target = raw * 2.0; break;
            case RANGE_N5_5:  target = raw * 5.0; break;
            case RANGE_0_5:   target = (raw + 1.0) / 2.0 * 5.0; break;
            default:          target = raw; break;
        }
    }

    double getGateDurationSeconds(GateDuration dur) {
        switch (dur) {
            case GATE_TRIGGER: return 0.0005;
            case GATE_5MS:     return 0.005;
            case GATE_10MS:    return 0.010;
            case GATE_20MS:    return 0.020;
            case GATE_50MS:    return 0.050;
            case GATE_100MS:   return 0.100;
            case GATE_200MS:   return 0.200;
            default: return 0.005;
        }
    }

    void updateParameters() {
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
        int newNum = rack::math::clamp((int) std::round(nb), 1, MAX_BALLS);
        if (newNum != numBalls) {
            // restart the trails of balls that join, so they don't draw a stale streak
            for (int i = numBalls; i < newNum; ++i) {
                trailHead[i] = 0;
                trailCount[i] = 0;
            }
            numBalls = newNum;
        }
    }

    void process(const ProcessArgs& args) override {
        updateParameters();

        bool resetNow = (params[RESET_BUTTON].getValue() > 0.5f) ||
                        (inputs[RESET_CV_INPUT].getVoltage() > 1.0f);
        if (resetNow && !lastResetState) {
            onReset();
            // re-seed random initial deltas so smoothness=1 moves after reset
            std::uniform_real_distribution<double> tiny(-1e-4, 1e-4);
            for (int i = 0; i < MAX_BALLS; ++i) {
                prevDeltaX[i] = tiny(rng);
                prevDeltaY[i] = tiny(rng);
            }
        }
        lastResetState = resetNow;

        // Decay the visual flashes
        for (int i = 0; i < MAX_BALLS; ++i) {
            flashXc[i]   = std::max(0.f, flashXc[i]   - 4.f * args.sampleTime);
            flashYc[i]   = std::max(0.f, flashYc[i]   - 4.f * args.sampleTime);
            flashWall[i] = std::max(0.f, flashWall[i] - 5.f * args.sampleTime);
        }

        // Standard deviation per sample for a fresh random step
        double sigma = stepSize * std::sqrt(speedFactor * args.sampleTime);
        std::normal_distribution<double> noise(0.0, sigma);

        for (int i = 0; i < numBalls; ++i) {
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
            if (x[i] > 10.0)  { x[i] = 20.0 - x[i];  side = 1; wpos = (float) y[i]; }
            if (x[i] < -10.0) { x[i] = -20.0 - x[i]; side = 0; wpos = (float) y[i]; }
            if (y[i] > 10.0)  { y[i] = 20.0 - y[i];  side = 2; wpos = (float) x[i]; }
            if (y[i] < -10.0) { y[i] = -20.0 - y[i]; side = 3; wpos = (float) x[i]; }
            if (side >= 0) {
                flashWall[i] = 1.f;
                wallSide[i] = side;
                wallPos[i] = wpos;
            }

            // Zero crossing detection
            if ((prevX[i] <= 0 && x[i] > 0) || (prevX[i] >= 0 && x[i] < 0)) {
                generateRandomVoctForBall(i, true);
                double dur = getGateDurationSeconds(gateXDuration);
                gateXTimer[i] = static_cast<int>(args.sampleRate * dur);
                flashXc[i] = 1.f;
                xcPos[i] = (float) y[i];
            }
            if ((prevY[i] <= 0 && y[i] > 0) || (prevY[i] >= 0 && y[i] < 0)) {
                generateRandomVoctForBall(i, false);
                double dur = getGateDurationSeconds(gateYDuration);
                gateYTimer[i] = static_cast<int>(args.sampleRate * dur);
                flashYc[i] = 1.f;
                ycPos[i] = (float) x[i];
            }

            if (gateXTimer[i] > 0) gateXTimer[i]--;
            if (gateYTimer[i] > 0) gateYTimer[i]--;
        }

        // Set polyphonic outputs
        outputs[X_OUTPUT].setChannels(numBalls);
        outputs[Y_OUTPUT].setChannels(numBalls);
        outputs[GATE_X_OUTPUT].setChannels(numBalls);
        outputs[GATE_Y_OUTPUT].setChannels(numBalls);
        outputs[VOCT_X_OUTPUT].setChannels(numBalls);
        outputs[VOCT_Y_OUTPUT].setChannels(numBalls);

        for (int i = 0; i < numBalls; ++i) {
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
        if (trailTimer >= 1.f / RW_TRAIL_HZ) {
            trailTimer = 0.f;
            for (int i = 0; i < numBalls; ++i) {
                float px = (float) x[i], py = (float) y[i];
                float dx = px - lastTX[i], dy = py - lastTY[i];
                if (trailCount[i] == 0 || dx * dx + dy * dy >= RW_TRAIL_DIST * RW_TRAIL_DIST) {
                    trailX[i][trailHead[i]] = px;
                    trailY[i][trailHead[i]] = py;
                    trailHead[i] = (trailHead[i] + 1) % RW_TRAIL_N;
                    if (trailCount[i] < RW_TRAIL_N) trailCount[i]++;
                    lastTX[i] = px;
                    lastTY[i] = py;
                }
            }
        }
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "voctXRange", json_integer(voctXRange));
        json_object_set_new(root, "voctYRange", json_integer(voctYRange));
        json_object_set_new(root, "gateXDuration", json_integer(gateXDuration));
        json_object_set_new(root, "gateYDuration", json_integer(gateYDuration));
        // the knob values (not the CV-modulated ones)
        json_object_set_new(root, "numBalls", json_integer((int) std::round(params[NUM_BALLS_PARAM].getValue())));
        json_object_set_new(root, "smoothness", json_real(params[SMOOTHNESS_PARAM].getValue()));
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* j;
        j = json_object_get(root, "voctXRange");
        if (j) voctXRange = (VoctRange) json_integer_value(j);
        j = json_object_get(root, "voctYRange");
        if (j) voctYRange = (VoctRange) json_integer_value(j);
        j = json_object_get(root, "gateXDuration");
        if (j) gateXDuration = (GateDuration) json_integer_value(j);
        j = json_object_get(root, "gateYDuration");
        if (j) gateYDuration = (GateDuration) json_integer_value(j);
        j = json_object_get(root, "numBalls");
        if (j) {
            int nb = rack::math::clamp((int) json_integer_value(j), 1, MAX_BALLS);
            numBalls = nb;
            params[NUM_BALLS_PARAM].setValue((float) nb);
        }
        j = json_object_get(root, "smoothness");
        if (j) {
            smoothness = json_real_value(j);
            params[SMOOTHNESS_PARAM].setValue((float) smoothness);
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

// ------------------------------------------------------------
// AGMM logo
// ------------------------------------------------------------
void drawLogo(NVGcontext* vg, float cx, float cy) {
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
    nvgRect(vg, tx + s,       ty,            s, s);
    nvgRect(vg, tx,           ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);

    // Wordmark
    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
             NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

// ------------------------------------------------------------
// Walker helpers
// ------------------------------------------------------------

// Colours run from cyan through violet to magenta, so all 16 stay distinct
// and fit the theme. The colour depends on the walker index, not on the count.
NVGcolor ballColor(int i, float sat, float light) {
    float h = 0.5f + 0.45f * (float) i / 15.f;
    return nvgHSL(h, sat, light);
}

NVGcolor withAlpha(NVGcolor c, float a) {
    c.a = a;
    return c;
}

// Shaded sphere with specular highlight and rim
void drawBall(NVGcontext* vg, float cx, float cy, float r, NVGcolor hi, NVGcolor base, NVGcolor edge) {
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

// Fading path (older = thinner and fainter). K points per stroked segment.
void drawTrail(NVGcontext* vg, const float* xs, const float* ys, int n, NVGcolor col,
               float width, float alpha, int K) {
    if (n < 2) return;
    nvgLineCap(vg, NVG_ROUND);
    nvgLineJoin(vg, NVG_ROUND);
    for (int s = 0; s < n - 1; s += K) {
        int e = std::min(s + K, n - 1);
        float t = (float) (s + 1) / (float) n;
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
struct RWPanelBackground : TransparentWidget {
    RWPanelBackground() { box.size = Vec(RW_PANEL_W, RW_PANEL_H); }

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

    // Caption with an underline (side blocks)
    void caption(NVGcontext* vg, float cx, float y, const char* txt, NVGcolor col, float x1, float x2) {
        drawText(vg, cx, y, txt, 8.f, col, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y + 8.f);
        nvgLineTo(vg, x2, y + 8.f);
        nvgStrokeColor(vg, withAlpha(col, 0.35f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Output cell: label on top, jack below
    void outCell(NVGcontext* vg, float cx, float cy, const char* txt) {
        plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
        label(vg, cx, cy - 19.f, txt);
    }

    // Left block cells: control in the middle, label below
    void jackCell(NVGcontext* vg, float cx, float cy, const char* txt) {
        plate(vg, cx - 17.f, cy - 17.f, 34.f, 34.f);
        label(vg, cx, cy + 25.f, txt);
    }
    void knobCell(NVGcontext* vg, float cx, float cy, float r, const char* txt) {
        ring(vg, cx, cy, r);
        label(vg, cx, cy + 25.f, txt);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, RW_PANEL_W, RW_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, RW_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) RW_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, RW_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, RW_PANEL_W - 1.f, RW_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, RW_PANEL_W / 2.f, 13.f);
        drawText(vg, RW_PANEL_W / 2.f, 31.f, "RANDOMWALK", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = RW_DISP_X - 5.f, by = RW_DISP_Y - 5.f;
        float bw = RW_DISP_W + 10.f, bh = RW_DISP_H + 10.f;
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
        const float l0 = RW_LCOL[0], l1 = RW_LCOL[1];
        knobCell(vg, l0, RW_LROW[0], 18.f, "SPEED");
        jackCell(vg, l1, RW_LROW[0], "SPD CV");
        knobCell(vg, l0, RW_LROW[1], 18.f, "STEP");
        jackCell(vg, l1, RW_LROW[1], "STEP CV");
        knobCell(vg, l0, RW_LROW[2], 18.f, "SMOOTH");
        jackCell(vg, l1, RW_LROW[2], "SMTH CV");
        knobCell(vg, l0, RW_LROW[3], 18.f, "BALLS");
        jackCell(vg, l1, RW_LROW[3], "BALLS CV");

        // ---- right block ----
        const float r0 = RW_RCOL[0], r1 = RW_RCOL[1];
        outCell(vg, r0, RW_ROUT[0], "X");
        outCell(vg, r1, RW_ROUT[0], "Y");
        outCell(vg, r0, RW_ROUT[1], "GATE X");
        outCell(vg, r1, RW_ROUT[1], "GATE Y");
        outCell(vg, r0, RW_ROUT[2], "V/OCT X");
        outCell(vg, r1, RW_ROUT[2], "V/OCT Y");
        outCell(vg, r0, RW_ROUT[3], "RESET");
        outCell(vg, r1, RW_ROUT[3], "RESET IN");

        // ---- position bars plate ----
        plate(vg, 34.f, 284.f, 352.f, 80.f);
        drawText(vg, 44.f, 292.f, "WALKER POSITION", 6.f, colAccent(),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.5f);
        drawText(vg, 376.f, 292.f, "X | Y", 6.f, colDim(),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.5f);

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Arena display
// ------------------------------------------------------------
struct RWDisplayWidget : TransparentWidget {
    RandomWalk* module = nullptr;

    RWDisplayWidget(RandomWalk* m) : module(m) {
        box.pos = Vec(RW_DISP_X, RW_DISP_Y);
        box.size = Vec(RW_DISP_W, RW_DISP_H);
    }

    // Static background: metal rail, glass playfield, grid, rail ticks
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float W = RW_DISP_W, H = RW_DISP_H, cx = W / 2.f, cy = H / 2.f, m = RW_MARGIN;

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
        for (int k = -5; k <= 5; ++k) {
            float p = cx + k * 20.f + 0.5f;
            nvgMoveTo(vg, p, 1.5f);      nvgLineTo(vg, p, 4.f);
            nvgMoveTo(vg, p, H - 4.f);   nvgLineTo(vg, p, H - 1.5f);
            float q = cy + k * 20.f + 0.5f;
            nvgMoveTo(vg, 1.5f, q);      nvgLineTo(vg, 4.f, q);
            nvgMoveTo(vg, W - 4.f, q);   nvgLineTo(vg, W - 1.5f, q);
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
        for (int k = -5; k <= 5; ++k) {
            if (k == 0) continue;
            float p = cx + k * 20.f + 0.5f;
            nvgMoveTo(vg, p, m);  nvgLineTo(vg, p, H - m);
            float q = cy + k * 20.f + 0.5f;
            nvgMoveTo(vg, m, q);  nvgLineTo(vg, W - m, q);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Axes (a gate fires whenever a walker crosses one)
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + 0.5f, m);  nvgLineTo(vg, cx + 0.5f, H - m);
        nvgMoveTo(vg, m, cy + 0.5f);  nvgLineTo(vg, W - m, cy + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 38));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Origin marker (where every walk starts)
        nvgBeginPath(vg);
        nvgCircle(vg, cx + 0.5f, cy + 0.5f, 3.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float W = RW_DISP_W, H = RW_DISP_H, cx = W / 2.f, cy = H / 2.f;
        const float S = RW_SCALE, m = RW_MARGIN;
        const float R = 4.4f;     // ball radius in pixels

        // ---- gather state (live, or a sample scene in the module browser) ----
        int n;
        float px[RW_N], py[RW_N], hx[RW_N], hy[RW_N];
        float fw[RW_N], fxc[RW_N], fyc[RW_N], wp[RW_N], xp[RW_N], yp[RW_N];
        int ws[RW_N];
        static const int TN = RW_TRAIL_N + 1;
        float tx[RW_N][TN], ty[RW_N][TN];
        int tn[RW_N];
        float spdFactor, stepV, smooth;

        for (int i = 0; i < RW_N; ++i) {
            px[i] = py[i] = hx[i] = hy[i] = 0.f;
            fw[i] = fxc[i] = fyc[i] = 0.f;
            wp[i] = xp[i] = yp[i] = 0.f;
            ws[i] = 0;
            tn[i] = 0;
        }

        if (module) {
            n = module->numBalls;
            for (int i = 0; i < n; ++i) {
                px[i] = (float) module->x[i];
                py[i] = (float) module->y[i];
                hx[i] = (float) module->prevDeltaX[i];
                hy[i] = (float) module->prevDeltaY[i];
                fw[i] = module->flashWall[i];
                fxc[i] = module->flashXc[i];
                fyc[i] = module->flashYc[i];
                ws[i] = module->wallSide[i];
                wp[i] = module->wallPos[i];
                xp[i] = module->xcPos[i];
                yp[i] = module->ycPos[i];

                int cnt = std::min(std::max(module->trailCount[i], 0), RW_TRAIL_N);
                int head = module->trailHead[i];
                for (int k = 0; k < cnt; ++k) {
                    int idx = ((head - cnt + k) % RW_TRAIL_N + RW_TRAIL_N) % RW_TRAIL_N;
                    tx[i][k] = cx + module->trailX[i][idx] * S;
                    ty[i][k] = cy - module->trailY[i][idx] * S;
                }
                // attach the path to the walker itself
                tx[i][cnt] = cx + px[i] * S;
                ty[i][cnt] = cy - py[i] * S;
                tn[i] = cnt + 1;
            }
            spdFactor = (float) module->speedFactor;
            stepV = (float) module->stepSize;
            smooth = (float) module->smoothness;
        } else {
            n = 6;
            const int pts = 140;
            for (int i = 0; i < n; ++i) {
                for (int k = 0; k < pts; ++k) {
                    float t = k * 0.09f + i * 10.f;
                    float x = 6.f * std::sin(0.7f * t + 1.3f * i) + 3.f * std::sin(1.9f * t + 2.1f * i);
                    float y = 6.f * std::sin(0.6f * t + 0.7f * i) + 3.f * std::cos(1.7f * t + (float) i);
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
            fxc[2] = 0.8f; xp[2] = 3.f;
            fyc[4] = 0.6f; yp[4] = -4.f;
            fw[1] = 0.9f; ws[1] = 1; wp[1] = 2.f;
            spdFactor = 2.1f;
            stepV = 0.1f;
            smooth = 0.5f;
        }

        // ---- wall reflection flashes on the rail ----
        nvgLineCap(vg, NVG_ROUND);
        for (int i = 0; i < n; ++i) {
            if (fw[i] < 0.02f) continue;
            float f = fw[i];
            float a = (ws[i] <= 1) ? (cy - wp[i] * S) : (cx + wp[i] * S);
            float lo = std::max(m, a - 16.f), hi = std::min(H - m, a + 16.f);
            float x0, y0, x1, y1;
            if (ws[i] == 0)      { x0 = x1 = m + 0.5f;     y0 = lo; y1 = hi; }
            else if (ws[i] == 1) { x0 = x1 = W - m - 0.5f; y0 = lo; y1 = hi; }
            else if (ws[i] == 2) { y0 = y1 = m + 0.5f;     x0 = lo; x1 = hi; }
            else                 { y0 = y1 = H - m - 0.5f; x0 = lo; x1 = hi; }
            for (int pass = 0; pass < 2; ++pass) {
                nvgBeginPath(vg);
                nvgMoveTo(vg, x0, y0);
                nvgLineTo(vg, x1, y1);
                if (pass == 0) {
                    nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.6f), 0.28f * f));
                    nvgStrokeWidth(vg, 7.f);
                } else {
                    nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.85f), f));
                    nvgStrokeWidth(vg, 1.8f);
                }
                nvgStroke(vg);
            }
        }

        // ---- axis crossing flashes (gate triggers) ----
        for (int i = 0; i < n; ++i) {
            for (int a = 0; a < 2; ++a) {
                float f = (a == 0) ? fxc[i] : fyc[i];
                if (f < 0.02f) continue;
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

        // ---- paths ----
        for (int i = 0; i < n; ++i) {
            NVGcolor c = ballColor(i, 0.85f, 0.6f);
            drawTrail(vg, tx[i], ty[i], tn[i], c, 4.f, 0.12f, 14);
            drawTrail(vg, tx[i], ty[i], tn[i], c, 1.2f, 0.60f, 5);
        }

        // ---- shadows ----
        nvgBeginPath(vg);
        for (int i = 0; i < n; ++i)
            nvgCircle(vg, cx + px[i] * S + 1.8f, cy - py[i] * S + 2.6f, R);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgFill(vg);

        // ---- momentum ticks, halos and balls ----
        for (int i = 0; i < n; ++i) {
            float bx = cx + px[i] * S, by = cy - py[i] * S;
            float flash = std::max(fxc[i], fyc[i]);
            NVGcolor c = ballColor(i, 0.85f, 0.6f);

            // Halo
            float ha = 0.12f + 0.32f * flash;
            nvgBeginPath(vg);
            nvgCircle(vg, bx, by, R * 3.f);
            nvgFillPaint(vg, nvgRadialGradient(vg, bx, by, R * 0.6f, R * 3.f,
                                               withAlpha(c, ha), withAlpha(c, 0.f)));
            nvgFill(vg);

            // Momentum tick: the direction the walker is carrying, visible with smoothness
            float mag = std::sqrt(hx[i] * hx[i] + hy[i] * hy[i]);
            if (smooth > 0.05f && mag > 1e-9f) {
                float dx = hx[i] / mag, dy = -hy[i] / mag;
                float len = 3.f + 9.f * smooth;
                nvgBeginPath(vg);
                nvgMoveTo(vg, bx + dx * (R + 1.5f), by + dy * (R + 1.5f));
                nvgLineTo(vg, bx + dx * (R + 1.5f + len), by + dy * (R + 1.5f + len));
                nvgStrokeColor(vg, withAlpha(ballColor(i, 0.9f, 0.85f), 0.2f + 0.5f * smooth));
                nvgStrokeWidth(vg, 1.2f);
                nvgLineCap(vg, NVG_ROUND);
                nvgStroke(vg);
            }

            drawBall(vg, bx, by, R,
                     ballColor(i, 0.9f, 0.86f),
                     ballColor(i, 0.8f, 0.42f),
                     ballColor(i, 0.8f, 0.16f));
        }

        // ---- scanlines ----
        nvgBeginPath(vg);
        for (int y = 0; y < (int) H; y += 3)
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
        snprintf(buf, sizeof(buf), "WALKERS %02d", n);
        drawText(vg, 11.f, 15.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "SPD %.1fX", spdFactor);
        drawText(vg, W - 11.f, 15.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "STEP %.2fV", stepV);
        drawText(vg, 11.f, H - 15.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "SMOOTH %3.0f%%", smooth * 100.f);
        drawText(vg, W - 11.f, H - 15.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// Per-walker position bars (X and Y voltage, bipolar)
// ------------------------------------------------------------
struct RWBarsWidget : TransparentWidget {
    RandomWalk* module = nullptr;

    RWBarsWidget(RandomWalk* m) : module(m) {
        box.pos = Vec(RW_BARS_X, RW_BARS_Y);
        box.size = Vec(RW_BARS_W, RW_BARS_H);
    }

    static float slot() { return RW_BARS_W / (float) RW_N; }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float mid = RW_BAR_H / 2.f;
        for (int i = 0; i < RW_N; ++i) {
            float cx = slot() * (i + 0.5f);

            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 9.f, 0.f, 8.f, RW_BAR_H, 2.f);
            nvgRoundedRect(vg, cx + 1.f, 0.f, 8.f, RW_BAR_H, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 10));
            nvgFill(vg);

            // Zero line
            nvgBeginPath(vg);
            nvgRect(vg, cx - 9.f, mid, 18.f, 1.f);
            nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgFill(vg);

            char buf[8];
            snprintf(buf, sizeof(buf), "%d", i + 1);
            drawText(vg, cx, RW_BAR_H + 8.f, buf, 6.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float mid = RW_BAR_H / 2.f;
        const float amp = mid - 2.f;

        int n = module ? module->numBalls : 6;

        auto bar = [&](float xl, float v, NVGcolor col) {
            float h = std::min(std::fabs(v) / 10.f, 1.f) * amp;
            if (h < 1.f) h = 1.f;
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

        for (int i = 0; i < RW_N; ++i) {
            float cx = slot() * (i + 0.5f);
            if (i >= n) {
                nvgBeginPath(vg);
                nvgRect(vg, cx - 3.f, mid, 6.f, 1.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 25));
                nvgFill(vg);
                continue;
            }

            float vx, vy, fx = 0.f, fy = 0.f;
            if (module) {
                vx = (float) module->x[i];
                vy = (float) module->y[i];
                fx = module->flashXc[i];
                fy = module->flashYc[i];
            } else {
                vx = 8.f * std::sin(1.3f * i + 0.4f);
                vy = 7.f * std::cos(1.9f * i + 0.8f);
            }

            bar(cx - 9.f, vx, withAlpha(ballColor(i, 0.9f, 0.62f), 0.95f));
            bar(cx + 1.f, vy, withAlpha(ballColor(i, 0.9f, 0.80f), 0.85f));

            // Gate flashes on the zero line
            if (fx > 0.02f) {
                nvgBeginPath(vg);
                nvgRect(vg, cx - 9.f, mid - 1.f, 8.f, 3.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, (int) (230.f * fx)));
                nvgFill(vg);
            }
            if (fy > 0.02f) {
                nvgBeginPath(vg);
                nvgRect(vg, cx + 1.f, mid - 1.f, 8.f, 3.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, (int) (230.f * fy)));
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
struct RandomWalkWidget : ModuleWidget {
    RandomWalkWidget(RandomWalk* module) {
        setModule(module);
        box.size = Vec(RW_PANEL_W, RW_PANEL_H);

        addChild(new RWPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new RWDisplayWidget(module));
        addChild(new RWBarsWidget(module));

        // ---- left block: knob + its CV jack ----
        const float l0 = RW_LCOL[0], l1 = RW_LCOL[1];
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, RW_LROW[0]), module, RandomWalk::SPEED_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, RW_LROW[0]), module, RandomWalk::SPEED_CV_INPUT));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, RW_LROW[1]), module, RandomWalk::STEP_SIZE_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, RW_LROW[1]), module, RandomWalk::STEP_CV_INPUT));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, RW_LROW[2]), module, RandomWalk::SMOOTHNESS_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, RW_LROW[2]), module, RandomWalk::SMOOTH_CV_INPUT));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(l0, RW_LROW[3]), module, RandomWalk::NUM_BALLS_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(l1, RW_LROW[3]), module, RandomWalk::BALLS_CV_INPUT));

        // ---- right block (all outputs are polyphonic) ----
        const float r0 = RW_RCOL[0], r1 = RW_RCOL[1];
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, RW_ROUT[0]), module, RandomWalk::X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, RW_ROUT[0]), module, RandomWalk::Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, RW_ROUT[1]), module, RandomWalk::GATE_X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, RW_ROUT[1]), module, RandomWalk::GATE_Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r0, RW_ROUT[2]), module, RandomWalk::VOCT_X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(r1, RW_ROUT[2]), module, RandomWalk::VOCT_Y_OUTPUT));

        addParam(createParamCentered<VCVButton>(Vec(r0, RW_ROUT[3]), module, RandomWalk::RESET_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(r1, RW_ROUT[3]), module, RandomWalk::RESET_CV_INPUT));
    }

    void appendContextMenu(ui::Menu* menu) override {
        RandomWalk* module = dynamic_cast<RandomWalk*>(this->module);
        if (!module) return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Voct X range", "",
            [=](ui::Menu* submenu) {
                const char* rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                for (int i = 0; i < RandomWalk::NUM_RANGES; ++i) {
                    submenu->addChild(createMenuItem(rangeNames[i],
                        (module->voctXRange == i) ? "✔" : "",
                        [=]() {
                            module->voctXRange = (RandomWalk::VoctRange) i;
                            for (int b = 0; b < RandomWalk::MAX_BALLS; ++b)
                                module->generateRandomVoctForBall(b, true);
                        }));
                }
            }
        ));
        menu->addChild(createSubmenuItem("Voct Y range", "",
            [=](ui::Menu* submenu) {
                const char* rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                for (int i = 0; i < RandomWalk::NUM_RANGES; ++i) {
                    submenu->addChild(createMenuItem(rangeNames[i],
                        (module->voctYRange == i) ? "✔" : "",
                        [=]() {
                            module->voctYRange = (RandomWalk::VoctRange) i;
                            for (int b = 0; b < RandomWalk::MAX_BALLS; ++b)
                                module->generateRandomVoctForBall(b, false);
                        }));
                }
            }
        ));
        menu->addChild(createSubmenuItem("Gate X duration", "",
            [=](ui::Menu* submenu) {
                const char* gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                for (int i = 0; i < RandomWalk::NUM_GATE_DURS; ++i) {
                    submenu->addChild(createMenuItem(gateNames[i],
                        (module->gateXDuration == i) ? "✔" : "",
                        [=]() { module->gateXDuration = (RandomWalk::GateDuration) i; }));
                }
            }
        ));
        menu->addChild(createSubmenuItem("Gate Y duration", "",
            [=](ui::Menu* submenu) {
                const char* gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                for (int i = 0; i < RandomWalk::NUM_GATE_DURS; ++i) {
                    submenu->addChild(createMenuItem(gateNames[i],
                        (module->gateYDuration == i) ? "✔" : "",
                        [=]() { module->gateYDuration = (RandomWalk::GateDuration) i; }));
                }
            }
        ));
    }
};

Model* modelRandomWalk = createModel<RandomWalk, RandomWalkWidget>("RandomWalk");