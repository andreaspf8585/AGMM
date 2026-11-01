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
static const int   PL_TRAIL_N  = 120;      // trail points per bob
static const float PL_TRAIL_HZ = 150.f;    // trail sample rate

static const float PL_PANEL_W = RACK_GRID_WIDTH * 28;   // 420
static const float PL_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float PL_DISP_W = 200.f;
static const float PL_DISP_H = 200.f;
static const float PL_DISP_X = (PL_PANEL_W - PL_DISP_W) / 2.f;
static const float PL_DISP_Y = 46.f;

// Side blocks: pendulum 1 on the left, pendulum 2 on the right
static const float PL_LCOL[2]   = {32.f, 72.f};
static const float PL_RCOL[2]   = {348.f, 388.f};
static const float PL_KNOB_Y    = 108.f;
static const float PL_OUT_Y[2]  = {176.f, 232.f};

// Bottom control row
static const float PL_CTRL_Y  = 312.f;
static const float PL_SPEED_X = 34.f;
static const float PL_SPDCV_X = 78.f;
static const float PL_GRAV_X  = 130.f;
static const float PL_DAMP_X  = 182.f;
static const float PL_KICKB_X = 246.f;
static const float PL_KICKI_X = 290.f;
static const float PL_RSTB_X  = 346.f;
static const float PL_RSTI_X  = 390.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct PsyLFO : Module {
    enum ParamId {
        SPEED_PARAM,
        MASS1_PARAM,
        MASS2_PARAM,
        LEN1_PARAM,
        LEN2_PARAM,
        RESET_BUTTON,
        GRAVITY_PARAM,      // new
        DAMPING_PARAM,      // new
        KICK_BUTTON,        // new
        PARAMS_LEN
    };
    enum InputId {
        RESET_CV_INPUT,
        SPEED_CV_INPUT,     // new
        KICK_INPUT,         // new
        INPUTS_LEN
    };
    enum OutputId {
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
    enum LightId {
        MASS1_LIGHT,
        MASS2_LIGHT,
        LIGHTS_LEN
    };

    // Voct range selection
    enum VoctRange {
        RANGE_0_1,
        RANGE_N1_1,
        RANGE_N2_2,
        RANGE_N5_5,
        RANGE_0_5,
        NUM_RANGES
    };
    // Gate duration selection (in ms, except trigger which is 0.5 ms)
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

    PsyLFO() {
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
        onReset();

        // Initial random voltages
        generateRandomVoct1();
        generateRandomVoct2();
    }

    void onReset() override {
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
    void kick() {
        std::uniform_real_distribution<double> d1(-4.0, 4.0);
        std::uniform_real_distribution<double> d2(-6.0, 6.0);
        omega1 += d1(rng);
        omega2 += d2(rng);
    }

    // Helper: map random value in [-1,1] to selected range
    double scaleVoct(double raw, VoctRange range) {
        switch (range) {
            case RANGE_0_1:   return (raw + 1.0) / 2.0;           // 0..1
            case RANGE_N1_1:  return raw;                         // -1..1
            case RANGE_N2_2:  return raw * 2.0;                   // -2..2
            case RANGE_N5_5:  return raw * 5.0;                   // -5..5
            case RANGE_0_5:   return (raw + 1.0) / 2.0 * 5.0;     // 0..5
            default: return raw;
        }
    }

    void generateRandomVoct1() {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        voct1Voltage = scaleVoct(raw, voct1Range);
    }

    void generateRandomVoct2() {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        voct2Voltage = scaleVoct(raw, voct2Range);
    }

    double getGateDurationSeconds(GateDuration dur) {
        switch (dur) {
            case GATE_TRIGGER: return 0.0005;   // 0.5 ms
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
        float speedKnob = params[SPEED_PARAM].getValue();
        if (inputs[SPEED_CV_INPUT].isConnected())
            speedKnob = rack::math::clamp(speedKnob + inputs[SPEED_CV_INPUT].getVoltage() / 10.f, 0.f, 1.f);
        speedFactor = 0.2 + speedKnob * 9.8;

        mass1 = params[MASS1_PARAM].getValue();
        mass2 = params[MASS2_PARAM].getValue();
        len1 = params[LEN1_PARAM].getValue();
        len2 = params[LEN2_PARAM].getValue();
        g = params[GRAVITY_PARAM].getValue();
        damping = params[DAMPING_PARAM].getValue() * 2.0;   // 1/s of simulated time
    }

    bool isValid() {
        return std::isfinite(theta1) && std::isfinite(theta2) &&
               std::isfinite(omega1) && std::isfinite(omega2);
    }

    void computeDerivatives(double t1, double t2, double w1, double w2,
                            double& dw1, double& dw2) {
        double sin_d = sin(t2 - t1);
        double cos_d = cos(t2 - t1);
        double denom1 = (mass1 + mass2) * len1 - mass2 * len1 * cos_d * cos_d + eps;
        double denom2 = len2 * (mass1 + mass2 - mass2 * cos_d * cos_d) + eps;

        dw1 = (mass2 * len1 * w1 * w1 * sin_d * cos_d +
               mass2 * g * sin(t2) * cos_d +
               mass2 * len2 * w2 * w2 * sin_d -
               (mass1 + mass2) * g * sin(t1)) / denom1;

        dw2 = (-mass2 * len2 * w2 * w2 * sin_d * cos_d +
               (mass1 + mass2) * g * sin(t1) * cos_d -
               (mass1 + mass2) * len1 * w1 * w1 * sin_d -
               (mass1 + mass2) * g * sin(t2)) / denom2;

        // Pivot friction (damping = 0 leaves the original behaviour)
        dw1 -= damping * w1;
        dw2 -= damping * w2;
    }

    void rk4(double dt) {
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
        if (theta1 < 0) theta1 += 2.0 * M_PI;
        theta1 -= M_PI;
        theta2 = fmod(theta2 + M_PI, 2.0 * M_PI);
        if (theta2 < 0) theta2 += 2.0 * M_PI;
        theta2 -= M_PI;
    }

    void process(const ProcessArgs& args) override {
        updateParameters();

        bool resetNow = (params[RESET_BUTTON].getValue() > 0.5f) ||
                        (inputs[RESET_CV_INPUT].getVoltage() > 1.0f);
        if (resetNow && !lastResetState) onReset();
        lastResetState = resetNow;

        bool kickNow = (params[KICK_BUTTON].getValue() > 0.5f) ||
                       (inputs[KICK_INPUT].getVoltage() > 1.0f);
        if (kickNow && !lastKickState) kick();
        lastKickState = kickNow;

        double dt = args.sampleTime * speedFactor;
        dt = std::min(dt, 0.001);
        rk4(dt);

        if (!isValid()) onReset();

        // Detect zero crossings and generate random voltages + gate triggers
        double currentSign1 = (theta1 >= 0) ? 1.0 : -1.0;
        double currentSign2 = (theta2 >= 0) ? 1.0 : -1.0;

        if (currentSign1 != prevTheta1Sign && std::abs(theta1) > 1e-6) {
            generateRandomVoct1();
            double durSec = getGateDurationSeconds(gate1Duration);
            gate1Timer = static_cast<int>(args.sampleRate * durSec);
            flash1 = 1.f;
        }
        if (currentSign2 != prevTheta2Sign && std::abs(theta2) > 1e-6) {
            generateRandomVoct2();
            double durSec = getGateDurationSeconds(gate2Duration);
            gate2Timer = static_cast<int>(args.sampleRate * durSec);
            flash2 = 1.f;
        }

        prevTheta1Sign = currentSign1;
        prevTheta2Sign = currentSign2;

        // Decrement gate timers
        if (gate1Timer > 0) gate1Timer--;
        if (gate2Timer > 0) gate2Timer--;

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
        if (trailTimer >= trailStep) {
            trailTimer -= trailStep;
            if (trailTimer > trailStep) trailTimer = 0.f;
            float reach = (float)std::max(len1 + len2, 0.1);
            trailX1[trailHead] = (float)x1 / reach;
            trailY1[trailHead] = (float)y1 / reach;
            trailX2[trailHead] = (float)x2 / reach;
            trailY2[trailHead] = (float)y2 / reach;
            trailHead = (trailHead + 1) % PL_TRAIL_N;
            if (trailCount < PL_TRAIL_N) trailCount++;
        }
    }

    // JSON serialisation
    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "voct1Range", json_integer(voct1Range));
        json_object_set_new(root, "voct2Range", json_integer(voct2Range));
        json_object_set_new(root, "gate1Duration", json_integer(gate1Duration));
        json_object_set_new(root, "gate2Duration", json_integer(gate2Duration));
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* j;
        j = json_object_get(root, "voct1Range");
        if (j) voct1Range = (VoctRange)json_integer_value(j);
        j = json_object_get(root, "voct2Range");
        if (j) voct2Range = (VoctRange)json_integer_value(j);
        j = json_object_get(root, "gate1Duration");
        if (j) gate1Duration = (GateDuration)json_integer_value(j);
        j = json_object_get(root, "gate2Duration");
        if (j) gate2Duration = (GateDuration)json_integer_value(j);
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
NVGcolor colP1()       { return nvgRGB(0x3d, 0xe0, 0xff); }   // pendulum 1: cyan
NVGcolor colP2()       { return nvgRGB(0xb0, 0x5a, 0xff); }   // pendulum 2: violet

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

// Cyan / violet LEDs (Rack has no built-in ones)
template <typename TBase = GrayModuleLightWidget>
struct TPLCyanLight : TBase {
    TPLCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using PLCyanLight = TPLCyanLight<>;

template <typename TBase = GrayModuleLightWidget>
struct TPLVioletLight : TBase {
    TPLVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
};
using PLVioletLight = TPLVioletLight<>;

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
// Physical pendulum parts
// ------------------------------------------------------------

// Metal rod: dark body, steel core, highlight toward the light (up-left)
void drawRod(NVGcontext* vg, float x0, float y0, float x1, float y1, float w, NVGcolor steel) {
    float dx = x1 - x0, dy = y1 - y0;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.5f) return;
    float nx = -dy / len, ny = dx / len;
    if (nx * -0.7f + ny * -0.7f < 0.f) { nx = -nx; ny = -ny; }

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

    nvgBeginPath(vg);
    nvgMoveTo(vg, x0 + nx * w * 0.22f, y0 + ny * w * 0.22f);
    nvgLineTo(vg, x1 + nx * w * 0.22f, y1 + ny * w * 0.22f);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 150));
    nvgStrokeWidth(vg, w * 0.26f);
    nvgStroke(vg);
}

// Shaded sphere with specular highlight and rim
void drawBob(NVGcontext* vg, float cx, float cy, float r, NVGcolor hi, NVGcolor base, NVGcolor edge) {
    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, r);
    nvgFillPaint(vg, nvgRadialGradient(vg, cx - r * 0.35f, cy - r * 0.4f, r * 0.1f, r * 1.25f, hi, base));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, r - 0.5f);
    nvgStrokeColor(vg, edge);
    nvgStrokeWidth(vg, 1.3f);
    nvgStroke(vg);

    // Bounce light on the lower right
    nvgBeginPath(vg);
    nvgCircle(vg, cx + r * 0.38f, cy + r * 0.42f, r * 0.3f);
    nvgFillColor(vg, nvgRGBA(255, 255, 255, 22));
    nvgFill(vg);

    // Specular
    nvgBeginPath(vg);
    nvgCircle(vg, cx - r * 0.38f, cy - r * 0.42f, r * 0.2f);
    nvgFillColor(vg, nvgRGBA(255, 255, 255, 210));
    nvgFill(vg);
}

// Pivot bearing: mounting disc, bearing ring, cap with a screw slot
void drawHub(NVGcontext* vg, float cx, float cy) {
    // Shadow
    nvgBeginPath(vg);
    nvgCircle(vg, cx + 2.f, cy + 3.f, 12.f);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
    nvgFill(vg);

    // Mounting disc
    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, 12.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, cx, cy - 12.f, cx, cy + 12.f,
                                       nvgRGB(0x3a, 0x41, 0x4c), nvgRGB(0x12, 0x15, 0x1a)));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 50));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);

    // Four tiny mounting screws
    for (int i = 0; i < 4; ++i) {
        float a = (float)M_PI * 0.25f + i * (float)M_PI * 0.5f;
        nvgBeginPath(vg);
        nvgCircle(vg, cx + std::cos(a) * 9.f, cy + std::sin(a) * 9.f, 1.1f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 90));
        nvgFill(vg);
    }

    // Bearing ring
    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, 6.5f);
    nvgFillPaint(vg, nvgRadialGradient(vg, cx - 2.f, cy - 2.f, 1.f, 8.f,
                                       nvgRGB(0xe4, 0xea, 0xf2), nvgRGB(0x4b, 0x54, 0x62)));
    nvgFill(vg);

    // Cap
    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, 3.6f);
    nvgFillPaint(vg, nvgRadialGradient(vg, cx - 1.f, cy - 1.f, 0.5f, 4.5f,
                                       nvgRGB(0xf6, 0xf9, 0xfd), nvgRGB(0x6b, 0x75, 0x84)));
    nvgFill(vg);

    // Slot
    nvgBeginPath(vg);
    nvgMoveTo(vg, cx - 2.4f, cy + 0.6f);
    nvgLineTo(vg, cx + 2.4f, cy - 0.6f);
    nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 160));
    nvgStrokeWidth(vg, 0.9f);
    nvgStroke(vg);
}

// Fading motion trail (older = thinner and fainter)
void drawTrail(NVGcontext* vg, const float* xs, const float* ys, int n, NVGcolor col, float width, float alpha) {
    if (n < 2) return;
    nvgLineCap(vg, NVG_ROUND);
    nvgLineJoin(vg, NVG_ROUND);
    const int K = 4;
    for (int s = 0; s < n - 1; s += K) {
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
struct PLPanelBackground : TransparentWidget {
    PLPanelBackground() { box.size = Vec(PL_PANEL_W, PL_PANEL_H); }

    void label(NVGcontext* vg, float x, float y, const char* txt) {
        drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
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
        NVGcolor lc = col;
        lc.a = 0.35f;
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y + 8.f);
        nvgLineTo(vg, x2, y + 8.f);
        nvgStrokeColor(vg, lc);
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Full-width divider with a centered caption
    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = PL_PANEL_W - 14.f, cx = PL_PANEL_W / 2.f;
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

    void jackCell(NVGcontext* vg, float cx, float cy, const char* txt) {
        plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
        label(vg, cx, cy - 19.f, txt);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PL_PANEL_W, PL_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PL_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) PL_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, PL_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, PL_PANEL_W - 1.f, PL_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, PL_PANEL_W / 2.f, 13.f);
        drawText(vg, PL_PANEL_W / 2.f, 31.f, "PSYLFO", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = PL_DISP_X - 5.f, by = PL_DISP_Y - 5.f;
        float bw = PL_DISP_W + 10.f, bh = PL_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // Side blocks
        caption(vg, 52.f, 54.f, "PENDULUM 1", colP1(), 12.f, 92.f);
        caption(vg, 368.f, 54.f, "PENDULUM 2", colP2(), 328.f, 408.f);

        for (int c = 0; c < 2; ++c) {
            ring(vg, PL_LCOL[c], PL_KNOB_Y, 18.f);
            ring(vg, PL_RCOL[c], PL_KNOB_Y, 18.f);
            label(vg, PL_LCOL[c], PL_KNOB_Y + 26.f, c == 0 ? "MASS" : "LENGTH");
            label(vg, PL_RCOL[c], PL_KNOB_Y + 26.f, c == 0 ? "MASS" : "LENGTH");
        }

        jackCell(vg, PL_LCOL[0], PL_OUT_Y[0], "X");
        jackCell(vg, PL_LCOL[1], PL_OUT_Y[0], "Y");
        jackCell(vg, PL_LCOL[0], PL_OUT_Y[1], "V/OCT");
        jackCell(vg, PL_LCOL[1], PL_OUT_Y[1], "GATE");
        jackCell(vg, PL_RCOL[0], PL_OUT_Y[0], "X");
        jackCell(vg, PL_RCOL[1], PL_OUT_Y[0], "Y");
        jackCell(vg, PL_RCOL[0], PL_OUT_Y[1], "V/OCT");
        jackCell(vg, PL_RCOL[1], PL_OUT_Y[1], "GATE");

        // Global controls
        section(vg, 270.f, "CONTROL");

        const float labelY = PL_CTRL_Y + 29.f;
        // Knobs
        ring(vg, PL_SPEED_X, PL_CTRL_Y, 20.f);
        ring(vg, PL_GRAV_X, PL_CTRL_Y, 20.f);
        ring(vg, PL_DAMP_X, PL_CTRL_Y, 20.f);
        label(vg, PL_SPEED_X, labelY, "SPEED");
        label(vg, PL_GRAV_X, labelY, "GRAVITY");
        label(vg, PL_DAMP_X, labelY, "DAMPING");
        // Jacks
        const float jx[3] = {PL_SPDCV_X, PL_KICKI_X, PL_RSTI_X};
        const char* jl[3] = {"SPD CV", "KICK IN", "RESET IN"};
        for (int i = 0; i < 3; ++i) {
            plate(vg, jx[i] - 17.f, PL_CTRL_Y - 19.f, 34.f, 38.f);
            label(vg, jx[i], labelY, jl[i]);
        }
        // Buttons
        ring(vg, PL_KICKB_X, PL_CTRL_Y, 17.f);
        ring(vg, PL_RSTB_X, PL_CTRL_Y, 17.f);
        label(vg, PL_KICKB_X, labelY, "KICK");
        label(vg, PL_RSTB_X, labelY, "RESET");

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Pendulum display
// ------------------------------------------------------------
struct PLDisplayWidget : TransparentWidget {
    PsyLFO* module = nullptr;

    PLDisplayWidget(PsyLFO* m) : module(m) {
        box.pos = Vec(PL_DISP_X, PL_DISP_Y);
        box.size = Vec(PL_DISP_W, PL_DISP_H);
    }

    // Static background: screen, dial face, degree ring, reference lines
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float cx = PL_DISP_W / 2.f, cy = PL_DISP_H / 2.f;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PL_DISP_W, PL_DISP_H);
        nvgFillColor(vg, nvgRGB(0x03, 0x05, 0x08));
        nvgFill(vg);

        // Dial face
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, 97.f);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy - 10.f, 8.f, 100.f,
                                           nvgRGB(0x0d, 0x1a, 0x24), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 46));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Concentric range rings
        for (int k = 1; k <= 2; ++k) {
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
        for (int i = 0; i < 36; ++i) {
            if (i % 3 == 0) continue;
            float a = i * 10.f * (float)M_PI / 180.f;
            float dx = std::sin(a), dy = std::cos(a);
            nvgMoveTo(vg, cx + dx * 94.f, cy + dy * 94.f);
            nvgLineTo(vg, cx + dx * 90.f, cy + dy * 90.f);
        }
        nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 60));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        for (int i = 0; i < 36; i += 3) {
            if (i == 0) continue;
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
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 220));
        nvgStrokeWidth(vg, 2.f);
        nvgStroke(vg);

        // Dashed plumb line (the vertical reference)
        nvgBeginPath(vg);
        for (float y = 14.f; y < 80.f; y += 6.f) {
            nvgMoveTo(vg, cx + 0.5f, cy + y);
            nvgLineTo(vg, cx + 0.5f, cy + y + 3.f);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
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
        const float cx = PL_DISP_W / 2.f, cy = PL_DISP_H / 2.f;
        const float Rpx = 66.f;   // pixel radius of the full reach

        // ---- state (live, or a sample pose in the module browser) ----
        float th1, th2, w1, w2, l1, l2, m1, m2, fl1 = 0.f, fl2 = 0.f, grav;
        float t1x[PL_TRAIL_N], t1y[PL_TRAIL_N], t2x[PL_TRAIL_N], t2y[PL_TRAIL_N];
        int tn = 0;

        if (module) {
            th1 = (float) module->theta1;  th2 = (float) module->theta2;
            w1 = (float) module->omega1;   w2 = (float) module->omega2;
            l1 = (float) module->len1;     l2 = (float) module->len2;
            m1 = (float) module->mass1;    m2 = (float) module->mass2;
            fl1 = module->flash1;          fl2 = module->flash2;
            grav = (float) module->g;
        } else {
            th1 = 0.7f;  th2 = 2.2f;
            w1 = 3.f;    w2 = -5.f;
            l1 = 1.2f;   l2 = 1.f;
            m1 = 2.f;    m2 = 1.f;
            grav = 9.81f;
        }

        float reach = std::max(l1 + l2, 0.1f);
        float scale = Rpx / reach;

        if (module) {
            int cnt = module->trailCount;
            int head = module->trailHead;
            tn = std::min(std::max(cnt, 0), PL_TRAIL_N);
            for (int i = 0; i < tn; ++i) {
                int idx = ((head - tn + i) % PL_TRAIL_N + PL_TRAIL_N) % PL_TRAIL_N;
                t1x[i] = cx + module->trailX1[idx] * Rpx;
                t1y[i] = cy - module->trailY1[idx] * Rpx;
                t2x[i] = cx + module->trailX2[idx] * Rpx;
                t2y[i] = cy - module->trailY2[idx] * Rpx;
            }
        } else {
            tn = PL_TRAIL_N;
            for (int i = 0; i < tn; ++i) {
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
        float e1 = std::min(std::fabs(w1) / 10.f, 1.f);
        float e2 = std::min(std::fabs(w2) / 10.f, 1.f);

        // ---- angle arcs (measured from the vertical) ----
        auto arcAt = [&](float ax, float ay, float r, float th, NVGcolor col) {
            if (std::fabs(th) < 0.02f) return;
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
        drawTrail(vg, t1x, t1y, tn, colP1(), 4.f, 0.10f);
        drawTrail(vg, t1x, t1y, tn, colP1(), 1.1f, 0.35f);
        drawTrail(vg, t2x, t2y, tn, colP2(), 6.f, 0.20f);
        drawTrail(vg, t2x, t2y, tn, nvgRGB(0xd8, 0xa8, 0xff), 1.8f, 0.95f);

        // ---- shadows ----
        const float sx = 2.5f, sy = 3.5f;
        nvgLineCap(vg, NVG_ROUND);
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + sx, cy + sy);
        nvgLineTo(vg, b1x + sx, b1y + sy);
        nvgLineTo(vg, b2x + sx, b2y + sy);
        nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgStrokeWidth(vg, 4.6f);
        nvgLineJoin(vg, NVG_ROUND);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgCircle(vg, b1x + sx + 1.f, b1y + sy + 1.f, r1);
        nvgCircle(vg, b2x + sx + 1.f, b2y + sy + 1.f, r2);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgFill(vg);

        // ---- rods ----
        drawRod(vg, cx, cy, b1x, b1y, 4.0f, nvgRGB(0x74, 0x9a, 0xab));
        drawRod(vg, b1x, b1y, b2x, b2y, 3.4f, nvgRGB(0x93, 0x84, 0xb0));

        // ---- halos + gate flashes ----
        auto halo = [&](float bx, float by, float r, NVGcolor col, float energy, float flash) {
            float a = 0.10f + 0.22f * energy + 0.35f * flash;
            nvgBeginPath(vg);
            nvgCircle(vg, bx, by, r * 3.f);
            nvgFillPaint(vg, nvgRadialGradient(vg, bx, by, r * 0.6f, r * 3.f,
                                               nvgRGBAf(col.r, col.g, col.b, a),
                                               nvgRGBAf(col.r, col.g, col.b, 0.f)));
            nvgFill(vg);
            if (flash > 0.02f) {
                nvgBeginPath(vg);
                nvgCircle(vg, bx, by, r + (1.f - flash) * 16.f);
                nvgStrokeColor(vg, nvgRGBAf(col.r, col.g, col.b, 0.8f * flash));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }
        };
        halo(b1x, b1y, r1, colP1(), e1, fl1);
        halo(b2x, b2y, r2, colP2(), e2, fl2);

        // ---- bobs ----
        drawBob(vg, b1x, b1y, r1, nvgRGB(0xd8, 0xf9, 0xff), nvgRGB(0x1f, 0x9f, 0xc0), nvgRGB(0x08, 0x40, 0x55));
        drawBob(vg, b2x, b2y, r2, nvgRGB(0xf1, 0xe3, 0xff), nvgRGB(0x7a, 0x3a, 0xc8), nvgRGB(0x2d, 0x10, 0x58));

        // ---- pivot ----
        drawHub(vg, cx, cy);

        // ---- scanlines ----
        nvgBeginPath(vg);
        for (int y = 0; y < (int) PL_DISP_H; y += 3)
            nvgRect(vg, 0, y + 1, PL_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
        nvgFill(vg);

        // ---- glass reflection ----
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, PL_DISP_W, PL_DISP_H * 0.35f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, PL_DISP_H * 0.35f,
                                           nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // ---- readouts (in the corners, outside the dial) ----
        char buf[32];
        float deg1 = th1 * 180.f / (float)M_PI;
        float deg2 = th2 * 180.f / (float)M_PI;
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
        NVGcolor c2 = nvgRGBA(0xb0, 0x5a, 0xff, 230);
        NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 220);

        snprintf(buf, sizeof(buf), "A1 %+4.0f", deg1);
        drawText(vg, 4.f, 8.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "W1 %+.1f", w1);
        drawText(vg, 4.f, 17.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "A2 %+4.0f", deg2);
        drawText(vg, PL_DISP_W - 4.f, 8.f, buf, 7.f, c2, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "W2 %+.1f", w2);
        drawText(vg, PL_DISP_W - 4.f, 17.f, buf, 7.f, c2, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "R %.2fM", reach);
        drawText(vg, 4.f, PL_DISP_H - 8.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "G %.1f", grav);
        drawText(vg, PL_DISP_W - 4.f, PL_DISP_H - 8.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct PsyLFOWidget : ModuleWidget
{
    PsyLFOWidget(PsyLFO *module)
    {
        setModule(module);
        box.size = Vec(PL_PANEL_W, PL_PANEL_H);

        addChild(new PLPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PLDisplayWidget(module));

        // Pendulum 1 (left)
        addChild(createLightCentered<MediumLight<PLCyanLight>>(Vec(52.f, 76.f), module, PsyLFO::MASS1_LIGHT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_LCOL[0], PL_KNOB_Y), module, PsyLFO::MASS1_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_LCOL[1], PL_KNOB_Y), module, PsyLFO::LEN1_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_LCOL[0], PL_OUT_Y[0]), module, PsyLFO::X1_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_LCOL[1], PL_OUT_Y[0]), module, PsyLFO::Y1_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_LCOL[0], PL_OUT_Y[1]), module, PsyLFO::VOCT1_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_LCOL[1], PL_OUT_Y[1]), module, PsyLFO::GATE1_OUTPUT));

        // Pendulum 2 (right)
        addChild(createLightCentered<MediumLight<PLVioletLight>>(Vec(368.f, 76.f), module, PsyLFO::MASS2_LIGHT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_RCOL[0], PL_KNOB_Y), module, PsyLFO::MASS2_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_RCOL[1], PL_KNOB_Y), module, PsyLFO::LEN2_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_RCOL[0], PL_OUT_Y[0]), module, PsyLFO::X2_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_RCOL[1], PL_OUT_Y[0]), module, PsyLFO::Y2_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_RCOL[0], PL_OUT_Y[1]), module, PsyLFO::VOCT2_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PL_RCOL[1], PL_OUT_Y[1]), module, PsyLFO::GATE2_OUTPUT));

        // Global controls
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_SPEED_X, PL_CTRL_Y), module, PsyLFO::SPEED_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PL_SPDCV_X, PL_CTRL_Y), module, PsyLFO::SPEED_CV_INPUT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_GRAV_X, PL_CTRL_Y), module, PsyLFO::GRAVITY_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PL_DAMP_X, PL_CTRL_Y), module, PsyLFO::DAMPING_PARAM));
        addParam(createParamCentered<VCVButton>(Vec(PL_KICKB_X, PL_CTRL_Y), module, PsyLFO::KICK_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PL_KICKI_X, PL_CTRL_Y), module, PsyLFO::KICK_INPUT));
        addParam(createParamCentered<VCVButton>(Vec(PL_RSTB_X, PL_CTRL_Y), module, PsyLFO::RESET_BUTTON));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PL_RSTI_X, PL_CTRL_Y), module, PsyLFO::RESET_CV_INPUT));
    }

    void appendContextMenu(ui::Menu* menu) override {
        PsyLFO* module = dynamic_cast<PsyLFO*>(this->module);
        if (!module) return;

        menu->addChild(new ui::MenuSeparator);

        // Voct1 range submenu
        menu->addChild(createSubmenuItem("Voct1 range", "",
            [=](ui::Menu* submenu) {
                const char* rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                for (int i = 0; i < PsyLFO::NUM_RANGES; ++i) {
                    submenu->addChild(createMenuItem(rangeNames[i],
                        (module->voct1Range == i) ? "✔" : "",
                        [=]() {
                            module->voct1Range = (PsyLFO::VoctRange)i;
                            module->generateRandomVoct1();
                        }));
                }
            }
        ));

        // Voct2 range submenu
        menu->addChild(createSubmenuItem("Voct2 range", "",
            [=](ui::Menu* submenu) {
                const char* rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                for (int i = 0; i < PsyLFO::NUM_RANGES; ++i) {
                    submenu->addChild(createMenuItem(rangeNames[i],
                        (module->voct2Range == i) ? "✔" : "",
                        [=]() {
                            module->voct2Range = (PsyLFO::VoctRange)i;
                            module->generateRandomVoct2();
                        }));
                }
            }
        ));

        // Gate1 duration submenu
        menu->addChild(createSubmenuItem("Gate1 duration", "",
            [=](ui::Menu* submenu) {
                const char* gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                for (int i = 0; i < PsyLFO::NUM_GATE_DURS; ++i) {
                    submenu->addChild(createMenuItem(gateNames[i],
                        (module->gate1Duration == i) ? "✔" : "",
                        [=]() {
                            module->gate1Duration = (PsyLFO::GateDuration)i;
                        }));
                }
            }
        ));

        // Gate2 duration submenu
        menu->addChild(createSubmenuItem("Gate2 duration", "",
            [=](ui::Menu* submenu) {
                const char* gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                for (int i = 0; i < PsyLFO::NUM_GATE_DURS; ++i) {
                    submenu->addChild(createMenuItem(gateNames[i],
                        (module->gate2Duration == i) ? "✔" : "",
                        [=]() {
                            module->gate2Duration = (PsyLFO::GateDuration)i;
                        }));
                }
            }
        ));
    }
};

Model* modelPsyLFO = createModel<PsyLFO, PsyLFOWidget>("PsyLFO");