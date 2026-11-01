#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <random>
#include <limits>
#include <memory>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   SP_TRAIL_N  = 120;      // trail points
static const float SP_TRAIL_HZ = 150.f;    // trail sample rate

static const float SP_PANEL_W = RACK_GRID_WIDTH * 24;   // 360
static const float SP_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float SP_DISP_W = 180.f;
static const float SP_DISP_H = 180.f;
static const float SP_DISP_X = (SP_PANEL_W - SP_DISP_W) / 2.f;   // 90
static const float SP_DISP_Y = 44.f;

// Side knob columns
static const float SP_LKNOB_X = 32.f;
static const float SP_RKNOB_X = 328.f;

// Control row
static const float SP_CTRL_SECTION_Y = 228.f;
static const float SP_CTRL_Y         = 256.f;
static const float SP_CTRL_LBL_Y     = 282.f;

// I/O row
static const float SP_IO_SECTION_Y   = 302.f;
static const float SP_IO_LBL_Y       = 324.f;
static const float SP_IO_Y           = 346.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct SpringPendulum : Module {
    enum ParamId {
        SPEED_PARAM,
        MASS_PARAM,
        LENGTH_PARAM,
        SPRING_K_PARAM,
        RESET_BUTTON,
        PUSH_STRENGTH_PARAM,
        PUSH_BUTTON,
        PARAMS_LEN
    };
    enum InputId {
        RESET_CV_INPUT,
        PUSH_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        X_OUTPUT,
        Y_OUTPUT,
        VOCT_OUTPUT,
        GATE_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        MASS_LIGHT,
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
    // Gate duration selection
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
    // Gravitational acceleration presets
    enum GPreset {
        G_EARTH,
        G_2X,
        G_4X,
        G_ZERO,
        NUM_GPRESETS
    };

    // State variables
    double x = 0.0, y = 0.0;
    double vx = 0.0, vy = 0.0;
    double mass = 1.0;
    double restLen = 1.0;
    double springK = 10.0;
    double speedFactor = 1.0;
    double g = 9.81;

    // Voct & gate state
    double voctVoltage = 0.0;
    double prevThetaSign = 1.0;
    int gateTimer = 0;

    // User settings
    VoctRange voctRange = RANGE_N1_1;
    GateDuration gateDuration = GATE_5MS;
    GPreset gPreset = G_EARTH;

    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger pushTrigger;
    bool lastResetState = false;
    std::mt19937 rng;

    // Display data
    float trailX[SP_TRAIL_N] = {};
    float trailY[SP_TRAIL_N] = {};
    int   trailHead = 0;
    int   trailCount = 0;
    float trailTimer = 0.f;

    SpringPendulum() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(SPEED_PARAM, 0.f, 1.f, 0.5f, "Speed", " Hz", 0.f, 10.f, 0.1f);
        configParam(MASS_PARAM, 0.1f, 10.f, 1.f, "Mass", " kg");
        configParam(LENGTH_PARAM, 0.2f, 3.f, 1.f, "Rest length", " m");
        configParam(SPRING_K_PARAM, 1.f, 100.f, 10.f, "Spring constant", " N/m");
        configParam(RESET_BUTTON, 0.f, 1.f, 0.f, "Reset");
        configParam(PUSH_STRENGTH_PARAM, 0.f, 10.f, 2.f, "Push strength", " m/s");
        configParam(PUSH_BUTTON, 0.f, 1.f, 0.f, "Random push");

        configInput(RESET_CV_INPUT, "Reset gate");
        configInput(PUSH_CV_INPUT, "Random push gate");

        configOutput(X_OUTPUT, "Mass X");
        configOutput(Y_OUTPUT, "Mass Y");
        configOutput(VOCT_OUTPUT, "V/Oct (random on theta=0)");
        configOutput(GATE_OUTPUT, "Gate (trigger on Voct change)");

        std::random_device rd;
        rng.seed(rd());
        onReset();
        generateRandomVoct();
        setGPreset();
    }

    void onReset() override {
        std::uniform_real_distribution<double> angleDist(-M_PI, M_PI);
        std::uniform_real_distribution<double> lenOffset(0.8, 1.2);
        double angle = angleDist(rng);
        double r = restLen * lenOffset(rng);
        x = r * sin(angle);
        y = -r * cos(angle);
        vx = 0.0;
        vy = 0.0;

        double theta = atan2(x, -y);
        prevThetaSign = (theta >= 0) ? 1.0 : -1.0;
        gateTimer = 0;

        trailHead = 0;
        trailCount = 0;
        trailTimer = 0.f;
    }

    void setGPreset() {
        switch (gPreset) {
            case G_EARTH: g = 9.81; break;
            case G_2X:    g = 19.62; break;
            case G_4X:    g = 39.24; break;
            case G_ZERO:  g = 0.0; break;
            default:      g = 9.81; break;
        }
    }

    double scaleVoct(double raw, VoctRange range) {
        switch (range) {
            case RANGE_0_1:   return (raw + 1.0) / 2.0;
            case RANGE_N1_1:  return raw;
            case RANGE_N2_2:  return raw * 2.0;
            case RANGE_N5_5:  return raw * 5.0;
            case RANGE_0_5:   return (raw + 1.0) / 2.0 * 5.0;
            default: return raw;
        }
    }

    void generateRandomVoct() {
        std::uniform_real_distribution<double> randRaw(-1.0, 1.0);
        double raw = randRaw(rng);
        voctVoltage = scaleVoct(raw, voctRange);
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
        float speedKnob = params[SPEED_PARAM].getValue();
        speedFactor = 0.2f + speedKnob * 9.8f;
        mass = params[MASS_PARAM].getValue();
        restLen = params[LENGTH_PARAM].getValue();
        springK = params[SPRING_K_PARAM].getValue();
    }

    void computeAcceleration(double px, double py, double& ax, double& ay) {
        double r = sqrt(px*px + py*py);
        if (r < 1e-6) r = 1e-6;
        double springForceMag = springK * (r - restLen);
        double fx_spring = -springForceMag * (px / r);
        double fy_spring = -springForceMag * (py / r);
        double fx = fx_spring;
        double fy = fy_spring - mass * g;
        ax = fx / mass;
        ay = fy / mass;
    }

    void rk4(double dt) {
        double k1_x = vx;
        double k1_y = vy;
        double a1_x, a1_y;
        computeAcceleration(x, y, a1_x, a1_y);
        double k1_vx = a1_x;
        double k1_vy = a1_y;

        double x2 = x + 0.5*dt * k1_x;
        double y2 = y + 0.5*dt * k1_y;
        double vx2 = vx + 0.5*dt * k1_vx;
        double vy2 = vy + 0.5*dt * k1_vy;
        double a2_x, a2_y;
        computeAcceleration(x2, y2, a2_x, a2_y);
        double k2_x = vx2;
        double k2_y = vy2;
        double k2_vx = a2_x;
        double k2_vy = a2_y;

        double x3 = x + 0.5*dt * k2_x;
        double y3 = y + 0.5*dt * k2_y;
        double vx3 = vx + 0.5*dt * k2_vx;
        double vy3 = vy + 0.5*dt * k2_vy;
        double a3_x, a3_y;
        computeAcceleration(x3, y3, a3_x, a3_y);
        double k3_x = vx3;
        double k3_y = vy3;
        double k3_vx = a3_x;
        double k3_vy = a3_y;

        double x4 = x + dt * k3_x;
        double y4 = y + dt * k3_y;
        double vx4 = vx + dt * k3_vx;
        double vy4 = vy + dt * k3_vy;
        double a4_x, a4_y;
        computeAcceleration(x4, y4, a4_x, a4_y);
        double k4_x = vx4;
        double k4_y = vy4;
        double k4_vx = a4_x;
        double k4_vy = a4_y;

        x += dt * (k1_x + 2.0*k2_x + 2.0*k3_x + k4_x) / 6.0;
        y += dt * (k1_y + 2.0*k2_y + 2.0*k3_y + k4_y) / 6.0;
        vx += dt * (k1_vx + 2.0*k2_vx + 2.0*k3_vx + k4_vx) / 6.0;
        vy += dt * (k1_vy + 2.0*k2_vy + 2.0*k3_vy + k4_vy) / 6.0;
    }

    void process(const ProcessArgs& args) override {
        updateParameters();

        bool resetNow = (params[RESET_BUTTON].getValue() > 0.5f) ||
                        (inputs[RESET_CV_INPUT].getVoltage() > 1.0f);
        if (resetNow && !lastResetState) onReset();
        lastResetState = resetNow;

        bool pushNow = (params[PUSH_BUTTON].getValue() > 0.5f) ||
                       (inputs[PUSH_CV_INPUT].getVoltage() > 1.0f);
        if (pushTrigger.process(pushNow)) {
            float strength = params[PUSH_STRENGTH_PARAM].getValue();
            std::uniform_real_distribution<double> angleDist(0.0, 2.0 * M_PI);
            double angle = angleDist(rng);
            double dv = strength;
            vx += dv * cos(angle);
            vy += dv * sin(angle);
        }

        double dt = args.sampleTime * speedFactor;
        dt = std::min(dt, 0.001);
        rk4(dt);

        if (std::abs(x) > 10.0 || std::abs(y) > 10.0) onReset();

        double theta = atan2(x, -y);
        double currentSign = (theta >= 0) ? 1.0 : -1.0;
        if (currentSign != prevThetaSign && std::abs(theta) > 1e-6 && y < 0.0) {
            generateRandomVoct();
            double durSec = getGateDurationSeconds(gateDuration);
            gateTimer = static_cast<int>(args.sampleRate * durSec);
        }
        prevThetaSign = currentSign;

        if (gateTimer > 0) gateTimer--;

        outputs[X_OUTPUT].setVoltage(rack::math::clamp((float)x, -5.0f, 5.0f));
        outputs[Y_OUTPUT].setVoltage(rack::math::clamp((float)y, -5.0f, 5.0f));
        outputs[VOCT_OUTPUT].setVoltage((float)voctVoltage);
        outputs[GATE_OUTPUT].setVoltage(gateTimer > 0 ? 10.0f : 0.0f);

        float speed = std::sqrt(vx*vx + vy*vy);
        float bright = std::min(1.0f, speed / 5.0f);
        lights[MASS_LIGHT].setBrightness(bright);

        // ---- display trail ----
        const float trailStep = 1.f / SP_TRAIL_HZ;
        trailTimer += args.sampleTime;
        if (trailTimer >= trailStep) {
            trailTimer -= trailStep;
            if (trailTimer > trailStep) trailTimer = 0.f;
            float reach = (float)std::max(restLen * 1.6, 0.5);
            trailX[trailHead] = (float)x / reach;
            trailY[trailHead] = (float)y / reach;
            trailHead = (trailHead + 1) % SP_TRAIL_N;
            if (trailCount < SP_TRAIL_N) trailCount++;
        }
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "voctRange", json_integer(voctRange));
        json_object_set_new(root, "gateDuration", json_integer(gateDuration));
        json_object_set_new(root, "gPreset", json_integer(gPreset));
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* j;
        j = json_object_get(root, "voctRange");
        if (j) voctRange = (VoctRange)json_integer_value(j);
        j = json_object_get(root, "gateDuration");
        if (j) gateDuration = (GateDuration)json_integer_value(j);
        j = json_object_get(root, "gPreset");
        if (j) {
            gPreset = (GPreset)json_integer_value(j);
            setGPreset();
        }
    }
};

// ------------------------------------------------------------
// Theme + drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colSpring()   { return nvgRGB(0x3d, 0xe0, 0xff); }
//NVGcolor colMass()     { return nvgRGB(0xff, 0x9c, 0x6a); }

void setFont(NVGcontext* vg) {
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font) nvgFontFaceId(vg, font->handle);
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

// Custom light for the mass LED — cyan-ish like the rest of the family
template <typename TBase = GrayModuleLightWidget>
struct TSPCyanLight : TBase {
    TSPCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using SPCyanLight = TSPCyanLight<>;

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
// Spring physics drawings
// ------------------------------------------------------------

// Coil spring: zigzag with dark border, steel core, white highlight
void drawSpring(NVGcontext* vg, float x0, float y0, float x1, float y1,
                int coils, float amp, NVGcolor steel) {
    float dx = x1 - x0, dy = y1 - y0;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.f) return;

    float ux = dx / len, uy = dy / len;
    float nx = -uy, ny = ux;

    auto zigzag = [&](float ampScale, float width, NVGcolor col) {
        nvgBeginPath(vg);
        nvgMoveTo(vg, x0, y0);
        int steps = coils * 2;
        for (int i = 1; i < steps; ++i) {
            float t = (float)i / steps;
            float side = (i % 2 == 0) ? -1.f : 1.f;
            float px = x0 + ux * len * t + nx * amp * side * ampScale;
            float py = y0 + uy * len * t + ny * amp * side * ampScale;
            nvgLineTo(vg, px, py);
        }
        nvgLineTo(vg, x1, y1);
        nvgStrokeColor(vg, col);
        nvgStrokeWidth(vg, width);
        nvgLineCap(vg, NVG_ROUND);
        nvgLineJoin(vg, NVG_ROUND);
        nvgStroke(vg);
    };

    zigzag(1.0f, 4.6f, nvgRGB(0x14, 0x17, 0x1d));
    zigzag(1.0f, 2.6f, steel);
    zigzag(0.75f, 0.8f, nvgRGBA(255, 255, 255, 150));
}

// Shaded sphere (mass) with highlight and rim
void drawBob(NVGcontext* vg, float cx, float cy, float r,
             NVGcolor hi, NVGcolor base, NVGcolor edge) {
    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, r);
    nvgFillPaint(vg, nvgRadialGradient(vg, cx - r * 0.35f, cy - r * 0.4f, r * 0.1f, r * 1.25f, hi, base));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, r - 0.5f);
    nvgStrokeColor(vg, edge);
    nvgStrokeWidth(vg, 1.3f);
    nvgStroke(vg);

    nvgBeginPath(vg);
    nvgCircle(vg, cx + r * 0.38f, cy + r * 0.42f, r * 0.3f);
    nvgFillColor(vg, nvgRGBA(255, 255, 255, 22));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgCircle(vg, cx - r * 0.38f, cy - r * 0.42f, r * 0.2f);
    nvgFillColor(vg, nvgRGBA(255, 255, 255, 210));
    nvgFill(vg);
}

// Machined pivot: mounting disc, bearing ring, cap with screw slot
void drawHub(NVGcontext* vg, float cx, float cy) {
    nvgBeginPath(vg);
    nvgCircle(vg, cx + 2.f, cy + 3.f, 12.f);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, 12.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, cx, cy - 12.f, cx, cy + 12.f,
                                       nvgRGB(0x3a, 0x41, 0x4c), nvgRGB(0x12, 0x15, 0x1a)));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 50));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);

    for (int i = 0; i < 4; ++i) {
        float a = (float)M_PI * 0.25f + i * (float)M_PI * 0.5f;
        nvgBeginPath(vg);
        nvgCircle(vg, cx + std::cos(a) * 9.f, cy + std::sin(a) * 9.f, 1.1f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 90));
        nvgFill(vg);
    }

    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, 6.5f);
    nvgFillPaint(vg, nvgRadialGradient(vg, cx - 2.f, cy - 2.f, 1.f, 8.f,
                                       nvgRGB(0xe4, 0xea, 0xf2), nvgRGB(0x4b, 0x54, 0x62)));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, 3.6f);
    nvgFillPaint(vg, nvgRadialGradient(vg, cx - 1.f, cy - 1.f, 0.5f, 4.5f,
                                       nvgRGB(0xf6, 0xf9, 0xfd), nvgRGB(0x6b, 0x75, 0x84)));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgMoveTo(vg, cx - 2.4f, cy + 0.6f);
    nvgLineTo(vg, cx + 2.4f, cy - 0.6f);
    nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 160));
    nvgStrokeWidth(vg, 0.9f);
    nvgStroke(vg);
}

// Fading motion trail
void drawTrail(NVGcontext* vg, const float* xs, const float* ys, int n,
               NVGcolor col, float width, float alpha) {
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
struct SPPanelBackground : TransparentWidget {
    SPPanelBackground() { box.size = Vec(SP_PANEL_W, SP_PANEL_H); }

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

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = SP_PANEL_W - 14.f, cx = SP_PANEL_W / 2.f;
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
        nvgRect(vg, 0, 0, SP_PANEL_W, SP_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SP_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) SP_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, SP_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, SP_PANEL_W - 1.f, SP_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, SP_PANEL_W / 2.f, 13.f);
        drawText(vg, SP_PANEL_W / 2.f, 31.f, "SPRING PENDULUM", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // Display bezel
        float bx = SP_DISP_X - 5.f, by = SP_DISP_Y - 5.f;
        float bw = SP_DISP_W + 10.f, bh = SP_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // Side knob rings + labels
        ring(vg, SP_LKNOB_X, 95.f, 18.f);
        ring(vg, SP_LKNOB_X, 158.f, 18.f);
        ring(vg, SP_RKNOB_X, 95.f, 18.f);
        ring(vg, SP_RKNOB_X, 158.f, 18.f);

        label(vg, SP_LKNOB_X, 95.f + 27.f, "SPEED");
        label(vg, SP_LKNOB_X, 158.f + 27.f, "MASS");
        label(vg, SP_RKNOB_X, 95.f + 27.f, "LENGTH");
        label(vg, SP_RKNOB_X, 158.f + 27.f, "SPRING K");

        // Control section
        section(vg, SP_CTRL_SECTION_Y, "CONTROL");
        ring(vg, 60.f, SP_CTRL_Y, 17.f);
        label(vg, 60.f, SP_CTRL_LBL_Y, "PUSH STR");

        // I/O section
        section(vg, SP_IO_SECTION_Y, "I / O");
        const char* io[6] = {"X", "Y", "V/OCT", "GATE", "RST IN", "PSH IN"};
        const float ioX[6] = {40.f, 96.f, 152.f, 208.f, 264.f, 320.f};
        for (int i = 0; i < 6; ++i) {
            jackCell(vg, ioX[i], SP_IO_Y, io[i]);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Spring display widget
// ------------------------------------------------------------
struct SPDisplayWidget : TransparentWidget {
    SpringPendulum* module = nullptr;

    SPDisplayWidget(SpringPendulum* m) : module(m) {
        box.pos = Vec(SP_DISP_X, SP_DISP_Y);
        box.size = Vec(SP_DISP_W, SP_DISP_H);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float cx = SP_DISP_W / 2.f;
        const float cy = SP_DISP_H / 2.f;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SP_DISP_W, SP_DISP_H);
        nvgFillColor(vg, nvgRGB(0x03, 0x05, 0x08));
        nvgFill(vg);

        // Dial face (radial glow)
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, SP_DISP_W / 2.f - 2.f);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, 10.f, SP_DISP_W * 0.55f,
                                           nvgRGB(0x0d, 0x1a, 0x24), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 46));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Range rings
        float rMax = SP_DISP_W / 2.f - 12.f;
        for (int k = 1; k <= 2; ++k) {
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, rMax * (float)k / 2.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, k == 2 ? 26 : 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Cross hair
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + 0.5f, cy - rMax + 4.f);
        nvgLineTo(vg, cx + 0.5f, cy + rMax - 4.f);
        nvgMoveTo(vg, cx - rMax + 4.f, cy + 0.5f);
        nvgLineTo(vg, cx + rMax - 4.f, cy + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
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
        const float cx = SP_DISP_W / 2.f;
        const float cy = SP_DISP_H / 2.f;
        const float Rpx = SP_DISP_W / 2.f - 12.f;

        // Read state
        double px = 0, py = -1, vx = 0, vy = 0, m = 1, K = 10, L = 1, g = 9.81;
        if (module) {
            px = module->x;      py = module->y;
            vx = module->vx;     vy = module->vy;
            m = module->mass;    K = module->springK;
            L = module->restLen; g = module->g;
        } else {
            px = 0.6 * std::sin(1.3);
            py = -1.0 * std::cos(1.3);
            vx = 2.0; vy = 0.5;
            m = 1.5; K = 20; L = 1.0; g = 9.81;
        }

        float reach = (float)std::max(L * 1.6, 0.5);
        float scale = Rpx / reach;

        // Mass position on screen
        float mx = cx + (float)px * scale;
        float my = cy + (float)py * scale * -1.f;  // y negative → below pivot

        // Trail
        int tn = 0;
        float tx[SP_TRAIL_N], ty[SP_TRAIL_N];
        if (module) {
            tn = std::min(std::max(module->trailCount, 0), SP_TRAIL_N);
            for (int i = 0; i < tn; ++i) {
                int idx = ((module->trailHead - tn + i) % SP_TRAIL_N + SP_TRAIL_N) % SP_TRAIL_N;
                tx[i] = cx + module->trailX[idx] * Rpx;
                ty[i] = cy - module->trailY[idx] * Rpx;
            }
        } else {
            tn = 90;
            for (int i = 0; i < tn; ++i) {
                float k = (float)(i - (tn - 1)) * 0.04f;
                float a1 = 1.3f + k * 0.7f;
                tx[i] = cx + (float)(L * std::sin(a1)) * scale * 1.4f;
                ty[i] = cy + (float)(L * std::cos(a1)) * scale * 1.4f;
            }
        }

        // Trail rendering (both wide halo and crisp inner line)
        drawTrail(vg, tx, ty, tn, colSpring(), 4.f, 0.10f);
        drawTrail(vg, tx, ty, tn, colSpring(), 1.1f, 0.4f);

        // Shadow of the spring (offset downward)
        nvgLineCap(vg, NVG_ROUND);
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + 2.5f, cy + 3.5f);
        nvgLineTo(vg, mx + 2.5f, my + 3.5f);
        nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 80));
        nvgStrokeWidth(vg, 5.f);
        nvgStroke(vg);

        // Spring coil — visual compression based on stretch
        float restLen = (float)L;
        float curLen = (float)std::sqrt(px*px + py*py);
        float stretch = curLen / std::max(restLen, 0.1f);
        int coils = (int)(10 + 4 * std::min(stretch, 1.6f));
        float coilAmp = 5.f;
        drawSpring(vg, cx, cy, mx, my, coils, coilAmp, colSpring());

        // Mass shadow on the ground (subtle)
        nvgBeginPath(vg);
        nvgEllipse(vg, mx + 3.f, my + 5.f, 9.f, 4.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 100));
        nvgFill(vg);

        // Mass radius proportional to cube root of mass
        float bobR = 6.f + 3.f * std::cbrt((float)m);

        // Energy glow (bright when moving fast)
        float speed = (float)std::sqrt(vx*vx + vy*vy);
        float energy = std::min(speed / 6.f, 1.f);
        nvgBeginPath(vg);
        nvgCircle(vg, mx, my, bobR * 3.f);
        nvgFillPaint(vg, nvgRadialGradient(vg, mx, my, bobR * 0.6f, bobR * 3.f,
                                           nvgRGBA(0xff, 0x9c, 0x6a, (int)(10 + 40 * energy)),
                                           nvgRGBA(0xff, 0x9c, 0x6a, 0)));
        nvgFill(vg);

        // Mass sphere
        drawBob(vg, mx, my, bobR,
                nvgRGB(0xff, 0xe4, 0xcc),
                nvgRGB(0xd0, 0x50, 0x1a),
                nvgRGB(0x4a, 0x14, 0x04));

        // Pivot
        drawHub(vg, cx, cy);

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) SP_DISP_H; y += 3)
            nvgRect(vg, 0, y + 1, SP_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SP_DISP_W, SP_DISP_H * 0.35f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SP_DISP_H * 0.35f,
                                           nvgRGBA(255, 255, 255, 14),
                                           nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[32];
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
        NVGcolor cm = nvgRGBA(0xff, 0x9c, 0x6a, 220);
        NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 220);

        snprintf(buf, sizeof(buf), "X %+.2f", (float)px);
        drawText(vg, 4.f, 8.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "Y %+.2f", (float)py);
        drawText(vg, 4.f, 17.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "VX %+.1f", (float)vx);
        drawText(vg, SP_DISP_W - 4.f, 8.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "VY %+.1f", (float)vy);
        drawText(vg, SP_DISP_W - 4.f, 17.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "K %.0f", (float)K);
        drawText(vg, 4.f, SP_DISP_H - 8.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "G %.1f", (float)g);
        drawText(vg, SP_DISP_W - 4.f, SP_DISP_H - 8.f, buf, 7.f, cd,
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct SpringPendulumWidget : ModuleWidget {
    SpringPendulumWidget(SpringPendulum* module) {
        setModule(module);
        box.size = Vec(SP_PANEL_W, SP_PANEL_H);

        addChild(new SPPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SPDisplayWidget(module));

        // Side knobs
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_LKNOB_X, 95.f), module, SpringPendulum::SPEED_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_LKNOB_X, 158.f), module, SpringPendulum::MASS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_RKNOB_X, 95.f), module, SpringPendulum::LENGTH_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_RKNOB_X, 158.f), module, SpringPendulum::SPRING_K_PARAM));

        // Control row
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(60.f, SP_CTRL_Y), module, SpringPendulum::PUSH_STRENGTH_PARAM));
        addParam(createParamCentered<LEDButton>(
            Vec(140.f, SP_CTRL_Y), module, SpringPendulum::PUSH_BUTTON));
        addParam(createParamCentered<LEDButton>(
            Vec(220.f, SP_CTRL_Y), module, SpringPendulum::RESET_BUTTON));
        addChild(createLightCentered<MediumLight<SPCyanLight>>(
            Vec(290.f, SP_CTRL_Y), module, SpringPendulum::MASS_LIGHT));

        // I/O row
        const float ioX[6] = {40.f, 96.f, 152.f, 208.f, 264.f, 320.f};
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(ioX[0], SP_IO_Y), module, SpringPendulum::X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(ioX[1], SP_IO_Y), module, SpringPendulum::Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(ioX[2], SP_IO_Y), module, SpringPendulum::VOCT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(ioX[3], SP_IO_Y), module, SpringPendulum::GATE_OUTPUT));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(ioX[4], SP_IO_Y), module, SpringPendulum::RESET_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(ioX[5], SP_IO_Y), module, SpringPendulum::PUSH_CV_INPUT));
    }

    void appendContextMenu(ui::Menu* menu) override {
        SpringPendulum* module = dynamic_cast<SpringPendulum*>(this->module);
        if (!module) return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Voct range", "",
            [=](ui::Menu* submenu) {
                const char* rangeNames[] = {"0 to 1", "-1 to 1", "-2 to 2", "-5 to 5", "0 to 5"};
                for (int i = 0; i < SpringPendulum::NUM_RANGES; ++i) {
                    submenu->addChild(createMenuItem(rangeNames[i],
                        (module->voctRange == i) ? "✔" : "",
                        [=]() {
                            module->voctRange = (SpringPendulum::VoctRange)i;
                            module->generateRandomVoct();
                        }));
                }
            }
        ));

        menu->addChild(createSubmenuItem("Gate duration", "",
            [=](ui::Menu* submenu) {
                const char* gateNames[] = {"Trigger (0.5 ms)", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms"};
                for (int i = 0; i < SpringPendulum::NUM_GATE_DURS; ++i) {
                    submenu->addChild(createMenuItem(gateNames[i],
                        (module->gateDuration == i) ? "✔" : "",
                        [=]() {
                            module->gateDuration = (SpringPendulum::GateDuration)i;
                        }));
                }
            }
        ));

        menu->addChild(createSubmenuItem("Gravitational acceleration", "",
            [=](ui::Menu* submenu) {
                const char* gNames[] = {"9.81 m/s² (Earth)", "2×g (19.62)", "4×g (39.24)", "0 (zero gravity)"};
                for (int i = 0; i < SpringPendulum::NUM_GPRESETS; ++i) {
                    submenu->addChild(createMenuItem(gNames[i],
                        (module->gPreset == i) ? "✔" : "",
                        [=]() {
                            module->gPreset = (SpringPendulum::GPreset)i;
                            module->setGPreset();
                        }));
                }
            }
        ));
    }
};

Model* modelSpringPendulum = createModel<SpringPendulum, SpringPendulumWidget>("SpringPendulum");