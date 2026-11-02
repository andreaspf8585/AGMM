#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <random>
#include <limits>
#include <memory>
#include <atomic>

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

// Dial geometry inside the display
static const float SP_DIAL_R = 78.f;
// Max distance of the mass centre from the dial centre (bob radius <= ~9.3, so it never touches the rim)
static const float SP_MAXPX  = 65.f;
// Soft-limit knee (fraction of SP_MAXPX where compression starts)
static const float SP_KNEE   = 0.85f;

// Physical geometry: the four springs are anchored at distance SP_ANCHOR_D from the centre
static const double SP_ANCHOR_D = 1.0;
static const float  SP_REACH    = 1.1f;   // display range, in units of SP_ANCHOR_D

// Side knob columns
static const float SP_LKNOB_X = 32.f;
static const float SP_RKNOB_X = 328.f;
static const float SP_KNOB_Y1 = 95.f;
static const float SP_KNOB_Y2 = 158.f;

// Control row
static const float SP_CTRL_SECTION_Y = 228.f;
static const float SP_CTRL_Y         = 256.f;
static const float SP_CTRL_LBL_Y     = 282.f;

// I/O row
static const float SP_IO_SECTION_Y   = 302.f;
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
        CHAOS_PARAM,
        DAMP_PARAM,
        GRAVITY_PARAM,
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

    struct State { double x, y, vx, vy; };

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
    std::atomic<bool>   grabbed{false};
    std::atomic<double> grabX{0.0};
    std::atomic<double> grabY{0.0};
    // Audio-thread copies used by the physics
    bool   handOn = false;
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
    int   trailHead = 0;
    int   trailCount = 0;
    float trailTimer = 0.f;

    SpringPendulum() {
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
        onReset();
        generateRandomVoct();
    }

    void onReset() override {
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
        chaos = params[CHAOS_PARAM].getValue();
        damping = params[DAMP_PARAM].getValue();
        g = params[GRAVITY_PARAM].getValue();
    }

    // Four springs: anchors at N (0,D), E (D,0), S (0,-D), W (-D,0)
    void deriv(const State& s, State& d) const {
        const double D = SP_ANCHOR_D;
        const double axs[4] = {0.0,  D, 0.0, -D};
        const double ays[4] = {  D, 0.0,  -D, 0.0};

        double fx = 0.0, fy = 0.0;
        for (int i = 0; i < 4; ++i) {
            double dx = axs[i] - s.x;
            double dy = ays[i] - s.y;
            double r = std::sqrt(dx * dx + dy * dy);
            if (r < 1e-6) continue;
            double f = springK * (r - restLen) / r;
            fx += f * dx;
            fy += f * dy;
        }
        d.x  = s.vx;
        d.y  = s.vy;
        d.vx = fx / mass - damping * s.vx + noiseAx;
        d.vy = fy / mass - damping * s.vy - g + noiseAy;

        // Mouse "hand": critically damped pull towards the cursor
        if (handOn) {
            d.vx += handWn * handWn * (handX - s.x) - 2.0 * handWn * s.vx;
            d.vy += handWn * handWn * (handY - s.y) - 2.0 * handWn * s.vy;
        }
    }

    void rk4(double dt) {
        State s{x, y, vx, vy};
        State k1, k2, k3, k4, t;

        auto advance = [&](const State& k, double h) {
            t.x  = s.x  + h * k.x;
            t.y  = s.y  + h * k.y;
            t.vx = s.vx + h * k.vx;
            t.vy = s.vy + h * k.vy;
        };

        deriv(s, k1);
        advance(k1, 0.5 * dt); deriv(t, k2);
        advance(k2, 0.5 * dt); deriv(t, k3);
        advance(k3, dt);       deriv(t, k4);

        x  += dt * (k1.x  + 2.0 * k2.x  + 2.0 * k3.x  + k4.x)  / 6.0;
        y  += dt * (k1.y  + 2.0 * k2.y  + 2.0 * k3.y  + k4.y)  / 6.0;
        vx += dt * (k1.vx + 2.0 * k2.vx + 2.0 * k3.vx + k4.vx) / 6.0;
        vy += dt * (k1.vy + 2.0 * k2.vy + 2.0 * k3.vy + k4.vy) / 6.0;
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
            const double tau = 0.12;                 // correlation time (s)
            const double sigma = chaos * 6.0;        // m/s^2
            double a = std::exp(-dt / tau);
            double b = sigma * std::sqrt(1.0 - a * a);
            noiseAx = noiseAx * a + b * gauss(rng);
            noiseAy = noiseAy * a + b * gauss(rng);
        }

        rk4(dt);

        // Safety: NaN guard
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(vx) || !std::isfinite(vy))
            onReset();

        // Soft wall: the sphere can't travel further than 2.5x the anchor distance
        {
            const double rMax = 2.5 * SP_ANCHOR_D;
            double rr = std::sqrt(x * x + y * y);
            if (rr > rMax) {
                double nx = x / rr, ny = y / rr;
                x = nx * rMax;
                y = ny * rMax;
                double vr = vx * nx + vy * ny;
                if (vr > 0.0) {
                    vx -= vr * nx;
                    vy -= vr * ny;
                }
            }
        }

        // New random V/Oct + gate each time the sphere crosses x = 0
        double currentSign = (x >= 0) ? 1.0 : -1.0;
        if (currentSign != prevXSign && std::abs(x) > 1e-6) {
            generateRandomVoct();
            double durSec = getGateDurationSeconds(gateDuration);
            gateTimer = static_cast<int>(args.sampleRate * durSec);
        }
        prevXSign = currentSign;

        if (gateTimer > 0) gateTimer--;

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
        if (trailTimer >= trailStep) {
            trailTimer -= trailStep;
            if (trailTimer > trailStep) trailTimer = 0.f;
            float reach = (float)SP_ANCHOR_D * SP_REACH;
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
        return root;
    }

    void dataFromJson(json_t* root) override {
        json_t* j;
        j = json_object_get(root, "voctRange");
        if (j) voctRange = (VoctRange)json_integer_value(j);
        j = json_object_get(root, "gateDuration");
        if (j) gateDuration = (GateDuration)json_integer_value(j);
    }
};

// ------------------------------------------------------------
// Theme + drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x25, 0x28, 0x30); }
NVGcolor colBgBottom() { return nvgRGB(0x13, 0x15, 0x19); }
NVGcolor colText()     { return nvgRGB(0xd6, 0xdc, 0xe6); }
NVGcolor colDim()      { return nvgRGB(0x8a, 0x95, 0xa5); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colSpring()   { return nvgRGB(0x5a, 0xe6, 0xff); }
NVGcolor colWarm()     { return nvgRGB(0xff, 0x9c, 0x6a); }

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

// Custom light for the energy LED
template <typename TBase = GrayModuleLightWidget>
struct TSPCyanLight : TBase {
    TSPCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using SPCyanLight = TSPCyanLight<>;

// Soft-limit: maps a normalised position (units of display reach) onto the dial so the
// result ALWAYS lies within a circle of radius maxPx. 1:1 near the centre, smoothly
// compressed near the rim, never crosses it.
void mapToDial(float ux, float uy, float maxPx, float& sx, float& sy) {
    float r = std::sqrt(ux * ux + uy * uy);
    if (r < 1e-6f) { sx = 0.f; sy = 0.f; return; }
    float f = (r <= SP_KNEE) ? r : SP_KNEE + (1.f - SP_KNEE) * std::tanh((r - SP_KNEE) / (1.f - SP_KNEE));
    float k = f / r * maxPx;
    sx = ux * k;
    sy = uy * k;
}

// Inverse of the radial part of mapToDial: f = screen radius / maxPx (0..1) -> normalised radius
float unmapRadius(float f) {
    if (f <= SP_KNEE) return f;
    float t = std::min((f - SP_KNEE) / (1.f - SP_KNEE), 0.97f);
    return SP_KNEE + (1.f - SP_KNEE) * std::atanh(t);
}

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

// Coil spring: glow, dark outline, coloured core, bright highlight
void drawSpring(NVGcontext* vg, float x0, float y0, float x1, float y1,
                int coils, float amp, NVGcolor steel) {
    float dx = x1 - x0, dy = y1 - y0;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.f) return;

    float ux = dx / len, uy = dy / len;
    float nx = -uy, ny = ux;

    // Straight lead-in / lead-out so the coil looks mounted
    const float lead = std::min(8.f, len * 0.18f);

    auto zigzag = [&](float ampScale, float width, NVGcolor col) {
        nvgBeginPath(vg);
        nvgMoveTo(vg, x0, y0);
        nvgLineTo(vg, x0 + ux * lead, y0 + uy * lead);
        int steps = coils * 2;
        float body = len - 2.f * lead;
        for (int i = 1; i < steps; ++i) {
            float t = (float)i / steps;
            float side = (i % 2 == 0) ? -1.f : 1.f;
            float px = x0 + ux * (lead + body * t) + nx * amp * side * ampScale;
            float py = y0 + uy * (lead + body * t) + ny * amp * side * ampScale;
            nvgLineTo(vg, px, py);
        }
        nvgLineTo(vg, x1 - ux * lead, y1 - uy * lead);
        nvgLineTo(vg, x1, y1);
        nvgStrokeColor(vg, col);
        nvgStrokeWidth(vg, width);
        nvgLineCap(vg, NVG_ROUND);
        nvgLineJoin(vg, NVG_ROUND);
        nvgStroke(vg);
    };

    zigzag(1.0f, 6.5f, nvgRGBAf(steel.r, steel.g, steel.b, 0.10f));   // glow
    zigzag(1.0f, 4.2f, nvgRGB(0x0d, 0x10, 0x15));                     // outline
    zigzag(1.0f, 2.4f, steel);                                        // core
    zigzag(0.8f, 0.7f, nvgRGBA(255, 255, 255, 170));                  // highlight
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

    // bounce light
    nvgBeginPath(vg);
    nvgCircle(vg, cx + r * 0.38f, cy + r * 0.42f, r * 0.3f);
    nvgFillColor(vg, nvgRGBA(255, 255, 255, 22));
    nvgFill(vg);

    // specular
    nvgBeginPath(vg);
    nvgCircle(vg, cx - r * 0.38f, cy - r * 0.42f, r * 0.2f);
    nvgFillColor(vg, nvgRGBA(255, 255, 255, 210));
    nvgFill(vg);
}

// Machined anchor post: dark base, steel cap, screw slot
void drawAnchor(NVGcontext* vg, float cx, float cy) {
    nvgBeginPath(vg);
    nvgCircle(vg, cx + 1.2f, cy + 2.f, 6.5f);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 100));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, 6.5f);
    nvgFillPaint(vg, nvgLinearGradient(vg, cx, cy - 6.5f, cx, cy + 6.5f,
                                       nvgRGB(0x4a, 0x52, 0x5e), nvgRGB(0x12, 0x15, 0x1a)));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 60));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);

    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, 3.6f);
    nvgFillPaint(vg, nvgRadialGradient(vg, cx - 1.f, cy - 1.f, 0.5f, 4.5f,
                                       nvgRGB(0xf6, 0xf9, 0xfd), nvgRGB(0x6b, 0x75, 0x84)));
    nvgFill(vg);

    nvgBeginPath(vg);
    nvgMoveTo(vg, cx - 2.2f, cy + 0.5f);
    nvgLineTo(vg, cx + 2.2f, cy - 0.5f);
    nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 170));
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

// Small tick arc around a knob (11 ticks across the knob's 270 degree sweep)
void drawKnobTicks(NVGcontext* vg, float cx, float cy, float r0, float r1) {
    nvgLineCap(vg, NVG_ROUND);
    for (int i = 0; i <= 10; ++i) {
        float a = (-0.75f + 1.5f * (float)i / 10.f) * (float)M_PI;
        bool major = (i == 0 || i == 5 || i == 10);
        float ra = major ? r0 - 1.5f : r0;
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + std::sin(a) * ra, cy - std::cos(a) * ra);
        nvgLineTo(vg, cx + std::sin(a) * r1, cy - std::cos(a) * r1);
        nvgStrokeColor(vg, major ? nvgRGBA(0x3d, 0xe0, 0xff, 150) : nvgRGBA(255, 255, 255, 60));
        nvgStrokeWidth(vg, major ? 1.2f : 0.9f);
        nvgStroke(vg);
    }
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct SPPanelBackground : TransparentWidget {
    SPPanelBackground() { box.size = Vec(SP_PANEL_W, SP_PANEL_H); }

    void label(NVGcontext* vg, float x, float y, const char* txt) {
        drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.0f);
    }

    void plate(NVGcontext* vg, float x, float y, float w, float h) {
        // soft drop
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 1.f, y - 1.f, w + 2.f, h + 3.f, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 6.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                           nvgRGBA(0, 0, 0, 105), nvgRGBA(0, 0, 0, 60)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void ring(NVGcontext* vg, float cx, float cy, float r) {
        // recessed well
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, r * 0.3f, r,
                                           nvgRGBA(0, 0, 0, 40), nvgRGBA(0, 0, 0, 110)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = SP_PANEL_W - 14.f, cx = SP_PANEL_W / 2.f;
        float w = 14.f + (float) strlen(txt) * 7.f;
        drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - w / 2.f, y);
        nvgStrokePaint(vg, nvgLinearGradient(vg, x1, y, cx - w / 2.f, y,
                                             nvgRGBA(0x3d, 0xe0, 0xff, 0), nvgRGBA(0x3d, 0xe0, 0xff, 100)));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + w / 2.f, y);
        nvgLineTo(vg, x2, y);
        nvgStrokePaint(vg, nvgLinearGradient(vg, cx + w / 2.f, y, x2, y,
                                             nvgRGBA(0x3d, 0xe0, 0xff, 100), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void jackCell(NVGcontext* vg, float cx, float cy, const char* txt, bool isOutput) {
        float x = cx - 17.f, y = cy - 27.f, w = 34.f, h = 44.f;
        plate(vg, x, y, w, h);

        // colour-coded top bar: cyan = output, orange = input
        NVGcolor c = isOutput ? nvgRGBA(0x3d, 0xe0, 0xff, 170) : nvgRGBA(0xff, 0x9c, 0x6a, 170);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x + 6.f, y + 1.5f, w - 12.f, 1.8f, 0.9f);
        nvgFillColor(vg, c);
        nvgFill(vg);

        label(vg, cx, cy - 18.f, txt);
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

        // Corner vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SP_PANEL_W, SP_PANEL_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, SP_PANEL_W / 2.f, SP_PANEL_H * 0.4f,
                                           SP_PANEL_H * 0.35f, SP_PANEL_H * 0.85f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
        nvgFill(vg);

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, SP_PANEL_W - 1.f, SP_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Top accent line
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SP_PANEL_W, 2.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, SP_PANEL_W, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // Logo + title
        drawLogo(vg, SP_PANEL_W / 2.f, 15.f);
        drawText(vg, SP_PANEL_W / 2.f, 32.f, "SPRING PENDULUM", 12.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // ---- Display bezel (metallic, recessed) ----
        float bx = SP_DISP_X - 5.f, by = SP_DISP_Y - 5.f;
        float bw = SP_DISP_W + 10.f, bh = SP_DISP_H + 10.f;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 4.f, by - 3.f, bw + 8.f, bh + 9.f, 10.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x4a, 0x50, 0x5a), nvgRGB(0x14, 0x16, 0x1a)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 2.f, by - 2.f, bw + 4.f, bh + 4.f, 7.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 36));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- Side knobs: wells, tick scales, labels ----
        const float kx[2] = {SP_LKNOB_X, SP_RKNOB_X};
        const float ky[2] = {SP_KNOB_Y1, SP_KNOB_Y2};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) {
                ring(vg, kx[i], ky[j], 17.f);
                drawKnobTicks(vg, kx[i], ky[j], 21.f, 24.f);
            }

        label(vg, SP_LKNOB_X, SP_KNOB_Y1 + 33.f, "SPEED");
        label(vg, SP_LKNOB_X, SP_KNOB_Y2 + 33.f, "MASS");
        label(vg, SP_RKNOB_X, SP_KNOB_Y1 + 33.f, "REST LEN");
        label(vg, SP_RKNOB_X, SP_KNOB_Y2 + 33.f, "SPRING K");

        // ---- Control section ----
        section(vg, SP_CTRL_SECTION_Y, "CONTROL");

        const float cKnobX[4] = {30.f, 82.f, 134.f, 186.f};
        const char* cKnobL[4] = {"PUSH STR", "CHAOS", "DAMP", "GRAVITY"};
        for (int i = 0; i < 4; ++i) {
            ring(vg, cKnobX[i], SP_CTRL_Y, 17.f);
            drawKnobTicks(vg, cKnobX[i], SP_CTRL_Y, 21.f, 24.f);
            label(vg, cKnobX[i], SP_CTRL_LBL_Y + 3.f, cKnobL[i]);
        }
        ring(vg, 232.f, SP_CTRL_Y, 13.f);
        label(vg, 232.f, SP_CTRL_LBL_Y + 3.f, "PUSH");
        ring(vg, 272.f, SP_CTRL_Y, 13.f);
        label(vg, 272.f, SP_CTRL_LBL_Y + 3.f, "RESET");
        ring(vg, 316.f, SP_CTRL_Y, 13.f);
        label(vg, 316.f, SP_CTRL_LBL_Y + 3.f, "ENERGY");

        // ---- I/O section ----
        section(vg, SP_IO_SECTION_Y, "I / O");
        const char* io[6] = {"X", "Y", "V/OCT", "GATE", "RST IN", "PSH IN"};
        const float ioX[6] = {40.f, 96.f, 152.f, 208.f, 264.f, 320.f};
        for (int i = 0; i < 6; ++i)
            jackCell(vg, ioX[i], SP_IO_Y, io[i], i < 4);

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Spring display widget (also handles mouse grabbing)
// ------------------------------------------------------------
struct SPDisplayWidget : TransparentWidget {
    SpringPendulum* module = nullptr;

    // Mouse interaction state
    bool dragging = false;
    Vec  dragPos;     // pointer position in display-local pixels
    Vec  dragOff;     // offset from pointer to ball centre at grab time

    SPDisplayWidget(SpringPendulum* m) : module(m) {
        box.pos = Vec(SP_DISP_X, SP_DISP_Y);
        box.size = Vec(SP_DISP_W, SP_DISP_H);
    }

    // ---- helpers ----
    static float reachUnits() { return (float)SP_ANCHOR_D * SP_REACH; }

    float bobRadius() const {
        return 4.5f + 2.2f * std::cbrt((float)(module ? module->mass : 1.5));
    }

    Vec ballPixel() const {
        float sx, sy;
        float reach = reachUnits();
        mapToDial((float)(module->x) / reach, (float)(module->y) / reach, SP_MAXPX, sx, sy);
        return Vec(SP_DISP_W / 2.f + sx, SP_DISP_H / 2.f - sy);
    }

    // Convert the pointer (plus grab offset) to a physical target and hand it to the module
    void updateGrab() {
        if (!module) return;
        Vec p = dragPos.plus(dragOff);
        float sx = p.x - SP_DISP_W / 2.f;
        float sy = -(p.y - SP_DISP_H / 2.f);
        float rs = std::sqrt(sx * sx + sy * sy);
        float ux = 0.f, uy = 0.f;
        if (rs > 1e-4f) {
            float f = std::min(rs / SP_MAXPX, 0.98f);
            float r = unmapRadius(f);
            ux = sx / rs * r;
            uy = sy / rs * r;
        }
        module->grabX = (double)(ux * reachUnits());
        module->grabY = (double)(uy * reachUnits());
    }

    // ---- mouse events ----
    void onButton(const ButtonEvent& e) override {
        if (module && e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS) {
            Vec ball = ballPixel();
            float dBall = e.pos.minus(ball).norm();
            float dCentre = e.pos.minus(Vec(SP_DISP_W / 2.f, SP_DISP_H / 2.f)).norm();
            bool onBall = dBall <= bobRadius() + 12.f;

            // Click on / near the ball: grab it where it is.
            // Click elsewhere inside the dial: the ball is pulled to the cursor.
            if (onBall || dCentre <= SP_DIAL_R + 4.f) {
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

    void onDragMove(const DragMoveEvent& e) override {
        if (!dragging || !module) return;
        dragPos = dragPos.plus(e.mouseDelta.div(getAbsoluteZoom()));
        updateGrab();
    }

    void onDragEnd(const DragEndEvent& e) override {
        dragging = false;
        if (module) module->grabbed = false;   // ball is released and keeps its velocity
        TransparentWidget::onDragEnd(e);
    }

    // ---- drawing ----
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
        nvgCircle(vg, cx, cy, SP_DIAL_R);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, 8.f, SP_DIAL_R,
                                           nvgRGB(0x10, 0x20, 0x2c), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);

        // Dial rim (double line)
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, SP_DIAL_R);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
        nvgStrokeWidth(vg, 1.2f);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, SP_DIAL_R + 3.f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Tick scale around the dial (every 5 deg, major every 30 deg)
        nvgLineCap(vg, NVG_BUTT);
        for (int i = 0; i < 72; ++i) {
            float a = (float)i * (float)M_PI / 36.f;
            bool major = (i % 6 == 0);
            float r0 = SP_DIAL_R + 5.f;
            float r1 = SP_DIAL_R + (major ? 10.f : 7.5f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, cx + std::sin(a) * r0, cy - std::cos(a) * r0);
            nvgLineTo(vg, cx + std::sin(a) * r1, cy - std::cos(a) * r1);
            nvgStrokeColor(vg, major ? nvgRGBA(0x3d, 0xe0, 0xff, 130) : nvgRGBA(255, 255, 255, 40));
            nvgStrokeWidth(vg, major ? 1.2f : 0.8f);
            nvgStroke(vg);
        }

        // Range rings
        for (int k = 1; k <= 3; ++k) {
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, SP_DIAL_R * (float)k / 4.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, k % 2 ? 14 : 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Cross hair
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + 0.5f, cy - SP_DIAL_R + 2.f);
        nvgLineTo(vg, cx + 0.5f, cy + SP_DIAL_R - 2.f);
        nvgMoveTo(vg, cx - SP_DIAL_R + 2.f, cy + 0.5f);
        nvgLineTo(vg, cx + SP_DIAL_R - 2.f, cy + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
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

        // Read state
        double px = 0, py = 0, vx = 0, vy = 0, m = 1, K = 10, L = 0.5, g = 0.0;
        bool grabbedNow = false;
        if (module) {
            px = module->x;      py = module->y;
            vx = module->vx;     vy = module->vy;
            m = module->mass;    K = module->springK;
            L = module->restLen; g = module->g;
            grabbedNow = module->grabbed.load();
        } else {
            px = 0.30; py = -0.18;
            vx = 1.5;  vy = 0.6;
            m = 1.5;   K = 20;  L = 0.5;  g = 0.0;
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
        float my = cy - sy;      // +y physical = up on screen

        // Anchor positions: N, E, S, W (same mapping, so springs end on the posts)
        const float dirX[4] = {0.f, 1.f, 0.f, -1.f};
        const float dirY[4] = {1.f, 0.f, -1.f, 0.f};
        float aX[4], aY[4];
        for (int i = 0; i < 4; ++i) {
            float ax, ay;
            mapToDial(dirX[i] * (float)SP_ANCHOR_D / reach,
                      dirY[i] * (float)SP_ANCHOR_D / reach, SP_MAXPX, ax, ay);
            aX[i] = cx + ax;
            aY[i] = cy - ay;
        }

        // Struts from the anchor posts out to the rim
        nvgLineCap(vg, NVG_ROUND);
        for (int i = 0; i < 4; ++i) {
            float ex = cx + dirX[i] * (SP_DIAL_R + 1.f);
            float ey = cy - dirY[i] * (SP_DIAL_R + 1.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, aX[i], aY[i]);
            nvgLineTo(vg, ex, ey);
            nvgStrokeColor(vg, nvgRGB(0x1a, 0x1e, 0x25));
            nvgStrokeWidth(vg, 5.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, aX[i], aY[i]);
            nvgLineTo(vg, ex, ey);
            nvgStrokeColor(vg, nvgRGB(0x4a, 0x52, 0x5e));
            nvgStrokeWidth(vg, 2.6f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, aX[i], aY[i]);
            nvgLineTo(vg, ex, ey);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 70));
            nvgStrokeWidth(vg, 0.7f);
            nvgStroke(vg);
        }

        // Trail (same mapping)
        int tn = 0;
        float tx[SP_TRAIL_N], ty[SP_TRAIL_N];
        if (module) {
            tn = std::min(std::max(module->trailCount, 0), SP_TRAIL_N);
            for (int i = 0; i < tn; ++i) {
                int idx = ((module->trailHead - tn + i) % SP_TRAIL_N + SP_TRAIL_N) % SP_TRAIL_N;
                float ax, ay;
                mapToDial(module->trailX[idx], module->trailY[idx], SP_MAXPX, ax, ay);
                tx[i] = cx + ax;
                ty[i] = cy - ay;
            }
        } else {
            tn = 90;
            for (int i = 0; i < tn; ++i) {
                float k = (float)i / (float)(tn - 1);
                float a1 = 6.0f * k;
                float rr = 0.18f + 0.12f * std::sin(3.f * a1);
                float ax, ay;
                mapToDial(rr * std::cos(a1), rr * std::sin(a1), SP_MAXPX, ax, ay);
                tx[i] = cx + ax;
                ty[i] = cy - ay;
            }
        }

        drawTrail(vg, tx, ty, tn, colSpring(), 4.5f, 0.10f);
        drawTrail(vg, tx, ty, tn, colSpring(), 1.2f, 0.45f);

        // Four springs (colour shifts cyan -> orange with tension)
        for (int i = 0; i < 4; ++i) {
            double ddx = dirX[i] * SP_ANCHOR_D - px;
            double ddy = dirY[i] * SP_ANCHOR_D - py;
            float rr = (float)std::sqrt(ddx * ddx + ddy * ddy);
            float tension = std::min(1.f, std::fabs(rr - (float)L) / 0.6f);
            NVGcolor col = nvgLerpRGBA(colSpring(), colWarm(), tension);

            // shadow
            nvgBeginPath(vg);
            nvgMoveTo(vg, aX[i] + 2.f, aY[i] + 3.f);
            nvgLineTo(vg, mx + 2.f, my + 3.f);
            nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 70));
            nvgStrokeWidth(vg, 4.5f);
            nvgStroke(vg);

            drawSpring(vg, aX[i], aY[i], mx, my, 11, 3.6f, col);
        }

        // Anchor posts on top of the spring ends
        for (int i = 0; i < 4; ++i)
            drawAnchor(vg, aX[i], aY[i]);

        // Energy glow (bright when moving fast)
        float speed = (float)std::sqrt(vx*vx + vy*vy);
        float energy = std::min(speed / 6.f, 1.f);
        nvgBeginPath(vg);
        nvgCircle(vg, mx, my, bobR * 3.f);
        nvgFillPaint(vg, nvgRadialGradient(vg, mx, my, bobR * 0.6f, bobR * 3.f,
                                           nvgRGBA(0xff, 0x9c, 0x6a, (int)(10 + 45 * energy)),
                                           nvgRGBA(0xff, 0x9c, 0x6a, 0)));
        nvgFill(vg);

        // Mass sphere
        drawBob(vg, mx, my, bobR,
                nvgRGB(0xff, 0xe8, 0xd2),
                nvgRGB(0xd0, 0x50, 0x1a),
                nvgRGB(0x4a, 0x14, 0x04));

        // Grab indicator: bright ring around the ball while the mouse holds it
        if (grabbedNow) {
            nvgBeginPath(vg);
            nvgCircle(vg, mx, my, bobR + 4.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 220));
            nvgStrokeWidth(vg, 1.5f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, mx, my, bobR + 7.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) SP_DISP_H; y += 3)
            nvgRect(vg, 0, y + 1, SP_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
        nvgFill(vg);

        // Edge vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SP_DISP_W, SP_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, SP_DISP_W * 0.40f, SP_DISP_W * 0.78f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 150)));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SP_DISP_W, SP_DISP_H * 0.35f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SP_DISP_H * 0.35f,
                                           nvgRGBA(255, 255, 255, 16),
                                           nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts (in the display corners, outside the dial)
        char buf[32];
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 220);
        NVGcolor cm = nvgRGBA(0xff, 0x9c, 0x6a, 230);
        NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 230);

        snprintf(buf, sizeof(buf), "X %+.2f", (float)px);
        drawText(vg, 5.f, 8.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "Y %+.2f", (float)py);
        drawText(vg, 5.f, 17.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "VX %+.1f", (float)vx);
        drawText(vg, SP_DISP_W - 5.f, 8.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "VY %+.1f", (float)vy);
        drawText(vg, SP_DISP_W - 5.f, 17.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "K %.0f", (float)K);
        drawText(vg, 5.f, SP_DISP_H - 8.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "G %.1f", (float)g);
        drawText(vg, SP_DISP_W - 5.f, SP_DISP_H - 8.f, buf, 7.f, cd,
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        if (grabbedNow)
            drawText(vg, cx, SP_DISP_H - 8.f, "GRAB", 7.f, c1,
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        nvgRestore(vg);

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
            Vec(SP_LKNOB_X, SP_KNOB_Y1), module, SpringPendulum::SPEED_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_LKNOB_X, SP_KNOB_Y2), module, SpringPendulum::MASS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_RKNOB_X, SP_KNOB_Y1), module, SpringPendulum::LENGTH_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(SP_RKNOB_X, SP_KNOB_Y2), module, SpringPendulum::SPRING_K_PARAM));

        // Control row
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(30.f, SP_CTRL_Y), module, SpringPendulum::PUSH_STRENGTH_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(82.f, SP_CTRL_Y), module, SpringPendulum::CHAOS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(134.f, SP_CTRL_Y), module, SpringPendulum::DAMP_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(186.f, SP_CTRL_Y), module, SpringPendulum::GRAVITY_PARAM));
        addParam(createParamCentered<LEDButton>(
            Vec(232.f, SP_CTRL_Y), module, SpringPendulum::PUSH_BUTTON));
        addParam(createParamCentered<LEDButton>(
            Vec(272.f, SP_CTRL_Y), module, SpringPendulum::RESET_BUTTON));
        addChild(createLightCentered<MediumLight<SPCyanLight>>(
            Vec(316.f, SP_CTRL_Y), module, SpringPendulum::MASS_LIGHT));

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
    }
};

Model* modelSpringPendulum = createModel<SpringPendulum, SpringPendulumWidget>("SpringPendulum");