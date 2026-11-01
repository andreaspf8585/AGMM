// Lifeform.cpp
// AGMM-styled ferrofluid visualiser.
//
// A glossy black ferrofluid "lifeform" floating in a bright glass dish inside a
// perforated white bezel. The lifeform is real 3D geometry: a lobed blob, ~140
// Rosensweig spikes and orbiting droplets, tumbling through space with perspective
// projection, depth sorting and normal-based gloss.
//
// Sound drives it through three bands (low / mid / high), an onset detector
// ("punch") and a silence detector: in silence everything merges into one blob.

#include <rack.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace rack;

template <typename T> inline T clampT(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }

// =====================================================================
//  Module
// =====================================================================
struct LifeformModule : Module
{
    enum ParamId
    {
        FIELD_PARAM, MOTION_PARAM, VISC_PARAM,
        SPIKES_PARAM, SENS_PARAM, GLOSS_PARAM,
        PAUSE_PARAM, RESET_PARAM, AUDIO_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        AUDIO_INPUT, PULSE_INPUT,
        FIELD_CV, MOTION_CV, VISC_CV,
        SPIKES_CV, SENS_CV, GLOSS_CV,
        INPUTS_LEN
    };
    enum OutputId { ENV_OUTPUT, FIELD_OUTPUT, OUTPUTS_LEN };
    enum LightId { PAUSE_LIGHT, AUDIO_LIGHT, LIGHTS_LEN };

    float env = 0.f;
    float pulseFlash = 0.f;
    float field = 0.f;
    float phase = 0.f;
    bool  prevPulse = false;

    // Three-band split for the visualiser
    float lp1 = 0.f, lp2 = 0.f;
    float eLow = 0.f, eMid = 0.f, eHigh = 0.f;

    bool paused = false;
    bool audioReactive = true;
    bool prevPauseButton = false;
    bool prevAudioButton = false;

    std::atomic<float> envSmoothed{0.f};
    std::atomic<float> fieldSmoothed{0.f};
    std::atomic<float> pulseSmoothed{0.f};
    std::atomic<float> lowSmoothed{0.f};
    std::atomic<float> midSmoothed{0.f};
    std::atomic<float> highSmoothed{0.f};
    std::atomic<float> magnetX{0.f};
    std::atomic<float> magnetY{0.f};

    LifeformModule()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(FIELD_PARAM,  0.f, 1.f, 0.55f, "Magnetic field");
        configParam(MOTION_PARAM, 0.f, 1.f, 0.45f, "Magnet motion");
        configParam(VISC_PARAM,   0.f, 1.f, 0.50f, "Fluid viscosity");
        configParam(SPIKES_PARAM, 0.f, 1.f, 0.65f, "Spike amplitude");
        configParam(SENS_PARAM,   0.f, 1.f, 0.65f, "Audio sensitivity");
        configParam(GLOSS_PARAM,  0.f, 1.f, 0.75f, "Gloss");

        configButton(PAUSE_PARAM, "Pause");
        configButton(RESET_PARAM, "Reset");
        configButton(AUDIO_PARAM, "Audio reactive");
        configLight(PAUSE_LIGHT, "Pause active");
        configLight(AUDIO_LIGHT, "Audio active");

        configInput(AUDIO_INPUT, "Audio");
        configInput(PULSE_INPUT, "Pulse trigger");
        configInput(FIELD_CV,  "Field CV");
        configInput(MOTION_CV, "Motion CV");
        configInput(VISC_CV,   "Viscosity CV");
        configInput(SPIKES_CV, "Spikes CV");
        configInput(SENS_CV,   "Sensitivity CV");
        configInput(GLOSS_CV,  "Gloss CV");

        configOutput(ENV_OUTPUT,   "Envelope");
        configOutput(FIELD_OUTPUT, "Field level");

        configBypass(AUDIO_INPUT, ENV_OUTPUT);
    }

    void onReset(const ResetEvent& e) override
    {
        Module::onReset(e);
        env = pulseFlash = field = phase = 0.f;
        lp1 = lp2 = eLow = eMid = eHigh = 0.f;
        prevPulse = false;
        paused = false;
        audioReactive = true;
        prevPauseButton = false;
        prevAudioButton = false;
        envSmoothed.store(0.f);
        fieldSmoothed.store(0.f);
        pulseSmoothed.store(0.f);
        lowSmoothed.store(0.f);
        midSmoothed.store(0.f);
        highSmoothed.store(0.f);
        magnetX.store(0.f);
        magnetY.store(0.f);
    }

    void process(const ProcessArgs& args) override
    {
        const float sr = args.sampleRate;

        const bool pausePressed = params[PAUSE_PARAM].getValue() > 0.5f;
        if (pausePressed && !prevPauseButton) paused = !paused;
        prevPauseButton = pausePressed;
        lights[PAUSE_LIGHT].setBrightness(paused ? 1.f : 0.f);

        const bool audioPressed = params[AUDIO_PARAM].getValue() > 0.5f;
        if (audioPressed && !prevAudioButton) audioReactive = !audioReactive;
        prevAudioButton = audioPressed;
        lights[AUDIO_LIGHT].setBrightness(audioReactive ? 1.f : 0.f);

        const float in = inputs[AUDIO_INPUT].getVoltage() / 5.f;
        const float rect = std::fabs(in);
        const float atk = 1.f - std::exp(-1.f / (0.004f * sr));
        const float rel = 1.f - std::exp(-1.f / (0.180f * sr));
        env += (rect - env) * (rect > env ? atk : rel);

        // ---- three-band envelope followers (low < 180 Hz < mid < 2.2 kHz < high) ----
        const float cLP1 = std::min(1.f, 2.f * (float)M_PI * 180.f  / sr);
        const float cLP2 = std::min(1.f, 2.f * (float)M_PI * 2200.f / sr);
        lp1 += (in - lp1) * cLP1;
        lp2 += (in - lp2) * cLP2;
        const float bandLow  = std::fabs(lp1) * 1.5f;
        const float bandMid  = std::fabs(lp2 - lp1) * 1.6f;
        const float bandHigh = std::fabs(in - lp2) * 2.0f;
        auto follow = [sr](float& e, float x, float aT, float rT) {
            const float a = std::min(1.f, 1.f / (aT * sr));
            const float r = std::min(1.f, 1.f / (rT * sr));
            e += (x - e) * (x > e ? a : r);
        };
        follow(eLow,  bandLow,  0.003f, 0.14f);
        follow(eMid,  bandMid,  0.002f, 0.09f);
        follow(eHigh, bandHigh, 0.001f, 0.06f);

        const bool pulse = inputs[PULSE_INPUT].getVoltage() >= 1.f;
        if (pulse && !prevPulse) pulseFlash = 1.f;
        prevPulse = pulse;
        pulseFlash -= args.sampleTime * 3.5f;
        if (pulseFlash < 0.f) pulseFlash = 0.f;

        auto v = [this](int p, int cv) {
            return clampT(params[p].getValue() + inputs[cv].getVoltage() / 10.f, 0.f, 1.f);
        };
        const float fieldKnob  = v(FIELD_PARAM,  FIELD_CV);
        const float motionKnob = v(MOTION_PARAM, MOTION_CV);
        const float sensKnob   = v(SENS_PARAM,   SENS_CV);

        if (!paused) {
            const float audioBoost = audioReactive ? env * sensKnob * 1.1f : 0.f;
            const float fieldTarget = clampT(fieldKnob + audioBoost + pulseFlash * 0.75f, 0.f, 1.f);
            // faster field response than before (was 55 ms)
            const float fieldCoef = 1.f - std::exp(-1.f / (0.025f * sr));
            field += (fieldTarget - field) * fieldCoef;

            phase += motionKnob * args.sampleTime * 1.35f;
            if (phase > 2.f * (float)M_PI) phase -= 2.f * (float)M_PI;
        }

        magnetX.store(std::cos(phase) * 0.55f);
        magnetY.store(std::sin(phase * 1.31f) * 0.55f);
        fieldSmoothed.store(field);
        envSmoothed.store(env);
        pulseSmoothed.store(pulseFlash);
        lowSmoothed.store(clampT(eLow, 0.f, 1.5f));
        midSmoothed.store(clampT(eMid, 0.f, 1.5f));
        highSmoothed.store(clampT(eHigh, 0.f, 1.5f));

        outputs[ENV_OUTPUT].setVoltage(clampT(env * 10.f, 0.f, 10.f));
        outputs[FIELD_OUTPUT].setVoltage(field * 10.f);
    }
};

// =====================================================================
//  Layout constants (18 HP = 270 px)
// =====================================================================
static const float LF_PANEL_W = RACK_GRID_WIDTH * 18;   // 270
static const float LF_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float LF_DISP_X = 20.f;
static const float LF_DISP_Y = 46.f;
static const float LF_DISP_W = 230.f;
static const float LF_DISP_H = 110.f;

static const float LF_DISH_R = 41.f;    // glass radius inside the bezel
static const float LF_CAM    = 320.f;   // camera distance for perspective

static const float LF_COL[3] = {55.f, 135.f, 215.f};
static const float LF_BTN_LABEL_Y = 162.f;
static const float LF_BTN_Y = 174.f;
static const float LF_BTN_LED_Y = 186.f;
static const float LF_ROW1_Y = 216.f;
static const float LF_CV1_Y  = 248.f;
static const float LF_ROW2_Y = 288.f;
static const float LF_CV2_Y  = 320.f;
static const float LF_IO_Y   = 356.f;

// =====================================================================
//  Theme + drawing helpers
// =====================================================================
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }

inline float hash1(float x) {
    const float s = std::sin(x * 127.1f + 311.7f) * 43758.5453f;
    return s - std::floor(s);
}

inline float smooth01(float a, float b, float x) {
    const float t = clampT((x - a) / (b - a), 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

// ---- tiny 3D maths ----
struct V3 { float x, y, z; };
inline V3 v3(float x, float y, float z) { V3 r; r.x = x; r.y = y; r.z = z; return r; }
inline V3 operator+(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline V3 operator-(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline V3 operator*(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
inline float dot3(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross3(V3 a, V3 b) {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline float len3(V3 a) { return std::sqrt(dot3(a, a)); }
inline V3 norm3(V3 a) { return a * (1.f / (len3(a) + 1e-6f)); }

struct M3 { float m[9]; };   // row-major
inline V3 m3mul(const M3& M, V3 v) {
    return v3(M.m[0] * v.x + M.m[1] * v.y + M.m[2] * v.z,
              M.m[3] * v.x + M.m[4] * v.y + M.m[5] * v.z,
              M.m[6] * v.x + M.m[7] * v.y + M.m[8] * v.z);
}

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

struct LFKnobDef { const char* name; int param; int cv; };
const LFKnobDef LF_KNOBS[6] = {
    {"FIELD",  LifeformModule::FIELD_PARAM,  LifeformModule::FIELD_CV},
    {"MOTION", LifeformModule::MOTION_PARAM, LifeformModule::MOTION_CV},
    {"VISC",   LifeformModule::VISC_PARAM,   LifeformModule::VISC_CV},
    {"SPIKES", LifeformModule::SPIKES_PARAM, LifeformModule::SPIKES_CV},
    {"SENS",   LifeformModule::SENS_PARAM,   LifeformModule::SENS_CV},
    {"GLOSS",  LifeformModule::GLOSS_PARAM,  LifeformModule::GLOSS_CV},
};

// =====================================================================
//  Ferrofluid display - 3D tumbling blob, spikes and droplets
// =====================================================================
struct LifeformDisplay : TransparentWidget
{
    enum { N_SPIKES = 140, N_DROPS = 14, LAT = 16, LON = 32, GN = LAT * LON, NB = 96 };

    struct Spike {
        V3    d;                    // local unit direction on the body
        float rnd, phase;           // per-spike randoms
        float h, v;                 // height (0..~1.2) and velocity
        // per-frame projected geometry (px, relative to dish centre)
        float bx, by, tx, ty, hw, spec, lit;
        bool  front, vis;
    };

    struct Drop {
        V3    e1, e2;               // orbital plane basis (local)
        float ang, dist, rest, r;   // orbit angle, distance (px), rest distance, radius
        float dv, w, phase;
        float x, y, sc, z, vis;     // projected position, scale, depth, visibility
    };

    Spike spikes[N_SPIKES];
    Drop  drops[N_DROPS];

    V3    ldir[GN];                 // local unit directions of the surface grid
    V3    wpos[GN];                 // world positions (relative to dish centre)
    V3    wnorm[GN];                // world normals
    float psx[GN], psy[GN], psc[GN];
    float sil[NB];                  // projected silhouette radius per angular bin

    M3    Rm;                       // body rotation
    V3    om;                       // angular velocity (world frame)
    V3    Lv, Hv, H2v, magD;        // key light, half vectors, magnet direction

    LifeformModule* module = nullptr;
    float  t = 0.f;
    double lastTime = 0.0;
    bool   prevResetButton = false;
    float  prevPulseVal = 0.f;
    float  prevEnvFrame = 0.f;
    float  onsetCool = 0.f;

    // Shared geometry / state, updated by stepPhysics, consumed by the draw code
    float bodyBase = 11.f;
    float wobAmt   = 0.08f;
    float stretch  = 0.f;
    float magA     = 0.f;
    float magLen   = 0.f;
    float offX = 0.f, offY = 0.f;
    float offVX = 0.f, offVY = 0.f;

    float act     = 0.f;    // smoothed audio activity (0 = silence)
    float calmK   = 1.f;    // 1 = fully merged single blob
    float wt      = 0.f;    // warped shape time (faster with sound)
    float spinVel = 0.f;    // |angular velocity|
    float ripAmt  = 0.f;
    float ell     = 0.12f;
    float punch   = 0.f;    // transient energy (onset detector)

    explicit LifeformDisplay(LifeformModule* m) : module(m)
    {
        box.pos  = Vec(LF_DISP_X, LF_DISP_Y);
        box.size = Vec(LF_DISP_W, LF_DISP_H);

        std::memset(ldir, 0, sizeof(ldir));
        std::memset(wpos, 0, sizeof(wpos));
        std::memset(wnorm, 0, sizeof(wnorm));
        std::memset(psx, 0, sizeof(psx));
        std::memset(psy, 0, sizeof(psy));
        std::memset(psc, 0, sizeof(psc));
        std::memset(sil, 0, sizeof(sil));

        // identity rotation + a gentle initial tilt
        const float I[9] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
        std::memcpy(Rm.m, I, sizeof(I));
        om = v3(0.3f, 0.7f, 0.1f);
        rotateBy(v3(0.5f, 0.8f, 0.2f), 1.f);

        Lv  = norm3(v3(-0.5f, -0.6f, 0.62f));
        Hv  = norm3(Lv + v3(0.f, 0.f, 1.f));
        const V3 L2 = norm3(v3(0.6f, 0.5f, 0.55f));
        H2v = norm3(L2 + v3(0.f, 0.f, 1.f));
        magD = norm3(v3(0.8f, 0.f, 0.6f));

        // surface grid (y is the pole axis)
        for (int i = 0; i < LAT; ++i) {
            const float th = (float)M_PI * ((float)i + 0.5f) / (float)LAT;
            for (int j = 0; j < LON; ++j) {
                const float ph = 2.f * (float)M_PI * (float)j / (float)LON;
                ldir[i * LON + j] = v3(std::sin(th) * std::cos(ph), std::cos(th), std::sin(th) * std::sin(ph));
            }
        }

        // spikes on a Fibonacci sphere
        for (int i = 0; i < N_SPIKES; ++i) {
            Spike& s = spikes[i];
            const float y  = 1.f - 2.f * ((float)i + 0.5f) / (float)N_SPIKES;
            const float rr = std::sqrt(std::max(0.f, 1.f - y * y));
            const float ph = (float)i * 2.399963f;
            s.d     = v3(std::cos(ph) * rr, y, std::sin(ph) * rr);
            s.rnd   = hash1(i * 3.3f + 1.f);
            s.phase = hash1(i * 5.1f) * 2.f * (float)M_PI;
            s.h = s.v = 0.f;
            s.bx = s.by = s.tx = s.ty = s.hw = s.spec = s.lit = 0.f;
            s.front = true; s.vis = false;
        }

        // droplets with individual tilted orbital planes
        for (int i = 0; i < N_DROPS; ++i) {
            Drop& d = drops[i];
            const V3 n = norm3(v3(hash1(i * 1.9f) - 0.5f, hash1(i * 2.9f + 1.f) - 0.5f, hash1(i * 3.9f + 2.f) - 0.5f));
            const V3 helper = (std::fabs(n.y) < 0.9f) ? v3(0.f, 1.f, 0.f) : v3(1.f, 0.f, 0.f);
            d.e1 = norm3(cross3(n, helper));
            d.e2 = cross3(n, d.e1);
            d.ang   = hash1(i * 9.1f) * 2.f * (float)M_PI;
            d.rest  = LF_DISH_R * (0.62f + 0.26f * hash1(i * 2.2f));
            d.dist  = d.rest;
            d.r     = (i < 3) ? (2.4f + 1.8f * hash1(i * 4.4f))
                              : (0.7f + 1.3f * hash1(i * 6.6f));
            d.dv    = 0.f;
            d.w     = (hash1(i * 7.7f) - 0.5f) * 0.9f;
            d.phase = hash1(i * 8.8f) * 6.2832f;
            d.x = d.y = d.z = 0.f; d.sc = 1.f; d.vis = 1.f;
        }

        stepPhysics(1.f / 60.f);
    }

    void resetLife()
    {
        for (int i = 0; i < N_SPIKES; ++i) { spikes[i].h = 0.f; spikes[i].v = 0.f; }
        for (int i = 0; i < N_DROPS; ++i)  { drops[i].dist = drops[i].rest; drops[i].dv = 0.f; }
        punch = 0.f;
    }

    // ---- 3D helpers ----
    static void project(const V3& P, float& x, float& y, float& s)
    {
        s = LF_CAM / (LF_CAM - P.z);
        x = P.x * s;
        y = P.y * s;
    }

    void rotateBy(V3 w, float dt)
    {
        const float wl  = len3(w);
        const float ang = wl * dt;
        if (ang < 1e-6f) return;
        const V3 k = w * (1.f / wl);
        const float c = std::cos(ang), s = std::sin(ang), C = 1.f - c;
        const float Q[9] = {
            c + k.x * k.x * C,       k.x * k.y * C - k.z * s, k.x * k.z * C + k.y * s,
            k.y * k.x * C + k.z * s, c + k.y * k.y * C,       k.y * k.z * C - k.x * s,
            k.z * k.x * C - k.y * s, k.z * k.y * C + k.x * s, c + k.z * k.z * C
        };
        float N[9];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                float sum = 0.f;
                for (int l = 0; l < 3; ++l) sum += Q[i * 3 + l] * Rm.m[l * 3 + j];
                N[i * 3 + j] = sum;
            }
        // re-orthonormalise to stop drift
        V3 r0 = norm3(v3(N[0], N[1], N[2]));
        V3 r1 = v3(N[3], N[4], N[5]);
        r1 = norm3(r1 - r0 * dot3(r1, r0));
        const V3 r2 = cross3(r0, r1);
        Rm.m[0] = r0.x; Rm.m[1] = r0.y; Rm.m[2] = r0.z;
        Rm.m[3] = r1.x; Rm.m[4] = r1.y; Rm.m[5] = r1.z;
        Rm.m[6] = r2.x; Rm.m[7] = r2.y; Rm.m[8] = r2.z;
    }

    // Surface radius (px) for a local direction d with world direction wd
    float surfaceR(const V3& d, const V3& wd) const
    {
        const float n = std::sin(2.1f * d.x + 1.7f * d.y + wt * 0.90f) * 0.50f
                      + std::sin(3.2f * d.y - 2.4f * d.z - wt * 1.30f + 1.7f) * 0.30f
                      + std::sin(1.6f * d.z + 2.2f * d.x + wt * 0.55f) * 0.40f
                      + std::sin(5.0f * d.x + 4.1f * d.y + 3.7f * d.z + wt * 2.10f) * 0.12f;
        const float al = std::max(0.f, dot3(wd, magD));
        const float r = bodyBase * (1.f + wobAmt * n
                                        + ripAmt * std::sin(7.f * d.y - wt * 7.f + 2.f * d.x)
                                        + ell * (1.5f * d.x * d.x - 0.5f)
                                        + stretch * al * al * al);
        const float oLen = std::sqrt(offX * offX + offY * offY);
        return std::min(r, LF_DISH_R * 0.80f - oLen);
    }

    void stepPhysics(float dt)
    {
        t += dt;
        const float R = LF_DISH_R;
        const float twoPi = 2.f * (float)M_PI; (void)twoPi;

        const float field = module ? module->fieldSmoothed.load() : 0.58f;
        const float env   = module ? module->envSmoothed.load()   : 0.1f;
        const float pulse = module ? module->pulseSmoothed.load() : 0.f;
        const float low   = module ? module->lowSmoothed.load()   : 0.20f;
        const float mid   = module ? module->midSmoothed.load()   : 0.10f;
        const float high  = module ? module->highSmoothed.load()  : 0.05f;
        const float mx    = module ? module->magnetX.load() : std::cos(t * 0.6f) * 0.5f;
        const float my    = module ? module->magnetY.load() : std::sin(t * 0.8f) * 0.5f;

        const float visc     = module ? module->params[LifeformModule::VISC_PARAM].getValue()   : 0.5f;
        const float spikeAmt = module ? module->params[LifeformModule::SPIKES_PARAM].getValue() : 0.65f;
        const float sens     = module ? module->params[LifeformModule::SENS_PARAM].getValue()   : 0.65f;
        const float motion   = module ? module->params[LifeformModule::MOTION_PARAM].getValue() : 0.45f;
        const bool  audio    = module ? module->audioReactive : true;

        const float criticalField = 0.28f;
        const float audioEnv = audio ? env * sens : 0.f;
        const float lowA     = audio ? low  * sens : 0.f;
        const float midA     = audio ? mid  * sens : 0.f;
        const float highA    = audio ? high * sens : 0.f;

        // ---- Onset detector ("punch"): transients hit the body hard ----
        punch *= std::exp(-dt * 6.f);
        onsetCool -= dt;
        float onset = 0.f;
        if (audio) {
            const float dEnv = env - prevEnvFrame;
            onset = std::max(0.f, dEnv * (0.016f / std::max(dt, 0.004f)) * (0.5f + sens) - 0.025f);
        }
        prevEnvFrame = env;
        punch = std::min(punch + onset * 1.6f, 1.2f);
        bool hit = false;
        if (onset > 0.12f && onsetCool <= 0.f) { hit = true; onsetCool = 0.07f; }

        // ---- Activity: silence collapses everything into one calm blob ----
        // (with Audio-reactive switched off the lifeform stays fully active)
        const float level     = env * (0.4f + sens) + punch * 0.2f;
        const float actTarget = audio ? std::max(smooth01(0.008f, 0.09f, level), pulse) : 1.f;
        const float actRate   = (actTarget > act) ? 30.f : 1.0f;     // instant bloom, slow merge
        act  += (actTarget - act) * (1.f - std::exp(-actRate * dt));
        calmK = smooth01(0.15f, 0.85f, 1.f - act);

        // Shape time runs faster with sound
        wt += dt * (0.7f + 0.8f * act + 1.5f * audioEnv + 1.0f * midA);

        // ---- 3D tumble ----
        const bool kick = (pulse - prevPulseVal) > 0.15f;
        prevPulseVal = pulse;

        const float spinMag = (0.18f + 0.8f * motion) * (0.35f + 0.65f * act) + 1.2f * lowA;
        const V3 axis = norm3(v3(0.7f * std::sin(t * 0.29f + 0.5f),
                                 0.8f + 0.5f * std::sin(t * 0.41f + 2.f),
                                 0.6f * std::cos(t * 0.23f)));
        const V3 omT = axis * spinMag;
        om = om + (omT - om) * (1.f - std::exp(-1.5f * dt));
        if (kick || hit) {
            const V3 rv = norm3(v3(hash1(t * 13.1f + 1.f) - 0.5f,
                                   hash1(t * 7.7f + 3.f) - 0.5f,
                                   hash1(t * 3.3f + 5.f) - 0.5f));
            om = om + rv * (kick ? 3.f : (1.2f + 2.f * onset));
        }
        spinVel = len3(om);
        if (spinVel > 5.f) { om = om * (5.f / spinVel); spinVel = 5.f; }
        rotateBy(om, dt);

        // ---- Body geometry ----
        magLen = clampT(std::sqrt(mx * mx + my * my) / 0.55f, 0.f, 1.2f);
        magA   = std::atan2(my, mx);
        magD   = norm3(v3(std::cos(magA) * 0.8f, std::sin(magA) * 0.8f, 0.6f));

        bodyBase = R * (0.26f + 0.09f * field * (0.4f + 0.6f * act)
                              + 0.10f * lowA + 0.04f * audioEnv + 0.02f * pulse + 0.05f * punch)
                     * (1.f + 0.20f * calmK + 0.03f * std::sin(wt * 1.7f));   // merged blob is bigger
        wobAmt  = 0.08f + 0.06f * field * act + 0.05f * audioEnv + 0.08f * midA;
        ripAmt  = 0.12f * midA + 0.10f * pulse + 0.06f * punch;
        ell     = 0.12f + 0.08f * act;
        stretch = clampT(0.20f * magLen * (0.4f + field) * (0.35f + 0.65f * act), 0.f, 0.30f);

        // Centre of mass follows the magnet with inertia -> overshoot and slosh
        const float cAmt = 0.45f + 0.55f * act;
        const float tx = (mx * R * 0.30f + std::cos(wt * 0.7f) * R * 0.04f) * cAmt;
        const float ty = (my * R * 0.30f + std::sin(wt * 0.9f) * R * 0.04f) * cAmt;
        if (hit) {
            offVX += (hash1(t * 5.1f) - 0.5f) * 30.f * (0.5f + onset);
            offVY += (hash1(t * 6.3f) - 0.5f) * 30.f * (0.5f + onset);
        }
        const float ck = 14.f, cd = 2.5f + visc * 4.f;
        offVX += (ck * (tx - offX) - cd * offVX) * dt;
        offVY += (ck * (ty - offY) - cd * offVY) * dt;
        offX  += offVX * dt;
        offY  += offVY * dt;
        const float oMax = R * 0.30f;
        offX = clampT(offX, -oMax, oMax);
        offY = clampT(offY, -oMax, oMax);

        // ---- Spikes ----
        for (int i = 0; i < N_SPIKES; ++i) {
            Spike& s = spikes[i];

            const V3 wd = m3mul(Rm, s.d);
            const float align = 0.5f + 0.5f * dot3(wd, magD);
            const float gate  = 0.25f + 0.75f * act;       // silence -> no spikes at any field
            const float localField = (field * (0.80f + 0.35f * align * magLen)
                                    + audioEnv * (0.45f + 0.70f * align)
                                    + highA * (0.15f + 0.55f * s.rnd)
                                    + pulse * 0.20f + punch * 0.30f) * gate;

            // Hysteresis: formed spikes need less field to persist
            const float threshold = (s.h > 0.05f) ? criticalField * 0.85f : criticalField;

            float target = 0.f;
            if (localField > threshold) {
                const float drive = localField - threshold;
                const float clump = 0.65f + 0.35f * std::sin(s.d.x * 5.f + s.d.z * 3.f + wt * 0.8f);
                const float wave  = 1.f + 0.28f * act * std::sin(wd.x * 3.f - wd.y * 2.f - wt * 2.4f);
                const float fizz  = 1.f + 0.5f * highA * std::sin(wt * 14.f + s.phase * 7.f);
                target = std::sqrt(drive) * 2.0f * spikeAmt
                       * (0.45f + 0.55f * s.rnd) * clump * wave * fizz
                       * (1.f + 0.5f * punch)
                       * (0.90f + 0.10f * std::sin(wt * 3.f + s.phase));
                target = std::min(target, 1.2f);
            }

            const float k    = 60.f * (1.1f - visc * 0.4f) * (1.f + 1.2f * audioEnv);
            const float damp = (8.f + visc * 8.f) * (1.f + 0.6f * audioEnv);
            s.v += (k * (target - s.h) - damp * s.v) * dt;
            s.h += s.v * dt;
            if (s.h < 0.f)  { s.h = 0.f; s.v = 0.f; }
            if (s.h > 1.3f) { s.h = 1.3f; s.v = 0.f; }
        }

        // ---- Satellite droplets (3D orbits) ----
        // Silence (calmK) pulls every droplet back into the main body
        const float pull = std::max(smooth01(0.30f, 0.85f, field * (0.5f + 0.5f * act)), calmK);
        const V3 off3 = v3(offX, offY, 0.f);

        for (int i = 0; i < N_DROPS; ++i) {
            Drop& d = drops[i];

            if (kick || hit) d.dv += (kick ? 38.f : 22.f * (0.5f + onset)) * (0.4f + hash1(i * 1.3f + t));

            float tgt = d.rest * (1.f - pull) + bodyBase * 0.45f * pull;
            tgt += audioEnv * 3.f * std::sin(wt * 8.f + d.phase);
            tgt += highA * 4.f * std::sin(wt * 17.f + d.phase * 2.f) * (1.f - pull);
            tgt += 2.5f * (1.f - pull) * std::sin(wt * 1.3f + d.phase);   // lazy bobbing

            const float k    = 26.f;
            const float damp = 5.f + visc * 7.f;
            d.dv   += (k * (tgt - d.dist) - damp * d.dv) * dt;
            d.dist += d.dv * dt;
            d.dist  = clampT(d.dist, 0.f, R * 0.9f);

            d.ang += ((d.w * 2.f + spinVel * (0.3f + 0.4f * d.phase / 6.2832f)) * (1.f - pull)) * dt;

            const float ca = std::cos(d.ang), sa = std::sin(d.ang);
            const V3 local = d.e1 * (ca * d.dist) + d.e2 * (sa * d.dist);
            V3 P = m3mul(Rm, local) + off3
                 + magD * (field * magLen * act * R * 0.10f * (1.f - pull));

            project(P, d.x, d.y, d.sc);
            d.z = P.z;

            const float lim = R * 0.93f - d.r * d.sc;
            const float len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len > lim && len > 0.001f) { d.x *= lim / len; d.y *= lim / len; }

            // droplets that sink into the body shrink away (merge)
            d.vis = smooth01(0.85f * bodyBase, 1.15f * bodyBase, d.dist);
        }

        buildGeometry();
    }

    // Build the projected 3D scene for this frame
    void buildGeometry()
    {
        const float R = LF_DISH_R;
        const V3 off3 = v3(offX, offY, 0.f);
        const float oLen = std::sqrt(offX * offX + offY * offY);

        // ---- surface points ----
        for (int i = 0; i < GN; ++i) {
            const V3 d  = ldir[i];
            const V3 wd = m3mul(Rm, d);
            wpos[i] = wd * surfaceR(d, wd) + off3;
        }

        // ---- normals from grid differences ----
        for (int i = 0; i < LAT; ++i) {
            for (int j = 0; j < LON; ++j) {
                const int id = i * LON + j;
                const int jp = i * LON + ((j + 1) % LON);
                const int jm = i * LON + ((j + LON - 1) % LON);
                const int ip = std::min(i + 1, (int)LAT - 1) * LON + j;
                const int im = std::max(i - 1, 0) * LON + j;
                wnorm[id] = norm3(cross3(wpos[jp] - wpos[jm], wpos[ip] - wpos[im]));
            }
        }

        // ---- projected silhouette ----
        for (int b = 0; b < NB; ++b) sil[b] = 0.f;
        for (int i = 0; i < GN; ++i) {
            project(wpos[i], psx[i], psy[i], psc[i]);
            const float dx = psx[i] - offX, dy = psy[i] - offY;
            const float rad = std::sqrt(dx * dx + dy * dy);
            float a = std::atan2(dy, dx);
            if (a < 0.f) a += 2.f * (float)M_PI;
            int bin = (int)(a / (2.f * (float)M_PI) * NB);
            if (bin >= NB) bin = NB - 1;
            if (rad > sil[bin]) sil[bin] = rad;
        }
        for (int b = 0; b < NB; ++b)
            if (sil[b] <= 0.f) sil[b] = bodyBase;
        float tmp[NB];
        for (int b = 0; b < NB; ++b)
            tmp[b] = 0.25f * sil[(b + NB - 1) % NB] + 0.5f * sil[b] + 0.25f * sil[(b + 1) % NB];
        const float silMax = R * 0.92f - oLen;
        for (int b = 0; b < NB; ++b) sil[b] = std::min(tmp[b], silMax);

        // ---- spikes ----
        const float lean = magLen * 0.25f * act;
        for (int i = 0; i < N_SPIKES; ++i) {
            Spike& s = spikes[i];
            const V3 wd = m3mul(Rm, s.d);
            const float rb = surfaceR(s.d, wd) - 1.2f;
            const V3 base = wd * rb + off3;
            const V3 nrm  = norm3(wd + magD * lean + cross3(om, wd) * 0.10f);
            const float hpx = s.h * R * 0.20f;

            s.vis = hpx > 0.6f;
            if (!s.vis) continue;

            const V3 tip = base + nrm * hpx;
            float sb, st;
            project(base, s.bx, s.by, sb);
            project(tip,  s.tx, s.ty, st);

            // keep tips inside the glass
            const float lim = R * 0.95f;
            const float dB = std::sqrt(s.bx * s.bx + s.by * s.by);
            const float dT = std::sqrt(s.tx * s.tx + s.ty * s.ty);
            if (dT > lim) {
                const float k = (dB < lim && dT > dB) ? (lim - dB) / (dT - dB) : 0.f;
                s.tx = s.bx + (s.tx - s.bx) * k;
                s.ty = s.by + (s.ty - s.by) * k;
            }

            s.front = nrm.z >= 0.f;
            s.hw    = (0.9f + 1.5f * std::min(1.f, hpx / 8.f)) * sb;
            s.spec  = std::pow(std::max(0.f, dot3(nrm, Hv)), 16.f);
            s.lit   = 0.35f + 0.65f * std::max(0.f, dot3(nrm, Lv));
        }
    }

    // ----- Drawing -----
    void drawDish(NVGcontext* vg, float cx, float cy, float R)
    {
        // Soft ambient glow + drop shadow behind the device
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, R + 24.f);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, R * 0.8f, R + 24.f,
                                           nvgRGBA(190, 215, 255, 34), nvgRGBA(190, 215, 255, 0)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy + 2.f, R + 14.f);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy + 2.f, R + 6.f, R + 14.f,
                                           nvgRGBA(0, 0, 0, 170), nvgRGBA(0, 0, 0, 0)));
        nvgFill(vg);

        // White bezel body
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, R + 10.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, cx - R, cy - R, cx + R, cy + R,
                                           nvgRGB(0xf8, 0xf9, 0xfb), nvgRGB(0xb2, 0xb9, 0xc4)));
        nvgFill(vg);

        // Bezel outer edge highlight / shade
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, R + 9.6f);
        nvgStrokeWidth(vg, 1.f);
        nvgStrokePaint(vg, nvgLinearGradient(vg, cx - R, cy - R, cx + R, cy + R,
                                             nvgRGBA(255, 255, 255, 220), nvgRGBA(60, 70, 85, 140)));
        nvgStroke(vg);

        // Speaker-style perforation ring
        nvgBeginPath(vg);
        const int dots = 84;
        for (int i = 0; i < dots; ++i) {
            const float a = 2.f * (float)M_PI * i / dots;
            nvgCircle(vg, cx + std::cos(a) * (R + 5.8f), cy + std::sin(a) * (R + 5.8f), 0.85f);
        }
        nvgFillColor(vg, nvgRGBA(52, 60, 74, 215));
        nvgFill(vg);

        // Groove around the glass
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, R + 1.6f);
        nvgStrokeWidth(vg, 2.f);
        nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 70));
        nvgStroke(vg);

        // Bright glass
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, R);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx - R * 0.15f, cy - R * 0.25f, R * 0.1f, R * 1.15f,
                                           nvgRGB(0xff, 0xff, 0xff), nvgRGB(0xd9, 0xde, 0xe6)));
        nvgFill(vg);
    }

    void drawShadow(NVGcontext* vg, float cx, float cy)
    {
        const float bb = bodyBase;
        nvgSave(vg);
        nvgTranslate(vg, cx + offX * 0.6f + 1.5f, cy + offY * 0.6f + bb * 1.05f + 3.f);
        nvgScale(vg, 1.f, 0.38f);
        nvgBeginPath(vg);
        nvgCircle(vg, 0.f, 0.f, bb * 1.5f);
        nvgFillPaint(vg, nvgRadialGradient(vg, 0.f, 0.f, bb * 0.3f, bb * 1.5f,
                                           nvgRGBA(40, 52, 76, 70), nvgRGBA(40, 52, 76, 0)));
        nvgFill(vg);
        nvgRestore(vg);
    }

    void silhouettePath(NVGcontext* vg, float bcx, float bcy)
    {
        nvgBeginPath(vg);
        for (int b = 0; b < NB; ++b) {
            const float a = 2.f * (float)M_PI * (float)b / (float)NB;
            const float x = bcx + std::cos(a) * sil[b];
            const float y = bcy + std::sin(a) * sil[b];
            if (b == 0) nvgMoveTo(vg, x, y);
            else        nvgLineTo(vg, x, y);
        }
        nvgClosePath(vg);
    }

    void drawBody(NVGcontext* vg, float cx, float cy, float gloss)
    {
        const float bcx = cx + offX, bcy = cy + offY, bb = bodyBase;

        // Glossy black body
        silhouettePath(vg, bcx, bcy);
        nvgFillPaint(vg, nvgRadialGradient(vg, bcx - bb * 0.25f, bcy - bb * 0.35f, bb * 0.1f, bb * 1.6f,
                                           nvgRGB(0x1e, 0x24, 0x2f), nvgRGB(0x02, 0x03, 0x05)));
        nvgFill(vg);

        // Cool rim light on the lower-right edge
        silhouettePath(vg, bcx, bcy);
        nvgStrokeWidth(vg, 1.2f);
        nvgStrokePaint(vg, nvgLinearGradient(vg, bcx - bb, bcy - bb, bcx + bb, bcy + bb,
                                             nvgRGBA(120, 170, 255, 0),
                                             nvgRGBA(130, 180, 255, (int)(110.f * gloss))));
        nvgStroke(vg);

        // Normal-based reflections: highlights slide over the lobes as the body tumbles
        for (int i = 0; i < GN; ++i) {
            const V3& N = wnorm[i];
            if (N.z < 0.25f) continue;
            const float sc = psc[i];
            const float px = cx + psx[i], py = cy + psy[i];

            const float dL = dot3(N, Lv);
            if (dL > 0.55f) {                      // broad sky sheen
                const float sh = std::pow(dL, 8.f);
                nvgBeginPath(vg);
                nvgCircle(vg, px, py, 4.2f * sc);
                nvgFillColor(vg, nvgRGBA(215, 232, 255, (int)(34.f * sh * gloss)));
                nvgFill(vg);
            }

            const float dH = dot3(N, Hv);
            if (dH > 0.80f) {                      // sharp key-light specular
                const float sp = std::pow(dH, 30.f);
                nvgBeginPath(vg);
                nvgCircle(vg, px, py, 3.4f * sc);
                nvgFillColor(vg, nvgRGBA(235, 245, 255, (int)(70.f * sp * gloss)));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, px, py, 1.6f * sc);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(235.f * sp * gloss)));
                nvgFill(vg);
            }

            const float dH2 = dot3(N, H2v);
            if (dH2 > 0.78f) {                     // cool fill reflection
                const float sp = std::pow(dH2, 18.f);
                nvgBeginPath(vg);
                nvgCircle(vg, px, py, 2.4f * sc);
                nvgFillColor(vg, nvgRGBA(120, 175, 255, (int)(95.f * sp * gloss)));
                nvgFill(vg);
            }
        }
    }

    void drawSpikes(NVGcontext* vg, float cx, float cy, float gloss, bool front)
    {
        // Pass 1: dark silhouettes (single path)
        nvgBeginPath(vg);
        for (int i = 0; i < N_SPIKES; ++i) {
            const Spike& s = spikes[i];
            if (!s.vis || s.front != front) continue;

            const float bx = cx + s.bx, by = cy + s.by;
            const float tx = cx + s.tx, ty = cy + s.ty;
            const float dx = tx - bx, dy = ty - by;
            const float L = std::sqrt(dx * dx + dy * dy);

            if (L < 1.0f) {                        // seen end-on: a small dark cone tip
                nvgCircle(vg, bx + dx * 0.5f, by + dy * 0.5f, s.hw * 0.8f);
                continue;
            }
            const float ux = dx / L, uy = dy / L;
            const float px = -uy, py = ux;
            const float hw = s.hw;
            const float mxp = bx + ux * L * 0.5f, myp = by + uy * L * 0.5f;

            nvgMoveTo(vg, bx - px * hw, by - py * hw);
            nvgQuadTo(vg, mxp - px * hw * 0.35f, myp - py * hw * 0.35f, tx, ty);
            nvgQuadTo(vg, mxp + px * hw * 0.35f, myp + py * hw * 0.35f, bx + px * hw, by + py * hw);
            nvgClosePath(vg);
        }
        nvgFillColor(vg, nvgRGB(0x05, 0x06, 0x09));
        nvgFill(vg);

        // Pass 2: edge glints along the spikes
        nvgBeginPath(vg);
        for (int i = 0; i < N_SPIKES; ++i) {
            const Spike& s = spikes[i];
            if (!s.vis || s.front != front) continue;

            const float bx = cx + s.bx, by = cy + s.by;
            const float tx = cx + s.tx, ty = cy + s.ty;
            const float dx = tx - bx, dy = ty - by;
            const float L = std::sqrt(dx * dx + dy * dy);
            if (L < 3.f) continue;
            const float ux = dx / L, uy = dy / L;
            nvgMoveTo(vg, bx - (-uy) * s.hw * 0.45f, by - ux * s.hw * 0.45f);
            nvgLineTo(vg, bx + dx * 0.96f, by + dy * 0.96f);
        }
        nvgStrokeWidth(vg, 0.6f);
        nvgLineCap(vg, NVG_ROUND);
        nvgStrokeColor(vg, nvgRGBA(190, 208, 235, (int)((front ? 110.f : 70.f) * gloss)));
        nvgStroke(vg);

        // Pass 3: sparkling tips where the spike catches the light
        for (int i = 0; i < N_SPIKES; ++i) {
            const Spike& s = spikes[i];
            if (!s.vis || s.front != front || s.spec < 0.12f) continue;
            nvgBeginPath(vg);
            nvgCircle(vg, cx + s.tx, cy + s.ty, 0.7f + 0.5f * s.spec);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(255.f * std::min(1.f, s.spec * 1.4f) * gloss * s.lit)));
            nvgFill(vg);
        }
    }

    void drawDrops(NVGcontext* vg, float cx, float cy, float gloss, bool front)
    {
        for (int i = 0; i < N_DROPS; ++i) {
            const Drop& d = drops[i];
            if ((d.z >= 0.f) != front) continue;
            if (d.vis < 0.02f) continue;

            const float rr = d.r * d.sc * std::sqrt(d.vis);
            const float px = cx + d.x, py = cy + d.y;

            nvgBeginPath(vg);
            nvgCircle(vg, px, py, rr);
            nvgFillPaint(vg, nvgRadialGradient(vg, px - rr * 0.3f, py - rr * 0.3f, 0.f, rr * 1.4f,
                                               nvgRGB(0x24, 0x2a, 0x35), nvgRGB(0x03, 0x04, 0x06)));
            nvgFill(vg);

            if (rr > 1.3f) {
                nvgBeginPath(vg);
                nvgCircle(vg, px - rr * 0.32f, py - rr * 0.38f, rr * 0.24f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(210.f * gloss)));
                nvgFill(vg);
            }
        }
    }

    void drawGlassOverlay(NVGcontext* vg, float cx, float cy, float R)
    {
        // Inner edge shading - gives the glass its depth
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, R);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, R * 0.72f, R,
                                           nvgRGBA(90, 104, 128, 0), nvgRGBA(90, 104, 128, 80)));
        nvgFill(vg);

        // Window reflection arc, top-left
        nvgBeginPath(vg);
        nvgArc(vg, cx, cy, R * 0.86f, (float)M_PI * 1.08f, (float)M_PI * 1.55f, NVG_CW);
        nvgStrokeWidth(vg, 2.6f);
        nvgLineCap(vg, NVG_ROUND);
        nvgStrokePaint(vg, nvgLinearGradient(vg, cx - R * 0.8f, cy - R * 0.1f, cx - R * 0.1f, cy - R * 0.85f,
                                             nvgRGBA(255, 255, 255, 190), nvgRGBA(255, 255, 255, 0)));
        nvgStroke(vg);

        // Small bounce highlight, lower-right
        nvgBeginPath(vg);
        nvgArc(vg, cx, cy, R * 0.90f, (float)M_PI * 0.15f, (float)M_PI * 0.38f, NVG_CW);
        nvgStrokeWidth(vg, 1.4f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 90));
        nvgStroke(vg);

        // Glass lip
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, R - 0.5f);
        nvgStrokeWidth(vg, 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 120));
        nvgStroke(vg);
    }

    void draw(const DrawArgs& args) override
    {
        NVGcontext* vg = args.vg;

        const double now = system::getTime();
        float dt = 1.f / 60.f;
        if (lastTime > 0.0)
            dt = clampT((float)(now - lastTime), 0.001f, 0.05f);
        lastTime = now;

        if (module) {
            const bool resetPressed = module->params[LifeformModule::RESET_PARAM].getValue() > 0.5f;
            if (resetPressed && !prevResetButton) resetLife();
            prevResetButton = resetPressed;
        }

        const bool paused = module && module->paused;
        if (!paused) stepPhysics(dt);

        const float cx = LF_DISP_W * 0.5f;
        const float cy = LF_DISP_H * 0.5f;
        const float R  = LF_DISH_R;
        const float gloss = module ? module->params[LifeformModule::GLOSS_PARAM].getValue() : 0.75f;

        drawDish(vg, cx, cy, R);

        nvgSave(vg);
        nvgScissor(vg, cx - R, cy - R, 2.f * R, 2.f * R);

        drawShadow(vg, cx, cy);

        // back-to-front: far spikes and droplets, the body, then near spikes and droplets
        drawSpikes(vg, cx, cy, gloss, false);
        drawDrops(vg, cx, cy, gloss, false);
        drawBody(vg, cx, cy, gloss);
        drawSpikes(vg, cx, cy, gloss, true);
        drawDrops(vg, cx, cy, gloss, true);

        nvgRestore(vg);

        drawGlassOverlay(vg, cx, cy, R);

        if (paused) {
            drawText(vg, 32.f, 12.f, "PAUSED", 7.f,
                     nvgRGBA(255, 220, 80, 220),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        }

        Widget::draw(args);
    }
};

// =====================================================================
//  Panel background
// =====================================================================
struct LFPanelBackground : TransparentWidget
{
    LFPanelBackground() { box.size = Vec(LF_PANEL_W, LF_PANEL_H); }

    void plate(NVGcontext* vg, float cx, float ry)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - 31.f, ry - 22.f, 62.f, 44.f, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt)
    {
        const float x1 = 14.f, x2 = LF_PANEL_W - 14.f, cx = LF_PANEL_W / 2.f;
        const float w = 10.f + (float)strlen(txt) * 7.f;
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

    void draw(const DrawArgs& args) override
    {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, LF_PANEL_W, LF_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, LF_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int)LF_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, LF_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, LF_PANEL_W - 1.f, LF_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        drawLogo(vg, LF_PANEL_W / 2.f, 13.f);
        drawText(vg, LF_PANEL_W / 2.f, 30.f, "LIFEFORM", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        const float bx = LF_DISP_X - 5.f, by = LF_DISP_Y - 5.f;
        const float bw = LF_DISP_W + 10.f, bh = LF_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        drawText(vg, 60.f,  LF_BTN_LABEL_Y, "PAUSE", 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        drawText(vg, 110.f, LF_BTN_LABEL_Y, "RESET", 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        drawText(vg, 160.f, LF_BTN_LABEL_Y, "AUDIO", 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);

        section(vg, 194.f, "MAGNETIC FIELD");
        section(vg, 266.f, "FLUID DYNAMICS");
        section(vg, 332.f, "I / O");

        for (int row = 0; row < 2; ++row) {
            const float ry = (row == 0) ? LF_ROW1_Y : LF_ROW2_Y;
            for (int col = 0; col < 3; ++col)
                plate(vg, LF_COL[col], ry);
        }

        for (int i = 0; i < 6; ++i) {
            const int row = i / 3, col = i % 3;
            const float ry = (row == 0) ? LF_ROW1_Y : LF_ROW2_Y;
            drawText(vg, LF_COL[col], ry + 14.f, LF_KNOBS[i].name, 8.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        const float ioX[4] = {40.f, 90.f, 180.f, 230.f};
        const char* ioLabels[4] = {"AUDIO", "PULSE", "ENV", "FIELD"};
        for (int i = 0; i < 4; ++i) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, ioX[i] - 20.f, LF_IO_Y - 18.f, 40.f, 36.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, ioX[i], LF_IO_Y - 24.f, ioLabels[i], 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        Widget::draw(args);
    }
};

} // namespace

// =====================================================================
//  Module widget
// =====================================================================
struct LifeformWidget : ModuleWidget
{
    explicit LifeformWidget(LifeformModule* module)
    {
        setModule(module);
        box.size = Vec(LF_PANEL_W, LF_PANEL_H);

        addChild(new LFPanelBackground());
        addChild(new LifeformDisplay(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - RACK_GRID_WIDTH, RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addParam(createParamCentered<TL1105>(Vec(60.f,  LF_BTN_Y), module, LifeformModule::PAUSE_PARAM));
        addParam(createParamCentered<TL1105>(Vec(110.f, LF_BTN_Y), module, LifeformModule::RESET_PARAM));
        addParam(createParamCentered<TL1105>(Vec(160.f, LF_BTN_Y), module, LifeformModule::AUDIO_PARAM));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(60.f,  LF_BTN_LED_Y), module, LifeformModule::PAUSE_LIGHT));
        addChild(createLightCentered<SmallLight<BlueLight>> (Vec(160.f, LF_BTN_LED_Y), module, LifeformModule::AUDIO_LIGHT));

        for (int i = 0; i < 6; ++i) {
            const int row = i / 3, col = i % 3;
            const float cx = LF_COL[col];
            const float ry = (row == 0) ? LF_ROW1_Y : LF_ROW2_Y;
            const float cy = (row == 0) ? LF_CV1_Y  : LF_CV2_Y;
            addParam(createParamCentered<RoundBlackKnob>(Vec(cx, ry - 6.f), module, LF_KNOBS[i].param));
            addInput(createInputCentered<PJ301MPort>(Vec(cx, cy), module, LF_KNOBS[i].cv));
        }

        const float ioX[4] = {40.f, 90.f, 180.f, 230.f};
        addInput (createInputCentered <PJ301MPort>(Vec(ioX[0], LF_IO_Y), module, LifeformModule::AUDIO_INPUT));
        addInput (createInputCentered <PJ301MPort>(Vec(ioX[1], LF_IO_Y), module, LifeformModule::PULSE_INPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(ioX[2], LF_IO_Y), module, LifeformModule::ENV_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(Vec(ioX[3], LF_IO_Y), module, LifeformModule::FIELD_OUTPUT));
    }
};

// =====================================================================
//  Model
// =====================================================================
Model* modelLifeform = createModel<LifeformModule, LifeformWidget>("Lifeform");