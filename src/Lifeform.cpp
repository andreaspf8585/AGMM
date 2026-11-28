#include "YouTubeLoad.hpp"
#include "GitLoad.hpp"
// Lifeform.cpp
// AGMM-styled ferrofluid visualiser.
//
// ============================================================================
// OVERVIEW
// ============================================================================
// Lifeform is an 18 HP audio-reactive visual module that renders a glossy black
// ferrofluid "lifeform" floating in a glass dish inside a perforated white
// bezel. The lifeform is real 3D geometry: a lobed blob, ~140 Rosensweig spikes
// and orbiting droplets, tumbling through space with perspective projection,
// depth sorting and normal-based gloss.
//
// Sound drives the visualisation through three bands (low / mid / high), an
// onset detector ("punch") and a silence detector. In silence everything merges
// into one calm blob; with audio the blob becomes active, spikes erupt, and
// droplets orbit around it. The magnet position, field strength and viscosity
// can be controlled by knobs and CV, and the module outputs an envelope and a
// field-level CV.
//
// File structure:
//   1. LifeformModule: audio analysis (envelope + three bands + pulse/onset),
//      parameter smoothing, and output generation.
//   2. Layout constants (18 HP = 270x380 px).
//   3. Anonymous-namespace UI:
//        - tiny 3D maths helpers (V3, M3, operators, dot/cross/norm)
//        - hash1, smooth01
//        - LifeformDisplay: the 3D ferrofluid renderer (physics + drawing)
//        - LFPanelBackground: static panel art
//   4. LifeformWidget: places buttons, knobs, jacks, lights and the display.
//   5. Model registration.
// ============================================================================

#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "Knob_arcs.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace rack;

// Clamps x into [lo, hi] for any arithmetic type.
template <typename T>
inline T clampT(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }

// =====================================================================
//  Module
// =====================================================================
struct LifeformModule : Module
{
    enum ParamId
    {
        FIELD_PARAM,
        MOTION_PARAM,
        VISC_PARAM,
        SPIKES_PARAM,
        SENS_PARAM,
        GLOSS_PARAM,
        PAUSE_PARAM,
        RESET_PARAM,
        AUDIO_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        AUDIO_INPUT,
        PULSE_INPUT,
        FIELD_CV,
        MOTION_CV,
        VISC_CV,
        SPIKES_CV,
        SENS_CV,
        GLOSS_CV,
        INPUTS_LEN
    };
    enum OutputId
    {
        ENV_OUTPUT,
        FIELD_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        PAUSE_LIGHT,
        AUDIO_LIGHT,
        LIGHTS_LEN
    };

    float env = 0.f;
    float pulseFlash = 0.f;
    float field = 0.f;
    float phase = 0.f;
    bool prevPulse = false;

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

    // Constructor: configures all parameters, inputs, outputs, lights and
    // bypass; sets up the atomic shared state used by the UI thread.
    LifeformModule()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(FIELD_PARAM, 0.f, 1.f, 0.55f, "Magnetic field");
        configParam(MOTION_PARAM, 0.f, 1.f, 0.45f, "Magnet motion");
        configParam(VISC_PARAM, 0.f, 1.f, 0.50f, "Fluid viscosity");
        configParam(SPIKES_PARAM, 0.f, 1.f, 0.65f, "Spike amplitude");
        configParam(SENS_PARAM, 0.f, 1.f, 0.65f, "Audio sensitivity");
        configParam(GLOSS_PARAM, 0.f, 1.f, 0.75f, "Gloss");

        configButton(PAUSE_PARAM, "Pause");
        configButton(RESET_PARAM, "Reset");
        configButton(AUDIO_PARAM, "Audio reactive");
        configLight(PAUSE_LIGHT, "Pause active");
        configLight(AUDIO_LIGHT, "Audio active");

        configInput(AUDIO_INPUT, "Audio");
        configInput(PULSE_INPUT, "Pulse trigger");
        configInput(FIELD_CV, "Field CV");
        configInput(MOTION_CV, "Motion CV");
        configInput(VISC_CV, "Viscosity CV");
        configInput(SPIKES_CV, "Spikes CV");
        configInput(SENS_CV, "Sensitivity CV");
        configInput(GLOSS_CV, "Gloss CV");

        configOutput(ENV_OUTPUT, "Envelope");
        configOutput(FIELD_OUTPUT, "Field level");

        configBypass(AUDIO_INPUT, ENV_OUTPUT);
    }

    // Rack reset hook: clears the audio analysis, smoothing state and atomics
    // so the visualiser returns to a neutral starting point.
    void onReset(const ResetEvent &e) override
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

    // Main audio callback: debounces the Pause / Audio-reactive buttons,
    // computes a fast/slow envelope, splits the input into low/mid/high bands
    // with envelope followers, detects pulse triggers, smooths the field
    // parameter (with audio boost and pulse flash), advances the magnet phase,
    // and publishes all state to the UI through the atomics and outputs.
    void process(const ProcessArgs &args) override
    {
        const float sr = args.sampleRate;

        const bool pausePressed = params[PAUSE_PARAM].getValue() > 0.5f;
        if (pausePressed && !prevPauseButton)
            paused = !paused;
        prevPauseButton = pausePressed;
        lights[PAUSE_LIGHT].setBrightness(paused ? 1.f : 0.f);

        const bool audioPressed = params[AUDIO_PARAM].getValue() > 0.5f;
        if (audioPressed && !prevAudioButton)
            audioReactive = !audioReactive;
        prevAudioButton = audioPressed;
        lights[AUDIO_LIGHT].setBrightness(audioReactive ? 1.f : 0.f);

        const float in = inputs[AUDIO_INPUT].getVoltage() / 5.f;
        const float rect = std::fabs(in);
        const float atk = 1.f - std::exp(-1.f / (0.004f * sr));
        const float rel = 1.f - std::exp(-1.f / (0.180f * sr));
        env += (rect - env) * (rect > env ? atk : rel);

        // ---- three-band envelope followers (low < 180 Hz < mid < 2.2 kHz < high) ----
        const float cLP1 = std::min(1.f, 2.f * (float)M_PI * 180.f / sr);
        const float cLP2 = std::min(1.f, 2.f * (float)M_PI * 2200.f / sr);
        lp1 += (in - lp1) * cLP1;
        lp2 += (in - lp2) * cLP2;
        const float bandLow = std::fabs(lp1) * 1.5f;
        const float bandMid = std::fabs(lp2 - lp1) * 1.6f;
        const float bandHigh = std::fabs(in - lp2) * 2.0f;
        // Simple asymmetric envelope follower for each band.
        auto follow = [sr](float &e, float x, float aT, float rT)
        {
            const float a = std::min(1.f, 1.f / (aT * sr));
            const float r = std::min(1.f, 1.f / (rT * sr));
            e += (x - e) * (x > e ? a : r);
        };
        follow(eLow, bandLow, 0.003f, 0.14f);
        follow(eMid, bandMid, 0.002f, 0.09f);
        follow(eHigh, bandHigh, 0.001f, 0.06f);

        const bool pulse = inputs[PULSE_INPUT].getVoltage() >= 1.f;
        if (pulse && !prevPulse)
            pulseFlash = 1.f;
        prevPulse = pulse;
        pulseFlash -= args.sampleTime * 3.5f;
        if (pulseFlash < 0.f)
            pulseFlash = 0.f;

        // Reads a knob plus its CV (0..10 V) and clamps the result to 0..1.
        auto v = [this](int p, int cv)
        {
            return clampT(params[p].getValue() + inputs[cv].getVoltage() / 10.f, 0.f, 1.f);
        };
        const float fieldKnob = v(FIELD_PARAM, FIELD_CV);
        const float motionKnob = v(MOTION_PARAM, MOTION_CV);
        const float sensKnob = v(SENS_PARAM, SENS_CV);

        if (!paused)
        {
            const float audioBoost = audioReactive ? env * sensKnob * 1.1f : 0.f;
            const float fieldTarget = clampT(fieldKnob + audioBoost + pulseFlash * 0.75f, 0.f, 1.f);
            // faster field response than before (was 55 ms)
            const float fieldCoef = 1.f - std::exp(-1.f / (0.025f * sr));
            field += (fieldTarget - field) * fieldCoef;

            phase += motionKnob * args.sampleTime * 1.35f;
            if (phase > 2.f * (float)M_PI)
                phase -= 2.f * (float)M_PI;
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
//  Layout (18 HP = 270 x 380)
//  Every control sits in a plate with its label above it.
// =====================================================================
static const float LF_PANEL_W = RACK_GRID_WIDTH * 18; // 270
static const float LF_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it): dish on top, readout row underneath
static const float LF_DISP_X = 20.f;
static const float LF_DISP_Y = 28.f;
static const float LF_DISP_W = 230.f;
static const float LF_AREA_H = 104.f; // dish area
static const float LF_DISP_H = 116.f; // + 12 px readout row

static const float LF_DISH_R = 41.f; // glass radius inside the bezel
static const float LF_CAM = 320.f;   // camera distance for perspective

// Three columns of 80 px plates
static const float LF_COL[3] = {48.f, 135.f, 222.f};
static const float LF_CELL_W = 80.f;

// Buttons row
static const float LF_BTN_TOP = 152.f, LF_BTN_H = 38.f; // label +9, button +25
// Knob rows: label +9 over the knob, "CV" label over the jack, knob and jack centred at +29
static const float LF_SEC1_Y = 199.f;
static const float LF_ROW_TOP[2] = {206.f, 258.f};
static const float LF_ROW_H = 48.f;
static const float LF_KNOB_DX = -14.f, LF_JACK_DX = 22.f;
static const float LF_CTRL_DY = 29.f;
// I/O
static const float LF_SEC2_Y = 315.f;
static const float LF_IO_TOP = 322.f, LF_IO_H = 36.f, LF_IO_W = 58.f; // label +8, jack +24
static const float LF_IO_X[4] = {37.f, 102.f, 167.f, 232.f};

namespace
{

    // Fast deterministic hash in [0,1) used for per-spike/per-droplet randoms.
    inline float hash1(float x)
    {
        const float s = std::sin(x * 127.1f + 311.7f) * 43758.5453f;
        return s - std::floor(s);
    }

    // Smoothstep between a and b; returns 0 below a, 1 above b, and a smooth
    // Hermite interpolation in between.
    inline float smooth01(float a, float b, float x)
    {
        const float t = clampT((x - a) / (b - a), 0.f, 1.f);
        return t * t * (3.f - 2.f * t);
    }

    // ---- tiny 3D maths ----
    // Minimal 3-component vector.
    struct V3
    {
        float x, y, z;
    };
    // Constructs a V3 from components.
    inline V3 v3(float x, float y, float z)
    {
        V3 r;
        r.x = x;
        r.y = y;
        r.z = z;
        return r;
    }
    // Vector addition.
    inline V3 operator+(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
    // Vector subtraction.
    inline V3 operator-(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
    // Scalar multiplication.
    inline V3 operator*(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
    // Dot product.
    inline float dot3(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    // Cross product.
    inline V3 cross3(V3 a, V3 b)
    {
        return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }
    // Euclidean length.
    inline float len3(V3 a) { return std::sqrt(dot3(a, a)); }
    // Returns a unit vector (with a small epsilon to avoid division by zero).
    inline V3 norm3(V3 a) { return a * (1.f / (len3(a) + 1e-6f)); }

    // Row-major 3x3 matrix.
    struct M3
    {
        float m[9];
    }; // row-major
    // Multiplies a 3x3 matrix by a V3.
    inline V3 m3mul(const M3 &M, V3 v)
    {
        return v3(M.m[0] * v.x + M.m[1] * v.y + M.m[2] * v.z,
                  M.m[3] * v.x + M.m[4] * v.y + M.m[5] * v.z,
                  M.m[6] * v.x + M.m[7] * v.y + M.m[8] * v.z);
    }

    // Knob table entry: display name, parameter ID and CV input ID.
    struct LFKnobDef
    {
        const char *name;
        int param;
        int cv;
    };
    const LFKnobDef LF_KNOBS[6] = {
        {"FIELD", LifeformModule::FIELD_PARAM, LifeformModule::FIELD_CV},
        {"MOTION", LifeformModule::MOTION_PARAM, LifeformModule::MOTION_CV},
        {"VISC", LifeformModule::VISC_PARAM, LifeformModule::VISC_CV},
        {"SPIKES", LifeformModule::SPIKES_PARAM, LifeformModule::SPIKES_CV},
        {"SENS", LifeformModule::SENS_PARAM, LifeformModule::SENS_CV},
        {"GLOSS", LifeformModule::GLOSS_PARAM, LifeformModule::GLOSS_CV},
    };

    // =====================================================================
    //  Ferrofluid display - 3D tumbling blob, spikes and droplets
    // =====================================================================
    // Renders the 3D ferrofluid lifeform: a lobed surface grid, Rosensweig
    // spikes, orbiting droplets, perspective projection, depth sorting and
    // normal-based gloss. Physics is stepped each frame from the module's
    // audio-reactive state.
    struct LifeformDisplay : TransparentWidget
    {
        enum
        {
            N_SPIKES = 140,
            N_DROPS = 14,
            LAT = 16,
            LON = 32,
            GN = LAT * LON,
            NB = 96
        };

        // One Rosensweig spike: a local surface direction, per-spike random
        // values, current height/velocity, and cached per-frame projected
        // geometry.
        struct Spike
        {
            V3 d;             // local unit direction on the body
            float rnd, phase; // per-spike randoms
            float h, v;       // height (0..~1.2) and velocity
            // per-frame projected geometry (px, relative to dish centre)
            float bx, by, tx, ty, hw, spec, lit;
            bool front, vis;
        };

        // One orbiting droplet: its orbital plane, angle/distance, radius and
        // cached projected position/scale/depth/visibility.
        struct Drop
        {
            V3 e1, e2;                // orbital plane basis (local)
            float ang, dist, rest, r; // orbit angle, distance (px), rest distance, radius
            float dv, w, phase;
            float x, y, sc, z, vis; // projected position, scale, depth, visibility
        };

        Spike spikes[N_SPIKES];
        Drop drops[N_DROPS];

        V3 ldir[GN];  // local unit directions of the surface grid
        V3 wpos[GN];  // world positions (relative to dish centre)
        V3 wnorm[GN]; // world normals
        float psx[GN], psy[GN], psc[GN];
        float sil[NB]; // projected silhouette radius per angular bin

        M3 Rm;                // body rotation
        V3 om;                // angular velocity (world frame)
        V3 Lv, Hv, H2v, magD; // key light, half vectors, magnet direction

        LifeformModule *module = nullptr;
        float t = 0.f;
        double lastTime = 0.0;
        bool prevResetButton = false;
        float prevPulseVal = 0.f;
        float prevEnvFrame = 0.f;
        float onsetCool = 0.f;

        // Shared geometry / state, updated by stepPhysics, consumed by the draw code
        float bodyBase = 11.f;
        float wobAmt = 0.08f;
        float stretch = 0.f;
        float magA = 0.f;
        float magLen = 0.f;
        float offX = 0.f, offY = 0.f;
        float offVX = 0.f, offVY = 0.f;

        float act = 0.f;     // smoothed audio activity (0 = silence)
        float calmK = 1.f;   // 1 = fully merged single blob
        float wt = 0.f;      // warped shape time (faster with sound)
        float spinVel = 0.f; // |angular velocity|
        float ripAmt = 0.f;
        float ell = 0.12f;
        float punch = 0.f; // transient energy (onset detector)

        // Constructor: initialises the surface grid, spikes and droplets,
        // sets up the lighting and magnet directions, and performs one physics
        // step so the first frame has valid geometry.
        explicit LifeformDisplay(LifeformModule *m) : module(m)
        {
            box.pos = Vec(LF_DISP_X, LF_DISP_Y);
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

            Lv = norm3(v3(-0.5f, -0.6f, 0.62f));
            Hv = norm3(Lv + v3(0.f, 0.f, 1.f));
            const V3 L2 = norm3(v3(0.6f, 0.5f, 0.55f));
            H2v = norm3(L2 + v3(0.f, 0.f, 1.f));
            magD = norm3(v3(0.8f, 0.f, 0.6f));

            // surface grid (y is the pole axis)
            for (int i = 0; i < LAT; ++i)
            {
                const float th = (float)M_PI * ((float)i + 0.5f) / (float)LAT;
                for (int j = 0; j < LON; ++j)
                {
                    const float ph = 2.f * (float)M_PI * (float)j / (float)LON;
                    ldir[i * LON + j] = v3(std::sin(th) * std::cos(ph), std::cos(th), std::sin(th) * std::sin(ph));
                }
            }

            // spikes on a Fibonacci sphere
            for (int i = 0; i < N_SPIKES; ++i)
            {
                Spike &s = spikes[i];
                const float y = 1.f - 2.f * ((float)i + 0.5f) / (float)N_SPIKES;
                const float rr = std::sqrt(std::max(0.f, 1.f - y * y));
                const float ph = (float)i * 2.399963f;
                s.d = v3(std::cos(ph) * rr, y, std::sin(ph) * rr);
                s.rnd = hash1(i * 3.3f + 1.f);
                s.phase = hash1(i * 5.1f) * 2.f * (float)M_PI;
                s.h = s.v = 0.f;
                s.bx = s.by = s.tx = s.ty = s.hw = s.spec = s.lit = 0.f;
                s.front = true;
                s.vis = false;
            }

            // droplets with individual tilted orbital planes
            for (int i = 0; i < N_DROPS; ++i)
            {
                Drop &d = drops[i];
                const V3 n = norm3(v3(hash1(i * 1.9f) - 0.5f, hash1(i * 2.9f + 1.f) - 0.5f, hash1(i * 3.9f + 2.f) - 0.5f));
                const V3 helper = (std::fabs(n.y) < 0.9f) ? v3(0.f, 1.f, 0.f) : v3(1.f, 0.f, 0.f);
                d.e1 = norm3(cross3(n, helper));
                d.e2 = cross3(n, d.e1);
                d.ang = hash1(i * 9.1f) * 2.f * (float)M_PI;
                d.rest = LF_DISH_R * (0.62f + 0.26f * hash1(i * 2.2f));
                d.dist = d.rest;
                d.r = (i < 3) ? (2.4f + 1.8f * hash1(i * 4.4f))
                              : (0.7f + 1.3f * hash1(i * 6.6f));
                d.dv = 0.f;
                d.w = (hash1(i * 7.7f) - 0.5f) * 0.9f;
                d.phase = hash1(i * 8.8f) * 6.2832f;
                d.x = d.y = d.z = 0.f;
                d.sc = 1.f;
                d.vis = 1.f;
            }

            stepPhysics(1.f / 60.f);
        }

        // Resets the spike heights/velocities and droplet distances/velocities
        // to their rest state (used by the RESET button).
        void resetLife()
        {
            for (int i = 0; i < N_SPIKES; ++i)
            {
                spikes[i].h = 0.f;
                spikes[i].v = 0.f;
            }
            for (int i = 0; i < N_DROPS; ++i)
            {
                drops[i].dist = drops[i].rest;
                drops[i].dv = 0.f;
            }
            punch = 0.f;
        }

        // ---- 3D helpers ----
        // Projects a world-space point P to 2D (x, y) and a scale factor `s`
        // using a simple perspective camera at z = LF_CAM.
        static void project(const V3 &P, float &x, float &y, float &s)
        {
            s = LF_CAM / (LF_CAM - P.z);
            x = P.x * s;
            y = P.y * s;
        }

        // Rotates the body orientation Rm by an angular velocity vector w over
        // dt, using the Rodrigues formula, and re-orthonormalises the matrix
        // to prevent numerical drift.
        void rotateBy(V3 w, float dt)
        {
            const float wl = len3(w);
            const float ang = wl * dt;
            if (ang < 1e-6f)
                return;
            const V3 k = w * (1.f / wl);
            const float c = std::cos(ang), s = std::sin(ang), C = 1.f - c;
            const float Q[9] = {
                c + k.x * k.x * C, k.x * k.y * C - k.z * s, k.x * k.z * C + k.y * s,
                k.y * k.x * C + k.z * s, c + k.y * k.y * C, k.y * k.z * C - k.x * s,
                k.z * k.x * C - k.y * s, k.z * k.y * C + k.x * s, c + k.z * k.z * C};
            float N[9];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                {
                    float sum = 0.f;
                    for (int l = 0; l < 3; ++l)
                        sum += Q[i * 3 + l] * Rm.m[l * 3 + j];
                    N[i * 3 + j] = sum;
                }
            // re-orthonormalise to stop drift
            V3 r0 = norm3(v3(N[0], N[1], N[2]));
            V3 r1 = v3(N[3], N[4], N[5]);
            r1 = norm3(r1 - r0 * dot3(r1, r0));
            const V3 r2 = cross3(r0, r1);
            Rm.m[0] = r0.x;
            Rm.m[1] = r0.y;
            Rm.m[2] = r0.z;
            Rm.m[3] = r1.x;
            Rm.m[4] = r1.y;
            Rm.m[5] = r1.z;
            Rm.m[6] = r2.x;
            Rm.m[7] = r2.y;
            Rm.m[8] = r2.z;
        }

        // Surface radius (px) for a local direction d with world direction wd
        // Returns the radius of the body surface at a given local direction,
        // including the wobble/ripple/ellipse/stretch terms and magnet
        // alignment, clamped so it stays inside the glass.
        float surfaceR(const V3 &d, const V3 &wd) const
        {
            const float n = std::sin(2.1f * d.x + 1.7f * d.y + wt * 0.90f) * 0.50f + std::sin(3.2f * d.y - 2.4f * d.z - wt * 1.30f + 1.7f) * 0.30f + std::sin(1.6f * d.z + 2.2f * d.x + wt * 0.55f) * 0.40f + std::sin(5.0f * d.x + 4.1f * d.y + 3.7f * d.z + wt * 2.10f) * 0.12f;
            const float al = std::max(0.f, dot3(wd, magD));
            const float r = bodyBase * (1.f + wobAmt * n + ripAmt * std::sin(7.f * d.y - wt * 7.f + 2.f * d.x) + ell * (1.5f * d.x * d.x - 0.5f) + stretch * al * al * al);
            const float oLen = std::sqrt(offX * offX + offY * offY);
            return std::min(r, LF_DISH_R * 0.80f - oLen);
        }

        // Advances the visual physics by dt: reads the module's smoothed audio
        // state, runs the onset detector, updates the body shape/tumble/offset,
        // integrates the spikes and droplets, and finally rebuilds the
        // projected geometry for drawing.
        void stepPhysics(float dt)
        {
            t += dt;
            const float R = LF_DISH_R;
            const float twoPi = 2.f * (float)M_PI;
            (void)twoPi;

            const float field = module ? module->fieldSmoothed.load() : 0.58f;
            const float env = module ? module->envSmoothed.load() : 0.1f;
            const float pulse = module ? module->pulseSmoothed.load() : 0.f;
            const float low = module ? module->lowSmoothed.load() : 0.20f;
            const float mid = module ? module->midSmoothed.load() : 0.10f;
            const float high = module ? module->highSmoothed.load() : 0.05f;
            const float mx = module ? module->magnetX.load() : std::cos(t * 0.6f) * 0.5f;
            const float my = module ? module->magnetY.load() : std::sin(t * 0.8f) * 0.5f;

            const float visc = module ? module->params[LifeformModule::VISC_PARAM].getValue() : 0.5f;
            const float spikeAmt = module ? module->params[LifeformModule::SPIKES_PARAM].getValue() : 0.65f;
            const float sens = module ? module->params[LifeformModule::SENS_PARAM].getValue() : 0.65f;
            const float motion = module ? module->params[LifeformModule::MOTION_PARAM].getValue() : 0.45f;
            const bool audio = module ? module->audioReactive : true;

            const float criticalField = 0.28f;
            const float audioEnv = audio ? env * sens : 0.f;
            const float lowA = audio ? low * sens : 0.f;
            const float midA = audio ? mid * sens : 0.f;
            const float highA = audio ? high * sens : 0.f;

            // ---- Onset detector ("punch"): transients hit the body hard ----
            punch *= std::exp(-dt * 6.f);
            onsetCool -= dt;
            float onset = 0.f;
            if (audio)
            {
                const float dEnv = env - prevEnvFrame;
                onset = std::max(0.f, dEnv * (0.016f / std::max(dt, 0.004f)) * (0.5f + sens) - 0.025f);
            }
            prevEnvFrame = env;
            punch = std::min(punch + onset * 1.6f, 1.2f);
            bool hit = false;
            if (onset > 0.12f && onsetCool <= 0.f)
            {
                hit = true;
                onsetCool = 0.07f;
            }

            // ---- Activity: silence collapses everything into one calm blob ----
            // (with Audio-reactive switched off the lifeform stays fully active)
            const float level = env * (0.4f + sens) + punch * 0.2f;
            const float actTarget = audio ? std::max(smooth01(0.008f, 0.09f, level), pulse) : 1.f;
            const float actRate = (actTarget > act) ? 30.f : 1.0f; // instant bloom, slow merge
            act += (actTarget - act) * (1.f - std::exp(-actRate * dt));
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
            if (kick || hit)
            {
                const V3 rv = norm3(v3(hash1(t * 13.1f + 1.f) - 0.5f,
                                       hash1(t * 7.7f + 3.f) - 0.5f,
                                       hash1(t * 3.3f + 5.f) - 0.5f));
                om = om + rv * (kick ? 3.f : (1.2f + 2.f * onset));
            }
            spinVel = len3(om);
            if (spinVel > 5.f)
            {
                om = om * (5.f / spinVel);
                spinVel = 5.f;
            }
            rotateBy(om, dt);

            // ---- Body geometry ----
            magLen = clampT(std::sqrt(mx * mx + my * my) / 0.55f, 0.f, 1.2f);
            magA = std::atan2(my, mx);
            magD = norm3(v3(std::cos(magA) * 0.8f, std::sin(magA) * 0.8f, 0.6f));

            bodyBase = R * (0.26f + 0.09f * field * (0.4f + 0.6f * act) + 0.10f * lowA + 0.04f * audioEnv + 0.02f * pulse + 0.05f * punch) * (1.f + 0.20f * calmK + 0.03f * std::sin(wt * 1.7f)); // merged blob is bigger
            wobAmt = 0.08f + 0.06f * field * act + 0.05f * audioEnv + 0.08f * midA;
            ripAmt = 0.12f * midA + 0.10f * pulse + 0.06f * punch;
            ell = 0.12f + 0.08f * act;
            stretch = clampT(0.20f * magLen * (0.4f + field) * (0.35f + 0.65f * act), 0.f, 0.30f);

            // Centre of mass follows the magnet with inertia -> overshoot and slosh
            const float cAmt = 0.45f + 0.55f * act;
            const float tx = (mx * R * 0.30f + std::cos(wt * 0.7f) * R * 0.04f) * cAmt;
            const float ty = (my * R * 0.30f + std::sin(wt * 0.9f) * R * 0.04f) * cAmt;
            if (hit)
            {
                offVX += (hash1(t * 5.1f) - 0.5f) * 30.f * (0.5f + onset);
                offVY += (hash1(t * 6.3f) - 0.5f) * 30.f * (0.5f + onset);
            }
            const float ck = 14.f, cd = 2.5f + visc * 4.f;
            offVX += (ck * (tx - offX) - cd * offVX) * dt;
            offVY += (ck * (ty - offY) - cd * offVY) * dt;
            offX += offVX * dt;
            offY += offVY * dt;
            const float oMax = R * 0.30f;
            offX = clampT(offX, -oMax, oMax);
            offY = clampT(offY, -oMax, oMax);

            // ---- Spikes ----
            for (int i = 0; i < N_SPIKES; ++i)
            {
                Spike &s = spikes[i];

                const V3 wd = m3mul(Rm, s.d);
                const float align = 0.5f + 0.5f * dot3(wd, magD);
                const float gate = 0.25f + 0.75f * act; // silence -> no spikes at any field
                const float localField = (field * (0.80f + 0.35f * align * magLen) + audioEnv * (0.45f + 0.70f * align) + highA * (0.15f + 0.55f * s.rnd) + pulse * 0.20f + punch * 0.30f) * gate;

                // Hysteresis: formed spikes need less field to persist
                const float threshold = (s.h > 0.05f) ? criticalField * 0.85f : criticalField;

                float target = 0.f;
                if (localField > threshold)
                {
                    const float drive = localField - threshold;
                    const float clump = 0.65f + 0.35f * std::sin(s.d.x * 5.f + s.d.z * 3.f + wt * 0.8f);
                    const float wave = 1.f + 0.28f * act * std::sin(wd.x * 3.f - wd.y * 2.f - wt * 2.4f);
                    const float fizz = 1.f + 0.5f * highA * std::sin(wt * 14.f + s.phase * 7.f);
                    target = std::sqrt(drive) * 2.0f * spikeAmt * (0.45f + 0.55f * s.rnd) * clump * wave * fizz * (1.f + 0.5f * punch) * (0.90f + 0.10f * std::sin(wt * 3.f + s.phase));
                    target = std::min(target, 1.2f);
                }

                const float k = 60.f * (1.1f - visc * 0.4f) * (1.f + 1.2f * audioEnv);
                const float damp = (8.f + visc * 8.f) * (1.f + 0.6f * audioEnv);
                s.v += (k * (target - s.h) - damp * s.v) * dt;
                s.h += s.v * dt;
                if (s.h < 0.f)
                {
                    s.h = 0.f;
                    s.v = 0.f;
                }
                if (s.h > 1.3f)
                {
                    s.h = 1.3f;
                    s.v = 0.f;
                }
            }

            // ---- Satellite droplets (3D orbits) ----
            // Silence (calmK) pulls every droplet back into the main body
            const float pull = std::max(smooth01(0.30f, 0.85f, field * (0.5f + 0.5f * act)), calmK);
            const V3 off3 = v3(offX, offY, 0.f);

            for (int i = 0; i < N_DROPS; ++i)
            {
                Drop &d = drops[i];

                if (kick || hit)
                    d.dv += (kick ? 38.f : 22.f * (0.5f + onset)) * (0.4f + hash1(i * 1.3f + t));

                float tgt = d.rest * (1.f - pull) + bodyBase * 0.45f * pull;
                tgt += audioEnv * 3.f * std::sin(wt * 8.f + d.phase);
                tgt += highA * 4.f * std::sin(wt * 17.f + d.phase * 2.f) * (1.f - pull);
                tgt += 2.5f * (1.f - pull) * std::sin(wt * 1.3f + d.phase); // lazy bobbing

                const float k = 26.f;
                const float damp = 5.f + visc * 7.f;
                d.dv += (k * (tgt - d.dist) - damp * d.dv) * dt;
                d.dist += d.dv * dt;
                d.dist = clampT(d.dist, 0.f, R * 0.9f);

                d.ang += ((d.w * 2.f + spinVel * (0.3f + 0.4f * d.phase / 6.2832f)) * (1.f - pull)) * dt;

                const float ca = std::cos(d.ang), sa = std::sin(d.ang);
                const V3 local = d.e1 * (ca * d.dist) + d.e2 * (sa * d.dist);
                V3 P = m3mul(Rm, local) + off3 + magD * (field * magLen * act * R * 0.10f * (1.f - pull));

                project(P, d.x, d.y, d.sc);
                d.z = P.z;

                const float lim = R * 0.93f - d.r * d.sc;
                const float len = std::sqrt(d.x * d.x + d.y * d.y);
                if (len > lim && len > 0.001f)
                {
                    d.x *= lim / len;
                    d.y *= lim / len;
                }

                // droplets that sink into the body shrink away (merge)
                d.vis = smooth01(0.85f * bodyBase, 1.15f * bodyBase, d.dist);
            }

            buildGeometry();
        }

        // Build the projected 3D scene for this frame
        // Computes world positions and normals of the surface grid, the
        // projected silhouette, and the projected geometry of every spike.
        void buildGeometry()
        {
            const float R = LF_DISH_R;
            const V3 off3 = v3(offX, offY, 0.f);
            const float oLen = std::sqrt(offX * offX + offY * offY);

            // ---- surface points ----
            for (int i = 0; i < GN; ++i)
            {
                const V3 d = ldir[i];
                const V3 wd = m3mul(Rm, d);
                wpos[i] = wd * surfaceR(d, wd) + off3;
            }

            // ---- normals from grid differences ----
            for (int i = 0; i < LAT; ++i)
            {
                for (int j = 0; j < LON; ++j)
                {
                    const int id = i * LON + j;
                    const int jp = i * LON + ((j + 1) % LON);
                    const int jm = i * LON + ((j + LON - 1) % LON);
                    const int ip = std::min(i + 1, (int)LAT - 1) * LON + j;
                    const int im = std::max(i - 1, 0) * LON + j;
                    wnorm[id] = norm3(cross3(wpos[jp] - wpos[jm], wpos[ip] - wpos[im]));
                }
            }

            // ---- projected silhouette ----
            for (int b = 0; b < NB; ++b)
                sil[b] = 0.f;
            for (int i = 0; i < GN; ++i)
            {
                project(wpos[i], psx[i], psy[i], psc[i]);
                const float dx = psx[i] - offX, dy = psy[i] - offY;
                const float rad = std::sqrt(dx * dx + dy * dy);
                float a = std::atan2(dy, dx);
                if (a < 0.f)
                    a += 2.f * (float)M_PI;
                int bin = (int)(a / (2.f * (float)M_PI) * NB);
                if (bin >= NB)
                    bin = NB - 1;
                if (rad > sil[bin])
                    sil[bin] = rad;
            }
            for (int b = 0; b < NB; ++b)
                if (sil[b] <= 0.f)
                    sil[b] = bodyBase;
            float tmp[NB];
            for (int b = 0; b < NB; ++b)
                tmp[b] = 0.25f * sil[(b + NB - 1) % NB] + 0.5f * sil[b] + 0.25f * sil[(b + 1) % NB];
            const float silMax = R * 0.92f - oLen;
            for (int b = 0; b < NB; ++b)
                sil[b] = std::min(tmp[b], silMax);

            // ---- spikes ----
            const float lean = magLen * 0.25f * act;
            for (int i = 0; i < N_SPIKES; ++i)
            {
                Spike &s = spikes[i];
                const V3 wd = m3mul(Rm, s.d);
                const float rb = surfaceR(s.d, wd) - 1.2f;
                const V3 base = wd * rb + off3;
                const V3 nrm = norm3(wd + magD * lean + cross3(om, wd) * 0.10f);
                const float hpx = s.h * R * 0.20f;

                s.vis = hpx > 0.6f;
                if (!s.vis)
                    continue;

                const V3 tip = base + nrm * hpx;
                float sb, st;
                project(base, s.bx, s.by, sb);
                project(tip, s.tx, s.ty, st);

                // keep tips inside the glass
                const float lim = R * 0.95f;
                const float dB = std::sqrt(s.bx * s.bx + s.by * s.by);
                const float dT = std::sqrt(s.tx * s.tx + s.ty * s.ty);
                if (dT > lim)
                {
                    const float k = (dB < lim && dT > dB) ? (lim - dB) / (dT - dB) : 0.f;
                    s.tx = s.bx + (s.tx - s.bx) * k;
                    s.ty = s.by + (s.ty - s.by) * k;
                }

                s.front = nrm.z >= 0.f;
                s.hw = (0.9f + 1.5f * std::min(1.f, hpx / 8.f)) * sb;
                s.spec = std::pow(std::max(0.f, dot3(nrm, Hv)), 16.f);
                s.lit = 0.35f + 0.65f * std::max(0.f, dot3(nrm, Lv));
            }
        }

        // ----- Drawing (flat fills only) -----
        // Draws the white bezel ring, the perforation dots, the groove and the
        // glass fill for the dish.
        void drawDish(NVGcontext *vg, float cx, float cy, float R)
        {
            // Light bezel ring
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, R + 10.f);
            nvgFillColor(vg, nvgRGB(0xc4, 0xcb, 0xd6));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Perforation ring
            nvgBeginPath(vg);
            const int dots = 84;
            for (int i = 0; i < dots; ++i)
            {
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

            // Glass
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, R);
            nvgFillColor(vg, nvgRGB(0xea, 0xee, 0xf4));
            nvgFill(vg);
        }

        // Draws the soft contact shadow beneath the body, following its
        // offset and size.
        void drawShadow(NVGcontext *vg, float cx, float cy)
        {
            const float bb = bodyBase;
            nvgBeginPath(vg);
            nvgEllipse(vg, cx + offX * 0.6f + 1.5f, cy + offY * 0.6f + bb * 1.05f + 3.f, bb * 1.2f, bb * 0.4f);
            nvgFillColor(vg, nvgRGBA(40, 52, 76, 40));
            nvgFill(vg);
        }

        // Builds a closed path along the projected silhouette polygon.
        void silhouettePath(NVGcontext *vg, float bcx, float bcy)
        {
            nvgBeginPath(vg);
            for (int b = 0; b < NB; ++b)
            {
                const float a = 2.f * (float)M_PI * (float)b / (float)NB;
                const float x = bcx + std::cos(a) * sil[b];
                const float y = bcy + std::sin(a) * sil[b];
                if (b == 0)
                    nvgMoveTo(vg, x, y);
                else
                    nvgLineTo(vg, x, y);
            }
            nvgClosePath(vg);
        }

        // Draws the black body silhouette with a thin cool rim, plus the
        // normal-based specular highlights that slide over the lobes.
        void drawBody(NVGcontext *vg, float cx, float cy, float gloss)
        {
            const float bcx = cx + offX, bcy = cy + offY;

            // Black body (flat) with a thin cool rim
            silhouettePath(vg, bcx, bcy);
            nvgFillColor(vg, nvgRGB(0x0a, 0x0d, 0x13));
            nvgFill(vg);
            silhouettePath(vg, bcx, bcy);
            nvgStrokeWidth(vg, 1.f);
            nvgStrokeColor(vg, nvgRGBA(130, 180, 255, (int)(90.f * gloss)));
            nvgStroke(vg);

            // Normal-based reflections: highlights slide over the lobes as the body tumbles
            for (int i = 0; i < GN; ++i)
            {
                const V3 &N = wnorm[i];
                if (N.z < 0.25f)
                    continue;
                const float sc = psc[i];
                const float px = cx + psx[i], py = cy + psy[i];

                const float dL = dot3(N, Lv);
                if (dL > 0.55f)
                { // broad sky sheen
                    const float sh = std::pow(dL, 8.f);
                    nvgBeginPath(vg);
                    nvgCircle(vg, px, py, 4.2f * sc);
                    nvgFillColor(vg, nvgRGBA(215, 232, 255, (int)(34.f * sh * gloss)));
                    nvgFill(vg);
                }

                const float dH = dot3(N, Hv);
                if (dH > 0.80f)
                { // sharp key-light specular
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
                if (dH2 > 0.78f)
                { // cool fill reflection
                    const float sp = std::pow(dH2, 18.f);
                    nvgBeginPath(vg);
                    nvgCircle(vg, px, py, 2.4f * sc);
                    nvgFillColor(vg, nvgRGBA(120, 175, 255, (int)(95.f * sp * gloss)));
                    nvgFill(vg);
                }
            }
        }

        // Draws the spikes for either the back (front=false) or the front
        // (front=true) of the body: first a combined dark silhouette pass, then
        // edge glints, then bright tips where they catch the light.
        void drawSpikes(NVGcontext *vg, float cx, float cy, float gloss, bool front)
        {
            // Pass 1: dark silhouettes (single path)
            nvgBeginPath(vg);
            for (int i = 0; i < N_SPIKES; ++i)
            {
                const Spike &s = spikes[i];
                if (!s.vis || s.front != front)
                    continue;

                const float bx = cx + s.bx, by = cy + s.by;
                const float tx = cx + s.tx, ty = cy + s.ty;
                const float dx = tx - bx, dy = ty - by;
                const float L = std::sqrt(dx * dx + dy * dy);

                if (L < 1.0f)
                { // seen end-on: a small dark cone tip
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
            for (int i = 0; i < N_SPIKES; ++i)
            {
                const Spike &s = spikes[i];
                if (!s.vis || s.front != front)
                    continue;

                const float bx = cx + s.bx, by = cy + s.by;
                const float tx = cx + s.tx, ty = cy + s.ty;
                const float dx = tx - bx, dy = ty - by;
                const float L = std::sqrt(dx * dx + dy * dy);
                if (L < 3.f)
                    continue;
                const float ux = dx / L, uy = dy / L;
                nvgMoveTo(vg, bx - (-uy) * s.hw * 0.45f, by - ux * s.hw * 0.45f);
                nvgLineTo(vg, bx + dx * 0.96f, by + dy * 0.96f);
            }
            nvgStrokeWidth(vg, 0.6f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStrokeColor(vg, nvgRGBA(190, 208, 235, (int)((front ? 110.f : 70.f) * gloss)));
            nvgStroke(vg);

            // Pass 3: sparkling tips where the spike catches the light
            for (int i = 0; i < N_SPIKES; ++i)
            {
                const Spike &s = spikes[i];
                if (!s.vis || s.front != front || s.spec < 0.12f)
                    continue;
                nvgBeginPath(vg);
                nvgCircle(vg, cx + s.tx, cy + s.ty, 0.7f + 0.5f * s.spec);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(255.f * std::min(1.f, s.spec * 1.4f) * gloss * s.lit)));
                nvgFill(vg);
            }
        }

        // Draws the orbiting droplets for either the back or the front of the
        // body (as determined by their depth z), with a small specular dot.
        void drawDrops(NVGcontext *vg, float cx, float cy, float gloss, bool front)
        {
            for (int i = 0; i < N_DROPS; ++i)
            {
                const Drop &d = drops[i];
                if ((d.z >= 0.f) != front)
                    continue;
                if (d.vis < 0.02f)
                    continue;

                const float rr = d.r * d.sc * std::sqrt(d.vis);
                const float px = cx + d.x, py = cy + d.y;

                nvgBeginPath(vg);
                nvgCircle(vg, px, py, rr);
                nvgFillColor(vg, nvgRGB(0x0a, 0x0d, 0x13));
                nvgFill(vg);

                if (rr > 1.3f)
                {
                    nvgBeginPath(vg);
                    nvgCircle(vg, px - rr * 0.32f, py - rr * 0.38f, rr * 0.24f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(210.f * gloss)));
                    nvgFill(vg);
                }
            }
        }

        // Main draw entry point: advances the frame (unless paused), draws the
        // dish, then back-to-front spikes/droplets/body/front spikes/droplets,
        // the glass lip, and the readout row (FIELD / status / ENV).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            const double now = system::getTime();
            float dt = 1.f / 60.f;
            if (lastTime > 0.0)
                dt = clampT((float)(now - lastTime), 0.001f, 0.05f);
            lastTime = now;

            if (module)
            {
                const bool resetPressed = module->params[LifeformModule::RESET_PARAM].getValue() > 0.5f;
                if (resetPressed && !prevResetButton)
                    resetLife();
                prevResetButton = resetPressed;
            }

            const bool paused = module && module->paused;
            if (!paused)
                stepPhysics(dt);

            agmm::screen(vg, 0.f, 0.f, LF_DISP_W, LF_DISP_H);

            const float cx = LF_DISP_W * 0.5f;
            const float cy = LF_AREA_H * 0.5f;
            const float R = LF_DISH_R;
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

            // Glass lip
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, R - 0.5f);
            nvgStrokeWidth(vg, 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 120));
            nvgStroke(vg);

            // Divider + readout row (own row, never over the dish)
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, LF_AREA_H + 0.5f);
            nvgLineTo(vg, LF_DISP_W, LF_AREA_H + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            const float ry = LF_AREA_H + 7.f;
            char buf[32];
            const float fld = module ? module->fieldSmoothed.load() : 0.58f;
            const float env = module ? module->envSmoothed.load() : 0.1f;
            snprintf(buf, sizeof(buf), "FIELD %3.0f%%", clampT(fld, 0.f, 1.f) * 100.f);
            agmm::text(vg, 6.f, ry, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "ENV %4.1fV", clampT(env * 10.f, 0.f, 10.f));
            agmm::text(vg, LF_DISP_W - 6.f, ry, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);
            const char *st = paused ? "PAUSED" : (calmK > 0.5f ? "MERGED" : "ACTIVE");
            agmm::text(vg, LF_DISP_W * 0.5f, ry, st, 7.f, paused ? agmm::warm() : agmm::dim(), agmm::ALIGN_C, 1.f);

            Widget::draw(args);
        }
    };

    // =====================================================================
    //  Panel background
    // =====================================================================
    // Static artwork: base panel, display bezel, the three button cells, the
    // two rows of knob/CV plates, and the four I/O plates.
    struct LFPanelBackground : TransparentWidget
    {
        LFPanelBackground() { box.size = Vec(LF_PANEL_W, LF_PANEL_H); }

        // Draws all static panel art: base panel, bezel, the PAUSE/RESET/AUDIO
        // buttons, the CONTROLS section with its six knob/CV plates, and the
        // I/O section with its four plates.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, LF_PANEL_W, LF_PANEL_H, "LIFEFORM");
            agmm::bezel(vg, LF_DISP_X, LF_DISP_Y, LF_DISP_W, LF_DISP_H);

            // Buttons
            const char *btn[3] = {"PAUSE", "RESET", "AUDIO"};
            for (int i = 0; i < 3; ++i)
            {
                agmm::plate(vg, LF_COL[i], LF_BTN_TOP, LF_CELL_W, LF_BTN_H, false);
                agmm::text(vg, LF_COL[i], LF_BTN_TOP + 9.f, btn[i], 8.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            // Knobs, each with its CV jack beside it
            agmm::sectionHeader(vg, LF_PANEL_W, LF_SEC1_Y, "CONTROLS", 10.f);
            for (int i = 0; i < 6; ++i)
            {
                const int row = i / 3, col = i % 3;
                const float top = LF_ROW_TOP[row];
                agmm::plate(vg, LF_COL[col], top, LF_CELL_W, LF_ROW_H, false);
                agmm::text(vg, LF_COL[col] + LF_KNOB_DX, top + 9.f, LF_KNOBS[i].name, 8.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                agmm::text(vg, LF_COL[col] + LF_JACK_DX, top + 9.f, "CV", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            // I/O: inputs plain, outputs tinted
            agmm::sectionHeader(vg, LF_PANEL_W, LF_SEC2_Y, "I / O", 10.f);
            const char *io[4] = {"AUDIO", "PULSE", "ENV", "FIELD"};
            for (int i = 0; i < 4; ++i)
            {
                agmm::plate(vg, LF_IO_X[i], LF_IO_TOP, LF_IO_W, LF_IO_H, i >= 2);
                agmm::text(vg, LF_IO_X[i], LF_IO_TOP + 8.f, io[i], 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            Widget::draw(args);
        }
    };

} // namespace

// =====================================================================
//  Module widget
// =====================================================================
// Places the static panel, the live LifeformDisplay, screws, the three
// buttons with their lights, the six knob/CV pairs and the four I/O jacks.
struct LifeformWidget : ModuleWidget
{
    // Builds the widget tree: background, display, screws, PAUSE/RESET/AUDIO
    // buttons plus the PAUSE/AUDIO lights, the six CONTROL knob/CV pairs, and
    // the AUDIO/PULSE inputs and ENV/FIELD outputs.
    explicit LifeformWidget(LifeformModule *module)
    {
        setModule(module);
        box.size = Vec(LF_PANEL_W, LF_PANEL_H);

        addChild(new LFPanelBackground());
        addChild(new LifeformDisplay(module));

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        const float by = LF_BTN_TOP + 25.f;
        addParam(createParamCentered<TL1105>(Vec(LF_COL[0], by), module, LifeformModule::PAUSE_PARAM));
        addParam(createParamCentered<TL1105>(Vec(LF_COL[1], by), module, LifeformModule::RESET_PARAM));
        addParam(createParamCentered<TL1105>(Vec(LF_COL[2], by), module, LifeformModule::AUDIO_PARAM));
        addChild(createLightCentered<SmallLight<agmm::VioletLight>>(Vec(LF_COL[0] + 17.f, by), module, LifeformModule::PAUSE_LIGHT));
        addChild(createLightCentered<SmallLight<agmm::CyanLight>>(Vec(LF_COL[2] + 17.f, by), module, LifeformModule::AUDIO_LIGHT));

        for (int i = 0; i < 6; ++i)
        {
            const int row = i / 3, col = i % 3;
            const float cy = LF_ROW_TOP[row] + LF_CTRL_DY;
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(LF_COL[col] + LF_KNOB_DX, cy), module, LF_KNOBS[i].param));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(LF_COL[col] + LF_JACK_DX, cy), module, LF_KNOBS[i].cv));
        }
        agmm::addKnobArcs(this, 2.f);
        const float jy = LF_IO_TOP + 24.f;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(LF_IO_X[0], jy), module, LifeformModule::AUDIO_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(LF_IO_X[1], jy), module, LifeformModule::PULSE_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(LF_IO_X[2], jy), module, LifeformModule::ENV_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(LF_IO_X[3], jy), module, LifeformModule::FIELD_OUTPUT));
    }

    void appendContextMenu(Menu *menu) override
    {
        agmm::addYouTubeLink(menu); // adds "Visit my youtube channel!"
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        // ... your other menu items ...
    }
};

// =====================================================================
//  Model
// =====================================================================
Model *modelLifeform = createModel<LifeformModule, LifeformWidget>("Lifeform");