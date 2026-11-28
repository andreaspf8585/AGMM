#include "YouTubeLoad.hpp"
#include "GitLoad.hpp"
// Evolved.cpp
// AGMM-styled "evolved" ferrofluid lifeform visualiser - 36 HP.
//
// ============================================================================
// OVERVIEW
// ============================================================================
// Evolved is the big brother of Lifeform. The glossy black 3D ferrofluid
// creature (lobed blob, Rosensweig spikes, orbiting droplets, perspective,
// depth sorting, normal-based gloss) now lives in a wide RECTANGULAR glass
// tank and has a small brain of its own:
//
//   * It reacts to sound (three bands, onset detector, silence detector) but
//     it does not need sound to live: it has ENERGY, gets hungry, hunts food
//     pellets, wanders, naps on the tank floor and remembers where it ate.
//   * It learns the TEMPO of the music (onset intervals -> beat period/phase)
//     and anticipates the next beat, even when a hit is missing.
//   * It senses the MOUSE: when the cursor hovers over the module the
//     lifeform notices, leans its spikes toward it, reaches out with its
//     droplets and swims after it. It has a TRUST value: gentle cursor
//     movement earns trust, fast aggressive movement makes it wary and it
//     keeps its distance (or gets startled and darts away).
//
// Moods: RESTING, WANDERING, HUNTING, CURIOUS (mouse), DANCING (audio),
//        STARTLED.
//
// Knobs (all with CV): FIELD, MOTION, VISC, SPIKES, SENS, GLOSS,
//        INTEL (how autonomous it is), CURIO (mouse attraction),
//        FOOD (food supply), SIZE (body size).
// Buttons: PAUSE, RESET, REACT (audio reactive on/off), MOUSE (mouse sense).
// Outputs: ENV, FIELD, X, Y (creature position, +-5 V), ENERGY (0..10 V),
//          EAT (trigger when it eats).
//
// File structure:
//   1. Helpers + layout constants (36 HP = 540x380 px).
//   2. Evolved: audio analysis + the creature's brain (control rate) +
//      outputs. The brain runs in the module so the outputs work even when
//      the display is not visible.
//   3. Anonymous-namespace UI: tiny 3D maths, EvolvedDisplay (3D renderer,
//      body physics, mouse sensing), EvPanelBackground.
//   4. EvolvedWidget and model registration.
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

namespace
{
    // Clamps x into [lo, hi] for any arithmetic type.
    template <typename T>
    inline T clampT(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }

    // Fast deterministic hash in [0,1) used for per-spike/per-droplet randoms.
    inline float hash1(float x)
    {
        const float s = std::sin(x * 127.1f + 311.7f) * 43758.5453f;
        return s - std::floor(s);
    }

    // Smoothstep between a and b (a may be larger than b for a falling edge).
    inline float smooth01(float a, float b, float x)
    {
        const float t = clampT((x - a) / (b - a), 0.f, 1.f);
        return t * t * (3.f - 2.f * t);
    }

    inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
} // namespace

// =====================================================================
//  Layout (36 HP = 540 x 380)
// =====================================================================
static const float EV_PANEL_W = RACK_GRID_WIDTH * 36; // 540
static const float EV_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Big display (bezel is drawn 4 px around it): tank on top, readout row below
static const float EV_DISP_X = 20.f;
static const float EV_DISP_Y = 28.f;
static const float EV_DISP_W = 500.f;
static const float EV_AREA_H = 178.f; // tank area
static const float EV_DISP_H = 190.f; // + 12 px readout row

// Rectangular glass tank inside the white perforated bezel
static const float EV_TANK_PAD = 6.f;
static const float EV_TANK_W = EV_DISP_W - 2.f * EV_TANK_PAD; // 488
static const float EV_TANK_H = EV_AREA_H - 2.f * EV_TANK_PAD; // 166

static const float EV_REF_R = 62.f; // creature scale unit (px)
static const float EV_CAM = 320.f;  // camera distance for perspective

// Region the creature's centre may roam in (px from tank centre)
static const float EV_BX = EV_TANK_W * 0.5f - 40.f;
static const float EV_BY = EV_TANK_H * 0.5f - 38.f;

static const int EV_FOODS = 5; // max food pellets
static const int EV_TICK = 64; // brain runs every N samples

// Controls band: 10 plates (knob + CV jack stacked)
static const float EV_SEC1_Y = 229.f;
static const float EV_CTRL_TOP = 236.f;
static const float EV_CTRL_H = 70.f;
static const float EV_CTRL_W = 46.f;
static const float EV_CTRL_STEP = 50.f;
// I/O + buttons band: 12 plates
static const float EV_SEC2_Y = 315.f;
static const float EV_IO_TOP = 322.f, EV_IO_H = 36.f, EV_IO_W = 38.f;
static const float EV_IO_STEP = EV_DISP_W / 12.f;

inline float evCtrlX(int i) { return EV_DISP_X + EV_CTRL_STEP * ((float)i + 0.5f); }
inline float evIoX(int i) { return EV_DISP_X + EV_IO_STEP * ((float)i + 0.5f); }

// =====================================================================
//  Module
// =====================================================================
struct Evolved : Module
{
    enum ParamId
    {
        FIELD_PARAM,
        MOTION_PARAM,
        VISC_PARAM,
        SPIKES_PARAM,
        SENS_PARAM,
        GLOSS_PARAM,
        INTEL_PARAM,
        CURIO_PARAM,
        FOOD_PARAM,
        SIZE_PARAM,
        PAUSE_PARAM,
        RESET_PARAM,
        AUDIO_PARAM,
        MOUSE_PARAM,
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
        INTEL_CV,
        CURIO_CV,
        FOOD_CV,
        SIZE_CV,
        INPUTS_LEN
    };
    enum OutputId
    {
        ENV_OUTPUT,
        FIELD_OUTPUT,
        X_OUTPUT,
        Y_OUTPUT,
        ENERGY_OUTPUT,
        EAT_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        PAUSE_LIGHT,
        AUDIO_LIGHT,
        MOUSE_LIGHT,
        LIGHTS_LEN
    };
    enum Mood
    {
        MOOD_REST,
        MOOD_WANDER,
        MOOD_SEEK,
        MOOD_CURIOUS,
        MOOD_DANCE,
        MOOD_STARTLE,
        MOOD_COUNT
    };

    // ---- audio analysis ----
    float env = 0.f;
    float pulseFlash = 0.f;
    float field = 0.f;
    float peakHold = 0.f;
    bool prevPulse = false;
    float lp1 = 0.f, lp2 = 0.f;
    float eLow = 0.f, eMid = 0.f, eHigh = 0.f;

    // cached filter coefficients (recomputed when the sample rate changes)
    float cachedSr = 0.f;
    float cAtk = 0.f, cRel = 0.f, cPeak = 0.f, cLP1 = 0.f, cLP2 = 0.f, cField = 0.f;
    float aLow = 0.f, rLow = 0.f, aMid = 0.f, rMid = 0.f, aHigh = 0.f, rHigh = 0.f;

    // ---- buttons / modes ----
    bool paused = false;
    bool audioReactive = true;
    bool mouseEnabled = true;
    bool prevPauseButton = false;
    bool prevAudioButton = false;
    bool prevMouseButton = false;
    bool prevResetButton = false;
    bool pendingKick = false;

    // ---- knobs (knob + CV), refreshed at brain rate ----
    float kField = 0.55f, kMotion = 0.45f, kVisc = 0.5f, kSpikes = 0.65f, kSens = 0.65f;
    float kGloss = 0.75f, kIntel = 0.7f, kCurio = 0.7f, kFood = 0.4f, kSize = 0.5f;

    // ---- the brain ----
    float t = 0.f;
    float posX = 0.f, posY = 0.f, velX = 0.f, velY = 0.f; // px from tank centre
    float energy = 0.7f;                                  // 0 starving .. 1 full
    float trust = 0.6f;                                   // 0 wary .. 1 trusting (of the mouse)
    float lifeAct = 0.f;                                  // own activity level (independent of sound)
    float audioAct = 0.f;                                 // sound-driven activity
    float punch = 0.f;
    float onsetRef = 0.f, onsetCool = 0.f;
    float phase = 0.f; // dance path phase
    Mood mood = MOOD_WANDER;
    float moodTimer = 0.f;
    float startleTimer = 0.f, startleCool = 0.f;
    float threatX = 0.f, threatY = 0.f;
    float tgtX = 0.f, tgtY = 0.f, retarget = 0.f;
    float eatFlash = 0.f;

    // tempo tracking
    float beatPeriod = 0.5f, beatPhase = 0.f, beatConf = 0.f, beatSince = 1.f, beatFlash = 0.f;

    // memory of where it found food
    float spotX[3] = {0.f, 0.f, 0.f}, spotY[3] = {0.f, 0.f, 0.f};
    int spotN = 0, spotHead = 0;

    // food pellets
    bool falive[EV_FOODS];
    float fx[EV_FOODS], fy[EV_FOODS], fty[EV_FOODS], fage[EV_FOODS];
    float foodTimer = 3.f, foodCool = 0.f;
    dsp::PulseGenerator eatPulse;

    int tick = 0;

    // ---- state published to the UI thread ----
    std::atomic<float> envSmoothed{0.f}, fieldSmoothed{0.f}, pulseSmoothed{0.f};
    std::atomic<float> lowSmoothed{0.f}, midSmoothed{0.f}, highSmoothed{0.f};
    std::atomic<float> posXA{0.f}, posYA{0.f}, velXA{0.f}, velYA{0.f};
    std::atomic<float> attrXA{0.f}, attrYA{0.f}, attrKA{0.f};
    std::atomic<float> energyA{0.7f}, trustA{0.6f}, lifeActA{0.f}, audioActA{0.f};
    std::atomic<float> eatFlashA{0.f}, beatFlashA{0.f}, beatConfA{0.f};
    std::atomic<float> viscA{0.5f}, spikesA{0.65f}, glossA{0.75f}, sensA{0.65f};
    std::atomic<float> motionA{0.45f}, sizeA{0.5f}, curioA{0.7f}, intelA{0.7f};
    std::atomic<int> moodA{MOOD_WANDER};
    std::atomic<int> hitSeq{0}, kickSeq{0}, eatSeq{0}, resetSeq{0};
    std::atomic<float> hitStr{0.f};
    std::atomic<float> foodXA[EV_FOODS], foodYA[EV_FOODS], foodShowA[EV_FOODS];

    // ---- mouse, written by the UI thread ----
    std::atomic<bool> mouseHover{false};
    std::atomic<float> mouseX{0.f}, mouseY{0.f}, mouseSpeed{0.f};

    // Constructor: configures all parameters, ports, lights and bypass.
    Evolved()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(FIELD_PARAM, 0.f, 1.f, 0.55f, "Magnetic field");
        configParam(MOTION_PARAM, 0.f, 1.f, 0.45f, "Swim speed / motion");
        configParam(VISC_PARAM, 0.f, 1.f, 0.50f, "Fluid viscosity");
        configParam(SPIKES_PARAM, 0.f, 1.f, 0.65f, "Spike amplitude");
        configParam(SENS_PARAM, 0.f, 1.f, 0.65f, "Audio sensitivity");
        configParam(GLOSS_PARAM, 0.f, 1.f, 0.75f, "Gloss");
        configParam(INTEL_PARAM, 0.f, 1.f, 0.70f, "Intelligence (autonomy)");
        configParam(CURIO_PARAM, 0.f, 1.f, 0.70f, "Curiosity (mouse attraction)");
        configParam(FOOD_PARAM, 0.f, 1.f, 0.40f, "Food supply");
        configParam(SIZE_PARAM, 0.f, 1.f, 0.50f, "Body size");

        configButton(PAUSE_PARAM, "Pause");
        configButton(RESET_PARAM, "Reset");
        configButton(AUDIO_PARAM, "Audio reactive");
        configButton(MOUSE_PARAM, "Mouse sense");
        configLight(PAUSE_LIGHT, "Pause active");
        configLight(AUDIO_LIGHT, "Audio active");
        configLight(MOUSE_LIGHT, "Mouse sense active");

        configInput(AUDIO_INPUT, "Audio");
        configInput(PULSE_INPUT, "Pulse / clock trigger");
        configInput(FIELD_CV, "Field CV");
        configInput(MOTION_CV, "Motion CV");
        configInput(VISC_CV, "Viscosity CV");
        configInput(SPIKES_CV, "Spikes CV");
        configInput(SENS_CV, "Sensitivity CV");
        configInput(GLOSS_CV, "Gloss CV");
        configInput(INTEL_CV, "Intelligence CV");
        configInput(CURIO_CV, "Curiosity CV");
        configInput(FOOD_CV, "Food CV");
        configInput(SIZE_CV, "Size CV");

        configOutput(ENV_OUTPUT, "Envelope");
        configOutput(FIELD_OUTPUT, "Field level");
        configOutput(X_OUTPUT, "Creature X position (+-5 V)");
        configOutput(Y_OUTPUT, "Creature Y position (+-5 V)");
        configOutput(ENERGY_OUTPUT, "Creature energy");
        configOutput(EAT_OUTPUT, "Eat trigger");

        configBypass(AUDIO_INPUT, ENV_OUTPUT);

        resetBrain();
    }

    // Recomputes every per-sample filter coefficient for a new sample rate.
    void updateCoefs(float sr)
    {
        cachedSr = sr;
        cAtk = 1.f - std::exp(-1.f / (0.004f * sr));
        cRel = 1.f - std::exp(-1.f / (0.180f * sr));
        cPeak = std::exp(-1.f / (0.010f * sr));
        cLP1 = std::min(1.f, 2.f * (float)M_PI * 180.f / sr);
        cLP2 = std::min(1.f, 2.f * (float)M_PI * 2200.f / sr);
        aLow = std::min(1.f, 1.f / (0.003f * sr));
        rLow = std::min(1.f, 1.f / (0.14f * sr));
        aMid = std::min(1.f, 1.f / (0.002f * sr));
        rMid = std::min(1.f, 1.f / (0.09f * sr));
        aHigh = std::min(1.f, 1.f / (0.001f * sr));
        rHigh = std::min(1.f, 1.f / (0.06f * sr));
        cField = 1.f - std::exp(-1.f / (0.025f * sr));
    }

    // Puts the creature back to a neutral, well-fed state in the tank centre
    // and clears food and memory. Also tells the UI to reset its spikes.
    void resetBrain()
    {
        t = 0.f;
        posX = posY = velX = velY = 0.f;
        energy = 0.7f;
        trust = 0.6f;
        lifeAct = audioAct = punch = 0.f;
        onsetRef = onsetCool = 0.f;
        phase = 0.f;
        mood = MOOD_WANDER;
        moodTimer = 0.f;
        startleTimer = startleCool = 0.f;
        tgtX = tgtY = 0.f;
        retarget = 0.f;
        eatFlash = 0.f;
        beatPeriod = 0.5f;
        beatPhase = beatConf = beatFlash = 0.f;
        beatSince = 1.f;
        spotN = spotHead = 0;
        foodTimer = 3.f;
        foodCool = 0.f;
        for (int i = 0; i < EV_FOODS; ++i)
        {
            falive[i] = false;
            fx[i] = fy[i] = fty[i] = fage[i] = 0.f;
            foodXA[i].store(0.f);
            foodYA[i].store(0.f);
            foodShowA[i].store(0.f);
        }
        publish();
        resetSeq.fetch_add(1);
    }

    // Rack reset hook.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        env = pulseFlash = field = peakHold = 0.f;
        lp1 = lp2 = eLow = eMid = eHigh = 0.f;
        prevPulse = false;
        paused = false;
        audioReactive = true;
        mouseEnabled = true;
        pendingKick = false;
        resetBrain();
    }

    // Saves toggles plus the creature's energy and trust (its "personality").
    json_t *dataToJson() override
    {
        json_t *r = json_object();
        json_object_set_new(r, "audioReactive", json_boolean(audioReactive));
        json_object_set_new(r, "mouseEnabled", json_boolean(mouseEnabled));
        json_object_set_new(r, "paused", json_boolean(paused));
        json_object_set_new(r, "energy", json_real(energy));
        json_object_set_new(r, "trust", json_real(trust));
        return r;
    }

    void dataFromJson(json_t *r) override
    {
        if (json_t *j = json_object_get(r, "audioReactive"))
            audioReactive = json_boolean_value(j);
        if (json_t *j = json_object_get(r, "mouseEnabled"))
            mouseEnabled = json_boolean_value(j);
        if (json_t *j = json_object_get(r, "paused"))
            paused = json_boolean_value(j);
        if (json_t *j = json_object_get(r, "energy"))
            energy = clampT((float)json_real_value(j), 0.f, 1.f);
        if (json_t *j = json_object_get(r, "trust"))
            trust = clampT((float)json_real_value(j), 0.f, 1.f);
    }

    // Knob + CV (0..10 V) clamped to 0..1.
    float cv(int p, int c)
    {
        return clampT(params[p].getValue() + inputs[c].getVoltage() / 10.f, 0.f, 1.f);
    }

    // Reads every knob + CV pair.
    void readKnobs()
    {
        kField = cv(FIELD_PARAM, FIELD_CV);
        kMotion = cv(MOTION_PARAM, MOTION_CV);
        kVisc = cv(VISC_PARAM, VISC_CV);
        kSpikes = cv(SPIKES_PARAM, SPIKES_CV);
        kSens = cv(SENS_PARAM, SENS_CV);
        kGloss = cv(GLOSS_PARAM, GLOSS_CV);
        kIntel = cv(INTEL_PARAM, INTEL_CV);
        kCurio = cv(CURIO_PARAM, CURIO_CV);
        kFood = cv(FOOD_PARAM, FOOD_CV);
        kSize = cv(SIZE_PARAM, SIZE_CV);
    }

    // ------------------------------------------------------------------
    //  Brain helpers
    // ------------------------------------------------------------------

    // Remembers a place where food was found (ring buffer of three spots).
    void rememberSpot(float x, float y)
    {
        spotX[spotHead] = x;
        spotY[spotHead] = y;
        spotHead = (spotHead + 1) % 3;
        spotN = std::min(spotN + 1, 3);
    }

    // Picks a new wandering target: sometimes a remembered feeding spot,
    // otherwise a random point at a decent distance from where it is now.
    void pickWanderTarget()
    {
        if (spotN > 0 && random::uniform() < 0.35f * kIntel)
        {
            const int k = (int)(random::uniform() * (float)spotN) % spotN;
            tgtX = spotX[k];
            tgtY = spotY[k];
            return;
        }
        for (int tries = 0; tries < 6; ++tries)
        {
            tgtX = (random::uniform() * 2.f - 1.f) * EV_BX;
            tgtY = (random::uniform() * 2.f - 1.f) * EV_BY * 0.9f;
            const float dx = tgtX - posX, dy = tgtY - posY;
            if (dx * dx + dy * dy > 70.f * 70.f)
                break;
        }
    }

    // Drops a food pellet from the top of the tank.
    void spawnFood()
    {
        int alive = 0, slot = -1;
        for (int i = 0; i < EV_FOODS; ++i)
        {
            if (falive[i])
                ++alive;
            else if (slot < 0)
                slot = i;
        }
        if (slot < 0 || alive >= 1 + (int)(kFood * 4.f + 0.5f))
            return;
        falive[slot] = true;
        fage[slot] = 0.f;
        fx[slot] = (random::uniform() * 2.f - 1.f) * EV_BX * 0.95f;
        fy[slot] = -EV_TANK_H * 0.5f + 6.f;
        fty[slot] = lerpf(-EV_BY * 0.5f, EV_BY, random::uniform());
    }

    // Eats pellet i: energy boost, trigger output, memory of the place.
    void eatFood(int i)
    {
        falive[i] = false;
        energy = std::min(1.f, energy + 0.30f);
        eatFlash = 1.f;
        eatSeq.fetch_add(1);
        eatPulse.trigger(0.015f);
        rememberSpot(fx[i], fy[i]);
        trust = std::min(1.f, trust + 0.03f);
    }

    // Startles the creature away from (fromX, fromY).
    void startle(float fromX, float fromY, float power)
    {
        startleTimer = 0.65f;
        startleCool = 2.5f;
        threatX = fromX;
        threatY = fromY;
        float dx = posX - fromX, dy = posY - fromY;
        float l = std::sqrt(dx * dx + dy * dy);
        if (l < 1.f)
        {
            const float a = random::uniform() * 2.f * (float)M_PI;
            dx = std::cos(a);
            dy = std::sin(a);
            l = 1.f;
        }
        dx /= l;
        dy /= l;
        threatX = posX - dx * 40.f;
        threatY = posY - dy * 40.f;
        velX += dx * 220.f * power * (0.4f + 0.6f * kIntel);
        velY += dy * 220.f * power * (0.4f + 0.6f * kIntel);
    }

    // A drum hit / pulse arrives: punch, hop, tempo tracking, startle, food.
    void registerHit(float str, bool fromPulse)
    {
        hitStr.store(str);
        hitSeq.fetch_add(1);
        if (fromPulse)
            kickSeq.fetch_add(1);
        punch = std::min(punch + str * 0.9f, 1.2f);

        // hop in time with the music
        velY -= 55.f * str * (0.4f + 0.6f * kIntel);
        velX += (random::uniform() - 0.5f) * 40.f * str;

        // ---- tempo tracking: onset interval -> beat period + phase lock ----
        if (str > 0.25f)
        {
            const float interval = beatSince;
            bool onGrid = false;
            if (interval > 0.2f && interval < 1.6f)
            {
                const float ratio = interval / beatPeriod;
                if (std::fabs(ratio - 1.f) < 0.2f)
                {
                    onGrid = true;
                    beatPeriod += (interval - beatPeriod) * 0.25f;
                    beatConf = std::min(1.f, beatConf + 0.18f);
                }
                else if (std::fabs(ratio - 0.5f) < 0.1f || std::fabs(ratio - 2.f) < 0.2f)
                {
                    // subdivision / half-time: keep the grid
                }
                else
                {
                    beatConf -= 0.25f;
                    if (beatConf < 0.1f)
                    {
                        beatPeriod = interval;
                        beatConf = 0.1f;
                    }
                }
            }
            beatPeriod = clampT(beatPeriod, 0.2f, 1.6f);
            if (beatConf < 0.3f)
            {
                beatPhase = 0.f;
                beatSince = 0.f;
            }
            else if (onGrid)
            {
                float err = beatPhase;
                if (err > 0.5f)
                    err -= 1.f;
                beatPhase -= 0.5f * err;
                if (beatPhase < 0.f)
                    beatPhase += 1.f;
                beatSince = 0.f;
            }
        }

        // a very loud hit wakes a sleeping creature with a fright
        if (str > 0.75f && mood == MOOD_REST && startleCool <= 0.f && random::uniform() < 0.6f)
            startle(posX + (random::uniform() - 0.5f) * 30.f, posY + (random::uniform() - 0.5f) * 30.f, 0.8f);

        // the music feeds it: strong hits sometimes drop a pellet
        if (str > 0.6f && kFood > 0.05f && foodCool <= 0.f && random::uniform() < 0.5f * kFood)
        {
            spawnFood();
            foodCool = 1.5f;
        }
    }

    // ------------------------------------------------------------------
    //  The brain: runs at control rate (every EV_TICK samples)
    // ------------------------------------------------------------------
    void brainStep(float dt)
    {
        t += dt;

        // ---- sound activity ----
        const float level = env * (0.4f + kSens) + punch * 0.2f;
        const float actT = audioReactive ? std::max(smooth01(0.008f, 0.09f, level), pulseFlash) : 0.f;
        audioAct += (actT - audioAct) * (1.f - std::exp(-(actT > audioAct ? 30.f : 1.0f) * dt));

        // ---- onset detector ----
        onsetCool -= dt;
        onsetRef += (env - onsetRef) * (1.f - std::exp(-dt / 0.12f));
        if (audioReactive && onsetCool <= 0.f && env > 0.03f && (env - onsetRef) > 0.05f + 0.25f * onsetRef)
        {
            onsetCool = 0.07f;
            onsetRef = env;
            registerHit(clampT(peakHold * (0.6f + kSens), 0.f, 1.f), false);
        }
        if (pendingKick)
        {
            pendingKick = false;
            registerHit(1.f, true);
        }

        // ---- decays & timers ----
        punch *= std::exp(-dt * 6.f);
        eatFlash *= std::exp(-dt * 4.f);
        beatFlash *= std::exp(-dt * 5.f);
        startleTimer -= dt;
        startleCool -= dt;
        foodCool -= dt;
        moodTimer += dt;

        // ---- tempo clock: predicts the next beat even when a hit is missing ----
        beatSince += dt;
        beatConf -= dt * (audioAct > 0.1f ? 0.05f : 0.4f);
        beatConf = clampT(beatConf, 0.f, 1.f);
        beatPhase += dt / beatPeriod;
        if (beatPhase >= 1.f)
        {
            beatPhase -= 1.f;
            if (audioReactive && beatConf > 0.5f && beatSince > 0.6f * beatPeriod)
            {
                beatFlash = std::max(beatFlash, 0.7f * beatConf);
                velY -= 25.f * beatConf * (0.3f + 0.7f * kIntel); // anticipation hop
            }
        }

        // ---- food pellets ----
        foodTimer -= dt;
        if (kFood > 0.02f && foodTimer <= 0.f)
        {
            spawnFood();
            foodTimer = (4.f + 22.f * (1.f - kFood)) * (0.6f + 0.8f * random::uniform());
        }
        int nearest = -1;
        float nearestD = 1e9f;
        for (int i = 0; i < EV_FOODS; ++i)
        {
            if (!falive[i])
                continue;
            fage[i] += dt;
            if (fy[i] < fty[i])
                fy[i] = std::min(fty[i], fy[i] + 28.f * dt);
            if (fage[i] > 30.f)
            {
                falive[i] = false;
                continue;
            }
            const float dx = fx[i] - posX, dy = fy[i] - posY;
            const float d = dx * dx + dy * dy;
            if (d < nearestD)
            {
                nearestD = d;
                nearest = i;
            }
        }
        const bool hasFood = nearest >= 0;

        // ---- sensing the mouse ----
        const bool mouseOn = mouseEnabled && mouseHover.load() && kCurio > 0.03f;
        const float mX = mouseX.load(), mY = mouseY.load(), mSpd = mouseSpeed.load();
        const float mdx = mX - posX, mdy = mY - posY;
        const float mDist = std::sqrt(mdx * mdx + mdy * mdy);

        // trust: gentle cursor earns it, aggressive cursor near the creature loses it
        if (mouseOn)
        {
            if (mSpd < 220.f)
                trust += 0.05f * dt;
            else if (mSpd > 520.f && mDist < 140.f)
                trust -= 0.30f * dt;
            if (mSpd > 800.f && mDist < 80.f && trust < 0.55f && startleTimer <= 0.f && startleCool <= 0.f)
                startle(mX, mY, 1.f);
        }
        else
        {
            trust += (0.6f - trust) * dt * 0.01f;
        }
        trust = clampT(trust, 0.f, 1.f);
        const float effC = kCurio * (0.25f + 0.75f * trust);

        // ---- metabolism: moving costs, sound feeds, resting recovers ----
        const float speedN = std::min(1.f, std::sqrt(velX * velX + velY * velY) / 120.f);
        float dE = -0.012f * (0.4f + speedN) * (0.4f + kIntel);
        dE += audioAct * 0.030f * (0.3f + kSens);
        if (mood == MOOD_REST)
            dE += 0.030f;
        energy = clampT(energy + dE * dt, 0.f, 1.f);
        const float hunger = 1.f - energy;

        // ---- decide mood ----
        Mood want;
        if (startleTimer > 0.f)
            want = MOOD_STARTLE;
        else if (mouseOn)
            want = MOOD_CURIOUS;
        else if (audioAct > (mood == MOOD_DANCE ? 0.2f : 0.35f))
            want = MOOD_DANCE;
        else if (hasFood && hunger > 0.2f)
            want = MOOD_SEEK;
        else if (energy < 0.12f || (mood == MOOD_REST && energy < 0.55f))
            want = MOOD_REST;
        else
            want = MOOD_WANDER;

        if (want != mood && (moodTimer > 0.8f || want == MOOD_STARTLE || mood == MOOD_STARTLE))
        {
            mood = want;
            moodTimer = 0.f;
            retarget = 0.f;
        }

        // ---- choose target & style of movement for the current mood ----
        const float selfScale = 0.35f + 0.65f * kIntel;
        float tx = posX, ty = posY, w = 0.f;
        float maxSp = 30.f, arrive = 2.f;
        float attrX = posX, attrY = posY, attrK = 0.f;
        float moodAct = 0.f;

        switch (mood)
        {
        case MOOD_REST:
            tx = posX;
            ty = EV_BY * 0.85f; // sinks to the tank floor
            w = kIntel;
            maxSp = 25.f;
            arrive = 1.5f;
            moodAct = 0.f;
            break;

        case MOOD_WANDER:
        {
            retarget -= dt;
            const float rx = tgtX - posX, ry = tgtY - posY;
            if (retarget <= 0.f || rx * rx + ry * ry < 100.f)
            {
                pickWanderTarget();
                retarget = 3.f + 5.f * random::uniform() * (1.3f - kMotion);
            }
            tx = tgtX;
            ty = tgtY;
            w = kIntel;
            maxSp = (20.f + 80.f * kMotion) * (0.5f + 0.5f * energy);
            attrX = tx;
            attrY = ty;
            attrK = 0.3f;
            moodAct = (0.30f + 0.30f * energy) * selfScale;
            break;
        }

        case MOOD_SEEK:
            if (hasFood)
            {
                tx = fx[nearest];
                ty = fy[nearest];
            }
            w = kIntel;
            maxSp = (45.f + 110.f * kMotion) * (0.8f + 0.5f * hunger);
            arrive = 2.5f;
            attrX = tx;
            attrY = ty;
            attrK = 0.7f;
            moodAct = 0.62f * selfScale;
            break;

        case MOOD_CURIOUS:
        {
            const bool wary = trust < 0.35f;
            if (wary)
            {
                // keeps its distance and circles the cursor
                float ux = posX - mX, uy = posY - mY;
                float ul = std::sqrt(ux * ux + uy * uy);
                if (ul < 1.f)
                {
                    ux = 1.f;
                    uy = 0.f;
                    ul = 1.f;
                }
                ux /= ul;
                uy /= ul;
                tx = mX + ux * 85.f - uy * 20.f * std::sin(t * 1.1f);
                ty = mY + uy * 85.f + ux * 20.f * std::sin(t * 1.1f);
                attrK = 0.35f;
            }
            else
            {
                // follows the cursor and "sniffs" around it once it arrives
                const float nearK = smooth01(40.f, 8.f, mDist);
                tx = mX + std::cos(t * 2.2f) * 10.f * nearK;
                ty = mY + std::sin(t * 2.2f) * 10.f * nearK;
                attrK = 0.5f + 0.5f * effC;
            }
            w = std::min(1.f, effC * 1.6f);
            maxSp = (60.f + 140.f * kMotion) * (0.6f + 0.4f * energy);
            arrive = 3.f;
            attrX = mX;
            attrY = mY;
            moodAct = wary ? 0.45f : (0.55f + 0.35f * effC);
            break;
        }

        case MOOD_DANCE:
            phase += kMotion * dt * 1.35f * (0.4f + audioAct);
            if (phase > 2.f * (float)M_PI)
                phase -= 2.f * (float)M_PI;
            tx = std::cos(phase) * EV_BX * 0.55f;
            ty = std::sin(phase * 1.31f) * EV_BY * 0.75f;
            w = 0.5f + 0.5f * audioAct;
            maxSp = (40.f + 120.f * kMotion) * (0.5f + audioAct);
            arrive = 2.5f;
            attrX = tx;
            attrY = ty;
            attrK = 0.6f * audioAct;
            moodAct = 1.f;
            break;

        case MOOD_STARTLE:
        default:
        {
            float ux = posX - threatX, uy = posY - threatY;
            const float ul = std::sqrt(ux * ux + uy * uy) + 1e-3f;
            tx = posX + ux / ul * 100.f;
            ty = posY + uy / ul * 100.f;
            w = 1.f;
            maxSp = 240.f;
            arrive = 4.f;
            moodAct = 1.f;
            break;
        }
        }

        lifeAct += (moodAct - lifeAct) * (1.f - std::exp(-dt * (moodAct > lifeAct ? 5.f : 0.6f)));

        // ---- steering (target blends with the tank centre by the weight w) ----
        const float gx = clampT(tx * w, -EV_BX, EV_BX);
        const float gy = clampT(ty * w, -EV_BY, EV_BY);
        const float dx = gx - posX, dy = gy - posY;
        const float dist = std::sqrt(dx * dx + dy * dy);
        const float sp = std::min(maxSp, dist * arrive);
        float dvx = 0.f, dvy = 0.f;
        if (dist > 0.5f)
        {
            dvx = dx / dist * sp;
            dvy = dy / dist * sp;
        }
        const float drift = 3.f * (0.3f + lifeAct); // never perfectly still
        dvx += std::sin(t * 0.61f + 1.3f) * drift;
        dvy += std::sin(t * 0.83f) * drift * 0.6f;

        const float accK = 1.f - std::exp(-dt * (5.f - 3.f * kVisc));
        velX += (dvx - velX) * accK;
        velY += (dvy - velY) * accK;
        posX += velX * dt;
        posY += velY * dt;
        if (posX > EV_BX)
        {
            posX = EV_BX;
            if (velX > 0.f)
                velX *= -0.3f;
        }
        if (posX < -EV_BX)
        {
            posX = -EV_BX;
            if (velX < 0.f)
                velX *= -0.3f;
        }
        if (posY > EV_BY)
        {
            posY = EV_BY;
            if (velY > 0.f)
                velY *= -0.3f;
        }
        if (posY < -EV_BY)
        {
            posY = -EV_BY;
            if (velY < 0.f)
                velY *= -0.3f;
        }

        // ---- eating ----
        for (int i = 0; i < EV_FOODS; ++i)
        {
            if (!falive[i])
                continue;
            const float ex = fx[i] - posX, ey = fy[i] - posY;
            if (ex * ex + ey * ey < 20.f * 20.f)
                eatFood(i);
        }

        attrXA.store(attrX);
        attrYA.store(attrY);
        attrKA.store(attrK);
    }

    // Publishes the brain and the analysis to the UI thread.
    void publish()
    {
        envSmoothed.store(env);
        fieldSmoothed.store(field);
        pulseSmoothed.store(pulseFlash);
        lowSmoothed.store(clampT(eLow, 0.f, 1.5f));
        midSmoothed.store(clampT(eMid, 0.f, 1.5f));
        highSmoothed.store(clampT(eHigh, 0.f, 1.5f));
        posXA.store(posX);
        posYA.store(posY);
        velXA.store(velX);
        velYA.store(velY);
        energyA.store(energy);
        trustA.store(trust);
        lifeActA.store(lifeAct);
        audioActA.store(audioAct);
        eatFlashA.store(eatFlash);
        beatFlashA.store(beatFlash);
        beatConfA.store(beatConf);
        viscA.store(kVisc);
        spikesA.store(kSpikes);
        glossA.store(kGloss);
        sensA.store(kSens);
        motionA.store(kMotion);
        sizeA.store(kSize);
        curioA.store(kCurio);
        intelA.store(kIntel);
        moodA.store((int)mood);
        for (int i = 0; i < EV_FOODS; ++i)
        {
            float show = 0.f;
            if (falive[i])
                show = std::min(1.f, fage[i] / 0.5f) * clampT((30.f - fage[i]) / 3.f, 0.f, 1.f);
            foodXA[i].store(fx[i]);
            foodYA[i].store(fy[i]);
            foodShowA[i].store(show);
        }
    }

    // Main audio callback: buttons, envelope / three bands / pulse, field,
    // brain tick, outputs.
    void process(const ProcessArgs &args) override
    {
        const float sr = args.sampleRate;
        if (sr != cachedSr)
            updateCoefs(sr);

        // ---- buttons ----
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

        const bool mousePressed = params[MOUSE_PARAM].getValue() > 0.5f;
        if (mousePressed && !prevMouseButton)
            mouseEnabled = !mouseEnabled;
        prevMouseButton = mousePressed;
        lights[MOUSE_LIGHT].setBrightness(mouseEnabled ? 1.f : 0.f);

        const bool resetPressed = params[RESET_PARAM].getValue() > 0.5f;
        if (resetPressed && !prevResetButton)
            resetBrain();
        prevResetButton = resetPressed;

        // ---- audio analysis ----
        const float in = inputs[AUDIO_INPUT].getVoltage() / 5.f;
        const float rect = std::fabs(in);
        env += (rect - env) * (rect > env ? cAtk : cRel);
        peakHold = std::max(rect, peakHold * cPeak);

        // three bands: low < 180 Hz < mid < 2.2 kHz < high
        lp1 += (in - lp1) * cLP1;
        lp2 += (in - lp2) * cLP2;
        const float bandLow = std::fabs(lp1) * 1.5f;
        const float bandMid = std::fabs(lp2 - lp1) * 1.6f;
        const float bandHigh = std::fabs(in - lp2) * 2.0f;
        eLow += (bandLow - eLow) * (bandLow > eLow ? aLow : rLow);
        eMid += (bandMid - eMid) * (bandMid > eMid ? aMid : rMid);
        eHigh += (bandHigh - eHigh) * (bandHigh > eHigh ? aHigh : rHigh);

        // pulse input
        const bool pulse = inputs[PULSE_INPUT].getVoltage() >= 1.f;
        if (pulse && !prevPulse)
        {
            pulseFlash = 1.f;
            pendingKick = true;
        }
        prevPulse = pulse;
        pulseFlash -= args.sampleTime * 3.5f;
        if (pulseFlash < 0.f)
            pulseFlash = 0.f;

        // ---- field (knob + audio + pulse + predicted beat + eating) ----
        if (!paused)
        {
            const float audioBoost = audioReactive ? env * kSens * 1.1f : 0.f;
            const float fieldTarget = clampT(kField + audioBoost + pulseFlash * 0.75f + beatFlash * 0.35f * kIntel + eatFlash * 0.25f, 0.f, 1.f);
            field += (fieldTarget - field) * cField;
        }

        // ---- brain tick ----
        if (++tick >= EV_TICK)
        {
            tick = 0;
            readKnobs();
            if (!paused)
                brainStep(EV_TICK * args.sampleTime);
            publish();
        }

        outputs[ENV_OUTPUT].setVoltage(clampT(env * 10.f, 0.f, 10.f));
        outputs[FIELD_OUTPUT].setVoltage(field * 10.f);
        outputs[X_OUTPUT].setVoltage(clampT(posX / EV_BX, -1.f, 1.f) * 5.f);
        outputs[Y_OUTPUT].setVoltage(clampT(posY / EV_BY, -1.f, 1.f) * 5.f);
        outputs[ENERGY_OUTPUT].setVoltage(energy * 10.f);
        outputs[EAT_OUTPUT].setVoltage(eatPulse.process(args.sampleTime) ? 10.f : 0.f);
    }
};

namespace
{

    // ---- tiny 3D maths ----
    struct V3
    {
        float x, y, z;
    };
    inline V3 v3(float x, float y, float z)
    {
        V3 r;
        r.x = x;
        r.y = y;
        r.z = z;
        return r;
    }
    inline V3 operator+(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
    inline V3 operator-(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
    inline V3 operator*(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
    inline float dot3(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline V3 cross3(V3 a, V3 b)
    {
        return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }
    inline float len3(V3 a) { return std::sqrt(dot3(a, a)); }
    inline V3 norm3(V3 a) { return a * (1.f / (len3(a) + 1e-6f)); }

    struct M3
    {
        float m[9];
    }; // row-major
    inline V3 m3mul(const M3 &M, V3 v)
    {
        return v3(M.m[0] * v.x + M.m[1] * v.y + M.m[2] * v.z,
                  M.m[3] * v.x + M.m[4] * v.y + M.m[5] * v.z,
                  M.m[6] * v.x + M.m[7] * v.y + M.m[8] * v.z);
    }

    // Knob table entry: display name, parameter ID and CV input ID.
    struct EvKnobDef
    {
        const char *name;
        int param;
        int cv;
    };
    const EvKnobDef EV_KNOBS[10] = {
        {"FIELD", Evolved::FIELD_PARAM, Evolved::FIELD_CV},
        {"MOTION", Evolved::MOTION_PARAM, Evolved::MOTION_CV},
        {"VISC", Evolved::VISC_PARAM, Evolved::VISC_CV},
        {"SPIKES", Evolved::SPIKES_PARAM, Evolved::SPIKES_CV},
        {"SENS", Evolved::SENS_PARAM, Evolved::SENS_CV},
        {"GLOSS", Evolved::GLOSS_PARAM, Evolved::GLOSS_CV},
        {"INTEL", Evolved::INTEL_PARAM, Evolved::INTEL_CV},
        {"CURIO", Evolved::CURIO_PARAM, Evolved::CURIO_CV},
        {"FOOD", Evolved::FOOD_PARAM, Evolved::FOOD_CV},
        {"SIZE", Evolved::SIZE_PARAM, Evolved::SIZE_CV},
    };

    const char *EV_IO_LABELS[12] = {"AUDIO", "PULSE", "ENV", "FIELD", "X", "Y", "ENERGY", "EAT", "PAUSE", "RESET", "REACT", "MOUSE"};
    const char *EV_MOOD_NAMES[6] = {"RESTING", "WANDERING", "HUNTING", "CURIOUS", "DANCING", "STARTLED"};

    // =====================================================================
    //  Ferrofluid display - 3D tumbling creature in a rectangular tank
    // =====================================================================
    struct EvolvedDisplay : TransparentWidget
    {
        enum
        {
            N_SPIKES = 160,
            N_DROPS = 16,
            LAT = 48, // surface resolution (was 16)
            LON = 96, // (was 32)
            GN = LAT * LON,
            NB = 256 // silhouette bins (was 96)
        };

        struct Spike
        {
            V3 d;             // local unit direction on the body
            float rnd, phase; // per-spike randoms
            float h, v;       // height and velocity
            float bx, by, tx, ty, hw, spec, lit;
            bool front, vis;
        };

        struct Drop
        {
            V3 e1, e2;
            float ang, dist, rest, r;
            float dv, w, phase;
            float x, y, sc, z, vis;
        };

        Spike spikes[N_SPIKES];
        Drop drops[N_DROPS];

        V3 ldir[GN];
        V3 wpos[GN];
        V3 wnorm[GN];
        float psx[GN], psy[GN], psc[GN];
        float lw[GN]; // sin(latitude): evens out highlight density near the poles
        float sil[NB];

        M3 Rm;
        V3 om;
        V3 Lv, Hv, H2v, magD, velD;

        Evolved *module = nullptr;
        float t = 0.f;
        double lastTime = 0.0;

        int lastHitSeq = 0, lastKickSeq = 0, lastEatSeq = 0, lastResetSeq = 0;

        // shared geometry / state for the draw code
        float bodyBase = 16.f;
        float wobAmt = 0.08f;
        float stretch = 0.f;
        float vStr = 0.f;
        float magA = 0.f, magLen = 0.f;
        float offX = 0.f, offY = 0.f;                       // total offset of the body centre from tank centre
        float bx = 0.f, by = 0.f;                           // brain position
        float bvx = 0.f, bvy = 0.f;                         // brain velocity
        float slX = 0.f, slY = 0.f, slVX = 0.f, slVY = 0.f; // slosh (soft-body lag)

        float act = 0.f;
        float calmK = 1.f;
        float wt = 0.f;
        float spinVel = 0.f;
        float ripAmt = 0.f;
        float ell = 0.12f;
        float punch = 0.f;
        float eatF = 0.f;
        float glowK = 0.f; // 0 = room bright, 1 = pitch dark: purple bioluminescence amount
        int mood = Evolved::MOOD_WANDER;
        float trust = 0.6f;
        float energy = 0.7f;

        // mouse sensing
        float mouseSense = 0.f;
        float mxL = 0.f, myL = 0.f; // mouse relative to tank centre
        float pmx = 0.f, pmy = 0.f;
        float mouseSpd = 0.f;
        bool mouseHad = false;
        bool mouseInTank = false;

        explicit EvolvedDisplay(Evolved *m) : module(m)
        {
            box.pos = Vec(EV_DISP_X, EV_DISP_Y);
            box.size = Vec(EV_DISP_W, EV_DISP_H);

            std::memset(ldir, 0, sizeof(ldir));
            std::memset(wpos, 0, sizeof(wpos));
            std::memset(wnorm, 0, sizeof(wnorm));
            std::memset(psx, 0, sizeof(psx));
            std::memset(psy, 0, sizeof(psy));
            std::memset(psc, 0, sizeof(psc));
            std::memset(sil, 0, sizeof(sil));

            const float I[9] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
            std::memcpy(Rm.m, I, sizeof(I));
            om = v3(0.3f, 0.7f, 0.1f);
            rotateBy(v3(0.5f, 0.8f, 0.2f), 1.f);

            Lv = norm3(v3(-0.5f, -0.6f, 0.62f));
            Hv = norm3(Lv + v3(0.f, 0.f, 1.f));
            const V3 L2 = norm3(v3(0.6f, 0.5f, 0.55f));
            H2v = norm3(L2 + v3(0.f, 0.f, 1.f));
            magD = norm3(v3(0.8f, 0.f, 0.6f));
            velD = v3(1.f, 0.f, 0.f);

            for (int i = 0; i < LAT; ++i)
            {
                const float th = (float)M_PI * ((float)i + 0.5f) / (float)LAT;
                for (int j = 0; j < LON; ++j)
                {
                    const float ph = 2.f * (float)M_PI * (float)j / (float)LON;
                    ldir[i * LON + j] = v3(std::sin(th) * std::cos(ph), std::cos(th), std::sin(th) * std::sin(ph));
                    lw[i * LON + j] = std::sin(th);
                }
            }

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

            for (int i = 0; i < N_DROPS; ++i)
            {
                Drop &d = drops[i];
                const V3 n = norm3(v3(hash1(i * 1.9f) - 0.5f, hash1(i * 2.9f + 1.f) - 0.5f, hash1(i * 3.9f + 2.f) - 0.5f));
                const V3 helper = (std::fabs(n.y) < 0.9f) ? v3(0.f, 1.f, 0.f) : v3(1.f, 0.f, 0.f);
                d.e1 = norm3(cross3(n, helper));
                d.e2 = cross3(n, d.e1);
                d.ang = hash1(i * 9.1f) * 2.f * (float)M_PI;
                d.rest = EV_REF_R * (0.62f + 0.26f * hash1(i * 2.2f));
                d.dist = d.rest;
                d.r = (i < 3) ? (2.6f + 2.0f * hash1(i * 4.4f))
                              : (0.8f + 1.5f * hash1(i * 6.6f));
                d.dv = 0.f;
                d.w = (hash1(i * 7.7f) - 0.5f) * 0.9f;
                d.phase = hash1(i * 8.8f) * 6.2832f;
                d.x = d.y = d.z = 0.f;
                d.sc = 1.f;
                d.vis = 1.f;
            }

            stepPhysics(1.f / 60.f);
        }

        // Resets spike heights, droplet distances and slosh (RESET button).
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
            slX = slY = slVX = slVY = 0.f;
        }

        // ---- 3D helpers ----
        static void project(const V3 &P, float &x, float &y, float &s)
        {
            s = EV_CAM / (EV_CAM - P.z);
            x = P.x * s;
            y = P.y * s;
        }

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

        // Radius of the body surface at local direction d (world direction wd):
        // wobble, ripple, ellipse, stretch toward the attractor and a
        // squash-and-stretch along the swimming direction.
        float surfaceR(const V3 &d, const V3 &wd) const
        {
            const float n = std::sin(2.1f * d.x + 1.7f * d.y + wt * 0.90f) * 0.50f + std::sin(3.2f * d.y - 2.4f * d.z - wt * 1.30f + 1.7f) * 0.30f + std::sin(1.6f * d.z + 2.2f * d.x + wt * 0.55f) * 0.40f + std::sin(5.0f * d.x + 4.1f * d.y + 3.7f * d.z + wt * 2.10f) * 0.12f;
            const float al = std::max(0.f, dot3(wd, magD));
            const float av = dot3(wd, velD);
            const float r = bodyBase * (1.f + wobAmt * n + ripAmt * std::sin(7.f * d.y - wt * 7.f + 2.f * d.x) + ell * (1.5f * d.x * d.x - 0.5f) + stretch * al * al * al + vStr * (av * av - 0.3f));
            return std::min(r, EV_TANK_H * 0.30f);
        }

        // Reads the module (or runs a demo when no module), senses the mouse
        // events, and advances body physics, spikes and droplets.
        void stepPhysics(float dt)
        {
            t += dt;
            const float R = EV_REF_R;
            Evolved *m = module;

            // ---- read state ----
            float field = 0.58f, env = 0.1f, pulse = 0.f, low = 0.2f, mid = 0.1f, high = 0.05f;
            float visc = 0.5f, spikeAmt = 0.65f, sens = 0.65f, motion = 0.45f, sizeK = 0.5f;
            float life = 0.6f, audioAct = 0.f;
            float attrX = 0.f, attrY = 0.f, attrK = 0.3f;
            float beatF = 0.f, beatC = 0.f;
            bool audio = true;
            eatF = 0.f;
            mood = Evolved::MOOD_WANDER;
            trust = 0.6f;
            energy = 0.7f;

            bool hit = false, kick = false, ate = false;
            float onset = 0.f;

            if (m)
            {
                field = m->fieldSmoothed.load();
                env = m->envSmoothed.load();
                pulse = m->pulseSmoothed.load();
                low = m->lowSmoothed.load();
                mid = m->midSmoothed.load();
                high = m->highSmoothed.load();
                visc = m->viscA.load();
                spikeAmt = m->spikesA.load();
                sens = m->sensA.load();
                motion = m->motionA.load();
                sizeK = m->sizeA.load();
                audio = m->audioReactive;
                bx = m->posXA.load();
                by = m->posYA.load();
                bvx = m->velXA.load();
                bvy = m->velYA.load();
                life = m->lifeActA.load();
                audioAct = m->audioActA.load();
                attrX = m->attrXA.load();
                attrY = m->attrYA.load();
                attrK = m->attrKA.load();
                eatF = m->eatFlashA.load();
                beatF = m->beatFlashA.load();
                beatC = m->beatConfA.load();
                mood = m->moodA.load();
                trust = m->trustA.load();
                energy = m->energyA.load();

                const int hs = m->hitSeq.load();
                if (hs != lastHitSeq)
                {
                    hit = true;
                    onset = m->hitStr.load();
                    lastHitSeq = hs;
                }
                const int ks = m->kickSeq.load();
                if (ks != lastKickSeq)
                {
                    kick = true;
                    lastKickSeq = ks;
                }
                const int es = m->eatSeq.load();
                if (es != lastEatSeq)
                {
                    ate = true;
                    lastEatSeq = es;
                }
            }
            else
            {
                // library preview: a self-driven demo creature
                bx = std::cos(t * 0.30f) * 150.f;
                by = std::sin(t * 0.45f) * 25.f;
                bvx = -std::sin(t * 0.30f) * 45.f;
                bvy = std::cos(t * 0.45f) * 11.f;
                attrX = bx + bvx;
                attrY = by + bvy;
            }
            if (ate)
            {
                hit = true;
                onset = std::max(onset, 0.55f);
            }

            const float sizeScale = 0.72f + 0.55f * sizeK;
            const float audioEnv = audio ? env * sens : 0.f;
            const float lowA = audio ? low * sens : 0.f;
            const float midA = audio ? mid * sens : 0.f;
            const float highA = audio ? high * sens : 0.f;
            const float criticalField = 0.28f;

            // ---- punch (transient energy) ----
            punch *= std::exp(-dt * 6.f);
            if (hit)
                punch = std::min(punch + onset * 1.3f, 1.2f);

            // ---- activity: sound OR the creature's own life keeps it blooming ----
            const float actT = std::max(audioAct, life);
            act += (actT - act) * (1.f - std::exp(-(actT > act ? 14.f : 2.f) * dt));
            calmK = smooth01(0.15f, 0.85f, 1.f - act);

            wt += dt * (0.7f + 0.8f * act + 1.5f * audioEnv + 1.0f * midA);

            // ---- 3D tumble (rolls along with the swimming direction) ----
            const float spinMag = (0.18f + 0.8f * motion) * (0.35f + 0.65f * act) + 1.2f * lowA;
            const V3 axis = norm3(v3(0.7f * std::sin(t * 0.29f + 0.5f),
                                     0.8f + 0.5f * std::sin(t * 0.41f + 2.f),
                                     0.6f * std::cos(t * 0.23f)));
            const V3 omT = axis * spinMag + v3(-bvy * 0.012f, bvx * 0.012f, 0.f);
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

            // ---- attractor (mouse / food / dance point): spikes lean toward it ----
            {
                const float adx = attrX - bx, ady = attrY - by;
                const float adist = std::sqrt(adx * adx + ady * ady);
                const float ak = attrK * smooth01(6.f, 45.f, adist);
                magLen = clampT(ak * 1.2f, 0.f, 1.2f);
                if (adist > 1.f)
                    magA = std::atan2(ady, adx);
                magD = norm3(v3(std::cos(magA) * 0.8f, std::sin(magA) * 0.8f, 0.6f));
            }

            // ---- body geometry ----
            bodyBase = R * sizeScale * (0.26f + 0.09f * field * (0.4f + 0.6f * act) + 0.10f * lowA + 0.04f * audioEnv + 0.02f * pulse + 0.05f * punch + 0.025f * beatF * beatC + 0.04f * eatF) * (1.f + 0.20f * calmK + 0.03f * std::sin(wt * 1.7f));
            wobAmt = 0.08f + 0.06f * field * act + 0.05f * audioEnv + 0.08f * midA;
            ripAmt = 0.12f * midA + 0.10f * pulse + 0.06f * punch;
            ell = 0.12f + 0.08f * act;
            stretch = clampT(0.20f * magLen * (0.4f + field) * (0.35f + 0.65f * act), 0.f, 0.30f);
            {
                const float speed = std::sqrt(bvx * bvx + bvy * bvy);
                vStr = clampT(speed / 180.f, 0.f, 1.f) * 0.20f;
                velD = norm3(v3(bvx, bvy, 0.f));
            }

            // ---- soft-body slosh: the body trails behind its motion and wobbles ----
            const float cAmt = 0.45f + 0.55f * act;
            const float stx = -bvx * 0.045f * cAmt + std::cos(wt * 0.7f) * R * 0.03f * cAmt;
            const float sty = -bvy * 0.045f * cAmt + std::sin(wt * 0.9f) * R * 0.03f * cAmt;
            if (hit)
            {
                slVX += (hash1(t * 5.1f) - 0.5f) * 30.f * (0.5f + onset);
                slVY += (hash1(t * 6.3f) - 0.5f) * 30.f * (0.5f + onset);
            }
            const float ck = 14.f, cd = 2.5f + visc * 4.f;
            slVX += (ck * (stx - slX) - cd * slVX) * dt;
            slVY += (ck * (sty - slY) - cd * slVY) * dt;
            slX += slVX * dt;
            slY += slVY * dt;
            const float sMax = R * 0.22f;
            slX = clampT(slX, -sMax, sMax);
            slY = clampT(slY, -sMax, sMax);
            offX = bx + slX;
            offY = by + slY;

            // ---- spikes ----
            for (int i = 0; i < N_SPIKES; ++i)
            {
                Spike &s = spikes[i];

                const V3 wd = m3mul(Rm, s.d);
                const float align = 0.5f + 0.5f * dot3(wd, magD);
                const float gate = 0.25f + 0.75f * act;
                const float localField = (field * (0.80f + 0.35f * align * magLen) + audioEnv * (0.45f + 0.70f * align) + highA * (0.15f + 0.55f * s.rnd) + pulse * 0.20f + punch * 0.30f + 0.15f * life) * gate;

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

            // ---- satellite droplets: wide 3D orbits, reach toward the mouse ----
            const float pull = std::max(smooth01(0.30f, 0.85f, field * (0.5f + 0.5f * act)), calmK);
            const V3 off3 = v3(offX, offY, 0.f);
            const float limDist = R * 0.9f * sizeScale;
            V3 toMouse = v3(mxL - offX, myL - offY, 0.f);
            const float tmLen = len3(toMouse);
            const V3 toMouseN = (tmLen > 1.f) ? norm3(toMouse) : v3(0.f, 0.f, 0.f);
            const float reach = std::min(tmLen, 70.f);

            for (int i = 0; i < N_DROPS; ++i)
            {
                Drop &d = drops[i];

                if (kick || hit)
                    d.dv += (kick ? 38.f : 22.f * (0.5f + onset)) * (0.4f + hash1(i * 1.3f + t));

                float tgt = d.rest * sizeScale * (1.f - pull) + bodyBase * 0.45f * pull;
                tgt += audioEnv * 3.f * std::sin(wt * 8.f + d.phase);
                tgt += highA * 4.f * std::sin(wt * 17.f + d.phase * 2.f) * (1.f - pull);
                tgt += 2.5f * (1.f - pull) * std::sin(wt * 1.3f + d.phase);

                const float k = 26.f;
                const float damp = 5.f + visc * 7.f;
                d.dv += (k * (tgt - d.dist) - damp * d.dv) * dt;
                d.dist += d.dv * dt;
                d.dist = clampT(d.dist, 0.f, limDist);

                d.ang += ((d.w * 2.f + spinVel * (0.3f + 0.4f * d.phase / 6.2832f)) * (1.f - pull)) * dt;

                const float ca = std::cos(d.ang), sa = std::sin(d.ang);
                const V3 local = d.e1 * (ca * d.dist) + d.e2 * (sa * d.dist);
                V3 P = m3mul(Rm, local);
                P.x *= 1.45f; // the tank is wide: orbits stretch horizontally
                P = P + off3 + magD * (field * magLen * act * R * 0.10f * (1.f - pull));

                // the cursor makes droplets reach out toward it
                if (mouseSense > 0.02f)
                    P = P + toMouseN * (reach * mouseSense * 0.45f * (0.3f + 0.7f * d.phase / 6.2832f) * (1.f - 0.5f * pull));

                project(P, d.x, d.y, d.sc);
                d.z = P.z;

                const float limX = EV_TANK_W * 0.5f - 4.f - d.r * d.sc;
                const float limY = EV_TANK_H * 0.5f - 4.f - d.r * d.sc;
                d.x = clampT(d.x, -limX, limX);
                d.y = clampT(d.y, -limY, limY);

                d.vis = smooth01(0.85f * bodyBase, 1.15f * bodyBase, d.dist);
            }

            buildGeometry();
        }

        // Builds the projected 3D scene for this frame.
        void buildGeometry()
        {
            const float R = EV_REF_R;
            const V3 off3 = v3(offX, offY, 0.f);

            for (int i = 0; i < GN; ++i)
            {
                const V3 d = ldir[i];
                const V3 wd = m3mul(Rm, d);
                wpos[i] = wd * surfaceR(d, wd) + off3;
            }

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
            // three smoothing passes give a clean, round outline at 256 bins
            float tmp[NB];
            for (int pass = 0; pass < 3; ++pass)
            {
                for (int b = 0; b < NB; ++b)
                    tmp[b] = 0.25f * sil[(b + NB - 1) % NB] + 0.5f * sil[b] + 0.25f * sil[(b + 1) % NB];
                for (int b = 0; b < NB; ++b)
                    sil[b] = tmp[b];
            }
            const float silMax = EV_TANK_H * 0.46f;
            for (int b = 0; b < NB; ++b)
                sil[b] = std::min(sil[b], silMax);

            // ---- spikes ----
            const float lean = magLen * 0.25f * act;
            const float limX = EV_TANK_W * 0.5f - 3.f;
            const float limY = EV_TANK_H * 0.5f - 3.f;
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

                // keep tips inside the glass rectangle
                s.tx = clampT(s.tx, -limX, limX);
                s.ty = clampT(s.ty, -limY, limY);

                s.front = nrm.z >= 0.f;
                s.hw = (0.9f + 1.5f * std::min(1.f, hpx / 8.f)) * sb;
                s.spec = std::pow(std::max(0.f, dot3(nrm, Hv)), 16.f);
                s.lit = 0.35f + 0.65f * std::max(0.f, dot3(nrm, Lv));
            }
        }

        // Finds the mouse over the module (rack coordinates), tells the module
        // about it, and smooths the "sense" amount used for visual reactions.
        void senseMouse(float dt)
        {
            mouseInTank = false;
            bool hov = false;
            float lx = 0.f, ly = 0.f;

            if (module)
            {
                ModuleWidget *mw = getAncestorOfType<ModuleWidget>();
                if (mw && APP && APP->scene && APP->scene->rack)
                {
                    const Vec loc = APP->scene->rack->getMousePos().minus(mw->box.pos);
                    hov = loc.x >= 0.f && loc.y >= 0.f && loc.x <= mw->box.size.x && loc.y <= mw->box.size.y;
                    lx = loc.x - (EV_DISP_X + EV_DISP_W * 0.5f);
                    ly = loc.y - (EV_DISP_Y + EV_AREA_H * 0.5f);
                }
            }

            if (hov)
            {
                if (mouseHad)
                {
                    const float inst = std::sqrt((lx - pmx) * (lx - pmx) + (ly - pmy) * (ly - pmy)) / std::max(dt, 0.001f);
                    mouseSpd += (inst - mouseSpd) * std::min(1.f, dt * 10.f);
                }
                pmx = lx;
                pmy = ly;
                mxL = lx;
                myL = ly;
                mouseInTank = std::fabs(lx) < EV_TANK_W * 0.5f && std::fabs(ly) < EV_TANK_H * 0.5f;
            }
            else
            {
                mouseSpd = 0.f;
            }
            mouseHad = hov;

            if (module)
            {
                module->mouseHover.store(hov);
                module->mouseX.store(lx);
                module->mouseY.store(ly);
                module->mouseSpeed.store(mouseSpd);
            }

            const float tgt = (module && hov && module->mouseEnabled) ? module->curioA.load() : 0.f;
            mouseSense += (tgt - mouseSense) * (1.f - std::exp(-6.f * dt));
        }

        // ----- Drawing (flat fills only) -----

        // White perforated bezel, groove and glass of the rectangular tank.
        void drawTank(NVGcontext *vg)
        {
            const float W = EV_DISP_W, H = EV_AREA_H, pad = EV_TANK_PAD;

            nvgBeginPath(vg);
            nvgRoundedRect(vg, 0.f, 0.f, W, H, 8.f);
            nvgFillColor(vg, nvgRGB(0xc4, 0xcb, 0xd6));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // perforation along the bezel
            nvgBeginPath(vg);
            for (float x = 10.f; x <= W - 10.f; x += 8.f)
            {
                nvgCircle(vg, x, pad * 0.5f, 0.85f);
                nvgCircle(vg, x, H - pad * 0.5f, 0.85f);
            }
            for (float y = 10.f; y <= H - 10.f; y += 8.f)
            {
                nvgCircle(vg, pad * 0.5f, y, 0.85f);
                nvgCircle(vg, W - pad * 0.5f, y, 0.85f);
            }
            nvgFillColor(vg, nvgRGBA(52, 60, 74, 215));
            nvgFill(vg);

            // groove
            nvgBeginPath(vg);
            nvgRoundedRect(vg, pad - 1.6f, pad - 1.6f, EV_TANK_W + 3.2f, EV_TANK_H + 3.2f, 5.f);
            nvgStrokeWidth(vg, 2.f);
            nvgStrokeColor(vg, nvgRGBA(0, 0, 0, 70));
            nvgStroke(vg);

            // glass
            nvgBeginPath(vg);
            nvgRoundedRect(vg, pad, pad, EV_TANK_W, EV_TANK_H, 4.f);
            nvgFillColor(vg, nvgRGB(0xea, 0xee, 0xf4));
            nvgFill(vg);

            // tank floor band and a soft vertical reflection
            nvgBeginPath(vg);
            nvgRect(vg, pad, pad + EV_TANK_H * 0.88f, EV_TANK_W, EV_TANK_H * 0.12f);
            nvgFillColor(vg, nvgRGBA(40, 52, 76, 16));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, pad + EV_TANK_W * 0.07f, pad + 4.f, 5.f, EV_TANK_H - 8.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 60));
            nvgFill(vg);
        }

        // Contact shadow on the tank floor; it shrinks and fades when the
        // creature swims higher.
        void drawShadow(NVGcontext *vg, float cx, float cy)
        {
            const float floorY = cy + EV_TANK_H * 0.5f - 10.f;
            const float h = floorY - (cy + offY);
            const float k = clampT(1.f - h / 150.f, 0.3f, 1.f);
            nvgBeginPath(vg);
            nvgEllipse(vg, cx + offX, floorY, bodyBase * 1.3f * k + 3.f, bodyBase * 0.30f * k + 1.5f);
            nvgFillColor(vg, nvgRGBA(40, 52, 76, (int)(46.f * k)));
            nvgFill(vg);
        }

        // Glowing food pellets.
        void drawFood(NVGcontext *vg, float cx, float cy)
        {
            if (!module)
                return;
            for (int i = 0; i < EV_FOODS; ++i)
            {
                const float show = module->foodShowA[i].load();
                if (show < 0.02f)
                    continue;
                const float x = cx + module->foodXA[i].load();
                const float y = cy + module->foodYA[i].load();
                const float pul = 0.5f + 0.5f * std::sin(t * 4.f + (float)i * 1.7f);
                const float r = 2.8f * show;

                nvgBeginPath(vg);
                nvgCircle(vg, x, y, r * 3.4f);
                nvgFillColor(vg, nvgRGBA(60, 190, 205, (int)(46.f * pul * show)));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, r);
                nvgFillColor(vg, nvgRGBA(30, 150, 170, (int)(255.f * show)));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, x - r * 0.3f, y - r * 0.3f, r * 0.35f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(230.f * show)));
                nvgFill(vg);
            }
        }

        // Soft ripples on the glass where the cursor is.
        void drawMouse(NVGcontext *vg, float cx, float cy)
        {
            if (mouseSense < 0.03f || !mouseInTank)
                return;
            const float x = cx + mxL, y = cy + myL;
            for (int k = 0; k < 3; ++k)
            {
                float ph = t * 0.9f + (float)k / 3.f;
                ph -= std::floor(ph);
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, 3.f + 16.f * ph);
                nvgStrokeWidth(vg, 1.f);
                nvgStrokeColor(vg, nvgRGBA(60, 110, 180, (int)((1.f - ph) * 70.f * mouseSense)));
                nvgStroke(vg);
            }
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, 1.6f);
            nvgFillColor(vg, nvgRGBA(60, 110, 180, (int)(120.f * mouseSense)));
            nvgFill(vg);
        }

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

        // Black silhouette with a cool rim plus normal-based specular highlights.
        void drawBody(NVGcontext *vg, float cx, float cy, float glossIn)
        {
            const float gloss = glossIn;
            const float bcx = cx + offX, bcy = cy + offY;

            silhouettePath(vg, bcx, bcy);
            nvgFillColor(vg, nvgRGB(0x0a, 0x0d, 0x13));
            nvgFill(vg);
            silhouettePath(vg, bcx, bcy);
            nvgStrokeWidth(vg, 1.f);
            nvgStrokeColor(vg, nvgRGBA(130, 180, 255, (int)(90.f * gloss)));
            nvgStroke(vg);

            for (int i = 0; i < GN; ++i)
            {
                const V3 &N = wnorm[i];
                if (N.z < 0.25f)
                    continue;
                // dots are smaller on the denser grid; alpha is weighted by
                // latitude so the poles don't glow brighter than the equator
                const float sc = psc[i] * 0.38f;
                const float gloss = glossIn * lw[i];
                const float px = cx + psx[i], py = cy + psy[i];

                const float dL = dot3(N, Lv);
                if (dL > 0.55f)
                {
                    const float sh = std::pow(dL, 8.f);
                    nvgBeginPath(vg);
                    nvgCircle(vg, px, py, 4.2f * sc);
                    nvgFillColor(vg, nvgRGBA(215, 232, 255, (int)(34.f * sh * gloss)));
                    nvgFill(vg);
                }

                const float dH = dot3(N, Hv);
                if (dH > 0.80f)
                {
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
                {
                    const float sp = std::pow(dH2, 18.f);
                    nvgBeginPath(vg);
                    nvgCircle(vg, px, py, 2.4f * sc);
                    nvgFillColor(vg, nvgRGBA(120, 175, 255, (int)(95.f * sp * gloss)));
                    nvgFill(vg);
                }
            }
        }

        // Spikes for the back (front=false) or front (front=true) of the body.
        void drawSpikes(NVGcontext *vg, float cx, float cy, float gloss, bool front)
        {
            // Pass 1: dark silhouettes
            nvgBeginPath(vg);
            for (int i = 0; i < N_SPIKES; ++i)
            {
                const Spike &s = spikes[i];
                if (!s.vis || s.front != front)
                    continue;

                const float bx0 = cx + s.bx, by0 = cy + s.by;
                const float tx = cx + s.tx, ty = cy + s.ty;
                const float dx = tx - bx0, dy = ty - by0;
                const float L = std::sqrt(dx * dx + dy * dy);

                if (L < 1.0f)
                {
                    nvgCircle(vg, bx0 + dx * 0.5f, by0 + dy * 0.5f, s.hw * 0.8f);
                    continue;
                }
                const float ux = dx / L, uy = dy / L;
                const float px = -uy, py = ux;
                const float hw = s.hw;
                const float mxp = bx0 + ux * L * 0.5f, myp = by0 + uy * L * 0.5f;

                nvgMoveTo(vg, bx0 - px * hw, by0 - py * hw);
                nvgQuadTo(vg, mxp - px * hw * 0.35f, myp - py * hw * 0.35f, tx, ty);
                nvgQuadTo(vg, mxp + px * hw * 0.35f, myp + py * hw * 0.35f, bx0 + px * hw, by0 + py * hw);
                nvgClosePath(vg);
            }
            nvgFillColor(vg, nvgRGB(0x05, 0x06, 0x09));
            nvgFill(vg);

            // Pass 2: edge glints
            nvgBeginPath(vg);
            for (int i = 0; i < N_SPIKES; ++i)
            {
                const Spike &s = spikes[i];
                if (!s.vis || s.front != front)
                    continue;

                const float bx0 = cx + s.bx, by0 = cy + s.by;
                const float tx = cx + s.tx, ty = cy + s.ty;
                const float dx = tx - bx0, dy = ty - by0;
                const float L = std::sqrt(dx * dx + dy * dy);
                if (L < 3.f)
                    continue;
                const float ux = dx / L;
                nvgMoveTo(vg, bx0 - (-dy / L) * s.hw * 0.45f, by0 - ux * s.hw * 0.45f);
                nvgLineTo(vg, bx0 + dx * 0.96f, by0 + dy * 0.96f);
            }
            nvgStrokeWidth(vg, 0.6f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStrokeColor(vg, nvgRGBA(190, 208, 235, (int)((front ? 110.f : 70.f) * gloss)));
            nvgStroke(vg);

            // Pass 3: sparkling tips
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

        // Orbiting droplets for the back or front of the body.
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

        // Purple bioluminescence, drawn on Rack's light layer so it stays
        // bright when the room is dimmed: halo, glowing rim, spike lines and
        // tips, droplets and food pellets.
        void drawGlow(NVGcontext *vg)
        {
            if (glowK < 0.01f)
                return;
            auto A = [](float v)
            { return (int)clampT(v, 0.f, 255.f); };

            const float cx = EV_DISP_W * 0.5f, cy = EV_AREA_H * 0.5f;
            const float k = clampT(glowK * (0.75f + 0.25f * act + 0.3f * punch), 0.f, 1.2f);
            const float bcx = cx + offX, bcy = cy + offY;

            nvgSave(vg);
            nvgScissor(vg, EV_TANK_PAD, EV_TANK_PAD, EV_TANK_W, EV_TANK_H);
            nvgGlobalCompositeOperation(vg, NVG_LIGHTER);

            // soft halo around the body
            const float reach = bodyBase * (1.8f + 1.8f * k) + 14.f * act;
            const NVGpaint halo = nvgRadialGradient(vg, bcx, bcy, bodyBase * 0.6f, reach,
                                                    nvgRGBA(150, 60, 255, A(150.f * k)), nvgRGBA(150, 60, 255, 0));
            nvgBeginPath(vg);
            nvgCircle(vg, bcx, bcy, reach);
            nvgFillPaint(vg, halo);
            nvgFill(vg);

            // glowing rim (wide soft stroke + thin bright stroke)
            silhouettePath(vg, bcx, bcy);
            nvgStrokeWidth(vg, 5.f);
            nvgStrokeColor(vg, nvgRGBA(140, 60, 255, A(60.f * k)));
            nvgStroke(vg);
            silhouettePath(vg, bcx, bcy);
            nvgStrokeWidth(vg, 1.4f);
            nvgStrokeColor(vg, nvgRGBA(190, 120, 255, A(210.f * k)));
            nvgStroke(vg);

            // spikes glow along their length and sparkle at the tips
            nvgBeginPath(vg);
            for (int i = 0; i < N_SPIKES; ++i)
            {
                const Spike &s = spikes[i];
                if (!s.vis)
                    continue;
                nvgMoveTo(vg, cx + s.bx, cy + s.by);
                nvgLineTo(vg, cx + s.tx, cy + s.ty);
            }
            nvgStrokeWidth(vg, 1.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStrokeColor(vg, nvgRGBA(170, 90, 255, A(150.f * k)));
            nvgStroke(vg);

            nvgBeginPath(vg);
            for (int i = 0; i < N_SPIKES; ++i)
            {
                const Spike &s = spikes[i];
                if (!s.vis)
                    continue;
                nvgCircle(vg, cx + s.tx, cy + s.ty, 1.3f);
            }
            nvgFillColor(vg, nvgRGBA(225, 175, 255, A(190.f * k)));
            nvgFill(vg);

            // droplets
            for (int i = 0; i < N_DROPS; ++i)
            {
                const Drop &d = drops[i];
                if (d.vis < 0.02f)
                    continue;
                const float rr = d.r * d.sc * std::sqrt(d.vis);
                nvgBeginPath(vg);
                nvgCircle(vg, cx + d.x, cy + d.y, rr * 2.4f + 1.f);
                nvgFillColor(vg, nvgRGBA(150, 60, 255, A(55.f * k * d.vis)));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, cx + d.x, cy + d.y, rr + 0.4f);
                nvgStrokeWidth(vg, 1.f);
                nvgStrokeColor(vg, nvgRGBA(200, 140, 255, A(180.f * k * d.vis)));
                nvgStroke(vg);
            }

            // food pellets glow cyan in the dark
            if (module)
            {
                for (int i = 0; i < EV_FOODS; ++i)
                {
                    const float show = module->foodShowA[i].load();
                    if (show < 0.02f)
                        continue;
                    const float x = cx + module->foodXA[i].load();
                    const float y = cy + module->foodYA[i].load();
                    nvgBeginPath(vg);
                    nvgCircle(vg, x, y, 9.f * show);
                    nvgFillColor(vg, nvgRGBA(60, 200, 220, A(70.f * k * show)));
                    nvgFill(vg);
                    nvgBeginPath(vg);
                    nvgCircle(vg, x, y, 2.4f * show);
                    nvgFillColor(vg, nvgRGBA(170, 245, 255, A(220.f * k * show)));
                    nvgFill(vg);
                }
            }

            nvgRestore(vg);
        }

        // Layer 1 is drawn after Rack's room dimming, so the glow shines.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer == 1)
                drawGlow(args.vg);
            Widget::drawLayer(args, layer);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            const double now = system::getTime();
            float dt = 1.f / 60.f;
            if (lastTime > 0.0)
                dt = clampT((float)(now - lastTime), 0.001f, 0.05f);
            lastTime = now;

            // room brightness below 60% -> purple glow, stronger the darker it gets
            const float glowTarget = smooth01(0.6f, 0.f, settings::rackBrightness);
            glowK += (glowTarget - glowK) * (1.f - std::exp(-4.f * dt));

            if (module)
            {
                const int rs = module->resetSeq.load();
                if (rs != lastResetSeq)
                {
                    lastResetSeq = rs;
                    resetLife();
                }
            }

            senseMouse(dt);

            const bool paused = module && module->paused;
            if (!paused)
                stepPhysics(dt);

            agmm::screen(vg, 0.f, 0.f, EV_DISP_W, EV_DISP_H);

            const float cx = EV_DISP_W * 0.5f;
            const float cy = EV_AREA_H * 0.5f;
            const float gloss = module ? module->glossA.load() : 0.75f;

            drawTank(vg);

            nvgSave(vg);
            nvgScissor(vg, EV_TANK_PAD, EV_TANK_PAD, EV_TANK_W, EV_TANK_H);
            drawShadow(vg, cx, cy);
            drawFood(vg, cx, cy);
            drawMouse(vg, cx, cy);

            // back-to-front: far spikes and droplets, the body, then near spikes and droplets
            drawSpikes(vg, cx, cy, gloss, false);
            drawDrops(vg, cx, cy, gloss, false);
            drawBody(vg, cx, cy, gloss);
            drawSpikes(vg, cx, cy, gloss, true);
            drawDrops(vg, cx, cy, gloss, true);
            nvgRestore(vg);

            // glass lip
            nvgBeginPath(vg);
            nvgRoundedRect(vg, EV_TANK_PAD + 0.5f, EV_TANK_PAD + 0.5f, EV_TANK_W - 1.f, EV_TANK_H - 1.f, 4.f);
            nvgStrokeWidth(vg, 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 120));
            nvgStroke(vg);

            // divider + readout row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, EV_AREA_H + 0.5f);
            nvgLineTo(vg, EV_DISP_W, EV_AREA_H + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            const float ry = EV_AREA_H + 7.f;
            char buf[48];
            const float fld = module ? module->fieldSmoothed.load() : 0.58f;
            const float env = module ? module->envSmoothed.load() : 0.1f;
            snprintf(buf, sizeof(buf), "FIELD %3.0f%%  NRG %3.0f%%", clampT(fld, 0.f, 1.f) * 100.f, clampT(energy, 0.f, 1.f) * 100.f);
            agmm::text(vg, 6.f, ry, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "ENV %4.1fV", clampT(env * 10.f, 0.f, 10.f));
            agmm::text(vg, EV_DISP_W - 6.f, ry, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);
            const char *st = paused ? "PAUSED" : ((mood == Evolved::MOOD_CURIOUS && trust < 0.35f) ? "WARY" : EV_MOOD_NAMES[clampT(mood, 0, 5)]);
            agmm::text(vg, EV_DISP_W * 0.5f, ry, st, 7.f, paused ? agmm::warm() : agmm::dim(), agmm::ALIGN_C, 1.f);

            Widget::draw(args);
        }
    };

    // =====================================================================
    //  Panel background
    // =====================================================================
    struct EvPanelBackground : TransparentWidget
    {
        EvPanelBackground() { box.size = Vec(EV_PANEL_W, EV_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, EV_PANEL_W, EV_PANEL_H, "EVOLVED");
            agmm::bezel(vg, EV_DISP_X, EV_DISP_Y, EV_DISP_W, EV_DISP_H);

            // Controls: knob + CV jack stacked in each plate
            agmm::sectionHeader(vg, EV_PANEL_W, EV_SEC1_Y, "CONTROLS", 10.f);
            for (int i = 0; i < 10; ++i)
            {
                const float cx = evCtrlX(i);
                agmm::plate(vg, cx, EV_CTRL_TOP, EV_CTRL_W, EV_CTRL_H, false);
                agmm::text(vg, cx, EV_CTRL_TOP + 9.f, EV_KNOBS[i].name, 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            // I/O + buttons: inputs and buttons plain, outputs tinted
            agmm::sectionHeader(vg, EV_PANEL_W, EV_SEC2_Y, "I / O", 10.f);
            for (int i = 0; i < 12; ++i)
            {
                const float cx = evIoX(i);
                agmm::plate(vg, cx, EV_IO_TOP, EV_IO_W, EV_IO_H, i >= 2 && i < 8);
                agmm::text(vg, cx, EV_IO_TOP + 8.f, EV_IO_LABELS[i], 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            Widget::draw(args);
        }
    };

} // namespace

// =====================================================================
//  Module widget
// =====================================================================
struct EvolvedWidget : ModuleWidget
{
    explicit EvolvedWidget(Evolved *module)
    {
        setModule(module);
        box.size = Vec(EV_PANEL_W, EV_PANEL_H);

        addChild(new EvPanelBackground());
        addChild(new EvolvedDisplay(module));

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        // ten knobs, each with its CV jack underneath
        for (int i = 0; i < 10; ++i)
        {
            const float cx = evCtrlX(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, EV_CTRL_TOP + 30.f), module, EV_KNOBS[i].param));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx, EV_CTRL_TOP + 55.f), module, EV_KNOBS[i].cv));
        }
        agmm::addKnobArcs(this, 2.f);
        // I/O band: 2 inputs, 6 outputs, 4 buttons
        const float jy = EV_IO_TOP + 24.f;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(evIoX(0), jy), module, Evolved::AUDIO_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(evIoX(1), jy), module, Evolved::PULSE_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(evIoX(2), jy), module, Evolved::ENV_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(evIoX(3), jy), module, Evolved::FIELD_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(evIoX(4), jy), module, Evolved::X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(evIoX(5), jy), module, Evolved::Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(evIoX(6), jy), module, Evolved::ENERGY_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(evIoX(7), jy), module, Evolved::EAT_OUTPUT));

        addParam(createParamCentered<TL1105>(Vec(evIoX(8), jy), module, Evolved::PAUSE_PARAM));
        addParam(createParamCentered<TL1105>(Vec(evIoX(9), jy), module, Evolved::RESET_PARAM));
        addParam(createParamCentered<TL1105>(Vec(evIoX(10), jy), module, Evolved::AUDIO_PARAM));
        addParam(createParamCentered<TL1105>(Vec(evIoX(11), jy), module, Evolved::MOUSE_PARAM));
        addChild(createLightCentered<SmallLight<agmm::VioletLight>>(Vec(evIoX(8) + 13.f, jy), module, Evolved::PAUSE_LIGHT));
        addChild(createLightCentered<SmallLight<agmm::CyanLight>>(Vec(evIoX(10) + 13.f, jy), module, Evolved::AUDIO_LIGHT));
        addChild(createLightCentered<SmallLight<agmm::CyanLight>>(Vec(evIoX(11) + 13.f, jy), module, Evolved::MOUSE_LIGHT));
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
Model *modelEvolved = createModel<Evolved, EvolvedWidget>("Evolved");