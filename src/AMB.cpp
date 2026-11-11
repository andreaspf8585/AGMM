#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <random>

// ============================================================================
// AMB – generative ambient machine (Eno / AMB-1 style)
// ============================================================================

// ------------------------------------------------------------
// Layout constants (16 HP = 240×380)
// ------------------------------------------------------------
static const float AMB_W = RACK_GRID_WIDTH * 16;
static const float AMB_H = RACK_GRID_HEIGHT;

static const float AMB_DX = 20.f, AMB_DY = 30.f, AMB_DW = 200.f, AMB_DH = 90.f;
static const float AMB_TX0 = 12.f, AMB_TY0 = 18.f, AMB_TW = 164.f, AMB_TH = 46.f;

static const float AMB_KCOL[4]  = {34.f, 91.f, 149.f, 206.f};
static const float AMB_CELL_W   = 54.f;

static const float AMB_KBIG_CELL_TOP = 128.f;
static const float AMB_KBIG_CELL_H   = 58.f;
static const float AMB_KBIG_CY       = 162.f;

static const float AMB_KS1_CELL_TOP = 192.f;
static const float AMB_KS1_CELL_H   = 38.f;
static const float AMB_KS1_CY       = 220.f;

static const float AMB_KS2_CELL_TOP = 234.f;
static const float AMB_KS2_CELL_H   = 38.f;
static const float AMB_KS2_CY       = 262.f;

static const float AMB_KS3_CELL_TOP = 276.f;
static const float AMB_KS3_CELL_H   = 38.f;
static const float AMB_KS3_CY       = 304.f;

static const float AMB_IO_CELL_TOP = 320.f;
static const float AMB_IO_CELL_H   = 46.f;
static const float AMB_IO_CY       = 350.f;

static const int AMB_NSCALES = 6;
static const char *AMB_SCALE_NAMES[AMB_NSCALES] = {
    "MAJOR", "MINOR", "DORIAN", "PHRYGIAN", "LYDIAN", "MIXOLYDIAN"
};
static const int AMB_SCALE_INTERVALS[AMB_NSCALES][7] = {
    {0, 2, 4, 5, 7, 9, 11},
    {0, 2, 3, 5, 7, 8, 10},
    {0, 2, 3, 5, 7, 9, 10},
    {0, 1, 3, 5, 7, 8, 10},
    {0, 2, 4, 6, 7, 9, 11},
    {0, 2, 4, 5, 7, 9, 10}
};
static const char *AMB_NOTE_NAMES[12] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

static const int   AMB_PITCH_MIN = -12;   // C2 (was 0)
static const int   AMB_PITCH_MAX = 48;    // C6
static const float AMB_HIST_WIN  = 15.f;

// 8 tape loops of unequal prime lengths
static const int AMB_LOOP_BEATS[8] = { 17, 23, 29, 31, 37, 41, 43, 47 };

// 6 Euclidean patterns (32-beat cycle across all)
static const int AMB_EUCLID[6][2]  = {
    { 32, 5 }, { 24, 4 }, { 16, 2 }, { 28, 3 }, { 20, 5 }, { 12, 3 }
};

// ---------------------------------------------------------------------------
// Param quantities
// ---------------------------------------------------------------------------
struct KeyParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        int v = clamp((int)std::round(getValue()), 0, 11);
        return AMB_NOTE_NAMES[v];
    }
};
struct ScaleParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        int v = clamp((int)std::round(getValue()), 0, AMB_NSCALES - 1);
        return AMB_SCALE_NAMES[v];
    }
};
struct TempoParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        return string::f("%.0f BPM", getValue());
    }
};
struct ChordIntervalParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        return string::f("%.0f s", getValue());
    }
};

// ============================================================================
// AmbEngine
// ============================================================================
struct AmbEngine
{
    // ---------------------------------------------------------------
    // Parameters
    // ---------------------------------------------------------------
    int   key          = 0;
    int   scaleIdx     = 0;
    float tempoBpm     = 120.f;
    float padLevel     = 0.55f;
    float bellLevel    = 0.55f;
    float bassLevel    = 0.55f;
    float pulseLevel   = 0.50f;
    float echoMix      = 0.40f;
    float echoFbk      = 0.55f;
    float reverbMix    = 0.55f;
    float reverbDecay  = 0.55f;
    float chordInterval = 24.f;

    // ---------------------------------------------------------------
    // Transport
    // ---------------------------------------------------------------
    double beatPhase      = 0.0;
    float  lastSampleRate = 0.f;

    // ---------------------------------------------------------------
    // Pad – 8 voices
    // ---------------------------------------------------------------
    static constexpr int PAD_VOICES = 8;
    float padPhase[PAD_VOICES]   = {};
    float padDetune[PAD_VOICES]  = {
        -0.08f, -0.055f, -0.03f, -0.01f,
         0.01f,  0.03f,   0.055f, 0.08f
    };
    float padLfo[PAD_VOICES]     = {};
    float padLfoRate[PAD_VOICES] = {
        0.07f, 0.11f, 0.05f, 0.13f,
        0.09f, 0.03f, 0.10f, 0.15f
    };
    float padChordRoot  = 12.f;
    float padTargetRoot = 12.f;
    float padChordTimer = 0.f;

    // ---------------------------------------------------------------
    // Bells – 8 loops
    // ---------------------------------------------------------------
    static constexpr int NBELLS = 8;
    float bellLoopPos[NBELLS] = {};
    int   bellDegree[NBELLS]  = {};
    float bellPhase[NBELLS]   = {};
    float bellEnv[NBELLS]     = {};
    float bellFreq[NBELLS]    = {};
    float bellModPhase[NBELLS]= {};
    float bellMod   = 1.5f;
    float bellIndex = 0.5f;

    // ---------------------------------------------------------------
    // Bass – 2 voices (sub + mid, one octave apart)
    // ---------------------------------------------------------------
    float bassPhaseSub = 0.f;
    float bassPhaseMid = 0.f;
    float bassPitch    = 0.f;   // -12..0
    float bassTarget   = 0.f;
    float bassLfoPhase = 0.f;

    // ---------------------------------------------------------------
    // Pulse – 6 Euclidean patterns
    // ---------------------------------------------------------------
    static constexpr int NPULSE = 6;
    int   euclidStep[NPULSE]    = {};
    float pulseEnv[NPULSE]      = {};
    float pulseHatEnv[NPULSE]   = {};
    float pulsePhase[NPULSE]    = {};
    float hatLP[NPULSE]         = {};
    bool  euclidPat[NPULSE][32] = {};
    int   euclidLen[NPULSE]     = {};

    // ---------------------------------------------------------------
    // Tape echo
    // ---------------------------------------------------------------
    static constexpr size_t ECHO_SIZE = 96000;
    std::vector<float> echoBufL, echoBufR;
    int   echoWrite = 0;
    int   echoTap1  = 0;
    int   echoTap2  = 0;

    // ---------------------------------------------------------------
    // Dattorro plate reverb
    // ---------------------------------------------------------------
    static constexpr int PRE_DELAY = 220;
    std::vector<float> preBuf;
    int   preIdx = 0;

    struct Allpass
    {
        std::vector<float> buf;
        int   idx = 0;
        float g   = 0.75f;
        void init(int delay) { buf.assign(std::max(delay, 1), 0.f); }
        float process(float x)
        {
            int n = (int)buf.size();
            float yn = buf[idx];
            float vn = x + g * yn;
            buf[idx] = vn;
            idx = (idx + 1) % n;
            return yn - g * vn;
        }
    };
    Allpass ap1, ap2, ap3, ap4;

    struct ModAllpass
    {
        std::vector<float> buf;
        int   idx = 0;
        float g   = 0.75f;
        float modPhase = 0.f;
        float modRate  = 0.5f;
        float modDepth = 4.f;
        void init(int delay) { buf.assign(std::max(delay, 1), 0.f); }
        float process(float x, float sr)
        {
            int n = (int)buf.size();
            modPhase += modRate / sr;
            if (modPhase > 1.f) modPhase -= 1.f;
            float mod = (1.f + std::sin(2.f * (float)M_PI * modPhase))
                      * 0.5f * modDepth;
            int r = (idx + (int)std::round(mod)) % n;
            float yn = buf[r];
            float vn = x + g * yn;
            buf[idx] = vn;
            idx = (idx + 1) % n;
            return yn - g * vn;
        }
    };
    ModAllpass tankAP[8];
    std::vector<float> tankDelay[4];
    int tankDelayIdx[4] = {};

    // ---------------------------------------------------------------
    // Display snapshot & output
    // ---------------------------------------------------------------
    struct NoteEvent { float age; float pitch; int layer; };
    std::vector<NoteEvent> history;
    static constexpr size_t HIST_MAX = 128;
    float meterL = 0.f, meterR = 0.f;
    float curChordRoot = 12.f;
    float outL = 0.f, outR = 0.f;

    float masterLP_L = 0.f, masterLP_R = 0.f;

    std::mt19937 rng;
    std::uniform_real_distribution<float> dist01{0.f, 1.f};

    // ---------------------------------------------------------------
    AmbEngine()
    {
        rng.seed(std::random_device{}());

        echoBufL.assign(ECHO_SIZE, 0.f);
        echoBufR.assign(ECHO_SIZE, 0.f);
        echoTap1 = (int)(0.30f * 48000.f);
        echoTap2 = (int)(1.20f * 48000.f);

        preBuf.assign(PRE_DELAY * 2, 0.f);

        ap1.init(142);
        ap2.init(107);
        ap3.init(379);
        ap4.init(277);

        int apLens[8] = { 672, 908, 1800, 2656, 672, 908, 1800, 2656 };
        for (int i = 0; i < 8; i++)
        {
            tankAP[i].init(apLens[i]);
            tankAP[i].g        = 0.75f;
            tankAP[i].modRate  = 0.5f + 0.1f * i;
            tankAP[i].modDepth = 4.f;
        }
        int dLens[4] = { 4453, 3720, 4217, 3163 };
        for (int i = 0; i < 4; i++)
            tankDelay[i].assign(std::max(dLens[i], 1), 0.f);

        for (int p = 0; p < NPULSE; p++)
        {
            int steps  = AMB_EUCLID[p][0];
            int pulses = AMB_EUCLID[p][1];
            euclidLen[p] = steps;
            for (int i = 0; i < steps; i++)
                euclidPat[p][i] = ((i * pulses) % steps) < pulses;
        }

        history.reserve(HIST_MAX + 2);
        pickNewChord();
    }

    // ---------------------------------------------------------------
    void pickNewChord()
    {
        const int *scale = AMB_SCALE_INTERVALS[scaleIdx];

        // Pad root: octave 1 or 2, cap at E5 (pitch 28)
        int octave = 1 + (dist01(rng) < 0.6f ? 0 : 1);
        int degree = std::min(6, (int)(dist01(rng) * 7.f));
        int semitone = octave * 12 + scale[degree] + key;
        while (semitone > 28) semitone -= 12;
        while (semitone < 12) semitone += 12;
        padTargetRoot = (float)semitone;
        padChordTimer = 0.f;

        // Bass: two octaves below pad, wrapped to pitch range [-12, 0]
        float b = padTargetRoot - 24.f;
        while (b < -12.f) b += 12.f;
        while (b > 0.f)   b -= 12.f;
        bassTarget = b;

        for (int bb = 0; bb < NBELLS; bb++)
            bellDegree[bb] = std::min(6, (int)(dist01(rng) * 7.f));

        addHistory(padTargetRoot, 0);
        addHistory(bassTarget, 2);
    }

    void addHistory(float pitch, int layer)
    {
        if (history.size() >= HIST_MAX)
            history.erase(history.begin());
        history.push_back({ 0.f, pitch, layer });
    }

    // ---------------------------------------------------------------
    void process(float sampleRate, float dt)
    {
        // 0. SR-dependent taps
        if (sampleRate != lastSampleRate)
        {
            echoTap1 = std::min((int)(0.30f * sampleRate), (int)ECHO_SIZE - 1);
            echoTap2 = std::min((int)(1.20f * sampleRate), (int)ECHO_SIZE - 1);
            lastSampleRate = sampleRate;
        }
        const float masterLpCoef = 1.f - std::exp(-2.f * (float)M_PI * 4000.f * dt);
        const float hatLpCoef    = 1.f - std::exp(-2.f * (float)M_PI * 3000.f * dt);

        // 1. Transport
        const float bps = std::max(tempoBpm, 20.f) / 60.f;
        beatPhase += bps * dt;

        // 2. Pad (8 voices)
        padChordTimer += dt;
        if (padChordTimer >= std::max(chordInterval, 2.f))
            pickNewChord();

        padChordRoot += (padTargetRoot - padChordRoot)
                      * (1.f - std::exp(-dt / 2.f));
        curChordRoot = padChordRoot;

        float padL = 0.f, padR = 0.f;
        for (int v = 0; v < PAD_VOICES; v++)
        {
            padLfo[v] += padLfoRate[v] * dt;
            if (padLfo[v] > 1.f) padLfo[v] -= 1.f;
            float lfo = 0.5f + 0.5f * std::sin(2.f * (float)M_PI * padLfo[v]);
            float voicePitch = padChordRoot + padDetune[v] * 12.f;
            float freq = 130.81f * std::pow(2.f, voicePitch / 12.f);
            padPhase[v] += freq * dt;
            if (padPhase[v] >= 1.f) padPhase[v] -= 1.f;
            float osc = std::sin(2.f * (float)M_PI * padPhase[v]);
            float env = 0.6f + 0.4f * lfo;
            float s   = osc * env * 0.09f;   // 8 voices × 0.09 ≈ 0.72 max (same total as before)
            float pan = (float)v / (PAD_VOICES - 1) * 2.f - 1.f;
            padL += s * (1.f - pan) * 0.5f;
            padR += s * (1.f + pan) * 0.5f;
        }
        padL *= padLevel;
        padR *= padLevel;

        // 3. Bells (8 loops)
        float bellL = 0.f, bellR = 0.f;
        for (int b = 0; b < NBELLS; b++)
        {
            float loopBeats = (float)AMB_LOOP_BEATS[b];
            float prevPos   = bellLoopPos[b];
            bellLoopPos[b]  = std::fmod((float)beatPhase, loopBeats);
            if (bellLoopPos[b] < prevPos)
            {
                const int *scale = AMB_SCALE_INTERVALS[scaleIdx];
                int deg  = std::min(6, bellDegree[b]);
                int oct  = 1 + (b % 3 == 0 ? 1 : 0);   // ~1/3 of bells one octave up
                int semi = oct * 12 + scale[deg] + key;
                while (semi > 30) semi -= 12;          // cap at F#5
                while (semi < 0)  semi += 12;
                bellFreq[b]     = 130.81f * std::pow(2.f, (float)semi / 12.f);
                bellEnv[b]      = 1.f;
                bellPhase[b]    = 0.f;
                bellModPhase[b] = 0.f;
                addHistory((float)semi, 1);
                bellDegree[b] = std::min(6, (int)(dist01(rng) * 7.f));
            }
            if (bellEnv[b] > 0.001f)
            {
                float carrierFreq = bellFreq[b];
                float modFreq     = carrierFreq * bellMod;
                bellModPhase[b] += modFreq * dt;
                if (bellModPhase[b] > 1.f) bellModPhase[b] -= 1.f;
                float mod = std::sin(2.f * (float)M_PI * bellModPhase[b]);
                bellPhase[b] += carrierFreq * dt;
                if (bellPhase[b] > 1.f) bellPhase[b] -= 1.f;
                float carrier = std::sin(2.f * (float)M_PI * bellPhase[b]
                                        + bellIndex * mod);
                float s = carrier * bellEnv[b] * 0.09f;   // 8 voices × 0.09 ≈ same total
                bellEnv[b] *= std::exp(-dt / 1.5f);
                float pan = ((float)b / (NBELLS - 1)) * 1.6f - 0.8f;
                bellL += s * (1.f - pan) * 0.5f;
                bellR += s * (1.f + pan) * 0.5f;
            }
        }
        bellL *= bellLevel;
        bellR *= bellLevel;

        // 4. Bass (2 voices: sub + mid one octave up)
        bassPitch += (bassTarget - bassPitch) * (1.f - std::exp(-dt / 1.5f));

        float bassSubFreq = 130.81f * std::pow(2.f, bassPitch / 12.f);
        float bassMidFreq = bassSubFreq * 2.f;

        bassPhaseSub += bassSubFreq * dt;
        if (bassPhaseSub >= 1.f) bassPhaseSub -= 1.f;
        float bassSub = std::sin(2.f * (float)M_PI * bassPhaseSub);
        bassSub += 0.30f * std::sin(4.f * (float)M_PI * bassPhaseSub);
        bassSub += 0.15f * std::sin(6.f * (float)M_PI * bassPhaseSub);

        bassPhaseMid += bassMidFreq * dt;
        if (bassPhaseMid >= 1.f) bassPhaseMid -= 1.f;
        float bassMid = std::sin(2.f * (float)M_PI * bassPhaseMid);
        bassMid += 0.25f * std::sin(4.f * (float)M_PI * bassPhaseMid);

        bassLfoPhase += 0.0625f * bps * dt;
        if (bassLfoPhase > 1.f) bassLfoPhase -= 1.f;
        float bassEnv = 0.6f + 0.4f * std::sin(2.f * (float)M_PI * bassLfoPhase);

        float bassSig = (bassSub * 0.50f + bassMid * 0.30f) * bassEnv * bassLevel;

        // 5. Pulse (6 Euclidean patterns)
        float pulseL = 0.f, pulseR = 0.f;
        for (int p = 0; p < NPULSE; p++)
        {
            int len = euclidLen[p];
            int newStep = ((int)std::floor((float)beatPhase / 32.f * (float)len)) % len;
            if (newStep < 0) newStep += len;
            if (newStep != euclidStep[p])
            {
                euclidStep[p] = newStep;
                if (euclidPat[p][newStep])
                {
                    pulseEnv[p]    = 1.f;
                    pulseHatEnv[p] = 1.f;
                    addHistory(bassTarget, 3);   // draw pulse at bass pitch
                }
            }
            if (pulseEnv[p] > 0.001f)
            {
                float thumpFreq = 60.f + 40.f * (p % 3);
                pulsePhase[p] += thumpFreq * dt;
                if (pulsePhase[p] >= 1.f) pulsePhase[p] -= 1.f;
                float thump = std::sin(2.f * (float)M_PI * pulsePhase[p]);
                float s = thump * pulseEnv[p] * 0.07f;
                pulseL += s;
                pulseR += s;
                pulseEnv[p] *= std::exp(-dt / 0.20f);
            }
            if (pulseHatEnv[p] > 0.001f)
            {
                float noise = dist01(rng) * 2.f - 1.f;
                hatLP[p] += (noise - hatLP[p]) * hatLpCoef;
                float hat = hatLP[p] * pulseHatEnv[p] * 0.05f;
                pulseL += hat * 0.6f;
                pulseR += hat * 0.4f;
                pulseHatEnv[p] *= std::exp(-dt / 0.04f);
            }
        }
        pulseL *= pulseLevel;
        pulseR *= pulseLevel;

        // 6. Dry sum
        float dryL = padL + bellL + bassSig + pulseL;
        float dryR = padR + bellR + bassSig + pulseR;
        dryL = std::tanh(dryL);
        dryR = std::tanh(dryR);

        // 7. Tape echo (internal feedback scaled for stability)
        int read1 = (echoWrite - echoTap1 + (int)ECHO_SIZE) % (int)ECHO_SIZE;
        int read2 = (echoWrite - echoTap2 + (int)ECHO_SIZE) % (int)ECHO_SIZE;
        float e1L = echoBufL[read1];
        float e1R = echoBufR[read1];
        float e2L = echoBufL[read2];
        float e2R = echoBufR[read2];

        float fbk = clamp(echoFbk, 0.f, 0.9f) * 0.65f;    // scaled internally
        echoBufL[echoWrite] = dryL + e1R * fbk + e2R * fbk * 0.5f;
        echoBufR[echoWrite] = dryR + e1L * fbk + e2L * fbk * 0.5f;
        echoWrite = (echoWrite + 1) % (int)ECHO_SIZE;

        float em = clamp(echoMix, 0.f, 1.f);
        float mixL = dryL * (1.f - em) + (e1L + e2L) * 0.5f * em;
        float mixR = dryR * (1.f - em) + (e1R + e2R) * 0.5f * em;

        // 8. Dattorro plate reverb
        preBuf[preIdx] = (mixL + mixR) * 0.5f;
        int preRead = (preIdx - PRE_DELAY + (int)preBuf.size()) % (int)preBuf.size();
        float pre = preBuf[preRead];
        preIdx = (preIdx + 1) % (int)preBuf.size();

        float diff = ap1.process(pre);
        diff = ap2.process(diff);
        diff = ap3.process(diff);
        diff = ap4.process(diff);

        float tankL = diff, tankR = diff;
        tankL = tankAP[0].process(tankL, sampleRate);
        tankL = tankAP[1].process(tankL, sampleRate);
        tankL = tankAP[2].process(tankL, sampleRate);
        tankL = tankAP[3].process(tankL, sampleRate);
        tankR = tankAP[4].process(tankR, sampleRate);
        tankR = tankAP[5].process(tankR, sampleRate);
        tankR = tankAP[6].process(tankR, sampleRate);
        tankR = tankAP[7].process(tankR, sampleRate);

        float tankFb = 0.30f + clamp(reverbDecay, 0.f, 1.f) * 0.40f;
        auto processDelay = [&](std::vector<float> &buf, int &idx, float in) -> float
        {
            int n = (int)buf.size();
            float y = buf[idx];
            buf[idx] = in + y * tankFb;
            idx = (idx + 1) % n;
            return y;
        };

        float d1 = processDelay(tankDelay[0], tankDelayIdx[0], tankL);
        float d2 = processDelay(tankDelay[1], tankDelayIdx[1], tankR);
        float d3 = processDelay(tankDelay[2], tankDelayIdx[2], tankL);
        float d4 = processDelay(tankDelay[3], tankDelayIdx[3], tankR);

        float revL = (d1 + d3) * 0.5f;
        float revR = (d2 + d4) * 0.5f;

        // 9. Final mix + master lowpass
        float wm = clamp(reverbMix, 0.f, 1.f);
        float finalL = mixL * (1.f - wm) + revL * wm;
        float finalR = mixR * (1.f - wm) + revR * wm;

        masterLP_L += (finalL - masterLP_L) * masterLpCoef;
        masterLP_R += (finalR - masterLP_R) * masterLpCoef;

        outL = std::tanh(masterLP_L);
        outR = std::tanh(masterLP_R);

        // 10. UI snapshot
        meterL = std::max(std::fabs(outL), meterL * (1.f - 5.f * dt));
        meterR = std::max(std::fabs(outR), meterR * (1.f - 5.f * dt));

        for (auto &e : history) e.age += dt;
        while (history.size() > HIST_MAX)
            history.erase(history.begin());
    }
};

// ============================================================================
// Module
// ============================================================================
struct AMB : Module
{
    enum ParamId
    {
        KEY_PARAM,
        SCALE_PARAM,
        TEMPO_PARAM,
        PAD_LEVEL_PARAM,
        BELL_LEVEL_PARAM,
        BASS_LEVEL_PARAM,
        PULSE_LEVEL_PARAM,
        ECHO_MIX_PARAM,
        ECHO_FBK_PARAM,
        REVERB_MIX_PARAM,
        REVERB_DECAY_PARAM,
        CHORD_INT_PARAM,
        PARAMS_LEN
    };
    enum InputId  { INPUTS_LEN };
    enum OutputId { OUT_L_OUTPUT, OUT_R_OUTPUT, OUTPUTS_LEN };
    enum LightId  { LIGHTS_LEN };

    AmbEngine engine;
    int lastKey   = -1;
    int lastScale = -1;

    AMB()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam<KeyParamQuantity>(KEY_PARAM,   0.f, 11.f, 0.f, "Key (C–B)");
        configParam<ScaleParamQuantity>(SCALE_PARAM, 0.f, (float)(AMB_NSCALES - 1), 0.f, "Scale");
        configParam<TempoParamQuantity>(TEMPO_PARAM, 40.f, 180.f, 120.f, "Tempo");

        configParam(PAD_LEVEL_PARAM,   0.f, 1.f, 0.55f, "Pad level",   "%", 0.f, 100.f);
        configParam(BELL_LEVEL_PARAM,  0.f, 1.f, 0.55f, "Bell level",  "%", 0.f, 100.f);
        configParam(BASS_LEVEL_PARAM,  0.f, 1.f, 0.55f, "Bass level",  "%", 0.f, 100.f);
        configParam(PULSE_LEVEL_PARAM, 0.f, 1.f, 0.50f, "Pulse level", "%", 0.f, 100.f);

        configParam(ECHO_MIX_PARAM,   0.f, 1.f,  0.40f, "Echo mix",      "%", 0.f, 100.f);
        configParam(ECHO_FBK_PARAM,   0.f, 0.9f, 0.55f, "Echo feedback", "%", 0.f, 100.f);
        configParam(REVERB_MIX_PARAM, 0.f, 1.f,  0.55f, "Reverb mix",    "%", 0.f, 100.f);
        configParam(REVERB_DECAY_PARAM, 0.f, 1.f, 0.55f, "Reverb decay", "%", 0.f, 100.f);

        configParam<ChordIntervalParamQuantity>(CHORD_INT_PARAM, 4.f, 60.f, 24.f, "Chord change interval");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");
    }

    void process(const ProcessArgs &args) override
    {
        int newKey   = clamp((int)std::round(params[KEY_PARAM].getValue()), 0, 11);
        int newScale = clamp((int)std::round(params[SCALE_PARAM].getValue()), 0, AMB_NSCALES - 1);

        if (newKey != lastKey || newScale != lastScale)
        {
            engine.key      = newKey;
            engine.scaleIdx = newScale;
            engine.pickNewChord();
            lastKey   = newKey;
            lastScale = newScale;
        }

        engine.tempoBpm      = params[TEMPO_PARAM].getValue();
        engine.padLevel      = params[PAD_LEVEL_PARAM].getValue();
        engine.bellLevel     = params[BELL_LEVEL_PARAM].getValue();
        engine.bassLevel     = params[BASS_LEVEL_PARAM].getValue();
        engine.pulseLevel    = params[PULSE_LEVEL_PARAM].getValue();
        engine.echoMix       = params[ECHO_MIX_PARAM].getValue();
        engine.echoFbk       = params[ECHO_FBK_PARAM].getValue();
        engine.reverbMix     = params[REVERB_MIX_PARAM].getValue();
        engine.reverbDecay   = params[REVERB_DECAY_PARAM].getValue();
        engine.chordInterval = params[CHORD_INT_PARAM].getValue();

        engine.process(args.sampleRate, args.sampleTime);

        outputs[OUT_L_OUTPUT].setVoltage(engine.outL * 5.f);
        outputs[OUT_R_OUTPUT].setVoltage(engine.outR * 5.f);
    }
};

// ============================================================================
// UI (agmm style)
// ============================================================================
namespace
{
    namespace ag = agmm;

    struct AMBPanel : TransparentWidget
    {
        AMBPanel() { box.size = Vec(AMB_W, AMB_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, AMB_W, AMB_H, "AMB");
            ag::bezel(vg, AMB_DX, AMB_DY, AMB_DW, AMB_DH);

            ag::cell(vg, 68.f,  AMB_KBIG_CELL_TOP, 90.f, AMB_KBIG_CELL_H, "KEY");
            ag::cell(vg, 172.f, AMB_KBIG_CELL_TOP, 90.f, AMB_KBIG_CELL_H, "SCALE");

            const char *r1[4] = { "TEMPO", "PAD", "BELL", "BASS" };
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, AMB_KCOL[i], AMB_KS1_CELL_TOP, AMB_CELL_W, AMB_KS1_CELL_H, r1[i]);

            const char *r2[4] = { "PULSE", "ECHO", "FBK", "REV MIX" };
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, AMB_KCOL[i], AMB_KS2_CELL_TOP, AMB_CELL_W, AMB_KS2_CELL_H, r2[i]);

            ag::cell(vg, AMB_KCOL[0], AMB_KS3_CELL_TOP, AMB_CELL_W, AMB_KS3_CELL_H, "REV DEC");
            ag::cell(vg, AMB_KCOL[1], AMB_KS3_CELL_TOP, AMB_CELL_W, AMB_KS3_CELL_H, "CHORD INT");

            ag::cell(vg, 68.f,  AMB_IO_CELL_TOP, 60.f, AMB_IO_CELL_H, "OUT L", true, 7.f);
            ag::cell(vg, 172.f, AMB_IO_CELL_TOP, 60.f, AMB_IO_CELL_H, "OUT R", true, 7.f);

            Widget::draw(args);
        }
    };

    struct AMBDisplay : TransparentWidget
    {
        AMB *module = nullptr;
        AMBDisplay(AMB *m) : module(m)
        {
            box.pos  = Vec(AMB_DX, AMB_DY);
            box.size = Vec(AMB_DW, AMB_DH);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float X0 = AMB_TX0, Y0 = AMB_TY0, TW = AMB_TW, TH = AMB_TH;

            ag::screen(vg, 0.f, 0.f, AMB_DW, AMB_DH);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 6.f, 13.5f);
            nvgLineTo(vg, AMB_DW - 6.f, 13.5f);
            nvgMoveTo(vg, 6.f, 77.5f);
            nvgLineTo(vg, AMB_DW - 6.f, 77.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, X0 - 1.f, Y0 - 1.f, TW + 2.f, TH + 2.f);

            // Octave gridlines at C2, C3, C4, C5, C6
            nvgBeginPath(vg);
            for (int s = AMB_PITCH_MIN; s <= AMB_PITCH_MAX; s += 12)
            {
                float y = Y0 + TH - (float)(s - AMB_PITCH_MIN)
                                    / (float)(AMB_PITCH_MAX - AMB_PITCH_MIN) * TH;
                nvgMoveTo(vg, X0, y + 0.5f);
                nvgLineTo(vg, X0 + TW, y + 0.5f);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 20));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            const char *tl[4] = { "15S", "10", "5", "0" };
            for (int k = 0; k < 4; ++k)
                ag::text(vg, X0 + TW - k * (TW / 3.f), Y0 + TH + 12.f,
                         tl[k], 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            ag::text(vg, X0 + TW + 8.f,  Y0 + TH + 12.f, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, X0 + TW + 14.f, Y0 + TH + 12.f, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1) { Widget::drawLayer(args, layer); return; }
            NVGcontext *vg = args.vg;
            const float X0 = AMB_TX0, Y0 = AMB_TY0, TW = AMB_TW, TH = AMB_TH;
            const float W  = AMB_DW, H  = AMB_DH;

            std::vector<AmbEngine::NoteEvent> evts;
            if (module)
            {
                for (auto &e : module->engine.history)
                    if (e.age <= AMB_HIST_WIN) evts.push_back(e);
            }
            else
            {
                for (int i = 0; i < 16; i++)
                    evts.push_back({ (float)i, 12.f + (i % 6) * 3.f, i % 4 });
            }

            auto pitchY = [&](float p) {
                float n = (p - (float)AMB_PITCH_MIN) / (float)(AMB_PITCH_MAX - AMB_PITCH_MIN);
                n = clamp(n, 0.f, 1.f);
                return Y0 + TH - n * TH;
            };
            auto ageX = [&](float a) {
                return X0 + TW - clamp(a / AMB_HIST_WIN, 0.f, 1.f) * TW;
            };

            NVGcolor colours[4] = {
                ag::withA(ag::violet(), 0.85f),
                ag::withA(ag::accent(), 0.85f),
                nvgRGB(0xff, 0xc0, 0x60),
                nvgRGB(0x90, 0x90, 0x90)
            };

            for (auto &e : evts)
            {
                float x = ageX(e.age);
                float y = pitchY(e.pitch);
                float a = clamp(1.f - e.age / AMB_HIST_WIN, 0.f, 1.f);
                NVGcolor c = colours[e.layer % 4];
                c.a *= a;
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, 2.5f);
                nvgFillColor(vg, c);
                nvgFill(vg);
            }

            int k = 0, s = 0;
            float chord = 12.f;
            if (module)
            {
                k = module->engine.key;
                s = module->engine.scaleIdx;
                chord = module->engine.curChordRoot;
            }
            k = clamp(k, 0, 11);
            s = clamp(s, 0, AMB_NSCALES - 1);

            char buf[64];
            snprintf(buf, sizeof(buf), "%s %s", AMB_NOTE_NAMES[k], AMB_SCALE_NAMES[s]);
            ag::text(vg, 6.f, 8.f, buf, 7.f, ag::withA(ag::accent(), 0.9f), ag::ALIGN_L, 0.5f);

            // Display oct: 130.81 Hz base, pitch 0 = C3 → oct = 3 + pitch/12
            int nearest = clamp((int)std::round(chord), AMB_PITCH_MIN, AMB_PITCH_MAX);
            int oct = 3 + (nearest < 0 ? (nearest - 11) / 12 : nearest / 12);
            int nn  = ((nearest % 12) + 12) % 12;
            snprintf(buf, sizeof(buf), "%s%d", AMB_NOTE_NAMES[nn], oct);
            ag::text(vg, W - 6.f, 9.f, buf, 9.f, ag::withA(ag::bright(), 0.9f), ag::ALIGN_R, 0.5f);

            ag::text(vg, 6.f,  H - 6.f, "PAD",   7.f, colours[0], ag::ALIGN_L, 0.5f);
            ag::text(vg, 30.f, H - 6.f, "BELL",  7.f, colours[1], ag::ALIGN_L, 0.5f);
            ag::text(vg, 56.f, H - 6.f, "BASS",  7.f, colours[2], ag::ALIGN_L, 0.5f);
            ag::text(vg, 82.f, H - 6.f, "PULSE", 7.f, colours[3], ag::ALIGN_L, 0.5f);

            float mL = module ? module->engine.meterL : 0.f;
            float mR = module ? module->engine.meterR : 0.f;
            ag::meter(vg, X0 + TW + 6.f,  Y0, 4.f, TH, mL / 5.f);
            ag::meter(vg, X0 + TW + 12.f, Y0, 4.f, TH, mR / 5.f);

            Widget::drawLayer(args, layer);
        }
    };
} // namespace

// ============================================================================
// Widget
// ============================================================================
struct AMBWidget : ModuleWidget
{
    AMBWidget(AMB *module)
    {
        setModule(module);
        box.size = Vec(AMB_W, AMB_H);

        addChild(new AMBPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new AMBDisplay(module));

        addParam(createParamCentered<RoundBigBlackKnob>(Vec(68.f,  AMB_KBIG_CY), module, AMB::KEY_PARAM));
        addParam(createParamCentered<RoundBigBlackKnob>(Vec(172.f, AMB_KBIG_CY), module, AMB::SCALE_PARAM));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[0], AMB_KS1_CY), module, AMB::TEMPO_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[1], AMB_KS1_CY), module, AMB::PAD_LEVEL_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[2], AMB_KS1_CY), module, AMB::BELL_LEVEL_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[3], AMB_KS1_CY), module, AMB::BASS_LEVEL_PARAM));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[0], AMB_KS2_CY), module, AMB::PULSE_LEVEL_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[1], AMB_KS2_CY), module, AMB::ECHO_MIX_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[2], AMB_KS2_CY), module, AMB::ECHO_FBK_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[3], AMB_KS2_CY), module, AMB::REVERB_MIX_PARAM));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[0], AMB_KS3_CY), module, AMB::REVERB_DECAY_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(AMB_KCOL[1], AMB_KS3_CY), module, AMB::CHORD_INT_PARAM));

        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(68.f,  AMB_IO_CY), module, AMB::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(172.f, AMB_IO_CY), module, AMB::OUT_R_OUTPUT));
    }
};

Model *modelAMB = createModel<AMB, AMBWidget>("AMB");