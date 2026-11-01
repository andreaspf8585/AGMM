#include "plugin.hpp"
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <algorithm>
#include <cstdint>

// =============================================================================
// RainStorm : layered rain + random, distance-aware thunder + echo + reverb
//
//  RAIN    = pink-noise wash  +  two layers of random droplets (resonant
//            band-passed noise bursts, Poisson timing)  +  fine hiss
//            +  low distant roar.  Slow random "gusts" modulate level,
//            brightness and droplet density, so the rain breathes.
//  THUNDER = up to 2 overlapping voices. Each strike picks a random distance:
//            close  -> sharp crack + crackle, bright rolling rumble, short delay
//            far    -> no crack, slow swell, dark long rumble, longer delay
//            The rumble is low-passed noise whose cutoff follows a sum of
//            random attack/decay "rolls", with slow amplitude trembling.
//            Strikes come randomly (Thunder Rate) and/or from the TRIG input.
// =============================================================================

static const float RS_W = RACK_GRID_WIDTH * 28;   // 420
static const float RS_H = RACK_GRID_HEIGHT;       // 380
static const float RS_DX = 30.f, RS_DY = 46.f, RS_DW = 360.f, RS_DH = 110.f;
static const float RS_R1 = 172.f, RS_R2 = 264.f, RS_CH = 84.f;
static float rsCol(int i) { return 35.f + 50.f * i; }

// The DSP helpers live in an anonymous namespace so their names (Comb, Allpass, ...)
// cannot clash with same-named types in other module files.
namespace {

inline float clampf(float x, float a, float b) {
    return x < a ? a : (x > b ? b : x);
}

inline float sanitize(float x) {
    if (!std::isfinite(x)) return 0.f;
    if (std::fabs(x) < 1e-20f) return 0.f;
    return x;
}

// ------------------------------- random --------------------------------------
struct Rng {
    uint32_t s = 1;
    Rng(uint32_t seed = 1) { reseed(seed); }
    void reseed(uint32_t seed) {
        seed = seed * 2654435761u + 0x9E3779B9u;
        if (!seed) seed = 1;
        s = seed;
        for (int i = 0; i < 4; ++i) nextU();
    }
    inline uint32_t nextU() {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return s;
    }
    inline float uni() { return (float)(nextU() >> 8) * (1.f / 16777216.f); } // [0,1)
    inline float bi()  { return uni() * 2.f - 1.f; }                           // [-1,1)
    inline float range(float a, float b) { return a + (b - a) * uni(); }
};

// ------------------------------- filters -------------------------------------
struct OnePole {
    float z = 0.f;
    inline float process(float x, float a) {
        z += a * (x - z);
        z = sanitize(z);
        return z;
    }
};

// Chamberlin state-variable filter, band-pass output
struct SVF {
    float low = 0.f, band = 0.f;
    inline float bandpass(float x, float f, float q) {
        low += f * band;
        float high = x - low - q * band;
        band += f * high;
        low = sanitize(low);
        band = sanitize(band);
        return band;
    }
};

// Paul Kellet economy pink-noise filter
struct PinkNoise {
    float b0 = 0.f, b1 = 0.f, b2 = 0.f;
    inline float process(float w) {
        b0 = 0.99765f * b0 + w * 0.0990460f;
        b1 = 0.96300f * b1 + w * 0.2965164f;
        b2 = 0.57000f * b2 + w * 1.0526913f;
        return (b0 + b1 + b2 + w * 0.1848f) * 0.3f;
    }
};

// =============================================================================
// RAIN
// =============================================================================
struct RainParams {
    float sr = 48000.f;
    float intensity = 0.f;
    float lvl = 0.f;        // overall level curve
    float dropLvl = 0.f;    // droplet level curve
    float gust = 1.f;       // gust multiplier (~0.2 .. 1.8)
    float washHpA = 0.f, washLpA = 0.f, hissA = 0.f, rumbleA = 0.f;
    float patterRate = 0.f, fineRate = 0.f;       // drops per second
    float patterDecay = 0.99f, fineDecay = 0.99f; // per-sample envelope decay
};

// Random droplets: Poisson-timed noise bursts through a resonant band-pass
// whose centre frequency is re-randomised on every drop.
struct DropLayer {
    Rng rng;
    SVF svf;
    float env = 0.f;
    float f = 0.3f;

    float process(float rate, float sr, float fMin, float fMax,
                  float decayMul, float amp, float q) {
        if (rng.uni() < rate / sr) {
            float a = rng.uni();
            env += amp * (0.25f + 0.75f * a * a);   // many soft drops, few loud
            if (env > 1.5f) env = 1.5f;
            float fc = fMin * std::pow(fMax / fMin, rng.uni());
            fc = std::min(fc, 0.18f * sr);
            f = 2.f * std::sin((float)M_PI * fc / sr);
        }
        float x = rng.bi() * env;
        env *= decayMul;
        if (env < 1e-6f) env = 0.f;
        return svf.bandpass(x, f, q) * q;
    }
};

struct RainChannel {
    Rng wRng, hRng;
    PinkNoise pink;
    OnePole hpLp, wl1, wl2, hissLp, rumLp;
    DropLayer patter, fine;

    RainChannel(uint32_t seed = 1) {
        wRng.reseed(seed * 7u + 1u);
        hRng.reseed(seed * 13u + 3u);
        patter.rng.reseed(seed * 17u + 5u);
        fine.rng.reseed(seed * 19u + 7u);
    }

    float process(const RainParams& p) {
        float w  = wRng.bi();
        float pk = pink.process(w);

        // wash: pink noise, high-passed ~380 Hz, low-passed by brightness
        float hp   = pk - hpLp.process(pk, p.washHpA);
        float wash = wl2.process(wl1.process(hp, p.washLpA), p.washLpA);

        // fine hiss from dense tiny drops
        float w2   = hRng.bi();
        float hiss = w2 - hissLp.process(w2, p.hissA);

        // distant low roar
        float rumble = rumLp.process(pk, p.rumbleA);

        // droplets
        float d1 = patter.process(p.patterRate, p.sr, 1400.f, 5000.f, p.patterDecay, 1.0f, 0.35f);
        float d2 = fine.process(p.fineRate,   p.sr, 3500.f, 8500.f, p.fineDecay,   0.6f, 0.50f);
        float drops = (d1 * 1.2f + d2 * 0.8f) * p.dropLvl;

        float out = wash   * 3.0f  * p.lvl * p.gust
                  + hiss   * 0.10f * p.lvl * p.lvl * p.gust
                  + rumble * 0.9f  * p.lvl * (0.7f + 0.3f * p.gust)
                  + drops;
        return out;
    }
};

// =============================================================================
// THUNDER
// =============================================================================
struct ThunderVoice {
    struct Peak {
        float startT = 0.f;
        float a = 1.f, attMul = 0.99f;   // attack  : (1 - a)
        float d = 1.f, decMul = 0.999f;  // decay   : d
        float amp = 0.f;
    };
    static const int MAXP = 14;
    Peak peaks[MAXP];
    int   numPeaks = 0;

    bool  active = false;
    float delaySamples = 0.f;   // flash-to-bang delay
    float t = 0.f, lastStart = 0.f;
    float amp = 1.f, bright = 1.f, pan = 0.f;

    float crackEnv = 0.f, crackDecMul = 0.999f, crackAmp = 0.f, crackF = 0.2f;

    // Display data
    float level = 0.f;          // current rumble level (0..1)
    float dist = 0.f;           // 0 = overhead, 1 = far away

    Rng rng;
    OnePole lpL1, lpL2, lpR1, lpR2, roll;
    SVF crackSvf;

    void start(float distance, float sr, float delaySec, float panDir) {
        active = true;
        t = 0.f;
        delaySamples = delaySec * sr;
        pan = panDir;
        dist = distance;
        amp = 1.f - 0.55f * distance;
        bright = 1.f - 0.7f * distance;

        float D = rng.range(3.f, 6.f) + distance * rng.range(3.f, 8.f);
        numPeaks = 6 + (int)(rng.uni() * 8.f);
        if (numPeaks > MAXP) numPeaks = MAXP;

        lastStart = 0.f;
        for (int i = 0; i < numPeaks; ++i) {
            Peak& p = peaks[i];
            float st = (i == 0) ? 0.f : D * std::pow(rng.uni(), 1.5f) * 0.85f;
            p.startT = st;
            p.a = 1.f;
            p.d = 1.f;
            float att = (i == 0) ? (0.02f + 0.5f * distance)
                                 : (rng.range(0.04f, 0.25f) + 0.6f * distance);
            float dec = rng.range(0.5f, 1.6f) + 1.2f * distance;
            p.attMul = std::exp(-1.f / (att * sr));
            p.decMul = std::exp(-1.f / (dec * sr));
            p.amp = (i == 0) ? 1.f
                             : (1.f - 0.75f * (st / D)) * rng.range(0.3f, 0.9f);
            if (st > lastStart) lastStart = st;
        }

        // crack only for close strikes
        crackAmp = clampf(1.3f * (1.f - 1.6f * distance), 0.f, 1.f);
        crackEnv = crackAmp;
        crackDecMul = std::exp(-1.f / (rng.range(0.02f, 0.06f) * sr));
        crackF = 2.f * std::sin((float)M_PI * rng.range(900.f, 2200.f) / sr);

        lpL1.z = lpL2.z = lpR1.z = lpR2.z = roll.z = 0.f;
        crackSvf.low = crackSvf.band = 0.f;
    }

    void process(float sr, float sampleTime, float rollA, float rollGain,
                 float& outL, float& outR) {
        outL = outR = 0.f;
        level = 0.f;
        if (!active) return;
        if (delaySamples > 0.f) { delaySamples -= 1.f; return; }

        // sum of random attack/decay rolls
        float env = 0.f;
        for (int i = 0; i < numPeaks; ++i) {
            Peak& p = peaks[i];
            if (t >= p.startT) {
                p.a *= p.attMul;
                p.d *= p.decMul;
                if (p.d < 1e-6f) p.d = 0.f;
                env += (1.f - p.a) * p.d * p.amp;
            }
        }
        t += sampleTime;
        env = env / (1.f + 0.6f * env);
        level = std::min(env * amp, 1.f);

        // slow trembling of the rumble level (rolling effect)
        float r  = roll.process(rng.bi(), rollA) * rollGain;
        float rm = clampf(1.f + r, 0.35f, 1.8f);

        // rumble: low-passed noise, cutoff follows the envelope
        float fc = 45.f + 330.f * bright * std::sqrt(std::max(env, 0.f));
        float x  = 2.f * (float)M_PI * fc / sr;
        float a  = x / (1.f + x);
        float comp = 1.2f / std::sqrt(a / (2.f - a));   // keep level independent of cutoff

        float c  = rng.bi();
        float nl = 0.6f * c + 0.8f * rng.bi();
        float nr = 0.6f * c + 0.8f * rng.bi();
        float rl = lpL2.process(lpL1.process(nl, a), a);
        float rr = lpR2.process(lpR1.process(nr, a), a);
        float rumbleAmp = env * rm;

        // crack + crackle (close strikes)
        float crack = 0.f;
        if (crackAmp > 0.001f) {
            if (t < 0.7f && rng.uni() < 450.f * std::exp(-t / 0.18f) * sampleTime)
                crackEnv += crackAmp * 0.5f * rng.uni();
            float n = rng.bi() * crackEnv;
            crackEnv *= crackDecMul;
            if (crackEnv < 1e-6f) crackEnv = 0.f;
            crack = crackSvf.bandpass(n, crackF, 0.7f) * 1.0f;
        }

        float L = rl * comp * rumbleAmp + crack * 0.8f;
        float R = rr * comp * rumbleAmp + crack * 0.8f;

        // pan (constant power, limited range)
        float ang = (pan * 0.6f * 0.5f + 0.5f) * (float)M_PI_2;
        outL = L * std::cos(ang) * 1.414f * amp;
        outR = R * std::sin(ang) * 1.414f * amp;

        if ((t > lastStart && env < 1e-4f && crackEnv < 1e-4f) || t > 40.f)
            active = false;
    }
};

// =============================================================================
// ECHO (stereo, smoothed time, darkened feedback)
// =============================================================================
struct EchoDelay {
    std::vector<float> bufL, bufR;
    int   size = 0, w = 0;
    float smTime = 0.3f, lpL = 0.f, lpR = 0.f;

    void init(float sr) {
        size = (int)(sr * 1.2f) + 8;
        bufL.assign(size, 0.f);
        bufR.assign(size, 0.f);
        w = 0; lpL = lpR = 0.f;
    }
    inline float readInterp(const std::vector<float>& b, float d) const {
        float pos = (float)w - d;
        while (pos < 0.f) pos += (float)size;
        if (pos >= (float)size) pos -= (float)size;
        int i0 = (int)pos;
        float fr = pos - (float)i0;
        int i1 = i0 + 1; if (i1 >= size) i1 -= size;
        return b[i0] * (1.f - fr) + b[i1] * fr;
    }
    void process(float inL, float inR, float timeSec, float fb, float sr,
                 float& dL, float& dR) {
        smTime += (timeSec - smTime) * 0.0004f;   // glide, no clicks
        float dl = clampf(smTime * sr,         2.f, (float)size - 4.f);
        float dr = clampf(smTime * 1.07f * sr, 2.f, (float)size - 4.f);
        dL = readInterp(bufL, dl);
        dR = readInterp(bufR, dr);
        lpL += 0.4f * (dL - lpL);   // each repeat gets darker
        lpR += 0.4f * (dR - lpR);
        lpL = sanitize(lpL); lpR = sanitize(lpR);
        bufL[w] = sanitize(2.f * std::tanh((inL + lpL * fb) * 0.5f));
        bufR[w] = sanitize(2.f * std::tanh((inR + lpR * fb) * 0.5f));
        if (++w >= size) w = 0;
    }
};

// =============================================================================
// REVERB (Freeverb topology: 8 damped combs + 4 allpasses per channel)
// =============================================================================
struct Comb {
    std::vector<float> buf;
    int idx = 0;
    float store = 0.f;
    void init(int n) { buf.assign(std::max(n, 1), 0.f); idx = 0; store = 0.f; }
    inline float process(float in, float fb, float damp) {
        float out = buf[idx];
        store = sanitize(out * (1.f - damp) + store * damp);
        buf[idx] = sanitize(in + store * fb);
        if (++idx >= (int)buf.size()) idx = 0;
        return out;
    }
};

struct Allpass {
    std::vector<float> buf;
    int idx = 0;
    void init(int n) { buf.assign(std::max(n, 1), 0.f); idx = 0; }
    inline float process(float in) {
        float b = buf[idx];
        float out = -in + b;
        buf[idx] = sanitize(in + b * 0.5f);
        if (++idx >= (int)buf.size()) idx = 0;
        return out;
    }
};

struct StormReverb {
    static const int NC = 8, NA = 4;
    Comb cL[NC], cR[NC];
    Allpass aL[NA], aR[NA];
    float fbSm = 0.85f;

    void init(float sr) {
        static const int combT[NC] = {1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};
        static const int apT[NA]   = {556, 441, 341, 225};
        float s = sr / 44100.f;
        for (int i = 0; i < NC; ++i) {
            cL[i].init((int)(combT[i] * s));
            cR[i].init((int)((combT[i] + 23) * s));
        }
        for (int i = 0; i < NA; ++i) {
            aL[i].init((int)(apT[i] * s));
            aR[i].init((int)((apT[i] + 23) * s));
        }
    }

    void process(float inL, float inR, float size, float& oL, float& oR) {
        float target = 0.72f + 0.26f * size;     // feedback < 0.98 : always decays
        fbSm += (target - fbSm) * 0.001f;
        const float damp = 0.35f;
        float xl = (inL * 0.7f + inR * 0.3f) * 0.015f;
        float xr = (inR * 0.7f + inL * 0.3f) * 0.015f;
        float sl = 0.f, sr_ = 0.f;
        for (int i = 0; i < NC; ++i) {
            sl  += cL[i].process(xl, fbSm, damp);
            sr_ += cR[i].process(xr, fbSm, damp);
        }
        for (int i = 0; i < NA; ++i) {
            sl  = aL[i].process(sl);
            sr_ = aR[i].process(sr_);
        }
        oL = sl * 3.f;
        oR = sr_ * 3.f;
    }
};

} // namespace

// =============================================================================
// RainStorm module
// =============================================================================
struct RainStorm : Module {
    // Existing IDs keep their positions (PARALLEL/SERIES slots are now REVERB SIZE/MIX)
    enum ParamId {
        INTENSITY_PARAM,
        OUTPUT_GAIN_PARAM,
        ECHO_TIME_PARAM,
        ECHO_FEEDBACK_PARAM,
        ECHO_MIX_PARAM,
        THUNDER_INTENSITY_PARAM,
        REVERB_SIZE_PARAM,
        REVERB_MIX_PARAM,
        WIDTH_PARAM,
        THUNDER_RATE_PARAM,
        GUST_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        TRIG_INPUT,
        INTENSITY_CV,
        ECHO_TIME_CV,
        ECHO_FEEDBACK_CV,
        ECHO_MIX_CV,
        OUTPUT_GAIN_CV,
        THUNDER_INTENSITY_CV,
        THUNDER_RATE_CV,
        INPUTS_LEN
    };
    enum OutputId {
        L_OUTPUT,
        R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        THUNDER_LIGHT,
        LIGHTS_LEN
    };

    // Rain
    RainChannel rainL{11u}, rainR{23u};
    float gustTarget = 0.5f, gustVal = 0.5f;
    int   gustTimer = 0;
    Rng   gustRng;

    // Thunder
    ThunderVoice voices[2];
    Rng   thunderRng;
    rack::dsp::SchmittTrigger thunderTrigger;
    float timeSinceThunder = 100.f;
    float flash = 0.f, flickerTimer = -1.f;

    // FX
    EchoDelay echo;
    StormReverb reverb;

    // Sample-rate dependent constants
    float cachedSr = -1.f;
    float patterDecay = 0.99f, fineDecay = 0.99f, flashDecay = 0.999f;
    float gustA = 0.0001f, rollA = 0.001f, rollGain = 10.f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float dispIntensity = 0.5f, dispGust = 1.f, dispRate = 0.f;
    float meterL = 0.f, meterR = 0.f;
    float boltX = 0.5f;

    RainStorm() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(INTENSITY_PARAM,         0.f,   1.f,   0.5f,  "Rain intensity", "%", 0.f, 100.f);
        configParam(OUTPUT_GAIN_PARAM,       0.f,   2.f,   1.f,   "Output gain", "%", 0.f, 100.f);
        configParam(ECHO_TIME_PARAM,         0.01f, 0.8f,  0.35f, "Echo time", " s");
        configParam(ECHO_FEEDBACK_PARAM,     0.f,   0.9f,  0.45f, "Echo feedback", "%", 0.f, 100.f);
        configParam(ECHO_MIX_PARAM,          0.f,   1.f,   0.15f, "Echo mix", "%", 0.f, 100.f);
        configParam(THUNDER_INTENSITY_PARAM, 0.f,   1.5f,  1.f,   "Thunder intensity", "%", 0.f, 100.f);
        configParam(REVERB_SIZE_PARAM,       0.f,   1.f,   0.6f,  "Reverb size", "%", 0.f, 100.f);
        configParam(REVERB_MIX_PARAM,        0.f,   1.f,   0.35f, "Reverb mix", "%", 0.f, 100.f);
        configParam(WIDTH_PARAM,             0.f,   1.f,   0.8f,  "Stereo width", "%", 0.f, 100.f);
        configParam(THUNDER_RATE_PARAM,      0.f,   1.f,   0.2f,  "Thunder rate", " /min", 0.f, 10.f);
        configParam(GUST_PARAM,              0.f,   1.f,   0.5f,  "Gusts", "%", 0.f, 100.f);

        configInput(TRIG_INPUT,            "Thunder trigger");
        configInput(INTENSITY_CV,          "Intensity CV (5 V = 100%)");
        configInput(ECHO_TIME_CV,          "Echo time CV (5 V = full range)");
        configInput(ECHO_FEEDBACK_CV,      "Echo feedback CV (5 V = full range)");
        configInput(ECHO_MIX_CV,           "Echo mix CV (5 V = 100%)");
        configInput(OUTPUT_GAIN_CV,        "Output gain CV (5 V = 200%)");
        configInput(THUNDER_INTENSITY_CV,  "Thunder intensity CV (5 V = 150%)");
        configInput(THUNDER_RATE_CV,       "Thunder rate CV (5 V = 10 /min)");

        configOutput(L_OUTPUT, "Left");
        configOutput(R_OUTPUT, "Right");
        configLight(THUNDER_LIGHT, "Lightning");

        std::random_device rd;
        thunderRng.reseed(rd());
        gustRng.reseed(rd());
        voices[0].rng.reseed(rd());
        voices[1].rng.reseed(rd());

        updateSampleRate(48000.f);
    }

    void onReset() override {
        cachedSr = -1.f;   // forces buffer re-init on next process()
        voices[0].active = voices[1].active = false;
        flash = 0.f; flickerTimer = -1.f;
    }

    void updateSampleRate(float sr) {
        cachedSr = sr;
        echo.init(sr);
        reverb.init(sr);
        patterDecay = std::exp(-1.f / (0.0060f * sr));
        fineDecay   = std::exp(-1.f / (0.0025f * sr));
        flashDecay  = std::exp(-1.f / (0.18f * sr));
        float xg = 2.f * (float)M_PI * 0.35f / sr;          // gust smoothing, 0.35 Hz
        gustA = xg / (1.f + xg);
        float xr = 2.f * (float)M_PI * 8.f / sr;            // rumble trembling, 8 Hz
        rollA = xr / (1.f + xr);
        rollGain = 0.30f / (0.577f * std::sqrt(rollA / (2.f - rollA)));
    }

    void triggerThunder(float sr, bool randomDelay) {
        float d = thunderRng.uni();                          // 0 = overhead, 1 = far away
        float delaySec = randomDelay ? (0.05f + d * 3.0f) : 0.f;

        int idx = -1;
        for (int i = 0; i < 2; ++i) if (!voices[i].active) { idx = i; break; }
        if (idx < 0) idx = (voices[0].t > voices[1].t) ? 0 : 1;   // steal the oldest

        voices[idx].start(d, sr, delaySec, thunderRng.range(-1.f, 1.f));
        flash = 1.f;
        flickerTimer = 0.09f;
        timeSinceThunder = 0.f;
        boltX = 0.15f + 0.7f * thunderRng.uni();
    }

    void process(const ProcessArgs& args) override {
        const float sr = args.sampleRate;
        if (sr != cachedSr) updateSampleRate(sr);

        // ----- parameters + CV (CV scaled to each knob's range, 5 V = full) -----
        float intensity = clampf(params[INTENSITY_PARAM].getValue()
                          + inputs[INTENSITY_CV].getVoltage() * (1.f / 5.f), 0.f, 1.f);
        float gain      = clampf(params[OUTPUT_GAIN_PARAM].getValue()
                          + inputs[OUTPUT_GAIN_CV].getVoltage() * (2.f / 5.f), 0.f, 2.f);
        float echoTime  = clampf(params[ECHO_TIME_PARAM].getValue()
                          + inputs[ECHO_TIME_CV].getVoltage() * (0.79f / 5.f), 0.01f, 0.8f);
        float echoFb    = clampf(params[ECHO_FEEDBACK_PARAM].getValue()
                          + inputs[ECHO_FEEDBACK_CV].getVoltage() * (0.9f / 5.f), 0.f, 0.9f);
        float echoMix   = clampf(params[ECHO_MIX_PARAM].getValue()
                          + inputs[ECHO_MIX_CV].getVoltage() * (1.f / 5.f), 0.f, 1.f);
        float thunderInt = clampf(params[THUNDER_INTENSITY_PARAM].getValue()
                          + inputs[THUNDER_INTENSITY_CV].getVoltage() * (1.5f / 5.f), 0.f, 1.5f);
        float thunderRate = clampf(params[THUNDER_RATE_PARAM].getValue()
                          + inputs[THUNDER_RATE_CV].getVoltage() * (1.f / 5.f), 0.f, 1.f) * 10.f;
        float revSize   = params[REVERB_SIZE_PARAM].getValue();
        float revMix    = params[REVERB_MIX_PARAM].getValue();
        float width     = params[WIDTH_PARAM].getValue();
        float gustDepth = params[GUST_PARAM].getValue();

        // ----- gusts: slow random wander -----
        if (--gustTimer <= 0) {
            gustTarget = gustRng.uni();
            gustTimer = (int)(gustRng.range(1.5f, 6.f) * sr);
        }
        gustVal += gustA * (gustTarget - gustVal);
        float gustMod = 1.f + gustDepth * (gustVal - 0.5f) * 1.6f;   // ~0.2 .. 1.8

        dispIntensity = intensity;
        dispGust = gustMod;
        dispRate = thunderRate;

        // ----- rain -----
        RainParams rp;
        rp.sr = sr;
        rp.intensity = intensity;
        rp.lvl = intensity * (0.3f + 0.7f * intensity);
        rp.dropLvl = clampf(intensity * 4.f, 0.f, 1.f) * (0.5f + 0.5f * intensity);
        rp.gust = gustMod;
        float washF = (2200.f + 5000.f * intensity) * (0.8f + 0.4f * gustVal);
        auto coef = [&](float hz) { float x = 2.f * (float)M_PI * hz / sr; return x / (1.f + x); };
        rp.washLpA = coef(washF);
        rp.washHpA = coef(380.f);
        rp.hissA   = coef(4500.f);
        rp.rumbleA = coef(160.f);
        float I2 = intensity * intensity;
        rp.patterRate  = (10.f + 220.f * I2) * gustMod;
        rp.fineRate    = (60.f + 2200.f * I2) * gustMod;
        rp.patterDecay = patterDecay;
        rp.fineDecay   = fineDecay;

        float rL = rainL.process(rp);
        float rR = rainR.process(rp);

        // stereo width (mid/side)
        float mid  = 0.5f * (rL + rR);
        float side = 0.5f * (rL - rR) * (width * 1.4f);
        rL = mid + side;
        rR = mid - side;

        // ----- thunder: manual trigger + random strikes -----
        timeSinceThunder += args.sampleTime;
        if (thunderTrigger.process(inputs[TRIG_INPUT].getVoltage(), 0.1f, 2.0f)) {
            triggerThunder(sr, false);                 // manual: sound starts immediately
        } else if (thunderRate > 0.001f && timeSinceThunder > 2.5f &&
                   thunderRng.uni() < (thunderRate / 60.f) * args.sampleTime) {
            triggerThunder(sr, true);                  // random: flash first, bang after delay
        }

        float tL = 0.f, tR = 0.f;
        for (int i = 0; i < 2; ++i) {
            float l, r;
            voices[i].process(sr, args.sampleTime, rollA, rollGain, l, r);
            tL += l; tR += r;
        }
        tL *= thunderInt * 0.9f;
        tR *= thunderInt * 0.9f;

        // lightning LED (double flicker)
        if (flickerTimer >= 0.f) {
            flickerTimer -= args.sampleTime;
            if (flickerTimer < 0.f) flash = std::max(flash, 0.8f);
        }
        flash *= flashDecay;
        lights[THUNDER_LIGHT].setBrightness(flash);

        float mainL = rL + tL;
        float mainR = rR + tR;

        // ----- echo -----
        float dL, dR;
        echo.process(mainL, mainR, echoTime, echoFb, sr, dL, dR);
        float eL = mainL + dL * echoMix;
        float eR = mainR + dR * echoMix;

        // ----- reverb -----
        float wL, wR;
        reverb.process(eL, eR, revSize, wL, wR);
        float outL = eL * (1.f - 0.5f * revMix) + wL * revMix;
        float outR = eR * (1.f - 0.5f * revMix) + wR * revMix;

        // ----- output -----
        outL = std::tanh(outL * gain);
        outR = std::tanh(outR * gain);
        float vL = sanitize(outL) * 5.f, vR = sanitize(outR) * 5.f;
        outputs[L_OUTPUT].setVoltage(vL);
        outputs[R_OUTPUT].setVoltage(vR);

        float md = 1.f - 5.f * args.sampleTime;
        meterL = std::max(std::fabs(vL), meterL * md);
        meterR = std::max(std::fabs(vR), meterR * md);
    }
};

// =============================================================================
// Panel + display
// =============================================================================
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colViolet()   { return nvgRGB(0xb0, 0x5a, 0xff); }
NVGcolor withAlpha(NVGcolor c, float a) { c.a = a; return c; }

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

template <typename TBase = GrayModuleLightWidget>
struct TRSCyanLight : TBase {
    TRSCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using RSCyanLight = TRSCyanLight<>;

void drawLogo(NVGcontext* vg, float cx, float cy) {
    const float w = 64.f, h = 18.f;
    float x = cx - w / 2.f, y = cy - h / 2.f;
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 7.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h, nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
    nvgFill(vg);
    nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
    nvgStrokeWidth(vg, 1.2f);
    nvgStroke(vg);
    const float s = 2.2f;
    float tx = x + 7.f, ty = y + h / 2.f - 3.f;
    nvgBeginPath(vg);
    nvgRect(vg, tx + s, ty, s, s);
    nvgRect(vg, tx, ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);
    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

void drawPanelBase(NVGcontext* vg, float W, float H, const char* title) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
    nvgFill(vg);
    nvgStrokeWidth(vg, 1.f);
    for (int y = 0; y < (int) H; y += 3) {
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0, y + 0.5f);
        nvgLineTo(vg, W, y + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
        nvgStroke(vg);
    }
    nvgBeginPath(vg);
    nvgRect(vg, 0.5f, 0.5f, W - 1.f, H - 1.f);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
    nvgStroke(vg);
    drawLogo(vg, W / 2.f, 13.f);
    drawText(vg, W / 2.f, 31.f, title, 13.f, colText(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);
}

void drawBezel(NVGcontext* vg, float x, float y, float w, float h) {
    float bx = x - 5.f, by = y - 5.f, bw = w + 10.f, bh = h + 10.f;
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh, nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
    nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
    nvgFill(vg);
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

void label(NVGcontext* vg, float x, float y, const char* t) {
    drawText(vg, x, y, t, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
}

void screenBase(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 20.f, W * 0.75f, nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
    nvgFill(vg);
}

void screenFinish(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    for (int y = 0; y < (int) H; y += 3) nvgRect(vg, 0, y + 1, W, 1);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H * 0.3f);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f, nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
    nvgFill(vg);
}

// Knob cell: name on top, knob below, an optional CV jack under the knob
void knobCell(NVGcontext* vg, int col, float top, const char* name) {
    float cx = rsCol(col);
    plate(vg, cx - 23.f, top, 46.f, RS_CH);
    label(vg, cx, top + 9.f, name);
    ring(vg, cx, top + 30.f, 16.f);
}
void jackCell(NVGcontext* vg, int col, float top, const char* name) {
    float cx = rsCol(col);
    plate(vg, cx - 23.f, top, 46.f, RS_CH);
    label(vg, cx, top + 9.f, name);
}

struct RSPanel : TransparentWidget {
    RSPanel() { box.size = Vec(RS_W, RS_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, RS_W, RS_H, "RAINSTORM");
        drawBezel(vg, RS_DX, RS_DY, RS_DW, RS_DH);

        const char* a[8] = {"INTENSE", "GUSTS", "WIDTH", "THUNDER", "T RATE", "ECHO T", "ECHO FB", "ECHO MIX"};
        for (int i = 0; i < 8; ++i) knobCell(vg, i, RS_R1, a[i]);

        knobCell(vg, 1, RS_R2, "GAIN");
        knobCell(vg, 2, RS_R2, "REV SIZE");
        knobCell(vg, 3, RS_R2, "REV MIX");
        jackCell(vg, 4, RS_R2, "TRIG IN");
        jackCell(vg, 5, RS_R2, "OUT L");
        jackCell(vg, 6, RS_R2, "OUT R");
        Widget::draw(args);
    }
};

// Display: animated rain, lightning, thunder voices, meters
struct RSDisplay : TransparentWidget {
    RainStorm* module = nullptr;
    RSDisplay(RainStorm* m) : module(m) {
        box.pos = Vec(RS_DX, RS_DY);
        box.size = Vec(RS_DW, RS_DH);
    }

    static constexpr float SX = 8.f, SY = 16.f, SW = 236.f, SH = 58.f;   // rain scene
    static constexpr float BX = 22.f, BW = 222.f;                        // thunder bars

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        screenBase(vg, RS_DW, RS_DH);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, SX - 1.f, SY - 1.f, SW + 2.f, SH + 2.f, 2.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 120));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        Widget::draw(args);
    }

    static float hsh(float x) {
        float s = std::sin(x) * 43758.5453f;
        return s - std::floor(s);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = RS_DW, H = RS_DH;
        const float t = (float) system::getTime();

        float inten = 0.6f, gust = 1.1f, rate = 2.f, mL = 2.f, mR = 1.8f, flash = 0.f, boltX = 0.5f, tsince = 100.f;
        float echo = 0.15f, rev = 0.35f, wid = 0.8f, gain = 1.f;
        float vlvl[2] = {0.f, 0.f}, vdist[2] = {0.f, 0.f};
        bool vact[2] = {false, false}, vwait[2] = {false, false};
        if (module) {
            inten = module->dispIntensity; gust = module->dispGust; rate = module->dispRate;
            mL = module->meterL; mR = module->meterR; flash = module->flash; boltX = module->boltX;
            tsince = module->timeSinceThunder;
            echo = module->params[RainStorm::ECHO_MIX_PARAM].getValue();
            rev = module->params[RainStorm::REVERB_MIX_PARAM].getValue();
            wid = module->params[RainStorm::WIDTH_PARAM].getValue();
            gain = module->params[RainStorm::OUTPUT_GAIN_PARAM].getValue();
            for (int i = 0; i < 2; ++i) {
                vact[i] = module->voices[i].active;
                vwait[i] = module->voices[i].active && module->voices[i].delaySamples > 0.f;
                vlvl[i] = module->voices[i].level;
                vdist[i] = module->voices[i].dist;
            }
        }

        // ---- rain scene ----
        nvgBeginPath(vg);
        nvgRect(vg, SX, SY, SW, SH);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, SY, 0, SY + SH, nvgRGB(0x0a, 0x14, 0x20), nvgRGB(0x03, 0x06, 0x0b)));
        nvgFill(vg);

        nvgSave(vg);
        nvgScissor(vg, SX, SY, SW, SH);

        // cloud band
        nvgBeginPath(vg);
        nvgRect(vg, SX, SY, SW, 16.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, SY, 0, SY + 16.f, nvgRGBA(70, 84, 104, 110), nvgRGBA(70, 84, 104, 0)));
        nvgFill(vg);

        // lightning flash over the whole scene
        if (flash > 0.01f) {
            nvgBeginPath(vg);
            nvgRect(vg, SX, SY, SW, SH);
            nvgFillColor(vg, nvgRGBA(190, 205, 255, (int) (110.f * std::min(flash, 1.f))));
            nvgFill(vg);
        }

        // rain streaks, two depth layers
        int n = (int) (6 + 54 * std::min(std::max(inten, 0.f), 1.f));
        float slant = (gust - 1.f) * 0.4f;
        for (int layerIdx = 0; layerIdx < 2; ++layerIdx) {
            nvgBeginPath(vg);
            for (int i = 0; i < n; ++i) {
                if ((i & 1) != layerIdx) continue;
                float h1 = hsh(i * 12.9898f + 1.3f), h2 = hsh(i * 78.233f + 4.1f);
                float len = (layerIdx ? 7.f : 4.f) + 5.f * h2;
                float speed = (50.f + 70.f * h1) * (0.6f + 0.8f * inten) * (layerIdx ? 1.3f : 1.f);
                float cycle = SH + len;
                float y = std::fmod(t * speed + h2 * cycle * 7.f, cycle) - len;
                float x = SX + h1 * SW + slant * y;
                nvgMoveTo(vg, x, SY + y);
                nvgLineTo(vg, x + slant * len, SY + y + len);
            }
            nvgStrokeColor(vg, layerIdx ? nvgRGBA(0x8a, 0xee, 0xff, 120) : nvgRGBA(0x3d, 0xe0, 0xff, 55));
            nvgStrokeWidth(vg, layerIdx ? 1.f : 0.7f);
            nvgStroke(vg);
        }

        // ground line
        nvgBeginPath(vg);
        nvgRect(vg, SX, SY + SH - 3.f, SW, 3.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 170));
        nvgFill(vg);

        // lightning bolt
        if (flash > 0.05f) {
            float bx = SX + boltX * SW;
            for (int pass = 0; pass < 2; ++pass) {
                nvgBeginPath(vg);
                float x = bx, y = SY;
                nvgMoveTo(vg, x, y);
                for (int k = 1; k <= 7; ++k) {
                    x = bx + (hsh(boltX * 100.f + k * 3.7f) - 0.5f) * 26.f;
                    y = SY + SH * k / 7.f;
                    nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, pass == 0 ? withAlpha(colViolet(), 0.35f * std::min(flash, 1.f)) : nvgRGBA(245, 240, 255, (int) (255.f * std::min(flash, 1.f))));
                nvgStrokeWidth(vg, pass == 0 ? 5.f : 1.4f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);
            }
        }
        nvgRestore(vg);

        // ---- thunder voices ----
        for (int i = 0; i < 2; ++i) {
            float y = 81.f + i * 8.f;
            char tb[8];
            snprintf(tb, sizeof(tb), "T%d", i + 1);
            drawText(vg, 8.f, y + 2.5f, tb, 6.f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, BX, y, BW, 5.f, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (vwait[i]) {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, BX, y, BW, 5.f, 2.f);
                nvgStrokeColor(vg, withAlpha(colViolet(), 0.5f + 0.4f * std::sin(t * 10.f)));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                drawText(vg, BX + BW - 3.f, y + 2.5f, "WAIT", 5.f, withAlpha(colViolet(), 0.9f), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
            } else if (vact[i]) {
                float w = std::max(std::min(vlvl[i], 1.f) * BW, 2.f);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, BX, y, w, 5.f, 2.f);
                nvgFillPaint(vg, nvgLinearGradient(vg, BX, 0, BX + BW, 0, colViolet(), nvgRGB(0xd8, 0xb0, 0xff)));
                nvgFill(vg);
                const char* dn = vdist[i] < 0.33f ? "NEAR" : (vdist[i] < 0.66f ? "MID" : "FAR");
                drawText(vg, BX + BW - 3.f, y + 2.5f, dn, 5.f, nvgRGBA(255, 255, 255, 150), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
            }
        }

        // ---- vertical bars: intensity, gust, L, R ----
        const float vb[4] = {254.f, 266.f, 280.f, 288.f};
        const char* vn[4] = {"I", "G", "L", "R"};
        float vv[4] = {inten, std::min(gust / 1.8f, 1.f), std::min(mL / 5.f, 1.f), std::min(mR / 5.f, 1.f)};
        for (int k = 0; k < 4; ++k) {
            nvgBeginPath(vg);
            nvgRect(vg, vb[k], 16.f, 6.f, 58.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            float lv = std::min(std::max(vv[k], 0.f), 1.f);
            if (lv > 0.01f) {
                nvgBeginPath(vg);
                nvgRect(vg, vb[k], 74.f - lv * 58.f, 6.f, lv * 58.f);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, 74.f, 0, 16.f, k < 2 ? colAccent() : nvgRGB(0x3d, 0xe0, 0xff), k < 2 ? nvgRGB(0x8a, 0xee, 0xff) : colViolet()));
                nvgFill(vg);
            }
            drawText(vg, vb[k] + 3.f, 80.f, vn[k], 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        }

        screenFinish(vg, W, H);

        char buf[64];
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
        NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 230);
        snprintf(buf, sizeof(buf), "RAIN %3.0f%%", inten * 100.f);
        drawText(vg, 8.f, 8.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, W / 2.f, 8.f, "STORM", 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
        snprintf(buf, sizeof(buf), "GUST %3.0f%%", gust * 100.f);
        drawText(vg, W - 8.f, 8.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        const float rx = W - 6.f;
        snprintf(buf, sizeof(buf), "RATE %.1f/M", rate);
        drawText(vg, rx, 24.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
        snprintf(buf, sizeof(buf), "ECHO %2.0f%%", echo * 100.f);
        drawText(vg, rx, 36.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
        snprintf(buf, sizeof(buf), "REV %2.0f%%", rev * 100.f);
        drawText(vg, rx, 48.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
        snprintf(buf, sizeof(buf), "WID %3.0f%%", wid * 100.f);
        drawText(vg, rx, 60.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
        snprintf(buf, sizeof(buf), "GAIN %3.0f%%", gain * 100.f);
        drawText(vg, rx, 72.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);

        if (tsince >= 100.f) snprintf(buf, sizeof(buf), "LAST BANG --");
        else snprintf(buf, sizeof(buf), "LAST BANG %.0fS AGO", tsince);
        drawText(vg, 8.f, 102.f, buf, 7.f, nvgRGBA(0xd8, 0xb0, 0xff, 220), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct RainStormWidget : ModuleWidget {
    RainStormWidget(RainStorm* module) {
        setModule(module);
        box.size = Vec(RS_W, RS_H);

        addChild(new RSPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new RSDisplay(module));

        // ---- row A: knob on top, CV jack below where the knob has one ----
        const int pa[8] = {RainStorm::INTENSITY_PARAM, RainStorm::GUST_PARAM, RainStorm::WIDTH_PARAM,
                           RainStorm::THUNDER_INTENSITY_PARAM, RainStorm::THUNDER_RATE_PARAM,
                           RainStorm::ECHO_TIME_PARAM, RainStorm::ECHO_FEEDBACK_PARAM, RainStorm::ECHO_MIX_PARAM};
        const int ca[8] = {RainStorm::INTENSITY_CV, -1, -1, RainStorm::THUNDER_INTENSITY_CV,
                           RainStorm::THUNDER_RATE_CV, RainStorm::ECHO_TIME_CV,
                           RainStorm::ECHO_FEEDBACK_CV, RainStorm::ECHO_MIX_CV};
        for (int i = 0; i < 8; ++i) {
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(rsCol(i), RS_R1 + 30.f), module, pa[i]));
            if (ca[i] >= 0)
                addInput(createInputCentered<DarkPJ301MPort>(Vec(rsCol(i), RS_R1 + 64.f), module, ca[i]));
        }

        // ---- row B ----
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(rsCol(1), RS_R2 + 30.f), module, RainStorm::OUTPUT_GAIN_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(rsCol(1), RS_R2 + 64.f), module, RainStorm::OUTPUT_GAIN_CV));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(rsCol(2), RS_R2 + 30.f), module, RainStorm::REVERB_SIZE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(rsCol(3), RS_R2 + 30.f), module, RainStorm::REVERB_MIX_PARAM));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(rsCol(4), RS_R2 + 36.f), module, RainStorm::TRIG_INPUT));
        addChild(createLightCentered<SmallLight<RSCyanLight>>(Vec(rsCol(4), RS_R2 + 66.f), module, RainStorm::THUNDER_LIGHT));

        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(rsCol(5), RS_R2 + 36.f), module, RainStorm::L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(rsCol(6), RS_R2 + 36.f), module, RainStorm::R_OUTPUT));
    }
};

Model* modelRainStorm = createModel<RainStorm, RainStormWidget>("RainStorm");