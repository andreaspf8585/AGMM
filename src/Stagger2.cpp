#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>
#include <map>
#include <string>
#include <regex>
#include <algorithm>
#include <random>
#include "dsp/minblep.hpp"

static const float S2_W = RACK_GRID_WIDTH * 28;   // 420
static const float S2_H = RACK_GRID_HEIGHT;       // 380
static const float S2_DX = 40.f, S2_DY = 46.f, S2_DW = 340.f, S2_DH = 62.f;
static float s2Col(int i) { return 35.f + 70.f * i; }
// Row tops (cells are 64 wide)
static const float S2_T1 = 126.f, S2_H1 = 70.f;
static const float S2_T2 = 200.f, S2_T3 = 254.f, S2_T4 = 308.f, S2_HK = 50.f;

// -----------------------------------------------------------------------------
// Custom ParamQuantity for the frequency knob (10 Hz - 6000 Hz, logarithmic)
// -----------------------------------------------------------------------------
struct S2FreqQ : rack::engine::ParamQuantity {
    static float mapToFreq(float v) {
        return 10.0f * std::pow(600.0f, v);
    }
    static float mapFromFreq(float f) {
        f = math::clamp(f, 10.0f, 6000.0f);
        return std::log(f / 10.0f) / std::log(600.0f);
    }

    float getDisplayValue() override {
        return mapToFreq(getValue());
    }

    void setDisplayValue(float displayValue) override {
        setValue(mapFromFreq(displayValue));
    }

    std::string getDisplayValueString() override {
        float freq = getDisplayValue();
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f Hz", freq);
        return std::string(buf);
    }

    void setDisplayValueString(std::string s) override {
        s.erase(std::remove_if(s.begin(), s.end(), ::isspace), s.end());
        if (s.empty()) return;

        auto parseNote = [](const std::string& note) -> float {
            static const std::map<std::string, int> noteMap = {
                {"C", 0}, {"C#", 1}, {"Db", 1}, {"D", 2}, {"D#", 3}, {"Eb", 3},
                {"E", 4}, {"F", 5}, {"F#", 6}, {"Gb", 6}, {"G", 7}, {"G#", 8},
                {"Ab", 8}, {"A", 9}, {"A#", 10}, {"Bb", 10}, {"B", 11}
            };
            std::regex noteRegex(R"(([A-Ga-g][#b]?)(-?\d+))");
            std::smatch match;
            if (std::regex_match(note, match, noteRegex)) {
                std::string pitchClass = match[1].str();
                int octave = std::stoi(match[2].str());
                for (auto& c : pitchClass) c = toupper(c);
                if (noteMap.find(pitchClass) != noteMap.end()) {
                    int semitone = noteMap.at(pitchClass) + (octave + 1) * 12;
                    float freq = 440.0f * std::pow(2.0f, (semitone - 69) / 12.0f);
                    return freq;
                }
            }
            return -1.0f;
        };

        try {
            size_t pos;
            float freq = std::stof(s, &pos);
            if (pos == s.length()) {
                setDisplayValue(freq);
                return;
            }
        } catch (...) {}

        float noteFreq = parseNote(s);
        if (noteFreq > 0) {
            setDisplayValue(noteFreq);
            return;
        }

        if (s[0] == '+' || s[0] == '-') {
            try {
                int semitones = std::stoi(s);
                float freq = dsp::FREQ_C4 * std::pow(2.0f, semitones / 12.0f);
                setDisplayValue(freq);
                return;
            } catch (...) {}
        }

        ParamQuantity::setDisplayValueString(s);
    }
};

// -----------------------------------------------------------------------------
// ParamQuantity subclasses (display of delay time, reverb decay, envelope decay)
// -----------------------------------------------------------------------------
struct S2DelayTimeQ : rack::engine::ParamQuantity {
    std::string getDisplayValueString() override {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.0f ms", 20.0f * std::pow(75.0f, getValue()));
        return std::string(buf);
    }
    void setDisplayValueString(std::string s) override {
        try {
            size_t pos;
            float ms = std::stof(s, &pos);
            if (ms > 0.f) {
                setValue(std::log(ms / 20.0f) / std::log(75.0f));
                return;
            }
        } catch (...) {}
        ParamQuantity::setDisplayValueString(s);
    }
};

struct S2RevDecayQ : rack::engine::ParamQuantity {
    std::string getDisplayValueString() override {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.2f s", 0.3f * std::pow(40.0f, getValue()));
        return std::string(buf);
    }
    void setDisplayValueString(std::string s) override {
        try {
            size_t pos;
            float sec = std::stof(s, &pos);
            if (sec > 0.f) {
                setValue(std::log(sec / 0.3f) / std::log(40.0f));
                return;
            }
        } catch (...) {}
        ParamQuantity::setDisplayValueString(s);
    }
};

struct S2EnvDecayQ : rack::engine::ParamQuantity {
    std::string getDisplayValueString() override {
        static const char* names[4] = {"300 ms", "1 s", "4 s", "12 s"};
        int i = (int)std::nearbyint(getValue());
        if (i < 0) i = 0;
        if (i > 3) i = 3;
        return names[i];
    }
};

// -----------------------------------------------------------------------------
// Modulated dual delay line: normal or ping-pong feedback, damped repeats,
// slow sine modulation of the read position (chorus / spacey wobble).
// -----------------------------------------------------------------------------
struct S2ModDelay {
    static constexpr int MAX_DELAY = 72008; // ~1.5 s at 48 kHz
    std::vector<float> bufL, bufR;
    int size = 0;
    int writeIdx = 0;
    float lpL = 0.f, lpR = 0.f;

    void setSampleRate(float sr) {
        (void)sr;
        if (bufL.empty()) {
            size = MAX_DELAY;
            bufL.assign(size, 0.f);
            bufR.assign(size, 0.f);
            writeIdx = 0;
        }
    }

    inline float readFrac(const std::vector<float>& buf, float pos) const {
        int i0 = (int)pos;
        float frac = pos - (float)i0;
        int i1 = i0 + 1;
        if (i1 >= size) i1 = 0;
        float a = buf[i0];
        float b = buf[i1];
        return a + (b - a) * frac;
    }

    void clear() {
        std::fill(bufL.begin(), bufL.end(), 0.f);
        std::fill(bufR.begin(), bufR.end(), 0.f);
        lpL = lpR = 0.f;
    }

    void process(float& ioL, float& ioR, float delaySec, float fb, float modDepth,
                 float lfoA01, float lfoB01, float dampCoef, int mode, float sr) {
        if (size == 0) return;
        float maxDelay = (float)(size - 4);
        float d = std::min(delaySec * sr, maxDelay);
        float mod = 8.0f * modDepth;
        float posL = d + mod * (lfoA01 * 2.0f - 1.0f);
        float posR = d + mod * (lfoB01 * 2.0f - 1.0f);
        posL = std::max(1.0f, std::min(posL, maxDelay));
        posR = std::max(1.0f, std::min(posR, maxDelay));

        float dl = readFrac(bufL, posL);
        float dr = readFrac(bufR, posR);

        lpL += dampCoef * (dl - lpL);
        lpR += dampCoef * (dr - lpR);

        float outL = ioL + (mode == 0 ? lpL : lpR) * fb;
        float outR = ioR + (mode == 0 ? lpR : lpL) * fb;

        bufL[writeIdx] = outL;
        bufR[writeIdx] = outR;
        writeIdx++;
        if (writeIdx >= size) writeIdx = 0;

        ioL = outL;
        ioR = outR;
    }
};

// -----------------------------------------------------------------------------
// Simple peak compressor. The threshold is fixed at 3 V, amount scales the
// ratio from 1:1 (off) to 10:1.
// -----------------------------------------------------------------------------
struct S2Comp {
    float env = 0.f;

    float process(float x, float amount, float sr) {
        if (amount < 0.001f) return x;

        float absx = std::fabs(x);
        float attackCoef = 1.f - std::exp(-1.f / (0.002f * sr));   // ~2 ms
        float releaseCoef = 1.f - std::exp(-1.f / (0.100f * sr));  // ~100 ms
        float coef = (absx > env) ? attackCoef : releaseCoef;
        env += coef * (absx - env);

        const float threshold = 3.0f;
        if (env <= threshold) return x;

        float ratio = 1.f + amount * 9.f;   // 1:1 .. 10:1
        float compressedEnv = threshold + (env - threshold) / ratio;
        float gainRed = compressedEnv / env;
        return x * gainRed;
    }
};

// -----------------------------------------------------------------------------
// FDN reverb: 8 mutually-prime comb filters in parallel + 4 series allpass
// filters, with cross-feed and polarity decorrelation for a wide tail.
// -----------------------------------------------------------------------------
struct S2Fdn {
    static constexpr int NUM_COMBS = 8;
    static constexpr int NUM_ALLPASS = 4;
    static constexpr int COMB_BUF = 2400;
    static constexpr int AP_BUF = 1100;

    std::vector<float> combBuf[NUM_COMBS];
    int combLen[NUM_COMBS] = {0};
    int combIdx[NUM_COMBS] = {0};
    float combFb[NUM_COMBS] = {0.f};
    float combLp[NUM_COMBS] = {0.f};

    std::vector<float> apBuf[NUM_ALLPASS];
    float apLenF[NUM_ALLPASS] = {0.f};
    int apIdx[NUM_ALLPASS] = {0};
    float apG = 0.5f;

    float crossL = 0.f, crossR = 0.f;
    float lastSize = -1.f, lastRt = -1.f;

    static int primeMult(int i) {
        static const int m[NUM_COMBS] = {101, 113, 127, 139, 149, 163, 173, 181};
        return m[i];
    }

    void setSampleRate(float sr) {
        if (!combBuf[0].empty()) return;
        for (int i = 0; i < NUM_COMBS; i++) {
            combBuf[i].assign(COMB_BUF, 0.f);
        }
        for (int i = 0; i < NUM_ALLPASS; i++) {
            apBuf[i].assign(AP_BUF, 0.f);
        }
        lastSize = -1.f;
        (void)sr;
    }

    void setParams(float sr, float sizeParam, float rt60) {
        for (int i = 0; i < NUM_COMBS; i++) {
            int len = (int)(primeMult(i) * 10.0f * (0.28f + 0.72f * sizeParam));
            int minPrev = (i == 0) ? 0 : combLen[i - 1];
            if (len <= minPrev + 5) len = minPrev + 7;
            if (len < 16) len = 16 + i * 7;
            if (len > COMB_BUF - 8) len = COMB_BUF - 8 - (NUM_COMBS - i) * 7;
            combLen[i] = len;
            float decay = std::pow(10.0f, -3.0f * (float)len / (rt60 * sr));
            combFb[i] = std::min(decay, 0.995f);
        }
        static const int apBase[NUM_ALLPASS] = {293, 409, 541, 691};
        for (int i = 0; i < NUM_ALLPASS; i++) {
            apLenF[i] = (float)apBase[i];
        }
    }

    inline float readFrac(const std::vector<float>& buf, int wr, float pos) const {
        int sz = (int)buf.size();
        int i0 = wr - (int)pos;
        if (i0 < 0) i0 += sz;
        int i1 = i0 + 1;
        if (i1 >= sz) i1 = 0;
        float frac = pos - (float)(int)pos;
        float a = buf[i0];
        float b = buf[i1];
        return a + (b - a) * frac;
    }

    inline float allpassUpdate(int i, float x, float len, float g) {
        std::vector<float>& b = apBuf[i];
        int sz = (int)b.size();
        int idx = apIdx[i];
        float y = readFrac(b, idx, len);
        b[idx] = x - g * y;
        apIdx[i] = (idx + 1) % sz;
        return y * g + x;
    }

    void clear() {
        for (int i = 0; i < NUM_COMBS; i++) {
            std::fill(combBuf[i].begin(), combBuf[i].end(), 0.f);
            combLp[i] = 0.f;
        }
        for (int i = 0; i < NUM_ALLPASS; i++) {
            std::fill(apBuf[i].begin(), apBuf[i].end(), 0.f);
        }
        crossL = crossR = 0.f;
    }

    void process(float inL, float inR, float& outL, float& outR,
                 float sizeParam, float rt60, float mod,
                 float lfoA01, float lfoB01, float dampCoef, float sr) {
        if (combBuf[0].empty()) { outL = outR = 0.f; return; }
        if (sizeParam != lastSize || rt60 != lastRt) {
            setParams(sr, sizeParam, rt60);
            lastSize = sizeParam;
            lastRt = rt60;
        }

        float bus = 0.5f * (inL - inR);
        float sum = 0.f;
        for (int i = 0; i < NUM_COMBS; i++) {
            std::vector<float>& b = combBuf[i];
            int len = combLen[i];
            int idx = combIdx[i];
            if (idx >= len) idx = 0;
            float d = b[idx];
            combLp[i] += dampCoef * (d - combLp[i]);
            float out = bus + combLp[i] * combFb[i];
            b[idx] = out;
            combIdx[i] = (idx + 1) % len;
            sum += d;
        }

        float l = lfoA01 * 0.5f - 0.25f;
        float m = lfoB01 * 0.5f - 0.25f;
        float y = sum;
        y = allpassUpdate(0, y, apLenF[0] + apLenF[0] * mod * l, apG);
        y = allpassUpdate(1, y, apLenF[1] + apLenF[1] * mod * m, apG);
        y = allpassUpdate(2, y, apLenF[2] + apLenF[2] * mod * m, apG);
        y = allpassUpdate(3, y, apLenF[3] + apLenF[3] * mod * l, apG);

        crossL += 0.5f * (y - crossL);
        crossR += 0.5f * (y - crossR);
        outL = y + crossR * 0.7f;
        outR = y - crossL * 0.7f;
    }
};

struct Stagger2 : Module {
    enum ParamId {
        PITCH_PARAM,
        CHAOS_PARAM,
        CHAOS_MULT_PARAM,
        WAVE_PARAM,
        PW_PARAM,
        PITCH_ATTEN_PARAM,
        CHAOS_ATTEN_PARAM,
        WAVE_ATTEN_PARAM,
        PW_ATTEN_PARAM,
        CV_RANGE_PARAM,
        DELAY_TIME_PARAM,
        DELAY_FB_PARAM,
        DELAY_MIX_PARAM,
        DELAY_MOD_PARAM,
        DELAY_MODE_PARAM,
        REV_SIZE_PARAM,
        REV_DECAY_PARAM,
        REV_MIX_PARAM,
        ENV_DECAY_PARAM,
        TRIG_MODE_PARAM,
        GAIN_PARAM,
        COMP_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        CV_INPUT,
        PITCH_CV_INPUT,
        CHAOS_CV_INPUT,
        WAVE_CV_INPUT,
        PW_CV_INPUT,
        CHAOS_RESET_INPUT,
        TRIG_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        VCO_OUTPUT,
        FX_L_OUTPUT,
        FX_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        CHAOS_LIGHT,
        ENV_LIGHT,
        LIGHTS_LEN
    };

    struct Voice {
        float phase = 0.0f;
        float lastPhase = 0.0f;
        dsp::MinBlepGenerator<16, 16> blepRising;
        dsp::MinBlepGenerator<16, 16> blepFalling;
        dsp::MinBlepGenerator<16, 16> blepSaw;
        // Per-voice space FX
        S2ModDelay delay;
        S2Fdn reverb;
        S2Comp comp;
        float env = 1.0f;
        bool gateHigh = false;
        float lastTrig = 0.0f;
    };
    std::vector<Voice> voices;

    // Post-FX master compressor for the stereo (mono-summed) FX outputs
    S2Comp compL, compR;

    float chaosDrift = 0.0f;
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist{-1.0f, 1.0f};

    float lfoPhaseA = 0.0f;
    float lfoPhaseB = 0.0f;

    static constexpr float multTable[5] = {1.0f, 2.0f, 3.0f, 5.0f, 10.0f};
    static constexpr float envDecayTable[4] = {0.3f, 1.0f, 4.0f, 12.0f};
    bool cvRangeTenVolt = false;
    float lastResetVoltage = 0.0f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float dispFreq = 261.6f, dispPw = 0.5f, dispPhase = 0.f;
    int dispWave = 0, dispChannels = 1;
    float dispEnv = 1.f;
    bool dispTrigConn = false;
    float meterL = 0.f, meterR = 0.f;

    Stagger2() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam<S2FreqQ>(PITCH_PARAM, 0.f, 1.f, S2FreqQ::mapFromFreq(dsp::FREQ_C4), "Frequency");

        configParam(CHAOS_PARAM, 0.f, 1.f, 0.0f, "Chaos depth", "%", 0.f, 100.f);
        configSwitch(CHAOS_MULT_PARAM, 0.f, 4.f, 0.f, "Chaos multiplier", {"x1", "x2", "x3", "x5", "x10"});
        configSwitch(WAVE_PARAM, 0.f, 3.f, 0.f, "Waveform", {"Sine", "Triangle", "Saw", "Square"});
        configParam(PW_PARAM, 0.f, 1.f, 0.5f, "Pulse width", "%", 0.f, 100.f);
        configParam(PITCH_ATTEN_PARAM, -1.f, 1.f, 0.f, "Pitch CV attenuverter", "%", 0.f, 100.f);
        configParam(CHAOS_ATTEN_PARAM, -1.f, 1.f, 0.f, "Chaos CV attenuverter", "%", 0.f, 100.f);
        configParam(WAVE_ATTEN_PARAM, -1.f, 1.f, 0.f, "Waveform CV attenuverter", "%", 0.f, 100.f);
        configParam(PW_ATTEN_PARAM, -1.f, 1.f, 0.f, "Pulse width CV attenuverter", "%", 0.f, 100.f);
        configSwitch(CV_RANGE_PARAM, 0.f, 1.f, 0.f, "CV range", {"+-5V", "+-10V"});

        // ---- Space FX ----
        configParam<S2DelayTimeQ>(DELAY_TIME_PARAM, 0.f, 1.f, 0.55f, "Delay time");
        configParam(DELAY_FB_PARAM, 0.f, 0.92f, 0.65f, "Delay feedback", "%", 0.f, 100.f);
        configParam(DELAY_MIX_PARAM, 0.f, 1.f, 0.6f, "Delay mix", "%", 0.f, 100.f);
        configParam(DELAY_MOD_PARAM, 0.f, 1.f, 0.5f, "Delay modulation", "%", 0.f, 100.f);
        configSwitch(DELAY_MODE_PARAM, 0.f, 1.f, 1.f, "Delay mode", {"Normal", "Ping-Pong"});

        configParam(REV_SIZE_PARAM, 0.f, 1.f, 0.75f, "Reverb size", "%", 0.f, 100.f);
        configParam<S2RevDecayQ>(REV_DECAY_PARAM, 0.f, 1.f, 0.55f, "Reverb decay");
        configParam(REV_MIX_PARAM, 0.f, 1.f, 0.75f, "Reverb mix", "%", 0.f, 100.f);

        // ---- Trigger / envelope ----
        configParam<S2EnvDecayQ>(ENV_DECAY_PARAM, 0.f, 3.f, 1.f, "Trigger decay");
        getParamQuantity(ENV_DECAY_PARAM)->snapEnabled = true;
        configSwitch(TRIG_MODE_PARAM, 0.f, 1.f, 0.f, "Trigger mode", {"Trig", "Gate"});

        // ---- Master output stage ----
        configParam(GAIN_PARAM, 0.f, 2.f, 0.7f, "Output gain", "%", 0.f, 100.f);
        configParam(COMP_PARAM, 0.f, 1.f, 0.5f, "Compressor", "%", 0.f, 100.f);

        configInput(CV_INPUT, "1V/octave pitch");
        configInput(PITCH_CV_INPUT, "Pitch modulation CV");
        configInput(CHAOS_CV_INPUT, "Chaos depth modulation CV");
        configInput(WAVE_CV_INPUT, "Waveform selection CV");
        configInput(PW_CV_INPUT, "Pulse width modulation CV");
        configInput(CHAOS_RESET_INPUT, "Reset chaos drift (trigger)");
        configInput(TRIG_INPUT, "Trigger (trig or gate)");
        configOutput(VCO_OUTPUT, "VCO + FX");
        configOutput(FX_L_OUTPUT, "FX left (mono sum)");
        configOutput(FX_R_OUTPUT, "FX right (mono sum)");
        configLight(CHAOS_LIGHT, "Chaos activity");
        configLight(ENV_LIGHT, "Envelope");

        std::random_device rd;
        rng.seed(rd());
    }

    void onReset() override {
        chaosDrift = 0.0f;
        for (auto& v : voices) {
            v.delay.clear();
            v.reverb.clear();
            v.env = 1.0f;
            v.comp.env = 0.f;
        }
        compL.env = 0.f;
        compR.env = 0.f;
    }

    void onSampleRateChange() override {
        // Buffers are sized for the worst case; nothing to reallocate.
    }

    void process(const ProcessArgs& args) override {
        float freqKnob = S2FreqQ::mapToFreq(params[PITCH_PARAM].getValue());
        float basePitchVoltage = std::log2(freqKnob / dsp::FREQ_C4);

        float chaosKnob = params[CHAOS_PARAM].getValue();
        float chaosMultIndex = params[CHAOS_MULT_PARAM].getValue();
        float waveKnob = params[WAVE_PARAM].getValue();
        float pwKnob = params[PW_PARAM].getValue();

        float chaosMultiplier = multTable[clamp((int)std::nearbyint(chaosMultIndex), 0, 4)];

        float pitchAtten = params[PITCH_ATTEN_PARAM].getValue();
        float chaosAtten = params[CHAOS_ATTEN_PARAM].getValue();
        float waveAtten = params[WAVE_ATTEN_PARAM].getValue();
        float pwAtten = params[PW_ATTEN_PARAM].getValue();

        cvRangeTenVolt = (params[CV_RANGE_PARAM].getValue() > 0.5f);
        float cvVoltageScale = cvRangeTenVolt ? 0.5f : 1.0f;

        float resetVoltage = inputs[CHAOS_RESET_INPUT].getVoltage();
        if (resetVoltage > 1.0f && lastResetVoltage <= 1.0f) {
            chaosDrift = 0.0f;
        }
        lastResetVoltage = resetVoltage;

        float whiteNoise = dist(rng);
        float cutoffHz = 1.5f;
        float alpha = 2.0f * M_PI * cutoffHz * args.sampleTime;
        alpha = std::min(alpha, 0.5f);
        chaosDrift = (1.0f - alpha) * chaosDrift + alpha * whiteNoise;

        // ---- FX / trigger controls ----
        float delayTimeSec = 20.0e-3f * std::pow(75.0f, params[DELAY_TIME_PARAM].getValue());
        float delayFb = params[DELAY_FB_PARAM].getValue();
        float delayMix = params[DELAY_MIX_PARAM].getValue();
        float delayMod = params[DELAY_MOD_PARAM].getValue();
        int delayMode = (int)std::nearbyint(params[DELAY_MODE_PARAM].getValue());

        float revSize = params[REV_SIZE_PARAM].getValue();
        float revRt60 = 0.3f * std::pow(40.0f, params[REV_DECAY_PARAM].getValue());
        float revMix = params[REV_MIX_PARAM].getValue();

        float envDecaySec = envDecayTable[clamp((int)std::nearbyint(params[ENV_DECAY_PARAM].getValue()), 0, 3)];
        int trigMode = (int)std::nearbyint(params[TRIG_MODE_PARAM].getValue());

        bool trigConnected = inputs[TRIG_INPUT].isConnected();
        float trigVoltage = inputs[TRIG_INPUT].getVoltage();

        // ---- Master output stage ----
        float gain = params[GAIN_PARAM].getValue();
        float compAmount = params[COMP_PARAM].getValue();

        // Shared slow LFOs
        lfoPhaseA += args.sampleTime * 0.23f;
        if (lfoPhaseA >= 1.0f) lfoPhaseA -= 1.0f;
        lfoPhaseB += args.sampleTime * 0.17f;
        if (lfoPhaseB >= 1.0f) lfoPhaseB -= 1.0f;
        float lfoA01 = 0.5f * (std::sin(2.0f * M_PI * lfoPhaseA) + 1.0f);
        float lfoB01 = 0.5f * (std::sin(2.0f * M_PI * lfoPhaseB) + 1.0f);

        int channels = std::max(1, inputs[CV_INPUT].getChannels());
        outputs[VCO_OUTPUT].setChannels(channels);
        outputs[FX_L_OUTPUT].setChannels(1);
        outputs[FX_R_OUTPUT].setChannels(1);
        if ((int)voices.size() < channels) {
            voices.resize(channels);
        }
        dispChannels = channels;

        auto fastExp2 = [](float x) {
            if (x == 0.0f) return 1.0f;
            union { float f; uint32_t i; } u;
            u.i = (uint32_t)((1 << 23) * (x + 126.94269504f));
            return u.f;
        };

        float fxSumL = 0.0f, fxSumR = 0.0f;
        float envMax = 0.0f;

        for (int c = 0; c < channels; ++c) {
            float pitchCV = inputs[PITCH_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float chaosCV = inputs[CHAOS_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float waveCV = inputs[WAVE_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float pwCV = inputs[PW_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;

            float modulatedPitchCV = pitchCV * pitchAtten;
            float modulatedChaosCV = chaosCV * chaosAtten;
            float modulatedWaveCV = waveCV * waveAtten;
            float modulatedPwCV = pwCV * pwAtten;

            float totalPitchCV = basePitchVoltage + inputs[CV_INPUT].getPolyVoltage(c) + modulatedPitchCV;

            float chaosDepth = chaosKnob + modulatedChaosCV / 5.0f;
            chaosDepth = math::clamp(chaosDepth, 0.0f, 1.0f);

            float chaosVoltage = chaosDrift * chaosDepth * chaosMultiplier;

            float totalCV = totalPitchCV + chaosVoltage;
            float freq = dsp::FREQ_C4 * fastExp2(totalCV);
            freq = math::clamp(freq, 10.0f, 6000.0f);

            float deltaPhase = freq * args.sampleTime;

            float waveSelect = waveKnob + (modulatedWaveCV / 5.0f) * 2.0f;
            waveSelect = math::clamp(waveSelect, 0.0f, 3.0f);
            int waveIndex = (int)std::nearbyint(waveSelect);

            float pwValue = pwKnob + (modulatedPwCV / 10.0f);
            pwValue = math::clamp(pwValue, 0.01f, 0.99f);

            Voice& voice = voices[c];
            float& phase = voice.phase;
            float lastPhase = voice.lastPhase;
            float out = 0.0f;

            lastPhase = phase;
            phase += deltaPhase;
            if (phase >= 1.0f) {
                phase -= 1.0f;
            }

            switch (waveIndex) {
                case 0:
                    out = std::sin(2.0f * M_PI * phase);
                    break;
                case 1:
                    out = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
                    break;
                case 2: {
                    float saw = 1.0f - 2.0f * phase;
                    if (phase < lastPhase) {
                        float t = (1.0f - lastPhase) / deltaPhase;
                        voice.blepSaw.insertDiscontinuity(t, 2.0f);
                    }
                    out = saw + voice.blepSaw.process();
                    break;
                }
                case 3: {
                    float square = (phase < pwValue) ? 1.0f : -1.0f;
                    if (phase < lastPhase) {
                        float t = (1.0f - lastPhase) / deltaPhase;
                        voice.blepRising.insertDiscontinuity(t, 2.0f);
                    }
                    if ((lastPhase < pwValue && phase >= pwValue) || (pwValue == 0.0f && phase < lastPhase)) {
                        float t = (pwValue - lastPhase) / deltaPhase;
                        voice.blepFalling.insertDiscontinuity(t, -2.0f);
                    }
                    out = square + voice.blepRising.process() + voice.blepFalling.process();
                    break;
                }
            }

            voice.lastPhase = phase;
            out *= 5.0f;

            if (voice.delay.bufL.empty()) {
                voice.delay.setSampleRate(args.sampleRate);
                voice.reverb.setSampleRate(args.sampleRate);
            }

            // ---- Trigger envelope ----
            float& env = voice.env;
            float attackCoef = std::min(1.0f, args.sampleTime / 0.003f);
            float decayCoef = args.sampleTime / (0.001f + envDecaySec);
            if (trigConnected) {
                if (trigMode == 0) {
                    if (trigVoltage > 1.0f && voice.lastTrig <= 1.0f) env = 1.0f;
                    env *= (1.0f - decayCoef);
                } else {
                    if (trigVoltage > 1.0f) voice.gateHigh = true;
                    else if (trigVoltage < 0.1f) voice.gateHigh = false;
                    if (voice.gateHigh) env += attackCoef * (1.0f - env);
                    else env *= (1.0f - decayCoef);
                }
            } else {
                env = 1.0f;
            }
            env = math::clamp(env, 0.0f, 1.0f);
            voice.lastTrig = trigVoltage;
            if (env > envMax) envMax = env;

            float sig = out * env;

            // ---- Space FX chain ----
            float dl = sig, dr = sig * 0.6f;
            voice.delay.process(dl, dr, delayTimeSec, delayFb, delayMod,
                                lfoA01, lfoB01, 0.35f, delayMode, args.sampleRate);
            float delayWetL = dl, delayWetR = dr;

            float revInL = sig * 0.5f + delayWetL;
            float revInR = sig * 0.5f + delayWetR;
            float revOutL = 0.f, revOutR = 0.f;
            voice.reverb.process(revInL, revInR, revOutL, revOutR,
                                 revSize, revRt60, delayMod, lfoA01, lfoB01, 0.30f, args.sampleRate);

            float wetL = delayMix * delayWetL + revMix * revOutL;
            float wetR = delayMix * delayWetR + revMix * revOutR;

            float norm = 2.0f / (1.0f + delayMix + revMix);
            float mixedL = (sig + 0.8f * wetL) * norm;
            float mixedR = (sig + 0.8f * wetR) * norm;

            // ---- Master gain + per-voice compressor on VCO output ----
            float vcoSample = (mixedL + mixedR * 0.15f) * gain;
            vcoSample = voice.comp.process(vcoSample, compAmount, args.sampleRate);
            outputs[VCO_OUTPUT].setVoltage(vcoSample, c);

            // Accumulate for the (post-summed) FX outputs
            fxSumL += mixedL * gain;
            fxSumR += mixedR * gain;

            if (c == 0) {
                dispFreq = freq; dispPw = pwValue; dispWave = waveIndex; dispPhase = phase;
            }
        }

        // ---- Master compressor for FX outputs ----
        fxSumL = compL.process(fxSumL, compAmount, args.sampleRate);
        fxSumR = compR.process(fxSumR, compAmount, args.sampleRate);
        outputs[FX_L_OUTPUT].setVoltage(math::clamp(fxSumL, -10.0f, 10.0f), 0);
        outputs[FX_R_OUTPUT].setVoltage(math::clamp(fxSumR, -10.0f, 10.0f), 0);

        float chaosIntensity = std::abs(chaosDrift) * chaosKnob;
        lights[CHAOS_LIGHT].setSmoothBrightness(chaosIntensity, args.sampleTime);
        lights[ENV_LIGHT].setSmoothBrightness(envMax, args.sampleTime);

        // display data
        dispEnv = envMax;
        dispTrigConn = trigConnected;
        float md = 1.f - 5.f * args.sampleTime;
        meterL = std::max(std::fabs(fxSumL), meterL * md);
        meterR = std::max(std::fabs(fxSumR), meterR * md);
    }
};

constexpr float Stagger2::multTable[5];
constexpr float Stagger2::envDecayTable[4];

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
struct TS2CyanLight : TBase {
    TS2CyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using S2CyanLight = TS2CyanLight<>;

template <typename TBase = GrayModuleLightWidget>
struct TS2VioletLight : TBase {
    TS2VioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
};
using S2VioletLight = TS2VioletLight<>;

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
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
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

float s2Wave(float p, int wave, float pw) {
    switch (wave) {
        case 1:  return (p < 0.5f) ? (4.f * p - 1.f) : (3.f - 4.f * p);
        case 2:  return 1.f - 2.f * p;
        case 3:  return (p < pw) ? 1.f : -1.f;
        default: return std::sin(2.f * (float) M_PI * p);
    }
}

// Cell helpers: all plates are 64 wide
void kCell(NVGcontext* vg, int col, float top, const char* t, bool withRing) {
    float cx = s2Col(col);
    plate(vg, cx - 32.f, top, 64.f, S2_HK);
    if (withRing) ring(vg, cx, top + 19.f, 16.f);
    label(vg, cx, top + 42.f, t);
}

struct S2Panel : TransparentWidget {
    S2Panel() { box.size = Vec(S2_W, S2_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, S2_W, S2_H, "STAGGER2");
        drawBezel(vg, S2_DX, S2_DY, S2_DW, S2_DH);

        // LED captions beside the title
        drawText(vg, 52.f, 18.f, "CHAOS", 7.f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, S2_W - 52.f, 18.f, "ENV", 7.f, colDim(), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        // row 1: four knobs with CV jack + attenuverter, then V/OCT and RESET
        const char* a[4] = {"FREQ", "CHAOS", "WAVE", "PW"};
        for (int i = 0; i < 4; ++i) {
            float cx = s2Col(i);
            plate(vg, cx - 32.f, S2_T1, 64.f, S2_H1);
            ring(vg, cx, S2_T1 + 17.f, 15.f);
            label(vg, cx, S2_T1 + 38.f, a[i]);
        }
        {
            const char* j[2] = {"V/OCT", "RESET"};
            for (int i = 0; i < 2; ++i) {
                float cx = s2Col(4 + i);
                plate(vg, cx - 32.f, S2_T1, 64.f, S2_H1);
                label(vg, cx, S2_T1 + 9.f, j[i]);
            }
        }

        // row 2: MULT, delay, ping-pong
        kCell(vg, 0, S2_T2, "MULT", true);
        kCell(vg, 1, S2_T2, "DLY TIME", true);
        kCell(vg, 2, S2_T2, "DLY FB", true);
        kCell(vg, 3, S2_T2, "DLY MOD", true);
        kCell(vg, 4, S2_T2, "DLY MIX", true);
        kCell(vg, 5, S2_T2, "PING-PONG", false);

        // row 3: reverb, trigger
        kCell(vg, 0, S2_T3, "RV SIZE", true);
        kCell(vg, 1, S2_T3, "RV TIME", true);
        kCell(vg, 2, S2_T3, "RV MIX", true);
        kCell(vg, 3, S2_T3, "TRIG IN", false);
        kCell(vg, 4, S2_T3, "ENV DECAY", true);
        kCell(vg, 5, S2_T3, "TRIG/GATE", false);

        // row 4: output stage
        kCell(vg, 0, S2_T4, "GAIN", true);
        kCell(vg, 1, S2_T4, "COMP", true);
        kCell(vg, 2, S2_T4, "CV RANGE", false);
        kCell(vg, 3, S2_T4, "FX L", false);
        kCell(vg, 4, S2_T4, "FX R", false);
        kCell(vg, 5, S2_T4, "VCO OUT", false);
        Widget::draw(args);
    }
};

// Display: waveform preview with live phase, FX readouts and output meters
struct S2Display : TransparentWidget {
    Stagger2* module = nullptr;
    S2Display(Stagger2* m) : module(m) {
        box.pos = Vec(S2_DX, S2_DY);
        box.size = Vec(S2_DW, S2_DH);
    }

    static constexpr float SX0 = 8.f, SY0 = 14.f, SW = 190.f, SH = 36.f;

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        screenBase(vg, S2_DW, S2_DH);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, SX0 - 1.f, SY0 - 1.f, SW + 2.f, SH + 2.f, 2.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgMoveTo(vg, SX0, SY0 + SH / 2.f + 0.5f);
        nvgLineTo(vg, SX0 + SW, SY0 + SH / 2.f + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
        nvgStroke(vg);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = S2_DW, H = S2_DH;

        float freq = 261.6f, pw = 0.5f, phase = 0.3f, env = 1.f, mL = 2.5f, mR = 2.1f;
        float dTime = 0.35f, dFb = 0.65f, rt = 3.2f, gain = 0.7f;
        int wave = 0, ch = 1, mode = 1, tmode = 0;
        bool trig = false;
        if (module) {
            freq = module->dispFreq; pw = module->dispPw; phase = module->dispPhase;
            wave = module->dispWave; ch = module->dispChannels;
            env = module->dispEnv; trig = module->dispTrigConn;
            mL = module->meterL; mR = module->meterR;
            dTime = 0.020f * std::pow(75.f, module->params[Stagger2::DELAY_TIME_PARAM].getValue());
            dFb = module->params[Stagger2::DELAY_FB_PARAM].getValue();
            rt = 0.3f * std::pow(40.f, module->params[Stagger2::REV_DECAY_PARAM].getValue());
            mode = (int) std::round(module->params[Stagger2::DELAY_MODE_PARAM].getValue());
            tmode = (int) std::round(module->params[Stagger2::TRIG_MODE_PARAM].getValue());
            gain = module->params[Stagger2::GAIN_PARAM].getValue();
        }
        wave = std::min(std::max(wave, 0), 3);

        // waveform, two cycles
        const float mid = SY0 + SH / 2.f, amp = SH / 2.f - 3.f;
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            const int N = 190;
            for (int i = 0; i <= N; ++i) {
                float t = 2.f * (float) i / N;
                float p = t - std::floor(t);
                float v = s2Wave(p, wave, pw) * (0.25f + 0.75f * env);
                float x = SX0 + SW * (float) i / N, y = mid - v * amp;
                if (i == 0) nvgMoveTo(vg, x, y);
                else nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 50) : nvgRGB(0x8a, 0xee, 0xff));
            nvgStrokeWidth(vg, pass == 0 ? 4.f : 1.3f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
        }
        {
            float x = SX0 + SW * 0.5f * phase, y = mid - s2Wave(phase, wave, pw) * (0.25f + 0.75f * env) * amp;
            nvgBeginPath(vg);
            nvgMoveTo(vg, x, SY0);
            nvgLineTo(vg, x, SY0 + SH);
            nvgStrokeColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, 4.f);
            nvgFillColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 70));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, 1.8f);
            nvgFillColor(vg, nvgRGB(0xf0, 0xe0, 0xff));
            nvgFill(vg);
        }

        // envelope bar (shows the trigger envelope)
        {
            nvgBeginPath(vg);
            nvgRect(vg, SX0 + SW + 8.f, SY0, 5.f, SH);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            float h = std::min(std::max(env, 0.f), 1.f) * SH;
            if (h > 0.5f) {
                nvgBeginPath(vg);
                nvgRect(vg, SX0 + SW + 8.f, SY0 + SH - h, 5.f, h);
                nvgFillColor(vg, withAlpha(colViolet(), 0.9f));
                nvgFill(vg);
            }
        }

        // readouts column
        char buf[48];
        const float rx = 226.f;
        snprintf(buf, sizeof(buf), "DLY %.0fMS", dTime * 1000.f);
        drawText(vg, rx, 18.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "%s FB%.0f%%", mode == 1 ? "PP" : "NRM", dFb * 100.f);
        drawText(vg, rx, 29.f, buf, 7.f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "RV %.1fS", rt);
        drawText(vg, rx, 40.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "%s", trig ? (tmode == 1 ? "GATE ENV" : "TRIG ENV") : "NO TRIG");
        drawText(vg, rx, 51.f, buf, 7.f, trig ? nvgRGBA(0xd8, 0xb0, 0xff, 230) : colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

        // output meters
        for (int k = 0; k < 2; ++k) {
            float lv = std::min((k == 0 ? mL : mR) / 5.f, 1.f);
            float x = 318.f + k * 7.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 14.f, 5.f, 36.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lv > 0.01f) {
                float h = lv * 36.f;
                nvgBeginPath(vg);
                nvgRect(vg, x, 50.f - h, 5.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, 50.f, 0, 14.f, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }

        screenFinish(vg, W, H);

        static const char* wn[4] = {"SINE", "TRI", "SAW", "SQR"};
        snprintf(buf, sizeof(buf), "%.1fHZ  %s%s", freq, wn[wave], ch > 1 ? "  POLY" : "");
        drawText(vg, 8.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "GAIN %.0f%%", gain * 100.f);
        drawText(vg, W - 8.f, 7.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct Stagger2Widget : ModuleWidget {
    Stagger2Widget(Stagger2* module) {
        setModule(module);
        box.size = Vec(S2_W, S2_H);

        addChild(new S2Panel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new S2Display(module));

        // LEDs beside the title
        addChild(createLightCentered<SmallLight<S2CyanLight>>(Vec(40.f, 18.f), module, Stagger2::CHAOS_LIGHT));
        addChild(createLightCentered<SmallLight<S2VioletLight>>(Vec(S2_W - 40.f, 18.f), module, Stagger2::ENV_LIGHT));

        // ---- row 1: knob on top, CV jack and attenuverter below; V/OCT and RESET jacks ----
        const int knob[4] = {Stagger2::PITCH_PARAM, Stagger2::CHAOS_PARAM, Stagger2::WAVE_PARAM, Stagger2::PW_PARAM};
        const int att[4] = {Stagger2::PITCH_ATTEN_PARAM, Stagger2::CHAOS_ATTEN_PARAM, Stagger2::WAVE_ATTEN_PARAM, Stagger2::PW_ATTEN_PARAM};
        const int cv[4] = {Stagger2::PITCH_CV_INPUT, Stagger2::CHAOS_CV_INPUT, Stagger2::WAVE_CV_INPUT, Stagger2::PW_CV_INPUT};
        for (int i = 0; i < 4; ++i) {
            float cx = s2Col(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, S2_T1 + 17.f), module, knob[i]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 15.f, S2_T1 + 53.f), module, cv[i]));
            addParam(createParamCentered<Trimpot>(Vec(cx + 15.f, S2_T1 + 53.f), module, att[i]));
        }
        addInput(createInputCentered<DarkPJ301MPort>(Vec(s2Col(4), S2_T1 + 40.f), module, Stagger2::CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(s2Col(5), S2_T1 + 40.f), module, Stagger2::CHAOS_RESET_INPUT));

        // ---- row 2 ----
        const float y2 = S2_T2 + 19.f;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(0), y2), module, Stagger2::CHAOS_MULT_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(1), y2), module, Stagger2::DELAY_TIME_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(2), y2), module, Stagger2::DELAY_FB_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(3), y2), module, Stagger2::DELAY_MOD_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(4), y2), module, Stagger2::DELAY_MIX_PARAM));
        addParam(createParamCentered<CKSS>(Vec(s2Col(5), y2), module, Stagger2::DELAY_MODE_PARAM));

        // ---- row 3 ----
        const float y3 = S2_T3 + 19.f;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(0), y3), module, Stagger2::REV_SIZE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(1), y3), module, Stagger2::REV_DECAY_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(2), y3), module, Stagger2::REV_MIX_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(s2Col(3), y3), module, Stagger2::TRIG_INPUT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(4), y3), module, Stagger2::ENV_DECAY_PARAM));
        addParam(createParamCentered<CKSS>(Vec(s2Col(5), y3), module, Stagger2::TRIG_MODE_PARAM));

        // ---- row 4 ----
        const float y4 = S2_T4 + 19.f;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(0), y4), module, Stagger2::GAIN_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(1), y4), module, Stagger2::COMP_PARAM));
        addParam(createParamCentered<CKSS>(Vec(s2Col(2), y4), module, Stagger2::CV_RANGE_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(s2Col(3), y4), module, Stagger2::FX_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(s2Col(4), y4), module, Stagger2::FX_R_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(s2Col(5), y4), module, Stagger2::VCO_OUTPUT));
    }
};

Model* modelStagger2 = createModel<Stagger2, Stagger2Widget>("Stagger2");