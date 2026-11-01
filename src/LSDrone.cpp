#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

// ============================================================
// Cathedral Drone engine – UNCHANGED from original
// ============================================================
struct LSDroneEngine
{
    struct Oscillator
    {
        float phase = 0.0f;
        float freq = 261.63f;
        float fine = 0.0f;
    };
    Oscillator oscA, oscB;
    float sampleRate = 44100.0f;
    float mix = 0.5f;
    float detune = 0.0f;

    float filterCutoff = 0.5f;
    float filterReso = 0.3f;
    float filterState = 0.0f;

    float lfoPhase = 0.0f;
    float lfoRate = 0.2f;
    float lfoDepth = 0.05f;
    float drift = 0.0f;
    float driftState = 0.0f;

    float *delayBufferL = nullptr, *delayBufferR = nullptr;
    int delayLength = 0, delayIdx = 0;
    float delayTime = 0.5f, delayFeedback = 0.6f, delayDiffusion = 0.3f;

    float *revBuffer = nullptr;
    int revLength = 0, revIdx = 0;
    float revDecay = 0.8f, revDamping = 0.5f;

    float washMix = 0.0f, shimmer = 0.0f;
    float modPhase = 0.0f, modRate = 0.1f, modDepth = 0.05f;

    float *echoBufferL = nullptr, *echoBufferR = nullptr;
    int echoLength = 0, echoIdx = 0;
    float echoTime = 0.5f;
    float echoFeedback = 0.5f;
    float echoMix = 0.5f;

    float lpfCutoff = 1.0f;
    float lpfReso = 0.0f;
    float lpfState = 0.0f;
    float lpfState2 = 0.0f;

    LSDroneEngine() {}
    ~LSDroneEngine()
    {
        delete[] delayBufferL;
        delete[] delayBufferR;
        delete[] revBuffer;
        delete[] echoBufferL;
        delete[] echoBufferR;
    }

    void init(float sr)
    {
        sampleRate = sr;
        delayLength = (int)(3.0f * sampleRate);
        delayBufferL = new float[delayLength]();
        delayBufferR = new float[delayLength]();
        revLength = (int)(2.0f * sampleRate);
        revBuffer = new float[revLength]();
        echoLength = (int)(2.0f * sampleRate);
        echoBufferL = new float[echoLength]();
        echoBufferR = new float[echoLength]();
    }

    void setFineA(float v) { oscA.fine = v; }
    void setFineB(float v) { oscB.fine = v; }
    void setDetune(float v) { detune = v; }
    void setMix(float v) { mix = v; }
    void setFilterCutoff(float v) { filterCutoff = v; }
    void setFilterReso(float v) { filterReso = v; }
    void setLfoRate(float v) { lfoRate = v; }
    void setLfoDepth(float v) { lfoDepth = v; }
    void setDrift(float v) { drift = v; }
    void setDelayTime(float v) { delayTime = v; }
    void setDelayFeedback(float v) { delayFeedback = v; }
    void setDelayDiffusion(float v) { delayDiffusion = v; }
    void setRevDecay(float v) { revDecay = v; }
    void setRevDamping(float v) { revDamping = v; }
    void setWash(float v) { washMix = v; }
    void setShimmer(float v) { shimmer = v; }
    void setModRate(float v) { modRate = v; }
    void setModDepth(float v) { modDepth = v; }
    void setEchoTime(float v) { echoTime = v; }
    void setEchoFeedback(float v) { echoFeedback = v; }
    void setEchoMix(float v) { echoMix = v; }
    void setLpfCutoff(float v) { lpfCutoff = v; }
    void setLpfReso(float v) { lpfReso = v; }

    float processOscillator(Oscillator &osc, float pitchMod)
    {
        float freq = osc.freq * std::pow(2.0f, pitchMod / 12.0f);
        osc.phase += freq / sampleRate;
        if (osc.phase >= 1.0f)
            osc.phase -= 1.0f;
        return std::sin(osc.phase * 2.0f * float(M_PI));
    }

    void process(float cvA, float cvB, float &leftOut, float &rightOut)
    {
        lfoPhase += lfoRate / sampleRate;
        if (lfoPhase >= 1.0f) lfoPhase -= 1.0f;
        float lfoMod = std::sin(lfoPhase * 2.0f * float(M_PI)) * lfoDepth;

        driftState += (rand() / (float)RAND_MAX - 0.5f) * 0.008f;
        driftState *= 0.998f;
        if (driftState > 0.2f) driftState = 0.2f;
        if (driftState < -0.2f) driftState = -0.2f;
        float driftMod = driftState * drift;
        float totalMod = lfoMod + driftMod;

        float pitchA = cvA;
        float pitchB = cvB;
        float freqA = 261.63f * std::pow(2.0f, pitchA + oscA.fine / 12.0f);
        float freqB = 261.63f * std::pow(2.0f, pitchB + oscB.fine / 12.0f);
        freqB *= std::pow(2.0f, detune / 12.0f);
        oscA.freq = freqA;
        oscB.freq = freqB;

        float sigA = processOscillator(oscA, totalMod);
        float sigB = processOscillator(oscB, totalMod);
        float mixFactor = mix;
        float dry = (1.0f - mixFactor) * sigA + mixFactor * sigB;

        float cutoffHz = 50.0f + filterCutoff * 4000.0f;
        float g = 1.0f - std::exp(-2.0f * float(M_PI) * cutoffHz / sampleRate);
        filterState += g * (dry - filterState);
        float filtered = filterState;

        modPhase += modRate / sampleRate;
        if (modPhase >= 1.0f) modPhase -= 1.0f;
        float modTime = modDepth * (std::sin(modPhase * 2.0f * float(M_PI)) * 0.5f + 0.5f);
        float currentDelaySec = delayTime + modTime * 0.1f;
        int delaySamps = (int)(currentDelaySec * sampleRate);
        if (delaySamps < 1) delaySamps = 1;
        int readIdx = delayIdx - delaySamps;
        if (readIdx < 0) readIdx += delayLength;
        float delayedL = delayBufferL[readIdx];
        float delayedR = delayBufferR[readIdx];

        float diffusedL = delayedL, diffusedR = delayedR;
        if (delayDiffusion > 0.0f)
        {
            float apCoeff = 0.5f * delayDiffusion;
            static float apStateL = 0.0f, apStateR = 0.0f;
            diffusedL = delayedL + apCoeff * apStateL;
            apStateL = delayedL - diffusedL;
            diffusedR = delayedR + apCoeff * apStateR;
            apStateR = delayedR - diffusedR;
        }

        delayBufferL[delayIdx] = filtered + diffusedL * delayFeedback;
        delayBufferR[delayIdx] = filtered + diffusedR * delayFeedback;
        delayIdx = (delayIdx + 1) % delayLength;

        int revSamps = (int)(0.8f * sampleRate);
        int revRead = revIdx - revSamps;
        if (revRead < 0) revRead += revLength;
        float revOut = revBuffer[revRead];
        float revIn = delayedL * 0.5f + delayedR * 0.5f;
        float damped = revOut * (1.0f - revDamping * 0.9f);
        revBuffer[revIdx] = revIn + damped * revDecay;
        revIdx = (revIdx + 1) % revLength;

        static float shimmerPhase = 0.0f;
        shimmerPhase += 440.0f / sampleRate;
        if (shimmerPhase >= 1.0f) shimmerPhase -= 1.0f;
        float shimmered = revOut * std::sin(shimmerPhase * 2.0f * float(M_PI)) * shimmer;

        float wet = delayedL * 0.5f + delayedR * 0.5f + revOut * 0.7f + shimmered * 0.5f;
        wet = wet / (1.0f + washMix);
        static float washState = 0.0f;
        washState += 0.01f * (wet - washState);
        wet = washState * washMix + dry * (1.0f - washMix);

        int echoSamps = (int)(echoTime * sampleRate);
        if (echoSamps < 1) echoSamps = 1;
        int echoRead = echoIdx - echoSamps;
        if (echoRead < 0) echoRead += echoLength;
        float echoL = echoBufferL[echoRead];
        float echoR = echoBufferR[echoRead];
        echoBufferL[echoIdx] = wet + echoL * echoFeedback;
        echoBufferR[echoIdx] = wet + echoR * echoFeedback;
        echoIdx = (echoIdx + 1) % echoLength;
        float echoMixedL = wet * (1.0f - echoMix) + echoL * echoMix;
        float echoMixedR = wet * (1.0f - echoMix) + echoR * echoMix;

        float finalCutoffHz = 50.0f + lpfCutoff * 20000.0f;
        float w0 = 2.0f * float(M_PI) * finalCutoffHz / sampleRate;
        float cosw0 = std::cos(w0);
        float alpha = std::sin(w0) / (2.0f * (1.0f + lpfReso * 10.0f));
        float b0 = (1.0f - cosw0) / 2.0f;
        float b1 = 1.0f - cosw0;
        float b2 = (1.0f - cosw0) / 2.0f;
        float a0 = 1.0f + alpha;
        float a1 = -2.0f * cosw0;
        float a2 = 1.0f - alpha;

        static float lpfX1L = 0, lpfX2L = 0, lpfY1L = 0, lpfY2L = 0;
        static float lpfX1R = 0, lpfX2R = 0, lpfY1R = 0, lpfY2R = 0;

        float inL = echoMixedL;
        float outL = (b0 * inL + b1 * lpfX1L + b2 * lpfX2L - a1 * lpfY1L - a2 * lpfY2L) / a0;
        lpfX2L = lpfX1L; lpfX1L = inL;
        lpfY2L = lpfY1L; lpfY1L = outL;

        float inR = echoMixedR;
        float outR = (b0 * inR + b1 * lpfX1R + b2 * lpfX2R - a1 * lpfY1R - a2 * lpfY2R) / a0;
        lpfX2R = lpfX1R; lpfX1R = inR;
        lpfY2R = lpfY1R; lpfY1R = outR;

        leftOut = std::tanh(outL * 0.8f);
        rightOut = std::tanh(outR * 0.8f);
    }
};

// ============================================================
// Display resolution / analyzer constants
// ============================================================
static const int   LS_SCOPE_N   = 160;     // waveform columns
static const int   LS_FFT_N     = 4096;    // analyzer window (power of 2)
static const int   LS_BARS      = 48;      // bar analyzer bands
static const int   LS_SPEC_PTS  = 232;     // spectrum curve points
static const float LS_DB_MIN    = -80.f;   // bottom of analyzer display
static const float LS_SPEC_FMIN = 30.f;
static const float LS_BAR_FMIN  = 40.f;

// ============================================================
// Module – engine + CV handling + scope tracking
// ============================================================
struct LSDrone : Module
{
    enum ParamId
    {
        PARAM_FINE_A, PARAM_FINE_B, PARAM_DETUNE, PARAM_MIX,
        PARAM_CUTOFF, PARAM_RESO,
        PARAM_LFO_RATE, PARAM_LFO_DEPTH, PARAM_DRIFT,
        PARAM_DELAY_TIME, PARAM_DELAY_FEED, PARAM_DELAY_DIFF,
        PARAM_REV_DECAY, PARAM_REV_DAMP,
        PARAM_WASH, PARAM_SHIMMER,
        PARAM_MOD_RATE, PARAM_MOD_DEPTH,
        PARAM_ECHO_TIME, PARAM_ECHO_FEED, PARAM_ECHO_MIX,
        PARAM_LPF_CUTOFF, PARAM_LPF_RESO,
        PARAM_ATTEN_FINE_A, PARAM_ATTEN_FINE_B, PARAM_ATTEN_DETUNE, PARAM_ATTEN_MIX,
        PARAM_ATTEN_CUTOFF, PARAM_ATTEN_RESO,
        PARAM_ATTEN_LFO_RATE, PARAM_ATTEN_LFO_DEPTH, PARAM_ATTEN_DRIFT,
        PARAM_ATTEN_DELAY_TIME, PARAM_ATTEN_DELAY_FEED, PARAM_ATTEN_DELAY_DIFF,
        PARAM_ATTEN_REV_DECAY, PARAM_ATTEN_REV_DAMP,
        PARAM_ATTEN_WASH, PARAM_ATTEN_SHIMMER,
        PARAM_ATTEN_MOD_RATE, PARAM_ATTEN_MOD_DEPTH,
        PARAM_ATTEN_ECHO_TIME, PARAM_ATTEN_ECHO_FEED, PARAM_ATTEN_ECHO_MIX,
        PARAM_ATTEN_LPF_CUTOFF, PARAM_ATTEN_LPF_RESO,
        PARAMS_LEN
    };
    enum InputId
    {
        CV_FINE_A, CV_FINE_B, CV_DETUNE, CV_MIX,
        CV_CUTOFF, CV_RESO,
        CV_LFO_RATE, CV_LFO_DEPTH, CV_DRIFT,
        CV_DELAY_TIME, CV_DELAY_FEED, CV_DELAY_DIFF,
        CV_REV_DECAY, CV_REV_DAMP,
        CV_WASH, CV_SHIMMER,
        CV_MOD_RATE, CV_MOD_DEPTH,
        CV_ECHO_TIME, CV_ECHO_FEED, CV_ECHO_MIX,
        CV_LPF_CUTOFF, CV_LPF_RESO,
        CV_A, CV_B,
        INPUTS_LEN
    };
    enum OutputId { OUTPUT_LEFT, OUTPUT_RIGHT, OUTPUTS_LEN };
    enum LightId  { LIGHTS_LEN };

    enum CVRange { BIPOLAR_M5_5, UNIPOLAR_0_10 };
    CVRange cvRangeMode = BIPOLAR_M5_5;

    enum ViewMode { VIEW_WAVE, VIEW_BARS, VIEW_SPECTRUM };
    ViewMode viewMode = VIEW_WAVE;

    LSDroneEngine engine;

    // Scope / meter state (written by audio thread, read by UI)
    float scopeMin[LS_SCOPE_N] = {};
    float scopeMax[LS_SCOPE_N] = {};
    int   scopeHead = 0;
    int   scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    // Analyzer ring buffer (mono, normalised to +/-1)
    float fftBuf[LS_FFT_N] = {};
    int   fftHead = 0;
    float dispSampleRate = 44100.f;

    LSDrone()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(PARAM_FINE_A, -12.f, 12.f, 0.f, "Fine A", " st");
        configParam(PARAM_FINE_B, -12.f, 12.f, 0.f, "Fine B", " st");
        configParam(PARAM_DETUNE, -24.f, 24.f, 0.f, "Detune", " st");
        configParam(PARAM_MIX, 0.f, 1.f, 0.5f, "Mix");
        configParam(PARAM_CUTOFF, 0.f, 1.f, 0.5f, "Cutoff");
        configParam(PARAM_RESO, 0.f, 1.f, 0.3f, "Resonance");
        configParam(PARAM_LFO_RATE, 0.01f, 5.f, 0.2f, "LFO Rate", " Hz");
        configParam(PARAM_LFO_DEPTH, 0.f, 2.f, 0.5f, "LFO Depth", " st");
        configParam(PARAM_DRIFT, 0.f, 1.f, 0.3f, "Drift");
        configParam(PARAM_DELAY_TIME, 0.1f, 2.5f, 0.8f, "Delay Time", " s");
        configParam(PARAM_DELAY_FEED, 0.f, 0.98f, 0.6f, "Delay Feedback");
        configParam(PARAM_DELAY_DIFF, 0.f, 1.f, 0.4f, "Delay Diffusion");
        configParam(PARAM_REV_DECAY, 0.5f, 0.99f, 0.85f, "Reverb Decay");
        configParam(PARAM_REV_DAMP, 0.f, 1.f, 0.5f, "Reverb Damping");
        configParam(PARAM_WASH, 0.f, 1.f, 0.6f, "Wash");
        configParam(PARAM_SHIMMER, 0.f, 1.f, 0.2f, "Shimmer");
        configParam(PARAM_MOD_RATE, 0.02f, 2.f, 0.15f, "Mod Rate", " Hz");
        configParam(PARAM_MOD_DEPTH, 0.f, 1.f, 0.3f, "Mod Depth");
        configParam(PARAM_ECHO_TIME, 0.05f, 1.5f, 0.4f, "Echo Time", " s");
        configParam(PARAM_ECHO_FEED, 0.f, 0.95f, 0.5f, "Echo Feedback");
        configParam(PARAM_ECHO_MIX, 0.f, 1.f, 0.4f, "Echo Mix");
        configParam(PARAM_LPF_CUTOFF, 0.f, 1.f, 0.8f, "LPF Cutoff");
        configParam(PARAM_LPF_RESO, 0.f, 1.f, 0.2f, "LPF Resonance");

        auto atten = [this](ParamId id, float def = 0.5f)
        { configParam(id, -1.f, 1.f, def, "Atten"); };
        atten(PARAM_ATTEN_FINE_A);
        atten(PARAM_ATTEN_FINE_B);
        atten(PARAM_ATTEN_DETUNE);
        atten(PARAM_ATTEN_MIX);
        atten(PARAM_ATTEN_CUTOFF);
        atten(PARAM_ATTEN_RESO);
        atten(PARAM_ATTEN_LFO_RATE);
        atten(PARAM_ATTEN_LFO_DEPTH);
        atten(PARAM_ATTEN_DRIFT);
        atten(PARAM_ATTEN_DELAY_TIME);
        atten(PARAM_ATTEN_DELAY_FEED);
        atten(PARAM_ATTEN_DELAY_DIFF);
        atten(PARAM_ATTEN_REV_DECAY);
        atten(PARAM_ATTEN_REV_DAMP);
        atten(PARAM_ATTEN_WASH);
        atten(PARAM_ATTEN_SHIMMER);
        atten(PARAM_ATTEN_MOD_RATE);
        atten(PARAM_ATTEN_MOD_DEPTH);
        atten(PARAM_ATTEN_ECHO_TIME);
        atten(PARAM_ATTEN_ECHO_FEED);
        atten(PARAM_ATTEN_ECHO_MIX);
        atten(PARAM_ATTEN_LPF_CUTOFF);
        atten(PARAM_ATTEN_LPF_RESO);

        configInput(CV_A, "V/oct A");
        configInput(CV_B, "V/oct B");
        configOutput(OUTPUT_LEFT, "L out");
        configOutput(OUTPUT_RIGHT, "R out");
    }

    void onSampleRateChange() override
    {
        engine.init(APP->engine->getSampleRate());
    }

    float getParamWithCV(ParamId paramId, InputId cvId, ParamId attenId, float minV, float maxV)
    {
        float knob = params[paramId].getValue();
        float cv = inputs[cvId].getVoltage();
        float atten = params[attenId].getValue();

        float cvScaled;
        if (cvRangeMode == UNIPOLAR_0_10)
            cvScaled = cv / 10.0f;
        else
            cvScaled = clamp(cv / 5.0f, -1.0f, 1.0f);

        float value = knob + cvScaled * atten;
        return clamp(value, minV, maxV);
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        json_object_set_new(root, "viewMode", json_integer(viewMode));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *mode = json_object_get(root, "cvRangeMode");
        if (mode)
            cvRangeMode = (CVRange)json_integer_value(mode);

        json_t *view = json_object_get(root, "viewMode");
        if (view) {
            int v = (int)json_integer_value(view);
            if (v < 0) v = 0;
            if (v > 2) v = 2;
            viewMode = (ViewMode)v;
        }
    }

    void process(const ProcessArgs &args) override
    {
        engine.setFineA(getParamWithCV(PARAM_FINE_A, CV_FINE_A, PARAM_ATTEN_FINE_A, -12, 12));
        engine.setFineB(getParamWithCV(PARAM_FINE_B, CV_FINE_B, PARAM_ATTEN_FINE_B, -12, 12));
        engine.setDetune(getParamWithCV(PARAM_DETUNE, CV_DETUNE, PARAM_ATTEN_DETUNE, -24, 24));
        engine.setMix(getParamWithCV(PARAM_MIX, CV_MIX, PARAM_ATTEN_MIX, 0, 1));
        engine.setFilterCutoff(getParamWithCV(PARAM_CUTOFF, CV_CUTOFF, PARAM_ATTEN_CUTOFF, 0, 1));
        engine.setFilterReso(getParamWithCV(PARAM_RESO, CV_RESO, PARAM_ATTEN_RESO, 0, 1));
        engine.setLfoRate(getParamWithCV(PARAM_LFO_RATE, CV_LFO_RATE, PARAM_ATTEN_LFO_RATE, 0.01f, 5));
        engine.setLfoDepth(getParamWithCV(PARAM_LFO_DEPTH, CV_LFO_DEPTH, PARAM_ATTEN_LFO_DEPTH, 0, 2));
        engine.setDrift(getParamWithCV(PARAM_DRIFT, CV_DRIFT, PARAM_ATTEN_DRIFT, 0, 1));
        engine.setDelayTime(getParamWithCV(PARAM_DELAY_TIME, CV_DELAY_TIME, PARAM_ATTEN_DELAY_TIME, 0.1f, 2.5f));
        engine.setDelayFeedback(getParamWithCV(PARAM_DELAY_FEED, CV_DELAY_FEED, PARAM_ATTEN_DELAY_FEED, 0, 0.98f));
        engine.setDelayDiffusion(getParamWithCV(PARAM_DELAY_DIFF, CV_DELAY_DIFF, PARAM_ATTEN_DELAY_DIFF, 0, 1));
        engine.setRevDecay(getParamWithCV(PARAM_REV_DECAY, CV_REV_DECAY, PARAM_ATTEN_REV_DECAY, 0.5f, 0.99f));
        engine.setRevDamping(getParamWithCV(PARAM_REV_DAMP, CV_REV_DAMP, PARAM_ATTEN_REV_DAMP, 0, 1));
        engine.setWash(getParamWithCV(PARAM_WASH, CV_WASH, PARAM_ATTEN_WASH, 0, 1));
        engine.setShimmer(getParamWithCV(PARAM_SHIMMER, CV_SHIMMER, PARAM_ATTEN_SHIMMER, 0, 1));
        engine.setModRate(getParamWithCV(PARAM_MOD_RATE, CV_MOD_RATE, PARAM_ATTEN_MOD_RATE, 0.02f, 2));
        engine.setModDepth(getParamWithCV(PARAM_MOD_DEPTH, CV_MOD_DEPTH, PARAM_ATTEN_MOD_DEPTH, 0, 1));
        engine.setEchoTime(getParamWithCV(PARAM_ECHO_TIME, CV_ECHO_TIME, PARAM_ATTEN_ECHO_TIME, 0.05f, 1.5f));
        engine.setEchoFeedback(getParamWithCV(PARAM_ECHO_FEED, CV_ECHO_FEED, PARAM_ATTEN_ECHO_FEED, 0, 0.95f));
        engine.setEchoMix(getParamWithCV(PARAM_ECHO_MIX, CV_ECHO_MIX, PARAM_ATTEN_ECHO_MIX, 0, 1));
        engine.setLpfCutoff(getParamWithCV(PARAM_LPF_CUTOFF, CV_LPF_CUTOFF, PARAM_ATTEN_LPF_CUTOFF, 0, 1));
        engine.setLpfReso(getParamWithCV(PARAM_LPF_RESO, CV_LPF_RESO, PARAM_ATTEN_LPF_RESO, 0, 1));

        float cvA = inputs[CV_A].getVoltage();
        float cvB = inputs[CV_B].getVoltage();
        float leftOut = 0.f, rightOut = 0.f;
        engine.process(cvA, cvB, leftOut, rightOut);

        float l = leftOut * 5.f;
        float r = rightOut * 5.f;
        outputs[OUTPUT_LEFT].setVoltage(l);
        outputs[OUTPUT_RIGHT].setVoltage(r);

        // ---- display data ----
        float decay = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(l), peakL * decay);
        peakR = std::max(std::fabs(r), peakR * decay);

        float mono = 0.5f * (l + r);

        // analyzer ring buffer
        dispSampleRate = args.sampleRate;
        fftBuf[fftHead] = mono * 0.2f;
        fftHead = (fftHead + 1) & (LS_FFT_N - 1);

        if (scopeCount == 0) scopeBMin = scopeBMax = mono;
        else {
            scopeBMin = std::min(scopeBMin, mono);
            scopeBMax = std::max(scopeBMax, mono);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket) {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % LS_SCOPE_N;
        }
    }
};

// ============================================================
// Theme + panel helpers (anonymous namespace – no link clashes)
// ============================================================
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colAccent2()  { return nvgRGB(0xb0, 0x5a, 0xff); }

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
// Small FFT analyzer used by the BARS / SPECTRUM display modes
// ------------------------------------------------------------
struct LSAnalyzer {
    float re[LS_FFT_N];
    float im[LS_FFT_N];
    float win[LS_FFT_N];
    float cosT[LS_FFT_N / 2];
    float sinT[LS_FFT_N / 2];
    float db[LS_FFT_N / 2];

    LSAnalyzer() {
        for (int n = 0; n < LS_FFT_N; ++n) {
            win[n] = 0.5f * (1.f - std::cos(2.f * float(M_PI) * n / (LS_FFT_N - 1)));
            re[n] = 0.f;
            im[n] = 0.f;
        }
        for (int k = 0; k < LS_FFT_N / 2; ++k) {
            cosT[k] = std::cos(2.f * float(M_PI) * k / LS_FFT_N);
            sinT[k] = -std::sin(2.f * float(M_PI) * k / LS_FFT_N);
            db[k] = -90.f;
        }
    }

    void update(const float* ring, int head) {
        const int N = LS_FFT_N;
        for (int n = 0; n < N; ++n) {
            int idx = (head + n) & (N - 1);   // oldest -> newest
            re[n] = ring[idx] * win[n];
            im[n] = 0.f;
        }

        // bit reversal
        for (int i = 1, j = 0; i < N; ++i) {
            int bit = N >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) {
                std::swap(re[i], re[j]);
                std::swap(im[i], im[j]);
            }
        }
        // butterflies
        for (int len = 2; len <= N; len <<= 1) {
            int half = len >> 1;
            int stride = N / len;
            for (int i = 0; i < N; i += len) {
                for (int j = 0; j < half; ++j) {
                    float wr = cosT[j * stride];
                    float wi = sinT[j * stride];
                    int a = i + j, b = i + j + half;
                    float vr = re[b] * wr - im[b] * wi;
                    float vi = re[b] * wi + im[b] * wr;
                    re[b] = re[a] - vr;
                    im[b] = im[a] - vi;
                    re[a] += vr;
                    im[a] += vi;
                }
            }
        }

        // magnitude -> dB (0 dB = full-scale sine), with fast attack / slow release
        const float norm = 4.f / (float) N;
        for (int k = 0; k < N / 2; ++k) {
            float m = std::sqrt(re[k] * re[k] + im[k] * im[k]) * norm;
            float d = 20.f * std::log10(m + 1e-7f);
            if (d < -90.f) d = -90.f;
            if (d > db[k]) db[k] = d;
            else db[k] += (d - db[k]) * 0.18f;
        }
    }

    float dbAt(float pos) const {
        if (pos < 0.f) pos = 0.f;
        float maxPos = (float)(LS_FFT_N / 2 - 2);
        if (pos > maxPos) pos = maxPos;
        int k = (int) pos;
        float t = pos - (float) k;
        return db[k] * (1.f - t) + db[k + 1] * t;
    }

    // level of a frequency band; interpolates for narrow bands, takes the max for wide ones
    float bandDb(float f0, float f1, float binHz) const {
        float b0 = f0 / binHz, b1 = f1 / binHz;
        if (b1 - b0 <= 1.f)
            return dbAt(0.5f * (b0 + b1));
        int k0 = std::max(1, (int) std::ceil(b0));
        int k1 = std::min(LS_FFT_N / 2 - 1, (int) std::floor(b1));
        float m = -90.f;
        for (int k = k0; k <= k1; ++k)
            m = std::max(m, db[k]);
        return m;
    }
};

float dbToNorm(float d) {
    float n = (d - LS_DB_MIN) / (0.f - LS_DB_MIN);
    return std::min(std::max(n, 0.f), 1.f);
}

} // namespace

// ============================================================
// Layout constants  (36 HP panel)
// ============================================================
static const float LS_PANEL_W = RACK_GRID_WIDTH * 36;   // 540 (36 HP)
static const float LS_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float LS_DISP_W = 480.f;
static const float LS_DISP_H = 38.f;
static const float LS_DISP_X = (LS_PANEL_W - LS_DISP_W) / 2.f;
static const float LS_DISP_Y = 46.f;

static const float LS_CELL_W   = 86.f;
static const float LS_CELL_H   = 54.f;
static const float LS_CELL_TOP = 23.f;                  // plate top = row - 23
static const float LS_ROW[4]   = {120.f, 178.f, 236.f, 294.f};
static const float LS_PORT_Y   = 358.f;

static const float LS_GAUGE_R   = 18.f;                 // gauge ring radius
static const float LS_JACK_DX   = 30.f;                 // CV jack / attenuverter offset
static const float LS_JACK_DY   = 9.f;

static const float LS_VIEW_W = 150.f;                   // display-mode switch width

static float lsColX(int col) { return LS_PANEL_W / 12.f * (2 * col + 1); }
// 540/12=45 → 45, 135, 225, 315, 405, 495

// ============================================================
// Control table (label + ids)
// ============================================================
namespace {
struct LSCellDef { const char* name; int param, attn, cv; };
}

static const LSCellDef LS_CELLS[23] = {
    {"FINE A",    LSDrone::PARAM_FINE_A,     LSDrone::PARAM_ATTEN_FINE_A,     LSDrone::CV_FINE_A},
    {"FINE B",    LSDrone::PARAM_FINE_B,     LSDrone::PARAM_ATTEN_FINE_B,     LSDrone::CV_FINE_B},
    {"DETUNE",    LSDrone::PARAM_DETUNE,     LSDrone::PARAM_ATTEN_DETUNE,     LSDrone::CV_DETUNE},
    {"MIX",       LSDrone::PARAM_MIX,        LSDrone::PARAM_ATTEN_MIX,        LSDrone::CV_MIX},
    {"CUTOFF",    LSDrone::PARAM_CUTOFF,     LSDrone::PARAM_ATTEN_CUTOFF,     LSDrone::CV_CUTOFF},
    {"RESO",      LSDrone::PARAM_RESO,       LSDrone::PARAM_ATTEN_RESO,       LSDrone::CV_RESO},
    {"LFO RATE",  LSDrone::PARAM_LFO_RATE,   LSDrone::PARAM_ATTEN_LFO_RATE,   LSDrone::CV_LFO_RATE},
    {"LFO DEPTH", LSDrone::PARAM_LFO_DEPTH,  LSDrone::PARAM_ATTEN_LFO_DEPTH,  LSDrone::CV_LFO_DEPTH},
    {"DRIFT",     LSDrone::PARAM_DRIFT,      LSDrone::PARAM_ATTEN_DRIFT,      LSDrone::CV_DRIFT},
    {"DELAY T",   LSDrone::PARAM_DELAY_TIME, LSDrone::PARAM_ATTEN_DELAY_TIME, LSDrone::CV_DELAY_TIME},
    {"DELAY FB",  LSDrone::PARAM_DELAY_FEED, LSDrone::PARAM_ATTEN_DELAY_FEED, LSDrone::CV_DELAY_FEED},
    {"DELAY DIF", LSDrone::PARAM_DELAY_DIFF, LSDrone::PARAM_ATTEN_DELAY_DIFF, LSDrone::CV_DELAY_DIFF},
    {"REV DECAY", LSDrone::PARAM_REV_DECAY,  LSDrone::PARAM_ATTEN_REV_DECAY,  LSDrone::CV_REV_DECAY},
    {"REV DAMP",  LSDrone::PARAM_REV_DAMP,   LSDrone::PARAM_ATTEN_REV_DAMP,   LSDrone::CV_REV_DAMP},
    {"WASH",      LSDrone::PARAM_WASH,       LSDrone::PARAM_ATTEN_WASH,       LSDrone::CV_WASH},
    {"SHIMMER",   LSDrone::PARAM_SHIMMER,    LSDrone::PARAM_ATTEN_SHIMMER,    LSDrone::CV_SHIMMER},
    {"MOD RATE",  LSDrone::PARAM_MOD_RATE,   LSDrone::PARAM_ATTEN_MOD_RATE,   LSDrone::CV_MOD_RATE},
    {"MOD DEPTH", LSDrone::PARAM_MOD_DEPTH,  LSDrone::PARAM_ATTEN_MOD_DEPTH,  LSDrone::CV_MOD_DEPTH},
    {"ECHO T",    LSDrone::PARAM_ECHO_TIME,  LSDrone::PARAM_ATTEN_ECHO_TIME,  LSDrone::CV_ECHO_TIME},
    {"ECHO FB",   LSDrone::PARAM_ECHO_FEED,  LSDrone::PARAM_ATTEN_ECHO_FEED,  LSDrone::CV_ECHO_FEED},
    {"ECHO MIX",  LSDrone::PARAM_ECHO_MIX,   LSDrone::PARAM_ATTEN_ECHO_MIX,   LSDrone::CV_ECHO_MIX},
    {"LPF CUT",   LSDrone::PARAM_LPF_CUTOFF, LSDrone::PARAM_ATTEN_LPF_CUTOFF, LSDrone::CV_LPF_CUTOFF},
    {"LPF RESO",  LSDrone::PARAM_LPF_RESO,   LSDrone::PARAM_ATTEN_LPF_RESO,   LSDrone::CV_LPF_RESO},
};

// ============================================================
// Knob with a gauge ring around it
// ============================================================
struct LSGaugeKnob : RoundSmallBlackKnob {
    struct Geo {
        float a0, a1;     // full track (nvg angles)
        float from, head; // value arc start / end
        bool bipolar;
    };

    Geo geo() {
        float t = 0.5f, tz = 0.f;
        bool bip = false;
        engine::ParamQuantity* pq = getParamQuantity();
        if (pq) {
            float mn = pq->getMinValue(), mx = pq->getMaxValue();
            if (mx > mn) {
                t = clamp((pq->getValue() - mn) / (mx - mn), 0.f, 1.f);
                if (mn < 0.f && mx > 0.f) {
                    bip = true;
                    tz = (0.f - mn) / (mx - mn);
                }
            }
        }
        Geo g;
        g.a0 = minAngle - float(M_PI) / 2.f;
        g.a1 = maxAngle - float(M_PI) / 2.f;
        g.head = g.a0 + (g.a1 - g.a0) * t;
        g.from = bip ? g.a0 + (g.a1 - g.a0) * tz : g.a0;
        g.bipolar = bip;
        return g;
    }

    // layer 0: dark track (+ zero mark for bipolar knobs)
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        float cx = box.size.x / 2.f, cy = box.size.y / 2.f;
        Geo g = geo();

        nvgSave(vg);
        nvgLineCap(vg, NVG_ROUND);
        nvgBeginPath(vg);
        nvgArc(vg, cx, cy, LS_GAUGE_R, g.a0, g.a1, NVG_CW);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 26));
        nvgStrokeWidth(vg, 2.5f);
        nvgStroke(vg);

        if (g.bipolar) {
            nvgBeginPath(vg);
            nvgCircle(vg, cx + LS_GAUGE_R * std::cos(g.from), cy + LS_GAUGE_R * std::sin(g.from), 1.3f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 90));
            nvgFill(vg);
        }
        nvgRestore(vg);

        RoundSmallBlackKnob::draw(args);
    }

    // layer 1: glowing value arc + head dot
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer == 1) {
            NVGcontext* vg = args.vg;
            float cx = box.size.x / 2.f, cy = box.size.y / 2.f;
            Geo g = geo();
            float lo = std::min(g.from, g.head);
            float hi = std::max(g.from, g.head);

            nvgSave(vg);
            nvgLineCap(vg, NVG_ROUND);
            if (hi - lo > 0.002f) {
                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, LS_GAUGE_R, lo, hi, NVG_CW);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 45));
                nvgStrokeWidth(vg, 5.5f);
                nvgStroke(vg);

                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, LS_GAUGE_R, lo, hi, NVG_CW);
                nvgStrokeColor(vg, colAccent());
                nvgStrokeWidth(vg, 2.5f);
                nvgStroke(vg);
            }
            nvgBeginPath(vg);
            nvgCircle(vg, cx + LS_GAUGE_R * std::cos(g.head), cy + LS_GAUGE_R * std::sin(g.head), 2.1f);
            nvgFillColor(vg, nvgRGB(0xe8, 0xfb, 0xff));
            nvgFill(vg);
            nvgRestore(vg);
        }
        RoundSmallBlackKnob::drawLayer(args, layer);
    }
};
 // namespace

// ============================================================
// Panel background
// ============================================================
struct LSPanelBackground : TransparentWidget {
    LSPanelBackground() { box.size = Vec(LS_PANEL_W, LS_PANEL_H); }

    void plate(NVGcontext* vg, float cx, float ry) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - LS_CELL_W / 2.f, ry - LS_CELL_TOP, LS_CELL_W, LS_CELL_H, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = LS_PANEL_W - 14.f, cx = LS_PANEL_W / 2.f;
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

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, LS_PANEL_W, LS_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, LS_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) LS_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, LS_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, LS_PANEL_W - 1.f, LS_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, LS_PANEL_W / 2.f, 13.f);
        drawText(vg, LS_PANEL_W / 2.f, 31.f, "LS DRONE", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = LS_DISP_X - 5.f, by = LS_DISP_Y - 5.f;
        float bw = LS_DISP_W + 10.f, bh = LS_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // Control cells (23 cells + CV range cell)
        for (int i = 0; i < 24; ++i) {
            int row = i / 6, col = i % 6;
            float cx = lsColX(col), ry = LS_ROW[row];
            plate(vg, cx, ry);
            if (i < 23)
                drawText(vg, cx, ry + 25.f, LS_CELLS[i].name, 7.5f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // I/O section
        section(vg, 332.f, "I / O");
        const char* io[4] = {"V/OCT A", "V/OCT B", "L OUT", "R OUT"};
        int   ioCols[4]  = {0, 1, 4, 5};
        for (int i = 0; i < 4; ++i) {
            float cx = lsColX(ioCols[i]);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 22.f, 340.f, 44.f, 40.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, cx, 348.f, io[i], 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        }

        // Display-mode switch plate (centre of I/O row)
        {
            float cx = LS_PANEL_W / 2.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - LS_VIEW_W / 2.f, 340.f, LS_VIEW_W, 40.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, cx, 348.f, "DISPLAY", 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        }

        Widget::draw(args);
    }
};

// ============================================================
// Scope display widget (WAVE / BARS / SPECTRUM)
// ============================================================
struct LSDisplayWidget : TransparentWidget {
    LSDrone* module = nullptr;
    std::unique_ptr<LSAnalyzer> an;
    float barLvl[LS_BARS] = {};
    float barPeak[LS_BARS] = {};

    LSDisplayWidget(LSDrone* m) : module(m) {
        box.pos = Vec(LS_DISP_X, LS_DISP_Y);
        box.size = Vec(LS_DISP_W, LS_DISP_H);
        an.reset(new LSAnalyzer());
    }

    static float scopeW() { return LS_DISP_W - 16.f; }

    int mode() const { return module ? (int) module->viewMode : (int) LSDrone::VIEW_WAVE; }
    float sampleRate() const { return (module && module->dispSampleRate > 1000.f) ? module->dispSampleRate : 48000.f; }
    float fMax() const { return std::min(18000.f, sampleRate() * 0.45f); }
    float binHz() const { return sampleRate() / (float) LS_FFT_N; }

    float freqToX(float f, float fmin) const {
        float fmax = fMax();
        return scopeW() * std::log(f / fmin) / std::log(fmax / fmin);
    }

    void step() override {
        if (module && module->viewMode != LSDrone::VIEW_WAVE) {
            an->update(module->fftBuf, module->fftHead);

            if (module->viewMode == LSDrone::VIEW_BARS) {
                float ratio = fMax() / LS_BAR_FMIN;
                for (int i = 0; i < LS_BARS; ++i) {
                    float f0 = LS_BAR_FMIN * std::pow(ratio, (float) i / LS_BARS);
                    float f1 = LS_BAR_FMIN * std::pow(ratio, (float) (i + 1) / LS_BARS);
                    float n = dbToNorm(an->bandDb(f0, f1, binHz()));
                    barLvl[i] = n;
                    barPeak[i] = std::max(n, barPeak[i] - 0.012f);
                }
            }
        }
        Widget::step();
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float sw = scopeW();

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, LS_DISP_W, LS_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, LS_DISP_W / 2.f, LS_DISP_H / 2.f, 20.f, LS_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        nvgBeginPath(vg);
        if (mode() == LSDrone::VIEW_WAVE) {
            for (float x = 0.f; x <= sw; x += 28.f) {
                nvgMoveTo(vg, x + 0.5f, 0);
                nvgLineTo(vg, x + 0.5f, LS_DISP_H);
            }
            nvgMoveTo(vg, 0, LS_DISP_H / 2.f + 0.5f);
            nvgLineTo(vg, sw, LS_DISP_H / 2.f + 0.5f);
        } else {
            // horizontal dB lines
            for (int i = 1; i < 4; ++i) {
                float y = LS_DISP_H * i / 4.f + 0.5f;
                nvgMoveTo(vg, 0, y);
                nvgLineTo(vg, sw, y);
            }
            // log-frequency lines (spectrum only)
            if (mode() == LSDrone::VIEW_SPECTRUM) {
                const float fl[3] = {100.f, 1000.f, 10000.f};
                for (int i = 0; i < 3; ++i) {
                    float x = freqToX(fl[i], LS_SPEC_FMIN) + 0.5f;
                    nvgMoveTo(vg, x, 0);
                    nvgLineTo(vg, x, LS_DISP_H);
                }
            }
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        Widget::draw(args);
    }

    // ---------- waveform ----------
    void drawWave(NVGcontext* vg) {
        const float mid = LS_DISP_H / 2.f;
        const float amp = mid - 2.f;
        const float sw = scopeW();
        const float colW = sw / (float) LS_SCOPE_N;

        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < LS_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % LS_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / LS_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - std::min(std::max(vmax / 5.f, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / 5.f, -1.f), 1.f) * amp;
                float h = std::max(y1 - y0, 1.f);
                float x = i * colW;
                if (pass == 0)
                    nvgRect(vg, x - 0.5f, y0 - 1.5f, colW + 1.f, h + 3.f);
                else
                    nvgRect(vg, x, y0, std::max(colW - 0.4f, 1.f), h);
            }
            nvgFillColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 32) : nvgRGB(0x8a, 0xee, 0xff));
            nvgFill(vg);
        }
    }

    // ---------- bars ----------
    void drawBars(NVGcontext* vg) {
        const float sw = scopeW();
        const float colW = sw / (float) LS_BARS;
        const float base = LS_DISP_H - 1.f;
        const float span = LS_DISP_H - 3.f;

        // glow
        nvgBeginPath(vg);
        for (int i = 0; i < LS_BARS; ++i) {
            float h = barLvl[i] * span;
            if (h < 0.5f) continue;
            nvgRect(vg, i * colW, base - h - 1.f, colW, h + 2.f);
        }
        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 28));
        nvgFill(vg);

        // bars
        nvgBeginPath(vg);
        for (int i = 0; i < LS_BARS; ++i) {
            float h = barLvl[i] * span;
            if (h < 0.5f) continue;
            nvgRect(vg, i * colW + 1.f, base - h, std::max(colW - 2.f, 1.f), h);
        }
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, LS_DISP_H, 0, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // peak holds
        nvgBeginPath(vg);
        for (int i = 0; i < LS_BARS; ++i) {
            if (barPeak[i] < 0.02f) continue;
            nvgRect(vg, i * colW + 1.f, base - barPeak[i] * span - 1.5f, std::max(colW - 2.f, 1.f), 1.5f);
        }
        nvgFillColor(vg, nvgRGB(0x8a, 0xee, 0xff));
        nvgFill(vg);
    }

    // ---------- spectrum ----------
    void drawSpectrum(NVGcontext* vg) {
        const float sw = scopeW();
        const float base = LS_DISP_H - 1.f;
        const float span = LS_DISP_H - 3.f;
        const float ratio = fMax() / LS_SPEC_FMIN;

        float ys[LS_SPEC_PTS + 1];
        for (int i = 0; i <= LS_SPEC_PTS; ++i) {
            float f0 = LS_SPEC_FMIN * std::pow(ratio, (float) i / LS_SPEC_PTS);
            float f1 = LS_SPEC_FMIN * std::pow(ratio, (float) (i + 1) / LS_SPEC_PTS);
            ys[i] = base - dbToNorm(an->bandDb(f0, f1, binHz())) * span;
        }

        // filled area
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0, LS_DISP_H);
        for (int i = 0; i <= LS_SPEC_PTS; ++i)
            nvgLineTo(vg, sw * i / LS_SPEC_PTS, ys[i]);
        nvgLineTo(vg, sw, LS_DISP_H);
        nvgClosePath(vg);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, LS_DISP_H,
                                           nvgRGBA(0x3d, 0xe0, 0xff, 110), nvgRGBA(0x3d, 0xe0, 0xff, 8)));
        nvgFill(vg);

        // line (glow + core)
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i <= LS_SPEC_PTS; ++i) {
                float x = sw * i / LS_SPEC_PTS;
                if (i == 0) nvgMoveTo(vg, x, ys[i]);
                else        nvgLineTo(vg, x, ys[i]);
            }
            nvgLineJoin(vg, NVG_ROUND);
            nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 40) : nvgRGB(0x8a, 0xee, 0xff));
            nvgStrokeWidth(vg, pass == 0 ? 3.f : 1.2f);
            nvgStroke(vg);
        }
    }

    void drawFreqLabels(NVGcontext* vg) {
        const float fmin = (mode() == LSDrone::VIEW_BARS) ? LS_BAR_FMIN : LS_SPEC_FMIN;
        const char* names[3] = {"100", "1K", "10K"};
        const float fl[3] = {100.f, 1000.f, 10000.f};
        for (int i = 0; i < 3; ++i) {
            float x = freqToX(fl[i], fmin);
            drawText(vg, x + 2.f, LS_DISP_H - 5.f, names[i], 6.f, nvgRGBA(0x7a, 0x85, 0x94, 170),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.5f);
        }
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float sw = scopeW();

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, LS_DISP_H);
        switch (mode()) {
            case LSDrone::VIEW_BARS:     drawBars(vg);     break;
            case LSDrone::VIEW_SPECTRUM: drawSpectrum(vg); break;
            default:                     drawWave(vg);     break;
        }
        nvgResetScissor(vg);
        nvgRestore(vg);

        if (mode() != LSDrone::VIEW_WAVE)
            drawFreqLabels(vg);

        // Level meters (L/R)
        for (int ch = 0; ch < 2; ++ch) {
            float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 3.f : 2.4f);
            float lvl = std::min(peak / 10.f, 1.f);
            float x = sw + 3.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 1.f, 4.f, LS_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (LS_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, LS_DISP_H - 1.f - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, LS_DISP_H, 0, 0,
                                                   nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) LS_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, LS_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, LS_DISP_W, LS_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, LS_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[32];
        float dly = module ? module->params[LSDrone::PARAM_DELAY_TIME].getValue() : 0.8f;
        float wash = module ? module->params[LSDrone::PARAM_WASH].getValue() : 0.6f;
        snprintf(buf, sizeof(buf), "DLY %4.0f MS", dly * 1000.f);
        drawText(vg, 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "WASH %3.0f%%", wash * 100.f);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

// ============================================================
// CV range cell (click to toggle)
// ============================================================
struct LSCvRangeWidget : TransparentWidget {
    LSDrone* module = nullptr;

    LSCvRangeWidget(LSDrone* m, float cx, float ry) : module(m) {
        box.pos = Vec(cx - LS_CELL_W / 2.f, ry - LS_CELL_TOP);
        box.size = Vec(LS_CELL_W, LS_CELL_H);
    }

    void onButton(const ButtonEvent& e) override {
        if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            module->cvRangeMode = (module->cvRangeMode == LSDrone::BIPOLAR_M5_5)
                                      ? LSDrone::UNIPOLAR_0_10
                                      : LSDrone::BIPOLAR_M5_5;
            e.consume(this);
        }
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        bool bipolar = module ? (module->cvRangeMode == LSDrone::BIPOLAR_M5_5) : true;

        drawText(vg, box.size.x / 2.f, LS_CELL_H - 6.f, "CV RANGE", 7.5f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        float sx = 8.f, sy = 6.f, sw = box.size.x - 16.f, sh = 20.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, sx, sy, sw, sh, 4.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        drawText(vg, box.size.x / 2.f, sy + sh / 2.f, bipolar ? "-5/+5V" : "0/+10V", 10.f,
                 colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, sx + 4.f, 31.f, sw - 8.f, 2.f, 1.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgFill(vg);
        nvgBeginPath(vg);
        if (bipolar)
            nvgRoundedRect(vg, box.size.x / 2.f - 12.f, 31.f, 24.f, 2.f, 1.f);
        else
            nvgRoundedRect(vg, sx + 4.f, 31.f, 24.f, 2.f, 1.f);
        nvgFillColor(vg, colAccent2());
        nvgFill(vg);

        drawText(vg, box.size.x / 2.f, 39.f, "CLICK", 7.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

// ============================================================
// Display-mode switch (WAVE | BARS | SPEC) – click a segment
// ============================================================
struct LSViewWidget : TransparentWidget {
    LSDrone* module = nullptr;

    LSViewWidget(LSDrone* m, float cx) : module(m) {
        box.pos = Vec(cx - LS_VIEW_W / 2.f, 340.f);
        box.size = Vec(LS_VIEW_W, 40.f);
    }

    static float segX() { return 8.f; }
    static float segW() { return (LS_VIEW_W - 16.f) / 3.f; }

    void onButton(const ButtonEvent& e) override {
        if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            int s = (int) std::floor((e.pos.x - segX()) / segW());
            s = std::max(0, std::min(2, s));
            module->viewMode = (LSDrone::ViewMode) s;
            e.consume(this);
        }
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        int cur = module ? (int) module->viewMode : 0;
        const char* names[3] = {"WAVE", "BARS", "SPEC"};

        const float x0 = segX(), y0 = 15.f, w = LS_VIEW_W - 16.f, h = 19.f, sw = segW();

        nvgBeginPath(vg);
        nvgRoundedRect(vg, x0, y0, w, h, 4.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // selected segment
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x0 + sw * cur + 1.5f, y0 + 1.5f, sw - 3.f, h - 3.f, 3.f);
        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 140));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // separators
        nvgBeginPath(vg);
        for (int i = 1; i < 3; ++i) {
            if (i == cur || i == cur + 1) continue;
            nvgMoveTo(vg, x0 + sw * i, y0 + 4.f);
            nvgLineTo(vg, x0 + sw * i, y0 + h - 4.f);
        }
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        for (int i = 0; i < 3; ++i)
            drawText(vg, x0 + sw * (i + 0.5f), y0 + h / 2.f, names[i], 8.f,
                     i == cur ? colAccent() : colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

// ============================================================
// Module widget
// ============================================================
struct LSDroneWidget : ModuleWidget {
    LSDroneWidget(LSDrone* module) {
        setModule(module);
        box.size = Vec(LS_PANEL_W, LS_PANEL_H);

        addChild(new LSPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new LSDisplayWidget(module));
        addChild(new LSCvRangeWidget(module, lsColX(5), LS_ROW[3]));
        addChild(new LSViewWidget(module, LS_PANEL_W / 2.f));

        // 23 gauge knobs with CV jack (left) + attenuverter (right)
        for (int i = 0; i < 23; ++i) {
            int row = i / 6, col = i % 6;
            float cx = lsColX(col), ry = LS_ROW[row];
            const LSCellDef& c = LS_CELLS[i];
            addParam(createParamCentered<LSGaugeKnob>(Vec(cx, ry), module, c.param));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - LS_JACK_DX, ry + LS_JACK_DY), module, c.cv));
            addParam(createParamCentered<Trimpot>(Vec(cx + LS_JACK_DX, ry + LS_JACK_DY), module, c.attn));
        }

        // I/O
        addInput(createInputCentered<DarkPJ301MPort>(Vec(lsColX(0), LS_PORT_Y), module, LSDrone::CV_A));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(lsColX(1), LS_PORT_Y), module, LSDrone::CV_B));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(lsColX(4), LS_PORT_Y), module, LSDrone::OUTPUT_LEFT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(lsColX(5), LS_PORT_Y), module, LSDrone::OUTPUT_RIGHT));
    }

    void appendContextMenu(Menu* menu) override {
        LSDrone* module = dynamic_cast<LSDrone*>(this->module);
        if (!module) return;

        // ---- CV range ----
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem {
            LSDrone* module;
            LSDrone::CVRange range;
            void onAction(const ActionEvent& e) override {
                module->cvRangeMode = range;
            }
            void step() override {
                rightText = (module->cvRangeMode == range) ? "✔" : "";
                MenuItem::step();
            }
        };

        CVRangeItem* bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = LSDrone::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);

        CVRangeItem* unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar)");
        unipolarItem->module = module;
        unipolarItem->range = LSDrone::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);

        // ---- Display mode ----
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Display"));

        struct ViewItem : MenuItem {
            LSDrone* module;
            LSDrone::ViewMode mode;
            void onAction(const ActionEvent& e) override {
                module->viewMode = mode;
            }
            void step() override {
                rightText = (module->viewMode == mode) ? "✔" : "";
                MenuItem::step();
            }
        };

        const char* viewNames[3] = {"Waveform", "Bars", "Spectrum"};
        for (int i = 0; i < 3; ++i) {
            ViewItem* item = createMenuItem<ViewItem>(viewNames[i]);
            item->module = module;
            item->mode = (LSDrone::ViewMode) i;
            menu->addChild(item);
        }
    }
};

Model *modelLSDrone = createModel<LSDrone, LSDroneWidget>("LSDrone");