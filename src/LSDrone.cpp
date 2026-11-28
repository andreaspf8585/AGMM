#include "YouTubeLoad.hpp"
#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <memory>

// ============================================================================
// LSDrone – Cathedral Drone
// ============================================================================
//
// OVERVIEW
// --------
// LSDrone is a 36 HP cathedral-style drone synthesizer with two detunable
// oscillators, a resonant low-pass, an LFO, a drift generator, a modulated
// stereo delay, a reverb with shimmer, an echo, and a final low-pass filter.
// Each of the 23 main parameters has a dedicated CV input and attenuverter,
// and the display can show the output waveform, a bar-graph spectrum, or a
// continuous spectrum curve.
//
// File structure:
//   1. LSDroneEngine – the full audio DSP chain (unchanged from the original).
//   2. LSDrone module – parameter/CV handling, scope and FFT ring buffers.
//   3. LSAnalyzer – a small real-FFT analyzer used by the BARS/SPECTRUM views.
//   4. UI widgets – panel background, display (wave/bars/spectrum), CV-range
//      toggle cell, and display-mode switch.
//   5. LSDroneWidget – places all controls, jacks and the display.
//   6. Model registration.
// ============================================================================

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

    // Constructor: does nothing heavy (buffers allocated in init()).
    LSDroneEngine() {}
    // Destructor: frees all delay/reverb/echo buffers.
    ~LSDroneEngine()
    {
        delete[] delayBufferL;
        delete[] delayBufferR;
        delete[] revBuffer;
        delete[] echoBufferL;
        delete[] echoBufferR;
    }

    // Allocates all delay/reverb/echo buffers for the given sample rate and
    // zero-initialises them.
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

    // Sets oscillator A fine-tune (in semitones).
    void setFineA(float v) { oscA.fine = v; }
    // Sets oscillator B fine-tune (in semitones).
    void setFineB(float v) { oscB.fine = v; }
    // Sets oscillator B detune relative to A (in semitones).
    void setDetune(float v) { detune = v; }
    // Sets the A/B mix (0 = all A, 1 = all B).
    void setMix(float v) { mix = v; }
    // Sets the pre-filter cutoff (0..1).
    void setFilterCutoff(float v) { filterCutoff = v; }
    // Sets the pre-filter resonance (0..1, currently only affects the final LPF).
    void setFilterReso(float v) { filterReso = v; }
    // Sets the LFO rate (Hz).
    void setLfoRate(float v) { lfoRate = v; }
    // Sets the LFO depth (semitones).
    void setLfoDepth(float v) { lfoDepth = v; }
    // Sets the drift amount (0..1).
    void setDrift(float v) { drift = v; }
    // Sets the delay time (seconds).
    void setDelayTime(float v) { delayTime = v; }
    // Sets the delay feedback (0..0.98).
    void setDelayFeedback(float v) { delayFeedback = v; }
    // Sets the delay diffusion (0..1).
    void setDelayDiffusion(float v) { delayDiffusion = v; }
    // Sets the reverb decay (0.5..0.99).
    void setRevDecay(float v) { revDecay = v; }
    // Sets the reverb damping (0..1).
    void setRevDamping(float v) { revDamping = v; }
    // Sets the wash mix (0..1).
    void setWash(float v) { washMix = v; }
    // Sets the shimmer amount (0..1).
    void setShimmer(float v) { shimmer = v; }
    // Sets the delay modulation rate (Hz).
    void setModRate(float v) { modRate = v; }
    // Sets the delay modulation depth (0..1).
    void setModDepth(float v) { modDepth = v; }
    // Sets the echo time (seconds).
    void setEchoTime(float v) { echoTime = v; }
    // Sets the echo feedback (0..0.95).
    void setEchoFeedback(float v) { echoFeedback = v; }
    // Sets the echo mix (0..1).
    void setEchoMix(float v) { echoMix = v; }
    // Sets the final low-pass cutoff (0..1).
    void setLpfCutoff(float v) { lpfCutoff = v; }
    // Sets the final low-pass resonance (0..1).
    void setLpfReso(float v) { lpfReso = v; }

    // Generates one sample of the given oscillator at the specified pitch
    // modulation (semitones), advancing its phase.
    float processOscillator(Oscillator &osc, float pitchMod)
    {
        float freq = osc.freq * std::pow(2.0f, pitchMod / 12.0f);
        osc.phase += freq / sampleRate;
        if (osc.phase >= 1.0f)
            osc.phase -= 1.0f;
        return std::sin(osc.phase * 2.0f * float(M_PI));
    }

    // Main DSP callback: computes the two oscillators, mixes, pre-filters,
    // modulates/runs the stereo delay, reverb, shimmer, wash, echo, final LPF,
    // and writes the left/right output (with soft tanh limiting).
    void process(float cvA, float cvB, float &leftOut, float &rightOut)
    {
        lfoPhase += lfoRate / sampleRate;
        if (lfoPhase >= 1.0f)
            lfoPhase -= 1.0f;
        float lfoMod = std::sin(lfoPhase * 2.0f * float(M_PI)) * lfoDepth;

        driftState += (rand() / (float)RAND_MAX - 0.5f) * 0.008f;
        driftState *= 0.998f;
        if (driftState > 0.2f)
            driftState = 0.2f;
        if (driftState < -0.2f)
            driftState = -0.2f;
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
        if (modPhase >= 1.0f)
            modPhase -= 1.0f;
        float modTime = modDepth * (std::sin(modPhase * 2.0f * float(M_PI)) * 0.5f + 0.5f);
        float currentDelaySec = delayTime + modTime * 0.1f;
        int delaySamps = (int)(currentDelaySec * sampleRate);
        if (delaySamps < 1)
            delaySamps = 1;
        int readIdx = delayIdx - delaySamps;
        if (readIdx < 0)
            readIdx += delayLength;
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
        if (revRead < 0)
            revRead += revLength;
        float revOut = revBuffer[revRead];
        float revIn = delayedL * 0.5f + delayedR * 0.5f;
        float damped = revOut * (1.0f - revDamping * 0.9f);
        revBuffer[revIdx] = revIn + damped * revDecay;
        revIdx = (revIdx + 1) % revLength;

        static float shimmerPhase = 0.0f;
        shimmerPhase += 440.0f / sampleRate;
        if (shimmerPhase >= 1.0f)
            shimmerPhase -= 1.0f;
        float shimmered = revOut * std::sin(shimmerPhase * 2.0f * float(M_PI)) * shimmer;

        float wet = delayedL * 0.5f + delayedR * 0.5f + revOut * 0.7f + shimmered * 0.5f;
        wet = wet / (1.0f + washMix);
        static float washState = 0.0f;
        washState += 0.01f * (wet - washState);
        wet = washState * washMix + dry * (1.0f - washMix);

        int echoSamps = (int)(echoTime * sampleRate);
        if (echoSamps < 1)
            echoSamps = 1;
        int echoRead = echoIdx - echoSamps;
        if (echoRead < 0)
            echoRead += echoLength;
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
        lpfX2L = lpfX1L;
        lpfX1L = inL;
        lpfY2L = lpfY1L;
        lpfY1L = outL;

        float inR = echoMixedR;
        float outR = (b0 * inR + b1 * lpfX1R + b2 * lpfX2R - a1 * lpfY1R - a2 * lpfY2R) / a0;
        lpfX2R = lpfX1R;
        lpfX1R = inR;
        lpfY2R = lpfY1R;
        lpfY1R = outR;

        leftOut = std::tanh(outL * 0.8f);
        rightOut = std::tanh(outR * 0.8f);
    }
};

// ============================================================
// Display resolution / analyzer constants
// ============================================================
static const int LS_SCOPE_N = 160;    // waveform columns
static const int LS_FFT_N = 4096;     // analyzer window (power of 2)
static const int LS_BARS = 48;        // bar analyzer bands
static const int LS_SPEC_PTS = 232;   // spectrum curve points
static const float LS_DB_MIN = -80.f; // bottom of analyzer display
static const float LS_SPEC_FMIN = 30.f;
static const float LS_BAR_FMIN = 40.f;

// ============================================================
// Module – engine + CV handling + scope tracking
// ============================================================
struct LSDrone : Module
{
    enum ParamId
    {
        PARAM_FINE_A,
        PARAM_FINE_B,
        PARAM_DETUNE,
        PARAM_MIX,
        PARAM_CUTOFF,
        PARAM_RESO,
        PARAM_LFO_RATE,
        PARAM_LFO_DEPTH,
        PARAM_DRIFT,
        PARAM_DELAY_TIME,
        PARAM_DELAY_FEED,
        PARAM_DELAY_DIFF,
        PARAM_REV_DECAY,
        PARAM_REV_DAMP,
        PARAM_WASH,
        PARAM_SHIMMER,
        PARAM_MOD_RATE,
        PARAM_MOD_DEPTH,
        PARAM_ECHO_TIME,
        PARAM_ECHO_FEED,
        PARAM_ECHO_MIX,
        PARAM_LPF_CUTOFF,
        PARAM_LPF_RESO,
        PARAM_ATTEN_FINE_A,
        PARAM_ATTEN_FINE_B,
        PARAM_ATTEN_DETUNE,
        PARAM_ATTEN_MIX,
        PARAM_ATTEN_CUTOFF,
        PARAM_ATTEN_RESO,
        PARAM_ATTEN_LFO_RATE,
        PARAM_ATTEN_LFO_DEPTH,
        PARAM_ATTEN_DRIFT,
        PARAM_ATTEN_DELAY_TIME,
        PARAM_ATTEN_DELAY_FEED,
        PARAM_ATTEN_DELAY_DIFF,
        PARAM_ATTEN_REV_DECAY,
        PARAM_ATTEN_REV_DAMP,
        PARAM_ATTEN_WASH,
        PARAM_ATTEN_SHIMMER,
        PARAM_ATTEN_MOD_RATE,
        PARAM_ATTEN_MOD_DEPTH,
        PARAM_ATTEN_ECHO_TIME,
        PARAM_ATTEN_ECHO_FEED,
        PARAM_ATTEN_ECHO_MIX,
        PARAM_ATTEN_LPF_CUTOFF,
        PARAM_ATTEN_LPF_RESO,
        PARAMS_LEN
    };
    enum InputId
    {
        CV_FINE_A,
        CV_FINE_B,
        CV_DETUNE,
        CV_MIX,
        CV_CUTOFF,
        CV_RESO,
        CV_LFO_RATE,
        CV_LFO_DEPTH,
        CV_DRIFT,
        CV_DELAY_TIME,
        CV_DELAY_FEED,
        CV_DELAY_DIFF,
        CV_REV_DECAY,
        CV_REV_DAMP,
        CV_WASH,
        CV_SHIMMER,
        CV_MOD_RATE,
        CV_MOD_DEPTH,
        CV_ECHO_TIME,
        CV_ECHO_FEED,
        CV_ECHO_MIX,
        CV_LPF_CUTOFF,
        CV_LPF_RESO,
        CV_A,
        CV_B,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_LEFT,
        OUTPUT_RIGHT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    enum CVRange
    {
        BIPOLAR_M5_5,
        UNIPOLAR_0_10
    };
    CVRange cvRangeMode = BIPOLAR_M5_5;

    enum ViewMode
    {
        VIEW_WAVE,
        VIEW_BARS,
        VIEW_SPECTRUM
    };
    ViewMode viewMode = VIEW_WAVE;

    LSDroneEngine engine;

    // Scope / meter state (written by audio thread, read by UI)
    float scopeMin[LS_SCOPE_N] = {};
    float scopeMax[LS_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    // Analyzer ring buffer (mono, normalised to +/-1)
    float fftBuf[LS_FFT_N] = {};
    int fftHead = 0;
    float dispSampleRate = 44100.f;

    // Constructor: configures every parameter, attenuverter, CV input and
    // output; sets up the default values.
    LSDrone()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(PARAM_FINE_A, -12.f, 12.f, 0.f, "Fine A", " st");
        configParam(PARAM_FINE_B, -12.f, 12.f, 0.f, "Fine B", " st");
        configParam(PARAM_DETUNE, -24.f, 24.f, 0.f, "Detune", " st");
        configParam(PARAM_MIX, 0.f, 1.f, 0.5f, "Mix", "%", 0.f, 100.f);
        configParam(PARAM_CUTOFF, 0.f, 1.f, 0.5f, "Cutoff", "%", 0.f, 100.f);
        configParam(PARAM_RESO, 0.f, 1.f, 0.3f, "Resonance", "%", 0.f, 100.f);
        configParam(PARAM_LFO_RATE, 0.01f, 5.f, 0.2f, "LFO Rate", " Hz");
        configParam(PARAM_LFO_DEPTH, 0.f, 2.f, 0.5f, "LFO Depth", " st");
        configParam(PARAM_DRIFT, 0.f, 1.f, 0.3f, "Drift", "%", 0.f, 100.f);
        configParam(PARAM_DELAY_TIME, 0.1f, 2.5f, 0.8f, "Delay Time", " s");
        configParam(PARAM_DELAY_FEED, 0.f, 0.98f, 0.6f, "Delay Feedback", "%", 0.f, 100.f);
        configParam(PARAM_DELAY_DIFF, 0.f, 1.f, 0.4f, "Delay Diffusion", "%", 0.f, 100.f);
        configParam(PARAM_REV_DECAY, 0.5f, 0.99f, 0.85f, "Reverb Decay", "%", 0.f, 100.f);
        configParam(PARAM_REV_DAMP, 0.f, 1.f, 0.5f, "Reverb Damping", "%", 0.f, 100.f);
        configParam(PARAM_WASH, 0.f, 1.f, 0.6f, "Wash", "%", 0.f, 100.f);
        configParam(PARAM_SHIMMER, 0.f, 1.f, 0.2f, "Shimmer", "%", 0.f, 100.f);
        configParam(PARAM_MOD_RATE, 0.02f, 2.f, 0.15f, "Mod Rate", " Hz");
        configParam(PARAM_MOD_DEPTH, 0.f, 1.f, 0.3f, "Mod Depth", "%", 0.f, 100.f);
        configParam(PARAM_ECHO_TIME, 0.05f, 1.5f, 0.4f, "Echo Time", " s");
        configParam(PARAM_ECHO_FEED, 0.f, 0.95f, 0.5f, "Echo Feedback", "%", 0.f, 100.f);
        configParam(PARAM_ECHO_MIX, 0.f, 1.f, 0.4f, "Echo Mix", "%", 0.f, 100.f);
        configParam(PARAM_LPF_CUTOFF, 0.f, 1.f, 0.8f, "LPF Cutoff", "%", 0.f, 100.f);
        configParam(PARAM_LPF_RESO, 0.f, 1.f, 0.2f, "LPF Resonance", "%", 0.f, 100.f);

        static const char *const cvNames[23] = {"Fine A", "Fine B", "Detune", "Mix", "Cutoff", "Resonance", "LFO rate", "LFO depth", "Drift", "Delay time", "Delay feedback", "Delay diffusion", "Reverb decay", "Reverb damping", "Wash", "Shimmer", "Mod rate", "Mod depth", "Echo time", "Echo feedback", "Echo mix", "LPF cutoff", "LPF resonance"};
        auto atten = [this](ParamId id, float def = 0.5f)
        { configParam(id, -1.f, 1.f, def, std::string(cvNames[id - PARAM_ATTEN_FINE_A]) + " attenuverter", "%", 0.f, 100.f); };
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

        for (int i = 0; i < 23; ++i)
            configInput(CV_FINE_A + i, std::string(cvNames[i]) + " CV");
        configInput(CV_A, "V/oct A");
        configInput(CV_B, "V/oct B");
        configOutput(OUTPUT_LEFT, "L out");
        configOutput(OUTPUT_RIGHT, "R out");
    }

    // Called when the sample rate changes: re-initialises the engine's buffers.
    void onSampleRateChange() override
    {
        engine.init(APP->engine->getSampleRate());
    }

    // Combines a knob value with its CV and attenuverter, normalises the CV
    // according to the selected CV range, and clamps to [minV, maxV].
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

    // Serialises the CV range mode and the display view mode.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        json_object_set_new(root, "viewMode", json_integer(viewMode));
        return root;
    }

    // Restores the CV range mode and display view mode from a patch,
    // clamping the view mode to a valid value.
    void dataFromJson(json_t *root) override
    {
        json_t *mode = json_object_get(root, "cvRangeMode");
        if (mode)
            cvRangeMode = (json_integer_value(mode) == UNIPOLAR_0_10) ? UNIPOLAR_0_10 : BIPOLAR_M5_5;

        json_t *view = json_object_get(root, "viewMode");
        if (view)
        {
            int v = (int)json_integer_value(view);
            if (v < 0)
                v = 0;
            if (v > 2)
                v = 2;
            viewMode = (ViewMode)v;
        }
    }

    // Main audio callback: reads every parameter with its CV/attenuverter,
    // runs the engine, writes the stereo output, and updates the scope ring
    // buffer and the FFT ring buffer for the display.
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

        if (scopeCount == 0)
            scopeBMin = scopeBMax = mono;
        else
        {
            scopeBMin = std::min(scopeBMin, mono);
            scopeBMax = std::max(scopeBMax, mono);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % LS_SCOPE_N;
        }
    }
};

namespace
{

    // ------------------------------------------------------------
    // Small FFT analyzer used by the BARS / SPECTRUM display modes
    // ------------------------------------------------------------
    struct LSAnalyzer
    {
        float re[LS_FFT_N];
        float im[LS_FFT_N];
        float win[LS_FFT_N];
        float cosT[LS_FFT_N / 2];
        float sinT[LS_FFT_N / 2];
        float db[LS_FFT_N / 2];

        // Constructor: precomputes the Hann window, the twiddle factors, and
        // initialises the magnitude buffer.
        LSAnalyzer()
        {
            for (int n = 0; n < LS_FFT_N; ++n)
            {
                win[n] = 0.5f * (1.f - std::cos(2.f * float(M_PI) * n / (LS_FFT_N - 1)));
                re[n] = 0.f;
                im[n] = 0.f;
            }
            for (int k = 0; k < LS_FFT_N / 2; ++k)
            {
                cosT[k] = std::cos(2.f * float(M_PI) * k / LS_FFT_N);
                sinT[k] = -std::sin(2.f * float(M_PI) * k / LS_FFT_N);
                db[k] = -90.f;
            }
        }

        // Runs an in-place radix-2 FFT on the latest window from the ring
        // buffer, then converts magnitudes to dB with fast attack / slow
        // release smoothing.
        void update(const float *ring, int head)
        {
            const int N = LS_FFT_N;
            for (int n = 0; n < N; ++n)
            {
                int idx = (head + n) & (N - 1); // oldest -> newest
                re[n] = ring[idx] * win[n];
                im[n] = 0.f;
            }

            // bit reversal
            for (int i = 1, j = 0; i < N; ++i)
            {
                int bit = N >> 1;
                for (; j & bit; bit >>= 1)
                    j ^= bit;
                j ^= bit;
                if (i < j)
                {
                    std::swap(re[i], re[j]);
                    std::swap(im[i], im[j]);
                }
            }
            // butterflies
            for (int len = 2; len <= N; len <<= 1)
            {
                int half = len >> 1;
                int stride = N / len;
                for (int i = 0; i < N; i += len)
                {
                    for (int j = 0; j < half; ++j)
                    {
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
            const float norm = 4.f / (float)N;
            for (int k = 0; k < N / 2; ++k)
            {
                float m = std::sqrt(re[k] * re[k] + im[k] * im[k]) * norm;
                float d = 20.f * std::log10(m + 1e-7f);
                if (d < -90.f)
                    d = -90.f;
                if (d > db[k])
                    db[k] = d;
                else
                    db[k] += (d - db[k]) * 0.18f;
            }
        }

        // Returns the interpolated dB value at a fractional bin position.
        float dbAt(float pos) const
        {
            if (pos < 0.f)
                pos = 0.f;
            float maxPos = (float)(LS_FFT_N / 2 - 2);
            if (pos > maxPos)
                pos = maxPos;
            int k = (int)pos;
            float t = pos - (float)k;
            return db[k] * (1.f - t) + db[k + 1] * t;
        }

        // level of a frequency band; interpolates for narrow bands, takes the max for wide ones
        // Returns the level in dB of a frequency band [f0, f1], either by
        // interpolating (narrow bands) or by taking the maximum bin (wide).
        float bandDb(float f0, float f1, float binHz) const
        {
            float b0 = f0 / binHz, b1 = f1 / binHz;
            if (b1 - b0 <= 1.f)
                return dbAt(0.5f * (b0 + b1));
            int k0 = std::max(1, (int)std::ceil(b0));
            int k1 = std::min(LS_FFT_N / 2 - 1, (int)std::floor(b1));
            float m = -90.f;
            for (int k = k0; k <= k1; ++k)
                m = std::max(m, db[k]);
            return m;
        }
    };

    // Converts a dB value to a 0..1 normalised height for the display.
    float dbToNorm(float d)
    {
        float n = (d - LS_DB_MIN) / (0.f - LS_DB_MIN);
        return std::min(std::max(n, 0.f), 1.f);
    }

} // namespace

// ============================================================
// Layout (36 HP = 540 x 380)
// Every control sits in a plate with its label ABOVE it. Rows, top to bottom:
//   y  26   display: scope + meters | divider | readout row
//   y  86   4 rows of 6 cells (name, then CV jack | knob | attenuverter)
//   y 322   I/O row
// ============================================================
static const float LS_PANEL_W = RACK_GRID_WIDTH * 36; // 540
static const float LS_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float LS_DISP_W = 480.f;
static const float LS_DISP_H = 50.f; // 38 scope + 12 readout row
static const float LS_DISP_X = (LS_PANEL_W - LS_DISP_W) / 2.f;
static const float LS_DISP_Y = 26.f;
static const float LS_SCOPE_W = 464.f; // meters sit to the right of it
static const float LS_SCOPE_H = 38.f;
static const float LS_FOOT_Y = LS_SCOPE_H + 7.f; // readout row centre

static const float LS_CELL_W = 86.f;
static const float LS_CELL_H = 54.f;
static const float LS_ROW_TOP[4] = {86.f, 142.f, 198.f, 254.f};
static const float LS_CTRL_DY = 27.f; // knob / jack / trimpot centre below the plate top
static const float LS_CAP_DY = 46.f;  // "CV" / "ATT" captions
static const float LS_JACK_DX = 30.f; // CV jack (left) and attenuverter (right)

static const float LS_SEC_Y = 315.f;
static const float LS_IO_TOP = 322.f, LS_IO_H = 36.f, LS_IO_W = 60.f; // label +8, jack +24
static const float LS_VIEW_W = 150.f;                                 // display-mode switch plate

// Returns the horizontal centre of control column `col` (0..5) on the panel.
static float lsColX(int col) { return LS_PANEL_W / 12.f * (2 * col + 1); }
// 540/12=45 -> 45, 135, 225, 315, 405, 495

// ============================================================
// Control table (label + ids)
// ============================================================
namespace
{
    struct LSCellDef
    {
        const char *name;
        int param, attn, cv;
    };
}

static const LSCellDef LS_CELLS[23] = {
    {"FINE A", LSDrone::PARAM_FINE_A, LSDrone::PARAM_ATTEN_FINE_A, LSDrone::CV_FINE_A},
    {"FINE B", LSDrone::PARAM_FINE_B, LSDrone::PARAM_ATTEN_FINE_B, LSDrone::CV_FINE_B},
    {"DETUNE", LSDrone::PARAM_DETUNE, LSDrone::PARAM_ATTEN_DETUNE, LSDrone::CV_DETUNE},
    {"MIX", LSDrone::PARAM_MIX, LSDrone::PARAM_ATTEN_MIX, LSDrone::CV_MIX},
    {"CUTOFF", LSDrone::PARAM_CUTOFF, LSDrone::PARAM_ATTEN_CUTOFF, LSDrone::CV_CUTOFF},
    {"RESO", LSDrone::PARAM_RESO, LSDrone::PARAM_ATTEN_RESO, LSDrone::CV_RESO},
    {"LFO RATE", LSDrone::PARAM_LFO_RATE, LSDrone::PARAM_ATTEN_LFO_RATE, LSDrone::CV_LFO_RATE},
    {"LFO DEPTH", LSDrone::PARAM_LFO_DEPTH, LSDrone::PARAM_ATTEN_LFO_DEPTH, LSDrone::CV_LFO_DEPTH},
    {"DRIFT", LSDrone::PARAM_DRIFT, LSDrone::PARAM_ATTEN_DRIFT, LSDrone::CV_DRIFT},
    {"DELAY T", LSDrone::PARAM_DELAY_TIME, LSDrone::PARAM_ATTEN_DELAY_TIME, LSDrone::CV_DELAY_TIME},
    {"DELAY FB", LSDrone::PARAM_DELAY_FEED, LSDrone::PARAM_ATTEN_DELAY_FEED, LSDrone::CV_DELAY_FEED},
    {"DELAY DIF", LSDrone::PARAM_DELAY_DIFF, LSDrone::PARAM_ATTEN_DELAY_DIFF, LSDrone::CV_DELAY_DIFF},
    {"REV DECAY", LSDrone::PARAM_REV_DECAY, LSDrone::PARAM_ATTEN_REV_DECAY, LSDrone::CV_REV_DECAY},
    {"REV DAMP", LSDrone::PARAM_REV_DAMP, LSDrone::PARAM_ATTEN_REV_DAMP, LSDrone::CV_REV_DAMP},
    {"WASH", LSDrone::PARAM_WASH, LSDrone::PARAM_ATTEN_WASH, LSDrone::CV_WASH},
    {"SHIMMER", LSDrone::PARAM_SHIMMER, LSDrone::PARAM_ATTEN_SHIMMER, LSDrone::CV_SHIMMER},
    {"MOD RATE", LSDrone::PARAM_MOD_RATE, LSDrone::PARAM_ATTEN_MOD_RATE, LSDrone::CV_MOD_RATE},
    {"MOD DEPTH", LSDrone::PARAM_MOD_DEPTH, LSDrone::PARAM_ATTEN_MOD_DEPTH, LSDrone::CV_MOD_DEPTH},
    {"ECHO T", LSDrone::PARAM_ECHO_TIME, LSDrone::PARAM_ATTEN_ECHO_TIME, LSDrone::CV_ECHO_TIME},
    {"ECHO FB", LSDrone::PARAM_ECHO_FEED, LSDrone::PARAM_ATTEN_ECHO_FEED, LSDrone::CV_ECHO_FEED},
    {"ECHO MIX", LSDrone::PARAM_ECHO_MIX, LSDrone::PARAM_ATTEN_ECHO_MIX, LSDrone::CV_ECHO_MIX},
    {"LPF CUT", LSDrone::PARAM_LPF_CUTOFF, LSDrone::PARAM_ATTEN_LPF_CUTOFF, LSDrone::CV_LPF_CUTOFF},
    {"LPF RESO", LSDrone::PARAM_LPF_RESO, LSDrone::PARAM_ATTEN_LPF_RESO, LSDrone::CV_LPF_RESO},
};

// ============================================================
// UI (anonymous namespace: names cannot clash with other module files)
// ============================================================
namespace
{

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the 24 control cells (23
    // parameter cells plus the CV-range cell), and the I/O row with the
    // display-mode switch plate in the middle.
    struct LSPanelBackground : TransparentWidget
    {
        LSPanelBackground() { box.size = Vec(LS_PANEL_W, LS_PANEL_H); }

        // Draws all static panel art: base panel, bezel, control cells with
        // their labels/CV/ATT captions, the I/O plates (outputs tinted) and
        // the display-mode switch plate.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, LS_PANEL_W, LS_PANEL_H, "LS DRONE");
            agmm::bezel(vg, LS_DISP_X, LS_DISP_Y, LS_DISP_W, LS_DISP_H);

            // 23 control cells + the CV range cell
            for (int i = 0; i < 24; ++i)
            {
                const int row = i / 6, col = i % 6;
                const float cx = lsColX(col), top = LS_ROW_TOP[row];
                agmm::plate(vg, cx, top, LS_CELL_W, LS_CELL_H, false);
                if (i < 23)
                {
                    agmm::text(vg, cx, top + 9.f, LS_CELLS[i].name, 8.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                    agmm::text(vg, cx - LS_JACK_DX, top + LS_CAP_DY, "CV", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                    agmm::text(vg, cx + LS_JACK_DX, top + LS_CAP_DY, "ATT", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                }
                else
                    agmm::text(vg, cx, top + 9.f, "CV RANGE", 8.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            // I/O: inputs plain, outputs tinted, display switch in the middle
            agmm::sectionHeader(vg, LS_PANEL_W, LS_SEC_Y, "I / O", 14.f);
            const char *io[4] = {"V/OCT A", "V/OCT B", "L OUT", "R OUT"};
            const int ioCols[4] = {0, 1, 4, 5};
            for (int i = 0; i < 4; ++i)
            {
                const float cx = lsColX(ioCols[i]);
                agmm::plate(vg, cx, LS_IO_TOP, LS_IO_W, LS_IO_H, i >= 2);
                agmm::text(vg, cx, LS_IO_TOP + 8.f, io[i], 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }
            agmm::plate(vg, LS_PANEL_W / 2.f, LS_IO_TOP, LS_VIEW_W, LS_IO_H, false);
            agmm::text(vg, LS_PANEL_W / 2.f, LS_IO_TOP + 8.f, "DISPLAY", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope display widget (WAVE / BARS / SPECTRUM)
    // ------------------------------------------------------------
    // Live display: chooses between the scope waveform, a bar spectrum, or a
    // continuous spectrum curve, and also draws the L/R meters, the readout
    // row (delay/wash) and the frequency-axis labels for the analyzer views.
    struct LSDisplayWidget : TransparentWidget
    {
        LSDrone *module = nullptr;
        std::unique_ptr<LSAnalyzer> an;
        float barLvl[LS_BARS] = {};
        float barPeak[LS_BARS] = {};

        // Constructor: positions the widget and allocates the FFT analyzer.
        LSDisplayWidget(LSDrone *m) : module(m)
        {
            box.pos = Vec(LS_DISP_X, LS_DISP_Y);
            box.size = Vec(LS_DISP_W, LS_DISP_H);
            an.reset(new LSAnalyzer());
        }

        // Returns the width of the scope area (the rest is meters).
        static float scopeW() { return LS_SCOPE_W; }

        // Returns the current view mode (or WAVE when there is no module).
        int mode() const { return module ? (int)module->viewMode : (int)LSDrone::VIEW_WAVE; }
        // Returns the current display sample rate (or 48 kHz as a fallback).
        float sampleRate() const { return (module && module->dispSampleRate > 1000.f) ? module->dispSampleRate : 48000.f; }
        // Returns the upper frequency limit of the analyzer (min(18 kHz, 0.45*fs)).
        float fMax() const { return std::min(18000.f, sampleRate() * 0.45f); }
        // Returns the width of one FFT bin in Hz.
        float binHz() const { return sampleRate() / (float)LS_FFT_N; }

        // Maps a frequency f (Hz) to an x pixel using a logarithmic axis
        // between `fmin` and fMax().
        float freqToX(float f, float fmin) const
        {
            float fmax = fMax();
            return scopeW() * std::log(f / fmin) / std::log(fmax / fmin);
        }

        // Called every UI frame: refreshes the FFT and, in BARS mode, updates
        // the smoothed bar levels and peak-hold values.
        void step() override
        {
            if (module && module->viewMode != LSDrone::VIEW_WAVE)
            {
                an->update(module->fftBuf, module->fftHead);

                if (module->viewMode == LSDrone::VIEW_BARS)
                {
                    float ratio = fMax() / LS_BAR_FMIN;
                    for (int i = 0; i < LS_BARS; ++i)
                    {
                        float f0 = LS_BAR_FMIN * std::pow(ratio, (float)i / LS_BARS);
                        float f1 = LS_BAR_FMIN * std::pow(ratio, (float)(i + 1) / LS_BARS);
                        float n = dbToNorm(an->bandDb(f0, f1, binHz()));
                        barLvl[i] = n;
                        barPeak[i] = std::max(n, barPeak[i] - 0.012f);
                    }
                }
            }
            Widget::step();
        }

        // Static layer: fills the display screen, draws the mode-appropriate
        // grid (time grid for WAVE, level lines for BARS/SPECTRUM, plus
        // frequency grid lines for SPECTRUM), and the divider above the
        // readout row.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float sw = scopeW();

            agmm::screen(vg, 0.f, 0.f, LS_DISP_W, LS_DISP_H);

            // Faint grid over the scope area only
            nvgBeginPath(vg);
            if (mode() == LSDrone::VIEW_WAVE)
            {
                for (float x = 0.f; x <= sw; x += 28.f)
                {
                    nvgMoveTo(vg, x + 0.5f, 0.f);
                    nvgLineTo(vg, x + 0.5f, LS_SCOPE_H);
                }
                nvgMoveTo(vg, 0.f, LS_SCOPE_H / 2.f + 0.5f);
                nvgLineTo(vg, sw, LS_SCOPE_H / 2.f + 0.5f);
            }
            else
            {
                for (int i = 1; i < 4; ++i)
                {
                    float y = LS_SCOPE_H * i / 4.f + 0.5f;
                    nvgMoveTo(vg, 0.f, y);
                    nvgLineTo(vg, sw, y);
                }
                if (mode() == LSDrone::VIEW_SPECTRUM)
                {
                    const float fl[3] = {100.f, 1000.f, 10000.f};
                    for (int i = 0; i < 3; ++i)
                    {
                        float x = freqToX(fl[i], LS_SPEC_FMIN) + 0.5f;
                        nvgMoveTo(vg, x, 0.f);
                        nvgLineTo(vg, x, LS_SCOPE_H);
                    }
                }
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Divider between the scope and the readout row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, LS_SCOPE_H + 1.5f);
            nvgLineTo(vg, LS_DISP_W, LS_SCOPE_H + 1.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
            nvgStroke(vg);

            Widget::draw(args);
        }

        // ---------- waveform ----------
        // Draws the min/max waveform columns from the module's scope ring
        // buffer (or a demo burst when there is no module).
        void drawWave(NVGcontext *vg)
        {
            const float mid = LS_SCOPE_H / 2.f;
            const float amp = mid - 2.f;
            const float colW = scopeW() / (float)LS_SCOPE_N;

            nvgBeginPath(vg);
            for (int i = 0; i < LS_SCOPE_N; ++i)
            {
                float vmin, vmax;
                if (module)
                {
                    int idx = (module->scopeHead + i) % LS_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                }
                else
                {
                    float t = (float)i / LS_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - std::min(std::max(vmax / 5.f, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / 5.f, -1.f), 1.f) * amp;
                nvgRect(vg, i * colW, y0, std::max(colW - 0.4f, 1.f), std::max(y1 - y0, 1.f));
            }
            nvgFillColor(vg, agmm::bright());
            nvgFill(vg);
        }

        // ---------- bars ----------
        // Draws the bar spectrum with peak-hold lines from the barLvl/barPeak
        // arrays.
        void drawBars(NVGcontext *vg)
        {
            const float colW = scopeW() / (float)LS_BARS;
            const float base = LS_SCOPE_H - 1.f;
            const float span = LS_SCOPE_H - 3.f;

            nvgBeginPath(vg);
            for (int i = 0; i < LS_BARS; ++i)
            {
                float h = barLvl[i] * span;
                if (h < 0.5f)
                    continue;
                nvgRect(vg, i * colW + 1.f, base - h, std::max(colW - 2.f, 1.f), h);
            }
            nvgFillColor(vg, agmm::accent());
            nvgFill(vg);

            // peak holds
            nvgBeginPath(vg);
            for (int i = 0; i < LS_BARS; ++i)
            {
                if (barPeak[i] < 0.02f)
                    continue;
                nvgRect(vg, i * colW + 1.f, base - barPeak[i] * span - 1.5f, std::max(colW - 2.f, 1.f), 1.5f);
            }
            nvgFillColor(vg, agmm::bright());
            nvgFill(vg);
        }

        // ---------- spectrum ----------
        // Draws the continuous spectrum curve (filled area plus 1.2 px line)
        // from the analyzer's dB values.
        void drawSpectrum(NVGcontext *vg)
        {
            const float sw = scopeW();
            const float base = LS_SCOPE_H - 1.f;
            const float span = LS_SCOPE_H - 3.f;
            const float ratio = fMax() / LS_SPEC_FMIN;

            float ys[LS_SPEC_PTS + 1];
            for (int i = 0; i <= LS_SPEC_PTS; ++i)
            {
                float f0 = LS_SPEC_FMIN * std::pow(ratio, (float)i / LS_SPEC_PTS);
                float f1 = LS_SPEC_FMIN * std::pow(ratio, (float)(i + 1) / LS_SPEC_PTS);
                ys[i] = base - dbToNorm(an->bandDb(f0, f1, binHz())) * span;
            }

            // filled area (flat)
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, LS_SCOPE_H);
            for (int i = 0; i <= LS_SPEC_PTS; ++i)
                nvgLineTo(vg, sw * i / LS_SPEC_PTS, ys[i]);
            nvgLineTo(vg, sw, LS_SCOPE_H);
            nvgClosePath(vg);
            nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
            nvgFill(vg);

            // line
            nvgBeginPath(vg);
            for (int i = 0; i <= LS_SPEC_PTS; ++i)
            {
                float x = sw * i / LS_SPEC_PTS;
                if (i == 0)
                    nvgMoveTo(vg, x, ys[i]);
                else
                    nvgLineTo(vg, x, ys[i]);
            }
            nvgLineJoin(vg, NVG_ROUND);
            nvgStrokeColor(vg, agmm::bright());
            nvgStrokeWidth(vg, 1.2f);
            nvgStroke(vg);
        }

        // Layer-1 overlay: dispatches to the wave/bars/spectrum draw routine,
        // clips it to the scope, then draws the meters and the readout row
        // (delay, wash, frequency labels).
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float sw = scopeW();

            nvgSave(vg);
            nvgScissor(vg, 0.f, 0.f, sw, LS_SCOPE_H);
            switch (mode())
            {
            case LSDrone::VIEW_BARS:
                drawBars(vg);
                break;
            case LSDrone::VIEW_SPECTRUM:
                drawSpectrum(vg);
                break;
            default:
                drawWave(vg);
                break;
            }
            nvgRestore(vg);

            // Level meters (L / R) to the right of the scope
            for (int ch = 0; ch < 2; ++ch)
            {
                float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 3.f : 2.4f);
                agmm::meter(vg, sw + 4.f + ch * 7.f, 2.f, 5.f, LS_SCOPE_H - 4.f, peak / 5.f);
            }

            // Readout row (own row, never over the scope): delay left, wash right,
            // frequency axis labels in between for the analyzer views
            char buf[32];
            float dly = module ? module->params[LSDrone::PARAM_DELAY_TIME].getValue() : 0.8f;
            float wash = module ? module->params[LSDrone::PARAM_WASH].getValue() : 0.6f;
            snprintf(buf, sizeof(buf), "DLY %4.0fMS", dly * 1000.f);
            agmm::text(vg, 4.f, LS_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "WASH %3.0f%%", wash * 100.f);
            agmm::text(vg, LS_DISP_W - 4.f, LS_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

            if (mode() != LSDrone::VIEW_WAVE)
            {
                const float fmin = (mode() == LSDrone::VIEW_BARS) ? LS_BAR_FMIN : LS_SPEC_FMIN;
                const char *names[3] = {"100", "1K", "10K"};
                const float fl[3] = {100.f, 1000.f, 10000.f};
                for (int i = 0; i < 3; ++i)
                    agmm::text(vg, freqToX(fl[i], fmin), LS_FOOT_Y, names[i], 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // CV range cell (click to toggle)
    // ------------------------------------------------------------
    // Clickable cell that toggles the module's CV range mode and shows the
    // current range ("-5/+5V" or "0/+10V").
    struct LSCvRangeWidget : TransparentWidget
    {
        LSDrone *module = nullptr;

        // Constructor: positions the widget over the 24th cell.
        LSCvRangeWidget(LSDrone *m, float cx, float top) : module(m)
        {
            box.pos = Vec(cx - LS_CELL_W / 2.f, top);
            box.size = Vec(LS_CELL_W, LS_CELL_H);
        }

        // Toggles the CV range mode on a left click.
        void onButton(const ButtonEvent &e) override
        {
            if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT)
            {
                module->cvRangeMode = (module->cvRangeMode == LSDrone::BIPOLAR_M5_5)
                                          ? LSDrone::UNIPOLAR_0_10
                                          : LSDrone::BIPOLAR_M5_5;
                e.consume(this);
            }
        }

        // Draws the current range value box and the "CLICK TO TOGGLE" hint.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            bool bipolar = module ? (module->cvRangeMode == LSDrone::BIPOLAR_M5_5) : true;

            // value box (label is drawn by the panel, 9 px below the plate top)
            const float sx = 8.f, sy = 17.f, sw = box.size.x - 16.f, sh = 20.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, sx, sy, sw, sh, 4.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            agmm::text(vg, box.size.x / 2.f, sy + sh / 2.f + 0.5f, bipolar ? "-5/+5V" : "0/+10V", 10.f,
                       agmm::accent(), agmm::ALIGN_C, 0.5f);
            agmm::text(vg, box.size.x / 2.f, 46.f, "CLICK TO TOGGLE", 7.f, agmm::dim(), agmm::ALIGN_C, 0.3f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display-mode switch (WAVE | BARS | SPEC) - click a segment
    // ------------------------------------------------------------
    // Segmented switch that selects the display view mode by clicking one of
    // its three segments.
    struct LSViewWidget : TransparentWidget
    {
        LSDrone *module = nullptr;

        // Constructor: positions the widget in the I/O row.
        LSViewWidget(LSDrone *m, float cx) : module(m)
        {
            box.pos = Vec(cx - LS_VIEW_W / 2.f, LS_IO_TOP);
            box.size = Vec(LS_VIEW_W, LS_IO_H);
        }

        // Left inset of the segment strip.
        static float segX() { return 8.f; }
        // Width of one segment.
        static float segW() { return (LS_VIEW_W - 16.f) / 3.f; }

        // Selects the view mode corresponding to the clicked segment.
        void onButton(const ButtonEvent &e) override
        {
            if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT)
            {
                int s = (int)std::floor((e.pos.x - segX()) / segW());
                s = std::max(0, std::min(2, s));
                module->viewMode = (LSDrone::ViewMode)s;
                e.consume(this);
            }
        }

        // Draws the three-segment switch, highlighting the current mode and
        // hiding the separators adjacent to it.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            int cur = module ? (int)module->viewMode : 0;
            const char *names[3] = {"WAVE", "BARS", "SPEC"};

            const float x0 = segX(), y0 = 14.f, w = LS_VIEW_W - 16.f, h = 18.f, sw = segW();

            nvgBeginPath(vg);
            nvgRoundedRect(vg, x0, y0, w, h, 4.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
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
            nvgStroke(vg);

            // separators
            nvgBeginPath(vg);
            for (int i = 1; i < 3; ++i)
            {
                if (i == cur || i == cur + 1)
                    continue;
                nvgMoveTo(vg, x0 + sw * i, y0 + 4.f);
                nvgLineTo(vg, x0 + sw * i, y0 + h - 4.f);
            }
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStroke(vg);

            for (int i = 0; i < 3; ++i)
                agmm::text(vg, x0 + sw * (i + 0.5f), y0 + h / 2.f + 0.5f, names[i], 8.f,
                           i == cur ? agmm::accent() : agmm::dim(), agmm::ALIGN_C, 0.5f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Value arcs around the knobs (flat track + flat arc)
    //
    // Bipolar params (FINE A/B, DETUNE, attenuverters) fill from
    // the centre, unipolar ones from the left end; negative bipolar values
    // are drawn warm. The radius is set per knob to suit its size and plate.
    // ------------------------------------------------------------
    struct LSKnobArcs : TransparentWidget
    {
        struct Arc
        {
            Vec pos;
            int param;
            float radius;
            bool bipolar;
        };

        LSDrone *module = nullptr;
        std::vector<Arc> arcs;

        LSKnobArcs(LSDrone *m) : module(m) { box.size = Vec(LS_PANEL_W, LS_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
            const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

            for (const Arc &k : arcs)
            {
                // Track
                nvgBeginPath(vg);
                nvgArc(vg, k.pos.x, k.pos.y, k.radius, a0, a1, NVG_CW);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                nvgStrokeWidth(vg, 2.f);
                nvgLineCap(vg, NVG_BUTT);
                nvgStroke(vg);

                if (!module)
                    continue;
                float v = module->paramQuantities[k.param]->getScaledValue();
                float from = k.bipolar ? 0.5f : 0.f;
                float lo = std::min(from, v), hi = std::max(from, v);
                if (hi - lo < 0.004f)
                    continue;
                NVGcolor col = (k.bipolar && v < 0.5f) ? agmm::warm() : agmm::accent();
                nvgBeginPath(vg);
                nvgArc(vg, k.pos.x, k.pos.y, k.radius, a0 + (a1 - a0) * lo, a0 + (a1 - a0) * hi, NVG_CW);
                nvgStrokeColor(vg, col);
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }
        }
    };

} // namespace

// ============================================================
// Module widget: adds the static panel, screws, the live display, the CV
// range cell, the display-mode switch, the 23 parameter/CV/attenuverter
// triplets, and the I/O jacks.
// ============================================================
struct LSDroneWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, display, CV-range
    // and display-mode widgets, the 23 knob/jack/trimpot cells, and the four
    // I/O jacks.
    LSDroneWidget(LSDrone *module)
    {
        setModule(module);
        box.size = Vec(LS_PANEL_W, LS_PANEL_H);

        addChild(new LSPanelBackground());
        LSKnobArcs *arcs = new LSKnobArcs(module);
        addChild(arcs);

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new LSDisplayWidget(module));
        addChild(new LSCvRangeWidget(module, lsColX(5), LS_ROW_TOP[3]));
        addChild(new LSViewWidget(module, LS_PANEL_W / 2.f));

        // 23 knobs with CV jack (left) + attenuverter (right)
        for (int i = 0; i < 23; ++i)
        {
            const int row = i / 6, col = i % 6;
            const float cx = lsColX(col);
            const float cy = LS_ROW_TOP[row] + LS_CTRL_DY;
            const LSCellDef &c = LS_CELLS[i];
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, cy), module, c.param));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - LS_JACK_DX, cy), module, c.cv));
            addParam(createParamCentered<Trimpot>(Vec(cx + LS_JACK_DX, cy), module, c.attn));

            const bool bipolar = c.param == LSDrone::PARAM_FINE_A || c.param == LSDrone::PARAM_FINE_B ||
                                 c.param == LSDrone::PARAM_DETUNE;
            arcs->arcs.push_back({Vec(cx, cy), c.param, 14.f, bipolar});
            arcs->arcs.push_back({Vec(cx + LS_JACK_DX, cy), c.attn, 11.5f, true});
        }

        // I/O
        const float jy = LS_IO_TOP + 24.f;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(lsColX(0), jy), module, LSDrone::CV_A));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(lsColX(1), jy), module, LSDrone::CV_B));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(lsColX(4), jy), module, LSDrone::OUTPUT_LEFT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(lsColX(5), jy), module, LSDrone::OUTPUT_RIGHT));
    }

    // Adds the right-click menu: a CV-range submenu (bipolar/unipolar) and a
    // display-mode submenu (waveform/bars/spectrum), each with a checkmark on
    // the active option.
    void appendContextMenu(Menu *menu) override
    {
        agmm::addYouTubeLink(menu); // adds "Visit my youtube channel!"
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        LSDrone *module = dynamic_cast<LSDrone *>(this->module);
        if (!module)
            return;

        // ---- CV range ----
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem
        {
            LSDrone *module;
            LSDrone::CVRange range;
            // Sets the module's CV range mode to this item's range.
            void onAction(const ActionEvent &e) override
            {
                module->cvRangeMode = range;
            }
            // Shows a checkmark when this item's range is the active mode.
            void step() override
            {
                rightText = CHECKMARK(module->cvRangeMode == range);
                MenuItem::step();
            }
        };

        CVRangeItem *bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = LSDrone::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);

        CVRangeItem *unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar)");
        unipolarItem->module = module;
        unipolarItem->range = LSDrone::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);

        // ---- Display mode ----
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Display"));

        struct ViewItem : MenuItem
        {
            LSDrone *module;
            LSDrone::ViewMode mode;
            // Sets the module's display view mode to this item's mode.
            void onAction(const ActionEvent &e) override
            {
                module->viewMode = mode;
            }
            // Shows a checkmark when this item's mode is the active mode.
            void step() override
            {
                rightText = CHECKMARK(module->viewMode == mode);
                MenuItem::step();
            }
        };

        const char *viewNames[3] = {"Waveform", "Bars", "Spectrum"};
        for (int i = 0; i < 3; ++i)
        {
            ViewItem *item = createMenuItem<ViewItem>(viewNames[i]);
            item->module = module;
            item->mode = (LSDrone::ViewMode)i;
            menu->addChild(item);
        }
    }
};

Model *modelLSDrone = createModel<LSDrone, LSDroneWidget>("LSDrone");