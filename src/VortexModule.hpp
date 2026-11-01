#pragma once

#include "plugin.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace Vortex {

static constexpr int NUM_CHANNELS = 16;

// ---------- Dattorro Plate Reverb Engine with Echo Mode ----------
struct DattorroEngine {
    // Reverb tank delay lines (stereo)
    std::vector<float> delayL, delayR;
    int writePos = 0;

    // Echo mode delay line (stereo)
    std::vector<float> echoDelayL, echoDelayR;
    int echoWritePos = 0;

    // Allpass filters for diffuser and tank modulation
    struct Allpass {
        std::vector<float> buffer;
        int pos = 0;
        float feedback = 0.7f;
        void resize(int sz) { buffer.assign(sz, 0.0f); pos = 0; }
        float process(float in) {
            float out = buffer[pos];
            buffer[pos] = in + out * feedback;
            if (++pos >= (int)buffer.size()) pos = 0;
            out = std::tanh(out);
            return out;
        }
    };

    Allpass diffuser[4];
    Allpass modAP_L, modAP_R;

    struct OnePole {
        float z = 0.0f;
        float process(float in, float cutoffNorm) {
            float a = std::exp(-2.0f * (float)M_PI * cutoffNorm);
            z = in * (1.0f - a) + z * a;
            return z;
        }
    };
    OnePole lowCutInput, lowCutReverb, highShelf;

    float lfoPhase = 0.0f;

    // Parameters
    float preDelayMs = 0.0f;
    float size = 0.6f;
    float decay = 0.5f;
    float diffusion = 0.7f;
    float highDamp = 0.5f;
    float lowCutFreq = 20.0f;
    float modSpeed = 1.0f;
    float modDepth = 0.0f;
    float dryWet = 0.5f;
    bool freeze = false;
    bool echoMode = false;          // new: true = delay, false = reverb

    // Echo mode parameters
    float echoTimeMs = 0.0f;
    float echoFeedback = 0.0f;
    float echoLevel = 0.0f;

    // Pre‑delay buffers
    std::vector<float> preBufL, preBufR;
    int preIdx = 0;
    int preLen = 0;

    // Internal state
    float tankL = 0.0f, tankR = 0.0f;

    void init(float sampleRate) {
        resizeDelays(sampleRate);
        reset();
    }

    void reset() {
        for (auto& ap : diffuser) std::fill(ap.buffer.begin(), ap.buffer.end(), 0.0f);
        std::fill(modAP_L.buffer.begin(), modAP_L.buffer.end(), 0.0f);
        std::fill(modAP_R.buffer.begin(), modAP_R.buffer.end(), 0.0f);
        std::fill(preBufL.begin(), preBufL.end(), 0.0f);
        std::fill(preBufR.begin(), preBufR.end(), 0.0f);
        std::fill(delayL.begin(), delayL.end(), 0.0f);
        std::fill(delayR.begin(), delayR.end(), 0.0f);
        std::fill(echoDelayL.begin(), echoDelayL.end(), 0.0f);
        std::fill(echoDelayR.begin(), echoDelayR.end(), 0.0f);
        tankL = tankR = 0.0f;
        lfoPhase = 0.0f;
        lowCutInput.z = lowCutReverb.z = highShelf.z = 0.0f;
        writePos = 0;
        echoWritePos = 0;
        preIdx = 0;
    }

    void resizeDelays(float sampleRate) {
        // Reverb tank
        float tankDelayMs = 20.0f + size * 130.0f;
        int tankSamples = std::max(64, (int)(tankDelayMs * 0.001f * sampleRate));
        delayL.assign(tankSamples, 0.0f);
        delayR.assign(tankSamples, 0.0f);
        writePos = 0;

        // Diffuser allpass lengths
        int diffLengths[4] = { 142, 107, 379, 277 };
        for (int i = 0; i < 4; ++i) {
            int len = (int)(diffLengths[i] * (0.5f + size) * sampleRate / 44100.0f);
            len = std::max(8, len);
            diffuser[i].resize(len);
            diffuser[i].feedback = 0.5f + diffusion * 0.4f;
        }

        // Modulated allpass length (5 ms)
        int apLen = (int)(0.005f * sampleRate);
        apLen = std::max(16, apLen);
        modAP_L.resize(apLen);
        modAP_R.resize(apLen);
        modAP_L.feedback = 0.7f;
        modAP_R.feedback = 0.7f;

        // Echo delay buffer (max 1 second)
        int echoMaxSamples = (int)(sampleRate * 1.0f);
        echoDelayL.assign(echoMaxSamples, 0.0f);
        echoDelayR.assign(echoMaxSamples, 0.0f);
        echoWritePos = 0;

        // Pre‑delay (max 500 ms)
        preLen = (int)(sampleRate * 0.5f);
        preBufL.assign(preLen, 0.0f);
        preBufR.assign(preLen, 0.0f);
        preIdx = 0;
    }

    void process(float sampleRate, float inL, float inR, float& outL, float& outR) {
        // ----- Pre‑delay -----
        int preSamples = (int)(preDelayMs * 0.001f * sampleRate);
        preSamples = rack::math::clamp(preSamples, 0, preLen - 1);
        float delayedL = preBufL[(preIdx - preSamples + preLen) % preLen];
        float delayedR = preBufR[(preIdx - preSamples + preLen) % preLen];
        preBufL[preIdx] = inL;
        preBufR[preIdx] = inR;
        preIdx = (preIdx + 1) % preLen;

        if (!echoMode) {
            // ----- REVERB MODE (original Dattorro) -----
            float input = (delayedL + delayedR) * 0.5f;
            float lowCutNorm = lowCutFreq / sampleRate;
            lowCutNorm = rack::math::clamp(lowCutNorm, 0.0f, 0.49f);
            input = lowCutInput.process(input, lowCutNorm);
            float diffused = input;
            for (int i = 0; i < 4; ++i) diffused = diffuser[i].process(diffused);

            float lfo = std::sin(2.0f * (float)M_PI * lfoPhase);
            float modSamples = modDepth * 0.001f * sampleRate;
            float modOffset = lfo * modSamples;
            int maxOffset = (int)delayL.size() / 4;
            modOffset = rack::math::clamp(modOffset, (float)-maxOffset, (float)maxOffset);
            int readPos = (writePos - (int)(delayL.size() * 0.3f) + (int)modOffset + delayL.size()) % delayL.size();
            float readL = delayL[readPos];
            float readR = delayR[readPos];

            float apL = modAP_L.process(readL);
            float apR = modAP_R.process(readR);
            float fbGain = freeze ? 0.99f : decay * 0.85f;
            float fbL = (apL - apR) * fbGain;
            float fbR = (apR + apL) * fbGain;
            tankL = rack::math::clamp(diffused + fbL, -2.0f, 2.0f);
            tankR = rack::math::clamp(diffused + fbR, -2.0f, 2.0f);
            delayL[writePos] = tankL;
            delayR[writePos] = tankR;
            writePos = (writePos + 1) % (int)delayL.size();

            float reverbL = lowCutReverb.process(readL, lowCutNorm);
            float reverbR = lowCutReverb.process(readR, lowCutNorm);
            float highCutoff = 5000.0f + highDamp * 15000.0f;
            float highNorm = highCutoff / sampleRate;
            highNorm = rack::math::clamp(highNorm, 0.0f, 0.49f);
            reverbL = highShelf.process(reverbL, highNorm);
            reverbR = highShelf.process(reverbR, highNorm);

            float wetL = reverbL;
            float wetR = reverbR;
            outL = inL * (1.0f - dryWet) + wetL * dryWet;
            outR = inR * (1.0f - dryWet) + wetR * dryWet;
        } else {
            // ----- ECHO MODE (stereo delay with feedback) -----
            int echoSamples = (int)(echoTimeMs * 0.001f * sampleRate);
            echoSamples = rack::math::clamp(echoSamples, 0, (int)echoDelayL.size() - 1);
            float echoOutL = echoDelayL[(echoWritePos - echoSamples + echoDelayL.size()) % echoDelayL.size()];
            float echoOutR = echoDelayR[(echoWritePos - echoSamples + echoDelayR.size()) % echoDelayR.size()];

            // Feedback + pre‑delayed signal
            float feedbackL = delayedL + echoOutL * echoFeedback;
            float feedbackR = delayedR + echoOutR * echoFeedback;
            feedbackL = rack::math::clamp(feedbackL, -5.0f, 5.0f);
            feedbackR = rack::math::clamp(feedbackR, -5.0f, 5.0f);
            echoDelayL[echoWritePos] = feedbackL;
            echoDelayR[echoWritePos] = feedbackR;
            echoWritePos = (echoWritePos + 1) % (int)echoDelayL.size();

            // Mix echo with dry (using echo level, but also allow dry/wet)
            float wetL = echoOutL * echoLevel;
            float wetR = echoOutR * echoLevel;
            outL = inL * (1.0f - dryWet) + wetL * dryWet;
            outR = inR * (1.0f - dryWet) + wetR * dryWet;
        }

        outL = rack::math::clamp(outL, -10.0f, 10.0f);
        outR = rack::math::clamp(outR, -10.0f, 10.0f);

        // LFO update (only needed for reverb mode, but harmless in echo)
        lfoPhase += modSpeed / sampleRate;
        if (lfoPhase >= 1.0f) lfoPhase -= 1.0f;
    }
};

} // namespace Vortex