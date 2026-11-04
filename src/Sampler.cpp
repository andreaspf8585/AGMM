#include "plugin.hpp"
#include <vector>
#include <string>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <fstream>
#include <osdialog.h>

// ------------------------------------------------------------
// Constants
// ------------------------------------------------------------

// Samples are stored in volts. Rack's full scale is 5 V, so a WAV with a peak of 1.0
// is loaded as 5 V and saved back the same way. Set this to 1.f to get the old
// behaviour (loaded files 14 dB quieter, recordings clipped on save).
static const float SP_WAV_FULLSCALE = 5.f;
static const int SP_PEAK_BINS = 1024;          // waveform overview resolution
static const float SP_MAX_REC_SECONDS = 300.f; // recording stops by itself after this
static const float SP_PI = 3.14159265358979323846f;

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float SP_PANEL_W = RACK_GRID_WIDTH * 36; // 540
static const float SP_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Waveform display
static const float SP_DISP_X = 130.f;
static const float SP_DISP_Y = 46.f;
static const float SP_DISP_W = 280.f;
static const float SP_DISP_H = 104.f;
static const float SP_WF_X0 = 18.f; // waveform area inside the display
static const float SP_WF_Y0 = 18.f;
static const float SP_WF_H = 68.f;
static const int SP_WF_W = 244;

// Centre control grid: 5 columns x 2 rows of 58 x 78 cells
static const float SP_CX[5] = {149.f, 210.f, 271.f, 332.f, 393.f};
static const float SP_R1 = 176.f; // top of row 1
static const float SP_R2 = 260.f; // top of row 2
static const float SP_CELL_W = 58.f;
static const float SP_CELL_H = 78.f;
static const float SP_KNOB_DY = 18.f; // knob centre below cell top
static const float SP_LABEL_DY = 39.f;
static const float SP_JACK_DY = 56.f; // CV jack / trimpot / value readout
static const float SP_TINY_DY = 73.f;
static const float SP_SW_DY = 24.f; // switch centre
static const float SP_SWREAD_DY = 51.f;
static const float SP_SWLABEL_DY = 66.f;

// Side blocks
static const float SP_LCOL[2] = {34.f, 90.f};
static const float SP_RCOL[2] = {452.f, 508.f};

// Left block rows
static const float SP_Y_MIX = 148.f;  // MIX knob + CLEAR button
static const float SP_Y_SOS = 200.f;  // SOS + LINE buttons
static const float SP_Y_REC = 284.f;  // REC button + trigger jack
static const float SP_Y_PLAY = 336.f; // PLAY button + trigger jack
// Right block bottom row
static const float SP_Y_RKNOB = 336.f; // SMOOTH + SOS FB knobs

// Sample info strip
static const float SP_INFO_X = 120.f;
static const float SP_INFO_Y = 344.f;
static const float SP_INFO_W = 302.f;
static const float SP_INFO_H = 22.f;

// ------------------------------------------------------------
// Simple Attack-Decay/Release Envelope
// ------------------------------------------------------------
struct SamplerEnvelope
{
    enum Stage
    {
        IDLE,
        ATTACK,
        DECAY
    };
    Stage stage = IDLE;
    float level = 0.0f;
    float attackTime = 0.0f; // seconds
    float decayTime = 0.5f;  // seconds

    void trigger()
    {
        stage = ATTACK;
    }

    void reset()
    {
        stage = IDLE;
        level = 0.0f;
    }

    float process(float dt, bool sustained)
    {
        if (stage == ATTACK)
        {
            if (attackTime <= 0.001f)
            {
                level = 1.0f;
                stage = DECAY;
            }
            else
            {
                level += dt / attackTime;
                if (level >= 1.0f)
                {
                    level = 1.0f;
                    stage = DECAY;
                }
            }
        }
        else if (stage == DECAY)
        {
            if (sustained)
            {
                level = 1.0f;
            }
            else
            {
                if (decayTime <= 0.001f)
                {
                    level = 0.0f;
                    stage = IDLE;
                }
                else
                {
                    level -= dt / decayTime;
                    if (level <= 0.0f)
                    {
                        level = 0.0f;
                        stage = IDLE;
                    }
                }
            }
        }
        else
        {
            level = 0.0f;
        }
        return level;
    }
};

// ------------------------------------------------------------
// Self-contained WAV loading and saving
// ------------------------------------------------------------
static bool spLoadWav(const std::string &path, std::vector<float> &left, std::vector<float> &right, float *fileSampleRate)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        return false;

    char riff[4];
    file.read(riff, 4);
    if (!file || std::strncmp(riff, "RIFF", 4) != 0)
        return false;
    file.seekg(4, std::ios::cur); // skip total size
    char wave[4];
    file.read(wave, 4);
    if (!file || std::strncmp(wave, "WAVE", 4) != 0)
        return false;

    uint16_t formatType = 0, channels = 0, bitsPerSample = 0;
    uint32_t sampleRate = 0;
    bool foundFmt = false, foundData = false;

    char chunkId[4];
    uint32_t chunkSize = 0;
    while (file.read(chunkId, 4))
    {
        file.read(reinterpret_cast<char *>(&chunkSize), 4);
        if (!file)
            break;

        if (std::strncmp(chunkId, "fmt ", 4) == 0)
        {
            if (chunkSize < 16)
                return false;
            file.read(reinterpret_cast<char *>(&formatType), 2);
            file.read(reinterpret_cast<char *>(&channels), 2);
            file.read(reinterpret_cast<char *>(&sampleRate), 4);
            file.seekg(6, std::ios::cur); // skip byte rate and block align
            file.read(reinterpret_cast<char *>(&bitsPerSample), 2);

            size_t remaining = chunkSize - 16;
            if (formatType == 0xFFFE && remaining >= 24)
            {
                // WAVE_FORMAT_EXTENSIBLE: the real format is the first 2 bytes of the sub-format GUID
                file.seekg(8, std::ios::cur);
                uint16_t sub = 0;
                file.read(reinterpret_cast<char *>(&sub), 2);
                formatType = sub;
                file.seekg(remaining - 10, std::ios::cur);
            }
            else if (remaining > 0)
            {
                file.seekg(remaining, std::ios::cur);
            }
            if (chunkSize & 1)
                file.seekg(1, std::ios::cur);
            foundFmt = true;
        }
        else if (std::strncmp(chunkId, "data", 4) == 0)
        {
            if (!foundFmt || channels == 0 || bitsPerSample == 0 || chunkSize > 500000000)
                return false;

            size_t bytesPerSample = bitsPerSample / 8;
            if (bytesPerSample == 0)
                return false;
            size_t frameBytes = (size_t)channels * bytesPerSample;

            std::vector<char> rawBuffer(chunkSize);
            file.read(rawBuffer.data(), chunkSize);
            size_t got = (size_t)file.gcount(); // tolerate truncated files
            size_t numSamples = got / frameBytes;

            left.clear();
            right.clear();
            left.reserve(numSamples);
            right.reserve(numSamples);

            for (size_t i = 0; i < numSamples; ++i)
            {
                float lVal = 0.0f;
                float rVal = 0.0f;

                for (int c = 0; c < channels && c < 2; ++c)
                {
                    size_t offset = i * frameBytes + (size_t)c * bytesPerSample;
                    float sampleVal = 0.0f;
                    if (formatType == 1)
                    { // PCM
                        if (bitsPerSample == 8)
                        {
                            sampleVal = ((int)(uint8_t)rawBuffer[offset] - 128) / 128.f;
                        }
                        else if (bitsPerSample == 16)
                        {
                            int16_t s16;
                            std::memcpy(&s16, &rawBuffer[offset], 2);
                            sampleVal = s16 / 32768.f;
                        }
                        else if (bitsPerSample == 24)
                        {
                            int32_t s24 = 0;
                            s24 |= (static_cast<uint8_t>(rawBuffer[offset]) << 0);
                            s24 |= (static_cast<uint8_t>(rawBuffer[offset + 1]) << 8);
                            s24 |= (static_cast<uint8_t>(rawBuffer[offset + 2]) << 16);
                            if (s24 & 0x800000)
                                s24 |= 0xFF000000;
                            sampleVal = s24 / 8388608.f;
                        }
                        else if (bitsPerSample == 32)
                        {
                            int32_t s32;
                            std::memcpy(&s32, &rawBuffer[offset], 4);
                            sampleVal = s32 / 2147483648.f;
                        }
                    }
                    else if (formatType == 3)
                    { // IEEE Float
                        if (bitsPerSample == 32)
                        {
                            float f32;
                            std::memcpy(&f32, &rawBuffer[offset], 4);
                            sampleVal = f32;
                        }
                    }
                    if (c == 0)
                        lVal = sampleVal;
                    else
                        rVal = sampleVal;
                }

                left.push_back(lVal);
                right.push_back(channels >= 2 ? rVal : lVal);
            }

            if (fileSampleRate)
                *fileSampleRate = static_cast<float>(sampleRate);
            foundData = true;
            break;
        }
        else
        {
            file.seekg(chunkSize + (chunkSize & 1), std::ios::cur);
        }
    }

    return foundFmt && foundData && !left.empty();
}

static bool spSaveWav(const std::string &path, const std::vector<float> &left, const std::vector<float> &right, float sampleRate)
{
    if (left.empty())
        return false;
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open())
        return false;

    uint32_t numSamples = (uint32_t)left.size();
    uint16_t channels = 2;
    uint32_t sampleRateVal = (uint32_t)sampleRate;
    uint16_t bitsPerSample = 16;
    uint32_t bytesPerSample = bitsPerSample / 8;
    uint32_t subChunk2Size = numSamples * channels * bytesPerSample;
    uint32_t chunkSize = 36 + subChunk2Size;

    file.write("RIFF", 4);
    file.write(reinterpret_cast<const char *>(&chunkSize), 4);
    file.write("WAVE", 4);
    file.write("fmt ", 4);

    uint32_t subChunk1Size = 16;
    uint16_t audioFormat = 1; // PCM
    uint32_t byteRate = sampleRateVal * channels * bytesPerSample;
    uint16_t blockAlign = channels * bytesPerSample;

    file.write(reinterpret_cast<const char *>(&subChunk1Size), 4);
    file.write(reinterpret_cast<const char *>(&audioFormat), 2);
    file.write(reinterpret_cast<const char *>(&channels), 2);
    file.write(reinterpret_cast<const char *>(&sampleRateVal), 4);
    file.write(reinterpret_cast<const char *>(&byteRate), 4);
    file.write(reinterpret_cast<const char *>(&blockAlign), 2);
    file.write(reinterpret_cast<const char *>(&bitsPerSample), 2);

    file.write("data", 4);
    file.write(reinterpret_cast<const char *>(&subChunk2Size), 4);

    for (size_t i = 0; i < numSamples; ++i)
    {
        float l = left[i] / SP_WAV_FULLSCALE;
        float r = (i < right.size() ? right[i] : left[i]) / SP_WAV_FULLSCALE;

        int16_t l16 = (int16_t)clamp(l * 32767.f, -32768.f, 32767.f);
        int16_t r16 = (int16_t)clamp(r * 32767.f, -32768.f, 32767.f);

        file.write(reinterpret_cast<const char *>(&l16), 2);
        file.write(reinterpret_cast<const char *>(&r16), 2);
    }

    return file.good();
}

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Sampler : Module
{
    // NOTE: new params are appended at the end so old patches keep their values.
    // LINE_SWITCH reuses the slot of the old MONITOR switch.
    enum ParamId
    {
        SPEED_PARAM,
        SPEED_CV_ATTEN,
        START_PARAM,
        START_CV_ATTEN,
        LENGTH_PARAM,
        LENGTH_CV_ATTEN,
        ATTACK_PARAM,
        DECAY_PARAM,
        PITCH_MODE_PARAM,
        PLAY_MODE_PARAM,
        DIRECTION_PARAM,
        REC_BUTTON,
        PLAY_BUTTON,
        LEVEL_PARAM,
        XFADE_PARAM,
        LINE_SWITCH, // was MONITOR_SWITCH
        CLEAR_BUTTON,
        MIX_PARAM,      // new: input / playback mix
        SOS_SWITCH,     // new: sound on sound (overdub)
        DECLICK_PARAM,  // new: start / end smoothing (ms)
        SOS_FDBK_PARAM, // new: level of the old material when overdubbing
        PARAMS_LEN
    };
    enum InputId
    {
        LEFT_INPUT,
        RIGHT_INPUT,
        RECORD_TRIGGER,
        PLAY_TRIGGER,
        SPEED_CV_INPUT,
        START_CV_INPUT,
        LENGTH_CV_INPUT,
        PITCH_CV_INPUT,
        GATE_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        LEFT_OUTPUT,
        RIGHT_OUTPUT,
        EOC_OUTPUT,
        PLAY_GATE_OUTPUT,
        REC_GATE_OUTPUT,
        ENV_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        REC_LIGHT,
        PLAY_LIGHT,
        PITCH_MODE_LIGHT,
        LOOP_MODE_LIGHT,
        SOS_LIGHT,
        LINE_LIGHT,
        LIGHTS_LEN
    };

    // State variables - atomics for thread-safe UI rendering
    std::atomic<bool> isRecording{false};
    std::atomic<bool> isPlaying{false};
    std::atomic<bool> sosOverdub{false}; // the current recording is an overdub
    std::atomic<float> playhead{0.0f};
    std::atomic<size_t> mainBufferSize{0};
    std::atomic<size_t> recBufferSize{0};
    std::atomic<float> loopStartNorm{0.0f};
    std::atomic<float> loopLengthNorm{1.0f};
    std::atomic<float> bufferRate{44100.0f}; // sample rate of the stored audio

    std::mutex bufferMutex;
    std::vector<float> leftSamples;
    std::vector<float> rightSamples;
    std::vector<float> tempLeft;
    std::vector<float> tempRight;

    // Waveform overview for the display (min / max per bin)
    std::vector<float> peakMin, peakMax;       // finished sample
    std::vector<float> recPeakMin, recPeakMax; // while recording (10 ms buckets)
    float recBucketMin = 0.f, recBucketMax = 0.f;
    int recBucketCount = 0;

    float sampleRate = 44100.0f; // engine sample rate

    bool lastRecordTrigger = false;
    bool lastPlayTrigger = false;
    bool lastClear = false;
    bool gateActive = false;
    bool pingPongForward = true;
    bool autoPlay = true; // start playing when a recording ends

    long sosLastIdx = -1; // last buffer index written by the overdub
    float fadeIn = 1.f;   // 0..1 start fade (declick)
    float mixSm = 0.f;    // smoothed input/playback mix

    SamplerEnvelope env;
    dsp::PulseGenerator eocPulse;
    std::string lastPath = "";

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float meterInL = 0.f, meterInR = 0.f, meterOutL = 0.f, meterOutR = 0.f;
    float displaySpeed = 1.f;

    Sampler()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(SPEED_PARAM, -3.f, 3.f, 1.f, "Playback speed / tuning", "x");
        configParam(SPEED_CV_ATTEN, -1.f, 1.f, 1.f, "Speed CV amount", "%", 0.f, 100.f);
        configParam(START_PARAM, 0.f, 1.f, 0.f, "Start position", "%", 0.f, 100.f);
        configParam(START_CV_ATTEN, -1.f, 1.f, 1.f, "Start CV amount", "%", 0.f, 100.f);
        configParam(LENGTH_PARAM, 0.f, 1.f, 1.f, "Playback length", "%", 0.f, 100.f);
        configParam(LENGTH_CV_ATTEN, -1.f, 1.f, 1.f, "Length CV amount", "%", 0.f, 100.f);
        configParam(ATTACK_PARAM, 0.f, 5.f, 0.f, "Envelope attack", " s");
        configParam(DECAY_PARAM, 0.f, 10.f, 10.f, "Envelope decay / release (10 s = sustain)", " s");
        configSwitch(PITCH_MODE_PARAM, 0.f, 1.f, 0.f, "Tuning mode", {"Speed (knob + CV)", "V/Oct"});
        configSwitch(PLAY_MODE_PARAM, 0.f, 2.f, 0.f, "Play mode", {"One-shot", "Loop", "Ping-pong"});
        configSwitch(DIRECTION_PARAM, 0.f, 1.f, 0.f, "Direction", {"Forward", "Reverse"});
        configSwitch(REC_BUTTON, 0.f, 1.f, 0.f, "Record", {"Off", "Recording"});
        configSwitch(PLAY_BUTTON, 0.f, 1.f, 0.f, "Play", {"Stopped", "Playing"});
        configParam(LEVEL_PARAM, 0.f, 2.f, 1.f, "Output level", "%", 0.f, 100.f);
        configParam(XFADE_PARAM, 1.f, 100.f, 15.f, "Loop crossfade", " ms");
        configSwitch(LINE_SWITCH, 0.f, 1.f, 0.f, "Line monitor (input stays audible through MIX when not recording)", {"Off", "On"});
        configButton(CLEAR_BUTTON, "Clear sample");

        configParam(MIX_PARAM, 0.f, 1.f, 0.f, "Mix (0% = playback, 100% = live input)", "%", 0.f, 100.f);
        configSwitch(SOS_SWITCH, 0.f, 1.f, 0.f, "Sound on sound (record overdubs onto the sample)", {"Off", "On"});
        configParam(DECLICK_PARAM, 0.f, 100.f, 3.f, "Start / end smoothing (anti-click fade)", " ms");
        configParam(SOS_FDBK_PARAM, 0.f, 1.f, 1.f, "Sound on sound: old material level", "%", 0.f, 100.f);

        configInput(LEFT_INPUT, "Left (mono if the right input is empty)");
        configInput(RIGHT_INPUT, "Right");
        configInput(RECORD_TRIGGER, "Record trigger (toggles)");
        configInput(PLAY_TRIGGER, "Play trigger (retriggers)");
        configInput(SPEED_CV_INPUT, "Speed CV (0 to 10 V)");
        configInput(START_CV_INPUT, "Start position CV (+-5 V)");
        configInput(LENGTH_CV_INPUT, "Length CV (+-5 V)");
        configInput(PITCH_CV_INPUT, "Pitch CV (1 V/oct)");
        configInput(GATE_INPUT, "Envelope sustain gate");

        configOutput(LEFT_OUTPUT, "Left");
        configOutput(RIGHT_OUTPUT, "Right");
        configOutput(EOC_OUTPUT, "End of cycle trigger (also on every loop wrap)");
        configOutput(PLAY_GATE_OUTPUT, "Playing gate");
        configOutput(REC_GATE_OUTPUT, "Recording gate");
        configOutput(ENV_OUTPUT, "Envelope (0 to 10 V)");

        configLight(REC_LIGHT, "Recording");
        configLight(PLAY_LIGHT, "Playing");
        configLight(SOS_LIGHT, "Sound on sound");
        configLight(LINE_LIGHT, "Line monitor");

        tempLeft.reserve(48000 * 10);
        tempRight.reserve(48000 * 10);
        recPeakMin.reserve(4096);
        recPeakMax.reserve(4096);
    }

    // ---------- helpers (the *Locked ones expect bufferMutex to be held) ----------
    inline float getInterpolated(const std::vector<float> &buffer, float pos)
    {
        size_t size = buffer.size();
        if (size == 0)
            return 0.0f;
        if (pos < 0.0f)
            pos = 0.0f;
        if (pos >= (float)size)
            pos = (float)size - 0.0001f;
        size_t idx = (size_t)pos;
        float frac = pos - idx;
        size_t nextIdx = (idx + 1 < size) ? idx + 1 : idx;
        return buffer[idx] * (1.0f - frac) + buffer[nextIdx] * frac;
    }

    void rebuildPeaksLocked()
    {
        size_t n = leftSamples.size();
        peakMin.assign(n ? SP_PEAK_BINS : 0, 0.f);
        peakMax.assign(n ? SP_PEAK_BINS : 0, 0.f);
        if (n == 0)
            return;
        for (int b = 0; b < SP_PEAK_BINS; ++b)
        {
            size_t s = n * (size_t)b / SP_PEAK_BINS;
            size_t e = n * (size_t)(b + 1) / SP_PEAK_BINS;
            if (e <= s)
                e = std::min(s + 1, n);
            float mn = 0.f, mx = 0.f;
            for (size_t j = s; j < e; ++j)
            {
                float l = leftSamples[j];
                float r = j < rightSamples.size() ? rightSamples[j] : l;
                mn = std::min(mn, std::min(l, r));
                mx = std::max(mx, std::max(l, r));
            }
            peakMin[b] = mn;
            peakMax[b] = mx;
        }
    }

    void clearLocked()
    {
        isRecording = false;
        isPlaying = false;
        sosOverdub = false;
        leftSamples.clear();
        rightSamples.clear();
        tempLeft.clear();
        tempRight.clear();
        peakMin.clear();
        peakMax.clear();
        recPeakMin.clear();
        recPeakMax.clear();
        mainBufferSize = 0;
        recBufferSize = 0;
        playhead = 0.f;
        lastPath = "";
        env.reset();
    }

    void startRecordingLocked(bool &triggerPlay, bool sos)
    {
        if (sos && !leftSamples.empty())
        {
            // Sound on sound: keep the sample, keep playing, overdub at the playhead
            sosOverdub = true;
            sosLastIdx = -1;
            recBufferSize = 0;
            isRecording = true;
            if (!isPlaying.load())
            {
                isPlaying = true;
                triggerPlay = true;
                params[PLAY_BUTTON].setValue(1.f);
            }
            return;
        }
        sosOverdub = false;
        tempLeft.clear();
        tempRight.clear();
        recPeakMin.clear();
        recPeakMax.clear();
        recBucketCount = 0;
        recBufferSize = 0;
        isRecording = true;
        isPlaying = false;
        params[PLAY_BUTTON].setValue(0.f);
        env.reset();
    }

    void stopRecordingLocked(bool &triggerPlay)
    {
        if (sosOverdub.load())
        {
            sosOverdub = false;
            isRecording = false;
            lastPath = ""; // the audio no longer matches the file
            rebuildPeaksLocked();
            return;
        }
        isRecording = false;
        if (!tempLeft.empty())
        {
            leftSamples = std::move(tempLeft);
            rightSamples = std::move(tempRight);
            tempLeft.clear();
            tempRight.clear();
            bufferRate = sampleRate;
            lastPath = ""; // the audio now only exists in memory
            mainBufferSize = leftSamples.size();
            rebuildPeaksLocked();
            playhead = 0.f;
            if (autoPlay)
            {
                isPlaying = true;
                triggerPlay = true;
                params[PLAY_BUTTON].setValue(1.f);
            }
        }
        recBufferSize = 0;
    }

    // Sound on sound write: new = old * fb + input, for every buffer index the playhead crossed
    void overdubLocked(long idx, float inL, float inR, float fb)
    {
        long n = (long)leftSamples.size();
        if (n == 0)
            return;
        idx = std::max(0L, std::min(idx, n - 1));
        if (idx == sosLastIdx)
            return; // slow playback: write each sample only once

        long lo, hi;
        if (sosLastIdx < 0 || std::labs(idx - sosLastIdx) > 64)
        {
            lo = hi = idx; // first sample or a loop wrap
        }
        else if (idx > sosLastIdx)
        {
            lo = sosLastIdx + 1;
            hi = idx;
        }
        else
        {
            lo = idx;
            hi = sosLastIdx - 1;
        }
        lo = std::max(0L, lo);
        hi = std::min(hi, n - 1);

        const bool haveR = rightSamples.size() == leftSamples.size();
        for (long i = lo; i <= hi; ++i)
        {
            leftSamples[i] = clamp(leftSamples[i] * fb + inL, -10.f, 10.f);
            if (haveR)
                rightSamples[i] = clamp(rightSamples[i] * fb + inR, -10.f, 10.f);
        }

        // keep the waveform overview roughly up to date (it only grows until recording stops)
        if (peakMin.size() == (size_t)SP_PEAK_BINS)
        {
            size_t bin = (size_t)idx * SP_PEAK_BINS / (size_t)n;
            if (bin >= (size_t)SP_PEAK_BINS)
                bin = SP_PEAK_BINS - 1;
            float mn = std::min(leftSamples[idx], haveR ? rightSamples[idx] : leftSamples[idx]);
            float mx = std::max(leftSamples[idx], haveR ? rightSamples[idx] : leftSamples[idx]);
            peakMin[bin] = std::min(peakMin[bin], mn);
            peakMax[bin] = std::max(peakMax[bin], mx);
        }
        sosLastIdx = idx;
    }

    // ---------- sample editing (right-click menu) ----------
    bool loadFile(const std::string &path)
    {
        std::vector<float> left, right;
        float fileRate = 44100.0f;
        if (!spLoadWav(path, left, right, &fileRate))
            return false;
        for (float &v : left)
            v *= SP_WAV_FULLSCALE;
        for (float &v : right)
            v *= SP_WAV_FULLSCALE;

        std::lock_guard<std::mutex> lock(bufferMutex);
        leftSamples = std::move(left);
        rightSamples = std::move(right);
        mainBufferSize = leftSamples.size();
        bufferRate = fileRate;
        lastPath = path;
        playhead = 0.0f;
        rebuildPeaksLocked();
        return true;
    }

    void loadWavDialog()
    {
        osdialog_filters *filters = osdialog_filters_parse("WAV:wav");
        char *path = osdialog_file(OSDIALOG_OPEN, NULL, NULL, filters);
        if (filters)
            osdialog_filters_free(filters);
        if (path)
        {
            std::string pathStr(path);
            free(path);
            loadFile(pathStr);
        }
    }

    void saveWavDialog()
    {
        osdialog_filters *filters = osdialog_filters_parse("WAV:wav");
        char *path = osdialog_file(OSDIALOG_SAVE, NULL, NULL, filters);
        if (filters)
            osdialog_filters_free(filters);
        if (path)
        {
            std::string pathStr(path);
            free(path);
            if (pathStr.size() < 4 || pathStr.substr(pathStr.size() - 4) != ".wav")
                pathStr += ".wav";

            std::vector<float> left, right;
            float rate;
            {
                std::lock_guard<std::mutex> lock(bufferMutex);
                left = leftSamples;
                right = rightSamples;
                rate = bufferRate;
            }
            spSaveWav(pathStr, left, right, rate); // written outside the lock
        }
    }

    void clearSample()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        clearLocked();
    }

    void normalizeSample()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        float peak = 0.f;
        for (float v : leftSamples)
            peak = std::max(peak, std::fabs(v));
        for (float v : rightSamples)
            peak = std::max(peak, std::fabs(v));
        if (peak < 1e-6f)
            return;
        float g = SP_WAV_FULLSCALE / peak;
        for (float &v : leftSamples)
            v *= g;
        for (float &v : rightSamples)
            v *= g;
        lastPath = "";
        rebuildPeaksLocked();
    }

    void reverseSample()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        if (leftSamples.empty())
            return;
        std::reverse(leftSamples.begin(), leftSamples.end());
        std::reverse(rightSamples.begin(), rightSamples.end());
        playhead = (float)leftSamples.size() - playhead.load();
        lastPath = "";
        rebuildPeaksLocked();
    }

    void trimToLoop()
    {
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            size_t size = leftSamples.size();
            if (size < 4)
                return;
            float s = loopStartNorm.load() * (float)size;
            float e = s + loopLengthNorm.load() * ((float)size - s);
            size_t si = (size_t)std::max(0.f, s);
            size_t ei = std::min(size, (size_t)std::max(0.f, e));
            if (ei <= si + 1)
                return;
            leftSamples = std::vector<float>(leftSamples.begin() + si, leftSamples.begin() + ei);
            rightSamples = std::vector<float>(rightSamples.begin() + si, rightSamples.begin() + ei);
            mainBufferSize = leftSamples.size();
            playhead = 0.f;
            lastPath = "";
            rebuildPeaksLocked();
        }
        paramQuantities[START_PARAM]->setValue(0.f);
        paramQuantities[LENGTH_PARAM]->setValue(1.f);
    }

    // ---------- DSP ----------
    void process(const ProcessArgs &args) override
    {
        sampleRate = args.sampleRate;
        float dt = args.sampleTime;

        // Read options
        bool pitchMode = params[PITCH_MODE_PARAM].getValue() > 0.5f;
        int playMode = clamp((int)roundf(params[PLAY_MODE_PARAM].getValue()), 0, 2);
        bool reverseDir = params[DIRECTION_PARAM].getValue() > 0.5f;
        bool lineOn = params[LINE_SWITCH].getValue() > 0.5f;
        bool sosOn = params[SOS_SWITCH].getValue() > 0.5f;
        float level = params[LEVEL_PARAM].getValue();
        float xfadeMs = params[XFADE_PARAM].getValue();
        float mixKnob = params[MIX_PARAM].getValue();
        float sosFb = params[SOS_FDBK_PARAM].getValue();
        float smoothSec = params[DECLICK_PARAM].getValue() * 0.001f;

        // Start & Length inputs with CV
        float startKnob = params[START_PARAM].getValue();
        float startCv = inputs[START_CV_INPUT].getVoltage();
        float startCvAtten = params[START_CV_ATTEN].getValue();
        float startNorm = clamp(startKnob + (startCv / 5.0f) * startCvAtten, 0.0f, 1.0f);
        loopStartNorm.store(startNorm);

        float lengthKnob = params[LENGTH_PARAM].getValue();
        float lengthCv = inputs[LENGTH_CV_INPUT].getVoltage();
        float lengthCvAtten = params[LENGTH_CV_ATTEN].getValue();
        float lengthNorm = clamp(lengthKnob + (lengthCv / 5.0f) * lengthCvAtten, 0.0f, 1.0f);
        loopLengthNorm.store(lengthNorm);

        env.attackTime = params[ATTACK_PARAM].getValue();
        env.decayTime = params[DECAY_PARAM].getValue();

        // Speed / V/Oct calculation
        float totalSpeed = 1.0f;
        if (pitchMode)
        {
            float pitchCV = inputs[PITCH_CV_INPUT].getVoltage();
            float fineTune = params[SPEED_PARAM].getValue();
            totalSpeed = fineTune * powf(2.0f, pitchCV);
        }
        else
        {
            float speedKnob = params[SPEED_PARAM].getValue();
            totalSpeed = speedKnob;
            if (inputs[SPEED_CV_INPUT].isConnected())
            {
                float cvAmount = params[SPEED_CV_ATTEN].getValue();
                float cvVal = clamp(inputs[SPEED_CV_INPUT].getVoltage() / 5.f - 1.f, -1.f, 1.f);
                totalSpeed = speedKnob + cvVal * cvAmount;
            }
        }
        if (reverseDir)
            totalSpeed = -totalSpeed;
        totalSpeed = clamp(totalSpeed, -16.0f, 16.0f);
        displaySpeed = totalSpeed;

        // Inputs (a mono source on the left feeds both channels)
        float inL = inputs[LEFT_INPUT].getVoltage();
        float inR = inputs[RIGHT_INPUT].getNormalVoltage(inL);
        bool recordTrigNow = inputs[RECORD_TRIGGER].getVoltage() > 1.0f;
        bool playTrigNow = inputs[PLAY_TRIGGER].getVoltage() > 1.0f;
        bool gateConnected = inputs[GATE_INPUT].isConnected();
        bool gateInNow = gateConnected ? (inputs[GATE_INPUT].getVoltage() > 1.0f) : false;

        bool triggerPlay = false;
        const size_t maxRecSamples = (size_t)(SP_MAX_REC_SECONDS * sampleRate);
        const int recBucketLen = std::max(1, (int)(sampleRate * 0.01f));
        const float rateRatio = clamp(bufferRate.load() / sampleRate, 0.05f, 20.f);

        {
            std::lock_guard<std::mutex> lock(bufferMutex);

            // Clear button
            bool clearNow = params[CLEAR_BUTTON].getValue() > 0.5f;
            if (clearNow && !lastClear)
            {
                clearLocked();
                params[REC_BUTTON].setValue(0.f);
                params[PLAY_BUTTON].setValue(0.f);
            }
            lastClear = clearNow;

            // Recording: the button is the source of truth, a trigger toggles it
            bool recEdge = recordTrigNow && !lastRecordTrigger;
            lastRecordTrigger = recordTrigNow;
            bool wantRec = params[REC_BUTTON].getValue() > 0.5f;
            if (recEdge)
            {
                wantRec = !isRecording.load();
                params[REC_BUTTON].setValue(wantRec ? 1.f : 0.f);
            }
            if (wantRec != isRecording.load())
            {
                if (wantRec)
                    startRecordingLocked(triggerPlay, sosOn);
                else
                    stopRecordingLocked(triggerPlay);
            }

            if (isRecording && !sosOverdub.load())
            {
                if (tempLeft.size() >= maxRecSamples)
                {
                    params[REC_BUTTON].setValue(0.f); // stops on the next sample
                }
                else
                {
                    tempLeft.push_back(inL);
                    tempRight.push_back(inR);
                    recBufferSize = tempLeft.size();

                    float mn = std::min(inL, inR), mx = std::max(inL, inR);
                    if (recBucketCount == 0)
                    {
                        recBucketMin = mn;
                        recBucketMax = mx;
                    }
                    else
                    {
                        recBucketMin = std::min(recBucketMin, mn);
                        recBucketMax = std::max(recBucketMax, mx);
                    }
                    if (++recBucketCount >= recBucketLen)
                    {
                        recBucketCount = 0;
                        recPeakMin.push_back(recBucketMin);
                        recPeakMax.push_back(recBucketMax);
                    }
                }
            }
            // (sound on sound overdubs are written in the playback section below)

            // Playback transport
            bool playEdge = playTrigNow && !lastPlayTrigger;
            lastPlayTrigger = playTrigNow;
            if (!isRecording)
            {
                bool playBtn = params[PLAY_BUTTON].getValue() > 0.5f;
                bool hasSample = !leftSamples.empty();
                if (playEdge && hasSample)
                {
                    params[PLAY_BUTTON].setValue(1.f);
                    isPlaying = true;
                    triggerPlay = true;
                }
                else if (playBtn && !isPlaying.load() && hasSample)
                {
                    isPlaying = true;
                    triggerPlay = true;
                }
                else if (playBtn && !hasSample)
                {
                    params[PLAY_BUTTON].setValue(0.f);
                }
                else if (!playBtn && isPlaying.load())
                {
                    isPlaying = false;
                    env.reset();
                }
            }

            // Retrigger settings
            if (triggerPlay && !leftSamples.empty())
            {
                float size = (float)leftSamples.size();
                float startIdx = startNorm * size;
                float endIdx = startIdx + lengthNorm * (size - startIdx);
                if (endIdx > size)
                    endIdx = size;

                if (totalSpeed >= 0.0f)
                {
                    playhead = startIdx;
                }
                else
                {
                    playhead = endIdx - 0.0001f;
                }
                pingPongForward = (totalSpeed >= 0.0f);
                fadeIn = 0.f; // restart the anti-click fade
                sosLastIdx = -1;
                env.trigger();
            }
        }

        if (gateInNow && !gateActive)
        {
            env.trigger();
        }
        gateActive = gateInNow;

        // Envelope sustain state selection
        bool sustainActive = gateConnected ? gateInNow : (env.decayTime > 9.9f);

        // DSP playback
        float leftOut = 0.0f;
        float rightOut = 0.0f;
        float envOut = 0.0f;
        bool localPlaying = isPlaying.load();
        float localPlayhead = playhead.load();

        if (localPlaying)
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            size_t size = leftSamples.size();
            if (size > 0)
            {
                float sizeF = (float)size;
                float loopStart = startNorm * sizeF;
                float loopEnd = loopStart + lengthNorm * (sizeF - loopStart);

                if (loopStart >= sizeF)
                    loopStart = sizeF - 1.0f;
                if (loopEnd > sizeF)
                    loopEnd = sizeF;
                if (loopEnd <= loopStart + 1.0f)
                    loopEnd = loopStart + 1.0f;

                float activeLength = loopEnd - loopStart;
                int crossfadeLen = std::max(1, (int)(bufferRate.load() * xfadeMs * 0.001f));

                float speedVal = totalSpeed;
                if (playMode == 2)
                { // Ping Pong direction flip
                    speedVal = pingPongForward ? fabsf(totalSpeed) : -fabsf(totalSpeed);
                }
                localPlayhead += speedVal * rateRatio;

                // Loop endpoint checks
                bool endOfSample = false;
                if (speedVal >= 0.0f)
                {
                    if (localPlayhead >= loopEnd)
                    {
                        if (playMode == 0)
                        {
                            endOfSample = true;
                        }
                        else if (playMode == 1)
                        { // Normal Loop
                            localPlayhead -= activeLength;
                            if (localPlayhead < loopStart)
                                localPlayhead = loopStart;
                            eocPulse.trigger(0.001f);
                        }
                        else if (playMode == 2)
                        { // Ping Pong Wrap
                            pingPongForward = false;
                            localPlayhead = loopEnd - (localPlayhead - loopEnd);
                            if (localPlayhead < loopStart)
                                localPlayhead = loopStart;
                            eocPulse.trigger(0.001f);
                        }
                    }
                }
                else
                {
                    if (localPlayhead < loopStart)
                    {
                        if (playMode == 0)
                        {
                            endOfSample = true;
                        }
                        else if (playMode == 1)
                        { // Normal Loop
                            localPlayhead += activeLength;
                            if (localPlayhead >= loopEnd)
                                localPlayhead = loopEnd - 0.0001f;
                            eocPulse.trigger(0.001f);
                        }
                        else if (playMode == 2)
                        { // Ping Pong Wrap
                            pingPongForward = true;
                            localPlayhead = loopStart + (loopStart - localPlayhead);
                            if (localPlayhead >= loopEnd)
                                localPlayhead = loopEnd - 0.0001f;
                            eocPulse.trigger(0.001f);
                        }
                    }
                }

                if (endOfSample)
                {
                    isPlaying = false;
                    localPlaying = false;
                    params[PLAY_BUTTON].setValue(0.0f);
                    // a one-shot overdub ends together with the playback
                    if (isRecording.load() && sosOverdub.load())
                        params[REC_BUTTON].setValue(0.0f);
                    env.reset();
                    eocPulse.trigger(0.001f);
                }
                else
                {
                    // Loop boundary crossfade
                    if (playMode == 1 && crossfadeLen > 0 && activeLength > (float)(crossfadeLen * 2))
                    {
                        float distToEnd = loopEnd - localPlayhead;
                        float distToStart = localPlayhead - loopStart;
                        float crossfadeFactor = 1.0f;

                        if (speedVal >= 0.0f && distToEnd < (float)crossfadeLen)
                        {
                            crossfadeFactor = distToEnd / (float)crossfadeLen;
                        }
                        else if (speedVal < 0.0f && distToStart < (float)crossfadeLen)
                        {
                            crossfadeFactor = distToStart / (float)crossfadeLen;
                        }

                        crossfadeFactor = clamp(crossfadeFactor, 0.0f, 1.0f);

                        if (crossfadeFactor < 1.0f)
                        {
                            float lNorm = getInterpolated(leftSamples, localPlayhead);
                            float rNorm = getInterpolated(rightSamples, localPlayhead);

                            float otherPos = (speedVal >= 0.0f) ? (localPlayhead - activeLength) : (localPlayhead + activeLength);

                            float lOther = getInterpolated(leftSamples, otherPos);
                            float rOther = getInterpolated(rightSamples, otherPos);

                            leftOut = lNorm * crossfadeFactor + lOther * (1.0f - crossfadeFactor);
                            rightOut = rNorm * crossfadeFactor + rOther * (1.0f - crossfadeFactor);
                        }
                        else
                        {
                            leftOut = getInterpolated(leftSamples, localPlayhead);
                            rightOut = getInterpolated(rightSamples, localPlayhead);
                        }
                    }
                    else
                    {
                        leftOut = getInterpolated(leftSamples, localPlayhead);
                        rightOut = getInterpolated(rightSamples, localPlayhead);
                    }

                    // Sound on sound: write after reading, so the new material is heard on the next pass
                    if (isRecording.load() && sosOverdub.load())
                        overdubLocked((long)localPlayhead, inL, inR, sosFb);

                    // Start fade-in (anti-click smoother)
                    fadeIn = std::min(1.f, fadeIn + dt / std::max(smoothSec, 1e-5f));
                    float smoothGain = fadeIn * fadeIn * (3.f - 2.f * fadeIn);

                    // One-shot: fade out before the end of the region
                    if (playMode == 0 && smoothSec > 0.f)
                    {
                        float distSamples = (speedVal >= 0.0f) ? (loopEnd - localPlayhead)
                                                               : (localPlayhead - loopStart);
                        float perSample = std::max(std::fabs(speedVal) * rateRatio, 1e-6f);
                        float remaining = distSamples / perSample; // engine samples left
                        float fadeLen = smoothSec * sampleRate;
                        if (remaining < fadeLen)
                            smoothGain *= clamp(remaining / fadeLen, 0.f, 1.f);
                    }

                    float envVal = env.process(dt, sustainActive);
                    envOut = envVal;
                    leftOut *= envVal * smoothGain;
                    rightOut *= envVal * smoothGain;

                    playhead.store(localPlayhead);
                }
            }
            else
            {
                isPlaying = false;
                localPlaying = false;
                params[PLAY_BUTTON].setValue(0.0f);
            }
        }

        // Level (playback only), then the input / playback mix
        leftOut *= level;
        rightOut *= level;

        // MIX is always live with LINE on; with LINE off it only works while recording
        float mixTarget = (lineOn || isRecording.load()) ? mixKnob : 0.f;
        mixSm += (mixTarget - mixSm) * std::min(1.f, dt / 0.005f);
        float inGain = std::sin(mixSm * SP_PI * 0.5f); // equal-power crossfade
        float pbGain = std::cos(mixSm * SP_PI * 0.5f);
        leftOut = leftOut * pbGain + inL * inGain;
        rightOut = rightOut * pbGain + inR * inGain;

        // Meters
        float md = 1.f - 5.f * dt;
        meterInL = std::max(std::fabs(inL), meterInL * md);
        meterInR = std::max(std::fabs(inR), meterInR * md);
        meterOutL = std::max(std::fabs(leftOut), meterOutL * md);
        meterOutR = std::max(std::fabs(rightOut), meterOutR * md);

        // Outputs
        outputs[EOC_OUTPUT].setVoltage(eocPulse.process(dt) ? 5.0f : 0.0f);
        outputs[LEFT_OUTPUT].setVoltage(leftOut);
        outputs[RIGHT_OUTPUT].setVoltage(rightOut);
        outputs[PLAY_GATE_OUTPUT].setVoltage(localPlaying ? 10.f : 0.f);
        outputs[REC_GATE_OUTPUT].setVoltage(isRecording.load() ? 10.f : 0.f);
        outputs[ENV_OUTPUT].setVoltage(envOut * 10.f);

        // Lights
        lights[REC_LIGHT].setSmoothBrightness(isRecording.load() ? 1.f : 0.f, dt);
        lights[PLAY_LIGHT].setSmoothBrightness(localPlaying ? 1.f : 0.f, dt);
        lights[PITCH_MODE_LIGHT].setBrightness(pitchMode ? 1.f : 0.f);
        lights[LOOP_MODE_LIGHT].setBrightness(playMode > 0 ? 1.f : 0.f);
        lights[SOS_LIGHT].setBrightness(sosOn ? 1.f : 0.f);
        lights[LINE_LIGHT].setBrightness(lineOn ? 1.f : 0.f);
    }

    // ---------- patch storage ----------
    // Persist the path, or the raw samples when there is no file behind the audio
    json_t *dataToJson() override
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        json_t *root = json_object();
        if (!lastPath.empty())
        {
            json_object_set_new(root, "lastPath", json_string(lastPath.c_str()));
        }
        else if (!leftSamples.empty())
        {
            json_t *leftArray = json_array();
            json_t *rightArray = json_array();
            for (size_t i = 0; i < leftSamples.size(); i++)
            {
                json_array_append_new(leftArray, json_real(leftSamples[i]));
                json_array_append_new(rightArray, json_real(i < rightSamples.size() ? rightSamples[i] : leftSamples[i]));
            }
            json_object_set_new(root, "leftSamples", leftArray);
            json_object_set_new(root, "rightSamples", rightArray);
        }
        json_object_set_new(root, "sampleRate", json_real(bufferRate.load()));
        json_object_set_new(root, "autoPlay", json_boolean(autoPlay));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *ap = json_object_get(root, "autoPlay");
        if (ap)
            autoPlay = json_is_true(ap);

        json_t *sr = json_object_get(root, "sampleRate");
        float storedRate = sr ? (float)json_number_value(sr) : 44100.f;

        json_t *pathJson = json_object_get(root, "lastPath");
        if (pathJson && json_is_string(pathJson))
        {
            if (loadFile(json_string_value(pathJson)))
                return;
        }

        json_t *leftArray = json_object_get(root, "leftSamples");
        json_t *rightArray = json_object_get(root, "rightSamples");
        if (leftArray && rightArray)
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            leftSamples.clear();
            rightSamples.clear();
            size_t len = json_array_size(leftArray);
            leftSamples.reserve(len);
            rightSamples.reserve(len);
            for (size_t i = 0; i < len; i++)
            {
                leftSamples.push_back((float)json_number_value(json_array_get(leftArray, i)));
                rightSamples.push_back((float)json_number_value(json_array_get(rightArray, i)));
            }
            mainBufferSize = len;
            bufferRate = storedRate > 1000.f ? storedRate : 44100.f;
            lastPath = "";
            playhead = 0.0f;
            rebuildPeaksLocked();
        }
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{

    // ------------------------------------------------------------
    // Theme (same palette as the other AGMM modules)
    // ------------------------------------------------------------
    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    NVGcolor colDim() { return nvgRGB(0x7a, 0x85, 0x94); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    NVGcolor colViolet() { return nvgRGB(0xb0, 0x5a, 0xff); }
    NVGcolor colRec() { return nvgRGB(0xff, 0x5a, 0x7a); }

    NVGcolor withAlpha(NVGcolor c, float a)
    {
        c.a = a;
        return c;
    }

    void setFont(NVGcontext *vg)
    {
        std::shared_ptr<window::Font> font =
            APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (font)
            nvgFontFaceId(vg, font->handle);
    }

    void drawText(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
                  int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f)
    {
        setFont(vg);
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Cyan LED (Rack has no built-in one)
    template <typename TBase = GrayModuleLightWidget>
    struct TSPCyanLight : TBase
    {
        TSPCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
    };
    using SPCyanLight = TSPCyanLight<>;

    // ------------------------------------------------------------
    // AGMM logo
    // ------------------------------------------------------------
    void drawLogo(NVGcontext *vg, float cx, float cy)
    {
        const float w = 64.f, h = 18.f;
        float x = cx - w / 2.f, y = cy - h / 2.f;

        // Glow
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
        nvgFill(vg);

        // Pill body
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 7.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                           nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
        nvgFill(vg);
        nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
                                             nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgStrokeWidth(vg, 1.2f);
        nvgStroke(vg);

        // Tiny cell triangle
        const float s = 2.2f;
        float tx = x + 7.f, ty = y + h / 2.f - 3.f;
        nvgBeginPath(vg);
        nvgRect(vg, tx + s, ty, s, s);
        nvgRect(vg, tx, ty + s * 1.4f, s, s);
        nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
        nvgFillColor(vg, colAccent());
        nvgFill(vg);

        // Wordmark
        drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
    }

    void fmtTime(char *buf, size_t n, float secs)
    {
        if (secs < 0.f)
            secs = 0.f;
        int m = (int)(secs / 60.f);
        int s = (int)secs % 60;
        int cs = (int)((secs - std::floor(secs)) * 100.f);
        snprintf(buf, n, "%02d:%02d.%02d", m, s, cs);
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct SPPanelBackground : TransparentWidget
    {
        SPPanelBackground() { box.size = Vec(SP_PANEL_W, SP_PANEL_H); }

        void label(NVGcontext *vg, float x, float y, const char *txt)
        {
            drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        void plate(NVGcontext *vg, float x, float y, float w, float h)
        {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, y, w, h, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 80));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        void ring(NVGcontext *vg, float cx, float cy, float r)
        {
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, r);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Caption with an underline (side blocks)
        void caption(NVGcontext *vg, float cx, float y, const char *txt, NVGcolor col, float x1, float x2)
        {
            drawText(vg, cx, y, txt, 8.f, col, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x1, y + 8.f);
            nvgLineTo(vg, x2, y + 8.f);
            nvgStrokeColor(vg, withAlpha(col, 0.35f));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Divider with a centered caption
        void section(NVGcontext *vg, float x1, float x2, float y, const char *txt)
        {
            float cx = (x1 + x2) / 2.f;
            float w = 10.f + (float)strlen(txt) * 7.f;
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

        // Jack cell: label on top, jack below
        void topCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
            label(vg, cx, cy - 19.f, txt);
        }
        // Control in the middle, label below
        void jackCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 17.f, 34.f, 34.f);
            label(vg, cx, cy + 25.f, txt);
        }
        void switchCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 20.f, 34.f, 40.f);
            label(vg, cx, cy + 25.f, txt);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = SP_PANEL_W, H = SP_PANEL_H;

            // Base gradient
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            // Brushed texture
            nvgStrokeWidth(vg, 1.f);
            for (int y = 0; y < (int)H; y += 3)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, y + 0.5f);
                nvgLineTo(vg, W, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
                nvgStroke(vg);
            }

            // Edge highlight
            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, W - 1.f, H - 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Logo + title
            drawLogo(vg, W / 2.f, 13.f);
            drawText(vg, W / 2.f, 31.f, "SAMPLER", 13.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

            // Display bezel
            float bx = SP_DISP_X - 5.f, by = SP_DISP_Y - 5.f;
            float bw = SP_DISP_W + 10.f, bh = SP_DISP_H + 10.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                               nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
            nvgFill(vg);

            // ---- left block: input + transport ----
            caption(vg, 62.f, 54.f, "INPUT", colAccent(), 12.f, 112.f);
            topCell(vg, SP_LCOL[0], 100.f, "L IN");
            topCell(vg, SP_LCOL[1], 100.f, "R IN");
            switchCell(vg, SP_LCOL[0], SP_Y_MIX, "MIX");
            switchCell(vg, SP_LCOL[1], SP_Y_MIX, "CLEAR");
            switchCell(vg, SP_LCOL[0], SP_Y_SOS, "SOS");
            switchCell(vg, SP_LCOL[1], SP_Y_SOS, "LINE");

            caption(vg, 62.f, 248.f, "TRANSPORT", colViolet(), 12.f, 112.f);
            jackCell(vg, SP_LCOL[0], SP_Y_REC, "REC");
            jackCell(vg, SP_LCOL[1], SP_Y_REC, "REC TRG");
            jackCell(vg, SP_LCOL[0], SP_Y_PLAY, "PLAY");
            jackCell(vg, SP_LCOL[1], SP_Y_PLAY, "PLAY TRG");

            // ---- right block: outputs + control inputs ----
            caption(vg, 480.f, 54.f, "OUTPUT", colViolet(), 435.f, 525.f);
            topCell(vg, SP_RCOL[0], 100.f, "L OUT");
            topCell(vg, SP_RCOL[1], 100.f, "R OUT");
            topCell(vg, SP_RCOL[0], 150.f, "EOC");
            topCell(vg, SP_RCOL[1], 150.f, "PLAY GT");
            topCell(vg, SP_RCOL[0], 200.f, "ENV");
            topCell(vg, SP_RCOL[1], 200.f, "REC GT");

            caption(vg, 480.f, 238.f, "CONTROL", colAccent(), 435.f, 525.f);
            topCell(vg, SP_RCOL[0], 284.f, "GATE IN");
            topCell(vg, SP_RCOL[1], 284.f, "V/OCT");
            switchCell(vg, SP_RCOL[0], SP_Y_RKNOB, "SMOOTH");
            switchCell(vg, SP_RCOL[1], SP_Y_RKNOB, "SOS FB");

            // ---- centre grid ----
            section(vg, 120.f, 422.f, 167.f, "PLAYBACK");
            for (int r = 0; r < 2; ++r)
                for (int c = 0; c < 5; ++c)
                    plate(vg, SP_CX[c] - SP_CELL_W / 2.f, r == 0 ? SP_R1 : SP_R2, SP_CELL_W, SP_CELL_H);

            // Row 1: SPEED, START, LENGTH (knob + CV jack + amount), ATTACK, DECAY
            const char *r1[5] = {"SPEED", "START", "LENGTH", "ATTACK", "DECAY"};
            for (int c = 0; c < 5; ++c)
            {
                ring(vg, SP_CX[c], SP_R1 + SP_KNOB_DY, 17.f);
                label(vg, SP_CX[c], SP_R1 + SP_LABEL_DY, r1[c]);
            }
            for (int c = 0; c < 3; ++c)
            {
                label(vg, SP_CX[c] - 15.f, SP_R1 + SP_TINY_DY, "CV");
                label(vg, SP_CX[c] + 15.f, SP_R1 + SP_TINY_DY, "AMT");
            }

            // Row 2: three switches, XFADE, LEVEL
            const char *sw[3] = {"TUNING", "MODE", "DIRECTION"};
            for (int c = 0; c < 3; ++c)
                label(vg, SP_CX[c], SP_R2 + SP_SWLABEL_DY, sw[c]);
            for (int c = 3; c < 5; ++c)
            {
                ring(vg, SP_CX[c], SP_R2 + SP_KNOB_DY, 17.f);
                label(vg, SP_CX[c], SP_R2 + SP_LABEL_DY, c == 3 ? "XFADE" : "LEVEL");
            }

            // ---- sample info strip ----
            plate(vg, SP_INFO_X, SP_INFO_Y, SP_INFO_W, SP_INFO_H);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Waveform display (also interactive: drag to select the loop region)
    // ------------------------------------------------------------
    struct SPDisplayWidget : TransparentWidget
    {
        Sampler *module = nullptr;
        float dragAnchor = 0.f;
        float dragCur = 0.f;

        SPDisplayWidget(Sampler *m) : module(m)
        {
            box.pos = Vec(SP_DISP_X, SP_DISP_Y);
            box.size = Vec(SP_DISP_W, SP_DISP_H);
        }

        // ---- mouse: press = set start (end is kept), drag = select a region, double click = reset ----
        float toNorm(float x) { return clamp((x - SP_WF_X0) / (float)SP_WF_W, 0.f, 1.f); }

        void setRegion(float a, float b)
        {
            if (!module)
                return;
            a = clamp(a, 0.f, 1.f);
            b = clamp(b, 0.f, 1.f);
            if (b < a)
                std::swap(a, b);
            float len = (a >= 0.9999f) ? 0.001f : (b - a) / (1.f - a);
            module->paramQuantities[Sampler::START_PARAM]->setValue(a);
            module->paramQuantities[Sampler::LENGTH_PARAM]->setValue(clamp(len, 0.001f, 1.f));
        }

        void onButton(const ButtonEvent &e) override
        {
            if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT)
            {
                dragAnchor = toNorm(e.pos.x);
                dragCur = dragAnchor;
                float s = module->params[Sampler::START_PARAM].getValue();
                float l = module->params[Sampler::LENGTH_PARAM].getValue();
                float endPrev = s + l * (1.f - s);
                if (endPrev <= dragAnchor + 0.002f)
                    endPrev = 1.f;
                setRegion(dragAnchor, endPrev);
                e.consume(this);
            }
        }

        void onDragMove(const DragMoveEvent &e) override
        {
            if (!module)
                return;
            float zoom = getAbsoluteZoom();
            if (zoom <= 0.f)
                zoom = 1.f;
            dragCur = clamp(dragCur + e.mouseDelta.x / zoom / (float)SP_WF_W, 0.f, 1.f);
            if (std::fabs(dragCur - dragAnchor) > 0.002f)
                setRegion(dragAnchor, dragCur);
        }

        void onDoubleClick(const DoubleClickEvent &e) override
        {
            if (!module)
                return;
            module->paramQuantities[Sampler::START_PARAM]->setValue(0.f);
            module->paramQuantities[Sampler::LENGTH_PARAM]->setValue(1.f);
        }

        // Static background: screen, waveform field, grid
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = SP_DISP_W, H = SP_DISP_H;
            const float X0 = SP_WF_X0, Y0 = SP_WF_Y0, WH = SP_WF_H, WW = (float)SP_WF_W;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H);
            nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 30.f, W * 0.7f,
                                               nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
            nvgFill(vg);

            // Waveform field
            nvgBeginPath(vg);
            nvgRoundedRect(vg, X0 - 1.f, Y0 - 1.f, WW + 2.f, WH + 2.f, 2.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Grid: 8 divisions, +-2.5 V lines and the zero line
            nvgBeginPath(vg);
            for (int i = 1; i < 8; ++i)
            {
                float x = X0 + WW * i / 8.f + 0.5f;
                nvgMoveTo(vg, x, Y0);
                nvgLineTo(vg, x, Y0 + WH);
            }
            float mid = Y0 + WH / 2.f, amp = WH / 2.f - 2.f;
            nvgMoveTo(vg, X0, mid - amp * 0.5f + 0.5f);
            nvgLineTo(vg, X0 + WW, mid - amp * 0.5f + 0.5f);
            nvgMoveTo(vg, X0, mid + amp * 0.5f + 0.5f);
            nvgLineTo(vg, X0 + WW, mid + amp * 0.5f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, X0, mid + 0.5f);
            nvgLineTo(vg, X0 + WW, mid + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 45));
            nvgStroke(vg);

            // Time ruler ticks under the waveform
            nvgBeginPath(vg);
            for (int i = 0; i <= 10; ++i)
            {
                float x = X0 + WW * i / 10.f + 0.5f;
                nvgMoveTo(vg, x, Y0 + WH + 1.f);
                nvgLineTo(vg, x, Y0 + WH + (i % 5 == 0 ? 4.f : 2.5f));
            }
            nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 70));
            nvgStroke(vg);

            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = SP_DISP_W, H = SP_DISP_H;
            const float X0 = SP_WF_X0, Y0 = SP_WF_Y0, WH = SP_WF_H, WW = (float)SP_WF_W;
            const float mid = Y0 + WH / 2.f, amp = WH / 2.f - 2.f;
            const int N = SP_WF_W;

            // ---- gather state (live, or a sample scene in the module browser) ----
            float colMin[SP_WF_W], colMax[SP_WF_W];
            bool has = false, rec = false, sos = false, recNew = false, play = false;
            float ph = 0.f, sN = 0.f, lN = 1.f, rate = 48000.f, speed = 1.f;
            size_t total = 0;
            int playMode = 1;
            bool rev = false, pitchMode = false;
            float mIL = 0.f, mIR = 0.f, mOL = 0.f, mOR = 0.f;
            for (int c = 0; c < N; ++c)
                colMin[c] = colMax[c] = 0.f;

            if (module)
            {
                rec = module->isRecording.load();
                sos = module->sosOverdub.load();
                recNew = rec && !sos; // recording a brand-new sample
                play = module->isPlaying.load();
                sN = module->loopStartNorm.load();
                lN = module->loopLengthNorm.load();
                rate = module->bufferRate.load();
                speed = module->displaySpeed;
                playMode = clamp((int)std::round(module->params[Sampler::PLAY_MODE_PARAM].getValue()), 0, 2);
                rev = module->params[Sampler::DIRECTION_PARAM].getValue() > 0.5f;
                pitchMode = module->params[Sampler::PITCH_MODE_PARAM].getValue() > 0.5f;
                mIL = module->meterInL;
                mIR = module->meterInR;
                mOL = module->meterOutL;
                mOR = module->meterOutR;
                total = recNew ? module->recBufferSize.load() : module->mainBufferSize.load();
                float phRaw = module->playhead.load();
                if (!recNew && total > 0)
                    ph = phRaw / (float)total;

                std::lock_guard<std::mutex> lock(module->bufferMutex);
                const std::vector<float> &smin = recNew ? module->recPeakMin : module->peakMin;
                const std::vector<float> &smax = recNew ? module->recPeakMax : module->peakMax;
                size_t m = std::min(smin.size(), smax.size());
                if (m > 0)
                {
                    has = true;
                    for (int c = 0; c < N; ++c)
                    {
                        size_t s = m * (size_t)c / (size_t)N;
                        size_t e = m * (size_t)(c + 1) / (size_t)N;
                        if (s >= m)
                            s = m - 1;
                        if (e <= s)
                            e = s + 1;
                        if (e > m)
                            e = m;
                        float mn = 0.f, mx = 0.f;
                        for (size_t j = s; j < e; ++j)
                        {
                            mn = std::min(mn, smin[j]);
                            mx = std::max(mx, smax[j]);
                        }
                        colMin[c] = mn;
                        colMax[c] = mx;
                    }
                }
            }
            else
            {
                has = true;
                play = true;
                ph = 0.4f;
                sN = 0.08f;
                lN = 0.75f;
                rate = 48000.f;
                total = 480000;
                for (int c = 0; c < N; ++c)
                {
                    float t = (float)c / N;
                    float e = 4.2f * std::exp(-2.2f * t) * (0.35f + 0.65f * std::fabs(std::sin(t * 38.f)));
                    colMax[c] = e;
                    colMin[c] = -e * 0.9f;
                }
                mIL = 2.f;
                mIR = 1.6f;
                mOL = 3.2f;
                mOR = 2.6f;
            }

            float eN = std::min(sN + lN * (1.f - sN), 1.f);
            auto inLoop = [&](int c)
            {
                if (recNew)
                    return true;
                float t = (c + 0.5f) / (float)N;
                return t >= sN && t <= eN;
            };

            // ---- waveform ----
            if (has)
            {
                for (int pass = 0; pass < 3; ++pass)
                { // 0: glow, 1: crisp inside loop, 2: outside loop
                    nvgBeginPath(vg);
                    for (int c = 0; c < N; ++c)
                    {
                        bool in = inLoop(c);
                        if (pass < 2 && !in)
                            continue;
                        if (pass == 2 && in)
                            continue;
                        float y0 = mid - std::min(std::max(colMax[c] / 5.f, -1.f), 1.f) * amp;
                        float y1 = mid - std::min(std::max(colMin[c] / 5.f, -1.f), 1.f) * amp;
                        float h = std::max(y1 - y0, 1.f);
                        float x = X0 + c;
                        if (pass == 0)
                            nvgRect(vg, x - 0.5f, y0 - 1.5f, 2.f, h + 3.f);
                        else
                            nvgRect(vg, x, y0, 1.f, h);
                    }
                    if (recNew)
                        nvgFillColor(vg, pass == 0 ? withAlpha(colRec(), 0.14f) : nvgRGB(0xff, 0x9a, 0xb0));
                    else if (pass == 0)
                        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 32));
                    else if (pass == 1)
                        nvgFillColor(vg, nvgRGB(0x8a, 0xee, 0xff));
                    else
                        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
                    nvgFill(vg);
                }
            }

            // ---- loop region ----
            if (has && !recNew)
            {
                float xs = X0 + sN * WW, xe = X0 + eN * WW;
                nvgBeginPath(vg);
                nvgRect(vg, xs, Y0, std::max(xe - xs, 1.f), WH);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
                nvgFill(vg);

                // start marker (cyan) and end marker (violet), each with a small flag
                nvgBeginPath(vg);
                nvgMoveTo(vg, xs + 0.5f, Y0);
                nvgLineTo(vg, xs + 0.5f, Y0 + WH);
                nvgStrokeColor(vg, nvgRGBA(0x8a, 0xee, 0xff, 220));
                nvgStrokeWidth(vg, 1.2f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, xs, Y0);
                nvgLineTo(vg, xs + 6.f, Y0);
                nvgLineTo(vg, xs, Y0 + 6.f);
                nvgClosePath(vg);
                nvgFillColor(vg, nvgRGB(0x8a, 0xee, 0xff));
                nvgFill(vg);

                nvgBeginPath(vg);
                nvgMoveTo(vg, xe - 0.5f, Y0);
                nvgLineTo(vg, xe - 0.5f, Y0 + WH);
                nvgStrokeColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 230));
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, xe, Y0);
                nvgLineTo(vg, xe - 6.f, Y0);
                nvgLineTo(vg, xe, Y0 + 6.f);
                nvgClosePath(vg);
                nvgFillColor(vg, colViolet());
                nvgFill(vg);
            }

            // ---- playhead (pink while overdubbing) ----
            if (has && play && !recNew)
            {
                float x = X0 + clamp(ph, 0.f, 1.f) * WW;
                NVGcolor pc = sos ? nvgRGB(0xff, 0x9a, 0xb0) : nvgRGB(255, 255, 255);
                nvgBeginPath(vg);
                nvgMoveTo(vg, x, Y0);
                nvgLineTo(vg, x, Y0 + WH);
                nvgStrokeColor(vg, withAlpha(pc, 0.16f));
                nvgStrokeWidth(vg, 4.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, x, Y0);
                nvgLineTo(vg, x, Y0 + WH);
                nvgStrokeColor(vg, withAlpha(pc, 0.94f));
                nvgStrokeWidth(vg, 1.4f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, x - 3.5f, Y0 - 3.f);
                nvgLineTo(vg, x + 3.5f, Y0 - 3.f);
                nvgLineTo(vg, x, Y0 + 2.f);
                nvgClosePath(vg);
                nvgFillColor(vg, pc);
                nvgFill(vg);
            }

            // ---- empty state ----
            if (!has && !rec)
            {
                drawText(vg, X0 + WW / 2.f, mid - 5.f, "NO SAMPLE", 10.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
                drawText(vg, X0 + WW / 2.f, mid + 8.f, "REC  OR  RIGHT-CLICK > LOAD WAV", 6.5f,
                         withAlpha(colDim(), 0.8f), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            }

            // ---- level meters (left: input, right: output) ----
            auto meter = [&](float x, float v)
            {
                float lvl = std::min(v / 5.f, 1.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, Y0, 4.f, WH);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
                nvgFill(vg);
                if (lvl > 0.01f)
                {
                    float h = lvl * WH;
                    nvgBeginPath(vg);
                    nvgRect(vg, x, Y0 + WH - h, 4.f, h);
                    nvgFillPaint(vg, nvgLinearGradient(vg, 0, Y0 + WH, 0, Y0,
                                                       nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                    nvgFill(vg);
                }
            };
            meter(3.f, mIL);
            meter(9.f, mIR);
            meter(W - 13.f, mOL);
            meter(W - 7.f, mOR);
            drawText(vg, 8.f, Y0 + WH + 8.f, "IN", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            drawText(vg, W - 10.f, Y0 + WH + 8.f, "OUT", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

            // ---- scanlines ----
            nvgBeginPath(vg);
            for (int y = 0; y < (int)H; y += 3)
                nvgRect(vg, 0, y + 1, W, 1);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
            nvgFill(vg);

            // ---- glass reflection ----
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H * 0.3f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f,
                                               nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
            nvgFill(vg);

            // ---- top line: state chip, mode, speed ----
            {
                const char *st = rec ? (sos ? "SOS" : "REC") : (play ? "PLAY" : "STOP");
                NVGcolor sc = rec ? colRec() : (play ? colAccent() : colDim());
                float pulse = rec ? 0.55f + 0.45f * (float)std::sin(system::getTime() * 8.0) : 1.f;
                float cw = 30.f;
                nvgBeginPath(vg);
                nvgRoundedRect(vg, 6.f, 3.f, cw, 11.f, 3.f);
                if (play || rec)
                {
                    nvgFillColor(vg, withAlpha(sc, 0.25f * pulse));
                    nvgFill(vg);
                }
                nvgStrokeColor(vg, withAlpha(sc, (play || rec) ? 0.9f * pulse : 0.3f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                drawText(vg, 6.f + cw / 2.f, 8.5f, st, 6.f, sc, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);

                static const char *modeNames[3] = {"ONE-SHOT", "LOOP", "PING-PONG"};
                char buf[64];
                snprintf(buf, sizeof(buf), "%s | %s", modeNames[playMode], (rev ? "REV" : "FWD"));
                drawText(vg, W / 2.f, 8.5f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 200),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

                snprintf(buf, sizeof(buf), pitchMode ? "TUNE %.2fX" : "SPD %+.2fX", speed);
                drawText(vg, W - 6.f, 8.5f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 200),
                         NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
            }

            // ---- bottom line: time and loop region ----
            {
                char buf[64], a[16], b[16];
                float secsTotal = rate > 0.f ? (float)total / rate : 0.f;
                if (recNew)
                {
                    fmtTime(a, sizeof(a), secsTotal);
                    snprintf(buf, sizeof(buf), "REC %s", a);
                }
                else if (has && play)
                {
                    fmtTime(a, sizeof(a), ph * secsTotal);
                    fmtTime(b, sizeof(b), secsTotal);
                    snprintf(buf, sizeof(buf), "%s / %s", a, b);
                }
                else if (has)
                {
                    fmtTime(b, sizeof(b), secsTotal);
                    snprintf(buf, sizeof(buf), "LEN %s", b);
                }
                else
                {
                    snprintf(buf, sizeof(buf), "--:--.--");
                }
                drawText(vg, 6.f, H - 8.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220),
                         NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

                if (has && !recNew)
                {
                    fmtTime(a, sizeof(a), sN * secsTotal);
                    fmtTime(b, sizeof(b), eN * secsTotal);
                    snprintf(buf, sizeof(buf), "%s - %s", a, b);
                    drawText(vg, W - 6.f, H - 8.f, buf, 7.f, colDim(),
                             NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
                }
            }

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Value readout under a knob / switch
    // ------------------------------------------------------------
    struct SPReadout : TransparentWidget
    {
        enum Fmt
        {
            SECONDS,
            MILLIS,
            PERCENT,
            PITCH_MODE,
            PLAY_MODE,
            DIRECTION,
            DECAY_SECONDS
        };
        Sampler *module = nullptr;
        int paramId = 0;
        Fmt fmt = SECONDS;
        float demo = 0.f;

        SPReadout(Sampler *m, int p, Fmt f, float demoVal, float cx, float cy) : module(m), paramId(p), fmt(f), demo(demoVal)
        {
            box.pos = Vec(cx - 28.f, cy - 6.f);
            box.size = Vec(56.f, 12.f);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            float v = module ? module->params[paramId].getValue() : demo;
            char buf[32];
            switch (fmt)
            {
            case SECONDS:
                snprintf(buf, sizeof(buf), "%.2f S", v);
                break;
            case DECAY_SECONDS:
                if (v > 9.9f)
                    snprintf(buf, sizeof(buf), "SUSTAIN");
                else
                    snprintf(buf, sizeof(buf), "%.2f S", v);
                break;
            case MILLIS:
                snprintf(buf, sizeof(buf), "%.0f MS", v);
                break;
            case PERCENT:
                snprintf(buf, sizeof(buf), "%.0f%%", v * 100.f);
                break;
            case PITCH_MODE:
                snprintf(buf, sizeof(buf), "%s", v > 0.5f ? "V/OCT" : "SPEED");
                break;
            case PLAY_MODE:
            {
                static const char *n[3] = {"ONE-SHOT", "LOOP", "PING-PONG"};
                snprintf(buf, sizeof(buf), "%s", n[clamp((int)std::round(v), 0, 2)]);
                break;
            }
            case DIRECTION:
                snprintf(buf, sizeof(buf), "%s", v > 0.5f ? "REVERSE" : "FORWARD");
                break;
            }
            drawText(vg, box.size.x / 2.f, box.size.y / 2.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Sample info strip (file name, length, sample rate)
    // ------------------------------------------------------------
    struct SPInfoWidget : TransparentWidget
    {
        Sampler *module = nullptr;

        SPInfoWidget(Sampler *m) : module(m)
        {
            box.pos = Vec(SP_INFO_X, SP_INFO_Y);
            box.size = Vec(SP_INFO_W, SP_INFO_H);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float cy = box.size.y / 2.f;

            std::string name;
            size_t total = 0;
            float rate = 48000.f;
            if (module)
            {
                {
                    std::lock_guard<std::mutex> lock(module->bufferMutex);
                    name = module->lastPath;
                }
                total = module->mainBufferSize.load();
                rate = module->bufferRate.load();
                if (name.empty() && total > 0)
                    name = "RECORDED SAMPLE";
                else if (!name.empty())
                {
                    size_t slash = name.find_last_of("/\\");
                    if (slash != std::string::npos)
                        name = name.substr(slash + 1);
                }
            }
            else
            {
                name = "demo_loop.wav";
                total = 480000;
            }
            if (name.empty())
                name = "NO SAMPLE LOADED";
            if (name.size() > 30)
                name = name.substr(0, 27) + "...";

            drawText(vg, 10.f, cy, name.c_str(), 7.5f,
                     total > 0 ? colText() : colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

            if (total > 0)
            {
                char t[16], buf[48];
                fmtTime(t, sizeof(t), (float)total / rate);
                snprintf(buf, sizeof(buf), "%s  %.1fKHZ", t, rate / 1000.f);
                drawText(vg, box.size.x - 10.f, cy, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 200),
                         NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
            }
            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct SamplerWidget : ModuleWidget
{
    SamplerWidget(Sampler *module)
    {
        setModule(module);
        box.size = Vec(SP_PANEL_W, SP_PANEL_H);

        addChild(new SPPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SPDisplayWidget(module));
        addChild(new SPInfoWidget(module));

        // ---- left block: input ----
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_LCOL[0], 100.f), module, Sampler::LEFT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_LCOL[1], 100.f), module, Sampler::RIGHT_INPUT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_LCOL[0], SP_Y_MIX), module, Sampler::MIX_PARAM));
        addParam(createParamCentered<VCVButton>(Vec(SP_LCOL[1], SP_Y_MIX), module, Sampler::CLEAR_BUTTON));

        // ---- left block: SOS / LINE (latching buttons with yellow LEDs) ----
        auto sosButton = createParamCentered<VCVButton>(Vec(SP_LCOL[0], SP_Y_SOS + 3.f), module, Sampler::SOS_SWITCH);
        sosButton->momentary = false;
        addParam(sosButton);
        addChild(createLightCentered<SmallLight<YellowLight>>(Vec(SP_LCOL[0], SP_Y_SOS - 12.f), module, Sampler::SOS_LIGHT));

        auto lineButton = createParamCentered<VCVButton>(Vec(SP_LCOL[1], SP_Y_SOS + 3.f), module, Sampler::LINE_SWITCH);
        lineButton->momentary = false;
        addParam(lineButton);
        addChild(createLightCentered<SmallLight<YellowLight>>(Vec(SP_LCOL[1], SP_Y_SOS - 12.f), module, Sampler::LINE_LIGHT));

        // ---- left block: transport ----
        auto recButton = createParamCentered<VCVButton>(Vec(SP_LCOL[0], SP_Y_REC), module, Sampler::REC_BUTTON);
        recButton->momentary = false;
        addParam(recButton);
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(62.f, SP_Y_REC), module, Sampler::REC_LIGHT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_LCOL[1], SP_Y_REC), module, Sampler::RECORD_TRIGGER));

        auto playButton = createParamCentered<VCVButton>(Vec(SP_LCOL[0], SP_Y_PLAY), module, Sampler::PLAY_BUTTON);
        playButton->momentary = false;
        addParam(playButton);
        addChild(createLightCentered<SmallLight<SPCyanLight>>(Vec(62.f, SP_Y_PLAY), module, Sampler::PLAY_LIGHT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_LCOL[1], SP_Y_PLAY), module, Sampler::PLAY_TRIGGER));

        // ---- right block ----
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[0], 100.f), module, Sampler::LEFT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[1], 100.f), module, Sampler::RIGHT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[0], 150.f), module, Sampler::EOC_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[1], 150.f), module, Sampler::PLAY_GATE_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[0], 200.f), module, Sampler::ENV_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[1], 200.f), module, Sampler::REC_GATE_OUTPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_RCOL[0], 284.f), module, Sampler::GATE_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_RCOL[1], 284.f), module, Sampler::PITCH_CV_INPUT));

        // new knobs: start/end smoothing and sound-on-sound old-material level
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_RCOL[0], SP_Y_RKNOB), module, Sampler::DECLICK_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_RCOL[1], SP_Y_RKNOB), module, Sampler::SOS_FDBK_PARAM));

        // ---- centre row 1: SPEED, START, LENGTH (knob + CV jack + amount), ATTACK, DECAY ----
        const float k1 = SP_R1 + SP_KNOB_DY, j1 = SP_R1 + SP_JACK_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[0], k1), module, Sampler::SPEED_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_CX[0] - 15.f, j1), module, Sampler::SPEED_CV_INPUT));
        addParam(createParamCentered<Trimpot>(Vec(SP_CX[0] + 15.f, j1), module, Sampler::SPEED_CV_ATTEN));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[1], k1), module, Sampler::START_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_CX[1] - 15.f, j1), module, Sampler::START_CV_INPUT));
        addParam(createParamCentered<Trimpot>(Vec(SP_CX[1] + 15.f, j1), module, Sampler::START_CV_ATTEN));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[2], k1), module, Sampler::LENGTH_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_CX[2] - 15.f, j1), module, Sampler::LENGTH_CV_INPUT));
        addParam(createParamCentered<Trimpot>(Vec(SP_CX[2] + 15.f, j1), module, Sampler::LENGTH_CV_ATTEN));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[3], k1), module, Sampler::ATTACK_PARAM));
        addChild(new SPReadout(module, Sampler::ATTACK_PARAM, SPReadout::SECONDS, 0.f, SP_CX[3], j1));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[4], k1), module, Sampler::DECAY_PARAM));
        addChild(new SPReadout(module, Sampler::DECAY_PARAM, SPReadout::DECAY_SECONDS, 10.f, SP_CX[4], j1));

        // ---- centre row 2: switches, XFADE, LEVEL ----
        const float s2 = SP_R2 + SP_SW_DY, r2 = SP_R2 + SP_SWREAD_DY;
        addParam(createParamCentered<CKSS>(Vec(SP_CX[0], s2), module, Sampler::PITCH_MODE_PARAM));
        addChild(new SPReadout(module, Sampler::PITCH_MODE_PARAM, SPReadout::PITCH_MODE, 0.f, SP_CX[0], r2));
        addParam(createParamCentered<CKSSThree>(Vec(SP_CX[1], s2), module, Sampler::PLAY_MODE_PARAM));
        addChild(new SPReadout(module, Sampler::PLAY_MODE_PARAM, SPReadout::PLAY_MODE, 1.f, SP_CX[1], r2));
        addParam(createParamCentered<CKSS>(Vec(SP_CX[2], s2), module, Sampler::DIRECTION_PARAM));
        addChild(new SPReadout(module, Sampler::DIRECTION_PARAM, SPReadout::DIRECTION, 0.f, SP_CX[2], r2));

        const float k2 = SP_R2 + SP_KNOB_DY, v2 = SP_R2 + SP_JACK_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[3], k2), module, Sampler::XFADE_PARAM));
        addChild(new SPReadout(module, Sampler::XFADE_PARAM, SPReadout::MILLIS, 15.f, SP_CX[3], v2));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[4], k2), module, Sampler::LEVEL_PARAM));
        addChild(new SPReadout(module, Sampler::LEVEL_PARAM, SPReadout::PERCENT, 1.f, SP_CX[4], v2));
    }

    void appendContextMenu(Menu *menu) override
    {
        ModuleWidget::appendContextMenu(menu);

        Sampler *module = dynamic_cast<Sampler *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Sample"));
        menu->addChild(createMenuItem("Load WAV sample...", "", [=]()
                                      { module->loadWavDialog(); }));
        menu->addChild(createMenuItem("Save WAV sample...", "", [=]()
                                      { module->saveWavDialog(); }));

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuItem("Normalize", "", [=]()
                                      { module->normalizeSample(); }));
        menu->addChild(createMenuItem("Reverse sample", "", [=]()
                                      { module->reverseSample(); }));
        menu->addChild(createMenuItem("Trim to loop region", "", [=]()
                                      { module->trimToLoop(); }));
        menu->addChild(createMenuItem("Clear sample", "", [=]()
                                      { module->clearSample(); }));

        menu->addChild(new MenuSeparator);
        menu->addChild(createCheckMenuItem("Auto-play after recording", "", [=]()
                                           { return module->autoPlay; }, [=]()
                                           { module->autoPlay = !module->autoPlay; }));
    }
};

Model *modelSampler = createModel<Sampler, SamplerWidget>("Sampler");