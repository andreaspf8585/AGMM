#include "YouTubeLoad.hpp"
#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
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
#include "Knob_arcs.hpp"

// ============================================================================
// Sampler
// ============================================================================
//
// OVERVIEW
// --------
// Sampler is a 36 HP mono/stereo sample recorder + player with a rich set of
// playback features. It can record from the L/R inputs (either a brand-new
// sample or, in Sound-on-Sound mode, overdub onto the existing one), load and
// save WAV files, and play back with adjustable speed / V-Oct tuning, start
// position, loop length, forward/reverse direction, one-shot / loop /
// ping-pong play modes, anti-click fades, a simple A-D envelope (with optional
// sustain gate), a loop crossfade and a configurable input/playback mix.
//
// Signal flow (per sample):
//   inputs L/R
//     -> (record path) temp buffers + waveform overview
//     -> (playback path) read at speed from the buffer region
//        [start, start + length] -> envelope -> level
//     -> mix (input + playback), monitored by LINE/MIX
//     -> outputs L/R
//
// Side outputs: EOC trigger (fires on loop wrap and one-shot end), Play gate,
// Record gate, envelope CV. A stereo-to-mono normalling rule means a single
// cable on the left input is duplicated to the right.
//
// File structure:
//   1. Constants and layout (36 HP = 540x380 px).
//   2. SamplerEnvelope helper struct (simple attack / decay with sustain).
//   3. spLoadWav / spSaveWav — self-contained 16-bit PCM WAV I/O.
//   4. Sampler module: state, DSP, recording/overdub, transport, and JSON
//      persistence.
//   5. Anonymous-namespace UI: colour helpers, panel background, interactive
//      waveform display, per-parameter readouts, sample info strip.
//   6. SamplerWidget: places all controls and the right-click menu.
//   7. Model registration.
// ============================================================================

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

// Waveform display (title sits at y = 15, bezel starts at y = 24)
static const float SP_DISP_X = 130.f;
static const float SP_DISP_Y = 28.f;
static const float SP_DISP_W = 280.f;
static const float SP_DISP_H = 104.f;
static const float SP_WF_X0 = 18.f; // waveform area inside the display
static const float SP_WF_Y0 = 20.f;
static const float SP_WF_H = 60.f;
static const int SP_WF_W = 244;

// Sample info strip, directly under the display
static const float SP_INFO_X = 130.f;
static const float SP_INFO_Y = 141.f;
static const float SP_INFO_W = 280.f;
static const float SP_INFO_H = 16.f;

// Centre control grid: 5 columns x 2 rows of 58 px wide cells
static const float SP_CX[5] = {149.f, 210.f, 271.f, 332.f, 393.f};
static const float SP_CELL_W = 58.f;
static const float SP_PB_Y = 167.f; // "PLAYBACK" header
static const float SP_R1 = 174.f;   // top of row 1
static const float SP_R1_H = 88.f;
static const float SP_R2 = 267.f; // top of row 2
static const float SP_R2_H = 78.f;
// offsets from the top of a cell (label is at +10)
static const float SP_KNOB_DY = 32.f;  // row 1 knob centre
static const float SP_JACK_DY = 62.f;  // CV jack / trimpot / value readout
static const float SP_TINY_DY = 80.f;  // CV / ATT captions
static const float SP_SW_DY = 36.f;    // row 2 switch centre
static const float SP_KNOB2_DY = 34.f; // row 2 knob centre
static const float SP_READ2_DY = 62.f; // row 2 value readout

// Side blocks: two columns of 40 px wide plates
static const float SP_LCOL[2] = {34.f, 90.f};
static const float SP_RCOL[2] = {452.f, 508.f};
static const float SP_SIDE_W = 40.f;

// ------------------------------------------------------------
// Simple Attack-Decay/Release Envelope
// ------------------------------------------------------------
// A minimal two-stage envelope (attack then decay) that optionally sustains
// at full level while a gate is held. Used to shape the sampler's playback.
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

    // Restarts the envelope from the beginning (retrigger).
    void trigger()
    {
        stage = ATTACK;
    }

    // Returns the envelope to IDLE with zero output.
    void reset()
    {
        stage = IDLE;
        level = 0.0f;
    }

    // Advances the envelope by dt and returns its current level (0..1).
    // `sustained` keeps the envelope at full level during decay (gate held or
    // decay time is essentially infinite).
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
// Loads a WAV file at `path` into `left`/`right` (in raw normalised units
// -1..+1; the caller scales by SP_WAV_FULLSCALE) and optionally writes the
// file's sample rate to `fileSampleRate`. Supports PCM 8/16/24/32-bit and
// IEEE 32-bit float, with WAVE_FORMAT_EXTENSIBLE detection. Mono files are
// duplicated into both channels. Returns true on success.
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

// Saves `left`/`right` (in volts) as a 16-bit PCM stereo WAV at `sampleRate`.
// Samples are scaled by SP_WAV_FULLSCALE and clipped to ±32767. Returns true
// on success.
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
// Sampler module: state (buffers, atomics, envelope), DSP, recording /
// overdub, sample editing helpers, and JSON persistence.
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

    // Constructor: configures every parameter, input, output and light,
    // reserves the temp/peak buffers, and sets up the defaults.
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

        configBypass(LEFT_INPUT, LEFT_OUTPUT);
        configBypass(RIGHT_INPUT, RIGHT_OUTPUT);

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
    // Reads `buffer` at fractional position `pos` with linear interpolation,
    // clamping `pos` into the valid range.
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

    // Recomputes the min/max overview used by the waveform display from the
    // current sample. Caller must hold bufferMutex.
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
                mx = std::max(mx, mx);
            }
            peakMin[b] = mn;
            peakMax[b] = mx;
        }
    }

    // Clears the sample, recording state, playhead, envelope and all overview
    // buffers. Caller must hold bufferMutex.
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

    // Starts a new recording session. If `sos` is true and a sample already
    // exists, this becomes a Sound-on-Sound overdub at the current playhead;
    // otherwise the temp buffers are reset and a fresh recording begins.
    // `triggerPlay` is set when playback should also (re)start. Caller must
    // hold bufferMutex.
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

    // Finishes the current recording. Overdubs simply end; a normal
    // recording is moved from the temp buffers into the live sample, the
    // waveform overview is rebuilt, and (if autoPlay) playback starts.
    // Caller must hold bufferMutex.
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
    // Writes input onto the existing sample at index range `[sosLastIdx+1, idx]`
    // (bounded and clamped) so that a slowing playhead still writes each
    // sample only once. Also refreshes the affected overview bin. Caller
    // must hold bufferMutex.
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
    // Loads a WAV file from `path` into the live sample, resets the playhead,
    // records the path for later saves, and rebuilds the overview.
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

    // Opens a WAV file picker and loads the chosen file (if any).
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

    // Opens a save-as dialog and writes the current sample to a WAV file
    // (appending ".wav" if missing). The file is written outside the lock.
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

    // Clears the sample and all associated state (thread-safe wrapper).
    void clearSample()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        clearLocked();
    }

    // Scales the whole sample so that its peak sits at SP_WAV_FULLSCALE.
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

    // Reverses both channels in place and mirrors the playhead so playback
    // continues at the equivalent reversed position.
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

    // Trims the sample to just the current loop region and resets the
    // start/length parameters to 0..1.
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
    // Main audio callback: reads parameters/CVs, handles record/play
    // transport and triggers, advances the playhead (with playback modes and
    // crossfades), applies the envelope and level, mixes with the live input
    // through the MIX knob, writes the outputs, and updates the meters,
    // pulses and lights.
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
    // Serialises either the WAV path (if the audio came from a file) or the
    // raw sample data, plus the buffer's sample rate and the auto-play flag.
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

    // Restores the auto-play flag, sample rate, and either the referenced
    // file or the raw sample data from a saved patch.
    void dataFromJson(json_t *root) override
    {
        if (!root)
            return;
        json_t *ap = json_object_get(root, "autoPlay");
        if (ap)
            autoPlay = json_is_true(ap);

        json_t *sr = json_object_get(root, "sampleRate");
        float storedRate = (sr && json_is_number(sr)) ? (float)json_number_value(sr) : 44100.f;
        if (!std::isfinite(storedRate) || storedRate < 1000.f || storedRate > 768000.f)
            storedRate = 44100.f;

        json_t *pathJson = json_object_get(root, "lastPath");
        if (pathJson && json_is_string(pathJson))
        {
            if (loadFile(json_string_value(pathJson)))
                return;
        }

        json_t *leftArray = json_object_get(root, "leftSamples");
        json_t *rightArray = json_object_get(root, "rightSamples");
        if (json_is_array(leftArray) && json_is_array(rightArray))
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            leftSamples.clear();
            rightSamples.clear();
            size_t len = json_array_size(leftArray);
            size_t maxLen = (size_t)(SP_MAX_REC_SECONDS * 768000.f);
            if (len > maxLen)
                len = maxLen;
            size_t rlen = json_array_size(rightArray);
            leftSamples.reserve(len);
            rightSamples.reserve(len);
            // Reads a finite float from a JSON number, or 0 if not finite.
            auto num = [](json_t *v)
            {
                float f = (float)json_number_value(v);
                return std::isfinite(f) ? f : 0.f;
            };
            for (size_t i = 0; i < len; i++)
            {
                float l = num(json_array_get(leftArray, i));
                leftSamples.push_back(l);
                rightSamples.push_back(i < rlen ? num(json_array_get(rightArray, i)) : l);
            }
            mainBufferSize = len;
            bufferRate = storedRate;
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
    namespace ag = agmm;

    // Returns the record accent colour (pink).
    NVGcolor colRec() { return nvgRGB(0xff, 0x5a, 0x7a); }
    // Returns the lighter record accent used for the recording waveform.
    NVGcolor colRecLight() { return nvgRGB(0xff, 0x9a, 0xb0); }

    // Formats a duration in seconds as "MM:SS.CC" into `buf`.
    void fmtTime(char *buf, size_t n, float secs)
    {
        if (secs < 0.f)
            secs = 0.f;
        int m = (int)(secs / 60.f);
        int s = (int)secs % 60;
        int cs = (int)((secs - std::floor(secs)) * 100.f);
        snprintf(buf, n, "%02d:%02d.%02d", m, s, cs);
    }

    // Block title with hairlines that stop a measured distance from the text
    // Draws a centred block header whose hairlines span from x1 to x2,
    // stopping a measured distance from the text.
    void blockHeader(NVGcontext *vg, float x1, float x2, float y, const char *txt, NVGcolor col)
    {
        const float cx = (x1 + x2) / 2.f;
        const float gap = ag::textWidth(vg, txt, 8.f, 2.f) / 2.f + 6.f;
        ag::text(vg, cx, y, txt, 8.f, col, ag::ALIGN_C, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - gap, y);
        nvgMoveTo(vg, cx + gap, y);
        nvgLineTo(vg, x2, y);
        nvgStrokeColor(vg, ag::withA(col, 0.3f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the LEFT input/transport
    // block, RIGHT output/control block, the sample info strip, and the
    // centre PLAYBACK grid (5 columns × 2 rows).
    struct SPPanelBackground : TransparentWidget
    {
        SPPanelBackground() { box.size = Vec(SP_PANEL_W, SP_PANEL_H); }

        // plate with a label 10 px below its top edge
        // Draws a side-block cell with the given label, using a smaller font
        // for long labels.
        void cell(NVGcontext *vg, float cx, float top, float h, const char *txt, bool tint = false)
        {
            ag::cell(vg, cx, top, SP_SIDE_W, h, txt, tint, txt[0] && strlen(txt) > 6 ? 7.f : 8.f);
        }

        // Draws all static panel art: base panel, bezel, the two block
        // headers, the left/right cells, the info strip plate and the centre
        // playback grid with its CV/ATT captions.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = SP_PANEL_W, H = SP_PANEL_H;

            ag::panelBase(vg, W, H, "SAMPLER");
            ag::bezel(vg, SP_DISP_X, SP_DISP_Y, SP_DISP_W, SP_DISP_H);

            // ---- left block: input + transport ----
            blockHeader(vg, 12.f, 112.f, 34.f, "INPUT", ag::accent());
            cell(vg, SP_LCOL[0], 44.f, 46.f, "L IN");
            cell(vg, SP_LCOL[1], 44.f, 46.f, "R IN");
            cell(vg, SP_LCOL[0], 96.f, 52.f, "MIX");
            cell(vg, SP_LCOL[1], 96.f, 52.f, "CLEAR");
            cell(vg, SP_LCOL[0], 154.f, 52.f, "SOS");
            cell(vg, SP_LCOL[1], 154.f, 52.f, "LINE");

            blockHeader(vg, 12.f, 112.f, 219.f, "TRANSPORT", ag::violet());
            cell(vg, SP_LCOL[0], 228.f, 56.f, "REC");
            cell(vg, SP_LCOL[1], 228.f, 56.f, "REC TRG");
            cell(vg, SP_LCOL[0], 290.f, 56.f, "PLAY");
            cell(vg, SP_LCOL[1], 290.f, 56.f, "PLAY TRG");

            // ---- right block: outputs + control ----
            blockHeader(vg, 432.f, 528.f, 34.f, "OUTPUT", ag::violet());
            cell(vg, SP_RCOL[0], 44.f, 46.f, "L OUT", true);
            cell(vg, SP_RCOL[1], 44.f, 46.f, "R OUT", true);
            cell(vg, SP_RCOL[0], 96.f, 46.f, "EOC", true);
            cell(vg, SP_RCOL[1], 96.f, 46.f, "PLAY GT", true);
            cell(vg, SP_RCOL[0], 148.f, 46.f, "ENV", true);
            cell(vg, SP_RCOL[1], 148.f, 46.f, "REC GT", true);

            blockHeader(vg, 432.f, 528.f, 210.f, "CONTROL", ag::accent());
            cell(vg, SP_RCOL[0], 220.f, 46.f, "GATE IN");
            cell(vg, SP_RCOL[1], 220.f, 46.f, "V/OCT");
            cell(vg, SP_RCOL[0], 272.f, 68.f, "SMOOTH");
            cell(vg, SP_RCOL[1], 272.f, 68.f, "SOS FB");

            // ---- sample info strip ----
            ag::plate(vg, SP_INFO_X + SP_INFO_W / 2.f, SP_INFO_Y, SP_INFO_W, SP_INFO_H);

            // ---- centre grid ----
            blockHeader(vg, 120.f, 422.f, SP_PB_Y, "PLAYBACK", ag::accent());
            const char *r1[5] = {"SPEED", "START", "LENGTH", "ATTACK", "DECAY"};
            for (int c = 0; c < 5; ++c)
                ag::cell(vg, SP_CX[c], SP_R1, SP_CELL_W, SP_R1_H, r1[c]);
            for (int c = 0; c < 3; ++c)
            {
                ag::text(vg, SP_CX[c] - 14.f, SP_R1 + SP_TINY_DY, "CV", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
                ag::text(vg, SP_CX[c] + 15.f, SP_R1 + SP_TINY_DY, "ATT", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            }

            const char *r2[5] = {"TUNING", "MODE", "DIRECTION", "XFADE", "LEVEL"};
            for (int c = 0; c < 5; ++c)
                ag::cell(vg, SP_CX[c], SP_R2, SP_CELL_W, SP_R2_H, r2[c]);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Waveform display (also interactive: drag to select the loop region)
    // ------------------------------------------------------------
    // Live waveform display + loop-region editor. Drag on the waveform to set
    // the start/end of the loop region, double-click to reset to the full
    // sample. Draws the overview, the loop markers, the playhead, level
    // meters, header state chip and footer time readouts.
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
        // Converts an x coordinate (widget-local) to a normalised 0..1
        // position inside the waveform area.
        float toNorm(float x) { return clamp((x - SP_WF_X0) / (float)SP_WF_W, 0.f, 1.f); }

        // Writes the [a, b] region (normalised) into the START and LENGTH
        // parameters, converting to start + relative length.
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

        // Left press starts a new selection from the click position while
        // keeping the existing end (or extending to the end of the sample).
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

        // Drag updates the end of the selection.
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

        // Double-click resets the loop region to the full sample.
        void onDoubleClick(const DoubleClickEvent &e) override
        {
            if (!module)
                return;
            module->paramQuantities[Sampler::START_PARAM]->setValue(0.f);
            module->paramQuantities[Sampler::LENGTH_PARAM]->setValue(1.f);
        }

        // Static background: screen, waveform frame, grid, ruler
        // Draws the display screen, waveform frame, header/footer hairlines,
        // time/voltage grid, zero line and the time-ruler ticks.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = SP_DISP_W;
            const float X0 = SP_WF_X0, Y0 = SP_WF_Y0, WH = SP_WF_H, WW = (float)SP_WF_W;

            ag::screen(vg, 0.f, 0.f, W, SP_DISP_H);
            ag::plotFrame(vg, X0 - 1.f, Y0 - 1.f, WW + 2.f, WH + 2.f);

            // hairlines that separate the header row and the footer row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, 16.5f);
            nvgLineTo(vg, W, 16.5f);
            nvgMoveTo(vg, 0.f, 93.5f);
            nvgLineTo(vg, W, 93.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
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

        // Layer-1 overlay: draws the waveform overview (bright inside the
        // loop region, dim outside), the loop start/end markers with flags,
        // the playhead (pink while overdubbing), the empty-state text, the
        // four level meters (in L/R and out L/R), the header state chip and
        // mode/speed text, and the footer time readouts. Uses a demo scene
        // in the module browser.
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
            // Returns true if the given column falls inside the loop region
            // (all columns count while a brand-new recording is in progress).
            auto inLoop = [&](int c)
            {
                if (recNew)
                    return true;
                float t = (c + 0.5f) / (float)N;
                return t >= sN && t <= eN;
            };

            // ---- waveform: bright inside the loop region, dim outside ----
            if (has)
            {
                for (int pass = 1; pass < 3; ++pass)
                {
                    nvgBeginPath(vg);
                    for (int c = 0; c < N; ++c)
                    {
                        bool in = inLoop(c);
                        if (pass == 1 && !in)
                            continue;
                        if (pass == 2 && in)
                            continue;
                        float y0 = mid - std::min(std::max(colMax[c] / 5.f, -1.f), 1.f) * amp;
                        float y1 = mid - std::min(std::max(colMin[c] / 5.f, -1.f), 1.f) * amp;
                        float h = std::max(y1 - y0, 1.f);
                        nvgRect(vg, X0 + c, y0, 1.f, h);
                    }
                    if (recNew)
                        nvgFillColor(vg, colRecLight());
                    else if (pass == 1)
                        nvgFillColor(vg, ag::bright());
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
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, xs, Y0);
                nvgLineTo(vg, xs + 6.f, Y0);
                nvgLineTo(vg, xs, Y0 + 6.f);
                nvgClosePath(vg);
                nvgFillColor(vg, ag::bright());
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
                nvgFillColor(vg, ag::violet());
                nvgFill(vg);
            }

            // ---- playhead (pink while overdubbing) ----
            if (has && play && !recNew)
            {
                float x = X0 + clamp(ph, 0.f, 1.f) * WW;
                NVGcolor pc = sos ? colRecLight() : nvgRGB(255, 255, 255);
                nvgBeginPath(vg);
                nvgMoveTo(vg, x, Y0);
                nvgLineTo(vg, x, Y0 + WH);
                nvgStrokeColor(vg, ag::withA(pc, 0.94f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // ---- empty state ----
            if (!has && !rec)
            {
                ag::text(vg, X0 + WW / 2.f, mid - 6.f, "NO SAMPLE", 10.f, ag::dim(), ag::ALIGN_C, 3.f);
                ag::text(vg, X0 + WW / 2.f, mid + 8.f, "REC OR RIGHT-CLICK > LOAD WAV", 7.f,
                         ag::dim(), ag::ALIGN_C, 0.5f);
            }

            // ---- level meters (left: input, right: output) ----
            ag::meter(vg, 3.f, Y0, 4.f, WH, mIL / 5.f);
            ag::meter(vg, 9.f, Y0, 4.f, WH, mIR / 5.f);
            ag::meter(vg, W - 13.f, Y0, 4.f, WH, mOL / 5.f);
            ag::meter(vg, W - 7.f, Y0, 4.f, WH, mOR / 5.f);
            ag::text(vg, 8.f, Y0 + WH + 8.f, "IN", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, W - 10.f, Y0 + WH + 8.f, "OUT", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            // ---- header row: state chip, mode, speed ----
            {
                const char *st = rec ? (sos ? "SOS" : "REC") : (play ? "PLAY" : "STOP");
                NVGcolor sc = rec ? colRec() : (play ? ag::accent() : ag::dim());
                const float cw = 30.f;
                ag::chip(vg, 6.f, 2.f, cw, 12.f, st, play || rec, sc);

                static const char *modeNames[3] = {"ONE-SHOT", "LOOP", "PING-PONG"};
                char buf[64];
                snprintf(buf, sizeof(buf), "%s | %s", modeNames[playMode], (rev ? "REV" : "FWD"));
                ag::text(vg, W / 2.f, 8.5f, buf, 7.f, ag::withA(ag::accent(), 0.8f), ag::ALIGN_C, 0.5f);

                snprintf(buf, sizeof(buf), pitchMode ? "TUNE %.2fX" : "SPD %+.2fX", speed);
                ag::text(vg, W - 6.f, 8.5f, buf, 7.f, ag::withA(ag::accent(), 0.8f), ag::ALIGN_R, 0.5f);
            }

            // ---- footer row: time and loop region ----
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
                ag::text(vg, 6.f, H - 6.f, buf, 7.f, nvgRGB(0xff, 0xe6, 0x80), ag::ALIGN_L, 0.5f);

                if (has && !recNew)
                {
                    fmtTime(a, sizeof(a), sN * secsTotal);
                    fmtTime(b, sizeof(b), eN * secsTotal);
                    snprintf(buf, sizeof(buf), "%s - %s", a, b);
                    ag::text(vg, W - 6.f, H - 6.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.5f);
                }
            }

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Value readout under a knob / switch
    // ------------------------------------------------------------
    // A small text widget that reads a given parameter and prints it in the
    // requested format (seconds, ms, %, pitch mode, play mode or direction).
    // Uses a fixed demo value in the module browser.
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

        // Constructor: positions the readout centred on (cx, cy).
        SPReadout(Sampler *m, int p, Fmt f, float demoVal, float cx, float cy) : module(m), paramId(p), fmt(f), demo(demoVal)
        {
            box.pos = Vec(cx - 28.f, cy - 6.f);
            box.size = Vec(56.f, 12.f);
        }

        // Layer-1 overlay: reads the parameter (or demo value) and prints it
        // according to the chosen format.
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
            ag::text(vg, box.size.x / 2.f, box.size.y / 2.f, buf, 7.f, ag::accent(), ag::ALIGN_C, 0.5f);
            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Sample info strip (file name, length, sample rate)
    // ------------------------------------------------------------
    // Live info strip: shows the loaded file name (or "RECORDED SAMPLE" /
    // "NO SAMPLE LOADED") on the left and the total length + sample rate on
    // the right.
    struct SPInfoWidget : TransparentWidget
    {
        Sampler *module = nullptr;

        SPInfoWidget(Sampler *m) : module(m)
        {
            box.pos = Vec(SP_INFO_X, SP_INFO_Y);
            box.size = Vec(SP_INFO_W, SP_INFO_H);
        }

        // Layer-1 overlay: reads the buffer state under the mutex and prints
        // the file name (trimmed to 24 chars) plus duration/rate.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float cy = box.size.y / 2.f + 0.5f;

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
            if (name.size() > 24)
                name = name.substr(0, 21) + "...";

            ag::text(vg, 8.f, cy, name.c_str(), 7.f, total > 0 ? ag::textCol() : ag::dim(), ag::ALIGN_L, 0.5f);

            if (total > 0 && rate > 0.f)
            {
                char t[16], buf[48];
                fmtTime(t, sizeof(t), (float)total / rate);
                snprintf(buf, sizeof(buf), "%s  %.1fKHZ", t, rate / 1000.f);
                ag::text(vg, box.size.x - 8.f, cy, buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            }
            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
// Places the static panel, screws, waveform display, info strip, all controls
// and jacks on the left/right blocks and the centre playback grid, and builds
// the right-click menu (load/save WAV, editing tools, auto-play toggle).
struct SamplerWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, display, info strip,
    // all left/right block controls and jacks, and the centre playback grid.
    SamplerWidget(Sampler *module)
    {
        setModule(module);
        box.size = Vec(SP_PANEL_W, SP_PANEL_H);

        addChild(new SPPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SPDisplayWidget(module));
        addChild(new SPInfoWidget(module));

        // ---- left block: input ----
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_LCOL[0], 75.f), module, Sampler::LEFT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_LCOL[1], 75.f), module, Sampler::RIGHT_INPUT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_LCOL[0], 127.f), module, Sampler::MIX_PARAM));
        addParam(createParamCentered<VCVButton>(Vec(SP_LCOL[1], 127.f), module, Sampler::CLEAR_BUTTON));

        // ---- left block: SOS / LINE (latching buttons with yellow LEDs above) ----
        auto sosButton = createParamCentered<VCVButton>(Vec(SP_LCOL[0], 192.f), module, Sampler::SOS_SWITCH);
        sosButton->momentary = false;
        addParam(sosButton);
        addChild(createLightCentered<SmallLight<YellowLight>>(Vec(SP_LCOL[0], 177.f), module, Sampler::SOS_LIGHT));

        auto lineButton = createParamCentered<VCVButton>(Vec(SP_LCOL[1], 192.f), module, Sampler::LINE_SWITCH);
        lineButton->momentary = false;
        addParam(lineButton);
        addChild(createLightCentered<SmallLight<YellowLight>>(Vec(SP_LCOL[1], 177.f), module, Sampler::LINE_LIGHT));

        // ---- left block: transport (cells at 228 and 290, controls at +38, LEDs at +21) ----
        auto recButton = createParamCentered<VCVButton>(Vec(SP_LCOL[0], 266.f), module, Sampler::REC_BUTTON);
        recButton->momentary = false;
        addParam(recButton);
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(SP_LCOL[0], 249.f), module, Sampler::REC_LIGHT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_LCOL[1], 266.f), module, Sampler::RECORD_TRIGGER));

        auto playButton = createParamCentered<VCVButton>(Vec(SP_LCOL[0], 328.f), module, Sampler::PLAY_BUTTON);
        playButton->momentary = false;
        addParam(playButton);
        addChild(createLightCentered<SmallLight<ag::CyanLight>>(Vec(SP_LCOL[0], 311.f), module, Sampler::PLAY_LIGHT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_LCOL[1], 328.f), module, Sampler::PLAY_TRIGGER));

        // ---- right block: outputs (cells at 44 / 96 / 148, jack at +31) ----
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[0], 75.f), module, Sampler::LEFT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[1], 75.f), module, Sampler::RIGHT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[0], 127.f), module, Sampler::EOC_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[1], 127.f), module, Sampler::PLAY_GATE_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[0], 179.f), module, Sampler::ENV_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(SP_RCOL[1], 179.f), module, Sampler::REC_GATE_OUTPUT));

        // ---- right block: control inputs and the two small knobs ----
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_RCOL[0], 251.f), module, Sampler::GATE_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_RCOL[1], 251.f), module, Sampler::PITCH_CV_INPUT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_RCOL[0], 302.f), module, Sampler::DECLICK_PARAM));
        addChild(new SPReadout(module, Sampler::DECLICK_PARAM, SPReadout::MILLIS, 3.f, SP_RCOL[0], 328.f));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_RCOL[1], 302.f), module, Sampler::SOS_FDBK_PARAM));
        addChild(new SPReadout(module, Sampler::SOS_FDBK_PARAM, SPReadout::PERCENT, 1.f, SP_RCOL[1], 328.f));

        // ---- centre row 1: SPEED, START, LENGTH (knob + CV jack + amount), ATTACK, DECAY ----
        const float k1 = SP_R1 + SP_KNOB_DY, j1 = SP_R1 + SP_JACK_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[0], k1), module, Sampler::SPEED_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_CX[0] - 14.f, j1), module, Sampler::SPEED_CV_INPUT));
        addParam(createParamCentered<Trimpot>(Vec(SP_CX[0] + 15.f, j1), module, Sampler::SPEED_CV_ATTEN));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[1], k1), module, Sampler::START_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_CX[1] - 14.f, j1), module, Sampler::START_CV_INPUT));
        addParam(createParamCentered<Trimpot>(Vec(SP_CX[1] + 15.f, j1), module, Sampler::START_CV_ATTEN));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[2], k1), module, Sampler::LENGTH_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SP_CX[2] - 14.f, j1), module, Sampler::LENGTH_CV_INPUT));
        addParam(createParamCentered<Trimpot>(Vec(SP_CX[2] + 15.f, j1), module, Sampler::LENGTH_CV_ATTEN));

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[3], k1), module, Sampler::ATTACK_PARAM));
        addChild(new SPReadout(module, Sampler::ATTACK_PARAM, SPReadout::SECONDS, 0.f, SP_CX[3], j1));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[4], k1), module, Sampler::DECAY_PARAM));
        addChild(new SPReadout(module, Sampler::DECAY_PARAM, SPReadout::DECAY_SECONDS, 10.f, SP_CX[4], j1));

        // ---- centre row 2: switches, XFADE, LEVEL ----
        const float s2 = SP_R2 + SP_SW_DY, r2 = SP_R2 + SP_READ2_DY;
        addParam(createParamCentered<CKSS>(Vec(SP_CX[0], s2), module, Sampler::PITCH_MODE_PARAM));
        addChild(new SPReadout(module, Sampler::PITCH_MODE_PARAM, SPReadout::PITCH_MODE, 0.f, SP_CX[0], r2));
        addParam(createParamCentered<CKSSThree>(Vec(SP_CX[1], s2), module, Sampler::PLAY_MODE_PARAM));
        addChild(new SPReadout(module, Sampler::PLAY_MODE_PARAM, SPReadout::PLAY_MODE, 1.f, SP_CX[1], r2));
        addParam(createParamCentered<CKSS>(Vec(SP_CX[2], s2), module, Sampler::DIRECTION_PARAM));
        addChild(new SPReadout(module, Sampler::DIRECTION_PARAM, SPReadout::DIRECTION, 0.f, SP_CX[2], r2));

        const float k2 = SP_R2 + SP_KNOB2_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[3], k2), module, Sampler::XFADE_PARAM));
        addChild(new SPReadout(module, Sampler::XFADE_PARAM, SPReadout::MILLIS, 15.f, SP_CX[3], r2));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_CX[4], k2), module, Sampler::LEVEL_PARAM));
        addChild(new SPReadout(module, Sampler::LEVEL_PARAM, SPReadout::PERCENT, 1.f, SP_CX[4], r2));
        agmm::addKnobArcs(this, 2.f);
    }

    // Adds the right-click menu: load/save WAV dialogs, editing actions
    // (normalize, reverse, trim to loop, clear) and the auto-play toggle.
    void appendContextMenu(Menu *menu) override
    {
        agmm::addYouTubeLink(menu); // adds "Visit my youtube channel!"
        agmm::addGitLink(menu); // adds "Visit my git repo!"
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
        menu->addChild(createMenuItem("Auto-play after recording", CHECKMARK(module->autoPlay), [=]()
                                      { module->autoPlay = !module->autoPlay; }));
    }
};

Model *modelSampler = createModel<Sampler, SamplerWidget>("Sampler");