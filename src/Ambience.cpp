#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <memory>
#include <algorithm>
// ANDREAS GOGOS

// =============================================================================
// AMBIENCE — algorithmic reverb (feedback delay network)
//
// Signal flow, in one line:
//
//   in -> pre-delay -> 4x all-pass (diffusion) -> FDN (4 delay lines,
//   damping low-pass per line, orthogonal feedback matrix, decay gain)
//   -> stereo tap mix -> shelving EQ -> wet gain -> dry/wet -> fade-in -> out
//
// The FDN uses four delay lines whose lengths are scaled by the SPACE knob.
// Feedback is made lossless-ish by an orthogonal mixing matrix (so the tail
// does not build up or cancel), and each line is damped with a 12 dB/oct
// low-pass before it is fed back: high frequencies die first, like air.
// =============================================================================

// =============================================================================
// Layout (24 HP = 360 x 380 px)
//
//   y   0-15   screws
//   y  15      title
//   y  28-128  display (titles / plots / axis labels / readouts, one row each)
//   y 142      section: REVERB
//   y 152-216  row 1: PRE-DELAY, SPACE, DECAY, DAMPING   (label / knob)
//   y 222-286  row 2: WET GAIN, MIX, MOD, FREEZE         (label / knob|switch)
//   y 296      section: I / O
//   y 304-356  7 jack cells (label / jack)
//   y 368      maker tag (between the bottom screws)
//
// Every control lives inside its plate with clear margins; no label shares
// space with a widget.
// =============================================================================
static const float AM_PANEL_W = RACK_GRID_WIDTH * 24; // 360
static const float AM_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float AM_TITLE_Y = 15.f;
static const float AM_MAKER_Y = 368.f;

// Display
static const float AM_DISP_X = 20.f;
static const float AM_DISP_Y = 28.f;
static const float AM_DISP_W = 320.f;
static const float AM_DISP_H = 100.f;

// Display rows (relative to the display)
static const float AM_D_TITLE_Y = 7.f;
static const float AM_D_PLOT_Y = 14.f;
static const float AM_D_PLOT_H = 50.f;
static const float AM_D_AXIS_Y = 71.f;
static const float AM_D_READ_Y = 89.f;
// Display columns
static const float AM_D_DECAY_X = 8.f, AM_D_DECAY_W = 150.f;
static const float AM_D_DAMP_X = 170.f, AM_D_DAMP_W = 118.f;
static const float AM_D_METER_X = 298.f; // 3 bars, 4 px wide, 8 px pitch

// Controls
static const float AM_SEC1_Y = 142.f;
// Four evenly spaced knob columns across the 360 px panel
static const float AM_COL_CX[4] = {49.5f, 136.5f, 223.5f, 310.5f};
static const float AM_CELL_W = 75.f;
static const float AM_CELL_H = 64.f;
static const float AM_ROW_TOP[2] = {152.f, 222.f};
static const float AM_CELL_LABEL = 10.f; // label offset from plate top
static const float AM_CELL_KNOB = 39.f;  // knob centre offset from plate top

// I/O
static const float AM_SEC2_Y = 296.f;
// Seven jack cells: five inputs then two outputs, 48 px apart
static float amIoX(int i) { return 36.f + 48.f * i; }
static const float AM_IO_PLATE_TOP = 304.f, AM_IO_PLATE_H = 52.f, AM_IO_PLATE_W = 44.f;
static const float AM_IO_LABEL_Y = 314.f;
static const float AM_IO_JACK_Y = 338.f;

// Base delay lengths of the FDN (used by the display for the RT60 estimate)
// These are the four round-trip times at SPACE = 1.0, in seconds.
static const float AM_BASE[4] = {0.0317f, 0.0383f, 0.0429f, 0.0451f};

// -----------------------------------------------------------------------------
// 2nd order Butterworth lowpass (12 dB/oct)
//
// Used as the damping filter inside each FDN branch. Coefficients follow the
// RBJ cookbook low-pass with Q = 1/sqrt(2), i.e. maximally flat (Butterworth).
// The filter is stateful (transposed direct form I), so each delay line owns
// its own instance.
// -----------------------------------------------------------------------------
class AmbLowpass2
{
public:
    AmbLowpass2() : b0_(0), b1_(0), b2_(0), a1_(0), a2_(0), x1_(0), x2_(0), y1_(0), y2_(0) {}
    void setCutoff(float freq, float sr)
    {
        // Clamp to a sane range: never below 10 Hz, never above Nyquist.
        if (freq <= 0)
            freq = 10;
        if (freq >= sr * 0.49f)
            freq = sr * 0.49f;
        // Standard bilinear-transform design for a 2-pole low-pass.
        float w0 = 2.0f * float(M_PI) * freq / sr;
        float cos_w0 = std::cos(w0);
        float sin_w0 = std::sin(w0);
        float Q = 0.70710678f;
        float alpha = sin_w0 / (2.0f * Q);
        float a0 = 1.0f + alpha;
        b0_ = (1.0f - cos_w0) / (2.0f * a0);
        b1_ = (1.0f - cos_w0) / a0;
        b2_ = (1.0f - cos_w0) / (2.0f * a0);
        a1_ = (-2.0f * cos_w0) / a0;
        a2_ = (1.0f - alpha) / a0;
    }
    float process(float in)
    {
        // Direct form I difference equation with two samples of history.
        float out = b0_ * in + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
        x2_ = x1_;
        x1_ = in;
        y2_ = y1_;
        y1_ = out;
        return out;
    }
    void reset() { x1_ = x2_ = y1_ = y2_ = 0; }

private:
    float b0_, b1_, b2_, a1_, a2_;
    float x1_, x2_, y1_, y2_;
};

// -----------------------------------------------------------------------------
// Delay line with fixed max capacity (no reallocation on Space change)
//
// The buffer is sized once for the longest possible delay (base * maxSpace).
// Changing SPACE only moves the read pointer, so no allocation happens on the
// audio thread and the tail does not click when the knob is turned.
//
// Read position is fractional and linearly interpolated, which is what makes
// the slow LFO modulation (modOffset_) possible without zipper noise.
// -----------------------------------------------------------------------------
class AmbDelayLine
{
public:
    AmbDelayLine() : buf_(), writePos_(0), delaySamples_(0), maxSamples_(0), modOffset_(0) {}
    void setMaxDelay(float maxSec, float sr)
    {
        // +2 samples of headroom so i0/i1 interpolation never runs off the end.
        maxSamples_ = std::max(1, int(maxSec * sr));
        buf_.resize(maxSamples_ + 2, 0.0f);
        writePos_ = 0;
        delaySamples_ = maxSamples_;
    }
    void setDelaySamples(int samples)
    {
        // Always keep at least one sample of delay: a zero-length delay inside
        // a feedback loop would be a unity-gain path and could run away.
        delaySamples_ = std::max(1, std::min(samples, maxSamples_));
    }
    void setModOffset(float offset) { modOffset_ = offset; }
    float read() const
    {
        // Write pointer minus delay, wrapped into the buffer, plus modulation.
        float readPos = float(writePos_) - float(delaySamples_) + modOffset_;
        if (readPos < 0)
            readPos += buf_.size();
        if (readPos >= buf_.size())
            readPos -= buf_.size();
        // Linear interpolation between the two neighbouring taps.
        int i0 = int(readPos);
        int i1 = i0 + 1;
        if (i1 >= (int)buf_.size())
            i1 -= buf_.size();
        float frac = readPos - i0;
        return buf_[i0] * (1.f - frac) + buf_[i1] * frac;
    }
    void write(float val)
    {
        buf_[writePos_] = val;
        writePos_ = (writePos_ + 1) % buf_.size();
    }
    void reset()
    {
        std::fill(buf_.begin(), buf_.end(), 0.0f);
        writePos_ = 0;
        modOffset_ = 0;
    }

private:
    std::vector<float> buf_;
    int writePos_, delaySamples_, maxSamples_;
    float modOffset_;
};

// -----------------------------------------------------------------------------
// All-pass filter for density
//
// Four of these run in series on the input. They smear transients into a
// dense "cloud" before the sound enters the FDN, which is what makes a
// reverb sound like a room rather than a set of discrete echoes.
// -----------------------------------------------------------------------------
class AmbAllPass
{
public:
    AmbAllPass() : buf_(), idx_(0), coeff_(0.6f) {}
    void setDelay(float sec, float sr)
    {
        int samples = std::max(1, int(sec * sr));
        buf_.resize(samples, 0.0f);
        idx_ = 0;
    }
    void setCoeff(float c) { coeff_ = c; }
    float process(float in)
    {
        // Schroeder all-pass: output = delayed - g*in, and the delayed sample
        // plus g*delayed is what gets stored. All-pass = flat magnitude.
        float dly = buf_[idx_];
        float out = -coeff_ * in + dly + coeff_ * dly;
        buf_[idx_] = in;
        idx_ = (idx_ + 1) % buf_.size();
        return out;
    }
    void reset()
    {
        std::fill(buf_.begin(), buf_.end(), 0.0f);
        idx_ = 0;
    }

private:
    std::vector<float> buf_;
    size_t idx_;
    float coeff_;
};

// -----------------------------------------------------------------------------
// Slow LFO for subtle modulation (sine)
//
// A very slow sine (default 0.3 Hz) shifts all delay-read positions together.
// Because the shifts are sub-sample to a few samples, this breaks up the
// metallic ringing of a static FDN without audible pitch wobble.
// -----------------------------------------------------------------------------
class AmbModLFO
{
public:
    AmbModLFO() : phase_(0), rate_(0.3f) {}
    void process(float sr)
    {
        phase_ += rate_ / sr;
        if (phase_ >= 1.0f)
            phase_ -= 1.0f;
    }
    float getValue() const { return std::sin(phase_ * 2.0f * float(M_PI)); }
    void reset() { phase_ = 0; }

private:
    float phase_, rate_;
};

// -----------------------------------------------------------------------------
// Pre-delay with fixed max capacity
//
// Gives the dry attack a gap before the reverb starts, which is the main cue
// the ear uses to judge room size. Buffer is preallocated for the maximum
// (200 ms * maxSpace) so turning the knob never allocates.
// -----------------------------------------------------------------------------
class AmbPreDelay
{
public:
    AmbPreDelay() : buf_(), idx_(0), maxSamples_(0), delaySamples_(0) {}
    void setMaxDelay(float maxSec, float sr)
    {
        maxSamples_ = std::max(1, int(maxSec * sr));
        buf_.resize(maxSamples_, 0.0f);
        idx_ = 0;
        delaySamples_ = maxSamples_;
    }
    void setDelaySamples(int samples) { delaySamples_ = std::max(1, std::min(samples, maxSamples_)); }
    float process(float in)
    {
        // Simple ring-buffer read/write: no interpolation needed here.
        int readPos = idx_ - delaySamples_;
        if (readPos < 0)
            readPos += maxSamples_;
        float out = buf_[readPos];
        buf_[idx_] = in;
        idx_ = (idx_ + 1) % maxSamples_;
        return out;
    }
    void reset()
    {
        std::fill(buf_.begin(), buf_.end(), 0.0f);
        idx_ = 0;
    }

private:
    std::vector<float> buf_;
    int idx_, maxSamples_, delaySamples_;
};

// -----------------------------------------------------------------------------
// Output shelving filters
//
// Fixed, gentle tonal shaping on the wet signal only (low shelf -3 dB at
// 150 Hz, high shelf -4 dB at 5 kHz). Without this the tail can sound bright
// and "splashy"; the shelves make it sit further back in the mix.
// -----------------------------------------------------------------------------

// Low shelf: boosts/cuts everything below `freq` by `gainDb`.
class AmbLowShelf
{
public:
    AmbLowShelf() : b0_(1), b1_(0), b2_(0), a1_(0), a2_(0), x1_(0), x2_(0), y1_(0), y2_(0) {}
    void setGain(float gainDb, float freq, float sr)
    {
        // RBJ low-shelf design, slope S = 1.
        float w0 = 2.0f * float(M_PI) * freq / sr;
        float cos_w0 = std::cos(w0);
        float sin_w0 = std::sin(w0);
        float S = 1.0f;
        float A = std::pow(10.0f, gainDb / 40.0f);
        float beta = std::sqrt(A) / S;
        float a0 = (A + 1) + (A - 1) * cos_w0 + 2 * beta * std::sqrt(A) * sin_w0;
        b0_ = A * ((A + 1) - (A - 1) * cos_w0 + 2 * beta * std::sqrt(A) * sin_w0) / a0;
        b1_ = 2 * A * ((A - 1) - (A + 1) * cos_w0) / a0;
        b2_ = A * ((A + 1) - (A - 1) * cos_w0 - 2 * beta * std::sqrt(A) * sin_w0) / a0;
        a1_ = -2 * ((A - 1) + (A + 1) * cos_w0) / a0;
        a2_ = ((A + 1) + (A - 1) * cos_w0 - 2 * beta * std::sqrt(A) * sin_w0) / a0;
    }
    float process(float in)
    {
        // Direct form I difference equation with two samples of history.
        float out = b0_ * in + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
        x2_ = x1_;
        x1_ = in;
        y2_ = y1_;
        y1_ = out;
        return out;
    }
    void reset() { x1_ = x2_ = y1_ = y2_ = 0; }

private:
    float b0_, b1_, b2_, a1_, a2_;
    float x1_, x2_, y1_, y2_;
};

// High shelf: boosts/cuts everything above `freq` by `gainDb`.
class AmbHighShelf
{
public:
    AmbHighShelf() : b0_(1), b1_(0), b2_(0), a1_(0), a2_(0), x1_(0), x2_(0), y1_(0), y2_(0) {}
    void setGain(float gainDb, float freq, float sr)
    {
        // RBJ high-shelf design, slope S = 1.
        float w0 = 2.0f * float(M_PI) * freq / sr;
        float cos_w0 = std::cos(w0);
        float sin_w0 = std::sin(w0);
        float S = 1.0f;
        float A = std::pow(10.0f, gainDb / 40.0f);
        float beta = std::sqrt(A) / S;
        float a0 = (A + 1) - (A - 1) * cos_w0 + 2 * beta * std::sqrt(A) * sin_w0;
        b0_ = A * ((A + 1) + (A - 1) * cos_w0 + 2 * beta * std::sqrt(A) * sin_w0) / a0;
        b1_ = -2 * A * ((A - 1) + (A + 1) * cos_w0) / a0;
        b2_ = A * ((A + 1) + (A - 1) * cos_w0 - 2 * beta * std::sqrt(A) * sin_w0) / a0;
        a1_ = 2 * ((A - 1) - (A + 1) * cos_w0) / a0;
        a2_ = ((A + 1) - (A - 1) * cos_w0 - 2 * beta * std::sqrt(A) * sin_w0) / a0;
    }
    float process(float in)
    {
        // Direct form I difference equation with two samples of history.
        float out = b0_ * in + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
        x2_ = x1_;
        x1_ = in;
        y2_ = y1_;
        y1_ = out;
        return out;
    }
    void reset() { x1_ = x2_ = y1_ = y2_ = 0; }

private:
    float b0_, b1_, b2_, a1_, a2_;
    float x1_, x2_, y1_, y2_;
};

// -----------------------------------------------------------------------------
// FDN Reverb Module
//
// Four delay lines, each with its own damping filter. On every sample:
//   1. read all four lines (with modulation),
//   2. low-pass each read sample (damping),
//   3. mix the four through an orthogonal matrix,
//   4. multiply by the decay gain and add the (diffused) input,
//   5. write back.
// The matrix keeps the total energy constant, so only the decay gain decides
// how fast the tail dies; that is what makes the DECAY knob behave musically.
// -----------------------------------------------------------------------------
struct Ambience : Module
{
    enum ParamId
    {
        PRE_DELAY_PARAM,
        SPACE_PARAM,
        DECAY_PARAM,
        DAMPING_PARAM,
        WET_GAIN_PARAM,
        MIX_PARAM,
        MOD_DEPTH_PARAM,
        FREEZE_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_L,
        INPUT_R,
        DECAY_CV_INPUT,
        MIX_CV_INPUT,
        FREEZE_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_L,
        OUTPUT_R,
        OUTPUTS_LEN
    };
    enum LightId
    {
        FREEZE_LIGHT,
        LIGHTS_LEN
    };

    static const int kNumDelays = 4;
    // Round-trip times of the four lines at SPACE = 1.0 (seconds). The values
    // are mutually prime-ish so the echoes do not line up into a pitch.
    const float baseDelays_[kNumDelays] = {0.0317f, 0.0383f, 0.0429f, 0.0451f};
    // Injection gain per line: equal input into all four keeps the excitation
    // balanced across the network.
    const float inGains_[kNumDelays] = {0.5f, 0.5f, 0.5f, 0.5f};
    // Stereo tap gains. Different signs/weights on L and R create the width
    // and the decorrelated "two ears" impression.
    const float outGainsL_[kNumDelays] = {0.6f, -0.4f, 0.5f, -0.3f};
    const float outGainsR_[kNumDelays] = {0.5f, 0.5f, -0.4f, -0.4f};

    float sr;
    bool srChanged;
    float maxSpace = 2.5f;    // SPACE knob maximum (delay scaling)
    float maxPreDelay = 0.2f; // pre-delay maximum in seconds

    AmbPreDelay pre;
    AmbDelayLine delays[kNumDelays];
    AmbLowpass2 filters[kNumDelays];
    AmbModLFO modLFO;
    AmbAllPass inputAPs[4];
    float apDelays[4] = {0.0021f, 0.0037f, 0.0054f, 0.0072f};
    AmbLowShelf lowShelf;
    AmbHighShelf highShelf;

    // One sample of every delay line, captured each tick and used both for the
    // feedback matrix and for the stereo wet taps.
    float outBuf[kNumDelays];

    // Fade-in
    int fadeCounter;
    int fadeSamples;

    // Freeze
    float freezeAmt = 0.f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float meterIn = 0.f, meterOutL = 0.f, meterOutR = 0.f;
    float dispDecay = 0.92f, dispMix = 0.5f;

    Ambience()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        // Pre-delay is configured in seconds and displayed in milliseconds.
        configParam(PRE_DELAY_PARAM, 0.0f, maxPreDelay, 0.02f, "Pre-delay", " ms", 0.f, 1000.f);
        // SPACE scales every delay length; 1.0 = the base times above.
        configParam(SPACE_PARAM, 0.5f, maxSpace, 1.0f, "Space", "x");
        // DECAY is the per-pass feedback gain. It is displayed as a percentage.
        configParam(DECAY_PARAM, 0.5f, 0.999f, 0.92f, "Decay", "%", 0.f, 100.f);
        // DAMPING is the cutoff of the low-pass inside each delay line.
        configParam(DAMPING_PARAM, 200.0f, 8000.0f, 1800.0f, "Damping", " Hz");
        // WET_GAIN trims the reverb only, so MIX can stay a pure balance.
        configParam(WET_GAIN_PARAM, 0.0f, 2.0f, 1.0f, "Wet gain", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.0f, 1.0f, 0.5f, "Dry/wet", "%", 0.f, 100.f);
        // MOD is the peak deviation of the read pointer, in samples.
        configParam(MOD_DEPTH_PARAM, 0.0f, 6.0f, 2.0f, "Modulation depth", " samples");
        configSwitch(FREEZE_PARAM, 0.f, 1.f, 0.f, "Freeze", {"Off", "On"});

        configInput(INPUT_L, "Left (mono if the right input is empty)");
        configInput(INPUT_R, "Right");
        // 10 V of decay CV adds 50 percentage points of feedback.
        configInput(DECAY_CV_INPUT, "Decay CV (10 V adds 50%)");
        // 10 V of mix CV adds 100 percentage points (fully wet).
        configInput(MIX_CV_INPUT, "Dry/wet CV (10 V adds 100%)");
        configInput(FREEZE_INPUT, "Freeze gate");
        configOutput(OUTPUT_L, "Left");
        configOutput(OUTPUT_R, "Right");
        configLight(FREEZE_LIGHT, "Freeze");

        configBypass(INPUT_L, OUTPUT_L);
        configBypass(INPUT_R, OUTPUT_R);

        sr = 0;
        srChanged = true;
        fadeCounter = 0;
        fadeSamples = 0;
        for (int i = 0; i < kNumDelays; ++i)
            outBuf[i] = 0.f;
    }

    void onSampleRateChange() override { srChanged = true; }
    void onReset() override
    {
        // Clear every stateful block so a reset never leaves a tail behind.
        pre.reset();
        for (int i = 0; i < kNumDelays; ++i)
        {
            delays[i].reset();
            filters[i].reset();
        }
        for (int i = 0; i < 4; ++i)
            inputAPs[i].reset();
        modLFO.reset();
        lowShelf.reset();
        highShelf.reset();
        fadeCounter = 0;
        freezeAmt = 0.f;
    }

    void applyFeedbackMatrix(float *x)
    {
        // Orthogonal (Householder-style) mixing: subtract the mean * 2 from
        // every line. The transform is its own inverse up to sign and preserves
        // energy, so the network diffuses without amplifying or cancelling.
        float sum = 0;
        for (int i = 0; i < kNumDelays; ++i)
            sum += x[i];
        float factor = (2.0f / kNumDelays) * sum;
        for (int i = 0; i < kNumDelays; ++i)
            x[i] -= factor;
    }

    void process(const ProcessArgs &args) override
    {
        // ---- sample-rate dependent setup (also handles a live rate change) ----
        if (srChanged || args.sampleRate != sr)
        {
            sr = args.sampleRate;
            srChanged = false;
            // 0.5 s fade-in after a rate change avoids a click from the reset
            // of the delay buffers.
            fadeSamples = int(0.5f * sr); // 0.5 second fade-in
            fadeCounter = 0;

            // Buffers are allocated here, once, for the worst case.
            pre.setMaxDelay(maxPreDelay * maxSpace, sr);
            for (int i = 0; i < kNumDelays; ++i)
            {
                delays[i].setMaxDelay(baseDelays_[i] * maxSpace, sr);
            }
            for (int i = 0; i < 4; ++i)
            {
                inputAPs[i].setDelay(apDelays[i], sr);
                inputAPs[i].setCoeff(0.6f);
            }
            // Fixed output voicing (see the shelf classes above).
            lowShelf.setGain(-3.0f, 150.0f, sr);
            highShelf.setGain(-4.0f, 5000.0f, sr);
        }

        // ---- parameters (knob + CV) ----
        float preTime = params[PRE_DELAY_PARAM].getValue();
        float space = params[SPACE_PARAM].getValue();
        float decay = params[DECAY_PARAM].getValue();
        decay = clamp(decay + inputs[DECAY_CV_INPUT].getVoltage() * 0.05f, 0.5f, 0.999f);
        float dampFreq = params[DAMPING_PARAM].getValue();
        float wetGain = params[WET_GAIN_PARAM].getValue();
        float mix = params[MIX_PARAM].getValue();
        mix = clamp(mix + inputs[MIX_CV_INPUT].getVoltage() * 0.1f, 0.0f, 1.0f);
        float modDepthSamples = params[MOD_DEPTH_PARAM].getValue();
        // Snapshot for the display (same values the audio path uses).
        dispDecay = decay;
        dispMix = mix;

        // ---- freeze: holds the tail (decay -> ~1) and closes the input ----
        bool freezeOn = params[FREEZE_PARAM].getValue() > 0.5f || inputs[FREEZE_INPUT].getVoltage() > 1.0f;
        // Smooth the freeze amount so toggling never clicks.
        freezeAmt += ((freezeOn ? 1.0f : 0.0f) - freezeAmt) * std::min(1.0f, 20.0f * args.sampleTime);
        lights[FREEZE_LIGHT].setBrightness(freezeAmt);
        // At full freeze the decay is pushed to ~1 (endless) and the input
        // gain to 0, so the existing tail keeps circulating untouched.
        float effDecay = decay + (0.9995f - decay) * freezeAmt;
        float inGain = 1.0f - freezeAmt;

        // ---- delay lengths (updated without clearing the buffers) ----
        int preSamples = std::max(1, int(preTime * space * sr + 0.5f));
        pre.setDelaySamples(preSamples);
        for (int i = 0; i < kNumDelays; ++i)
        {
            int dlySamples = std::max(1, int(baseDelays_[i] * space * sr + 0.5f));
            delays[i].setDelaySamples(dlySamples);
        }

        // ---- damping filters (cutoff follows the DAMPING knob) ----
        for (int i = 0; i < kNumDelays; ++i)
            filters[i].setCutoff(dampFreq, sr);

        // ---- modulation: one shared offset for all four lines ----
        modLFO.process(sr);
        float modOffset = modLFO.getValue() * modDepthSamples;
        for (int i = 0; i < kNumDelays; ++i)
            delays[i].setModOffset(modOffset);

        // ---- input (a mono source on the left feeds both channels) ----
        float leftIn = inputs[INPUT_L].getVoltageSum();
        float rightIn = inputs[INPUT_R].isConnected() ? inputs[INPUT_R].getVoltageSum() : leftIn;
        float dryL = leftIn;
        float dryR = rightIn;
        // The tank itself is mono; width comes from the output tap pattern.
        float monoIn = (leftIn + rightIn) * 0.5f;

        // ---- pre-delay ----
        float pred = monoIn;
        if (preSamples > 1)
            pred = pre.process(monoIn);

        // ---- input all-pass chain (diffusion) ----
        float dense = pred * inGain;
        for (int i = 0; i < 4; ++i)
            dense = inputAPs[i].process(dense);

        // ---- FDN: read -> lowpass -> matrix -> write ----
        for (int i = 0; i < kNumDelays; ++i)
            outBuf[i] = filters[i].process(delays[i].read());

        float fb[kNumDelays];
        std::copy(outBuf, outBuf + kNumDelays, fb);
        applyFeedbackMatrix(fb);
        // Decay gain (per round trip) is what sets the tail length.
        for (int i = 0; i < kNumDelays; ++i)
            fb[i] *= effDecay;

        for (int i = 0; i < kNumDelays; ++i)
            delays[i].write(dense * inGains_[i] + fb[i]);

        // ---- wet mix: different tap signs/weights for L and R ----
        float wetL = 0, wetR = 0;
        for (int i = 0; i < kNumDelays; ++i)
        {
            wetL += outBuf[i] * outGainsL_[i];
            wetR += outBuf[i] * outGainsR_[i];
        }
        // Output voicing: high shelf then low shelf, then the wet trim.
        wetL = lowShelf.process(highShelf.process(wetL));
        wetR = lowShelf.process(highShelf.process(wetR));
        wetL *= wetGain;
        wetR *= wetGain;

        // ---- dry/wet ----
        float outL = dryL * (1.0f - mix) + wetL * mix;
        float outR = dryR * (1.0f - mix) + wetR * mix;

        // ---- soft fade-in (0.5 seconds) ----
        if (fadeCounter < fadeSamples)
        {
            // Quadratic ramp: quieter at the start, so the onset is inaudible.
            float fade = float(fadeCounter) / float(fadeSamples);
            fade = fade * fade; // quadratic
            outL *= fade;
            outR *= fade;
            fadeCounter++;
        }

        // ---- safe clamping (+-12V) ----
        if (outL < -12.0f)
            outL = -12.0f;
        else if (outL > 12.0f)
            outL = 12.0f;
        if (outR < -12.0f)
            outR = -12.0f;
        else if (outR > 12.0f)
            outR = 12.0f;

        outputs[OUTPUT_L].setVoltage(outL);
        outputs[OUTPUT_R].setVoltage(outR);

        // ---- display data: peak meters with a slow decay ----
        float md = 1.0f - 6.0f * args.sampleTime;
        meterIn = std::max(std::fabs(monoIn), meterIn * md);
        meterOutL = std::max(std::fabs(outL), meterOutL * md);
        meterOutR = std::max(std::fabs(outR), meterOutR * md);
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
    NVGcolor colDim() { return nvgRGB(0x8a, 0x95, 0xa4); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    // NVGcolor colViolet() { return nvgRGB(0xb0, 0x5a, 0xff); }

    // Loads the shared panel font. Returns false when there is no window yet
    // (e.g. while the module browser renders an offscreen preview).
    bool setFont(NVGcontext *vg)
    {
        if (!APP || !APP->window)
            return false;
        std::shared_ptr<window::Font> font =
            APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (!font)
            return false;
        nvgFontFaceId(vg, font->handle);
        return true;
    }

    // Text helper: no-ops if the font is unavailable, otherwise draws with the
    // panel's standard size/letter-spacing handling.
    void drawText(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
                  int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f)
    {
        if (!setFont(vg))
            return;
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Violet LED used for the FREEZE indicator (Rack ships no violet light).
    template <typename TBase = GrayModuleLightWidget>
    struct TAMVioletLight : TBase
    {
        TAMVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
    };
    using AMVioletLight = TAMVioletLight<>;

    // Panel labels, in the same order as the widgets are added below.
    const char *AM_KNOB_NAMES[8] = {"PRE-DELAY", "SPACE", "DECAY", "DAMPING",
                                    "WET GAIN", "MIX", "MOD", "FREEZE"};
    const char *AM_IO_NAMES[7] = {"L IN", "R IN", "DECAY CV", "MIX CV", "FREEZE", "L OUT", "R OUT"};

    // ------------------------------------------------------------
    // Panel background (static artwork only)
    //
    // Everything here is drawn once per frame on layer 0: gradient, title,
    // display bezel, the eight control plates and the seven jack plates.
    // No module state is read, so this widget is identical for every instance.
    // ------------------------------------------------------------
    struct AMPanelBackground : TransparentWidget
    {
        AMPanelBackground() { box.size = Vec(AM_PANEL_W, AM_PANEL_H); }

        // Rounded plate behind one control. `tint` marks output jacks.
        void plate(NVGcontext *vg, float cx, float top, float w, float h, bool tint = false)
        {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - w / 2.f, top, w, h, 5.f);
            nvgFillColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 22) : nvgRGBA(0, 0, 0, 55));
            nvgFill(vg);
            nvgStrokeColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 60) : nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Section title with hairlines that stop a measured distance from the text
        void section(NVGcontext *vg, float y, const char *txt)
        {
            const float x1 = 14.f, x2 = AM_PANEL_W - 14.f, cx = AM_PANEL_W / 2.f;
            if (!setFont(vg))
                return;
            nvgFontSize(vg, 8.f);
            nvgTextLetterSpacing(vg, 2.f);
            // Measure the text so the lines never touch the letters.
            float b[4];
            nvgTextBounds(vg, 0.f, 0.f, txt, NULL, b);
            float gap = (b[2] - b[0]) / 2.f + 6.f;

            drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x1, y);
            nvgLineTo(vg, cx - gap, y);
            nvgMoveTo(vg, cx + gap, y);
            nvgLineTo(vg, x2, y);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = AM_PANEL_W, H = AM_PANEL_H;

            // Base gradient
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            // Title + maker
            drawText(vg, W / 2.f, AM_TITLE_Y, "AMBIENCE", 12.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);
            drawText(vg, W / 2.f, AM_MAKER_Y, "AGMM", 9.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

            // Display bezel (the display widget itself sits 4 px inside it)
            nvgBeginPath(vg);
            nvgRoundedRect(vg, AM_DISP_X - 4.f, AM_DISP_Y - 4.f,
                           AM_DISP_W + 8.f, AM_DISP_H + 8.f, 5.f);
            nvgFillColor(vg, nvgRGB(0x0b, 0x0c, 0x0e));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // ---- REVERB: 2 rows x 4 cells ----
            section(vg, AM_SEC1_Y, "REVERB");
            for (int i = 0; i < 8; ++i)
            {
                float cx = AM_COL_CX[i % 4];
                float top = AM_ROW_TOP[i / 4];
                plate(vg, cx, top, AM_CELL_W, AM_CELL_H);
                // Labels sit just under the plate top; the widget goes below.
                drawText(vg, cx, top + AM_CELL_LABEL, AM_KNOB_NAMES[i], 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            }

            // ---- I / O (inputs left, outputs right, outputs tinted) ----
            section(vg, AM_SEC2_Y, "I / O");
            for (int i = 0; i < 7; ++i)
            {
                float cx = amIoX(i);
                bool isOutput = (i >= 5);
                plate(vg, cx, AM_IO_PLATE_TOP, AM_IO_PLATE_W, AM_IO_PLATE_H, isOutput);
                drawText(vg, cx, AM_IO_LABEL_Y, AM_IO_NAMES[i], 7.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            }

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: decay curve | damping response | level meters.
    // Titles, axis labels and readouts each have their own row; no text
    // is ever drawn over a plot.
    //
    // Layer 0 draws the screen, the two plot frames, their grids and every
    // static label. Layer 1 draws the live curves and meters.
    // ------------------------------------------------------------
    struct AMDisplayWidget : TransparentWidget
    {
        Ambience *module = nullptr;

        AMDisplayWidget(Ambience *m) : module(m)
        {
            box.pos = Vec(AM_DISP_X, AM_DISP_Y);
            box.size = Vec(AM_DISP_W, AM_DISP_H);
        }

        // Recessed frame around one plot (slightly outside the plot rect).
        static void plotFrame(NVGcontext *vg, float x, float y, float w, float h)
        {
            nvgBeginPath(vg);
            nvgRect(vg, x - 0.5f, y - 0.5f, w + 1.f, h + 1.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Static layer: screen, plot frames, grids, titles, axis labels
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = AM_DISP_W, H = AM_DISP_H;

            // Screen background: near-black so the cyan curves read clearly.
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
            nvgFill(vg);

            const float py = AM_D_PLOT_Y, ph = AM_D_PLOT_H;
            plotFrame(vg, AM_D_DECAY_X, py, AM_D_DECAY_W, ph);
            plotFrame(vg, AM_D_DAMP_X, py, AM_D_DAMP_W, ph);

            // Grids: decay every 2 s over 8 s; damping at 100 Hz, 1 kHz, 10 kHz
            nvgBeginPath(vg);
            for (int t = 1; t < 4; ++t)
            {
                float x = AM_D_DECAY_X + AM_D_DECAY_W * t / 4.f + 0.5f;
                nvgMoveTo(vg, x, py);
                nvgLineTo(vg, x, py + ph);
            }
            const float decades[3] = {100.f, 1000.f, 10000.f};
            for (int d = 0; d < 3; ++d)
            {
                float x = AM_D_DAMP_X + AM_D_DAMP_W * (std::log10(decades[d] / 20.f) / 3.f) + 0.5f;
                nvgMoveTo(vg, x, py);
                nvgLineTo(vg, x, py + ph);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 18));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Titles (above the plots)
            drawText(vg, AM_D_DECAY_X, AM_D_TITLE_Y, "DECAY", 7.f, colDim(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            drawText(vg, AM_D_DAMP_X, AM_D_TITLE_Y, "DAMPING", 7.f, colDim(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            drawText(vg, AM_D_METER_X + 10.f, AM_D_TITLE_Y, "LVL", 7.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

            // Axis labels (below the plots)
            drawText(vg, AM_D_DECAY_X, AM_D_AXIS_Y, "0", 7.f, colDim(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.f);
            drawText(vg, AM_D_DECAY_X + AM_D_DECAY_W, AM_D_AXIS_Y, "8 S", 7.f, colDim(),
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.f);
            drawText(vg, AM_D_DAMP_X, AM_D_AXIS_Y, "20", 7.f, colDim(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.f);
            drawText(vg, AM_D_DAMP_X + AM_D_DAMP_W, AM_D_AXIS_Y, "20K", 7.f, colDim(),
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.f);
            // Meter channel labels: I(nput), L, R.
            const char *ml[3] = {"I", "L", "R"};
            for (int k = 0; k < 3; ++k)
                drawText(vg, AM_D_METER_X + 2.f + k * 8.f, AM_D_AXIS_Y, ml[k], 7.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

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

            // ---- gather state (live, or a sample scene in the module browser) ----
            float mIn, mL, mR, pre, space, decay, damp, mix, frz;
            if (module)
            {
                mIn = module->meterIn;
                mL = module->meterOutL;
                mR = module->meterOutR;
                pre = module->params[Ambience::PRE_DELAY_PARAM].getValue();
                space = module->params[Ambience::SPACE_PARAM].getValue();
                decay = module->dispDecay;
                damp = module->params[Ambience::DAMPING_PARAM].getValue();
                mix = module->dispMix;
                frz = module->freezeAmt;
            }
            else
            {
                // Fixed demo scene so the browser preview looks alive.
                mIn = 4.f;
                mL = 5.f;
                mR = 4.f;
                pre = 0.02f;
                space = 1.f;
                decay = 0.92f;
                damp = 1800.f;
                mix = 0.5f;
                frz = 0.f;
            }

            // RT60 estimate: each line loses (decay) per round trip of its own
            // length. Averaging 1/length over the four lines gives an effective
            // loop count per second; combining with the per-loop dB loss yields
            // the time for -60 dB. It is an estimate, but it tracks the knob.
            float meanInv = 0.f;
            for (int i = 0; i < 4; ++i)
                meanInv += 1.f / AM_BASE[i];
            meanInv /= 4.f;
            float dbPerLoop = -20.f * std::log10(std::max(decay, 0.001f));
            float rt60 = (dbPerLoop > 1e-6f) ? 60.f * space / (dbPerLoop * meanInv) : 1e6f;
            bool frozen = frz > 0.5f;
            float preS = pre * space;

            const float py = AM_D_PLOT_Y, ph = AM_D_PLOT_H;

            // ---- decay curve (0..8 s) ----
            // Amplitude model: silence during the pre-delay, then an exponential
            // -60 dB decay over rt60. Freeze draws a flat line at 1.
            {
                const float fx = AM_D_DECAY_X, fw = AM_D_DECAY_W;
                auto amp = [&](float t) -> float
                {
                    if (t < preS)
                        return 0.f;
                    return frozen ? 1.f : std::exp(-6.9078f * (t - preS) / rt60);
                };
                // Clip to the plot rect so the curve never spills over labels.
                nvgSave(vg);
                nvgScissor(vg, fx, py, fw, ph);

                // Filled area under the curve.
                nvgBeginPath(vg);
                nvgMoveTo(vg, fx, py + ph - 1.f);
                for (int i = 0; i <= (int)fw; ++i)
                    nvgLineTo(vg, fx + i, py + ph - 1.f - amp(8.f * i / fw) * (ph - 3.f));
                nvgLineTo(vg, fx + fw, py + ph - 1.f);
                nvgClosePath(vg);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                nvgFill(vg);

                // Crisp curve on top of the fill.
                nvgBeginPath(vg);
                for (int i = 0; i <= (int)fw; ++i)
                {
                    float x = fx + i, y = py + ph - 1.f - amp(8.f * i / fw) * (ph - 3.f);
                    if (i == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, nvgRGB(0x8a, 0xee, 0xff));
                nvgStrokeWidth(vg, 1.3f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                // RT60 marker: violet dashed vertical line at -60 dB time.
                if (!frozen && rt60 + preS < 8.f)
                {
                    float mx = fx + fw * (rt60 + preS) / 8.f;
                    nvgBeginPath(vg);
                    for (float y = py + 1.f; y < py + ph - 1.f; y += 4.f)
                    {
                        nvgMoveTo(vg, mx + 0.5f, y);
                        nvgLineTo(vg, mx + 0.5f, y + 2.f);
                    }
                    nvgStrokeColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 230));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                nvgResetScissor(vg);
                nvgRestore(vg);
            }

            // ---- damping response (log 20 Hz..20 kHz, 0..-36 dB) ----
            // 4-pole (24 dB/oct) roll-off model, normalised to the plot height.
            {
                const float fx = AM_D_DAMP_X, fw = AM_D_DAMP_W;
                auto yAt = [&](int i) -> float
                {
                    float f = 20.f * std::pow(1000.f, (float)i / fw);
                    float db = -10.f * std::log10(1.f + std::pow(f / damp, 4.f));
                    float t = std::min(std::max(-db / 36.f, 0.f), 1.f);
                    return py + 2.f + t * (ph - 4.f);
                };

                // Filled area.
                nvgBeginPath(vg);
                nvgMoveTo(vg, fx, py + ph - 1.f);
                for (int i = 0; i <= (int)fw; ++i)
                    nvgLineTo(vg, fx + i, yAt(i));
                nvgLineTo(vg, fx + fw, py + ph - 1.f);
                nvgClosePath(vg);
                nvgFillColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 40));
                nvgFill(vg);

                // Curve.
                nvgBeginPath(vg);
                for (int i = 0; i <= (int)fw; ++i)
                {
                    if (i == 0)
                        nvgMoveTo(vg, fx + i, yAt(i));
                    else
                        nvgLineTo(vg, fx + i, yAt(i));
                }
                nvgStrokeColor(vg, nvgRGB(0xd8, 0xb0, 0xff));
                nvgStrokeWidth(vg, 1.2f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                // Cutoff marker: cyan line at the damping frequency.
                float cxm = fx + fw * (std::log10(damp / 20.f) / 3.f);
                nvgBeginPath(vg);
                nvgRect(vg, cxm, py, 1.f, ph);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 180));
                nvgFill(vg);
            }

            // ---- meters: input, L, R ----
            for (int k = 0; k < 3; ++k)
            {
                float v = k == 0 ? mIn : (k == 1 ? mL : mR);
                float lv = std::min(v / 10.f, 1.f);
                float x = AM_D_METER_X + k * 8.f;
                // Track.
                nvgBeginPath(vg);
                nvgRect(vg, x, py, 4.f, ph);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
                // Bar, bottom-up, cyan to violet.
                if (lv > 0.01f)
                {
                    float h = lv * ph;
                    nvgBeginPath(vg);
                    nvgRect(vg, x, py + ph - h, 4.f, h);
                    nvgFillPaint(vg, nvgLinearGradient(vg, 0, py + ph, 0, py,
                                                       nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                    nvgFill(vg);
                }
            }

            // ---- axis-row values (under the plots, centred) ----
            // These replace the plain axis labels with the current values, so
            // the row always shows something useful.
            char buf[64];
            NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa4, 255);
            snprintf(buf, sizeof(buf), "PRE %.0fMS", pre * 1000.f);
            drawText(vg, AM_D_DECAY_X + AM_D_DECAY_W / 2.f, AM_D_AXIS_Y, buf, 7.f, cd,
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            snprintf(buf, sizeof(buf), "FC %.1fK", damp / 1000.f);
            drawText(vg, AM_D_DAMP_X + AM_D_DAMP_W / 2.f, AM_D_AXIS_Y, buf, 7.f, cd,
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

            // ---- readout row: 4 evenly spaced values ----
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 230);
            if (frozen)
                snprintf(buf, sizeof(buf), "RT60 INF");
            else if (rt60 > 99.f)
                snprintf(buf, sizeof(buf), "RT60 >99S");
            else
                snprintf(buf, sizeof(buf), "RT60 %.1fS", rt60);
            drawText(vg, 40.f, AM_D_READ_Y, buf, 7.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "SPACE %.2fX", space);
            drawText(vg, 120.f, AM_D_READ_Y, buf, 7.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "DECAY %2.0f%%", decay * 100.f);
            drawText(vg, 200.f, AM_D_READ_Y, buf, 7.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "MIX %3.0f%%", mix * 100.f);
            drawText(vg, 280.f, AM_D_READ_Y, buf, 7.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// -----------------------------------------------------------------------------
// Widget
//
// Widget order matters: the panel background is added first (bottom), the
// display next, then the knobs/jacks on top. Positions come from the layout
// constants at the top of the file, so panel artwork and widgets never drift.
// -----------------------------------------------------------------------------
struct AmbienceWidget : ModuleWidget
{
    AmbienceWidget(Ambience *module)
    {
        setModule(module);
        box.size = Vec(AM_PANEL_W, AM_PANEL_H);

        addChild(new AMPanelBackground());

        // Four rack screws, one per corner.
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new AMDisplayWidget(module));

        // ---- knobs: 4 columns x 2 rows (the 8th cell is the freeze switch) ----
        // Order matches AM_KNOB_NAMES: row 1 = pre-delay, space, decay, damping;
        // row 2 = wet gain, mix, mod, freeze.
        const int knobIds[7] = {Ambience::PRE_DELAY_PARAM, Ambience::SPACE_PARAM, Ambience::DECAY_PARAM,
                                Ambience::DAMPING_PARAM, Ambience::WET_GAIN_PARAM, Ambience::MIX_PARAM,
                                Ambience::MOD_DEPTH_PARAM};
        for (int i = 0; i < 7; ++i)
        {
            float cx = AM_COL_CX[i % 4];
            float cy = AM_ROW_TOP[i / 4] + AM_CELL_KNOB;
            addParam(createParamCentered<RoundBlackKnob>(Vec(cx, cy), module, knobIds[i]));
        }

        // Freeze: switch with its LED beside it, both inside the cell
        {
            float cx = AM_COL_CX[3];
            float cy = AM_ROW_TOP[1] + AM_CELL_KNOB;
            addParam(createParamCentered<CKSS>(Vec(cx - 6.f, cy), module, Ambience::FREEZE_PARAM));
            addChild(createLightCentered<SmallLight<AMVioletLight>>(Vec(cx + 14.f, cy), module, Ambience::FREEZE_LIGHT));
        }

        // ---- I/O: L IN, R IN, DECAY CV, MIX CV, FREEZE | L OUT, R OUT ----
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amIoX(0), AM_IO_JACK_Y), module, Ambience::INPUT_L));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amIoX(1), AM_IO_JACK_Y), module, Ambience::INPUT_R));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amIoX(2), AM_IO_JACK_Y), module, Ambience::DECAY_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amIoX(3), AM_IO_JACK_Y), module, Ambience::MIX_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amIoX(4), AM_IO_JACK_Y), module, Ambience::FREEZE_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(amIoX(5), AM_IO_JACK_Y), module, Ambience::OUTPUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(amIoX(6), AM_IO_JACK_Y), module, Ambience::OUTPUT_R));
    }
};

Model *modelAmbience = createModel<Ambience, AmbienceWidget>("Ambience");