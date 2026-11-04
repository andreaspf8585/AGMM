#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
// ANDREAS GOGOS
// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float AM_PANEL_W = RACK_GRID_WIDTH * 24; // 360
static const float AM_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float AM_DISP_X = 20.f;
static const float AM_DISP_Y = 46.f;
static const float AM_DISP_W = 320.f;
static const float AM_DISP_H = 114.f;

static const float AM_KCOL[4] = {45.f, 135.f, 225.f, 315.f};
static const float AM_KROW[2] = {198.f, 258.f};
static const float AM_JACK_Y = 340.f;
static float amJackX(int i) { return 36.f + 48.f * i; }

// Base delay lengths of the FDN (used by the display for the RT60 estimate)
static const float AM_BASE[4] = {0.0317f, 0.0383f, 0.0429f, 0.0451f};

// -----------------------------------------------------------------------------
// 2nd order Butterworth lowpass (12 dB/oct)
// -----------------------------------------------------------------------------
class AmbLowpass2
{
public:
    AmbLowpass2() : b0_(0), b1_(0), b2_(0), a1_(0), a2_(0), x1_(0), x2_(0), y1_(0), y2_(0) {}
    void setCutoff(float freq, float sr)
    {
        if (freq <= 0)
            freq = 10;
        if (freq >= sr * 0.49f)
            freq = sr * 0.49f;
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
// -----------------------------------------------------------------------------
class AmbDelayLine
{
public:
    AmbDelayLine() : buf_(), writePos_(0), delaySamples_(0), maxSamples_(0), modOffset_(0) {}
    void setMaxDelay(float maxSec, float sr)
    {
        maxSamples_ = std::max(1, int(maxSec * sr));
        buf_.resize(maxSamples_ + 2, 0.0f);
        writePos_ = 0;
        delaySamples_ = maxSamples_;
    }
    void setDelaySamples(int samples)
    {
        delaySamples_ = std::max(1, std::min(samples, maxSamples_));
    }
    void setModOffset(float offset) { modOffset_ = offset; }
    float read() const
    {
        float readPos = float(writePos_) - float(delaySamples_) + modOffset_;
        if (readPos < 0)
            readPos += buf_.size();
        if (readPos >= buf_.size())
            readPos -= buf_.size();
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
// -----------------------------------------------------------------------------
class AmbLowShelf
{
public:
    AmbLowShelf() : b0_(1), b1_(0), b2_(0), a1_(0), a2_(0), x1_(0), x2_(0), y1_(0), y2_(0) {}
    void setGain(float gainDb, float freq, float sr)
    {
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

class AmbHighShelf
{
public:
    AmbHighShelf() : b0_(1), b1_(0), b2_(0), a1_(0), a2_(0), x1_(0), x2_(0), y1_(0), y2_(0) {}
    void setGain(float gainDb, float freq, float sr)
    {
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
        MOD_DEPTH_PARAM, // new
        FREEZE_PARAM,    // new
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_L,
        INPUT_R,
        DECAY_CV_INPUT, // new
        MIX_CV_INPUT,   // new
        FREEZE_INPUT,   // new
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
    const float baseDelays_[kNumDelays] = {0.0317f, 0.0383f, 0.0429f, 0.0451f};
    const float inGains_[kNumDelays] = {0.5f, 0.5f, 0.5f, 0.5f};
    const float outGainsL_[kNumDelays] = {0.6f, -0.4f, 0.5f, -0.3f};
    const float outGainsR_[kNumDelays] = {0.5f, 0.5f, -0.4f, -0.4f};

    float sr;
    bool srChanged;
    float maxSpace = 2.5f;
    float maxPreDelay = 0.2f;

    AmbPreDelay pre;
    AmbDelayLine delays[kNumDelays];
    AmbLowpass2 filters[kNumDelays];
    AmbModLFO modLFO;
    AmbAllPass inputAPs[4];
    float apDelays[4] = {0.0021f, 0.0037f, 0.0054f, 0.0072f};
    AmbLowShelf lowShelf;
    AmbHighShelf highShelf;

    float outBuf[kNumDelays];

    // Fade-in
    int fadeCounter;
    int fadeSamples;

    // Freeze
    float freezeAmt = 0.f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float nodeLevel[kNumDelays] = {};
    float meterIn = 0.f, meterOutL = 0.f, meterOutR = 0.f;
    float dispDecay = 0.92f, dispMix = 0.5f;

    Ambience()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PRE_DELAY_PARAM, 0.0f, maxPreDelay, 0.02f, "Pre-delay", " ms", 0.f, 1000.f);
        configParam(SPACE_PARAM, 0.5f, maxSpace, 1.0f, "Space", "x");
        configParam(DECAY_PARAM, 0.5f, 0.999f, 0.92f, "Decay", "%", 0.f, 100.f);
        configParam(DAMPING_PARAM, 200.0f, 8000.0f, 1800.0f, "Damping", " Hz");
        configParam(WET_GAIN_PARAM, 0.0f, 2.0f, 1.0f, "Wet gain", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.0f, 1.0f, 0.5f, "Dry/wet", "%", 0.f, 100.f);
        configParam(MOD_DEPTH_PARAM, 0.0f, 6.0f, 2.0f, "Modulation depth", " samples");
        configSwitch(FREEZE_PARAM, 0.f, 1.f, 0.f, "Freeze", {"Off", "On"});

        configInput(INPUT_L, "Left (mono if the right input is empty)");
        configInput(INPUT_R, "Right");
        configInput(DECAY_CV_INPUT, "Decay CV (10 V adds 50%)");
        configInput(MIX_CV_INPUT, "Dry/wet CV (10 V adds 100%)");
        configInput(FREEZE_INPUT, "Freeze gate");
        configOutput(OUTPUT_L, "Left");
        configOutput(OUTPUT_R, "Right");
        configLight(FREEZE_LIGHT, "Freeze");

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
        float sum = 0;
        for (int i = 0; i < kNumDelays; ++i)
            sum += x[i];
        float factor = (2.0f / kNumDelays) * sum;
        for (int i = 0; i < kNumDelays; ++i)
            x[i] -= factor;
    }

    void process(const ProcessArgs &args) override
    {
        if (srChanged || args.sampleRate != sr)
        {
            sr = args.sampleRate;
            srChanged = false;
            fadeSamples = int(0.5f * sr); // 0.5 second fade-in
            fadeCounter = 0;

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
            lowShelf.setGain(-3.0f, 150.0f, sr);
            highShelf.setGain(-4.0f, 5000.0f, sr);
        }

        // Parameters
        float preTime = params[PRE_DELAY_PARAM].getValue();
        float space = params[SPACE_PARAM].getValue();
        float decay = params[DECAY_PARAM].getValue();
        decay = clamp(decay + inputs[DECAY_CV_INPUT].getVoltage() * 0.05f, 0.5f, 0.999f);
        float dampFreq = params[DAMPING_PARAM].getValue();
        float wetGain = params[WET_GAIN_PARAM].getValue();
        float mix = params[MIX_PARAM].getValue();
        mix = clamp(mix + inputs[MIX_CV_INPUT].getVoltage() * 0.1f, 0.0f, 1.0f);
        float modDepthSamples = params[MOD_DEPTH_PARAM].getValue();
        dispDecay = decay;
        dispMix = mix;

        // Freeze: holds the tail (decay -> ~1) and closes the input
        bool freezeOn = params[FREEZE_PARAM].getValue() > 0.5f || inputs[FREEZE_INPUT].getVoltage() > 1.0f;
        freezeAmt += ((freezeOn ? 1.0f : 0.0f) - freezeAmt) * std::min(1.0f, 20.0f * args.sampleTime);
        lights[FREEZE_LIGHT].setBrightness(freezeAmt);
        float effDecay = decay + (0.9995f - decay) * freezeAmt;
        float inGain = 1.0f - freezeAmt;

        // Update delay lengths without clearing
        int preSamples = std::max(1, int(preTime * space * sr + 0.5f));
        pre.setDelaySamples(preSamples);
        for (int i = 0; i < kNumDelays; ++i)
        {
            int dlySamples = std::max(1, int(baseDelays_[i] * space * sr + 0.5f));
            delays[i].setDelaySamples(dlySamples);
        }

        // Damping filters
        for (int i = 0; i < kNumDelays; ++i)
            filters[i].setCutoff(dampFreq, sr);

        // Modulation
        modLFO.process(sr);
        float modOffset = modLFO.getValue() * modDepthSamples;
        for (int i = 0; i < kNumDelays; ++i)
            delays[i].setModOffset(modOffset);

        // Input (a mono source on the left feeds both channels)
        float leftIn = inputs[INPUT_L].getVoltageSum();
        float rightIn = inputs[INPUT_R].isConnected() ? inputs[INPUT_R].getVoltageSum() : leftIn;
        float dryL = leftIn;
        float dryR = rightIn;
        float monoIn = (leftIn + rightIn) * 0.5f;

        // Pre-delay
        float pred = monoIn;
        if (preSamples > 1)
            pred = pre.process(monoIn);

        // Input all-pass chain (density)
        float dense = pred * inGain;
        for (int i = 0; i < 4; ++i)
            dense = inputAPs[i].process(dense);

        // FDN: read -> lowpass -> matrix -> write
        for (int i = 0; i < kNumDelays; ++i)
            outBuf[i] = filters[i].process(delays[i].read());

        float fb[kNumDelays];
        std::copy(outBuf, outBuf + kNumDelays, fb);
        applyFeedbackMatrix(fb);
        for (int i = 0; i < kNumDelays; ++i)
            fb[i] *= effDecay;

        for (int i = 0; i < kNumDelays; ++i)
            delays[i].write(dense * inGains_[i] + fb[i]);

        // Wet mix
        float wetL = 0, wetR = 0;
        for (int i = 0; i < kNumDelays; ++i)
        {
            wetL += outBuf[i] * outGainsL_[i];
            wetR += outBuf[i] * outGainsR_[i];
        }
        wetL = lowShelf.process(highShelf.process(wetL));
        wetR = lowShelf.process(highShelf.process(wetR));
        wetL *= wetGain;
        wetR *= wetGain;

        // Dry/wet
        float outL = dryL * (1.0f - mix) + wetL * mix;
        float outR = dryR * (1.0f - mix) + wetR * mix;

        // Soft fade-in (0.5 seconds)
        if (fadeCounter < fadeSamples)
        {
            float fade = float(fadeCounter) / float(fadeSamples);
            fade = fade * fade; // quadratic
            outL *= fade;
            outR *= fade;
            fadeCounter++;
        }

        // Safe clamping (+-12V)
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

        // Display data
        float md = 1.0f - 6.0f * args.sampleTime;
        for (int i = 0; i < kNumDelays; ++i)
            nodeLevel[i] = std::max(std::fabs(outBuf[i]), nodeLevel[i] * md);
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
    NVGcolor colDim() { return nvgRGB(0x7a, 0x85, 0x94); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    NVGcolor colViolet() { return nvgRGB(0xb0, 0x5a, 0xff); }

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

    template <typename TBase = GrayModuleLightWidget>
    struct TAMVioletLight : TBase
    {
        TAMVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
    };
    using AMVioletLight = TAMVioletLight<>;

    // ------------------------------------------------------------
    // AGMM logo
    // ------------------------------------------------------------
    void drawLogo(NVGcontext *vg, float cx, float cy)
    {
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
        nvgRect(vg, tx + s, ty, s, s);
        nvgRect(vg, tx, ty + s * 1.4f, s, s);
        nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
        nvgFillColor(vg, colAccent());
        nvgFill(vg);

        drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
    }

    // Shaded sphere for the FDN nodes
    void drawNode(NVGcontext *vg, float cx, float cy, float r, NVGcolor hi, NVGcolor base, NVGcolor edge)
    {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx - r * 0.35f, cy - r * 0.4f, r * 0.1f, r * 1.25f, hi, base));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r - 0.4f);
        nvgStrokeColor(vg, edge);
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgCircle(vg, cx - r * 0.38f, cy - r * 0.42f, r * 0.22f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 210));
        nvgFill(vg);
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct AMPanelBackground : TransparentWidget
    {
        AMPanelBackground() { box.size = Vec(AM_PANEL_W, AM_PANEL_H); }

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

        void topCell(NVGcontext *vg, float cx, float cy, const char *txt)
        {
            plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
            label(vg, cx, cy - 19.f, txt);
        }

        void section(NVGcontext *vg, float y, const char *txt)
        {
            float x1 = 14.f, x2 = AM_PANEL_W - 14.f, cx = AM_PANEL_W / 2.f;
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

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = AM_PANEL_W, H = AM_PANEL_H;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            nvgStrokeWidth(vg, 1.f);
            for (int y = 0; y < (int)H; y += 3)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, y + 0.5f);
                nvgLineTo(vg, W, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
                nvgStroke(vg);
            }

            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, W - 1.f, H - 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            drawLogo(vg, W / 2.f, 13.f);
            drawText(vg, W / 2.f, 31.f, "AMBIENCE", 13.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

            // Display bezel
            float bx = AM_DISP_X - 5.f, by = AM_DISP_Y - 5.f;
            float bw = AM_DISP_W + 10.f, bh = AM_DISP_H + 10.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                               nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
            nvgFill(vg);

            // ---- knobs ----
            const char *names[8] = {"PRE-DELAY", "SPACE", "DECAY", "DAMPING", "WET GAIN", "MIX", "MOD", "FREEZE"};
            for (int i = 0; i < 8; ++i)
            {
                float cx = AM_KCOL[i % 4], cy = AM_KROW[i / 4];
                if (i == 7)
                {
                    plate(vg, cx - 17.f, cy - 20.f, 34.f, 40.f);
                    label(vg, cx, cy + 28.f, names[i]);
                }
                else
                {
                    ring(vg, cx, cy, 22.f);
                    label(vg, cx, cy + 28.f, names[i]);
                }
            }

            // ---- jack row ----
            section(vg, 298.f, "I / O");
            const char *io[7] = {"L IN", "R IN", "DECAY CV", "FRZ GATE", "MIX CV", "OUT L", "OUT R"};
            for (int i = 0; i < 7; ++i)
                topCell(vg, amJackX(i), AM_JACK_Y, io[i]);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: FDN network, decay curve, damping response
    // ------------------------------------------------------------
    struct AMDisplayWidget : TransparentWidget
    {
        Ambience *module = nullptr;

        AMDisplayWidget(Ambience *m) : module(m)
        {
            box.pos = Vec(AM_DISP_X, AM_DISP_Y);
            box.size = Vec(AM_DISP_W, AM_DISP_H);
        }

        // Static background: screen and the two plot fields
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float W = AM_DISP_W, H = AM_DISP_H;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, W, H);
            nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 30.f, W * 0.7f,
                                               nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
            nvgFill(vg);

            // Decay field and damping field
            const float fx = 168.f, fw = 108.f;
            const float fy[2] = {20.f, 76.f};
            const float fh[2] = {42.f, 22.f};
            for (int k = 0; k < 2; ++k)
            {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, fx - 1.f, fy[k] - 1.f, fw + 2.f, fh[k] + 2.f, 2.f);
                nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
                nvgFill(vg);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // Decay grid: every 2 s over 8 s
            nvgBeginPath(vg);
            for (int t = 1; t < 4; ++t)
            {
                float x = fx + fw * t / 4.f + 0.5f;
                nvgMoveTo(vg, x, fy[0]);
                nvgLineTo(vg, x, fy[0] + fh[0]);
            }
            // Damping grid: decades (100, 1k, 10k)
            for (int d = 1; d <= 3; ++d)
            {
                float x = fx + fw * (std::log10(std::pow(10.f, (float)d) * 10.f / 20.f) / 3.f) + 0.5f;
                nvgMoveTo(vg, x, fy[1]);
                nvgLineTo(vg, x, fy[1] + fh[1]);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            drawText(vg, fx, fy[0] + fh[0] + 6.f, "0", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.f);
            drawText(vg, fx + fw, fy[0] + fh[0] + 6.f, "8S", 5.5f, colDim(), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.f);
            drawText(vg, fx + 2.f, fy[0] + 6.f, "DECAY", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            drawText(vg, fx + 2.f, fy[1] + 6.f, "DAMP", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

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
            const float W = AM_DISP_W, H = AM_DISP_H;

            // ---- gather state (live, or a sample scene in the module browser) ----
            float lvl[4], mIn, mL, mR, pre, space, decay, damp, mix, wet, frz;
            if (module)
            {
                for (int i = 0; i < 4; ++i)
                    lvl[i] = module->nodeLevel[i];
                mIn = module->meterIn;
                mL = module->meterOutL;
                mR = module->meterOutR;
                pre = module->params[Ambience::PRE_DELAY_PARAM].getValue();
                space = module->params[Ambience::SPACE_PARAM].getValue();
                decay = module->dispDecay;
                damp = module->params[Ambience::DAMPING_PARAM].getValue();
                mix = module->dispMix;
                wet = module->params[Ambience::WET_GAIN_PARAM].getValue();
                frz = module->freezeAmt;
            }
            else
            {
                lvl[0] = 1.8f;
                lvl[1] = 1.1f;
                lvl[2] = 2.2f;
                lvl[3] = 0.8f;
                mIn = 2.f;
                mL = 2.8f;
                mR = 2.3f;
                pre = 0.02f;
                space = 1.f;
                decay = 0.92f;
                damp = 1800.f;
                mix = 0.5f;
                wet = 1.f;
                frz = 0.f;
            }

            // RT60 estimate: each line loses (decay) per round trip of its own length
            float meanInv = 0.f;
            for (int i = 0; i < 4; ++i)
                meanInv += 1.f / AM_BASE[i];
            meanInv /= 4.f;
            float dbPerLoop = -20.f * std::log10(std::max(decay, 0.001f));
            float rt60 = (dbPerLoop > 1e-6f) ? 60.f * space / (dbPerLoop * meanInv) : 1e6f;
            bool frozen = frz > 0.5f;

            // ---- FDN network ----
            const float cx = 78.f, cy = 58.f;
            const float nx[4] = {cx - 27.f, cx + 27.f, cx + 27.f, cx - 27.f};
            const float ny[4] = {cy - 24.f, cy - 24.f, cy + 24.f, cy + 24.f};
            float act[4], rad[4];
            for (int i = 0; i < 4; ++i)
            {
                act[i] = std::min(lvl[i] / 2.5f, 1.f);
                rad[i] = 3.5f + 3.5f * (AM_BASE[i] * space) / (0.0451f * 2.5f) * 2.f;
            }

            // Edges (the feedback matrix connects every node to every other)
            for (int i = 0; i < 4; ++i)
            {
                for (int j = i + 1; j < 4; ++j)
                {
                    float a = 0.07f + 0.55f * 0.5f * (act[i] + act[j]);
                    for (int pass = 0; pass < 2; ++pass)
                    {
                        nvgBeginPath(vg);
                        nvgMoveTo(vg, nx[i], ny[i]);
                        nvgLineTo(vg, nx[j], ny[j]);
                        nvgStrokeColor(vg, pass == 0 ? nvgRGBAf(0.24f, 0.88f, 1.f, a * 0.35f)
                                                     : nvgRGBAf(0.54f, 0.93f, 1.f, a));
                        nvgStrokeWidth(vg, pass == 0 ? 3.5f : 1.f);
                        nvgStroke(vg);
                    }
                }
            }

            // Input and outputs
            float inA = std::min(mIn / 5.f, 1.f);
            float lA = std::min(mL / 5.f, 1.f), rA = std::min(mR / 5.f, 1.f);
            const float ix = 12.f, lx = 144.f;
            auto link = [&](float x0, float y0, float x1, float y1, float a, NVGcolor c)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, x0, y0);
                nvgLineTo(vg, x1, y1);
                nvgStrokeColor(vg, withAlpha(c, 0.12f + 0.6f * a));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            };
            link(ix, cy, nx[0], ny[0], inA, colAccent());
            link(ix, cy, nx[3], ny[3], inA, colAccent());
            link(nx[1], ny[1], lx, cy - 14.f, lA, colViolet());
            link(nx[2], ny[2], lx, cy + 14.f, rA, colViolet());

            auto dot = [&](float x, float y, float a, NVGcolor c)
            {
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, 5.f + 3.f * a);
                nvgFillColor(vg, withAlpha(c, 0.15f + 0.25f * a));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, 2.6f);
                nvgFillColor(vg, withAlpha(c, 0.6f + 0.4f * a));
                nvgFill(vg);
            };
            dot(ix, cy, inA, colAccent());
            dot(lx, cy - 14.f, lA, colViolet());
            dot(lx, cy + 14.f, rA, colViolet());
            drawText(vg, ix, cy + 10.f, "IN", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            drawText(vg, lx + 9.f, cy - 14.f, "L", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.5f);
            drawText(vg, lx + 9.f, cy + 14.f, "R", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.5f);

            // Nodes
            for (int i = 0; i < 4; ++i)
            {
                float hue = 0.5f + 0.12f * (float)i;
                NVGcolor hc = nvgHSL(hue, 0.9f, 0.6f);
                nvgBeginPath(vg);
                nvgCircle(vg, nx[i], ny[i], rad[i] * 2.6f);
                nvgFillPaint(vg, nvgRadialGradient(vg, nx[i], ny[i], rad[i] * 0.6f, rad[i] * 2.6f,
                                                   withAlpha(hc, 0.08f + 0.35f * act[i]), withAlpha(hc, 0.f)));
                nvgFill(vg);
                drawNode(vg, nx[i], ny[i], rad[i], nvgHSL(hue, 0.9f, 0.86f), nvgHSL(hue, 0.8f, 0.42f), nvgHSL(hue, 0.8f, 0.16f));
            }

            if (frozen)
            {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, 8.f, 16.f, 36.f, 11.f, 3.f);
                nvgFillColor(vg, withAlpha(colViolet(), 0.25f));
                nvgFill(vg);
                nvgStrokeColor(vg, withAlpha(colViolet(), 0.9f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                drawText(vg, 26.f, 21.5f, "FREEZE", 6.f, colViolet(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
            }

            // ---- decay curve (0..8 s) ----
            {
                const float fx = 168.f, fy = 20.f, fw = 108.f, fh = 42.f;
                float preS = pre * space;
                nvgBeginPath(vg);
                nvgMoveTo(vg, fx, fy + fh - 1.f);
                for (int i = 0; i <= (int)fw; ++i)
                {
                    float t = 8.f * i / fw;
                    float a;
                    if (frozen)
                        a = (t < preS) ? 0.f : 1.f;
                    else
                        a = (t < preS) ? 0.f : std::exp(-6.9078f * (t - preS) / rt60);
                    nvgLineTo(vg, fx + i, fy + fh - 1.f - a * (fh - 2.f));
                }
                nvgLineTo(vg, fx + fw, fy + fh - 1.f);
                nvgClosePath(vg);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, fy, 0, fy + fh,
                                                   nvgRGBA(0x3d, 0xe0, 0xff, 70), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
                nvgFill(vg);
                for (int pass = 0; pass < 2; ++pass)
                {
                    nvgBeginPath(vg);
                    for (int i = 0; i <= (int)fw; ++i)
                    {
                        float t = 8.f * i / fw;
                        float a;
                        if (frozen)
                            a = (t < preS) ? 0.f : 1.f;
                        else
                            a = (t < preS) ? 0.f : std::exp(-6.9078f * (t - preS) / rt60);
                        float x = fx + i, y = fy + fh - 1.f - a * (fh - 2.f);
                        if (i == 0)
                            nvgMoveTo(vg, x, y);
                        else
                            nvgLineTo(vg, x, y);
                    }
                    nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 50) : nvgRGB(0x8a, 0xee, 0xff));
                    nvgStrokeWidth(vg, pass == 0 ? 3.5f : 1.3f);
                    nvgLineJoin(vg, NVG_ROUND);
                    nvgStroke(vg);
                }
                // RT60 marker
                if (!frozen && rt60 + preS < 8.f)
                {
                    float mx = fx + fw * (rt60 + preS) / 8.f;
                    nvgBeginPath(vg);
                    for (float y = fy + 1.f; y < fy + fh - 1.f; y += 4.f)
                    {
                        nvgMoveTo(vg, mx + 0.5f, y);
                        nvgLineTo(vg, mx + 0.5f, y + 2.f);
                    }
                    nvgStrokeColor(vg, withAlpha(colViolet(), 0.9f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
            }

            // ---- damping response (log 20 Hz..20 kHz, 0..-36 dB) ----
            {
                const float fx = 168.f, fy = 76.f, fw = 108.f, fh = 22.f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, fx, fy + fh - 1.f);
                for (int i = 0; i <= (int)fw; ++i)
                {
                    float f = 20.f * std::pow(1000.f, (float)i / fw);
                    float db = -10.f * std::log10(1.f + std::pow(f / damp, 4.f));
                    float t = std::min(std::max(-db / 36.f, 0.f), 1.f);
                    nvgLineTo(vg, fx + i, fy + 1.f + t * (fh - 2.f));
                }
                nvgLineTo(vg, fx + fw, fy + fh - 1.f);
                nvgClosePath(vg);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, fy, 0, fy + fh,
                                                   nvgRGBA(0xb0, 0x5a, 0xff, 60), nvgRGBA(0xb0, 0x5a, 0xff, 0)));
                nvgFill(vg);
                for (int pass = 0; pass < 2; ++pass)
                {
                    nvgBeginPath(vg);
                    for (int i = 0; i <= (int)fw; ++i)
                    {
                        float f = 20.f * std::pow(1000.f, (float)i / fw);
                        float db = -10.f * std::log10(1.f + std::pow(f / damp, 4.f));
                        float t = std::min(std::max(-db / 36.f, 0.f), 1.f);
                        float x = fx + i, y = fy + 1.f + t * (fh - 2.f);
                        if (i == 0)
                            nvgMoveTo(vg, x, y);
                        else
                            nvgLineTo(vg, x, y);
                    }
                    nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0xb0, 0x5a, 0xff, 60) : nvgRGB(0xd8, 0xb0, 0xff));
                    nvgStrokeWidth(vg, pass == 0 ? 3.5f : 1.2f);
                    nvgLineJoin(vg, NVG_ROUND);
                    nvgStroke(vg);
                }
                // Cutoff marker
                float cxm = fx + fw * (std::log10(damp / 20.f) / 3.f);
                nvgBeginPath(vg);
                nvgRect(vg, cxm, fy, 1.f, fh);
                nvgFillColor(vg, withAlpha(colAccent(), 0.7f));
                nvgFill(vg);
            }

            // ---- meters: input, L, R ----
            for (int k = 0; k < 3; ++k)
            {
                float v = k == 0 ? mIn : (k == 1 ? mL : mR);
                float lv = std::min(v / 5.f, 1.f);
                float x = 288.f + k * 7.f;
                nvgBeginPath(vg);
                nvgRect(vg, x, 20.f, 5.f, 78.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
                nvgFill(vg);
                if (lv > 0.01f)
                {
                    float h = lv * 78.f;
                    nvgBeginPath(vg);
                    nvgRect(vg, x, 98.f - h, 5.f, h);
                    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 98.f, 0, 20.f,
                                                       nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                    nvgFill(vg);
                }
            }
            drawText(vg, 290.5f, 104.f, "I", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
            drawText(vg, 297.5f, 104.f, "L", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
            drawText(vg, 304.5f, 104.f, "R", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

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

            // ---- readouts ----
            char buf[64];
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
            NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 230);
            snprintf(buf, sizeof(buf), "SPACE %.2fX", space);
            drawText(vg, 8.f, 8.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "DECAY %2.0f%%", decay * 100.f);
            drawText(vg, W / 2.f, 8.f, buf, 7.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "MIX %3.0f%%", mix * 100.f);
            drawText(vg, W - 8.f, 8.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            if (frozen)
                snprintf(buf, sizeof(buf), "RT60 INF");
            else if (rt60 > 99.f)
                snprintf(buf, sizeof(buf), "RT60 >99S");
            else
                snprintf(buf, sizeof(buf), "RT60 %.1fS", rt60);
            drawText(vg, 8.f, 107.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "DAMP %.1fK", damp / 1000.f);
            drawText(vg, W / 2.f, 107.f, buf, 7.f, cd, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "PRE %3.0fMS  WET %3.0f%%", pre * 1000.f, wet * 100.f);
            drawText(vg, W - 8.f, 107.f, buf, 6.5f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// -----------------------------------------------------------------------------
// Widget
// -----------------------------------------------------------------------------
struct AmbienceWidget : ModuleWidget
{
    AmbienceWidget(Ambience *module)
    {
        setModule(module);
        box.size = Vec(AM_PANEL_W, AM_PANEL_H); 

        addChild(new AMPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new AMDisplayWidget(module));

        // ---- knobs: 4 columns x 2 rows ----
        const int knobIds[7] = {Ambience::PRE_DELAY_PARAM, Ambience::SPACE_PARAM, Ambience::DECAY_PARAM,
                                Ambience::DAMPING_PARAM, Ambience::WET_GAIN_PARAM, Ambience::MIX_PARAM,
                                Ambience::MOD_DEPTH_PARAM};
        for (int i = 0; i < 7; ++i)
            addParam(createParamCentered<RoundBlackKnob>(Vec(AM_KCOL[i % 4], AM_KROW[i / 4]), module, knobIds[i]));

        // Freeze switch with LED
        addParam(createParamCentered<CKSS>(Vec(AM_KCOL[3] - 2.f, AM_KROW[1]), module, Ambience::FREEZE_PARAM));
        addChild(createLightCentered<SmallLight<AMVioletLight>>(Vec(AM_KCOL[3] + 12.f, AM_KROW[1] - 13.f), module, Ambience::FREEZE_LIGHT));

        // ---- jack row ----
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amJackX(0), AM_JACK_Y), module, Ambience::INPUT_L));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amJackX(1), AM_JACK_Y), module, Ambience::INPUT_R));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amJackX(2), AM_JACK_Y), module, Ambience::DECAY_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amJackX(3), AM_JACK_Y), module, Ambience::FREEZE_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(amJackX(4), AM_JACK_Y), module, Ambience::MIX_CV_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(amJackX(5), AM_JACK_Y), module, Ambience::OUTPUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(amJackX(6), AM_JACK_Y), module, Ambience::OUTPUT_R));
    }
};

Model *modelAmbience = createModel<Ambience, AmbienceWidget>("Ambience");