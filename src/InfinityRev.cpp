#include "plugin.hpp"
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ============================================================================
//  InfinityRev - 8-line Feedback Delay Network reverb
//  Signal flow:
//    in -> pre-delay -> 4x allpass diffusion (per channel) -> 8-line FDN
//          (Hadamard feedback matrix, modulated delays, per-line damping,
//           RT60-based decay, freeze) -> decorrelated stereo taps -> width
//    + early reflections (6 taps) -> equal-power dry/wet mix -> DC block/limit
// ============================================================================

static const float IR_W = RACK_GRID_WIDTH * 24;   // 360
static const float IR_H = RACK_GRID_HEIGHT;       // 380
static const float IR_DX = 20.f, IR_DY = 46.f, IR_DW = 320.f, IR_DH = 120.f;
static const float IR_R1 = 184.f, IR_R2 = 246.f, IR_CH = 56.f;
static const float IR_JY = 346.f;
static float irCol(int i) { return 40.f + 70.f * i; }
static float irJx(int i) { return 31.f + 42.5f * i; }

// Circular delay line with linear + cubic (Hermite) fractional read.
struct IRDelayLine {
    std::vector<float> buf;
    int w = 0, n = 0;

    void init(int size) {
        n = std::max(size, 16);
        buf.assign(n, 0.f);
        w = 0;
    }
    void write(float x) {
        buf[w] = x;
        if (++w >= n) w = 0;
    }
    int wrap(int i) const {
        while (i < 0) i += n;
        while (i >= n) i -= n;
        return i;
    }
    // d >= 1
    float readLinear(float d) const {
        float rp = (float)w - d;
        int i1 = (int)std::floor(rp);
        float f = rp - (float)i1;
        i1 = wrap(i1);
        int i2 = wrap(i1 + 1);
        return buf[i1] + f * (buf[i2] - buf[i1]);
    }
    // d >= 3
    float readCubic(float d) const {
        float rp = (float)w - d;
        int i1 = (int)std::floor(rp);
        float f = rp - (float)i1;
        i1 = wrap(i1);
        float xm1 = buf[wrap(i1 - 1)];
        float x0  = buf[i1];
        float x1  = buf[wrap(i1 + 1)];
        float x2  = buf[wrap(i1 + 2)];
        float c1 = 0.5f * (x1 - xm1);
        float c2 = xm1 - 2.5f * x0 + 2.f * x1 - 0.5f * x2;
        float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
        return ((c3 * f + c2) * f + c1) * f + x0;
    }
};

// Schroeder allpass (fixed length)
struct IRAllpass {
    std::vector<float> buf;
    int pos = 0;
    void init(int len) { buf.assign(std::max(len, 1), 0.f); pos = 0; }
    float process(float x, float g) {
        float d = buf[pos];
        float v = x - g * d;
        buf[pos] = v;
        if (++pos >= (int)buf.size()) pos = 0;
        return d + g * v;
    }
};

static const int IR_N = 8;

// Delay lengths (ms) at Size = 1.0 - mutually "inharmonic" to avoid ringing
static const float kBaseMs[IR_N] = {29.7f, 37.1f, 41.1f, 43.7f, 53.3f, 59.9f, 67.3f, 73.1f};
// Slow, different LFO rates per line (Hz)
static const float kLfoHz[IR_N]  = {0.11f, 0.17f, 0.23f, 0.29f, 0.37f, 0.43f, 0.53f, 0.61f};
// Output tap signs (orthogonal Hadamard rows -> decorrelated L/R)
static const float kSignL[IR_N]  = {1, -1, -1, 1, 1, -1, -1, 1};
static const float kSignR[IR_N]  = {1, 1, -1, -1, -1, -1, 1, 1};
// Diffuser delays (ms)
static const float kApMsL[4]  = {4.77f, 3.60f, 12.73f, 9.31f};
static const float kApMsR[4]  = {5.13f, 3.91f, 11.89f, 8.67f};
// Early reflection taps
static const float kErMs[6]   = {7.3f, 11.9f, 17.1f, 23.7f, 31.3f, 41.9f};
static const float kErGL[6]   = {0.80f, -0.60f, 0.50f, -0.40f, 0.30f, -0.25f};
static const float kErGR[6]   = {-0.70f, 0.70f, -0.45f, 0.45f, -0.30f, 0.28f};

// Tooltip helpers
struct IRDecayQ : rack::engine::ParamQuantity {
    std::string getDisplayValueString() override {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f s", 0.2f * std::pow(300.f, getValue()));
        return std::string(buf);
    }
};
struct IRDampQ : rack::engine::ParamQuantity {
    std::string getDisplayValueString() override {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f kHz", 20.f * std::pow(0.05f, getValue()));
        return std::string(buf);
    }
};

struct InfinityRev : Module {
    enum ParamId {
        PREDELAY_PARAM, SIZE_PARAM,
        DECAY_PARAM, DAMP_PARAM,
        DIFFUSION_PARAM, MOD_PARAM,
        ER_PARAM, WIDTH_PARAM,
        MIX_PARAM, FREEZE_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        INPUT_L, INPUT_R,
        DECAY_CV_INPUT, SIZE_CV_INPUT, MIX_CV_INPUT, FREEZE_GATE_INPUT,
        INPUTS_LEN
    };
    enum OutputId { OUTPUT_L, OUTPUT_R, OUTPUTS_LEN };
    enum LightId { FREEZE_LIGHT, LIGHTS_LEN };

    float sr = 0.f;
    IRDelayLine pdL, pdR, er;
    IRAllpass apL[4], apR[4];
    IRDelayLine fdn[IR_N];
    float lp[IR_N] = {};
    float phase[IR_N] = {};
    float gain[IR_N] = {};
    float dampK = 0.5f;
    float apG = 0.6f;
    float sizeS = 1.f;
    float freezeS = 0.f;
    float smoothK = 0.001f;
    int ctr = 0;
    float dcInL = 0.f, dcOutL = 0.f, dcInR = 0.f, dcOutR = 0.f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float lineLevel[IR_N] = {};
    float meterIn = 0.f, meterOutL = 0.f, meterOutR = 0.f;
    float dispRt60 = 4.f, dispCut = 8000.f, dispMix = 0.35f;

    InfinityRev() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PREDELAY_PARAM, 0.f, 250.f, 20.f, "Pre-delay", " ms");
        configParam(SIZE_PARAM, 0.5f, 2.f, 1.f, "Size", "x");
        configParam<IRDecayQ>(DECAY_PARAM, 0.f, 1.f, 0.65f, "Decay (RT60)");
        configParam<IRDampQ>(DAMP_PARAM, 0.f, 1.f, 0.4f, "Damping (cutoff)");
        configParam(DIFFUSION_PARAM, 0.f, 1.f, 0.75f, "Diffusion", "%", 0.f, 100.f);
        configParam(MOD_PARAM, 0.f, 1.f, 0.3f, "Modulation", "%", 0.f, 100.f);
        configParam(ER_PARAM, 0.f, 1.f, 0.4f, "Early reflections", "%", 0.f, 100.f);
        configParam(WIDTH_PARAM, 0.f, 1.5f, 1.f, "Stereo width", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.35f, "Mix (dry/wet)", "%", 0.f, 100.f);
        configSwitch(FREEZE_PARAM, 0.f, 1.f, 0.f, "Freeze", {"Off", "On"});
        configInput(INPUT_L, "Left (mono if the right input is empty)");
        configInput(INPUT_R, "Right");
        configInput(DECAY_CV_INPUT, "Decay CV (10 V = full range)");
        configInput(SIZE_CV_INPUT, "Size CV (10 V adds 1.5x)");
        configInput(MIX_CV_INPUT, "Mix CV (10 V = 100%)");
        configInput(FREEZE_GATE_INPUT, "Freeze gate");
        configOutput(OUTPUT_L, "Left");
        configOutput(OUTPUT_R, "Right");
        configLight(FREEZE_LIGHT, "Freeze");
        prepare(48000.f);
    }

    void prepare(float newSr) {
        sr = newSr;
        pdL.init((int)(0.26f * sr));
        pdR.init((int)(0.26f * sr));
        er.init((int)(0.10f * sr));
        for (int i = 0; i < 4; ++i) {
            apL[i].init((int)std::round(kApMsL[i] * 0.001f * sr));
            apR[i].init((int)std::round(kApMsR[i] * 0.001f * sr));
        }
        for (int i = 0; i < IR_N; ++i) {
            fdn[i].init((int)(0.30f * sr));
            lp[i] = 0.f;
            phase[i] = (float)i / (float)IR_N;   // spread LFO phases
            gain[i] = 0.9f;
        }
        smoothK = 1.f - std::exp(-1.f / (0.08f * sr));
        ctr = 0;
        dcInL = dcOutL = dcInR = dcOutR = 0.f;
    }

    void onReset() override { prepare(sr); }

    // Control-rate update (every 32 samples)
    void updateControl() {
        float dec = params[DECAY_PARAM].getValue() + inputs[DECAY_CV_INPUT].getVoltage() * 0.1f;
        dec = rack::math::clamp(dec, 0.f, 1.f);
        float rt60 = 0.2f * std::pow(300.f, dec);          // 0.2 s .. 60 s
        dispRt60 = rt60;

        for (int i = 0; i < IR_N; ++i) {
            float delaySec = kBaseMs[i] * 0.001f * sizeS;
            float g = std::pow(10.f, -3.f * delaySec / rt60);
            gain[i] = g + (0.99995f - g) * freezeS;         // freeze = infinite
        }

        float damp = params[DAMP_PARAM].getValue();
        float cutoff = 20000.f * std::pow(0.05f, damp);      // 20k .. 1k
        cutoff = std::min(cutoff, 0.45f * sr);
        dispCut = cutoff;
        float k = 1.f - std::exp(-2.f * (float)M_PI * cutoff / sr);
        dampK = k + (1.f - k) * freezeS;                     // no damping while frozen

        apG = 0.1f + 0.65f * params[DIFFUSION_PARAM].getValue();
    }

    static float softLimit(float x) {
        const float t = 8.f;
        float a = std::fabs(x);
        if (a <= t) return x;
        float over = a - t;
        float y = t + 2.f * std::tanh(over / 2.f);           // asymptote at 10 V
        return x < 0.f ? -y : y;
    }

    void process(const ProcessArgs& args) override {
        if (args.sampleRate != sr) prepare(args.sampleRate);

        float inL = inputs[INPUT_L].getVoltage();
        float inR = inputs[INPUT_R].isConnected() ? inputs[INPUT_R].getVoltage() : inL;

        // Smoothed size / freeze
        float sizeT = params[SIZE_PARAM].getValue() + inputs[SIZE_CV_INPUT].getVoltage() * 0.15f;
        sizeT = rack::math::clamp(sizeT, 0.5f, 2.f);
        sizeS += smoothK * (sizeT - sizeS);

        bool frz = params[FREEZE_PARAM].getValue() > 0.5f
                || inputs[FREEZE_GATE_INPUT].getVoltage() >= 1.f;
        freezeS += smoothK * ((frz ? 1.f : 0.f) - freezeS);
        lights[FREEZE_LIGHT].setBrightness(freezeS);

        if (--ctr <= 0) { ctr = 32; updateControl(); }

        // ---- Pre-delay + early reflections ----
        float pdSamp = std::max(3.f, params[PREDELAY_PARAM].getValue() * 0.001f * sr);
        float pL = pdL.readLinear(pdSamp);
        float pR = pdR.readLinear(pdSamp);
        pdL.write(inL);
        pdR.write(inR);

        float erLevel = params[ER_PARAM].getValue();
        float erL = 0.f, erR = 0.f;
        for (int t = 0; t < 6; ++t) {
            float s = er.readLinear(kErMs[t] * 0.001f * sr);
            erL += s * kErGL[t];
            erR += s * kErGR[t];
        }
        er.write(0.5f * (inL + inR));

        // ---- Input diffusion ----
        float dL = pL, dR = pR;
        for (int k = 0; k < 4; ++k) {
            dL = apL[k].process(dL, apG);
            dR = apR[k].process(dR, apG);
        }
        float inGain = (1.f - freezeS) * 0.5f;
        dL *= inGain;
        dR *= inGain;

        // ---- FDN ----
        float modDepth = params[MOD_PARAM].getValue() * 0.0005f * sr;   // up to ~0.5 ms
        float y[IR_N];
        float md = 1.f - 6.f * args.sampleTime;
        for (int i = 0; i < IR_N; ++i) {
            float lfo = std::sin(2.f * (float)M_PI * phase[i]);
            phase[i] += kLfoHz[i] / sr;
            if (phase[i] >= 1.f) phase[i] -= 1.f;

            float d = kBaseMs[i] * 0.001f * sr * sizeS + modDepth * lfo;
            d = rack::math::clamp(d, 4.f, (float)fdn[i].n - 4.f);
            float v = fdn[i].readCubic(d);
            lp[i] += dampK * (v - lp[i]);
            y[i] = lp[i] * gain[i];
            lineLevel[i] = std::max(std::fabs(y[i]), lineLevel[i] * md);
        }

        // Output taps (before the feedback matrix)
        float wL = 0.f, wR = 0.f;
        for (int i = 0; i < IR_N; ++i) {
            wL += kSignL[i] * y[i];
            wR += kSignR[i] * y[i];
        }
        wL *= 0.3f;
        wR *= 0.3f;

        // Fast Walsh-Hadamard transform (orthonormal feedback matrix)
        float m[IR_N];
        for (int i = 0; i < IR_N; ++i) m[i] = y[i];
        for (int len = 1; len < IR_N; len <<= 1)
            for (int i = 0; i < IR_N; i += len * 2)
                for (int j = i; j < i + len; ++j) {
                    float a = m[j], b = m[j + len];
                    m[j] = a + b;
                    m[j + len] = a - b;
                }
        const float hn = 0.35355339f; // 1/sqrt(8)
        for (int i = 0; i < IR_N; ++i)
            fdn[i].write(m[i] * hn + ((i & 1) ? dR : dL));

        // ---- Wet assembly ----
        wL += erL * erLevel * 0.35f;
        wR += erR * erLevel * 0.35f;

        float mid = 0.5f * (wL + wR);
        float side = 0.5f * (wL - wR) * params[WIDTH_PARAM].getValue();
        wL = mid + side;
        wR = mid - side;

        // ---- Equal-power mix ----
        float mix = params[MIX_PARAM].getValue() + inputs[MIX_CV_INPUT].getVoltage() * 0.1f;
        mix = rack::math::clamp(mix, 0.f, 1.f);
        dispMix = mix;
        float dryG = std::cos(mix * 0.5f * (float)M_PI);
        float wetG = std::sin(mix * 0.5f * (float)M_PI);
        float oL = dryG * inL + wetG * wL;
        float oR = dryG * inR + wetG * wR;

        // ---- DC block + limiter ----
        float hpL = oL - dcInL + 0.995f * dcOutL;  dcInL = oL;  dcOutL = hpL;
        float hpR = oR - dcInR + 0.995f * dcOutR;  dcInR = oR;  dcOutR = hpR;

        float finalL = softLimit(hpL), finalR = softLimit(hpR);
        outputs[OUTPUT_L].setVoltage(finalL);
        outputs[OUTPUT_R].setVoltage(finalR);

        meterIn = std::max(0.5f * (std::fabs(inL) + std::fabs(inR)), meterIn * md);
        meterOutL = std::max(std::fabs(finalL), meterOutL * md);
        meterOutR = std::max(std::fabs(finalR), meterOutR * md);
    }
};

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
struct TIRVioletLight : TBase {
    TIRVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
};
using IRVioletLight = TIRVioletLight<>;

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

void topCell(NVGcontext* vg, float cx, float cy, const char* t) {
    plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
    label(vg, cx, cy - 19.f, t);
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

// Shaded sphere for the FDN nodes
void drawNode(NVGcontext* vg, float cx, float cy, float r, NVGcolor hi, NVGcolor base, NVGcolor edge) {
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

struct IRPanel : TransparentWidget {
    IRPanel() { box.size = Vec(IR_W, IR_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, IR_W, IR_H, "INFINITYREV");
        drawBezel(vg, IR_DX, IR_DY, IR_DW, IR_DH);

        const char* r1[5] = {"PRE-DELAY", "SIZE", "DECAY", "DAMPING", "DIFFUSE"};
        const char* r2[5] = {"MOD", "EARLY", "WIDTH", "MIX", "FREEZE"};
        for (int i = 0; i < 5; ++i) {
            float cx = irCol(i);
            plate(vg, cx - 32.f, IR_R1, 64.f, IR_CH);
            ring(vg, cx, IR_R1 + 20.f, 16.f);
            label(vg, cx, IR_R1 + 46.f, r1[i]);

            plate(vg, cx - 32.f, IR_R2, 64.f, IR_CH);
            if (i < 4) ring(vg, cx, IR_R2 + 20.f, 16.f);
            label(vg, cx, IR_R2 + 46.f, r2[i]);
        }

        const char* io[8] = {"L IN", "R IN", "DEC CV", "SIZ CV", "MIX CV", "FRZ", "OUT L", "OUT R"};
        for (int i = 0; i < 8; ++i) topCell(vg, irJx(i), IR_JY, io[i]);
        Widget::draw(args);
    }
};

// Display: FDN network, decay curve and damping response
struct IRDisplay : TransparentWidget {
    InfinityRev* module = nullptr;
    IRDisplay(InfinityRev* m) : module(m) {
        box.pos = Vec(IR_DX, IR_DY);
        box.size = Vec(IR_DW, IR_DH);
    }

    static constexpr float FX = 170.f, FW = 120.f;
    static constexpr float DY0 = 20.f, DH0 = 42.f;     // decay field
    static constexpr float PY0 = 76.f, PH0 = 20.f;     // damping field

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        screenBase(vg, IR_DW, IR_DH);
        const float fy[2] = {DY0, PY0}, fh[2] = {DH0, PH0};
        for (int k = 0; k < 2; ++k) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, FX - 1.f, fy[k] - 1.f, FW + 2.f, fh[k] + 2.f, 2.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }
        nvgBeginPath(vg);
        for (int t = 1; t < 4; ++t) {
            float x = FX + FW * t / 4.f + 0.5f;
            nvgMoveTo(vg, x, DY0);
            nvgLineTo(vg, x, DY0 + DH0);
        }
        for (int d = 1; d <= 3; ++d) {     // 100 Hz, 1 kHz, 10 kHz
            float x = FX + FW * (std::log10(std::pow(10.f, (float) d) * 10.f / 20.f) / 3.f) + 0.5f;
            nvgMoveTo(vg, x, PY0);
            nvgLineTo(vg, x, PY0 + PH0);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        drawText(vg, FX + 2.f, DY0 + 6.f, "DECAY 0-8S", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, FX + 2.f, PY0 + 6.f, "DAMP", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = IR_DW, H = IR_DH;

        float lvl[IR_N], mIn = 2.f, mL = 2.8f, mR = 2.4f, size = 1.f, rt60 = 4.f, cut = 8000.f;
        float frz = 0.f, pre = 20.f, er = 0.4f, wid = 1.f, mix = 0.35f;
        for (int i = 0; i < IR_N; ++i) lvl[i] = 0.4f + 1.6f * std::fabs(std::sin(i * 1.3f + 0.4f));
        if (module) {
            for (int i = 0; i < IR_N; ++i) lvl[i] = module->lineLevel[i];
            mIn = module->meterIn; mL = module->meterOutL; mR = module->meterOutR;
            size = module->sizeS; rt60 = module->dispRt60; cut = module->dispCut;
            frz = module->freezeS; mix = module->dispMix;
            pre = module->params[InfinityRev::PREDELAY_PARAM].getValue();
            er = module->params[InfinityRev::ER_PARAM].getValue();
            wid = module->params[InfinityRev::WIDTH_PARAM].getValue();
        }
        rt60 = std::max(rt60, 0.01f);

        // ---- FDN network: 8 delay lines on a ring, joined by the Hadamard matrix ----
        const float cx = 72.f, cy = 66.f, R = 40.f;
        float nx[IR_N], ny[IR_N], act[IR_N];
        for (int i = 0; i < IR_N; ++i) {
            float a = -(float) M_PI / 2.f + i * (float) M_PI / 4.f;
            nx[i] = cx + R * std::cos(a);
            ny[i] = cy + R * std::sin(a);
            act[i] = std::min(lvl[i] / 2.5f, 1.f);
        }
        NVGcolor lineCol = nvgLerpRGBA(colAccent(), colViolet(), frz);
        for (int i = 0; i < IR_N; ++i) {
            for (int j = i + 1; j < IR_N; ++j) {
                bool neighbour = (j - i == 1) || (i == 0 && j == IR_N - 1);
                float a = 0.04f + 0.5f * 0.5f * (act[i] + act[j]);
                if (neighbour) {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, nx[i], ny[i]);
                    nvgLineTo(vg, nx[j], ny[j]);
                    nvgStrokeColor(vg, withAlpha(lineCol, a * 0.35f));
                    nvgStrokeWidth(vg, 3.5f);
                    nvgStroke(vg);
                }
                nvgBeginPath(vg);
                nvgMoveTo(vg, nx[i], ny[i]);
                nvgLineTo(vg, nx[j], ny[j]);
                nvgStrokeColor(vg, withAlpha(nvgLerpRGBA(lineCol, nvgRGB(255, 255, 255), 0.4f), neighbour ? a : a * 0.7f));
                nvgStrokeWidth(vg, neighbour ? 1.f : 0.6f);
                nvgStroke(vg);
            }
        }

        // input and output dots
        float inA = std::min(mIn / 5.f, 1.f);
        float lA = std::min(mL / 5.f, 1.f), rA = std::min(mR / 5.f, 1.f);
        const float ix = 12.f, ox = 134.f;
        auto link = [&](float x0, float y0, float x1, float y1, float a, NVGcolor c) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, x0, y0);
            nvgLineTo(vg, x1, y1);
            nvgStrokeColor(vg, withAlpha(c, 0.12f + 0.6f * a));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        };
        link(ix, cy, nx[6], ny[6], inA, colAccent());
        link(nx[2], ny[2], ox, cy - 14.f, lA, colViolet());
        link(nx[2], ny[2], ox, cy + 14.f, rA, colViolet());
        auto dot = [&](float x, float y, float a, NVGcolor c) {
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
        dot(ox, cy - 14.f, lA, colViolet());
        dot(ox, cy + 14.f, rA, colViolet());
        drawText(vg, ix, cy + 10.f, "IN", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, ox + 8.f, cy - 14.f, "L", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.5f);
        drawText(vg, ox + 8.f, cy + 14.f, "R", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.5f);

        // nodes: size follows delay length (Size), glow follows the signal
        for (int i = 0; i < IR_N; ++i) {
            float r = 3.2f + 4.f * (kBaseMs[i] * size / 146.2f);
            float hue = 0.5f + 0.06f * (float) i;
            NVGcolor hc = nvgHSL(hue, 0.9f, 0.6f);
            nvgBeginPath(vg);
            nvgCircle(vg, nx[i], ny[i], r * 2.6f);
            nvgFillPaint(vg, nvgRadialGradient(vg, nx[i], ny[i], r * 0.6f, r * 2.6f,
                                               withAlpha(hc, 0.08f + 0.35f * act[i]), withAlpha(hc, 0.f)));
            nvgFill(vg);
            drawNode(vg, nx[i], ny[i], r, nvgHSL(hue, 0.9f, 0.86f), nvgHSL(hue, 0.8f, 0.42f), nvgHSL(hue, 0.8f, 0.16f));
        }
        if (frz > 0.5f)
            drawText(vg, cx, cy, "FREEZE", 7.f, withAlpha(colViolet(), 0.95f), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        // ---- decay curve (0..8 s) ----
        {
            float preS = pre * 0.001f;
            auto amp = [&](float t) {
                float a = (t < preS) ? 0.f : std::exp(-6.9078f * (t - preS) / rt60);
                float fl = (t < preS) ? 0.f : 1.f;
                return a * (1.f - frz) + fl * frz;
            };
            nvgBeginPath(vg);
            nvgMoveTo(vg, FX, DY0 + DH0 - 1.f);
            for (int i = 0; i <= (int) FW; ++i)
                nvgLineTo(vg, FX + i, DY0 + DH0 - 1.f - amp(8.f * i / FW) * (DH0 - 2.f));
            nvgLineTo(vg, FX + FW, DY0 + DH0 - 1.f);
            nvgClosePath(vg);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, DY0, 0, DY0 + DH0, nvgRGBA(0x3d, 0xe0, 0xff, 70), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
            nvgFill(vg);
            for (int pass = 0; pass < 2; ++pass) {
                nvgBeginPath(vg);
                for (int i = 0; i <= (int) FW; ++i) {
                    float x = FX + i, y = DY0 + DH0 - 1.f - amp(8.f * i / FW) * (DH0 - 2.f);
                    if (i == 0) nvgMoveTo(vg, x, y);
                    else nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 50) : nvgRGB(0x8a, 0xee, 0xff));
                nvgStrokeWidth(vg, pass == 0 ? 3.5f : 1.3f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);
            }
            if (frz < 0.5f && rt60 + preS < 8.f) {
                float mx = FX + FW * (rt60 + preS) / 8.f;
                nvgBeginPath(vg);
                for (float y = DY0 + 1.f; y < DY0 + DH0 - 1.f; y += 4.f) {
                    nvgMoveTo(vg, mx + 0.5f, y);
                    nvgLineTo(vg, mx + 0.5f, y + 2.f);
                }
                nvgStrokeColor(vg, withAlpha(colViolet(), 0.9f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
        }

        // ---- damping response (one-pole, log 20 Hz..20 kHz, 0..-36 dB) ----
        {
            float fc = cut * (1.f - frz) + 40000.f * frz;
            auto yAt = [&](int i) {
                float f = 20.f * std::pow(1000.f, (float) i / FW);
                float db = -10.f * std::log10(1.f + (f / fc) * (f / fc));
                float t = std::min(std::max(-db / 36.f, 0.f), 1.f);
                return PY0 + 1.f + t * (PH0 - 2.f);
            };
            for (int pass = 0; pass < 2; ++pass) {
                nvgBeginPath(vg);
                for (int i = 0; i <= (int) FW; ++i) {
                    if (i == 0) nvgMoveTo(vg, FX + i, yAt(i));
                    else nvgLineTo(vg, FX + i, yAt(i));
                }
                nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0xb0, 0x5a, 0xff, 60) : nvgRGB(0xd8, 0xb0, 0xff));
                nvgStrokeWidth(vg, pass == 0 ? 3.5f : 1.2f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);
            }
            float cxm = FX + FW * std::min(std::log10(std::max(fc, 20.f) / 20.f) / 3.f, 1.f);
            nvgBeginPath(vg);
            nvgRect(vg, cxm, PY0, 1.f, PH0);
            nvgFillColor(vg, withAlpha(colAccent(), 0.7f));
            nvgFill(vg);
        }

        // ---- meters: in, L, R ----
        for (int k = 0; k < 3; ++k) {
            float v = k == 0 ? mIn : (k == 1 ? mL : mR);
            float lv = std::min(v / 5.f, 1.f);
            float x = 300.f + k * 7.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 20.f, 5.f, 76.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lv > 0.01f) {
                nvgBeginPath(vg);
                nvgRect(vg, x, 96.f - lv * 76.f, 5.f, lv * 76.f);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, 96.f, 0, 20.f, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }
        drawText(vg, 302.5f, 102.f, "I", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        drawText(vg, 309.5f, 102.f, "L", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        drawText(vg, 316.5f, 102.f, "R", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

        screenFinish(vg, W, H);

        char buf[64];
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
        NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 230);
        snprintf(buf, sizeof(buf), "SIZE %.2fX", size);
        drawText(vg, 8.f, 8.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        if (frz > 0.5f) snprintf(buf, sizeof(buf), "RT60 INF");
        else if (rt60 > 99.f) snprintf(buf, sizeof(buf), "RT60 >99S");
        else snprintf(buf, sizeof(buf), "RT60 %.1fS", rt60);
        drawText(vg, W / 2.f, 8.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "MIX %3.0f%%", mix * 100.f);
        drawText(vg, W - 8.f, 8.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "PRE %3.0fMS", pre);
        drawText(vg, 8.f, 113.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "ER %2.0f%%  WID %3.0f%%", er * 100.f, wid * 100.f);
        drawText(vg, W / 2.f, 113.f, buf, 7.f, cd, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "DAMP %.1fK", cut / 1000.f);
        drawText(vg, W - 8.f, 113.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct InfinityRevWidget : ModuleWidget {
    InfinityRevWidget(InfinityRev* module) {
        setModule(module);
        box.size = Vec(IR_W, IR_H);

        addChild(new IRPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new IRDisplay(module));

        const int row1[5] = {InfinityRev::PREDELAY_PARAM, InfinityRev::SIZE_PARAM, InfinityRev::DECAY_PARAM,
                             InfinityRev::DAMP_PARAM, InfinityRev::DIFFUSION_PARAM};
        const int row2[4] = {InfinityRev::MOD_PARAM, InfinityRev::ER_PARAM, InfinityRev::WIDTH_PARAM, InfinityRev::MIX_PARAM};
        for (int i = 0; i < 5; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(irCol(i), IR_R1 + 20.f), module, row1[i]));
        for (int i = 0; i < 4; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(irCol(i), IR_R2 + 20.f), module, row2[i]));
        addParam(createParamCentered<CKSS>(Vec(irCol(4) - 3.f, IR_R2 + 21.f), module, InfinityRev::FREEZE_PARAM));
        addChild(createLightCentered<SmallLight<IRVioletLight>>(Vec(irCol(4) + 14.f, IR_R2 + 10.f), module, InfinityRev::FREEZE_LIGHT));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(0), IR_JY), module, InfinityRev::INPUT_L));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(1), IR_JY), module, InfinityRev::INPUT_R));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(2), IR_JY), module, InfinityRev::DECAY_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(3), IR_JY), module, InfinityRev::SIZE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(4), IR_JY), module, InfinityRev::MIX_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(irJx(5), IR_JY), module, InfinityRev::FREEZE_GATE_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(irJx(6), IR_JY), module, InfinityRev::OUTPUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(irJx(7), IR_JY), module, InfinityRev::OUTPUT_R));
    }
};

Model* modelInfinityRev = createModel<InfinityRev, InfinityRevWidget>("InfinityRev");