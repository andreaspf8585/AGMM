#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float PR_W = RACK_GRID_WIDTH * 14;   // 210
static const float PR_H = RACK_GRID_HEIGHT;       // 380
static const float PR_DX = 30.f, PR_DY = 46.f, PR_DW = 150.f, PR_DH = 92.f;
static const float PR_ROW[4] = {164.f, 202.f, 240.f, 278.f};   // top of each control row
static const float PR_ROWH = 34.f;
static const float PR_CVX = 100.f, PR_ATX = 137.f, PR_KX = 173.f;
static const float PR_IOY = 346.f;
static float prIoX(int i) { return 30.f + 50.f * i; }

// ------------------------------------------------------------
// Freeverb-style reverb (no runaway gain)
// ------------------------------------------------------------
struct PsyReverbEngine {
    struct DelayLine {
        float *buffer = nullptr;
        int length = 0;
        int idx = 0;

        void init(int len) {
            destroy();                       // re-init no longer leaks
            length = std::max(len, 1);
            buffer = new float[length]();
            idx = 0;
        }
        void destroy() {
            delete[] buffer;
            buffer = nullptr;
        }
        float read() const { return buffer[idx]; }
        void write(float value) {
            buffer[idx] = value;
            idx = (idx + 1) % length;
        }
        void reset() {
            if (buffer) {
                for (int i = 0; i < length; i++) buffer[i] = 0.0f;
            }
            idx = 0;
        }
    };

    float sampleRate = 44100.0f;
    float wet = 0.5f;
    float dry = 0.5f;
    float roomsize = 0.8f;
    float damping = 0.5f;

    DelayLine comb[8];
    DelayLine allpass[4];
    float combLowpassState[8] = {};

    PsyReverbEngine() {}
    ~PsyReverbEngine() {
        for (int i = 0; i < 8; i++) comb[i].destroy();
        for (int i = 0; i < 4; i++) allpass[i].destroy();
    }

    void init(float sr) {
        sampleRate = sr;

        int combDelays[8] = {
            (int)(0.0297f * sr), (int)(0.0371f * sr), (int)(0.0411f * sr), (int)(0.0437f * sr),
            (int)(0.0489f * sr), (int)(0.0557f * sr), (int)(0.0623f * sr), (int)(0.0697f * sr)
        };
        int apDelays[4] = {
            (int)(0.0051f * sr), (int)(0.0072f * sr), (int)(0.0103f * sr), (int)(0.0149f * sr)
        };

        for (int i = 0; i < 8; i++) {
            comb[i].init(combDelays[i]);
            combLowpassState[i] = 0.0f;
        }
        for (int i = 0; i < 4; i++)
            allpass[i].init(apDelays[i]);
    }

    void setRoomsize(float value) { roomsize = value; }
    void setDamping(float value) { damping = value; }

    inline float lowpass(float in, float &state, float cutoff) {
        if (cutoff >= 19999.0f) return in;
        float coeff = 1.0f - std::exp(-2.0f * float(M_PI) * cutoff / sampleRate);
        state = state + coeff * (in - state);
        return state;
    }

    // allpass: feedback uses the output sample, not the delayed sample
    inline float processAllpass(float in, DelayLine &delay, float g) {
        float d = delay.read();
        float out = -in + d;
        delay.write(in + out * g);
        return out;
    }

    void processStereo(float leftIn, float rightIn, float &leftOut, float &rightOut) {
        float leftReverb = 0.0f;
        float dampFreq = 20000.0f - damping * 19900.0f;

        for (int i = 0; i < 8; i++) {
            float del = comb[i].read();
            del = lowpass(del, combLowpassState[i], dampFreq);
            float fb = 0.5f + roomsize * 0.48f;   // max 0.98
            comb[i].write(leftIn + del * fb);
            leftReverb += del;
        }
        leftReverb = leftReverb / 8.0f;

        for (int i = 0; i < 4; i++)
            leftReverb = processAllpass(leftReverb, allpass[i], 0.7f);

        float rightReverb = leftReverb;  // mono reverb, same on both channels

        float wetGain = 1.0f;
        leftOut = (dry * leftIn) + (wet * leftReverb * wetGain);
        rightOut = (dry * rightIn) + (wet * rightReverb * wetGain);

        leftOut = math::clamp(leftOut, -10.0f, 10.0f);
        rightOut = math::clamp(rightOut, -10.0f, 10.0f);
    }

    void processMono(float in, float &outL, float &outR) {
        processStereo(in, in, outL, outR);
    }
};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct PsyReverb : Module {
    enum ParamId {
        PARAM_WET, PARAM_DRY, PARAM_ROOMSIZE, PARAM_DAMPING,
        PARAM_ATTEN_WET, PARAM_ATTEN_DRY, PARAM_ATTEN_ROOMSIZE, PARAM_ATTEN_DAMPING,
        PARAMS_LEN
    };
    enum InputId {
        INPUT_LEFT, INPUT_RIGHT,
        CV_WET, CV_DRY, CV_ROOMSIZE, CV_DAMPING,
        INPUTS_LEN
    };
    enum OutputId { OUTPUT_LEFT, OUTPUT_RIGHT, OUTPUTS_LEN };
    enum LightId { LIGHTS_LEN };

    PsyReverbEngine reverb;
    dsp::ClockDivider paramDivider;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float meterIn = 0.f, meterOutL = 0.f, meterOutR = 0.f;

    PsyReverb() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(PARAM_WET, 0.f, 1.f, 0.5f, "Wet", "%", 0.f, 100.f);
        configParam(PARAM_DRY, 0.f, 1.f, 0.5f, "Dry", "%", 0.f, 100.f);
        configParam(PARAM_ROOMSIZE, 0.f, 1.f, 0.8f, "Room size", "%", 0.f, 100.f);
        configParam(PARAM_DAMPING, 0.f, 1.f, 0.5f, "Damping", "%", 0.f, 100.f);

        configParam(PARAM_ATTEN_WET, -1.f, 1.f, 0.f, "Wet CV attenuverter", "%", 0.f, 100.f);
        configParam(PARAM_ATTEN_DRY, -1.f, 1.f, 0.f, "Dry CV attenuverter", "%", 0.f, 100.f);
        configParam(PARAM_ATTEN_ROOMSIZE, -1.f, 1.f, 0.f, "Room size CV attenuverter", "%", 0.f, 100.f);
        configParam(PARAM_ATTEN_DAMPING, -1.f, 1.f, 0.f, "Damping CV attenuverter", "%", 0.f, 100.f);

        configInput(INPUT_LEFT, "Left (mono if the right input is empty)");
        configInput(INPUT_RIGHT, "Right");
        configInput(CV_WET, "Wet CV (10 V = 100%)");
        configInput(CV_DRY, "Dry CV (10 V = 100%)");
        configInput(CV_ROOMSIZE, "Room size CV (10 V = 100%)");
        configInput(CV_DAMPING, "Damping CV (10 V = 100%)");
        configOutput(OUTPUT_LEFT, "Left");
        configOutput(OUTPUT_RIGHT, "Right");

        paramDivider.setDivision(32);
        onSampleRateChange();
    }

    void onSampleRateChange() override {
        reverb.init(APP->engine->getSampleRate());
    }

    void process(const ProcessArgs& args) override {
        if (paramDivider.process()) {
            auto compute = [this](ParamId param, ParamId atten, InputId cv) {
                float base = params[param].getValue();
                float attenVal = params[atten].getValue();
                float cvVal = inputs[cv].getVoltage() / 10.f;
                return clamp(base + cvVal * attenVal, 0.f, 1.f);
            };

            reverb.wet = compute(PARAM_WET, PARAM_ATTEN_WET, CV_WET);
            reverb.dry = compute(PARAM_DRY, PARAM_ATTEN_DRY, CV_DRY);
            reverb.setRoomsize(compute(PARAM_ROOMSIZE, PARAM_ATTEN_ROOMSIZE, CV_ROOMSIZE));
            reverb.setDamping(compute(PARAM_DAMPING, PARAM_ATTEN_DAMPING, CV_DAMPING));
        }

        float leftIn = inputs[INPUT_LEFT].getVoltageSum();
        float rightIn = inputs[INPUT_RIGHT].getNormalVoltage(leftIn);
        float leftOut = 0.f, rightOut = 0.f;

        if (inputs[INPUT_RIGHT].isConnected())
            reverb.processStereo(leftIn, rightIn, leftOut, rightOut);
        else
            reverb.processMono(leftIn, leftOut, rightOut);

        outputs[OUTPUT_LEFT].setVoltage(leftOut);
        outputs[OUTPUT_RIGHT].setVoltage(rightOut);

        float md = 1.f - 5.f * args.sampleTime;
        meterIn = std::max(0.5f * (std::fabs(leftIn) + std::fabs(rightIn)), meterIn * md);
        meterOutL = std::max(std::fabs(leftOut), meterOutL * md);
        meterOutR = std::max(std::fabs(rightOut), meterOutR * md);
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
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

struct PRPanel : TransparentWidget {
    PRPanel() { box.size = Vec(PR_W, PR_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, PR_W, PR_H, "PSYREVERB");
        drawBezel(vg, PR_DX, PR_DY, PR_DW, PR_DH);

        drawText(vg, PR_CVX, 157.f, "CV", 6.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, PR_ATX, 157.f, "ATT", 6.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, PR_KX, 157.f, "KNOB", 6.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        const char* names[4] = {"WET", "DRY", "ROOM", "DAMP"};
        for (int i = 0; i < 4; ++i) {
            plate(vg, 10.f, PR_ROW[i], 190.f, PR_ROWH);
            drawText(vg, 20.f, PR_ROW[i] + PR_ROWH / 2.f, names[i], 8.f, colText(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.5f);
        }

        const char* io[4] = {"L IN", "R IN", "L OUT", "R OUT"};
        for (int i = 0; i < 4; ++i) topCell(vg, prIoX(i), PR_IOY, io[i]);
        Widget::draw(args);
    }
};

struct PRDisplay : TransparentWidget {
    PsyReverb* module = nullptr;
    PRDisplay(PsyReverb* m) : module(m) {
        box.pos = Vec(PR_DX, PR_DY);
        box.size = Vec(PR_DW, PR_DH);
    }

    static constexpr float FX = 10.f, FW = 100.f;
    static constexpr float DY = 16.f, DH = 34.f;     // decay field
    static constexpr float PY = 58.f, PH = 18.f;     // damping field

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        screenBase(vg, PR_DW, PR_DH);
        const float fy[2] = {DY, PY}, fh[2] = {DH, PH};
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
            nvgMoveTo(vg, x, DY);
            nvgLineTo(vg, x, DY + DH);
        }
        for (int d = 1; d <= 3; ++d) {     // 100 Hz, 1 kHz, 10 kHz
            float x = FX + FW * (std::log10(std::pow(10.f, (float) d) * 10.f / 20.f) / 3.f) + 0.5f;
            nvgMoveTo(vg, x, PY);
            nvgLineTo(vg, x, PY + PH);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        drawText(vg, FX + 2.f, DY + 6.f, "DECAY 0-8S", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, FX + 2.f, PY + 6.f, "DAMP", 5.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = PR_DW, H = PR_DH;

        float wet = 0.5f, dry = 0.5f, room = 0.8f, damp = 0.5f, mIn = 2.f, mL = 2.8f, mR = 2.8f;
        if (module) {
            wet = module->reverb.wet; dry = module->reverb.dry;
            room = module->reverb.roomsize; damp = module->reverb.damping;
            mIn = module->meterIn; mL = module->meterOutL; mR = module->meterOutR;
        }

        // RT60 estimate from the comb feedback and the mean comb length
        const float meanDelay = 0.04478f;
        float fb = 0.5f + room * 0.48f;
        float dbLoop = -20.f * std::log10(std::max(fb, 0.001f));
        float rt60 = dbLoop > 1e-6f ? 60.f * meanDelay / dbLoop : 99.f;
        float cutoff = 20000.f - damp * 19900.f;

        // decay curve
        {
            nvgBeginPath(vg);
            nvgMoveTo(vg, FX, DY + DH - 1.f);
            for (int i = 0; i <= (int) FW; ++i) {
                float t = 8.f * i / FW;
                nvgLineTo(vg, FX + i, DY + DH - 1.f - std::exp(-6.9078f * t / rt60) * (DH - 2.f));
            }
            nvgLineTo(vg, FX + FW, DY + DH - 1.f);
            nvgClosePath(vg);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, DY, 0, DY + DH, nvgRGBA(0x3d, 0xe0, 0xff, 70), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
            nvgFill(vg);
            for (int pass = 0; pass < 2; ++pass) {
                nvgBeginPath(vg);
                for (int i = 0; i <= (int) FW; ++i) {
                    float t = 8.f * i / FW;
                    float y = DY + DH - 1.f - std::exp(-6.9078f * t / rt60) * (DH - 2.f);
                    if (i == 0) nvgMoveTo(vg, FX + i, y);
                    else nvgLineTo(vg, FX + i, y);
                }
                nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 50) : nvgRGB(0x8a, 0xee, 0xff));
                nvgStrokeWidth(vg, pass == 0 ? 3.5f : 1.3f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);
            }
            if (rt60 < 8.f) {
                float mx = FX + FW * rt60 / 8.f;
                nvgBeginPath(vg);
                for (float y = DY + 1.f; y < DY + DH - 1.f; y += 4.f) {
                    nvgMoveTo(vg, mx + 0.5f, y);
                    nvgLineTo(vg, mx + 0.5f, y + 2.f);
                }
                nvgStrokeColor(vg, withAlpha(colViolet(), 0.9f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
        }

        // damping response (one-pole, log 20 Hz..20 kHz, 0..-36 dB)
        {
            for (int pass = 0; pass < 2; ++pass) {
                nvgBeginPath(vg);
                for (int i = 0; i <= (int) FW; ++i) {
                    float f = 20.f * std::pow(1000.f, (float) i / FW);
                    float db = -10.f * std::log10(1.f + (f / cutoff) * (f / cutoff));
                    float t = std::min(std::max(-db / 36.f, 0.f), 1.f);
                    float y = PY + 1.f + t * (PH - 2.f);
                    if (i == 0) nvgMoveTo(vg, FX + i, y);
                    else nvgLineTo(vg, FX + i, y);
                }
                nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0xb0, 0x5a, 0xff, 60) : nvgRGB(0xd8, 0xb0, 0xff));
                nvgStrokeWidth(vg, pass == 0 ? 3.5f : 1.2f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);
            }
            float cxm = FX + FW * (std::log10(std::max(cutoff, 20.f) / 20.f) / 3.f);
            nvgBeginPath(vg);
            nvgRect(vg, cxm, PY, 1.f, PH);
            nvgFillColor(vg, withAlpha(colAccent(), 0.7f));
            nvgFill(vg);
        }

        // meters: in, L, R
        for (int k = 0; k < 3; ++k) {
            float v = k == 0 ? mIn : (k == 1 ? mL : mR);
            float lv = std::min(v / 5.f, 1.f);
            float x = 118.f + k * 8.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 16.f, 5.f, 60.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lv > 0.01f) {
                nvgBeginPath(vg);
                nvgRect(vg, x, 76.f - lv * 60.f, 5.f, lv * 60.f);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, 76.f, 0, 16.f, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }
        drawText(vg, 120.5f, 82.f, "I", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        drawText(vg, 128.5f, 82.f, "L", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        drawText(vg, 136.5f, 82.f, "R", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

        screenFinish(vg, W, H);

        char buf[48];
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
        snprintf(buf, sizeof(buf), "ROOM %3.0f%%", room * 100.f);
        drawText(vg, 6.f, 7.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "WET %3.0f%%", wet * 100.f);
        drawText(vg, W - 6.f, 7.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "RT60 %.1fS", rt60);
        drawText(vg, 6.f, 86.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "DRY %3.0f%%", dry * 100.f);
        drawText(vg, W - 6.f, 86.f, buf, 7.f, nvgRGBA(0x7a, 0x85, 0x94, 230), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct PsyReverbWidget : ModuleWidget {
    PsyReverbWidget(PsyReverb* module) {
        setModule(module);
        box.size = Vec(PR_W, PR_H);

        addChild(new PRPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PRDisplay(module));

        const int knobs[4] = {PsyReverb::PARAM_WET, PsyReverb::PARAM_DRY, PsyReverb::PARAM_ROOMSIZE, PsyReverb::PARAM_DAMPING};
        const int attens[4] = {PsyReverb::PARAM_ATTEN_WET, PsyReverb::PARAM_ATTEN_DRY, PsyReverb::PARAM_ATTEN_ROOMSIZE, PsyReverb::PARAM_ATTEN_DAMPING};
        const int cvs[4] = {PsyReverb::CV_WET, PsyReverb::CV_DRY, PsyReverb::CV_ROOMSIZE, PsyReverb::CV_DAMPING};
        for (int i = 0; i < 4; ++i) {
            float cy = PR_ROW[i] + PR_ROWH / 2.f;
            addInput(createInputCentered<DarkPJ301MPort>(Vec(PR_CVX, cy), module, cvs[i]));
            addParam(createParamCentered<Trimpot>(Vec(PR_ATX, cy), module, attens[i]));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PR_KX, cy), module, knobs[i]));
        }

        addInput(createInputCentered<DarkPJ301MPort>(Vec(prIoX(0), PR_IOY), module, PsyReverb::INPUT_LEFT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(prIoX(1), PR_IOY), module, PsyReverb::INPUT_RIGHT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(prIoX(2), PR_IOY), module, PsyReverb::OUTPUT_LEFT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(prIoX(3), PR_IOY), module, PsyReverb::OUTPUT_RIGHT));
    }
};

Model* modelPsyReverb = createModel<PsyReverb, PsyReverbWidget>("PsyReverb");