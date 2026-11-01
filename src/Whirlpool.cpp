#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>

static const float WH_W = RACK_GRID_WIDTH * 20;   // 300
static const float WH_H = RACK_GRID_HEIGHT;       // 380
static const float WH_DX = 30.f, WH_DY = 46.f, WH_DW = 240.f, WH_DH = 90.f;
static const float WH_RA = 156.f, WH_RAH = 90.f;  // row A: 5 knobs with CV + attenuverter
static const float WH_RB = 256.f, WH_RBH = 50.f;  // row B: stereo switches + clear
static const float WH_JY = 346.f;                 // row C: jacks
static float whCol(int i) { return 38.f + 56.f * i; }

struct Whirlpool : Module {
    enum ParamId {
        REPLACE_PARAM,
        REPLACE_ATTEN,
        BRIGHTNESS_PARAM,
        BRIGHTNESS_ATTEN,
        DETUNE_PARAM,
        DETUNE_ATTEN,
        BIGNESS_PARAM,
        BIGNESS_ATTEN,
        MIX_PARAM,
        MIX_ATTEN,
        IN_STEREO_BUTTON_PARAM,
        OUT_STEREO_BUTTON_PARAM,
        CLEAR_BUTTON_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        IN_L_INPUT,
        IN_R_INPUT,
        REPLACE_CV_INPUT,
        BRIGHTNESS_CV_INPUT,
        DETUNE_CV_INPUT,
        BIGNESS_CV_INPUT,
        MIX_CV_INPUT,
        CLEAR_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        CLEAR_BUTTON_LIGHT,
        LIGHTS_LEN
    };

    struct Tank {
        std::vector<float> delayL[4];
        std::vector<float> delayR[4];
        size_t writeIdx[4] = {0, 0, 0, 0};
        float iirL = 0.0f;
        float iirR = 0.0f;
        float lfoPhase = 0.0f;

        void resize(int sizes[4]) {
            for (int i = 0; i < 4; ++i) {
                delayL[i].resize(sizes[i], 0.0f);
                delayR[i].resize(sizes[i], 0.0f);
                // a shrinking buffer must not leave the write index out of range
                if (writeIdx[i] >= (size_t) sizes[i]) writeIdx[i] = 0;
            }
        }

        void reset() {
            for (int i = 0; i < 4; ++i) {
                std::fill(delayL[i].begin(), delayL[i].end(), 0.0f);
                std::fill(delayR[i].begin(), delayR[i].end(), 0.0f);
                writeIdx[i] = 0;
            }
            iirL = iirR = 0.0f;
            lfoPhase = 0.0f;
        }
    };

    struct Channel {
        Tank tanks[4];
        void reset() {
            for (auto& t : tanks) t.reset();
        }
        void resize(int tankSizes[4][4]) {
            for (int i = 0; i < 4; ++i)
                tanks[i].resize(tankSizes[i]);
        }
    };

    static constexpr int MAX_CHANNELS = 16;
    Channel channels[MAX_CHANNELS];

    static constexpr int BASE_A = 4801;
    static constexpr int BASE_B = 3607;
    static constexpr int BASE_C = 2411;
    static constexpr int BASE_D = 1201;

    // Clear trigger handling
    dsp::SchmittTrigger clearButtonTrigger;
    dsp::SchmittTrigger clearInputTrigger;
    bool clearRequested = false;
    float clearFade = 1.0f;
    bool clearActive = false;

    int limiterRecovery = 0;
    bool stereoInput = false;
    bool stereoOutput = false;

    float lastBigness = -1.f;
    float lastSr = 0.f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float dispLevel[4] = {};
    float dispLfo[4] = {};
    float dispVal[5] = {};      // replace, brightness, detune, bigness, mix (with CV)
    float meterIn = 0.f, meterOutL = 0.f, meterOutR = 0.f;
    int dispCh = 1;

    Whirlpool() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(REPLACE_PARAM, 0.f, 1.f, 0.f, "Replace", "%", 0.f, 100.f);
        configParam(REPLACE_ATTEN, -1.f, 1.f, 0.f, "Replace CV attenuverter", "%", 0.f, 100.f);
        configParam(BRIGHTNESS_PARAM, 0.f, 1.f, 0.5f, "Brightness", "%", 0.f, 100.f);
        configParam(BRIGHTNESS_ATTEN, -1.f, 1.f, 0.f, "Brightness CV attenuverter", "%", 0.f, 100.f);
        configParam(DETUNE_PARAM, 0.f, 1.f, 0.2f, "Detune", "%", 0.f, 100.f);
        configParam(DETUNE_ATTEN, -1.f, 1.f, 0.f, "Detune CV attenuverter", "%", 0.f, 100.f);
        configParam(BIGNESS_PARAM, 0.f, 1.f, 0.6f, "Bigness", "%", 0.f, 100.f);
        configParam(BIGNESS_ATTEN, -1.f, 1.f, 0.f, "Bigness CV attenuverter", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/wet", "%", 0.f, 100.f);
        configParam(MIX_ATTEN, -1.f, 1.f, 0.f, "Mix CV attenuverter", "%", 0.f, 100.f);

        configSwitch(IN_STEREO_BUTTON_PARAM, 0.f, 1.f, 0.f, "Stereo input", {"Off (mono / poly)", "On"});
        configSwitch(OUT_STEREO_BUTTON_PARAM, 0.f, 1.f, 0.f, "Stereo output", {"Off (mono / poly)", "On"});
        configButton(CLEAR_BUTTON_PARAM, "Clear");

        configInput(IN_L_INPUT, "Left / poly");
        configInput(IN_R_INPUT, "Right");
        configInput(REPLACE_CV_INPUT, "Replace CV (10 V = 100%)");
        configInput(BRIGHTNESS_CV_INPUT, "Brightness CV (10 V = 100%)");
        configInput(DETUNE_CV_INPUT, "Detune CV (10 V = 100%)");
        configInput(BIGNESS_CV_INPUT, "Bigness CV (10 V = 100%)");
        configInput(MIX_CV_INPUT, "Mix CV (10 V = 100%)");
        configInput(CLEAR_INPUT, "Clear trigger");

        configOutput(OUT_L_OUTPUT, "Left / poly");
        configOutput(OUT_R_OUTPUT, "Right");
        configLight(CLEAR_BUTTON_LIGHT, "Clear");

        onReset();
    }

    void onReset() override {
        for (int c = 0; c < MAX_CHANNELS; ++c)
            channels[c].reset();
        clearRequested = false;
        clearFade = 1.0f;
        clearActive = false;
        limiterRecovery = 0;
        stereoInput = false;
        stereoOutput = false;
    }

    void resizeDelays(float bigness, float sampleRate) {
        float rateFactor = sampleRate / 44100.0f;
        float sizeScale = 0.2f + bigness * 1.2f;
        int newSizes[4][4];
        for (int t = 0; t < 4; ++t) {
            float tankScale = 1.0f - t * 0.15f;
            newSizes[t][0] = std::max(16, (int)(BASE_A * sizeScale * tankScale * rateFactor));
            newSizes[t][1] = std::max(16, (int)(BASE_B * sizeScale * tankScale * rateFactor));
            newSizes[t][2] = std::max(16, (int)(BASE_C * sizeScale * tankScale * rateFactor));
            newSizes[t][3] = std::max(16, (int)(BASE_D * sizeScale * tankScale * rateFactor));
        }
        for (int ch = 0; ch < MAX_CHANNELS; ++ch)
            channels[ch].resize(newSizes);
    }

    float getParam(int paramId, int attenId, int cvId) {
        float knob = params[paramId].getValue();
        float atten = params[attenId].getValue();
        float cv = inputs[cvId].getVoltageSum() / 10.f;
        float value = knob + atten * cv;
        return rack::math::clamp(value, 0.0f, 1.0f);
    }

    void process(const ProcessArgs& args) override {
        float sampleRate = args.sampleRate;
        float sampleTime = 1.0f / sampleRate;

        int inLch = inputs[IN_L_INPUT].getChannels();
        int inRch = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getChannels() : 0;
        bool useStereoIn = params[IN_STEREO_BUTTON_PARAM].getValue() > 0.5f;

        int numChannels = 1;
        if (useStereoIn) {
            numChannels = 2;
            stereoInput = true;
        } else {
            numChannels = std::max(inLch, inRch);
            numChannels = rack::math::clamp(numChannels, 1, MAX_CHANNELS);
            stereoInput = false;
        }
        dispCh = numChannels;

        bool useStereoOut = params[OUT_STEREO_BUTTON_PARAM].getValue() > 0.5f;
        if (useStereoOut) {
            outputs[OUT_L_OUTPUT].setChannels(2);
            outputs[OUT_R_OUTPUT].setChannels(2);
            stereoOutput = true;
        } else {
            outputs[OUT_L_OUTPUT].setChannels(numChannels);
            outputs[OUT_R_OUTPUT].setChannels(numChannels);
            stereoOutput = false;
        }

        // Read parameters (raw, no smoothing)
        float replace = getParam(REPLACE_PARAM, REPLACE_ATTEN, REPLACE_CV_INPUT);
        float brightness = getParam(BRIGHTNESS_PARAM, BRIGHTNESS_ATTEN, BRIGHTNESS_CV_INPUT);
        float detune = getParam(DETUNE_PARAM, DETUNE_ATTEN, DETUNE_CV_INPUT);
        float bigness = getParam(BIGNESS_PARAM, BIGNESS_ATTEN, BIGNESS_CV_INPUT);
        float mix = getParam(MIX_PARAM, MIX_ATTEN, MIX_CV_INPUT);
        dispVal[0] = replace; dispVal[1] = brightness; dispVal[2] = detune;
        dispVal[3] = bigness; dispVal[4] = mix;

        // Resize delays when bigness (or the sample rate) changes
        if (std::abs(bigness - lastBigness) > 0.01f || sampleRate != lastSr) {
            resizeDelays(bigness, sampleRate);
            lastBigness = bigness;
            lastSr = sampleRate;
        }

        // Clear trigger with fade
        bool clearPressed = params[CLEAR_BUTTON_PARAM].getValue() > 0.5f;
        bool clearTrig = clearButtonTrigger.process(clearPressed ? 1.f : 0.f) ||
                         clearInputTrigger.process(inputs[CLEAR_INPUT].getVoltage());
        if (clearTrig) clearRequested = true;

        if (clearRequested) {
            clearFade -= sampleTime * 10.f;
            if (clearFade <= 0.0f) {
                for (int i = 0; i < MAX_CHANNELS; ++i) channels[i].reset();
                clearFade = 0.0f;
                clearRequested = false;
                clearActive = true;
            }
        } else if (clearActive) {
            clearFade += sampleTime * 10.f;
            if (clearFade >= 1.0f) {
                clearFade = 1.0f;
                clearActive = false;
            }
        } else {
            clearFade = 1.0f;
        }
        lights[CLEAR_BUTTON_LIGHT].setSmoothBrightness(clearPressed ? 1.0f : 0.0f, sampleTime);

        // Algorithm parameters
        float feedback = 0.35f + (1.0f - replace) * 0.35f;
        float lowpassBase = 0.01f + brightness * 0.35f;
        lowpassBase = rack::math::clamp(lowpassBase, 0.001f, 0.99f);
        float lfoDepth = 4.0f + detune * 20.0f;
        float md = 1.f - 5.f * sampleTime;

        for (int c = 0; c < numChannels; ++c) {
            float dryL, dryR;
            if (useStereoIn) {
                if (c == 0) {
                    dryL = inputs[IN_L_INPUT].getVoltage();
                    dryR = inputs[IN_R_INPUT].getVoltage();
                } else {
                    dryL = inputs[IN_R_INPUT].getVoltage();
                    dryR = inputs[IN_L_INPUT].getVoltage();
                }
            } else {
                dryL = inputs[IN_L_INPUT].getPolyVoltage(c);
                dryR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getPolyVoltage(c) : dryL;
            }
            if (!std::isfinite(dryL)) dryL = 0.f;
            if (!std::isfinite(dryR)) dryR = 0.f;

            float currentInL = dryL;
            float currentInR = dryR;

            // 4 serial tanks
            for (int t = 0; t < 4; ++t) {
                Tank& tank = channels[c].tanks[t];

                // LFO with per-tank frequency variation
                float lfoSpeed = 0.1f + detune * (1.2f + t * 0.2f);
                tank.lfoPhase += lfoSpeed / sampleRate;
                if (tank.lfoPhase >= 1.0f) tank.lfoPhase -= 1.0f;
                float phase = tank.lfoPhase * 2.0f * M_PI;
                float modL = std::sin(phase) * lfoDepth;
                float modR = std::cos(phase) * lfoDepth;

                // Linear interpolated read from a delay line
                auto readDelay = [&](const std::vector<float>& buf, size_t idx, float offset) -> float {
                    int size = (int)buf.size();
                    if (size <= 1) return 0.f;
                    float pos = (float)idx - offset;
                    if (pos < 0) pos += size;
                    int i0 = (int)pos;
                    int i1 = (i0 + 1) % size;
                    float frac = pos - i0;
                    return buf[i0] * (1.f - frac) + buf[i1] * frac;
                };

                float offsetA = tank.delayL[0].size() * 0.35f;
                float offsetB = tank.delayL[1].size() * 0.40f;
                float offsetC = tank.delayL[2].size() * 0.50f;
                float offsetD = tank.delayL[3].size() * 0.60f;

                float outA_L = readDelay(tank.delayL[0], tank.writeIdx[0], offsetA + modL);
                float outA_R = readDelay(tank.delayR[0], tank.writeIdx[0], offsetA + modR);
                float outB_L = readDelay(tank.delayL[1], tank.writeIdx[1], offsetB - modR);
                float outB_R = readDelay(tank.delayR[1], tank.writeIdx[1], offsetB - modL);
                float outC_L = readDelay(tank.delayL[2], tank.writeIdx[2], offsetC);
                float outC_R = readDelay(tank.delayR[2], tank.writeIdx[2], offsetC);
                float outD_L = readDelay(tank.delayL[3], tank.writeIdx[3], offsetD);
                float outD_R = readDelay(tank.delayR[3], tank.writeIdx[3], offsetD);

                // Mix into tank
                float tankL = currentInL + (outA_L - outC_R) * feedback * 0.35f;
                float tankR = currentInR + (outA_R + outC_L) * feedback * 0.35f;
                tankL = rack::math::clamp(tankL, -12.f, 12.f);
                tankR = rack::math::clamp(tankR, -12.f, 12.f);

                // Write to A
                tank.delayL[0][tank.writeIdx[0]] = tankL;
                tank.delayR[0][tank.writeIdx[0]] = tankR;

                // Write to B
                float bL = rack::math::clamp(tankL - outB_R * feedback * 0.35f, -12.f, 12.f);
                float bR = rack::math::clamp(tankR + outB_L * feedback * 0.35f, -12.f, 12.f);
                tank.delayL[1][tank.writeIdx[1]] = bL;
                tank.delayR[1][tank.writeIdx[1]] = bR;

                // Write to C
                float cL = rack::math::clamp(outB_L * 0.35f + outD_R * 0.35f, -12.f, 12.f);
                float cR = rack::math::clamp(outB_R * 0.35f - outD_L * 0.35f, -12.f, 12.f);
                tank.delayL[2][tank.writeIdx[2]] = cL;
                tank.delayR[2][tank.writeIdx[2]] = cR;

                // Write to D
                tank.delayL[3][tank.writeIdx[3]] = outC_L;
                tank.delayR[3][tank.writeIdx[3]] = outC_R;

                // Advance pointers
                for (int i = 0; i < 4; ++i)
                    tank.writeIdx[i] = (tank.writeIdx[i] + 1) % tank.delayL[i].size();

                // Lowpass filter on the output of this tank
                float sumL = (outA_L + outB_L + outC_L) * 0.333f;
                float sumR = (outA_R + outB_R + outC_R) * 0.333f;
                tank.iirL += lowpassBase * (sumL - tank.iirL);
                tank.iirR += lowpassBase * (sumR - tank.iirR);
                if (!std::isfinite(tank.iirL)) tank.iirL = 0.f;
                if (!std::isfinite(tank.iirR)) tank.iirR = 0.f;

                currentInL = tank.iirL;
                currentInR = tank.iirR;

                if (c == 0) {
                    float lv = std::max(std::fabs(tank.iirL), std::fabs(tank.iirR));
                    dispLevel[t] = std::max(lv, dispLevel[t] * md);
                    dispLfo[t] = tank.lfoPhase;
                }
            }

            // Final mix
            float outL = (1.f - mix) * dryL + mix * currentInL;
            float outR = (1.f - mix) * dryR + mix * currentInR;
            outL *= clearFade;
            outR *= clearFade;
            outL = rack::math::clamp(outL, -12.f, 12.f);
            outR = rack::math::clamp(outR, -12.f, 12.f);

            // Safety limiter
            if (!std::isfinite(outL) || !std::isfinite(outR)) {
                outL = outR = 0.f;
                if (limiterRecovery == 0) {
                    for (int i = 0; i < MAX_CHANNELS; ++i) channels[i].reset();
                    limiterRecovery = (int)(sampleRate * 0.1f);
                }
            }
            if (limiterRecovery > 0) {
                outL = outR = 0.f;
                limiterRecovery--;
            }

            if (c == 0) {
                meterIn = std::max(0.5f * (std::fabs(dryL) + std::fabs(dryR)), meterIn * md);
                meterOutL = std::max(std::fabs(outL), meterOutL * md);
                meterOutR = std::max(std::fabs(outR), meterOutR * md);
            }

            // Output routing
            if (useStereoOut) {
                if (c == 0) {
                    outputs[OUT_L_OUTPUT].setVoltage(outL, 0);
                    outputs[OUT_R_OUTPUT].setVoltage(outR, 0);
                } else if (c == 1) {
                    outputs[OUT_L_OUTPUT].setVoltage(outR, 1);
                    outputs[OUT_R_OUTPUT].setVoltage(outL, 1);
                }
            } else {
                outputs[OUT_L_OUTPUT].setVoltage(outL, c);
                outputs[OUT_R_OUTPUT].setVoltage(outR, c);
            }
        }
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
struct TWHCyanLight : TBase {
    TWHCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using WHCyanLight = TWHCyanLight<>;

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

struct WHPanel : TransparentWidget {
    WHPanel() { box.size = Vec(WH_W, WH_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, WH_W, WH_H, "WHIRLPOOL");
        drawBezel(vg, WH_DX, WH_DY, WH_DW, WH_DH);

        const char* names[5] = {"REPLACE", "BRIGHT", "DETUNE", "BIGNESS", "MIX"};
        for (int i = 0; i < 5; ++i) {
            float cx = whCol(i);
            plate(vg, cx - 28.f, WH_RA, 56.f, WH_RAH);
            label(vg, cx, WH_RA + 9.f, names[i]);
            ring(vg, cx, WH_RA + 33.f, 16.f);
            label(vg, cx - 14.f, WH_RA + 80.f, "CV");
            label(vg, cx + 15.f, WH_RA + 80.f, "ATT");
        }

        const char* b[3] = {"STEREO IN", "STEREO OUT", "CLEAR"};
        for (int i = 0; i < 3; ++i) {
            float cx = whCol(i + 1);
            plate(vg, cx - 28.f, WH_RB, 56.f, WH_RBH);
            label(vg, cx, WH_RB + 41.f, b[i]);
        }

        const char* io[5] = {"L IN", "R IN", "CLR IN", "OUT L", "OUT R"};
        for (int i = 0; i < 5; ++i) topCell(vg, whCol(i), WH_JY, io[i]);
        Widget::draw(args);
    }
};

// Display: four tanks as concentric rings, each driven by its own LFO and level
struct WHDisplay : TransparentWidget {
    Whirlpool* module = nullptr;
    WHDisplay(Whirlpool* m) : module(m) {
        box.pos = Vec(WH_DX, WH_DY);
        box.size = Vec(WH_DW, WH_DH);
    }

    void draw(const DrawArgs& args) override {
        screenBase(args.vg, WH_DW, WH_DH);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = WH_DW, H = WH_DH;

        float lvl[4] = {1.6f, 1.1f, 0.7f, 0.4f};
        float lfo[4] = {0.10f, 0.35f, 0.60f, 0.85f};
        float pv[5] = {0.2f, 0.5f, 0.3f, 0.6f, 0.5f};
        float mIn = 2.f, mL = 2.6f, mR = 2.2f, fade = 1.f;
        int ch = 1;
        bool sIn = false, sOut = false;
        if (module) {
            for (int t = 0; t < 4; ++t) { lvl[t] = module->dispLevel[t]; lfo[t] = module->dispLfo[t]; }
            for (int i = 0; i < 5; ++i) pv[i] = module->dispVal[i];
            mIn = module->meterIn; mL = module->meterOutL; mR = module->meterOutR;
            fade = module->clearFade;
            ch = module->dispCh;
            sIn = module->params[Whirlpool::IN_STEREO_BUTTON_PARAM].getValue() > 0.5f;
            sOut = module->params[Whirlpool::OUT_STEREO_BUTTON_PARAM].getValue() > 0.5f;
        }

        // ---- vortex ----
        const float cx = 56.f, cy = 47.f;
        const float R = 18.f + 22.f * pv[3];
        for (int t = 0; t < 4; ++t) {
            float r = R * (1.f - 0.15f * t);
            float a = std::min(lvl[t] / 4.f, 1.f);
            NVGcolor col = nvgLerpRGBA(colAccent(), colViolet(), t / 3.f);

            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, r);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (int) (14 + 20 * a)));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            float a0 = lfo[t] * 2.f * (float) M_PI + t * 0.9f;
            float len = 1.0f + 3.0f * a;
            for (int pass = 0; pass < 2; ++pass) {
                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, r, a0, a0 + len, NVG_CW);
                nvgStrokeColor(vg, pass == 0 ? withAlpha(col, 0.18f + 0.25f * a) : withAlpha(nvgLerpRGBA(col, nvgRGB(255, 255, 255), 0.45f), 0.5f + 0.5f * a));
                nvgStrokeWidth(vg, pass == 0 ? 4.f : 1.4f);
                nvgLineCap(vg, NVG_ROUND);
                nvgStroke(vg);
            }
            float hx = cx + r * std::cos(a0 + len), hy = cy + r * std::sin(a0 + len);
            nvgBeginPath(vg);
            nvgCircle(vg, hx, hy, 3.5f + 2.f * a);
            nvgFillColor(vg, withAlpha(col, 0.25f + 0.3f * a));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, hx, hy, 1.7f);
            nvgFillColor(vg, nvgRGB(255, 255, 255));
            nvgFill(vg);
        }
        // centre: the drain
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, 3.f);
        nvgFillColor(vg, withAlpha(colAccent(), 0.35f + 0.5f * fade));
        nvgFill(vg);
        if (fade < 0.99f) {
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, 4.f + (1.f - fade) * 34.f);
            nvgStrokeColor(vg, withAlpha(colViolet(), 0.9f * (1.f - fade)));
            nvgStrokeWidth(vg, 1.5f);
            nvgStroke(vg);
        }

        // ---- parameter bars ----
        const char* nm[5] = {"REPL", "BRGT", "DTUN", "BIG", "MIX"};
        char buf[32];
        for (int i = 0; i < 5; ++i) {
            float y = 16.f + i * 11.f;
            drawText(vg, 112.f, y, nm[i], 6.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, 144.f, y - 2.f, 60.f, 4.f, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgFill(vg);
            float v = std::min(std::max(pv[i], 0.f), 1.f);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, 144.f, y - 2.f, std::max(60.f * v, 2.f), 4.f, 2.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 144.f, 0, 204.f, 0, colAccent(), colViolet()));
            nvgFill(vg);
            snprintf(buf, sizeof(buf), "%3.0f%%", v * 100.f);
            drawText(vg, 232.f, y, buf, 6.5f, nvgRGBA(0x3d, 0xe0, 0xff, 200), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
        }

        // ---- meters ----
        auto hbar = [&](float y, const char* txt, float v) {
            drawText(vg, 112.f, y, txt, 6.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            nvgBeginPath(vg);
            nvgRect(vg, 132.f, y - 2.f, 68.f, 4.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            float lv = std::min(v / 5.f, 1.f);
            if (lv > 0.01f) {
                nvgBeginPath(vg);
                nvgRect(vg, 132.f, y - 2.f, 68.f * lv, 4.f);
                nvgFillPaint(vg, nvgLinearGradient(vg, 132.f, 0, 200.f, 0, colAccent(), colViolet()));
                nvgFill(vg);
            }
        };
        hbar(72.f, "IN", mIn);
        hbar(81.f, "OUT", 0.5f * (mL + mR));

        // ---- chips ----
        auto chip = [&](float x, float y, const char* txt, bool on) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, y, 30.f, 10.f, 3.f);
            if (on) {
                nvgFillColor(vg, withAlpha(colAccent(), 0.25f));
                nvgFill(vg);
                nvgStrokeColor(vg, withAlpha(colAccent(), 0.9f));
            } else {
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            }
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, x + 15.f, y + 5.f, txt, 6.f, on ? colAccent() : colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        };
        chip(206.f, 67.f, "ST IN", sIn);
        chip(206.f, 78.f, "ST OUT", sOut);

        screenFinish(vg, W, H);

        snprintf(buf, sizeof(buf), "CH %d", ch);
        drawText(vg, 6.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        if (fade < 0.99f)
            drawText(vg, W - 6.f, 7.f, "CLEARING", 7.f, withAlpha(colViolet(), 0.95f), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct WhirlpoolWidget : ModuleWidget {
    WhirlpoolWidget(Whirlpool* module) {
        setModule(module);
        box.size = Vec(WH_W, WH_H);

        addChild(new WHPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new WHDisplay(module));

        // ---- row A: knob on top, CV jack (left) and attenuverter (right) below ----
        for (int i = 0; i < 5; ++i) {
            float cx = whCol(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, WH_RA + 33.f), module, Whirlpool::REPLACE_PARAM + 2 * i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 14.f, WH_RA + 64.f), module, Whirlpool::REPLACE_CV_INPUT + i));
            addParam(createParamCentered<Trimpot>(Vec(cx + 15.f, WH_RA + 64.f), module, Whirlpool::REPLACE_ATTEN + 2 * i));
        }

        // ---- row B ----
        addParam(createParamCentered<CKSS>(Vec(whCol(1), WH_RB + 21.f), module, Whirlpool::IN_STEREO_BUTTON_PARAM));
        addParam(createParamCentered<CKSS>(Vec(whCol(2), WH_RB + 21.f), module, Whirlpool::OUT_STEREO_BUTTON_PARAM));
        addParam(createParamCentered<VCVButton>(Vec(whCol(3) - 3.f, WH_RB + 21.f), module, Whirlpool::CLEAR_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<WHCyanLight>>(Vec(whCol(3) + 16.f, WH_RB + 12.f), module, Whirlpool::CLEAR_BUTTON_LIGHT));

        // ---- row C ----
        addInput(createInputCentered<DarkPJ301MPort>(Vec(whCol(0), WH_JY), module, Whirlpool::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(whCol(1), WH_JY), module, Whirlpool::IN_R_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(whCol(2), WH_JY), module, Whirlpool::CLEAR_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(whCol(3), WH_JY), module, Whirlpool::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(whCol(4), WH_JY), module, Whirlpool::OUT_R_OUTPUT));
    }
};

Model* modelWhirlpool = createModel<Whirlpool, WhirlpoolWidget>("Whirlpool");