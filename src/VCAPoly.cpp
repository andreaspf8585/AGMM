#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

static const float VP_W = RACK_GRID_WIDTH * 14;   // 210 (wider for extra input)
static const float VP_H = RACK_GRID_HEIGHT;       // 380
static const float VP_DX = 20.f, VP_DY = 46.f, VP_DW = 140.f, VP_DH = 200.f;
static const float VP_BX0 = 26.f, VP_BW = 106.f;  // bar area inside the display
static const float VP_ROW0 = 24.f, VP_PITCH = 10.f;
static const float VP_KNOBY = 288.f;
static const float VP_JY = 348.f;
static float vpJx(int i) { return 30.f + 50.f * i; }

struct VCAPoly : Module {
    enum ParamId { PARAM_LIMIT, PARAMS_LEN };
    enum InputId { INPUT_AUDIO, INPUT_ENV, INPUT_LIMIT_CV, INPUTS_LEN };
    enum OutputId { OUTPUT_AUDIO, OUTPUTS_LEN };
    enum LightId { LIGHTS_LEN };

    // Meter: fill ratio (0..1) per voice, relative to limit*10V
    float envelopeFill[16] = {};

    // Display data (written by the audio thread, read by the UI thread)
    float gainNow[16] = {};
    float outPeak[16] = {};
    int activeCh = 0;
    bool envConn = false;
    bool limitCvConn = false;

    VCAPoly() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PARAM_LIMIT, 0.f, 2.f, 1.f, "Global gain limit", "x");
        configInput(INPUT_AUDIO, "Polyphonic audio");
        configInput(INPUT_ENV, "Polyphonic envelope CV (0 to 10 V)");
        configInput(INPUT_LIMIT_CV, "Polyphonic gain limit CV (0 to 10 V)");
        configOutput(OUTPUT_AUDIO, "Polyphonic output");
    }

    void process(const ProcessArgs& args) override {
        int audioChannels = inputs[INPUT_AUDIO].getChannels();
        activeCh = audioChannels;
        if (audioChannels == 0) {
            for (int c = 0; c < 16; ++c) { envelopeFill[c] = 0.f; gainNow[c] = 0.f; outPeak[c] = 0.f; }
            return;
        }

        bool envConnected = inputs[INPUT_ENV].isConnected();
        envConn = envConnected;
        int envChannels = envConnected ? inputs[INPUT_ENV].getChannels() : 0;

        bool limitCvConnected = inputs[INPUT_LIMIT_CV].isConnected();
        limitCvConn = limitCvConnected;
        int limitCvChannels = limitCvConnected ? inputs[INPUT_LIMIT_CV].getChannels() : 0;

        int channels = audioChannels;  // output always follows audio channel count

        outputs[OUTPUT_AUDIO].setChannels(channels);

        float globalLimit = params[PARAM_LIMIT].getValue();
        float md = 1.f - 5.f * args.sampleTime;

        for (int c = 0; c < channels; ++c) {
            float audio = inputs[INPUT_AUDIO].getVoltage(c);
            float out, fill, g;

            // Per-channel limit: global knob value, overridden by limit CV if connected
            float limit = globalLimit;
            if (limitCvConnected) {
                float limitCv = (c < limitCvChannels) ? inputs[INPUT_LIMIT_CV].getVoltage(c) : 0.f;
                limit = clamp(limitCv / 10.f, 0.f, 2.f);
            }
            float limitVoltage = limit * 10.f;

            if (!envConnected) {
                out = audio;       // no envelope cable: unity gain
                fill = 1.0f;
                g = 1.f;
            } else {
                float envVoltage = (c < envChannels) ? inputs[INPUT_ENV].getVoltage(c) : 0.f;
                float envNorm = clamp(envVoltage / 10.f, 0.f, 1.f);
                g = envNorm * limit;
                out = audio * g;
                fill = (limitVoltage > 0.f) ? clamp(envVoltage / limitVoltage, 0.f, 1.f) : 0.f;
            }
            outputs[OUTPUT_AUDIO].setVoltage(out, c);
            envelopeFill[c] = fill;
            gainNow[c] = g;
            outPeak[c] = std::max(std::fabs(out), outPeak[c] * md);
        }

        for (int c = channels; c < 16; ++c) { envelopeFill[c] = 0.f; gainNow[c] = 0.f; outPeak[c] = 0.f; }
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

struct VPPanel : TransparentWidget {
    VPPanel() { box.size = Vec(VP_W, VP_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, VP_W, VP_H, "VCAPOLY");
        drawBezel(vg, VP_DX, VP_DY, VP_DW, VP_DH);
        ring(vg, VP_W / 2.f, VP_KNOBY, 18.f);
        label(vg, VP_W / 2.f, VP_KNOBY + 25.f, "GLOBAL LIMIT");
        const char* names[4] = {"AUDIO", "ENV", "LIMIT", "OUT"};
        for (int i = 0; i < 4; ++i) topCell(vg, vpJx(i), VP_JY, names[i]);
        Widget::draw(args);
    }
};

// 16 voice rows: gain bar (0..2x) with the limit marker, plus a thin output meter
struct VPDisplay : TransparentWidget {
    VCAPoly* module = nullptr;
    VPDisplay(VCAPoly* m) : module(m) {
        box.pos = Vec(VP_DX, VP_DY);
        box.size = Vec(VP_DW, VP_DH);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        screenBase(vg, VP_DW, VP_DH);

        // scale ticks at 0, 1x and 2x
        const char* sc[3] = {"0", "1", "2"};
        nvgBeginPath(vg);
        for (int k = 0; k < 3; ++k) {
            float x = VP_BX0 + VP_BW * k / 2.f + 0.5f;
            nvgMoveTo(vg, x, 17.f);
            nvgLineTo(vg, x, VP_ROW0 + 16 * VP_PITCH);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        for (int k = 0; k < 3; ++k)
            drawText(vg, VP_BX0 + VP_BW * k / 2.f, 11.f, sc[k], 6.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

        for (int c = 0; c < 16; ++c) {
            float y = VP_ROW0 + c * VP_PITCH;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, VP_BX0, y, VP_BW, 6.f, 1.5f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 9));
            nvgFill(vg);
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", c + 1);
            drawText(vg, 10.f, y + 3.f, buf, 6.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        }
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;

        float globalLimit = 1.f;
        int n = 8;
        bool conn = true;
        bool limitCvConn = false;
        float gain[16], pk[16];
        float perChLimit[16];
        if (module) {
            globalLimit = module->params[VCAPoly::PARAM_LIMIT].getValue();
            n = module->activeCh;
            conn = module->envConn;
            limitCvConn = module->limitCvConn;
            for (int c = 0; c < 16; ++c) { gain[c] = module->gainNow[c]; pk[c] = module->outPeak[c]; }
            // compute per-channel limits for display
            int limitCvChannels = limitCvConn ? module->inputs[VCAPoly::INPUT_LIMIT_CV].getChannels() : 0;
            for (int c = 0; c < 16; ++c) {
                float lim = globalLimit;
                if (limitCvConn) {
                    float limitCv = (c < limitCvChannels) ? module->inputs[VCAPoly::INPUT_LIMIT_CV].getVoltage(c) : 0.f;
                    lim = clamp(limitCv / 10.f, 0.f, 2.f);
                }
                perChLimit[c] = lim;
            }
        } else {
            for (int c = 0; c < 16; ++c) {
                gain[c] = std::max(0.f, globalLimit * (0.5f + 0.5f * std::sin(c * 0.7f + 0.3f)));
                pk[c] = 4.f * gain[c] * 0.5f;
                perChLimit[c] = globalLimit;
            }
        }
        n = std::min(std::max(n, 0), 16);

        for (int c = 0; c < 16; ++c) {
            float y = VP_ROW0 + c * VP_PITCH;
            if (c >= n) {
                nvgBeginPath(vg);
                nvgRect(vg, VP_BX0, y + 2.5f, 3.f, 1.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 28));
                nvgFill(vg);
                continue;
            }
            float t = (float) c / 15.f;
            NVGcolor col = nvgHSL(0.5f + 0.12f * t, 0.9f, 0.6f);
            float w = std::min(gain[c] / 2.f, 1.f) * VP_BW;
            if (w > 0.5f) {
                nvgBeginPath(vg);
                nvgRect(vg, VP_BX0 - 1.f, y - 1.5f, w + 2.f, 9.f);
                nvgFillColor(vg, withAlpha(col, 0.14f));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, VP_BX0, y, w, 6.f, 1.5f);
                nvgFillPaint(vg, nvgLinearGradient(vg, VP_BX0, 0, VP_BX0 + VP_BW, 0, withAlpha(col, 0.45f), withAlpha(col, 1.f)));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRect(vg, VP_BX0 + w - 1.5f, y, 1.5f, 6.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 190));
                nvgFill(vg);
            }
            // output peak, a thin line under the bar
            float pw = std::min(pk[c] / 5.f, 1.f) * VP_BW;
            if (pw > 0.5f) {
                nvgBeginPath(vg);
                nvgRect(vg, VP_BX0, y + 6.5f, pw, 1.5f);
                nvgFillColor(vg, withAlpha(colViolet(), 0.9f));
                nvgFill(vg);
            }
        }

        // limit markers (per-channel)
        if (conn) {
            if (limitCvConn) {
                // draw per-channel limit markers
                for (int c = 0; c < n; ++c) {
                    float y = VP_ROW0 + c * VP_PITCH;
                    float lx = VP_BX0 + VP_BW * (perChLimit[c] / 2.f);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, lx, y - 1.f);
                    nvgLineTo(vg, lx, y + 7.f);
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 180));
                    nvgStrokeWidth(vg, 1.2f);
                    nvgStroke(vg);
                }
            } else {
                // single global limit marker
                float lx = VP_BX0 + VP_BW * (globalLimit / 2.f);
                nvgBeginPath(vg);
                nvgMoveTo(vg, lx, 16.f);
                nvgLineTo(vg, lx, VP_ROW0 + 16 * VP_PITCH - 2.f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 230));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, lx - 3.5f, 14.f);
                nvgLineTo(vg, lx + 3.5f, 14.f);
                nvgLineTo(vg, lx, 19.f);
                nvgClosePath(vg);
                nvgFillColor(vg, nvgRGB(255, 255, 255));
                nvgFill(vg);
            }
        }

        screenFinish(vg, VP_DW, VP_DH);

        char buf[40];
        snprintf(buf, sizeof(buf), "GLOBAL %.2fX", globalLimit);
        drawText(vg, 8.f, VP_DH - 9.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        if (limitCvConn) {
            snprintf(buf, sizeof(buf), "CH %d LIMIT CV", n);
            drawText(vg, VP_DW - 8.f, VP_DH - 9.f, buf, 7.f, nvgRGBA(0xff, 0xa0, 0x40, 210), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        } else {
            snprintf(buf, sizeof(buf), conn ? "CH %d" : "CH %d UNITY", n);
            drawText(vg, VP_DW - 8.f, VP_DH - 9.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        }
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct VCAPolyWidget : ModuleWidget {
    VCAPolyWidget(VCAPoly* module) {
        setModule(module);
        box.size = Vec(VP_W, VP_H);

        addChild(new VPPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new VPDisplay(module));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(VP_W / 2.f, VP_KNOBY), module, VCAPoly::PARAM_LIMIT));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(vpJx(0), VP_JY), module, VCAPoly::INPUT_AUDIO));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vpJx(1), VP_JY), module, VCAPoly::INPUT_ENV));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vpJx(2), VP_JY), module, VCAPoly::INPUT_LIMIT_CV));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(vpJx(3), VP_JY), module, VCAPoly::OUTPUT_AUDIO));
    }
};

Model* modelVCAPoly = createModel<VCAPoly, VCAPolyWidget>("VCAPoly");