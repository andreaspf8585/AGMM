#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

static const float V2_W = RACK_GRID_WIDTH * 12;   // 180
static const float V2_H = RACK_GRID_HEIGHT;       // 380
static const float V2_DX = 20.f, V2_DY = 46.f, V2_DW = 140.f, V2_DH = 170.f;
static const float V2_CX[2] = {55.f, 125.f};
static const float V2_BW = 44.f, V2_BH = 110.f, V2_BY = 70.f;   // limit bar
static const float V2_ROW[3] = {254.f, 300.f, 346.f};

struct VCA2 : Module {
    enum ParamId { PARAM_LIMIT1, PARAM_LIMIT2, PARAMS_LEN };
    enum InputId { INPUT_AUDIO1, INPUT_AUDIO2, INPUT_ENV1, INPUT_ENV2, INPUTS_LEN };
    enum OutputId { OUTPUT_1, OUTPUT_2, OUTPUTS_LEN };
    enum LightId { LIGHTS_LEN };

    // Display data: envelope as % of the limit, and output peak
    float barFillPercent[2] = {0.f, 0.f};
    float outPeak[2] = {0.f, 0.f};
    bool envConn[2] = {false, false};

    VCA2() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PARAM_LIMIT1, 0.f, 2.f, 1.f, "Channel 1 gain limit", "x");
        configParam(PARAM_LIMIT2, 0.f, 2.f, 1.f, "Channel 2 gain limit", "x");

        configInput(INPUT_AUDIO1, "Channel 1 audio");
        configInput(INPUT_AUDIO2, "Channel 2 audio");
        configInput(INPUT_ENV1, "Channel 1 envelope CV (0 to 10 V)");
        configInput(INPUT_ENV2, "Channel 2 envelope CV (0 to 10 V)");

        configOutput(OUTPUT_1, "Channel 1");
        configOutput(OUTPUT_2, "Channel 2");
    }

    void process(const ProcessArgs& args) override {
        float md = 1.f - 5.f * args.sampleTime;
        for (int ch = 0; ch < 2; ++ch) {
            float in = inputs[INPUT_AUDIO1 + ch].getVoltage();
            bool conn = inputs[INPUT_ENV1 + ch].isConnected();
            float limit = params[PARAM_LIMIT1 + ch].getValue();
            float out, fill;

            if (!conn) {
                out = in;          // no envelope: unity gain, full bar
                fill = 100.f;
            } else {
                float env = inputs[INPUT_ENV1 + ch].getVoltage();
                float envNorm = clamp(env / 10.f, 0.f, 1.f);
                out = in * envNorm * limit;
                float limitV = limit * 10.f;
                fill = (limitV > 0.f) ? clamp(env / limitV, 0.f, 1.f) * 100.f : 0.f;
            }
            outputs[OUTPUT_1 + ch].setVoltage(out);
            barFillPercent[ch] = fill;
            envConn[ch] = conn;
            outPeak[ch] = std::max(std::fabs(out), outPeak[ch] * md);
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

struct V2Panel : TransparentWidget {
    V2Panel() { box.size = Vec(V2_W, V2_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, V2_W, V2_H, "VCA2");
        drawBezel(vg, V2_DX, V2_DY, V2_DW, V2_DH);
        const char* names[3] = {"AUDIO", "ENV", "OUT"};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 2; ++c)
                topCell(vg, V2_CX[c], V2_ROW[r], names[r]);
        Widget::draw(args);
    }
};

struct V2Display : TransparentWidget {
    V2Display() {
        box.pos = Vec(V2_DX, V2_DY);
        box.size = Vec(V2_DW, V2_DH);
    }
    void draw(const DrawArgs& args) override {
        screenBase(args.vg, V2_DW, V2_DH);
        drawText(args.vg, V2_CX[0] - V2_DX, 12.f, "CH 1", 7.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        drawText(args.vg, V2_CX[1] - V2_DX, 12.f, "CH 2", 7.f, colViolet(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        Widget::draw(args);
    }
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer == 1) screenFinish(args.vg, V2_DW, V2_DH);
        Widget::drawLayer(args, layer);
    }
};

// Interactive limit bar: click or drag to set the limit, double-click to reset to 1x
struct V2Bar : TransparentWidget {
    VCA2* module = nullptr;
    int ch = 0;
    float dragY = 0.f;

    V2Bar(VCA2* m, int c) : module(m), ch(c) {
        box.pos = Vec(V2_CX[c] - V2_BW / 2.f, V2_BY);
        box.size = Vec(V2_BW, V2_BH);
    }

    void apply() {
        if (!module) return;
        float v = (1.f - dragY / V2_BH) * 2.f;
        module->paramQuantities[VCA2::PARAM_LIMIT1 + ch]->setValue(clamp(v, 0.f, 2.f));
    }

    void onButton(const ButtonEvent& e) override {
        if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            dragY = clamp(e.pos.y, 0.f, V2_BH);
            apply();
            e.consume(this);
        }
    }

    void onDragMove(const DragMoveEvent& e) override {
        float z = getAbsoluteZoom();
        if (z <= 0.f) z = 1.f;
        dragY = clamp(dragY + e.mouseDelta.y / z, 0.f, V2_BH);
        apply();
    }

    void onDoubleClick(const DoubleClickEvent& e) override {
        if (module) module->paramQuantities[VCA2::PARAM_LIMIT1 + ch]->setValue(1.f);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        NVGcolor col = ch == 0 ? colAccent() : colViolet();

        float limit = module ? module->params[VCA2::PARAM_LIMIT1 + ch].getValue() : (ch == 0 ? 1.2f : 0.8f);
        float fillPct = module ? module->barFillPercent[ch] : (ch == 0 ? 70.f : 45.f);
        float peak = module ? module->outPeak[ch] : (ch == 0 ? 3.f : 2.f);
        bool conn = module ? module->envConn[ch] : true;

        float markerY = V2_BH * (1.f - limit / 2.f);
        float fillH = (fillPct / 100.f) * (V2_BH - markerY);

        // track
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0, 0, V2_BW, V2_BH, 3.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 120));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // ticks every 0.5x
        nvgBeginPath(vg);
        for (int k = 1; k < 4; ++k) {
            float y = V2_BH * (1.f - k * 0.5f / 2.f) + 0.5f;
            nvgMoveTo(vg, 0, y);
            nvgLineTo(vg, 6.f, y);
            nvgMoveTo(vg, V2_BW - 6.f, y);
            nvgLineTo(vg, V2_BW, y);
        }
        nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 70));
        nvgStroke(vg);

        // fill (never goes above the limit marker)
        if (fillH > 0.5f) {
            nvgBeginPath(vg);
            nvgRect(vg, 3.f, V2_BH - fillH, V2_BW - 6.f - 6.f, fillH);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, V2_BH, 0, markerY, withAlpha(col, 0.3f), withAlpha(col, 0.95f)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, 3.f, V2_BH - fillH, V2_BW - 12.f, 1.5f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 190));
            nvgFill(vg);
        }

        // output peak meter
        float pl = std::min(peak / 5.f, 1.f);
        nvgBeginPath(vg);
        nvgRect(vg, V2_BW - 8.f, 3.f, 5.f, V2_BH - 6.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
        nvgFill(vg);
        if (pl > 0.01f) {
            nvgBeginPath(vg);
            nvgRect(vg, V2_BW - 8.f, V2_BH - 3.f - pl * (V2_BH - 6.f), 5.f, pl * (V2_BH - 6.f));
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, V2_BH, 0, 0, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
            nvgFill(vg);
        }

        // limit marker + handle
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0, markerY);
        nvgLineTo(vg, V2_BW, markerY);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 240));
        nvgStrokeWidth(vg, 1.6f);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgMoveTo(vg, -5.f, markerY - 4.f);
        nvgLineTo(vg, 0.f, markerY);
        nvgLineTo(vg, -5.f, markerY + 4.f);
        nvgClosePath(vg);
        nvgMoveTo(vg, V2_BW + 5.f, markerY - 4.f);
        nvgLineTo(vg, V2_BW, markerY);
        nvgLineTo(vg, V2_BW + 5.f, markerY + 4.f);
        nvgClosePath(vg);
        nvgFillColor(vg, col);
        nvgFill(vg);

        // readouts
        char buf[32];
        snprintf(buf, sizeof(buf), "LIM %.2fX", limit);
        drawText(vg, V2_BW / 2.f, V2_BH + 10.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        if (conn) snprintf(buf, sizeof(buf), "ENV %.0f%%", fillPct);
        else snprintf(buf, sizeof(buf), "NO ENV");
        drawText(vg, V2_BW / 2.f, V2_BH + 21.f, buf, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct VCA2Widget : ModuleWidget {
    VCA2Widget(VCA2* module) {
        setModule(module);
        box.size = Vec(V2_W, V2_H);

        addChild(new V2Panel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new V2Display());
        addChild(new V2Bar(module, 0));
        addChild(new V2Bar(module, 1));

        for (int c = 0; c < 2; ++c) {
            addInput(createInputCentered<DarkPJ301MPort>(Vec(V2_CX[c], V2_ROW[0]), module, VCA2::INPUT_AUDIO1 + c));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(V2_CX[c], V2_ROW[1]), module, VCA2::INPUT_ENV1 + c));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(V2_CX[c], V2_ROW[2]), module, VCA2::OUTPUT_1 + c));
        }
    }
};

Model* modelVCA2 = createModel<VCA2, VCA2Widget>("VCA2");