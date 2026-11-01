#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

static const float SW_W = RACK_GRID_WIDTH * 12;   // 180
static const float SW_H = RACK_GRID_HEIGHT;       // 380
static const float SW_DX = 20.f, SW_DY = 46.f, SW_DW = 140.f, SW_DH = 84.f;
static const float SW_ROW[2] = {146.f, 226.f};    // top of each amount row
static const float SW_ROWH = 72.f;
static const float SW_COL[3] = {40.f, 90.f, 140.f};
static const float SW_IOY = 338.f;
static float swIoX(int i) { return 27.f + 42.f * i; }

struct Swapper : Module {
    enum ParamId { LTO_R_KNOB, RTO_L_KNOB, LTO_R_ATTEN, RTO_L_ATTEN, PARAMS_LEN };
    enum InputId { L_IN, R_IN, CV_LTOR, CV_RTOL, INPUTS_LEN };
    enum OutputId { L_OUT, R_OUT, OUTPUTS_LEN };
    enum LightId { LIGHTS_LEN };

    // Display data (written by the audio thread, read by the UI thread)
    float amtLR = 0.f, amtRL = 0.f;
    float mInL = 0.f, mInR = 0.f, mOutL = 0.f, mOutR = 0.f;

    Swapper() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(LTO_R_KNOB, 0.f, 1.f, 0.f, "L to R amount", "%", 0.f, 100.f);
        configParam(RTO_L_KNOB, 0.f, 1.f, 0.f, "R to L amount", "%", 0.f, 100.f);
        configParam(LTO_R_ATTEN, -1.f, 1.f, 0.f, "L to R CV attenuverter", "%", 0.f, 100.f);
        configParam(RTO_L_ATTEN, -1.f, 1.f, 0.f, "R to L CV attenuverter", "%", 0.f, 100.f);

        configInput(L_IN, "Left");
        configInput(R_IN, "Right");
        configInput(CV_LTOR, "CV for L to R (10 V = 100%)");
        configInput(CV_RTOL, "CV for R to L (10 V = 100%)");
        configOutput(L_OUT, "Left");
        configOutput(R_OUT, "Right");
    }

    void process(const ProcessArgs& args) override {
        float lIn = inputs[L_IN].getVoltage();
        float rIn = inputs[R_IN].getVoltage();

        float ltoR_manual = params[LTO_R_KNOB].getValue();
        float rtoL_manual = params[RTO_L_KNOB].getValue();

        float ltoR_cv = inputs[CV_LTOR].getVoltage() / 10.f;
        float rtoL_cv = inputs[CV_RTOL].getVoltage() / 10.f;
        float ltoR_atten = params[LTO_R_ATTEN].getValue();
        float rtoL_atten = params[RTO_L_ATTEN].getValue();

        float ltoR_amount = clamp(ltoR_manual + ltoR_atten * ltoR_cv, 0.f, 1.f);
        float rtoL_amount = clamp(rtoL_manual + rtoL_atten * rtoL_cv, 0.f, 1.f);

        float lOut = (1.f - rtoL_amount) * lIn + rtoL_amount * rIn;
        float rOut = (1.f - ltoR_amount) * rIn + ltoR_amount * lIn;

        outputs[L_OUT].setVoltage(lOut);
        outputs[R_OUT].setVoltage(rOut);

        amtLR = ltoR_amount;
        amtRL = rtoL_amount;
        float md = 1.f - 5.f * args.sampleTime;
        mInL = std::max(std::fabs(lIn), mInL * md);
        mInR = std::max(std::fabs(rIn), mInR * md);
        mOutL = std::max(std::fabs(lOut), mOutL * md);
        mOutR = std::max(std::fabs(rOut), mOutR * md);
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

// Brushed panel, edge highlight, logo and title
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

struct SWPanel : TransparentWidget {
    SWPanel() { box.size = Vec(SW_W, SW_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, SW_W, SW_H, "SWAPPER");
        drawBezel(vg, SW_DX, SW_DY, SW_DW, SW_DH);

        const char* names[2] = {"L TO R", "R TO L"};
        for (int r = 0; r < 2; ++r) {
            float top = SW_ROW[r];
            plate(vg, 14.f, top, 152.f, SW_ROWH);
            drawText(vg, 22.f, top + 9.f, names[r], 8.f, r == 0 ? colAccent() : colViolet(),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 2.f);
            ring(vg, SW_COL[0], top + 38.f, 17.f);
            label(vg, SW_COL[0], top + 63.f, "AMOUNT");
            label(vg, SW_COL[1], top + 63.f, "ATT");
            label(vg, SW_COL[2], top + 63.f, "CV");
        }

        const char* io[4] = {"L IN", "R IN", "L OUT", "R OUT"};
        for (int i = 0; i < 4; ++i) topCell(vg, swIoX(i), SW_IOY, io[i]);
        Widget::draw(args);
    }
};

// Routing diagram: line weight shows how much of each input reaches each output
struct SWDisplay : TransparentWidget {
    Swapper* module = nullptr;
    SWDisplay(Swapper* m) : module(m) {
        box.pos = Vec(SW_DX, SW_DY);
        box.size = Vec(SW_DW, SW_DH);
    }

    void draw(const DrawArgs& args) override {
        screenBase(args.vg, SW_DW, SW_DH);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = SW_DW, H = SW_DH;

        float lr = 0.35f, rl = 0.6f, iL = 2.f, iR = 3.f, oL = 3.f, oR = 2.5f;
        if (module) {
            lr = module->amtLR; rl = module->amtRL;
            iL = module->mInL; iR = module->mInR; oL = module->mOutL; oR = module->mOutR;
        }

        const float xi = 30.f, xo = W - 30.f, yL = 32.f, yR = 62.f;

        // source L (cyan): IN L -> OUT L (1 - rl is NOT this; it is R into L) and IN L -> OUT R
        struct Link { float x0, y0, x1, y1, w; bool fromL; };
        Link links[4] = {
            {xi, yL, xo, yL, 1.f - rl, true},     // IN L -> OUT L
            {xi, yR, xo, yL, rl, false},          // IN R -> OUT L
            {xi, yR, xo, yR, 1.f - lr, false},    // IN R -> OUT R
            {xi, yL, xo, yR, lr, true},           // IN L -> OUT R
        };
        for (const Link& k : links) {
            NVGcolor c = k.fromL ? colAccent() : colViolet();
            for (int pass = 0; pass < 2; ++pass) {
                nvgBeginPath(vg);
                nvgMoveTo(vg, k.x0, k.y0);
                nvgLineTo(vg, k.x1, k.y1);
                nvgStrokeColor(vg, pass == 0 ? withAlpha(c, 0.10f + 0.25f * k.w) : withAlpha(c, 0.12f + 0.88f * k.w));
                nvgStrokeWidth(vg, pass == 0 ? 2.f + 5.f * k.w : 0.8f + 2.f * k.w);
                nvgLineCap(vg, NVG_ROUND);
                nvgStroke(vg);
            }
        }

        // nodes
        auto node = [&](float x, float y, float v, NVGcolor c) {
            float a = std::min(v / 5.f, 1.f);
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, 6.f + 4.f * a);
            nvgFillColor(vg, withAlpha(c, 0.12f + 0.3f * a));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, 3.f);
            nvgFillColor(vg, withAlpha(c, 0.6f + 0.4f * a));
            nvgFill(vg);
        };
        node(xi, yL, iL, colAccent());
        node(xi, yR, iR, colViolet());
        node(xo, yL, oL, colAccent());
        node(xo, yR, oR, colViolet());
        drawText(vg, 10.f, yL, "L", 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        drawText(vg, 10.f, yR, "R", 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        drawText(vg, W - 10.f, yL, "L", 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        drawText(vg, W - 10.f, yR, "R", 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

        screenFinish(vg, W, H);

        char buf[32];
        snprintf(buf, sizeof(buf), "L>R %3.0f%%", lr * 100.f);
        drawText(vg, 6.f, 8.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "R>L %3.0f%%", rl * 100.f);
        drawText(vg, W - 6.f, 8.f, buf, 7.f, nvgRGBA(0xb0, 0x5a, 0xff, 230), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, W / 2.f, H - 7.f, "IN", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct SwapperWidget : ModuleWidget {
    SwapperWidget(Swapper* module) {
        setModule(module);
        box.size = Vec(SW_W, SW_H);

        addChild(new SWPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(new SWDisplay(module));

        const int knob[2] = {Swapper::LTO_R_KNOB, Swapper::RTO_L_KNOB};
        const int att[2] = {Swapper::LTO_R_ATTEN, Swapper::RTO_L_ATTEN};
        const int cv[2] = {Swapper::CV_LTOR, Swapper::CV_RTOL};
        for (int r = 0; r < 2; ++r) {
            float top = SW_ROW[r];
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SW_COL[0], top + 38.f), module, knob[r]));
            addParam(createParamCentered<Trimpot>(Vec(SW_COL[1], top + 38.f), module, att[r]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(SW_COL[2], top + 38.f), module, cv[r]));
        }

        addInput(createInputCentered<DarkPJ301MPort>(Vec(swIoX(0), SW_IOY), module, Swapper::L_IN));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(swIoX(1), SW_IOY), module, Swapper::R_IN));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(swIoX(2), SW_IOY), module, Swapper::L_OUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(swIoX(3), SW_IOY), module, Swapper::R_OUT));
    }
};

Model* modelSwapper = createModel<Swapper, SwapperWidget>("Swapper");