#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float SL_PANEL_W = RACK_GRID_WIDTH * 12;   // 180
static const float SL_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Channel display (same as Merger)
static const float SL_DISP_W = 132.f;
static const float SL_DISP_H = 52.f;
static const float SL_DISP_X = (SL_PANEL_W - SL_DISP_W) / 2.f;   // 24
static const float SL_DISP_Y = 48.f;

// Polyphonic input
static const float SL_SECTION_IN_Y = 116.f;
static const float SL_IN_X         = SL_PANEL_W / 2.f;
static const float SL_IN_Y         = 146.f;

// Output grid (4 x 4)
static const float SL_SECTION_OUT_Y = 180.f;
static const float SL_GRID_X[4]     = {30.f, 70.f, 110.f, 150.f};
static const float SL_GRID_Y[4]     = {213.f, 254.f, 295.f, 336.f};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Splitter : Module {
    enum ParamId {
        PARAMS_LEN
    };
    enum InputId {
        POLY_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        ENUMS(SPLIT_OUTPUT, 16),
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    // Shown on the display
    int channels = 0;

    Splitter() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configInput(POLY_INPUT, "Polyphonic");
        for (int i = 0; i < 16; ++i)
            configOutput(SPLIT_OUTPUT + i, string::f("Channel %d", i + 1));
    }

    void process(const ProcessArgs& args) override {
        int ch = inputs[POLY_INPUT].isConnected() ? inputs[POLY_INPUT].getChannels() : 0;
        channels = ch;

        for (int i = 0; i < 16; ++i) {
            outputs[SPLIT_OUTPUT + i].setChannels(1);
            outputs[SPLIT_OUTPUT + i].setVoltage(i < ch ? inputs[POLY_INPUT].getVoltage(i) : 0.f);
        }
    }
};

// ------------------------------------------------------------
// Theme + drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x25, 0x28, 0x30); }
NVGcolor colBgBottom() { return nvgRGB(0x13, 0x15, 0x19); }
NVGcolor colText()     { return nvgRGB(0xd6, 0xdc, 0xe6); }
NVGcolor colDim()      { return nvgRGB(0x8a, 0x95, 0xa5); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }

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
    nvgRect(vg, tx + s,       ty,            s, s);
    nvgRect(vg, tx,           ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);

    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
             NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct SLPanelBackground : TransparentWidget {
    SLPanelBackground() { box.size = Vec(SL_PANEL_W, SL_PANEL_H); }

    void label(NVGcontext* vg, float x, float y, const char* txt) {
        drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.0f);
    }

    void plate(NVGcontext* vg, float x, float y, float w, float h) {
        // soft drop
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 1.f, y - 1.f, w + 2.f, h + 3.f, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 6.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                           nvgRGBA(0, 0, 0, 105), nvgRGBA(0, 0, 0, 60)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 12.f, x2 = SL_PANEL_W - 12.f, cx = SL_PANEL_W / 2.f;
        float w = 14.f + (float) strlen(txt) * 7.f;
        drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - w / 2.f, y);
        nvgStrokePaint(vg, nvgLinearGradient(vg, x1, y, cx - w / 2.f, y,
                                             nvgRGBA(0x3d, 0xe0, 0xff, 0), nvgRGBA(0x3d, 0xe0, 0xff, 100)));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + w / 2.f, y);
        nvgLineTo(vg, x2, y);
        nvgStrokePaint(vg, nvgLinearGradient(vg, cx + w / 2.f, y, x2, y,
                                             nvgRGBA(0x3d, 0xe0, 0xff, 100), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Jack cell: plate, colour bar (orange = input, cyan = output), label
    void jackCell(NVGcontext* vg, float cx, float cy, float w, const char* txt, bool isOutput) {
        float x = cx - w / 2.f, y = cy - 21.f, h = 38.f;
        plate(vg, x, y, w, h);

        NVGcolor c = isOutput ? nvgRGBA(0x3d, 0xe0, 0xff, 170) : nvgRGBA(0xff, 0x9c, 0x6a, 170);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x + 6.f, y + 1.5f, w - 12.f, 1.8f, 0.9f);
        nvgFillColor(vg, c);
        nvgFill(vg);

        label(vg, cx, cy - 14.f, txt);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SL_PANEL_W, SL_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SL_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) SL_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, SL_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Corner vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SL_PANEL_W, SL_PANEL_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, SL_PANEL_W / 2.f, SL_PANEL_H * 0.4f,
                                           SL_PANEL_H * 0.30f, SL_PANEL_H * 0.80f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
        nvgFill(vg);

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, SL_PANEL_W - 1.f, SL_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Top accent line
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SL_PANEL_W, 2.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, SL_PANEL_W, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // Logo + title
        drawLogo(vg, SL_PANEL_W / 2.f, 15.f);
        drawText(vg, SL_PANEL_W / 2.f, 32.f, "SPLITTER", 12.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // ---- Display bezel (metallic, recessed) ----
        float bx = SL_DISP_X - 5.f, by = SL_DISP_Y - 5.f;
        float bw = SL_DISP_W + 10.f, bh = SL_DISP_H + 10.f;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 4.f, by - 3.f, bw + 8.f, bh + 9.f, 10.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x4a, 0x50, 0x5a), nvgRGB(0x14, 0x16, 0x1a)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 2.f, by - 2.f, bw + 4.f, bh + 4.f, 7.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 36));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- Input ----
        section(vg, SL_SECTION_IN_Y, "INPUT");
        jackCell(vg, SL_IN_X, SL_IN_Y, 58.f, "POLY IN", false);

        // ---- Outputs ----
        section(vg, SL_SECTION_OUT_Y, "OUTPUTS");
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", r * 4 + c + 1);
                jackCell(vg, SL_GRID_X[c], SL_GRID_Y[r], 34.f, buf, true);
            }
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Channel count display
// ------------------------------------------------------------
struct SLDisplayWidget : TransparentWidget {
    Splitter* module = nullptr;

    SLDisplayWidget(Splitter* m) : module(m) {
        box.pos = Vec(SL_DISP_X, SL_DISP_Y);
        box.size = Vec(SL_DISP_W, SL_DISP_H);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Screen
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0, 0, SL_DISP_W, SL_DISP_H, 4.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SL_DISP_H,
                                           nvgRGB(0x0a, 0x16, 0x1f), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);

        // Thin inner border
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0.5f, 0.5f, SL_DISP_W - 1.f, SL_DISP_H - 1.f, 4.f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        // State (module is null in the module browser: show a demo)
        int ch = module ? module->channels : 8;
        ch = std::min(std::max(ch, 0), 16);

        nvgSave(vg);
        nvgScissor(vg, 0, 0, SL_DISP_W, SL_DISP_H);

        // Soft glow behind the number
        float gx = SL_DISP_W / 2.f, gy = 24.f;
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SL_DISP_W, SL_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, gx, gy, 4.f, 40.f,
                                           nvgRGBA(0x3d, 0xe0, 0xff, ch > 0 ? 38 : 8),
                                           nvgRGBA(0x3d, 0xe0, 0xff, 0)));
        nvgFill(vg);

        // Caption
        drawText(vg, SL_DISP_W / 2.f, 8.f, "CHANNELS", 7.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        // Big number
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", ch);
        NVGcolor numCol = (ch > 0) ? nvgRGB(0x7a, 0xec, 0xff) : nvgRGBA(0x8a, 0x95, 0xa5, 150);
        drawText(vg, SL_DISP_W / 2.f, 25.f, buf, 26.f, numCol,
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        // 16-segment bar: bright = channel present in the input, faint = unused
        const float pitch = 7.5f, segW = 5.5f, segH = 5.f;
        const float x0 = (SL_DISP_W - (16.f * pitch - (pitch - segW))) / 2.f;
        const float y0 = SL_DISP_H - 11.f;
        for (int i = 0; i < 16; ++i) {
            float sx = x0 + i * pitch;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, sx, y0, segW, segH, 1.f);
            if (i < ch) {
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 235));
                nvgFill(vg);
                // glow
                nvgBeginPath(vg);
                nvgRoundedRect(vg, sx - 1.5f, y0 - 1.5f, segW + 3.f, segH + 3.f, 2.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
                nvgFill(vg);
            } else {
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) SL_DISP_H; y += 3)
            nvgRect(vg, 0, y + 1, SL_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
        nvgFill(vg);

        // Edge vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SL_DISP_W, SL_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, SL_DISP_W / 2.f, SL_DISP_H / 2.f,
                                           SL_DISP_W * 0.30f, SL_DISP_W * 0.70f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 130)));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, SL_DISP_W, SL_DISP_H * 0.35f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, SL_DISP_H * 0.35f,
                                           nvgRGBA(255, 255, 255, 16),
                                           nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        nvgRestore(vg);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct SplitterWidget : ModuleWidget {
    SplitterWidget(Splitter* module) {
        setModule(module);
        box.size = Vec(SL_PANEL_W, SL_PANEL_H);

        addChild(new SLPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SLDisplayWidget(module));

        // Polyphonic input
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(SL_IN_X, SL_IN_Y + 3.f), module, Splitter::POLY_INPUT));

        // 16 outputs in a 4 x 4 grid (row by row)
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                addOutput(createOutputCentered<DarkPJ301MPort>(
                    Vec(SL_GRID_X[c], SL_GRID_Y[r] + 3.f), module,
                    Splitter::SPLIT_OUTPUT + r * 4 + c));
            }
        }
    }
};

Model* modelSplitter = createModel<Splitter, SplitterWidget>("Splitter");