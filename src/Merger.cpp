#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float MG_PANEL_W = RACK_GRID_WIDTH * 12;   // 180
static const float MG_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Channel display
static const float MG_DISP_W = 132.f;
static const float MG_DISP_H = 52.f;
static const float MG_DISP_X = (MG_PANEL_W - MG_DISP_W) / 2.f;   // 24
static const float MG_DISP_Y = 48.f;

// Input grid (4 x 4)
static const float MG_SECTION_IN_Y = 120.f;
static const float MG_GRID_X[4]    = {30.f, 70.f, 110.f, 150.f};
static const float MG_GRID_Y[4]    = {155.f, 198.f, 241.f, 284.f};

// Output
static const float MG_SECTION_OUT_Y = 315.f;
static const float MG_OUT_X         = MG_PANEL_W / 2.f;
static const float MG_OUT_Y         = 349.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Merger : Module {
    enum ParamId {
        PARAMS_LEN
    };
    enum InputId {
        ENUMS(MERGE_INPUT, 16),
        INPUTS_LEN
    };
    enum OutputId {
        POLY_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    // Shown on the display
    int  channels = 0;
    bool active[16] = {};

    Merger() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        for (int i = 0; i < 16; ++i)
            configInput(MERGE_INPUT + i, string::f("Channel %d", i + 1));
        configOutput(POLY_OUTPUT, "Polyphonic");
    }

    void process(const ProcessArgs& args) override {
        // Output channel count = highest connected input (unconnected gaps carry 0 V)
        int last = 0;
        for (int i = 0; i < 16; ++i) {
            bool c = inputs[MERGE_INPUT + i].isConnected();
            active[i] = c;
            if (c) last = i + 1;
        }
        channels = last;

        outputs[POLY_OUTPUT].setChannels(last);
        for (int i = 0; i < last; ++i)
            outputs[POLY_OUTPUT].setVoltage(inputs[MERGE_INPUT + i].getVoltage(), i);
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
//NVGcolor colWarm()     { return nvgRGB(0xff, 0x9c, 0x6a); }

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
struct MGPanelBackground : TransparentWidget {
    MGPanelBackground() { box.size = Vec(MG_PANEL_W, MG_PANEL_H); }

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
        float x1 = 12.f, x2 = MG_PANEL_W - 12.f, cx = MG_PANEL_W / 2.f;
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
        float x = cx - w / 2.f, y = cy - 23.f, h = 40.f;
        plate(vg, x, y, w, h);

        NVGcolor c = isOutput ? nvgRGBA(0x3d, 0xe0, 0xff, 170) : nvgRGBA(0xff, 0x9c, 0x6a, 170);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x + 6.f, y + 1.5f, w - 12.f, 1.8f, 0.9f);
        nvgFillColor(vg, c);
        nvgFill(vg);

        label(vg, cx, cy - 15.f, txt);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, MG_PANEL_W, MG_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, MG_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) MG_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, MG_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Corner vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, MG_PANEL_W, MG_PANEL_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, MG_PANEL_W / 2.f, MG_PANEL_H * 0.4f,
                                           MG_PANEL_H * 0.30f, MG_PANEL_H * 0.80f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
        nvgFill(vg);

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, MG_PANEL_W - 1.f, MG_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Top accent line
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, MG_PANEL_W, 2.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, MG_PANEL_W, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        // Logo + title
        drawLogo(vg, MG_PANEL_W / 2.f, 15.f);
        drawText(vg, MG_PANEL_W / 2.f, 32.f, "MERGER", 12.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // ---- Display bezel (metallic, recessed) ----
        float bx = MG_DISP_X - 5.f, by = MG_DISP_Y - 5.f;
        float bw = MG_DISP_W + 10.f, bh = MG_DISP_H + 10.f;

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

        // ---- Inputs ----
        section(vg, MG_SECTION_IN_Y, "INPUTS");
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", r * 4 + c + 1);
                jackCell(vg, MG_GRID_X[c], MG_GRID_Y[r], 34.f, buf, false);
            }
        }

        // ---- Output ----
        section(vg, MG_SECTION_OUT_Y, "OUTPUT");
        jackCell(vg, MG_OUT_X, MG_OUT_Y, 58.f, "POLY OUT", true);

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Channel count display
// ------------------------------------------------------------
struct MGDisplayWidget : TransparentWidget {
    Merger* module = nullptr;

    MGDisplayWidget(Merger* m) : module(m) {
        box.pos = Vec(MG_DISP_X, MG_DISP_Y);
        box.size = Vec(MG_DISP_W, MG_DISP_H);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Screen
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0, 0, MG_DISP_W, MG_DISP_H, 4.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, MG_DISP_H,
                                           nvgRGB(0x0a, 0x16, 0x1f), nvgRGB(0x03, 0x06, 0x09)));
        nvgFill(vg);

        // Thin inner border
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0.5f, 0.5f, MG_DISP_W - 1.f, MG_DISP_H - 1.f, 4.f);
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
        int ch = 8;
        bool act[16];
        for (int i = 0; i < 16; ++i) act[i] = (i != 3 && i < 8);
        if (module) {
            ch = module->channels;
            for (int i = 0; i < 16; ++i) act[i] = module->active[i];
        }

        nvgSave(vg);
        nvgScissor(vg, 0, 0, MG_DISP_W, MG_DISP_H);

        // Soft glow behind the number
        float gx = MG_DISP_W / 2.f, gy = 24.f;
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, MG_DISP_W, MG_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, gx, gy, 4.f, 40.f,
                                           nvgRGBA(0x3d, 0xe0, 0xff, ch > 0 ? 38 : 8),
                                           nvgRGBA(0x3d, 0xe0, 0xff, 0)));
        nvgFill(vg);

        // Caption
        drawText(vg, MG_DISP_W / 2.f, 8.f, "CHANNELS", 7.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

        // Big number
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", ch);
        NVGcolor numCol = (ch > 0) ? nvgRGB(0x7a, 0xec, 0xff) : nvgRGBA(0x8a, 0x95, 0xa5, 150);
        drawText(vg, MG_DISP_W / 2.f, 25.f, buf, 26.f, numCol,
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        // 16-segment bar: bright = connected, dim outline = inside range but
        // unconnected (carries 0 V), very dim = unused
        const float pitch = 7.5f, segW = 5.5f, segH = 5.f;
        const float x0 = (MG_DISP_W - (16.f * pitch - (pitch - segW))) / 2.f;
        const float y0 = MG_DISP_H - 11.f;
        for (int i = 0; i < 16; ++i) {
            float sx = x0 + i * pitch;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, sx, y0, segW, segH, 1.f);
            if (act[i]) {
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 235));
                nvgFill(vg);
                // glow
                nvgBeginPath(vg);
                nvgRoundedRect(vg, sx - 1.5f, y0 - 1.5f, segW + 3.f, segH + 3.f, 2.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
                nvgFill(vg);
            } else if (i < ch) {
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, sx + 0.5f, y0 + 0.5f, segW - 1.f, segH - 1.f, 1.f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 110));
                nvgStrokeWidth(vg, 0.8f);
                nvgStroke(vg);
            } else {
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) MG_DISP_H; y += 3)
            nvgRect(vg, 0, y + 1, MG_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
        nvgFill(vg);

        // Edge vignette
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, MG_DISP_W, MG_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, MG_DISP_W / 2.f, MG_DISP_H / 2.f,
                                           MG_DISP_W * 0.30f, MG_DISP_W * 0.70f,
                                           nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 130)));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, MG_DISP_W, MG_DISP_H * 0.35f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, MG_DISP_H * 0.35f,
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
struct MergerWidget : ModuleWidget {
    MergerWidget(Merger* module) {
        setModule(module);
        box.size = Vec(MG_PANEL_W, MG_PANEL_H);

        addChild(new MGPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new MGDisplayWidget(module));

        // 16 inputs in a 4 x 4 grid (row by row)
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                addInput(createInputCentered<DarkPJ301MPort>(
                    Vec(MG_GRID_X[c], MG_GRID_Y[r] + 2.f), module,
                    Merger::MERGE_INPUT + r * 4 + c));
            }
        }

        // Polyphonic output
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(MG_OUT_X, MG_OUT_Y + 2.f), module, Merger::POLY_OUTPUT));
    }
};

Model* modelMerger = createModel<Merger, MergerWidget>("Merger");