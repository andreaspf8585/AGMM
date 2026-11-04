#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float RG_PANEL_W = RACK_GRID_WIDTH * 6; // 90
static const float RG_PANEL_H = RACK_GRID_HEIGHT;    // 380

static const float RG_DISP_W = 64.f;
static const float RG_DISP_H = 64.f;
static const float RG_DISP_X = (RG_PANEL_W - RG_DISP_W) / 2.f;
static const float RG_DISP_Y = 52.f;

static const float RG_CX = RG_PANEL_W / 2.f;
static const float RG_LOW_Y = 166.f;
static const float RG_HIGH_Y = 228.f;
static const float RG_PORT_Y = 326.f;
static const float RG_PORT_X[2] = {26.f, 64.f};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Range : Module
{
    enum ParamId
    {
        LOW_PARAM,
        HIGH_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        SIGNAL_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        SIGNAL_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // For the display (written by the audio thread, read by the UI thread)
    float lastIn = 0.f;
    float lastOut = 0.f;
    bool active = false;

    Range()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(LOW_PARAM, -10.f, 10.f, 0.f, "Low", " V");
        configParam(HIGH_PARAM, -10.f, 10.f, 10.f, "High", " V");
        configInput(SIGNAL_INPUT, "Signal");
        configOutput(SIGNAL_OUTPUT, "Output");
    }

    void process(const ProcessArgs &args) override
    {
        if (!inputs[SIGNAL_INPUT].isConnected())
        {
            outputs[SIGNAL_OUTPUT].setVoltage(0.f);
            active = false;
            return;
        }

        float in = inputs[SIGNAL_INPUT].getVoltage();
        float low = params[LOW_PARAM].getValue();
        float high = params[HIGH_PARAM].getValue();

        // Map input [-5V, +5V] to output [low, high]
        float out = ((in + 5.f) / 10.f) * (high - low) + low;

        outputs[SIGNAL_OUTPUT].setVoltage(out);

        lastIn = in;
        lastOut = out;
        active = true;
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{

    // ------------------------------------------------------------
    // Theme (same palette as CellAutoGate / HeatRev)
    // ------------------------------------------------------------
    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    NVGcolor colDim() { return nvgRGB(0x7a, 0x85, 0x94); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }

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

    // ------------------------------------------------------------
    // AGMM logo
    // ------------------------------------------------------------
    void drawLogo(NVGcontext *vg, float cx, float cy)
    {
        const float w = 64.f, h = 18.f;
        float x = cx - w / 2.f, y = cy - h / 2.f;

        // Glow
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
        nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
        nvgFill(vg);

        // Pill body
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 7.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                           nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
        nvgFill(vg);
        nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
                                             nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgStrokeWidth(vg, 1.2f);
        nvgStroke(vg);

        // Tiny cell triangle
        const float s = 2.2f;
        float tx = x + 7.f, ty = y + h / 2.f - 3.f;
        nvgBeginPath(vg);
        nvgRect(vg, tx + s, ty, s, s);
        nvgRect(vg, tx, ty + s * 1.4f, s, s);
        nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
        nvgFillColor(vg, colAccent());
        nvgFill(vg);

        // Wordmark
        drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct RGPanelBackground : TransparentWidget
    {
        RGPanelBackground() { box.size = Vec(RG_PANEL_W, RG_PANEL_H); }

        void label(NVGcontext *vg, float x, float y, const char *txt)
        {
            drawText(vg, x, y, txt, 8.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        }

        void section(NVGcontext *vg, float y, const char *txt)
        {
            float x1 = 12.f, x2 = RG_PANEL_W - 12.f, cx = RG_PANEL_W / 2.f;
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

        void ring(NVGcontext *vg, float cx, float cy)
        {
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, 20.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            // Base gradient
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, RG_PANEL_W, RG_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, RG_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            // Brushed texture
            nvgStrokeWidth(vg, 1.f);
            for (int y = 0; y < (int)RG_PANEL_H; y += 3)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, y + 0.5f);
                nvgLineTo(vg, RG_PANEL_W, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
                nvgStroke(vg);
            }

            // Edge highlight
            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, RG_PANEL_W - 1.f, RG_PANEL_H - 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Logo + title
            drawLogo(vg, RG_PANEL_W / 2.f, 15.f);
            drawText(vg, RG_PANEL_W / 2.f, 34.f, "RANGE", 14.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

            // Display bezel
            float bx = RG_DISP_X - 5.f, by = RG_DISP_Y - 5.f;
            float bw = RG_DISP_W + 10.f, bh = RG_DISP_H + 10.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                               nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
            nvgFill(vg);

            // Sections
            section(vg, 136.f, "SCALE");
            section(vg, 288.f, "I / O");

            // Knob rings + labels
            ring(vg, RG_CX, RG_LOW_Y);
            ring(vg, RG_CX, RG_HIGH_Y);
            label(vg, RG_CX, RG_LOW_Y + 27.f, "LOW");
            label(vg, RG_CX, RG_HIGH_Y + 27.f, "HIGH");

            // Port plates + labels
            const char *io[2] = {"IN", "OUT"};
            for (int i = 0; i < 2; ++i)
            {
                float cx = RG_PORT_X[i];
                nvgBeginPath(vg);
                nvgRoundedRect(vg, cx - 18.f, RG_PORT_Y - 18.f, 36.f, 36.f, 6.f);
                nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
                nvgFill(vg);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                label(vg, cx, RG_PORT_Y + 28.f, io[i]);
            }

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Transfer-function display
    // ------------------------------------------------------------
    struct RGDisplayWidget : TransparentWidget
    {
        Range *module = nullptr;

        RGDisplayWidget(Range *m) : module(m)
        {
            box.pos = Vec(RG_DISP_X, RG_DISP_Y);
            box.size = Vec(RG_DISP_W, RG_DISP_H);
        }

        // Voltage (-10..+10) to display coordinates
        static float px(float v) { return (v + 10.f) / 20.f * RG_DISP_W; }
        static float py(float v) { return RG_DISP_H - (v + 10.f) / 20.f * RG_DISP_H; }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, RG_DISP_W, RG_DISP_H);
            nvgFillPaint(vg, nvgRadialGradient(vg, RG_DISP_W / 2.f, RG_DISP_H / 2.f, 12.f, RG_DISP_W * 0.8f,
                                               nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
            nvgFill(vg);

            // Nominal input window (-5V..+5V)
            nvgBeginPath(vg);
            nvgRect(vg, px(-5.f), 0, px(5.f) - px(-5.f), RG_DISP_H);
            nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 10));
            nvgFill(vg);

            // Grid every 5V
            nvgBeginPath(vg);
            for (int v = -10; v <= 10; v += 5)
            {
                nvgMoveTo(vg, px((float)v) + 0.5f, 0);
                nvgLineTo(vg, px((float)v) + 0.5f, RG_DISP_H);
                nvgMoveTo(vg, 0, py((float)v) + 0.5f);
                nvgLineTo(vg, RG_DISP_W, py((float)v) + 0.5f);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Zero axes
            nvgBeginPath(vg);
            nvgMoveTo(vg, px(0.f) + 0.5f, 0);
            nvgLineTo(vg, px(0.f) + 0.5f, RG_DISP_H);
            nvgMoveTo(vg, 0, py(0.f) + 0.5f);
            nvgLineTo(vg, RG_DISP_W, py(0.f) + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStroke(vg);

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

            float low = module ? module->params[Range::LOW_PARAM].getValue() : 0.f;
            float high = module ? module->params[Range::HIGH_PARAM].getValue() : 10.f;
            float span = high - low;

            nvgSave(vg);
            nvgScissor(vg, 0, 0, RG_DISP_W, RG_DISP_H);

            // Transfer curve: out = ((in + 5) / 10) * (high - low) + low
            float x0 = -10.f, x1 = 10.f;
            float y0 = ((x0 + 5.f) / 10.f) * span + low;
            float y1 = ((x1 + 5.f) / 10.f) * span + low;

            // Glow pass, then crisp pass
            for (int pass = 0; pass < 2; ++pass)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, px(x0), py(y0));
                nvgLineTo(vg, px(x1), py(y1));
                nvgLineCap(vg, NVG_ROUND);
                if (pass == 0)
                {
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
                    nvgStrokeWidth(vg, 5.f);
                }
                else
                {
                    nvgStrokeColor(vg, nvgRGB(0x8a, 0xee, 0xff));
                    nvgStrokeWidth(vg, 1.4f);
                }
                nvgStroke(vg);
            }

            // Output range markers (low / high)
            for (int i = 0; i < 2; ++i)
            {
                float v = (i == 0) ? low : high;
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, py(v));
                nvgLineTo(vg, 5.f, py(v));
                nvgMoveTo(vg, RG_DISP_W - 5.f, py(v));
                nvgLineTo(vg, RG_DISP_W, py(v));
                nvgStrokeColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 200));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }

            // Live signal marker
            bool active = module ? module->active : true;
            float in = module ? module->lastIn : 2.f;
            float out = module ? module->lastOut : ((2.f + 5.f) / 10.f) * span + low;
            if (active)
            {
                float cx = px(clamp(in, -10.f, 10.f));
                float cy = py(clamp(out, -10.f, 10.f));

                nvgBeginPath(vg);
                nvgMoveTo(vg, cx, cy);
                nvgLineTo(vg, cx, RG_DISP_H);
                nvgMoveTo(vg, cx, cy);
                nvgLineTo(vg, 0, cy);
                nvgStrokeColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 90));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, 5.f);
                nvgFillColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 60));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, 2.2f);
                nvgFillColor(vg, nvgRGB(0xf0, 0xe0, 0xff));
                nvgFill(vg);
            }

            nvgResetScissor(vg);
            nvgRestore(vg);

            // Scanlines
            nvgBeginPath(vg);
            for (int y = 0; y < (int)RG_DISP_H; y += 2)
                nvgRect(vg, 0, y + 1, RG_DISP_W, 1);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
            nvgFill(vg);

            // Glass reflection
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, RG_DISP_W, RG_DISP_H * 0.4f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, RG_DISP_H * 0.4f,
                                               nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
            nvgFill(vg);

            // Readout under the display
            char buf[32];
            snprintf(buf, sizeof(buf), "%+.1f / %+.1f V", low, high);
            drawText(vg, RG_DISP_W / 2.f, RG_DISP_H + 14.f, buf, 8.f, colAccent(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct RangeWidget : ModuleWidget
{
    RangeWidget(Range *module)
    {
        setModule(module);
        box.size = Vec(RG_PANEL_W, RG_PANEL_H);

        addChild(new RGPanelBackground());

        // Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new RGDisplayWidget(module));

        // Knobs
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(RG_CX, RG_LOW_Y), module, Range::LOW_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(RG_CX, RG_HIGH_Y), module, Range::HIGH_PARAM));

        // Ports
        addInput(createInputCentered<DarkPJ301MPort>(Vec(RG_PORT_X[0], RG_PORT_Y), module, Range::SIGNAL_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(RG_PORT_X[1], RG_PORT_Y), module, Range::SIGNAL_OUTPUT));
    }
};

Model *modelRange = createModel<Range, RangeWidget>("Range");