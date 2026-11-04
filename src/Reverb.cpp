#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

static constexpr int MAX_DELAY = 192000;
static const int RV_SCOPE_N = 96;

// ============================================================
// Module – DSP unchanged, scope tracking added
// ============================================================
struct Reverb : Module
{
    float delayBuffer[MAX_DELAY] = {};
    int writePos = 0;
    float lowpass = 0.f;

    // ---- display state ----
    float scopeMin[RV_SCOPE_N] = {};
    float scopeMax[RV_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakOut = 0.f;

    enum ParamId
    {
        TIME_PARAM,
        DAMP_PARAM,
        DELAY_MS_PARAM,
        DRIVE_PARAM,
        MIX_PARAM,
        OUTPUT_GAIN_PARAM,
        PARAMS_LEN
    };

    enum InputId
    {
        AUDIO_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        AUDIO_OUTPUT,
        OUTPUTS_LEN
    };

    Reverb()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);
        configParam(TIME_PARAM, 0.f, 0.995f, 0.5f, "Decay (Feedback)");
        configParam(DAMP_PARAM, 0.f, 1.f, 0.5f, "Damping", " Hz", 200.f, 8000.f, 200.f);
        configParam(DELAY_MS_PARAM, 0.f, 1000.f, 200.f, "Delay Time", " ms");
        configParam(DRIVE_PARAM, 0.f, 1.f, 0.f, "Drive", " %", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/Wet");
        configParam(OUTPUT_GAIN_PARAM, 0.f, 3.f, 1.f, "Output Gain");
        configInput(AUDIO_INPUT, "Audio In");
        configOutput(AUDIO_OUTPUT, "Audio Out");
    }

    void process(const ProcessArgs &args) override
    {
        float in = inputs[AUDIO_INPUT].getVoltage();
        float time = params[TIME_PARAM].getValue();
        float damp = params[DAMP_PARAM].getValue();
        float drive = params[DRIVE_PARAM].getValue();
        float mix = params[MIX_PARAM].getValue();
        float outGain = params[OUTPUT_GAIN_PARAM].getValue();

        // Damping low‑pass coefficient
        float minFreq = 200.f;
        float maxFreq = 8000.f;
        float cutoff = minFreq * std::pow(maxFreq / minFreq, damp);
        float dampingCoeff = 1.f - std::exp(-2.f * M_PI * cutoff / args.sampleRate);
        dampingCoeff = rack::math::clamp(dampingCoeff, 0.f, 1.f);

        // Delay line with linear interpolation
        float delayMs = params[DELAY_MS_PARAM].getValue();
        float delaySamplesFloat = delayMs * 0.001f * args.sampleRate;
        delaySamplesFloat = rack::math::clamp(delaySamplesFloat, 1.f, MAX_DELAY - 1.f);
        int delayInt = static_cast<int>(delaySamplesFloat);
        float frac = delaySamplesFloat - delayInt;

        int readPos0 = writePos - delayInt;
        if (readPos0 < 0)
            readPos0 += MAX_DELAY;
        int readPos1 = readPos0 - 1;
        if (readPos1 < 0)
            readPos1 += MAX_DELAY;

        float delayed0 = delayBuffer[readPos0];
        float delayed1 = delayBuffer[readPos1];
        float delayed = delayed0 + frac * (delayed1 - delayed0);

        lowpass += dampingCoeff * (delayed - lowpass);
        float filtered = lowpass;

        float feedbackIn = in + filtered * time;
        delayBuffer[writePos] = std::tanh(feedbackIn);
        writePos = (writePos + 1) % MAX_DELAY;

        float wet = filtered;

        if (drive > 0.f)
        {
            float driveAmount = drive * 4.f;
            float norm = 1.f / std::tanh(driveAmount);
            wet = std::tanh(driveAmount * wet) * norm;
        }

        float mixed = (1.f - mix) * in + mix * wet;
        float out = 5.f * std::tanh(mixed * outGain);

        outputs[AUDIO_OUTPUT].setVoltage(out);

        // ---- display data ----
        float decayRate = 1.f - 4.f * args.sampleTime;
        peakOut = std::max(std::fabs(out), peakOut * decayRate);

        if (scopeCount == 0)
            scopeBMin = scopeBMax = out;
        else
        {
            scopeBMin = std::min(scopeBMin, out);
            scopeBMax = std::max(scopeBMax, out);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % RV_SCOPE_N;
        }
    }
};

// ============================================================
// Layout constants (12 HP = 180px wide)
// ============================================================
static const float RV_PANEL_W = RACK_GRID_WIDTH * 12; // 180
static const float RV_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float RV_DISP_W = 150.f;
static const float RV_DISP_H = 34.f;
static const float RV_DISP_X = (RV_PANEL_W - RV_DISP_W) / 2.f;
static const float RV_DISP_Y = 46.f;

static const float RV_CELL_W = 70.f;
static const float RV_CELL_H = 56.f;
static const float RV_ROW[3] = {130.f, 190.f, 250.f};
static const float RV_COL[2] = {45.f, 135.f};
static const float RV_PORT_Y = 326.f;

// ============================================================
// Theme + drawing helpers (anonymous namespace)
// ============================================================
namespace
{

    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    NVGcolor colDim() { return nvgRGB(0x7a, 0x85, 0x94); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    // NVGcolor colAccent2()  { return nvgRGB(0xb0, 0x5a, 0xff); }

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

        // Cell triangle
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

    // Cell table – 6 controls, 2 cols × 3 rows
    struct RvCellDef
    {
        const char *name;
        int param;
    };
    const RvCellDef RV_CELLS[6] = {
        {"DECAY", Reverb::TIME_PARAM},
        {"DAMP", Reverb::DAMP_PARAM},
        {"DELAY", Reverb::DELAY_MS_PARAM},
        {"DRIVE", Reverb::DRIVE_PARAM},
        {"MIX", Reverb::MIX_PARAM},
        {"GAIN", Reverb::OUTPUT_GAIN_PARAM},
    };

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct RvPanelBackground : TransparentWidget
    {
        RvPanelBackground() { box.size = Vec(RV_PANEL_W, RV_PANEL_H); }

        void plate(NVGcontext *vg, float cx, float ry)
        {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - RV_CELL_W / 2.f, ry - RV_CELL_H / 2.f,
                           RV_CELL_W, RV_CELL_H, 7.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        void section(NVGcontext *vg, float y, const char *txt)
        {
            float x1 = 14.f, x2 = RV_PANEL_W - 14.f, cx = RV_PANEL_W / 2.f;
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

            // Base gradient
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, RV_PANEL_W, RV_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, RV_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            // Brushed texture
            nvgStrokeWidth(vg, 1.f);
            for (int y = 0; y < (int)RV_PANEL_H; y += 3)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, y + 0.5f);
                nvgLineTo(vg, RV_PANEL_W, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
                nvgStroke(vg);
            }

            // Edge highlight
            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, RV_PANEL_W - 1.f, RV_PANEL_H - 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Logo + title
            drawLogo(vg, RV_PANEL_W / 2.f, 13.f);
            drawText(vg, RV_PANEL_W / 2.f, 31.f, "REVERB", 13.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

            // Display bezel
            float bx = RV_DISP_X - 5.f, by = RV_DISP_Y - 5.f;
            float bw = RV_DISP_W + 10.f, bh = RV_DISP_H + 10.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                               nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
            nvgFill(vg);

            // Control cells
            for (int i = 0; i < 6; ++i)
            {
                int row = i / 2, col = i % 2;
                float cx = RV_COL[col], ry = RV_ROW[row];
                plate(vg, cx, ry);
                drawText(vg, cx, ry + 18.f, RV_CELLS[i].name, 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            }

            // I/O section
            section(vg, 293.f, "I / O");
            const char *io[2] = {"INPUT", "OUTPUT"};
            for (int i = 0; i < 2; ++i)
            {
                float cx = RV_COL[i];
                nvgBeginPath(vg);
                nvgRoundedRect(vg, cx - 22.f, 302.f, 44.f, 42.f, 6.f);
                nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
                nvgFill(vg);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                drawText(vg, cx, 310.f, io[i], 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            }

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope display + single mono level meter
    // ------------------------------------------------------------
    struct RvDisplayWidget : TransparentWidget
    {
        Reverb *module = nullptr;

        RvDisplayWidget(Reverb *m) : module(m)
        {
            box.pos = Vec(RV_DISP_X, RV_DISP_Y);
            box.size = Vec(RV_DISP_W, RV_DISP_H);
        }

        static float scopeW() { return RV_DISP_W - 16.f; }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, RV_DISP_W, RV_DISP_H);
            nvgFillPaint(vg, nvgRadialGradient(vg, RV_DISP_W / 2.f, RV_DISP_H / 2.f, 20.f,
                                               RV_DISP_W * 0.6f,
                                               nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
            nvgFill(vg);

            // Grid
            nvgBeginPath(vg);
            for (float x = 0.f; x <= scopeW(); x += 28.f)
            {
                nvgMoveTo(vg, x + 0.5f, 0);
                nvgLineTo(vg, x + 0.5f, RV_DISP_H);
            }
            nvgMoveTo(vg, 0, RV_DISP_H / 2.f + 0.5f);
            nvgLineTo(vg, scopeW(), RV_DISP_H / 2.f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
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

            const float mid = RV_DISP_H / 2.f;
            const float amp = mid - 2.f;
            const float sw = scopeW();
            const float colW = sw / (float)RV_SCOPE_N;

            nvgSave(vg);
            nvgScissor(vg, 0, 0, sw, RV_DISP_H);
            for (int pass = 0; pass < 2; ++pass)
            { // 0: glow, 1: crisp
                nvgBeginPath(vg);
                for (int i = 0; i < RV_SCOPE_N; ++i)
                {
                    float vmin, vmax;
                    if (module)
                    {
                        int idx = (module->scopeHead + i) % RV_SCOPE_N;
                        vmin = module->scopeMin[idx];
                        vmax = module->scopeMax[idx];
                    }
                    else
                    {
                        float t = (float)i / RV_SCOPE_N;
                        float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                        vmax = e;
                        vmin = -e;
                    }
                    float y0 = mid - std::min(std::max(vmax / 5.f, -1.f), 1.f) * amp;
                    float y1 = mid - std::min(std::max(vmin / 5.f, -1.f), 1.f) * amp;
                    float h = std::max(y1 - y0, 1.f);
                    float x = i * colW;
                    if (pass == 0)
                        nvgRect(vg, x - 0.5f, y0 - 1.5f, colW + 1.f, h + 3.f);
                    else
                        nvgRect(vg, x, y0, std::max(colW - 0.4f, 1.f), h);
                }
                nvgFillColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 32) : nvgRGB(0x8a, 0xee, 0xff));
                nvgFill(vg);
            }
            nvgResetScissor(vg);
            nvgRestore(vg);

            // Mono level meter
            {
                float peak = module ? module->peakOut : 2.4f;
                float lvl = std::min(peak / 5.f, 1.f);
                float x = sw + 3.f;
                float mw = 6.f;
                nvgBeginPath(vg);
                nvgRect(vg, x, 1.f, mw, RV_DISP_H - 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
                nvgFill(vg);
                if (lvl > 0.01f)
                {
                    float h = lvl * (RV_DISP_H - 2.f);
                    nvgBeginPath(vg);
                    nvgRect(vg, x, RV_DISP_H - 1.f - h, mw, h);
                    nvgFillPaint(vg, nvgLinearGradient(vg, 0, RV_DISP_H, 0, 0,
                                                       nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                    nvgFill(vg);
                }
            }

            // Scanlines
            nvgBeginPath(vg);
            for (int y = 0; y < (int)RV_DISP_H; y += 2)
                nvgRect(vg, 0, y + 1, RV_DISP_W, 1);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
            nvgFill(vg);

            // Glass reflection
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, RV_DISP_W, RV_DISP_H * 0.4f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, RV_DISP_H * 0.4f,
                                               nvgRGBA(255, 255, 255, 16),
                                               nvgRGBA(255, 255, 255, 0)));
            nvgFill(vg);

            // Readouts
            char buf[32];
            float dly = module ? module->params[Reverb::DELAY_MS_PARAM].getValue() : 200.f;
            float mx = module ? module->params[Reverb::MIX_PARAM].getValue() : 0.5f;
            snprintf(buf, sizeof(buf), "DLY %3.0fMS", dly);
            drawText(vg, 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "MIX %3.0f%%", mx * 100.f);
            drawText(vg, sw - 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Widget
// ============================================================
struct ReverbWidget : ModuleWidget
{
    ReverbWidget(Reverb *module)
    {
        setModule(module);
        box.size = Vec(RV_PANEL_W, RV_PANEL_H);

        addChild(new RvPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new RvDisplayWidget(module));

        // 6 knobs – 2 columns × 3 rows
        for (int i = 0; i < 6; ++i)
        {
            int row = i / 2, col = i % 2;
            float cx = RV_COL[col], ry = RV_ROW[row];
            const RvCellDef &c = RV_CELLS[i];
            addParam(createParamCentered<RoundBlackKnob>(Vec(cx, ry - 6.f), module, c.param));
        }

        // I/O jacks – input left, output right
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(RV_COL[0], RV_PORT_Y), module, Reverb::AUDIO_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(RV_COL[1], RV_PORT_Y), module, Reverb::AUDIO_OUTPUT));
    }
};

Model *modelReverb = createModel<Reverb, ReverbWidget>("Reverb");