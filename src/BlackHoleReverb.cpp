#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

static const int NUM_DELAYS = 6;
static const int MAX_DELAY = 96000;
static const int PRE_DELAY_MAX = 44100;
static const int BH_SCOPE_N = 96;

// ============================================================
// Module – DSP preserved, scope tracking + CV range added
// ============================================================
struct BlackHoleReverb : Module
{
    float delayL[NUM_DELAYS][MAX_DELAY] = {};
    float delayR[NUM_DELAYS][MAX_DELAY] = {};

    int writeIndex = 0;

    float preDelayBuffer[PRE_DELAY_MAX] = {};
    int preWrite = 0;

    float modPhase[NUM_DELAYS] = {};
    float filterStoreL[NUM_DELAYS] = {};
    float filterStoreR[NUM_DELAYS] = {};

    enum CVRange
    {
        BIPOLAR_M5_5,
        UNIPOLAR_0_10
    };
    CVRange cvRangeMode = BIPOLAR_M5_5;

    // ---- display state ----
    float scopeMin[BH_SCOPE_N] = {};
    float scopeMax[BH_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    enum ParamId
    {
        MIX_PARAM,
        FEEDBACK_PARAM,
        DAMP_PARAM,
        MOD_PARAM,
        MIX_ATTEN,
        FEEDBACK_ATTEN,
        DAMP_ATTEN,
        MOD_ATTEN,
        PARAMS_LEN
    };

    enum InputId
    {
        IN_L,
        IN_R,
        MIX_CV,
        FEEDBACK_CV,
        DAMP_CV,
        MOD_CV,
        INPUTS_LEN
    };

    enum OutputId
    {
        OUT_L,
        OUT_R,
        OUTPUTS_LEN
    };

    BlackHoleReverb()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);

        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Mix", "%", 0.f, 100.f);
        configParam(FEEDBACK_PARAM, 0.f, 0.98f, 0.50f, "Feedback", "%", 0.f, 100.f);
        configParam(DAMP_PARAM, 0.f, 1.f, 0.7f, "Darkness", "%", 0.f, 100.f);
        configParam(MOD_PARAM, 0.f, 1.f, 0.5f, "Modulation", "%", 0.f, 100.f);

        configParam(MIX_ATTEN, 0.f, 1.f, 0.f, "Mix CV amount", "%", 0.f, 100.f);
        configParam(FEEDBACK_ATTEN, 0.f, 1.f, 0.f, "Feedback CV amount", "%", 0.f, 100.f);
        configParam(DAMP_ATTEN, 0.f, 1.f, 0.f, "Darkness CV amount", "%", 0.f, 100.f);
        configParam(MOD_ATTEN, 0.f, 1.f, 0.f, "Modulation CV amount", "%", 0.f, 100.f);

        configInput(IN_L, "Left");
        configInput(IN_R, "Right");
        configInput(MIX_CV, "Mix CV");
        configInput(FEEDBACK_CV, "Feedback CV");
        configInput(DAMP_CV, "Darkness CV");
        configInput(MOD_CV, "Modulation CV");

        configOutput(OUT_L, "Left");
        configOutput(OUT_R, "Right");

        configBypass(IN_L, OUT_L);
        configBypass(IN_R, OUT_R);
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *mode = json_object_get(root, "cvRangeMode");
        if (mode)
        {
            int m = (int)json_integer_value(mode);
            cvRangeMode = (m == UNIPOLAR_0_10) ? UNIPOLAR_0_10 : BIPOLAR_M5_5;
        }
    }

    float scaleCV(float v) const
    {
        if (cvRangeMode == UNIPOLAR_0_10)
            return v / 10.f;
        return clamp(v / 5.f, -1.f, 1.f);
    }

    void process(const ProcessArgs &args) override
    {
        if (args.sampleTime <= 0.0f)
            return;

        auto safe = [](float x) -> float
        {
            return std::isnormal(x) ? x : 0.0f;
        };

        float inL = safe(inputs[IN_L].getVoltage());
        float inR = safe(inputs[IN_R].getVoltage());

        float mixKnob = params[MIX_PARAM].getValue();
        float fbKnob = params[FEEDBACK_PARAM].getValue();
        float dampKnob = params[DAMP_PARAM].getValue();
        float modKnob = params[MOD_PARAM].getValue();

        float mixCV = safe(scaleCV(inputs[MIX_CV].getVoltage()));
        float fbCV = safe(scaleCV(inputs[FEEDBACK_CV].getVoltage()));
        float dampCV = safe(scaleCV(inputs[DAMP_CV].getVoltage()));
        float modCV = safe(scaleCV(inputs[MOD_CV].getVoltage()));

        float mixAtten = params[MIX_ATTEN].getValue();
        float fbAtten = params[FEEDBACK_ATTEN].getValue();
        float dampAtten = params[DAMP_ATTEN].getValue();
        float modAtten = params[MOD_ATTEN].getValue();

        float mix = clamp(mixKnob + mixCV * mixAtten, 0.f, 1.f);
        float feedback = clamp(fbKnob + fbCV * fbAtten, 0.f, 0.98f);
        float damp = clamp(dampKnob + dampCV * dampAtten, 0.f, 1.f);
        float modAmount = clamp(modKnob + modCV * modAtten, 0.f, 1.f);

        int preDelaySamples = (int)(0.08f * args.sampleRate);
        preDelaySamples = clamp(preDelaySamples, 0, PRE_DELAY_MAX - 1);
        int preRead = preWrite - preDelaySamples;
        if (preRead < 0)
            preRead += PRE_DELAY_MAX;
        float preDelayed = preDelayBuffer[preRead];
        preDelayBuffer[preWrite] = (inL + inR) * 0.5f;
        preWrite = (preWrite + 1) % PRE_DELAY_MAX;

        float revInL = preDelayed;
        float revInR = preDelayed;

        float dampSq = damp * damp;

        static const int baseDelays[NUM_DELAYS] = {
            2741, 3347, 3823, 4421, 5101, 5897};

        float wetL = 0.f, wetR = 0.f;

        for (int i = 0; i < NUM_DELAYS; i++)
        {
            modPhase[i] += args.sampleTime * (0.08f + 0.04f * i);
            if (modPhase[i] > 1.f)
                modPhase[i] -= 1.f;
            float mod = std::sin(2.f * M_PI * modPhase[i]);
            int delay = baseDelays[i] + (int)(mod * 240.f * modAmount);
            int readIndex = writeIndex - delay;
            if (readIndex < 0)
                readIndex += MAX_DELAY;

            float dl = delayL[i][readIndex];
            filterStoreL[i] = dl * (1.f - dampSq) + filterStoreL[i] * dampSq;
            float fbL = filterStoreL[i];
            float cross = delayR[i][readIndex] * 0.4f;
            delayL[i][writeIndex] = revInL + (fbL + cross) * feedback;
            wetL += dl;

            float dr = delayR[i][readIndex];
            filterStoreR[i] = dr * (1.f - dampSq) + filterStoreR[i] * dampSq;
            float fbR = filterStoreR[i];
            float crossR = delayL[i][readIndex] * 0.4f;
            delayR[i][writeIndex] = revInR + (fbR + crossR) * feedback;
            wetR += dr;
        }

        writeIndex++;
        if (writeIndex >= MAX_DELAY)
            writeIndex = 0;

        wetL /= NUM_DELAYS;
        wetR /= NUM_DELAYS;

        const float wetBoost = 1.4f;
        wetL *= wetBoost;
        wetR *= wetBoost;

        float outL = (1.f - mix) * inL + mix * wetL;
        float outR = (1.f - mix) * inR + mix * wetR;

        const float clipLevel = 10.f;
        outL = clipLevel * std::tanh(outL / clipLevel);
        outR = clipLevel * std::tanh(outR / clipLevel);

        outputs[OUT_L].setVoltage(safe(outL));
        outputs[OUT_R].setVoltage(safe(outR));

        // ---- display data ----
        float decay = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(outL), peakL * decay);
        peakR = std::max(std::fabs(outR), peakR * decay);

        float mono = 0.5f * (outL + outR);
        if (scopeCount == 0)
            scopeBMin = scopeBMax = mono;
        else
        {
            scopeBMin = std::min(scopeBMin, mono);
            scopeBMax = std::max(scopeBMax, mono);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % BH_SCOPE_N;
        }
    }
};

// ============================================================
// Layout (12 HP = 180 x 380 px)
//
//   y   0-15   screws
//   y  15      title
//   y  30-62   display (scope + meters)
//   y  76      readouts (own row, under the display)
//   y  88      section: REVERB
//   y  96-184  row 1: label / knob / CV jack + attenuator / tiny labels
//   y 190-278  row 2
//   y 292      section: I / O
//   y 300-352  4 jack cells (2 inputs left, 2 outputs right)
//   y 367      maker tag
//
// Every control lives inside its plate; no label shares space with a widget.
// ============================================================
static const float BH_PANEL_W = RACK_GRID_WIDTH * 12; // 180
static const float BH_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float BH_TITLE_Y = 15.f;
static const float BH_MAKER_Y = 367.f;

static const float BH_DISP_W = 150.f;
static const float BH_DISP_H = 32.f;
static const float BH_DISP_X = (BH_PANEL_W - BH_DISP_W) / 2.f; // 15
static const float BH_DISP_Y = 30.f;
static const float BH_READ_Y = 76.f; // panel coordinates

static const float BH_SEC1_Y = 88.f;
static const float BH_CELL_W = 76.f, BH_CELL_H = 88.f;
static const float BH_COL[2] = {48.f, 132.f};
static const float BH_ROW_TOP[2] = {96.f, 190.f};
static const float BH_CELL_LABEL = 10.f; // offsets from the plate top
static const float BH_CELL_KNOB = 32.f;
static const float BH_CELL_JACK = 62.f;
static const float BH_CELL_SUBLBL = 80.f;
static const float BH_SUB_DX = 17.f; // CV jack left, attenuator right

static const float BH_SEC2_Y = 292.f;
static const float BH_IO_TOP = 300.f, BH_IO_H = 52.f, BH_IO_W = 38.f;
static const float BH_IO_LABEL = 10.f, BH_IO_JACK = 34.f;
static float bhIoX(int i) { return 25.5f + 43.f * i; }

// ============================================================
// Theme + drawing helpers (anonymous namespace)
// ============================================================
namespace
{

    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    NVGcolor colDim() { return nvgRGB(0x8a, 0x95, 0xa4); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    NVGcolor colAccent2() { return nvgRGB(0xb0, 0x5a, 0xff); }

    const int ALIGN_L = NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE;
    const int ALIGN_C = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE;
    const int ALIGN_R = NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE;

    bool setFont(NVGcontext *vg)
    {
        if (!APP || !APP->window)
            return false;
        std::shared_ptr<window::Font> font =
            APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (!font)
            return false;
        nvgFontFaceId(vg, font->handle);
        return true;
    }

    void drawText(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
                  int align = ALIGN_C, float spacing = 0.5f)
    {
        if (!setFont(vg))
            return;
        nvgFontSize(vg, size);
        nvgTextLetterSpacing(vg, spacing);
        nvgTextAlign(vg, align);
        nvgFillColor(vg, col);
        nvgText(vg, x, y, s, NULL);
    }

    // Control plate; tint = output jacks
    void plate(NVGcontext *vg, float cx, float top, float w, float h, bool tint = false)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - w / 2.f, top, w, h, 5.f);
        nvgFillColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 22) : nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, tint ? nvgRGBA(0x3d, 0xe0, 0xff, 60) : nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // Section title with hairlines that stop a measured distance from the text
    void sectionHeader(NVGcontext *vg, float W, float y, const char *txt)
    {
        const float x1 = 12.f, x2 = W - 12.f, cx = W / 2.f;
        if (!setFont(vg))
            return;
        nvgFontSize(vg, 8.f);
        nvgTextLetterSpacing(vg, 2.f);
        float b[4];
        nvgTextBounds(vg, 0.f, 0.f, txt, NULL, b);
        float gap = (b[2] - b[0]) / 2.f + 6.f;

        drawText(vg, cx, y, txt, 8.f, colAccent(), ALIGN_C, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - gap, y);
        nvgMoveTo(vg, cx + gap, y);
        nvgLineTo(vg, x2, y);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    struct BhCellDef
    {
        const char *name;
        int param, attn, cv;
    };
    const BhCellDef BH_CELLS[4] = {
        {"MIX", BlackHoleReverb::MIX_PARAM, BlackHoleReverb::MIX_ATTEN, BlackHoleReverb::MIX_CV},
        {"FEEDBACK", BlackHoleReverb::FEEDBACK_PARAM, BlackHoleReverb::FEEDBACK_ATTEN, BlackHoleReverb::FEEDBACK_CV},
        {"DARKNESS", BlackHoleReverb::DAMP_PARAM, BlackHoleReverb::DAMP_ATTEN, BlackHoleReverb::DAMP_CV},
        {"MOD", BlackHoleReverb::MOD_PARAM, BlackHoleReverb::MOD_ATTEN, BlackHoleReverb::MOD_CV},
    };
    const char *BH_IO_NAMES[4] = {"L IN", "R IN", "L OUT", "R OUT"};

    // ------------------------------------------------------------
    // Panel background (static artwork only)
    // ------------------------------------------------------------
    struct BhPanelBackground : TransparentWidget
    {
        BhPanelBackground() { box.size = Vec(BH_PANEL_W, BH_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, BH_PANEL_W, BH_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, BH_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            drawText(vg, BH_PANEL_W / 2.f, BH_TITLE_Y, "BLACK HOLE", 12.f, colText(), ALIGN_C, 3.f);
            drawText(vg, BH_PANEL_W / 2.f, BH_MAKER_Y, "AGMM", 9.f, colDim(), ALIGN_C, 3.f);

            // Display bezel
            nvgBeginPath(vg);
            nvgRoundedRect(vg, BH_DISP_X - 4.f, BH_DISP_Y - 4.f,
                           BH_DISP_W + 8.f, BH_DISP_H + 8.f, 5.f);
            nvgFillColor(vg, nvgRGB(0x0b, 0x0c, 0x0e));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // ---- REVERB: 2 x 2 cells ----
            sectionHeader(vg, BH_PANEL_W, BH_SEC1_Y, "REVERB");
            for (int i = 0; i < 4; ++i)
            {
                float cx = BH_COL[i % 2];
                float top = BH_ROW_TOP[i / 2];
                plate(vg, cx, top, BH_CELL_W, BH_CELL_H);
                drawText(vg, cx, top + BH_CELL_LABEL, BH_CELLS[i].name, 8.f, colDim(), ALIGN_C, 0.5f);
                drawText(vg, cx - BH_SUB_DX, top + BH_CELL_SUBLBL, "CV", 7.f, colDim(), ALIGN_C, 0.5f);
                drawText(vg, cx + BH_SUB_DX, top + BH_CELL_SUBLBL, "ATT", 7.f, colDim(), ALIGN_C, 0.5f);
            }

            // ---- I / O (inputs left, outputs right, outputs tinted) ----
            sectionHeader(vg, BH_PANEL_W, BH_SEC2_Y, "I / O");
            for (int i = 0; i < 4; ++i)
            {
                float cx = bhIoX(i);
                plate(vg, cx, BH_IO_TOP, BH_IO_W, BH_IO_H, i >= 2);
                drawText(vg, cx, BH_IO_TOP + BH_IO_LABEL, BH_IO_NAMES[i], 7.f, colDim(), ALIGN_C, 0.5f);
            }

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope + L/R level meters. Readouts live in their own row below
    // the display, never over the waveform.
    // ------------------------------------------------------------
    struct BhDisplayWidget : TransparentWidget
    {
        BlackHoleReverb *module = nullptr;

        BhDisplayWidget(BlackHoleReverb *m) : module(m)
        {
            box.pos = Vec(BH_DISP_X, BH_DISP_Y);
            box.size = Vec(BH_DISP_W, BH_DISP_H);
        }

        static float scopeW() { return BH_DISP_W - 16.f; }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, BH_DISP_W, BH_DISP_H);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
            nvgFill(vg);

            nvgBeginPath(vg);
            for (float x = 0.f; x <= scopeW(); x += 28.f)
            {
                nvgMoveTo(vg, x + 0.5f, 0);
                nvgLineTo(vg, x + 0.5f, BH_DISP_H);
            }
            nvgMoveTo(vg, 0, BH_DISP_H / 2.f + 0.5f);
            nvgLineTo(vg, scopeW(), BH_DISP_H / 2.f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer == 1)
            {
                NVGcontext *vg = args.vg;

                const float mid = BH_DISP_H / 2.f;
                const float amp = mid - 2.f;
                const float sw = scopeW();
                const float colW = sw / (float)BH_SCOPE_N;

                // Waveform
                nvgSave(vg);
                nvgScissor(vg, 0, 0, sw, BH_DISP_H);
                nvgBeginPath(vg);
                for (int i = 0; i < BH_SCOPE_N; ++i)
                {
                    float vmin, vmax;
                    if (module)
                    {
                        int idx = (module->scopeHead + i) % BH_SCOPE_N;
                        vmin = module->scopeMin[idx];
                        vmax = module->scopeMax[idx];
                    }
                    else
                    {
                        float t = (float)i / BH_SCOPE_N;
                        float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                        vmax = e;
                        vmin = -e;
                    }
                    float y0 = mid - clamp(vmax / 5.f, -1.f, 1.f) * amp;
                    float y1 = mid - clamp(vmin / 5.f, -1.f, 1.f) * amp;
                    nvgRect(vg, i * colW, y0, std::max(colW - 0.4f, 1.f), std::max(y1 - y0, 1.f));
                }
                nvgFillColor(vg, nvgRGB(0x8a, 0xee, 0xff));
                nvgFill(vg);
                nvgResetScissor(vg);
                nvgRestore(vg);

                // L / R level meters (right of the scope)
                for (int ch = 0; ch < 2; ++ch)
                {
                    float peak = module ? (ch == 0 ? module->peakL : module->peakR)
                                        : (ch == 0 ? 3.f : 2.4f);
                    float lvl = std::min(peak / 10.f, 1.f);
                    float x = sw + 4.f + ch * 6.f;
                    nvgBeginPath(vg);
                    nvgRect(vg, x, 1.f, 4.f, BH_DISP_H - 2.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                    nvgFill(vg);
                    if (lvl > 0.01f)
                    {
                        float h = lvl * (BH_DISP_H - 2.f);
                        nvgBeginPath(vg);
                        nvgRect(vg, x, BH_DISP_H - 1.f - h, 4.f, h);
                        nvgFillPaint(vg, nvgLinearGradient(vg, 0, BH_DISP_H, 0, 0,
                                                           colAccent(), colAccent2()));
                        nvgFill(vg);
                    }
                }

                // Readouts: own row under the display, centred over the two columns
                char buf[32];
                float mx = module ? module->params[BlackHoleReverb::MIX_PARAM].getValue() : 0.5f;
                float fb = module ? module->params[BlackHoleReverb::FEEDBACK_PARAM].getValue() : 0.5f;
                const float ry = BH_READ_Y - BH_DISP_Y;
                snprintf(buf, sizeof(buf), "MIX %3.0f%%", mx * 100.f);
                drawText(vg, BH_COL[0] - BH_DISP_X, ry, buf, 7.f, colAccent(), ALIGN_C, 0.5f);
                snprintf(buf, sizeof(buf), "FB %3.0f%%", fb * 100.f);
                drawText(vg, BH_COL[1] - BH_DISP_X, ry, buf, 7.f, colAccent(), ALIGN_C, 0.5f);
            }
            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Module widget
// ============================================================
struct BlackHoleReverbWidget : ModuleWidget
{
    BlackHoleReverbWidget(BlackHoleReverb *module)
    {
        setModule(module);
        box.size = Vec(BH_PANEL_W, BH_PANEL_H);

        addChild(new BhPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new BhDisplayWidget(module));

        // 4 cells: knob, then CV jack (left) + attenuator (right) below it
        for (int i = 0; i < 4; ++i)
        {
            float cx = BH_COL[i % 2];
            float top = BH_ROW_TOP[i / 2];
            const BhCellDef &c = BH_CELLS[i];
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(cx, top + BH_CELL_KNOB), module, c.param));
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(cx - BH_SUB_DX, top + BH_CELL_JACK), module, c.cv));
            addParam(createParamCentered<Trimpot>(
                Vec(cx + BH_SUB_DX, top + BH_CELL_JACK), module, c.attn));
        }

        // I/O: L IN, R IN | L OUT, R OUT
        const float jy = BH_IO_TOP + BH_IO_JACK;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(bhIoX(0), jy), module, BlackHoleReverb::IN_L));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(bhIoX(1), jy), module, BlackHoleReverb::IN_R));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(bhIoX(2), jy), module, BlackHoleReverb::OUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(bhIoX(3), jy), module, BlackHoleReverb::OUT_R));
    }

    void appendContextMenu(Menu *menu) override
    {
        BlackHoleReverb *module = dynamic_cast<BlackHoleReverb *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem
        {
            BlackHoleReverb *module;
            BlackHoleReverb::CVRange range;
            void onAction(const ActionEvent &e) override
            {
                module->cvRangeMode = range;
            }
            void step() override
            {
                rightText = CHECKMARK(module->cvRangeMode == range);
                MenuItem::step();
            }
        };

        CVRangeItem *bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = BlackHoleReverb::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);

        CVRangeItem *unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar)");
        unipolarItem->module = module;
        unipolarItem->range = BlackHoleReverb::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);
    }
};

Model *modelBlackHoleReverb = createModel<BlackHoleReverb, BlackHoleReverbWidget>("BlackHoleReverb");