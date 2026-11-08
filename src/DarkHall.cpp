#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

static const int NUM_COMBS = 8;
static const int NUM_ALLPASS = 4;
static const int MAX_DELAY = 96000;
static const int DH_SCOPE_N = 96;

// ============================================================
// Module – DSP byte-for-byte unchanged, scope tracking added
// ============================================================
struct DarkHall : Module
{
    float combL[NUM_COMBS][MAX_DELAY] = {};
    float combR[NUM_COMBS][MAX_DELAY] = {};
    float combStoreL[NUM_COMBS] = {};
    float combStoreR[NUM_COMBS] = {};

    float allpassL[NUM_ALLPASS][MAX_DELAY] = {};
    float allpassR[NUM_ALLPASS][MAX_DELAY] = {};
    int apIdxL[NUM_ALLPASS] = {};
    int apIdxR[NUM_ALLPASS] = {};

    float modPhase[NUM_COMBS] = {};

    int writeIndex = 0;

    // ---- display state ----
    float scopeMin[DH_SCOPE_N] = {};
    float scopeMax[DH_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    enum ParamId
    {
        MIX_PARAM,
        SIZE_PARAM,
        DAMP_PARAM,
        MOD_PARAM,
        PARAMS_LEN
    };

    enum InputId
    {
        IN_L,
        IN_R,
        INPUTS_LEN
    };

    enum OutputId
    {
        OUT_L,
        OUT_R,
        OUTPUTS_LEN
    };

    DarkHall()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);

        configParam(MIX_PARAM, 0.f, 1.f, 0.4f, "Mix", "%", 0.f, 100.f);
        configParam(SIZE_PARAM, 0.f, 1.f, 0.7f, "Room size", "%", 0.f, 100.f);
        configParam(DAMP_PARAM, 0.f, 1.f, 0.6f, "Damping", "%", 0.f, 100.f);
        configParam(MOD_PARAM, 0.f, 1.f, 0.3f, "Modulation", "%", 0.f, 100.f);

        configInput(IN_L, "Left");
        configInput(IN_R, "Right");

        configOutput(OUT_L, "Left");
        configOutput(OUT_R, "Right");

        configBypass(IN_L, OUT_L);
        configBypass(IN_R, OUT_R);
    }

    float allpass(float input, float *buf, int &idx, float gain)
    {
        float delayed = buf[idx];
        float output = -input + delayed;

        buf[idx] = input + delayed * gain;

        idx++;
        if (idx >= MAX_DELAY)
            idx = 0;

        return output;
    }

    void process(const ProcessArgs &args) override
    {
        // =====================
        // INPUT (attenuated)
        // =====================
        float inL = inputs[IN_L].getVoltage() * 0.2f;
        float inR = inputs[IN_R].getVoltage() * 0.2f;

        float mix = params[MIX_PARAM].getValue();
        float size = params[SIZE_PARAM].getValue();
        float damp = std::pow(params[DAMP_PARAM].getValue(), 2.f);
        float modAmt = params[MOD_PARAM].getValue();

        static const int baseDelays[NUM_COMBS] = {
            1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};

        float sumL = 0.f;
        float sumR = 0.f;

        float feedback = 0.75f + size * 0.2f;

        for (int i = 0; i < NUM_COMBS; i++)
        {
            modPhase[i] += args.sampleTime * (0.05f + i * 0.01f);
            if (modPhase[i] > 1.f)
                modPhase[i] -= 1.f;

            float mod = std::sin(2.f * M_PI * modPhase[i]);
            int delay = baseDelays[i] + (int)(mod * 25.f * modAmt);
            delay = (int)(delay * (0.5f + size));

            int rIdx = writeIndex - delay;
            if (rIdx < 0)
                rIdx += MAX_DELAY;

            float dl = combL[i][rIdx];
            float dr = combR[i][rIdx];

            combStoreL[i] = dl * (1.f - damp) + combStoreL[i] * damp;
            combStoreR[i] = dr * (1.f - damp) + combStoreR[i] * damp;

            combStoreL[i] = clamp(combStoreL[i], -10.f, 10.f);
            combStoreR[i] = clamp(combStoreR[i], -10.f, 10.f);

            float cross = dr * 0.08f;
            float crossR = dl * 0.08f;

            float fbL = (combStoreL[i] + cross) * feedback;
            float fbR = (combStoreR[i] + crossR) * feedback;

            fbL = clamp(fbL, -5.f, 5.f);
            fbR = clamp(fbR, -5.f, 5.f);

            combL[i][writeIndex] = inL + fbL;
            combR[i][writeIndex] = inR + fbR;

            sumL += dl;
            sumR += dr;
        }

        writeIndex++;
        if (writeIndex >= MAX_DELAY)
            writeIndex = 0;

        sumL /= NUM_COMBS;
        sumR /= NUM_COMBS;

        float apL = sumL;
        float apR = sumR;

        for (int i = 0; i < NUM_ALLPASS; i++)
        {
            apL = allpass(apL, allpassL[i], apIdxL[i], 0.6f);
            apR = allpass(apR, allpassR[i], apIdxR[i], 0.6f);
        }

        apL *= 0.35f;
        apR *= 0.35f;

        float dry = cosf(mix * M_PI_2);
        float wet = sinf(mix * M_PI_2);

        float outL = dry * inL + wet * apL;
        float outR = dry * inR + wet * apR;

        outL = tanh(outL * 1.2f);
        outR = tanh(outR * 1.2f);

        if (!std::isfinite(outL))
            outL = 0.f;
        if (!std::isfinite(outR))
            outR = 0.f;

        outputs[OUT_L].setVoltage(outL * 5.f);
        outputs[OUT_R].setVoltage(outR * 5.f);

        // ---- display data ----
        float vL = outL * 5.f;
        float vR = outR * 5.f;
        float decay = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(vL), peakL * decay);
        peakR = std::max(std::fabs(vR), peakR * decay);

        float mono = 0.5f * (vL + vR);
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
            scopeHead = (scopeHead + 1) % DH_SCOPE_N;
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
//   y  96-172  row 1: label / knob
//   y 180-256  row 2
//   y 288      section: I / O
//   y 296-348  4 jack cells (2 inputs left, 2 outputs right)
//   y 367      maker tag
//
// Every control lives inside its plate; no label shares space with a widget.
// ============================================================
static const float DH_PANEL_W = RACK_GRID_WIDTH * 12; // 180
static const float DH_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float DH_TITLE_Y = 15.f;
static const float DH_MAKER_Y = 367.f;

static const float DH_DISP_W = 150.f;
static const float DH_DISP_H = 32.f;
static const float DH_DISP_X = (DH_PANEL_W - DH_DISP_W) / 2.f; // 15
static const float DH_DISP_Y = 30.f;
static const float DH_READ_Y = 76.f; // panel coordinates

static const float DH_SEC1_Y = 88.f;
static const float DH_CELL_W = 76.f, DH_CELL_H = 76.f;
static const float DH_COL[2] = {48.f, 132.f};
static const float DH_ROW_TOP[2] = {96.f, 180.f};
static const float DH_CELL_LABEL = 10.f; // offsets from the plate top
static const float DH_CELL_KNOB = 42.f;

static const float DH_SEC2_Y = 288.f;
static const float DH_IO_TOP = 296.f, DH_IO_H = 52.f, DH_IO_W = 38.f;
static const float DH_IO_LABEL = 10.f, DH_IO_JACK = 34.f;
static float dhIoX(int i) { return 25.5f + 43.f * i; }

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

    const int ALIGN_C = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE;

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

    struct DhCellDef
    {
        const char *name;
        int param;
    };
    const DhCellDef DH_CELLS[4] = {
        {"MIX", DarkHall::MIX_PARAM},
        {"SIZE", DarkHall::SIZE_PARAM},
        {"DAMP", DarkHall::DAMP_PARAM},
        {"MOD", DarkHall::MOD_PARAM},
    };
    const char *DH_IO_NAMES[4] = {"L IN", "R IN", "L OUT", "R OUT"};

    // ------------------------------------------------------------
    // Panel background (static artwork only)
    // ------------------------------------------------------------
    struct DhPanelBackground : TransparentWidget
    {
        DhPanelBackground() { box.size = Vec(DH_PANEL_W, DH_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, DH_PANEL_W, DH_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DH_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            drawText(vg, DH_PANEL_W / 2.f, DH_TITLE_Y, "DARK HALL", 12.f, colText(), ALIGN_C, 3.f);
            drawText(vg, DH_PANEL_W / 2.f, DH_MAKER_Y, "AGMM", 9.f, colDim(), ALIGN_C, 3.f);

            // Display bezel
            nvgBeginPath(vg);
            nvgRoundedRect(vg, DH_DISP_X - 4.f, DH_DISP_Y - 4.f,
                           DH_DISP_W + 8.f, DH_DISP_H + 8.f, 5.f);
            nvgFillColor(vg, nvgRGB(0x0b, 0x0c, 0x0e));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // ---- REVERB: 2 x 2 cells (label above the knob) ----
            sectionHeader(vg, DH_PANEL_W, DH_SEC1_Y, "REVERB");
            for (int i = 0; i < 4; ++i)
            {
                float cx = DH_COL[i % 2];
                float top = DH_ROW_TOP[i / 2];
                plate(vg, cx, top, DH_CELL_W, DH_CELL_H);
                drawText(vg, cx, top + DH_CELL_LABEL, DH_CELLS[i].name, 8.f, colDim(), ALIGN_C, 0.5f);
            }

            // ---- I / O (inputs left, outputs right, outputs tinted) ----
            sectionHeader(vg, DH_PANEL_W, DH_SEC2_Y, "I / O");
            for (int i = 0; i < 4; ++i)
            {
                float cx = dhIoX(i);
                plate(vg, cx, DH_IO_TOP, DH_IO_W, DH_IO_H, i >= 2);
                drawText(vg, cx, DH_IO_TOP + DH_IO_LABEL, DH_IO_NAMES[i], 7.f, colDim(), ALIGN_C, 0.5f);
            }

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope + L/R level meters. Readouts live in their own row below
    // the display, never over the waveform.
    // ------------------------------------------------------------
    struct DhDisplayWidget : TransparentWidget
    {
        DarkHall *module = nullptr;

        DhDisplayWidget(DarkHall *m) : module(m)
        {
            box.pos = Vec(DH_DISP_X, DH_DISP_Y);
            box.size = Vec(DH_DISP_W, DH_DISP_H);
        }

        static float scopeW() { return DH_DISP_W - 16.f; }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, DH_DISP_W, DH_DISP_H);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
            nvgFill(vg);

            nvgBeginPath(vg);
            for (float x = 0.f; x <= scopeW(); x += 28.f)
            {
                nvgMoveTo(vg, x + 0.5f, 0);
                nvgLineTo(vg, x + 0.5f, DH_DISP_H);
            }
            nvgMoveTo(vg, 0, DH_DISP_H / 2.f + 0.5f);
            nvgLineTo(vg, scopeW(), DH_DISP_H / 2.f + 0.5f);
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

                const float mid = DH_DISP_H / 2.f;
                const float amp = mid - 2.f;
                const float sw = scopeW();
                const float colW = sw / (float)DH_SCOPE_N;

                // Waveform
                nvgSave(vg);
                nvgScissor(vg, 0, 0, sw, DH_DISP_H);
                nvgBeginPath(vg);
                for (int i = 0; i < DH_SCOPE_N; ++i)
                {
                    float vmin, vmax;
                    if (module)
                    {
                        int idx = (module->scopeHead + i) % DH_SCOPE_N;
                        vmin = module->scopeMin[idx];
                        vmax = module->scopeMax[idx];
                    }
                    else
                    {
                        float t = (float)i / DH_SCOPE_N;
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
                    nvgRect(vg, x, 1.f, 4.f, DH_DISP_H - 2.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                    nvgFill(vg);
                    if (lvl > 0.01f)
                    {
                        float h = lvl * (DH_DISP_H - 2.f);
                        nvgBeginPath(vg);
                        nvgRect(vg, x, DH_DISP_H - 1.f - h, 4.f, h);
                        nvgFillPaint(vg, nvgLinearGradient(vg, 0, DH_DISP_H, 0, 0,
                                                           colAccent(), colAccent2()));
                        nvgFill(vg);
                    }
                }

                // Readouts: own row under the display, centred over the two columns
                char buf[32];
                float mx = module ? module->params[DarkHall::MIX_PARAM].getValue() : 0.4f;
                float sz = module ? module->params[DarkHall::SIZE_PARAM].getValue() : 0.7f;
                const float ry = DH_READ_Y - DH_DISP_Y;
                snprintf(buf, sizeof(buf), "MIX %3.0f%%", mx * 100.f);
                drawText(vg, DH_COL[0] - DH_DISP_X, ry, buf, 7.f, colAccent(), ALIGN_C, 0.5f);
                snprintf(buf, sizeof(buf), "SIZE %3.0f%%", sz * 100.f);
                drawText(vg, DH_COL[1] - DH_DISP_X, ry, buf, 7.f, colAccent(), ALIGN_C, 0.5f);
            }
            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Module widget
// ============================================================
struct DarkHallWidget : ModuleWidget
{
    DarkHallWidget(DarkHall *module)
    {
        setModule(module);
        box.size = Vec(DH_PANEL_W, DH_PANEL_H);

        addChild(new DhPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new DhDisplayWidget(module));

        // 4 knobs (2 x 2), centred under their labels
        for (int i = 0; i < 4; ++i)
        {
            float cx = DH_COL[i % 2];
            float top = DH_ROW_TOP[i / 2];
            addParam(createParamCentered<RoundBlackKnob>(
                Vec(cx, top + DH_CELL_KNOB), module, DH_CELLS[i].param));
        }

        // I/O: L IN, R IN | L OUT, R OUT
        const float jy = DH_IO_TOP + DH_IO_JACK;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(dhIoX(0), jy), module, DarkHall::IN_L));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(dhIoX(1), jy), module, DarkHall::IN_R));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(dhIoX(2), jy), module, DarkHall::OUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(dhIoX(3), jy), module, DarkHall::OUT_R));
    }
};

Model *modelDarkHall = createModel<DarkHall, DarkHallWidget>("DarkHall");