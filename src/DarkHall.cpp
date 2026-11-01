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
    int   scopeHead = 0;
    int   scopeCount = 0;
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

        configParam(MIX_PARAM, 0.f, 1.f, 0.4f, "Mix");
        configParam(SIZE_PARAM, 0.f, 1.f, 0.7f, "Room Size");
        configParam(DAMP_PARAM, 0.f, 1.f, 0.6f, "Damping");
        configParam(MOD_PARAM, 0.f, 1.f, 0.3f, "Modulation");

        configInput(IN_L, "Left In");
        configInput(IN_R, "Right In");

        configOutput(OUT_L, "Left Out");
        configOutput(OUT_R, "Right Out");
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
        if (scopeCount == 0) scopeBMin = scopeBMax = mono;
        else {
            scopeBMin = std::min(scopeBMin, mono);
            scopeBMax = std::max(scopeBMax, mono);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket) {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % DH_SCOPE_N;
        }
    }
};

// ============================================================
// Layout constants (12 HP = 180 px)
// ============================================================
static const float DH_PANEL_W = RACK_GRID_WIDTH * 12;   // 180
static const float DH_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float DH_DISP_W = 150.f;
static const float DH_DISP_H = 34.f;
static const float DH_DISP_X = (DH_PANEL_W - DH_DISP_W) / 2.f;
static const float DH_DISP_Y = 46.f;

static const float DH_CELL_W = 60.f;
static const float DH_CELL_H = 70.f;
static const float DH_ROW[2] = {145.f, 225.f};
static const float DH_COL[2] = {45.f, 135.f};
static const float DH_PORT_Y = 322.f;

// ============================================================
// Theme + drawing helpers (anonymous namespace)
// ============================================================
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colAccent2()  { return nvgRGB(0xb0, 0x5a, 0xff); }

void setFont(NVGcontext* vg) {
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font)
        nvgFontFaceId(vg, font->handle);
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

struct DhCellDef { const char* name; int param; };
const DhCellDef DH_CELLS[4] = {
    {"MIX",   DarkHall::MIX_PARAM},
    {"SIZE",  DarkHall::SIZE_PARAM},
    {"DAMP",  DarkHall::DAMP_PARAM},
    {"MOD",   DarkHall::MOD_PARAM},
};

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct DhPanelBackground : TransparentWidget {
    DhPanelBackground() { box.size = Vec(DH_PANEL_W, DH_PANEL_H); }

    void plate(NVGcontext* vg, float cx, float ry) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - DH_CELL_W / 2.f, ry - DH_CELL_H / 2.f,
                       DH_CELL_W, DH_CELL_H, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = DH_PANEL_W - 14.f, cx = DH_PANEL_W / 2.f;
        float w = 10.f + (float) strlen(txt) * 7.f;
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

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DH_PANEL_W, DH_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DH_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) DH_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, DH_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, DH_PANEL_W - 1.f, DH_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, DH_PANEL_W / 2.f, 13.f);
        drawText(vg, DH_PANEL_W / 2.f, 31.f, "DARK HALL", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // Display bezel
        float bx = DH_DISP_X - 5.f, by = DH_DISP_Y - 5.f;
        float bw = DH_DISP_W + 10.f, bh = DH_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // 4 control cells
        for (int i = 0; i < 4; ++i) {
            int row = i / 2, col = i % 2;
            float cx = DH_COL[col], ry = DH_ROW[row];
            plate(vg, cx, ry);
            drawText(vg, cx, ry + 18.f, DH_CELLS[i].name, 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // I/O section
        section(vg, 290.f, "I / O");
        const char* io[4] = {"L IN", "R IN", "L OUT", "R OUT"};
        for (int i = 0; i < 4; ++i) {
            float cx = 22.5f + i * 45.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 20.f, 296.f, 40.f, 42.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, cx, 304.f, io[i], 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Scope display + L/R level meters
// ------------------------------------------------------------
struct DhDisplayWidget : TransparentWidget {
    DarkHall* module = nullptr;

    DhDisplayWidget(DarkHall* m) : module(m) {
        box.pos = Vec(DH_DISP_X, DH_DISP_Y);
        box.size = Vec(DH_DISP_W, DH_DISP_H);
    }

    static float scopeW() { return DH_DISP_W - 16.f; }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DH_DISP_W, DH_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, DH_DISP_W / 2.f, DH_DISP_H / 2.f, 20.f,
                                           DH_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        nvgBeginPath(vg);
        for (float x = 0.f; x <= scopeW(); x += 28.f) {
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

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        const float mid  = DH_DISP_H / 2.f;
        const float amp  = mid - 2.f;
        const float sw   = scopeW();
        const float colW = sw / (float) DH_SCOPE_N;

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, DH_DISP_H);
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < DH_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % DH_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / DH_SCOPE_N;
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

        // L / R level meters
        for (int ch = 0; ch < 2; ++ch) {
            float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 3.f : 2.4f);
            float lvl = std::min(peak / 10.f, 1.f);
            float x = sw + 3.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 1.f, 4.f, DH_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (DH_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, DH_DISP_H - 1.f - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, DH_DISP_H, 0, 0,
                                                   colAccent(), colAccent2()));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) DH_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, DH_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DH_DISP_W, DH_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DH_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16),
                                           nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[32];
        float mx = module ? module->params[DarkHall::MIX_PARAM].getValue()  : 0.4f;
        float sz = module ? module->params[DarkHall::SIZE_PARAM].getValue() : 0.7f;
        snprintf(buf, sizeof(buf), "MIX %3.0f%%", mx * 100.f);
        drawText(vg, 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "SIZE %3.0f%%", sz * 100.f);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ============================================================
// Module widget
// ============================================================
struct DarkHallWidget : ModuleWidget {
    DarkHallWidget(DarkHall* module) {
        setModule(module);
        box.size = Vec(DH_PANEL_W, DH_PANEL_H);

        addChild(new DhPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new DhDisplayWidget(module));

        // 4 knobs (2×2)
        for (int i = 0; i < 4; ++i) {
            int row = i / 2, col = i % 2;
            float cx = DH_COL[col], ry = DH_ROW[row];
            addParam(createParamCentered<RoundBlackKnob>(Vec(cx, ry - 8.f), module, DH_CELLS[i].param));
        }

        // I/O – 4 jacks in a row: L IN, R IN, L OUT, R OUT
        const int ioIds[4] = {
            DarkHall::IN_L,
            DarkHall::IN_R,
            DarkHall::OUT_L,
            DarkHall::OUT_R,
        };
        for (int i = 0; i < 4; ++i) {
            float cx = 22.5f + i * 45.f;
            if (i < 2)
                addInput(createInputCentered<DarkPJ301MPort>(Vec(cx, DH_PORT_Y), module, ioIds[i]));
            else
                addOutput(createOutputCentered<DarkPJ301MPort>(Vec(cx, DH_PORT_Y), module, ioIds[i]));
        }
    }
};

Model *modelDarkHall = createModel<DarkHall, DarkHallWidget>("DarkHall");