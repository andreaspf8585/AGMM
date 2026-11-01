#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ============================================================
// Bastion - Multi-Engine Filter Module
// Engines: Moog Ladder, TB-303 Diode, MS-20, SEM SVF,
//          Steiner-Parker, CEM3320, Roland OTA, EMS Diode,
//          EDP Wasp, Buchla LPG
// ============================================================

enum BastionEngine {
    ENGINE_MOOG = 0,
    ENGINE_TB303,
    ENGINE_MS20,
    ENGINE_SEM,
    ENGINE_STEINER,
    ENGINE_CEM3320,
    ENGINE_ROLAND_OTA,
    ENGINE_EMS_DIODE,
    ENGINE_WASP,
    ENGINE_BUCHLA_LPG,
    NUM_ENGINES
};

// ---------- DSP (unchanged) ----------

struct LadderState { float s[4] = {0.f, 0.f, 0.f, 0.f}; };

static inline float onePoleLP(float in, float& state, float g) {
    float G = g / (1.f + g);
    float v = (in - state) * G;
    float out = v + state;
    state = out + v;
    return out;
}

static float processLadder(float in, LadderState& st, float fc, float res, float drive, float fs, bool diode) {
    float fcHz = clamp(fc, 10.f, fs * 0.45f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float k = clamp(res, 0.f, 1.f) * (diode ? 3.6f : 4.0f);
    float x = in * (1.f + drive * 4.f);
    x = diode ? std::tanh(x * 1.2f) : std::tanh(x);
    float fb = st.s[3] * k;
    if (diode) fb = std::tanh(fb * 0.9f) * 1.1f;
    float u = x - fb;
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = onePoleLP(u, st.s[3], g);
    return u;
}

static float processMS20(float in, LadderState& st, float fc, float res, float drive, float fs) {
    float fcHz = clamp(fc, 10.f, fs * 0.45f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float x = std::tanh(in * (1.f + drive * 3.f));
    float k = clamp(res, 0.f, 1.f) * 3.8f;
    static thread_local float hp1 = 0.f, hp2 = 0.f;
    float gHP = std::tan((float)M_PI * clamp(fcHz * 0.5f, 10.f, fs * 0.4f) / fs);
    float hpIn = x - st.s[3] * k;
    hp1 = onePoleLP(hpIn, hp1, gHP);
    float hpf = hpIn - hp1;
    hp2 = onePoleLP(hpf, hp2, gHP);
    hpf = hpf - hp2;
    float y = hpf;
    y = onePoleLP(y, st.s[0], g);
    y = onePoleLP(y, st.s[1], g);
    st.s[3] = y;
    return y;
}

struct SVFState { float lp = 0.f, bp = 0.f, hp = 0.f; };

static void processSVF(float in, SVFState& st, float fc, float res, float drive, float fs) {
    float fcHz = clamp(fc, 20.f, fs * 0.25f);
    float f = 2.f * std::sin((float)M_PI * fcHz / fs);
    float q = clamp(1.f - res, 0.02f, 1.f);
    float x = std::tanh(in * (1.f + drive * 3.f));
    float hp = x - st.lp - q * st.bp;
    float bp = f * hp + st.bp;
    float lp = f * bp + st.lp;
    if (std::fabs(lp) < 1e-12f) lp = 0.f;
    if (std::fabs(bp) < 1e-12f) bp = 0.f;
    st.hp = hp; st.bp = bp; st.lp = lp;
}

struct SPState { float s1 = 0.f, s2 = 0.f, bp = 0.f; };

static void processSteiner(float in, SPState& st, float fc, float res, float drive, float fs) {
    float fcHz = clamp(fc, 20.f, fs * 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float k = clamp(res, 0.f, 1.f) * 3.5f;
    float x = std::tanh(in * (1.f + drive * 3.f));
    float fb = st.s2 * k * 0.9f;
    float hp = x - st.s1 - fb;
    float lp1 = onePoleLP(hp, st.s1, g);
    float lp2 = onePoleLP(lp1, st.s2, g);
    st.bp = hp - lp1 - lp2;
}

static float processCEM3320(float in, LadderState& st, float fc, float res, float drive, float fs) {
    float fcHz = clamp(fc, 10.f, fs * 0.45f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float k = clamp(res, 0.f, 1.f) * 3.9f;
    float x = std::tanh(in * (1.f + drive * 3.5f));
    float u = x - std::tanh(st.s[3] * k * 0.85f);
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = onePoleLP(u, st.s[3], g);
    return std::tanh(u * 1.1f);
}

static float processRolandOTA(float in, LadderState& st, float fc, float res, float drive, float fs) {
    float fcHz = clamp(fc, 10.f, fs * 0.45f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float k = clamp(res, 0.f, 1.f) * 3.7f;
    float x = std::tanh(in * (1.f + drive * 3.f));
    float fb = std::tanh(st.s[3] * k * 0.9f);
    float u = x - fb;
    u = onePoleLP(u, st.s[0], g);
    u = std::tanh(u * 1.1f);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = std::tanh(u * 1.05f);
    u = onePoleLP(u, st.s[3], g);
    return u;
}

static float processWasp(float in, LadderState& st, float fc, float res, float drive, float fs) {
    float fcHz = clamp(fc, 10.f, fs * 0.45f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float k = clamp(res, 0.f, 1.f) * 4.2f;
    float x = std::tanh(in * (1.f + drive * 6.f));
    float fb = st.s[2] * k;
    float u = x - fb;
    u = std::tanh(u);
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    return std::tanh(u * 1.2f);
}

struct LPGState { float vactrol = 0.f; };

static float processLPG(float in, LPGState& st, float fc, float res, float drive, float fs) {
    float level = clamp(fc / 1000.f, 0.f, 1.f);
    float target = level;
    float lag = 0.0015f + (1.f - level) * 0.03f;
    st.vactrol += (target - st.vactrol) * lag * (fs / 48000.f);
    if (st.vactrol < 0.f) st.vactrol = 0.f;
    if (st.vactrol > 1.f) st.vactrol = 1.f;
    float vca = in * st.vactrol * (1.f + drive * 2.f);
    float lpgFc = 200.f + st.vactrol * 12000.f;
    float g = std::tan((float)M_PI * clamp(lpgFc, 20.f, fs * 0.45f) / fs);
    static thread_local float lp = 0.f;
    lp = onePoleLP(vca, lp, g);
    return std::tanh(lp * 1.3f);
}

// ============================================================
// Panel layout constants — DOUBLED WIDTH
// ============================================================
static const float BS_W = RACK_GRID_WIDTH * 64;   // 480 (doubled)
static const float BS_H = RACK_GRID_HEIGHT;       // 380

// ---- Left column (signal path) ----
static const float BS_MAIN_Y   = 90.f;    // Cutoff / Res / Drive knobs
static const float BS_SIGIN_Y  = 170.f;   // Signal input jacks (LP/BP/HP)
static const float BS_OUT_Y    = 300.f;   // Output jacks (LP/BP/HP)

// ---- Right column (engine + CV) ----
static const float BS_ENGINE_Y = 70.f;    // Engine selector + lights
static const float BS_CVATT_Y  = 160.f;   // CV attenuverters
static const float BS_CVIN_Y   = 220.f;   // CV input jacks

// Column X positions
// Left half occupies 0..240, right half 240..480
static float bsLeftX(int i)  { return 60.f + 60.f * i; }   // 60, 120, 180
static float bsRightX(int i) { return 300.f + 60.f * i; }  // 300, 360, 420
static const float BS_RIGHT_CX = 360.f;                    // right half center

// ============================================================
struct Bastion : Module {
    enum ParamId {
        ENGINE_PARAM,
        CUTOFF_PARAM, RES_PARAM, DRIVE_PARAM,
        CUTOFF_CV_PARAM, RES_CV_PARAM, DRIVE_CV_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        CUTOFF_INPUT, RES_INPUT, DRIVE_INPUT,
        LP_INPUT, BP_INPUT, HP_INPUT,
        INPUTS_LEN
    };
    enum OutputId { LP_OUTPUT, BP_OUTPUT, HP_OUTPUT, OUTPUTS_LEN };
    enum LightId { LIGHT_ENGINE_A, LIGHT_ENGINE_B, LIGHTS_LEN };

    LadderState ladder;
    SVFState    svf;
    SPState     sp;
    LPGState    lpg;

    float lpState = 0.f, bpState = 0.f, hpState = 0.f;
    int lastEngine = -1;
    float enginePulse = 0.f;

    Bastion() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configSwitch(ENGINE_PARAM, 0.f, NUM_ENGINES - 1, 0.f, "Engine",
            {"Moog Ladder", "TB-303 Diode", "MS-20", "SEM SVF",
             "Steiner-Parker", "CEM3320", "Roland OTA",
             "EMS Diode", "EDP Wasp", "Buchla LPG"});

        configParam(CUTOFF_PARAM, 0.f, 1.f, 0.5f, "Cutoff");
        configParam(RES_PARAM,    0.f, 1.f, 0.3f, "Resonance");
        configParam(DRIVE_PARAM,  0.f, 1.f, 0.f,  "Drive");
        configParam(CUTOFF_CV_PARAM, -1.f, 1.f, 0.f, "Cutoff CV amount");
        configParam(RES_CV_PARAM,    -1.f, 1.f, 0.f, "Resonance CV amount");
        configParam(DRIVE_CV_PARAM,  -1.f, 1.f, 0.f, "Drive CV amount");

        configInput(CUTOFF_INPUT, "Cutoff CV");
        configInput(RES_INPUT,    "Resonance CV");
        configInput(DRIVE_INPUT,  "Drive CV");
        configInput(LP_INPUT, "LP input");
        configInput(BP_INPUT, "BP input");
        configInput(HP_INPUT, "HP input");

        configOutput(LP_OUTPUT, "LP");
        configOutput(BP_OUTPUT, "BP");
        configOutput(HP_OUTPUT, "HP");

        configLight(LIGHT_ENGINE_A, "Engine A");
        configLight(LIGHT_ENGINE_B, "Engine B");
    }

    void onReset() override {
        ladder = LadderState(); svf = SVFState();
        sp = SPState(); lpg = LPGState();
        lpState = bpState = hpState = 0.f;
        lastEngine = -1;
    }

    float computeCutoff(float base, float cvAmt, float cv) {
        float norm = clamp(base, 0.f, 1.f);
        float hz = 20.f * std::pow(1000.f, norm);
        float cvShift = cvAmt * cv;
        hz *= std::pow(2.f, cvShift);
        return clamp(hz, 10.f, 20000.f);
    }

    void process(const ProcessArgs& args) override {
        int engine = (int)std::round(params[ENGINE_PARAM].getValue());
        engine = clamp(engine, 0, NUM_ENGINES - 1);
        if (engine != lastEngine) {
            ladder = LadderState(); svf = SVFState();
            sp = SPState(); lpg = LPGState();
            lastEngine = engine;
            enginePulse = 1.f;
        }
        enginePulse *= 0.92f;

        float fs = args.sampleRate;

        float cvCut = 0.f, cvRes = 0.f, cvDrive = 0.f;
        if (inputs[CUTOFF_INPUT].isConnected()) cvCut   = inputs[CUTOFF_INPUT].getVoltage();
        if (inputs[RES_INPUT].isConnected())    cvRes   = inputs[RES_INPUT].getVoltage();
        if (inputs[DRIVE_INPUT].isConnected())  cvDrive = inputs[DRIVE_INPUT].getVoltage();

        float cutoff = computeCutoff(
            params[CUTOFF_PARAM].getValue(),
            params[CUTOFF_CV_PARAM].getValue(),
            cvCut * 0.2f);

        float res = clamp(params[RES_PARAM].getValue()
                        + params[RES_CV_PARAM].getValue() * cvRes * 0.2f, 0.f, 1.f);
        float drive = clamp(params[DRIVE_PARAM].getValue()
                        + params[DRIVE_CV_PARAM].getValue() * cvDrive * 0.2f, 0.f, 1.f);

        float inL = 0.f, inB = 0.f, inH = 0.f;
        bool anyIn = false;
        if (inputs[LP_INPUT].isConnected()) { inL = inputs[LP_INPUT].getVoltage(); anyIn = true; }
        if (inputs[BP_INPUT].isConnected()) { inB = inputs[BP_INPUT].getVoltage(); anyIn = true; }
        if (inputs[HP_INPUT].isConnected()) { inH = inputs[HP_INPUT].getVoltage(); anyIn = true; }
        float input = anyIn ? (inL + inB + inH) : 0.f;

        float outLP = 0.f, outBP = 0.f, outHP = 0.f;

        switch (engine) {
            case ENGINE_MOOG: {
                outLP = processLadder(input, ladder, cutoff, res, drive, fs, false);
                outBP = ladder.s[1] - ladder.s[2];
                outHP = input - outLP - outBP;
            } break;
            case ENGINE_TB303: {
                outLP = processLadder(input, ladder, cutoff, res, drive, fs, true);
                outBP = ladder.s[1] - ladder.s[2];
                outHP = input - outLP;
            } break;
            case ENGINE_MS20: {
                outLP = processMS20(input, ladder, cutoff, res, drive, fs);
                outBP = ladder.s[0] - ladder.s[1];
                outHP = input - outLP;
            } break;
            case ENGINE_SEM: {
                processSVF(input, svf, cutoff, res, drive, fs);
                outLP = svf.lp; outBP = svf.bp; outHP = svf.hp;
            } break;
            case ENGINE_STEINER: {
                processSteiner(input, sp, cutoff, res, drive, fs);
                outLP = sp.s2;
                outBP = sp.bp;
                outHP = input - outLP - outBP;
            } break;
            case ENGINE_CEM3320: {
                outLP = processCEM3320(input, ladder, cutoff, res, drive, fs);
                outBP = ladder.s[1] - ladder.s[2];
                outHP = input - outLP;
            } break;
            case ENGINE_ROLAND_OTA: {
                outLP = processRolandOTA(input, ladder, cutoff, res, drive, fs);
                outBP = ladder.s[1] - ladder.s[2];
                outHP = input - outLP;
            } break;
            case ENGINE_EMS_DIODE: {
                outLP = processLadder(input, ladder, cutoff, res, drive, fs, true);
                outBP = ladder.s[1] - ladder.s[2];
                outHP = input - outLP;
            } break;
            case ENGINE_WASP: {
                outLP = processWasp(input, ladder, cutoff, res, drive, fs);
                outBP = ladder.s[0] - ladder.s[1];
                outHP = input - outLP;
            } break;
            case ENGINE_BUCHLA_LPG: {
                outLP = processLPG(input, lpg, cutoff, res, drive, fs);
                outBP = outLP * 0.7f;
                outHP = input - outLP;
            } break;
        }

        outLP = clamp(outLP, -12.f, 12.f);
        outBP = clamp(outBP, -12.f, 12.f);
        outHP = clamp(outHP, -12.f, 12.f);

        outputs[LP_OUTPUT].setVoltage(outLP);
        outputs[BP_OUTPUT].setVoltage(outBP);
        outputs[HP_OUTPUT].setVoltage(outHP);

        lights[LIGHT_ENGINE_A].setBrightness(enginePulse);
        lights[LIGHT_ENGINE_B].setBrightness(clamp(std::fabs(outLP) * 0.15f, 0.f, 1.f));
    }
};

// ============================================================
// UI helpers
// ============================================================
namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colHi()       { return nvgRGB(0x50, 0xff, 0x90); }
NVGcolor colMag()      { return nvgRGB(0xb0, 0x5a, 0xff); }

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

void label(NVGcontext* vg, float x, float y, const char* t, float size = 7.f, NVGcolor c = colDim()) {
    drawText(vg, x, y, t, size, c, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
}

void sectionHeader(NVGcontext* vg, float x, float y, float w, const char* title) {
    nvgBeginPath(vg);
    nvgMoveTo(vg, x, y);
    nvgLineTo(vg, x + w, y);
    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
    drawText(vg, x + w / 2.f, y - 8.f, title, 6.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);
}

// Vertical divider between left (signal) and right (engine+CV) halves
void verticalDivider(NVGcontext* vg, float x, float y0, float y1) {
    nvgBeginPath(vg);
    nvgMoveTo(vg, x, y0);
    nvgLineTo(vg, x, y1);
    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

} // namespace

// ============================================================
struct BastionPanel : TransparentWidget {
    BastionPanel() { box.size = Vec(BS_W, BS_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, BS_W, BS_H, "BASTION");

        // Vertical divider: left signal path | right engine+CV
        verticalDivider(vg, 240.f, 50.f, 350.f);

        // ---- LEFT: FILTER (main knobs) ----
        sectionHeader(vg, 20.f, BS_MAIN_Y - 28.f, 200.f, "FILTER");
        const char* mainTitles[3] = {"CUTOFF", "RES", "DRIVE"};
        for (int i = 0; i < 3; ++i) {
            float cx = bsLeftX(i);
            ring(vg, cx, BS_MAIN_Y, 22.f);
            label(vg, cx, BS_MAIN_Y - 30.f, mainTitles[i], 7.f);
        }

        // ---- LEFT: INPUTS ----
        sectionHeader(vg, 20.f, BS_SIGIN_Y - 32.f, 200.f, "INPUTS");
        label(vg, bsLeftX(0), BS_SIGIN_Y + 24.f, "LP", 7.f, colHi());
        label(vg, bsLeftX(1), BS_SIGIN_Y + 24.f, "BP", 7.f, colAccent());
        label(vg, bsLeftX(2), BS_SIGIN_Y + 24.f, "HP", 7.f, colMag());

        // ---- LEFT: OUTPUTS ----
        sectionHeader(vg, 20.f, BS_OUT_Y - 32.f, 200.f, "OUTPUTS");
        label(vg, bsLeftX(0), BS_OUT_Y + 24.f, "LP", 7.f, colHi());
        label(vg, bsLeftX(1), BS_OUT_Y + 24.f, "BP", 7.f, colAccent());
        label(vg, bsLeftX(2), BS_OUT_Y + 24.f, "HP", 7.f, colMag());

        // ---- RIGHT: ENGINE ----
        ring(vg, BS_RIGHT_CX, BS_ENGINE_Y, 26.f);
        label(vg, BS_RIGHT_CX, BS_ENGINE_Y - 34.f, "ENGINE", 7.f, colAccent());

        // ---- RIGHT: CV ----
        sectionHeader(vg, 260.f, BS_CVATT_Y - 20.f, 200.f, "CV");
        const char* cvTitles[3] = {"CUT", "RES", "DRV"};
        for (int i = 0; i < 3; ++i) {
            float cx = bsRightX(i);
            plate(vg, cx - 12.f, BS_CVATT_Y - 12.f, 24.f, 24.f);
            label(vg, cx, BS_CVATT_Y - 20.f, cvTitles[i], 6.f);
            plate(vg, cx - 12.f, BS_CVIN_Y - 12.f, 24.f, 24.f);
        }
        label(vg, BS_RIGHT_CX, BS_CVIN_Y + 24.f, "MODULATION", 6.f, colDim());

        Widget::draw(args);
    }
};

// ============================================================
// Engine selector knob
// ============================================================
struct BastionEngineKnob : SvgKnob {
    BastionEngineKnob() {
        minAngle = -0.83f * M_PI;
        maxAngle = 0.83f * M_PI;
        setSvg(APP->window->loadSvg(asset::system("res/ComponentLibrary/Rogan1.svg")));
        box.size = Vec(36.f, 36.f);
    }
};

// ============================================================
struct BastionWidget : ModuleWidget {
    BastionWidget(Bastion* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/Bastion.svg")));

        // Panel (doubled width)
        addChild(new BastionPanel());

        // ========================================================
        // LEFT COLUMN — signal path
        // ========================================================

        // Main knobs: Cutoff / Res / Drive
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(bsLeftX(0), BS_MAIN_Y), module, Bastion::CUTOFF_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(bsLeftX(1), BS_MAIN_Y), module, Bastion::RES_PARAM));
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(bsLeftX(2), BS_MAIN_Y), module, Bastion::DRIVE_PARAM));

        // Signal inputs: LP / BP / HP
        addInput(createInputCentered<PJ301MPort>(
            Vec(bsLeftX(0), BS_SIGIN_Y), module, Bastion::LP_INPUT));
        addInput(createInputCentered<PJ301MPort>(
            Vec(bsLeftX(1), BS_SIGIN_Y), module, Bastion::BP_INPUT));
        addInput(createInputCentered<PJ301MPort>(
            Vec(bsLeftX(2), BS_SIGIN_Y), module, Bastion::HP_INPUT));

        // Outputs: LP / BP / HP
        addOutput(createOutputCentered<PJ301MPort>(
            Vec(bsLeftX(0), BS_OUT_Y), module, Bastion::LP_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(
            Vec(bsLeftX(1), BS_OUT_Y), module, Bastion::BP_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(
            Vec(bsLeftX(2), BS_OUT_Y), module, Bastion::HP_OUTPUT));

        // ========================================================
        // RIGHT COLUMN — engine + CV modulation
        // ========================================================

        // Engine selector (top center of right half)
        addParam(createParamCentered<BastionEngineKnob>(
            Vec(BS_RIGHT_CX, BS_ENGINE_Y), module, Bastion::ENGINE_PARAM));

        // Engine indicator lights (flanking the knob, wider spacing)
        addChild(createLightCentered<SmallLight<GreenLight>>(
            Vec(BS_RIGHT_CX - 56.f, BS_ENGINE_Y), module, Bastion::LIGHT_ENGINE_A));
                addChild(createLightCentered<SmallLight<GreenLight>>(
            Vec(BS_RIGHT_CX + 56.f, BS_ENGINE_Y), module, Bastion::LIGHT_ENGINE_B));

        // CV attenuverters: Cut / Res / Drv
        addParam(createParamCentered<Trimpot>(
            Vec(bsRightX(0), BS_CVATT_Y), module, Bastion::CUTOFF_CV_PARAM));
        addParam(createParamCentered<Trimpot>(
            Vec(bsRightX(1), BS_CVATT_Y), module, Bastion::RES_CV_PARAM));
        addParam(createParamCentered<Trimpot>(
            Vec(bsRightX(2), BS_CVATT_Y), module, Bastion::DRIVE_CV_PARAM));

        // CV inputs: Cut / Res / Drv
        addInput(createInputCentered<PJ301MPort>(
            Vec(bsRightX(0), BS_CVIN_Y), module, Bastion::CUTOFF_INPUT));
        addInput(createInputCentered<PJ301MPort>(
            Vec(bsRightX(1), BS_CVIN_Y), module, Bastion::RES_INPUT));
        addInput(createInputCentered<PJ301MPort>(
            Vec(bsRightX(2), BS_CVIN_Y), module, Bastion::DRIVE_INPUT));

        // Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    }
};

// ============================================================
Model* modelBastion = createModel<Bastion, BastionWidget>("Bastion");