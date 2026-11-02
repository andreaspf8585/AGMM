#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <complex>

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

static const char* const ENGINE_NAMES[NUM_ENGINES] = {
    "Moog Ladder", "TB-303 Diode", "MS-20", "SEM SVF",
    "Steiner-Parker", "CEM3320", "Roland OTA",
    "EMS Diode", "EDP Wasp", "Buchla LPG"
};

static const char* const ENGINE_DESCS[NUM_ENGINES] = {
    "4-POLE TRANSISTOR LADDER",
    "DIODE LADDER",
    "2-POLE + HP FEEDBACK",
    "2-POLE STATE VARIABLE",
    "2-POLE MULTIMODE",
    "4-POLE OTA LADDER",
    "4-POLE OTA CASCADE",
    "DIODE LADDER",
    "3-POLE DRIVEN",
    "VACTROL LOW-PASS GATE"
};

// ---------- DSP ----------

struct LadderState {
    float s[4] = {0.f, 0.f, 0.f, 0.f};
    float h[2] = {0.f, 0.f};   // extra state used by the MS-20 high-pass stages
};

static inline float onePoleLP(float in, float& state, float g) {
    float G = g / (1.f + g);
    float v = (in - state) * G;
    float out = v + state;
    state = out + v;
    return out;
}

// Soft clippers that work in the +-5 V audio domain (linear for small signals)
static inline float softClip(float v, float lim) { return lim * std::tanh(v / lim); }
static inline float sat5(float v) { return 5.f * std::tanh(v * 0.2f); }

// Drive stage: 0..1 -> 0..~25 dB of gain into an asymmetric tanh, with makeup
// gain compensation. Adds even + odd harmonics, getting harder as drive rises.
static inline float driveStage(float in, float d, float asym = 0.f) {
    d = clamp(d, 0.f, 1.5f);
    float gain = std::exp(d * 2.9f);               // 1x .. ~18x (more past 1.0)
    float bias = d * (0.10f + asym);
    float y = std::tanh(in * 0.2f * gain + bias) - std::tanh(bias);
    return y * 5.f / (1.f + d * 0.45f);
}

static inline float fcLimit(float fc, float fs, float frac) {
    return clamp(fc, 10.f, fs * frac);
}

// Zero-delay-feedback prediction for a chain of n one-pole stages with
// global negative feedback k. Returns the predicted output of the last stage,
// so the feedback can be applied without a unit-delay (stable up to high fc).
static inline float ladderPredict(const float* s, int n, float G, float x, float k) {
    float a = 1.f - G;
    float S = 0.f, Gn = 1.f;
    for (int i = 0; i < n; ++i) { S = S * G + a * s[i]; Gn *= G; }
    return (Gn * x + S) / (1.f + k * Gn);
}

static float processLadder(float in, LadderState& st, float fc, float res, float drive, float fs, bool diode) {
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float k = clamp(res, 0.f, 1.f) * (diode ? 3.6f : 4.0f);
    float x = driveStage(in, drive, diode ? 0.08f : 0.f);
    float y4 = ladderPredict(st.s, 4, G, x, k);
    float fb = diode ? sat5(k * y4 * 0.9f) * 1.1f : sat5(k * y4);
    float u = x - fb;
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = onePoleLP(u, st.s[3], g);
    return u;
}

static float processMS20(float in, LadderState& st, float fc, float res, float drive, float fs) {
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float x = driveStage(in, drive, 0.12f);
    float k = clamp(res, 0.f, 1.f) * 3.8f;
    float gHP = std::tan((float)M_PI * clamp(fcHz * 0.5f, 10.f, fs * 0.4f) / fs);
    float Gh = gHP / (1.f + gHP), ah = 1.f - Gh, a = 1.f - G;

    // zero-delay prediction of the output through HP-HP-LP-LP
    float P  = G * G * ah * ah;
    float Ch = -ah * ah * st.h[0] - ah * st.h[1];
    float Cl = G * a * st.s[0] + a * st.s[1];
    float Q  = G * G * Ch + Cl;
    float yEst = (P * x + Q) / (1.f + k * P);

    float hpIn = x - sat5(k * yEst);
    st.h[0] = onePoleLP(hpIn, st.h[0], gHP);
    float hpf = hpIn - st.h[0];
    st.h[1] = onePoleLP(hpf, st.h[1], gHP);
    hpf = hpf - st.h[1];
    float y = hpf;
    y = onePoleLP(y, st.s[0], g);
    y = onePoleLP(y, st.s[1], g);
    st.s[3] = y;
    return y;
}

struct SVFState { float lp = 0.f, bp = 0.f, hp = 0.f; };

static void processSVF(float in, SVFState& st, float fc, float res, float drive, float fs) {
    float fcHz = fcLimit(fc, fs, 0.2f);
    float f = 2.f * std::sin((float)M_PI * fcHz / fs);
    float q = clamp(1.f - res, 0.02f, 1.f);
    float x = driveStage(in, drive, 0.05f);
    float hp = x - st.lp - q * st.bp;
    float bp = softClip(f * hp + st.bp, 10.f);   // analog-style limiting keeps resonance bounded
    float lp = softClip(f * bp + st.lp, 10.f);
    if (std::fabs(lp) < 1e-12f) lp = 0.f;
    if (std::fabs(bp) < 1e-12f) bp = 0.f;
    st.hp = hp; st.bp = bp; st.lp = lp;
}

struct SPState { float s1 = 0.f, s2 = 0.f, bp = 0.f; };

static void processSteiner(float in, SPState& st, float fc, float res, float drive, float fs) {
    float fcHz = fcLimit(fc, fs, 0.25f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float k = clamp(res, 0.f, 1.f) * 3.5f;
    float x = driveStage(in, drive, 0.06f);
    float fb = softClip(st.s2 * k * 0.9f, 5.f);
    float hp = x - st.s1 - fb;
    float lp1 = onePoleLP(hp, st.s1, g);
    float lp2 = onePoleLP(lp1, st.s2, g);
    st.bp = hp - lp1 - lp2;
}

static float processCEM3320(float in, LadderState& st, float fc, float res, float drive, float fs) {
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float k = clamp(res, 0.f, 1.f) * 3.9f;
    float x = driveStage(in, drive, 0.04f);
    float y4 = ladderPredict(st.s, 4, G, x, k);
    float u = x - sat5(k * y4 * 0.85f);
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = onePoleLP(u, st.s[3], g);
    return sat5(u * 1.1f);
}

static float processRolandOTA(float in, LadderState& st, float fc, float res, float drive, float fs) {
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float k = clamp(res, 0.f, 1.f) * 3.7f;
    float x = driveStage(in, drive, 0.02f);
    float y4 = ladderPredict(st.s, 4, G, x, k);
    float u = x - sat5(k * y4 * 0.9f);
    u = onePoleLP(u, st.s[0], g);
    u = sat5(u * 1.1f);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    u = sat5(u * 1.05f);
    u = onePoleLP(u, st.s[3], g);
    return u;
}

static float processWasp(float in, LadderState& st, float fc, float res, float drive, float fs) {
    float fcHz = fcLimit(fc, fs, 0.4f);
    float g = std::tan((float)M_PI * fcHz / fs);
    float G = g / (1.f + g);
    float k = clamp(res, 0.f, 1.f) * 4.2f;
    float x = driveStage(in, drive * 1.25f, 0.10f);   // the Wasp is the grittiest engine
    float y3 = ladderPredict(st.s, 3, G, x, k);
    float u = sat5(x - sat5(k * y3));
    u = onePoleLP(u, st.s[0], g);
    u = onePoleLP(u, st.s[1], g);
    u = onePoleLP(u, st.s[2], g);
    return sat5(u * 1.2f);
}

struct LPGState { float vactrol = 0.f; float lp = 0.f; };

static float processLPG(float in, LPGState& st, float fc, float res, float drive, float fs) {
    float level = clamp(fc / 1000.f, 0.f, 1.f);
    float target = level;
    float lag = 0.0015f + (1.f - level) * 0.03f;
    st.vactrol += (target - st.vactrol) * lag * (fs / 48000.f);
    if (st.vactrol < 0.f) st.vactrol = 0.f;
    if (st.vactrol > 1.f) st.vactrol = 1.f;
    float vca = driveStage(in, drive, 0.05f) * st.vactrol;
    float lpgFc = 200.f + st.vactrol * 12000.f;
    float g = std::tan((float)M_PI * clamp(lpgFc, 20.f, fs * 0.4f) / fs);
    st.lp = onePoleLP(vca, st.lp, g);
    return sat5(st.lp);
}

// ============================================================
// Panel layout constants (16 HP = 240 px wide, 380 px high)
// ============================================================
static const float BS_W = RACK_GRID_WIDTH * 16;   // 240
static const float BS_H = RACK_GRID_HEIGHT;       // 380

// Display area
static const float DISP_Y   = 44.f;
static const float DISP_H   = 66.f;
static const float RESP_X   = 12.f,  RESP_W  = 144.f;
static const float SCOPE_X  = 162.f, SCOPE_W = 50.f;
static const float METER_X  = 218.f;               // two bars, 5 px wide each

// Engine strip
static const float ENG_X    = 34.f;
static const float ENG_Y    = 138.f;
static const float NAME_X   = 64.f, NAME_Y = 120.f, NAME_W = 164.f, NAME_H = 36.f;

// Main knobs
static const float KNOB_TITLE_Y = 166.f;
static const float KNOB_Y       = 198.f;
static const float READ_Y       = 225.f;
static const float KNOB_ARC_R   = 25.f;

// CV section
static const float CV_HDR_Y   = 240.f;
static const float CV_TRIM_Y  = 257.f;
static const float CV_JACK_Y  = 279.f;

// I/O section
static const float IO_Y0     = 294.f;
static const float IO_H      = 68.f;
static const float IO_LBL_Y  = 302.f;
static const float IN_Y      = 320.f;
static const float OUT_Y     = 346.f;

static float colX(int i) { return 48.f + 72.f * i; }   // 48, 120, 192

// Knob sweep (matches Rack's standard knobs)
static const float KNOB_MIN_ANGLE = -0.83f * (float)M_PI;
static const float KNOB_MAX_ANGLE =  0.83f * (float)M_PI;
static float knobAngle(float t) {   // t in 0..1 -> NanoVG angle (0 = +x axis)
    return -(float)M_PI / 2.f + KNOB_MIN_ANGLE + t * (KNOB_MAX_ANGLE - KNOB_MIN_ANGLE);
}

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

    static const int SCOPE_N = 64;

    LadderState ladder;
    SVFState    svf;
    SPState     sp;
    LPGState    lpg;

    int lastEngine = -1;
    float enginePulse = 0.f;

    // ---- Values shared with the UI (read-only from the widget) ----
    float dispCutoff = 632.f;
    float dispRes    = 0.3f;
    float dispDrive  = 0.f;
    float inPeak     = 0.f;
    float outPeak    = 0.f;
    float scopeMin[SCOPE_N] = {};
    float scopeMax[SCOPE_N] = {};
    int   scopePos   = 0;
    float scopeAccMin = 0.f, scopeAccMax = 0.f;
    int   scopeCount = 0;

    Bastion() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configSwitch(ENGINE_PARAM, 0.f, NUM_ENGINES - 1, 0.f, "Engine",
            std::vector<std::string>(ENGINE_NAMES, ENGINE_NAMES + NUM_ENGINES));

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

        configLight(LIGHT_ENGINE_A, "Engine change");
        configLight(LIGHT_ENGINE_B, "Signal activity");
    }

    void onReset() override {
        ladder = LadderState(); svf = SVFState();
        sp = SPState(); lpg = LPGState();
        lastEngine = -1;
        inPeak = outPeak = 0.f;
        for (int i = 0; i < SCOPE_N; ++i) scopeMin[i] = scopeMax[i] = 0.f;
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

        dispCutoff = cutoff;
        dispRes    = res;
        dispDrive  = drive;

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

        if (!std::isfinite(outLP) || !std::isfinite(outBP) || !std::isfinite(outHP)) {
            ladder = LadderState(); svf = SVFState(); sp = SPState(); lpg = LPGState();
            outLP = outBP = outHP = 0.f;
        }
        outLP = clamp(outLP, -12.f, 12.f);
        outBP = clamp(outBP, -12.f, 12.f);
        outHP = clamp(outHP, -12.f, 12.f);

        outputs[LP_OUTPUT].setVoltage(outLP);
        outputs[BP_OUTPUT].setVoltage(outBP);
        outputs[HP_OUTPUT].setVoltage(outHP);

        // ---- UI feedback: meters ----
        float peakDecay = std::exp(-1.f / (fs * 0.18f));
        inPeak  = std::max(std::fabs(input), inPeak * peakDecay);
        float outMax = std::max(std::fabs(outLP), std::max(std::fabs(outBP), std::fabs(outHP)));
        outPeak = std::max(outMax, outPeak * peakDecay);

        // ---- UI feedback: scope (LP output, ~30 ms window) ----
        int stride = std::max(1, (int)(fs * 0.03f / SCOPE_N));
        if (scopeCount == 0) { scopeAccMin = scopeAccMax = outLP; }
        scopeAccMin = std::min(scopeAccMin, outLP);
        scopeAccMax = std::max(scopeAccMax, outLP);
        if (++scopeCount >= stride) {
            scopeMin[scopePos] = scopeAccMin;
            scopeMax[scopePos] = scopeAccMax;
            scopePos = (scopePos + 1) % SCOPE_N;
            scopeCount = 0;
        }

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

NVGcolor withAlpha(NVGcolor c, float a) { c.a = a; return c; }

// One accent colour per engine (used by display, knob and name plate)
NVGcolor engineColor(int e) {
    static const NVGcolor cols[NUM_ENGINES] = {
        nvgRGB(0x3d, 0xe0, 0xff),  // Moog       - cyan
        nvgRGB(0xff, 0xb0, 0x3d),  // TB-303     - amber
        nvgRGB(0xff, 0x5a, 0x6e),  // MS-20      - red
        nvgRGB(0x50, 0xff, 0x90),  // SEM        - green
        nvgRGB(0xb0, 0x5a, 0xff),  // Steiner    - purple
        nvgRGB(0x5a, 0x9b, 0xff),  // CEM3320    - blue
        nvgRGB(0xff, 0xe0, 0x4a),  // Roland OTA - yellow
        nvgRGB(0xff, 0x8a, 0x5a),  // EMS        - orange
        nvgRGB(0xe0, 0xff, 0x3d),  // Wasp       - lime
        nvgRGB(0x3d, 0xff, 0xd0),  // Buchla     - teal
    };
    return cols[clamp(e, 0, NUM_ENGINES - 1)];
}

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

void plate(NVGcontext* vg, float x, float y, float w, float h, float r = 6.f) {
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, r);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 80));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

// Dark inset screen
void screen(NVGcontext* vg, float x, float y, float w, float h) {
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 4.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h, nvgRGB(0x06, 0x08, 0x0b), nvgRGB(0x0d, 0x10, 0x15)));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 38));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

void label(NVGcolor c, NVGcontext* vg, float x, float y, const char* t, float size = 7.f) {
    drawText(vg, x, y, t, size, c, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
}

void sectionHeader(NVGcontext* vg, float x, float y, float w, const char* title) {
    nvgBeginPath(vg);
    nvgMoveTo(vg, x, y);
    nvgLineTo(vg, x + w, y);
    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
    drawText(vg, x + w / 2.f, y - 7.f, title, 6.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);
}

void fmtHz(char* b, size_t n, float hz) {
    if (hz >= 1000.f) snprintf(b, n, "%.2f kHz", hz / 1000.f);
    else              snprintf(b, n, "%.0f Hz", hz);
}

void drawKnobArc(NVGcontext* vg, float cx, float cy, float r, float t0, float t1,
                 NVGcolor col, float width) {
    if (t1 - t0 < 0.001f) return;
    nvgBeginPath(vg);
    nvgArc(vg, cx, cy, r, knobAngle(t0), knobAngle(t1), NVG_CW);
    nvgStrokeColor(vg, col);
    nvgStrokeWidth(vg, width);
    nvgLineCap(vg, NVG_ROUND);
    nvgStroke(vg);
}

// Approximate magnitude response (dB) of the LP output for each engine.
float responseDb(int engine, float f, float fc, float res) {
    typedef std::complex<float> C;
    float w = f / std::max(fc, 1.f);
    C s(0.f, w);
    C one(1.f, 0.f);
    C H(1.f, 0.f);
    switch (engine) {
        case ENGINE_MOOG:
        case ENGINE_TB303:
        case ENGINE_CEM3320:
        case ENGINE_ROLAND_OTA:
        case ENGINE_EMS_DIODE: {
            float kMax = (engine == ENGINE_MOOG) ? 4.0f :
                         (engine == ENGINE_CEM3320) ? 3.9f :
                         (engine == ENGINE_ROLAND_OTA) ? 3.7f : 3.6f;
            float k = clamp(res, 0.f, 1.f) * kMax;
            C p = one + s;
            C p4 = p * p * p * p;
            H = one / (p4 + C(k, 0.f));
        } break;
        case ENGINE_WASP: {
            float k = clamp(res, 0.f, 1.f) * 4.2f;
            C p = one + s;
            C p3 = p * p * p;
            H = one / (p3 + C(k, 0.f));
        } break;
        case ENGINE_MS20: {
            float Q = 0.6f + clamp(res, 0.f, 1.f) * 10.f;
            H = one / (s * s + s * (1.f / Q) + one);
        } break;
        case ENGINE_SEM: {
            float q = clamp(1.f - res, 0.02f, 1.f);
            H = one / (s * s + s * q + one);
        } break;
        case ENGINE_STEINER: {
            float Q = 0.7f + clamp(res, 0.f, 1.f) * 6.f;
            H = one / (s * s + s * (1.f / Q) + one);
        } break;
        case ENGINE_BUCHLA_LPG: {
            float fcEff = 200.f + clamp(fc / 1000.f, 0.f, 1.f) * 12000.f;
            C s2(0.f, f / fcEff);
            H = one / (one + s2);
        } break;
    }
    float mag = std::abs(H);
    return 20.f * std::log10(mag + 1e-6f);
}

} // namespace

// ============================================================
// Panel widget (static artwork + live displays)
// ============================================================
struct BastionPanel : TransparentWidget {
    Bastion* bm = nullptr;

    BastionPanel() { box.size = Vec(BS_W, BS_H); }

    // ---------------- Static artwork ----------------
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, BS_W, BS_H, "BASTION");

        // ---- Displays ----
        screen(vg, RESP_X, DISP_Y, RESP_W, DISP_H);
        screen(vg, SCOPE_X, DISP_Y, SCOPE_W, DISP_H);
        screen(vg, METER_X - 2.f, DISP_Y, 14.f, DISP_H);
        drawText(vg, RESP_X + 4.f, DISP_Y + 7.f, "RESPONSE", 5.f, colDim(),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, SCOPE_X + 4.f, DISP_Y + 7.f, "SCOPE", 5.f, colDim(),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        label(colDim(), vg, METER_X + 2.5f, DISP_Y + DISP_H + 5.f, "I", 5.f);
        label(colDim(), vg, METER_X + 8.5f, DISP_Y + DISP_H + 5.f, "O", 5.f);

        // ---- Engine selector ----
        // Detent ticks around the engine knob
        for (int i = 0; i < NUM_ENGINES; ++i) {
            float a = knobAngle(i / (float)(NUM_ENGINES - 1));
            nvgBeginPath(vg);
            nvgMoveTo(vg, ENG_X + std::cos(a) * 19.f, ENG_Y + std::sin(a) * 19.f);
            nvgLineTo(vg, ENG_X + std::cos(a) * 23.f, ENG_Y + std::sin(a) * 23.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 55));
            nvgStrokeWidth(vg, 1.2f);
            nvgStroke(vg);
        }
        label(colAccent(), vg, ENG_X, ENG_Y + 29.f - 1.f, "ENGINE", 5.5f);

        plate(vg, NAME_X, NAME_Y, NAME_W, NAME_H, 5.f);
        drawText(vg, NAME_X + 112.f + 40.f, NAME_Y + 11.f, "SWITCH", 5.f, colDim(),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, NAME_X + 112.f + 40.f, NAME_Y + 25.f, "SIGNAL", 5.f, colDim(),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        // ---- Main knobs ----
        const char* mainTitles[3] = {"CUTOFF", "RESONANCE", "DRIVE"};
        for (int i = 0; i < 3; ++i) {
            float cx = colX(i);
            // knob well
            nvgBeginPath(vg);
            nvgCircle(vg, cx, KNOB_Y, 21.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 18));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            // track
            drawKnobArc(vg, cx, KNOB_Y, KNOB_ARC_R, 0.f, 1.f, nvgRGBA(255, 255, 255, 28), 2.5f);
            label(colText(), vg, cx, KNOB_TITLE_Y, mainTitles[i], 6.5f);
        }

        // ---- CV section ----
        sectionHeader(vg, 10.f, CV_HDR_Y, BS_W - 20.f, "CV MODULATION");
        const char* cvTitles[3] = {"CUT", "RES", "DRV"};
        const NVGcolor cvCols[3] = {colAccent(), colHi(), colMag()};
        for (int i = 0; i < 3; ++i) {
            float cx = colX(i);
            plate(vg, cx - 14.f, CV_TRIM_Y - 10.f, 28.f, 46.f, 5.f);
            // tiny colour tag on the plate
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 8.f, CV_TRIM_Y - 10.f, 16.f, 1.5f, 0.7f);
            nvgFillColor(vg, cvCols[i]);
            nvgFill(vg);
            (void) cvTitles;
        }
        label(colDim(), vg, 20.f, CV_TRIM_Y, "ATT", 5.5f);
        label(colDim(), vg, 20.f, CV_JACK_Y, "CV IN", 5.5f);

        // ---- I/O section ----
        plate(vg, 8.f, IO_Y0, BS_W - 16.f, IO_H, 6.f);
        const char* ioTitles[3] = {"LP", "BP", "HP"};
        const NVGcolor ioCols[3] = {colHi(), colAccent(), colMag()};
        for (int i = 0; i < 3; ++i)
            label(ioCols[i], vg, colX(i), IO_LBL_Y, ioTitles[i], 7.f);
        label(colDim(), vg, 20.f, IN_Y, "IN", 6.f);
        label(colDim(), vg, 20.f, OUT_Y, "OUT", 6.f);
        // separator between IN and OUT rows
        nvgBeginPath(vg);
        nvgMoveTo(vg, 34.f, (IN_Y + OUT_Y) / 2.f);
        nvgLineTo(vg, BS_W - 12.f, (IN_Y + OUT_Y) / 2.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 14));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        Widget::draw(args);
    }

    // ---------------- Live (glowing) layer ----------------
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer == 1) drawLive(args.vg);
        Widget::drawLayer(args, layer);
    }

    void drawLive(NVGcontext* vg) {
        // Pull values (or sensible demo values in the module browser)
        int   engine = 0;
        float cutoff = 632.f, res = 0.3f, drive = 0.f;
        float pCut = 0.5f, pRes = 0.3f, pDrive = 0.f;
        float inPk = 0.f, outPk = 0.f;
        if (bm) {
            engine = clamp((int)std::round(bm->params[Bastion::ENGINE_PARAM].getValue()), 0, NUM_ENGINES - 1);
            cutoff = bm->dispCutoff; res = bm->dispRes; drive = bm->dispDrive;
            pCut   = bm->params[Bastion::CUTOFF_PARAM].getValue();
            pRes   = bm->params[Bastion::RES_PARAM].getValue();
            pDrive = bm->params[Bastion::DRIVE_PARAM].getValue();
            inPk   = bm->inPeak; outPk = bm->outPeak;
        }
        NVGcolor ec = engineColor(engine);

        drawResponse(vg, engine, cutoff, res, ec);
        drawScope(vg, ec);
        drawMeters(vg, inPk, outPk);
        drawEnginePlate(vg, engine, ec);

        // Engine tick highlight
        {
            float a = knobAngle(engine / (float)(NUM_ENGINES - 1));
            nvgBeginPath(vg);
            nvgMoveTo(vg, ENG_X + std::cos(a) * 18.f, ENG_Y + std::sin(a) * 18.f);
            nvgLineTo(vg, ENG_X + std::cos(a) * 24.f, ENG_Y + std::sin(a) * 24.f);
            nvgStrokeColor(vg, ec);
            nvgStrokeWidth(vg, 2.2f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStroke(vg);
        }

        // Knob value arcs + readouts
        float vals[3]  = {pCut, pRes, pDrive};
        NVGcolor cols[3] = {colAccent(), colHi(), colMag()};
        char buf[32];
        for (int i = 0; i < 3; ++i) {
            float cx = colX(i);
            drawKnobArc(vg, cx, KNOB_Y, KNOB_ARC_R, 0.f, clamp(vals[i], 0.f, 1.f), cols[i], 2.5f);
            if (i == 0)      fmtHz(buf, sizeof buf, cutoff);
            else if (i == 1) snprintf(buf, sizeof buf, "%.0f %%", res * 100.f);
            else             snprintf(buf, sizeof buf, "%.0f %%", drive * 100.f);
            drawText(vg, cx, READ_Y, buf, 7.f, cols[i], NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }
    }

    void drawResponse(NVGcontext* vg, int engine, float fc, float res, NVGcolor ec) {
        const float x0 = RESP_X + 3.f, x1 = RESP_X + RESP_W - 3.f;
        const float y0 = DISP_Y + 12.f, y1 = DISP_Y + DISP_H - 10.f;
        const float w = x1 - x0, h = y1 - y0;
        const float dbMin = -42.f, dbMax = 24.f;

        nvgSave(vg);
        nvgScissor(vg, RESP_X + 1.f, DISP_Y + 1.f, RESP_W - 2.f, DISP_H - 2.f);

        auto dbToY = [&](float db) {
            float t = clamp((db - dbMin) / (dbMax - dbMin), 0.f, 1.f);
            return y1 - t * h;
        };
        auto fToX = [&](float f) {
            return x0 + std::log(f / 20.f) / std::log(1000.f) * w;
        };

        // grid: decades
        const float fGrid[3] = {100.f, 1000.f, 10000.f};
        const char* fLbl[3] = {"100", "1k", "10k"};
        for (int i = 0; i < 3; ++i) {
            float gx = fToX(fGrid[i]);
            nvgBeginPath(vg);
            nvgMoveTo(vg, gx, y0);
            nvgLineTo(vg, gx, y1);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            label(colDim(), vg, gx, DISP_Y + DISP_H - 4.f, fLbl[i], 5.f);
        }
        // grid: dB lines
        const float dbGrid[3] = {12.f, 0.f, -24.f};
        const char* dbLbl[3] = {"+12", "0", "-24"};
        for (int i = 0; i < 3; ++i) {
            float gy = dbToY(dbGrid[i]);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x0, gy);
            nvgLineTo(vg, x1, gy);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, dbGrid[i] == 0.f ? 30 : 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, x1 - 1.f, gy - 3.f, dbLbl[i], 4.5f, colDim(),
                     NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.3f);
        }

        // cutoff marker
        float fcX = clamp(fToX(std::max(fc, 20.f)), x0, x1);
        nvgBeginPath(vg);
        nvgMoveTo(vg, fcX, y0);
        nvgLineTo(vg, fcX, y1);
        nvgStrokeColor(vg, withAlpha(ec, 0.35f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // response curve
        const int N = 96;
        nvgBeginPath(vg);
        for (int i = 0; i <= N; ++i) {
            float t = i / (float)N;
            float f = 20.f * std::pow(1000.f, t);
            float db = responseDb(engine, f, fc, res);
            float px = x0 + t * w;
            float py = dbToY(db);
            if (i == 0) nvgMoveTo(vg, px, py); else nvgLineTo(vg, px, py);
        }
        nvgLineTo(vg, x1, y1);
        nvgLineTo(vg, x0, y1);
        nvgClosePath(vg);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, y0, 0, y1, withAlpha(ec, 0.40f), withAlpha(ec, 0.02f)));
        nvgFill(vg);

        nvgBeginPath(vg);
        for (int i = 0; i <= N; ++i) {
            float t = i / (float)N;
            float f = 20.f * std::pow(1000.f, t);
            float db = responseDb(engine, f, fc, res);
            float px = x0 + t * w;
            float py = dbToY(db);
            if (i == 0) nvgMoveTo(vg, px, py); else nvgLineTo(vg, px, py);
        }
        nvgStrokeColor(vg, ec);
        nvgStrokeWidth(vg, 1.4f);
        nvgLineJoin(vg, NVG_ROUND);
        nvgStroke(vg);

        nvgRestore(vg);
    }

    void drawScope(NVGcontext* vg, NVGcolor ec) {
        const int N = Bastion::SCOPE_N;
        const float x0 = SCOPE_X + 3.f, x1 = SCOPE_X + SCOPE_W - 3.f;
        const float cy = DISP_Y + 12.f + (DISP_H - 22.f) / 2.f;
        const float amp = (DISP_H - 22.f) / 2.f;
        const float w = x1 - x0;

        nvgSave(vg);
        nvgScissor(vg, SCOPE_X + 1.f, DISP_Y + 1.f, SCOPE_W - 2.f, DISP_H - 2.f);

        // centre line
        nvgBeginPath(vg);
        nvgMoveTo(vg, x0, cy);
        nvgLineTo(vg, x1, cy);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        float mn[Bastion::SCOPE_N], mx[Bastion::SCOPE_N];
        if (bm) {
            int pos = bm->scopePos;
            for (int i = 0; i < N; ++i) {
                int idx = (pos + i) % N;
                mn[i] = bm->scopeMin[idx];
                mx[i] = bm->scopeMax[idx];
            }
        } else {
            for (int i = 0; i < N; ++i) {   // demo waveform for the module browser
                float v = 3.5f * std::sin(i * 0.45f);
                mn[i] = v - 0.3f; mx[i] = v + 0.3f;
            }
        }

        const float scale = 1.f / 6.f;   // +-6 V full height
        nvgBeginPath(vg);
        for (int i = 0; i < N; ++i) {
            float px = x0 + w * i / (float)(N - 1);
            float py = cy - clamp(mx[i] * scale, -1.f, 1.f) * amp;
            if (i == 0) nvgMoveTo(vg, px, py); else nvgLineTo(vg, px, py);
        }
        for (int i = N - 1; i >= 0; --i) {
            float px = x0 + w * i / (float)(N - 1);
            float py = cy - clamp(mn[i] * scale, -1.f, 1.f) * amp;
            nvgLineTo(vg, px, py);
        }
        nvgClosePath(vg);
        nvgFillColor(vg, withAlpha(ec, 0.35f));
        nvgFill(vg);
        nvgStrokeColor(vg, ec);
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        nvgRestore(vg);
    }

    void drawMeters(NVGcontext* vg, float inPk, float outPk) {
        const float y0 = DISP_Y + 4.f, h = DISP_H - 8.f;
        float levels[2] = {clamp(inPk / 10.f, 0.f, 1.f), clamp(outPk / 10.f, 0.f, 1.f)};
        for (int i = 0; i < 2; ++i) {
            float x = METER_X + i * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, y0, 5.f, h);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            float lh = levels[i] * h;
            if (lh > 0.5f) {
                nvgBeginPath(vg);
                nvgRect(vg, x, y0 + h - lh, 5.f, lh);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, y0 + h, 0, y0,
                                 nvgRGB(0x50, 0xff, 0x90), nvgRGB(0xff, 0x50, 0x50)));
                nvgFill(vg);
            }
        }
    }

    void drawEnginePlate(NVGcontext* vg, int engine, NVGcolor ec) {
        char buf[16];
        snprintf(buf, sizeof buf, "%02d / %02d", engine + 1, (int)NUM_ENGINES);
        drawText(vg, NAME_X + 7.f, NAME_Y + 8.f, buf, 5.5f, colDim(),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        drawText(vg, NAME_X + 7.f, NAME_Y + 19.f, ENGINE_NAMES[engine], 10.5f, ec,
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
        drawText(vg, NAME_X + 7.f, NAME_Y + 29.f, ENGINE_DESCS[engine], 5.f, colDim(),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
    }
};

// ============================================================
// Engine selector knob - fully custom-drawn (no SVG required)
// ============================================================
struct BastionEngineKnob : app::Knob {
    BastionEngineKnob() {
        box.size = Vec(30.f, 30.f);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        float cx = box.size.x / 2.f, cy = box.size.y / 2.f;
        float r = 14.f;

        float value = 0.f;
        if (engine::ParamQuantity* pq = getParamQuantity())
            value = pq->getValue();
        int engine = clamp((int)std::round(value), 0, NUM_ENGINES - 1);
        NVGcolor ec = engineColor(engine);

        // drop shadow
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy + 1.5f, r + 1.5f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 120));
        nvgFill(vg);

        // body
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillPaint(vg, nvgRadialGradient(vg, cx - 4.f, cy - 5.f, 1.f, r + 4.f,
                         nvgRGB(0x4a, 0x50, 0x5c), nvgRGB(0x12, 0x14, 0x18)));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 50));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // inner cap
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r * 0.62f);
        nvgFillPaint(vg, nvgLinearGradient(vg, cx, cy - r, cx, cy + r,
                         nvgRGB(0x2a, 0x2e, 0x36), nvgRGB(0x0e, 0x10, 0x14)));
        nvgFill(vg);

        // pointer
        float t = value / (float)(NUM_ENGINES - 1);
        float a = knobAngle(clamp(t, 0.f, 1.f));
        nvgBeginPath(vg);
        nvgMoveTo(vg, cx + std::cos(a) * 3.f,  cy + std::sin(a) * 3.f);
        nvgLineTo(vg, cx + std::cos(a) * 12.f, cy + std::sin(a) * 12.f);
        nvgStrokeColor(vg, ec);
        nvgStrokeWidth(vg, 2.4f);
        nvgLineCap(vg, NVG_ROUND);
        nvgStroke(vg);

        // centre dot
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, 1.6f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 70));
        nvgFill(vg);
    }
};

// ============================================================
struct BastionWidget : ModuleWidget {
    BastionWidget(Bastion* module) {
        setModule(module);
        box.size = Vec(BS_W, BS_H);   // 16 HP, panel is drawn entirely by BastionPanel

        BastionPanel* panel = new BastionPanel();
        panel->bm = module;
        addChild(panel);

        // ---- Engine selector + indicator lights ----
        addParam(createParamCentered<BastionEngineKnob>(
            Vec(ENG_X, ENG_Y), module, Bastion::ENGINE_PARAM));
        addChild(createLightCentered<SmallLight<GreenLight>>(
            Vec(NAME_X + NAME_W - 10.f, NAME_Y + 11.f), module, Bastion::LIGHT_ENGINE_A));
        addChild(createLightCentered<SmallLight<GreenLight>>(
            Vec(NAME_X + NAME_W - 10.f, NAME_Y + 25.f), module, Bastion::LIGHT_ENGINE_B));

        // ---- Main knobs ----
        addParam(createParamCentered<RoundBigBlackKnob>(
            Vec(colX(0), KNOB_Y), module, Bastion::CUTOFF_PARAM));
        addParam(createParamCentered<RoundBigBlackKnob>(
            Vec(colX(1), KNOB_Y), module, Bastion::RES_PARAM));
        addParam(createParamCentered<RoundBigBlackKnob>(
            Vec(colX(2), KNOB_Y), module, Bastion::DRIVE_PARAM));

        // ---- CV attenuverters ----
        addParam(createParamCentered<Trimpot>(
            Vec(colX(0), CV_TRIM_Y), module, Bastion::CUTOFF_CV_PARAM));
        addParam(createParamCentered<Trimpot>(
            Vec(colX(1), CV_TRIM_Y), module, Bastion::RES_CV_PARAM));
        addParam(createParamCentered<Trimpot>(
            Vec(colX(2), CV_TRIM_Y), module, Bastion::DRIVE_CV_PARAM));

        // ---- CV inputs ----
        addInput(createInputCentered<PJ301MPort>(
            Vec(colX(0), CV_JACK_Y), module, Bastion::CUTOFF_INPUT));
        addInput(createInputCentered<PJ301MPort>(
            Vec(colX(1), CV_JACK_Y), module, Bastion::RES_INPUT));
        addInput(createInputCentered<PJ301MPort>(
            Vec(colX(2), CV_JACK_Y), module, Bastion::DRIVE_INPUT));

        // ---- Signal inputs ----
        addInput(createInputCentered<PJ301MPort>(
            Vec(colX(0), IN_Y), module, Bastion::LP_INPUT));
        addInput(createInputCentered<PJ301MPort>(
            Vec(colX(1), IN_Y), module, Bastion::BP_INPUT));
        addInput(createInputCentered<PJ301MPort>(
            Vec(colX(2), IN_Y), module, Bastion::HP_INPUT));

        // ---- Outputs ----
        addOutput(createOutputCentered<PJ301MPort>(
            Vec(colX(0), OUT_Y), module, Bastion::LP_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(
            Vec(colX(1), OUT_Y), module, Bastion::BP_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(
            Vec(colX(2), OUT_Y), module, Bastion::HP_OUTPUT));

        // ---- Screws ----
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    }

    void appendContextMenu(Menu* menu) override {
        Bastion* module = dynamic_cast<Bastion*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);

        menu->addChild(createSubmenuItem("Engine", ENGINE_NAMES[clamp((int)std::round(module->params[Bastion::ENGINE_PARAM].getValue()), 0, NUM_ENGINES - 1)],
            [=](Menu* submenu) {
                for (int i = 0; i < NUM_ENGINES; ++i) {
                    submenu->addChild(createCheckMenuItem(ENGINE_NAMES[i], "",
                        [=]() { return (int)std::round(module->params[Bastion::ENGINE_PARAM].getValue()) == i; },
                        [=]() { module->params[Bastion::ENGINE_PARAM].setValue((float)i); }
                    ));
                }
            }
        ));

        menu->addChild(createMenuItem("Reset State", "", [=]() {
            module->onReset();
        }));
    }
};

// ============================================================
// Module registration
// ============================================================
Model* modelBastion = createModel<Bastion, BastionWidget>("Bastion");