#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int MB_NUM_FUNCS = 20;
static const int MB_COLS = 128;   // scope columns (min/max pairs)
static const int MB_TRAIL_N = 48; // transfer-plot trail

static const float MB_PANEL_W = RACK_GRID_WIDTH * 18; // 270
static const float MB_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display
static const float MB_DISP_W = 230.f;
static const float MB_DISP_H = 150.f;
static const float MB_DISP_X = (MB_PANEL_W - MB_DISP_W) / 2.f; // 20
static const float MB_DISP_Y = 44.f;

// Transfer plot + harmonics (display-local)
static const float MB_TF_X0 = 10.f, MB_TF_Y0 = 18.f, MB_TF_S = 66.f;
static const float MB_HM_X0 = 10.f, MB_HM_W = 66.f;
static const float MB_HM_TOP = 108.f, MB_HM_BASE = 137.f, MB_HM_LBL_Y = 143.5f;

// Scopes (display-local)
static const float MB_SC_X0 = 90.f, MB_SC_X1 = 222.f;
static const float MB_SCI_Y0 = 18.f, MB_SCI_Y1 = 76.f;  // IN
static const float MB_SCO_Y0 = 85.f, MB_SCO_Y1 = 143.f; // OUT

// Controls
static const float MB_SECTION_CTRL = 214.f;
static const float MB_KNOB_Y = 246.f;

// I/O
static const float MB_SECTION_IO = 306.f;
static const float MB_IO_Y = 346.f;
static const float MB_IO_X[6] = {30.f, 72.f, 114.f, 156.f, 198.f, 240.f};

// ------------------------------------------------------------
// Shaping functions (shared by DSP and display)
// ------------------------------------------------------------
static const char *const MB_NAMES[MB_NUM_FUNCS] = {
    "A^2", "A^3", "sin(A)", "cos(A)", "tan(A)", "sqrt(A)", "cbrt(A)", "exp(A)",
    "sinh(A)", "cosh(A)", "tanh(A)", "fold(A)", "T3(A)", "T5(A)", "|A|",
    "half(A)", "asin(A)", "atan(A)", "log(A)", "crush(A)"};

static const char *const MB_DESC[MB_NUM_FUNCS] = {
    "Square: rectifies, octave up",
    "Cube: odd harmonics, soft fuzz",
    "Sine: smooth saturation",
    "Cosine: inverted, octave doubling",
    "Tangent: bright mid boost",
    "Square root: gritty, expands quiet detail",
    "Cube root: milder expansion",
    "Exponential: asymmetric, even harmonics",
    "Hyperbolic sine: tight, expanding",
    "Hyperbolic cosine: rectifying, octave up",
    "Hyperbolic tangent: tube-style saturation",
    "Wavefolder: triangle fold",
    "Chebyshev 3rd: adds the 3rd harmonic",
    "Chebyshev 5th: adds the 5th harmonic",
    "Full-wave rectifier",
    "Half-wave rectifier",
    "Arcsine: pushes peaks outward",
    "Arctangent: soft clipper",
    "Logarithmic compressor",
    "Bit crusher: 9 levels"};

// Raw function on x in [-1, 1]
static inline float mbRaw(float x, int f)
{
    switch (f)
    {
    case 0:
        return x * x;
    case 1:
        return x * x * x;
    case 2:
        return std::sin(1.5707963f * x);
    case 3:
        return std::cos(3.1415927f * x);
    case 4:
        return std::tan(1.2f * x);
    case 5:
        return std::sqrt(std::fabs(x)) * (x < 0.f ? -1.f : 1.f);
    case 6:
        return std::cbrt(x);
    case 7:
        return std::exp(x);
    case 8:
        return std::sinh(2.f * x);
    case 9:
        return std::cosh(2.f * x);
    case 10:
        return std::tanh(3.f * x);
    case 11:
        return 0.63661977f * std::asin(rack::math::clamp(std::sin(6.2831853f * x), -1.f, 1.f));
    case 12:
        return 4.f * x * x * x - 3.f * x;
    case 13:
    {
        float x2 = x * x;
        return x * (16.f * x2 * x2 - 20.f * x2 + 5.f);
    }
    case 14:
        return std::fabs(x);
    case 15:
        return std::max(x, 0.f);
    case 16:
        return 0.63661977f * std::asin(rack::math::clamp(x, -1.f, 1.f));
    case 17:
        return std::atan(4.f * x);
    case 18:
        return std::log1p(8.f * std::fabs(x)) * (x < 0.f ? -1.f : 1.f);
    case 19:
        return std::round(4.f * x) * 0.25f;
    default:
        return x;
    }
}

// Every function is shifted so that f(0) = 0 (silence stays silent, no DC
// at rest) and scaled so that its largest excursion on [-1, 1] is exactly 1.
struct MBNorm
{
    float off[MB_NUM_FUNCS];
    float gain[MB_NUM_FUNCS];
    MBNorm()
    {
        for (int f = 0; f < MB_NUM_FUNCS; ++f)
        {
            off[f] = mbRaw(0.f, f);
            float m = 1e-3f;
            for (int i = 0; i <= 512; ++i)
            {
                float x = -1.f + 2.f * (float)i / 512.f;
                m = std::max(m, std::fabs(mbRaw(x, f) - off[f]));
            }
            gain[f] = 1.f / m;
        }
    }
};

static const MBNorm &mbNorm()
{
    static const MBNorm n;
    return n;
}

static inline float mbShape(float x, int f)
{
    const MBNorm &n = mbNorm();
    return (mbRaw(x, f) - n.off[f]) * n.gain[f];
}

// Keeps the function argument inside [-1, 1]; linear up to 0.9, then a smooth knee
static inline float mbSoftClip(float x)
{
    float a = std::fabs(x);
    if (a <= 0.9f)
        return x;
    float y = 0.9f + 0.1f * std::tanh((a - 0.9f) * 10.f);
    return (x < 0.f) ? -y : y;
}

// Output limiter: transparent up to 8 V, then a soft knee that never passes 10 V
static inline float mbLimit(float x)
{
    float a = std::fabs(x);
    if (a <= 8.f)
        return x;
    float y = 8.f + 2.f * std::tanh((a - 8.f) * 0.5f);
    return (x < 0.f) ? -y : y;
}

static inline float mbSanitize(float v)
{
    if (!std::isfinite(v))
        return 0.f;
    return rack::math::clamp(v, -30.f, 30.f);
}

// Time knob: 0..1 -> 5 ms .. 4 s per screen
struct MBTimeQ : rack::engine::ParamQuantity
{
    std::string getDisplayValueString() override
    {
        float t = 0.005f * std::pow(800.f, getValue());
        char b[32];
        if (t < 0.1f)
            snprintf(b, sizeof(b), "%.1f ms", t * 1000.f);
        else if (t < 1.f)
            snprintf(b, sizeof(b), "%.0f ms", t * 1000.f);
        else
            snprintf(b, sizeof(b), "%.2f s", t);
        return std::string(b);
    }
};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct MagicBox : Module
{
    enum ParamId
    {
        FUNC_PARAM,
        TIME_PARAM,
        DRIVE_PARAM,
        MIX_PARAM,
        LEVEL_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        IN_L_INPUT,
        IN_R_INPUT,
        FUNC_CV_INPUT,
        TIME_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };
    enum CapState
    {
        CAP_RUN,
        CAP_ARMED
    };

    // Settings (saved)
    bool autoLevel = true; // normalise by the input envelope
    bool dcBlock = true;   // remove DC created by asymmetric functions

    // DSP state
    float peak[2] = {0.f, 0.f};
    float dcX[2] = {0.f, 0.f}, dcY[2] = {0.f, 0.f};
    int funcCur = 0, funcPrev = 0;
    float xfade = 0.f;

    // Scope capture: channels 0 = IN L, 1 = IN R, 2 = OUT L, 3 = OUT R
    float scMin[4][MB_COLS] = {};
    float scMax[4][MB_COLS] = {};
    bool scValid[MB_COLS] = {};
    float accMin[4] = {}, accMax[4] = {};
    int capState = CAP_ARMED;
    int capCol = 0;
    float capCount = 0.f;
    float holdTimer = 0.f;
    bool trigLow = false;
    bool lastTrig = false;
    float sweepScale = 0.02f;

    // Display data
    float vTime = 0.02f, vDrive = 1.f, vMix = 1.f;
    float peakIn = 0.f, peakOut = 0.f, limitFlash = 0.f;
    bool vRConn = false, vLConn = false;
    float opN = 0.f, opY = 0.f;
    float trN[MB_TRAIL_N] = {}, trY[MB_TRAIL_N] = {};
    int trHead = 0, trCount = 0;
    float trTimer = 0.f;

    MagicBox()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        std::vector<std::string> names;
        for (int i = 0; i < MB_NUM_FUNCS; ++i)
            names.push_back(MB_NAMES[i]);
        configSwitch(FUNC_PARAM, 0.f, (float)(MB_NUM_FUNCS - 1), 2.f, "Function", names);

        configParam<MBTimeQ>(TIME_PARAM, 0.f, 1.f, 0.22f, "Time scale");
        configParam(DRIVE_PARAM, -2.f, 2.f, 0.f, "Drive", "x", 2.f, 1.f);
        configParam(MIX_PARAM, 0.f, 1.f, 1.f, "Dry / wet", "%", 0.f, 100.f);
        configParam(LEVEL_PARAM, 0.f, 1.5f, 1.f, "Output level", "%", 0.f, 100.f);

        configInput(IN_L_INPUT, "Left / mono");
        configInput(IN_R_INPUT, "Right (normalled to left)");
        configInput(FUNC_CV_INPUT, "Function CV (0-10 V sweeps all functions)");
        configInput(TIME_CV_INPUT, "Time CV (0-10 V sweeps the full range)");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");
    }

    void onReset() override
    {
        autoLevel = true;
        dcBlock = true;
        peak[0] = peak[1] = 0.f;
        dcX[0] = dcX[1] = dcY[0] = dcY[1] = 0.f;
        xfade = 0.f;
        std::memset(scMin, 0, sizeof(scMin));
        std::memset(scMax, 0, sizeof(scMax));
        std::memset(scValid, 0, sizeof(scValid));
        capState = CAP_ARMED;
        capCol = 0;
        capCount = 0.f;
        holdTimer = 0.f;
        trHead = trCount = 0;
        limitFlash = 0.f;
    }

    // ---------------- scope capture ----------------
    void startSweep(const float *c)
    {
        capState = CAP_RUN;
        capCol = 0;
        capCount = 0.f;
        for (int i = 0; i < 4; ++i)
            accMin[i] = accMax[i] = c[i];
    }

    // c[0..3] = IN L, IN R, OUT L, OUT R (volts); trigV = trigger source
    void scopeStep(const float *c, float trigV, float dt, float fs, float timeScale)
    {
        // A big change of the time knob restarts the sweep with a clean screen
        if (std::fabs(timeScale - sweepScale) > 0.06f * sweepScale)
        {
            sweepScale = timeScale;
            std::memset(scValid, 0, sizeof(scValid));
            startSweep(c);
        }

        if (capState == CAP_ARMED)
        {
            holdTimer += dt;
            if (trigV < -0.05f)
                trigLow = true;
            bool trig = trigLow && trigV > 0.f;
            float timeout = 0.03f + 0.1f * timeScale;
            if (trig || holdTimer > timeout)
            {
                lastTrig = trig;
                trigLow = false;
                startSweep(c);
            }
            else
            {
                return;
            }
        }

        for (int i = 0; i < 4; ++i)
        {
            accMin[i] = std::min(accMin[i], c[i]);
            accMax[i] = std::max(accMax[i], c[i]);
        }
        float spc = std::max(timeScale * fs / (float)MB_COLS, 0.05f); // samples per column
        capCount += 1.f;
        while (capCount >= spc && capCol < MB_COLS)
        {
            for (int i = 0; i < 4; ++i)
            {
                scMin[i][capCol] = accMin[i];
                scMax[i][capCol] = accMax[i];
            }
            scValid[capCol] = true;
            capCol++;
            capCount -= spc;
            for (int i = 0; i < 4; ++i)
                accMin[i] = accMax[i] = c[i];
        }
        if (capCol >= MB_COLS)
        {
            capState = CAP_ARMED;
            holdTimer = 0.f;
        }
    }

    // ---------------- audio ----------------
    void process(const ProcessArgs &args) override
    {
        const float fs = args.sampleRate;
        const float dt = args.sampleTime;

        // ---- parameters + CV ----
        float fv = params[FUNC_PARAM].getValue() + inputs[FUNC_CV_INPUT].getVoltage() * 0.1f * (float)(MB_NUM_FUNCS - 1);
        int func = rack::math::clamp((int)std::lround(fv), 0, MB_NUM_FUNCS - 1);
        if (func != funcCur)
        {
            funcPrev = funcCur;
            funcCur = func;
            xfade = 1.f; // short crossfade: no click when the function changes
        }

        float tp = rack::math::clamp(
            params[TIME_PARAM].getValue() + inputs[TIME_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        float timeScale = 0.005f * std::pow(800.f, tp);

        float drive = std::exp2(params[DRIVE_PARAM].getValue());
        float mix = params[MIX_PARAM].getValue();
        float level = params[LEVEL_PARAM].getValue();

        // ---- inputs ----
        bool lConn = inputs[IN_L_INPUT].isConnected();
        bool rConn = inputs[IN_R_INPUT].isConnected();
        float in[2];
        in[0] = mbSanitize(inputs[IN_L_INPUT].getVoltage());
        in[1] = rConn ? mbSanitize(inputs[IN_R_INPUT].getVoltage()) : in[0];

        const float peakDecay = std::exp(-dt / 0.4f);
        const float dcR = rack::math::clamp(1.f - 6.2831853f * 8.f / fs, 0.9f, 0.99999f);

        float outv[2];
        bool limited = false;

        for (int ch = 0; ch < 2; ++ch)
        {
            float v = in[ch];
            float x = v * 0.2f; // 5 V -> 1

            float g = 1.f;
            if (autoLevel)
            {
                peak[ch] = std::max(std::fabs(x), peak[ch] * peakDecay);
                g = std::max(peak[ch], 0.02f); // noise floor: never amplify hiss
            }
            float n = x / g;                  // normalised input
            float xs = mbSoftClip(n * drive); // always inside [-1, 1]

            float ys = mbShape(xs, funcCur);
            float w = ys;
            if (xfade > 0.f)
                w += (mbShape(xs, funcPrev) - w) * xfade;

            if (dcBlock)
            {
                float y = w - dcX[ch] + dcR * dcY[ch];
                dcX[ch] = w;
                dcY[ch] = y;
                w = y;
            }
            else
            {
                dcX[ch] = w;
                dcY[ch] = 0.f;
            }

            float wet = w * g * 5.f;
            float o = (v + (wet - v) * mix) * level;
            if (std::fabs(o) > 8.f)
                limited = true;
            o = mbLimit(o);
            if (!std::isfinite(o))
            { // last line of defence
                o = 0.f;
                dcX[ch] = dcY[ch] = 0.f;
                peak[ch] = 0.f;
            }
            outv[ch] = o;

            if (ch == 0)
            {
                opN = n;
                opY = ys;
            }
        }
        xfade = std::max(0.f, xfade - dt / 0.008f);

        outputs[OUT_L_OUTPUT].setVoltage(outv[0]);
        outputs[OUT_R_OUTPUT].setVoltage(outv[1]);

        // ---- scope ----
        float c[4] = {in[0], in[1], outv[0], outv[1]};
        scopeStep(c, lConn ? in[0] : in[1], dt, fs, timeScale);

        // ---- display data ----
        vTime = timeScale;
        vDrive = drive;
        vMix = mix;
        vLConn = lConn;
        vRConn = rConn;

        float pk = std::exp(-dt / 0.6f);
        peakIn = std::max(std::max(std::fabs(in[0]), std::fabs(in[1])), peakIn * pk);
        peakOut = std::max(std::max(std::fabs(outv[0]), std::fabs(outv[1])), peakOut * pk);
        limitFlash = limited ? 1.f : std::max(0.f, limitFlash - dt * 3.f);

        trTimer += dt;
        const float trStep = 1.f / 90.f;
        if (trTimer >= trStep)
        {
            trTimer -= trStep;
            if (trTimer > trStep)
                trTimer = 0.f;
            trN[trHead] = opN;
            trY[trHead] = opY;
            trHead = (trHead + 1) % MB_TRAIL_N;
            if (trCount < MB_TRAIL_N)
                trCount++;
        }
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "autoLevel", json_boolean(autoLevel));
        json_object_set_new(root, "dcBlock", json_boolean(dcBlock));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *j = json_object_get(root, "autoLevel");
        if (j)
            autoLevel = json_is_true(j);
        j = json_object_get(root, "dcBlock");
        if (j)
            dcBlock = json_is_true(j);
    }
};

// ------------------------------------------------------------
// Theme + drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace
{

    NVGcolor colBgTop() { return nvgRGB(0x25, 0x28, 0x30); }
    NVGcolor colBgBottom() { return nvgRGB(0x13, 0x15, 0x19); }
    NVGcolor colText() { return nvgRGB(0xd6, 0xdc, 0xe6); }
    NVGcolor colDim() { return nvgRGB(0x8a, 0x95, 0xa5); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    NVGcolor colBright() { return nvgRGB(0x7a, 0xec, 0xff); }
    NVGcolor colWarm() { return nvgRGB(0xff, 0x9c, 0x6a); }

    NVGcolor withA(NVGcolor c, float a) { return nvgRGBAf(c.r, c.g, c.b, a); }

    struct MBKnob
    {
        int param;
        float x;
        bool bipolar;
        int ticks;
        const char *label;
    };
    const MBKnob MB_KNOBS[5] = {
        {MagicBox::FUNC_PARAM, 35.f, false, 20, "FUNCTION"},
        {MagicBox::TIME_PARAM, 85.f, false, 11, "TIME"},
        {MagicBox::DRIVE_PARAM, 135.f, true, 11, "DRIVE"},
        {MagicBox::MIX_PARAM, 185.f, false, 11, "MIX"},
        {MagicBox::LEVEL_PARAM, 235.f, false, 11, "LEVEL"},
    };

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

    void dashedLine(NVGcontext *vg, float x0, float y0, float x1, float y1,
                    float dash, float gap, NVGcolor col)
    {
        float dx = x1 - x0, dy = y1 - y0;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-3f)
            return;
        float ux = dx / len, uy = dy / len;
        nvgBeginPath(vg);
        for (float t = 0.f; t < len; t += dash + gap)
        {
            float t2 = std::min(t + dash, len);
            nvgMoveTo(vg, x0 + ux * t, y0 + uy * t);
            nvgLineTo(vg, x0 + ux * t2, y0 + uy * t2);
        }
        nvgStrokeColor(vg, col);
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void drawLogo(NVGcontext *vg, float cx, float cy)
    {
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
        nvgRect(vg, tx + s, ty, s, s);
        nvgRect(vg, tx, ty + s * 1.4f, s, s);
        nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
        nvgFillColor(vg, colAccent());
        nvgFill(vg);

        drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
    }

    void drawKnobTicks(NVGcontext *vg, float cx, float cy, float r0, float r1, int n = 11)
    {
        nvgLineCap(vg, NVG_ROUND);
        for (int i = 0; i < n; ++i)
        {
            float a = (-0.75f + 1.5f * (float)i / (float)(n - 1)) * (float)M_PI;
            bool major = (i == 0 || i == n - 1 || (n % 2 == 1 && i == (n - 1) / 2));
            float ra = major ? r0 - 1.5f : r0;
            nvgBeginPath(vg);
            nvgMoveTo(vg, cx + std::sin(a) * ra, cy - std::cos(a) * ra);
            nvgLineTo(vg, cx + std::sin(a) * r1, cy - std::cos(a) * r1);
            nvgStrokeColor(vg, major ? nvgRGBA(0x3d, 0xe0, 0xff, 150) : nvgRGBA(255, 255, 255, 60));
            nvgStrokeWidth(vg, major ? 1.2f : 0.9f);
            nvgStroke(vg);
        }
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct MBPanelBackground : TransparentWidget
    {
        MBPanelBackground() { box.size = Vec(MB_PANEL_W, MB_PANEL_H); }

        void label(NVGcontext *vg, float x, float y, const char *txt)
        {
            drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.0f);
        }

        void plate(NVGcontext *vg, float x, float y, float w, float h)
        {
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

        void ring(NVGcontext *vg, float cx, float cy, float r)
        {
            nvgBeginPath(vg);
            nvgCircle(vg, cx, cy, r);
            nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, r * 0.3f, r,
                                               nvgRGBA(0, 0, 0, 40), nvgRGBA(0, 0, 0, 110)));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        void section(NVGcontext *vg, float y, const char *txt)
        {
            float x1 = 12.f, x2 = MB_PANEL_W - 12.f, cx = MB_PANEL_W / 2.f;
            float w = 14.f + (float)strlen(txt) * 7.f;
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

        void jackCell(NVGcontext *vg, float cx, float cy, const char *txt, bool isOutput)
        {
            const float w = 30.f, h = 44.f;
            float x = cx - w / 2.f, y = cy - 27.f;
            plate(vg, x, y, w, h);

            NVGcolor c = isOutput ? nvgRGBA(0x3d, 0xe0, 0xff, 170) : nvgRGBA(0xff, 0x9c, 0x6a, 170);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x + 6.f, y + 1.5f, w - 12.f, 1.8f, 0.9f);
            nvgFillColor(vg, c);
            nvgFill(vg);

            label(vg, cx, cy - 19.f, txt);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            // Base gradient
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, MB_PANEL_W, MB_PANEL_H);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, MB_PANEL_H, colBgTop(), colBgBottom()));
            nvgFill(vg);

            // Brushed texture
            nvgStrokeWidth(vg, 1.f);
            for (int y = 0; y < (int)MB_PANEL_H; y += 3)
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, 0, y + 0.5f);
                nvgLineTo(vg, MB_PANEL_W, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
                nvgStroke(vg);
            }

            // Corner vignette
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, MB_PANEL_W, MB_PANEL_H);
            nvgFillPaint(vg, nvgRadialGradient(vg, MB_PANEL_W / 2.f, MB_PANEL_H * 0.4f,
                                               MB_PANEL_H * 0.35f, MB_PANEL_H * 0.85f,
                                               nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
            nvgFill(vg);

            // Edge highlight
            nvgBeginPath(vg);
            nvgRect(vg, 0.5f, 0.5f, MB_PANEL_W - 1.f, MB_PANEL_H - 1.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Top accent line
            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, MB_PANEL_W, 2.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, MB_PANEL_W, 0,
                                               nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
            nvgFill(vg);

            // Logo + title
            drawLogo(vg, MB_PANEL_W / 2.f, 15.f);
            drawText(vg, MB_PANEL_W / 2.f, 32.f, "MAGICBOX", 12.f, colText(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

            // ---- Display bezel (metallic, recessed) ----
            float bx = MB_DISP_X - 5.f, by = MB_DISP_Y - 5.f;
            float bw = MB_DISP_W + 10.f, bh = MB_DISP_H + 10.f;

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

            // ---- Knobs ----
            section(vg, MB_SECTION_CTRL, "CONTROL");
            for (int k = 0; k < 5; ++k)
            {
                const MBKnob &kn = MB_KNOBS[k];
                ring(vg, kn.x, MB_KNOB_Y, 17.f);
                drawKnobTicks(vg, kn.x, MB_KNOB_Y, 21.f, 24.f, kn.ticks);
                label(vg, kn.x, MB_KNOB_Y + 29.f, kn.label);
            }

            // ---- I/O ----
            section(vg, MB_SECTION_IO, "I / O");
            const char *io[6] = {"IN L", "IN R", "FN CV", "TM CV", "OUT L", "OUT R"};
            for (int i = 0; i < 6; ++i)
                jackCell(vg, MB_IO_X[i], MB_IO_Y, io[i], i >= 4);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Live value arcs around the knobs
    // ------------------------------------------------------------
    struct MBKnobArcs : TransparentWidget
    {
        MagicBox *module = nullptr;

        MBKnobArcs(MagicBox *m) : module(m) { box.size = Vec(MB_PANEL_W, MB_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float R = 17.2f;
            const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
            const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

            for (int k = 0; k < 5; ++k)
            {
                const MBKnob &kn = MB_KNOBS[k];

                nvgBeginPath(vg);
                nvgArc(vg, kn.x, MB_KNOB_Y, R, a0, a1, NVG_CW);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
                nvgStrokeWidth(vg, 2.f);
                nvgLineCap(vg, NVG_ROUND);
                nvgStroke(vg);

                if (!module)
                    continue;
                float v = module->paramQuantities[kn.param]->getScaledValue(); // 0..1
                float from = kn.bipolar ? 0.5f : 0.f;
                float lo = std::min(from, v), hi = std::max(from, v);
                if (hi - lo < 0.004f)
                    continue;

                float s = a0 + (a1 - a0) * lo;
                float e = a0 + (a1 - a0) * hi;
                NVGcolor col = (kn.bipolar && v < 0.5f) ? colWarm() : colAccent();

                nvgBeginPath(vg);
                nvgArc(vg, kn.x, MB_KNOB_Y, R, s, e, NVG_CW);
                nvgStrokeColor(vg, withA(col, 0.18f));
                nvgStrokeWidth(vg, 5.f);
                nvgStroke(vg);

                nvgBeginPath(vg);
                nvgArc(vg, kn.x, MB_KNOB_Y, R, s, e, NVG_CW);
                nvgStrokeColor(vg, withA(col, 0.95f));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }
        }
    };

    // ------------------------------------------------------------
    // Display: transfer curve, harmonics, IN / OUT scopes
    // ------------------------------------------------------------
    struct MBDisplayWidget : TransparentWidget
    {
        MagicBox *module = nullptr;

        // harmonic cache
        int hFunc = -1;
        float hDrive = -1.f;
        float hMag[8] = {};

        MBDisplayWidget(MagicBox *m) : module(m)
        {
            box.pos = Vec(MB_DISP_X, MB_DISP_Y);
            box.size = Vec(MB_DISP_W, MB_DISP_H);
        }

        static float scopeHH(float y0, float y1) { return (y1 - y0) * 0.5f - 1.5f; }

        // ---- static scope frame + grid ----
        void scopeFrame(NVGcontext *vg, float x0, float y0, float x1, float y1)
        {
            float w = x1 - x0, yc = (y0 + y1) * 0.5f, hh = scopeHH(y0, y1);

            nvgBeginPath(vg);
            nvgRoundedRect(vg, x0, y0, w, y1 - y0, 3.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 5));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            for (int i = 1; i < 4; ++i)
            {
                float x = x0 + w * (float)i / 4.f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x + 0.5f, y0 + 1.f);
                nvgLineTo(vg, x + 0.5f, y1 - 1.f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            const float vs[3] = {5.f, 0.f, -5.f};
            for (int i = 0; i < 3; ++i)
            {
                float y = yc - vs[i] / 10.f * hh;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x0 + 1.f, y + 0.5f);
                nvgLineTo(vg, x1 - 1.f, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, vs[i] == 0.f ? 44 : 14));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            // limiter threshold (+-8 V)
            for (int s = -1; s <= 1; s += 2)
            {
                float y = yc - (float)s * 0.8f * hh;
                dashedLine(vg, x0 + 1.f, y + 0.5f, x1 - 1.f, y + 0.5f, 2.f, 3.f,
                           nvgRGBA(0xff, 0x9c, 0x6a, 46));
            }
        }

        // ---- static background ----
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            nvgBeginPath(vg);
            nvgRoundedRect(vg, 0, 0, MB_DISP_W, MB_DISP_H, 4.f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, MB_DISP_H,
                                               nvgRGB(0x0a, 0x16, 0x1f), nvgRGB(0x03, 0x06, 0x09)));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgRoundedRect(vg, 0.5f, 0.5f, MB_DISP_W - 1.f, MB_DISP_H - 1.f, 4.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Transfer plot frame
            {
                const float X0 = MB_TF_X0, Y0 = MB_TF_Y0, S = MB_TF_S;
                nvgBeginPath(vg);
                nvgRoundedRect(vg, X0, Y0, S, S, 3.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 5));
                nvgFill(vg);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                for (int i = 1; i < 4; ++i)
                {
                    float t = (float)i / 4.f * S;
                    bool mid = (i == 2);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, X0 + t + 0.5f, Y0 + 1.f);
                    nvgLineTo(vg, X0 + t + 0.5f, Y0 + S - 1.f);
                    nvgMoveTo(vg, X0 + 1.f, Y0 + t + 0.5f);
                    nvgLineTo(vg, X0 + S - 1.f, Y0 + t + 0.5f);
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, mid ? 40 : 13));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                // identity (no processing)
                dashedLine(vg, X0 + 1.f, Y0 + S - 1.f, X0 + S - 1.f, Y0 + 1.f, 3.f, 3.f,
                           nvgRGBA(255, 255, 255, 34));
            }

            // Harmonic grid (-12, -24, -36 dB) and number row
            {
                float hgt = MB_HM_BASE - MB_HM_TOP;
                for (int i = 0; i <= 4; ++i)
                {
                    float y = MB_HM_TOP + hgt * (float)i / 4.f;
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, MB_HM_X0, y + 0.5f);
                    nvgLineTo(vg, MB_HM_X0 + MB_HM_W, y + 0.5f);
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, i == 4 ? 40 : 12));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                float pitch = MB_HM_W / 8.f;
                for (int k = 0; k < 8; ++k)
                {
                    char b[4];
                    snprintf(b, sizeof(b), "%d", k + 1);
                    drawText(vg, MB_HM_X0 + ((float)k + 0.5f) * pitch, MB_HM_LBL_Y, b, 5.f,
                             nvgRGBA(0x8a, 0x95, 0xa5, 190), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
                }
            }

            NVGcolor lc = nvgRGBA(0x8a, 0x95, 0xa5, 170);
            drawText(vg, MB_TF_X0, 90.5f, "TRANSFER", 5.2f, lc, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            drawText(vg, MB_TF_X0, 101.f, "HARMONICS", 5.2f, lc, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

            scopeFrame(vg, MB_SC_X0, MB_SCI_Y0, MB_SC_X1, MB_SCI_Y1);
            scopeFrame(vg, MB_SC_X0, MB_SCO_Y0, MB_SC_X1, MB_SCO_Y1);

            Widget::draw(args);
        }

        // ---- harmonic analysis of a full-scale sine through the shaper ----
        void updateHarmonics(int func, float drive)
        {
            if (func == hFunc && std::fabs(drive - hDrive) < 1e-4f)
                return;
            hFunc = func;
            hDrive = drive;
            const int N = 128;
            float re[8] = {}, im[8] = {};
            for (int i = 0; i < N; ++i)
            {
                float th = 6.2831853f * (float)i / (float)N;
                float y = mbShape(mbSoftClip(drive * std::sin(th)), func);
                for (int k = 0; k < 8; ++k)
                {
                    re[k] += y * std::cos((float)(k + 1) * th);
                    im[k] += y * std::sin((float)(k + 1) * th);
                }
            }
            for (int k = 0; k < 8; ++k)
                hMag[k] = 2.f * std::sqrt(re[k] * re[k] + im[k] * im[k]) / (float)N;
        }

        // ---- one scope ----
        void drawScope(NVGcontext *vg, float x0, float y0, float x1, float y1,
                       float (*mn)[MB_COLS], float (*mx)[MB_COLS], const bool *valid,
                       int chL, int chR, bool showR, int cursor, bool dimOld)
        {
            const float w = x1 - x0, yc = (y0 + y1) * 0.5f, hh = scopeHH(y0, y1);

            nvgSave(vg);
            nvgScissor(vg, x0, y0, w, y1 - y0);

            auto yMap = [&](float v)
            {
                v = rack::math::clamp(v, -10.5f, 10.5f);
                return yc - v / 10.f * hh;
            };
            auto xMap = [&](int i)
            { return x0 + ((float)i + 0.5f) / (float)MB_COLS * w; };

            auto trace = [&](int ch, int a, int b, NVGcolor col, float alpha)
            {
                a = std::max(a, 0);
                b = std::min(b, MB_COLS);
                if (b - a < 2)
                    return;

                // envelope
                int n = 0;
                nvgBeginPath(vg);
                for (int i = a; i < b; ++i)
                {
                    if (!valid[i])
                        continue;
                    float x = xMap(i), y = yMap(mx[ch][i]);
                    if (!n)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                    n++;
                }
                if (n < 2)
                    return;
                for (int i = b - 1; i >= a; --i)
                {
                    if (!valid[i])
                        continue;
                    nvgLineTo(vg, xMap(i), yMap(mn[ch][i]));
                }
                nvgClosePath(vg);
                nvgFillColor(vg, withA(col, 0.16f * alpha));
                nvgFill(vg);

                nvgLineCap(vg, NVG_ROUND);
                nvgLineJoin(vg, NVG_ROUND);
                for (int pass = 0; pass < 2; ++pass)
                {
                    float (*arr)[MB_COLS] = (pass == 0) ? mx : mn;
                    for (int layerI = 0; layerI < 2; ++layerI)
                    {
                        nvgBeginPath(vg);
                        bool first = true;
                        for (int i = a; i < b; ++i)
                        {
                            if (!valid[i])
                                continue;
                            float x = xMap(i), y = yMap(arr[ch][i]);
                            if (first)
                            {
                                nvgMoveTo(vg, x, y);
                                first = false;
                            }
                            else
                                nvgLineTo(vg, x, y);
                        }
                        if (layerI == 0)
                        {
                            nvgStrokeColor(vg, withA(col, 0.12f * alpha));
                            nvgStrokeWidth(vg, 3.2f);
                        }
                        else
                        {
                            nvgStrokeColor(vg, withA(col, 0.95f * alpha));
                            nvgStrokeWidth(vg, 1.1f);
                        }
                        nvgStroke(vg);
                    }
                }
            };

            int split = dimOld ? cursor : MB_COLS;
            if (showR)
            {
                trace(chR, 0, split, colWarm(), 0.9f);
                if (dimOld)
                    trace(chR, std::max(split - 1, 0), MB_COLS, colWarm(), 0.30f);
            }
            trace(chL, 0, split, colBright(), 1.f);
            if (dimOld)
                trace(chL, std::max(split - 1, 0), MB_COLS, colBright(), 0.32f);

            // sweep cursor
            if (dimOld && cursor < MB_COLS)
            {
                float x = x0 + (float)cursor / (float)MB_COLS * w;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x + 0.5f, y0 + 1.f);
                nvgLineTo(vg, x + 0.5f, y1 - 1.f);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 90));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            nvgRestore(vg);
        }

        // ---- live content ----
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            // ----- snapshot of the state -----
            int func = 2;
            float drive = 1.f, timeScale = 0.022f, peakIn = 4.5f, peakOut = 4.8f, limitF = 0.f;
            float opN = 0.5f, opY = 0.f;
            bool lConn = true, rConn = true, autoLvl = true, dcB = true, trig = true;
            int capState = MagicBox::CAP_ARMED, capCol = MB_COLS;
            int trHead = 0, trCount = 0;
            float mn[4][MB_COLS], mx[4][MB_COLS];
            bool valid[MB_COLS];
            float trN[MB_TRAIL_N], trY[MB_TRAIL_N];

            if (module)
            {
                func = module->funcCur;
                drive = module->vDrive;
                timeScale = module->vTime;
                peakIn = module->peakIn;
                peakOut = module->peakOut;
                limitF = module->limitFlash;
                opN = module->opN;
                opY = module->opY;
                lConn = module->vLConn;
                rConn = module->vRConn;
                autoLvl = module->autoLevel;
                dcB = module->dcBlock;
                trig = module->lastTrig;
                capState = module->capState;
                capCol = module->capCol;
                trHead = module->trHead;
                trCount = module->trCount;
                std::memcpy(mn, module->scMin, sizeof(mn));
                std::memcpy(mx, module->scMax, sizeof(mx));
                std::memcpy(valid, module->scValid, sizeof(valid));
                std::memcpy(trN, module->trN, sizeof(trN));
                std::memcpy(trY, module->trY, sizeof(trY));
            }
            else
            {
                // demo values for the module browser
                for (int i = 0; i < MB_COLS; ++i)
                {
                    float t = (float)i / (float)(MB_COLS - 1);
                    float a = std::sin(6.2831853f * 3.f * t);
                    float b = std::sin(6.2831853f * 3.f * t + 0.7f);
                    mn[0][i] = mx[0][i] = 4.5f * a;
                    mn[1][i] = mx[1][i] = 4.5f * b;
                    mn[2][i] = mx[2][i] = 5.f * mbShape(mbSoftClip(a), func);
                    mn[3][i] = mx[3][i] = 5.f * mbShape(mbSoftClip(b), func);
                    valid[i] = true;
                }
                opN = 0.55f;
                opY = mbShape(mbSoftClip(opN), func);
                trCount = MB_TRAIL_N;
                for (int i = 0; i < MB_TRAIL_N; ++i)
                {
                    trN[i] = std::sin(0.22f * (float)i);
                    trY[i] = mbShape(mbSoftClip(trN[i]), func);
                }
            }
            func = rack::math::clamp(func, 0, MB_NUM_FUNCS - 1);
            updateHarmonics(func, drive);

            nvgSave(vg);
            nvgScissor(vg, 0, 0, MB_DISP_W, MB_DISP_H);

            // ================= transfer curve =================
            {
                const float X0 = MB_TF_X0, Y0 = MB_TF_Y0, S = MB_TF_S;
                auto px = [&](float x)
                { return X0 + (rack::math::clamp(x, -1.1f, 1.1f) + 1.f) * 0.5f * S; };
                auto py = [&](float y)
                { return Y0 + S - (rack::math::clamp(y, -1.1f, 1.1f) + 1.f) * 0.5f * S; };

                nvgSave(vg);
                nvgScissor(vg, X0, Y0, S, S);

                const int N = 96;
                float cx[N], cy[N];
                for (int i = 0; i < N; ++i)
                {
                    float x = -1.f + 2.f * (float)i / (float)(N - 1);
                    cx[i] = px(x);
                    cy[i] = py(mbShape(mbSoftClip(x * drive), func));
                }

                // fill between the curve and the zero axis
                nvgBeginPath(vg);
                nvgMoveTo(vg, cx[0], py(0.f));
                for (int i = 0; i < N; ++i)
                    nvgLineTo(vg, cx[i], cy[i]);
                nvgLineTo(vg, cx[N - 1], py(0.f));
                nvgClosePath(vg);
                nvgFillColor(vg, withA(colAccent(), 0.10f));
                nvgFill(vg);

                nvgLineCap(vg, NVG_ROUND);
                nvgLineJoin(vg, NVG_ROUND);
                nvgBeginPath(vg);
                nvgMoveTo(vg, cx[0], cy[0]);
                for (int i = 1; i < N; ++i)
                    nvgLineTo(vg, cx[i], cy[i]);
                nvgStrokeColor(vg, withA(colAccent(), 0.16f));
                nvgStrokeWidth(vg, 4.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, cx[0], cy[0]);
                for (int i = 1; i < N; ++i)
                    nvgLineTo(vg, cx[i], cy[i]);
                nvgStrokeColor(vg, withA(colBright(), 0.95f));
                nvgStrokeWidth(vg, 1.3f);
                nvgStroke(vg);

                // recent operating points (they all lie on the curve)
                for (int i = 0; i < trCount; ++i)
                {
                    int idx = ((trHead - trCount + i) % MB_TRAIL_N + MB_TRAIL_N) % MB_TRAIL_N;
                    float age = (float)(i + 1) / (float)trCount;
                    nvgBeginPath(vg);
                    nvgCircle(vg, px(trN[idx]), py(trY[idx]), 1.1f);
                    nvgFillColor(vg, withA(colWarm(), 0.55f * age * age));
                    nvgFill(vg);
                }

                // live point with guides
                float mxp = px(opN), myp = py(opY);
                dashedLine(vg, mxp + 0.5f, py(0.f), mxp + 0.5f, myp, 2.f, 2.f, withA(colWarm(), 0.5f));
                dashedLine(vg, px(0.f), myp + 0.5f, mxp, myp + 0.5f, 2.f, 2.f, withA(colBright(), 0.5f));
                nvgBeginPath(vg);
                nvgCircle(vg, mxp, myp, 5.f);
                nvgFillColor(vg, withA(colBright(), 0.25f));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, mxp, myp, 2.3f);
                nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, 0.97f));
                nvgFill(vg);

                nvgRestore(vg);
            }

            // ================= harmonic bars =================
            {
                float pitch = MB_HM_W / 8.f, bw = pitch - 2.5f;
                float hgt = MB_HM_BASE - MB_HM_TOP;
                for (int k = 0; k < 8; ++k)
                {
                    float db = 20.f * std::log10(std::max(hMag[k], 1e-5f));
                    float f = rack::math::clamp((db + 48.f) / 48.f, 0.f, 1.f);
                    float h = f * hgt;
                    float x = MB_HM_X0 + (float)k * pitch + (pitch - bw) * 0.5f;
                    bool odd = (k % 2 == 0); // harmonics 1, 3, 5, 7
                    NVGcolor col = odd ? colAccent() : colWarm();
                    NVGcolor hi = odd ? colBright() : nvgRGB(0xff, 0xc8, 0xa8);
                    if (h < 0.5f)
                        continue;

                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, x - 1.5f, MB_HM_BASE - h - 1.5f, bw + 3.f, h + 1.5f, 2.f);
                    nvgFillColor(vg, withA(col, 0.12f));
                    nvgFill(vg);

                    nvgBeginPath(vg);
                    nvgRect(vg, x, MB_HM_BASE - h, bw, h);
                    nvgFillPaint(vg, nvgLinearGradient(vg, 0, MB_HM_BASE, 0, MB_HM_BASE - h,
                                                       withA(col, 0.25f), withA(col, 0.95f)));
                    nvgFill(vg);

                    nvgBeginPath(vg);
                    nvgRect(vg, x - 0.5f, MB_HM_BASE - h - 0.5f, bw + 1.f, 1.5f);
                    nvgFillColor(vg, withA(hi, 0.95f));
                    nvgFill(vg);
                }
            }

            // ================= scopes =================
            bool dimOld = (timeScale >= 0.1f);
            int cursor = (capState == MagicBox::CAP_RUN) ? capCol : MB_COLS;
            drawScope(vg, MB_SC_X0, MB_SCI_Y0, MB_SC_X1, MB_SCI_Y1, mn, mx, valid, 0, 1, rConn, cursor, dimOld);
            drawScope(vg, MB_SC_X0, MB_SCO_Y0, MB_SC_X1, MB_SCO_Y1, mn, mx, valid, 2, 3, rConn, cursor, dimOld);

            // scope labels
            {
                char buf[32];
                NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 210);
                NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 220);

                drawText(vg, MB_SC_X0 + 4.f, MB_SCI_Y0 + 6.f, rConn ? "IN L/R" : "IN", 5.5f, cd,
                         NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
                snprintf(buf, sizeof(buf), "PK %.1fV", peakIn);
                drawText(vg, MB_SC_X1 - 4.f, MB_SCI_Y0 + 6.f, buf, 5.5f, c1,
                         NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);

                drawText(vg, MB_SC_X0 + 4.f, MB_SCO_Y0 + 6.f, rConn ? "OUT L/R" : "OUT", 5.5f, cd,
                         NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
                snprintf(buf, sizeof(buf), "PK %.1fV", peakOut);
                drawText(vg, MB_SC_X1 - 4.f, MB_SCO_Y0 + 6.f, buf, 5.5f, c1,
                         NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);

                // limiter lamp
                float lx = (MB_SC_X0 + MB_SC_X1) * 0.5f + 8.f, ly = MB_SCO_Y0 + 6.f;
                nvgBeginPath(vg);
                nvgRoundedRect(vg, lx - 12.f, ly - 4.f, 24.f, 8.f, 2.5f);
                if (limitF > 0.02f)
                {
                    nvgFillColor(vg, withA(colWarm(), 0.20f + 0.45f * limitF));
                    nvgFill(vg);
                    nvgStrokeColor(vg, withA(colWarm(), 0.9f));
                }
                else
                {
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
                }
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                drawText(vg, lx, ly + 0.5f, "LIMIT", 5.2f,
                         limitF > 0.02f ? colWarm() : nvgRGBA(0x8a, 0x95, 0xa5, 150),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.6f);
            }

            // ================= header readouts =================
            {
                char buf[48];
                NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 230);
                NVGcolor cm = nvgRGBA(0xff, 0x9c, 0x6a, 230);
                NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 230);

                drawText(vg, 6.f, 8.f, MB_NAMES[func], 7.5f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

                snprintf(buf, sizeof(buf), "%s%s%s", autoLvl ? "AUTO" : "FIXED",
                         dcB ? "+DC" : "", lConn || rConn ? "" : "  NO IN");
                drawText(vg, MB_DISP_W / 2.f + 6.f, 8.f, buf, 6.f, cd,
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

                if (timeScale < 0.1f)
                    snprintf(buf, sizeof(buf), "%.1f ms", timeScale * 1000.f);
                else if (timeScale < 1.f)
                    snprintf(buf, sizeof(buf), "%.0f ms", timeScale * 1000.f);
                else
                    snprintf(buf, sizeof(buf), "%.2f s", timeScale);
                drawText(vg, MB_DISP_W - 6.f, 8.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

                // trigger status next to the time
                drawText(vg, MB_DISP_W - 50.f, 8.f, trig ? "TRIG" : "AUTO", 5.5f,
                         trig ? colBright() : nvgRGBA(0x8a, 0x95, 0xa5, 170),
                         NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
            }

            // ================= scanlines, vignette, glass =================
            nvgBeginPath(vg);
            for (int y = 0; y < (int)MB_DISP_H; y += 3)
                nvgRect(vg, 0, y + 1, MB_DISP_W, 1);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, MB_DISP_W, MB_DISP_H);
            nvgFillPaint(vg, nvgRadialGradient(vg, MB_DISP_W / 2.f, MB_DISP_H / 2.f,
                                               MB_DISP_W * 0.45f, MB_DISP_W * 0.88f,
                                               nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 115)));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgRect(vg, 0, 0, MB_DISP_W, MB_DISP_H * 0.30f);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, MB_DISP_H * 0.30f,
                                               nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
            nvgFill(vg);

            nvgRestore(vg);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct MagicBoxWidget : ModuleWidget
{
    MagicBoxWidget(MagicBox *module)
    {
        setModule(module);
        box.size = Vec(MB_PANEL_W, MB_PANEL_H);

        addChild(new MBPanelBackground());
        addChild(new MBKnobArcs(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new MBDisplayWidget(module));

        // Knobs
        for (int k = 0; k < 5; ++k)
        {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(MB_KNOBS[k].x, MB_KNOB_Y), module, MB_KNOBS[k].param));
        }

        // I/O
        addInput(createInputCentered<DarkPJ301MPort>(Vec(MB_IO_X[0], MB_IO_Y), module, MagicBox::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(MB_IO_X[1], MB_IO_Y), module, MagicBox::IN_R_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(MB_IO_X[2], MB_IO_Y), module, MagicBox::FUNC_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(MB_IO_X[3], MB_IO_Y), module, MagicBox::TIME_CV_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(MB_IO_X[4], MB_IO_Y), module, MagicBox::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(MB_IO_X[5], MB_IO_Y), module, MagicBox::OUT_R_OUTPUT));
    }

    void appendContextMenu(ui::Menu *menu) override
    {
        MagicBox *module = dynamic_cast<MagicBox *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Function", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             int cur = (int)std::lround(module->params[MagicBox::FUNC_PARAM].getValue());
                                             for (int i = 0; i < MB_NUM_FUNCS; ++i)
                                             {
                                                 std::string t = std::string(MB_NAMES[i]) + "  -  " + MB_DESC[i];
                                                 submenu->addChild(createMenuItem(t, (cur == i) ? "✔" : "",
                                                                                  [=]()
                                                                                  { module->params[MagicBox::FUNC_PARAM].setValue((float)i); }));
                                             }
                                         }));

        menu->addChild(createBoolPtrMenuItem("Auto level (normalise to input envelope)", "", &module->autoLevel));
        menu->addChild(createBoolPtrMenuItem("DC blocker (turn off for CV signals)", "", &module->dcBlock));
    }
};

Model *modelMagicBox = createModel<MagicBox, MagicBoxWidget>("MagicBox");