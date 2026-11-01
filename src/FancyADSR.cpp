#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float FA_PANEL_W = RACK_GRID_WIDTH * 20;   // 300
static const float FA_PANEL_H = RACK_GRID_HEIGHT;       // 380

// Display
static const float FA_DISP_X = 30.f;
static const float FA_DISP_Y = 44.f;
static const float FA_DISP_W = 240.f;
static const float FA_DISP_H = 92.f;

// Scope area inside the display
static const float FA_SX0 = 16.f;
static const float FA_SY0 = 16.f;
static const float FA_SW  = 200.f;
static const float FA_SH  = 56.f;

// Rows
static const float FA_ROWA_TOP = 164.f;   // envelope knobs + CV jacks
static const float FA_ROWA_H   = 84.f;
static const float FA_ROWB_TOP = 254.f;   // shapes, loop, time
static const float FA_ROWB_H   = 56.f;
static const float FA_ROWC_TOP = 316.f;   // I/O
static const float FA_ROWC_H   = 46.f;
static const float FA_CELL_W   = 66.f;

static float faColX(int i) { return 37.5f + 75.f * i; }

// ------------------------------------------------------------
// Fancy ADSR Module
// ------------------------------------------------------------
struct FancyADSR : Module
{
    enum ParamIds
    {
        ATTACK_PARAM,
        DECAY_PARAM,
        SUSTAIN_PARAM,
        RELEASE_PARAM,
        ATTACK_SHAPE_PARAM,   // 0 = linear, 1 = exponential
        RELEASE_SHAPE_PARAM,
        LOOP_TOGGLE_PARAM,
        TIME_SCALE_PARAM,     // seconds per full graph width
        NUM_PARAMS
    };
    enum InputIds
    {
        GATE_INPUT,
        TRIGGER_INPUT,
        ATTACK_CV_INPUT,
        DECAY_CV_INPUT,
        SUSTAIN_CV_INPUT,
        RELEASE_CV_INPUT,
        NUM_INPUTS
    };
    enum OutputIds
    {
        ENV_OUTPUT,
        EOR_OUTPUT,
        NUM_OUTPUTS
    };
    enum LightIds
    {
        LOOP_LIGHT,
        NUM_LIGHTS
    };

    enum Stage { STAGE_IDLE, STAGE_ATTACK, STAGE_DECAY, STAGE_SUSTAIN, STAGE_RELEASE };
    Stage stage = STAGE_IDLE;
    float env = 0.0f;
    float stageTime = 0.0f;
    float eorPulseRemaining = 0.0f;
    bool lastGateState = false;
    bool lastTriggerState = false;

    // Real-time graph history (0-10V)
    static const int HISTORY_SIZE = 400;
    float history[HISTORY_SIZE];
    int historyCount = 0;
    int historyWritePos = 0;
    float sampleAccumulator = 0.0f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float dispAttack = 0.01f, dispDecay = 0.1f, dispSustain = 0.8f, dispRelease = 0.5f;
    float dispAttackShape = 1.f, dispReleaseShape = 1.f;
    bool  dispGate = false;
    float eorFlash = 0.f;

    // Parameter helpers (with CV)
    float getAttack()    { return rack::math::clamp(params[ATTACK_PARAM].getValue() + inputs[ATTACK_CV_INPUT].getVoltage() / 5.0f, 0.001f, 10.0f); }
    float getDecay()     { return rack::math::clamp(params[DECAY_PARAM].getValue()  + inputs[DECAY_CV_INPUT].getVoltage()  / 5.0f, 0.001f, 10.0f); }
    float getSustain()   { return rack::math::clamp(params[SUSTAIN_PARAM].getValue() + inputs[SUSTAIN_CV_INPUT].getVoltage() / 5.0f, 0.0f, 1.0f); }
    float getRelease()   { return rack::math::clamp(params[RELEASE_PARAM].getValue() + inputs[RELEASE_CV_INPUT].getVoltage() / 5.0f, 0.001f, 10.0f); }
    bool  getLoop()      { return params[LOOP_TOGGLE_PARAM].getValue() > 0.5f; }
    float getTimeScale() { return rack::math::clamp(params[TIME_SCALE_PARAM].getValue(), 0.05f, 5.0f); }

    FancyADSR()
    {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configParam(ATTACK_PARAM,    0.001f, 10.0f, 0.01f, "Attack",      " s");
        configParam(DECAY_PARAM,     0.001f, 10.0f, 0.1f,  "Decay",       " s");
        configParam(SUSTAIN_PARAM,   0.0f,   1.0f,  0.8f,  "Sustain",     "%", 0.f, 100.f);
        configParam(RELEASE_PARAM,   0.001f, 10.0f, 0.5f,  "Release",     " s");
        configSwitch(ATTACK_SHAPE_PARAM, 0.f, 1.f, 1.f, "Attack shape", {"Linear", "Exponential"});
        configSwitch(RELEASE_SHAPE_PARAM, 0.f, 1.f, 1.f, "Release shape", {"Linear", "Exponential"});
        configSwitch(LOOP_TOGGLE_PARAM, 0.f, 1.f, 0.f, "Loop mode", {"Off", "On"});
        configParam(TIME_SCALE_PARAM, 0.05f, 5.0f, 1.0f, "Scope time (full width)", " s");

        configInput(GATE_INPUT,      "Gate");
        configInput(TRIGGER_INPUT,   "Retrigger");
        configInput(ATTACK_CV_INPUT, "Attack CV (5 V adds 1 s)");
        configInput(DECAY_CV_INPUT,  "Decay CV (5 V adds 1 s)");
        configInput(SUSTAIN_CV_INPUT,"Sustain CV (5 V adds 100%)");
        configInput(RELEASE_CV_INPUT,"Release CV (5 V adds 1 s)");
        configOutput(ENV_OUTPUT,     "Envelope (0 to 10 V)");
        configOutput(EOR_OUTPUT,     "End of release");

        configLight(LOOP_LIGHT, "Loop mode");

        onReset();
    }

    void onReset() override
    {
        stage = STAGE_IDLE;
        env = 0.0f;
        stageTime = 0.0f;
        eorPulseRemaining = 0.0f;
        lastGateState = false;
        lastTriggerState = false;
        historyCount = 0;
        historyWritePos = 0;
        sampleAccumulator = 0.0f;
        for (int i = 0; i < HISTORY_SIZE; ++i) history[i] = 0.0f;
    }

    // Helper: exponential that reaches target (1 or sustain) after 'duration' seconds
    // shape: 0=linear, 1=exponential (with proper timing)
    float runSegment(float start, float target, float duration, float t, float shape)
    {
        if (duration <= 0.0f) return target;
        float alpha = rack::math::clamp(t / duration, 0.0f, 1.0f);
        if (shape <= 0.5f) // linear
        {
            return start + (target - start) * alpha;
        }
        else // exponential: tau = duration / -ln(0.001) so that at t=duration, value is within 0.1% of target
        {
            const float eps = 0.001f;
            float tau = duration / -std::log(eps);
            float factor = 1.0f - std::exp(-t / tau);
            return start + (target - start) * factor;
        }
    }

    void onGate(bool high)
    {
        if (high)
        {
            if (stage == STAGE_IDLE || stage == STAGE_RELEASE)
            {
                stage = STAGE_ATTACK;
                stageTime = 0.0f;
            }
            else if (stage == STAGE_SUSTAIN && getLoop())
            {
                stage = STAGE_ATTACK;
                stageTime = 0.0f;
            }
        }
        else
        {
            if (stage == STAGE_ATTACK || stage == STAGE_DECAY || stage == STAGE_SUSTAIN)
            {
                stage = STAGE_RELEASE;
                stageTime = 0.0f;
            }
        }
    }

    void onTrigger(bool high)
    {
        if (high && stage != STAGE_IDLE)
        {
            stage = STAGE_ATTACK;
            stageTime = 0.0f;
        }
    }

    void process(const ProcessArgs &args) override
    {
        float sampleTime = args.sampleTime;

        // Gate / trigger edges
        bool gateHigh = inputs[GATE_INPUT].getVoltage() > 1.0f;
        if (gateHigh && !lastGateState) onGate(true);
        else if (!gateHigh && lastGateState) onGate(false);
        lastGateState = gateHigh;

        bool triggerHigh = inputs[TRIGGER_INPUT].getVoltage() > 1.0f;
        if (triggerHigh && !lastTriggerState) onTrigger(true);
        lastTriggerState = triggerHigh;

        float attackTime  = getAttack();
        float decayTime   = getDecay();
        float sustainLev  = getSustain();
        float releaseTime = getRelease();
        float attackShape  = params[ATTACK_SHAPE_PARAM].getValue();
        float releaseShape = params[RELEASE_SHAPE_PARAM].getValue();

        // Advance envelope
        switch (stage)
        {
            case STAGE_ATTACK:
                env = runSegment(0.0f, 1.0f, attackTime, stageTime, attackShape);
                stageTime += sampleTime;
                if (stageTime >= attackTime)
                {
                    env = 1.0f;
                    stage = STAGE_DECAY;
                    stageTime = 0.0f;
                }
                break;

            case STAGE_DECAY:
                // Decay is always exponential (typical ADSR), timing is correct
                env = runSegment(1.0f, sustainLev, decayTime, stageTime, 1.0f);
                stageTime += sampleTime;
                if (stageTime >= decayTime)
                {
                    env = sustainLev;
                    stage = STAGE_SUSTAIN;
                    stageTime = 0.0f;
                }
                break;

            case STAGE_SUSTAIN:
                env = sustainLev;
                if (getLoop() && gateHigh)
                {
                    stage = STAGE_ATTACK;
                    stageTime = 0.0f;
                }
                break;

            case STAGE_RELEASE:
                env = runSegment(env, 0.0f, releaseTime, stageTime, releaseShape);
                stageTime += sampleTime;
                if (stageTime >= releaseTime)
                {
                    env = 0.0f;
                    stage = STAGE_IDLE;
                    eorPulseRemaining = 0.005f; // 5 ms pulse
                    eorFlash = 1.f;
                }
                break;

            default: // IDLE
                env = 0.0f;
                break;
        }

        float outVoltage = env * 10.0f;
        outputs[ENV_OUTPUT].setVoltage(outVoltage);

        // End-of-release trigger
        if (eorPulseRemaining > 0.0f)
        {
            outputs[EOR_OUTPUT].setVoltage(10.0f);
            eorPulseRemaining -= sampleTime;
            if (eorPulseRemaining <= 0.0f)
                outputs[EOR_OUTPUT].setVoltage(0.0f);
        }
        else
        {
            outputs[EOR_OUTPUT].setVoltage(0.0f);
        }

        lights[LOOP_LIGHT].setBrightness(getLoop() ? 1.0f : 0.0f);

        // ---- display data ----
        dispAttack = attackTime;
        dispDecay = decayTime;
        dispSustain = sustainLev;
        dispRelease = releaseTime;
        dispAttackShape = attackShape;
        dispReleaseShape = releaseShape;
        dispGate = gateHigh;
        eorFlash = std::max(0.f, eorFlash - 5.f * sampleTime);

        // ----- Real-time graph sampling -----
        float timeScale = getTimeScale();
        float sampleInterval = timeScale / HISTORY_SIZE;
        sampleAccumulator += sampleTime;
        while (sampleAccumulator >= sampleInterval)
        {
            history[historyWritePos] = outVoltage;
            historyWritePos = (historyWritePos + 1) % HISTORY_SIZE;
            if (historyCount < HISTORY_SIZE) historyCount++;
            sampleAccumulator -= sampleInterval;
        }
    }

    // For graph widget access
    int getHistorySize() const { return historyCount; }
    float getHistorySample(int index) const
    {
        if (index < 0 || index >= historyCount) return 0.0f;
        int oldest = (historyWritePos - historyCount + HISTORY_SIZE) % HISTORY_SIZE;
        return history[(oldest + index) % HISTORY_SIZE];
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace {

// ------------------------------------------------------------
// Theme (same palette as the other AGMM modules)
// ------------------------------------------------------------
NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colViolet()   { return nvgRGB(0xb0, 0x5a, 0xff); }

NVGcolor withAlpha(NVGcolor c, float a) {
    c.a = a;
    return c;
}

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

// Cyan LED (Rack has no built-in one)
template <typename TBase = GrayModuleLightWidget>
struct TFACyanLight : TBase {
    TFACyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using FACyanLight = TFACyanLight<>;

// ------------------------------------------------------------
// AGMM logo
// ------------------------------------------------------------
void drawLogo(NVGcontext* vg, float cx, float cy) {
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
    nvgRect(vg, tx + s,       ty,            s, s);
    nvgRect(vg, tx,           ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);

    // Wordmark
    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
             NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

// Same segment maths as the module (used for the ideal-curve preview)
float segCurve(float start, float target, float dur, float t, float shape) {
    if (dur <= 0.f) return target;
    float a = std::min(std::max(t / dur, 0.f), 1.f);
    if (shape <= 0.5f)
        return start + (target - start) * a;
    const float eps = 0.001f;
    float tau = dur / -std::log(eps);
    return start + (target - start) * (1.f - std::exp(-t / tau));
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct FAPanelBackground : TransparentWidget {
    FAPanelBackground() { box.size = Vec(FA_PANEL_W, FA_PANEL_H); }

    void label(NVGcontext* vg, float x, float y, const char* txt) {
        drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
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

    // Full-width divider with a centered caption
    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = FA_PANEL_W - 14.f, cx = FA_PANEL_W / 2.f;
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
        const float W = FA_PANEL_W, H = FA_PANEL_H;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, W - 1.f, H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, W / 2.f, 13.f);
        drawText(vg, W / 2.f, 31.f, "FANCYADSR", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = FA_DISP_X - 5.f, by = FA_DISP_Y - 5.f;
        float bw = FA_DISP_W + 10.f, bh = FA_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- row A: envelope knobs with CV jacks ----
        section(vg, 152.f, "ENVELOPE");
        const char* names[4] = {"ATTACK", "DECAY", "SUSTAIN", "RELEASE"};
        for (int i = 0; i < 4; ++i) {
            float cx = faColX(i);
            plate(vg, cx - FA_CELL_W / 2.f, FA_ROWA_TOP, FA_CELL_W, FA_ROWA_H);
            label(vg, cx, FA_ROWA_TOP + 9.f, names[i]);
            ring(vg, cx, FA_ROWA_TOP + 34.f, 17.f);
            label(vg, cx + 22.f, FA_ROWA_TOP + 66.f, "CV");
        }

        // ---- row B: shapes, loop, time ----
        const char* b[4] = {"ATK SHAPE", "REL SHAPE", "LOOP", "TIME"};
        for (int i = 0; i < 4; ++i) {
            float cx = faColX(i);
            plate(vg, cx - FA_CELL_W / 2.f, FA_ROWB_TOP, FA_CELL_W, FA_ROWB_H);
            label(vg, cx, FA_ROWB_TOP + 47.f, b[i]);
        }
        ring(vg, faColX(3), FA_ROWB_TOP + 24.f, 16.f);

        // ---- row C: I/O ----
        const char* io[4] = {"GATE", "RETRIG", "ENV OUT", "EOR"};
        for (int i = 0; i < 4; ++i) {
            float cx = faColX(i);
            plate(vg, cx - FA_CELL_W / 2.f, FA_ROWC_TOP, FA_CELL_W, FA_ROWC_H);
            label(vg, cx, FA_ROWC_TOP + 8.f, io[i]);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Scope display: live envelope trace + ideal-curve preview
// ------------------------------------------------------------
struct FADisplayWidget : TransparentWidget {
    FancyADSR* module = nullptr;

    FADisplayWidget(FancyADSR* m) : module(m) {
        box.pos = Vec(FA_DISP_X, FA_DISP_Y);
        box.size = Vec(FA_DISP_W, FA_DISP_H);
    }

    static float vy(float v) {   // volts (0..10) to display y
        return FA_SY0 + FA_SH - 1.f - std::min(std::max(v / 10.f, 0.f), 1.f) * (FA_SH - 2.f);
    }

    // Static background: screen, scope field, grid
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float W = FA_DISP_W, H = FA_DISP_H;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H);
        nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 30.f, W * 0.7f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        // Scope field
        nvgBeginPath(vg);
        nvgRoundedRect(vg, FA_SX0 - 1.f, FA_SY0 - 1.f, FA_SW + 2.f, FA_SH + 2.f, 2.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Grid: 10 time divisions, 0 / 2.5 / 5 / 7.5 / 10 V
        nvgBeginPath(vg);
        for (int i = 1; i < 10; ++i) {
            float x = FA_SX0 + FA_SW * i / 10.f + 0.5f;
            nvgMoveTo(vg, x, FA_SY0);
            nvgLineTo(vg, x, FA_SY0 + FA_SH);
        }
        for (int k = 1; k < 4; ++k) {
            float y = FA_SY0 + FA_SH * k / 4.f + 0.5f;
            nvgMoveTo(vg, FA_SX0, y);
            nvgLineTo(vg, FA_SX0 + FA_SW, y);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Voltage ticks on the left edge
        nvgBeginPath(vg);
        for (int k = 0; k <= 4; ++k) {
            float y = FA_SY0 + FA_SH - k * (FA_SH - 1.f) / 4.f + 0.5f;
            nvgMoveTo(vg, FA_SX0 - 4.f, y);
            nvgLineTo(vg, FA_SX0 - 1.f, y);
        }
        nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 80));
        nvgStroke(vg);
        drawText(vg, 7.f, FA_SY0 + 2.f, "10", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        drawText(vg, 7.f, FA_SY0 + FA_SH - 2.f, "0", 5.5f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float W = FA_DISP_W, H = FA_DISP_H;

        // ---- gather state (live, or a sample scene in the module browser) ----
        float A, D, S, R, aShape, rShape, envV, timeScale;
        bool gate, loop;
        int stage;
        float eor;
        int n;
        float samples[FancyADSR::HISTORY_SIZE];

        if (module) {
            A = module->dispAttack; D = module->dispDecay; S = module->dispSustain; R = module->dispRelease;
            aShape = module->dispAttackShape; rShape = module->dispReleaseShape;
            envV = module->env * 10.f;
            timeScale = module->getTimeScale();
            gate = module->dispGate;
            loop = module->params[FancyADSR::LOOP_TOGGLE_PARAM].getValue() > 0.5f;
            stage = (int) module->stage;
            eor = module->eorFlash;
            n = std::min(std::max(module->getHistorySize(), 0), (int) FancyADSR::HISTORY_SIZE);
            for (int i = 0; i < n; ++i) samples[i] = module->getHistorySample(i);
        } else {
            A = 0.05f; D = 0.3f; S = 0.6f; R = 0.8f; aShape = 1.f; rShape = 1.f;
            timeScale = 2.f; gate = false; loop = false; stage = FancyADSR::STAGE_RELEASE; eor = 0.f;
            n = 300;
            float total = A + D + R + 0.25f * (A + D + R);
            for (int i = 0; i < n; ++i) {
                float t = total * (float) i / (float) n;
                float hold = A + D + 0.25f * (A + D + R);
                float v;
                if (t < A) v = segCurve(0.f, 1.f, A, t, aShape);
                else if (t < A + D) v = segCurve(1.f, S, D, t - A, 1.f);
                else if (t < hold) v = S;
                else v = segCurve(S, 0.f, R, t - hold, rShape);
                samples[i] = v * 10.f;
            }
            envV = samples[n - 1];
        }

        // ---- ideal ADSR preview (the shape the current knobs describe) ----
        {
            float hold = 0.25f * (A + D + R);
            float total = A + D + hold + R;
            const int N = 100;
            nvgBeginPath(vg);
            for (int i = 0; i <= N; ++i) {
                float t = total * (float) i / (float) N;
                float v;
                if (t < A) v = segCurve(0.f, 1.f, A, t, aShape);
                else if (t < A + D) v = segCurve(1.f, S, D, t - A, 1.f);
                else if (t < A + D + hold) v = S;
                else v = segCurve(S, 0.f, R, t - A - D - hold, rShape);
                float x = FA_SX0 + FA_SW * (float) i / (float) N;
                float y = vy(v * 10.f);
                if (i == 0) nvgMoveTo(vg, x, y);
                else nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, withAlpha(colViolet(), 0.55f));
            nvgStrokeWidth(vg, 1.f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
            drawText(vg, FA_SX0 + 3.f, FA_SY0 + 6.f, "PREVIEW", 5.5f, withAlpha(colViolet(), 0.7f),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
        }

        // ---- live trace ----
        nvgSave(vg);
        nvgScissor(vg, FA_SX0, FA_SY0, FA_SW, FA_SH);
        if (n >= 2) {
            // Fill under the trace
            nvgBeginPath(vg);
            nvgMoveTo(vg, FA_SX0, FA_SY0 + FA_SH);
            for (int i = 0; i < n; ++i) {
                float x = FA_SX0 + FA_SW * (float) i / (float) (FancyADSR::HISTORY_SIZE - 1);
                nvgLineTo(vg, x, vy(samples[i]));
            }
            nvgLineTo(vg, FA_SX0 + FA_SW * (float) (n - 1) / (float) (FancyADSR::HISTORY_SIZE - 1), FA_SY0 + FA_SH);
            nvgClosePath(vg);
            nvgFillPaint(vg, nvgLinearGradient(vg, 0, FA_SY0, 0, FA_SY0 + FA_SH,
                                               nvgRGBA(0x3d, 0xe0, 0xff, 70), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
            nvgFill(vg);

            for (int pass = 0; pass < 2; ++pass) {   // 0: glow, 1: crisp
                nvgBeginPath(vg);
                for (int i = 0; i < n; ++i) {
                    float x = FA_SX0 + FA_SW * (float) i / (float) (FancyADSR::HISTORY_SIZE - 1);
                    float y = vy(samples[i]);
                    if (i == 0) nvgMoveTo(vg, x, y);
                    else nvgLineTo(vg, x, y);
                }
                nvgLineJoin(vg, NVG_ROUND);
                if (pass == 0) {
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
                    nvgStrokeWidth(vg, 4.f);
                } else {
                    nvgStrokeColor(vg, nvgRGB(0x8a, 0xee, 0xff));
                    nvgStrokeWidth(vg, 1.4f);
                }
                nvgStroke(vg);
            }

            // Head dot
            float hx = FA_SX0 + FA_SW * (float) (n - 1) / (float) (FancyADSR::HISTORY_SIZE - 1);
            float hy = vy(samples[n - 1]);
            nvgBeginPath(vg);
            nvgCircle(vg, hx, hy, 4.f);
            nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, hx, hy, 1.8f);
            nvgFillColor(vg, nvgRGB(255, 255, 255));
            nvgFill(vg);
        }
        nvgResetScissor(vg);
        nvgRestore(vg);

        // ---- level meter ----
        {
            float lvl = std::min(std::max(envV / 10.f, 0.f), 1.f);
            const float mx = FA_SX0 + FA_SW + 8.f;
            nvgBeginPath(vg);
            nvgRect(vg, mx, FA_SY0, 6.f, FA_SH);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.005f) {
                float h = lvl * FA_SH;
                nvgBeginPath(vg);
                nvgRect(vg, mx, FA_SY0 + FA_SH - h, 6.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, FA_SY0 + FA_SH, 0, FA_SY0,
                                                   nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }

        // ---- stage / state chips ----
        auto chip = [&](float x, float w, const char* txt, bool on, NVGcolor col) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, 77.f, w, 11.f, 3.f);
            if (on) {
                nvgFillColor(vg, withAlpha(col, 0.25f));
                nvgFill(vg);
                nvgStrokeColor(vg, withAlpha(col, 0.9f));
            } else {
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            }
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, x + w / 2.f, 82.5f, txt, 6.f, on ? col : colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        };
        chip(16.f,  32.f, "ATK", stage == FancyADSR::STAGE_ATTACK,  colAccent());
        chip(52.f,  32.f, "DEC", stage == FancyADSR::STAGE_DECAY,   colAccent());
        chip(88.f,  32.f, "SUS", stage == FancyADSR::STAGE_SUSTAIN, colAccent());
        chip(124.f, 32.f, "REL", stage == FancyADSR::STAGE_RELEASE, colViolet());
        chip(164.f, 32.f, "GATE", gate, colAccent());
        chip(200.f, 32.f, "LOOP", loop, colViolet());

        // EOR flash around the REL chip
        if (eor > 0.02f) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, 124.f - 1.5f, 77.f - 1.5f, 35.f, 14.f, 4.f);
            nvgStrokeColor(vg, withAlpha(colViolet(), 0.9f * eor));
            nvgStrokeWidth(vg, 1.5f);
            nvgStroke(vg);
        }

        // ---- scanlines ----
        nvgBeginPath(vg);
        for (int y = 0; y < (int) H; y += 3)
            nvgRect(vg, 0, y + 1, W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
        nvgFill(vg);

        // ---- glass reflection ----
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H * 0.3f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f,
                                           nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // ---- readouts ----
        char buf[64];
        snprintf(buf, sizeof(buf), "A %.2f  D %.2f  S %.0f%%  R %.2f", A, D, S * 100.f, R);
        drawText(vg, 6.f, 8.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 200),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
        snprintf(buf, sizeof(buf), "%.1fV", envV);
        drawText(vg, W - 6.f, 8.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
        snprintf(buf, sizeof(buf), "%.2fS", timeScale);
        drawText(vg, FA_SX0 + FA_SW - 3.f, FA_SY0 + FA_SH - 6.f, buf, 6.f, colDim(),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module Widget
// ------------------------------------------------------------
struct FancyADSRWidget : ModuleWidget
{
    FancyADSRWidget(FancyADSR *module)
    {
        setModule(module);
        box.size = Vec(FA_PANEL_W, FA_PANEL_H);

        addChild(new FAPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new FADisplayWidget(module));

        // ---- row A: knob on top, CV jack below ----
        const int knobIds[4] = {FancyADSR::ATTACK_PARAM, FancyADSR::DECAY_PARAM,
                                FancyADSR::SUSTAIN_PARAM, FancyADSR::RELEASE_PARAM};
        const int cvIds[4] = {FancyADSR::ATTACK_CV_INPUT, FancyADSR::DECAY_CV_INPUT,
                              FancyADSR::SUSTAIN_CV_INPUT, FancyADSR::RELEASE_CV_INPUT};
        for (int i = 0; i < 4; ++i) {
            float cx = faColX(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, FA_ROWA_TOP + 34.f), module, knobIds[i]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 4.f, FA_ROWA_TOP + 66.f), module, cvIds[i]));
        }

        // ---- row B: shapes, loop, time ----
        const float by = FA_ROWB_TOP + 24.f;
        addParam(createParamCentered<CKSS>(Vec(faColX(0), by), module, FancyADSR::ATTACK_SHAPE_PARAM));
        addParam(createParamCentered<CKSS>(Vec(faColX(1), by), module, FancyADSR::RELEASE_SHAPE_PARAM));
        addParam(createParamCentered<CKSS>(Vec(faColX(2) - 6.f, by), module, FancyADSR::LOOP_TOGGLE_PARAM));
        addChild(createLightCentered<SmallLight<FACyanLight>>(Vec(faColX(2) + 12.f, by), module, FancyADSR::LOOP_LIGHT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(faColX(3), by), module, FancyADSR::TIME_SCALE_PARAM));

        // ---- row C: I/O ----
        const float cy = FA_ROWC_TOP + 28.f;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(faColX(0), cy), module, FancyADSR::GATE_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(faColX(1), cy), module, FancyADSR::TRIGGER_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(faColX(2), cy), module, FancyADSR::ENV_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(faColX(3), cy), module, FancyADSR::EOR_OUTPUT));
    }
};

Model *modelFancyADSR = createModel<FancyADSR, FancyADSRWidget>("FancyADSR");