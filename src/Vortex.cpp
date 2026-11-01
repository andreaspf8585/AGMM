#include "VortexModule.hpp"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

static const int VX_SCOPE_N = 96;

// ============================================================
// Module – DSP preserved, CV range option + scope tracking
// ============================================================
struct VortexModule : Module {
    enum ParamId {
        PRE_DELAY_PARAM, PRE_DELAY_ATTEN,
        SIZE_PARAM, SIZE_ATTEN,
        DECAY_PARAM, DECAY_ATTEN,
        DIFFUSION_PARAM, DIFFUSION_ATTEN,
        HIGH_DAMP_PARAM, HIGH_DAMP_ATTEN,
        LOW_CUT_PARAM, LOW_CUT_ATTEN,
        MOD_SPEED_PARAM, MOD_SPEED_ATTEN,
        MOD_DEPTH_PARAM, MOD_DEPTH_ATTEN,
        MIX_PARAM, MIX_ATTEN,
        ECHO_MODE_BUTTON_PARAM,
        ECHO_TIME_PARAM, ECHO_TIME_ATTEN,
        ECHO_FEEDBACK_PARAM, ECHO_FEEDBACK_ATTEN,
        ECHO_LEVEL_PARAM, ECHO_LEVEL_ATTEN,
        FREEZE_BUTTON_PARAM,
        IN_STEREO_BUTTON_PARAM, OUT_STEREO_BUTTON_PARAM,
        CLEAR_BUTTON_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        IN_L_INPUT, IN_R_INPUT,
        PRE_DELAY_CV_INPUT, SIZE_CV_INPUT, DECAY_CV_INPUT,
        DIFFUSION_CV_INPUT, HIGH_DAMP_CV_INPUT, LOW_CUT_CV_INPUT,
        MOD_SPEED_CV_INPUT, MOD_DEPTH_CV_INPUT, MIX_CV_INPUT,
        ECHO_TIME_CV_INPUT, ECHO_FEEDBACK_CV_INPUT, ECHO_LEVEL_CV_INPUT,
        FREEZE_CV_INPUT,
        CLEAR_INPUT,
        INPUTS_LEN
    };
    enum OutputId { OUT_L_OUTPUT, OUT_R_OUTPUT, OUTPUTS_LEN };
    enum LightId {
        CLEAR_BUTTON_LIGHT, FREEZE_BUTTON_LIGHT, ECHO_MODE_BUTTON_LIGHT,
        LIGHTS_LEN
    };

    enum CVRange { BIPOLAR_M5_5, UNIPOLAR_0_10 };
    CVRange cvRangeMode = UNIPOLAR_0_10;

    // ---- display state ----
    float scopeMin[VX_SCOPE_N] = {};
    float scopeMax[VX_SCOPE_N] = {};
    int   scopeHead = 0;
    int   scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    Vortex::DattorroEngine engines[Vortex::NUM_CHANNELS];
    dsp::SchmittTrigger clearButtonTrigger, clearInputTrigger;
    dsp::SchmittTrigger freezeButtonTrigger, freezeInputTrigger;
    dsp::SchmittTrigger echoModeButtonTrigger;
    bool clearRequested = false;
    float clearFade = 1.0f;
    bool clearActive = false;
    int limiterRecovery = 0;
    bool stereoInput = false, stereoOutput = false;
    bool freezeState = false;
    bool echoModeState = false;

    VortexModule() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(PRE_DELAY_PARAM, 0.f, 1.f, 0.f, "Pre‑Delay", "Pre‑delay time (0…500 ms)");
        configParam(PRE_DELAY_ATTEN, -1.f, 1.f, 0.f, "Pre‑Delay CV Atten");
        configParam(SIZE_PARAM, 0.f, 1.f, 0.6f, "Size", "Reverb size / delay times");
        configParam(SIZE_ATTEN, -1.f, 1.f, 0.f, "Size CV Atten");
        configParam(DECAY_PARAM, 0.f, 1.f, 0.5f, "Decay", "Reverb decay time");
        configParam(DECAY_ATTEN, -1.f, 1.f, 0.f, "Decay CV Atten");
        configParam(DIFFUSION_PARAM, 0.f, 1.f, 0.7f, "Diffusion", "Early reflection density");
        configParam(DIFFUSION_ATTEN, -1.f, 1.f, 0.f, "Diffusion CV Atten");
        configParam(HIGH_DAMP_PARAM, 0.f, 1.f, 0.5f, "High Damp", "High‑frequency damping");
        configParam(HIGH_DAMP_ATTEN, -1.f, 1.f, 0.f, "High Damp CV Atten");
        configParam(LOW_CUT_PARAM, 0.f, 1.f, 0.f, "Low Cut", "Low‑cut frequency (20…500 Hz)");
        configParam(LOW_CUT_ATTEN, -1.f, 1.f, 0.f, "Low Cut CV Atten");
        configParam(MOD_SPEED_PARAM, 0.f, 1.f, 0.3f, "Mod Speed", "Modulation LFO speed (0.1…5 Hz)");
        configParam(MOD_SPEED_ATTEN, -1.f, 1.f, 0.f, "Mod Speed CV Atten");
        configParam(MOD_DEPTH_PARAM, 0.f, 1.f, 0.2f, "Mod Depth", "Modulation depth (0…10 ms)");
        configParam(MOD_DEPTH_ATTEN, -1.f, 1.f, 0.f, "Mod Depth CV Atten");
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/Wet", "Balance dry and processed");
        configParam(MIX_ATTEN, -1.f, 1.f, 0.f, "Mix CV Atten");

        configParam(ECHO_TIME_PARAM, 0.f, 1.f, 0.3f, "Echo Time", "Echo delay time (0…1000 ms)");
        configParam(ECHO_TIME_ATTEN, -1.f, 1.f, 0.f, "Echo Time CV Atten");
        configParam(ECHO_FEEDBACK_PARAM, 0.f, 1.f, 0.5f, "Echo Feedback", "Echo feedback (0…0.95)");
        configParam(ECHO_FEEDBACK_ATTEN, -1.f, 1.f, 0.f, "Echo Feedback CV Atten");
        configParam(ECHO_LEVEL_PARAM, 0.f, 1.f, 0.8f, "Echo Level", "Echo level (0…1)");
        configParam(ECHO_LEVEL_ATTEN, -1.f, 1.f, 0.f, "Echo Level CV Atten");

        configParam(ECHO_MODE_BUTTON_PARAM, 0.f, 1.f, 0.f, "Mode", "Toggle Reverb / Echo");
        configParam(FREEZE_BUTTON_PARAM, 0.f, 1.f, 0.f, "Freeze", "Hold reverb tail (reverb mode only)");
        configParam(IN_STEREO_BUTTON_PARAM, 0.f, 1.f, 0.f, "Stereo input");
        configParam(OUT_STEREO_BUTTON_PARAM, 0.f, 1.f, 0.f, "Stereo output");
        configParam(CLEAR_BUTTON_PARAM, 0.f, 1.f, 0.f, "Clear", "Reset reverb/echo tails");

        configInput(IN_L_INPUT, "Left / Poly");
        configInput(IN_R_INPUT, "Right");
        configInput(PRE_DELAY_CV_INPUT, "Pre‑Delay CV");
        configInput(SIZE_CV_INPUT, "Size CV");
        configInput(DECAY_CV_INPUT, "Decay CV");
        configInput(DIFFUSION_CV_INPUT, "Diffusion CV");
        configInput(HIGH_DAMP_CV_INPUT, "High Damp CV");
        configInput(LOW_CUT_CV_INPUT, "Low Cut CV");
        configInput(MOD_SPEED_CV_INPUT, "Mod Speed CV");
        configInput(MOD_DEPTH_CV_INPUT, "Mod Depth CV");
        configInput(MIX_CV_INPUT, "Mix CV");
        configInput(ECHO_TIME_CV_INPUT, "Echo Time CV");
        configInput(ECHO_FEEDBACK_CV_INPUT, "Echo Feedback CV");
        configInput(ECHO_LEVEL_CV_INPUT, "Echo Level CV");
        configInput(FREEZE_CV_INPUT, "Freeze gate");
        configInput(CLEAR_INPUT, "Clear gate");

        configOutput(OUT_L_OUTPUT, "Left / Poly");
        configOutput(OUT_R_OUTPUT, "Right");

        configLight(CLEAR_BUTTON_LIGHT, "Clear status");
        configLight(FREEZE_BUTTON_LIGHT, "Freeze status");
        configLight(ECHO_MODE_BUTTON_LIGHT, "Echo mode active");

        onReset();
    }

    void onReset() override {
        for (auto& eng : engines) eng.reset();
        clearRequested = false;
        clearFade = 1.0f;
        clearActive = false;
        limiterRecovery = 0;
        stereoInput = stereoOutput = false;
        freezeState = false;
        echoModeState = false;
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        return root;
    }
    void dataFromJson(json_t* root) override {
        json_t* mode = json_object_get(root, "cvRangeMode");
        if (mode) cvRangeMode = (CVRange)json_integer_value(mode);
    }

    float getParam(int paramId, int attenId, int cvId) {
        float knob = params[paramId].getValue();
        float atten = params[attenId].getValue();
        float cvRaw = inputs[cvId].getVoltageSum();
        float cv;
        if (cvRangeMode == BIPOLAR_M5_5)
            cv = rack::math::clamp(cvRaw / 5.f, -1.f, 1.f);
        else
            cv = cvRaw / 10.f;
        float value = knob + atten * cv;
        return rack::math::clamp(value, 0.0f, 1.0f);
    }

    void process(const ProcessArgs& args) override {
        float sampleRate = args.sampleRate;

        int inLch = inputs[IN_L_INPUT].getChannels();
        int inRch = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getChannels() : 0;
        bool useStereoIn = params[IN_STEREO_BUTTON_PARAM].getValue() > 0.5f;
        int numChannels = useStereoIn ? 2 : std::max(1, std::max(inLch, inRch));
        numChannels = rack::math::clamp(numChannels, 1, Vortex::NUM_CHANNELS);
        bool useStereoOut = params[OUT_STEREO_BUTTON_PARAM].getValue() > 0.5f;
        outputs[OUT_L_OUTPUT].setChannels(useStereoOut ? 2 : numChannels);
        outputs[OUT_R_OUTPUT].setChannels(useStereoOut ? 2 : numChannels);

        float preDelayKnob = getParam(PRE_DELAY_PARAM, PRE_DELAY_ATTEN, PRE_DELAY_CV_INPUT);
        float sizeKnob     = getParam(SIZE_PARAM, SIZE_ATTEN, SIZE_CV_INPUT);
        float decayKnob    = getParam(DECAY_PARAM, DECAY_ATTEN, DECAY_CV_INPUT);
        float diffusionKnob= getParam(DIFFUSION_PARAM, DIFFUSION_ATTEN, DIFFUSION_CV_INPUT);
        float highDampKnob = getParam(HIGH_DAMP_PARAM, HIGH_DAMP_ATTEN, HIGH_DAMP_CV_INPUT);
        float lowCutKnob   = getParam(LOW_CUT_PARAM, LOW_CUT_ATTEN, LOW_CUT_CV_INPUT);
        float modSpeedKnob = getParam(MOD_SPEED_PARAM, MOD_SPEED_ATTEN, MOD_SPEED_CV_INPUT);
        float modDepthKnob = getParam(MOD_DEPTH_PARAM, MOD_DEPTH_ATTEN, MOD_DEPTH_CV_INPUT);
        float mixKnob      = getParam(MIX_PARAM, MIX_ATTEN, MIX_CV_INPUT);

        float echoTimeKnob = getParam(ECHO_TIME_PARAM, ECHO_TIME_ATTEN, ECHO_TIME_CV_INPUT);
        float echoFbKnob   = getParam(ECHO_FEEDBACK_PARAM, ECHO_FEEDBACK_ATTEN, ECHO_FEEDBACK_CV_INPUT);
        float echoLevelKnob= getParam(ECHO_LEVEL_PARAM, ECHO_LEVEL_ATTEN, ECHO_LEVEL_CV_INPUT);

        bool echoModePressed = params[ECHO_MODE_BUTTON_PARAM].getValue() > 0.5f;
        if (echoModeButtonTrigger.process(echoModePressed))
            echoModeState = !echoModeState;
        lights[ECHO_MODE_BUTTON_LIGHT].setSmoothBrightness(echoModeState ? 1.0f : 0.0f, args.sampleTime);

        bool freezePressed = params[FREEZE_BUTTON_PARAM].getValue() > 0.5f;
        bool freezeTrig = freezeButtonTrigger.process(freezePressed) ||
                          freezeInputTrigger.process(inputs[FREEZE_CV_INPUT].getVoltage());
        if (freezeTrig) freezeState = !freezeState;
        lights[FREEZE_BUTTON_LIGHT].setSmoothBrightness(freezeState ? 1.0f : 0.0f, args.sampleTime);

        float preDelayMs = preDelayKnob * 500.0f;
        float size = sizeKnob;
        float decay = decayKnob;
        float diffusion = diffusionKnob;
        float highDamp = highDampKnob;
        float lowCutFreq = 20.0f + lowCutKnob * 480.0f;
        float modSpeed = 0.1f + modSpeedKnob * 4.9f;
        float modDepthMs = modDepthKnob * 10.0f;
        float dryWet = mixKnob;
        float echoTimeMs = echoTimeKnob * 1000.0f;
        float echoFeedback = echoFbKnob * 0.95f;
        float echoLevel = echoLevelKnob;

        static float lastSize = -1.f;
        if (std::abs(size - lastSize) > 0.01f && !echoModeState) {
            for (auto& eng : engines) {
                eng.size = size;
                eng.resizeDelays(sampleRate);
            }
            lastSize = size;
        }

        bool clearPressed = params[CLEAR_BUTTON_PARAM].getValue() > 0.5f;
        bool clearTrig = clearButtonTrigger.process(clearPressed) ||
                         clearInputTrigger.process(inputs[CLEAR_INPUT].getVoltage());
        if (clearTrig) clearRequested = true;

        if (clearRequested) {
            clearFade -= sampleRate * 0.00001f;
            if (clearFade <= 0.0f) {
                for (auto& eng : engines) eng.reset();
                clearFade = 0.0f;
                clearRequested = false;
                clearActive = true;
            }
        } else if (clearActive) {
            clearFade += sampleRate * 0.00001f;
            if (clearFade >= 1.0f) {
                clearFade = 1.0f;
                clearActive = false;
            }
        } else {
            clearFade = 1.0f;
        }
        lights[CLEAR_BUTTON_LIGHT].setSmoothBrightness(clearPressed ? 1.0f : 0.0f, args.sampleTime);

        for (int c = 0; c < numChannels; ++c) {
            float dryL = inputs[IN_L_INPUT].getPolyVoltage(c);
            float dryR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getPolyVoltage(c) : dryL;
            if (!std::isfinite(dryL)) dryL = 0.f;
            if (!std::isfinite(dryR)) dryR = 0.f;

            auto& eng = engines[c];
            eng.preDelayMs = preDelayMs;
            eng.size = size;
            eng.decay = decay;
            eng.diffusion = diffusion;
            eng.highDamp = highDamp;
            eng.lowCutFreq = lowCutFreq;
            eng.modSpeed = modSpeed;
            eng.modDepth = modDepthMs;
            eng.dryWet = dryWet;
            eng.freeze = freezeState;
            eng.echoMode = echoModeState;
            eng.echoTimeMs = echoTimeMs;
            eng.echoFeedback = echoFeedback;
            eng.echoLevel = echoLevel;

            float wetL, wetR;
            eng.process(sampleRate, dryL * clearFade, dryR * clearFade, wetL, wetR);
            wetL *= clearFade;
            wetR *= clearFade;

            if (!std::isfinite(wetL) || !std::isfinite(wetR)) {
                wetL = wetR = 0.f;
                if (limiterRecovery == 0) {
                    for (auto& eng2 : engines) eng2.reset();
                    limiterRecovery = (int)(sampleRate * 0.1f);
                }
            }
            if (limiterRecovery > 0) {
                wetL = wetR = 0.f;
                limiterRecovery--;
            }

            if (useStereoOut) {
                if (c == 0) {
                    outputs[OUT_L_OUTPUT].setVoltage(wetL, 0);
                    outputs[OUT_R_OUTPUT].setVoltage(wetR, 0);
                } else if (c == 1) {
                    outputs[OUT_L_OUTPUT].setVoltage(wetR, 1);
                    outputs[OUT_R_OUTPUT].setVoltage(wetL, 1);
                }
            } else {
                outputs[OUT_L_OUTPUT].setVoltage(wetL, c);
                outputs[OUT_R_OUTPUT].setVoltage(wetR, c);
            }
        }

        // ---- display tracking (channel 0) ----
        float vL = outputs[OUT_L_OUTPUT].getVoltage(0);
        float vR = outputs[OUT_R_OUTPUT].getVoltage(0);
        float decayRate = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(vL), peakL * decayRate);
        peakR = std::max(std::fabs(vR), peakR * decayRate);

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
            scopeHead = (scopeHead + 1) % VX_SCOPE_N;
        }
    }
};

// ============================================================
// Layout constants (32 HP = 480 px)
// ============================================================
static const float VX_PANEL_W = RACK_GRID_WIDTH * 32;   // 480
static const float VX_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float VX_DISP_W = 380.f;
static const float VX_DISP_H = 34.f;
static const float VX_DISP_X = (VX_PANEL_W - VX_DISP_W) / 2.f;   // 50
static const float VX_DISP_Y = 44.f;

// 6 columns for knob cells
static const float VX_COL_X[6] = {40.f, 120.f, 200.f, 280.f, 360.f, 440.f};
static const float VX_ROW_Y[2] = {120.f, 195.f};

// 5 button x positions
static const float VX_BTN_X[5] = {48.f, 144.f, 240.f, 336.f, 432.f};
static const float VX_BTN_Y = 278.f;

// 6 I/O jack x positions (same as cell columns)
static const float VX_IO_Y = 340.f;

// ============================================================
// Theme + helpers (anonymous namespace)
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

// Cell table – 12 knobs with CV + atten
struct VxCellDef { const char* name; int param, atten, cv; };
const VxCellDef VX_CELLS[12] = {
    {"PRE_DLY", VortexModule::PRE_DELAY_PARAM,  VortexModule::PRE_DELAY_ATTEN,  VortexModule::PRE_DELAY_CV_INPUT},
    {"SIZE",    VortexModule::SIZE_PARAM,       VortexModule::SIZE_ATTEN,       VortexModule::SIZE_CV_INPUT},
    {"DECAY",   VortexModule::DECAY_PARAM,      VortexModule::DECAY_ATTEN,      VortexModule::DECAY_CV_INPUT},
    {"DIFF",    VortexModule::DIFFUSION_PARAM,  VortexModule::DIFFUSION_ATTEN,  VortexModule::DIFFUSION_CV_INPUT},
    {"DAMP",    VortexModule::HIGH_DAMP_PARAM,  VortexModule::HIGH_DAMP_ATTEN,  VortexModule::HIGH_DAMP_CV_INPUT},
    {"L-CUT",   VortexModule::LOW_CUT_PARAM,    VortexModule::LOW_CUT_ATTEN,    VortexModule::LOW_CUT_CV_INPUT},
    {"SPEED",   VortexModule::MOD_SPEED_PARAM,  VortexModule::MOD_SPEED_ATTEN,  VortexModule::MOD_SPEED_CV_INPUT},
    {"DEPTH",   VortexModule::MOD_DEPTH_PARAM,  VortexModule::MOD_DEPTH_ATTEN,  VortexModule::MOD_DEPTH_CV_INPUT},
    {"MIX",     VortexModule::MIX_PARAM,        VortexModule::MIX_ATTEN,        VortexModule::MIX_CV_INPUT},
    {"E-TIME",  VortexModule::ECHO_TIME_PARAM,  VortexModule::ECHO_TIME_ATTEN,  VortexModule::ECHO_TIME_CV_INPUT},
    {"E-FB",    VortexModule::ECHO_FEEDBACK_PARAM, VortexModule::ECHO_FEEDBACK_ATTEN, VortexModule::ECHO_FEEDBACK_CV_INPUT},
    {"E-LVL",   VortexModule::ECHO_LEVEL_PARAM, VortexModule::ECHO_LEVEL_ATTEN, VortexModule::ECHO_LEVEL_CV_INPUT},
};

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct VxPanelBackground : TransparentWidget {
    VortexModule* module = nullptr;
    VxPanelBackground(VortexModule* m) : module(m) {
        box.size = Vec(VX_PANEL_W, VX_PANEL_H);
    }

    void plate(NVGcontext* vg, float cx, float ry) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - 27.f, ry - 17.f, 54.f, 70.f, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = VX_PANEL_W - 14.f, cx = VX_PANEL_W / 2.f;
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
        nvgRect(vg, 0, 0, VX_PANEL_W, VX_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, VX_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) VX_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, VX_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, VX_PANEL_W - 1.f, VX_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, VX_PANEL_W / 2.f, 13.f);
        drawText(vg, VX_PANEL_W / 2.f, 31.f, "VORTEX", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = VX_DISP_X - 5.f, by = VX_DISP_Y - 5.f;
        float bw = VX_DISP_W + 10.f, bh = VX_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- PARAMETERS section (12 cells, 6×2) ----
        section(vg, 88.f, "PARAMETERS");
        for (int i = 0; i < 12; ++i) {
            int row = i / 6, col = i % 6;
            float cx = VX_COL_X[col], ry = VX_ROW_Y[row];
            plate(vg, cx, ry);
            drawText(vg, cx, ry + 20.f, VX_CELLS[i].name, 7.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.3f);
        }

        // ---- MODE section (buttons + labels) ----
        section(vg, 252.f, "MODE / CONTROL");
        const char* btnLabels[5] = {"REV/ECHO", "FREEZE", "CLEAR", "IN STEREO", "OUT STEREO"};
        for (int i = 0; i < 5; ++i) {
            NVGcolor c = colDim();
            // Stereo buttons: brighten when on
            if (i >= 3 && module) {
                int pid = (i == 3) ? VortexModule::IN_STEREO_BUTTON_PARAM
                                   : VortexModule::OUT_STEREO_BUTTON_PARAM;
                if (module->params[pid].getValue() > 0.5f) c = colAccent();
            }
            drawText(vg, VX_BTN_X[i], VX_BTN_Y + 18.f, btnLabels[i], 6.f, c,
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.3f);
        }

        // ---- I / O section (6 jacks) ----
        section(vg, 305.f, "I / O");
        const char* io[6] = {"L IN", "R IN", "L OUT", "R OUT", "FREEZE", "CLEAR"};
        for (int i = 0; i < 6; ++i) {
            float cx = VX_COL_X[i];
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 22.f, VX_IO_Y - 22.f, 44.f, 42.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, cx, VX_IO_Y - 15.f, io[i], 7.5f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Display widget – scope + meter + mode readout
// ------------------------------------------------------------
struct VxDisplayWidget : TransparentWidget {
    VortexModule* module = nullptr;

    VxDisplayWidget(VortexModule* m) : module(m) {
        box.pos = Vec(VX_DISP_X, VX_DISP_Y);
        box.size = Vec(VX_DISP_W, VX_DISP_H);
    }

    static float scopeW() { return VX_DISP_W - 16.f; }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, VX_DISP_W, VX_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, VX_DISP_W / 2.f, VX_DISP_H / 2.f, 20.f,
                                           VX_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        nvgBeginPath(vg);
        for (float x = 0.f; x <= scopeW(); x += 28.f) {
            nvgMoveTo(vg, x + 0.5f, 0);
            nvgLineTo(vg, x + 0.5f, VX_DISP_H);
        }
        nvgMoveTo(vg, 0, VX_DISP_H / 2.f + 0.5f);
        nvgLineTo(vg, scopeW(), VX_DISP_H / 2.f + 0.5f);
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

        const float mid  = VX_DISP_H / 2.f;
        const float amp  = mid - 2.f;
        const float sw   = scopeW();
        const float colW = sw / (float) VX_SCOPE_N;
        const float range = 10.f;

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, VX_DISP_H);
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < VX_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % VX_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / VX_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - std::min(std::max(vmax / range, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / range, -1.f), 1.f) * amp;
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
            float lvl = std::min(peak / range, 1.f);
            float x = sw + 3.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 1.f, 4.f, VX_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (VX_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, VX_DISP_H - 1.f - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, VX_DISP_H, 0, 0,
                                                   colAccent(), colAccent2()));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) VX_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, VX_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, VX_DISP_W, VX_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, VX_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16),
                                           nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[48];
        bool echoMode = module ? module->echoModeState : false;
        const char* modeStr = echoMode ? "ECHO" : "REVERB";
        NVGcolor modeCol = echoMode ? colAccent2() : colAccent();
        drawText(vg, 4.f, 7.f, modeStr, 7.f, modeCol,
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

        float mx = module ? module->params[VortexModule::MIX_PARAM].getValue() : 0.5f;
        snprintf(buf, sizeof(buf), "MIX %3.0f%%", mx * 100.f);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        // Bottom row: current pitch-ish info
        float sz = module ? module->params[VortexModule::SIZE_PARAM].getValue() : 0.6f;
        float dc = module ? module->params[VortexModule::DECAY_PARAM].getValue() : 0.5f;
        snprintf(buf, sizeof(buf), "SIZE %3.0f%%   DECAY %3.0f%%", sz * 100.f, dc * 100.f);
        drawText(vg, sw / 2.f, VX_DISP_H - 5.f, buf, 6.f,
                 nvgRGBA(0x7a, 0x85, 0x94, 200),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ============================================================
// Module widget
// ============================================================
struct VortexWidget : ModuleWidget {
    VortexWidget(VortexModule* module) {
        setModule(module);
        box.size = Vec(VX_PANEL_W, VX_PANEL_H);

        addChild(new VxPanelBackground(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new VxDisplayWidget(module));

        // 12 knob cells (6×2)
        for (int i = 0; i < 12; ++i) {
            int row = i / 6, col = i % 6;
            float cx = VX_COL_X[col], ry = VX_ROW_Y[row];
            const VxCellDef& c = VX_CELLS[i];
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, ry), module, c.param));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 14.f, ry + 35.f), module, c.cv));
            addParam(createParamCentered<Trimpot>(Vec(cx + 14.f, ry + 35.f), module, c.atten));
        }

        // 5 buttons: ECHO_MODE, FREEZE, CLEAR, IN_STEREO, OUT_STEREO
        addParam(createParamCentered<LEDButton>(Vec(VX_BTN_X[0], VX_BTN_Y), module,
            VortexModule::ECHO_MODE_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(VX_BTN_X[0], VX_BTN_Y), module,
            VortexModule::ECHO_MODE_BUTTON_LIGHT));

        addParam(createParamCentered<LEDButton>(Vec(VX_BTN_X[1], VX_BTN_Y), module,
            VortexModule::FREEZE_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(VX_BTN_X[1], VX_BTN_Y), module,
            VortexModule::FREEZE_BUTTON_LIGHT));

        addParam(createParamCentered<LEDButton>(Vec(VX_BTN_X[2], VX_BTN_Y), module,
            VortexModule::CLEAR_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(VX_BTN_X[2], VX_BTN_Y), module,
            VortexModule::CLEAR_BUTTON_LIGHT));

        addParam(createParamCentered<LEDButton>(Vec(VX_BTN_X[3], VX_BTN_Y), module,
            VortexModule::IN_STEREO_BUTTON_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(VX_BTN_X[4], VX_BTN_Y), module,
            VortexModule::OUT_STEREO_BUTTON_PARAM));

        // 6 I/O jacks: L IN, R IN, L OUT, R OUT, FREEZE gate, CLEAR gate
        addInput(createInputCentered<DarkPJ301MPort>(Vec(VX_COL_X[0], VX_IO_Y), module,
            VortexModule::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(VX_COL_X[1], VX_IO_Y), module,
            VortexModule::IN_R_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(VX_COL_X[2], VX_IO_Y), module,
            VortexModule::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(VX_COL_X[3], VX_IO_Y), module,
            VortexModule::OUT_R_OUTPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(VX_COL_X[4], VX_IO_Y), module,
            VortexModule::FREEZE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(VX_COL_X[5], VX_IO_Y), module,
            VortexModule::CLEAR_INPUT));
    }

    void appendContextMenu(Menu* menu) override {
        VortexModule* module = dynamic_cast<VortexModule*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem {
            VortexModule* module;
            VortexModule::CVRange range;
            void onAction(const ActionEvent& e) override {
                module->cvRangeMode = range;
            }
            void step() override {
                rightText = (module->cvRangeMode == range) ? "✔" : "";
                MenuItem::step();
            }
        };

        CVRangeItem* unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar, default)");
        unipolarItem->module = module;
        unipolarItem->range = VortexModule::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);

        CVRangeItem* bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = VortexModule::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);
    }
};

Model* modelVortex = createModel<VortexModule, VortexWidget>("Vortex");