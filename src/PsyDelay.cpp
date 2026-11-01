#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <random>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   PD_BINS = 130;                          // display overview (2 px per bin)
static const float PD_PANEL_W = RACK_GRID_WIDTH * 36;      // 540
static const float PD_PANEL_H = RACK_GRID_HEIGHT;          // 380

static const float PD_DISP_X = 124.f;
static const float PD_DISP_Y = 46.f;
static const float PD_DISP_W = 292.f;
static const float PD_DISP_H = 130.f;

static const float PD_LCOL[2] = {30.f, 90.f};
static const float PD_RCOL[2] = {450.f, 510.f};
static const float PD_KROW[3] = {96.f, 156.f, 216.f};
static const float PD_SW_Y = 222.f;
static const float PD_JACK_Y = 318.f;
static float pdCenterX(int i) { return 148.f + 48.8f * i; }   // 6 centre columns

// ------------------------------------------------------------
// Soft limiter for the feedback path (keeps self-oscillation from blowing up)
// ------------------------------------------------------------
static inline float pdSoftLimit(float x) {
    if (!std::isfinite(x)) return 0.f;
    float a = std::fabs(x);
    if (a <= 8.f) return x;
    float y = 8.f + 4.f * std::tanh((a - 8.f) / 4.f);
    return x < 0.f ? -y : y;
}

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct PsyDelay : Module {
    enum ParamId {
        TIME_PARAM,
        FEEDBACK_PARAM,
        MIX_PARAM,
        COLOR_PARAM,
        DRIVE_PARAM,
        MOD_DEPTH_PARAM,
        REVERSE_SWITCH,
        CHAOS_SWITCH,
        PINGPONG_SWITCH,
        BITCRUSH_SWITCH,
        FREEZE_SWITCH,
        PHASE_FLIP_SWITCH,
        HOWL_PARAM,
        RING_MOD_PARAM,
        GLITCH_PARAM,
        GATE_THRESH_PARAM,
        SRR_PARAM,
        CROSS_MOD_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        LEFT_INPUT,
        RIGHT_INPUT,
        TIME_CV_INPUT,
        FREEZE_CV_INPUT,
        HOWL_CV_INPUT,
        RING_CV_INPUT,
        GLITCH_CV_INPUT,
        GATE_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        FREEZE_LIGHT,
        BITCRUSH_LIGHT,
        PINGPONG_LIGHT,
        REVERSE_LIGHT,
        CHAOS_LIGHT,
        PHASE_FLIP_LIGHT,
        LIGHTS_LEN
    };

    // Delay buffers
    std::vector<float> delayL, delayR;
    int writePos = 0;
    float sampleRate = 44100.0f;
    size_t bufferSize = 0;

    // Output filter state (for COLOR)
    float outFilterL = 0.0f, outFilterR = 0.0f;

    // LFO / Chaos
    float lfoPhase = 0.0f;
    float chaosState = 0.5f;

    // Ring modulator phase
    float ringPhase = 0.0f;

    // Glitch RNG
    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist{0.0f, 1.0f};

    // Sample rate reduction state
    float srrCounter = 0.0f;
    float lastWetL = 0.0f, lastWetR = 0.0f;

    // Glitch smoothing
    float glitchFade = 0.0f;
    float glitchOldL = 0.0f, glitchOldR = 0.0f;

    // Input gate
    float gateEnv = 0.f;
    float gateGain = 1.f;

    // Flag to initialise output filter
    bool filterInitialised = false;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float peakBins[PD_BINS] = {};
    int peakWriteBin = 0;
    int lastBin = -1;
    float dispReadAge = 0.2f;      // 0 = "now", 1 = oldest sample
    float dispMod = 0.f;
    float glitchFlash = 0.f;
    float meterL = 0.f, meterR = 0.f;
    bool dispGateActive = false, dispGateOpen = true;

    PsyDelay() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(TIME_PARAM, 0.f, 1.f, 0.5f, "Delay time", " s", 0.f, 1.99f, 0.01f);
        configParam(FEEDBACK_PARAM, 0.f, 1.2f, 0.8f, "Feedback", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/wet", "%", 0.f, 100.f);
        configParam(COLOR_PARAM, 0.01f, 0.5f, 0.2f, "Tone (low = darker)", "%", 0.f, 200.f);
        configParam(DRIVE_PARAM, 0.f, 3.f, 1.0f, "Drive (saturation)", "x", 0.f, 1.f, 1.f);
        configParam(MOD_DEPTH_PARAM, 0.f, 1.f, 0.2f, "Mod depth", "%", 0.f, 100.f);
        configSwitch(REVERSE_SWITCH, 0.f, 1.f, 0.f, "Reverse", {"Off", "On"});
        configSwitch(CHAOS_SWITCH, 0.f, 1.f, 0.f, "Chaos LFO", {"Off", "On"});
        configSwitch(PINGPONG_SWITCH, 0.f, 1.f, 0.f, "Ping-pong", {"Off", "On"});
        configSwitch(BITCRUSH_SWITCH, 0.f, 1.f, 0.f, "Bitcrusher", {"Off", "On"});
        configSwitch(FREEZE_SWITCH, 0.f, 1.f, 0.f, "Freeze", {"Off", "On"});
        configSwitch(PHASE_FLIP_SWITCH, 0.f, 1.f, 0.f, "Phase flip", {"Off", "On"});
        configParam(HOWL_PARAM, 0.f, 1.f, 0.f, "Howl", "%", 0.f, 100.f);
        configParam(RING_MOD_PARAM, 0.f, 1.f, 0.f, "Ring mod", "%", 0.f, 100.f);
        configParam(GLITCH_PARAM, 0.f, 1.f, 0.f, "Glitch", "%", 0.f, 100.f);
        configParam(GATE_THRESH_PARAM, 0.f, 1.f, 0.f, "Input gate threshold", " V", 0.f, 4.f);
        configParam(SRR_PARAM, 0.f, 1.f, 0.f, "Sample hold", "%", 0.f, 100.f);
        configParam(CROSS_MOD_PARAM, 0.f, 1.f, 0.f, "Cross-mod", "%", 0.f, 100.f);

        configInput(LEFT_INPUT, "Left (mono if the right input is empty)");
        configInput(RIGHT_INPUT, "Right");
        configInput(TIME_CV_INPUT, "Time CV (0 to 10 V)");
        configInput(FREEZE_CV_INPUT, "Freeze gate");
        configInput(HOWL_CV_INPUT, "Howl CV (0 to 10 V)");
        configInput(RING_CV_INPUT, "Ring mod CV (0 to 10 V)");
        configInput(GLITCH_CV_INPUT, "Glitch CV (0 to 10 V)");
        configInput(GATE_CV_INPUT, "Gate threshold CV (0 to 10 V)");

        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");

        configLight(FREEZE_LIGHT, "Freeze");
        configLight(BITCRUSH_LIGHT, "Bitcrush");
        configLight(PINGPONG_LIGHT, "Ping-pong");
        configLight(REVERSE_LIGHT, "Reverse");
        configLight(CHAOS_LIGHT, "Chaos");
        configLight(PHASE_FLIP_LIGHT, "Phase flip");

        setSampleRate(44100.0f);
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        bufferSize = (size_t)(sr * 2.0f) + 1;
        delayL.assign(bufferSize, 0.0f);
        delayR.assign(bufferSize, 0.0f);
        writePos = 0;
        outFilterL = outFilterR = 0.0f;
        lfoPhase = 0.0f;
        chaosState = 0.5f;
        ringPhase = 0.0f;
        srrCounter = 0.0f;
        lastWetL = lastWetR = 0.0f;
        glitchFade = 0.0f;
        glitchOldL = glitchOldR = 0.0f;
        filterInitialised = false;
        for (int i = 0; i < PD_BINS; ++i) peakBins[i] = 0.f;
        lastBin = -1;
    }

    void onSampleRateChange() override {
        setSampleRate(APP->engine->getSampleRate());
    }

    void process(const ProcessArgs &args) override {
        float sr = args.sampleRate;
        if (sr != sampleRate) setSampleRate(sr);

        float inL = inputs[LEFT_INPUT].getVoltage();
        float inR = inputs[RIGHT_INPUT].isConnected() ? inputs[RIGHT_INPUT].getVoltage() : inL;

        // ===== 0. Input gate (threshold 0 = always open) =====
        float gateT = clamp(params[GATE_THRESH_PARAM].getValue() + inputs[GATE_CV_INPUT].getVoltage() / 10.0f, 0.0f, 1.0f) * 4.0f;
        float lvl = std::max(std::fabs(inL), std::fabs(inR));
        gateEnv = std::max(lvl, gateEnv * (1.0f - 10.0f * args.sampleTime));
        bool gateOpen = gateEnv >= gateT;
        gateGain += ((gateOpen ? 1.0f : 0.0f) - gateGain) * std::min(1.0f, 200.0f * args.sampleTime);
        dispGateActive = gateT > 0.001f;
        dispGateOpen = gateOpen;

        // ===== 1. Delay time & modulation =====
        float timeKnob = params[TIME_PARAM].getValue();
        float timeCv = inputs[TIME_CV_INPUT].getVoltage() / 10.0f;
        float time01 = clamp(timeKnob + timeCv, 0.0f, 1.0f);
        float maxDelaySec = (bufferSize - 1) / sr;
        float delaySec = 0.01f + time01 * (maxDelaySec - 0.01f);

        bool chaosMode = params[CHAOS_SWITCH].getValue() > 0.5f;
        lights[CHAOS_LIGHT].setBrightness(chaosMode ? 1.0f : 0.0f);
        float modDepth = params[MOD_DEPTH_PARAM].getValue();
        float modSignal;
        if (chaosMode) {
            chaosState = 3.9f * chaosState * (1.0f - chaosState);
            modSignal = (chaosState - 0.5f) * 2.0f;
        } else {
            lfoPhase += 0.5f * args.sampleTime;
            if (lfoPhase > 1.0f) lfoPhase -= 1.0f;
            modSignal = sinf(2.0f * M_PI * lfoPhase);
        }
        dispMod = modSignal * modDepth;
        float delayMod = modSignal * modDepth * 0.05f;
        float delaySamples = delaySec * sr * (1.0f + delayMod);
        delaySamples = clamp(delaySamples, 1.0f, (float)(bufferSize - 1));

        // ===== 2. Read position (with Reverse) =====
        bool reverse = params[REVERSE_SWITCH].getValue() > 0.5f;
        lights[REVERSE_LIGHT].setBrightness(reverse ? 1.0f : 0.0f);
        float readPos;
        if (reverse) {
            readPos = (float)writePos + delaySamples;
            while (readPos >= bufferSize) readPos -= bufferSize;
        } else {
            readPos = (float)writePos - delaySamples;
            while (readPos < 0) readPos += bufferSize;
        }
        {
            float age = (float)writePos - readPos;
            if (age < 0.f) age += (float)bufferSize;
            dispReadAge = age / (float)bufferSize;
        }

        // ===== 3. Cross-modulation (right channel offset) =====
        float crossMod = params[CROSS_MOD_PARAM].getValue();
        float crossOffset = crossMod * (delayL[writePos] * 0.002f * sr);
        float readPosR = readPos + crossOffset;
        while (readPosR >= bufferSize) readPosR -= bufferSize;
        while (readPosR < 0) readPosR += bufferSize;

        // Linear interpolation (left)
        int idxL1 = (int)readPos;
        int idxL2 = (idxL1 + 1) % bufferSize;
        float frac = readPos - idxL1;
        float wetRawL = (1.0f - frac) * delayL[idxL1] + frac * delayL[idxL2];

        // Linear interpolation (right)
        int idxR1 = (int)readPosR;
        int idxR2 = (idxR1 + 1) % bufferSize;
        float fracR = readPosR - idxR1;
        float wetRawR = (1.0f - fracR) * delayR[idxR1] + fracR * delayR[idxR2];

        // ===== 4. Glitch with smoothing =====
        float glitchAmount = params[GLITCH_PARAM].getValue();
        float glitchCv = inputs[GLITCH_CV_INPUT].getVoltage() / 10.0f;
        glitchAmount = clamp(glitchAmount + glitchCv, 0.0f, 1.0f);
        if (glitchAmount > 0.0f && dist(rng) < glitchAmount * 0.01f) {
            int jump = (int)(dist(rng) * bufferSize * glitchAmount);
            readPos = fmodf(readPos + jump, (float)bufferSize);
            if (readPos < 0) readPos += bufferSize;
            readPosR = fmodf(readPosR + jump, (float)bufferSize);
            if (readPosR < 0) readPosR += bufferSize;
            glitchOldL = wetRawL;
            glitchOldR = wetRawR;
            glitchFade = 1.0f;
            glitchFlash = 1.0f;
            idxL1 = (int)readPos;
            idxL2 = (idxL1 + 1) % bufferSize;
            frac = readPos - idxL1;
            wetRawL = (1.0f - frac) * delayL[idxL1] + frac * delayL[idxL2];
            idxR1 = (int)readPosR;
            idxR2 = (idxR1 + 1) % bufferSize;
            fracR = readPosR - idxR1;
            wetRawR = (1.0f - fracR) * delayR[idxR1] + fracR * delayR[idxR2];
        }
        // Crossfade over ~2 ms to avoid clicks
        if (glitchFade > 0.0f) {
            float fade = glitchFade;
            wetRawL = wetRawL * (1.0f - fade) + glitchOldL * fade;
            wetRawR = wetRawR * (1.0f - fade) + glitchOldR * fade;
            glitchFade -= args.sampleTime / 0.002f;
            if (glitchFade < 0.0f) glitchFade = 0.0f;
        }
        glitchFlash = std::max(0.f, glitchFlash - 4.f * args.sampleTime);

        // ===== 5. Ring modulation (softer carrier) =====
        float ringAmount = params[RING_MOD_PARAM].getValue();
        float ringCv = inputs[RING_CV_INPUT].getVoltage() / 10.0f;
        ringAmount = clamp(ringAmount + ringCv, 0.0f, 1.0f);
        if (ringAmount > 0.0f) {
            ringPhase += 2.0f * M_PI * 50.0f * args.sampleTime;
            if (ringPhase > 2.0f * M_PI) ringPhase -= 2.0f * M_PI;
            float carrier = sinf(ringPhase);
            wetRawL = wetRawL * (1.0f - ringAmount * 0.5f) + wetRawL * carrier * ringAmount;
            wetRawR = wetRawR * (1.0f - ringAmount * 0.5f) + wetRawR * carrier * ringAmount;
        }

        // ===== 6. Sample rate reduction (disabled by default) =====
        float srr = params[SRR_PARAM].getValue();
        if (srr > 0.0f) {
            float step = 1.0f / (1.0f + srr * 99.0f);
            srrCounter += args.sampleTime;
            if (srrCounter >= step) {
                srrCounter = 0.0f;
                lastWetL = wetRawL;
                lastWetR = wetRawR;
            } else {
                wetRawL = lastWetL;
                wetRawR = lastWetR;
            }
        }

        // ===== 7. Bitcrusher =====
        bool bitcrush = params[BITCRUSH_SWITCH].getValue() > 0.5f;
        lights[BITCRUSH_LIGHT].setBrightness(bitcrush ? 1.0f : 0.0f);
        if (bitcrush) {
            float bits = 6.0f;
            wetRawL = roundf(wetRawL * bits) / bits;
            wetRawR = roundf(wetRawR * bits) / bits;
        }

        // ===== 8. Feedback (using raw delay output) =====
        float feedback = params[FEEDBACK_PARAM].getValue();
        float howl = params[HOWL_PARAM].getValue();
        float howlCv = inputs[HOWL_CV_INPUT].getVoltage() / 10.0f;
        howl = clamp(howl + howlCv, 0.0f, 1.0f);
        float feedbackAmount = feedback + howl * 0.2f;
        float howlSignal = tanhf(wetRawL * howl * 2.0f);
        feedbackAmount += howlSignal * 0.1f;

        // ===== 9. Freeze =====
        bool freeze = (params[FREEZE_SWITCH].getValue() > 0.5f) || (inputs[FREEZE_CV_INPUT].getVoltage() > 1.0f);
        lights[FREEZE_LIGHT].setBrightness(freeze ? 1.0f : 0.0f);
        if (!freeze) {
            delayL[writePos] = pdSoftLimit(inL * gateGain + wetRawL * feedbackAmount);
            delayR[writePos] = pdSoftLimit(inR * gateGain + wetRawR * feedbackAmount);
        } else {
            delayL[writePos] = delayL[writePos] * 0.999f;
            delayR[writePos] = delayR[writePos] * 0.999f;
        }

        // Display overview of what is stored in the delay line
        {
            int bin = (int)((long long)writePos * PD_BINS / (long long)bufferSize);
            if (bin >= PD_BINS) bin = PD_BINS - 1;
            if (bin != lastBin) {
                peakBins[bin] = 0.f;
                lastBin = bin;
            }
            float wv = 0.5f * (std::fabs(delayL[writePos]) + std::fabs(delayR[writePos]));
            if (wv > peakBins[bin]) peakBins[bin] = wv;
            peakWriteBin = bin;
        }

        writePos = (writePos + 1) % bufferSize;

        // ===== 10. Output processing: Drive (saturation) then Color (LPF) =====
        float drive = params[DRIVE_PARAM].getValue() + 1.0f;
        float color = params[COLOR_PARAM].getValue();
        float satL = tanhf(wetRawL * drive);
        float satR = tanhf(wetRawR * drive);

        if (!filterInitialised && (fabs(satL) > 0.001f || fabs(satR) > 0.001f)) {
            outFilterL = satL;
            outFilterR = satR;
            filterInitialised = true;
        } else {
            outFilterL += color * (satL - outFilterL);
            outFilterR += color * (satR - outFilterR);
        }
        float processedL = outFilterL;
        float processedR = outFilterR;

        // ===== 11. Phase Flip & Ping-Pong =====
        bool phaseFlip = params[PHASE_FLIP_SWITCH].getValue() > 0.5f;
        lights[PHASE_FLIP_LIGHT].setBrightness(phaseFlip ? 1.0f : 0.0f);
        bool pingpong = params[PINGPONG_SWITCH].getValue() > 0.5f;
        lights[PINGPONG_LIGHT].setBrightness(pingpong ? 1.0f : 0.0f);

        float outL = processedL;
        float outR = processedR;
        if (pingpong) outR = -processedL;
        if (phaseFlip) outR = -outR;

        // ===== 12. Dry/Wet mix with 1% dry bleed at full wet =====
        float mix = params[MIX_PARAM].getValue();
        float dryAmount = 1.0f - mix;
        if (mix >= 0.999f) dryAmount = 0.01f;
        float finalL = inL * dryAmount + outL * mix;
        float finalR = inR * dryAmount + outR * mix;

        outputs[OUT_L_OUTPUT].setVoltage(finalL);
        outputs[OUT_R_OUTPUT].setVoltage(finalR);

        // Meters
        float md = 1.0f - 5.0f * args.sampleTime;
        meterL = std::max(std::fabs(finalL), meterL * md);
        meterR = std::max(std::fabs(finalR), meterR * md);
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

template <typename TBase = GrayModuleLightWidget>
struct TPDCyanLight : TBase {
    TPDCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using PDCyanLight = TPDCyanLight<>;

template <typename TBase = GrayModuleLightWidget>
struct TPDVioletLight : TBase {
    TPDVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
};
using PDVioletLight = TPDVioletLight<>;

// ------------------------------------------------------------
// AGMM logo
// ------------------------------------------------------------
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

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct PDPanelBackground : TransparentWidget {
    PDPanelBackground() { box.size = Vec(PD_PANEL_W, PD_PANEL_H); }

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

    void caption(NVGcontext* vg, float cx, float y, const char* txt, NVGcolor col, float x1, float x2) {
        drawText(vg, cx, y, txt, 8.f, col, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y + 8.f);
        nvgLineTo(vg, x2, y + 8.f);
        nvgStrokeColor(vg, withAlpha(col, 0.35f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void topCell(NVGcontext* vg, float cx, float cy, const char* txt) {
        plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
        label(vg, cx, cy - 19.f, txt);
    }
    void switchCell(NVGcontext* vg, float cx, float cy, const char* txt) {
        plate(vg, cx - 17.f, cy - 20.f, 34.f, 40.f);
        label(vg, cx, cy + 25.f, txt);
    }
    void knobCell(NVGcontext* vg, float cx, float cy, float r, const char* txt) {
        ring(vg, cx, cy, r);
        label(vg, cx, cy + 25.f, txt);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float W = PD_PANEL_W, H = PD_PANEL_H;

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
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        drawLogo(vg, W / 2.f, 13.f);
        drawText(vg, W / 2.f, 31.f, "PSYDELAY", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = PD_DISP_X - 5.f, by = PD_DISP_Y - 5.f;
        float bw = PD_DISP_W + 10.f, bh = PD_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // ---- knob blocks ----
        caption(vg, 60.f, 54.f, "DELAY", colAccent(), 12.f, 108.f);
        caption(vg, 480.f, 54.f, "FX", colViolet(), 432.f, 528.f);

        const char* leftNames[6] = {"TIME", "FEEDBACK", "MIX", "TONE", "DRIVE", "MOD"};
        const char* rightNames[6] = {"HOWL", "RING", "GLITCH", "GATE", "SRR", "CROSS"};
        for (int i = 0; i < 6; ++i) {
            knobCell(vg, PD_LCOL[i % 2], PD_KROW[i / 2], 18.f, leftNames[i]);
            knobCell(vg, PD_RCOL[i % 2], PD_KROW[i / 2], 18.f, rightNames[i]);
        }

        // ---- switch row ----
        const char* sw[6] = {"REVERSE", "CHAOS", "PINGPONG", "CRUSH", "PHASE", "FREEZE"};
        for (int i = 0; i < 6; ++i)
            switchCell(vg, pdCenterX(i), PD_SW_Y, sw[i]);

        // ---- jack row ----
        caption(vg, 60.f, 270.f, "INPUT", colAccent(), 12.f, 108.f);
        caption(vg, 480.f, 270.f, "OUTPUT", colViolet(), 432.f, 528.f);
        {
            float x1 = 124.f, x2 = 416.f, y = 270.f, cx = (x1 + x2) / 2.f;
            float w = 10.f + 8.f * 7.f;
            drawText(vg, cx, y, "CV INPUT", 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
            nvgBeginPath(vg);
            nvgMoveTo(vg, x1, y); nvgLineTo(vg, cx - w / 2.f, y);
            nvgMoveTo(vg, cx + w / 2.f, y); nvgLineTo(vg, x2, y);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }
        topCell(vg, PD_LCOL[0], PD_JACK_Y, "L IN");
        topCell(vg, PD_LCOL[1], PD_JACK_Y, "R IN");
        const char* cv[6] = {"TIME CV", "HOWL CV", "RING CV", "GLT CV", "GATE CV", "FRZ GATE"};
        for (int i = 0; i < 6; ++i)
            topCell(vg, pdCenterX(i), PD_JACK_Y, cv[i]);
        topCell(vg, PD_RCOL[0], PD_JACK_Y, "OUT L");
        topCell(vg, PD_RCOL[1], PD_JACK_Y, "OUT R");

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Display: the delay line as a timeline, with the read tap
// ------------------------------------------------------------
struct PDDisplayWidget : TransparentWidget {
    PsyDelay* module = nullptr;

    PDDisplayWidget(PsyDelay* m) : module(m) {
        box.pos = Vec(PD_DISP_X, PD_DISP_Y);
        box.size = Vec(PD_DISP_W, PD_DISP_H);
    }

    // Static background: screen, timeline field, grid, ruler
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        const float W = PD_DISP_W, H = PD_DISP_H;
        const float X0 = 14.f, Y0 = 20.f, TW = 260.f, TH = 64.f, mid = Y0 + TH / 2.f;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H);
        nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 30.f, W * 0.7f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, X0 - 1.f, Y0 - 1.f, TW + 2.f, TH + 2.f, 2.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Grid every 0.5 s (the buffer holds 2 s, "now" is on the right)
        nvgBeginPath(vg);
        for (int k = 1; k < 4; ++k) {
            float x = X0 + TW - k * (TW / 4.f) + 0.5f;
            nvgMoveTo(vg, x, Y0);
            nvgLineTo(vg, x, Y0 + TH);
        }
        nvgMoveTo(vg, X0, mid + 0.5f);
        nvgLineTo(vg, X0 + TW, mid + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Ruler
        nvgBeginPath(vg);
        for (int k = 0; k <= 4; ++k) {
            float x = X0 + TW - k * (TW / 4.f) + 0.5f;
            nvgMoveTo(vg, x, Y0 + TH + 1.f);
            nvgLineTo(vg, x, Y0 + TH + 4.f);
        }
        nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 80));
        nvgStroke(vg);
        const char* tl[5] = {"0", "0.5", "1.0", "1.5", "2.0S"};
        for (int k = 0; k <= 4; ++k)
            drawText(vg, X0 + TW - k * (TW / 4.f), Y0 + TH + 9.f, tl[k], 5.5f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        const float W = PD_DISP_W, H = PD_DISP_H;
        const float X0 = 14.f, Y0 = 20.f, TW = 260.f, TH = 64.f, mid = Y0 + TH / 2.f;

        // ---- gather state (live, or a sample scene in the module browser) ----
        float ord[PD_BINS];
        float readAge, mod, glf, mL, mR;
        float timeMs, fb, mix, drive, glitch, ring, srr;
        bool frz, rev, chs, png, crush, phs, gAct, gOpen;

        if (module) {
            int wb = module->peakWriteBin;
            for (int i = 0; i < PD_BINS; ++i)
                ord[i] = module->peakBins[(wb + 1 + i) % PD_BINS];
            readAge = module->dispReadAge;
            mod = module->dispMod;
            glf = module->glitchFlash;
            mL = module->meterL; mR = module->meterR;
            float tk = module->params[PsyDelay::TIME_PARAM].getValue();
            timeMs = (0.01f + tk * 1.99f) * 1000.f;
            fb = module->params[PsyDelay::FEEDBACK_PARAM].getValue();
            mix = module->params[PsyDelay::MIX_PARAM].getValue();
            drive = module->params[PsyDelay::DRIVE_PARAM].getValue() + 1.f;
            glitch = module->params[PsyDelay::GLITCH_PARAM].getValue();
            ring = module->params[PsyDelay::RING_MOD_PARAM].getValue();
            srr = module->params[PsyDelay::SRR_PARAM].getValue();
            frz = module->params[PsyDelay::FREEZE_SWITCH].getValue() > 0.5f;
            rev = module->params[PsyDelay::REVERSE_SWITCH].getValue() > 0.5f;
            chs = module->params[PsyDelay::CHAOS_SWITCH].getValue() > 0.5f;
            png = module->params[PsyDelay::PINGPONG_SWITCH].getValue() > 0.5f;
            crush = module->params[PsyDelay::BITCRUSH_SWITCH].getValue() > 0.5f;
            phs = module->params[PsyDelay::PHASE_FLIP_SWITCH].getValue() > 0.5f;
            gAct = module->dispGateActive;
            gOpen = module->dispGateOpen;
        } else {
            for (int i = 0; i < PD_BINS; ++i) {
                float age = (float)(PD_BINS - 1 - i);
                ord[i] = 4.f * std::exp(-age * 0.022f) * (0.35f + 0.65f * std::fabs(std::sin(age * 0.33f)));
            }
            readAge = 0.25f; mod = 0.3f; glf = 0.f; mL = 3.f; mR = 2.4f;
            timeMs = 500.f; fb = 0.8f; mix = 0.5f; drive = 2.f; glitch = 0.f; ring = 0.f; srr = 0.f;
            frz = false; rev = false; chs = false; png = true; crush = false; phs = false; gAct = false; gOpen = true;
        }

        // ---- delay line contents (mirrored bars, newest on the right) ----
        nvgSave(vg);
        nvgScissor(vg, X0, Y0, TW, TH);
        const float amp = TH / 2.f - 2.f;
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < PD_BINS; ++i) {
                float h = std::min(ord[i] / 5.f, 1.f) * amp;
                if (h < 0.4f) continue;
                float x = X0 + i * 2.f;
                if (pass == 0) nvgRect(vg, x - 0.5f, mid - h - 1.5f, 3.f, 2.f * h + 3.f);
                else nvgRect(vg, x, mid - h, 1.6f, 2.f * h);
            }
            if (frz)
                nvgFillColor(vg, pass == 0 ? withAlpha(colViolet(), 0.16f) : nvgRGB(0xd8, 0xb0, 0xff));
            else
                nvgFillColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 32) : nvgRGB(0x8a, 0xee, 0xff));
            nvgFill(vg);
        }
        nvgResetScissor(vg);
        nvgRestore(vg);

        // ---- tap region, tap marker, now marker ----
        float tx = X0 + (1.f - std::min(std::max(readAge, 0.f), 1.f)) * TW;
        nvgBeginPath(vg);
        nvgRect(vg, tx, Y0, X0 + TW - tx, TH);
        nvgFillColor(vg, withAlpha(colViolet(), 0.10f));
        nvgFill(vg);

        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, tx + 0.5f, Y0);
            nvgLineTo(vg, tx + 0.5f, Y0 + TH);
            nvgStrokeColor(vg, pass == 0 ? withAlpha(colViolet(), 0.25f + 0.5f * glf) : nvgRGBA(0xd8, 0xb0, 0xff, 240));
            nvgStrokeWidth(vg, pass == 0 ? 4.f : 1.3f);
            nvgStroke(vg);
        }
        nvgBeginPath(vg);
        nvgMoveTo(vg, tx - 3.5f, Y0 - 3.f);
        nvgLineTo(vg, tx + 4.5f, Y0 - 3.f);
        nvgLineTo(vg, tx + 0.5f, Y0 + 2.f);
        nvgClosePath(vg);
        nvgFillColor(vg, colViolet());
        nvgFill(vg);

        if (glf > 0.02f) {
            nvgBeginPath(vg);
            nvgCircle(vg, tx + 0.5f, mid, 3.f + (1.f - glf) * 14.f);
            nvgStrokeColor(vg, withAlpha(colViolet(), 0.9f * glf));
            nvgStrokeWidth(vg, 1.5f);
            nvgStroke(vg);
        }

        // Modulation wobble under the tap
        {
            float mx = tx + 0.5f + std::min(std::max(mod, -1.f), 1.f) * 8.f;
            nvgBeginPath(vg);
            nvgCircle(vg, mx, Y0 + TH - 5.f, 2.f);
            nvgFillColor(vg, withAlpha(colViolet(), 0.9f));
            nvgFill(vg);
        }

        // Now marker
        nvgBeginPath(vg);
        nvgRect(vg, X0 + TW - 1.5f, Y0, 1.5f, TH);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 200));
        nvgFill(vg);

        if (frz)
            drawText(vg, X0 + TW / 2.f, Y0 + 8.f, "FROZEN", 7.f, withAlpha(colViolet(), 0.9f),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
        drawText(vg, tx + (tx > X0 + 24.f ? -4.f : 4.f), Y0 + 8.f, rev ? "TAP (REV)" : "TAP", 5.5f,
                 withAlpha(colViolet(), 0.9f),
                 tx > X0 + 24.f ? (NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE) : (NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE), 0.5f);

        // ---- output meters ----
        for (int ch = 0; ch < 2; ++ch) {
            float v = ch == 0 ? mL : mR;
            float lv = std::min(v / 5.f, 1.f);
            float x = X0 + TW + 6.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, Y0, 4.f, TH);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lv > 0.01f) {
                float h = lv * TH;
                nvgBeginPath(vg);
                nvgRect(vg, x, Y0 + TH - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, Y0 + TH, 0, Y0,
                                                   nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }

        // ---- state chips ----
        auto chip = [&](float x, float w, const char* txt, bool on, NVGcolor col) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, x, 101.f, w, 11.f, 3.f);
            if (on) {
                nvgFillColor(vg, withAlpha(col, 0.25f));
                nvgFill(vg);
                nvgStrokeColor(vg, withAlpha(col, 0.9f));
            } else {
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
            }
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, x + w / 2.f, 106.5f, txt, 6.f, on ? col : colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
        };
        chip(14.f,  34.f, "FRZ",  frz,   colViolet());
        chip(52.f,  34.f, "REV",  rev,   colAccent());
        chip(90.f,  34.f, "CHS",  chs,   colAccent());
        chip(128.f, 34.f, "PNG",  png,   colAccent());
        chip(166.f, 34.f, "CRSH", crush, colAccent());
        chip(204.f, 34.f, "PHS",  phs,   colAccent());
        chip(242.f, 34.f, "GATE", gAct && gOpen, colViolet());

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
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
        NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 230);
        snprintf(buf, sizeof(buf), "TIME %4.0fMS", timeMs);
        drawText(vg, 14.f, 9.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "FB %3.0f%%", fb * 100.f);
        drawText(vg, W / 2.f, 9.f, buf, 7.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "MIX %3.0f%%", mix * 100.f);
        drawText(vg, W - 14.f, 9.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        snprintf(buf, sizeof(buf), "DRV %.1fX", drive);
        drawText(vg, 14.f, 123.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "GLT %.0f%%  RNG %.0f%%", glitch * 100.f, ring * 100.f);
        drawText(vg, W / 2.f, 123.f, buf, 7.f, cd, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "SRR %.0f%%", srr * 100.f);
        drawText(vg, W - 14.f, 123.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Widget
// ------------------------------------------------------------
struct PsyDelayWidget : ModuleWidget {
    PsyDelayWidget(PsyDelay *module) {
        setModule(module);
        box.size = Vec(PD_PANEL_W, PD_PANEL_H);

        addChild(new PDPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PDDisplayWidget(module));

        // ---- left knobs ----
        const int leftIds[6] = {PsyDelay::TIME_PARAM, PsyDelay::FEEDBACK_PARAM, PsyDelay::MIX_PARAM,
                                PsyDelay::COLOR_PARAM, PsyDelay::DRIVE_PARAM, PsyDelay::MOD_DEPTH_PARAM};
        const int rightIds[6] = {PsyDelay::HOWL_PARAM, PsyDelay::RING_MOD_PARAM, PsyDelay::GLITCH_PARAM,
                                 PsyDelay::GATE_THRESH_PARAM, PsyDelay::SRR_PARAM, PsyDelay::CROSS_MOD_PARAM};
        for (int i = 0; i < 6; ++i) {
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PD_LCOL[i % 2], PD_KROW[i / 2]), module, leftIds[i]));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PD_RCOL[i % 2], PD_KROW[i / 2]), module, rightIds[i]));
        }

        // ---- switches with LEDs ----
        const int swIds[6] = {PsyDelay::REVERSE_SWITCH, PsyDelay::CHAOS_SWITCH, PsyDelay::PINGPONG_SWITCH,
                              PsyDelay::BITCRUSH_SWITCH, PsyDelay::PHASE_FLIP_SWITCH, PsyDelay::FREEZE_SWITCH};
        const int ledIds[6] = {PsyDelay::REVERSE_LIGHT, PsyDelay::CHAOS_LIGHT, PsyDelay::PINGPONG_LIGHT,
                               PsyDelay::BITCRUSH_LIGHT, PsyDelay::PHASE_FLIP_LIGHT, PsyDelay::FREEZE_LIGHT};
        for (int i = 0; i < 6; ++i) {
            float cx = pdCenterX(i);
            addParam(createParamCentered<CKSS>(Vec(cx - 2.f, PD_SW_Y), module, swIds[i]));
            if (i == 5)
                addChild(createLightCentered<SmallLight<PDVioletLight>>(Vec(cx + 12.f, PD_SW_Y - 13.f), module, ledIds[i]));
            else
                addChild(createLightCentered<SmallLight<PDCyanLight>>(Vec(cx + 12.f, PD_SW_Y - 13.f), module, ledIds[i]));
        }

        // ---- jacks ----
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PD_LCOL[0], PD_JACK_Y), module, PsyDelay::LEFT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(PD_LCOL[1], PD_JACK_Y), module, PsyDelay::RIGHT_INPUT));
        const int cvIds[6] = {PsyDelay::TIME_CV_INPUT, PsyDelay::HOWL_CV_INPUT, PsyDelay::RING_CV_INPUT,
                              PsyDelay::GLITCH_CV_INPUT, PsyDelay::GATE_CV_INPUT, PsyDelay::FREEZE_CV_INPUT};
        for (int i = 0; i < 6; ++i)
            addInput(createInputCentered<DarkPJ301MPort>(Vec(pdCenterX(i), PD_JACK_Y), module, cvIds[i]));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PD_RCOL[0], PD_JACK_Y), module, PsyDelay::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(PD_RCOL[1], PD_JACK_Y), module, PsyDelay::OUT_R_OUTPUT));
    }
};

Model *modelPsyDelay = createModel<PsyDelay, PsyDelayWidget>("PsyDelay");