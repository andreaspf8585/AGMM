#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   SD_BINS = 82;                          // 2 px per bin
static const float SD_W = RACK_GRID_WIDTH * 16;           // 240
static const float SD_H = RACK_GRID_HEIGHT;               // 380
static const float SD_DX = 20.f, SD_DY = 46.f, SD_DW = 200.f, SD_DH = 84.f;
static const float SD_KCOL[4] = {30.f, 90.f, 150.f, 210.f};
static const float SD_KROW[2] = {166.f, 220.f};
static const float SD_SWY = 282.f;
static const float SD_JY = 346.f;
static float sdJx(int i) { return 30.f + 30.f * i; }     // 7 jacks, 30 px apart

// Clock sync: the clock input is treated as one pulse per quarter note (beat).
// The time knobs select one of these divisions when a clock is patched.
static const int   SD_NDIV = 10;
static const float SD_DIV_MULT[SD_NDIV] = {0.25f, 1.f / 3.f, 0.5f, 2.f / 3.f, 0.75f, 1.f, 1.5f, 2.f, 3.f, 4.f};
static const char* SD_DIV_NAME[SD_NDIV] = {"1/16", "1/8T", "1/8", "1/4T", "1/8.", "1/4", "1/4.", "1/2", "1/2.", "1/1"};

struct SickDelay : Module
{
    // New items are appended at the end of each enum so old patches keep loading.
    enum ParamId
    {
        TIME_PARAM,          // left delay time
        FEEDBACK_PARAM,
        MIX_PARAM,
        COLOR_PARAM,
        DRIVE_PARAM,
        MOD_PARAM,
        PINGPONG_SWITCH,
        BITCRUSH_SWITCH,
        FREEZE_SWITCH,
        TIME_R_PARAM,        // right delay time
        LINK_SWITCH,         // right time follows left time
        PARAMS_LEN
    };
    enum InputId
    {
        LEFT_INPUT,
        RIGHT_INPUT,
        TIME_CV_INPUT,
        FREEZE_CV_INPUT,
        CLOCK_INPUT,
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
        FREEZE_LIGHT,
        BITCRUSH_LIGHT,
        PINGPONG_LIGHT,
        LINK_LIGHT,
        LIGHTS_LEN
    };

    static constexpr size_t BUFFER_SIZE = 96000;
    float bufferL[BUFFER_SIZE] = {};
    float bufferR[BUFFER_SIZE] = {};
    int writePosL = 0;
    int writePosR = 0;
    float lastSampleL = 0.0f;
    float lastSampleR = 0.0f;
    float lfoPhase = 0.0f;

    // Clock tracking
    dsp::SchmittTrigger clockTrigger;
    float clockTimer = 0.f;      // seconds since the last clock edge
    float clockPeriod = 0.5f;    // measured seconds per clock pulse (beat)
    bool clockHasEdge = false;
    bool clockValid = false;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float peakBins[SD_BINS] = {};
    int peakWriteBin = 0;
    int lastBin = -1;
    float dispAgeL = 0.25f, dispAgeR = 0.25f;   // tap positions: 0 = now, 1 = oldest
    float dispMsL = 0.f, dispMsR = 0.f;
    int dispDivL = -1, dispDivR = -1;           // -1 = free running
    bool dispSync = false;
    float dispMod = 0.f;
    float meterL = 0.f, meterR = 0.f;

    SickDelay()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        // Full range is (BUFFER_SIZE - 2000) samples, about 1.96 s at 48 kHz.
        // With a clock patched the knobs pick a note division instead.
        configParam(TIME_PARAM, 0.f, 1.f, 0.5f, "Left delay time (at 48 kHz) / division when synced", " s", 0.f, 1.958f);
        configParam(TIME_R_PARAM, 0.f, 1.f, 0.5f, "Right delay time (at 48 kHz) / division when synced", " s", 0.f, 1.958f);
        configParam(FEEDBACK_PARAM, 0.f, 1.0f, 0.5f, "Feedback", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/wet mix", "%", 0.f, 100.f);
        configParam(COLOR_PARAM, 0.01f, 0.5f, 0.2f, "Tone (low = darker)", "%", 0.f, 200.f);
        configParam(DRIVE_PARAM, 0.f, 3.f, 0.5f, "Drive (saturation)", "x", 0.f, 1.f, 1.f);
        configParam(MOD_PARAM, 0.f, 1.f, 0.2f, "Modulation depth", "%", 0.f, 100.f);
        configSwitch(PINGPONG_SWITCH, 0.f, 1.f, 0.f, "Ping-pong", {"Off", "On"});
        configSwitch(BITCRUSH_SWITCH, 0.f, 1.f, 0.f, "Bitcrusher", {"Off", "On"});
        configSwitch(FREEZE_SWITCH, 0.f, 1.f, 0.f, "Freeze", {"Off", "On"});
        configSwitch(LINK_SWITCH, 0.f, 1.f, 0.f, "Link L/R time (right follows left)", {"Off", "On"});

        configInput(LEFT_INPUT, "Left (mono if the right input is empty)");
        configInput(RIGHT_INPUT, "Right");
        configInput(TIME_CV_INPUT, "Time CV, both channels (10 V = 100%)");
        configInput(FREEZE_CV_INPUT, "Freeze gate");
        configInput(CLOCK_INPUT, "Clock (1 pulse per beat) - syncs the delay times");
        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");
        configLight(FREEZE_LIGHT, "Freeze");
        configLight(BITCRUSH_LIGHT, "Bitcrusher");
        configLight(PINGPONG_LIGHT, "Ping-pong");
        configLight(LINK_LIGHT, "Link L/R time");
    }

    void onReset() override
    {
        clockTrigger.reset();
        clockTimer = 0.f;
        clockHasEdge = false;
        clockValid = false;
    }

    void process(const ProcessArgs &args) override
    {
        float inL = inputs[LEFT_INPUT].getVoltage();
        float inR = inputs[RIGHT_INPUT].isConnected() ? inputs[RIGHT_INPUT].getVoltage() : inL; // mono fallback

        // 1. FREEZE LOGIC (switch OR gate)
        bool freezeActive = (params[FREEZE_SWITCH].getValue() > 0.5f) || (inputs[FREEZE_CV_INPUT].getVoltage() > 2.0f);
        lights[FREEZE_LIGHT].setBrightness(freezeActive ? 1.0f : 0.0f);

        // 2. CLOCK TRACKING (measure time between rising edges)
        if (inputs[CLOCK_INPUT].isConnected())
        {
            clockTimer += args.sampleTime;
            if (clockTrigger.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f))
            {
                if (clockHasEdge && clockTimer > 0.01f)
                {
                    clockPeriod = clockTimer;
                    clockValid = true;
                }
                clockHasEdge = true;
                clockTimer = 0.f;
            }
            if (clockTimer > 4.f)
            {
                // clock stopped: fall back to free-running time
                clockValid = false;
                clockHasEdge = false;
            }
        }
        else
        {
            clockValid = false;
            clockHasEdge = false;
            clockTimer = 0.f;
        }
        bool sync = clockValid;

        // 3. MODULATION (shared LFO)
        lfoPhase += 0.5f * args.sampleTime;
        if (lfoPhase > 1.f)
            lfoPhase -= 1.f;
        float lfoSine = sinf(2.f * M_PI * lfoPhase);
        float lfo = lfoSine * params[MOD_PARAM].getValue() * 200.f;
        dispMod = lfoSine * params[MOD_PARAM].getValue();

        // 4. DELAY TIMES (left / right, free or clock synced)
        float cv = inputs[TIME_CV_INPUT].isConnected() ? inputs[TIME_CV_INPUT].getVoltage() * 0.1f : 0.f;
        bool link = params[LINK_SWITCH].getValue() > 0.5f;
        lights[LINK_LIGHT].setBrightness(link ? 1.0f : 0.0f);

        const float maxDelay = (float)BUFFER_SIZE - 2000.f;
        auto calcDelay = [&](float knob, int &divOut) -> float
        {
            float t = clamp(knob + cv, 0.f, 1.f);
            float d;
            if (sync)
            {
                int idx = clamp((int)(t * SD_NDIV), 0, SD_NDIV - 1);
                divOut = idx;
                d = clockPeriod * SD_DIV_MULT[idx] * args.sampleRate;
                // too long for the buffer: halve until it fits (keeps it rhythmic)
                while (d > maxDelay)
                    d *= 0.5f;
            }
            else
            {
                divOut = -1;
                d = t * maxDelay;
            }
            return clamp(d + lfo, 1.f, (float)BUFFER_SIZE - 1.f);
        };

        float knobL = params[TIME_PARAM].getValue();
        float knobR = link ? knobL : params[TIME_R_PARAM].getValue();
        int divL, divR;
        float delaySamplesL = calcDelay(knobL, divL);
        float delaySamplesR = calcDelay(knobR, divR);

        dispAgeL = delaySamplesL / (float)BUFFER_SIZE;
        dispAgeR = delaySamplesR / (float)BUFFER_SIZE;
        dispMsL = delaySamplesL / args.sampleRate * 1000.f;
        dispMsR = delaySamplesR / args.sampleRate * 1000.f;
        dispDivL = divL;
        dispDivR = divR;
        dispSync = sync;

        // Helper function to read from a delay line
        auto readDelay = [&](const float *buffer, int writePos, float delaySamples)
        {
            float readPos = (float)writePos - delaySamples;
            while (readPos < 0)
                readPos += (float)BUFFER_SIZE;
            int i1 = (int)readPos % (int)BUFFER_SIZE;
            int i2 = (i1 + 1) % (int)BUFFER_SIZE;
            float frac = readPos - (int)readPos;
            return (1.f - frac) * buffer[i1] + frac * buffer[i2];
        };

        // 5. READ DELAY LINES
        float delayedL = readDelay(bufferL, writePosL, delaySamplesL);
        float delayedR = readDelay(bufferR, writePosR, delaySamplesR);

        // 6. BITCRUSHER (applied independently to each channel)
        bool bitcrushOn = params[BITCRUSH_SWITCH].getValue() > 0.5f;
        if (bitcrushOn)
        {
            float bits = 6.0f;
            delayedL = roundf(delayedL * bits) / bits;
            delayedR = roundf(delayedR * bits) / bits;
            lights[BITCRUSH_LIGHT].setBrightness(1.0f);
        }
        else
        {
            lights[BITCRUSH_LIGHT].setBrightness(0.0f);
        }

        // 7. SATURATION & FILTER (one-pole lowpass per channel)
        float feedback = params[FEEDBACK_PARAM].getValue();
        float drive = params[DRIVE_PARAM].getValue() + 1.0f;
        float color = params[COLOR_PARAM].getValue();

        float processedL = tanhf(delayedL * drive);
        float processedR = tanhf(delayedR * drive);
        lastSampleL += color * (processedL - lastSampleL);
        lastSampleR += color * (processedR - lastSampleR);

        // 8. WRITE TO DELAY LINES (skip when frozen)
        if (!freezeActive)
        {
            bufferL[writePosL] = inL + (lastSampleL * feedback);
            bufferR[writePosR] = inR + (lastSampleR * feedback);
        }
        else
        {
            // slow decay when frozen
            bufferL[writePosL] = lastSampleL * 0.999f;
            bufferR[writePosR] = lastSampleR * 0.999f;
        }

        // Display overview of what is stored in the delay line
        {
            int bin = (int)((long long)writePosL * SD_BINS / (long long)BUFFER_SIZE);
            if (bin >= SD_BINS) bin = SD_BINS - 1;
            if (bin != lastBin) {
                peakBins[bin] = 0.f;
                lastBin = bin;
            }
            float wv = 0.5f * (std::fabs(bufferL[writePosL]) + std::fabs(bufferR[writePosR]));
            if (wv > peakBins[bin]) peakBins[bin] = wv;
            peakWriteBin = bin;
        }

        writePosL = (writePosL + 1) % BUFFER_SIZE;
        writePosR = (writePosR + 1) % BUFFER_SIZE;

        // 9. OUTPUT - apply ping-pong if enabled (right output = inverted left delay)
        float mix = params[MIX_PARAM].getValue();
        float outL = lastSampleL;
        float outR = lastSampleR;

        bool pingpongOn = params[PINGPONG_SWITCH].getValue() > 0.5f;
        if (pingpongOn)
        {
            outR = -lastSampleL;
            lights[PINGPONG_LIGHT].setBrightness(1.0f);
        }
        else
        {
            lights[PINGPONG_LIGHT].setBrightness(0.0f);
        }

        float finalL = rack::math::crossfade(inL, outL, mix);
        float finalR = rack::math::crossfade(inR, outR, mix);
        outputs[OUT_L_OUTPUT].setVoltage(finalL);
        outputs[OUT_R_OUTPUT].setVoltage(finalR);

        float md = 1.f - 5.f * args.sampleTime;
        meterL = std::max(std::fabs(finalL), meterL * md);
        meterR = std::max(std::fabs(finalR), meterR * md);
    }
};

namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colViolet()   { return nvgRGB(0xb0, 0x5a, 0xff); }
NVGcolor withAlpha(NVGcolor c, float a) { c.a = a; return c; }

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

template <typename TBase = GrayModuleLightWidget>
struct TSDCyanLight : TBase {
    TSDCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using SDCyanLight = TSDCyanLight<>;

template <typename TBase = GrayModuleLightWidget>
struct TSDVioletLight : TBase {
    TSDVioletLight() { this->addBaseColor(nvgRGB(0xb0, 0x5a, 0xff)); }
};
using SDVioletLight = TSDVioletLight<>;

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

void drawBezel(NVGcontext* vg, float x, float y, float w, float h) {
    float bx = x - 5.f, by = y - 5.f, bw = w + 10.f, bh = h + 10.f;
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh, nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
    nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
    nvgFill(vg);
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

void label(NVGcontext* vg, float x, float y, const char* t, float size = 7.f) {
    drawText(vg, x, y, t, size, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
}

void topCell(NVGcontext* vg, float cx, float cy, const char* t) {
    plate(vg, cx - 14.f, cy - 27.f, 28.f, 44.f);
    label(vg, cx, cy - 19.f, t, 6.f);
}

void screenBase(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 20.f, W * 0.75f, nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
    nvgFill(vg);
}

void screenFinish(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    for (int y = 0; y < (int) H; y += 3) nvgRect(vg, 0, y + 1, W, 1);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H * 0.3f);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f, nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
    nvgFill(vg);
}

struct SDPanel : TransparentWidget {
    SDPanel() { box.size = Vec(SD_W, SD_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, SD_W, SD_H, "SICKDELAY");
        drawBezel(vg, SD_DX, SD_DY, SD_DW, SD_DH);

        // 7 knobs in a 4 x 2 grid (last slot of row 2 is empty)
        const char* names[7] = {"TIME L", "TIME R", "FEEDBACK", "MIX", "TONE", "DRIVE", "MOD"};
        for (int i = 0; i < 7; ++i) {
            float cx = SD_KCOL[i % 4], cy = SD_KROW[i / 4];
            ring(vg, cx, cy, 18.f);
            label(vg, cx, cy + 25.f, names[i]);
        }

        const char* sw[4] = {"PINGPONG", "CRUSH", "FREEZE", "LINK L/R"};
        for (int i = 0; i < 4; ++i) {
            plate(vg, SD_KCOL[i] - 17.f, SD_SWY - 20.f, 34.f, 40.f);
            label(vg, SD_KCOL[i], SD_SWY + 25.f, sw[i]);
        }

        const char* io[7] = {"L IN", "R IN", "TIME CV", "CLOCK", "FREEZE", "OUT L", "OUT R"};
        for (int i = 0; i < 7; ++i) topCell(vg, sdJx(i), SD_JY, io[i]);
        Widget::draw(args);
    }
};

// Display: the delay line as a timeline, with the read taps
struct SDDisplay : TransparentWidget {
    SickDelay* module = nullptr;
    SDDisplay(SickDelay* m) : module(m) {
        box.pos = Vec(SD_DX, SD_DY);
        box.size = Vec(SD_DW, SD_DH);
    }

    static constexpr float X0 = 12.f, Y0 = 16.f, TW = 164.f, TH = 46.f;

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        screenBase(vg, SD_DW, SD_DH);
        float mid = Y0 + TH / 2.f;

        nvgBeginPath(vg);
        nvgRoundedRect(vg, X0 - 1.f, Y0 - 1.f, TW + 2.f, TH + 2.f, 2.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // grid every 0.5 s (the buffer holds about 2 s, "now" is on the right)
        nvgBeginPath(vg);
        for (int k = 1; k < 4; ++k) {
            float x = X0 + TW - k * (TW / 4.f) + 0.5f;
            nvgMoveTo(vg, x, Y0);
            nvgLineTo(vg, x, Y0 + TH);
        }
        nvgMoveTo(vg, X0, mid + 0.5f);
        nvgLineTo(vg, X0 + TW, mid + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
        nvgStroke(vg);

        const char* tl[5] = {"0", "0.5", "1.0", "1.5", "2.0S"};
        for (int k = 0; k <= 4; ++k)
            drawText(vg, X0 + TW - k * (TW / 4.f), Y0 + TH + 8.f, tl[k], 5.5f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = SD_DW, H = SD_DH;
        const float mid = Y0 + TH / 2.f;

        float ord[SD_BINS];
        float ageL = 0.3f, ageR = 0.2f, msL = 600.f, msR = 400.f, mod = 0.3f, mL = 3.f, mR = 2.4f;
        float fb = 0.5f, mixv = 0.5f, drive = 1.5f;
        bool frz = false, png = false, crush = false, sync = false, link = false;
        int divL = -1, divR = -1;

        if (module) {
            int wb = module->peakWriteBin;
            for (int i = 0; i < SD_BINS; ++i)
                ord[i] = module->peakBins[(wb + 1 + i) % SD_BINS];
            ageL = module->dispAgeL; ageR = module->dispAgeR;
            msL = module->dispMsL; msR = module->dispMsR;
            divL = module->dispDivL; divR = module->dispDivR;
            sync = module->dispSync;
            mod = module->dispMod;
            mL = module->meterL; mR = module->meterR;
            fb = module->params[SickDelay::FEEDBACK_PARAM].getValue();
            mixv = module->params[SickDelay::MIX_PARAM].getValue();
            drive = module->params[SickDelay::DRIVE_PARAM].getValue() + 1.f;
            frz = module->params[SickDelay::FREEZE_SWITCH].getValue() > 0.5f;
            png = module->params[SickDelay::PINGPONG_SWITCH].getValue() > 0.5f;
            crush = module->params[SickDelay::BITCRUSH_SWITCH].getValue() > 0.5f;
            link = module->params[SickDelay::LINK_SWITCH].getValue() > 0.5f;
        } else {
            for (int i = 0; i < SD_BINS; ++i) {
                float a = (float)(SD_BINS - 1 - i);
                ord[i] = 4.f * std::exp(-a * 0.035f) * (0.35f + 0.65f * std::fabs(std::sin(a * 0.33f)));
            }
        }

        // delay line contents (mirrored bars, newest on the right)
        nvgSave(vg);
        nvgScissor(vg, X0, Y0, TW, TH);
        const float amp = TH / 2.f - 2.f;
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < SD_BINS; ++i) {
                float h = std::min(ord[i] / 5.f, 1.f) * amp;
                if (h < 0.4f) continue;
                float x = X0 + i * 2.f;
                if (pass == 0) nvgRect(vg, x - 0.5f, mid - h - 1.5f, 3.f, 2.f * h + 3.f);
                else nvgRect(vg, x, mid - h, 1.6f, 2.f * h);
            }
            if (frz) nvgFillColor(vg, pass == 0 ? withAlpha(colViolet(), 0.16f) : nvgRGB(0xd8, 0xb0, 0xff));
            else nvgFillColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 32) : nvgRGB(0x8a, 0xee, 0xff));
            nvgFill(vg);
        }
        nvgResetScissor(vg);
        nvgRestore(vg);

        // tap regions and tap markers: left = violet (top triangle), right = cyan (bottom triangle)
        float txL = X0 + (1.f - std::min(std::max(ageL, 0.f), 1.f)) * TW;
        float txR = X0 + (1.f - std::min(std::max(ageR, 0.f), 1.f)) * TW;

        nvgBeginPath(vg);
        nvgRect(vg, txL, Y0, X0 + TW - txL, TH);
        nvgFillColor(vg, withAlpha(colViolet(), 0.08f));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRect(vg, txR, Y0, X0 + TW - txR, TH);
        nvgFillColor(vg, withAlpha(colAccent(), 0.06f));
        nvgFill(vg);

        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, txL + 0.5f, Y0);
            nvgLineTo(vg, txL + 0.5f, Y0 + TH);
            nvgStrokeColor(vg, pass == 0 ? withAlpha(colViolet(), 0.3f) : nvgRGBA(0xd8, 0xb0, 0xff, 240));
            nvgStrokeWidth(vg, pass == 0 ? 4.f : 1.3f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, txR + 0.5f, Y0);
            nvgLineTo(vg, txR + 0.5f, Y0 + TH);
            nvgStrokeColor(vg, pass == 0 ? withAlpha(colAccent(), 0.3f) : nvgRGBA(0xc0, 0xf6, 0xff, 240));
            nvgStrokeWidth(vg, pass == 0 ? 4.f : 1.3f);
            nvgStroke(vg);
        }
        nvgBeginPath(vg);
        nvgMoveTo(vg, txL - 3.5f, Y0 - 3.f);
        nvgLineTo(vg, txL + 4.5f, Y0 - 3.f);
        nvgLineTo(vg, txL + 0.5f, Y0 + 2.f);
        nvgClosePath(vg);
        nvgFillColor(vg, colViolet());
        nvgFill(vg);

        nvgBeginPath(vg);
        nvgMoveTo(vg, txR - 3.5f, Y0 + TH + 3.f);
        nvgLineTo(vg, txR + 4.5f, Y0 + TH + 3.f);
        nvgLineTo(vg, txR + 0.5f, Y0 + TH - 2.f);
        nvgClosePath(vg);
        nvgFillColor(vg, colAccent());
        nvgFill(vg);

        // modulation wobble marker under the left tap
        {
            float mx = txL + 0.5f + std::min(std::max(mod, -1.f), 1.f) * 8.f;
            nvgBeginPath(vg);
            nvgCircle(vg, mx, Y0 + TH - 5.f, 2.f);
            nvgFillColor(vg, withAlpha(colViolet(), 0.9f));
            nvgFill(vg);
        }

        nvgBeginPath(vg);
        nvgRect(vg, X0 + TW - 1.5f, Y0, 1.5f, TH);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 200));
        nvgFill(vg);

        if (frz)
            drawText(vg, X0 + TW / 2.f, Y0 + 8.f, "FROZEN", 7.f, withAlpha(colViolet(), 0.9f),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

        // output meters
        for (int ch = 0; ch < 2; ++ch) {
            float lv = std::min((ch == 0 ? mL : mR) / 5.f, 1.f);
            float x = X0 + TW + 6.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, Y0, 4.f, TH);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lv > 0.01f) {
                float h = lv * TH;
                nvgBeginPath(vg);
                nvgRect(vg, x, Y0 + TH - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, Y0 + TH, 0, Y0, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }

        screenFinish(vg, W, H);

        char buf[64];
        NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
        NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 230);

        // top row: L / R time (note division when synced), FB, MIX
        if (sync && divL >= 0)
            snprintf(buf, sizeof(buf), "L %s", SD_DIV_NAME[divL]);
        else
            snprintf(buf, sizeof(buf), "L %4.0fMS", msL);
        drawText(vg, 6.f, 7.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        if (sync && divR >= 0)
            snprintf(buf, sizeof(buf), "R %s", SD_DIV_NAME[divR]);
        else
            snprintf(buf, sizeof(buf), "R %4.0fMS", msR);
        drawText(vg, 62.f, 7.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "FB %3.0f%%", fb * 100.f);
        drawText(vg, 130.f, 7.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "MIX %3.0f%%", mixv * 100.f);
        drawText(vg, W - 6.f, 7.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        // bottom row: drive, sync / link state, ping-pong / crush
        snprintf(buf, sizeof(buf), "DRV %.1fX", drive);
        drawText(vg, 6.f, H - 6.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "%s%s", sync ? "SYNC " : "", link ? "LINK" : "");
        drawText(vg, W / 2.f, H - 6.f, buf, 7.f, withAlpha(colViolet(), 0.85f), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "%s%s", png ? "PNG " : "", crush ? "CRSH" : "");
        drawText(vg, W - 6.f, H - 6.f, buf, 7.f, withAlpha(colAccent(), 0.85f), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct SickDelayWidget : ModuleWidget
{
    SickDelayWidget(SickDelay *module)
    {
        setModule(module);
        box.size = Vec(SD_W, SD_H);

        addChild(new SDPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SDDisplay(module));

        const int ids[7] = {SickDelay::TIME_PARAM, SickDelay::TIME_R_PARAM, SickDelay::FEEDBACK_PARAM,
                            SickDelay::MIX_PARAM, SickDelay::COLOR_PARAM, SickDelay::DRIVE_PARAM,
                            SickDelay::MOD_PARAM};
        for (int i = 0; i < 7; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SD_KCOL[i % 4], SD_KROW[i / 4]), module, ids[i]));

        const int sw[4] = {SickDelay::PINGPONG_SWITCH, SickDelay::BITCRUSH_SWITCH, SickDelay::FREEZE_SWITCH, SickDelay::LINK_SWITCH};
        const int led[4] = {SickDelay::PINGPONG_LIGHT, SickDelay::BITCRUSH_LIGHT, SickDelay::FREEZE_LIGHT, SickDelay::LINK_LIGHT};
        for (int i = 0; i < 4; ++i) {
            float cx = SD_KCOL[i];
            addParam(createParamCentered<CKSS>(Vec(cx - 2.f, SD_SWY), module, sw[i]));
            if (i == 0 || i == 3)
                addChild(createLightCentered<SmallLight<SDCyanLight>>(Vec(cx + 12.f, SD_SWY - 13.f), module, led[i]));
            else
                addChild(createLightCentered<SmallLight<SDVioletLight>>(Vec(cx + 12.f, SD_SWY - 13.f), module, led[i]));
        }

        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdJx(0), SD_JY), module, SickDelay::LEFT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdJx(1), SD_JY), module, SickDelay::RIGHT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdJx(2), SD_JY), module, SickDelay::TIME_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdJx(3), SD_JY), module, SickDelay::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdJx(4), SD_JY), module, SickDelay::FREEZE_CV_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(sdJx(5), SD_JY), module, SickDelay::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(sdJx(6), SD_JY), module, SickDelay::OUT_R_OUTPUT));
    }
};

Model *modelSickDelay = createModel<SickDelay, SickDelayWidget>("SickDelay");