#include "plugin.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float DC_W = RACK_GRID_WIDTH * 16; // 240
static const float DC_H = RACK_GRID_HEIGHT;     // 380
static const float DC_DX = 20.f, DC_DY = 46.f, DC_DW = 200.f, DC_DH = 96.f;
static const float DC_CHW = 12.5f; // 16 x 12.5 = 200
static const float DC_LEDY = 162.f;
static const float DC_KY = 226.f;
static const float DC_IOY = 332.f;
static float dcIoX(int i) { return 36.f + 56.f * i; }

// ------------------------------------------------------------
// Scales
// ------------------------------------------------------------
struct DiceScale
{
    std::string name;
    std::vector<float> intervals; // semitone intervals from root
};

static const std::vector<DiceScale> &diceScales()
{
    static const std::vector<DiceScale> scales = {
        {"Major", {0, 2, 4, 5, 7, 9, 11}},
        {"Minor (Natural)", {0, 2, 3, 5, 7, 8, 10}},
        {"Harmonic Minor", {0, 2, 3, 5, 7, 8, 11}},
        {"Melodic Minor", {0, 2, 3, 5, 7, 9, 11}},
        {"Chromatic", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}},
        {"Pentatonic Major", {0, 2, 4, 7, 9}},
        {"Pentatonic Minor", {0, 3, 5, 7, 10}},
        {"Blues", {0, 3, 5, 6, 7, 10}},
        {"Bebop", {0, 2, 4, 5, 7, 9, 10, 11}},
        {"Dorian", {0, 2, 3, 5, 7, 9, 10}},
        {"Phrygian", {0, 1, 3, 5, 7, 8, 10}},
        {"Lydian", {0, 2, 4, 6, 7, 9, 11}},
        {"Mixolydian", {0, 2, 4, 5, 7, 9, 10}},
        {"Aeolian", {0, 2, 3, 5, 7, 8, 10}},
        {"Locrian", {0, 1, 3, 5, 6, 8, 10}},
        {"Major Pentatonic", {0, 2, 4, 7, 9}},
        {"Minor Pentatonic", {0, 3, 5, 7, 10}},
        {"Egyptian", {0, 2, 5, 7, 10}},
        {"Japanese (Hirajoshi)", {0, 2, 3, 7, 8}},
        {"Japanese (Kumoi)", {0, 2, 3, 7, 9}},
        {"Japanese (Iwato)", {0, 1, 5, 6, 10}},
        {"Hungarian Minor", {0, 2, 3, 6, 7, 8, 11}},
        {"Romanian Minor", {0, 2, 3, 6, 7, 9, 10}},
        {"Spanish (Phrygian)", {0, 1, 3, 5, 7, 8, 10}},
        {"Flamenco", {0, 1, 3, 4, 5, 7, 8, 10}},
        {"Whole Tone", {0, 2, 4, 6, 8, 10}},
        {"Diminished", {0, 2, 3, 5, 6, 8, 9, 11}},
        {"Augmented", {0, 3, 4, 7, 8, 11}},
        {"Enigmatic", {0, 1, 4, 6, 8, 10, 11}},
        {"Balinese", {0, 1, 3, 5, 7, 8, 10}},
        {"Pelog", {0, 1, 3, 4, 5, 7, 8, 10}},
        {"Slendro", {0, 3, 5, 7, 10}},
        {"Arabian", {0, 1, 4, 5, 7, 8, 11}},
        {"Persian", {0, 1, 4, 5, 6, 8, 11}},
        {"Byzantine", {0, 1, 4, 5, 7, 8, 11}},
        {"Egyptian Minor", {0, 2, 3, 6, 7, 8, 11}},
        {"Hindu", {0, 2, 4, 5, 7, 8, 10}},
        {"Oriental", {0, 1, 4, 5, 6, 9, 10}},
        {"Mysterious", {0, 1, 3, 5, 6, 7, 8, 10}},
        {"Prometheus", {0, 2, 4, 6, 9, 10}},
        {"Prometheus Neapolitan", {0, 1, 3, 6, 8, 10}},
        {"Neapolitan Major", {0, 1, 3, 5, 7, 9, 11}},
        {"Neapolitan Minor", {0, 1, 3, 5, 7, 8, 11}},
        {"Double Harmonic", {0, 1, 4, 5, 7, 8, 11}},
        {"Hungarian Gypsy", {0, 2, 3, 6, 7, 8, 11}},
        {"Ukranian Dorian", {0, 2, 3, 6, 7, 9, 10}},
        {"Marva", {0, 1, 4, 6, 7, 9, 11}},
        {"Todi", {0, 1, 3, 6, 7, 8, 11}},
        {"Purvi", {0, 1, 4, 6, 7, 8, 11}},
        {"Bhairav", {0, 1, 4, 5, 7, 8, 11}},
        {"Bilawal", {0, 2, 4, 5, 7, 9, 11}},
        {"Kafi", {0, 2, 3, 5, 7, 9, 10}},
        {"Asavari", {0, 2, 3, 5, 7, 8, 10}},
        {"C Major Blues", {0, 2, 3, 5, 7, 8, 10}},
        {"C Minor Blues", {0, 3, 5, 6, 7, 10}},
        {"Major Locrian", {0, 2, 4, 5, 6, 8, 10}},
        {"Lydian Minor", {0, 2, 4, 6, 7, 8, 10}},
        {"Lydian Augmented", {0, 2, 4, 6, 8, 9, 11}},
        {"Phrygian Dominant", {0, 1, 4, 5, 7, 8, 10}},
        {"Spanish Gypsy", {0, 1, 4, 5, 7, 8, 10}},
        {"Algerian", {0, 2, 4, 5, 7, 8, 10}}};
    return scales;
}

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Dice : Module
{
    enum ParamId
    {
        PARAM_POLYPHONY,
        PARAM_PROBABILITY,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_CLOCK,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_GATES,
        OUTPUT_VOCT,
        OUTPUT_QUANTIZED,
        OUTPUTS_LEN
    };
    enum LightId
    {
        ENUMS(LIGHT_GATES, 16),
        LIGHTS_LEN
    };

    // State
    float gateVoltages[16] = {};
    float voctVoltages[16] = {};
    float quantizedVoltages[16] = {};
    float gateTimers[16] = {};
    bool lastClockState = false;

    float gateTime = 0.01f; // seconds (default 10 ms)
    int randomRange = 0;    // 0: 0..1 V, 1: -1..1, 2: -2..2, 3: -5..5
    int selectedScale = 0;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    int activePoly = 1;
    float flash[16] = {};

    Dice()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PARAM_POLYPHONY, 1.f, 16.f, 1.f, "Polyphony", " voices");
        paramQuantities[PARAM_POLYPHONY]->snapEnabled = true;
        configParam(PARAM_PROBABILITY, 0.f, 1.f, 0.5f, "Gate probability", "%", 0.f, 100.f);
        configInput(INPUT_CLOCK, "Clock");
        configOutput(OUTPUT_GATES, "Gates");
        configOutput(OUTPUT_VOCT, "V/Oct (random)");
        configOutput(OUTPUT_QUANTIZED, "V/Oct (quantized)");
        for (int i = 0; i < 16; ++i)
            configLight(LIGHT_GATES + i, string::f("Channel %d gate", i + 1));
    }

    float quantizeToScale(float voltage, const DiceScale &scale)
    {
        float semitones = voltage * 12.0f;
        int octave = (int)std::floor(semitones);
        float fractional = semitones - octave;

        float bestMatch = 0.0f;
        float minDiff = 100.0f;

        for (float interval : scale.intervals)
        {
            float degree = interval / 12.0f;
            float diff = std::abs(fractional - degree);
            if (diff < minDiff)
            {
                minDiff = diff;
                bestMatch = degree;
            }
            diff = std::abs((fractional + 1.0f) - degree);
            if (diff < minDiff)
            {
                minDiff = diff;
                bestMatch = degree - 1.0f;
            }
        }
        return (float)(octave + (int)std::floor(bestMatch + 0.5f)) + bestMatch;
    }

    void generateRandomVoltage(int channel)
    {
        float voltage = 0.0f;
        switch (randomRange)
        {
        case 0:
            voltage = random::uniform();
            break;
        case 1:
            voltage = (random::uniform() * 2.0f) - 1.0f;
            break;
        case 2:
            voltage = (random::uniform() * 4.0f) - 2.0f;
            break;
        case 3:
            voltage = (random::uniform() * 10.0f) - 5.0f;
            break;
        }
        voctVoltages[channel] = voltage;
        const auto &sc = diceScales();
        quantizedVoltages[channel] = quantizeToScale(voltage, sc[clamp(selectedScale, 0, (int)sc.size() - 1)]);
    }

    void onReset(const ResetEvent &e) override
    {
        gateTime = 0.01f;
        randomRange = 0;
        selectedScale = 0;
        Module::onReset(e);
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "gateTime", json_real(gateTime));
        json_object_set_new(root, "randomRange", json_integer(randomRange));
        json_object_set_new(root, "selectedScale", json_integer(selectedScale));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *gt = json_object_get(root, "gateTime");
        if (gt)
            gateTime = json_real_value(gt);
        json_t *rr = json_object_get(root, "randomRange");
        if (rr)
            randomRange = clamp((int)json_integer_value(rr), 0, 3);
        json_t *ss = json_object_get(root, "selectedScale");
        if (ss)
            selectedScale = clamp((int)json_integer_value(ss), 0, (int)diceScales().size() - 1);
    }

    void process(const ProcessArgs &args) override
    {
        float clockVoltage = inputs[INPUT_CLOCK].getVoltage();
        bool clockHigh = clockVoltage >= 1.0f;
        bool risingEdge = clockHigh && !lastClockState;
        lastClockState = clockHigh;

        int polyphony = clamp((int)std::round(params[PARAM_POLYPHONY].getValue()), 1, 16);
        float probability = params[PARAM_PROBABILITY].getValue();

        if (risingEdge)
        {
            for (int c = 0; c < polyphony; ++c)
            {
                if (random::uniform() < probability)
                {
                    gateVoltages[c] = 10.0f;
                    gateTimers[c] = gateTime;
                    generateRandomVoltage(c);
                }
                else
                {
                    gateVoltages[c] = 0.0f;
                    gateTimers[c] = 0.0f;
                }
            }
        }

        for (int c = 0; c < 16; ++c)
        {
            if (gateTimers[c] > 0.0f)
            {
                gateTimers[c] -= args.sampleTime;
                if (gateTimers[c] <= 0.0f)
                    gateVoltages[c] = 0.0f;
            }
        }

        for (int c = polyphony; c < 16; ++c)
        {
            gateVoltages[c] = 0.0f;
            voctVoltages[c] = 0.0f;
            quantizedVoltages[c] = 0.0f;
            gateTimers[c] = 0.0f;
        }

        outputs[OUTPUT_GATES].setChannels(polyphony);
        outputs[OUTPUT_VOCT].setChannels(polyphony);
        outputs[OUTPUT_QUANTIZED].setChannels(polyphony);

        for (int c = 0; c < polyphony; ++c)
        {
            outputs[OUTPUT_GATES].setVoltage(gateVoltages[c], c);
            outputs[OUTPUT_VOCT].setVoltage(voctVoltages[c], c);
            outputs[OUTPUT_QUANTIZED].setVoltage(quantizedVoltages[c], c);
        }

        // LEDs and display flashes: each gate lights up and then fades,
        // so even a 5 ms gate is visible
        activePoly = polyphony;
        for (int c = 0; c < 16; ++c)
        {
            bool g = (c < polyphony) && (gateVoltages[c] > 0.f);
            if (g)
                flash[c] = 1.f;
            else
                flash[c] = std::max(0.f, flash[c] - 8.f * args.sampleTime);
            lights[LIGHT_GATES + c].setBrightness(flash[c]);
        }
    }
};

namespace
{

    NVGcolor colBgTop() { return nvgRGB(0x23, 0x26, 0x2d); }
    NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
    NVGcolor colText() { return nvgRGB(0xc9, 0xd1, 0xdc); }
    NVGcolor colDim() { return nvgRGB(0x7a, 0x85, 0x94); }
    NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
    // NVGcolor colViolet()   { return nvgRGB(0xb0, 0x5a, 0xff); }
    // NVGcolor withAlpha(NVGcolor c, float a) { c.a = a; return c; }

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

    template <typename TBase = GrayModuleLightWidget>
    struct TDCCyanLight : TBase
    {
        TDCCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
    };
    using DCCyanLight = TDCCyanLight<>;

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

    void drawPanelBase(NVGcontext *vg, float W, float H, const char *title)
    {
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
        nvgFill(vg);
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int)H; y += 3)
        {
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

    void drawBezel(NVGcontext *vg, float x, float y, float w, float h)
    {
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

    void plate(NVGcontext *vg, float x, float y, float w, float h)
    {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 6.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 80));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void ring(NVGcontext *vg, float cx, float cy, float r)
    {
        nvgBeginPath(vg);
        nvgCircle(vg, cx, cy, r);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void label(NVGcontext *vg, float x, float y, const char *t)
    {
        drawText(vg, x, y, t, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
    }

    void topCell(NVGcontext *vg, float cx, float cy, const char *t)
    {
        plate(vg, cx - 17.f, cy - 27.f, 34.f, 44.f);
        label(vg, cx, cy - 19.f, t);
    }

    void section(NVGcontext *vg, float W, float y, const char *txt)
    {
        float x1 = 14.f, x2 = W - 14.f, cx = W / 2.f;
        float w = 10.f + (float)strlen(txt) * 7.f;
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

    void screenBase(NVGcontext *vg, float W, float H)
    {
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H);
        nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 20.f, W * 0.75f, nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);
    }

    void screenFinish(NVGcontext *vg, float W, float H)
    {
        nvgBeginPath(vg);
        for (int y = 0; y < (int)H; y += 3)
            nvgRect(vg, 0, y + 1, W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, W, H * 0.3f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f, nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);
    }

    struct DCPanel : TransparentWidget
    {
        DCPanel() { box.size = Vec(DC_W, DC_H); }
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            drawPanelBase(vg, DC_W, DC_H, "DICE");
            drawBezel(vg, DC_DX, DC_DY, DC_DW, DC_DH);

            plate(vg, DC_DX, 154.f, DC_DW, 16.f); // LED plate

            section(vg, DC_W, 186.f, "CONTROL");
            ring(vg, 70.f, DC_KY, 18.f);
            ring(vg, 170.f, DC_KY, 18.f);
            label(vg, 70.f, DC_KY + 25.f, "POLYPHONY");
            label(vg, 170.f, DC_KY + 25.f, "PROBABILITY");

            section(vg, DC_W, 286.f, "I / O");
            const char *io[4] = {"CLK", "GATES", "RANDOM", "QUANT"};
            for (int i = 0; i < 4; ++i)
                topCell(vg, dcIoX(i), DC_IOY, io[i]);
            Widget::draw(args);
        }
    };

    struct DCDisplay : TransparentWidget
    {
        Dice *module = nullptr;
        DCDisplay(Dice *m) : module(m)
        {
            box.pos = Vec(DC_DX, DC_DY);
            box.size = Vec(DC_DW, DC_DH);
        }

        static constexpr float CY0 = 20.f, CH = 56.f;

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            screenBase(vg, DC_DW, DC_DH);
            for (int c = 0; c < 16; ++c)
            {
                nvgBeginPath(vg);
                nvgRoundedRect(vg, c * DC_CHW + 1.5f, CY0, DC_CHW - 3.f, CH, 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 9));
                nvgFill(vg);
            }
            float mid = CY0 + CH / 2.f;
            nvgBeginPath(vg);
            for (int k = -2; k <= 2; ++k)
            {
                float y = mid - k * (CH / 2.f) / 2.f + 0.5f;
                nvgMoveTo(vg, 0, y);
                nvgLineTo(vg, DC_DW, y);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, mid + 0.5f);
            nvgLineTo(vg, DC_DW, mid + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 45));
            nvgStroke(vg);
            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = DC_DW, H = DC_DH;
            const float mid = CY0 + CH / 2.f, amp = CH / 2.f - 2.f; // pixels for 5 V

            int poly = 10, scaleIdx = 0, range = 3;
            float quant[16], raw[16], fl[16], gateMs = 10.f, prob = 0.6f;
            if (module)
            {
                poly = module->activePoly;
                scaleIdx = module->selectedScale;
                range = module->randomRange;
                gateMs = module->gateTime * 1000.f;
                prob = module->params[Dice::PARAM_PROBABILITY].getValue();
                for (int c = 0; c < 16; ++c)
                {
                    quant[c] = module->quantizedVoltages[c];
                    raw[c] = module->voctVoltages[c];
                    fl[c] = module->flash[c];
                }
            }
            else
            {
                for (int c = 0; c < 16; ++c)
                {
                    quant[c] = 3.4f * std::sin(c * 0.85f + 0.6f);
                    raw[c] = quant[c] + 0.4f * std::sin(c * 2.3f);
                    fl[c] = (c % 3 == 0) ? 1.f : (c % 5 == 0 ? 0.5f : 0.f);
                }
            }
            poly = std::min(std::max(poly, 1), 16);
            scaleIdx = std::min(std::max(scaleIdx, 0), (int)diceScales().size() - 1);
            range = std::min(std::max(range, 0), 3);

            for (int c = 0; c < 16; ++c)
            {
                float x = c * DC_CHW + 1.5f, w = DC_CHW - 3.f;
                if (c >= poly)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 2.f, mid, w - 4.f, 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 22));
                    nvgFill(vg);
                    continue;
                }
                float f = fl[c];

                float yr = mid - std::min(std::max(raw[c] / 5.f, -1.f), 1.f) * amp;
                nvgBeginPath(vg);
                nvgRect(vg, x - 0.5f, yr, w + 1.f, 1.f);
                nvgFillColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 150));
                nvgFill(vg);

                float yq = mid - std::min(std::max(quant[c] / 5.f, -1.f), 1.f) * amp;
                float top = std::min(mid, yq);
                float h = std::max(std::fabs(mid - yq), 1.5f);
                if (f > 0.02f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x - 1.5f, top - 1.5f, w + 3.f, h + 3.f);
                    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (int)(50.f * f)));
                    nvgFill(vg);
                }
                nvgBeginPath(vg);
                nvgRect(vg, x, top, w, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, mid + amp, 0, mid - amp,
                                                   nvgRGBA(0x1f, 0x9f, 0xc0, (int)(110 + 120 * f)),
                                                   nvgRGBA(0x8a, 0xee, 0xff, (int)(110 + 145 * f))));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRect(vg, x, yq - 1.f, w, 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, (int)(110 + 145 * f)));
                nvgFill(vg);
            }

            screenFinish(vg, W, H);

            static const char *rangeNames[4] = {"0/+1V", "-1/+1V", "-2/+2V", "-5/+5V"};
            char buf[64];
            NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
            NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 230);
            drawText(vg, 6.f, 8.f, diceScales()[scaleIdx].name.c_str(), 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "RNG %s", rangeNames[range]);
            drawText(vg, W - 6.f, 8.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "POLY %d", poly);
            drawText(vg, 6.f, H - 8.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "PROB %.0f%%", prob * 100.f);
            drawText(vg, W / 2.f, H - 8.f, buf, 7.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
            snprintf(buf, sizeof(buf), "GATE %.0fMS", gateMs);
            drawText(vg, W - 6.f, H - 8.f, buf, 7.f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
            Widget::drawLayer(args, layer);
        }
    };

} // namespace

struct DiceWidget : ModuleWidget
{
    DiceWidget(Dice *module)
    {
        setModule(module);
        box.size = Vec(DC_W, DC_H);

        addChild(new DCPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new DCDisplay(module));

        addParam(createParamCentered<RoundBlackKnob>(Vec(70.f, DC_KY), module, Dice::PARAM_POLYPHONY));
        addParam(createParamCentered<RoundBlackKnob>(Vec(170.f, DC_KY), module, Dice::PARAM_PROBABILITY));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(dcIoX(0), DC_IOY), module, Dice::INPUT_CLOCK));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(dcIoX(1), DC_IOY), module, Dice::OUTPUT_GATES));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(dcIoX(2), DC_IOY), module, Dice::OUTPUT_VOCT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(dcIoX(3), DC_IOY), module, Dice::OUTPUT_QUANTIZED));

        for (int i = 0; i < 16; ++i)
            addChild(createLightCentered<SmallLight<DCCyanLight>>(
                Vec(DC_DX + (i + 0.5f) * DC_CHW, DC_LEDY), module, Dice::LIGHT_GATES + i));
    }

    void appendContextMenu(Menu *menu) override
    {
        Dice *module = dynamic_cast<Dice *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);

        menu->addChild(createSubmenuItem("Gate time", "",
                                         [=](Menu *sub)
                                         {
                                             static const float T[7] = {0.005f, 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f};
                                             static const char *N[7] = {"5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms", "500 ms"};
                                             for (int i = 0; i < 7; ++i)
                                                 sub->addChild(createMenuItem(N[i], (module->gateTime == T[i]) ? "✔" : "",
                                                                              [=]()
                                                                              { module->gateTime = T[i]; }));
                                         }));

        menu->addChild(createSubmenuItem("Random V/Oct range", "",
                                         [=](Menu *sub)
                                         {
                                             static const char *N[4] = {"0 to +1 V", "-1 to +1 V", "-2 to +2 V", "-5 to +5 V"};
                                             for (int i = 0; i < 4; ++i)
                                                 sub->addChild(createMenuItem(N[i], (module->randomRange == i) ? "✔" : "",
                                                                              [=]()
                                                                              { module->randomRange = i; }));
                                         }));

        menu->addChild(createSubmenuItem("Quantization scale", "",
                                         [=](Menu *sub)
                                         {
                                             const auto &sc = diceScales();
                                             for (int i = 0; i < (int)sc.size(); ++i)
                                                 sub->addChild(createMenuItem(sc[i].name, (module->selectedScale == i) ? "✔" : "",
                                                                              [=]()
                                                                              { module->selectedScale = i; }));
                                         }));
    }
};

Model *modelDice = createModel<Dice, DiceWidget>("Dice");