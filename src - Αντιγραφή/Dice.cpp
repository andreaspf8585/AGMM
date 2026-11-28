#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include "Knob_arcs.hpp"
#include <algorithm>

// ============================================================
// Dice
// ============================================================
//
// -----------------------------------------------------------------------------
// OVERVIEW
// -----------------------------------------------------------------------------
// Dice is a 16 HP polyphonic random-gate / random-V-Oct generator. On every
// rising clock edge it rolls a die per voice: with the PROBABILITY setting a
// voice fires, opening its gate for the configured gate time and latching a
// new random voltage. That voltage is emitted both raw (RANDOM output) and
// quantized to a selectable scale (QUANT output), making the module usable as
// a random sequencer, a probabilistic trigger source, or a quantized random
// melody generator.
//
// File structure:
//   1. Layout constants (16 HP = 240x380 px).
//   2. Scale table (diceScales()) with 60+ named scales.
//   3. Dice module: state, config, quantization, random generation, JSON,
//      and process().
//   4. Anonymous-namespace UI: DCPanel (static art) and DCDisplay (live
//      plot, header and footer rows).
//   5. DiceWidget: places controls, LEDs and the context menu.
//   6. Model registration.
// -----------------------------------------------------------------------------

// ------------------------------------------------------------
// Layout (16 HP = 240 x 380)
// ------------------------------------------------------------
static const float DC_W = RACK_GRID_WIDTH * 16; // 240
static const float DC_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it)
static const float DC_DX = 20.f, DC_DY = 32.f, DC_DW = 200.f, DC_DH = 100.f;
static const float DC_CHW = 12.5f; // 16 x 12.5 = 200
// Rows inside the display
static const float DC_HEAD_Y = 9.f;  // header text row
static const float DC_PLOT_Y = 18.f; // plot top
static const float DC_PLOT_H = 64.f; // plot height
static const float DC_FOOT_Y = 91.f; // footer text row

// LED strip
static const float DC_LED_TOP = 146.f, DC_LED_H = 16.f;
static const float DC_LEDY = DC_LED_TOP + DC_LED_H / 2.f;

// Controls
static const float DC_SEC1_Y = 182.f;
static const float DC_CTL_TOP = 194.f, DC_CTL_H = 66.f, DC_CTL_W = 88.f;
static const float DC_KY = DC_CTL_TOP + 40.f; // knob centre (label above, knob 38 px)
static const float DC_KX0 = 68.f, DC_KX1 = 172.f;

// I/O
static const float DC_SEC2_Y = 278.f;
static const float DC_IO_TOP = 290.f, DC_IO_H = 62.f, DC_IO_W = 48.f;
static const float DC_IOY = DC_IO_TOP + 38.f; // jack centre (label above, jack 24 px)
// Horizontal centre of I/O jack i (0..3): clock in then 3 outputs.
static float dcIoX(int i) { return 36.f + 56.f * i; }

// ------------------------------------------------------------
// Scales
// ------------------------------------------------------------
// One named quantization scale: a display name plus its semitone intervals
// from the root.
struct DiceScale
{
    std::string name;
    std::vector<float> intervals; // semitone intervals from root
};

// Returns the static list of all built-in quantization scales. Built once
// on first call and reused thereafter.
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
// Module (DSP unchanged)
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

    // Constructor: configures the two knobs, clock input, three outputs and
    // the 16 per-channel gate lights.
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

    // Quantizes `voltage` (in V/Oct) to the nearest note of `scale`. Works in
    // semitone space: splits into an octave plus a fractional part, finds the
    // closest scale degree (also considering the wrap to the next octave),
    // and returns the quantized voltage.
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

    // Rolls a new random voltage for `channel` according to the selected
    // range, stores it in the raw output buffer, and stores the quantized
    // version of it for the quantized output.
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

    // Rack reset hook: restores gate time, random range and scale to their
    // defaults, then defers to the base reset.
    void onReset(const ResetEvent &e) override
    {
        gateTime = 0.01f;
        randomRange = 0;
        selectedScale = 0;
        Module::onReset(e);
    }

    // Serialises the gate time, random range and selected scale so patches
    // reopen with the same settings.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "gateTime", json_real(gateTime));
        json_object_set_new(root, "randomRange", json_integer(randomRange));
        json_object_set_new(root, "selectedScale", json_integer(selectedScale));
        return root;
    }

    // Restores the gate time, random range and selected scale from a saved
    // patch, clamping each to its valid range.
    void dataFromJson(json_t *root) override
    {
        json_t *gt = json_object_get(root, "gateTime");
        if (gt && json_is_number(gt))
            gateTime = clamp((float)json_number_value(gt), 0.001f, 1.f);
        json_t *rr = json_object_get(root, "randomRange");
        if (rr && json_is_integer(rr))
            randomRange = clamp((int)json_integer_value(rr), 0, 3);
        json_t *ss = json_object_get(root, "selectedScale");
        if (ss && json_is_integer(ss))
            selectedScale = clamp((int)json_integer_value(ss), 0, (int)diceScales().size() - 1);
    }

    // Main callback: detects clock rising edges, rolls the die per voice,
    // decrements gate timers, clears unused channels, writes the three
    // polyphonic outputs, and updates the LED flash envelopes.
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

// ------------------------------------------------------------
// UI
// ------------------------------------------------------------
namespace
{

    // Static artwork: base panel, display bezel, LED strip plate, and the
    // CONTROL and I/O section frames.
    struct DCPanel : TransparentWidget
    {
        DCPanel() { box.size = Vec(DC_W, DC_H); }

        // Draws the static panel artwork: base panel, display bezel, LED
        // strip plate, CONTROL section with its two cells, and the I/O
        // section with its four jack cells (outputs tinted).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, DC_W, DC_H, "DICE");
            agmm::bezel(vg, DC_DX, DC_DY, DC_DW, DC_DH);

            // gate LED strip
            nvgBeginPath(vg);
            nvgRoundedRect(vg, DC_DX, DC_LED_TOP, DC_DW, DC_LED_H, 5.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            agmm::sectionHeader(vg, DC_W, DC_SEC1_Y, "CONTROL");
            agmm::cell(vg, DC_KX0, DC_CTL_TOP, DC_CTL_W, DC_CTL_H, "POLYPHONY");
            agmm::cell(vg, DC_KX1, DC_CTL_TOP, DC_CTL_W, DC_CTL_H, "PROBABILITY");

            agmm::sectionHeader(vg, DC_W, DC_SEC2_Y, "I / O");
            agmm::cell(vg, dcIoX(0), DC_IO_TOP, DC_IO_W, DC_IO_H, "CLK", false);
            agmm::cell(vg, dcIoX(1), DC_IO_TOP, DC_IO_W, DC_IO_H, "GATES", true);
            agmm::cell(vg, dcIoX(2), DC_IO_TOP, DC_IO_W, DC_IO_H, "RANDOM", true);
            agmm::cell(vg, dcIoX(3), DC_IO_TOP, DC_IO_W, DC_IO_H, "QUANT", true);
            Widget::draw(args);
        }
    };

    // Live display: 16 per-channel bars (quantized filled, raw as a tick),
    // plus a header row (scale / range) and footer row (poly / prob / gate).
    struct DCDisplay : TransparentWidget
    {
        Dice *module = nullptr;
        DCDisplay(Dice *m) : module(m)
        {
            box.pos = Vec(DC_DX, DC_DY);
            box.size = Vec(DC_DW, DC_DH);
        }

        // Static layer: fills the display area with the flat screen colour.
        void draw(const DrawArgs &args) override
        {
            agmm::screen(args.vg, 0.f, 0.f, DC_DW, DC_DH);
            Widget::draw(args);
        }

        // Layer-1 overlay: draws the plot frame, channel columns, voltage
        // grid, quantized bars and raw ticks, plus the header and footer
        // text rows. Falls back to a sample scene in the module browser.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = DC_DW;
            const float mid = DC_PLOT_Y + DC_PLOT_H / 2.f;
            const float amp = DC_PLOT_H / 2.f - 3.f; // pixels for 5 V

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
            poly = clamp(poly, 1, 16);
            scaleIdx = clamp(scaleIdx, 0, (int)diceScales().size() - 1);
            range = clamp(range, 0, 3);

            // ---- plot area (its own frame, no text inside) ----
            agmm::plotFrame(vg, 0.5f, DC_PLOT_Y, W - 1.f, DC_PLOT_H);

            // channel columns
            for (int c = 0; c < 16; ++c)
            {
                nvgBeginPath(vg);
                nvgRect(vg, c * DC_CHW + 1.5f, DC_PLOT_Y + 1.f, DC_CHW - 3.f, DC_PLOT_H - 2.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 8));
                nvgFill(vg);
            }
            // grid: +-2.5 V and 0 V
            nvgBeginPath(vg);
            for (int k = -1; k <= 1; ++k)
            {
                float y = std::floor(mid - k * amp / 2.f) + 0.5f;
                nvgMoveTo(vg, 1.f, y);
                nvgLineTo(vg, W - 1.f, y);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // bars
            for (int c = 0; c < 16; ++c)
            {
                float x = c * DC_CHW + 1.5f, w = DC_CHW - 3.f;
                if (c >= poly)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 2.f, mid, w - 4.f, 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 30));
                    nvgFill(vg);
                    continue;
                }
                float f = clamp(fl[c], 0.f, 1.f);

                // quantized value: bar from zero line
                float yq = mid - clamp(quant[c] / 5.f, -1.f, 1.f) * amp;
                float top = std::min(mid, yq);
                float h = std::max(std::fabs(mid - yq), 1.5f);
                nvgBeginPath(vg);
                nvgRect(vg, x, top, w, h);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (int)(90 + 150 * f)));
                nvgFill(vg);

                // raw value: violet tick
                float yr = mid - clamp(raw[c] / 5.f, -1.f, 1.f) * amp;
                nvgBeginPath(vg);
                nvgRect(vg, x - 0.5f, yr - 0.5f, w + 1.f, 1.f);
                nvgFillColor(vg, agmm::violet());
                nvgFill(vg);
            }

            // ---- header row: scale (left), range (right) ----
            static const char *rangeNames[4] = {"0/+1V", "-1/+1V", "-2/+2V", "-5/+5V"};
            char buf[64];
            snprintf(buf, sizeof(buf), "RNG %s", rangeNames[range]);
            const float rngW = agmm::textWidth(vg, buf, 7.f);
            agmm::text(vg, W - 6.f, DC_HEAD_Y, buf, 7.f, agmm::dim(), agmm::ALIGN_R);
            // scale name: shorten if it would touch the range readout
            std::string sname = diceScales()[scaleIdx].name;
            const float maxW = W - 12.f - rngW - 8.f;
            while (sname.size() > 3 && agmm::textWidth(vg, sname.c_str(), 7.f) > maxW)
                sname.pop_back();
            agmm::text(vg, 6.f, DC_HEAD_Y, sname.c_str(), 7.f, agmm::accent(), agmm::ALIGN_L);

            // ---- footer row: poly / prob / gate ----
            snprintf(buf, sizeof(buf), "POLY %d", poly);
            agmm::text(vg, 6.f, DC_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_L);
            snprintf(buf, sizeof(buf), "PROB %.0f%%", prob * 100.f);
            agmm::text(vg, W / 2.f, DC_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_C);
            snprintf(buf, sizeof(buf), "GATE %.0fMS", gateMs);
            agmm::text(vg, W - 6.f, DC_FOOT_Y, buf, 7.f, agmm::dim(), agmm::ALIGN_R);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Module widget: adds the panel, screws, display, two knobs, the four
// I/O jacks, the 16 gate LEDs, and the context menu (gate time, random
// range, quantization scale).
// ============================================================
struct DiceWidget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, live display, two knobs,
    // the clock input and three outputs, and the 16 gate LEDs.
    DiceWidget(Dice *module)
    {
        setModule(module);
        box.size = Vec(DC_W, DC_H);

        addChild(new DCPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new DCDisplay(module));

        addParam(createParamCentered<RoundBlackKnob>(Vec(DC_KX0, DC_KY), module, Dice::PARAM_POLYPHONY));
        addParam(createParamCentered<RoundBlackKnob>(Vec(DC_KX1, DC_KY), module, Dice::PARAM_PROBABILITY));

        addInput(createInputCentered<DarkPJ301MPort>(Vec(dcIoX(0), DC_IOY), module, Dice::INPUT_CLOCK));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(dcIoX(1), DC_IOY), module, Dice::OUTPUT_GATES));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(dcIoX(2), DC_IOY), module, Dice::OUTPUT_VOCT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(dcIoX(3), DC_IOY), module, Dice::OUTPUT_QUANTIZED));

        for (int i = 0; i < 16; ++i)
            addChild(createLightCentered<SmallLight<agmm::CyanLight>>(
                Vec(DC_DX + (i + 0.5f) * DC_CHW, DC_LEDY), module, Dice::LIGHT_GATES + i));
        agmm::addKnobArcs(this, 6.f);
            }

    // Adds the right-click menu: "Gate time", "Random V/Oct range" and
    // "Quantization scale" submenus, each with a checkmark on the active
    // option.
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
                                                 sub->addChild(createMenuItem(N[i], CHECKMARK(std::fabs(module->gateTime - T[i]) < 1e-6f),
                                                                              [=]()
                                                                              { module->gateTime = T[i]; }));
                                         }));

        menu->addChild(createSubmenuItem("Random V/Oct range", "",
                                         [=](Menu *sub)
                                         {
                                             static const char *N[4] = {"0 to +1 V", "-1 to +1 V", "-2 to +2 V", "-5 to +5 V"};
                                             for (int i = 0; i < 4; ++i)
                                                 sub->addChild(createMenuItem(N[i], CHECKMARK(module->randomRange == i),
                                                                              [=]()
                                                                              { module->randomRange = i; }));
                                         }));

        menu->addChild(createSubmenuItem("Quantization scale", "",
                                         [=](Menu *sub)
                                         {
                                             const auto &sc = diceScales();
                                             for (int i = 0; i < (int)sc.size(); ++i)
                                                 sub->addChild(createMenuItem(sc[i].name, CHECKMARK(module->selectedScale == i),
                                                                              [=]()
                                                                              { module->selectedScale = i; }));
                                         }));
    }
};

Model *modelDice = createModel<Dice, DiceWidget>("Dice");