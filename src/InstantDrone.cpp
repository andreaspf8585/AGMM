#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout (20 HP = 300 x 380)
// Every control sits in a plate with its label above it; CV jacks are labelled "CV".
// ------------------------------------------------------------
static const int DR_SCOPE_N = 55;
static const float DR_PANEL_W = RACK_GRID_WIDTH * 20; // 300
static const float DR_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Top section
static const float DR_TOP_Y = 28.f;
static const float DR_INT_CX = 40.f, DR_INT_W = 64.f, DR_INT_H = 86.f;     // large knob, label +10, knob centre +54
static const float DR_GAIN_CX = 254.f, DR_GAIN_W = 76.f, DR_GAIN_H = 44.f; // small knob centre +28
static const float DR_OUT_TOP = 78.f, DR_OUT_W = 34.f, DR_OUT_H = 38.f;    // jack centre +25
static const float DR_OUT_CX[2] = {236.f, 272.f};

// Display (bezel is drawn 4 px around it). Rows: scope + meters | divider | readouts
static const float DR_DISP_X = 84.f, DR_DISP_Y = 34.f, DR_DISP_W = 120.f, DR_DISP_H = 70.f;
static const float DR_SCOPE_W = 100.f, DR_SCOPE_H = 56.f;
static const float DR_FOOT_Y = 64.f;

// Oscillators: column header text, then three rows of plates
static const float DR_SEC_Y = 124.f;
static const float DR_HEAD_Y = 137.f;
static const float DR_ROW_TOP[3] = {145.f, 213.f, 281.f};
static const float DR_PLATE_H = 66.f, DR_PLATE_W = 39.f;
static const float DR_KNOB_DY = 17.f;                                      // knob centre below plate top
static const float DR_CVLBL_DY = 36.f;                                     // "CV" caption
static const float DR_JACK_DY = 52.f;                                      // jack centre
static const float DR_WAVE_CX = 24.f, DR_WAVE_W = 40.f;                    // waveform column
static const float DR_COLX[6] = {67.f, 108.f, 149.f, 190.f, 231.f, 272.f}; // FREQ DRIFT MIX FILTER SEND ENV

// ------------------------------------------------------------
// Multi-tap delay for dense reverb
// ------------------------------------------------------------
struct DroneDelay
{
    float buffer[44100] = {0.f};
    int writePos = 0;

    float process(float input, float feedback, int length)
    {
        length = clamp(length, 1, 44099);
        int readPos = (writePos - length + 44100) % 44100;
        float output = buffer[readPos];
        buffer[writePos] = input + output * feedback;
        writePos = (writePos + 1) % 44100;
        return output;
    }
};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct InstantDrone : Module
{
    enum ParamId
    {
        // Oscillator parameters
        FREQ1_PARAM,
        FREQ2_PARAM,
        FREQ3_PARAM,
        WAVE1_PARAM,
        WAVE2_PARAM,
        WAVE3_PARAM,
        MIX1_PARAM,
        MIX2_PARAM,
        MIX3_PARAM,
        DRIFT1_PARAM,
        DRIFT2_PARAM,
        DRIFT3_PARAM,
        SEND1_PARAM,
        SEND2_PARAM,
        SEND3_PARAM,
        FILT1_PARAM,
        FILT2_PARAM,
        FILT3_PARAM,
        REVERB_WET_PARAM,
        OUTPUT_GAIN_PARAM,
        // Envelope bypass switches (0=off, 1=on)
        ENV1_SWITCH,
        ENV2_SWITCH,
        ENV3_SWITCH,
        PARAMS_LEN
    };
    enum InputId
    {
        VOCT1_INPUT,
        VOCT2_INPUT,
        VOCT3_INPUT,
        DRIFT1_CV,
        DRIFT2_CV,
        DRIFT3_CV,
        MIX1_CV,
        MIX2_CV,
        MIX3_CV,
        FILT1_CV,
        FILT2_CV,
        FILT3_CV,
        SEND1_CV,
        SEND2_CV,
        SEND3_CV,
        ENV1_INPUT,
        ENV2_INPUT,
        ENV3_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        LEFT_OUTPUT,
        RIGHT_OUTPUT,
        OUTPUTS_LEN
    };

    // Oscillator state
    float phases[3] = {0.f, 0.f, 0.f};
    float driftValues[3] = {0.f, 0.f, 0.f};
    float driftTargets[3] = {0.f, 0.f, 0.f};

    // Per-oscillator lowpass filters
    float lowpass1 = 0.f, lowpass2 = 0.f, lowpass3 = 0.f;

    DroneDelay delayL[4];
    DroneDelay delayR[4];

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float scopeMin[DR_SCOPE_N] = {};
    float scopeMax[DR_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    InstantDrone()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);

        for (int i = 0; i < 3; i++)
        {
            configParam(FREQ1_PARAM + i, -3.f, 3.f, 0.f, string::f("Osc %d frequency", i + 1), " oct");
            configSwitch(WAVE1_PARAM + i, 0.f, 2.f, 0.f, string::f("Osc %d waveform", i + 1),
                         {"Sine", "Triangle", "Saw"});
            configParam(MIX1_PARAM + i, 0.f, 1.f, 0.5f, string::f("Osc %d mix", i + 1), "%", 0.f, 100.f);
            configParam(DRIFT1_PARAM + i, 0.f, 1.f, 0.2f, string::f("Osc %d drift", i + 1), "%", 0.f, 100.f);
            configParam(SEND1_PARAM + i, 0.f, 1.f, 0.5f, string::f("Osc %d reverb send", i + 1), "%", 0.f, 100.f);
            configParam(FILT1_PARAM + i, 0.f, 1.f, 0.6f, string::f("Osc %d filter cutoff", i + 1), "%", 0.f, 100.f);

            configInput(VOCT1_INPUT + i, string::f("Osc %d V/Oct", i + 1));
            configInput(DRIFT1_CV + i, string::f("Osc %d drift CV", i + 1));
            configInput(MIX1_CV + i, string::f("Osc %d mix CV", i + 1));
            configInput(FILT1_CV + i, string::f("Osc %d filter CV", i + 1));
            configInput(SEND1_CV + i, string::f("Osc %d send CV", i + 1));
            configInput(ENV1_INPUT + i, string::f("Osc %d envelope", i + 1));
        }
        configParam(REVERB_WET_PARAM, 0.f, 1.f, 0.7f, "Drone intensity", "%", 0.f, 100.f);
        configParam(OUTPUT_GAIN_PARAM, -10.f, 10.f, 0.f, "Output gain", " dB");

        // Envelope bypass switches (0=off / bypassed, 1=on / active)
        configSwitch(ENV1_SWITCH, 0.f, 1.f, 0.f, "Osc 1 envelope bypass", {"Off (bypassed)", "On (active)"});
        configSwitch(ENV2_SWITCH, 0.f, 1.f, 0.f, "Osc 2 envelope bypass", {"Off (bypassed)", "On (active)"});
        configSwitch(ENV3_SWITCH, 0.f, 1.f, 0.f, "Osc 3 envelope bypass", {"Off (bypassed)", "On (active)"});

        configOutput(LEFT_OUTPUT, "Left");
        configOutput(RIGHT_OUTPUT, "Right");
    }

    float getWaveform(float phase, int type)
    {
        switch (type)
        {
        case 0:
            return std::sin(2.f * M_PI * phase);
        case 1:
            return 1.f - 4.f * std::fabs(phase - 0.5f);
        case 2:
            return 2.f * phase - 1.f;
        default:
            return std::sin(2.f * M_PI * phase);
        }
    }

    void process(const ProcessArgs &args) override
    {
        float drySum = 0.f;
        float revInput = 0.f;

        for (int i = 0; i < 3; i++)
        {
            // ---- Drift random walk ----
            if (random::uniform() < 0.002f)
            {
                driftTargets[i] = (random::uniform() - 0.5f) * 2.0f;
            }
            const float driftSlew = 0.01f;
            driftValues[i] += (driftTargets[i] - driftValues[i]) * driftSlew;

            // ---- Pitch (V/oct + knob) ----
            float pitch = params[FREQ1_PARAM + i].getValue() + inputs[VOCT1_INPUT + i].getNormalVoltage(0.f);

            // ---- Drift amount (knob + CV, 0..1) ----
            float driftKnob = params[DRIFT1_PARAM + i].getValue();
            float driftCV = inputs[DRIFT1_CV + i].getNormalVoltage(0.f) / 5.f;
            float driftAmount = clamp(driftKnob + driftCV, 0.f, 1.f);
            pitch += driftValues[i] * driftAmount;

            // ---- Frequency & phase ----
            float freq = 261.626f * std::pow(2.f, pitch);
            phases[i] += freq * args.sampleTime;
            if (phases[i] >= 1.f)
                phases[i] -= 1.f;

            // ---- Waveform ----
            int waveType = (int)params[WAVE1_PARAM + i].getValue();
            float rawOsc = getWaveform(phases[i], waveType);

            // ---- Base gain (mix knob + CV) ----
            float mixKnob = params[MIX1_PARAM + i].getValue();
            float mixCV = inputs[MIX1_CV + i].getNormalVoltage(0.f) / 5.f;
            float baseGain = clamp(mixKnob + mixCV, 0.f, 1.f);

            // ---- Envelope handling (bypass switch) ----
            float envelope = 1.f; // default = no envelope (full gain)
            bool envelopeActive = params[ENV1_SWITCH + i].getValue() > 0.5f;
            if (envelopeActive)
            {
                // Envelope input: 0-10V -> 0-1 multiplier
                envelope = inputs[ENV1_INPUT + i].getNormalVoltage(0.f) / 10.f;
                envelope = clamp(envelope, 0.f, 1.f);
            }
            // Final amplitude: raw * 5V * baseGain * envelope (or 1 if bypassed)
            float oscillator = rawOsc * 5.0f * baseGain * envelope;

            // ---- Per-oscillator low-pass filter ----
            float filtKnob = params[FILT1_PARAM + i].getValue();
            float filtCV = inputs[FILT1_CV + i].getNormalVoltage(0.f) / 5.f;
            float cutoff = clamp(filtKnob + filtCV, 0.01f, 0.99f);
            float f = cutoff * 0.15f;
            float filtered;
            switch (i)
            {
            case 0:
                lowpass1 += f * (oscillator - lowpass1);
                filtered = lowpass1;
                break;
            case 1:
                lowpass2 += f * (oscillator - lowpass2);
                filtered = lowpass2;
                break;
            default:
                lowpass3 += f * (oscillator - lowpass3);
                filtered = lowpass3;
                break;
            }

            // ---- Dry sum & reverb send ----
            drySum += filtered;
            float sendKnob = params[SEND1_PARAM + i].getValue();
            float sendCV = inputs[SEND1_CV + i].getNormalVoltage(0.f) / 5.f;
            float send = clamp(sendKnob + sendCV, 0.f, 1.f);
            revInput += filtered * send;
        }

        // ---- Multi-tap drone reverb ----
        float wetL = 0.f, wetR = 0.f;
        float wetAmount = params[REVERB_WET_PARAM].getValue();
        const float fb = 0.985f;
        int timesL[] = {3431, 5743, 7853, 9437};
        int timesR[] = {3721, 5923, 8147, 9601};

        for (int i = 0; i < 4; i++)
        {
            wetL += delayL[i].process(revInput, fb, timesL[i]);
            wetR += delayR[i].process(revInput, fb, timesR[i]);
        }
        wetL /= 4.0f;
        wetR /= 4.0f;

        float outL = drySum + wetL * wetAmount;
        float outR = drySum + wetR * wetAmount;

        // ---- Output gain ----
        float gainDb = params[OUTPUT_GAIN_PARAM].getValue();
        float gainLin = std::pow(10.f, gainDb / 20.f);
        outL *= gainLin;
        outR *= gainLin;

        outputs[LEFT_OUTPUT].setVoltage(outL);
        outputs[RIGHT_OUTPUT].setVoltage(outR);

        // ---- display data ----
        float decay = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(outL), peakL * decay);
        peakR = std::max(std::fabs(outR), peakR * decay);

        float mono = 0.5f * (outL + outR);
        if (scopeCount == 0)
            scopeBMin = scopeBMax = mono;
        else
        {
            scopeBMin = std::min(scopeBMin, mono);
            scopeBMax = std::max(scopeBMax, mono);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.008f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % DR_SCOPE_N;
        }
    }
};

// ------------------------------------------------------------
// UI (anonymous namespace: names cannot clash with other module files)
// ------------------------------------------------------------
namespace
{

    // Same shapes as InstantDrone::getWaveform
    float dWave(float phase, int type)
    {
        switch (type)
        {
        case 1:
            return 1.f - 4.f * std::fabs(phase - 0.5f);
        case 2:
            return 2.f * phase - 1.f;
        default:
            return std::sin(2.f * (float)M_PI * phase);
        }
    }

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    struct DRPanelBackground : TransparentWidget
    {
        DRPanelBackground() { box.size = Vec(DR_PANEL_W, DR_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, DR_PANEL_W, DR_PANEL_H, "INSTANTDRONE");
            agmm::bezel(vg, DR_DISP_X, DR_DISP_Y, DR_DISP_W, DR_DISP_H);

            // Master section
            agmm::cell(vg, DR_INT_CX, DR_TOP_Y, DR_INT_W, DR_INT_H, "INTENSITY", false, 8.f);
            agmm::cell(vg, DR_GAIN_CX, DR_TOP_Y, DR_GAIN_W, DR_GAIN_H, "GAIN", false, 8.f);
            agmm::cell(vg, DR_OUT_CX[0], DR_OUT_TOP, DR_OUT_W, DR_OUT_H, "L OUT", true, 7.f);
            agmm::cell(vg, DR_OUT_CX[1], DR_OUT_TOP, DR_OUT_W, DR_OUT_H, "R OUT", true, 7.f);

            // Oscillators
            agmm::sectionHeader(vg, DR_PANEL_W, DR_SEC_Y, "OSCILLATORS", 10.f);
            agmm::text(vg, DR_WAVE_CX, DR_HEAD_Y, "WAVE", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            const char *heads[6] = {"FREQ", "DRIFT", "MIX", "FILTER", "SEND", "ENV"};
            for (int c = 0; c < 6; ++c)
                agmm::text(vg, DR_COLX[c], DR_HEAD_Y, heads[c], 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);

            const char *cvNames[6] = {"V/OCT", "CV", "CV", "CV", "CV", "IN"};
            for (int r = 0; r < 3; ++r)
            {
                const float top = DR_ROW_TOP[r];

                // waveform column: switch + mini screen + oscillator name
                agmm::plate(vg, DR_WAVE_CX, top, DR_WAVE_W, DR_PLATE_H, false);
                char buf[8];
                snprintf(buf, sizeof(buf), "VCO%d", r + 1);
                agmm::text(vg, DR_WAVE_CX, top + DR_JACK_DY, buf, 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);

                for (int c = 0; c < 6; ++c)
                {
                    agmm::plate(vg, DR_COLX[c], top, DR_PLATE_W, DR_PLATE_H, false);
                    agmm::text(vg, DR_COLX[c], top + DR_CVLBL_DY, cvNames[c], 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                }
            }

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Output scope + level meters, readouts in their own row
    // ------------------------------------------------------------
    struct DRDisplayWidget : TransparentWidget
    {
        InstantDrone *module = nullptr;

        DRDisplayWidget(InstantDrone *m) : module(m)
        {
            box.pos = Vec(DR_DISP_X, DR_DISP_Y);
            box.size = Vec(DR_DISP_W, DR_DISP_H);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, DR_DISP_W, DR_DISP_H);

            // Faint grid over the scope area only
            nvgBeginPath(vg);
            for (float x = 0.f; x <= DR_SCOPE_W; x += 20.f)
            {
                nvgMoveTo(vg, x + 0.5f, 0.f);
                nvgLineTo(vg, x + 0.5f, DR_SCOPE_H);
            }
            nvgMoveTo(vg, 0.f, DR_SCOPE_H / 2.f + 0.5f);
            nvgLineTo(vg, DR_SCOPE_W, DR_SCOPE_H / 2.f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Divider between the scope and the readout row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, DR_SCOPE_H + 1.5f);
            nvgLineTo(vg, DR_DISP_W, DR_SCOPE_H + 1.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
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

            const float mid = DR_SCOPE_H / 2.f;
            const float amp = mid - 2.f; // pixels for 10 V
            const float colW = DR_SCOPE_W / (float)DR_SCOPE_N;

            nvgSave(vg);
            nvgScissor(vg, 0.f, 0.f, DR_SCOPE_W, DR_SCOPE_H);
            nvgBeginPath(vg);
            for (int i = 0; i < DR_SCOPE_N; ++i)
            {
                float vmin, vmax;
                if (module)
                {
                    int idx = (module->scopeHead + i) % DR_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                }
                else
                {
                    float t = (float)i / DR_SCOPE_N;
                    float e = 6.f * (0.55f + 0.45f * std::sin(t * 11.f)) * (0.5f + 0.5f * t);
                    vmax = e;
                    vmin = -e * 0.9f;
                }
                float y0 = mid - std::min(std::max(vmax / 10.f, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / 10.f, -1.f), 1.f) * amp;
                nvgRect(vg, i * colW, y0, std::max(colW - 0.4f, 1.f), std::max(y1 - y0, 1.f));
            }
            nvgFillColor(vg, agmm::bright());
            nvgFill(vg);
            nvgRestore(vg);

            // Level meters (L / R) to the right of the scope
            for (int ch = 0; ch < 2; ++ch)
            {
                float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 5.f : 4.2f);
                agmm::meter(vg, DR_SCOPE_W + 6.f + ch * 7.f, 3.f, 5.f, DR_SCOPE_H - 6.f, peak / 10.f);
            }

            // Readout row (own row, never over the scope)
            char buf[32];
            float inten = module ? module->params[InstantDrone::REVERB_WET_PARAM].getValue() : 0.7f;
            float gain = module ? module->params[InstantDrone::OUTPUT_GAIN_PARAM].getValue() : 0.f;
            snprintf(buf, sizeof(buf), "INT %3.0f%%", inten * 100.f);
            agmm::text(vg, 4.f, DR_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "GAIN %+.1fDB", gain);
            agmm::text(vg, DR_DISP_W - 4.f, DR_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Mini waveform screen (one per oscillator): the selected shape with a dot
    // at the oscillator's live phase, name underneath
    // ------------------------------------------------------------
    struct DRWaveWidget : TransparentWidget
    {
        InstantDrone *module = nullptr;
        int idx = 0;
        static constexpr float SW = 16.f; // screen width
        static constexpr float SH = 24.f; // screen height

        DRWaveWidget(InstantDrone *m, int i, float x, float y) : module(m), idx(i)
        {
            box.pos = Vec(x, y);
            box.size = Vec(SW, 36.f);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, 0.f, 0.f, SW, SH, 3.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
            nvgStrokeWidth(vg, 1.f);
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

            int type = module ? (int)module->params[InstantDrone::WAVE1_PARAM + idx].getValue() : idx;
            type = std::min(std::max(type, 0), 2);
            float phase = module ? module->phases[idx] : 0.3f;

            const float w = SW - 4.f;
            const float mid = SH / 2.f;
            const float amp = SH / 2.f - 4.f;

            nvgBeginPath(vg);
            for (int i = 0; i <= 20; ++i)
            {
                float p = (float)i / 20.f;
                float x = 2.f + p * w;
                float y = mid - dWave(p, type) * amp;
                if (i == 0)
                    nvgMoveTo(vg, x, y);
                else
                    nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, agmm::bright());
            nvgStrokeWidth(vg, 1.f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);

            // Live phase dot
            nvgBeginPath(vg);
            nvgCircle(vg, 2.f + phase * w, mid - dWave(phase, type) * amp, 1.6f);
            nvgFillColor(vg, agmm::violet());
            nvgFill(vg);

            static const char *names[3] = {"SIN", "TRI", "SAW"};
            agmm::text(vg, SW / 2.f, SH + 8.f, names[type], 7.f, agmm::accent(), agmm::ALIGN_C, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Envelope-active LED (flat dot)
    // ------------------------------------------------------------
    struct DREnvLed : TransparentWidget
    {
        InstantDrone *module = nullptr;
        int idx = 0;

        DREnvLed(InstantDrone *m, int i, float cx, float cy) : module(m), idx(i)
        {
            box.pos = Vec(cx - 4.f, cy - 4.f);
            box.size = Vec(8.f, 8.f);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            bool on = module ? (module->params[InstantDrone::ENV1_SWITCH + idx].getValue() > 0.5f) : (idx == 1);

            nvgBeginPath(vg);
            nvgCircle(vg, 4.f, 4.f, 3.f);
            nvgFillColor(vg, on ? agmm::bright() : nvgRGBA(255, 255, 255, 28));
            nvgFill(vg);
            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct InstantDroneWidget : ModuleWidget
{
    InstantDroneWidget(InstantDrone *module)
    {
        setModule(module);
        box.size = Vec(DR_PANEL_W, DR_PANEL_H);

        addChild(new DRPanelBackground());

        // Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        // Displays (added before the controls so the controls stay on top)
        addChild(new DRDisplayWidget(module));
        for (int r = 0; r < 3; ++r)
        {
            const float top = DR_ROW_TOP[r];
            addChild(new DRWaveWidget(module, r, 26.f, top + 3.f));
            addChild(new DREnvLed(module, r, DR_COLX[5] + 11.f, top + DR_KNOB_DY));
        }

        // Master section
        addParam(createParamCentered<RoundLargeBlackKnob>(Vec(DR_INT_CX, DR_TOP_Y + 54.f), module, InstantDrone::REVERB_WET_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_GAIN_CX, DR_TOP_Y + 28.f), module, InstantDrone::OUTPUT_GAIN_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(DR_OUT_CX[0], DR_OUT_TOP + 25.f), module, InstantDrone::LEFT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(DR_OUT_CX[1], DR_OUT_TOP + 25.f), module, InstantDrone::RIGHT_OUTPUT));

        // Oscillator rows
        for (int i = 0; i < 3; ++i)
        {
            const float ky = DR_ROW_TOP[i] + DR_KNOB_DY;
            const float jy = DR_ROW_TOP[i] + DR_JACK_DY;

            // Waveform switch (the mini screen sits to its right)
            addParam(createParamCentered<CKSSThree>(Vec(14.5f, ky), module, InstantDrone::WAVE1_PARAM + i));

            // Freq (V/Oct), Drift, Mix, Filter, Send: knob on top, CV jack below
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[0], ky), module, InstantDrone::FREQ1_PARAM + i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[0], jy), module, InstantDrone::VOCT1_INPUT + i));

            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[1], ky), module, InstantDrone::DRIFT1_PARAM + i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[1], jy), module, InstantDrone::DRIFT1_CV + i));

            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[2], ky), module, InstantDrone::MIX1_PARAM + i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[2], jy), module, InstantDrone::MIX1_CV + i));

            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[3], ky), module, InstantDrone::FILT1_PARAM + i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[3], jy), module, InstantDrone::FILT1_CV + i));

            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[4], ky), module, InstantDrone::SEND1_PARAM + i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[4], jy), module, InstantDrone::SEND1_CV + i));

            // Envelope bypass switch (LED to its right) + envelope input
            addParam(createParamCentered<CKSS>(Vec(DR_COLX[5] - 5.f, ky), module, InstantDrone::ENV1_SWITCH + i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[5], jy), module, InstantDrone::ENV1_INPUT + i));
        }
    }
};

Model *modelInstantDrone = createModel<InstantDrone, InstantDroneWidget>("InstantDrone");