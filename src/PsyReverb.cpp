#include "YouTubeLoad.hpp"
#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include "Knob_arcs.hpp"

// ============================================================
// PsyReverb
// ============================================================
//
// OVERVIEW
// --------
// PsyReverb is a 14 HP Freeverb-style stereo reverb. The wet path is eight
// parallel damped comb filters feeding four series Schroeder all-passes,
// with the total reverb made mono-shared between L and R for a wide, stable
// image. The four main parameters (Wet, Dry, Room size, Damping) each have
// a dedicated CV input and bipolar attenuverter, and the small display shows
// the decay envelope, the damping response, and I/O level meters.
//
// Signal flow:
//   in L/R -> dry gain -> out
//              \-> 8 damped combs (feedback from Room size)
//                    -> 4 Schroeder all-passes
//                          -> wet gain -> out
//
// File structure:
//   1. Layout constants (14 HP = 210x380 px).
//   2. PsyReverbEngine: the Freeverb DSP (comb + all-pass, damping, mono
//      shared wet output).
//   3. PsyReverb module: parameter / CV handling, meter and display state.
//   4. Anonymous-namespace UI: PRPanel and PRDisplay.
//   5. PsyReverbWidget: places controls, jacks and the display.
//   6. Model registration.
// ============================================================

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const float PR_W = RACK_GRID_WIDTH * 14; // 210
static const float PR_H = RACK_GRID_HEIGHT;     // 380

// Display (170 x 114): header, decay field, damping field, axis row, footer
static const float PR_DX = 20.f, PR_DY = 34.f, PR_DW = 170.f, PR_DH = 114.f;

// Control rows: header labels at y = 160, then 4 plates of 32 px
static const float PR_HDR_Y = 160.f;
static const float PR_ROW[4] = {168.f, 204.f, 240.f, 276.f}; // top of each control row
static const float PR_ROWH = 32.f;
static const float PR_CVX = 100.f, PR_ATX = 137.f, PR_KX = 173.f;

// I/O cells: label on top, jack below
static const float PR_IO_TOP = 314.f, PR_IO_H = 42.f, PR_IO_W = 44.f;
// Returns the horizontal centre of I/O cell i (0..3): two ins, two outs.
static float prIoX(int i) { return 33.75f + 47.5f * i; }
// Returns the jack centre y for the I/O row.
static float prIoY() { return PR_IO_TOP + 28.f; }

// ------------------------------------------------------------
// Freeverb-style reverb (no runaway gain)
// ------------------------------------------------------------
// A compact Freeverb-style stereo reverb engine: eight damped comb filters
// and four series Schroeder all-passes. The wet output is mono-shared
// between L and R, which gives a wide, stable tail and avoids energy
// build-up between channels.
struct PsyReverbEngine
{
    // Simple circular delay line with read/write, reset and explicit
    // allocation / free.
    struct DelayLine
    {
        float *buffer = nullptr;
        int length = 0;
        int idx = 0;

        // Allocates the delay line to `len` samples (frees any previous
        // buffer first) and zero-initialises it.
        void init(int len)
        {
            destroy(); // re-init no longer leaks
            length = std::max(len, 1);
            buffer = new float[length]();
            idx = 0;
        }
        // Frees the underlying buffer (safe to call multiple times).
        void destroy()
        {
            delete[] buffer;
            buffer = nullptr;
        }
        // Returns the sample at the read head.
        float read() const { return buffer[idx]; }
        // Writes `value` at the read head and advances the head (wrapping).
        void write(float value)
        {
            buffer[idx] = value;
            idx = (idx + 1) % length;
        }
        // Zeroes the buffer contents and resets the read/write head.
        void reset()
        {
            if (buffer)
            {
                for (int i = 0; i < length; i++)
                    buffer[i] = 0.0f;
            }
            idx = 0;
        }
    };

    float sampleRate = 44100.0f;
    float wet = 0.5f;
    float dry = 0.5f;
    float roomsize = 0.8f;
    float damping = 0.5f;

    DelayLine comb[8];
    DelayLine allpass[4];
    float combLowpassState[8] = {};

    // Constructor: does nothing (buffers are allocated in init()).
    PsyReverbEngine() {}
    // Destructor: frees all comb and all-pass delay buffers.
    ~PsyReverbEngine()
    {
        for (int i = 0; i < 8; i++)
            comb[i].destroy();
        for (int i = 0; i < 4; i++)
            allpass[i].destroy();
    }

    // (Re)allocates the eight comb lines and four all-pass lines for the
    // given sample rate, using Freeverb's fixed delay times, and resets the
    // damping state.
    void init(float sr)
    {
        sampleRate = sr;

        int combDelays[8] = {
            (int)(0.0297f * sr), (int)(0.0371f * sr), (int)(0.0411f * sr), (int)(0.0437f * sr),
            (int)(0.0489f * sr), (int)(0.0557f * sr), (int)(0.0623f * sr), (int)(0.0697f * sr)};
        int apDelays[4] = {
            (int)(0.0051f * sr), (int)(0.0072f * sr), (int)(0.0103f * sr), (int)(0.0149f * sr)};

        for (int i = 0; i < 8; i++)
        {
            comb[i].init(combDelays[i]);
            combLowpassState[i] = 0.0f;
        }
        for (int i = 0; i < 4; i++)
            allpass[i].init(apDelays[i]);
    }

    // Sets the room size (0..1), which linearly raises the comb feedback.
    void setRoomsize(float value) { roomsize = value; }
    // Sets the damping (0..1), which lowers the comb low-pass cutoff.
    void setDamping(float value) { damping = value; }

    // One-pole low-pass used inside each comb's feedback loop. Cutoffs at or
    // above 20 kHz bypass the filter entirely.
    inline float lowpass(float in, float &state, float cutoff)
    {
        if (cutoff >= 19999.0f)
            return in;
        float coeff = 1.0f - std::exp(-2.0f * float(M_PI) * cutoff / sampleRate);
        state = state + coeff * (in - state);
        return state;
    }

    // allpass: feedback uses the output sample, not the delayed sample
    // Processes one sample through a fixed-gain Schroeder all-pass. The
    // all-pass feedback uses the all-pass output rather than the delayed
    // sample, which keeps the structure well-behaved at high gains.
    inline float processAllpass(float in, DelayLine &delay, float g)
    {
        float d = delay.read();
        float out = -in + d;
        delay.write(in + out * g);
        return out;
    }

    // Runs one stereo sample through the full reverb. The wet signal is
    // generated from the sum of eight damped combs into four all-passes, is
    // shared between L and R, then mixed with the dry input and clamped to
    // ±10 V.
    void processStereo(float leftIn, float rightIn, float &leftOut, float &rightOut)
    {
        float leftReverb = 0.0f;
        float dampFreq = 20000.0f - damping * 19900.0f;

        for (int i = 0; i < 8; i++)
        {
            float del = comb[i].read();
            del = lowpass(del, combLowpassState[i], dampFreq);
            float fb = 0.5f + roomsize * 0.48f; // max 0.98
            comb[i].write(leftIn + del * fb);
            leftReverb += del;
        }
        leftReverb = leftReverb / 8.0f;

        for (int i = 0; i < 4; i++)
            leftReverb = processAllpass(leftReverb, allpass[i], 0.7f);

        float rightReverb = leftReverb; // mono reverb, same on both channels

        float wetGain = 1.0f;
        leftOut = (dry * leftIn) + (wet * leftReverb * wetGain);
        rightOut = (dry * rightIn) + (wet * rightReverb * wetGain);

        leftOut = math::clamp(leftOut, -10.0f, 10.0f);
        rightOut = math::clamp(rightOut, -10.0f, 10.0f);
    }

    // Convenience overload for mono input: feeds the same signal to both
    // channels of processStereo().
    void processMono(float in, float &outL, float &outR)
    {
        processStereo(in, in, outL, outR);
    }
};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct PsyReverb : Module
{
    enum ParamId
    {
        PARAM_WET,
        PARAM_DRY,
        PARAM_ROOMSIZE,
        PARAM_DAMPING,
        PARAM_ATTEN_WET,
        PARAM_ATTEN_DRY,
        PARAM_ATTEN_ROOMSIZE,
        PARAM_ATTEN_DAMPING,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_LEFT,
        INPUT_RIGHT,
        CV_WET,
        CV_DRY,
        CV_ROOMSIZE,
        CV_DAMPING,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_LEFT,
        OUTPUT_RIGHT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    PsyReverbEngine reverb;
    dsp::ClockDivider paramDivider;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float meterIn = 0.f, meterOutL = 0.f, meterOutR = 0.f;

    // Constructor: configures the four main parameters, their four CV
    // attenuverters, the six inputs, the stereo outputs and the bypass, then
    // initialises the parameter divider and the reverb at the current sample
    // rate.
    PsyReverb()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(PARAM_WET, 0.f, 1.f, 0.5f, "Wet", "%", 0.f, 100.f);
        configParam(PARAM_DRY, 0.f, 1.f, 0.5f, "Dry", "%", 0.f, 100.f);
        configParam(PARAM_ROOMSIZE, 0.f, 1.f, 0.8f, "Room size", "%", 0.f, 100.f);
        configParam(PARAM_DAMPING, 0.f, 1.f, 0.5f, "Damping", "%", 0.f, 100.f);

        configParam(PARAM_ATTEN_WET, -1.f, 1.f, 0.f, "Wet CV attenuverter", "%", 0.f, 100.f);
        configParam(PARAM_ATTEN_DRY, -1.f, 1.f, 0.f, "Dry CV attenuverter", "%", 0.f, 100.f);
        configParam(PARAM_ATTEN_ROOMSIZE, -1.f, 1.f, 0.f, "Room size CV attenuverter", "%", 0.f, 100.f);
        configParam(PARAM_ATTEN_DAMPING, -1.f, 1.f, 0.f, "Damping CV attenuverter", "%", 0.f, 100.f);

        configInput(INPUT_LEFT, "Left (mono if the right input is empty)");
        configInput(INPUT_RIGHT, "Right");
        configInput(CV_WET, "Wet CV (10 V = 100%)");
        configInput(CV_DRY, "Dry CV (10 V = 100%)");
        configInput(CV_ROOMSIZE, "Room size CV (10 V = 100%)");
        configInput(CV_DAMPING, "Damping CV (10 V = 100%)");
        configOutput(OUTPUT_LEFT, "Left");
        configOutput(OUTPUT_RIGHT, "Right");
        configBypass(INPUT_LEFT, OUTPUT_LEFT);
        configBypass(INPUT_RIGHT, OUTPUT_RIGHT);

        paramDivider.setDivision(32);
        onSampleRateChange();
    }

    // Rack sample-rate-change hook: reallocates the reverb's delay lines to
    // match the engine's new sample rate.
    void onSampleRateChange() override
    {
        reverb.init(APP->engine->getSampleRate());
    }

    // Main audio callback: updates the four wet/dry/room/damping parameters
    // (with CV and attenuverters) every 32 samples, runs the reverb on the
    // stereo (or mono-normalled) input, writes the outputs, and decays the
    // three meter levels for the display.
    void process(const ProcessArgs &args) override
    {
        if (paramDivider.process())
        {
            // Reads one knob + CV*attenuverter value, clamped to 0..1.
            auto compute = [this](ParamId param, ParamId atten, InputId cv)
            {
                float base = params[param].getValue();
                float attenVal = params[atten].getValue();
                float cvVal = inputs[cv].getVoltage() / 10.f;
                return clamp(base + cvVal * attenVal, 0.f, 1.f);
            };

            reverb.wet = compute(PARAM_WET, PARAM_ATTEN_WET, CV_WET);
            reverb.dry = compute(PARAM_DRY, PARAM_ATTEN_DRY, CV_DRY);
            reverb.setRoomsize(compute(PARAM_ROOMSIZE, PARAM_ATTEN_ROOMSIZE, CV_ROOMSIZE));
            reverb.setDamping(compute(PARAM_DAMPING, PARAM_ATTEN_DAMPING, CV_DAMPING));
        }

        float leftIn = inputs[INPUT_LEFT].getVoltageSum();
        float rightIn = inputs[INPUT_RIGHT].getNormalVoltage(leftIn);
        float leftOut = 0.f, rightOut = 0.f;

        if (inputs[INPUT_RIGHT].isConnected())
            reverb.processStereo(leftIn, rightIn, leftOut, rightOut);
        else
            reverb.processMono(leftIn, leftOut, rightOut);

        outputs[OUTPUT_LEFT].setVoltage(leftOut);
        outputs[OUTPUT_RIGHT].setVoltage(rightOut);

        float md = 1.f - 5.f * args.sampleTime;
        meterIn = std::max(0.5f * (std::fabs(leftIn) + std::fabs(rightIn)), meterIn * md);
        meterOutL = std::max(std::fabs(leftOut), meterOutL * md);
        meterOutR = std::max(std::fabs(rightOut), meterOutR * md);
    }
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace
{
    namespace ag = agmm;

    // ------------------------------------------------------------
    // Panel: title, display frame, labelled control rows, I/O cells
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the CV/ATT/KNOB column
    // headers, the four full-width control-row plates with their labels, and
    // the four I/O cells (outputs tinted).
    struct PRPanel : TransparentWidget
    {
        PRPanel() { box.size = Vec(PR_W, PR_H); }

        // Draws all static panel art: base panel, bezel, column headers, the
        // four control rows (WET/DRY/ROOM/DAMP) and the four I/O cells.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, PR_W, PR_H, "PSYREVERB");
            ag::bezel(vg, PR_DX, PR_DY, PR_DW, PR_DH);

            // Column headers
            ag::text(vg, PR_CVX, PR_HDR_Y, "CV", 7.f, ag::accent(), ag::ALIGN_C, 1.f);
            ag::text(vg, PR_ATX, PR_HDR_Y, "ATT", 7.f, ag::accent(), ag::ALIGN_C, 1.f);
            ag::text(vg, PR_KX, PR_HDR_Y, "KNOB", 7.f, ag::accent(), ag::ALIGN_C, 1.f);

            // Control rows: full-width plate, name on the left
            const char *names[4] = {"WET", "DRY", "ROOM", "DAMP"};
            for (int i = 0; i < 4; ++i)
            {
                ag::plate(vg, PR_W / 2.f, PR_ROW[i], 190.f, PR_ROWH);
                ag::text(vg, 20.f, PR_ROW[i] + PR_ROWH / 2.f, names[i], 8.f, ag::textCol(),
                         ag::ALIGN_L, 1.5f);
            }

            // I/O
            const char *io[4] = {"L IN", "R IN", "L OUT", "R OUT"};
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, prIoX(i), PR_IO_TOP, PR_IO_W, PR_IO_H, io[i], i >= 2);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display rows (display-local): header 0-15, decay caption 22, decay field 27-55,
    // damping caption 63, damping field 68-86, axis labels 93, footer 107
    // ------------------------------------------------------------
    // Live display: draws the RT60 decay envelope and the one-pole damping
    // response (both derived from the current parameters), the I/O/L/R
    // level meters, and header/footer readouts. Uses sample values in the
    // module browser.
    struct PRDisplay : TransparentWidget
    {
        PsyReverb *module = nullptr;
        PRDisplay(PsyReverb *m) : module(m)
        {
            box.pos = Vec(PR_DX, PR_DY);
            box.size = Vec(PR_DW, PR_DH);
        }

        static constexpr float FX = 8.f, FW = 120.f;
        static constexpr float DY = 27.f, DH = 28.f; // decay field
        static constexpr float PY = 68.f, PH = 18.f; // damping field
        static constexpr float MX = 138.f;           // first meter

        // Draws a horizontal divider inside the display at y.
        static void hairline(NVGcontext *vg, float y)
        {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, y);
            nvgLineTo(vg, PR_DW - 8.f, y);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Maps a frequency f (Hz) to an x pixel on the damping plot's
        // 20 Hz..20 kHz logarithmic axis.
        static float dampX(float f) { return FX + FW * (std::log10(std::max(f, 20.f) / 20.f) / 3.f); }

        // Static layer: fills the display screen, draws the header/footer
        // dividers, the two plot frames, their grids, and all captions and
        // axis labels (frequency labels below the damping plot, meter
        // channel labels below the meters).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, PR_DW, PR_DH);
            hairline(vg, 15.5f);
            hairline(vg, 100.5f);

            ag::plotFrame(vg, FX, DY, FW, DH);
            ag::plotFrame(vg, FX, PY, FW, PH);

            // grid
            nvgBeginPath(vg);
            for (int t = 1; t < 4; ++t)
            {
                float x = FX + FW * t / 4.f + 0.5f;
                nvgMoveTo(vg, x, DY + 1.f);
                nvgLineTo(vg, x, DY + DH - 1.f);
            }
            const float fq[3] = {100.f, 1000.f, 10000.f};
            for (int d = 0; d < 3; ++d)
            {
                float x = dampX(fq[d]) + 0.5f;
                nvgMoveTo(vg, x, PY + 1.f);
                nvgLineTo(vg, x, PY + PH - 1.f);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // captions above the fields, frequency labels below the damping field
            ag::text(vg, FX, 22.f, "DECAY 0-8S", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            ag::text(vg, FX, 63.f, "DAMPING", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            ag::text(vg, dampX(100.f), 93.f, "100", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, dampX(1000.f), 93.f, "1K", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, dampX(10000.f), 93.f, "10K", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            // meter channel labels
            ag::text(vg, MX + 2.5f, 93.f, "I", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, MX + 10.5f, 93.f, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, MX + 18.5f, 93.f, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            Widget::draw(args);
        }

        // Layer-1 overlay: estimates RT60 from the current room size and
        // draws the decay envelope (with an RT60 marker), the one-pole
        // damping response (with a cutoff marker), the three level meters,
        // and the header/footer readouts.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = PR_DW;

            float wet = 0.5f, dry = 0.5f, room = 0.8f, damp = 0.5f, mIn = 2.f, mL = 2.8f, mR = 2.8f;
            if (module)
            {
                wet = module->reverb.wet;
                dry = module->reverb.dry;
                room = module->reverb.roomsize;
                damp = module->reverb.damping;
                mIn = module->meterIn;
                mL = module->meterOutL;
                mR = module->meterOutR;
            }

            // RT60 estimate from the comb feedback and the mean comb length
            const float meanDelay = 0.04478f;
            float fb = 0.5f + room * 0.48f;
            float dbLoop = -20.f * std::log10(std::max(fb, 0.001f));
            float rt60 = dbLoop > 1e-6f ? 60.f * meanDelay / dbLoop : 99.f;
            float cutoff = 20000.f - damp * 19900.f;

            // decay curve
            {
                nvgBeginPath(vg);
                nvgMoveTo(vg, FX, DY + DH - 1.f);
                for (int i = 0; i <= (int)FW; ++i)
                {
                    float t = 8.f * i / FW;
                    nvgLineTo(vg, FX + i, DY + DH - 1.f - std::exp(-6.9078f * t / rt60) * (DH - 2.f));
                }
                nvgLineTo(vg, FX + FW, DY + DH - 1.f);
                nvgClosePath(vg);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.2f));
                nvgFill(vg);

                nvgBeginPath(vg);
                for (int i = 0; i <= (int)FW; ++i)
                {
                    float t = 8.f * i / FW;
                    float y = DY + DH - 1.f - std::exp(-6.9078f * t / rt60) * (DH - 2.f);
                    if (i == 0)
                        nvgMoveTo(vg, FX + i, y);
                    else
                        nvgLineTo(vg, FX + i, y);
                }
                nvgStrokeColor(vg, ag::bright());
                nvgStrokeWidth(vg, 1.3f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                if (rt60 < 8.f)
                {
                    float mx = FX + FW * rt60 / 8.f;
                    nvgBeginPath(vg);
                    for (float y = DY + 1.f; y < DY + DH - 1.f; y += 4.f)
                    {
                        nvgMoveTo(vg, mx + 0.5f, y);
                        nvgLineTo(vg, mx + 0.5f, y + 2.f);
                    }
                    nvgStrokeColor(vg, ag::withA(ag::violet(), 0.9f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
            }

            // damping response (one-pole, log 20 Hz..20 kHz, 0..-36 dB)
            {
                nvgBeginPath(vg);
                for (int i = 0; i <= (int)FW; ++i)
                {
                    float f = 20.f * std::pow(1000.f, (float)i / FW);
                    float db = -10.f * std::log10(1.f + (f / cutoff) * (f / cutoff));
                    float t = std::min(std::max(-db / 36.f, 0.f), 1.f);
                    float y = PY + 1.f + t * (PH - 2.f);
                    if (i == 0)
                        nvgMoveTo(vg, FX + i, y);
                    else
                        nvgLineTo(vg, FX + i, y);
                }
                nvgStrokeColor(vg, nvgRGB(0xd8, 0xb0, 0xff));
                nvgStrokeWidth(vg, 1.2f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                float cxm = dampX(cutoff);
                nvgBeginPath(vg);
                nvgRect(vg, cxm, PY, 1.f, PH);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.7f));
                nvgFill(vg);
            }

            // meters: in, L, R (spanning both fields)
            ag::meter(vg, MX, DY, 5.f, PY + PH - DY, std::min(mIn / 5.f, 1.f));
            ag::meter(vg, MX + 8.f, DY, 5.f, PY + PH - DY, std::min(mL / 5.f, 1.f));
            ag::meter(vg, MX + 16.f, DY, 5.f, PY + PH - DY, std::min(mR / 5.f, 1.f));

            // header + footer readouts
            char buf[48];
            snprintf(buf, sizeof(buf), "ROOM %.0f%%", room * 100.f);
            ag::text(vg, 8.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "WET %.0f%%", wet * 100.f);
            ag::text(vg, W - 8.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            snprintf(buf, sizeof(buf), "RT60 %.1fS", rt60);
            ag::text(vg, 8.f, 107.f, buf, 7.f, nvgRGB(0xff, 0xe6, 0x80), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "DRY %.0f%%", dry * 100.f);
            ag::text(vg, W - 8.f, 107.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Module widget: adds the static panel, screws, the live display, the four
// control rows (CV jack + attenuverter + knob) and the four I/O jacks.
// ============================================================
struct PsyReverbWidget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, display, the four
    // WET/DRY/ROOM/DAMP control rows, and the four I/O jacks (outputs
    // tinted by the panel art).
    PsyReverbWidget(PsyReverb *module)
    {
        setModule(module);
        box.size = Vec(PR_W, PR_H);

        addChild(new PRPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PRDisplay(module));

        const int knobs[4] = {PsyReverb::PARAM_WET, PsyReverb::PARAM_DRY, PsyReverb::PARAM_ROOMSIZE, PsyReverb::PARAM_DAMPING};
        const int attens[4] = {PsyReverb::PARAM_ATTEN_WET, PsyReverb::PARAM_ATTEN_DRY, PsyReverb::PARAM_ATTEN_ROOMSIZE, PsyReverb::PARAM_ATTEN_DAMPING};
        const int cvs[4] = {PsyReverb::CV_WET, PsyReverb::CV_DRY, PsyReverb::CV_ROOMSIZE, PsyReverb::CV_DAMPING};
        for (int i = 0; i < 4; ++i)
        {
            float cy = PR_ROW[i] + PR_ROWH / 2.f;
            addInput(createInputCentered<DarkPJ301MPort>(Vec(PR_CVX, cy), module, cvs[i]));
            addParam(createParamCentered<Trimpot>(Vec(PR_ATX, cy), module, attens[i]));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(PR_KX, cy), module, knobs[i]));
        }
        agmm::addKnobArcs(this, 2.f);
        const float y = prIoY();
        addInput(createInputCentered<DarkPJ301MPort>(Vec(prIoX(0), y), module, PsyReverb::INPUT_LEFT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(prIoX(1), y), module, PsyReverb::INPUT_RIGHT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(prIoX(2), y), module, PsyReverb::OUTPUT_LEFT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(prIoX(3), y), module, PsyReverb::OUTPUT_RIGHT));
    }

    void appendContextMenu(Menu *menu) override
    {
        agmm::addYouTubeLink(menu); // adds "Visit my youtube channel!"
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        // ... your other menu items ...
    }
};

Model *modelPsyReverb = createModel<PsyReverb, PsyReverbWidget>("PsyReverb");