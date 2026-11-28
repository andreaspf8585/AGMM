#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include "Knob_arcs.hpp"

// ============================================================================
// SickDelay
// ============================================================================
//
// OVERVIEW
// --------
// SickDelay is a 16 HP stereo delay with independent left/right times (or
// linked), feedback, dry/wet mix, a tone (one-pole lowpass) control, drive
// saturation, modulation (LFO on both delay times), ping-pong stereo routing,
// a bitcrusher, freeze, and clock-synced delay times.
//
// The module can be clocked externally: when a clock is patched, the TIME
// knobs select note divisions (1/16, 1/8T, 1/8, 1/4T, 1/8., 1/4, 1/4., 1/2,
// 1/2., 1/1) based on the measured clock period. Without a clock, the knobs
// set the delay time directly (up to ~1.96 s at 48 kHz). The right delay time
// can either follow the left (LINK) or be independent.
//
// A 200×90 display shows the delay-line contents as a timeline, the left and
// right read taps, the "now" write position, output meters, and readouts for
// time (ms or note division), feedback, mix, drive, and the active sync/link/
// ping-pong/crush states.
//
// File structure:
//   1. Layout constants (16 HP = 240×380 px) and clock division tables.
//   2. SickDelay module: state, clock tracking, DSP process().
//   3. Anonymous-namespace UI: SDPanel (static art) and SDDisplay (live
//      timeline + readouts).
//   4. SickDelayWidget: places knobs, switches, jacks and lights.
//   5. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int SD_BINS = 82;                  // 2 px per bin
static const float SD_W = RACK_GRID_WIDTH * 16; // 240
static const float SD_H = RACK_GRID_HEIGHT;     // 380

// Display (200 x 90): header row, 164 x 46 timeline, axis row, footer row
static const float SD_DX = 20.f, SD_DY = 30.f, SD_DW = 200.f, SD_DH = 90.f;
static const float SD_TX0 = 12.f, SD_TY0 = 18.f, SD_TW = 164.f, SD_TH = 46.f;

// Control grid: 4 columns of 54 px wide cells, label at +10 from the cell top
static const float SD_KCOL[4] = {34.f, 91.f, 149.f, 206.f};
static const float SD_CELL_W = 54.f;
static const float SD_KTOP[2] = {132.f, 190.f}; // knob rows (cells 52 high, knob at +32)
static const float SD_KCELL_H = 52.f;
static const float SD_SWTOP = 248.f; // switch row (cells 50 high, switch at +31)
static const float SD_SWCELL_H = 50.f;
static const float SD_IOTOP = 306.f; // jack row (cells 48 high, jack at +30)
static const float SD_IOCELL_H = 48.f;
static const float SD_IOCELL_W = 36.f;
// Returns the horizontal centre of I/O jack i (0..5) for the bottom row.
static float sdIoX(int i) { return 24.f + 38.4f * (float)i; } // 6 jacks

// Clock sync: the clock input is treated as one pulse per quarter note (beat).
// The time knobs select one of these divisions when a clock is patched.
static const int SD_NDIV = 10;
static const float SD_DIV_MULT[SD_NDIV] = {0.25f, 1.f / 3.f, 0.5f, 2.f / 3.f, 0.75f, 1.f, 1.5f, 2.f, 3.f, 4.f};
static const char *SD_DIV_NAME[SD_NDIV] = {"1/16", "1/8T", "1/8", "1/4T", "1/8.", "1/4", "1/4.", "1/2", "1/2.", "1/1"};

// ============================================================================
// Module
// ============================================================================
// Stereo delay with independent/linked L/R times, feedback, tone, drive,
// modulation, ping-pong, bitcrusher, freeze and clock sync.
struct SickDelay : Module
{
    // New items are appended at the end of each enum so old patches keep loading.
    enum ParamId
    {
        TIME_PARAM, // left delay time
        FEEDBACK_PARAM,
        MIX_PARAM,
        COLOR_PARAM,
        DRIVE_PARAM,
        MOD_PARAM,
        PINGPONG_SWITCH,
        BITCRUSH_SWITCH,
        FREEZE_SWITCH,
        TIME_R_PARAM, // right delay time
        LINK_SWITCH,  // right time follows left time
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

    // Delay buffers (stereo, ~2 s at 48 kHz)
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
    float clockTimer = 0.f;   // seconds since the last clock edge
    float clockPeriod = 0.5f; // measured seconds per clock pulse (beat)
    bool clockHasEdge = false;
    bool clockValid = false;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float peakBins[SD_BINS] = {};
    int peakWriteBin = 0;
    int lastBin = -1;
    float dispAgeL = 0.25f, dispAgeR = 0.25f; // tap positions: 0 = now, 1 = oldest
    float dispMsL = 0.f, dispMsR = 0.f;
    int dispDivL = -1, dispDivR = -1; // -1 = free running
    bool dispSync = false;
    float dispMod = 0.f;
    float meterL = 0.f, meterR = 0.f;

    // Constructor: configures every parameter, input, output and light.
    // TIME knobs default to 0.5 (≈0.98 s at 48 kHz) and are re-interpreted as
    // note divisions when a clock is patched.
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
        configBypass(LEFT_INPUT, OUT_L_OUTPUT);
        configBypass(RIGHT_INPUT, OUT_R_OUTPUT);
        configLight(FREEZE_LIGHT, "Freeze");
        configLight(BITCRUSH_LIGHT, "Bitcrusher");
        configLight(PINGPONG_LIGHT, "Ping-pong");
        configLight(LINK_LIGHT, "Link L/R time");
    }

    // Rack reset hook: clears the clock tracking state.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        clockTrigger.reset();
        clockTimer = 0.f;
        clockHasEdge = false;
        clockValid = false;
    }

    // Main audio callback: reads inputs and controls, tracks the external
    // clock, computes L/R delay times (free or synced), reads the delay taps,
    // applies bitcrush / saturation / tone, writes back the delay lines
    // (skipped when frozen), applies ping-pong and dry/wet mix, and updates
    // the meters and display snapshot.
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
        // Computes a delay in samples from a knob (0..1) plus CV. When synced,
        // maps the knob to one of the SD_DIV_MULT note divisions and halves the
        // delay until it fits in the buffer. When free, maps directly to the
        // buffer length. Finally adds the LFO modulation and clamps to a safe
        // range.
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

        // Reads `buffer` at `delaySamples` behind `writePos` using linear
        // interpolation, wrapping around the buffer.
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
            if (bin >= SD_BINS)
                bin = SD_BINS - 1;
            if (bin != lastBin)
            {
                peakBins[bin] = 0.f;
                lastBin = bin;
            }
            float wv = 0.5f * (std::fabs(bufferL[writePosL]) + std::fabs(bufferR[writePosR]));
            if (wv > peakBins[bin])
                peakBins[bin] = wv;
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

// Anonymous namespace: UI helpers and the two widgets.
namespace
{
    namespace ag = agmm;

    // ------------------------------------------------------------
    // Panel background: title, display frame, one labelled plate per control
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the 8 knob/switch cells, the
    // switch row cells, and the six I/O jack cells (outputs tinted).
    struct SDPanel : TransparentWidget
    {
        SDPanel() { box.size = Vec(SD_W, SD_H); }

        // Draws all static panel art including section headers and cell
        // plates with labels.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, SD_W, SD_H, "SICKDELAY");
            ag::bezel(vg, SD_DX, SD_DY, SD_DW, SD_DH);

            // 7 knobs in a 4 x 2 grid; the last slot holds the LINK switch
            const char *names[8] = {"TIME L", "TIME R", "FEEDBACK", "MIX", "TONE", "DRIVE", "MOD", "LINK L/R"};
            for (int i = 0; i < 8; ++i)
                ag::cell(vg, SD_KCOL[i % 4], SD_KTOP[i / 4], SD_CELL_W, SD_KCELL_H, names[i]);

            // switch row: three switches and the freeze gate jack
            const char *sw[4] = {"PINGPONG", "CRUSH", "FREEZE", "FRZ GATE"};
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, SD_KCOL[i], SD_SWTOP, SD_CELL_W, SD_SWCELL_H, sw[i]);

            // jack row: four inputs, then the two outputs (tinted)
            const char *io[6] = {"L IN", "R IN", "TIME CV", "CLOCK", "OUT L", "OUT R"};
            for (int i = 0; i < 6; ++i)
                ag::cell(vg, sdIoX(i), SD_IOTOP, SD_IOCELL_W, SD_IOCELL_H, io[i], i >= 4, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: the delay line as a timeline, with the read taps
    // ------------------------------------------------------------
    // Live timeline display: shows the delay-line contents as mirrored bars,
    // the left/right read taps (with triangle markers and modulation wobble),
    // the "now" write position, output meters and all text readouts. Uses
    // demo data in the module browser.
    struct SDDisplay : TransparentWidget
    {
        SickDelay *module = nullptr;
        SDDisplay(SickDelay *m) : module(m)
        {
            box.pos = Vec(SD_DX, SD_DY);
            box.size = Vec(SD_DW, SD_DH);
        }

        // Static layer: fills the screen, draws the header/footer hairlines,
        // the timeline frame, the 0.5 s grid, the time ruler and meter
        // captions.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float X0 = SD_TX0, Y0 = SD_TY0, TW = SD_TW, TH = SD_TH;
            ag::screen(vg, 0.f, 0.f, SD_DW, SD_DH);
            const float mid = Y0 + TH / 2.f;

            // hairlines under the header row and above the footer row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 6.f, 13.5f);
            nvgLineTo(vg, SD_DW - 6.f, 13.5f);
            nvgMoveTo(vg, 6.f, 77.5f);
            nvgLineTo(vg, SD_DW - 6.f, 77.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, X0 - 1.f, Y0 - 1.f, TW + 2.f, TH + 2.f);

            // grid every 0.5 s (the buffer holds about 2 s, "now" is on the right)
            nvgBeginPath(vg);
            for (int k = 1; k < 4; ++k)
            {
                float x = X0 + TW - k * (TW / 4.f) + 0.5f;
                nvgMoveTo(vg, x, Y0);
                nvgLineTo(vg, x, Y0 + TH);
            }
            nvgMoveTo(vg, X0, mid + 0.5f);
            nvgLineTo(vg, X0 + TW, mid + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // time axis in its own row under the timeline
            const char *tl[5] = {"0", "0.5", "1.0", "1.5", "2.0S"};
            for (int k = 0; k <= 4; ++k)
                ag::text(vg, X0 + TW - k * (TW / 4.f), Y0 + TH + 12.f, tl[k], 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            // meter captions
            ag::text(vg, X0 + TW + 8.f, Y0 + TH + 12.f, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, X0 + TW + 14.f, Y0 + TH + 12.f, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            Widget::draw(args);
        }

        // Layer-1 overlay: draws the mirrored delay-line bars, the left (violet)
        // and right (cyan) read taps with triangle markers, the modulation
        // wobble, the "now" line, "FROZEN" label, output meters, and the
        // header/footer readouts (time/division, feedback, mix, drive, state).
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float X0 = SD_TX0, Y0 = SD_TY0, TW = SD_TW, TH = SD_TH;
            const float W = SD_DW, H = SD_DH;
            const float mid = Y0 + TH / 2.f;

            float ord[SD_BINS];
            float ageL = 0.3f, ageR = 0.2f, msL = 600.f, msR = 400.f, mod = 0.3f, mL = 3.f, mR = 2.4f;
            float fb = 0.5f, mixv = 0.5f, drive = 1.5f;
            bool frz = false, png = false, crush = false, sync = false, link = false;
            int divL = -1, divR = -1;

            if (module)
            {
                int wb = rack::math::clamp(module->peakWriteBin, 0, SD_BINS - 1);
                for (int i = 0; i < SD_BINS; ++i)
                    ord[i] = module->peakBins[(wb + 1 + i) % SD_BINS];
                ageL = module->dispAgeL;
                ageR = module->dispAgeR;
                msL = module->dispMsL;
                msR = module->dispMsR;
                divL = module->dispDivL;
                divR = module->dispDivR;
                sync = module->dispSync;
                mod = module->dispMod;
                mL = module->meterL;
                mR = module->meterR;
                fb = module->params[SickDelay::FEEDBACK_PARAM].getValue();
                mixv = module->params[SickDelay::MIX_PARAM].getValue();
                drive = module->params[SickDelay::DRIVE_PARAM].getValue() + 1.f;
                frz = module->params[SickDelay::FREEZE_SWITCH].getValue() > 0.5f;
                png = module->params[SickDelay::PINGPONG_SWITCH].getValue() > 0.5f;
                crush = module->params[SickDelay::BITCRUSH_SWITCH].getValue() > 0.5f;
                link = module->params[SickDelay::LINK_SWITCH].getValue() > 0.5f;
                // only show a division name that exists
                if (divL >= SD_NDIV)
                    divL = SD_NDIV - 1;
                if (divR >= SD_NDIV)
                    divR = SD_NDIV - 1;
            }
            else
            {
                for (int i = 0; i < SD_BINS; ++i)
                {
                    float a = (float)(SD_BINS - 1 - i);
                    ord[i] = 4.f * std::exp(-a * 0.035f) * (0.35f + 0.65f * std::fabs(std::sin(a * 0.33f)));
                }
            }

            // delay line contents (mirrored bars, newest on the right)
            nvgSave(vg);
            nvgScissor(vg, X0, Y0, TW, TH);
            const float amp = TH / 2.f - 2.f;
            nvgBeginPath(vg);
            for (int i = 0; i < SD_BINS; ++i)
            {
                float h = std::min(ord[i] / 5.f, 1.f) * amp;
                if (h < 0.4f)
                    continue;
                nvgRect(vg, X0 + i * 2.f, mid - h, 1.6f, 2.f * h);
            }
            nvgFillColor(vg, frz ? nvgRGB(0xd8, 0xb0, 0xff) : ag::bright());
            nvgFill(vg);
            nvgRestore(vg);

            // tap regions and tap markers: left = violet (top triangle), right = cyan (bottom triangle)
            float txL = X0 + (1.f - std::min(std::max(ageL, 0.f), 1.f)) * TW;
            float txR = X0 + (1.f - std::min(std::max(ageR, 0.f), 1.f)) * TW;

            nvgBeginPath(vg);
            nvgRect(vg, txL, Y0, X0 + TW - txL, TH);
            nvgFillColor(vg, ag::withA(ag::violet(), 0.08f));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, txR, Y0, X0 + TW - txR, TH);
            nvgFillColor(vg, ag::withA(ag::accent(), 0.06f));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, txL + 0.5f, Y0);
            nvgLineTo(vg, txL + 0.5f, Y0 + TH);
            nvgStrokeColor(vg, nvgRGBA(0xd8, 0xb0, 0xff, 240));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, txR + 0.5f, Y0);
            nvgLineTo(vg, txR + 0.5f, Y0 + TH);
            nvgStrokeColor(vg, nvgRGBA(0xc0, 0xf6, 0xff, 240));
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, txL - 3.5f, Y0 - 3.f);
            nvgLineTo(vg, txL + 4.5f, Y0 - 3.f);
            nvgLineTo(vg, txL + 0.5f, Y0 + 2.f);
            nvgClosePath(vg);
            nvgFillColor(vg, ag::violet());
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, txR - 3.5f, Y0 + TH + 3.f);
            nvgLineTo(vg, txR + 4.5f, Y0 + TH + 3.f);
            nvgLineTo(vg, txR + 0.5f, Y0 + TH - 2.f);
            nvgClosePath(vg);
            nvgFillColor(vg, ag::accent());
            nvgFill(vg);

            // modulation wobble marker under the left tap
            {
                float mx = txL + 0.5f + std::min(std::max(mod, -1.f), 1.f) * 8.f;
                nvgBeginPath(vg);
                nvgCircle(vg, mx, Y0 + TH - 5.f, 2.f);
                nvgFillColor(vg, ag::withA(ag::violet(), 0.9f));
                nvgFill(vg);
            }

            // "now" line
            nvgBeginPath(vg);
            nvgRect(vg, X0 + TW - 1.5f, Y0, 1.5f, TH);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 200));
            nvgFill(vg);

            if (frz)
                ag::text(vg, X0 + TW / 2.f, Y0 + 9.f, "FROZEN", 7.f, ag::withA(ag::violet(), 0.9f), ag::ALIGN_C, 3.f);

            // output meters
            ag::meter(vg, X0 + TW + 6.f, Y0, 4.f, TH, mL / 5.f);
            ag::meter(vg, X0 + TW + 12.f, Y0, 4.f, TH, mR / 5.f);

            char buf[64];
            NVGcolor c1 = ag::withA(ag::accent(), 0.85f);
            NVGcolor cd = ag::dim();

            // header row: L / R time (note division when synced), FB, MIX
            if (sync && divL >= 0)
                snprintf(buf, sizeof(buf), "L %s", SD_DIV_NAME[divL]);
            else
                snprintf(buf, sizeof(buf), "L %4.0fMS", msL);
            ag::text(vg, 6.f, 7.f, buf, 7.f, c1, ag::ALIGN_L, 0.5f);
            if (sync && divR >= 0)
                snprintf(buf, sizeof(buf), "R %s", SD_DIV_NAME[divR]);
            else
                snprintf(buf, sizeof(buf), "R %4.0fMS", msR);
            ag::text(vg, 48.f, 7.f, buf, 7.f, c1, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "FB %3.0f%%", fb * 100.f);
            ag::text(vg, 92.f, 7.f, buf, 7.f, c1, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "MIX %3.0f%%", mixv * 100.f);
            ag::text(vg, W - 6.f, 7.f, buf, 7.f, c1, ag::ALIGN_R, 0.5f);

            // footer row: drive, sync / link state, ping-pong / crush
            snprintf(buf, sizeof(buf), "DRV %.1fX", drive);
            ag::text(vg, 6.f, H - 6.f, buf, 7.f, cd, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "%s%s", sync ? "SYNC " : "", link ? "LINK" : "");
            ag::text(vg, W / 2.f, H - 6.f, buf, 7.f, ag::withA(ag::violet(), 0.85f), ag::ALIGN_C, 0.5f);
            snprintf(buf, sizeof(buf), "%s%s", png ? "PNG " : "", crush ? "CRSH" : "");
            ag::text(vg, W - 6.f, H - 6.f, buf, 7.f, ag::withA(ag::accent(), 0.85f), ag::ALIGN_R, 0.5f);
            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================================
// Module widget
// ============================================================================
// Places the static panel, screws, live timeline display, the 7 knobs, the 4
// switches (with LEDs), the freeze gate input, and the six I/O jacks.
struct SickDelayWidget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, display, knob grid,
    // switch row (with LEDs), freeze gate input and I/O jacks.
    SickDelayWidget(SickDelay *module)
    {
        setModule(module);
        box.size = Vec(SD_W, SD_H);

        addChild(new SDPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SDDisplay(module));

        // ---- 7 knobs (4 x 2 grid, knob at +32 inside its cell) ----
        const int ids[7] = {SickDelay::TIME_PARAM, SickDelay::TIME_R_PARAM, SickDelay::FEEDBACK_PARAM,
                            SickDelay::MIX_PARAM, SickDelay::COLOR_PARAM, SickDelay::DRIVE_PARAM,
                            SickDelay::MOD_PARAM};
        for (int i = 0; i < 7; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SD_KCOL[i % 4], SD_KTOP[i / 4] + 32.f), module, ids[i]));

        // ---- switches: PINGPONG, CRUSH, FREEZE in the switch row, LINK in the last knob slot ----
        const int sw[4] = {SickDelay::PINGPONG_SWITCH, SickDelay::BITCRUSH_SWITCH, SickDelay::FREEZE_SWITCH, SickDelay::LINK_SWITCH};
        const int led[4] = {SickDelay::PINGPONG_LIGHT, SickDelay::BITCRUSH_LIGHT, SickDelay::FREEZE_LIGHT, SickDelay::LINK_LIGHT};
        for (int i = 0; i < 4; ++i)
        {
            float cx = SD_KCOL[i];
            float cy = (i < 3 ? SD_SWTOP : SD_KTOP[1]) + 31.f;
            addParam(createParamCentered<CKSS>(Vec(cx - 6.f, cy), module, sw[i]));
            if (i == 0 || i == 3)
                addChild(createLightCentered<SmallLight<ag::CyanLight>>(Vec(cx + 14.f, cy), module, led[i]));
            else
                addChild(createLightCentered<SmallLight<ag::VioletLight>>(Vec(cx + 14.f, cy), module, led[i]));
        }
        addInput(createInputCentered<DarkPJ301MPort>(Vec(SD_KCOL[3], SD_SWTOP + 30.f), module, SickDelay::FREEZE_CV_INPUT));

        // ---- jack row ----
        const float jy = SD_IOTOP + 30.f;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdIoX(0), jy), module, SickDelay::LEFT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdIoX(1), jy), module, SickDelay::RIGHT_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdIoX(2), jy), module, SickDelay::TIME_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(sdIoX(3), jy), module, SickDelay::CLOCK_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(sdIoX(4), jy), module, SickDelay::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(sdIoX(5), jy), module, SickDelay::OUT_R_OUTPUT));
        agmm::addKnobArcs(this, 6.f);
    }

    void appendContextMenu(Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        // ... your other menu items ...
    }
};

Model *modelSickDelay = createModel<SickDelay, SickDelayWidget>("SickDelay");