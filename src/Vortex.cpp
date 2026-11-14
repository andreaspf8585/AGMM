// ============================================================================
// Vortex — dual-mode reverb / echo (32 HP Rack module)
// ----------------------------------------------------------------------------
// Overview:
//   * Two engines in one module: REVERB (a plate) and ECHO (a modulated
//     echo with optional ping-pong). MODE button toggles; it latches and
//     is saved with the patch.
//   * 12 parameter cells, each with a knob, CV jack and attenuverter:
//       PRE-DLY, SIZE*, DECAY*, DIFF/PING, DAMP, LOW CUT,
//       MOD SPD, MOD DEP, MIX, ECHO TIME*, ECHO FB*, ECHO LVL*.
//       (*) SIZE and DECAY are used only in reverb mode; the ECHO cell
//       group is used only in echo mode. Inactive cells are dimmed.
//   * FREEZE: latching button plus a gate input (high = freeze).
//   * CLEAR: momentary button plus a trigger input; fades the wet
//     signal out, resets the engines, then fades back in.
//   * IN/OUT stereo switches; mono/poly otherwise (up to NUM_CHANNELS).
//   * CV range menu: 0..+10 V (default) or -5..+5 V, applied to every
//     CV input (attenuverter then scales this into the knob offset).
//   * Three display modes (WAVE / BARS / SPECTRUM) drawn by VxDisplay;
//     click a tab, click the plot to cycle, or use the context menu.
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include "VortexModule.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <vector>

// ------------------------------------------------------------
// Layout constants (32 HP = 480 px)
// ------------------------------------------------------------
static const int VX_SCOPE_N = 396;              // waveform history bins (1 px each)
static const int VX_SPEC_N = 4096;              // FFT size
static const int VX_BANDS = 198;                // spectrum bands (2 px each)
static const float VX_W = RACK_GRID_WIDTH * 32; // 480
static const float VX_H = RACK_GRID_HEIGHT;     // 380

// Display (440 x 112) sits just under the title; the bezel adds 4 px around it.
static const float VX_DX = 20.f, VX_DY = 26.f, VX_DW = 440.f, VX_DH = 112.f;
static const float VX_COL[6] = {40.f, 120.f, 200.f, 280.f, 360.f, 440.f}; // knob cell centres
// Knob cells: 70 x 72, label at +10, knob at +30, CV jack (left) and attenuverter (right) at +58.
static const float VX_ROW[2] = {148.f, 224.f}; // top of each cell plate
static const float VX_CELL_W = 70.f, VX_CELL_H = 72.f;
static const float VX_KNOB_DY = 30.f, VX_SUB_DY = 58.f, VX_SUB_DX = 16.f;
// Bottom strip: 11 cells of 36 x 46, label at +10, control at +30 (ends at y = 350).
static const float VX_STRIP_Y = 304.f, VX_STRIP_W = 36.f, VX_STRIP_H = 46.f, VX_STRIP_DY = 30.f;
static float vxStripX(int i) { return 40.f + 40.f * (float)i; } // 11 items, 40 px apart

// Plot area inside the display (display-local coordinates).
static const float VP_X = 8.f, VP_Y = 18.f, VP_W = 396.f, VP_H = 76.f;
static const float VP_TABX[3] = {6.f, 46.f, 86.f};
static const float VP_TABW[3] = {36.f, 36.f, 46.f};

// ============================================================
// Module
//
// The DSP itself lives in Vortex::Voice / Vortex::Controller
// (declared in VortexModule.hpp). This file's job is:
//   * translate Rack params / CV into Vortex::Params once per sample,
//   * drive the per-voice engine,
//   * mix dry + wet with equal-power crossfade,
//   * feed display data (wave history, spectrum, peaks, readouts),
//   * persist non-param state (CV range, display mode, freeze, echo).
// ============================================================
struct VortexModule : Module
{
    // Order is unchanged from earlier versions, so saved patches keep loading.
    enum ParamId
    {
        PRE_DELAY_PARAM,
        PRE_DELAY_ATTEN,
        SIZE_PARAM,
        SIZE_ATTEN,
        DECAY_PARAM,
        DECAY_ATTEN,
        DIFFUSION_PARAM,
        DIFFUSION_ATTEN,
        HIGH_DAMP_PARAM,
        HIGH_DAMP_ATTEN,
        LOW_CUT_PARAM,
        LOW_CUT_ATTEN,
        MOD_SPEED_PARAM,
        MOD_SPEED_ATTEN,
        MOD_DEPTH_PARAM,
        MOD_DEPTH_ATTEN,
        MIX_PARAM,
        MIX_ATTEN,
        ECHO_MODE_BUTTON_PARAM,
        ECHO_TIME_PARAM,
        ECHO_TIME_ATTEN,
        ECHO_FEEDBACK_PARAM,
        ECHO_FEEDBACK_ATTEN,
        ECHO_LEVEL_PARAM,
        ECHO_LEVEL_ATTEN,
        FREEZE_BUTTON_PARAM,
        IN_STEREO_BUTTON_PARAM,
        OUT_STEREO_BUTTON_PARAM,
        CLEAR_BUTTON_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        IN_L_INPUT,
        IN_R_INPUT,
        PRE_DELAY_CV_INPUT,
        SIZE_CV_INPUT,
        DECAY_CV_INPUT,
        DIFFUSION_CV_INPUT,
        HIGH_DAMP_CV_INPUT,
        LOW_CUT_CV_INPUT,
        MOD_SPEED_CV_INPUT,
        MOD_DEPTH_CV_INPUT,
        MIX_CV_INPUT,
        ECHO_TIME_CV_INPUT,
        ECHO_FEEDBACK_CV_INPUT,
        ECHO_LEVEL_CV_INPUT,
        FREEZE_CV_INPUT,
        CLEAR_INPUT,
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
        CLEAR_BUTTON_LIGHT,
        FREEZE_BUTTON_LIGHT,
        ECHO_MODE_BUTTON_LIGHT,
        LIGHTS_LEN
    };

    // CV interpretation mode (persisted via dataToJson).
    enum CVRange
    {
        BIPOLAR_M5_5,
        UNIPOLAR_0_10
    };
    CVRange cvRangeMode = UNIPOLAR_0_10;

    // ---- engine ----
    Vortex::Voice voices[Vortex::NUM_CHANNELS];
    Vortex::Controller ctl;
    float allocSr = 0.f;              // sample rate the voices were init'd with
    float kMix = 0.001f, mixS = 0.5f; // one-pole coefficient + state for MIX smoothing

    dsp::SchmittTrigger clearButtonTrigger, clearInputTrigger;
    dsp::SchmittTrigger freezeButtonTrigger, echoModeButtonTrigger;
    bool clearRequested = false; // latched request to clear the tail
    float clearFade = 1.0f;      // 0..1 fade used during a clear
    bool clearActive = false;    // true while fading back in after a clear
    int limiterRecovery = 0;     // countdown after a NaN/limiter event
    bool freezeLatched = false;  // latching freeze state (saved with the patch)
    bool echoModeState = false;  // false = reverb, true = echo (saved)

    // ---- display data (written by the audio thread, read by the UI thread;
    //      a torn read is only cosmetic) ----
    int dispMode = 0;        // 0 wave, 1 bars, 2 spectrum (saved with the patch)
    int dispCh = 1;          // current channel count shown in the header
    bool dispFreeze = false; // freeze state shown in the header
    float dispVal[12] = {};  // pre, size, decay, diff, damp, lowcut, speed, depth, mix, eTime, eFb, eLvl (with CV)
    float dispRt60 = 2.f, dispEchoMs = 300.f, dispSize = 1.f, dispSr = 48000.f;
    float peakL = 0.f, peakR = 0.f; // output peak meters (stereo)

    // Waveform history: min/max per bin, one bin per pixel; L, R and the
    // mono-input overlay are stored separately. scopeHead is the newest bin.
    float scopeMinL[VX_SCOPE_N] = {}, scopeMaxL[VX_SCOPE_N] = {};
    float scopeMinR[VX_SCOPE_N] = {}, scopeMaxR[VX_SCOPE_N] = {};
    float scopeMinI[VX_SCOPE_N] = {}, scopeMaxI[VX_SCOPE_N] = {};
    int scopeHead = 0, scopeCount = 0, scopeBinSamples = 485;
    // Running accumulators between bin commits.
    float accMinL = 1e9f, accMaxL = -1e9f, accMinR = 1e9f, accMaxR = -1e9f, accMinI = 1e9f, accMaxI = -1e9f;

    // Circular buffer feeding the FFT (power-of-two size).
    float specBuf[VX_SPEC_N] = {};
    int specPos = 0;

    // --------------------------------------------------------
    // Constructor — configure params, inputs, outputs, lights.
    // Param displays are wired via displayBase/displayMultiplier
    // so the UI shows real units (ms, s, Hz, %).
    // --------------------------------------------------------
    VortexModule()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(PRE_DELAY_PARAM, 0.f, 1.f, 0.f, "Pre-delay", " ms", 0.f, 500.f);
        configParam(PRE_DELAY_ATTEN, -1.f, 1.f, 0.f, "Pre-delay CV attenuverter", "%", 0.f, 100.f);
        configParam(SIZE_PARAM, 0.f, 1.f, 0.6f, "Size (room scale, reverb only)", "%", 0.f, 150.f, 25.f);
        configParam(SIZE_ATTEN, -1.f, 1.f, 0.f, "Size CV attenuverter", "%", 0.f, 100.f);
        configParam(DECAY_PARAM, 0.f, 1.f, 0.5f, "Decay (RT60, reverb only)", " s", 100.f, 0.2f);
        configParam(DECAY_ATTEN, -1.f, 1.f, 0.f, "Decay CV attenuverter", "%", 0.f, 100.f);
        configParam(DIFFUSION_PARAM, 0.f, 1.f, 0.7f, "Diffusion (echo: ping-pong amount)", "%", 0.f, 100.f);
        configParam(DIFFUSION_ATTEN, -1.f, 1.f, 0.f, "Diffusion CV attenuverter", "%", 0.f, 100.f);
        configParam(HIGH_DAMP_PARAM, 0.f, 1.f, 0.5f, "Damping cutoff (turn up = darker; also the echo feedback filter)", " Hz", 2500.f / 18000.f, 18000.f);
        configParam(HIGH_DAMP_ATTEN, -1.f, 1.f, 0.f, "Damping CV attenuverter", "%", 0.f, 100.f);
        configParam(LOW_CUT_PARAM, 0.f, 1.f, 0.f, "Low cut", " Hz", 0.f, 480.f, 20.f);
        configParam(LOW_CUT_ATTEN, -1.f, 1.f, 0.f, "Low cut CV attenuverter", "%", 0.f, 100.f);
        configParam(MOD_SPEED_PARAM, 0.f, 1.f, 0.3f, "Modulation speed", " Hz", 0.f, 4.9f, 0.1f);
        configParam(MOD_SPEED_ATTEN, -1.f, 1.f, 0.f, "Modulation speed CV attenuverter", "%", 0.f, 100.f);
        configParam(MOD_DEPTH_PARAM, 0.f, 1.f, 0.2f, "Modulation depth (echo: wow)", "%", 0.f, 100.f);
        configParam(MOD_DEPTH_ATTEN, -1.f, 1.f, 0.f, "Modulation depth CV attenuverter", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/wet", "%", 0.f, 100.f);
        configParam(MIX_ATTEN, -1.f, 1.f, 0.f, "Mix CV attenuverter", "%", 0.f, 100.f);

        configParam(ECHO_TIME_PARAM, 0.f, 1.f, 0.3f, "Echo time", " ms", 0.f, 1000.f);
        configParam(ECHO_TIME_ATTEN, -1.f, 1.f, 0.f, "Echo time CV attenuverter", "%", 0.f, 100.f);
        configParam(ECHO_FEEDBACK_PARAM, 0.f, 1.f, 0.5f, "Echo feedback", "%", 0.f, 95.f);
        configParam(ECHO_FEEDBACK_ATTEN, -1.f, 1.f, 0.f, "Echo feedback CV attenuverter", "%", 0.f, 100.f);
        configParam(ECHO_LEVEL_PARAM, 0.f, 1.f, 0.8f, "Echo level", "%", 0.f, 100.f);
        configParam(ECHO_LEVEL_ATTEN, -1.f, 1.f, 0.f, "Echo level CV attenuverter", "%", 0.f, 100.f);

        configButton(ECHO_MODE_BUTTON_PARAM, "Reverb / echo mode");
        configButton(FREEZE_BUTTON_PARAM, "Freeze (latching, the gate input also freezes)");
        configButton(CLEAR_BUTTON_PARAM, "Clear tails");
        configSwitch(IN_STEREO_BUTTON_PARAM, 0.f, 1.f, 0.f, "Stereo input (L/R into one voice)", {"Off (mono / poly)", "On"});
        configSwitch(OUT_STEREO_BUTTON_PARAM, 0.f, 1.f, 0.f, "Stereo output (voices summed to L/R)", {"Off (mono / poly)", "On"});

        configInput(IN_L_INPUT, "Left / poly");
        configInput(IN_R_INPUT, "Right");
        configInput(PRE_DELAY_CV_INPUT, "Pre-delay CV");
        configInput(SIZE_CV_INPUT, "Size CV");
        configInput(DECAY_CV_INPUT, "Decay CV");
        configInput(DIFFUSION_CV_INPUT, "Diffusion CV");
        configInput(HIGH_DAMP_CV_INPUT, "Damping CV");
        configInput(LOW_CUT_CV_INPUT, "Low cut CV");
        configInput(MOD_SPEED_CV_INPUT, "Modulation speed CV");
        configInput(MOD_DEPTH_CV_INPUT, "Modulation depth CV");
        configInput(MIX_CV_INPUT, "Mix CV");
        configInput(ECHO_TIME_CV_INPUT, "Echo time CV");
        configInput(ECHO_FEEDBACK_CV_INPUT, "Echo feedback CV");
        configInput(ECHO_LEVEL_CV_INPUT, "Echo level CV");
        configInput(FREEZE_CV_INPUT, "Freeze gate");
        configInput(CLEAR_INPUT, "Clear trigger");

        configOutput(OUT_L_OUTPUT, "Left / poly");
        configOutput(OUT_R_OUTPUT, "Right");

        configLight(CLEAR_BUTTON_LIGHT, "Clear");
        configLight(FREEZE_BUTTON_LIGHT, "Freeze");
        configLight(ECHO_MODE_BUTTON_LIGHT, "Echo mode");

        configBypass(IN_L_INPUT, OUT_L_OUTPUT);
        configBypass(IN_R_INPUT, OUT_R_OUTPUT);
    }

    // --------------------------------------------------------
    // Reset all engine state and clear the latched toggles.
    // Forces a re-allocation on the next sample by clearing lastSr.
    // --------------------------------------------------------
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        for (int c = 0; c < Vortex::NUM_CHANNELS; ++c)
            voices[c].reset();
        ctl.lastSr = 0.f; // snap smoothers to the controls on the next sample
        clearRequested = false;
        clearFade = 1.0f;
        clearActive = false;
        limiterRecovery = 0;
        freezeLatched = false;
        echoModeState = false;
    }

    // --------------------------------------------------------
    // Persist non-param state: CV range, display mode, freeze latch, echo mode.
    // --------------------------------------------------------
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        json_object_set_new(root, "displayMode", json_integer(dispMode));
        json_object_set_new(root, "freeze", json_boolean(freezeLatched));
        json_object_set_new(root, "echoMode", json_boolean(echoModeState));
        return root;
    }

    // --------------------------------------------------------
    // Restore the fields written by dataToJson() (with clamps).
    // --------------------------------------------------------
    void dataFromJson(json_t *root) override
    {
        json_t *j = json_object_get(root, "cvRangeMode");
        if (j)
            cvRangeMode = (json_integer_value(j) == (json_int_t)BIPOLAR_M5_5) ? BIPOLAR_M5_5 : UNIPOLAR_0_10;
        j = json_object_get(root, "displayMode");
        if (j)
            dispMode = rack::math::clamp((int)json_integer_value(j), 0, 2);
        j = json_object_get(root, "freeze");
        if (j)
            freezeLatched = json_is_true(j);
        j = json_object_get(root, "echoMode");
        if (j)
            echoModeState = json_is_true(j);
    }

    // --------------------------------------------------------
    // (Re)initialise the engines for a given sample rate.
    // Called lazily whenever the Rack sample rate changes.
    // `scopeBinSamples` sizes each wave bin so that ~99 bins/s are
    // produced (396 bins ~ 4 s of visible history).
    // --------------------------------------------------------
    void allocate(float sr)
    {
        for (int c = 0; c < Vortex::NUM_CHANNELS; ++c)
            voices[c].init(sr);
        allocSr = sr;
        dispSr = sr;
        kMix = 1.f - std::exp(-1.f / (0.02f * sr));      // 20 ms mix smoothing
        scopeBinSamples = std::max(1, (int)(sr / 99.f)); // 396 bins ~ 4 s of history
        ctl.lastSr = 0.f;
    }

    // --------------------------------------------------------
    // Read one (param, attenuverter, CV) triple and combine them.
    // cvRangeMode determines how a raw CV is interpreted:
    //   BIPOLAR_M5_5:  cv = clamp(raw/5, -1, 1)  (i.e. -5..+5 V -> -1..1)
    //   UNIPOLAR_0_10: cv = raw / 10             (0..+10 V -> 0..1)
    // Final value = clamp(knob + atten * cv, 0, 1).
    // --------------------------------------------------------
    float getParam(int paramId, int attenId, int cvId)
    {
        float knob = params[paramId].getValue();
        float atten = params[attenId].getValue();
        float cvRaw = inputs[cvId].getVoltageSum();
        float cv;
        if (cvRangeMode == BIPOLAR_M5_5)
            cv = rack::math::clamp(cvRaw / 5.f, -1.f, 1.f);
        else
            cv = cvRaw / 10.f;
        return rack::math::clamp(knob + atten * cv, 0.0f, 1.0f);
    }

    // --------------------------------------------------------
    // Main per-sample processing:
    //  1. Detect channel configuration (mono/poly/stereo-in/out).
    //  2. Read all 12 (param, atten, CV) tuples + MIX.
    //  3. Latch mode/freeze buttons; combine freeze state.
    //  4. Update the per-sample controller (smoothing), snapshot
    //     values into the display fields.
    //  5. Handle the clear fade/trigger sequence.
    //  6. Run each active voice, mix dry + wet (equal-power),
    //     sum for stereo output if requested, with NaN/limiter safety.
    //  7. Feed the display: peak meters, scope bins, FFT buffer.
    // --------------------------------------------------------
    void process(const ProcessArgs &args) override
    {
        const float sr = args.sampleRate;
        const float sampleTime = args.sampleTime;
        if (sr != allocSr)
            allocate(sr);

        // ---------------- channels ----------------
        int inLch = inputs[IN_L_INPUT].getChannels();
        int inRch = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getChannels() : 0;
        bool stIn = params[IN_STEREO_BUTTON_PARAM].getValue() > 0.5f;
        bool stOut = params[OUT_STEREO_BUTTON_PARAM].getValue() > 0.5f;
        int numCh = stIn ? 1 : rack::math::clamp(std::max(inLch, inRch), 1, Vortex::NUM_CHANNELS);
        dispCh = numCh;
        outputs[OUT_L_OUTPUT].setChannels(stOut ? 1 : numCh);
        outputs[OUT_R_OUTPUT].setChannels(stOut ? 1 : numCh);

        // ---------------- controls ----------------
        Vortex::Params p;
        p.pre = getParam(PRE_DELAY_PARAM, PRE_DELAY_ATTEN, PRE_DELAY_CV_INPUT);
        p.size = getParam(SIZE_PARAM, SIZE_ATTEN, SIZE_CV_INPUT);
        p.decay = getParam(DECAY_PARAM, DECAY_ATTEN, DECAY_CV_INPUT);
        p.diff = getParam(DIFFUSION_PARAM, DIFFUSION_ATTEN, DIFFUSION_CV_INPUT);
        p.damp = getParam(HIGH_DAMP_PARAM, HIGH_DAMP_ATTEN, HIGH_DAMP_CV_INPUT);
        p.lowcut = getParam(LOW_CUT_PARAM, LOW_CUT_ATTEN, LOW_CUT_CV_INPUT);
        p.speed = getParam(MOD_SPEED_PARAM, MOD_SPEED_ATTEN, MOD_SPEED_CV_INPUT);
        p.depth = getParam(MOD_DEPTH_PARAM, MOD_DEPTH_ATTEN, MOD_DEPTH_CV_INPUT);
        float mix = getParam(MIX_PARAM, MIX_ATTEN, MIX_CV_INPUT);
        p.eTime = getParam(ECHO_TIME_PARAM, ECHO_TIME_ATTEN, ECHO_TIME_CV_INPUT);
        p.eFb = getParam(ECHO_FEEDBACK_PARAM, ECHO_FEEDBACK_ATTEN, ECHO_FEEDBACK_CV_INPUT);
        p.eLvl = getParam(ECHO_LEVEL_PARAM, ECHO_LEVEL_ATTEN, ECHO_LEVEL_CV_INPUT);

        // Mode + freeze: the buttons latch, the freeze gate holds while high.
        if (echoModeButtonTrigger.process(params[ECHO_MODE_BUTTON_PARAM].getValue() > 0.5f ? 1.f : 0.f))
            echoModeState = !echoModeState;
        if (freezeButtonTrigger.process(params[FREEZE_BUTTON_PARAM].getValue() > 0.5f ? 1.f : 0.f))
            freezeLatched = !freezeLatched;
        bool freezeActive = freezeLatched || inputs[FREEZE_CV_INPUT].getVoltage() > 2.f;
        p.freeze = freezeActive;
        p.echo = echoModeState;
        lights[ECHO_MODE_BUTTON_LIGHT].setSmoothBrightness(echoModeState ? 1.f : 0.f, sampleTime);
        lights[FREEZE_BUTTON_LIGHT].setSmoothBrightness(freezeActive ? 1.f : 0.f, sampleTime);

        // Smoothed per-sample engine state.
        ctl.update(p, sr);
        const Vortex::Shared &sh = ctl.sh;

        // Snapshot the current control values for the display / footer.
        dispFreeze = freezeActive;
        dispVal[0] = p.pre;
        dispVal[1] = p.size;
        dispVal[2] = p.decay;
        dispVal[3] = p.diff;
        dispVal[4] = p.damp;
        dispVal[5] = p.lowcut;
        dispVal[6] = p.speed;
        dispVal[7] = p.depth;
        dispVal[8] = mix;
        dispVal[9] = p.eTime;
        dispVal[10] = p.eFb;
        dispVal[11] = p.eLvl;
        dispRt60 = ctl.rt60();
        dispEchoMs = ctl.echoMs();
        dispSize = ctl.sizeScale();

        // ---------------- clear: fade the wet signal, reset, fade back ----------------
        bool clearPressed = params[CLEAR_BUTTON_PARAM].getValue() > 0.5f;
        bool clearTrig = clearButtonTrigger.process(clearPressed ? 1.f : 0.f) ||
                         clearInputTrigger.process(inputs[CLEAR_INPUT].getVoltage());
        if (clearTrig)
            clearRequested = true;

        if (clearRequested)
        {
            // Phase 1: fade the wet signal down.
            clearFade -= sampleTime * 10.f;
            if (clearFade <= 0.0f)
            {
                for (int i = 0; i < Vortex::NUM_CHANNELS; ++i)
                    voices[i].reset();
                clearFade = 0.0f;
                clearRequested = false;
                clearActive = true;
            }
        }
        else if (clearActive)
        {
            // Phase 2: fade the (now-cleared) wet signal back in.
            clearFade += sampleTime * 10.f;
            if (clearFade >= 1.0f)
            {
                clearFade = 1.0f;
                clearActive = false;
            }
        }
        else
        {
            clearFade = 1.0f;
        }
        lights[CLEAR_BUTTON_LIGHT].setSmoothBrightness(clearPressed ? 1.0f : 0.0f, sampleTime);

        // ---------------- voices ----------------
        // Equal-power dry/wet crossfade; mix is smoothed by kMix.
        mixS += kMix * (mix - mixS);
        const float cd = std::cos(mixS * 0.5f * (float)M_PI); // equal-power dry / wet
        const float sw = std::sin(mixS * 0.5f * (float)M_PI);
        // Peak meter decay coefficient.
        const float md = 1.f - 4.f * sampleTime;

        float sumL = 0.f, sumR = 0.f, monIn = 0.f;
        for (int c = 0; c < numCh; ++c)
        {
            // Dry input: in stereo-in mode both voices see the same L/R;
            // otherwise per-channel poly inputs are used (L falls back to R).
            float dryL, dryR;
            if (stIn)
            {
                dryL = inputs[IN_L_INPUT].getVoltage();
                dryR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getVoltage() : dryL;
            }
            else
            {
                dryL = inputs[IN_L_INPUT].getPolyVoltage(c);
                dryR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getPolyVoltage(c) : dryL;
            }
            if (!std::isfinite(dryL))
                dryL = 0.f;
            if (!std::isfinite(dryR))
                dryR = 0.f;

            float wetL, wetR;
            voices[c].process(sh, dryL, dryR, wetL, wetR);

            // Safety: never let a NaN or runaway value out.
            if (!std::isfinite(wetL) || !std::isfinite(wetR))
            {
                wetL = wetR = 0.f;
                if (limiterRecovery == 0)
                {
                    for (int i = 0; i < Vortex::NUM_CHANNELS; ++i)
                        voices[i].reset();
                    limiterRecovery = (int)(sr * 0.1f);
                }
            }
            if (limiterRecovery > 0)
            {
                wetL = wetR = 0.f;
                limiterRecovery--;
            }

            // Equal-power dry/wet mix, hard-clamped to ±12 V.
            float outL = rack::math::clamp(dryL * cd + wetL * clearFade * sw, -12.f, 12.f);
            float outR = rack::math::clamp(dryR * cd + wetR * clearFade * sw, -12.f, 12.f);

            sumL += outL;
            sumR += outR;
            monIn += 0.5f * (dryL + dryR);

            if (!stOut)
            {
                outputs[OUT_L_OUTPUT].setVoltage(outL, c);
                outputs[OUT_R_OUTPUT].setVoltage(outR, c);
            }
        }
        // Stereo-out: sum all voices to L/R.
        if (stOut)
        {
            sumL = rack::math::clamp(sumL, -12.f, 12.f);
            sumR = rack::math::clamp(sumR, -12.f, 12.f);
            outputs[OUT_L_OUTPUT].setVoltage(sumL, 0);
            outputs[OUT_R_OUTPUT].setVoltage(sumR, 0);
        }

        // ---------------- display feeds ----------------
        // Stereo peak meters with one-pole decay.
        peakL = std::max(std::fabs(sumL), peakL * md);
        peakR = std::max(std::fabs(sumR), peakR * md);

        // FFT ring buffer (mono sum, power-of-two size).
        specBuf[specPos] = 0.5f * (sumL + sumR);
        specPos = (specPos + 1) & (VX_SPEC_N - 1);

        // Wave history: accumulate min/max per channel, then commit one bin
        // every `scopeBinSamples` samples.
        accMinL = std::min(accMinL, sumL);
        accMaxL = std::max(accMaxL, sumL);
        accMinR = std::min(accMinR, sumR);
        accMaxR = std::max(accMaxR, sumR);
        accMinI = std::min(accMinI, monIn);
        accMaxI = std::max(accMaxI, monIn);
        if (++scopeCount >= scopeBinSamples)
        {
            scopeCount = 0;
            scopeHead = (scopeHead + 1) % VX_SCOPE_N;
            scopeMinL[scopeHead] = accMinL;
            scopeMaxL[scopeHead] = accMaxL;
            scopeMinR[scopeHead] = accMinR;
            scopeMaxR[scopeHead] = accMaxR;
            scopeMinI[scopeHead] = accMinI;
            scopeMaxI[scopeHead] = accMaxI;
            accMinL = accMinR = accMinI = 1e9f;
            accMaxL = accMaxR = accMaxI = -1e9f;
        }
    }
};

// ============================================================
// Drawing helpers (anonymous namespace — UI-only)
// ------------------------------------------------------------
// Shared flat panel kit (agmm_ui.hpp): flat fills and hairline strokes only.
// ============================================================
namespace
{

    namespace ag = agmm;

    // ------------------------------------------------------------
    // Control cells: 12 knobs, each with a CV jack and an attenuverter.
    // group: 0 = used in both modes, 1 = reverb only, 2 = echo only
    // ------------------------------------------------------------
    struct VxCellDef
    {
        const char *name;
        int param, atten, cv, group;
    };
    const VxCellDef VX_CELLS[12] = {
        {"PRE-DLY", VortexModule::PRE_DELAY_PARAM, VortexModule::PRE_DELAY_ATTEN, VortexModule::PRE_DELAY_CV_INPUT, 0},
        {"SIZE", VortexModule::SIZE_PARAM, VortexModule::SIZE_ATTEN, VortexModule::SIZE_CV_INPUT, 1},
        {"DECAY", VortexModule::DECAY_PARAM, VortexModule::DECAY_ATTEN, VortexModule::DECAY_CV_INPUT, 1},
        {"DIFF/PING", VortexModule::DIFFUSION_PARAM, VortexModule::DIFFUSION_ATTEN, VortexModule::DIFFUSION_CV_INPUT, 0},
        {"DAMP", VortexModule::HIGH_DAMP_PARAM, VortexModule::HIGH_DAMP_ATTEN, VortexModule::HIGH_DAMP_CV_INPUT, 0},
        {"LOW CUT", VortexModule::LOW_CUT_PARAM, VortexModule::LOW_CUT_ATTEN, VortexModule::LOW_CUT_CV_INPUT, 0},
        {"MOD SPD", VortexModule::MOD_SPEED_PARAM, VortexModule::MOD_SPEED_ATTEN, VortexModule::MOD_SPEED_CV_INPUT, 0},
        {"MOD DEP", VortexModule::MOD_DEPTH_PARAM, VortexModule::MOD_DEPTH_ATTEN, VortexModule::MOD_DEPTH_CV_INPUT, 0},
        {"MIX", VortexModule::MIX_PARAM, VortexModule::MIX_ATTEN, VortexModule::MIX_CV_INPUT, 0},
        {"ECHO TIME", VortexModule::ECHO_TIME_PARAM, VortexModule::ECHO_TIME_ATTEN, VortexModule::ECHO_TIME_CV_INPUT, 2},
        {"ECHO FB", VortexModule::ECHO_FEEDBACK_PARAM, VortexModule::ECHO_FEEDBACK_ATTEN, VortexModule::ECHO_FEEDBACK_CV_INPUT, 2},
        {"ECHO LVL", VortexModule::ECHO_LEVEL_PARAM, VortexModule::ECHO_LEVEL_ATTEN, VortexModule::ECHO_LEVEL_CV_INPUT, 2},
    };

    // Strip order: buttons / switches, then inputs, then outputs (tinted).
    const char *VX_STRIP_NAMES[11] = {"MODE", "FREEZE", "CLEAR", "ST IN", "ST OUT",
                                      "L IN", "R IN", "FRZ IN", "CLR IN", "L OUT", "R OUT"};

    // ------------------------------------------------------------
    // VxPanel — static panel chrome:
    //   * panel base, bezel
    //   * 12 knob cells (label dimmed if inactive for the current mode)
    //   * bottom strip with a labelled cell per button / switch / jack
    // The actual controls are placed by VortexWidget at the same positions.
    // ------------------------------------------------------------
    struct VxPanel : TransparentWidget
    {
        VortexModule *module = nullptr;
        VxPanel(VortexModule *m) : module(m) { box.size = Vec(VX_W, VX_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, VX_W, VX_H, "VORTEX");
            ag::bezel(vg, VX_DX, VX_DY, VX_DW, VX_DH);

            // Current mode: from the module when editing, reverb in the browser.
            const bool echo = module ? module->echoModeState : false;

            // 12 knob cells in two rows of six.
            for (int i = 0; i < 12; ++i)
            {
                int row = i / 6, col = i % 6;
                float cx = VX_COL[col], T = VX_ROW[row];
                const VxCellDef &c = VX_CELLS[i];
                bool active = (c.group == 0) || (c.group == 1 && !echo) || (c.group == 2 && echo);
                NVGcolor tint = (c.group == 2) ? ag::violet() : ag::accent();

                ag::plate(vg, cx, T, VX_CELL_W, VX_CELL_H, false);
                if (active && c.group != 0)
                {
                    // mode-specific cells get a tinted hairline while active
                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, cx - VX_CELL_W / 2.f, T, VX_CELL_W, VX_CELL_H, 5.f);
                    nvgStrokeColor(vg, ag::withA(tint, 0.55f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                NVGcolor lc = active ? (c.group != 0 ? tint : ag::dim()) : nvgRGBA(0x8a, 0x95, 0xa4, 110);
                ag::text(vg, cx, T + 10.f, c.name, 7.f, lc, ag::ALIGN_C, 0.4f);
            }

            // Strip: 5 buttons / switches, 4 inputs, 2 outputs (tinted).
            for (int i = 0; i < 11; ++i)
                ag::cell(vg, vxStripX(i), VX_STRIP_Y, VX_STRIP_W, VX_STRIP_H, VX_STRIP_NAMES[i], i >= 9, 7.f);
            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // VxDisplay — live 440x112 display with three modes:
    //   WAVE     : scrolling min/max envelope (L cyan, R violet, dry white)
    //   BARS     : reverb => the 14 plate taps, echo => repeats in dB
    //   SPECTRUM : log-freq FFT with smoothing + peak-hold line
    // Interaction: click a tab (top row) to switch modes, or click the
    // plot area to cycle; also set from the context menu.
    // If module == nullptr (browser preview) synthetic demo data is drawn.
    // ------------------------------------------------------------
    struct VxDisplay : OpaqueWidget
    {
        VortexModule *module = nullptr;

        // UI-side caches (mains-thread only).
        float barPeak[Vortex::NUM_TAPS] = {};
        float specSm[VX_BANDS], specPk[VX_BANDS];
        std::vector<float> win, twC, twS, re, im, pw;
        float specDb[VX_BANDS]; // latest spectrum in dB (per widget instance)

        // Per-draw copy of the wave history (avoids racing the audio thread).
        float scMinL[VX_SCOPE_N], scMaxL[VX_SCOPE_N], scMinR[VX_SCOPE_N], scMaxR[VX_SCOPE_N];
        float scMinI[VX_SCOPE_N], scMaxI[VX_SCOPE_N];

        // --------------------------------------------------------
        // Precompute the Hann window and twiddle factors so the FFT
        // only does the actual butterfly passes at draw time.
        // --------------------------------------------------------
        VxDisplay(VortexModule *m) : module(m)
        {
            box.pos = Vec(VX_DX, VX_DY);
            box.size = Vec(VX_DW, VX_DH);

            win.resize(VX_SPEC_N);
            twC.resize(VX_SPEC_N / 2);
            twS.resize(VX_SPEC_N / 2);
            re.assign(VX_SPEC_N, 0.f);
            im.assign(VX_SPEC_N, 0.f);
            pw.assign(VX_SPEC_N / 2, 0.f);
            // Hann window.
            for (int i = 0; i < VX_SPEC_N; ++i)
                win[i] = 0.5f - 0.5f * std::cos(2.f * (float)M_PI * i / (float)VX_SPEC_N); // Hann
            // Twiddle factors for the FFT (cos/sin tables).
            for (int i = 0; i < VX_SPEC_N / 2; ++i)
            {
                twC[i] = std::cos(2.f * (float)M_PI * i / (float)VX_SPEC_N);
                twS[i] = std::sin(2.f * (float)M_PI * i / (float)VX_SPEC_N);
            }
            // Spectrum smoothing / peak-hold state (start at floor).
            for (int b = 0; b < VX_BANDS; ++b)
            {
                specSm[b] = -90.f;
                specPk[b] = -90.f;
                specDb[b] = -90.f;
            }
            for (int i = 0; i < VX_SCOPE_N; ++i)
            {
                scMinL[i] = scMinR[i] = scMinI[i] = 0.f;
                scMaxL[i] = scMaxR[i] = scMaxI[i] = 0.f;
            }
        }

        // --------------------------------------------------------
        // Click handling:
        //   * top row inside a tab rect -> switch to that mode.
        //   * click inside the plot area -> cycle to the next mode.
        // --------------------------------------------------------
        void onButton(const ButtonEvent &e) override
        {
            if (e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS)
            {
                if (module)
                {
                    if (e.pos.y < 15.f)
                    {
                        for (int i = 0; i < 3; ++i)
                            if (e.pos.x >= VP_TABX[i] && e.pos.x < VP_TABX[i] + VP_TABW[i])
                                module->dispMode = i;
                    }
                    else if (e.pos.y < VP_Y + VP_H + 2.f)
                    {
                        module->dispMode = (module->dispMode + 1) % 3;
                    }
                }
                e.consume(this);
            }
        }

        // --------------------------------------------------------
        // In-place iterative radix-2 FFT on the `re`/`im` vectors.
        // Assumes the size is a power of two (VX_SPEC_N = 4096).
        // Uses the precomputed twiddle factors in twC / twS.
        // --------------------------------------------------------
        void fft()
        {
            const int N = VX_SPEC_N;
            // Bit-reversal permutation.
            for (int i = 1, j = 0; i < N; ++i)
            {
                int bit = N >> 1;
                for (; j & bit; bit >>= 1)
                    j ^= bit;
                j ^= bit;
                if (i < j)
                {
                    std::swap(re[i], re[j]);
                    std::swap(im[i], im[j]);
                }
            }
            // Butterfly passes.
            for (int len = 2; len <= N; len <<= 1)
            {
                int half = len >> 1, step = N / len;
                for (int i = 0; i < N; i += len)
                {
                    for (int k = 0; k < half; ++k)
                    {
                        float wr = twC[k * step], wi = -twS[k * step];
                        int a = i + k, b = a + half;
                        float tr = re[b] * wr - im[b] * wi;
                        float ti = re[b] * wi + im[b] * wr;
                        re[b] = re[a] - tr;
                        im[b] = im[a] - ti;
                        re[a] += tr;
                        im[a] += ti;
                    }
                }
            }
        }

        // --------------------------------------------------------
        // Fill dbOut[VX_BANDS] (dB re 5 V peak) from the module's
        // output history:
        //   * window + FFT the most recent VX_SPEC_N samples
        //   * bin into VX_BANDS log-spaced bands from 20 Hz to Nyquist
        //   * single-bin bands use linear interpolation between bins,
        //     wider bands average their included bins
        //   * convert to dB and normalise so 0 dB = 5 V amplitude
        // --------------------------------------------------------
        void computeSpectrum(float sr, float *dbOut)
        {
            int pos = module->specPos;
            // Apply the window and zero the imaginary part.
            for (int i = 0; i < VX_SPEC_N; ++i)
            {
                re[i] = module->specBuf[(pos + i) & (VX_SPEC_N - 1)] * win[i];
                im[i] = 0.f;
            }
            fft();
            // Magnitude -> power spectrum for the first half.
            for (int k = 0; k < VX_SPEC_N / 2; ++k)
            {
                float a = std::sqrt(re[k] * re[k] + im[k] * im[k]) * 4.f / (float)VX_SPEC_N;
                pw[k] = a * a;
            }
            // Log-band aggregation.
            const float fMin = 20.f, fMax = std::min(20000.f, sr * 0.5f);
            const float binHz = sr / (float)VX_SPEC_N;
            const float ratio = fMax / fMin;
            for (int b = 0; b < VX_BANDS; ++b)
            {
                float f0 = fMin * std::pow(ratio, (float)b / VX_BANDS);
                float f1 = fMin * std::pow(ratio, (float)(b + 1) / VX_BANDS);
                float lo = f0 / binHz, hi = f1 / binHz;
                float p;
                if (hi - lo < 1.f)
                {
                    // Narrow band: linear interpolation between two bins.
                    float posb = 0.5f * (lo + hi);
                    int i0 = rack::math::clamp((int)posb, 0, VX_SPEC_N / 2 - 2);
                    float fr = posb - (float)i0;
                    p = pw[i0] * (1.f - fr) + pw[i0 + 1] * fr;
                }
                else
                {
                    // Wider band: average of the included bins.
                    int i0 = rack::math::clamp((int)std::ceil(lo), 0, VX_SPEC_N / 2 - 1);
                    int i1 = rack::math::clamp((int)std::floor(hi), i0, VX_SPEC_N / 2 - 1);
                    float s = 0.f;
                    for (int k = i0; k <= i1; ++k)
                        s += pw[k];
                    p = s / (float)(i1 - i0 + 1);
                }
                dbOut[b] = 10.f * std::log10(p / 25.f + 1e-12f); // 0 dB = 5 V amplitude
            }
        }

        // ---- static background (flat screen + plot frame) ----
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, VX_DW, VX_DH);
            ag::plotFrame(vg, VP_X - 1.f, VP_Y - 1.f, VP_W + 2.f, VP_H + 2.f);
            Widget::draw(args);
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
            const float W = VX_DW, H = VX_DH;
            const float PX = VP_X, PY = VP_Y, PW = VP_W, PH = VP_H;

            // ---------- gather data (or demo data when previewing) ----------
            int mode = 0, ch = 1;
            bool echo = false, frz = false, sIn = false, sOut = false;
            float pv[12] = {0.f, 0.6f, 0.5f, 0.7f, 0.5f, 0.f, 0.3f, 0.2f, 0.5f, 0.3f, 0.5f, 0.8f};
            float mL = 2.6f, mR = 2.2f, fade = 1.f, rt = 2.f, sr = 48000.f, size = 1.15f, eMs = 300.f;
            float tapLv[Vortex::NUM_TAPS];
            for (int i = 0; i < Vortex::NUM_TAPS; ++i)
                tapLv[i] = 0.f;

            if (module)
            {
                mode = rack::math::clamp(module->dispMode, 0, 2);
                ch = module->dispCh;
                echo = module->echoModeState;
                frz = module->dispFreeze;
                for (int i = 0; i < 12; ++i)
                    pv[i] = module->dispVal[i];
                mL = module->peakL;
                mR = module->peakR;
                fade = module->clearFade;
                rt = module->dispRt60;
                sr = module->dispSr;
                size = module->dispSize;
                eMs = module->dispEchoMs;
                sIn = module->params[VortexModule::IN_STEREO_BUTTON_PARAM].getValue() > 0.5f;
                sOut = module->params[VortexModule::OUT_STEREO_BUTTON_PARAM].getValue() > 0.5f;
                for (int i = 0; i < Vortex::NUM_TAPS; ++i)
                    tapLv[i] = module->voices[0].plate.tapLvl[i];
                if (mode == 0)
                {
                    // Copy the wave history in chronological order (oldest first).
                    int head = module->scopeHead;
                    for (int i = 0; i < VX_SCOPE_N; ++i)
                    {
                        int idx = (head + 1 + i) % VX_SCOPE_N;
                        scMinL[i] = module->scopeMinL[idx];
                        scMaxL[i] = module->scopeMaxL[idx];
                        scMinR[i] = module->scopeMinR[idx];
                        scMaxR[i] = module->scopeMaxR[idx];
                        scMinI[i] = module->scopeMinI[idx];
                        scMaxI[i] = module->scopeMaxI[idx];
                    }
                }
                else if (mode == 2)
                {
                    computeSpectrum(sr, specDb);
                }
            }
            else
            {
                // module browser preview: synthetic data
                mode = 2;
                for (int i = 0; i < Vortex::NUM_TAPS; ++i)
                    tapLv[i] = 3.f * (0.3f + 0.7f * std::fabs(std::sin((float)i * 1.7f + 0.4f)));
                for (int b = 0; b < VX_BANDS; ++b)
                    specDb[b] = -16.f - 58.f * std::pow((float)b / (float)VX_BANDS, 0.75f) + 5.f * std::sin((float)b * 0.45f);
            }

            // ---------- plot (clipped to the frame) ----------
            nvgSave(vg);
            nvgScissor(vg, PX - 1.f, PY - 1.f, PW + 2.f, PH + 2.f);
            nvgLineCap(vg, NVG_BUTT);

            if (mode == 0)
            {
                // ===== WAVE: scrolling min/max envelope (L cyan, R violet), dry input in white =====
                const float mid = PY + PH / 2.f, half = PH / 2.f - 2.f, sc = half / 5.f;

                // Grid (vertical 1/8 divisions, horizontal quarter marks).
                nvgBeginPath(vg);
                for (int k = 1; k < 8; ++k)
                {
                    float x = PX + PW - (float)k * (PW / 8.f) + 0.5f;
                    nvgMoveTo(vg, x, PY);
                    nvgLineTo(vg, x, PY + PH);
                }
                nvgMoveTo(vg, PX, mid + 0.5f);
                nvgLineTo(vg, PX + PW, mid + 0.5f);
                nvgMoveTo(vg, PX, mid - half * 0.5f + 0.5f);
                nvgLineTo(vg, PX + PW, mid - half * 0.5f + 0.5f);
                nvgMoveTo(vg, PX, mid + half * 0.5f + 0.5f);
                nvgLineTo(vg, PX + PW, mid + half * 0.5f + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // One envelope (min..max per bin) in 4 flat alpha bands (older = fainter).
                const float step = PW / (float)VX_SCOPE_N;
                auto env = [&](const float *mn, const float *mx, NVGcolor col, float baseA)
                {
                    for (int q = 0; q < 4; ++q)
                    {
                        int i0 = q * VX_SCOPE_N / 4, i1 = (q + 1) * VX_SCOPE_N / 4;
                        nvgBeginPath(vg);
                        for (int i = i0; i < i1; ++i)
                        {
                            if (mx[i] < mn[i])
                                continue; // empty bin
                            float hi = rack::math::clamp(mx[i] * sc, -half, half);
                            float lo = rack::math::clamp(mn[i] * sc, -half, half);
                            float hh = std::max(hi - lo, 1.f);
                            nvgRect(vg, PX + (float)i * step, mid - hi, std::max(step, 1.f), hh);
                        }
                        nvgFillColor(vg, ag::withA(col, baseA * (0.35f + 0.65f * (float)(q + 1) / 4.f)));
                        nvgFill(vg);
                    }
                };
                env(scMinI, scMaxI, nvgRGB(255, 255, 255), 0.22f); // dry input (white)
                env(scMinR, scMaxR, ag::violet(), 0.75f);          // right
                env(scMinL, scMaxL, ag::accent(), 0.75f);          // left

                // Legend.
                const char *lg[3] = {"L", "R", "IN"};
                NVGcolor lc[3] = {ag::accent(), ag::violet(), nvgRGB(255, 255, 255)};
                for (int i = 0; i < 3; ++i)
                {
                    float x = PX + 6.f + (float)i * 24.f;
                    nvgBeginPath(vg);
                    nvgRect(vg, x, PY + 5.f, 4.f, 4.f);
                    nvgFillColor(vg, ag::withA(lc[i], 0.9f));
                    nvgFill(vg);
                    ag::text(vg, x + 7.f, PY + 7.f, lg[i], 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
                }
                ag::text(vg, PX + 4.f, PY + PH - 6.f, "-4 S", 7.f, ag::dim(), ag::ALIGN_L, 0.f);
                ag::text(vg, PX + PW - 4.f, PY + PH - 6.f, "NOW", 7.f, ag::dim(), ag::ALIGN_R, 0.f);
            }
            else if (mode == 1)
            {
                // ===== BARS: reverb = the 14 output taps of the plate; echo = the repeat levels =====
                const int NB = Vortex::NUM_TAPS;
                const float cell = PW / (float)NB, barW = 20.f;
                const float base = PY + PH - 12.f, maxH = PH - 24.f; // bars stay below the caption

                // Horizontal guides.
                nvgBeginPath(vg);
                for (int k = 1; k <= 3; ++k)
                {
                    float y = base - maxH * (float)k / 4.f + 0.5f;
                    nvgMoveTo(vg, PX, y);
                    nvgLineTo(vg, PX + PW, y);
                }
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Bar heights: echo uses the geometric repeat-decay envelope
                // (dB-log mapped to 0..1); reverb uses the measured tap levels.
                const float fb = frz ? 1.f : pv[10] * 0.95f;
                const float pp = pv[3];
                char buf[32];
                for (int i = 0; i < NB; ++i)
                {
                    float h;
                    if (echo)
                    {
                        float lvl = std::max(pv[11] * std::pow(fb, (float)i), 1e-6f);         // repeat i+1
                        h = rack::math::clamp(1.f + 20.f * std::log10(lvl) / 60.f, 0.f, 1.f); // -60 .. 0 dB
                    }
                    else
                    {
                        h = rack::math::clamp(std::sqrt(tapLv[i] / 4.f), 0.f, 1.f);
                    }
                    barPeak[i] = std::max(h, barPeak[i] - 0.007f);
                    float x = PX + (float)i * cell + (cell - barW) / 2.f;
                    float bh = std::max(h * maxH, 1.f);
                    NVGcolor c0;
                    if (echo)
                        c0 = (pp > 0.5f && (i & 1)) ? ag::violet() : ag::accent(); // ping-pong alternate colouring
                    else
                        c0 = (i < 7) ? ag::accent() : ag::violet(); // left taps | right taps

                    // Flat bar body.
                    nvgBeginPath(vg);
                    nvgRect(vg, x, base - bh, barW, bh);
                    nvgFillColor(vg, ag::withA(c0, 0.7f));
                    nvgFill(vg);
                    // Peak-hold line.
                    float py = base - barPeak[i] * maxH;
                    nvgBeginPath(vg);
                    nvgRect(vg, x, py - 1.f, barW, 1.5f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 220));
                    nvgFill(vg);

                    // Labels: echo = seconds, reverb = tap delay in ms (scaled by size).
                    if (echo)
                        snprintf(buf, sizeof(buf), "%.1f", (float)(i + 1) * eMs / 1000.f);
                    else
                    {
                        float ref = (i < 7) ? Vortex::TAP_L[i] : Vortex::TAP_R[i - 7];
                        snprintf(buf, sizeof(buf), "%.0f", ref / Vortex::REF_SR * 1000.f * size);
                    }
                    ag::text(vg, x + barW / 2.f, base + 6.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.f);
                }

                ag::text(vg, PX + 6.f, PY + 7.f, echo ? "ECHO REPEATS (S)  0 TO -60 DB" : "PLATE OUTPUT TAPS (MS)  L | R",
                         7.f, ag::dim(), ag::ALIGN_L, 0.5f);
                // Divider between the L and R halves in reverb mode.
                if (!echo)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, PX + PW / 2.f - 0.5f, PY + 14.f, 1.f, PH - 16.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 30));
                    nvgFill(vg);
                }
            }
            else
            {
                // ===== SPECTRUM: log-frequency FFT of the output =====
                const float fMin = 20.f, fMax = std::min(20000.f, sr * 0.5f);
                auto xOf = [&](float f)
                { return PX + PW * std::log(f / fMin) / std::log(fMax / fMin); };
                auto yOf = [&](float db)
                { return PY + PH - rack::math::clamp((db + 90.f) / 90.f, 0.f, 1.f) * PH; };

                // Grid: log-frequency verticals and dB horizontals.
                nvgBeginPath(vg);
                const float fl[9] = {50.f, 100.f, 200.f, 500.f, 1000.f, 2000.f, 5000.f, 10000.f, 20000.f};
                for (int i = 0; i < 9; ++i)
                {
                    if (fl[i] >= fMax)
                        continue;
                    float x = xOf(fl[i]) + 0.5f;
                    nvgMoveTo(vg, x, PY);
                    nvgLineTo(vg, x, PY + PH);
                }
                for (int d = -20; d >= -80; d -= 20)
                {
                    float y = yOf((float)d) + 0.5f;
                    nvgMoveTo(vg, PX, y);
                    nvgLineTo(vg, PX + PW, y);
                }
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Smoothing (fast attack, slow release) + peak-hold decay.
                for (int b = 0; b < VX_BANDS; ++b)
                {
                    float v = rack::math::clamp(specDb[b], -90.f, 0.f);
                    specSm[b] += (v > specSm[b] ? 0.55f : 0.14f) * (v - specSm[b]);
                    specPk[b] = std::max(specSm[b], specPk[b] - 0.35f);
                }

                // Filled spectrum (flat area under the smoothed curve).
                const float bw = PW / (float)VX_BANDS;
                nvgBeginPath(vg);
                nvgMoveTo(vg, PX, PY + PH);
                for (int b = 0; b < VX_BANDS; ++b)
                    nvgLineTo(vg, PX + ((float)b + 0.5f) * bw, yOf(specSm[b]));
                nvgLineTo(vg, PX + PW, PY + PH);
                nvgClosePath(vg);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.18f));
                nvgFill(vg);

                // Peak-hold line.
                nvgBeginPath(vg);
                for (int b = 0; b < VX_BANDS; ++b)
                {
                    float x = PX + ((float)b + 0.5f) * bw, y = yOf(specPk[b]);
                    if (b == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, ag::withA(ag::violet(), 0.55f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // Main curve: a single hairline.
                nvgBeginPath(vg);
                for (int b = 0; b < VX_BANDS; ++b)
                {
                    float x = PX + ((float)b + 0.5f) * bw, y = yOf(specSm[b]);
                    if (b == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, ag::bright());
                nvgStrokeWidth(vg, 1.f);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                // Frequency / dB labels.
                const char *fn[9] = {"50", "100", "200", "500", "1K", "2K", "5K", "10K", "20K"};
                for (int i = 0; i < 9; ++i)
                {
                    if (fl[i] >= fMax)
                        continue;
                    ag::text(vg, xOf(fl[i]), PY + PH - 5.f, fn[i], 7.f, ag::dim(), ag::ALIGN_C, 0.f);
                }
                char buf[16];
                for (int d = -20; d >= -80; d -= 20)
                {
                    snprintf(buf, sizeof(buf), "%d", d);
                    ag::text(vg, PX + 3.f, yOf((float)d) - 5.f, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.f);
                }
            }

            // Clear flash across the plot while a clear is in progress.
            if (fade < 0.99f)
            {
                nvgBeginPath(vg);
                nvgRect(vg, PX, PY, PW, PH);
                nvgFillColor(vg, ag::withA(ag::violet(), 0.28f * (1.f - fade)));
                nvgFill(vg);
            }
            nvgResetScissor(vg);
            nvgRestore(vg);

            // ---------- stereo output meters (right of the plot) ----------
            ag::meter(vg, PX + PW + 8.f, PY, 6.f, PH, mL / 5.f);
            ag::meter(vg, PX + PW + 17.f, PY, 6.f, PH, mR / 5.f);

            // ---------- header: tabs + status ----------
            const char *tn[3] = {"WAVE", "BARS", "SPECTRUM"};
            for (int i = 0; i < 3; ++i)
            {
                bool on = (i == mode);
                nvgBeginPath(vg);
                nvgRoundedRect(vg, VP_TABX[i], 3.f, VP_TABW[i], 11.f, 3.f);
                if (on)
                {
                    nvgFillColor(vg, ag::withA(ag::accent(), 0.22f));
                    nvgFill(vg);
                    nvgStrokeColor(vg, ag::withA(ag::accent(), 0.9f));
                }
                else
                {
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                }
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                ag::text(vg, VP_TABX[i] + VP_TABW[i] / 2.f, 8.8f, tn[i], 7.f, on ? ag::accent() : ag::dim(),
                         ag::ALIGN_C, 0.5f);
            }
            char buf[96];
            // Mode + status.
            ag::text(vg, W - 6.f, 8.5f, echo ? "ECHO" : "REVERB", 7.f, echo ? ag::violet() : ag::accent(),
                     ag::ALIGN_R, 1.f);
            if (fade < 0.99f)
            {
                ag::text(vg, W - 52.f, 8.5f, "CLEARING", 7.f, ag::violet(), ag::ALIGN_R, 0.5f);
            }
            else
            {
                snprintf(buf, sizeof(buf), "%s%s%sCH %d", frz ? "FROZEN  " : "", sIn ? "ST IN  " : "", sOut ? "ST OUT  " : "", ch);
                ag::text(vg, W - 52.f, 8.5f, buf, 7.f, frz ? ag::violet() : ag::accent(), ag::ALIGN_R, 0.5f);
            }

            // ---------- footer: readouts for the current mode ----------
            // Five pairs: PRE, SIZE/TIME, RT60/FDBK, DAMP/PING, MIX.
            const char *nm[5];
            char val[5][16];
            snprintf(val[0], sizeof(val[0]), "%.0fMS", pv[0] * 500.f);
            float fc = 18000.f * std::pow(2500.f / 18000.f, pv[4]);
            if (!echo)
            {
                nm[0] = "PRE";
                nm[1] = "SIZE";
                nm[2] = "RT60";
                nm[3] = "DAMP";
                nm[4] = "MIX";
                snprintf(val[1], sizeof(val[1]), "%.0f%%", size * 100.f);
                snprintf(val[2], sizeof(val[2]), "%.1fS", rt);
                snprintf(val[3], sizeof(val[3]), "%.1fK", fc / 1000.f);
                snprintf(val[4], sizeof(val[4]), "%.0f%%", pv[8] * 100.f);
            }
            else
            {
                nm[0] = "PRE";
                nm[1] = "TIME";
                nm[2] = "FDBK";
                nm[3] = "PING";
                nm[4] = "MIX";
                snprintf(val[1], sizeof(val[1]), "%.0fMS", eMs);
                snprintf(val[2], sizeof(val[2]), "%.0f%%", frz ? 100.f : pv[10] * 95.f);
                snprintf(val[3], sizeof(val[3]), "%.0f%%", pv[3] * 100.f);
                snprintf(val[4], sizeof(val[4]), "%.0f%%", pv[8] * 100.f);
            }
            const float cw = (W - 16.f) / 5.f;
            for (int i = 0; i < 5; ++i)
            {
                float x = 8.f + (float)i * cw;
                ag::text(vg, x, H - 7.f, nm[i], 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
                ag::text(vg, x + cw - 8.f, H - 7.f, val[i], 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            }

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Module widget
// ------------------------------------------------------------
// Layout:
//   * static panel + display
//   * 12 control cells (knob / CV jack / trimpot attenuverter)
//   * bottom strip: 3 LED buttons, 2 CKSS switches, 6 jacks
//   * context menu: display mode + CV range
// ============================================================
struct VortexWidget : ModuleWidget
{
    VortexWidget(VortexModule *module)
    {
        setModule(module);
        box.size = Vec(VX_W, VX_H);

        addChild(new VxPanel(module));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new VxDisplay(module));

        // 12 cells: knob on top, CV jack (left) and attenuverter (right) below.
        for (int i = 0; i < 12; ++i)
        {
            int row = i / 6, col = i % 6;
            float cx = VX_COL[col], T = VX_ROW[row];
            const VxCellDef &c = VX_CELLS[i];
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, T + VX_KNOB_DY), module, c.param));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - VX_SUB_DX, T + VX_SUB_DY), module, c.cv));
            addParam(createParamCentered<Trimpot>(Vec(cx + VX_SUB_DX, T + VX_SUB_DY), module, c.atten));
        }

        // Strip: mode / freeze / clear buttons, stereo switches, then the jacks.
        const float sy = VX_STRIP_Y + VX_STRIP_DY;
        addParam(createParamCentered<LEDButton>(Vec(vxStripX(0), sy), module, VortexModule::ECHO_MODE_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<ag::VioletLight>>(Vec(vxStripX(0), sy), module, VortexModule::ECHO_MODE_BUTTON_LIGHT));
        addParam(createParamCentered<LEDButton>(Vec(vxStripX(1), sy), module, VortexModule::FREEZE_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<ag::CyanLight>>(Vec(vxStripX(1), sy), module, VortexModule::FREEZE_BUTTON_LIGHT));
        addParam(createParamCentered<LEDButton>(Vec(vxStripX(2), sy), module, VortexModule::CLEAR_BUTTON_PARAM));
        addChild(createLightCentered<SmallLight<ag::CyanLight>>(Vec(vxStripX(2), sy), module, VortexModule::CLEAR_BUTTON_LIGHT));
        addParam(createParamCentered<CKSS>(Vec(vxStripX(3), sy), module, VortexModule::IN_STEREO_BUTTON_PARAM));
        addParam(createParamCentered<CKSS>(Vec(vxStripX(4), sy), module, VortexModule::OUT_STEREO_BUTTON_PARAM));

        // Inputs (L, R, freeze gate, clear trigger), then outputs (L, R).
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vxStripX(5), sy), module, VortexModule::IN_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vxStripX(6), sy), module, VortexModule::IN_R_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vxStripX(7), sy), module, VortexModule::FREEZE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vxStripX(8), sy), module, VortexModule::CLEAR_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(vxStripX(9), sy), module, VortexModule::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(vxStripX(10), sy), module, VortexModule::OUT_R_OUTPUT));
    }

    // Add a "Display" submenu (Wave / Bars / Spectrum) and a "CV range"
    // submenu (unipolar / bipolar) to the module context menu.
    void appendContextMenu(Menu *menu) override
    {
        VortexModule *module = dynamic_cast<VortexModule *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createSubmenuItem("Display", "", [=](Menu *sub)
                                         {
            const char* names[3] = {"Wave", "Bars", "Spectrum"};
            for (int i = 0; i < 3; ++i)
                sub->addChild(createCheckMenuItem(names[i], "",
                    [=]() { return module->dispMode == i; },
                    [=]() { module->dispMode = i; })); }));
        menu->addChild(createSubmenuItem("CV range", "", [=](Menu *sub)
                                         {
            sub->addChild(createCheckMenuItem("0 V to +10 V (unipolar, default)", "",
                [=]() { return module->cvRangeMode == VortexModule::UNIPOLAR_0_10; },
                [=]() { module->cvRangeMode = VortexModule::UNIPOLAR_0_10; }));
            sub->addChild(createCheckMenuItem("-5 V to +5 V (bipolar)", "",
                [=]() { return module->cvRangeMode == VortexModule::BIPOLAR_M5_5; },
                [=]() { module->cvRangeMode = VortexModule::BIPOLAR_M5_5; })); }));
    }
};

Model *modelVortex = createModel<VortexModule, VortexWidget>("Vortex");