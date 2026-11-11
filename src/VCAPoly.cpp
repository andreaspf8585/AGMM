// ============================================================================
// VCAPoly — polyphonic VCA with global limit (14 HP Rack module)
// ----------------------------------------------------------------------------
// Overview:
//   * Polyphonic audio in / out, up to 16 voices.
//   * Per-voice gain = env * limit, where:
//       - `env`   comes from a poly ENV CV (0..10 V per channel), or 1.0
//                 when the ENV input is unpatched (unity gain),
//       - `limit` comes from the GLOBAL LIMIT knob, or from a per-channel
//                 LIMIT CV override when that jack is patched.
//   * Output channel count follows the AUDIO input's channel count.
//   * Display:
//       16 stacked voice rows showing the current gain (0..2x) as a bar,
//       a thin violet line = output peak, and a limit marker per row
//       (per-channel if LIMIT CV is patched, otherwise one global line).
//   * Footer shows the global limit plus "CH n", "UNITY" or "LIM CV".
//   * The output jack is bypassed to the input when unpatched.
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants (14 HP = 210 px wide)
// ------------------------------------------------------------
static const float VP_W = RACK_GRID_WIDTH * 14; // 210
static const float VP_H = RACK_GRID_HEIGHT;     // 380

// Display (140 x 196): scale header, 16 voice rows, footer. Bezel adds 4 px around it.
static const float VP_DW = 140.f, VP_DH = 196.f;
static const float VP_DX = (VP_W - VP_DW) / 2.f; // 35
static const float VP_DY = 26.f;
static const float VP_HEAD_LINE = 15.5f;            // hairline under the scale header (display-local)
static const float VP_BX0 = 26.f, VP_BW = 106.f;    // bar area inside the display
static const float VP_ROW0 = 22.f, VP_PITCH = 10.f; // first row y and row pitch
static const float VP_FOOT_LINE = 184.5f;           // hairline above the footer row
static const float VP_FOOT_Y = 190.f;               // footer text centre line

// Global limit knob cell: 80 x 52, label at +10, knob at +32
static const float VP_KNOB_TOP = 234.f, VP_KNOB_W = 80.f, VP_KNOB_H = 52.f;
static const float VP_KNOBY = VP_KNOB_TOP + 32.f;

// Jack cells: four 44 x 42, label at +10, jack at +28
static const float VP_JTOP = 296.f, VP_JW = 44.f, VP_JH = 42.f, VP_JDY = 28.f;
static float vpJx(int i) { return 30.f + 50.f * (float)i; }

// ------------------------------------------------------------
// Module — polyphonic DSP + state.
//
// Gain law per voice:
//   * ENV unpatched  -> g = 1, out = in, meter fill = 1.0
//   * ENV patched    -> g = clamp(env/10,0,1) * limit
//                      out = in * g
//                      fill = clamp(env/(limit*10),0,1)  (so the bar and
//                             limit marker share a coordinate system)
//
// Limit source:
//   * LIMIT CV patched -> limit = clamp(limitCv/10, 0, 2) per channel
//   * otherwise         -> limit = GLOBAL LIMIT knob value
//
// Display members are written every sample and read by the UI thread;
// torn reads are cosmetic only.
// ------------------------------------------------------------
struct VCAPoly : Module
{
    enum ParamId
    {
        PARAM_LIMIT,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUT_AUDIO,
        INPUT_ENV,
        INPUT_LIMIT_CV,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_AUDIO,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // --------------------------------------------------------
    // Meter: fill ratio (0..1) per voice, relative to limit * 10 V.
    // --------------------------------------------------------
    float envelopeFill[16] = {};

    // --------------------------------------------------------
    // Display data (written by the audio thread, read by the UI thread).
    // --------------------------------------------------------
    float gainNow[16] = {};   // current per-voice gain
    float outPeak[16] = {};   // per-voice output peak (one-pole decay)
    int activeCh = 0;         // number of channels being processed
    bool envConn = false;     // true if the ENV input is patched
    bool limitCvConn = false; // true if the LIMIT CV input is patched

    // --------------------------------------------------------
    // Constructor — configure the single param, inputs, output, bypass.
    // --------------------------------------------------------
    VCAPoly()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PARAM_LIMIT, 0.f, 2.f, 1.f, "Global gain limit", "x");
        configInput(INPUT_AUDIO, "Polyphonic audio");
        configInput(INPUT_ENV, "Polyphonic envelope CV (0 to 10 V)");
        configInput(INPUT_LIMIT_CV, "Polyphonic gain limit CV (0 to 10 V)");
        configOutput(OUTPUT_AUDIO, "Polyphonic output");

        // Pass-through when unpatched.
        configBypass(INPUT_AUDIO, OUTPUT_AUDIO);
    }

    // --------------------------------------------------------
    // Per-sample DSP for all active voices.
    //   * No AUDIO channels     -> emit 0 channels, clear all display state.
    //   * ENV unpatched         -> unity gain, fill = 1.0.
    //   * ENV patched           -> per-voice gain = envNorm * limit;
    //                              fill = env/(limit*10), capped at 1.
    //   * Missing env/limit-CV channels read as 0 V.
    // --------------------------------------------------------
    void process(const ProcessArgs &args) override
    {
        int audioChannels = inputs[INPUT_AUDIO].getChannels();
        activeCh = audioChannels;
        if (audioChannels == 0)
        {
            // No audio connected: emit no channels and wipe the display.
            outputs[OUTPUT_AUDIO].setChannels(0);
            for (int c = 0; c < 16; ++c)
            {
                envelopeFill[c] = 0.f;
                gainNow[c] = 0.f;
                outPeak[c] = 0.f;
            }
            return;
        }

        bool envConnected = inputs[INPUT_ENV].isConnected();
        envConn = envConnected;
        int envChannels = envConnected ? inputs[INPUT_ENV].getChannels() : 0;

        bool limitCvConnected = inputs[INPUT_LIMIT_CV].isConnected();
        limitCvConn = limitCvConnected;
        int limitCvChannels = limitCvConnected ? inputs[INPUT_LIMIT_CV].getChannels() : 0;

        int channels = audioChannels; // output always follows audio channel count

        outputs[OUTPUT_AUDIO].setChannels(channels);

        float globalLimit = params[PARAM_LIMIT].getValue();
        // Per-sample peak decay coefficient for the display meters.
        float md = 1.f - 5.f * args.sampleTime;

        for (int c = 0; c < channels; ++c)
        {
            float audio = inputs[INPUT_AUDIO].getVoltage(c);
            float out, fill, g;

            // Per-channel limit: global knob value, overridden by limit CV if connected.
            float limit = globalLimit;
            if (limitCvConnected)
            {
                float limitCv = (c < limitCvChannels) ? inputs[INPUT_LIMIT_CV].getVoltage(c) : 0.f;
                limit = rack::math::clamp(limitCv / 10.f, 0.f, 2.f);
            }
            float limitVoltage = limit * 10.f;

            if (!envConnected)
            {
                out = audio; // no envelope cable: unity gain
                fill = 1.0f;
                g = 1.f;
            }
            else
            {
                float envVoltage = (c < envChannels) ? inputs[INPUT_ENV].getVoltage(c) : 0.f;
                float envNorm = rack::math::clamp(envVoltage / 10.f, 0.f, 1.f);
                g = envNorm * limit;
                out = audio * g;
                fill = (limitVoltage > 0.f) ? rack::math::clamp(envVoltage / limitVoltage, 0.f, 1.f) : 0.f;
            }
            outputs[OUTPUT_AUDIO].setVoltage(out, c);
            envelopeFill[c] = fill;
            gainNow[c] = g;
            outPeak[c] = std::max(std::fabs(out), outPeak[c] * md);
        }

        // Clear display state for voices that are no longer active.
        for (int c = channels; c < 16; ++c)
        {
            envelopeFill[c] = 0.f;
            gainNow[c] = 0.f;
            outPeak[c] = 0.f;
        }
    }
};

// ------------------------------------------------------------
// Drawing helpers (anonymous namespace — UI-only)
// ------------------------------------------------------------
namespace
{

    namespace ag = agmm;

    // ------------------------------------------------------------
    // VPPanel — static panel chrome:
    //   * title + panel base
    //   * display bezel
    //   * single GLOBAL LIMIT knob cell
    //   * four jack cells: AUDIO, ENV, LIMIT, OUT (OUT tinted as an output)
    // The actual widgets are placed by VCAPolyWidget at the same positions.
    // ------------------------------------------------------------
    struct VPPanel : TransparentWidget
    {
        VPPanel() { box.size = Vec(VP_W, VP_H); }
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, VP_W, VP_H, "VCAPOLY");
            ag::bezel(vg, VP_DX, VP_DY, VP_DW, VP_DH);

            // GLOBAL LIMIT knob cell.
            ag::cell(vg, VP_W / 2.f, VP_KNOB_TOP, VP_KNOB_W, VP_KNOB_H, "GLOBAL LIMIT");

            // Jack cells; the OUT cell is tinted (output colouring).
            const char *names[4] = {"AUDIO", "ENV", "LIMIT", "OUT"};
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, vpJx(i), VP_JTOP, VP_JW, VP_JH, names[i], i == 3, 7.f);
            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // VPKnobArc — arc track + value arc around the GLOBAL LIMIT knob.
    // Non-interactive; purely visual feedback.
    // ------------------------------------------------------------
    struct VPKnobArc : TransparentWidget
    {
        VCAPoly *module = nullptr;
        VPKnobArc(VCAPoly *m) : module(m) { box.size = Vec(VP_W, VP_H); }
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float R = 16.f;
            // Arc from ~225° to ~315° (CCW around the knob face).
            const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
            const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;
            // Track
            nvgBeginPath(vg);
            nvgArc(vg, VP_W / 2.f, VP_KNOBY, R, a0, a1, NVG_CW);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgStrokeWidth(vg, 2.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgStroke(vg);
            // Value arc (skip when very small).
            if (!module)
                return;
            float v = module->paramQuantities[VCAPoly::PARAM_LIMIT]->getScaledValue();
            if (v < 0.004f)
                return;
            nvgBeginPath(vg);
            nvgArc(vg, VP_W / 2.f, VP_KNOBY, R, a0, a0 + (a1 - a0) * v, NVG_CW);
            nvgStrokeColor(vg, ag::accent());
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
        }
    };

    // ------------------------------------------------------------
    // VPDisplay — the live 140x196 voice display.
    //   draw()       : static background (screen fill, separators,
    //                  scale gridlines at 0 / 1x / 2x, row slots,
    //                  voice numbers 1..16).
    //   drawLayer(1) : live content per voice (gain bar 0..2x, output
    //                  peak tick, per-row or global limit marker),
    //                  footer readouts (GLOBAL x.xxX + CH n / UNITY /
    //                  LIM CV).
    // If module == nullptr (browser preview) synthetic data is drawn.
    // ------------------------------------------------------------
    struct VPDisplay : TransparentWidget
    {
        VCAPoly *module = nullptr;
        VPDisplay(VCAPoly *m) : module(m)
        {
            box.pos = Vec(VP_DX, VP_DY);
            box.size = Vec(VP_DW, VP_DH);
        }

        // ---- static background ----
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, VP_DW, VP_DH);

            // Separators under the scale header and above the footer.
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, VP_HEAD_LINE);
            nvgLineTo(vg, VP_DW - 8.f, VP_HEAD_LINE);
            nvgMoveTo(vg, 8.f, VP_FOOT_LINE);
            nvgLineTo(vg, VP_DW - 8.f, VP_FOOT_LINE);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Scale gridlines at 0, 1x and 2x, labels in the header row.
            const char *sc[3] = {"0", "1X", "2X"};
            const float rowsEnd = VP_ROW0 + 16.f * VP_PITCH - 2.f;
            nvgBeginPath(vg);
            for (int k = 0; k < 3; ++k)
            {
                float x = VP_BX0 + VP_BW * (float)k / 2.f + 0.5f;
                nvgMoveTo(vg, x, VP_HEAD_LINE + 2.f);
                nvgLineTo(vg, x, rowsEnd);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 26));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            for (int k = 0; k < 3; ++k)
                ag::text(vg, VP_BX0 + VP_BW * (float)k / 2.f, 8.f, sc[k], 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            // Row slots + voice numbers.
            for (int c = 0; c < 16; ++c)
            {
                float y = VP_ROW0 + (float)c * VP_PITCH;
                nvgBeginPath(vg);
                nvgRect(vg, VP_BX0, y, VP_BW, 6.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
                nvgFill(vg);
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", c + 1);
                ag::text(vg, 11.f, y + 3.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            }
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

            // Snapshot module state (or demo data) into locals.
            float globalLimit = 1.f;
            int n = 8;
            bool conn = true;
            bool limitCvConn = false;
            float gain[16], pk[16];
            float perChLimit[16];
            if (module)
            {
                globalLimit = module->params[VCAPoly::PARAM_LIMIT].getValue();
                n = module->activeCh;
                conn = module->envConn;
                limitCvConn = module->limitCvConn;
                for (int c = 0; c < 16; ++c)
                {
                    gain[c] = module->gainNow[c];
                    pk[c] = module->outPeak[c];
                }
                // Per-channel limits for display: knob value, or LIMIT CV if patched.
                int limitCvChannels = limitCvConn ? module->inputs[VCAPoly::INPUT_LIMIT_CV].getChannels() : 0;
                for (int c = 0; c < 16; ++c)
                {
                    float lim = globalLimit;
                    if (limitCvConn)
                    {
                        float limitCv = (c < limitCvChannels) ? module->inputs[VCAPoly::INPUT_LIMIT_CV].getVoltage(c) : 0.f;
                        lim = rack::math::clamp(limitCv / 10.f, 0.f, 2.f);
                    }
                    perChLimit[c] = lim;
                }
            }
            else
            {
                // demo values for the module browser
                for (int c = 0; c < 16; ++c)
                {
                    gain[c] = std::max(0.f, globalLimit * (0.5f + 0.5f * std::sin((float)c * 0.7f + 0.3f)));
                    pk[c] = 4.f * gain[c] * 0.5f;
                    perChLimit[c] = globalLimit;
                }
            }
            n = std::min(std::max(n, 0), 16);

            // Voice bars + output peak ticks.
            for (int c = 0; c < 16; ++c)
            {
                float y = VP_ROW0 + (float)c * VP_PITCH;
                if (c >= n)
                {
                    // Inactive voice: small dash.
                    nvgBeginPath(vg);
                    nvgRect(vg, VP_BX0, y + 2.5f, 3.f, 1.f);
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 40));
                    nvgFill(vg);
                    continue;
                }
                // Gain bar (0..2x scaled to the full width).
                float w = std::min(std::max(gain[c] / 2.f, 0.f), 1.f) * VP_BW;
                if (w > 0.5f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, VP_BX0, y, w, 6.f);
                    nvgFillColor(vg, ag::withA(ag::accent(), 0.6f));
                    nvgFill(vg);
                    // Bright tip on the leading edge of the bar.
                    nvgBeginPath(vg);
                    nvgRect(vg, VP_BX0 + w - 1.5f, y, 1.5f, 6.f);
                    nvgFillColor(vg, ag::bright());
                    nvgFill(vg);
                }
                // Output peak, a thin violet line under the bar.
                float pw = std::min(std::max(pk[c] / 5.f, 0.f), 1.f) * VP_BW;
                if (pw > 0.5f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, VP_BX0, y + 6.5f, pw, 1.5f);
                    nvgFillColor(vg, ag::withA(ag::violet(), 0.9f));
                    nvgFill(vg);
                }
            }

            // Limit markers: per-row if LIMIT CV is patched, otherwise one global line.
            if (conn)
            {
                if (limitCvConn)
                {
                    // Per-voice vertical tick at x = limit.
                    for (int c = 0; c < n; ++c)
                    {
                        float y = VP_ROW0 + (float)c * VP_PITCH;
                        float lx = VP_BX0 + VP_BW * (perChLimit[c] / 2.f);
                        nvgBeginPath(vg);
                        nvgMoveTo(vg, lx, y - 1.f);
                        nvgLineTo(vg, lx, y + 7.f);
                        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 200));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }
                }
                else
                {
                    // One global vertical line + downward triangle at the top.
                    float lx = VP_BX0 + VP_BW * (globalLimit / 2.f);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, lx, VP_HEAD_LINE + 4.f);
                    nvgLineTo(vg, lx, VP_ROW0 + 16.f * VP_PITCH - 2.f);
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 230));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, lx - 3.5f, VP_HEAD_LINE + 1.5f);
                    nvgLineTo(vg, lx + 3.5f, VP_HEAD_LINE + 1.5f);
                    nvgLineTo(vg, lx, VP_HEAD_LINE + 6.f);
                    nvgClosePath(vg);
                    nvgFillColor(vg, nvgRGB(255, 255, 255));
                    nvgFill(vg);
                }
            }

            // Footer readouts: GLOBAL x.xxX on the left, status on the right.
            char buf[40];
            snprintf(buf, sizeof(buf), "GLOBAL %.2fX", globalLimit);
            ag::text(vg, 8.f, VP_FOOT_Y, buf, 7.f, ag::textCol(), ag::ALIGN_L, 0.5f);
            if (limitCvConn)
            {
                snprintf(buf, sizeof(buf), "CH %d LIM CV", n);
                ag::text(vg, VP_DW - 8.f, VP_FOOT_Y, buf, 7.f, ag::warm(), ag::ALIGN_R, 0.5f);
            }
            else
            {
                snprintf(buf, sizeof(buf), conn ? "CH %d" : "CH %d UNITY", n);
                ag::text(vg, VP_DW - 8.f, VP_FOOT_Y, buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            }
            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// VCAPolyWidget — Rack widget.
//   * panel background, knob arc, screws
//   * display + GLOBAL LIMIT knob
//   * four jacks: AUDIO, ENV, LIMIT, OUT
// ------------------------------------------------------------
struct VCAPolyWidget : ModuleWidget
{
    VCAPolyWidget(VCAPoly *module)
    {
        setModule(module);
        box.size = Vec(VP_W, VP_H);

        addChild(new VPPanel());
        addChild(new VPKnobArc(module));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new VPDisplay(module));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(VP_W / 2.f, VP_KNOBY), module, VCAPoly::PARAM_LIMIT));

        // Jacks: audio in, env in, limit CV in, audio out.
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vpJx(0), VP_JTOP + VP_JDY), module, VCAPoly::INPUT_AUDIO));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vpJx(1), VP_JTOP + VP_JDY), module, VCAPoly::INPUT_ENV));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(vpJx(2), VP_JTOP + VP_JDY), module, VCAPoly::INPUT_LIMIT_CV));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(vpJx(3), VP_JTOP + VP_JDY), module, VCAPoly::OUTPUT_AUDIO));
    }
};

Model *modelVCAPoly = createModel<VCAPoly, VCAPolyWidget>("VCAPoly");