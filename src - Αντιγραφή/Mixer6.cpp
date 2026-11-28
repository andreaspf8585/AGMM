#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <string>
#include "Knob_arcs.hpp"

static const int MX_SCOPE_N = 96;

// ============================================================
// Stereo 6-channel mixer – per-channel L/R inputs, L/R outputs
//
// Per channel: gain = LEVEL knob + (CV * attenuverter), clamped to 0..1 and
// applied to both L and R. All six channels are summed, then scaled by MASTER
// GAIN. The channel layout is taller than Mixer3 because CV and ATT sit in
// separate rows inside a narrower 56 px plate.
//
// CV scaling is selectable and saved with the patch:
//   UNIPOLAR_0_10 (default): 10 V -> +1.0 full level offset.
//   BIPOLAR_M5_5:            +-5 V -> +-1.0, clamped.
// ============================================================
struct Mixer6 : Module
{
    enum CVRange
    {
        BIPOLAR_M5_5,
        UNIPOLAR_0_10
    };
    CVRange cvRangeMode = UNIPOLAR_0_10; // preserve original (cv/10) behavior

    // Display state
    //
    // Scope is a min/max envelope per column (96 columns). Written by the audio
    // thread, read by the UI; a torn read only affects the drawing.
    float scopeMin[MX_SCOPE_N] = {};
    float scopeMax[MX_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    enum ParamId
    {
        // Six levels, then six attenuverters, then the master. Both groups are
        // contiguous so the DSP loop can use PARAM_CH1_LEVEL + ch and
        // PARAM_CH1_ATTV + ch.
        PARAM_CH1_LEVEL,
        PARAM_CH2_LEVEL,
        PARAM_CH3_LEVEL,
        PARAM_CH4_LEVEL,
        PARAM_CH5_LEVEL,
        PARAM_CH6_LEVEL,
        PARAM_CH1_ATTV,
        PARAM_CH2_ATTV,
        PARAM_CH3_ATTV,
        PARAM_CH4_ATTV,
        PARAM_CH5_ATTV,
        PARAM_CH6_ATTV,
        PARAM_MASTER_GAIN,
        PARAMS_LEN
    };
    enum InputId
    {
        // Interleaved so INPUT_CH1_L + ch*2 gives (L,R) for channel ch
        //
        // Twelve audio jacks first (six L/R pairs), then six CV jacks. The DSP
        // uses stride 2 for audio and stride 1 for CV.
        INPUT_CH1_L,
        INPUT_CH1_R,
        INPUT_CH2_L,
        INPUT_CH2_R,
        INPUT_CH3_L,
        INPUT_CH3_R,
        INPUT_CH4_L,
        INPUT_CH4_R,
        INPUT_CH5_L,
        INPUT_CH5_R,
        INPUT_CH6_L,
        INPUT_CH6_R,
        INPUT_CH1_CV,
        INPUT_CH2_CV,
        INPUT_CH3_CV,
        INPUT_CH4_CV,
        INPUT_CH5_CV,
        INPUT_CH6_CV,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUTPUT_L,
        OUTPUT_R,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    Mixer6()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        // Levels default to 0: a freshly added mixer stays silent.
        for (int ch = 0; ch < 6; ++ch)
            configParam(PARAM_CH1_LEVEL + ch, 0.f, 1.f, 0.f,
                        "Channel " + std::to_string(ch + 1) + " level", "%", 0.f, 100.f);

        // Attenuverters are bipolar and default to 0 (no CV influence).
        for (int ch = 0; ch < 6; ++ch)
            configParam(PARAM_CH1_ATTV + ch, -1.f, 1.f, 0.f,
                        "Channel " + std::to_string(ch + 1) + " CV attenuverter", "%", 0.f, 100.f);

        configParam(PARAM_MASTER_GAIN, 0.f, 2.f, 1.f, "Master gain", "%", 0.f, 100.f);

        // Jack names per channel, using the same interleaved indices as the DSP.
        for (int ch = 0; ch < 6; ++ch)
        {
            std::string n = std::to_string(ch + 1);
            configInput(INPUT_CH1_L + ch * 2, "Channel " + n + " L audio");
            configInput(INPUT_CH1_L + ch * 2 + 1, "Channel " + n + " R audio");
            configInput(INPUT_CH1_CV + ch, "Channel " + n + " level CV");
        }

        configOutput(OUTPUT_L, "Mix L output");
        configOutput(OUTPUT_R, "Mix R output");
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        return root;
    }
    void dataFromJson(json_t *root) override
    {
        json_t *mode = json_object_get(root, "cvRangeMode");
        if (mode)
            cvRangeMode = (json_integer_value(mode) == BIPOLAR_M5_5) ? BIPOLAR_M5_5 : UNIPOLAR_0_10;
    }

    // Maps raw CV volts to a +-1 multiplier. Bipolar clamps so a large CV
    // cannot overshoot; unipolar simply divides by 10 (original behaviour).
    float scaleCV(float v) const
    {
        if (cvRangeMode == BIPOLAR_M5_5)
            return std::max(-1.f, std::min(v / 5.f, 1.f));
        return v / 10.f; // UNIPOLAR_0_10 – original behavior
    }

    void process(const ProcessArgs &args) override
    {
        float mixL = 0.f;
        float mixR = 0.f;

        for (int ch = 0; ch < 6; ++ch)
        {
            float levelKnob = params[PARAM_CH1_LEVEL + ch].getValue();
            float attenuverter = params[PARAM_CH1_ATTV + ch].getValue();

            float cv = inputs[INPUT_CH1_CV + ch].getVoltage();
            float cvOffset = scaleCV(cv) * attenuverter;

            // Knob + CV, kept in 0..1: the mixer never inverts phase and never
            // goes above unity per channel.
            float gain = levelKnob + cvOffset;
            if (gain < 0.f)
                gain = 0.f;
            if (gain > 1.f)
                gain = 1.f;

            float inL = inputs[INPUT_CH1_L + ch * 2].getVoltage();
            float inR = inputs[INPUT_CH1_L + ch * 2 + 1].getVoltage();

            mixL += inL * gain;
            mixR += inR * gain;
        }

        float masterGain = params[PARAM_MASTER_GAIN].getValue();
        float outL = mixL * masterGain;
        float outR = mixR * masterGain;

        outputs[OUTPUT_L].setVoltage(outL);
        outputs[OUTPUT_R].setVoltage(outR);

        // ---- display tracking ----
        // Peak meters with a ~0.25 s release.
        float decay = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(outL), peakL * decay);
        peakR = std::max(std::fabs(outR), peakR * decay);

        // Scope: fold the stereo mix to mono, accumulate min/max, and commit a
        // column every ~2.5 ms. 96 columns therefore cover ~240 ms.
        float mono = 0.5f * (outL + outR);
        if (scopeCount == 0)
            scopeBMin = scopeBMax = mono;
        else
        {
            scopeBMin = std::min(scopeBMin, mono);
            scopeBMax = std::max(scopeBMax, mono);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % MX_SCOPE_N;
        }
    }
};

// ============================================================
// Layout (26 HP = 390 x 380)
// Every control sits in a plate with its label ABOVE it.
// ============================================================
static const float MX_PANEL_W = RACK_GRID_WIDTH * 26; // 390
static const float MX_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it): scope + meters | divider | readout row
static const float MX_DISP_W = 340.f;
static const float MX_DISP_H = 46.f;
static const float MX_DISP_X = (MX_PANEL_W - MX_DISP_W) / 2.f; // 25
static const float MX_DISP_Y = 28.f;
static const float MX_SCOPE_W = 320.f;
static const float MX_SCOPE_H = 34.f;
static const float MX_FOOT_Y = MX_SCOPE_H + 7.f;

// 6 channel plates, 60 px apart. Inside each (offsets from the plate top):
//   +9 "CH n" | +21 L/R captions | +36 jacks | +58 LEVEL caption | +78 knob |
//   +100 CV caption | +114 CV jack | +135 ATT caption | +148 attenuverter
static const float MX_COL_X[6] = {45.f, 105.f, 165.f, 225.f, 285.f, 345.f};
static const float MX_PLATE_W = 56.f, MX_PLATE_H = 160.f;
static const float MX_JACK_OFFSET = 15.f; // L at cx-15, R at cx+15
static const float MX_SEC_CH_Y = 90.f;
static const float MX_CH_TOP = 98.f;

// Master row: L OUT | GAIN | R OUT (label +10, control centre +38)
static const float MX_SEC_MASTER_Y = 270.f;
static const float MX_MASTER_TOP = 278.f, MX_MASTER_H = 64.f;
static const float MX_GAIN_CX = MX_PANEL_W / 2.f, MX_GAIN_W = 70.f;
static const float MX_OUT_OFFSET = 85.f, MX_OUT_W = 62.f;

// ============================================================
// UI (anonymous namespace: names cannot clash with other module files)
// ============================================================
namespace
{

    // ------------------------------------------------------------
    // Panel background
    //
    // Static artwork only. Each channel plate is drawn by hand because it
    // stacks four label rows; the master row uses the shared cell helper.
    // ------------------------------------------------------------
    struct MxPanelBackground : TransparentWidget
    {
        MxPanelBackground() { box.size = Vec(MX_PANEL_W, MX_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, MX_PANEL_W, MX_PANEL_H, "MIXER 6 STEREO");
            agmm::bezel(vg, MX_DISP_X, MX_DISP_Y, MX_DISP_W, MX_DISP_H);

            // ---- channels ----
            agmm::sectionHeader(vg, MX_PANEL_W, MX_SEC_CH_Y, "CHANNELS", 14.f);
            char buf[16];
            for (int ch = 0; ch < 6; ++ch)
            {
                const float cx = MX_COL_X[ch], top = MX_CH_TOP;
                const float lx = cx - MX_JACK_OFFSET, rx = cx + MX_JACK_OFFSET;
                agmm::plate(vg, cx, top, MX_PLATE_W, MX_PLATE_H, false);

                // Channel title, then the L/R pair above their jacks.
                snprintf(buf, sizeof(buf), "CH %d", ch + 1);
                agmm::text(vg, cx, top + 9.f, buf, 8.f, agmm::accent(), agmm::ALIGN_C, 0.5f);

                agmm::text(vg, lx, top + 21.f, "L", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                agmm::text(vg, rx, top + 21.f, "R", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);

                // Level, then CV and ATT each get their own caption row.
                agmm::text(vg, cx, top + 58.f, "LEVEL", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                agmm::text(vg, cx, top + 100.f, "CV", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                agmm::text(vg, cx, top + 135.f, "ATT", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            // ---- master: outputs tinted ----
            // The two outputs sit symmetrically around the centred GAIN cell.
            agmm::sectionHeader(vg, MX_PANEL_W, MX_SEC_MASTER_Y, "MASTER", 14.f);
            agmm::cell(vg, MX_GAIN_CX - MX_OUT_OFFSET, MX_MASTER_TOP, MX_OUT_W, MX_MASTER_H, "L OUT", true, 7.f);
            agmm::cell(vg, MX_GAIN_CX, MX_MASTER_TOP, MX_GAIN_W, MX_MASTER_H, "GAIN", false, 7.f);
            agmm::cell(vg, MX_GAIN_CX + MX_OUT_OFFSET, MX_MASTER_TOP, MX_OUT_W, MX_MASTER_H, "R OUT", true, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope display + L/R level meters, readouts in their own row
    //
    // Layer 0: screen, scope grid, divider.
    // Layer 1: live trace, meters, PK/GAIN readouts.
    // ------------------------------------------------------------
    struct MxDisplayWidget : TransparentWidget
    {
        Mixer6 *module = nullptr;

        MxDisplayWidget(Mixer6 *m) : module(m)
        {
            box.pos = Vec(MX_DISP_X, MX_DISP_Y);
            box.size = Vec(MX_DISP_W, MX_DISP_H);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, MX_DISP_W, MX_DISP_H);

            // Faint grid over the scope area only; the readout row stays clean.
            nvgBeginPath(vg);
            for (float x = 0.f; x <= MX_SCOPE_W; x += 28.f)
            {
                nvgMoveTo(vg, x + 0.5f, 0.f);
                nvgLineTo(vg, x + 0.5f, MX_SCOPE_H);
            }
            nvgMoveTo(vg, 0.f, MX_SCOPE_H / 2.f + 0.5f);
            nvgLineTo(vg, MX_SCOPE_W, MX_SCOPE_H / 2.f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Divider between the scope and the readout row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, MX_SCOPE_H + 1.5f);
            nvgLineTo(vg, MX_DISP_W, MX_SCOPE_H + 1.5f);
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

            const float mid = MX_SCOPE_H / 2.f;
            const float amp = mid - 2.f;
            const float colW = MX_SCOPE_W / (float)MX_SCOPE_N;
            const float range = 10.f;

            // One min/max rectangle per column: a filled envelope, so fast
            // transients are never missed between columns.
            nvgSave(vg);
            nvgScissor(vg, 0.f, 0.f, MX_SCOPE_W, MX_SCOPE_H);
            nvgBeginPath(vg);
            for (int i = 0; i < MX_SCOPE_N; ++i)
            {
                float vmin, vmax;
                if (module)
                {
                    int idx = (module->scopeHead + i) % MX_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                }
                else
                {
                    // Demo waveform for the module browser (decaying burst).
                    float t = (float)i / MX_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - std::min(std::max(vmax / range, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / range, -1.f), 1.f) * amp;
                nvgRect(vg, i * colW, y0, std::max(colW - 0.4f, 1.f), std::max(y1 - y0, 1.f));
            }
            nvgFillColor(vg, agmm::bright());
            nvgFill(vg);
            nvgRestore(vg);

            // L / R level meters to the right of the scope
            for (int ch = 0; ch < 2; ++ch)
            {
                float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 3.f : 2.4f);
                agmm::meter(vg, MX_SCOPE_W + 4.f + ch * 7.f, 2.f, 5.f, MX_SCOPE_H - 4.f, peak / range);
            }

            // Readout row: hotter peak on the left, master gain on the right.
            char buf[32];
            float gain = module ? module->params[Mixer6::PARAM_MASTER_GAIN].getValue() : 1.f;
            float peak = module ? std::max(module->peakL, module->peakR) : 0.f;
            snprintf(buf, sizeof(buf), "PK %4.1fV", peak);
            agmm::text(vg, 4.f, MX_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "GAIN %3.0f%%", gain * 100.f);
            agmm::text(vg, MX_DISP_W - 4.f, MX_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Module widget
//
// Order: panel background, then display, then controls. Widget indices use the
// same strides as the DSP (2 for audio, 1 for CV).
// ============================================================
struct Mixer6Widget : ModuleWidget
{
    Mixer6Widget(Mixer6 *module)
    {
        setModule(module);
        box.size = Vec(MX_PANEL_W, MX_PANEL_H);

        addChild(new MxPanelBackground());

        // Four rack screws, one per corner.
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new MxDisplayWidget(module));

        // 6 stereo channel columns
        for (int ch = 0; ch < 6; ++ch)
        {
            const float cx = MX_COL_X[ch], top = MX_CH_TOP;
            const float lx = cx - MX_JACK_OFFSET, rx = cx + MX_JACK_OFFSET;

            // Stereo input pair
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(lx, top + 36.f), module,
                (Mixer6::InputId)(Mixer6::INPUT_CH1_L + ch * 2)));
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(rx, top + 36.f), module,
                (Mixer6::InputId)(Mixer6::INPUT_CH1_L + ch * 2 + 1)));

            // Level knob (controls both L and R of this channel)
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(cx, top + 78.f), module,
                (Mixer6::ParamId)(Mixer6::PARAM_CH1_LEVEL + ch)));

            // CV input + attenuverter
            // Narrow plate: CV and ATT are stacked, not side by side.
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(cx, top + 114.f), module,
                (Mixer6::InputId)(Mixer6::INPUT_CH1_CV + ch)));
            addParam(createParamCentered<Trimpot>(
                Vec(cx, top + 148.f), module,
                (Mixer6::ParamId)(Mixer6::PARAM_CH1_ATTV + ch)));
        }
        agmm::addKnobArcs(this, 2.f);
        // Master gain + stereo outputs
        // All three on one row, outputs symmetric around the gain knob.
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(MX_GAIN_CX, MX_MASTER_TOP + 38.f), module, Mixer6::PARAM_MASTER_GAIN));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(MX_GAIN_CX - MX_OUT_OFFSET, MX_MASTER_TOP + 38.f), module, Mixer6::OUTPUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(MX_GAIN_CX + MX_OUT_OFFSET, MX_MASTER_TOP + 38.f), module, Mixer6::OUTPUT_R));
        agmm::addKnobArcs(this, 2.f);
    }

    void appendContextMenu(Menu *menu) override
    {
        Mixer6 *module = dynamic_cast<Mixer6 *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        // Local item: draws the checkmark and writes the mode back (persisted
        // through dataToJson / dataFromJson).
        struct CVRangeItem : MenuItem
        {
            Mixer6 *module;
            Mixer6::CVRange range;
            void onAction(const ActionEvent &e) override
            {
                module->cvRangeMode = range;
            }
            void step() override
            {
                rightText = CHECKMARK(module->cvRangeMode == range);
                MenuItem::step();
            }
        };

        CVRangeItem *unipolarItem = createMenuItem<CVRangeItem>("0V to +10V (unipolar, default)");
        unipolarItem->module = module;
        unipolarItem->range = Mixer6::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);

        CVRangeItem *bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = Mixer6::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);
    }
};

Model *modelMixer6 = createModel<Mixer6, Mixer6Widget>("Mixer6");