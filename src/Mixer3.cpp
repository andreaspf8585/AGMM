#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <string>

static const int M3_SCOPE_N = 96;

// ============================================================
// Stereo 3-channel mixer – per-channel L/R inputs, L/R outputs
// ============================================================
struct Mixer3 : Module
{
    enum CVRange
    {
        BIPOLAR_M5_5,
        UNIPOLAR_0_10
    };
    CVRange cvRangeMode = UNIPOLAR_0_10; // default = original cv/10 behaviour

    // Display state
    float scopeMin[M3_SCOPE_N] = {};
    float scopeMax[M3_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;

    enum ParamId
    {
        PARAM_CH1_LEVEL,
        PARAM_CH2_LEVEL,
        PARAM_CH3_LEVEL,
        PARAM_CH1_ATTV,
        PARAM_CH2_ATTV,
        PARAM_CH3_ATTV,
        PARAM_MASTER_GAIN,
        PARAMS_LEN
    };
    enum InputId
    {
        // Interleaved so INPUT_CH1_L + ch*2 gives (L,R) for channel ch
        INPUT_CH1_L,
        INPUT_CH1_R,
        INPUT_CH2_L,
        INPUT_CH2_R,
        INPUT_CH3_L,
        INPUT_CH3_R,
        INPUT_CH1_CV,
        INPUT_CH2_CV,
        INPUT_CH3_CV,
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

    Mixer3()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(PARAM_CH1_LEVEL, 0.f, 1.f, 0.f, "Channel 1 level", "%", 0.f, 100.f);
        configParam(PARAM_CH2_LEVEL, 0.f, 1.f, 0.f, "Channel 2 level", "%", 0.f, 100.f);
        configParam(PARAM_CH3_LEVEL, 0.f, 1.f, 0.f, "Channel 3 level", "%", 0.f, 100.f);

        configParam(PARAM_CH1_ATTV, -1.f, 1.f, 0.f, "Channel 1 CV attenuverter", "%", 0.f, 100.f);
        configParam(PARAM_CH2_ATTV, -1.f, 1.f, 0.f, "Channel 2 CV attenuverter", "%", 0.f, 100.f);
        configParam(PARAM_CH3_ATTV, -1.f, 1.f, 0.f, "Channel 3 CV attenuverter", "%", 0.f, 100.f);

        configParam(PARAM_MASTER_GAIN, 0.f, 2.f, 1.f, "Master gain", "%", 0.f, 100.f);

        for (int ch = 0; ch < 3; ++ch)
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

    float scaleCV(float v) const
    {
        if (cvRangeMode == BIPOLAR_M5_5)
            return std::max(-1.f, std::min(v / 5.f, 1.f));
        return v / 10.f;
    }

    void process(const ProcessArgs &args) override
    {
        float mixL = 0.f;
        float mixR = 0.f;

        for (int ch = 0; ch < 3; ++ch)
        {
            float levelKnob = params[PARAM_CH1_LEVEL + ch].getValue();
            float attenuverter = params[PARAM_CH1_ATTV + ch].getValue();

            float cv = inputs[INPUT_CH1_CV + ch].getVoltage();
            float cvOffset = scaleCV(cv) * attenuverter;

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
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % M3_SCOPE_N;
        }
    }
};

// ============================================================
// Layout (18 HP = 270 x 380)
// Every control sits in a plate with its label ABOVE it.
// ============================================================
static const float M3_PANEL_W = RACK_GRID_WIDTH * 18; // 270
static const float M3_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel is drawn 4 px around it): scope + meters | divider | readout row
static const float M3_DISP_W = 226.f;
static const float M3_DISP_H = 46.f;
static const float M3_DISP_X = (M3_PANEL_W - M3_DISP_W) / 2.f; // 22
static const float M3_DISP_Y = 28.f;
static const float M3_SCOPE_W = 206.f;
static const float M3_SCOPE_H = 34.f;
static const float M3_FOOT_Y = M3_SCOPE_H + 7.f;

// 3 channel plates. Inside each (offsets from the plate top):
//   +9 "CH n" | +22 L/R captions | +38 jacks | +62 LEVEL caption | +82 knob | +106 CV/ATT captions | +122 jacks
static const float M3_COL_X[3] = {48.f, 135.f, 222.f};
static const float M3_PLATE_W = 80.f, M3_PLATE_H = 142.f;
static const float M3_JACK_OFFSET = 20.f; // L / CV at cx-20, R / ATT at cx+20
static const float M3_SEC_CH_Y = 90.f;
static const float M3_CH_TOP = 98.f;

// Master row: L OUT | GAIN | R OUT
static const float M3_SEC_MASTER_Y = 256.f;
static const float M3_MASTER_TOP = 264.f, M3_MASTER_H = 64.f;
static const float M3_GAIN_CX = 135.f, M3_GAIN_W = 70.f;
static const float M3_OUT_CX[2] = {48.f, 222.f};
static const float M3_OUT_W = 62.f;

// ============================================================
// UI (anonymous namespace: names cannot clash with other module files)
// ============================================================
namespace
{

    struct M3PanelBackground : TransparentWidget
    {
        M3PanelBackground() { box.size = Vec(M3_PANEL_W, M3_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, M3_PANEL_W, M3_PANEL_H, "MIXER 3 STEREO");
            agmm::bezel(vg, M3_DISP_X, M3_DISP_Y, M3_DISP_W, M3_DISP_H);

            // ---- channels ----
            agmm::sectionHeader(vg, M3_PANEL_W, M3_SEC_CH_Y, "CHANNELS", 14.f);
            char buf[16];
            for (int ch = 0; ch < 3; ++ch)
            {
                const float cx = M3_COL_X[ch], top = M3_CH_TOP;
                const float lx = cx - M3_JACK_OFFSET, rx = cx + M3_JACK_OFFSET;
                agmm::plate(vg, cx, top, M3_PLATE_W, M3_PLATE_H, false);

                snprintf(buf, sizeof(buf), "CH %d", ch + 1);
                agmm::text(vg, cx, top + 9.f, buf, 8.f, agmm::accent(), agmm::ALIGN_C, 1.f);

                agmm::text(vg, lx, top + 22.f, "L", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                agmm::text(vg, rx, top + 22.f, "R", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);

                agmm::text(vg, cx, top + 62.f, "LEVEL", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);

                agmm::text(vg, lx, top + 106.f, "CV", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
                agmm::text(vg, rx, top + 106.f, "ATT", 7.f, agmm::dim(), agmm::ALIGN_C, 0.5f);
            }

            // ---- master: outputs tinted ----
            agmm::sectionHeader(vg, M3_PANEL_W, M3_SEC_MASTER_Y, "MASTER", 14.f);
            agmm::cell(vg, M3_OUT_CX[0], M3_MASTER_TOP, M3_OUT_W, M3_MASTER_H, "L OUT", true, 7.f);
            agmm::cell(vg, M3_GAIN_CX, M3_MASTER_TOP, M3_GAIN_W, M3_MASTER_H, "GAIN", false, 7.f);
            agmm::cell(vg, M3_OUT_CX[1], M3_MASTER_TOP, M3_OUT_W, M3_MASTER_H, "R OUT", true, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope display + L/R level meters, readouts in their own row
    // ------------------------------------------------------------
    struct M3DisplayWidget : TransparentWidget
    {
        Mixer3 *module = nullptr;

        M3DisplayWidget(Mixer3 *m) : module(m)
        {
            box.pos = Vec(M3_DISP_X, M3_DISP_Y);
            box.size = Vec(M3_DISP_W, M3_DISP_H);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, M3_DISP_W, M3_DISP_H);

            // Faint grid over the scope area only
            nvgBeginPath(vg);
            for (float x = 0.f; x <= M3_SCOPE_W; x += 28.f)
            {
                nvgMoveTo(vg, x + 0.5f, 0.f);
                nvgLineTo(vg, x + 0.5f, M3_SCOPE_H);
            }
            nvgMoveTo(vg, 0.f, M3_SCOPE_H / 2.f + 0.5f);
            nvgLineTo(vg, M3_SCOPE_W, M3_SCOPE_H / 2.f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Divider between the scope and the readout row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, M3_SCOPE_H + 1.5f);
            nvgLineTo(vg, M3_DISP_W, M3_SCOPE_H + 1.5f);
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

            const float mid = M3_SCOPE_H / 2.f;
            const float amp = mid - 2.f;
            const float colW = M3_SCOPE_W / (float)M3_SCOPE_N;
            const float range = 10.f; // +-10 V display range for a hot mixer

            nvgSave(vg);
            nvgScissor(vg, 0.f, 0.f, M3_SCOPE_W, M3_SCOPE_H);
            nvgBeginPath(vg);
            for (int i = 0; i < M3_SCOPE_N; ++i)
            {
                float vmin, vmax;
                if (module)
                {
                    int idx = (module->scopeHead + i) % M3_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                }
                else
                {
                    float t = (float)i / M3_SCOPE_N;
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
                agmm::meter(vg, M3_SCOPE_W + 4.f + ch * 7.f, 2.f, 5.f, M3_SCOPE_H - 4.f, peak / range);
            }

            // Readout row
            char buf[32];
            float gain = module ? module->params[Mixer3::PARAM_MASTER_GAIN].getValue() : 1.f;
            float peak = module ? std::max(module->peakL, module->peakR) : 0.f;
            snprintf(buf, sizeof(buf), "PK %4.1fV", peak);
            agmm::text(vg, 4.f, M3_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "GAIN %3.0f%%", gain * 100.f);
            agmm::text(vg, M3_DISP_W - 4.f, M3_FOOT_Y, buf, 7.f, agmm::accent(), agmm::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Module widget
// ============================================================
struct Mixer3Widget : ModuleWidget
{
    Mixer3Widget(Mixer3 *module)
    {
        setModule(module);
        box.size = Vec(M3_PANEL_W, M3_PANEL_H);

        addChild(new M3PanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new M3DisplayWidget(module));

        // 3 stereo channel columns
        for (int ch = 0; ch < 3; ++ch)
        {
            const float cx = M3_COL_X[ch], top = M3_CH_TOP;
            const float lx = cx - M3_JACK_OFFSET, rx = cx + M3_JACK_OFFSET;

            // Stereo input pair
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(lx, top + 38.f), module, (Mixer3::InputId)(Mixer3::INPUT_CH1_L + ch * 2)));
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(rx, top + 38.f), module, (Mixer3::InputId)(Mixer3::INPUT_CH1_L + ch * 2 + 1)));

            // Level knob (controls both L and R of this channel)
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(cx, top + 82.f), module, (Mixer3::ParamId)(Mixer3::PARAM_CH1_LEVEL + ch)));

            // CV input jack + attenuverter
            addInput(createInputCentered<DarkPJ301MPort>(
                Vec(lx, top + 122.f), module, (Mixer3::InputId)(Mixer3::INPUT_CH1_CV + ch)));
            addParam(createParamCentered<Trimpot>(
                Vec(rx, top + 122.f), module, (Mixer3::ParamId)(Mixer3::PARAM_CH1_ATTV + ch)));
        }

        // Master gain + stereo outputs
        addParam(createParamCentered<RoundBlackKnob>(
            Vec(M3_GAIN_CX, M3_MASTER_TOP + 38.f), module, Mixer3::PARAM_MASTER_GAIN));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(M3_OUT_CX[0], M3_MASTER_TOP + 38.f), module, Mixer3::OUTPUT_L));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(M3_OUT_CX[1], M3_MASTER_TOP + 38.f), module, Mixer3::OUTPUT_R));
    }

    void appendContextMenu(Menu *menu) override
    {
        Mixer3 *module = dynamic_cast<Mixer3 *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));

        struct CVRangeItem : MenuItem
        {
            Mixer3 *module;
            Mixer3::CVRange range;
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
        unipolarItem->range = Mixer3::UNIPOLAR_0_10;
        menu->addChild(unipolarItem);

        CVRangeItem *bipolarItem = createMenuItem<CVRangeItem>("-5V to +5V (bipolar)");
        bipolarItem->module = module;
        bipolarItem->range = Mixer3::BIPOLAR_M5_5;
        menu->addChild(bipolarItem);
    }
};

Model *modelMixer3 = createModel<Mixer3, Mixer3Widget>("Mixer3");