#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Layout (12 HP = 180 x 380)
// Every control sits in a plate with its label ABOVE it.
// ------------------------------------------------------------
static const float MG_PANEL_W = RACK_GRID_WIDTH * 12; // 180
static const float MG_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Channel display (bezel is drawn 4 px around it): caption, number, divider, 16 segments
static const float MG_DISP_W = 132.f;
static const float MG_DISP_H = 56.f;
static const float MG_DISP_X = (MG_PANEL_W - MG_DISP_W) / 2.f; // 24
static const float MG_DISP_Y = 28.f;

// Input grid (4 x 4 plates: label +10, jack centre +27)
static const float MG_SECTION_IN_Y = 98.f;
static const float MG_GRID_X[4] = {30.f, 70.f, 110.f, 150.f};
static const float MG_ROW_TOP[4] = {106.f, 149.f, 192.f, 235.f};
static const float MG_CELL_W = 36.f, MG_CELL_H = 40.f;

// Output (label +10, jack centre +29)
static const float MG_SECTION_OUT_Y = 287.f;
static const float MG_OUT_X = MG_PANEL_W / 2.f;
static const float MG_OUT_TOP = 295.f, MG_OUT_W = 70.f, MG_OUT_H = 44.f;

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Merger : Module
{
    enum ParamId
    {
        PARAMS_LEN
    };
    enum InputId
    {
        ENUMS(MERGE_INPUT, 16),
        INPUTS_LEN
    };
    enum OutputId
    {
        POLY_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // Shown on the display
    int channels = 0;
    bool active[16] = {};

    Merger()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        for (int i = 0; i < 16; ++i)
            configInput(MERGE_INPUT + i, string::f("Channel %d", i + 1));
        configOutput(POLY_OUTPUT, "Polyphonic");
    }

    void process(const ProcessArgs &args) override
    {
        // Output channel count = highest connected input (unconnected gaps carry 0 V)
        int last = 0;
        for (int i = 0; i < 16; ++i)
        {
            bool c = inputs[MERGE_INPUT + i].isConnected();
            active[i] = c;
            if (c)
                last = i + 1;
        }
        channels = last;

        outputs[POLY_OUTPUT].setChannels(last);
        for (int i = 0; i < last; ++i)
            outputs[POLY_OUTPUT].setVoltage(inputs[MERGE_INPUT + i].getVoltage(), i);
    }
};

// ------------------------------------------------------------
// UI (anonymous namespace: names cannot clash with other module files)
// ------------------------------------------------------------
namespace
{

    struct MGPanelBackground : TransparentWidget
    {
        MGPanelBackground() { box.size = Vec(MG_PANEL_W, MG_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::panelBase(vg, MG_PANEL_W, MG_PANEL_H, "MERGER");
            agmm::bezel(vg, MG_DISP_X, MG_DISP_Y, MG_DISP_W, MG_DISP_H);

            // ---- inputs ----
            agmm::sectionHeader(vg, MG_PANEL_W, MG_SECTION_IN_Y, "INPUTS", 12.f);
            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "%d", r * 4 + c + 1);
                    agmm::cell(vg, MG_GRID_X[c], MG_ROW_TOP[r], MG_CELL_W, MG_CELL_H, buf, false, 7.f);
                }
            }

            // ---- output ----
            agmm::sectionHeader(vg, MG_PANEL_W, MG_SECTION_OUT_Y, "OUTPUT", 12.f);
            agmm::cell(vg, MG_OUT_X, MG_OUT_TOP, MG_OUT_W, MG_OUT_H, "POLY OUT", true, 7.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Channel count display
    // ------------------------------------------------------------
    struct MGDisplayWidget : TransparentWidget
    {
        Merger *module = nullptr;

        MGDisplayWidget(Merger *m) : module(m)
        {
            box.pos = Vec(MG_DISP_X, MG_DISP_Y);
            box.size = Vec(MG_DISP_W, MG_DISP_H);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            agmm::screen(vg, 0.f, 0.f, MG_DISP_W, MG_DISP_H);

            // Divider between the number and the segment row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, 41.5f);
            nvgLineTo(vg, MG_DISP_W, 41.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
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

            // State (module is null in the module browser: show a demo)
            int ch = 8;
            bool act[16];
            for (int i = 0; i < 16; ++i)
                act[i] = (i != 3 && i < 8);
            if (module)
            {
                ch = module->channels;
                for (int i = 0; i < 16; ++i)
                    act[i] = module->active[i];
            }

            // Caption row
            agmm::text(vg, MG_DISP_W / 2.f, 8.f, "CHANNELS", 7.f, agmm::dim(), agmm::ALIGN_C, 2.f);

            // Big number
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", ch);
            agmm::text(vg, MG_DISP_W / 2.f, 26.f, buf, 24.f,
                       ch > 0 ? agmm::bright() : agmm::withA(agmm::dim(), 0.6f), agmm::ALIGN_C, 1.f);

            // 16-segment bar: bright = connected, outlined = inside range but
            // unconnected (carries 0 V), very dim = unused
            const float pitch = 7.5f, segW = 5.5f, segH = 5.f;
            const float x0 = (MG_DISP_W - (16.f * pitch - (pitch - segW))) / 2.f;
            const float y0 = 46.f;
            for (int i = 0; i < 16; ++i)
            {
                float sx = x0 + i * pitch;
                nvgBeginPath(vg);
                nvgRoundedRect(vg, sx, y0, segW, segH, 1.f);
                if (act[i])
                {
                    nvgFillColor(vg, agmm::accent());
                    nvgFill(vg);
                }
                else if (i < ch)
                {
                    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
                    nvgFill(vg);
                    nvgBeginPath(vg);
                    nvgRoundedRect(vg, sx + 0.5f, y0 + 0.5f, segW - 1.f, segH - 1.f, 1.f);
                    nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 110));
                    nvgStrokeWidth(vg, 0.8f);
                    nvgStroke(vg);
                }
                else
                {
                    nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                    nvgFill(vg);
                }
            }

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct MergerWidget : ModuleWidget
{
    MergerWidget(Merger *module)
    {
        setModule(module);
        box.size = Vec(MG_PANEL_W, MG_PANEL_H);

        addChild(new MGPanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new MGDisplayWidget(module));

        // 16 inputs in a 4 x 4 grid (row by row)
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                addInput(createInputCentered<DarkPJ301MPort>(
                    Vec(MG_GRID_X[c], MG_ROW_TOP[r] + 27.f), module,
                    Merger::MERGE_INPUT + r * 4 + c));
            }
        }

        // Polyphonic output
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(MG_OUT_X, MG_OUT_TOP + 29.f), module, Merger::POLY_OUTPUT));
    }
};

Model *modelMerger = createModel<Merger, MergerWidget>("Merger");