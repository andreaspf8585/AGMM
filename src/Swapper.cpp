// ============================================================================
// Swapper — 12 HP dual-channel crossfader / swapper
// ----------------------------------------------------------------------------
// Left and Right inputs each have an independent "amount" control that decides
// how much of the *other* channel is mixed into this output:
//
//     L_out = (1 - R>L) * L_in + (R>L) * R_in
//     R_out = (1 - L>R) * R_in + (L>R) * L_in
//
// Each amount is (knob + attenuverter * CV), clamped to [0, 1]. CV is scaled so
// 10 V = 100 %. The panel display visualises both amounts as line weights in a
// routing diagram plus a live IN/OUT level readout.
// ============================================================================

#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ---- panel layout (12 HP = 180 px wide) ----
// All coordinates below are in Rack pixel space (RACK_GRID_WIDTH = 15 px).
// They are used both by the panel background painter and by the widget
// constructor, so the drawn cells and the placed controls stay aligned.
static const float SW_W = RACK_GRID_WIDTH * 12; // 180  panel width
static const float SW_H = RACK_GRID_HEIGHT;     // 380  panel height

// Display (140 x 90): header row, routing diagram, footer row. Bezel adds 4 px around it.
static const float SW_DX = 20.f, SW_DY = 30.f, SW_DW = 140.f, SW_DH = 90.f;
static const float SW_HEAD_LINE = 15.5f;               // hairline under the header row
static const float SW_FOOT_LINE = 74.5f;               // hairline above the footer row
static const float SW_FOOT_TXT = SW_DH - 8.f;          // footer text centre line
static const float SW_XI = 30.f, SW_XO = SW_DW - 30.f; // input / output node columns
static const float SW_YL = 33.f, SW_YR = 57.f;         // L / R node rows

// Amount rows: header, then three 46 x 52 cells (label +10, control +32)
static const float SW_HDR[2] = {138.f, 212.f};
static const float SW_ROW[2] = {146.f, 220.f}; // cell tops
static const float SW_CELL_W = 46.f, SW_CELL_H = 52.f, SW_CTL_DY = 32.f;
static const float SW_COL[3] = {40.f, 90.f, 140.f};

// I/O row: four 36 x 50 cells on a 40 px pitch, jack at +30
static const float SW_IO_HDR = 288.f;
static const float SW_IO_TOP = 296.f;
static const float SW_IO_W = 36.f, SW_IO_H = 50.f, SW_IO_JACK_DY = 30.f;
// Horizontal centre of I/O cell i (0..3), left to right.
static float swIoX(int i) { return 30.f + 40.f * (float)i; }

// ============================================================================
// Swapper — the DSP module.
// ----------------------------------------------------------------------------
// Owns the parameter/port definitions and performs the per-sample crossfade.
// It also publishes a small snapshot of its internal state (amounts and
// peak-follower levels) for the display widget to render on the UI thread.
// ============================================================================
struct Swapper : Module
{
    enum ParamId
    {
        LTO_R_KNOB,
        RTO_L_KNOB,
        LTO_R_ATTEN,
        RTO_L_ATTEN,
        PARAMS_LEN
    };
    enum InputId
    {
        L_IN,
        R_IN,
        CV_LTOR,
        CV_RTOL,
        INPUTS_LEN
    };
    enum OutputId
    {
        L_OUT,
        R_OUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // Display data (written by the audio thread, read by the UI thread)
    float amtLR = 0.f, amtRL = 0.f;                         // current mix amounts (0..1)
    float mInL = 0.f, mInR = 0.f, mOutL = 0.f, mOutR = 0.f; // peak levels for the meter

    // Configure ports and parameters; set up pass-through bypass.
    Swapper()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(LTO_R_KNOB, 0.f, 1.f, 0.f, "L to R amount", "%", 0.f, 100.f);
        configParam(RTO_L_KNOB, 0.f, 1.f, 0.f, "R to L amount", "%", 0.f, 100.f);
        configParam(LTO_R_ATTEN, -1.f, 1.f, 0.f, "L to R CV attenuverter", "%", 0.f, 100.f);
        configParam(RTO_L_ATTEN, -1.f, 1.f, 0.f, "R to L CV attenuverter", "%", 0.f, 100.f);

        configInput(L_IN, "Left");
        configInput(R_IN, "Right");
        configInput(CV_LTOR, "CV for L to R (10 V = 100%)");
        configInput(CV_RTOL, "CV for R to L (10 V = 100%)");
        configOutput(L_OUT, "Left");
        configOutput(R_OUT, "Right");

        // Bypass passes each side straight through
        configBypass(L_IN, L_OUT);
        configBypass(R_IN, R_OUT);
    }

    // Per-sample work: read inputs, compute the two mix amounts, crossfade,
    // and update the peak-follower state consumed by the display.
    void process(const ProcessArgs &args) override
    {
        float lIn = inputs[L_IN].getVoltage();
        float rIn = inputs[R_IN].getVoltage();

        float ltoR_manual = params[LTO_R_KNOB].getValue();
        float rtoL_manual = params[RTO_L_KNOB].getValue();

        float ltoR_cv = inputs[CV_LTOR].getVoltage() / 10.f;
        float rtoL_cv = inputs[CV_RTOL].getVoltage() / 10.f;
        float ltoR_atten = params[LTO_R_ATTEN].getValue();
        float rtoL_atten = params[RTO_L_ATTEN].getValue();

        float ltoR_amount = clamp(ltoR_manual + ltoR_atten * ltoR_cv, 0.f, 1.f);
        float rtoL_amount = clamp(rtoL_manual + rtoL_atten * rtoL_cv, 0.f, 1.f);

        float lOut = (1.f - rtoL_amount) * lIn + rtoL_amount * rIn;
        float rOut = (1.f - ltoR_amount) * rIn + ltoR_amount * lIn;

        outputs[L_OUT].setVoltage(lOut);
        outputs[R_OUT].setVoltage(rOut);

        amtLR = ltoR_amount;
        amtRL = rtoL_amount;
        // Simple peak follower with ~5 Hz decay, so the meter falls gently.
        float md = 1.f - 5.f * args.sampleTime;
        mInL = std::max(std::fabs(lIn), mInL * md);
        mInR = std::max(std::fabs(rIn), mInR * md);
        mOutL = std::max(std::fabs(lOut), mOutL * md);
        mOutR = std::max(std::fabs(rOut), mOutR * md);
    }
};

namespace
{

    namespace ag = agmm; // shorthand for the shared UI helpers

    // Section title in a given colour; hairlines stop a measured distance from the text
    void colorHeader(NVGcontext *vg, float W, float y, const char *txt, NVGcolor col)
    {
        const float cx = W / 2.f, margin = 12.f;
        const float gap = ag::textWidth(vg, txt, 8.f, 2.f) / 2.f + 6.f;
        ag::text(vg, cx, y, txt, 8.f, col, ag::ALIGN_C, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, margin, y);
        nvgLineTo(vg, cx - gap, y);
        nvgMoveTo(vg, cx + gap, y);
        nvgLineTo(vg, W - margin, y);
        nvgStrokeColor(vg, ag::withA(col, 0.3f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // ------------------------------------------------------------
    // Panel background: title, display frame, one labelled cell per control
    // ------------------------------------------------------------
    struct SWPanel : TransparentWidget
    {
        SWPanel() { box.size = Vec(SW_W, SW_H); }
        // Static layer: paints the full panel chrome (base, bezel, section
        // headers, and the empty cells that sit behind the knobs/ports).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, SW_W, SW_H, "SWAPPER");
            ag::bezel(vg, SW_DX, SW_DY, SW_DW, SW_DH);

            // Two "amount" sections, each with AMOUNT / ATT / CV columns.
            const char *names[2] = {"L TO R", "R TO L"};
            for (int r = 0; r < 2; ++r)
            {
                colorHeader(vg, SW_W, SW_HDR[r], names[r], r == 0 ? ag::accent() : ag::violet());
                const char *lbl[3] = {"AMOUNT", "ATT", "CV"};
                for (int c = 0; c < 3; ++c)
                    ag::cell(vg, SW_COL[c], SW_ROW[r], SW_CELL_W, SW_CELL_H, lbl[c]);
            }

            // I/O section: four cells, outputs drawn with the output accent.
            ag::sectionHeader(vg, SW_W, SW_IO_HDR, "I / O");
            const char *io[4] = {"L IN", "R IN", "L OUT", "R OUT"};
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, swIoX(i), SW_IO_TOP, SW_IO_W, SW_IO_H, io[i], i >= 2);
            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Routing diagram: line weight shows how much of each input reaches each output
    // ------------------------------------------------------------
    struct SWDisplay : TransparentWidget
    {
        Swapper *module = nullptr;
        SWDisplay(Swapper *m) : module(m)
        {
            box.pos = Vec(SW_DX, SW_DY);
            box.size = Vec(SW_DW, SW_DH);
        }

        // Base layer: screen fill and the two hairlines bracketing the diagram.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, SW_DW, SW_DH);

            // separators under the header and above the footer
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, SW_HEAD_LINE);
            nvgLineTo(vg, SW_DW - 8.f, SW_HEAD_LINE);
            nvgMoveTo(vg, 8.f, SW_FOOT_LINE);
            nvgLineTo(vg, SW_DW - 8.f, SW_FOOT_LINE);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            Widget::draw(args);
        }

        // Overlay layer (layer 1): the live routing graph, nodes, and readouts.
        // Colours: cyan = signal originating on the left, violet = the right.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = SW_DW;

            // Sensible idle values so the display still looks sane if there is no module.
            float lr = 0.35f, rl = 0.6f, iL = 2.f, iR = 3.f, oL = 3.f, oR = 2.5f;
            if (module)
            {
                lr = module->amtLR;
                rl = module->amtRL;
                iL = module->mInL;
                iR = module->mInR;
                oL = module->mOutL;
                oR = module->mOutR;
            }

            const float xi = SW_XI, xo = SW_XO, yL = SW_YL, yR = SW_YR;

            // Four links. Cyan = signal from L, violet = signal from R.
            // The `w` field (0..1) is the fraction of that input reaching that output.
            struct Link
            {
                float x0, y0, x1, y1, w;
                bool fromL;
            };
            Link links[4] = {
                {xi, yL, xo, yL, 1.f - rl, true},  // IN L -> OUT L
                {xi, yR, xo, yL, rl, false},       // IN R -> OUT L
                {xi, yR, xo, yR, 1.f - lr, false}, // IN R -> OUT R
                {xi, yL, xo, yR, lr, true},        // IN L -> OUT R
            };
            nvgLineCap(vg, NVG_ROUND);
            for (const Link &k : links)
            {
                NVGcolor c = k.fromL ? ag::accent() : ag::violet();
                nvgBeginPath(vg);
                nvgMoveTo(vg, k.x0, k.y0);
                nvgLineTo(vg, k.x1, k.y1);
                nvgStrokeColor(vg, ag::withA(c, 0.15f + 0.85f * k.w));
                nvgStrokeWidth(vg, 0.8f + 2.4f * k.w);
                nvgStroke(vg);
            }

            // nodes: ring + level-dependent fill
            // Ring is constant; the inner dot's alpha tracks the signal level (0..5 V).
            auto node = [&](float x, float y, float v, NVGcolor c)
            {
                float a = std::min(v / 5.f, 1.f);
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, 6.f);
                nvgStrokeColor(vg, ag::withA(c, 0.35f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, 3.f);
                nvgFillColor(vg, ag::withA(c, 0.5f + 0.5f * a));
                nvgFill(vg);
            };
            node(xi, yL, iL, ag::accent());
            node(xi, yR, iR, ag::violet());
            node(xo, yL, oL, ag::accent());
            node(xo, yR, oR, ag::violet());
            ag::text(vg, 10.f, yL, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, 10.f, yR, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, W - 10.f, yL, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, W - 10.f, yR, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            // header and footer readouts
            char buf[32];
            snprintf(buf, sizeof(buf), "L>R %3.0f%%", lr * 100.f);
            ag::text(vg, 6.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "R>L %3.0f%%", rl * 100.f);
            ag::text(vg, W - 6.f, 8.f, buf, 7.f, ag::violet(), ag::ALIGN_R, 0.5f);
            ag::text(vg, xi, SW_FOOT_TXT, "IN", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            ag::text(vg, xo, SW_FOOT_TXT, "OUT", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================================
// SwapperWidget — panel assembly.
// ----------------------------------------------------------------------------
// Stacks the background painter, screws, and display, then places each knob,
// attenuverter, CV input, and I/O jack at the coordinates used by SWPanel.
// ============================================================================
struct SwapperWidget : ModuleWidget
{
    SwapperWidget(Swapper *module)
    {
        setModule(module);
        box.size = Vec(SW_W, SW_H);

        addChild(new SWPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(new SWDisplay(module));

        // Two identical rows (L>R, R>L): knob / attenuverter / CV input.
        const int knob[2] = {Swapper::LTO_R_KNOB, Swapper::RTO_L_KNOB};
        const int att[2] = {Swapper::LTO_R_ATTEN, Swapper::RTO_L_ATTEN};
        const int cv[2] = {Swapper::CV_LTOR, Swapper::CV_RTOL};
        for (int r = 0; r < 2; ++r)
        {
            float cy = SW_ROW[r] + SW_CTL_DY;
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SW_COL[0], cy), module, knob[r]));
            addParam(createParamCentered<Trimpot>(Vec(SW_COL[1], cy), module, att[r]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(SW_COL[2], cy), module, cv[r]));
        }

        // I/O row: L IN, R IN, L OUT, R OUT.
        const float jy = SW_IO_TOP + SW_IO_JACK_DY;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(swIoX(0), jy), module, Swapper::L_IN));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(swIoX(1), jy), module, Swapper::R_IN));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(swIoX(2), jy), module, Swapper::L_OUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(swIoX(3), jy), module, Swapper::R_OUT));
    }
};

Model *modelSwapper = createModel<Swapper, SwapperWidget>("Swapper");