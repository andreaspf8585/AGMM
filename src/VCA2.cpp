// ============================================================================
// VCA2 — dual VCA with per-channel gain limit (12 HP Rack module)
// ----------------------------------------------------------------------------
// Overview:
//   * Two independent channels. Each channel:
//       - AUDIO input (audio source)
//       - ENV input (0..10 V envelope; unpatched = unity gain)
//       - LIMIT knob (0x..2x gain ceiling)
//       - OUT jack (bypassed to the audio input when unpatched)
//   * Display: two vertical "limit bars" (0..2x) that double as
//     interactive controls (click/drag to set the limit, double-click to
//     reset to 1x) and as envelope meters (fill = env / limit, capped
//     at the limit marker) plus a thin output peak meter on the right.
//   * No envelopes patched => unity gain and a full bar.
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>  

// ------------------------------------------------------------
// Layout constants (12 HP = 180 px wide)
// ------------------------------------------------------------
static const float V2_W = RACK_GRID_WIDTH * 12;   // 180
static const float V2_H = RACK_GRID_HEIGHT;       // 380

// Display (140 x 170): header row, two limit bars, readouts. Bezel adds 4 px around it.
static const float V2_DX = 20.f, V2_DY = 26.f, V2_DW = 140.f, V2_DH = 170.f;
static const float V2_HEAD_LINE = 15.5f;          // hairline under the header row (display-local)
static const float V2_CX[2] = {55.f, 125.f};      // channel centre lines (panel x)
static const float V2_BW = 44.f, V2_BH = 110.f;   // limit bar size
static const float V2_BY = V2_DY + 24.f;          // limit bar top (panel coordinates)

// Jack cells: three rows of two, 56 x 42, label at +10, jack at +28
static const float V2_CELL_W = 56.f, V2_CELL_H = 42.f, V2_JACK_DY = 28.f;
static const float V2_TOP[3] = {208.f, 254.f, 300.f};

// ------------------------------------------------------------
// Module — DSP + state.
//
// Each channel is processed identically: out = in * envNorm * limit
// (or out = in with unity gain when the ENV input is unpatched).
// Per-channel display values (bar fill %, output peak, env-connected
// flag) are mirrored into member fields for the UI to read.
// ------------------------------------------------------------
struct VCA2 : Module {
    enum ParamId { PARAM_LIMIT1, PARAM_LIMIT2, PARAMS_LEN };
    enum InputId { INPUT_AUDIO1, INPUT_AUDIO2, INPUT_ENV1, INPUT_ENV2, INPUTS_LEN };
    enum OutputId { OUTPUT_1, OUTPUT_2, OUTPUTS_LEN };
    enum LightId { LIGHTS_LEN };

    // --------------------------------------------------------
    // Display data: envelope as % of the limit, and output peak.
    // Written by process(), read by the UI (a torn read is cosmetic).
    // --------------------------------------------------------
    float barFillPercent[2] = {0.f, 0.f};
    float outPeak[2] = {0.f, 0.f};
    bool envConn[2] = {false, false};

    // --------------------------------------------------------
    // Constructor — config params, inputs, outputs, bypass.
    // --------------------------------------------------------
    VCA2() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(PARAM_LIMIT1, 0.f, 2.f, 1.f, "Channel 1 gain limit", "x");
        configParam(PARAM_LIMIT2, 0.f, 2.f, 1.f, "Channel 2 gain limit", "x");

        configInput(INPUT_AUDIO1, "Channel 1 audio");
        configInput(INPUT_AUDIO2, "Channel 2 audio");
        configInput(INPUT_ENV1, "Channel 1 envelope CV (0 to 10 V)");
        configInput(INPUT_ENV2, "Channel 2 envelope CV (0 to 10 V)");

        configOutput(OUTPUT_1, "Channel 1");
        configOutput(OUTPUT_2, "Channel 2");

        // Pass-through when unpatched.
        configBypass(INPUT_AUDIO1, OUTPUT_1);
        configBypass(INPUT_AUDIO2, OUTPUT_2);
    }

    // --------------------------------------------------------
    // Per-sample DSP for both channels.
    //   * ENV unpatched  -> unity gain (out = in), full bar.
    //   * ENV patched    -> out = in * clamp(env/10,0,1) * limit;
    //                       bar fill = env / (limit * 10), capped at 1.
    //   * Output peak uses a one-pole decay (`md`) for the UI meter.
    // --------------------------------------------------------
    void process(const ProcessArgs& args) override {
        // Per-sample peak decay coefficient (about 200 ms time constant at 48 kHz).
        float md = 1.f - 5.f * args.sampleTime;
        for (int ch = 0; ch < 2; ++ch) {
            float in = inputs[INPUT_AUDIO1 + ch].getVoltage();
            bool conn = inputs[INPUT_ENV1 + ch].isConnected();
            float limit = params[PARAM_LIMIT1 + ch].getValue();
            float out, fill;

            if (!conn) {
                out = in;          // no envelope: unity gain, full bar
                fill = 100.f;
            } else {
                float env = inputs[INPUT_ENV1 + ch].getVoltage();
                float envNorm = rack::math::clamp(env / 10.f, 0.f, 1.f);
                out = in * envNorm * limit;
                float limitV = limit * 10.f;
                // Bar fill is relative to the limit so the marker and fill
                // share the same coordinate system.
                fill = (limitV > 0.f) ? rack::math::clamp(env / limitV, 0.f, 1.f) * 100.f : 0.f;
            }
            outputs[OUTPUT_1 + ch].setVoltage(out);
            barFillPercent[ch] = fill;
            envConn[ch] = conn;
            outPeak[ch] = std::max(std::fabs(out), outPeak[ch] * md);
        }
    }
};

// ------------------------------------------------------------
// Drawing helpers (anonymous namespace — UI-only)
// ------------------------------------------------------------
namespace {

namespace ag = agmm;

// ------------------------------------------------------------
// V2Panel — static panel chrome:
//   * title, display bezel
//   * six labelled jack cells (3 rows: audio, env, out)
// ------------------------------------------------------------
struct V2Panel : TransparentWidget {
    V2Panel() { box.size = Vec(V2_W, V2_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        ag::panelBase(vg, V2_W, V2_H, "VCA2");
        ag::bezel(vg, V2_DX, V2_DY, V2_DW, V2_DH);

        // Rows: audio / env / out; the out row is tinted (outputs).
        const char* names[3][2] = {{"AUDIO 1", "AUDIO 2"}, {"ENV 1", "ENV 2"}, {"OUT 1", "OUT 2"}};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 2; ++c)
                ag::cell(vg, V2_CX[c], V2_TOP[r], V2_CELL_W, V2_CELL_H, names[r][c], r == 2);
        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// V2Display — static screen background for the display:
//   * flat screen fill
//   * hairline under the header
//   * channel headers "CH 1" (cyan) and "CH 2" (violet)
// (The bars themselves are drawn by V2Bar on top.)
// ------------------------------------------------------------
struct V2Display : TransparentWidget {
    V2Display() {
        box.pos = Vec(V2_DX, V2_DY);
        box.size = Vec(V2_DW, V2_DH);
    }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        ag::screen(vg, 0.f, 0.f, V2_DW, V2_DH);

        // Header separator.
        nvgBeginPath(vg);
        nvgMoveTo(vg, 8.f, V2_HEAD_LINE);
        nvgLineTo(vg, V2_DW - 8.f, V2_HEAD_LINE);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Channel captions (display-local coordinates).
        ag::text(vg, V2_CX[0] - V2_DX, 8.f, "CH 1", 8.f, ag::accent(), ag::ALIGN_C, 2.f);
        ag::text(vg, V2_CX[1] - V2_DX, 8.f, "CH 2", 8.f, ag::violet(), ag::ALIGN_C, 2.f);
        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// V2Bar — interactive vertical limit bar for one channel.
// Behaviour:
//   * click or drag inside the bar sets the limit (1 at top -> 0,
//     2 at bottom edge -> top edge mapping: v = (1 - y/BH) * 2).
//   * double-click resets the limit to 1x.
// Rendering:
//   * track frame + ticks at every 0.5x
//   * fill from the bottom up to the limit marker only
//   * thin output peak meter on the right edge
//   * horizontal limit marker line + triangle handles
//   * readouts below: "LIM x.xxX" and "ENV nn%" (or "NO ENV")
// ------------------------------------------------------------
struct V2Bar : TransparentWidget {
    VCA2* module = nullptr;
    int ch = 0;
    float dragY = 0.f;  // vertical drag position in bar-local coordinates

    V2Bar(VCA2* m, int c) : module(m), ch(c) {
        box.pos = Vec(V2_CX[c] - V2_BW / 2.f, V2_BY);
        box.size = Vec(V2_BW, V2_BH);
    }

    // Convert dragY -> limit param and write it to the module.
    void apply() {
        if (!module) return;
        float v = (1.f - dragY / V2_BH) * 2.f;
        module->paramQuantities[VCA2::PARAM_LIMIT1 + ch]->setValue(rack::math::clamp(v, 0.f, 2.f));
    }

    // On click: capture the y position and apply immediately.
    void onButton(const ButtonEvent& e) override {
        if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            dragY = rack::math::clamp(e.pos.y, 0.f, V2_BH);
            apply();
            e.consume(this);
        }
    }

    // On drag: accumulate movement scaled by UI zoom, then apply.
    void onDragMove(const DragMoveEvent& e) override {
        float z = getAbsoluteZoom();
        if (z <= 0.f) z = 1.f;
        dragY = rack::math::clamp(dragY + e.mouseDelta.y / z, 0.f, V2_BH);
        apply();
    }

    // Double-click resets the limit to 1x.
    void onDoubleClick(const DoubleClickEvent& e) override {
        if (module) module->paramQuantities[VCA2::PARAM_LIMIT1 + ch]->setValue(1.f);
    }

    // Per-sample live drawing (layer 1).
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        NVGcolor col = ch == 0 ? ag::accent() : ag::violet();

        // Sample current module state (or demo values when previewing).
        float limit = module ? module->params[VCA2::PARAM_LIMIT1 + ch].getValue() : (ch == 0 ? 1.2f : 0.8f);
        float fillPct = module ? module->barFillPercent[ch] : (ch == 0 ? 70.f : 45.f);
        float peak = module ? module->outPeak[ch] : (ch == 0 ? 3.f : 2.f);
        bool conn = module ? module->envConn[ch] : true;
        limit = rack::math::clamp(limit, 0.f, 2.f);

        float markerY = V2_BH * (1.f - limit / 2.f);
        float fillH = (fillPct / 100.f) * (V2_BH - markerY);

        // Track
        ag::plotFrame(vg, 0.f, 0.f, V2_BW, V2_BH);

        // Ticks at every 0.5x
        nvgBeginPath(vg);
        for (int k = 1; k < 4; ++k) {
            float y = V2_BH * (1.f - (float)k * 0.5f / 2.f) + 0.5f;
            nvgMoveTo(vg, 0.f, y);
            nvgLineTo(vg, 6.f, y);
            nvgMoveTo(vg, V2_BW - 6.f, y);
            nvgLineTo(vg, V2_BW, y);
        }
        nvgStrokeColor(vg, nvgRGBA(0xc9, 0xd1, 0xdc, 70));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Fill (never above the limit marker), with a bright top edge.
        if (fillH > 0.5f) {
            nvgBeginPath(vg);
            nvgRect(vg, 3.f, V2_BH - fillH, V2_BW - 12.f, fillH);
            nvgFillColor(vg, ag::withA(col, 0.6f));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, 3.f, V2_BH - fillH, V2_BW - 12.f, 1.5f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 190));
            nvgFill(vg);
        }

        // Output peak meter (right edge of the bar).
        ag::meter(vg, V2_BW - 8.f, 3.f, 5.f, V2_BH - 6.f, peak / 5.f);

        // Limit marker line + side triangles.
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0.f, markerY);
        nvgLineTo(vg, V2_BW, markerY);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 240));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgMoveTo(vg, -5.f, markerY - 4.f);
        nvgLineTo(vg, 0.f, markerY);
        nvgLineTo(vg, -5.f, markerY + 4.f);
        nvgClosePath(vg);
        nvgMoveTo(vg, V2_BW + 5.f, markerY - 4.f);
        nvgLineTo(vg, V2_BW, markerY);
        nvgLineTo(vg, V2_BW + 5.f, markerY + 4.f);
        nvgClosePath(vg);
        nvgFillColor(vg, col);
        nvgFill(vg);

        // Readouts under the bar.
        char buf[32];
        snprintf(buf, sizeof(buf), "LIM %.2fX", limit);
        ag::text(vg, V2_BW / 2.f, V2_BH + 11.f, buf, 7.f, ag::textCol(), ag::ALIGN_C, 0.5f);
        if (conn) snprintf(buf, sizeof(buf), "ENV %.0f%%", fillPct);
        else      snprintf(buf, sizeof(buf), "NO ENV");
        ag::text(vg, V2_BW / 2.f, V2_BH + 23.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.5f);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// VCA2Widget — Rack widget.
//   * panel background, screws
//   * static display background + two interactive bars
//   * jacks laid out in three rows (audio / env / out), two columns
// ------------------------------------------------------------
struct VCA2Widget : ModuleWidget {
    VCA2Widget(VCA2* module) {
        setModule(module);
        box.size = Vec(V2_W, V2_H);

        addChild(new V2Panel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new V2Display());
        addChild(new V2Bar(module, 0));
        addChild(new V2Bar(module, 1));

        // Per-column jacks: audio / env / out.
        for (int c = 0; c < 2; ++c) {
            addInput(createInputCentered<DarkPJ301MPort>(Vec(V2_CX[c], V2_TOP[0] + V2_JACK_DY), module, VCA2::INPUT_AUDIO1 + c));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(V2_CX[c], V2_TOP[1] + V2_JACK_DY), module, VCA2::INPUT_ENV1 + c));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(V2_CX[c], V2_TOP[2] + V2_JACK_DY), module, VCA2::OUTPUT_1 + c));
        }
    }
};

Model* modelVCA2 = createModel<VCA2, VCA2Widget>("VCA2");