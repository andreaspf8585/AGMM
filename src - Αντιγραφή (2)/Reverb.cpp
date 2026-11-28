#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include "Knob_arcs.hpp"

// ============================================================================
// Reverb
// ============================================================================
//
// OVERVIEW
// --------
// Reverb is a 12 HP single-line feedback delay reverb with damping, drive and
// dry/wet mix. It is deliberately minimal: one mono delay line (up to ~4 s at
// 48 kHz) with a damped feedback path, a soft-clipping tanh write, an optional
// input drive saturator, and an output gain. The result is a warm, feedback-
// based ambience rather than a full FDN.
//
// Signal flow:
//   in -> (mixed dry) -> write into delay buffer (tanh of input + filtered*decay)
//     -> read delay line (linear interpolation) -> damping low-pass
//       -> feedback into write -> wet path -> optional drive -> dry/wet mix
//         -> output gain * tanh -> out
//
// The display shows a min/max scope of the output and a single mono level
// meter, plus header readouts for the delay time and mix.
//
// File structure:
//   1. Constants (MAX_DELAY, RV_SCOPE_N).
//   2. Reverb module: parameters, the feedback delay DSP, scope tracking.
//   3. Layout constants (12 HP = 180x380 px).
//   4. Anonymous-namespace UI: RvPanelBackground (static art) and
//      RvDisplayWidget (scope + meter + readouts).
//   5. ReverbWidget: places screws, display, six knobs and the I/O jacks.
//   6. Model registration.
// ============================================================================

static constexpr int MAX_DELAY = 192000;
static const int RV_SCOPE_N = 96;

// ============================================================
// Module – DSP unchanged, scope tracking added
// ============================================================
// Mono feedback-delay reverb with damping, drive and dry/wet mix.
struct Reverb : Module
{
    float delayBuffer[MAX_DELAY] = {};
    int writePos = 0;
    float lowpass = 0.f;

    // ---- display state ----
    float scopeMin[RV_SCOPE_N] = {};
    float scopeMax[RV_SCOPE_N] = {};
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakOut = 0.f;

    enum ParamId
    {
        TIME_PARAM,
        DAMP_PARAM,
        DELAY_MS_PARAM,
        DRIVE_PARAM,
        MIX_PARAM,
        OUTPUT_GAIN_PARAM,
        PARAMS_LEN
    };

    enum InputId
    {
        AUDIO_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        AUDIO_OUTPUT,
        OUTPUTS_LEN
    };

    // Constructor: configures the six parameters (decay feedback, damping,
    // delay time, drive, dry/wet and output gain), the single audio input,
    // the single audio output and the input->output bypass.
    Reverb()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);
        configParam(TIME_PARAM, 0.f, 0.995f, 0.5f, "Decay (Feedback)", "%", 0.f, 100.f);
        configParam(DAMP_PARAM, 0.f, 1.f, 0.5f, "Damping", " Hz", 40.f, 200.f); // cutoff = 200 * 40^damp
        configParam(DELAY_MS_PARAM, 0.f, 1000.f, 200.f, "Delay Time", " ms");
        configParam(DRIVE_PARAM, 0.f, 1.f, 0.f, "Drive", "%", 0.f, 100.f);
        configParam(MIX_PARAM, 0.f, 1.f, 0.5f, "Dry/Wet", "%", 0.f, 100.f);
        configParam(OUTPUT_GAIN_PARAM, 0.f, 3.f, 1.f, "Output Gain", "%", 0.f, 100.f);
        configInput(AUDIO_INPUT, "Audio In");
        configOutput(AUDIO_OUTPUT, "Audio Out");
        configBypass(AUDIO_INPUT, AUDIO_OUTPUT);
    }

    // Main audio callback: computes the damping coefficient from the DAMP
    // knob, reads the delay line with linear interpolation, applies the
    // damping low-pass, writes input + filtered feedback (soft-clipped) back
    // into the buffer, optionally saturates the wet signal, mixes dry/wet,
    // applies output gain and tanh limiting, and updates the scope/meter
    // data.
    void process(const ProcessArgs &args) override
    {
        float in = inputs[AUDIO_INPUT].getVoltage();
        float time = params[TIME_PARAM].getValue();
        float damp = params[DAMP_PARAM].getValue();
        float drive = params[DRIVE_PARAM].getValue();
        float mix = params[MIX_PARAM].getValue();
        float outGain = params[OUTPUT_GAIN_PARAM].getValue();

        // Damping low‑pass coefficient
        float minFreq = 200.f;
        float maxFreq = 8000.f;
        float cutoff = minFreq * std::pow(maxFreq / minFreq, damp);
        float dampingCoeff = 1.f - std::exp(-2.f * M_PI * cutoff / args.sampleRate);
        dampingCoeff = rack::math::clamp(dampingCoeff, 0.f, 1.f);

        // Delay line with linear interpolation
        float delayMs = params[DELAY_MS_PARAM].getValue();
        float delaySamplesFloat = delayMs * 0.001f * args.sampleRate;
        delaySamplesFloat = rack::math::clamp(delaySamplesFloat, 1.f, MAX_DELAY - 1.f);
        int delayInt = static_cast<int>(delaySamplesFloat);
        float frac = delaySamplesFloat - delayInt;

        int readPos0 = writePos - delayInt;
        if (readPos0 < 0)
            readPos0 += MAX_DELAY;
        int readPos1 = readPos0 - 1;
        if (readPos1 < 0)
            readPos1 += MAX_DELAY;

        float delayed0 = delayBuffer[readPos0];
        float delayed1 = delayBuffer[readPos1];
        float delayed = delayed0 + frac * (delayed1 - delayed0);

        lowpass += dampingCoeff * (delayed - lowpass);
        float filtered = lowpass;

        float feedbackIn = in + filtered * time;
        delayBuffer[writePos] = std::tanh(feedbackIn);
        writePos = (writePos + 1) % MAX_DELAY;

        float wet = filtered;

        if (drive > 0.f)
        {
            float driveAmount = drive * 4.f;
            float norm = 1.f / std::tanh(driveAmount);
            wet = std::tanh(driveAmount * wet) * norm;
        }

        float mixed = (1.f - mix) * in + mix * wet;
        float out = 5.f * std::tanh(mixed * outGain);

        outputs[AUDIO_OUTPUT].setVoltage(out);

        // ---- display data ----
        float decayRate = 1.f - 4.f * args.sampleTime;
        peakOut = std::max(std::fabs(out), peakOut * decayRate);

        if (scopeCount == 0)
            scopeBMin = scopeBMax = out;
        else
        {
            scopeBMin = std::min(scopeBMin, out);
            scopeBMax = std::max(scopeBMax, out);
        }
        int bucket = std::max(1, (int)(args.sampleRate * 0.0025f));
        if (++scopeCount >= bucket)
        {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % RV_SCOPE_N;
        }
    }
};

// ============================================================
// Layout constants (12 HP = 180 px wide)
//   display 150 x 56: header row, scope + meter; three rows of labelled knob cells;
//   an I/O row at the bottom
// ============================================================
static const float RV_PANEL_W = RACK_GRID_WIDTH * 12; // 180
static const float RV_PANEL_H = RACK_GRID_HEIGHT;     // 380

static const float RV_DISP_W = 150.f;
static const float RV_DISP_H = 56.f;
static const float RV_DISP_X = (RV_PANEL_W - RV_DISP_W) / 2.f;
static const float RV_DISP_Y = 34.f;
// scope plot (display-local) and meter column
static const float RV_PLOT_X = 8.f, RV_PLOT_Y = 17.f, RV_PLOT_W = 122.f, RV_PLOT_H = 34.f;
static const float RV_METER_X = 136.f, RV_METER_W = 6.f;

static const float RV_CELL_W = 76.f;
static const float RV_CELL_H = 62.f;
static const float RV_ROW_TOP[3] = {100.f, 166.f, 232.f};
static const float RV_COL[2] = {45.f, 135.f};
// Returns the knob centre y for control row `row` (0..2), 36 px below the
// cell's top edge.
static float rvKnobY(int row) { return RV_ROW_TOP[row] + 36.f; }

static const float RV_IO_SEC_Y = 306.f;
static const float RV_IO_TOP = 314.f;
static const float RV_IO_H = 42.f;
// Returns the jack centre y for the I/O row, 28 px below the cell's top edge.
static float rvPortY() { return RV_IO_TOP + 28.f; }

// ============================================================
// Panel + display (anonymous namespace)
// ============================================================
namespace
{
    namespace ag = agmm;

    // Cell table: 6 controls, 2 columns x 3 rows
    // Metadata for one knob cell: display name plus the module parameter ID.
    struct RvCellDef
    {
        const char *name;
        int param;
    };
    const RvCellDef RV_CELLS[6] = {
        {"DECAY", Reverb::TIME_PARAM},
        {"DAMP", Reverb::DAMP_PARAM},
        {"DELAY", Reverb::DELAY_MS_PARAM},
        {"DRIVE", Reverb::DRIVE_PARAM},
        {"MIX", Reverb::MIX_PARAM},
        {"GAIN", Reverb::OUTPUT_GAIN_PARAM},
    };

    // ------------------------------------------------------------
    // Panel background
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the six knob cells (2x3)
    // and the I/O row with its INPUT/OUTPUT cells (output tinted).
    struct RvPanelBackground : TransparentWidget
    {
        RvPanelBackground() { box.size = Vec(RV_PANEL_W, RV_PANEL_H); }

        // Draws all static panel art: base panel, bezel, the six knob cells,
        // the I/O section header and its two jack cells.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, RV_PANEL_W, RV_PANEL_H, "REVERB");
            ag::bezel(vg, RV_DISP_X, RV_DISP_Y, RV_DISP_W, RV_DISP_H);

            // Control cells: name on top, knob below
            for (int i = 0; i < 6; ++i)
                ag::cell(vg, RV_COL[i % 2], RV_ROW_TOP[i / 2], RV_CELL_W, RV_CELL_H, RV_CELLS[i].name);

            // I/O row (output tinted)
            ag::sectionHeader(vg, RV_PANEL_W, RV_IO_SEC_Y, "I / O", 14.f);
            ag::cell(vg, RV_COL[0], RV_IO_TOP, RV_CELL_W, RV_IO_H, "INPUT");
            ag::cell(vg, RV_COL[1], RV_IO_TOP, RV_CELL_W, RV_IO_H, "OUTPUT", true);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Scope display + single mono level meter
    // ------------------------------------------------------------
    // Live display: draws the output min/max scope, a single mono level
    // meter and the DLY / MIX header readouts. Uses demo data when there is
    // no module (module browser).
    struct RvDisplayWidget : TransparentWidget
    {
        Reverb *module = nullptr;

        // Constructor: positions the widget under the display bezel.
        RvDisplayWidget(Reverb *m) : module(m)
        {
            box.pos = Vec(RV_DISP_X, RV_DISP_Y);
            box.size = Vec(RV_DISP_W, RV_DISP_H);
        }

        // Static layer: fills the screen, draws the header divider hairline,
        // the scope plot frame and its grid (quarter divisions plus a zero
        // line).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, RV_DISP_W, RV_DISP_H);

            // header hairline: the DLY / MIX readouts live above it
            nvgBeginPath(vg);
            nvgMoveTo(vg, 6.f, 14.5f);
            nvgLineTo(vg, RV_DISP_W - 6.f, 14.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, RV_PLOT_X, RV_PLOT_Y, RV_PLOT_W, RV_PLOT_H);

            // Grid: quarters across, zero line
            nvgBeginPath(vg);
            for (int k = 1; k < 4; ++k)
            {
                float x = RV_PLOT_X + std::floor(RV_PLOT_W * k / 4.f) + 0.5f;
                nvgMoveTo(vg, x, RV_PLOT_Y);
                nvgLineTo(vg, x, RV_PLOT_Y + RV_PLOT_H);
            }
            nvgMoveTo(vg, RV_PLOT_X, RV_PLOT_Y + RV_PLOT_H / 2.f + 0.5f);
            nvgLineTo(vg, RV_PLOT_X + RV_PLOT_W, RV_PLOT_Y + RV_PLOT_H / 2.f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the output min/max scope from the module's
        // ring buffer (or a demo shape), the mono level meter and the DLY /
        // MIX header readouts.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            const float mid = RV_PLOT_H / 2.f;
            const float amp = mid - 2.f;
            const float sw = RV_PLOT_W;
            const float colW = sw / (float)RV_SCOPE_N;

            nvgSave(vg);
            nvgTranslate(vg, RV_PLOT_X, RV_PLOT_Y);
            nvgScissor(vg, 0, 0, sw, RV_PLOT_H);
            nvgBeginPath(vg);
            for (int i = 0; i < RV_SCOPE_N; ++i)
            {
                float vmin, vmax;
                if (module)
                {
                    int idx = (module->scopeHead + i) % RV_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                }
                else
                {
                    float t = (float)i / RV_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - std::min(std::max(vmax / 5.f, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / 5.f, -1.f), 1.f) * amp;
                float h = std::max(y1 - y0, 1.f);
                nvgRect(vg, i * colW, y0, std::max(colW - 0.4f, 1.f), h);
            }
            nvgFillColor(vg, ag::bright());
            nvgFill(vg);
            nvgRestore(vg);

            // Mono level meter (right of the scope)
            float peak = module ? module->peakOut : 2.4f;
            ag::meter(vg, RV_METER_X, RV_PLOT_Y, RV_METER_W, RV_PLOT_H, std::min(peak / 5.f, 1.f));

            // Header readouts
            char buf[32];
            float dly = module ? module->params[Reverb::DELAY_MS_PARAM].getValue() : 200.f;
            float mx = module ? module->params[Reverb::MIX_PARAM].getValue() : 0.5f;
            snprintf(buf, sizeof(buf), "DLY %3.0fMS", dly);
            ag::text(vg, 8.f, 7.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "MIX %3.0f%%", mx * 100.f);
            ag::text(vg, RV_DISP_W - 8.f, 7.f, buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================
// Widget: adds the static panel, screws, the live display, the six knobs
// (2x3 grid) and the input/output jacks.
// ============================================================
struct ReverbWidget : ModuleWidget
{
    // Builds the widget tree: static background, screws, display, the six
    // knobs positioned from the RV_CELLS table, and the input/output jacks.
    ReverbWidget(Reverb *module)
    {
        setModule(module);
        
        box.size = Vec(RV_PANEL_W, RV_PANEL_H);

        addChild(new RvPanelBackground());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new RvDisplayWidget(module));

        // 6 knobs: 2 columns x 3 rows, knob 36 px below each cell's top edge
        for (int i = 0; i < 6; ++i)
        {
            int row = i / 2, col = i % 2;
            addParam(createParamCentered<RoundBlackKnob>(Vec(RV_COL[col], rvKnobY(row)), module, RV_CELLS[i].param));
        }

        // I/O jacks: input left, output right
        addInput(createInputCentered<DarkPJ301MPort>(Vec(RV_COL[0], rvPortY()), module, Reverb::AUDIO_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(RV_COL[1], rvPortY()), module, Reverb::AUDIO_OUTPUT));
        agmm::addKnobArcs(this, 6.f);
    }

    void appendContextMenu(Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        // ... your other menu items ...
    }
};

Model *modelReverb = createModel<Reverb, ReverbWidget>("Reverb");