#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <climits>
#include <cstring>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// ============================================================================
// Quantara
// ============================================================================
//
// OVERVIEW
// --------
// Quantara is a 24 HP polyphonic pitch quantizer. It takes a polyphonic
// 1 V/oct input and snaps each voice to the nearest (or nearest-above /
// nearest-below) note of a selectable scale, with optional glide and a
// semitone transpose. Each voice emits a short note-change trigger whenever
// its quantized note changes.
//
// Quantization can either run continuously (unclocked) or as a sample & hold
// on every rising clock edge. The module supports:
//   - 16 built-in scales plus a user-editable custom scale.
//   - Root transposition via knob + 1 V/oct CV.
//   - Scale selection via knob + 0..10 V CV.
//   - Semitone transpose via knob + 1 V/oct CV.
//   - Glide (portamento) between successive output pitches.
//   - Three quantize modes: nearest, down (floor), up (ceil).
//
// The display combines:
//   - a 12-note pitch-class wheel showing the scale polygon, root note,
//     active voices and their correction lines,
//   - a transfer staircase plot of input-vs-output semitones with the
//     active channel highlighted plus a short trail,
//   - a clickable mini keyboard for toggling notes of the custom scale
//     (clicking a key edits the custom scale and switches to it).
//
// File structure:
//   1. Constants and layout (24 HP = 360x380 px).
//   2. Scale tables, note names and the shared quantizer maths.
//   3. Quantara module: parameters / CV, polyphonic quantize+glide DSP,
//      caching, JSON persistence, display snapshots.
//   4. Anonymous-namespace UI: colour helpers, QZPanelBackground,
//      QZKnobArcs, QZDisplayWidget (wheel + staircase + keyboard).
//   5. QuantaraWidget: places controls, jacks and the display, plus the
//      right-click menu (scale, quantize mode, custom-scale actions).
//   6. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Constants
// ------------------------------------------------------------
static const int QZ_NUM_SCALES = 17; // 16 presets + custom
static const int QZ_MAX_CH = 16;
static const int QZ_TRAIL_N = 40;
static const float QZ_TRAIL_HZ = 60.f;

static const float QZ_PANEL_W = RACK_GRID_WIDTH * 24; // 360
static const float QZ_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (228 x 140): header row, wheel + transfer plot, keyboard, caption row
static const float QZ_DISP_W = 228.f;
static const float QZ_DISP_H = 140.f;
static const float QZ_DISP_X = (QZ_PANEL_W - QZ_DISP_W) / 2.f; // 66
static const float QZ_DISP_Y = 44.f;

// Pitch-class wheel (display-local); note labels sit 9 px outside the ring
static const float QZ_WC_X = 44.f, QZ_WC_Y = 55.5f, QZ_WC_R = 26.f;

// Transfer plot (display-local)
static const float QZ_PX0 = 100.f, QZ_PX1 = 220.f;
static const float QZ_PY0 = 19.f, QZ_PY1 = 93.f;

// Keyboard (display-local)
static const float QZ_KB_X0 = 12.f, QZ_KB_X1 = 216.f;
static const float QZ_KB_Y0 = 100.f, QZ_KB_WH = 24.f, QZ_KB_BH = 14.f, QZ_KB_BW = 17.f;

// Side cells: ROOT, SCALE (left) / SHIFT, GLIDE (right); knob 36 px below the cell top
static const float QZ_SIDE_W = 50.f, QZ_SIDE_H = 62.f;
static const float QZ_KNOB_X[4] = {33.f, 33.f, 327.f, 327.f};
static const float QZ_SIDE_TOP[4] = {44.f, 122.f, 44.f, 122.f};
static const float QZ_KNOB_Y[4] = {80.f, 158.f, 80.f, 158.f};

// CV row and I/O row: 52 x 52 cells, jack 32 px below the cell top
static const float QZ_CELL_W = 52.f, QZ_CELL_H = 52.f;
static const float QZ_CV_SEC_Y = 204.f;
static const float QZ_CV_TOP = 214.f;
static const float QZ_CV_X[5] = {60.f, 120.f, 180.f, 240.f, 300.f};
static const float QZ_IO_SEC_Y = 282.f;
static const float QZ_IO_TOP = 292.f;
static const float QZ_IO_X[3] = {120.f, 180.f, 240.f};
// Returns the jack centre y for the I/O cell whose top edge is `top`.
static inline float qzJackY(float top) { return top + 32.f; }

// ------------------------------------------------------------
// Scales and quantizer maths (shared by DSP and display)
// ------------------------------------------------------------
static const char *const QZ_NOTE_NAMES[12] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

static const char *const QZ_SCALE_SHORT[QZ_NUM_SCALES] = {
    "CHROMATIC", "MAJOR", "MINOR", "HARM MINOR", "MEL MINOR", "DORIAN", "PHRYGIAN",
    "LYDIAN", "MIXOLYDIAN", "LOCRIAN", "MAJ PENT", "MIN PENT", "BLUES", "WHOLE TONE",
    "DIMINISHED", "PHRYG DOM", "CUSTOM"};

static const char *const QZ_SCALE_FULL[QZ_NUM_SCALES] = {
    "Chromatic", "Major (Ionian)", "Natural Minor (Aeolian)", "Harmonic Minor",
    "Melodic Minor", "Dorian", "Phrygian", "Lydian", "Mixolydian", "Locrian",
    "Major Pentatonic", "Minor Pentatonic", "Blues", "Whole Tone",
    "Diminished (W-H)", "Phrygian Dominant", "Custom"};

// Bit i = scale contains interval i (semitones above the root)
static const uint16_t QZ_SCALE_MASK[QZ_NUM_SCALES - 1] = {
    0xFFF, // chromatic
    0xAB5, // 0 2 4 5 7 9 11
    0x5AD, // 0 2 3 5 7 8 10
    0x9AD, // 0 2 3 5 7 8 11
    0xAAD, // 0 2 3 5 7 9 11
    0x6AD, // 0 2 3 5 7 9 10
    0x5AB, // 0 1 3 5 7 8 10
    0xAD5, // 0 2 4 6 7 9 11
    0x6B5, // 0 2 4 5 7 9 10
    0x56B, // 0 1 3 5 6 8 10
    0x295, // 0 2 4 7 9
    0x4A9, // 0 3 5 7 10
    0x4E9, // 0 3 5 6 7 10
    0x555, // 0 2 4 6 8 10
    0xB6D, // 0 2 3 5 6 8 9 11
    0x5B3  // 0 1 4 5 7 8 10
};

// Returns a positive modulo into 0..b-1 (works for negative a).
static inline int qzMod(int a, int b) { return ((a % b) + b) % b; }

// Rotates a 12-bit relative mask by `root` semitones (circular shift,
// wrapping bits around), giving a mask of absolute pitch classes.
static inline uint16_t qzRot(uint16_t rel, int root)
{
    root = qzMod(root, 12);
    return (uint16_t)((((int)rel << root) | ((int)rel >> (12 - root))) & 0xFFF);
}

// Returns the number of set bits in a 12-bit mask.
static inline int qzCount(uint16_t m)
{
    int n = 0;
    for (int i = 0; i < 12; ++i)
        n += (m >> i) & 1;
    return n;
}

// n = pitch in semitones (float), mask = absolute pitch classes, mode 0 nearest / 1 down / 2 up
// Quantizes a (fractional) semitone value to the nearest allowed note, or
// the closest allowed note at or below (mode 1) / at or above (mode 2) it.
// If the mask is empty the input is simply rounded (bypass behaviour).
static inline int qzQuant(float n, uint16_t mask, int mode)
{
    if (mask == 0)
        return (int)std::lround(n);
    int base = (int)std::floor(n);
    int best = base;
    float bd = 1e9f;
    for (int cand = base - 13; cand <= base + 13; ++cand)
    {
        if (!((mask >> qzMod(cand, 12)) & 1))
            continue;
        float d = (float)cand - n;
        if (mode == 1)
        {
            if (d <= 1e-3f)
                best = cand;
        }
        else if (mode == 2)
        {
            if (d >= -1e-3f)
            {
                best = cand;
                break;
            }
        }
        else
        {
            float ad = std::fabs(d);
            if (ad < bd)
            {
                bd = ad;
                best = cand;
            }
        }
    }
    return best;
}

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// Polyphonic pitch quantizer with glide, transpose, clocked sample & hold
// and a user-editable custom scale.
struct Quantara : Module
{
    enum ParamId
    {
        ROOT_PARAM,
        SCALE_PARAM,
        SHIFT_PARAM,
        GLIDE_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        IN_INPUT,
        CLOCK_INPUT,
        ROOT_CV_INPUT,
        SCALE_CV_INPUT,
        SHIFT_CV_INPUT,
        GLIDE_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUT_OUTPUT,
        TRIG_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    enum Mode
    {
        MODE_NEAREST,
        MODE_DOWN,
        MODE_UP,
        NUM_MODES
    };

    // Settings (saved)
    uint16_t userMask = 0xAB5; // custom scale, relative to the root
    int mode = MODE_NEAREST;

    // Audio state
    float heldIn[QZ_MAX_CH] = {};
    float targetQ[QZ_MAX_CH] = {};
    float outS[QZ_MAX_CH] = {};
    int lastNote[QZ_MAX_CH];
    float cacheIn[QZ_MAX_CH] = {};
    int cacheNote[QZ_MAX_CH] = {};
    float cacheQ[QZ_MAX_CH] = {};
    bool cacheOk[QZ_MAX_CH] = {};
    uint32_t cacheKey = 0xFFFFFFFFu;
    int lastCh = 0;

    dsp::SchmittTrigger clkTrig;
    dsp::PulseGenerator trigs[QZ_MAX_CH];

    // Display data
    int vRoot = 0, vScale = 1, vShift = 0, vNch = 0;
    uint16_t vMask = 0xAB5;
    bool vClocked = false;
    int vNote[QZ_MAX_CH] = {};
    float pcFlash[12] = {};
    float clockFlash = 0.f;
    float trailIn[QZ_TRAIL_N] = {};
    float trailOut[QZ_TRAIL_N] = {};
    int trailHead = 0;
    int trailCount = 0;
    float trailTimer = 0.f;

    // Constructor: configures the root / scale / shift / glide parameters
    // (with the display names built from the scale tables), the six inputs,
    // the stereo (poly) outputs and the bypass, then marks every voice as
    // not yet quantized.
    Quantara()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        std::vector<std::string> rootNames;
        for (int i = 0; i < 12; ++i)
            rootNames.push_back(QZ_NOTE_NAMES[i]);
        configSwitch(ROOT_PARAM, 0.f, 11.f, 0.f, "Root", rootNames);

        std::vector<std::string> scaleNames;
        for (int i = 0; i < QZ_NUM_SCALES; ++i)
            scaleNames.push_back(QZ_SCALE_FULL[i]);
        configSwitch(SCALE_PARAM, 0.f, (float)(QZ_NUM_SCALES - 1), 1.f, "Scale", scaleNames);

        configParam(SHIFT_PARAM, -12.f, 12.f, 0.f, "Transpose", " semitones");
        paramQuantities[SHIFT_PARAM]->snapEnabled = true;
        configParam(GLIDE_PARAM, 0.f, 1.f, 0.f, "Glide", "%", 0.f, 100.f);

        configInput(IN_INPUT, "Pitch (1 V/oct, polyphonic)");
        configInput(CLOCK_INPUT, "Clock (sample & hold; unpatched = continuous)");
        configInput(ROOT_CV_INPUT, "Root CV (1 V/oct)");
        configInput(SCALE_CV_INPUT, "Scale CV (0-10 V)");
        configInput(SHIFT_CV_INPUT, "Transpose CV (1 V/oct)");
        configInput(GLIDE_CV_INPUT, "Glide CV (0-10 V)");

        configOutput(OUT_OUTPUT, "Quantized pitch (1 V/oct, polyphonic)");
        configOutput(TRIG_OUTPUT, "Note change trigger (polyphonic)");

        configBypass(IN_INPUT, OUT_OUTPUT);

        for (int c = 0; c < QZ_MAX_CH; ++c)
            lastNote[c] = INT_MIN;
    }

    // Rack reset hook: clears the custom scale, quantize mode, per-voice
    // DSP/cache state, flashes and trail buffer.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        userMask = 0xAB5;
        mode = MODE_NEAREST;
        std::memset(heldIn, 0, sizeof(heldIn));
        std::memset(targetQ, 0, sizeof(targetQ));
        std::memset(outS, 0, sizeof(outS));
        std::memset(cacheOk, 0, sizeof(cacheOk));
        std::memset(pcFlash, 0, sizeof(pcFlash));
        for (int c = 0; c < QZ_MAX_CH; ++c)
            lastNote[c] = INT_MIN;
        lastCh = 0;
        trailHead = trailCount = 0;
        trailTimer = 0.f;
    }

    // Returns the relative (root-agnostic) 12-bit scale mask for scale index
    // `s`: a built-in mask for presets 0..15, or the custom mask otherwise.
    uint16_t relMaskFor(int s) const
    {
        return (s >= 0 && s < QZ_NUM_SCALES - 1) ? QZ_SCALE_MASK[s] : (uint16_t)(userMask & 0xFFF);
    }

    // Toggle one absolute pitch class; edits the custom scale
    // Flips the bit for absolute pitch class `absPc` in the custom scale
    // (relative to the current display root) and switches the scale
    // parameter to "Custom". Called by clicking the on-display keyboard.
    void toggleNote(int absPc)
    {
        int bit = qzMod(absPc - vRoot, 12);
        uint16_t rel = relMaskFor(vScale);
        rel ^= (uint16_t)(1 << bit);
        userMask = rel & 0xFFF;
        params[SCALE_PARAM].setValue((float)(QZ_NUM_SCALES - 1));
    }

    // Main audio callback: reads all parameters + CVs, computes the absolute
    // scale mask and a cache key, handles the optional clock sample & hold,
    // then per channel quantizes the input (with caching), applies glide,
    // adds the transpose and writes the pitch + trigger outputs. Also
    // updates the display snapshot and trail buffer.
    void process(const ProcessArgs &args) override
    {
        const float dt = args.sampleTime;

        // ---- parameters + CV ----
        int rootP = (int)std::round(params[ROOT_PARAM].getValue());
        int root = qzMod(rootP + (int)std::round(inputs[ROOT_CV_INPUT].getVoltage() * 12.f), 12);

        float sc = params[SCALE_PARAM].getValue() + inputs[SCALE_CV_INPUT].getVoltage() * 0.1f * (float)(QZ_NUM_SCALES - 1);
        int sIdx = rack::math::clamp((int)std::round(sc), 0, QZ_NUM_SCALES - 1);

        int shift = rack::math::clamp(
            (int)std::round(params[SHIFT_PARAM].getValue() + inputs[SHIFT_CV_INPUT].getVoltage() * 12.f),
            -48, 48);

        float glide = rack::math::clamp(
            params[GLIDE_PARAM].getValue() + inputs[GLIDE_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
        float tau = (glide < 0.002f) ? 0.f : 0.002f * std::pow(500.f, glide); // 2 ms .. 1 s
        float a = (tau > 0.f) ? (1.f - std::exp(-dt / tau)) : 1.f;

        uint16_t mask = qzRot(relMaskFor(sIdx), root);
        uint32_t key = (uint32_t)mask | ((uint32_t)mode << 12);
        if (key != cacheKey)
        {
            cacheKey = key;
            std::memset(cacheOk, 0, sizeof(cacheOk));
        }

        // ---- clock ----
        bool clocked = inputs[CLOCK_INPUT].isConnected();
        bool edge = clkTrig.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f);
        if (edge)
            clockFlash = 1.f;

        // ---- channels ----
        int nIn = inputs[IN_INPUT].isConnected() ? inputs[IN_INPUT].getChannels() : 0;
        int nOut = std::max(nIn, 1);
        outputs[OUT_OUTPUT].setChannels(nOut);
        outputs[TRIG_OUTPUT].setChannels(nOut);

        for (int c = 0; c < nOut; ++c)
        {
            if (nIn == 0)
            {
                outputs[OUT_OUTPUT].setVoltage(0.f, c);
                outputs[TRIG_OUTPUT].setVoltage(0.f, c);
                continue;
            }

            float vin = inputs[IN_INPUT].getVoltage(c);
            bool fresh = (c >= lastCh);

            if (!clocked || edge || fresh)
            {
                if (!cacheOk[c] || cacheIn[c] != vin)
                {
                    int n = qzQuant(vin * 12.f, mask, mode);
                    cacheNote[c] = n;
                    cacheQ[c] = (mask == 0) ? vin : (float)n / 12.f;
                    cacheIn[c] = vin;
                    cacheOk[c] = true;
                }
                heldIn[c] = vin;
                targetQ[c] = cacheQ[c];
                int note = cacheNote[c];
                vNote[c] = note;

                if (!fresh && lastNote[c] != INT_MIN && note != lastNote[c])
                {
                    trigs[c].trigger(1e-3f);
                    pcFlash[qzMod(note, 12)] = 1.f;
                }
                lastNote[c] = note;
                if (fresh)
                    outS[c] = targetQ[c];
            }

            outS[c] += (targetQ[c] - outS[c]) * a;
            outputs[OUT_OUTPUT].setVoltage(outS[c] + (float)shift / 12.f, c);
            outputs[TRIG_OUTPUT].setVoltage(trigs[c].process(dt) ? 10.f : 0.f, c);
        }
        lastCh = nIn;

        // ---- display data ----
        vRoot = root;
        vScale = sIdx;
        vShift = shift;
        vMask = mask;
        vNch = nIn;
        vClocked = clocked;

        for (int i = 0; i < 12; ++i)
            pcFlash[i] = std::max(0.f, pcFlash[i] - dt * 5.f);
        clockFlash = std::max(0.f, clockFlash - dt * 6.f);

        const float step = 1.f / QZ_TRAIL_HZ;
        trailTimer += dt;
        if (trailTimer >= step)
        {
            trailTimer -= step;
            if (trailTimer > step)
                trailTimer = 0.f;
            if (nIn > 0)
            {
                trailIn[trailHead] = heldIn[0] * 12.f;
                trailOut[trailHead] = (float)vNote[0];
                trailHead = (trailHead + 1) % QZ_TRAIL_N;
                if (trailCount < QZ_TRAIL_N)
                    trailCount++;
            }
        }
    }

    // Serialises the custom scale mask and the quantize mode.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "userMask", json_integer(userMask));
        json_object_set_new(root, "mode", json_integer(mode));
        return root;
    }

    // Restores the custom scale mask (12-bit) and the quantize mode from a
    // patch, clamping each to its valid range.
    void dataFromJson(json_t *root) override
    {
        json_t *j = json_object_get(root, "userMask");
        if (j && json_is_integer(j))
            userMask = (uint16_t)(json_integer_value(j) & 0xFFF);
        j = json_object_get(root, "mode");
        if (j && json_is_integer(j))
            mode = rack::math::clamp((int)json_integer_value(j), 0, (int)NUM_MODES - 1);
    }
};

// ------------------------------------------------------------
// Theme + drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace
{
    namespace ag = agmm;

    // Returns the bright highlight colour used for active notes / line work.
    NVGcolor colBright() { return nvgRGB(0x7a, 0xec, 0xff); }
    // Returns the warm accent colour used for the root note / input markers.
    NVGcolor colWarm() { return ag::warm(); }

    const int QZ_WPC[7] = {0, 2, 4, 5, 7, 9, 11}; // white keys
    const int QZ_BPC[5] = {1, 3, 6, 8, 10};       // black keys
    const int QZ_BIDX[5] = {0, 1, 3, 4, 5};       // white key to the left of each black key

    // Maps a semitone value t (0..12) to an x pixel inside the transfer plot.
    inline float qzPx(float t) { return QZ_PX0 + t / 12.f * (QZ_PX1 - QZ_PX0); }
    // Maps a semitone value v (-1..13) to a y pixel inside the transfer plot
    // (higher notes higher on the plot).
    inline float qzPy(float v) { return QZ_PY1 - (v + 1.f) / 14.f * (QZ_PY1 - QZ_PY0); }

    // ------------------------------------------------------------
    // Panel background: title, display frame, labelled plates
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the four side knob cells,
    // the CV INPUTS row, the I/O row (outputs tinted), and the footer text.
    struct QZPanelBackground : TransparentWidget
    {
        QZPanelBackground() { box.size = Vec(QZ_PANEL_W, QZ_PANEL_H); }

        // Draws all static panel art: base panel, bezel, side ROOT/SCALE/
        // SHIFT/GLIDE cells, the five CV INPUTS cells, the three I/O cells
        // (outputs tinted) and the "1 V/OCT - POLYPHONIC - 12 NOTE" footer.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, QZ_PANEL_W, QZ_PANEL_H, "QUANTARA");
            ag::bezel(vg, QZ_DISP_X, QZ_DISP_Y, QZ_DISP_W, QZ_DISP_H);

            // ---- side knobs ----
            const char *knobL[4] = {"ROOT", "SCALE", "SHIFT", "GLIDE"};
            for (int k = 0; k < 4; ++k)
                ag::cell(vg, QZ_KNOB_X[k], QZ_SIDE_TOP[k], QZ_SIDE_W, QZ_SIDE_H, knobL[k]);

            // ---- CV inputs ----
            ag::sectionHeader(vg, QZ_PANEL_W, QZ_CV_SEC_Y, "CV INPUTS");
            const char *cv[5] = {"CLOCK", "ROOT", "SCALE", "SHIFT", "GLIDE"};
            for (int i = 0; i < 5; ++i)
                ag::cell(vg, QZ_CV_X[i], QZ_CV_TOP, QZ_CELL_W, QZ_CELL_H, cv[i]);

            // ---- I/O (outputs tinted) ----
            ag::sectionHeader(vg, QZ_PANEL_W, QZ_IO_SEC_Y, "I / O");
            const char *io[3] = {"IN", "OUT", "TRIG"};
            for (int i = 0; i < 3; ++i)
                ag::cell(vg, QZ_IO_X[i], QZ_IO_TOP, QZ_CELL_W, QZ_CELL_H, io[i], i > 0);

            ag::text(vg, QZ_PANEL_W / 2.f, 354.f, "1 V/OCT  -  POLYPHONIC  -  12 NOTE", 7.f,
                     ag::withA(ag::dim(), 0.7f), ag::ALIGN_C, 1.f);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Live value arcs around the knobs (flat, hairline-width)
    // ------------------------------------------------------------
    // Draws a track arc around each of the four side knobs plus a coloured
    // arc showing the current value; the SHIFT arc is bipolar (from centre,
    // warm when negative).
    struct QZKnobArcs : TransparentWidget
    {
        Quantara *module = nullptr;

        QZKnobArcs(Quantara *m) : module(m) { box.size = Vec(QZ_PANEL_W, QZ_PANEL_H); }

        // Draws the value arcs for the four knobs (skipping any arc whose
        // change from the origin is negligible).
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            const float R = 17.2f;
            const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
            const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

            for (int k = 0; k < 4; ++k)
            {
                float cx = QZ_KNOB_X[k], cy = QZ_KNOB_Y[k];

                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, R, a0, a1, NVG_CW);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
                nvgStrokeWidth(vg, 2.f);
                nvgLineCap(vg, NVG_ROUND);
                nvgStroke(vg);

                if (!module)
                    continue;
                float v = module->paramQuantities[k]->getScaledValue(); // 0..1
                bool bipolar = (k == Quantara::SHIFT_PARAM);
                float from = bipolar ? 0.5f : 0.f;
                float lo = std::min(from, v), hi = std::max(from, v);
                if (hi - lo < 0.004f)
                    continue;

                float s = a0 + (a1 - a0) * lo;
                float e = a0 + (a1 - a0) * hi;
                NVGcolor col = (bipolar && v < 0.5f) ? colWarm() : ag::accent();

                nvgBeginPath(vg);
                nvgArc(vg, cx, cy, R, s, e, NVG_CW);
                nvgStrokeColor(vg, ag::withA(col, 0.95f));
                nvgStrokeWidth(vg, 2.f);
                nvgStroke(vg);
            }
        }
    };

    // ------------------------------------------------------------
    // Display: pitch-class wheel + transfer staircase + clickable keyboard
    // ------------------------------------------------------------
    // Live display combining the pitch-class wheel, the transfer staircase
    // and the clickable scale keyboard. Falls back to demo data in the
    // module browser.
    struct QZDisplayWidget : Widget
    {
        Quantara *module = nullptr;
        int hoverKey = -1;

        QZDisplayWidget(Quantara *m) : module(m)
        {
            box.pos = Vec(QZ_DISP_X, QZ_DISP_Y);
            box.size = Vec(QZ_DISP_W, QZ_DISP_H);
        }

        // ---- keyboard hit test ----
        // Returns the pitch class under point p, or -1 if p is not over the
        // keyboard (black keys are checked first, then the white key under
        // the horizontal position).
        int keyAt(Vec p) const
        {
            const float ww = (QZ_KB_X1 - QZ_KB_X0) / 7.f;
            if (p.y < QZ_KB_Y0 || p.y > QZ_KB_Y0 + QZ_KB_WH || p.x < QZ_KB_X0 || p.x > QZ_KB_X1)
                return -1;
            if (p.y <= QZ_KB_Y0 + QZ_KB_BH)
            {
                for (int j = 0; j < 5; ++j)
                {
                    float bx = QZ_KB_X0 + (float)(QZ_BIDX[j] + 1) * ww;
                    if (std::fabs(p.x - bx) <= QZ_KB_BW / 2.f)
                        return QZ_BPC[j];
                }
            }
            int wi = rack::math::clamp((int)((p.x - QZ_KB_X0) / ww), 0, 6);
            return QZ_WPC[wi];
        }

        // ---- mouse ----
        // Left-click over a key toggles that pitch class in the custom
        // scale (and switches the module to the custom scale).
        void onButton(const ButtonEvent &e) override
        {
            if (module && e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS)
            {
                int k = keyAt(e.pos);
                if (k >= 0)
                {
                    module->toggleNote(k);
                    e.consume(this);
                    return;
                }
            }
            Widget::onButton(e);
        }

        // Updates the hover key (used for the hovered-key highlight) and
        // consumes the event when over a key.
        void onHover(const HoverEvent &e) override
        {
            hoverKey = keyAt(e.pos);
            if (hoverKey >= 0)
                e.consume(this);
            else
                Widget::onHover(e);
        }

        // Clears the hover key when the pointer leaves the widget.
        void onLeave(const LeaveEvent &e) override
        {
            hoverKey = -1;
            Widget::onLeave(e);
        }

        // ---- static background ----
        // Draws the display screen, the header divider, the wheel's two
        // rings, the transfer plot frame, the plot grids (semitone and
        // octave lines), and the keyboard backdrop.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, QZ_DISP_W, QZ_DISP_H);

            // Header hairline: the readouts live above it
            nvgBeginPath(vg);
            nvgMoveTo(vg, 6.f, 14.5f);
            nvgLineTo(vg, QZ_DISP_W - 6.f, 14.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Wheel rings
            nvgBeginPath(vg);
            nvgCircle(vg, QZ_WC_X, QZ_WC_Y, QZ_WC_R);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 42));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, QZ_WC_X, QZ_WC_Y, QZ_WC_R - 9.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
            nvgStroke(vg);

            // Plot frame
            ag::plotFrame(vg, QZ_PX0, QZ_PY0, QZ_PX1 - QZ_PX0, QZ_PY1 - QZ_PY0);

            // Plot grid: one line per semitone, brighter on octaves
            for (int t = 1; t < 12; ++t)
            {
                float x = qzPx((float)t);
                nvgBeginPath(vg);
                nvgMoveTo(vg, x + 0.5f, QZ_PY0);
                nvgLineTo(vg, x + 0.5f, QZ_PY1);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (t == 6) ? 20 : 9));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            for (int v = -1; v <= 13; ++v)
            {
                float y = qzPy((float)v);
                nvgBeginPath(vg);
                nvgMoveTo(vg, QZ_PX0, y + 0.5f);
                nvgLineTo(vg, QZ_PX1, y + 0.5f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, (v == 0 || v == 12) ? 34 : 9));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // Keyboard backdrop
            nvgBeginPath(vg);
            nvgRoundedRect(vg, QZ_KB_X0 - 3.f, QZ_KB_Y0 - 3.f,
                           QZ_KB_X1 - QZ_KB_X0 + 6.f, QZ_KB_WH + 6.f, 3.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
            nvgFill(vg);

            Widget::draw(args);
        }

        // ---- live content ----
        // Layer-1 overlay: draws the pitch-class wheel (scale polygon,
        // correction lines, note dots/labels, centre count), the transfer
        // staircase (ticks, identity line, staircase curve, trail and
        // channel markers), the keyboard (scale/root/active-key tinting,
        // hover outline, root bar, note letters), and the header readouts
        // plus the caption row.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            int root, scale, shift, nCh, mode, trHead, trCount;
            int note[QZ_MAX_CH];
            uint16_t mask;
            bool clocked;
            float clockFlash;
            float pcFlash[12], held[QZ_MAX_CH], trIn[QZ_TRAIL_N], trOut[QZ_TRAIL_N];

            if (module)
            {
                root = module->vRoot;
                scale = module->vScale;
                shift = module->vShift;
                nCh = module->vNch;
                mode = module->mode;
                mask = module->vMask;
                clocked = module->vClocked;
                clockFlash = module->clockFlash;
                trHead = module->trailHead;
                trCount = module->trailCount;
                for (int i = 0; i < 12; ++i)
                    pcFlash[i] = module->pcFlash[i];
                for (int c = 0; c < QZ_MAX_CH; ++c)
                {
                    note[c] = module->vNote[c];
                    held[c] = module->heldIn[c];
                }
                for (int i = 0; i < QZ_TRAIL_N; ++i)
                {
                    trIn[i] = module->trailIn[i];
                    trOut[i] = module->trailOut[i];
                }
            }
            else
            {
                // demo values for the module browser
                root = 0;
                scale = 1;
                shift = 0;
                nCh = 3;
                mode = 0;
                mask = QZ_SCALE_MASK[1];
                clocked = false;
                clockFlash = 0.f;
                for (int i = 0; i < 12; ++i)
                    pcFlash[i] = 0.f;
                pcFlash[4] = 0.6f;
                for (int c = 0; c < QZ_MAX_CH; ++c)
                {
                    held[c] = 0.f;
                    note[c] = 0;
                }
                held[0] = 0.37f;
                held[1] = 0.59f;
                held[2] = 0.20f;
                for (int c = 0; c < 3; ++c)
                    note[c] = qzQuant(held[c] * 12.f, mask, 0);
                trHead = 0;
                trCount = QZ_TRAIL_N;
                for (int i = 0; i < QZ_TRAIL_N; ++i)
                {
                    trIn[i] = 4.5f + 3.f * std::sin(0.2f * (float)i);
                    trOut[i] = (float)qzQuant(trIn[i], mask, 0);
                }
            }
            nCh = rack::math::clamp(nCh, 0, QZ_MAX_CH);
            scale = rack::math::clamp(scale, 0, QZ_NUM_SCALES - 1);
            mode = rack::math::clamp(mode, 0, (int)Quantara::NUM_MODES - 1);
            trCount = rack::math::clamp(trCount, 0, QZ_TRAIL_N);

            bool act[12] = {};
            for (int c = 0; c < nCh; ++c)
                act[qzMod(note[c], 12)] = true;

            nvgSave(vg);
            nvgScissor(vg, 0, 0, QZ_DISP_W, QZ_DISP_H);

            // ================= pitch-class wheel =================
            {
                const float cx = QZ_WC_X, cy = QZ_WC_Y, R = QZ_WC_R;
                // Returns the 2D position (x, y) on the wheel at a
                // fractional pitch class `pcf` and radius `rad`.
                auto pcPos = [&](float pcf, float rad, float &x, float &y)
                {
                    float a = pcf / 12.f * 2.f * (float)M_PI - (float)M_PI / 2.f;
                    x = cx + std::cos(a) * rad;
                    y = cy + std::sin(a) * rad;
                };

                // scale polygon
                float px[12], py[12];
                int pn = 0;
                for (int pc = 0; pc < 12; ++pc)
                {
                    if ((mask >> pc) & 1)
                    {
                        pcPos((float)pc, R, px[pn], py[pn]);
                        pn++;
                    }
                }
                if (pn >= 2)
                {
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, px[0], py[0]);
                    for (int i = 1; i < pn; ++i)
                        nvgLineTo(vg, px[i], py[i]);
                    if (pn >= 3)
                    {
                        nvgClosePath(vg);
                        nvgFillColor(vg, ag::withA(ag::accent(), 0.10f));
                        nvgFill(vg);
                    }
                    nvgLineJoin(vg, NVG_ROUND);
                    nvgStrokeColor(vg, ag::withA(ag::accent(), 0.35f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }

                // input -> quantized correction lines and input markers
                for (int c = 0; c < nCh; ++c)
                {
                    float n = held[c] * 12.f;
                    float pcf = n - 12.f * std::floor(n / 12.f);
                    float mx, my, tx, ty;
                    pcPos(pcf, R - 7.f, mx, my);
                    pcPos((float)qzMod(note[c], 12), R, tx, ty);

                    nvgBeginPath(vg);
                    nvgMoveTo(vg, mx, my);
                    nvgLineTo(vg, tx, ty);
                    nvgStrokeColor(vg, ag::withA(colWarm(), c == 0 ? 0.65f : 0.25f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);

                    nvgBeginPath(vg);
                    nvgCircle(vg, mx, my, c == 0 ? 2.4f : 1.7f);
                    nvgFillColor(vg, ag::withA(colWarm(), c == 0 ? 0.95f : 0.6f));
                    nvgFill(vg);
                }

                // note dots + labels (labels sit 9 px outside the ring)
                for (int pc = 0; pc < 12; ++pc)
                {
                    bool in = (mask >> pc) & 1;
                    bool isRoot = (pc == root);
                    float f = pcFlash[pc];
                    float x, y, lx, ly;
                    pcPos((float)pc, R, x, y);
                    pcPos((float)pc, R + 9.f, lx, ly);

                    NVGcolor col = isRoot ? colWarm() : (in ? ag::accent() : ag::dim());

                    if (act[pc])
                    {
                        nvgBeginPath(vg);
                        nvgCircle(vg, x, y, 6.5f);
                        nvgFillColor(vg, ag::withA(col, 0.22f + 0.30f * f));
                        nvgFill(vg);
                    }

                    nvgBeginPath(vg);
                    nvgCircle(vg, x, y, act[pc] ? 4.2f : (in ? 3.2f : 1.5f));
                    nvgFillColor(vg, in ? ag::withA(col, 0.95f) : ag::withA(col, 0.45f));
                    nvgFill(vg);
                    if (act[pc])
                    {
                        nvgBeginPath(vg);
                        nvgCircle(vg, x, y, 1.8f);
                        nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, 0.95f));
                        nvgFill(vg);
                    }
                    if (isRoot)
                    {
                        nvgBeginPath(vg);
                        nvgCircle(vg, x, y, 5.6f);
                        nvgStrokeColor(vg, ag::withA(colWarm(), 0.8f));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }

                    ag::text(vg, lx, ly, QZ_NOTE_NAMES[pc], 7.f,
                             isRoot ? colWarm() : (in ? colBright() : ag::withA(ag::dim(), 0.55f)),
                             ag::ALIGN_C, 0.f);
                }

                // centre: number of notes
                char b[8];
                int cnt = qzCount(mask);
                if (cnt > 0)
                    snprintf(b, sizeof(b), "%d", cnt);
                else
                    snprintf(b, sizeof(b), "--");
                ag::text(vg, cx, cy - 4.f, b, 13.f, colBright(), ag::ALIGN_C, 0.f);
                ag::text(vg, cx, cy + 8.f, cnt > 0 ? "NOTES" : "BYPASS", 7.f, ag::dim(),
                         ag::ALIGN_C, 0.f);
            }

            // ================= transfer staircase =================
            {
                nvgSave(vg);
                nvgScissor(vg, QZ_PX0, QZ_PY0, QZ_PX1 - QZ_PX0, QZ_PY1 - QZ_PY0);

                // allowed-note ticks on the right and bottom edges
                for (int v = -1; v <= 13; ++v)
                {
                    if (mask != 0 && !((mask >> qzMod(v, 12)) & 1))
                        continue;
                    float y = qzPy((float)v);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, QZ_PX1 - 4.f, y + 0.5f);
                    nvgLineTo(vg, QZ_PX1, y + 0.5f);
                    nvgStrokeColor(vg, ag::withA(ag::accent(), 0.55f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }
                for (int t = 0; t <= 12; ++t)
                {
                    if (mask != 0 && !((mask >> qzMod(t, 12)) & 1))
                        continue;
                    float x = qzPx((float)t);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, x + 0.5f, QZ_PY1 - 4.f);
                    nvgLineTo(vg, x + 0.5f, QZ_PY1);
                    nvgStrokeColor(vg, ag::withA(ag::accent(), 0.55f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }

                // identity line (dashed)
                for (int i = 0; i < 24; i += 2)
                {
                    float t0 = (float)i * 0.5f, t1 = (float)(i + 1) * 0.5f;
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, qzPx(t0), qzPy(t0));
                    nvgLineTo(vg, qzPx(t1), qzPy(t1));
                    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 34));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                }

                // staircase
                const int NP = 121;
                float sx[NP], sy[NP];
                for (int i = 0; i < NP; ++i)
                {
                    float t = 0.1f * (float)i;
                    float q = (mask == 0) ? t : (float)qzQuant(t, mask, mode);
                    sx[i] = qzPx(t);
                    sy[i] = qzPy(q);
                }

                nvgBeginPath(vg);
                nvgMoveTo(vg, sx[0], QZ_PY1);
                for (int i = 0; i < NP; ++i)
                    nvgLineTo(vg, sx[i], sy[i]);
                nvgLineTo(vg, sx[NP - 1], QZ_PY1);
                nvgClosePath(vg);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.10f));
                nvgFill(vg);

                nvgLineCap(vg, NVG_ROUND);
                nvgLineJoin(vg, NVG_ROUND);
                nvgBeginPath(vg);
                nvgMoveTo(vg, sx[0], sy[0]);
                for (int i = 1; i < NP; ++i)
                    nvgLineTo(vg, sx[i], sy[i]);
                nvgStrokeColor(vg, ag::withA(colBright(), 0.95f));
                nvgStrokeWidth(vg, 1.3f);
                nvgStroke(vg);

                // trail of channel 1
                if (nCh > 0)
                {
                    for (int i = 0; i < trCount; ++i)
                    {
                        int idx = ((trHead - trCount + i) % QZ_TRAIL_N + QZ_TRAIL_N) % QZ_TRAIL_N;
                        float base = std::floor(trIn[idx] / 12.f);
                        float xi = trIn[idx] - 12.f * base;
                        float yo = rack::math::clamp(trOut[idx] - 12.f * base, -1.f, 13.f);
                        float age = (float)(i + 1) / (float)trCount;
                        nvgBeginPath(vg);
                        nvgCircle(vg, qzPx(xi), qzPy(yo), 1.1f);
                        nvgFillColor(vg, ag::withA(colWarm(), 0.55f * age * age));
                        nvgFill(vg);
                    }
                }

                // channel markers
                for (int c = nCh - 1; c >= 0; --c)
                {
                    float n = held[c] * 12.f;
                    float base = std::floor(n / 12.f);
                    float xi = n - 12.f * base;
                    float yo = rack::math::clamp((float)note[c] - 12.f * base, -1.f, 13.f);
                    float x = qzPx(xi), y = qzPy(yo);

                    if (c == 0)
                    {
                        // guides
                        nvgBeginPath(vg);
                        nvgMoveTo(vg, x, QZ_PY1);
                        nvgLineTo(vg, x, y);
                        nvgStrokeColor(vg, ag::withA(colWarm(), 0.45f));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                        nvgBeginPath(vg);
                        nvgMoveTo(vg, x, y);
                        nvgLineTo(vg, QZ_PX1, y);
                        nvgStrokeColor(vg, ag::withA(colBright(), 0.45f));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);

                        nvgBeginPath(vg);
                        nvgCircle(vg, x, y, 3.2f);
                        nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, 0.97f));
                        nvgFill(vg);
                        nvgStrokeColor(vg, ag::withA(colBright(), 0.9f));
                        nvgStrokeWidth(vg, 1.f);
                        nvgStroke(vg);
                    }
                    else
                    {
                        nvgBeginPath(vg);
                        nvgCircle(vg, x, y, 1.9f);
                        nvgFillColor(vg, ag::withA(colBright(), 0.8f));
                        nvgFill(vg);
                    }
                }

                nvgRestore(vg);
            }

            // ================= keyboard =================
            {
                const float ww = (QZ_KB_X1 - QZ_KB_X0) / 7.f;

                // Draws one mini-keyboard key with scale/root/active tint,
                // hover outline, root bar and (for white keys) the note name.
                auto drawKey = [&](float x, float y, float w, float h, int pc, bool black)
                {
                    bool in = (mask >> pc) & 1;
                    bool isRoot = (pc == root);
                    bool a = act[pc];
                    float f = pcFlash[pc];
                    bool hv = (hoverKey == pc);

                    nvgBeginPath(vg);
                    nvgRoundedRectVarying(vg, x, y, w, h, 1.f, 1.f, 3.f, 3.f);
                    nvgFillColor(vg, black ? nvgRGB(0x08, 0x0a, 0x0e) : nvgRGB(0x16, 0x1c, 0x24));
                    nvgFill(vg);

                    if (in)
                    {
                        NVGcolor tint = isRoot ? colWarm() : ag::accent();
                        nvgBeginPath(vg);
                        nvgRoundedRectVarying(vg, x, y, w, h, 1.f, 1.f, 3.f, 3.f);
                        nvgFillColor(vg, ag::withA(tint, 0.28f));
                        nvgFill(vg);
                    }
                    if (a)
                    {
                        nvgBeginPath(vg);
                        nvgRoundedRectVarying(vg, x, y, w, h, 1.f, 1.f, 3.f, 3.f);
                        nvgFillColor(vg, nvgRGBAf(0.80f, 0.97f, 1.f, 0.30f + 0.40f * f));
                        nvgFill(vg);
                    }

                    nvgBeginPath(vg);
                    nvgRoundedRectVarying(vg, x, y, w, h, 1.f, 1.f, 3.f, 3.f);
                    nvgStrokeColor(vg, hv ? ag::withA(colBright(), 0.95f)
                                          : nvgRGBA(255, 255, 255, black ? 34 : 44));
                    nvgStrokeWidth(vg, hv ? 1.4f : 1.f);
                    nvgStroke(vg);

                    if (isRoot)
                    {
                        nvgBeginPath(vg);
                        nvgRect(vg, x + 2.5f, y + h - 3.5f, w - 5.f, 2.f);
                        nvgFillColor(vg, ag::withA(colWarm(), 0.95f));
                        nvgFill(vg);
                    }
                    if (!black)
                    {
                        // single-letter name, centred in the key above the root bar
                        ag::text(vg, x + w / 2.f, y + h - 9.f, QZ_NOTE_NAMES[pc], 7.f,
                                 in ? colBright() : ag::withA(ag::dim(), 0.6f),
                                 ag::ALIGN_C, 0.f);
                    }
                };

                for (int i = 0; i < 7; ++i)
                    drawKey(QZ_KB_X0 + (float)i * ww + 0.5f, QZ_KB_Y0, ww - 1.f, QZ_KB_WH, QZ_WPC[i], false);
                for (int j = 0; j < 5; ++j)
                {
                    float bx = QZ_KB_X0 + (float)(QZ_BIDX[j] + 1) * ww - QZ_KB_BW / 2.f;
                    drawKey(bx, QZ_KB_Y0, QZ_KB_BW, QZ_KB_BH, QZ_BPC[j], true);
                }
            }

            // ================= header readouts + caption row =================
            {
                char buf[48];
                NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 220);
                NVGcolor cm = nvgRGBA(0xff, 0x9c, 0x6a, 230);
                NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 230);

                // clock lamp
                nvgBeginPath(vg);
                nvgCircle(vg, 8.f, 7.f, 2.8f);
                nvgFillColor(vg, ag::withA(ag::accent(), (clocked ? 0.25f : 0.12f) + 0.75f * clockFlash));
                nvgFill(vg);

                snprintf(buf, sizeof(buf), "%s %s", QZ_NOTE_NAMES[qzMod(root, 12)], QZ_SCALE_SHORT[scale]);
                ag::text(vg, 15.f, 7.f, buf, 7.f, c1, ag::ALIGN_L, 0.5f);

                snprintf(buf, sizeof(buf), "SHIFT %+d", shift);
                ag::text(vg, QZ_DISP_W / 2.f + 36.f, 7.f, buf, 7.f, cm, ag::ALIGN_C, 0.5f);

                if (nCh > 0)
                    snprintf(buf, sizeof(buf), "CH %d %s", nCh, clocked ? "HOLD" : "LIVE");
                else
                    snprintf(buf, sizeof(buf), "NO INPUT");
                ag::text(vg, QZ_DISP_W - 6.f, 7.f, buf, 7.f, cd, ag::ALIGN_R, 0.5f);

                // caption row under the keyboard: hint / hover text, quantize mode on the right
                if (hoverKey >= 0)
                {
                    bool on = (mask >> hoverKey) & 1;
                    snprintf(buf, sizeof(buf), "%s %s - CLICK TO TOGGLE",
                             QZ_NOTE_NAMES[hoverKey], on ? "ON" : "OFF");
                    ag::text(vg, 8.f, 134.f, buf, 7.f, c1, ag::ALIGN_L, 0.5f);
                }
                else
                {
                    ag::text(vg, 8.f, 134.f, "CLICK KEYS TO EDIT SCALE", 7.f,
                             ag::withA(ag::dim(), 0.7f), ag::ALIGN_L, 0.5f);
                }
                static const char *mn[3] = {"NEAREST", "DOWN", "UP"};
                ag::text(vg, QZ_DISP_W - 6.f, 134.f, mn[mode], 7.f, ag::violet(), ag::ALIGN_R, 0.5f);
            }

            nvgRestore(vg);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget: adds the static panel, knob value arcs, screws, the live
// display, the four side knobs, the five CV input jacks and the three I/O
// jacks, plus the right-click menu.
// ------------------------------------------------------------
struct QuantaraWidget : ModuleWidget
{
    // Builds the widget tree: static background, knob value arcs, screws,
    // display, the four side knobs (ROOT / SCALE / SHIFT / GLIDE), the five
    // CV inputs (CLOCK / ROOT / SCALE / SHIFT / GLIDE) and the three I/O
    // jacks (IN / OUT / TRIG).
    QuantaraWidget(Quantara *module)
    {
        setModule(module);
        box.size = Vec(QZ_PANEL_W, QZ_PANEL_H);

        addChild(new QZPanelBackground());
        addChild(new QZKnobArcs(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new QZDisplayWidget(module));

        // Side knobs: ROOT, SCALE, SHIFT, GLIDE
        for (int k = 0; k < 4; ++k)
        {
            addParam(createParamCentered<RoundSmallBlackKnob>(
                Vec(QZ_KNOB_X[k], QZ_KNOB_Y[k]), module, k));
        }

        // CV row
        const float cvY = qzJackY(QZ_CV_TOP);
        addInput(createInputCentered<DarkPJ301MPort>(Vec(QZ_CV_X[0], cvY), module, Quantara::CLOCK_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(QZ_CV_X[1], cvY), module, Quantara::ROOT_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(QZ_CV_X[2], cvY), module, Quantara::SCALE_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(QZ_CV_X[3], cvY), module, Quantara::SHIFT_CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(QZ_CV_X[4], cvY), module, Quantara::GLIDE_CV_INPUT));

        // I/O row
        const float ioY = qzJackY(QZ_IO_TOP);
        addInput(createInputCentered<DarkPJ301MPort>(Vec(QZ_IO_X[0], ioY), module, Quantara::IN_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(QZ_IO_X[1], ioY), module, Quantara::OUT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(QZ_IO_X[2], ioY), module, Quantara::TRIG_OUTPUT));
    }

    // Adds the right-click menu: a "Scale" submenu with all scale names, a
    // "Quantize mode" submenu (nearest / down / up), and a "Custom scale"
    // submenu with "Copy current scale", "Clear all notes" and "All notes on"
    // actions.
    void appendContextMenu(ui::Menu *menu) override
    {
        Quantara *module = dynamic_cast<Quantara *>(this->module);
        if (!module)
            return;

        menu->addChild(new ui::MenuSeparator);

        menu->addChild(createSubmenuItem("Scale", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             for (int i = 0; i < QZ_NUM_SCALES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(QZ_SCALE_FULL[i],
                                                                                  CHECKMARK(module->vScale == i),
                                                                                  [=]()
                                                                                  { module->params[Quantara::SCALE_PARAM].setValue((float)i); }));
                                             }
                                         }));

        menu->addChild(createSubmenuItem("Quantize mode", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             const char *names[] = {"Nearest", "Down (floor)", "Up (ceil)"};
                                             for (int i = 0; i < Quantara::NUM_MODES; ++i)
                                             {
                                                 submenu->addChild(createMenuItem(names[i],
                                                                                  CHECKMARK(module->mode == i),
                                                                                  [=]()
                                                                                  { module->mode = i; }));
                                             }
                                         }));

        menu->addChild(createSubmenuItem("Custom scale", "",
                                         [=](ui::Menu *submenu)
                                         {
                                             submenu->addChild(createMenuItem("Copy current scale", "", [=]()
                                                                              {
                    module->userMask = module->relMaskFor(module->vScale);
                    module->params[Quantara::SCALE_PARAM].setValue((float)(QZ_NUM_SCALES - 1)); }));
                                             submenu->addChild(createMenuItem("Clear all notes (bypass)", "", [=]()
                                                                              {
                    module->userMask = 0;
                    module->params[Quantara::SCALE_PARAM].setValue((float)(QZ_NUM_SCALES - 1)); }));
                                             submenu->addChild(createMenuItem("All notes on", "", [=]()
                                                                              {
                    module->userMask = 0xFFF;
                    module->params[Quantara::SCALE_PARAM].setValue((float)(QZ_NUM_SCALES - 1)); }));
                                         }));
    }
};

Model *modelQuantara = createModel<Quantara, QuantaraWidget>("Quantara");