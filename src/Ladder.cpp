#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>

// ============================================================================
// Ladder — Shepard-tone VCO
// ============================================================================
//
// OVERVIEW
// --------
// Ladder is a 24 HP VCO that renders the classic **Shepard–Risset glissando**:
// a stack of octave-spaced partials whose amplitudes are weighted by a
// Gaussian window in log-frequency space. As the SPEED knob advances, all
// partials slide together; partials leave the window while new ones fade in,
// so the ear hears a pitch that climbs (or descends) forever.
//
// Sigma scales with the partial count so that every selected oscillator stays
// audible — the outermost partial always has ~13.5 % of the peak amplitude.
// At the default 6 partials sigma ≈ 1.25, which is the classic Shepard feel.
//
// Click prevention: FREQ/FINE and the partial span are slewed, per-partial
// gains are smoothed, and phases are never reset, so parameter jumps do not
// produce discontinuities in the output.
//
// Controls:
//   FREQ      perceived pitch (default C4), shown as note name or Hz
//   FINE      fine tune (±100 cents)
//   WAVE      waveform of every partial: Sine / Saw / Triangle / Step
//   PULSE     duty cycle of the Step wave (5..95 %)
//   SPEED     glissando rate (0 = static chord, 1 = ~4 semitones/sec)
//   OSCS      number of octave-spaced partials (1..20, default 6)
//   DIR       sweep direction: Ascending (default) or Descending
//
// Right-click:
//   - Text field: enter a frequency ("440") or a note name ("A#3", "Bb2").
//   - Display-mode toggle: show pitch as Note name or as Hz.
//
// Outputs: OUT L / OUT R — stereo, with partials panned by their position in
// the Gaussian window (low partials left, high partials right).
// ============================================================================

// ------------------------------------------------------------
// Layout constants (24 HP = 360 px wide)
// ------------------------------------------------------------
static const float LD_PANEL_W = RACK_GRID_WIDTH * 24; // 360
static const float LD_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (320 x 112): header row, two framed plots, footer row. Bezel adds 4 px around it.
static const float LD_DISP_W = 320.f;
static const float LD_DISP_H = 112.f;
static const float LD_DISP_X = (LD_PANEL_W - LD_DISP_W) / 2.f; // 20
static const float LD_DISP_Y = 26.f;
static const float LD_HEAD_LINE = 15.5f;                       // hairline under the header row
static const float LD_PLOT_Y = 20.f, LD_PLOT_H = 72.f;         // both plots share this band
static const float LD_FOOT_LINE = 96.5f;                       // hairline above the footer row
static const float LD_FOOT_TXT = LD_DISP_H - 8.f;              // footer text centre line
static const float LD_SCOPE_X = 8.f,   LD_SCOPE_W = 154.f;     // waveform scope
static const float LD_SPEC_X  = 168.f, LD_SPEC_W  = 144.f;     // partial spectrum

// Control cells: 76 px wide, label centred 10 px below the top edge, control centre 32 px below it
static const float LD_CELL_W = 76.f;
static const float LD_CELL_H = 52.f;
static const float LD_CTL_DY = 32.f;

// Row 1 (FREQ, FINE, WAVE, PULSE)
static const float LD_VCO_HDR  = 150.f;
static const float LD_ROW1_TOP = 158.f;
static const float LD_ROW1_Y   = LD_ROW1_TOP + LD_CTL_DY;
static const float LD_ROW1_X[4] = {54.f, 138.f, 222.f, 306.f};

// Row 2 (SPEED, OSCS, DIR)
static const float LD_SHEP_HDR = 220.f;
static const float LD_ROW2_TOP = 228.f;
static const float LD_ROW2_Y   = LD_ROW2_TOP + LD_CTL_DY;
static const float LD_ROW2_X[3] = {96.f, 180.f, 264.f};

// I/O row (V/OCT, OUT L, OUT R): jack centred 30 px below the cell top
static const float LD_IO_HDR = 290.f;
static const float LD_IO_TOP = 298.f;
static const float LD_IO_H   = 50.f;
static const float LD_IO_Y   = LD_IO_TOP + 30.f;
static const float LD_IO_X[3] = {96.f, 180.f, 264.f};

// ------------------------------------------------------------
// Shared helpers
// ------------------------------------------------------------
namespace
{
    // Converts semitones above C4 to Hz (C4 = 261.625565 Hz).
    inline float ldSemisToHz(float semis)
    {
        return 261.625565f * std::pow(2.f, semis / 12.f);
    }

    // Formats a frequency as a note name with octave (e.g. "A#3").
    inline void ldFreqToNote(float hz, char *buf, size_t n)
    {
        if (hz <= 0.f)
        {
            snprintf(buf, n, "--");
            return;
        }
        static const char *names[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
        float midiNote = 69.f + 12.f * std::log2(hz / 440.f);
        int midiInt = (int)std::round(midiNote);
        int octave = midiInt / 12 - 1;
        int pc = midiInt % 12;
        if (pc < 0)
            pc += 12;
        snprintf(buf, n, "%s%d", names[pc], octave);
    }

    // Parses a raw frequency ("440") or a note name ("C4", "A#3", "Bb2").
    inline bool ldParseFreqString(const char *s, float &semisFromC4)
    {
        if (!s || !*s)
            return false;

        char *end = nullptr;
        float hz = strtof(s, &end);
        if (end != s && end && *end == '\0' && hz > 0.f)
        {
            semisFromC4 = 12.f * std::log2(hz / 261.625565f);
            return true;
        }

        const char *p = s;
        while (*p == ' ' || *p == '\t')
            p++;
        char c = *p;
        int pc = -1;
        switch (c)
        {
        case 'C': case 'c': pc = 0; break;
        case 'D': case 'd': pc = 2; break;
        case 'E': case 'e': pc = 4; break;
        case 'F': case 'f': pc = 5; break;
        case 'G': case 'g': pc = 7; break;
        case 'A': case 'a': pc = 9; break;
        case 'B': case 'b': pc = 11; break;
        default: return false;
        }
        p++;

        if (*p == '#')
        {
            pc += 1;
            p++;
        }
        else if (*p == 'b')
        {
            const char *next = p + 1;
            while (*next == ' ' || *next == '\t')
                next++;
            if (*next >= '0' && *next <= '9')
            {
                pc -= 1;
                p++;
            }
        }

        while (*p == ' ' || *p == '\t')
            p++;
        int sign = 1;
        if (*p == '-') { sign = -1; p++; }
        else if (*p == '+') { p++; }
        if (*p < '0' || *p > '9')
            return false;
        int oct = 0;
        while (*p >= '0' && *p <= '9')
        {
            oct = oct * 10 + (*p - '0');
            p++;
        }
        oct *= sign;

        while (*p == ' ' || *p == '\t')
            p++;
        if (*p != '\0')
            return false;

        int midi = (oct + 1) * 12 + pc;
        semisFromC4 = (float)(midi - 60);
        return true;
    }

    // Renders one partial's waveform at normalised phase `ph` (0..1).
    //
    // The pure triangle's 3rd harmonic is only -19 dB, which gets buried in
    // a dense Shepard stack. We double the natural 3rd harmonic (adding a
    // -0.15·cos(6π·ph) term) to lift it to about -13 dB, so the triangle
    // stays audibly distinct from a sine.
    inline float ldWaveSample(float ph, int wave, float pw)
    {
        switch (wave)
        {
        case 0: // Sine
            return std::sin(2.f * (float)M_PI * ph);
        case 1: // Saw
            return 2.f * ph - 1.f;
        case 2: // Triangle (with boosted 3rd harmonic)
        {
            float tri = 1.f - 4.f * std::fabs(ph - 0.5f);
            float h3 = -std::cos(6.f * (float)M_PI * ph) * 0.15f;
            return (tri + h3) / 1.15f;
        }
        default: // Step / pulse (DC-free)
            return (ph < pw ? 1.f : -1.f) - (2.f * pw - 1.f);
        }
    }
} // namespace

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// Shepard-tone VCO: up to 20 octave-spaced partials, Gaussian-weighted with
// sigma scaled by partial count, swept by SPEED (ascending or descending),
// and panned per-partial.
struct Ladder : Module
{
    enum ParamId
    {
        FREQ_PARAM,
        FINE_PARAM,
        WAVE_PARAM,
        PULSE_WIDTH_PARAM,
        LADDER_SPEED_PARAM,
        NUM_OSCS_PARAM,
        DIRECTION_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        VOCT_INPUT,
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
        LIGHTS_LEN
    };

    enum FreqDisplay
    {
        DISP_NOTE,
        DISP_HZ
    };

    static const int LD_MAX_PARTIALS = 20;

    // Custom ParamQuantity: FREQ shows note or Hz.
    struct FreqQuantity : ParamQuantity
    {
        Ladder *module = nullptr;
        std::string getDisplayValueString() override
        {
            float semis = getValue();
            float hz = ldSemisToHz(semis);
            char buf[32];
            if (module && module->freqDisplay == DISP_HZ)
                snprintf(buf, sizeof(buf), "%.2f Hz", hz);
            else
                ldFreqToNote(hz, buf, sizeof(buf));
            return buf;
        }
    };

    // Custom ParamQuantity: WAVE shows the waveform name.
    struct WaveQuantity : ParamQuantity
    {
        std::string getDisplayValueString() override
        {
            static const char *names[] = {"Sine", "Saw", "Triangle", "Step"};
            int i = rack::math::clamp((int)std::round(getValue()), 0, 3);
            return names[i];
        }
    };

    // Custom ParamQuantity: OSCS snaps to integer.
    struct NumOscsQuantity : ParamQuantity
    {
        std::string getDisplayValueString() override
        {
            int n = rack::math::clamp((int)std::round(getValue()), 1, LD_MAX_PARTIALS);
            return string::f("%d", n);
        }
    };

    FreqDisplay freqDisplay = DISP_NOTE;

    float phases[LD_MAX_PARTIALS] = {};
    float sweep = 0.f;

    // Click-prevention state
    float smoothSemis = 0.f;               // slewed FREQ + FINE
    float smoothHalfSpan = 2.5f;           // slewed (numOscs - 1) / 2
    bool smoothInit = false;
    float ampSmooth[LD_MAX_PARTIALS] = {}; // per-partial gain smoothing

    // Display snapshot
    float dispPitchSemis = 0.f;
    float dispSweep = 0.f;
    float dispAmp[LD_MAX_PARTIALS] = {};
    float dispPan[LD_MAX_PARTIALS] = {};
    float dispFreqs[LD_MAX_PARTIALS] = {};
    float dispPhases[LD_MAX_PARTIALS] = {};
    int dispWave = 0;
    float dispPulseWidth = 0.5f;
    int dispNumOscs = 6;
    bool dispDescending = false;

    Ladder()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        auto *fq = configParam<FreqQuantity>(FREQ_PARAM, -48.f, 48.f, 0.f, "Frequency", " st");
        fq->module = this;

        configParam(FINE_PARAM, -100.f, 100.f, 0.f, "Fine", " ct");

        configParam<WaveQuantity>(WAVE_PARAM, 0.f, 3.f, 0.f, "Waveform");
        paramQuantities[WAVE_PARAM]->snapEnabled = true;

        configParam(PULSE_WIDTH_PARAM, 0.05f, 0.95f, 0.5f, "Pulse width", "%", 0.f, 100.f);
        configParam(LADDER_SPEED_PARAM, 0.f, 1.f, 0.3f, "Ladder speed", "%", 0.f, 100.f);

        configParam<NumOscsQuantity>(NUM_OSCS_PARAM, 1.f, (float)LD_MAX_PARTIALS, 6.f, "Oscillators", "", 0.f, 1.f);
        paramQuantities[NUM_OSCS_PARAM]->snapEnabled = true;

        configSwitch(DIRECTION_PARAM, 0.f, 1.f, 0.f, "Direction", {"Ascending", "Descending"});

        configInput(VOCT_INPUT, "V/oct");
        configOutput(OUT_L_OUTPUT, "L out");
        configOutput(OUT_R_OUTPUT, "R out");
    }

    void onReset() override
    {
        smoothInit = false;
        for (int i = 0; i < LD_MAX_PARTIALS; ++i)
            ampSmooth[i] = 0.f;
    }

    void process(const ProcessArgs &args) override
    {
        const float sr = args.sampleRate;

        // ---- controls ----
        float knobSemis = params[FREQ_PARAM].getValue()
                        + params[FINE_PARAM].getValue() / 100.f;
        float speed = params[LADDER_SPEED_PARAM].getValue();
        int wave = rack::math::clamp((int)std::round(params[WAVE_PARAM].getValue()), 0, 3);
        float pw = params[PULSE_WIDTH_PARAM].getValue();
        int numOscs = rack::math::clamp((int)std::round(params[NUM_OSCS_PARAM].getValue()), 1, LD_MAX_PARTIALS);
        bool descending = params[DIRECTION_PARAM].getValue() > 0.5f;

        const float targetHalfSpan = (float)(numOscs - 1) * 0.5f;
        if (!smoothInit)
        {
            smoothSemis = knobSemis;
            smoothHalfSpan = targetHalfSpan;
            smoothInit = true;
        }

        // ~8 ms one-pole slew on FREQ+FINE: removes the click on frequency
        // jumps while still feeling instant. V/OCT stays unslewed so CV
        // tracking is not affected.
        const float pitchCoef = 1.f - std::exp(-args.sampleTime / 0.008f);
        smoothSemis += (knobSemis - smoothSemis) * pitchCoef;

        // Slower slew on the partial span. Changing OSCS shifts every
        // partial by half an octave per step; slewing avoids a hard jump.
        const float spanCoef = 1.f - std::exp(-args.sampleTime / 0.020f);
        smoothHalfSpan += (targetHalfSpan - smoothHalfSpan) * spanCoef;

        float baseSemis = smoothSemis + inputs[VOCT_INPUT].getVoltage() * 12.f;

        // ---- Shepard sweep ----
        if (speed > 0.001f)
        {
            float sweepRate = speed * 4.f;
            if (descending)
            {
                sweep -= sweepRate * args.sampleTime;
                while (sweep < 0.f)
                    sweep += 12.f;
            }
            else
            {
                sweep += sweepRate * args.sampleTime;
                while (sweep >= 12.f)
                    sweep -= 12.f;
            }
        }

        // ---- Shepard tone ----
        //
        // sigma scales with N: sigma = max(1, (N-1)/4). This keeps the
        // outermost partial at exp(-2) ≈ 0.135 of the peak, so all chosen
        // oscillators are audible. For N=6, sigma ≈ 1.25 ≈ classic Shepard.
        // Computed from the slewed span so it changes smoothly with OSCS.
        float sigma = std::max(1.0f, smoothHalfSpan * 2.f * 0.25f);
        float stereoWidth = 0.55f;
        float centerLog2 = baseSemis / 12.f;
        float halfSpan = smoothHalfSpan;

        // ~5 ms per-partial gain smoothing (fades partials in/out on OSCS changes)
        const float ampCoef = 1.f - std::exp(-args.sampleTime / 0.005f);

        float outL = 0.f, outR = 0.f;
        float sumAmpSq = 0.f;

        // Runs over every slot so inactive partials fade out smoothly and
        // phases are never reset.
        for (int i = 0; i < LD_MAX_PARTIALS; ++i)
        {
            bool active = i < numOscs;
            float offsetOct = (float)i - halfSpan;
            float semis = baseSemis + sweep + offsetOct * 12.f;
            float freqIdeal = ldSemisToHz(semis);

            // Gaussian computed on the ideal (unclamped) frequency so that
            // sub-audio / above-Nyquist partials keep their correct weight
            // and show up in the display.
            float logF = std::log2(std::max(freqIdeal, 1.f) / 261.625565f);
            float dist = logF - centerLog2;
            float target = active ? std::exp(-dist * dist / (2.f * sigma * sigma)) : 0.f;

            float fc = rack::math::clamp(freqIdeal, 10.f, sr * 0.45f);
            phases[i] += fc * args.sampleTime;
            phases[i] -= std::floor(phases[i]);

            ampSmooth[i] += (target - ampSmooth[i]) * ampCoef;
            float amp = ampSmooth[i];

            dispAmp[i] = target;
            dispFreqs[i] = fc;
            dispPhases[i] = phases[i];

            if (amp < 1e-4f)
            {
                dispPan[i] = 0.f;
                continue;
            }
            sumAmpSq += amp * amp;

            float s = ldWaveSample(phases[i], wave, pw);

            float pan = rack::math::clamp(dist / sigma * stereoWidth, -1.f, 1.f);
            float ang = (pan + 1.f) * 0.25f * (float)M_PI;
            float gL = std::cos(ang);
            float gR = std::sin(ang);
            dispPan[i] = pan;

            outL += s * amp * gL;
            outR += s * amp * gR;
        }

        // RMS normalisation keeps level consistent from N=1 to N=20.
        if (sumAmpSq > 0.001f)
        {
            float inv = 1.8f / std::sqrt(sumAmpSq);
            outL *= inv;
            outR *= inv;
        }

        outL = 5.f * std::tanh(outL * 0.8f);
        outR = 5.f * std::tanh(outR * 0.8f);

        outputs[OUT_L_OUTPUT].setVoltage(outL);
        outputs[OUT_R_OUTPUT].setVoltage(outR);

        // ---- display snapshot ----
        dispPitchSemis = baseSemis;
        dispSweep = sweep;
        dispWave = wave;
        dispPulseWidth = pw;
        dispNumOscs = numOscs;
        dispDescending = descending;
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "freqDisplay", json_integer((int)freqDisplay));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *j = json_object_get(root, "freqDisplay");
        if (j && json_is_integer(j))
            freqDisplay = (json_integer_value(j) == DISP_HZ) ? DISP_HZ : DISP_NOTE;
    }
};

// ------------------------------------------------------------
// Panel background (static artwork)
// ------------------------------------------------------------
namespace
{
    namespace ag = agmm;

    struct LDPanel : TransparentWidget
    {
        LDPanel() { box.size = Vec(LD_PANEL_W, LD_PANEL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::panelBase(vg, LD_PANEL_W, LD_PANEL_H, "LADDER");
            ag::bezel(vg, LD_DISP_X, LD_DISP_Y, LD_DISP_W, LD_DISP_H);

            ag::sectionHeader(vg, LD_PANEL_W, LD_VCO_HDR, "VCO", 16.f);
            const char *vcoLabels[4] = {"FREQ", "FINE", "WAVE", "PULSE"};
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, LD_ROW1_X[i], LD_ROW1_TOP, LD_CELL_W, LD_CELL_H, vcoLabels[i]);

            ag::sectionHeader(vg, LD_PANEL_W, LD_SHEP_HDR, "SHEPARD", 16.f);
            const char *shepLabels[3] = {"SPEED", "OSCS", "DIR"};
            for (int i = 0; i < 3; ++i)
                ag::cell(vg, LD_ROW2_X[i], LD_ROW2_TOP, LD_CELL_W, LD_CELL_H, shepLabels[i]);

            ag::sectionHeader(vg, LD_PANEL_W, LD_IO_HDR, "I / O", 16.f);
            ag::cell(vg, LD_IO_X[0], LD_IO_TOP, LD_CELL_W, LD_IO_H, "V/OCT");
            ag::cell(vg, LD_IO_X[1], LD_IO_TOP, LD_CELL_W, LD_IO_H, "OUT L", true);
            ag::cell(vg, LD_IO_X[2], LD_IO_TOP, LD_CELL_W, LD_IO_H, "OUT R", true);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display
    //   header : pitch | waveform | partial count + direction
    //   left   : scope of the summed waveform (4 ms window)
    //   right  : partials on a log-frequency axis. Each stem slides through the fixed
    //            Gaussian window as the sweep advances; ticks on the baseline are the
    //            rungs of the ladder at sweep = 0. Stem colour = stereo position.
    //   footer : scope window | pan legend, sweep position, axis direction
    // ------------------------------------------------------------
    struct LDDisplay : TransparentWidget
    {
        Ladder *module = nullptr;

        LDDisplay(Ladder *m) : module(m)
        {
            box.pos = Vec(LD_DISP_X, LD_DISP_Y);
            box.size = Vec(LD_DISP_W, LD_DISP_H);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;

            ag::screen(vg, 0.f, 0.f, LD_DISP_W, LD_DISP_H);

            // separators under the header and above the footer
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, LD_HEAD_LINE);
            nvgLineTo(vg, LD_DISP_W - 8.f, LD_HEAD_LINE);
            nvgMoveTo(vg, 8.f, LD_FOOT_LINE);
            nvgLineTo(vg, LD_DISP_W - 8.f, LD_FOOT_LINE);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, LD_SCOPE_X, LD_PLOT_Y, LD_SCOPE_W, LD_PLOT_H);
            ag::plotFrame(vg, LD_SPEC_X, LD_PLOT_Y, LD_SPEC_W, LD_PLOT_H);

            // scope graticule: 4 time divisions, zero line
            nvgBeginPath(vg);
            for (int k = 1; k < 4; ++k)
            {
                float x = LD_SCOPE_X + LD_SCOPE_W * (float)k / 4.f;
                nvgMoveTo(vg, x, LD_PLOT_Y + 1.f);
                nvgLineTo(vg, x, LD_PLOT_Y + LD_PLOT_H - 1.f);
            }
            nvgMoveTo(vg, LD_SCOPE_X + 1.f, LD_PLOT_Y + LD_PLOT_H / 2.f);
            nvgLineTo(vg, LD_SCOPE_X + LD_SCOPE_W - 1.f, LD_PLOT_Y + LD_PLOT_H / 2.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
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

            float pitchSemis = 0.f;
            float sweep = 0.f;
            float amp[Ladder::LD_MAX_PARTIALS] = {};
            float pan[Ladder::LD_MAX_PARTIALS] = {};
            float freqs[Ladder::LD_MAX_PARTIALS] = {};
            float phases[Ladder::LD_MAX_PARTIALS] = {};
            int wave = 0;
            float pw = 0.5f;
            int numOscs = 6;
            bool descending = false;
            Ladder::FreqDisplay mode = Ladder::DISP_NOTE;

            if (module)
            {
                pitchSemis = module->dispPitchSemis;
                sweep = module->dispSweep;
                wave = module->dispWave;
                pw = module->dispPulseWidth;
                numOscs = module->dispNumOscs;
                descending = module->dispDescending;
                mode = module->freqDisplay;
                for (int i = 0; i < Ladder::LD_MAX_PARTIALS; ++i)
                {
                    amp[i] = module->dispAmp[i];
                    pan[i] = module->dispPan[i];
                    freqs[i] = module->dispFreqs[i];
                    phases[i] = module->dispPhases[i];
                }
            }
            else
            {
                // module browser preview
                pitchSemis = 0.f;
                sweep = 3.f;
                wave = 0;
                pw = 0.5f;
                numOscs = 6;
                descending = false;
                float sigma = std::max(1.0f, (float)(numOscs - 1) * 0.25f);
                float halfSpan = (numOscs - 1) * 0.5f;
                for (int i = 0; i < Ladder::LD_MAX_PARTIALS; ++i)
                {
                    if (i >= numOscs)
                    {
                        amp[i] = 0.f;
                        continue;
                    }
                    float offsetOct = (float)i - halfSpan;
                    float semis = pitchSemis + sweep + offsetOct * 12.f;
                    freqs[i] = ldSemisToHz(semis);
                    phases[i] = (float)i * 0.13f;
                    float dist = sweep / 12.f + offsetOct;
                    amp[i] = std::exp(-dist * dist / (2.f * sigma * sigma));
                    pan[i] = std::max(-1.f, std::min(1.f, dist / sigma * 0.55f));
                }
            }
            numOscs = std::max(1, std::min(numOscs, (int)Ladder::LD_MAX_PARTIALS));

            const float W = LD_DISP_W;
            char buf[64];

            // ---- header: pitch | waveform | partials + direction ----
            float freq = ldSemisToHz(pitchSemis);
            if (mode == Ladder::DISP_HZ)
                snprintf(buf, sizeof(buf), "%.2f Hz", freq);
            else
                ldFreqToNote(freq, buf, sizeof(buf));
            ag::text(vg, 8.f, 8.f, buf, 12.f, ag::bright(), ag::ALIGN_L, 0.5f);

            static const char *waveNames[4] = {"SINE", "SAW", "TRIANGLE", "STEP"};
            if (wave == 3)
                snprintf(buf, sizeof(buf), "STEP %.0f%%", pw * 100.f);
            else
                snprintf(buf, sizeof(buf), "%s", waveNames[std::max(0, std::min(wave, 3))]);
            ag::text(vg, W / 2.f, 8.f, buf, 7.f, ag::textCol(), ag::ALIGN_C, 1.f);

            snprintf(buf, sizeof(buf), "%d OSC  %s", numOscs, descending ? "DESC" : "ASC");
            ag::text(vg, W - 8.f, 8.f, buf, 7.f,
                     descending ? ag::warm() : ag::accent(), ag::ALIGN_R, 0.5f);

            // ---- scope (left plot) ----
            {
                nvgSave(vg);
                nvgScissor(vg, LD_SCOPE_X, LD_PLOT_Y, LD_SCOPE_W, LD_PLOT_H);

                const float midY = LD_PLOT_Y + LD_PLOT_H / 2.f;
                const float ampPx = 30.f;
                const int NP = 154;
                const float windowSec = 0.004f;

                nvgBeginPath(vg);
                for (int px = 0; px < NP; ++px)
                {
                    float t = (float)px / (float)(NP - 1);
                    float timeSec = t * windowSec;
                    float sum = 0.f;
                    for (int i = 0; i < numOscs; ++i)
                    {
                        if (amp[i] < 0.05f)
                            continue;
                        float ph = phases[i] + timeSec * freqs[i];
                        ph -= std::floor(ph);
                        sum += ldWaveSample(ph, wave, pw) * amp[i];
                    }
                    sum *= 0.6f;
                    float x = LD_SCOPE_X + t * LD_SCOPE_W;
                    float y = midY - std::max(-1.f, std::min(1.f, sum)) * ampPx;
                    if (px == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, ag::bright());
                nvgStrokeWidth(vg, 1.5f);
                nvgLineCap(vg, NVG_ROUND);
                nvgLineJoin(vg, NVG_ROUND);
                nvgStroke(vg);

                nvgRestore(vg);
            }

            // ---- partial spectrum (right plot) ----
            {
                nvgSave(vg);
                nvgScissor(vg, LD_SPEC_X, LD_PLOT_Y, LD_SPEC_W, LD_PLOT_H);

                const float cx = LD_SPEC_X + LD_SPEC_W / 2.f;
                const float base = LD_PLOT_Y + LD_PLOT_H - 6.f;   // baseline
                const float maxH = LD_PLOT_H - 16.f;
                const float halfSpan = (float)(numOscs - 1) * 0.5f;
                const float sigma = std::max(1.0f, (float)(numOscs - 1) * 0.25f);
                const float R = halfSpan + 1.25f;                  // octaves from centre to edge
                const float pxPerOct = (LD_SPEC_W / 2.f - 8.f) / R;

                // Gaussian window (fixed): area + outline
                const float ex0 = LD_SPEC_X + 2.f, ex1 = LD_SPEC_X + LD_SPEC_W - 2.f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, ex0, base);
                for (float x = ex0; x <= ex1 + 0.01f; x += 2.f)
                {
                    float d = (x - cx) / pxPerOct;
                    nvgLineTo(vg, x, base - std::exp(-d * d / (2.f * sigma * sigma)) * maxH);
                }
                nvgLineTo(vg, ex1, base);
                nvgClosePath(vg);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
                nvgFill(vg);

                nvgBeginPath(vg);
                for (float x = ex0; x <= ex1 + 0.01f; x += 2.f)
                {
                    float d = (x - cx) / pxPerOct;
                    float y = base - std::exp(-d * d / (2.f * sigma * sigma)) * maxH;
                    if (x == ex0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // baseline and the rungs at sweep = 0
                nvgBeginPath(vg);
                nvgMoveTo(vg, ex0, base + 0.5f);
                nvgLineTo(vg, ex1, base + 0.5f);
                for (int i = 0; i < numOscs; ++i)
                {
                    float x = cx + ((float)i - halfSpan) * pxPerOct;
                    nvgMoveTo(vg, x, base);
                    nvgLineTo(vg, x, base + 3.f);
                }
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 50));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // partials: stem + head, positioned by log-frequency distance from the window centre
                for (int i = 0; i < numOscs; ++i)
                {
                    float a = std::max(0.f, std::min(1.f, amp[i]));
                    float d = sweep / 12.f + (float)i - halfSpan;
                    float x = cx + d * pxPerOct;
                    float p = std::max(-1.f, std::min(1.f, pan[i]));
                    NVGcolor c = p < 0.f ? ag::withA(ag::bright(), 0.6f + 0.4f * (-p))
                                         : ag::withA(ag::violet(), 0.6f + 0.4f * p);
                    float h = a * maxH;
                    if (h < 1.f)
                    {
                        nvgBeginPath(vg);
                        nvgRect(vg, x - 1.f, base - 2.f, 2.f, 2.f);
                        nvgFillColor(vg, nvgRGBA(255, 255, 255, 40));
                        nvgFill(vg);
                        continue;
                    }
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, x, base);
                    nvgLineTo(vg, x, base - h);
                    nvgStrokeColor(vg, c);
                    nvgStrokeWidth(vg, 2.f);
                    nvgLineCap(vg, NVG_BUTT);
                    nvgStroke(vg);

                    nvgBeginPath(vg);
                    nvgCircle(vg, x, base - h, 2.4f);
                    nvgFillColor(vg, c);
                    nvgFill(vg);
                }

                nvgRestore(vg);
            }

            // ---- footer ----
            ag::text(vg, LD_SCOPE_X + 2.f, LD_FOOT_TXT, "SCOPE 4 MS", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);

            // pan legend: L / R chips
            nvgBeginPath(vg);
            nvgRect(vg, LD_SPEC_X + 2.f, LD_FOOT_TXT - 2.5f, 5.f, 5.f);
            nvgFillColor(vg, ag::bright());
            nvgFill(vg);
            ag::text(vg, LD_SPEC_X + 10.f, LD_FOOT_TXT, "L", 7.f, ag::dim(), ag::ALIGN_L, 0.f);
            nvgBeginPath(vg);
            nvgRect(vg, LD_SPEC_X + 22.f, LD_FOOT_TXT - 2.5f, 5.f, 5.f);
            nvgFillColor(vg, ag::violet());
            nvgFill(vg);
            ag::text(vg, LD_SPEC_X + 30.f, LD_FOOT_TXT, "R", 7.f, ag::dim(), ag::ALIGN_L, 0.f);

            snprintf(buf, sizeof(buf), "SWEEP %.1f ST", sweep);
            ag::text(vg, LD_SPEC_X + LD_SPEC_W / 2.f + 6.f, LD_FOOT_TXT, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            ag::text(vg, LD_SPEC_X + LD_SPEC_W - 2.f, LD_FOOT_TXT, "LOW>HIGH", 7.f, ag::dim(), ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };
} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct LadderWidget : ModuleWidget
{
    // Text field that commits the entered Hz or note to FREQ_PARAM on Enter.
    struct FreqTextField : ui::TextField
    {
        Ladder *module = nullptr;

        void onSelectKey(const SelectKeyEvent &e) override
        {
            if (e.action == GLFW_PRESS &&
                (e.key == GLFW_KEY_ENTER || e.key == GLFW_KEY_KP_ENTER))
            {
                float semis = 0.f;
                if (module && ldParseFreqString(text.c_str(), semis))
                {
                    semis = rack::math::clamp(semis, -48.f, 48.f);
                    module->params[Ladder::FREQ_PARAM].setValue(semis);
                }
                if (auto *overlay = getAncestorOfType<ui::MenuOverlay>())
                    overlay->requestDelete();
                e.consume(this);
                return;
            }
            ui::TextField::onSelectKey(e);
        }
    };

    // Menu item that toggles the on-screen display between note and Hz.
    struct DisplayModeItem : MenuItem
    {
        Ladder *module = nullptr;
        Ladder::FreqDisplay mode = Ladder::DISP_NOTE;

        void onAction(const ActionEvent &e) override
        {
            if (module)
                module->freqDisplay = mode;
        }

        void step() override
        {
            rightText = CHECKMARK(module && module->freqDisplay == mode);
            MenuItem::step();
        }
    };

    LadderWidget(Ladder *module)
    {
        setModule(module);
        box.size = Vec(LD_PANEL_W, LD_PANEL_H);

        addChild(new LDPanel());

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new LDDisplay(module));

        // Row 1: FREQ, FINE, WAVE, PULSE (all the same knob size, centred in their cells)
        const int ids1[4] = {Ladder::FREQ_PARAM, Ladder::FINE_PARAM, Ladder::WAVE_PARAM, Ladder::PULSE_WIDTH_PARAM};
        for (int i = 0; i < 4; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(LD_ROW1_X[i], LD_ROW1_Y), module, ids1[i]));

        // Row 2: SPEED, OSCS, DIR
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(LD_ROW2_X[0], LD_ROW2_Y), module, Ladder::LADDER_SPEED_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(LD_ROW2_X[1], LD_ROW2_Y), module, Ladder::NUM_OSCS_PARAM));
        addParam(createParamCentered<CKSS>(Vec(LD_ROW2_X[2], LD_ROW2_Y), module, Ladder::DIRECTION_PARAM));

        // I/O: input left, outputs right
        addInput(createInputCentered<DarkPJ301MPort>(Vec(LD_IO_X[0], LD_IO_Y), module, Ladder::VOCT_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(LD_IO_X[1], LD_IO_Y), module, Ladder::OUT_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(LD_IO_X[2], LD_IO_Y), module, Ladder::OUT_R_OUTPUT));
    }

    void appendContextMenu(Menu *menu) override
    {
        Ladder *m = dynamic_cast<Ladder *>(this->module);
        if (!m)
            return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Set frequency (Hz or note, e.g. 440 or A#3):"));

        auto *tf = new FreqTextField();
        tf->module = m;
        tf->box.size.x = 130.f;
        menu->addChild(tf);

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Display as:"));

        auto *noteItem = createMenuItem<DisplayModeItem>("Note (C4, A#3, ...)");
        noteItem->module = m;
        noteItem->mode = Ladder::DISP_NOTE;
        menu->addChild(noteItem);

        auto *hzItem = createMenuItem<DisplayModeItem>("Frequency (Hz)");
        hzItem->module = m;
        hzItem->mode = Ladder::DISP_HZ;
        menu->addChild(hzItem);
    }
};

Model *modelLadder = createModel<Ladder, LadderWidget>("Ladder");