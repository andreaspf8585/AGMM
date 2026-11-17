#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>
#include <map>
#include <string>
#include <regex>
#include <algorithm>
#include <random>
#include "dsp/minblep.hpp"

// ============================================================================
// Stagger
// ============================================================================
//
// OVERVIEW
// --------
// Stagger is a 20 HP polyphonic VCO with a stereo "chaos" pitch drift. Each
// voice runs two independent oscillators (L and R) whose pitch is the base
// pitch plus a slow, filtered random drift — so the two channels slowly
// detune against each other, giving a wide, unstable, "staggering" tone.
//
// Controls:
//   FREQ         Base frequency (10 Hz .. 6 kHz, logarithmic)
//   CHAOS        Depth of the random pitch drift
//   WAVE         Waveform (sine / triangle / saw / square)
//   PW           Pulse width of the square wave
//   MULT         Chaos multiplier (x1, x2, x3, x5, x10)
//   RANGE        CV input range (+-5 V or +-10 V)
//   RESET        Resets the chaos drift to zero on a rising edge
//   V/OCT        1 V/oct pitch input (polyphonic)
//
// Each of the four knob rows also has a dedicated CV input and attenuverter,
// so chaos depth, waveform, and pulse width can all be modulated.
//
// Outputs:
//   OUT L, OUT R   Polyphonic VCOs, one voice per channel of the 1 V/oct input.
//                  The two channels drift independently for stereo width.
//
// The display shows the current waveform (two cycles) with a phase marker,
// plus a scrolling history of the chaos drift and its depth. The FREQ
// parameter is a custom ParamQuantity that accepts both frequency values
// ("440") and note names ("A4", "C#3") in its editable field.
//
// File structure:
//   1. Layout constants and shared display settings.
//   2. StFreqQ custom ParamQuantity for the frequency knob.
//   3. Stagger module: Voice struct, config, DSP, drift, per-voice outputs.
//   4. Anonymous-namespace UI helpers: stWave, STPanel, STDisplay.
//   5. StaggerWidget: places all knobs, jacks, switches and the display.
//   6. Model registration.
// ============================================================================

static const int ST_DRIFT_N = 128;             // more history points -> more spikes
static const float ST_CHAOS_DISP_GAIN = 100.f; // bring the tiny drift into view
static const float ST_CHAOS_LED_GAIN = 160.f;  // make the LED clearly bright

// ---- panel layout (20 HP = 300 px wide) ----
static const float ST_W = RACK_GRID_WIDTH * 20; // 300
static const float ST_H = RACK_GRID_HEIGHT;     // 380

// Display (240 x 80): header row, two framed plots, footer row. Bezel adds 4 px around it.
static const float ST_DX = 30.f, ST_DY = 26.f, ST_DW = 240.f, ST_DH = 80.f;
static const float ST_HEAD_LINE = 15.5f;               // hairline under the header row
static const float ST_PLOT_Y = 20.f, ST_PLOT_H = 40.f; // both plots share this band
static const float ST_FOOT_LINE = 64.5f;               // hairline above the footer row
static const float ST_FOOT_TXT = ST_DH - 8.f;          // footer text centre line
static const float ST_SC_X = 8.f, ST_SC_W = 150.f;     // waveform plot
static const float ST_DR_X = 170.f, ST_DR_W = 62.f;    // chaos drift plot

// Cells are 64 px wide on a 72 px pitch: label centred 10 px below the top edge
static const float ST_CW = 64.f;
// Returns the horizontal centre of control column i (0..3).
static float stCol(int i) { return 42.f + 72.f * (float)i; }

// Row A: FREQ, CHAOS, WAVE, PW. Knob at +30, "CV" / "ATT" captions at +50, jack and trimpot at +67
static const float ST_HDR_A = 122.f, ST_TOP_A = 130.f, ST_H_A = 84.f;
static const float ST_A_KNOB = ST_TOP_A + 30.f, ST_A_CAP = ST_TOP_A + 50.f, ST_A_JACK = ST_TOP_A + 67.f;

// Row B: MULT, RANGE, RESET, V/OCT. Control at +30
static const float ST_HDR_B = 224.f, ST_TOP_B = 232.f, ST_H_B = 50.f;
static const float ST_B_Y = ST_TOP_B + 30.f;

// Row C: CHAOS light, OUT L, OUT R. Control at +30
static const float ST_HDR_C = 292.f, ST_TOP_C = 300.f, ST_H_C = 50.f;
static const float ST_C_Y = ST_TOP_C + 30.f;
// Returns the horizontal centre of the bottom-row cell i (0..2).
static float stColC(int i) { return 78.f + 72.f * (float)i; }

// -----------------------------------------------------------------------------
// Custom ParamQuantity for the frequency knob (10 Hz - 6000 Hz, logarithmic)
// -----------------------------------------------------------------------------
// ParamQuantity that maps the raw knob (0..1) to a frequency on a logarithmic
// curve, and parses the editable display string as either Hz, a note name, or
// a semitone offset from C4.
struct StFreqQ : rack::engine::ParamQuantity
{
    // Converts a normalised knob value (0..1) to Hz on a 10 Hz..6 kHz log curve.
    static float mapToFreq(float v)
    {
        return 10.0f * std::pow(600.0f, v);
    }
    // Inverse of mapToFreq: converts a frequency in Hz back to a normalised
    // knob value, clamped to the 10 Hz..6 kHz range.
    static float mapFromFreq(float f)
    {
        f = math::clamp(f, 10.0f, 6000.0f);
        return std::log(f / 10.0f) / std::log(600.0f);
    }

    // Returns the effective display value: the knob mapped to Hz.
    float getDisplayValue() override
    {
        return mapToFreq(getValue());
    }

    // Accepts a display value (Hz) and converts it back to a knob position.
    void setDisplayValue(float displayValue) override
    {
        setValue(mapFromFreq(displayValue));
    }

    // Returns the display string for the current value, e.g. "261.6 Hz".
    std::string getDisplayValueString() override
    {
        float freq = getDisplayValue();
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f Hz", freq);
        return std::string(buf);
    }

    // Parses an editable string. Accepts:
    //   - a raw frequency ("440", "261.63")
    //   - a note name with octave ("A4", "C#3", "Bb2")
    //   - a semitone offset from C4 ("+12", "-5")
    // Whitespace is ignored; unknown input falls back to the base class.
    void setDisplayValueString(std::string s) override
    {
        s.erase(std::remove_if(s.begin(), s.end(), ::isspace), s.end());
        if (s.empty())
            return;

        // Parses a note name like "A4", "C#3", "Bb2" into a frequency in Hz.
        // Returns -1 if the string is not a valid note name.
        auto parseNote = [](const std::string &note) -> float
        {
            static const std::map<std::string, int> noteMap = {
                {"C", 0}, {"C#", 1}, {"Db", 1}, {"D", 2}, {"D#", 3}, {"Eb", 3}, {"E", 4}, {"F", 5}, {"F#", 6}, {"Gb", 6}, {"G", 7}, {"G#", 8}, {"Ab", 8}, {"A", 9}, {"A#", 10}, {"Bb", 10}, {"B", 11}};
            std::regex noteRegex(R"(([A-Ga-g][#b]?)(-?\d+))");
            std::smatch match;
            if (std::regex_match(note, match, noteRegex))
            {
                std::string pitchClass = match[1].str();
                int octave = std::stoi(match[2].str());
                for (auto &c : pitchClass)
                    c = toupper(c);
                if (noteMap.find(pitchClass) != noteMap.end())
                {
                    int semitone = noteMap.at(pitchClass) + (octave + 1) * 12;
                    float freq = 440.0f * std::pow(2.0f, (semitone - 69) / 12.0f);
                    return freq;
                }
            }
            return -1.0f;
        };

        // Try a plain frequency first
        try
        {
            size_t pos;
            float freq = std::stof(s, &pos);
            if (pos == s.length())
            {
                setDisplayValue(freq);
                return;
            }
        }
        catch (...)
        {
        }

        // Then try a note name
        float noteFreq = parseNote(s);
        if (noteFreq > 0)
        {
            setDisplayValue(noteFreq);
            return;
        }

        // Then try a signed semitone offset from C4
        if (s[0] == '+' || s[0] == '-')
        {
            try
            {
                int semitones = std::stoi(s);
                float freq = dsp::FREQ_C4 * std::pow(2.0f, semitones / 12.0f);
                setDisplayValue(freq);
                return;
            }
            catch (...)
            {
            }
        }

        ParamQuantity::setDisplayValueString(s);
    }
};

// ============================================================================
// Module
// ============================================================================
// Polyphonic stereo VCO with two independent filtered-random pitch drifts
// (one per channel), configurable waveforms, and CV control over pitch,
// chaos depth, waveform and pulse width.
struct Stagger : Module
{
    enum ParamId
    {
        PITCH_PARAM,
        CHAOS_PARAM,
        CHAOS_MULT_PARAM,
        WAVE_PARAM,
        PW_PARAM,
        PITCH_ATTEN_PARAM,
        CHAOS_ATTEN_PARAM,
        WAVE_ATTEN_PARAM,
        PW_ATTEN_PARAM,
        CV_RANGE_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        CV_INPUT,
        PITCH_CV_INPUT,
        CHAOS_CV_INPUT,
        WAVE_CV_INPUT,
        PW_CV_INPUT,
        CHAOS_RESET_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        VCO_L_OUTPUT,
        VCO_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        CHAOS_LIGHT,
        LIGHTS_LEN
    };

    // Per-voice oscillator state: separate phase and minBLEP generators for
    // the left and right channels, so their drifts decouple cleanly.
    struct Voice
    {
        // Left channel
        float phase = 0.0f;
        float lastPhase = 0.0f;
        dsp::MinBlepGenerator<16, 16> blepRising;
        dsp::MinBlepGenerator<16, 16> blepFalling;
        dsp::MinBlepGenerator<16, 16> blepSaw;

        // Right channel (independent chaos drift -> stereo decorrelation)
        float phaseR = 0.0f;
        float lastPhaseR = 0.0f;
        dsp::MinBlepGenerator<16, 16> blepRisingR;
        dsp::MinBlepGenerator<16, 16> blepFallingR;
        dsp::MinBlepGenerator<16, 16> blepSawR;
    };
    std::vector<Voice> voices;

    float chaosDrift = 0.0f;  // left
    float chaosDriftR = 0.0f; // right
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist{-1.0f, 1.0f};

    // Predefined chaos multipliers exposed on the MULT switch.
    static constexpr float multTable[5] = {1.0f, 2.0f, 3.0f, 5.0f, 10.0f};
    bool cvRangeTenVolt = false;
    float lastResetVoltage = 0.0f;

    // Display data (written by the audio thread, read by the UI thread)
    float dispFreq = 261.6f, dispPw = 0.5f, dispPhase = 0.f, dispChaos = 0.f;
    int dispWave = 0;
    int dispChannels = 1;
    float driftHist[ST_DRIFT_N] = {};
    int driftHead = 0;
    float driftTimer = 0.f;
    float driftPeak = 0.f; // per-frame peak-hold (signed) for a spikier trace

    // Constructor: configures every parameter (with custom ParamQuantity for
    // FREQ), input, output and light, and seeds the RNG.
    Stagger()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam<StFreqQ>(PITCH_PARAM, 0.f, 1.f, StFreqQ::mapFromFreq(dsp::FREQ_C4), "Frequency");

        configParam(CHAOS_PARAM, 0.f, 1.f, 0.0f, "Chaos depth", "%", 0.f, 100.f);
        configSwitch(CHAOS_MULT_PARAM, 0.f, 4.f, 0.f, "Chaos multiplier", {"x1", "x2", "x3", "x5", "x10"});
        configSwitch(WAVE_PARAM, 0.f, 3.f, 0.f, "Waveform", {"Sine", "Triangle", "Saw", "Square"});
        configParam(PW_PARAM, 0.f, 1.f, 0.5f, "Pulse width", "%", 0.f, 100.f);
        configParam(PITCH_ATTEN_PARAM, -1.f, 1.f, 0.f, "Pitch CV attenuverter", "%", 0.f, 100.f);
        configParam(CHAOS_ATTEN_PARAM, -1.f, 1.f, 0.f, "Chaos CV attenuverter", "%", 0.f, 100.f);
        configParam(WAVE_ATTEN_PARAM, -1.f, 1.f, 0.f, "Waveform CV attenuverter", "%", 0.f, 100.f);
        configParam(PW_ATTEN_PARAM, -1.f, 1.f, 0.f, "Pulse width CV attenuverter", "%", 0.f, 100.f);
        configSwitch(CV_RANGE_PARAM, 0.f, 1.f, 0.f, "CV range", {"+-5V", "+-10V"});

        configInput(CV_INPUT, "1V/octave pitch");
        configInput(PITCH_CV_INPUT, "Pitch modulation CV");
        configInput(CHAOS_CV_INPUT, "Chaos depth modulation CV");
        configInput(WAVE_CV_INPUT, "Waveform selection CV");
        configInput(PW_CV_INPUT, "Pulse width modulation CV");
        configInput(CHAOS_RESET_INPUT, "Reset chaos drift (trigger)");
        configOutput(VCO_L_OUTPUT, "VCO Left");
        configOutput(VCO_R_OUTPUT, "VCO Right");
        configLight(CHAOS_LIGHT, "Chaos activity");

        std::random_device rd;
        rng.seed(rd());
    }

    // Rack reset hook: clears both chaos drifts and the history buffer, and
    // drops all voices so they get recreated on the next process().
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        chaosDrift = 0.0f;
        chaosDriftR = 0.0f;
        driftPeak = 0.f;
        std::fill(driftHist, driftHist + ST_DRIFT_N, 0.f);
        voices.clear();
    }

    // Main audio callback: reads parameters and CVs, updates both chaos
    // drift filters, renders every active voice's left/right oscillators
    // with minBLEP-corrected waveforms, writes the outputs and updates the
    // LED and display history.
    void process(const ProcessArgs &args) override
    {
        float freqKnob = StFreqQ::mapToFreq(params[PITCH_PARAM].getValue());
        float basePitchVoltage = std::log2(freqKnob / dsp::FREQ_C4);

        float chaosKnob = params[CHAOS_PARAM].getValue();
        float chaosMultIndex = params[CHAOS_MULT_PARAM].getValue();
        float waveKnob = params[WAVE_PARAM].getValue();
        float pwKnob = params[PW_PARAM].getValue();

        float chaosMultiplier = multTable[clamp((int)std::nearbyint(chaosMultIndex), 0, 4)];

        float pitchAtten = params[PITCH_ATTEN_PARAM].getValue();
        float chaosAtten = params[CHAOS_ATTEN_PARAM].getValue();
        float waveAtten = params[WAVE_ATTEN_PARAM].getValue();
        float pwAtten = params[PW_ATTEN_PARAM].getValue();

        cvRangeTenVolt = (params[CV_RANGE_PARAM].getValue() > 0.5f);
        float cvVoltageScale = cvRangeTenVolt ? 0.5f : 1.0f;

        // Chaos reset on a rising edge of the reset input
        float resetVoltage = inputs[CHAOS_RESET_INPUT].getVoltage();
        if (resetVoltage > 1.0f && lastResetVoltage <= 1.0f)
        {
            chaosDrift = 0.0f;
            chaosDriftR = 0.0f;
        }
        lastResetVoltage = resetVoltage;

        // Independent filtered noise per channel -> decorrelated stereo pitch drift
        float whiteNoiseL = dist(rng);
        float whiteNoiseR = dist(rng);
        float cutoffHz = 1.5f;
        float alpha = 2.0f * M_PI * cutoffHz * args.sampleTime;
        alpha = std::min(alpha, 0.5f);
        chaosDrift = (1.0f - alpha) * chaosDrift + alpha * whiteNoiseL;
        chaosDriftR = (1.0f - alpha) * chaosDriftR + alpha * whiteNoiseR;

        // Grow the voice count to match the polyphonic input's channel count
        int channels = std::max(1, inputs[CV_INPUT].getChannels());
        outputs[VCO_L_OUTPUT].setChannels(channels);
        outputs[VCO_R_OUTPUT].setChannels(channels);
        if ((int)voices.size() < channels)
        {
            voices.resize(channels);
        }
        dispChannels = channels;

        // Fast 2^x approximation (bit-twiddling exp2, accurate enough for
        // pitch conversion at ~1e-3 relative error).
        auto fastExp2 = [](float x)
        {
            if (x == 0.0f)
                return 1.0f;
            union
            {
                float f;
                uint32_t i;
            } u;
            u.i = (uint32_t)((1 << 23) * (x + 126.94269504f));
            return u.f;
        };

        // Renders one channel of the chosen waveform for a voice, with
        // minBLEP corrections on the saw/square discontinuities. Returns the
        // waveform sample in [-1, 1].
        auto genWave = [&](float &phase, float &lastPhase, int waveIdx, float pw,
                           dsp::MinBlepGenerator<16, 16> &bRise,
                           dsp::MinBlepGenerator<16, 16> &bFall,
                           dsp::MinBlepGenerator<16, 16> &bSaw,
                           float dPhase) -> float
        {
            float out = 0.0f;
            lastPhase = phase;
            phase += dPhase;
            if (phase >= 1.0f)
                phase -= 1.0f;

            switch (waveIdx)
            {
            case 0:
                out = std::sin(2.0f * M_PI * phase);
                break;
            case 1:
                out = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
                break;
            case 2:
            {
                float saw = 1.0f - 2.0f * phase;
                if (phase < lastPhase)
                {
                    float t = (1.0f - lastPhase) / dPhase;
                    bSaw.insertDiscontinuity(t, 2.0f);
                }
                out = saw + bSaw.process();
                break;
            }
            case 3:
            {
                float square = (phase < pw) ? 1.0f : -1.0f;
                if (phase < lastPhase)
                {
                    float t = (1.0f - lastPhase) / dPhase;
                    bRise.insertDiscontinuity(t, 2.0f);
                }
                if ((lastPhase < pw && phase >= pw) || (pw == 0.0f && phase < lastPhase))
                {
                    float t = (pw - lastPhase) / dPhase;
                    bFall.insertDiscontinuity(t, -2.0f);
                }
                out = square + bRise.process() + bFall.process();
                break;
            }
            }
            return out;
        };

        for (int c = 0; c < channels; ++c)
        {
            float pitchCV = inputs[PITCH_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float chaosCV = inputs[CHAOS_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float waveCV = inputs[WAVE_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float pwCV = inputs[PW_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;

            float modulatedPitchCV = pitchCV * pitchAtten;
            float modulatedChaosCV = chaosCV * chaosAtten;
            float modulatedWaveCV = waveCV * waveAtten;
            float modulatedPwCV = pwCV * pwAtten;

            float totalPitchCV = basePitchVoltage + inputs[CV_INPUT].getPolyVoltage(c) + modulatedPitchCV;

            float chaosDepth = chaosKnob + modulatedChaosCV / 5.0f;
            chaosDepth = math::clamp(chaosDepth, 0.0f, 1.0f);

            // --- Left pitch ---
            float chaosVoltageL = chaosDrift * chaosDepth * chaosMultiplier;
            float totalCVL = totalPitchCV + chaosVoltageL;
            float freqL = dsp::FREQ_C4 * fastExp2(totalCVL);
            freqL = math::clamp(freqL, 10.0f, 6000.0f);
            float deltaPhaseL = freqL * args.sampleTime;

            // --- Right pitch (independent chaos drift) ---
            float chaosVoltageR = chaosDriftR * chaosDepth * chaosMultiplier;
            float totalCVR = totalPitchCV + chaosVoltageR;
            float freqR = dsp::FREQ_C4 * fastExp2(totalCVR);
            freqR = math::clamp(freqR, 10.0f, 6000.0f);
            float deltaPhaseR = freqR * args.sampleTime;

            // Waveform select (quantized to the 4 discrete waves)
            float waveSelect = waveKnob + (modulatedWaveCV / 5.0f) * 2.0f;
            waveSelect = math::clamp(waveSelect, 0.0f, 3.0f);
            int waveIndex = (int)std::nearbyint(waveSelect);

            // Pulse width (clamped to avoid degenerate pulse edges)
            float pwValue = pwKnob + (modulatedPwCV / 10.0f);
            pwValue = math::clamp(pwValue, 0.01f, 0.99f);

            Voice &voice = voices[c];

            float outL = genWave(voice.phase, voice.lastPhase, waveIndex, pwValue,
                                 voice.blepRising, voice.blepFalling, voice.blepSaw,
                                 deltaPhaseL);
            float outR = genWave(voice.phaseR, voice.lastPhaseR, waveIndex, pwValue,
                                 voice.blepRisingR, voice.blepFallingR, voice.blepSawR,
                                 deltaPhaseR);

            outputs[VCO_L_OUTPUT].setVoltage(outL * 5.0f, c);
            outputs[VCO_R_OUTPUT].setVoltage(outR * 5.0f, c);

            if (c == 0)
            {
                dispFreq = freqL;
                dispPw = pwValue;
                dispWave = waveIndex;
                dispPhase = voice.phase;
                dispChaos = chaosDrift * chaosDepth;
                // peak-hold (signed) within the display frame -> spikier trace
                if (std::abs(dispChaos) > std::abs(driftPeak))
                    driftPeak = dispChaos;
            }
        }

        // LED: scaled up so it is clearly bright and visibly follows the drift
        float chaosIntensity = math::clamp(std::abs(chaosDrift) * chaosKnob * ST_CHAOS_LED_GAIN, 0.f, 1.f);
        lights[CHAOS_LIGHT].setSmoothBrightness(chaosIntensity, args.sampleTime);

        // Chaos history for the display (~60 points per second)
        driftTimer += args.sampleTime;
        if (driftTimer >= 1.f / 60.f)
        {
            driftTimer -= 1.f / 60.f;
            if (driftTimer > 1.f / 60.f)
                driftTimer = 0.f;
            driftHist[driftHead] = math::clamp(driftPeak * ST_CHAOS_DISP_GAIN, -1.f, 1.f);
            driftHead = (driftHead + 1) % ST_DRIFT_N;
            driftPeak = 0.f;
        }
    }
};

constexpr float Stagger::multTable[5];

// Anonymous namespace: UI helpers and the two panel widgets.
namespace
{

    namespace ag = agmm;

    // Evaluates the selected waveform at normalized phase p (0..1), using pw
    // for the pulse width of the square wave. Matches the DSP waveforms.
    float stWave(float p, int wave, float pw)
    {
        switch (wave)
        {
        case 1:
            return (p < 0.5f) ? (4.f * p - 1.f) : (3.f - 4.f * p);
        case 2:
            return 1.f - 2.f * p;
        case 3:
            return (p < pw) ? 1.f : -1.f;
        default:
            return std::sin(2.f * (float)M_PI * p);
        }
    }

    // ------------------------------------------------------------
    // Panel background: title, display frame, one labelled cell per control
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the VCO section with its four
    // knob cells and CV/ATT captions, the CONTROL section with four cells, and
    // the OUTPUT section with its three cells.
    struct STPanel : TransparentWidget
    {
        STPanel() { box.size = Vec(ST_W, ST_H); }
        // Draws all static panel art including section headers and cell plates.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, ST_W, ST_H, "STAGGER");
            ag::bezel(vg, ST_DX, ST_DY, ST_DW, ST_DH);

            ag::sectionHeader(vg, ST_W, ST_HDR_A, "VCO", 10.f);
            const char *a[4] = {"FREQ", "CHAOS", "WAVE", "PW"};
            for (int i = 0; i < 4; ++i)
            {
                float cx = stCol(i);
                ag::cell(vg, cx, ST_TOP_A, ST_CW, ST_H_A, a[i]);
                ag::text(vg, cx - 16.f, ST_A_CAP, "CV", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
                ag::text(vg, cx + 16.f, ST_A_CAP, "ATT", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            }

            ag::sectionHeader(vg, ST_W, ST_HDR_B, "CONTROL", 10.f);
            const char *b[4] = {"MULT", "RANGE", "RESET", "V/OCT"};
            for (int i = 0; i < 4; ++i)
                ag::cell(vg, stCol(i), ST_TOP_B, ST_CW, ST_H_B, b[i]);

            ag::sectionHeader(vg, ST_W, ST_HDR_C, "OUTPUT", 10.f);
            ag::cell(vg, stColC(0), ST_TOP_C, ST_CW, ST_H_C, "CHAOS");
            ag::cell(vg, stColC(1), ST_TOP_C, ST_CW, ST_H_C, "OUT L", true);
            ag::cell(vg, stColC(2), ST_TOP_C, ST_CW, ST_H_C, "OUT R", true);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: waveform (two cycles, phase marker) and chaos drift history
    // ------------------------------------------------------------
    // Live display: draws the current waveform (two cycles) with a moving phase
    // marker, the scrolling chaos drift history, header readouts (frequency,
    // waveform, poly channel count) and footer readouts (pulse width or cycle
    // count, chaos depth). Uses demo data in the module browser.
    struct STDisplay : TransparentWidget
    {
        Stagger *module = nullptr;
        // Constructor: positions the display widget under its bezel.
        STDisplay(Stagger *m) : module(m)
        {
            box.pos = Vec(ST_DX, ST_DY);
            box.size = Vec(ST_DW, ST_DH);
        }

        // Static layer: fills the screen, draws the header/footer hairlines, the
        // two plot frames and their zero lines.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, ST_DW, ST_DH);

            // separators under the header and above the footer
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, ST_HEAD_LINE);
            nvgLineTo(vg, ST_DW - 8.f, ST_HEAD_LINE);
            nvgMoveTo(vg, 8.f, ST_FOOT_LINE);
            nvgLineTo(vg, ST_DW - 8.f, ST_FOOT_LINE);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            const float xs[2] = {ST_SC_X, ST_DR_X}, ws[2] = {ST_SC_W, ST_DR_W};
            for (int k = 0; k < 2; ++k)
            {
                ag::plotFrame(vg, xs[k], ST_PLOT_Y, ws[k], ST_PLOT_H);
                nvgBeginPath(vg); // zero line
                nvgMoveTo(vg, xs[k] + 1.f, ST_PLOT_Y + ST_PLOT_H / 2.f);
                nvgLineTo(vg, xs[k] + ws[k] - 1.f, ST_PLOT_Y + ST_PLOT_H / 2.f);
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }
            Widget::draw(args);
        }

        // Layer-1 overlay: draws the waveform (two cycles) with a phase marker,
        // the chaos drift history trace, and the header/footer readouts.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = ST_DW;

            // Snapshot (or demo values in the module browser)
            float freq = 261.6f, pw = 0.5f, phase = 0.3f, depth = 0.4f;
            int wave = 0, ch = 1;
            float hist[ST_DRIFT_N];
            if (module)
            {
                freq = module->dispFreq;
                pw = module->dispPw;
                phase = module->dispPhase;
                wave = module->dispWave;
                ch = module->dispChannels;
                depth = module->params[Stagger::CHAOS_PARAM].getValue();
                for (int i = 0; i < ST_DRIFT_N; ++i)
                    hist[i] = module->driftHist[(module->driftHead + i) % ST_DRIFT_N];
            }
            else
            {
                // Demo trace: same visual density for any ST_DRIFT_N
                for (int i = 0; i < ST_DRIFT_N; ++i)
                {
                    float t = (float)i / (ST_DRIFT_N - 1);
                    hist[i] = 0.45f * std::sin(t * 12.f) + 0.25f * std::sin(t * 34.f + 1.f);
                }
            }
            wave = std::min(std::max(wave, 0), 3);

            // ---- waveform plot ----
            const float mid = ST_PLOT_Y + ST_PLOT_H / 2.f, amp = ST_PLOT_H / 2.f - 4.f;
            nvgBeginPath(vg);
            const int N = 150;
            for (int i = 0; i <= N; ++i)
            {
                float t = 2.f * (float)i / N;
                float p = t - std::floor(t);
                float v = stWave(p, wave, pw);
                float x = ST_SC_X + ST_SC_W * (float)i / N, y = mid - v * amp;
                if (i == 0)
                    nvgMoveTo(vg, x, y);
                else
                    nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, ag::bright());
            nvgStrokeWidth(vg, 1.5f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);

            // phase marker: hairline and dot
            {
                float x = ST_SC_X + ST_SC_W * 0.5f * phase, y = mid - stWave(phase, wave, pw) * amp;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x, ST_PLOT_Y + 1.f);
                nvgLineTo(vg, x, ST_PLOT_Y + ST_PLOT_H - 1.f);
                nvgStrokeColor(vg, ag::withA(ag::violet(), 0.5f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, 2.f);
                nvgFillColor(vg, ag::violet());
                nvgFill(vg);
            }

            // ---- chaos drift plot (128 history points span the plot width) ----
            nvgSave(vg);
            nvgScissor(vg, ST_DR_X, ST_PLOT_Y, ST_DR_W, ST_PLOT_H);
            const float dm = ST_PLOT_Y + ST_PLOT_H / 2.f, da = ST_PLOT_H / 2.f - 3.f;
            const float xStep = (ST_DR_W - 2.f) / (ST_DRIFT_N - 1);
            nvgBeginPath(vg);
            for (int i = 0; i < ST_DRIFT_N; ++i)
            {
                float v = std::min(std::max(hist[i], -1.f), 1.f);
                float x = ST_DR_X + 1.f + (float)i * xStep, y = dm - v * da;
                if (i == 0)
                    nvgMoveTo(vg, x, y);
                else
                    nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, ag::violet());
            nvgStrokeWidth(vg, 1.2f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
            nvgRestore(vg);

            // ---- header / footer readouts ----
            static const char *wn[4] = {"SINE", "TRI", "SAW", "SQR"};
            char buf[48];
            snprintf(buf, sizeof(buf), "%.1f HZ", freq);
            ag::text(vg, 8.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "%s%s", wn[wave], ch > 1 ? "  POLY" : "");
            ag::text(vg, W - 8.f, 8.f, buf, 7.f, ag::warm(), ag::ALIGN_R, 0.5f);

            if (wave == 3)
                snprintf(buf, sizeof(buf), "PW %.0f%%", pw * 100.f);
            else
                snprintf(buf, sizeof(buf), "2 CYCLES");
            ag::text(vg, ST_SC_X + 2.f, ST_FOOT_TXT, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "CHAOS %.0f%%", depth * 100.f);
            ag::text(vg, ST_DR_X + ST_DR_W - 2.f, ST_FOOT_TXT, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
// Places the static panel, screws, live display, and the three rows of
// controls (VCO knobs with CV/attenuverters, CONTROL switches/jacks, OUTPUT
// light and jacks).
struct StaggerWidget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, display, row A
    // (knob + CV + attenuverter per cell), row B (mult, range, reset, v/oct)
    // and row C (chaos LED + two output jacks).
    StaggerWidget(Stagger *module)
    {
        setModule(module);
        box.size = Vec(ST_W, ST_H);

        addChild(new STPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new STDisplay(module));

        // Row A: knob, CV input (left) and attenuverter (right) per cell
        const int knob[4] = {Stagger::PITCH_PARAM, Stagger::CHAOS_PARAM, Stagger::WAVE_PARAM, Stagger::PW_PARAM};
        const int att[4] = {Stagger::PITCH_ATTEN_PARAM, Stagger::CHAOS_ATTEN_PARAM, Stagger::WAVE_ATTEN_PARAM, Stagger::PW_ATTEN_PARAM};
        const int cv[4] = {Stagger::PITCH_CV_INPUT, Stagger::CHAOS_CV_INPUT, Stagger::WAVE_CV_INPUT, Stagger::PW_CV_INPUT};
        for (int i = 0; i < 4; ++i)
        {
            float cx = stCol(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, ST_A_KNOB), module, knob[i]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 16.f, ST_A_JACK), module, cv[i]));
            addParam(createParamCentered<Trimpot>(Vec(cx + 16.f, ST_A_JACK), module, att[i]));
        }

        // Row B
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(stCol(0), ST_B_Y), module, Stagger::CHAOS_MULT_PARAM));
        addParam(createParamCentered<CKSS>(Vec(stCol(1), ST_B_Y), module, Stagger::CV_RANGE_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(stCol(2), ST_B_Y), module, Stagger::CHAOS_RESET_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(stCol(3), ST_B_Y), module, Stagger::CV_INPUT));

        // Row C: chaos light, then the outputs
        addChild(createLightCentered<SmallLight<ag::CyanLight>>(Vec(stColC(0), ST_C_Y), module, Stagger::CHAOS_LIGHT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(stColC(1), ST_C_Y), module, Stagger::VCO_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(stColC(2), ST_C_Y), module, Stagger::VCO_R_OUTPUT));
    }
};

Model *modelStagger = createModel<Stagger, StaggerWidget>("Stagger");