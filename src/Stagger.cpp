#include "plugin.hpp"
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

static const int   ST_DRIFT_N = 128;                 // more history points -> more spikes
static const float ST_CHAOS_DISP_GAIN = 100.f;       // bring the tiny drift into view
static const float ST_CHAOS_LED_GAIN  = 160.f;       // make the LED clearly bright
static const float ST_W = RACK_GRID_WIDTH * 20;   // 300
static const float ST_H = RACK_GRID_HEIGHT;       // 380
static const float ST_DX = 30.f, ST_DY = 46.f, ST_DW = 240.f, ST_DH = 62.f;
static const float ST_RA = 170.f;     // row A knob centre (4 CV cells)
static const float ST_RB = 250.f;     // row B control centre
static const float ST_IOY = 338.f;
static float stCol(int i) { return 37.5f + 75.f * i; }

// -----------------------------------------------------------------------------
// Custom ParamQuantity for the frequency knob (10 Hz - 6000 Hz, logarithmic)
// -----------------------------------------------------------------------------
struct StFreqQ : rack::engine::ParamQuantity {
    static float mapToFreq(float v) {
        return 10.0f * std::pow(600.0f, v);
    }
    static float mapFromFreq(float f) {
        f = math::clamp(f, 10.0f, 6000.0f);
        return std::log(f / 10.0f) / std::log(600.0f);
    }

    float getDisplayValue() override {
        return mapToFreq(getValue());
    }

    void setDisplayValue(float displayValue) override {
        setValue(mapFromFreq(displayValue));
    }

    std::string getDisplayValueString() override {
        float freq = getDisplayValue();
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f Hz", freq);
        return std::string(buf);
    }

    void setDisplayValueString(std::string s) override {
        s.erase(std::remove_if(s.begin(), s.end(), ::isspace), s.end());
        if (s.empty()) return;

        auto parseNote = [](const std::string& note) -> float {
            static const std::map<std::string, int> noteMap = {
                {"C", 0}, {"C#", 1}, {"Db", 1}, {"D", 2}, {"D#", 3}, {"Eb", 3},
                {"E", 4}, {"F", 5}, {"F#", 6}, {"Gb", 6}, {"G", 7}, {"G#", 8},
                {"Ab", 8}, {"A", 9}, {"A#", 10}, {"Bb", 10}, {"B", 11}
            };
            std::regex noteRegex(R"(([A-Ga-g][#b]?)(-?\d+))");
            std::smatch match;
            if (std::regex_match(note, match, noteRegex)) {
                std::string pitchClass = match[1].str();
                int octave = std::stoi(match[2].str());
                for (auto& c : pitchClass) c = toupper(c);
                if (noteMap.find(pitchClass) != noteMap.end()) {
                    int semitone = noteMap.at(pitchClass) + (octave + 1) * 12;
                    float freq = 440.0f * std::pow(2.0f, (semitone - 69) / 12.0f);
                    return freq;
                }
            }
            return -1.0f;
        };

        try {
            size_t pos;
            float freq = std::stof(s, &pos);
            if (pos == s.length()) {
                setDisplayValue(freq);
                return;
            }
        } catch (...) {}

        float noteFreq = parseNote(s);
        if (noteFreq > 0) {
            setDisplayValue(noteFreq);
            return;
        }

        if (s[0] == '+' || s[0] == '-') {
            try {
                int semitones = std::stoi(s);
                float freq = dsp::FREQ_C4 * std::pow(2.0f, semitones / 12.0f);
                setDisplayValue(freq);
                return;
            } catch (...) {}
        }

        ParamQuantity::setDisplayValueString(s);
    }
};

struct Stagger : Module {
    enum ParamId {
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
    enum InputId {
        CV_INPUT,
        PITCH_CV_INPUT,
        CHAOS_CV_INPUT,
        WAVE_CV_INPUT,
        PW_CV_INPUT,
        CHAOS_RESET_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        VCO_L_OUTPUT,
        VCO_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        CHAOS_LIGHT,
        LIGHTS_LEN
    };

    struct Voice {
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

    float chaosDrift = 0.0f;   // left
    float chaosDriftR = 0.0f;  // right
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist{-1.0f, 1.0f};

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
    float driftPeak = 0.f;   // per-frame peak-hold (signed) for a spikier trace

    Stagger() {
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

    void onReset() override {
        chaosDrift = 0.0f;
        chaosDriftR = 0.0f;
        driftPeak = 0.f;
        std::fill(driftHist, driftHist + ST_DRIFT_N, 0.f);
        voices.clear();
    }

    void process(const ProcessArgs& args) override {
        float freqKnob = StFreqQ::mapToFreq(params[PITCH_PARAM].getValue());
        float basePitchVoltage = std::log2(freqKnob / dsp::FREQ_C4);

        float chaosKnob = params[CHAOS_PARAM].getValue();
        float chaosMultIndex = params[CHAOS_MULT_PARAM].getValue();
        float waveKnob = params[WAVE_PARAM].getValue();
        float pwKnob = params[PW_PARAM].getValue();

        float chaosMultiplier = multTable[clamp((int)std::nearbyint(chaosMultIndex), 0, 4)];

        float pitchAtten = params[PITCH_ATTEN_PARAM].getValue();
        float chaosAtten = params[CHAOS_ATTEN_PARAM].getValue();
        float waveAtten  = params[WAVE_ATTEN_PARAM].getValue();
        float pwAtten    = params[PW_ATTEN_PARAM].getValue();

        cvRangeTenVolt = (params[CV_RANGE_PARAM].getValue() > 0.5f);
        float cvVoltageScale = cvRangeTenVolt ? 0.5f : 1.0f;

        float resetVoltage = inputs[CHAOS_RESET_INPUT].getVoltage();
        if (resetVoltage > 1.0f && lastResetVoltage <= 1.0f) {
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
        chaosDrift  = (1.0f - alpha) * chaosDrift  + alpha * whiteNoiseL;
        chaosDriftR = (1.0f - alpha) * chaosDriftR + alpha * whiteNoiseR;

        int channels = std::max(1, inputs[CV_INPUT].getChannels());
        outputs[VCO_L_OUTPUT].setChannels(channels);
        outputs[VCO_R_OUTPUT].setChannels(channels);
        if ((int)voices.size() < channels) {
            voices.resize(channels);
        }
        dispChannels = channels;

        auto fastExp2 = [](float x) {
            if (x == 0.0f) return 1.0f;
            union { float f; uint32_t i; } u;
            u.i = (uint32_t)((1 << 23) * (x + 126.94269504f));
            return u.f;
        };

        auto genWave = [&](float& phase, float& lastPhase, int waveIdx, float pw,
                           dsp::MinBlepGenerator<16,16>& bRise,
                           dsp::MinBlepGenerator<16,16>& bFall,
                           dsp::MinBlepGenerator<16,16>& bSaw,
                           float dPhase) -> float {
            float out = 0.0f;
            lastPhase = phase;
            phase += dPhase;
            if (phase >= 1.0f) phase -= 1.0f;

            switch (waveIdx) {
                case 0:
                    out = std::sin(2.0f * M_PI * phase);
                    break;
                case 1:
                    out = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
                    break;
                case 2: {
                    float saw = 1.0f - 2.0f * phase;
                    if (phase < lastPhase) {
                        float t = (1.0f - lastPhase) / dPhase;
                        bSaw.insertDiscontinuity(t, 2.0f);
                    }
                    out = saw + bSaw.process();
                    break;
                }
                case 3: {
                    float square = (phase < pw) ? 1.0f : -1.0f;
                    if (phase < lastPhase) {
                        float t = (1.0f - lastPhase) / dPhase;
                        bRise.insertDiscontinuity(t, 2.0f);
                    }
                    if ((lastPhase < pw && phase >= pw) || (pw == 0.0f && phase < lastPhase)) {
                        float t = (pw - lastPhase) / dPhase;
                        bFall.insertDiscontinuity(t, -2.0f);
                    }
                    out = square + bRise.process() + bFall.process();
                    break;
                }
            }
            return out;
        };

        for (int c = 0; c < channels; ++c) {
            float pitchCV = inputs[PITCH_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float chaosCV = inputs[CHAOS_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float waveCV  = inputs[WAVE_CV_INPUT].getPolyVoltage(c)  * cvVoltageScale;
            float pwCV    = inputs[PW_CV_INPUT].getPolyVoltage(c)    * cvVoltageScale;

            float modulatedPitchCV = pitchCV * pitchAtten;
            float modulatedChaosCV = chaosCV * chaosAtten;
            float modulatedWaveCV  = waveCV  * waveAtten;
            float modulatedPwCV    = pwCV    * pwAtten;

            float totalPitchCV = basePitchVoltage
                               + inputs[CV_INPUT].getPolyVoltage(c)
                               + modulatedPitchCV;

            float chaosDepth = chaosKnob + modulatedChaosCV / 5.0f;
            chaosDepth = math::clamp(chaosDepth, 0.0f, 1.0f);

            // --- Left pitch ---
            float chaosVoltageL = chaosDrift  * chaosDepth * chaosMultiplier;
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

            float waveSelect = waveKnob + (modulatedWaveCV / 5.0f) * 2.0f;
            waveSelect = math::clamp(waveSelect, 0.0f, 3.0f);
            int waveIndex = (int)std::nearbyint(waveSelect);

            float pwValue = pwKnob + (modulatedPwCV / 10.0f);
            pwValue = math::clamp(pwValue, 0.01f, 0.99f);

            Voice& voice = voices[c];

            float outL = genWave(voice.phase, voice.lastPhase, waveIndex, pwValue,
                                 voice.blepRising, voice.blepFalling, voice.blepSaw,
                                 deltaPhaseL);
            float outR = genWave(voice.phaseR, voice.lastPhaseR, waveIndex, pwValue,
                                 voice.blepRisingR, voice.blepFallingR, voice.blepSawR,
                                 deltaPhaseR);

            outputs[VCO_L_OUTPUT].setVoltage(outL * 5.0f, c);
            outputs[VCO_R_OUTPUT].setVoltage(outR * 5.0f, c);

            if (c == 0) {
                dispFreq = freqL;
                dispPw = pwValue;
                dispWave = waveIndex;
                dispPhase = voice.phase;
                dispChaos = chaosDrift * chaosDepth;
                // peak-hold (signed) within the display frame -> spikier trace
                if (std::abs(dispChaos) > std::abs(driftPeak)) driftPeak = dispChaos;
            }
        }

        // LED: scaled up so it is clearly bright and visibly follows the drift
        float chaosIntensity = math::clamp(std::abs(chaosDrift) * chaosKnob * ST_CHAOS_LED_GAIN, 0.f, 1.f);
        lights[CHAOS_LIGHT].setSmoothBrightness(chaosIntensity, args.sampleTime);

        // Chaos history for the display (~60 points per second)
        driftTimer += args.sampleTime;
        if (driftTimer >= 1.f / 60.f) {
            driftTimer -= 1.f / 60.f;
            if (driftTimer > 1.f / 60.f) driftTimer = 0.f;
            driftHist[driftHead] = math::clamp(driftPeak * ST_CHAOS_DISP_GAIN, -1.f, 1.f);
            driftHead = (driftHead + 1) % ST_DRIFT_N;
            driftPeak = 0.f;
        }
    }
};

constexpr float Stagger::multTable[5];

namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }

void setFont(NVGcontext* vg) {
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font) nvgFontFaceId(vg, font->handle);
}

void drawText(NVGcontext* vg, float x, float y, const char* s, float size, NVGcolor col,
              int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f) {
    setFont(vg);
    nvgFontSize(vg, size);
    nvgTextLetterSpacing(vg, spacing);
    nvgTextAlign(vg, align);
    nvgFillColor(vg, col);
    nvgText(vg, x, y, s, NULL);
}

template <typename TBase = GrayModuleLightWidget>
struct TSTCyanLight : TBase {
    TSTCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
};
using STCyanLight = TSTCyanLight<>;

void drawLogo(NVGcontext* vg, float cx, float cy) {
    const float w = 64.f, h = 18.f;
    float x = cx - w / 2.f, y = cy - h / 2.f;
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 7.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h, nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
    nvgFill(vg);
    nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y, nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
    nvgStrokeWidth(vg, 1.2f);
    nvgStroke(vg);
    const float s = 2.2f;
    float tx = x + 7.f, ty = y + h / 2.f - 3.f;
    nvgBeginPath(vg);
    nvgRect(vg, tx + s, ty, s, s);
    nvgRect(vg, tx, ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);
    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

void drawPanelBase(NVGcontext* vg, float W, float H, const char* title) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H, colBgTop(), colBgBottom()));
    nvgFill(vg);
    nvgStrokeWidth(vg, 1.f);
    for (int y = 0; y < (int) H; y += 3) {
        nvgBeginPath(vg);
        nvgMoveTo(vg, 0, y + 0.5f);
        nvgLineTo(vg, W, y + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
        nvgStroke(vg);
    }
    nvgBeginPath(vg);
    nvgRect(vg, 0.5f, 0.5f, W - 1.f, H - 1.f);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
    nvgStroke(vg);
    drawLogo(vg, W / 2.f, 13.f);
    drawText(vg, W / 2.f, 31.f, title, 13.f, colText(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);
}

void drawBezel(NVGcontext* vg, float x, float y, float w, float h) {
    float bx = x - 5.f, by = y - 5.f, bw = w + 10.f, bh = h + 10.f;
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh, nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
    nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
    nvgFill(vg);
}

void plate(NVGcontext* vg, float x, float y, float w, float h) {
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 6.f);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

void ring(NVGcontext* vg, float cx, float cy, float r) {
    nvgBeginPath(vg);
    nvgCircle(vg, cx, cy, r);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 60));
    nvgFill(vg);
    nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
    nvgStrokeWidth(vg, 1.f);
    nvgStroke(vg);
}

void label(NVGcontext* vg, float x, float y, const char* t) {
    drawText(vg, x, y, t, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
}

void screenBase(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H);
    nvgFillPaint(vg, nvgRadialGradient(vg, W / 2.f, H / 2.f, 20.f, W * 0.75f, nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
    nvgFill(vg);
}

void screenFinish(NVGcontext* vg, float W, float H) {
    nvgBeginPath(vg);
    for (int y = 0; y < (int) H; y += 3) nvgRect(vg, 0, y + 1, W, 1);
    nvgFillColor(vg, nvgRGBA(0, 0, 0, 22));
    nvgFill(vg);
    nvgBeginPath(vg);
    nvgRect(vg, 0, 0, W, H * 0.3f);
    nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H * 0.3f, nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
    nvgFill(vg);
}

float stWave(float p, int wave, float pw) {
    switch (wave) {
        case 1:  return (p < 0.5f) ? (4.f * p - 1.f) : (3.f - 4.f * p);
        case 2:  return 1.f - 2.f * p;
        case 3:  return (p < pw) ? 1.f : -1.f;
        default: return std::sin(2.f * (float) M_PI * p);
    }
}

struct STPanel : TransparentWidget {
    STPanel() { box.size = Vec(ST_W, ST_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        drawPanelBase(vg, ST_W, ST_H, "STAGGER");
        drawBezel(vg, ST_DX, ST_DY, ST_DW, ST_DH);

        const char* a[4] = {"FREQ", "CHAOS", "WAVE", "PW"};
        for (int i = 0; i < 4; ++i) {
            float cx = stCol(i);
            plate(vg, cx - 33.f, ST_RA - 17.f, 66.f, 70.f);
            label(vg, cx, ST_RA + 21.f, a[i]);
            ring(vg, cx, ST_RA, 17.f);
            label(vg, cx - 16.f, ST_RA + 58.f, "CV");
            label(vg, cx + 16.f, ST_RA + 58.f, "ATT");
        }

        const char* b[4] = {"MULT", "RANGE", "RESET", "V/OCT"};
        for (int i = 0; i < 4; ++i) {
            float cx = stCol(i);
            plate(vg, cx - 33.f, ST_RB - 17.f, 66.f, 52.f);
            label(vg, cx, ST_RB + 25.f, b[i]);
        }
        ring(vg, stCol(0), ST_RB, 17.f);

        // row C: CHAOS light | OUT L | OUT R
        plate(vg, stCol(0) - 33.f, ST_IOY - 27.f, 66.f, 44.f);
        label(vg, stCol(0), ST_IOY - 19.f, "CHAOS");
        plate(vg, stCol(2) - 33.f, ST_IOY - 27.f, 66.f, 44.f);
        label(vg, stCol(2), ST_IOY - 19.f, "OUT L");
        plate(vg, stCol(3) - 33.f, ST_IOY - 27.f, 66.f, 44.f);
        label(vg, stCol(3), ST_IOY - 19.f, "OUT R");
        Widget::draw(args);
    }
};

struct STDisplay : TransparentWidget {
    Stagger* module = nullptr;
    STDisplay(Stagger* m) : module(m) {
        box.pos = Vec(ST_DX, ST_DY);
        box.size = Vec(ST_DW, ST_DH);
    }

    static constexpr float SX0 = 8.f, SY0 = 14.f, SW = 150.f, SH = 36.f;
    static constexpr float DX0 = 170.f, DY0 = 14.f, DW0 = 64.f, DH0 = 36.f;

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        screenBase(vg, ST_DW, ST_DH);
        const float xs[2] = {SX0, DX0}, ys[2] = {SY0, DY0}, ws[2] = {SW, DW0}, hs[2] = {SH, DH0};
        for (int k = 0; k < 2; ++k) {
            nvgBeginPath(vg);
            nvgRoundedRect(vg, xs[k] - 1.f, ys[k] - 1.f, ws[k] + 2.f, hs[k] + 2.f, 2.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 110));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, xs[k], ys[k] + hs[k] / 2.f + 0.5f);
            nvgLineTo(vg, xs[k] + ws[k], ys[k] + hs[k] / 2.f + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
            nvgStroke(vg);
        }
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        const float W = ST_DW, H = ST_DH;

        float freq = 261.6f, pw = 0.5f, phase = 0.3f, depth = 0.4f;
        int wave = 0, ch = 1;
        float hist[ST_DRIFT_N];
        if (module) {
            freq = module->dispFreq; pw = module->dispPw; phase = module->dispPhase;
            wave = module->dispWave; ch = module->dispChannels;
            depth = module->params[Stagger::CHAOS_PARAM].getValue();
            for (int i = 0; i < ST_DRIFT_N; ++i)
                hist[i] = module->driftHist[(module->driftHead + i) % ST_DRIFT_N];
        } else {
            // Demo trace: keep the same visual density for any ST_DRIFT_N
            for (int i = 0; i < ST_DRIFT_N; ++i) {
                float t = (float) i / (ST_DRIFT_N - 1);
                hist[i] = 0.45f * std::sin(t * 12.f) + 0.25f * std::sin(t * 34.f + 1.f);
            }
        }
        wave = std::min(std::max(wave, 0), 3);

        const float mid = SY0 + SH / 2.f, amp = SH / 2.f - 3.f;
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            const int N = 150;
            for (int i = 0; i <= N; ++i) {
                float t = 2.f * (float) i / N;
                float p = t - std::floor(t);
                float v = stWave(p, wave, pw);
                float x = SX0 + SW * (float) i / N, y = mid - v * amp;
                if (i == 0) nvgMoveTo(vg, x, y);
                else nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 50) : nvgRGB(0x8a, 0xee, 0xff));
            nvgStrokeWidth(vg, pass == 0 ? 4.f : 1.3f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
        }
        {
            float x = SX0 + SW * 0.5f * phase, y = mid - stWave(phase, wave, pw) * amp;
            nvgBeginPath(vg);
            nvgMoveTo(vg, x, SY0);
            nvgLineTo(vg, x, SY0 + SH);
            nvgStrokeColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, 4.f);
            nvgFillColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 70));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, x, y, 1.8f);
            nvgFillColor(vg, nvgRGB(0xf0, 0xe0, 0xff));
            nvgFill(vg);
        }

        nvgSave(vg);
        nvgScissor(vg, DX0, DY0, DW0, DH0);
        const float dm = DY0 + DH0 / 2.f, da = DH0 / 2.f - 2.f;
        // 128 history points must span the 64 px wide field
        const float xStep = (DW0 - 1.f) / (ST_DRIFT_N - 1);
        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i < ST_DRIFT_N; ++i) {
                float v = std::min(std::max(hist[i], -1.f), 1.f);
                float x = DX0 + (float) i * xStep, y = dm - v * da;
                if (i == 0) nvgMoveTo(vg, x, y);
                else nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0xb0, 0x5a, 0xff, 60) : nvgRGB(0xd8, 0xb0, 0xff));
            nvgStrokeWidth(vg, pass == 0 ? 3.5f : 1.2f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
        }
        nvgResetScissor(vg);
        nvgRestore(vg);

        screenFinish(vg, W, H);

        static const char* wn[4] = {"SINE", "TRI", "SAW", "SQR"};
        char buf[48];
        snprintf(buf, sizeof(buf), "%.1fHZ", freq);
        drawText(vg, 8.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 210), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "%s%s", wn[wave], ch > 1 ? "  POLY" : "");
        drawText(vg, W - 8.f, 7.f, buf, 7.f, nvgRGBA(0xff, 0xe6, 0x80, 220), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        if (wave == 3) snprintf(buf, sizeof(buf), "PW %.0f%%", pw * 100.f);
        else snprintf(buf, sizeof(buf), "1 CYCLE x2");
        drawText(vg, 8.f, H - 6.f, buf, 6.5f, colDim(), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
        snprintf(buf, sizeof(buf), "CHAOS %.0f%%", depth * 100.f);
        drawText(vg, W - 8.f, H - 6.f, buf, 6.5f, colDim(), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 0.8f);
        Widget::drawLayer(args, layer);
    }
};

} // namespace

struct StaggerWidget : ModuleWidget {
    StaggerWidget(Stagger* module) {
        setModule(module);
        box.size = Vec(ST_W, ST_H);

        addChild(new STPanel());
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new STDisplay(module));

        const int knob[4] = {Stagger::PITCH_PARAM, Stagger::CHAOS_PARAM, Stagger::WAVE_PARAM, Stagger::PW_PARAM};
        const int att[4]  = {Stagger::PITCH_ATTEN_PARAM, Stagger::CHAOS_ATTEN_PARAM, Stagger::WAVE_ATTEN_PARAM, Stagger::PW_ATTEN_PARAM};
        const int cv[4]   = {Stagger::PITCH_CV_INPUT, Stagger::CHAOS_CV_INPUT, Stagger::WAVE_CV_INPUT, Stagger::PW_CV_INPUT};
        for (int i = 0; i < 4; ++i) {
            float cx = stCol(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, ST_RA), module, knob[i]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 16.f, ST_RA + 40.f), module, cv[i]));
            addParam(createParamCentered<Trimpot>(Vec(cx + 16.f, ST_RA + 40.f), module, att[i]));
        }

        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(stCol(0), ST_RB), module, Stagger::CHAOS_MULT_PARAM));
        addParam(createParamCentered<CKSS>(Vec(stCol(1), ST_RB), module, Stagger::CV_RANGE_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(stCol(2), ST_RB), module, Stagger::CHAOS_RESET_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(stCol(3), ST_RB), module, Stagger::CV_INPUT));

        // Row C: chaos LED + L/R outputs
        addChild(createLightCentered<SmallLight<STCyanLight>>(Vec(stCol(0), ST_IOY), module, Stagger::CHAOS_LIGHT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(stCol(2), ST_IOY), module, Stagger::VCO_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(stCol(3), ST_IOY), module, Stagger::VCO_R_OUTPUT));
    }
};

Model* modelStagger = createModel<Stagger, StaggerWidget>("Stagger");