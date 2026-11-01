#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   DR_SCOPE_N = 55;
static const float DR_PANEL_W = RACK_GRID_WIDTH * 20;   // 300
static const float DR_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float DR_DISP_X = 80.f;
static const float DR_DISP_Y = 54.f;
static const float DR_DISP_W = 124.f;
static const float DR_DISP_H = 56.f;

static const float DR_ROW[3]  = {184.f, 248.f, 312.f};   // knob centre of each oscillator row
static const float DR_COLX[6] = {68.f, 108.f, 148.f, 188.f, 228.f, 268.f}; // FREQ DRIFT MIX FILTER SEND ENV
static const float DR_WAVE_X  = 15.f;                    // waveform switch x
static const float DR_CV_DY   = 28.f;                    // CV jack offset below knob

// ------------------------------------------------------------
// Multi-tap delay for dense reverb
// ------------------------------------------------------------
struct DroneDelay
{
	float buffer[44100] = {0.f};
	int writePos = 0;

	float process(float input, float feedback, int length)
	{
		length = clamp(length, 1, 44099);
		int readPos = (writePos - length + 44100) % 44100;
		float output = buffer[readPos];
		buffer[writePos] = input + output * feedback;
		writePos = (writePos + 1) % 44100;
		return output;
	}
};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct InstantDrone : Module
{
	enum ParamId
	{
		// Oscillator parameters
		FREQ1_PARAM,
		FREQ2_PARAM,
		FREQ3_PARAM,
		WAVE1_PARAM,
		WAVE2_PARAM,
		WAVE3_PARAM,
		MIX1_PARAM,
		MIX2_PARAM,
		MIX3_PARAM,
		DRIFT1_PARAM,
		DRIFT2_PARAM,
		DRIFT3_PARAM,
		SEND1_PARAM,
		SEND2_PARAM,
		SEND3_PARAM,
		FILT1_PARAM,
		FILT2_PARAM,
		FILT3_PARAM,
		REVERB_WET_PARAM,
		OUTPUT_GAIN_PARAM,
		// Envelope bypass switches (0=off, 1=on)
		ENV1_SWITCH,
		ENV2_SWITCH,
		ENV3_SWITCH,
		PARAMS_LEN
	};
	enum InputId
	{
		VOCT1_INPUT,
		VOCT2_INPUT,
		VOCT3_INPUT,
		DRIFT1_CV,
		DRIFT2_CV,
		DRIFT3_CV,
		MIX1_CV,
		MIX2_CV,
		MIX3_CV,
		FILT1_CV,
		FILT2_CV,
		FILT3_CV,
		SEND1_CV,
		SEND2_CV,
		SEND3_CV,
		ENV1_INPUT,
		ENV2_INPUT,
		ENV3_INPUT,
		INPUTS_LEN
	};
	enum OutputId
	{
		LEFT_OUTPUT,
		RIGHT_OUTPUT,
		OUTPUTS_LEN
	};

	// Oscillator state
	float phases[3] = {0.f, 0.f, 0.f};
	float driftValues[3] = {0.f, 0.f, 0.f};
	float driftTargets[3] = {0.f, 0.f, 0.f};

	// Per-oscillator lowpass filters
	float lowpass1 = 0.f, lowpass2 = 0.f, lowpass3 = 0.f;

	DroneDelay delayL[4];
	DroneDelay delayR[4];

	// Display data (written by the audio thread, read by the UI thread;
	// a torn read is only cosmetic)
	float scopeMin[DR_SCOPE_N] = {};
	float scopeMax[DR_SCOPE_N] = {};
	int scopeHead = 0;
	int scopeCount = 0;
	float scopeBMin = 0.f, scopeBMax = 0.f;
	float peakL = 0.f, peakR = 0.f;

	InstantDrone()
	{
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN);

		for (int i = 0; i < 3; i++)
		{
			configParam(FREQ1_PARAM + i, -3.f, 3.f, 0.f, string::f("Osc %d frequency", i + 1), " oct");
			configSwitch(WAVE1_PARAM + i, 0.f, 2.f, 0.f, string::f("Osc %d waveform", i + 1),
						 {"Sine", "Triangle", "Saw"});
			configParam(MIX1_PARAM + i, 0.f, 1.f, 0.5f, string::f("Osc %d mix", i + 1), "%", 0.f, 100.f);
			configParam(DRIFT1_PARAM + i, 0.f, 1.f, 0.2f, string::f("Osc %d drift", i + 1), "%", 0.f, 100.f);
			configParam(SEND1_PARAM + i, 0.f, 1.f, 0.5f, string::f("Osc %d reverb send", i + 1), "%", 0.f, 100.f);
			configParam(FILT1_PARAM + i, 0.f, 1.f, 0.6f, string::f("Osc %d filter cutoff", i + 1), "%", 0.f, 100.f);

			configInput(VOCT1_INPUT + i, string::f("Osc %d V/Oct", i + 1));
			configInput(DRIFT1_CV + i, string::f("Osc %d drift CV", i + 1));
			configInput(MIX1_CV + i, string::f("Osc %d mix CV", i + 1));
			configInput(FILT1_CV + i, string::f("Osc %d filter CV", i + 1));
			configInput(SEND1_CV + i, string::f("Osc %d send CV", i + 1));
			configInput(ENV1_INPUT + i, string::f("Osc %d envelope", i + 1));
		}
		configParam(REVERB_WET_PARAM, 0.f, 1.f, 0.7f, "Drone intensity", "%", 0.f, 100.f);
		configParam(OUTPUT_GAIN_PARAM, -10.f, 10.f, 0.f, "Output gain", " dB");

		// Envelope bypass switches (0=off / bypassed, 1=on / active)
		configSwitch(ENV1_SWITCH, 0.f, 1.f, 0.f, "Osc 1 envelope bypass", {"Off (bypassed)", "On (active)"});
		configSwitch(ENV2_SWITCH, 0.f, 1.f, 0.f, "Osc 2 envelope bypass", {"Off (bypassed)", "On (active)"});
		configSwitch(ENV3_SWITCH, 0.f, 1.f, 0.f, "Osc 3 envelope bypass", {"Off (bypassed)", "On (active)"});

		configOutput(LEFT_OUTPUT, "Left");
		configOutput(RIGHT_OUTPUT, "Right");
	}

	float getWaveform(float phase, int type)
	{
		switch (type)
		{
		case 0:
			return std::sin(2.f * M_PI * phase);
		case 1:
			return 1.f - 4.f * std::fabs(phase - 0.5f);
		case 2:
			return 2.f * phase - 1.f;
		default:
			return std::sin(2.f * M_PI * phase);
		}
	}

	void process(const ProcessArgs &args) override
	{
		float drySum = 0.f;
		float revInput = 0.f;

		for (int i = 0; i < 3; i++)
		{
			// ---- Drift random walk ----
			if (random::uniform() < 0.002f)
			{
				driftTargets[i] = (random::uniform() - 0.5f) * 2.0f;
			}
			const float driftSlew = 0.01f;
			driftValues[i] += (driftTargets[i] - driftValues[i]) * driftSlew;

			// ---- Pitch (V/oct + knob) ----
			float pitch = params[FREQ1_PARAM + i].getValue() + inputs[VOCT1_INPUT + i].getNormalVoltage(0.f);

			// ---- Drift amount (knob + CV, 0..1) ----
			float driftKnob = params[DRIFT1_PARAM + i].getValue();
			float driftCV = inputs[DRIFT1_CV + i].getNormalVoltage(0.f) / 5.f;
			float driftAmount = clamp(driftKnob + driftCV, 0.f, 1.f);
			pitch += driftValues[i] * driftAmount;

			// ---- Frequency & phase ----
			float freq = 261.626f * std::pow(2.f, pitch);
			phases[i] += freq * args.sampleTime;
			if (phases[i] >= 1.f)
				phases[i] -= 1.f;

			// ---- Waveform ----
			int waveType = (int)params[WAVE1_PARAM + i].getValue();
			float rawOsc = getWaveform(phases[i], waveType);

			// ---- Base gain (mix knob + CV) ----
			float mixKnob = params[MIX1_PARAM + i].getValue();
			float mixCV = inputs[MIX1_CV + i].getNormalVoltage(0.f) / 5.f;
			float baseGain = clamp(mixKnob + mixCV, 0.f, 1.f);

			// ---- Envelope handling (bypass switch) ----
			float envelope = 1.f; // default = no envelope (full gain)
			bool envelopeActive = params[ENV1_SWITCH + i].getValue() > 0.5f;
			if (envelopeActive)
			{
				// Envelope input: 0-10V -> 0-1 multiplier
				envelope = inputs[ENV1_INPUT + i].getNormalVoltage(0.f) / 10.f;
				envelope = clamp(envelope, 0.f, 1.f);
			}
			// Final amplitude: raw * 5V * baseGain * envelope (or 1 if bypassed)
			float oscillator = rawOsc * 5.0f * baseGain * envelope;

			// ---- Per-oscillator low-pass filter ----
			float filtKnob = params[FILT1_PARAM + i].getValue();
			float filtCV = inputs[FILT1_CV + i].getNormalVoltage(0.f) / 5.f;
			float cutoff = clamp(filtKnob + filtCV, 0.01f, 0.99f);
			float f = cutoff * 0.15f;
			float filtered;
			switch (i)
			{
			case 0:
				lowpass1 += f * (oscillator - lowpass1);
				filtered = lowpass1;
				break;
			case 1:
				lowpass2 += f * (oscillator - lowpass2);
				filtered = lowpass2;
				break;
			default:
				lowpass3 += f * (oscillator - lowpass3);
				filtered = lowpass3;
				break;
			}

			// ---- Dry sum & reverb send ----
			drySum += filtered;
			float sendKnob = params[SEND1_PARAM + i].getValue();
			float sendCV = inputs[SEND1_CV + i].getNormalVoltage(0.f) / 5.f;
			float send = clamp(sendKnob + sendCV, 0.f, 1.f);
			revInput += filtered * send;
		}

		// ---- Multi-tap drone reverb ----
		float wetL = 0.f, wetR = 0.f;
		float wetAmount = params[REVERB_WET_PARAM].getValue();
		const float fb = 0.985f;
		int timesL[] = {3431, 5743, 7853, 9437};
		int timesR[] = {3721, 5923, 8147, 9601};

		for (int i = 0; i < 4; i++)
		{
			wetL += delayL[i].process(revInput, fb, timesL[i]);
			wetR += delayR[i].process(revInput, fb, timesR[i]);
		}
		wetL /= 4.0f;
		wetR /= 4.0f;

		float outL = drySum + wetL * wetAmount;
		float outR = drySum + wetR * wetAmount;

		// ---- Output gain ----
		float gainDb = params[OUTPUT_GAIN_PARAM].getValue();
		float gainLin = std::pow(10.f, gainDb / 20.f);
		outL *= gainLin;
		outR *= gainLin;

		outputs[LEFT_OUTPUT].setVoltage(outL);
		outputs[RIGHT_OUTPUT].setVoltage(outR);

		// ---- display data ----
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
		int bucket = std::max(1, (int)(args.sampleRate * 0.008f));
		if (++scopeCount >= bucket)
		{
			scopeCount = 0;
			scopeMin[scopeHead] = scopeBMin;
			scopeMax[scopeHead] = scopeBMax;
			scopeHead = (scopeHead + 1) % DR_SCOPE_N;
		}
	}
};

// Everything below lives in an anonymous namespace so the names cannot clash
// with the same-named helpers in the other module files.
namespace {

// ------------------------------------------------------------
// Theme (same palette as the other AGMM modules)
// ------------------------------------------------------------
NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }

void setFont(NVGcontext* vg) {
    std::shared_ptr<window::Font> font =
        APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
    if (font)
        nvgFontFaceId(vg, font->handle);
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

// ------------------------------------------------------------
// AGMM logo
// ------------------------------------------------------------
void drawLogo(NVGcontext* vg, float cx, float cy) {
    const float w = 64.f, h = 18.f;
    float x = cx - w / 2.f, y = cy - h / 2.f;

    // Glow
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
    nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
    nvgFill(vg);

    // Pill body
    nvgBeginPath(vg);
    nvgRoundedRect(vg, x, y, w, h, 7.f);
    nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
                                       nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
    nvgFill(vg);
    nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
                                         nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
    nvgStrokeWidth(vg, 1.2f);
    nvgStroke(vg);

    // Tiny cell triangle
    const float s = 2.2f;
    float tx = x + 7.f, ty = y + h / 2.f - 3.f;
    nvgBeginPath(vg);
    nvgRect(vg, tx + s,       ty,            s, s);
    nvgRect(vg, tx,           ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);

    // Wordmark
    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
             NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

// Same shapes as InstantDrone::getWaveform
float dWave(float phase, int type) {
    switch (type) {
        case 1:  return 1.f - 4.f * std::fabs(phase - 0.5f);
        case 2:  return 2.f * phase - 1.f;
        default: return std::sin(2.f * (float) M_PI * phase);
    }
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct DRPanelBackground : TransparentWidget {
    DRPanelBackground() { box.size = Vec(DR_PANEL_W, DR_PANEL_H); }

    void plate(NVGcontext* vg, float x, float y, float w, float h) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 6.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
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

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = DR_PANEL_W - 14.f, cx = DR_PANEL_W / 2.f;
        float w = 10.f + (float) strlen(txt) * 7.f;
        drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - w / 2.f, y);
        nvgMoveTo(vg, cx + w / 2.f, y);
        nvgLineTo(vg, x2, y);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 70));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        // Base gradient
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DR_PANEL_W, DR_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DR_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        // Brushed texture
        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) DR_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, DR_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        // Edge highlight
        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, DR_PANEL_W - 1.f, DR_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        // Logo + title
        drawLogo(vg, DR_PANEL_W / 2.f, 13.f);
        drawText(vg, DR_PANEL_W / 2.f, 31.f, "INSTANTDRONE", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = DR_DISP_X - 5.f, by = DR_DISP_Y - 5.f;
        float bw = DR_DISP_W + 10.f, bh = DR_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // Intensity + gain
        ring(vg, 40.f, 80.f, 27.f);
        drawText(vg, 40.f, 114.f, "INTENSITY", 8.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        ring(vg, 254.f, 62.f, 20.f);
        drawText(vg, 254.f, 87.f, "GAIN", 8.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        // Output plates
        const char* outs[2] = {"L OUT", "R OUT"};
        const float outX[2] = {236.f, 272.f};
        for (int i = 0; i < 2; ++i) {
            plate(vg, outX[i] - 17.f, 92.f, 34.f, 38.f);
            drawText(vg, outX[i], 99.f, outs[i], 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // Oscillator section
        section(vg, 140.f, "OSCILLATORS");
        drawText(vg, 25.f, 154.f, "WAVE", 8.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        const char* heads[6] = {"FREQ", "DRIFT", "MIX", "FILTER", "SEND", "ENV"};
        for (int c = 0; c < 6; ++c)
            drawText(vg, DR_COLX[c], 154.f, heads[c], 8.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        for (int r = 0; r < 3; ++r) {
            float ry = DR_ROW[r];
            plate(vg, 4.f, ry - 20.f, 42.f, 62.f);
            char buf[8];
            snprintf(buf, sizeof(buf), "VCO%d", r + 1);
            drawText(vg, DR_WAVE_X, ry + 17.f, buf, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            for (int c = 0; c < 6; ++c)
                plate(vg, DR_COLX[c] - 18.f, ry - 20.f, 36.f, 62.f);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Output scope + level meters
// ------------------------------------------------------------
struct DRDisplayWidget : TransparentWidget {
    InstantDrone* module = nullptr;

    DRDisplayWidget(InstantDrone* m) : module(m) {
        box.pos = Vec(DR_DISP_X, DR_DISP_Y);
        box.size = Vec(DR_DISP_W, DR_DISP_H);
    }

    static float scopeW() { return DR_DISP_W - 14.f; }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DR_DISP_W, DR_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, DR_DISP_W / 2.f, DR_DISP_H / 2.f, 20.f, DR_DISP_W * 0.7f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        // Faint grid
        nvgBeginPath(vg);
        for (float x = 0.f; x <= scopeW(); x += 22.f) {
            nvgMoveTo(vg, x + 0.5f, 0);
            nvgLineTo(vg, x + 0.5f, DR_DISP_H);
        }
        nvgMoveTo(vg, 0, DR_DISP_H / 2.f + 0.5f);
        nvgLineTo(vg, scopeW(), DR_DISP_H / 2.f + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 16));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        const float mid = DR_DISP_H / 2.f;
        const float amp = mid - 2.f;       // pixels for 10 V
        const float sw = scopeW();
        const float colW = sw / (float) DR_SCOPE_N;

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, DR_DISP_H);
        for (int pass = 0; pass < 2; ++pass) {   // 0: glow, 1: crisp
            nvgBeginPath(vg);
            for (int i = 0; i < DR_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % DR_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / DR_SCOPE_N;
                    float e = 6.f * (0.55f + 0.45f * std::sin(t * 11.f)) * (0.5f + 0.5f * t);
                    vmax = e;
                    vmin = -e * 0.9f;
                }
                float y0 = mid - std::min(std::max(vmax / 10.f, -1.f), 1.f) * amp;
                float y1 = mid - std::min(std::max(vmin / 10.f, -1.f), 1.f) * amp;
                float h = std::max(y1 - y0, 1.f);
                float x = i * colW;
                if (pass == 0)
                    nvgRect(vg, x - 0.5f, y0 - 1.5f, colW + 1.f, h + 3.f);
                else
                    nvgRect(vg, x, y0, std::max(colW - 0.4f, 1.f), h);
            }
            nvgFillColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 32) : nvgRGB(0x8a, 0xee, 0xff));
            nvgFill(vg);
        }
        nvgResetScissor(vg);
        nvgRestore(vg);

        // Level meters (L / R)
        for (int ch = 0; ch < 2; ++ch) {
            float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 5.f : 4.2f);
            float lvl = std::min(peak / 10.f, 1.f);
            float x = sw + 2.f + ch * 5.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 1.f, 3.f, DR_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (DR_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, DR_DISP_H - 1.f - h, 3.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, DR_DISP_H, 0, 0,
                                                   nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) DR_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, DR_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, DR_DISP_W, DR_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, DR_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[32];
        float inten = module ? module->params[InstantDrone::REVERB_WET_PARAM].getValue() : 0.7f;
        float gain  = module ? module->params[InstantDrone::OUTPUT_GAIN_PARAM].getValue() : 0.f;
        snprintf(buf, sizeof(buf), "INT %3.0f%%", inten * 100.f);
        drawText(vg, 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "GAIN %+.1fDB", gain);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, nvgRGBA(0x3d, 0xe0, 0xff, 190),
                 NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// Mini waveform screen (one per oscillator): shows the selected shape
// with a dot at the oscillator's live phase
// ------------------------------------------------------------
struct DRWaveWidget : TransparentWidget {
    InstantDrone* module = nullptr;
    int idx = 0;
    static constexpr float SH = 24.f;   // screen height

    DRWaveWidget(InstantDrone* m, int i, float x, float y) : module(m), idx(i) {
        box.pos = Vec(x, y);
        box.size = Vec(20.f, 36.f);
    }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0, 0, box.size.x, SH, 3.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;

        int type = module ? (int) module->params[InstantDrone::WAVE1_PARAM + idx].getValue() : idx;
        type = std::min(std::max(type, 0), 2);
        float phase = module ? module->phases[idx] : 0.3f;

        const float w = box.size.x - 4.f;
        const float mid = SH / 2.f;
        const float amp = SH / 2.f - 4.f;

        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            for (int i = 0; i <= 20; ++i) {
                float p = (float) i / 20.f;
                float x = 2.f + p * w;
                float y = mid - dWave(p, type) * amp;
                if (i == 0) nvgMoveTo(vg, x, y);
                else nvgLineTo(vg, x, y);
            }
            if (pass == 0) {
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 50));
                nvgStrokeWidth(vg, 3.f);
            } else {
                nvgStrokeColor(vg, nvgRGB(0x8a, 0xee, 0xff));
                nvgStrokeWidth(vg, 1.f);
            }
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
        }

        // Live phase dot
        float dx = 2.f + phase * w;
        float dy = mid - dWave(phase, type) * amp;
        nvgBeginPath(vg);
        nvgCircle(vg, dx, dy, 3.2f);
        nvgFillColor(vg, nvgRGBA(0xb0, 0x5a, 0xff, 70));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgCircle(vg, dx, dy, 1.5f);
        nvgFillColor(vg, nvgRGB(0xf0, 0xe0, 0xff));
        nvgFill(vg);

        static const char* names[3] = {"SIN", "TRI", "SAW"};
        drawText(vg, box.size.x / 2.f, SH + 9.f, names[type], 7.f, colAccent(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// Envelope-active LED
// ------------------------------------------------------------
struct DREnvLed : TransparentWidget {
    InstantDrone* module = nullptr;
    int idx = 0;

    DREnvLed(InstantDrone* m, int i, float cx, float cy) : module(m), idx(i) {
        box.pos = Vec(cx - 4.f, cy - 4.f);
        box.size = Vec(8.f, 8.f);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        bool on = module ? (module->params[InstantDrone::ENV1_SWITCH + idx].getValue() > 0.5f) : (idx == 1);

        if (on) {
            nvgBeginPath(vg);
            nvgCircle(vg, 4.f, 4.f, 4.f);
            nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgCircle(vg, 4.f, 4.f, 2.f);
            nvgFillColor(vg, nvgRGB(0x8a, 0xee, 0xff));
            nvgFill(vg);
        } else {
            nvgBeginPath(vg);
            nvgCircle(vg, 4.f, 4.f, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 28));
            nvgFill(vg);
        }
        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct InstantDroneWidget : ModuleWidget
{
	InstantDroneWidget(InstantDrone *module)
	{
		setModule(module);
		box.size = Vec(DR_PANEL_W, DR_PANEL_H);

		addChild(new DRPanelBackground());

		// Screws
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

		// Displays (added before the controls so the controls stay on top)
		addChild(new DRDisplayWidget(module));
		for (int r = 0; r < 3; ++r)
		{
			addChild(new DRWaveWidget(module, r, 25.f, DR_ROW[r] - 16.f));
			addChild(new DREnvLed(module, r, DR_COLX[5] + 12.f, DR_ROW[r] - 14.f));
		}

		// Master section
		addParam(createParamCentered<RoundLargeBlackKnob>(Vec(40.f, 80.f), module, InstantDrone::REVERB_WET_PARAM));
		addParam(createParamCentered<RoundSmallBlackKnob>(Vec(254.f, 62.f), module, InstantDrone::OUTPUT_GAIN_PARAM));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(236.f, 117.f), module, InstantDrone::LEFT_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(272.f, 117.f), module, InstantDrone::RIGHT_OUTPUT));

		// Oscillator rows
		for (int i = 0; i < 3; ++i)
		{
			float ry = DR_ROW[i];
			float jy = ry + DR_CV_DY;

			// Waveform switch
			addParam(createParamCentered<CKSSThree>(Vec(DR_WAVE_X, ry), module, InstantDrone::WAVE1_PARAM + i));

			// Freq (V/Oct), Drift, Mix, Filter, Send: knob on top, CV jack below
			addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[0], ry), module, InstantDrone::FREQ1_PARAM + i));
			addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[0], jy), module, InstantDrone::VOCT1_INPUT + i));

			addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[1], ry), module, InstantDrone::DRIFT1_PARAM + i));
			addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[1], jy), module, InstantDrone::DRIFT1_CV + i));

			addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[2], ry), module, InstantDrone::MIX1_PARAM + i));
			addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[2], jy), module, InstantDrone::MIX1_CV + i));

			addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[3], ry), module, InstantDrone::FILT1_PARAM + i));
			addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[3], jy), module, InstantDrone::FILT1_CV + i));

			addParam(createParamCentered<RoundSmallBlackKnob>(Vec(DR_COLX[4], ry), module, InstantDrone::SEND1_PARAM + i));
			addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[4], jy), module, InstantDrone::SEND1_CV + i));

			// Envelope bypass switch + envelope input
			addParam(createParamCentered<CKSS>(Vec(DR_COLX[5], ry - 3.f), module, InstantDrone::ENV1_SWITCH + i));
			addInput(createInputCentered<DarkPJ301MPort>(Vec(DR_COLX[5], jy), module, InstantDrone::ENV1_INPUT + i));
		}
	}
};

Model *modelInstantDrone = createModel<InstantDrone, InstantDroneWidget>("InstantDrone");