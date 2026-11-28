// Crypto.cpp - frequency-code granular processor for VCV Rack (AGMM panel kit, 54 HP).
//
// Concept
// -------
//   * Every audio sample of IN L / IN R becomes a 16-bit code that is mapped exponentially onto a
//     frequency between LOW and HIGH:  f = LOW * (HIGH / LOW) ^ (code / 65535).
//   * CLK defines a tick. The distance between two ticks is N samples (BPM = 60 * sr / N). SAMPLES
//     picks the captured window: N, N/2, N/3, N/4, N/8, N/16 or N/32 (capture starts on the tick, so
//     the window always fits inside it). When it closes, the frequency of every captured sample and
//     the MEAN frequency per channel are shown on the display (BIN / OCT / DEC / HEX).
//   * Three engines (MODE):
//       TONE   - an oscillator per channel at the mean frequency.
//       CRYPTO - grains whose pitch is drawn from the captured frequency codes.
//       CLOUD  - a Clouds / Supercell style granulator running on the recorded input audio.
//   * Supercell style processor: POSITION, PITCH, SHAPE, MIX, SPACE (reverb), FEEDBACK, GAIN IN and
//     GAIN OUT, each of the first six with a CV input. V/OCT transposes the pitch.
//   * The screen shows NUMBERS, a SPECTROGRAM or the WAVEform of the output.
//   * With no CLK patched the module free-runs on an internal clock (right-click menu).
//
// Registration (plugin.hpp / plugin.cpp / plugin.json):
//     extern Model *modelCrypto;           p->addModel(modelCrypto);
//     { "slug": "Crypto", "name": "Crypto", "tags": [...] }

#include "plugin.hpp"
#include "agmm_ui.hpp"            

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace crypto
{

	// ------------------------------------------------------------------------------------
	// Constants / helpers
	// ------------------------------------------------------------------------------------
	constexpr float PI_F = 3.14159265358979f;
	constexpr int MAXV = 32;   // frequency codes kept per channel per tick (for the display)
	constexpr int LOG_N = 64;  // tick records kept
	constexpr int GRAINS = 48; // grain voices
	constexpr int DIV_COUNT = 7;
	constexpr size_t BUF_N = 1u << 18; // recorded input history (~5.4 s at 48 kHz)
	constexpr size_t BUF_MASK = BUF_N - 1;
	constexpr int VIZ_N = 4096; // output history for the wave / spectrogram views
	static const int DIVS[DIV_COUNT] = {1, 2, 3, 4, 8, 16, 32};
	static const char *DIV_NAMES[DIV_COUNT] = {"N", "N/2", "N/3", "N/4", "N/8", "N/16", "N/32"};
	static const int DIGITS[4] = {16, 6, 5, 4}; // digits per value for BIN / OCT / DEC / HEX

	template <class T>
	inline T clp(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }

	struct Rng
	{
		uint64_t s;
		explicit Rng(uint64_t seed = 0x9E3779B97F4A7C15ull) : s(seed ? seed : 1) {}
		float uniform()
		{
			s ^= s << 13;
			s ^= s >> 7;
			s ^= s << 17;
			return (float)((s >> 40) & 0xFFFFFF) / 16777216.f;
		}
	};

	// ---- knob table (single source of truth for mapping, tooltips and panel text) ----
	enum KnobIdx
	{
		K_LOW,
		K_HIGH,
		K_GLIDE,
		K_SIZE,
		K_DENS,
		K_GLITCH,
		K_SPREAD,
		K_POS,
		K_PITCH,
		K_SHAPE,
		K_MIX,
		K_SPACE,
		K_FB,
		K_GIN,
		K_GOUT,
		K_COUNT
	};

	struct KnobDef
	{
		const char *label;
		float lo, hi;
		bool exp;
		const char *unit;
		float def; // default knob position 0..1
	};

	static const KnobDef KNOBS[K_COUNT] = {
		{"LOW", 20.f, 2000.f, true, "Hz", 0.35f},
		{"HIGH", 200.f, 20000.f, true, "Hz", 0.75f},
		{"GLIDE", 0.f, 500.f, false, "ms", 0.10f},
		{"SIZE", 5.f, 500.f, true, "ms", 0.45f},
		{"DENSITY", 1.f, 120.f, true, "/s", 0.55f},
		{"GLITCH", 0.f, 100.f, false, "%", 0.15f},
		{"SPREAD", 0.f, 100.f, false, "%", 0.20f},
		{"POSITION", 0.f, 100.f, false, "%", 0.30f},
		{"PITCH", -24.f, 24.f, false, "st", 0.50f},
		{"SHAPE", 0.f, 100.f, false, "%", 0.35f},
		{"MIX", 0.f, 100.f, false, "%", 0.75f},
		{"SPACE", 0.f, 100.f, false, "%", 0.25f},
		{"FEEDBACK", 0.f, 100.f, false, "%", 0.00f},
		{"GAIN IN", 0.f, 200.f, false, "%", 0.50f},
		{"GAIN OUT", 0.f, 200.f, false, "%", 0.50f},
	};

	inline float mapKnob(const KnobDef &d, float x)
	{
		x = clp(x, 0.f, 1.f);
		return d.exp ? d.lo * std::pow(d.hi / d.lo, x) : d.lo + x * (d.hi - d.lo);
	}

	inline std::string fmtKnob(const KnobDef &d, float v)
	{
		char b[48];
		if (d.lo < 0.f)
			std::snprintf(b, sizeof b, "%+.1f", v);
		else if (std::fabs(v) >= 1000.f)
			std::snprintf(b, sizeof b, "%.1fk", v / 1000.f);
		else if (std::fabs(v) >= 100.f)
			std::snprintf(b, sizeof b, "%.0f", v);
		else if (std::fabs(v) >= 10.f)
			std::snprintf(b, sizeof b, "%.1f", v);
		else
			std::snprintf(b, sizeof b, "%.2f", v);
		std::string s = b;
		if (d.unit[0])
		{
			s += " ";
			s += d.unit;
		}
		return s;
	}

	// Fixed-width number in the selected base (0 BIN, 1 OCT, 2 DEC, 3 HEX).
	inline std::string fmtNum(uint32_t v, int base)
	{
		v = std::min<uint32_t>(v, 65535u);
		char b[40];
		switch (base)
		{
		case 0:
			for (int i = 0; i < 16; i++)
				b[i] = ((v >> (15 - i)) & 1u) ? '1' : '0';
			b[16] = 0;
			break;
		case 1:
			std::snprintf(b, sizeof b, "%06o", (unsigned)v);
			break;
		case 2:
			std::snprintf(b, sizeof b, "%05u", (unsigned)v);
			break;
		default:
			std::snprintf(b, sizeof b, "%04X", (unsigned)v);
			break;
		}
		return b;
	}

	// ---- oscillator ----
	inline float polyBlep(float t, float dt)
	{
		if (t < dt)
		{
			t /= dt;
			return t + t - t * t - 1.f;
		}
		if (t > 1.f - dt)
		{
			t = (t - 1.f) / dt;
			return t * t + t + t + 1.f;
		}
		return 0.f;
	}

	// wave: 0 sine, 1 triangle, 2 saw (band-limited), 3 square (band-limited)
	inline float oscSample(int wave, float ph, float dt)
	{
		switch (wave)
		{
		case 0:
			return std::sin(2.f * PI_F * ph);
		case 1:
			return 4.f * std::fabs(ph - 0.5f) - 1.f;
		case 2:
			return 2.f * ph - 1.f - polyBlep(ph, dt);
		default:
		{
			float s = ph < 0.5f ? 1.f : -1.f;
			s += polyBlep(ph, dt);
			float p2 = ph + 0.5f;
			if (p2 >= 1.f)
				p2 -= 1.f;
			s -= polyBlep(p2, dt);
			return s;
		}
		}
	}

	// Grain window. shape 0 = Hann, 0.5 = flat top with short ramps, 1 = percussive (fast attack,
	// long exponential decay).
	inline float grainEnv(float t, float shape)
	{
		if (shape <= 0.5f)
		{
			float r = clp(0.5f - shape * 0.96f, 0.02f, 0.5f);
			if (t < r)
				return 0.5f * (1.f - std::cos(PI_F * t / r));
			if (t > 1.f - r)
				return 0.5f * (1.f - std::cos(PI_F * (1.f - t) / r));
			return 1.f;
		}
		const float a = 0.02f;
		float k = (shape - 0.5f) * 14.f;
		float in = t < a ? 0.5f * (1.f - std::cos(PI_F * t / a)) : 1.f;
		return in * std::exp(-k * t);
	}

	// ---- small stereo Freeverb-style reverb ----
	struct Reverb
	{
		static constexpr int NC = 4, NA = 2;
		struct Line
		{
			std::vector<float> b;
			int i = 0;
		};
		Line comb[2][NC], ap[2][NA];
		float lp[2][NC] = {};
		float fb = 0.8f, damp = 0.3f;

		void init(float sr)
		{
			static const int cl[2][NC] = {{1557, 1617, 1491, 1422}, {1580, 1640, 1514, 1445}};
			static const int al[2][NA] = {{225, 556}, {248, 579}};
			const float s = sr / 44100.f;
			for (int ch = 0; ch < 2; ch++)
			{
				for (int c = 0; c < NC; c++)
				{
					comb[ch][c].b.assign((size_t)std::max(8, (int)(cl[ch][c] * s)), 0.f);
					comb[ch][c].i = 0;
					lp[ch][c] = 0.f;
				}
				for (int a = 0; a < NA; a++)
				{
					ap[ch][a].b.assign((size_t)std::max(8, (int)(al[ch][a] * s)), 0.f);
					ap[ch][a].i = 0;
				}
			}
		}

		float tick(int ch, float x)
		{
			float acc = 0.f;
			for (int c = 0; c < NC; c++)
			{
				Line &l = comb[ch][c];
				float y = l.b[(size_t)l.i];
				lp[ch][c] = y * (1.f - damp) + lp[ch][c] * damp;
				l.b[(size_t)l.i] = x * 0.03f + lp[ch][c] * fb;
				if (++l.i >= (int)l.b.size())
					l.i = 0;
				acc += y;
			}
			for (int a = 0; a < NA; a++)
			{
				Line &l = ap[ch][a];
				float b = l.b[(size_t)l.i];
				float o = -acc + b;
				l.b[(size_t)l.i] = acc + b * 0.5f;
				if (++l.i >= (int)l.b.size())
					l.i = 0;
				acc = o;
			}
			return acc;
		}
	};

	// ------------------------------------------------------------------------------------
	// One captured tick, as shown on the display.
	// ------------------------------------------------------------------------------------
	struct TickRec
	{
		uint32_t tick = 0;
		uint32_t window = 0; // samples captured (W)
		uint32_t period = 0; // samples between the two ticks (N)
		uint8_t div = 0;
		uint8_t n = 0; // stored codes per channel (<= MAXV, evenly strided over the window)
		float bpm = 0.f;
		float lo = 20.f, hi = 20000.f;
		float mean[2] = {0.f, 0.f}; // mean frequency in Hz
		uint16_t code[2][MAXV] = {};
	};

	inline int hzRound(const TickRec &r, uint16_t code)
	{
		float f = r.lo * std::exp((float)code / 65535.f * std::log(r.hi / r.lo));
		return clp((int)std::lround(f), 0, 65535);
	}

	enum Mode
	{
		MODE_TONE,
		MODE_CRYPTO,
		MODE_CLOUD
	};

	enum ParamIds
	{
		P_MODE,	  // Mode
		P_BASE,	  // 0 bin, 1 oct, 2 dec, 3 hex
		P_WAVE,	  // 0 sine, 1 tri, 2 saw, 3 square
		P_VIEW,	  // 0 numbers, 1 spectrogram, 2 wave
		P_HOLD,	  // hold the current frequencies (no new captures)
		P_FREEZE, // freeze the recorded input buffer
		P_CLEAR,  // clear the log
		P_DIV,	  // 0..6 -> N, N/2, N/3, N/4, N/8, N/16, N/32
		P_KNOB0,  // KNOBS[]
		P_COUNT = P_KNOB0 + K_COUNT
	};

	// ------------------------------------------------------------------------------------
	// The module
	// ------------------------------------------------------------------------------------
	struct Crypto : Module
	{
		enum InputId
		{
			L_INPUT,
			R_INPUT,
			CLK_INPUT,
			VOCT_INPUT,
			DIV_INPUT,
			POS_CV, // POS_CV .. FB_CV are consecutive: POSITION, PITCH, SHAPE, MIX, SPACE, FEEDBACK
			PITCH_CV,
			SHAPE_CV,
			MIX_CV,
			SPACE_CV,
			FB_CV,
			INPUTS_LEN
		};
		enum OutputId
		{
			L_OUTPUT,
			R_OUTPUT,
			FL_OUTPUT,
			FR_OUTPUT,
			TRIG_OUTPUT,
			GATE_OUTPUT,
			OUTPUTS_LEN
		};

		// ---- shared with the UI thread ----
		TickRec history[LOG_N];
		std::atomic<uint32_t> ticks{0};
		std::atomic<uint32_t> periodShown{0}, windowShown{0};
		std::atomic<int> divShown{0}, grainsShown{0};
		std::atomic<float> bpmShown{0.f}, capProgress{0.f}, srShown{48000.f};
		std::atomic<float> meanShown[2];
		std::atomic<bool> extClock{false}, clearReq{false};
		std::atomic<float> vuIn[2], vuOut[2];
		float vizL[VIZ_N] = {}, vizR[VIZ_N] = {}; // last output samples (normalised), ring
		std::atomic<uint32_t> vizPos{0};
		int intBpm = 120; // internal clock tempo (menu), saved with the patch

		// ---- control values (audio thread) ----
		int mode = MODE_TONE, wave = 0, divIdx = 0;
		bool hold = false, freeze = false;
		float k[K_COUNT] = {};
		float fLo = 20.f, fHi = 20000.f, glideCoef = 1.f;
		float gIn = 1.f, gOut = 1.f, dryG = 0.f, wetG = 1.f, fbAmt = 0.f, space = 0.f;
		float shape = 0.35f, posN = 0.3f, pitchRatio = 1.f;
		uint32_t ctrlCount = 0;
		float lastSr = 48000.f, revSr = 0.f;
		int activeGrains = 0;

		// ---- clock / capture state ----
		dsp::SchmittTrigger clkTrig;
		dsp::PulseGenerator tickPulse;
		bool lastExt = false, haveEdge = false;
		uint32_t sinceEdge = 0, period = 0, intCount = 0;

		bool capOn = false;
		uint32_t capIdx = 0, capLen = 0, capStride = 1;
		int capN = 0, capDiv = 0;
		double sum[2] = {0.0, 0.0};
		float capLo = 20.f, capHi = 20000.f, capLn = 1.f;
		TickRec cur;

		// ---- synth / granular state ----
		bool hasData = false;
		float fTarget[2] = {440.f, 440.f}, fCur[2] = {440.f, 440.f};
		float phase[2] = {0.f, 0.f};
		uint16_t pool[2][MAXV] = {};
		int poolN = 0;
		float poolLo = 20.f, poolHi = 20000.f;

		std::vector<float> bufL, bufR; // recorded input history
		uint64_t wpos = 0;			   // samples written so far

		struct Grain
		{
			bool on = false;
			int ch = 0; // 0 / 1 = oscillator grain on that channel, 2 = stereo buffer grain
			double p = 0.0;
			float ph = 0.f, inc = 0.f, pl = 1.f, pr = 1.f;
			int age = 0, len = 1;
		};
		Grain grains[GRAINS];
		float spawnAcc = 0.f, nextSpawn = 1.f;
		Rng rng;
		Reverb rev;

		float pkIn[2] = {0.f, 0.f}, pkOut[2] = {0.f, 0.f};
		int vuCount = 0;

		Crypto() : rng((uint64_t)(uintptr_t)this ^ 0x5DEECE66Dull)
		{
			config(P_COUNT, INPUTS_LEN, OUTPUTS_LEN);
			configSwitch(P_MODE, 0, 2, 0, "Mode", {"Tone (mean frequency)", "Crypto grains", "Cloud (audio granulator)"});
			configSwitch(P_BASE, 0, 3, 3, "Number base", {"Binary", "Octal", "Decimal", "Hexadecimal"});
			configSwitch(P_WAVE, 0, 3, 0, "Waveform", {"Sine", "Triangle", "Saw", "Square"});
			configSwitch(P_VIEW, 0, 2, 0, "Display", {"Numbers", "Spectrogram", "Wave"});
			configSwitch(P_HOLD, 0, 1, 0, "Hold frequencies", {"Off", "On"});
			configSwitch(P_FREEZE, 0, 1, 0, "Freeze input buffer", {"Off", "On"});
			configButton(P_CLEAR, "Clear log");
			configSwitch(P_DIV, 0, DIV_COUNT - 1, 0, "Samples per tick",
						 {"N (whole tick)", "N/2", "N/3", "N/4", "N/8", "N/16", "N/32"});
			for (int i = 0; i < K_COUNT; i++)
			{
				const KnobDef &d = KNOBS[i];
				std::string unit = std::string(" ") + d.unit;
				if (d.exp)
					configParam(P_KNOB0 + i, 0.f, 1.f, d.def, d.label, unit, d.hi / d.lo, d.lo, 0.f);
				else
					configParam(P_KNOB0 + i, 0.f, 1.f, d.def, d.label, unit, 0.f, d.hi - d.lo, d.lo);
			}
			configInput(L_INPUT, "Left audio");
			configInput(R_INPUT, "Right audio (normalled to left)");
			configInput(CLK_INPUT, "Clock (tick)");
			configInput(VOCT_INPUT, "Pitch (1 V/oct)");
			configInput(DIV_INPUT, "Samples-per-tick CV (0-10 V sweeps N .. N/32)");
			configInput(POS_CV, "Position CV");
			configInput(PITCH_CV, "Pitch CV");
			configInput(SHAPE_CV, "Shape CV");
			configInput(MIX_CV, "Mix CV");
			configInput(SPACE_CV, "Space CV");
			configInput(FB_CV, "Feedback CV");
			configOutput(L_OUTPUT, "Left audio");
			configOutput(R_OUTPUT, "Right audio");
			configOutput(FL_OUTPUT, "Left mean frequency (1 V/oct, 0 V = C4)");
			configOutput(FR_OUTPUT, "Right mean frequency (1 V/oct, 0 V = C4)");
			configOutput(TRIG_OUTPUT, "Window closed trigger");
			configOutput(GATE_OUTPUT, "Capture window gate");
			for (int i = 0; i < 2; i++)
			{
				vuIn[i].store(0.f);
				vuOut[i].store(0.f);
				meanShown[i].store(0.f);
			}
			bufL.assign(BUF_N, 0.f);
			bufR.assign(BUF_N, 0.f);
		}

		void resetState()
		{
			haveEdge = false;
			sinceEdge = 0;
			period = 0;
			intCount = 0;
			capOn = false;
			hasData = false;
			poolN = 0;
			for (auto &g : grains)
				g.on = false;
			phase[0] = phase[1] = 0.f;
			periodShown.store(0);
			windowShown.store(0);
			bpmShown.store(0.f);
		}

		void onReset() override
		{
			resetState();
			std::fill(bufL.begin(), bufL.end(), 0.f);
			std::fill(bufR.begin(), bufR.end(), 0.f);
			wpos = 0;
			ticks.store(0, std::memory_order_release);
		}

		// ================================================================================
		// Control (every 16 samples)
		// ================================================================================
		float knobValue(int idx, int cvInput)
		{
			float x = params[P_KNOB0 + idx].getValue();
			if (cvInput >= 0 && inputs[cvInput].isConnected())
				x += inputs[cvInput].getVoltage() * 0.1f; // 0..10 V covers the whole knob
			return mapKnob(KNOBS[idx], x);
		}

		void updateControl(float sr)
		{
			mode = clp((int)std::round(params[P_MODE].getValue()), 0, 2);
			hold = params[P_HOLD].getValue() > 0.5f;
			freeze = params[P_FREEZE].getValue() > 0.5f;
			wave = clp((int)std::round(params[P_WAVE].getValue()), 0, 3);

			for (int i = 0; i < K_COUNT; i++)
				k[i] = mapKnob(KNOBS[i], params[P_KNOB0 + i].getValue());
			k[K_POS] = knobValue(K_POS, POS_CV);
			k[K_PITCH] = knobValue(K_PITCH, PITCH_CV);
			k[K_SHAPE] = knobValue(K_SHAPE, SHAPE_CV);
			k[K_MIX] = knobValue(K_MIX, MIX_CV);
			k[K_SPACE] = knobValue(K_SPACE, SPACE_CV);
			k[K_FB] = knobValue(K_FB, FB_CV);

			fLo = k[K_LOW];
			fHi = std::max(k[K_HIGH], fLo * 1.5f);
			glideCoef = k[K_GLIDE] > 0.5f ? 1.f - std::exp(-1000.f / (k[K_GLIDE] * sr)) : 1.f;

			float semis = k[K_PITCH] + (inputs[VOCT_INPUT].isConnected() ? 12.f * inputs[VOCT_INPUT].getVoltage() : 0.f);
			pitchRatio = std::exp2(clp(semis, -60.f, 60.f) / 12.f);
			posN = clp(k[K_POS] / 100.f, 0.f, 1.f);
			shape = clp(k[K_SHAPE] / 100.f, 0.f, 1.f);
			space = clp(k[K_SPACE] / 100.f, 0.f, 1.f);
			fbAmt = clp(k[K_FB] / 100.f, 0.f, 1.f) * 0.95f;
			float mix = clp(k[K_MIX] / 100.f, 0.f, 1.f);
			dryG = std::cos(mix * PI_F * 0.5f);
			wetG = std::sin(mix * PI_F * 0.5f);
			gIn = k[K_GIN] / 100.f;
			gOut = k[K_GOUT] / 100.f;
			rev.fb = 0.70f + 0.28f * space;
			rev.damp = 0.40f - 0.25f * space;

			float dv = params[P_DIV].getValue();
			if (inputs[DIV_INPUT].isConnected())
				dv += inputs[DIV_INPUT].getVoltage() * 0.6f; // 0..10 V covers all seven options
			divIdx = clp((int)std::round(dv), 0, DIV_COUNT - 1);
			divShown.store(divIdx, std::memory_order_relaxed);

			if (params[P_CLEAR].getValue() > 0.5f)
			{
				clearReq.store(true);
				params[P_CLEAR].setValue(0.f);
			}
			if (clearReq.exchange(false))
				ticks.store(0, std::memory_order_release);

			extClock.store(inputs[CLK_INPUT].isConnected(), std::memory_order_relaxed);
			capProgress.store(capOn && capLen ? (float)capIdx / (float)capLen : 0.f, std::memory_order_relaxed);
			grainsShown.store(activeGrains, std::memory_order_relaxed);
		}

		// ================================================================================
		// Clock + capture
		// ================================================================================
		void startCapture()
		{
			capDiv = divIdx;
			capLen = std::max<uint32_t>(1, period / (uint32_t)DIVS[capDiv]);
			capStride = (capLen + MAXV - 1) / MAXV;
			capIdx = 0;
			capN = 0;
			sum[0] = sum[1] = 0.0;
			capLo = fLo;
			capHi = fHi;
			capLn = std::log(capHi / capLo);
			capOn = true;
			windowShown.store(capLen, std::memory_order_relaxed);
		}

		void finishCapture()
		{
			capOn = false;
			if (capIdx == 0)
				return;

			cur.window = capIdx;
			cur.period = period;
			cur.div = (uint8_t)capDiv;
			cur.n = (uint8_t)capN;
			cur.bpm = period ? 60.f * lastSr / (float)period : 0.f;
			cur.lo = capLo;
			cur.hi = capHi;
			for (int ch = 0; ch < 2; ch++)
			{
				cur.mean[ch] = (float)(sum[ch] / (double)capIdx);
				fTarget[ch] = cur.mean[ch];
				meanShown[ch].store(cur.mean[ch], std::memory_order_relaxed);
				if (!hasData)
					fCur[ch] = fTarget[ch];
				for (int j = 0; j < capN; j++)
					pool[ch][j] = cur.code[ch][j];
			}
			poolN = capN;
			poolLo = capLo;
			poolHi = capHi;
			hasData = true;

			uint32_t t = ticks.load(std::memory_order_relaxed);
			cur.tick = t + 1;
			history[t % LOG_N] = cur;
			ticks.store(t + 1, std::memory_order_release);
			tickPulse.trigger(1e-3f);
		}

		void onTick()
		{
			if (capOn)
				finishCapture(); // the next tick arrived early: close the window with what we have
			if (hold || period < 2)
				return;
			startCapture();
		}

		void captureSample(float l, float r)
		{
			const float v[2] = {l, r};
			const bool store = (capIdx % capStride) == 0 && capN < MAXV;
			for (int ch = 0; ch < 2; ch++)
			{
				float x = clp((v[ch] + 5.f) * 0.1f, 0.f, 1.f);
				sum[ch] += (double)(capLo * std::exp(x * capLn));
				if (store)
					cur.code[ch][capN] = (uint16_t)std::lround(x * 65535.f);
			}
			if (store)
				capN++;
			if (++capIdx >= capLen)
				finishCapture();
		}

		// ================================================================================
		// Granular engines
		// ================================================================================
		// CRYPTO grains: pitch drawn from the captured frequency codes (POSITION picks the region of
		// the code list, GLITCH flips bits of the code, SPREAD detunes).
		// CLOUD grains: read the recorded input audio (POSITION = how far back).
		void spawnGrain(int ch)
		{
			Grain *g = nullptr;
			for (auto &gr : grains)
				if (!gr.on)
				{
					g = &gr;
					break;
				}
			if (!g)
				return;

			const float glitch = k[K_GLITCH] / 100.f;
			const float spread = k[K_SPREAD] / 100.f;
			int len = std::max(16, (int)(k[K_SIZE] * 0.001f * lastSr));
			if (rng.uniform() < glitch) // glitchy grain length: 0.25x .. 2x
				len = std::max(16, (int)((float)len * (0.25f + 1.75f * rng.uniform())));

			if (mode == MODE_CRYPTO)
			{
				if (poolN <= 0)
					return;
				int idx = (int)(posN * (float)(poolN - 1) + (rng.uniform() - 0.5f) * spread * (float)poolN);
				uint16_t c = pool[ch][clp(idx, 0, poolN - 1)];
				if (rng.uniform() < glitch) // flip 1-3 random bits of the code
				{
					int nb = 1 + (int)(rng.uniform() * 3.f);
					for (int i = 0; i < nb; i++)
						c = (uint16_t)(c ^ (1u << (int)(rng.uniform() * 16.f)));
				}
				float f = poolLo * std::exp((float)c / 65535.f * std::log(poolHi / poolLo));
				f *= pitchRatio * std::exp2(spread * (2.f * rng.uniform() - 1.f));
				f = clp(f, 5.f, 0.45f * lastSr);
				g->ch = ch;
				g->ph = 0.f;
				g->inc = f / lastSr;
			}
			else
			{
				float ratio = pitchRatio * std::exp2(0.5f * spread * (2.f * rng.uniform() - 1.f));
				bool rev = rng.uniform() < glitch * 0.5f; // glitch: some grains play backwards
				double lagMin = rev ? 256.0 : 256.0 + (double)len * std::max(0.0, (double)ratio - 1.0);
				double lagMax = std::min<double>((double)wpos, (double)(BUF_N - 8192)) - (rev ? (double)len * ratio : 0.0);
				if (lagMax <= lagMin)
					return;
				float pn = clp(posN + (2.f * rng.uniform() - 1.f) * 0.15f * spread, 0.f, 1.f);
				double lag = lagMin + (double)pn * (lagMax - lagMin);
				float pan = clp(0.5f + (rng.uniform() - 0.5f) * spread, 0.f, 1.f);
				g->ch = 2;
				g->p = (double)wpos - lag;
				g->inc = rev ? -ratio : ratio;
				g->pl = std::cos(pan * PI_F * 0.5f);
				g->pr = std::sin(pan * PI_F * 0.5f);
			}
			g->age = 0;
			g->len = len;
			g->on = true;
		}

		void grainEngine(float &eL, float &eR)
		{
			spawnAcc += k[K_DENS] / lastSr;
			if (spawnAcc >= nextSpawn)
			{
				spawnAcc -= nextSpawn;
				nextSpawn = 0.7f + 0.6f * rng.uniform();
				if (mode == MODE_CRYPTO)
				{
					spawnGrain(0);
					spawnGrain(1);
				}
				else
					spawnGrain(2);
			}

			const uint64_t loopLen = std::max<uint64_t>(4, std::min<uint64_t>(wpos, (uint64_t)(2.f * lastSr)));
			float sl = 0.f, sr2 = 0.f;
			int n = 0;
			for (auto &g : grains)
			{
				if (!g.on)
					continue;
				// a grain that mixes the wrong engine (mode switched) just dies
				if ((mode == MODE_CRYPTO) != (g.ch < 2))
				{
					g.on = false;
					continue;
				}
				float env = grainEnv((float)g.age / (float)g.len, shape);
				if (g.ch == 2)
				{
					if (g.p >= (double)wpos - 2.0)
					{
						if (freeze)
							g.p -= (double)loopLen; // frozen buffer: loop inside the last 2 s
						else
						{
							g.on = false;
							continue;
						}
					}
					if (g.p < (double)wpos - (double)(BUF_N - 4))
					{
						g.on = false;
						continue;
					}
					int64_t ip = (int64_t)std::floor(g.p);
					float fr = (float)(g.p - (double)ip);
					size_t i0 = (size_t)ip & BUF_MASK, i1 = (size_t)(ip + 1) & BUF_MASK;
					sl += env * g.pl * (bufL[i0] + (bufL[i1] - bufL[i0]) * fr);
					sr2 += env * g.pr * (bufR[i0] + (bufR[i1] - bufR[i0]) * fr);
					g.p += (double)g.inc;
				}
				else
				{
					float s = env * oscSample(wave, g.ph, std::min(g.inc, 0.49f));
					if (g.ch == 0)
						sl += s;
					else
						sr2 += s;
					g.ph += g.inc;
					if (g.ph >= 1.f)
						g.ph -= 1.f;
				}
				n++;
				if (++g.age >= g.len)
					g.on = false;
			}
			activeGrains = n;
			float norm = 1.f / std::sqrt((float)std::max(1, n));
			eL = sl * norm;
			eR = sr2 * norm;
		}

		// ================================================================================
		// Audio thread
		// ================================================================================
		void process(const ProcessArgs &args) override
		{
			const float sr = args.sampleRate;
			lastSr = sr;
			if (sr != revSr)
			{
				rev.init(sr);
				revSr = sr;
				srShown.store(sr, std::memory_order_relaxed);
			}
			if ((ctrlCount++ & 15u) == 0)
				updateControl(sr);

			const bool cL = inputs[L_INPUT].isConnected(), cR = inputs[R_INPUT].isConnected();
			const float l = cL ? inputs[L_INPUT].getVoltage() : (cR ? inputs[R_INPUT].getVoltage() : 0.f);
			const float r = cR ? inputs[R_INPUT].getVoltage() : l;
			pkIn[0] = std::max(pkIn[0], std::fabs(l) / 5.f);
			pkIn[1] = std::max(pkIn[1], std::fabs(r) / 5.f);

			// ---- clock ----
			const bool ext = inputs[CLK_INPUT].isConnected();
			if (ext != lastExt)
			{
				lastExt = ext;
				resetState();
			}
			if (ext)
			{
				sinceEdge++;
				if (clkTrig.process(inputs[CLK_INPUT].getVoltage(), 0.1f, 2.f))
				{
					if (haveEdge)
					{
						period = sinceEdge;
						periodShown.store(period, std::memory_order_relaxed);
						bpmShown.store(60.f * sr / (float)period, std::memory_order_relaxed);
					}
					haveEdge = true;
					sinceEdge = 0;
					onTick();
				}
			}
			else
			{
				period = (uint32_t)(60.f * sr / (float)std::max(1, intBpm));
				periodShown.store(period, std::memory_order_relaxed);
				bpmShown.store((float)intBpm, std::memory_order_relaxed);
				if (++intCount >= period)
				{
					intCount = 0;
					onTick();
				}
			}
			if (capOn)
				captureSample(l, r);

			// ---- input stage ----
			const float dL = l * 0.2f * gIn, dR = r * 0.2f * gIn; // +-5 V = +-1

			// ---- engine ----
			float eL = 0.f, eR = 0.f;
			for (int ch = 0; ch < 2; ch++)
				fCur[ch] += (fTarget[ch] - fCur[ch]) * glideCoef;
			if (mode == MODE_CLOUD)
				grainEngine(eL, eR);
			else if (hasData)
			{
				if (mode == MODE_CRYPTO)
					grainEngine(eL, eR);
				else
				{
					float *e[2] = {&eL, &eR};
					for (int ch = 0; ch < 2; ch++)
					{
						float dt = std::min(fCur[ch] * pitchRatio / sr, 0.49f);
						*e[ch] = oscSample(wave, phase[ch], dt);
						phase[ch] += dt;
						if (phase[ch] >= 1.f)
							phase[ch] -= 1.f;
					}
				}
			}

			// ---- space (reverb) + feedback into the recorded buffer ----
			const float yL = eL * (1.f - 0.5f * space) + rev.tick(0, eL) * space * 2.f;
			const float yR = eR * (1.f - 0.5f * space) + rev.tick(1, eR) * space * 2.f;
			if (!freeze)
			{
				bufL[(size_t)wpos & BUF_MASK] = clp(dL + std::tanh(yL * fbAmt) * fbAmt, -2.f, 2.f);
				bufR[(size_t)wpos & BUF_MASK] = clp(dR + std::tanh(yR * fbAmt) * fbAmt, -2.f, 2.f);
				wpos++;
			}

			// ---- mix + output ----
			const float oL = clp((dL * dryG + yL * wetG) * gOut * 5.f, -10.f, 10.f);
			const float oR = clp((dR * dryG + yR * wetG) * gOut * 5.f, -10.f, 10.f);
			outputs[L_OUTPUT].setVoltage(oL);
			outputs[R_OUTPUT].setVoltage(oR);
			outputs[FL_OUTPUT].setVoltage(hasData ? std::log2(std::max(fCur[0] * pitchRatio, 1.f) / 261.6256f) : 0.f);
			outputs[FR_OUTPUT].setVoltage(hasData ? std::log2(std::max(fCur[1] * pitchRatio, 1.f) / 261.6256f) : 0.f);
			outputs[TRIG_OUTPUT].setVoltage(tickPulse.process(args.sampleTime) ? 10.f : 0.f);
			outputs[GATE_OUTPUT].setVoltage(capOn ? 10.f : 0.f);

			// ---- visualisation / meters ----
			uint32_t vp = vizPos.load(std::memory_order_relaxed);
			vizL[vp & (VIZ_N - 1)] = oL * 0.2f;
			vizR[vp & (VIZ_N - 1)] = oR * 0.2f;
			vizPos.store(vp + 1, std::memory_order_release);

			pkOut[0] = std::max(pkOut[0], std::fabs(oL) / 5.f);
			pkOut[1] = std::max(pkOut[1], std::fabs(oR) / 5.f);
			if (++vuCount >= 256)
			{
				vuCount = 0;
				for (int i = 0; i < 2; i++)
				{
					vuIn[i].store(std::max(pkIn[i], vuIn[i].load() * 0.85f), std::memory_order_relaxed);
					vuOut[i].store(std::max(pkOut[i], vuOut[i].load() * 0.85f), std::memory_order_relaxed);
					pkIn[i] = pkOut[i] = 0.f;
				}
			}
		}

		json_t *dataToJson() override
		{
			json_t *root = json_object();
			json_object_set_new(root, "intBpm", json_integer(intBpm));
			return root;
		}

		void dataFromJson(json_t *root) override
		{
			json_t *j = json_object_get(root, "intBpm");
			if (j)
				intBpm = clp((int)json_integer_value(j), 20, 300);
		}
	};

	// ------------------------------------------------------------------------------------
	// Widgets
	// ------------------------------------------------------------------------------------
	static const float PW = 54 * RACK_GRID_WIDTH; // 810: double the width of MP3_DNA
	constexpr float SCR_X = 16.f, SCR_Y = 28.f, SCR_W = 640.f, SCR_H = 128.f;
	constexpr float IN_W = 778.f; // usable width between the side margins

	struct ChipRow : app::ParamWidget
	{
		std::vector<std::string> labels;
		void draw(const DrawArgs &args) override
		{
			int n = (int)labels.size();
			if (n == 0)
				return;
			int sel = 0;
			if (auto *pq = getParamQuantity())
				sel = (int)std::round(pq->getValue());
			float cw = box.size.x / (float)n;
			for (int i = 0; i < n; i++)
				agmm::chip(args.vg, i * cw + 1.f, 0.f, cw - 2.f, box.size.y, labels[(size_t)i].c_str(), i == sel, agmm::accent());
		}
		void onButton(const event::Button &e) override
		{
			if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT && !labels.empty())
			{
				if (auto *pq = getParamQuantity())
				{
					int n = (int)labels.size();
					int i = clp((int)(e.pos.x / (box.size.x / (float)n)), 0, n - 1);
					pq->setValue((float)i);
				}
				e.consume(this);
				return;
			}
			ParamWidget::onButton(e);
		}
	};

	struct ChipButton : app::ParamWidget
	{
		std::string label;
		bool latch = true;
		NVGcolor col = agmm::accent();
		void draw(const DrawArgs &args) override
		{
			bool on = false;
			if (auto *pq = getParamQuantity())
				on = pq->getValue() > 0.5f;
			agmm::chip(args.vg, 1.f, 0.f, box.size.x - 2.f, box.size.y, label.c_str(), on, col);
		}
		void onButton(const event::Button &e) override
		{
			if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT)
			{
				if (auto *pq = getParamQuantity())
					pq->setValue(latch ? (pq->getValue() > 0.5f ? 0.f : 1.f) : 1.f);
				e.consume(this);
				return;
			}
			ParamWidget::onButton(e);
		}
	};

	struct AgmmKnob : app::Knob
	{
		AgmmKnob() { box.size = Vec(26.f, 26.f); }
		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			const float r = box.size.x / 2.f;
			float v = 0.5f;
			if (auto *pq = getParamQuantity())
				v = pq->getScaledValue();
			const float a0 = -0.75f * PI_F, a1 = 0.75f * PI_F;
			const float a = a0 + v * (a1 - a0);

			nvgBeginPath(vg);
			nvgCircle(vg, r, r, r - 4.f);
			nvgFillColor(vg, nvgRGB(0x0b, 0x0c, 0x0e));
			nvgFill(vg);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			const float cs = -PI_F / 2.f;
			nvgBeginPath(vg);
			nvgArc(vg, r, r, r - 1.5f, a0 + cs, a1 + cs, NVG_CW);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
			nvgStrokeWidth(vg, 1.5f);
			nvgStroke(vg);
			if (v > 0.002f)
			{
				nvgBeginPath(vg);
				nvgArc(vg, r, r, r - 1.5f, a0 + cs, a + cs, NVG_CW);
				nvgStrokeColor(vg, agmm::accent());
				nvgStrokeWidth(vg, 1.5f);
				nvgStroke(vg);
			}
			nvgBeginPath(vg);
			nvgMoveTo(vg, r + std::sin(a) * (r * 0.25f), r - std::cos(a) * (r * 0.25f));
			nvgLineTo(vg, r + std::sin(a) * (r - 5.f), r - std::cos(a) * (r - 5.f));
			nvgStrokeColor(vg, agmm::textCol());
			nvgStrokeWidth(vg, 1.5f);
			nvgStroke(vg);
		}
	};

	// ---- display text (NUMBERS view) ----
	struct DLine
	{
		std::string head, body;
		NVGcolor hcol, bcol;
	};

	// Appends the three lines of one tick, bottom-up (R row, L row, header) so that the vector
	// reads newest-first from the bottom of the screen.
	inline void buildLines(std::vector<DLine> &out, const TickRec &r, int base, int cols)
	{
		const int d = DIGITS[base];
		const NVGcolor lil = nvgRGB(0xd8, 0xb0, 0xff);
		for (int ch = 1; ch >= 0; ch--)
		{
			std::string head = std::string(ch ? "R " : "L ") +
							   fmtNum((uint32_t)clp((int)std::lround(r.mean[ch]), 0, 65535), base) + " | ";
			int avail = cols - (int)head.size();
			int per = d + 1;
			int cnt = (int)r.n;
			if (cnt * per > avail)
				cnt = std::max(0, (avail - 3) / per);
			std::string body;
			for (int j = 0; j < cnt; j++)
			{
				body += fmtNum((uint32_t)hzRound(r, r.code[ch][j]), base);
				body += ' ';
			}
			if (cnt < (int)r.n)
				body += "+" + std::to_string((int)r.n - cnt);
			out.push_back({head, body, ch ? agmm::violet() : agmm::accent(), ch ? lil : agmm::bright()});
		}
		char b[32];
		std::snprintf(b, sizeof b, "%.1f", r.bpm);
		char h[16];
		std::snprintf(h, sizeof h, "#%04u ", (unsigned)(r.tick % 10000u));
		out.push_back({h,
					   std::string(DIV_NAMES[std::min<int>(r.div, DIV_COUNT - 1)]) + "  W=" + std::to_string(r.window) +
						   "  " + b + "BPM",
					   agmm::dim(), agmm::dim()});
	}

	inline void makeDemo(TickRec *r, int n)
	{
		uint32_t s = 12345u;
		for (int i = 0; i < n; i++)
		{
			r[i].tick = (uint32_t)(i + 1);
			r[i].window = 3000;
			r[i].period = 24000;
			r[i].div = 4;
			r[i].n = MAXV;
			r[i].bpm = 120.f;
			for (int ch = 0; ch < 2; ch++)
			{
				double sm = 0.0;
				for (int j = 0; j < MAXV; j++)
				{
					s = s * 1664525u + 1013904223u;
					r[i].code[ch][j] = (uint16_t)(32768 + ((s >> 16) & 0x3FFF) - 0x2000);
					sm += hzRound(r[i], r[i].code[ch][j]);
				}
				r[i].mean[ch] = (float)(sm / MAXV);
			}
		}
	}

	inline std::string tickText(const TickRec &r, int base)
	{
		static const char *names[4] = {"BIN", "OCT", "DEC", "HEX"};
		std::string s = "#" + std::to_string(r.tick) + " " + DIV_NAMES[std::min<int>(r.div, DIV_COUNT - 1)] +
						" N=" + std::to_string(r.period) + " W=" + std::to_string(r.window) + " (" + names[base] + ", Hz)\n";
		for (int ch = 0; ch < 2; ch++)
		{
			s += ch ? "R mean " : "L mean ";
			s += fmtNum((uint32_t)clp((int)std::lround(r.mean[ch]), 0, 65535), base) + "\n";
			s += ch ? "R: " : "L: ";
			for (int j = 0; j < (int)r.n; j++)
				s += fmtNum((uint32_t)hzRound(r, r.code[ch][j]), base) + " ";
			s += "\n";
		}
		return s;
	}

	// ---- the screen: NUMBERS / SPECTROGRAM / WAVE ----
	struct DataScreen : widget::Widget
	{
		static constexpr int FN = 1024, BANDS = 40, COLS = 106;
		Crypto *module = nullptr;
		dsp::RealFFT fft;
		std::vector<float> fin, fout, win, spec; // spec: COLS x BANDS levels 0..1, ring of columns
		int specHead = 0;
		float demoPh = 0.f;

		DataScreen() : fft(FN), fin(FN, 0.f), fout(FN, 0.f), win(FN, 0.f), spec((size_t)(COLS * BANDS), 0.f)
		{
			for (int i = 0; i < FN; i++)
				win[(size_t)i] = 0.5f * (1.f - std::cos(2.f * PI_F * (float)i / (float)(FN - 1)));
		}

		void drawLayer(const DrawArgs &args, int layer) override
		{
			if (layer == 1)
				drawData(args.vg);
			Widget::drawLayer(args, layer);
		}

		// Computes one new spectrogram column from the latest output samples.
		void updateSpec()
		{
			float *col = &spec[(size_t)(specHead * BANDS)];
			if (!module)
			{
				demoPh += 0.12f;
				for (int b = 0; b < BANDS; b++)
					col[b] = clp(0.55f + 0.35f * std::sin(demoPh + (float)b * 0.35f) - (float)b * 0.006f, 0.f, 1.f);
			}
			else
			{
				const uint32_t vp = module->vizPos.load(std::memory_order_acquire);
				for (int i = 0; i < FN; i++)
				{
					size_t idx = (size_t)(vp - (uint32_t)FN + (uint32_t)i) & (size_t)(VIZ_N - 1);
					fin[(size_t)i] = 0.5f * (module->vizL[idx] + module->vizR[idx]) * win[(size_t)i];
				}
				fft.rfft(fin.data(), fout.data());
				const float sr = std::max(8000.f, module->srShown.load(std::memory_order_relaxed));
				for (int b = 0; b < BANDS; b++)
				{
					float f0 = 20.f * std::pow(1000.f, (float)b / (float)BANDS);
					float f1 = std::min(20.f * std::pow(1000.f, (float)(b + 1) / (float)BANDS), 0.49f * sr);
					int k0 = clp((int)(f0 * (float)FN / sr), 1, FN / 2 - 1);
					int k1 = clp((int)(f1 * (float)FN / sr), k0, FN / 2 - 1);
					float m = 0.f;
					for (int kk = k0; kk <= k1; kk++)
					{
						float re = fout[(size_t)(2 * kk)], im = fout[(size_t)(2 * kk + 1)];
						m = std::max(m, std::sqrt(re * re + im * im) * 4.f / (float)FN);
					}
					col[b] = clp((20.f * std::log10(m + 1e-9f) + 90.f) / 80.f, 0.f, 1.f);
				}
			}
			specHead = (specHead + 1) % COLS;
		}

		void drawSpectrogram(NVGcontext *vg)
		{
			updateSpec();
			const float cw = box.size.x / (float)COLS, ch = box.size.y / (float)BANDS;
			const NVGcolor cols[6] = {agmm::withA(agmm::accent(), 0.f), agmm::withA(agmm::accent(), 0.18f),
									  agmm::withA(agmm::accent(), 0.38f), agmm::withA(agmm::accent(), 0.65f),
									  agmm::withA(agmm::bright(), 0.9f), agmm::withA(agmm::warm(), 0.95f)};
			for (int q = 1; q < 6; q++)
			{
				nvgBeginPath(vg);
				for (int c = 0; c < COLS; c++)
				{
					const float *col = &spec[(size_t)(((specHead + c) % COLS) * BANDS)];
					for (int b = 0; b < BANDS; b++)
						if ((int)(col[b] * 5.99f) == q)
							nvgRect(vg, (float)c * cw, box.size.y - (float)(b + 1) * ch, cw + 0.5f, ch + 0.5f);
				}
				nvgFillColor(vg, cols[q]);
				nvgFill(vg);
			}
			const float fq[3] = {100.f, 1000.f, 10000.f};
			const char *fl[3] = {"100", "1K", "10K"};
			for (int i = 0; i < 3; i++)
			{
				float b = (float)BANDS * std::log10(fq[i] / 20.f) / 3.f;
				agmm::text(vg, 3.f, box.size.y - (b + 0.5f) * ch, fl[i], 7.f, agmm::dim(), agmm::ALIGN_L, 0.f);
			}
		}

		void drawWave(NVGcontext *vg)
		{
			const float H = box.size.y, W = box.size.x;
			const float laneH = H * 0.23f, cy[2] = {H * 0.27f, H * 0.73f};
			nvgBeginPath(vg);
			for (int i = 0; i < 2; i++)
			{
				nvgMoveTo(vg, 0.f, cy[i]);
				nvgLineTo(vg, W, cy[i]);
			}
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 24));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			const int span = 2048, cols = std::max(1, (int)(W / 2.f)), spp = std::max(1, span / cols);
			const uint32_t vp = module ? module->vizPos.load(std::memory_order_acquire) : 0u;
			for (int ch = 0; ch < 2; ch++)
			{
				nvgBeginPath(vg);
				for (int c = 0; c < cols; c++)
				{
					float mn = 0.f, mx = 0.f;
					if (module)
					{
						mn = 1e9f;
						mx = -1e9f;
						for (int j = 0; j < spp; j++)
						{
							size_t idx = (size_t)(vp - (uint32_t)span + (uint32_t)(c * spp + j)) & (size_t)(VIZ_N - 1);
							float v = ch ? module->vizR[idx] : module->vizL[idx];
							mn = std::min(mn, v);
							mx = std::max(mx, v);
						}
					}
					else
					{
						float ph = (float)c * 0.09f + (ch ? 1.1f : 0.f);
						mn = 0.8f * std::sin(ph) * std::sin(ph * 0.13f);
						mx = mn + 0.04f;
					}
					float x = (float)c * (W / (float)cols) + 0.5f;
					nvgMoveTo(vg, x, cy[ch] - clp(mx, -1.f, 1.f) * laneH);
					nvgLineTo(vg, x, cy[ch] - clp(mn, -1.f, 1.f) * laneH + 0.8f);
				}
				nvgStrokeColor(vg, ch ? agmm::violet() : agmm::accent());
				nvgStrokeWidth(vg, 1.2f);
				nvgStroke(vg);
			}
			agmm::text(vg, 4.f, 8.f, "L", 7.f, agmm::accent(), agmm::ALIGN_L, 0.f);
			agmm::text(vg, 4.f, H * 0.5f + 8.f, "R", 7.f, agmm::violet(), agmm::ALIGN_L, 0.f);
		}

		void drawNumbers(NVGcontext *vg)
		{
			const float fs = 8.f, sp = 0.4f, rowH = 9.5f;
			const float cw = agmm::textWidth(vg, "0000000000", fs, sp) / 10.f;
			if (cw <= 0.f)
				return;
			const int base = module ? clp((int)std::round(module->params[P_BASE].getValue()), 0, 3) : 3;
			const int cols = (int)((box.size.x - 8.f) / cw);
			const int rows = std::max(1, (int)((box.size.y - 6.f) / rowH));

			uint32_t t = module ? module->ticks.load(std::memory_order_acquire) : 4;
			if (t == 0)
			{
				bool ext = module && module->extClock.load();
				agmm::text(vg, box.size.x / 2.f, box.size.y / 2.f - 5.f,
						   ext ? "WAITING FOR 2 CLOCK PULSES" : "AWAITING FIRST TICK", 8.f, agmm::dim(), agmm::ALIGN_C, 2.f);
				agmm::text(vg, box.size.x / 2.f, box.size.y / 2.f + 7.f,
						   ext ? "EXTERNAL CLOCK" : "INTERNAL CLOCK - NO CLK PATCHED", 7.f, agmm::dim(), agmm::ALIGN_C, 1.f);
				return;
			}
			static TickRec demo[4];
			static bool demoInit = false;
			if (!module && !demoInit)
			{
				makeDemo(demo, 4);
				demoInit = true;
			}
			std::vector<DLine> lines;
			uint32_t oldest = t > (uint32_t)LOG_N ? t - (uint32_t)LOG_N : 0;
			for (uint32_t i = t; i > oldest && (int)lines.size() < rows + 3; i--)
			{
				TickRec rec = module ? module->history[(i - 1) % LOG_N] : demo[(i - 1) % 4];
				buildLines(lines, rec, base, cols);
			}
			for (int i = 0; i < (int)lines.size() && i < rows; i++)
			{
				float y = 3.f + (float)(rows - 1 - i) * rowH + rowH / 2.f;
				const DLine &ln = lines[(size_t)i];
				agmm::text(vg, 4.f, y, ln.head.c_str(), fs, ln.hcol, agmm::ALIGN_L, sp);
				agmm::text(vg, 4.f + (float)ln.head.size() * cw, y, ln.body.c_str(), fs, ln.bcol, agmm::ALIGN_L, sp);
			}
		}

		void drawData(NVGcontext *vg)
		{
			nvgSave(vg);
			nvgScissor(vg, 0, 0, box.size.x, box.size.y);
			const int view = module ? clp((int)std::round(module->params[P_VIEW].getValue()), 0, 2) : 0;
			if (view == 1)
				drawSpectrogram(vg);
			else if (view == 2)
				drawWave(vg);
			else
				drawNumbers(vg);

			// capture window progress along the bottom edge
			if (module)
			{
				float p = clp(module->capProgress.load(std::memory_order_relaxed), 0.f, 1.f);
				nvgBeginPath(vg);
				nvgRect(vg, 0.f, box.size.y - 2.f, box.size.x * p, 2.f);
				nvgFillColor(vg, agmm::withA(agmm::accent(), 0.8f));
				nvgFill(vg);
			}
			nvgRestore(vg);
		}
	};

	struct CryptoWidget : ModuleWidget
	{
		CryptoWidget(Crypto *module)
		{
			setModule(module);
			box.size = Vec(PW, RACK_GRID_HEIGHT);

			addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
			addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
			addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
			addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

			auto *ds = new DataScreen;
			ds->module = module;
			ds->box.pos = Vec(SCR_X, SCR_Y);
			ds->box.size = Vec(SCR_W, SCR_H);
			addChild(ds);

			auto row = [&](float x, float y, float w, int id, std::vector<std::string> labels)
			{
				auto *r = createParam<ChipRow>(Vec(x, y), module, id);
				r->box.size = Vec(w, 16.f);
				r->labels = labels;
				addParam(r);
			};
			row(16.f, 186.f, 220.f, P_MODE, {"TONE", "CRYPTO", "CLOUD"});
			row(246.f, 186.f, 200.f, P_WAVE, {"SINE", "TRI", "SAW", "SQR"});
			row(456.f, 186.f, 180.f, P_BASE, {"BIN", "OCT", "DEC", "HEX"});
			row(646.f, 186.f, 148.f, P_VIEW, {"NUM", "SPEC", "WAVE"});

			auto btn = [&](float x, float y, float w, int id, const char *label, bool latch, NVGcolor col)
			{
				auto *b = createParam<ChipButton>(Vec(x, y), module, id);
				b->box.size = Vec(w, 16.f);
				b->label = label;
				b->latch = latch;
				b->col = col;
				addParam(b);
			};
			btn(16.f, 206.f, 100.f, P_HOLD, "HOLD", true, agmm::warm());
			btn(126.f, 206.f, 100.f, P_FREEZE, "FREEZE", true, agmm::bright());
			btn(236.f, 206.f, 100.f, P_CLEAR, "CLEAR", false, agmm::violet());

			addParam(createParamCentered<AgmmKnob>(Vec(knobX(0), 240.f), module, P_DIV));
			for (int i = 0; i < K_COUNT; i++)
				addParam(createParamCentered<AgmmKnob>(Vec(knobX(i + 1), 240.f), module, P_KNOB0 + i));

			const float py = 318.f;
			for (int i = 0; i < Crypto::INPUTS_LEN; i++)
				addInput(createInputCentered<PJ301MPort>(Vec(portX(i), py), module, i));
			for (int i = 0; i < Crypto::OUTPUTS_LEN; i++)
				addOutput(createOutputCentered<PJ301MPort>(Vec(portX(Crypto::INPUTS_LEN + i), py), module, i));
		}

		static float knobX(int i) { return 16.f + IN_W / 16.f * ((float)i + 0.5f); }
		static float portX(int i) { return 16.f + IN_W / 17.f * ((float)i + 0.5f); }

		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			Crypto *m = dynamic_cast<Crypto *>(module);

			agmm::panelBase(vg, box.size.x, box.size.y, "CRYPTO");
			agmm::bezel(vg, SCR_X, SCR_Y, SCR_W, SCR_H);
			agmm::screen(vg, SCR_X, SCR_Y, SCR_W, SCR_H);

			// level meters: IN L, IN R, OUT L, OUT R
			const char *ml[4] = {"IL", "IR", "OL", "OR"};
			for (int i = 0; i < 4; i++)
			{
				float x = 668.f + i * 11.f;
				float lv = 0.f;
				if (m)
					lv = std::sqrt(clp(i < 2 ? m->vuIn[i].load() : m->vuOut[i - 2].load(), 0.f, 1.f));
				agmm::meter(vg, x, SCR_Y, 7.f, SCR_H, lv);
				agmm::text(vg, x + 3.5f, SCR_Y + SCR_H + 8.f, ml[i], 7.f, agmm::dim(), agmm::ALIGN_C, 0.f);
			}

			// read-out plate next to the meters
			agmm::plate(vg, 757.f, SCR_Y, 76.f, SCR_H, false);
			char t[64];
			auto readout = [&](float y, const char *label, const char *value)
			{
				agmm::text(vg, 727.f, y, label, 7.f, agmm::dim(), agmm::ALIGN_L, 0.5f);
				agmm::text(vg, 727.f, y + 10.f, value, 8.f, agmm::textCol(), agmm::ALIGN_L, 0.3f);
			};
			std::snprintf(t, sizeof t, "%.1f Hz", m ? m->meanShown[0].load() : 440.f);
			readout(SCR_Y + 12.f, "MEAN L", t);
			std::snprintf(t, sizeof t, "%.1f Hz", m ? m->meanShown[1].load() : 440.f);
			readout(SCR_Y + 42.f, "MEAN R", t);
			std::snprintf(t, sizeof t, "%d", m ? m->grainsShown.load() : 0);
			readout(SCR_Y + 72.f, "GRAINS", t);
			std::snprintf(t, sizeof t, "%.0f k", m ? m->srShown.load() / 1000.f : 48.f);
			readout(SCR_Y + 102.f, "RATE", t);

			// status row
			const bool ext = m ? m->extClock.load() : true;
			const float bpm = m ? m->bpmShown.load() : 120.f;
			const int mode = m ? clp((int)std::round(m->params[P_MODE].getValue()), 0, 2) : 0;
			const int di = m ? clp(m->divShown.load(), 0, DIV_COUNT - 1) : 0;
			float x = 16.f;
			auto chipAt = [&](const char *txt, bool on, NVGcolor col)
			{
				float w = agmm::textWidth(vg, txt, 7.f, 0.5f) + 10.f;
				agmm::chip(vg, x, 168.f, w, 12.f, txt, on, col);
				x += w + 4.f;
			};
			if (ext && bpm <= 0.f)
				chipAt("EXT  NO CLOCK", false, agmm::dim());
			else
			{
				std::snprintf(t, sizeof t, "%s %.1f BPM", ext ? "EXT" : "INT", bpm);
				chipAt(t, true, ext ? agmm::accent() : agmm::warm());
			}
			if (m)
			{
				std::snprintf(t, sizeof t, "N=%u", (unsigned)m->periodShown.load());
				chipAt(t, false, agmm::dim());
				std::snprintf(t, sizeof t, "W=%u %s", (unsigned)m->windowShown.load(), DIV_NAMES[di]);
				chipAt(t, false, agmm::dim());
			}
			static const char *modeNames[3] = {"TONE", "CRYPTO", "CLOUD"};
			chipAt(modeNames[mode], true, mode == MODE_TONE ? agmm::accent() : (mode == MODE_CRYPTO ? agmm::violet() : agmm::warm()));
			if (m && m->params[P_FREEZE].getValue() > 0.5f)
				chipAt("BUFFER FROZEN", true, agmm::bright());
			if (m && m->params[P_HOLD].getValue() > 0.5f)
				chipAt("HELD", true, agmm::warm());
			if (m)
			{
				std::snprintf(t, sizeof t, "TICK %u", (unsigned)m->ticks.load());
				agmm::text(vg, 794.f, 174.f, t, 7.f, agmm::dim(), agmm::ALIGN_R, 0.5f);
			}

			// knob labels + values (dimmed when the knob does nothing in the current mode)
			agmm::text(vg, knobX(0), 261.f, "SAMPLES", 7.f, agmm::dim(), agmm::ALIGN_C, 0.3f);
			agmm::text(vg, knobX(0), 271.f, DIV_NAMES[di], 7.f, agmm::textCol(), agmm::ALIGN_C, 0.3f);
			for (int i = 0; i < K_COUNT; i++)
			{
				const KnobDef &d = KNOBS[i];
				const bool grainOnly = (i == K_SIZE || i == K_DENS || i == K_GLITCH || i == K_SPREAD || i == K_POS || i == K_SHAPE);
				const bool active = !grainOnly || mode != MODE_TONE;
				float cx = knobX(i + 1);
				agmm::text(vg, cx, 261.f, d.label, 7.f, active ? agmm::dim() : agmm::withA(agmm::dim(), 0.45f), agmm::ALIGN_C, 0.3f);
				std::string v = "-";
				if (m)
					v = fmtKnob(d, mapKnob(d, m->params[P_KNOB0 + i].getValue()));
				agmm::text(vg, cx, 271.f, v.c_str(), 7.f,
						   active ? agmm::textCol() : agmm::withA(agmm::textCol(), 0.45f), agmm::ALIGN_C, 0.3f);
			}
			// hairline between the capture group and the processor group
			const float sx = 16.f + IN_W / 16.f * 8.f;
			nvgBeginPath(vg);
			nvgMoveTo(vg, sx, 228.f);
			nvgLineTo(vg, sx, 276.f);
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			// port plates + labels
			const float top = 282.f, h = 58.f;
			const char *names[Crypto::INPUTS_LEN + Crypto::OUTPUTS_LEN] = {
				"IN L", "IN R", "CLK", "V/OCT", "DIV CV", "POS CV", "PCH CV", "SHP CV", "MIX CV", "SPC CV", "FB CV",
				"OUT L", "OUT R", "F L", "F R", "TRIG", "GATE"};
			for (int i = 0; i < Crypto::INPUTS_LEN + Crypto::OUTPUTS_LEN; i++)
				agmm::cell(vg, portX(i), top, 38.f, h, names[i], i >= Crypto::INPUTS_LEN, 7.f);

			ModuleWidget::draw(args);
		}

		void copyLastTick(Crypto *m)
		{
			uint32_t t = m->ticks.load(std::memory_order_acquire);
			if (t == 0)
				return;
			TickRec rec = m->history[(t - 1) % LOG_N];
			int base = clp((int)std::round(m->params[P_BASE].getValue()), 0, 3);
			glfwSetClipboardString(APP->window->win, tickText(rec, base).c_str());
		}

		void appendContextMenu(Menu *menu) override
		{
			Crypto *m = dynamic_cast<Crypto *>(module);
			if (!m)
				return;
			menu->addChild(new MenuSeparator);
			menu->addChild(createSubmenuItem("Internal clock", std::to_string(m->intBpm) + " BPM", [m](Menu *sub)
											 {
                for (int bpm : {40, 60, 80, 90, 100, 110, 120, 128, 140, 150, 174, 200})
                    sub->addChild(createCheckMenuItem(std::to_string(bpm) + " BPM", "",
                                                      [m, bpm]() { return m->intBpm == bpm; },
                                                      [m, bpm]() { m->intBpm = bpm; })); }));
			menu->addChild(createMenuItem("Copy last tick to clipboard", "", [this, m]()
										  { copyLastTick(m); }));
			menu->addChild(createMenuItem("Clear log", "", [m]()
										  { m->clearReq.store(true); }));
		}
	};

} // namespace crypto

Model *modelCrypto = createModel<crypto::Crypto, crypto::CryptoWidget>("Crypto");