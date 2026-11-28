#include "GitLoad.hpp"
// Clockwork.cpp - chainable master clock with six sub-clocks (AGMM).
//
// Features (inspired by Impromptu's Clocked(https://library.vcvrack.com/ImpromptuModular/Clocked), re-implemented from scratch):
//   - Master tempo 30-300 BPM (snap knob), or driven externally from the BPM input:
//       * CV mode:      1V/oct, 0V = 120 BPM
//       * Clock mode:   tempo + phase locked to an external clock at 4/8/12/24 PPQN
//   - Run / Reset buttons + inputs, Reset / Run / BPM outputs for daisy-chaining
//   - Optional "emit reset when run stops"
//   - Six sub-clocks, each with a multiply/divide ratio (x1 ... x64, /1 ... /64,
//     including x1.5 and x2.5), swing, pulse width, delay (fractions of a period),
//     swing + PW CV inputs and a clock output
//   - Ratio changes are applied at the next master bar so everything stays in sync
//   - Phase-based engine: every sub-clock is derived from the master phase and a bar
//     counter, so tempo changes stretch all clocks smoothly with no drift
//
// Visualisations (all drawn with the agmm_ui kit):
//   - Per-clock pattern scope: swing / PW shaped two-period waveform, ghost outline
//     of the undelayed pattern when delay is used, live playhead, frame progress bar
//     for divided clocks and a SYNC flag while a ratio change is pending
//   - Master scope and status chips (RUN / source / RST / LOCK)
//   - Touching a swing / PW / delay knob shows its value on that row's display
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include "Knob_arcs.hpp"

namespace ag = agmm;

// ---------------------------------------------------------------------------
// Shared constants and helpers
// ---------------------------------------------------------------------------
namespace
{
	const int NUM_CH = 7;  // channel 0 = master, 1..6 = sub-clocks
	const int NUM_SUB = 6;

	// 32HP panel
	const float PANEL_W = 480.f;
	const float PANEL_H = 380.f;

	// Layout columns / rows
	const float COL_RATIO = 30.f;
	const float COL_SWING = 278.f;
	const float COL_PW = 304.f;
	const float COL_DELAY = 330.f;
	const float COL_SWCV = 364.f;
	const float COL_PWCV = 396.f;
	const float COL_LED = 420.f;
	const float COL_OUT = 444.f;
	const float MASTER_Y = 48.f;
	inline float rowY(int sub) { return 140.f + 36.f * (float)sub; }

	const float RATIOS[34] = {1, 1.5, 2, 2.5, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
							  16, 17, 19, 23, 24, 29, 31, 32, 37, 41, 43, 47, 48, 53, 59, 61, 64};

	const float DELAY_FRACS[8] = {0.f, 1.f / 16.f, 1.f / 8.f, 1.f / 4.f, 1.f / 3.f, 1.f / 2.f, 2.f / 3.f, 3.f / 4.f};
	const char* const DELAY_SHORT[8] = {"0", "1/16", "1/8", "1/4", "1/3", "1/2", "2/3", "3/4"};

	const int PPQN_TABLE[5] = {0, 4, 8, 12, 24};  // index = bpmMode (0 = CV)

	const double LEN_MIN = 120.0 / 300.0;  // master double-period limits (seconds)
	const double LEN_MAX = 120.0 / 30.0;
	const double LEN_MIN_EXT = 120.0 / 450.0;  // wider range while locking to an external clock
	const double LEN_MAX_EXT = 120.0 / 20.0;

	int gcdInt(int a, int b)
	{
		while (b != 0)
		{
			int t = a % b;
			a = b;
			b = t;
		}
		return a;
	}

	double fracPart(double x) { return x - std::floor(x); }

	// Rack's clamp() only has int and float overloads, so doubles need their own.
	double clampD(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

	// A ratio p/q: p sub double-periods happen every q master double-periods.
	struct Ratio
	{
		int p = 1;
		int q = 1;
	};

	bool sameRatio(const Ratio& a, const Ratio& b) { return a.p == b.p && a.q == b.q; }

	// knob index: 0..33 multiply, -1..-33 divide
	Ratio ratioFromKnob(int idx)
	{
		int n = std::min(std::abs(idx), 33);
		int r2 = (int)(RATIOS[n] * 2.f + 0.5f);  // ratio doubled, always an integer
		Ratio r;
		if (idx >= 0)
		{
			r.p = r2;
			r.q = 2;
		}
		else
		{
			r.p = 2;
			r.q = r2;
		}
		int g = gcdInt(r.p, r.q);
		r.p /= g;
		r.q /= g;
		return r;
	}

	void ratioLabel(int idx, char* buf, size_t n)
	{
		int i = std::min(std::abs(idx), 33);
		int r2 = (int)(RATIOS[i] * 2.f + 0.5f);
		char op = (idx < 0 && i != 0) ? '/' : 'x';
		if (r2 % 2 == 1)
			snprintf(buf, n, "%c%d.5", op, r2 / 2);
		else
			snprintf(buf, n, "%c%d", op, r2 / 2);
	}

	// Pulse geometry for one double period (all in seconds).
	// Two pulses per double period; the second one is displaced by the swing.
	// Pulses and gaps never get shorter than 1 ms so downstream sync never misses an edge.
	struct PulseGeom
	{
		double start2;
		double width;
	};

	PulseGeom pulseGeom(double lenSec, double swing, double pw)
	{
		const double ms = 0.001;
		const double T = lenSec * 0.5;
		double sw = std::max(0.0, T - 2.0 * ms) * swing;
		double wMax = std::max(ms, T - ms - std::fabs(sw));
		PulseGeom g;
		g.width = ms + (wMax - ms) * pw;
		g.start2 = T + sw;
		return g;
	}

	bool gateHigh(double phase, double lenSec, double swing, double pw, double delayPhase)
	{
		double ph = phase - delayPhase;
		if (ph < 0.0)
			ph += 1.0;
		PulseGeom g = pulseGeom(lenSec, swing, pw);
		double t = ph * lenSec;
		return (t < g.width) || (t >= g.start2 && t < g.start2 + g.width);
	}
} // namespace

// ---------------------------------------------------------------------------
// Param quantity showing a readable ratio in tooltips
// ---------------------------------------------------------------------------
struct RatioQuantity : ParamQuantity
{
	std::string getDisplayValueString() override
	{
		char buf[16];
		ratioLabel((int)std::round(getValue()), buf, sizeof(buf));
		return std::string(buf);
	}
	std::string getUnit() override { return ""; }
};

// ---------------------------------------------------------------------------
// Module
// ---------------------------------------------------------------------------
struct Clockwork : Module
{
	enum ParamId
	{
		BPM_PARAM,
		RUN_PARAM,
		RESET_PARAM,
		MODE_PARAM,
		ENUMS(RATIO_PARAMS, 6),
		ENUMS(SWING_PARAMS, 7),
		ENUMS(PW_PARAMS, 7),
		ENUMS(DELAY_PARAMS, 6),
		PARAMS_LEN
	};
	enum InputId
	{
		RESET_INPUT,
		RUN_INPUT,
		BPM_INPUT,
		ENUMS(SWING_INPUTS, 7),
		ENUMS(PW_INPUTS, 7),
		INPUTS_LEN
	};
	enum OutputId
	{
		ENUMS(CLK_OUTPUTS, 7),
		RESET_OUTPUT,
		RUN_OUTPUT,
		BPM_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId
	{
		RESET_LIGHT,
		RUN_LIGHT,
		ENUMS(CLK_LIGHTS, 7),
		LIGHTS_LEN
	};

	// ---- saved state ----
	bool running = true;
	int bpmMode = 0;  // 0 = CV (1V/oct), 1..4 = external clock detect at 4/8/12/24 PPQN
	bool resetOnStop = false;

	// ---- engine state (public so the widgets can visualise it) ----
	double masterPhase = 0.0;  // 0..1 over one master double period
	double masterLen = 1.0;    // seconds per master double period (120 / BPM)
	int bars[NUM_SUB] = {};
	Ratio cur[NUM_SUB];
	Ratio pending[NUM_SUB];
	bool pendingFlag[NUM_SUB] = {};

	float swing[NUM_CH] = {};
	float pw[NUM_CH] = {};
	float delayFrac[NUM_SUB] = {};
	double subPhase[NUM_CH] = {};
	double subLen[NUM_CH] = {};
	bool gateOut[NUM_CH] = {};

	// external clock detection
	int extIndex = -1;
	bool haveEdge = false;
	bool firstInterval = true;
	double edgeTimer = 0.0;
	double lastInterval = 0.25;

	float resetFlash = 0.f;
	bool firstProcess = true;

	dsp::SchmittTrigger runTrig, resetTrig, bpmTrig, modeTrig;
	dsp::PulseGenerator resetPulse, runPulse;
	dsp::ClockDivider ctrlDiv;

	Clockwork()
	{
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

		configParam(BPM_PARAM, 30.f, 300.f, 120.f, "Tempo", " BPM");
		paramQuantities[BPM_PARAM]->snapEnabled = true;
		configButton(RUN_PARAM, "Run / pause");
		configButton(RESET_PARAM, "Reset");
		configButton(MODE_PARAM, "BPM input mode (CV / clock PPQN)");

		std::vector<std::string> delayLabels;
		delayLabels.push_back("None");
		delayLabels.push_back("1/16 period");
		delayLabels.push_back("1/8 period");
		delayLabels.push_back("1/4 period");
		delayLabels.push_back("1/3 period");
		delayLabels.push_back("1/2 period");
		delayLabels.push_back("2/3 period");
		delayLabels.push_back("3/4 period");

		for (int c = 0; c < NUM_CH; c++)
		{
			std::string n = (c == 0) ? "Master" : ("Clock " + std::to_string(c));
			configParam(SWING_PARAMS + c, -1.f, 1.f, 0.f, n + " swing", "%", 0.f, 100.f);
			configParam(PW_PARAMS + c, 0.f, 1.f, 0.5f, n + " pulse width", "%", 0.f, 100.f);
			configInput(SWING_INPUTS + c, n + " swing CV (+/-5V)");
			configInput(PW_INPUTS + c, n + " pulse width CV (0-10V)");
			configOutput(CLK_OUTPUTS + c, n + " clock");
			if (c > 0)
			{
				configParam<RatioQuantity>(RATIO_PARAMS + c - 1, -33.f, 33.f, 0.f, n + " ratio");
				paramQuantities[RATIO_PARAMS + c - 1]->snapEnabled = true;
				configSwitch(DELAY_PARAMS + c - 1, 0.f, 7.f, 0.f, n + " delay", delayLabels);
			}
		}
		configInput(RESET_INPUT, "Reset");
		configInput(RUN_INPUT, "Run toggle");
		configInput(BPM_INPUT, "BPM (1V/oct CV or clock)");
		configOutput(RESET_OUTPUT, "Reset");
		configOutput(RUN_OUTPUT, "Run");
		configOutput(BPM_OUTPUT, "BPM (1V/oct)");

		ctrlDiv.setDivision(16);
		onReset();
	}

	void onReset() override
	{
		running = true;
		for (int i = 0; i < NUM_SUB; i++)
		{
			cur[i] = Ratio();
			pending[i] = Ratio();
			pendingFlag[i] = false;
		}
		masterLen = 1.0;
		firstProcess = true;
		doReset(false);
		haveEdge = false;
	}

	json_t* dataToJson() override
	{
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "running", json_boolean(running));
		json_object_set_new(rootJ, "bpmMode", json_integer(bpmMode));
		json_object_set_new(rootJ, "resetOnStop", json_boolean(resetOnStop));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override
	{
		json_t* j = json_object_get(rootJ, "running");
		if (j)
			running = json_is_true(j);
		j = json_object_get(rootJ, "bpmMode");
		if (j)
			bpmMode = clamp((int)json_integer_value(j), 0, 4);
		j = json_object_get(rootJ, "resetOnStop");
		if (j)
			resetOnStop = json_is_true(j);
		firstProcess = true;
	}

	// -----------------------------------------------------------------------
	void doReset(bool emit)
	{
		masterPhase = 0.0;
		for (int i = 0; i < NUM_SUB; i++)
		{
			bars[i] = 0;
			if (pendingFlag[i])
			{
				cur[i] = pending[i];
				pendingFlag[i] = false;
			}
		}
		extIndex = -1;
		resetFlash = 1.f;
		if (emit)
			resetPulse.trigger(1e-3f);
	}

	void setRunning(bool r)
	{
		running = r;
		runPulse.trigger(1e-3f);
		if (!r && resetOnStop)
			doReset(true);
	}

	// Called at every master bar boundary: advance frame counters, apply ratio changes.
	void advanceBars()
	{
		for (int i = 0; i < NUM_SUB; i++)
		{
			if (pendingFlag[i])
			{
				cur[i] = pending[i];
				pendingFlag[i] = false;
				bars[i] = 0;
			}
			else
			{
				bars[i]++;
				if (bars[i] >= cur[i].q)
					bars[i] = 0;
			}
		}
	}

	// Control-rate: knobs, CVs, ratios, master tempo from knob / CV.
	void updateControls(bool bpmConn)
	{
		for (int c = 0; c < NUM_CH; c++)
		{
			float s = params[SWING_PARAMS + c].getValue();
			if (inputs[SWING_INPUTS + c].isConnected())
				s += inputs[SWING_INPUTS + c].getVoltage() / 5.f;
			swing[c] = clamp(s, -1.f, 1.f);

			float p = params[PW_PARAMS + c].getValue();
			if (inputs[PW_INPUTS + c].isConnected())
				p += inputs[PW_INPUTS + c].getVoltage() / 10.f;
			pw[c] = clamp(p, 0.f, 1.f);
		}

		for (int i = 0; i < NUM_SUB; i++)
		{
			int d = clamp((int)std::round(params[DELAY_PARAMS + i].getValue()), 0, 7);
			delayFrac[i] = DELAY_FRACS[d];

			Ratio r = ratioFromKnob((int)std::round(params[RATIO_PARAMS + i].getValue()));
			if (!running)
			{
				if (!sameRatio(r, cur[i]))
					bars[i] = 0;
				cur[i] = r;
				pendingFlag[i] = false;
			}
			else if (pendingFlag[i])
			{
				if (sameRatio(r, cur[i]))
					pendingFlag[i] = false;
				else
					pending[i] = r;
			}
			else if (!sameRatio(r, cur[i]))
			{
				pending[i] = r;
				pendingFlag[i] = true;
			}
		}

		bool detect = bpmConn && bpmMode > 0;
		if (!detect)
		{
			if (bpmConn)
				masterLen = clampD(1.0 / std::pow(2.0, (double)inputs[BPM_INPUT].getVoltage()), LEN_MIN, LEN_MAX);
			else
				masterLen = 120.0 / clamp(params[BPM_PARAM].getValue(), 30.f, 300.f);
		}
	}

	// Rising edge of the external clock (clock-detect mode).
	void onExtEdge()
	{
		const int ppqn = PPQN_TABLE[clamp(bpmMode, 1, 4)];
		const int n = ppqn * 2;  // edges per master double period

		if (!running)
		{
			running = true;  // an external clock is the only way to start in this mode
			runPulse.trigger(1e-3f);
			doReset(false);
			haveEdge = false;
		}

		if (haveEdge)
		{
			double target = clampD(edgeTimer * n, LEN_MIN_EXT, LEN_MAX_EXT);
			if (firstInterval)
				masterLen = target;
			else
				masterLen += 0.4 * (target - masterLen);
			firstInterval = false;
			lastInterval = edgeTimer;
		}
		else
		{
			firstInterval = true;
			lastInterval = LEN_MAX / (double)n;  // worst-case guess until a real interval is measured
		}
		edgeTimer = 0.0;

		extIndex = (extIndex + 1) % n;
		if (extIndex == 0)
		{
			if (masterPhase > 0.5)  // we had not wrapped yet: do it now
				advanceBars();
			masterPhase = 0.0;
		}
		else
		{
			double expected = (double)extIndex / (double)n;
			double err = expected - masterPhase;
			if (err > 0.5)
				err -= 1.0;
			else if (err < -0.5)
				err += 1.0;
			if (std::fabs(err) < 0.25)
				masterPhase = clampD(masterPhase + 0.5 * err, 0.0, 0.999999);
		}
		haveEdge = true;
	}

	void process(const ProcessArgs& args) override
	{
		const double dt = args.sampleTime;
		const bool bpmConn = inputs[BPM_INPUT].isConnected();
		const bool detect = bpmConn && bpmMode > 0;

		if (firstProcess || ctrlDiv.process())
		{
			updateControls(bpmConn);
			firstProcess = false;
		}

		// Run (toggle). In clock-detect mode only stopping is allowed manually.
		if (runTrig.process(params[RUN_PARAM].getValue() + inputs[RUN_INPUT].getVoltage(), 0.1f, 1.f))
		{
			if (running)
				setRunning(false);
			else if (!detect)
				setRunning(true);
		}

		// Reset
		if (resetTrig.process(params[RESET_PARAM].getValue() + inputs[RESET_INPUT].getVoltage(), 0.1f, 1.f))
			doReset(true);

		// BPM mode button: CV -> 4 -> 8 -> 12 -> 24 PPQN -> CV
		if (modeTrig.process(params[MODE_PARAM].getValue(), 0.1f, 1.f))
		{
			bpmMode = (bpmMode + 1) % 5;
			haveEdge = false;
			extIndex = -1;
		}

		// External clock detection
		if (detect)
		{
			if (haveEdge)
				edgeTimer += dt;
			if (bpmTrig.process(inputs[BPM_INPUT].getVoltage(), 0.1f, 2.f))
				onExtEdge();
			if (running && haveEdge && edgeTimer > lastInterval * 2.5 + 0.1)
			{
				setRunning(false);
				haveEdge = false;
				extIndex = -1;
			}
		}
		else
		{
			bpmTrig.reset();
			haveEdge = false;
		}

		// Master phase
		if (running)
		{
			masterPhase += dt / masterLen;
			if (masterPhase >= 1.0)
			{
				masterPhase -= 1.0;
				if (masterPhase >= 1.0)
					masterPhase = 0.0;
				advanceBars();
			}
		}

		// Clocks
		subPhase[0] = masterPhase;
		subLen[0] = masterLen;
		for (int i = 0; i < NUM_SUB; i++)
		{
			double u = (double)bars[i] + masterPhase;
			subPhase[i + 1] = fracPart(u * (double)cur[i].p / (double)cur[i].q);
			subLen[i + 1] = masterLen * (double)cur[i].q / (double)cur[i].p;
		}

		for (int c = 0; c < NUM_CH; c++)
		{
			bool high = false;
			if (running)
			{
				double d = (c > 0) ? (double)delayFrac[c - 1] * 0.5 : 0.0;
				high = gateHigh(subPhase[c], subLen[c], swing[c], pw[c], d);
			}
			gateOut[c] = high;
			outputs[CLK_OUTPUTS + c].setVoltage(high ? 10.f : 0.f);
			lights[CLK_LIGHTS + c].setSmoothBrightness(high ? 1.f : 0.f, (float)dt);
		}

		// Chaining outputs
		outputs[RESET_OUTPUT].setVoltage(resetPulse.process((float)dt) ? 10.f : 0.f);
		outputs[RUN_OUTPUT].setVoltage(runPulse.process((float)dt) ? 10.f : 0.f);
		outputs[BPM_OUTPUT].setVoltage(bpmConn ? inputs[BPM_INPUT].getVoltage() : -std::log2((float)masterLen));

		// Lights
		lights[RUN_LIGHT].setBrightness(running ? 1.f : 0.f);
		lights[RESET_LIGHT].setSmoothBrightness(resetFlash, (float)dt);
		resetFlash = std::max(0.f, resetFlash - (float)dt * 6.f);
	}
};

// ---------------------------------------------------------------------------
// Widgets
// ---------------------------------------------------------------------------

// Value shown on a row's display while one of its knobs is touched.
static bool touchedInfo(Clockwork* m, int ch, char* buf, size_t n)
{
	if (!m || !APP || !APP->scene || !APP->scene->rack)
		return false;
	ParamWidget* w = APP->scene->rack->getTouchedParam();
	if (!w || w->module != m)
		return false;
	int id = w->paramId;
	if (id == Clockwork::SWING_PARAMS + ch)
	{
		snprintf(buf, n, "S%+d", (int)std::round(m->params[id].getValue() * 100.f));
		return true;
	}
	if (id == Clockwork::PW_PARAMS + ch)
	{
		snprintf(buf, n, "PW%d", (int)std::round(m->params[id].getValue() * 100.f));
		return true;
	}
	if (ch > 0 && id == Clockwork::DELAY_PARAMS + ch - 1)
	{
		int d = clamp((int)std::round(m->params[id].getValue()), 0, 7);
		snprintf(buf, n, "D%s", DELAY_SHORT[d]);
		return true;
	}
	return false;
}

// Static panel artwork
struct ClockworkPanel : TransparentWidget
{
	void draw(const DrawArgs& args) override
	{
		NVGcontext* vg = args.vg;
		const float W = PANEL_W, H = PANEL_H;
		ag::panelBase(vg, W, H, "CLOCKWORK");

		// Master block
		ag::plate(vg, W / 2.f, 26.f, W - 16.f, 82.f);
		ag::plate(vg, COL_OUT, 34.f, 40.f, 36.f, true);
		ag::bezel(vg, 198.f, 34.f, 58.f, 30.f);
		ag::screen(vg, 198.f, 34.f, 58.f, 30.f);

		// master labels
		ag::text(vg, 24.f, 66.f, "RESET", 7.f, ag::dim());
		ag::text(vg, 64.f, 66.f, "RUN", 7.f, ag::dim());
		ag::text(vg, 104.f, 66.f, "BPM IN", 7.f, ag::dim());
		ag::text(vg, 24.f, 106.f, "RST", 7.f, ag::dim());
		ag::text(vg, 64.f, 106.f, "RUN", 7.f, ag::dim());
		ag::text(vg, 104.f, 106.f, "MODE", 7.f, ag::dim());
		ag::text(vg, 160.f, 106.f, "TEMPO", 7.f, ag::dim());
		ag::text(vg, COL_SWING, 66.f, "SWING", 7.f, ag::dim());
		ag::text(vg, COL_PW, 66.f, "PW", 7.f, ag::dim());
		ag::text(vg, COL_SWCV, 66.f, "SWG CV", 7.f, ag::dim());
		ag::text(vg, COL_PWCV, 66.f, "PW CV", 7.f, ag::dim());
		ag::text(vg, COL_OUT, 66.f, "MASTER", 7.f, ag::accent());

		// Sub-clock column headers
		const float hy = 116.f;
		ag::text(vg, COL_RATIO + 8.f, hy, "RATIO", 7.f, ag::dim());
		ag::text(vg, 178.f, hy, "PATTERN  (2 PERIODS)", 7.f, ag::dim(), ag::ALIGN_C, 1.f);
		ag::text(vg, COL_SWING, hy, "SWING", 7.f, ag::dim());
		ag::text(vg, COL_PW, hy, "PW", 7.f, ag::dim());
		ag::text(vg, COL_DELAY, hy, "DELAY", 7.f, ag::dim());
		ag::text(vg, COL_SWCV, hy, "SWG CV", 7.f, ag::dim());
		ag::text(vg, COL_PWCV, hy, "PW CV", 7.f, ag::dim());
		ag::text(vg, COL_OUT, hy, "OUT", 7.f, ag::accent());

		// Sub rows
		for (int i = 0; i < NUM_SUB; i++)
		{
			float cy = rowY(i);
			ag::plate(vg, W / 2.f, cy - 17.f, W - 16.f, 34.f);
			ag::plate(vg, COL_OUT, cy - 15.f, 40.f, 30.f, true);
			char num[4];
			snprintf(num, sizeof(num), "%d", i + 1);
			ag::text(vg, 14.f, cy, num, 8.f, ag::accent());
			ag::bezel(vg, 50.f, cy - 9.f, 38.f, 18.f);
			ag::screen(vg, 50.f, cy - 9.f, 38.f, 18.f);
		}

		// Bottom row: chain outputs
		ag::plate(vg, 108.f, 339.f, 198.f, 26.f, true);
		ag::text(vg, 40.f, 352.f, "RESET", 7.f, ag::dim(), ag::ALIGN_L);
		ag::text(vg, 110.f, 352.f, "RUN", 7.f, ag::dim(), ag::ALIGN_L);
		ag::text(vg, 180.f, 352.f, "BPM", 7.f, ag::dim(), ag::ALIGN_L);
	}
};

// BPM / ratio display
struct ClockworkDisplay : TransparentWidget
{
	Clockwork* module = nullptr;
	int ch = 0;

	void draw(const DrawArgs& args) override
	{
		NVGcontext* vg = args.vg;
		char buf[16];
		float size = (ch == 0) ? 20.f : 13.f;
		NVGcolor col = ag::accent();

		if (!module)
		{
			snprintf(buf, sizeof(buf), ch == 0 ? "120" : "x2");
		}
		else if (touchedInfo(module, ch, buf, sizeof(buf)))
		{
			col = ag::warm();
			size = (ch == 0) ? 16.f : 11.f;
		}
		else if (ch == 0)
		{
			snprintf(buf, sizeof(buf), "%3d", (int)std::round(120.0 / module->masterLen));
		}
		else
		{
			ratioLabel((int)std::round(module->params[Clockwork::RATIO_PARAMS + ch - 1].getValue()), buf, sizeof(buf));
			if (module->pendingFlag[ch - 1])
				col = ag::violet();  // ratio change waiting for next bar
		}
		ag::text(vg, box.size.x / 2.f, box.size.y / 2.f + 1.f, buf, size, col, ag::ALIGN_C, 1.f);
	}
};

// Pattern scope: one double period, shaped by swing / PW / delay, with live playhead
struct ClockworkScope : TransparentWidget
{
	Clockwork* module = nullptr;
	int ch = 0;

	void draw(const DrawArgs& args) override
	{
		NVGcontext* vg = args.vg;
		const float w = box.size.x, h = box.size.y;
		ag::plotFrame(vg, 0.f, 0.f, w, h);

		double swing = 0.0, pw = 0.5, len = 0.5, phase = 0.0, delayFrac = 0.0, frame = 0.0;
		int q = 1;
		bool pending = false, high = false, running = true;

		if (module)
		{
			swing = module->swing[ch];
			pw = module->pw[ch];
			len = std::max(module->subLen[ch], 1e-4);
			phase = module->subPhase[ch];
			high = module->gateOut[ch];
			running = module->running;
			if (ch > 0)
			{
				delayFrac = module->delayFrac[ch - 1];
				q = module->cur[ch - 1].q;
				pending = module->pendingFlag[ch - 1];
				frame = ((double)module->bars[ch - 1] + module->masterPhase) / (double)q;
			}
		}
		else
		{
			// library / browser preview
			swing = 0.15 * (double)(ch % 3) - 0.1;
			pw = 0.3 + 0.08 * (double)ch;
			len = 0.5;
			phase = fracPart(0.15 + 0.11 * (double)ch);
			delayFrac = (ch % 2) ? 0.25 : 0.0;
			q = (ch % 3 == 0) ? 4 : 1;
			frame = 0.4;
		}

		PulseGeom g = pulseGeom(len, swing, pw);
		const float s1 = 0.f;
		const float s2 = (float)(g.start2 / len);
		const float wd = (float)(g.width / len);
		const float d = (float)(delayFrac * 0.5);

		const float yTop = 5.f;
		const float yBase = h - 7.f;

		// grid: quarter lines, stronger at the period boundary
		for (int k = 1; k < 4; k++)
		{
			nvgBeginPath(vg);
			nvgMoveTo(vg, w * (float)k / 4.f, 2.f);
			nvgLineTo(vg, w * (float)k / 4.f, h - 2.f);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, k == 2 ? 34 : 14));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
		}
		// baseline
		nvgBeginPath(vg);
		nvgMoveTo(vg, 0.f, yBase);
		nvgLineTo(vg, w, yBase);
		nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
		nvgStrokeWidth(vg, 1.f);
		nvgStroke(vg);

		NVGcolor trace = (ch == 0) ? ag::bright() : ag::accent();

		// ghost (undelayed) pattern
		if (d > 0.f)
		{
			drawPulse(vg, w, yTop, yBase, s1, wd, 0.f, false, ag::dim());
			drawPulse(vg, w, yTop, yBase, s2, wd, 0.f, false, ag::dim());
		}
		// actual (delayed) pattern
		drawPulse(vg, w, yTop, yBase, s1, wd, d, true, trace);
		drawPulse(vg, w, yTop, yBase, s2, wd, d, true, trace);

		// playhead
		if (running)
		{
			float x = (float)phase * w;
			nvgBeginPath(vg);
			nvgMoveTo(vg, x, 2.f);
			nvgLineTo(vg, x, h - 2.f);
			nvgStrokeColor(vg, high ? ag::bright() : ag::withA(ag::textCol(), 0.55f));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
			nvgBeginPath(vg);
			nvgCircle(vg, x, 3.f, 1.8f);
			nvgFillColor(vg, high ? ag::bright() : ag::textCol());
			nvgFill(vg);
		}

		// frame progress (divided / odd ratios only)
		if (q > 1)
		{
			nvgBeginPath(vg);
			nvgRect(vg, 0.f, h - 3.f, w, 2.f);
			nvgFillColor(vg, nvgRGBA(255, 255, 255, 18));
			nvgFill(vg);
			nvgBeginPath(vg);
			nvgRect(vg, 0.f, h - 3.f, (float)clampD(frame, 0.0, 1.0) * w, 2.f);
			nvgFillColor(vg, ag::withA(ag::violet(), 0.9f));
			nvgFill(vg);
		}

		if (pending)
			ag::text(vg, w - 4.f, 8.f, "SYNC", 7.f, ag::violet(), ag::ALIGN_R, 1.f);
	}

	// Draws one pulse starting at fraction `start`, shifted by `shift`, wrapping at 1.
	static void drawPulse(NVGcontext* vg, float w, float yTop, float yBase, float start, float width, float shift,
						  bool filled, NVGcolor col)
	{
		float x0 = start + shift;
		x0 -= std::floor(x0);
		float x1 = x0 + width;
		if (x1 <= 1.f)
		{
			rectSeg(vg, w, yTop, yBase, x0, x1, filled, col);
		}
		else
		{
			rectSeg(vg, w, yTop, yBase, x0, 1.f, filled, col);
			rectSeg(vg, w, yTop, yBase, 0.f, x1 - 1.f, filled, col);
		}
	}

	static void rectSeg(NVGcontext* vg, float w, float yTop, float yBase, float a, float b, bool filled, NVGcolor col)
	{
		float x = a * w;
		float rw = std::max((b - a) * w, 1.f);
		nvgBeginPath(vg);
		nvgRect(vg, x, yTop, rw, yBase - yTop);
		if (filled)
		{
			nvgFillColor(vg, ag::withA(col, 0.30f));
			nvgFill(vg);
			nvgStrokeColor(vg, ag::withA(col, 0.95f));
		}
		else
		{
			nvgStrokeColor(vg, ag::withA(col, 0.45f));
		}
		nvgStrokeWidth(vg, 1.f);
		nvgStroke(vg);
	}
};

// Status chips next to the master scope (2 x 2 grid, kept inside the master plate)
struct ClockworkChips : TransparentWidget
{
	Clockwork* module = nullptr;

	static constexpr float CHIP_W = 35.f;
	static constexpr float CHIP_H = 12.f;
	static constexpr float CHIP_GAP = 4.f;

	void draw(const DrawArgs& args) override
	{
		NVGcontext* vg = args.vg;
		bool running = true, conn = false, lock = false, rst = false;
		char modeTxt[16] = "KNOB";

		if (module)
		{
			running = module->running;
			conn = module->inputs[Clockwork::BPM_INPUT].isConnected();
			rst = module->resetFlash > 0.05f;
			if (conn)
			{
				if (module->bpmMode == 0)
					snprintf(modeTxt, sizeof(modeTxt), "CV");
				else
				{
					snprintf(modeTxt, sizeof(modeTxt), "EXT P%d", PPQN_TABLE[module->bpmMode]);
					lock = module->running && module->haveEdge;
				}
			}
		}
		const float x2 = CHIP_W + CHIP_GAP;
		const float y2 = CHIP_H + CHIP_GAP;
		ag::chip(vg, 0.f, 0.f, CHIP_W, CHIP_H, "RUN", running, ag::accent());
		ag::chip(vg, x2, 0.f, CHIP_W, CHIP_H, modeTxt, conn, ag::violet());
		ag::chip(vg, 0.f, y2, CHIP_W, CHIP_H, "RST", rst, ag::warm());
		ag::chip(vg, x2, y2, CHIP_W, CHIP_H, "LOCK", lock, ag::bright());
	}
};

struct ClockworkWidget : ModuleWidget
{
	ClockworkWidget(Clockwork* module)
	{
		setModule(module);
		box.size = Vec(PANEL_W, PANEL_H);  // panel is drawn in code (agmm flat style), no SVG needed

		ClockworkPanel* bg = new ClockworkPanel();
		bg->box.size = box.size;
		addChild(bg);

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		// ---- master block ----
		addInput(createInputCentered<PJ301MPort>(Vec(24.f, MASTER_Y), module, Clockwork::RESET_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(64.f, MASTER_Y), module, Clockwork::RUN_INPUT));
		addInput(createInputCentered<PJ301MPort>(Vec(104.f, MASTER_Y), module, Clockwork::BPM_INPUT));

		addParam(createLightParamCentered<VCVLightBezel<ag::CyanLight>>(Vec(24.f, 88.f), module, Clockwork::RESET_PARAM, Clockwork::RESET_LIGHT));
		addParam(createLightParamCentered<VCVLightBezel<ag::CyanLight>>(Vec(64.f, 88.f), module, Clockwork::RUN_PARAM, Clockwork::RUN_LIGHT));
		addParam(createParamCentered<TL1105>(Vec(104.f, 88.f), module, Clockwork::MODE_PARAM));

		addParam(createParamCentered<RoundBigBlackKnob>(Vec(160.f, 70.f), module, Clockwork::BPM_PARAM));

		ClockworkDisplay* bpmDisp = new ClockworkDisplay();
		bpmDisp->box.pos = Vec(198.f, 34.f);
		bpmDisp->box.size = Vec(58.f, 30.f);
		bpmDisp->module = module;
		bpmDisp->ch = 0;
		addChild(bpmDisp);

		addParam(createParamCentered<Trimpot>(Vec(COL_SWING, MASTER_Y), module, Clockwork::SWING_PARAMS + 0));
		addParam(createParamCentered<Trimpot>(Vec(COL_PW, MASTER_Y), module, Clockwork::PW_PARAMS + 0));
		addInput(createInputCentered<PJ301MPort>(Vec(COL_SWCV, MASTER_Y), module, Clockwork::SWING_INPUTS + 0));
		addInput(createInputCentered<PJ301MPort>(Vec(COL_PWCV, MASTER_Y), module, Clockwork::PW_INPUTS + 0));
		addChild(createLightCentered<SmallLight<ag::CyanLight>>(Vec(COL_LED, MASTER_Y), module, Clockwork::CLK_LIGHTS + 0));
		addOutput(createOutputCentered<PJ301MPort>(Vec(COL_OUT, MASTER_Y), module, Clockwork::CLK_OUTPUTS + 0));

		ClockworkScope* mScope = new ClockworkScope();
		mScope->box.pos = Vec(198.f, 74.f);
		mScope->box.size = Vec(184.f, 28.f);
		mScope->module = module;
		mScope->ch = 0;
		addChild(mScope);

		// chips: 2 x 35px + 4px gap = 74px wide, x = 390..464 (master plate ends at x = 472)
		ClockworkChips* chips = new ClockworkChips();
		chips->box.pos = Vec(390.f, 74.f);
		chips->box.size = Vec(74.f, 28.f);
		chips->module = module;
		addChild(chips);

		// ---- sub-clock rows ----
		for (int i = 0; i < NUM_SUB; i++)
		{
			const float cy = rowY(i);
			const int ch = i + 1;

			addParam(createParamCentered<RoundSmallBlackKnob>(Vec(COL_RATIO, cy), module, Clockwork::RATIO_PARAMS + i));

			ClockworkDisplay* d = new ClockworkDisplay();
			d->box.pos = Vec(50.f, cy - 9.f);
			d->box.size = Vec(38.f, 18.f);
			d->module = module;
			d->ch = ch;
			addChild(d);

			ClockworkScope* s = new ClockworkScope();
			s->box.pos = Vec(98.f, cy - 13.f);
			s->box.size = Vec(160.f, 26.f);
			s->module = module;
			s->ch = ch;
			addChild(s);

			addParam(createParamCentered<Trimpot>(Vec(COL_SWING, cy), module, Clockwork::SWING_PARAMS + ch));
			addParam(createParamCentered<Trimpot>(Vec(COL_PW, cy), module, Clockwork::PW_PARAMS + ch));
			addParam(createParamCentered<Trimpot>(Vec(COL_DELAY, cy), module, Clockwork::DELAY_PARAMS + i));
			addInput(createInputCentered<PJ301MPort>(Vec(COL_SWCV, cy), module, Clockwork::SWING_INPUTS + ch));
			addInput(createInputCentered<PJ301MPort>(Vec(COL_PWCV, cy), module, Clockwork::PW_INPUTS + ch));
			addChild(createLightCentered<SmallLight<ag::CyanLight>>(Vec(COL_LED, cy), module, Clockwork::CLK_LIGHTS + ch));
			addOutput(createOutputCentered<PJ301MPort>(Vec(COL_OUT, cy), module, Clockwork::CLK_OUTPUTS + ch));
		}
		agmm::addKnobArcs(this, 2.f);
		// ---- bottom row ----
		addOutput(createOutputCentered<PJ301MPort>(Vec(24.f, 352.f), module, Clockwork::RESET_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(94.f, 352.f), module, Clockwork::RUN_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(Vec(164.f, 352.f), module, Clockwork::BPM_OUTPUT));
	}

	void appendContextMenu(Menu* menu) override
	{
	    agmm::addGitLink(menu); // adds "Visit my git repo!"
	    Clockwork* m = dynamic_cast<Clockwork*>(module);
		if (!m)
			return;
		menu->addChild(new MenuSeparator);
		menu->addChild(createBoolPtrMenuItem("Emit reset when run stops", "", &m->resetOnStop));
		std::vector<std::string> modes;
		modes.push_back("CV (1V/oct, 0V = 120 BPM)");
		modes.push_back("Clock detect, 4 PPQN");
		modes.push_back("Clock detect, 8 PPQN");
		modes.push_back("Clock detect, 12 PPQN");
		modes.push_back("Clock detect, 24 PPQN");
		menu->addChild(createIndexSubmenuItem("BPM input mode", modes,
			[=]() { return m->bpmMode; },
			[=](int i) {
				m->bpmMode = i;
				m->haveEdge = false;
				m->extIndex = -1;
			}));
	}
};

Model* modelClockwork = createModel<Clockwork, ClockworkWidget>("Clockwork");