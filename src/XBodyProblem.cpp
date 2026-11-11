// ============================================================================
// XBodyProblem — N-body gravity LFO (28 HP Rack module)
// ----------------------------------------------------------------------------
// Overview:
//   * Simulates 2..10 point masses in a unit square under mutual gravity.
//     The simulation is driven as a polyphonic LFO: per-mass X, Y, speed
//     and wall-event triggers are output as separate poly channels.
//   * MASSES    number of bodies (2..10)
//     SPEED     simulation speed multiplier (exp2, +/-4 octaves)
//     GRAVITY   gravitational constant multiplier (exp2, +/-4 octaves)
//     RANGE     output voltage mapping (0..1, -1..1, -5..5, 0..10, -10..10)
//     SOFTEN    softening length (close-encounter smoothing)
//     TRAIL     trail length in the display (0..1)
//     WALLS     wrap-around (Pac-Man) or bounce
//     MASSES    equal masses or random per-body masses
//   * RESET / KICK buttons (with a RESET jack) restart or randomise
//     velocities. Simulation state persists through patch reload.
//   * Big square display shows bodies, velocity vectors, gravity links,
//     trails, wall style, centre of mass (bounce mode), and readouts.
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <string>

// ------------------------------------------------------------
// Constants
// ------------------------------------------------------------
static const int XB_MAXN = 10;
static const int XB_TN = 120; // trail points
static const float XB_TRAIL_HZ = 60.f;

static const float XB_PANEL_W = RACK_GRID_WIDTH * 28; // 420
static const float XB_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Square display (220 x 220) plus a 32 px readout strip underneath; the bezel adds 4 px.
static const float XB_S = 220.f;
static const float XB_RO_H = 32.f;
static const float XB_DX = (XB_PANEL_W - XB_S) / 2.f; // 100
static const float XB_DY = 26.f;

// Side knob cells: three on each side of the square. 50 x 52, label at +10, knob at +32
static const float XB_SIDE_W = 50.f, XB_SIDE_H = 52.f;
static const float XB_LKX = 48.f, XB_RKX = 372.f;
static const float XB_SIDE_TOP[3] = {40.f, 110.f, 180.f};
static const float XB_KY[3] = {72.f, 142.f, 212.f}; // SIDE_TOP + 32

// I/O row: nine 38 x 50 cells, label at +10, control at +32 (ends at y = 348)
static const float XB_IO_HDR = 290.f;
static const float XB_IO_TOP = 298.f;
static const float XB_IO_W = 38.f, XB_IO_H = 50.f;
static const float XB_IO_Y = XB_IO_TOP + 32.f;
static inline float xbIoX(int i) { return 40.f + 42.5f * (float)i; }

// ------------------------------------------------------------
// Helpers shared by DSP and display
// ------------------------------------------------------------
static const char *const XB_RANGE_NAMES[5] = {"0 to 1", "-1 to 1", "-5 to 5", "0 to 10", "-10 to 10"};

// normalised position 0..1 -> output voltage for the chosen range
static inline float xbMap(float v, int range)
{
	switch (range)
	{
	case 0:
		return v;
	case 1:
		return 2.f * v - 1.f;
	case 2:
		return 10.f * v - 5.f;
	case 3:
		return 10.f * v;
	default:
		return 20.f * v - 10.f;
	}
}

// ------------------------------------------------------------
// Module — N-body simulation + polyphonic LFO.
//
// All bodies live in a unit square (x,y in [0,1], +y up). Forces come
// from soft Newtonian gravity: a = G * (m / (r^2 + eps^2)) * unit(r),
// with the acceleration capped per body to keep close encounters tame.
//
// Integration is leapfrog (kick-drift-kick) with up to 16 substeps per
// sample so `SPEED` can go very high without losing stability. A simple
// NaN guard resets the system if anything blows up.
// ------------------------------------------------------------
struct XBodyProblem : Module
{
	enum ParamId
	{
		MASSES_PARAM,
		SPEED_PARAM,
		GRAVITY_PARAM,
		RANGE_PARAM,
		SOFT_PARAM,
		TRAIL_PARAM,
		WALL_PARAM,
		MASSMODE_PARAM,
		RESET_PARAM,
		KICK_PARAM,
		PARAMS_LEN
	};
	enum InputId
	{
		RESET_INPUT,
		INPUTS_LEN
	};
	enum OutputId
	{
		X_OUTPUT,
		Y_OUTPUT,
		VEL_OUTPUT,
		EVENT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId
	{
		RESET_LIGHT,
		KICK_LIGHT,
		LIGHTS_LEN
	};

	// --------------------------------------------------------
	// Simulation state (unit square, +y up).
	// --------------------------------------------------------
	float px[XB_MAXN] = {}, py[XB_MAXN] = {}; // positions
	float vx[XB_MAXN] = {}, vy[XB_MAXN] = {}; // velocities
	float ax[XB_MAXN] = {}, ay[XB_MAXN] = {}; // accelerations
	float rm[XB_MAXN] = {};					  // random masses (used when MASSMODE = random)
	int n = 2;								  // current number of bodies
	bool needAcc = true;					  // recompute accelerations on next step

	// --------------------------------------------------------
	// Settings (saved with the patch).
	// --------------------------------------------------------
	bool paused = false;
	bool zeroMomentum = true; // subtract bulk momentum on reset

	dsp::SchmittTrigger resetBtnTrig, resetJackTrig, kickTrig;
	dsp::PulseGenerator evPulse[XB_MAXN];

	// --------------------------------------------------------
	// Display data (written by the audio thread, read by the UI thread;
	// a torn read is only cosmetic).
	// --------------------------------------------------------
	float trX[XB_MAXN][XB_TN] = {}; // trails (unit square)
	float trY[XB_MAXN][XB_TN] = {};
	int trHead = 0, trCount = 0; // trail ring write head / count
	float trTimer = 0.f;		 // time since last trail point

	// Last-frame readouts.
	float vG = 0.05f, vEps = 0.03f, vSpeed = 1.f, vKE = 0.f;
	bool vWrap = true, vRandMass = false;
	int vRange = 2;

	float resetFlash = 0.f, kickFlash = 0.f, eventFlash[XB_MAXN] = {}; // LED / event decays

	// --------------------------------------------------------
	// Constructor — configure params, inputs, outputs, lights;
	// then take an initial random start.
	// --------------------------------------------------------
	XBodyProblem()
	{
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

		configParam(MASSES_PARAM, 2.f, 10.f, 2.f, "Number of masses");
		paramQuantities[MASSES_PARAM]->snapEnabled = true;
		configParam(SPEED_PARAM, -4.f, 4.f, 0.f, "Simulation speed", "x", 2.f, 1.f);
		configParam(GRAVITY_PARAM, -4.f, 4.f, 0.f, "Gravity strength", "x", 2.f, 1.f);
		configSwitch(RANGE_PARAM, 0.f, 4.f, 2.f, "Output range",
					 {"0 to 1", "-1 to 1", "-5 to 5", "0 to 10", "-10 to 10"});
		configParam(SOFT_PARAM, 0.f, 1.f, 0.4f, "Softening (close-encounter smoothing)", "%", 0.f, 100.f);
		configParam(TRAIL_PARAM, 0.f, 1.f, 0.6f, "Trail length", "%", 0.f, 100.f);
		configSwitch(WALL_PARAM, 0.f, 1.f, 0.f, "Walls", {"Wrap around (Pac-Man)", "Bounce"});
		configSwitch(MASSMODE_PARAM, 0.f, 1.f, 0.f, "Masses", {"Equal", "Random"});
		configButton(RESET_PARAM, "Reset (new random start)");
		configButton(KICK_PARAM, "Kick (random impulse)");

		configInput(RESET_INPUT, "Reset trigger");
		configOutput(X_OUTPUT, "X position (polyphonic)");
		configOutput(Y_OUTPUT, "Y position (polyphonic)");
		configOutput(VEL_OUTPUT, "Speed 0-10 V (polyphonic)");
		configOutput(EVENT_OUTPUT, "Wall event trigger (polyphonic)");

		doReset();
	}

	// Uniform random in [0, 1).
	static float rnd() { return rack::random::uniform(); }

	// Per-body mass: 1 when equal masses, or the stored random mass.
	float massOf(int i) const { return vRandMass ? rm[i] : 1.f; }

	// Random position / velocity / mass for body i (keeps clear of the edges).
	void initBody(int i)
	{
		px[i] = 0.1f + 0.8f * rnd();
		py[i] = 0.1f + 0.8f * rnd();
		vx[i] = (rnd() - 0.5f) * 0.4f;
		vy[i] = (rnd() - 0.5f) * 0.4f;
		rm[i] = 0.4f + 2.f * rnd();
	}

	// --------------------------------------------------------
	// Fresh random start. Also (optionally) removes bulk momentum
	// so the system doesn't drift as a whole, then clears the trail.
	// --------------------------------------------------------
	void doReset()
	{
		for (int i = 0; i < XB_MAXN; ++i)
			initBody(i);
		if (zeroMomentum)
		{
			float mx = 0.f, my = 0.f, mt = 0.f;
			for (int i = 0; i < n; ++i)
			{
				float m = massOf(i);
				mx += m * vx[i];
				my += m * vy[i];
				mt += m;
			}
			if (mt > 0.f)
				for (int i = 0; i < n; ++i)
				{
					vx[i] -= mx / mt;
					vy[i] -= my / mt;
				}
		}
		std::memset(trX, 0, sizeof(trX));
		std::memset(trY, 0, sizeof(trY));
		trHead = trCount = 0;
		needAcc = true;
	}

	// Rack reset: unpause and start a new random system.
	void onReset(const ResetEvent &e) override
	{
		Module::onReset(e);
		paused = false;
		zeroMomentum = true;
		doReset();
	}

	// --------------------------------------------------------
	// Compute pairwise gravitational accelerations.
	//   * wrap: use the minimum-image distance (Pac-Man topology)
	//   * soft: add eps^2 in the denominator to avoid singularities
	//   * each body's acceleration magnitude is capped at 400 so a
	//     close encounter can't blow the integrator up
	// --------------------------------------------------------
	void computeAcc(bool wrap, float G, float eps2)
	{
		for (int i = 0; i < n; ++i)
			ax[i] = ay[i] = 0.f;
		for (int i = 0; i < n; ++i)
		{
			for (int j = i + 1; j < n; ++j)
			{
				float dx = px[j] - px[i], dy = py[j] - py[i];
				if (wrap)
				{
					// Minimum-image: move the pair to the closest copy.
					dx -= std::round(dx);
					dy -= std::round(dy);
				}
				float r2 = dx * dx + dy * dy + eps2;
				float f = G / (r2 * std::sqrt(r2)); // G / r^3
				float mi = massOf(i), mj = massOf(j);
				ax[i] += f * mj * dx;
				ay[i] += f * mj * dy;
				ax[j] -= f * mi * dx;
				ay[j] -= f * mi * dy;
			}
		}
		for (int i = 0; i < n; ++i)
		{ // keep extreme encounters tame
			float a = std::sqrt(ax[i] * ax[i] + ay[i] * ay[i]);
			if (a > 400.f)
			{
				ax[i] *= 400.f / a;
				ay[i] *= 400.f / a;
			}
		}
	}

	// --------------------------------------------------------
	// One leapfrog (kick-drift-kick) step:
	//   half-kick -> drift -> (recompute acc) -> half-kick
	// Position updates apply wrap or bounce and set ev[i] when a
	// wall is hit. Per-body speed is capped at 3 units/sec.
	// --------------------------------------------------------
	void physicsStep(float h, bool wrap, float G, float eps2, bool *ev)
	{
		for (int i = 0; i < n; ++i)
		{
			// First half-kick, speed cap, then drift.
			vx[i] += ax[i] * 0.5f * h;
			vy[i] += ay[i] * 0.5f * h;
			float sp = std::sqrt(vx[i] * vx[i] + vy[i] * vy[i]);
			if (sp > 3.f)
			{
				vx[i] *= 3.f / sp;
				vy[i] *= 3.f / sp;
			}
			px[i] += vx[i] * h;
			py[i] += vy[i] * h;
			if (wrap)
			{
				// Wrap: just fold the coordinate back into [0,1).
				if (px[i] < 0.f || px[i] >= 1.f)
				{
					px[i] -= std::floor(px[i]);
					ev[i] = true;
				}
				if (py[i] < 0.f || py[i] >= 1.f)
				{
					py[i] -= std::floor(py[i]);
					ev[i] = true;
				}
			}
			else
			{
				// Bounce: reflect the position and flip the velocity.
				if (px[i] < 0.f)
				{
					px[i] = -px[i];
					vx[i] = -vx[i];
					ev[i] = true;
				}
				else if (px[i] > 1.f)
				{
					px[i] = 2.f - px[i];
					vx[i] = -vx[i];
					ev[i] = true;
				}
				if (py[i] < 0.f)
				{
					py[i] = -py[i];
					vy[i] = -vy[i];
					ev[i] = true;
				}
				else if (py[i] > 1.f)
				{
					py[i] = 2.f - py[i];
					vy[i] = -vy[i];
					ev[i] = true;
				}
				// Defensive clamp in case the reflection overshoots.
				px[i] = rack::math::clamp(px[i], 0.f, 1.f);
				py[i] = rack::math::clamp(py[i], 0.f, 1.f);
			}
		}
		computeAcc(wrap, G, eps2);
		for (int i = 0; i < n; ++i)
		{
			// Second half-kick.
			vx[i] += ax[i] * 0.5f * h;
			vy[i] += ay[i] * 0.5f * h;
		}
	}

	// --------------------------------------------------------
	// Main per-sample processing:
	//  1. Read params, adapt the body count if it changed.
	//  2. Handle RESET (button or jack) and KICK.
	//  3. Run the physics with substeps (bounded total dt).
	//  4. NaN guard -> full reset if anything misbehaves.
	//  5. Emit X / Y / SPEED / EVENT (poly, one channel per body).
	//  6. Advance display state: trail ring, readouts, LED flashes.
	// --------------------------------------------------------
	void process(const ProcessArgs &args) override
	{
		const float dt = args.sampleTime;

		// ---- parameters ----
		int nP = rack::math::clamp((int)std::round(params[MASSES_PARAM].getValue()), 2, XB_MAXN);
		if (nP != n)
		{
			// Seed any newly added bodies.
			for (int i = n; i < nP; ++i)
				initBody(i);
			n = nP;
			needAcc = true;
		}
		bool wrap = params[WALL_PARAM].getValue() < 0.5f;
		vRandMass = params[MASSMODE_PARAM].getValue() > 0.5f;
		float speed = std::exp2(params[SPEED_PARAM].getValue());
		float G = 0.05f * std::exp2(params[GRAVITY_PARAM].getValue());
		float eps = 0.01f * std::pow(15.f, params[SOFT_PARAM].getValue());
		float eps2 = eps * eps;
		int range = rack::math::clamp((int)std::round(params[RANGE_PARAM].getValue()), 0, 4);

		// ---- reset / kick ----
		bool rs = resetBtnTrig.process(params[RESET_PARAM].getValue(), 0.1f, 1.f);
		rs |= resetJackTrig.process(inputs[RESET_INPUT].getVoltage(), 0.1f, 2.f);
		if (rs)
		{
			doReset();
			resetFlash = 1.f;
		}
		if (kickTrig.process(params[KICK_PARAM].getValue(), 0.1f, 1.f))
		{
			// Add a random impulse to every body.
			for (int i = 0; i < n; ++i)
			{
				float a = rnd() * 6.2831853f;
				vx[i] += 0.5f * std::cos(a);
				vy[i] += 0.5f * std::sin(a);
			}
			kickFlash = 1.f;
		}
		if (needAcc)
		{
			computeAcc(wrap, G, eps2);
			needAcc = false;
		}

		// ---- physics ----
		// Substep so the leapfrog integrator stays stable at high SPEED.
		bool ev[XB_MAXN] = {};
		if (!paused)
		{
			float total = dt * speed;
			int sub = rack::math::clamp((int)std::ceil(total / 0.002f), 1, 16);
			float h = total / (float)sub;
			for (int s = 0; s < sub; ++s)
				physicsStep(h, wrap, G, eps2, ev);
		}
		// NaN guard: any non-finite state -> wipe and start over.
		for (int i = 0; i < n; ++i)
		{
			if (!std::isfinite(px[i]) || !std::isfinite(py[i]) ||
				!std::isfinite(vx[i]) || !std::isfinite(vy[i]))
			{
				doReset();
				break;
			}
		}

		// ---- outputs ----
		// Four poly outputs, one channel per body.
		outputs[X_OUTPUT].setChannels(n);
		outputs[Y_OUTPUT].setChannels(n);
		outputs[VEL_OUTPUT].setChannels(n);
		outputs[EVENT_OUTPUT].setChannels(n);
		float ke = 0.f;
		for (int i = 0; i < n; ++i)
		{
			outputs[X_OUTPUT].setVoltage(xbMap(px[i], range), i);
			outputs[Y_OUTPUT].setVoltage(xbMap(py[i], range), i);
			float sp = std::sqrt(vx[i] * vx[i] + vy[i] * vy[i]);
			outputs[VEL_OUTPUT].setVoltage(rack::math::clamp(sp / 1.5f, 0.f, 1.f) * 10.f, i);
			if (ev[i])
			{
				// Trigger the wall-event pulse; also flash the display body.
				evPulse[i].trigger(1e-3f);
				eventFlash[i] = 1.f;
			}
			outputs[EVENT_OUTPUT].setVoltage(evPulse[i].process(dt) ? 10.f : 0.f, i);
			ke += 0.5f * massOf(i) * sp * sp;
			eventFlash[i] = std::max(0.f, eventFlash[i] - dt * 4.f);
		}

		// ---- display data ----
		// Snapshot the readouts the UI wants (per-frame, cheap).
		vG = G;
		vEps = eps;
		vSpeed = speed;
		vWrap = wrap;
		vRange = range;
		vKE = ke;
		resetFlash = std::max(0.f, resetFlash - dt * 4.f);
		kickFlash = std::max(0.f, kickFlash - dt * 4.f);
		lights[RESET_LIGHT].setBrightness(resetFlash);
		lights[KICK_LIGHT].setBrightness(kickFlash);

		// Trail commit at XB_TRAIL_HZ.
		trTimer += dt;
		const float st = 1.f / XB_TRAIL_HZ;
		if (trTimer >= st)
		{
			trTimer -= st;
			if (trTimer > st)
				trTimer = 0.f; // avoid drift after a long pause
			for (int i = 0; i < n; ++i)
			{
				trX[i][trHead] = px[i];
				trY[i][trHead] = py[i];
			}
			trHead = (trHead + 1) % XB_TN;
			if (trCount < XB_TN)
				trCount++;
		}
	}

	// --------------------------------------------------------
	// Persist simulation state + settings. The full state (positions,
	// velocities, masses) is saved so a patch reload continues exactly
	// where it left off.
	// --------------------------------------------------------
	json_t *dataToJson() override
	{
		json_t *root = json_object();
		json_object_set_new(root, "paused", json_boolean(paused));
		json_object_set_new(root, "zeroMomentum", json_boolean(zeroMomentum));
		json_object_set_new(root, "n", json_integer(n));
		json_t *a = json_array(), *b = json_array(), *c = json_array(), *d = json_array(), *e = json_array();
		for (int i = 0; i < XB_MAXN; ++i)
		{
			json_array_append_new(a, json_real(px[i]));
			json_array_append_new(b, json_real(py[i]));
			json_array_append_new(c, json_real(vx[i]));
			json_array_append_new(d, json_real(vy[i]));
			json_array_append_new(e, json_real(rm[i]));
		}
		json_object_set_new(root, "px", a);
		json_object_set_new(root, "py", b);
		json_object_set_new(root, "vx", c);
		json_object_set_new(root, "vy", d);
		json_object_set_new(root, "rm", e);
		return root;
	}

	// --------------------------------------------------------
	// Restore the fields written by dataToJson() (arrays are read
	// best-effort and clamped by the caller's parameters).
	// --------------------------------------------------------
	void dataFromJson(json_t *root) override
	{
		json_t *j = json_object_get(root, "paused");
		if (j)
			paused = json_is_true(j);
		j = json_object_get(root, "zeroMomentum");
		if (j)
			zeroMomentum = json_is_true(j);
		j = json_object_get(root, "n");
		if (j)
			n = rack::math::clamp((int)json_integer_value(j), 2, XB_MAXN);
		const char *keys[5] = {"px", "py", "vx", "vy", "rm"};
		float *arrs[5] = {px, py, vx, vy, rm};
		for (int k = 0; k < 5; ++k)
		{
			json_t *arr = json_object_get(root, keys[k]);
			if (!arr || !json_is_array(arr))
				continue;
			for (int i = 0; i < XB_MAXN; ++i)
			{
				json_t *v = json_array_get(arr, i);
				if (v)
					arrs[k][i] = (float)json_number_value(v);
			}
		}
		needAcc = true;
	}
};

// ------------------------------------------------------------
// Knob table: position, bipolar flag, label, cell top.
// ------------------------------------------------------------
struct XBKnob
{
	int param;
	float x, y;
	bool bipolar;
	const char *label;
	float top;
};
static const XBKnob XB_KNOBS[6] = {
	{XBodyProblem::MASSES_PARAM, XB_LKX, XB_KY[0], false, "MASSES", XB_SIDE_TOP[0]},
	{XBodyProblem::SPEED_PARAM, XB_LKX, XB_KY[1], true, "SPEED", XB_SIDE_TOP[1]},
	{XBodyProblem::GRAVITY_PARAM, XB_LKX, XB_KY[2], true, "GRAVITY", XB_SIDE_TOP[2]},
	{XBodyProblem::RANGE_PARAM, XB_RKX, XB_KY[0], false, "RANGE", XB_SIDE_TOP[0]},
	{XBodyProblem::SOFT_PARAM, XB_RKX, XB_KY[1], false, "SOFTEN", XB_SIDE_TOP[1]},
	{XBodyProblem::TRAIL_PARAM, XB_RKX, XB_KY[2], false, "TRAIL", XB_SIDE_TOP[2]},
};

// ------------------------------------------------------------
// Drawing helpers (anonymous namespace — UI-only)
// Shared flat panel kit (agmm_ui.hpp): flat fills and hairline strokes only.
// ------------------------------------------------------------
namespace
{

	namespace ag = agmm;

	// Per-body display colours (cycled for N > 10).
	NVGcolor bodyCol(int i)
	{
		static const unsigned char c[XB_MAXN][3] = {
			{0x3d, 0xe0, 0xff}, {0xff, 0x9c, 0x6a}, {0xb0, 0x5a, 0xff}, {0x5a, 0xf0, 0xa0}, {0xff, 0xe6, 0x80}, {0xff, 0x6a, 0xb8}, {0x6a, 0x8a, 0xff}, {0xb8, 0xf0, 0x5a}, {0xff, 0x5a, 0x5a}, {0x5a, 0xe0, 0xd0}};
		return nvgRGB(c[i % XB_MAXN][0], c[i % XB_MAXN][1], c[i % XB_MAXN][2]);
	}

	// ------------------------------------------------------------
	// XBPanelBackground — static panel chrome:
	//   * flat panel, title, display frame
	//   * 6 labelled knob cells (three on each side of the square)
	//   * CONTROL / I/O header + 9 labelled cells (outputs tinted)
	// ------------------------------------------------------------
	struct XBPanelBackground : TransparentWidget
	{
		XBPanelBackground() { box.size = Vec(XB_PANEL_W, XB_PANEL_H); }

		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			ag::panelBase(vg, XB_PANEL_W, XB_PANEL_H, "XBODYPROBLEM");
			ag::bezel(vg, XB_DX, XB_DY, XB_S, XB_S + XB_RO_H);

			// ---- Knob cells ----
			for (int k = 0; k < 6; ++k)
			{
				const XBKnob &kn = XB_KNOBS[k];
				ag::cell(vg, kn.x, kn.top, XB_SIDE_W, XB_SIDE_H, kn.label);
			}

			// ---- I/O row: switches, buttons, input, then tinted outputs ----
			ag::sectionHeader(vg, XB_PANEL_W, XB_IO_HDR, "CONTROL / I/O");
			const char *names[9] = {"WALLS", "MASSES", "RESET", "RST IN", "KICK", "X", "Y", "SPEED", "EVENT"};
			for (int i = 0; i < 9; ++i)
				ag::cell(vg, xbIoX(i), XB_IO_TOP, XB_IO_W, XB_IO_H, names[i], i >= 5);

			Widget::draw(args);
		}
	};

	// ------------------------------------------------------------
	// XBKnobArcs — live value arcs around the 6 knobs (track + one arc).
	//   * bipolar knobs draw from the centre outwards (cyan above, warm below)
	//   * unipolar knobs draw from the leftmost position.
	// ------------------------------------------------------------
	struct XBKnobArcs : TransparentWidget
	{
		XBodyProblem *module = nullptr;

		XBKnobArcs(XBodyProblem *m) : module(m) { box.size = Vec(XB_PANEL_W, XB_PANEL_H); }

		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			const float R = 16.f;
			const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
			const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

			for (int k = 0; k < 6; ++k)
			{
				const XBKnob &kn = XB_KNOBS[k];

				nvgBeginPath(vg);
				nvgArc(vg, kn.x, kn.y, R, a0, a1, NVG_CW);
				nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
				nvgStrokeWidth(vg, 2.f);
				nvgLineCap(vg, NVG_ROUND);
				nvgStroke(vg);

				if (!module)
					continue;
				float v = module->paramQuantities[kn.param]->getScaledValue();
				float from = kn.bipolar ? 0.5f : 0.f;
				float lo = std::min(from, v), hi = std::max(from, v);
				if (hi - lo < 0.004f)
					continue;

				float s = a0 + (a1 - a0) * lo;
				float e = a0 + (a1 - a0) * hi;
				NVGcolor col = (kn.bipolar && v < 0.5f) ? ag::warm() : ag::accent();

				nvgBeginPath(vg);
				nvgArc(vg, kn.x, kn.y, R, s, e, NVG_CW);
				nvgStrokeColor(vg, col);
				nvgStrokeWidth(vg, 2.f);
				nvgStroke(vg);
			}
		}
	};

	// ------------------------------------------------------------
	// XBDisplayWidget — the live 220x220 square simulation display plus a
	// 32 px readout strip underneath (readouts sit outside the plot so they
	// never cover the bodies).
	//   draw()       : static background (screen fill + grid + edge ticks).
	//   drawLayer(1) : bodies, gravity links, trails, wall style,
	//                  centre of mass (bounce mode), readouts.
	// If module == nullptr (browser preview) a static 4-body demo is drawn.
	// ------------------------------------------------------------
	struct XBDisplayWidget : TransparentWidget
	{
		XBodyProblem *module = nullptr;

		// Per-draw copy of the trail history (avoids racing the audio thread).
		float dTrX[XB_MAXN][XB_TN];
		float dTrY[XB_MAXN][XB_TN];

		XBDisplayWidget(XBodyProblem *m) : module(m)
		{
			box.pos = Vec(XB_DX, XB_DY);
			box.size = Vec(XB_S, XB_S + XB_RO_H);
			std::memset(dTrX, 0, sizeof(dTrX));
			std::memset(dTrY, 0, sizeof(dTrY));
		}

		// ---- static background: screen + grid + readout separator ----
		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			const float S = XB_S;

			ag::screen(vg, 0.f, 0.f, S, S + XB_RO_H);

			// Tenth grid (mid line brighter).
			for (int i = 1; i < 10; ++i)
			{
				float t = S * (float)i / 10.f;
				bool mid = (i == 5);
				nvgBeginPath(vg);
				nvgMoveTo(vg, t + 0.5f, 1.f);
				nvgLineTo(vg, t + 0.5f, S - 1.f);
				nvgMoveTo(vg, 1.f, t + 0.5f);
				nvgLineTo(vg, S - 1.f, t + 0.5f);
				nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, mid ? 30 : 12));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}
			// Fine ticks every 0.05 on the edges.
			nvgBeginPath(vg);
			for (int i = 1; i < 20; ++i)
			{
				float t = S * (float)i / 20.f;
				nvgMoveTo(vg, t + 0.5f, 1.f);
				nvgLineTo(vg, t + 0.5f, 4.f);
				nvgMoveTo(vg, t + 0.5f, S - 4.f);
				nvgLineTo(vg, t + 0.5f, S - 1.f);
				nvgMoveTo(vg, 1.f, t + 0.5f);
				nvgLineTo(vg, 4.f, t + 0.5f);
				nvgMoveTo(vg, S - 4.f, t + 0.5f);
				nvgLineTo(vg, S - 1.f, t + 0.5f);
			}
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			// Separator between the plot and the readout strip.
			nvgBeginPath(vg);
			nvgMoveTo(vg, 0.f, S + 0.5f);
			nvgLineTo(vg, S, S + 0.5f);
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 60));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			Widget::draw(args);
		}

		// Draw one body: flat disc, outline, and an event ring that fades out.
		void drawBody(NVGcontext *vg, float x, float y, float r, NVGcolor col, float flash)
		{
			nvgBeginPath(vg);
			nvgCircle(vg, x, y, r);
			nvgFillColor(vg, ag::withA(col, 0.9f));
			nvgFill(vg);

			if (flash > 0.02f)
			{
				nvgBeginPath(vg);
				nvgCircle(vg, x, y, r + 2.f + 3.f * (1.f - flash));
				nvgStrokeColor(vg, ag::withA(col, flash));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}
		}

		// ---- live simulation rendering ----
		void drawLayer(const DrawArgs &args, int layer) override
		{
			if (layer != 1)
			{
				Widget::drawLayer(args, layer);
				return;
			}
			NVGcontext *vg = args.vg;
			const float S = XB_S;

			// ----- snapshot (module or demo) -----
			int n = 4;
			float px[XB_MAXN], py[XB_MAXN], vx[XB_MAXN], vy[XB_MAXN], mass[XB_MAXN], evf[XB_MAXN];
			bool wrap = true, paused = false, randMass = false;
			float G = 0.05f, eps = 0.03f, speed = 1.f, ke = 0.f, trailK = 0.6f;
			int range = 2, head = 0, count = 0;

			if (module)
			{
				n = module->n;
				wrap = module->vWrap;
				paused = module->paused;
				randMass = module->vRandMass;
				G = module->vG;
				eps = module->vEps;
				speed = module->vSpeed;
				ke = module->vKE;
				range = module->vRange;
				head = module->trHead;
				count = module->trCount;
				trailK = module->params[XBodyProblem::TRAIL_PARAM].getValue();
				for (int i = 0; i < XB_MAXN; ++i)
				{
					px[i] = module->px[i];
					py[i] = module->py[i];
					vx[i] = module->vx[i];
					vy[i] = module->vy[i];
					mass[i] = module->massOf(i);
					evf[i] = module->eventFlash[i];
				}
				std::memcpy(dTrX, module->trX, sizeof(dTrX));
				std::memcpy(dTrY, module->trY, sizeof(dTrY));
			}
			else
			{
				// demo for the module browser: 4 bodies on a circular orbit
				for (int i = 0; i < XB_MAXN; ++i)
				{
					float a = 6.2831853f * (float)i / 4.f + 0.4f;
					px[i] = 0.5f + 0.28f * std::cos(a);
					py[i] = 0.5f + 0.28f * std::sin(a);
					vx[i] = -0.3f * std::sin(a);
					vy[i] = 0.3f * std::cos(a);
					mass[i] = 1.f;
					evf[i] = 0.f;
				}
				count = XB_TN;
				for (int i = 0; i < 4; ++i)
					for (int k = 0; k < XB_TN; ++k)
					{
						float a = 6.2831853f * (float)i / 4.f + 0.4f - 0.012f * (float)(XB_TN - 1 - k);
						dTrX[i][k] = 0.5f + 0.28f * std::cos(a);
						dTrY[i][k] = 0.5f + 0.28f * std::sin(a);
					}
			}
			n = rack::math::clamp(n, 2, XB_MAXN);
			range = rack::math::clamp(range, 0, 4);
			head = rack::math::clamp(head, 0, XB_TN - 1);
			count = rack::math::clamp(count, 0, XB_TN);

			// Unit-square -> pixel mapping (+y up -> screen -y).
			auto SX = [&](float x)
			{ return x * S; };
			auto SY = [&](float y)
			{ return (1.f - y) * S; };

			nvgSave(vg);
			nvgScissor(vg, 0, 0, S, S);

			// ================= gravity links =================
			// Faint line between every pair, alpha/width scale with F.
			nvgLineCap(vg, NVG_ROUND);
			for (int i = 0; i < n; ++i)
			{
				for (int j = i + 1; j < n; ++j)
				{
					float dx = px[j] - px[i], dy = py[j] - py[i];
					if (wrap)
					{
						dx -= std::round(dx);
						dy -= std::round(dy);
					}
					float r2 = dx * dx + dy * dy + eps * eps;
					float f = G * mass[i] * mass[j] / r2;
					float a = rack::math::clamp(f * 1.2f, 0.f, 0.55f);
					if (a < 0.03f)
						continue;
					NVGcolor c = nvgLerpRGBA(bodyCol(i), bodyCol(j), 0.5f);
					nvgBeginPath(vg);
					nvgMoveTo(vg, SX(px[i]), SY(py[i]));
					nvgLineTo(vg, SX(px[i] + dx), SY(py[i] + dy));
					if (wrap)
					{
						// Mirror link (Pac-Man: also draw the wrapped copy).
						nvgMoveTo(vg, SX(px[j]), SY(py[j]));
						nvgLineTo(vg, SX(px[j] - dx), SY(py[j] - dy));
					}
					nvgStrokeColor(vg, ag::withA(c, a));
					nvgStrokeWidth(vg, 1.f);
					nvgStroke(vg);
				}
			}

			// ================= trails =================
			// Drawn as short chunks with quadratic alpha (older = dimmer)
			// and a break when the body wrapped.
			int L = std::min(count, (int)std::round(trailK * (float)XB_TN));
			if (L >= 2)
			{
				nvgLineJoin(vg, NVG_ROUND);
				const int K = 4;
				for (int i = 0; i < n; ++i)
				{
					NVGcolor col = bodyCol(i);
					for (int s = 0; s < L - 1; s += K)
					{
						int e = std::min(s + K, L - 1);
						float t = (float)(s + 1) / (float)L;
						nvgBeginPath(vg);
						int i0 = ((head - L + s) % XB_TN + XB_TN) % XB_TN;
						nvgMoveTo(vg, SX(dTrX[i][i0]), SY(dTrY[i][i0]));
						for (int k = s + 1; k <= e; ++k)
						{
							int ia = ((head - L + k - 1) % XB_TN + XB_TN) % XB_TN;
							int ib = ((head - L + k) % XB_TN + XB_TN) % XB_TN;
							// Detect a wrap "jump" and start a new sub-path.
							bool jump = std::fabs(dTrX[i][ib] - dTrX[i][ia]) > 0.5f ||
										std::fabs(dTrY[i][ib] - dTrY[i][ia]) > 0.5f;
							if (jump)
								nvgMoveTo(vg, SX(dTrX[i][ib]), SY(dTrY[i][ib]));
							else
								nvgLineTo(vg, SX(dTrX[i][ib]), SY(dTrY[i][ib]));
						}
						nvgStrokeColor(vg, ag::withA(col, 0.55f * t * t));
						nvgStrokeWidth(vg, 0.8f + 1.2f * t);
						nvgStroke(vg);
					}
				}
			}

			// ================= centre of mass (bounce mode) =================
			if (!wrap)
			{
				float mx = 0.f, my = 0.f, mt = 0.f;
				for (int i = 0; i < n; ++i)
				{
					mx += mass[i] * px[i];
					my += mass[i] * py[i];
					mt += mass[i];
				}
				if (mt > 0.f)
				{
					mx /= mt;
					my /= mt;
					nvgBeginPath(vg);
					nvgMoveTo(vg, SX(mx) - 4.f, SY(my));
					nvgLineTo(vg, SX(mx) + 4.f, SY(my));
					nvgMoveTo(vg, SX(mx), SY(my) - 4.f);
					nvgLineTo(vg, SX(mx), SY(my) + 4.f);
					nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 110));
					nvgStrokeWidth(vg, 1.f);
					nvgStroke(vg);
				}
			}

			// ================= bodies (with wrap ghosts near the edges) =================
			// In wrap mode the same body is drawn up to 9 times (3x3 grid)
			// so bodies straddling an edge appear on both sides.
			for (int i = 0; i < n; ++i)
			{
				float r = 3.5f + 2.8f * std::cbrt(mass[i]);
				NVGcolor col = bodyCol(i);
				for (int oy = -1; oy <= 1; ++oy)
				{
					for (int ox = -1; ox <= 1; ++ox)
					{
						if (!wrap && (ox || oy))
							continue;
						float x = SX(px[i]) + (float)ox * S, y = SY(py[i]) + (float)oy * S;
						if (x < -r * 3.f || x > S + r * 3.f || y < -r * 3.f || y > S + r * 3.f)
							continue;

						// Velocity vector.
						nvgBeginPath(vg);
						nvgMoveTo(vg, x, y);
						nvgLineTo(vg, x + vx[i] * S * 0.22f, y - vy[i] * S * 0.22f);
						nvgStrokeColor(vg, ag::withA(col, 0.6f));
						nvgStrokeWidth(vg, 1.f);
						nvgStroke(vg);

						// Body + number (dark digit on the flat disc).
						drawBody(vg, x, y, r, col, evf[i]);
						char b[4];
						snprintf(b, sizeof(b), "%d", i + 1);
						ag::text(vg, x + 0.3f, y + 0.8f, b, 7.f, nvgRGBA(8, 10, 14, 235), ag::ALIGN_C, 0.f);
					}
				}
			}

			// ================= wall style =================
			if (wrap)
			{
				// Dashed cyan frame: leaving one side re-enters on the other.
				nvgBeginPath(vg);
				for (float t = 0.f; t < S; t += 8.f)
				{
					float t2 = std::min(t + 4.f, S);
					nvgMoveTo(vg, t, 1.f);
					nvgLineTo(vg, t2, 1.f);
					nvgMoveTo(vg, t, S - 1.f);
					nvgLineTo(vg, t2, S - 1.f);
					nvgMoveTo(vg, 1.f, t);
					nvgLineTo(vg, 1.f, t2);
					nvgMoveTo(vg, S - 1.f, t);
					nvgLineTo(vg, S - 1.f, t2);
				}
				nvgStrokeColor(vg, ag::withA(ag::accent(), 0.55f));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}
			else
			{
				// Solid warm frame for bounce mode.
				nvgBeginPath(vg);
				nvgRect(vg, 1.f, 1.f, S - 2.f, S - 2.f);
				nvgStrokeColor(vg, ag::withA(ag::warm(), 0.8f));
				nvgStrokeWidth(vg, 1.5f);
				nvgStroke(vg);
			}

			nvgRestore(vg);

			// ================= readout strip (below the plot) =================
			{
				char buf[64];
				const float y1 = S + 10.f, y2 = S + 23.f;

				snprintf(buf, sizeof(buf), "N %02d  %s%s", n, wrap ? "WRAP" : "BOUNCE", randMass ? "  RND" : "");
				ag::text(vg, 8.f, y1, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);

				snprintf(buf, sizeof(buf), "X/Y %s V", XB_RANGE_NAMES[range]);
				ag::text(vg, S - 8.f, y1, buf, 7.f, ag::textCol(), ag::ALIGN_R, 0.5f);

				if (paused)
					ag::text(vg, S / 2.f + 6.f, y1, "PAUSED", 7.f, ag::warm(), ag::ALIGN_C, 1.f);

				snprintf(buf, sizeof(buf), "SPD x%.2f  G x%.2f", speed, G / 0.05f);
				ag::text(vg, 8.f, y2, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.5f);

				snprintf(buf, sizeof(buf), "KE %.3f", ke);
				ag::text(vg, S - 8.f, y2, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.5f);
			}

			Widget::drawLayer(args, layer);
		}
	};

} // namespace

// ------------------------------------------------------------
// XBodyProblemWidget — Rack widget.
//   * panel background + knob arcs + display + screws
//   * six knobs (three on each side of the square)
//   * CONTROL / I/O row: 2 switches, RESET/KICK buttons with LEDs,
//     1 reset input jack, 4 poly output jacks (X / Y / SPEED / EVENT)
//   * context menu: pause, zero-momentum-on-reset, new random start
// ------------------------------------------------------------
struct XBodyProblemWidget : ModuleWidget
{
	XBodyProblemWidget(XBodyProblem *module)
	{
		setModule(module);
		box.size = Vec(XB_PANEL_W, XB_PANEL_H);

		addChild(new XBPanelBackground());
		addChild(new XBKnobArcs(module));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

		addChild(new XBDisplayWidget(module));

		// Six knobs (positions from XB_KNOBS).
		for (int k = 0; k < 6; ++k)
		{
			addParam(createParamCentered<RoundSmallBlackKnob>(
				Vec(XB_KNOBS[k].x, XB_KNOBS[k].y), module, XB_KNOBS[k].param));
		}

		// Control + I/O row.
		addParam(createParamCentered<CKSS>(Vec(xbIoX(0), XB_IO_Y), module, XBodyProblem::WALL_PARAM));
		addParam(createParamCentered<CKSS>(Vec(xbIoX(1), XB_IO_Y), module, XBodyProblem::MASSMODE_PARAM));

		// RESET button + LED + jack.
		addParam(createParamCentered<LEDButton>(Vec(xbIoX(2), XB_IO_Y), module, XBodyProblem::RESET_PARAM));
		addChild(createLightCentered<MediumLight<ag::CyanLight>>(Vec(xbIoX(2), XB_IO_Y), module, XBodyProblem::RESET_LIGHT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(xbIoX(3), XB_IO_Y), module, XBodyProblem::RESET_INPUT));

		// KICK button + LED.
		addParam(createParamCentered<LEDButton>(Vec(xbIoX(4), XB_IO_Y), module, XBodyProblem::KICK_PARAM));
		addChild(createLightCentered<MediumLight<ag::CyanLight>>(Vec(xbIoX(4), XB_IO_Y), module, XBodyProblem::KICK_LIGHT));

		// Poly outputs.
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(xbIoX(5), XB_IO_Y), module, XBodyProblem::X_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(xbIoX(6), XB_IO_Y), module, XBodyProblem::Y_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(xbIoX(7), XB_IO_Y), module, XBodyProblem::VEL_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(xbIoX(8), XB_IO_Y), module, XBodyProblem::EVENT_OUTPUT));
	}

	// Context menu: pause, zero-momentum toggle, new random start.
	void appendContextMenu(ui::Menu *menu) override
	{
		XBodyProblem *module = dynamic_cast<XBodyProblem *>(this->module);
		if (!module)
			return;

		menu->addChild(new ui::MenuSeparator);
		menu->addChild(createBoolPtrMenuItem("Pause simulation", "", &module->paused));
		menu->addChild(createBoolPtrMenuItem("Zero total momentum on reset", "", &module->zeroMomentum));
		menu->addChild(createMenuItem("New random start", "", [=]()
									  { module->doReset(); }));
	}
};

Model *modelXBodyProblem = createModel<XBodyProblem, XBodyProblemWidget>("XBodyProblem");