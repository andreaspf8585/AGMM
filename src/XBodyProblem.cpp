#include "plugin.hpp"
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
static const float XB_PANEL_H = RACK_GRID_HEIGHT;	  // 380

// Square display
static const float XB_S = 220.f;
static const float XB_DX = (XB_PANEL_W - XB_S) / 2.f; // 100
static const float XB_DY = 46.f;

// Side knobs: three on each side of the square
static const float XB_LKX = 48.f, XB_RKX = 372.f;
static const float XB_KY[3] = {86.f, 156.f, 226.f};

// I/O row
static const float XB_IO_SEC_Y = 294.f;
static const float XB_IO_Y = 340.f;
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
// Module
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

	// Simulation state (unit square, +y up)
	float px[XB_MAXN] = {}, py[XB_MAXN] = {};
	float vx[XB_MAXN] = {}, vy[XB_MAXN] = {};
	float ax[XB_MAXN] = {}, ay[XB_MAXN] = {};
	float rm[XB_MAXN] = {}; // random masses
	int n = 2;
	bool needAcc = true;

	// Settings (saved)
	bool paused = false;
	bool zeroMomentum = true;

	dsp::SchmittTrigger resetBtnTrig, resetJackTrig, kickTrig;
	dsp::PulseGenerator evPulse[XB_MAXN];

	// Display data
	float trX[XB_MAXN][XB_TN] = {};
	float trY[XB_MAXN][XB_TN] = {};
	int trHead = 0, trCount = 0;
	float trTimer = 0.f;
	float vG = 0.05f, vEps = 0.03f, vSpeed = 1.f, vKE = 0.f;
	bool vWrap = true, vRandMass = false;
	int vRange = 2;
	float resetFlash = 0.f, kickFlash = 0.f, eventFlash[XB_MAXN] = {};

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

	static float rnd() { return rack::random::uniform(); }

	float massOf(int i) const { return vRandMass ? rm[i] : 1.f; }

	void initBody(int i)
	{
		px[i] = 0.1f + 0.8f * rnd();
		py[i] = 0.1f + 0.8f * rnd();
		vx[i] = (rnd() - 0.5f) * 0.4f;
		vy[i] = (rnd() - 0.5f) * 0.4f;
		rm[i] = 0.4f + 2.f * rnd();
	}

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

	void onReset() override
	{
		paused = false;
		zeroMomentum = true;
		doReset();
	}

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
					dx -= std::round(dx);
					dy -= std::round(dy);
				}
				float r2 = dx * dx + dy * dy + eps2;
				float f = G / (r2 * std::sqrt(r2));
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

	// one leapfrog step; flags events in ev[]
	void physicsStep(float h, bool wrap, float G, float eps2, bool *ev)
	{
		for (int i = 0; i < n; ++i)
		{
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
				px[i] = rack::math::clamp(px[i], 0.f, 1.f);
				py[i] = rack::math::clamp(py[i], 0.f, 1.f);
			}
		}
		computeAcc(wrap, G, eps2);
		for (int i = 0; i < n; ++i)
		{
			vx[i] += ax[i] * 0.5f * h;
			vy[i] += ay[i] * 0.5f * h;
		}
	}

	void process(const ProcessArgs &args) override
	{
		const float dt = args.sampleTime;

		// ---- parameters ----
		int nP = rack::math::clamp((int)std::round(params[MASSES_PARAM].getValue()), 2, XB_MAXN);
		if (nP != n)
		{
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
		bool ev[XB_MAXN] = {};
		if (!paused)
		{
			float total = dt * speed;
			int sub = rack::math::clamp((int)std::ceil(total / 0.002f), 1, 16);
			float h = total / (float)sub;
			for (int s = 0; s < sub; ++s)
				physicsStep(h, wrap, G, eps2, ev);
		}
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
				evPulse[i].trigger(1e-3f);
				eventFlash[i] = 1.f;
			}
			outputs[EVENT_OUTPUT].setVoltage(evPulse[i].process(dt) ? 10.f : 0.f, i);
			ke += 0.5f * massOf(i) * sp * sp;
			eventFlash[i] = std::max(0.f, eventFlash[i] - dt * 4.f);
		}

		// ---- display data ----
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

		trTimer += dt;
		const float st = 1.f / XB_TRAIL_HZ;
		if (trTimer >= st)
		{
			trTimer -= st;
			if (trTimer > st)
				trTimer = 0.f;
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
// Knob table
// ------------------------------------------------------------
struct XBKnob
{
	int param;
	float x, y;
	int ticks;
	bool bipolar;
	const char *label;
};
static const XBKnob XB_KNOBS[6] = {
	{XBodyProblem::MASSES_PARAM, XB_LKX, XB_KY[0], 9, false, "MASSES"},
	{XBodyProblem::SPEED_PARAM, XB_LKX, XB_KY[1], 9, true, "SPEED"},
	{XBodyProblem::GRAVITY_PARAM, XB_LKX, XB_KY[2], 9, true, "GRAVITY"},
	{XBodyProblem::RANGE_PARAM, XB_RKX, XB_KY[0], 5, false, "RANGE"},
	{XBodyProblem::SOFT_PARAM, XB_RKX, XB_KY[1], 11, false, "SOFTEN"},
	{XBodyProblem::TRAIL_PARAM, XB_RKX, XB_KY[2], 11, false, "TRAIL"},
};

// ------------------------------------------------------------
// Theme + drawing helpers (anonymous namespace)
// ------------------------------------------------------------
namespace
{

	NVGcolor colBgTop() { return nvgRGB(0x25, 0x28, 0x30); }
	NVGcolor colBgBottom() { return nvgRGB(0x13, 0x15, 0x19); }
	NVGcolor colText() { return nvgRGB(0xd6, 0xdc, 0xe6); }
	NVGcolor colDim() { return nvgRGB(0x8a, 0x95, 0xa5); }
	NVGcolor colAccent() { return nvgRGB(0x3d, 0xe0, 0xff); }
	// NVGcolor colBright()   { return nvgRGB(0x7a, 0xec, 0xff); }
	NVGcolor colWarm() { return nvgRGB(0xff, 0x9c, 0x6a); }

	NVGcolor withA(NVGcolor c, float a) { return nvgRGBAf(c.r, c.g, c.b, a); }

	NVGcolor bodyCol(int i)
	{
		static const unsigned char c[XB_MAXN][3] = {
			{0x3d, 0xe0, 0xff}, {0xff, 0x9c, 0x6a}, {0xb0, 0x5a, 0xff}, {0x5a, 0xf0, 0xa0}, {0xff, 0xe6, 0x80}, {0xff, 0x6a, 0xb8}, {0x6a, 0x8a, 0xff}, {0xb8, 0xf0, 0x5a}, {0xff, 0x5a, 0x5a}, {0x5a, 0xe0, 0xd0}};
		return nvgRGB(c[i % XB_MAXN][0], c[i % XB_MAXN][1], c[i % XB_MAXN][2]);
	}

	template <typename TBase = GrayModuleLightWidget>
	struct TXBCyanLight : TBase
	{
		TXBCyanLight() { this->addBaseColor(nvgRGB(0x3d, 0xe0, 0xff)); }
	};
	using XBCyanLight = TXBCyanLight<>;

	void setFont(NVGcontext *vg)
	{
		std::shared_ptr<window::Font> font =
			APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
		if (font)
			nvgFontFaceId(vg, font->handle);
	}

	void drawText(NVGcontext *vg, float x, float y, const char *s, float size, NVGcolor col,
				  int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 1.f)
	{
		setFont(vg);
		nvgFontSize(vg, size);
		nvgTextLetterSpacing(vg, spacing);
		nvgTextAlign(vg, align);
		nvgFillColor(vg, col);
		nvgText(vg, x, y, s, NULL);
	}

	void drawLogo(NVGcontext *vg, float cx, float cy)
	{
		const float w = 64.f, h = 18.f;
		float x = cx - w / 2.f, y = cy - h / 2.f;

		nvgBeginPath(vg);
		nvgRoundedRect(vg, x - 2.f, y - 2.f, w + 4.f, h + 4.f, 9.f);
		nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
		nvgFill(vg);

		nvgBeginPath(vg);
		nvgRoundedRect(vg, x, y, w, h, 7.f);
		nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
										   nvgRGB(0x0b, 0x0d, 0x11), nvgRGB(0x1c, 0x20, 0x27)));
		nvgFill(vg);
		nvgStrokePaint(vg, nvgLinearGradient(vg, x, y, x + w, y,
											 nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
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

		drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
				 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
	}

	void drawKnobTicks(NVGcontext *vg, float cx, float cy, float r0, float r1, int n = 11)
	{
		nvgLineCap(vg, NVG_ROUND);
		for (int i = 0; i < n; ++i)
		{
			float a = (-0.75f + 1.5f * (float)i / (float)(n - 1)) * (float)M_PI;
			bool major = (i == 0 || i == n - 1 || (n % 2 == 1 && i == (n - 1) / 2));
			float ra = major ? r0 - 1.5f : r0;
			nvgBeginPath(vg);
			nvgMoveTo(vg, cx + std::sin(a) * ra, cy - std::cos(a) * ra);
			nvgLineTo(vg, cx + std::sin(a) * r1, cy - std::cos(a) * r1);
			nvgStrokeColor(vg, major ? nvgRGBA(0x3d, 0xe0, 0xff, 150) : nvgRGBA(255, 255, 255, 60));
			nvgStrokeWidth(vg, major ? 1.2f : 0.9f);
			nvgStroke(vg);
		}
	}

	// ------------------------------------------------------------
	// Panel background
	// ------------------------------------------------------------
	struct XBPanelBackground : TransparentWidget
	{
		XBPanelBackground() { box.size = Vec(XB_PANEL_W, XB_PANEL_H); }

		void label(NVGcontext *vg, float x, float y, const char *txt)
		{
			drawText(vg, x, y, txt, 7.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.0f);
		}

		void plate(NVGcontext *vg, float x, float y, float w, float h)
		{
			nvgBeginPath(vg);
			nvgRoundedRect(vg, x - 1.f, y - 1.f, w + 2.f, h + 3.f, 7.f);
			nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRoundedRect(vg, x, y, w, h, 6.f);
			nvgFillPaint(vg, nvgLinearGradient(vg, x, y, x, y + h,
											   nvgRGBA(0, 0, 0, 105), nvgRGBA(0, 0, 0, 60)));
			nvgFill(vg);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
		}

		void ring(NVGcontext *vg, float cx, float cy, float r)
		{
			nvgBeginPath(vg);
			nvgCircle(vg, cx, cy, r);
			nvgFillPaint(vg, nvgRadialGradient(vg, cx, cy, r * 0.3f, r,
											   nvgRGBA(0, 0, 0, 40), nvgRGBA(0, 0, 0, 110)));
			nvgFill(vg);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
		}

		void section(NVGcontext *vg, float y, const char *txt)
		{
			float x1 = 12.f, x2 = XB_PANEL_W - 12.f, cx = XB_PANEL_W / 2.f;
			float w = 14.f + (float)strlen(txt) * 7.f;
			drawText(vg, cx, y, txt, 8.f, colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);

			nvgBeginPath(vg);
			nvgMoveTo(vg, x1, y);
			nvgLineTo(vg, cx - w / 2.f, y);
			nvgStrokePaint(vg, nvgLinearGradient(vg, x1, y, cx - w / 2.f, y,
												 nvgRGBA(0x3d, 0xe0, 0xff, 0), nvgRGBA(0x3d, 0xe0, 0xff, 100)));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgMoveTo(vg, cx + w / 2.f, y);
			nvgLineTo(vg, x2, y);
			nvgStrokePaint(vg, nvgLinearGradient(vg, cx + w / 2.f, y, x2, y,
												 nvgRGBA(0x3d, 0xe0, 0xff, 100), nvgRGBA(0x3d, 0xe0, 0xff, 0)));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
		}

		// kind: 0 = input (orange), 1 = output (cyan), 2 = control (purple)
		void cell(NVGcontext *vg, float cx, float cy, const char *txt, int kind)
		{
			const float w = 36.f, top = -27.f, h = 44.f;
			float x = cx - w / 2.f, y = cy + top;
			plate(vg, x, y, w, h);

			NVGcolor c = (kind == 0)   ? nvgRGBA(0xff, 0x9c, 0x6a, 170)
						 : (kind == 1) ? nvgRGBA(0x3d, 0xe0, 0xff, 170)
									   : nvgRGBA(0xb0, 0x5a, 0xff, 170);
			nvgBeginPath(vg);
			nvgRoundedRect(vg, x + 6.f, y + 1.5f, w - 12.f, 1.8f, 0.9f);
			nvgFillColor(vg, c);
			nvgFill(vg);

			label(vg, cx, cy - 19.f, txt);
		}

		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, XB_PANEL_W, XB_PANEL_H);
			nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, XB_PANEL_H, colBgTop(), colBgBottom()));
			nvgFill(vg);

			nvgStrokeWidth(vg, 1.f);
			for (int y = 0; y < (int)XB_PANEL_H; y += 3)
			{
				nvgBeginPath(vg);
				nvgMoveTo(vg, 0, y + 0.5f);
				nvgLineTo(vg, XB_PANEL_W, y + 0.5f);
				nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
				nvgStroke(vg);
			}

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, XB_PANEL_W, XB_PANEL_H);
			nvgFillPaint(vg, nvgRadialGradient(vg, XB_PANEL_W / 2.f, XB_PANEL_H * 0.4f,
											   XB_PANEL_H * 0.35f, XB_PANEL_H * 0.95f,
											   nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRect(vg, 0.5f, 0.5f, XB_PANEL_W - 1.f, XB_PANEL_H - 1.f);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, XB_PANEL_W, 2.f);
			nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, XB_PANEL_W, 0,
											   nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
			nvgFill(vg);

			drawLogo(vg, XB_PANEL_W / 2.f, 15.f);
			drawText(vg, XB_PANEL_W / 2.f, 32.f, "XBODYPROBLEM", 12.f, colText(),
					 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

			// ---- Display bezel ----
			float bx = XB_DX - 5.f, by = XB_DY - 5.f;
			float bw = XB_S + 10.f, bh = XB_S + 10.f;

			nvgBeginPath(vg);
			nvgRoundedRect(vg, bx - 4.f, by - 3.f, bw + 8.f, bh + 9.f, 10.f);
			nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
			nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
											   nvgRGB(0x4a, 0x50, 0x5a), nvgRGB(0x14, 0x16, 0x1a)));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRoundedRect(vg, bx - 2.f, by - 2.f, bw + 4.f, bh + 4.f, 7.f);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 36));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
			nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
			nvgFill(vg);

			// ---- Knobs ----
			for (int k = 0; k < 6; ++k)
			{
				const XBKnob &kn = XB_KNOBS[k];
				ring(vg, kn.x, kn.y, 17.f);
				drawKnobTicks(vg, kn.x, kn.y, 21.f, 24.f, kn.ticks);
				label(vg, kn.x, kn.y + 29.f, kn.label);
			}

			// ---- I/O row ----
			section(vg, XB_IO_SEC_Y, "CONTROL  /  I / O");
			const char *names[9] = {"WALLS", "MASSES", "RESET", "RST IN", "KICK", "X", "Y", "SPEED", "EVENT"};
			const int kinds[9] = {2, 2, 2, 0, 2, 1, 1, 1, 1};
			for (int i = 0; i < 9; ++i)
				cell(vg, xbIoX(i), XB_IO_Y, names[i], kinds[i]);

			drawText(vg, XB_PANEL_W / 2.f, 368.f, "N-BODY GRAVITY LFO  -  POLYPHONIC X / Y", 6.f,
					 nvgRGBA(0x8a, 0x95, 0xa5, 120), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);

			Widget::draw(args);
		}
	};

	// ------------------------------------------------------------
	// Live value arcs around the knobs
	// ------------------------------------------------------------
	struct XBKnobArcs : TransparentWidget
	{
		XBodyProblem *module = nullptr;

		XBKnobArcs(XBodyProblem *m) : module(m) { box.size = Vec(XB_PANEL_W, XB_PANEL_H); }

		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			const float R = 17.2f;
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
				NVGcolor col = (kn.bipolar && v < 0.5f) ? colWarm() : colAccent();

				nvgBeginPath(vg);
				nvgArc(vg, kn.x, kn.y, R, s, e, NVG_CW);
				nvgStrokeColor(vg, withA(col, 0.18f));
				nvgStrokeWidth(vg, 5.f);
				nvgStroke(vg);

				nvgBeginPath(vg);
				nvgArc(vg, kn.x, kn.y, R, s, e, NVG_CW);
				nvgStrokeColor(vg, withA(col, 0.95f));
				nvgStrokeWidth(vg, 2.f);
				nvgStroke(vg);
			}
		}
	};

	// ------------------------------------------------------------
	// The big square display
	// ------------------------------------------------------------
	struct XBDisplayWidget : TransparentWidget
	{
		XBodyProblem *module = nullptr;

		XBDisplayWidget(XBodyProblem *m) : module(m)
		{
			box.pos = Vec(XB_DX, XB_DY);
			box.size = Vec(XB_S, XB_S);
		}

		// ---- static background: screen + grid ----
		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			const float S = XB_S;

			nvgBeginPath(vg);
			nvgRoundedRect(vg, 0, 0, S, S, 4.f);
			nvgFillPaint(vg, nvgRadialGradient(vg, S / 2.f, S / 2.f, 10.f, S * 0.8f,
											   nvgRGB(0x0d, 0x1a, 0x24), nvgRGB(0x03, 0x06, 0x09)));
			nvgFill(vg);

			for (int i = 1; i < 10; ++i)
			{
				float t = S * (float)i / 10.f;
				bool mid = (i == 5);
				nvgBeginPath(vg);
				nvgMoveTo(vg, t + 0.5f, 1.f);
				nvgLineTo(vg, t + 0.5f, S - 1.f);
				nvgMoveTo(vg, 1.f, t + 0.5f);
				nvgLineTo(vg, S - 1.f, t + 0.5f);
				nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, mid ? 26 : 10));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}
			// fine ticks every 0.05 on the edges
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

			Widget::draw(args);
		}

		void drawBody(NVGcontext *vg, float x, float y, float r, NVGcolor col, float glow)
		{
			NVGcolor hi = nvgLerpRGBA(col, nvgRGB(255, 255, 255), 0.65f);
			NVGcolor lo = nvgLerpRGBA(col, nvgRGB(0, 0, 0), 0.55f);

			nvgBeginPath(vg);
			nvgCircle(vg, x, y, r * 2.8f);
			nvgFillPaint(vg, nvgRadialGradient(vg, x, y, r * 0.6f, r * 2.8f,
											   withA(col, 0.14f + 0.35f * glow), withA(col, 0.f)));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, x, y, r);
			nvgFillPaint(vg, nvgRadialGradient(vg, x - r * 0.35f, y - r * 0.4f, r * 0.1f, r * 1.3f, hi, lo));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, x, y, r - 0.4f);
			nvgStrokeColor(vg, withA(col, 0.9f));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, x - r * 0.38f, y - r * 0.42f, r * 0.2f);
			nvgFillColor(vg, nvgRGBA(255, 255, 255, 210));
			nvgFill(vg);
		}

		void drawLayer(const DrawArgs &args, int layer) override
		{
			if (layer != 1)
			{
				Widget::drawLayer(args, layer);
				return;
			}
			NVGcontext *vg = args.vg;
			const float S = XB_S;

			// ----- snapshot -----
			int n = 4;
			float px[XB_MAXN], py[XB_MAXN], vx[XB_MAXN], vy[XB_MAXN], mass[XB_MAXN], evf[XB_MAXN];
			bool wrap = true, paused = false, randMass = false;
			float G = 0.05f, eps = 0.03f, speed = 1.f, ke = 0.f, trailK = 0.6f;
			int range = 2, head = 0, count = 0;
			static float trX[XB_MAXN][XB_TN], trY[XB_MAXN][XB_TN];

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
				std::memcpy(trX, module->trX, sizeof(trX));
				std::memcpy(trY, module->trY, sizeof(trY));
			}
			else
			{
				// demo for the module browser
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
						trX[i][k] = 0.5f + 0.28f * std::cos(a);
						trY[i][k] = 0.5f + 0.28f * std::sin(a);
					}
			}
			n = rack::math::clamp(n, 2, XB_MAXN);

			auto SX = [&](float x)
			{ return x * S; };
			auto SY = [&](float y)
			{ return (1.f - y) * S; };

			nvgSave(vg);
			nvgScissor(vg, 0, 0, S, S);

			// ================= gravity links =================
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
						nvgMoveTo(vg, SX(px[j]), SY(py[j]));
						nvgLineTo(vg, SX(px[j] - dx), SY(py[j] - dy));
					}
					nvgStrokeColor(vg, withA(c, a));
					nvgStrokeWidth(vg, 0.8f + 1.6f * a);
					nvgStroke(vg);
				}
			}

			// ================= trails =================
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
						nvgMoveTo(vg, SX(trX[i][i0]), SY(trY[i][i0]));
						for (int k = s + 1; k <= e; ++k)
						{
							int ia = ((head - L + k - 1) % XB_TN + XB_TN) % XB_TN;
							int ib = ((head - L + k) % XB_TN + XB_TN) % XB_TN;
							bool jump = std::fabs(trX[i][ib] - trX[i][ia]) > 0.5f ||
										std::fabs(trY[i][ib] - trY[i][ia]) > 0.5f;
							if (jump)
								nvgMoveTo(vg, SX(trX[i][ib]), SY(trY[i][ib]));
							else
								nvgLineTo(vg, SX(trX[i][ib]), SY(trY[i][ib]));
						}
						nvgStrokeColor(vg, withA(col, 0.55f * t * t));
						nvgStrokeWidth(vg, 0.6f + 1.6f * t);
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

			// ================= bodies (with wrap ghosts near the edges) =================
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

						// velocity vector
						nvgBeginPath(vg);
						nvgMoveTo(vg, x, y);
						nvgLineTo(vg, x + vx[i] * S * 0.22f, y - vy[i] * S * 0.22f);
						nvgStrokeColor(vg, withA(col, 0.6f));
						nvgStrokeWidth(vg, 1.f);
						nvgStroke(vg);

						drawBody(vg, x, y, r, col, evf[i]);
						char b[4];
						snprintf(b, sizeof(b), "%d", i + 1);
						drawText(vg, x + 0.3f, y + 0.8f, b, 5.5f, nvgRGBA(8, 10, 14, 220),
								 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
					}
				}
			}

			// ================= wall style =================
			if (wrap)
			{
				// dashed cyan frame with small arrows: leaving one side re-enters on the other
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
				nvgStrokeColor(vg, withA(colAccent(), 0.55f));
				nvgStrokeWidth(vg, 1.4f);
				nvgStroke(vg);
			}
			else
			{
				nvgBeginPath(vg);
				nvgRoundedRect(vg, 1.f, 1.f, S - 2.f, S - 2.f, 3.f);
				nvgStrokeColor(vg, withA(colWarm(), 0.75f));
				nvgStrokeWidth(vg, 2.f);
				nvgStroke(vg);
				nvgBeginPath(vg);
				nvgRoundedRect(vg, 1.f, 1.f, S - 2.f, S - 2.f, 3.f);
				nvgStrokeColor(vg, withA(colWarm(), 0.16f));
				nvgStrokeWidth(vg, 6.f);
				nvgStroke(vg);
			}

			// ================= readouts =================
			{
				char buf[64];
				NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 230);
				NVGcolor cm = nvgRGBA(0xff, 0xe6, 0x80, 225);
				NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 235);

				snprintf(buf, sizeof(buf), "N %02d  %s%s", n, wrap ? "WRAP" : "BOUNCE", randMass ? "  RND" : "");
				drawText(vg, 8.f, 10.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

				snprintf(buf, sizeof(buf), "X/Y %s V", XB_RANGE_NAMES[range]);
				drawText(vg, S - 8.f, 10.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

				snprintf(buf, sizeof(buf), "SPD x%.2f  G x%.2f", speed, G / 0.05f);
				drawText(vg, 8.f, S - 10.f, buf, 6.5f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

				snprintf(buf, sizeof(buf), "KE %.3f", ke);
				drawText(vg, S - 8.f, S - 10.f, buf, 6.5f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

				if (paused)
					drawText(vg, S / 2.f, 10.f, "PAUSED", 7.f, colWarm(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 2.f);
			}

			// ================= scanlines, vignette, glass =================
			nvgBeginPath(vg);
			for (int y = 0; y < (int)S; y += 3)
				nvgRect(vg, 0, y + 1, S, 1);
			nvgFillColor(vg, nvgRGBA(0, 0, 0, 18));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, S, S);
			nvgFillPaint(vg, nvgRadialGradient(vg, S / 2.f, S / 2.f, S * 0.55f, S * 0.80f,
											   nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, S, S * 0.25f);
			nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, S * 0.25f,
											   nvgRGBA(255, 255, 255, 14), nvgRGBA(255, 255, 255, 0)));
			nvgFill(vg);

			nvgRestore(vg);

			Widget::drawLayer(args, layer);
		}
	};

} // namespace

// ------------------------------------------------------------
// Module widget
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

		for (int k = 0; k < 6; ++k)
		{
			addParam(createParamCentered<RoundSmallBlackKnob>(
				Vec(XB_KNOBS[k].x, XB_KNOBS[k].y), module, XB_KNOBS[k].param));
		}

		// control + I/O row
		addParam(createParamCentered<CKSS>(Vec(xbIoX(0), XB_IO_Y), module, XBodyProblem::WALL_PARAM));
		addParam(createParamCentered<CKSS>(Vec(xbIoX(1), XB_IO_Y), module, XBodyProblem::MASSMODE_PARAM));

		addParam(createParamCentered<LEDButton>(Vec(xbIoX(2), XB_IO_Y), module, XBodyProblem::RESET_PARAM));
		addChild(createLightCentered<MediumLight<XBCyanLight>>(Vec(xbIoX(2), XB_IO_Y), module, XBodyProblem::RESET_LIGHT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(xbIoX(3), XB_IO_Y), module, XBodyProblem::RESET_INPUT));

		addParam(createParamCentered<LEDButton>(Vec(xbIoX(4), XB_IO_Y), module, XBodyProblem::KICK_PARAM));
		addChild(createLightCentered<MediumLight<XBCyanLight>>(Vec(xbIoX(4), XB_IO_Y), module, XBodyProblem::KICK_LIGHT));

		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(xbIoX(5), XB_IO_Y), module, XBodyProblem::X_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(xbIoX(6), XB_IO_Y), module, XBodyProblem::Y_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(xbIoX(7), XB_IO_Y), module, XBodyProblem::VEL_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(xbIoX(8), XB_IO_Y), module, XBodyProblem::EVENT_OUTPUT));
	}

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
//
//
//

Model *modelXBodyProblem = createModel<XBodyProblem, XBodyProblemWidget>("XBodyProblem");