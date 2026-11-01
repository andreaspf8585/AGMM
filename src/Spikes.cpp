#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int   SP_TRAIL = 400;                         // points kept in the 3D trail
static const float SP_W = RACK_GRID_WIDTH * 16;            // 240
static const float SP_H = RACK_GRID_HEIGHT;                // 380
static const float SP_DX = 20.f, SP_DY = 46.f, SP_DW = 200.f, SP_DH = 118.f;
static const float SP_KCOL[3] = {40.f, 120.f, 200.f};
static const float SP_KROW[2] = {208.f, 274.f};
static const float SP_JY = 346.f;
static float spJx(int i) { return 30.f + 30.f * i; }      // 7 jacks, 30 px apart

struct Spikes : Module {
	// New items are appended at the end so old patches keep loading.
	enum ParamId {
		SPEED_PARAM,
		BLEND_PARAM,
		FRACTAL_PARAM,
		FRACTAL_SPEED_PARAM,
		SCALE_PARAM,
		ROTATE_PARAM,          // view only: auto-rotation speed of the 3D display
		PARAMS_LEN
	};
	enum InputId {
		SPEED_INPUT,
		BLEND_INPUT,
		FRACTAL_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		X_OUTPUT,
		Y_OUTPUT,
		Z_OUTPUT,
		MIX_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	// ------------------------------------------------------------------
	// Chaotic attractor states
	// ------------------------------------------------------------------
	double lorenzX  =  0.1, lorenzY  = 0.0, lorenzZ  = 0.0;
	double rosslerX =  1.0, rosslerY = 1.0, rosslerZ = 0.0;

	// ------------------------------------------------------------------
	// Fractal correlated randomness (fractional Brownian motion)
	//   N octaves of smoothly-interpolated random values; amplitudes
	//   fall off as 2^(-H) per octave. H controls correlation length.
	// ------------------------------------------------------------------
	static const int NUM_OCTAVES = 6;
	static constexpr double HURST = 0.65;

	struct Octave {
		double current = 0.0;
		double target  = 0.0;
		double phase   = 0.0;
		bool   initialized = false;
	};
	Octave octaves[NUM_OCTAVES];

	// xorshift32 RNG for cheap, high-quality uniform noise
	uint32_t rngState = 0x9E3779B9u;
	inline uint32_t xorshift() {
		uint32_t x = rngState;
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		rngState = x;
		return x;
	}
	inline double randUnipolar() { return (xorshift() & 0xFFFFFF) / 16777216.0; }
	inline double randBipolar()  { return randUnipolar() * 2.0 - 1.0; }

	// ------------------------------------------------------------------
	// Display data (written by the audio thread, read by the UI thread;
	// a torn read is only cosmetic)
	// ------------------------------------------------------------------
	float trailX[SP_TRAIL] = {};
	float trailY[SP_TRAIL] = {};
	float trailZ[SP_TRAIL] = {};
	int trailHead = 0;           // index of the newest point
	int trailCount = 0;          // valid points (grows until SP_TRAIL)
	double trailAcc = 0.0;       // attractor time since the last trail point
	float outX = 0.f, outY = 0.f, outZ = 0.f, outMix = 0.f;   // volts
	float dispSpeed = 1.f, dispBlend = 0.5f, dispFractal = 0.2f;

	Spikes() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

		configParam(SPEED_PARAM,         0.01f, 10.f, 1.f, "Speed");
		configParam(BLEND_PARAM,         0.f,   1.f,  0.5f, "Lorenz / Rössler Blend", "%", 0.f, 100.f);
		configParam(FRACTAL_PARAM,       0.f,   1.f,  0.2f, "Fractal Amount", "%", 0.f, 100.f);
		configParam(FRACTAL_SPEED_PARAM, 0.01f, 50.f, 1.f, "Fractal Speed", " Hz");
		configParam(SCALE_PARAM,         0.1f,  10.f, 5.f, "Scale", " V");
		configParam(ROTATE_PARAM,       -1.f,   1.f,  0.25f, "Display rotation speed", "%", 0.f, 100.f);

		configInput(SPEED_INPUT,   "Speed CV (1 V/oct)");
		configInput(BLEND_INPUT,   "Blend CV (10 V = 100%)");
		configInput(FRACTAL_INPUT, "Fractal CV (10 V = 100%)");

		configOutput(X_OUTPUT,   "X");
		configOutput(Y_OUTPUT,   "Y");
		configOutput(Z_OUTPUT,   "Z");
		configOutput(MIX_OUTPUT, "Mix");
	}

	void onReset() override {
		lorenzX = 0.1; lorenzY = 0.0; lorenzZ = 0.0;
		rosslerX = 1.0; rosslerY = 1.0; rosslerZ = 0.0;
		for (int i = 0; i < NUM_OCTAVES; i++)
			octaves[i] = Octave();
		trailHead = 0;
		trailCount = 0;
		trailAcc = 0.0;
		for (int i = 0; i < SP_TRAIL; i++)
			trailX[i] = trailY[i] = trailZ[i] = 0.f;
	}

	// ------------------------------------------------------------------
	// Vector fields
	// ------------------------------------------------------------------
	static void lorenzDeriv(const double s[3], double d[3]) {
		const double sigma = 10.0, rho = 28.0, beta = 8.0/3.0;
		d[0] = sigma * (s[1] - s[0]);
		d[1] = s[0] * (rho - s[2]) - s[1];
		d[2] = s[0] * s[1] - beta * s[2];
	}
	static void rosslerDeriv(const double s[3], double d[3]) {
		const double a = 0.2, b = 0.2, c = 5.7;
		d[0] = -s[1] - s[2];
		d[1] = s[0] + a * s[1];
		d[2] = b + s[2] * (s[0] - c);
	}

	// Classical RK4 step
	static void rk4(double s[3], double dt, void(*deriv)(const double*, double*)) {
		double k1[3], k2[3], k3[3], k4[3], t[3];
		deriv(s, k1);
		for (int i = 0; i < 3; i++) t[i] = s[i] + 0.5 * dt * k1[i];
		deriv(t, k2);
		for (int i = 0; i < 3; i++) t[i] = s[i] + 0.5 * dt * k2[i];
		deriv(t, k3);
		for (int i = 0; i < 3; i++) t[i] = s[i] + dt * k3[i];
		deriv(t, k4);
		for (int i = 0; i < 3; i++)
			s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
	}

	// ------------------------------------------------------------------
	// Fractal correlated noise: summed octaves of smoothed random values
	// ------------------------------------------------------------------
	double computeFractal(double baseFreq, double sr) {
		double total  = 0.0;
		double normSq = 0.0;
		double amp    = 1.0;
		double freq   = baseFreq;

		for (int i = 0; i < NUM_OCTAVES; i++) {
			Octave& o = octaves[i];
			if (!o.initialized) {
				o.current = randBipolar();
				o.target  = randBipolar();
				o.phase   = randUnipolar();
				o.initialized = true;
			}
			double inc = freq / sr;
			if (inc > 0.45) inc = 0.45; // keep well below Nyquist
			o.phase += inc;
			while (o.phase >= 1.0) {
				o.phase -= 1.0;
				o.current = o.target;
				o.target  = randBipolar();
			}
			// smoothstep interpolation between current and target
			double t  = o.phase;
			double sm = t * t * (3.0 - 2.0 * t);
			double v  = o.current * (1.0 - sm) + o.target * sm;

			total  += v * amp;
			normSq += amp * amp;
			amp    *= std::pow(0.5, HURST);
			freq   *= 2.0;
		}
		return total / std::sqrt(normSq);
	}

	void process(const ProcessArgs& args) override {
		// ---------------- Parameters & CV ----------------
		float speed = params[SPEED_PARAM].getValue();
		if (inputs[SPEED_INPUT].isConnected()) {
			float cv = clamp(inputs[SPEED_INPUT].getVoltage(), -5.f, 5.f);
			speed *= std::pow(2.f, cv);
		}
		speed = clamp(speed, 0.001f, 100.f);

		float blend = params[BLEND_PARAM].getValue();
		if (inputs[BLEND_INPUT].isConnected())
			blend = clamp(blend + inputs[BLEND_INPUT].getVoltage() * 0.1f, 0.f, 1.f);

		float fractalAmt = params[FRACTAL_PARAM].getValue();
		if (inputs[FRACTAL_INPUT].isConnected())
			fractalAmt = clamp(fractalAmt + inputs[FRACTAL_INPUT].getVoltage() * 0.1f, 0.f, 1.f);

		float fractalSpeed = params[FRACTAL_SPEED_PARAM].getValue();
		float scale        = params[SCALE_PARAM].getValue();

		dispSpeed = speed;
		dispBlend = blend;
		dispFractal = fractalAmt;

		// ---------------- Integrate attractors ----------------
		// speed=1 → roughly one attractor orbit per second at 48 kHz
		double dt = speed / args.sampleRate;
		const int substeps = 2;
		double subdt = dt / substeps;
		for (int s = 0; s < substeps; s++) {
			double L[3] = { lorenzX, lorenzY, lorenzZ };
			rk4(L, subdt, lorenzDeriv);
			lorenzX = L[0]; lorenzY = L[1]; lorenzZ = L[2];

			double R[3] = { rosslerX, rosslerY, rosslerZ };
			rk4(R, subdt, rosslerDeriv);
			rosslerX = R[0]; rosslerY = R[1]; rosslerZ = R[2];
		}

		// Safety net: re-seed an attractor if it ever leaves the finite range
		if (!std::isfinite(lorenzX + lorenzY + lorenzZ)) {
			lorenzX = 0.1; lorenzY = 0.0; lorenzZ = 0.0;
		}
		if (!std::isfinite(rosslerX + rosslerY + rosslerZ)) {
			rosslerX = 1.0; rosslerY = 1.0; rosslerZ = 0.0;
		}

		// ---------------- Fractal correlated noise ----------------
		double fractal = computeFractal(fractalSpeed, args.sampleRate);

		// ---------------- Normalize attractor outputs to ~[-1, 1] -----
		double lx = lorenzX / 20.0;
		double ly = lorenzY / 25.0;
		double lz = (lorenzZ - 25.0) / 25.0;

		double rx = rosslerX / 12.0;
		double ry = rosslerY / 12.0;
		double rz = (rosslerZ - 5.0) / 15.0;   // Rössler z is spiky

		// ---------------- Blend Lorenz ↔ Rössler ----------------
		double b  = blend;
		double mx = (1.0 - b) * lx + b * rx;
		double my = (1.0 - b) * ly + b * ry;
		double mz = (1.0 - b) * lz + b * rz;

		// ---------------- Mix in fractal correlated randomness ----------------
		double f = fractalAmt;
		mx = mx * (1.0 - f) + fractal * f;
		my = my * (1.0 - f) + fractal * f;
		mz = mz * (1.0 - f) + fractal * f;

		// ---------------- Mix bus ----------------
		double mix = (mx + my + mz) / 3.0;

		// ---------------- Output ----------------
		float vx   = clamp((float)(mx  * scale), -10.f, 10.f);
		float vy   = clamp((float)(my  * scale), -10.f, 10.f);
		float vz   = clamp((float)(mz  * scale), -10.f, 10.f);
		float vmix = clamp((float)(mix * scale), -10.f, 10.f);
		outputs[X_OUTPUT].setVoltage(vx);
		outputs[Y_OUTPUT].setVoltage(vy);
		outputs[Z_OUTPUT].setVoltage(vz);
		outputs[MIX_OUTPUT].setVoltage(vmix);

		outX = vx; outY = vy; outZ = vz; outMix = vmix;

		// ---------------- 3D trail for the display ----------------
		// One point per 0.015 units of attractor time, so the trail shape is
		// independent of the speed setting.
		trailAcc += dt;
		if (trailAcc >= 0.015) {
			trailAcc = std::fmod(trailAcc, 0.015);
			int h = (trailHead + 1) % SP_TRAIL;
			trailX[h] = clamp((float)mx, -1.6f, 1.6f);
			trailY[h] = clamp((float)my, -1.6f, 1.6f);
			trailZ[h] = clamp((float)mz, -1.6f, 1.6f);
			trailHead = h;
			if (trailCount < SP_TRAIL)
				trailCount++;
		}
	}
};


namespace {

NVGcolor colBgTop()    { return nvgRGB(0x23, 0x26, 0x2d); }
NVGcolor colBgBottom() { return nvgRGB(0x14, 0x16, 0x1a); }
NVGcolor colText()     { return nvgRGB(0xc9, 0xd1, 0xdc); }
NVGcolor colDim()      { return nvgRGB(0x7a, 0x85, 0x94); }
NVGcolor colAccent()   { return nvgRGB(0x3d, 0xe0, 0xff); }
NVGcolor colViolet()   { return nvgRGB(0xb0, 0x5a, 0xff); }
NVGcolor withAlpha(NVGcolor c, float a) { c.a = a; return c; }

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
	nvgFillColor(vg, nvgRGBA(0, 0, 0, 80));
	nvgFill(vg);
	nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
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

void label(NVGcontext* vg, float x, float y, const char* t, float size = 7.f) {
	drawText(vg, x, y, t, size, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
}

void topCell(NVGcontext* vg, float cx, float cy, const char* t, NVGcolor col) {
	plate(vg, cx - 14.f, cy - 27.f, 28.f, 44.f);
	drawText(vg, cx, cy - 19.f, t, 6.f, col, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
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

struct SPPanel : TransparentWidget {
	SPPanel() { box.size = Vec(SP_W, SP_H); }
	void draw(const DrawArgs& args) override {
		NVGcontext* vg = args.vg;
		drawPanelBase(vg, SP_W, SP_H, "SPIKES");
		drawBezel(vg, SP_DX, SP_DY, SP_DW, SP_DH);

		const char* names[6] = {"SPEED", "BLEND", "FRACTAL", "FRC SPEED", "SCALE", "ROTATE"};
		for (int i = 0; i < 6; ++i) {
			float cx = SP_KCOL[i % 3], cy = SP_KROW[i / 3];
			ring(vg, cx, cy, 18.f);
			label(vg, cx, cy + 25.f, names[i]);
		}

		// jack row: 3 CV inputs | 4 outputs
		const char* io[7] = {"SPEED", "BLEND", "FRACT", "X", "Y", "Z", "MIX"};
		for (int i = 0; i < 7; ++i)
			topCell(vg, spJx(i), SP_JY, io[i], i < 3 ? colDim() : withAlpha(colAccent(), 0.9f));

		// divider between inputs and outputs + small captions
		float dx = 0.5f * (spJx(2) + spJx(3));
		nvgBeginPath(vg);
		nvgMoveTo(vg, dx + 0.5f, SP_JY - 27.f);
		nvgLineTo(vg, dx + 0.5f, SP_JY + 17.f);
		nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 40));
		nvgStrokeWidth(vg, 1.f);
		nvgStroke(vg);
		drawText(vg, spJx(1), SP_JY + 27.f, "CV IN", 6.f, colDim(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);
		drawText(vg, 0.5f * (spJx(3) + spJx(6)), SP_JY + 27.f, "OUT", 6.f, withAlpha(colAccent(), 0.7f),
		         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);
		Widget::draw(args);
	}
};

// Display: rotating 3D view of the attractor trail.
// Drag to rotate by hand, double-click to reset the view.
struct SPDisplay : TransparentWidget {
	Spikes* module = nullptr;

	float yaw = 0.6f, pitch = 0.5f;
	double lastT = 0.0;

	// demo trail for the module browser
	float demoX[SP_TRAIL], demoY[SP_TRAIL], demoZ[SP_TRAIL];

	SPDisplay(Spikes* m) : module(m) {
		box.pos = Vec(SP_DX, SP_DY);
		box.size = Vec(SP_DW, SP_DH);

		if (!module) {
			double s[3] = {0.1, 0.0, 0.0};
			for (int i = 0; i < 2000; ++i)
				Spikes::rk4(s, 0.005, Spikes::lorenzDeriv);
			for (int i = 0; i < SP_TRAIL; ++i) {
				for (int k = 0; k < 3; ++k)
					Spikes::rk4(s, 0.005, Spikes::lorenzDeriv);
				demoX[i] = (float)(s[0] / 20.0);
				demoY[i] = (float)(s[1] / 25.0);
				demoZ[i] = (float)((s[2] - 25.0) / 25.0);
			}
		}
	}

	void onButton(const ButtonEvent& e) override {
		if (e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS)
			e.consume(this);
	}
	void onDragMove(const DragMoveEvent& e) override {
		yaw += e.mouseDelta.x * 0.012f;
		pitch = clamp(pitch + e.mouseDelta.y * 0.012f, -1.45f, 1.45f);
	}
	void onDoubleClick(const DoubleClickEvent& e) override {
		yaw = 0.6f;
		pitch = 0.5f;
	}

	void draw(const DrawArgs& args) override {
		NVGcontext* vg = args.vg;
		screenBase(vg, SP_DW, SP_DH);

		// inner frame
		nvgBeginPath(vg);
		nvgRoundedRect(vg, 3.5f, 13.5f, SP_DW - 7.f, SP_DH - 27.f, 2.f);
		nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
		nvgFill(vg);
		nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 36));
		nvgStrokeWidth(vg, 1.f);
		nvgStroke(vg);
		Widget::draw(args);
	}

	void drawLayer(const DrawArgs& args, int layer) override {
		if (layer != 1) { Widget::drawLayer(args, layer); return; }
		NVGcontext* vg = args.vg;
		const float W = SP_DW, H = SP_DH;

		// ---- gather data ----
		static float ox[SP_TRAIL], oy[SP_TRAIL], oz[SP_TRAIL];   // ordered oldest -> newest
		int count = SP_TRAIL;
		float rot = 0.25f;
		float spd = 1.f, bld = 0.5f, frc = 0.2f;
		float vx = 0.f, vy = 0.f, vz = 0.f, vm = 0.f;

		if (module) {
			int head = module->trailHead;
			count = std::min(module->trailCount, SP_TRAIL);
			for (int i = 0; i < count; ++i) {
				int idx = ((head - (count - 1) + i) % SP_TRAIL + SP_TRAIL) % SP_TRAIL;
				ox[i] = module->trailX[idx];
				oy[i] = module->trailY[idx];
				oz[i] = module->trailZ[idx];
			}
			rot = module->params[Spikes::ROTATE_PARAM].getValue();
			spd = module->dispSpeed; bld = module->dispBlend; frc = module->dispFractal;
			vx = module->outX; vy = module->outY; vz = module->outZ; vm = module->outMix;
		} else {
			for (int i = 0; i < SP_TRAIL; ++i) { ox[i] = demoX[i]; oy[i] = demoY[i]; oz[i] = demoZ[i]; }
			vx = 3.1f; vy = -1.4f; vz = 0.6f; vm = 0.8f;
		}

		// ---- advance the view rotation ----
		double now = system::getTime();
		float dt = (lastT > 0.0) ? clamp((float)(now - lastT), 0.f, 0.1f) : 0.f;
		lastT = now;
		yaw += dt * rot * 1.6f;
		if (yaw > 2.f * M_PI) yaw -= 2.f * M_PI;
		if (yaw < -2.f * M_PI) yaw += 2.f * M_PI;

		// ---- projection ----
		const float cx = W / 2.f, cy = H / 2.f + 1.f, S = 33.f;
		const float cyw = std::cos(yaw), syw = std::sin(yaw);
		const float cp = std::cos(pitch), sp = std::sin(pitch);

		// attractor (x, y, z) -> world (x, -z up, y depth), yaw about the vertical axis, then pitch
		auto proj = [&](float x, float y, float z, float& px, float& py, float& k) {
			float wx = x, wy = -z, wz = y;
			float x1 = wx * cyw + wz * syw;
			float z1 = -wx * syw + wz * cyw;
			float y2 = wy * cp - z1 * sp;
			float z2 = wy * sp + z1 * cp;
			k = 1.f / (1.f + z2 * 0.22f);
			px = cx + x1 * S * k;
			py = cy + y2 * S * k;
		};

		nvgSave(vg);
		nvgScissor(vg, 4.f, 14.f, W - 8.f, H - 28.f);
		nvgLineCap(vg, NVG_ROUND);
		nvgLineJoin(vg, NVG_ROUND);

		// ---- reference cube ----
		{
			float cxs[8], cys[8], ck;
			for (int i = 0; i < 8; ++i) {
				float x = (i & 1) ? 1.f : -1.f;
				float y = (i & 2) ? 1.f : -1.f;
				float z = (i & 4) ? 1.f : -1.f;
				proj(x, y, z, cxs[i], cys[i], ck);
			}
			nvgBeginPath(vg);
			for (int i = 0; i < 8; ++i) {
				for (int bit = 1; bit <= 4; bit <<= 1) {
					int j = i ^ bit;
					if (j > i) {
						nvgMoveTo(vg, cxs[i], cys[i]);
						nvgLineTo(vg, cxs[j], cys[j]);
					}
				}
			}
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 26));
			nvgStrokeWidth(vg, 0.8f);
			nvgStroke(vg);

			// floor grid (z = -1 plane, the bottom of the cube)
			nvgBeginPath(vg);
			for (int g = -1; g <= 1; ++g) {
				float ax, ay, bx, by, k;
				proj((float)g, -1.f, -1.f, ax, ay, k);
				proj((float)g,  1.f, -1.f, bx, by, k);
				nvgMoveTo(vg, ax, ay); nvgLineTo(vg, bx, by);
				proj(-1.f, (float)g, -1.f, ax, ay, k);
				proj( 1.f, (float)g, -1.f, bx, by, k);
				nvgMoveTo(vg, ax, ay); nvgLineTo(vg, bx, by);
			}
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
			nvgStrokeWidth(vg, 0.6f);
			nvgStroke(vg);
		}

		// ---- axes ----
		{
			const float ax[3][3] = {{1.3f, 0.f, 0.f}, {0.f, 1.3f, 0.f}, {0.f, 0.f, 1.3f}};
			const char* an[3] = {"X", "Y", "Z"};
			NVGcolor ac[3] = {colAccent(), colViolet(), nvgRGB(0xe6, 0xe9, 0xef)};
			float ox0, oy0, k0;
			proj(0.f, 0.f, 0.f, ox0, oy0, k0);
			for (int a = 0; a < 3; ++a) {
				float px, py, k;
				proj(ax[a][0], ax[a][1], ax[a][2], px, py, k);
				nvgBeginPath(vg);
				nvgMoveTo(vg, ox0, oy0);
				nvgLineTo(vg, px, py);
				nvgStrokeColor(vg, withAlpha(ac[a], 0.35f));
				nvgStrokeWidth(vg, 0.9f);
				nvgStroke(vg);
				drawText(vg, px, py, an[a], 6.5f, withAlpha(ac[a], 0.85f),
				         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
			}
		}

		// ---- trail ----
		if (count >= 2) {
			float px0, py0, k0;
			proj(ox[0], oy[0], oz[0], px0, py0, k0);
			NVGcolor cOld = nvgRGBA(0xb0, 0x5a, 0xff, 255);
			NVGcolor cNew = nvgRGBA(0x8a, 0xee, 0xff, 255);
			for (int i = 1; i < count; ++i) {
				float px1, py1, k1;
				proj(ox[i], oy[i], oz[i], px1, py1, k1);
				float t = (float)i / (float)(count - 1);      // 0 old .. 1 new
				float a = std::pow(t, 1.6f);
				NVGcolor c = nvgLerpRGBA(cOld, cNew, t);

				// soft glow on the freshest part of the trail
				if (i > count - 70) {
					nvgBeginPath(vg);
					nvgMoveTo(vg, px0, py0);
					nvgLineTo(vg, px1, py1);
					nvgStrokeColor(vg, withAlpha(c, a * 0.18f));
					nvgStrokeWidth(vg, 3.6f * k1);
					nvgStroke(vg);
				}
				nvgBeginPath(vg);
				nvgMoveTo(vg, px0, py0);
				nvgLineTo(vg, px1, py1);
				nvgStrokeColor(vg, withAlpha(c, 0.08f + a * 0.92f));
				nvgStrokeWidth(vg, (0.5f + 1.1f * t) * k1);
				nvgStroke(vg);

				px0 = px1; py0 = py1;
			}

			// head marker
			nvgBeginPath(vg);
			nvgCircle(vg, px0, py0, 5.5f);
			nvgFillPaint(vg, nvgRadialGradient(vg, px0, py0, 0.f, 5.5f, nvgRGBA(0x8a, 0xee, 0xff, 130), nvgRGBA(0x8a, 0xee, 0xff, 0)));
			nvgFill(vg);
			nvgBeginPath(vg);
			nvgCircle(vg, px0, py0, 1.8f);
			nvgFillColor(vg, nvgRGB(255, 255, 255));
			nvgFill(vg);
		}

		nvgResetScissor(vg);
		nvgRestore(vg);

		screenFinish(vg, W, H);

		// ---- readouts ----
		char buf[64];
		NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 210);
		NVGcolor cd = nvgRGBA(0x7a, 0x85, 0x94, 230);

		snprintf(buf, sizeof(buf), "SPD %.2fX", spd);
		drawText(vg, 6.f, 7.f, buf, 7.f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
		snprintf(buf, sizeof(buf), "L/R %3.0f%%", bld * 100.f);
		drawText(vg, W / 2.f, 7.f, buf, 7.f, c1, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
		snprintf(buf, sizeof(buf), "FRC %3.0f%%", frc * 100.f);
		drawText(vg, W - 6.f, 7.f, buf, 7.f, c1, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

		snprintf(buf, sizeof(buf), "X%+.1f", vx);
		drawText(vg, 6.f, H - 6.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
		snprintf(buf, sizeof(buf), "Y%+.1f", vy);
		drawText(vg, 50.f, H - 6.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
		snprintf(buf, sizeof(buf), "Z%+.1f", vz);
		drawText(vg, 94.f, H - 6.f, buf, 7.f, cd, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
		snprintf(buf, sizeof(buf), "MIX %+.1fV", vm);
		drawText(vg, W - 6.f, H - 6.f, buf, 7.f, withAlpha(colViolet(), 0.9f), NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

		Widget::drawLayer(args, layer);
	}
};

} // namespace


struct SpikesWidget : ModuleWidget {
	SpikesWidget(Spikes* module) {
		setModule(module);
		box.size = Vec(SP_W, SP_H);

		addChild(new SPPanel());
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

		addChild(new SPDisplay(module));

		// ---------------- Param knobs (3 x 2 grid) ----------------
		const int ids[6] = {Spikes::SPEED_PARAM, Spikes::BLEND_PARAM, Spikes::FRACTAL_PARAM,
		                    Spikes::FRACTAL_SPEED_PARAM, Spikes::SCALE_PARAM, Spikes::ROTATE_PARAM};
		for (int i = 0; i < 6; ++i)
			addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_KCOL[i % 3], SP_KROW[i / 3]), module, ids[i]));

		// ---------------- CV inputs ----------------
		addInput(createInputCentered<DarkPJ301MPort>(Vec(spJx(0), SP_JY), module, Spikes::SPEED_INPUT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(spJx(1), SP_JY), module, Spikes::BLEND_INPUT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(spJx(2), SP_JY), module, Spikes::FRACTAL_INPUT));

		// ---------------- Outputs ----------------
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(spJx(3), SP_JY), module, Spikes::X_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(spJx(4), SP_JY), module, Spikes::Y_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(spJx(5), SP_JY), module, Spikes::Z_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(spJx(6), SP_JY), module, Spikes::MIX_OUTPUT));
	}
};


Model* modelSpikes = createModel<Spikes, SpikesWidget>("Spikes");