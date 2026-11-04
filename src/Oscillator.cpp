#include "plugin.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <complex>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>

// ------------------------------------------------------------
// Constants
// ------------------------------------------------------------
static const int OS_NW = 30;			// waveforms
static const int OS_TN = 2048;			// table size
static const int OS_LV = 11;			// band-limited mip levels
static const int OS_STRIDE = OS_TN + 1; // + guard point
static const int OS_SIM = 256;			// display simulation length
static const int OS_NH = 64;			// harmonics shown
static const int OS_MAXV = 16;

static const int OS_HARM[OS_LV] = {1023, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1};

static const float OS_PANEL_W = RACK_GRID_WIDTH * 24; // 360
static const float OS_PANEL_H = RACK_GRID_HEIGHT;	  // 380

// Display
static const float OS_DISP_W = 228.f;
static const float OS_DISP_H = 140.f;
static const float OS_DISP_X = (OS_PANEL_W - OS_DISP_W) / 2.f; // 66
static const float OS_DISP_Y = 44.f;

// Display geometry (display-local)
static const float OS_WX0 = 8.f, OS_WX1 = 110.f;   // waveform box
static const float OS_SX0 = 118.f, OS_SX1 = 220.f; // spectrum box
static const float OS_PY0 = 18.f, OS_PY1 = 80.f;
static const float OS_WYC = 49.f, OS_WAMP = 27.f;
static const float OS_DB_TOP = 6.f, OS_DB_BOT = -60.f;
static const float OS_GX0 = 8.f, OS_GY0 = 90.f, OS_GCW = 212.f / 15.f;
static const float OS_GRP = 18.f, OS_GCH = 16.f;

static const float OS_CTRL_SEC_Y = 204.f;
static const float OS_IO_SEC_Y = 283.f;
static const float OS_IO_Y = 327.f;
static inline float osIoX(int i) { return 40.f + 40.f * (float)i; }

// ------------------------------------------------------------
// Waveforms
// ------------------------------------------------------------
static const char *const OS_NAMES[OS_NW] = {
	"Sine", "Triangle", "Saw Up", "Saw Down", "Square", "Pulse 25%", "Pulse 12%",
	"Half Sine", "Abs Sine", "Trapezoid", "Stairs 4", "Stairs 8", "Sine + Octave",
	"Sine + 3rd", "Organ 1", "Organ 2", "Clarinet", "Brass", "Vowel Ah", "Vowel Oo",
	"E-Piano", "Folded Sine", "Clipped Sine", "Soft Saw", "Sync Saw", "Parabola",
	"Saw + Square", "Tri Fold", "Gauss Pulse", "Random Steps"};

static inline double osFrac(double x) { return x - std::floor(x); }
static inline double osSq(double x) { return x * x; }
static double osTri(double p)
{
	p = osFrac(p);
	if (p < 0.25)
		return 4.0 * p;
	if (p < 0.75)
		return 2.0 - 4.0 * p;
	return 4.0 * p - 4.0;
}
static double osSaw(double p) { return 2.0 * osFrac(p + 0.5) - 1.0; }
static double osAdd(double p, const double *a, int n)
{
	double s = 0.0;
	for (int h = 0; h < n; ++h)
		if (a[h] != 0.0)
			s += a[h] * std::sin(6.283185307179586 * (double)(h + 1) * p);
	return s;
}

static double osWaveFn(int w, double p)
{
	const double TAU = 6.283185307179586;
	switch (w)
	{
	case 0:
		return std::sin(TAU * p);
	case 1:
		return osTri(p);
	case 2:
		return osSaw(p);
	case 3:
		return -osSaw(p);
	case 4:
		return p < 0.5 ? 1.0 : -1.0;
	case 5:
		return p < 0.25 ? 1.0 : -1.0;
	case 6:
		return p < 0.125 ? 1.0 : -1.0;
	case 7:
		return std::max(std::sin(TAU * p), 0.0);
	case 8:
		return std::fabs(std::sin(TAU * p));
	case 9:
		return std::min(1.0, std::max(-1.0, 1.6 * osTri(p)));
	case 10:
		return std::floor(p * 4.0) / 3.0 * 2.0 - 1.0;
	case 11:
		return std::floor(p * 8.0) / 7.0 * 2.0 - 1.0;
	case 12:
		return std::sin(TAU * p) + 0.5 * std::sin(2.0 * TAU * p);
	case 13:
		return std::sin(TAU * p) + 0.5 * std::sin(3.0 * TAU * p);
	case 14:
	{
		static const double a[8] = {1, 0.8, 0.6, 0.4, 0, 0.25, 0, 0.2};
		return osAdd(p, a, 8);
	}
	case 15:
	{
		static const double a[8] = {1, 0.6, 0.8, 0.3, 0.5, 0.2, 0, 0.3};
		return osAdd(p, a, 8);
	}
	case 16:
	{
		static const double a[13] = {1, 0, 0.75, 0, 0.5, 0, 0.14, 0, 0.5, 0, 0.12, 0, 0.17};
		return osAdd(p, a, 13);
	}
	case 17:
	{
		static const double a[9] = {1, 0.8, 0.9, 1, 0.7, 0.5, 0.3, 0.2, 0.1};
		return osAdd(p, a, 9);
	}
	case 18:
	{
		double s = 0.0;
		for (int h = 1; h <= 64; ++h)
		{
			double a = (1.0 / h) * (1.0 + 4.0 * std::exp(-osSq((h - 6.0) / 1.6)) + 3.0 * std::exp(-osSq((h - 10.0) / 2.2)));
			s += a * std::sin(TAU * h * p);
		}
		return s;
	}
	case 19:
	{
		double s = 0.0;
		for (int h = 1; h <= 64; ++h)
		{
			double a = (1.0 / h) * (1.0 + 6.0 * std::exp(-osSq((h - 2.5) / 1.0)) + 1.5 * std::exp(-osSq((h - 7.0) / 1.5)));
			s += a * std::sin(TAU * h * p);
		}
		return s;
	}
	case 20:
	{
		static const double a[8] = {1, 0.6, 0.15, 0.4, 0.05, 0.2, 0.02, 0.08};
		return osAdd(p, a, 8);
	}
	case 21:
		return std::sin(2.6 * std::sin(TAU * p));
	case 22:
		return std::min(1.0, std::max(-1.0, 1.8 * std::sin(TAU * p)));
	case 23:
		return std::tanh(2.2 * osSaw(p));
	case 24:
		return 2.0 * osFrac(2.5 * p) - 1.0;
	case 25:
	{
		double q = osFrac(p) - 0.5;
		return 1.0 - 8.0 * q * q;
	}
	case 26:
		return 0.5 * (osSaw(p) + (p < 0.5 ? 1.0 : -1.0));
	case 27:
		return 0.6366197723675814 * std::asin(std::sin(1.5707963267948966 * 2.4 * osTri(p)));
	case 28:
	{
		double d = (p < 0.5) ? p : p - 1.0;
		return std::exp(-osSq(d / 0.07));
	}
	case 29:
	{
		uint32_t h = (uint32_t)std::floor(p * 16.0) * 2654435761u + 0x9e3779b9u;
		h ^= h >> 15;
		h *= 2246822519u;
		h ^= h >> 13;
		h *= 3266489917u;
		h ^= h >> 16;
		return (double)(h & 0xFFFFu) / 32767.5 - 1.0;
	}
	default:
		return 0.0;
	}
}

static void osFFT(std::vector<std::complex<double>> &a, bool inv)
{
	const int n = (int)a.size();
	for (int i = 1, j = 0; i < n; ++i)
	{
		int bit = n >> 1;
		for (; j & bit; bit >>= 1)
			j ^= bit;
		j ^= bit;
		if (i < j)
			std::swap(a[i], a[j]);
	}
	for (int len = 2; len <= n; len <<= 1)
	{
		double ang = 6.283185307179586 / (double)len * (inv ? 1.0 : -1.0);
		std::complex<double> wl(std::cos(ang), std::sin(ang));
		for (int i = 0; i < n; i += len)
		{
			std::complex<double> w(1.0, 0.0);
			for (int j = 0; j < len / 2; ++j)
			{
				std::complex<double> u = a[i + j], v = a[i + j + len / 2] * w;
				a[i + j] = u + v;
				a[i + j + len / 2] = u - v;
				w *= wl;
			}
		}
	}
	if (inv)
		for (auto &x : a)
			x /= (double)n;
}

// Band-limited wavetables: 30 waves x 11 levels (level k keeps the harmonics a
// note at that pitch can have below Nyquist). DC removed, peak normalised to 1.
struct OSTables
{
	std::vector<float> d;
	OSTables() : d((size_t)OS_NW * OS_LV * OS_STRIDE)
	{
		std::vector<std::complex<double>> x(OS_TN), y(OS_TN);
		for (int w = 0; w < OS_NW; ++w)
		{
			for (int i = 0; i < OS_TN; ++i)
				x[i] = osWaveFn(w, ((double)i + 0.5) / (double)OS_TN);
			osFFT(x, false);
			x[0] = 0.0;
			double gain = 1.0;
			for (int k = 0; k < OS_LV; ++k)
			{
				std::fill(y.begin(), y.end(), std::complex<double>(0.0, 0.0));
				for (int h = 1; h <= OS_HARM[k]; ++h)
				{
					y[h] = x[h];
					y[OS_TN - h] = std::conj(x[h]);
				}
				osFFT(y, true);
				float *t = &d[((size_t)w * OS_LV + k) * OS_STRIDE];
				if (k == 0)
				{
					double pk = 1e-9;
					for (int i = 0; i < OS_TN; ++i)
						pk = std::max(pk, std::fabs(y[i].real()));
					gain = 1.0 / pk;
				}
				for (int i = 0; i < OS_TN; ++i)
					t[i] = (float)(y[i].real() * gain);
				t[OS_TN] = t[0];
			}
		}
	}
	const float *level(int w, int k) const { return &d[((size_t)w * OS_LV + k) * OS_STRIDE]; }
};

static const OSTables &osTables()
{
	static const OSTables t;
	return t;
}

// ph in [0, 1); samples were taken at (i + 0.5) / N
static inline float osRead(const float *t, float ph)
{
	float pos = ph * (float)OS_TN - 0.5f;
	if (pos < 0.f)
		pos += (float)OS_TN;
	int i = (int)pos;
	if (i > OS_TN - 1)
		i = OS_TN - 1;
	float fr = pos - (float)i;
	return t[i] + (t[i + 1] - t[i]) * fr;
}

static inline int osLevel(float f, float fs)
{
	float hm = 0.5f * fs / std::max(f, 0.01f);
	int k = 0;
	while (k < OS_LV - 1 && (float)OS_HARM[k] > hm)
		k++;
	return k;
}

// Filter helpers (knob 0..1 -> Hz, 0 = bypassed)
static inline float osLoHz(float v) { return (v < 0.001f) ? 0.f : 20.f * std::pow(100.f, v); }
static inline float osHiHz(float v) { return (v > 0.999f) ? 0.f : 200.f * std::pow(100.f, v); }

static std::string osHzStr(float hz)
{
	char b[32];
	if (hz < 1000.f)
		snprintf(b, sizeof(b), "%.0f Hz", hz);
	else
		snprintf(b, sizeof(b), "%.2f kHz", hz / 1000.f);
	return std::string(b);
}

struct OSLoQ : rack::engine::ParamQuantity
{
	std::string getDisplayValueString() override
	{
		float hz = osLoHz(getValue());
		return hz <= 0.f ? std::string("Off") : osHzStr(hz);
	}
};
struct OSHiQ : rack::engine::ParamQuantity
{
	std::string getDisplayValueString() override
	{
		float hz = osHiHz(getValue());
		return hz <= 0.f ? std::string("Off") : osHzStr(hz);
	}
};

// ------------------------------------------------------------
// Frequency knob: value = semitones from C4 (-48 .. +48 = C0 .. C8)
// The right-click value field accepts note names ("C#3", "Bb2", "a4")
// and frequencies ("440", "880 Hz", "1.2k").
// ------------------------------------------------------------
static const float OS_C4_HZ = 261.6256f;

static std::string osNoteName(int n)
{ // n = semitones from C4
	static const char *nn[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
	int midi = 60 + n;
	int pc = ((midi % 12) + 12) % 12;
	int oct = (int)std::floor((float)midi / 12.f) - 1;
	char b[16];
	snprintf(b, sizeof(b), "%s%d", nn[pc], oct);
	return std::string(b);
}

static bool osParseNote(const std::string &s, int &n)
{
	size_t i = 0, len = s.size();
	while (i < len && std::isspace((unsigned char)s[i]))
		i++;
	if (i >= len)
		return false;
	char c = (char)std::toupper((unsigned char)s[i]);
	if (c < 'A' || c > 'G')
		return false;
	static const int pcs[7] = {9, 11, 0, 2, 4, 5, 7}; // A B C D E F G
	int pc = pcs[c - 'A'];
	i++;
	int acc = 0;
	while (i < len && (s[i] == '#' || s[i] == 'b'))
	{ // flat = lowercase b only
		acc += (s[i] == '#') ? 1 : -1;
		i++;
	}
	while (i < len && std::isspace((unsigned char)s[i]))
		i++;
	bool neg = false;
	if (i < len && s[i] == '-')
	{
		neg = true;
		i++;
	}
	int oct = 4;
	if (i < len && std::isdigit((unsigned char)s[i]))
	{
		int v = 0;
		while (i < len && std::isdigit((unsigned char)s[i]))
		{
			v = std::min(v * 10 + (s[i] - '0'), 99);
			i++;
		}
		oct = neg ? -v : v;
	}
	else if (neg)
	{
		return false;
	}
	while (i < len && std::isspace((unsigned char)s[i]))
		i++;
	if (i < len)
		return false; // trailing garbage
	n = pc + acc + 12 * (oct - 4);
	return true;
}

struct OSFreqQ : rack::engine::ParamQuantity
{
	std::string getDisplayValueString() override
	{
		return osNoteName((int)std::lround(getValue()));
	}
	std::string getUnit() override
	{
		float hz = OS_C4_HZ * std::exp2((float)std::lround(getValue()) / 12.f);
		char b[32];
		snprintf(b, sizeof(b), " (%.2f Hz)", hz);
		return std::string(b);
	}
	void setDisplayValueString(std::string s) override
	{
		int n = 0;
		if (osParseNote(s, n))
		{
			setValue((float)rack::math::clamp(n, -48, 48));
			return;
		}
		// frequency in Hz (or kHz)
		const char *st = s.c_str();
		char *end = nullptr;
		double f = std::strtod(st, &end);
		if (end == st || !(f > 0.0))
			return;
		std::string rest(end);
		for (auto &ch : rest)
			ch = (char)std::tolower((unsigned char)ch);
		size_t k = 0;
		while (k < rest.size() && std::isspace((unsigned char)rest[k]))
			k++;
		if (k < rest.size() && rest[k] == 'k')
			f *= 1000.0;
		double semis = 12.0 * std::log2(f / (double)OS_C4_HZ);
		setValue((float)rack::math::clamp((int)std::lround(semis), -48, 48));
	}
};

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
struct Oscillator : Module
{
	enum ParamId
	{
		WAVE_PARAM,
		COARSE_PARAM,
		DETUNE_PARAM,
		FB_PARAM,
		FM_PARAM,
		LOCUT_PARAM,
		HICUT_PARAM,
		LEVEL_PARAM,
		FREQ_PARAM,
		PARAMS_LEN
	};
	enum InputId
	{
		VOCT_INPUT,
		FM_INPUT,
		FB_CV_INPUT,
		WAVE_CV_INPUT,
		LOCUT_CV_INPUT,
		HICUT_CV_INPUT,
		INPUTS_LEN
	};
	enum OutputId
	{
		OUT_OUTPUT,
		SUM_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId
	{
		LIGHTS_LEN
	};

	const OSTables *tab = nullptr;

	// Voice state
	float phase[OS_MAXV] = {};
	float fb1[OS_MAXV] = {}, fb2[OS_MAXV] = {};
	float hpA[OS_MAXV] = {}, hpB[OS_MAXV] = {};
	float lpA[OS_MAXV] = {}, lpB[OS_MAXV] = {};
	int lastN = 0;
	int waveCur = 0, wavePrev = 0;
	float xfade = 0.f;

	// Filter coefficient cache
	float cLoHz = -1.f, cHiHz = -1.f, cFs = 0.f;
	float loA1 = 0.f, loA2 = 0.f, loA3 = 0.f;
	float hiA1 = 0.f, hiA2 = 0.f, hiA3 = 0.f;

	// Display data
	int vWave = 0, vN = 1;
	float vF0 = 261.6256f, vFb = 0.f, vLo = 0.f, vHi = 0.f, vFs = 44100.f;
	float vPhase[OS_MAXV] = {};
	float pk = 0.f;

	Oscillator()
	{
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

		std::vector<std::string> names;
		for (int i = 0; i < OS_NW; ++i)
			names.push_back(OS_NAMES[i]);
		configSwitch(WAVE_PARAM, 0.f, (float)(OS_NW - 1), 0.f, "Waveform", names);

		configParam(COARSE_PARAM, -36.f, 36.f, 0.f, "Coarse tune (transpose)", " semitones");
		paramQuantities[COARSE_PARAM]->snapEnabled = true;
		configParam(DETUNE_PARAM, -100.f, 100.f, 0.f, "Detune", " cents");
		configParam(FB_PARAM, 0.f, 1.f, 0.f, "Feedback", "%", 0.f, 100.f);
		configParam(FM_PARAM, 0.f, 1.f, 0.f, "FM depth", "%", 0.f, 100.f);
		configParam<OSLoQ>(LOCUT_PARAM, 0.f, 1.f, 0.f, "Low cut");
		configParam<OSHiQ>(HICUT_PARAM, 0.f, 1.f, 1.f, "High cut");
		configParam(LEVEL_PARAM, 0.f, 1.5f, 1.f, "Output level", "%", 0.f, 100.f);
		configParam<OSFreqQ>(FREQ_PARAM, -48.f, 48.f, 0.f, "Frequency");
		paramQuantities[FREQ_PARAM]->snapEnabled = true;

		configInput(VOCT_INPUT, "V/Oct (polyphonic)");
		configInput(FM_INPUT, "FM (phase modulation, polyphonic)");
		configInput(FB_CV_INPUT, "Feedback CV (0-10 V, polyphonic)");
		configInput(WAVE_CV_INPUT, "Waveform CV (0-10 V sweeps all waves)");
		configInput(LOCUT_CV_INPUT, "Low cut CV (0-10 V)");
		configInput(HICUT_CV_INPUT, "High cut CV (0-10 V)");

		configOutput(OUT_OUTPUT, "Polyphonic");
		configOutput(SUM_OUTPUT, "Sum of all voices");

		tab = &osTables();
	}

	void onReset() override
	{
		std::memset(phase, 0, sizeof(phase));
		std::memset(fb1, 0, sizeof(fb1));
		std::memset(fb2, 0, sizeof(fb2));
		std::memset(hpA, 0, sizeof(hpA));
		std::memset(hpB, 0, sizeof(hpB));
		std::memset(lpA, 0, sizeof(lpA));
		std::memset(lpB, 0, sizeof(lpB));
		xfade = 0.f;
		lastN = 0;
		pk = 0.f;
	}

	static inline void svfCoef(float fc, float fs, float k, float &a1, float &a2, float &a3)
	{
		float g = std::tan((float)M_PI * fc / fs);
		a1 = 1.f / (1.f + g * (g + k));
		a2 = g * a1;
		a3 = g * a2;
	}

	void process(const ProcessArgs &args) override
	{
		const float fs = args.sampleRate;
		const float dt = args.sampleTime;
		const float K = 1.41421356f; // Butterworth

		int nV = inputs[VOCT_INPUT].isConnected() ? inputs[VOCT_INPUT].getChannels() : 1;
		nV = rack::math::clamp(nV, 1, OS_MAXV);
		for (int c = lastN; c < nV; ++c)
		{
			phase[c] = 0.f;
			fb1[c] = fb2[c] = 0.f;
			hpA[c] = hpB[c] = lpA[c] = lpB[c] = 0.f;
		}
		lastN = nV;

		// ---- waveform (with a short crossfade on change) ----
		float wv = params[WAVE_PARAM].getValue() + inputs[WAVE_CV_INPUT].getVoltage() * 0.1f * (float)(OS_NW - 1);
		int wave = rack::math::clamp((int)std::lround(wv), 0, OS_NW - 1);
		if (wave != waveCur)
		{
			wavePrev = waveCur;
			waveCur = wave;
			xfade = 1.f;
		}

		// ---- parameters ----
		// FREQ is in semitones from C4, so its default (0) gives C4 = 261.63 Hz at 0 V
		float pitchBase = (params[FREQ_PARAM].getValue() + params[COARSE_PARAM].getValue()) / 12.f + params[DETUNE_PARAM].getValue() / 1200.f;
		float fbKnob = params[FB_PARAM].getValue();
		float fmAmt = params[FM_PARAM].getValue();
		float level = params[LEVEL_PARAM].getValue();

		float loK = rack::math::clamp(params[LOCUT_PARAM].getValue() + inputs[LOCUT_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
		float hiK = rack::math::clamp(params[HICUT_PARAM].getValue() + inputs[HICUT_CV_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
		float loHz = osLoHz(loK);
		float hiHz = osHiHz(hiK);
		if (hiHz > 0.f)
			hiHz = std::min(hiHz, 0.45f * fs);
		if (loHz > 0.f)
			loHz = std::min(loHz, 0.45f * fs);

		if (loHz != cLoHz || fs != cFs)
		{
			cLoHz = loHz;
			if (loHz > 0.f)
				svfCoef(loHz, fs, K, loA1, loA2, loA3);
		}
		if (hiHz != cHiHz || fs != cFs)
		{
			cHiHz = hiHz;
			if (hiHz > 0.f)
				svfCoef(hiHz, fs, K, hiA1, hiA2, hiA3);
		}
		cFs = fs;

		const bool fmOn = inputs[FM_INPUT].isConnected();
		const bool vOn = inputs[VOCT_INPUT].isConnected();

		outputs[OUT_OUTPUT].setChannels(nV);
		float sum = 0.f;
		float peak = 0.f;

		for (int c = 0; c < nV; ++c)
		{
			float pitch = (vOn ? inputs[VOCT_INPUT].getVoltage(c) : 0.f) + pitchBase;
			pitch = rack::math::clamp(pitch, -8.f, 8.f);
			float f = rack::math::clamp(OS_C4_HZ * std::exp2(pitch), 0.05f, 0.49f * fs);

			float fb = rack::math::clamp(fbKnob + inputs[FB_CV_INPUT].getPolyVoltage(c) * 0.1f, 0.f, 1.f);
			float fm = fmOn ? inputs[FM_INPUT].getPolyVoltage(c) : 0.f;

			// phase offset in cycles: feedback (average of the last 2 outputs) + FM
			float off = fb * fb * 0.375f * (fb1[c] + fb2[c]) + fm * fmAmt * 0.2f;
			float ph = phase[c] + off;
			ph -= std::floor(ph);

			int k = osLevel(f, fs);
			float y = osRead(tab->level(waveCur, k), ph);
			if (xfade > 0.f)
				y += (osRead(tab->level(wavePrev, k), ph) - y) * xfade;

			fb2[c] = fb1[c];
			fb1[c] = y;
			phase[c] += f / fs;
			if (phase[c] >= 1.f)
				phase[c] -= 1.f;

			// low cut (high-pass), then high cut (low-pass), 12 dB/oct each
			if (loHz > 0.f)
			{
				float v3 = y - hpB[c];
				float v1 = loA1 * hpA[c] + loA2 * v3;
				float v2 = hpB[c] + loA2 * hpA[c] + loA3 * v3;
				hpA[c] = 2.f * v1 - hpA[c];
				hpB[c] = 2.f * v2 - hpB[c];
				y = y - K * v1 - v2;
			}
			if (hiHz > 0.f)
			{
				float v3 = y - lpB[c];
				float v1 = hiA1 * lpA[c] + hiA2 * v3;
				float v2 = lpB[c] + hiA2 * lpA[c] + hiA3 * v3;
				lpA[c] = 2.f * v1 - lpA[c];
				lpB[c] = 2.f * v2 - lpB[c];
				y = v2;
			}

			float o = y * 5.f * level;
			if (!std::isfinite(o))
			{
				o = 0.f;
				fb1[c] = fb2[c] = 0.f;
				hpA[c] = hpB[c] = lpA[c] = lpB[c] = 0.f;
			}
			outputs[OUT_OUTPUT].setVoltage(o, c);
			sum += o;
			peak = std::max(peak, std::fabs(o));

			vPhase[c] = phase[c];
			if (c == 0)
			{
				vF0 = f;
				vFb = fb;
			}
		}
		outputs[SUM_OUTPUT].setVoltage(sum);

		xfade = std::max(0.f, xfade - dt / 0.008f);

		vWave = waveCur;
		vN = nV;
		vLo = loHz;
		vHi = hiHz;
		vFs = fs;
		pk = std::max(peak, pk * std::exp(-dt / 0.5f));
	}
};

// ------------------------------------------------------------
// Knob layout: two columns beside the display, then a row of five
// (FREQ sits at the left end of the control row)
// ------------------------------------------------------------
struct OSKnob
{
	int param;
	float x, y;
	int ticks;
	bool bipolar;
	const char *label;
};
static const OSKnob OS_KNOBS[9] = {
	{Oscillator::WAVE_PARAM, 28.f, 84.f, 15, false, "WAVE"},
	{Oscillator::COARSE_PARAM, 28.f, 150.f, 9, true, "COARSE"},
	{Oscillator::DETUNE_PARAM, 332.f, 84.f, 9, true, "DETUNE"},
	{Oscillator::FB_PARAM, 332.f, 150.f, 11, false, "FEEDBACK"},
	{Oscillator::FREQ_PARAM, 40.f, 236.f, 9, true, "FREQ"},
	{Oscillator::FM_PARAM, 110.f, 236.f, 11, false, "FM"},
	{Oscillator::LOCUT_PARAM, 180.f, 236.f, 11, false, "LO CUT"},
	{Oscillator::HICUT_PARAM, 250.f, 236.f, 11, false, "HI CUT"},
	{Oscillator::LEVEL_PARAM, 320.f, 236.f, 11, false, "LEVEL"},
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
	NVGcolor colBright() { return nvgRGB(0x7a, 0xec, 0xff); }
	NVGcolor colWarm() { return nvgRGB(0xff, 0x9c, 0x6a); }
	NVGcolor colViolet() { return nvgRGB(0xb0, 0x5a, 0xff); }

	NVGcolor withA(NVGcolor c, float a) { return nvgRGBAf(c.r, c.g, c.b, a); }

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

	void noteString(float f, char *buf, size_t n)
	{
		static const char *nn[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
		float m = 69.f + 12.f * std::log2(std::max(f, 0.01f) / 440.f);
		int mi = (int)std::lround(m);
		int pc = ((mi % 12) + 12) % 12;
		int oct = (int)std::floor((float)mi / 12.f) - 1;
		if (f >= 1000.f)
			snprintf(buf, n, "%s%d %.2fkHz", nn[pc], oct, f / 1000.f);
		else
			snprintf(buf, n, "%s%d %.1fHz", nn[pc], oct, f);
	}

	// ------------------------------------------------------------
	// Panel background
	// ------------------------------------------------------------
	struct OSPanelBackground : TransparentWidget
	{
		OSPanelBackground() { box.size = Vec(OS_PANEL_W, OS_PANEL_H); }

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
			float x1 = 12.f, x2 = OS_PANEL_W - 12.f, cx = OS_PANEL_W / 2.f;
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

		void cell(NVGcontext *vg, float cx, float cy, const char *txt, bool isOutput)
		{
			const float w = 32.f, top = -27.f, h = 44.f;
			float x = cx - w / 2.f, y = cy + top;
			plate(vg, x, y, w, h);

			NVGcolor c = isOutput ? nvgRGBA(0x3d, 0xe0, 0xff, 170) : nvgRGBA(0xff, 0x9c, 0x6a, 170);
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
			nvgRect(vg, 0, 0, OS_PANEL_W, OS_PANEL_H);
			nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, OS_PANEL_H, colBgTop(), colBgBottom()));
			nvgFill(vg);

			nvgStrokeWidth(vg, 1.f);
			for (int y = 0; y < (int)OS_PANEL_H; y += 3)
			{
				nvgBeginPath(vg);
				nvgMoveTo(vg, 0, y + 0.5f);
				nvgLineTo(vg, OS_PANEL_W, y + 0.5f);
				nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
				nvgStroke(vg);
			}

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, OS_PANEL_W, OS_PANEL_H);
			nvgFillPaint(vg, nvgRadialGradient(vg, OS_PANEL_W / 2.f, OS_PANEL_H * 0.4f,
											   OS_PANEL_H * 0.35f, OS_PANEL_H * 0.85f,
											   nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 90)));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRect(vg, 0.5f, 0.5f, OS_PANEL_W - 1.f, OS_PANEL_H - 1.f);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, OS_PANEL_W, 2.f);
			nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, OS_PANEL_W, 0,
											   nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
			nvgFill(vg);

			drawLogo(vg, OS_PANEL_W / 2.f, 15.f);
			drawText(vg, OS_PANEL_W / 2.f, 32.f, "OSCILLATOR", 12.f, colText(),
					 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);

			// ---- Display bezel ----
			float bx = OS_DISP_X - 5.f, by = OS_DISP_Y - 5.f;
			float bw = OS_DISP_W + 10.f, bh = OS_DISP_H + 10.f;

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
			section(vg, OS_CTRL_SEC_Y, "CONTROL");
			for (int k = 0; k < 9; ++k)
			{
				const OSKnob &kn = OS_KNOBS[k];
				ring(vg, kn.x, kn.y, 17.f);
				drawKnobTicks(vg, kn.x, kn.y, 21.f, 24.f, kn.ticks);
				label(vg, kn.x, kn.y + 29.f, kn.label);
			}

			// ---- I/O ----
			section(vg, OS_IO_SEC_Y, "I / O");
			const char *io[8] = {"V/OCT", "FM", "FB CV", "WAVE", "LO CV", "HI CV", "POLY", "SUM"};
			for (int i = 0; i < 8; ++i)
				cell(vg, osIoX(i), OS_IO_Y, io[i], i >= 6);

			drawText(vg, OS_PANEL_W / 2.f, 366.f, "POLYPHONIC VCO  -  30 WAVES  -  FEEDBACK", 6.f,
					 nvgRGBA(0x8a, 0x95, 0xa5, 120), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);

			Widget::draw(args);
		}
	};

	// ------------------------------------------------------------
	// Live value arcs around the knobs
	// ------------------------------------------------------------
	struct OSKnobArcs : TransparentWidget
	{
		Oscillator *module = nullptr;

		OSKnobArcs(Oscillator *m) : module(m) { box.size = Vec(OS_PANEL_W, OS_PANEL_H); }

		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			const float R = 17.2f;
			const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
			const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

			for (int k = 0; k < 9; ++k)
			{
				const OSKnob &kn = OS_KNOBS[k];

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
	// Display: waveform + spectrum with filter curve + clickable wave bank
	// ------------------------------------------------------------
	struct OSDisplayWidget : Widget
	{
		Oscillator *module = nullptr;
		int hoverIdx = -1;

		// thumbnails of all waves
		float thumb[OS_NW][24];

		// simulation cache (waveform with feedback, and its harmonics)
		int cWave = -1, cFb = -1;
		float curve[OS_SIM] = {};
		float hAmp[OS_NH + 1] = {};
		float cosT[OS_SIM], sinT[OS_SIM];

		OSDisplayWidget(Oscillator *m) : module(m)
		{
			box.pos = Vec(OS_DISP_X, OS_DISP_Y);
			box.size = Vec(OS_DISP_W, OS_DISP_H);

			const OSTables &tb = osTables();
			for (int w = 0; w < OS_NW; ++w)
			{
				const float *t = tb.level(w, 4);
				for (int j = 0; j < 24; ++j)
					thumb[w][j] = osRead(t, (float)j / 24.f);
			}
			for (int i = 0; i < OS_SIM; ++i)
			{
				cosT[i] = std::cos(2.f * (float)M_PI * (float)i / (float)OS_SIM);
				sinT[i] = std::sin(2.f * (float)M_PI * (float)i / (float)OS_SIM);
			}
		}

		// ---- hit testing ----
		int galleryAt(Vec p) const
		{
			if (p.x < OS_GX0 || p.x >= OS_GX0 + 212.f)
				return -1;
			float ry = p.y - OS_GY0;
			if (ry < 0.f || ry >= 2.f * OS_GRP)
				return -1;
			int row = (int)(ry / OS_GRP);
			if (ry - (float)row * OS_GRP > OS_GCH)
				return -1;
			int col = (int)((p.x - OS_GX0) / OS_GCW);
			int idx = row * 15 + col;
			return (idx >= 0 && idx < OS_NW) ? idx : -1;
		}

		void onButton(const ButtonEvent &e) override
		{
			if (module && e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS)
			{
				int i = galleryAt(e.pos);
				if (i >= 0)
				{
					module->params[Oscillator::WAVE_PARAM].setValue((float)i);
					e.consume(this);
					return;
				}
			}
			Widget::onButton(e);
		}

		void onHover(const HoverEvent &e) override
		{
			hoverIdx = galleryAt(e.pos);
			if (hoverIdx >= 0)
				e.consume(this);
			else
				Widget::onHover(e);
		}

		void onLeave(const LeaveEvent &e) override
		{
			hoverIdx = -1;
			Widget::onLeave(e);
		}

		// ---- coordinate helpers ----
		static float dbY(float db)
		{
			db = rack::math::clamp(db, OS_DB_BOT, OS_DB_TOP);
			return OS_PY1 - (db - OS_DB_BOT) / (OS_DB_TOP - OS_DB_BOT) * (OS_PY1 - OS_PY0);
		}
		static float freqX(float f)
		{
			f = rack::math::clamp(f, 20.f, 20000.f);
			return OS_SX0 + (std::log10(f) - 1.30103f) / 3.f * (OS_SX1 - OS_SX0);
		}

		// ---- simulate one cycle with the same feedback law as the DSP ----
		void simulate(int wave, float fb)
		{
			int key = (int)std::lround(fb * 1000.f);
			if (wave == cWave && key == cFb)
				return;
			cWave = wave;
			cFb = key;

			const float *t = osTables().level(wave, 3); // 128 harmonics: alias-free at 256 points
			float p1 = 0.f, p2 = 0.f, ph = 0.f;
			float k = fb * fb * 0.375f;
			for (int cyc = 0; cyc < 4; ++cyc)
			{
				for (int i = 0; i < OS_SIM; ++i)
				{
					float q = ph + k * (p1 + p2);
					q -= std::floor(q);
					float y = osRead(t, q);
					p2 = p1;
					p1 = y;
					ph += 1.f / (float)OS_SIM;
					if (ph >= 1.f)
						ph -= 1.f;
					if (cyc == 3)
						curve[i] = y;
				}
			}
			for (int h = 1; h <= OS_NH; ++h)
			{
				float re = 0.f, im = 0.f;
				for (int i = 0; i < OS_SIM; ++i)
				{
					int idx = (h * i) & (OS_SIM - 1);
					re += curve[i] * cosT[idx];
					im += curve[i] * sinT[idx];
				}
				hAmp[h] = 2.f / (float)OS_SIM * std::sqrt(re * re + im * im);
			}
		}

		// ---- static background ----
		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;

			nvgBeginPath(vg);
			nvgRoundedRect(vg, 0, 0, OS_DISP_W, OS_DISP_H, 4.f);
			nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, OS_DISP_H,
											   nvgRGB(0x0a, 0x16, 0x1f), nvgRGB(0x03, 0x06, 0x09)));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRoundedRect(vg, 0.5f, 0.5f, OS_DISP_W - 1.f, OS_DISP_H - 1.f, 4.f);
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			// Waveform box + grid
			nvgBeginPath(vg);
			nvgRoundedRect(vg, OS_WX0, OS_PY0, OS_WX1 - OS_WX0, OS_PY1 - OS_PY0, 3.f);
			nvgFillColor(vg, nvgRGBA(255, 255, 255, 5));
			nvgFill(vg);
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
			for (int i = 1; i < 4; ++i)
			{
				float x = OS_WX0 + (OS_WX1 - OS_WX0) * (float)i / 4.f;
				nvgBeginPath(vg);
				nvgMoveTo(vg, x + 0.5f, OS_PY0 + 1.f);
				nvgLineTo(vg, x + 0.5f, OS_PY1 - 1.f);
				nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 12));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}
			for (int i = -1; i <= 1; ++i)
			{
				float y = OS_WYC - (float)i * OS_WAMP;
				nvgBeginPath(vg);
				nvgMoveTo(vg, OS_WX0 + 1.f, y + 0.5f);
				nvgLineTo(vg, OS_WX1 - 1.f, y + 0.5f);
				nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, i == 0 ? 42 : 12));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}

			// Spectrum box + grid
			nvgBeginPath(vg);
			nvgRoundedRect(vg, OS_SX0, OS_PY0, OS_SX1 - OS_SX0, OS_PY1 - OS_PY0, 3.f);
			nvgFillColor(vg, nvgRGBA(255, 255, 255, 5));
			nvgFill(vg);
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);

			const float gf[9] = {50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
			for (int i = 0; i < 9; ++i)
			{
				bool major = (gf[i] == 100.f || gf[i] == 1000.f || gf[i] == 10000.f);
				float x = freqX(gf[i]);
				nvgBeginPath(vg);
				nvgMoveTo(vg, x + 0.5f, OS_PY0 + 1.f);
				nvgLineTo(vg, x + 0.5f, OS_PY1 - 1.f);
				nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, major ? 30 : 12));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}
			const float gd[3] = {0.f, -20.f, -40.f};
			for (int i = 0; i < 3; ++i)
			{
				float y = dbY(gd[i]);
				nvgBeginPath(vg);
				nvgMoveTo(vg, OS_SX0 + 1.f, y + 0.5f);
				nvgLineTo(vg, OS_SX1 - 1.f, y + 0.5f);
				nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, i == 0 ? 36 : 12));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);
			}
			NVGcolor lc = nvgRGBA(0x8a, 0x95, 0xa5, 170);
			drawText(vg, freqX(100.f), 85.f, "100", 5.f, lc, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
			drawText(vg, freqX(1000.f), 85.f, "1k", 5.f, lc, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);
			drawText(vg, freqX(10000.f), 85.f, "10k", 5.f, lc, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.f);

			// Gallery backdrop
			nvgBeginPath(vg);
			nvgRoundedRect(vg, OS_GX0 - 2.f, OS_GY0 - 2.f, 216.f, 2.f * OS_GRP + 2.f, 3.f);
			nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
			nvgFill(vg);

			Widget::draw(args);
		}

		// ---- live content ----
		void drawLayer(const DrawArgs &args, int layer) override
		{
			if (layer != 1)
			{
				Widget::drawLayer(args, layer);
				return;
			}
			NVGcontext *vg = args.vg;

			// ----- snapshot -----
			int wave = 2, nV = 3;
			float fb = 0.f, f0 = 220.f, lo = 0.f, hi = 0.f, fs = 44100.f, pk = 3.5f;
			float ph[OS_MAXV] = {};
			if (module)
			{
				wave = module->vWave;
				nV = module->vN;
				fb = module->vFb;
				f0 = module->vF0;
				lo = module->vLo;
				hi = module->vHi;
				fs = module->vFs;
				pk = module->pk;
				for (int i = 0; i < OS_MAXV; ++i)
					ph[i] = module->vPhase[i];
			}
			else
			{
				ph[0] = 0.3f;
				ph[1] = 0.55f;
				ph[2] = 0.8f;
				hi = 4000.f;
			}
			wave = rack::math::clamp(wave, 0, OS_NW - 1);
			nV = rack::math::clamp(nV, 1, OS_MAXV);
			simulate(wave, fb);

			nvgSave(vg);
			nvgScissor(vg, 0, 0, OS_DISP_W, OS_DISP_H);

			// ================= waveform =================
			{
				nvgSave(vg);
				nvgScissor(vg, OS_WX0, OS_PY0, OS_WX1 - OS_WX0, OS_PY1 - OS_PY0);
				const float ww = OS_WX1 - OS_WX0 - 4.f;
				const float wx = OS_WX0 + 2.f;
				auto yOf = [&](float v)
				{ return OS_WYC - rack::math::clamp(v, -1.15f, 1.15f) * OS_WAMP; };

				// fill
				nvgBeginPath(vg);
				nvgMoveTo(vg, wx, OS_WYC);
				for (int i = 0; i < OS_SIM; ++i)
					nvgLineTo(vg, wx + (float)i / (float)(OS_SIM - 1) * ww, yOf(curve[i]));
				nvgLineTo(vg, wx + ww, OS_WYC);
				nvgClosePath(vg);
				nvgFillColor(vg, withA(colAccent(), 0.10f));
				nvgFill(vg);

				nvgLineCap(vg, NVG_ROUND);
				nvgLineJoin(vg, NVG_ROUND);
				for (int pass = 0; pass < 2; ++pass)
				{
					nvgBeginPath(vg);
					for (int i = 0; i < OS_SIM; ++i)
					{
						float x = wx + (float)i / (float)(OS_SIM - 1) * ww;
						float y = yOf(curve[i]);
						if (i == 0)
							nvgMoveTo(vg, x, y);
						else
							nvgLineTo(vg, x, y);
					}
					if (pass == 0)
					{
						nvgStrokeColor(vg, withA(colAccent(), 0.16f));
						nvgStrokeWidth(vg, 4.f);
					}
					else
					{
						nvgStrokeColor(vg, withA(colBright(), 0.95f));
						nvgStrokeWidth(vg, 1.3f);
					}
					nvgStroke(vg);
				}

				// voice phase cursors
				for (int c = nV - 1; c >= 0; --c)
				{
					float p = ph[c] - std::floor(ph[c]);
					int idx = rack::math::clamp((int)(p * (float)OS_SIM), 0, OS_SIM - 1);
					float x = wx + p * ww, y = yOf(curve[idx]);
					if (c == 0)
					{
						nvgBeginPath(vg);
						nvgMoveTo(vg, x + 0.5f, OS_PY0 + 1.f);
						nvgLineTo(vg, x + 0.5f, OS_PY1 - 1.f);
						nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 50));
						nvgStrokeWidth(vg, 1.f);
						nvgStroke(vg);
						nvgBeginPath(vg);
						nvgCircle(vg, x, y, 5.f);
						nvgFillColor(vg, withA(colBright(), 0.25f));
						nvgFill(vg);
						nvgBeginPath(vg);
						nvgCircle(vg, x, y, 2.4f);
						nvgFillColor(vg, nvgRGBAf(1.f, 1.f, 1.f, 0.97f));
						nvgFill(vg);
					}
					else
					{
						nvgBeginPath(vg);
						nvgCircle(vg, x, y, 1.7f);
						nvgFillColor(vg, withA(colWarm(), 0.85f));
						nvgFill(vg);
					}
				}
				nvgRestore(vg);

				char b[32];
				snprintf(b, sizeof(b), "WAVEFORM  FB %.0f%%", fb * 100.f);
				drawText(vg, OS_WX0, 85.f, b, 5.2f, nvgRGBA(0x8a, 0x95, 0xa5, 190),
						 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
			}

			// ================= spectrum + filter curve =================
			{
				nvgSave(vg);
				nvgScissor(vg, OS_SX0, OS_PY0, OS_SX1 - OS_SX0, OS_PY1 - OS_PY0);

				auto filtPow = [&](float f)
				{
					float g2 = 1.f;
					if (lo > 0.f)
					{
						float r = f / lo;
						float r4 = r * r * r * r;
						g2 *= r4 / (1.f + r4);
					}
					if (hi > 0.f)
					{
						float r = f / hi;
						float r4 = r * r * r * r;
						g2 *= 1.f / (1.f + r4);
					}
					return g2;
				};

				// Nyquist marker
				float nx = freqX(0.5f * fs);
				nvgBeginPath(vg);
				nvgMoveTo(vg, nx + 0.5f, OS_PY0);
				nvgLineTo(vg, nx + 0.5f, OS_PY1);
				nvgStrokeColor(vg, withA(colWarm(), 0.35f));
				nvgStrokeWidth(vg, 1.f);
				nvgStroke(vg);

				// harmonics: dim = raw, bright = after the filters
				float nyq = 0.5f * fs;
				for (int h = 1; h <= OS_NH; ++h)
				{
					float f = (float)h * f0;
					if (f < 20.f || f > 20000.f || f > nyq)
						continue;
					float db = 20.f * std::log10(std::max(hAmp[h], 1e-6f));
					if (db <= OS_DB_BOT)
						continue;
					float x = freqX(f);
					float yR = dbY(db);
					nvgBeginPath(vg);
					nvgRect(vg, x - 1.f, yR, 2.f, OS_PY1 - yR);
					nvgFillColor(vg, withA(colAccent(), 0.20f));
					nvgFill(vg);

					float dbF = db + 10.f * std::log10(std::max(filtPow(f), 1e-9f));
					if (dbF > OS_DB_BOT)
					{
						float yF = dbY(dbF);
						nvgBeginPath(vg);
						nvgRect(vg, x - 1.f, yF, 2.f, OS_PY1 - yF);
						nvgFillPaint(vg, nvgLinearGradient(vg, 0, OS_PY1, 0, yF,
														   withA(colAccent(), 0.30f), withA(colAccent(), 0.95f)));
						nvgFill(vg);
						nvgBeginPath(vg);
						nvgRect(vg, x - 1.5f, yF - 0.5f, 3.f, 1.4f);
						nvgFillColor(vg, withA(colBright(), 0.95f));
						nvgFill(vg);
					}
				}

				// filter response curve (0 dB reference)
				const int NP = 56;
				nvgLineCap(vg, NVG_ROUND);
				nvgLineJoin(vg, NVG_ROUND);
				for (int pass = 0; pass < 2; ++pass)
				{
					nvgBeginPath(vg);
					for (int i = 0; i < NP; ++i)
					{
						float t = (float)i / (float)(NP - 1);
						float f = 20.f * std::pow(1000.f, t);
						float db = 10.f * std::log10(std::max(filtPow(f), 1e-9f));
						float x = OS_SX0 + t * (OS_SX1 - OS_SX0);
						float y = dbY(db);
						if (i == 0)
							nvgMoveTo(vg, x, y);
						else
							nvgLineTo(vg, x, y);
					}
					if (pass == 0)
					{
						nvgStrokeColor(vg, withA(colViolet(), 0.18f));
						nvgStrokeWidth(vg, 3.5f);
					}
					else
					{
						nvgStrokeColor(vg, withA(colViolet(), 0.95f));
						nvgStrokeWidth(vg, 1.1f);
					}
					nvgStroke(vg);
				}

				// cutoff markers
				if (lo > 0.f)
				{
					float x = freqX(lo);
					nvgBeginPath(vg);
					nvgCircle(vg, x, dbY(-3.f), 2.2f);
					nvgFillColor(vg, withA(colViolet(), 0.95f));
					nvgFill(vg);
				}
				if (hi > 0.f)
				{
					float x = freqX(hi);
					nvgBeginPath(vg);
					nvgCircle(vg, x, dbY(-3.f), 2.2f);
					nvgFillColor(vg, withA(colViolet(), 0.95f));
					nvgFill(vg);
				}
				nvgRestore(vg);

				drawText(vg, OS_SX0 + 3.f, OS_PY0 + 5.f, "SPECTRUM", 5.2f, nvgRGBA(0x8a, 0x95, 0xa5, 190),
						 NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.8f);
			}

			// ================= wave bank =================
			for (int i = 0; i < OS_NW; ++i)
			{
				int row = i / 15, col = i % 15;
				float x0 = OS_GX0 + (float)col * OS_GCW + 0.5f;
				float y0 = OS_GY0 + (float)row * OS_GRP;
				float w = OS_GCW - 1.f;
				bool act = (i == wave);
				bool hov = (i == hoverIdx);

				nvgBeginPath(vg);
				nvgRoundedRect(vg, x0, y0, w, OS_GCH, 2.5f);
				nvgFillColor(vg, act ? withA(colAccent(), 0.22f) : nvgRGBA(255, 255, 255, hov ? 14 : 6));
				nvgFill(vg);
				if (act || hov)
				{
					nvgStrokeColor(vg, act ? withA(colAccent(), 0.9f) : withA(colBright(), 0.6f));
					nvgStrokeWidth(vg, 1.f);
					nvgStroke(vg);
				}

				// thumbnail
				float tx = x0 + 2.f, tw = w - 4.f, yc = y0 + OS_GCH * 0.5f, amp = 5.8f;
				nvgLineCap(vg, NVG_ROUND);
				nvgLineJoin(vg, NVG_ROUND);
				nvgBeginPath(vg);
				for (int j = 0; j < 24; ++j)
				{
					float x = tx + (float)j / 23.f * tw;
					float y = yc - rack::math::clamp(thumb[i][j], -1.1f, 1.1f) * amp;
					if (j == 0)
						nvgMoveTo(vg, x, y);
					else
						nvgLineTo(vg, x, y);
				}
				nvgStrokeColor(vg, act ? withA(colBright(), 0.98f)
									   : nvgRGBA(0x8a, 0x95, 0xa5, hov ? 230 : 150));
				nvgStrokeWidth(vg, act ? 1.2f : 0.9f);
				nvgStroke(vg);
			}

			// ================= readouts =================
			{
				char buf[64];
				NVGcolor c1 = nvgRGBA(0x3d, 0xe0, 0xff, 230);
				NVGcolor cm = nvgRGBA(0xff, 0xe6, 0x80, 220);
				NVGcolor cd = nvgRGBA(0x8a, 0x95, 0xa5, 230);

				drawText(vg, 6.f, 8.f, OS_NAMES[wave], 7.5f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

				snprintf(buf, sizeof(buf), "V%d", nV);
				drawText(vg, OS_DISP_W / 2.f - 4.f, 8.f, buf, 7.f, cd, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

				noteString(f0, buf, sizeof(buf));
				drawText(vg, OS_DISP_W - 6.f, 8.f, buf, 7.f, cm, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

				if (hoverIdx >= 0)
					snprintf(buf, sizeof(buf), "%02d %s  -  CLICK", hoverIdx + 1, OS_NAMES[hoverIdx]);
				else
					snprintf(buf, sizeof(buf), "WAVE %02d/%d", wave + 1, OS_NW);
				drawText(vg, OS_GX0, 132.5f, buf, 5.5f, c1, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

				std::string ls = (lo > 0.f) ? osHzStr(lo) : std::string("OFF");
				std::string hs = (hi > 0.f) ? osHzStr(hi) : std::string("OFF");
				snprintf(buf, sizeof(buf), "LC %s  HC %s", ls.c_str(), hs.c_str());
				drawText(vg, OS_DISP_W / 2.f + 22.f, 132.5f, buf, 5.2f, nvgRGBA(0xb0, 0x5a, 0xff, 220),
						 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.6f);

				snprintf(buf, sizeof(buf), "PK %.1fV", pk);
				drawText(vg, OS_DISP_W - 8.f, 132.5f, buf, 5.5f, cd, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
			}

			// ================= scanlines, vignette, glass =================
			nvgBeginPath(vg);
			for (int y = 0; y < (int)OS_DISP_H; y += 3)
				nvgRect(vg, 0, y + 1, OS_DISP_W, 1);
			nvgFillColor(vg, nvgRGBA(0, 0, 0, 20));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, OS_DISP_W, OS_DISP_H);
			nvgFillPaint(vg, nvgRadialGradient(vg, OS_DISP_W / 2.f, OS_DISP_H / 2.f,
											   OS_DISP_W * 0.45f, OS_DISP_W * 0.85f,
											   nvgRGBA(0, 0, 0, 0), nvgRGBA(0, 0, 0, 115)));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgRect(vg, 0, 0, OS_DISP_W, OS_DISP_H * 0.30f);
			nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, OS_DISP_H * 0.30f,
											   nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
			nvgFill(vg);

			nvgRestore(vg);

			Widget::drawLayer(args, layer);
		}
	};

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
struct OscillatorWidget : ModuleWidget
{
	OscillatorWidget(Oscillator *module)
	{
		setModule(module);
		box.size = Vec(OS_PANEL_W, OS_PANEL_H);

		addChild(new OSPanelBackground());
		addChild(new OSKnobArcs(module));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

		addChild(new OSDisplayWidget(module));

		for (int k = 0; k < 9; ++k)
		{
			addParam(createParamCentered<RoundSmallBlackKnob>(
				Vec(OS_KNOBS[k].x, OS_KNOBS[k].y), module, OS_KNOBS[k].param));
		}

		addInput(createInputCentered<DarkPJ301MPort>(Vec(osIoX(0), OS_IO_Y), module, Oscillator::VOCT_INPUT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(osIoX(1), OS_IO_Y), module, Oscillator::FM_INPUT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(osIoX(2), OS_IO_Y), module, Oscillator::FB_CV_INPUT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(osIoX(3), OS_IO_Y), module, Oscillator::WAVE_CV_INPUT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(osIoX(4), OS_IO_Y), module, Oscillator::LOCUT_CV_INPUT));
		addInput(createInputCentered<DarkPJ301MPort>(Vec(osIoX(5), OS_IO_Y), module, Oscillator::HICUT_CV_INPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(osIoX(6), OS_IO_Y), module, Oscillator::OUT_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(Vec(osIoX(7), OS_IO_Y), module, Oscillator::SUM_OUTPUT));
	}
};

Model *modelOscillator = createModel<Oscillator, OscillatorWidget>("Oscillator");