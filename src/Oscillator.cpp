#include "plugin.hpp"
#include "agmm_ui.hpp"
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

// Display (wide: the whole width of the panel, with its own rows)
static const float OS_DISP_W = 328.f;
static const float OS_DISP_H = 160.f;
static const float OS_DISP_X = 16.f;
static const float OS_DISP_Y = 34.f;

// Display geometry (display-local)
// header row 0-18, captions y=26, plots 33-93, axis labels y=101,
// wave bank 110-140, footer divider 146, footer text y=153
static const float OS_WX0 = 8.f, OS_WX1 = 160.f;   // waveform box
static const float OS_SX0 = 168.f, OS_SX1 = 320.f; // spectrum box
static const float OS_PY0 = 33.f, OS_PY1 = 93.f;
static const float OS_WYC = 63.f, OS_WAMP = 27.f;
static const float OS_DB_TOP = 6.f, OS_DB_BOT = -60.f;
static const float OS_GX0 = 8.f, OS_GY0 = 110.f, OS_GW = 312.f, OS_GCW = 312.f / 15.f;
static const float OS_GRP = 16.f, OS_GCH = 14.f;

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
		configBypass(VOCT_INPUT, OUT_OUTPUT);

		tab = &osTables();
	}

	void onReset(const ResetEvent &e) override
	{
		Module::onReset(e);
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
// Control layout: two rows of labelled plates.
// Row 1: knob + its CV / input jack. Row 2: knobs and the two outputs.
// ------------------------------------------------------------
struct OSCell
{
	const char *label;
	float cx, top, w, h;
	int param;	// -1 = none
	int input;	// -1 = none
	int output; // -1 = none
	const char *jackLabel;
	bool bipolar;
};

static const float OS_R1_TOP = 204.f, OS_R1_H = 84.f;
static const float OS_R2_TOP = 294.f, OS_R2_H = 62.f;
static const int OS_NCELL = 11;
static const OSCell OS_CELLS[OS_NCELL] = {
	// row 1 (pitch 57)
	{"FREQ", 37.5f, OS_R1_TOP, 54.f, OS_R1_H, Oscillator::FREQ_PARAM, Oscillator::VOCT_INPUT, -1, "V/O", true},
	{"FM", 94.5f, OS_R1_TOP, 54.f, OS_R1_H, Oscillator::FM_PARAM, Oscillator::FM_INPUT, -1, "IN", false},
	{"WAVE", 151.5f, OS_R1_TOP, 54.f, OS_R1_H, Oscillator::WAVE_PARAM, Oscillator::WAVE_CV_INPUT, -1, "CV", false},
	{"FEEDBACK", 208.5f, OS_R1_TOP, 54.f, OS_R1_H, Oscillator::FB_PARAM, Oscillator::FB_CV_INPUT, -1, "CV", false},
	{"LO CUT", 265.5f, OS_R1_TOP, 54.f, OS_R1_H, Oscillator::LOCUT_PARAM, Oscillator::LOCUT_CV_INPUT, -1, "CV", false},
	{"HI CUT", 322.5f, OS_R1_TOP, 54.f, OS_R1_H, Oscillator::HICUT_PARAM, Oscillator::HICUT_CV_INPUT, -1, "CV", false},
	// row 2 (pitch 68)
	{"COARSE", 44.f, OS_R2_TOP, 62.f, OS_R2_H, Oscillator::COARSE_PARAM, -1, -1, nullptr, true},
	{"DETUNE", 112.f, OS_R2_TOP, 62.f, OS_R2_H, Oscillator::DETUNE_PARAM, -1, -1, nullptr, true},
	{"LEVEL", 180.f, OS_R2_TOP, 62.f, OS_R2_H, Oscillator::LEVEL_PARAM, -1, -1, nullptr, false},
	{"POLY OUT", 248.f, OS_R2_TOP, 62.f, OS_R2_H, -1, -1, Oscillator::OUT_OUTPUT, nullptr, false},
	{"SUM OUT", 316.f, OS_R2_TOP, 62.f, OS_R2_H, -1, -1, Oscillator::SUM_OUTPUT, nullptr, false},
};
static inline float osKnobY(const OSCell &c) { return c.top + 38.f; }
static inline float osJackY(const OSCell &c)
{
	return (c.output >= 0) ? c.top + 38.f : c.top + 66.f;
}

namespace ag = agmm;

namespace
{

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
	// Panel background: title, display frame, labelled plates
	// ------------------------------------------------------------
	struct OSPanelBackground : TransparentWidget
	{
		OSPanelBackground() { box.size = Vec(OS_PANEL_W, OS_PANEL_H); }

		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			ag::panelBase(vg, OS_PANEL_W, OS_PANEL_H, "OSCILLATOR");
			ag::bezel(vg, OS_DISP_X, OS_DISP_Y, OS_DISP_W, OS_DISP_H);

			for (int i = 0; i < OS_NCELL; ++i)
			{
				const OSCell &c = OS_CELLS[i];
				ag::cell(vg, c.cx, c.top, c.w, c.h, c.label, c.output >= 0);
				if (c.jackLabel)
					ag::text(vg, c.cx + 14.5f, osJackY(c), c.jackLabel, 7.f, ag::dim(), ag::ALIGN_L, 0.f);
			}
			Widget::draw(args);
		}
	};

	// ------------------------------------------------------------
	// Value arcs around the knobs (flat track + flat arc)
	// ------------------------------------------------------------
	struct OSKnobArcs : TransparentWidget
	{
		Oscillator *module = nullptr;

		OSKnobArcs(Oscillator *m) : module(m) { box.size = Vec(OS_PANEL_W, OS_PANEL_H); }

		void draw(const DrawArgs &args) override
		{
			NVGcontext *vg = args.vg;
			const float R = 18.f;
			const float a0 = -0.75f * (float)M_PI - (float)M_PI / 2.f;
			const float a1 = 0.75f * (float)M_PI - (float)M_PI / 2.f;

			for (int i = 0; i < OS_NCELL; ++i)
			{
				const OSCell &c = OS_CELLS[i];
				if (c.param < 0)
					continue;
				float cx = c.cx, cy = osKnobY(c);

				nvgBeginPath(vg);
				nvgArc(vg, cx, cy, R, a0, a1, NVG_CW);
				nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
				nvgStrokeWidth(vg, 2.f);
				nvgLineCap(vg, NVG_BUTT);
				nvgStroke(vg);

				if (!module)
					continue;
				float v = module->paramQuantities[c.param]->getScaledValue();
				float from = c.bipolar ? 0.5f : 0.f;
				float lo = std::min(from, v), hi = std::max(from, v);
				if (hi - lo < 0.004f)
					continue;
				NVGcolor col = (c.bipolar && v < 0.5f) ? ag::warm() : ag::accent();
				nvgBeginPath(vg);
				nvgArc(vg, cx, cy, R, a0 + (a1 - a0) * lo, a0 + (a1 - a0) * hi, NVG_CW);
				nvgStrokeColor(vg, col);
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
			if (p.x < OS_GX0 || p.x >= OS_GX0 + OS_GW)
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

		static void hline(NVGcontext *vg, float x0, float x1, float y, int alpha)
		{
			nvgBeginPath(vg);
			nvgMoveTo(vg, x0, y + 0.5f);
			nvgLineTo(vg, x1, y + 0.5f);
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, alpha));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
		}
		static void vline(NVGcontext *vg, float x, float y0, float y1, int alpha)
		{
			nvgBeginPath(vg);
			nvgMoveTo(vg, x + 0.5f, y0);
			nvgLineTo(vg, x + 0.5f, y1);
			nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, alpha));
			nvgStrokeWidth(vg, 1.f);
			nvgStroke(vg);
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
			ag::screen(vg, 0.f, 0.f, OS_DISP_W, OS_DISP_H);

			// header and footer dividers
			hline(vg, 8.f, OS_DISP_W - 8.f, 18.f, 40);
			hline(vg, 8.f, OS_DISP_W - 8.f, 146.f, 40);

			// captions (their own row above the plots)
			ag::text(vg, OS_WX0, 26.f, "WAVEFORM", 7.f, ag::dim(), ag::ALIGN_L, 0.8f);
			ag::text(vg, OS_SX0, 26.f, "SPECTRUM", 7.f, ag::dim(), ag::ALIGN_L, 0.8f);

			// Waveform box + grid
			ag::plotFrame(vg, OS_WX0, OS_PY0, OS_WX1 - OS_WX0, OS_PY1 - OS_PY0);
			for (int i = 1; i < 4; ++i)
				vline(vg, OS_WX0 + (OS_WX1 - OS_WX0) * (float)i / 4.f, OS_PY0 + 1.f, OS_PY1 - 1.f, 14);
			for (int i = -1; i <= 1; ++i)
				hline(vg, OS_WX0 + 1.f, OS_WX1 - 1.f, OS_WYC - (float)i * OS_WAMP, i == 0 ? 44 : 14);

			// Spectrum box + grid
			ag::plotFrame(vg, OS_SX0, OS_PY0, OS_SX1 - OS_SX0, OS_PY1 - OS_PY0);
			const float gf[9] = {50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
			for (int i = 0; i < 9; ++i)
			{
				bool major = (gf[i] == 100.f || gf[i] == 1000.f || gf[i] == 10000.f);
				vline(vg, freqX(gf[i]), OS_PY0 + 1.f, OS_PY1 - 1.f, major ? 36 : 14);
			}
			const float gd[3] = {0.f, -20.f, -40.f};
			for (int i = 0; i < 3; ++i)
				hline(vg, OS_SX0 + 1.f, OS_SX1 - 1.f, dbY(gd[i]), i == 0 ? 40 : 14);

			// frequency labels in their own row below the spectrum
			ag::text(vg, freqX(100.f), 101.f, "100", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
			ag::text(vg, freqX(1000.f), 101.f, "1k", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
			ag::text(vg, freqX(10000.f), 101.f, "10k", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

			// Wave bank backdrop
			nvgBeginPath(vg);
			nvgRoundedRect(vg, OS_GX0 - 2.f, OS_GY0 - 2.f, OS_GW + 4.f, 2.f * OS_GRP + 2.f, 3.f);
			nvgFillColor(vg, nvgRGBA(255, 255, 255, 6));
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

				nvgBeginPath(vg);
				nvgMoveTo(vg, wx, OS_WYC);
				for (int i = 0; i < OS_SIM; ++i)
					nvgLineTo(vg, wx + (float)i / (float)(OS_SIM - 1) * ww, yOf(curve[i]));
				nvgLineTo(vg, wx + ww, OS_WYC);
				nvgClosePath(vg);
				nvgFillColor(vg, ag::withA(ag::accent(), 0.12f));
				nvgFill(vg);

				nvgLineJoin(vg, NVG_ROUND);
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
				nvgStrokeColor(vg, ag::bright());
				nvgStrokeWidth(vg, 1.3f);
				nvgStroke(vg);

				// voice phase cursors
				for (int c = nV - 1; c >= 0; --c)
				{
					float p = ph[c] - std::floor(ph[c]);
					int idx = rack::math::clamp((int)(p * (float)OS_SIM), 0, OS_SIM - 1);
					float x = wx + p * ww, y = yOf(curve[idx]);
					if (c == 0)
					{
						vline(vg, x, OS_PY0 + 1.f, OS_PY1 - 1.f, 50);
						nvgBeginPath(vg);
						nvgCircle(vg, x, y, 2.6f);
						nvgFillColor(vg, nvgRGB(255, 255, 255));
						nvgFill(vg);
					}
					else
					{
						nvgBeginPath(vg);
						nvgCircle(vg, x, y, 1.7f);
						nvgFillColor(vg, ag::warm());
						nvgFill(vg);
					}
				}
				nvgRestore(vg);

				// feedback readout, right end of the caption row
				char b[24];
				snprintf(b, sizeof(b), "FB %.0f%%", fb * 100.f);
				ag::text(vg, OS_WX1, 26.f, b, 7.f, ag::textCol(), ag::ALIGN_R, 0.5f);
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
				nvgStrokeColor(vg, ag::withA(ag::warm(), 0.4f));
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
					nvgFillColor(vg, ag::withA(ag::accent(), 0.22f));
					nvgFill(vg);

					float dbF = db + 10.f * std::log10(std::max(filtPow(f), 1e-9f));
					if (dbF > OS_DB_BOT)
					{
						float yF = dbY(dbF);
						nvgBeginPath(vg);
						nvgRect(vg, x - 1.f, yF, 2.f, OS_PY1 - yF);
						nvgFillColor(vg, ag::withA(ag::accent(), 0.85f));
						nvgFill(vg);
					}
				}

				// filter response curve (0 dB reference)
				const int NP = 56;
				nvgLineJoin(vg, NVG_ROUND);
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
				nvgStrokeColor(vg, ag::violet());
				nvgStrokeWidth(vg, 1.2f);
				nvgStroke(vg);

				// cutoff markers
				float cuts[2] = {lo, hi};
				for (int i = 0; i < 2; ++i)
				{
					if (cuts[i] <= 0.f)
						continue;
					nvgBeginPath(vg);
					nvgCircle(vg, freqX(cuts[i]), dbY(-3.f), 2.2f);
					nvgFillColor(vg, ag::violet());
					nvgFill(vg);
				}
				nvgRestore(vg);

				// voices readout, right end of the caption row
				char b[24];
				snprintf(b, sizeof(b), "%d VOICE%s", nV, nV == 1 ? "" : "S");
				ag::text(vg, OS_SX1, 26.f, b, 7.f, ag::textCol(), ag::ALIGN_R, 0.5f);
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
				nvgRoundedRect(vg, x0, y0, w, OS_GCH, 2.f);
				nvgFillColor(vg, act ? ag::withA(ag::accent(), 0.22f) : nvgRGBA(255, 255, 255, hov ? 16 : 6));
				nvgFill(vg);
				if (act || hov)
				{
					nvgStrokeColor(vg, act ? ag::accent() : ag::withA(ag::bright(), 0.6f));
					nvgStrokeWidth(vg, 1.f);
					nvgStroke(vg);
				}

				float tx = x0 + 2.f, tw = w - 4.f, yc = y0 + OS_GCH * 0.5f, amp = 4.8f;
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
				nvgStrokeColor(vg, act ? ag::bright() : nvgRGBA(0x8a, 0x95, 0xa5, hov ? 230 : 150));
				nvgStrokeWidth(vg, act ? 1.2f : 0.9f);
				nvgStroke(vg);
			}

			// ================= header row =================
			{
				char buf[64];
				ag::text(vg, 8.f, 9.f, OS_NAMES[wave], 8.f, ag::accent(), ag::ALIGN_L, 1.f);
				snprintf(buf, sizeof(buf), "%02d/%d", wave + 1, OS_NW);
				ag::text(vg, OS_DISP_W / 2.f, 9.f, buf, 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
				noteString(f0, buf, sizeof(buf));
				ag::text(vg, OS_DISP_W - 8.f, 9.f, buf, 7.f, nvgRGB(0xff, 0xe6, 0x80), ag::ALIGN_R, 0.5f);
			}

			// ================= footer row =================
			{
				char buf[64];
				if (hoverIdx >= 0)
					snprintf(buf, sizeof(buf), "%02d %s - CLICK", hoverIdx + 1, OS_NAMES[hoverIdx]);
				else
					snprintf(buf, sizeof(buf), "CLICK A WAVE");
				ag::text(vg, 8.f, 153.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);

				std::string ls = (lo > 0.f) ? osHzStr(lo) : std::string("OFF");
				std::string hs = (hi > 0.f) ? osHzStr(hi) : std::string("OFF");
				snprintf(buf, sizeof(buf), "LC %s  HC %s", ls.c_str(), hs.c_str());
				ag::text(vg, OS_DISP_W / 2.f + 24.f, 153.f, buf, 7.f, ag::violet(), ag::ALIGN_C, 0.3f);

				snprintf(buf, sizeof(buf), "PK %.1fV", pk);
				ag::text(vg, OS_DISP_W - 8.f, 153.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.5f);
			}

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

		for (int i = 0; i < OS_NCELL; ++i)
		{
			const OSCell &c = OS_CELLS[i];
			if (c.param >= 0)
				addParam(createParamCentered<RoundSmallBlackKnob>(Vec(c.cx, osKnobY(c)), module, c.param));
			if (c.input >= 0)
				addInput(createInputCentered<DarkPJ301MPort>(Vec(c.cx, osJackY(c)), module, c.input));
			if (c.output >= 0)
				addOutput(createOutputCentered<DarkPJ301MPort>(Vec(c.cx, osJackY(c)), module, c.output));
		}
	}
};

Model *modelOscillator = createModel<Oscillator, OscillatorWidget>("Oscillator");