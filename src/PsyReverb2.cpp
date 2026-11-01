// PsyReverb2.cpp
// Single-file VCV Rack module styled to match AGMM's Reverb panel.
// Based on Airwindows Galactic / Sapphire Galaxy, extended with damping,
// predelay, mod, width, freeze, shelving EQ, low cut, ducking, CV + attenuverter
// per knob, and a Wave / Bars / Spectrum display.

#include <rack.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace rack;

// ---------------------------------------------------------------------
//  Small utilities
// ---------------------------------------------------------------------
template <typename T> inline T clampT(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double sq(double x) { return x * x; }
inline double cb(double x) { return x * x * x; }

// ---------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------
static const int   P2_SCOPE_N   = 160;     // waveform columns
static const int   P2_FFT_N     = 1024;    // FFT size
static const int   P2_FFT_RING  = 4096;    // audio -> UI ring buffer (power of 2)
static const int   P2_BARS      = 24;      // analyzer bars
static const int   P2_SPEC_COLS = 150;     // spectrum curve resolution
static const int   P2_NUM_CTRL  = 14;      // knobs with CV + attenuverter
static const float P2_FMIN      = 30.f;
static const float P2_FMAX      = 18000.f;
static const float P2_DB_FLOOR  = -72.f;   // analyzer floor (dB re 5 V)
static const float P2_TILT      = 3.f;     // dB/oct display tilt (flattens pink noise)
static const float P2_TWO_PI    = 6.28318530717958647692f;

// =====================================================================
//  Helpers + Reverb Engine
// =====================================================================
namespace psyrv
{
    constexpr int NDELAYS        = 13;
    constexpr int MIN_CYCLE      = 1;
    constexpr int MAX_CYCLE      = 4;
    constexpr int MOD_TANK_DELAY = 256;
    constexpr int MOD_TANK_ALLOC = 1024;
    constexpr int MAX_PREDELAY   = 24000;

    // ---- small DSP helpers ----------------------------------------
    // Transparent up to 6 V, smooth knee towards a ceiling of 10 V
    inline float softLimit(float x)
    {
        const float k = 6.f, r = 4.f;
        float a = std::fabs(x);
        if (a <= k) return x;
        float y = k + r * std::tanh((a - k) / r);
        return x < 0.f ? -y : y;
    }

    struct OnePole
    {
        float z = 0.f;
        float coeff = 1.f;
        void setCutoff(float freq, float sr)
        {
            if (freq < 1.f) freq = 1.f;
            if (freq > sr * 0.45f) freq = sr * 0.45f;
            coeff = 1.f - std::exp(-P2_TWO_PI * freq / sr);
        }
        float process(float in) { z += coeff * (in - z); return z; }
    };

    // RBJ shelving biquad, slope S = 1, transposed direct form II
    struct Biquad
    {
        float b0 = 1.f, b1 = 0.f, b2 = 0.f, a1 = 0.f, a2 = 0.f;
        float z1 = 0.f, z2 = 0.f;

        float process(float x)
        {
            float y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }
        void copyCoeffs(const Biquad& o) { b0 = o.b0; b1 = o.b1; b2 = o.b2; a1 = o.a1; a2 = o.a2; }

        void setLowShelf(float freq, float dB, float sr)
        {
            freq = clampT(freq, 10.f, sr * 0.45f);
            float A = std::pow(10.f, dB / 40.f);
            float w0 = P2_TWO_PI * freq / sr;
            float cw = std::cos(w0), sw = std::sin(w0);
            float alpha = sw / std::sqrt(2.f);
            float tsa = 2.f * std::sqrt(A) * alpha;
            float a0 = (A + 1.f) + (A - 1.f) * cw + tsa;
            b0 = (A * ((A + 1.f) - (A - 1.f) * cw + tsa)) / a0;
            b1 = (2.f * A * ((A - 1.f) - (A + 1.f) * cw)) / a0;
            b2 = (A * ((A + 1.f) - (A - 1.f) * cw - tsa)) / a0;
            a1 = (-2.f * ((A - 1.f) + (A + 1.f) * cw)) / a0;
            a2 = ((A + 1.f) + (A - 1.f) * cw - tsa) / a0;
        }

        void setHighShelf(float freq, float dB, float sr)
        {
            freq = clampT(freq, 10.f, sr * 0.45f);
            float A = std::pow(10.f, dB / 40.f);
            float w0 = P2_TWO_PI * freq / sr;
            float cw = std::cos(w0), sw = std::sin(w0);
            float alpha = sw / std::sqrt(2.f);
            float tsa = 2.f * std::sqrt(A) * alpha;
            float a0 = (A + 1.f) - (A - 1.f) * cw + tsa;
            b0 = (A * ((A + 1.f) + (A - 1.f) * cw + tsa)) / a0;
            b1 = (-2.f * A * ((A - 1.f) + (A + 1.f) * cw)) / a0;
            b2 = (A * ((A + 1.f) + (A - 1.f) * cw - tsa)) / a0;
            a1 = (2.f * ((A - 1.f) - (A + 1.f) * cw)) / a0;
            a2 = ((A + 1.f) - (A - 1.f) * cw - tsa) / a0;
        }
    };

    // Small radix-2 FFT (display only, UI thread)
    struct FFT
    {
        static constexpr int N = P2_FFT_N;
        static constexpr int LOG2N = 10;
        float cosT[N / 2], sinT[N / 2];
        float window[N];
        int rev[N];

        FFT()
        {
            static_assert((1 << LOG2N) == N, "FFT size mismatch");
            for (int k = 0; k < N / 2; ++k)
            {
                cosT[k] = std::cos(P2_TWO_PI * k / N);
                sinT[k] = -std::sin(P2_TWO_PI * k / N);
            }
            for (int i = 0; i < N; ++i)
            {
                window[i] = 0.5f - 0.5f * std::cos(P2_TWO_PI * i / (N - 1));
                int r = 0;
                for (int b = 0; b < LOG2N; ++b)
                    if ((i >> b) & 1)
                        r |= 1 << (LOG2N - 1 - b);
                rev[i] = r;
            }
        }

        void forward(const float* in, float* re, float* im) const
        {
            for (int i = 0; i < N; ++i)
            {
                re[rev[i]] = in[i] * window[i];
                im[rev[i]] = 0.f;
            }
            for (int len = 2; len <= N; len <<= 1)
            {
                int half = len / 2, step = N / len;
                for (int i = 0; i < N; i += len)
                {
                    for (int j = 0; j < half; ++j)
                    {
                        float wr = cosT[j * step], wi = sinT[j * step];
                        int a = i + j, b = i + j + half;
                        float vr = re[b] * wr - im[b] * wi;
                        float vi = re[b] * wi + im[b] * wr;
                        re[b] = re[a] - vr;
                        im[b] = im[a] - vi;
                        re[a] += vr;
                        im[a] += vi;
                    }
                }
            }
        }
    };

    // ---- reverb engine (unchanged) ---------------------------------
    struct StereoFrame
    {
        double ch[2];
        StereoFrame()                   : ch{0.0, 0.0} {}
        StereoFrame(double l, double r) : ch{l, r} {}
        void clear()                    { ch[0] = ch[1] = 0.0; }
        StereoFrame operator+(const StereoFrame& o) const { return { ch[0]+o.ch[0], ch[1]+o.ch[1] }; }
        StereoFrame operator-(const StereoFrame& o) const { return { ch[0]-o.ch[0], ch[1]-o.ch[1] }; }
        StereoFrame operator*(double s)             const { return { ch[0]*s, ch[1]*s }; }
        StereoFrame operator/(double d)             const { return { ch[0]/d, ch[1]/d }; }
        StereoFrame flip()                          const { return { ch[1], ch[0] }; }
    };

    struct StereoQuad
    {
        StereoFrame q[4];
        void clear()      { for (int i = 0; i < 4; i++) q[i].clear(); }
        StereoFrame sum() const { return q[0] + q[1] + q[2] + q[3]; }
    };

    struct DelayLine
    {
        std::vector<StereoFrame> buf;
        int count = 1;
        int delay = 1;
        void clear() { count = 1; for (auto& f : buf) f.clear(); }
        void advance() { ++count; if (count < 0 || count > delay) count = 0; }
        int  reverse(int length) const
        {
            const int off = (length > delay) ? (delay + 1) : 0;
            return length - off;
        }
        StereoFrame& head() { return buf.at(count); }
        StereoFrame  tail() { return buf.at(reverse(count)); }
    };

    struct Engine
    {
        float decayKnob    = 0.5f;
        float sizeKnob     = 0.5f;
        float brightKnob   = 0.5f;
        float dampKnob     = 0.3f;
        float predelayKnob = 0.0f;
        float modKnob      = 0.3f;
        float widthKnob    = 0.5f;
        float mixKnob      = 1.0f;
        bool  freeze       = false;

        uint32_t fpd[2] = {0, 0};
        double   modPhase = 0.0;
        double   oldFpd   = 0.0;
        int      cycle    = 0;

        DelayLine   delay[NDELAYS];
        StereoQuad  feedback;
        StereoQuad  dampState;
        StereoFrame iirA;
        StereoFrame iirB;
        StereoFrame lastRef[MAX_CYCLE + 1];

        std::vector<StereoFrame> predelayBuf;
        int                      predelayWrite = 0;

        const int tankSize[12] = {
            4801, 2909, 1153,  461,
            7607, 4217, 2269, 1597,
            3407, 1823,  859,  331
        };

        Engine()
        {
            delay[ 0].buf.resize( 9700);
            delay[ 1].buf.resize( 6000);
            delay[ 2].buf.resize( 2320);
            delay[ 3].buf.resize(  940);
            delay[ 4].buf.resize(15220);
            delay[ 5].buf.resize( 8460);
            delay[ 6].buf.resize( 4540);
            delay[ 7].buf.resize( 3200);
            delay[ 8].buf.resize( 6480);
            delay[ 9].buf.resize( 3660);
            delay[10].buf.resize( 1720);
            delay[11].buf.resize(  680);
            delay[12].buf.resize(MOD_TANK_ALLOC);
            predelayBuf.resize(MAX_PREDELAY);
            initialize();
        }

        void initialize()
        {
            fpd[0]   = 2756923396u;
            fpd[1]   = 2341963165u;
            modPhase = 0.0;
            oldFpd   = 429496.7295;
            cycle    = 0;
            for (int i = 0; i < NDELAYS; i++) delay[i].clear();
            for (int i = 0; i <= MAX_CYCLE; i++) lastRef[i].clear();
            feedback.clear();
            dampState.clear();
            iirA.clear();
            iirB.clear();
            for (auto& f : predelayBuf) f.clear();
            predelayWrite = 0;
        }

        void setDecay(double v)    { decayKnob    = clampT<float>(v, 0.f, 1.f); }
        void setSize(double v)     { sizeKnob     = clampT<float>(v, 0.f, 1.f); }
        void setBright(double v)   { brightKnob   = clampT<float>(v, 0.f, 1.f); }
        void setDamp(double v)     { dampKnob     = clampT<float>(v, 0.f, 1.f); }
        void setPredelay(double v) { predelayKnob = clampT<float>(v, 0.f, 1.f); }
        void setMod(double v)      { modKnob      = clampT<float>(v, 0.f, 1.f); }
        void setWidth(double v)    { widthKnob    = clampT<float>(v, 0.f, 1.f); }
        void setMix(double v)      { mixKnob      = clampT<float>(v, 0.f, 1.f); }
        void setFreeze(bool v)     { freeze       = v; }

    private:
        static double dither(double x, uint32_t& f)
        {
            int expon;
            frexpf(static_cast<float>(x), &expon);
            f ^= f << 13;
            f ^= f >> 17;
            f ^= f << 5;
            return x + ((double(f) - uint32_t(0x7fffffff)) * 5.5e-36l * std::exp2(expon + 62.0));
        }

        void writeTank(int idx, const StereoFrame& f)
        {
            delay[idx].head() = f;
            delay[idx].advance();
        }

        StereoQuad readQuad(int start)
        {
            StereoQuad q;
            q.q[0] = delay[start + 0].tail();
            q.q[1] = delay[start + 1].tail();
            q.q[2] = delay[start + 2].tail();
            q.q[3] = delay[start + 3].tail();
            return q;
        }

        static StereoQuad stir(const StereoQuad& f)
        {
            StereoQuad g;
            g.q[0] = f.q[0] - (f.q[1] + f.q[2] + f.q[3]);
            g.q[1] = f.q[1] - (f.q[0] + f.q[2] + f.q[3]);
            g.q[2] = f.q[2] - (f.q[0] + f.q[1] + f.q[3]);
            g.q[3] = f.q[3] - (f.q[0] + f.q[1] + f.q[2]);
            return g;
        }

        void writeQuad(int start, const StereoQuad& f)
        {
            const StereoQuad g = stir(f);
            for (int i = 0; i < 4; i++) writeTank(start + i, g.q[i]);
        }

        void reflect(int start, const StereoFrame& in, double regen)
        {
            for (int i = 0; i < 4; i++)
                writeTank(start + i, in + feedback.q[i].flip() * regen);
        }

        double interp(int ch, double phaseOffset)
        {
            const double depth = 127.0 * modKnob;
            const double ofs   = (std::sin(modPhase + phaseOffset) + 1.0) * depth;
            const double frac  = ofs - std::floor(ofs);
            const int    base  = delay[12].count + static_cast<int>(ofs);
            const int    i1    = delay[12].reverse(base);
            const int    i2    = delay[12].reverse(base + 1);
            const auto&  b     = delay[12].buf;
            return b.at(i1).ch[ch & 1] * (1.0 - frac)
                 + b.at(i2).ch[ch & 1] * frac;
        }

        void mixDown(const StereoFrame& s)
        {
            switch (cycle)
            {
            case 4:
                lastRef[0] = lastRef[4];
                lastRef[2] = (lastRef[0] + s) / 2.0;
                lastRef[1] = (lastRef[0] + lastRef[2]) / 2.0;
                lastRef[3] = (lastRef[2] + s) / 2.0;
                lastRef[4] = s;
                break;
            case 3:
                lastRef[0] = lastRef[3];
                lastRef[2] = (lastRef[0] + lastRef[0] + s) / 3.0;
                lastRef[1] = (lastRef[0] + s + s) / 3.0;
                lastRef[3] = s;
                break;
            case 2:
                lastRef[0] = lastRef[2];
                lastRef[1] = (lastRef[0] + s) / 2.0;
                lastRef[2] = s;
                break;
            case 1:
            default:
                lastRef[0] = s;
                break;
            }
        }

        static double softClip(double x)
        {
            if (x >  1.0) return  1.0 + std::tanh(x - 1.0);
            if (x < -1.0) return -1.0 + std::tanh(x + 1.0);
            return x;
        }

    public:
        void process(float sampleRate, float inL, float inR, float& outL, float& outR)
        {
            double ol = 0.0, orr = 0.0;
            process(static_cast<double>(sampleRate),
                    static_cast<double>(inL),
                    static_cast<double>(inR),
                    ol, orr);
            outL = static_cast<float>(ol);
            outR = static_cast<float>(orr);
        }

        void process(double sampleRateHz, double inL, double inR, double& outL, double& outR)
        {
            const double overallscale = sampleRateHz / 44100.0;
            const int    cycleEnd     = clampT<int>(static_cast<int>(std::floor(overallscale)),
                                                    MIN_CYCLE, MAX_CYCLE);
            if (cycle > cycleEnd - 1) cycle = cycleEnd - 1;

            double regen, atten;
            if (freeze) { regen = 0.5;  atten = 0.0; }
            else        { regen = 0.0625 + decayKnob * 0.0625;
                          atten = 1.0    - decayKnob * 0.5; }

            const double lowpass = sq(1.00001 - (1.0 - brightKnob)) / std::sqrt(overallscale);
            const double size    = sizeKnob  * 1.77 + 0.1;
            const double wet     = 1.0 - cb(1.0 - mixKnob);
            const double width   = widthKnob * 2.0;

            for (int i = 0; i < 12; i++)
                delay[i].delay = static_cast<int>(tankSize[i] * size);
            delay[12].delay = MOD_TANK_DELAY;

            int predelayLen = static_cast<int>(predelayKnob * 0.1 * sampleRateHz);
            predelayLen = clampT<int>(predelayLen, 0, MAX_PREDELAY - 1);

            if (std::abs(inL) < 1.18e-23) inL = fpd[0] * 1.18e-17;
            if (std::abs(inR) < 1.18e-23) inR = fpd[1] * 1.18e-17;
            const StereoFrame dry(inL, inR);

            predelayBuf[predelayWrite] = dry;
            int rdIdx = predelayWrite - predelayLen;
            while (rdIdx < 0) rdIdx += MAX_PREDELAY;
            const StereoFrame predelayed = predelayBuf[rdIdx];
            predelayWrite = (predelayWrite + 1) % MAX_PREDELAY;

            const double modSpeed = cb(modKnob) * 0.001;
            modPhase += oldFpd * modSpeed;
            if (modPhase > 2.0 * M_PI) {
                modPhase = 0.0;
                oldFpd   = 0.4294967295 + (fpd[0] * 0.0000000000618);
            }

            writeTank(12, predelayed * atten);

            const StereoFrame phasor(interp(0, 0.0), interp(1, M_PI_2));
            StereoFrame sample = iirA = iirA * (1.0 - lowpass) + phasor * lowpass;

            if (++cycle == cycleEnd)
            {
                reflect(8, sample, regen);
                writeQuad(0, readQuad(8));
                writeQuad(4, readQuad(0));

                const StereoQuad f = readQuad(4);
                feedback = stir(f);

                if (dampKnob > 0.001f) {
                    const double alpha = 1.0 - dampKnob * 0.9;
                    for (int i = 0; i < 4; i++) {
                        for (int c = 0; c < 2; c++) {
                            dampState.q[i].ch[c] =
                                alpha * dampState.q[i].ch[c] +
                                (1.0 - alpha) * feedback.q[i].ch[c];
                            feedback.q[i].ch[c] = dampState.q[i].ch[c];
                        }
                    }
                }

                if (freeze) {
                    for (int i = 0; i < 4; i++)
                        for (int c = 0; c < 2; c++)
                            feedback.q[i].ch[c] = softClip(feedback.q[i].ch[c]);
                }

                mixDown(f.sum() / 8.0);
                cycle = 0;
            }

            iirB = iirB * (1.0 - lowpass) + lastRef[cycle] * lowpass;

            const double mid  = (iirB.ch[0] + iirB.ch[1]) * 0.5;
            const double side = (iirB.ch[0] - iirB.ch[1]) * 0.5;
            const StereoFrame widened(mid + side * width, mid - side * width);

            const StereoFrame out = widened * wet + dry * (1.0 - wet);
            outL = dither(out.ch[0], fpd[0]);
            outR = dither(out.ch[1], fpd[1]);
        }
    };
} // namespace psyrv

// =====================================================================
//  Control table (grid order: 6 columns x 3 rows; row 3 holds 2 knobs,
//  FREEZE, CLEAR, BYPASS and the CV range cell)
// =====================================================================
struct P2CtlSpec
{
    const char* name;    // panel label
    const char* label;   // tooltip label
    float lo, hi, def;
    const char* unit;
    float dispBase, dispMul;
};

static const P2CtlSpec P2_CTL[P2_NUM_CTRL] = {
    {"IN GAIN",  "Input gain",       0.f,   2.f,  1.f,  "%",   0.f, 100.f},
    {"PREDELAY", "Predelay",         0.f,   1.f,  0.f,  " ms", 0.f, 100.f},
    {"SIZE",     "Size",             0.f,   1.f,  0.5f, "%",   0.f, 100.f},
    {"DECAY",    "Decay",            0.f,   1.f,  0.5f, "%",   0.f, 100.f},
    {"BRIGHT",   "Brightness",       0.f,   1.f,  0.5f, "%",   0.f, 100.f},
    {"DAMP",     "Damping",          0.f,   1.f,  0.3f, "%",   0.f, 100.f},
    {"MOD",      "Modulation",       0.f,   1.f,  0.3f, "%",   0.f, 100.f},
    {"WIDTH",    "Stereo width",     0.f,   1.f,  0.5f, "%",   0.f, 200.f},
    {"LO GAIN",  "Low shelf gain",  -12.f,  12.f, 0.f,  " dB", 0.f, 1.f},
    {"HI GAIN",  "High shelf gain", -12.f,  12.f, 0.f,  " dB", 0.f, 1.f},
    {"LOW CUT",  "Low cut",          0.f,   1.f,  0.f,  "%",   0.f, 100.f},
    {"DUCK",     "Ducking",          0.f,   1.f,  0.f,  "%",   0.f, 100.f},
    {"MIX",      "Mix",              0.f,   1.f,  1.f,  "%",   0.f, 100.f},
    {"OUT LEVEL","Output level",     0.f,   2.f,  1.f,  "%",   0.f, 100.f},
};

// =====================================================================
//  Module
// =====================================================================
struct PsyReverb2Module : Module
{
    enum ParamId
    {
        // knobs (order matches P2_CTL)
        IN_GAIN_PARAM, PREDELAY_PARAM, SIZE_PARAM, DECAY_PARAM,
        BRIGHT_PARAM, DAMP_PARAM, MOD_PARAM, WIDTH_PARAM,
        LO_GAIN_PARAM, HI_GAIN_PARAM, LOW_CUT_PARAM, DUCK_PARAM,
        MIX_PARAM, OUT_LEVEL_PARAM,
        // attenuverters (same order)
        ATTN_FIRST,
        ATTN_LAST = ATTN_FIRST + P2_NUM_CTRL - 1,
        // buttons / switches
        FREEZE_PARAM,
        CLEAR_PARAM,
        BYPASS_PARAM,
        PARAMS_LEN
    };

    enum InputId
    {
        AUDIO_L_INPUT, AUDIO_R_INPUT,
        FREEZE_GATE_INPUT, CLEAR_TRIG_INPUT,
        CV_FIRST,
        CV_LAST = CV_FIRST + P2_NUM_CTRL - 1,
        INPUTS_LEN
    };

    enum OutputId { AUDIO_L_OUTPUT, AUDIO_R_OUTPUT, OUTPUTS_LEN };
    enum LightId  { FREEZE_LIGHT, CLEAR_LIGHT, LIGHTS_LEN };

    static_assert(OUT_LEVEL_PARAM == P2_NUM_CTRL - 1, "knob count mismatch");

    enum CVRange { BIPOLAR_M5_5, UNIPOLAR_0_10 };
    enum VisMode { VIS_WAVE, VIS_BARS, VIS_SPECTRUM, VIS_COUNT };

    // ---- settings (saved with the patch) ----
    CVRange cvRangeMode = BIPOLAR_M5_5;
    int   visMode     = VIS_SPECTRUM;
    bool  eqPre       = false;   // false: EQ on reverb output, true: EQ on reverb input
    bool  softClip    = true;
    float loFreq      = 200.f;
    float hiFreq      = 4000.f;
    bool  freezeLatch = false;

    // ---- DSP ----
    psyrv::Engine engine;
    psyrv::Biquad loL, loR, hiL, hiR;
    psyrv::OnePole hpL1, hpL2, hpR1, hpR2;
    dsp::SchmittTrigger clearTrig;

    float dspSr = 0.f;
    bool  needSnap = true;
    int   ctrlCounter = 0;
    bool  prevFreezePressed = false;
    bool  prevClearPressed  = false;
    float clearFlash = 0.f;

    float dcR = 0.999f, dcXL = 0.f, dcYL = 0.f, dcXR = 0.f, dcYR = 0.f;
    float kSmooth = 0.01f, kAtk = 0.01f, kRel = 0.0001f;
    float env = 0.f;

    // smoothed parameters
    float sIn = 1.f, sMix = 1.f, sOut = 1.f, sLo = 0.f, sHi = 0.f;
    float sDuck = 0.f, sBypass = 0.f;

    // ---- display data (audio thread writes, UI thread reads; torn reads are cosmetic) ----
    float scopeMin[P2_SCOPE_N] = {};
    float scopeMax[P2_SCOPE_N] = {};
    int   scopeHead = 0;
    int   scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;
    float fftRing[P2_FFT_RING] = {};
    int   fftHead = 0;
    float dispPre = 0.f, dispSize = 0.5f, dispMix = 1.f, dispLo = 0.f, dispHi = 0.f;
    bool  dispBypass = false, dispFreeze = false;
    float clipTimer = 0.f;

    PsyReverb2Module()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        for (int i = 0; i < P2_NUM_CTRL; ++i)
        {
            const P2CtlSpec& s = P2_CTL[i];
            configParam(i, s.lo, s.hi, s.def, s.label, s.unit, s.dispBase, s.dispMul);
            configParam(ATTN_FIRST + i, -1.f, 1.f, 0.f,
                        std::string(s.label) + " CV amount", "%", 0.f, 100.f);
            configInput(CV_FIRST + i, std::string(s.label) + " CV");
        }

        configButton(FREEZE_PARAM, "Freeze (toggle)");
        configButton(CLEAR_PARAM,  "Clear tank");
        configSwitch(BYPASS_PARAM, 0.f, 1.f, 0.f, "Bypass", {"Off", "On"});
        configLight (FREEZE_LIGHT, "Freeze active");
        configLight (CLEAR_LIGHT,  "Clear pressed");

        configInput (AUDIO_L_INPUT,     "Audio left");
        configInput (AUDIO_R_INPUT,     "Audio right");
        configInput (FREEZE_GATE_INPUT, "Freeze gate");
        configInput (CLEAR_TRIG_INPUT,  "Clear trigger");
        configOutput(AUDIO_L_OUTPUT,    "Audio left");
        configOutput(AUDIO_R_OUTPUT,    "Audio right");

        configBypass(AUDIO_L_INPUT, AUDIO_L_OUTPUT);
        configBypass(AUDIO_R_INPUT, AUDIO_R_OUTPUT);
    }

    void resetState()
    {
        engine.initialize();
        loL = psyrv::Biquad(); loR = psyrv::Biquad();
        hiL = psyrv::Biquad(); hiR = psyrv::Biquad();
        hpL1.z = hpL2.z = hpR1.z = hpR2.z = 0.f;
        dcXL = dcYL = dcXR = dcYR = 0.f;
        env = 0.f;
        clearFlash = 0.f;
        prevFreezePressed = prevClearPressed = false;
        for (int i = 0; i < P2_SCOPE_N; i++) { scopeMin[i] = scopeMax[i] = 0.f; }
        scopeHead = scopeCount = 0;
        scopeBMin = scopeBMax = 0.f;
        peakL = peakR = 0.f;
        clipTimer = 0.f;
        needSnap = true;
        ctrlCounter = 0;
    }

    void onReset(const ResetEvent& e) override
    {
        Module::onReset(e);
        freezeLatch = false;
        resetState();
    }

    void setupDSP(float sr)
    {
        dspSr = sr;
        dcR = 1.f - P2_TWO_PI * 10.f / sr;
        kSmooth = 1.f - std::exp(-1.f / (0.010f * sr));
        kAtk    = 1.f - std::exp(-1.f / (0.005f * sr));
        kRel    = 1.f - std::exp(-1.f / (0.300f * sr));
        loL = psyrv::Biquad(); loR = psyrv::Biquad();
        hiL = psyrv::Biquad(); hiR = psyrv::Biquad();
        hpL1.z = hpL2.z = hpR1.z = hpR2.z = 0.f;
        dcXL = dcYL = dcXR = dcYR = 0.f;
        ctrlCounter = 0;
        needSnap = true;
    }

    float getCtl(int i)
    {
        const P2CtlSpec& s = P2_CTL[i];
        float knob = params[i].getValue();
        float cv   = inputs[CV_FIRST + i].getVoltage();
        float attn = params[ATTN_FIRST + i].getValue();

        float cvScaled = (cvRangeMode == UNIPOLAR_0_10) ? cv / 10.f : cv / 5.f;
        cvScaled = clampT(cvScaled, -1.f, 1.f);

        // CV sweeps the whole range of the parameter at full attenuverter
        return clampT(knob + cvScaled * attn * (s.hi - s.lo), s.lo, s.hi);
    }

    json_t* dataToJson() override
    {
        json_t* root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        json_object_set_new(root, "visMode",     json_integer(visMode));
        json_object_set_new(root, "eqPre",       json_boolean(eqPre));
        json_object_set_new(root, "softClip",    json_boolean(softClip));
        json_object_set_new(root, "loFreq",      json_real(loFreq));
        json_object_set_new(root, "hiFreq",      json_real(hiFreq));
        json_object_set_new(root, "freezeLatch", json_boolean(freezeLatch));
        return root;
    }

    void dataFromJson(json_t* root) override
    {
        if (json_t* j = json_object_get(root, "cvRangeMode"))
            cvRangeMode = (CVRange) json_integer_value(j);
        if (json_t* j = json_object_get(root, "visMode"))
            visMode = (int) clampT((float) json_integer_value(j), 0.f, (float) (VIS_COUNT - 1));
        if (json_t* j = json_object_get(root, "eqPre"))
            eqPre = json_boolean_value(j);
        if (json_t* j = json_object_get(root, "softClip"))
            softClip = json_boolean_value(j);
        if (json_t* j = json_object_get(root, "loFreq"))
            loFreq = clampT((float) json_number_value(j), 40.f, 1000.f);
        if (json_t* j = json_object_get(root, "hiFreq"))
            hiFreq = clampT((float) json_number_value(j), 1000.f, 12000.f);
        if (json_t* j = json_object_get(root, "freezeLatch"))
            freezeLatch = json_boolean_value(j);
    }

    void process(const ProcessArgs& args) override
    {
        const float sr = args.sampleRate;
        if (sr != dspSr)
            setupDSP(sr);

        // ---- parameters ----
        float c[P2_NUM_CTRL];
        for (int i = 0; i < P2_NUM_CTRL; ++i)
            c[i] = getCtl(i);

        // ---- freeze (button latch + gate input) ----
        const bool frzPressed = params[FREEZE_PARAM].getValue() > 0.5f;
        if (frzPressed && !prevFreezePressed) freezeLatch = !freezeLatch;
        prevFreezePressed = frzPressed;
        const bool freezeOn = freezeLatch || inputs[FREEZE_GATE_INPUT].getVoltage() >= 2.f;
        lights[FREEZE_LIGHT].setBrightness(freezeOn ? 1.f : 0.f);

        // ---- clear (button + trigger input) ----
        const bool clrPressed = params[CLEAR_PARAM].getValue() > 0.5f;
        bool doClear = (clrPressed && !prevClearPressed);
        prevClearPressed = clrPressed;
        if (clearTrig.process(inputs[CLEAR_TRIG_INPUT].getVoltage(), 0.1f, 2.f))
            doClear = true;
        if (doClear) {
            engine.initialize();
            hpL1.z = hpL2.z = hpR1.z = hpR2.z = 0.f;
            clearFlash = 0.2f;
        }
        if (clearFlash > 0.f) clearFlash -= args.sampleTime;
        lights[CLEAR_LIGHT].setBrightness(clearFlash > 0.f ? 1.f : 0.f);

        const float tBypass = params[BYPASS_PARAM].getValue() > 0.5f ? 1.f : 0.f;

        if (needSnap)
        {
            sIn = c[IN_GAIN_PARAM];   sMix = c[MIX_PARAM];
            sOut = c[OUT_LEVEL_PARAM]; sLo = c[LO_GAIN_PARAM];
            sHi = c[HI_GAIN_PARAM];   sDuck = c[DUCK_PARAM];
            sBypass = tBypass;
            needSnap = false;
        }
        sIn    += (c[IN_GAIN_PARAM]   - sIn)   * kSmooth;
        sMix   += (c[MIX_PARAM]       - sMix)  * kSmooth;
        sOut   += (c[OUT_LEVEL_PARAM] - sOut)  * kSmooth;
        sLo    += (c[LO_GAIN_PARAM]   - sLo)   * kSmooth;
        sHi    += (c[HI_GAIN_PARAM]   - sHi)   * kSmooth;
        sDuck  += (c[DUCK_PARAM]      - sDuck) * kSmooth;
        sBypass += (tBypass - sBypass) * kSmooth;

        // ---- control-rate coefficient updates (every 16 samples) ----
        if ((ctrlCounter++ & 15) == 0)
        {
            loL.setLowShelf(loFreq, sLo, sr);   loR.copyCoeffs(loL);
            hiL.setHighShelf(hiFreq, sHi, sr);  hiR.copyCoeffs(hiL);

            float hpF = 20.f * std::pow(50.f, c[LOW_CUT_PARAM]);   // 20 Hz .. 1 kHz
            hpL1.setCutoff(hpF, sr); hpL2.coeff = hpL1.coeff;
            hpR1.setCutoff(hpF, sr); hpR2.coeff = hpR1.coeff;
        }

        // ---- engine parameters ----
        engine.setDecay   (c[DECAY_PARAM]);
        engine.setSize    (c[SIZE_PARAM]);
        engine.setBright  (c[BRIGHT_PARAM]);
        engine.setDamp    (c[DAMP_PARAM]);
        engine.setPredelay(c[PREDELAY_PARAM]);
        engine.setMod     (c[MOD_PARAM]);
        engine.setWidth   (c[WIDTH_PARAM]);
        engine.setMix     (1.f);               // engine runs fully wet; dry/wet is done here
        engine.setFreeze(freezeOn);

        // ---- inputs (mono normalled to both sides) ----
        const bool lc = inputs[AUDIO_L_INPUT].isConnected();
        const bool rc = inputs[AUDIO_R_INPUT].isConnected();
        float rawL = lc ? inputs[AUDIO_L_INPUT].getVoltageSum() : 0.f;
        float rawR = rc ? inputs[AUDIO_R_INPUT].getVoltageSum() : 0.f;
        if (lc && !rc) rawR = rawL;
        if (rc && !lc) rawL = rawR;

        const float dryL = rawL * sIn;
        const float dryR = rawR * sIn;

        // DC blocker on the reverb feed
        float fxL = dryL - dcXL + dcR * dcYL;  dcXL = dryL;  dcYL = fxL;
        float fxR = dryR - dcXR + dcR * dcYR;  dcXR = dryR;  dcYR = fxR;

        // input envelope (for ducking)
        {
            float lvl = std::max(std::fabs(dryL), std::fabs(dryR));
            env += (lvl - env) * (lvl > env ? kAtk : kRel);
        }

        // ---- EQ before the reverb (optional) ----
        if (eqPre)
        {
            fxL = hiL.process(loL.process(fxL));
            fxR = hiR.process(loR.process(fxR));
        }

        // ---- reverb (fully wet) ----
        float wetL = 0.f, wetR = 0.f;
        engine.process(sr, fxL, fxR, wetL, wetR);

        // ---- EQ after the reverb (default) ----
        if (!eqPre)
        {
            wetL = hiL.process(loL.process(wetL));
            wetR = hiR.process(loR.process(wetR));
        }

        // ---- low cut (12 dB/oct high-pass) ----
        wetL -= hpL1.process(wetL);   wetL -= hpL2.process(wetL);
        wetR -= hpR1.process(wetR);   wetR -= hpR2.process(wetR);

        // ---- ducking ----
        const float duckGain = 1.f - sDuck * clampT(env / 3.f, 0.f, 1.f);
        wetL *= duckGain;
        wetR *= duckGain;

        // ---- dry / wet (same taper as the original engine mix) ----
        const float wetAmt = 1.f - (float) cb(1.0 - sMix);
        float outL = dryL * (1.f - wetAmt) + wetL * wetAmt;
        float outR = dryR * (1.f - wetAmt) + wetR * wetAmt;

        outL *= sOut;
        outR *= sOut;

        // ---- click-free bypass (raw input, tails keep running) ----
        outL = outL * (1.f - sBypass) + rawL * sBypass;
        outR = outR * (1.f - sBypass) + rawR * sBypass;

        // ---- output protection ----
        if (!std::isfinite(outL) || !std::isfinite(outR)) {
            outL = outR = 0.f;
            engine.initialize();
        }

        if (std::fabs(outL) > 10.f || std::fabs(outR) > 10.f)
            clipTimer = 0.3f;
        else
            clipTimer = std::max(0.f, clipTimer - args.sampleTime);

        if (softClip) {
            outL = psyrv::softLimit(outL);
            outR = psyrv::softLimit(outR);
        }

        outputs[AUDIO_L_OUTPUT].setVoltage(outL);
        outputs[AUDIO_R_OUTPUT].setVoltage(outR);

        // ---- display data ----
        dispPre  = c[PREDELAY_PARAM] * 100.f;
        dispSize = c[SIZE_PARAM];
        dispMix  = c[MIX_PARAM];
        dispLo   = c[LO_GAIN_PARAM];
        dispHi   = c[HI_GAIN_PARAM];
        dispBypass = tBypass > 0.5f;
        dispFreeze = freezeOn;

        const float decayRate = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(outL), peakL * decayRate);
        peakR = std::max(std::fabs(outR), peakR * decayRate);

        const float monoOut = 0.5f * (outL + outR);

        fftRing[fftHead] = monoOut;
        fftHead = (fftHead + 1) & (P2_FFT_RING - 1);

        if (scopeCount == 0) scopeBMin = scopeBMax = monoOut;
        else {
            scopeBMin = std::min(scopeBMin, monoOut);
            scopeBMax = std::max(scopeBMax, monoOut);
        }
        const int bucket = std::max(1, (int) (sr * 0.0025f));
        if (++scopeCount >= bucket) {
            scopeCount = 0;
            scopeMin[scopeHead] = scopeBMin;
            scopeMax[scopeHead] = scopeBMax;
            scopeHead = (scopeHead + 1) % P2_SCOPE_N;
        }
    }
};

// =====================================================================
//  Layout constants (26 HP = 390 px wide, 6 columns)
// =====================================================================
static const float P2_PANEL_W = RACK_GRID_WIDTH * 26;   // 390
static const float P2_PANEL_H = RACK_GRID_HEIGHT;       // 380

static const float P2_DISP_W = 350.f;
static const float P2_DISP_H = 38.f;
static const float P2_DISP_X = (P2_PANEL_W - P2_DISP_W) / 2.f;
static const float P2_DISP_Y = 46.f;

static const float P2_CELL_W = 60.f;
static const float P2_CELL_H = 70.f;
static const float P2_ROW[3] = {112.f, 183.f, 254.f};   // knob centre of each row
static const float P2_PORT_Y = 348.f;

static float p2ColX(int col) { return P2_PANEL_W / 12.f * (2 * col + 1); }

// =====================================================================
//  Theme + drawing helpers
// =====================================================================
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

void drawLogo(NVGcontext* vg, float cx, float cy) {
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
    nvgRect(vg, tx + s,       ty,            s, s);
    nvgRect(vg, tx,           ty + s * 1.4f, s, s);
    nvgRect(vg, tx + s * 2.f, ty + s * 1.4f, s, s);
    nvgFillColor(vg, colAccent());
    nvgFill(vg);

    drawText(vg, cx + 5.f, cy + 0.5f, "AGMM", 11.f, colText(),
             NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
}

// ------------------------------------------------------------
// Panel background
// ------------------------------------------------------------
struct P2PanelBackground : TransparentWidget {
    P2PanelBackground() { box.size = Vec(P2_PANEL_W, P2_PANEL_H); }

    void plate(NVGcontext* vg, float cx, float ry) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, cx - P2_CELL_W / 2.f, ry - 17.f, P2_CELL_W, P2_CELL_H, 7.f);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 55));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    void section(NVGcontext* vg, float y, const char* txt) {
        float x1 = 14.f, x2 = P2_PANEL_W - 14.f, cx = P2_PANEL_W / 2.f;
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

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, P2_PANEL_W, P2_PANEL_H);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, P2_PANEL_H, colBgTop(), colBgBottom()));
        nvgFill(vg);

        nvgStrokeWidth(vg, 1.f);
        for (int y = 0; y < (int) P2_PANEL_H; y += 3) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, y + 0.5f);
            nvgLineTo(vg, P2_PANEL_W, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, (y % 6 == 0) ? 5 : 2));
            nvgStroke(vg);
        }

        nvgBeginPath(vg);
        nvgRect(vg, 0.5f, 0.5f, P2_PANEL_W - 1.f, P2_PANEL_H - 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        drawLogo(vg, P2_PANEL_W / 2.f, 13.f);
        drawText(vg, P2_PANEL_W / 2.f, 31.f, "PSYREVERB 2", 13.f, colText(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 4.f);

        // Display bezel
        float bx = P2_DISP_X - 5.f, by = P2_DISP_Y - 5.f;
        float bw = P2_DISP_W + 10.f, bh = P2_DISP_H + 10.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx - 3.f, by - 3.f, bw + 6.f, bh + 6.f, 8.f);
        nvgFillPaint(vg, nvgLinearGradient(vg, bx, by, bx, by + bh,
                                           nvgRGB(0x0b, 0x0c, 0x0e), nvgRGB(0x2c, 0x30, 0x38)));
        nvgFill(vg);
        nvgBeginPath(vg);
        nvgRoundedRect(vg, bx, by, bw, bh, 6.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);

        // Control cells: 14 knobs + FREEZE + CLEAR + BYPASS + CV RANGE
        for (int i = 0; i < 18; ++i) {
            int row = i / 6, col = i % 6;
            float cx = p2ColX(col), ry = P2_ROW[row];
            plate(vg, cx, ry);
            if (i < P2_NUM_CTRL)
                drawText(vg, cx, ry + 21.f, P2_CTL[i].name, 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            else if (i == 14)
                drawText(vg, cx, ry + 44.f, "FREEZE", 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            else if (i == 15)
                drawText(vg, cx, ry + 44.f, "CLEAR", 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
            else if (i == 16)
                drawText(vg, cx, ry + 44.f, "BYPASS", 8.f, colDim(),
                         NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        // I/O strip
        section(vg, 314.f, "I / O");
        const char* io[6] = {"L IN", "R IN", "FRZ GATE", "CLR TRIG", "L OUT", "R OUT"};
        for (int i = 0; i < 6; ++i) {
            float cx = p2ColX(i);
            nvgBeginPath(vg);
            nvgRoundedRect(vg, cx - 28.f, 322.f, 56.f, 42.f, 6.f);
            nvgFillColor(vg, nvgRGBA(0, 0, 0, 90));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            drawText(vg, cx, 330.f, io[i], 8.f, colDim(),
                     NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);
        }

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// Display: WAVE / BARS / SPECTRUM (click to cycle)
// ------------------------------------------------------------
const char* P2_VIS_NAMES[3] = {"WAVE", "BARS", "SPECTRUM"};

struct P2DisplayWidget : Widget {
    PsyReverb2Module* module = nullptr;

    psyrv::FFT fft;
    float frame[P2_FFT_N];
    float re[P2_FFT_N], im[P2_FFT_N];
    float binDb[P2_FFT_N / 2];      // smoothed magnitude per bin (dB re 5 V)
    float barLvl[P2_BARS];
    float barPeak[P2_BARS];

    P2DisplayWidget(PsyReverb2Module* m) : module(m) {
        box.pos  = Vec(P2_DISP_X, P2_DISP_Y);
        box.size = Vec(P2_DISP_W, P2_DISP_H);
        for (int i = 0; i < P2_FFT_N; ++i) frame[i] = re[i] = im[i] = 0.f;
        for (int i = 0; i < P2_FFT_N / 2; ++i) binDb[i] = -120.f;
        for (int i = 0; i < P2_BARS; ++i) barLvl[i] = barPeak[i] = 0.f;
    }

    static float scopeW() { return P2_DISP_W - 16.f; }

    void onButton(const ButtonEvent& e) override {
        if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            module->visMode = (module->visMode + 1) % PsyReverb2Module::VIS_COUNT;
            e.consume(this);
            return;
        }
        Widget::onButton(e);
    }

    float sampleRate() const { return (module && module->dspSr > 0.f) ? module->dspSr : 48000.f; }

    void updateSpectrum() {
        const int half = P2_FFT_N / 2;
        if (module) {
            int start = (module->fftHead - P2_FFT_N + P2_FFT_RING) & (P2_FFT_RING - 1);
            for (int i = 0; i < P2_FFT_N; ++i)
                frame[i] = module->fftRing[(start + i) & (P2_FFT_RING - 1)];
            fft.forward(frame, re, im);
            for (int k = 1; k < half; ++k) {
                float mag = std::sqrt(re[k] * re[k] + im[k] * im[k]);
                float amp = 4.f * mag / (float) P2_FFT_N;               // Hann-window corrected
                float db = 20.f * std::log10(amp / 5.f + 1e-7f);
                db = clampT(db, -120.f, 12.f);
                float s = binDb[k];
                binDb[k] = (db > s) ? s + (db - s) * 0.6f : s + (db - s) * 0.14f;
            }
        } else {
            // preview data for the module browser
            float binHz = 48000.f / P2_FFT_N;
            for (int k = 1; k < half; ++k) {
                float f = k * binHz;
                binDb[k] = -14.f - 17.f * std::log10(1.f + f / 150.f)
                           - 5.f * std::fabs(std::sin(f * 0.0017f));
            }
        }
    }

    float levelAt(float fLo, float fHi, float binHz) const {
        const int maxB = P2_FFT_N / 2 - 2;
        float b0 = fLo / binHz, b1 = fHi / binHz;
        if (b1 - b0 < 1.f) {
            float cc = clampT(0.5f * (b0 + b1), 1.f, (float) maxB);
            int i = (int) cc;
            float frac = cc - (float) i;
            return binDb[i] * (1.f - frac) + binDb[i + 1] * frac;
        }
        int i0 = std::max(1, (int) std::ceil(b0));
        int i1 = std::min(maxB, (int) std::floor(b1));
        if (i1 < i0) i1 = i0;
        float m = -200.f;
        for (int i = i0; i <= i1; ++i)
            m = std::max(m, binDb[i]);
        return m;
    }

    float bandNorm(float fLo, float fHi, float binHz) const {
        float fc = std::sqrt(fLo * fHi);
        float db = levelAt(fLo, fHi, binHz) + P2_TILT * std::log2(fc / 1000.f);
        return clampT((db - P2_DB_FLOOR) / (-P2_DB_FLOOR), 0.f, 1.f);
    }

    // ---- views ----
    void drawWave(NVGcontext* vg, float sw) {
        const float mid = P2_DISP_H / 2.f;
        const float amp = mid - 2.f;
        const float colW = sw / (float) P2_SCOPE_N;

        for (int pass = 0; pass < 2; ++pass) {   // 0: glow, 1: crisp
            nvgBeginPath(vg);
            for (int i = 0; i < P2_SCOPE_N; ++i) {
                float vmin, vmax;
                if (module) {
                    int idx = (module->scopeHead + i) % P2_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                } else {
                    float t = (float) i / P2_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - clampT(vmax / 5.f, -1.f, 1.f) * amp;
                float y1 = mid - clampT(vmin / 5.f, -1.f, 1.f) * amp;
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
    }

    void drawFreqGrid(NVGcontext* vg, float sw) {
        const float fs[3] = {100.f, 1000.f, 10000.f};
        const char* ls[3] = {"100", "1K", "10K"};
        const float lr = std::log(P2_FMAX / P2_FMIN);
        nvgBeginPath(vg);
        for (int i = 0; i < 3; ++i) {
            float x = std::floor(sw * std::log(fs[i] / P2_FMIN) / lr) + 0.5f;
            nvgMoveTo(vg, x, 0);
            nvgLineTo(vg, x, P2_DISP_H);
        }
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
        for (int i = 0; i < 3; ++i) {
            float x = sw * std::log(fs[i] / P2_FMIN) / lr;
            drawText(vg, x + 2.f, P2_DISP_H - 5.f, ls[i], 6.f, nvgRGBA(0x7a, 0x85, 0x94, 200),
                     NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 0.5f);
        }
    }

    void drawBars(NVGcontext* vg, float sw) {
        const float binHz = sampleRate() / P2_FFT_N;
        const float ratio = P2_FMAX / P2_FMIN;
        const float slot = sw / (float) P2_BARS;
        const float barW = slot - 2.f;
        const float H = P2_DISP_H;

        drawFreqGrid(vg, sw);

        nvgBeginPath(vg);
        for (int b = 0; b < P2_BARS; ++b) {
            float fLo = P2_FMIN * std::pow(ratio, (float) b / P2_BARS);
            float fHi = P2_FMIN * std::pow(ratio, (float) (b + 1) / P2_BARS);
            float n = bandNorm(fLo, fHi, binHz);
            barLvl[b] = (n > barLvl[b]) ? n : barLvl[b] * 0.86f + n * 0.14f;
            barPeak[b] = std::max(barLvl[b], barPeak[b] - 0.010f);
            float h = barLvl[b] * (H - 3.f);
            if (h > 0.5f)
                nvgRect(vg, b * slot + 1.f, H - 1.f - h, barW, h);
        }
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, H, 0, 0,
                                           nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
        nvgFill(vg);

        nvgBeginPath(vg);
        for (int b = 0; b < P2_BARS; ++b) {
            float y = H - 1.f - barPeak[b] * (H - 3.f);
            if (barPeak[b] > 0.02f)
                nvgRect(vg, b * slot + 1.f, y - 1.f, barW, 1.5f);
        }
        nvgFillColor(vg, nvgRGBA(0xd8, 0xf8, 0xff, 230));
        nvgFill(vg);
    }

    void drawSpectrum(NVGcontext* vg, float sw) {
        const float binHz = sampleRate() / P2_FFT_N;
        const float ratio = P2_FMAX / P2_FMIN;
        const float H = P2_DISP_H;
        float xs[P2_SPEC_COLS], ys[P2_SPEC_COLS];

        drawFreqGrid(vg, sw);

        for (int c = 0; c < P2_SPEC_COLS; ++c) {
            float fLo = P2_FMIN * std::pow(ratio, (float) c / P2_SPEC_COLS);
            float fHi = P2_FMIN * std::pow(ratio, (float) (c + 1) / P2_SPEC_COLS);
            float n = bandNorm(fLo, fHi, binHz);
            xs[c] = (c + 0.5f) * sw / P2_SPEC_COLS;
            ys[c] = H - 1.f - n * (H - 3.f);
        }

        nvgBeginPath(vg);
        nvgMoveTo(vg, 0.f, H);
        nvgLineTo(vg, 0.f, ys[0]);
        for (int c = 0; c < P2_SPEC_COLS; ++c)
            nvgLineTo(vg, xs[c], ys[c]);
        nvgLineTo(vg, sw, ys[P2_SPEC_COLS - 1]);
        nvgLineTo(vg, sw, H);
        nvgClosePath(vg);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, H,
                                           nvgRGBA(0x3d, 0xe0, 0xff, 120), nvgRGBA(0xb0, 0x5a, 0xff, 8)));
        nvgFill(vg);

        for (int pass = 0; pass < 2; ++pass) {
            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, ys[0]);
            for (int c = 0; c < P2_SPEC_COLS; ++c)
                nvgLineTo(vg, xs[c], ys[c]);
            nvgLineTo(vg, sw, ys[P2_SPEC_COLS - 1]);
            nvgStrokeColor(vg, pass == 0 ? nvgRGBA(0x3d, 0xe0, 0xff, 50) : nvgRGB(0x8a, 0xee, 0xff));
            nvgStrokeWidth(vg, pass == 0 ? 3.f : 1.1f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
        }
    }

    // ---- main ----
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;

        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, P2_DISP_W, P2_DISP_H);
        nvgFillPaint(vg, nvgRadialGradient(vg, P2_DISP_W / 2.f, P2_DISP_H / 2.f, 20.f,
                                           P2_DISP_W * 0.6f,
                                           nvgRGB(0x08, 0x10, 0x16), nvgRGB(0x02, 0x04, 0x06)));
        nvgFill(vg);

        nvgBeginPath(vg);
        for (float x = 0.f; x <= scopeW(); x += 28.f) {
            nvgMoveTo(vg, x + 0.5f, 0);
            nvgLineTo(vg, x + 0.5f, P2_DISP_H);
        }
        nvgMoveTo(vg, 0, P2_DISP_H / 2.f + 0.5f);
        nvgLineTo(vg, scopeW(), P2_DISP_H / 2.f + 0.5f);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
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
        const float sw = scopeW();
        int mode = module ? module->visMode : (int) PsyReverb2Module::VIS_SPECTRUM;

        if (mode != PsyReverb2Module::VIS_WAVE)
            updateSpectrum();

        nvgSave(vg);
        nvgScissor(vg, 0, 0, sw, P2_DISP_H);
        if (mode == PsyReverb2Module::VIS_WAVE)
            drawWave(vg, sw);
        else if (mode == PsyReverb2Module::VIS_BARS)
            drawBars(vg, sw);
        else
            drawSpectrum(vg, sw);
        nvgResetScissor(vg);
        nvgRestore(vg);

        // Level meters (L / R)
        for (int ch = 0; ch < 2; ++ch) {
            float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 3.f : 2.4f);
            float lvl = std::min(peak / 10.f, 1.f);
            float x = sw + 3.f + ch * 6.f;
            nvgBeginPath(vg);
            nvgRect(vg, x, 1.f, 4.f, P2_DISP_H - 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgFill(vg);
            if (lvl > 0.01f) {
                float h = lvl * (P2_DISP_H - 2.f);
                nvgBeginPath(vg);
                nvgRect(vg, x, P2_DISP_H - 1.f - h, 4.f, h);
                nvgFillPaint(vg, nvgLinearGradient(vg, 0, P2_DISP_H, 0, 0,
                                                   nvgRGB(0x3d, 0xe0, 0xff), nvgRGB(0xb0, 0x5a, 0xff)));
                nvgFill(vg);
            }
        }

        // Scanlines
        nvgBeginPath(vg);
        for (int y = 0; y < (int) P2_DISP_H; y += 2)
            nvgRect(vg, 0, y + 1, P2_DISP_W, 1);
        nvgFillColor(vg, nvgRGBA(0, 0, 0, 40));
        nvgFill(vg);

        // Glass reflection
        nvgBeginPath(vg);
        nvgRect(vg, 0, 0, P2_DISP_W, P2_DISP_H * 0.4f);
        nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, P2_DISP_H * 0.4f,
                                           nvgRGBA(255, 255, 255, 16), nvgRGBA(255, 255, 255, 0)));
        nvgFill(vg);

        // Readouts
        char buf[48];
        float pre  = module ? module->dispPre  : 0.f;
        float size = module ? module->dispSize : 0.5f;
        float mix  = module ? module->dispMix  : 1.f;
        float lo   = module ? module->dispLo   : 0.f;
        float hi   = module ? module->dispHi   : 0.f;
        NVGcolor ro = nvgRGBA(0x3d, 0xe0, 0xff, 190);

        snprintf(buf, sizeof(buf), "PRE %3.0f MS  SIZE %3.0f", pre, size * 100.f);
        drawText(vg, 4.f, 7.f, buf, 7.f, ro, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "LO %+.1f  HI %+.1f", lo, hi);
        drawText(vg, sw / 2.f + 30.f, 7.f, buf, 7.f, ro, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);
        snprintf(buf, sizeof(buf), "MIX %3.0f%%", mix * 100.f);
        drawText(vg, sw - 4.f, 7.f, buf, 7.f, ro, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);

        // Mode name (bottom-left)
        drawText(vg, 4.f, P2_DISP_H - 5.f, P2_VIS_NAMES[mode], 6.f,
                 nvgRGBA(0xb0, 0x5a, 0xff, 220), NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE, 1.f);

        // Status flags (bottom-right)
        if (module) {
            char st[32] = "";
            if (module->dispFreeze) strcat(st, "FRZ ");
            if (module->dispBypass) strcat(st, "BYP ");
            if (module->clipTimer > 0.f) strcat(st, "CLIP");
            if (st[0])
                drawText(vg, sw - 4.f, P2_DISP_H - 5.f, st, 6.f,
                         module->clipTimer > 0.f ? nvgRGBA(0xff, 0x50, 0x60, 255)
                                                 : nvgRGBA(0xff, 0xc8, 0x50, 230),
                         NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE, 1.f);
        }

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// CV range cell (click to toggle)
// ------------------------------------------------------------
struct P2CvRangeWidget : Widget {
    PsyReverb2Module* module = nullptr;

    P2CvRangeWidget(PsyReverb2Module* m, float cx, float ry) : module(m) {
        box.pos  = Vec(cx - P2_CELL_W / 2.f, ry - 17.f);
        box.size = Vec(P2_CELL_W, P2_CELL_H);
    }

    void onButton(const ButtonEvent& e) override {
        if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            module->cvRangeMode = (module->cvRangeMode == PsyReverb2Module::BIPOLAR_M5_5)
                                      ? PsyReverb2Module::UNIPOLAR_0_10
                                      : PsyReverb2Module::BIPOLAR_M5_5;
            e.consume(this);
            return;
        }
        Widget::onButton(e);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) {
            Widget::drawLayer(args, layer);
            return;
        }
        NVGcontext* vg = args.vg;
        bool bipolar = module ? (module->cvRangeMode == PsyReverb2Module::BIPOLAR_M5_5) : true;

        drawText(vg, box.size.x / 2.f, 21.f + 17.f, "CV RANGE", 8.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.5f);

        float sx = 6.f, sy = 10.f, sw = box.size.x - 12.f, sh = 24.f;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, sx, sy, sw, sh, 4.f);
        nvgFillColor(vg, nvgRGB(0x05, 0x07, 0x0a));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        drawText(vg, box.size.x / 2.f, sy + sh / 2.f, bipolar ? "-5/+5V" : "0/+10V", 10.f,
                 colAccent(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        nvgBeginPath(vg);
        nvgRoundedRect(vg, sx + 4.f, 44.f, sw - 8.f, 2.f, 1.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 20));
        nvgFill(vg);
        nvgBeginPath(vg);
        if (bipolar)
            nvgRoundedRect(vg, box.size.x / 2.f - 12.f, 44.f, 24.f, 2.f, 1.f);
        else
            nvgRoundedRect(vg, sx + 4.f, 44.f, 24.f, 2.f, 1.f);
        nvgFillColor(vg, nvgRGB(0xb0, 0x5a, 0xff));
        nvgFill(vg);

        drawText(vg, box.size.x / 2.f, 58.f, "CLICK", 7.f, colDim(),
                 NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.f);

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// Context-menu helpers
// ------------------------------------------------------------
struct P2ChoiceItem : MenuItem {
    std::function<bool()> isOn;
    std::function<void()> action;
    void onAction(const ActionEvent& e) override { action(); }
    void step() override {
        rightText = (isOn && isOn()) ? "✔" : "";
        MenuItem::step();
    }
};

void addChoice(Menu* menu, const std::string& label,
               std::function<bool()> isOn, std::function<void()> action) {
    P2ChoiceItem* item = createMenuItem<P2ChoiceItem>(label);
    item->isOn = isOn;
    item->action = action;
    menu->addChild(item);
}

} // namespace

// =====================================================================
//  Widget
// =====================================================================
struct PsyReverb2Widget : ModuleWidget {
    PsyReverb2Widget(PsyReverb2Module* module) {
        setModule(module);
        box.size = Vec(P2_PANEL_W, P2_PANEL_H);

        addChild(new P2PanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new P2DisplayWidget(module));

        // 14 controls: knob on top, CV jack (left) and attenuverter (right) below
        for (int i = 0; i < P2_NUM_CTRL; ++i) {
            int row = i / 6, col = i % 6;
            float cx = p2ColX(col), ry = P2_ROW[row];
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, ry), module, i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 16.f, ry + 40.f), module,
                                                         PsyReverb2Module::CV_FIRST + i));
            addParam(createParamCentered<Trimpot>(Vec(cx + 16.f, ry + 40.f), module,
                                                  PsyReverb2Module::ATTN_FIRST + i));
        }

        // Row 3: FREEZE (col 2), CLEAR (col 3), BYPASS (col 4), CV range (col 5)
        const float r3 = P2_ROW[2];
        addParam(createParamCentered<TL1105>(Vec(p2ColX(2), r3 + 6.f), module,
                                             PsyReverb2Module::FREEZE_PARAM));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(p2ColX(2), r3 + 24.f), module,
                                                             PsyReverb2Module::FREEZE_LIGHT));
        addParam(createParamCentered<TL1105>(Vec(p2ColX(3), r3 + 6.f), module,
                                             PsyReverb2Module::CLEAR_PARAM));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(p2ColX(3), r3 + 24.f), module,
                                                           PsyReverb2Module::CLEAR_LIGHT));
        addParam(createParamCentered<CKSS>(Vec(p2ColX(4), r3 + 14.f), module,
                                           PsyReverb2Module::BYPASS_PARAM));
        addChild(new P2CvRangeWidget(module, p2ColX(5), r3));

        // I/O strip
        addInput (createInputCentered <DarkPJ301MPort>(Vec(p2ColX(0), P2_PORT_Y), module,
                                                       PsyReverb2Module::AUDIO_L_INPUT));
        addInput (createInputCentered <DarkPJ301MPort>(Vec(p2ColX(1), P2_PORT_Y), module,
                                                       PsyReverb2Module::AUDIO_R_INPUT));
        addInput (createInputCentered <DarkPJ301MPort>(Vec(p2ColX(2), P2_PORT_Y), module,
                                                       PsyReverb2Module::FREEZE_GATE_INPUT));
        addInput (createInputCentered <DarkPJ301MPort>(Vec(p2ColX(3), P2_PORT_Y), module,
                                                       PsyReverb2Module::CLEAR_TRIG_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(p2ColX(4), P2_PORT_Y), module,
                                                       PsyReverb2Module::AUDIO_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(p2ColX(5), P2_PORT_Y), module,
                                                       PsyReverb2Module::AUDIO_R_OUTPUT));
    }

    void appendContextMenu(Menu* menu) override {
        PsyReverb2Module* m = dynamic_cast<PsyReverb2Module*>(this->module);
        assert(m);

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));
        addChoice(menu, "-5V to +5V (bipolar)",
                  [m]() { return m->cvRangeMode == PsyReverb2Module::BIPOLAR_M5_5; },
                  [m]() { m->cvRangeMode = PsyReverb2Module::BIPOLAR_M5_5; });
        addChoice(menu, "0V to +10V (unipolar)",
                  [m]() { return m->cvRangeMode == PsyReverb2Module::UNIPOLAR_0_10; },
                  [m]() { m->cvRangeMode = PsyReverb2Module::UNIPOLAR_0_10; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Display"));
        for (int v = 0; v < PsyReverb2Module::VIS_COUNT; ++v)
            addChoice(menu, P2_VIS_NAMES[v],
                      [m, v]() { return m->visMode == v; },
                      [m, v]() { m->visMode = v; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("EQ Position"));
        addChoice(menu, "After reverb (wet output)",
                  [m]() { return !m->eqPre; },
                  [m]() { m->eqPre = false; });
        addChoice(menu, "Before reverb (shapes the tank input)",
                  [m]() { return m->eqPre; },
                  [m]() { m->eqPre = true; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Low Shelf Frequency"));
        const float loF[4] = {100.f, 200.f, 300.f, 500.f};
        for (int i = 0; i < 4; ++i) {
            float f = loF[i];
            addChoice(menu, std::to_string((int) f) + " Hz",
                      [m, f]() { return m->loFreq == f; },
                      [m, f]() { m->loFreq = f; });
        }

        menu->addChild(createMenuLabel("High Shelf Frequency"));
        const float hiF[4] = {2500.f, 4000.f, 6000.f, 8000.f};
        for (int i = 0; i < 4; ++i) {
            float f = hiF[i];
            addChoice(menu, std::to_string((int) f) + " Hz",
                      [m, f]() { return m->hiFreq == f; },
                      [m, f]() { m->hiFreq = f; });
        }

        menu->addChild(new MenuSeparator);
        addChoice(menu, "Output soft limiter (10 V)",
                  [m]() { return m->softClip; },
                  [m]() { m->softClip = !m->softClip; });
        addChoice(menu, "Clear reverb tank",
                  nullptr,
                  [m]() { m->engine.initialize(); });
    }
};

// =====================================================================
//  Model
// =====================================================================
Model* modelPsyReverb2 = createModel<PsyReverb2Module, PsyReverb2Widget>("PsyReverb2");