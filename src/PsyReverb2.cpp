// PsyReverb2.cpp
// Single-file VCV Rack module styled to match AGMM's Reverb panel.
// Based on Airwindows Galactic / Sapphire Galaxy, extended with damping,
// predelay, mod, width, freeze, shelving EQ, low cut, ducking, CV + attenuverter
// per knob, and a Wave / Bars / Spectrum display.
//
// ============================================================================
// OVERVIEW
// ============================================================================
// PsyReverb2 is a 26 HP "galactic" reverb: a feedback delay network based on
// Airwindows Galactic/Sapphire Galaxy, wrapped in an AGMM-styled panel with
// a full set of extra tone and workflow features:
//   - 14 main parameters, each with a dedicated CV input and bipolar
//     attenuverter (input gain, predelay, size, decay, brightness, damping,
//     mod, width, low/high shelf gain, low cut, ducking, mix, output level).
//   - Pre-reverb or post-reverb shelving EQ (menu-selectable), low cut,
//     input ducking, freeze (button latch + gate), tank clear (button +
//     trigger), and a click-free bypass.
//   - A 350x56 display switchable between WAVE, BARS and SPECTRUM views,
//     with header readouts (predelay, size, lo/hi gain, mix) and a status
//     column showing the active view, FRZ/BYP/CLIP flags, and L/R meters.
//   - A clickable CV-range cell (bipolar ±5 V or unipolar 0..10 V) and a
//     right-click menu with the same choice plus view mode, EQ position,
//     shelf frequencies, soft-limiter and a "clear tank" item.
//
// File structure:
//   1. Small utilities (clampT, sq, cb) and module constants.
//   2. psyrv namespace: DSP helpers (softLimit, OnePole, Biquad, FFT),
//      the StereoFrame / StereoQuad / DelayLine data types, and the
//      Airwindows-style Engine (13 delay lines, 4-quad feedback matrix,
//      tank stir, damping, predelay, freeze, modulation, stereo widening).
//   3. P2_CTL control table for the 14 knobs.
//   4. PsyReverb2Module: parameter/CV reading, smoothing, EQ, low cut,
//      ducking, freeze/clear handling, process(), and display snapshots.
//   5. Layout constants (26 HP = 390x380 px).
//   6. Anonymous-namespace UI: P2PanelBackground, P2DisplayWidget,
//      P2CvRangeWidget, P2ChoiceItem + addChoice.
//   7. PsyReverb2Widget: places all controls and the context menu.
//   8. Model registration.
// ============================================================================

#include "plugin.hpp"
#include "agmm_ui.hpp"

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
// Clamps x into [lo, hi] for any arithmetic type.
template <typename T>
inline T clampT(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }
// Returns x squared.
inline double sq(double x) { return x * x; }
// Returns x cubed.
inline double cb(double x) { return x * x * x; }

// ---------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------
static const int P2_SCOPE_N = 160;   // waveform columns
static const int P2_FFT_N = 1024;    // FFT size
static const int P2_FFT_RING = 4096; // audio -> UI ring buffer (power of 2)
static const int P2_BARS = 24;       // analyzer bars
static const int P2_SPEC_COLS = 150; // spectrum curve resolution
static const int P2_NUM_CTRL = 14;   // knobs with CV + attenuverter
static const float P2_FMIN = 30.f;
static const float P2_FMAX = 18000.f;
static const float P2_DB_FLOOR = -72.f; // analyzer floor (dB re 5 V)
static const float P2_TILT = 3.f;       // dB/oct display tilt (flattens pink noise)
static const float P2_TWO_PI = 6.28318530717958647692f;

// =====================================================================
//  Helpers + Reverb Engine
// =====================================================================
namespace psyrv
{
    constexpr int NDELAYS = 13;
    constexpr int MIN_CYCLE = 1;
    constexpr int MAX_CYCLE = 4;
    constexpr int MOD_TANK_DELAY = 256;
    constexpr int MOD_TANK_ALLOC = 1024;
    constexpr int MAX_PREDELAY = 24000;

    // ---- small DSP helpers ----------------------------------------
    // Transparent up to 6 V, smooth knee towards a ceiling of 10 V
    // Soft-limits a sample: linear up to ±6 V, then a tanh shoulder that
    // asymptotes to ±10 V.
    inline float softLimit(float x)
    {
        const float k = 6.f, r = 4.f;
        float a = std::fabs(x);
        if (a <= k)
            return x;
        float y = k + r * std::tanh((a - k) / r);
        return x < 0.f ? -y : y;
    }

    // Simple one-pole low-pass with a settable cutoff.
    struct OnePole
    {
        float z = 0.f;
        float coeff = 1.f;
        // Sets the low-pass cutoff in Hz (clamped to [1, 0.45*fs]).
        void setCutoff(float freq, float sr)
        {
            if (freq < 1.f)
                freq = 1.f;
            if (freq > sr * 0.45f)
                freq = sr * 0.45f;
            coeff = 1.f - std::exp(-P2_TWO_PI * freq / sr);
        }
        // Processes one sample and returns the filtered output.
        float process(float in)
        {
            z += coeff * (in - z);
            return z;
        }
    };

    // RBJ shelving biquad, slope S = 1, transposed direct form II
    // A 2nd-order shelving filter. Coefficients can be shared between L and
    // R via copyCoeffs() while each channel keeps its own internal state.
    struct Biquad
    {
        float b0 = 1.f, b1 = 0.f, b2 = 0.f, a1 = 0.f, a2 = 0.f;
        float z1 = 0.f, z2 = 0.f;

        // Processes one sample (transposed direct form II) and returns the
        // filtered output.
        float process(float x)
        {
            float y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }
        // Copies coefficients (but not internal state) from another biquad.
        void copyCoeffs(const Biquad &o)
        {
            b0 = o.b0;
            b1 = o.b1;
            b2 = o.b2;
            a1 = o.a1;
            a2 = o.a2;
        }

        // Configures the biquad as an RBJ low shelf at `freq` Hz with `dB`
        // gain.
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

        // Configures the biquad as an RBJ high shelf at `freq` Hz with `dB`
        // gain.
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
    // Precomputes the twiddle factors, Hann window and bit-reversal table for
    // an N-point forward FFT.
    struct FFT
    {
        static constexpr int N = P2_FFT_N;
        static constexpr int LOG2N = 10;
        float cosT[N / 2], sinT[N / 2];
        float window[N];
        int rev[N];

        // Precomputes twiddles, window and bit-reversal permutation.
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

        // Forward FFT of windowed `in`, writing complex output into re/im.
        void forward(const float *in, float *re, float *im) const
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
    // A stereo sample: two doubles for L/R. Supports the small set of vector
    // operations the tank uses.
    struct StereoFrame
    {
        double ch[2];
        StereoFrame() : ch{0.0, 0.0} {}
        StereoFrame(double l, double r) : ch{l, r} {}
        // Zeroes both channels.
        void clear() { ch[0] = ch[1] = 0.0; }
        // Element-wise addition.
        StereoFrame operator+(const StereoFrame &o) const { return {ch[0] + o.ch[0], ch[1] + o.ch[1]}; }
        // Element-wise subtraction.
        StereoFrame operator-(const StereoFrame &o) const { return {ch[0] - o.ch[0], ch[1] - o.ch[1]}; }
        // Scalar multiplication.
        StereoFrame operator*(double s) const { return {ch[0] * s, ch[1] * s}; }
        // Scalar division.
        StereoFrame operator/(double d) const { return {ch[0] / d, ch[1] / d}; }
        // Returns a copy with L and R swapped.
        StereoFrame flip() const { return {ch[1], ch[0]}; }
    };

    // Four stereo frames forming a quad (the tank uses groups of 4 lines).
    struct StereoQuad
    {
        StereoFrame q[4];
        // Zeroes all four frames.
        void clear()
        {
            for (int i = 0; i < 4; i++)
                q[i].clear();
        }
        // Returns the element-wise sum of the four frames.
        StereoFrame sum() const { return q[0] + q[1] + q[2] + q[3]; }
    };

    // Circular delay line of StereoFrames with head/tail access. `count` is
    // the write index; `delay` is the effective length in frames; `tail()`
    // returns the read point.
    struct DelayLine
    {
        std::vector<StereoFrame> buf;
        int count = 1;
        int delay = 1;
        // Zeroes all frames and resets the write index.
        void clear()
        {
            count = 1;
            for (auto &f : buf)
                f.clear();
        }
        // Advances the write index (wrapping).
        void advance()
        {
            ++count;
            if (count < 0 || count > delay)
                count = 0;
        }
        // Returns the read index for the current length.
        int reverse(int length) const
        {
            const int off = (length > delay) ? (delay + 1) : 0;
            return length - off;
        }
        // Returns a reference to the frame at the write head.
        StereoFrame &head() { return buf.at(count); }
        // Returns a copy of the frame at the read (tail) position.
        StereoFrame tail() { return buf.at(reverse(count)); }
    };

    // The galactic-style reverb tank: 12 fixed delay lines grouped into 3
    // quads (feedback), plus a 13th modulated tank line for the phasor read,
    // a shared predelay buffer, per-channel feedback/damp state, and IIR
    // smoothing filters.
    struct Engine
    {
        float decayKnob = 0.5f;
        float sizeKnob = 0.5f;
        float brightKnob = 0.5f;
        float dampKnob = 0.3f;
        float predelayKnob = 0.0f;
        float modKnob = 0.3f;
        float widthKnob = 0.5f;
        float mixKnob = 1.0f;
        bool freeze = false;

        uint32_t fpd[2] = {0, 0};
        double modPhase = 0.0;
        double oldFpd = 0.0;
        int cycle = 0;

        DelayLine delay[NDELAYS];
        StereoQuad feedback;
        StereoQuad dampState;
        StereoFrame iirA;
        StereoFrame iirB;
        StereoFrame lastRef[MAX_CYCLE + 1];

        std::vector<StereoFrame> predelayBuf;
        int predelayWrite = 0;

        const int tankSize[12] = {
            4801, 2909, 1153, 461,
            7607, 4217, 2269, 1597,
            3407, 1823, 859, 331};

        // Allocates every delay line and the predelay buffer at its maximum
        // size, then calls initialize() to prime the RNG and clear state.
        Engine()
        {
            delay[0].buf.resize(9700);
            delay[1].buf.resize(6000);
            delay[2].buf.resize(2320);
            delay[3].buf.resize(940);
            delay[4].buf.resize(15220);
            delay[5].buf.resize(8460);
            delay[6].buf.resize(4540);
            delay[7].buf.resize(3200);
            delay[8].buf.resize(6480);
            delay[9].buf.resize(3660);
            delay[10].buf.resize(1720);
            delay[11].buf.resize(680);
            delay[12].buf.resize(MOD_TANK_ALLOC);
            predelayBuf.resize(MAX_PREDELAY);
            initialize();
        }

        // Resets the RNG seeds, clears all delay lines, feedback/damp/IIR
        // state, the lastRef history and the predelay buffer.
        void initialize()
        {
            fpd[0] = 2756923396u;
            fpd[1] = 2341963165u;
            modPhase = 0.0;
            oldFpd = 429496.7295;
            cycle = 0;
            for (int i = 0; i < NDELAYS; i++)
                delay[i].clear();
            for (int i = 0; i <= MAX_CYCLE; i++)
                lastRef[i].clear();
            feedback.clear();
            dampState.clear();
            iirA.clear();
            iirB.clear();
            for (auto &f : predelayBuf)
                f.clear();
            predelayWrite = 0;
        }

        // Sets the decay knob (clamped 0..1).
        void setDecay(double v) { decayKnob = clampT<float>(v, 0.f, 1.f); }
        // Sets the size knob (clamped 0..1).
        void setSize(double v) { sizeKnob = clampT<float>(v, 0.f, 1.f); }
        // Sets the brightness knob (clamped 0..1).
        void setBright(double v) { brightKnob = clampT<float>(v, 0.f, 1.f); }
        // Sets the damping knob (clamped 0..1).
        void setDamp(double v) { dampKnob = clampT<float>(v, 0.f, 1.f); }
        // Sets the predelay knob (clamped 0..1).
        void setPredelay(double v) { predelayKnob = clampT<float>(v, 0.f, 1.f); }
        // Sets the modulation knob (clamped 0..1).
        void setMod(double v) { modKnob = clampT<float>(v, 0.f, 1.f); }
        // Sets the width knob (clamped 0..1).
        void setWidth(double v) { widthKnob = clampT<float>(v, 0.f, 1.f); }
        // Sets the internal wet mix knob (clamped 0..1).
        void setMix(double v) { mixKnob = clampT<float>(v, 0.f, 1.f); }
        // Sets the freeze flag.
        void setFreeze(bool v) { freeze = v; }

    private:
        // Adds a tiny pseudo-random dither to `x`, scaled by the magnitude
        // of the sample, using the given 32-bit PRNG state.
        static double dither(double x, uint32_t &f)
        {
            int expon;
            frexpf(static_cast<float>(x), &expon);
            f ^= f << 13;
            f ^= f >> 17;
            f ^= f << 5;
            return x + ((double(f) - uint32_t(0x7fffffff)) * 5.5e-36l * std::exp2(expon + 62.0));
        }

        // Writes a frame into the head of the given delay line and advances
        // its write index.
        void writeTank(int idx, const StereoFrame &f)
        {
            delay[idx].head() = f;
            delay[idx].advance();
        }

        // Reads the tails of four consecutive delay lines into a quad.
        StereoQuad readQuad(int start)
        {
            StereoQuad q;
            q.q[0] = delay[start + 0].tail();
            q.q[1] = delay[start + 1].tail();
            q.q[2] = delay[start + 2].tail();
            q.q[3] = delay[start + 3].tail();
            return q;
        }

        // Hadamard-like feedback mixing: each frame is replaced by itself
        // minus the sum of the other three frames (energy-preserving).
        static StereoQuad stir(const StereoQuad &f)
        {
            StereoQuad g;
            g.q[0] = f.q[0] - (f.q[1] + f.q[2] + f.q[3]);
            g.q[1] = f.q[1] - (f.q[0] + f.q[2] + f.q[3]);
            g.q[2] = f.q[2] - (f.q[0] + f.q[1] + f.q[3]);
            g.q[3] = f.q[3] - (f.q[0] + f.q[1] + f.q[2]);
            return g;
        }

        // Stirs a quad and writes the four resulting frames into four
        // consecutive delay lines starting at `start`.
        void writeQuad(int start, const StereoQuad &f)
        {
            const StereoQuad g = stir(f);
            for (int i = 0; i < 4; i++)
                writeTank(start + i, g.q[i]);
        }

        // Reflects `in` into a quad of delay lines, adding the flipped
        // feedback (with `regen` gain) to each frame.
        void reflect(int start, const StereoFrame &in, double regen)
        {
            for (int i = 0; i < 4; i++)
                writeTank(start + i, in + feedback.q[i].flip() * regen);
        }

        // Reads the modulated tank line at fractional delay with a phase
        // offset per channel.
        double interp(int ch, double phaseOffset)
        {
            const double depth = 127.0 * modKnob;
            const double ofs = (std::sin(modPhase + phaseOffset) + 1.0) * depth;
            const double frac = ofs - std::floor(ofs);
            const int base = delay[12].count + static_cast<int>(ofs);
            const int i1 = delay[12].reverse(base);
            const int i2 = delay[12].reverse(base + 1);
            const auto &b = delay[12].buf;
            return b.at(i1).ch[ch & 1] * (1.0 - frac) + b.at(i2).ch[ch & 1] * frac;
        }

        // Updates the short lastRef history with a new sample `s`, using
        // progressively longer averaging depending on the current cycle
        // count (helps smooth the multi-rate tank output).
        void mixDown(const StereoFrame &s)
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

        // Soft-clips to ±(1 + tanh(x∓1)), used in freeze mode to keep the
        // tank from blowing up.
        static double softClip(double x)
        {
            if (x > 1.0)
                return 1.0 + std::tanh(x - 1.0);
            if (x < -1.0)
                return -1.0 + std::tanh(x + 1.0);
            return x;
        }

    public:
        // Convenience overload of process() using float in/out.
        void process(float sampleRate, float inL, float inR, float &outL, float &outR)
        {
            double ol = 0.0, orr = 0.0;
            process(static_cast<double>(sampleRate),
                    static_cast<double>(inL),
                    static_cast<double>(inR),
                    ol, orr);
            outL = static_cast<float>(ol);
            outR = static_cast<float>(orr);
        }

        // Main double-precision process(): handles multirate cycling, tank
        // feedback, damping, freeze, modulation, width, mix and dither.
        void process(double sampleRateHz, double inL, double inR, double &outL, double &outR)
        {
            const double overallscale = sampleRateHz / 44100.0;
            const int cycleEnd = clampT<int>(static_cast<int>(std::floor(overallscale)),
                                             MIN_CYCLE, MAX_CYCLE);
            if (cycle > cycleEnd - 1)
                cycle = cycleEnd - 1;

            double regen, atten;
            if (freeze)
            {
                regen = 0.5;
                atten = 0.0;
            }
            else
            {
                regen = 0.0625 + decayKnob * 0.0625;
                atten = 1.0 - decayKnob * 0.5;
            }

            const double lowpass = sq(1.00001 - (1.0 - brightKnob)) / std::sqrt(overallscale);
            const double size = sizeKnob * 1.77 + 0.1;
            const double wet = 1.0 - cb(1.0 - mixKnob);
            const double width = widthKnob * 2.0;

            for (int i = 0; i < 12; i++)
                delay[i].delay = static_cast<int>(tankSize[i] * size);
            delay[12].delay = MOD_TANK_DELAY;

            int predelayLen = static_cast<int>(predelayKnob * 0.1 * sampleRateHz);
            predelayLen = clampT<int>(predelayLen, 0, MAX_PREDELAY - 1);

            if (std::abs(inL) < 1.18e-23)
                inL = fpd[0] * 1.18e-17;
            if (std::abs(inR) < 1.18e-23)
                inR = fpd[1] * 1.18e-17;
            const StereoFrame dry(inL, inR);

            predelayBuf[predelayWrite] = dry;
            int rdIdx = predelayWrite - predelayLen;
            while (rdIdx < 0)
                rdIdx += MAX_PREDELAY;
            const StereoFrame predelayed = predelayBuf[rdIdx];
            predelayWrite = (predelayWrite + 1) % MAX_PREDELAY;

            const double modSpeed = cb(modKnob) * 0.001;
            modPhase += oldFpd * modSpeed;
            if (modPhase > 2.0 * M_PI)
            {
                modPhase = 0.0;
                oldFpd = 0.4294967295 + (fpd[0] * 0.0000000000618);
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

                if (dampKnob > 0.001f)
                {
                    const double alpha = 1.0 - dampKnob * 0.9;
                    for (int i = 0; i < 4; i++)
                    {
                        for (int c = 0; c < 2; c++)
                        {
                            dampState.q[i].ch[c] =
                                alpha * dampState.q[i].ch[c] +
                                (1.0 - alpha) * feedback.q[i].ch[c];
                            feedback.q[i].ch[c] = dampState.q[i].ch[c];
                        }
                    }
                }

                if (freeze)
                {
                    for (int i = 0; i < 4; i++)
                        for (int c = 0; c < 2; c++)
                            feedback.q[i].ch[c] = softClip(feedback.q[i].ch[c]);
                }

                mixDown(f.sum() / 8.0);
                cycle = 0;
            }

            iirB = iirB * (1.0 - lowpass) + lastRef[cycle] * lowpass;

            const double mid = (iirB.ch[0] + iirB.ch[1]) * 0.5;
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
// Metadata for one of the 14 knobs: label, tooltip, range, default, unit,
// and display scaling.
struct P2CtlSpec
{
    const char *name;  // panel label
    const char *label; // tooltip label
    float lo, hi, def;
    const char *unit;
    float dispBase, dispMul;
};

static const P2CtlSpec P2_CTL[P2_NUM_CTRL] = {
    {"IN GAIN", "Input gain", 0.f, 2.f, 1.f, "%", 0.f, 100.f},
    {"PREDELAY", "Predelay", 0.f, 1.f, 0.f, " ms", 0.f, 100.f},
    {"SIZE", "Size", 0.f, 1.f, 0.5f, "%", 0.f, 100.f},
    {"DECAY", "Decay", 0.f, 1.f, 0.5f, "%", 0.f, 100.f},
    {"BRIGHT", "Brightness", 0.f, 1.f, 0.5f, "%", 0.f, 100.f},
    {"DAMP", "Damping", 0.f, 1.f, 0.3f, "%", 0.f, 100.f},
    {"MOD", "Modulation", 0.f, 1.f, 0.3f, "%", 0.f, 100.f},
    {"WIDTH", "Stereo width", 0.f, 1.f, 0.5f, "%", 0.f, 200.f},
    {"LO GAIN", "Low shelf gain", -12.f, 12.f, 0.f, " dB", 0.f, 1.f},
    {"HI GAIN", "High shelf gain", -12.f, 12.f, 0.f, " dB", 0.f, 1.f},
    {"LOW CUT", "Low cut", 0.f, 1.f, 0.f, "%", 0.f, 100.f},
    {"DUCK", "Ducking", 0.f, 1.f, 0.f, "%", 0.f, 100.f},
    {"MIX", "Mix", 0.f, 1.f, 1.f, "%", 0.f, 100.f},
    {"OUT LEVEL", "Output level", 0.f, 2.f, 1.f, "%", 0.f, 100.f},
};

// =====================================================================
//  Module
// =====================================================================
struct PsyReverb2Module : Module
{
    enum ParamId
    {
        // knobs (order matches P2_CTL)
        IN_GAIN_PARAM,
        PREDELAY_PARAM,
        SIZE_PARAM,
        DECAY_PARAM,
        BRIGHT_PARAM,
        DAMP_PARAM,
        MOD_PARAM,
        WIDTH_PARAM,
        LO_GAIN_PARAM,
        HI_GAIN_PARAM,
        LOW_CUT_PARAM,
        DUCK_PARAM,
        MIX_PARAM,
        OUT_LEVEL_PARAM,
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
        AUDIO_L_INPUT,
        AUDIO_R_INPUT,
        FREEZE_GATE_INPUT,
        CLEAR_TRIG_INPUT,
        CV_FIRST,
        CV_LAST = CV_FIRST + P2_NUM_CTRL - 1,
        INPUTS_LEN
    };

    enum OutputId
    {
        AUDIO_L_OUTPUT,
        AUDIO_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        FREEZE_LIGHT,
        CLEAR_LIGHT,
        LIGHTS_LEN
    };

    static_assert(OUT_LEVEL_PARAM == P2_NUM_CTRL - 1, "knob count mismatch");

    enum CVRange
    {
        BIPOLAR_M5_5,
        UNIPOLAR_0_10
    };
    enum VisMode
    {
        VIS_WAVE,
        VIS_BARS,
        VIS_SPECTRUM,
        VIS_COUNT
    };

    // ---- settings (saved with the patch) ----
    CVRange cvRangeMode = BIPOLAR_M5_5;
    int visMode = VIS_SPECTRUM;
    bool eqPre = false; // false: EQ on reverb output, true: EQ on reverb input
    bool softClip = true;
    float loFreq = 200.f;
    float hiFreq = 4000.f;
    bool freezeLatch = false;

    // ---- DSP ----
    psyrv::Engine engine;
    psyrv::Biquad loL, loR, hiL, hiR;
    psyrv::OnePole hpL1, hpL2, hpR1, hpR2;
    dsp::SchmittTrigger clearTrig;

    float dspSr = 0.f;
    bool needSnap = true;
    int ctrlCounter = 0;
    bool prevFreezePressed = false;
    bool prevClearPressed = false;
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
    int scopeHead = 0;
    int scopeCount = 0;
    float scopeBMin = 0.f, scopeBMax = 0.f;
    float peakL = 0.f, peakR = 0.f;
    float fftRing[P2_FFT_RING] = {};
    int fftHead = 0;
    float dispPre = 0.f, dispSize = 0.5f, dispMix = 1.f, dispLo = 0.f, dispHi = 0.f;
    bool dispBypass = false, dispFreeze = false;
    float clipTimer = 0.f;

    // Constructor: configures the 14 knobs and their attenuverters/CV inputs
    // from P2_CTL, then the three buttons/switches (FREEZE, CLEAR, BYPASS),
    // the two LEDs, the four audio/trigger inputs, the stereo outputs, and
    // stereo bypass.
    PsyReverb2Module()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        for (int i = 0; i < P2_NUM_CTRL; ++i)
        {
            const P2CtlSpec &s = P2_CTL[i];
            configParam(i, s.lo, s.hi, s.def, s.label, s.unit, s.dispBase, s.dispMul);
            configParam(ATTN_FIRST + i, -1.f, 1.f, 0.f,
                        std::string(s.label) + " CV amount", "%", 0.f, 100.f);
            configInput(CV_FIRST + i, std::string(s.label) + " CV");
        }

        configButton(FREEZE_PARAM, "Freeze (toggle)");
        configButton(CLEAR_PARAM, "Clear tank");
        configSwitch(BYPASS_PARAM, 0.f, 1.f, 0.f, "Bypass", {"Off", "On"});
        configLight(FREEZE_LIGHT, "Freeze active");
        configLight(CLEAR_LIGHT, "Clear pressed");

        configInput(AUDIO_L_INPUT, "Audio left");
        configInput(AUDIO_R_INPUT, "Audio right");
        configInput(FREEZE_GATE_INPUT, "Freeze gate");
        configInput(CLEAR_TRIG_INPUT, "Clear trigger");
        configOutput(AUDIO_L_OUTPUT, "Audio left");
        configOutput(AUDIO_R_OUTPUT, "Audio right");

        configBypass(AUDIO_L_INPUT, AUDIO_L_OUTPUT);
        configBypass(AUDIO_R_INPUT, AUDIO_R_OUTPUT);
    }

    // Clears the engine, the EQ/low-cut/DC/envelope state, the display
    // buffers and flash/trigger state, and forces a parameter re-snap.
    void resetState()
    {
        engine.initialize();
        loL = psyrv::Biquad();
        loR = psyrv::Biquad();
        hiL = psyrv::Biquad();
        hiR = psyrv::Biquad();
        hpL1.z = hpL2.z = hpR1.z = hpR2.z = 0.f;
        dcXL = dcYL = dcXR = dcYR = 0.f;
        env = 0.f;
        clearFlash = 0.f;
        prevFreezePressed = prevClearPressed = false;
        for (int i = 0; i < P2_SCOPE_N; i++)
        {
            scopeMin[i] = scopeMax[i] = 0.f;
        }
        scopeHead = scopeCount = 0;
        scopeBMin = scopeBMax = 0.f;
        peakL = peakR = 0.f;
        clipTimer = 0.f;
        needSnap = true;
        ctrlCounter = 0;
    }

    // Rack reset hook: releases the freeze latch and resets all DSP/display
    // state.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        freezeLatch = false;
        resetState();
    }

    // (Re)computes all sample-rate-dependent coefficients (DC blocker,
    // smoothing, envelope attack/release) and clears the biquads, high-pass
    // and DC state.
    void setupDSP(float sr)
    {
        dspSr = sr;
        dcR = 1.f - P2_TWO_PI * 10.f / sr;
        kSmooth = 1.f - std::exp(-1.f / (0.010f * sr));
        kAtk = 1.f - std::exp(-1.f / (0.005f * sr));
        kRel = 1.f - std::exp(-1.f / (0.300f * sr));
        loL = psyrv::Biquad();
        loR = psyrv::Biquad();
        hiL = psyrv::Biquad();
        hiR = psyrv::Biquad();
        hpL1.z = hpL2.z = hpR1.z = hpR2.z = 0.f;
        dcXL = dcYL = dcXR = dcYR = 0.f;
        ctrlCounter = 0;
        needSnap = true;
    }

    // Returns the effective value of control `i`: knob + CV × attenuverter,
    // with the CV normalised according to the selected CV range mode.
    float getCtl(int i)
    {
        const P2CtlSpec &s = P2_CTL[i];
        float knob = params[i].getValue();
        float cv = inputs[CV_FIRST + i].getVoltage();
        float attn = params[ATTN_FIRST + i].getValue();

        float cvScaled = (cvRangeMode == UNIPOLAR_0_10) ? cv / 10.f : cv / 5.f;
        cvScaled = clampT(cvScaled, -1.f, 1.f);

        // CV sweeps the whole range of the parameter at full attenuverter
        return clampT(knob + cvScaled * attn * (s.hi - s.lo), s.lo, s.hi);
    }

    // Serialises the CV range, view mode, EQ position, soft-clip flag, shelf
    // frequencies and the freeze latch.
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "cvRangeMode", json_integer(cvRangeMode));
        json_object_set_new(root, "visMode", json_integer(visMode));
        json_object_set_new(root, "eqPre", json_boolean(eqPre));
        json_object_set_new(root, "softClip", json_boolean(softClip));
        json_object_set_new(root, "loFreq", json_real(loFreq));
        json_object_set_new(root, "hiFreq", json_real(hiFreq));
        json_object_set_new(root, "freezeLatch", json_boolean(freezeLatch));
        return root;
    }

    // Restores the persisted settings from a patch, clamping each to its
    // valid range.
    void dataFromJson(json_t *root) override
    {
        if (json_t *j = json_object_get(root, "cvRangeMode"))
            cvRangeMode = (json_integer_value(j) == UNIPOLAR_0_10) ? UNIPOLAR_0_10 : BIPOLAR_M5_5;
        if (json_t *j = json_object_get(root, "visMode"))
            visMode = (int)clampT((float)json_integer_value(j), 0.f, (float)(VIS_COUNT - 1));
        if (json_t *j = json_object_get(root, "eqPre"))
            eqPre = json_boolean_value(j);
        if (json_t *j = json_object_get(root, "softClip"))
            softClip = json_boolean_value(j);
        if (json_t *j = json_object_get(root, "loFreq"))
            loFreq = clampT((float)json_number_value(j), 40.f, 1000.f);
        if (json_t *j = json_object_get(root, "hiFreq"))
            hiFreq = clampT((float)json_number_value(j), 1000.f, 12000.f);
        if (json_t *j = json_object_get(root, "freezeLatch"))
            freezeLatch = json_boolean_value(j);
    }

    // Main audio callback: reads all knobs with their CVs, handles freeze
    // (latch + gate), clear (button + trigger), smoothing of the main
    // parameters, control-rate EQ/low-cut updates, then runs the engine,
    // applies EQ, low cut, ducking, dry/wet, output level, click-free
    // bypass, and soft limiting, and updates the scope/FFT/meter display
    // data.
    void process(const ProcessArgs &args) override
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
        if (frzPressed && !prevFreezePressed)
            freezeLatch = !freezeLatch;
        prevFreezePressed = frzPressed;
        const bool freezeOn = freezeLatch || inputs[FREEZE_GATE_INPUT].getVoltage() >= 2.f;
        lights[FREEZE_LIGHT].setBrightness(freezeOn ? 1.f : 0.f);

        // ---- clear (button + trigger input) ----
        const bool clrPressed = params[CLEAR_PARAM].getValue() > 0.5f;
        bool doClear = (clrPressed && !prevClearPressed);
        prevClearPressed = clrPressed;
        if (clearTrig.process(inputs[CLEAR_TRIG_INPUT].getVoltage(), 0.1f, 2.f))
            doClear = true;
        if (doClear)
        {
            engine.initialize();
            hpL1.z = hpL2.z = hpR1.z = hpR2.z = 0.f;
            clearFlash = 0.2f;
        }
        if (clearFlash > 0.f)
            clearFlash -= args.sampleTime;
        lights[CLEAR_LIGHT].setBrightness(clearFlash > 0.f ? 1.f : 0.f);

        const float tBypass = params[BYPASS_PARAM].getValue() > 0.5f ? 1.f : 0.f;

        if (needSnap)
        {
            sIn = c[IN_GAIN_PARAM];
            sMix = c[MIX_PARAM];
            sOut = c[OUT_LEVEL_PARAM];
            sLo = c[LO_GAIN_PARAM];
            sHi = c[HI_GAIN_PARAM];
            sDuck = c[DUCK_PARAM];
            sBypass = tBypass;
            needSnap = false;
        }
        sIn += (c[IN_GAIN_PARAM] - sIn) * kSmooth;
        sMix += (c[MIX_PARAM] - sMix) * kSmooth;
        sOut += (c[OUT_LEVEL_PARAM] - sOut) * kSmooth;
        sLo += (c[LO_GAIN_PARAM] - sLo) * kSmooth;
        sHi += (c[HI_GAIN_PARAM] - sHi) * kSmooth;
        sDuck += (c[DUCK_PARAM] - sDuck) * kSmooth;
        sBypass += (tBypass - sBypass) * kSmooth;

        // ---- control-rate coefficient updates (every 16 samples) ----
        if ((ctrlCounter++ & 15) == 0)
        {
            loL.setLowShelf(loFreq, sLo, sr);
            loR.copyCoeffs(loL);
            hiL.setHighShelf(hiFreq, sHi, sr);
            hiR.copyCoeffs(hiL);

            float hpF = 20.f * std::pow(50.f, c[LOW_CUT_PARAM]); // 20 Hz .. 1 kHz
            hpL1.setCutoff(hpF, sr);
            hpL2.coeff = hpL1.coeff;
            hpR1.setCutoff(hpF, sr);
            hpR2.coeff = hpR1.coeff;
        }

        // ---- engine parameters ----
        engine.setDecay(c[DECAY_PARAM]);
        engine.setSize(c[SIZE_PARAM]);
        engine.setBright(c[BRIGHT_PARAM]);
        engine.setDamp(c[DAMP_PARAM]);
        engine.setPredelay(c[PREDELAY_PARAM]);
        engine.setMod(c[MOD_PARAM]);
        engine.setWidth(c[WIDTH_PARAM]);
        engine.setMix(1.f); // engine runs fully wet; dry/wet is done here
        engine.setFreeze(freezeOn);

        // ---- inputs (mono normalled to both sides) ----
        const bool lc = inputs[AUDIO_L_INPUT].isConnected();
        const bool rc = inputs[AUDIO_R_INPUT].isConnected();
        float rawL = lc ? inputs[AUDIO_L_INPUT].getVoltageSum() : 0.f;
        float rawR = rc ? inputs[AUDIO_R_INPUT].getVoltageSum() : 0.f;
        if (lc && !rc)
            rawR = rawL;
        if (rc && !lc)
            rawL = rawR;

        const float dryL = rawL * sIn;
        const float dryR = rawR * sIn;

        // DC blocker on the reverb feed
        float fxL = dryL - dcXL + dcR * dcYL;
        dcXL = dryL;
        dcYL = fxL;
        float fxR = dryR - dcXR + dcR * dcYR;
        dcXR = dryR;
        dcYR = fxR;

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
        wetL -= hpL1.process(wetL);
        wetL -= hpL2.process(wetL);
        wetR -= hpR1.process(wetR);
        wetR -= hpR2.process(wetR);

        // ---- ducking ----
        const float duckGain = 1.f - sDuck * clampT(env / 3.f, 0.f, 1.f);
        wetL *= duckGain;
        wetR *= duckGain;

        // ---- dry / wet (same taper as the original engine mix) ----
        const float wetAmt = 1.f - (float)cb(1.0 - sMix);
        float outL = dryL * (1.f - wetAmt) + wetL * wetAmt;
        float outR = dryR * (1.f - wetAmt) + wetR * wetAmt;

        outL *= sOut;
        outR *= sOut;

        // ---- click-free bypass (raw input, tails keep running) ----
        outL = outL * (1.f - sBypass) + rawL * sBypass;
        outR = outR * (1.f - sBypass) + rawR * sBypass;

        // ---- output protection ----
        if (!std::isfinite(outL) || !std::isfinite(outR))
        {
            outL = outR = 0.f;
            engine.initialize();
        }

        if (std::fabs(outL) > 10.f || std::fabs(outR) > 10.f)
            clipTimer = 0.3f;
        else
            clipTimer = std::max(0.f, clipTimer - args.sampleTime);

        if (softClip)
        {
            outL = psyrv::softLimit(outL);
            outR = psyrv::softLimit(outR);
        }

        outputs[AUDIO_L_OUTPUT].setVoltage(outL);
        outputs[AUDIO_R_OUTPUT].setVoltage(outR);

        // ---- display data ----
        dispPre = c[PREDELAY_PARAM] * 100.f;
        dispSize = c[SIZE_PARAM];
        dispMix = c[MIX_PARAM];
        dispLo = c[LO_GAIN_PARAM];
        dispHi = c[HI_GAIN_PARAM];
        dispBypass = tBypass > 0.5f;
        dispFreeze = freezeOn;

        const float decayRate = 1.f - 4.f * args.sampleTime;
        peakL = std::max(std::fabs(outL), peakL * decayRate);
        peakR = std::max(std::fabs(outR), peakR * decayRate);

        const float monoOut = 0.5f * (outL + outR);

        fftRing[fftHead] = monoOut;
        fftHead = (fftHead + 1) & (P2_FFT_RING - 1);

        if (scopeCount == 0)
            scopeBMin = scopeBMax = monoOut;
        else
        {
            scopeBMin = std::min(scopeBMin, monoOut);
            scopeBMax = std::max(scopeBMax, monoOut);
        }
        const int bucket = std::max(1, (int)(sr * 0.0025f));
        if (++scopeCount >= bucket)
        {
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
static const float P2_PANEL_W = RACK_GRID_WIDTH * 26; // 390
static const float P2_PANEL_H = RACK_GRID_HEIGHT;     // 380

// Display (350 x 56): header row, plot, axis row, right-hand info column
static const float P2_DISP_W = 350.f;
static const float P2_DISP_H = 56.f;
static const float P2_DISP_X = 20.f;
static const float P2_DISP_Y = 34.f;
// plot area, display-local
static const float P2_PLOT_X = 8.f, P2_PLOT_Y = 17.f, P2_PLOT_W = 254.f, P2_PLOT_H = 28.f;

// Control cells: 6 columns x 3 rows, then an I/O strip
static const float P2_CELL_W = 59.f;
static const float P2_CELL_H = 74.f;
static const float P2_ROW_TOP[3] = {98.f, 175.f, 252.f};
static const float P2_STRIP_TOP = 329.f;
static const float P2_STRIP_H = 28.f;

// Returns the horizontal centre of control column `col` (0..5).
static float p2ColX(int col) { return 195.f + ((float)col - 2.5f) * 63.f; }
// Returns the knob centre y for control row `row` (0..2).
static float p2KnobY(int row) { return P2_ROW_TOP[row] + 27.f; }
// Returns the CV/attenuverter y for control row `row` (0..2).
static float p2JackY(int row) { return P2_ROW_TOP[row] + 60.f; }
// Returns the vertical centre of the I/O strip.
static float p2StripY() { return P2_STRIP_TOP + P2_STRIP_H / 2.f; }

// =====================================================================
//  UI (anonymous namespace)
// =====================================================================
namespace
{
    namespace ag = agmm;

    // ------------------------------------------------------------
    // Panel background: title, display frame, labelled plates
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, the 18 control-cell plates
    // (14 knobs + FREEZE + CLEAR + BYPASS + CV RANGE), and the six I/O strip
    // plates (outputs tinted).
    struct P2PanelBackground : TransparentWidget
    {
        P2PanelBackground() { box.size = Vec(P2_PANEL_W, P2_PANEL_H); }

        // Draws all static panel art: base panel, bezel, the 18 cell plates
        // with their labels (and CV/ATT captions on the knob cells), and the
        // six I/O strip plates.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, P2_PANEL_W, P2_PANEL_H, "PSYREVERB 2");
            ag::bezel(vg, P2_DISP_X, P2_DISP_Y, P2_DISP_W, P2_DISP_H);

            // Control cells: 14 knobs + FREEZE + CLEAR + BYPASS + CV RANGE (6 columns x 3 rows)
            const char *extra[4] = {"FREEZE", "CLEAR", "BYPASS", "CV RANGE"};
            for (int i = 0; i < 18; ++i)
            {
                int row = i / 6, col = i % 6;
                float cx = p2ColX(col), top = P2_ROW_TOP[row];
                ag::plate(vg, cx, top, P2_CELL_W, P2_CELL_H);
                const char *name = (i < P2_NUM_CTRL) ? P2_CTL[i].name : extra[i - P2_NUM_CTRL];
                ag::text(vg, cx, top + 8.f, name, 8.f, ag::dim(), ag::ALIGN_C, 0.5f);
                if (i < P2_NUM_CTRL)
                {
                    ag::text(vg, cx - 16.f, top + 45.f, "CV", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
                    ag::text(vg, cx + 16.f, top + 45.f, "ATT", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
                }
            }

            // I/O strip: jack or switch on the left, its name to the right
            const char *io[6] = {"L IN", "R IN", "FRZ IN", "CLR IN", "L OUT", "R OUT"};
            for (int i = 0; i < 6; ++i)
            {
                float cx = p2ColX(i);
                ag::plate(vg, cx, P2_STRIP_TOP, P2_CELL_W, P2_STRIP_H, i >= 4);
                ag::text(vg, cx - 2.f, p2StripY(), io[i], 7.f, ag::textCol(), ag::ALIGN_L, 0.f);
            }

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: WAVE / BARS / SPECTRUM (click to cycle)
    // ------------------------------------------------------------
    const char *P2_VIS_NAMES[3] = {"WAVE", "BARS", "SPECTRUM"};

    // Live display: click to cycle the view; draws the WAVE scope, the BARS
    // spectrum, or the continuous SPECTRUM curve, plus L/R meters, header
    // readouts and the right-hand info column. Uses demo data in the module
    // browser.
    struct P2DisplayWidget : Widget
    {
        PsyReverb2Module *module = nullptr;

        psyrv::FFT fft;
        float frame[P2_FFT_N];
        float re[P2_FFT_N], im[P2_FFT_N];
        float binDb[P2_FFT_N / 2]; // smoothed magnitude per bin (dB re 5 V)
        float barLvl[P2_BARS];
        float barPeak[P2_BARS];

        // Constructor: positions the widget and clears all analyzer buffers.
        P2DisplayWidget(PsyReverb2Module *m) : module(m)
        {
            box.pos = Vec(P2_DISP_X, P2_DISP_Y);
            box.size = Vec(P2_DISP_W, P2_DISP_H);
            for (int i = 0; i < P2_FFT_N; ++i)
                frame[i] = re[i] = im[i] = 0.f;
            for (int i = 0; i < P2_FFT_N / 2; ++i)
                binDb[i] = -120.f;
            for (int i = 0; i < P2_BARS; ++i)
                barLvl[i] = barPeak[i] = 0.f;
        }

        // Left click cycles the display view (WAVE -> BARS -> SPECTRUM).
        void onButton(const ButtonEvent &e) override
        {
            if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT)
            {
                module->visMode = (module->visMode + 1) % PsyReverb2Module::VIS_COUNT;
                e.consume(this);
                return;
            }
            Widget::onButton(e);
        }

        // Returns the module's DSP sample rate, falling back to 48 kHz in the
        // module browser.
        float sampleRate() const { return (module && module->dspSr > 0.f) ? module->dspSr : 48000.f; }

        // Refreshes the FFT from the module's ring buffer and updates binDb[]
        // with fast-attack / slow-release smoothing. Uses a plausible demo
        // spectrum in the module browser.
        void updateSpectrum()
        {
            const int half = P2_FFT_N / 2;
            if (module)
            {
                int start = (module->fftHead - P2_FFT_N + P2_FFT_RING) & (P2_FFT_RING - 1);
                for (int i = 0; i < P2_FFT_N; ++i)
                    frame[i] = module->fftRing[(start + i) & (P2_FFT_RING - 1)];
                fft.forward(frame, re, im);
                for (int k = 1; k < half; ++k)
                {
                    float mag = std::sqrt(re[k] * re[k] + im[k] * im[k]);
                    float amp = 4.f * mag / (float)P2_FFT_N; // Hann-window corrected
                    float db = 20.f * std::log10(amp / 5.f + 1e-7f);
                    db = clampT(db, -120.f, 12.f);
                    float s = binDb[k];
                    binDb[k] = (db > s) ? s + (db - s) * 0.6f : s + (db - s) * 0.14f;
                }
            }
            else
            {
                // preview data for the module browser
                float binHz = 48000.f / P2_FFT_N;
                for (int k = 1; k < half; ++k)
                {
                    float f = k * binHz;
                    binDb[k] = -14.f - 17.f * std::log10(1.f + f / 150.f) - 5.f * std::fabs(std::sin(f * 0.0017f));
                }
            }
        }

        // Returns the level in dB of a frequency range [fLo, fHi], using
        // interpolation for narrow ranges and the max bin for wide ones.
        float levelAt(float fLo, float fHi, float binHz) const
        {
            const int maxB = P2_FFT_N / 2 - 2;
            float b0 = fLo / binHz, b1 = fHi / binHz;
            if (b1 - b0 < 1.f)
            {
                float cc = clampT(0.5f * (b0 + b1), 1.f, (float)maxB);
                int i = (int)cc;
                float frac = cc - (float)i;
                return binDb[i] * (1.f - frac) + binDb[i + 1] * frac;
            }
            int i0 = std::max(1, (int)std::ceil(b0));
            int i1 = std::min(maxB, (int)std::floor(b1));
            if (i1 < i0)
                i1 = i0;
            float m = -200.f;
            for (int i = i0; i <= i1; ++i)
                m = std::max(m, binDb[i]);
            return m;
        }

        // Normalises a frequency band's level to 0..1 using the display tilt
        // and dB floor.
        float bandNorm(float fLo, float fHi, float binHz) const
        {
            float fc = std::sqrt(fLo * fHi);
            float db = levelAt(fLo, fHi, binHz) + P2_TILT * std::log2(fc / 1000.f);
            return clampT((db - P2_DB_FLOOR) / (-P2_DB_FLOOR), 0.f, 1.f);
        }

        // ---- views (plot-local coordinates, 0..sw x 0..P2_PLOT_H) ----
        // Draws the WAVE view: min/max waveform columns from the scope ring
        // buffer (or a demo shape), around a centre line.
        void drawWave(NVGcontext *vg, float sw)
        {
            const float mid = P2_PLOT_H / 2.f;
            const float amp = mid - 2.f;
            const float colW = sw / (float)P2_SCOPE_N;

            nvgBeginPath(vg);
            nvgMoveTo(vg, 0, mid + 0.5f);
            nvgLineTo(vg, sw, mid + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 30));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            for (int i = 0; i < P2_SCOPE_N; ++i)
            {
                float vmin, vmax;
                if (module)
                {
                    int idx = (module->scopeHead + i) % P2_SCOPE_N;
                    vmin = module->scopeMin[idx];
                    vmax = module->scopeMax[idx];
                }
                else
                {
                    float t = (float)i / P2_SCOPE_N;
                    float e = 3.5f * std::exp(-3.f * t) * (0.6f + 0.4f * std::sin(t * 90.f));
                    vmax = e;
                    vmin = -e;
                }
                float y0 = mid - clampT(vmax / 5.f, -1.f, 1.f) * amp;
                float y1 = mid - clampT(vmin / 5.f, -1.f, 1.f) * amp;
                float h = std::max(y1 - y0, 1.f);
                nvgRect(vg, i * colW, y0, std::max(colW - 0.4f, 1.f), h);
            }
            nvgFillColor(vg, ag::bright());
            nvgFill(vg);
        }

        // Draws the vertical frequency grid lines (100 Hz, 1 kHz, 10 kHz) for
        // the BARS and SPECTRUM views.
        void drawFreqGrid(NVGcontext *vg, float sw)
        {
            const float fs[3] = {100.f, 1000.f, 10000.f};
            const float lr = std::log(P2_FMAX / P2_FMIN);
            nvgBeginPath(vg);
            for (int i = 0; i < 3; ++i)
            {
                float x = std::floor(sw * std::log(fs[i] / P2_FMIN) / lr) + 0.5f;
                nvgMoveTo(vg, x, 0);
                nvgLineTo(vg, x, P2_PLOT_H);
            }
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 28));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // frequency labels live in their own row under the plot (display-local coordinates)
        // Draws the 100 / 1K / 10K labels on the axis row below the plot.
        void drawFreqLabels(NVGcontext *vg, float sw)
        {
            const float fs[3] = {100.f, 1000.f, 10000.f};
            const char *ls[3] = {"100", "1K", "10K"};
            const float lr = std::log(P2_FMAX / P2_FMIN);
            for (int i = 0; i < 3; ++i)
            {
                float x = P2_PLOT_X + sw * std::log(fs[i] / P2_FMIN) / lr;
                ag::text(vg, x, P2_PLOT_Y + P2_PLOT_H + 6.f, ls[i], 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            }
        }

        // Draws the BARS view: frequency grid, 24 log-spaced bars with
        // smooth attack / slow release, and peak-hold caps.
        void drawBars(NVGcontext *vg, float sw)
        {
            const float binHz = sampleRate() / P2_FFT_N;
            const float ratio = P2_FMAX / P2_FMIN;
            const float slot = sw / (float)P2_BARS;
            const float barW = slot - 2.f;
            const float H = P2_PLOT_H;

            drawFreqGrid(vg, sw);

            nvgBeginPath(vg);
            for (int b = 0; b < P2_BARS; ++b)
            {
                float fLo = P2_FMIN * std::pow(ratio, (float)b / P2_BARS);
                float fHi = P2_FMIN * std::pow(ratio, (float)(b + 1) / P2_BARS);
                float n = bandNorm(fLo, fHi, binHz);
                barLvl[b] = (n > barLvl[b]) ? n : barLvl[b] * 0.86f + n * 0.14f;
                barPeak[b] = std::max(barLvl[b], barPeak[b] - 0.010f);
                float h = barLvl[b] * (H - 3.f);
                if (h > 0.5f)
                    nvgRect(vg, b * slot + 1.f, H - 1.f - h, barW, h);
            }
            nvgFillColor(vg, ag::accent());
            nvgFill(vg);

            nvgBeginPath(vg);
            for (int b = 0; b < P2_BARS; ++b)
            {
                float y = H - 1.f - barPeak[b] * (H - 3.f);
                if (barPeak[b] > 0.02f)
                    nvgRect(vg, b * slot + 1.f, y - 1.f, barW, 1.5f);
            }
            nvgFillColor(vg, nvgRGB(0xd8, 0xf8, 0xff));
            nvgFill(vg);
        }

        // Draws the SPECTRUM view: frequency grid, a filled area and a smooth
        // outline built from P2_SPEC_COLS log-spaced bands.
        void drawSpectrum(NVGcontext *vg, float sw)
        {
            const float binHz = sampleRate() / P2_FFT_N;
            const float ratio = P2_FMAX / P2_FMIN;
            const float H = P2_PLOT_H;
            float xs[P2_SPEC_COLS], ys[P2_SPEC_COLS];

            drawFreqGrid(vg, sw);

            for (int c = 0; c < P2_SPEC_COLS; ++c)
            {
                float fLo = P2_FMIN * std::pow(ratio, (float)c / P2_SPEC_COLS);
                float fHi = P2_FMIN * std::pow(ratio, (float)(c + 1) / P2_SPEC_COLS);
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
            nvgFillColor(vg, ag::withA(ag::accent(), 0.25f));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 0.f, ys[0]);
            for (int c = 0; c < P2_SPEC_COLS; ++c)
                nvgLineTo(vg, xs[c], ys[c]);
            nvgLineTo(vg, sw, ys[P2_SPEC_COLS - 1]);
            nvgStrokeColor(vg, ag::bright());
            nvgStrokeWidth(vg, 1.1f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);
        }

        // ---- static background ----
        // Draws the display screen, the header divider, the plot frame and
        // the L/R meter labels.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, P2_DISP_W, P2_DISP_H);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, 14.5f);
            nvgLineTo(vg, P2_DISP_W - 8.f, 14.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, P2_PLOT_X, P2_PLOT_Y, P2_PLOT_W, P2_PLOT_H);

            ag::text(vg, 270.f, P2_PLOT_Y + P2_PLOT_H + 6.f, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, 276.f, P2_PLOT_Y + P2_PLOT_H + 6.f, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            Widget::draw(args);
        }

        // Layer-1 overlay: dispatches to the wave/bars/spectrum draw routine,
        // clips it to the plot, then draws the frequency labels, the L/R
        // meters, the header readouts (predelay, size, lo/hi gain, mix) and
        // the right-hand info column (view name and FRZ/BYP/CLIP flags).
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float sw = P2_PLOT_W;
            int mode = module ? module->visMode : (int)PsyReverb2Module::VIS_SPECTRUM;
            mode = (int)clampT((float)mode, 0.f, (float)(PsyReverb2Module::VIS_COUNT - 1));

            if (mode != PsyReverb2Module::VIS_WAVE)
                updateSpectrum();

            nvgSave(vg);
            nvgTranslate(vg, P2_PLOT_X, P2_PLOT_Y);
            nvgScissor(vg, 0, 0, sw, P2_PLOT_H);
            if (mode == PsyReverb2Module::VIS_WAVE)
                drawWave(vg, sw);
            else if (mode == PsyReverb2Module::VIS_BARS)
                drawBars(vg, sw);
            else
                drawSpectrum(vg, sw);
            nvgRestore(vg);

            if (mode != PsyReverb2Module::VIS_WAVE)
                drawFreqLabels(vg, sw);

            // Level meters (L / R)
            for (int ch = 0; ch < 2; ++ch)
            {
                float peak = module ? (ch == 0 ? module->peakL : module->peakR) : (ch == 0 ? 3.f : 2.4f);
                ag::meter(vg, 268.f + (float)ch * 6.f, P2_PLOT_Y, 4.f, P2_PLOT_H, std::min(peak / 10.f, 1.f));
            }

            // Header readouts
            char buf[48];
            float pre = module ? module->dispPre : 0.f;
            float size = module ? module->dispSize : 0.5f;
            float mix = module ? module->dispMix : 1.f;
            float lo = module ? module->dispLo : 0.f;
            float hi = module ? module->dispHi : 0.f;

            snprintf(buf, sizeof(buf), "PRE %.0f MS  SIZE %.0f", pre, size * 100.f);
            ag::text(vg, 8.f, 7.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "LO %+.1f  HI %+.1f", lo, hi);
            ag::text(vg, P2_DISP_W / 2.f + 40.f, 7.f, buf, 7.f, ag::accent(), ag::ALIGN_C, 0.5f);
            snprintf(buf, sizeof(buf), "MIX %.0f%%", mix * 100.f);
            ag::text(vg, P2_DISP_W - 8.f, 7.f, buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);

            // Right-hand info column: view name, status flags
            ag::text(vg, 313.f, 24.f, P2_VIS_NAMES[mode], 7.f, ag::violet(), ag::ALIGN_C, 0.5f);
            if (module)
            {
                char st[32] = "";
                if (module->dispFreeze)
                    strcat(st, "FRZ ");
                if (module->dispBypass)
                    strcat(st, "BYP ");
                if (module->clipTimer > 0.f)
                    strcat(st, "CLIP");
                if (st[0])
                    ag::text(vg, 313.f, 38.f, st, 7.f,
                             module->clipTimer > 0.f ? nvgRGB(0xff, 0x50, 0x60) : nvgRGB(0xff, 0xc8, 0x50),
                             ag::ALIGN_C, 0.f);
            }

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // CV range cell (click to toggle). The name is drawn by the panel plate.
    // ------------------------------------------------------------
    // Clickable cell that toggles the CV range mode (bipolar/unipolar) and
    // shows the current range and a small range-bar marker.
    struct P2CvRangeWidget : Widget
    {
        PsyReverb2Module *module = nullptr;

        // Constructor: positions the widget over its grid cell.
        P2CvRangeWidget(PsyReverb2Module *m, float cx, float top) : module(m)
        {
            box.pos = Vec(cx - P2_CELL_W / 2.f, top);
            box.size = Vec(P2_CELL_W, P2_CELL_H);
        }

        // Toggles the CV range mode on a left click.
        void onButton(const ButtonEvent &e) override
        {
            if (module && e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT)
            {
                module->cvRangeMode = (module->cvRangeMode == PsyReverb2Module::BIPOLAR_M5_5)
                                          ? PsyReverb2Module::UNIPOLAR_0_10
                                          : PsyReverb2Module::BIPOLAR_M5_5;
                e.consume(this);
                return;
            }
            Widget::onButton(e);
        }

        // Layer-1 overlay: draws the current range value box, a range-bar
        // marker (centre for bipolar, left for unipolar) and the "CLICK"
        // hint.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            bool bipolar = module ? (module->cvRangeMode == PsyReverb2Module::BIPOLAR_M5_5) : true;
            const float cx = box.size.x / 2.f;

            const float sx = 5.f, sy = 17.f, sw = box.size.x - 10.f, sh = 24.f;
            nvgBeginPath(vg);
            nvgRoundedRect(vg, sx, sy, sw, sh, 3.f);
            nvgFillColor(vg, nvgRGB(0x05, 0x08, 0x0c));
            nvgFill(vg);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 90));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            ag::text(vg, cx, sy + sh / 2.f, bipolar ? "-5/+5V" : "0/+10V", 9.f, ag::accent(), ag::ALIGN_C, 0.5f);

            const float by = 49.f;
            nvgBeginPath(vg);
            nvgRect(vg, sx + 4.f, by, sw - 8.f, 2.f);
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 25));
            nvgFill(vg);
            nvgBeginPath(vg);
            if (bipolar)
                nvgRect(vg, cx - 12.f, by, 24.f, 2.f);
            else
                nvgRect(vg, sx + 4.f, by, 24.f, 2.f);
            nvgFillColor(vg, ag::violet());
            nvgFill(vg);

            ag::text(vg, cx, 63.f, "CLICK", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // Context-menu helpers
    // ------------------------------------------------------------
    // A generic checkmark menu item driven by lambdas: `isOn` supplies the
    // current state (for the checkmark), `action` runs on click.
    struct P2ChoiceItem : MenuItem
    {
        std::function<bool()> isOn;
        std::function<void()> action;
        // Runs the item's action.
        void onAction(const ActionEvent &e) override { action(); }
        // Updates the checkmark from isOn() (if provided) and forwards to the
        // base step.
        void step() override
        {
            rightText = CHECKMARK(isOn && isOn());
            MenuItem::step();
        }
    };

    // Creates a P2ChoiceItem with the given label, state query (may be null
    // for an action-only item) and action, and adds it to `menu`.
    void addChoice(Menu *menu, const std::string &label,
                   std::function<bool()> isOn, std::function<void()> action)
    {
        P2ChoiceItem *item = createMenuItem<P2ChoiceItem>(label);
        item->isOn = isOn;
        item->action = action;
        menu->addChild(item);
    }

} // namespace

// =====================================================================
//  Widget: adds the static panel, screws, the live display, the 14
//  knob/CV/attenuverter rows, the FREEZE/CLEAR/BYPASS controls and lights,
//  the CV-range cell, and the six I/O strip jacks.
// =====================================================================
struct PsyReverb2Widget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, display, the 14 control
    // rows, the row-3 buttons/switch/CV-range cell, and the six I/O jacks.
    PsyReverb2Widget(PsyReverb2Module *module)
    {
        setModule(module);
        box.size = Vec(P2_PANEL_W, P2_PANEL_H);

        addChild(new P2PanelBackground());

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new P2DisplayWidget(module));

        // 14 controls: knob on top, CV jack (left) and attenuverter (right) below
        for (int i = 0; i < P2_NUM_CTRL; ++i)
        {
            int row = i / 6, col = i % 6;
            float cx = p2ColX(col);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, p2KnobY(row)), module, i));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 16.f, p2JackY(row)), module,
                                                         PsyReverb2Module::CV_FIRST + i));
            addParam(createParamCentered<Trimpot>(Vec(cx + 16.f, p2JackY(row)), module,
                                                  PsyReverb2Module::ATTN_FIRST + i));
        }

        // Row 3: FREEZE (col 2), CLEAR (col 3), BYPASS (col 4), CV range (col 5)
        const float t3 = P2_ROW_TOP[2];
        addParam(createParamCentered<TL1105>(Vec(p2ColX(2), t3 + 32.f), module,
                                             PsyReverb2Module::FREEZE_PARAM));
        addChild(createLightCentered<SmallLight<GreenLight>>(Vec(p2ColX(2), t3 + 54.f), module,
                                                             PsyReverb2Module::FREEZE_LIGHT));
        addParam(createParamCentered<TL1105>(Vec(p2ColX(3), t3 + 32.f), module,
                                             PsyReverb2Module::CLEAR_PARAM));
        addChild(createLightCentered<SmallLight<RedLight>>(Vec(p2ColX(3), t3 + 54.f), module,
                                                           PsyReverb2Module::CLEAR_LIGHT));
        addParam(createParamCentered<CKSS>(Vec(p2ColX(4), t3 + 42.f), module,
                                           PsyReverb2Module::BYPASS_PARAM));
        addChild(new P2CvRangeWidget(module, p2ColX(5), t3));

        // I/O strip: jack left of its label
        const float y = p2StripY();
        addInput(createInputCentered<DarkPJ301MPort>(Vec(p2ColX(0) - 16.f, y), module, PsyReverb2Module::AUDIO_L_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(p2ColX(1) - 16.f, y), module, PsyReverb2Module::AUDIO_R_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(p2ColX(2) - 16.f, y), module, PsyReverb2Module::FREEZE_GATE_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(p2ColX(3) - 16.f, y), module, PsyReverb2Module::CLEAR_TRIG_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(p2ColX(4) - 16.f, y), module, PsyReverb2Module::AUDIO_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(p2ColX(5) - 16.f, y), module, PsyReverb2Module::AUDIO_R_OUTPUT));
    }

    // Adds the right-click menu: CV range, display mode, EQ position, low
    // and high shelf frequencies, the output soft limiter toggle, and a
    // "clear reverb tank" action.
    void appendContextMenu(Menu *menu) override
    {
        PsyReverb2Module *m = dynamic_cast<PsyReverb2Module *>(this->module);
        assert(m);

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("CV Range"));
        addChoice(menu, "-5V to +5V (bipolar)", [m]()
                  { return m->cvRangeMode == PsyReverb2Module::BIPOLAR_M5_5; }, [m]()
                  { m->cvRangeMode = PsyReverb2Module::BIPOLAR_M5_5; });
        addChoice(menu, "0V to +10V (unipolar)", [m]()
                  { return m->cvRangeMode == PsyReverb2Module::UNIPOLAR_0_10; }, [m]()
                  { m->cvRangeMode = PsyReverb2Module::UNIPOLAR_0_10; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Display"));
        for (int v = 0; v < PsyReverb2Module::VIS_COUNT; ++v)
            addChoice(menu, P2_VIS_NAMES[v], [m, v]()
                      { return m->visMode == v; }, [m, v]()
                      { m->visMode = v; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("EQ Position"));
        addChoice(menu, "After reverb (wet output)", [m]()
                  { return !m->eqPre; }, [m]()
                  { m->eqPre = false; });
        addChoice(menu, "Before reverb (shapes the tank input)", [m]()
                  { return m->eqPre; }, [m]()
                  { m->eqPre = true; });

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Low Shelf Frequency"));
        const float loF[4] = {100.f, 200.f, 300.f, 500.f};
        for (int i = 0; i < 4; ++i)
        {
            float f = loF[i];
            addChoice(menu, std::to_string((int)f) + " Hz", [m, f]()
                      { return m->loFreq == f; }, [m, f]()
                      { m->loFreq = f; });
        }

        menu->addChild(createMenuLabel("High Shelf Frequency"));
        const float hiF[4] = {2500.f, 4000.f, 6000.f, 8000.f};
        for (int i = 0; i < 4; ++i)
        {
            float f = hiF[i];
            addChoice(menu, std::to_string((int)f) + " Hz", [m, f]()
                      { return m->hiFreq == f; }, [m, f]()
                      { m->hiFreq = f; });
        }

        menu->addChild(new MenuSeparator);
        addChoice(menu, "Output soft limiter (10 V)", [m]()
                  { return m->softClip; }, [m]()
                  { m->softClip = !m->softClip; });
        addChoice(menu, "Clear reverb tank",
                  nullptr,
                  [m]()
                  { m->engine.initialize(); });
    }
};

// =====================================================================
//  Model
// =====================================================================
Model *modelPsyReverb2 = createModel<PsyReverb2Module, PsyReverb2Widget>("PsyReverb2");