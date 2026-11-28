#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>
#include <map>
#include <string>
#include <regex>
#include <algorithm>
#include <random>
#include <initializer_list>
#include "dsp/minblep.hpp"
#include "Knob_arcs.hpp"

// ============================================================================
// Stagger2  (32 HP)
// ============================================================================
//
// OVERVIEW
// --------
// Stagger2 is an expanded Stagger VCO: a stereo "chaos drift" oscillator with
// 40 selectable waveforms, three Plaits-style macro controls (HARMONICS,
// TIMBRE, MORPH, each with its own CV input + attenuverter), a per-voice
// modulated stereo delay, an 8-line FDN reverb, a trigger/gate envelope, a
// master gain stage and a built-in compressor.
//
// WAVEFORMS (WAVE knob, 0..39, snapped; WAVE CV sweeps the list)
// --------------------------------------------------------------
// The original four waves (0..3) are unchanged (minBLEP saw / square, PW knob)
// and ignore the three macro knobs.
//
//   0 Sine            1 Triangle        2 Saw             3 Square (PW)
//
// Band-limited additive stacks (harmonics are cut off at Nyquist).
//   H = number of harmonics, T = spectral slope (1/k -> 1/k^2),
//   M = phase blend of every partial (sine -> cosine)
//   4 Saw Stack       all harmonics
//   5 Square Stack    odd harmonics
//   6 Even Stack      even harmonics
//   7 Prime Stack     1 + prime-numbered harmonics
//   8 Fibonacci       harmonics 1 2 3 5 8 13 21 34
//   9 Octave Stack    harmonics 1 2 4 8 16 32
//  10 Organ           drawbars 1 2 3 4 5 6 8
//  11 Third Stack     harmonics 1 4 7 10 13 ...
//  12 Formant         H = peak position, T = peak width, M = phase blend
//  13 Dual Formant    H = formant 1, T = formant 2 spacing, M = phase blend
//
// Waveshapers / modulation (2x oversampled)
//  14 Wavefold        H = fold gain,    T = bias,       M = second fold stage
//  15 Tri Fold        H = fold gain,    T = bias,       M = smoothing
//  16 Saw Fold        H = fold gain,    T = bias,       M = saw -> sine input
//  17 Hard Clip       H = drive,        T = bias,       M = clip -> tanh
//  18 Soft Sat        H = drive,        T = bias,       M = sine -> triangle
//  19 Stair Saw       H = steps,        T = smoothing,  M = curve
//  20 Stair Tri       H = steps,        T = smoothing,  M = skew
//  21 Decimated       H = sample points,T = smoothing,  M = bit depth
//  22 FM 1:N          H = ratio 1..8,   T = index,      M = sine -> tri modulator
//  23 FM Nested       H = ratio 1..6,   T = index 1,    M = index 2
//
// Sync / phase distortion / ring mod
//  24 Sync Saw        H = sync ratio,   T = window,     M = saw -> tri
//  25 Sync Square     H = sync ratio,   T = window,     M = pulse -> sine (PW knob = width)
//  26 Sync Sine       H = sync ratio,   T = window,     M = sub level
//  27 PD Warp         H = warp 1,       T = warp 2,     M = sine -> tri
//  28 PD Resonant     H = resonance,    T = window pow, M = saw -> tri window
//  29 Ring Mod        H = ratio 1..8,   T = ring -> AM, M = sine -> tri modulator
//
// Classic shapes
//  30 Rect Sine       H = power,        T = dry sine,   M = half -> full rectify
//  31 Parabola        H = exponent,     T = skew,       M = parabola -> sine
//  32 Exp Saw         H = curvature,    T = symmetric,  M = concave -> convex
//  33 Trapezoid       H = slope,        T = rounding,   M = skew
//  34 Narrow Pulse    H = width,        T = saw mix,    M = pulse -> ramp
//  35 Twin Pulse      H = width,        T = spacing,    M = 2nd pulse level
//  36 Gaussian        H = width,        T = centre,     M = bell -> mexican hat
//  37 Sinc            H = lobes,        T = centre,     M = window
//  38 Circle          H = squareness,   T = skew,       M = circle -> sine
//  39 Random Steps    H = steps,        T = smoothing,  M = re-roll per cycle
//
// Waves 4..39 are DC-blocked (5 Hz one-pole high-pass).
//
// Controls:
//   FREQ / WAVE / HARM / TIMBRE / MORPH / CHAOS / PW   (each with CV + ATT)
//   V/OCT, RESET, TRIG IN, ENV DECAY, TRIG/GATE, MULT, CV RANGE
//   DLY TIME / FB / MOD / MIX, PING-PONG, RV SIZE / TIME / MIX
//   GAIN, COMP
//
// Outputs:
//   VCO OUT      Polyphonic VCO + FX (one voice per input channel)
//   FX L / FX R  Mono-summed stereo FX output
//
// File structure:
//   1. Layout constants and shared display settings.
//   2. Waveform tables and shape functions.
//   3. Custom ParamQuantity subclasses.
//   4. DSP building blocks: S2ModDelay, S2Comp, S2Fdn.
//   5. Stagger2 module.
//   6. Anonymous-namespace UI helpers: S2Panel, S2Display.
//   7. Stagger2Widget and model registration.
// ============================================================================

// ---- panel layout (32 HP = 480 px wide) ----
static const float S2_W = RACK_GRID_WIDTH * 32; // 480
static const float S2_H = RACK_GRID_HEIGHT;     // 380

// Display (400 x 80): header row, plots, footer row. Bezel adds 4 px around it.
static const float S2_DX = 40.f, S2_DY = 26.f, S2_DW = 400.f, S2_DH = 80.f;
static const float S2_HEAD_LINE = 15.5f;                             // hairline under the header row
static const float S2_PLOT_Y = 20.f, S2_PLOT_H = 40.f;               // all plots share this band
static const float S2_FOOT_LINE = 64.5f;                             // hairline above the footer row
static const float S2_FOOT_TXT = S2_DH - 8.f;                        // footer text centre line
static const float S2_SC_X = 8.f, S2_SC_W = 180.f;                   // waveform scope
static const float S2_MT_X = 204.f, S2_MT_DX = 16.f, S2_MT_W = 10.f; // ENV, L, R meters
static const float S2_RO_X = 262.f, S2_RO_W = 130.f;                 // readout box

// Cells are 60 px wide on a 65 px pitch (seven columns, 15 px side margins)
static const float S2_CW = 60.f;
static const int S2_COLS = 7;
// Returns the horizontal centre of control column i (0..6).
static float s2Col(int i) { return 45.f + 65.f * (float)i; }

// Row 1: knob at +30, "CV" / "ATT" captions at +50, jack and trimpot at +65
static const float S2_T1 = 114.f, S2_H1 = 80.f;
static const float S2_R1_KNOB = S2_T1 + 30.f, S2_R1_CAP = S2_T1 + 50.f, S2_R1_JACK = S2_T1 + 65.f;
// Rows 2-4: control centred 30 px below the cell top
static const float S2_T2 = 200.f, S2_T3 = 254.f, S2_T4 = 308.f, S2_HK = 50.f;
static const float S2_CTL_DY = 30.f;

// -----------------------------------------------------------------------------
// Waveform tables and shape functions
// -----------------------------------------------------------------------------
static const int S2_NUM_WAVES = 40;
static const int S2_FIRST_ADD = 4; // first additive wave
static const int S2_LAST_ADD = 13; // last additive wave
static const int S2_MAX_HARM = 40; // most partials an additive wave uses
static constexpr float S2_2PI = 6.28318530718f;

static const char *const S2_WAVE_NAMES[S2_NUM_WAVES] = {
    "Sine", "Triangle", "Saw", "Square",
    "Saw Stack", "Square Stack", "Even Stack", "Prime Stack", "Fibonacci", "Octave Stack",
    "Organ", "Third Stack", "Formant", "Dual Formant",
    "Wavefold", "Tri Fold", "Saw Fold", "Hard Clip", "Soft Sat", "Stair Saw",
    "Stair Tri", "Decimated", "FM 1:N", "FM Nested",
    "Sync Saw", "Sync Square", "Sync Sine", "PD Warp", "PD Resonant", "Ring Mod",
    "Rect Sine", "Parabola", "Exp Saw", "Trapezoid", "Narrow Pulse", "Twin Pulse",
    "Gaussian", "Sinc", "Circle", "Random Steps"};

static inline float s2Clamp(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static inline float s2Lerp(float a, float b, float t) { return a + (b - a) * t; }
static inline float s2Frac(float x) { return x - std::floor(x); }
static inline float s2Tri(float p) { return (p < 0.5f) ? (4.f * p - 1.f) : (3.f - 4.f * p); }
static inline float s2Saw(float p) { return 1.f - 2.f * p; }

// Transparent soft limiter: linear below 60% of `limit`, then smoothly
// saturates towards +-limit. Used to keep feedback loops and outputs bounded.
static inline float s2SoftLimit(float x, float limit)
{
    const float knee = 0.6f * limit;
    float a = std::fabs(x);
    if (a <= knee)
        return x;
    float y = knee + (limit - knee) * std::tanh((a - knee) / (limit - knee));
    return x < 0.f ? -y : y;
}

// Triangle wavefolder: identity on [-1, 1], reflects outside that range.
static inline float s2Fold(float x)
{
    float x4 = x * 0.25f + 0.25f;
    x4 -= std::floor(x4);
    return 1.f - std::fabs(4.f * x4 - 2.f);
}

// Piecewise-linear phase warp: moves the phase midpoint to c.
static inline float s2Warp(float p, float c)
{
    c = s2Clamp(c, 0.02f, 0.98f);
    return (p < c) ? 0.5f * p / c : 0.5f + 0.5f * (p - c) / (1.f - c);
}

// Exponential curve 0..1 -> 0..1, concave (m = 0) to convex (m = 1).
static inline float s2ExpCurve(float x, float k, float m)
{
    float cc = (1.f - std::exp(-k * x)) / (1.f - std::exp(-k));
    float cv = (std::exp(k * x) - 1.f) / (std::exp(k) - 1.f);
    return s2Lerp(cc, cv, m);
}

// Lorentzian peak used by the formant waves.
static inline float s2Lor(float x, float c, float w)
{
    float d = (x - c) / w;
    return 1.f / (1.f + d * d);
}

// Small integer hash -> [-1, 1)
static inline float s2Hash(uint32_t n)
{
    n = n * 747796405u + 2891336453u;
    n = ((n >> ((n >> 28u) + 4u)) ^ n) * 277803737u;
    n = (n >> 22u) ^ n;
    return (float)(n & 0xFFFFFFu) / (float)0x800000 - 1.f;
}

// Step value of the Random Steps wave; m re-rolls the pattern every cycle.
static inline float s2Step(uint32_t i, uint32_t cyc, float m)
{
    float a = s2Hash(i * 2654435761u + 17u);
    float b = s2Hash(i * 2654435761u + cyc * 40503u + 99991u);
    return s2Lerp(a, b, m);
}

// Bit masks selecting which harmonics each additive wave contains.
static uint64_t s2MaskOf(std::initializer_list<int> l)
{
    uint64_t m = 0;
    for (int k : l)
        m |= (1ULL << k);
    return m;
}
static uint64_t s2MaskStep(int start, int step)
{
    uint64_t m = 0;
    for (int k = start; k < 64; k += step)
        m |= (1ULL << k);
    return m;
}
static const uint64_t S2_MASK_ODD = s2MaskStep(1, 2);
static const uint64_t S2_MASK_EVEN = s2MaskStep(2, 2);
static const uint64_t S2_MASK_THIRD = s2MaskStep(1, 3);
static const uint64_t S2_MASK_PRIME = s2MaskOf({1, 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37});
static const uint64_t S2_MASK_FIB = s2MaskOf({1, 2, 3, 5, 8, 13, 21, 34});
static const uint64_t S2_MASK_OCT = s2MaskOf({1, 2, 4, 8, 16, 32});
static const uint64_t S2_MASK_ORGAN = s2MaskOf({1, 2, 3, 4, 5, 6, 8});

struct S2Tables
{
    float invSqrt[S2_MAX_HARM + 2];
    S2Tables()
    {
        invSqrt[0] = 0.f;
        for (int k = 1; k < S2_MAX_HARM + 2; ++k)
            invSqrt[k] = 1.f / std::sqrt((float)k);
    }
};
static const S2Tables s2Tab;

// Band-limited additive waves (4..13). nMax is the highest partial that stays
// below Nyquist. Partials are generated with a rotation recurrence, so the
// cost is one multiply-add per harmonic instead of one sin() per harmonic.
static float s2Additive(int wave, float p, float h, float t, float m, int nMax)
{
    int kMax;
    if (wave == 12 || wave == 13)
        kMax = S2_MAX_HARM;
    else
        kMax = 1 + (int)(h * (float)(S2_MAX_HARM - 1) + 0.5f);
    kMax = std::min(kMax, std::max(1, nMax));
    kMax = std::min(kMax, S2_MAX_HARM);

    uint64_t mask = ~0ULL;
    switch (wave)
    {
    case 5:
        mask = S2_MASK_ODD;
        break;
    case 6:
        mask = S2_MASK_EVEN;
        break;
    case 7:
        mask = S2_MASK_PRIME;
        break;
    case 8:
        mask = S2_MASK_FIB;
        break;
    case 9:
        mask = S2_MASK_OCT;
        break;
    case 10:
        mask = S2_MASK_ORGAN;
        break;
    case 11:
        mask = S2_MASK_THIRD;
        break;
    default:
        break;
    }

    const float th = S2_2PI * p;
    const float s1 = std::sin(th), c1 = std::cos(th);
    float sk = s1, ck = c1; // sin(k*th), cos(k*th) for k = 1
    const float psi = m * (float)M_PI * 0.5f;
    const float cp = std::cos(psi), sp = std::sin(psi);

    // formant parameters
    const float f1 = (wave == 12) ? 1.f + h * 14.f : 1.f + h * 10.f;
    const float f2 = f1 * (1.6f + t * 2.4f);
    const float w1 = (wave == 12) ? 0.6f + t * 3.4f : 0.8f;

    float sum = 0.f, pwr = 0.f;
    for (int k = 1; k <= kMax; ++k)
    {
        if ((mask >> k) & 1ULL)
        {
            float a;
            if (wave == 12)
                a = s2Lor((float)k, f1, w1) * s2Tab.invSqrt[k];
            else if (wave == 13)
                a = (s2Lor((float)k, f1, w1) + 0.7f * s2Lor((float)k, f2, 1.4f)) * s2Tab.invSqrt[k];
            else
            {
                float fk = (float)k;
                a = (1.f - t) / fk + t / (fk * fk);
            }
            sum += a * (cp * sk + sp * ck);
            pwr += a * a;
        }
        float sn = sk * c1 + ck * s1;
        ck = ck * c1 - sk * s1;
        sk = sn;
    }
    if (pwr < 1e-12f)
        return 0.f;
    return sum * (0.8f / std::sqrt(pwr));
}

// All non-additive waves. p = phase 0..1, h/t/m = harmonics/timbre/morph 0..1,
// pw = pulse-width knob, cyc = cycle counter (used by Random Steps).
static float s2Shape(int wave, float p, float h, float t, float m, float pw, uint32_t cyc)
{
    const float th = S2_2PI * p;
    switch (wave)
    {
    case 0:
        return std::sin(th);
    case 1:
        return s2Tri(p);
    case 2:
        return s2Saw(p);
    case 3:
        return (p < pw) ? 1.f : -1.f;

    // ---- waveshapers ----
    case 14: // Wavefold
    {
        float x = std::sin(th) * (1.f + h * 7.f) + (t - 0.5f) * 2.f;
        float y = s2Fold(x);
        return s2Fold(y * (1.f + m * 3.f));
    }
    case 15: // Tri Fold
    {
        float x = s2Tri(p) * (1.f + h * 7.f) + (t - 0.5f) * 2.f;
        float y = s2Fold(x);
        return s2Lerp(y, std::sin(y * (float)M_PI * 0.5f), m);
    }
    case 16: // Saw Fold
    {
        float base = s2Lerp(s2Saw(p), std::sin(th), m);
        return s2Fold(base * (1.f + h * 7.f) + (t - 0.5f) * 2.f);
    }
    case 17: // Hard Clip
    {
        float g = 1.f + h * 12.f;
        float x = std::sin(th) * g + (t - 0.5f) * g;
        return s2Lerp(s2Clamp(x, -1.f, 1.f), std::tanh(x), m);
    }
    case 18: // Soft Sat
    {
        float g = 1.f + h * 10.f;
        float base = s2Lerp(std::sin(th), s2Tri(p), m);
        return std::tanh(base * g + (t - 0.5f) * 2.f) / std::tanh(g);
    }
    case 19: // Stair Saw
    {
        int n = 2 + (int)(h * 30.f);
        float e = std::pow(2.f, (m - 0.5f) * 4.f);
        float pp = std::pow(p, e);
        float y = 1.f - 2.f * std::floor(pp * (float)n) / (float)(n - 1);
        return s2Lerp(y, 1.f - 2.f * pp, t * t);
    }
    case 20: // Stair Tri
    {
        int n = 2 + (int)(h * 30.f);
        float tri = s2Tri(s2Warp(p, 0.05f + 0.9f * m));
        float u = tri * 0.5f + 0.5f;
        float lev = std::min(std::floor(u * (float)n), (float)(n - 1));
        float y = 2.f * lev / (float)(n - 1) - 1.f;
        return s2Lerp(y, tri, t * t);
    }
    case 21: // Decimated Sine
    {
        int n = 3 + (int)(h * 29.f);
        float pq = std::floor(p * (float)n) / (float)n;
        float y = std::sin(S2_2PI * pq);
        float lv = 2.f + m * 30.f;
        y = std::floor(y * lv + 0.5f) / lv;
        return s2Lerp(y, std::sin(th), t * t);
    }
    case 22: // FM 1:N
    {
        int r = 1 + (int)(h * 7.99f);
        float mod = s2Lerp(std::sin((float)r * th), s2Tri(s2Frac((float)r * p)), m);
        return std::sin(th + t * 8.f * mod);
    }
    case 23: // FM Nested
    {
        int r = 1 + (int)(h * 5.99f);
        return std::sin(th + t * 6.f * std::sin((float)r * th + m * 6.f * std::sin(th)));
    }

    // ---- sync / phase distortion / ring mod ----
    case 24: // Sync Saw
    {
        float p2 = s2Frac(p * (1.f + h * 7.f));
        float win = s2Lerp(1.f, std::sin((float)M_PI * p), t);
        return s2Lerp(s2Saw(p2), s2Tri(p2), m) * win;
    }
    case 25: // Sync Square
    {
        float p2 = s2Frac(p * (1.f + h * 7.f));
        float win = s2Lerp(1.f, std::sin((float)M_PI * p), t);
        float sq = (p2 < pw) ? 1.f : -1.f;
        return s2Lerp(sq, std::sin(S2_2PI * p2), m) * win;
    }
    case 26: // Sync Sine
    {
        float p2 = s2Frac(p * (1.f + h * 7.f));
        float win = s2Lerp(1.f, std::sin((float)M_PI * p), t);
        float y = (std::sin(S2_2PI * p2) + m * std::sin((float)M_PI * p2)) / (1.f + 0.5f * m);
        return y * win;
    }
    case 27: // PD Warp
    {
        float p1 = s2Warp(p, 0.5f - 0.48f * h);
        float p2 = s2Warp(p1, 0.5f - 0.48f * t);
        return s2Lerp(std::sin(S2_2PI * p2), s2Tri(p2), m);
    }
    case 28: // PD Resonant
    {
        float res = 1.f + h * 15.f;
        float w = std::pow(std::max(0.f, 1.f - p), 1.f + t * 3.f);
        float wt = 1.f - std::fabs(2.f * p - 1.f);
        return std::sin(th * res) * s2Lerp(w, wt, m);
    }
    case 29: // Ring Mod -> AM
    {
        int r = 1 + (int)(h * 7.99f);
        float mod = s2Lerp(std::sin((float)r * th), s2Tri(s2Frac((float)r * p)), m);
        return std::sin(th) * s2Lerp(mod, 0.5f * (mod + 1.f), t);
    }

    // ---- classic shapes ----
    case 30: // Rect Sine
    {
        float s = std::sin(th);
        float r = s2Lerp(std::max(s, 0.f), std::fabs(s), m);
        r = std::pow(r, 0.4f + 2.2f * h);
        return s2Lerp(2.f * r - 1.f, s, t * t);
    }
    case 31: // Parabola
    {
        float x = 2.f * s2Warp(p, 0.1f + 0.8f * t) - 1.f;
        float y = 1.f - 2.f * std::pow(std::fabs(x), 1.f + h * 3.f);
        return s2Lerp(y, -std::cos(th), m);
    }
    case 32: // Exp Saw
    {
        float k = 0.5f + h * 9.f;
        float y = 1.f - 2.f * s2ExpCurve(p, k, m);
        float q = (p < 0.5f) ? 2.f * p : 2.f - 2.f * p;
        float ys = 1.f - 2.f * s2ExpCurve(q, k, m);
        return s2Lerp(y, ys, t * t);
    }
    case 33: // Trapezoid
    {
        float tri = s2Tri(s2Warp(p, 0.05f + 0.9f * m));
        float y = s2Clamp(tri * (1.f + h * 7.f), -1.f, 1.f);
        return s2Lerp(y, std::sin(y * (float)M_PI * 0.5f), t);
    }
    case 34: // Narrow Pulse
    {
        float w = 0.5f - 0.48f * h;
        float y = (p < w) ? 1.f : -1.f;
        float ramp = (p < w) ? 1.f - 2.f * (p / w) : -1.f;
        y = s2Lerp(y, ramp, m);
        return s2Lerp(y, s2Saw(p), t);
    }
    case 35: // Twin Pulse
    {
        float w = 0.05f + 0.4f * h;
        float o = 0.1f + 0.8f * t;
        float a = (p < w) ? 1.f : 0.f;
        float b = (s2Frac(p + o) < w) ? 1.f : 0.f;
        float wb = 1.f - 1.8f * m * m;
        return (a + wb * b) * 0.8f;
    }
    case 36: // Gaussian / Mexican hat
    {
        float sigma = 0.03f + 0.25f * (1.f - h);
        float x = s2Frac(p - (0.1f + 0.8f * t) + 0.5f) - 0.5f;
        float u = x / sigma;
        float g = std::exp(-0.5f * u * u);
        return 2.f * s2Lerp(g, (1.f - u * u) * g, m) - 1.f;
    }
    case 37: // Sinc
    {
        float x = s2Frac(p - (0.1f + 0.8f * t) + 0.5f) - 0.5f;
        float arg = x * (4.f + h * 40.f);
        float s = (std::fabs(arg) < 1e-5f) ? 1.f : std::sin((float)M_PI * arg) / ((float)M_PI * arg);
        float win = s2Lerp(1.f, 0.5f + 0.5f * std::cos(S2_2PI * x), m);
        return 1.5f * s * win;
    }
    case 38: // Circle / superellipse
    {
        float ph = s2Warp(p, 0.1f + 0.8f * t);
        float e = 1.f + h * 3.f;
        float q = (ph < 0.5f) ? ph * 4.f - 1.f : ph * 4.f - 3.f;
        float mag = std::pow(std::max(0.f, 1.f - std::pow(std::fabs(q), e)), 1.f / e);
        float y = (ph < 0.5f) ? mag : -mag;
        return s2Lerp(y, std::sin(S2_2PI * ph), m);
    }
    case 39: // Random Steps
    {
        int n = 2 + (int)(h * 30.f);
        float f = p * (float)n;
        int i = std::min((int)std::floor(f), n - 1);
        float fr = f - (float)i;
        float v = s2Step((uint32_t)i, cyc, m);
        float vn = s2Step((uint32_t)((i + 1) % n), cyc, m);
        return s2Lerp(v, s2Lerp(v, vn, fr), t);
    }
    default:
        return std::sin(th);
    }
}

// Evaluates any of the 40 waves at phase p without oversampling or DC
// blocking (used by the display; nMax is the full additive range).
static float s2Eval(int wave, float p, float h, float t, float m, float pw, uint32_t cyc)
{
    if (wave >= S2_FIRST_ADD && wave <= S2_LAST_ADD)
        return s2Additive(wave, p, h, t, m, S2_MAX_HARM);
    return s2Shape(wave, p, h, t, m, pw, cyc);
}

// -----------------------------------------------------------------------------
// Custom ParamQuantity for the frequency knob (10 Hz - 6000 Hz, logarithmic)
// -----------------------------------------------------------------------------
// ParamQuantity that maps the raw knob (0..1) to a frequency on a logarithmic
// curve, and parses the editable display string as either Hz, a note name, or
// a semitone offset from C4.
struct S2FreqQ : rack::engine::ParamQuantity
{
    // Converts a normalised knob value (0..1) to Hz on a 10 Hz..6 kHz log curve.
    static float mapToFreq(float v)
    {
        return 10.0f * std::pow(600.0f, v);
    }
    // Inverse of mapToFreq: converts a frequency in Hz back to a normalised
    // knob value, clamped to the 10 Hz..6 kHz range.
    static float mapFromFreq(float f)
    {
        f = math::clamp(f, 10.0f, 6000.0f);
        return std::log(f / 10.0f) / std::log(600.0f);
    }

    // Returns the effective display value: the knob mapped to Hz.
    float getDisplayValue() override
    {
        return mapToFreq(getValue());
    }

    // Accepts a display value (Hz) and converts it back to a knob position.
    void setDisplayValue(float displayValue) override
    {
        setValue(mapFromFreq(displayValue));
    }

    // Returns the display string for the current value, e.g. "261.6 Hz".
    std::string getDisplayValueString() override
    {
        float freq = getDisplayValue();
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f Hz", freq);
        return std::string(buf);
    }

    // Parses an editable string. Accepts:
    //   - a raw frequency ("440", "261.63")
    //   - a note name with octave ("A4", "C#3", "Bb2")
    //   - a semitone offset from C4 ("+12", "-5")
    // Whitespace is ignored; unknown input falls back to the base class.
    void setDisplayValueString(std::string s) override
    {
        s.erase(std::remove_if(s.begin(), s.end(), ::isspace), s.end());
        if (s.empty())
            return;

        // Parses a note name like "A4", "C#3", "Bb2" into a frequency in Hz.
        // Returns -1 if the string is not a valid note name.
        auto parseNote = [](const std::string &note) -> float
        {
            static const std::map<std::string, int> noteMap = {
                {"C", 0}, {"C#", 1}, {"Db", 1}, {"D", 2}, {"D#", 3}, {"Eb", 3}, {"E", 4}, {"F", 5}, {"F#", 6}, {"Gb", 6}, {"G", 7}, {"G#", 8}, {"Ab", 8}, {"A", 9}, {"A#", 10}, {"Bb", 10}, {"B", 11}};
            std::regex noteRegex(R"(([A-Ga-g][#b]?)(-?\d+))");
            std::smatch match;
            if (std::regex_match(note, match, noteRegex))
            {
                std::string pitchClass = match[1].str();
                int octave = std::stoi(match[2].str());
                for (auto &c : pitchClass)
                    c = toupper(c);
                if (noteMap.find(pitchClass) != noteMap.end())
                {
                    int semitone = noteMap.at(pitchClass) + (octave + 1) * 12;
                    float freq = 440.0f * std::pow(2.0f, (semitone - 69) / 12.0f);
                    return freq;
                }
            }
            return -1.0f;
        };

        // Try a plain frequency first
        try
        {
            size_t pos;
            float freq = std::stof(s, &pos);
            if (pos == s.length())
            {
                setDisplayValue(freq);
                return;
            }
        }
        catch (...)
        {
        }

        // Then try a note name
        float noteFreq = parseNote(s);
        if (noteFreq > 0)
        {
            setDisplayValue(noteFreq);
            return;
        }

        // Then try a signed semitone offset from C4
        if (s[0] == '+' || s[0] == '-')
        {
            try
            {
                int semitones = std::stoi(s);
                float freq = dsp::FREQ_C4 * std::pow(2.0f, semitones / 12.0f);
                setDisplayValue(freq);
                return;
            }
            catch (...)
            {
            }
        }

        ParamQuantity::setDisplayValueString(s);
    }
};

// -----------------------------------------------------------------------------
// ParamQuantity subclasses (waveform, delay time, reverb decay, envelope decay)
// -----------------------------------------------------------------------------
// ParamQuantity for the waveform knob: a 40-position snap selector that shows
// the wave name, and accepts an index ("12") or a (partial) name ("organ").
struct S2WaveQ : rack::engine::ParamQuantity
{
    // Returns the wave name for the nearest of the 40 positions.
    std::string getDisplayValueString() override
    {
        int i = (int)std::nearbyint(getValue());
        i = std::max(0, std::min(i, S2_NUM_WAVES - 1));
        return S2_WAVE_NAMES[i];
    }
    // Accepts a wave index or a case-insensitive name / name prefix.
    void setDisplayValueString(std::string s) override
    {
        try
        {
            size_t pos;
            int v = std::stoi(s, &pos);
            if (pos == s.length())
            {
                setValue((float)std::max(0, std::min(v, S2_NUM_WAVES - 1)));
                return;
            }
        }
        catch (...)
        {
        }
        auto lower = [](std::string x)
        {
            for (auto &c : x)
                c = (char)tolower((unsigned char)c);
            return x;
        };
        std::string key = lower(s);
        if (!key.empty())
        {
            for (int i = 0; i < S2_NUM_WAVES; ++i)
            {
                if (lower(S2_WAVE_NAMES[i]).compare(0, key.size(), key) == 0)
                {
                    setValue((float)i);
                    return;
                }
            }
        }
        ParamQuantity::setDisplayValueString(s);
    }
};

// ParamQuantity for the delay-time knob: maps 0..1 to 20 ms .. 1.5 s on a
// logarithmic curve, and accepts an editable "ms" string.
struct S2DelayTimeQ : rack::engine::ParamQuantity
{
    // Returns the display string (e.g. "320 ms") for the current knob value.
    std::string getDisplayValueString() override
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.0f ms", 20.0f * std::pow(75.0f, getValue()));
        return std::string(buf);
    }
    // Parses a "ms" string back to a knob value on the same log curve.
    void setDisplayValueString(std::string s) override
    {
        try
        {
            size_t pos;
            float ms = std::stof(s, &pos);
            if (ms > 0.f)
            {
                setValue(std::log(ms / 20.0f) / std::log(75.0f));
                return;
            }
        }
        catch (...)
        {
        }
        ParamQuantity::setDisplayValueString(s);
    }
};

// ParamQuantity for the reverb-decay knob: maps 0..1 to 0.3 s .. 12 s on a
// logarithmic curve, and accepts an editable "s" string.
struct S2RevDecayQ : rack::engine::ParamQuantity
{
    // Returns the display string (e.g. "2.40 s") for the current knob value.
    std::string getDisplayValueString() override
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.2f s", 0.3f * std::pow(40.0f, getValue()));
        return std::string(buf);
    }
    // Parses an "s" string back to a knob value on the same log curve.
    void setDisplayValueString(std::string s) override
    {
        try
        {
            size_t pos;
            float sec = std::stof(s, &pos);
            if (sec > 0.f)
            {
                setValue(std::log(sec / 0.3f) / std::log(40.0f));
                return;
            }
        }
        catch (...)
        {
        }
        ParamQuantity::setDisplayValueString(s);
    }
};

// ParamQuantity for the trigger-envelope decay knob: a 4-position snap
// selector showing the four decay times.
struct S2EnvDecayQ : rack::engine::ParamQuantity
{
    // Returns the display string ("300 ms" / "1 s" / "4 s" / "12 s") based
    // on which of the four discrete positions the knob is nearest to.
    std::string getDisplayValueString() override
    {
        static const char *names[4] = {"300 ms", "1 s", "4 s", "12 s"};
        int i = (int)std::nearbyint(getValue());
        if (i < 0)
            i = 0;
        if (i > 3)
            i = 3;
        return names[i];
    }
};

// -----------------------------------------------------------------------------
// Modulated dual delay line: normal or ping-pong feedback, damped repeats,
// slow sine modulation of the read position (chorus / spacey wobble).
// -----------------------------------------------------------------------------
// A stereo delay line with interpolated fractional reads, one-pole damping
// in the feedback path and slowly-modulated read positions for chorus/wobble.
struct S2ModDelay
{
    static constexpr int MAX_DELAY = 72008; // ~1.5 s at 48 kHz
    std::vector<float> bufL, bufR;
    int size = 0;
    int writeIdx = 0;
    float lpL = 0.f, lpR = 0.f;
    float hpL = 0.f, hpR = 0.f; // DC-blocker state of the feedback path

    // Allocates the buffers once; further calls are no-ops.
    void setSampleRate(float sr)
    {
        (void)sr;
        if (bufL.empty())
        {
            size = MAX_DELAY;
            bufL.assign(size, 0.f);
            bufR.assign(size, 0.f);
            writeIdx = 0;
        }
    }

    // Reads the buffer `pos` samples BEHIND the write head (fractional, linear
    // interpolation, wrap-around). The old version read an absolute index, so the
    // effective delay time kept sweeping and could drop to ~0 samples, which made
    // the feedback loop resonate wildly.
    inline float readFrac(const std::vector<float> &buf, float pos) const
    {
        int ip = (int)pos;
        float frac = pos - (float)ip;
        int i0 = writeIdx - ip;
        if (i0 < 0)
            i0 += size;
        int i1 = i0 - 1;
        if (i1 < 0)
            i1 += size;
        float a = buf[i0];
        float b = buf[i1];
        return a + (b - a) * frac;
    }

    // Zeroes both delay buffers and resets the damping state.
    void clear()
    {
        std::fill(bufL.begin(), bufL.end(), 0.f);
        std::fill(bufR.begin(), bufR.end(), 0.f);
        lpL = lpR = 0.f;
        hpL = hpR = 0.f;
    }

    // Processes one stereo sample through the delay. Reads with LFO-modulated
    // positions, damps the feedback, applies normal or ping-pong feedback,
    // writes to the buffer, and updates ioL/ioR to the (dry + wet) output.
    void process(float &ioL, float &ioR, float delaySec, float fb, float modDepth,
                 float lfoA01, float lfoB01, float dampCoef, int mode, float sr)
    {
        if (size == 0)
            return;
        float maxDelay = (float)(size - 4);
        float d = std::min(delaySec * sr, maxDelay);
        float mod = 8.0f * modDepth;
        float posL = d + mod * (lfoA01 * 2.0f - 1.0f);
        float posR = d + mod * (lfoB01 * 2.0f - 1.0f);
        posL = std::max(1.0f, std::min(posL, maxDelay));
        posR = std::max(1.0f, std::min(posR, maxDelay));

        float dl = readFrac(bufL, posL);
        float dr = readFrac(bufR, posR);

        lpL += dampCoef * (dl - lpL);
        lpR += dampCoef * (dr - lpR);

        // DC-block the feedback path (~15 Hz) so offsets cannot build up
        const float hpCoef = 2.f * (float)M_PI * 15.f / sr;
        hpL += hpCoef * (lpL - hpL);
        hpR += hpCoef * (lpR - hpR);
        float fbL = lpL - hpL, fbR = lpR - hpR;

        float outL = ioL + (mode == 0 ? fbL : fbR) * fb;
        float outR = ioR + (mode == 0 ? fbR : fbL) * fb;

        // Soft-limit what is stored: runaway feedback saturates instead of exploding
        outL = s2SoftLimit(outL, 12.f);
        outR = s2SoftLimit(outR, 12.f);

        bufL[writeIdx] = outL;
        bufR[writeIdx] = outR;
        writeIdx++;
        if (writeIdx >= size)
            writeIdx = 0;

        ioL = outL;
        ioR = outR;
    }
};

// -----------------------------------------------------------------------------
// Simple peak compressor. The threshold is fixed at 3 V, amount scales the
// ratio from 1:1 (off) to 10:1.
// -----------------------------------------------------------------------------
// A simple feed-forward peak compressor: `amount` controls the ratio, with a
// fixed 3 V threshold and asymmetric attack/release on the envelope follower.
struct S2Comp
{
    float env = 0.f;

    // Processes one sample: tracks the peak envelope and applies gain
    // reduction above the 3 V threshold. Returns the (possibly) compressed
    // sample. `amount` <= 0.001 bypasses entirely.
    float process(float x, float amount, float sr)
    {
        if (amount < 0.001f)
            return x;

        float absx = std::fabs(x);
        float attackCoef = 1.f - std::exp(-1.f / (0.002f * sr));  // ~2 ms
        float releaseCoef = 1.f - std::exp(-1.f / (0.100f * sr)); // ~100 ms
        float coef = (absx > env) ? attackCoef : releaseCoef;
        env += coef * (absx - env);

        const float threshold = 3.0f;
        if (env <= threshold)
            return x;

        float ratio = 1.f + amount * 9.f; // 1:1 .. 10:1
        float compressedEnv = threshold + (env - threshold) / ratio;
        float gainRed = compressedEnv / env;
        return x * gainRed;
    }
};

// -----------------------------------------------------------------------------
// FDN reverb: 8 mutually-prime comb filters in parallel + 4 series allpass
// filters, with cross-feed and polarity decorrelation for a wide tail.
// -----------------------------------------------------------------------------
// An 8-line feedback delay network reverb: 8 prime-length damped combs in
// parallel, followed by 4 series Schroeder allpasses. Cross-feed between the
// two allpass outputs produces a decorrelated stereo tail.
struct S2Fdn
{
    static constexpr int NUM_COMBS = 8;
    static constexpr int NUM_ALLPASS = 4;
    static constexpr int COMB_BUF = 2400;
    static constexpr int AP_BUF = 1100;

    std::vector<float> combBuf[NUM_COMBS];
    int combLen[NUM_COMBS] = {0};
    int combIdx[NUM_COMBS] = {0};
    float combFb[NUM_COMBS] = {0.f};
    float combIn[NUM_COMBS] = {0.f}; // input gain that keeps every comb at ~unity RMS gain
    float combLp[NUM_COMBS] = {0.f};

    std::vector<float> apBuf[NUM_ALLPASS];
    float apLenF[NUM_ALLPASS] = {0.f};
    int apIdx[NUM_ALLPASS] = {0};
    float apG = 0.5f;

    float crossL = 0.f, crossR = 0.f;
    float inLP = 0.f; // input DC-blocker state
    float lastSize = -1.f, lastRt = -1.f;

    // Returns the i-th prime multiplier used to make the comb lengths
    // mutually prime (so their modal frequencies don't line up).
    static int primeMult(int i)
    {
        static const int m[NUM_COMBS] = {101, 113, 127, 139, 149, 163, 173, 181};
        return m[i];
    }

    // Allocates the comb/allpass buffers once; further calls are no-ops.
    void setSampleRate(float sr)
    {
        if (!combBuf[0].empty())
            return;
        for (int i = 0; i < NUM_COMBS; i++)
        {
            combBuf[i].assign(COMB_BUF, 0.f);
        }
        for (int i = 0; i < NUM_ALLPASS; i++)
        {
            apBuf[i].assign(AP_BUF, 0.f);
        }
        lastSize = -1.f;
        (void)sr;
    }

    // Recomputes per-comb delay lengths and feedback gains from `sizeParam`
    // and `rt60`. Each line's decay factor is set so the reverb loses 60 dB
    // over rt60 seconds; feedback is capped at 0.995 for stability.
    void setParams(float sr, float sizeParam, float rt60)
    {
        for (int i = 0; i < NUM_COMBS; i++)
        {
            int len = (int)(primeMult(i) * 10.0f * (0.28f + 0.72f * sizeParam));
            int minPrev = (i == 0) ? 0 : combLen[i - 1];
            if (len <= minPrev + 5)
                len = minPrev + 7;
            if (len < 16)
                len = 16 + i * 7;
            if (len > COMB_BUF - 8)
                len = COMB_BUF - 8 - (NUM_COMBS - i) * 7;
            combLen[i] = len;
            float decay = std::pow(10.0f, -3.0f * (float)len / (rt60 * sr));
            combFb[i] = std::min(decay, 0.995f);
            combIn[i] = std::sqrt(1.f - combFb[i] * combFb[i]);
        }
        static const int apBase[NUM_ALLPASS] = {293, 409, 541, 691};
        for (int i = 0; i < NUM_ALLPASS; i++)
        {
            apLenF[i] = (float)apBase[i];
        }
    }

    // Reads a buffer at a fractional delay `pos` behind the write head `wr`
    // with linear interpolation and wrap-around.
    inline float readFrac(const std::vector<float> &buf, int wr, float pos) const
    {
        int sz = (int)buf.size();
        int i0 = wr - (int)pos;
        if (i0 < 0)
            i0 += sz;
        int i1 = i0 + 1;
        if (i1 >= sz)
            i1 = 0;
        float frac = pos - (float)(int)pos;
        float a = buf[i0];
        float b = buf[i1];
        return a + (b - a) * frac;
    }

    // Runs one sample through all-pass filter i at length `len` and gain `g`,
    // returning its output. Used by process() for each of the 4 stages.
    inline float allpassUpdate(int i, float x, float len, float g)
    {
        std::vector<float> &b = apBuf[i];
        int sz = (int)b.size();
        int idx = apIdx[i];
        float y = readFrac(b, idx, len);
        b[idx] = x - g * y;
        apIdx[i] = (idx + 1) % sz;
        return y * g + x;
    }

    // Clears all comb/allpass buffers, damping state and cross-feed state.
    void clear()
    {
        for (int i = 0; i < NUM_COMBS; i++)
        {
            std::fill(combBuf[i].begin(), combBuf[i].end(), 0.f);
            combLp[i] = 0.f;
        }
        for (int i = 0; i < NUM_ALLPASS; i++)
        {
            std::fill(apBuf[i].begin(), apBuf[i].end(), 0.f);
        }
        crossL = crossR = 0.f;
        inLP = 0.f;
    }

    // Processes one stereo sample through the reverb. The mono difference of
    // the inputs drives the 8 parallel combs, whose sum is diffused by the 4
    // series allpasses (with LFO-modulated lengths), then split into L/R with
    // cross-feed for a wide tail.
    void process(float inL, float inR, float &outL, float &outR,
                 float sizeParam, float rt60, float mod,
                 float lfoA01, float lfoB01, float dampCoef, float sr)
    {
        if (combBuf[0].empty())
        {
            outL = outR = 0.f;
            return;
        }
        if (sizeParam != lastSize || rt60 != lastRt)
        {
            setParams(sr, sizeParam, rt60);
            lastSize = sizeParam;
            lastRt = rt60;
        }

        float bus = 0.5f * (inL - inR);
        inLP += 0.002f * (bus - inLP); // remove DC (~15 Hz high-pass)
        bus -= inLP;
        float sum = 0.f;
        for (int i = 0; i < NUM_COMBS; i++)
        {
            std::vector<float> &b = combBuf[i];
            int len = combLen[i];
            int idx = combIdx[i];
            if (idx >= len)
                idx = 0;
            float d = b[idx];
            combLp[i] += dampCoef * (d - combLp[i]);
            // input scaled by sqrt(1 - fb^2): long decays no longer multiply the level
            float out = bus * combIn[i] + combLp[i] * combFb[i];
            b[idx] = s2SoftLimit(out, 16.f);
            combIdx[i] = (idx + 1) % len;
            sum += d;
        }

        float l = lfoA01 * 0.5f - 0.25f;
        float m = lfoB01 * 0.5f - 0.25f;
        float y = sum;
        y = allpassUpdate(0, y, apLenF[0] + apLenF[0] * mod * l, apG);
        y = allpassUpdate(1, y, apLenF[1] + apLenF[1] * mod * m, apG);
        y = allpassUpdate(2, y, apLenF[2] + apLenF[2] * mod * m, apG);
        y = allpassUpdate(3, y, apLenF[3] + apLenF[3] * mod * l, apG);

        crossL += 0.5f * (y - crossL);
        crossR += 0.5f * (y - crossR);
        outL = 0.5f * (y + crossR * 0.7f);
        outR = 0.5f * (y - crossL * 0.7f);
    }
};

// ============================================================================
// Module
// ============================================================================
// Polyphonic stereo VCO (chaos-drifted, 40 waveforms, harmonics/timbre/morph
// macros) feeding a per-voice modulated delay and FDN reverb, with a
// trigger/gate envelope, master gain and compressor.
struct Stagger2 : Module
{
    // NOTE: the original parameters/inputs keep their ids; everything new is
    // appended before the *_LEN entry so old patches still load.
    enum ParamId
    {
        PITCH_PARAM,
        CHAOS_PARAM,
        CHAOS_MULT_PARAM,
        WAVE_PARAM,
        PW_PARAM,
        PITCH_ATTEN_PARAM,
        CHAOS_ATTEN_PARAM,
        WAVE_ATTEN_PARAM,
        PW_ATTEN_PARAM,
        CV_RANGE_PARAM,
        DELAY_TIME_PARAM,
        DELAY_FB_PARAM,
        DELAY_MIX_PARAM,
        DELAY_MOD_PARAM,
        DELAY_MODE_PARAM,
        REV_SIZE_PARAM,
        REV_DECAY_PARAM,
        REV_MIX_PARAM,
        ENV_DECAY_PARAM,
        TRIG_MODE_PARAM,
        GAIN_PARAM,
        COMP_PARAM,
        HARMONICS_PARAM,
        TIMBRE_PARAM,
        MORPH_PARAM,
        HARMONICS_ATTEN_PARAM,
        TIMBRE_ATTEN_PARAM,
        MORPH_ATTEN_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        CV_INPUT,
        PITCH_CV_INPUT,
        CHAOS_CV_INPUT,
        WAVE_CV_INPUT,
        PW_CV_INPUT,
        CHAOS_RESET_INPUT,
        TRIG_INPUT,
        HARMONICS_CV_INPUT,
        TIMBRE_CV_INPUT,
        MORPH_CV_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        VCO_OUTPUT,
        FX_L_OUTPUT,
        FX_R_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        CHAOS_LIGHT,
        ENV_LIGHT,
        LIGHTS_LEN
    };

    // Per-voice state: minBLEP oscillator, its own delay and reverb, its own
    // compressor and its own trigger envelope.
    struct Voice
    {
        float phase = 0.0f;
        float lastPhase = 0.0f;
        uint32_t cycle = 0;         // counts completed oscillator cycles
        float dcX = 0.f, dcY = 0.f; // DC blocker state (waves 4..39)
        dsp::MinBlepGenerator<16, 16> blepRising;
        dsp::MinBlepGenerator<16, 16> blepFalling;
        dsp::MinBlepGenerator<16, 16> blepSaw;
        // Per-voice space FX
        S2ModDelay delay;
        S2Fdn reverb;
        S2Comp comp;
        float env = 1.0f;
        bool gateHigh = false;
        float lastTrig = 0.0f;
    };
    std::vector<Voice> voices;

    // Post-FX master compressor for the stereo (mono-summed) FX outputs
    S2Comp compL, compR;

    float chaosDrift = 0.0f;
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist{-1.0f, 1.0f};

    float lfoPhaseA = 0.0f;
    float lfoPhaseB = 0.0f;

    // Predefined chaos multipliers and envelope decay times exposed via switches.
    static constexpr float multTable[5] = {1.0f, 2.0f, 3.0f, 5.0f, 10.0f};
    static constexpr float envDecayTable[4] = {0.3f, 1.0f, 4.0f, 12.0f};
    bool cvRangeTenVolt = false;
    float lastResetVoltage = 0.0f;

    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    float dispFreq = 261.6f, dispPw = 0.5f, dispPhase = 0.f;
    float dispH = 0.5f, dispT = 0.5f, dispM = 0.5f;
    int dispWave = 0, dispChannels = 1;
    float dispEnv = 1.f;
    bool dispTrigConn = false;
    float meterL = 0.f, meterR = 0.f;

    // Constructor: configures every parameter, input, output and light,
    // including the custom ParamQuantity types, and seeds the RNG.
    Stagger2()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam<S2FreqQ>(PITCH_PARAM, 0.f, 1.f, S2FreqQ::mapFromFreq(dsp::FREQ_C4), "Frequency");

        configParam(CHAOS_PARAM, 0.f, 1.f, 0.0f, "Chaos depth", "%", 0.f, 100.f);
        configSwitch(CHAOS_MULT_PARAM, 0.f, 4.f, 0.f, "Chaos multiplier", {"x1", "x2", "x3", "x5", "x10"});
        configParam<S2WaveQ>(WAVE_PARAM, 0.f, (float)(S2_NUM_WAVES - 1), 0.f, "Waveform");
        getParamQuantity(WAVE_PARAM)->snapEnabled = true;
        configParam(PW_PARAM, 0.f, 1.f, 0.5f, "Pulse width", "%", 0.f, 100.f);
        configParam(PITCH_ATTEN_PARAM, -1.f, 1.f, 0.f, "Pitch CV attenuverter", "%", 0.f, 100.f);
        configParam(CHAOS_ATTEN_PARAM, -1.f, 1.f, 0.f, "Chaos CV attenuverter", "%", 0.f, 100.f);
        configParam(WAVE_ATTEN_PARAM, -1.f, 1.f, 0.f, "Waveform CV attenuverter", "%", 0.f, 100.f);
        configParam(PW_ATTEN_PARAM, -1.f, 1.f, 0.f, "Pulse width CV attenuverter", "%", 0.f, 100.f);
        configSwitch(CV_RANGE_PARAM, 0.f, 1.f, 0.f, "CV range", {"+-5V", "+-10V"});

        // ---- Macro controls (waves 4..39) ----
        configParam(HARMONICS_PARAM, 0.f, 1.f, 0.5f, "Harmonics", "%", 0.f, 100.f);
        configParam(TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Timbre", "%", 0.f, 100.f);
        configParam(MORPH_PARAM, 0.f, 1.f, 0.5f, "Morph", "%", 0.f, 100.f);
        configParam(HARMONICS_ATTEN_PARAM, -1.f, 1.f, 0.f, "Harmonics CV attenuverter", "%", 0.f, 100.f);
        configParam(TIMBRE_ATTEN_PARAM, -1.f, 1.f, 0.f, "Timbre CV attenuverter", "%", 0.f, 100.f);
        configParam(MORPH_ATTEN_PARAM, -1.f, 1.f, 0.f, "Morph CV attenuverter", "%", 0.f, 100.f);

        // ---- Space FX ----
        configParam<S2DelayTimeQ>(DELAY_TIME_PARAM, 0.f, 1.f, 0.55f, "Delay time");
        configParam(DELAY_FB_PARAM, 0.f, 0.92f, 0.65f, "Delay feedback", "%", 0.f, 100.f);
        configParam(DELAY_MIX_PARAM, 0.f, 1.f, 0.6f, "Delay mix", "%", 0.f, 100.f);
        configParam(DELAY_MOD_PARAM, 0.f, 1.f, 0.5f, "Delay modulation", "%", 0.f, 100.f);
        configSwitch(DELAY_MODE_PARAM, 0.f, 1.f, 1.f, "Delay mode", {"Normal", "Ping-Pong"});

        configParam(REV_SIZE_PARAM, 0.f, 1.f, 0.75f, "Reverb size", "%", 0.f, 100.f);
        configParam<S2RevDecayQ>(REV_DECAY_PARAM, 0.f, 1.f, 0.55f, "Reverb decay");
        configParam(REV_MIX_PARAM, 0.f, 1.f, 0.75f, "Reverb mix", "%", 0.f, 100.f);

        // ---- Trigger / envelope ----
        configParam<S2EnvDecayQ>(ENV_DECAY_PARAM, 0.f, 3.f, 1.f, "Trigger decay");
        getParamQuantity(ENV_DECAY_PARAM)->snapEnabled = true;
        configSwitch(TRIG_MODE_PARAM, 0.f, 1.f, 0.f, "Trigger mode", {"Trig", "Gate"});

        // ---- Master output stage ----
        configParam(GAIN_PARAM, 0.f, 2.f, 0.7f, "Output gain", "%", 0.f, 100.f);
        configParam(COMP_PARAM, 0.f, 1.f, 0.5f, "Compressor", "%", 0.f, 100.f);

        configInput(CV_INPUT, "1V/octave pitch");
        configInput(PITCH_CV_INPUT, "Pitch modulation CV");
        configInput(CHAOS_CV_INPUT, "Chaos depth modulation CV");
        configInput(WAVE_CV_INPUT, "Waveform selection CV");
        configInput(PW_CV_INPUT, "Pulse width modulation CV");
        configInput(CHAOS_RESET_INPUT, "Reset chaos drift (trigger)");
        configInput(TRIG_INPUT, "Trigger (trig or gate)");
        configInput(HARMONICS_CV_INPUT, "Harmonics CV");
        configInput(TIMBRE_CV_INPUT, "Timbre CV");
        configInput(MORPH_CV_INPUT, "Morph CV");
        configOutput(VCO_OUTPUT, "VCO + FX");
        configOutput(FX_L_OUTPUT, "FX left (mono sum)");
        configOutput(FX_R_OUTPUT, "FX right (mono sum)");
        configLight(CHAOS_LIGHT, "Chaos activity");
        configLight(ENV_LIGHT, "Envelope");

        std::random_device rd;
        rng.seed(rd());
    }

    // Rack reset hook: clears the chaos drift, per-voice FX/envelope state
    // and the master compressors.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        chaosDrift = 0.0f;
        for (auto &v : voices)
        {
            v.delay.clear();
            v.reverb.clear();
            v.env = 1.0f;
            v.comp.env = 0.f;
            v.cycle = 0;
            v.dcX = v.dcY = 0.f;
        }
        compL.env = 0.f;
        compR.env = 0.f;
    }

    // Rack sample-rate-change hook: buffers are already sized for the worst
    // case, so no reallocation is required.
    void onSampleRateChange() override
    {
        // Buffers are sized for the worst case; nothing to reallocate.
    }

    // Main audio callback: reads parameters and CVs, updates the chaos drift
    // and shared LFOs, renders each active voice through the oscillator,
    // envelope and FX chain, sums the stereo FX outputs and compresses them,
    // and updates the display data and lights.
    void process(const ProcessArgs &args) override
    {
        float freqKnob = S2FreqQ::mapToFreq(params[PITCH_PARAM].getValue());
        float basePitchVoltage = std::log2(freqKnob / dsp::FREQ_C4);

        float chaosKnob = params[CHAOS_PARAM].getValue();
        float chaosMultIndex = params[CHAOS_MULT_PARAM].getValue();
        float waveKnob = params[WAVE_PARAM].getValue();
        float pwKnob = params[PW_PARAM].getValue();

        float harmKnob = params[HARMONICS_PARAM].getValue();
        float timbKnob = params[TIMBRE_PARAM].getValue();
        float morphKnob = params[MORPH_PARAM].getValue();
        float harmAtten = params[HARMONICS_ATTEN_PARAM].getValue();
        float timbAtten = params[TIMBRE_ATTEN_PARAM].getValue();
        float morphAtten = params[MORPH_ATTEN_PARAM].getValue();

        float chaosMultiplier = multTable[clamp((int)std::nearbyint(chaosMultIndex), 0, 4)];

        float pitchAtten = params[PITCH_ATTEN_PARAM].getValue();
        float chaosAtten = params[CHAOS_ATTEN_PARAM].getValue();
        float waveAtten = params[WAVE_ATTEN_PARAM].getValue();
        float pwAtten = params[PW_ATTEN_PARAM].getValue();

        cvRangeTenVolt = (params[CV_RANGE_PARAM].getValue() > 0.5f);
        float cvVoltageScale = cvRangeTenVolt ? 0.5f : 1.0f;

        // Chaos reset on a rising edge of the reset input
        float resetVoltage = inputs[CHAOS_RESET_INPUT].getVoltage();
        if (resetVoltage > 1.0f && lastResetVoltage <= 1.0f)
        {
            chaosDrift = 0.0f;
        }
        lastResetVoltage = resetVoltage;

        // Filtered white noise -> slowly drifting chaos voltage
        float whiteNoise = dist(rng);
        float cutoffHz = 1.5f;
        float alpha = 2.0f * M_PI * cutoffHz * args.sampleTime;
        alpha = std::min(alpha, 0.5f);
        chaosDrift = (1.0f - alpha) * chaosDrift + alpha * whiteNoise;

        // ---- FX / trigger controls ----
        float delayTimeSec = 20.0e-3f * std::pow(75.0f, params[DELAY_TIME_PARAM].getValue());
        float delayFb = params[DELAY_FB_PARAM].getValue();
        float delayMix = params[DELAY_MIX_PARAM].getValue();
        float delayMod = params[DELAY_MOD_PARAM].getValue();
        int delayMode = (int)std::nearbyint(params[DELAY_MODE_PARAM].getValue());

        float revSize = params[REV_SIZE_PARAM].getValue();
        float revRt60 = 0.3f * std::pow(40.0f, params[REV_DECAY_PARAM].getValue());
        float revMix = params[REV_MIX_PARAM].getValue();

        float envDecaySec = envDecayTable[clamp((int)std::nearbyint(params[ENV_DECAY_PARAM].getValue()), 0, 3)];
        int trigMode = (int)std::nearbyint(params[TRIG_MODE_PARAM].getValue());

        bool trigConnected = inputs[TRIG_INPUT].isConnected();
        float trigVoltage = inputs[TRIG_INPUT].getVoltage();

        // ---- Master output stage ----
        float gain = params[GAIN_PARAM].getValue();
        float compAmount = params[COMP_PARAM].getValue();

        // Shared slow LFOs used by both the delay and the reverb for modulation
        lfoPhaseA += args.sampleTime * 0.23f;
        if (lfoPhaseA >= 1.0f)
            lfoPhaseA -= 1.0f;
        lfoPhaseB += args.sampleTime * 0.17f;
        if (lfoPhaseB >= 1.0f)
            lfoPhaseB -= 1.0f;
        float lfoA01 = 0.5f * (std::sin(2.0f * M_PI * lfoPhaseA) + 1.0f);
        float lfoB01 = 0.5f * (std::sin(2.0f * M_PI * lfoPhaseB) + 1.0f);

        // DC blocker coefficient (~5 Hz high-pass)
        float dcR = s2Clamp(1.f - S2_2PI * 5.f * args.sampleTime, 0.9f, 0.99999f);

        // Grow the voice count to match the polyphonic input's channel count
        int channels = std::max(1, inputs[CV_INPUT].getChannels());
        outputs[VCO_OUTPUT].setChannels(channels);
        outputs[FX_L_OUTPUT].setChannels(1);
        outputs[FX_R_OUTPUT].setChannels(1);
        if ((int)voices.size() < channels)
        {
            voices.resize(channels);
        }
        dispChannels = channels;

        float fxSumL = 0.0f, fxSumR = 0.0f;
        float envMax = 0.0f;

        for (int c = 0; c < channels; ++c)
        {
            float pitchCV = inputs[PITCH_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float chaosCV = inputs[CHAOS_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float waveCV = inputs[WAVE_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;
            float pwCV = inputs[PW_CV_INPUT].getPolyVoltage(c) * cvVoltageScale;

            float modulatedPitchCV = pitchCV * pitchAtten;
            float modulatedChaosCV = chaosCV * chaosAtten;
            float modulatedWaveCV = waveCV * waveAtten;
            float modulatedPwCV = pwCV * pwAtten;

            // Macro controls: knob + attenuverted CV (+-5 V sweeps the full range)
            float harm = math::clamp(harmKnob + harmAtten * inputs[HARMONICS_CV_INPUT].getPolyVoltage(c) * cvVoltageScale / 5.0f, 0.0f, 1.0f);
            float timb = math::clamp(timbKnob + timbAtten * inputs[TIMBRE_CV_INPUT].getPolyVoltage(c) * cvVoltageScale / 5.0f, 0.0f, 1.0f);
            float morph = math::clamp(morphKnob + morphAtten * inputs[MORPH_CV_INPUT].getPolyVoltage(c) * cvVoltageScale / 5.0f, 0.0f, 1.0f);

            float totalPitchCV = basePitchVoltage + inputs[CV_INPUT].getPolyVoltage(c) + modulatedPitchCV;

            float chaosDepth = chaosKnob + modulatedChaosCV / 5.0f;
            chaosDepth = math::clamp(chaosDepth, 0.0f, 1.0f);

            float chaosVoltage = chaosDrift * chaosDepth * chaosMultiplier;

            float totalCV = totalPitchCV + chaosVoltage;
            // exact 2^x (the old bit-twiddling approximation could be up to ~6% off pitch)
            float freq = dsp::FREQ_C4 * std::exp2(totalCV);
            freq = math::clamp(freq, 10.0f, 6000.0f);

            float deltaPhase = freq * args.sampleTime;

            // Waveform select (quantized to the 40 discrete waves; +-5 V at full
            // attenuverter sweeps half of the list in either direction)
            float waveSelect = waveKnob + (modulatedWaveCV / 5.0f) * ((float)(S2_NUM_WAVES - 1) * 0.5f);
            waveSelect = math::clamp(waveSelect, 0.0f, (float)(S2_NUM_WAVES - 1));
            int waveIndex = (int)std::nearbyint(waveSelect);

            // Pulse width (clamped to avoid degenerate pulse edges)
            float pwValue = pwKnob + (modulatedPwCV / 10.0f);
            pwValue = math::clamp(pwValue, 0.01f, 0.99f);

            Voice &voice = voices[c];
            float &phase = voice.phase;
            float lastPhase = voice.lastPhase;
            float out = 0.0f;

            lastPhase = phase;
            phase += deltaPhase;
            if (phase >= 1.0f)
            {
                phase -= 1.0f;
                voice.cycle++;
            }

            if (waveIndex < S2_FIRST_ADD)
            {
                // Original four waves: minBLEP anti-aliasing on saw and square.
                switch (waveIndex)
                {
                case 0:
                    out = std::sin(2.0f * M_PI * phase);
                    break;
                case 1:
                    out = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
                    break;
                case 2:
                {
                    float saw = 1.0f - 2.0f * phase;
                    if (phase < lastPhase)
                    {
                        float t = (1.0f - lastPhase) / deltaPhase;
                        voice.blepSaw.insertDiscontinuity(t, 2.0f);
                    }
                    out = saw + voice.blepSaw.process();
                    break;
                }
                case 3:
                {
                    float square = (phase < pwValue) ? 1.0f : -1.0f;
                    if (phase < lastPhase)
                    {
                        float t = (1.0f - lastPhase) / deltaPhase;
                        voice.blepRising.insertDiscontinuity(t, 2.0f);
                    }
                    if ((lastPhase < pwValue && phase >= pwValue) || (pwValue == 0.0f && phase < lastPhase))
                    {
                        float t = (pwValue - lastPhase) / deltaPhase;
                        voice.blepFalling.insertDiscontinuity(t, -2.0f);
                    }
                    out = square + voice.blepRising.process() + voice.blepFalling.process();
                    break;
                }
                }
            }
            else if (waveIndex <= S2_LAST_ADD)
            {
                // Band-limited additive waves: partials stop below Nyquist.
                int nMax = std::max(1, (int)(0.49f / deltaPhase));
                out = s2Additive(waveIndex, phase, harm, timb, morph, nMax);
            }
            else
            {
                // Waveshapers: 2-point oversampling (average of two phase samples)
                float p0 = s2Frac(phase - 0.5f * deltaPhase);
                float a = s2Shape(waveIndex, phase, harm, timb, morph, pwValue, voice.cycle);
                float b = s2Shape(waveIndex, p0, harm, timb, morph, pwValue, voice.cycle);
                out = 0.5f * (a + b);
            }

            // DC blocker (state always runs; only used for waves 4..39)
            {
                float dcIn = out;
                float dcOut = dcIn - voice.dcX + dcR * voice.dcY;
                voice.dcX = dcIn;
                voice.dcY = dcOut;
                if (waveIndex >= S2_FIRST_ADD)
                    out = dcOut;
            }

            voice.lastPhase = phase;
            out *= 5.0f;

            // Lazy buffer allocation on the first sample
            if (voice.delay.bufL.empty())
            {
                voice.delay.setSampleRate(args.sampleRate);
                voice.reverb.setSampleRate(args.sampleRate);
            }

            // ---- Trigger envelope ----
            float &env = voice.env;
            float attackCoef = std::min(1.0f, args.sampleTime / 0.003f);
            float decayCoef = args.sampleTime / (0.001f + envDecaySec);
            if (trigConnected)
            {
                if (trigMode == 0)
                {
                    // Trig mode: rising edge fires a one-shot, then decay
                    if (trigVoltage > 1.0f && voice.lastTrig <= 1.0f)
                        env = 1.0f;
                    env *= (1.0f - decayCoef);
                }
                else
                {
                    // Gate mode: high level sustains, low level decays
                    if (trigVoltage > 1.0f)
                        voice.gateHigh = true;
                    else if (trigVoltage < 0.1f)
                        voice.gateHigh = false;
                    if (voice.gateHigh)
                        env += attackCoef * (1.0f - env);
                    else
                        env *= (1.0f - decayCoef);
                }
            }
            else
            {
                // No trigger patched: envelope is fully open
                env = 1.0f;
            }
            env = math::clamp(env, 0.0f, 1.0f);
            voice.lastTrig = trigVoltage;
            if (env > envMax)
                envMax = env;

            float sig = out * env;

            // ---- Space FX chain ----
            float dl = sig, dr = sig * 0.6f;
            voice.delay.process(dl, dr, delayTimeSec, delayFb, delayMod,
                                lfoA01, lfoB01, 0.35f, delayMode, args.sampleRate);
            float delayWetL = dl, delayWetR = dr;

            // Feed the reverb from the dry signal plus the delay output
            float revInL = sig * 0.5f + delayWetL;
            float revInR = sig * 0.5f + delayWetR;
            float revOutL = 0.f, revOutR = 0.f;
            voice.reverb.process(revInL, revInR, revOutL, revOutR,
                                 revSize, revRt60, delayMod, lfoA01, lfoB01, 0.30f, args.sampleRate);

            float wetL = delayMix * delayWetL + revMix * revOutL;
            float wetR = delayMix * delayWetR + revMix * revOutR;

            // Level-normalised dry/wet mix
            float norm = 2.0f / (1.0f + delayMix + revMix);
            float mixedL = (sig + 0.8f * wetL) * norm;
            float mixedR = (sig + 0.8f * wetR) * norm;

            // ---- Master gain + per-voice compressor on VCO output ----
            // Self-heal: if anything ever goes NaN/inf, flush this voice's FX state
            if (!std::isfinite(mixedL) || !std::isfinite(mixedR))
            {
                voice.delay.clear();
                voice.reverb.clear();
                voice.comp.env = 0.f;
                mixedL = mixedR = 0.f;
            }
            float vcoSample = (mixedL + mixedR * 0.15f) * gain;
            vcoSample = voice.comp.process(vcoSample, compAmount, args.sampleRate);
            vcoSample = s2SoftLimit(vcoSample, 10.f); // never exceeds +-10 V
            outputs[VCO_OUTPUT].setVoltage(vcoSample, c);

            // Accumulate for the (post-summed) FX outputs
            fxSumL += mixedL * gain;
            fxSumR += mixedR * gain;

            if (c == 0)
            {
                dispFreq = freq;
                dispPw = pwValue;
                dispWave = waveIndex;
                dispPhase = phase;
                dispH = harm;
                dispT = timb;
                dispM = morph;
            }
        }

        // ---- Master compressor for FX outputs ----
        // Polyphonic voices are summed incoherently, so scale by 1/sqrt(N)
        float polyScale = 1.f / std::sqrt((float)channels);
        fxSumL = compL.process(fxSumL * polyScale, compAmount, args.sampleRate);
        fxSumR = compR.process(fxSumR * polyScale, compAmount, args.sampleRate);
        fxSumL = s2SoftLimit(fxSumL, 10.f);
        fxSumR = s2SoftLimit(fxSumR, 10.f);
        outputs[FX_L_OUTPUT].setVoltage(fxSumL, 0);
        outputs[FX_R_OUTPUT].setVoltage(fxSumR, 0);

        float chaosIntensity = std::abs(chaosDrift) * chaosKnob;
        lights[CHAOS_LIGHT].setSmoothBrightness(chaosIntensity, args.sampleTime);
        lights[ENV_LIGHT].setSmoothBrightness(envMax, args.sampleTime);

        // display data
        dispEnv = envMax;
        dispTrigConn = trigConnected;
        float md = 1.f - 5.f * args.sampleTime;
        meterL = std::max(std::fabs(fxSumL), meterL * md);
        meterR = std::max(std::fabs(fxSumR), meterR * md);
    }
};

constexpr float Stagger2::multTable[5];
constexpr float Stagger2::envDecayTable[4];

// Anonymous namespace: UI helpers and the two panel widgets.
namespace
{

    namespace ag = agmm;

    // ------------------------------------------------------------
    // Panel background: title, display frame, one labelled cell per control
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, LED captions, row 1 with its
    // seven knob cells (plus CV/ATT captions) and rows 2-4 with seven
    // labelled cells each (row 4's outputs are tinted, its 4th slot is empty).
    struct S2Panel : TransparentWidget
    {
        S2Panel() { box.size = Vec(S2_W, S2_H); }
        // Draws all static panel art including section headers and cell plates.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, S2_W, S2_H, "STAGGER2");
            ag::bezel(vg, S2_DX, S2_DY, S2_DW, S2_DH);

            // LED captions beside the title
            ag::text(vg, 54.f, 15.f, "CHAOS", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            ag::text(vg, S2_W - 54.f, 15.f, "ENV", 7.f, ag::dim(), ag::ALIGN_R, 0.5f);

            // row 1: seven knobs, each with a CV jack + attenuverter
            const char *a[S2_COLS] = {"FREQ", "WAVE", "HARM", "TIMBRE", "MORPH", "CHAOS", "PW"};
            for (int i = 0; i < S2_COLS; ++i)
            {
                float cx = s2Col(i);
                ag::cell(vg, cx, S2_T1, S2_CW, S2_H1, a[i]);
                ag::text(vg, cx - 14.f, S2_R1_CAP, "CV", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
                ag::text(vg, cx + 14.f, S2_R1_CAP, "ATT", 7.f, ag::dim(), ag::ALIGN_C, 0.5f);
            }

            // rows 2-4: one cell per control, outputs tinted
            const char *r2[S2_COLS] = {"V/OCT", "RESET", "TRIG IN", "ENV DECAY", "TRIG/GATE", "MULT", "CV RANGE"};
            const char *r3[S2_COLS] = {"DLY TIME", "DLY FB", "DLY MOD", "DLY MIX", "PING-PONG", "RV SIZE", "RV TIME"};
            const char *r4[S2_COLS] = {"RV MIX", "GAIN", "COMP", "", "FX L", "FX R", "VCO OUT"};
            for (int i = 0; i < S2_COLS; ++i)
            {
                ag::cell(vg, s2Col(i), S2_T2, S2_CW, S2_HK, r2[i]);
                ag::cell(vg, s2Col(i), S2_T3, S2_CW, S2_HK, r3[i]);
                if (i != 3)
                    ag::cell(vg, s2Col(i), S2_T4, S2_CW, S2_HK, r4[i], i >= 4);
            }
            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // Display: waveform scope, envelope + output meters, FX readout box
    // ------------------------------------------------------------
    // Live display: draws the waveform scope (two cycles, scaled by the trigger
    // envelope), three vertical meters (ENV, L, R), a four-row FX readout box,
    // and the header/footer text. Uses demo data in the module browser.
    struct S2Display : TransparentWidget
    {
        Stagger2 *module = nullptr;
        // Constructor: positions the display widget under its bezel.
        S2Display(Stagger2 *m) : module(m)
        {
            box.pos = Vec(S2_DX, S2_DY);
            box.size = Vec(S2_DW, S2_DH);
        }

        // Static layer: fills the screen, draws the header/footer hairlines, the
        // scope and readout frames and the scope zero line.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, S2_DW, S2_DH);

            // separators under the header and above the footer
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, S2_HEAD_LINE);
            nvgLineTo(vg, S2_DW - 8.f, S2_HEAD_LINE);
            nvgMoveTo(vg, 8.f, S2_FOOT_LINE);
            nvgLineTo(vg, S2_DW - 8.f, S2_FOOT_LINE);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, S2_SC_X, S2_PLOT_Y, S2_SC_W, S2_PLOT_H);
            ag::plotFrame(vg, S2_RO_X, S2_PLOT_Y, S2_RO_W, S2_PLOT_H);

            nvgBeginPath(vg); // scope zero line
            nvgMoveTo(vg, S2_SC_X + 1.f, S2_PLOT_Y + S2_PLOT_H / 2.f);
            nvgLineTo(vg, S2_SC_X + S2_SC_W - 1.f, S2_PLOT_Y + S2_PLOT_H / 2.f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        // Layer-1 overlay: draws the waveform (with a phase marker), three
        // vertical meters, the four-row FX readout box, and the header/footer
        // readouts.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = S2_DW;

            // Snapshot (or demo values in the module browser)
            float freq = 261.6f, pw = 0.5f, phase = 0.3f, env = 1.f, mL = 2.5f, mR = 2.1f;
            float dTime = 0.35f, dFb = 0.65f, rt = 3.2f, gain = 0.7f;
            float hh = 0.5f, tt = 0.5f, mm = 0.5f;
            int wave = 0, ch = 1, mode = 1, tmode = 0;
            bool trig = false;
            if (module)
            {
                freq = module->dispFreq;
                pw = module->dispPw;
                phase = module->dispPhase;
                wave = module->dispWave;
                ch = module->dispChannels;
                env = module->dispEnv;
                trig = module->dispTrigConn;
                mL = module->meterL;
                mR = module->meterR;
                hh = module->dispH;
                tt = module->dispT;
                mm = module->dispM;
                dTime = 0.020f * std::pow(75.f, module->params[Stagger2::DELAY_TIME_PARAM].getValue());
                dFb = module->params[Stagger2::DELAY_FB_PARAM].getValue();
                rt = 0.3f * std::pow(40.f, module->params[Stagger2::REV_DECAY_PARAM].getValue());
                mode = (int)std::round(module->params[Stagger2::DELAY_MODE_PARAM].getValue());
                tmode = (int)std::round(module->params[Stagger2::TRIG_MODE_PARAM].getValue());
                gain = module->params[Stagger2::GAIN_PARAM].getValue();
            }
            wave = std::min(std::max(wave, 0), S2_NUM_WAVES - 1);
            env = std::min(std::max(env, 0.f), 1.f);
            const bool dcBlocked = (wave >= S2_FIRST_ADD);

            // ---- scope: two cycles, scaled by the trigger envelope ----
            // Waves 4..39 are DC-blocked in the DSP, so the scope removes the mean
            // and shrinks anything above full scale.
            const float mid = S2_PLOT_Y + S2_PLOT_H / 2.f, amp = S2_PLOT_H / 2.f - 4.f;
            const float envScale = 0.25f + 0.75f * env;
            const int N = 180;
            float vals[N + 1];
            float mean = 0.f, peak = 0.f;
            for (int i = 0; i <= N; ++i)
            {
                float t2 = 2.f * (float)i / N;
                float p = t2 - std::floor(t2);
                vals[i] = s2Eval(wave, p, hh, tt, mm, pw, (uint32_t)std::floor(t2));
                mean += vals[i];
            }
            mean = dcBlocked ? mean / (float)(N + 1) : 0.f;
            for (int i = 0; i <= N; ++i)
            {
                vals[i] -= mean;
                peak = std::max(peak, std::fabs(vals[i]));
            }
            const float scl = (peak > 1.f) ? 1.f / peak : 1.f;

            nvgBeginPath(vg);
            for (int i = 0; i <= N; ++i)
            {
                float x = S2_SC_X + S2_SC_W * (float)i / N, y = mid - vals[i] * scl * envScale * amp;
                if (i == 0)
                    nvgMoveTo(vg, x, y);
                else
                    nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, ag::bright());
            nvgStrokeWidth(vg, 1.5f);
            nvgLineJoin(vg, NVG_ROUND);
            nvgStroke(vg);

            { // phase marker: hairline and dot
                float x = S2_SC_X + S2_SC_W * 0.5f * phase;
                float v = (s2Eval(wave, phase, hh, tt, mm, pw, 0) - mean) * scl;
                float y = mid - v * envScale * amp;
                nvgBeginPath(vg);
                nvgMoveTo(vg, x, S2_PLOT_Y + 1.f);
                nvgLineTo(vg, x, S2_PLOT_Y + S2_PLOT_H - 1.f);
                nvgStrokeColor(vg, ag::withA(ag::violet(), 0.5f));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, x, y, 2.f);
                nvgFillColor(vg, ag::violet());
                nvgFill(vg);
            }

            // ---- meters: ENV (violet), L, R ----
            const float my = S2_PLOT_Y + 2.f, mh = S2_PLOT_H - 4.f;
            {
                float x = S2_MT_X;
                nvgBeginPath(vg);
                nvgRect(vg, x, my, S2_MT_W, mh);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 14));
                nvgFill(vg);
                float h = env * mh;
                if (h > 0.5f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x, my + mh - h, S2_MT_W, h);
                    nvgFillColor(vg, ag::violet());
                    nvgFill(vg);
                }
            }
            ag::meter(vg, S2_MT_X + S2_MT_DX, my, S2_MT_W, mh, mL / 5.f);
            ag::meter(vg, S2_MT_X + 2.f * S2_MT_DX, my, S2_MT_W, mh, mR / 5.f);

            // ---- readout box: label left, value right ----
            char buf[64];
            const float lx = S2_RO_X + 6.f, rx = S2_RO_X + S2_RO_W - 6.f;
            const float ly[4] = {S2_PLOT_Y + 6.f, S2_PLOT_Y + 16.f, S2_PLOT_Y + 26.f, S2_PLOT_Y + 36.f};
            const char *names[4] = {"DELAY", mode == 1 ? "PING-PONG" : "NORMAL", "REVERB", "TRIGGER"};
            for (int k = 0; k < 4; ++k)
                ag::text(vg, lx, ly[k], names[k], 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "%.0f MS", dTime * 1000.f);
            ag::text(vg, rx, ly[0], buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            snprintf(buf, sizeof(buf), "FB %.0f%%", dFb * 100.f);
            ag::text(vg, rx, ly[1], buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            snprintf(buf, sizeof(buf), "%.1f S", rt);
            ag::text(vg, rx, ly[2], buf, 7.f, ag::accent(), ag::ALIGN_R, 0.5f);
            ag::text(vg, rx, ly[3], trig ? (tmode == 1 ? "GATE" : "TRIG") : "OFF", 7.f,
                     trig ? ag::violet() : ag::dim(), ag::ALIGN_R, 0.5f);

            // ---- header and footer ----
            char wname[24];
            snprintf(wname, sizeof(wname), "%s", S2_WAVE_NAMES[wave]);
            for (char *q = wname; *q; ++q)
                *q = (char)toupper((unsigned char)*q);
            snprintf(buf, sizeof(buf), "%.1f HZ  %s%s", freq, wname, ch > 1 ? "  POLY" : "");
            ag::text(vg, 8.f, 8.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "GAIN %.0f%%", gain * 100.f);
            ag::text(vg, W - 8.f, 8.f, buf, 7.f, ag::warm(), ag::ALIGN_R, 0.5f);

            if (wave >= S2_FIRST_ADD)
                snprintf(buf, sizeof(buf), "H %.0f  T %.0f  M %.0f", hh * 100.f, tt * 100.f, mm * 100.f);
            else if (wave == 3)
                snprintf(buf, sizeof(buf), "PW %.0f%%", pw * 100.f);
            else
                snprintf(buf, sizeof(buf), "2 CYCLES");
            ag::text(vg, S2_SC_X + 2.f, S2_FOOT_TXT, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            ag::text(vg, S2_MT_X + S2_MT_W / 2.f, S2_FOOT_TXT, "ENV", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, S2_MT_X + S2_MT_DX + S2_MT_W / 2.f, S2_FOOT_TXT, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, S2_MT_X + 2.f * S2_MT_DX + S2_MT_W / 2.f, S2_FOOT_TXT, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, S2_RO_X + S2_RO_W - 2.f, S2_FOOT_TXT, "FX", 7.f, ag::dim(), ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
// Places the static panel, screws, live display, the two status LEDs, and
// the four rows of controls (oscillator, inputs/envelope, delay + reverb,
// output stage).
struct Stagger2Widget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, display, LEDs,
    // row 1 (7 knobs, each with CV jack + attenuverter),
    // row 2 (V/OCT, RESET, trigger section, MULT, CV range),
    // row 3 (delay + start of reverb), row 4 (reverb mix, gain, comp, outputs).
    Stagger2Widget(Stagger2 *module)
    {
        setModule(module);
        box.size = Vec(S2_W, S2_H);

        addChild(new S2Panel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new S2Display(module));

        // LEDs beside the title
        addChild(createLightCentered<SmallLight<ag::CyanLight>>(Vec(44.f, 15.f), module, Stagger2::CHAOS_LIGHT));
        addChild(createLightCentered<SmallLight<ag::VioletLight>>(Vec(S2_W - 44.f, 15.f), module, Stagger2::ENV_LIGHT));

        // ---- row 1: knob on top, CV jack and attenuverter below ----
        const int knob[S2_COLS] = {Stagger2::PITCH_PARAM, Stagger2::WAVE_PARAM, Stagger2::HARMONICS_PARAM,
                                   Stagger2::TIMBRE_PARAM, Stagger2::MORPH_PARAM, Stagger2::CHAOS_PARAM,
                                   Stagger2::PW_PARAM};
        const int att[S2_COLS] = {Stagger2::PITCH_ATTEN_PARAM, Stagger2::WAVE_ATTEN_PARAM, Stagger2::HARMONICS_ATTEN_PARAM,
                                  Stagger2::TIMBRE_ATTEN_PARAM, Stagger2::MORPH_ATTEN_PARAM, Stagger2::CHAOS_ATTEN_PARAM,
                                  Stagger2::PW_ATTEN_PARAM};
        const int cv[S2_COLS] = {Stagger2::PITCH_CV_INPUT, Stagger2::WAVE_CV_INPUT, Stagger2::HARMONICS_CV_INPUT,
                                 Stagger2::TIMBRE_CV_INPUT, Stagger2::MORPH_CV_INPUT, Stagger2::CHAOS_CV_INPUT,
                                 Stagger2::PW_CV_INPUT};
        for (int i = 0; i < S2_COLS; ++i)
        {
            float cx = s2Col(i);
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx, S2_R1_KNOB), module, knob[i]));
            addInput(createInputCentered<DarkPJ301MPort>(Vec(cx - 14.f, S2_R1_JACK), module, cv[i]));
            addParam(createParamCentered<Trimpot>(Vec(cx + 14.f, S2_R1_JACK), module, att[i]));
        }
        agmm::addKnobArcs(this, 2.f);
        // ---- row 2: pitch / reset / trigger / chaos multiplier / CV range ----
        const float y2 = S2_T2 + S2_CTL_DY;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(s2Col(0), y2), module, Stagger2::CV_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(s2Col(1), y2), module, Stagger2::CHAOS_RESET_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(s2Col(2), y2), module, Stagger2::TRIG_INPUT));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(3), y2), module, Stagger2::ENV_DECAY_PARAM));
        addParam(createParamCentered<CKSS>(Vec(s2Col(4), y2), module, Stagger2::TRIG_MODE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(5), y2), module, Stagger2::CHAOS_MULT_PARAM));
        addParam(createParamCentered<CKSS>(Vec(s2Col(6), y2), module, Stagger2::CV_RANGE_PARAM));
        agmm::addKnobArcs(this, 2.f);
        // ---- row 3: delay, then reverb size / time ----
        const float y3 = S2_T3 + S2_CTL_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(0), y3), module, Stagger2::DELAY_TIME_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(1), y3), module, Stagger2::DELAY_FB_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(2), y3), module, Stagger2::DELAY_MOD_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(3), y3), module, Stagger2::DELAY_MIX_PARAM));
        addParam(createParamCentered<CKSS>(Vec(s2Col(4), y3), module, Stagger2::DELAY_MODE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(5), y3), module, Stagger2::REV_SIZE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(6), y3), module, Stagger2::REV_DECAY_PARAM));

        // ---- row 4: reverb mix, output stage, outputs ----
        const float y4 = S2_T4 + S2_CTL_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(0), y4), module, Stagger2::REV_MIX_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(1), y4), module, Stagger2::GAIN_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(s2Col(2), y4), module, Stagger2::COMP_PARAM));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(s2Col(4), y4), module, Stagger2::FX_L_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(s2Col(5), y4), module, Stagger2::FX_R_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(s2Col(6), y4), module, Stagger2::VCO_OUTPUT));
        agmm::addKnobArcs(this, 2.f);    
    }
};

Model *modelStagger2 = createModel<Stagger2, Stagger2Widget>("Stagger2");