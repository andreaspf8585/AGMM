#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>

// ============================================================================
// AMB - generative ambient machine (Eno / AMB-1 style), dark and wide
// ----------------------------------------------------------------------------
// The machine composes by itself: it picks a chord every few seconds and plays
// eight layers against it:
//
//   PAD     slow detuned chord pad                CHOIR   formant "ah-oh-oo" voices
//   BELL    eight long FM bell loops              PLUCK   sparse music-box plucks
//   BASS    sub bass and low rumble               AIR     pitched wind swells
//   PULSE   Euclidean heartbeats and dust         GLASS   shimmering high partials
//
// Everything goes through a tape echo, a granular cloud, a probabilistic
// glitch stage and a very long reverb. Nothing ever repeats exactly.
//
//   KEY / SCALE   tonal centre and scale (default C minor)
//   TEMPO         speed of the bell loops, plucks, heartbeats and glitches
//   CHORD         seconds between chord changes (chords cross-fade over 7 s)
//   PAD..GLASS    layer levels
//   ECHO          TIME, FBK (feedback) and MIX of the tape ping-pong echo
//   REVERB        DECAY (0.8 .. 32 s), DAMP (darkness of the tail), MIX
//   GRAINS        MIX, SIZE, DENS (grains per second), SPRAY (pitch scatter
//                 and reversed grains: octaves, fifths, fourths)
//   GLITCH        PROB (chance per 16th note), LEN (slice length)
//                 stutter, reverse, bit-crush, tape stop, drop-out
//   DARK          overall tone: low values are bright, high values are dark
//   FREEZE        holds the reverb as an endless drone (button or gate input)
//
//   CLK IN   external clock: the tempo and beat phase follow the incoming pulses
//            (menu: pulses per beat 1..24, optional echo-time sync). The TEMPO
//            knob takes over again when the clock stops or is unplugged.
//   Inputs   GLT IN   trigger: fire a glitch now
//            FRZ IN   gate: hold the reverb while high
//   Outputs  OUT L / OUT R   stereo audio
//            CHORD    root of the current chord, 1 V/oct (C4 = 0 V)
//            BELL     pitch of the last bell, 1 V/oct
//            TRIG     trigger on every bell
//            CLOCK    trigger on every beat
//            GLITCH   gate, high while a glitch is running
//            LEVEL    output level, 0..10 V
// ============================================================================

// ============================================================================
// Scales
// ============================================================================
static const int AMB_NSCALES = 10;
static const char *AMB_SCALE_NAMES[AMB_NSCALES] = {
    "MAJOR", "MINOR", "DORIAN", "PHRYGIAN", "LYDIAN", "MIXOLYDIAN",
    "HARM MINOR", "LOCRIAN", "MIN PENT", "WHOLE TONE"};
static const int AMB_SCALE_LEN[AMB_NSCALES] = {7, 7, 7, 7, 7, 7, 7, 7, 5, 6};
static const int AMB_SCALE_INTERVALS[AMB_NSCALES][7] = {
    {0, 2, 4, 5, 7, 9, 11},
    {0, 2, 3, 5, 7, 8, 10},
    {0, 2, 3, 5, 7, 9, 10},
    {0, 1, 3, 5, 7, 8, 10},
    {0, 2, 4, 6, 7, 9, 11},
    {0, 2, 4, 5, 7, 9, 10},
    {0, 2, 3, 5, 7, 8, 11},
    {0, 1, 3, 5, 6, 8, 10},
    {0, 3, 5, 7, 10, 0, 0},
    {0, 2, 4, 6, 8, 10, 0}};
static const char *AMB_NOTE_NAMES[12] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

static const float AMB_PITCH_MIN = -16.f; // plot range (semitones above C3)
static const float AMB_PITCH_MAX = 52.f;
static const float AMB_HIST_WIN = 20.f; // seconds shown in the display

// Layer ids used in the note history
enum AmbLayer
{
    LAYER_PAD = 0,
    LAYER_BELL,
    LAYER_BASS,
    LAYER_PULSE,
    LAYER_CHOIR,
    LAYER_PLUCK,
    LAYER_AIR,
    LAYER_GLASS,
    LAYER_GLITCH,
    LAYER_COUNT
};

// 8 tape loops of unequal prime lengths (in beats)
static const int AMB_LOOP_BEATS[8] = {17, 23, 29, 31, 37, 41, 43, 47};

// 6 Euclidean patterns (32-beat cycle across all)
static const int AMB_EUCLID[6][2] = {
    {32, 5}, {24, 4}, {16, 2}, {28, 3}, {20, 5}, {12, 3}};

// Formant table for the choir: F1, F2, F3 for "ah", "oh", "oo", "eh", "ee"
static const float AMB_VOWELS[5][3] = {
    {800.f, 1150.f, 2900.f},
    {450.f, 800.f, 2830.f},
    {325.f, 700.f, 2700.f},
    {530.f, 1840.f, 2480.f},
    {270.f, 2290.f, 3010.f}};

static const int AMB_NSPEC = 32;

// ============================================================================
// DSP building blocks (no Rack dependencies)
// ============================================================================
namespace ambdsp
{
    static const float kPi = 3.14159265358979f;

    inline float lpCoef(float fc, float dt)
    {
        return 1.f - std::exp(-2.f * kPi * fc * dt);
    }

    inline int floorDiv(int a, int b)
    {
        int q = a / b;
        if ((a % b != 0) && ((a < 0) != (b < 0)))
            --q;
        return q;
    }

    inline float clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }

    // Fractional delay line (linear interpolation).
    struct DelayLine
    {
        std::vector<float> buf;
        int w = 0;
        int size = 4;

        void init(int n)
        {
            size = std::max(n, 2) + 2;
            buf.assign((size_t)size, 0.f);
            w = 0;
        }
        // Read before write: returns the sample written d samples ago.
        inline float read(float d) const
        {
            float rp = (float)w - d;
            while (rp < 0.f)
                rp += (float)size;
            int i0 = (int)rp;
            float f = rp - (float)i0;
            if (i0 >= size)
                i0 -= size;
            int i1 = i0 + 1;
            if (i1 >= size)
                i1 = 0;
            return buf[(size_t)i0] + (buf[(size_t)i1] - buf[(size_t)i0]) * f;
        }
        inline void write(float x)
        {
            buf[(size_t)w] = x;
            if (++w >= size)
                w = 0;
        }
    };

    // Schroeder all-pass with a fixed integer delay.
    struct Allpass
    {
        std::vector<float> buf;
        int idx = 0;
        float g = 0.6f;
        void init(int n)
        {
            buf.assign((size_t)std::max(n, 1), 0.f);
            idx = 0;
        }
        inline float process(float x)
        {
            float yn = buf[(size_t)idx];
            float vn = x + g * yn;
            buf[(size_t)idx] = vn;
            if (++idx >= (int)buf.size())
                idx = 0;
            return yn - g * vn;
        }
    };

    // Chamberlin state-variable filter, band-pass output.
    // f = 2 sin(pi fc / sr), q = 1 / Q. Peak gain of the band-pass is 1 / q.
    struct Svf
    {
        float lo = 0.f, bp = 0.f;
        inline float bandpass(float x, float f, float q)
        {
            lo += f * bp;
            float hp = x - lo - q * bp;
            bp += f * hp;
            return bp;
        }
        void reset() { lo = bp = 0.f; }
    };

    // PolyBLEP band-limited saw. ph in 0..1, inc = freq / sr.
    inline float sawPb(float ph, float inc)
    {
        float y = 2.f * ph - 1.f;
        if (ph < inc)
        {
            float t = ph / inc;
            y -= (t + t - t * t - 1.f);
        }
        else if (ph > 1.f - inc)
        {
            float t = (ph - 1.f) / inc;
            y -= (t * t + t + t + 1.f);
        }
        return y;
    }

    // ------------------------------------------------------------------
    // 2048-point complex FFT (radix 2) used by the spectrum display.
    // ------------------------------------------------------------------
    struct Fft
    {
        static const int N = 2048;
        float re[N], im[N];
        float cosT[N / 2], sinT[N / 2];
        float win[N];
        uint16_t rev[N];

        void init()
        {
            for (int i = 0; i < N / 2; ++i)
            {
                cosT[i] = std::cos(2.f * kPi * (float)i / (float)N);
                sinT[i] = std::sin(2.f * kPi * (float)i / (float)N);
            }
            for (int i = 0; i < N; ++i)
            {
                win[i] = 0.5f - 0.5f * std::cos(2.f * kPi * (float)i / (float)(N - 1));
                int r = 0, x = i;
                for (int b = 0; b < 11; ++b)
                {
                    r = (r << 1) | (x & 1);
                    x >>= 1;
                }
                rev[i] = (uint16_t)r;
                re[i] = im[i] = 0.f;
            }
        }
        void run()
        {
            for (int i = 0; i < N; ++i)
            {
                int j = rev[i];
                if (j > i)
                {
                    std::swap(re[i], re[j]);
                    std::swap(im[i], im[j]);
                }
            }
            for (int size = 2; size <= N; size <<= 1)
            {
                const int half = size >> 1;
                const int step = N / size;
                for (int i = 0; i < N; i += size)
                {
                    for (int k = 0; k < half; ++k)
                    {
                        const float wr = cosT[k * step];
                        const float wi = -sinT[k * step];
                        const int a = i + k, b = a + half;
                        const float tr = re[b] * wr - im[b] * wi;
                        const float ti = re[b] * wi + im[b] * wr;
                        re[b] = re[a] - tr;
                        im[b] = im[a] - ti;
                        re[a] += tr;
                        im[a] += ti;
                    }
                }
            }
        }
    };

    // ------------------------------------------------------------------
    // 8-line feedback delay network with a Hadamard mixing matrix.
    //   * RT60 is set directly (0.8 s .. 32 s)
    //   * a one-pole low-pass in every line darkens the tail
    //   * slowly modulated lines remove the metallic ringing
    //   * freeze: unity loop gain, no filtering, input muted
    // ------------------------------------------------------------------
    struct Reverb
    {
        static const int N = 8;
        float sr = 48000.f;
        DelayLine line[N];
        float len[N] = {};
        float lfoPh[N] = {};
        float lfoRate[N] = {0.07f, 0.11f, 0.15f, 0.19f, 0.23f, 0.27f, 0.31f, 0.37f};
        float lp[N] = {};
        float g[N] = {};
        Allpass apL[3], apR[3];
        DelayLine preL, preR;
        float preSamples = 1440.f;
        float modBase = 6.f;  // modulation depth in samples
        float modDepth = 6.f; // current depth (fades to 0 when frozen)
        float lpA = 0.3f;
        float injGain = 1.f;
        float outGain = 0.3f;
        int ctrl = 0;
        float hpXL = 0.f, hpYL = 0.f, hpXR = 0.f, hpYR = 0.f;

        void init(float sampleRate)
        {
            sr = sampleRate;
            static const int base[N] = {1913, 2269, 2657, 3067, 3527, 4013, 4519, 5099};
            const float k = sr / 48000.f;
            modBase = 6.f * k;
            modDepth = modBase;
            for (int i = 0; i < N; ++i)
            {
                len[i] = std::floor((float)base[i] * k);
                line[i].init((int)len[i] + (int)modBase + 8);
                lfoPh[i] = (float)i / (float)N;
                lp[i] = 0.f;
                g[i] = 0.9f;
            }
            static const int dl[3] = {142, 107, 379};
            static const int dr[3] = {149, 113, 367};
            const float kd = sr / 29761.f;
            for (int i = 0; i < 3; ++i)
            {
                apL[i].init((int)std::round((float)dl[i] * kd));
                apR[i].init((int)std::round((float)dr[i] * kd));
                apL[i].g = apR[i].g = 0.6f;
            }
            int pre = (int)(0.030f * sr);
            preL.init(pre + 4);
            preR.init(pre + 4);
            preSamples = (float)pre;
            ctrl = 0;
            injGain = 1.f;
            hpXL = hpYL = hpXR = hpYR = 0.f;
        }

        static inline void hadamard8(float *x)
        {
            for (int h = 1; h < 8; h *= 2)
                for (int i = 0; i < 8; i += 2 * h)
                    for (int j = i; j < i + h; ++j)
                    {
                        float a = x[j], b = x[j + h];
                        x[j] = a + b;
                        x[j + h] = a - b;
                    }
            for (int i = 0; i < 8; ++i)
                x[i] *= 0.35355339f;
        }

        void process(float inL, float inR, float decay01, float damp01, bool freeze,
                     float dt, float &outL, float &outR)
        {
            // Control rate (every 32 samples): RT60 -> loop gains, damping
            if (--ctrl <= 0)
            {
                ctrl = 32;
                float rt60 = 0.8f * std::pow(40.f, rack_clamp01(decay01));
                float fc = 12000.f * std::pow(0.0667f, rack_clamp01(damp01));
                if (freeze)
                {
                    for (int i = 0; i < N; ++i)
                        g[i] = 1.f;
                    lpA = 1.f;
                }
                else
                {
                    for (int i = 0; i < N; ++i)
                        g[i] = std::pow(10.f, -3.f * len[i] / (sr * rt60));
                    lpA = lpCoef(fc, dt);
                }
                outGain = 0.55f / std::sqrt(std::max(rt60, 1.f));
            }
            // Frozen: no modulation, so the loop is truly lossless
            modDepth += ((freeze ? 0.f : modBase) - modDepth) * lpCoef(1.f, dt);
            // Input fades out/in when freezing (50 ms)
            injGain += ((freeze ? 0.f : 1.f) - injGain) * lpCoef(3.f, dt);

            float xl = preL.read(preSamples);
            float xr = preR.read(preSamples);
            preL.write(inL);
            preR.write(inR);
            for (int i = 0; i < 3; ++i)
            {
                xl = apL[i].process(xl);
                xr = apR[i].process(xr);
            }
            xl *= injGain;
            xr *= injGain;

            float v[N], u[N];
            for (int i = 0; i < N; ++i)
            {
                lfoPh[i] += lfoRate[i] * dt;
                if (lfoPh[i] >= 1.f)
                    lfoPh[i] -= 1.f;
                float d = len[i] + modDepth * std::sin(2.f * kPi * lfoPh[i]);
                v[i] = line[i].read(d);
                lp[i] += lpA * (v[i] - lp[i]);
                u[i] = g[i] * lp[i];
            }
            hadamard8(u);
            for (int i = 0; i < N; ++i)
            {
                float sgn = (i & 2) ? -1.f : 1.f;
                float inj = ((i & 1) ? xr : xl) * sgn * 0.5f;
                line[i].write(u[i] + inj);
            }

            float wl = (v[0] - v[1] + v[2] - v[3] + v[4] - v[5] + v[6] - v[7]);
            float wr = (v[0] + v[1] - v[2] - v[3] + v[4] + v[5] - v[6] - v[7]);
            wl *= outGain;
            wr *= outGain;

            // DC blocker (about 15 Hz)
            const float r = 1.f - 2.f * kPi * 15.f * dt;
            hpYL = wl - hpXL + r * hpYL;
            hpXL = wl;
            hpYR = wr - hpXR + r * hpYR;
            hpXR = wr;
            outL = hpYL;
            outR = hpYR;
        }

        static inline float rack_clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
    };

    // ------------------------------------------------------------------
    // Tape-style ping-pong echo: dotted right side, wow and flutter, dark
    // saturating feedback path.
    // ------------------------------------------------------------------
    struct Echo
    {
        float sr = 48000.f;
        DelayLine dl[2];
        float timeSm = 0.5f;
        float wowPh = 0.f, flutPh = 0.f, driftPh = 0.f;
        float lpL = 0.f, lpR = 0.f, hpL = 0.f, hpR = 0.f;

        void init(float sampleRate)
        {
            sr = sampleRate;
            dl[0].init((int)(sr * 2.4f));
            dl[1].init((int)(sr * 2.4f));
            lpL = lpR = hpL = hpR = 0.f;
        }

        void process(float inL, float inR, float timeS, float fb, float dark, float dt,
                     float &wetL, float &wetR)
        {
            timeSm += (timeS - timeSm) * lpCoef(1.5f, dt); // tape speed glide
            wowPh += 0.31f * dt;
            if (wowPh >= 1.f)
                wowPh -= 1.f;
            flutPh += 4.7f * dt;
            if (flutPh >= 1.f)
                flutPh -= 1.f;
            driftPh += 0.053f * dt;
            if (driftPh >= 1.f)
                driftPh -= 1.f;
            float wow = 0.0030f * std::sin(2.f * kPi * wowPh) + 0.0007f * std::sin(2.f * kPi * flutPh) + 0.0020f * std::sin(2.f * kPi * driftPh);

            float tl = timeSm * sr * (1.f + wow);
            float tr = timeSm * 1.5f * sr * (1.f - wow * 0.8f);
            tl = std::min(std::max(tl, 8.f), sr * 2.2f);
            tr = std::min(std::max(tr, 8.f), sr * 2.2f);

            float eL = dl[0].read(tl);
            float eR = dl[1].read(tr);

            // Feedback path: low-pass (darker with DARK), high-pass, soft clip
            float fcLp = 2800.f * std::pow(0.35f, dark);
            float aLp = lpCoef(fcLp, dt);
            float aHp = lpCoef(150.f, dt);
            lpL += aLp * (eR - lpL); // ping-pong: right feeds left
            lpR += aLp * (eL - lpR);
            hpL += aHp * (lpL - hpL);
            hpR += aHp * (lpR - hpR);
            float fbL = std::tanh((lpL - hpL) * 1.2f) * (1.f / 1.2f);
            float fbR = std::tanh((lpR - hpR) * 1.2f) * (1.f / 1.2f);

            float f = std::min(std::max(fb, 0.f), 0.92f);
            dl[0].write(inL + fbL * f);
            dl[1].write(inR + fbR * f);
            wetL = eL;
            wetR = eR;
        }
    };

    // ------------------------------------------------------------------
    // Granular cloud. Records the incoming signal into a 4 s ring and plays
    // up to 28 overlapping grains from random places in it. Grains can be
    // pitched (octaves, fifths, fourths) and reversed.
    // ------------------------------------------------------------------
    struct Granular
    {
        static const int NG = 28;
        struct Grain
        {
            bool on = false;
            float dist = 0.f; // samples behind the write head
            float rate = 1.f; // change of dist per sample (1 - pitch ratio)
            float ratio = 1.f;
            float pL = 0.7f, pR = 0.7f;
            float amp = 0.f;
            int age = 0, len = 1;
        };
        Grain g[NG];
        std::vector<float> bufL, bufR;
        int size = 8, w = 0;
        float sr = 48000.f;
        uint32_t rng = 0x9E3779B9u;
        int active = 0;

        float rnd()
        {
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            return (float)(rng >> 8) * (1.f / 16777216.f);
        }
        void seed(uint32_t s) { rng = s | 1u; }

        void init(float sampleRate)
        {
            sr = sampleRate;
            size = (int)(sr * 4.f) + 8;
            bufL.assign((size_t)size, 0.f);
            bufR.assign((size_t)size, 0.f);
            w = 0;
            for (int i = 0; i < NG; ++i)
                g[i].on = false;
            active = 0;
        }

        void spawn(float sizeSec, float spray)
        {
            int i = -1;
            for (int k = 0; k < NG; ++k)
                if (!g[k].on)
                {
                    i = k;
                    break;
                }
            if (i < 0)
                return;
            Grain &gr = g[i];

            float len = sizeSec * sr * (0.7f + 0.6f * rnd());
            len = std::min(std::max(len, 0.01f * sr), 1.2f * sr);

            float r = 1.f;
            if (rnd() < spray)
            {
                static const float ratios[7] = {0.5f, 0.75f, 1.5f, 2.f, 0.5f, 2.f, 3.f};
                r = ratios[std::min(6, (int)(rnd() * 7.f))];
                if (rnd() < 0.25f * spray)
                    r = -r;
            }
            r *= 1.f + (rnd() - 0.5f) * 0.006f;
            const float rate = 1.f - r;

            len = std::min(len, 0.5f * (float)(size - 32) / std::max(1.f, std::fabs(rate)));
            float lo = 16.f, hi = (float)size - 16.f;
            if (rate < 0.f)
                lo += -rate * len;
            else
                hi -= rate * len;
            if (hi <= lo + 8.f)
                return;
            const float u = rnd();
            gr.dist = lo + (hi - lo) * u * u; // bias toward recent material
            gr.rate = rate;
            gr.ratio = r;
            gr.len = std::max(1, (int)len);
            gr.age = 0;
            gr.amp = 0.6f + 0.4f * rnd();
            const float pan = (rnd() * 2.f - 1.f) * 0.9f;
            const float a = (pan * 0.5f + 0.5f) * 0.5f * kPi;
            gr.pL = std::cos(a);
            gr.pR = std::sin(a);
            gr.on = true;
        }

        void process(float inL, float inR, float sizeSec, float dens, float spray,
                     float dt, float &outL, float &outR)
        {
            if (rnd() < dens * dt)
                spawn(sizeSec, spray);

            float oL = 0.f, oR = 0.f;
            int n = 0;
            for (int k = 0; k < NG; ++k)
            {
                Grain &gr = g[k];
                if (!gr.on)
                    continue;
                ++n;
                const float ph = (float)gr.age / (float)gr.len;
                const float x = ph * 2.f - 1.f;
                float win = 1.f - x * x;
                win *= win;

                float rp = (float)w - gr.dist;
                while (rp < 0.f)
                    rp += (float)size;
                int i0 = (int)rp;
                const float f = rp - (float)i0;
                if (i0 >= size)
                    i0 -= size;
                int i1 = i0 + 1;
                if (i1 >= size)
                    i1 = 0;
                const float sL = bufL[(size_t)i0] + (bufL[(size_t)i1] - bufL[(size_t)i0]) * f;
                const float sR = bufR[(size_t)i0] + (bufR[(size_t)i1] - bufR[(size_t)i0]) * f;
                const float m = 0.5f * (sL + sR);
                const float a = win * gr.amp;
                oL += (m * gr.pL * 0.6f + sL * 0.4f) * a;
                oR += (m * gr.pR * 0.6f + sR * 0.4f) * a;

                gr.dist += gr.rate;
                if (++gr.age >= gr.len)
                    gr.on = false;
            }
            active = n;

            // Overlap normalisation
            const float overlap = std::max(1.f, dens * sizeSec);
            const float norm = 1.f / std::sqrt(overlap);
            outL = oL * norm;
            outR = oR * norm;

            bufL[(size_t)w] = inL;
            bufR[(size_t)w] = inR;
            if (++w >= size)
                w = 0;
        }
    };

    // ------------------------------------------------------------------
    // Glitch stage: stutter, reverse, bit-crush, tape stop, drop-out.
    // Records the signal continuously; when triggered it replaces the
    // output with the glitched version (with short cross-fades).
    // ------------------------------------------------------------------
    struct Glitch
    {
        enum
        {
            STUTTER = 0,
            REVERSE,
            CRUSH,
            TAPESTOP,
            DROPOUT
        };

        std::vector<float> bufL, bufR, frzL, frzR;
        int size = 8, cap = 8, w = 0;
        float sr = 48000.f;
        bool active = false;
        int type = 0;
        int t = 0, total = 1;
        int slice = 1, curSlice = 1, pos = 0;
        int edge = 96;
        bool accel = false;
        float tsPos = 0.f;
        int startW = 0;
        float tsLpL = 0.f, tsLpR = 0.f;
        float holdL = 0.f, holdR = 0.f;
        int holdCnt = 0, crushDiv = 4;
        float crushQ = 16.f;
        float progress = 0.f;

        void init(float sampleRate)
        {
            sr = sampleRate;
            size = (int)(sr * 2.f) + 8;
            cap = (int)(sr * 1.2f) + 8;
            bufL.assign((size_t)size, 0.f);
            bufR.assign((size_t)size, 0.f);
            frzL.assign((size_t)cap, 0.f);
            frzR.assign((size_t)cap, 0.f);
            w = 0;
            active = false;
            edge = std::max(8, (int)(0.003f * sr));
        }

        void start(int ty, float sliceSec, float totalSec, bool acc, int div, float q)
        {
            type = ty;
            slice = std::min(std::max((int)(sliceSec * sr), (int)(0.02f * sr)), cap - 8);
            curSlice = slice;
            total = std::max((int)(totalSec * sr), (int)(0.04f * sr));
            t = 0;
            pos = 0;
            accel = acc;
            crushDiv = std::max(2, div);
            crushQ = q;
            holdCnt = 0;
            tsPos = 0.f;
            tsLpL = tsLpR = 0.f;
            startW = w - 1 + size;
            if (ty == STUTTER || ty == REVERSE)
            {
                for (int i = 0; i < slice; ++i)
                {
                    int src = w - slice + i;
                    while (src < 0)
                        src += size;
                    frzL[(size_t)i] = bufL[(size_t)src];
                    frzR[(size_t)i] = bufR[(size_t)src];
                }
            }
            progress = 0.f;
            active = true;
        }

        void process(float inL, float inR, float &outL, float &outR)
        {
            float rL = inL, rR = inR;
            if (active)
            {
                const int fade = std::max(1, edge * 2);
                float e = 1.f;
                if (t < fade)
                    e = (float)t / (float)fade;
                const int rem = total - t;
                if (rem < fade)
                    e = std::min(e, (float)rem / (float)fade);

                float sL = 0.f, sR = 0.f;
                switch (type)
                {
                case STUTTER:
                case REVERSE:
                {
                    const int p = pos;
                    const int idx = (type == STUTTER) ? p : (curSlice - 1 - p);
                    float gn = 1.f;
                    if (p < edge)
                        gn = (float)p / (float)edge;
                    const int tail = curSlice - p;
                    if (tail < edge)
                        gn = std::min(gn, (float)tail / (float)edge);
                    sL = frzL[(size_t)idx] * gn;
                    sR = frzR[(size_t)idx] * gn;
                    if (++pos >= curSlice)
                    {
                        pos = 0;
                        const int minS = (int)(0.03f * sr);
                        if (accel && curSlice > minS)
                            curSlice = std::max(minS, (int)((float)curSlice * 0.72f));
                    }
                    break;
                }
                case CRUSH:
                {
                    if (--holdCnt <= 0)
                    {
                        holdCnt = crushDiv;
                        holdL = inL;
                        holdR = inR;
                    }
                    sL = std::round(holdL * crushQ) / crushQ * 0.8f;
                    sR = std::round(holdR * crushQ) / crushQ * 0.8f;
                    break;
                }
                case TAPESTOP:
                {
                    float sp = 1.f - (float)t / (float)total;
                    sp *= sp;
                    tsPos += sp;
                    float rp = (float)startW + tsPos;
                    while (rp >= (float)size)
                        rp -= (float)size;
                    int i0 = (int)rp;
                    const float f = rp - (float)i0;
                    int i1 = i0 + 1;
                    if (i1 >= size)
                        i1 = 0;
                    const float xL = bufL[(size_t)i0] + (bufL[(size_t)i1] - bufL[(size_t)i0]) * f;
                    const float xR = bufR[(size_t)i0] + (bufR[(size_t)i1] - bufR[(size_t)i0]) * f;
                    const float a = 0.03f + 0.97f * sp;
                    tsLpL += a * (xL - tsLpL);
                    tsLpR += a * (xR - tsLpR);
                    const float vol = std::sqrt(sp);
                    sL = tsLpL * vol;
                    sR = tsLpR * vol;
                    break;
                }
                default: // DROPOUT
                    break;
                }
                rL = inL * (1.f - e) + sL * e;
                rR = inR * (1.f - e) + sR * e;
                progress = (float)t / (float)total;
                if (++t >= total)
                    active = false;
            }
            bufL[(size_t)w] = inL;
            bufR[(size_t)w] = inR;
            if (++w >= size)
                w = 0;
            outL = rL;
            outR = rR;
        }
    };
} // namespace ambdsp

// ============================================================================
// AmbEngine
// ============================================================================
struct AmbEngine
{
    // ---------------------------------------------------------------
    // Parameters (set by the module every sample)
    // ---------------------------------------------------------------
    int key = 0;
    int scaleIdx = 1;
    float tempoBpm = 72.f;
    // External clock lock (set by the module; unused while extClock is false)
    bool extClock = false;
    float extPhaseErr = 0.f; // beats still to be absorbed into beatPhase
    float padLevel = 0.60f;
    float bellLevel = 0.45f;
    float bassLevel = 0.50f;
    float pulseLevel = 0.25f;
    float choirLevel = 0.40f;
    float pluckLevel = 0.35f;
    float airLevel = 0.30f;
    float glassLevel = 0.30f;
    float dark = 0.60f;
    float echoTime = 0.50f;
    float echoMix = 0.45f;
    float echoFbk = 0.60f;
    float reverbMix = 0.60f;
    float reverbDecay = 0.70f;
    float reverbDamp = 0.55f;
    float grainMix = 0.35f;
    float grainSize = 0.20f;
    float grainDens = 9.f;
    float grainSpray = 0.35f;
    float glitchProb = 0.15f;
    float glitchLen = 0.40f;
    float chordInterval = 24.f;
    bool freeze = false;
    bool glitchTrigIn = false; // request a glitch now (consumed)

    // ---------------------------------------------------------------
    // Outputs and display data
    // ---------------------------------------------------------------
    float outL = 0.f, outR = 0.f;
    float level = 0.f;
    float meterL = 0.f, meterR = 0.f;
    float peakL = 0.f, peakR = 0.f;
    float chordRoot = 0.f;      // pad root, semitones above C3 (smoothed)
    float lastBellSemis = 12.f; // semitones above C3
    int bellTrigCount = 0;
    int beatCount = 0;
    double time = 0.0;
    float echoMs = 500.f;
    float reverbSec = 8.f;
    float chordProgress = 0.f;
    bool glitchActive = false;
    int glitchType = 0;
    float corr = 0.f;

    struct NoteEvent
    {
        double born;
        double end;
        float pitch;
        int layer;
    };
    static const int HIST_MAX = 320;
    NoteEvent hist[HIST_MAX];
    int histPos = 0;

    // Spectrum (0..1 per band, smoothed) and peak hold
    float spec[AMB_NSPEC] = {};
    float specPk[AMB_NSPEC] = {};
    // Goniometer ring
    static const int GON_N = 192;
    float gonL[GON_N] = {}, gonR[GON_N] = {};
    int gonPos = 0;

    // ---------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------
    float sr = 0.f;
    uint32_t rngState = 2463534242u;
    double beatPhase = 0.0;
    int lastBeat = -1;

    // Pad: two chord sets that cross-fade; 4 notes x 2 detuned voices each
    struct PadSet
    {
        float note[4] = {0.f, 7.f, 12.f, 14.f};
        float phase[8] = {};
        float freq[8] = {}; // Hz of each voice, set when the chord is chosen
        float gain = 0.f;
    };
    PadSet pad[2];
    int padCur = 0;
    float padFade = 1.f; // 0..1 progress of the cross-fade
    float padLfo[8] = {0.f, 0.13f, 0.27f, 0.41f, 0.55f, 0.69f, 0.83f, 0.97f};
    float padLfoRate[8] = {0.031f, 0.047f, 0.023f, 0.059f, 0.037f, 0.071f, 0.043f, 0.029f};
    float padTimer = 0.f;
    int lastDegree = -1;
    float breathPh = 0.f;

    // Bells
    static const int NBELLS = 8;
    float bellPh[NBELLS] = {}, bellModPh[NBELLS] = {};
    float bellEnv[NBELLS] = {}, bellAge[NBELLS] = {};
    float bellFreq[NBELLS] = {}, bellRatio[NBELLS] = {};
    float bellVel[NBELLS] = {}, bellPan[NBELLS] = {};
    int bellCycle[NBELLS] = {};

    // Bass + rumble
    float bassPitch = -6.f, bassTarget = -6.f;
    float bassPhase = 0.f, bassLfo = 0.f;
    float texLp1 = 0.f, texLp2 = 0.f;

    // Pulse
    static const int NPULSE = 6;
    int euclidStep[NPULSE] = {};
    float pulseEnv[NPULSE] = {}, pulsePhase[NPULSE] = {};
    float tickEnv[NPULSE] = {}, tickLp[NPULSE] = {};
    bool euclidPat[NPULSE][32] = {};
    int euclidLen[NPULSE] = {};

    // Choir: 4 chord voices, 2 detuned saws each, 3 formant filters per side
    float choirTarget[4] = {0.f, 3.f, 7.f, 14.f};
    float choirSemi[4] = {0.f, 3.f, 7.f, 14.f};
    float choirF[8] = {};
    float choirPh[8] = {};
    ambdsp::Svf choirSvf[2][3];
    float formF[3] = {0.1f, 0.15f, 0.35f};
    float vowelPh = 0.f, choirSwellPh = 0.f, choirVibPh = 0.f;

    // Pluck
    static const int NPLUCK = 6;
    float plPh[NPLUCK] = {}, plModPh[NPLUCK] = {}, plEnv[NPLUCK] = {}, plMod[NPLUCK] = {};
    float plAge[NPLUCK] = {}, plFreq[NPLUCK] = {}, plPan[NPLUCK] = {}, plVel[NPLUCK] = {};
    int plNext = 0, lastPluckStep = -1;
    float pluckDensPh = 0.f;
    float lastPluckSemis = 24.f;
    int pluckTrigCount = 0;

    // Air: pitched noise swells
    bool airOn = false;
    float airT = 0.f, airLen = 10.f, airIdle = 3.f, airEnv = 0.f, airFc = 800.f;
    float airF1 = 0.1f, airF2 = 0.15f;
    ambdsp::Svf airSvf[2][2];

    // Glass: 4 notes x 3 partials of shimmering sines
    float glassTarget[4] = {24.f, 27.f, 31.f, 36.f};
    float glassF[4] = {};
    float glPh[12] = {};
    float glLfo[12] = {};
    float glRate[12] = {};
    float glAmp[12] = {};
    float glassFade = 0.f;
    bool glassPending = false;

    int layerCtrl = 0;

    // Smoothed controls
    float sPad = 0.f, sBell = 0.f, sBass = 0.f, sPulse = 0.f;
    float sChoir = 0.f, sPluck = 0.f, sAir = 0.f, sGlass = 0.f;
    float sEchoMix = 0.f, sRevMix = 0.f, sDark = 0.f, sGrain = 0.f;

    // Effects and master
    ambdsp::Echo echo;
    ambdsp::Reverb reverb;
    ambdsp::Granular gran;
    ambdsp::Glitch glitch;
    float mLp1L = 0.f, mLp2L = 0.f, mLp1R = 0.f, mLp2R = 0.f;
    float dcXL = 0.f, dcYL = 0.f, dcXR = 0.f, dcYR = 0.f;

    // Glitch scheduling
    int lastGlitchStep = -1;
    bool glitchWas = false;
    double glitchEnd = -100.0;

    // Analysis
    ambdsp::Fft fft;
    float anaRing[2048] = {};
    int anaPos = 0, anaHop = 0;
    int bandLo[AMB_NSPEC] = {}, bandHi[AMB_NSPEC] = {};
    float bandTilt[AMB_NSPEC] = {};
    float bandHold[AMB_NSPEC] = {};
    int gonDec = 0, gonStep = 4;
    float sLR = 0.f, sLL = 0.f, sRR = 0.f;
    float peakHoldL = 0.f, peakHoldR = 0.f;

    // ---------------------------------------------------------------
    float rnd()
    {
        rngState ^= rngState << 13;
        rngState ^= rngState >> 17;
        rngState ^= rngState << 5;
        return (float)(rngState >> 8) * (1.f / 16777216.f);
    }

    void seed(uint32_t s)
    {
        rngState = s | 1u;
        gran.seed(s * 2654435761u + 12345u);
    }

    AmbEngine()
    {
        for (int p = 0; p < NPULSE; p++)
        {
            int steps = AMB_EUCLID[p][0];
            int pulses = AMB_EUCLID[p][1];
            euclidLen[p] = steps;
            for (int i = 0; i < steps; i++)
                euclidPat[p][i] = ((i * pulses) % steps) < pulses;
        }
        for (int i = 0; i < HIST_MAX; i++)
            hist[i] = NoteEvent{-1000.0, -1000.0, 0.f, 0};
        for (int b = 0; b < NBELLS; b++)
            bellRatio[b] = (b % 3 == 1) ? 2.f : ((b % 3 == 2) ? 1.5f : 1.f);
        for (int i = 0; i < 12; ++i)
        {
            glRate[i] = 0.045f + 0.017f * (float)((i * 7) % 11);
            glLfo[i] = (float)((i * 5) % 12) / 12.f;
        }
        fft.init();
    }

    void setSampleRate(float sampleRate)
    {
        if (sampleRate == sr)
            return;
        sr = sampleRate;
        echo.init(sr);
        reverb.init(sr);
        gran.init(sr);
        glitch.init(sr);
        gonStep = std::max(1, (int)(sr / 12000.f));

        // Log-spaced analysis bands, 30 Hz .. 16 kHz
        const float binHz = sr / (float)ambdsp::Fft::N;
        const float fmin = 30.f;
        const float fmax = std::min(16000.f, 0.45f * sr);
        for (int b = 0; b < AMB_NSPEC; ++b)
        {
            const float f0 = fmin * std::pow(fmax / fmin, (float)b / (float)AMB_NSPEC);
            const float f1 = fmin * std::pow(fmax / fmin, (float)(b + 1) / (float)AMB_NSPEC);
            int i0 = (int)std::floor(f0 / binHz);
            int i1 = (int)std::floor(f1 / binHz);
            i0 = std::max(1, i0);
            i1 = std::min(ambdsp::Fft::N / 2 - 1, std::max(i1, i0));
            bandLo[b] = i0;
            bandHi[b] = i1;
            bandTilt[b] = 2.5f * std::log2(std::sqrt(f0 * f1) / 1000.f);
        }
    }

    // ---------------------------------------------------------------
    int degSemis(int deg) const
    {
        const int n = AMB_SCALE_LEN[scaleIdx];
        const int oct = ambdsp::floorDiv(deg, n);
        const int idx = deg - oct * n;
        return AMB_SCALE_INTERVALS[scaleIdx][idx] + 12 * oct;
    }

    void addHistory(float pitch, int layer, double end)
    {
        hist[histPos] = NoteEvent{time, end, pitch, layer};
        histPos = (histPos + 1) % HIST_MAX;
    }

    // Close open sustained events of a layer (pad, choir, glass)
    void closeEvents(int layer)
    {
        for (int i = 0; i < HIST_MAX; i++)
            if (hist[i].layer == layer && hist[i].end < 0.0 && hist[i].born > -999.0)
                hist[i].end = time;
    }

    void retargetGlass()
    {
        closeEvents(LAYER_GLASS);
        for (int k = 0; k < 4; ++k)
        {
            int semi = (int)std::round(pad[padCur].note[k]) + 24;
            while (semi > 41)
                semi -= 12;
            while (semi < 17)
                semi += 12;
            glassTarget[k] = (float)semi;
            glassF[k] = 130.81f * std::pow(2.f, (float)semi / 12.f);
            addHistory((float)semi, LAYER_GLASS, -1.0);
        }
    }

    // Choose a new chord: root, third, fifth and a colour tone from the scale.
    void pickNewChord()
    {
        const int n = AMB_SCALE_LEN[scaleIdx];
        static const float w[7] = {0.30f, 0.08f, 0.14f, 0.18f, 0.16f, 0.10f, 0.04f};
        float tot = 0.f;
        for (int d = 0; d < std::min(n, 7); d++)
            tot += w[d];
        int deg = 0;
        for (int tries = 0; tries < 3; tries++)
        {
            float r = rnd() * tot, acc = 0.f;
            deg = 0;
            for (int d = 0; d < std::min(n, 7); d++)
            {
                acc += w[d];
                if (r <= acc)
                {
                    deg = d;
                    break;
                }
            }
            if (deg != lastDegree || rnd() < 0.3f)
                break;
        }
        lastDegree = deg;

        int root = (degSemis(deg) + key) % 12;
        if (root >= 8)
            root -= 12; // keep the pad low and dark
        const int rs = degSemis(deg);
        float third = (float)(degSemis(deg + 2) - rs);
        float fifth = (float)(degSemis(deg + 4) - rs);
        float colourChoices[3] = {
            (float)(degSemis(deg + 1) - rs + 12), // ninth
            (float)(degSemis(deg + 6) - rs),      // seventh
            12.f                                  // octave
        };
        float colour = colourChoices[std::min(2, (int)(rnd() * 3.f))];

        // Start the cross-fade into a fresh chord set.
        closeEvents(LAYER_PAD);
        closeEvents(LAYER_CHOIR);
        padCur = 1 - padCur;
        pad[padCur].note[0] = (float)root;
        pad[padCur].note[1] = (float)root + third;
        pad[padCur].note[2] = (float)root + fifth;
        pad[padCur].note[3] = (float)root + colour;
        {
            static const float det[4][2] = {{-0.06f, 0.05f}, {-0.08f, 0.07f}, {-0.05f, 0.09f}, {-0.10f, 0.06f}};
            for (int k = 0; k < 4; ++k)
                for (int j = 0; j < 2; ++j)
                    pad[padCur].freq[k * 2 + j] = 130.81f * std::pow(2.f, (pad[padCur].note[k] + det[k][j]) / 12.f);
        }
        padFade = 0.f;
        padTimer = 0.f;

        float b = (float)root - 12.f;
        while (b < -12.f)
            b += 12.f;
        while (b > 0.f)
            b -= 12.f;
        bassTarget = b;

        for (int k = 0; k < 4; k++)
        {
            addHistory(pad[padCur].note[k], LAYER_PAD, -1.0);
            choirTarget[k] = pad[padCur].note[k] + 12.f;
            addHistory(choirTarget[k], LAYER_CHOIR, -1.0);
        }
        addHistory(bassTarget, LAYER_BASS, time + 12.0);

        glassPending = true;
    }

    // First chord: no cross-fade.
    void startChord()
    {
        pickNewChord();
        padFade = 1.f;
        pad[padCur].gain = 1.f;
        pad[1 - padCur].gain = 0.f;
        bassPitch = bassTarget;
        chordRoot = pad[padCur].note[0];
        for (int k = 0; k < 4; ++k)
            choirSemi[k] = choirTarget[k];
        retargetGlass();
        glassPending = false;
        glassFade = 1.f;
    }

    // ---------------------------------------------------------------
    void triggerPluck()
    {
        const int v = plNext;
        plNext = (plNext + 1) % NPLUCK;
        int semi;
        if (rnd() < 0.65f)
            semi = (int)std::round(pad[padCur].note[std::min(3, (int)(rnd() * 4.f))]) + (rnd() < 0.5f ? 24 : 12);
        else
            semi = degSemis((int)(rnd() * (float)AMB_SCALE_LEN[scaleIdx])) + key + 24;
        while (semi > 45)
            semi -= 12;
        while (semi < 24)
            semi += 12;
        plFreq[v] = 130.81f * std::pow(2.f, (float)semi / 12.f);
        plEnv[v] = 1.f;
        plMod[v] = 1.f;
        plAge[v] = 0.f;
        plPh[v] = 0.f;
        plModPh[v] = 0.f;
        plVel[v] = 0.5f + 0.5f * rnd();
        plPan[v] = rnd() * 1.4f - 0.7f;
        lastPluckSemis = (float)semi;
        ++pluckTrigCount;
        addHistory((float)semi, LAYER_PLUCK, time);
    }

    void startAir()
    {
        airOn = true;
        airT = 0.f;
        airLen = 7.f + rnd() * 9.f;
        int semi = (int)std::round(pad[padCur].note[std::min(3, (int)(rnd() * 4.f))]) + 24 + (rnd() < 0.5f ? 12 : 0);
        while (semi > 50)
            semi -= 12;
        airFc = 130.81f * std::pow(2.f, (float)semi / 12.f);
        for (int c = 0; c < 2; ++c)
            for (int k = 0; k < 2; ++k)
                airSvf[c][k].reset();
        addHistory((float)semi, LAYER_AIR, time + (double)airLen);
    }

    void fireGlitch()
    {
        const float r = rnd();
        int ty;
        if (r < 0.30f)
            ty = ambdsp::Glitch::STUTTER;
        else if (r < 0.45f)
            ty = ambdsp::Glitch::REVERSE;
        else if (r < 0.65f)
            ty = ambdsp::Glitch::CRUSH;
        else if (r < 0.80f)
            ty = ambdsp::Glitch::TAPESTOP;
        else
            ty = ambdsp::Glitch::DROPOUT;

        const float beatSec = 60.f / std::max(tempoBpm, 20.f);
        float sliceBeats = 0.125f * std::pow(8.f, ambdsp::clamp01(glitchLen));
        if (rnd() < 0.35f)
            sliceBeats *= 0.5f;
        const float sliceSec = std::min(std::max(sliceBeats * beatSec, 0.03f), 1.0f);

        float totalSec = sliceSec;
        bool acc = false;
        int div = 4;
        float q = 16.f;
        switch (ty)
        {
        case ambdsp::Glitch::STUTTER:
            totalSec = std::min(sliceSec * (float)(3 + (int)(rnd() * 6.f)), 3.5f);
            acc = rnd() < 0.4f;
            break;
        case ambdsp::Glitch::REVERSE:
            totalSec = sliceSec * (float)(1 + (int)(rnd() * 2.f));
            break;
        case ambdsp::Glitch::CRUSH:
            totalSec = beatSec * (0.5f + rnd() * 1.5f);
            div = 3 + (int)(rnd() * 14.f);
            q = std::pow(2.f, (float)(3 + (int)(rnd() * 4.f)) - 1.f);
            break;
        case ambdsp::Glitch::TAPESTOP:
            totalSec = std::min(std::max(sliceSec * 2.f, 0.3f), 1.6f);
            break;
        default:
            totalSec = std::max(0.08f, sliceSec * (float)(1 + (int)(rnd() * 3.f)));
            break;
        }
        glitch.start(ty, sliceSec, totalSec, acc, div, q);
        glitchType = ty;
        addHistory((float)ty, LAYER_GLITCH, time + (double)totalSec);
    }

    // ---------------------------------------------------------------
    // Spectrum analysis, run once per 1024 samples
    // ---------------------------------------------------------------
    void analyse(float dt)
    {
        ambdsp::Fft &f = fft;
        for (int i = 0; i < ambdsp::Fft::N; ++i)
        {
            f.re[i] = anaRing[(anaPos + i) & 2047] * f.win[i];
            f.im[i] = 0.f;
        }
        f.run();

        const float norm = 1.f / ((float)ambdsp::Fft::N * 0.25f);
        const float hopSec = 1024.f * dt;
        for (int b = 0; b < AMB_NSPEC; ++b)
        {
            float p = 0.f;
            for (int k = bandLo[b]; k <= bandHi[b]; ++k)
                p += f.re[k] * f.re[k] + f.im[k] * f.im[k];
            float a = std::sqrt(p) * norm;
            float db = 20.f * std::log10(std::max(a, 1e-6f)) + bandTilt[b];
            float v = ambdsp::clamp01((db + 80.f) / 74.f);

            if (v > spec[b])
                spec[b] += (v - spec[b]) * 0.65f;
            else
                spec[b] += (v - spec[b]) * 0.14f;

            if (spec[b] >= specPk[b])
            {
                specPk[b] = spec[b];
                bandHold[b] = 0.7f;
            }
            else
            {
                bandHold[b] -= hopSec;
                if (bandHold[b] <= 0.f)
                    specPk[b] = std::max(spec[b], specPk[b] - hopSec * 0.35f);
            }
        }
    }

    void feedAnalysis(float l, float r, float dt)
    {
        anaRing[anaPos] = 0.5f * (l + r);
        anaPos = (anaPos + 1) & 2047;
        if (++anaHop >= 1024)
        {
            anaHop = 0;
            analyse(dt);
        }

        if (++gonDec >= gonStep)
        {
            gonDec = 0;
            gonL[gonPos] = l;
            gonR[gonPos] = r;
            gonPos = (gonPos + 1) % GON_N;
        }

        const float k = ambdsp::lpCoef(2.f, dt);
        sLR += (l * r - sLR) * k;
        sLL += (l * l - sLL) * k;
        sRR += (r * r - sRR) * k;
        corr = sLR / std::sqrt(sLL * sRR + 1e-9f);

        const float rel = std::exp(-dt / 0.25f);
        meterL = std::max(std::fabs(l), meterL * rel);
        meterR = std::max(std::fabs(r), meterR * rel);
        level = std::max(meterL, meterR);

        // Peak hold: hold 1.2 s, then fall
        const float al = std::fabs(l), ar = std::fabs(r);
        if (al >= peakL)
        {
            peakL = al;
            peakHoldL = 1.2f;
        }
        else
        {
            peakHoldL -= dt;
            if (peakHoldL < 0.f)
                peakL = std::max(al, peakL - dt * 0.6f);
        }
        if (ar >= peakR)
        {
            peakR = ar;
            peakHoldR = 1.2f;
        }
        else
        {
            peakHoldR -= dt;
            if (peakHoldR < 0.f)
                peakR = std::max(ar, peakR - dt * 0.6f);
        }
    }

    // Control-rate update of the new layers (every 32 samples)
    void updateLayerControl(float dt)
    {
        const float dtc = dt * 32.f;
        const float twoPi = 2.f * ambdsp::kPi;

        // Choir formants (vowel morph) and voice pitches
        {
            const float x = vowelPh * 5.f;
            const int i0 = ((int)x) % 5;
            const int i1 = (i0 + 1) % 5;
            float fr = x - std::floor(x);
            fr = fr * fr * (3.f - 2.f * fr);
            for (int j = 0; j < 3; ++j)
            {
                float fc = AMB_VOWELS[i0][j] + (AMB_VOWELS[i1][j] - AMB_VOWELS[i0][j]) * fr;
                fc = std::min(fc, 0.3f * sr);
                formF[j] = 2.f * std::sin(ambdsp::kPi * fc / sr);
            }
            choirVibPh += 5.2f * dtc;
            if (choirVibPh >= 1.f)
                choirVibPh -= 1.f;
            const float vib = 1.f + 0.0035f * std::sin(twoPi * choirVibPh);
            const float kc = ambdsp::lpCoef(0.45f, dtc);
            for (int k = 0; k < 4; ++k)
            {
                choirSemi[k] += (choirTarget[k] - choirSemi[k]) * kc;
                const float f = 130.81f * std::pow(2.f, choirSemi[k] / 12.f) * vib;
                choirF[k * 2] = f * 0.9965f;
                choirF[k * 2 + 1] = f * 1.0035f;
            }
        }

        // Air centre frequencies
        {
            const float fc = std::min(airFc, 0.3f * sr);
            airF1 = 2.f * std::sin(ambdsp::kPi * fc / sr);
            airF2 = 2.f * std::sin(ambdsp::kPi * std::min(fc * 1.5f, 0.3f * sr) / sr);
        }

        // Glass partial amplitudes (slow shimmer)
        {
            static const float partial[3] = {1.f, 0.38f, 0.16f};
            for (int i = 0; i < 12; ++i)
            {
                glLfo[i] += glRate[i] * dtc;
                if (glLfo[i] >= 1.f)
                    glLfo[i] -= 1.f;
                const float s = 0.5f + 0.5f * std::sin(twoPi * glLfo[i]);
                glAmp[i] = s * s * partial[i % 3];
            }
        }
    }

    // ---------------------------------------------------------------
    void process(float dt)
    {
        time += dt;
        const float twoPi = 2.f * ambdsp::kPi;
        const float darkAmt = std::min(std::max(sDark, 0.f), 1.f);

        // Smooth the level controls (about 50 ms)
        {
            float k = ambdsp::lpCoef(3.f, dt);
            sPad += (padLevel - sPad) * k;
            sBell += (bellLevel - sBell) * k;
            sBass += (bassLevel - sBass) * k;
            sPulse += (pulseLevel - sPulse) * k;
            sChoir += (choirLevel - sChoir) * k;
            sPluck += (pluckLevel - sPluck) * k;
            sAir += (airLevel - sAir) * k;
            sGlass += (glassLevel - sGlass) * k;
            sEchoMix += (echoMix - sEchoMix) * k;
            sRevMix += (reverbMix - sRevMix) * k;
            sDark += (dark - sDark) * k;
            sGrain += (grainMix - sGrain) * k;
        }

        // Per-sample envelope decay constants
        const float dBell = std::exp(-dt / 3.8f);
        const float dPulse = std::exp(-dt / 0.28f);
        const float dTick = std::exp(-dt / 0.03f);
        const float dPluck = std::exp(-dt / 1.2f);
        const float dPlMod = std::exp(-dt / 0.16f);

        // 1. Transport
        const float bps = std::max(tempoBpm, 20.f) / 60.f;
        beatPhase += (double)bps * dt;
        if (extPhaseErr != 0.f)
        {
            // Absorb the phase error smoothly: never run backwards, never leap
            const double step = (double)bps * dt;
            double c = (double)extPhaseErr * std::min(1.0, 8.0 * (double)dt);
            c = std::max(-0.9 * step, std::min(3.0 * step, c));
            if (std::fabs((double)extPhaseErr) < 1e-5)
                c = (double)extPhaseErr;
            beatPhase += c;
            extPhaseErr -= (float)c;
        }
        int beat = (int)std::floor(beatPhase);
        if (beat != lastBeat)
        {
            if (lastBeat >= 0)
                ++beatCount;
            lastBeat = beat;
        }

        // 2. Pad: a new chord every chordInterval seconds, cross-faded
        padTimer += dt;
        if (padTimer >= std::max(chordInterval, 2.f))
            pickNewChord();
        chordProgress = padTimer / std::max(chordInterval, 2.f);

        padFade = std::min(1.f, padFade + dt / 7.f);
        pad[padCur].gain = std::sin(0.5f * ambdsp::kPi * padFade);
        pad[1 - padCur].gain = std::cos(0.5f * ambdsp::kPi * padFade);
        chordRoot += (pad[padCur].note[0] - chordRoot) * ambdsp::lpCoef(0.15f, dt);

        breathPh += dt / 37.f;
        if (breathPh >= 1.f)
            breathPh -= 1.f;
        const float breath = 0.82f + 0.18f * std::sin(twoPi * breathPh);

        const float bright = 0.06f + 0.55f * (1.f - darkAmt);
        float padL = 0.f, padR = 0.f;
        for (int v = 0; v < 8; ++v)
        {
            padLfo[v] += padLfoRate[v] * dt;
            if (padLfo[v] >= 1.f)
                padLfo[v] -= 1.f;
        }
        for (int s = 0; s < 2; ++s)
        {
            PadSet &ps = pad[s];
            if (ps.gain < 0.001f)
                continue;
            for (int k = 0; k < 4; ++k)
            {
                static const float panK[4] = {0.f, -0.45f, 0.45f, 0.f};
                for (int j = 0; j < 2; ++j)
                {
                    const int vi = k * 2 + j;
                    ps.phase[vi] += ps.freq[vi] * dt;
                    if (ps.phase[vi] >= 1.f)
                        ps.phase[vi] -= 1.f;
                    float th = twoPi * ps.phase[vi];
                    float lfo = 0.5f + 0.5f * std::sin(twoPi * padLfo[vi]);
                    float hb = bright * (0.35f + 0.65f * lfo);
                    float osc = std::sin(th) + hb * 0.5f * std::sin(2.f * th) + hb * 0.28f * std::sin(3.f * th);
                    float amp = (0.55f + 0.45f * lfo) * (k == 0 ? 1.15f : 0.8f);
                    float sv = osc * amp * 0.105f * ps.gain;
                    float pan = std::min(std::max(panK[k] + (j ? 0.3f : -0.3f), -1.f), 1.f);
                    padL += sv * (1.f - pan) * 0.5f;
                    padR += sv * (1.f + pan) * 0.5f;
                }
            }
        }
        padL *= sPad * breath;
        padR *= sPad * breath;

        // 3. Bells: 8 long loops of unequal length, soft FM, dark
        float bellL = 0.f, bellR = 0.f;
        for (int b = 0; b < NBELLS; ++b)
        {
            static const float off[8] = {0.93f, 0.80f, 0.61f, 0.45f, 0.30f, 0.20f, 0.12f, 0.05f};
            const float loop = (float)AMB_LOOP_BEATS[b];
            int cyc = (int)std::floor((beatPhase + (double)(off[b] * loop)) / (double)loop);
            if (cyc != bellCycle[b])
            {
                bellCycle[b] = cyc;
                if (rnd() < 0.78f)
                {
                    int semi;
                    const int up = (b % 3 == 0) ? 24 : 12;
                    if (rnd() < 0.55f)
                        semi = (int)pad[padCur].note[std::min(3, (int)(rnd() * 4.f))] + up;
                    else
                        semi = degSemis((int)(rnd() * (float)AMB_SCALE_LEN[scaleIdx])) + key + up;
                    while (semi > 33)
                        semi -= 12;
                    while (semi < 12)
                        semi += 12;
                    bellFreq[b] = 130.81f * std::pow(2.f, (float)semi / 12.f);
                    bellEnv[b] = 1.f;
                    bellAge[b] = 0.f;
                    bellPh[b] = 0.f;
                    bellModPh[b] = 0.f;
                    bellVel[b] = 0.55f + 0.45f * rnd();
                    bellPan[b] = rnd() * 1.6f - 0.8f;
                    lastBellSemis = (float)semi;
                    ++bellTrigCount;
                    addHistory((float)semi, LAYER_BELL, -1.0);
                }
            }
            if (bellEnv[b] > 0.0008f)
            {
                bellAge[b] += dt;
                bellModPh[b] += bellFreq[b] * bellRatio[b] * dt;
                if (bellModPh[b] >= 1.f)
                    bellModPh[b] -= 1.f;
                bellPh[b] += bellFreq[b] * dt;
                if (bellPh[b] >= 1.f)
                    bellPh[b] -= 1.f;
                const float idx = (0.35f + 0.9f * (1.f - darkAmt)) * bellVel[b] * bellEnv[b] * bellEnv[b] * 1.6f;
                float carrier = std::sin(twoPi * bellPh[b] + idx * std::sin(twoPi * bellModPh[b]));
                float atk = std::min(1.f, bellAge[b] / 0.015f);
                float sv = carrier * bellEnv[b] * atk * bellVel[b] * 0.26f;
                bellEnv[b] *= dBell;
                bellL += sv * (1.f - bellPan[b]) * 0.5f;
                bellR += sv * (1.f + bellPan[b]) * 0.5f;
            }
        }
        bellL *= sBell;
        bellR *= sBell;

        // 4. Bass: sine sub with a touch of second harmonic, plus a slow rumble
        bassPitch += (bassTarget - bassPitch) * ambdsp::lpCoef(0.11f, dt);
        float bassF = 130.81f * std::pow(2.f, bassPitch / 12.f);
        bassPhase += bassF * dt;
        if (bassPhase >= 1.f)
            bassPhase -= 1.f;
        bassLfo += dt / 23.f;
        if (bassLfo >= 1.f)
            bassLfo -= 1.f;
        float bth = twoPi * bassPhase;
        float bassSig = (std::sin(bth) + 0.22f * std::sin(2.f * bth)) * (0.72f + 0.28f * std::sin(twoPi * bassLfo));

        float nz = rnd() * 2.f - 1.f;
        const float a90 = ambdsp::lpCoef(90.f, dt);
        texLp1 += a90 * (nz - texLp1);
        texLp2 += a90 * (texLp1 - texLp2);
        float rumble = texLp2 * 9.f * (0.6f + 0.4f * std::sin(twoPi * (bassLfo * 3.f + 0.2f)));

        float bassOut = (bassSig * 0.085f + rumble * 0.028f) * sBass;

        // 5. Pulse: soft low heartbeats and dusty ticks
        float pulseL = 0.f, pulseR = 0.f;
        const float tickA = ambdsp::lpCoef(1800.f, dt);
        for (int p = 0; p < NPULSE; ++p)
        {
            int len = euclidLen[p];
            int ns = ((int)std::floor((float)(beatPhase / 32.0) * (float)len)) % len;
            if (ns < 0)
                ns += len;
            if (ns != euclidStep[p])
            {
                euclidStep[p] = ns;
                if (euclidPat[p][ns] && rnd() < 0.85f)
                {
                    pulseEnv[p] = 1.f;
                    tickEnv[p] = 1.f;
                    pulsePhase[p] = 0.f;
                    addHistory(0.f, LAYER_PULSE, time);
                }
            }
            if (pulseEnv[p] > 0.001f)
            {
                float f0 = 44.f + 14.f * (float)(p % 3);
                float fr = f0 * (1.f + 1.6f * pulseEnv[p] * pulseEnv[p]);
                pulsePhase[p] += fr * dt;
                if (pulsePhase[p] >= 1.f)
                    pulsePhase[p] -= 1.f;
                float sv = std::sin(twoPi * pulsePhase[p]) * pulseEnv[p] * 0.07f;
                pulseL += sv;
                pulseR += sv;
                pulseEnv[p] *= dPulse;
            }
            if (tickEnv[p] > 0.001f)
            {
                float n2 = rnd() * 2.f - 1.f;
                tickLp[p] += (n2 - tickLp[p]) * tickA;
                float tk = tickLp[p] * tickEnv[p] * 0.06f;
                pulseL += tk * (0.5f + 0.1f * (float)p);
                pulseR += tk * (1.0f - 0.1f * (float)p);
                tickEnv[p] *= dTick;
            }
        }
        pulseL *= sPulse;
        pulseR *= sPulse;

        // ------------------------------------------------------------
        // New layers
        // ------------------------------------------------------------
        if (--layerCtrl <= 0)
        {
            layerCtrl = 32;
            updateLayerControl(dt);
        }

        // 6. Choir: detuned band-limited saws through moving vowel formants
        vowelPh += dt / 47.f;
        if (vowelPh >= 1.f)
            vowelPh -= 1.f;
        choirSwellPh += dt / 29.f;
        if (choirSwellPh >= 1.f)
            choirSwellPh -= 1.f;
        float choirL = 0.f, choirR = 0.f;
        {
            static const float fq[3] = {0.13f, 0.10f, 0.08f}; // 1/Q per formant
            static const float fa[3] = {1.0f, 0.55f, 0.25f};
            float sawL = 0.f, sawR = 0.f;
            for (int o = 0; o < 8; ++o)
            {
                const float inc = choirF[o] * dt;
                choirPh[o] += inc;
                if (choirPh[o] >= 1.f)
                    choirPh[o] -= 1.f;
                const float s = ambdsp::sawPb(choirPh[o], inc);
                if (((o >> 1) & 1) == 0)
                    sawL += s;
                else
                    sawR += s;
            }
            sawL *= 0.25f;
            sawR *= 0.25f;
            float fl = 0.f, fr = 0.f;
            for (int j = 0; j < 3; ++j)
            {
                fl += choirSvf[0][j].bandpass(sawL, formF[j], fq[j]) * fq[j] * fa[j];
                fr += choirSvf[1][j].bandpass(sawR, formF[j], fq[j]) * fq[j] * fa[j];
            }
            const float sw = std::sin(ambdsp::kPi * choirSwellPh);
            const float swell = 0.30f + 0.70f * sw * sw;
            choirL = (fl * 0.8f + fr * 0.2f) * swell * 1.3f * sChoir;
            choirR = (fr * 0.8f + fl * 0.2f) * swell * 1.3f * sChoir;
        }

        // 7. Pluck: sparse FM music-box notes, phrased by a slow density LFO
        pluckDensPh += dt / 53.f;
        if (pluckDensPh >= 1.f)
            pluckDensPh -= 1.f;
        {
            const int pstep = (int)std::floor(beatPhase * 2.0);
            if (pstep != lastPluckStep)
            {
                lastPluckStep = pstep;
                const float dens = 0.40f + 0.60f * (0.5f + 0.5f * std::sin(twoPi * pluckDensPh));
                if (rnd() < 0.20f * dens)
                    triggerPluck();
            }
        }
        float plL = 0.f, plR = 0.f;
        for (int v = 0; v < NPLUCK; ++v)
        {
            if (plEnv[v] <= 0.0008f)
                continue;
            plAge[v] += dt;
            plModPh[v] += plFreq[v] * 2.f * dt;
            if (plModPh[v] >= 1.f)
                plModPh[v] -= 1.f;
            plPh[v] += plFreq[v] * dt;
            if (plPh[v] >= 1.f)
                plPh[v] -= 1.f;
            const float idx = 2.2f * plVel[v] * plMod[v];
            const float car = std::sin(twoPi * plPh[v] + idx * std::sin(twoPi * plModPh[v]));
            const float atk = std::min(1.f, plAge[v] / 0.002f);
            const float sv = car * plEnv[v] * atk * plVel[v] * 0.30f;
            plEnv[v] *= dPluck;
            plMod[v] *= dPlMod;
            plL += sv * (1.f - plPan[v]) * 0.5f;
            plR += sv * (1.f + plPan[v]) * 0.5f;
        }
        plL *= sPluck;
        plR *= sPluck;

        // 8. Air: pitched noise swells (band-pass noise on a chord tone + a fifth)
        float airL = 0.f, airR = 0.f;
        if (airOn)
        {
            airT += dt;
            const float p = airT / airLen;
            if (p >= 1.f)
            {
                airOn = false;
                airIdle = 1.f + rnd() * 6.f;
                airEnv = 0.f;
            }
            else
            {
                const float s = std::sin(ambdsp::kPi * p);
                airEnv = s * s;
            }
        }
        else
        {
            airEnv = 0.f;
            airIdle -= dt;
            if (airIdle <= 0.f)
                startAir();
        }
        if (airEnv > 0.0005f)
        {
            static const float q1 = 0.05f, q2 = 0.06f;
            const float nL = rnd() * 2.f - 1.f, nR = rnd() * 2.f - 1.f;
            const float aL = airSvf[0][0].bandpass(nL, airF1, q1) * q1 + 0.5f * airSvf[0][1].bandpass(nL, airF2, q2) * q2;
            const float aR = airSvf[1][0].bandpass(nR, airF1, q1) * q1 + 0.5f * airSvf[1][1].bandpass(nR, airF2, q2) * q2;
            airL = aL * airEnv * 3.6f * sAir;
            airR = aR * airEnv * 3.6f * sAir;
        }

        // 9. Glass: shimmering sine partials; the voices dip out while the chord changes
        float glL = 0.f, glR = 0.f;
        {
            if (glassPending)
            {
                glassFade -= dt / 2.5f;
                if (glassFade <= 0.f)
                {
                    glassFade = 0.f;
                    glassPending = false;
                    retargetGlass();
                }
            }
            else
                glassFade = std::min(1.f, glassFade + dt / 4.f);
            const float gg = glassFade * glassFade * (3.f - 2.f * glassFade);
            if (gg > 0.0005f)
            {
                static const float panG[4] = {-0.7f, 0.35f, -0.3f, 0.7f};
                for (int k = 0; k < 4; ++k)
                {
                    float acc = 0.f;
                    for (int p = 0; p < 3; ++p)
                    {
                        const int i = k * 3 + p;
                        glPh[i] += glassF[k] * (float)(p + 1) * dt;
                        if (glPh[i] >= 1.f)
                            glPh[i] -= 1.f;
                        acc += std::sin(twoPi * glPh[i]) * glAmp[i];
                    }
                    acc *= 0.14f * gg;
                    glL += acc * (1.f - panG[k]) * 0.5f;
                    glR += acc * (1.f + panG[k]) * 0.5f;
                }
            }
            glL *= sGlass;
            glR *= sGlass;
        }

        // 10. Dry sum
        float dryL = std::tanh(padL + bellL + bassOut + pulseL + choirL + plL + airL + glL);
        float dryR = std::tanh(padR + bellR + bassOut + pulseR + choirR + plR + airR + glR);

        // 11. Tape echo (added on top of the dry signal)
        float wetEL, wetER;
        echo.process(dryL, dryR, echoTime, echoFbk, darkAmt, dt, wetEL, wetER);
        float em = std::min(std::max(sEchoMix, 0.f), 1.f);
        float mixL = dryL + wetEL * em * 1.1f;
        float mixR = dryR + wetER * em * 1.1f;
        echoMs = echoTime * 1000.f;

        // 12. Granular cloud, fed by dry + a little echo, added to the mix
        {
            float grL, grR;
            gran.process(dryL + wetEL * em * 0.5f, dryR + wetER * em * 0.5f,
                         std::max(grainSize, 0.01f), std::max(grainDens, 0.1f),
                         ambdsp::clamp01(grainSpray), dt, grL, grR);
            const float gm = std::min(std::max(sGrain, 0.f), 1.f) * 1.6f;
            mixL += grL * gm;
            mixR += grR * gm;
        }

        // 13. Glitch: scheduled on 16th notes, or forced from the trigger input
        {
            bool fire = false;
            const int gstep = (int)std::floor(beatPhase * 4.0);
            if (gstep != lastGlitchStep)
            {
                lastGlitchStep = gstep;
                if (!glitch.active && glitchProb > 0.001f && (time - glitchEnd) > 0.35 && rnd() < glitchProb * 0.25f)
                    fire = true;
            }
            if (glitchTrigIn)
            {
                glitchTrigIn = false;
                if (!glitch.active)
                    fire = true;
            }
            if (fire)
                fireGlitch();
            float gL, gR;
            glitch.process(mixL, mixR, gL, gR);
            mixL = gL;
            mixR = gR;
            if (glitchWas && !glitch.active)
                glitchEnd = time;
            glitchWas = glitch.active;
            glitchActive = glitch.active;
        }

        // 14. Reverb: fed by dry + echoes + grains + glitches
        float rvL, rvR;
        reverb.process(mixL * 0.9f, mixR * 0.9f, reverbDecay, reverbDamp, freeze, dt, rvL, rvR);
        reverbSec = 0.8f * std::pow(40.f, std::min(std::max(reverbDecay, 0.f), 1.f));
        float wm = std::min(std::max(sRevMix, 0.f), 1.f);
        float finL = mixL * (1.f - 0.6f * wm) + rvL * wm * 2.2f;
        float finR = mixR * (1.f - 0.6f * wm) + rvR * wm * 2.2f;

        // 15. Master: two-pole low-pass that follows DARK, DC block, soft limit
        float mfc = std::min(std::max(12000.f * std::pow(0.06f, darkAmt), 500.f), 0.45f * sr);
        float ma = ambdsp::lpCoef(mfc, dt);
        mLp1L += ma * (finL - mLp1L);
        mLp2L += ma * (mLp1L - mLp2L);
        mLp1R += ma * (finR - mLp1R);
        mLp2R += ma * (mLp1R - mLp2R);
        const float rdc = 1.f - twoPi * 12.f * dt;
        dcYL = mLp2L - dcXL + rdc * dcYL;
        dcXL = mLp2L;
        dcYR = mLp2R - dcXR + rdc * dcYR;
        dcXR = mLp2R;
        outL = std::tanh(dcYL * 1.1f);
        outR = std::tanh(dcYR * 1.1f);

        // 16. Meters, spectrum, goniometer
        feedAnalysis(outL, outR, dt);
    }
};

// ------------------------------------------------------------
// Layout constants (36 HP = 540 x 380)
// ------------------------------------------------------------
static const float AMB_W = RACK_GRID_WIDTH * 36; // 540
static const float AMB_H = RACK_GRID_HEIGHT;     // 380

// Display (bezel adds 4 px on every side)
static const float AMB_DX = 20.f, AMB_DY = 24.f, AMB_DW = 500.f, AMB_DH = 124.f;

// Inside the display (local coordinates)
static const float AMB_HDR_H = 18.f;
// Piano roll
static const float AMB_PLOT_L = 30.f;
static const float AMB_PLOT_R = 292.f;
static const float AMB_PLOT_T = 22.f;
static const float AMB_PLOT_B = 92.f;
// Spectrum analyser
static const float AMB_SP_L = 312.f, AMB_SP_R = 466.f, AMB_SP_T = 22.f, AMB_SP_B = 60.f;
// Goniometer
static const float AMB_GO_CX = 330.f, AMB_GO_CY = 86.f, AMB_GO_R = 16.f;
// Grain strip
static const float AMB_GR_L = 360.f, AMB_GR_R = 466.f, AMB_GR_T = 70.f, AMB_GR_B = 86.f;
// Level meters
static const float AMB_MET_X = 476.f; // two meters, 6 px wide, 10 px apart

// Control rows: twelve equal cells spanning x = 10 .. 530
static const float AMB_ROW_X0 = 10.f;
static const float AMB_ROW_W = 520.f;
static const float AMB_CELL_W = 40.f;
static const float AMB_CELL_H = 50.f;
static const float AMB_CTRL_DY = 33.f; // control centre below the cell top

static const float AMB_R1_HDR = 161.f, AMB_R1_TOP = 169.f; // COMPOSE | LAYERS
static const float AMB_R2_HDR = 228.f, AMB_R2_TOP = 236.f; // ECHO | REVERB | GRAINS | MASTER
static const float AMB_R3_HDR = 295.f, AMB_R3_TOP = 303.f; // GLITCH | I / O

static inline float ambCellX(int i)
{
    return AMB_ROW_X0 + AMB_ROW_W / 12.f * ((float)i + 0.5f);
}
static inline float ambCellEdge(int i)
{
    return AMB_ROW_X0 + AMB_ROW_W / 12.f * (float)i;
}
// Row 3 holds thirteen cells (the CLK IN jack was added): 40 px pitch
static const float AMB_R3_CELL_W = 37.f;
static inline float ambCellX13(int i)
{
    return AMB_ROW_X0 + AMB_ROW_W / 13.f * ((float)i + 0.5f);
}
static inline float ambCellEdge13(int i)
{
    return AMB_ROW_X0 + AMB_ROW_W / 13.f * (float)i;
}

// ---------------------------------------------------------------------------
// Param quantities
// ---------------------------------------------------------------------------
struct KeyParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        int v = rack::math::clamp((int)std::round(getValue()), 0, 11);
        return AMB_NOTE_NAMES[v];
    }
};
struct TempoParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        return string::f("%.0f BPM", getValue());
    }
};
struct ChordIntervalParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        return string::f("%.0f s", getValue());
    }
};
struct EchoTimeParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        return string::f("%.0f ms", getValue() * 1000.f);
    }
};
struct GrainSizeParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        return string::f("%.0f ms", getValue() * 1000.f);
    }
};
struct GrainDensParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        return string::f("%.1f /s", getValue());
    }
};
struct GlitchLenParamQuantity : ParamQuantity
{
    std::string getDisplayValueString() override
    {
        float beats = 0.125f * std::pow(8.f, getValue());
        return string::f("%.2f beat", beats);
    }
};

// ============================================================================
// Module
// ============================================================================
struct AMB : Module
{
    // New parameters, inputs and outputs are appended after the original ones
    // so that existing patches keep loading correctly.
    enum ParamId
    {
        KEY_PARAM,
        SCALE_PARAM,
        TEMPO_PARAM,
        PAD_LEVEL_PARAM,
        BELL_LEVEL_PARAM,
        BASS_LEVEL_PARAM,
        PULSE_LEVEL_PARAM,
        ECHO_MIX_PARAM,
        ECHO_FBK_PARAM,
        REVERB_MIX_PARAM,
        REVERB_DECAY_PARAM,
        CHORD_INT_PARAM,
        DARK_PARAM,
        ECHO_TIME_PARAM,
        REVERB_DAMP_PARAM,
        FREEZE_PARAM,
        CHOIR_LEVEL_PARAM,
        PLUCK_LEVEL_PARAM,
        AIR_LEVEL_PARAM,
        GLASS_LEVEL_PARAM,
        GRAIN_MIX_PARAM,
        GRAIN_SIZE_PARAM,
        GRAIN_DENS_PARAM,
        GRAIN_SPRAY_PARAM,
        GLITCH_PROB_PARAM,
        GLITCH_LEN_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        FREEZE_INPUT,
        GLITCH_INPUT,
        CLOCK_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        OUT_L_OUTPUT,
        OUT_R_OUTPUT,
        CHORD_OUTPUT,
        BELL_OUTPUT,
        TRIG_OUTPUT,
        CLOCK_OUTPUT,
        LEVEL_OUTPUT,
        GLITCH_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        FREEZE_LIGHT,
        LIGHTS_LEN
    };

    AmbEngine engine;
    int lastKey = -1;
    int lastScale = -1;
    bool started = false;
    bool frozen = false;
    // External clock
    dsp::SchmittTrigger clockTrig;
    int clockPpb = 1;        // clock pulses per beat (menu)
    bool echoSync = false;   // lock echo time to the clock (menu)
    float clockTimer = 0.f;  // seconds since the last pulse
    float clockPeriod = 0.f; // smoothed pulse period, 0 = unknown
    int extPulseIdx = 0;
    bool clockLocked = false;
    int lastBellCount = 0;
    int lastBeatCount = 0;

    dsp::SchmittTrigger freezeBtn, glitchTrig;
    dsp::PulseGenerator trigPulse, clockPulse;

    AMB()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam<KeyParamQuantity>(KEY_PARAM, 0.f, 11.f, 0.f, "Key");
        paramQuantities[KEY_PARAM]->snapEnabled = true;
        {
            std::vector<std::string> names;
            for (int i = 0; i < AMB_NSCALES; i++)
                names.push_back(AMB_SCALE_NAMES[i]);
            configSwitch(SCALE_PARAM, 0.f, (float)(AMB_NSCALES - 1), 1.f, "Scale", names);
        }
        configParam<TempoParamQuantity>(TEMPO_PARAM, 40.f, 180.f, 72.f, "Tempo");

        configParam(PAD_LEVEL_PARAM, 0.f, 1.f, 0.60f, "Pad level", "%", 0.f, 100.f);
        configParam(BELL_LEVEL_PARAM, 0.f, 1.f, 0.45f, "Bell level", "%", 0.f, 100.f);
        configParam(BASS_LEVEL_PARAM, 0.f, 1.f, 0.50f, "Bass level", "%", 0.f, 100.f);
        configParam(PULSE_LEVEL_PARAM, 0.f, 1.f, 0.25f, "Pulse level", "%", 0.f, 100.f);
        configParam(CHOIR_LEVEL_PARAM, 0.f, 1.f, 0.40f, "Choir level", "%", 0.f, 100.f);
        configParam(PLUCK_LEVEL_PARAM, 0.f, 1.f, 0.35f, "Pluck level", "%", 0.f, 100.f);
        configParam(AIR_LEVEL_PARAM, 0.f, 1.f, 0.30f, "Air level", "%", 0.f, 100.f);
        configParam(GLASS_LEVEL_PARAM, 0.f, 1.f, 0.30f, "Glass level", "%", 0.f, 100.f);

        configParam(ECHO_MIX_PARAM, 0.f, 1.f, 0.45f, "Echo mix", "%", 0.f, 100.f);
        configParam(ECHO_FBK_PARAM, 0.f, 0.9f, 0.60f, "Echo feedback", "%", 0.f, 100.f);
        configParam(REVERB_MIX_PARAM, 0.f, 1.f, 0.60f, "Reverb mix", "%", 0.f, 100.f);
        configParam(REVERB_DECAY_PARAM, 0.f, 1.f, 0.70f, "Reverb decay", "%", 0.f, 100.f);

        configParam<ChordIntervalParamQuantity>(CHORD_INT_PARAM, 4.f, 60.f, 24.f, "Chord change interval");

        configParam(DARK_PARAM, 0.f, 1.f, 0.60f, "Dark (tone)", "%", 0.f, 100.f);
        configParam<EchoTimeParamQuantity>(ECHO_TIME_PARAM, 0.12f, 1.2f, 0.50f, "Echo time");
        configParam(REVERB_DAMP_PARAM, 0.f, 1.f, 0.55f, "Reverb damping", "%", 0.f, 100.f);
        configButton(FREEZE_PARAM, "Freeze reverb");

        configParam(GRAIN_MIX_PARAM, 0.f, 1.f, 0.35f, "Grain mix", "%", 0.f, 100.f);
        configParam<GrainSizeParamQuantity>(GRAIN_SIZE_PARAM, 0.03f, 0.8f, 0.20f, "Grain size");
        configParam<GrainDensParamQuantity>(GRAIN_DENS_PARAM, 1.f, 40.f, 9.f, "Grain density");
        configParam(GRAIN_SPRAY_PARAM, 0.f, 1.f, 0.35f, "Grain spray (pitch scatter)", "%", 0.f, 100.f);

        configParam(GLITCH_PROB_PARAM, 0.f, 1.f, 0.15f, "Glitch probability", "%", 0.f, 100.f);
        configParam<GlitchLenParamQuantity>(GLITCH_LEN_PARAM, 0.f, 1.f, 0.40f, "Glitch slice length");

        configInput(FREEZE_INPUT, "Freeze gate");
        configInput(GLITCH_INPUT, "Glitch trigger");
        configInput(CLOCK_INPUT, "External clock");
        configOutput(OUT_L_OUTPUT, "Left");
        configOutput(OUT_R_OUTPUT, "Right");
        configOutput(CHORD_OUTPUT, "Chord root (1 V/oct)");
        configOutput(BELL_OUTPUT, "Bell pitch (1 V/oct)");
        configOutput(TRIG_OUTPUT, "Bell trigger");
        configOutput(CLOCK_OUTPUT, "Beat clock");
        configOutput(LEVEL_OUTPUT, "Output level");
        configOutput(GLITCH_OUTPUT, "Glitch gate");

        engine.seed(random::u32());
    }

    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        frozen = false;
        clockPeriod = 0.f;
        clockLocked = false;
        lastKey = lastScale = -1; // pick a fresh chord on the next sample
    }

    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "frozen", json_boolean(frozen));
        json_object_set_new(root, "clockPpb", json_integer(clockPpb));
        json_object_set_new(root, "echoSync", json_boolean(echoSync));
        return root;
    }

    void dataFromJson(json_t *root) override
    {
        json_t *f = json_object_get(root, "frozen");
        if (f)
            frozen = json_is_true(f);
        json_t *cp = json_object_get(root, "clockPpb");
        if (cp)
            clockPpb = rack::math::clamp((int)json_integer_value(cp), 1, 48);
        json_t *es = json_object_get(root, "echoSync");
        if (es)
            echoSync = json_is_true(es);
    }

    void process(const ProcessArgs &args) override
    {
        engine.setSampleRate(args.sampleRate);

        int newKey = rack::math::clamp((int)std::round(params[KEY_PARAM].getValue()), 0, 11);
        int newScale = rack::math::clamp((int)std::round(params[SCALE_PARAM].getValue()), 0, AMB_NSCALES - 1);
        if (newKey != lastKey || newScale != lastScale)
        {
            engine.key = newKey;
            engine.scaleIdx = newScale;
            if (!started)
            {
                engine.startChord();
                started = true;
            }
            else
                engine.pickNewChord();
            lastKey = newKey;
            lastScale = newScale;
        }

        if (freezeBtn.process(params[FREEZE_PARAM].getValue(), 0.1f, 1.f))
            frozen = !frozen;
        bool freezeNow = frozen || inputs[FREEZE_INPUT].getVoltage() > 1.f;
        lights[FREEZE_LIGHT].setBrightness(freezeNow ? 1.f : 0.f);

        if (glitchTrig.process(inputs[GLITCH_INPUT].getVoltage(), 0.1f, 1.f))
            engine.glitchTrigIn = true;

        engine.freeze = freezeNow;
        engine.tempoBpm = params[TEMPO_PARAM].getValue();

        // ---- external clock: follow its tempo and lock the beat phase ----
        clockTimer += args.sampleTime;
        if (inputs[CLOCK_INPUT].isConnected())
        {
            if (clockTrig.process(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f))
            {
                const float t = clockTimer;
                clockTimer = 0.f;
                if (t > 0.01f && t < 4.f)
                {
                    if (clockPeriod <= 0.f || std::fabs(t - clockPeriod) > 0.25f * clockPeriod)
                    {
                        clockPeriod = t; // first pulse pair, or a tempo jump
                        extPulseIdx = 0;
                    }
                    else
                        clockPeriod += (t - clockPeriod) * 0.25f;
                    clockLocked = true;
                    if (extPulseIdx % clockPpb == 0)
                    {
                        const double p = engine.beatPhase;
                        engine.extPhaseErr = (float)(std::floor(p + 0.5) - p);
                    }
                    ++extPulseIdx;
                }
                else
                {
                    clockPeriod = 0.f;
                    clockLocked = false;
                }
            }
            if (clockLocked && clockTimer > std::max(2.5f * clockPeriod, 1.f))
            {
                clockLocked = false; // clock stopped: back to the TEMPO knob
                clockPeriod = 0.f;
            }
        }
        else
        {
            clockLocked = false;
            clockPeriod = 0.f;
        }
        engine.extClock = clockLocked;
        if (clockLocked)
            engine.tempoBpm = rack::math::clamp(60.f / (clockPeriod * (float)clockPpb), 20.f, 300.f);
        engine.padLevel = params[PAD_LEVEL_PARAM].getValue();
        engine.bellLevel = params[BELL_LEVEL_PARAM].getValue();
        engine.bassLevel = params[BASS_LEVEL_PARAM].getValue();
        engine.pulseLevel = params[PULSE_LEVEL_PARAM].getValue();
        engine.choirLevel = params[CHOIR_LEVEL_PARAM].getValue();
        engine.pluckLevel = params[PLUCK_LEVEL_PARAM].getValue();
        engine.airLevel = params[AIR_LEVEL_PARAM].getValue();
        engine.glassLevel = params[GLASS_LEVEL_PARAM].getValue();
        engine.echoMix = params[ECHO_MIX_PARAM].getValue();
        engine.echoFbk = params[ECHO_FBK_PARAM].getValue();
        engine.reverbMix = params[REVERB_MIX_PARAM].getValue();
        engine.reverbDecay = params[REVERB_DECAY_PARAM].getValue();
        engine.chordInterval = params[CHORD_INT_PARAM].getValue();
        engine.dark = params[DARK_PARAM].getValue();
        engine.echoTime = params[ECHO_TIME_PARAM].getValue();
        if (echoSync && clockLocked)
        {
            // The TIME knob picks a beat division: 1/16, 1/8, dotted 1/8, 1/4, dotted 1/4, 1/2
            static const float div[6] = {0.25f, 0.5f, 0.75f, 1.f, 1.5f, 2.f};
            const float n = rack::math::clamp((engine.echoTime - 0.12f) / 1.08f, 0.f, 0.9999f);
            float t = div[(int)(n * 6.f)] * 60.f / std::max(engine.tempoBpm, 20.f);
            while (t > 1.2f)
                t *= 0.5f;
            while (t < 0.12f)
                t *= 2.f;
            engine.echoTime = t;
        }
        engine.reverbDamp = params[REVERB_DAMP_PARAM].getValue();
        engine.grainMix = params[GRAIN_MIX_PARAM].getValue();
        engine.grainSize = params[GRAIN_SIZE_PARAM].getValue();
        engine.grainDens = params[GRAIN_DENS_PARAM].getValue();
        engine.grainSpray = params[GRAIN_SPRAY_PARAM].getValue();
        engine.glitchProb = params[GLITCH_PROB_PARAM].getValue();
        engine.glitchLen = params[GLITCH_LEN_PARAM].getValue();

        engine.process(args.sampleTime);

        outputs[OUT_L_OUTPUT].setVoltage(engine.outL * 5.f);
        outputs[OUT_R_OUTPUT].setVoltage(engine.outR * 5.f);

        // Notes are semitones above C3, so C4 = 12 = 0 V
        outputs[CHORD_OUTPUT].setVoltage((engine.chordRoot - 12.f) / 12.f);
        outputs[BELL_OUTPUT].setVoltage((engine.lastBellSemis - 12.f) / 12.f);

        if (engine.bellTrigCount != lastBellCount)
        {
            lastBellCount = engine.bellTrigCount;
            trigPulse.trigger(5e-3f);
        }
        if (engine.beatCount != lastBeatCount)
        {
            lastBeatCount = engine.beatCount;
            clockPulse.trigger(5e-3f);
        }
        outputs[TRIG_OUTPUT].setVoltage(trigPulse.process(args.sampleTime) ? 10.f : 0.f);
        outputs[CLOCK_OUTPUT].setVoltage(clockPulse.process(args.sampleTime) ? 10.f : 0.f);
        outputs[LEVEL_OUTPUT].setVoltage(rack::math::clamp(engine.level * 18.f, 0.f, 10.f));
        outputs[GLITCH_OUTPUT].setVoltage(engine.glitchActive ? 10.f : 0.f);
    }
};

// ============================================================================
// UI (agmm style)
// ============================================================================
namespace
{
    namespace ag = agmm;

    // Note name for semitones above C3 (0 = C3).
    void ambNoteName(int semis, char *buf, size_t n)
    {
        const int midi = 48 + semis;
        const int pc = ((midi % 12) + 12) % 12;
        const int oct = ambdsp::floorDiv(midi, 12) - 1;
        snprintf(buf, n, "%s%d", AMB_NOTE_NAMES[pc], oct);
    }

    NVGcolor ambLayerColour(int layer)
    {
        switch (layer)
        {
        case LAYER_PAD:
            return ag::violet();
        case LAYER_BELL:
            return ag::accent();
        case LAYER_BASS:
            return ag::warm();
        case LAYER_PULSE:
            return nvgRGB(0xa0, 0xa8, 0xb4);
        case LAYER_CHOIR:
            return nvgRGB(0xff, 0x7a, 0xb8);
        case LAYER_PLUCK:
            return nvgRGB(0x7a, 0xf0, 0x9a);
        case LAYER_AIR:
            return nvgRGB(0xb8, 0xdc, 0xff);
        case LAYER_GLASS:
            return nvgRGB(0xff, 0xd8, 0x6a);
        default:
            return nvgRGB(0xff, 0x4d, 0x5e);
        }
    }

    // Section title over a group of cells between x0 and x1.
    void ambGroupHeader(NVGcontext *vg, float x0, float x1, float y, const char *txt)
    {
        nvgSave(vg);
        nvgTranslate(vg, x0, 0.f);
        ag::sectionHeader(vg, x1 - x0, y, txt, 4.f);
        nvgRestore(vg);
    }

    struct AMBPanel : TransparentWidget
    {
        AMBPanel() { box.size = Vec(AMB_W, AMB_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, AMB_W, AMB_H, "AMB");
            ag::bezel(vg, AMB_DX, AMB_DY, AMB_DW, AMB_DH);

            // Row 1: composition | layer levels
            ambGroupHeader(vg, ambCellEdge(0), ambCellEdge(4), AMB_R1_HDR, "COMPOSE");
            ambGroupHeader(vg, ambCellEdge(4), ambCellEdge(12), AMB_R1_HDR, "LAYERS");
            const char *r1[12] = {"KEY", "SCALE", "TEMPO", "CHORD",
                                  "PAD", "BELL", "BASS", "PULSE", "CHOIR", "PLUCK", "AIR", "GLASS"};
            for (int i = 0; i < 12; ++i)
                ag::cell(vg, ambCellX(i), AMB_R1_TOP, AMB_CELL_W, AMB_CELL_H, r1[i]);

            // Row 2: echo | reverb | grains | master
            ambGroupHeader(vg, ambCellEdge(0), ambCellEdge(3), AMB_R2_HDR, "ECHO");
            ambGroupHeader(vg, ambCellEdge(3), ambCellEdge(6), AMB_R2_HDR, "REVERB");
            ambGroupHeader(vg, ambCellEdge(6), ambCellEdge(10), AMB_R2_HDR, "GRAINS");
            ambGroupHeader(vg, ambCellEdge(10), ambCellEdge(12), AMB_R2_HDR, "MASTER");
            const char *r2[12] = {"TIME", "FBK", "MIX", "DECAY", "DAMP", "MIX",
                                  "MIX", "SIZE", "DENS", "SPRAY", "DARK", "FREEZE"};
            for (int i = 0; i < 12; ++i)
                ag::cell(vg, ambCellX(i), AMB_R2_TOP, AMB_CELL_W, AMB_CELL_H, r2[i]);

            // Row 3: glitch | inputs and outputs (outputs tinted)
            ambGroupHeader(vg, ambCellEdge13(0), ambCellEdge13(2), AMB_R3_HDR, "GLITCH");
            ambGroupHeader(vg, ambCellEdge13(2), ambCellEdge13(13), AMB_R3_HDR, "I / O");
            const char *r3[13] = {"PROB", "LEN", "GLT IN", "FRZ IN", "CLK IN", "OUT L", "OUT R",
                                  "CHORD", "BELL", "TRIG", "CLOCK", "GLITCH", "LEVEL"};
            for (int i = 0; i < 13; ++i)
                ag::cell(vg, ambCellX13(i), AMB_R3_TOP, AMB_R3_CELL_W, AMB_CELL_H, r3[i], i >= 5);

            Widget::draw(args);
        }
    };

    struct AMBDisplay : TransparentWidget
    {
        AMB *module = nullptr;
        AMBDisplay(AMB *m) : module(m)
        {
            box.pos = Vec(AMB_DX, AMB_DY);
            box.size = Vec(AMB_DW, AMB_DH);
        }

        static float pitchY(float p)
        {
            float n = (p - AMB_PITCH_MIN) / (AMB_PITCH_MAX - AMB_PITCH_MIN);
            n = rack::math::clamp(n, 0.f, 1.f);
            return AMB_PLOT_B - n * (AMB_PLOT_B - AMB_PLOT_T);
        }
        // x position for a time `age` seconds in the past
        static float ageX(double age)
        {
            float a = rack::math::clamp((float)(age / (double)AMB_HIST_WIN), 0.f, 1.f);
            return AMB_PLOT_R - a * (AMB_PLOT_R - AMB_PLOT_L);
        }
        // x position of a frequency in the spectrum plot (30 Hz .. 16 kHz)
        static float freqX(float f)
        {
            float n = std::log(f / 30.f) / std::log(16000.f / 30.f);
            return AMB_SP_L + 1.f + rack::math::clamp(n, 0.f, 1.f) * (AMB_SP_R - AMB_SP_L - 2.f);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, AMB_DW, AMB_DH);

            // Hairlines under the header and above the legend row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, AMB_HDR_H + 0.5f);
            nvgLineTo(vg, AMB_DW - 8.f, AMB_HDR_H + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, 105.5f);
            nvgLineTo(vg, AMB_DW - 8.f, 105.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // ---------- piano roll frame, grids and labels ----------
            ag::plotFrame(vg, AMB_PLOT_L, AMB_PLOT_T, AMB_PLOT_R - AMB_PLOT_L, AMB_PLOT_B - AMB_PLOT_T);

            // Octave grid: C2 .. C7 (C3 = 0 semitones)
            static const char *octTxt[6] = {"C2", "C3", "C4", "C5", "C6", "C7"};
            nvgBeginPath(vg);
            for (int i = 0; i < 6; ++i)
            {
                float y = std::floor(pitchY((float)(-12 + 12 * i))) + 0.5f;
                nvgMoveTo(vg, AMB_PLOT_L + 1.f, y);
                nvgLineTo(vg, AMB_PLOT_R - 1.f, y);
            }
            // Time grid every 5 s
            for (int k = 1; k < 4; ++k)
            {
                float x = std::floor(ageX(5.0 * k)) + 0.5f;
                nvgMoveTo(vg, x, AMB_PLOT_T + 1.f);
                nvgLineTo(vg, x, AMB_PLOT_B - 1.f);
            }
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            for (int i = 0; i < 6; ++i)
                ag::text(vg, AMB_PLOT_L - 4.f, pitchY((float)(-12 + 12 * i)), octTxt[i], 7.f, ag::dim(), ag::ALIGN_R, 0.f);

            static const char *timeTxt[5] = {"20S", "15", "10", "5", "NOW"};
            for (int k = 0; k < 5; ++k)
                ag::text(vg, ageX(20.0 - 5.0 * k), AMB_PLOT_B + 8.f, timeTxt[k], 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            // ---------- spectrum frame and grid ----------
            ag::plotFrame(vg, AMB_SP_L, AMB_SP_T, AMB_SP_R - AMB_SP_L, AMB_SP_B - AMB_SP_T);
            nvgBeginPath(vg);
            for (int i = 1; i < 4; ++i)
            {
                float y = std::floor(AMB_SP_T + (AMB_SP_B - AMB_SP_T) * (float)i * 0.25f) + 0.5f;
                nvgMoveTo(vg, AMB_SP_L + 1.f, y);
                nvgLineTo(vg, AMB_SP_R - 1.f, y);
            }
            static const float gridF[7] = {100.f, 200.f, 500.f, 1000.f, 2000.f, 5000.f, 10000.f};
            for (int i = 0; i < 7; ++i)
            {
                float x = std::floor(freqX(gridF[i])) + 0.5f;
                nvgMoveTo(vg, x, AMB_SP_T + 1.f);
                nvgLineTo(vg, x, AMB_SP_B - 1.f);
            }
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 12));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            ag::text(vg, freqX(100.f), AMB_SP_B + 6.f, "100", 6.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, freqX(1000.f), AMB_SP_B + 6.f, "1K", 6.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, freqX(10000.f), AMB_SP_B + 6.f, "10K", 6.f, ag::dim(), ag::ALIGN_C, 0.f);

            // ---------- goniometer frame ----------
            nvgBeginPath(vg);
            nvgCircle(vg, AMB_GO_CX, AMB_GO_CY, AMB_GO_R);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 36));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            nvgBeginPath(vg);
            nvgMoveTo(vg, AMB_GO_CX - AMB_GO_R, AMB_GO_CY);
            nvgLineTo(vg, AMB_GO_CX + AMB_GO_R, AMB_GO_CY);
            nvgMoveTo(vg, AMB_GO_CX, AMB_GO_CY - AMB_GO_R);
            nvgLineTo(vg, AMB_GO_CX, AMB_GO_CY + AMB_GO_R);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 14));
            nvgStroke(vg);
            ag::text(vg, AMB_GO_CX - AMB_GO_R - 4.f, AMB_GO_CY, "L", 6.f, ag::dim(), ag::ALIGN_R, 0.f);
            ag::text(vg, AMB_GO_CX + AMB_GO_R + 4.f, AMB_GO_CY, "R", 6.f, ag::dim(), ag::ALIGN_L, 0.f);

            // ---------- grain strip frame ----------
            ag::plotFrame(vg, AMB_GR_L, AMB_GR_T, AMB_GR_R - AMB_GR_L, AMB_GR_B - AMB_GR_T);
            ag::text(vg, AMB_GR_L + 3.f, AMB_GR_T - 5.f, "", 6.f, ag::dim(), ag::ALIGN_L, 0.3f);

            // ---------- meters labels ----------
            ag::text(vg, AMB_MET_X + 3.f, AMB_PLOT_B + 8.f, "L", 7.f, ag::dim(), ag::ALIGN_C, 0.f);
            ag::text(vg, AMB_MET_X + 13.f, AMB_PLOT_B + 8.f, "R", 7.f, ag::dim(), ag::ALIGN_C, 0.f);

            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            // -------- snapshot --------
            int key = 0, scale = 1, beatCount = 0, nGrains = 0;
            float tempo = 72.f, chordProg = 0.f, mL = 0.f, mR = 0.f, pkL = 0.f, pkR = 0.f;
            float echoMs = 500.f, revSec = 8.f, corr = 0.f;
            bool frozen = false, glitchOn = false, extClk = false;
            double now = 0.0;
            float chordNotes[4] = {0.f, 3.f, 7.f, 14.f};
            const AmbEngine::NoteEvent *hist = nullptr;
            int histN = 0;
            const float *spec = nullptr, *specPk = nullptr;
            const float *gonL = nullptr, *gonR = nullptr;
            int gonPos = 0;
            const ambdsp::Granular::Grain *grains = nullptr;
            float grainBuf = 192000.f;

            AmbEngine::NoteEvent demo[48];
            float demoSpec[AMB_NSPEC], demoPk[AMB_NSPEC];
            float demoGonL[AmbEngine::GON_N], demoGonR[AmbEngine::GON_N];
            ambdsp::Granular::Grain demoGr[ambdsp::Granular::NG];

            if (module)
            {
                const AmbEngine &e = module->engine;
                key = e.key;
                scale = e.scaleIdx;
                tempo = e.tempoBpm;
                extClk = e.extClock;
                // chord = e.chordRoot;
                chordProg = e.chordProgress;
                mL = e.meterL;
                mR = e.meterR;
                pkL = e.peakL;
                pkR = e.peakR;
                echoMs = e.echoMs;
                revSec = e.reverbSec;
                frozen = e.freeze;
                now = e.time;
                beatCount = e.beatCount;
                corr = e.corr;
                glitchOn = e.glitchActive;
                // glitchType = e.glitchType;
                for (int k = 0; k < 4; ++k)
                    chordNotes[k] = e.pad[e.padCur].note[k];
                hist = e.hist;
                histN = AmbEngine::HIST_MAX;
                spec = e.spec;
                specPk = e.specPk;
                gonL = e.gonL;
                gonR = e.gonR;
                gonPos = e.gonPos;
                grains = e.gran.g;
                grainBuf = (float)std::max(8, e.gran.size);
                nGrains = e.gran.active;
            }
            else
            {
                // Static preview for the module browser
                now = 20.0;
                tempo = 72.f;
                // chord = 0.f;
                chordProg = 0.62f;
                beatCount = 37;
                mL = 0.35f;
                mR = 0.30f;
                pkL = 0.5f;
                pkR = 0.45f;
                corr = 0.78f;
                glitchOn = true;
                // glitchType = 0;
                nGrains = 9;
                int n = 0;
                const float pads[4] = {0.f, 3.f, 7.f, 10.f};
                for (int k = 0; k < 4; ++k)
                    demo[n++] = AmbEngine::NoteEvent{4.0, -1.0, pads[k], LAYER_PAD};
                for (int k = 0; k < 4; ++k)
                    demo[n++] = AmbEngine::NoteEvent{0.0, 4.0, pads[k] + 5.f, LAYER_PAD};
                for (int k = 0; k < 4; ++k)
                    demo[n++] = AmbEngine::NoteEvent{4.0, -1.0, pads[k] + 12.f, LAYER_CHOIR};
                for (int k = 0; k < 4; ++k)
                    demo[n++] = AmbEngine::NoteEvent{4.0, -1.0, pads[k] + 24.f, LAYER_GLASS};
                demo[n++] = AmbEngine::NoteEvent{4.0, 16.0, -9.f, LAYER_BASS};
                for (int k = 0; k < 8; ++k)
                    demo[n++] = AmbEngine::NoteEvent{3.0 + 2.2 * k, -1.0, 15.f + 3.f * (float)(k % 5), LAYER_BELL};
                for (int k = 0; k < 7; ++k)
                    demo[n++] = AmbEngine::NoteEvent{1.0 + 2.7 * k, 1.0 + 2.7 * k, 0.f, LAYER_PULSE};
                for (int k = 0; k < 6; ++k)
                    demo[n++] = AmbEngine::NoteEvent{2.0 + 3.1 * k, 2.0 + 3.1 * k, 27.f + 4.f * (float)(k % 4), LAYER_PLUCK};
                demo[n++] = AmbEngine::NoteEvent{6.0, 15.0, 38.f, LAYER_AIR};
                demo[n++] = AmbEngine::NoteEvent{12.0, 13.4, 0.f, LAYER_GLITCH};
                demo[n++] = AmbEngine::NoteEvent{17.5, 18.1, 3.f, LAYER_GLITCH};
                hist = demo;
                histN = n;
                for (int b = 0; b < AMB_NSPEC; ++b)
                {
                    float x = (float)b / (float)(AMB_NSPEC - 1);
                    float v = 0.85f * std::exp(-x * 2.6f) * (0.65f + 0.35f * std::sin(x * 23.f)) + 0.08f;
                    demoSpec[b] = rack::math::clamp(v, 0.f, 1.f);
                    demoPk[b] = rack::math::clamp(v + 0.06f, 0.f, 1.f);
                }
                spec = demoSpec;
                specPk = demoPk;
                for (int i = 0; i < AmbEngine::GON_N; ++i)
                {
                    float p = (float)i * 0.21f;
                    demoGonL[i] = 0.32f * std::sin(p) + 0.12f * std::sin(p * 2.3f);
                    demoGonR[i] = 0.28f * std::sin(p + 0.7f) + 0.12f * std::sin(p * 1.7f);
                }
                gonL = demoGonL;
                gonR = demoGonR;
                gonPos = 0;
                for (int i = 0; i < ambdsp::Granular::NG; ++i)
                    demoGr[i].on = false;
                const float dists[9] = {0.05f, 0.12f, 0.2f, 0.33f, 0.41f, 0.5f, 0.62f, 0.75f, 0.9f};
                const float ratios[9] = {1.f, 2.f, 0.5f, 1.f, 1.5f, -1.f, 1.f, 2.f, 0.75f};
                for (int i = 0; i < 9; ++i)
                {
                    demoGr[i].on = true;
                    demoGr[i].dist = dists[i] * grainBuf;
                    demoGr[i].ratio = ratios[i];
                    demoGr[i].len = 1000;
                    demoGr[i].age = 140 + i * 90;
                    demoGr[i].amp = 0.8f;
                }
                grains = demoGr;
            }
            key = rack::math::clamp(key, 0, 11);
            scale = rack::math::clamp(scale, 0, AMB_NSCALES - 1);

            // -------- scale rows in the piano roll --------
            {
                bool inScale[12] = {};
                for (int i = 0; i < AMB_SCALE_LEN[scale]; ++i)
                    inScale[(key + AMB_SCALE_INTERVALS[scale][i]) % 12] = true;
                nvgBeginPath(vg);
                for (int s = (int)AMB_PITCH_MIN; s <= (int)AMB_PITCH_MAX; ++s)
                {
                    const int pc = ((s % 12) + 12) % 12;
                    if (!inScale[pc] || pc == key)
                        continue;
                    const float y = std::floor(pitchY((float)s)) + 0.5f;
                    nvgMoveTo(vg, AMB_PLOT_L + 1.f, y);
                    nvgLineTo(vg, AMB_PLOT_R - 1.f, y);
                }
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 9));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                for (int s = (int)AMB_PITCH_MIN; s <= (int)AMB_PITCH_MAX; ++s)
                {
                    if ((((s % 12) + 12) % 12) != key)
                        continue;
                    const float y = std::floor(pitchY((float)s)) + 0.5f;
                    nvgMoveTo(vg, AMB_PLOT_L + 1.f, y);
                    nvgLineTo(vg, AMB_PLOT_R - 1.f, y);
                }
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 34));
                nvgStroke(vg);
            }

            // -------- events --------
            nvgSave(vg);
            nvgScissor(vg, AMB_PLOT_L, AMB_PLOT_T, AMB_PLOT_R - AMB_PLOT_L, AMB_PLOT_B - AMB_PLOT_T);
            // Three passes: glitch bands and air washes behind, sustained bars, then points
            for (int pass = 0; pass < 3; ++pass)
            {
                for (int i = 0; i < histN; ++i)
                {
                    const AmbEngine::NoteEvent &ev = hist[i];
                    if (ev.born < -999.0)
                        continue;
                    const double age = now - ev.born;
                    if (age < 0.0)
                        continue;
                    const double endT = (ev.end < 0.0) ? now : ev.end;
                    if (now - endT > (double)AMB_HIST_WIN)
                        continue;
                    const int lay = rack::math::clamp(ev.layer, 0, (int)LAYER_COUNT - 1);
                    const bool point = (lay == LAYER_BELL || lay == LAYER_PULSE || lay == LAYER_PLUCK);
                    if (point && age > (double)AMB_HIST_WIN)
                        continue;
                    const bool back = (lay == LAYER_GLITCH || lay == LAYER_AIR);
                    const bool sustained = (lay == LAYER_PAD || lay == LAYER_BASS || lay == LAYER_CHOIR || lay == LAYER_GLASS);
                    if (pass == 1 && !sustained)
                        continue;
                    if (pass == 2 && !point)
                        continue;
                    if (pass == 0 && !back)
                        continue;

                    const float fadeAge = point ? (float)age : (float)(now - endT);
                    const float fade = rack::math::clamp(1.f - fadeAge / AMB_HIST_WIN * 0.7f, 0.2f, 1.f);
                    const float x0 = ageX(age);
                    const float x1 = ageX(now - endT);
                    const NVGcolor col = ambLayerColour(lay);

                    if (lay == LAYER_GLITCH)
                    {
                        const float w = std::max(x1 - x0, 2.f);
                        nvgBeginPath(vg);
                        nvgRect(vg, x0, AMB_PLOT_T, w, AMB_PLOT_B - AMB_PLOT_T);
                        nvgFillColor(vg, ag::withA(col, 0.16f * fade));
                        nvgFill(vg);
                        nvgBeginPath(vg);
                        nvgRect(vg, x0, AMB_PLOT_T, 1.f, AMB_PLOT_B - AMB_PLOT_T);
                        nvgFillColor(vg, ag::withA(col, 0.75f * fade));
                        nvgFill(vg);
                        static const char *tag[5] = {"S", "R", "C", "T", "D"};
                        const int ti = rack::math::clamp((int)ev.pitch, 0, 4);
                        ag::text(vg, x0 + 4.f, AMB_PLOT_T + 6.f, tag[ti], 6.f, ag::withA(col, 0.9f * fade), ag::ALIGN_L, 0.f);
                    }
                    else if (lay == LAYER_AIR)
                    {
                        // Wind swell: soft band that fades in and out over its length
                        const float y = pitchY(ev.pitch);
                        const float w = std::max(x1 - x0, 1.f);
                        NVGpaint pt = nvgLinearGradient(vg, x0, 0.f, x0 + w, 0.f,
                                                        ag::withA(col, 0.f), ag::withA(col, 0.30f * fade));
                        nvgBeginPath(vg);
                        nvgRect(vg, x0, y - 4.f, w, 8.f);
                        nvgFillPaint(vg, pt);
                        nvgFill(vg);
                    }
                    else if (sustained)
                    {
                        const float h = (lay == LAYER_BASS) ? 4.f : (lay == LAYER_PAD ? 3.f : 2.f);
                        const float y = pitchY(ev.pitch);
                        nvgBeginPath(vg);
                        nvgRect(vg, x0, y - h * 0.5f, std::max(x1 - x0, 1.f), h);
                        nvgFillColor(vg, ag::withA(col, (lay == LAYER_GLASS ? 0.60f : 0.80f) * fade));
                        nvgFill(vg);
                    }
                    else if (lay == LAYER_BELL)
                    {
                        nvgBeginPath(vg);
                        nvgCircle(vg, x0, pitchY(ev.pitch), 2.6f);
                        nvgFillColor(vg, ag::withA(col, 0.90f * fade));
                        nvgFill(vg);
                    }
                    else if (lay == LAYER_PLUCK)
                    {
                        // Plucks: small diamonds with a decay tail
                        const float y = pitchY(ev.pitch);
                        nvgBeginPath(vg);
                        nvgMoveTo(vg, x0, y - 2.4f);
                        nvgLineTo(vg, x0 + 2.4f, y);
                        nvgLineTo(vg, x0, y + 2.4f);
                        nvgLineTo(vg, x0 - 2.4f, y);
                        nvgClosePath(vg);
                        nvgFillColor(vg, ag::withA(col, 0.95f * fade));
                        nvgFill(vg);
                        nvgBeginPath(vg);
                        nvgRect(vg, x0 + 2.f, y - 0.5f, std::min(14.f, AMB_PLOT_R - x0 - 2.f), 1.f);
                        nvgFillColor(vg, ag::withA(col, 0.30f * fade));
                        nvgFill(vg);
                    }
                    else
                    {
                        // Pulses are short ticks along the bottom edge
                        nvgBeginPath(vg);
                        nvgRect(vg, x0 - 0.5f, AMB_PLOT_B - 6.f, 1.f, 5.f);
                        nvgFillColor(vg, ag::withA(col, 0.85f * fade));
                        nvgFill(vg);
                    }
                }
            }
            // NOW cursor with a soft glow
            {
                NVGpaint glow = nvgLinearGradient(vg, AMB_PLOT_R - 10.f, 0.f, AMB_PLOT_R, 0.f,
                                                  nvgRGBA(0x3d, 0xe0, 0xff, 0), nvgRGBA(0x3d, 0xe0, 0xff, 34));
                nvgBeginPath(vg);
                nvgRect(vg, AMB_PLOT_R - 10.f, AMB_PLOT_T, 10.f, AMB_PLOT_B - AMB_PLOT_T);
                nvgFillPaint(vg, glow);
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRect(vg, AMB_PLOT_R - 1.f, AMB_PLOT_T, 1.f, AMB_PLOT_B - AMB_PLOT_T);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 150));
                nvgFill(vg);
            }
            nvgRestore(vg);

            // -------- header --------
            char buf[96];
            snprintf(buf, sizeof(buf), "%s %s", AMB_NOTE_NAMES[key], AMB_SCALE_NAMES[scale]);
            ag::text(vg, 8.f, AMB_HDR_H * 0.5f, buf, 8.f, ag::accent(), ag::ALIGN_L, 0.5f);

            snprintf(buf, sizeof(buf), "%.0f BPM", tempo);
            ag::text(vg, 128.f, AMB_HDR_H * 0.5f, buf, 8.f, extClk ? ag::accent() : ag::dim(), ag::ALIGN_L, 0.5f);
            if (extClk)
                ag::text(vg, 128.f + ag::textWidth(vg, buf, 8.f, 0.5f) + 4.f, AMB_HDR_H * 0.5f, "EXT", 6.f, ag::accent(), ag::ALIGN_L, 0.5f);

            snprintf(buf, sizeof(buf), "BAR %03d.%d", beatCount / 4 + 1, beatCount % 4 + 1);
            ag::text(vg, 184.f, AMB_HDR_H * 0.5f, buf, 8.f, ag::dim(), ag::ALIGN_L, 0.5f);

            {
                char a[4][8];
                for (int k = 0; k < 4; ++k)
                    ambNoteName((int)std::round(chordNotes[k]), a[k], sizeof(a[k]));
                snprintf(buf, sizeof(buf), "%s  %s  %s  %s", a[0], a[1], a[2], a[3]);
                ag::text(vg, 404.f - 4.f, AMB_HDR_H * 0.5f, buf, 8.f, ag::bright(), ag::ALIGN_R, 0.5f);
            }
            ag::chip(vg, AMB_DW - 8.f - 40.f, 3.f, 40.f, 12.f, "GLITCH", glitchOn, ambLayerColour(LAYER_GLITCH));
            ag::chip(vg, AMB_DW - 8.f - 40.f - 4.f - 40.f, 3.f, 40.f, 12.f, "FREEZE", frozen, ag::accent());

            // Chord timer: thin progress line along the header hairline
            {
                const float x0 = 8.f, x1 = AMB_DW - 8.f;
                const float p = rack::math::clamp(chordProg, 0.f, 1.f);
                nvgBeginPath(vg);
                nvgRect(vg, x0, AMB_HDR_H, (x1 - x0) * p, 1.f);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 170));
                nvgFill(vg);
            }

            // -------- spectrum --------
            {
                const float bw = (AMB_SP_R - AMB_SP_L - 2.f) / (float)AMB_NSPEC;
                const float ph = AMB_SP_B - AMB_SP_T - 2.f;
                NVGpaint pt = nvgLinearGradient(vg, 0.f, AMB_SP_B, 0.f, AMB_SP_T,
                                                ag::withA(ag::violet(), 0.55f), ag::withA(ag::accent(), 0.95f));
                nvgBeginPath(vg);
                for (int b = 0; b < AMB_NSPEC; ++b)
                {
                    const float h = rack::math::clamp(spec[b], 0.f, 1.f) * ph;
                    if (h < 0.5f)
                        continue;
                    nvgRect(vg, AMB_SP_L + 1.f + (float)b * bw + 0.5f, AMB_SP_B - 1.f - h, bw - 1.f, h);
                }
                nvgFillPaint(vg, pt);
                nvgFill(vg);
                nvgBeginPath(vg);
                for (int b = 0; b < AMB_NSPEC; ++b)
                {
                    const float h = rack::math::clamp(specPk[b], 0.f, 1.f) * ph;
                    if (h < 1.f)
                        continue;
                    nvgRect(vg, AMB_SP_L + 1.f + (float)b * bw + 0.5f, AMB_SP_B - 2.f - h, bw - 1.f, 1.f);
                }
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 150));
                nvgFill(vg);
                ag::text(vg, AMB_SP_L + 4.f, AMB_SP_T + 6.f, "SPECTRUM", 6.f, ag::dim(), ag::ALIGN_L, 0.3f);
            }

            // -------- goniometer --------
            {
                nvgSave(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, AMB_GO_CX, AMB_GO_CY, AMB_GO_R);
                nvgScissor(vg, AMB_GO_CX - AMB_GO_R, AMB_GO_CY - AMB_GO_R, AMB_GO_R * 2.f, AMB_GO_R * 2.f);
                const int N = AmbEngine::GON_N;
                for (int third = 0; third < 3; ++third)
                {
                    nvgBeginPath(vg);
                    for (int i = third * (N / 3); i < (third + 1) * (N / 3); ++i)
                    {
                        const int idx = (gonPos + i) % N; // oldest first
                        const float l = gonL[idx], r = gonR[idx];
                        float px = (r - l) * 0.7071f * AMB_GO_R * 1.8f;
                        float py = -(l + r) * 0.7071f * AMB_GO_R * 1.8f;
                        const float d = std::sqrt(px * px + py * py);
                        if (d > AMB_GO_R - 1.f)
                        {
                            px *= (AMB_GO_R - 1.f) / d;
                            py *= (AMB_GO_R - 1.f) / d;
                        }
                        nvgRect(vg, AMB_GO_CX + px - 0.5f, AMB_GO_CY + py - 0.5f, 1.2f, 1.2f);
                    }
                    const float a = (third == 0) ? 0.22f : (third == 1 ? 0.45f : 0.80f);
                    nvgFillColor(vg, ag::withA(ag::accent(), a));
                    nvgFill(vg);
                }
                nvgRestore(vg);
            }

            // -------- grain strip --------
            {
                const float gw = AMB_GR_R - AMB_GR_L - 2.f;
                const float gh = AMB_GR_B - AMB_GR_T - 2.f;
                for (int i = 0; i < ambdsp::Granular::NG; ++i)
                {
                    const ambdsp::Granular::Grain &g = grains[i];
                    if (!g.on)
                        continue;
                    const float ph = (float)g.age / (float)std::max(1, g.len);
                    const float xx = ph * 2.f - 1.f;
                    float win = 1.f - xx * xx;
                    win *= win;
                    const float x = AMB_GR_R - 1.f - rack::math::clamp(g.dist / grainBuf, 0.f, 1.f) * gw;
                    const float h = std::max(1.f, win * g.amp * gh);
                    NVGcolor c;
                    if (g.ratio < 0.f)
                        c = ambLayerColour(LAYER_GLITCH);
                    else if (g.ratio > 1.05f)
                        c = ag::accent();
                    else if (g.ratio < 0.95f)
                        c = ag::warm();
                    else
                        c = ag::violet();
                    nvgBeginPath(vg);
                    nvgRect(vg, x - 1.f, AMB_GR_B - 1.f - h, 2.f, h);
                    nvgFillColor(vg, ag::withA(c, 0.9f));
                    nvgFill(vg);
                }
                // "now" edge
                nvgBeginPath(vg);
                nvgRect(vg, AMB_GR_R - 1.f, AMB_GR_T, 1.f, AMB_GR_B - AMB_GR_T);
                nvgFillColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 120));
                nvgFill(vg);

                snprintf(buf, sizeof(buf), "GRAINS %d", nGrains);
                ag::text(vg, AMB_GR_L, 95.f, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.3f);

                // Stereo correlation bar
                const float cx0 = 400.f, cw = 66.f, cy = 100.f;
                ag::text(vg, AMB_GR_L, 101.f, "CORR", 6.f, ag::dim(), ag::ALIGN_L, 0.3f);
                nvgBeginPath(vg);
                nvgRect(vg, cx0, cy, cw, 3.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 20));
                nvgFill(vg);
                const float c = rack::math::clamp(corr, -1.f, 1.f);
                nvgBeginPath(vg);
                if (c >= 0.f)
                    nvgRect(vg, cx0 + cw * 0.5f, cy, cw * 0.5f * c, 3.f);
                else
                    nvgRect(vg, cx0 + cw * 0.5f + cw * 0.5f * c, cy, -cw * 0.5f * c, 3.f);
                nvgFillColor(vg, c >= 0.f ? ag::withA(ag::accent(), 0.9f) : ag::withA(ambLayerColour(LAYER_GLITCH), 0.9f));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRect(vg, cx0 + cw * 0.5f - 0.5f, cy - 1.f, 1.f, 5.f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 90));
                nvgFill(vg);
            }

            // -------- meters with peak hold --------
            {
                const float mh = AMB_PLOT_B - AMB_PLOT_T;
                ag::meter(vg, AMB_MET_X, AMB_PLOT_T, 6.f, mh, mL * 1.8f);
                ag::meter(vg, AMB_MET_X + 10.f, AMB_PLOT_T, 6.f, mh, mR * 1.8f);
                const float pk[2] = {pkL, pkR};
                nvgBeginPath(vg);
                for (int c = 0; c < 2; ++c)
                {
                    const float v = rack::math::clamp(pk[c] * 1.8f, 0.f, 1.f);
                    nvgRect(vg, AMB_MET_X + 10.f * (float)c, AMB_PLOT_B - v * mh - 1.f, 6.f, 1.f);
                }
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 200));
                nvgFill(vg);
            }

            // -------- legend and delay times --------
            {
                static const char *names[LAYER_COUNT] = {"PAD", "BELL", "BASS", "PULS", "CHOR", "PLCK", "AIR", "GLAS", "GLCH"};
                for (int i = 0; i < (int)LAYER_COUNT; ++i)
                {
                    const float lx = 8.f + 31.f * (float)i;
                    nvgBeginPath(vg);
                    nvgRect(vg, lx, 112.f, 6.f, 6.f);
                    nvgFillColor(vg, ambLayerColour(i));
                    nvgFill(vg);
                    ag::text(vg, lx + 9.f, 115.f, names[i], 6.f, ag::dim(), ag::ALIGN_L, 0.2f);
                }
                snprintf(buf, sizeof(buf), "ECHO %.0f MS   REV %.1f S", echoMs, revSec);
                ag::text(vg, AMB_DW - 8.f, 115.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.3f);
            }

            Widget::drawLayer(args, layer);
        }
    };
} // namespace

// ============================================================================
// Widget
// ============================================================================
struct AMBWidget : ModuleWidget
{
    AMBWidget(AMB *module)
    {
        setModule(module);
        box.size = Vec(AMB_W, AMB_H);

        addChild(new AMBPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new AMBDisplay(module));

        // Row 1: KEY, SCALE, TEMPO, CHORD | PAD, BELL, BASS, PULSE, CHOIR, PLUCK, AIR, GLASS
        const float y1 = AMB_R1_TOP + AMB_CTRL_DY;
        static const int row1[12] = {
            AMB::KEY_PARAM, AMB::SCALE_PARAM, AMB::TEMPO_PARAM, AMB::CHORD_INT_PARAM,
            AMB::PAD_LEVEL_PARAM, AMB::BELL_LEVEL_PARAM, AMB::BASS_LEVEL_PARAM, AMB::PULSE_LEVEL_PARAM,
            AMB::CHOIR_LEVEL_PARAM, AMB::PLUCK_LEVEL_PARAM, AMB::AIR_LEVEL_PARAM, AMB::GLASS_LEVEL_PARAM};
        for (int i = 0; i < 12; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(i), y1), module, row1[i]));

        // Row 2: echo TIME, FBK, MIX | reverb DECAY, DAMP, MIX | grain MIX, SIZE, DENS, SPRAY | DARK, FREEZE
        const float y2 = AMB_R2_TOP + AMB_CTRL_DY;
        static const int row2[11] = {
            AMB::ECHO_TIME_PARAM, AMB::ECHO_FBK_PARAM, AMB::ECHO_MIX_PARAM,
            AMB::REVERB_DECAY_PARAM, AMB::REVERB_DAMP_PARAM, AMB::REVERB_MIX_PARAM,
            AMB::GRAIN_MIX_PARAM, AMB::GRAIN_SIZE_PARAM, AMB::GRAIN_DENS_PARAM, AMB::GRAIN_SPRAY_PARAM,
            AMB::DARK_PARAM};
        for (int i = 0; i < 11; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(i), y2), module, row2[i]));
        addParam(createParamCentered<LEDButton>(Vec(ambCellX(11), y2), module, AMB::FREEZE_PARAM));
        addChild(createLightCentered<MediumLight<agmm::CyanLight>>(Vec(ambCellX(11), y2), module, AMB::FREEZE_LIGHT));

        // Row 3: glitch PROB, LEN | GLT IN, FRZ IN | OUT L, OUT R, CHORD, BELL, TRIG, CLOCK, GLITCH, LEVEL
        const float y3 = AMB_R3_TOP + AMB_CTRL_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX13(0), y3), module, AMB::GLITCH_PROB_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX13(1), y3), module, AMB::GLITCH_LEN_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(ambCellX13(2), y3), module, AMB::GLITCH_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(ambCellX13(3), y3), module, AMB::FREEZE_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(ambCellX13(4), y3), module, AMB::CLOCK_INPUT));
        static const int outs3[8] = {AMB::OUT_L_OUTPUT, AMB::OUT_R_OUTPUT, AMB::CHORD_OUTPUT, AMB::BELL_OUTPUT,
                                     AMB::TRIG_OUTPUT, AMB::CLOCK_OUTPUT, AMB::GLITCH_OUTPUT, AMB::LEVEL_OUTPUT};
        for (int i = 0; i < 8; ++i)
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ambCellX13(5 + i), y3), module, outs3[i]));
    }

    void appendContextMenu(Menu *menu) override
    {
        ModuleWidget::appendContextMenu(menu);
        AMB *m = dynamic_cast<AMB *>(module);
        if (!m)
            return;
        menu->addChild(new MenuSeparator);
        menu->addChild(createSubmenuItem("Clock pulses per beat", std::to_string(m->clockPpb), [=](Menu *sub)
                                         {
            static const int opts[7] = {1, 2, 4, 8, 12, 16, 24};
            for (int i = 0; i < 7; ++i)
            {
                const int v = opts[i];
                sub->addChild(createCheckMenuItem(std::to_string(v), "", [=]() { return m->clockPpb == v; },
                                                  [=]() { m->clockPpb = v; m->extPulseIdx = 0; }));
            } }));
        menu->addChild(createBoolPtrMenuItem("Sync echo time to clock (TIME picks the division)", "", &m->echoSync));
    }
};

Model *modelAMB = createModel<AMB, AMBWidget>("AMB");