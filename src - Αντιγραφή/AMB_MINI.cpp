
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>

#include <algorithm>
#include <vector>
#include "Knob_arcs.hpp"

// ============================================================================
// AMB_MINI — a compact generative ambient instrument inspired by Eno / AMB-1.
// The patch composes on its own: a fresh chord is chosen every few seconds,
// eight bell loops of unequal prime length play against it, and a sub bass,
// a low rumble and soft Euclidean heartbeats fill out the texture. Everything
// runs through a tape echo and a very long reverb. Nothing repeats exactly.
//
//   KEY / SCALE   tonal centre and mode (default C minor)
//   TEMPO         clock for the bell loops and heartbeats
//   CHORD         seconds between chord changes (cross-fade lasts 7 s)
//   PAD / BELL / BASS / PULSE   layer mix
//   ECHO          TIME, FBK and MIX of the tape ping-pong echo
//   REVERB        DECAY (0.8 .. 32 s), DAMP (tail darkness), MIX
//   DARK          global tone (0 = bright, 1 = dark)
//   FREEZE        latches the reverb into an infinite drone
//
//   FRZ IN        gate input: freezes the reverb while high
//   OUT L/R       stereo audio output
//   CHORD         current chord root, 1 V/oct (C4 = 0 V)
//   BELL          pitch of the most recent bell, 1 V/oct
//   TRIG          trigger fired on every bell
//   CLOCK         trigger fired on every beat
//   LEVEL         smoothed output level, 0..10 V
// ============================================================================

namespace
{

    // ============================================================================
    // Scale tables
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

    static const float AMB_PITCH_MIN = -16.f; // display minimum (semitones from C3)
    static const float AMB_PITCH_MAX = 36.f;  // display maximum
    static const float AMB_HIST_WIN = 20.f;   // display window, seconds

    // Prime-number loop lengths (in beats) keep the bell patterns drifting.
    static const int AMB_LOOP_BEATS[8] = {17, 23, 29, 31, 37, 41, 43, 47};

    // Euclidean patterns, each spanning a 32-beat cycle.
    static const int AMB_EUCLID[6][2] = {
        {32, 5}, {24, 4}, {16, 2}, {28, 3}, {20, 5}, {12, 3}};

    // ============================================================================
    // DSP primitives, free of Rack dependencies
    // ============================================================================
    namespace ambdsp
    {
        static const float kPi = 3.14159265358979f;

        // One-pole low-pass coefficient for cutoff fc at sample interval dt.
        inline float lpCoef(float fc, float dt)
        {
            return 1.f - std::exp(-2.f * kPi * fc * dt);
        }

        // Integer division that rounds towards negative infinity.
        inline int floorDiv(int a, int b)
        {
            int q = a / b;
            if ((a % b != 0) && ((a < 0) != (b < 0)))
                --q;
            return q;
        }

        // Linear-interpolating fractional delay line.
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

            // Read before write: returns the sample that was written d samples ago.
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

        // Fixed-length Schroeder all-pass.
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

        // Eight-line feedback delay network with a Hadamard mixing matrix.
        //   - RT60 is set directly (0.8 to 32 s)
        //   - every line has a one-pole low-pass, darkening the tail
        //   - slowly modulated line lengths break up the metallic ringing
        //   - freeze forces unity loop gain, disables filtering and mutes input
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
            float modBase = 6.f;
            float modDepth = 6.f;
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

            // In-place 8-point Hadamard transform, normalised by 1/sqrt(8).
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
                // Control rate (every 32 samples): RT60 to loop gains, damping.
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

                // Frozen: no modulation, so the loop is truly lossless.
                modDepth += ((freeze ? 0.f : modBase) - modDepth) * lpCoef(1.f, dt);
                // Input fades out/in over ~50 ms when freezing.
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

                // DC blocker around 15 Hz.
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

        // Tape-style ping-pong echo: dotted right side, wow and flutter, and a
        // darkening, saturating feedback path.
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

                // Feedback path: low-pass (darker with DARK), high-pass, soft clip.
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
    } // namespace ambdsp

    // ============================================================================
    // AmbEngine: the whole generative voice
    // ============================================================================
    struct AmbEngine
    {
        // Parameters, refreshed by the module on every sample.
        int key = 0;
        int scaleIdx = 1;
        float tempoBpm = 72.f;
        float padLevel = 0.60f;
        float bellLevel = 0.45f;
        float bassLevel = 0.50f;
        float pulseLevel = 0.25f;
        float dark = 0.60f;
        float echoTime = 0.50f;
        float echoMix = 0.45f;
        float echoFbk = 0.60f;
        float reverbMix = 0.60f;
        float reverbDecay = 0.70f;
        float reverbDamp = 0.55f;
        float chordInterval = 24.f;
        bool freeze = false;

        // Outputs and display-facing state.
        float outL = 0.f, outR = 0.f;
        float level = 0.f;
        float meterL = 0.f, meterR = 0.f;
        float chordRoot = 0.f;      // pad root, semitones above C3 (smoothed)
        float lastBellSemis = 12.f; // semitones above C3
        int bellTrigCount = 0;
        int beatCount = 0;
        double time = 0.0;
        float echoMs = 500.f;
        float reverbSec = 8.f;

        // Rolling history of notes for the display.
        struct NoteEvent
        {
            double born;
            double end;
            float pitch;
            int layer;
        };
        static const int HIST_MAX = 160;
        NoteEvent hist[HIST_MAX];
        int histPos = 0;

        // Internal state.
        float sr = 0.f;
        uint32_t rngState = 2463534242u;
        double beatPhase = 0.0;
        int lastBeat = -1;

        // Pad: two chord sets that cross-fade; 4 notes x 2 detuned voices each.
        struct PadSet
        {
            float note[4] = {0.f, 7.f, 12.f, 14.f};
            float phase[8] = {};
            float freq[8] = {};
            float gain = 0.f;
        };
        PadSet pad[2];
        int padCur = 0;
        float padFade = 1.f;
        float padLfo[8] = {0.f, 0.13f, 0.27f, 0.41f, 0.55f, 0.69f, 0.83f, 0.97f};
        float padLfoRate[8] = {0.031f, 0.047f, 0.023f, 0.059f, 0.037f, 0.071f, 0.043f, 0.029f};
        float padTimer = 0.f;
        int lastDegree = -1;
        float breathPh = 0.f;

        // Bells: eight independent loops.
        static const int NBELLS = 8;
        float bellPh[NBELLS] = {}, bellModPh[NBELLS] = {};
        float bellEnv[NBELLS] = {}, bellAge[NBELLS] = {};
        float bellFreq[NBELLS] = {}, bellRatio[NBELLS] = {};
        float bellVel[NBELLS] = {}, bellPan[NBELLS] = {};
        int bellCycle[NBELLS] = {};

        // Bass and rumble.
        float bassPitch = -6.f, bassTarget = -6.f;
        float bassPhase = 0.f, bassLfo = 0.f;
        float texLp1 = 0.f, texLp2 = 0.f;

        // Heartbeats and ticks.
        static const int NPULSE = 6;
        int euclidStep[NPULSE] = {};
        float pulseEnv[NPULSE] = {}, pulsePhase[NPULSE] = {};
        float tickEnv[NPULSE] = {}, tickLp[NPULSE] = {};
        bool euclidPat[NPULSE][32] = {};
        int euclidLen[NPULSE] = {};

        // Smoothed controls.
        float sPad = 0.f, sBell = 0.f, sBass = 0.f, sPulse = 0.f;
        float sEchoMix = 0.f, sRevMix = 0.f, sDark = 0.f;

        // Effects and master stage.
        ambdsp::Echo echo;
        ambdsp::Reverb reverb;
        float mLp1L = 0.f, mLp2L = 0.f, mLp1R = 0.f, mLp2R = 0.f;
        float dcXL = 0.f, dcYL = 0.f, dcXR = 0.f, dcYR = 0.f;

        // Xorshift RNG, uniform in [0, 1).
        float rnd()
        {
            rngState ^= rngState << 13;
            rngState ^= rngState >> 17;
            rngState ^= rngState << 5;
            return (float)(rngState >> 8) * (1.f / 16777216.f);
        }

        void seed(uint32_t s) { rngState = s | 1u; }

        AmbEngine()
        {
            // Pre-compute the Euclidean patterns.
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
            // Ratio between each bell's carrier and modulator.
            for (int b = 0; b < NBELLS; b++)
                bellRatio[b] = (b % 3 == 1) ? 2.f : ((b % 3 == 2) ? 1.5f : 1.f);
        }

        void setSampleRate(float sampleRate)
        {
            if (sampleRate == sr)
                return;
            sr = sampleRate;
            echo.init(sr);
            reverb.init(sr);
        }

        // Convert a scale degree (can be negative or larger than one octave) to
        // semitones above C3 within the current scale.
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

        // Mark all still-sustaining pad events as ended at the current time.
        void closePadEvents()
        {
            for (int i = 0; i < HIST_MAX; i++)
                if (hist[i].layer == 0 && hist[i].end < 0.0 && hist[i].born > -999.0)
                    hist[i].end = time;
        }

        // Choose a new chord: root, third, fifth and one colour tone from the scale.
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

            // Begin a cross-fade into a fresh chord set.
            closePadEvents();
            int old = padCur;
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
            (void)old;
            padTimer = 0.f;

            // Fold the bass root into the low octave.
            float b = (float)root - 12.f;
            while (b < -12.f)
                b += 12.f;
            while (b > 0.f)
                b -= 12.f;
            bassTarget = b;

            for (int k = 0; k < 4; k++)
                addHistory(pad[padCur].note[k], 0, -1.0);
            addHistory(bassTarget, 2, time + 12.0);
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
        }

        // Main per-sample update.
        void process(float dt)
        {
            time += dt;
            const float twoPi = 2.f * ambdsp::kPi;
            const float darkAmt = std::min(std::max(sDark, 0.f), 1.f);

            // Smooth the level controls (~50 ms).
            {
                float k = ambdsp::lpCoef(3.f, dt);
                sPad += (padLevel - sPad) * k;
                sBell += (bellLevel - sBell) * k;
                sBass += (bassLevel - sBass) * k;
                sPulse += (pulseLevel - sPulse) * k;
                sEchoMix += (echoMix - sEchoMix) * k;
                sRevMix += (reverbMix - sRevMix) * k;
                sDark += (dark - sDark) * k;
            }

            // 1. Transport.
            const float bps = std::max(tempoBpm, 20.f) / 60.f;
            beatPhase += (double)bps * dt;
            int beat = (int)std::floor(beatPhase);
            if (beat != lastBeat)
            {
                if (lastBeat >= 0)
                    ++beatCount;
                lastBeat = beat;
            }

            // 2. Pad: pick a new chord every chordInterval seconds and cross-fade.
            padTimer += dt;
            if (padTimer >= std::max(chordInterval, 2.f))
                pickNewChord();

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

            // 3. Bells: eight loops of unequal prime length, soft FM, dark.
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
                        addHistory((float)semi, 1, -1.0);
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
                    bellEnv[b] *= std::exp(-dt / 3.8f);
                    bellL += sv * (1.f - bellPan[b]) * 0.5f;
                    bellR += sv * (1.f + bellPan[b]) * 0.5f;
                }
            }
            bellL *= sBell;
            bellR *= sBell;

            // 4. Bass: sine sub with a touch of second harmonic, plus a slow rumble.
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

            // 5. Pulse layer: soft low heartbeats and dusty ticks.
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
                        addHistory(0.f, 3, time);
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
                    pulseEnv[p] *= std::exp(-dt / 0.28f);
                }
                if (tickEnv[p] > 0.001f)
                {
                    float n2 = rnd() * 2.f - 1.f;
                    tickLp[p] += (n2 - tickLp[p]) * tickA;
                    float tk = tickLp[p] * tickEnv[p] * 0.06f;
                    pulseL += tk * (0.5f + 0.1f * (float)p);
                    pulseR += tk * (1.0f - 0.1f * (float)p);
                    tickEnv[p] *= std::exp(-dt / 0.03f);
                }
            }
            pulseL *= sPulse;
            pulseR *= sPulse;

            // 6. Dry sum.
            float dryL = std::tanh(padL + bellL + bassOut + pulseL);
            float dryR = std::tanh(padR + bellR + bassOut + pulseR);

            // 7. Tape echo added on top of the dry signal.
            float wetEL, wetER;
            echo.process(dryL, dryR, echoTime, echoFbk, darkAmt, dt, wetEL, wetER);
            float em = std::min(std::max(sEchoMix, 0.f), 1.f);
            float mixL = dryL + wetEL * em * 1.1f;
            float mixR = dryR + wetER * em * 1.1f;
            echoMs = echoTime * 1000.f;

            // 8. Reverb, fed by dry plus echoes.
            float rvL, rvR;
            reverb.process(mixL * 0.9f, mixR * 0.9f, reverbDecay, reverbDamp, freeze, dt, rvL, rvR);
            reverbSec = 0.8f * std::pow(40.f, std::min(std::max(reverbDecay, 0.f), 1.f));
            float wm = std::min(std::max(sRevMix, 0.f), 1.f);
            float finL = mixL * (1.f - 0.6f * wm) + rvL * wm * 2.2f;
            float finR = mixR * (1.f - 0.6f * wm) + rvR * wm * 2.2f;

            // 9. Master: two-pole low-pass tracking DARK, DC block, soft limit.
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

            // 10. Meters.
            float rel = std::exp(-dt / 0.25f);
            meterL = std::max(std::fabs(outL), meterL * rel);
            meterR = std::max(std::fabs(outR), meterR * rel);
            level = std::max(meterL, meterR);
        }
    };

    // ------------------------------------------------------------
    // Panel geometry (28 HP = 420 x 380)
    // ------------------------------------------------------------
    static const float AMB_W = RACK_GRID_WIDTH * 28; // 420
    static const float AMB_H = RACK_GRID_HEIGHT;     // 380

    // Display area (the bezel adds 4 px on every side).
    static const float AMB_DX = 20.f, AMB_DY = 24.f, AMB_DW = 380.f, AMB_DH = 124.f;

    // Inside the display (local coordinates).
    static const float AMB_HDR_H = 18.f;
    static const float AMB_PLOT_L = 30.f;
    static const float AMB_PLOT_R = 336.f;
    static const float AMB_PLOT_T = 22.f;
    static const float AMB_PLOT_B = 92.f;
    static const float AMB_MET_X = 348.f; // two level meters, 6 px wide, 10 px apart

    // Control rows: eight equal cells spanning x = 10 .. 410.
    static const float AMB_ROW_X0 = 10.f;
    static const float AMB_ROW_W = 400.f;
    static const float AMB_CELL_W = 46.f;
    static const float AMB_CELL_H = 50.f;
    static const float AMB_CTRL_DY = 33.f; // control centre below the cell top

    static const float AMB_R1_HDR = 161.f, AMB_R1_TOP = 169.f; // COMPOSE | LAYERS
    static const float AMB_R2_HDR = 228.f, AMB_R2_TOP = 236.f; // ECHO | REVERB | MASTER
    static const float AMB_R3_HDR = 295.f, AMB_R3_TOP = 303.f; // I / O

    static inline float ambCellX(int i)
    {
        return AMB_ROW_X0 + AMB_ROW_W / 8.f * ((float)i + 0.5f);
    }

    // ---------------------------------------------------------------------------
    // Parameter display helpers
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

    // ============================================================================
    // AMB_MINI module
    // ============================================================================
    struct AMB_MINI : Module
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
            PARAMS_LEN
        };
        enum InputId
        {
            FREEZE_INPUT,
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
        int lastBellCount = 0;
        int lastBeatCount = 0;

        dsp::SchmittTrigger freezeBtn;
        dsp::PulseGenerator trigPulse, clockPulse;

        AMB_MINI()
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

            configParam(ECHO_MIX_PARAM, 0.f, 1.f, 0.45f, "Echo mix", "%", 0.f, 100.f);
            configParam(ECHO_FBK_PARAM, 0.f, 0.9f, 0.60f, "Echo feedback", "%", 0.f, 100.f);
            configParam(REVERB_MIX_PARAM, 0.f, 1.f, 0.60f, "Reverb mix", "%", 0.f, 100.f);
            configParam(REVERB_DECAY_PARAM, 0.f, 1.f, 0.70f, "Reverb decay", "%", 0.f, 100.f);

            configParam<ChordIntervalParamQuantity>(CHORD_INT_PARAM, 4.f, 60.f, 24.f, "Chord change interval");

            configParam(DARK_PARAM, 0.f, 1.f, 0.60f, "Dark (tone)", "%", 0.f, 100.f);
            configParam<EchoTimeParamQuantity>(ECHO_TIME_PARAM, 0.12f, 1.2f, 0.50f, "Echo time");
            configParam(REVERB_DAMP_PARAM, 0.f, 1.f, 0.55f, "Reverb damping", "%", 0.f, 100.f);
            configButton(FREEZE_PARAM, "Freeze reverb");

            configInput(FREEZE_INPUT, "Freeze gate");
            configOutput(OUT_L_OUTPUT, "Left");
            configOutput(OUT_R_OUTPUT, "Right");
            configOutput(CHORD_OUTPUT, "Chord root (1 V/oct)");
            configOutput(BELL_OUTPUT, "Bell pitch (1 V/oct)");
            configOutput(TRIG_OUTPUT, "Bell trigger");
            configOutput(CLOCK_OUTPUT, "Beat clock");
            configOutput(LEVEL_OUTPUT, "Output level");

            engine.seed(random::u32());
        }

        void onReset(const ResetEvent &e) override
        {
            Module::onReset(e);
            frozen = false;
            lastKey = lastScale = -1; // pick a fresh chord on the next sample
        }

        json_t *dataToJson() override
        {
            json_t *root = json_object();
            json_object_set_new(root, "frozen", json_boolean(frozen));
            return root;
        }

        void dataFromJson(json_t *root) override
        {
            json_t *f = json_object_get(root, "frozen");
            if (f)
                frozen = json_is_true(f);
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

            engine.freeze = freezeNow;
            engine.tempoBpm = params[TEMPO_PARAM].getValue();
            engine.padLevel = params[PAD_LEVEL_PARAM].getValue();
            engine.bellLevel = params[BELL_LEVEL_PARAM].getValue();
            engine.bassLevel = params[BASS_LEVEL_PARAM].getValue();
            engine.pulseLevel = params[PULSE_LEVEL_PARAM].getValue();
            engine.echoMix = params[ECHO_MIX_PARAM].getValue();
            engine.echoFbk = params[ECHO_FBK_PARAM].getValue();
            engine.reverbMix = params[REVERB_MIX_PARAM].getValue();
            engine.reverbDecay = params[REVERB_DECAY_PARAM].getValue();
            engine.chordInterval = params[CHORD_INT_PARAM].getValue();
            engine.dark = params[DARK_PARAM].getValue();
            engine.echoTime = params[ECHO_TIME_PARAM].getValue();
            engine.reverbDamp = params[REVERB_DAMP_PARAM].getValue();

            engine.process(args.sampleTime);

            outputs[OUT_L_OUTPUT].setVoltage(engine.outL * 5.f);
            outputs[OUT_R_OUTPUT].setVoltage(engine.outR * 5.f);

            // Pitches are semitones above C3, so C4 = 12 -> 0 V.
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
        }
    };

    // ============================================================================
    // User interface in the agmm style
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

        // Section title drawn above a group of cells from x0 to x1.
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
                ag::panelBase(vg, AMB_W, AMB_H, "AMB_MINI");
                ag::bezel(vg, AMB_DX, AMB_DY, AMB_DW, AMB_DH);

                // Row 1: composition | layer levels.
                ambGroupHeader(vg, 10.f, 210.f, AMB_R1_HDR, "COMPOSE");
                ambGroupHeader(vg, 210.f, 410.f, AMB_R1_HDR, "LAYERS");
                const char *r1[8] = {"KEY", "SCALE", "TEMPO", "CHORD", "PAD", "BELL", "BASS", "PULSE"};
                for (int i = 0; i < 8; ++i)
                    ag::cell(vg, ambCellX(i), AMB_R1_TOP, AMB_CELL_W, AMB_CELL_H, r1[i]);

                // Row 2: echo | reverb | master.
                ambGroupHeader(vg, 10.f, 160.f, AMB_R2_HDR, "ECHO");
                ambGroupHeader(vg, 160.f, 310.f, AMB_R2_HDR, "REVERB");
                ambGroupHeader(vg, 310.f, 410.f, AMB_R2_HDR, "MASTER");
                const char *r2[8] = {"TIME", "FBK", "MIX", "DECAY", "DAMP", "MIX", "DARK", "FREEZE"};
                for (int i = 0; i < 8; ++i)
                    ag::cell(vg, ambCellX(i), AMB_R2_TOP, AMB_CELL_W, AMB_CELL_H, r2[i]);

                // Row 3: freeze input on the left, tinted outputs afterwards.
                ag::sectionHeader(vg, AMB_W, AMB_R3_HDR, "I / O", 10.f);
                const char *r3[8] = {"FRZ IN", "OUT L", "OUT R", "CHORD", "BELL", "TRIG", "CLOCK", "LEVEL"};
                for (int i = 0; i < 8; ++i)
                    ag::cell(vg, ambCellX(i), AMB_R3_TOP, AMB_CELL_W, AMB_CELL_H, r3[i], i >= 1);

                Widget::draw(args);
            }
        };

        struct AMBDisplay : TransparentWidget
        {
            AMB_MINI *module = nullptr;
            AMBDisplay(AMB_MINI *m) : module(m)
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

            // x position for a time `age` seconds in the past.
            static float ageX(double age)
            {
                float a = rack::math::clamp((float)(age / (double)AMB_HIST_WIN), 0.f, 1.f);
                return AMB_PLOT_R - a * (AMB_PLOT_R - AMB_PLOT_L);
            }

            void draw(const DrawArgs &args) override
            {
                NVGcontext *vg = args.vg;
                ag::screen(vg, 0.f, 0.f, AMB_DW, AMB_DH);

                // Hairlines under the header and above the legend row.
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

                ag::plotFrame(vg, AMB_PLOT_L, AMB_PLOT_T, AMB_PLOT_R - AMB_PLOT_L, AMB_PLOT_B - AMB_PLOT_T);

                // Octave grid: C2 .. C6 (C3 = 0 semitones).
                static const char *octTxt[5] = {"C2", "C3", "C4", "C5", "C6"};
                nvgBeginPath(vg);
                for (int i = 0; i < 5; ++i)
                {
                    float y = std::floor(pitchY((float)(-12 + 12 * i))) + 0.5f;
                    nvgMoveTo(vg, AMB_PLOT_L + 1.f, y);
                    nvgLineTo(vg, AMB_PLOT_R - 1.f, y);
                }
                // Vertical grid lines every 5 s.
                for (int k = 1; k < 4; ++k)
                {
                    float x = std::floor(ageX(5.0 * k)) + 0.5f;
                    nvgMoveTo(vg, x, AMB_PLOT_T + 1.f);
                    nvgLineTo(vg, x, AMB_PLOT_B - 1.f);
                }
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 14));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                for (int i = 0; i < 5; ++i)
                    ag::text(vg, AMB_PLOT_L - 4.f, pitchY((float)(-12 + 12 * i)), octTxt[i], 7.f, ag::dim(), ag::ALIGN_R, 0.f);

                static const char *timeTxt[5] = {"20S", "15", "10", "5", "NOW"};
                for (int k = 0; k < 5; ++k)
                    ag::text(vg, ageX(20.0 - 5.0 * k), AMB_PLOT_B + 8.f, timeTxt[k], 7.f, ag::dim(), ag::ALIGN_C, 0.f);

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

                const NVGcolor colours[4] = {
                    ag::violet(),            // pad
                    ag::accent(),            // bell
                    ag::warm(),              // bass
                    nvgRGB(0xa0, 0xa8, 0xb4) // pulse
                };

                // -------- snapshot --------
                int key = 0, scale = 1;
                float tempo = 72.f, chord = 0.f, mL = 0.f, mR = 0.f, echoMs = 500.f, revSec = 8.f;
                bool frozen = false;
                double now = 0.0;
                const AmbEngine::NoteEvent *hist = nullptr;
                AmbEngine::NoteEvent demo[24];

                if (module)
                {
                    const AmbEngine &e = module->engine;
                    key = e.key;
                    scale = e.scaleIdx;
                    tempo = e.tempoBpm;
                    chord = e.chordRoot;
                    mL = e.meterL;
                    mR = e.meterR;
                    echoMs = e.echoMs;
                    revSec = e.reverbSec;
                    frozen = e.freeze;
                    now = e.time;
                    hist = e.hist;
                }
                else
                {
                    // Static preview for the module browser.
                    now = 20.0;
                    int n = 0;
                    const float pads[4] = {0.f, 3.f, 7.f, 10.f};
                    for (int k = 0; k < 4; ++k)
                        demo[n++] = AmbEngine::NoteEvent{4.0, -1.0, pads[k], 0};
                    for (int k = 0; k < 4; ++k)
                        demo[n++] = AmbEngine::NoteEvent{0.0, 4.0, pads[k] + 5.f, 0};
                    demo[n++] = AmbEngine::NoteEvent{4.0, 16.0, -9.f, 2};
                    for (int k = 0; k < 8; ++k)
                        demo[n++] = AmbEngine::NoteEvent{3.0 + 2.2 * k, -1.0, 15.f + 3.f * (float)(k % 5), 1};
                    for (int k = 0; k < 7; ++k)
                        demo[n++] = AmbEngine::NoteEvent{1.0 + 2.7 * k, 1.0 + 2.7 * k, 0.f, 3};
                    hist = demo;
                }
                key = rack::math::clamp(key, 0, 11);
                scale = rack::math::clamp(scale, 0, AMB_NSCALES - 1);
                const int histN = module ? AmbEngine::HIST_MAX : 24;

                // -------- events --------
                nvgSave(vg);
                nvgScissor(vg, AMB_PLOT_L, AMB_PLOT_T, AMB_PLOT_R - AMB_PLOT_L, AMB_PLOT_B - AMB_PLOT_T);
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
                    const int lay = rack::math::clamp(ev.layer, 0, 3);
                    // Bells and pulses are instantaneous; drop them once off-screen.
                    if ((lay == 1 || lay == 3) && age > (double)AMB_HIST_WIN)
                        continue;
                    const float fadeAge = (lay == 1 || lay == 3) ? (float)age : (float)(now - endT);
                    const float fade = rack::math::clamp(1.f - fadeAge / AMB_HIST_WIN * 0.7f, 0.2f, 1.f);
                    float x0 = ageX(age);
                    float x1 = ageX(now - endT);

                    if (lay == 0 || lay == 2)
                    {
                        // Pad and bass notes: sustained bars.
                        float h = (lay == 0) ? 3.f : 4.f;
                        float y = pitchY(ev.pitch);
                        nvgBeginPath(vg);
                        nvgRect(vg, x0, y - h * 0.5f, std::max(x1 - x0, 1.f), h);
                        nvgFillColor(vg, ag::withA(colours[lay], 0.80f * fade));
                        nvgFill(vg);
                    }
                    else if (lay == 1)
                    {
                        // Bells: dots that fade as they ring out.
                        nvgBeginPath(vg);
                        nvgCircle(vg, x0, pitchY(ev.pitch), 2.6f);
                        nvgFillColor(vg, ag::withA(colours[1], 0.90f * fade));
                        nvgFill(vg);
                    }
                    else
                    {
                        // Pulses: short ticks along the bottom edge.
                        nvgBeginPath(vg);
                        nvgRect(vg, x0 - 0.5f, AMB_PLOT_B - 6.f, 1.f, 5.f);
                        nvgFillColor(vg, ag::withA(colours[3], 0.85f * fade));
                        nvgFill(vg);
                    }
                }
                nvgRestore(vg);

                // -------- header --------
                char buf[64];
                snprintf(buf, sizeof(buf), "%s %s", AMB_NOTE_NAMES[key], AMB_SCALE_NAMES[scale]);
                ag::text(vg, 8.f, AMB_HDR_H * 0.5f, buf, 8.f, ag::accent(), ag::ALIGN_L, 0.5f);

                snprintf(buf, sizeof(buf), "%.0f BPM", tempo);
                ag::text(vg, AMB_DW * 0.5f, AMB_HDR_H * 0.5f, buf, 8.f, ag::dim(), ag::ALIGN_C, 0.5f);

                char nb[24];
                ambNoteName((int)std::round(chord), nb, sizeof(nb));
                ag::text(vg, AMB_DW - 8.f, AMB_HDR_H * 0.5f, nb, 9.f, ag::bright(), ag::ALIGN_R, 0.5f);
                ag::chip(vg, AMB_DW - 8.f - 22.f - 4.f - 40.f, 3.f, 40.f, 12.f, "FREEZE", frozen, ag::accent());

                // -------- meters --------
                ag::meter(vg, AMB_MET_X, AMB_PLOT_T, 6.f, AMB_PLOT_B - AMB_PLOT_T, mL * 1.8f);
                ag::meter(vg, AMB_MET_X + 10.f, AMB_PLOT_T, 6.f, AMB_PLOT_B - AMB_PLOT_T, mR * 1.8f);

                // -------- legend and delay times --------
                const char *names[4] = {"PAD", "BELL", "BASS", "PULSE"};
                const float lx[4] = {8.f, 52.f, 100.f, 150.f};
                for (int i = 0; i < 4; ++i)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, lx[i], 112.f, 6.f, 6.f);
                    nvgFillColor(vg, colours[i]);
                    nvgFill(vg);
                    ag::text(vg, lx[i] + 10.f, 115.f, names[i], 7.f, ag::dim(), ag::ALIGN_L, 0.3f);
                }
                snprintf(buf, sizeof(buf), "ECHO %.0f MS   REV %.1f S", echoMs, revSec);
                ag::text(vg, AMB_DW - 8.f, 115.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.3f);

                Widget::drawLayer(args, layer);
            }
        };
    } // namespace

    // ============================================================================
    // Widget
    // ============================================================================
    struct AMB_MINIWidget : ModuleWidget
    {
        AMB_MINIWidget(AMB_MINI *module)
        {
            setModule(module);
            box.size = Vec(AMB_W, AMB_H);

            addChild(new AMBPanel());
            addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
            addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
            addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
            addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

            addChild(new AMBDisplay(module));

            // Row 1: KEY, SCALE, TEMPO, CHORD | PAD, BELL, BASS, PULSE.
            const float y1 = AMB_R1_TOP + AMB_CTRL_DY;
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(0), y1), module, AMB_MINI::KEY_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(1), y1), module, AMB_MINI::SCALE_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(2), y1), module, AMB_MINI::TEMPO_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(3), y1), module, AMB_MINI::CHORD_INT_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(4), y1), module, AMB_MINI::PAD_LEVEL_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(5), y1), module, AMB_MINI::BELL_LEVEL_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(6), y1), module, AMB_MINI::BASS_LEVEL_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(7), y1), module, AMB_MINI::PULSE_LEVEL_PARAM));

            // Row 2: echo TIME, FBK, MIX | reverb DECAY, DAMP, MIX | DARK, FREEZE.
            const float y2 = AMB_R2_TOP + AMB_CTRL_DY;
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(0), y2), module, AMB_MINI::ECHO_TIME_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(1), y2), module, AMB_MINI::ECHO_FBK_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(2), y2), module, AMB_MINI::ECHO_MIX_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(3), y2), module, AMB_MINI::REVERB_DECAY_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(4), y2), module, AMB_MINI::REVERB_DAMP_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(5), y2), module, AMB_MINI::REVERB_MIX_PARAM));
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(ambCellX(6), y2), module, AMB_MINI::DARK_PARAM));
            addParam(createParamCentered<LEDButton>(Vec(ambCellX(7), y2), module, AMB_MINI::FREEZE_PARAM));
            addChild(createLightCentered<MediumLight<agmm::CyanLight>>(Vec(ambCellX(7), y2), module, AMB_MINI::FREEZE_LIGHT));

            // Row 3: FRZ IN | OUT L, OUT R, CHORD, BELL, TRIG, CLOCK, LEVEL.
            const float y3 = AMB_R3_TOP + AMB_CTRL_DY;
            addInput(createInputCentered<DarkPJ301MPort>(Vec(ambCellX(0), y3), module, AMB_MINI::FREEZE_INPUT));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ambCellX(1), y3), module, AMB_MINI::OUT_L_OUTPUT));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ambCellX(2), y3), module, AMB_MINI::OUT_R_OUTPUT));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ambCellX(3), y3), module, AMB_MINI::CHORD_OUTPUT));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ambCellX(4), y3), module, AMB_MINI::BELL_OUTPUT));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ambCellX(5), y3), module, AMB_MINI::TRIG_OUTPUT));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ambCellX(6), y3), module, AMB_MINI::CLOCK_OUTPUT));
            addOutput(createOutputCentered<DarkPJ301MPort>(Vec(ambCellX(7), y3), module, AMB_MINI::LEVEL_OUTPUT));
            agmm::addKnobArcs(this, 6.f);
        }

        
    };

} // anonymous namespace

// Expose the model with external linkage so Rack can discover it.
Model *modelAMB_MINI = createModel<AMB_MINI, AMB_MINIWidget>("AMB_MINI");