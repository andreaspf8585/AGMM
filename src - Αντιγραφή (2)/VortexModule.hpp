#pragma once

// ------------------------------------------------------------------
// Vortex engine
//
//   REVERB  Dattorro plate: pre-delay -> 4 input allpass diffusers ->
//           figure-eight tank (modulated allpass, delay, damping, decay,
//           second allpass, delay per half) -> 14 decorrelated output taps
//           -> low-cut biquad.  Delay lengths follow the size control
//           smoothly (no buffer re-allocation, no clicks) and decay is set
//           as an RT60 time, independent of the size.
//
//   ECHO    Stereo tape-style delay: Hermite read, smoothed time, wow/flutter
//           from the mod controls, filtered + softly saturated feedback,
//           ping-pong amount, freeze as an infinite loop.
//
//   Both engines run side by side with gated inputs, so switching mode
//   crossfades and the old tail rings out instead of being cut.
//
//   Self-contained (standard library only), C++11.
// ------------------------------------------------------------------

#include <cmath>
#include <vector>
#include <algorithm>

namespace Vortex
{

    static constexpr int NUM_CHANNELS = 16;
    static constexpr int NUM_TAPS = 14; // 7 output taps per side

    static const float PI_F = 3.14159265358979f;
    static const float REF_SR = 29761.f; // sample rate of Dattorro's reference lengths
    static const float VX_SIZE_MIN = 0.25f;
    static const float VX_SIZE_MAX = 1.75f;

    // Dattorro plate geometry, in samples at 29761 Hz
    static const float VX_IN_AP[4] = {142.f, 107.f, 379.f, 277.f};
    static const float L_AP1 = 672.f, L_D1 = 4453.f, L_AP2 = 1800.f, L_D2 = 3720.f;
    static const float R_AP1 = 908.f, R_D1 = 4217.f, R_AP2 = 2656.f, R_D2 = 3163.f;
    static const float LOOP_REF = 672.f + 4453.f + 1800.f + 3720.f + 908.f + 4217.f + 2656.f + 3163.f;

    // output taps (reference samples)
    static const float TAP_L[7] = {266.f, 2974.f, 1913.f, 1996.f, 1990.f, 187.f, 1066.f};
    static const float TAP_R[7] = {353.f, 3627.f, 1228.f, 2673.f, 2111.f, 335.f, 121.f};

    static const float PLATE_IN = 0.5f;  // input trim into the tank
    static const float PLATE_OUT = 1.2f; // tap sum trim

    static inline float clampf(float x, float lo, float hi)
    {
        return x < lo ? lo : (x > hi ? hi : x);
    }

    // ------------------------------------------------------------------
    // Per-sample control block, produced once per sample by Controller and
    // read by every voice.
    // ------------------------------------------------------------------
    struct Shared
    {
        float sr = 48000.f, srf = 48000.f / REF_SR;
        float preDelay = 0.f; // samples

        // plate
        float sizeScale = 1.15f, decay = 0.5f, dampCoef = 0.5f;
        float inDiff1 = 0.6f, inDiff2 = 0.5f, decDiff1 = 0.55f, decDiff2 = 0.4f;
        float revInGain = 1.f;
        float modDepth = 0.f;                                         // samples
        float modA_L = 0.f, modA_R = 1.f, modB_L = 0.f, modB_R = 1.f; // -1..1 LFO quadrature pairs
        float dcR = 0.999f;

        // echo
        float echoDelay = 14400.f, echoMod = 0.f, echoFb = 0.5f, echoLp = 1.f, echoHp = 0.003f;
        float echoPP = 0.f, echoSat = 1.f, echoInGain = 0.f, echoLevel = 0.8f;
        float echoRotC = 1.f, echoRotS = 0.f; // stereo feedback rotation (energy preserving)

        // wet low-cut (biquad coefficients)
        float hpB0 = 1.f, hpB1 = 0.f, hpB2 = 0.f, hpA1 = 0.f, hpA2 = 0.f;

        // display ballistics
        float envDecay = 0.9996f, tapDecay = 0.9999f;
    };

    // ------------------------------------------------------------------
    // Building blocks
    // ------------------------------------------------------------------
    struct Delay
    {
        std::vector<float> buf;
        int mask = 0, w = 0;

        void init(int maxDelay)
        {
            int len = 16;
            while (len < maxDelay + 8)
                len <<= 1;
            buf.assign((size_t)len, 0.f);
            mask = len - 1;
            w = 0;
        }
        void clear()
        {
            std::fill(buf.begin(), buf.end(), 0.f);
            w = 0;
        }
        inline float limit() const { return (float)(mask - 4); }

        // All reads happen BEFORE the push of the current sample: read(d) = x[n - d].
        inline void push(float x)
        {
            buf[w] = x;
            w = (w + 1) & mask;
        }
        inline float at(int d) const { return buf[(w - d) & mask]; }
        inline float readLin(float d) const
        {
            d = clampf(d, 1.f, limit());
            int i = (int)d;
            float f = d - (float)i;
            float a = buf[(w - i) & mask];
            float b = buf[(w - i - 1) & mask];
            return a + f * (b - a);
        }
        inline float readHerm(float d) const
        {
            d = clampf(d, 2.f, limit());
            int i = (int)d;
            float f = d - (float)i;
            float ym1 = buf[(w - i + 1) & mask];
            float y0 = buf[(w - i) & mask];
            float y1 = buf[(w - i - 1) & mask];
            float y2 = buf[(w - i - 2) & mask];
            float c1 = 0.5f * (y1 - ym1);
            float c2 = ym1 - 2.5f * y0 + 2.f * y1 - 0.5f * y2;
            float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
            return ((c3 * f + c2) * f + c1) * f + y0;
        }
    };

    // Schroeder allpass: w[n] = x[n] + g w[n-D],  y[n] = w[n-D] - g w[n]
    struct Allpass
    {
        Delay d;
        void init(int maxDelay) { d.init(maxDelay); }
        void clear() { d.clear(); }
        inline float process(float x, int len, float g)
        {
            float o = d.at(len);
            float w = x + g * o;
            d.push(w);
            return o - g * w;
        }
        inline float processMod(float x, float len, float g)
        {
            float o = d.readHerm(len);
            float w = x + g * o;
            d.push(w);
            return o - g * w;
        }
    };

    struct Biquad
    {
        float z1 = 0.f, z2 = 0.f;
        void reset() { z1 = z2 = 0.f; }
        inline float process(float x, const Shared &s)
        {
            float y = s.hpB0 * x + z1;
            z1 = s.hpB1 * x - s.hpA1 * y + z2;
            z2 = s.hpB2 * x - s.hpA2 * y;
            return y;
        }
    };

    static inline float softSat(float x) { return 12.f * std::tanh(x * (1.f / 12.f)); }

    // ------------------------------------------------------------------
    // Dattorro plate
    // ------------------------------------------------------------------
    struct Plate
    {
        Allpass inAp[4];
        int inLen[4] = {2, 2, 2, 2};
        Allpass apL1, apL2, apR1, apR2;
        Delay dL1, dL2, dR1, dR2;
        float lpL = 0.f, lpR = 0.f;
        float dcxL = 0.f, dcyL = 0.f, dcxR = 0.f, dcyR = 0.f;
        float fbL = 0.f, fbR = 0.f;
        float tapLvl[NUM_TAPS] = {}; // display only

        static int mx(float ref, float srf, float headroom)
        {
            return (int)std::ceil(ref * srf * VX_SIZE_MAX * headroom) + 8;
        }

        void init(float sr)
        {
            float srf = sr / REF_SR;
            for (int i = 0; i < 4; ++i)
            {
                inLen[i] = std::max(2, (int)std::lround(VX_IN_AP[i] * srf));
                inAp[i].init(inLen[i] + 4);
            }
            apL1.init(mx(L_AP1, srf, 1.5f));
            apL2.init(mx(L_AP2, srf, 1.5f));
            apR1.init(mx(R_AP1, srf, 1.5f));
            apR2.init(mx(R_AP2, srf, 1.5f));
            dL1.init(mx(L_D1, srf, 1.f));
            dL2.init(mx(L_D2, srf, 1.f));
            dR1.init(mx(R_D1, srf, 1.f));
            dR2.init(mx(R_D2, srf, 1.f));
            reset();
        }

        void reset()
        {
            for (int i = 0; i < 4; ++i)
                inAp[i].clear();
            apL1.clear();
            apL2.clear();
            apR1.clear();
            apR2.clear();
            dL1.clear();
            dL2.clear();
            dR1.clear();
            dR2.clear();
            lpL = lpR = 0.f;
            dcxL = dcyL = dcxR = dcyR = 0.f;
            fbL = fbR = 0.f;
            for (int i = 0; i < NUM_TAPS; ++i)
                tapLvl[i] = 0.f;
        }

        void process(const Shared &s, float in, float &outL, float &outR)
        {
            // ---- input diffusion ----
            float x = in * PLATE_IN;
            x = inAp[0].process(x, inLen[0], s.inDiff1);
            x = inAp[1].process(x, inLen[1], s.inDiff1);
            x = inAp[2].process(x, inLen[2], s.inDiff2);
            x = inAp[3].process(x, inLen[3], s.inDiff2);
            x *= s.revInGain;

            const float k = s.srf * s.sizeScale; // reference samples -> samples

            // ---- left half (fed by the right half of the previous sample) ----
            float lenA = L_AP1 * k, lenB = L_AP2 * k;
            float eA = std::min(s.modDepth, 0.4f * lenA);
            float eB = std::min(0.5f * s.modDepth, 0.2f * lenB);
            float yl = apL1.processMod(x + fbR, lenA + eA * s.modA_L, -s.decDiff1);
            yl = clampf(yl, -32.f, 32.f);
            float r1 = dL1.readHerm(L_D1 * k);
            dL1.push(yl);
            lpL += s.dampCoef * (r1 - lpL);
            float a2 = apL2.processMod(lpL * s.decay, lenB + eB * s.modB_L, s.decDiff2);
            a2 = clampf(a2, -32.f, 32.f);
            float r2 = dL2.readHerm(L_D2 * k);
            dL2.push(a2);
            float oL = r2 - dcxL + s.dcR * dcyL;
            dcxL = r2;
            dcyL = oL;

            // ---- right half ----
            lenA = R_AP1 * k;
            lenB = R_AP2 * k;
            eA = std::min(s.modDepth, 0.4f * lenA);
            eB = std::min(0.5f * s.modDepth, 0.2f * lenB);
            float yr = apR1.processMod(x + fbL, lenA + eA * s.modA_R, -s.decDiff1);
            yr = clampf(yr, -32.f, 32.f);
            r1 = dR1.readHerm(R_D1 * k);
            dR1.push(yr);
            lpR += s.dampCoef * (r1 - lpR);
            a2 = apR2.processMod(lpR * s.decay, lenB + eB * s.modB_R, s.decDiff2);
            a2 = clampf(a2, -32.f, 32.f);
            r2 = dR2.readHerm(R_D2 * k);
            dR2.push(a2);
            float oR = r2 - dcxR + s.dcR * dcyR;
            dcxR = r2;
            dcyR = oR;

            fbL = oL;
            fbR = oR;

            // ---- decorrelated output taps ----
            float t[NUM_TAPS];
            t[0] = dR1.readLin(TAP_L[0] * k);
            t[1] = dR1.readLin(TAP_L[1] * k);
            t[2] = -apR2.d.readLin(TAP_L[2] * k);
            t[3] = dR2.readLin(TAP_L[3] * k);
            t[4] = -dL1.readLin(TAP_L[4] * k);
            t[5] = -apL2.d.readLin(TAP_L[5] * k);
            t[6] = -dL2.readLin(TAP_L[6] * k);
            t[7] = dL1.readLin(TAP_R[0] * k);
            t[8] = dL1.readLin(TAP_R[1] * k);
            t[9] = -apL2.d.readLin(TAP_R[2] * k);
            t[10] = dL2.readLin(TAP_R[3] * k);
            t[11] = -dR1.readLin(TAP_R[4] * k);
            t[12] = -apR2.d.readLin(TAP_R[5] * k);
            t[13] = -dR2.readLin(TAP_R[6] * k);

            float sl = 0.f, sr2 = 0.f;
            for (int i = 0; i < 7; ++i)
            {
                sl += t[i];
                sr2 += t[i + 7];
            }
            outL = sl * PLATE_OUT;
            outR = sr2 * PLATE_OUT;

            for (int i = 0; i < NUM_TAPS; ++i)
                tapLvl[i] = std::max(std::fabs(t[i]), tapLvl[i] * s.tapDecay);
        }
    };

    // ------------------------------------------------------------------
    // Stereo echo
    // ------------------------------------------------------------------
    struct Echo
    {
        Delay dl, dr;
        float lpL = 0.f, lpR = 0.f, hpL = 0.f, hpR = 0.f;

        void init(float sr)
        {
            int m = (int)(1.03f * sr) + 16; // 1 s plus modulation headroom
            dl.init(m);
            dr.init(m);
            reset();
        }
        void reset()
        {
            dl.clear();
            dr.clear();
            lpL = lpR = hpL = hpR = 0.f;
        }

        void process(const Shared &s, float inL, float inR, float &outL, float &outR)
        {
            float tL = s.echoDelay + s.echoMod * s.modA_L;
            float tR = s.echoDelay + s.echoMod * s.modA_R;
            float rL = dl.readHerm(tL);
            float rR = dr.readHerm(tR);

            // feedback path: damping lowpass, low-cut, soft saturation
            lpL += s.echoLp * (rL - lpL);
            lpR += s.echoLp * (rR - lpR);
            hpL += s.echoHp * (lpL - hpL);
            hpR += s.echoHp * (lpR - hpR);
            float fl = lpL - hpL, fr = lpR - hpR;
            fl += s.echoSat * (softSat(fl) - fl);
            fr += s.echoSat * (softSat(fr) - fr);
            fl *= s.echoFb;
            fr *= s.echoFb;

            // ping-pong: 0 = two independent delays, 1 = repeats bounce left <-> right
            // (a rotation, so the loop gain stays <= 1 whatever the amount)
            float pp = s.echoPP;
            float toL = s.echoRotC * fl + s.echoRotS * fr;
            float toR = s.echoRotC * fr - s.echoRotS * fl;
            float injL = (inL * (1.f - 0.5f * pp) + inR * 0.5f * pp) * s.echoInGain;
            float injR = inR * (1.f - pp) * s.echoInGain;

            dl.push(clampf(injL + toL, -32.f, 32.f));
            dr.push(clampf(injR + toR, -32.f, 32.f));
            outL = rL;
            outR = rR;
        }
    };

    // ------------------------------------------------------------------
    // One polyphonic voice: pre-delay, both engines, wet low-cut
    // ------------------------------------------------------------------
    struct Voice
    {
        Delay preL, preR;
        Plate plate;
        Echo echo;
        Biquad hpL, hpR;
        float revEnv = 0.f, echoEnv = 0.f;

        void init(float sr)
        {
            int m = (int)(0.5f * sr) + 8;
            preL.init(m);
            preR.init(m);
            plate.init(sr);
            echo.init(sr);
            reset();
        }
        void reset()
        {
            preL.clear();
            preR.clear();
            plate.reset();
            echo.reset();
            hpL.reset();
            hpR.reset();
            revEnv = echoEnv = 0.f;
        }

        static inline float preRead(Delay &d, float x, float t)
        {
            float out = x;
            if (t > 0.f)
            {
                float wgt = t < 1.f ? t : 1.f;
                float del = d.readLin(t < 1.f ? 1.f : t);
                out = x + wgt * (del - x);
            }
            d.push(x);
            return out;
        }

        void process(const Shared &s, float inL, float inR, float &wetL, float &wetR)
        {
            float pl = preRead(preL, inL, s.preDelay);
            float pr = preRead(preR, inR, s.preDelay);

            float rL = 0.f, rR = 0.f, eL = 0.f, eR = 0.f;

            // each engine only runs while it has input or a tail above -100 dB
            if (s.revInGain > 1e-4f || revEnv > 1e-5f)
            {
                plate.process(s, 0.5f * (pl + pr), rL, rR);
                revEnv = std::max(std::max(std::fabs(rL), std::fabs(rR)), revEnv * s.envDecay);
            }
            if (s.echoInGain > 1e-4f || echoEnv > 1e-5f)
            {
                echo.process(s, pl, pr, eL, eR);
                echoEnv = std::max(std::max(std::fabs(eL), std::fabs(eR)), echoEnv * s.envDecay);
            }

            wetL = hpL.process(rL + eL * s.echoLevel, s);
            wetR = hpR.process(rR + eR * s.echoLevel, s);
        }
    };

    // ------------------------------------------------------------------
    // Controller: raw 0..1 controls -> smoothed per-sample Shared block
    // ------------------------------------------------------------------
    struct Params
    {
        float pre = 0.f, size = 0.6f, decay = 0.5f, diff = 0.7f, damp = 0.5f, lowcut = 0.f;
        float speed = 0.3f, depth = 0.2f, eTime = 0.3f, eFb = 0.5f, eLvl = 0.8f;
        bool freeze = false, echo = false;
    };

    struct Controller
    {
        Shared sh;
        float lastSr = 0.f;
        float kS = 0.001f, kMode = 0.001f;
        float preS = 0.f, sizeS = 1.15f, rtS = 2.f, dampS = 6000.f, lowS = 20.f, diffS = 0.7f;
        float depthS = 0.2f, eTimeS = 14400.f, eFbS = 0.475f, eLvlS = 0.8f;
        float modeRev = 1.f, modeEcho = 0.f, freezeS = 0.f;
        float lfoA = 0.f, lfoB = 0.31f, lastHpFc = -1.f;

        struct Targets
        {
            float pre, size, rt, damp, low, diff, depth, eTime, eFb, eLvl;
        };

        static Targets targets(const Params &p, float sr)
        {
            Targets t;
            t.pre = p.pre * 0.5f * sr;                                                   // 0 .. 500 ms
            t.size = VX_SIZE_MIN + (VX_SIZE_MAX - VX_SIZE_MIN) * p.size;                 // delay scale
            t.rt = 0.2f * std::pow(100.f, p.decay);                                      // RT60, 0.2 .. 20 s
            t.damp = std::min(18000.f * std::pow(2500.f / 18000.f, p.damp), 0.45f * sr); // damping cutoff
            t.low = 20.f + 480.f * p.lowcut;                                             // low cut, Hz
            t.diff = p.diff;
            t.depth = p.depth;
            t.eTime = std::max(2.f, p.eTime * sr); // 0 .. 1 s
            t.eFb = p.eFb * 0.95f;
            t.eLvl = p.eLvl;
            return t;
        }

        void setup(float sr)
        {
            lastSr = sr;
            kS = 1.f - std::exp(-1.f / (0.04f * sr));
            kMode = 1.f - std::exp(-1.f / (0.03f * sr));
            sh.sr = sr;
            sh.srf = sr / REF_SR;
            sh.dcR = 1.f - 2.f * PI_F * 6.f / sr;
            sh.envDecay = std::exp(-1.f / (0.05f * sr));
            sh.tapDecay = std::exp(-1.f / (0.25f * sr));
            lastHpFc = -1.f;
        }

        void snap(const Params &p, float sr)
        {
            Targets t = targets(p, sr);
            preS = t.pre;
            sizeS = t.size;
            rtS = t.rt;
            dampS = t.damp;
            lowS = t.low;
            diffS = t.diff;
            depthS = t.depth;
            eTimeS = t.eTime;
            eFbS = t.eFb;
            eLvlS = t.eLvl;
            modeRev = p.echo ? 0.f : 1.f;
            modeEcho = 1.f - modeRev;
            freezeS = p.freeze ? 1.f : 0.f;
        }

        void setHp(float fc, float sr)
        {
            float w0 = 2.f * PI_F * fc / sr;
            float cw = std::cos(w0), sw = std::sin(w0);
            float alpha = sw / (2.f * 0.70710678f);
            float a0 = 1.f + alpha;
            sh.hpB0 = (1.f + cw) * 0.5f / a0;
            sh.hpB1 = -(1.f + cw) / a0;
            sh.hpB2 = sh.hpB0;
            sh.hpA1 = -2.f * cw / a0;
            sh.hpA2 = (1.f - alpha) / a0;
            lastHpFc = fc;
        }

        void update(const Params &p, float sr)
        {
            if (sr != lastSr)
            {
                setup(sr);
                snap(p, sr);
            }
            const float dt = 1.f / sr;
            Targets t = targets(p, sr);

            preS += kS * (t.pre - preS);
            sizeS += kS * 0.5f * (t.size - sizeS); // slower, so size changes glide like tape
            rtS += kS * (t.rt - rtS);
            dampS += kS * (t.damp - dampS);
            lowS += kS * (t.low - lowS);
            diffS += kS * (t.diff - diffS);
            depthS += kS * (t.depth - depthS);
            eTimeS += kS * 0.4f * (t.eTime - eTimeS); // echo time glides (varispeed)
            eFbS += kS * (t.eFb - eFbS);
            eLvlS += kS * (t.eLvl - eLvlS);

            float tRev = p.echo ? 0.f : 1.f;
            modeRev += kMode * (tRev - modeRev);
            modeEcho += kMode * ((1.f - tRev) - modeEcho);
            freezeS += kMode * ((p.freeze ? 1.f : 0.f) - freezeS);

            // LFOs
            float speed = 0.1f + 4.9f * p.speed;
            lfoA += speed * dt;
            if (lfoA >= 1.f)
                lfoA -= 1.f;
            lfoB += speed * 0.71f * dt;
            if (lfoB >= 1.f)
                lfoB -= 1.f;
            sh.modA_L = std::sin(2.f * PI_F * lfoA);
            sh.modA_R = std::cos(2.f * PI_F * lfoA);
            sh.modB_L = std::sin(2.f * PI_F * lfoB);
            sh.modB_R = std::cos(2.f * PI_F * lfoB);

            // shared
            float inF = 1.f - freezeS;
            sh.preDelay = preS;
            sh.revInGain = modeRev * inF; // (scaled by the tail-length compensation below)
            sh.echoInGain = modeEcho * inF;

            // plate
            sh.sizeScale = sizeS;
            float loopT = LOOP_REF / REF_SR * sizeS;         // one trip round the figure-eight
            float dec = std::pow(10.f, -1.5f * loopT / rtS); // two half-passes per trip
            sh.decay = dec + freezeS * (0.9997f - dec);
            // long tails recirculate more energy: trim the input so loudness stays comparable
            sh.revInGain *= std::sqrt(std::max(0.05f, 1.f - dec * dec * dec * dec));
            float dampA = 1.f - std::exp(-2.f * PI_F * dampS / sr);
            sh.dampCoef = dampA + freezeS * (1.f - dampA);
            sh.inDiff1 = 0.20f + 0.55f * diffS;
            sh.inDiff2 = 0.15f + 0.475f * diffS;
            sh.decDiff1 = 0.25f + 0.45f * diffS;
            sh.decDiff2 = 0.20f + 0.30f * diffS;
            sh.modDepth = depthS * 0.005f * sr; // up to 5 ms excursion

            // echo
            sh.echoDelay = eTimeS;
            sh.echoMod = depthS * 0.0035f * sr; // up to 3.5 ms wow
            sh.echoFb = eFbS + freezeS * (1.f - eFbS);
            sh.echoLp = sh.dampCoef;
            sh.echoHp = 1.f - std::exp(-2.f * PI_F * lowS / sr);
            sh.echoPP = diffS;
            sh.echoRotC = std::cos(diffS * 0.5f * PI_F);
            sh.echoRotS = std::sin(diffS * 0.5f * PI_F);
            sh.echoSat = 1.f - freezeS;
            sh.echoLevel = eLvlS;

            // wet low-cut
            if (std::fabs(lowS - lastHpFc) > 0.02f)
                setHp(lowS, sr);
        }

        float rt60() const { return rtS; }
        float echoMs() const { return eTimeS / lastSr * 1000.f; }
        float sizeScale() const { return sizeS; }
    };

} // namespace Vortex