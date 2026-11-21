<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Stagger2.cpp
-->

# Stagger2

## 1. Overview

`Stagger2` is a 32 HP expanded version of the Stagger VCO. The core oscillator uses the same "chaos drift" concept as Stagger (a slow, filtered random pitch drift), but the signal is then fed through a full per-voice space FX chain: a modulated stereo delay, an 8-line FDN reverb, a trigger/gate envelope, a per-voice compressor, a master gain stage and a master compressor. The oscillator itself is now a 40-waveform engine with three Plaits-style macro controls (HARMONICS, TIMBRE, MORPH), each with its own CV input and attenuverter.

Key features:
- 40 selectable waveforms (original 4 minBLEP waves, band-limited additive stacks, waveshapers, sync/PD/ring-mod, classic shapes)
- 3 macro controls (HARMONICS, TIMBRE, MORPH) with CV + attenuverters, applied to waves 4..39
- Single chaos drift per voice (mono oscillator per voice, stereo FX)
- Per-voice modulated stereo delay (normal or ping-pong feedback, damped repeats, slow LFO modulation)
- Per-voice 8-line FDN reverb (8 prime-length combs + 4 allpasses, cross-feed stereo tail)
- Per-voice trigger/gate envelope with 4 decay times (300 ms / 1 s / 4 s / 12 s)
- Per-voice compressor + master compressor for the FX sum
- 3 outputs: VCO+FX (polyphonic), FX L (mono), FX R (mono)
- Editable frequency (Hz/note), waveform (name/index), delay time (ms), reverb decay (s), envelope decay
- Live display: waveform scope (2 cycles), ENV/L/R meters, FX readout box

## Module Screenshot

![Stagger2 VCV Rack Module](images/Stagger2.png)

## 2. Architecture

### Main Classes

- `Stagger2`: Core module class inheriting from `rack::Module`
- `S2FreqQ`: Custom ParamQuantity for the frequency knob (log Hz, note/semitone entry)
- `S2WaveQ`: Custom ParamQuantity for the waveform knob (40-position snap, name/index entry)
- `S2DelayTimeQ`: Custom ParamQuantity for delay time (log 20 ms..1.5 s, editable ms)
- `S2RevDecayQ`: Custom ParamQuantity for reverb decay (log 0.3..12 s, editable s)
- `S2EnvDecayQ`: Custom ParamQuantity for envelope decay (4-position snap)
- `S2ModDelay`: Modulated stereo delay line with damping and DC-blocked feedback
- `S2Comp`: Feed-forward peak compressor (fixed 3 V threshold, adjustable ratio)
- `S2Fdn`: 8-line feedback delay network reverb (8 prime combs + 4 series allpasses)
- `S2Panel`: Static panel artwork
- `S2Display`: Live display (scope, meters, FX readout)

### Inheritance

- `Stagger2` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### Structures

- `Voice { phase, lastPhase, cycle, dcX, dcY, blepRising, blepFalling, blepSaw, delay, reverb, comp, env, gateHigh, lastTrig }` — one voice with its own oscillator state, FX chain and envelope

### State (grouped)

| Group | Storage |
|-------|---------|
| Voices | `std::vector<Voice> voices` |
| Master compressors | `S2Comp compL, compR` |
| Chaos drift | `float chaosDrift` (single, shared across L/R of a voice) |
| RNG | `std::mt19937 rng`, `std::uniform_real_distribution<float> dist` |
| Shared LFOs | `float lfoPhaseA, lfoPhaseB` (used by delay and reverb) |
| Transport | `bool cvRangeTenVolt`, `float lastResetVoltage` |
| Display | `float dispFreq, dispPw, dispPhase, dispH, dispT, dispM`, `int dispWave, dispChannels`, `float dispEnv`, `bool dispTrigConn`, `float meterL, meterR` |

## 3. Waveforms (40)

The WAVE knob (0..39, snapped) selects the waveform. The WAVE CV input plus attenuverter sweeps the list (±5 V at full attenuverter sweeps half the list in either direction). Waves 0..3 ignore the HARMONICS / TIMBRE / MORPH knobs; waves 4..39 use them as documented below.

### 0..3 — Original waves (minBLEP)

| # | Name | Notes |
|---|------|-------|
| 0 | Sine | |
| 1 | Triangle | |
| 2 | Saw | minBLEP at wrap |
| 3 | Square | minBLEP at both edges; PW knob sets width |

### 4..13 — Band-limited additive stacks

All partials are cut off below Nyquist. H, T, M refer to the HARMONICS, TIMBRE, MORPH macros.

| # | Name | H | T | M |
|---|------|---|---|---|
| 4 | Saw Stack | harmonic count | spectral slope 1/k → 1/k² | phase blend sine→cosine |
| 5 | Square Stack | harmonic count | spectral slope | phase blend |
| 6 | Even Stack | harmonic count | spectral slope | phase blend |
| 7 | Prime Stack | harmonic count | spectral slope | phase blend |
| 8 | Fibonacci | harmonic count | spectral slope | phase blend |
| 9 | Octave Stack | harmonic count | spectral slope | phase blend |
| 10 | Organ | harmonic count | spectral slope | phase blend |
| 11 | Third Stack | harmonic count | spectral slope | phase blend |
| 12 | Formant | peak position | peak width | phase blend |
| 13 | Dual Formant | formant 1 position | formant 2 spacing | phase blend |

### 14..23 — Waveshapers / modulation (2× oversampled)

| # | Name | H | T | M |
|---|------|---|---|---|
| 14 | Wavefold | fold gain | bias | second fold stage |
| 15 | Tri Fold | fold gain | bias | smoothing |
| 16 | Saw Fold | fold gain | bias | saw→sine input |
| 17 | Hard Clip | drive | bias | clip→tanh |
| 18 | Soft Sat | drive | bias | sine→triangle |
| 19 | Stair Saw | steps | smoothing | curve |
| 20 | Stair Tri | steps | smoothing | skew |
| 21 | Decimated | sample points | smoothing | bit depth |
| 22 | FM 1:N | ratio 1..8 | index | sine→tri modulator |
| 23 | FM Nested | ratio 1..6 | index 1 | index 2 |

### 24..29 — Sync / phase distortion / ring mod

| # | Name | H | T | M |
|---|------|---|---|---|
| 24 | Sync Saw | sync ratio | window | saw→tri |
| 25 | Sync Square | sync ratio | window | pulse→sine (PW knob = width) |
| 26 | Sync Sine | sync ratio | window | sub level |
| 27 | PD Warp | warp 1 | warp 2 | sine→tri |
| 28 | PD Resonant | resonance | window power | saw→tri window |
| 29 | Ring Mod | ratio 1..8 | ring→AM | sine→tri modulator |

### 30..39 — Classic shapes

| # | Name | H | T | M |
|---|------|---|---|---|
| 30 | Rect Sine | power | dry sine | half→full rectify |
| 31 | Parabola | exponent | skew | parabola→sine |
| 32 | Exp Saw | curvature | symmetric | concave→convex |
| 33 | Trapezoid | slope | rounding | skew |
| 34 | Narrow Pulse | width | saw mix | pulse→ramp |
| 35 | Twin Pulse | width | spacing | 2nd pulse level |
| 36 | Gaussian | width | centre | bell→mexican hat |
| 37 | Sinc | lobes | centre | window |
| 38 | Circle | squareness | skew | circle→sine |
| 39 | Random Steps | steps | smoothing | re-roll per cycle |

Waves 4..39 are DC-blocked (~5 Hz one-pole high-pass). Only waves 0..3 use minBLEP; the rest are band-limited by construction (additive) or by 2× oversampling (waveshapers).

## 4. DSP Theory

### Single chaos drift per voice

Unlike `Stagger` (which has independent L/R drifts), `Stagger2` uses a single filtered noise source per voice, shared by L and R. The stereo width comes from the FX chain, not from pitch divergence:

$$
\text{chaos} \leftarrow (1 - \alpha)\,\text{chaos} + \alpha\,w, \quad w \sim \mathcal{U}[-1,1]
$$

with $\alpha = \min(2\pi \cdot 1.5 / f_s,\ 0.5)$. One noise draw and one filter update per sample (not per voice).

### Pitch computation

$$
\text{pitch} = \text{basePitch} + \text{V/Oct} + \text{pitchCV} \cdot \text{atten} + \text{chaos} \cdot \text{depth} \cdot \text{mult}
$$

### Exact 2^x

The frequency is computed with the exact `std::exp2` (the old bit-twiddling approximation is gone — its up-to-6% pitch error was unacceptable). Frequencies are clamped to 10 Hz..6 kHz.

### Additive waves (4..13)

Band-limited harmonic stacks. Each partial k is generated with a rotation recurrence (one multiply-add per harmonic instead of a `sin()` call), and partials above Nyquist are skipped (`nMax = 0.49 / deltaPhase`). The partial amplitude is:

$$
a_k = \frac{1-t}{k} + \frac{t}{k^2}
$$

for the stacks, or a Lorentzian peak $1/(1 + ((k-f_1)/w_1)^2)$ for the formants. Partial phase is blended from sine to cosine by the MORPH parameter. Output is normalised to ~0.8 RMS.

### Waveshapers (14..39)

All non-additive shapes are evaluated at two sub-samples (one full phase and one half-phase back) and averaged — a cheap 2× oversampling that reduces aliasing without a filter.

### Macros

The HARMONICS, TIMBRE and MORPH knobs each have a CV input and attenuverter:

$$
h = \text{clamp}\!\big(h_{\text{knob}} + a_h \cdot V_h \cdot \text{cvScale} / 5,\ 0,\ 1\big)
$$

and similarly for `t` and `m`. With cvScale = 1 (±5 V range), a full-scale CV sweeps the whole range.

### Delay modulation

The delay read position is modulated by two shared slow LFOs (0.23 Hz and 0.17 Hz):

$$
\text{pos}_{L/R} = d + 8 \cdot \text{modDepth} \cdot (\text{lfo}_{A/B} \cdot 2 - 1)
$$

where $d$ is the base delay in samples (20 ms..1.5 s, log). The L/R LFOs are different, so the two channels wobble independently.

### Delay feedback

The delay has a one-pole lowpass in the feedback path (damping) and a ~15 Hz DC-blocker, then feeds back either normally (mode 0) or cross-channel (mode 1, ping-pong). The stored sample is soft-limited at 12 V.

### Reverb

An 8-line FDN: 8 prime-length damped combs in parallel, followed by 4 series Schroeder allpasses. The comb lengths are:

$$
L_i = m_i \cdot 10 \cdot (0.28 + 0.72 \cdot \text{size}), \quad m_i \in \{101, 113, 127, 139, 149, 163, 173, 181\}
$$

Each comb's feedback gain sets the RT60:

$$
g_i = \min\!\big(10^{-3 L_i / (T_{60} \cdot f_s)},\ 0.995\big)
$$

Input is the mono difference of L/R, DC-blocked. The allpass lengths are modulated by the same LFOs (with `mod = delayMod`), and cross-feed produces a wide stereo tail.

### Envelope

Per-voice envelope, controlled by TRIG_INPUT and TRIG_MODE:

- **Trig mode**: rising edge (>1 V with previous ≤1 V) sets env = 1, then decays with `decayCoef = dt / (0.001 + decaySec)`
- **Gate mode**: gate >1 V → env attacks to 1 (`attackCoef = dt/0.003`), gate <0.1 V → env decays
- **No trigger**: env = 1

`decaySec` is selected from `{0.3, 1.0, 4.0, 12.0}` seconds by `ENV_DECAY_PARAM`.

### Compressor

Fixed 3 V threshold, ratio 1:1..10:1 (controlled by the COMP amount), asymmetric attack (~2 ms) / release (~100 ms) envelope follower. Used per-voice on the VCO output and per-channel on the FX sum.

### FX mix

Dry/wet mixing per voice:

$$
\text{wet}_{L/R} = \text{delayMix} \cdot \text{delay}_{L/R} + \text{revMix} \cdot \text{rev}_{L/R}
$$

$$
\text{mix}_{L/R} = (\text{sig} + 0.8 \cdot \text{wet}_{L/R}) \cdot \frac{2}{1 + \text{delayMix} + \text{revMix}}
$$

### VCO output

$$
\text{vco} = (\text{mix}_L + 0.15 \cdot \text{mix}_R) \cdot \text{gain}
$$

then per-voice compressor, then soft-limited at 10 V.

### FX sum

Each voice's `mixL`/`mixR` (post-gain) is accumulated into a stereo bus, scaled by `1/sqrt(channels)`, passed through the master compressors, soft-limited at 10 V, and written to FX L/R.

## 5. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `PITCH_PARAM` | 0 | 1 | ≈C4 | Frequency (10 Hz .. 6 kHz, log) |
| `WAVE_PARAM` | 0 | 39 | 0 | Waveform (snapped) |
| `HARMONICS_PARAM` | 0 | 1 | 0.5 | Harmonics macro |
| `TIMBRE_PARAM` | 0 | 1 | 0.5 | Timbre macro |
| `MORPH_PARAM` | 0 | 1 | 0.5 | Morph macro |
| `CHAOS_PARAM` | 0 | 1 | 0 | Chaos depth |
| `PW_PARAM` | 0 | 1 | 0.5 | Pulse width |
| `DELAY_TIME_PARAM` | 0 | 1 | 0.55 | Delay time (20 ms .. 1.5 s, log) |
| `DELAY_FB_PARAM` | 0 | 0.92 | 0.65 | Delay feedback |
| `DELAY_MOD_PARAM` | 0 | 1 | 0.5 | Delay LFO depth |
| `DELAY_MIX_PARAM` | 0 | 1 | 0.6 | Delay mix |
| `REV_SIZE_PARAM` | 0 | 1 | 0.75 | Reverb size |
| `REV_DECAY_PARAM` | 0 | 1 | 0.55 | Reverb decay (0.3 .. 12 s, log) |
| `REV_MIX_PARAM` | 0 | 1 | 0.75 | Reverb mix |
| `ENV_DECAY_PARAM` | 0 | 3 | 1 | Envelope decay (300 ms / 1 s / 4 s / 12 s) |
| `GAIN_PARAM` | 0 | 2 | 0.7 | Output gain |
| `COMP_PARAM` | 0 | 1 | 0.5 | Compressor amount |

### Attenuverters

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `PITCH_ATTEN_PARAM` | −1 | 1 | 0 | Pitch CV attenuverter |
| `CHAOS_ATTEN_PARAM` | −1 | 1 | 0 | Chaos CV attenuverter |
| `WAVE_ATTEN_PARAM` | −1 | 1 | 0 | Waveform CV attenuverter |
| `PW_ATTEN_PARAM` | −1 | 1 | 0 | Pulse width CV attenuverter |
| `HARMONICS_ATTEN_PARAM` | −1 | 1 | 0 | Harmonics CV attenuverter |
| `TIMBRE_ATTEN_PARAM` | −1 | 1 | 0 | Timbre CV attenuverter |
| `MORPH_ATTEN_PARAM` | −1 | 1 | 0 | Morph CV attenuverter |

### Switches

| Parameter | Values | Default | Description |
|-----------|--------|---------|-------------|
| `CV_RANGE_PARAM` | ±5 V / ±10 V | ±5 V | CV input scale |
| `CHAOS_MULT_PARAM` | ×1 / ×2 / ×3 / ×5 / ×10 | ×1 | Chaos multiplier |
| `DELAY_MODE_PARAM` | Normal / Ping-Pong | Ping-Pong | Delay feedback routing |
| `TRIG_MODE_PARAM` | Trig / Gate | Trig | Envelope trigger mode |

### Inputs

| Input | Description |
|-------|-------------|
| `CV_INPUT` | 1 V/oct pitch (polyphonic) |
| `PITCH_CV_INPUT` | Pitch modulation CV |
| `CHAOS_CV_INPUT` | Chaos depth modulation CV |
| `WAVE_CV_INPUT` | Waveform selection CV |
| `PW_CV_INPUT` | Pulse width modulation CV |
| `CHAOS_RESET_INPUT` | Reset chaos drift (trigger) |
| `TRIG_INPUT` | Trigger / gate for the envelope |
| `HARMONICS_CV_INPUT` | Harmonics CV |
| `TIMBRE_CV_INPUT` | Timbre CV |
| `MORPH_CV_INPUT` | Morph CV |

### Outputs

| Output | Description |
|--------|-------------|
| `VCO_OUTPUT` | VCO + FX (polyphonic) |
| `FX_L_OUTPUT` | FX left (mono sum) |
| `FX_R_OUTPUT` | FX right (mono sum) |

### Lights

- `CHAOS_LIGHT` — Chaos activity (`|chaosDrift| * chaosKnob`, smooth)
- `ENV_LIGHT` — Envelope (max across voices, smooth)

### Editable fields

- **Frequency**: Hz (e.g. "440"), note name (e.g. "A4", "C#3"), or semitone offset from C4 ("+12", "-5")
- **Waveform**: index ("12") or case-insensitive name / name prefix ("organ")
- **Delay time**: milliseconds ("320")
- **Reverb decay**: seconds ("2.4")

### Enumerations

- `ParamId`: Pitch, Chaos, ChaosMult, Wave, PW, PitchAtten, ChaosAtten, WaveAtten, PWAtten, CVRange, DelayTime, DelayFb, DelayMix, DelayMod, DelayMode, RevSize, RevDecay, RevMix, EnvDecay, TrigMode, Gain, Comp, Harmonics, Timbre, Morph, HarmonicsAtten, TimbreAtten, MorphAtten
- `InputId`: VOct, PitchCV, ChaosCV, WaveCV, PWCV, ChaosReset, Trig, HarmonicsCV, TimbreCV, MorphCV
- `OutputId`: VcoOut, FxL, FxR
- `LightId`: Chaos, Env

Note: the original Stagger parameters/inputs keep their ids; everything new is appended before the `*_LEN` entry so old patches still load.

## 6. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CV_INPUT` | CV | 1 V/oct | Pitch (polyphonic) |
| `PITCH_CV_INPUT` | CV | ±10 V | Pitch modulation (with attenuverter) |
| `CHAOS_CV_INPUT` | CV | ±10 V | Chaos modulation (with attenuverter) |
| `WAVE_CV_INPUT` | CV | ±10 V | Waveform selection (with attenuverter) |
| `PW_CV_INPUT` | CV | ±10 V | Pulse width modulation (with attenuverter) |
| `CHAOS_RESET_INPUT` | Gate | > 1 V | Reset chaos drift |
| `TRIG_INPUT` | Gate | > 1 V | Trigger / gate for the envelope |
| `HARMONICS_CV_INPUT` | CV | ±10 V | Harmonics macro modulation |
| `TIMBRE_CV_INPUT` | CV | ±10 V | Timbre macro modulation |
| `MORPH_CV_INPUT` | CV | ±10 V | Morph macro modulation |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `VCO_OUTPUT` | Poly Audio | ±10 V | VCO + FX (one voice per input channel) |
| `FX_L_OUTPUT` | Mono Audio | ±10 V | FX left (mono sum of all voices) |
| `FX_R_OUTPUT` | Mono Audio | ±10 V | FX right (mono sum of all voices) |

## 7. Signal Processing Pipeline

1. **Read controls** — Pitch, chaos, wave, PW, macros, attenuverters, CV range.
2. **Chaos reset** — Rising edge on `CHAOS_RESET_INPUT` zeros `chaosDrift`.
3. **Update chaos** — One-pole LP on a single noise draw.
4. **Read FX controls** — Delay time/fb/mix/mod/mode; reverb size/decay/mix; env decay/trig mode; gain/comp.
5. **Update shared LFOs** — Two slow sines (0.23 Hz, 0.17 Hz).
6. **Determine channels** — `max(1, CV_INPUT channels)`; set VCO channel count and mono FX outputs; grow the voices vector if needed.
7. **For each voice**:
   - Read V/Oct, pitch CV, chaos CV, wave CV, PW CV, macro CVs.
   - Compute pitch, frequency (exact `exp2`), phase increment.
   - Select waveform (quantized 0..39).
   - Determine PW (clamped 0.01..0.99).
   - Render the waveform:
     - waves 0..3: minBLEP
     - waves 4..13: band-limited additive (Nyquist-limited)
     - waves 14..39: 2× oversampled waveshaper
   - DC-block (waves 4..39).
   - Lazy-allocate delay and reverb buffers.
   - Update trigger envelope.
   - Run delay → reverb → delay/reverb mix.
   - Self-heal on NaN/inf.
   - VCO output = `(mixedL + 0.15 * mixedR) * gain`, then per-voice compressor, soft-limit at 10 V.
   - Accumulate `mixL * gain` / `mixR * gain` into the FX bus.
   - Channel 0 updates the display fields.
8. **FX sum** — Scale by `1/sqrt(channels)`, pass through the master compressors, soft-limit at 10 V, write FX L/R.
9. **Update LEDs and meters** — Chaos and envelope lights; meter peak-hold.

## 8. Important Functions

### `S2FreqQ::mapToFreq(float v)` / `mapFromFreq(float f)`

10 Hz..6 kHz log mapping (`10 * 600^v`) and its inverse.

### `S2FreqQ::setDisplayValueString(std::string s)`

Parses Hz ("440"), note names ("A4", "C#3", "Bb2"), or signed semitone offsets from C4 ("+12", "-5"). Whitespace is ignored; unrecognised text falls back to the base class.

### `S2WaveQ::getDisplayValueString()` / `setDisplayValueString(std::string s)`

Returns the wave name for the nearest of the 40 positions; accepts an index ("12") or a case-insensitive name/name prefix ("organ").

### `S2DelayTimeQ::getDisplayValueString()` / `setDisplayValueString(std::string s)`

Maps 0..1 to 20 ms..1.5 s (`20 * 75^v`); accepts an editable "ms" string.

### `S2RevDecayQ::getDisplayValueString()` / `setDisplayValueString(std::string s)`

Maps 0..1 to 0.3 s..12 s (`0.3 * 40^v`); accepts an editable "s" string.

### `S2EnvDecayQ::getDisplayValueString()`

Returns one of "300 ms", "1 s", "4 s", "12 s" based on the nearest of the four snapped positions.

### `s2Additive(int wave, float p, float h, float t, float m, int nMax)`

Generates a band-limited additive wave (4..13). Partials above Nyquist (`nMax`) are skipped. Partials are computed with a rotation recurrence; the output is normalised to ~0.8 RMS.

### `s2Shape(int wave, float p, float h, float t, float m, float pw, uint32_t cyc)`

Generates a non-additive wave (0..3, 14..39). `cyc` is the voice's cycle counter, used by Random Steps to re-roll its pattern per cycle.

### `s2Eval(int wave, float p, float h, float t, float m, float pw, uint32_t cyc)`

Evaluates any of the 40 waves without oversampling or DC blocking (used by the display).

### `s2SoftLimit(float x, float limit)`

Linear below 60% of `limit`, then smooth `tanh` saturation towards ±limit. Used in the delay feedback, reverb comb output, VCO output and FX outputs.

### `S2ModDelay::process(...)`

Stereo delay with LFO-modulated fractional reads, one-pole damping, ~15 Hz DC-blocked feedback, normal or ping-pong routing, soft-limited at 12 V inside the buffer.

### `S2Fdn::process(...)`

8-line FDN reverb. Input is the mono difference of L/R, DC-blocked, driving the 8 parallel combs; the sum is diffused by 4 series allpasses (with LFO-modulated lengths) and split into L/R with cross-feed for a wide tail.

### `S2Comp::process(float x, float amount, float sr)`

Peak compressor with fixed 3 V threshold, ratio 1:1..10:1, asymmetric attack (~2 ms) / release (~100 ms). Bypasses entirely when `amount < 0.001`.

### `Stagger2::onReset(const ResetEvent &e)`

Clears the chaos drift, every voice's delay/reverb/envelope/compressor/cycle/DC state, and both master compressors.

### `Stagger2::onSampleRateChange()`

No-op: all buffers are sized for the worst case.

### `Stagger2::process(const ProcessArgs &args)`

Main callback, described in §7.

## 9. Mathematical Implementation

### Chaos drift

$$
\text{chaos} \leftarrow (1 - \alpha)\,\text{chaos} + \alpha\,w, \quad \alpha = \min\!\left(2\pi \cdot 1.5 / f_s,\ 0.5\right)
$$

### Pitch and frequency

$$
\text{pitch} = \text{basePitch} + V_{\text{oct}} + a_p V_p + \text{chaos} \cdot \text{depth} \cdot \text{mult}
$$

$$
f = \text{FREQ\_C4} \cdot 2^{\text{pitch}}, \quad \Delta\phi = f \cdot \Delta t
$$

### Macro CV

$$
h = \operatorname{clamp}\!\left(h_k + a_h V_h s / 5,\ 0,\ 1\right)
$$

(similarly for `t`, `m`), where $s \in \{1, 0.5\}$ is the CV range scale.

### Waveform CV

$$
w = \operatorname{clamp}\!\left(w_k + a_w V_w \cdot \tfrac{39}{10},\ 0,\ 39\right) \quad \text{quantised to 0..39}
$$

### Delay read position

$$
\text{pos}_{L/R} = d + 8 \cdot \text{modDepth} \cdot (\text{lfo}_{A/B} \cdot 2 - 1)
$$

### Reverb comb lengths

$$
L_i = m_i \cdot 10 \cdot (0.28 + 0.72 \cdot \text{size}), \quad m_i \in \{101, 113, 127, 139, 149, 163, 173, 181\}
$$

### Reverb comb feedback

$$
g_i = \min\!\left(10^{-3 L_i / (T_{60} f_s)},\ 0.995\right), \quad \text{input gain} = \sqrt{1 - g_i^2}
$$

### Envelope decay

$$
\text{env} \leftarrow \text{env} \cdot \left(1 - \frac{\Delta t}{0.001 + T_{\text{decay}}}\right)
$$

with $T_{\text{decay}} \in \{0.3, 1, 4, 12\}$ s.

### Compressor

$$
\text{ratio} = 1 + 9 \cdot \text{amount}, \quad \text{env}_{\text{comp}} = 3 + \frac{\text{env} - 3}{\text{ratio}}
$$

### Output

$$
\text{VCO} = \operatorname{softlimit10}\!\Big(\text{comp}\big[(m_L + 0.15 m_R) \cdot g\big]\Big)
$$

$$
\text{FX}_{L/R} = \operatorname{softlimit10}\!\Big(\text{comp}_{L/R}\Big[\tfrac{1}{\sqrt{N}} \sum_c m_{L/R}^{(c)} g\Big]\Big)
$$

## 10. Performance and Optimization

### CPU Usage

- Per sample per voice: 1 oscillator (minBLEP for waves 0..3, additive for 4..13, 2× oversampled shaper for 14..39), then the delay and reverb
- Additive waves cost ~1 multiply-add per harmonic (up to 40) — cheaper than one `sin()` per partial
- FDN reverb: 8 combs × 4 allpass stages per sample per voice
- Scales linearly with polyphony
- The FX sum and master compressors add a small fixed cost per sample

### Memory Allocation

- Voices vector grows only when the channel count increases; no allocation once warmed up
- Delay: 2 × 72008 floats per voice (~576 KB per voice at 48 kHz for both channels)
- Reverb: 8 × 2400 + 4 × 1100 floats per voice (~84 KB per voice)
- Buffers are allocated lazily on the first sample (`voice.delay.bufL.empty()`)
- Display state is fixed-size

### Buffers

- Per-voice delay: `MAX_DELAY = 72008` samples per channel (~1.5 s at 48 kHz)
- Per-voice reverb: `COMB_BUF = 2400` samples × 8, `AP_BUF = 1100` samples × 4

### Real-time Processing

- All loops bounded
- No allocation in steady state, no blocking
- Self-heal flushes the voice's FX state if NaN/inf appears

### Thread Safety

- Display fields (`dispFreq`, `dispPw`, `dispPhase`, `dispH`, `dispT`, `dispM`, `dispWave`, `dispChannels`, `dispEnv`, `dispTrigConn`, `meterL`, `meterR`) are written by the audio thread and read by the UI thread via torn reads; cosmetic only
- The voices vector and per-voice FX state are only touched by the audio thread

### Numerical Stability

- Frequencies clamped to 10..6000 Hz
- Pulse width clamped to 0.01..0.99
- Chaos depth clamped to 0..1
- Waveform selection clamped to 0..39
- Filter coefficient α clamped to ≤0.5
- Delay feedback capped at 0.92 (param) and soft-limited at 12 V (buffer)
- Reverb comb feedback capped at 0.995 and soft-limited at 16 V (buffer)
- VCO and FX outputs soft-limited at 10 V
- DC blockers on the delay feedback (~15 Hz) and reverb input (~15 Hz)

## 11. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<cstdint>`, `<vector>`, `<map>`, `<string>`, `<regex>`, `<algorithm>`, `<random>`, `<initializer_list>`
- `"dsp/minblep.hpp"` — `rack::dsp::MinBlepGenerator`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `rack::dsp::MinBlepGenerator<16,16>`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::meter`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`, `agmm::VioletLight`

## 12. Summary

`Stagger2` takes the Stagger chaos-drift VCO and turns it into a complete voice: 40 waveforms (four classic, ten band-limited additive stacks, ten waveshapers, six sync/PD/ring-mod variants, ten classic shapes), three Plaits-style macro controls with full CV, a modulated stereo delay with ping-pong option, an 8-line FDN reverb, a trigger/gate envelope, a per-voice compressor, a master gain and a master compressor for the FX sum.

The output stage is well thought out: the VCO output is polyphonic (one voice per input channel), while the FX L/R outputs are a mono sum — convenient for sending the wet signal to a separate mixer channel. A `1/sqrt(N)` poly scale keeps the summed FX level sane when many voices are active, and a NaN/inf self-heal flushes any voice whose FX state goes unstable.

The display shows the current waveform (two cycles, scaled by the envelope), an ENV meter, L/R output meters, and a four-row FX readout (delay time + feedback, delay mode, reverb decay, trigger mode). All the key editable values (frequency, waveform, delay time, reverb decay) are exposed in their knob text fields.

Compared with `Stagger`, the sound design is much more open-ended: 40 waves × 3 macros × chaos × delay × reverb × envelope is a lot of range, and the three output jacks make it usable both as a self-contained voice (VCO OUT) and as a send-effect processor (FX L/R). The main trade-offs are CPU cost (the per-voice reverb and delay are not cheap when polyphony is high) and the loss of true per-channel pitch divergence (Stagger's stereo wide drift is gone; Stagger2's width comes from the FX chain instead).