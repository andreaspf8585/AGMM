<!--
AGMM AUTOMATIC DOCUMENTATION
Source: AMB.cpp
-->

# AMB

## 1. Overview

`AMB` is a 36 HP generative ambient machine in the spirit of Brian Eno's AMB-1 tapes. It composes by itself: it picks a chord every few seconds, and against that chord it plays eight independent layers — a slow detuned pad, eight long FM bell loops of unequal prime lengths, a sub bass with a low rumble, six Euclidean heartbeat layers, formant-filtered "ah-oh-oo" choir voices, sparse music-box plucks, pitched wind swells, and shimmering high glass partials. The whole mix then runs through a tape ping-pong echo, a granular cloud, a probabilistic glitch stage, and a very long reverb. Nothing ever repeats exactly.

Key features:
- Ten scales, automatic chord progression with 7 s equal-power cross-fade
- Eight layers (PAD, BELL, BASS, PULSE, CHOIR, PLUCK, AIR, GLASS) with independent levels
- Formant-filtered choir (PolyBLEP saws through moving vowel filters)
- Eight long bell loops of unequal prime lengths (17, 23, 29, 31, 37, 41, 43, 47 beats)
- Six Euclidean heartbeat layers
- Tape-style ping-pong echo with wow, flutter, drift, saturating feedback
- Granular cloud with 28 voices, pitch scatter (octaves, fifths, fourths, reversal)
- Probabilistic glitch stage: stutter, reverse, bit-crush, tape stop, dropout
- 8-line FDN reverb with Hadamard matrix (RT60 up to 32 s), freezable
- Wide 500 × 124 display: piano roll, spectrum, goniometer, grain buffer, meters
- CV / gate outputs: chord root, bell pitch, bell trigger, clock, glitch gate, level

## Module Screenshot

![AMB VCV Rack Module](images/amb.png)

## 2. Architecture

### Main Classes

- `AMB` — Rack module class inheriting from `rack::Module`
- `AmbEngine` — the entire generative engine (no Rack dependencies)
- `ambdsp::DelayLine`, `ambdsp::Allpass`, `ambdsp::Svf` — DSP primitives
- `ambdsp::sawPb` — PolyBLEP band-limited saw
- `ambdsp::Fft` — 2048-point radix-2 FFT (for the spectrum display)
- `ambdsp::Reverb` — 8-line FDN with Hadamard mixing
- `ambdsp::Echo` — tape-style ping-pong delay
- `ambdsp::Granular` — 28-voice granular cloud
- `ambdsp::Glitch` — stutter / reverse / crush / tape-stop / dropout
- `AMBPanel` — static panel artwork
- `AMBDisplay` — 500 × 124 composite display

### Inheritance

- `AMB` inherits from `rack::Module`
- `AmbEngine` is a standalone struct held by value
- `AMBPanel` and `AMBDisplay` inherit from `rack::TransparentWidget`

### Structures

- `ambdsp::DelayLine` — fractional delay line (linear interpolation)
- `ambdsp::Allpass` — Schroeder allpass with fixed integer delay
- `ambdsp::Svf` — Chamberlin state-variable filter (band-pass output)
- `ambdsp::Fft` — 2048-point FFT with precomputed twiddle factors and Hann window
- `ambdsp::Granular::Grain` — one grain descriptor (dist, rate, ratio, amp, age, len, pan)
- `AmbEngine::NoteEvent { double born, end; float pitch; int layer; }`
- `AmbEngine::PadSet { float note[4]; float phase[8]; float freq[8]; float gain; }`

### State (grouped)

| Group | Storage |
|-------|---------|
| Parameters | 26 params (key, scale, tempo, chord interval, 8 layer levels, echo/reverb/grain/glitch, dark, freeze) |
| Transport | `double beatPhase`, `int lastBeat`, `double time` |
| Pad | `PadSet pad[2]`, `int padCur`, `float padFade`, `padLfo[8]`, `padLfoRate[8]`, `padTimer`, `lastDegree` |
| Bells | `bellPh[8]`, `bellModPh[8]`, `bellEnv[8]`, `bellAge[8]`, `bellFreq[8]`, `bellRatio[8]`, `bellVel[8]`, `bellPan[8]`, `bellCycle[8]` |
| Bass | `bassPitch, bassTarget, bassPhase, bassLfo`, `texLp1, texLp2` |
| Pulse | `euclidStep[6]`, `pulseEnv[6]`, `pulsePhase[6]`, `tickEnv[6]`, `tickLp[6]`, `euclidPat[6][32]`, `euclidLen[6]` |
| Choir | `choirTarget[4]`, `choirSemi[4]`, `choirF[8]`, `choirPh[8]`, `choirSvf[2][3]`, `formF[3]`, `vowelPh`, `choirSwellPh`, `choirVibPh` |
| Pluck | `plPh[6]`, `plModPh[6]`, `plEnv[6]`, `plMod[6]`, `plAge[6]`, `plFreq[6]`, `plPan[6]`, `plVel[6]`, `plNext`, `lastPluckStep`, `pluckDensPh` |
| Air | `airOn`, `airT`, `airLen`, `airIdle`, `airEnv`, `airFc`, `airF1`, `airF2`, `airSvf[2][2]` |
| Glass | `glassTarget[4]`, `glassF[4]`, `glPh[12]`, `glLfo[12]`, `glRate[12]`, `glAmp[12]`, `glassFade`, `glassPending` |
| Smoothed controls | `sPad, sBell, sBass, sPulse, sChoir, sPluck, sAir, sGlass, sEchoMix, sRevMix, sDark, sGrain` |
| Effects | `echo`, `reverb`, `gran`, `glitch` |
| Master | `mLp1L, mLp2L, mLp1R, mLp2R`, `dcXL, dcYL, dcXR, dcYR` |
| Analysis | `fft`, `anaRing[2048]`, `anaPos`, `anaHop`, `bandLo[32]`, `bandHi[32]`, `bandTilt[32]`, `bandHold[32]`, `spec[32]`, `specPk[32]`, `gonL[192]`, `gonR[192]`, `gonPos`, `corr`, `peakL, peakR`, `peakHoldL, peakHoldR` |
| History | `NoteEvent hist[320]`, `histPos` |
| RNG | `uint32_t rngState` |

## 3. DSP Theory

### Transport

The engine runs a beat clock at `TEMPO` BPM. Beat phase is fractional and drives every time-based layer.

### Chord scheduling

Every `CHORD` seconds (4 – 60 s, default 24 s) the engine picks a new chord from the current scale, and cross-fades between the old `PadSet` and the new one over 7 s with an equal-power curve:

$$
g_{\text{new}} = \sin\!\left(\tfrac{\pi}{2} t\right), \quad g_{\text{old}} = \cos\!\left(\tfrac{\pi}{2} t\right)
$$

### Scale degrees

Degrees wrap through the scale table, with an exact integer divisor for negatives:

$$
\text{degSemis}(d) = \text{INTERVALS}[(d \bmod n)] + 12 \lfloor d/n \rfloor
$$

### Pad (LAYER_PAD)

Four chord tones (root, third, fifth, colour), each rendered as two detuned voices. Every voice is a three-partial additive sine with one LFO modulating brightness and amplitude:

$$
v = \sin\theta + 0.5\, h_b \sin 2\theta + 0.28\, h_b \sin 3\theta, \quad h_b = \text{bright}\cdot(0.35 + 0.65\, \text{lfo})
$$

$$
\text{bright} = 0.06 + 0.55(1 - \text{DARK})
$$

A 37 s breath LFO modulates the total pad gain.

### Bells (LAYER_BELL)

Eight loops of unequal prime lengths (17, 23, 29, 31, 37, 41, 43, 47 beats). A new bell is fired at each loop boundary with 78 % probability. Each bell is an FM oscillator with envelope `exp(−t/3.8)`:

$$
\text{idx} = (0.35 + 0.9\,(1 - \text{DARK})) \cdot \text{vel} \cdot \text{env}^2 \cdot 1.6
$$
$$
\text{carrier} = \sin(2\pi\theta + \text{idx} \cdot \sin(2\pi\theta_m))
$$

### Bass (LAYER_BASS)

A sine wave with 22 % second harmonic, a slow 0.5 Hz amplitude LFO, and a two-pole low-passed noise rumble at 90 Hz.

### Pulses (LAYER_PULSE)

Six Euclidean layers with cycle lengths/pulses: (32,5), (24,4), (16,2), (28,3), (20,5), (12,3). Each hit fires a low-frequency sine "kick" (0.28 s decay) and a filtered noise "tick" (0.03 s decay).

### Choir (LAYER_CHOIR)

Four chord voices (root + third + fifth + colour, one octave up), each rendered as two detuned PolyBLEP saws. Each channel (L, R) then goes through three formant band-pass filters driven by a slow "ah-oh-oo-eh-ee" vowel morph. Peak band-pass gain is normalised by `1/Q`.

### Pluck (LAYER_PLUCK)

Sparse music-box plucks, triggered by a slow density LFO (`0.40 + 0.60·sin`) against twice the beat rate. Each pluck is a two-operator FM bell with envelope `exp(−t/1.2)` and modulator `exp(−t/0.16)`.

### Air (LAYER_AIR)

Pitched noise swells. On trigger, the engine picks a chord tone + 12 semitones + (0 or 12), then band-passes white noise at that frequency (plus a fifth) with two band-pass filters. Envelope follows `sin²(π t / L)` with `L` between 7 and 16 s; idle time between swells is 1 – 7 s.

### Glass (LAYER_GLASS)

Four chord tones, one octave up, each rendered as three sine partials with slowly modulated amplitudes (12 individual LFOs incommensurate). During a chord change, glass fades out over 2.5 s, retargets, and fades back in over 4 s.

### Granular (post-dry, pre-reverb)

Records the dry + half-echo mix into a 4 s ring buffer, plays up to 28 overlapping grains from random positions. Pitch scatter options: octave down/up, fourth up, fifth up, two octaves up; 25 % chance per spray to reverse. Grain window is `(1 − x²)²`, pan via equal-power `cos/sin`.

### Glitch (post-granular, pre-reverb)

Scheduled on 16th notes (probability per step = `PROB × 0.25`, gated by ≥ 0.35 s since the last glitch) or fired by the GLT IN trigger. Five types:
- **Stutter** — replays a 30 – 1000 ms frozen slice, optionally accelerating
- **Reverse** — plays the frozen slice backwards
- **Crush** — bit-crush (hold time + quantization)
- **Tape stop** — plays the buffer with a quadratically decaying playback speed
- **Dropout** — mutes to silence for the slice duration

All types cross-fade in/out with 2 × `edge` (≈ 6 ms) fade samples.

### Echo (post-dry, pre-granular/reverb)

Stereo ping-pong with independent L/R delay times, wow + flutter + drift LFOs, a one-pole low-pass (frequency = `2800 · 0.35^DARK`) and high-pass at 150 Hz in the feedback path, `tanh` soft clip. Right channel feeds left, left feeds right.

### Reverb (final wet path)

8-line feedback delay network with the orthonormal 8 × 8 Hadamard matrix (`HH^T = I`, scaled by `1/√8`). Per-line RT60 gain `10^{−3·d_i / (f_s·T)}` with `T = 0.8·40^{DECAY}`, one-pole damping at `fc = 12000 · 0.0667^{DAMP}`, slow LFO read-position modulation. Three input allpasses per side and a 30 ms pre-delay. **Freeze** sets `g_i = 1`, disables modulation, mutes input.

### Master

Two-pole low-pass whose cutoff follows `DARK` (`f_c = 12000·0.06^{DARK}`), DC blocker at 12 Hz, `tanh(1.1 x)` soft saturation.

### Analysis

2048-point FFT every 1024 samples: log-spaced bands (32 bands, 30 Hz – 16 kHz) with tilt normalisation and fast-attack/slow-release smoothing. Peak-hold with 0.7 s hold time. Goniometer ring (192 points), stereo correlation, and peak meters with 1.2 s hold.

### RNG

xorshift32 with a fresh seed on patch load; `gran` uses its own seed derived from the main one so they're decorrelated.

## 4. Parameters and Controls

### Row 1 — COMPOSE | LAYERS

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `KEY_PARAM` | 0 | 11 | 0 | Key (C, C#, D … B), snapped |
| `SCALE_PARAM` | 0 | 9 | 1 | Scale (Major, Minor, Dorian, Phrygian, Lydian, Mixolydian, Harm Minor, Locrian, Min Pent, Whole Tone) |
| `TEMPO_PARAM` | 40 | 180 | 72 | Tempo in BPM |
| `CHORD_INT_PARAM` | 4 | 60 | 24 | Chord change interval (s) |
| `PAD_LEVEL_PARAM` | 0 | 1 | 0.60 | Pad level |
| `BELL_LEVEL_PARAM` | 0 | 1 | 0.45 | Bell level |
| `BASS_LEVEL_PARAM` | 0 | 1 | 0.50 | Bass level |
| `PULSE_LEVEL_PARAM` | 0 | 1 | 0.25 | Pulse (heartbeat) level |
| `CHOIR_LEVEL_PARAM` | 0 | 1 | 0.40 | Choir level |
| `PLUCK_LEVEL_PARAM` | 0 | 1 | 0.35 | Pluck level |
| `AIR_LEVEL_PARAM` | 0 | 1 | 0.30 | Air level |
| `GLASS_LEVEL_PARAM` | 0 | 1 | 0.30 | Glass level |

### Row 2 — ECHO | REVERB | GRAINS | MASTER

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `ECHO_TIME_PARAM` | 0.12 | 1.2 | 0.50 | Echo time (s) |
| `ECHO_FBK_PARAM` | 0 | 0.9 | 0.60 | Echo feedback |
| `ECHO_MIX_PARAM` | 0 | 1 | 0.45 | Echo mix |
| `REVERB_DECAY_PARAM` | 0 | 1 | 0.70 | Reverb decay (RT60 0.8 – 32 s) |
| `REVERB_DAMP_PARAM` | 0 | 1 | 0.55 | Reverb damping |
| `REVERB_MIX_PARAM` | 0 | 1 | 0.60 | Reverb mix |
| `GRAIN_MIX_PARAM` | 0 | 1 | 0.35 | Grain mix |
| `GRAIN_SIZE_PARAM` | 0.03 | 0.8 | 0.20 | Grain size (s) |
| `GRAIN_DENS_PARAM` | 1 | 40 | 9 | Grain density (grains/s) |
| `GRAIN_SPRAY_PARAM` | 0 | 1 | 0.35 | Pitch scatter amount |
| `DARK_PARAM` | 0 | 1 | 0.60 | Master tone (low = bright, high = dark) |
| `FREEZE_PARAM` | 0 / 1 | — | — | Latching reverb freeze |

### Row 3 — GLITCH | I/O

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `GLITCH_PROB_PARAM` | 0 | 1 | 0.15 | Glitch probability per 16th note |
| `GLITCH_LEN_PARAM` | 0 | 1 | 0.40 | Glitch slice length, exponential (≈ 0.125 → 1 beat) |

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `GLITCH_INPUT` | Trigger | 0..10 V | Rising edge fires a glitch immediately |
| `FREEZE_INPUT` | Gate | 0..10 V | > 1 V freezes the reverb (OR with the button) |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUT_L_OUTPUT` | Audio | ±5 V | Left stereo audio |
| `OUT_R_OUTPUT` | Audio | ±5 V | Right stereo audio |
| `CHORD_OUTPUT` | CV | 1 V/oct | Chord root, C4 = 0 V |
| `BELL_OUTPUT` | CV | 1 V/oct | Pitch of the last bell |
| `TRIG_OUTPUT` | Trigger | 0 / 10 V | 5 ms pulse per new bell |
| `CLOCK_OUTPUT` | Trigger | 0 / 10 V | 5 ms pulse per beat |
| `GLITCH_OUTPUT` | Gate | 0 / 10 V | +10 V while a glitch is running |
| `LEVEL_OUTPUT` | CV | 0..10 V | Peak output level |

### Lights

- `FREEZE_LIGHT` — illuminated while frozen (button or gate).

### Menu

No menu options.

## 6. Signal Processing Pipeline

Per sample, `AmbEngine::process(dt)` runs:

1. **Smoothing** — 50 ms one-pole on all 12 level / mix / dark targets.
2. **Transport** — advance `beatPhase`; update `beatCount` on integer beat boundaries.
3. **Chord scheduling** — advance `padTimer`; pick a new chord on overflow; advance pad cross-fade and `chordRoot` one-pole.
4. **Render Pad** — 2 chord sets × 4 notes × 2 detuned voices.
5. **Render Bells** — 8 loops; envs > 0.0008 only.
6. **Render Bass** — sine + second harmonic + noise rumble.
7. **Render Pulses** — 6 Euclidean layers with kick + tick envelopes.
8. **Control-rate update** (every 32 samples) — choir formants, air band-pass coefficients, glass partial amplitudes.
9. **Render Choir** — 8 PolyBLEP saws → 3 formant band-passes per side, with vowel morph and swell LFO.
10. **Render Pluck** — sparse FM music-box; 6 voices.
11. **Render Air** — pitched noise swells (band-passed white noise).
12. **Render Glass** — 4 × 3 sines; smooth fade during chord change.
13. **Dry sum** — `tanh(pad + bell + bass + pulse + choir + pluck + air + glass)`.
14. **Echo** — tape ping-pong on dry.
15. **Granular** — dry + 50 % echo, added to the mix.
16. **Glitch** — scheduled on 16ths or triggered; replaces the mix output while active.
17. **Reverb** — 8-line FDN on the mix.
18. **Master** — two-pole low-pass following DARK, DC blocker, soft limit.
19. **Analysis** — FFT, spectrum, peak hold, goniometer, stereo correlation, peak meters.

## 7. Important Functions

### `void AmbEngine::pickNewChord()`

Chooses a new chord via weighted-random degree selection (weights 0.30, 0.08, 0.14, 0.18, 0.16, 0.10, 0.04), then arms the 7 s cross-fade into the other `PadSet`. Updates the bass target, all four choir targets, glass targets (deferred), and writes new history events for pad, choir, and bass.

### `void AmbEngine::startChord()`

First-time chord: calls `pickNewChord()`, snaps `padFade = 1`, `bassPitch = bassTarget`, `choirSemi = choirTarget`, `glassFade = 1`.

### `int AmbEngine::degSemis(int deg) const`

Maps a scale degree (may be negative) to semitones above C3 using `AMB_SCALE_INTERVALS`.

### `void AmbEngine::triggerPluck()` / `void AmbEngine::startAir()` / `void AmbEngine::fireGlitch()`

Layer-specific event generators; each closes / opens matching `hist[]` events and consumes / updates RNG state.

### `void AmbEngine::updateLayerControl(float dt)`

Control-rate (32 samples) updates for the three new layers: choir vowel morph + voice frequencies with vibrato, air band-pass coefficients, glass partial amplitude LFOs.

### `void AmbEngine::analyse(float dt)` / `void AmbEngine::feedAnalysis(float l, float r, float dt)`

FFT every 1024 samples, log-band dB conversion with tilt normalisation, fast-attack/slow-release smoothing, peak-hold decay, goniometer ring push, correlation EMA, peak-hold meters.

### `void AmbEngine::process(float dt)`

Main callback described in §6.

### `void ambdsp::Reverb::process(...)` / `void ambdsp::Echo::process(...)` / `void ambdsp::Granular::process(...)` / `void ambdsp::Glitch::process(...)`

Per-sample effect blocks (see §3).

### `void AMB::process(const ProcessArgs &args)`

Rack wrapper: sets sample rate, handles key/scale change, buttons, freeze OR gate, transfers 26 parameters into `AmbEngine`, calls `engine.process(dt)`, writes 8 outputs, drives 2 trigger generators from counter diffs.

### `json_t *AMB::dataToJson()` / `void AMB::dataFromJson(json_t *)`

Persist `frozen` with the patch.

## 8. Mathematical Implementation

### Scale degree lookup

$$
\text{semis}(d) = \text{INTERVALS}\!\left[(d \bmod n + n) \bmod n\right] + 12 \lfloor d/n \rfloor
$$

### Pad cross-fade

$$
g_\text{new} = \sin\!\left(\tfrac{\pi}{2} t\right), \quad g_\text{old} = \cos\!\left(\tfrac{\pi}{2} t\right), \quad t = \min(1, \text{padFade}/7)
$$

### Pad voice

$$
\begin{aligned}
v(t) &= \sin\theta + 0.5\, h_b \sin 2\theta + 0.28\, h_b \sin 3\theta \\
h_b &= \text{bright}\,(0.35 + 0.65\, \text{lfo}_k) \\
\text{amp} &= (0.55 + 0.45\, \text{lfo}_k) \, a_k \, 0.105 \, g_\text{set}
\end{aligned}
$$

### Bell loop cycle

$$
c_b = \left\lfloor \frac{\text{beatPhase} + \text{off}_b \, L_b}{L_b} \right\rfloor,\quad L_b \in \{17, 23, 29, 31, 37, 41, 43, 47\}
$$

### Bell FM

$$
\text{idx} = (0.35 + 0.9\,(1-\text{DARK})) \cdot \text{vel} \cdot \text{env}^2 \cdot 1.6
$$

### PolyBLEP saw

$$
y = 2\text{ph} - 1 - \begin{cases} t + t - t^2 - 1, & \text{ph} < \text{inc}, t = \text{ph}/\text{inc} \\ t^2 + 2t + 1, & \text{ph} > 1 - \text{inc}, t = (\text{ph}-1)/\text{inc} \end{cases}
$$

### Euclidean rhythm

$$
\text{pat}_p[i] = ((i \cdot \text{pulses}_p) \bmod \text{steps}_p) < \text{pulses}_p
$$

### Reverb RT60 gain

$$
g_i = 10^{-3 \cdot d_i / (f_s \cdot T)}, \quad T = 0.8 \cdot 40^{\text{DECAY}}
$$

### Hadamard (orthonormal)

Three butterfly passes then scale by `1/√8` — so `H^T H = I`.

### Granular window

$$
w = \left(1 - x^2\right)^2, \quad x = 2\,\frac{\text{age}}{\text{len}} - 1
$$

### Echo (wow + flutter + drift)

$$
w = 0.0030\sin(2\pi\phi_w) + 0.0007\sin(2\pi\phi_f) + 0.0020\sin(2\pi\phi_d)
$$
$$
t_L = \text{timeSm} \cdot f_s \cdot (1 + w), \quad t_R = 1.5\,t_L \cdot (1 - 0.8 w)
$$

### Master low-pass

$$
f_c = 12000 \cdot 0.06^{\text{DARK}}, \quad \alpha = 1 - e^{-2\pi f_c dt}
$$

### Spectrum dB (per band)

$$
a = \sqrt{\sum \left(\Re^2 + \Im^2\right)} \cdot \frac{1}{N/4}, \quad \text{dB} = 20 \log_{10}(\max(a, 10^{-6})) + \text{tilt}
$$

with `tilt = 2.5·log₂(√(f₀·f₁) / 1000)` so that a pink-ish spectrum reads flat.

## 9. Performance and Optimization

### CPU Usage

- Pad: 16 oscillators (8 active × 2 chord sets, one inactive) × 3 partials + 8 LFOs
- Bells: up to 8 FM oscillators
- Choir: 8 PolyBLEP saws + 3 formant filters per side
- Pluck: up to 6 FM voices
- Glass: 12 sines + 12 LFOs at control rate
- Air: 4 SVFs on white noise
- Pulses: 6 Euclidean layers × (sine + filtered noise)
- Granular: up to 28 grains × 2 read + 2 write per sample
- Glitch: 1 tap read + 1 write per sample
- Echo: 2 delay reads + filtered feedback
- Reverb: 8 FDN lines + 6 allpasses + 2 pre-delay lines
- Master: 2 one-poles + DC + tanh + 2 one-poles + DC + tanh
- Analysis: one 2048-point FFT per 1024 samples (~48 per second at 48 kHz)

Rough estimate: 2500 – 4000 flops/sample, dominated by the granular cloud, granular + reverb, and the FFT.

### Memory Allocation

- All buffers allocated in `setSampleRate()`; no allocation in `process()`
- Reverb: 8 lines × `next_pow2(~5099 + 8)` samples × 4 B ≈ 260 KB
- Echo: 2 lines × 2.4 s × 4 B ≈ 920 KB worst case
- Granular: 2 × 4 s × 4 B ≈ 1.5 MB
- Glitch: 2 × 2 s + 2 × 1.2 s ≈ 2.5 MB
- `hist[320]` ≈ 6 KB

### Real-time Processing

- All inner loops bounded by fixed constants (8 pad voices, 8 bells, 6 pulses, 6 plucks, 8 FDN lines, 28 grains, 32 spectrum bands)
- No allocation, no locks in the audio path

### Thread Safety

- All engine state written by the audio thread
- Display reads via torn reads (cosmetic)

### Numerical Stability

- xorshift32 RNG never returns 0
- Bell env gated at 0.0008 before running
- Echo time clamped to `[8 samples, 2.2 s]`
- Glitch slice clamped to `[0.02 s, cap − 8]`
- Granular rate / span bounds checked per spawn
- Reverb feedback `g_i < 1` outside freeze; freeze sets `g_i = 1` and mutes input so the loop is lossless
- Master soft-clips with `tanh` before the output
- All output voltages clamped by Rack

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdint>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<vector>`

### External Classes

- `rack::Module`, `rack::ModuleWidget`, `rack::TransparentWidget`
- `rack::ParamQuantity` and subclasses (`KeyParamQuantity`, `TempoParamQuantity`, `ChordIntervalParamQuantity`, `EchoTimeParamQuantity`, `GrainSizeParamQuantity`, `GrainDensParamQuantity`, `GlitchLenParamQuantity`)
- `dsp::SchmittTrigger`, `dsp::PulseGenerator`
- `rack::random` (initial seed)
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::sectionHeader`, `agmm::screen`, `agmm::plotFrame`, `agmm::meter`, `agmm::chip`, `agmm::text`, `agmm::accent`, `agmm::violet`, `agmm::warm`, `agmm::bright`, `agmm::dim`, `agmm::withA`, `agmm::ALIGN_L/C/R`

## 11. Summary

`AMB` is a wide, self-contained generative ambient instrument. It schedules its own chords from a ten-scale selection, plays eight independent layers against them, and washes the result through an echo, a granular cloud, a probabilistic glitch stage, and a very long freezable FDN reverb. The prime-loop bells and the equal-power chord cross-fade guarantee that no two minutes are ever the same, while the DARK control lets you tilt the entire machine from bright shimmer to a dark, sub-dominated wash.

The 500 × 124 display is the heart of the UI: a 20 s piano-roll history with nine colour-coded layers, a real-time 32-band spectrum analyser, a goniometer, a live grain-buffer view with colour-coded pitch ratios, and stereo meters with peak hold. The GLITCH and FREEZE chips in the header make the current state readable at a glance, and the pitch / trigger outputs make the generative material easy to drive external voices in key and time.

The main trade-offs are CPU (four effects in series, 28-voice granular cloud, 2048-point FFT at ~48 Hz) and control density (26 parameters split across three 12-cell rows); once you have the layout in muscle memory it feels less like a module and more like a small composing studio. `FREEZE` combined with `GLITCH` is the sweet spot for performance — capture a drone, then let the glitch stage chew on it in time with the beat.