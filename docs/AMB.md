<!--
AGMM AUTOMATIC DOCUMENTATION
Source: AMB.cpp
-->

# AMB

## 1. Overview

`AMB` is a generative ambient machine inspired by Brian Eno's *Discreet Music* and the AMB-1 algorithm. It layers five independent voices under a shared key and scale, driven by a single BPM clock. The result is a slowly evolving, always-musical texture that never exactly repeats.

Key features:
- 8-voice detuned pad with per-voice LFO
- 8 tape-loop bells with mutually prime loop lengths (17–47 beats)
- Sub-bass with LFO amplitude
- 6 Euclidean pulse layers on a 32-beat cycle
- Ping-pong tape echo with cross-feedback
- Dattorro-style plate reverb with modulated tank
- Selectable key (C–B) and scale (Major, Minor, Dorian, Phrygian, Lydian, Mixolydian)
- Chord-change interval knob (4–60 s)
- Real-time note history scope

## Module Screenshot

![AMB VCV Rack Module](images/amb.png)

## 2. Architecture

### Main Classes

- `AMB`: Core module class inheriting from `rack::Module`
- `AmbEngine`: The full DSP engine (separate struct)
- `AMBPanel`: Static panel artwork
- `AMBDisplay`: Live note-history scope
- ParamQuantity subclasses: `KeyParamQuantity`, `ScaleParamQuantity`, `TempoParamQuantity`, `ChordIntervalParamQuantity`

### Inheritance

- `AMB` inherits from `rack::Module`
- `AMBPanel` and `AMBDisplay` inherit from `rack::TransparentWidget`
- The ParamQuantity subclasses inherit from `rack::engine::ParamQuantity`

### Structures

- `AmbEngine::NoteEvent { float age; float pitch; int layer; }` — one note in the display history
- `AmbEngine::Allpass` — Schroeder all-pass for the reverb diffuser
- `AmbEngine::ModAllpass` — modulated all-pass for the reverb tank
- `std::vector<NoteEvent> history` — recent note events (up to 128)

### Engine State (grouped)

| Group | Storage |
|-------|---------|
| Transport | `beatPhase`, `lastSampleRate` |
| Pad | `padPhase[8]`, `padDetune[8]`, `padLfo[8]`, `padLfoRate[8]`, `padChordRoot`, `padTargetRoot`, `padChordTimer` |
| Bells | `bellLoopPos[8]`, `bellDegree[8]`, `bellPhase[8]`, `bellEnv[8]`, `bellFreq[8]`, `bellModPhase[8]`, `bellMod`, `bellIndex` |
| Bass | `bassPhaseSub`, `bassPhaseMid`, `bassPitch`, `bassTarget`, `bassLfoPhase` |
| Pulse | `euclidStep[6]`, `pulseEnv[6]`, `pulseHatEnv[6]`, `pulsePhase[6]`, `hatLP[6]`, `euclidPat[6][32]`, `euclidLen[6]` |
| Echo | `echoBufL/R` (96000 samples), `echoWrite`, `echoTap1`, `echoTap2` |
| Reverb | `preBuf[440]`, `ap1..ap4`, `tankAP[8]`, `tankDelay[4]` |
| Output | `meterL`, `meterR`, `curChordRoot`, `outL`, `outR`, `masterLP_L/R` |
| RNG | `std::mt19937 rng`, `std::uniform_real_distribution<float> dist01` |

## 3. DSP Theory

### Transport

A single phase accumulator advances at the master BPM:

$$
\phi(t) = \phi(t-1) + \frac{\text{BPM}}{60} \cdot \Delta t
$$

All rhythm layers derive their position from this phase, so changing the BPM time-stretches every layer in sync.

### Pad chord generation

The pad plays an 8-voice detuned chord. The chord root is chosen from the current scale, folded into a one-octave-plus range (C3..E5):

$$
\text{root} = 12 + \text{scale}[\text{degree}] + \text{key} \pmod{12}, \quad \text{root} \in [12, 28]
$$

The chord glides toward its target exponentially (τ ≈ 2 s):

$$
p_{\text{chord}}(t) = p_{\text{chord}}(t-1) + \left(p_{\text{target}} - p_{\text{chord}}(t-1)\right)\left(1 - e^{-\Delta t / 2}\right)
$$

### Detuned pad

The 8 pad voices share the chord root but have a fixed detune table:

$$
D = \{-0.08, -0.055, -0.03, -0.01, 0.01, 0.03, 0.055, 0.08\}
$$

so each voice is detuned by $D[v]$ semitones. Each voice also gets a slow LFO (0.03–0.15 Hz) that modulates amplitude between 0.6 and 1.0, giving the pad a gentle breathing quality.

### Bell tape loops

Each of the 8 bells has a loop length in beats from a mutually prime set:

$$
L = \{17, 23, 29, 31, 37, 41, 43, 47\}
$$

Its position is:

$$
\text{pos}_b(t) = \phi(t) \bmod L_b
$$

On every wrap (pos_b drops below its previous value) the bell re-fires: a new scale degree and octave are rolled, the envelope is reset, and the FM phase is zeroed. Because the loop lengths share no common factor, the composite bell pattern never exactly repeats.

The bell is a phase-modulated sine:

$$
\text{bell} = \sin(2\pi\phi_c + I\sin(2\pi\phi_m))
$$

with $I = 0.5$ and $f_m = 1.5 f_c$.

### Euclidean pulses

Six layers use Euclidean rhythms $E(k, n)$ encoded on a 32-beat cycle:

$$
E(k, n)[i] = \left((i \cdot k) \bmod n\right) < k
$$

The six patterns are `{32,5}`, `{24,4}`, `{16,2}`, `{28,3}`, `{20,5}`, `{12,3}`. The step index for pattern $p$ is:

$$
\text{step}_p(t) = \left\lfloor \frac{\phi(t)}{32} \cdot n_p \right\rfloor \bmod n_p
$$

When a pulse fires, the layer spawns a short thump (30–80 Hz sine) and a decaying hat (band-passed noise) with exponential envelopes.

### Tape echo

The echo is a cross-feedback stereo delay:

$$
\begin{aligned}
\text{echoL}[n] &= \text{dry}[n] + \text{echoR}[n - T_1] f + \text{echoR}[n - T_2] \tfrac{f}{2} \\
\text{echoR}[n] &= \text{dry}[n] + \text{echoL}[n - T_1] f + \text{echoL}[n - T_2] \tfrac{f}{2}
\end{aligned}
$$

with $T_1 = 0.30\,\text{s}$, $T_2 = 1.20\,\text{s}$, and $f = \min(f_{\text{knob}}, 0.9) \cdot 0.65$. The 0.65 scaling keeps the loop stable even at maximum feedback.

### Dattorro plate reverb

The reverb is a classic Dattorro topology: four fixed all-passes diffuse the input, then a tank of eight modulated all-passes and four delay lines provides a dense, slowly evolving tail. Feedback is $0.30 + 0.40 \cdot \text{decay}$ per round trip, capped at 0.70 to ensure the tail decays within ~10 s.

### Output chain

$$
\text{out} = \tanh\left(\text{LP}_{4\text{kHz}}\left(\text{mix}_{\text{echo}} + \text{mix}_{\text{reverb}}\right)\right) \cdot 5
$$

The 4 kHz master low-pass keeps the tail from sounding brittle, and the `tanh` limits the output to ±5 V.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `KEY_PARAM` | 0 | 11 | 0 | Key note (C–B) |
| `SCALE_PARAM` | 0 | 5 | 0 | Scale |
| `TEMPO_PARAM` | 40 | 180 | 120 | BPM |
| `PAD_LEVEL_PARAM` | 0 | 1 | 0.55 | Pad volume |
| `BELL_LEVEL_PARAM` | 0 | 1 | 0.55 | Bell volume |
| `BASS_LEVEL_PARAM` | 0 | 1 | 0.55 | Bass volume |
| `PULSE_LEVEL_PARAM` | 0 | 1 | 0.50 | Pulse volume |
| `ECHO_MIX_PARAM` | 0 | 1 | 0.40 | Echo dry/wet |
| `ECHO_FBK_PARAM` | 0 | 0.9 | 0.55 | Echo feedback |
| `REVERB_MIX_PARAM` | 0 | 1 | 0.55 | Reverb dry/wet |
| `REVERB_DECAY_PARAM` | 0 | 1 | 0.55 | Reverb RT60 (0.3–0.7) |
| `CHORD_INT_PARAM` | 4 | 60 | 24 | Chord change interval (s) |

### Buttons / Switches

None.

### Outputs

| Output | Type | Range | Description |
|--------|------|-------|-------------|
| `OUT_L_OUTPUT` | Audio | ±5 V | Left |
| `OUT_R_OUTPUT` | Audio | ±5 V | Right |

### Enumerations

- `ParamId`: Key, Scale, Tempo, Pad, Bell, Bass, Pulse, EchoMix, EchoFbk, ReverbMix, ReverbDecay, ChordInterval
- `InputId`: none
- `OutputId`: OutL, OutR
- `LightId`: none

## 5. Inputs and Outputs

### Input

None — the module is a self-contained generator with no CV inputs.

### Output

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUT_L_OUTPUT` | Audio | ±5 V | Left output |
| `OUT_R_OUTPUT` | Audio | ±5 V | Right output |

### Signal Levels

All five layers are summed and soft-clipped with `tanh`, so the output can never exceed ±5 V. Each layer's level knob goes from silence (0) to full (1); with all layers at 1, the output is heavily saturated (intentional — Eno-style ambience likes a bit of compression).

## 6. Signal Processing Pipeline

1. Scale change detection — If the key or scale knob changed since the last sample, call `pickNewChord()` immediately.
2. Transport — `beatPhase += (BPM / 60) * dt`.
3. Pad chord timer — When `padChordTimer >= chordInterval`, pick a new chord.
4. Pad synthesis — For each of the 8 voices: update LFO, compute frequency, update phase, accumulate into pad L/R with pan.
5. Bell synthesis — For each of the 8 bells: update loop position; on wrap, re-arm envelope and re-roll degree; if envelope is active, render an FM sine bell.
6. Bass synthesis — Slew `bassPitch` toward `bassTarget`, generate sub + mid sines with harmonics, apply LFO amplitude.
7. Pulse synthesis — For each of the 6 Euclidean patterns: advance step index; on a pulse, trigger thump and hat envelopes; render active envelopes.
8. Dry sum — Mix the four layers with their level knobs, apply `tanh`.
9. Echo — Read two taps from the shared delay line, write back the cross-feedback, mix dry/wet.
10. Reverb — Pre-delay → 4 fixed all-passes → tank (8 modulated all-passes + 4 delay lines) → mix dry/wet.
11. Master — Final 4 kHz low-pass + `tanh` + ×5 V scaling.
12. UI snapshot — Update meters, advance trail ages, push new note events to the history buffer.

## 7. Important Functions

### `pickNewChord()`

Chooses a new pad chord root and bass note within the current scale, re-rolls all 8 bell degrees, and pushes the pad and bass notes into the note history.

```cpp
int octave = 1 + (dist01(rng) < 0.6f ? 0 : 1);
int degree = std::min(6, (int)(dist01(rng) * 7.f));
int semitone = octave * 12 + scale[degree] + key;
while (semitone > 28) semitone -= 12;
while (semitone < 12) semitone += 12;
padTargetRoot = (float)semitone;
```

### `addHistory(float pitch, int layer)`

Pushes a note event into the 128-entry ring buffer, dropping the oldest if full.

```cpp
if (history.size() >= HIST_MAX)
    history.erase(history.begin());
history.push_back({ 0.f, pitch, layer });
```

### `process(float sampleRate, float dt)`

The full 12-step audio loop described in §6.

### `Allpass::process(float x)`

Standard Schroeder all-pass used in the input diffuser.

```cpp
float yn = buf[idx];
float vn = x + g * yn;
buf[idx] = vn;
idx = (idx + 1) % n;
return yn - g * vn;
```

### `ModAllpass::process(float x, float sr)`

Same as `Allpass` but with a sine-modulated read index for the tank diffusers.

## 8. Mathematical Implementation

### Pad detune table

$$
D = \{-0.08, -0.055, -0.03, -0.01, 0.01, 0.03, 0.055, 0.08\}
$$

### Bell pitch

$$
f_{\text{bell}} = 130.81 \cdot 2^{\left(\text{oct} \cdot 12 + \text{scale}[\text{deg}] + \text{key}\right)/12}
$$

with `oct` ∈ {1, 2}.

### Bell FM

$$
\text{out} = \sin\left(2\pi\phi_c + 0.5 \sin(2\pi\phi_m)\right), \quad f_m = 1.5 f_c
$$

### Bass voice

$$
\text{bass} = 0.5 \sin\phi_s + 0.15 \sin 2\phi_s + 0.075 \sin 3\phi_s + 0.15 \sin 2\phi_m + 0.0375 \sin 4\phi_m
$$

with $\phi_s = 2\pi f_{\text{sub}} t$ and $\phi_m = 2\phi_s$.

### Euclidean generation

```cpp
for (int i = 0; i < steps; i++)
    euclidPat[p][i] = ((i * pulses) % steps) < pulses;
```

### Reverb feedback bound

$$
f_{\text{reverb}} = 0.30 + 0.40 \cdot \text{decay} \le 0.70
$$

### Echo feedback scaling

$$
f_{\text{echo}} = \min(\text{knob}, 0.9) \cdot 0.65
$$

### Final output

$$
\text{out} = \tanh\left(\text{LP}_{4\text{kHz}}\left(\text{mix}\right)\right) \cdot 5
$$

## 9. Performance and Optimization

### CPU Usage

- Bells: 8 × (sin + FM sine + envelope + exp decay) ≈ 8 × 25 flops
- Pad: 8 × (sin + LFO sine + phase wrap) ≈ 8 × 15 flops
- Pulse: 6 × (conditional thump + conditional hat) ≈ 6 × 10 flops
- Reverb: 8 tank all-passes + 4 delay lines + 4 pre-delay all-passes ≈ 200 flops
- Echo: 2 tap reads + 2 writes ≈ 20 flops
- Total: ~600–800 flops/sample plus ~30 sin calls

### Memory Allocation

- All delay buffers are allocated once in the `AmbEngine` constructor
- No allocation occurs in `process()`
- `history` is a `std::vector` reserved to `HIST_MAX + 2`, so `push_back` never reallocates after warm-up

### Buffers

- `echoBufL` / `echoBufR`: 96000 floats each (~384 KB each)
- `preBuf`: 440 floats
- `tankDelay[4]`: 4453 + 3720 + 4217 + 3163 ≈ 15.5 K floats
- `tankAP[8]`: 672–2656 floats each
- Total ~100 K floats ≈ 400 KB

### Real-time Processing

- All loops have fixed, bounded iteration counts
- No blocking operations or allocations in `process()`
- The engine is fully sample-deterministic

### Thread Safety

- All engine state is written by the audio thread and read by the UI thread via torn reads (documented as "cosmetic only")
- The note history is a separate ring buffer that's read-only from the UI

### Numerical Stability

- All voices use `tanh` soft clipping in the output path
- Master `tanh` limits to ±5 V
- Bells and bass use exponential-decay envelopes that decay to zero
- The 0.65 echo-feedback scaling prevents runaway
- The 0.70 reverb feedback cap prevents runaway

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers (`panelBase`, `bezel`, `cell`, etc.)
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<vector>`, `<random>`

### External Classes

- `rack::Module` — base module class
- `rack::TransparentWidget` — transparent overlay widget
- `rack::engine::ParamQuantity` — parameter descriptor
- `rack::random::uniform()` — RNG utility
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::screen`, `agmm::plotFrame`, `agmm::text`, `agmm::meter`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::dim`, `agmm::bright`, `agmm::warm`, `agmm::textCol`, `agmm::ALIGN_L/C/R`

## 11. Summary

`AMB` is a generative ambient module that combines five independent synthesis layers under a shared key and scale. Its design emphasizes slow, organic evolution: chord changes every 4–60 s, mutually prime bell loop lengths, and a wandering pad detune table all contribute to a texture that never exactly repeats.

The DSP is built from small, focused primitives (all-pass, modulated all-pass, one-pole LP, ring buffer) and combined into larger structures (pad, bells, pulses, echo, reverb) that are each individually simple but together produce a rich, layered sound. All loops are bounded, all buffers are pre-allocated, and soft clipping at every stage ensures the output stays within ±5 V.

Because it has no CV inputs, `AMB` is meant to be a self-contained texture source — patch its stereo output into a mixer or another effect and let it run. The two knobs that most change its character are **CHORD INT** (how often the harmony shifts) and **SCALE** (the mood of the harmony).
