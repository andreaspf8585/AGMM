<!--
AGMM AUTOMATIC DOCUMENTATION
Source: TritoneParadox.cpp
-->

# TritoneParadox

## 1. Overview

`TritoneParadox` is a 28 HP Shepard-tone pair generator for the tritone paradox illusion. Two Shepard tones separated by a tritone (6 semitones) are heard as ascending by some listeners and descending by others. The direction of perceived motion depends on the absolute pitch classes of the tones and on the position of the fixed spectral envelope that weights the octave-spaced partials.

Key features:
- Shepard-tone pair with adjustable interval (5–7 semitones)
- Adjustable Gaussian spectral envelope (centre + width)
- Optional context tone before the pair (Off / Lower / Higher)
- Manual (trigger) or Auto (free-running) sequencing
- Adjustable cycle rate (0.1–200 Hz, exponential)
- Adjustable crossfade time (1–50 ms)
- Stereo spread
- Live display: envelope with partials, pitch-class circle marking the interval
- Gate A, Gate B, EOC outputs

## Module Screenshot

![TritoneParadox VCV Rack Module](images/tritoneparadox.png)

## 2. Architecture

### Main Classes

- `TritoneParadox`: Core module class inheriting from `rack::Module`
- `PitchQuantity`, `PartialsQuantity`, `RateQuantity`: Custom ParamQuantity classes
- `ToneBank`: Struct holding partial frequencies, amps, pan gains
- `TPPanel`: Static panel artwork
- `TPDisplay`: Live display (envelope + chroma)

### Inheritance

- `TritoneParadox` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### Structures

- `ToneBank { float freq[20], amp[20], gl[20], gr[20], norm; }` — one Shepard tone
- `SeqState`: SEQ_IDLE, SEQ_CONTEXT, SEQ_TONE_A, SEQ_TONE_B

### State (grouped)

| Group | Storage |
|-------|---------|
| Tone banks | `ToneBank bankA, bankB, bankC` |
| Phases | `float phasesA[20]`, `phasesB[20]`, `phasesC[20]` |
| Sequencer | `SeqState seqState`, `float seqTimer`, `bool lastTrig` |
| Gains | `float gainA, gainB, gainCtx` (smoothed) |
| Control-rate | `int ctrlCount`, `float rateHz, smoothK, levelGain, sweepPhase` |
| EOC | `dsp::PulseGenerator eocPulse` |
| Settings | `bool monoOut` |
| Display | `float dispPitchSemis, dispEnvCenterHz, dispEnvWidth, dispTritoneSemis, dispGainA, dispGainB`, `int dispPartials, dispSeqState, dispContextMode`, `bool dispAuto` |

## 3. DSP Theory

### Shepard tone

A Shepard tone is a stack of octave-spaced partials weighted by a fixed Gaussian window in log-frequency:

$$
A_i = \exp\left(-\frac{(\log_2 f_i - \log_2 f_0)^2}{2\sigma^2}\right)
$$

where $f_i = f_0 \cdot 2^i$ for $i = 0, \ldots, N-1$.

### Spectral envelope

The envelope is a Gaussian in log-frequency space, centred at `ENV_CENTER` Hz with `ENV_WIDTH` octaves:

$$
w(f) = \exp\left(-\frac{(\log_2(f/f_c))^2}{2\sigma^2}\right)
$$

### Tone bank

For each partial `i`:

$$
f_i = f_{\text{base}} \cdot 2^i
$$

$$
A_i = w(f_i)
$$

$$
\text{pan}_i = \text{clamp}\left(\frac{\log_2(f_i/f_c)}{\sigma} \cdot \text{spread}, -1, 1\right)
$$

$$
g^L_i = \cos((\text{pan}_i + 1) \pi / 4), \quad g^R_i = \sin((\text{pan}_i + 1) \pi / 4)
$$

The bank is normalized so that its RMS is 1:

$$
\text{norm} = \frac{1.8}{\sqrt{\sum_i A_i^2}}
$$

### The tritone pair

- **Tone A**: base pitch = `PITCH` (root note + V/Oct)
- **Tone B**: base pitch = `PITCH + TRITONE` (usually +6 semitones)
- **Context tone** (optional): base pitch = `PITCH ± 12` (Lower / Higher)

### Sequencer

The sequencer advances through states on a timer:

$$
\Delta t_{\text{state}} = \frac{1}{f_{\text{rate}}}
$$

For the context state, $\Delta t_{\text{context}} = 0.8 \cdot \Delta t_{\text{state}}$.

Transitions: CONTEXT → TONE_A → TONE_B → (EOC) → CONTEXT (loop) or IDLE.

### Anti-click crossfade

Each tone's gain is smoothed toward its target (1 if the sequencer is in that state, else 0) over `smoothK`:

$$
\text{smoothK} = 1 - e^{-\Delta t / \tau}, \quad \tau = \text{fade} \cdot 0.001
$$

### Sweep

The envelope center can be swept automatically:

$$
f_c = f_{c0} \cdot 2^{\text{envCv} + \text{sweepDepth} \sin(2\pi t / 20)}
$$

This lets you explore how envelope position affects the illusion.

### Stereo rendering

Each tone is summed with its pan gains:

$$
\text{outL} += \sum_i A_i g^L_i s_i, \quad \text{outR} += \sum_i A_i g^R_i s_i
$$

where $s_i = \sin(2\pi \phi_i)$.

The two tones A and B are added together with their smoothed gains.

### Final output

$$
\text{out} = 5 \tanh(\text{out} \cdot 0.8) \cdot \text{level}
$$

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `PITCH_PARAM` | −48 | 48 | 0 | Pitch (semitones from C4) |
| `ENV_CENTER_PARAM` | 65 | 2000 | 370 | Envelope centre (Hz) |
| `ENV_WIDTH_PARAM` | 0.5 | 3 | 1.5 | Envelope width (octaves) |
| `PARTIALS_PARAM` | 1 | 20 | 6 | Number of partials |
| `TRITONE_PARAM` | 5 | 7 | 6 | Tritone interval |
| `RATE_PARAM` | 0 | 1 | 0.303 | Cycle rate (0.1–200 Hz) |
| `SWEEP_PARAM` | 0 | 2 | 0 | Envelope sweep depth (oct) |
| `FADE_PARAM` | 1 | 50 | 5 | Crossfade time (ms) |
| `SPREAD_PARAM` | 0 | 1 | 0.55 | Stereo spread |
| `LEVEL_PARAM` | 0 | 1.5 | 1 | Output level |

### Switches

| Parameter | Description |
|-----------|-------------|
| `MODE_PARAM` | Manual / Auto |
| `CONTEXT_PARAM` | Off / Lower / Higher |

### Buttons

| Button | Description |
|--------|-------------|
| `PLAY_PARAM` | Play sequence |

### Inputs

| Input | Description |
|-------|-------------|
| `TRIG_INPUT` | Trigger |
| `VOCT_INPUT` | Pitch CV (1 V/oct) |
| `ENV_CV_INPUT` | Envelope centre CV (1 V/oct) |
| `RATE_CV_INPUT` | Cycle rate CV (1 V/oct) |

### Outputs

| Output | Description |
|--------|-------------|
| `OUT_L_OUTPUT` | Left |
| `OUT_R_OUTPUT` | Right |
| `GATE_A_OUTPUT` | Tone A gate |
| `GATE_B_OUTPUT` | Tone B gate |
| `EOC_OUTPUT` | End of cycle |

### Enumerations

- `ParamId`: Pitch, EnvCenter, EnvWidth, Partials, Tritone, Mode, Rate, Context, Sweep, Fade, Spread, Level, Play
- `InputId`: Trig, VOct, EnvCV, RateCV
- `OutputId`: OutL, OutR, GateA, GateB, EOC

### Menu

- Mono output (sum L + R)

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `TRIG_INPUT` | Gate | > 1 V | Trigger sequence |
| `VOCT_INPUT` | CV | 1 V/oct | Pitch |
| `ENV_CV_INPUT` | CV | ±6 V | Envelope centre |
| `RATE_CV_INPUT` | CV | ±6 V | Cycle rate |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUT_L_OUTPUT` | Audio | ±5 V | Left |
| `OUT_R_OUTPUT` | Audio | ±5 V | Right |
| `GATE_A_OUTPUT` | Gate | 0/10 V | Tone A gate |
| `GATE_B_OUTPUT` | Gate | 0/10 V | Tone B gate |
| `EOC_OUTPUT` | Gate | 0/10 V | End of cycle |

## 6. Signal Processing Pipeline

1. **Sample-rate setup** — None needed (no delay lines).
2. **Sweep phase** — Advance by `dt / 20`.
3. **Control-rate update (every 16 samples)**:
   - Compute pitch from knob + V/Oct.
   - Compute envelope center from knob + CV + sweep.
   - Build tone banks A, B, C.
   - Compute rate from knob + CV.
   - Compute smoothing coefficient.
   - Snapshot for display.
4. **Sequencer** — Advance state if timer expired.
5. **Anti-click gains** — Smooth each tone's gain toward its target.
6. **Per-sample rendering** — For each active tone, advance phases and sum into L/R.
7. **Mono sum** — If monoOut, average L/R.
8. **Soft limit** — tanh at ±5 V.
9. **Write outputs** — L, R, gates, EOC.

## 7. Important Functions

### `tpEnvelopeWeight(float hz, float centreHz, float sigma)`

Gaussian weight for one partial.

### `buildBank(ToneBank &b, float baseSemis, float envCenterHz, float envWidth, float spread, int partials, float sr)`

Builds the tone bank: frequencies, weights, pan gains, normalisation.

### `renderBank(const ToneBank &b, float *phase, float gain, float dt, float &outL, float &outR)`

Renders one tone into L/R.

### `startSequence()`

Starts the sequencer from the beginning.

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Envelope weight

$$
w(f) = \exp\left(-\frac{(\log_2(f/f_c))^2}{2\sigma^2}\right)
$$

### Pan

$$
\text{pan}_i = \text{clamp}\left(\frac{\log_2(f_i/f_c)}{\sigma} \cdot \text{spread}, -1, 1\right)
$$

### Pan gains

$$
g^L = \cos((\text{pan}+1)\pi/4), \quad g^R = \sin((\text{pan}+1)\pi/4)
$$

### RMS normalisation

$$
\text{norm} = \frac{1.8}{\sqrt{\sum A_i^2}}
$$

### Sweep

$$
f_c = f_{c0} \cdot 2^{\text{envCv} + \text{sweepDepth}\sin(2\pi t / 20)}
$$

### Rate

$$
\text{rate} = 0.1 \cdot 2000^{\text{knob}} \cdot 2^{\text{rateCV}}
$$

### Crossfade smoothing

$$
\tau = \text{fade} \cdot 0.001 \text{ s}, \quad k = 1 - e^{-\Delta t / \tau}
$$

### Output

$$
\text{out} = 5 \tanh(\text{out} \cdot 0.8) \cdot \text{level}
$$

## 9. Performance and Optimization

### CPU Usage

- Control rate: 2 tone banks × 20 partials ≈ 200 flops
- Per sample: 20 partials × 2 tones × (sin + pan) ≈ 200 flops
- Total: ~200 flops/sample + 200 flops/16 samples (control rate)

### Memory Allocation

- All state fixed-size
- ToneBank: 20 × 4 floats × 3 banks = 240 floats
- No allocation in `process()`

### Buffers

- `ToneBank bankA, bankB, bankC`: 3 × 80 floats = 240 floats
- Phase arrays: 3 × 20 floats = 60 floats

### Real-time Processing

- All loops bounded by 20 (partials)
- No allocation, no blocking
- Control-rate updates every 16 samples

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads

### Numerical Stability

- `tanh` soft limit on output
- All frequencies clamped to `0.45 * sr`
- No feedback loops

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstdlib>`, `<cstring>`, `<algorithm>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `rack::dsp::PulseGenerator`, `rack::dsp::SchmittTrigger`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::chip`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`

## 11. Summary

`TritoneParadox` is a specialized Shepard-tone generator designed to demonstrate the tritone paradox: the illusion that a tritone pair can be heard as either ascending or descending depending on the absolute pitch classes.

The module gives you control over the two key variables of the illusion: the pitch class of the first tone (PITCH), and the spectral envelope (centre and width). Moving the envelope centre shifts the perceptual bias, and sweeping it automatically makes the illusion shift in real time.

The display is excellent for understanding the illusion: it shows the envelope with partials for both tones, and a pitch-class circle with the interval marked. You can see exactly which partials are dominant at any moment, which is the key to understanding why the illusion works.

The main limitations are: only 20 partials (enough for the illusion, but not a full synth), and the rate knob's exponential mapping makes it hard to dial in very slow cycles. But for a pedagogical/experimental module, this is excellent.