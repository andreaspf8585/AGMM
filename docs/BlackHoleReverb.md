<!--
AGMM AUTOMATIC DOCUMENTATION
Source: BlackHoleReverb.cpp
-->

# BlackHoleReverb

## 1. Overview

`BlackHoleReverb` is a 12 HP stereo reverb built from six parallel modulated delay lines with damping, a shared pre-delay, cross-feedback for stereo spread, and a dry/wet mix. It exposes CV control (with attenuverters) over mix, feedback, darkness and modulation, plus a live scope and L/R level meters. The "Black Hole" name refers to the way long tails swallow the dry signal at high feedback — the reverb feels like it has absorbed the source.

Key features:
- 6 modulated delay lines with per-line low-pass damping
- Shared pre-delay (80 ms fixed)
- L/R cross-feedback (0.4×) for stereo width
- CV inputs with attenuverters on mix, feedback, darkness, modulation
- Selectable CV range (bipolar ±5 V or unipolar 0–10 V) via context menu, persisted
- Soft tanh limiter at ±10 V on the output
- Live scope of the mono output, plus L/R level meters
- Non-finite safety net — any NaN/Inf becomes 0

## Module Screenshot

![BlackHoleReverb VCV Rack Module](images/blackholereverb.png)

## 2. Architecture

### Main Classes

- `BlackHoleReverb`: Core module class inheriting from `rack::Module`
- `BhPanelBackground`: Static panel artwork
- `BhDisplayWidget`: Live scope + meters + readouts

### Inheritance

- `BlackHoleReverb` inherits from `rack::Module`
- `BhPanelBackground` and `BhDisplayWidget` inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Delay lines | `float delayL[6][MAX_DELAY]`, `float delayR[6][MAX_DELAY]` (MAX_DELAY = 96000) |
| Write head | `int writeIndex` |
| Pre-delay | `float preDelayBuffer[PRE_DELAY_MAX]` (44100 samples), `int preWrite` |
| Modulation | `float modPhase[6]` |
| Damping | `float filterStoreL[6]`, `float filterStoreR[6]` |
| CV mode | `CVRange cvRangeMode` (BIPOLAR_M5_5, UNIPOLAR_0_10) |
| Display | `float scopeMin[96]`, `scopeMax[96]`, `scopeHead`, `scopeCount`, `scopeBMin`, `scopeBMax`, `peakL`, `peakR` |

## 3. DSP Theory

### Parallel modulated delay lines

The wet path is the sum of six delay lines:

$$
\text{wetL}[n] = \frac{1}{6} \sum_{i=0}^{5} \text{delayL}_i[n - D_i[n]]
$$

where each $D_i[n]$ is a base delay plus a sine modulation:

$$
D_i[n] = \text{base}_i + \text{mod}_i \cdot 240 \cdot \text{modAmt}
$$

with baseDelays = `{2741, 3347, 3823, 4421, 5101, 5897}` (samples at 48 kHz) and $\text{mod}_i = \sin(2\pi f_i t)$ with $f_i = 0.08 + 0.04 i$ Hz.

### Damping

Each line has its own one-pole low-pass in the feedback path:

$$
\text{store}[n] = \text{delay}[n](1-d) + \text{store}[n-1]d
$$

where $d = \text{damp}^2$ (so the knob has a quadratic effect on brightness).

### Cross-feedback

To create stereo width without needing a full cross-feedback matrix, the module injects a small amount of the other channel's delay into each line's feedback:

$$
\begin{aligned}
\text{delayL}_i[n] &= \text{revInL} + (fbL_i + 0.4\cdot\text{delayR}_i[n-D_i])f \\
\text{delayR}_i[n] &= \text{revInR} + (fbR_i + 0.4\cdot\text{delayL}_i[n-D_i])f
\end{aligned}
$$

The 0.4× factor is small enough that the cross-feedback is audible as width but not so large that the reverb collapses to mono.

### Pre-delay

The 80 ms pre-delay is a simple ring buffer read:

$$
\text{revInL}[n] = \text{preDelayBuffer}[(n - 0.08 f_s) \bmod f_s]
$$

The pre-delay gives the dry attack a gap before the reverb starts, which is the main cue the ear uses to judge room size.

### Output soft-limit

The output is clamped to ±10 V with a tanh:

$$
\text{out} = 10\tanh(\text{mix}/10)
$$

### CV range scaling

The module can interpret CV either as bipolar ±5 V or unipolar 0–10 V:

$$
\text{scaleCV}(v) = \begin{cases} v/10 & \text{unipolar} \\ \text{clamp}(v/5,-1,1) & \text{bipolar} \end{cases}
$$

The choice is stored in the patch and shown in the context menu.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `MIX_PARAM` | 0 | 1 | 0.5 | Dry/wet |
| `FEEDBACK_PARAM` | 0 | 0.98 | 0.50 | Feedback |
| `DAMP_PARAM` | 0 | 1 | 0.7 | Darkness (higher = darker) |
| `MOD_PARAM` | 0 | 1 | 0.5 | Modulation depth |

### Attenuverters

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| `MIX_ATTEN` | 0 | 1 | 0 | Mix CV amount |
| `FEEDBACK_ATTEN` | 0 | 1 | 0 | Feedback CV amount |
| `DAMP_ATTEN` | 0 | 1 | 0 | Darkness CV amount |
| `MOD_ATTEN` | 0 | 1 | 0 | Modulation CV amount |

### Inputs

| Input | Description |
|-------|-------------|
| `IN_L` | Left |
| `IN_R` | Right |
| `MIX_CV` | Mix CV |
| `FEEDBACK_CV` | Feedback CV |
| `DAMP_CV` | Darkness CV |
| `MOD_CV` | Modulation CV |

### Outputs

| Output | Description |
|--------|-------------|
| `OUT_L` | Left |
| `OUT_R` | Right |

### Enumerations

- `CVRange`: BIPOLAR_M5_5, UNIPOLAR_0_10
- `ParamId`: Mix, Feedback, Damp, Mod, MixAtten, FeedbackAtten, DampAtten, ModAtten
- `InputId`: InL, InR, MixCV, FeedbackCV, DampCV, ModCV
- `OutputId`: OutL, OutR

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `IN_L` | Audio | ±10 V | Left audio |
| `IN_R` | Audio | ±10 V | Right audio |
| `MIX_CV` | CV | ±10 V | Mix modulation |
| `FEEDBACK_CV` | CV | ±10 V | Feedback modulation |
| `DAMP_CV` | CV | ±10 V | Darkness modulation |
| `MOD_CV` | CV | ±10 V | Modulation depth modulation |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUT_L` | Audio | ±10 V | Left output |
| `OUT_R` | Audio | ±10 V | Right output |

### Bypass

`configBypass(IN_L, OUT_L)` and `configBypass(IN_R, OUT_R)` — so with no cables, input passes straight through.

## 6. Signal Processing Pipeline

1. Pre-delay — Write the mono sum to the pre-delay buffer, read at 80 ms.
2. Per-line read — For each of the 6 delay lines, read at `baseDelay + mod`, with linear interpolation.
3. Damping — Low-pass the delayed sample in the feedback path.
4. Cross-feed — Read the other channel's delay line at the same index.
5. Write — Write input + (damped + crossfeed) × feedback back to the delay line.
6. Wet sum — Sum the 6 delays and divide by 6.
7. Wet boost — Multiply by 1.4.
8. Dry/wet — `out = (1-mix)*in + mix*wet`.
9. Soft limit — Clamp to ±10 V with tanh.
10. Display — Peak followers, scope ring buffer.

## 7. Important Functions

### `scaleCV(float v)`

Returns the CV scaled according to the current CV range mode.

```cpp
if (cvRangeMode == UNIPOLAR_0_10) return v / 10.f;
return clamp(v / 5.f, -1.f, 1.f);
```

### `safe(float x)`

Returns `x` if it's a normal float, otherwise `0`. Used on the inputs so a broken cable never poisons the delay lines.

### `process(const ProcessArgs &args)`

The main callback described in §6.

### `dataToJson() / dataFromJson(json_t*)`

Persist the CV range mode so a patch reopens with the same setting.

## 8. Mathematical Implementation

### Modulated delay read

$$
D_i[n] = \text{base}_i + 240\cdot\text{modAmt}\cdot\sin(2\pi f_i t),\quad f_i = 0.08 + 0.04 i \text{ Hz}
$$

The 240-sample peak deviation gives about 5 ms of pitch modulation at max depth — enough for chorusing, not enough for obvious vibrato.

### Cross-feed write

$$
\begin{aligned}
\text{delayL}_i[n] &= \text{revInL} + (fbL_i + 0.4\cdot\text{delayR}_i)f \\
\text{delayR}_i[n] &= \text{revInR} + (fbR_i + 0.4\cdot\text{delayL}_i)f
\end{aligned}
$$

where $fbL_i = \text{storeL}_i$ is the damped delay output. The 0.4 factor means that at high feedback the L and R tails become correlated — but not identical — giving a stable, wide image.

### Damping squared

$$
d = \text{damp}^2
$$

The square makes the knob feel more natural: at knob = 0.5 the effective damping is 0.25, which is quite bright; at knob = 1.0 it's full darkening.

### Output tanh

$$
\text{out} = 10\tanh(\text{mix}/10)
$$

This is linear for |mix| < 3 V and saturates toward ±10 V.

## 9. Performance and Optimization

### CPU Usage

- 6 lines × (1 read + 1 LP + 1 write + 1 cross-feed) ≈ 6 × 10 flops ≈ 60 flops
- Pre-delay: 1 read + 1 write
- Modulation: 6 sin calls per sample
- Total: ~150 flops/sample, plus 6 sin calls at 48 kHz

### Memory Allocation

- All buffers are allocated once in the constructor:
  - `delayL[6][96000]`, `delayR[6][96000]` ≈ 1.15 M floats ≈ 4.6 MB
  - `preDelayBuffer[44100]` ≈ 176 KB
- No allocation in `process()`

### Buffers

- 6 × 2 × 96000 floats for the delay lines
- 44100 floats for the pre-delay
- 96 floats each for scope min/max

### Real-time Processing

- All loops are bounded
- Pre-delay is a simple ring buffer
- No allocation, no blocking

### Thread Safety

- All state is written by the audio thread and read by the UI thread via torn reads
- The `safe()` wrapper on inputs prevents NaN from broken cables from poisoning the delay lines

### Numerical Stability

- `safe()` on all input and output values
- tanh soft limit on output
- Damping in the feedback path (so high-frequency energy is always being removed)
- Cross-feed factor 0.4 < 1, so it can't build up

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::plate`, `agmm::text`, `agmm::plotFrame`, `agmm::screen`, `agmm::sectionHeader`, `agmm::meter`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`

## 11. Summary

`BlackHoleReverb` is a compact stereo reverb built on six parallel modulated delay lines. Unlike a full FDN, it doesn't use a feedback matrix; instead, it relies on the natural decorrelation of six different base delays plus a small L/R cross-feed. This gives a reverb that is simpler to reason about than a Freeverb or FDN, at the cost of a somewhat less dense tail — but with a very musical, chorusy character.

The module's main strengths are its clean, predictable parameter response (the damping-squared curve, the 0.4× cross-feed, the 0.98 feedback cap), its selectable CV range (so it works well with both bipolar LFOs and unipolar envelopes), and its robustness (non-finite safety net, tanh limiter, always-decaying feedback). The fixed 80 ms pre-delay is a reasonable default but not adjustable, which is the module's main limitation.
