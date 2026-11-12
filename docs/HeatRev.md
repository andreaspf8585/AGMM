<!--
AGMM AUTOMATIC DOCUMENTATION
Source: HeatRev.cpp
-->

# HeatRev

## 1. Overview

`HeatRev` is a 12 HP hybrid reverb/echo effect built from three layers:
1. A stereo delay/echo stage with feedback and L/R time spread.
2. A "Cathedral" reverb: pre-delay, early reflections, input diffusion, and an 8-line FDN with an orthonormal Hadamard feedback matrix.
3. A 2D "heat grid" diffusion network that adds a slowly moving, warm resonant texture to the tail.

Key features:
- 8-line FDN reverb (Cathedral) with Hadamard feedback
- Pre-delay, early reflections, input all-pass diffusion
- Reverb controls: Diffusion, Decay, Size, Mix, Damping, Level, Space
- Delay controls: Time, Feedback, Level, Stereo Spread
- 2D heat-grid diffusion network with live heat-map display
- Auto-scaling heat-map visualization

## Module Screenshot

![HeatRev VCV Rack Module](images/heatrev.png)

## 2. Architecture

### Main Classes

- `HeatRev`: Core module class inheriting from `rack::Module`
- `SimpleLPF`: One-pole low-pass
- `StereoDelay`: Stereo delay line with linear-interpolated fractional reads
- `Cathedral`: Pre-delay + early reflections + input diffusion + 8-line FDN
- `HRPanelBackground`: Static panel artwork
- `HRDisplayWidget`: Live heat-map display

### Inheritance

- `HeatRev` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### Structures

- `Cathedral::line[8]`: 8 delay lines (32768 samples each)
- `Cathedral::pdL/pdR`: pre-delay buffers (131072 samples each)
- `Cathedral::apL[4]/apR[4]`: input diffusers (2048 samples each)
- `StereoDelay`: L/R delay buffers with fractional read

### State (grouped)

| Group | Storage |
|-------|---------|
| Heat grid | `std::vector<float> state, next`, `int size` |
| Delay | `StereoDelay delay` |
| Reverb | `Cathedral cathedral` |
| DC block | `float dcL, dcR` |
| LP filter | `SimpleLPF lpL, lpR` |
| Display | `float viz[16][40]`, `float vizNorm`, `int vizCounter` |

## 3. DSP Theory

### Layer 1: Stereo delay

The delay is a stereo feedback delay with smoothed time:

$$
\text{smTime} \leftarrow \text{smTime} + (T - \text{smTime}) \cdot 0.0004
$$

Read positions are interpolated linearly, and the feedback is low-passed (0.4× one-pole) so each repeat gets darker.

### Layer 2: Cathedral reverb

**Pre-delay**: A circular buffer read at `(0.010 + 0.060 * space) * sr` samples back.

**Early reflections**: 7 taps per channel at fixed ms offsets (7, 19, 31, 47, 67, 89, 113 ms), with fixed gains that alternate sign:

$$
\text{erL} = \sum_k g^L_k \cdot \text{pdL}[\text{pre} + \text{ms}^L_k \cdot \text{tscale}]
$$

**Input diffusion**: 4 series Schroeder all-passes per channel.

**FDN**: 8 delay lines, each with its own damping filter and per-line RT60 gain:

$$
g_i = 10^{-3 D_i / (RT60 \cdot f_s)}
$$

Feedback matrix is the **8×8 Hadamard** matrix, normalized by $1/\sqrt{8}$ so it's orthonormal:

$$
A = \frac{1}{\sqrt{8}} H_8
$$

The FDN output is the sum of all 8 lines with alternating L/R taps.

### Layer 3: Heat grid

A 2D diffusion grid of size `size × size` (up to 64 × 64). Each sample:

1. **Inject**: left channel into column 0, right channel into row (size−1).
2. **Diffuse**: standard 5-point Laplacian:

$$
\text{next}[i] = \text{state}[i] + \text{diffK} \cdot \left(\text{state}[L] + \text{state}[R] + \text{state}[U] + \text{state}[D] - 4\text{state}[i]\right)
$$

3. **Decay**: `next *= gridFb`.
4. **Read**: average 3 cells at each side, symmetrically. Left reads around (0.28, 0.62), right around (0.62, 0.28).

### Output mix

$$
\text{out} = (1-\text{mix}) \cdot \text{in} + \text{mix} \cdot \text{level} \cdot \left(\text{fdn} + \text{er} \cdot 0.35 \cdot \text{space} + 0.35 \cdot \text{heat}\right)
$$

Followed by a `tanh` soft limit at ±8 V and clamping at ±10 V.

### Safety

If anything becomes non-finite, the module flushes the reverb and heat grid.

## 4. Parameters and Controls

### Reverb knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `DIFFUSION_PARAM` | 0 | 1 | 0.7 | Input diffusion |
| `DECAY_PARAM` | 0 | 1 | 0.8 | RT60 (0.5–15 s) |
| `SIZE_PARAM` | 4 | 64 | 48 | Size in cells |
| `MIX_PARAM` | 0 | 1 | 0.7 | Dry/wet |
| `DAMPING_PARAM` | 0 | 1 | 0.55 | Damping cutoff |
| `LEVEL_PARAM` | 0 | 1 | 0.8 | Wet level |
| `SPACE_PARAM` | 0 | 1 | 0.7 | Pre-delay + reflections + shimmer |

### Delay knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `DELAY_TIME_PARAM` | 10 ms | 500 ms | 80 ms | Delay time |
| `DELAY_FEEDBACK_PARAM` | 0 | 0.95 | 0.4 | Feedback |
| `DELAY_LEVEL_PARAM` | 0 | 1 | 0.7 | Delay level |
| `STEREO_SPREAD_PARAM` | 0 | 50 ms | 15 ms | L/R delay offset |

### Inputs

| Input | Description |
|-------|-------------|
| `INPUT_L` | Left |
| `INPUT_R` | Right |

### Outputs

| Output | Description |
|--------|-------------|
| `OUTPUT_L` | Left |
| `OUTPUT_R` | Right |

### Enumerations

- `ParamId`: Diffusion, Decay, Size, Mix, Damping, Level, DelayTime, DelayFeedback, DelayLevel, StereoSpread, Space
- `InputId`: InL, InR
- `OutputId`: OutL, OutR

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `INPUT_L` | Audio | ±10 V | Left audio |
| `INPUT_R` | Audio | ±10 V | Right audio |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUTPUT_L` | Audio | ±10 V | Left output (clamped) |
| `OUTPUT_R` | Audio | ±10 V | Right output (clamped) |

### Bypass

`configBypass(INPUT_L, OUTPUT_L)` and `configBypass(INPUT_R, OUTPUT_R)`.

## 6. Signal Processing Pipeline

1. **Sample rate update** — Set delay sample rate.
2. **Parameter read** — All knobs.
3. **Cathedral settings** — Compute RT60, scale, room size.
4. **Heat-grid settings** — Diffusion coefficient, grid feedback, LP cutoff, resize if needed.
5. **Delay settings** — L/R delay times, feedback.
6. **Delay processing** — Read delayed L/R, write new L/R with feedback.
7. **Reverb processing** — Run Cathedral with the dry + delayed input.
8. **Heat-grid injection** — Inject reverb input into grid edges.
9. **Heat-grid diffusion** — Laplacian + decay.
10. **Heat-grid read** — Average 3 cells per side.
11. **DC block + LP** — Remove DC and limit highs.
12. **Wet mix** — fdn + ER·space + 0.35·heat, scaled by level.
13. **Soft limiter** — tanh at ±8 V.
14. **Dry/wet mix** — With the original input.
15. **Output clamp** — ±10 V.
16. **Display update** — Every 512 samples, downsample the heat grid to 16×40 for the display.

## 7. Important Functions

### `StereoDelay::process(float inL, float inR, float timeSec, float fb, float sr, float &dL, float &dR)`

Smoothed-time stereo delay with linear interpolation and low-passed feedback.

### `Cathedral::process(float inL, float inR, float sr, float rt60, float damping, float scaleTarget, float diffusion, float space, float &fdnL, float &fdnR, float &erL, float &erR)`

The full Cathedral reverb: pre-delay → early reflections → input diffusion → 8-line FDN.

### `HeatRev::updateViz()`

Downsamples the heat grid to the display resolution (40 × 16) and auto-scales it.

### `HeatRev::process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Cathedral RT60 gain

$$
g_i = 10^{-3 \cdot D_i / (\text{RT60} \cdot f_s)}
$$

### Hadamard feedback

$$
A = \frac{1}{\sqrt{8}} H_8
$$

where $H_8$ is the 8×8 Hadamard matrix.

### Heat-grid diffusion

$$
\text{next}[i] = \text{state}[i] + \text{diffK} \cdot (\text{state}[L] + \text{state}[R] + \text{state}[U] + \text{state}[D] - 4\text{state}[i])
$$

### Heat-grid decay

$$
\text{next} \leftarrow \text{next} \cdot \text{gridFb}
$$

with `gridFb` clamped to `[0.99, 0.99995]`.

### Heat-grid read (L)

$$
\text{rawL} = \frac{1}{3}\left(\text{state}[0.28, 0.62] + \text{state}[0.50, 0.50] + \text{state}[0.72, 0.35]\right)
$$

### Heat-grid read (R)

$$
\text{rawR} = \frac{1}{3}\left(\text{state}[0.62, 0.28] + \text{state}[0.50, 0.50] + \text{state}[0.35, 0.72]\right)
$$

### DC block

$$
y[n] = x[n] - x[n-1] + 0.998 y[n-1]
$$

### Final mix

$$
\text{out} = (1-\text{mix})\text{in} + \text{mix} \cdot \text{level} \cdot (\text{fdn} + \text{er} \cdot 0.35 \cdot \text{space} + 0.35 \cdot \text{heat})
$$

## 9. Performance and Optimization

### CPU Usage

- Cathedral: 8 delay lines × (read + damping + write + Hadamard) ≈ 8 × 20 flops = 160
- Pre-delay + early reflections: ~30 flops
- Input diffusion: 4 × 2 × all-pass ≈ 60 flops
- Heat grid: size² × 10 flops (up to 64 × 64 × 10 = 40 K flops) — the dominant cost
- Delay: ~40 flops
- Filters + output: ~30 flops
- Total: ~42 K flops/sample at size = 64 — borderline for some CPUs

### Memory Allocation

- Cathedral: 8 × 32768 + 2 × 131072 + 8 × 2048 ≈ 540 K floats ≈ 2.2 MB
- Heat grid: 64 × 64 × 2 = 8192 floats ≈ 33 KB
- Delay: 2 × 57.6 K floats ≈ 460 KB

### Buffers

- Cathedral FDN: 262144 floats
- Cathedral pre-delay: 262144 floats
- Cathedral allpasses: 16384 floats
- Heat grid: 8192 floats
- Delay: 115200 floats

### Real-time Processing

- Heat grid is O(size²) per sample
- At size = 64, this is 4096 cells × 10 flops = 40 K flops/sample
- The heat grid resizes on knob change (allocates new vectors), which could cause a glitch — but only on user input

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads
- The display grid (`viz`) is updated every 512 samples

### Numerical Stability

- Cathedral: FDN gain < 1, orthogonal matrix
- Heat grid: feedback clamped to < 1
- Output: tanh + clamp
- Non-finite safety net: reset everything

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<vector>`, `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<utility>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`

## 11. Summary

`HeatRev` is a hybrid reverb/echo that combines three layers: a stereo delay, a Cathedral 8-line FDN reverb, and a 2D heat-grid diffusion network. The heat grid is the distinctive feature — it's a slow-moving Laplacian diffusion that adds a warm, evolving resonant texture to the tail. The live heat-map display shows the grid state as a matrix of cyan (positive) and violet (negative) cells.

The DSP is sophisticated: the Cathedral reverb uses an orthonormal Hadamard feedback matrix, the heat grid uses a 5-point Laplacian with feedback decay, and the whole thing is soft-limited to avoid runaway. The heat grid is the most interesting part — it's essentially a physical model of heat diffusion in a 2D plate, and it gives the reverb a character you won't find anywhere else.

The main limitation is CPU: at size = 64, the heat grid alone is 40 K flops/sample, which can strain slower CPUs. For most patches, size = 32 is a better balance. Also, the heat grid resizes on knob change (allocates new vectors), which can cause a small glitch.
