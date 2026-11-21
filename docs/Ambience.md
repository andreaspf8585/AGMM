<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Ambience.cpp
-->

# Ambience

## 1. Overview

`Ambience` is a 24 HP stereo algorithmic reverb built from a four-line feedback delay network (FDN). Its design is deliberately transparent: the signal flow is easy to follow, every block has a single clear purpose, and the feedback matrix is an orthogonal Hadamard transform so the tail cannot build up or cancel. The result is a clean, musical reverb that works well on both percussive and sustained material.

Key features:
- 4-line FDN with an orthogonal feedback matrix (energy-preserving)
- 4-stage all-pass diffusion on the input
- Per-line damping one-pole low-pass in the feedback path
- Fixed output shelving EQ (low shelf –3 dB @ 150 Hz, high shelf –4 dB @ 5 kHz)
- Shared pre-delay (0–200 ms × SPACE) with click-free read
- Freeze mode: decay → ~1 and input → 0
- CV inputs with attenuverters for Decay and Mix
- 0.5 s quadratic fade-in after every sample-rate change
- Real-time display of decay envelope, damping response and I/O meters

## Module Screenshot

![Ambience VCV Rack Module](images/Ambience.png)

## 2. Architecture

### Main Classes

- `Ambience`: Core module class inheriting from `rack::Module`
- DSP blocks:
  - `AmbLowpass2` — 2nd-order Butterworth one-pole
  - `AmbDelayLine` — fixed-capacity interpolated delay line
  - `AmbAllPass` — Schroeder all-pass for diffusion
  - `AmbModLFO` — slow sine LFO for the delay-tap modulation
  - `AmbPreDelay` — pre-delay ring buffer
  - `AmbLowShelf` — RBJ low shelf
  - `AmbHighShelf` — RBJ high shelf
- `AMPPanelBackground`: Static panel artwork
- `AMDisplayWidget`: Live scope + meters + readouts

### Inheritance

- `Ambience` inherits from `rack::Module`
- `AMPPanelBackground` and `AMDisplayWidget` inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Delays | `AmbDelayLine delays[4]` |
| Filters | `AmbLowpass2 filters[4]` |
| Pre-delay | `AmbPreDelay pre` |
| Diffusion | `AmbAllPass inputAPs[4]` |
| Modulation | `AmbModLFO modLFO` |
| EQ | `AmbLowShelf lowShelf`, `AmbHighShelf highShelf` |
| Buffers | `float outBuf[4]` (one sample per delay line) |
| Feedback | `float fb[4]` (scratch) |
| Smoothing | `float freezeAmt`, `int fadeCounter`, `int fadeSamples` |
| Display | `float meterIn`, `meterOutL`, `meterOutR`, `dispDecay`, `dispMix` |

## 3. DSP Theory

### 4-line feedback delay network

An FDN is a set of $N$ delay lines whose outputs are mixed by a feedback matrix $A$ before being written back:

$$
\mathbf{y}[n] = \sum_i g_i \mathbf{x}_i[n - D_i], \quad \mathbf{x}[n] = \mathbf{in}[n] + A \mathbf{y}[n]
$$

For stability, the eigenvalues of $A$ must have magnitude ≤ 1. The simplest energy-preserving matrix is the orthogonal Hadamard transform. This module uses a Householder-style variant (subtract 2× the mean), which is equivalent and orthonormal:

$$
A \mathbf{x} = \mathbf{x} - \frac{2}{N} \left(\sum_i x_i\right) \mathbf{1}
$$

Because it preserves energy exactly, the tail length is controlled purely by the per-line gain $g_i$.

### Per-line RT60 gain

Each delay line loses $g_i$ of its energy per round trip. To achieve a target RT60:

$$
g_i = 10^{-3 D_i / (RT60 \cdot f_s)}
$$

where $D_i$ is the delay length in samples and $f_s$ is the sample rate. This is the "decay gain" that the DECAY knob sets.

### Damping

Each line has its own 2nd-order Butterworth low-pass (RBJ cookbook, Q = 0.707) that removes high frequencies on each pass — like air absorption. The cutoff is controlled by the DAMPING knob (200 Hz .. 8 kHz).

### All-pass diffusion

Four Schroeder all-passes on the input smear transients into a dense cloud before they enter the FDN, which is what makes a reverb sound like a room rather than a set of discrete echoes. The all-pass gain is fixed at 0.6.

### Output shelving EQ

Two RBJ shelving filters shape the wet signal: low shelf –3 dB below 150 Hz, high shelf –4 dB above 5 kHz. These are fixed (not user-adjustable) and make the tail sit further back in the mix.

### Freeze

When freeze is on, the decay is pushed toward 1 and the input is cut:

$$
\text{effDecay} = \text{decay} + (0.9995 - \text{decay}) \cdot \text{freezeAmt}, \quad \text{inGain} = 1 - \text{freezeAmt}
$$

Because the decay gain is now ~1, the existing tail keeps circulating indefinitely without new input. The freeze amount is smoothed with a 20 ms one-pole so toggling never clicks.

### 0.5 s fade-in

After every sample-rate change, the module ramps its output from 0 to 1 over 0.5 s with a quadratic curve, which prevents the click that would otherwise occur when the delay buffers are cleared.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `PRE_DELAY_PARAM` | 0 | 200 ms | 20 ms | Pre-delay time |
| `SPACE_PARAM` | 0.5 | 2.5 | 1.0 | Multiplies every delay length |
| `DECAY_PARAM` | 0.5 | 0.999 | 0.92 | Feedback gain per round trip |
| `DAMPING_PARAM` | 200 Hz | 8000 Hz | 1800 Hz | Damping cutoff |
| `WET_GAIN_PARAM` | 0 | 2 | 1 | Wet-only output trim |
| `MIX_PARAM` | 0 | 1 | 0.5 | Dry/wet |
| `MOD_DEPTH_PARAM` | 0 | 6 samples | 2 | Peak deviation of read pointer |

### Switches / Inputs

| Input | Description |
|-------|-------------|
| `INPUT_L` | Left input |
| `INPUT_R` | Right input |
| `DECAY_CV_INPUT` | Decay CV (10 V adds 50%) |
| `MIX_CV_INPUT` | Mix CV (10 V adds 100%) |
| `FREEZE_INPUT` | Freeze gate |

### Switch

| Parameter | Description |
|-----------|-------------|
| `FREEZE_PARAM` | Freeze on/off |

### Outputs

| Output | Description |
|--------|-------------|
| `OUTPUT_L` | Left output |
| `OUTPUT_R` | Right output |

### Enumerations

- `ParamId`: PreDelay, Space, Decay, Damping, WetGain, Mix, ModDepth, Freeze
- `InputId`: InputL, InputR, DecayCV, MixCV, Freeze
- `OutputId`: OutputL, OutputR
- `LightId`: FreezeLight

## 5. Inputs and Outputs

### Input

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `INPUT_L` | Audio | ±10 V | Left input; if R is empty, L feeds both |
| `INPUT_R` | Audio | ±10 V | Right input |
| `DECAY_CV_INPUT` | CV | ±10 V | Decay modulation |
| `MIX_CV_INPUT` | CV | ±10 V | Mix modulation |
| `FREEZE_INPUT` | Gate | > 1 V | Freeze gate |

### Output

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUTPUT_L` | Audio | ±12 V | Left output (clamped) |
| `OUTPUT_R` | Audio | ±12 V | Right output (clamped) |

### Bypass

`configBypass(INPUT_L, OUTPUT_L)` and `configBypass(INPUT_R, OUTPUT_R)` — with no cables, input passes straight through.

## 6. Signal Processing Pipeline

1. Sample-rate setup — If the sample rate changed, reallocate every delay buffer, set `fadeSamples = 0.5 * sr`, and reset the fade counter.
2. Parameter read — Knobs and CVs.
3. Delay-length update — Compute pre-delay in samples, then `dlySamples = baseDelays[i] * space * sr`. `setDelaySamples` does not clear the buffer, so SPACE changes are click-free.
4. Damping update — Recompute each `AmbLowpass2` cutoff from `dampFreq`.
5. Modulation update — Advance the shared `AmbModLFO` and set `modOffset` for all four delay lines.
6. Input — Read L/R, mono-sum for the wet path.
7. Pre-delay — Read the pre-delay line with linear interpolation, write the new sample.
8. Diffusion — Run the input through 4 series all-passes.
9. FDN iteration — For each of the 4 lines: read (with modulation), low-pass, multiply by decay gain. Then apply the orthogonal feedback matrix. Then write input + feedback.
10. Wet mix — Combine the 4 line reads with different L/R weights.
11. EQ — Low shelf, then high shelf.
12. Wet gain — Multiply by `wetGain`.
13. Dry/wet — Mix with the dry signal.
14. Fade-in — Apply the quadratic 0.5 s ramp.
15. Clamp — Clamp to ±12 V.
16. Write — Set the two output voltages.
17. Meters — Update input/L/R peak followers with a slow decay.

## 7. Important Functions

### `applyFeedbackMatrix(float *x)`

Applies the Householder-style orthogonal matrix to a 4-element vector.

```cpp
float sum = 0;
for (int i = 0; i < kNumDelays; ++i) sum += x[i];
float factor = (2.0f / kNumDelays) * sum;
for (int i = 0; i < kNumDelays; ++i) x[i] -= factor;
```

### `AmbDelayLine::read()`

Reads at a fractional delay with linear interpolation, using the write head minus the current delay.

```cpp
float readPos = float(writePos_) - float(delaySamples_) + modOffset_;
if (readPos < 0) readPos += buf_.size();
if (readPos >= buf_.size()) readPos -= buf_.size();
int i0 = int(readPos);
int i1 = i0 + 1;
if (i1 >= (int)buf_.size()) i1 -= buf_.size();
float frac = readPos - i0;
return buf_[i0] * (1.f - frac) + buf_[i1] * frac;
```

### `AmbLowpass2::setCutoff(float freq, float sr)`

Designs an RBJ 2-pole Butterworth low-pass at `freq`.

### `process(const ProcessArgs &args)`

The full pipeline described in §6.

## 8. Mathematical Implementation

### RT60 gain

$$
g_i = 10^{-3 \cdot D_i / (RT60 \cdot f_s)}
$$

The factor 3 comes from −60 dB / 20 = −3 in log10 terms.

### Damping filter

RBJ 2-pole low-pass with $Q = 1/\sqrt{2} \approx 0.707$:

$$
\begin{aligned}
\omega_0 &= 2\pi f_c / f_s \\
\alpha &= \sin(\omega_0)/(2Q) \\
b_0 &= \frac{1 - \cos\omega_0}{2} / a_0,\quad b_1 = \frac{1 - \cos\omega_0}{a_0},\quad b_2 = \frac{1 - \cos\omega_0}{2} / a_0 \\
a_0 &= 1 + \alpha,\quad a_1 = -2\cos\omega_0 / a_0,\quad a_2 = (1 - \alpha)/a_0
\end{aligned}
$$

### Freeze decay mapping

$$
\text{effDecay} = \text{decay} + (0.9995 - \text{decay}) \cdot \text{freezeAmt}
$$

### Input gain during freeze

$$
\text{inGain} = 1 - \text{freezeAmt}
$$

### Fade-in ramp

$$
\text{gain}(n) = \left(\frac{n}{N}\right)^2, \quad N = 0.5 f_s
$$

## 9. Performance and Optimization

### CPU Usage

- 4 lines × (1 read + 1 LP + 1 mult + 1 write) ≈ 4 × 10 flops
- 4 series all-passes ≈ 4 × 4 flops
- Pre-delay read: 1 interpolated read
- EQ: 2 biquads ≈ 2 × 5 flops
- Total: ~100 flops/sample, or ~5 MFLOP/s at 48 kHz — trivial for any modern CPU

### Memory Allocation

- All buffers allocated in the constructor
- No allocation in `process()`
- SPACE changes only move read pointers — never reallocate

### Buffers

- `delays[i]`: `baseDelays_[i] * 2.5 * sr` floats ≈ 9500 floats each at 48 kHz
- `pre`: `200 ms × 2.5 × sr` ≈ 24000 floats
- Total ≈ 62 K floats ≈ 248 KB

### Real-time Processing

- All loops are bounded
- No allocation, no blocking
- Sample-rate changes are handled by a `bool srChanged` flag

### Thread Safety

- All state is written by the audio thread and read by the UI thread via torn reads
- The 0.5 s fade-in after a sample-rate change prevents clicks even if the UI is reading mid-fade

### Numerical Stability

- `tanh`-based input all-passes keep the diffused signal bounded
- Per-line decay gain $g_i < 1$ always
- Orthogonal feedback matrix preserves energy exactly
- Output clamped to ±12 V

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::plate`, `agmm::text`, `agmm::plotFrame`, `agmm::screen`, `agmm::meter`, `agmm::sectionHeader`, `agmm::chip`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`

## 11. Summary

`Ambience` is a compact, well-structured stereo algorithmic reverb built around a 4-line FDN. Its design favors clarity and musicality over parametric flexibility: the decay time is controlled by a single knob, the damping is a simple low-pass, and the output is gently voiced with fixed shelving filters. The 4-line FDN with an orthogonal Hadamard matrix is a textbook design — it is unconditionally stable, energy-preserving, and produces a dense, natural tail.

The module's distinctive features are its freeze mode (with smoothed transition), its 0.5 s quadratic fade-in after sample-rate changes, and its pre-delay × SPACE coupling (so the pre-delay scales with the room size). The CV inputs with attenuverters on Decay and Mix make it playable, and the shared pre-delay / diffusion / damping chain keeps the CPU cost very low.

Because the output is clamped to ±12 V and the input is only `tanh`-limited (not clamped), `Ambience` can be pushed into soft saturation at extreme settings — which is a musical feature, not a bug. For clean reverb use, keep the input levels moderate.
