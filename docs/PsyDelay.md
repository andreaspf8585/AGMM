<!--
AGMM AUTOMATIC DOCUMENTATION
Source: PsyDelay.cpp
-->

# PsyDelay

## 1. Overview

`PsyDelay` is a 36 HP psychedelic stereo delay with a rich set of self-modulation features. Its core is a modulated stereo delay line (up to 2 seconds) with feedback, reverse playback, ping-pong and phase-flip stereo routing, plus an input gate and a chain of character effects on the wet path: time modulation, cross-mod, glitch, ring mod, sample-rate reduction, bitcrush, feedback/howl, freeze, drive, colour LPF.

Key features:
- Stereo delay up to 2 s
- Reverse playback, ping-pong, phase-flip routing
- Input gate with threshold
- Chaos LFO (logistic map) or normal sine LFO for time modulation
- Cross-mod (right channel offset from left feedback)
- Glitch (random delay jump with smoothing)
- Ring mod (50 Hz carrier)
- Sample-rate reduction
- Bitcrusher (6 bits)
- Howl (feedback boost with tanh signal)
- Freeze
- Drive (tanh saturation)
- Colour LPF
- Live delay-line timeline display with tap markers and state chips

## Module Screenshot

![PsyDelay VCV Rack Module](images/PsyDelay.png)

## 2. Architecture

### Main Classes

- `PsyDelay`: Core module class inheriting from `rack::Module`
- `PDPanelBackground`: Static panel artwork
- `PDDisplayWidget`: Live delay-line timeline display

### Inheritance

- `PsyDelay` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Delay buffers | `std::vector<float> delayL`, `delayR` (2 s + 1 sample) |
| Write head | `int writePos` |
| Sample rate | `float sampleRate` |
| Output filters | `float outFilterL`, `outFilterR` |
| LFO | `float lfoPhase` |
| Chaos | `float chaosState` |
| Ring mod | `float ringPhase` |
| Glitch | `std::mt19937 rng`, `std::uniform_real_distribution<float> dist`, `float glitchFade, glitchOldL, glitchOldR` |
| SRR | `float srrCounter`, `lastWetL`, `lastWetR` |
| Gate | `float gateEnv, gateGain` |
| Filter init | `bool filterInitialised` |
| Display | `float peakBins[130]`, `int peakWriteBin, lastBin`, `float dispReadAge`, `dispMod`, `glitchFlash`, `meterL, meterR`, `bool dispGateActive, dispGateOpen` |

## 3. DSP Theory

### Delay time

The time knob + CV maps to a delay in samples:

$$
t_{\text{norm}} = \text{clamp}(\text{timeKnob} + \text{timeCV}/10, 0, 1)
$$

$$
\text{delaySec} = 0.01 + t_{\text{norm}} \cdot (t_{\max} - 0.01)
$$

### Modulation

The modulation source is either a sine LFO or a chaos LFO (logistic map):

- **Sine**: $m(t) = \sin(2\pi f_{\text{LFO}} t)$
- **Chaos**: $\text{state} \leftarrow 3.9 \cdot \text{state} \cdot (1 - \text{state})$, $m(t) = 2(\text{state} - 0.5)$

The delay time is modulated by `modSignal * modDepth * 0.05`, giving up to ±5% pitch modulation.

### Read position

For forward playback, the read position is `writePos - delaySamples`. For reverse, it's `writePos + delaySamples` (with wrap).

### Cross-mod

The right channel's read position is offset by the left channel's current output:

$$
\text{readPosR} = \text{readPos} + \text{crossMod} \cdot \text{delayL}[\text{writePos}] \cdot 0.002 \cdot f_s
$$

This creates a "feedback into the stereo image" effect.

### Glitch

With probability `glitchAmount * 0.01` per sample, the read position jumps by a random amount:

$$
\text{jump} = \text{random} \cdot \text{bufferSize} \cdot \text{glitchAmount}
$$

The jump is smoothed over ~2 ms by crossfading between the old and new read outputs.

### Ring mod

A 50 Hz sine carrier multiplies the wet signal:

$$
\text{wet} = \text{wet}(1 - \text{ringAmount} \cdot 0.5) + \text{wet} \cdot \text{carrier} \cdot \text{ringAmount}
$$

### Sample-rate reduction

The wet signal is held for `1 / (1 + srr * 99)` samples:

$$
\text{hold} = \frac{1}{1 + 99 \cdot \text{srr}}
$$

### Bitcrusher

Quantization to 6 bits (12 levels):

$$
\text{wet} = \text{round}(\text{wet} \cdot 6) / 6
$$

### Feedback

$$
fb = \text{feedback} + \text{howl} \cdot 0.2
$$

where `howl` can add up to 0.2 to the feedback. Plus a howl signal from a tanh of the wet:

$$
\text{howlSignal} = \tanh(\text{wet} \cdot \text{howl} \cdot 2), \quad \text{feedbackAmount} = fb + \text{howlSignal} \cdot 0.1
$$

### Freeze

When frozen, the delay buffers are attenuated to 0.999 per sample (slow decay):

$$
\text{buf} \leftarrow \text{buf} \cdot 0.999
$$

### Drive

$$
\text{sat} = \tanh(\text{wet} \cdot (1 + \text{drive}))
$$

### Colour

One-pole LP with coefficient `color` (0.01 .. 0.5):

$$
\text{lp} \leftarrow \text{lp} + \text{color} \cdot (\text{sat} - \text{lp})
$$

### Ping-pong / phase flip

- **Ping-pong**: `outR = -processedL`
- **Phase flip**: `outR = -outR`

### Input gate

A gate on the input signal (threshold 0 = always open):

$$
\text{gateEnv} \leftarrow \max(\text{level}, \text{gateEnv} \cdot (1 - 10 \Delta t))
$$

$$
\text{gateOpen} = \text{gateEnv} \ge \text{gateT}, \quad \text{gateGain} \leftarrow \text{gateGain} + (\text{gateOpen} ? 1 : 0 - \text{gateGain}) \cdot \min(1, 200 \Delta t)
$$

### Dry/wet mix

$$
\text{out} = \text{in} \cdot \text{dryAmount} + \text{wet} \cdot \text{mix}
$$

with `dryAmount = 1 - mix` (or 0.01 when mix = 1 for a tiny dry bleed).

## 4. Parameters and Controls

### Knobs (Delay section, left)

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `TIME_PARAM` | 0 | 1 | 0.5 | Delay time (0.01–2 s) |
| `FEEDBACK_PARAM` | 0 | 1.2 | 0.8 | Feedback |
| `MIX_PARAM` | 0 | 1 | 0.5 | Dry/wet |
| `COLOR_PARAM` | 0.01 | 0.5 | 0.2 | Tone |
| `DRIVE_PARAM` | 0 | 3 | 1.0 | Drive |
| `MOD_DEPTH_PARAM` | 0 | 1 | 0.2 | Mod depth |

### Knobs (FX section, right)

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `HOWL_PARAM` | 0 | 1 | 0 | Howl |
| `RING_MOD_PARAM` | 0 | 1 | 0 | Ring mod |
| `GLITCH_PARAM` | 0 | 1 | 0 | Glitch |
| `GATE_THRESH_PARAM` | 0 | 1 | 0 | Gate threshold |
| `SRR_PARAM` | 0 | 1 | 0 | Sample hold |
| `CROSS_MOD_PARAM` | 0 | 1 | 0 | Cross-mod |

### Switches

| Switch | Description |
|--------|-------------|
| `REVERSE_SWITCH` | Reverse playback |
| `CHAOS_SWITCH` | Chaos LFO |
| `PINGPONG_SWITCH` | Ping-pong |
| `BITCRUSH_SWITCH` | Bitcrusher |
| `FREEZE_SWITCH` | Freeze |
| `PHASE_FLIP_SWITCH` | Phase flip |

### Inputs

| Input | Description |
|-------|-------------|
| `LEFT_INPUT` | Left (mono if R empty) |
| `RIGHT_INPUT` | Right |
| `TIME_CV_INPUT` | Time CV |
| `FREEZE_CV_INPUT` | Freeze gate |
| `HOWL_CV_INPUT` | Howl CV |
| `RING_CV_INPUT` | Ring mod CV |
| `GLITCH_CV_INPUT` | Glitch CV |
| `GATE_CV_INPUT` | Gate threshold CV |

### Outputs

| Output | Description |
|--------|-------------|
| `OUT_L_OUTPUT` | Left |
| `OUT_R_OUTPUT` | Right |

### Enumerations

- `ParamId`: Time, Feedback, Mix, Color, Drive, ModDepth, Reverse, Chaos, Pingpong, Bitcrush, Freeze, PhaseFlip, Howl, RingMod, Glitch, GateThresh, SRR, CrossMod
- `InputId`: Left, Right, TimeCV, FreezeCV, HowlCV, RingCV, GlitchCV, GateCV
- `OutputId`: OutL, OutR

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `LEFT_INPUT` | Audio | ±10 V | Left audio |
| `RIGHT_INPUT` | Audio | ±10 V | Right audio |
| `TIME_CV_INPUT` | CV | 0–10 V | Time modulation |
| `FREEZE_CV_INPUT` | Gate | > 1 V | Freeze |
| `HOWL_CV_INPUT` | CV | 0–10 V | Howl |
| `RING_CV_INPUT` | CV | 0–10 V | Ring mod |
| `GLITCH_CV_INPUT` | CV | 0–10 V | Glitch |
| `GATE_CV_INPUT` | CV | 0–10 V | Gate threshold |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUT_L_OUTPUT` | Audio | ±10 V | Left |
| `OUT_R_OUTPUT` | Audio | ±10 V | Right |

### Bypass

`configBypass(LEFT_INPUT, OUT_L_OUTPUT)` and `configBypass(RIGHT_INPUT, OUT_R_OUTPUT)`.

## 6. Signal Processing Pipeline

1. **Sample-rate change** — Reallocate delay buffers.
2. **Input gate** — Track envelope, gate open state.
3. **Delay time** — Compute from knob + CV.
4. **Modulation** — Advance LFO or chaos state.
5. **Read position** — Normal (write − delay) or reverse (write + delay).
6. **Cross-mod offset** — Add to right read position.
7. **Read + interpolate** — Linear interp for both channels.
8. **Glitch** — Random jump with crossfade.
9. **Ring mod** — 50 Hz carrier.
10. **Sample-rate reduction**.
11. **Bitcrusher**.
12. **Feedback** — With howl signal.
13. **Freeze** — Write or decay.
14. **Drive** — tanh saturation.
15. **Colour LP**.
16. **Ping-pong / phase-flip**.
17. **Mix** — Dry/wet.
18. **Write outputs** — Clamped.
19. **Meters + display** — Peak bins, read age, tap markers.

## 7. Important Functions

### `setSampleRate(float sr)`

Allocates the delay buffers to `2 * sr + 1` samples.

### `onSampleRateChange()`

Reallocates the delay buffers.

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Delay time

$$
t_{\text{norm}} = \text{clamp}(\text{timeKnob} + \text{CV}/10, 0, 1), \quad \text{delaySec} = 0.01 + t_{\text{norm}} \cdot (t_{\max} - 0.01)
$$

### Modulation

$$
\text{modSignal}_{\text{sine}} = \sin(2\pi f_{\text{LFO}} t)
$$

$$
\text{modSignal}_{\text{chaos}} = 2(\text{chaosState} - 0.5)
$$

### Feedback

$$
fb = \text{feedback} + \text{howl} \cdot 0.2 + \tanh(\text{wet} \cdot \text{howl} \cdot 2) \cdot 0.1
$$

### Drive

$$
\text{sat} = \tanh(\text{wet} \cdot (1 + \text{drive}))
$$

### Colour

$$
\text{lp} \leftarrow \text{lp} + \text{color} \cdot (\text{sat} - \text{lp})
$$

### SRR

$$
\text{step} = \frac{1}{1 + 99 \cdot \text{srr}}
$$

### Bitcrusher

$$
\text{wet} \leftarrow \text{round}(\text{wet} \cdot 6) / 6
$$

### Ping-pong

$$
\text{outR} = -\text{processedL}
$$

### Dry/wet mix

$$
\text{out} = \text{in} \cdot (1 - \text{mix}) + \text{wet} \cdot \text{mix}
$$

## 9. Performance and Optimization

### CPU Usage

- Per sample: 6 FX blocks + 4 delay line reads + 1 LFO ≈ 200 flops
- Total: ~200 flops/sample, plus occasional random calls in glitch mode

### Memory Allocation

- `delayL`, `delayR`: 2 s × 48 kHz = 96000 floats each ≈ 768 KB
- No allocation in `process()`

### Buffers

- Delay L/R: 96000 floats each
- Display: 130 floats

### Real-time Processing

- All loops bounded
- No allocation, no blocking
- `std::mt19937` used for glitch (fast)

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads

### Numerical Stability

- `pdSoftLimit` on feedback path
- `tanh` on drive and ring mod
- Delay buffer reset on sample-rate change
- No `!std::isfinite` check, but the output should be bounded by the `pdSoftLimit`

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<vector>`, `<random>`, `<algorithm>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::chip`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`, `agmm::VioletLight`

## 11. Summary

`PsyDelay` is a psychedelic stereo delay that packs a huge amount of character into a single module: reverse, ping-pong, phase-flip, chorus, ring mod, glitch, bitcrush, SRR, freeze, drive, colour LPF. It's the kind of module you patch when you want your delay to *do something* — not just echo, but distort and shimmer and stutter.

The display is excellent: the delay line is shown as a mirrored bar timeline, with the read tap as a vertical line, the modulation as a wobble, the glitch as a flash ring, and the state as chips (FRZ/REV/CHS/PNG/CRSH/PHS/GATE). It's a very complete visualization of what the module is doing.

The main limitations are: many of the FX are extreme (bitcrush is fixed at 6 bits, ring mod is fixed at 50 Hz), and there are no menu options to fine-tune them. For a more controlled multi-FX, look at `Pharmakon`; for a more extreme delay, this is exactly right.
