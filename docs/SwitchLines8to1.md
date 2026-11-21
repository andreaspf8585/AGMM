<!--
AGMM AUTOMATIC DOCUMENTATION
Source: SwitchLines8to1.cpp
-->

# SwitchLines8to1

## 1. Overview

`SwitchLines8to1` is a 24 HP 8-to-1 sequential switch. Eight signal inputs (CH_INPUT 1..8); exactly one is routed to OUT. A clocked step sequencer (LENGTH, START, DIR: forward/reverse/pendulum/random) selects the input. FADE adds a short crossfade between the previous and the new selection (implemented by blending `fromV` → live input value over `fadeT`).

Key features:
- 8 signal inputs → 1 output (one at a time)
- Clocked step sequencer with LENGTH, START, DIR
- 4 direction modes: forward, reverse, pendulum, random
- Crossfade between old and new selection
- One gate per input (high when selected)
- EOC trigger at end of cycle
- Manual STEP button
- Live routing display: input meters, converging curves, output bar, history strip

## Module Screenshot

![SwitchLines8to1 VCV Rack Module](images/SwitchLines8to1.png)

## 2. Architecture

### Main Classes

- `SwitchLines8to1`: Core module class inheriting from `rack::Module`
- `SWPanelBackground`: Static panel artwork
- `SWKnobArcs`: Live value arcs
- `SWDisplayWidget`: Live routing display

### Inheritance

- `SwitchLines8to1` inherits from `rack::Module`
- All UI widgets inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Playhead | `int pos`, `dirSign` |
| Selection | `int curCh`, `prevCh` |
| Crossfade | `float fromV`, `xf` |
| Output | `float outV` |
| Display | `float vin[8]`, `bool linked[8]`, `float pulse`, `clockFlash`, `eocFlash` |
| History | `int hist[16]`, `histHead`, `histCount`, `stepCount` |
| Settings | `int vLen, vStart, vDir`, `float vFadeT` |
| Triggers | `dsp::SchmittTrigger clockTrig, stepTrig, resetTrig`, `dsp::PulseGenerator eocPulse` |

## 3. DSP Theory

### Clocked step sequencer

Same as `SwitchLines1to8`: on each clock, advance position according to direction (forward/reverse/pendulum/random). The actual channel is `curCh = (S + pos) % 8`.

### Crossfade

When the selection changes:

$$
\text{fromV} = \text{outV}, \quad \text{xf} = 0
$$

Each sample, `xf` increases by `Δt / T_fade` (clamped to 1), and the output is:

$$
\text{out} = \text{fromV} \cdot (1 - \text{xf}) + \text{vin}[\text{curCh}] \cdot \text{xf}
$$

So the old value fades out while the new input fades in.

### Signal routing

All 8 inputs are read every sample (because the fade uses `vin[curCh]`, and pre-computing all inputs is cheap). The output is the crossfaded mix.

Gates are 1 for `curCh`, 0 otherwise.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `LENGTH_PARAM` | 1 | 8 | 8 | Sequence length |
| `START_PARAM` | 1 | 8 | 1 | First input |
| `DIR_PARAM` | 0 | 3 | 0 | Direction |
| `FADE_PARAM` | 0 | 1 | 0 | Crossfade (0 = off) |

### Buttons

| Button | Description |
|--------|-------------|
| `STEP_PARAM` | Manual clock step |

### Inputs

| Input | Description |
|-------|-------------|
| `CLOCK_INPUT` | Clock |
| `RESET_INPUT` | Reset |
| `CH_INPUT + 0..7` | 8 signal inputs |

### Outputs

| Output | Description |
|--------|-------------|
| `OUT_OUTPUT` | Selected input |
| `EOC_OUTPUT` | End of cycle |
| `GATE_OUTPUT + 0..7` | 8 gates |

### Lights

- `STEP_LIGHT` — Clock flash

### Enumerations

- `ParamId`: Length, Start, Dir, Fade, Step
- `InputId`: Clock, Reset, CH_INPUT (8)
- `OutputId`: Out, EOC, GATE_OUTPUT (8)
- `LightId`: Step

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CLOCK_INPUT` | Gate | > 1 V | Clock |
| `RESET_INPUT` | Gate | > 1 V | Reset |
| `CH_INPUT + 0..7` | CV/Audio | ±10 V | 8 inputs |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUT_OUTPUT` | CV/Audio | ±10 V | Selected input |
| `EOC_OUTPUT` | Gate | 0/10 V | End of cycle |
| `GATE_OUTPUT + 0..7` | Gate | 0/10 V | 8 gates |

## 6. Signal Processing Pipeline

1. **Read params** — Length, start, dir, fade.
2. **Reset detection** — Reset to start.
3. **Clock edge** — Advance position, capture `fromV`, reset `xf`.
4. **Read all inputs** — Store in `vin[]`.
5. **Crossfade** — Update `xf` and compute `outV`.
6. **Write outputs** — OUT, gates, EOC.
7. **Update display** — Meter levels, flashes.

## 7. Important Functions

### `doStep(int S, int L, int dir, float fadeT)`

Advances the playhead and starts the crossfade.

### `doReset(int S, int L, int dir)`

Resets to the start.

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Position advance

Same as `SwitchLines1to8`.

### Channel mapping

$$
\text{curCh} = (S + \text{pos}) \bmod 8
$$

### Crossfade

$$
\text{outV} = \text{fromV}(1 - \text{xf}) + \text{vin}[\text{curCh}] \cdot \text{xf}
$$

### Fade time

$$
T_{\text{fade}} = \begin{cases} 0 & \text{fade} < 0.01 \\ 0.002 + \text{fade}^2 \cdot 0.4 & \text{fade} \ge 0.01 \end{cases}
$$

## 9. Performance and Optimization

### CPU Usage

- Per sample: 8 input reads + 1 crossfade + 8 gate writes ≈ 30 flops
- Per clock edge: 1 position advance + 1 history update ≈ 20 flops
- Total: ~30 flops/sample

### Memory Allocation

- All state fixed-size
- No allocation in `process()`

### Buffers

- `vin[8]`, `linked[8]`, `hist[16]`: 32 floats

### Real-time Processing

- All loops bounded by 8
- No allocation, no blocking

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads

### Numerical Stability

- `xf` clamped to `[0, 1]`
- `fadeT = 0` gives instant snap
- No feedback

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstdint>`, `<cstring>`, `<algorithm>`, `<memory>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `rack::dsp::SchmittTrigger`, `rack::dsp::PulseGenerator`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::plate`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::meter`, `agmm::chip`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`

## 11. Summary

`SwitchLines8to1` is the inverse of `SwitchLines1to8`: it takes 8 inputs and routes one to the output, selecting via a clocked step sequencer. The crossfade works in the opposite direction: instead of fading per-output gains, it fades from the previous output value to the new input value.

The **single-value crossfade** is simpler than the per-output gain approach: it only stores one `fromV` and one `xf`, and the output is a single blend. This is more efficient (30 flops vs 50 flops), but it means there's no way to fade between two simultaneous inputs (only one at a time).

The display is well-designed: the input meters show all 8 input levels, the converging curves show the signal flowing from the selected input to the hub, and the output bar shows the routed level. The history strip shows the last 16 selections.

The main limitations are: no CV inputs, and the crossfade is linear (not equal-power), so at xf = 0.5 the output is `(fromV + vin[curCh])/2`, which is quieter than either endpoint. But for a sequential switch, this is excellent.