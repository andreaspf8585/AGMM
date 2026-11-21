<!--
AGMM AUTOMATIC DOCUMENTATION
Source: SwitchLines1to8.cpp
-->

# SwitchLines1to8

## 1. Overview

`SwitchLines1to8` is a 24 HP 1-to-8 sequential switch that routes a single signal input to one of eight outputs. A clocked step sequencer walks a window of active outputs defined by LENGTH (1..8) and START (1..8); the direction of traversal is set by DIR (Forward / Reverse / Pendulum / Random). The FADE control crossfades the newly-selected output in while the previous one fades out. Each channel also has a gate output that goes high while it is selected, and an end-of-cycle trigger pulse is provided on the EOC jack.

Key features:
- One input, eight switched outputs
- Window sequencer with START + LENGTH
- Forward / Reverse / Pendulum / Random traversal
- Per-output linear crossfade (0 ms to ~400 ms)
- Eight gate outputs, one EOC trigger
- Clock, reset, and manual STEP with a clock LED
- Custom 224 × 152 display: input bar, routing fan, per-output meters, 16-cell history

## Module Screenshot

![SwitchLines1to8 VCV Rack Module](images/SwitchLines1to8.png)

## 2. Architecture

### Main Classes

- `SwitchLines1to8` — core module class inheriting from `rack::Module`
- `SWPanelBackground` — static panel artwork
- `SWKnobArcs` — live value arcs around the four side knobs
- `SWDisplayWidget` — the 224 × 152 routing display

### Inheritance

- `SwitchLines1to8` inherits from `rack::Module`
- All three UI widgets inherit from `rack::TransparentWidget`

### Structures

No dedicated structs; all state lives directly on the module class.

### State (grouped)

| Group | Storage |
|-------|---------|
| Sequencer | `int pos`, `int dirSign`, `int curCh`, `int prevCh`, `float gain[8]` |
| Triggers | `dsp::SchmittTrigger clockTrig, stepTrig, resetTrig` |
| EOC | `dsp::PulseGenerator eocPulse` |
| Display inputs | `float inV`, `bool inLinked`, `float outV[8]`, `bool linked[8]` |
| Display — history | `int hist[16]`, `int histHead`, `int histCount` |
| Display — animation | `float pulse`, `float clockFlash`, `float eocFlash` |
| Display — readouts | `uint32_t stepCount`, `int vLen, vStart, vDir`, `float vFadeT` |

## 3. DSP Theory

### Sequencing

The sequencer keeps a position `pos ∈ [0, L)` inside the active window of `L = LENGTH` outputs, and the current channel is:

$$
\text{curCh} = (S + \text{pos}) \bmod 8
$$

with `S = START − 1`.

The four direction modes behave as follows:

- **Forward**: `pos ← pos + 1`, wrapping to `0` at `pos = L` → EOC fires.
- **Reverse**: `pos ← pos − 1`, wrapping to `L − 1` at `pos < 0` → EOC fires.
- **Pendulum**: `pos += dirSign`; on reaching `L` the direction flips and `pos = L − 2`; on reaching `−1` the direction flips back, `pos = 1`, and EOC fires.
- **Random**: a uniform integer in `[0, L)` is chosen with rejection of the current position; EOC fires whenever the new position is `0`.

### Fade

The crossfade is realised as eight independent 0..1 gains, each ramping linearly at `1/fadeT` per second toward its target (1 for `curCh`, 0 otherwise). The selected output therefore fades in over `fadeT` seconds while all others fade out at the same rate, producing complementary gains (though not equal-power).

The FADE knob maps quadratically to time:

$$
t_{\text{fade}} = \begin{cases}
0 & \text{if } f < 0.01 \\
0.002 + 0.4\,f^{2} & \text{otherwise}
\end{cases}
$$

so the usable range is 2 ms to ≈ 402 ms.

### Outputs

Each output voltage is:

$$
V_i = V_{\text{in}} \cdot g_i
$$

The gates are hard switches (`+10 V` for `curCh`, `0 V` otherwise), and EOC triggers a 1 ms pulse when a cycle ends.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `LENGTH_PARAM` | 1 | 8 | 8 | Sequence length (snapped) |
| `START_PARAM` | 1 | 8 | 1 | First output of the window (snapped) |
| `DIR_PARAM` | 0 | 3 | 0 | Direction (Forward / Reverse / Pendulum / Random) |
| `FADE_PARAM` | 0 | 1 | 0 | Crossfade time |

### Button

| Parameter | Description |
|-----------|-------------|
| `STEP_PARAM` | Manual clock step (momentary) |

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CLOCK_INPUT` | Clock | 0..10 V | Rising edge advances one step |
| `RESET_INPUT` | Trigger | 0..10 V | Rising edge resets to the start of the window |
| `SIGNAL_INPUT` | Audio/CV | ±10 V | Signal routed to the selected output |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CH_OUTPUT 1..8` | Audio/CV | ±10 V | Gated copies of `SIGNAL_INPUT` |
| `GATE_OUTPUT 1..8` | Gate | 0/10 V | +10 V while that output is selected |
| `EOC_OUTPUT` | Trigger | 0/10 V | 1 ms pulse at the end of each cycle |

### Menu

No menu options.

## 6. Signal Processing Pipeline

1. **Read params** — LENGTH, START, DIR, FADE (clamped).
2. **Handle RESET** — on rising edge, call `doReset` (no crossfade, jump to window start).
3. **Handle clock / STEP** — on rising edge of either, call `doStep`.
4. **Read SIGNAL_INPUT** — 0 V if unpatched.
5. **Ramp gains** — each `g_i` moves linearly toward its target at `1/fadeT` per second.
6. **Write outputs** — `OUT_i = inV · g_i`; gates `+10 V` for `curCh`, `0 V` otherwise.
7. **EOC** — set `EOC_OUT` from `eocPulse.process(dt)`.
8. **Timers** — decay `clockFlash` and `eocFlash`; advance `pulse`; snapshot readouts into `vLen / vStart / vDir / vFadeT`.

## 7. Important Functions

### `void doReset(int S, int L, int dir)`

Resets `pos` to the start of the window (`L − 1` in reverse, otherwise 0), sets `dirSign = +1`, snaps all gains to the new selection without a crossfade, and clears `stepCount`.

### `void doStep(int S, int L, int dir)`

Advances the sequencer by one step according to `dir`, updates `curCh`, `prevCh`, `pulse`, `clockFlash`, `stepCount`, pushes the new channel onto the history ring, and triggers EOC if the cycle completed.

### `void process(const ProcessArgs &args)`

The main audio-thread callback described in §6.

### `void bezier(...)`

UI-only helper: evaluates a cubic Bezier for the routing-fan curves.

## 8. Mathematical Implementation

### Step transitions

Forward:
$$
\text{pos} \leftarrow (\text{pos} + 1) \bmod L, \quad \text{EOC if wrapped}
$$

Reverse:
$$
\text{pos} \leftarrow (\text{pos} - 1 + L) \bmod L, \quad \text{EOC if wrapped}
$$

Pendulum (two states, `dirSign ∈ {−1, +1}`):
$$
\begin{aligned}
&\text{pos} \leftarrow \text{pos} + \text{dirSign} \\
&\text{if pos} \ge L:\ \text{pos} = L - 2,\ \text{dirSign} = -1 \\
&\text{if pos} < 0:\ \text{pos} = 1,\ \text{dirSign} = +1,\ \text{EOC}
\end{aligned}
$$

### Fade gain

Each sample, for every `i`:
$$
g_i \leftarrow \begin{cases}
\min(1, g_i + \Delta) & \text{if target} = 1 \\
\max(0, g_i - \Delta) & \text{if target} = 0
\end{cases}, \quad \Delta = \frac{dt}{t_{\text{fade}}}
$$

If `fadeT = 0`, the target is applied directly (no ramp).

### Current channel

$$
\text{curCh} = (S + \text{pos}) \bmod 8, \quad S = \text{START} - 1
$$

## 9. Performance and Optimization

### CPU Usage

- 8 gain updates, 8 signal-output writes, 8 gate-output writes
- 3 Schmitt triggers + 1 PulseGenerator
- ≈ 30–40 arithmetic ops per sample in the DSP path

### Memory Allocation

- All state is fixed-size; no allocation in `process()`
- Display arrays: `hist[16]`, `outV[8]`, `linked[8]`, `gain[8]` ≈ 50 words

### Real-time Processing

- Trivial cost; suitable for many instances

### Thread Safety

- Display fields written by audio thread, read by UI thread via torn reads (cosmetic only)

### Numerical Stability

- `pos` is clamped to `[0, L−1]` after parameter changes
- `curCh` is clamped to `[0, 7]` after every step
- Gains are constrained to `[0, 1]` by construction

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstdint>`, `<cstring>`, `<algorithm>`, `<memory>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::ModuleWidget`
- `dsp::SchmittTrigger`, `dsp::PulseGenerator`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::sectionHeader`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::withA`, `agmm::ALIGN_L/C/R`

## 11. Summary

`SwitchLines1to8` is a compact 1-to-8 router that walks a sub-range of its outputs under clock control. The START / LENGTH / DIR combination gives it the feel of a miniature step sequencer, and the FADE crossfade plus per-output gates make it drive downstream sequencers, envelopes, or gate-hungry modules cleanly.

The 16-cell history strip and the routing fan (with a travelling pulse dot) make the pattern visible at a glance, and the EOC output provides a natural boundary for looped patterns. Pair it with a matrix mixer or an envelope bank for pseudo-polyphonic behaviour out of one input.