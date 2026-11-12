<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Gates.cpp
-->

# Gates

## 1. Overview

`Gates` is a 24 HP polyphonic logic processor. It takes two gate inputs (A and B) and produces seven simultaneous logic outputs: AND, OR, NOT (of A), NAND, NOR, XOR and XNOR. Each input is treated as "high" when its voltage exceeds 1 V, and every output is 10 V when its logic function is true and 0 V when false. All outputs are polyphonic: the channel count is the larger of the two inputs' channel counts (at least one).

Key features:
- 7 logic outputs: AND, OR, NOT, NAND, NOR, XOR, XNOR
- Fully polyphonic (channel count = max(A, B))
- Live 9-row scrolling logic scope
- Adjustable time axis (5 ms .. 50 s)
- Per-output activity LEDs
- Per-input activity LEDs
- Log-mapped TIME knob with friendly display

## Module Screenshot

![Gates VCV Rack Module](images/gates.png)

## 2. Architecture

### Main Classes

- `Gates`: Core module class inheriting from `rack::Module`
- `TimeQuantity`: Custom ParamQuantity for the log TIME knob
- `GTPanel`: Static panel artwork
- `GTDisplay`: Live logic scope

### Inheritance

- `Gates` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Current state | `bool aNow`, `bool bNow`, `bool outNow[7]` |
| History | `bool histA[512]`, `histB[512]`, `histOut[7][512]`, `int histPos` |
| Timing | `float screenMs`, `double histAccum` |

## 3. DSP Theory

### Logic functions

For inputs A and B:

$$
\begin{aligned}
\text{AND} &= A \land B \\
\text{OR} &= A \lor B \\
\text{NOT} &= \lnot A \\
\text{NAND} &= \lnot(A \land B) \\
\text{NOR} &= \lnot(A \lor B) \\
\text{XOR} &= A \oplus B \\
\text{XNOR} &= A \odot B
\end{aligned}
$$

### Input threshold

Each input is high if its voltage exceeds 1 V:

$$
\text{high}(v) = v > 1.0
$$

### Polyphony

The output channel count is `max(A_ch, B_ch)` (at least 1). For channels where an input has no channel, it reads 0 (so `high = false`).

### History sampling

The 512-slot history buffer spans exactly `screenMs` milliseconds:

$$
\Delta t_{\text{slot}} = \frac{\text{screenMs}}{1000 \cdot 512} \text{ s}
$$

Every `Δt_slot` seconds, all 9 signals (A, B, and 7 outputs) are written to the history.

### Time knob mapping

The TIME knob uses an inverted logarithmic mapping:

$$
\text{ms} = 50000 \cdot \left(\frac{5}{50000}\right)^t
$$

where $t \in [0, 1]$ is the knob position. So:

- $t = 0$ → 50 s/screen (slowest)
- $t = 0.5$ → 500 ms/screen (default)
- $t = 1$ → 5 ms/screen (fastest)

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `PARAM_TIME` | 0 | 1 | 0.5 | Time axis (log, 5 ms .. 50 s) |

### Inputs

| Input | Description |
|-------|-------------|
| `INPUT_A` | Input A |
| `INPUT_B` | Input B |

### Outputs

| Output | Description |
|--------|-------------|
| `OUTPUT_AND` | A AND B |
| `OUTPUT_OR` | A OR B |
| `OUTPUT_NOT` | NOT A |
| `OUTPUT_NAND` | NOT (A AND B) |
| `OUTPUT_NOR` | NOT (A OR B) |
| `OUTPUT_XOR` | A XOR B |
| `OUTPUT_XNOR` | A XNOR B |

### Lights

- `LIGHT_A`, `LIGHT_B` — input activity
- `LIGHT_AND` .. `LIGHT_XNOR` — output activity

### Enumerations

- `ParamId`: Time
- `InputId`: A, B
- `OutputId`: And, Or, Not, Nand, Nor, Xor, Xnor
- `LightId`: A, B, And, Or, Not, Nand, Nor, Xor, Xnor

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `INPUT_A` | Poly Gate | > 1 V = high | Input A |
| `INPUT_B` | Poly Gate | > 1 V = high | Input B |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUTPUT_AND` | Poly Gate | 0/10 V | A AND B |
| `OUTPUT_OR` | Poly Gate | 0/10 V | A OR B |
| `OUTPUT_NOT` | Poly Gate | 0/10 V | NOT A |
| `OUTPUT_NAND` | Poly Gate | 0/10 V | NAND |
| `OUTPUT_NOR` | Poly Gate | 0/10 V | NOR |
| `OUTPUT_XOR` | Poly Gate | 0/10 V | XOR |
| `OUTPUT_XNOR` | Poly Gate | 0/10 V | XNOR |

All outputs share the same channel count = `max(A_ch, B_ch)`.

## 6. Signal Processing Pipeline

1. **Read TIME knob** — Map to `screenMs` via `timeKnobToMs()`.
2. **Determine channel count** — `max(A_ch, B_ch, 1)`.
3. **Per-channel logic** — For each channel, evaluate A and B (threshold at 1 V), compute all 7 functions, write to outputs.
4. **Snapshot** — Save channel 0's A, B and output states for the display and LEDs.
5. **Update LEDs** — Set each LED from channel 0's corresponding state.
6. **History sampling** — Advance `histAccum` by `sampleTime`; while it exceeds `slotSec`, write the current A, B and 7 outputs to `history[histPos]` and advance.
7. **Set output channels** — Ensure all 7 outputs have the computed channel count.

## 7. Important Functions

### `timeKnobToMs(float t)`

Converts knob position (0..1) to milliseconds per screen.

```cpp
t = clamp(t, 0.f, 1.f);
return 50000.f * std::pow(5.f / 50000.f, t);
```

### `msToTimeKnob(float ms)`

Inverse of the above, used by the ParamQuantity.

### `TimeQuantity::getDisplayValue()`

Returns the current value in milliseconds.

### `TimeQuantity::setDisplayValue(float v)`

Accepts a value in milliseconds and converts it back to a knob position.

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Time knob mapping

$$
\text{ms} = 50000 \cdot \left(\frac{5}{50000}\right)^t
$$

### History slot duration

$$
\Delta t_{\text{slot}} = \frac{\text{screenMs}}{1000 \cdot 512}
$$

### Logic functions

All are standard boolean operations evaluated per channel.

### Channel count

$$
N_{\text{out}} = \max(A_{\text{ch}}, B_{\text{ch}}, 1)
$$

## 9. Performance and Optimization

### CPU Usage

- Per sample: 7 logic ops per channel × up to 16 channels ≈ 112 ops
- LED updates: 9 per sample
- History: 1 write per `Δt_slot` (rare)
- Total: ~200 flops/sample

### Memory Allocation

- All state is fixed-size
- History buffers: 512 × 9 = 4608 bools ≈ 4.6 KB
- No allocation in `process()`

### Buffers

- `histA[512]`, `histB[512]`, `histOut[7][512]`: 4608 bools

### Real-time Processing

- All operations are O(1) per channel
- No allocation, no blocking
- History sampling rate is bounded by `screenMs` (fastest is 5 ms / 512 = 9.77 µs, ~100 K writes/s)

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads
- History is written by audio thread, read as a snapshot by UI

### Numerical Stability

- Pure boolean logic — no numerical issues
- Threshold at 1 V is deterministic

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::dim`, `agmm::bright`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`

## 11. Summary

`Gates` is a polyphonic logic processor that produces all seven standard logic functions from two gate inputs. It's a simple but very useful utility for combining gates — for example, gating a trigger with a clock (AND), or gating it with the inverse of a reset (NAND), or producing a trigger on rising/falling edges (XOR).

The most distinctive feature is the live 9-row logic scope, which shows the history of A, B and all seven outputs as scrolling square waves. The TIME knob makes it easy to see both very short (< 5 ms) and very long (> 10 s) gate patterns.

The main limitation is that it only has two inputs — no NOT for B, no multi-input AND/OR. For more complex logic, chain multiple `Gates` modules or use a `Bastion` filter as a threshold detector.
