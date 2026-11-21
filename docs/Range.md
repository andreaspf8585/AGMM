<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Range.cpp
-->

# Range

## 1. Overview

`Range` is a 9 HP utility module that linearly rescales a control-voltage signal. It takes a bipolar input nominally in [−5 V, +5 V] and maps it onto an arbitrary output window [LOW, HIGH], both of which are user-adjustable via front-panel knobs (each spanning −10 V .. +10 V). The transfer is purely linear:

$$
\text{out} = \frac{\text{in} + 5}{10} \cdot (\text{HIGH} - \text{LOW}) + \text{LOW}
$$

so an input of −5 V produces LOW, +5 V produces HIGH, and 0 V produces the midpoint. This makes the module useful for converting a bipolar LFO or envelope into a unipolar/attenuated/offset control signal.

Key features:
- Maps [−5, +5] V input to [LOW, HIGH] output
- LOW and HIGH adjustable from −10 V to +10 V
- Live transfer curve display
- Moving dot for current input/output pair
- LOW/HIGH readout
- No CV inputs (it's a manual utility)

## Module Screenshot

![Range VCV Rack Module](images/Range.png)

## 2. Architecture

### Main Classes

- `Range`: Core module class inheriting from `rack::Module`
- `RGPanelBackground`: Static panel artwork
- `RGDisplayWidget`: Live transfer curve + moving dot + readout

### Inheritance

- `Range` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### State

| Group | Storage |
|-------|---------|
| Display | `float lastIn`, `float lastOut`, `bool active` |

## 3. DSP Theory

### Linear transfer function

The input is normalized to [0, 1] by:

$$
t = \frac{\text{in} + 5}{10}
$$

Then mapped to the output window:

$$
\text{out} = \text{LOW} + t \cdot (\text{HIGH} - \text{LOW})
$$

### Clamping

The input is not clamped — if you feed +20 V into a [−5, +5] input, the output will be `LOW + 1.5 * (HIGH - LOW)`, which could exceed the HIGH endpoint. This is intentional: it lets you extend the range beyond the nominal window if you need to.

### Active state

If the input is not connected, the module outputs 0 V and sets `active = false` (the display hides the moving dot).

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `LOW_PARAM` | −10 | 10 | 0 | Low endpoint of the output window |
| `HIGH_PARAM` | −10 | 10 | 10 | High endpoint of the output window |

### Inputs

| Input | Description |
|-------|-------------|
| `SIGNAL_INPUT` | Signal input |

### Outputs

| Output | Description |
|--------|-------------|
| `SIGNAL_OUTPUT` | Scaled output |

### Enumerations

- `ParamId`: LOW, HIGH
- `InputId`: SignalIn
- `OutputId`: SignalOut

## 5. Inputs and Outputs

### Input

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `SIGNAL_INPUT` | CV/Audio | ±10 V (nominal ±5 V) | Input signal |

### Output

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `SIGNAL_OUTPUT` | CV/Audio | see `[LOW, HIGH]` | Scaled output |

### Bypass

`configBypass(SIGNAL_INPUT, SIGNAL_OUTPUT)` — with no cable, the input passes through.

## 6. Signal Processing Pipeline

1. **Check input connection** — If not connected, output 0 V, set `active = false`, return.
2. **Read LOW and HIGH knobs**.
3. **Read input voltage**.
4. **Compute output** — `out = ((in + 5) / 10) * (HIGH - LOW) + LOW`.
5. **Write output**.
6. **Update display** — Save `lastIn`, `lastOut`, set `active = true`.

## 7. Important Functions

### `process(const ProcessArgs &args)`

The only function. It's a simple linear mapping.

## 8. Mathematical Implementation

### Transfer function

$$
\text{out} = \frac{\text{in} + 5}{10} \cdot (\text{HIGH} - \text{LOW}) + \text{LOW}
$$

### Pixel mapping (display)

Voltage → pixel X:

$$
x(v) = \frac{v + 10}{20} \cdot \text{DISP\_W}
$$

Voltage → pixel Y:

$$
y(v) = \text{DISP\_H} - \frac{v + 10}{20} \cdot \text{DISP\_H}
$$

## 9. Performance and Optimization

### CPU Usage

- 1 addition, 1 division, 1 multiplication, 1 addition per sample ≈ 4 flops
- Display: ~200 flops/frame
- Total: negligible

### Memory Allocation

- No allocation ever
- Fixed-size state

### Buffers

- None

### Real-time Processing

- O(1) per sample
- No branching except the isConnected check
- No allocation, no blocking

### Thread Safety

- `lastIn`, `lastOut`, `active` are written by audio thread, read by UI thread via torn reads

### Numerical Stability

- No feedback, no accumulation
- `std::isfinite` not needed (input is bounded by ±10 V)

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`

## 11. Summary

`Range` is the simplest module in the AGMM collection. It does one thing: linearly remap a voltage range. It's useful for converting a bipolar LFO (say, −5 to +5 V) into a unipolar control signal (0 to +10 V), or for attenuating/offsetting a signal into a specific range.

The display is excellent: it shows the full transfer curve, the LOW and HIGH output markers, and the live input/output pair as a moving dot with crosshairs. You can see exactly what the module is doing at a glance.

The main limitation is that it has no CV inputs — LOW and HIGH are manual only. For CV-controlled range mapping, chain it with a `Swapper` or use a `Bastion` filter as a VCA. But for a manual utility, this is exactly right.
