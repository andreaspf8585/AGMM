<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Range.cpp
-->

# Range

## 1. Overview

`Range` is a 9 HP polyphonic utility module that linearly rescales a control-voltage signal. It maps an arbitrary input window `[LOW IN, HIGH IN]` onto an arbitrary output window `[LOW, HIGH]`. All four endpoints are user-adjustable via front-panel knobs (each spanning −10 V .. +10 V). The transfer is purely linear:

$$
\text{out} = \frac{\text{in} - \text{LOW IN}}{\text{HIGH IN} - \text{LOW IN}} \cdot (\text{HIGH} - \text{LOW}) + \text{LOW}
$$

so an input of `LOW IN` produces `LOW`, `HIGH IN` produces `HIGH`, and the middle of the input window produces the midpoint. This makes the module useful for converting a bipolar LFO or envelope into a unipolar / attenuated / offset control signal, or for renormalising an unusual CV window into a standard one.

Key features:
- Maps an adjustable input window `[LOW IN, HIGH IN]` onto an adjustable output window `[LOW, HIGH]`
- All four endpoints adjustable from −10 V to +10 V
- Inverting the input window (LOW IN > HIGH IN) inverts the transfer — useful for flipped envelopes/LFOs
- Optional output clamp to `[LOW, HIGH]` (default on)
- Right-click "Input range presets" submenu snaps the input knobs to common windows
- Polyphonic: processes up to 16 channels (SIMD, 4 at a time)
- Live transfer curve display with a moving dot for the current input/output pair
- LOW/HIGH output readout and current input window tag

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
| Display | `float lastIn[16]`, `float lastOut[16]`, `int channels`, `bool active` |
| Settings | `bool clampOut` (persisted); knob values are persisted automatically by Rack |

## 3. DSP Theory

### Linear transfer function

Let the input window be `[inLo, inHi]` and the output window be `[low, high]`. The input is normalized to `[0, 1]` by:

$$
t = \frac{\text{in} - \text{inLo}}{\text{inHi} - \text{inLo}}
$$

Then mapped to the output window:

$$
\text{out} = \text{low} + t \cdot (\text{high} - \text{low})
$$

### Degenerate window guard

If `inHi == inLo` the denominator would be zero. The implementation substitutes a tiny epsilon of the same sign (±1e-4) so the mapping remains finite. The window is otherwise free — including `inLo > inHi`, which yields an inverted (negated-slope) transfer.

### Clamping

When `clampOut` is enabled (default), the output is clamped into `[min(low, high), max(low, high)]`. This keeps the result inside the output window even if the input overshoots `[inLo, inHi]`. When disabled, the raw linear extrapolation is passed through, letting you intentionally drive the output beyond `LOW` / `HIGH`.

### Active state

If the input is not connected, the module outputs 0 V (single channel) and sets `active = false` (the display hides the moving dot). Otherwise the module produces the same number of output channels as the input cable carries.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `LOW_IN_PARAM`  | −10 | 10 | −10 | Low endpoint of the input window (left column, top) |
| `HIGH_IN_PARAM` | −10 | 10 |  10 | High endpoint of the input window (left column, bottom) |
| `LOW_PARAM`     | −10 | 10 |   0 | Low endpoint of the output window (right column, top) |
| `HIGH_PARAM`    | −10 | 10 |  10 | High endpoint of the output window (right column, bottom) |

The two left-hand knobs (`LOW IN` / `HIGH IN`) are the authoritative input range and always take effect. The right-hand knobs (`LOW` / `HIGH`) set the output window. Both pairs are shared by all polyphonic channels.

### Context menu

- **Input range presets** — snaps `LOW IN` / `HIGH IN` to a common window. Presets are:
  - `-10 .. +10 V` (defaults)
  - `-5 .. +5 V`
  - `0 .. 10 V`
  - `0 .. 5 V`
  - `-1 .. +1 V`
  - `0 .. 1 V`
  
  This is a convenience shortcut only; the knobs remain authoritative and can be tweaked freely after choosing a preset.
- **Clamp output to LOW..HIGH** — toggle (default on).

### Inputs

| Input | Description |
|-------|-------------|
| `SIGNAL_INPUT` | Signal input (polyphonic, up to 16 ch) |

### Outputs

| Output | Description |
|--------|-------------|
| `SIGNAL_OUTPUT` | Scaled output (same channel count as input) |

### Enumerations

- `ParamId`: `LOW_IN_PARAM`, `HIGH_IN_PARAM`, `LOW_PARAM`, `HIGH_PARAM`
- `InputId`: `SIGNAL_INPUT`
- `OutputId`: `SIGNAL_OUTPUT`

## 5. Inputs and Outputs

### Input

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `SIGNAL_INPUT` | CV/Audio, polyphonic | ±10 V | Input signal; channel count determines output channel count |

### Output

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `SIGNAL_OUTPUT` | CV/Audio, polyphonic | see `[LOW, HIGH]` (clamped when enabled) | Scaled output |

### Bypass

`configBypass(SIGNAL_INPUT, SIGNAL_OUTPUT)` — with no cable plugged into the input, Rack can pass a signal on the input through to the output as bypass.

## 6. Signal Processing Pipeline

1. **Check input connection** — if not connected, output 0 V (1 channel), set `active = false`, return.
2. **Read input channel count** `n` (clamped to 1..16).
3. **Read all four knobs** (`inLo`, `inHi`, `low`, `high`) and the `clampOut` flag.
4. **Compute scale** — `scale = (high - low) / (inHi - inLo)`, guarded against a zero denominator; also compute the sorted clamp window `[wLo, wHi]`.
5. **Set output channel count** to `n`.
6. **Per 4-channel SIMD block**:
   - Load `in = getVoltageSimd<float_4>(c)`
   - Compute `out = (in - inLo) * scale + low`
   - If `clampOut`, `out = simd::clamp(out, wLo, wHi)`
   - Store to `SIGNAL_OUTPUT`
   - Save `lastIn[c..c+3]`, `lastOut[c..c+3]` for the display
7. **Set `channels = n`, `active = true`.**

## 7. Important Functions

### `process(const ProcessArgs &args)`

The audio callback. Implements the linear mapping above, in SIMD blocks of four channels, with the zero-denominator guard and optional output clamp.

### `dataToJson()` / `dataFromJson()`

Persists `clampOut`. `dataFromJson()` also performs a **legacy migration**: older patches stored an integer preset index under `"inRange"`. If present, the input knobs are snapped to the matching preset (Rack loads parameter values before calling `dataFromJson`, so this override is safe and reproduces the original transfer for old patches).

## 8. Mathematical Implementation

### Transfer function

$$
\text{out} = \frac{\text{in} - \text{inLo}}{\text{inHi} - \text{inLo}} \cdot (\text{high} - \text{low}) + \text{low}
$$

with the denominator guarded by ±1e-4 when `inHi == inLo`.

### Clamp

$$
\text{out} \mathrel{:=} \operatorname{clamp}(\text{out},\ \min(\text{low},\text{high}),\ \max(\text{low},\text{high})) \quad \text{(if clampOut)}
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

The transfer curve is drawn at the x positions `{-10, min(inLo,inHi), max(inLo,inHi), 10}`, so an inverted input window still produces a clean single-segment path.

## 9. Performance and Optimization

### CPU Usage

- Per 4-channel SIMD block: 1 subtraction, 1 multiply, 1 add, plus an optional clamp ≈ a few flops per sample per channel
- Display: a couple hundred flops/frame
- Total: negligible

### Memory Allocation

- No allocation ever
- Fixed-size state (`lastIn[16]`, `lastOut[16]`)

### Buffers

- None

### Real-time Processing

- O(1) per sample (SIMD, 4 channels at a time)
- No allocation, no blocking
- Branching limited to `isConnected`, `clampOut`, and the loop bound

### Thread Safety

- `lastIn`, `lastOut`, `channels`, `active` are written by the audio thread and read by the UI thread via torn reads. Acceptable for display-only data.
- `clampOut` is a plain bool toggled from the UI; a torn read cannot happen for a `bool` and the worst case is one block using the previous value.

### Numerical Stability

- No feedback, no accumulation
- Zero-denominator guarded with a signed epsilon
- Output values are finite for any finite input; clamped to `[LOW, HIGH]` when `clampOut` is enabled

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `simd::float_4`, `simd::clamp`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`

## 11. Summary

`Range` is the simplest module in the AGMM collection. It does one thing: linearly remap a voltage window onto another voltage window. It's useful for converting a bipolar LFO (say, −5 to +5 V) into a unipolar control signal (0 to +10 V), for renormalising an unusual CV range into a standard one, or for inverting a signal by swapping `LOW IN` and `HIGH IN`.

Four knobs (left column = input window, right column = output window) give full manual control, and the "Input range presets" submenu is a quick way to snap the input window to common conventions. The output can optionally be clamped so it never leaves `[LOW, HIGH]`, or left unclamped so it can intentionally overshoot.

The display is excellent: it shows the full transfer curve (including flat shoulders when clamping is enabled), the `LOW` and `HIGH` output markers, the highlighted input window, the current input window tag, and the live input/output pair as a moving dot with crosshairs. Polyphonic input is displayed as a bright dot for channel 1 plus small dots for channels 2..N, with a `N CH` tag.

The main limitation is that it has no CV inputs — all four endpoints are manual only. For CV-controlled range mapping, chain it with a `Swapper` or use a `Bastion` filter as a VCA. But for a manual utility, this is exactly right.