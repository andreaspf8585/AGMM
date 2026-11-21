<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Mixer3.cpp
-->

# Mixer3

## 1. Overview

`Mixer3` is an 18 HP stereo 3-channel mixer. Each channel has L/R inputs, a level knob, and a CV input with a bipolar attenuverter. The three channels are summed and scaled by a master gain. The module is meant to be a simple, straightforward stereo mixer for patching multiple sources into one stereo bus.

Key features:
- 3 stereo channels (L/R inputs per channel)
- Per-channel level knob + CV with bipolar attenuverter
- Master gain
- Scope of the mono mix + L/R meters
- Selectable CV range (bipolar/unipolar) via menu, persisted

## Module Screenshot

![Mixer3 VCV Rack Module](images/Mixer3.png)

## 2. Architecture

### Main Classes

- `Mixer3`: Core module class inheriting from `rack::Module`
- `M3PanelBackground`: Static panel artwork
- `M3DisplayWidget`: Live scope + meters + readouts

### Inheritance

- `Mixer3` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| CV mode | `CVRange cvRangeMode` |
| Display | `float scopeMin[96]`, `scopeMax[96]`, `scopeHead`, `scopeCount`, `scopeBMin`, `scopeBMax`, `peakL`, `peakR` |

## 3. DSP Theory

### Per-channel gain

Each channel's gain is:

$$
g_c = \text{clamp}(\text{LEVEL}_c + \text{atten}_c \cdot \text{scaleCV}(\text{CV}_c), 0, 1)
$$

where:

$$
\text{scaleCV}(v) = \begin{cases} v / 10 & \text{unipolar} \\ \text{clamp}(v/5, -1, 1) & \text{bipolar} \end{cases}
$$

The gain is clamped to `[0, 1]` so a channel can never invert phase or amplify above unity.

### Summing

$$
\text{mixL} = \sum_c \text{inL}_c \cdot g_c, \quad \text{mixR} = \sum_c \text{inR}_c \cdot g_c
$$

### Master gain

$$
\text{outL} = \text{mixL} \cdot g_{\text{master}}, \quad \text{outR} = \text{mixR} \cdot g_{\text{master}}
$$

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `PARAM_CH1_LEVEL` | 0 | 1 | 0 | Channel 1 level |
| `PARAM_CH2_LEVEL` | 0 | 1 | 0 | Channel 2 level |
| `PARAM_CH3_LEVEL` | 0 | 1 | 0 | Channel 3 level |
| `PARAM_CH1_ATTV` | −1 | 1 | 0 | Ch 1 CV attenuverter |
| `PARAM_CH2_ATTV` | −1 | 1 | 0 | Ch 2 CV attenuverter |
| `PARAM_CH3_ATTV` | −1 | 1 | 0 | Ch 3 CV attenuverter |
| `PARAM_MASTER_GAIN` | 0 | 2 | 1 | Master gain |

### Inputs

| Input | Description |
|-------|-------------|
| `INPUT_CH1_L` | Ch 1 L audio |
| `INPUT_CH1_R` | Ch 1 R audio |
| `INPUT_CH2_L` | Ch 2 L audio |
| `INPUT_CH2_R` | Ch 2 R audio |
| `INPUT_CH3_L` | Ch 3 L audio |
| `INPUT_CH3_R` | Ch 3 R audio |
| `INPUT_CH1_CV` | Ch 1 level CV |
| `INPUT_CH2_CV` | Ch 2 level CV |
| `INPUT_CH3_CV` | Ch 3 level CV |

### Outputs

| Output | Description |
|--------|-------------|
| `OUTPUT_L` | Mix L output |
| `OUTPUT_R` | Mix R output |

### Enumerations

- `CVRange`: BIPOLAR_M5_5, UNIPOLAR_0_10
- `ParamId`: CH1Level..CH3Level, CH1Attv..CH3Attv, MasterGain
- `InputId`: CH1L..CH3R, CH1CV..CH3CV
- `OutputId`: OutputL, OutputR

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `INPUT_CH1_L` | Audio | ±10 V | Channel 1 L |
| `INPUT_CH1_R` | Audio | ±10 V | Channel 1 R |
| `INPUT_CH2_L` | Audio | ±10 V | Channel 2 L |
| `INPUT_CH2_R` | Audio | ±10 V | Channel 2 R |
| `INPUT_CH3_L` | Audio | ±10 V | Channel 3 L |
| `INPUT_CH3_R` | Audio | ±10 V | Channel 3 R |
| `INPUT_CH1_CV` | CV | ±10 V | Ch 1 level CV |
| `INPUT_CH2_CV` | CV | ±10 V | Ch 2 level CV |
| `INPUT_CH3_CV` | CV | ±10 V | Ch 3 level CV |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUTPUT_L` | Audio | ±10 V | Mix L |
| `OUTPUT_R` | Audio | ±10 V | Mix R |

## 6. Signal Processing Pipeline

1. **Read CV mode** from `cvRangeMode`.
2. **For each channel**:
   - Read level knob, attenuverter, CV.
   - Compute `gain = clamp(knob + atten * scaleCV(cv), 0, 1)`.
   - Read L and R inputs.
   - Add `inL * gain` and `inR * gain` to mix.
3. **Read master gain**.
4. **Multiply mix** by master gain.
5. **Write outputs**.
6. **Update display** — peak followers + scope ring buffer.

## 7. Important Functions

### `scaleCV(float v)`

Scales a CV according to the current mode.

```cpp
if (cvRangeMode == BIPOLAR_M5_5) return std::max(-1.f, std::min(v / 5.f, 1.f));
return v / 10.f;
```

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Per-channel gain

$$
g_c = \text{clamp}(\text{LEVEL}_c + \text{atten}_c \cdot \text{scaleCV}(\text{CV}_c), 0, 1)
$$

### Summing

$$
\text{mixL} = \sum_{c=1}^{3} \text{inL}_c \cdot g_c, \quad \text{mixR} = \sum_{c=1}^{3} \text{inR}_c \cdot g_c
$$

### Master output

$$
\text{outL} = \text{mixL} \cdot g_{\text{master}}, \quad \text{outR} = \text{mixR} \cdot g_{\text{master}}
$$

### Peak followers

$$
\text{peakL} \leftarrow \max(|\text{outL}|, \text{peakL} \cdot (1 - 4\Delta t))
$$

Same for R.

## 9. Performance and Optimization

### CPU Usage

- 3 channels × (2 reads + 2 multiplies + 1 add) ≈ 30 flops
- 1 master multiply
- Peak + scope ≈ 10 flops
- Total: ~50 flops/sample

### Memory Allocation

- No allocation in `process()`
- `scopeMin[96]`, `scopeMax[96]` fixed-size
- No `std::vector` used

### Buffers

- `scopeMin[96]`, `scopeMax[96]`: 192 floats

### Real-time Processing

- Loop bounded by 3 (channels)
- No allocation, no blocking
- `scopeCount` increments every sample and resets at the bucket size

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads
- `cvRangeMode` is read by UI thread but only written from the context menu (rare)

### Numerical Stability

- Gain is clamped to `[0, 1]`
- Master gain is `[0, 2]`
- No feedback, no accumulation issues
- All outputs are bounded by input voltage × max gain

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`, `<string>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::plate`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::meter`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`

## 11. Summary

`Mixer3` is a straightforward stereo 3-channel mixer. It has everything you need for a small mix (per-channel level, CV with attenuverter, master gain) and nothing you don't (no routing, no sends, no EQ).

The `scaleCV` feature is worth noting: it lets the same module work with both bipolar LFOs (±5 V) and unipolar envelopes (0–10 V) without changing the patch. The attenuverter then scales that CV into the knob offset, and the result is clamped to `[0, 1]` so a channel can never invert phase or amplify above unity (which would cause instability in a feedback patch).

The main limitations are: only 3 channels (for more, use `Mixer6`), no mute/solo (you have to turn the level down), and no pan (the stereo width comes from the input signals, not from panning). For a small modular mixer, these are acceptable trade-offs.
