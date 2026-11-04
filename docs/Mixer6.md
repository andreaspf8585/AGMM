<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Mixer6.cpp
Generated: 2026-11-04 06:23
-->

# Mixer6

## 1. Overview

The `Mixer6` module is a stereo 6-channel audio mixer for VCV Rack. It accepts six stereo input channels (L/R), each with independent level control, CV-controlled attenuation, and optional CV input for dynamic level adjustment. The module provides a master gain control to scale the final mixed output. It also includes visual feedback through an oscilloscope-style display and peak meter indicators.

## 2. Architecture

### Main Classes
- `Mixer6`: Core module class inheriting from `rack::Module`
- `MxPanelBackground`: Custom widget for drawing the panel background
- `MxDisplayWidget`: Custom widget for rendering the scope and meters
- `Mixer6Widget`: Module widget that constructs the UI layout

### Inheritance
- `Mixer6` inherits from `rack::Module`
- `MxPanelBackground` and `MxDisplayWidget` inherit from `rack::TransparentWidget`

### Structures
- `CVRange`: Enum defining CV input voltage range modes (`BIPOLAR_M5_5`, `UNIPOLAR_0_10`)
- `ParamId`: Enum for module parameters (6 channel levels, 6 channel attenuverters, master gain)
- `InputId`: Enum for module inputs (12 stereo audio inputs, 6 CV inputs)
- `OutputId`: Enum for module outputs (L/R mix)

### Relationships
- `Mixer6Widget` owns and manages the UI components (`MxPanelBackground`, `MxDisplayWidget`)
- `Mixer6` holds state data including scope buffers, peak values, and CV range mode
- All parameters and inputs are indexed using enums for type-safe access

### Module Organization
- Parameters: 19 total (6 levels + 6 attenuverters + 1 master gain)
- Inputs: 18 total (12 stereo audio + 6 CV)
- Outputs: 2 (L/R mix)

## 3. DSP Theory

The module implements a simple linear mixing algorithm with optional CV control.

### Mathematical Equations
For each channel $ ch \in [0,5] $:

$$
\text{gain}_{ch} = \max(0, \min(1, \text{level}_{ch} + \text{attenuverter}_{ch} \cdot \text{CV}_{ch}))
$$

$$
\text{mix}_L = \sum_{ch=0}^{5} \text{inL}_{ch} \cdot \text{gain}_{ch}
$$

$$
\text{mix}_R = \sum_{ch=0}^{5} \text{inR}_{ch} \cdot \text{gain}_{ch}
$$

$$
\text{out}_L = \text{mix}_L \cdot \text{masterGain}
$$

$$
\text{out}_R = \text{mix}_R \cdot \text{masterGain}
$$

### Algorithms
- Linear mixing with clamping to [0,1] range for gain
- CV scaling based on mode (`UNIPOLAR_0_10` or `BIPOLAR_M5_5`)
- Peak detection and decay filtering for meters
- Scope buffer updating with fixed-size circular buffer

### Filters
None explicitly implemented. Uses simple first-order decay filter for peak metering.

### Oscillators
None.

### Delay Lines
None.

### Feedback Systems
None.

### Modulation
CV inputs modulate channel gain via attenuverter control.

### Random Processes
None.

## 4. Parameters and Controls

### Knobs
- **Channel Level Knobs (6)**: Range [0,1], controls base gain per channel
- **Master Gain Knob**: Range [0,2], scales final mixed output

### Switches
- None

### Buttons
- None

### CV Inputs
- **Channel CV Inputs (6)**: Voltage-controlled level adjustment
  - Voltage range depends on `cvRangeMode`:
    - `UNIPOLAR_0_10`: 0V to +10V → scaled to [-1,1]
    - `BIPOLAR_M5_5`: -5V to +5V → scaled to [-1,1]

### Gates
- None

### Enumerations
- `CVRange`: 
  - `UNIPOLAR_0_10` (default)
  - `BIPOLAR_M5_5`

### Parameter Ranges
- Channel level: [0,1]
- Channel attenuverter: [-1,1]
- Master gain: [0,2]

### Default Values
- All channel levels: 0.0
- All channel attenuverters: 0.0
- Master gain: 1.0

## 5. Inputs and Outputs

### Audio Inputs
- **Channel L/R Pairs (6)**: 
  - Voltage range: ±10V (standard VCV Rack)
  - Behavior: Stereo audio input

### CV Inputs
- **Channel CV Inputs (6)**:
  - Voltage range: Depends on `cvRangeMode`
  - Behavior: Modulates channel gain via attenuverter

### Outputs
- **Mix L/R Outputs**:
  - Voltage range: ±10V
  - Behavior: Mixed stereo output signal

## 6. Signal Processing Pipeline

1. **Input Reading**: Read all 12 audio inputs and 6 CV inputs
2. **CV Scaling**: Scale CV values based on `cvRangeMode`
3. **Gain Calculation**: Compute per-channel gain with clamping:
   - Base gain = level knob value
   - Modulated gain = base + (attenuverter × scaled CV)
   - Clamp to [0,1]
4. **Mixing**: Accumulate weighted audio samples from all channels
5. **Master Scaling**: Apply master gain to mixed output
6. **Output**: Set L/R outputs with final values
7. **Display Update**:
   - Peak detection and decay filtering for meters
   - Scope buffer updating with mono signal

## 7. Important Functions

- `scaleCV(float v)`: Scales CV input voltage based on current mode
- `process(const ProcessArgs&)`: Main DSP processing function
- `dataToJson()`: Serializes module state to JSON
- `dataFromJson(json_t*)`: Deserializes module state from JSON
- `MxDisplayWidget::drawLayer()`: Renders scope and meters

## 8. Mathematical Implementation

### Gain Calculation
```cpp
float gain = levelKnob + cvOffset;
if (gain < 0.f) gain = 0.f;
if (gain > 1.f) gain = 1.f;
```

### CV Scaling
```cpp
if (cvRangeMode == BIPOLAR_M5_5)
    return std::max(-1.f, std::min(v / 5.f, 1.f));
return v / 10.f;  // UNIPOLAR_0_10 – original behavior
```

### Peak Detection with Decay
```cpp
float decay = 1.f - 4.f * args.sampleTime;
peakL = std::max(std::fabs(outL), peakL * decay);
peakR = std::max(std::fabs(outR), peakR * decay);
```

## 9. Performance and Optimization

### CPU Usage Considerations
- Simple linear mixing with minimal floating-point operations
- No complex algorithms or lookups
- Constant-time processing per sample

### Memory Allocation
- Fixed-size arrays for scope buffers (96 elements)
- Stack-based allocation for temporary variables
- No dynamic memory allocation during processing

### Buffers
- `scopeMin[MX_SCOPE_N]` and `scopeMax[MX_SCOPE_N]`: Circular buffer for oscilloscope display
- `scopeHead`, `scopeCount`: Buffer management indices

### Real-time Processing
- All operations performed in real-time within `process()` function
- No blocking or time-consuming operations

### Thread Safety
- Module state accessed only from main thread during `process()`
- No concurrent access patterns assumed

### Numerical Stability
- Clamping ensures gain values remain within [0,1]
- Decay factor uses sample time for consistent behavior across sample rates
- No recursive or feedback-based algorithms that could cause instability

## 10. Dependencies

- VCV Rack SDK (`rack.hpp`)
- Standard C++ libraries: `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`, `<string>`
- VCV Rack UI components: `TransparentWidget`, `ModuleWidget`, `DarkPJ301MPort`, `RoundSmallBlackKnob`, `Trimpot`, `RoundBlackKnob`
- JSON serialization via `json_t` from VCV Rack

## 11. Summary

The `Mixer6` module is a straightforward stereo 6-channel audio mixer with CV-controlled gain stages and master output scaling. It implements linear mixing with optional attenuation, peak detection for meters, and oscilloscope-style visualization. The design emphasizes simplicity and real-time performance while providing flexible control through parameter modulation. The module supports two CV input voltage ranges for compatibility with different control sources.