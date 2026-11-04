<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Reverb.cpp
Generated: 2026-11-04 06:30
-->

# Reverb

## 1. Overview

The `Reverb` module is a VCV Rack plugin implementing a simple yet effective reverb effect using a feedback delay line with damping and drive control. It simulates the natural decay of sound in an acoustic space by introducing delayed, attenuated versions of the input signal back into itself.

Key features include:
- Adjustable decay time (feedback)
- Damping filter to simulate high-frequency loss
- Variable delay time
- Drive control for distortion
- Dry/wet mix
- Output gain control
- Visual scope display showing signal amplitude over time

## Module Screenshot

![Reverb VCV Rack Module](images/reverb.png)

## 2. Architecture

### Main Classes

- `Reverb`: The core DSP module inheriting from `rack::Module`
- `RvPanelBackground`: Custom widget for panel background drawing
- `RvDisplayWidget`: Custom widget for the scope and readout display
- `ReverbWidget`: VCV Rack widget that ties together the module and its UI

### Inheritance

The `Reverb` class inherits from `rack::Module`, providing standard VCV Rack module functionality including parameter configuration, input/output handling, and processing.

### Structures

- `RvCellDef`: Structure mapping control names to parameter IDs for layout
- `RvDisplayWidget`: Custom drawing widget for scope visualization
- `RvPanelBackground`: Custom drawing widget for panel background elements

### Relationships

The module is composed of:
1. DSP processing logic in the `Reverb` class
2. UI components (`ReverbWidget`, `RvDisplayWidget`, `RvPanelBackground`)
3. Layout constants defined in anonymous namespace
4. Parameter and input/output definitions within `Reverb`

### Module Organization

The module is organized as a single `.cpp` file with:
- DSP implementation (`Reverb` class)
- UI drawing code in anonymous namespace
- Widget construction logic
- Model registration at the end

## 3. DSP Theory

This reverb uses a feedback delay line (FDBL) architecture with damping and drive effects.

### Mathematical Equations

The core algorithm is based on:

1. **Feedback Delay Line**:
   $$
   y[n] = x[n] + \alpha \cdot d[n - D]
   $$

2. **Low-pass Filtering**:
   $$
   y_{lp}[n] = y_{lp}[n-1] + \beta (x[n] - y_{lp}[n-1])
   $$

3. **Tanh Distortion**:
   $$
   y = \tanh(\alpha x)
   $$

4. **Linear Interpolation for Delay Read**:
   $$
   d[n] = d_0[n] + f(d_1[n] - d_0[n])
   $$

### Algorithms

- Feedback delay line with linear interpolation
- First-order low-pass filter for damping
- Tanh distortion function for drive effect
- Dry/wet mixing
- Output gain control

### Filters

A first-order low-pass filter is implemented using a recursive formula:
$$
y[n] = y[n-1] + \beta(x[n] - y[n-1])
$$

Where $\beta$ is the damping coefficient calculated from the damping parameter.

### Delay Lines

The delay line uses a circular buffer with:
- Fixed maximum size: `MAX_DELAY = 192000` samples
- Linear interpolation between delay taps for smooth modulation
- Read position calculation with wraparound logic

### Feedback Systems

Feedback is implemented as:
$$
\text{feedbackIn} = \text{input} + \text{filteredDelay} \times \text{time}
$$

Where `time` represents the feedback coefficient (decay parameter).

### Modulation

The delay time is modulated via a parameter that maps to actual samples using:
$$
\text{delaySamples} = \text{delayMs} \times 0.001 \times \text{sampleRate}
$$

### Random Processes

No random processes are implemented in this module.

## 4. Parameters and Controls

### Knobs

- **DECAY (TIME_PARAM)**: Range [0, 0.995], default 0.5
- **DAMP (DAMP_PARAM)**: Range [0, 1], default 0.5
- **DELAY (DELAY_MS_PARAM)**: Range [0, 1000], default 200 ms
- **DRIVE (DRIVE_PARAM)**: Range [0, 1], default 0
- **MIX (MIX_PARAM)**: Range [0, 1], default 0.5
- **GAIN (OUTPUT_GAIN_PARAM)**: Range [0, 3], default 1

### Switches/Buttons

None

### CV Inputs

- **AUDIO_INPUT**: Audio signal input

### Gates

None

### Enumerations

- `ParamId`: Defines parameter IDs for all controls
- `InputId`: Defines input IDs (single audio input)
- `OutputId`: Defines output IDs (single audio output)

### Parameter Ranges and Defaults

| Parameter | Min | Max | Default |
|-----------|-----|-----|---------|
| TIME_PARAM | 0.0 | 0.995 | 0.5 |
| DAMP_PARAM | 0.0 | 1.0 | 0.5 |
| DELAY_MS_PARAM | 0.0 | 1000.0 | 200.0 |
| DRIVE_PARAM | 0.0 | 1.0 | 0.0 |
| MIX_PARAM | 0.0 | 1.0 | 0.5 |
| OUTPUT_GAIN_PARAM | 0.0 | 3.0 | 1.0 |

## 5. Inputs and Outputs

### AUDIO_INPUT

- **Type**: Audio input
- **Voltage Range**: ±10V (standard VCV Rack audio range)
- **Signal Behavior**: Input audio signal to be processed by the reverb effect

### AUDIO_OUTPUT

- **Type**: Audio output
- **Voltage Range**: ±10V (standard VCV Rack audio range)
- **Signal Behavior**: Mixed dry/wet processed audio signal

## 6. Signal Processing Pipeline

1. Read input voltage from `AUDIO_INPUT`
2. Retrieve parameter values:
   - Time (feedback coefficient)
   - Damping
   - Delay time in ms
   - Drive amount
   - Mix ratio
   - Output gain
3. Calculate damping coefficient from damping parameter using:
   $$
   \text{cutoff} = 200 \times \left(\frac{8000}{200}\right)^{\text{damp}} \\
   \beta = 1 - e^{-2\pi \cdot \text{cutoff}/\text{sampleRate}}
   $$
4. Calculate delay samples from delay time:
   $$
   \text{delaySamples} = \text{delayMs} \times 0.001 \times \text{sampleRate}
   $$
5. Perform linear interpolation on delay line read positions
6. Apply low-pass filtering to delayed signal using damping coefficient
7. Calculate feedback input:
   $$
   \text{feedbackIn} = \text{input} + \text{filteredDelay} \times \text{time}
   $$
8. Store feedback input in delay buffer with tanh distortion
9. Apply drive effect if enabled:
   $$
   y = \tanh(\text{driveAmount} \cdot x) \times \text{norm}
   $$
10. Mix dry and wet signals:
    $$
    \text{mixed} = (1 - \text{mix}) \cdot \text{input} + \text{mix} \cdot \text{wet}
    $$
11. Apply output gain with tanh distortion:
    $$
    \text{out} = 5 \cdot \tanh(\text{mixed} \times \text{outGain})
    $$
12. Output processed signal to `AUDIO_OUTPUT`

## 7. Important Functions

### `Reverb::process()`

Main DSP processing function that:
- Reads input and parameters
- Calculates delay line positions with interpolation
- Applies damping filter
- Implements feedback loop
- Applies drive effect
- Mixes dry/wet signals
- Outputs final signal with gain control

### `RvDisplayWidget::drawLayer()`

Handles scope visualization by:
- Drawing grid lines
- Rendering amplitude buckets
- Drawing level meter
- Displaying parameter readouts

## 8. Mathematical Implementation

### Damping Coefficient Calculation

$$
\beta = 1 - e^{-2\pi \cdot \text{cutoff}/\text{sampleRate}}
$$

Where cutoff frequency is mapped from damping parameter:
$$
\text{cutoff} = 200 \times \left(\frac{8000}{200}\right)^{\text{damp}} = 200 \times 40^{\text{damp}}
$$

### Linear Interpolation

For delay read positions:
$$
\text{delayed} = d_0 + f(d_1 - d_0)
$$

Where $f$ is the fractional part of delay samples.

### Tanh Distortion

$$
y = \tanh(\alpha x) \times \text{norm}
$$

With normalization:
$$
\text{norm} = \frac{1}{\tanh(\text{driveAmount})}
$$

## 9. Performance and Optimization

### CPU Usage Considerations

- Uses fixed-size circular buffer (constant memory)
- Simple mathematical operations (exp, tanh, clamp)
- Minimal branching in main processing loop
- Efficient interpolation using integer arithmetic

### Memory Allocation

- Static buffer: `float delayBuffer[MAX_DELAY]` (192000 floats = ~768KB)
- Small fixed-size arrays for scope tracking
- No dynamic allocation during processing

### Buffers

- Circular delay line buffer of size 192000 samples
- Scope tracking buffers of 96 elements each

### Real-time Processing

- All operations are real-time safe
- No blocking or time-consuming operations
- Fixed sample rate processing

### Thread Safety

- Single-threaded DSP processing
- No shared state between processing threads

### Numerical Stability

- Clamp all values to prevent overflow/underflow
- Proper handling of delay line wraparound
- Use of `std::tanh` for distortion (stable)
- Proper interpolation to avoid artifacts

## 10. Dependencies

### Libraries

- VCV Rack SDK (`rack.hpp`)
- Standard C++ libraries: `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`

### Classes and Headers

- `rack::Module` (base module class)
- `rack::Param` (parameter handling)
- `rack::Input`/`Output` (input/output handling)
- `rack::math::clamp` (clamping function)
- `nvg::context` (for UI drawing)
- `Vec`, `Widget`, `TransparentWidget` (VCV Rack UI classes)

## 11. Summary

The `Reverb` module implements a classic feedback delay line reverb effect with damping and drive controls. It uses a circular buffer for the delay line, first-order low-pass filtering for damping, and tanh distortion for drive effects. The implementation is optimized for real-time performance with fixed-size buffers and efficient mathematical operations. The module provides visual feedback through a scope display showing signal amplitude over time, and includes comprehensive parameter controls for shaping the reverb characteristics.