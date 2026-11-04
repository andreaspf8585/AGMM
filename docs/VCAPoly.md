<!--
AGMM AUTOMATIC DOCUMENTATION
Source: VCAPoly.cpp
Generated: 2026-11-04 06:38
-->

# VCAPoly

## 1. Overview

The `VCAPoly` module is a polyphonic voltage-controlled amplifier (VCA) designed for the VCV Rack platform. It provides per-channel gain control with envelope following and global or per-channel limit control. The module supports up to 16 voices and visualizes the current gain and output levels via an on-module display.

## 2. Architecture

### Main Classes

- `VCAPoly`: Core DSP module class inheriting from `rack::Module`
- `VPPanel`: UI panel widget for module appearance
- `VPDisplay`: Custom display widget showing gain bars and metering
- `VCAPolyWidget`: Module widget that ties the module to its UI components

### Inheritance

- `VCAPoly` inherits from `rack::Module`
- `VPPanel` and `VPDisplay` inherit from `rack::TransparentWidget`

### Structures

- Enumerations for parameters, inputs, outputs, and lights
- Arrays for storing per-channel data:
  - `envelopeFill[16]`: Fill ratio (0..1) of envelope per voice
  - `gainNow[16]`: Current gain per channel
  - `outPeak[16]`: Peak output level per channel
- Shared data between audio and UI threads:
  - `activeCh`: Number of active channels
  - `envConn`: Whether envelope input is connected
  - `limitCvConn`: Whether limit CV input is connected

### Relationships

- `VCAPolyWidget` contains a `VPPanel` and `VPDisplay`
- `VCAPolyWidget` manages connections to `VCAPoly` module
- `VPDisplay` reads from `VCAPoly`'s shared data for visualization
- `VCAPoly` processes audio in its `process()` method

## 3. DSP Theory

### Algorithms

The core algorithm is a per-channel envelope-following VCA with optional limit control:

1. **Envelope Detection**: Voltage from `INPUT_ENV` is normalized to [0,1] range
2. **Gain Calculation**: 
   - Base gain = `envVoltage / 10.0 * limit`
   - If no envelope input: unity gain (gain = 1)
3. **Limiting**:
   - Global limit from knob (`PARAM_LIMIT`)
   - Per-channel override via `INPUT_LIMIT_CV` if connected
4. **Output**: `output = audio * gain`

### Filters

None implemented in this module.

### Oscillators

None implemented in this module.

### Delay Lines

None implemented in this module.

### Feedback Systems

None implemented in this module.

### Modulation

- Envelope modulation of gain (via `INPUT_ENV`)
- Gain limit modulation via CV input (`INPUT_LIMIT_CV`)
- Global gain limit control via knob (`PARAM_LIMIT`)

### Random Processes

None implemented in this module.

## 4. Parameters and Controls

### Knobs

- **PARAM_LIMIT**:
  - Range: [0, 2]
  - Default: 1
  - Units: Gain multiplier (x)
  - Function: Global gain limit control

### Switches/Buttons

None

### CV Inputs

- **INPUT_ENV**: Polyphonic envelope CV input (0 to 10 V)
  - Function: Modulates gain per channel
- **INPUT_LIMIT_CV**: Polyphonic gain limit CV input (0 to 10 V)
  - Function: Overrides global limit per channel when connected

### Gates

None

### Enumerations

- `ParamId`: { PARAM_LIMIT }
- `InputId`: { INPUT_AUDIO, INPUT_ENV, INPUT_LIMIT_CV }
- `OutputId`: { OUTPUT_AUDIO }
- `LightId`: { LIGHTS_LEN }

### Parameter Ranges and Default Values

| Parameter | Range | Default |
|-----------|-------|---------|
| PARAM_LIMIT | [0, 2] | 1 |

## 5. Inputs and Outputs

### Inputs

- **INPUT_AUDIO** (Polyphonic):
  - Voltage range: Any
  - Function: Audio input signal to be processed
- **INPUT_ENV** (Polyphonic):
  - Voltage range: [0, 10] V
  - Function: Envelope control signal for gain modulation
- **INPUT_LIMIT_CV** (Polyphonic):
  - Voltage range: [0, 10] V
  - Function: Per-channel limit override

### Outputs

- **OUTPUT_AUDIO** (Polyphonic):
  - Voltage range: Any
  - Function: Processed audio output with gain applied

## 6. Signal Processing Pipeline

1. **Channel Count Detection**: Determine number of active audio channels
2. **Connection Check**: Verify if envelope and limit CV inputs are connected
3. **Per-Channel Processing**:
   - Get audio voltage from input
   - Calculate per-channel limit (global knob or CV override)
   - Normalize envelope voltage to [0,1]
   - Compute gain: `gain = envNorm * limit`
   - Apply gain to audio: `output = audio * gain`
   - Update metering data (`envelopeFill`, `gainNow`, `outPeak`)
4. **Output Assignment**: Set output channels to match input channels
5. **Peak Detection**: Update peak values with exponential decay (time constant = 5 samples)

## 7. Important Functions

### Main Processing Function

- `process(const ProcessArgs& args)`: Core audio processing routine that handles all channel logic and gain calculations

### UI Drawing Functions

- `drawPanelBase()`, `drawBezel()`, `drawLogo()`, etc.: UI rendering functions for panel appearance
- `VPDisplay::drawLayer()`: Renders the visualization display including gain bars and peak meters

## 8. Mathematical Implementation

### Gain Calculation

$$
g = \frac{V_{env}}{10} \cdot L
$$

Where:
- $ g $: gain factor (0 to 2)
- $ V_{env} $: envelope voltage (0 to 10 V)
- $ L $: limit value (0 to 2)

### Limit Override

$$
L_{channel} = 
\begin{cases}
L_{global}, & \text{if } \texttt{limitCvConnected} = \text{false} \\
\min\left(\max\left(\frac{V_{cv}}{10}, 0\right), 2\right), & \text{if } \texttt{limitCvConnected} = \text{true}
\end{cases}
$$

### Peak Detection

$$
peak_{new} = \max(|output|, peak_{old} \cdot (1 - 5 \cdot \Delta t))
$$

Where:
- $ \Delta t $: sample time
- The decay factor is approximately 0.999999999 for typical sample rates

## 9. Performance and Optimization

### CPU Usage Considerations

- Single loop processing per channel (16 max)
- Minimal mathematical operations per sample
- Efficient use of `clamp()` function

### Memory Allocation

- Static arrays for channel data (16 elements each)
- No dynamic allocation during audio processing
- Thread-safe access to shared data between audio and UI threads

### Buffers

- No dedicated buffers used
- Per-channel data stored in static arrays

### Real-time Processing

- All operations performed within `process()` method
- No blocking or time-consuming operations
- Efficient use of VCV Rack's real-time processing model

### Thread Safety

- Shared data (`envelopeFill`, `gainNow`, `outPeak`, etc.) accessed by both audio and UI threads
- Audio thread writes to shared data, UI thread reads from it
- No explicit synchronization mechanisms used (assumes proper VCV Rack threading model)

### Numerical Stability

- Use of `clamp()` for input validation
- Proper handling of zero division cases
- Exponential decay for peak detection ensures stable behavior

## 10. Dependencies

- **Headers**: `plugin.hpp`, `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`
- **Classes**: `rack::Module`, `rack::TransparentWidget`, `rack::ModuleWidget`
- **Functions**: `nvgRGB()`, `nvgRGBA()`, `nvgLinearGradient()`, `nvgRadialGradient()`, `nvgHSL()`
- **Constants**: VCV Rack grid constants (`RACK_GRID_WIDTH`, `RACK_GRID_HEIGHT`)

## 11. Summary

The `VCAPoly` module implements a polyphonic voltage-controlled amplifier with envelope following and gain limiting capabilities. It supports up to 16 channels of audio processing, with each channel's gain controlled by an envelope input and optionally limited by either a global knob or per-channel CV input. The module provides visual feedback through an on-module display showing current gain levels and output peaks for each voice. The implementation is efficient and suitable for real-time audio processing within the VCV Rack environment, using standard DSP techniques with minimal computational overhead.