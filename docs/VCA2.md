<!--
AGMM AUTOMATIC DOCUMENTATION
Source: VCA2.cpp
Generated: 2026-11-04 06:37
-->

# VCA2

## 1. Overview

The **VCA2** module is a dual-channel Voltage-Controlled Amplifier (VCA) designed for the VCV Rack modular synthesizer platform. It allows users to control the amplitude of two independent audio signals using envelope control voltages, with adjustable gain limits per channel.

Each channel supports:
- Audio input
- Envelope CV input (0–10V)
- Output signal
- Interactive limit bar for setting and visualizing gain limits

The module includes a graphical user interface (GUI) that displays real-time information such as envelope percentage relative to the limit and output peak levels.

## 2. Architecture

### Main Classes

| Class | Description |
|-------|-------------|
| `VCA2` | Core module class inheriting from `rack::Module`. Implements signal processing logic and parameter handling. |
| `V2Panel` | Widget for drawing the main panel background including title, bezel, and labels. |
| `V2Display` | Widget for drawing the screen area with channel labels. |
| `V2Bar` | Interactive limit bar widget for each channel allowing user adjustment of gain limits via mouse drag or double-click reset. |
| `VCA2Widget` | Widget container class that assembles all UI elements (`V2Panel`, `V2Display`, `V2Bar`) and connects inputs/outputs to the module. |

### Inheritance

- `VCA2` inherits from `rack::Module`.
- All GUI widgets inherit from `rack::TransparentWidget`.

### Structures

- `VCA2`: Main module structure with parameters, inputs, outputs, and internal state.
- `V2Bar`: Interactive limit bar widget with drag/double-click behavior.

### Relationships Between Classes

- `VCA2Widget` owns instances of:
  - `V2Panel`
  - `V2Display`
  - Two `V2Bar` widgets (one per channel)
  - Input/output ports
- Each `V2Bar` holds a pointer to its associated `VCA2` module and knows which channel it controls.
- The `VCA2` module stores:
  - Parameter values (`PARAM_LIMIT1`, `PARAM_LIMIT2`)
  - Input voltage data (`INPUT_AUDIO1`, `INPUT_AUDIO2`, `INPUT_ENV1`, `INPUT_ENV2`)
  - Output voltage data (`OUTPUT_1`, `OUTPUT_2`)
  - Display-related state: `barFillPercent`, `outPeak`, `envConn`

### Module Organization

The module is organized into:
- Core DSP logic in the `process()` method of `VCA2`
- GUI rendering code in anonymous namespace functions
- Widget assembly logic in `VCA2Widget` constructor

## 3. DSP Theory

This module implements a basic **voltage-controlled amplifier** with envelope modulation and gain limiting.

### Algorithm

For each channel (ch = 0 or 1):

1. If no envelope is connected:
   - Output = Input audio signal
   - Bar fill percentage = 100%

2. If envelope is connected:
   - Normalize envelope voltage from 0–10V to 0–1
   - Apply gain: `out = in * envNorm * limit`
   - Calculate bar fill percentage based on normalized envelope and limit:
     $$
     \text{fill} = 
     \begin{cases}
     \left(\frac{\text{env}}{\text{limit} \cdot 10}\right) \times 100\%, & \text{if } \text{limit} > 0 \\
     0, & \text{otherwise}
     \end{cases}
     $$

### Filters

None implemented.

### Oscillators

None implemented.

### Delay Lines

None implemented.

### Feedback Systems

None implemented.

### Modulation

Envelope modulation is used to control gain. The envelope CV input is normalized from 0–10V range to 0–1 before multiplication with the audio signal and limit.

### Random Processes

None implemented.

## 4. Parameters and Controls

| Parameter | ID | Description | Range | Default |
|----------|----|-------------|-------|---------|
| `PARAM_LIMIT1` | 0 | Channel 1 gain limit | 0–2x | 1x |
| `PARAM_LIMIT2` | 1 | Channel 2 gain limit | 0–2x | 1x |

### Enumerations

```cpp
enum ParamId { PARAM_LIMIT1, PARAM_LIMIT2, PARAMS_LEN };
enum InputId { INPUT_AUDIO1, INPUT_AUDIO2, INPUT_ENV1, INPUT_ENV2, INPUTS_LEN };
enum OutputId { OUTPUT_1, OUTPUT_2, OUTPUTS_LEN };
enum LightId { LIGHTS_LEN };
```

### Parameter Ranges

- Gain limits are clamped between 0 and 2.
- Default value for both channels is 1x.

## 5. Inputs and Outputs

| Name | ID | Type | Voltage Range | Description |
|------|----|------|---------------|-------------|
| `INPUT_AUDIO1` | 0 | Audio In | ±10V typical | Channel 1 audio input |
| `INPUT_AUDIO2` | 1 | Audio In | ±10V typical | Channel 2 audio input |
| `INPUT_ENV1` | 2 | CV In | 0–10V | Channel 1 envelope control voltage |
| `INPUT_ENV2` | 3 | CV In | 0–10V | Channel 2 envelope control voltage |
| `OUTPUT_1` | 0 | Audio Out | ±10V typical | Channel 1 output |
| `OUTPUT_2` | 1 | Audio Out | ±10V typical | Channel 2 output |

## 6. Signal Processing Pipeline

1. **Input Reading**: Read audio and envelope voltages from corresponding inputs.
2. **Envelope Normalization**: Convert envelope CV (0–10V) to normalized value (0–1).
3. **Gain Calculation**:
   - If no envelope connected: output = input audio
   - Else: output = input audio × normalized envelope × limit
4. **Bar Fill Percentage Update**:
   $$
   \text{fill} = 
   \begin{cases}
   \left(\frac{\text{env}}{\text{limit} \cdot 10}\right) \times 100\%, & \text{if } \text{limit} > 0 \\
   0, & \text{otherwise}
   \end{cases}
   $$
5. **Peak Detection**: Update peak meter using leaky integrator:
   $$
   \text{peak}[ch] = \max(|\text{out}|, \text{peak}[ch] \cdot (1 - 5 \cdot \Delta t))
   $$

## 7. Important Functions

| Function | Responsibility |
|---------|----------------|
| `VCA2::process()` | Main DSP loop; handles input/output processing and bar fill updates |
| `V2Bar::onButton()` | Handles mouse press to initiate dragging of limit bar |
| `V2Bar::onDragMove()` | Updates limit value during drag operation |
| `V2Bar::onDoubleClick()` | Resets limit to 1x on double-click |
| `VCA2Widget::VCA2Widget()` | Assembles UI components and connects ports |

## 8. Mathematical Implementation

### Gain Calculation

For channel `ch`:
$$
\text{out} = 
\begin{cases}
\text{in}, & \text{if } \text{envConn}[ch] == \text{false} \\
\text{in} \times \left(\frac{\text{env}}{10}\right) \times \text{limit}, & \text{if } \text{envConn}[ch] == \text{true}
\end{cases}
$$

### Bar Fill Percentage

$$
\text{fill} = 
\begin{cases}
\left(\frac{\text{env}}{\text{limit} \cdot 10}\right) \times 100\%, & \text{if } \text{limit} > 0 \\
0, & \text{otherwise}
\end{cases}
$$

### Peak Detection

Leaky integrator:
$$
\text{peak}[ch] = \max(|\text{out}|, \text{peak}[ch] \cdot (1 - 5 \cdot \Delta t))
$$

## 9. Performance and Optimization

- **CPU Usage**: Minimal; only performs simple arithmetic operations per sample.
- **Memory Allocation**: Static allocation used for all data structures.
- **Buffers**: No dedicated buffers required; uses built-in VCV Rack input/output buffers.
- **Real-time Processing**: Designed for real-time audio processing with fixed-size loops.
- **Thread Safety**: Module assumes single-threaded access during `process()` call.
- **Numerical Stability**: Clamp operations prevent invalid values; normalized envelope ensures bounded gain.

## 10. Dependencies

### Libraries/Headers

- `rack.hpp`: Core VCV Rack API
- `<cmath>`: Standard math functions (`std::fabs`, `std::max`)
- `<cstdio>`: Standard I/O (used in `snprintf`)
- `<cstring>`: String operations (not used directly)
- `<algorithm>`: Standard algorithms (`std::clamp`)

### Classes

- `rack::Module`
- `rack::TransparentWidget`
- `rack::Vec`, `rack::NVGcontext`, `rack::DrawArgs`
- `rack::ButtonEvent`, `rack::DragMoveEvent`, `rack::DoubleClickEvent`

## 11. Summary

The **VCA2** module is a dual-channel voltage-controlled amplifier with envelope modulation and adjustable gain limits. It provides real-time visualization of envelope percentage relative to the limit and output peak levels through an interactive GUI. The DSP algorithm computes output signals using normalized envelope control voltages multiplied by input audio and channel-specific gain limits, with optional unity gain when no envelope is connected. The module supports both analog-style modulation and digital precision in its calculations, making it suitable for dynamic audio shaping within a modular synthesizer environment.