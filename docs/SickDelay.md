<!--
AGMM AUTOMATIC DOCUMENTATION
Source: SickDelay.cpp
Generated: 2026-11-04 06:31
-->

# SickDelay

## 1. Overview

The `SickDelay` module is a stereo delay effect for VCV Rack that provides flexible delay time control, feedback, mixing, tone shaping, and modulation. It features:

- Stereo input/output processing
- Independent left/right delay times
- Clock synchronization for rhythmic delays
- Modulation via LFO
- Bitcrushing effect
- Ping-pong stereo effect
- Freeze functionality to hold delay contents
- Visual display of delay line contents

The module implements a classic analog-style delay with digital signal processing enhancements, including saturation and filtering.

## 2. Architecture

### Main Classes

- `SickDelay`: Core module class inheriting from `rack::Module`
- `SickDelayWidget`: UI widget class inheriting from `rack::ModuleWidget`
- `SDPanel`: Custom panel drawing widget
- `SDDisplay`: Visual delay line display widget

### Inheritance

```
SickDelay → rack::Module
SickDelayWidget → rack::ModuleWidget
SDPanel → rack::TransparentWidget
SDDisplay → rack::TransparentWidget
```

### Structures

- **Parameter IDs**: Enumerated parameters for knobs, switches, and lights
- **Input/Output IDs**: Enumerated ports
- **Buffer Management**: Circular buffers for left/right delay lines
- **Clock Tracking**: Schmitt trigger for clock detection with timing measurement
- **Display Data**: Peak bins for visualizing delay line contents

### Relationships

- `SickDelayWidget` owns and manages all UI components including:
  - Panel drawing (`SDPanel`)
  - Visual display (`SDDisplay`)
  - Knobs, switches, lights
  - Input/output jacks
- `SickDelay` contains the audio processing logic and maintains state for:
  - Delay buffers
  - Clock tracking
  - Display data
  - Parameter values

### Module Organization

The module is organized into:
1. **Configuration**: Parameter setup in constructor
2. **Reset Handling**: Clock timer reset on module reset
3. **Audio Processing**: Main `process()` method with pipeline stages
4. **UI Components**: Panel drawing and visual display logic

## 3. DSP Theory

### Mathematical Equations

**Delay Line Read/Write:**
$$ y[n] = x[n - d] $$

Where:
- $ y[n] $: output sample at time n
- $ x[n] $: input sample at time n  
- $ d $: delay in samples

**Linear Interpolation for Fractional Delay:**
$$ y[n] = (1 - f) \cdot x[i_1] + f \cdot x[i_2} $$

Where:
- $ f $: fractional part of delay
- $ i_1, i_2 $: integer indices

**Feedback Loop:**
$$ y_{out}[n] = x[n] + (y_{delay}[n] \cdot fb) $$

Where:
- $ fb $: feedback gain

**Tanh Saturation:**
$$ y = \tanh(x \cdot drive) $$

**Low-pass Filter (One-pole):**
$$ y[n] = y[n-1] + color \cdot (x[n] - y[n-1]) $$

### Algorithms

1. **Clock Synchronization**: Measures time between clock pulses using Schmitt trigger
2. **Delay Line Reading**: Circular buffer with linear interpolation for fractional delays
3. **Modulation**: LFO sine wave applied to delay times
4. **Bitcrushing**: Quantization to 6-bit resolution
5. **Saturation**: Hyperbolic tangent function for soft clipping
6. **Filtering**: One-pole lowpass filter for tone shaping

### Filters

- **One-pole Lowpass Filter**: Simple first-order IIR filter
- **Tanh Saturation**: Nonlinear distortion function

### Oscillators

- **LFO**: Sine wave oscillator at 0.5 Hz rate (1 beat per second)

### Delay Lines

- **Circular Buffers**: Two separate buffers for left/right channels
- **Write Position Tracking**: Integer indices with modulo arithmetic
- **Fractional Read Access**: Linear interpolation between samples

### Feedback Systems

- **Feedback Loop**: Output fed back into delay line input
- **Freeze Mode**: Slow decay when frozen (0.999 feedback)

### Modulation

- **LFO Modulation**: Sine wave at 0.5 Hz rate
- **Modulation Depth Control**: Multiplies LFO output by parameter value
- **Time Modulation**: Adds modulation to delay time calculation

### Random Processes

- None implemented in this module

## 4. Parameters and Controls

### Knobs

| Parameter | ID | Default | Range | Units | Description |
|-----------|----|---------|-------|-------|-------------|
| TIME_PARAM | 0 | 0.5 | 0.0 - 1.0 | - | Left delay time (at 48 kHz) / division when synced |
| FEEDBACK_PARAM | 2 | 0.5 | 0.0 - 1.0 | - | Feedback |
| MIX_PARAM | 3 | 0.5 | 0.0 - 1.0 | - | Dry/wet mix |
| COLOR_PARAM | 4 | 0.2 | 0.01 - 0.5 | - | Tone (low = darker) |
| DRIVE_PARAM | 5 | 0.5 | 0.0 - 3.0 | - | Drive (saturation) |
| MOD_PARAM | 6 | 0.2 | 0.0 - 1.0 | - | Modulation depth |

### Switches

| Parameter | ID | Default | States | Description |
|-----------|----|---------|--------|-------------|
| PINGPONG_SWITCH | 7 | Off | Off/On | Ping-pong stereo effect |
| BITCRUSH_SWITCH | 8 | Off | Off/On | Bitcrushing effect |
| FREEZE_SWITCH | 9 | Off | Off/On | Freeze delay contents |
| LINK_SWITCH | 11 | Off | Off/On | Link L/R time (right follows left) |

### Buttons

- None explicitly defined as buttons in code

### CV Inputs

| Input | ID | Description |
|-------|----|-------------|
| TIME_CV_INPUT | 2 | Time CV, both channels (10 V = 100%) |
| FREEZE_CV_INPUT | 4 | Freeze gate |
| CLOCK_INPUT | 5 | Clock (1 pulse per beat) - syncs the delay times |

### Gates

- None explicitly defined as gates in code

### Enumerations

- **ParamId**: Parameter enumeration
- **InputId**: Input port enumeration  
- **OutputId**: Output port enumeration
- **LightId**: Light enumeration

### Parameter Ranges

- All parameters are normalized (0.0 - 1.0) except:
  - `TIME_PARAM` and `TIME_R_PARAM`: 0.0 - 1.0 with custom scale
  - `DRIVE_PARAM`: 0.0 - 3.0 (linear)
  - `COLOR_PARAM`: 0.01 - 0.5 (logarithmic-like)

### Default Values

- TIME_PARAM: 0.5
- FEEDBACK_PARAM: 0.5
- MIX_PARAM: 0.5
- COLOR_PARAM: 0.2
- DRIVE_PARAM: 0.5
- MOD_PARAM: 0.2
- PINGPONG_SWITCH: Off (0.0)
- BITCRUSH_SWITCH: Off (0.0)
- FREEZE_SWITCH: Off (0.0)
- LINK_SWITCH: Off (0.0)

## 5. Inputs and Outputs

### Inputs

| Input | ID | Voltage Range | Description |
|-------|----|---------------|-------------|
| LEFT_INPUT | 0 | -10V to +10V | Left input (mono if right input is empty) |
| RIGHT_INPUT | 1 | -10V to +10V | Right input |
| TIME_CV_INPUT | 2 | -10V to +10V | Time CV, both channels (10 V = 100%) |
| FREEZE_CV_INPUT | 4 | -10V to +10V | Freeze gate (2.0V threshold) |
| CLOCK_INPUT | 5 | -10V to +10V | Clock input (1 pulse per beat) |

### Outputs

| Output | ID | Voltage Range | Description |
|--------|----|---------------|-------------|
| OUT_L_OUTPUT | 0 | -10V to +10V | Left output |
| OUT_R_OUTPUT | 1 | -10V to +10V | Right output |

## 6. Signal Processing Pipeline

1. **Input Handling**: Read left/right inputs, mono fallback for right input
2. **Freeze Logic**: Check switch and CV gate for freeze activation
3. **Clock Tracking**: Measure time between clock edges using Schmitt trigger
4. **Modulation**: Generate LFO sine wave at 0.5 Hz rate
5. **Delay Time Calculation**: 
   - Apply CV input to knob value
   - Convert to samples or note divisions when synced
   - Add modulation and LFO effect
6. **Delay Line Reading**: Read from circular buffers with linear interpolation
7. **Bitcrushing**: Quantize output to 6-bit resolution if enabled
8. **Saturation**: Apply hyperbolic tangent distortion
9. **Filtering**: Apply one-pole lowpass filter for tone shaping
10. **Write to Delay Lines**: Write input + feedback to buffers (skip when frozen)
11. **Display Update**: Update visual peak bins for delay line visualization
12. **Output Processing**: Apply ping-pong effect and mix dry/wet signal

## 7. Important Functions

### `process()`
Main audio processing function that implements the complete DSP pipeline:
- Handles all parameter and input processing
- Manages clock synchronization
- Implements delay line operations
- Applies effects (bitcrush, saturation, filtering)
- Controls output mixing and routing

### `calcDelay()`
Lambda function for calculating delay time in samples:
- Converts knob + CV to normalized value
- Applies clock synchronization when valid
- Handles division mapping for rhythmic delays
- Ensures delay fits within buffer limits

### `readDelay()`
Lambda function for reading from delay line:
- Performs circular buffer indexing with modulo arithmetic
- Implements linear interpolation for fractional delays
- Returns interpolated sample value

### `onReset()`
Handles module reset:
- Resets clock trigger state
- Clears timing variables

## 8. Mathematical Implementation

### Delay Time Calculation

$$ d = \begin{cases}
\text{clockPeriod} \times SD\_DIV\_MULT[idx] \times \text{sampleRate}, & \text{if synced} \\
knob \times maxDelay, & \text{otherwise}
\end{cases} $$

Where:
- $ maxDelay = BUFFER\_SIZE - 2000 $
- $ SD\_DIV\_MULT = [0.25, 1/3, 0.5, 2/3, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0] $

### Linear Interpolation

$$ y[n] = (1 - f) \cdot x[i_1] + f \cdot x[i_2} $$

Where:
- $ f = readPos - floor(readPos) $
- $ i_1 = readPos \% BUFFER\_SIZE $
- $ i_2 = (i_1 + 1) \% BUFFER\_SIZE $

### One-pole Lowpass Filter

$$ y[n] = y[n-1] + color \cdot (x[n] - y[n-1]) $$

### Hyperbolic Tangent Saturation

$$ y = \tanh(x \cdot drive) $$

## 9. Performance and Optimization

### CPU Usage Considerations

- **Buffer Operations**: Circular buffer indexing with modulo arithmetic
- **Interpolation**: Linear interpolation adds minimal computational overhead
- **LFO Generation**: Simple sine wave calculation at each sample
- **Conditional Logic**: Switches and flags add minimal overhead

### Memory Allocation

- **Fixed Buffers**: Two circular buffers of size `BUFFER_SIZE` (96000 samples)
- **Stack Variables**: All processing variables allocated on stack
- **No Dynamic Allocation**: No heap allocation during audio processing

### Buffers

- **Size**: 96000 samples (about 1.96 seconds at 48 kHz)
- **Type**: Circular buffers for left/right channels
- **Access**: Integer indices with modulo arithmetic

### Real-time Processing

- **Sample-based**: All operations performed per sample
- **No Blocking**: Non-blocking audio processing
- **Thread Safety**: Single-threaded audio processing model

### Thread Safety

- **Audio Thread Only**: All processing occurs in audio thread context
- **Display Data**: Torn reads are cosmetic only (no critical data corruption)

### Numerical Stability

- **Clamping**: All delay values clamped to valid ranges
- **Overflow Prevention**: Circular buffer indices use modulo arithmetic
- **Precision**: Single precision floating point used throughout

## 10. Dependencies

### Libraries and Headers

- `rack.hpp`: VCV Rack core library
- `<cmath>`: Standard math functions (sinf, tanhf)
- `<cstdio>`: Standard I/O functions
- `<cstring>`: String operations
- `<algorithm>`: Standard algorithms

### VCV Rack Components

- `dsp::SchmittTrigger`: Clock edge detection
- `rack::math::crossfade`: Linear crossfader
- `Module`, `ModuleWidget`: Base module classes
- `Param`, `Input`, `Output`, `Light`: Port and parameter types
- `Vec`, `DrawArgs`: Vector and drawing utilities

### Custom Components

- `TransparentWidget`: Base for custom UI widgets
- `RoundSmallBlackKnob`: Knob widget type
- `CKSS`: Toggle switch widget type
- `DarkPJ301MPort`: Jack port widget type
- `SmallLight`: Light widget type

## 11. Summary

The `SickDelay` module implements a stereo delay effect with advanced features including:

**Core Features:**
- Stereo delay processing with independent left/right times
- Clock synchronization for rhythmic delays (10 note divisions)
- Modulation via LFO
- Bitcrushing and saturation effects
- Ping-pong stereo effect
- Freeze functionality

**Technical Implementation:**
- Uses circular buffers for efficient delay line storage
- Implements linear interpolation for fractional delays
- Includes one-pole lowpass filtering for tone shaping
- Features clock tracking with Schmitt trigger detection
- Provides visual feedback through display widget

**Performance Characteristics:**
- Real-time processing at 48 kHz sample rate
- Fixed memory allocation (no dynamic allocation)
- Efficient circular buffer operations
- Minimal CPU overhead from conditional logic and interpolation

The module provides a rich set of controls for creative delay manipulation while maintaining audio quality and performance suitable for real-time VCV Rack use.