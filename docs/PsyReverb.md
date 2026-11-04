<!--
AGMM AUTOMATIC DOCUMENTATION
Source: PsyReverb.cpp
Generated: 2026-11-04 06:27
-->

# PsyReverb

## 1. Overview

The `PsyReverb` module is a stereo reverb effect designed for the VCV Rack platform. It implements a Freeverb-style reverb algorithm with modifications to prevent runaway gain, ensuring stable audio processing without clipping or instability.

Key features:
- Stereo input/output processing
- Four main parameters: Wet, Dry, Room size, Damping
- CV control with attenuverters for all parameters
- Mono-to-stereo conversion when only left input is connected
- Visual metering for input and output signals
- Real-time parameter updates via clock divider

## Module Screenshot

![PsyReverb VCV Rack Module](images/psyreverb.png)

## 2. Architecture

### Main Classes

1. **`PsyReverbEngine`** - Core DSP engine implementing the reverb algorithm
2. **`PsyReverb`** - VCV Rack module wrapper managing parameters, inputs/outputs, and processing
3. **`PRPanel`** - UI panel drawing for the module's front panel
4. **`PRDisplay`** - Visual display showing reverb characteristics

### Inheritance

- `PsyReverb` inherits from `rack::Module`
- UI components inherit from `rack::widget::TransparentWidget`

### Structures

#### `PsyReverbEngine::DelayLine`
A circular buffer implementation for delay lines:
```cpp
struct DelayLine {
    float *buffer = nullptr;
    int length = 0;
    int idx = 0;
    
    void init(int len);
    void destroy();
    float read() const;
    void write(float value);
    void reset();
};
```

#### `PsyReverbEngine`
Main reverb engine with:
- 8 comb filters (delay lines)
- 4 allpass filters (delay lines)
- Lowpass filter state for each comb
- Parameter storage (`wet`, `dry`, `roomsize`, `damping`)
- Sample rate tracking

### Module Organization

The module is organized as a single `.cpp` file with:
1. Layout constants and helper functions
2. DSP engine implementation (`PsyReverbEngine`)
3. VCV Rack module wrapper (`PsyReverb`)
4. UI components in anonymous namespace
5. Widget class (`PsyReverbWidget`) for GUI

## 3. DSP Theory

### Algorithm Overview

The reverb uses a modified Freeverb architecture with:
- **Comb filters**: 8 parallel delay lines with feedback
- **Allpass filters**: 4 delay lines with feedback (used to color the reverb)
- **Lowpass filtering**: Damped comb filter response
- **Feedback gain control**: Prevents runaway gain

### Mathematical Equations

#### Comb Filter Response
$$y[n] = x[n] + g \cdot d[n - L]$$

Where:
- $x[n]$ is input signal
- $y[n]$ is output signal  
- $g$ is feedback gain (0.5 + roomsize × 0.48)
- $d[n - L]$ is delayed sample at delay length $L$

#### Lowpass Filter (One-pole)
$$y[n] = y[n-1] + \alpha(x[n] - y[n-1])$$

Where:
$$\alpha = 1 - e^{-2\pi f_c / f_s}$$

#### Allpass Filter
$$y[n] = -x[n] + d[n - L]$$
$$d[n] = x[n] + y[n] \cdot g$$

### Filters and Delay Lines

The implementation uses:
- **Comb filters**: 8 delay lines with different lengths (0.0297s to 0.0697s)
- **Allpass filters**: 4 delay lines with different lengths (0.0051s to 0.0149s)  
- **Lowpass filtering**: Applied to comb filter outputs for damping effect

### Feedback System

Feedback gain is calculated as:
$$g_{fb} = 0.5 + \text{roomsize} \times 0.48$$

This ensures maximum feedback gain of 0.98, preventing instability.

## 4. Parameters and Controls

### Knobs
- **Wet**: Mix between dry and wet signal (0-100%)
- **Dry**: Mix between input and output (0-100%)  
- **Room size**: Size of the virtual room (0-100%)
- **Damping**: High-frequency damping (0-100%)

### Switches/Buttons
None

### CV Inputs
- **Wet CV**: Controls wet parameter (10V = 100%)
- **Dry CV**: Controls dry parameter (10V = 100%)
- **Room size CV**: Controls room size (10V = 100%)
- **Damping CV**: Controls damping (10V = 100%)

### Gates
None

### Enumerations
None

### Parameter Ranges and Defaults
| Parameter | Min | Max | Default |
|-----------|-----|-----|---------|
| Wet       | 0.0 | 1.0 | 0.5     |
| Dry       | 0.0 | 1.0 | 0.5     |
| Room size | 0.0 | 1.0 | 0.8     |
| Damping   | 0.0 | 1.0 | 0.5     |

### Attenuverter Parameters
- **Wet CV attenuverter**: -1.0 to +1.0 (default: 0.0)
- **Dry CV attenuverter**: -1.0 to +1.0 (default: 0.0)
- **Room size CV attenuverter**: -1.0 to +1.0 (default: 0.0)
- **Damping CV attenuverter**: -1.0 to +1.0 (default: 0.0)

## 5. Inputs and Outputs

### Inputs
| Name | Voltage Range | Behavior |
|------|---------------|----------|
| Left input | ±10V | Stereo processing, mono if right is disconnected |
| Right input | ±10V | Stereo processing |
| Wet CV | 0-10V | Controls wet parameter (10V = 100%) |
| Dry CV | 0-10V | Controls dry parameter (10V = 100%) |
| Room size CV | 0-10V | Controls room size (10V = 100%) |
| Damping CV | 0-10V | Controls damping (10V = 100%) |

### Outputs
| Name | Voltage Range | Behavior |
|------|---------------|----------|
| Left output | ±10V | Processed stereo signal |
| Right output | ±10V | Processed stereo signal |

## 6. Signal Processing Pipeline

1. **Parameter Update** (every 32 samples):
   - Read parameter knobs
   - Apply CV input with attenuverter
   - Clamp to valid range [0,1]
   - Update reverb engine parameters

2. **Input Processing**:
   - Read left and right inputs
   - If right input disconnected, use left input for both channels (mono-to-stereo)

3. **Reverb Processing**:
   - For each comb filter:
     - Read delayed sample
     - Apply lowpass filtering with damping
     - Calculate feedback gain: $g = 0.5 + \text{roomsize} \times 0.48$
     - Write new sample to delay line
     - Accumulate output
   - Average comb outputs (divide by 8)
   - Process through allpass filters (4 stages)
   - Apply wet/dry mix

4. **Output**:
   - Apply dry/wet mix with gain control
   - Clamp output to [-10, 10]V range
   - Set output voltages

## 7. Important Functions

### `PsyReverbEngine::init(float sr)`
Initializes delay lines with calculated lengths based on sample rate.

### `PsyReverbEngine::processStereo(float leftIn, float rightIn, float &leftOut, float &rightOut)`
Main stereo reverb processing function:
- Processes 8 comb filters in parallel
- Applies damping via lowpass filtering
- Processes 4 allpass filters in series
- Mixes dry and wet signals

### `PsyReverbEngine::processMono(float in, float &outL, float &outR)`
Handles mono input by duplicating signal to both outputs.

### `PsyReverb::process(const ProcessArgs &args)`
VCV Rack processing function:
- Updates parameters via clock divider (32 sample rate)
- Reads inputs and handles mono/stereo routing
- Calls appropriate reverb processing function
- Updates metering values

## 8. Mathematical Implementation

### Delay Line Implementation
```cpp
void DelayLine::init(int len) {
    length = std::max(len, 1);
    buffer = new float[length]();
    idx = 0;
}

void DelayLine::write(float value) {
    buffer[idx] = value;
    idx = (idx + 1) % length;
}
```

### Lowpass Filter Implementation
```cpp
inline float lowpass(float in, float &state, float cutoff) {
    if (cutoff >= 19999.0f)
        return in;
    float coeff = 1.0f - std::exp(-2.0f * float(M_PI) * cutoff / sampleRate);
    state = state + coeff * (in - state);
    return state;
}
```

### Comb Filter Feedback Gain
$$g_{fb} = 0.5 + \text{roomsize} \times 0.48$$

Maximum gain: $0.5 + 1.0 \times 0.48 = 0.98$

## 9. Performance and Optimization

### CPU Usage Considerations
- Uses fixed-size delay lines (no dynamic allocation during processing)
- Processes only 1/32 of samples for parameter updates
- Minimal floating-point operations per sample
- Efficient circular buffer indexing

### Memory Allocation
- Delay line buffers allocated once in `init()`
- No heap allocations during audio processing
- Destructor properly cleans up all memory

### Buffers
- Fixed-size delay lines (8 comb + 4 allpass)
- Each delay line uses a circular buffer of appropriate length
- Buffer sizes calculated from sample rate and fixed delays

### Real-time Processing
- All processing done in real-time audio thread
- Parameter updates synchronized with audio rate
- No blocking operations or locks

### Thread Safety
- Single-threaded design (audio thread only)
- Meter values updated atomically (torn reads are cosmetic)

### Numerical Stability
- Feedback gain clamped to maximum 0.98
- Output clamped to [-10, 10]V range
- Lowpass filter uses stable one-pole implementation
- Delay line indices use modulo arithmetic

## 10. Dependencies

### Libraries and Headers
- `rack.hpp` - VCV Rack core library
- `<cmath>` - Standard math functions
- `<cstdio>` - Standard I/O (unused in current code)
- `<cstring>` - String operations (unused in current code)
- `<algorithm>` - Standard algorithms

### Classes and Types
- `rack::Module` - Base module class
- `rack::dsp::ClockDivider` - Sample rate divider for parameter updates
- `rack::widget::TransparentWidget` - UI widget base
- `rack::Param` - Parameter handling
- `rack::Input` - Input handling
- `rack::Output` - Output handling

## 11. Summary

The `PsyReverb` module implements a modified Freeverb-style reverb algorithm optimized for VCV Rack. It features:

- **Stereo processing** with automatic mono-to-stereo conversion
- **Four controllable parameters**: Wet, Dry, Room size, Damping  
- **CV control** with attenuverters for real-time modulation
- **Stable feedback system** preventing runaway gain
- **Visual display** showing reverb characteristics (decay curve, damping response)
- **Real-time metering** of input/output signals

The implementation uses 8 comb filters and 4 allpass filters arranged in a classic reverb topology, with careful attention to numerical stability and performance optimization for real-time audio processing.