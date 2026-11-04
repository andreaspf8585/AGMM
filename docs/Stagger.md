<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Stagger.cpp
Generated: 2026-11-04 06:34
-->

# Stagger

## 1. Overview

The **Stagger** module is a stereo voltage-controlled oscillator (VCO) with chaotic pitch modulation capabilities. It generates four standard waveforms (sine, triangle, sawtooth, square) and provides independent stereo pitch drift controlled by a chaos parameter. The module supports polyphonic inputs and outputs, allowing multiple channels to be driven simultaneously.

Key features include:
- Four waveform selection options
- Independent left/right stereo pitch modulation with chaotic drift
- CV inputs for pitch, chaos depth, waveform, and pulse width
- Reset input for chaos drift
- Visual display showing current frequency, waveform, pulse width, and chaos history
- Attenuverter controls for all CV inputs
- CV range selection (±5V or ±10V)

## 2. Architecture

### Main Classes

- **`StFreqQ`**: Custom `ParamQuantity` class that implements logarithmic frequency mapping for the pitch knob with custom display formatting.
- **`Stagger`**: Main module class inheriting from `rack::engine::Module`.
- **`Voice`**: Inner struct representing a single voice/channel, containing phase and BLEP generators for each channel.

### Inheritance

The `StFreqQ` class inherits from `rack::engine::ParamQuantity`, providing custom parameter display and value mapping for the frequency knob.

### Structures

- **`Voice`**: Contains per-channel state including:
  - Phase tracking (`phase`, `lastPhase`)
  - BLEP generators for waveform discontinuities (`blepRising`, `blepFalling`, `blepSaw`)
  - Right channel variants (`phaseR`, `lastPhaseR`, `blepRisingR`, etc.)

### Relationships

- The `Stagger` module contains a vector of `Voice` objects, one per channel.
- Each `Voice` has independent phase and BLEP state for left and right channels.
- The `StFreqQ` class is used as the parameter quantity for the pitch knob.

### Module Organization

The module is organized into:
1. Parameter definitions (enum `ParamId`)
2. Input definitions (enum `InputId`)
3. Output definitions (enum `OutputId`)
4. Light definitions (enum `LightId`)
5. Internal state management
6. Audio processing in `process()` method
7. UI components in `StaggerWidget`

## 3. DSP Theory

### Mathematical Equations

The core frequency calculation uses the relationship:
$$ f = f_0 \cdot 2^{V} $$

Where:
- $ f $ is the output frequency
- $ f_0 = 261.6\,Hz $ (C4 note)
- $ V $ is the voltage in octaves

The pitch modulation is implemented using:
$$ f_{actual} = f_0 \cdot 2^{V_{base} + V_{CV}} $$

### Algorithms

#### Fast Exponential
A fast approximation of $ 2^x $ is used:
```cpp
auto fastExp2 = [](float x) {
    if (x == 0.0f) return 1.0f;
    union { float f; uint32_t i; } u;
    u.i = (uint32_t)((1 << 23) * (x + 126.94269504f));
    return u.f;
};
```

#### Phase Accumulation
For each waveform, the phase is incremented by:
$$ \Delta\phi = f \cdot \Delta t $$

Where $ \Delta t $ is the sample time.

### Filters

A first-order low-pass filter is used to smooth chaotic noise:
$$ y[n] = (1 - \alpha) \cdot y[n-1] + \alpha \cdot x[n] $$

With cutoff frequency:
$$ \alpha = 2\pi f_c \Delta t $$

### Oscillators

The module supports four waveform types with different generation methods:

1. **Sine**: Direct sine wave calculation
2. **Triangle**: Piecewise linear interpolation
3. **Sawtooth**: Linear ramp with discontinuity correction using BLEP
4. **Square**: Pulse wave with discontinuity correction using BLEP

### Delay Lines

No explicit delay lines are used, but the chaotic drift is implemented as a filtered noise process.

### Feedback Systems

The chaotic pitch modulation uses feedback from a filtered noise source to create time-varying pitch modulation.

### Modulation

All parameters can be modulated via CV inputs with attenuverter controls. The modulation is applied in the following way:
$$ V_{mod} = V_{base} + (V_{CV} \cdot A) $$

Where $ A $ is the attenuverter value.

### Random Processes

White noise is generated using `std::mt19937` with a uniform distribution over [-1, 1]. This noise is filtered to create low-frequency chaotic drift.

## 4. Parameters and Controls

### Knobs

- **PITCH_PARAM** (Frequency): 10 Hz - 6000 Hz, logarithmic scale
- **CHAOS_PARAM** (Chaos depth): 0% - 100%
- **WAVE_PARAM** (Waveform): 0-3 (Sine, Triangle, Saw, Square)
- **PW_PARAM** (Pulse width): 0% - 100% (Square wave only)

### Switches

- **CHAOS_MULT_PARAM**: Chaos multiplier (x1, x2, x3, x5, x10)
- **CV_RANGE_PARAM**: CV range selection (±5V or ±10V)

### Buttons

None

### CV Inputs

- **CV_INPUT**: 1V/octave pitch input
- **PITCH_CV_INPUT**: Pitch modulation CV
- **CHAOS_CV_INPUT**: Chaos depth modulation CV
- **WAVE_CV_INPUT**: Waveform selection CV
- **PW_CV_INPUT**: Pulse width modulation CV
- **CHAOS_RESET_INPUT**: Reset chaos drift (trigger)

### Gates

None

### Enumerations

- **Waveform types**: 0=Sine, 1=Triangle, 2=Saw, 3=Square
- **Chaos multipliers**: 0=x1, 1=x2, 2=x3, 3=x5, 4=x10

### Parameter Ranges and Default Values

| Parameter | Range | Default |
|-----------|-------|---------|
| PITCH_PARAM | 0.0 - 1.0 | FREQ_C4 (261.6 Hz) |
| CHAOS_PARAM | 0.0 - 1.0 | 0.0 |
| CHAOS_MULT_PARAM | 0.0 - 4.0 | 0.0 |
| WAVE_PARAM | 0.0 - 3.0 | 0.0 |
| PW_PARAM | 0.0 - 1.0 | 0.5 |
| PITCH_ATTEN_PARAM | -1.0 - 1.0 | 0.0 |
| CHAOS_ATTEN_PARAM | -1.0 - 1.0 | 0.0 |
| WAVE_ATTEN_PARAM | -1.0 - 1.0 | 0.0 |
| PW_ATTEN_PARAM | -1.0 - 1.0 | 0.0 |
| CV_RANGE_PARAM | 0.0 - 1.0 | 0.0 |

## 5. Inputs and Outputs

### Inputs

- **CV_INPUT**: 1V/octave pitch input (0V = C4)
- **PITCH_CV_INPUT**: Pitch modulation CV
- **CHAOS_CV_INPUT**: Chaos depth modulation CV
- **WAVE_CV_INPUT**: Waveform selection CV
- **PW_CV_INPUT**: Pulse width modulation CV
- **CHAOS_RESET_INPUT**: Reset chaos drift (trigger, >1.0V to reset)

### Outputs

- **VCO_L_OUTPUT**: Left channel output voltage (±5V)
- **VCO_R_OUTPUT**: Right channel output voltage (±5V)

## 6. Signal Processing Pipeline

1. **Initialization**:
   - Set up parameter configurations
   - Initialize random number generator
   - Clear voice vector

2. **Reset Handling**:
   - Check for trigger on chaos reset input
   - Reset chaos drift if triggered

3. **Noise Generation and Filtering**:
   - Generate white noise samples
   - Apply first-order low-pass filter to create chaotic drift

4. **Channel Processing Loop**:
   - For each channel:
     - Read CV inputs
     - Apply attenuverters
     - Calculate total pitch voltage
     - Apply chaos modulation
     - Calculate frequency using fast exponential
     - Generate waveform using appropriate method
     - Output voltage (multiplied by 5.0)

5. **Display Update**:
   - Update display data for UI
   - Update chaos LED brightness

6. **History Tracking**:
   - Store chaos drift values in circular buffer
   - Update peak-hold value for visual representation

## 7. Important Functions

### `StFreqQ::mapToFreq`
Converts normalized parameter value to frequency using logarithmic mapping:
$$ f = 10 \cdot 600^{v} $$

### `StFreqQ::mapFromFreq`
Converts frequency back to normalized parameter value:
$$ v = \frac{\log(f/10)}{\log(600)} $$

### `process()`
Main audio processing function that:
- Reads parameters and CV inputs
- Generates filtered noise for chaos drift
- Processes each channel through waveform generation
- Updates display data
- Controls LED brightness

### `genWave`
Lambda function that generates waveforms with appropriate discontinuity handling using BLEP generators.

## 8. Mathematical Implementation

### Fast Exponential Approximation

The fast exponential implementation uses bit manipulation:
```cpp
union { float f; uint32_t i; } u;
u.i = (uint32_t)((1 << 23) * (x + 126.94269504f));
return u.f;
```

This approximates $ 2^x $ with good accuracy for audio frequencies.

### Low-Pass Filter

First-order low-pass filter:
$$ y[n] = (1 - \alpha) \cdot y[n-1] + \alpha \cdot x[n] $$

Where:
$$ \alpha = 2\pi f_c \Delta t $$

With $ f_c = 1.5\,Hz $ and $ \Delta t $ being the sample time.

### BLEP Implementation

MinBLEP generators are used to correct discontinuities in sawtooth and square waveforms:
- `blepRising`: Corrects rising edges
- `blepFalling`: Corrects falling edges  
- `blepSaw`: Corrects sawtooth discontinuities

## 9. Performance and Optimization

### CPU Usage Considerations

The module uses optimized algorithms including:
- Fast exponential approximation
- Minimal floating-point operations per sample
- Efficient vector processing for polyphonic channels

### Memory Allocation

- Uses fixed-size arrays for history tracking (`driftHist`)
- Pre-allocated voice vector that resizes as needed
- No dynamic allocation during audio processing

### Buffers

- Circular buffer of size `ST_DRIFT_N` (128) for chaos history display
- Voice vector that grows with number of channels

### Real-time Processing

All operations are designed to be real-time safe:
- No blocking operations
- Fixed-size memory allocations
- Efficient algorithms

### Thread Safety

The module is designed for single-threaded audio processing. Display data is written by audio thread and read by UI thread, but no synchronization is implemented.

### Numerical Stability

- Uses `std::nearbyint` for rounding to avoid floating-point precision issues
- Clamps values to prevent out-of-bounds errors
- Uses `math::clamp` for all parameter ranges

## 10. Dependencies

### Libraries and Headers

- `rack.hpp`: VCV Rack core library
- `<cmath>`: Standard math functions
- `<cstdio>`, `<cstring>`: Standard C I/O and string operations
- `<cstdint>`: Fixed-width integer types
- `<vector>`, `<map>`, `<string>`, `<regex>`, `<algorithm>`, `<random>`: STL containers and algorithms
- `"dsp/minblep.hpp"`: MinBLEP DSP utilities

### Classes and Types

- `rack::engine::Module`: Base module class
- `rack::engine::ParamQuantity`: Parameter quantity base class
- `dsp::MinBlepGenerator<16, 16>`: BLEP generator class
- `std::mt19937`: Mersenne Twister random number generator
- `std::uniform_real_distribution<float>`: Uniform distribution for noise generation

## 11. Summary

The **Stagger** module is a stereo VCO with chaotic pitch modulation capabilities. It generates four waveform types (sine, triangle, sawtooth, square) with independent left/right channel processing and chaotic drift that creates time-varying pitch modulation. The module supports polyphonic inputs/outputs, CV modulation with attenuverters, and provides visual feedback through a display showing current parameters and chaos history. The implementation uses optimized DSP techniques including fast exponential approximation, first-order filtering for noise shaping, and MinBLEP correction for waveform discontinuities to ensure high-quality audio output while maintaining real-time performance.