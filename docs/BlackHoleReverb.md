<!--
AGMM AUTOMATIC DOCUMENTATION
Source: BlackHoleReverb.cpp
Generated: 2026-11-04 06:13
-->

# BlackHoleReverb

## 1. Overview

The `BlackHoleReverb` module is a stereo reverb effect for VCV Rack that simulates a black hole's acoustic environment using a combination of delay lines, feedback, and modulation. It provides a dark, deep, and atmospheric reverb sound with parameters for mix, feedback, damping, and modulation.

## 2. Architecture

### Main Classes
- `BlackHoleReverb`: The core DSP module inheriting from `rack::Module`
- `BlackHoleReverbWidget`: The UI widget for the module, inheriting from `rack::ModuleWidget`
- Anonymous namespace: Contains helper classes for drawing and panel layout

### Inheritance
- `BlackHoleReverb` inherits from `rack::Module`
- `BlackHoleReverbWidget` inherits from `rack::ModuleWidget`
- Drawing helpers inherit from `rack::TransparentWidget`

### Structures
- `CVRange`: Enumeration with two modes: `BIPOLAR_M5_5` and `UNIPOLAR_0_10`
- `BhCellDef`: Structure for defining control cell layout (name, param ID, attn ID, CV ID)
- `BhPanelBackground`: Panel drawing helper
- `BhDisplayWidget`: Scope display and level meter widget

### Relationships
- `BlackHoleReverb` contains all DSP state including delay buffers, filter stores, and modulation phase
- `BlackHoleReverbWidget` manages UI elements and connects to the module
- `BhDisplayWidget` displays real-time scope data and level meters
- `BhPanelBackground` handles panel drawing

### Module Organization
The module is organized into:
1. DSP processing in `process()` method
2. Parameter configuration in constructor
3. JSON serialization for state persistence
4. UI layout and drawing components

## 3. DSP Theory

### Mathematical Equations

The reverb uses a multi-tap delay line architecture with feedback and damping:

$$y[n] = \alpha x[n] + (1 - \alpha) y[n-1]$$

Where:
- $y[n]$ is the filtered output
- $x[n]$ is the input
- $\alpha$ is the filter coefficient

The delay lines implement a comb filter structure with feedback:

$$output = input + feedback \times delayed\_signal$$

Modulation is implemented as sinusoidal variation of delay times:

$$delay_{var} = base\_delay + modulation\_amount \times 240 \times \sin(2\pi f \times t)$$

### Algorithms
- Multi-tap delay line with 6 delay lines
- Comb filter feedback structure
- Low-pass filtering with damping coefficient
- Sinusoidal modulation of delay times
- Pre-delay buffer for initial signal processing
- Peak detection and scope display

### Filters
- First-order low-pass filters implemented using recursive averaging:
$$filter_{store}[n] = input \times (1 - damp^2) + filter_{store}[n-1] \times damp^2$$

### Delay Lines
- 6 delay lines with fixed base delays: [2741, 3347, 3823, 4421, 5101, 5897] samples
- Each delay line has a maximum length of 96000 samples
- Modulation varies delay by ±240 samples based on modulation parameter

### Feedback Systems
- Feedback is applied to each delay line with cross-feedback between left and right channels
- Feedback coefficient clamped between 0 and 0.98
- Cross-feedback factor of 0.4 for stereo widening effect

### Modulation
- Sinusoidal LFO modulating delay times at different frequencies
- Modulation frequency increases linearly with delay line index
- Modulation amount scaled by parameter value and CV input

### Random Processes
- No true random processes; uses deterministic sinusoidal modulation

## 4. Parameters and Controls

### Knobs
- **MIX**: 0.0 to 1.0, default 0.5 - Controls wet/dry mix ratio
- **FEEDBACK**: 0.0 to 0.98, default 0.5 - Controls feedback amount (clamped)
- **DAMP**: 0.0 to 1.0, default 0.7 - Controls damping (darkness) coefficient
- **MOD**: 0.0 to 1.0, default 0.5 - Controls modulation amount

### Switches
- None explicitly defined in code

### Buttons
- None explicitly defined in code

### CV Inputs
- **MIX CV**: Accepts voltage for mix parameter modulation
- **FEEDBACK CV**: Accepts voltage for feedback parameter modulation
- **DAMP CV**: Accepts voltage for damping parameter modulation
- **MOD CV**: Accepts voltage for modulation parameter modulation

### Gates
- None explicitly defined in code

### Enumerations
- `CVRange`: 
  - `BIPOLAR_M5_5` (-5V to +5V)
  - `UNIPOLAR_0_10` (0V to +10V)

### Parameter Ranges
- MIX: 0.0 to 1.0
- FEEDBACK: 0.0 to 0.98
- DAMP: 0.0 to 1.0
- MOD: 0.0 to 1.0

### Default Values
- MIX: 0.5
- FEEDBACK: 0.5
- DAMP: 0.7
- MOD: 0.5

## 5. Inputs and Outputs

### Inputs
- **IN_L**: Left input signal (voltage range: ±10V)
- **IN_R**: Right input signal (voltage range: ±10V)
- **MIX_CV**: Mix CV modulation input (voltage range: ±10V or 0-10V depending on mode)
- **FEEDBACK_CV**: Feedback CV modulation input (voltage range: ±10V or 0-10V depending on mode)
- **DAMP_CV**: Damping CV modulation input (voltage range: ±10V or 0-10V depending on mode)
- **MOD_CV**: Modulation CV modulation input (voltage range: ±10V or 0-10V depending on mode)

### Outputs
- **OUT_L**: Left output signal (voltage range: ±10V)
- **OUT_R**: Right output signal (voltage range: ±10V)

## 6. Signal Processing Pipeline

1. **Input Processing**: Read left and right inputs, normalize invalid values
2. **Parameter Processing**: Read knobs, CV inputs, and attenuators; scale CVs based on mode
3. **Pre-delay**: Apply 80ms pre-delay using circular buffer
4. **Delay Line Processing**:
   - Process 6 delay lines with different base delays
   - Apply sinusoidal modulation to each delay line
   - Implement comb filter feedback with damping
   - Cross-feedback between left and right channels
5. **Mixing**: Combine original input with processed signal based on mix parameter
6. **Clipping**: Apply tanh clipping to prevent overdrive
7. **Output**: Write processed signals to output jacks

## 7. Important Functions

### `process()`
Main DSP processing function that:
- Handles sample rate and time validation
- Reads input signals and parameters
- Applies CV scaling based on mode
- Implements pre-delay buffer
- Processes all 6 delay lines with modulation
- Combines wet/dry signals
- Applies output clipping
- Updates scope display data

### `scaleCV()`
Converts CV input voltage to normalized value:
- If `UNIPOLAR_0_10`: divide by 10.0
- If `BIPOLAR_M5_5`: clamp to [-1, 1] after dividing by 5.0

### `dataToJson()` / `dataFromJson()`
Handles module state persistence:
- Saves CV range mode to JSON
- Loads CV range mode from JSON

## 8. Mathematical Implementation

### Delay Line Implementation
```cpp
int readIndex = writeIndex - delay;
if (readIndex < 0)
    readIndex += MAX_DELAY;

float dl = delayL[i][readIndex];
filterStoreL[i] = dl * (1.f - dampSq) + filterStoreL[i] * dampSq;
float fbL = filterStoreL[i];
delayL[i][writeIndex] = revInL + (fbL + cross) * feedback;
```

### Modulation Implementation
```cpp
modPhase[i] += args.sampleTime * (0.08f + 0.04f * i);
if (modPhase[i] > 1.f)
    modPhase[i] -= 1.f;
float mod = std::sin(2.f * M_PI * modPhase[i]);
int delay = baseDelays[i] + (int)(mod * 240.f * modAmount);
```

### Damping Filter
```cpp
filterStoreL[i] = dl * (1.f - dampSq) + filterStoreL[i] * dampSq;
```

### Clipping
```cpp
outL = clipLevel * std::tanh(outL / clipLevel);
```

## 9. Performance and Optimization

### CPU Usage Considerations
- Uses fixed-size delay buffers (6 × 96000 samples)
- Implements efficient circular buffer indexing
- Limits modulation frequency to prevent excessive computation
- Uses simple mathematical operations for real-time processing

### Memory Allocation
- Pre-allocated static buffers:
  - `delayL[NUM_DELAYS][MAX_DELAY]`
  - `delayR[NUM_DELAYS][MAX_DELAY]`
  - `preDelayBuffer[PRE_DELAY_MAX]`
  - Scope display buffers (`scopeMin`, `scopeMax`)

### Buffers
- 6 delay lines with 96000 sample capacity each
- Pre-delay buffer with 44100 samples capacity
- Scope display buffer with 96 buckets

### Real-time Processing
- All processing done in `process()` method
- Uses fixed sample time for consistent timing
- Implements safe input handling with normalization

### Thread Safety
- No explicit thread safety mechanisms (assumes single-threaded Rack environment)
- All state variables are updated atomically within process loop

### Numerical Stability
- Input values clamped to prevent invalid operations
- Feedback coefficient clamped to 0.98 maximum
- Uses `std::isnormal()` for input validation
- Tanh clipping prevents signal overflow

## 10. Dependencies

### Libraries
- `rack.hpp`: VCV Rack core library
- `<cmath>`: Standard math functions
- `<cstdio>`: Standard I/O (unused in current implementation)
- `<cstring>`: String operations (unused in current implementation)
- `<algorithm>`: Standard algorithms
- `<memory>`: Smart pointers

### Classes and Headers
- `rack::Module`: Base module class
- `rack::ModuleWidget`: Base widget class
- `rack::TransparentWidget`: Base drawing widget
- `rack::Param`: Parameter handling
- `rack::Input`: Input signal handling
- `rack::Output`: Output signal handling
- `rack::json_t`: JSON serialization support
- `rack::Vec`, `rack::DrawArgs`, `rack::NVGcontext`: Drawing and geometry

## 11. Summary

The `BlackHoleReverb` is a stereo reverb module implementing a multi-tap delay line architecture with sinusoidal modulation. It features six delay lines with fixed base delays, comb filter feedback, damping filters, and cross-channel feedback for stereo widening. The module provides four main parameters (mix, feedback, damping, modulation) with CV inputs and attenuators, plus a CV range selection for compatibility with different voltage standards. The implementation uses efficient circular buffers, real-time processing, and tanh clipping for signal limiting, making it suitable for live performance in VCV Rack.