<!--
AGMM AUTOMATIC DOCUMENTATION
Source: DarkHall.cpp
Generated: 2026-11-04 06:15
-->

# DarkHall

## 1. Overview

The `DarkHall` module is a stereo reverb effect designed for the VCV Rack modular synthesizer platform. It implements a reverberation algorithm based on comb and allpass filters, creating a spatial audio effect that simulates the acoustic properties of a room or hall.

Key features include:
- Stereo input/output processing
- Four controllable parameters: Mix, Room Size, Damping, and Modulation
- Real-time scope visualization of output signal
- Level meters for left/right channels
- Visual panel with control knobs and I/O jacks

## 2. Architecture

### Main Classes

The module is implemented as a single class `DarkHall` inheriting from `rack::Module`.

### Inheritance

```
DarkHall : Module
```

### Structures

The `DarkHall` class contains several arrays and variables organized for DSP processing:

- **Delay Buffers**: 
  - `combL[NUM_COMBS][MAX_DELAY]`, `combR[NUM_COMBS][MAX_DELAY]`: Comb filter delay lines for left/right channels
  - `allpassL[NUM_ALLPASS][MAX_DELAY]`, `allpassR[NUM_ALLPASS][MAX_DELAY]`: Allpass filter delay lines for left/right channels

- **Storage Variables**:
  - `combStoreL[NUM_COMBS]`, `combStoreR[NUM_COMBS]`: Feedback storage for comb filters
  - `apIdxL[NUM_ALLPASS]`, `apIdxR[NUM_ALLPASS]`: Index pointers for allpass filters
  - `modPhase[NUM_COMBS]`: Phase values for modulation

- **Scope Tracking**:
  - `scopeMin[DH_SCOPE_N]`, `scopeMax[DH_SCOPE_N]`: Min/max signal values for scope display
  - `scopeHead`, `scopeCount`: Scope buffer management
  - `scopeBMin`, `scopeBMax`: Current bucket min/max values
  - `peakL`, `peakR`: Peak level meters

### Relationships Between Classes

The module uses standard VCV Rack architecture:
- `DarkHallWidget` is a `ModuleWidget` that hosts the `DarkHall` module
- The widget creates UI elements (knobs, jacks, display) and connects them to the module's parameters and I/O
- All DSP processing occurs within the `process()` method of `DarkHall`

### Module Organization

The module follows standard VCV Rack conventions:
- Parameter configuration in constructor using `configParam()`
- Input/output configuration using `configInput()`/`configOutput()`
- DSP processing in `process()` method
- UI elements defined in anonymous namespace within the same file

## 3. DSP Theory

### Mathematical Equations

The reverb algorithm is based on a combination of comb and allpass filters:

**Comb Filter:**
$$
y[n] = x[n] + g \cdot x[n - D]
$$

Where:
- $x[n]$ is input signal
- $y[n]$ is output signal  
- $g$ is feedback gain
- $D$ is delay in samples

**Allpass Filter:**
$$
y[n] = -x[n] + x[n-D] + g \cdot y[n-D]
$$

Where:
- $x[n]$ is input signal
- $y[n]$ is output signal
- $g$ is feedback gain (typically 0.6)
- $D$ is delay in samples

### Algorithms

The implementation uses a multi-tap comb filter architecture with:
1. **8 comb filters** in parallel, each with different delays
2. **4 allpass filters** in series for diffusion
3. **Modulation** of comb filter delays using sine wave LFOs
4. **Damping** applied to comb filter outputs
5. **Feedback gain** controlled by room size parameter

### Filters

- **Comb Filters**: 8 parallel comb filters with different delay times
- **Allpass Filters**: 4 cascaded allpass filters for diffusion
- **Modulation Filters**: Sine wave LFOs modulating comb delays

### Oscillators

- **Modulation LFOs**: Sine wave oscillators with varying frequencies for each comb filter
- **Frequency Calculation**: 
  $$
  f_i = 0.05 + i \cdot 0.01 \quad \text{(Hz)}
  $$

### Delay Lines

- **Maximum Delay**: 96,000 samples (approximately 2 seconds at 48kHz)
- **Comb Delays**: Base delays from array `[1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617]` samples
- **Allpass Delays**: Not explicitly specified, but implemented as fixed delays in the buffers

### Feedback Systems

The feedback system includes:
1. **Comb Filter Feedback**: 
   $$
   \text{feedback} = 0.75 + \text{size} \cdot 0.2
   $$
2. **Damping Application**: 
   $$
   \text{output}_{\text{damped}} = \text{input} \cdot (1 - \text{damp}) + \text{stored} \cdot \text{damp}
   $$

### Modulation

- **Modulation Amount**: Controlled by `MOD_PARAM`
- **Modulation Depth**: 25 samples maximum deviation
- **Modulation Frequency**: Varies per comb filter with base frequency of 0.05Hz + i*0.01Hz

### Random Processes

The algorithm does not use true random processes, but rather deterministic modulation using sine wave LFOs.

## 4. Parameters and Controls

### Knobs

- **MIX**: 
  - Range: 0.0 to 1.0
  - Default: 0.4
  - Function: Mix between dry (input) and wet (processed) signals

- **SIZE**: 
  - Range: 0.0 to 1.0  
  - Default: 0.7
  - Function: Controls room size, affects delay times and feedback gain

- **DAMP**: 
  - Range: 0.0 to 1.0
  - Default: 0.6
  - Function: Controls damping (low-pass filtering) of comb filter outputs

- **MOD**: 
  - Range: 0.0 to 1.0
  - Default: 0.3
  - Function: Controls modulation amount for comb delays

### Switches and Buttons

None implemented.

### CV Inputs

None implemented.

### Gates

None implemented.

### Enumerations

```cpp
enum ParamId
{
    MIX_PARAM,
    SIZE_PARAM,
    DAMP_PARAM,
    MOD_PARAM,
    PARAMS_LEN
};

enum InputId
{
    IN_L,
    IN_R,
    INPUTS_LEN
};

enum OutputId
{
    OUT_L,
    OUT_R,
    OUTPUTS_LEN
};
```

### Parameter Ranges and Default Values

| Parameter | Range | Default |
|-----------|-------|---------|
| MIX_PARAM | 0.0 - 1.0 | 0.4 |
| SIZE_PARAM | 0.0 - 1.0 | 0.7 |
| DAMP_PARAM | 0.0 - 1.0 | 0.6 |
| MOD_PARAM | 0.0 - 1.0 | 0.3 |

## 5. Inputs and Outputs

### Inputs

- **IN_L** (Left Input):
  - Voltage Range: ±10V
  - Signal Behavior: Stereo input signal, attenuated by factor of 0.2 before processing

- **IN_R** (Right Input):
  - Voltage Range: ±10V  
  - Signal Behavior: Stereo input signal, attenuated by factor of 0.2 before processing

### Outputs

- **OUT_L** (Left Output):
  - Voltage Range: ±10V
  - Signal Behavior: Processed stereo output signal scaled by 5.0

- **OUT_R** (Right Output):
  - Voltage Range: ±10V
  - Signal Behavior: Processed stereo output signal scaled by 5.0

## 6. Signal Processing Pipeline

1. **Input Processing**: 
   - Read left/right inputs and apply 0.2 attenuation factor
   - Retrieve parameter values (mix, size, damp, mod)

2. **Comb Filter Processing**:
   - Update modulation phases for each comb filter
   - Calculate variable delays with modulation applied
   - Apply delay-based feedback with damping
   - Accumulate comb filter outputs

3. **Allpass Filtering**:
   - Apply 4 cascaded allpass filters to combined comb output
   - Scale output by 0.35 factor

4. **Mixing**:
   - Apply sine/cosine mixing based on mix parameter
   - Apply tanh distortion with 1.2 scaling factor

5. **Output Processing**:
   - Clamp finite values to prevent NaN/inf
   - Scale output by 5.0 for final voltage level
   - Update scope display data and peak meters

## 7. Important Functions

### `allpass()` Function

Implements a single allpass filter stage:
- Takes input signal, delay buffer, index reference, and gain
- Updates delay buffer with feedback
- Returns filtered output
- Handles circular buffer indexing

### `process()` Function

Main DSP processing routine:
- Reads inputs and parameters
- Processes comb filters in loop (8 filters)
- Applies modulation to delays
- Implements damping filtering
- Applies allpass filters (4 stages)
- Mixes dry/wet signals
- Updates scope display data
- Sets output voltages

### Constructor

Initializes module configuration:
- Configures parameters with default values and ranges
- Configures inputs/outputs
- Initializes delay buffers to zero

## 8. Mathematical Implementation

### Core Equations

**Feedback Gain Calculation:**
$$
\text{feedback} = 0.75 + \text{size} \cdot 0.2
$$

**Damping Application:**
$$
\text{output}_{\text{damped}} = \text{input} \cdot (1 - \text{damp}) + \text{stored} \cdot \text{damp}
$$

**Modulation Delay Calculation:**
$$
\text{delay} = \text{baseDelay}_i + \left\lfloor \sin(2\pi \cdot \text{modPhase}_i) \cdot 25 \cdot \text{modAmt} \right\rfloor
$$

**Final Delay Scaling:**
$$
\text{delay}_{\text{final}} = \text{delay} \cdot (0.5 + \text{size})
$$

### Numerical Methods

- **Circular Buffer Indexing**: Uses modulo arithmetic for delay line indexing
- **Clamping**: Applied to prevent signal overflow in feedback paths
- **Tanh Distortion**: Used for output limiting and soft clipping
- **Phase Accumulation**: Linear phase increment for modulation LFOs

## 9. Performance and Optimization

### CPU Usage Considerations

- **Processing Complexity**: O(N) where N = number of comb filters (8)
- **Memory Access**: Sequential access to delay buffers with circular indexing
- **Mathematical Operations**: Basic arithmetic, trigonometric functions, clamping

### Memory Allocation

- **Static Buffers**: All delay lines allocated statically at compile time
- **Buffer Size**: 96,000 samples maximum per delay line (180KB per channel)
- **Total Memory**: ~2.3MB for all buffers (8 comb + 4 allpass filters)

### Buffers

- **Delay Lines**: Fixed-size circular buffers with maximum size of 96,000 samples
- **Scope Buffer**: 96-element circular buffer for display data
- **Index Tracking**: Integer indices for circular buffer management

### Real-time Processing

- **Sample-based Processing**: Each call processes one sample
- **No Dynamic Allocation**: All memory allocated at initialization
- **Efficient Indexing**: Modular arithmetic for circular buffer access

### Thread Safety

- **Single-threaded**: Designed for real-time audio processing in Rack's main thread
- **No External Synchronization**: Assumes single-threaded access to module state

### Numerical Stability

- **Clamping**: Applied to prevent overflow in feedback paths
- **Finite Value Checks**: NaN/inf checking on output signals
- **Limited Range Operations**: All parameters constrained to [0,1] range

## 10. Dependencies

### Libraries and Headers

- `rack.hpp`: VCV Rack core library
- `<cmath>`: Standard math functions (sin, pow, etc.)
- `<cstdio>`: Standard I/O functions
- `<cstring>`: String/memory operations
- `<algorithm>`: Standard algorithms (min/max)
- `<memory>`: Smart pointer support

### External Classes

- `rack::Module`: Base module class
- `rack::ModuleWidget`: Widget base class for UI
- `rack::ParamQuantity`: Parameter handling
- `rack::Input`: Input port handling
- `rack::Output`: Output port handling
- `rack::TransparentWidget`: UI widget base
- `rack::NVGcontext`: NanoVG graphics context

## 11. Summary

The DarkHall module implements a stereo reverb effect using comb and allpass filter techniques. It features:

- **8 parallel comb filters** with variable delays and modulation
- **4 cascaded allpass filters** for diffusion
- **Four controllable parameters**: Mix, Size, Damping, Modulation
- **Real-time scope visualization** of output signal
- **Peak level meters** for left/right channels
- **Stereo input/output processing**
- **Efficient circular buffer implementation**
- **Tanh distortion for output limiting**

The algorithm produces a rich, spatial reverb effect suitable for ambient and atmospheric sound design. The implementation is optimized for real-time performance with minimal memory allocation and efficient DSP operations.