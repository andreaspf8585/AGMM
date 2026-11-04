<!--
AGMM AUTOMATIC DOCUMENTATION
Source: InstantDrone.cpp
Generated: 2026-11-04 06:21
-->

# InstantDrone

## 1. Overview

The `InstantDrone` module is a multi-oscillator drone generator designed for VCV Rack. It produces three distinct oscillator voices that can be modulated and processed through various effects including low-pass filtering, envelope control, and a dense reverb system. The module features:

- Three independent oscillators with sine, triangle, and sawtooth waveforms
- Individual pitch control via knob and CV input (V/Oct)
- Random drift modulation for each oscillator
- Per-oscillator mixing and filter cutoff controls
- Reverb send levels with dedicated wet/dry mix
- Multi-tap delay-based reverb system
- Envelope bypass switches for each oscillator
- Stereo output with adjustable gain
- Visual display showing signal scope and level meters

## 2. Architecture

### Main Classes

- **`InstantDrone`**: The core module class inheriting from `rack::Module`
- **`DroneDelay`**: A multi-tap delay line structure for reverb processing
- **`InstantDroneWidget`**: UI widget class inheriting from `rack::ModuleWidget`
- **Anonymous namespace functions**: UI drawing helpers and panel components

### Inheritance

- `InstantDrone` inherits from `rack::Module`
- `InstantDroneWidget` inherits from `rack::ModuleWidget`

### Structures

- **`DroneDelay`**:
  - Circular buffer of size 44100 floats
  - Write position tracking
  - `process()` method for delay line operation

### Relationships

- `InstantDrone` contains:
  - Three oscillator state variables (`phases`, `driftValues`, `driftTargets`)
  - Three lowpass filter state variables (`lowpass1`, `lowpass2`, `lowpass3`)
  - Four `DroneDelay` instances for left and right channels
  - Scope display data arrays for visualization

### Module Organization

- Parameters defined in `enum ParamId`
- Inputs defined in `enum InputId`
- Outputs defined in `enum OutputId`
- All UI components are created in the widget constructor (`InstantDroneWidget`)
- DSP processing occurs in `process()` method
- UI rendering is handled by various widget classes within anonymous namespace

## 3. DSP Theory

### Mathematical Equations

#### Oscillator Generation:
$$ \text{osc}(t) = \text{waveform}(\phi(t)) $$

Where $\phi(t)$ is the phase, updated as:
$$ \phi(t + dt) = \phi(t) + f(t) \cdot dt $$

#### Pitch Calculation:
$$ f = 261.626 \cdot 2^{p} $$

Where $p$ is pitch in octaves.

#### Lowpass Filter:
$$ y[n] = y[n-1] + f \cdot (x[n] - y[n-1]) $$

#### Random Walk Drift:
$$ \text{drift}[n] = \text{drift}[n-1] + (\text{target} - \text{drift}[n-1]) \cdot s $$

Where $s$ is the slew rate.

### Algorithms

- **Multi-tap delay**: Implements a circular buffer with multiple read positions
- **Lowpass filtering**: First-order IIR filter (leaky integrator)
- **Random drift**: Continuous random walk with periodic target updates
- **Envelope processing**: Linear interpolation between 0 and 1 based on CV input
- **Stereo reverb**: Multi-tap delay with different tap lengths for left/right channels

### Filters

- First-order lowpass IIR filter implemented as:
$$ y[n] = y[n-1] + f \cdot (x[n] - y[n-1]) $$

Where $f$ is the filter coefficient.

### Oscillators

Three waveform types supported:
1. Sine: $\sin(2\pi \phi)$
2. Triangle: $1 - 4|\phi - 0.5|$
3. Sawtooth: $2\phi - 1$

### Delay Lines

- Circular buffer of size 44100 samples (assuming 44.1kHz sample rate)
- Four delay taps per channel with different lengths
- Feedback coefficient of 0.985

### Feedback Systems

- Random walk drift uses feedback to maintain continuous modulation
- Reverb system uses feedback in delay lines

### Modulation

- V/Oct input: 1V/octave scaling
- CV inputs: 0-10V scaled to 0-1 multiplier
- Knob inputs: direct parameter control
- Envelope input: 0-10V scaled to 0-1 gain multiplier

### Random Processes

- Continuous random walk drift with periodic target updates (2‰ rate)
- Uniform distribution for target generation

## 4. Parameters and Controls

### Knobs

| Parameter | Default | Range | Units |
|-----------|---------|-------|-------|
| `FREQ1_PARAM` | 0.0 | -3.0 to 3.0 | oct |
| `FREQ2_PARAM` | 0.0 | -3.0 to 3.0 | oct |
| `FREQ3_PARAM` | 0.0 | -3.0 to 3.0 | oct |
| `MIX1_PARAM` | 0.5 | 0.0 to 1.0 | % |
| `MIX2_PARAM` | 0.5 | 0.0 to 1.0 | % |
| `MIX3_PARAM` | 0.5 | 0.0 to 1.0 | % |
| `DRIFT1_PARAM` | 0.2 | 0.0 to 1.0 | % |
| `DRIFT2_PARAM` | 0.2 | 0.0 to 1.0 | % |
| `DRIFT3_PARAM` | 0.2 | 0.0 to 1.0 | % |
| `SEND1_PARAM` | 0.5 | 0.0 to 1.0 | % |
| `SEND2_PARAM` | 0.5 | 0.0 to 1.0 | % |
| `SEND3_PARAM` | 0.5 | 0.0 to 1.0 | % |
| `FILT1_PARAM` | 0.6 | 0.0 to 1.0 | % |
| `FILT2_PARAM` | 0.6 | 0.0 to 1.0 | % |
| `FILT3_PARAM` | 0.6 | 0.0 to 1.0 | % |
| `REVERB_WET_PARAM` | 0.7 | 0.0 to 1.0 | % |
| `OUTPUT_GAIN_PARAM` | 0.0 | -10.0 to 10.0 | dB |

### Switches

| Parameter | Default | States |
|-----------|---------|--------|
| `WAVE1_PARAM` | 0 (Sine) | 0=Sine, 1=Triangle, 2=Saw |
| `WAVE2_PARAM` | 0 (Sine) | 0=Sine, 1=Triangle, 2=Saw |
| `WAVE3_PARAM` | 0 (Sine) | 0=Sine, 1=Triangle, 2=Saw |
| `ENV1_SWITCH` | 0 (Off) | 0=Bypassed, 1=Active |
| `ENV2_SWITCH` | 0 (Off) | 0=Bypassed, 1=Active |
| `ENV3_SWITCH` | 0 (Off) | 0=Bypassed, 1=Active |

### CV Inputs

| Input | Range | Function |
|-------|-------|----------|
| `VOCT1_INPUT` | -10V to +10V | Pitch modulation |
| `VOCT2_INPUT` | -10V to +10V | Pitch modulation |
| `VOCT3_INPUT` | -10V to +10V | Pitch modulation |
| `DRIFT1_CV` | -5V to +5V | Drift amount modulation |
| `DRIFT2_CV` | -5V to +5V | Drift amount modulation |
| `DRIFT3_CV` | -5V to +5V | Drift amount modulation |
| `MIX1_CV` | -5V to +5V | Mix level modulation |
| `MIX2_CV` | -5V to +5V | Mix level modulation |
| `MIX3_CV` | -5V to +5V | Mix level modulation |
| `FILT1_CV` | -5V to +5V | Filter cutoff modulation |
| `FILT2_CV` | -5V to +5V | Filter cutoff modulation |
| `FILT3_CV` | -5V to +5V | Filter cutoff modulation |
| `SEND1_CV` | -5V to +5V | Reverb send modulation |
| `SEND2_CV` | -5V to +5V | Reverb send modulation |
| `SEND3_CV` | -5V to +5V | Reverb send modulation |
| `ENV1_INPUT` | 0V to +10V | Envelope signal |
| `ENV2_INPUT` | 0V to +10V | Envelope signal |
| `ENV3_INPUT` | 0V to +10V | Envelope signal |

### Enumerations

- `ParamId`: All parameters defined in enum
- `InputId`: All inputs defined in enum
- `OutputId`: All outputs defined in enum

## 5. Inputs and Outputs

### Inputs

| Input | Voltage Range | Signal Behavior |
|-------|---------------|-----------------|
| `VOCT1_INPUT` | -10V to +10V | Pitch modulation (1V/octave) |
| `VOCT2_INPUT` | -10V to +10V | Pitch modulation (1V/octave) |
| `VOCT3_INPUT` | -10V to +10V | Pitch modulation (1V/octave) |
| `DRIFT1_CV` | -5V to +5V | Drift amount modulation (0-1 scale) |
| `DRIFT2_CV` | -5V to +5V | Drift amount modulation (0-1 scale) |
| `DRIFT3_CV` | -5V to +5V | Drift amount modulation (0-1 scale) |
| `MIX1_CV` | -5V to +5V | Mix level modulation (0-1 scale) |
| `MIX2_CV` | -5V to +5V | Mix level modulation (0-1 scale) |
| `MIX3_CV` | -5V to +5V | Mix level modulation (0-1 scale) |
| `FILT1_CV` | -5V to +5V | Filter cutoff modulation (0.01-0.99 scale) |
| `FILT2_CV` | -5V to +5V | Filter cutoff modulation (0.01-0.99 scale) |
| `FILT3_CV` | -5V to +5V | Filter cutoff modulation (0.01-0.99 scale) |
| `SEND1_CV` | -5V to +5V | Reverb send modulation (0-1 scale) |
| `SEND2_CV` | -5V to +5V | Reverb send modulation (0-1 scale) |
| `SEND3_CV` | -5V to +5V | Reverb send modulation (0-1 scale) |
| `ENV1_INPUT` | 0V to +10V | Envelope signal (0-1 scale) |
| `ENV2_INPUT` | 0V to +10V | Envelope signal (0-1 scale) |
| `ENV3_INPUT` | 0V to +10V | Envelope signal (0-1 scale) |

### Outputs

| Output | Voltage Range | Signal Behavior |
|--------|---------------|-----------------|
| `LEFT_OUTPUT` | -10V to +10V | Left stereo channel output |
| `RIGHT_OUTPUT` | -10V to +10V | Right stereo channel output |

## 6. Signal Processing Pipeline

1. **Oscillator Generation**:
   - Each oscillator generates waveform based on phase
   - Phase is updated using frequency and sample time
   - Waveform type selected from switch parameter

2. **Pitch Modulation**:
   - Base pitch from knob + V/Oct input
   - Drift modulation added (random walk with CV control)

3. **Envelope Processing**:
   - Envelope bypass switch determines if envelope is applied
   - If active, envelope input scaled to 0-1 and applied as gain multiplier

4. **Mixing and Filtering**:
   - Mix level from knob + CV input
   - Lowpass filter applied with cutoff from knob + CV input

5. **Reverb Processing**:
   - Dry signal sum from all oscillators
   - Reverb send levels from knob + CV input
   - Multi-tap delay system with different tap lengths for left/right channels

6. **Final Output**:
   - Wet/dry mix applied to reverb output
   - Overall gain applied from master gain knob
   - Stereo outputs generated

## 7. Important Functions

### `process()`
Main DSP processing function that:
- Updates oscillator phases and drift values
- Calculates pitch with modulation
- Generates waveforms
- Applies envelope control
- Processes lowpass filtering
- Implements multi-tap reverb system
- Computes final stereo output

### `getWaveform()`
Returns waveform value for given phase and type:
- Sine: $\sin(2\pi \phi)$
- Triangle: $1 - 4|\phi - 0.5|$
- Sawtooth: $2\phi - 1$

### `DroneDelay::process()`
Performs delay line operation:
- Reads from buffer at calculated position
- Writes input + feedback output to write position
- Updates circular buffer index

## 8. Mathematical Implementation

### Oscillator Generation
$$ \text{osc}(t) = \begin{cases}
\sin(2\pi \phi) & \text{if type = 0} \\
1 - 4|\phi - 0.5| & \text{if type = 1} \\
2\phi - 1 & \text{if type = 2}
\end{cases} $$

### Pitch Calculation
$$ f = 261.626 \cdot 2^{p} $$
Where $p = \text{knob} + \text{V/Oct input}$

### Phase Update
$$ \phi(t + dt) = \phi(t) + f \cdot dt $$

### Lowpass Filter
$$ y[n] = y[n-1] + f \cdot (x[n] - y[n-1]) $$
Where $f = \text{cutoff} \cdot 0.15$

### Random Walk Drift
$$ \text{drift}[n] = \text{drift}[n-1] + (\text{target} - \text{drift}[n-1]) \cdot 0.01 $$

### Envelope Application
$$ \text{gain} = \text{raw\_osc} \cdot 5.0 \cdot \text{base\_gain} \cdot \text{envelope} $$

## 9. Performance and Optimization

### CPU Usage Considerations

- Three oscillators with waveform generation
- Four delay taps per channel (8 total)
- Lowpass filters applied to each oscillator
- Random number generation for drift
- Real-time processing at 44.1kHz sample rate

### Memory Allocation

- Static buffers:
  - `DroneDelay` buffer: 44100 floats (176KB)
  - Scope display data: 55 * 2 = 110 floats (440 bytes)
  - Module state variables: ~30 floats
- No dynamic allocation in DSP path

### Buffers

- Circular delay buffers of 44100 samples each
- Scope visualization buffer of 55 elements
- All buffers are static and preallocated

### Real-time Processing

- All processing done within `process()` method
- No blocking operations or memory allocation
- Sample-based processing with fixed time steps
- Uses VCV Rack's sample time for timing

### Thread Safety

- Module state accessed by audio thread only
- Scope data shared between audio and UI threads (torn read acceptable)
- No synchronization primitives needed for core DSP

### Numerical Stability

- First-order IIR filters are stable with proper coefficient bounds
- Clamp operations prevent out-of-range values
- Circular buffer indexing uses modulo arithmetic
- Proper handling of floating-point operations

## 10. Dependencies

### Libraries and Headers

- `rack.hpp` - VCV Rack core library
- `cmath` - Standard math functions
- `cstdio` - C standard I/O
- `cstring` - C string operations
- `algorithm` - STL algorithms

### Classes and Structures

- `rack::Module` - Base module class
- `rack::ModuleWidget` - Base widget class
- `DroneDelay` - Custom delay line structure
- `NVGcontext` - NanoVG rendering context (UI only)
- `Vec` - Vector class for positioning
- `DrawArgs` - Drawing arguments

### VCV Rack Components

- `createParamCentered`, `createInputCentered`, `createOutputCentered` - UI component creation
- `RackGridWidth`, `RackGridHeight` - Panel sizing constants
- `nvgRGB`, `nvgRGBA`, `nvgLinearGradient` - NanoVG drawing functions

## 11. Summary

The `InstantDrone` module is a sophisticated multi-oscillator drone generator with real-time modulation capabilities and a dense reverb system. It implements three independent oscillators with sine, triangle, and sawtooth waveforms, each with individual pitch, drift, mix, filter, and send controls. The module features:

- Continuous random walk drift for natural modulation
- Per-oscillator lowpass filtering
- Multi-tap delay-based reverb system with different tap lengths for stereo
- Envelope bypass switches for each oscillator
- Comprehensive CV control for all parameters
- Stereo output with master gain control
- Visual display showing signal scope and level meters

The implementation uses efficient DSP algorithms including first-order IIR filters, circular buffers for delay lines, and proper numerical handling to ensure real-time performance on VCV Rack. The module is designed for creating ambient drone textures with rich harmonic content and spatial effects.