<!--
AGMM AUTOMATIC DOCUMENTATION
Source: InfinityRev.cpp
Generated: 2026-11-04 06:20
-->

# InfinityRev

## 1. Overview

InfinityRev is a high-quality stereo reverb module for VCV Rack that implements an 8-line Feedback Delay Network (FDN) with advanced features including:

- **8-line FDN reverb** with Hadamard feedback matrix
- **Modulated delays** for rich, evolving textures
- **RT60-based decay control** for precise reverb time adjustment
- **Frequency damping** with cutoff control
- **Pre-delay and early reflections** for spatial definition
- **Diffusion control** to adjust the "smearing" effect
- **Stereo width control** for spatial expansion
- **Freeze function** to hold reverb in infinite decay
- **CV-controlled parameters** for dynamic modulation
- **Real-time visual display** showing FDN network, decay curve and damping response

The module is designed as a professional-grade reverb with a focus on musicality and control over the reverb characteristics.

## 2. Architecture

### Main Classes

- `IRDelayLine` - Circular delay line with linear and cubic interpolation
- `IRAllpass` - Schroeder allpass filter for diffusion
- `InfinityRev` - Main module class implementing the reverb algorithm
- `IRDecayQ`, `IRDampQ` - Custom parameter quantity classes for display formatting
- `IRPanel`, `IRDisplay` - UI components for panel and visual display

### Inheritance

- `InfinityRev` inherits from `rack::engine::Module`
- `IRDecayQ`, `IRDampQ` inherit from `rack::engine::ParamQuantity`
- UI classes inherit from `TransparentWidget` and `ModuleWidget`

### Structures

- `IRDelayLine`: Circular buffer with read/write pointers, interpolation methods
- `IRAllpass`: Delay line with feedback for allpass filtering
- `InfinityRev`: Main module with parameter/state management

### Relationships

- `InfinityRev` contains:
  - Multiple `IRDelayLine` instances (pre-delay, early reflections, FDN lines)
  - Multiple `IRAllpass` instances (diffusion)
  - Parameter and state variables for control
  - UI display components
- `IRDelayLine` and `IRAllpass` are used as building blocks within `InfinityRev`
- Custom parameter quantity classes provide specialized display formatting

### Module Organization

The module is organized into:
1. **Core DSP**: Delay lines, allpass filters, FDN processing
2. **Control Logic**: Parameter updates, CV handling, freeze logic
3. **Audio Processing**: Main `process()` method with signal flow
4. **UI Components**: Panel drawing, display visualization, widget layout

## 3. DSP Theory

### Mathematical Equations

**Feedback Delay Network (FDN)**
$$y_i[n] = \sum_{j=0}^{N-1} H_{ij} \cdot x_j[n-D_j] \cdot g_j$$

Where:
- $H_{ij}$ is the Hadamard feedback matrix
- $x_j$ is input to line $j$
- $D_j$ is delay of line $j$
- $g_j$ is gain of line $j$

**Schroeder Allpass Filter**
$$y[n] = x[n] - g \cdot d[n]$$
$$d[n] = y[n] + g \cdot x[n-1]$$

Where:
- $g$ is the feedback coefficient (typically 0.6)
- $x[n]$ is input sample
- $d[n]$ is delay buffer value

**Hadamard Matrix**
The 8×8 Hadamard matrix used for feedback is constructed using the recursive formula:
$$H_{2n} = \begin{bmatrix} H_n & H_n \\ H_n & -H_n \end{bmatrix}$$

**Modulated Delay**
$$D(t) = D_0 + \Delta D \cdot \sin(2\pi f t)$$

Where:
- $D_0$ is base delay
- $\Delta D$ is modulation depth
- $f$ is LFO frequency

### Algorithms

1. **Pre-delay**: Linear interpolation delay line
2. **Early reflections**: Fixed tap delays with amplitude coefficients
3. **Diffusion**: 4-stage Schroeder allpass filters (left/right channels)
4. **FDN**: 8-line network with Hadamard feedback matrix
5. **Modulation**: Sinusoidal LFO modulation of delay times
6. **Damping**: One-pole low-pass filter applied to each FDN line
7. **Mixing**: Equal-power crossfade between dry and wet signals

### Filters

- **One-pole low-pass filters** for damping (implemented as IIR)
- **DC blocking filters** using leaky integrators
- **Soft limiting** for output clamping

### Oscillators

- **Sinusoidal LFOs** for delay modulation (8 different frequencies)
- **Phase accumulation** for LFO generation

### Delay Lines

- **Circular buffers** with wraparound indexing
- **Linear interpolation** for fractional delays
- **Cubic (Hermite) interpolation** for higher quality
- **Fixed-size buffers** with precomputed sizes

### Feedback Systems

- **Hadamard matrix feedback** for decorrelation between lines
- **Freeze mode** that sets gain to 1.0 for infinite decay
- **Modulated delays** that vary delay times over time

### Modulation

- **LFO modulation** of delay times (8 different frequencies)
- **CV modulation** of parameters (size, decay, mix, freeze)
- **Parameter smoothing** using exponential filters

### Random Processes

- **No explicit random processes** in the algorithm
- **Hadamard matrix** provides pseudo-random decorrelation

## 4. Parameters and Controls

### Knobs

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| PREDELAY | 0-250 ms | 20 ms | Pre-delay time |
| SIZE | 0.5-2.0x | 1.0x | Reverb size multiplier |
| DECAY | 0-1.0 | 0.65 | RT60 decay time (0.2s to 60s) |
| DAMP | 0-1.0 | 0.4 | Damping cutoff frequency (20kHz to 1kHz) |
| DIFFUSION | 0-1.0 | 0.75 | Diffusion level (0.1 to 0.8) |
| MOD | 0-1.0 | 0.3 | Modulation depth |
| ER | 0-1.0 | 0.4 | Early reflections level |
| WIDTH | 0-1.5 | 1.0 | Stereo width |
| MIX | 0-1.0 | 0.35 | Dry/wet mix |

### Switches

| Parameter | States | Default | Description |
|-----------|--------|---------|-------------|
| FREEZE | Off/On | Off | Freeze reverb in infinite decay |

### CV Inputs

| Input | Voltage Range | Description |
|-------|---------------|-------------|
| DECAY CV | -10V to +10V | Decay time modulation (10V = full range) |
| SIZE CV | -10V to +10V | Size modulation (+10V adds 1.5x) |
| MIX CV | -10V to +10V | Mix level modulation (10V = 100%) |
| FREEZE GATE | 0V to +10V | Freeze gate input |

### Gates

| Input | Description |
|-------|-------------|
| FREEZE GATE | Gate input to freeze reverb |

### Enumerations

- `ParamId`: Parameter enumeration
- `InputId`: Input enumeration  
- `OutputId`: Output enumeration
- `LightId`: Light enumeration

## 5. Inputs and Outputs

### Inputs

| Name | Type | Voltage Range | Description |
|------|------|---------------|-------------|
| INPUT_L | Audio | ±10V | Left input (mono if right input is empty) |
| INPUT_R | Audio | ±10V | Right input |
| DECAY_CV_INPUT | CV | -10V to +10V | Decay time modulation |
| SIZE_CV_INPUT | CV | -10V to +10V | Size modulation |
| MIX_CV_INPUT | CV | -10V to +10V | Mix level modulation |
| FREEZE_GATE_INPUT | Gate | 0V to +10V | Freeze gate input |

### Outputs

| Name | Type | Voltage Range | Description |
|------|------|---------------|-------------|
| OUTPUT_L | Audio | ±10V | Left output |
| OUTPUT_R | Audio | ±10V | Right output |

## 6. Signal Processing Pipeline

1. **Input Processing**:
   - Read input samples
   - Handle mono-to-stereo conversion
   - Apply DC blocking and limiting

2. **Pre-delay + Early Reflections**:
   - Pre-delay line (linear interpolation)
   - Early reflection taps (6 fixed delays)
   - Write to early reflection delay line

3. **Input Diffusion**:
   - 4-stage Schroeder allpass filters (left/right channels)
   - Apply diffusion gain

4. **FDN Processing**:
   - 8-line feedback delay network
   - Modulated delay times (sinusoidal LFOs)
   - One-pole damping filter on each line
   - Hadamard feedback matrix (fast Walsh-Hadamard transform)

5. **Output Processing**:
   - Output tap signs (orthogonal Hadamard rows)
   - Stereo width control
   - Equal-power mixing
   - DC blocking and soft limiting

6. **Control Updates**:
   - Parameter smoothing
   - CV processing
   - Freeze logic

## 7. Important Functions

### `InfinityRev::process()`
Main audio processing function that implements the complete reverb algorithm.

### `IRDelayLine::readLinear()` / `readCubic()`
Fractional delay interpolation methods with linear and cubic (Hermite) interpolation.

### `IRAllpass::process()`
Schroeder allpass filter implementation.

### `InfinityRev::updateControl()`
Control-rate update function that processes parameters and CV inputs.

### `InfinityRev::prepare()`
Initialization function called on sample rate changes or reset.

### `InfinityRev::softLimit()`
Soft limiting function to prevent clipping with asymptotic behavior.

### `InfinityRev::onReset()`
Reset handler that calls prepare().

## 8. Mathematical Implementation

### Delay Line Interpolation

**Linear interpolation:**
$$y = x_1 + f(x_2 - x_1)$$

**Cubic (Hermite) interpolation:**
$$y = x_0 + \frac{f}{2}[(x_1 - x_{-1}) + (x_1 - x_0)] + f^2\left[\frac{3(x_0 - x_1)}{2} + \frac{x_{-1} - x_2}{2}\right] + f^3\left[\frac{x_2 - x_{-1}}{2} + \frac{x_1 - x_0}{2}\right]$$

### One-pole Low-pass Filter
$$y[n] = k \cdot x[n] + (1-k) \cdot y[n-1]$$

Where $k = 1 - e^{-2\pi f_c / f_s}$

### Fast Walsh-Hadamard Transform
8×8 Hadamard matrix computed recursively:
$$H_{2n} = \begin{bmatrix} H_n & H_n \\ H_n & -H_n \end{bmatrix}$$

### Exponential Smoothing
$$y[n] = (1-\alpha) \cdot y[n-1] + \alpha \cdot x[n]$$

Where $\alpha$ is the smoothing coefficient.

## 9. UI Components

### Panel Visualization (`IRPanel`)
- Black panel with silver screws
- Parameter knob positions
- Input/output port locations

### Display Visualization (`IRDisplay`)
- **FDN Network**: 8 delay lines arranged in a ring, connected by Hadamard matrix
- **Decay Curve**: RT60 decay envelope (0-8 seconds)
- **Damping Response**: One-pole filter response (20Hz to 20kHz)
- **Meters**: Input, left output, right output level meters
- **Text Labels**: Size, RT60, Mix, Pre-delay, ER, Width, Damping

### Parameter Smoothing
- Exponential filters for smooth parameter transitions
- CV modulation with proper scaling

## 10. Technical Details

### Memory Usage
- 8 FDN delay lines with precomputed sizes
- 4 allpass filter stages per channel
- Pre-delay and early reflection buffers

### Performance Considerations
- Fixed-size buffers for predictable memory usage
- Efficient interpolation methods (linear/cubic)
- Fast Walsh-Hadamard transform implementation
- Minimal floating-point operations per sample

### Sample Rate Handling
- Dynamic buffer sizing based on sample rate
- Proper CV scaling for different sample rates
- Control updates at 1/4 audio rate for efficiency