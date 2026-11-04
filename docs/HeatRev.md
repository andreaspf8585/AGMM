<!--
AGMM AUTOMATIC DOCUMENTATION
Source: HeatRev.cpp
Generated: 2026-11-04 06:19
-->

# HeatRev

## 1. Overview

The HeatRev module is a stereo reverb and delay processor that combines a cathedral reverberation engine with a heat-grid simulation for warm, organic tonal coloration. It features:

- **Cathedral Reverb**: A high-quality 8-line Feedback Delay Network (FDN) reverb with early reflections, pre-delay, and input diffusion
- **Heat Grid Simulation**: A cellular automaton-inspired heat propagation system that adds warmth and character to the signal
- **Stereo Delay**: A configurable stereo delay line with feedback and fractional delay interpolation
- **Visual Display**: Real-time visualization of the heat-grid state

The module is designed for artistic sound design, adding depth and warmth to audio signals through a combination of algorithmic reverb and organic simulation.

## 2. Architecture

### Main Classes

1. **HeatRev** - The main module class inheriting from `rack::Module`
2. **SimpleLPF** - One-pole low-pass filter for DC blocking
3. **StereoDelay** - Stereo delay line with feedback and fractional interpolation
4. **Cathedral** - 8-line FDN reverb engine with early reflections and pre-delay

### Inheritance

- `HeatRev` inherits from `rack::Module`
- No other inheritance relationships in the core implementation

### Structures

- `SimpleLPF`: Simple one-pole low-pass filter
- `StereoDelay`: Stereo delay line with buffer management and fractional read
- `Cathedral`: Complex reverb engine with:
  - 8 delay lines (FDN)
  - Pre-delay buffer
  - Early reflection buffers
  - Allpass filters for input diffusion
  - LFO modulation for line delays
  - Hadamard mixing matrix

### Relationships

- `HeatRev` contains instances of `SimpleLPF`, `StereoDelay`, and `Cathedral`
- The heat-grid simulation is implemented directly within `HeatRev::process()`
- All DSP components are integrated into the main processing loop

### Module Organization

The module is organized as a single C++ file with:
1. DSP component definitions (classes)
2. Main module class (`HeatRev`)
3. UI widget implementation in anonymous namespace
4. Panel background and display widgets
5. Knob and port placement logic

## 3. DSP Theory

### Mathematical Equations

**One-pole low-pass filter:**
$$ y[n] = y[n-1] + a(x[n] - y[n-1]) $$
where $ a = 1 - e^{-2\pi f_c T} $

**Delay line with fractional interpolation:**
$$ y(t) = x[i] + (t - i)(x[i+1] - x[i]) $$

**Reverb loop gain from RT60:**
$$ g_i = 10^{-3 \cdot L / (RT60 \cdot sr)} $$
where $ L $ is the delay line length in samples

**Hadamard mixing matrix:**
Energy-preserving orthogonal matrix for mixing reverb lines

### Algorithms

1. **Cathedral Reverb**: 
   - 8-line FDN with orthonormal Hadamard mixing
   - Pre-delay + early reflections
   - Input diffusion using allpass filters
   - LFO modulation to avoid metallic ringing
   - Stability by construction (loop gain < 1)

2. **Heat Grid Simulation**:
   - 2D cellular automaton with Laplacian operator
   - Diffusion process: $ u_{new} = u + diffK \cdot \nabla^2 u $
   - Boundary conditions: periodic with injection at edges

3. **Stereo Delay**:
   - Fractional delay interpolation using linear interpolation
   - Separate left/right delay lines
   - Feedback with clamping to prevent instability

### Filters

- **SimpleLPF**: One-pole low-pass filter for DC blocking
- **Damping Filter**: First-order low-pass filter applied to reverb tail
- **Heat Grid Low-pass**: Simple LPF for smoothing heat output

### Oscillators

- **LFO Modulation**: Sine wave oscillators modulating delay line lengths
- **Modulation Frequency**: $ f = (0.11 + 0.037 \cdot i) / sr $

### Delay Lines

- **Cathedral FDN**: 8 delay lines with fixed lengths from lookup table
- **Pre-delay**: Single delay line for early reflections
- **StereoDelay**: Two delay lines with fractional interpolation

### Feedback Systems

- **Reverb Feedback**: Loop gain controlled by RT60 and damping
- **Heat Grid Feedback**: Diffusion coefficient controls feedback amount
- **Delay Feedback**: Configurable feedback with maximum 95% clamping

### Modulation

- **LFO Modulation**: Slow sine wave modulation of delay line lengths
- **Parameter Modulation**: All parameters are modulatable via CV inputs

### Random Processes

- **Heat Grid**: Deterministic cellular automaton (no true randomness)
- **Modulation**: LFOs provide periodic modulation for natural variation

## 4. Parameters and Controls

### Knobs

1. **DIFFUSION_PARAM** (0.0 - 1.0, default 0.7)
   - Controls input diffusion in reverb
   - Range: 0% to 100%

2. **DECAY_PARAM** (0.0 - 1.0, default 0.8)
   - Controls reverb decay time (RT60)
   - Range: 0% to 100%
   - Maps to 0.5s to 15s RT60

3. **SIZE_PARAM** (4.0 - 64.0, default 48.0)
   - Controls heat-grid size
   - Range: 4 to 64 cells
   - Snap enabled

4. **MIX_PARAM** (0.0 - 1.0, default 0.7)
   - Controls wet/dry mix
   - Range: 0% to 100%

5. **DAMPING_PARAM** (0.0 - 1.0, default 0.55)
   - Controls high-frequency damping
   - Range: 0% to 100%
   - Higher values = darker reverb

6. **LEVEL_PARAM** (0.0 - 1.0, default 0.8)
   - Controls overall output level
   - Range: 0% to 100%

7. **DELAY_TIME_PARAM** (10.0 - 500.0, default 80.0)
   - Controls delay time in milliseconds
   - Range: 10ms to 500ms

8. **DELAY_FEEDBACK_PARAM** (0.0 - 0.95, default 0.4)
   - Controls delay feedback amount
   - Range: 0% to 95%

9. **DELAY_LEVEL_PARAM** (0.0 - 1.0, default 0.7)
   - Controls delay output level
   - Range: 0% to 100%

10. **STEREO_SPREAD_PARAM** (0.0 - 50.0, default 15.0)
    - Controls stereo spread in delay
    - Range: 0ms to 50ms

11. **SPACE_PARAM** (0.0 - 1.0, default 0.7)
    - Controls pre-delay and early reflection
    - Range: 0% to 100%

### CV Inputs

- **INPUT_L**: Left input signal
- **INPUT_R**: Right input signal

### Gates

None explicitly defined in the code.

### Enumerations

- `ParamId`: Enum for parameter IDs (11 parameters)
- `InputId`: Enum for input IDs (2 inputs)
- `OutputId`: Enum for output IDs (2 outputs)
- `LightId`: Enum for light IDs (0 lights)

## 5. Inputs and Outputs

### Inputs

1. **INPUT_L** (Left Input)
   - Voltage range: ±10V
   - Signal behavior: Audio input signal

2. **INPUT_R** (Right Input)
   - Voltage range: ±10V
   - Signal behavior: Audio input signal

### Outputs

1. **OUTPUT_L** (Left Output)
   - Voltage range: ±10V
   - Signal behavior: Mixed dry and wet signals

2. **OUTPUT_R** (Right Output)
   - Voltage range: ±10V
   - Signal behavior: Mixed dry and wet signals

## 6. Signal Processing Pipeline

1. **Input Processing**
   - Read left/right input voltages
   - Apply soft clipping to prevent overdrive

2. **Delay Processing**
   - Calculate delay times for left/right channels
   - Process stereo delay with feedback
   - Mix delay output with dry signal

3. **Reverb Processing**
   - Process cathedral reverb engine
   - Apply early reflections and pre-delay
   - Apply damping filter to reverb tail
   - Apply Hadamard mixing matrix

4. **Heat Grid Processing**
   - Inject reverb signal into grid edges
   - Apply diffusion and feedback
   - Sample grid at specific points for output

5. **Mixing**
   - Combine reverb, delay, and heat signals
   - Apply wet/dry mix ratio
   - Apply final level control

6. **Output Processing**
   - Apply smooth limiter to prevent clipping
   - Apply soft clipping to prevent overdrive
   - Set output voltages

## 7. Important Functions

### HeatRev Class Methods

1. **`process()`** - Main processing function called every audio block
2. **`onReset()`** - Reset all internal state
3. **`updateViz()`** - Update visualization data for display

### DSP Component Methods

1. **`SimpleLPF::setCutoff()`** - Set filter cutoff frequency
2. **`SimpleLPF::process()`** - Apply low-pass filtering
3. **`StereoDelay::setSampleRate()`** - Set sample rate for delay processing
4. **`StereoDelay::resize()`** - Resize delay buffers
5. **`StereoDelay::setDelayMs()`** - Set delay times in milliseconds
6. **`StereoDelay::setFeedback()`** - Set feedback amount
7. **`StereoDelay::process()`** - Process stereo delay
8. **`Cathedral::init()`** - Initialize reverb engine
9. **`Cathedral::reset()`** - Reset reverb state
10. **`Cathedral::allpass()`** - Allpass filter implementation
11. **`Cathedral::readPD()`** - Read from pre-delay buffer
12. **`Cathedral::process()`** - Main reverb processing function

## 8. Mathematical Implementation

### One-pole Low-pass Filter

```cpp
void SimpleLPF::setCutoff(float normCutoff) {
    if (normCutoff <= 0.0f) a = 0.0f;
    else if (normCutoff >= 1.0f) a = 1.0f;
    else a = 1.0f - std::exp(-2.0f * float(M_PI) * normCutoff);
}

float SimpleLPF::process(float x) {
    y = y + a * (x - y);
    return y;
}
```

### Fractional Delay Interpolation

```cpp
float readBuffer(const std::vector<float>& buf, float delaySamples) {
    if (delaySamples <= 0.0f) return 0.0f;
    float readPos = static_cast<float>(writePos) - delaySamples;
    if (readPos < 0.0f) readPos += bufferSize;
    int idx1 = static_cast<int>(readPos);
    int idx2 = (idx1 + 1) % bufferSize;
    float frac = readPos - idx1;
    return buf[idx1] + frac * (buf[idx2] - buf[idx1]);
}
```

### Reverb Loop Gain Calculation

```cpp
g[i] = std::pow(10.f, -3.f * len / (rt60 * sr));
```

### Hadamard Mixing Matrix

```cpp
for (int h = 1; h < N; h <<= 1) {
    for (int i = 0; i < N; i += (h << 1)) {
        for (int j = i; j < i + h; ++j) {
            const float a = y[j], c = y[j + h];
            y[j] = a + c;
            y[j + h] = a - c;
        }
    }
}
```

### Heat Grid Diffusion

```cpp
float laplacian = (u_left + u_right + u_up + u_down) - 4.0f * u;
next[idx] = u + diffK * laplacian;
```

## 9. Performance and Optimization

### CPU Usage Considerations

- **Reverb Engine**: 8 delay lines with LFO modulation, Hadamard mixing
- **Heat Grid**: 2D cellular automaton with boundary conditions
- **Delay Processing**: Fractional interpolation with buffer management
- **Visualization**: Periodic snapshot updates (~90 Hz)

### Memory Allocation

- **Dynamic Buffers**: 
  - Delay lines: `std::vector<float>` for each delay line
  - Heat grid: `std::vector<float>` of size `size × size`
  - Visualization: Fixed-size 2D array `viz[VIZ_ROWS][VIZ_COLS]`

### Buffers

- **Delay Buffers**: Dynamically resized based on delay time requirements
- **Heat Grid**: Fixed-size buffers for current and next states
- **Pre-delay**: Circular buffer of fixed size (131072 samples)
- **Early Reflections**: Circular buffers of fixed size (2048 samples each)

### Real-time Processing

- All processing occurs in the audio thread
- No blocking operations or external synchronization
- Efficient memory access patterns with cache-friendly layouts

### Thread Safety

- Single-threaded design (audio thread only)
- Visualization data accessed by UI thread with no locking required
- Torn reads in visualization are cosmetic only

### Numerical Stability

- **Clamping**: All feedback values clamped to prevent instability
- **Limiting**: Soft limiting applied to output signals
- **Finite Check**: Safety net for non-finite values
- **Energy Preservation**: Hadamard mixing matrix preserves energy
- **Stability by Construction**: Loop gains always < 1 for reverb

## 10. Dependencies

### Libraries

- `rack.hpp` - VCV Rack core library
- Standard C++ STL: `<vector>`, `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<utility>`

### Classes and Headers

- `rack::Module` - Base module class
- `rack::math::clamp()` - Value clamping function
- `rack::math::isfinite()` - Floating-point finiteness check
- `rack::math::tanh()` - Hyperbolic tangent function
- `rack::window::Font` - Font handling for UI
- `rack::window::Widget` - Widget base class
- `rack::window::DrawArgs` - Drawing arguments

### External Dependencies

- VCV Rack 2 API
- NVG (NanoVG) for UI rendering
- System font loading capabilities

## 11. Summary

HeatRev is a sophisticated stereo reverb and delay processor that combines algorithmic reverb with organic heat-grid simulation. The module features:

- **Cathedral Reverb Engine**: 8-line FDN with early reflections, pre-delay, and input diffusion
- **Heat Grid Simulation**: Cellular automaton-based warmth generation
- **Stereo Delay**: Fractional delay interpolation with feedback
- **Visual Feedback**: Real-time heat-grid visualization
- **Parameter Modulation**: All parameters are CV modulatable
- **Stability**: Built-in stability measures for all feedback systems

The implementation uses efficient DSP algorithms including fractional delay interpolation, Hadamard mixing matrices, and cellular automata. The module is designed for artistic sound design applications where natural-sounding reverb with organic warmth is desired.