<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Phasmal.cpp
Generated: 2026-11-04 06:25
-->

# Phasmal

## 1. Overview

Phasmal is a phaser effect module for VCV Rack 2 that implements a digital simulation of a classic analog phaser circuit. It provides stereo phasing with modulated all-pass filter stages, offering controls for rate, depth, center frequency, feedback, stages, stereo spread, mix, and output level.

The module features:
- Dual-channel (left/right) processing
- LFO-driven modulation of filter frequencies
- Configurable LFO shapes (sine/triangle)
- Visual display showing frequency response and LFO scope
- CV inputs for real-time parameter modulation

## 2. Architecture

### Main Classes

**Phasmal**: The core module class inheriting from `rack::Module`
**PhasmalWidget**: The UI widget class inheriting from `rack::ModuleWidget`
**PhasmalViz**: A shared data structure used by both DSP and display code for visualization parameters

### Inheritance

- `Phasmal` inherits from `rack::Module`
- `PhasmalWidget` inherits from `rack::ModuleWidget`
- Anonymous namespace contains helper classes:
  - `PHPanelBackground`: Panel background drawing
  - `PHKnobArcs`: Live knob value indicators
  - `PHDisplayWidget`: Frequency response and LFO scope display

### Structures

**PhasmalViz**: 
```cpp
struct PhasmalViz
{
    float fcL = 780.f, fcR = 780.f;
    float fcMin = 100.f, fcMax = 6000.f;
    float fb = 0.4f, mix = 0.5f, level = 1.f;
    float rate = 0.5f, depth = 0.6f, spread = 0.5f;
    float phase = 0.15f;
    int stages = 6;
    int shape = 0;
    float fs = 44100.f;
};
```

### Relationships

- `Phasmal` contains DSP state and processing logic
- `PhasmalWidget` manages UI elements and connects to `Phasmal`
- `PhasmalViz` is shared between DSP processing and display drawing for visualization
- All drawing helpers are anonymous namespace classes that draw within the widget context

## 3. DSP Theory

### Mathematical Foundation

The phaser effect is implemented using a cascade of first-order all-pass filters, which create phase shifts that interfere with each other to produce notches in the frequency response.

### Algorithm

1. **LFO Generation**: 
   - Generates either sine or triangle wave at specified rate
   - `phLfo(float p, int shape)` implements LFO generation
   - Sine: $ \sin(2\pi p) $
   - Triangle: $ 1 - 4|\text{frac}(p + 0.25) - 0.5| $

2. **All-pass Filter Coefficient Calculation**:
   - First-order all-pass coefficient for corner frequency $ f_c $:
   $$ a = \frac{\tan(\pi f_c / f_s) - 1}{\tan(\pi f_c / f_s) + 1} $$

3. **Frequency Response Calculation**:
   - Magnitude response of digital phaser at frequency $ f $:
   $$ H(f) = (1 - M) + M \cdot \frac{N}{1 - F \cdot A \cdot z^{-1}} $$
   where:
   - $ N = \text{norm} = 1 - 0.5|F| $
   - $ A = \frac{a + z^{-1}}{1 + a \cdot z^{-1}} $
   - $ F = \text{feedback} $
   - $ M = \text{mix} $

4. **Filter Cascade**:
   - Each stage applies the all-pass filter with coefficient $ a $
   - Feedback is applied to previous output
   - Output is processed through tanh for resonance control

### Filters

- First-order all-pass filters implemented in cascade
- Feedback system with resonance control via tanh function
- Frequency modulation using LFO sweep

### Modulation

- Rate: Controls LFO frequency (logarithmic scale)
- Depth: Controls LFO amplitude (0-1 range)
- Center frequency: Base frequency for sweep (logarithmic scale)
- Feedback: Amount of signal fed back into filter stages
- Stereo spread: Phase difference between left/right channels

### Random Processes

- None directly implemented; uses deterministic LFO for modulation

## 4. Parameters and Controls

### Knobs

| Parameter | ID | Range | Default | Units |
|-----------|----|-------|---------|-------|
| RATE | `RATE_PARAM` | -5 to 4 | -1 | Hz (log scale) |
| DEPTH | `DEPTH_PARAM` | 0 to 1 | 0.6 | % |
| FREQ | `FREQ_PARAM` | 6.64 to 12.3 | 9.6 | Hz (log scale) |
| FEEDBACK | `FEEDBACK_PARAM` | -0.95 to 0.95 | 0.4 | % |
| STAGES | `STAGES_PARAM` | 0 to 5 | 2 | stages (2-12) |
| SPREAD | `SPREAD_PARAM` | 0 to 1 | 0.5 | degrees |
| MIX | `MIX_PARAM` | 0 to 1 | 0.5 | % |
| LEVEL | `LEVEL_PARAM` | 0 to 2 | 1 | % |

### Switches

- LFO shape switch (sine/triangle)

### Buttons

- None

### CV Inputs

| Input | ID | Range | Description |
|-------|----|-------|-------------|
| RATE_CV_INPUT | `RATE_CV_INPUT` | ±5 V | Rate modulation (1V/oct) |
| DEPTH_CV_INPUT | `DEPTH_CV_INPUT` | ±5 V | Depth modulation (+-50%) |
| FREQ_CV_INPUT | `FREQ_CV_INPUT` | ±5 V | Frequency modulation (1V/oct) |
| FB_CV_INPUT | `FB_CV_INPUT` | ±5 V | Feedback modulation (+-50%) |

### Gates

- None

### Enumerations

```cpp
enum LfoShape
{
    SHAPE_SINE,
    SHAPE_TRI,
    NUM_SHAPES
};
```

## 5. Inputs and Outputs

### Inputs

| Name | ID | Description |
|------|----|-------------|
| IN_L_INPUT | `IN_L_INPUT` | Left input (mono or stereo) |
| IN_R_INPUT | `IN_R_INPUT` | Right input (normalled to left) |
| RATE_CV_INPUT | `RATE_CV_INPUT` | Rate CV (1 V/oct) |
| DEPTH_CV_INPUT | `DEPTH_CV_INPUT` | Depth CV (+-5 V = +-50%) |
| FREQ_CV_INPUT | `FREQ_CV_INPUT` | Center frequency CV (1 V/oct) |
| FB_CV_INPUT | `FB_CV_INPUT` | Feedback CV (+-5 V = +-50%) |

### Outputs

| Name | ID | Description |
|------|----|-------------|
| OUT_L_OUTPUT | `OUT_L_OUTPUT` | Left output |
| OUT_R_OUTPUT | `OUT_R_OUTPUT` | Right output |

## 6. Signal Processing Pipeline

1. **Parameter Processing**:
   - Read parameter values and CV inputs
   - Apply clamping to valid ranges
   - Convert log scale parameters to linear values

2. **LFO Generation**:
   - Update phase with rate × sample time
   - Generate LFO waveform (sine or triangle)
   - Apply stereo spread to right channel

3. **Frequency Sweep**:
   - Calculate sweep range based on depth and center frequency
   - Apply LFO modulation to generate dynamic frequencies
   - Clamp frequencies to valid ranges

4. **Filter Processing**:
   - Process input through cascade of all-pass filters
   - Apply feedback with resonance control (tanh)
   - Mix original and filtered signals

5. **Output Generation**:
   - Set output voltages with level scaling
   - Update visualization data for display

## 7. Important Functions

### `process()`
Main DSP processing function that:
- Reads parameters and CV inputs
- Updates LFO phase
- Calculates sweep frequencies
- Processes audio through filter cascade
- Sets output voltages

### `phLfo()`
Generates LFO waveform:
- Sine or triangle based on shape parameter
- Returns normalized waveform value

### `phStageCoef()`
Calculates all-pass filter coefficient:
- Uses tangent function for digital implementation
- Returns filter coefficient for given frequency and sample rate

### `phMagDb()`
Computes magnitude response:
- Calculates exact phaser frequency response at given frequency
- Used for display visualization

### `dataToJson()` / `dataFromJson()`
Handles plugin state persistence:
- Saves LFO shape setting to JSON
- Loads LFO shape from saved configuration

## 8. Mathematical Implementation

### All-pass Filter Coefficient

$$ a = \frac{\tan(\pi f_c / f_s) - 1}{\tan(\pi f_c / f_s) + 1} $$

### LFO Waveforms

**Sine:**
$$ \sin(2\pi p) $$

**Triangle:**
$$ 1 - 4|\text{frac}(p + 0.25) - 0.5| $$

### Frequency Sweep

$$ f_{\text{left}} = f_0 \cdot 2^{s \cdot lfo_L} $$
$$ f_{\text{right}} = f_0 \cdot 2^{s \cdot lfo_R} $$

Where:
- $ f_0 $: center frequency
- $ s $: sweep depth (octaves)
- $ lfo_L/R $: LFO values for left/right channels

### Feedback Processing

$$ x_{\text{new}} = \text{tanh}(x_{\text{filtered}} \cdot 0.2) \cdot 5 $$

## 9. Performance and Optimization

### CPU Usage Considerations

- Uses efficient first-order all-pass filters
- Minimal floating-point operations per sample
- No complex transcendental functions in critical path
- Limited memory usage with fixed-size arrays

### Memory Allocation

- Fixed-size DSP state arrays: `st[2][12]` and `fbState[2]`
- No dynamic allocation during processing
- Stack-based temporary variables

### Buffers

- Internal filter state stored in fixed arrays
- No external buffers required

### Real-time Processing

- All operations performed within real-time constraints
- No blocking or time-consuming operations
- Sample-accurate LFO phase updates

### Thread Safety

- Single-threaded processing (VCV Rack's single audio thread)
- No concurrent access to module state during processing

### Numerical Stability

- Uses clamping for parameter ranges to prevent invalid values
- Proper handling of edge cases in frequency calculations
- Tanh function used for resonance control to prevent overflow

## 10. Dependencies

### Libraries/Headers

- `rack.hpp` - VCV Rack core library
- `<cmath>` - Standard math functions
- `<cstdio>` - Standard I/O
- `<cstring>` - String operations
- `<complex>` - Complex number support for frequency response calculation
- `<algorithm>` - Standard algorithms
- `<memory>` - Smart pointers

### External Classes

- `rack::Module` - Base module class
- `rack::ModuleWidget` - Base widget class
- `rack::ParamQuantity` - Parameter handling
- `rack::math::clamp()` - Value clamping function
- `nvgXXX` functions - NanoVG drawing functions (UI elements)

## 11. Summary

Phasmal is a sophisticated phaser effect module implementing digital all-pass filter cascades with LFO modulation. It provides stereo processing with configurable parameters including rate, depth, center frequency, feedback, stages, and stereo spread. The implementation uses first-order all-pass filters for accurate simulation of analog phaser behavior, with proper handling of feedback and resonance control. The module includes a comprehensive visual display showing frequency response and LFO scope, making it both functional and informative for audio processing applications.