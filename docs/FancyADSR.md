<!--
AGMM AUTOMATIC DOCUMENTATION
Source: FancyADSR.cpp
Generated: 2026-11-04 06:18
-->

# FancyADSR

## 1. Overview

The `FancyADSR` module is a VCV Rack 2 plugin that implements an advanced ADSR (Attack, Decay, Sustain, Release) envelope generator with additional features such as:

- Configurable attack and release curve shapes (linear or exponential)
- Loop mode for continuous envelope generation
- Real-time waveform visualization in the module UI
- End-of-release output trigger
- CV-controlled parameters
- Gate and trigger inputs for envelope control

The module provides a visual representation of the current envelope shape and allows users to observe real-time envelope behavior through an oscilloscope-style display.

## Module Screenshot

![FancyADSR VCV Rack Module](images/fancyadsr.png)

## 2. Architecture

### Main Classes

- **`FancyADSR`**: The core module class inheriting from `rack::Module`
- **`FAPanelBackground`**: Custom panel background widget
- **`FADisplayWidget`**: Custom display widget for visualizing envelope shape and history
- **`FancyADSRWidget`**: Module widget that constructs the UI layout

### Inheritance

- `FancyADSR` inherits from `rack::Module`
- `FAPanelBackground`, `FADisplayWidget`, and `FancyADSRWidget` inherit from `rack::widget::TransparentWidget`

### Structures

```cpp
struct FancyADSR : Module
{
    enum ParamIds { ... };
    enum InputIds { ... };
    enum OutputIds { ... };
    enum LightIds { ... };
    
    enum Stage { STAGE_IDLE, STAGE_ATTACK, STAGE_DECAY, STAGE_SUSTAIN, STAGE_RELEASE };
    
    // State variables
    Stage stage;
    float env;
    float stageTime;
    float eorPulseRemaining;
    bool lastGateState;
    bool lastTriggerState;
    
    // History buffer for display
    static const int HISTORY_SIZE = 400;
    float history[HISTORY_SIZE];
    int historyCount;
    int historyWritePos;
    float sampleAccumulator;
    
    // Display data (shared between audio and UI threads)
    float dispAttack, dispDecay, dispSustain, dispRelease;
    float dispAttackShape, dispReleaseShape;
    bool dispGate;
    float eorFlash;
};
```

### Relationships

- `FancyADSRWidget` contains:
  - `FAPanelBackground`
  - `FADisplayWidget` (linked to the module)
  - Knobs, switches, and ports arranged in rows
- `FADisplayWidget` reads from `FancyADSR`'s display data (`disp*`) and history buffer
- The module maintains internal state for envelope generation and UI display

### Module Organization

The module is organized into:
1. Core DSP logic in `FancyADSR`
2. UI components in anonymous namespace (panel, display, widgets)
3. Module registration via `modelFancyADSR` at the end of the file

## 3. DSP Theory

### Mathematical Equations

#### Exponential Envelope Segments
For exponential curves, the value at time $t$ is computed using:
$$ v(t) = start + (target - start) \cdot (1 - e^{-t/\tau}) $$

Where:
- $\tau = duration / -\ln(0.001)$ to ensure the curve reaches within 0.1% of target at $duration$

#### Linear Envelope Segments
For linear curves, the value is computed as:
$$ v(t) = start + (target - start) \cdot \frac{t}{duration} $$

### Algorithms

The module implements a state machine with five stages:
1. **STAGE_IDLE**: Output is 0V
2. **STAGE_ATTACK**: From 0V to 10V (or 1.0 in normalized units)
3. **STAGE_DECAY**: From 10V to sustain level (exponential decay)
4. **STAGE_SUSTAIN**: Constant at sustain level
5. **STAGE_RELEASE**: From current level to 0V

### Filters and Modulation

- No explicit filters are implemented
- Parameters are modulated via CV inputs with scaling factor of 1/5 V per unit change
- All parameters are clamped to valid ranges

### Feedback Systems

None explicitly implemented, but the module uses feedback for:
- Loop mode: When sustain is reached and loop is enabled, attack restarts from stage 0
- End-of-release pulse generation: A short 5ms pulse when release completes

## 4. Parameters and Controls

### Knobs

| Parameter | ID | Default | Range | Units |
|-----------|----|---------|-------|-------|
| ATTACK_PARAM | 0 | 0.01s | 0.001-10.0s | seconds |
| DECAY_PARAM | 1 | 0.1s | 0.001-10.0s | seconds |
| SUSTAIN_PARAM | 2 | 0.8 | 0.0-1.0 | normalized |
| RELEASE_PARAM | 3 | 0.5s | 0.001-10.0s | seconds |
| ATTACK_SHAPE_PARAM | 4 | 1 (exponential) | 0-1 | enum |
| RELEASE_SHAPE_PARAM | 5 | 1 (exponential) | 0-1 | enum |
| LOOP_TOGGLE_PARAM | 6 | 0 (off) | 0-1 | enum |
| TIME_SCALE_PARAM | 7 | 1.0s | 0.05-5.0s | seconds |

### Switches

| Switch | ID | Default | Options |
|--------|----|---------|---------|
| ATTACK_SHAPE_PARAM | 4 | 1 (exponential) | Linear, Exponential |
| RELEASE_SHAPE_PARAM | 5 | 1 (exponential) | Linear, Exponential |
| LOOP_TOGGLE_PARAM | 6 | 0 (off) | Off, On |

### Enumerations

```cpp
enum Stage { STAGE_IDLE, STAGE_ATTACK, STAGE_DECAY, STAGE_SUSTAIN, STAGE_RELEASE };
```

## 5. Inputs and Outputs

### Inputs

| Input | ID | Voltage Range | Signal Behavior |
|-------|----|---------------|-----------------|
| GATE_INPUT | 0 | 0-10V | Gate control (high > 1.0V triggers attack, low triggers release) |
| TRIGGER_INPUT | 1 | 0-10V | Trigger input (high > 1.0V restarts attack from current stage) |
| ATTACK_CV_INPUT | 2 | -5V to +5V | Modulates attack time (5V adds 1 second) |
| DECAY_CV_INPUT | 3 | -5V to +5V | Modulates decay time (5V adds 1 second) |
| SUSTAIN_CV_INPUT | 4 | -5V to +5V | Modulates sustain level (5V adds 100%) |
| RELEASE_CV_INPUT | 5 | -5V to +5V | Modulates release time (5V adds 1 second) |

### Outputs

| Output | ID | Voltage Range | Signal Behavior |
|--------|----|---------------|-----------------|
| ENV_OUTPUT | 0 | 0-10V | Envelope output voltage |
| EOR_OUTPUT | 1 | 0-10V | End-of-release trigger (5ms pulse) |

## 6. Signal Processing Pipeline

1. **Input Edge Detection**:
   - Detect rising/falling edges on gate input
   - Detect rising edges on trigger input

2. **Parameter Retrieval**:
   - Get attack, decay, sustain, release times with CV modulation
   - Get shape parameters (linear/exponential)
   - Get loop mode state

3. **Stage Transition Logic**:
   - Gate high → Attack stage if idle or release
   - Gate low → Release stage if in attack/decay/sustain
   - Trigger high → Restart attack from current stage

4. **Envelope Generation**:
   - Advance envelope through stages using `runSegment()` function
   - Apply exponential decay for decay stage (always exponential)
   - Handle loop mode transitions

5. **Output Generation**:
   - Convert normalized envelope to 0-10V output
   - Generate EOR pulse when release completes

6. **Display Update**:
   - Update display data for UI
   - Sample envelope for history buffer
   - Update EOR flash state

## 7. Important Functions

### Core DSP Functions

```cpp
void onGate(bool high) {
    // Handle gate transitions
}

void onTrigger(bool high) {
    // Handle trigger transitions
}

float runSegment(float start, float target, float duration, float t, float shape) {
    // Compute envelope segment value with linear or exponential curve
}
```

### Parameter Helpers

```cpp
float getAttack() { ... }
float getDecay() { ... }
float getSustain() { ... }
float getRelease() { ... }
bool getLoop() { ... }
float getTimeScale() { ... }
```

### Process Function

```cpp
void process(const ProcessArgs &args) override {
    // Main DSP loop
}
```

## 8. Mathematical Implementation

### Exponential Curve Implementation

The exponential curve uses a time constant $\tau$ calculated as:
$$ \tau = \frac{duration}{-\ln(0.001)} $$

This ensures that at $t = duration$, the value is within 0.1% of the target.

### Segment Calculation

```cpp
float runSegment(float start, float target, float duration, float t, float shape)
{
    if (duration <= 0.0f) return target;
    float alpha = rack::math::clamp(t / duration, 0.0f, 1.0f);
    if (shape <= 0.5f) // linear
    {
        return start + (target - start) * alpha;
    }
    else // exponential: tau = duration / -ln(0.001)
    {
        const float eps = 0.001f;
        float tau = duration / -std::log(eps);
        float factor = 1.0f - std::exp(-t / tau);
        return start + (target - start) * factor;
    }
}
```

### Loop Mode Logic

When loop mode is enabled and gate is high during sustain:
```cpp
if (getLoop() && gateHigh)
{
    stage = STAGE_ATTACK;
    stageTime = 0.0f;
}
```

## 9. Performance and Optimization

### CPU Usage Considerations

- The module performs minimal floating-point operations per sample
- Uses lookup tables for display rendering (via history buffer)
- No complex mathematical functions beyond `exp()` and `log()`
- Efficient state machine with simple conditional logic

### Memory Allocation

- Fixed-size arrays:
  - `history[HISTORY_SIZE]` (400 floats)
  - Preallocated in constructor
- No dynamic allocation during processing

### Buffers

- History buffer: 400 samples for real-time display
- Sample accumulator for time-based sampling

### Real-time Processing

- Uses VCV Rack's sample-time based processing
- All calculations are deterministic and sample-rate independent
- No blocking operations or thread synchronization required

### Thread Safety

- Display data (`disp*`) is written by audio thread, read by UI thread
- Uses torn reads which are cosmetic only (no functional impact)
- No mutexes or locks needed for display data access

### Numerical Stability

- All parameters are clamped to valid ranges
- Exponential calculations use stable `std::exp()` function
- Time-based calculations use sample-time increments
- No division by zero in envelope calculations

## 10. Dependencies

### Libraries and Headers

- `rack.hpp` (VCV Rack core)
- `<cmath>` for mathematical functions (`std::exp`, `std::log`)
- `<cstdio>` for formatted I/O
- `<cstring>` for memory operations
- `<algorithm>` for clamp function

### VCV Rack Classes Used

- `Module`
- `ParamIds`, `InputIds`, `OutputIds`, `LightIds` enums
- `ProcessArgs`
- `configParam`, `configSwitch`, `configInput`, `configOutput`, `configLight`
- `onReset()`, `process()` virtual methods
- `rack::math::clamp`

### Custom Classes

- `FancyADSR` (main module)
- `FAPanelBackground`
- `FADisplayWidget`
- `FancyADSRWidget`
- `TFACyanLight`
- `FACyanLight`

## 11. Summary

The `FancyADSR` module is a sophisticated envelope generator that implements standard ADSR behavior with enhanced features including configurable curve shapes, loop mode, and real-time visualization. It provides both linear and exponential attack/decay/release curves, CV modulation for all parameters, and an oscilloscope-style display showing the current envelope shape and history.

The implementation uses a clean state machine approach with efficient mathematical calculations for envelope segments. The module is optimized for real-time performance with minimal CPU usage and no dynamic memory allocation during processing. The UI provides visual feedback through a custom display widget that shows both live envelope tracing and ideal curve preview, making it easy to understand the current envelope shape and parameters.