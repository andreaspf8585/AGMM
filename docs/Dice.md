<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Dice.cpp
Generated: 2026-11-04 06:16
-->

# Dice

## 1. Overview

The Dice module is a polyphonic random voltage generator for VCV Rack that produces gate signals and voltage outputs (V/Oct) based on clock inputs. It generates random voltages within configurable ranges, quantizes them to musical scales, and outputs gates with configurable probability and duration.

Key features:
- Polyphonic operation (1-16 voices)
- Clock-triggered random voltage generation
- Configurable gate probability (0-100%)
- Gate duration control (5ms to 500ms)
- Random voltage range selection (-5/+5V, -2/+2V, -1/+1V, 0/+1V)
- Musical scale quantization with 64 predefined scales
- Visual display showing current configuration and output voltages

## Module Screenshot

![Dice VCV Rack Module](images/dice.png)

## 2. Architecture

### Main Classes

**Dice**: The primary module class inheriting from `rack::Module`
- Manages all state and processing logic
- Handles parameters, inputs, outputs, and lights
- Implements core DSP algorithms for random voltage generation and quantization

**DiceWidget**: UI component for the module
- Extends `rack::ModuleWidget` 
- Provides visual representation including:
  - Panel layout with controls and I/O ports
  - Custom display widget showing voltage waveforms
  - LED indicators for each voice channel
  - Context menu for configuration options

### Inheritance

```
Dice : Module
DiceWidget : ModuleWidget
```

### Structures

**DiceScale**: 
- `std::string name` - Scale name (e.g., "Major", "Minor")
- `std::vector<float> intervals` - Semitone intervals from root note

### Relationships

- `Dice` contains state arrays for 16 voices: gate voltages, V/Oct voltages, quantized voltages, and gate timers
- `DiceWidget` owns the UI components and displays data from `Dice`
- The module communicates with the host system through standard VCV Rack interfaces (parameters, inputs, outputs, lights)

### Module Organization

The module is organized as a single source file (`Dice.cpp`) containing:
1. Layout constants for UI positioning
2. Scale definitions (64 predefined musical scales)
3. Main `Dice` class implementation
4. UI drawing code in anonymous namespace
5. Widget creation and context menu logic

## 3. DSP Theory

### Mathematical Equations

**Random Voltage Generation:**
$$ v_{rand} = \begin{cases}
u \cdot 1.0 & \text{if range = 0} \\
u \cdot 2.0 - 1.0 & \text{if range = 1} \\
u \cdot 4.0 - 2.0 & \text{if range = 2} \\
u \cdot 10.0 - 5.0 & \text{if range = 3}
\end{cases} $$

Where $ u \in [0,1) $ is a uniform random variable.

**Quantization to Scale:**
$$ v_{quantized} = \text{octave} + \text{bestMatch} $$

Where:
- $\text{octave} = \lfloor \frac{v_{rand} \cdot 12}{12} \rfloor$
- $\text{bestMatch}$ is the closest interval from the selected scale's intervals

**Gate Timing:**
$$ \text{gateTimer}[c] = \max(0, \text{gateTimer}[c] - \Delta t) $$

Where $\Delta t$ is the sample time.

### Algorithms

**Random Voltage Generation Algorithm:**
1. Select random range based on `randomRange` parameter
2. Generate uniform random value $ u \in [0,1) $
3. Scale and offset according to selected range:
   - Range 0: $[0,1)$ → $[0,1)$
   - Range 1: $[0,1)$ → $[-1,1)$
   - Range 2: $[0,1)$ → $[-2,2)$
   - Range 3: $[0,1)$ → $[-5,5)$

**Scale Quantization Algorithm:**
1. Convert voltage to semitones: $ \text{semitones} = v_{rand} \cdot 12 $
2. Extract octave and fractional part
3. For each interval in the scale:
   - Calculate difference between fractional part and interval (normalized to 0-1)
   - Consider both positive and negative octave shifts
4. Select the interval with minimum difference

**Gate Generation Algorithm:**
1. Detect rising clock edge
2. For each active voice channel:
   - Generate random probability check
   - If successful, set gate voltage to 10V and initialize timer
3. Decrement gate timers for all channels
4. When timer expires, reset gate voltage to 0

### Filters

None implemented.

### Oscillators

None implemented.

### Delay Lines

None implemented.

### Feedback Systems

None implemented.

### Modulation

- Clock input modulates when gates are generated
- Probability parameter modulates gate generation chance
- Polyphony parameter controls number of active voices

### Random Processes

Uniform random number generation via `random::uniform()` for:
- Gate probability decisions (0-100%)
- Voltage generation within specified ranges

## 4. Parameters and Controls

### Knobs

**PARAM_POLYPHONY** (70, 226)
- Range: 1 to 16 voices
- Default: 1 voice
- Units: voices

**PARAM_PROBABILITY** (170, 226)
- Range: 0.0 to 1.0 (0% to 100%)
- Default: 0.5 (50%)
- Units: %

### Switches/Buttons

None implemented.

### CV Inputs

**INPUT_CLOCK** (36, 332)
- Voltage range: 0V to 10V
- Behavior: Rising edge triggers gate generation

### Gates

None implemented.

### Enumerations

**Dice::ParamId**
- `PARAM_POLYPHONY`
- `PARAM_PROBABILITY`

**Dice::InputId**
- `INPUT_CLOCK`

**Dice::OutputId**
- `OUTPUT_GATES`
- `OUTPUT_VOCT`
- `OUTPUT_QUANTIZED`

**Dice::LightId**
- `LIGHT_GATES` (16 lights, one per voice)

### Parameter Ranges and Default Values

| Parameter | Range | Default |
|-----------|-------|---------|
| PARAM_POLYPHONY | 1-16 | 1 |
| PARAM_PROBABILITY | 0.0-1.0 | 0.5 |

## 5. Inputs and Outputs

### Input

**INPUT_CLOCK**
- Voltage range: 0V to 10V
- Behavior: Clock signal triggers gate generation
- Connection type: DarkPJ301M port

### Outputs

**OUTPUT_GATES**
- Voltage range: 0V or 10V (gate)
- Channels: 16 (polyphonic)
- Connection type: DarkPJ301M port

**OUTPUT_VOCT**
- Voltage range: -5V to +5V (random V/Oct)
- Channels: 16 (polyphonic)
- Connection type: DarkPJ301M port

**OUTPUT_QUANTIZED**
- Voltage range: -5V to +5V (quantized V/Oct)
- Channels: 16 (polyphonic)
- Connection type: DarkPJ301M port

## 6. Signal Processing Pipeline

1. **Clock Detection**: Detect rising edge on INPUT_CLOCK
2. **Polyphony Handling**: Clamp PARAM_POLYPHONY to [1,16] range
3. **Gate Generation**:
   - For each voice channel up to polyphony:
     - Generate random probability check
     - If successful: set gate voltage = 10V, initialize timer
     - If failed: set gate voltage = 0V, reset timer
4. **Voltage Generation**:
   - Generate random voltage in selected range
   - Quantize to selected musical scale
5. **Gate Timer Management**:
   - Decrement all gate timers by sample time
   - When timer expires, set gate voltage = 0V
6. **Output Routing**:
   - Set output channels based on current polyphony
   - Output gate, random V/Oct, and quantized V/Oct voltages

## 7. Important Functions

### `quantizeToScale(float voltage, const DiceScale &scale)`
- Converts a voltage to the closest note in the given scale
- Implements nearest neighbor search through scale intervals
- Handles octave wrapping for fractional semitones

### `generateRandomVoltage(int channel)`
- Generates random voltage for specified voice channel
- Applies selected range scaling based on `randomRange` parameter
- Calls quantization function with current scale

### `onReset(const ResetEvent &e)`
- Resets module state to defaults:
  - Gate time = 10ms
  - Random range = 0 (0 to +1V)
  - Selected scale = 0 (Major)

### `dataToJson()`
- Serializes module state for saving presets

### `dataFromJson(json_t *root)`
- Deserializes module state from saved presets

### `process(const ProcessArgs &args)`
- Main processing function called each audio block
- Handles clock detection and edge triggering
- Manages all voice states and output generation

## 8. Mathematical Implementation

### Random Voltage Generation

The implementation uses `random::uniform()` to generate a uniform random value in [0,1), then scales it according to the selected range:

```cpp
switch (randomRange)
{
case 0: voltage = random::uniform(); break;           // [0,1)
case 1: voltage = (random::uniform() * 2.0f) - 1.0f;  // [-1,1)
case 2: voltage = (random::uniform() * 4.0f) - 2.0f;  // [-2,2)
case 3: voltage = (random::uniform() * 10.0f) - 5.0f; // [-5,5)
}
```

### Scale Quantization

The quantization algorithm:
1. Converts voltage to semitones: $ \text{semitones} = v_{rand} \cdot 12 $
2. Extracts octave and fractional part
3. For each interval in the scale:
   - Calculates difference between fractional part and interval (0-1 range)
   - Considers both positive and negative octave shifts
4. Selects the interval with minimum difference

### Gate Timing

```cpp
gateTimers[c] -= args.sampleTime;
if (gateTimers[c] <= 0.0f)
    gateVoltages[c] = 0.0f;
```

## 9. Performance and Optimization

### CPU Usage Considerations

- Single audio thread processing
- Minimal floating-point operations per voice
- Efficient array access patterns
- No complex mathematical functions beyond basic trigonometry for UI visualization

### Memory Allocation

- Static arrays for voice state (16 elements each)
- Precomputed scale data stored in static vector
- No dynamic memory allocation during processing

### Buffers

- Voice state arrays: 16 elements each
- No dedicated buffers required

### Real-time Processing

- All operations completed within audio block time
- No blocking operations or waits
- Sample-accurate timing using `args.sampleTime`

### Thread Safety

- Module state accessed only from audio thread
- UI thread reads display data (torn reads are cosmetic)
- No synchronization primitives required

### Numerical Stability

- Uses standard floating-point arithmetic
- Proper clamping of values to valid ranges
- No recursive or iterative algorithms that could accumulate errors

## 10. Dependencies

### Libraries

- `rack.hpp` - VCV Rack core library
- Standard C++ libraries: `<vector>`, `<string>`, `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`
- JSON handling via `json_t` (from VCV Rack)

### Classes and Headers

- `rack::Module` - Base module class
- `rack::ModuleWidget` - UI widget base class
- `rack::ParamQuantity` - Parameter handling
- `rack::Input`/`Output` - I/O port handling
- `rack::Light` - Light indicator handling
- `rack::random::uniform()` - Random number generation
- `json_t` - JSON serialization support

## 11. Summary

The Dice module is a polyphonic random voltage generator that produces gate signals and V/Oct outputs based on clock inputs. It features:

- 16-channel polyphony with configurable voice count (1-16)
- Clock-triggered gate generation with adjustable probability (0-100%)
- Configurable gate duration (5ms to 500ms)
- Four random voltage ranges (-5/+5V, -2/+2V, -1/+1V, 0/+1V)
- 64 predefined musical scales for quantization
- Visual display showing current configuration and output waveforms
- Preset saving/loading capabilities

The implementation uses efficient algorithms for random voltage generation and scale quantization, with minimal CPU overhead suitable for real-time audio processing in VCV Rack.