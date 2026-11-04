<!--
AGMM AUTOMATIC DOCUMENTATION
Source: PsySeq.cpp
Generated: 2026-11-04 06:28
-->

# PsySeq

## 1. Overview

PsySeq is a step sequencer module for VCV Rack that allows users to create and manipulate sequences of voltages over time. It features an 8x8 grid layout with 64 programmable steps, configurable sequence length, start position, and direction (forward, reverse, or random). The module supports both fixed voltage settings per step and random voltage generation per step. Additionally, it includes mutation capabilities that can randomly modify step voltages after complete cycle repetitions.

## 2. Architecture

### Main Classes
- **PsySeq**: The core module class inheriting from `rack::engine::Module`
- **WaitCyclesParamQuantity**: Custom parameter quantity for discrete wait cycle values (0, 4, 8, 16)
- **PsySeqWidget**: UI widget class inheriting from `rack::engine::ModuleWidget`
- Anonymous namespace containing:
  - Panel background drawing helpers
  - Grid visualization components (`PSGridWidget`, `PSStatusWidget`)
  - Theme color definitions and drawing utilities

### Inheritance
- `PsySeq` inherits from `rack::engine::Module`
- `PsySeqWidget` inherits from `rack::engine::ModuleWidget`
- `TPSCyanLight` template inherits from `GrayModuleLightWidget`

### Structures
- Enumerations for parameters, inputs, outputs, and lights
- State variables for position tracking, voltage ranges, and cycle counters
- DSP trigger objects (`SchmittTrigger`, `PulseGenerator`)
- Data structures for UI display (flash states, current index)

### Relationships
- `PsySeqWidget` owns and manages `PSGridWidget` and `PSStatusWidget`
- `PsySeq` maintains state and processes audio samples
- Custom parameter quantities override value setting behavior to enforce discrete values

### Module Organization
The module is organized into:
1. Core logic in `PsySeq` class
2. UI components in anonymous namespace
3. Widget layout in `PsySeqWidget` class
4. Parameter definitions and configuration
5. Input/output handling and signal processing pipeline

## 3. DSP Theory

### Algorithms
- **Step sequencing**: Implements a state machine to traverse sequence steps based on direction settings
- **Clock synchronization**: Uses Schmitt triggers for reliable clock detection
- **Randomization**: Utilizes `random::uniform()` for generating uniform random numbers in [0,1)
- **Mutation**: Periodic modification of step voltages with configurable probability and wait cycles

### Filters
None explicitly implemented.

### Oscillators
None explicitly implemented.

### Delay Lines
None explicitly implemented.

### Feedback Systems
None explicitly implemented.

### Modulation
- Voltage modulation through 64 knob parameters (0.0 to 1.0)
- Direction switching via parameter input
- Random voltage generation mode

### Random Processes
The module implements a random process for:
1. **Step voltage generation** when `RANDOM_VOLTAGE_PARAM` is enabled
2. **Mutation attempts** with configurable probability
3. **Random direction selection** when `DIRECTION_PARAM` is set to "Random"

## 4. Parameters and Controls

### Knobs
- **STEPS_PARAM**: 1 to 64 steps, default 8
- **START_PARAM**: 1 to 64 start step, default 1
- **MUTATION_PARAM**: Mutation probability (0.0 to 1.0), default 0.0
- **MUTATION_WAIT_PARAM**: Wait cycles before mutation attempt (0, 4, 8, 16), default 0

### Switches
- **DIRECTION_PARAM**: Direction selection (0 = Forward, 1 = Reverse, 2 = Random)
- **RANDOM_VOLTAGE_PARAM**: Random voltage per step mode (0 = Off, 1 = On)

### Buttons
- **RANDOMIZE_BUTTON**: Manual randomize all steps button

### CV Inputs
- **CLOCK_INPUT**: Clock signal for advancing sequence
- **RESET_INPUT**: Reset position and cycle counter
- **RANDOMIZE_TRIGGER_INPUT**: External trigger to randomize all steps

### Gates
- **CYCLE_GATE_OUTPUT**: Gate output triggered at end of cycle (10V when active)
- **STEP_TRIGGER_OUTPUT**: Trigger output on each step (10V when active)

### Enumerations
- `ParamId`: Parameter identifiers for all controls
- `InputId`: Input identifiers
- `OutputId`: Output identifiers
- `LightId`: Light identifiers

### Parameter Ranges and Default Values
| Parameter | Range | Default |
|-----------|-------|---------|
| STEPS_PARAM | 1 to 64 | 8 |
| START_PARAM | 1 to 64 | 1 |
| DIRECTION_PARAM | 0 to 2 | 0 (Forward) |
| RANDOM_VOLTAGE_PARAM | 0 to 1 | 0 (Off) |
| MUTATION_PARAM | 0.0 to 1.0 | 0.0 |
| MUTATION_WAIT_PARAM | {0, 4, 8, 16} | 0 |

## 5. Inputs and Outputs

### Inputs
- **CLOCK_INPUT**: Voltage input for clock signal (typically 0V or 10V)
- **RESET_INPUT**: Voltage input to reset sequence position and cycle counter
- **RANDOMIZE_TRIGGER_INPUT**: Voltage trigger input to randomize all step voltages

### Outputs
- **OUTPUT_OUT**: CV output with current step voltage
- **CYCLE_GATE_OUTPUT**: Gate output (10V when cycle completes)
- **STEP_TRIGGER_OUTPUT**: Trigger output (10V on each step)
- **INV_OUTPUT**: Inverted CV output (voltMin + voltMax - heldVoltage)

### Voltage Ranges
- All CV inputs: 0.0 to 10.0 volts
- All CV outputs: -10.0 to 10.0 volts (configurable voltage range)
- Gate outputs: 0.0 or 10.0 volts

## 6. Signal Processing Pipeline

1. **Input processing**: Clock, reset, and randomize triggers are processed using Schmitt triggers
2. **State update**: Sequence position is updated based on clock input and direction setting
3. **Step voltage calculation**: Current step voltage computed from knob values or random generation
4. **Mutation logic**: Periodic mutation attempt when cycle completes and wait cycles have elapsed
5. **Output generation**: CV output, inverted CV, gate, and trigger outputs are set based on current state
6. **UI updates**: Display data (current index, flash states) updated for visualization

## 7. Important Functions

- `process()`: Main audio processing function that handles clocking, reset, randomization, and output generation
- `computeVoltageForStep()`: Calculates voltage for a given step based on mode (knob or random)
- `updateHeldVoltage()`: Updates the current held voltage value
- `randomizeAllStepKnobs()`: Sets all 64 step parameters to random values
- `clearAllStepKnobs()`: Sets all 64 step parameters to zero
- `mapIndex()`: Maps sequence position to actual knob index with start step consideration
- `dataToJson()` and `dataFromJson()`: Serialization for saving/loading module state

## 8. Mathematical Implementation

### Voltage Calculation
$$ V_{step} = \begin{cases}
\text{random}(0,1) \times (V_{max} - V_{min}) + V_{min} & \text{if random mode} \\
V_{min} + K_i \times (V_{max} - V_{min}) & \text{otherwise}
\end{cases} $$

Where:
- $K_i$ is the knob value for step $i$
- $V_{min}, V_{max}$ are configurable voltage range limits

### Direction Logic
$$ \text{next\_pos} = \begin{cases}
(\text{pos} + 1) \bmod \text{steps} & \text{if forward (direction = 0)} \\
(\text{pos} - 1 + \text{steps}) \bmod \text{steps} & \text{if reverse (direction = 1)} \\
\lfloor \text{random}(0,1) \times \text{steps} \rfloor & \text{if random (direction = 2)}
\end{cases} $$

### Mutation Probability
Mutation occurs with probability $P_{mut}$ when:
$$ \text{cycleCounter} \geq \text{waitCycles} \quad \text{or} \quad \text{waitCycles} = 0 $$

## 9. Performance and Optimization

### CPU Usage Considerations
- Uses efficient state machine for step sequencing
- Minimal floating-point operations per sample
- Optimized trigger detection with Schmitt triggers
- Limited memory usage (64 parameters, small state variables)

### Memory Allocation
- Static allocation for all parameters and state variables
- No dynamic memory allocation in audio thread
- Preallocated arrays for UI display data

### Buffers
- Uses `dsp::PulseGenerator` for gate/trigger outputs
- No large buffers required

### Real-time Processing
- All processing occurs within the audio thread
- No blocking operations or external synchronization
- Efficient use of VCV Rack's built-in DSP utilities

### Thread Safety
- Module state is accessed only from audio thread
- UI thread reads display data (torn reads are cosmetic)
- No explicit thread synchronization required

### Numerical Stability
- Uses standard floating-point arithmetic
- Proper handling of modulo operations for wraparound
- Clamp values to valid ranges where necessary

## 10. Dependencies

### Libraries and Headers
- `rack.hpp`: VCV Rack core library
- `<vector>`, `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`: Standard C++ libraries
- `dsp::SchmittTrigger`: VCV Rack DSP utilities
- `dsp::PulseGenerator`: VCV Rack DSP utilities
- `random::uniform()`: VCV Rack random number generation
- `json_t`: JSON serialization support

### Classes and Types
- `rack::engine::Module`
- `rack::engine::ParamQuantity`
- `rack::engine::ModuleWidget`
- `dsp::SchmittTrigger`
- `dsp::PulseGenerator`
- `window::Font`

## 11. Summary

PsySeq is a feature-rich step sequencer module for VCV Rack with an 8x8 grid layout supporting up to 64 steps. It provides forward, reverse, and random direction modes, along with configurable voltage ranges and random voltage generation per step. The module includes mutation capabilities that can periodically modify step voltages after complete cycle repetitions. Key features include:
- 64 programmable step knobs with LED indicators
- Configurable sequence length and start position
- Multiple direction modes (forward, reverse, random)
- Random voltage generation mode
- Mutation system with configurable probability and wait cycles
- Comprehensive UI with grid visualization and status display
- External trigger inputs for clock, reset, and randomization
- Multiple output signals including CV, gate, and trigger outputs
- Preset voltage range configurations
- Save/load functionality for module state