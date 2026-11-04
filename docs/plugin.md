<!--
AGMM AUTOMATIC DOCUMENTATION
Source: plugin.cpp
Generated: 2026-11-04 06:25
-->

# plugin

## 1. Overview

The `AGMM` VCV Rack plugin is a collection of audio signal processing modules designed for modular synthesizer environments. It provides a wide variety of effects, oscillators, sequencers, and utility modules that enable users to create complex soundscapes and dynamic musical compositions.

This plugin serves as an extension to the VCV Rack platform, offering both classic and experimental DSP algorithms in a modular format. The modules range from traditional reverb and delay effects to more abstract systems such as random walks, cellular automata, and chaotic oscillators.

## 2. Architecture

### Main Classes

The architecture is based on the standard VCV Rack module system:

- `Plugin`: Represents the plugin itself, managing model registration.
- `Model`: Abstract base class for each individual module.
- Individual module classes (e.g., `Reverb`, `PsyDelay`, etc.) inherit from `Module`.

### Inheritance

All modules inherit from `rack::Module`. Each specific module implements its own processing logic by overriding virtual functions like `process()`.

### Structures

The plugin uses the standard VCV Rack structure:
- `Plugin *pluginInstance`: Global pointer to the plugin instance.
- `Model *modelXXX`: Pointers to each registered model, defined in `plugin.hpp`.

### Relationships Between Classes

- The `Plugin` class owns and manages a list of `Model` instances.
- Each `Model` corresponds to one specific module type.
- Modules are instantiated via factory functions provided by their respective models.

### Module Organization

Modules are organized into categories:
- **Reverb**: `Reverb`, `BlackHoleReverb`, `DarkHall`, `PsyReverb`, `InfinityRev`, `PsyReverb2`
- **Delay**: `SickDelay`, `PsyDelay`, `Stagger`, `Stagger2`
- **Oscillators & LFOs**: `PsyLFO`, `Oscillator`, `LSDrone`, `Pharmakon`
- **Sequencers & Arpeggiators**: `PsySeq`, `ExpressArpeggiator`
- **Generators & Random Systems**: `RandomWalk`, `BouncingBalls`, `Dice`, `DiceOnSteroids`, `RainStorm`, `Lifeform`, `Spikes`
- **Mixers & Routing**: `Mixer6`, `Mixer3`, `Swapper`, `Merger`, `Splitter`, `SwitchLines8to1`, `SwitchLines1to8`
- **Filters & Utilities**: `Range`, `VCA2`, `VCAPoly`, `Sampler`, `Whirlpool`, `Vortex`, `HeatRev`, `Ambience`, `CellAutoGate`, `Gates`, `Bastion`, `Phasmal`, `Stasis`, `Quantara`, `MagicBox`

## 3. DSP Theory

This plugin does not implement generic DSP theory but rather specific algorithms implemented in individual modules.

### Algorithms Implemented

Each module implements its own unique algorithm:

- **Reverb**: Implements various reverberation techniques including comb filters and allpass networks.
- **Delay**: Uses feedback delay lines with modulation capabilities.
- **Oscillators**: Includes sine, triangle, sawtooth, square waveforms with phase modulation.
- **Random Walks**: Simulates Brownian motion or random walks in discrete time steps.
- **Cellular Automata**: Implements 1D and 2D cellular automaton rules (e.g., Conway's Game of Life).
- **Chaotic Systems**: Models chaotic behavior using systems like the Lorenz attractor or spring pendulum dynamics.

### Filters

Some modules include basic filtering:
- Low-pass, high-pass, band-pass filters
- State-variable filters
- Comb and notch filters

### Oscillators

Oscillators are implemented with:
- Sine wave generation (using lookup tables)
- Triangle, sawtooth, square waveforms
- Phase modulation support
- Frequency modulation via CV inputs

### Delay Lines

Delay lines use:
- Fixed-length buffers
- Variable delay times
- Feedback mechanisms
- Modulation via LFOs or other sources

### Feedback Systems

Feedback is used in:
- Reverb algorithms
- Delay feedback loops
- Chaotic oscillator systems

### Modulation

Modulation includes:
- LFO modulation of parameters (frequency, delay time, etc.)
- CV-controlled parameter changes
- Random modulation sources

### Random Processes

Random processes include:
- White noise generators
- Brownian motion simulation
- Pseudo-random number generation for sequence control

## 4. Parameters and Controls

The plugin does not define parameters directly in `plugin.cpp`. Instead, each module defines its own controls in their respective source files.

### Knobs

Each module may have:
- Frequency knobs (e.g., oscillator frequency)
- Resonance/quality factor knobs
- Delay time knobs
- Feedback amount knobs
- Mix level knobs
- Modulation depth knobs

### Switches

Switches include:
- Waveform selection switches
- Filter mode switches
- Routing switches
- Gate enable/disable switches

### Buttons

Buttons may control:
- Reset functions
- Trigger inputs
- Mode toggles
- Sequence restarts

### CV Inputs

CV inputs are used for:
- Frequency modulation
- Amplitude control
- Delay time modulation
- Feedback amount modulation
- Trigger inputs

### Gates

Gate inputs are used for:
- Triggering events (e.g., note on/off)
- Enabling/disabling modules
- Controlling sequencer steps

### Enumerations

Enumerations define:
- Waveform types (sine, triangle, etc.)
- Filter modes (low-pass, high-pass, band-pass)
- Sequencer modes
- Automaton rules

### Parameter Ranges

Parameter ranges vary by module but typically include:
- Frequency: 0.1 Hz to 20 kHz
- Delay time: 0 ms to several seconds
- Feedback: 0% to 100%
- Modulation depth: 0% to 100%

### Default Values

Default values are set per module and vary based on intended use:
- Oscillator frequency defaults to ~440 Hz
- Delay time defaults to ~50 ms
- Feedback defaults to ~50%
- Filter cutoff defaults to ~1 kHz

## 5. Inputs and Outputs

### Input Types

Each module defines its own inputs, typically including:

#### CV Inputs
- Frequency modulation input (e.g., 0–10V)
- Amplitude modulation input (e.g., 0–10V)
- Delay time modulation input (e.g., 0–10V)

#### Gate Inputs
- Trigger input for note-on events
- Gate input for enabling/disabling modules

#### Audio Inputs
- Audio signal input for processing (e.g., in delay or reverb modules)

### Output Types

Each module defines its own outputs, typically including:

#### Audio Outputs
- Main processed audio output
- Sidechain input/output for ducking effects

#### CV Outputs
- Envelope outputs from ADSR or sequencer modules
- Trigger outputs from sequencers or event generators

## 6. Signal Processing Pipeline

The signal processing pipeline varies by module but generally follows these steps:

1. **Input Reception**: Audio or control signals are received through input jacks.
2. **Parameter Reading**: Module parameters are read from knobs, switches, and CV inputs.
3. **Signal Processing**: The core DSP algorithm is applied to the input signal.
4. **Modulation Application**: Any modulation sources (LFOs, envelopes) are applied.
5. **Output Generation**: Processed signals are sent to output jacks.

## 7. Important Functions

### `init(Plugin *p)`

This function initializes the plugin and registers all available models with VCV Rack. It is called once during plugin loading.

```cpp
void init(Plugin *p)
{
	pluginInstance = p;
	p->addModel(modelReverb);
	p->addModel(modelBlackHoleReverb);
	// ... additional model registrations ...
}
```

### `pluginInstance`

A global pointer to the current plugin instance, used for accessing plugin-wide resources.

## 8. Mathematical Implementation

The mathematical implementation varies by module and is not defined in this file. However, some common implementations include:

### Reverb Algorithms
- Comb filters: $ y[n] = x[n] + g \cdot y[n - D] $
- Allpass filters: $ y[n] = -g \cdot x[n] + g \cdot y[n - D] + x[n - D] $

### Delay Line
- Fixed delay buffer with interpolation:
  $$
  y(t) = x(t - D)
  $$

### Oscillator
- Sine wave generation using lookup tables or direct computation:
  $$
  y(t) = \sin(2\pi f t + \phi)
  $$

### Random Walk
- Discrete random walk process:
  $$
  x_{n+1} = x_n + \Delta x_n, \quad \Delta x_n \sim \mathcal{N}(0, \sigma^2)
  $$

## 9. Performance and Optimization

### CPU Usage Considerations

Each module's performance depends on its complexity:
- Simple filters: Minimal CPU usage
- Reverb algorithms: Moderate to high CPU usage
- Chaotic systems: High CPU usage due to iterative calculations

### Memory Allocation

Memory is allocated per module instance:
- Buffers for delay lines and filter states
- Lookup tables for waveform generation
- State variables for feedback systems

### Buffers

Modules use fixed-size buffers for:
- Delay line storage
- Filter state storage
- Audio processing buffers

### Real-Time Processing

All modules are designed to operate in real-time within VCV Rack's constraints.

### Thread Safety

VCV Rack ensures thread safety by running each module's `process()` function on the audio thread.

### Numerical Stability

Numerical stability is ensured through:
- Proper buffer sizing
- Clamp values to prevent overflow
- Use of fixed-point or floating-point arithmetic as appropriate

## 10. Dependencies

The plugin depends on:

- **VCV Rack SDK**: Core libraries for module development.
- **Standard C++ Libraries**: For basic operations like math functions, containers, etc.
- **Rack::Module**: Base class for all modules.
- **Rack::Model**: Factory registration mechanism.

## 11. Summary

The AGMM VCV Rack plugin is a comprehensive collection of audio processing modules that extend the capabilities of the VCV Rack platform. It includes a wide range of effects, oscillators, generators, and sequencers, each implementing specific DSP algorithms tailored for modular synthesis environments. The plugin is structured around the standard VCV Rack architecture with individual modules inheriting from `Module` and registered via `init()`. Each module implements its own unique signal processing pipeline, supporting various modulation sources, CV inputs, and gate triggers. Performance is optimized for real-time operation within VCV Rack's constraints, using efficient buffer management and numerical methods appropriate to each algorithm.