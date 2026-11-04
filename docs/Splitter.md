<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Splitter.cpp
Generated: 2026-11-04 06:33
-->

# Splitter

## 1. Overview

The **Splitter** module is a polyphonic signal distributor that takes a single polyphonic input and splits it into up to 16 individual mono outputs. It allows users to route multi-channel audio signals to separate output channels, making it useful for signal routing in modular synthesizer systems.

## 2. Architecture

### Main Classes
- `Splitter`: The core module class inheriting from `rack::Module`
- `SLPanelBackground`: Custom widget for drawing the panel background and layout elements
- `SLDisplayWidget`: Custom widget for displaying channel count information
- `SplitterWidget`: Module widget class that constructs and positions all UI elements

### Inheritance
- `Splitter` inherits from `rack::Module`
- Both custom widgets inherit from `rack::TransparentWidget`

### Structures
- Enumerations for parameters, inputs, outputs, and lights
- Layout constants for positioning UI elements
- Custom drawing functions using NanoVG for rendering

### Relationships
- The `SplitterWidget` owns the `Splitter` module instance
- The `SLPanelBackground` and `SLDisplayWidget` are child widgets of `SplitterWidget`
- All output ports are positioned in a 4x4 grid layout

### Module Organization
The module is organized as a single C++ source file (`Splitter.cpp`) with:
- Module definition and implementation
- Custom drawing code in anonymous namespace
- Widget construction logic
- Panel layout constants

## 3. DSP Theory

This module performs **signal routing** rather than signal processing. It implements a simple **polyphonic signal splitting algorithm**:

$$
V_{out,i} = 
\begin{cases}
V_{in,i} & \text{if } i < N \\
0 & \text{otherwise}
\end{cases}
$$

Where:
- $ V_{out,i} $: Output voltage for channel $ i $
- $ V_{in,i} $: Input voltage for channel $ i $
- $ N $: Number of active channels in the input signal

The module does not implement any filters, oscillators, delay lines, feedback systems, modulation, or random processes. It simply routes signals from one source to multiple destinations.

## 4. Parameters and Controls

### Knobs
None

### Switches
None

### Buttons
None

### CV Inputs
- **POLY_INPUT**: Polyphonic input (16-channel)

### Gates
None

### Enumerations
- `ParamId`: Empty enumeration (no parameters)
- `InputId`: 
  - `POLY_INPUT`
- `OutputId`: 
  - `SPLIT_OUTPUT` through `SPLIT_OUTPUT + 15` (16 outputs total)
- `LightId`: Empty enumeration (no lights)

### Parameter Ranges
None

### Default Values
None

## 5. Inputs and Outputs

### Input
- **POLY_INPUT**: 
  - Voltage range: ±10V (standard VCV Rack convention)
  - Signal behavior: Polyphonic audio input with up to 16 channels
  - Connection type: 3.5mm jack

### Outputs
- **SPLIT_OUTPUT + i** (16 outputs total):
  - Voltage range: ±10V (standard VCV Rack convention)
  - Signal behavior: Mono output for each channel
  - Connection type: 3.5mm jack
  - Layout: 4x4 grid positioned at specific coordinates

## 6. Signal Processing Pipeline

1. **Input detection**: Check if polyphonic input is connected
2. **Channel count determination**: Get number of channels from input signal
3. **Signal routing**: For each of the 16 output channels:
   - Set channel count to 1 (mono)
   - If channel index < input channel count: set voltage to input voltage at that channel
   - Otherwise: set voltage to 0V

## 7. Important Functions

- `Splitter::process()`: Main processing function that routes polyphonic input to individual outputs
- `SLPanelBackground::draw()`: Draws the complete panel background with layout elements
- `SLDisplayWidget::drawLayer()`: Renders the channel count display with visual indicators
- `SplitterWidget::SplitterWidget()`: Constructor that builds the module UI

## 8. Mathematical Implementation

The implementation uses simple mathematical operations:
- Channel counting via `inputs[POLY_INPUT].getChannels()`
- Voltage assignment using `inputs[POLY_INPUT].getVoltage(i)`
- Conditional assignment with ternary operator: `i < ch ? inputs[POLY_INPUT].getVoltage(i) : 0.f`

No complex mathematical functions or algorithms are implemented.

## 9. Performance and Optimization

### CPU Usage Considerations
- Minimal CPU usage as it only performs simple routing operations
- No expensive DSP calculations or filtering

### Memory Allocation
- Stack-based allocation for local variables
- No dynamic memory allocation in the main processing loop

### Buffers
- Uses VCV Rack's built-in buffer management for audio signals
- No custom buffers required

### Real-time Processing
- Optimized for real-time operation with minimal latency
- Simple O(n) algorithm where n is 16 (number of outputs)

### Thread Safety
- Depends on VCV Rack's thread safety guarantees for module processing
- No explicit synchronization needed

### Numerical Stability
- Uses standard floating-point arithmetic
- No numerical instability concerns due to simple operations

## 10. Dependencies

### Libraries
- `rack.hpp`: VCV Rack core library
- `NanoVG`: For custom drawing operations

### Classes and Headers
- `rack::Module`: Base module class
- `rack::TransparentWidget`: Base widget class for custom UI elements
- `rack::DrawArgs`: Drawing context arguments
- `rack::Vec`, `rack::NVGcontext`: Vector and graphics context types
- `std::shared_ptr<window::Font>`: Font loading utilities

### External Dependencies
- VCV Rack 2 framework
- System font file: `ShareTechMono-Regular.ttf`

## 11. Summary

The **Splitter** module is a simple yet essential routing component that distributes polyphonic audio signals to up to 16 individual mono outputs. It implements basic signal routing without any DSP processing, making it efficient and suitable for real-time modular synthesis applications. The module features a clean, professional UI with channel count visualization and follows VCV Rack's standard conventions for input/output handling and panel design.