<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Merger.cpp
Generated: 2026-11-04 06:22
-->

# Merger

## 1. Overview

The `Merger` module is a polyphonic signal combiner for VCV Rack. It accepts up to 16 mono audio inputs and merges them into a single polyphonic output. The module dynamically adjusts the number of channels in the output based on which input channels are connected, with unconnected inputs contributing 0V to the merged signal.

## 2. Architecture

### Main Classes

- **`Merger`**: Core module class inheriting from `rack::Module`
- **`MergerWidget`**: UI widget class inheriting from `rack::ModuleWidget`
- **`MGPanelBackground`**: Custom panel drawing class inheriting from `rack::TransparentWidget`
- **`MGDisplayWidget`**: Channel count display widget inheriting from `rack::TransparentWidget`

### Inheritance

- `Merger` → `Module`
- `MergerWidget` → `ModuleWidget`
- `MGPanelBackground`, `MGDisplayWidget` → `TransparentWidget`

### Structures

- **Parameter/Control Enumerations**:
  - `ParamId`: Empty (no parameters)
  - `InputId`: `MERGE_INPUT` enum with 16 entries
  - `OutputId`: `POLY_OUTPUT`
  - `LightId`: Empty (no lights)

### Relationships

- `MergerWidget` owns and manages all UI components including:
  - Panel background (`MGPanelBackground`)
  - Channel display (`MGDisplayWidget`)
  - Input jacks (16 total)
  - Output jack
- `Merger` holds state information:
  - `channels`: Current number of active channels
  - `active[16]`: Boolean array tracking which inputs are connected

### Module Organization

The module is organized into:
1. Core DSP logic in `Merger::process()`
2. UI drawing components in anonymous namespace
3. Widget instantiation in `MergerWidget` constructor

## 3. DSP Theory

The Merger performs a simple signal routing operation without any signal processing or mathematical transformation.

### Algorithms

- **Channel Detection**: Iterates through all 16 inputs to determine the highest connected input channel.
- **Signal Routing**: For each active channel, copies the voltage from the corresponding input to the output.

### Filters/Oscillators/Delay Lines/Feedback Systems

None implemented. This is a pure routing module.

### Modulation/Random Processes

No modulation or random processes present.

## 4. Parameters and Controls

### Knobs/Switches/Buttons

- **None**: The module has no user-controllable parameters.

### CV Inputs/Gates

- **16 Input Channels** (`MERGE_INPUT + i` where `i = 0..15`):
  - Each input is a mono audio jack
  - Voltage range: ±12V (standard VCV Rack audio range)
  - Behavior: Connected inputs contribute their signal to the output

### Enumerations

- `InputId`: `MERGE_INPUT + i` for i = 0..15
- `OutputId`: `POLY_OUTPUT`

### Parameter Ranges/Default Values

- **No parameters**: All values are determined by input connection status.

## 5. Inputs and Outputs

### Inputs

| Name | Type | Voltage Range | Behavior |
|------|------|---------------|----------|
| `MERGE_INPUT + i` (i = 0..15) | Mono Audio | ±12V | Input channel; connected inputs contribute their signal to output |

### Outputs

| Name | Type | Voltage Range | Behavior |
|------|------|---------------|----------|
| `POLY_OUTPUT` | Polyphonic Audio | ±12V | Output channel count determined by highest connected input |

## 6. Signal Processing Pipeline

1. **Input Connection Detection**:
   - Iterate through all 16 inputs
   - Check if each input is connected using `isConnected()`
   - Update `active[i]` array accordingly
   - Track the highest connected input index (`last`)

2. **Channel Count Determination**:
   - Set `channels = last` (number of active channels)
   - If no inputs are connected, `channels = 0`

3. **Output Configuration**:
   - Call `outputs[POLY_OUTPUT].setChannels(last)` to configure polyphonic output
   - For each active channel `i`, call `outputs[POLY_OUTPUT].setVoltage(inputs[MERGE_INPUT + i].getVoltage(), i)`

## 7. Important Functions

### `Merger::process()`
- **Purpose**: Main DSP processing function
- **Responsibility**:
  - Detects connected inputs
  - Updates channel count
  - Routes signals from inputs to output

### `MergerWidget::MergerWidget()`
- **Purpose**: UI initialization
- **Responsibility**:
  - Sets up module reference
  - Creates and positions all UI components
  - Adds input/output jacks

### `MGDisplayWidget::drawLayer()`
- **Purpose**: Custom display rendering
- **Responsibility**:
  - Shows current channel count
  - Visualizes active/inactive channels with segmented bar

## 8. Mathematical Implementation

No mathematical operations are performed. The implementation is purely logical:

```cpp
void process(const ProcessArgs& args) override {
    int last = 0;
    for (int i = 0; i < 16; ++i) {
        bool c = inputs[MERGE_INPUT + i].isConnected();
        active[i] = c;
        if (c) last = i + 1;
    }
    channels = last;

    outputs[POLY_OUTPUT].setChannels(last);
    for (int i = 0; i < last; ++i)
        outputs[POLY_OUTPUT].setVoltage(inputs[MERGE_INPUT + i].getVoltage(), i);
}
```

## 9. Performance and Optimization

### CPU Usage Considerations

- **Minimal**: Single loop through 16 inputs
- **Constant time complexity**: O(1) operation regardless of input state

### Memory Allocation

- **Static allocation**: No dynamic memory allocation
- **Fixed-size arrays**: `active[16]` array with known size

### Buffers

- **No buffers**: Direct signal routing without delay or storage

### Real-time Processing

- **Optimized for real-time**: Simple iteration and direct assignment
- **Thread safety**: Uses standard VCV Rack module threading model

### Numerical Stability

- **None applicable**: No numerical computation involved

## 10. Dependencies

### Libraries/Headers

- `rack.hpp`: Core VCV Rack API
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`: Standard C++ libraries
- `<memory>`: For `std::shared_ptr`

### Classes/Types

- `Module`, `ModuleWidget`, `TransparentWidget`: VCV Rack base classes
- `Input`, `Output`: VCV Rack signal handling classes
- `Vec`, `NVGcontext`, `NVGcolor`: VCV Rack graphics and vector types
- `APP`, `window::Font`: VCV Rack application context

## 11. Summary

The `Merger` module is a simple yet effective polyphonic signal combiner that dynamically adjusts its output channel count based on input connections. It accepts up to 16 mono inputs and routes them to a single polyphonic output, with unconnected inputs contributing 0V. The implementation is optimized for real-time performance with minimal CPU overhead and no mathematical operations beyond basic signal routing. The module features a custom UI with visual channel indication and follows standard VCV Rack conventions for module design and behavior.