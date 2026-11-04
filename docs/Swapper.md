<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Swapper.cpp
Generated: 2026-11-04 06:36
-->

# Swapper

## 1. Overview

The **Swapper** module is a stereo signal routing device designed for VCV Rack that allows users to swap or mix left and right audio signals based on configurable amounts. It enables bidirectional signal swapping between the left and right channels, with manual control knobs and CV inputs for dynamic adjustment.

It features:
- Manual control of L-to-R and R-to-L swap amounts.
- CV-controlled attenuverters for real-time modulation.
- Visual feedback showing current swap levels and signal strength.
- Stereo input/output routing with configurable mixing behavior.

## 2. Architecture

### Main Classes
- `Swapper`: The core module class inheriting from `rack::Module`.
- `SWPanel`: A custom widget for drawing the physical panel layout.
- `SWDisplay`: A custom widget for rendering the signal routing visualization.
- `SwapperWidget`: The UI wrapper that assembles all components.

### Inheritance
- `Swapper` inherits from `rack::Module`.
- `SWPanel`, `SWDisplay`, and `SwapperWidget` inherit from `rack::TransparentWidget`.

### Structures
- Enumerations define parameter IDs, input IDs, output IDs, and light IDs.
- Constants define layout dimensions (`SW_W`, `SW_H`, etc.) for consistent UI sizing.

### Relationships Between Classes
- `SwapperWidget` owns the `Swapper` module instance and manages child widgets like `SWPanel` and `SWDisplay`.
- `SWDisplay` reads data from the `Swapper` module to render visual feedback.
- The module's parameters, inputs, and outputs are configured via `configParam`, `configInput`, and `configOutput`.

### Module Organization
The module is organized into:
1. **Core DSP logic** in `process()` method.
2. **UI rendering code** in anonymous namespace functions (`drawPanelBase`, `drawBezel`, etc.).
3. **Widget hierarchy** defined in `SwapperWidget` constructor.

## 3. DSP Theory

The Swapper implements a simple **signal mixing operation** where:
- Each channel (L and R) receives input from both channels.
- The amount of signal transferred between channels is controlled by a mix factor.

### Mathematical Equations
Let:
- $ l_{in} $ = left input voltage
- $ r_{in} $ = right input voltage
- $ l_{out} $ = left output voltage
- $ r_{out} $ = right output voltage
- $ a_{LR} $ = amount of L to R signal (0–1)
- $ a_{RL} $ = amount of R to L signal (0–1)

The output voltages are computed as:
$$
l_{out} = (1 - a_{RL}) \cdot l_{in} + a_{RL} \cdot r_{in}
$$
$$
r_{out} = (1 - a_{LR}) \cdot r_{in} + a_{LR} \cdot l_{in}
$$

### Algorithms
- Linear interpolation between input and output signals.
- Clamp values to [0, 1] range after applying CV modulation.
- Exponential smoothing for display metering.

### Filters / Oscillators / Delay Lines / Feedback Systems
None implemented in this module.

### Modulation
- CV inputs modulate the amount of signal transfer via attenuverters.
- Attenuverter values are applied linearly to CV input voltages (scaled by 10V = 100%).

### Random Processes
None implemented.

## 4. Parameters and Controls

| Control | Type | Description | Range | Default |
|--------|------|-------------|-------|---------|
| `LTO_R_KNOB` | Knob | Manual control for L to R swap amount | 0–1 | 0 |
| `RTO_L_KNOB` | Knob | Manual control for R to L swap amount | 0–1 | 0 |
| `LTO_R_ATTEN` | Trimpot | CV attenuverter for L to R signal | -1 to +1 | 0 |
| `RTO_L_ATTEN` | Trimpot | CV attenuverter for R to L signal | -1 to +1 | 0 |

### Input IDs
| ID | Name | Description |
|----|------|-------------|
| `L_IN` | Left input | Stereo left input |
| `R_IN` | Right input | Stereo right input |
| `CV_LTOR` | CV for L to R | Modulates L to R amount (10V = 100%) |
| `CV_RTOL` | CV for R to L | Modulates R to L amount (10V = 100%) |

### Output IDs
| ID | Name | Description |
|----|------|-------------|
| `L_OUT` | Left output | Stereo left output |
| `R_OUT` | Right output | Stereo right output |

## 5. Inputs and Outputs

### Inputs
- **L_IN (Left Input)**: Voltage range ±10V, stereo input.
- **R_IN (Right Input)**: Voltage range ±10V, stereo input.
- **CV_LTOR (CV for L to R)**: Voltage range ±10V, modulates the L to R swap amount.
- **CV_RTOL (CV for R to L)**: Voltage range ±10V, modulates the R to L swap amount.

### Outputs
- **L_OUT (Left Output)**: Voltage range ±10V, stereo output.
- **R_OUT (Right Output)**: Voltage range ±10V, stereo output.

## 6. Signal Processing Pipeline

1. Read input voltages from `L_IN` and `R_IN`.
2. Retrieve knob values for manual swap amounts (`LTO_R_KNOB`, `RTO_L_KNOB`).
3. Read CV inputs (`CV_LTOR`, `CV_RTOL`) and scale them to 0–1 range (divided by 10V).
4. Apply attenuverter settings to CV signals.
5. Combine manual and CV amounts, clamp result to [0, 1].
6. Compute output voltages using weighted sum equations.
7. Set output voltages on `L_OUT` and `R_OUT`.
8. Update metering values (`mInL`, `mInR`, `mOutL`, `mOutR`) for UI display.

## 7. Important Functions

- `process()`: Main DSP processing function that calculates output signals.
- `drawPanelBase()`: Draws the base panel with gradient background and logo.
- `drawBezel()`: Draws bezel around control area.
- `SWDisplay::drawLayer()`: Renders signal routing visualization including:
  - Lines showing signal flow with varying thickness.
  - Nodes indicating input/output levels.
  - Text labels for current swap percentages.

## 8. Mathematical Implementation

### Core Mixing Equations
```cpp
float lOut = (1.f - rtoL_amount) * lIn + rtoL_amount * rIn;
float rOut = (1.f - ltoR_amount) * rIn + ltoR_amount * lIn;
```

### CV Modulation
```cpp
float ltoR_cv = inputs[CV_LTOR].getVoltage() / 10.f;
float rtoL_cv = inputs[CV_RTOL].getVoltage() / 10.f;
float ltoR_atten = params[LTO_R_ATTEN].getValue();
float rtoL_atten = params[RTO_L_ATTEN].getValue();

float ltoR_amount = clamp(ltoR_manual + ltoR_atten * ltoR_cv, 0.f, 1.f);
float rtoL_amount = clamp(rtoL_manual + rtoL_atten * rtoL_cv, 0.f, 1.f);
```

### Metering (Exponential Smoothing)
```cpp
float md = 1.f - 5.f * args.sampleTime;
mInL = std::max(std::fabs(lIn), mInL * md);
mInR = std::max(std::fabs(rIn), mInR * md);
mOutL = std::max(std::fabs(lOut), mOutL * md);
mOutR = std::max(std::fabs(rOut), mOutR * md);
```

## 9. Performance and Optimization

### CPU Usage Considerations
- Simple arithmetic operations; minimal CPU load.
- No complex filters or oscillators.

### Memory Allocation
- Uses stack variables only.
- No dynamic memory allocation.

### Buffers
- None required for core functionality.

### Real-time Processing
- All processing done in `process()` which runs at audio sample rate.
- Uses standard VCV Rack real-time constraints.

### Thread Safety
- Module data is accessed by audio thread (`process`) and UI thread (`drawLayer`).
- Metering values (`mInL`, etc.) are written by audio thread, read by UI thread.
- No synchronization primitives used; assumes single-threaded access to UI data.

### Numerical Stability
- Clamp results to [0, 1] range to prevent overflow.
- Exponential smoothing avoids numerical instability in meter updates.

## 10. Dependencies

### Libraries
- VCV Rack SDK (`rack.hpp`)
- NVG (NanoVG for rendering)

### Classes/Headers
- `Module`, `ParamId`, `InputId`, `OutputId`, `LightId`
- `ProcessArgs`
- `TransparentWidget`
- `DrawArgs`
- `Vec`, `NVGcontext`, `NVGcolor`

## 11. Summary

The **Swapper** module is a stereo signal routing device that allows bidirectional swapping of left and right audio channels with manual controls and CV modulation. It implements a straightforward mixing algorithm using weighted sums, where the amount of signal transfer between channels can be adjusted via knobs or CV inputs.

Key features include:
- Two independent swap controls (L→R and R→L).
- CV attenuverters for real-time modulation.
- Visual feedback showing current swap levels and signal strength.
- Efficient DSP implementation with minimal CPU usage.
- Clean UI rendering using NanoVG.

The module supports standard VCV Rack stereo I/O, with voltage ranges up to ±10V. It uses exponential smoothing for metering to provide smooth visual feedback without flickering.