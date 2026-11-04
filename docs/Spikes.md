<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Spikes.cpp
Generated: 2026-11-04 06:32
-->

# Spikes

## 1. Overview

The **Spikes** module is a VCV Rack plugin that generates chaotic attractor signals using the Lorenz and Rössler systems, with additional fractal noise modulation. It outputs three-dimensional coordinates (X, Y, Z) along with a mixed signal (MIX). The module features a 3D visualization display showing the trajectory of the attractor in real-time.

## 2. Architecture

### Main Classes
- `Spikes`: Core module class inheriting from `rack::Module`
- `SPPanel`: UI panel widget for the module's front panel
- `SPDisplay`: 3D visualization display widget
- `Octave`: Structure for managing fractional Brownian motion octaves

### Inheritance
- `Spikes` inherits from `rack::Module`

### Structures
- `Octave`: Contains state for each octave in fractal noise generation:
  - `current`: Current random value
  - `target`: Next random value
  - `phase`: Interpolation phase (0.0 to 1.0)
  - `initialized`: Boolean flag indicating initialization status

### Relationships
- `Spikes` contains all state variables and processing logic
- `SPDisplay` reads data from `Spikes` for visualization
- `Octave` instances are part of `Spikes`'s internal state

### Module Organization
The module is organized into:
1. Parameter definitions (knobs, switches)
2. Input/output definitions
3. State variables for chaotic attractors and fractal noise
4. DSP processing functions
5. UI components for display and panel layout

## 3. DSP Theory

### Mathematical Equations

#### Lorenz Attractor
The Lorenz system is defined by:
$$
\begin{aligned}
\frac{dx}{dt} &= \sigma(y - x) \\
\frac{dy}{dt} &= x(\rho - z) - y \\
\frac{dz}{dt} &= xy - \beta z
\end{aligned}
$$

Where:
- $\sigma = 10$
- $\rho = 28$ 
- $\beta = \frac{8}{3}$

#### Rössler Attractor
The Rössler system is defined by:
$$
\begin{aligned}
\frac{dx}{dt} &= -y - z \\
\frac{dy}{dt} &= x + ay \\
\frac{dz}{dt} &= b + z(x - c)
\end{aligned}
$$

Where:
- $a = 0.2$
- $b = 0.2$
- $c = 5.7$

### Algorithms
1. **Runge-Kutta 4th Order Integration (RK4)**: Used to numerically integrate the attractor differential equations
2. **Fractional Brownian Motion (fBm)**: Implemented using multiple octaves of interpolated random values with Hurst exponent $H = 0.65$
3. **Smoothstep Interpolation**: For smooth transitions between random values in fractal noise generation

### Filters and Oscillators
- No explicit filters or oscillators beyond the chaotic attractors themselves
- The module uses a custom XORShift32 RNG for noise generation

### Delay Lines
- Uses a circular buffer (`trailX`, `trailY`, `trailZ`) of size 400 for storing attractor trajectory points
- Implements a time accumulator (`trailAcc`) to control trail point sampling rate

### Feedback Systems
- The chaotic attractors themselves are feedback systems with nonlinear dynamics
- The output signals feed back into the attractor state variables through integration

### Modulation
- All parameters can be modulated via CV inputs:
  - SPEED_INPUT: Modulates integration speed (1V/octave scaling)
  - BLEND_INPUT: Mixes between Lorenz and Rössler attractors (10V = 100%)
  - FRACTAL_INPUT: Controls fractal noise amount (10V = 100%)

### Random Processes
- Implements fractional Brownian motion using multiple octaves of interpolated random values
- Uses XORShift32 algorithm for high-quality uniform noise generation

## 4. Parameters and Controls

### Knobs
- **SPEED_PARAM** (0.01 to 10, default 1): Controls attractor integration speed
- **BLEND_PARAM** (0 to 1, default 0.5): Mixes between Lorenz and Rössler attractors
- **FRACTAL_PARAM** (0 to 1, default 0.2): Amount of fractal noise modulation
- **FRACTAL_SPEED_PARAM** (0.01 to 50, default 1): Speed of fractal noise generation
- **SCALE_PARAM** (0.1 to 10, default 5): Output signal scaling factor
- **ROTATE_PARAM** (-1 to 1, default 0.25): Display rotation speed

### Switches and Buttons
- None explicitly defined in the code

### CV Inputs
- **SPEED_INPUT**: Speed modulation (1V/octave)
- **BLEND_INPUT**: Blend modulation (10V = 100%)
- **FRACTAL_INPUT**: Fractal amount modulation (10V = 100%)

### Enumerations
- `ParamId`: Enum for parameter IDs
- `InputId`: Enum for input IDs  
- `OutputId`: Enum for output IDs
- `LightId`: Empty enum for light IDs

### Parameter Ranges and Default Values
| Parameter | Range | Default |
|-----------|-------|---------|
| SPEED_PARAM | 0.01 to 10 | 1 |
| BLEND_PARAM | 0 to 1 | 0.5 |
| FRACTAL_PARAM | 0 to 1 | 0.2 |
| FRACTAL_SPEED_PARAM | 0.01 to 50 | 1 |
| SCALE_PARAM | 0.1 to 10 | 5 |
| ROTATE_PARAM | -1 to 1 | 0.25 |

## 5. Inputs and Outputs

### Inputs
- **SPEED_INPUT**: Voltage input for speed modulation (range: -5V to +5V)
- **BLEND_INPUT**: Voltage input for blend modulation (range: -10V to +10V)  
- **FRACTAL_INPUT**: Voltage input for fractal amount modulation (range: -10V to +10V)

### Outputs
- **X_OUTPUT**: X coordinate signal (range: -10V to +10V)
- **Y_OUTPUT**: Y coordinate signal (range: -10V to +10V)
- **Z_OUTPUT**: Z coordinate signal (range: -10V to +10V)
- **MIX_OUTPUT**: Mixed signal (range: -10V to +10V)

## 6. Signal Processing Pipeline

1. **Parameter Processing**: Read and scale parameters with CV inputs
2. **Attractor Integration**: 
   - Integrate both Lorenz and Rössler attractors using RK4 method
   - Substep integration for numerical stability
3. **State Validation**: Reset attractor states if values become infinite
4. **Fractal Noise Generation**:
   - Compute fractal noise using multiple octaves of interpolated random values
   - Apply smoothstep interpolation between random values
5. **Signal Normalization**: Normalize attractor outputs to approximately [-1, 1]
6. **Blending**: Mix Lorenz and Rössler outputs based on blend parameter
7. **Fractal Mixing**: Add fractal noise to the blended signal
8. **Output Scaling**: Scale final signals by the SCALE parameter
9. **Trail Update**: Store current attractor position in circular buffer for display

## 7. Important Functions

### Core Processing
- `process()`: Main DSP processing function that integrates attractors, generates fractal noise, and outputs signals
- `computeFractal()`: Generates fractional Brownian motion using multiple octaves
- `rk4()`: Implements Runge-Kutta 4th order integration for differential equations

### Attractor Functions
- `lorenzDeriv()`: Computes derivatives for the Lorenz attractor system
- `rosslerDeriv()`: Computes derivatives for the Rössler attractor system

### Random Number Generation
- `xorshift()`: XORShift32 random number generator
- `randUnipolar()`: Generates uniform random values in [0, 1)
- `randBipolar()`: Generates uniform random values in [-1, 1)

### Reset Function
- `onReset()`: Initializes module state on reset

## 8. Mathematical Implementation

### Fractal Noise Generation
The implementation uses fractional Brownian motion with Hurst exponent $H = 0.65$:

$$
\text{fractal} = \sum_{i=0}^{N-1} \text{amp}_i \cdot \text{interpolate}(current_i, target_i, phase_i)
$$

Where:
- $\text{amp}_i = 2^{-H \cdot i}$ (amplitude decreases with octave)
- Interpolation uses smoothstep function: $s = t^2(3 - 2t)$

### Integration Method
The RK4 integration is implemented as:

$$
\begin{aligned}
k_1 &= f(x_n) \\
k_2 &= f(x_n + \frac{dt}{2} k_1) \\
k_3 &= f(x_n + \frac{dt}{2} k_2) \\
k_4 &= f(x_n + dt \cdot k_3) \\
x_{n+1} &= x_n + \frac{dt}{6}(k_1 + 2k_2 + 2k_3 + k_4)
\end{aligned}
$$

### Signal Normalization
$$
\begin{aligned}
\text{normalized}_x &= \frac{\text{attractor}_x}{20.0} \\
\text{normalized}_y &= \frac{\text{attractor}_y}{25.0} \\
\text{normalized}_z &= \frac{\text{attractor}_z - 25.0}{25.0}
\end{aligned}
$$

## 9. Performance and Optimization

### CPU Usage Considerations
- Uses RK4 integration which is computationally intensive but stable
- Implements substepping to maintain stability at higher speeds
- Fractal noise generation uses multiple octaves but with efficient interpolation

### Memory Allocation
- Pre-allocated arrays for attractor states and trail buffers
- No dynamic memory allocation during processing
- Trail buffer size fixed at 400 points

### Buffers
- Circular buffer of size 400 for storing attractor trajectory points
- Fixed-size arrays for octave state management

### Real-time Processing
- All processing done in the audio thread
- Uses efficient numerical methods and precomputed values
- Avoids expensive operations like division or trigonometric functions in critical paths

### Thread Safety
- Module state is accessed from audio thread only
- Display reads are safe (torn reads are cosmetic)
- No locking mechanisms required for basic operation

### Numerical Stability
- Uses RK4 integration for numerical stability
- Implements safety net to reset attractor states if values become infinite
- Limits fractal noise frequency to prevent aliasing
- Clamps all parameters to valid ranges

## 10. Dependencies

### Libraries and Headers
- `rack.hpp`: VCV Rack core library
- Standard C++ libraries: `<cmath>`, `<cstdio>`, `<algorithm>`
- VCV Rack UI components: `window::Font`, `nvg*` drawing functions

### Classes and Types
- `rack::Module`: Base module class
- `rack::ParamWidget`, `rack::InputWidget`, `rack::OutputWidget`: UI widgets
- `NVGcontext`, `NVGcolor`: NanoVG graphics context and color types
- `Vec`, `DrawArgs`, `ButtonEvent`, `DragMoveEvent`, `DoubleClickEvent`: VCV Rack geometry and event types

## 11. Summary

The **Spikes** module implements a chaotic signal generator using the Lorenz and Rössler attractors with fractional Brownian motion fractal noise modulation. It provides three-dimensional coordinate outputs (X, Y, Z) and a mixed output (MIX), along with a real-time 3D visualization of the attractor trajectory. The implementation uses RK4 integration for numerical stability, XORShift32 RNG for high-quality noise generation, and fractional Brownian motion for correlated random processes. The module features CV modulation for all parameters and includes safety mechanisms to prevent numerical instability.