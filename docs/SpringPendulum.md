<!--
AGMM AUTOMATIC DOCUMENTATION
Source: SpringPendulum.cpp
-->

# SpringPendulum

## 1. Overview

`SpringPendulum` is a 24 HP chaotic modulation source based on a physical spring-pendulum: a mass suspended by four linear springs anchored at the four compass points of a circular dial, with optional gravity, damping and a smooth random force. The mass is integrated in real time with RK4 at an adjustable simulation speed, and can be grabbed and dragged with the mouse.

Key features:
- 4-spring pendulum physics (RK4 integration)
- Mouse-grabbable mass
- X/Y position outputs (±5 V)
- Random V/Oct latched on X = 0 crossing
- Gate output on same crossing
- Speed, Mass, Rest Length, Spring K, Chaos, Damping, Gravity knobs
- Push button (random impulse) and Reset button
- Live 180×188 dial display with anchor posts, springs, bob and trail

## Module Screenshot

![SpringPendulum VCV Rack Module](images/springpendulum.png)

## 2. Architecture

### Main Classes

- `SpringPendulum`: Core module class inheriting from `rack::Module`
- `SPPanelBackground`: Static panel artwork
- `SPDisplayWidget`: Live dial display + mouse grab

### Inheritance

- `SpringPendulum` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### Structures

- `State { double x, y, vx, vy; }` — physics state
- `VoctRange`: Range0_1, RangeN1_1, RangeN2_2, RangeN5_5, Range0_5
- `GateDuration`: Trigger, 5ms, 10ms, 20ms, 50ms, 100ms, 200ms

### State (grouped)

| Group | Storage |
|-------|---------|
| Physics | `double x, y, vx, vy` |
| Params | `double mass, restLen, springK, speedFactor, damping, chaos, g` |
| Noise | `double noiseAx, noiseAy` |
| Mouse grab | `std::atomic<bool> grabbed`, `std::atomic<double> grabX, grabY`, `bool handOn`, `double handX, handY, handWn` |
| V/Oct | `double voctVoltage`, `prevXSign` |
| Gate | `int gateTimer` |
| Ranges | `VoctRange voctRange` |
| Gate durations | `GateDuration gateDuration` |
| Transport | `dsp::SchmittTrigger resetTrigger, pushTrigger`, `bool lastResetState` |
| RNG | `std::mt19937 rng`, `std::normal_distribution<double> gauss` |
| Display | `float trailX[120]`, `trailY[120]`, `trailHead`, `trailCount`, `trailTimer` |

## 3. DSP Theory

### Spring forces

The mass is connected to four anchor points (N, E, S, W) at distance D = 1.0 from the origin. Each spring's force is linear (Hooke's law):

$$
\mathbf{F}_i = k(| \mathbf{r}_i | - L_0) \cdot \frac{\mathbf{r}_i}{| \mathbf{r}_i |}
$$

where $\mathbf{r}_i = \mathbf{a}_i - \mathbf{p}$ is the vector from the mass to anchor $i$, $k$ is the spring constant, and $L_0$ is the rest length.

The total force on the mass is:

$$
\mathbf{F}_{\text{total}} = \sum_{i=1}^{4} \mathbf{F}_i
$$

### Equation of motion

$$
\ddot{\mathbf{p}} = \frac{\mathbf{F}_{\text{total}}}{m} - d \dot{\mathbf{p}} + \mathbf{g} + \mathbf{a}_{\text{noise}}
$$

where $m$ is the mass, $d$ is the damping, $\mathbf{g}$ is gravity (downward), and $\mathbf{a}_{\text{noise}}$ is the random force.

### Ornstein-Uhlenbeck noise

The random force is a **smooth** noise (Ornstein-Uhlenbeck process), not white noise:

$$
\mathbf{a}_{\text{noise}} \leftarrow \mathbf{a}_{\text{noise}} \cdot e^{-\Delta t/\tau} + \sigma \sqrt{1 - e^{-2\Delta t/\tau}} \cdot \mathbf{g}
$$

where $\tau = 0.12$ s, $\sigma = \text{chaos} \cdot 6$, and $\mathbf{g} \sim \mathcal{N}(0, 1)$. This gives a slowly-varying random force, which makes the motion more "organic" than pure white noise.

### RK4 integration

The state is advanced by `dt = sampleTime * speedFactor` (capped at 0.001) using the classical 4th-order Runge-Kutta.

### Soft wall

If the mass strays too far, it's clamped to a soft wall at r = 2.5 D:

$$
r > 2.5 \implies \mathbf{p} \leftarrow \frac{\mathbf{p}}{r} \cdot 2.5, \quad \text{remove outward velocity}
$$

This prevents the mass from flying away.

### Mouse grab

When the mouse grabs the mass, a critically-damped spring pulls the mass toward the cursor:

$$
\ddot{\mathbf{p}} + = \omega_n^2 (\mathbf{p}_{\text{cursor}} - \mathbf{p}) - 2\omega_n \dot{\mathbf{p}}
$$

where $\omega_n = 25 \cdot \text{speedFactor}$. The damping is critical (2·ω_n), so the mass follows the cursor without overshooting.

### Zero-crossing detection

When the mass crosses X = 0:

$$
\text{sign}(x_{n-1}) \ne \text{sign}(x_n) \implies \text{latch V/Oct, fire gate}
$$

### V/Oct ranges

Same as `PsyLFO`: 0..1, −1..1, −2..2, −5..5, 0..5.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `SPEED_PARAM` | 0 | 1 | 0.5 | Speed (0.2–10×) |
| `MASS_PARAM` | 0.1 | 10 | 1 | Mass (kg) |
| `LENGTH_PARAM` | 0.1 | 0.9 | 0.5 | Rest length (m) |
| `SPRING_K_PARAM` | 1 | 100 | 10 | Spring constant (N/m) |
| `CHAOS_PARAM` | 0 | 1 | 0.35 | Chaos (random force) |
| `DAMP_PARAM` | 0.02 | 2 | 0.3 | Damping (1/s) |
| `GRAVITY_PARAM` | 0 | 40 | 0 | Gravity (m/s²) |

### Buttons

| Button | Description |
|--------|-------------|
| `PUSH_BUTTON` | Random push |
| `RESET_BUTTON` | Reset |

### Inputs

| Input | Description |
|-------|-------------|
| `RESET_CV_INPUT` | Reset gate |
| `PUSH_CV_INPUT` | Push trigger |

### Outputs

| Output | Description |
|--------|-------------|
| `X_OUTPUT` | Mass X |
| `Y_OUTPUT` | Mass Y |
| `VOCT_OUTPUT` | V/Oct (random on X=0 crossing) |
| `GATE_OUTPUT` | Gate (trigger on V/Oct change) |

### Lights

- `MASS_LIGHT` — Current speed

### Enumerations

- `VoctRange`: Range0_1, RangeN1_1, RangeN2_2, RangeN5_5, Range0_5
- `GateDuration`: Trigger, 5ms, 10ms, 20ms, 50ms, 100ms, 200ms
- `ParamId`: Speed, Mass, Length, SpringK, Reset, PushStrength, Push, Chaos, Damp, Gravity
- `InputId`: ResetCV, PushCV
- `OutputId`: X, Y, VOct, Gate
- `LightId`: Mass

### Menu

- V/Oct range (5 options)
- Gate duration (7 options)

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `RESET_CV_INPUT` | Gate | > 1 V | Reset |
| `PUSH_CV_INPUT` | Gate | > 1 V | Random push |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `X_OUTPUT` | CV | ±5 V | Mass X |
| `Y_OUTPUT` | CV | ±5 V | Mass Y |
| `VOCT_OUTPUT` | CV | see range | Random V/Oct |
| `GATE_OUTPUT` | Gate | 0/10 V | Gate |

## 6. Signal Processing Pipeline

1. **Read params** — Speed, mass, rest length, spring K, chaos, damping, gravity.
2. **Reset detection** — Button or CV → reset state.
3. **Push detection** — Button or CV → add random impulse.
4. **Mouse grab** — Copy atomics to local.
5. **Compute `dt`** — `sampleTime * speedFactor`, capped at 0.001.
6. **Update noise** — OU process.
7. **RK4** — Integrate physics.
8. **NaN guard** — Reset if non-finite.
9. **Soft wall** — Clamp to 2.5 D.
10. **Zero-crossing detection** — Latch V/Oct, fire gate.
11. **Write outputs**.
12. **LED** — Speed brightness.
13. **Trail sampling**.

## 7. Important Functions

### `deriv(const State &s, State &d)`

Computes the derivative of the physics state.

### `rk4(double dt)`

RK4 integration step.

### `resetState()`

Randomises the initial state.

### `kick()`

Adds a random angular impulse.

### `generateRandomVoct()`

Generates a new random V/Oct value in the current range.

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Spring force

$$
\mathbf{F}_i = k(| \mathbf{r}_i | - L_0) \frac{\mathbf{r}_i}{| \mathbf{r}_i |}
$$

### Equation of motion

$$
\ddot{\mathbf{p}} = \frac{\sum_i \mathbf{F}_i}{m} - d \dot{\mathbf{p}} + \mathbf{g} + \mathbf{a}_{\text{noise}}
$$

### OU noise

$$
\mathbf{a}_{\text{noise}} \leftarrow \mathbf{a}_{\text{noise}} e^{-\Delta t/\tau} + \sigma \sqrt{1 - e^{-2\Delta t/\tau}} \mathbf{g}
$$

### Mouse spring

$$
\ddot{\mathbf{p}} + = \omega_n^2 (\mathbf{p}_{\text{cursor}} - \mathbf{p}) - 2\omega_n \dot{\mathbf{p}}
$$

### V/Oct ranges

| Range | Formula |
|-------|---------|
| 0..1 | $(u+1)/2$ |
| −1..1 | $u$ |
| −2..2 | $2u$ |
| −5..5 | $5u$ |
| 0..5 | $(u+1)/2 \cdot 5$ |

## 9. Performance and Optimization

### CPU Usage

- RK4: 4 derivative evaluations per sample, each with 4 sqrt + 4 sqrt + trig ≈ 20 transcendentals/sample
- Plus OU noise, zero-crossing, output writes ≈ 100 flops
- Total: ~200 flops/sample

### Memory Allocation

- All state fixed-size
- No allocation in `process()`
- Trails: 120 × 2 = 240 floats

### Buffers

- `trailX[120]`, `trailY[120]`: 240 floats

### Real-time Processing

- RK4 is deterministic
- `dt` capped at 0.001
- Mouse grab is via atomics (lock-free)
- No allocation, no blocking

### Thread Safety

- Mouse grab is communicated via atomics (`grabbed`, `grabX`, `grabY`)
- All other state is audio-thread only (except display trail, which is written by audio thread and read by UI thread)

### Numerical Stability

- `dt` capped at 0.001
- `!std::isfinite` → reset
- Soft wall at r = 2.5 D
- V/Oct latched on crossing (doesn't drift)

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<random>`, `<limits>`, `<memory>`, `<atomic>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `rack::dsp::SchmittTrigger`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`

## 11. Summary

`SpringPendulum` is a chaotic modulation source based on a physical spring-pendulum. Unlike `PsyLFO` (a double pendulum, which is purely driven by gravity), this one has four springs anchoring it to a plane, so it can oscillate, orbit, or settle. The mouse grab is a nice touch: you can physically pull the mass to a new position and let it go, and the physics will take over.

The Ornstein-Uhlenbeck noise makes the motion more organic than white noise — the mass wanders slowly rather than jittering. Combined with the spring physics, this gives a very naturalistic output that's good for modulating filters, VCOs, or anything else that benefits from slow, organic movement.

The main limitations are: CPU-heavy (200 flops/sample + 20 transcendentals), and the mouse grab requires the UI thread (can't be automated from a patch). But for a "hands-on" chaotic modulation source, this is excellent.