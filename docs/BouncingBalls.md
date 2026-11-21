<!--
AGMM AUTOMATIC DOCUMENTATION
Source: BouncingBalls.cpp
-->

# BouncingBalls

## 1. Overview

`BouncingBalls` is a 28 HP chaos generator: up to 16 balls bounce inside a 20 V × 20 V arena, and their X/Y positions, zero-crossings, ball-ball collisions and wall hits are all exposed as polyphonic outputs (one channel per ball). Each ball also has its own random V/Oct value, re-randomised on every zero-crossing of that axis, so the module can drive up to 16 voices of a synth.

Key features:
- Up to 16 balls in a 20 V × 20 V arena
- Polyphonic outputs: X, Y, gate X, gate Y, V/Oct X, V/Oct Y, collision gate, wall-hit gate
- Configurable base speed and angle (with CV)
- Ball-ball collisions (toggleable)
- Adjustable zero-crossing gate duration (0.5 ms .. 200 ms via menu)
- Selectable V/Oct range (0..1, −1..1, −2..2, −5..5, 0..5) via menu
- Live arena display with fading trails, axis-crossing flashes, wall-hit flashes and collision rings
- Per-ball speed bars and position strip
- 16 collision LEDs + 16 wall LEDs

## Module Screenshot

![BouncingBalls VCV Rack Module](images/BouncingBalls.png)

## 2. Architecture

### Main Classes

- `BouncingBalls`: Core module class inheriting from `rack::Module`
- UI widgets:
  - `BBPanelBackground` — static art
  - `BBDisplayWidget` — live arena with trails, flashes
  - `BBReadoutWidget` — text row under the arena
  - `BBBarsWidget` — per-ball speed bars
- Custom lights: `BBCyanLight`, `BBVioletLight`

### Inheritance

- `BouncingBalls` inherits from `rack::Module`
- All four UI widgets inherit from `rack::TransparentWidget`
- The two lights are template specializations of `GrayModuleLightWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Positions | `double posX[16]`, `posY[16]` |
| Velocities | `double velX[16]`, `velY[16]` |
| Previous positions | `double prevX[16]`, `prevY[16]` |
| Gate timers | `int gateXTimer[16]`, `gateYTimer[16]` |
| Extra gate timers | `int collisionTimer[16]`, `wallHitTimer[16]` |
| V/Oct | `double voctX[16]`, `voctY[16]` |
| Params | `speedBase`, `angleBase`, `numBalls` |
| Previous params | `prevSpeed`, `prevAngle` |
| Ranges | `VoctRange voctRange`, `GateDuration gateDuration` |
| RNG | `std::mt19937 rng` |
| Display — trail | `float trailX[16][36]`, `trailY[16][36]`, `trailHead`, `trailCount`, `trailTimer` |
| Display — flashes | `flashCol[16]`, `flashWall[16]`, `flashXc[16]`, `flashYc[16]` |
| Display — events | `wallSide[16]`, `wallPos[16]`, `xcPos[16]`, `ycPos[16]` |

## 3. DSP Theory

### Ball physics

Each ball is a point in a 20 V × 20 V arena with position $(x, y)$ and velocity $(v_x, v_y)$. The physics is:

$$
x[n] \leftarrow x[n-1] + v_x[n-1]\Delta t, \quad y[n] \leftarrow y[n-1] + v_y[n-1]\Delta t
$$

### Wall reflection

If the ball crosses a wall, it is folded back and its velocity is negated:

$$
x > 10 \implies x' = 20 - x, \quad v_x' = -v_x
$$

This is exact reflection (no energy loss, no overshoot).

### Ball-ball collision

For each pair $(i, j)$, if the distance $d = |\mathbf{p}_i - \mathbf{p}_j|$ is less than $2r$ (twice the ball radius), the balls are pushed apart along the normal $\mathbf{n} = (\mathbf{p}_i - \mathbf{p}_j)/d$:

$$
\mathbf{p}_i' = \mathbf{p}_i + \mathbf{n}(2r-d)/2, \quad \mathbf{p}_j' = \mathbf{p}_j - \mathbf{n}(2r-d)/2
$$

If the relative velocity along the normal is negative (approaching), the normal component is exchanged:

$$
\mathbf{v}_i' = \mathbf{v}_i - \mathbf{n}(\mathbf{v}_i \cdot \mathbf{n})2, \quad \mathbf{v}_j' = \mathbf{v}_j + \mathbf{n}(\mathbf{v}_i \cdot \mathbf{n})2
$$

This is a perfectly elastic collision. Two passes are made per sample so that multi-ball piles settle.

### Zero-crossing detection

For each ball and each axis, if the sign of the position changed since the last sample:

$$
\text{sign}(x[n-1]) \ne \text{sign}(x[n]) \implies \text{fire gate X}
$$

When this happens: a new random V/Oct value is generated, the gate timer is set, the flash is set, and the other-axis position is recorded.

### Random V/Oct

The V/Oct value is a fresh uniform random in $[-1, 1]$ scaled to the current range:

$$
\text{voct} = \text{range\_scale} \cdot u
$$

where $u \sim \text{Uniform}[-1, 1]$ and `range_scale` depends on the selected range (1, 1, 2, 5, 2.5 for the five options).

### Trail sampling

Each ball's trail is a 36-point ring buffer, sampled at 50 Hz:

$$
\text{trailX}[b][\text{head}] = x[b], \quad \text{head} \leftarrow (\text{head}+1) \bmod 36
$$

The display draws the trail with alpha decreasing with age, so it fades over ~0.7 s.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `SPEED_PARAM` | 0 | 20 V/s | 5 V/s | Base ball speed |
| `ANGLE_PARAM` | −180° | +180° | 0° | Base ball angle |
| `NUM_BALLS_PARAM` | 1 | 16 | 1 | Number of balls |
| `ANGLE_ATTENUATOR_PARAM` | −1 | 1 | 0 | Angle CV amount |

### Buttons / Switches

| Parameter | Description |
|-----------|-------------|
| `RESET_BUTTON` | Reset all balls |
| `COLLISIONS_SWITCH_PARAM` | Enable/disable ball-ball collisions |

### Inputs

| Input | Description |
|-------|-------------|
| `RESET_CV_INPUT` | Reset gate |
| `ANGLE_CV_INPUT` | Angle CV |
| `SPEED_CV_INPUT` | Speed CV (10 V adds 20 V/s) |
| `BALLS_CV_INPUT` | Number-of-balls CV (10 V adds 15 balls) |

### Outputs

| Output | Description |
|--------|-------------|
| `X_OUTPUT` | X (polyphonic) |
| `Y_OUTPUT` | Y (polyphonic) |
| `GATE_X_OUTPUT` | Gate X (polyphonic) |
| `GATE_Y_OUTPUT` | Gate Y (polyphonic) |
| `VOCT_X_OUTPUT` | V/Oct X (polyphonic) |
| `VOCT_Y_OUTPUT` | V/Oct Y (polyphonic) |
| `COLLISION_GATE_OUTPUT` | Collision gate (polyphonic) |
| `WALL_HIT_GATE_OUTPUT` | Wall-hit gate (polyphonic) |

### Enumerations

- `VoctRange`: Range0_1, RangeN1_1, RangeN2_2, RangeN5_5, Range0_5
- `GateDuration`: Trigger, 5ms, 10ms, 20ms, 50ms, 100ms, 200ms
- `ParamId`: Speed, Angle, NumBalls, ResetButton, AngleAttenuator, CollisionsSwitch
- `InputId`: ResetCV, AngleCV, SpeedCV, BallsCV
- `OutputId`: XOut, YOut, GateX, GateY, VOctX, VOctY, CollisionGate, WallHitGate
- `LightId`: CollisionLED1..16, WallLED1..16

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `RESET_CV_INPUT` | Gate | > 1 V | Resets all balls |
| `ANGLE_CV_INPUT` | CV | ±10 V | Angle modulation (attenuverted) |
| `SPEED_CV_INPUT` | CV | 0–10 V | Speed (adds 0–20 V/s) |
| `BALLS_CV_INPUT` | CV | 0–10 V | Ball count (adds 0–15 balls) |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `X_OUTPUT` | Poly | ±10 V | X position per ball |
| `Y_OUTPUT` | Poly | ±10 V | Y position per ball |
| `GATE_X_OUTPUT` | Poly Gate | 0/10 V | Zero-crossing gate X |
| `GATE_Y_OUTPUT` | Poly Gate | 0/10 V | Zero-crossing gate Y |
| `VOCT_X_OUTPUT` | Poly CV | see range | Random V/Oct X |
| `VOCT_Y_OUTPUT` | Poly CV | see range | Random V/Oct Y |
| `COLLISION_GATE_OUTPUT` | Poly Gate | 0/10 V | Ball-ball collision |
| `WALL_HIT_GATE_OUTPUT` | Poly Gate | 0/10 V | Wall hit |

## 6. Signal Processing Pipeline

1. Parameter update — Read speed, angle, balls; smooth angle; create any newly added balls; reorient velocities.
2. Reset detection — On a rising edge of the reset button or CV, reset all balls.
3. Flash decay — Decay the visual flash values.
4. Per-ball physics — For each ball: save previous position, integrate position from velocity, reflect off walls, detect zero-crossings.
5. Ball-ball collisions — If enabled, run two relaxation passes; record collisions.
6. Decrement timers — Decrement gate, collision and wall timers.
7. LEDs — Set collision and wall LED brightness from flash values.
8. Outputs — Set the 8 polyphonic outputs, each with `numBalls` channels.
9. Trail sampling — Every 1/50 s, sample all ball positions into the trail ring.

## 7. Important Functions

### `onReset()`

Re-seeds the ball positions, velocities, and per-ball V/Oct values.

```cpp
std::uniform_real_distribution<double> posDist(-8.0, 8.0);
std::uniform_real_distribution<double> angleOff(-M_PI / 4, M_PI / 4);
std::uniform_real_distribution<double> speedOff(0.8, 1.2);
for (int i = 0; i < numBalls; ++i) {
    posX[i] = posDist(rng);
    posY[i] = posDist(rng);
    double angleRad = angleBase + angleOff(rng);
    double speed = speedBase * speedOff(rng);
    velX[i] = speed * cos(angleRad);
    velY[i] = speed * sin(angleRad);
    // ...
}
```

### `updateParameters()`

Reads speed / angle / ball-count controls and CVs, computes the effective values, creates newly added balls, and reorients existing velocities when the base angle or speed changes.

```cpp
if (speedBase != prevSpeed || newAngleBase != prevAngle) {
    for (int i = 0; i < numBalls; ++i) {
        double oldSpeed = std::sqrt(velX[i]*velX[i] + velY[i]*velY[i]);
        double oldAngle = std::atan2(velY[i], velX[i]);
        if (oldSpeed > 0.01) {
            double delta = oldAngle - prevAngle;
            double newAngle = newAngleBase + delta;
            double newSpeed = (prevSpeed > 1e-6) ? speedBase * (oldSpeed / prevSpeed) : speedBase;
            velX[i] = newSpeed * cos(newAngle);
            velY[i] = newSpeed * sin(newAngle);
        }
        // ...
    }
}
```

### `resolveCollisionsAndRecord(bool record, bool collided[16])`

Two relaxation passes over all pairs, pushing overlapping balls apart and exchanging the normal velocity component.

### `generateRandomVoct(int ball, bool forX)`

Generates a new random V/Oct value for one ball and one axis, scaled to the current range.

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Ball position integration

$$
\mathbf{p}[n] = \mathbf{p}[n-1] + \mathbf{v}[n-1]\Delta t
$$

with $\Delta t = 1/f_s$.

### Wall reflection

$$
x > 10 \implies x' = 20 - x, \quad v_x' = -v_x
$$

This preserves energy exactly (no bounce loss).

### Zero-crossing gate

If the sign changed, the gate timer is set to `sampleRate * duration` samples, which decrements to zero.

### Ball-ball collision

For pairs with $d < 2r$:

$$
\mathbf{n} = \frac{\mathbf{p}_i - \mathbf{p}_j}{d}, \quad \mathbf{p}_i' = \mathbf{p}_i + \mathbf{n}\frac{2r-d}{2}, \quad \mathbf{p}_j' = \mathbf{p}_j - \mathbf{n}\frac{2r-d}{2}
$$

Then, if $\mathbf{v}_{\text{rel}} \cdot \mathbf{n} < 0$:

$$
\mathbf{v}_i' = \mathbf{v}_i - \mathbf{n}(\mathbf{v}_{\text{rel}}\cdot\mathbf{n}), \quad \mathbf{v}_j' = \mathbf{v}_j + \mathbf{n}(\mathbf{v}_{\text{rel}}\cdot\mathbf{n})
$$

### Random V/Oct ranges

| Range | Formula |
|-------|---------|
| 0..1 | $(u+1)/2$ |
| −1..1 | $u$ |
| −2..2 | $2u$ |
| −5..5 | $5u$ |
| 0..5 | $(u+1)/2 \cdot 5$ |

where $u \sim \text{Uniform}[-1, 1]$.

## 9. Performance and Optimization

### CPU Usage

- 16 balls × (position + wall check + zero-crossing + timer decrement) ≈ 16 × 20 flops ≈ 320 flops
- Ball-ball collisions (if enabled): 16 choose 2 = 120 pairs × 2 passes × 15 flops ≈ 3600 flops
- Total without collisions: ~500 flops/sample
- Total with collisions: ~4000 flops/sample

### Memory Allocation

- All state is fixed-size arrays
- No allocation in `process()`
- Trails are 16 × 36 × 2 = 1152 floats ≈ 4.6 KB

### Buffers

- `trailX[16][36]`, `trailY[16][36]`: 1152 floats
- `flashCol[16]`, `flashWall[16]`, `flashXc[16]`, `flashYc[16]`: 64 floats
- Various per-ball timers: 4 × 16 = 64 ints

### Real-time Processing

- All loops are bounded by `numBalls` (≤ 16)
- Collision resolution is O(n²), which is fine for n ≤ 16
- No allocation, no blocking

### Thread Safety

- All state is written by the audio thread and read by the UI thread via torn reads
- The `std::mt19937` is audio-thread-only

### Numerical Stability

- All physics is done in `double` so float errors don't accumulate
- Wall reflection is exact (no overshoot)
- Ball-ball collision has a 2-pass relaxation to prevent jitter
- No division by zero (the `dist > 0` check before normalising)

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`, `<random>`, `<limits>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `rack::dsp::SchmittTrigger`
- `agmm::panelBase`, `agmm::bezel`, `agmm::plate`, `agmm::text`, `agmm::plotFrame`, `agmm::screen`, `agmm::sectionHeader`, `agmm::meter`, `agmm::chip`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::textCol`, `agmm::CyanLight`, `agmm::VioletLight`

## 11. Summary

`BouncingBalls` is a polyphonic chaos generator built on a simple 2D physics simulation: point masses bouncing in a 20 V × 20 V arena. It exposes every event the simulation generates — position, zero-crossing gates, ball-ball collisions, wall hits — as polyphonic outputs, which makes it a Swiss-army knife for generative patching.

The physics is exact elastic reflection with no energy loss, which means the balls keep moving forever (until reset). The collision resolution is a simple 2-pass relaxation that settles multi-ball piles without jitter. The per-ball random V/Oct value is re-rolled on every zero-crossing, giving a constantly changing but slowly-varying melody line.

The module's main strengths are its polyphonic outputs (16 independent voices from one module) and its visual feedback (the arena display shows trails, flashes, crossing rings, collision rings). Its main limitation is that it has only one "physics scene" — you can't have different balls in different arenas — but for generative work this is usually fine.
