<!--
AGMM AUTOMATIC DOCUMENTATION
Source: XBodyProblem.cpp
-->

# XBodyProblem

## 1. Overview

`XBodyProblem` is a 28 HP N-body gravity LFO that simulates 2 to 10 point masses in a unit square under mutual Newtonian gravity. It is driven as a polyphonic LFO: per-body X, Y, speed, and wall-event triggers are emitted on four separate polyphonic outputs, so a single cable can drive up to 10 independent modulation destinations. The simulation state persists through patch reloads, so a saved patch continues the exact trajectory.

Key features:
- 2 – 10 point masses under mutual gravity, in a unit square
- Leapfrog integrator with sub-stepping (up to 16 substeps/sample)
- Wrap-around or bounce walls
- Equal or random per-body masses
- Polyphonic X / Y / SPEED / EVENT outputs
- Reset and Kick buttons (RESET also on a trigger input)
- 220 × 220 square display with trails, gravity links, and readouts

## Module Screenshot

![XBodyProblem VCV Rack Module](images/xbodyproblem.png)

## 2. Architecture

### Main Classes

- `XBodyProblem` — core module class inheriting from `rack::Module`
- `XBPanelBackground` — static panel artwork
- `XBKnobArcs` — live value arcs around the six knobs
- `XBDisplayWidget` — the 220 × 220 simulation display plus 32 px readout strip

### Inheritance

- `XBodyProblem` inherits from `rack::Module`
- All three UI widgets inherit from `rack::TransparentWidget`

### Structures

- `XBKnob { int param; float x, y; bool bipolar; const char *label; float top; }` — UI-only knob descriptor table

### State (grouped)

| Group | Storage |
|-------|---------|
| Simulation | `float px[10], py[10]`, `float vx[10], vy[10]`, `float ax[10], ay[10]`, `float rm[10]` |
| Topology | `int n`, `bool needAcc` |
| Settings (persisted) | `bool paused`, `bool zeroMomentum` |
| Triggers | `dsp::SchmittTrigger resetBtnTrig, resetJackTrig, kickTrig` |
| Event pulses | `dsp::PulseGenerator evPulse[10]` |
| Display — trails | `float trX[10][120]`, `float trY[10][120]`, `int trHead, trCount`, `float trTimer` |
| Display — readouts | `float vG, vEps, vSpeed, vKE`, `bool vWrap, vRandMass`, `int vRange` |
| Display — flashes | `float resetFlash, kickFlash`, `float eventFlash[10]` |

## 3. DSP Theory

### Gravitational acceleration

For every pair `(i, j)` with `i < j`:

$$
\mathbf{r} = \mathbf{p}_j - \mathbf{p}_i \quad(\text{folded to }(-0.5, 0.5]\text{ in wrap mode})
$$

$$
r^2 = |\mathbf{r}|^2 + \epsilon^2, \quad
\mathbf{a}_i \mathrel{+}= G\, m_j\, \frac{\mathbf{r}}{r^3}, \quad
\mathbf{a}_j \mathrel{-}= G\, m_i\, \frac{\mathbf{r}}{r^3}
$$

The magnitude of every `a_i` is clamped to 400 units/s² to prevent close encounters from destabilising the integrator.

### Leapfrog (kick-drift-kick)

Per substep of size `h`:

$$
\mathbf{v}_i \mathrel{+}= \frac{h}{2}\mathbf{a}_i \quad (\text{first kick, with } |\mathbf{v}_i| \le 3)
$$
$$
\mathbf{p}_i \mathrel{+}= h\,\mathbf{v}_i \quad (\text{drift, then wrap / bounce})
$$
$$
\mathbf{a}_i \leftarrow \text{recomputed}
$$
$$
\mathbf{v}_i \mathrel{+}= \frac{h}{2}\mathbf{a}_i \quad (\text{second kick})
$$

The number of substeps is `clamp(ceil(dt · speed / 0.002), 1, 16)`, and `h = (dt · speed) / sub`.

### Walls

- **Wrap**: any `p < 0` or `p ≥ 1` is folded by subtracting its floor, and the wall event fires.
- **Bounce**: `p` is reflected (`p → −p` or `p → 2 − p`), the velocity component is negated, and the wall event fires.

### Masses

`massOf(i)` returns `1` in Equal mode, or the stored random mass `rm[i] ∈ [0.4, 2.4]` in Random mode.

### Output mapping

`xbMap(v, range)` maps a normalised position `v ∈ [0, 1]` to the selected voltage range:

| Range index | Mapping |
|---|---|
| 0 | `v` (0..1 V) |
| 1 | `2v − 1` (−1..+1 V) |
| 2 | `10v − 5` (−5..+5 V, default) |
| 3 | `10v` (0..10 V) |
| 4 | `20v − 10` (−10..+10 V) |

### SPEED output

`out = clamp(|v| / 1.5, 0, 1) · 10`, so a body moving at 1.5 units/s reads 10 V.

### Trail sampling

One point per body is committed to a 120-slot ring buffer at 60 Hz (`1/60 s`), independent of `SPEED`, so the trail geometry is invariant to speed changes.

## 4. Parameters and Controls

### Six knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `MASSES_PARAM` | 2 | 10 | 2 | Number of bodies (snapped) |
| `SPEED_PARAM` | −4 | +4 | 0 | Simulation speed, `exp2` octaves |
| `GRAVITY_PARAM` | −4 | +4 | 0 | Gravity, `exp2` octaves |
| `RANGE_PARAM` | 0 | 4 | 2 | Output mapping (5 options) |
| `SOFT_PARAM` | 0 | 1 | 0.4 | Softening length |
| `TRAIL_PARAM` | 0 | 1 | 0.6 | Fraction of trail drawn |

### Buttons and switches

| Parameter | Description |
|---|---|
| `WALL_PARAM` | Wrap-around / Bounce (CKSS) |
| `MASSMODE_PARAM` | Equal / Random masses (CKSS) |
| `RESET_PARAM` | Momentary reset |
| `KICK_PARAM` | Momentary random impulse |

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `RESET_INPUT` | Trigger | 0..10 V | Rising edge resets the simulation |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `X_OUTPUT` | CV (poly) | per RANGE | X positions, one channel per body |
| `Y_OUTPUT` | CV (poly) | per RANGE | Y positions, one channel per body |
| `VEL_OUTPUT` | CV (poly) | 0..10 V | Per-body speed |
| `EVENT_OUTPUT` | Trigger (poly) | 0/10 V | 1 ms pulse on each wall event |

The poly channel counts track `MASSES` automatically.

### Lights

- `RESET_LIGHT`, `KICK_LIGHT`

### Menu

- **Pause simulation**
- **Zero total momentum on reset** (default on)
- **New random start**

## 6. Signal Processing Pipeline

1. **Read params** — MASSES (adapt `n` if changed), WALLS, MASSMODE, SPEED, GRAVITY, SOFT, RANGE.
2. **Handle RESET** — button OR jack; call `doReset()` and flash `resetFlash`.
3. **Handle KICK** — random impulse to every body, flash `kickFlash`.
4. **Initialize accelerations** — if `needAcc`, call `computeAcc`.
5. **Integrate** — up to 16 leapfrog substeps, collecting wall events in `ev[]`.
6. **NaN guard** — if any `p` or `v` is non-finite, `doReset()`.
7. **Write outputs** — poly `X`, `Y`, `VEL`, `EVENT`, each on `n` channels.
8. **Display feeds** — snapshot `vG, vEps, vSpeed, vWrap, vRange, vKE`; decay flashes; commit trail point at 60 Hz.

## 7. Important Functions

### `void initBody(int i)`

Randomises position, velocity, and mass for body `i`.

### `void doReset()`

Re-initialises all 10 bodies, removes bulk momentum if `zeroMomentum`, clears trails and sets `needAcc = true`.

### `void computeAcc(bool wrap, float G, float eps2)`

Pairwise loop building `ax[]`/`ay[]` (see §3). Applies the per-body acceleration cap.

### `void physicsStep(float h, bool wrap, float G, float eps2, bool *ev)`

One leapfrog step (kick-drift-recompute-kick), with wall handling and `ev[i]` set on wall events.

### `void process(const ProcessArgs &args)`

Main callback described in §6.

### `json_t *dataToJson()` / `void dataFromJson(json_t *)`

Persist `paused`, `zeroMomentum`, `n`, `px`, `py`, `vx`, `vy`, `rm`.

## 8. Mathematical Implementation

### Pairwise acceleration

$$
\mathbf{a}_i = \sum_{j \ne i} G\, m_j \frac{\mathbf{p}_j - \mathbf{p}_i}{\left(|\mathbf{p}_j - \mathbf{p}_i|^2 + \epsilon^2\right)^{3/2}}
$$

with minimum-image distance in wrap mode.

### Leapfrog half-kick + drift

$$
\mathbf{v}_i^{(n+1/2)} = \mathbf{v}_i^{(n)} + \tfrac{h}{2} \mathbf{a}_i^{(n)}
$$
$$
\mathbf{p}_i^{(n+1)} = \mathbf{p}_i^{(n)} + h\, \mathbf{v}_i^{(n+1/2)}
$$

(After recomputing `a`, apply the second half-kick.)

### Wall handling (bounce)

$$
p_i \leftarrow |p_i|, \quad v_i \leftarrow -v_i \quad (\text{lower wall})
$$
$$
p_i \leftarrow 2 - p_i, \quad v_i \leftarrow -v_i \quad (\text{upper wall})
$$

### Sub-step count

$$
\text{sub} = \text{clamp}\!\left(\left\lceil \frac{dt \cdot \text{speed}}{0.002} \right\rceil, 1, 16\right), \quad h = \frac{dt \cdot \text{speed}}{\text{sub}}
$$

### Softening

$$
\epsilon = 0.01 \cdot 15^{\text{SOFTEN}}
$$

## 9. Performance and Optimization

### CPU Usage

- Pairwise loop: `n(n−1)/2 ≤ 45` pairs per substep
- Up to 16 substeps: ≤ 720 pair evaluations per sample worst case
- ≈ 30 flops per pair → ~20 K flops/sample worst case

### Memory Allocation

- All state fixed-size; no allocation in `process()`
- Trails: `10 × 120 × 2` floats = 2400 floats ≈ 9.6 KB

### Real-time Processing

- All loops bounded by `n ≤ 10` and `sub ≤ 16`
- Suitable for a handful of instances on one core

### Thread Safety

- Trail arrays and readouts written by the audio thread, read by the UI via torn reads

### Numerical Stability

- Acceleration cap (400) and speed cap (3) bound the integrator
- Softening ε ≥ 0.01 (from `SOFTEN = 0`) keeps `1/r³` finite
- NaN guard triggers a full reset on any non-finite state
- Position clamped to `[0, 1]` after every reflect

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`, `<string>`

### External Classes

- `rack::Module`, `rack::ModuleWidget`, `rack::TransparentWidget`
- `dsp::SchmittTrigger`, `dsp::PulseGenerator`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::sectionHeader`, `agmm::screen`, `agmm::text`, `agmm::accent`, `agmm::warm`, `agmm::bright`, `agmm::dim`, `agmm::withA`, `agmm::ALIGN_L/C/R`

## 11. Summary

`XBodyProblem` is a modulation source built on a real N-body physics simulation rather than a mathematical chaos equation. The 2-to-10-body range gives it a whole spectrum of behaviour: a pair of bodies becomes a slow stable LFO, five bodies becomes a chaotic modulation cluster, and ten bodies becomes a wall of independent random-ish modulators that all share the same wall-hit rhythm.

The 220 × 220 display is the star of the show: gravity links show force strength in real time, wrap-mode ghosting makes bodies that cross an edge appear on both sides, and the trail (measured in attractor-space units, not wall-clock time) stays the same shape regardless of `SPEED`. The persisted simulation state means that leaving a patch running between sessions preserves the exact trajectory.