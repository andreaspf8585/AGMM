<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Lifeform.cpp
-->

# Lifeform

## 1. Overview

`Lifeform` is an 18 HP audio-reactive visual module that renders a glossy black ferrofluid "lifeform" floating in a glass dish inside a perforated white bezel. The lifeform is real 3D geometry: a lobed blob, ~140 Rosensweig spikes and orbiting droplets, tumbling through space with perspective projection, depth sorting and normal-based gloss.

Key features:
- Real 3D ferrofluid visualisation (not just a scope)
- ~140 Rosensweig spikes + 14 orbiting droplets
- Low/mid/high band analysis + onset detection
- Silence detection merges everything into one calm blob
- Magnet position and field strength can be modulated
- Pause / Reset / Audio-reactive buttons
- Envelope and field-level CV outputs
- Drag the display to rotate? (no — rotation is automatic from the physics)

## Module Screenshot

![Lifeform VCV Rack Module](images/lifeform.png)

## 2. Architecture

### Main Classes

- `LifeformModule`: Core module class inheriting from `rack::Module`
- `LifeformDisplay`: The 3D ferrofluid renderer (physics + drawing)
- `LFPanelBackground`: Static panel artwork
- 3D math helpers: `V3`, `M3`, `dot3`, `cross3`, `len3`, `norm3`, `m3mul`

### Inheritance

- `LifeformModule` inherits from `rack::Module`
- `LifeformDisplay` and `LFPanelBackground` inherit from `rack::TransparentWidget`

### Structures

- `V3 { float x, y, z; }` — minimal 3D vector
- `M3 { float m[9]; }` — row-major 3×3 matrix
- `LifeformDisplay::Spike { V3 d; float rnd, phase, h, v, bx, by, tx, ty, hw, spec, lit; bool front, vis; }`
- `LifeformDisplay::Drop { V3 e1, e2; float ang, dist, rest, r, dv, w, phase, x, y, sc, z, vis; }`
- `LifeformDisplay::ldir[GN]`, `wpos[GN]`, `wnorm[GN]` — surface grid (LAT × LON)

### State (grouped)

| Group | Storage |
|-------|---------|
| Audio analysis | `float env`, `lp1`, `lp2`, `eLow`, `eMid`, `eHigh`, `pulseFlash` |
| Field/magnet | `float field`, `phase` |
| Transport | `bool paused`, `prevPauseButton`, `prevAudioButton`, `prevPulse` |
| Shared atomics | `envSmoothed`, `fieldSmoothed`, `pulseSmoothed`, `lowSmoothed`, `midSmoothed`, `highSmoothed`, `magnetX`, `magnetY` |
| Display — 3D | `Spike spikes[140]`, `Drop drops[14]`, `V3 ldir[512]`, `wpos[512]`, `wnorm[512]`, `psx[512]`, `psy[512]`, `psc[512]`, `sil[96]` |
| Display — state | `M3 Rm`, `V3 om`, `V3 Lv, Hv, H2v, magD`, `float bodyBase, wobAmt, stretch, magA, magLen, offX, offY, offVX, offVY, act, calmK, wt, spinVel, ripAmt, ell, punch` |

## 3. DSP Theory

### Audio analysis

The input audio is normalized to ±1 (by `/5`) and processed:

- **Envelope follower** — fast attack (4 ms), slow release (180 ms)
- **Low band** — 1-pole LP at 180 Hz
- **Mid band** — 1-pole LP at 2200 Hz − low band
- **High band** — input − mid band LP
- **Pulse flash** — triggered on a rising edge of the PULSE input, decays at 3.5/s

Each band has its own asymmetric envelope follower (fast attack, slow release). The three bands drive the visual state.

### Onset detection ("punch")

The onset detector measures the rate of change of the envelope:

$$
\text{onset} = \max\left(0, \frac{\Delta \text{env}}{dt} \cdot \frac{0.016}{\max(dt, 0.004)} \cdot (0.5 + \text{sens}) - 0.025\right)
$$

$$
\text{punch} \leftarrow \min(\text{punch} + 1.6 \cdot \text{onset}, 1.2)
$$

The punch decays at 6/s. When a hit is detected (onset > 0.12 and cool-down > 0), the body gets a random rotational impulse.

### Activity collapse

In silence, the lifeform collapses into a single calm blob:

$$
\text{level} = \text{env}(0.4 + \text{sens}) + 0.2 \cdot \text{punch}
$$

$$
\text{actTarget} = \max(\text{smooth01}(0.008, 0.09, \text{level}), \text{pulse})
$$

$$
\text{act} \leftarrow \text{act} + (\text{actTarget} - \text{act}) \cdot (1 - e^{-\text{rate} \cdot \Delta t})
$$

where `rate = 30/s` for rising, `1/s` for falling. Then:

$$
\text{calmK} = \text{smooth01}(0.15, 0.85, 1 - \text{act})
$$

So `calmK = 1` when act = 0 (silent) and `calmK = 0` when act = 1 (active).

### 3D tumble

The body rotates with an angular velocity `om` that tracks a target axis * spin magnitude. A hit or kick adds a random impulse. `om` is smoothly tracked and capped at 5 rad/s.

### Body geometry

The body radius is a sum of sinusoidal wobbles in local direction `d`:

$$
r = \text{base}\left(1 + \text{wob} \cdot n + \text{rip} \cdot \sin(7 d_y - \text{wt} \cdot 7 + 2 d_x) + \text{ell}(1.5 d_x^2 - 0.5) + \text{stretch} \cdot a^3\right)
$$

where:
- `n` is a sum of 4 sines
- `a = max(0, d · magD)` (magnet alignment)
- `base`, `wob`, `rip`, `ell`, `stretch` are all driven by the audio state

### Spikes

Each spike has a local direction `d` on the body surface. The visible height depends on:

$$
\text{localField} = (\text{field} \cdot (0.8 + 0.35 \cdot a \cdot \text{magLen}) + \text{audioEnv}(0.45 + 0.7a) + \text{highA}(0.15 + 0.55 \cdot \text{rnd}) + 0.2 \text{pulse} + 0.3 \text{punch}) \cdot \text{gate}
$$

where `gate = 0.25 + 0.75 * act`. Below a threshold, the spike is 0. Above the threshold, it grows as `sqrt(drive)`.

### Droplets

14 droplets orbit the body at various distances. Each has its own orbital plane (defined by e1, e2). The distance oscillates with the audio and slowly bobs; when `punch` or `hit` fires, the droplet gets a random outward impulse.

### Projection

The camera is at `z = 320` with perspective:

$$
s = \frac{320}{320 - z}, \quad x' = x s, \quad y' = y s
$$

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `FIELD_PARAM` | 0 | 1 | 0.55 | Magnetic field |
| `MOTION_PARAM` | 0 | 1 | 0.45 | Magnet motion |
| `VISC_PARAM` | 0 | 1 | 0.50 | Fluid viscosity |
| `SPIKES_PARAM` | 0 | 1 | 0.65 | Spike amplitude |
| `SENS_PARAM` | 0 | 1 | 0.65 | Audio sensitivity |
| `GLOSS_PARAM` | 0 | 1 | 0.75 | Gloss |

### Buttons

| Button | Description |
|--------|-------------|
| `PAUSE_PARAM` | Pause the simulation |
| `RESET_PARAM` | Reset the lifeform |
| `AUDIO_PARAM` | Toggle audio-reactive mode |

### Inputs

| Input | Description |
|-------|-------------|
| `AUDIO_INPUT` | Audio |
| `PULSE_INPUT` | Pulse trigger |
| `FIELD_CV` | Field CV |
| `MOTION_CV` | Motion CV |
| `VISC_CV` | Viscosity CV |
| `SPIKES_CV` | Spikes CV |
| `SENS_CV` | Sensitivity CV |
| `GLOSS_CV` | Gloss CV |

### Outputs

| Output | Description |
|--------|-------------|
| `ENV_OUTPUT` | Envelope (0–10 V) |
| `FIELD_OUTPUT` | Field level (0–10 V) |

### Lights

- `PAUSE_LIGHT` — On when paused
- `AUDIO_LIGHT` — On when audio-reactive

### Enumerations

- `ParamId`: Field, Motion, Visc, Spikes, Sens, Gloss, Pause, Reset, Audio
- `InputId`: Audio, Pulse, FieldCV, MotionCV, ViscCV, SpikesCV, SensCV, GlossCV
- `OutputId`: Env, Field
- `LightId`: Pause, Audio

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `AUDIO_INPUT` | Audio | ±5 V | Audio |
| `PULSE_INPUT` | Gate | > 1 V | Pulse trigger |
| `FIELD_CV` | CV | 0–10 V | Field offset |
| `MOTION_CV` | CV | 0–10 V | Motion offset |
| `VISC_CV` | CV | 0–10 V | Viscosity offset |
| `SPIKES_CV` | CV | 0–10 V | Spikes offset |
| `SENS_CV` | CV | 0–10 V | Sensitivity offset |
| `GLOSS_CV` | CV | 0–10 V | Gloss offset |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `ENV_OUTPUT` | CV | 0–10 V | Envelope |
| `FIELD_OUTPUT` | CV | 0–10 V | Field level |

### Bypass

`configBypass(AUDIO_INPUT, ENV_OUTPUT)` — audio passes through to the envelope output when no cable.

## 6. Signal Processing Pipeline

1. **Button debounce** — Toggle Pause and Audio-reactive on button press.
2. **Envelope** — Fast/slow envelope from input.
3. **Band split** — Low/mid/high with their own followers.
4. **Pulse flash** — Rising edge of PULSE sets flash = 1; decays at 3.5/s.
5. **Parameter + CV read** — Field, motion, sens.
6. **Field update** — Audio boost + pulse flash; one-pole smoothed (25 ms).
7. **Magnet phase** — Advances at `motion * 1.35` rad/s.
8. **Publish to UI** — All smoothed values written to atomics.
9. **Write outputs** — Env (0–10 V), field (0–10 V).

The UI-thread physics (in `LifeformDisplay::stepPhysics`) reads the atomics and drives the 3D simulation at display-frame rate.

## 7. Important Functions

### `v(int p, int cv)`

Reads a knob + CV, clamped to 0..1:

```cpp
return clampT(params[p].getValue() + inputs[cv].getVoltage() / 10.f, 0.f, 1.f);
```

### `LifeformDisplay::surfaceR(const V3 &d, const V3 &wd)`

Returns the body surface radius for a local direction `d` with world direction `wd`.

### `LifeformDisplay::stepPhysics(float dt)`

The big physics step: reads smoothed audio, runs the onset detector, updates the body shape/tumble/offset, integrates the spikes and droplets, and rebuilds the projected geometry.

### `LifeformDisplay::buildGeometry()`

Projects every surface grid point, computes the silhouette, and projects every spike and droplet into 2D.

### `LifeformDisplay::drawSpikes(NVGcontext *vg, float cx, float cy, float gloss, bool front)`

Renders spikes in 3 passes: dark silhouettes, edge glints, sparkle tips.

### `LifeformDisplay::drawBody(NVGcontext *vg, float cx, float cy, float gloss)`

Renders the black body silhouette plus normal-based reflections.

## 8. Mathematical Implementation

### Envelope

$$
\text{env} \leftarrow \text{env} + (|\text{in}| - \text{env}) \cdot \begin{cases} a_{\text{atk}} & |\text{in}| > \text{env} \\ a_{\text{rel}} & |\text{in}| \leq \text{env} \end{cases}
$$

### Band filters

$$
\text{lp1} \leftarrow \text{lp1} + (\text{in} - \text{lp1}) \cdot c_{LP1}, \quad c_{LP1} = \min(1, 2\pi \cdot 180 / f_s)
$$

$$
\text{bandLow} = 1.5|\text{lp1}|, \quad \text{bandMid} = 1.6|\text{lp2} - \text{lp1}|, \quad \text{bandHigh} = 2.0|\text{in} - \text{lp2}|
$$

### Onset

$$
\text{onset} = \max\left(0, \Delta\text{env} \cdot \frac{0.016}{\max(dt, 0.004)} \cdot (0.5 + \text{sens}) - 0.025\right)
$$

### Activity

$$
\text{actTarget} = \max(\text{smooth01}(0.008, 0.09, \text{level}), \text{pulse})
$$

### Rotational dynamics

$$
\mathbf{om} \leftarrow \mathbf{om} + (\mathbf{om}_{T} - \mathbf{om}) \cdot (1 - e^{-1.5 \Delta t})
$$

### Body shape

$$
r = \text{base}(1 + \text{wob} \cdot n + \text{rip}\sin(7d_y - 7\text{wt} + 2d_x) + \text{ell}(1.5d_x^2 - 0.5) + \text{stretch}\cdot a^3)
$$

### Spike height

$$
h \leftarrow h + (k(\text{target} - h) - d \cdot v) \cdot \Delta t
$$

with `k = 60 * (1.1 - visc * 0.4) * (1 + 1.2 * audioEnv)` and `d = (8 + visc * 8) * (1 + 0.6 * audioEnv)`.

### Projection

$$
s = \frac{320}{320 - z}, \quad x' = xs, \quad y' = ys
$$

## 9. Performance and Optimization

### CPU Usage

- Audio thread: ~500 flops/sample (LP filters + 3 followers + envelope + atomics)
- UI thread (per frame at 60 fps):
  - `stepPhysics`: 140 spikes × ~30 flops + 14 droplets × ~50 flops + 512 grid points × ~20 flops ≈ 16 K flops
  - `buildGeometry`: 512 grid points × ~30 flops + 140 spikes × ~20 flops ≈ 18 K flops
  - Drawing: 140 spikes × 3 passes + 512 grid reflections ≈ 20 K flops
- Total: ~50 K flops/frame on the UI thread

### Memory Allocation

- All state is fixed-size
- `spikes[140]`, `drops[14]`, `ldir[512]`, `wpos[512]`, `wnorm[512]`, `psx/psy/psc[512]`, `sil[96]` ≈ 20 KB total
- No allocation in `process()`

### Buffers

- 140 Spike structs (~40 bytes each) ≈ 5.6 KB
- 14 Drop structs (~48 bytes each) ≈ 672 B
- 512 × (2 × V3 + 3 floats) = 512 × 44 ≈ 22 KB
- 96 silhouette floats

### Real-time Processing

- Physics runs on the UI thread at ~60 fps (not sample rate)
- Audio analysis runs at sample rate but is minimal
- No allocation, no blocking on either thread

### Thread Safety

- Audio thread → UI: via `std::atomic<float>` (envSmoothed, etc.)
- UI thread reads them via `.load()`
- A torn read is only cosmetic

### Numerical Stability

- All `V3` operations use small epsilons to avoid division by zero
- `clampT` on all values
- `!std::isfinite` checks in some places

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<algorithm>`, `<atomic>`, `<cmath>`, `<cstdio>`, `<cstring>`, `<string>`, `<vector>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::plate`, `agmm::text`, `agmm::screen`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`, `agmm::VioletLight`

## 11. Summary

`Lifeform` is the most visually ambitious module in the AGMM collection: a 3D ferrofluid simulation rendered in real time, driven by audio analysis. It's more of an "instrument" than a "module" — you patch audio into it and it responds with a living, glowing blob that grows spikes and orbits droplets in response to the sound.

The DSP is minimal (just envelope and three band filters), but the UI is where the complexity lives: ~140 spikes, 14 droplets, a 512-point surface grid, and the physics that drives them all. Every visual element is derived from the audio state or the field/motion knobs, so the visualisation feels alive and responsive.

The main limitations are: the UI physics runs at 60 fps (so very fast transients might be missed), and the module is CPU-heavy on the UI thread (about 50 K flops per frame). But for a visual instrument, it's a beautiful piece of work.
