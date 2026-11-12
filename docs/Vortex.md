<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Vortex.cpp
-->

# Vortex

## 1. Overview

`Vortex` is a 32 HP dual-mode reverb / echo effect. It contains two engines — a plate reverb and a modulated echo with optional ping-pong — switched by a latching MODE button. Twelve parameter cells (knob, CV jack, and bipolar attenuverter each) cover both modes; cells that do not apply to the active mode are dimmed. The module supports mono / poly and stereo-in / stereo-out operation, a latching or gate-driven freeze, and a clear sequence that fades the wet path out, resets every engine, and fades back in.

Key features:
- Two engines (plate reverb, modulated echo with ping-pong) in one module
- Latching MODE button, saved with the patch
- 12 parameter cells with CV + attenuverter each
- Per-mode active-cell dimming
- Latching freeze + freeze gate input
- Clear button + clear trigger input
- Three display modes (Wave / Bars / Spectrum)
- Selectable CV range (unipolar 0..+10 V or bipolar −5..+5 V)

## Module Screenshot

![Vortex VCV Rack Module](images/vortex.png)

## 2. Architecture

### Main Classes

- `VortexModule` — core module class inheriting from `rack::Module`
- `Vortex::Voice` and `Vortex::Controller` — DSP engine (declared in `VortexModule.hpp`)
- `VxPanel` — static panel artwork
- `VxDisplay` — 440 × 112 interactive display (three modes)

### Inheritance

- `VortexModule` inherits from `rack::Module`
- `VxPanel` inherits from `rack::TransparentWidget`
- `VxDisplay` inherits from `rack::OpaqueWidget` (handles clicks)

### Structures

- `VxCellDef { const char *name; int param, atten, cv, group; }` — one entry per parameter cell; `group` selects mode-specific dimming

### State (grouped)

| Group | Storage |
|-------|---------|
| Engines | `Vortex::Voice voices[16]`, `Vortex::Controller ctl` |
| Rate / smoothing | `float allocSr`, `float kMix`, `float mixS` |
| Clear sequence | `bool clearRequested`, `float clearFade`, `bool clearActive` |
| Latches | `bool freezeLatched`, `bool echoModeState` |
| Settings (persisted) | `CVRange cvRangeMode`, `int dispMode` |
| Display — readouts | `int dispCh`, `bool dispFreeze`, `float dispVal[12]`, `float dispRt60`, `float dispEchoMs`, `float dispSize`, `float dispSr` |
| Display — meters | `float peakL`, `float peakR` |
| Display — scope | `float scopeMinL/R/I[396]`, `float scopeMaxL/R/I[396]`, `int scopeHead`, `int scopeCount`, `int scopeBinSamples` |
| Display — FFT | `float specBuf[4096]`, `int specPos` |
| Safety | `int limiterRecovery` |

## 3. DSP Theory

### Reverb mode (plate)

The plate engine (inside `Vortex::Voice`) is a network of 14 tapped delays with diffusion and damping, mapped to a stereo output. The controller derives the current RT60 and plate size from the smoothed controls; the display uses these for the BARS and footer readouts.

### Echo mode

The echo engine is a modulated stereo delay with feedback. The DIFF/PING control crossfades between a single cross-fed delay and a ping-pong structure where even repeats alternate left and right.

### Control CV reading

Each cell value is:

$$
v = \text{clamp}\big(\text{knob} + \text{atten} \cdot \text{cv}, 0, 1\big)
$$

where `cv` depends on the CV range mode:

$$
\text{cv} = \begin{cases}
\text{clamp}\big(v_{\text{in}}/5, -1, +1\big) & \text{bipolar } (-5..+5\text{ V}) \\
v_{\text{in}}/10 & \text{unipolar } (0..+10\text{ V})
\end{cases}
$$

MIX is additionally smoothed by a one-pole filter (20 ms time constant) before being used for the dry / wet crossfade.

### Equal-power dry / wet

$$
\text{out} = d \cdot \cos\!\left(\tfrac{\pi}{2} m\right) + w \cdot \sin\!\left(\tfrac{\pi}{2} m\right)
$$

so that `dry² + wet² = 1` when `m` is the smoothed mix.

### Freeze and clear

- **Freeze**: latching state `freezeLatched` is ORed with the freeze gate input (> 2 V). The engine freezes internally when this is high.
- **Clear**: `clearRequested` triggers a phase-1 fade from `clearFade = 1` down to 0 at `10/s`, then every voice is reset and phase 2 ramps back to 1 at the same rate (~200 ms total).

### Channel routing

- `ST IN = Off`: each poly channel feeds its own voice, `L/R` are independent.
- `ST IN = On`: a single voice consumes the L/R jacks.
- `ST OUT = Off`: each voice writes to its own poly channel.
- `ST OUT = On`: all voices are summed to one stereo pair.

## 4. Parameters and Controls

### 12 knob cells

Each cell has a knob, a left-side CV jack, and a right-side bipolar attenuverter.

| Cell | Reverb | Echo |
|---|---|---|
| `PRE_DELAY` | 0 – 500 ms pre-delay | 0 – 500 ms pre-delay |
| `SIZE` | Plate size | — (dimmed) |
| `DECAY` | RT60, 0.2 – 12 s | — (dimmed) |
| `DIFFUSION` | Diffusion | Ping-pong amount |
| `HIGH_DAMP` | Damping cutoff (2500 Hz – 18 kHz) | Feedback LP cutoff |
| `LOW_CUT` | Low cut (20 – 500 Hz) | Low cut |
| `MOD_SPEED` | Modulation rate | Wow rate |
| `MOD_DEPTH` | Modulation depth | Wow depth |
| `MIX` | Dry/wet, equal power | Dry/wet, equal power |
| `ECHO_TIME` | — (dimmed) | Delay time, 0 – 1000 ms |
| `ECHO_FEEDBACK` | — (dimmed) | Feedback, 0 – 95 % |
| `ECHO_LEVEL` | — (dimmed) | Wet level, 0 – 100 % |

### Buttons and switches

| Parameter | Description |
|---|---|
| `ECHO_MODE_BUTTON_PARAM` | Latching reverb / echo toggle (saved) |
| `FREEZE_BUTTON_PARAM` | Latching freeze toggle (saved) |
| `CLEAR_BUTTON_PARAM` | Momentary clear |
| `IN_STEREO_BUTTON_PARAM` | Stereo input toggle (CKSS) |
| `OUT_STEREO_BUTTON_PARAM` | Stereo output toggle (CKSS) |

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `IN_L_INPUT` | Audio | ±10 V | Left / poly |
| `IN_R_INPUT` | Audio | ±10 V | Right |
| `PRE_DELAY_CV_INPUT` | CV | 0..10 V / −5..+5 V | Pre-delay CV |
| `SIZE_CV_INPUT` | CV | same | Size CV |
| `DECAY_CV_INPUT` | CV | same | Decay CV |
| `DIFFUSION_CV_INPUT` | CV | same | Diffusion CV |
| `HIGH_DAMP_CV_INPUT` | CV | same | Damping CV |
| `LOW_CUT_CV_INPUT` | CV | same | Low-cut CV |
| `MOD_SPEED_CV_INPUT` | CV | same | Mod speed CV |
| `MOD_DEPTH_CV_INPUT` | CV | same | Mod depth CV |
| `MIX_CV_INPUT` | CV | same | Mix CV |
| `ECHO_TIME_CV_INPUT` | CV | same | Echo time CV |
| `ECHO_FEEDBACK_CV_INPUT` | CV | same | Echo feedback CV |
| `ECHO_LEVEL_CV_INPUT` | CV | same | Echo level CV |
| `FREEZE_CV_INPUT` | Gate | 0..10 V | Freeze while > 2 V |
| `CLEAR_INPUT` | Trigger | 0..10 V | Clear trigger |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUT_L_OUTPUT` | Audio | ±12 V | Left / poly |
| `OUT_R_OUTPUT` | Audio | ±12 V | Right |

### Lights

- `CLEAR_BUTTON_LIGHT`, `FREEZE_BUTTON_LIGHT`, `ECHO_MODE_BUTTON_LIGHT`

### Menu

- **Display** → Wave / Bars / Spectrum
- **CV range** → 0 V to +10 V (unipolar, default) or −5 V to +5 V (bipolar)

## 6. Signal Processing Pipeline

1. **Allocate** — on sample-rate change, `voices[c].init(sr)`; refresh `kMix` and `scopeBinSamples`.
2. **Detect channels** — mono / poly / stereo in / stereo out; set output channel counts.
3. **Read params + CV** — 12 cells via `getParam(param, atten, cv)` and `mix`.
4. **Latch buttons** — MODE and FREEZE via Schmitt triggers; freeze OR with gate.
5. **Update controller** — `ctl.update(p, sr)` smooths all per-sample engine data.
6. **Snapshot display** — copy current values into `dispVal[]`, `dispRt60`, `dispEchoMs`, `dispSize`.
7. **Clear sequence** — advance `clearFade` per state machine.
8. **Run voices** — per poly channel, call `voices[c].process(...)`; blend dry/wet with equal-power (`mixS` smoothed).
9. **Safety** — NaN check → reset all engines; also a short recovery mute after any limiter event.
10. **Sum / write outputs** — stereo-out sums to one pair, otherwise per-channel.
11. **Display feeds** — peak meters (one-pole decay), scope min/max bins, FFT ring buffer.

## 7. Important Functions

### `float getParam(int paramId, int attenId, int cvId)`

Reads one (param, attenuverter, CV) triple and combines them per §3. Result is clamped to `[0, 1]`.

### `void allocate(float sr)`

(Re-)initialises all voices for a new sample rate and updates `kMix` (20 ms) and `scopeBinSamples` (≈ 99 bins/s).

### `void process(const ProcessArgs &args)`

Main callback described in §6.

### `json_t *dataToJson()` / `void dataFromJson(json_t *)`

Persist `cvRangeMode`, `dispMode`, `freezeLatched`, `echoModeState`.

### `void fft()`

UI-only: in-place iterative radix-2 FFT on the display's `re`/`im` vectors.

### `void computeSpectrum(float sr, float *dbOut)`

UI-only: windowed FFT of `specBuf`, log-band aggregation, dB output (0 dB = 5 V amplitude).

## 8. Mathematical Implementation

### Equal-power dry/wet

$$
c_d = \cos\!\left(\tfrac{\pi}{2} m_s\right), \quad s_w = \sin\!\left(\tfrac{\pi}{2} m_s\right)
$$

$$
m_s \leftarrow m_s + k_{\text{mix}} \cdot (m - m_s), \quad k_{\text{mix}} = 1 - e^{-1/(0.02\, f_s)}
$$

### Clear fade

$$
\text{phase 1 (fade out): } \dot{c}_f = -10/s \quad \text{down to } 0
$$
$$
\text{phase 2 (fade in): } \dot{c}_f = +10/s \quad \text{up to } 1
$$

### CV range

$$
\text{cv} = \begin{cases}
\text{clamp}\big(v_{\text{in}}/5, -1, +1\big) & \text{bipolar} \\
v_{\text{in}}/10 & \text{unipolar}
\end{cases}
$$

### Display spectrum normalisation

$$
a_k = \sqrt{\Re^2 + \Im^2} \cdot \frac{4}{N}, \quad \text{dB}_k = 10 \log_{10}\!\left(a_k^2 / 25\right)
$$

so that a 5 V peak tone reads 0 dB.

## 9. Performance and Optimization

### CPU Usage

- 12 cells × (knob + atten + CV clamp) + 16 voices (poly, up to)
- Controller smoothing and one-pole mix smoothing add small overhead
- Plate engine: ≈ 14 taps + diffusers, per voice
- Echo engine: 1 delay read/write + feedback filter, per voice

### Memory Allocation

- No allocation in `process()` after `allocate(sr)`
- Display buffers: scope `396 × 6` floats, FFT `4096` floats
- FFT workspace pre-allocated on the UI side (`std::vector`)

### Real-time Processing

- All loops bounded by `NUM_CHANNELS` (16) or fixed sizes
- 16-voice stereo reverb fits comfortably on a single core

### Thread Safety

- Display fields written by the audio thread, read via torn reads (cosmetic)

### Numerical Stability

- Wet signal NaN check → all voices reset; brief output mute via `limiterRecovery`
- Hard clamp on outputs to `±12 V`
- CV values clamped to `[0, 1]` before use

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `VortexModule.hpp` — `Vortex::Voice`, `Vortex::Controller`, `Vortex::Params`
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`, `<vector>`

### External Classes

- `rack::Module`, `rack::ModuleWidget`, `rack::OpaqueWidget`
- `dsp::SchmittTrigger`
- `agmm::panelBase`, `agmm::bezel`, `agmm::plate`, `agmm::cell`, `agmm::screen`, `agmm::plotFrame`, `agmm::meter`, `agmm::text`, `agmm::accent`, `agmm::bright`, `agmm::violet`, `agmm::warm`, `agmm::dim`, `agmm::withA`, `agmm::ALIGN_L/C/R`

## 11. Summary

`Vortex` packs a plate reverb and a modulated echo into one module and lets you flip between them with a single button, with a full set of CV-controllable parameters per mode. The latching freeze, the clear sequence, and the polyphonic / stereo switches make it usable as a both a performance reverb and a plain send effect.

The three display modes (Wave / Bars / Spectrum) give different views of the same wet output, and the per-mode dimming of inactive cells keeps the panel readable while switching between the two engines. The selectable CV range in the context menu is a small quality-of-life touch for the modular-first user.