<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Bastion.cpp
-->

# Bastion

## 1. Overview

`Bastion` is a 16 HP multi-engine filter offering ten selectable classic filter topologies. The selected engine processes one summed audio input and produces simultaneous low-pass (LP), band-pass (BP), and high-pass (HP) outputs. Each engine has its own accent colour that appears consistently on the panel, the response curve, and the scope — so the module looks visibly different as you scroll through engines.

Key features:
- 10 engines: Moog ladder, TB-303 diode, MS-20, SEM SVF, Steiner-Parker, CEM3320, Roland OTA, EMS diode, EDP Wasp, Buchla LPG
- 3 simultaneous outputs (LP, BP, HP)
- Cutoff, resonance, drive knobs with CV and attenuverters
- Three summed inputs
- BastionEngineKnob: a fully custom-drawn engine selector knob
- Real-time response estimation: LP magnitude response drawn by an analytic transfer function per engine
- Live scope of the LP output
- Per-engine accent colour used for curve, scope, pointer and name
- Safety net: any non-finite output triggers a full state reset

## Module Screenshot

![Bastion VCV Rack Module](images/Bastion.png)

## 2. Architecture

### Main Classes

- `Bastion`: Core module class inheriting from `rack::Module`
- DSP state structs:
  - `LadderState` — 4 one-pole stages + 2 HP states
  - `SVFState` — LP/BP/HP triple for the SEM engine
  - `SPState` — 2 one-pole stages + derived BP
  - `LPGState` — vactrol envelope + one-pole LP
- DSP functions:
  - `onePoleLP`, `softClip`, `sat5`, `driveStage`, `fcLimit`, `ladderPredict`
  - `processLadder`, `processMS20`, `processSVF`, `processSteiner`, `processCEM3320`, `processRolandOTA`, `processWasp`, `processLPG`
- UI widgets:
  - `BastionPanel` — static art + live layer-1 overlay
  - `BastionEngineKnob` — custom knob
- Helpers: `engineColor`, `responseDb`, `fmtHz`, `drawText`, `plate`, `screen`, `plotFrame`, `sectionHeader`

### Inheritance

- `Bastion` inherits from `rack::Module`
- `BastionPanel` inherits from `rack::TransparentWidget`
- `BastionEngineKnob` inherits from `rack::app::Knob`

### State (grouped)

| Group | Storage |
|-------|---------|
| Ladder | `LadderState ladder` |
| SVF | `SVFState svf` |
| Steiner | `SPState sp` |
| LPG | `LPGState lpg` |
| Engine | `int lastEngine`, `float enginePulse` |
| Display | `dispCutoff`, `dispRes`, `dispDrive`, `inPeak`, `outPeak`, `scopeMin[64]`, `scopeMax[64]`, `scopePos`, `scopeAccMin`, `scopeAccMax`, `scopeCount` |

## 3. DSP Theory

### The ten engines

Each engine is a different filter topology. All share the same cutoff and resonance knobs but differ in:

| Engine | Type | Pole count | Character |
|--------|------|------------|-----------|
| Moog Ladder | 4-pole ladder | 4 | Smooth, classic |
| TB-303 Diode | Diode ladder | 4 | Aggressive, squelchy |
| MS-20 | HP + LP with global feedback | 2+2 | Screaming, unstable |
| SEM SVF | State-variable | 2 | Warm, multi-mode |
| Steiner-Parker | 2-pole multimode | 2 | Bright, SVF-like |
| CEM3320 | OTA ladder | 4 | Soft, compressed |
| Roland OTA | OTA cascade | 4 | Compressed resonance |
| EMS Diode | Diode ladder | 4 | Fuzzy, vintage |
| EDP Wasp | 3-pole driven | 3 | Gritty, driven |
| Buchla LPG | Vactrol LPG | 1 | Plucky, natural |

### Zero-delay feedback (ZDF)

The Moog, TB-303, CEM3320, Roland OTA and EMS Diode engines all use zero-delay feedback (ZDF) — a technique where the feedback is resolved analytically each sample rather than delayed by one sample. This makes the filter stable up to very high cutoff frequencies and avoids the zipper noise you'd otherwise get at high resonance.

The ZDF prediction for an N-stage cascade is:

$$
y_{\text{pred}} = \frac{G^N x + S}{1 + k G^N}
$$

where $G = g/(1+g)$ is the per-stage TPT coefficient, $S$ is the accumulated state contribution, and $k$ is the feedback amount.

### Drive

The drive stage adds even and odd harmonics via an asymmetric `tanh`:

$$
y = \tanh\left(\frac{x \cdot \text{gain}}{5} + \text{bias}\right) - \tanh(\text{bias})
$$

with $\text{gain} = e^{d \cdot 2.9}$ and $\text{bias} = d \cdot 0.10$. The bias term makes the saturation asymmetric, generating even harmonics. Output is scaled by $5/(1 + 0.45 d)$.

### MS-20 topology

The MS-20 is famously a high-pass into a low-pass with global negative feedback. The module models this exactly: two HP one-poles, then two LP one-poles, with a global feedback loop from the last LP stage back to the first HP. This is what gives the MS-20 its distinctive screaming resonance.

### Buchla LPG

The LPG is a vactrol-based low-pass gate: the vactrol has a slow release, so when the input level drops the gate closes slowly, producing the characteristic plucky decay:

$$
v_{\text{vac}} \leftarrow v_{\text{vac}} + (v_{\text{target}} - v_{\text{vac}}) \cdot \text{lag} \cdot f_s / 48000
$$

The vactrol state then drives both a VCA and the low-pass cutoff.

### Analytic response estimation

The UI's response curve is drawn from an analytic transfer function, not a live measurement:

- Moog, TB-303, CEM3320, Roland OTA, EMS Diode: $H(s) = 1/((1+s)^4 + k)$
- Wasp: $H(s) = 1/((1+s)^3 + k)$
- MS-20: $H(s) = 1/(s^2 + s/Q + 1)$
- SEM, Steiner: 2-pole SVF
- Buchla LPG: 1-pole LP at 200 + 12000·cutoff/1000

where $s = j\omega/\omega_c$ and $k = \text{res} \cdot k_{\max}$ per engine.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `ENGINE_PARAM` | 0 | 9 | 0 | Engine index |
| `CUTOFF_PARAM` | 0 | 1 | 0.5 | Cutoff (log 20 Hz .. 20 kHz) |
| `RES_PARAM` | 0 | 1 | 0.3 | Resonance |
| `DRIVE_PARAM` | 0 | 1 | 0 | Drive |
| `CUTOFF_CV_PARAM` | −1 | 1 | 0 | Cutoff CV amount |
| `RES_CV_PARAM` | −1 | 1 | 0 | Resonance CV amount |
| `DRIVE_CV_PARAM` | −1 | 1 | 0 | Drive CV amount |

### Inputs

| Input | Description |
|-------|-------------|
| `CUTOFF_INPUT` | Cutoff CV |
| `RES_INPUT` | Resonance CV |
| `DRIVE_INPUT` | Drive CV |
| `LP_INPUT` | Input 1 (summed) |
| `BP_INPUT` | Input 2 (summed) |
| `HP_INPUT` | Input 3 (summed) |

### Outputs

| Output | Description |
|--------|-------------|
| `LP_OUTPUT` | Low-pass |
| `BP_OUTPUT` | Band-pass |
| `HP_OUTPUT` | High-pass |

### Lights

- `LIGHT_ENGINE_A`: engine-change flash
- `LIGHT_ENGINE_B`: signal activity

### Enumerations

- `BastionEngine`: EngineMoog, EngineTB303, EngineMS20, EngineSEM, EngineSteiner, EngineCEM3320, EngineRolandOTA, EngineEMSDiode, EngineWasp, EngineBuchlaLPG, NumEngines
- `ParamId`: Engine, Cutoff, Res, Drive, CutoffCV, ResCV, DriveCV
- `InputId`: CutoffIn, ResIn, DriveIn, LPIn, BPIn, HPIn
- `OutputId`: LPOut, BPOut, HPOut
- `LightId`: EngineA, EngineB

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CUTOFF_INPUT` | CV | ±10 V | Cutoff modulation (0.2 oct/V) |
| `RES_INPUT` | CV | ±10 V | Resonance modulation |
| `DRIVE_INPUT` | CV | ±10 V | Drive modulation |
| `LP_INPUT` | Audio | ±10 V | Input 1 |
| `BP_INPUT` | Audio | ±10 V | Input 2 |
| `HP_INPUT` | Audio | ±10 V | Input 3 |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `LP_OUTPUT` | Audio | ±12 V | Low-pass |
| `BP_OUTPUT` | Audio | ±12 V | Band-pass |
| `HP_OUTPUT` | Audio | ±12 V | High-pass |

### Bypass

`configBypass(LP_INPUT, LP_OUTPUT)` — so with no cables, input 1 passes straight through to the LP output.

## 6. Signal Processing Pipeline

1. Engine change detection — If the engine knob changed, reset all DSP state and flash the engine LED.
2. Parameter read — Knobs, CVs, attenuverters.
3. Cutoff computation — `hz = 20 * 1000^base`, then `hz *= 2^(cvAmt * cv * 0.2)` (0.2 oct/V), clamped to 10 Hz .. 20 kHz.
4. Resonance and drive — Sum of knob and CV, clamped to 0..1.
5. Input sum — Sum all three inputs.
6. Engine dispatch — `switch(engine)` to the appropriate DSP function.
7. Output assignment — Per-engine LP/BP/HP derivation.
8. Safety net — If any output is non-finite, reset all state and output silence.
9. Clamp — Clamp to ±12 V.
10. Write — Set the three output voltages.
11. Meters — Peak followers for input and output.
12. Scope — Sample the LP output into the ring buffer at ~30 ms/bin.
13. LEDs — Engine-change pulse and output activity.

## 7. Important Functions

### `computeCutoff(float base, float cvAmt, float cv)`

```cpp
float norm = clamp(base, 0, 1);
float hz = 20.f * std::pow(1000.f, norm);
float cvShift = cvAmt * cv * 0.2f;
hz *= std::pow(2.f, cvShift);
return clamp(hz, 10.f, 20000.f);
```

### `ladderPredict(const float *s, int n, float G, float x, float k)`

The ZDF prediction used by every ladder engine:

```cpp
float a = 1.f - G;
float S = 0.f, Gn = 1.f;
for (int i = 0; i < n; ++i) {
    S = S * G + a * s[i];
    Gn *= G;
}
return (Gn * x + S) / (1.f + k * Gn);
```

### `processLadder(float in, LadderState &st, float fc, float res, float drive, float fs, bool diode)`

The shared 4-pole ladder engine (Moog, TB-303, EMS Diode). `diode = true` switches to the diode-ladder feedback saturation and slightly lower feedback coefficient.

### `processMS20(float in, LadderState &st, float fc, float res, float drive, float fs)`

The MS-20 engine. Uses the same `LadderState` but with two HP stages before the two LP stages, and an explicit ZDF prediction of the HP-HP-LP-LP cascade.

### `processLPG(float in, LPGState &st, float fc, float res, float drive, float fs)`

The Buchla LPG engine. Uses the vactrol model described in §3.

### `responseDb(int engine, float f, float fc, float res)`

The analytic transfer function used only for the UI's response curve.

## 8. Mathematical Implementation

### Exponential cutoff

$$
f_c = 20 \cdot 1000^{\text{base}} \cdot 2^{0.2 \cdot \text{amt} \cdot \text{cv}}
$$

clamped to 10 Hz .. 20 kHz.

### TPT one-pole

$$
G = \frac{g}{1+g},\quad g = \tan\left(\frac{\pi f_c}{f_s}\right)
$$

$$
y = v + \text{state},\quad v = (x - \text{state})G,\quad \text{state} = y + v
$$

### ZDF ladder feedback

$$
u = x - \text{sat}(k \cdot y_{\text{pred}})
$$

where $y_{\text{pred}}$ is the ZDF prediction. This gives a stable, alias-free ladder.

### SEM SVF

$$
\text{hp} = x - \text{lp} - q\,\text{bp},\quad \text{bp} = \text{softClip}(f\,\text{hp} + \text{bp}),\quad \text{lp} = \text{softClip}(f\,\text{bp} + \text{lp})
$$

with $f = 2\sin(\pi f_c / f_s)$ and $q = 1 - \text{res}$ (clamped to 0.02 .. 1).

### Buchla LPG

$$
v_{\text{vac}} \leftarrow v_{\text{vac}} + (v_{\text{target}} - v_{\text{vac}}) \cdot \text{lag} \cdot f_s/48000
$$

$$
\text{out} = \text{LPF}\left(\text{VCA}(x, v_{\text{vac}}), 200 + 12000 v_{\text{vac}}\right)
$$

The `lag` is 1.5 ms for fast attack, 30 ms for slow release.

### Drive stage

$$
y = \tanh\left(\frac{x \cdot \text{gain}}{5} + \text{bias}\right) - \tanh(\text{bias}),\quad \text{gain} = e^{2.9 d}, \text{bias} = 0.10 d
$$

Output scaled by $5/(1 + 0.45 d)$.

### Soft clip

$$
\text{sat}_5(x) = 5\tanh(x/5)
$$

## 9. Performance and Optimization

### CPU Usage

- Per engine: ~30–50 flops per sample
- UI: the response curve is computed once per frame (96 samples × complex math ≈ 10 K flops)
- Scope: one min/max update per stride (~30 ms of audio), negligible

### Memory Allocation

- All state is fixed-size (no `std::vector`)
- No allocation in `process()`
- Scope buffers are 64 floats each, embedded in the module

### Buffers

- `LadderState`: 6 floats
- `SVFState`: 3 floats
- `SPState`: 3 floats
- `LPGState`: 2 floats
- `scopeMin[64]`, `scopeMax[64]`: 128 floats
- Total < 200 floats

### Real-time Processing

- All engines are constant-time per sample
- The engine switch happens once per sample (predictable branch)
- No allocation, no I/O

### Thread Safety

- All DSP state is written by the audio thread and read by the UI thread via torn reads
- The engine index is read once per sample; any change resets state — fine because engine changes are user-initiated and rare

### Numerical Stability

- `tanh` soft clipping at every output
- `tanh` soft clipping on the ZDF feedback (`sat5(k * y4)`)
- Non-finite safety net resets all state
- Output clamped to ±12 V

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<complex>`, `<memory>`, `<string>`, `<vector>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::app::Knob`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::plate`, `agmm::text`, `agmm::plotFrame`, `agmm::screen`, `agmm::sectionHeader`, `agmm::meter`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::textCol`

## 11. Summary

`Bastion` is a multi-engine filter that packs ten classic topologies into a single 16 HP module. Its design is both encyclopedic (it reproduces the character of the Moog ladder, TB-303, MS-20, SEM, Steiner-Parker, CEM3320, Roland OTA, EMS diode, EDP Wasp, and Buchla LPG) and practical (the three simultaneous outputs, CV-able cutoff/resonance/drive, and per-engine accent colour make it easy to use in a real patch).

The DSP is built from reusable primitives — TPT one-pole, ZDF prediction, drive stage, soft clip — that are composed into each engine's topology. The ZDF technique for the ladder engines is the key to their stability and musicality at high resonance. The analytic response curves on the display are a nice touch: they're not live measurements but they accurately reflect each engine's character.

The module's main limitation is that it has only one input (with three summed jacks), so you can't use it as a true stereo filter. For stereo use, you'd need two Bastions, or use one of the AGMM stereo filter modules. It's also CPU-heavy for a filter (about 100 flops/sample), but still very cheap compared to a full synth voice.
