<!--
AGMM AUTOMATIC DOCUMENTATION
Source: LSDrone.cpp
-->

# LSDrone

## 1. Overview

`LSDrone` is a 36 HP cathedral-style drone synthesizer with two detunable oscillators, a resonant low-pass, an LFO, a drift generator, a modulated stereo delay, a reverb with shimmer, an echo, and a final low-pass filter. Each of the 23 main parameters has a dedicated CV input and attenuverter, and the display can show the output waveform, a bar-graph spectrum, or a continuous spectrum curve.

Key features:
- 2 detunable oscillators
- Resonant LPF + LFO + drift
- Modulated stereo delay + shimmer reverb + echo
- Final low-pass filter
- 23 parameters, each with CV and attenuverter
- 3 display modes: waveform, bar spectrum, curve spectrum
- 4096-point FFT analyzer
- CV range selectable (bipolar/unipolar) via menu

## Module Screenshot

![LSDrone VCV Rack Module](images/LSDrone.png)

## 2. Architecture

### Main Classes

- `LSDrone`: Core module class inheriting from `rack::Module`
- `LSDroneEngine`: The full DSP chain
- `LSAnalyzer`: 4096-point FFT analyzer
- `LSPanelBackground`: Static panel artwork
- `LSDisplayWidget`: Live display with 3 view modes
- `LSCvRangeWidget`: Clickable CV range toggle
- `LSViewWidget`: Display mode switch

### Inheritance

- `LSDrone` inherits from `rack::Module`
- All UI widgets inherit from `rack::TransparentWidget`

### Structures

- `LSDroneEngine::Oscillator { float phase, freq, fine; }`
- `LSCellDef { const char *name; int param, attn, cv; }` — one row of the 23-param table

### State (grouped)

| Group | Storage |
|-------|---------|
| Engine | `LSDroneEngine engine` |
| Display | `float scopeMin[160]`, `scopeMax[160]`, `scopeHead`, `scopeCount`, `scopeBMin`, `scopeBMax`, `peakL`, `peakR` |
| Analyzer | `float fftBuf[4096]`, `int fftHead`, `float dispSampleRate` |
| CV mode | `CVRange cvRangeMode` |
| View mode | `ViewMode viewMode` |

## 3. DSP Theory

### Oscillators

Each oscillator produces a sine at:

$$
f_A = 261.63 \cdot 2^{p_A + \text{fine}_A/12}
$$

$$
f_B = 261.63 \cdot 2^{p_B + \text{fine}_B/12 + \text{detune}/12}
$$

where `p_A`, `p_B` are the V/Oct pitch CVs and `detune` is the detune amount.

### LFO

A single sine LFO modulates both oscillators' pitch:

$$
\text{lfoMod} = \sin(2\pi \phi_{\text{lfo}}) \cdot \text{lfoDepth}
$$

### Drift

A slow random walk adds pitch wobble:

$$
\text{driftState} \leftarrow \text{driftState} \cdot 0.998 + u \cdot 0.008, \quad u \sim \text{Uniform}[-0.5, 0.5]
$$

Clamped to ±0.2, scaled by the DRIFT knob.

### Filters

Two filters:
- **Pre-filter**: one-pole LP at `50 + cutoff * 4000` Hz
- **Final filter**: 2-pole RBJ LP with resonance

### Modulated delay

A stereo delay with LFO modulation on the delay time:

$$
D(n) = \text{delayTime} + 0.1 \cdot \text{modDepth} \cdot (0.5 + 0.5 \sin(2\pi \phi_{\text{mod}}))
$$

The delay line has a feedback tap (0.6 default) and an all-pass diffuser (0.4 default).

### Reverb

A single-tap feedback reverb with decay and damping:

$$
\text{revBuf} \leftarrow \text{revIn} + \text{damped} \cdot \text{revDecay}
$$

### Shimmer

A 440 Hz sine multiplied by the reverb output and the SHIMMER knob, then added to the wet signal. This is a simple but effective shimmer: it's not a pitch-shifted copy of the reverb, it's just a fixed tone modulated by the reverb's amplitude.

### Echo

A stereo delay with its own time and feedback, mixed into the wet signal.

### Final LPF

2-pole RBJ low-pass with resonance.

## 4. Parameters and Controls

### 23 parameter knobs (each with CV + attenuverter)

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `PARAM_FINE_A` | −12 | 12 | 0 | Osc A fine tune (semitones) |
| `PARAM_FINE_B` | −12 | 12 | 0 | Osc B fine tune (semitones) |
| `PARAM_DETUNE` | −24 | 24 | 0 | Osc B detune (semitones) |
| `PARAM_MIX` | 0 | 1 | 0.5 | A/B mix |
| `PARAM_CUTOFF` | 0 | 1 | 0.5 | Filter cutoff |
| `PARAM_RESO` | 0 | 1 | 0.3 | Filter resonance |
| `PARAM_LFO_RATE` | 0.01 | 5 | 0.2 | LFO rate (Hz) |
| `PARAM_LFO_DEPTH` | 0 | 2 | 0.5 | LFO depth (semitones) |
| `PARAM_DRIFT` | 0 | 1 | 0.3 | Drift amount |
| `PARAM_DELAY_TIME` | 0.1 | 2.5 | 0.8 | Delay time (s) |
| `PARAM_DELAY_FEED` | 0 | 0.98 | 0.6 | Delay feedback |
| `PARAM_DELAY_DIFF` | 0 | 1 | 0.4 | Delay diffusion |
| `PARAM_REV_DECAY` | 0.5 | 0.99 | 0.85 | Reverb decay |
| `PARAM_REV_DAMP` | 0 | 1 | 0.5 | Reverb damping |
| `PARAM_WASH` | 0 | 1 | 0.6 | Wash mix |
| `PARAM_SHIMMER` | 0 | 1 | 0.2 | Shimmer amount |
| `PARAM_MOD_RATE` | 0.02 | 2 | 0.15 | Modulation rate (Hz) |
| `PARAM_MOD_DEPTH` | 0 | 1 | 0.3 | Modulation depth |
| `PARAM_ECHO_TIME` | 0.05 | 1.5 | 0.4 | Echo time (s) |
| `PARAM_ECHO_FEED` | 0 | 0.95 | 0.5 | Echo feedback |
| `PARAM_ECHO_MIX` | 0 | 1 | 0.4 | Echo mix |
| `PARAM_LPF_CUTOFF` | 0 | 1 | 0.8 | Final LPF cutoff |
| `PARAM_LPF_RESO` | 0 | 1 | 0.2 | Final LPF resonance |

### Attenuverters

23 attenuverters (one per parameter), range −1..+1, default 0.5.

### Inputs

- 23 CV inputs (one per parameter)
- `CV_A` — V/Oct for osc A
- `CV_B` — V/Oct for osc B

### Outputs

- `OUTPUT_LEFT` — Left
- `OUTPUT_RIGHT` — Right

### Enumerations

- `CVRange`: BIPOLAR_M5_5, UNIPOLAR_0_10
- `ViewMode`: VIEW_WAVE, VIEW_BARS, VIEW_SPECTRUM
- `ParamId`: 23 params + 23 attens
- `InputId`: 23 CVs + 2 pitch
- `OutputId`: Left, Right

## 5. Inputs and Outputs

### Inputs

23 CV inputs (one per parameter) + 2 pitch CVs.

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUTPUT_LEFT` | Audio | ±10 V | Left |
| `OUTPUT_RIGHT` | Audio | ±10 V | Right |

### Menu

- CV range: bipolar (−5..+5) or unipolar (0..+10)
- Display mode: waveform, bars, spectrum

## 6. Signal Processing Pipeline

1. **Read all 23 params with CV + attenuverter** — `getParamWithCV()` returns `knob + cv_scaled * atten`, clamped.
2. **Run the engine** — `engine.process(cvA, cvB, leftOut, rightOut)`.
3. **Write outputs** — Left × 5, Right × 5.
4. **Update display** — Peaks, FFT ring buffer, scope ring buffer.

## 7. Important Functions

### `getParamWithCV(ParamId paramId, InputId cvId, ParamId attenId, float minV, float maxV)`

Reads a knob, its CV, and its attenuverter, combines them, clamps to `[minV, maxV]`.

```cpp
float knob = params[paramId].getValue();
float cv = inputs[cvId].getVoltage();
float atten = params[attenId].getValue();
float cvScaled = (cvRangeMode == UNIPOLAR_0_10) ? (cv / 10.f) : clamp(cv / 5.f, -1, 1);
float value = knob + cvScaled * atten;
return clamp(value, minV, maxV);
```

### `LSDroneEngine::process(float cvA, float cvB, float &leftOut, float &rightOut)`

The full DSP chain described in §3.

### `LSAnalyzer::update(const float *ring, int head)`

Runs an in-place radix-2 FFT on the 4096-sample window, converts magnitudes to dB with fast-attack/slow-release smoothing.

### `LSDrone::process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Oscillator frequency

$$
f_A = 261.63 \cdot 2^{p_A + \text{fine}_A/12}, \quad f_B = 261.63 \cdot 2^{p_B + \text{fine}_B/12 + \text{detune}/12}
$$

### LFO

$$
\text{lfoMod} = \sin(2\pi \phi_{\text{lfo}}) \cdot \text{lfoDepth}
$$

### Drift

$$
\text{driftState} \leftarrow \text{driftState} \cdot 0.998 + u \cdot 0.008, \quad u \sim \text{Uniform}[-0.5, 0.5]
$$

### Pre-filter cutoff

$$
f_c = 50 + \text{cutoff} \cdot 4000 \text{ Hz}
$$

### Delay time

$$
D(n) = \text{delayTime} + 0.1 \cdot \text{modDepth} \cdot (0.5 + 0.5\sin(2\pi \phi_{\text{mod}}))
$$

### Final LPF coefficients

Standard RBJ 2-pole LP with `Q = 1 / (2 (1 + reso * 10))`.

### Waveform display

`scopeMin`/`scopeMax` are min/max of the mono output, updated every 2.5 ms.

### Analyzer

4096-point FFT of the recent output, converted to dB with smoothing. The bars view groups the bins into 48 log-spaced bands; the spectrum view uses 232 log-spaced points.

## 9. Performance and Optimization

### CPU Usage

- Audio thread: 2 oscillators + 2 filters + LFO + drift + delay + reverb + echo ≈ 400 flops/sample
- UI thread: 4096-point FFT (only when in BARS or SPECTRUM mode) ≈ 50 K flops/frame at 60 fps
- Display: 160-point scope + 48 bars + 232-point spectrum

### Memory Allocation

- Engine buffers: delay (3 × sr samples), reverb (2 × sr samples), echo (2 × sr samples) ≈ 5 MB
- FFT buffers: 4096 floats × 3 = 48 KB
- No allocation in `process()`

### Buffers

- Delay: 3 × 48000 floats ≈ 576 KB
- Reverb: 2 × 48000 floats ≈ 384 KB
- Echo: 2 × 48000 floats ≈ 384 KB
- FFT: 3 × 4096 floats ≈ 48 KB

### Real-time Processing

- All loops bounded
- No allocation, no blocking
- FFT is `unique_ptr` (only allocated once)

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads

### Numerical Stability

- `tanh` soft limit on output
- All oscillators are sine waves (bounded)
- Delay feedback 0.98 < 1
- Final LPF with resonance doesn't self-oscillate due to RBJ design

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`, `<memory>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `rack::dsp::SchmittTrigger`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::plate`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::meter`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`

## 11. Summary

`LSDrone` is the "big" drone synth of the AGMM collection. It combines two detunable oscillators, a resonant filter, an LFO, drift, a modulated delay, a shimmer reverb, an echo, and a final LPF into a single 36 HP module. Every parameter has CV and an attenuverter, so it's extremely playable.

The DSP is not particularly novel — the oscillators are simple sines, the filters are textbook, the reverb is a single-tap feedback — but the combination is greater than the sum of its parts. The shimmer (a 440 Hz sine modulated by the reverb) is a nice touch that adds air without needing a pitch shifter.

The display is the standout feature: 3 modes (waveform, bars, spectrum) driven by a 4096-point FFT analyzer, so you can see exactly what the module is producing. The CV range and view mode are both persisted in the patch.

The main limitation is CPU: with the FFT analyzer running, the UI thread does ~50 K flops/frame, which can strain slower machines. But for a cathedral drone synth, the visual feedback is worth it.
