<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Stagger.cpp
-->

# Stagger

## 1. Overview

`Stagger` is a 20 HP polyphonic VCO with a stereo "chaos" pitch drift. Each voice runs two independent oscillators (L and R) whose pitch is the base pitch plus a slow, filtered random drift — so the two channels slowly detune against each other, giving a wide, unstable, "staggering" tone.

Key features:
- Polyphonic VCO with independent L/R drift
- 4 waveforms: Sine, Triangle, Saw, Square (minBLEP anti-aliased)
- Pulse-width control for square
- Chaos multiplier (×1, ×2, ×3, ×5, ×10)
- CV range selectable (±5 V or ±10 V)
- CV + attenuverter on chaos, waveform and pulse width
- 1 V/oct pitch input (no attenuverter)
- Reset input for the chaos drift
- Editable frequency field (Hz or note name, with semitone offsets)
- Scope + chaos drift history display

## Module Screenshot

![Stagger VCV Rack Module](images/Stagger.png)

## 2. Architecture

### Main Classes

- `Stagger`: Core module class inheriting from `rack::Module`
- `StFreqQ`: Custom ParamQuantity for the frequency knob
- `STPanel`: Static panel artwork
- `STDisplay`: Live scope + drift history display

### Inheritance

- `Stagger` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### Structures

- `Voice { phase, lastPhase, blepRising, blepFalling, blepSaw, phaseR, lastPhaseR, blepRisingR, blepFallingR, blepSawR }` — one voice with L and R channels

### State (grouped)

| Group | Storage |
|-------|---------|
| Voices | `std::vector<Voice> voices` |
| Chaos drift | `float chaosDrift`, `chaosDriftR` |
| RNG | `std::mt19937 rng`, `std::uniform_real_distribution<float> dist` |
| Transport | `bool cvRangeTenVolt`, `float lastResetVoltage` |
| Display | `float dispFreq, dispPw, dispPhase, dispChaos`, `int dispWave`, `int dispChannels`, `float driftHist[128]`, `int driftHead`, `float driftTimer, driftPeak` |

## 3. DSP Theory

### Stagger concept

The name "Stagger" refers to the way the two channels slowly drift apart in pitch. Each voice runs two oscillators — one for L, one for R — with independent filtered random drift:

$$
f_L = f_{\text{base}} + \text{chaos}_L \cdot \text{mult}
$$

$$
f_R = f_{\text{base}} + \text{chaos}_R \cdot \text{mult}
$$

where $\text{chaos}_L$ and $\text{chaos}_R$ are independent one-pole filtered white noise:

$$
\text{chaos}_L \leftarrow (1 - \alpha) \cdot \text{chaos}_L + \alpha \cdot w_L
$$

with $\alpha = \min(2\pi f_c/f_s, 0.5)$ and $f_c = 1.5$ Hz. The slow filter makes the drift smooth (like tape wow/flutter).

### Pitch computation

$$
\text{pitch} = \text{basePitch} + \text{V/Oct} + \text{chaos} \cdot \text{depth} \cdot \text{mult}
$$

### Fast 2^x approximation

Instead of `pow(2, x)`, the module uses a bit-twiddling approximation:

```cpp
union { float f; uint32_t i; } u;
u.i = (uint32_t)((1 << 23) * (x + 126.94269504f));
return u.f;
```

This is accurate to ~1e-3 relative error, which is good enough for pitch.

### Waveforms

- **Sine**: $\sin(2\pi\phi)$
- **Triangle**: `(φ < 0.5 ? 4φ − 1 : 3 − 4φ)` — equivalent to $1 - 4|\phi - 0.5|$, trough at φ=0 and φ=1, peak at φ=0.5
- **Saw**: $1 - 2\phi$, with minBLEP correction at wrap
- **Square**: $(\phi < pw ? 1 : -1)$, with minBLEP correction at both edges

### minBLEP

The minBLEP (minimum band-limited step) is a correction applied at discontinuities to reduce aliasing. For a saw wrap, it inserts a 2.0 discontinuity; for a square edge, it inserts a ±2.0 discontinuity. The DSP then adds the minBLEP output to the raw waveform.

### Chaos multiplier

The multiplier is selected from a discrete table:

$$
\text{mult} = \{1, 2, 3, 5, 10\}
$$

So the chaos drift can be subtle (×1) or extreme (×10).

### Pulse width

The square pulse width is adjustable from 0.01 to 0.99:

$$
\text{pw} = \text{clamp}(\text{knob} + \text{pwCV} \cdot \text{atten} / 10, 0.01, 0.99)
$$

### CV range

The CV range switch toggles between ±5 V (scale = 1) and ±10 V (scale = 0.5). This affects all CV inputs.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `PITCH_PARAM` | 0 | 1 | ≈C4 | Frequency (10 Hz .. 6 kHz, logarithmic) |
| `CHAOS_PARAM` | 0 | 1 | 0 | Chaos depth |
| `WAVE_PARAM` | 0 | 3 | 0 | Waveform |
| `PW_PARAM` | 0 | 1 | 0.5 | Pulse width |
| `CHAOS_MULT_PARAM` | 0 | 4 | 0 | Chaos multiplier |
| `CHAOS_ATTEN_PARAM` | −1 | 1 | 0 | Chaos CV attenuverter |
| `WAVE_ATTEN_PARAM` | −1 | 1 | 0 | Wave CV attenuverter |
| `PW_ATTEN_PARAM` | −1 | 1 | 0 | Pulse width CV attenuverter |

### Switches

| Parameter | Description |
|-----------|-------------|
| `CV_RANGE_PARAM` | ±5 V or ±10 V |

### Inputs

| Input | Description |
|-------|-------------|
| `CV_INPUT` | 1 V/oct pitch (polyphonic) |
| `CHAOS_CV_INPUT` | Chaos CV |
| `WAVE_CV_INPUT` | Waveform select CV |
| `PW_CV_INPUT` | Pulse width CV |
| `CHAOS_RESET_INPUT` | Reset chaos drift |

### Outputs

| Output | Description |
|--------|-------------|
| `VCO_L_OUTPUT` | Left (polyphonic) |
| `VCO_R_OUTPUT` | Right (polyphonic) |

### Lights

- `CHAOS_LIGHT` — Chaos activity

### Enumerations

- `ParamId`: Pitch, Chaos, ChaosMult, Wave, PW, ChaosAtten, WaveAtten, PWAtten, CVRange
- `InputId`: CV_INPUT (V/Oct), ChaosCV, WaveCV, PWCV, ChaosReset
- `OutputId`: OutL, OutR
- `LightId`: Chaos

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CV_INPUT` | CV | 1 V/oct | Pitch (polyphonic) |
| `CHAOS_CV_INPUT` | CV | ±10 V | Chaos modulation |
| `WAVE_CV_INPUT` | CV | ±10 V | Waveform selection |
| `PW_CV_INPUT` | CV | ±10 V | Pulse width modulation |
| `CHAOS_RESET_INPUT` | Gate | > 1 V | Reset chaos drift |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `VCO_L_OUTPUT` | Poly Audio | ±5 V | Left (polyphonic) |
| `VCO_R_OUTPUT` | Poly Audio | ±5 V | Right (polyphonic) |

## 6. Signal Processing Pipeline

1. **Read base pitch** from FREQ knob.
2. **Read controls** — Chaos, wave, pulse width, chaos multiplier, CV attens.
3. **CV range scale** — 1.0 for ±5 V, 0.5 for ±10 V.
4. **Chaos reset** — On rising edge (>1 V with previous ≤1 V), zero both drift values.
5. **Update chaos** — Two independent one-pole filters on white noise.
6. **Determine channels** — `max(1, CV_INPUT channels)`; set both outputs' channel counts; grow the voices vector if needed.
7. **For each voice**:
   - Read V/Oct, chaos CV, wave CV, PW CV.
   - Compute total pitch for L and R (independent chaos drift added per channel).
   - Compute frequency with fast 2^x, clamped to 10 Hz..6 kHz.
   - Compute phase increment.
   - Determine wave index (quantized to 0..3).
   - Determine PW (clamped to 0.01..0.99).
   - Render L and R with minBLEP-corrected waveforms; write ±5 V outputs.
   - For channel 0, update display fields and peak-hold `driftPeak`.
8. **LED** — `clamp(|chaosDrift| * chaosKnob * gain, 0, 1)`, smooth-brightness.
9. **Display** — At ~60 Hz, push the current `driftPeak * gain` into `driftHist`, advance `driftHead`, reset `driftPeak`.

## 7. Important Functions

### `StFreqQ::mapToFreq(float v)`

Maps a normalised knob value (0..1) to Hz on a 10 Hz..6 kHz log curve: `10 * 600^v`.

### `StFreqQ::mapFromFreq(float f)`

Inverse of `mapToFreq`; clamps `f` to 10..6000 Hz.

### `StFreqQ::getDisplayValueString()`

Returns the current frequency formatted as `"%.1f Hz"`.

### `StFreqQ::setDisplayValueString(std::string s)`

Parses the editable text field. Accepts:
- A raw frequency ("440", "261.63") → set directly
- A note name with octave ("A4", "C#3", "Bb2") → converted to Hz via A4 = 440 Hz
- A signed semitone offset from C4 ("+12", "-5") → `FREQ_C4 * 2^(st/12)`

Whitespace is ignored; unrecognised text falls back to the base class.

### `Stagger::onReset(const ResetEvent &e)`

Clears both chaos drifts, the drift peak and history, and drops all voices so they are recreated on the next `process()`.

### `Stagger::process(const ProcessArgs &args)`

Main callback, described in §6.

## 8. Mathematical Implementation

### Chaos drift (one-pole filter per channel)

$$
\text{drift} \leftarrow (1 - \alpha)\,\text{drift} + \alpha\,w, \quad w \sim \mathcal{U}[-1,1]
$$

$$
\alpha = \min\!\left(2\pi \cdot 1.5 \cdot \frac{1}{f_s},\ 0.5\right)
$$

### Pitch and frequency

$$
\text{pitch}_{L/R} = \text{basePitch} + \text{V/Oct} + \text{chaos}_{L/R} \cdot \text{depth} \cdot \text{mult}
$$

$$
f_{L/R} = \text{FREQ\_C4} \cdot 2^{\text{pitch}_{L/R}}, \quad \Delta\phi = f \cdot \Delta t
$$

### LED intensity

$$
\text{intensity} = \operatorname{clamp}\!\big(|\text{drift}| \cdot \text{chaosKnob} \cdot 160,\ 0,\ 1\big)
$$

### CV range scale

$$
\text{scale} = \begin{cases} 1.0 & \text{±5 V range} \\ 0.5 & \text{±10 V range} \end{cases}
$$

## 9. Performance and Optimization

### CPU Usage

- Per sample per voice: 2 oscillators (L and R) with minBLEP, each ~30–60 flops → ~100 flops/voice/sample
- Scales linearly with polyphony
- Per sample once: 2 noise draws, 2 one-pole filters, LED update, output writes

### Memory Allocation

- Voices vector resized only when the channel count grows; no allocation once warmed up
- Fixed-size `driftHist[128]`
- `disp*` display state is fixed-size

### Buffers

- `driftHist[128]` — scroll history for the chaos drift display

### Real-time Processing

- All loops bounded
- No allocation in steady state, no blocking

### Thread Safety

- Display fields (`dispFreq`, `dispPw`, `dispPhase`, `dispWave`, `dispChannels`, `driftHist`, `driftHead`) are written by the audio thread and read by the UI thread via torn reads; cosmetic only
- Voice vector is only touched by the audio thread

### Numerical Stability

- Frequencies clamped to 10..6000 Hz
- Pulse width clamped to 0.01..0.99
- Chaos depth clamped to 0..1
- Waveform selection clamped to 0..3
- Filter coefficient α clamped to ≤0.5

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<cstdint>`, `<vector>`, `<map>`, `<string>`, `<regex>`, `<algorithm>`, `<random>`
- `"dsp/minblep.hpp"` — `rack::dsp::MinBlepGenerator`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `rack::dsp::MinBlepGenerator<16,16>`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`

## 11. Summary

`Stagger` is a polyphonic stereo VCO whose defining feature is the slow, filtered random "chaos" drift applied independently to the left and right channels. The result is a wide, unstable tone that slowly staggers out of tune with itself — great for pads, drones and anything that wants width and instability without explicit detune controls.

The DSP is straightforward but well-featured: four anti-aliased waveforms (minBLEP on saw and square), pulse-width control, a chaos multiplier switch, CV with attenuverters on chaos/wave/PW, a selectable CV range, and a chaos-reset input. The frequency field is editable and accepts Hz, note names, or semitone offsets from C4.

The display shows the current waveform (two cycles) with a phase marker, and a scrolling history of the chaos drift — so you can visually confirm the amount of instability at a glance.

Compared with a plain VCO, Stagger trades precise tuning for deliberate instability. The pitch input is polyphonic and has no attenuverter, so pitch modulation must be done externally; the chaos depth is the primary modulator here. If you need fine pitch modulation, see other AGMM VCOs with a dedicated pitch-CV attenuverter.