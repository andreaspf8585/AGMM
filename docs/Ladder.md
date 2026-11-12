<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Ladder.cpp
-->

# Ladder

## 1. Overview

`Ladder` is a 24 HP VCO that renders the classic Shepard–Risset glissando: a stack of octave-spaced partials whose amplitudes are weighted by a Gaussian window in log-frequency space. As the SPEED knob advances, all partials slide together; partials leave the window while new ones fade in, so the ear hears a pitch that climbs (or descends) forever.

Key features:
- Up to 20 octave-spaced partials
- Gaussian amplitude weighting in log-frequency space (sigma scaled by partial count)
- Shepard sweep with ascending or descending direction
- 4 waveforms: Sine, Saw, Triangle, Step
- Pulse-width control for Step waveform
- Stereo output with per-partial panning (low partials left, high partials right)
- Oscilloscope and partial-spectrum display
- Text field for entering frequency or note name via context menu
- Display-mode toggle (Note name or Hz)

## Module Screenshot

![Ladder VCV Rack Module](images/ladder.png)

## 2. Architecture

### Main Classes

- `Ladder`: Core module class inheriting from `rack::Module`
- `FreqQuantity`, `WaveQuantity`, `NumOscsQuantity`: Custom ParamQuantity classes
- `LDPanel`: Static panel artwork
- `LDDisplay`: Live scope + partial spectrum

### Inheritance

- `Ladder` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Phases | `float phases[20]` |
| Sweep | `float sweep` |
| Partial count | `int prevNumOscs` |
| Display | `float dispPitchSemis`, `dispSweep`, `dispAmp[20]`, `dispPan[20]`, `dispFreqs[20]`, `dispPhases[20]`, `int dispWave`, `float dispPulseWidth`, `int dispNumOscs`, `bool dispDescending` |

## 3. DSP Theory

### The Shepard tone

A Shepard tone is a stack of octave-spaced partials whose amplitudes are weighted by a fixed Gaussian window in log-frequency space:

$$
A_i = \exp\left(-\frac{(\log_2 f_i - \log_2 f_0)^2}{2\sigma^2}\right)
$$

where $f_i = f_0 \cdot 2^{i - N/2}$ for $i = 0, \ldots, N-1$.

The key insight: because the Gaussian is fixed in log-frequency, shifting all partials by an octave produces the same amplitude pattern (just with the partials shifted by one index). This is what makes the pitch appear to climb or descend endlessly.

### Sigma scaling

To keep all chosen partials audible, sigma scales with N:

$$
\sigma = \max\left(1, \frac{N-1}{4}\right)
$$

At N = 6, sigma = 1.25, which is the classic Shepard feel. At N = 20, sigma = 4.75, which makes the window wide enough that even the outermost partial (at exp(−2) ≈ 0.135 of the peak) is audible.

### Sweep

The sweep advances at `speed * 4` semitones/sec:

$$
\text{sweep} \leftarrow \text{sweep} + \text{sweepRate} \cdot \Delta t \pmod{12}
$$

The modulo-12 wrap keeps the sweep in one octave, but the partials keep sliding.

### Partial synthesis

For each partial:

$$
\phi_i \leftarrow \phi_i + f_i \cdot \Delta t \pmod{1}
$$

$$
s_i = \text{waveform}(\phi_i)
$$

### Stereo panning

Each partial is panned by its position in the Gaussian window:

$$
\text{pan}_i = \text{clamp}\left(\frac{\log_2(f_i/f_0)}{\sigma} \cdot \text{stereoWidth}, -1, 1\right)
$$

Low partials go left, high partials go right.

### RMS normalization

The output is normalized by the RMS sum of amplitudes:

$$
\text{out} \leftarrow \frac{1.8 \cdot \text{out}}{\sqrt{\sum_i A_i^2}}
$$

This keeps the output level consistent from N = 1 to N = 20.

### Waveforms

- **Sine**: $\sin(2\pi\phi)$
- **Saw**: $2\phi - 1$
- **Triangle** (with boosted 3rd harmonic): $(1 - 4|\phi - 0.5| - 0.15\cos(6\pi\phi)) / 1.15$
- **Step** (DC-free pulse): $(\phi < pw ? 1 : -1) - (2pw - 1)$

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `FREQ_PARAM` | −48 | 48 | 0 | Base frequency (semitones from C4) |
| `FINE_PARAM` | −100 | 100 | 0 | Fine tune (cents) |
| `WAVE_PARAM` | 0 | 3 | 0 | Waveform |
| `PULSE_WIDTH_PARAM` | 0.05 | 0.95 | 0.5 | Pulse width (Step only) |
| `LADDER_SPEED_PARAM` | 0 | 1 | 0.3 | Glissando rate |
| `NUM_OSCS_PARAM` | 1 | 20 | 6 | Number of partials |
| `DIRECTION_PARAM` | 0 | 1 | 0 | Ascending / Descending |

### Inputs

| Input | Description |
|-------|-------------|
| `VOCT_INPUT` | V/oct |

### Outputs

| Output | Description |
|--------|-------------|
| `OUT_L_OUTPUT` | Left |
| `OUT_R_OUTPUT` | Right |

### Enumerations

- `FreqDisplay`: DISP_NOTE, DISP_HZ
- `ParamId`: Freq, Fine, Wave, PulseWidth, Speed, NumOscs, Direction
- `InputId`: VOct
- `OutputId`: OutL, OutR

## 5. Inputs and Outputs

### Input

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `VOCT_INPUT` | CV | 1 V/oct | Pitch CV |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `OUT_L_OUTPUT` | Audio | ±5 V | Left (low partials) |
| `OUT_R_OUTPUT` | Audio | ±5 V | Right (high partials) |

## 6. Signal Processing Pipeline

1. **Read controls** — Base semis, speed, wave, pulse width, numOscs, descending.
2. **Reset phases if numOscs changed**.
3. **Advance sweep** — `sweep += speed * 4 * dt` (mod 12).
4. **Compute sigma** — `max(1, (N-1) * 0.25)`.
5. **For each partial**:
   - Compute `semis = baseSemis + sweep + offsetOct * 12`.
   - Compute ideal frequency.
   - Compute Gaussian amplitude.
   - Compute frequency, clamp to Nyquist.
   - Advance phase.
   - Compute waveform sample.
   - Compute pan.
   - Accumulate into L/R.
6. **RMS normalize**.
7. **Soft limit** — tanh × 0.8 × 5.
8. **Display snapshot** — Save all display values.

## 7. Important Functions

### `ldSemisToHz(float semis)`

$$
f = 261.625565 \cdot 2^{\text{semis}/12}
$$

### `ldWaveSample(float ph, int wave, float pw)`

Waveform generator, shared by DSP and display.

### `FreqQuantity::getDisplayValueString()`

Returns note name or Hz depending on display mode.

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Shepard amplitude

$$
A_i = \exp\left(-\frac{(\log_2 f_i - \log_2 f_0)^2}{2\sigma^2}\right)
$$

### Sigma scaling

$$
\sigma = \max\left(1, \frac{N-1}{4}\right)
$$

### Sweep

$$
\text{sweep} = (\text{sweep} \pm \text{sweepRate} \cdot \Delta t) \bmod 12
$$

### Pan

$$
\text{pan}_i = \text{clamp}\left(\frac{\log_2(f_i/f_0)}{\sigma} \cdot 0.55, -1, 1\right)
$$

### RMS normalize

$$
\text{inv} = \frac{1.8}{\sqrt{\sum A_i^2}}
$$

### Soft limit

$$
\text{out} = 5\tanh(0.8 \cdot \text{out})
$$

## 9. Performance and Optimization

### CPU Usage

- Per partial: 1 sin (for phase) + 1 waveform call + 1 Gaussian + 1 pan ≈ 20 flops
- Max 20 partials × 20 flops = 400 flops/sample
- Display: 154-point scope + 20 partial stems ≈ 4000 flops/frame
- Total: ~500 flops/sample + 4 K flops/frame for the display

### Memory Allocation

- All state fixed-size
- No allocation in `process()`
- Display arrays: 20 × 5 floats = 100 floats

### Buffers

- `phases[20]`: 20 floats
- Display arrays: 20 × 4 = 80 floats

### Real-time Processing

- Loop bounded by 20 (numOscs)
- No allocation, no blocking
- Display scope uses display-thread only (no DSP impact)

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads

### Numerical Stability

- `clamp(fc, 10, sr*0.45)` on partial frequencies
- `tanh` soft limit on output
- `sumAmpSq > 0.001` check before normalizing

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstdlib>`, `<cstring>`, `<algorithm>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`, `rack::app::Knob`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`

## 11. Summary

`Ladder` is a Shepard–Risset glissando VCO. It's a very specialized module — it doesn't make a "normal" oscillator sound, it makes an auditory illusion. The Gaussian-weighted octave-spaced partials create a pitch that appears to climb (or descend) forever, and the stereo panning (low partials left, high partials right) adds to the spatial illusion.

The DSP is elegant: the entire pitch is controlled by a single sweep phase, and the partials are all derived from `baseSemis + sweep + offset * 12`. The sigma scaling with N ensures that all partials are audible regardless of how many you choose (which is not true of the original Shepard design, where more partials means the outer ones get quieter).

The main limitation is that it's a "one-trick pony" — it only does Shepard glissandos. But for that one trick, it's very well implemented, and the display (scope + partial spectrum) makes it easy to understand what's happening.
