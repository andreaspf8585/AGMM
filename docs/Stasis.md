<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Stasis.cpp
-->

# Stasis

## 1. Overview

`Stasis` is a 24 HP 8-stage clocked sample-and-hold "shift register" with probability and slew. On each clock edge every stage samples a new value:

- Stage 1 takes its input from a patched jack, or (if unpatched) either an internal white-noise source or, with LOOP on, the last active stage's previous value (a feedback loop).
- Stages 2..8 take the previous stage's previous value (a shift register).
- A patched input at stage i breaks the chain at that point.
- With PROB below 100%, each stage independently decides whether to sample or hold (and flashes a skipped outline).

Key features:
- 8-stage shift register with per-stage input (breaking the chain)
- Per-stage probability (sample or hold)
- Per-stage slew (1 ms .. 2 s)
- Loop mode (last active stage feeds stage 1)
- Bipolar level
- Polyphonic 8-channel output
- Live display: bars, chain arrows, loop bus, 24-column history heat map
- 24 HP panel with side knobs, control row, channels rows

## Module Screenshot

![Stasis VCV Rack Module](images/Stasis.png)

## 2. Architecture

### Main Classes

- `Stasis`: Core module class inheriting from `rack::Module`
- `STPanelBackground`: Static panel artwork
- `STKnobArcs`: Live value arcs
- `STDisplayWidget`: Live display (bars + chain + heat map)

### Inheritance

- `Stasis` inherits from `rack::Module`
- All UI widgets inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Register | `float held[8]`, `float outv[8]` |
| Display | `float flash[8]`, `float skip[8]`, `bool linked[8]`, `float clockFlash`, `float hist[8][24]`, `bool histUpd[8][24]`, `int histHead`, `int histCount`, `uint32_t stepCount` |
| Settings | `int vLen`, `float vProb`, `float vTau`, `bool loopOn` |
| Triggers | `dsp::SchmittTrigger clockTrig, stepTrig, resetTrig, loopTrig` |

## 3. DSP Theory

### Shift register

The module has 8 sample-and-hold stages, each with its own input jack and output jack. On each clock:

- Stage 0: samples from `CH_INPUT[0]` if patched; otherwise from noise or (if LOOP) from the last active stage's old value.
- Stage i (i > 0): samples from `CH_INPUT[i]` if patched; otherwise from stage i−1's old value.

This makes it a shift register, where the chain shifts right by one stage per clock.

### Probability

Each stage independently decides whether to sample or hold:

$$
\text{visited} = (u < \text{prob})
$$

If visited, the stage updates its held value and flashes. Otherwise, it holds (and flashes a skip outline).

### Slew

The output of each stage is a one-pole smoothing of the held value:

$$
\text{out} \leftarrow \text{out} + (\text{held} - \text{out}) \cdot a
$$

where $a = 1 - e^{-\Delta t/\tau}$ and $\tau = 1 \text{ ms} \cdot 2000^{\text{knob}}$ (1 ms .. 2 s).

### Loop

When LOOP is on, stage 0 samples from the last active stage (`len−1`) instead of noise. This creates a feedback loop where the value travels through all stages, then wraps back to stage 0.

### Chain detection

The `linked[i]` boolean records whether stage i is normalled (its input jack is unpatched). The display uses this to draw chain arrows (cyan when linked, orange when broken).

### History heat map

A 24-column ring buffer stores the held values over time. Each column is one clock, each row is one stage. The colour shows polarity (cyan positive, violet negative), and the alpha shows magnitude. The white left edge marks the samples that landed (visited).

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `SLEW_PARAM` | 0 | 1 | 0 | Slew (1 ms .. 2 s) |
| `PROB_PARAM` | 0 | 1 | 1 | Sample probability |
| `LENGTH_PARAM` | 1 | 8 | 8 | Loop length |
| `LEVEL_PARAM` | −1 | 1 | 1 | Output level |

### Buttons

| Button | Description |
|--------|-------------|
| `LOOP_PARAM` | Loop (last stage feeds stage 1) |
| `STEP_PARAM` | Manual clock step |

### Inputs

| Input | Description |
|-------|-------------|
| `CLOCK_INPUT` | Clock |
| `RESET_INPUT` | Reset |
| `CH_INPUT + 0..7` | 8 stage inputs |

### Outputs

| Output | Description |
|--------|-------------|
| `CH_OUTPUT + 0..7` | 8 stage outputs |
| `POLY_OUTPUT` | Polyphonic 8-channel |

### Lights

- `LOOP_LIGHT` — Loop active
- `STEP_LIGHT` — Clock flash

### Enumerations

- `ParamId`: Slew, Prob, Length, Level, Loop, Step
- `InputId`: Clock, Reset, CH_INPUT (8)
- `OutputId`: CH_OUTPUT (8), Poly
- `LightId`: Loop, Step

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CLOCK_INPUT` | Gate | > 1 V | Clock |
| `RESET_INPUT` | Gate | > 1 V | Reset |
| `CH_INPUT + 0..7` | CV | ±10 V | 8 stage inputs |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CH_OUTPUT + 0..7` | CV | ±10 V | 8 stage outputs |
| `POLY_OUTPUT` | Poly CV | ±10 V | Polyphonic 8-channel |

## 6. Signal Processing Pipeline

1. **Read params** — Slew, probability, length, level.
2. **Loop button** — Toggle loopOn.
3. **Reset button** — Clear register.
4. **Clock/step edge**:
   - Save all held values (`old[]`).
   - For each stage:
     - Read input (or noise/loop if unpatched).
     - Roll probability: sample or hold.
     - Record to history.
5. **Slew** — Update `outv` toward `held`.
6. **Outputs** — Write stage outputs and poly output (× level).
7. **Display** — Flash/skip decay, clock flash decay.
8. **LEDs** — Loop and step.

## 7. Important Functions

### `doClock(int len, float prob)`

Performs one clock step over the first `len` stages.

```cpp
float old[8];
for (int i = 0; i < 8; ++i) old[i] = held[i];

for (int i = 0; i < 8; ++i) {
    bool conn = inputs[CH_INPUT + i].isConnected();
    float src;
    if (conn) src = inputs[CH_INPUT + i].getVoltage();
    else if (i == 0) src = loopOn ? old[len - 1] : (random::uniform() * 10.f - 5.f);
    else src = old[i - 1];
    linked[i] = !conn;

    bool take = (prob >= 0.999f) || (random::uniform() < prob);
    if (take) {
        held[i] = src;
        flash[i] = 1.f;
    } else {
        skip[i] = 1.f;
    }
    hist[i][histHead] = held[i];
    histUpd[i][histHead] = take;
}
histHead = (histHead + 1) % 24;
if (histCount < 24) histCount++;
stepCount++;
clockFlash = 1.f;
```

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Slew

$$
\tau = 0.001 \cdot 2000^{\text{knob}} \text{ s}
$$

$$
a = 1 - e^{-\Delta t/\tau}, \quad \text{out} \leftarrow \text{out} + a(\text{held} - \text{out})
$$

### Probability

$$
\text{take} = (u < \text{prob})
$$

### Loop source

$$
\text{src}_0 = \begin{cases} \text{noise} & \text{loop off} \\ \text{old}[\text{len} - 1] & \text{loop on} \end{cases}
$$

## 9. Performance and Optimization

### CPU Usage

- Per clock edge: 8 stages × ~10 flops = 80 flops
- Per sample: 8 slew updates + 8 outputs = ~50 flops
- Display: 8 bars + 8 chain arrows + 24×8 heat map ≈ 500 flops/frame
- Total: ~50 flops/sample

### Memory Allocation

- All state fixed-size
- `hist[8][24]` = 192 floats ≈ 768 bytes
- `held[8]`, `outv[8]` = 64 bytes

### Buffers

- `hist[8][24]`: 768 bytes

### Real-time Processing

- All loops bounded
- No allocation, no blocking
- `random::uniform()` used for probability and noise

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads

### Numerical Stability

- All values clamped to ±10 V by Rack output
- Slew prevents clicks
- No feedback loops (except in loop mode, which is bounded by probability)

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstdint>`, `<cstring>`, `<algorithm>`, `<memory>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `rack::dsp::SchmittTrigger`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::plate`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::chip`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`

## 11. Summary

`Stasis` is an 8-stage clocked shift register with per-stage probability, slew, and loop. It's the kind of module you patch when you want a "sequence" that evolves: each clock, the values shift through the register, but with probability they might hold, and with loop they might feed back to stage 1.

The **per-stage input** is the key feature: by patching a different signal into stage 4, you break the chain at that point and inject a new value. This lets you construct complex "modulation" patterns from simple components.

The display is excellent: the bars show the current values, the chain arrows show whether the register is linked or broken, the loop bus shows the feedback path, and the heat map shows 24 clocks of history. You can understand exactly what's happening at a glance.

The main limitations are: 8 stages is a small register, and there are no CV inputs for the controls (everything is manual). But for a "hands-on" shift register, this is excellent.