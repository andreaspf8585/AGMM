<!--
AGMM AUTOMATIC DOCUMENTATION
Source: ExrpessArpeggiator.cpp
-->

# ExpressArpeggiator

## 1. Overview

`ExpressArpeggiator` is a 28 HP generative arpeggiator/sequencer. It holds a 16-step pattern of notes (all within a chosen root, scale and octave span), and advances through the active steps on each clock edge according to a selected direction (forward, backward, random, drunk, pendulum). Steps can be muted (rests), locked (protected from randomize/mutation), and the pattern can be randomized or probabilistically mutated while it plays.

Key features:
- 16-step pattern with per-step notes, mutes, locks
- 60+ scales
- Root note, octave span (1–4), transpose
- 5 direction modes: forward, backward, random, drunk, pendulum
- Per-step mutation probability
- Randomize button + external random trigger
- Interactive display: drag to paint notes, click to mute/lock, click a step number to set the loop length, click a direction chip to change direction
- Per-step end-of-cycle gate
- Step-position CV output
- Glide time

## Module Screenshot

![ExpressArpeggiator VCV Rack Module](images/expressarpeggiator.png)

## 2. Architecture

### Main Classes

- `ExpressArpeggiator`: Core module class inheriting from `rack::Module`
- `ScaleDef`: Struct for a named scale
- `EARootQ`, `EAScaleQ`: Custom ParamQuantity for key/scale knobs
- `EAPanel`: Static panel artwork
- `EADisplay`: Interactive pattern display

### Inheritance

- `ExpressArpeggiator` inherits from `rack::Module`
- `EAPanel` and `EADisplay` inherit from `rack::TransparentWidget`
- `EADisplay` inherits from `rack::Widget` (interactive)

### Structures

- `ScaleDef { std::string name; std::vector<int> intervals; }`
- `EAKnob { int param; float x, top, w; const char *label; }`

### State (grouped)

| Group | Storage |
|-------|---------|
| Pattern | `int noteSemi[16]`, `bool locked[16]`, `bool muted[16]` |
| Playhead | `int currentStep`, `int dirSign`, `bool playing`, `bool curRest` |
| Transport | `uint32_t stepCounter` |
| Scale | `uint16_t scaleMask` (absolute pitch classes) |
| Audio out | `float targetV, outV`, `bool outInit`, `float gateRemaining` |
| Clock timing | `float sinceClock, clockPeriod`, `bool clockSeen` |
| Display | `float stepGlow[16]`, `float eocFlash, restFlash, mutFlash, randFlash`, `int mutIdx, playPc, vTranspose` |

## 3. DSP Theory

### Scale and note model

Notes are stored as semitones relative to C4 (0). The pattern is a 16-entry array of `noteSemi`. Each note is quantized to the current scale and folded into the current octave span `[root, root + 12*span - 1]`.

### Quantization

For each note, `fixNote(s)` finds the closest scale note (up to ±6 semitones search), then octave-folds into the root..root+span range.

### Direction modes

- **Forward**: `currentStep++`, wrap to 0 at `numSteps`
- **Backward**: `currentStep--`, wrap to `numSteps−1`
- **Random**: `currentStep = randInt(numSteps)`
- **Drunk**: `currentStep += randInt(3) - 1`, wrapping to `[0, numSteps)`
- **Pendulum**: bounce between 0 and `numSteps−1` using `dirSign`

### Playhead initialisation

On the first clock after reset/stop: `playing = true`, `currentStep = (dir == backward) ? numSteps-1 : 0`.

### Rest and gate logic

On each step:

$$
\text{rest} = \text{muted}[c] \lor (u < \text{restProb})
$$

If not rest:
- `playPc = noteSemi[c] mod 12`
- `gateRemaining = gateLen * clockPeriod` (or infinite if gateLen ≥ 0.995)
- `targetV = noteSemi[c] / 12`

If rest:
- `gateRemaining = 0`
- `restFlash = 1`

### End of cycle

`eoc = true` is returned when the playhead wraps around (forward: `currentStep >= n → 0`; backward: `currentStep < 0 → n-1`; random/drunk: every `n` steps; pendulum: when bouncing back from the top).

### Glide

The output voltage glides toward `targetV` with an exponential smoothing:

$$
\tau = 0.002 \cdot 250^{\text{glide}} \text{ s}, \quad a = 1 - e^{-\Delta t / \tau}
$$

$$
\text{outV} \leftarrow \text{outV} + a(\text{targetV} + \text{transpose}/12 - \text{outV})
$$

### Mutation

On each clock (after the new step is set), with probability `mutP`, one random unlocked step is replaced with a fresh scale note.

### Randomize

Every unlocked step is replaced with a fresh scale note.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `ROOT_NOTE_PARAM` | 0 | 11 | 0 | Root note (C–B) |
| `SCALE_PARAM` | 0 | N−1 | 0 | Scale |
| `STEPS_PARAM` | 2 | 16 | 4 | Number of steps |
| `DIRECTION_PARAM` | 0 | 4 | 0 | Direction |
| `MUTATION_PARAM` | 0 | 1 | 0 | Mutation probability |
| `OCTAVES_PARAM` | 1 | 4 | 1 | Octave span |
| `GATE_LEN_PARAM` | 0.05 | 1 | 0.5 | Gate length (fraction of clock period) |
| `REST_PARAM` | 0 | 1 | 0 | Rest probability |
| `GLIDE_PARAM` | 0 | 1 | 0 | Glide |
| `TRANSPOSE_PARAM` | −12 | 12 | 0 | Transpose (semitones) |

### Buttons

| Button | Description |
|--------|-------------|
| `RANDOMIZE_BUTTON` | Randomize pattern |

### Inputs

| Input | Description |
|-------|-------------|
| `CLOCK_INPUT` | Clock |
| `RANDOM_TRIGGER_INPUT` | Random trigger |
| `RESET_INPUT` | Reset (next clock plays step 1) |
| `TRANSPOSE_CV_INPUT` | Transpose CV (1 V/oct) |
| `MUT_CV_INPUT` | Mutation CV (0–10 V) |

### Outputs

| Output | Description |
|--------|-------------|
| `VOCT_OUTPUT` | V/Oct |
| `CYCLE_GATE_OUTPUT` | End of cycle |
| `GATE_OUTPUT` | Gate |
| `STEP_OUTPUT` | Step position (0–10 V) |

### Lights

- `RANDOM_LIGHT` — flashes on randomize

### Enumerations

- `ParamId`: RootNote, Scale, Steps, Direction, Mutation, RandomizeButton, Octaves, GateLen, Rest, Glide, Transpose
- `InputId`: Clock, RandomTrigger, Reset, TransposeCV, MutCV
- `OutputId`: VOct, CycleGate, Gate, Step
- `LightId`: RandomLight

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `CLOCK_INPUT` | Gate | > 1 V | Clock edge advances the playhead |
| `RANDOM_TRIGGER_INPUT` | Gate | > 1 V | Re-randomizes the pattern |
| `RESET_INPUT` | Gate | > 1 V | Next clock plays step 1 |
| `TRANSPOSE_CV_INPUT` | CV | 1 V/oct | Transpose CV |
| `MUT_CV_INPUT` | CV | ±10 V | Mutation probability offset |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `VOCT_OUTPUT` | CV | 1 V/oct | Current note pitch (glided) |
| `CYCLE_GATE_OUTPUT` | Gate | 0/10 V | 1 ms pulse at end of cycle |
| `GATE_OUTPUT` | Gate | 0/10 V | Note gate |
| `STEP_OUTPUT` | CV | 0–10 V | Step position (0 = first, 10 = last) |

## 6. Signal Processing Pipeline

1. Detect key/scale/span change — re-quantize all notes.
2. Read numSteps; clamp currentStep.
3. Read rest prob, gate len, glide, transpose, mut prob.
4. Handle randomize button / trigger — re-randomize all unlocked steps.
5. Handle reset — `playing = false`, `stepCounter = 0`, `dirSign = 1`, gate off, rest off.
6. **Clock edge**:
   - Update `clockPeriod` from `sinceClock`.
   - Call `doStep(restP, gateLen, mutP)`.
7. Update target pitch.
8. Apply glide to output voltage.
9. Decrement gate timer.
10. Write outputs (V/Oct, gate, cycle gate, step position).
11. Decay display flashes; update random LED.

## 7. Important Functions

### `doStep(float restP, float gateLen, float mutP)`

Advances the playhead, decides whether the step is a rest, sets the gate/pitch state, fires EOC, and applies mutation.

```cpp
if (!playing) {
    playing = true;
    dirSign = 1;
    currentStep = (dir == 1) ? numSteps - 1 : 0;
} else {
    eoc = advance(dir);
}
currentStep = clamp(currentStep, 0, numSteps - 1);
bool rest = muted[currentStep] || (restP > 0.f && random::uniform() < restP);
curRest = rest;
stepGlow[currentStep] = 1.f;
if (!rest) {
    playPc = eaMod12(noteSemi[currentStep]);
    gateRemaining = (gateLen >= 0.995f) ? 1e9f : std::max(0.001f, clockPeriod * gateLen);
} else {
    gateRemaining = 0.f;
    restFlash = 1.f;
}
if (eoc) { eocPulse.trigger(1e-3f); eocFlash = 1.f; }
maybeMutate(mutP);
```

### `advance(int dir)`

Advances the playhead according to the direction mode, returns true if a cycle ended.

### `fixNote(int s)`

Quantizes `s` to the nearest scale note, folded into `[root, root + span*12 - 1]`.

```cpp
int best = s;
for (int d = 0; d <= 6; ++d) {
    if (inScale(s - d, root, iv)) { best = s - d; break; }
    if (inScale(s + d, root, iv)) { best = s + d; break; }
}
while (best < lo) best += 12;
while (best > hi) best -= 12;
return best;
```

### `randomizePattern()`

Replaces every unlocked step with a fresh scale note.

### `maybeMutate(float p)`

With probability `p`, replaces one random unlocked step with a fresh scale note.

### `process(const ProcessArgs &args)`

Main callback described in §6.

## 8. Mathematical Implementation

### Scale membership

$$
\text{inScale}(s) = \left((s - \text{root}) \bmod 12\right) \in \text{intervals}
$$

### Quantization

$$
\text{best} = \arg\min_{d \in [0, 6]} \left|\left((s \pm d - \text{root}) \bmod 12\right) \in \text{intervals}\right|
$$

### Glide

$$
\tau = 0.002 \cdot 250^{\text{glide}}, \quad a = 1 - e^{-\Delta t / \tau}
$$

### Gate time

$$
\text{gateRemaining} = \text{clockPeriod} \cdot \text{gateLen}
$$

### Step position CV

$$
\text{out} = 10 \cdot \frac{\text{currentStep}}{\max(\text{numSteps} - 1, 1)}
$$

## 9. Performance and Optimization

### CPU Usage

- Per clock edge: 16 steps × (fixNote + maybe mutate) ≈ 16 × 30 flops
- Per sample: 1 glide update + 4 output writes ≈ 20 flops
- Display: 16 glow decays + 1 random flash decay

### Memory Allocation

- All state is fixed-size
- No allocation in `process()`
- Scale table is static

### Buffers

- `noteSemi[16]`, `locked[16]`, `muted[16]`: 48 small values
- `stepGlow[16]`: 16 floats

### Real-time Processing

- All loops bounded by 16
- No allocation, no blocking
- `random::uniform()` used for mutation and randomization

### Thread Safety

- All state written by audio thread, read by UI thread via torn reads
- The UI thread can modify `noteSemi`, `locked`, `muted` directly (via the interactive display), which is technically a data race but harmless because the display only writes when the user is actively dragging

### Numerical Stability

- All values clamped to safe ranges
- Quantization is deterministic
- No floating-point accumulation

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<vector>`, `<string>`, `<cmath>`, `<cstdio>`, `<cstring>`, `<cstdint>`, `<algorithm>`, `<memory>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`, `rack::Widget`
- `rack::dsp::SchmittTrigger`, `rack::dsp::PulseGenerator`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::plate`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::chip`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::dim`, `agmm::bright`, `agmm::warm`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`

## 11. Summary

`ExpressArpeggiator` is a generative arpeggiator/sequencer with an unusually interactive display: you can drag to paint notes, click to mute or lock steps, click a step number to set the loop length, and click a direction chip to change direction. It's one of the few AGMM modules with a "true" interactive display — most of the others are read-only.

The pattern model is simple: 16 notes, each with a lock and a mute flag. Notes are stored as semitones relative to C4 and quantized to the current scale. The playhead advances according to one of five direction modes (forward, backward, random, drunk, pendulum), and each step is either a note, a rest (muted or by chance), or a mutation (with probability).

The main strengths are: the interactive display (unusual and fun), the range of direction modes, and the mutation/randomize features that keep the pattern evolving. The main limitations are: no CV inputs for most parameters (only transpose and mutation have CV), and the pattern is 16 steps (fine for arpeggios, limiting for longer sequences).
