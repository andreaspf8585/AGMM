# FancyADSR

## 1. Overview

`FancyADSR` is a 20 HP polyphonic envelope generator (up to 16 channels) with per-stage CV control, selectable linear/exponential attack and release shapes, an optional looping mode, a selectable ADSR/AD mode, a TEST gate button, and a live scope that shows both the ideal envelope curve implied by the current knobs (violet preview) and the actual envelope output history (cyan live trace for the selected channel, thinner coloured traces for the others).

Key features:
- Polyphonic, up to 16 independent voices; channel count follows the GATE/RETRIG inputs
- Per-stage CV on Attack, Decay, Sustain, Release (polyphonic CV modulates each channel separately; a mono CV applies to every channel)
- Selectable linear or exponential attack shape
- Selectable linear or exponential release shape
- Selectable ADSR / AD mode (AD treats the gate as a trigger)
- Loop mode (retrigger from gate while held)
- 5 ms end-of-release (EOR) pulse (end-of-decay in AD mode)
- Momentary TEST button that gates every channel while held
- Live scope: preview + per-channel live traces, plus stage/state chips, readouts and level meter
- Selectable scope time axis (50 ms .. 5 s)
- Right-click menu selects which channel the readouts/chips/meter follow

## Module Screenshot

![FancyADSR VCV Rack Module](images/FancyADSR.png)

## 2. Architecture

### Main Classes

- `FancyADSR`: Core module class inheriting from `rack::Module`
- `FAPanelBackground`: Static panel artwork
- `FADisplayWidget`: Live scope + readouts + chips
- `segCurve` helper: shared curve math for the preview (mirrors `runSegment`)

### Inheritance

- `FancyADSR` inherits from `rack::Module`
- Both UI widgets inherit from `rack::TransparentWidget`

### State (grouped)

| Group | Storage |
|-------|---------|
| Per-channel voice | `Voice voices[16]` — each holds `Stage stage`, `float env`, `float stageTime`, `attackStart`, `decayStart`, `releaseStart`, `float eorPulseRemaining`, `bool lastGate`, `bool lastTrigger` |
| Display channel | `int displayChannel` (saved with the patch) |
| History | `float history[16][400]`, `int historyCount`, `int historyWritePos`, `float sampleAccumulator` (all channels sampled together, one shared write head) |
| Display | `dispAttack, dispDecay, dispSustain, dispRelease`, `dispAttackShape, dispReleaseShape`, `dispEnv`, `dispStage`, `dispChannels`, `dispChannelShown`, `dispGate`, `dispAD`, `float eorFlash` |

Each `Voice` also caches the level an attack rises from (`attackStart`), the level an AD-mode decay falls from (`decayStart`), and the level a release falls from (`releaseStart`, captured when the release begins).

### Panel layout (20 HP = 300 × 380 px)

- Display (240 × 112) at (30, 32), inside a 4 px bezel, with rows for readouts, scope, legend and stage chips.
- Row A: four knobs (A/D/S/R) with their CV jacks beneath.
- Row B: five cells — MODE switch, ATK SHAPE switch, REL SHAPE switch, LOOP switch + light, TIME knob.
- Row C: GATE jack + TEST button, RETRIG jack, ENV OUT, EOR.

## 3. DSP Theory

### Envelope stages

The envelope has five stages:

1. **IDLE** — `env = 0`
2. **ATTACK** — ramp from `attackStart` to 1 over `attackTime`
3. **DECAY** — ramp from 1 (ADSR) or `decayStart` (AD) to the target over `decayTime` (always exponential)
4. **SUSTAIN** — hold at `sustainLev` (ADSR only)
5. **RELEASE** — ramp from `releaseStart` down to 0 over `releaseTime`

### Linear vs exponential segments

The `runSegment` function handles both shapes:

**Linear** (`shape = 0`):

$$
y = \text{start} + (\text{target} - \text{start})\frac{t}{D}
$$

**Exponential** (`shape = 1`):

$$
y = \text{start} + (\text{target} - \text{start})\left(1 - e^{-t/\tau}\right), \quad \tau = \frac{D}{-\ln(0.001)}
$$

The `-ln(0.001)` factor ensures the exponential segment reaches within 0.1% of its target after `D` seconds — so exponential and linear attacks of the same duration sound comparable in length. Decay is always exponential (standard ADSR behaviour).

### ADSR mode (default)

- **Gate rising edge**: if IDLE or RELEASE → start attack from 0; if SUSTAIN and looping → start attack from 0.
- **Gate falling edge**: if ATTACK/DECAY/SUSTAIN → start release from the current level.
- **Trigger rising edge**: if not IDLE → restart attack from 0.
- Attack rises from 0 regardless of the current level (original behaviour).
- Decay always falls to `sustainLev`.
- EOR pulses 5 ms when the release reaches zero.
- LOOP retriggers from SUSTAIN while the gate is held.

### AD mode

- **Gate rising edge** → start attack from the current level (retrigger mid-envelope does not snap to zero); the **falling edge is ignored**, so the envelope always plays through.
- **Trigger rising edge** → start attack from the current level from any stage.
- Envelope runs ATTACK → DECAY (down to zero) → IDLE. SUSTAIN and RELEASE knobs are ignored.
- EOR pulses 5 ms at the end of the decay.
- LOOP repeats the envelope while the gate is held; with LOOP off the envelope completes once.
- Switching to AD while sitting in SUSTAIN: the voice is moved to DECAY (starting from the current level) so it falls away to zero instead of hanging forever.

### Polyphony

The channel count is `max(1, max(gateChannels, triggerChannels))`, clamped to 16. `ENV_OUTPUT` and `EOR_OUTPUT` carry that many channels. Knobs, mode, shapes, loop and time controls are shared across all channels. A monophonic gate/retrigger is seen by every channel; a monophonic CV is applied to every channel. TEST gates every channel while held. Channels beyond the active count are reset each block so they start clean if the count grows again.

### End-of-release pulse

When the envelope reaches 0 (end of release in ADSR mode, end of decay in AD mode), a 5 ms pulse is fired on the EOR output and `eorFlash` is set for the display.

### History

Every active channel's envelope output is sampled into its own 400-entry ring buffer at a rate of `timeScale/400` per slot, where `timeScale` is the scope time axis (50 ms .. 5 s). All channels share one write position, so they stay in sync. This gives a scrolling scope that always shows the last `timeScale` seconds for every voice. Inactive channels record 0.

### Preview

The UI computes an "ideal" curve from the current knob values (100 sampled points) using `segCurve`, which mirrors `runSegment`. In ADSR mode the preview is attack → decay → hold → release; in AD mode it is attack → decay-to-zero → short tail of silence. It is drawn as a violet line so you can see the shape the envelope will have before you play it.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `ATTACK_PARAM` | 0.001 s | 10 s | 0.01 s | Attack time |
| `DECAY_PARAM` | 0.001 s | 10 s | 0.1 s | Decay time |
| `SUSTAIN_PARAM` | 0 | 1 | 0.8 | Sustain level (0–100%) |
| `RELEASE_PARAM` | 0.001 s | 10 s | 0.5 s | Release time |
| `TIME_SCALE_PARAM` | 0.05 s | 5 s | 1 s | Scope time axis (full width) |

### Switches

| Switch | Default | Description |
|--------|---------|-------------|
| `ATTACK_SHAPE_PARAM` | Exponential | Linear / Exponential |
| `RELEASE_SHAPE_PARAM` | Exponential | Linear / Exponential |
| `LOOP_TOGGLE_PARAM` | Off | Loop on/off |
| `MODE_PARAM` | ADSR | ADSR / AD (gate acts as trigger). Appended last so old patches stay compatible. |

### Button

| Button | Description |
|--------|-------------|
| `TEST_PARAM` | Momentary; acts as a gate on every channel while held |

### Inputs

| Input | Description |
|-------|-------------|
| `GATE_INPUT` | Gate (polyphonic) |
| `TRIGGER_INPUT` | Retrigger (polyphonic) |
| `ATTACK_CV_INPUT` | Attack CV (5 V adds 1 s, polyphonic) |
| `DECAY_CV_INPUT` | Decay CV (5 V adds 1 s, polyphonic) |
| `SUSTAIN_CV_INPUT` | Sustain CV (5 V adds 100%, polyphonic) |
| `RELEASE_CV_INPUT` | Release CV (5 V adds 1 s, polyphonic) |

### Outputs

| Output | Description |
|--------|-------------|
| `ENV_OUTPUT` | Envelope (0–10 V, polyphonic) |
| `EOR_OUTPUT` | End-of-release (end-of-decay in AD mode) 5 ms pulse (polyphonic) |

### Lights

- `LOOP_LIGHT` — On when loop mode is enabled

### Context Menu

- **Display channel** — selects which polyphony channel (1–16) the readouts, stage chips and level meter follow. The scope always plots every active channel. Saved in the patch as `displayChannel`.

### Enumerations

- `Stage`: IDLE, ATTACK, DECAY, SUSTAIN, RELEASE
- `ParamId`: Attack, Decay, Sustain, Release, AttackShape, ReleaseShape, Loop, TimeScale, Mode, Test
- `InputId`: Gate, Trigger, AttackCV, DecayCV, SustainCV, ReleaseCV
- `OutputId`: Env, EOR
- `LightId`: Loop

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `GATE_INPUT` | Gate | > 1 V | Envelope gate (polyphonic) |
| `TRIGGER_INPUT` | Gate | > 1 V | Retrigger (polyphonic) |
| `ATTACK_CV_INPUT` | CV | ±5 V | Attack offset, scaled to 1 s per 5 V (polyphonic) |
| `DECAY_CV_INPUT` | CV | ±5 V | Decay offset, scaled to 1 s per 5 V (polyphonic) |
| `SUSTAIN_CV_INPUT` | CV | ±5 V | Sustain offset, scaled to 100% per 5 V (polyphonic) |
| `RELEASE_CV_INPUT` | CV | ±5 V | Release offset, scaled to 1 s per 5 V (polyphonic) |

A monophonic CV or gate is applied to every channel.

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `ENV_OUTPUT` | CV | 0–10 V | Envelope (polyphonic) |
| `EOR_OUTPUT` | Gate | 0/10 V | End-of-release (end-of-decay in AD) 5 ms pulse (polyphonic) |

## 6. Signal Processing Pipeline

1. **Read shared state** — mode, loop, TEST, attack/release shapes.
2. **Determine channel count** — `max(1, max(gateChannels, retrigChannels))`, clamped to 16; set output channel counts.
3. **For each active channel `c`:**
   1. **Gate/trigger edge detect** — `gateHigh = (polyVoltage(c) > 1 V) || TEST`; `triggerHigh = polyVoltage(c) > 1 V`. Edge transitions call `onGate(voice, high, adMode, loop)` / `onTrigger(voice, high, adMode)`.
   2. **AD-mode SUSTAIN fixup** — if `adMode` and the voice is in SUSTAIN, move to DECAY from the current level.
   3. **Read parameters + CVs** — `getAttack/Decay/Sustain/Release(c)` read knob + per-channel CV and clamp.
   4. **Stage evaluation**:
      - **ATTACK**: `env = runSegment(attackStart, 1, attackTime, stageTime, attackShape)`; increment; if `>= attackTime`, go to DECAY with `decayStart = 1`.
      - **DECAY**: `env = runSegment(adMode ? decayStart : 1, adMode ? 0 : sustainLev, decayTime, stageTime, 1.0)`; increment; if `>= decayTime`:
        - AD mode: `env = 0`, fire EOR, then loop-restart if LOOP and gate held, else IDLE.
        - ADSR: `env = sustainLev`, go to SUSTAIN.
      - **SUSTAIN**: `env = sustainLev`; if loop and gate, restart attack.
      - **RELEASE**: `env = runSegment(releaseStart, 0, releaseTime, stageTime, releaseShape)`; increment; if `>= releaseTime`, `env = 0`, go to IDLE, fire EOR.
      - **IDLE**: `env = 0`.
   5. **Write env output channel `c`** — `env * 10 V`; store in `outV[c]` for history.
   6. **Write EOR channel `c`** — 10 V if pulse active, else 0.
   7. **Display data** — if `c == dispCh`, snapshot times, level, gate, stage, and set `eorFlash` on EOR.
4. **Reset channels beyond the active count.**
5. **Update loop LED**; update display globals (`dispAD`, `dispChannels`, `dispChannelShown`, shapes); decay `eorFlash`.
6. **History sampling** — every `timeScale/400` seconds, write all channels' `outV[c]` into their ring buffers at the shared write head and advance it.

## 7. Important Functions

### `runSegment(float start, float target, float duration, float t, float shape)`

Evaluates one envelope segment at time `t`.

```cpp
if (duration <= 0.0f) return target;
float alpha = clamp(t / duration, 0.0f, 1.0f);
if (shape <= 0.5f) { // linear
    return start + (target - start) * alpha;
} else { // exponential
    const float eps = 0.001f;
    float tau = duration / -std::log(eps);
    float factor = 1.0f - std::exp(-t / tau);
    return start + (target - start) * factor;
}
```

### `startAttack(Voice &v, bool fromCurrentLevel)`

Sets `stage = ATTACK`, resets `stageTime`, and sets `attackStart = fromCurrentLevel ? v.env : 0`.

### `onGate(Voice &v, bool high, bool adMode, bool loop)`

```cpp
if (adMode) {
    if (high) startAttack(v, true);
    return; // falling edge ignored in AD mode
}
if (high) {
    if (v.stage == STAGE_IDLE || v.stage == STAGE_RELEASE)
        startAttack(v, false);
    else if (v.stage == STAGE_SUSTAIN && loop)
        startAttack(v, false);
} else {
    if (v.stage == STAGE_ATTACK || v.stage == STAGE_DECAY || v.stage == STAGE_SUSTAIN) {
        v.stage = STAGE_RELEASE;
        v.stageTime = 0.0f;
        v.releaseStart = v.env; // captured once
    }
}
```

### `onTrigger(Voice &v, bool high, bool adMode)`

AD mode: a rising edge starts an attack from the current level from any stage. ADSR mode: a rising edge while not IDLE restarts the attack from 0.

### `getAttack(int c)`, `getDecay(int c)`, `getSustain(int c)`, `getRelease(int c)`

Read knob + per-channel CV (mono CV applied to all channels), clamp to valid ranges.

### `getHistorySize()` / `getHistorySample(int ch, int index)`

Return the number of stored samples (0..400) and channel `ch`'s sample at `index` (0 = oldest, `historyCount-1` = newest). Out-of-range indices/channels return 0.

### `process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Linear segment

$$
y = \text{start} + (\text{target} - \text{start})\frac{t}{D}
$$

### Exponential segment

$$
\tau = \frac{D}{-\ln(0.001)}, \quad y = \text{start} + (\text{target} - \text{start})(1 - e^{-t/\tau})
$$

### Stage time

$$
\text{stageTime} \leftarrow \text{stageTime} + \Delta t
$$

Transition when `stageTime >= duration`.

### History sampling

$$
\text{interval} = \frac{\text{timeScale}}{400}
$$

Every `interval` seconds, write each channel's `outV[c]` to `history[c][historyWritePos]`, advance the shared write head, and saturate `historyCount` at 400.

### CV scaling (per channel `c`)

$$
\text{attackTime} = \text{clamp}(\text{knob} + \text{CV}_c/5, 0.001, 10)
$$

Same for decay and release.

$$
\text{sustainLev} = \text{clamp}(\text{knob} + \text{CV}_c/5, 0, 1)
$$

## 9. Performance and Optimization

### CPU Usage

- Per active channel per sample: 1 stage evaluation + 1 segment call (exp or linear) + 1 EOR write ≈ 30 flops
- Display: up to 16 live traces (400 points each) + 100-point preview + 6 chips
- History sampling runs only every `timeScale/400` seconds (a `while` loop that may write several slots per sample at fast time bases)

### Memory Allocation

- All state is fixed-size
- History buffer: 16 × 400 floats = 25.6 KB
- No allocation in `process()`

### Buffers

- `history[16][400]`: 25.6 KB

### Real-time Processing

- All operations are O(1) per channel per sample (O(channels) per sample overall)
- No allocation, no blocking
- The exponential segment uses 1 `exp` call per active channel during attack/decay/release

### Thread Safety

- All state is written by the audio thread and read by the UI thread via torn reads (cosmetic only)
- Each channel's 400-entry history buffer is written sequentially by the audio thread and read as a snapshot by the UI

### Numerical Stability

- All times clamped to 0.001 .. 10 s
- Sustain clamped to 0..1
- Envelope output in 0..10 V
- Out-of-range history reads return 0

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<cmath>`, `<cstdio>`, `<cstring>`, `<algorithm>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::engine::ParamQuantity`
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::plate`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::meter`, `agmm::chip`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`
- UI widgets: `RoundSmallBlackKnob`, `DarkPJ301MPort`, `CKSS`, `TL1105`, `ScrewBlack`, `SmallLight<agmm::CyanLight>`

## 11. Summary

`FancyADSR` is a classic ADSR envelope generator with a polyphonic (up to 16-voice) engine and a few nice extras: per-stage CV, selectable attack/release shapes (linear or exponential), loop mode, a TEST gate button, an EOR pulse (end-of-release, or end-of-decay in AD mode), and a live scope that shows both the ideal curve and the actual output of every channel.

The linear/exponential option is significant: exponential attacks sound "punchy" while linear attacks sound "smooth", and being able to switch them per-stage (not globally) is a feature usually only found on high-end hardware envelopes. The `τ = D / -ln(0.001)` formula ensures that exponential segments reach their target within 0.1% after exactly `D` seconds — so exponential and linear segments of the same duration feel comparable in length. Decay is always exponential, matching conventional ADSR behaviour.

The AD mode turns the module into a one-shot attack/decay envelope driven purely by the gate/trigger edges (the gate acts as a trigger, so the envelope always plays through). The polyphonic engine follows the gate/retrigger channel counts, and the right-click "Display channel" menu selects which voice the readouts, chips and level meter follow while the scope still overlays all active voices.

Limitations: the EOR pulse is fixed at 5 ms, AD mode ignores the Sustain and Release controls, and the knobs/mode/shape/loop/time controls are shared by all channels (only the CVs and display selection are per channel). For a simpler envelope, see any basic ADSR; for a more feature-rich envelope, see the `FancyADSR`-inspired `Pharmakon` (which adds a full multi-FX chain) or `InstantDrone` (which adds an envelope per oscillator).