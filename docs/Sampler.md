<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Sampler.cpp
-->

# Sampler

## 1. Overview

`Sampler` is a 36 HP mono/stereo sample recorder + player with a rich set of playback features. It can record from the L/R inputs (either a brand-new sample or, in Sound-on-Sound mode, overdub onto the existing one), load and save WAV files, and play back with adjustable speed / V-Oct tuning, start position, loop length, forward/reverse direction, one-shot / loop / ping-pong play modes, anti-click fades, a simple A-D envelope (with optional sustain gate), a loop crossfade and a configurable input/playback mix.

Key features:
- Record from L/R inputs (new or overdub)
- Load/save WAV files (16-bit PCM, 8/16/24/32-bit, IEEE float)
- Adjustable playback speed / V/Oct tuning
- Start position, length, direction, play mode (one-shot/loop/ping-pong)
- Anti-click fades (declick)
- A-D envelope (attack/decay/sustain) with optional gate
- Loop crossfade
- Input/playback mix
- EOC trigger, play gate, record gate, envelope CV
- Interactive waveform display (drag to set loop region)
- Context menu: load, save, normalize, reverse, trim to loop, clear, auto-play
- Sound-on-sound (overdub) mode

## Module Screenshot

![Sampler VCV Rack Module](images/Sampler.png)

## 2. Architecture

### Main Classes

- `Sampler`: Core module class inheriting from `rack::Module`
- `SamplerEnvelope`: Simple A-D envelope with optional sustain
- `spLoadWav`, `spSaveWav`: Self-contained WAV I/O
- `SPPanelBackground`: Static panel artwork
- `SPDisplayWidget`: Interactive waveform display
- `SPReadout`: Per-parameter readout
- `SPInfoWidget`: Sample info strip

### Inheritance

- `Sampler` inherits from `rack::Module`
- All UI widgets inherit from `rack::TransparentWidget` or `rack::Widget`

### Structures

- `SamplerEnvelope::Stage`: IDLE, ATTACK, DECAY

### State (grouped)

| Group | Storage |
|-------|---------|
| Buffers (shared) | `std::mutex bufferMutex`, `std::vector<float> leftSamples, rightSamples, tempLeft, tempRight` |
| Atomics | `std::atomic<bool> isRecording, isPlaying, sosOverdub` |
| Atomics | `std::atomic<float> playhead, loopStartNorm, loopLengthNorm, bufferRate` |
| Atomics | `std::atomic<size_t> mainBufferSize, recBufferSize` |
| Peaks | `std::vector<float> peakMin, peakMax, recPeakMin, recPeakMax`, `float recBucketMin, recBucketMax`, `int recBucketCount` |
| Sample rate | `float sampleRate` |
| Transport | `bool lastRecordTrigger, lastPlayTrigger, lastClear, gateActive, pingPongForward, autoPlay` |
| Overdub | `long sosLastIdx` |
| Fades | `float fadeIn`, `float mixSm` |
| Envelope | `SamplerEnvelope env` |
| EOC | `dsp::PulseGenerator eocPulse` |
| Path | `std::string lastPath` |
| Display | `float meterInL, meterInR, meterOutL, meterOutR`, `float displaySpeed` |

## 3. DSP Theory

### WAV loading/saving

The module has its own self-contained WAV I/O (`spLoadWav`, `spSaveWav`). It supports PCM 8/16/24/32-bit and IEEE 32-bit float, with `WAVE_FORMAT_EXTENSIBLE` detection. Mono files are duplicated to stereo. Files are scaled by `SP_WAV_FULLSCALE = 5 V` (so a peak of 1.0 in the WAV = 5 V in Rack).

### Playback

The playhead advances by `speed` each sample:

$$
\text{playhead} \leftarrow \text{playhead} + \text{speed} \cdot \text{rateRatio}
$$

where `rateRatio = bufferRate / sampleRate`. This lets the sample play at its native rate regardless of the Rack sample rate.

The output is read with linear interpolation:

$$
\text{out} = \text{buffer}[\text{idx}](1 - \text{frac}) + \text{buffer}[\text{idx}+1] \cdot \text{frac}
$$

### Play modes

- **One-shot**: play once, then stop
- **Loop**: wrap from end to start
- **Ping-pong**: bounce between start and end (forwards, then backwards, then forwards, etc.)

### Direction

- **Forward**: `speed = +totalSpeed`
- **Reverse**: `speed = -totalSpeed`

In ping-pong mode, `speed` alternates between `+|totalSpeed|` and `-|totalSpeed|`.

### Loop crossfade

When the playhead is near the loop boundary, crossfade between the two ends:

$$
\text{out} = \text{current} \cdot \text{factor} + \text{otherEnd} \cdot (1 - \text{factor})
$$

This prevents clicks at the loop seam.

### Anti-click fades

The start fade-in is quadratic over `declick` ms:

$$
\text{gain} = f^2(3 - 2f)
$$

where $f = \text{fadeIn}$. The end fade-out (for one-shot) fades over the last `declick` ms.

### Envelope

Simple A-D envelope with optional sustain gate:

- **IDLE**: output 0
- **ATTACK**: linear ramp 0 → 1 over `attackTime`
- **DECAY**: exponential decay toward 0 over `decayTime`, or hold at 1 if `sustained` (gate held) or decay time is ~infinite

### Sound-on-sound (overdub)

When recording in SOS mode:

$$
\text{buffer}[i] \leftarrow \text{buffer}[i] \cdot fb + \text{input}, \quad i \in [\text{lastIdx}+1, \text{idx}]
$$

where `fb` is the SOS feedback (old material level). This overwrites the existing sample at every buffer index the playhead crossed.

### Playback mix

Input/playback crossfade (equal-power):

$$
\text{out} = \text{playback} \cdot \cos(m \cdot \pi/2) + \text{input} \cdot \sin(m \cdot \pi/2)
$$

with `m` smoothed for click-free transitions.

## 4. Parameters and Controls

### Knobs

| Parameter | Min | Max | Default | Description |
|-----------|-----|-----|---------|-------------|
| `SPEED_PARAM` | −3 | 3 | 1 | Speed / tuning (×) |
| `SPEED_CV_ATTEN` | −1 | 1 | 1 | Speed CV amount |
| `START_PARAM` | 0 | 1 | 0 | Start position |
| `START_CV_ATTEN` | −1 | 1 | 1 | Start CV amount |
| `LENGTH_PARAM` | 0 | 1 | 1 | Playback length |
| `LENGTH_CV_ATTEN` | −1 | 1 | 1 | Length CV amount |
| `ATTACK_PARAM` | 0 | 5 | 0 | Envelope attack (s) |
| `DECAY_PARAM` | 0 | 10 | 10 | Envelope decay (s) |
| `LEVEL_PARAM` | 0 | 2 | 1 | Output level |
| `XFADE_PARAM` | 1 | 100 | 15 | Loop crossfade (ms) |
| `MIX_PARAM` | 0 | 1 | 0 | Input/playback mix |
| `DECLICK_PARAM` | 0 | 100 | 3 | Anti-click fade (ms) |
| `SOS_FDBK_PARAM` | 0 | 1 | 1 | SOS feedback |

### Switches

| Parameter | Description |
|-----------|-------------|
| `PITCH_MODE_PARAM` | Speed / V/Oct |
| `PLAY_MODE_PARAM` | One-shot / Loop / Ping-pong |
| `DIRECTION_PARAM` | Forward / Reverse |
| `LINE_SWITCH` | Line monitor |
| `SOS_SWITCH` | Sound on sound |

### Buttons

| Parameter | Description |
|-----------|-------------|
| `REC_BUTTON` | Record |
| `PLAY_BUTTON` | Play |
| `CLEAR_BUTTON` | Clear sample |

### Inputs

| Input | Description |
|-------|-------------|
| `LEFT_INPUT` | L in |
| `RIGHT_INPUT` | R in |
| `RECORD_TRIGGER` | Record trigger |
| `PLAY_TRIGGER` | Play trigger |
| `SPEED_CV_INPUT` | Speed CV |
| `START_CV_INPUT` | Start CV |
| `LENGTH_CV_INPUT` | Length CV |
| `PITCH_CV_INPUT` | Pitch CV (1 V/oct) |
| `GATE_INPUT` | Envelope sustain gate |

### Outputs

| Output | Description |
|--------|-------------|
| `LEFT_OUTPUT` | L out |
| `RIGHT_OUTPUT` | R out |
| `EOC_OUTPUT` | End of cycle |
| `PLAY_GATE_OUTPUT` | Playing gate |
| `REC_GATE_OUTPUT` | Recording gate |
| `ENV_OUTPUT` | Envelope CV |

### Lights

- `REC_LIGHT` — Recording
- `PLAY_LIGHT` — Playing
- `PITCH_MODE_LIGHT` — V/Oct mode
- `LOOP_MODE_LIGHT` — Loop mode
- `SOS_LIGHT` — SOS mode
- `LINE_LIGHT` — Line monitor

### Menu

- Load WAV
- Save WAV
- Normalize
- Reverse sample
- Trim to loop region
- Clear sample
- Auto-play after recording

### Enumerations

- `ParamId`: Speed, SpeedCVAtten, Start, StartCVAtten, Length, LengthCVAtten, Attack, Decay, PitchMode, PlayMode, Direction, RecButton, PlayButton, Level, Xfade, LineSwitch, ClearButton, Mix, SOSSwitch, Declick, SOSFdbk
- `InputId`: Left, Right, RecordTrigger, PlayTrigger, SpeedCV, StartCV, LengthCV, PitchCV, Gate
- `OutputId`: Left, Right, EOC, PlayGate, RecGate, Env
- `LightId`: Rec, Play, PitchMode, LoopMode, SOS, Line

## 5. Inputs and Outputs

### Inputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `LEFT_INPUT` | Audio | ±10 V | Left (mono if R empty) |
| `RIGHT_INPUT` | Audio | ±10 V | Right |
| `RECORD_TRIGGER` | Gate | > 1 V | Record toggle |
| `PLAY_TRIGGER` | Gate | > 1 V | Play retrigger |
| `SPEED_CV_INPUT` | CV | ±10 V | Speed |
| `START_CV_INPUT` | CV | ±5 V | Start position |
| `LENGTH_CV_INPUT` | CV | ±5 V | Length |
| `PITCH_CV_INPUT` | CV | 1 V/oct | V/Oct |
| `GATE_INPUT` | Gate | > 1 V | Sustain |

### Outputs

| Name | Type | Range | Description |
|------|------|-------|-------------|
| `LEFT_OUTPUT` | Audio | ±10 V | Left |
| `RIGHT_OUTPUT` | Audio | ±10 V | Right |
| `EOC_OUTPUT` | Gate | 0/5 V | End of cycle |
| `PLAY_GATE_OUTPUT` | Gate | 0/10 V | Playing |
| `REC_GATE_OUTPUT` | Gate | 0/10 V | Recording |
| `ENV_OUTPUT` | CV | 0–10 V | Envelope |

### Bypass

`configBypass(LEFT_INPUT, LEFT_OUTPUT)` and `configBypass(RIGHT_INPUT, RIGHT_OUTPUT)`.

## 6. Signal Processing Pipeline

1. **Sample rate update**.
2. **Read options** — Pitch mode, play mode, direction, line, SOS, level, xfade, mix, SOS feedback, declick.
3. **Read controls with CV** — Start, length, speed.
4. **Handle record trigger** — Toggle recording state.
5. **Start/stop recording**:
   - If starting, allocate temp buffers (or start SOS overdub).
   - If stopping, move temp buffers to live, rebuild peaks.
6. **During recording**: append to temp buffers; update peak overview (10 ms buckets).
7. **Playback transport**:
   - If play trigger and sample exists, start playing.
   - If play button on and sample exists, start playing.
   - If play button off, stop.
8. **If playing**:
   - Read start/length, compute loop bounds.
   - Advance playhead.
   - Handle loop boundary (wrap, bounce, or stop for one-shot).
   - Read with interpolation (or crossfade at loop boundary).
   - If SOS recording, write input to buffer at playhead.
   - Apply fade-in, envelope, output.
9. **Mix with input** — Equal-power crossfade.
10. **Write outputs**.
11. **Update meters, lights**.

## 7. Important Functions

### `spLoadWav(const std::string &path, std::vector<float> &left, std::vector<float> &right, float *fileSampleRate)`

Parses the WAV file, converts to float. Handles PCM 8/16/24/32 and IEEE float.

### `spSaveWav(const std::string &path, const std::vector<float> &left, const std::vector<float> &right, float sampleRate)`

Writes a 16-bit PCM stereo WAV.

### `Sampler::loadFile(const std::string &path)`

Loads a WAV into the live sample.

### `Sampler::loadWavDialog()`, `saveWavDialog()`

OS dialogs for file I/O.

### `Sampler::normalizeSample()`

Scales the sample so peak = `SP_WAV_FULLSCALE`.

### `Sampler::reverseSample()`

Reverses the sample and mirrors the playhead.

### `Sampler::trimToLoop()`

Trims the sample to the current loop region.

### `Sampler::process(const ProcessArgs &args)`

The main callback described in §6.

## 8. Mathematical Implementation

### Playhead advance

$$
\text{playhead} \leftarrow \text{playhead} + \text{speed} \cdot \text{rateRatio}
$$

### Loop crossfade

$$
\text{out} = \text{current} \cdot f + \text{otherEnd} \cdot (1 - f)
$$

### Anti-click fade

$$
\text{gain} = f^2(3 - 2f)
$$

### Envelope attack/decay

$$
\text{env} \leftarrow \text{env} + \Delta t / \text{attackTime} \quad (\text{attack})
$$

$$
\text{env} \leftarrow \text{env} - \Delta t / \text{decayTime} \quad (\text{decay})
$$

### SOS overdub

$$
\text{buffer}[i] \leftarrow \text{buffer}[i] \cdot fb + \text{in}
$$

### Equal-power mix

$$
\text{out} = \text{playback} \cdot \cos(m \cdot \pi/2) + \text{input} \cdot \sin(m \cdot \pi/2)
$$

### Playback speed (V/Oct)

$$
\text{speed} = \text{fine} \cdot 2^{\text{pitchCV}}
$$

### Playback speed (Speed mode)

$$
\text{speed} = \text{knob} + \text{CV} \cdot \text{atten} \cdot \text{clamp}(\text{cv}/5 - 1, -1, 1)
$$

## 9. Performance and Optimization

### CPU Usage

- Per sample: 1 playhead advance + 1-2 reads + envelope + mix ≈ 30 flops
- Display: 280 × 2 UI elements + 56 waveform samples ≈ 1000 flops/frame
- Total: ~30 flops/sample

### Memory Allocation

- Buffers: up to 300 s × 48 kHz × 2 channels = 28.8 M floats ≈ 115 MB (worst case)
- Realistic: 1–10 s samples = 96 K–960 K floats = 384 KB–3.8 MB
- Peak overview: 1024 floats
- No allocation in `process()` except on record/load

### Buffers

- Left/right sample buffers
- Left/right recording buffers (temp)
- Peak overview buffers

### Real-time Processing

- All loops bounded
- Mutex (`std::mutex bufferMutex`) protects sample data
- File I/O is done outside the lock
- Recording appends to temp buffers (amortised allocation)

### Thread Safety

- `bufferMutex` protects all sample data access
- Atomics (`isRecording`, `isPlaying`, `playhead`, etc.) for UI thread communication
- UI thread reads `peakMin`/`peakMax` under the mutex

### Numerical Stability

- `clamp` on all parameters
- `!std::isfinite` checks in WAV loading
- No feedback loops except SOS overdub (which is clamped to ±10 V)

## 10. Dependencies

### Libraries/Headers

- `plugin.hpp` — VCV Rack core
- `agmm_ui.hpp` — shared AGMM UI helpers
- `<vector>`, `<string>`, `<cstdio>`, `<cstdint>`, `<cmath>`, `<cstring>`, `<algorithm>`, `<atomic>`, `<mutex>`, `<fstream>`, `<osdialog.h>`

### External Classes

- `rack::Module`, `rack::TransparentWidget`, `rack::Widget`, `rack::engine::ParamQuantity`
- `rack::dsp::PulseGenerator`, `rack::dsp::SchmittTrigger`
- `osdialog_filters_parse`, `osdialog_file` — file dialogs
- `agmm::panelBase`, `agmm::bezel`, `agmm::cell`, `agmm::plate`, `agmm::text`, `agmm::screen`, `agmm::plotFrame`, `agmm::sectionHeader`, `agmm::withA`, `agmm::violet`, `agmm::accent`, `agmm::bright`, `agmm::warm`, `agmm::dim`, `agmm::ALIGN_L/C/R`, `agmm::CyanLight`, `agmm::VioletLight`

## 11. Summary

`Sampler` is the most complex module in the AGMM collection. It's a full-featured sample player with recording, WAV I/O, V/Oct tuning, multiple play modes, envelope, SOS overdub, and an interactive waveform display. It's the kind of module you build a whole patch around — patch a sample in, use the start/length/loop controls to define a region, then play it back with a sequencer.

The waveform display is excellent: you can drag to set the loop region, double-click to reset, and see the peak overview in real time. The info strip shows the file name, length, and sample rate. The readouts under each knob show the current value, so you always know what's happening.

The main limitations are: WAV files are limited to 16-bit PCM on save (though loading supports up to 32-bit), and the default sample rate is 44.1 kHz (the module doesn't auto-detect the Rack sample rate for its own buffer). But for a sample player, this is very capable.
