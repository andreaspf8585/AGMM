# AGMM
author: Andreas Gkogkos, andreaspf8585@gmail.com

## Introduction
The AGMM plugin is a comprehensive suite of 61 modular synthesizer modules designed for creating evolving, self-composing soundscapes. Spanning from complete generative ambient machines to specialized effects, chaotic modulation sources, and probability-driven sequencers, this collection represents a complete toolkit for exploring the boundaries between composed and emergent music.

At its core, the collection embraces the philosophy of self-composing machines—instruments that can schedule their own chords, layers, and textures. The flagship AMB module exemplifies this approach: an 8-voice detuned pad with tape-loop bells, Euclidean heartbeat layers, and probabilistic processing that runs through granular clouds and cathedral reverbs. Its compact sibling AMB_MINI offers the same automatic chord progression engine in a smaller footprint, allowing both to coexist in a single patch.

Beyond the generative instruments, the collection offers an exhaustive array of reverbs—from the minimal feedback-delay warmth of Reverb to the fully parametric 8-line FDN cathedrals of InfinityRev and Whirlpool. The HeatRev module pushes further still, combining delay, FDN reverb, and a 2D Laplacian heat-grid diffusion network into a hybrid texture generator with live thermal visualization.

The collection's chaos and modulation sources are equally ambitious: physics-driven generators based on pendula, strange attractors, N-body gravity simulations, and even a Galton board with real ball physics. These aren't abstract LFOs but simulated physical systems integrated at audio rate with RK4 methods, producing genuinely unpredictable yet controllable modulation.

Probability and randomness are woven throughout—from the simple dice rolls of Dice to the 16-voice pattern-recording capabilities of DiceOnSteroids, from the per-channel dice of LuckyGate to Euclidean step grids and generative arpeggiators like ArpLib with its 20-group chord library.

The collection is rounded out by practical utilities: polyphonic logic processors, sequential switches with crossfade, CV range mappers, mixers with per-channel CV control, and a polyphonic oscilloscope. Visual modules like Evolved (a ferrofluid lifeform with its own brain, moods, and trust system) and Lifeform add a dimension of audio-reactive generative eye candy. And Crypto takes a different angle entirely: a frequency-code granular processor that turns every sample into a 16-bit code, decodes it back to sound through three different engines, and lets you watch the numbers dance across a Supercell-style processor chain.

Whether you're building a self-playing ambient patch, exploring chaotic modulation, or simply seeking characterful reverbs and delays, this collection provides the building blocks—and the inspiration—for music that discovers itself.

---
## Module Documentation

[Explore the AGMM modules documentation (alphabeticaly)](docs/README.md)

# AGMM Module Catalogue

The plugin is a collection of **61 modules** grouped into functional families. Most modules are standalone — you can build an entire patch from any single category — but they are designed to chain together (generative sources → modulation → filters/effects → mixers → outputs).

---

## 1. Generative Ambient Instruments
*Self-composing machines that schedule their own chords, layers and textures.*

| Module | Description | 
|--------|-------------|
| **AMB** | 36 HP generative ambient machine (Eno / AMB-1 style): an 8-voice detuned pad, 8 tape-loop bells of unequal prime lengths, sub bass, six Euclidean heartbeat layers, formant choir, sparse music-box plucks, pitched wind swells, shimmering glass partials. Runs through tape ping-pong echo, 28-voice granular cloud, probabilistic glitch stage, and an 8-line FDN reverb. Ten scales, automatic chord progression with 7 s cross-fade, external clock input with pulses-per-beat menu, optional clock-synced echo time. |
| **AMB_MINI** | 28 HP compact sibling of AMB: cross-fading 8-voice pad, 8 tape-loop bells, sub bass, six Euclidean pulse layers, tape echo and an 8-line FDN reverb. Same automatic chord progression and ten-scale selection, without the choir / pluck / air / glass layers or the granular / glitch stages. Can be loaded alongside the full AMB in the same patch. |
| **InstantDrone** | Three-oscillator drone synth with per-oscillator V/Oct pitch, sine/triangle/saw waveforms, mix, slow drift, low-pass filter and reverb send — all CV-able. Optional per-oscillator envelope input with a bypass switch. Multi-tap delay-based reverb. |
| **LSDrone** | Cathedral-style drone synth: two detunable oscillators, resonant LPF, LFO, drift, modulated stereo delay, shimmer reverb, echo and final LPF. All 23 parameters have CV inputs and attenuverters. Display switches between waveform, bar spectrum and spectrum curve. |

---

## 2. Reverbs
*From minimal feedback-delay ambience to fully parametric 8-line FDN cathedrals.*

| Module | Description |
|--------|-------------|
| **Ambience** | Algorithmic reverb built on a 4-line FDN with input all-pass diffusion, per-line damping low-pass filters, shared pre-delay, output shelving EQ, and freeze. CV over decay and mix. |
| **BlackHoleReverb** | Stereo reverb using six parallel modulated delay lines with damping, shared pre-delay and feedback. CV inputs with attenuverters for mix, feedback, darkness and modulation; selectable bipolar/unipolar CV range. |
| **DarkHall** | Stereo reverb built from eight damped comb filters feeding four series all-passes (Schroeder / Freeverb-style) with L/R cross-feed, size, damping and modulation controls, plus a scope and L/R level meters. |
| **DarkHallGranular** | Galaxy-style stereo reverb with an input diffusion chain feeding two cross-coupled delay tanks. Replace, Brightness, Detune and Bigness controls with CV and attenuverters, input/output gain, dry/wet and a reset input. |
| **HeatRev** | Hybrid reverb/echo with three layers: a smoothed stereo feedback delay, an 8-line Cathedral FDN reverb with orthonormal Hadamard feedback, and a 2D Laplacian heat-grid diffusion network that adds a warm, slowly evolving resonant texture. Live thermal display with RT60 readout, L/R meters and energy history. All buffers preallocated (no audio-thread allocation); input sanitising and a watchdog flush keep it stable. |
| **InfinityRev** | Fully parametric 8-line FDN reverb using an orthonormal Hadamard feedback matrix, modulated delays, per-line damping, RT60-based decay and a freeze switch/gate. Input all-pass diffusion, early reflections, width and equal-power dry/wet. |
| **PsyReverb** | Freeverb-style stereo reverb: eight parallel damped comb filters feeding four series Schroeder all-passes, with the wet signal mono-shared between L and R for a wide, stable image. Wet / Dry / Room / Damping each have CV and bipolar attenuverters. Display shows decay envelope, damping response and I/O meters. |
| **PsyReverb2** | "Galactic"-style reverb (Airwindows-inspired) extended with damping, predelay, mod, width, freeze, shelving EQ, low cut and ducking. CV + attenuverter per knob, pre/post-reverb EQ, click-free bypass, tank-clear button/trigger. Display switches between WAVE, BARS and SPECTRUM views. |
| **Reverb** | Minimal mono feedback-delay reverb with a damped feedback loop, soft-clipped write, optional input drive saturation, dry/wet mix and output gain. Warm, feedback-based ambience. Scope and single mono level meter. |
| **Whirlpool** | Polyphonic reverb built on an 8-line FDN per voice, with orthonormal Hadamard feedback, 4-stage input all-pass diffusion, early reflections, RT60-based per-line gains, one-pole damping and per-line modulation. Five CV-able controls (Replace, Brightness, Detune, Bigness, Mix), stereo in/out, and WAVE/BARS/SPECTRUM display. |

---

## 3. Delays & Echoes
*Tape, ping-pong, reverse, and glitch-flavoured stereo delays.*

| Module | Description |
|--------|-------------|
| **PsyDelay** | Psychedelic stereo delay (up to 2 s) with feedback, reverse, ping-pong and phase-flip routing, an input gate, and wet character FX: chaos-LFO time modulation, cross-mod, glitch, ring mod, sample-rate reduction, bit-crush, howl and freeze. Display shows the full delay line as a mirrored timeline with tap markers and state chips. |
| **SickDelay** | Stereo delay with independent / linked L/R times (or clock-synced note divisions), feedback, mix, tone, drive, modulation, ping-pong, bit-crusher and freeze. Display shows the delay-line contents as a timeline with L/R read taps and all state readouts. |

---

## 4. Multi-Effects & Processors
*Everything-in-one boxes that combine several effect blocks, plus the frequency-code granular processor.*

| Module | Description |
|--------|-------------|
| **Crypto** | 54 HP polyphonic frequency-code granular processor. Every audio sample is coded as a 16-bit frequency code (LOW..HIGH exponential map) and shown as binary/octal/decimal/hex numbers; a clocked capture window per tick drives three engines (TONE mean-frequency oscillator, CRYPTO code-driven grain pool, CLOUD audio granulator). Supercell-style processor: POSITION, PITCH, SHAPE, MIX, SPACE, FEEDBACK, GAIN IN, GAIN OUT (all CV-able), 1 V/oct, tape-style saturating feedback into a ~5.4 s recording buffer, Freeverb-style reverb, and a NUMBERS / SPECTROGRAM / WAVE display. |
| **Pharmakon** | Stereo multi-FX: modulated stereo delay (ping-pong, detune/chorus tap), Schroeder reverb, distortion, two-band shelving EQ, spacy LFO-modulated LPF and M/S width. 16 main parameters with individual CV and attenuverters. Display switches between waveform, bar spectrum and spectrum curve. |
| **Vortex** | Dual-mode reverb / echo. Reverb is a plate with 12 CV-able parameters (pre-delay, size, decay, diffusion, damping, low cut, mod speed/depth, mix); echo is a modulated delay with ping-pong. Freeze (latch + gate), Clear (fade-out/reset/fade-in), stereo in/out switches, and WAVE/BARS/SPECTRUM display. |

---

## 5. Filters & Waveshapers
*Character filters and analogue-flavoured non-linearities.*

| Module | Description |
|--------|-------------|
| **Bastion** | Multi-engine filter offering ten selectable classic topologies (Moog ladder, TB-303, MS-20, SEM SVF, Steiner-Parker, CEM3320, Roland OTA, EMS diode, EDP Wasp, Buchla LPG). Simultaneous LP/BP/HP outputs with CV-able cutoff, resonance and drive. |
| **MagicBox** | Waveshaper with 20 selectable shaping functions, auto-level, DC blocker, drive, dry/wet and output level. Display shows the live transfer curve with an operating-point trail, eight harmonic bars, and IN/OUT scopes. |
| **Phasmal** | Stereo phaser with a chain of 2–12 first-order all-pass stages and global feedback, swept by a sine or triangle LFO. Stereo spread and CV over rate/depth/center/feedback. Display shows the exact magnitude response of the current settings, the sweep range and an LFO scope. |

---

## 6. Oscillators & Voices
*Tonal sources, from band-limited wavetables to Shepard–Risset glissandi.*

| Module | Description |
|--------|-------------|
| **Ladder** | Shepard–Risset glissando VCO. Up to 20 octave-spaced partials weighted by a Gaussian window in log-frequency space; SPEED slides them together so pitch appears to rise (or fall) endlessly. Stereo output with per-partial panning; oscilloscope and partial-spectrum display. |
| **Oscillator** | Polyphonic band-limited wavetable oscillator. 30 waveforms baked into an 11-level mipmap selected by pitch so table playback is alias-free. LO CUT and HI CUT SVF stages, feedback and FM as phase offsets. Display shows waveform, spectrum and filter response plus a clickable 30-wave gallery. |
| **Stagger** | Polyphonic stereo VCO with independent filtered-random pitch drift per channel (chaos), giving a wide, unstable tone. Sine/triangle/saw/square with minBLEP anti-aliasing; 1 V/oct pitch input (no attenuverter) and CV + attenuverter on chaos, waveform and pulse width. Display shows the waveform with a phase marker plus a scrolling chaos history. |
| **Stagger2** | 32 HP expanded Stagger: 40 waveforms (classic, band-limited additive, waveshapers, sync/PD/ring-mod, classic shapes) with three Plaits-style macro controls (HARMONICS / TIMBRE / MORPH, each with CV + attenuverter), plus a per-voice modulated stereo delay, an 8-line FDN reverb, a trigger/gate envelope, a per-voice compressor, master gain and a master compressor. Polyphonic VCO+FX output plus a mono-summed stereo FX pair. Wave scope, ENV/L/R meters and FX readouts. |
| **TritoneParadox** | Shepard-tone pair generator for the tritone paradox illusion. Two tones separated by a 5–7 semitone interval with an adjustable Gaussian spectral envelope (centre + width), optional lower/higher context tone, and Manual/Auto sequencing. Display shows the envelope with partials, plus a pitch-class circle marking the interval. |

---

## 7. Chaos & Modulation Sources
*Physics-driven modulation: pendula, attractors, N-body systems, gravity wells.*

| Module | Description |
|--------|-------------|
| **BouncingBalls** | Chaos generator with up to 16 balls bouncing inside a 20 V × 20 V arena. Polyphonic outputs for X/Y position, per-ball random V/Oct, zero-crossing gates, ball-ball collision gates, and wall-hit gates. |
| **Galton** | Interactive Galton board with real ball physics: balls fall under gravity, bounce off a triangular peg lattice and collect in bins, forming a bell curve. Outputs slot gates, slot/row CV, landing/peg-hit triggers, and polyphonic V/Oct + gates. |
| **PsyLFO** | Chaotic modulation source built from a real double-pendulum physics simulation integrated with RK4 at audio rate. Per pendulum: X/Y position outputs, V/Oct that latches a random value on each zero-crossing, and a gate output. Global Speed, Gravity, Damping, Kick and Reset. Live animated display. |
| **RandomWalk** | Polyphonic random-walk generator: up to 16 walkers wander in a ±10 V arena with Gaussian steps, configurable step size, speed and AR(1) smoothness (momentum). Per walker: X/Y position, axis-crossing gates, and random V/Oct latched on each crossing. Arena display with fading trails and a 16-bar position strip. |
| **Spikes** | Chaotic modulation source combining Lorenz and Rössler strange attractors (integrated with RK4) with a fractal-correlated fBm noise generator. Blend, fractal amount/speed, speed (1 V/oct) and scale controls; outputs X/Y/Z/MIX. Rotatable 3D trail view of the blended attractor. |
| **SpringPendulum** | Chaotic modulation source based on a physical spring-pendulum: a mass on four linear springs with optional gravity, damping and random force, integrated with RK4. Mouse-grabbable. Outputs X/Y position, plus a random V/Oct latched on each X = 0 crossing with a matching gate. Live dial display with fading trail. |
| **XBodyProblem** | N-body gravity LFO: simulates 2–10 point masses under mutual gravity inside a unit square. Per-mass X/Y position, speed and wall-event trigger outputs as poly channels. Wrap (Pac-Man) or bounce walls, equal or random masses, adjustable gravity/speed/softening. Big square display with bodies, velocity vectors, gravity links, trails and centre of mass. |

---

## 8. Random & Probability
*Dice rolls, weighted randomness and probabilistic gate / pitch generation.*

| Module | Description |
|--------|-------------|
| **Dice** | Polyphonic random-gate / random V/Oct generator. On each clock edge, per-voice dice rolls open gates with configurable probability and latch random voltages, output raw and quantized to one of 60+ scales. |
| **DiceOnSteroids** | Expanded 16-voice probabilistic gate / random V/Oct generator with random modes (uniform, walk, drunk, gaussian), a 16 × 64 pattern recorder/player, train mode, glide, CV over most parameters, and an RNG seed input. |
| **LuckyGate** | 8 HP polyphonic probabilistic gate. On every rising edge each channel rolls an independent die against the PROBABILITY knob (plus 0–10 V CV); a successful roll makes the output follow the input gate for its entire duration, otherwise it stays low. Live 16-cell channel grid (pass = accent, reject = warm), 32-slot rolling history, running HIT % and CH readouts, and a probability ring with a CV-modulated effective-value dot. |

---

## 9. Sequencers & Pattern Generators
*Everything from Euclidean-style step grids to generative arpeggiators.*

| Module | Description |
|--------|-------------|
| **Arplib** | 22 HP chord-progression arpeggiator. 20 stored chord sequences backed by a 20-group library (10 minor, 10 major, 50 progressions each) with per-chord circle-of-fifths overrides. 9 arpeggio patterns, swing, probability, gate randomisation, velocity humanise, and a RNDNEXT switch that replaces the fixed-order chord advance with a uniform random pick. Two-way SEQ knob / SEQ CV selection, PROG CV progression scanning, polyphonic chord output, change and loop triggers, and a live chord-strip + 16-step piano-roll display. |
| **CellAutoGate** | One-dimensional elementary cellular automaton (Wolfram rules 0–255) on a 64-cell row, sampled at 1–16 positions to produce a polyphonic gate pattern. Internal or external clock, randomize/reset, with a history display and rule truth table. |
| **ExpressArpeggiator** | Generative 16-step arpeggiator with root, 60+ scales, octave range and forward/backward/random/drunk/pendulum direction. Interactive display to paint notes, mute/lock steps, set loop length and direction; per-step mutation and randomization. |
| **PsySeq** | 8 × 8 step sequencer with per-cell voltage and probability trimpots. Forward/reverse/random direction, adjustable start step, random-voltage-per-step mode, and periodic mutation (with configurable wait cycles). Outputs CV, inverted CV, cycle gate and step trigger; voltage knobs accept typed volts or note names. Live 8 × 8 grid and status screen. |
| **Stasis** | 8-stage clocked sample-and-hold shift register with per-stage probability, slew (1 ms – 2 s), loop-back and bipolar level. Each stage has its own input (breaking the chain) and output; also a poly 8-channel output. Display shows per-stage bars, chain arrows, loop bus and a 24-column history heat map. |

---

## 10. Pitch Processing
*Quantisation, tuning and scale mapping.*

| Module | Description |
|--------|-------------|
| **Quantara** | Polyphonic 1 V/oct pitch quantizer with 16 built-in scales plus an editable custom scale. Root/scale/transpose via knob + CV, glide, three quantize modes (nearest/down/up), and optional clocked sample & hold. Each voice emits a note-change trigger. Display shows a pitch-class wheel, staircase transfer plot and a clickable keyboard for editing the custom scale. |

---

## 11. Sampling
*Audio capture and playback.*

| Module | Description |
|--------|-------------|
| **Sampler** | Mono/stereo sample recorder + player. Record from L/R (new sample or Sound-on-Sound overdub), load/save WAV, adjust playback speed or V/Oct tuning, start/length, direction, one-shot/loop/ping-pong modes, anti-click fades, A-D envelope with sustain gate, loop crossfade, and input/playback mix. Interactive waveform display (drag to set the loop region). |

---

## 12. Utilities & Logic
*Routing, logic, range mapping, crossfading and master clock generation.*

| Module | Description |
|--------|-------------|
| **Clockwork** | 32 HP chainable master clock with six sub-clocks. Master tempo 30–300 BPM (snap knob) or external BPM input as 1 V/oct CV or as a clock at 4/8/12/24 PPQN with PLL-style tempo and phase lock. Each sub-clock has a multiply/divide ratio (x1…x64, /1…/64, including x1.5 and x2.5), swing, pulse width, delay, swing/PW CV inputs and a gate output. Phase-based engine keeps all clocks in sync through tempo changes; ratio changes are deferred to the next master bar and flagged with a SYNC badge on the per-row scope. Run/Reset buttons + inputs, Reset/Run/BPM chain outputs, optional reset-on-stop. Per-row two-period pattern scope with ghost outline, live playhead and frame progress; master scope with RUN/source/RST/LOCK status chips. |
| **Gates** | Polyphonic logic processor taking two gate inputs (A, B) and producing seven simultaneous logic outputs: AND, OR, NOT (A), NAND, NOR, XOR, XNOR. Live 9-row scrolling logic scope with adjustable time-per-screen. |
| **Merger** | 16 monophonic inputs merged into one polyphonic output. Output channel count equals the highest connected input, so gaps carry 0 V and channel numbering stays stable. Display shows the active channel count and a 16-segment bar. |
| **Range** | Polyphonic utility CV range mapper: linearly rescales an adjustable input window [LOW IN, HIGH IN] onto an adjustable output window [LOW, HIGH], all four endpoints set by front-panel knobs (−10 V .. +10 V). Optional output clamp and input-range presets via right-click menu. Display shows the live transfer curve, output range markers, input window, and a moving dot for the current input/output (multi-channel dots for poly). |
| **Splitter** | Expands a single polyphonic input into sixteen monophonic outputs. Each output jack carries one channel of the incoming poly signal (or 0 V). Display shows the current channel count and a 16-segment activity bar. |
| **Swapper** | Dual-channel crossfader. Independent L→R and R→L amounts (knob + CV/attenuverter) blend the two signals into each output. Routing-diagram display shows the four link weights as line thickness plus IN/OUT level nodes. |

---

## 13. Mixers & VCAs
*Level control, mixing and polyphonic gain.*

| Module | Description |
|--------|-------------|
| **Mixer3** | Stereo 3-channel mixer with per-channel L/R inputs, a level knob plus CV with bipolar attenuverter, and a master gain. Scope and L/R level meters. CV range selectable (bipolar/unipolar) from the menu. |
| **Mixer6** | Six-channel version of Mixer3 with the same structure (per-channel L/R inputs, level knob + CV/attenuverter, master gain), laid out in narrower stacked plates for a taller panel. Scope and L/R meters. |
| **VCA2** | Dual VCA with a per-channel gain-limit knob (0× .. 2×). ENV input drives the gain (unpatched = unity), and the limit clamps the maximum. The two vertical limit bars double as interactive controls (click/drag to set, double-click to reset) and as envelope + output-peak meters. |
| **VCAPoly** | Polyphonic VCA (up to 16 voices). Per-voice gain = ENV × limit; limit comes from the global knob, or from a per-voice LIMIT CV when patched. ENV unpatched gives unity gain. Display stacks 16 voice rows with gain bars, output-peak ticks and per-row or global limit markers. |

---

## 14. Envelopes
*Shaped control voltages.*

| Module | Description |
|--------|-------------|
| **FancyADSR** | Polyphonic (up to 16-channel) envelope generator with per-stage CV, selectable linear/exponential attack and release shapes, switchable ADSR/AD modes (AD treats the gate as a trigger), loop mode, a TEST gate button, and a 5 ms end-of-release (end-of-decay in AD mode) pulse. The scope overlays the ideal envelope preview with live traces for every channel, plus stage/state chips, A/D/S/R + output readouts, a level meter and a user-adjustable time axis; the right-click menu selects which channel's readouts are shown. |

---

## 15. Switches & Routing
*Sequential routing of one signal across many or many across one.*

| Module | Description |
|--------|-------------|
| **SwitchLines1to8** | 1-to-8 sequential switch: one SIGNAL input routed to one of eight outputs by a clocked sequencer (LENGTH/START/DIR: forward/reverse/pendulum/random). Per-output crossfade on change, one gate per output (high when selected), EOC trigger, and a live routing display with fan-out curves, meters and history strip. |
| **SwitchLines8to1** | 8-to-1 sequential switch: one of eight inputs routed to a single output by a clocked sequencer, with crossfade, one gate per input (high when selected), EOC trigger, and a live routing display with input meters, converging curves and history strip. |

---

## 16. Noise, Weather & Field Recordings
*Environmental textures and non-tonal sounds.*

| Module | Description |
|--------|-------------|
| **RainStorm** | Generative weather module: layered rain (pink-noise wash, two Poisson-timed resonant droplet layers, hiss, distant rumble) with slow gusts, plus up to two overlapping thunder voices with random distance and flash-to-bang delay. Stereo echo and Freeverb-style reverb. Animated rain/lightning display. |

---

## 17. Visualisers
*Audio-reactive graphics, generative eye candy and live signal monitors.*

| Module | Description |
|--------|-------------|
| **Evolved** | 36 HP "evolved" ferrofluid lifeform visualiser — the big brother of Lifeform. The glossy black 3D creature (lobed blob, Rosensweig spikes, orbiting droplets, perspective, depth sorting, normal-based gloss) lives in a wide rectangular glass tank and has a small brain of its own. It reacts to sound (three bands, onset detector, silence detector) but does not need it: it has **energy**, gets hungry, hunts food pellets, wanders, naps on the tank floor and remembers where it ate. It learns the **tempo** (onset intervals → beat period/phase) and anticipates the next beat even when a hit is missing. It senses the **mouse** — leaning spikes toward it, reaching out with droplets, swimming after it — and keeps a **trust** value: gentle motion earns trust, fast aggressive motion makes it wary and it darts away. Moods: RESTING, WANDERING, HUNTING, CURIOUS, DANCING, STARTLED. Controls: FIELD, MOTION, VISC, SPIKES, SENS, GLOSS, INTEL, CURIO, FOOD, SIZE (all with CV). Buttons: PAUSE, RESET, REACT (audio reactive), MOUSE (mouse sense). Outputs: ENV, FIELD, X, Y (±5 V), ENERGY (0–10 V), EAT (trigger). Purple bioluminescence in the dark, cyan-glowing food pellets. The brain runs in the module so outputs keep working even when the display is not visible. |
| **Lifeform** | Audio-reactive 3D ferrofluid visualiser. A glossy black lobed blob with ~140 Rosensweig spikes and orbiting droplets tumbles inside a glass dish. Low/mid/high bands, onset detection and silence detection drive shape, spikes and droplet orbits. |
| **Plot** | 24 HP polyphonic oscilloscope / signal visualizer. Two polyphonic inputs (A and B) and two polyphonic thru outputs. Free / Auto / Normal trigger with Schmitt hysteresis and sub-sample interpolation, external trigger input, 10 × 8 voltage/time grid, min/max envelope traces, display hold, and per-window statistics (Vpp, Max, Min, RMS, Avg, frequency) for the focus channel of each input. |

---

## Quick Reference by Category

| Category | Count | Modules |
|----------|-------|---------|
| Generative Ambient | 4 | AMB, AMB_MINI, InstantDrone, LSDrone |
| Reverbs | 10 | Ambience, BlackHoleReverb, DarkHall, DarkHallGranular, HeatRev, InfinityRev, PsyReverb, PsyReverb2, Reverb, Whirlpool |
| Delays & Echoes | 2 | PsyDelay, SickDelay |
| Multi-Effects & Processors | 3 | Crypto, Pharmakon, Vortex |
| Filters & Waveshapers | 3 | Bastion, MagicBox, Phasmal |
| Oscillators & Voices | 5 | Ladder, Oscillator, Stagger, Stagger2, TritoneParadox |
| Chaos & Modulation | 7 | BouncingBalls, Galton, PsyLFO, RandomWalk, Spikes, SpringPendulum, XBodyProblem |
| Random & Probability | 3 | Dice, DiceOnSteroids, LuckyGate |
| Sequencers & Patterns | 5 | Arplib, CellAutoGate, ExpressArpeggiator, PsySeq, Stasis |
| Pitch Processing | 1 | Quantara |
| Sampling | 1 | Sampler |
| Utilities & Logic | 6 | Clockwork, Gates, Merger, Range, Splitter, Swapper |
| Mixers & VCAs | 4 | Mixer3, Mixer6, VCA2, VCAPoly |
| Envelopes | 1 | FancyADSR |
| Switches & Routing | 2 | SwitchLines1to8, SwitchLines8to1 |
| Noise & Weather | 1 | RainStorm |
| Visualisers | 3 | Evolved, Lifeform, Plot |
| **Total** | **61** | |
## Module Documentation

[Explore the AGMM modules documentation (alphabeticaly)](docs/README.md)

[Just screenshots of the modules (alphabeticaly)](docs/ImagesList.md)