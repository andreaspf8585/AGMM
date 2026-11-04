# AGMM Documentation

Technical documentation for the AGMM VCV Rack 2 plugin.

## About AGMM

AGMM is a collection of custom VCV Rack modules implemented in C++.

## Module Documentation

| Module | Description |
|---|---|
| [Ambience](Ambience.md) | The Ambience module is a stereo reverb effect designed for the VCV Rack modular synthesizer platform. It implements a Feedback Delay Network (FDN) based reverb algorithm with additional features such as pre-delay, modula |
| [Bastion](Bastion.md) | Technical documentation for this AGMM component. |
| [BlackHoleReverb](BlackHoleReverb.md) | The BlackHoleReverb module is a stereo reverb effect for VCV Rack that simulates a black hole's acoustic environment using a combination of delay lines, feedback, and modulation. It provides a dark, deep, and atmospheric |
| [BouncingBalls](BouncingBalls.md) | Technical documentation for this AGMM component. |
| [CellAutoGate](CellAutoGate.md) | The CellAutoGate module implements a one-dimensional cellular automaton with gate output functionality, designed for VCV Rack. It simulates the evolution of a binary cellular automaton where each cell's state is determin |
| [DarkHall](DarkHall.md) | The DarkHall module is a stereo reverb effect designed for the VCV Rack modular synthesizer platform. It implements a reverberation algorithm based on comb and allpass filters, creating a spatial audio effect that simula |
| [DarkHallGranular](DarkHallGranular.md) | The DarkHallGranular module is a stereo reverb effect designed for the VCV Rack platform, implementing a granular-based reverberation system inspired by "galaxy-style" stereo reverb techniques. It processes stereo audio |
| [Dice](Dice.md) | The Dice module is a polyphonic random voltage generator for VCV Rack that produces gate signals and voltage outputs (V/Oct) based on clock inputs. It generates random voltages within configurable ranges, quantizes them |
| [DiceOnSteroids](DiceOnSteroids.md) | Technical documentation for this AGMM component. |
| [ExrpessArpeggiator](ExrpessArpeggiator.md) | Technical documentation for this AGMM component. |
| [FancyADSR](FancyADSR.md) | The FancyADSR module is a VCV Rack 2 plugin that implements an advanced ADSR (Attack, Decay, Sustain, Release) envelope generator with additional features such as: - Configurable attack and release curve shapes (linear o |
| [Gates](Gates.md) | The Gates module is a digital logic gate implementation for VCV Rack that processes two binary inputs (A and B) through various Boolean operations to produce seven distinct outputs: AND, OR, NOT, NAND, NOR, XOR, and XNOR |
| [HeatRev](HeatRev.md) | The HeatRev module is a stereo reverb and delay processor that combines a cathedral reverberation engine with a heat-grid simulation for warm, organic tonal coloration. It features: - Cathedral Reverb: A high-quality 8-l |
| [InfinityRev](InfinityRev.md) | InfinityRev is a high-quality stereo reverb module for VCV Rack that implements an 8-line Feedback Delay Network (FDN) with advanced features including: - 8-line FDN reverb with Hadamard feedback matrix - Modulated delay |
| [InstantDrone](InstantDrone.md) | The InstantDrone module is a multi-oscillator drone generator designed for VCV Rack. It produces three distinct oscillator voices that can be modulated and processed through various effects including low-pass filtering, |
| [Lifeform](Lifeform.md) | Technical documentation for this AGMM component. |
| [LSDrone](LSDrone.md) | Technical documentation for this AGMM component. |
| [MagicBox](MagicBox.md) | Technical documentation for this AGMM component. |
| [Merger](Merger.md) | The Merger module is a polyphonic signal combiner for VCV Rack. It accepts up to 16 mono audio inputs and merges them into a single polyphonic output. The module dynamically adjusts the number of channels in the output b |
| [Mixer3](Mixer3.md) | The Mixer3 module is a stereo 3-channel audio mixer for VCV Rack. It accepts three stereo input channels (L/R) and provides a single stereo output (L/R). Each channel has independent level control, CV-controlled attenuat |
| [Mixer6](Mixer6.md) | The Mixer6 module is a stereo 6-channel audio mixer for VCV Rack. It accepts six stereo input channels (L/R), each with independent level control, CV-controlled attenuation, and optional CV input for dynamic level adjust |
| [Oscillator](Oscillator.md) | Technical documentation for this AGMM component. |
| [Pharmakon](Pharmakon.md) | Technical documentation for this AGMM component. |
| [Phasmal](Phasmal.md) | Phasmal is a phaser effect module for VCV Rack 2 that implements a digital simulation of a classic analog phaser circuit. It provides stereo phasing with modulated all-pass filter stages, offering controls for rate, dept |
| [plugin](plugin.md) | The AGMM VCV Rack plugin is a collection of audio signal processing modules designed for modular synthesizer environments. It provides a wide variety of effects, oscillators, sequencers, and utility modules that enable u |
| [PsyDelay](PsyDelay.md) | The PsyDelay module is a stereo delay effect for VCV Rack that provides advanced delay functionality with modulation, feedback control, and various audio processing effects. It features a dual-channel delay line with con |
| [PsyLFO](PsyLFO.md) | Technical documentation for this AGMM component. |
| [PsyReverb](PsyReverb.md) | The PsyReverb module is a stereo reverb effect designed for the VCV Rack platform. It implements a Freeverb-style reverb algorithm with modifications to prevent runaway gain, ensuring stable audio processing without clip |
| [PsyReverb2](PsyReverb2.md) | Technical documentation for this AGMM component. |
| [PsySeq](PsySeq.md) | PsySeq is a step sequencer module for VCV Rack that allows users to create and manipulate sequences of voltages over time. It features an 8x8 grid layout with 64 programmable steps, configurable sequence length, start po |
| [Quantara](Quantara.md) | Technical documentation for this AGMM component. |
| [RainStorm](RainStorm.md) | Technical documentation for this AGMM component. |
| [RandomWalk](RandomWalk.md) | Technical documentation for this AGMM component. |
| [Range](Range.md) | The Range module is a signal scaling module that maps an input voltage range to a user-defined output voltage range. It accepts a single audio signal input and scales it linearly between two configurable voltage levels, |
| [Reverb](Reverb.md) | The Reverb module is a VCV Rack plugin implementing a simple yet effective reverb effect using a feedback delay line with damping and drive control. It simulates the natural decay of sound in an acoustic space by introdu |
| [Sampler](Sampler.md) | Technical documentation for this AGMM component. |
| [SickDelay](SickDelay.md) | The SickDelay module is a stereo delay effect for VCV Rack that provides flexible delay time control, feedback, mixing, tone shaping, and modulation. It features: - Stereo input/output processing - Independent left/right |
| [Spikes](Spikes.md) | The Spikes module is a VCV Rack plugin that generates chaotic attractor signals using the Lorenz and Rössler systems, with additional fractal noise modulation. It outputs three-dimensional coordinates (X, Y, Z) along wit |
| [Splitter](Splitter.md) | The Splitter module is a polyphonic signal distributor that takes a single polyphonic input and splits it into up to 16 individual mono outputs. It allows users to route multi-channel audio signals to separate output cha |
| [SpringPendulum](SpringPendulum.md) | Technical documentation for this AGMM component. |
| [Stagger](Stagger.md) | The Stagger module is a stereo voltage-controlled oscillator (VCO) with chaotic pitch modulation capabilities. It generates four standard waveforms (sine, triangle, sawtooth, square) and provides independent stereo pitch |
| [Stagger2](Stagger2.md) | Technical documentation for this AGMM component. |
| [Stasis](Stasis.md) | The Stasis module is a polyphonic signal delay and memory register system designed for VCV Rack. It functions as an 8-stage signal storage system where each stage can hold a sampled voltage value, with the ability to sam |
| [Swapper](Swapper.md) | The Swapper module is a stereo signal routing device designed for VCV Rack that allows users to swap or mix left and right audio signals based on configurable amounts. It enables bidirectional signal swapping between the |
| [SwitchLines1to8](SwitchLines1to8.md) | The SwitchLines1to8 module is a sequencer-style signal switcher that routes an input signal to one of eight output channels based on configurable sequence parameters. It supports multiple sequencing directions (forward, |
| [SwitchLines8to1](SwitchLines8to1.md) | The SwitchLines8to1 module is a multi-channel switch matrix that selects one of eight input signals to route to a single output. It features sequenced switching with configurable direction, length, and crossfade behavior |
| [VCA2](VCA2.md) | The VCA2 module is a dual-channel Voltage-Controlled Amplifier (VCA) designed for the VCV Rack modular synthesizer platform. It allows users to control the amplitude of two independent audio signals using envelope contro |
| [VCAPoly](VCAPoly.md) | The VCAPoly module is a polyphonic voltage-controlled amplifier (VCA) designed for the VCV Rack platform. It provides per-channel gain control with envelope following and global or per-channel limit control. The module s |
| [Vortex](Vortex.md) | Technical documentation for this AGMM component. |
| [Whirlpool](Whirlpool.md) | Technical documentation for this AGMM component. |

## Standalone Header Documentation


## Documentation Generation

- Generator: Python
- AI model: `qwen3-coder:30b`
- Backend: Ollama
- Source language: C++
- Documentation format: Markdown

## Generation Statistics

- C++ files discovered: 50
- Header files discovered: 2
- Successful documentation files: 50

---

Automatically generated documentation for AGMM.