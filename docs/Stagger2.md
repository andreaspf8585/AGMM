<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Stagger2.cpp
Generated: 2026-11-04 06:34
-->

This code defines a VCV Rack module called `Stagger2`, which is a polyphonic synthesizer voice with built-in effects (delay and reverb). Here's a breakdown of its key components:

## Module Screenshot

![Stagger2 VCV Rack Module](images/stagger2.png)


1.  **Module Class (`Stagger2`)**:
    *   Inherits from `rack::Module`.
    *   Manages parameters, inputs, outputs, and the core audio processing logic for one voice.
    *   Contains member variables for various state (e.g., `freq`, `pwValue`, `waveIndex`, `chaosDrift`, `envMax`, `dispFreq`, etc.) and objects for effects (`delay`, `reverb`, `comp`, `compL`, `compR`).
    *   Implements the `process` function which is called for each audio block.
    *   Handles polyphonic voice management (up to 16 voices, controlled by `polyCount`).
    *   Processes input signals (pitch CV, chaos CV, etc.), generates waveform samples, applies effects (delay and reverb), mixes them with the original signal, and applies a master gain and compressor.
    *   Calculates and stores display data (`dispFreq`, `dispPw`, `dispWave`, `dispPhase`, `dispEnv`, `dispTrigConn`) for the widget's visual feedback.
    *   Implements a master compressor on the final FX outputs (`fxSumL`, `fxSumR`).
    *   Updates light indicators (CHAOS and ENV lights) based on internal state.

2.  **Widget Class (`Stagger2Widget`)**:
    *   Inherits from `rack::ModuleWidget`.
    *   Defines the visual appearance of the module in the Rack UI.
    *   Creates a panel (`S2Panel`) with a background, bezel, and labels for knobs and jacks.
    *   Adds the display widget (`S2Display`) to show waveform preview, FX settings, and output meters.
    *   Places all the physical controls (knobs, switches, jacks) on the panel according to predefined positions.
    *   Connects the module's parameters and inputs/outputs to the UI elements.

3.  **Display Widget (`S2Display`)**:
    *   Inherits from `TransparentWidget`.
    *   Draws a custom display area within the module's panel.
    *   Shows a live preview of the waveform being generated, including the current phase.
    *   Displays real-time information about delay time, feedback, reverb time, trigger mode, and envelope level.
    *   Shows output meters for the left and right FX outputs.

4.  **Helper Functions and Constants**:
    *   `s2Wave`: A function to generate waveform samples (sine, triangle, sawtooth, square) based on phase and pulse width.
    *   Various drawing functions (`drawLogo`, `drawPanelBase`, `drawBezel`, `plate`, `ring`, `label`, `screenBase`, `screenFinish`) for creating the visual UI elements.
    *   Constants like `S2_W`, `S2_H`, `S2_T1`, etc., define the dimensions and layout of the module panel.

5.  **Effects Processing**:
    *   **Delay**: Uses a `voice.delay.process` function, likely implementing a stereo delay line with modulation (LFO), feedback, and ping-pong mode.
    *   **Reverb**: Uses a `voice.reverb.process` function, likely a convolution or algorithmic reverb.
    *   **Compressor**: Applies both a per-voice compressor (`voice.comp`) on the VCO output and a master compressor (`compL`, `compR`) on the FX outputs.

6.  **Polyphony**:
    *   The module supports polyphonic operation, processing multiple voices simultaneously (up to 16).
    *   It uses `polyCount` to determine how many voices are active.
    *   Each voice has its own set of parameters and effect processors (`voice.delay`, `voice.reverb`, `voice.comp`).

7.  **Parameters and Inputs**:
    *   Parameters (knobs): Pitch, Chaos, Waveform, Pulse Width, Mult, Delay Time, Delay Feedback, Delay Modulation, Delay Mix, Delay Mode (Ping-Pong), Reverb Size, Reverb Decay, Reverb Mix, Trigger Mode, Gain, Comp, CV Range.
    *   Inputs: V/OCT (for pitch), CHAOS (for chaos modulation), WAVE (for waveform selection), PW (for pulse width), CHAOS RESET (to reset the chaos state), TRIG (to trigger envelope), PITCH CV, CHAOS CV, WAVE CV, PW CV.
    *   Outputs: VCO OUT (the processed voice output), FX L/R (the mixed effects output).

8.  **Visual Feedback**:
    *   The module includes lights for the CHAOS and ENV states.
    *   The display widget provides a real-time waveform preview, showing the current phase, and displays various effect settings and output levels.

In summary, `Stagger2` is a sophisticated polyphonic synthesizer voice with integrated delay and reverb effects, offering a rich palette of sound-shaping capabilities through its extensive parameter set and visual feedback.