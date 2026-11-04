<!--
AGMM AUTOMATIC DOCUMENTATION
Source: PsyReverb2.cpp
Generated: 2026-11-04 06:27
-->

This code defines a VCV Rack module for a reverb effect called "PsyReverb2". Here's a breakdown of its key components:

1.  **Module Structure (`PsyReverb2Module`)**:
    *   Manages the core reverb algorithm, including parameters (pre-delay, size, mix, low/high EQ, etc.), audio buffers, and state variables.
    *   Handles CV input processing for parameters.
    *   Implements the `process` function to handle audio and control signals.
    *   Includes functions for initializing the reverb tank (`initialize`) and updating internal state (`update`).
    *   Contains member variables for storing parameter values, audio buffers (`fftRing`, `scopeMin/Max`, `delayLine`), and flags (`dispFreeze`, `dispBypass`, `clipTimer`).

2.  **Widget Structure (`PsyReverb2Widget`)**:
    *   Defines the visual layout of the module in the VCV Rack interface.
    *   Creates the main panel background (`P2PanelBackground`).
    *   Adds controls (knobs, switches, lights) for parameters like pre-delay, size, mix, etc., using `createParam`, `createInput`, and `createOutput`.
    *   Adds a custom display widget (`P2DisplayWidget`) to visualize the reverb's frequency spectrum or waveform.
    *   Adds a custom CV range toggle widget (`P2CvRangeWidget`).
    *   Implements context menu options for changing settings like CV range, display mode, EQ position, shelf frequencies, and toggling the soft limiter.

3.  **Display Widget (`P2DisplayWidget`)**:
    *   Visualizes the reverb's audio signal or spectrum.
    *   Supports three display modes: WAVE (oscilloscope-style), BARS (frequency bars), and SPECTRUM (smooth frequency curve).
    *   Handles updating the visualization data based on the module's internal buffers (`fftRing`, `scopeMin/Max`).
    *   Includes level meters for left/right channels.
    *   Draws scanlines, a glass reflection effect, and readouts for current settings.

4.  **Custom Widgets**:
    *   `P2PanelBackground`: Draws the main panel background with a gradient.
    *   `P2CvRangeWidget`: A custom widget for toggling the CV input range (bipolar -5V/+5V or unipolar 0V/+10V).
    *   `P2ChoiceItem`: A helper class for creating menu items in the context menu.

5.  **Constants and Definitions**:
    *   Defines various constants like panel dimensions (`P2_PANEL_W`, `P2_PANEL_H`), control positions (`P2_ROW`, `P2_COL_X`), display sizes (`P2_DISP_W`, `P2_DISP_H`), buffer sizes (`P2_FFT_N`, `P2_SCOPE_N`, etc.), and parameter names (`P2_CTL`, `P2_VIS_NAMES`).

6.  **Context Menu**:
    *   Adds a right-click menu to the module with options to change CV range, display mode, EQ position, shelf frequencies, and toggle the soft limiter or clear the reverb tank.

This module provides a sophisticated reverb effect with multiple visualization modes, CV control, and various user-adjustable parameters for shaping the sound.