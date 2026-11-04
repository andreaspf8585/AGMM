<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Quantara.cpp
Generated: 2026-11-04 06:28
-->

This code defines a VCV Rack module called `Quantara`, which is a polyphonic pitch quantizer with scale and mode selection, visual feedback, and interactive controls.

Here's a breakdown of the key components:

1.  **Module Definition (`Quantara` class):**
    *   Inherits from `rack::Module`.
    *   Defines parameters (knobs, switches) for ROOT, SCALE, SHIFT, GLIDE, and MODE.
    *   Defines inputs (CLOCK, ROOT_CV, SCALE_CV, SHIFT_CV, GLIDE_CV, IN).
    *   Defines outputs (OUT, TRIG).
    *   Stores internal state like `vRoot`, `vScale`, `vShift`, `vMask`, `vNote`, `heldIn`, `trailIn`, `trailOut`, etc.
    *   Implements `process()` to handle incoming signals and update the module's state based on inputs, parameters, and clocking.

2.  **Widget Definition (`QuantaraWidget` class):**
    *   Inherits from `rack::ModuleWidget`.
    *   Sets up the physical layout of the module in the Rack UI (size, screws).
    *   Adds the custom display widget (`QZDisplayWidget`) to show visual feedback.
    *   Adds knobs for ROOT, SCALE, SHIFT, GLIDE parameters.
    *   Adds input/output ports for CV and audio signals.
    *   Implements `appendContextMenu()` to add a context menu with options to:
        *   Select predefined scales.
        *   Choose quantization mode (Nearest, Down, Up).
        *   Edit the custom scale (copy current, clear, all notes on).

3.  **Display Widget (`QZDisplayWidget` class):**
    *   Inherits from `rack::Widget`.
    *   Handles drawing the visual representation of the quantizer.
    *   Draws a pitch-class wheel showing the selected scale and root note.
    *   Draws a transfer staircase plot showing how input pitches are mapped to output notes.
    *   Draws a keyboard display showing the current scale with active notes highlighted.
    *   Shows readouts for root, scale, shift, and channel status.
    *   Implements interactive features like clicking keys to toggle scale notes.

4.  **Key Features:**
    *   **Polyphonic:** Can handle multiple input channels (up to `QZ_MAX_CH`).
    *   **Scale Selection:** Supports 12 predefined scales and a custom user-defined scale.
    *   **Quantization Modes:** Nearest, floor (down), or ceiling (up) quantization.
    *   **Visual Feedback:**
        *   Pitch-class wheel showing the scale and root note.
        *   Transfer staircase plot visualizing the quantization function.
        *   Keyboard display with active notes highlighted.
        *   Clock indicator lamp.
        *   Readouts for parameters and status.
    *   **Interactive Editing:** Clicking keys on the keyboard allows users to edit the custom scale.
    *   **Trail Display:** Shows the history of input/output note pairs for channel 1.

5.  **Constants and Helper Functions:**
    *   Defines various constants (`QZ_...`) for layout, colors, sizes, etc.
    *   Includes helper functions like `qzQuant`, `qzMod`, `qzCount` for scale manipulation and quantization logic.
    *   Uses `nvg` (NanoVG) for drawing graphics.

This module provides a sophisticated way to quantize polyphonic input signals according to various musical scales, with a visually rich interface that helps users understand and manipulate the quantization process.