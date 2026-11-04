<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Whirlpool.cpp
Generated: 2026-11-04 06:39
-->

The provided C++ code defines a VCV Rack module named `Whirlpool`, which is a reverb-like audio effect with multiple visualization modes (waveform, bars, spectrum). Here's a breakdown of its key components:

### 1. **Module Definition (`Whirlpool`)**
- Inherits from `rack::Module`.
- Defines parameters for:
  - Input/output stereo switching.
  - Clear button and light.
  - Replace parameters (5 knobs with CV inputs and attenuverters).
  - Display mode (WAVE/BARS/SPECTRUM) and channel selection.
- Includes internal buffers for:
  - Spectrum analysis (`specBuf`, `specPos`).
  - Scope data (`scopeMinL`, `scopeMaxL`, etc.).
  - Line levels and delay times (`dispLine`, `dispDelayMs`).
- Uses a `dsp::SchmittTrigger` for the clear button.
- Implements a `process()` function that:
  - Updates input/output signals.
  - Processes FDN (Feedback Delay Network) lines.
  - Updates scope buffers.
  - Computes spectrum for visualization.
  - Handles clear fade and meter updates.

## Module Screenshot

![Whirlpool VCV Rack Module](images/Whirlpool.png)

### 2. **Widget Definition (`WhirlpoolWidget`)**
- Inherits from `rack::ModuleWidget`.
- Sets up the module's UI:
  - Panel background (`WHPanel`).
  - Display area (`WHDisplay`).
  - Knobs, CV jacks, attenuverters, and buttons for parameters.
  - Context menu to switch display modes.

### 3. **Display Widget (`WHDisplay`)**
- Inherits from `rack::OpaqueWidget`.
- Handles drawing of the visualization:
  - **WAVE Mode**: Shows min/max envelope of L/R outputs and dry input.
  - **BARS Mode**: Displays 8 FDN line levels with peak hold and delay lengths.
  - **SPECTRUM Mode**: Shows log-frequency FFT of output.
- Implements:
  - FFT computation (radix-2, in-place).
  - Spectrum analysis using Hann windowing.
  - Scrolling scope buffer for waveform display.
  - Peak hold and smoothing for spectrum visualization.
  - Stereo meters for output levels.
  - Tabbed interface for switching modes.
  - Context menu to change display mode.

### 4. **Key Features**
- **Multi-Mode Display**: Toggle between waveform, bar graph, and spectrum views.
- **FDN Reverb**: Implements a feedback delay network with adjustable parameters.
- **Real-time Visualization**: Updates in response to audio input and parameter changes.
- **Stereo Support**: Handles both mono and stereo inputs/outputs.
- **Clear Functionality**: Resets the reverb tail with a fade effect.

### 5. **Technical Details**
- Uses `nvg` for drawing (NanoVG).
- Implements custom FFT algorithm for spectrum analysis.
- Manages buffers for real-time audio processing.
- Handles UI interaction via mouse events.
- Includes a context menu for changing display modes.

This module is designed to be a versatile reverb effect with visual feedback, suitable for experimental and musical applications in VCV Rack.