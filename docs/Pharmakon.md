<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Pharmakon.cpp
Generated: 2026-11-04 06:24
-->

This is a complex VCV Rack module implementation for a "Pharmakon" reverb/delay effect unit. Here's a breakdown of the key components:

## Module Structure

The code defines a comprehensive audio effect module with:
- **16 adjustable controls** (knobs) arranged in a grid layout
- **Display visualization** that cycles between WAVE, BARS, and SPECTRUM modes
- **Input/output section** with stereo inputs/outputs and bypass/ping-pong functionality
- **CV control capabilities** for real-time parameter adjustment
- **Advanced audio processing** including FFT-based spectrum analysis

## Key Features

### Control Layout
- 16 knobs arranged in 3 rows × 6 columns
- FREEZE switch (row 3, column 4)
- CV RANGE selector (row 3, column 5) - toggles between bipolar (-5V to +5V) and unipolar (0V to +10V) modes
- I/O section with:
  - Left/Right inputs
  - Ping-Pong bypass switch
  - Bypass switch
  - Left/Right outputs

### Audio Processing
- **FFT-based spectrum analysis** for visualization
- **Scope display** showing waveform in real-time
- **Frequency bars** and **spectrum visualization**
- **Level meters** for left/right channels
- **Soft limiting** option for output protection
- **EQ positioning** control (dry/wet vs. whole output)
- **Low/high shelf frequency settings**

### Visualization System
The display widget supports three visualization modes:
1. **WAVE**: Shows audio waveform with glow effect
2. **BARS**: Frequency bars with peak indicators
3. **SPECTRUM**: Smooth spectrum analysis with gradient coloring

### Technical Details
- Uses NVG (NanoVG) for rendering graphics
- Implements custom FFT processing for real-time audio analysis
- Includes proper scissor clipping for display area
- Features scanline and glass reflection effects for retro CRT appearance
- Supports context menu configuration for all parameters
- Includes soft clipper to prevent output distortion

### Parameter Management
- CV range toggle (bipolar/unipolar)
- Display mode cycling (WAVE/BARS/SPECTRUM)
- EQ position control (post-processing vs. pre-processing)
- Low/high shelf frequency settings
- Output soft limiting option

This module demonstrates advanced VCV Rack development techniques including custom widgets, real-time audio visualization, and comprehensive parameter control with both GUI and CV interfaces. The code is well-structured with clear separation of concerns between the core DSP logic and the graphical user interface components.