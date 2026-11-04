<!--
AGMM AUTOMATIC DOCUMENTATION
Source: Oscillator.cpp
Generated: 2026-11-04 06:24
-->

This code defines a VCV Rack module for an oscillator with advanced waveform visualization and filtering capabilities. Here's a breakdown of its key components:

## Core Functionality

**Oscillator Module**: 
- Generates waveforms with feedback control
- Supports 128 different waveforms (OS_NW = 128)
- Features multi-voice operation (up to 3 voices)
- Includes filter controls for low/high cutoff frequencies
- Has CV inputs for modulation

## Key Features

**Visualization System**:
- Real-time waveform display with feedback simulation
- Harmonic spectrum analysis showing frequency content
- Filter response curve visualization
- Interactive wave bank gallery (click to select waveforms)
- Phase cursors for each voice
- Scanline and vignette effects for CRT-like appearance

**Controls**:
- 9 knobs for various parameters:
  - Waveform selection
  - Feedback amount
  - Voice count
  - Frequency modulation
  - Filter cutoffs
  - Peak voltage
- Multiple CV inputs for modulation
- Audio outputs (main and sum)

## Technical Implementation

**Waveform Generation**:
- Uses lookup tables for efficient waveform synthesis
- Implements feedback simulation with alias-free harmonic content
- Supports 256-point waveforms with 128 harmonics

**Display System**:
- Custom drawing routines using NanoVG
- Real-time spectrum analysis with FFT-like processing
- Interactive gallery with hover effects
- Color-coded visualization elements (accent, warm, violet colors)

**Audio Processing**:
- Multi-voice synthesis with phase control
- Filter implementation using 4th-order Butterworth-style responses
- Peak voltage monitoring
- Proper scaling for audio signal levels

## UI Components

1. **OSDisplayWidget**: Main visualization area showing waveform and spectrum
2. **OSKnobArcs**: Decorative arc elements around knobs
3. **OSPanelBackground**: Module panel with styling
4. **OscillatorWidget**: Complete module interface with all controls

The oscillator provides a rich, visual way to explore different waveforms while offering extensive modulation capabilities through its CV inputs and parameter controls. The display updates in real-time to show both the current waveform and its harmonic content, making it useful for both sound design and educational purposes.

This is a sophisticated oscillator module that combines traditional synthesis with modern visualization techniques, typical of high-end VCV Rack modules.