<!--
AGMM AUTOMATIC DOCUMENTATION
Source: MagicBox.cpp
Generated: 2026-11-04 06:22
-->

This code defines a VCV Rack module called `MagicBox`, which is a versatile audio signal processor with multiple functions including various waveshaping and distortion algorithms, harmonic analysis, and real-time visualization of input/output signals.

Key features of this module:

1. **Multiple Processing Functions**: The module supports 8 different waveshaping functions (soft clip, hard clip, tanh, etc.) that can be selected via a parameter or CV input.

2. **Real-time Visualization**:
   - Transfer curve display showing the relationship between input and output signals
   - Harmonic analysis showing frequency content of the processed signal
   - Scope displays for both input and output signals with color-coded traces
   - Operating point indicators showing current processing state

3. **Interactive Controls**:
   - Drive control to adjust the intensity of the waveshaping effect
   - Time control to adjust the scope display update rate
   - Function selection via knob or CV input
   - Auto-level and DC-blocking options

4. **Advanced Features**:
   - Scope with historical data visualization (old traces)
   - Trigger detection and status display
   - Limiter indicator showing when signal exceeds thresholds
   - Peak meter displays for input and output signals

5. **Visual Design**:
   - Dark theme with blue accents
   - Scanline effects and vignetting for CRT-like appearance
   - Color-coded elements (blue for main, orange for warnings)
   - Smooth gradients and rounded corners

The module is designed to be a flexible audio processor that can be used for various creative sound design tasks, from gentle waveshaping to aggressive distortion, with real-time feedback through its comprehensive visualization system. The code includes both the module logic and the widget implementation with all visual elements.

To use this module:
1. Connect audio signals to the input jacks
2. Adjust the drive parameter to control the intensity of the effect
3. Select different processing functions using the function knob or CV input
4. Monitor the real-time visualization to understand how the signal is being processed
5. Use the auto-level and DC-blocking options as needed for your specific application