<!--
AGMM AUTOMATIC DOCUMENTATION
Source: RandomWalk.cpp
Generated: 2026-11-04 06:29
-->

The provided code is a complete implementation of the `RandomWalk` module for the VCV Rack modular synthesizer framework. It defines both the core logic (`RandomWalk` class) and its visual representation (`RandomWalkWidget`, `RWDisplayWidget`, `RWBarsWidget`). Here's a breakdown of what this code does:

## Module Screenshot

![RandomWalk VCV Rack Module](images/randomwalk.png)

---

### 🧠 **Core Module Logic (`RandomWalk` Class)**

This part handles all internal state and computation for simulating random walks in 2D space.

#### Key Features:
- **Multiple Walkers (Balls):**
  - Up to `RW_N` (typically 8) independent walkers.
  - Each walker has a position `(x, y)` and previous movement direction `(prevDeltaX, prevDeltaY)`.

- **Movement Mechanics:**
  - Walkers move in discrete steps based on:
    - Speed factor (`speedFactor`)
    - Step size (`stepSize`)
    - Smoothness setting (`smoothness`) for visual interpolation

- **Physics & Boundaries:**
  - When a walker hits a boundary, it reflects off the wall.
  - Reflections are tracked with `wallSide`, `wallPos`, and `flashWall`.

- **Outputs:**
  - Voltage outputs for X/Y positions (`X_OUTPUT`, `Y_OUTPUT`)
  - Gate triggers when crossing axes (`GATE_X_OUTPUT`, `GATE_Y_OUTPUT`)
  - V/Oct (voltage per octave) outputs derived from position (`VOCT_X_OUTPUT`, `VOCT_Y_OUTPUT`)

- **Controls:**
  - Knobs for:
    - Speed
    - Step size
    - Smoothness
    - Number of balls
  - CV inputs for each control
  - Reset button and input

- **Configuration Options via Context Menu:**
  - Voct X/Y range settings (0–1, –1 to 1, etc.)
  - Gate duration options (from 0.5 ms to 200 ms)

---

### 🖼️ **Visual Widget (`RandomWalkWidget`) and Subwidgets**

#### Components:
1. **`RWDisplayWidget`:**
   - Displays the arena with:
     - Metal rail background
     - Playfield grid
     - Walker paths (trail)
     - Flashing reflections on walls
     - Gate trigger flashes
     - Momentum ticks and halos
     - Scanlines and glass reflection effect
     - Readouts for number of walkers, speed, step size, smoothness

2. **`RWBarsWidget`:**
   - Shows position bars for each walker (X and Y).
   - Bars show voltage values (bipolar).
   - Gate flashes appear on zero line when triggered.

3. **Module Panel Layout:**
   - Knobs and CV jacks on the left side.
   - Outputs arranged in a right block.
   - Context menu allows configuration of ranges and gate durations.

---

### 🔧 **Key Constants & Definitions**

```cpp
const int RW_N = 8;                    // Max number of walkers
const int RW_TRAIL_N = 100;            // Trail length per walker
const float RW_SCALE = 12.f;           // Scale factor for drawing positions
const float RW_MARGIN = 16.f;          // Margin around playfield
```

These define how many balls, trail length, scaling, and layout work.

---

### 🎨 **Visual Styling**

- Uses `nvg` (NanoVG) for vector graphics rendering.
- Custom color scheme with:
  - Accent colors (`colAccent()`)
  - Dim text (`colDim()`)
  - Ball-specific coloring via `ballColor(i, ...)`
- Gradient backgrounds and shadows for depth
- Flash effects for reflections and gate triggers

---

### 📦 **Integration into VCV Rack**

The module is registered using:
```cpp
Model *modelRandomWalk = createModel<RandomWalk, RandomWalkWidget>("RandomWalk");
```

This makes it available in the plugin menu under "RandomWalk".

---

### ✅ Summary

You've created a visually rich and functionally robust **random walk simulation** module for VCV Rack. It simulates multiple independent walkers moving randomly within a confined space, with:
- Real-time visualization of paths
- Interactive controls (knobs + CV)
- Polyphonic outputs (X/Y position, gates, V/Oct)
- Configurable physics and display parameters

It's suitable for use in generative music systems, experimental sound design, or as part of a larger modular patch.

Let me know if you'd like help:
- Adding new features
- Optimizing performance
- Porting to another framework
- Documenting the code further