<!--
AGMM AUTOMATIC DOCUMENTATION
Source: CellAutoGate.cpp
Generated: 2026-11-04 06:14
-->

# CellAutoGate

## 1. Overview

The `CellAutoGate` module implements a **one-dimensional cellular automaton** with gate output functionality, designed for VCV Rack. It simulates the evolution of a binary cellular automaton where each cell's state is determined by its immediate neighbors according to a specified rule.

Key features:
- Configurable **rule set** (0–255) using a 3-bit neighborhood lookup
- Adjustable **speed** or clock rate in Hz
- **Polyphonic gate outputs** (1–16 channels)
- External **clock input**
- **Randomize** and **Reset** controls
- Visual display of the automaton history with CRT-style effects

## Module Screenshot

![CellAutoGate VCV Rack Module](images/cellautogate.png)

## 2. Architecture

### Main Classes

- `CellAutoGate`: Core module class inheriting from `rack::Module`
- `PanelBackground`: Static panel artwork widget
- `CellularDisplayWidget`: Dynamic visualization of cellular automaton state
- `TCyanLight<TBase>`: Custom light widget for cyan LEDs
- `CellAutoGateWidget`: Module UI widget that composes the panel

### Inheritance

- `CellAutoGate` inherits from `rack::Module`
- `CellularDisplayWidget` inherits from `rack::TransparentWidget`
- `PanelBackground` inherits from `TransparentWidget`
- `TCyanLight<TBase>` is a template specialization of `GrayModuleLightWidget`

### Structures

- `std::array<uint64_t, ROWS>`: Ring buffer for storing automaton history
- `uint64_t currentRow`: Current state of the automaton (bitfield)
- `int ruleset[8]`: Lookup table for rule interpretation (8 entries)

### Relationships

- `CellAutoGateWidget` owns and manages:
  - `PanelBackground`
  - `CellularDisplayWidget`
  - All controls (`Knobs`, `Buttons`, `Ports`, `Lights`)
- `CellularDisplayWidget` accesses `CellAutoGate` state for rendering
- `CellAutoGate` maintains internal state and processes audio samples

## 3. DSP Theory

This module implements a **cellular automaton**, specifically a **1D binary cellular automaton** with **von Neumann neighborhood** (3-cell window: left, center, right).

### Mathematical Foundation

The evolution of the automaton is governed by a **rule table**:
- Each cell's next state depends on its current state and that of its immediate neighbors
- The rule is encoded as an 8-bit number (0–255)
- For each possible 3-bit neighborhood pattern, there is a corresponding output bit

Let $ N_i(t) $ be the state of cell $ i $ at time $ t $, where $ N_i \in \{0,1\} $

The next state is computed as:
$$
N_i(t+1) = R\left( N_{i-1}(t), N_i(t), N_{i+1}(t) \right)
$$

Where $ R $ is a lookup function defined by the rule number.

### Cellular Automaton Rule Encoding

The rule number $ r $ defines the transition function:
$$
R(l, c, r) = \text{ruleset}[l \cdot 4 + c \cdot 2 + r]
$$

Where:
- $ l $: left neighbor state (0 or 1)
- $ c $: center cell state (0 or 1)
- $ r $: right neighbor state (0 or 1)

The index is calculated as a binary number:
$$
\text{index} = l \cdot 4 + c \cdot 2 + r
$$

### Signal Processing

No analog signal processing occurs. The module operates purely in the digital domain, updating its internal state on each sample tick or clock event.

## 4. Parameters and Controls

### Knobs

| Parameter      | Min | Max | Default | Description |
|----------------|-----|-----|---------|-------------|
| `RULE_PARAM`   | 0   | 255 | 30      | Automaton rule number (0–255) |
| `SPEED_PARAM`  | 1   | 160 | 10      | Clock rate in Hz when no external clock is connected |
| `POLY_PARAM`   | 1   | 16  | 1       | Number of polyphonic output channels |

### Buttons

| Button         | Description |
|----------------|-------------|
| `RANDOM_PARAM` | Randomizes the initial state of the automaton |
| `RESET_PARAM`  | Resets the automaton to its initial state (center cell on) |

### Switches / Inputs

| Input          | Description |
|----------------|-------------|
| `CLOCK_INPUT`  | External clock signal; triggers automaton step when connected |

### Outputs

| Output         | Description |
|----------------|-------------|
| `GATE_OUTPUT`  | Polyphonic gate output (10V = on, 0V = off) |

### Enumerations

- `ParamId`: Rule, Speed, Polyphony, Random, Reset
- `InputId`: Clock input
- `OutputId`: Gate output
- `LightId`: Gate lights (16 total)

## 5. Inputs and Outputs

### Input

| Name            | Type        | Voltage Range | Description |
|-----------------|-------------|---------------|-------------|
| `CLOCK_INPUT`   | Audio        | 0–10V         | External clock signal; triggers automaton step when connected |

### Output

| Name             | Type        | Voltage Range | Description |
|------------------|-------------|---------------|-------------|
| `GATE_OUTPUT`    | Polyphonic  | 0–10V         | Gate outputs, one per channel; 10V = active, 0V = inactive |

## 6. Signal Processing Pipeline

1. **Initialization**
   - Module initializes with rule 30 and center cell on
   - History buffer is cleared

2. **Parameter Updates**
   - Rule parameter updates ruleset lookup table
   - Polyphony parameter adjusts output channel count

3. **Event Handling**
   - `RANDOM_PARAM` button triggers random state initialization
   - `RESET_PARAM` button resets to initial state
   - `CLOCK_INPUT` triggers automaton step if connected

4. **Timing Control**
   - If external clock is connected:
     - Step on rising edge of clock signal
   - Else:
     - Step at fixed interval determined by `SPEED_PARAM`

5. **Automaton Step**
   - For each cell in current row:
     - Extract left, center, right neighbor states
     - Compute index from neighborhood pattern
     - Lookup next state using ruleset
   - Update current row with new state
   - Push updated row to history buffer

6. **Output Generation**
   - Distribute gate outputs across polyphonic channels
   - Each channel's output is determined by a specific cell position in the current row
   - Cell position is calculated using a linear distribution across columns

## 7. Important Functions

### `updateRuleset(int rule)`
Converts rule number into lookup table:
```cpp
for (int i = 0; i < 8; ++i)
    ruleset[i] = (rule >> i) & 1;
```

### `stepAutomaton()`
Performs one step of cellular automaton evolution:
- Iterates through all columns
- Extracts left, center, right neighbors
- Computes index from neighborhood pattern
- Looks up next state in ruleset
- Sets new row state

### `process(const ProcessArgs &args)`
Main processing function:
- Handles parameter changes
- Manages timing and triggering
- Updates output gates
- Controls light indicators

### `randomize()`
Generates random initial state:
```cpp
uint64_t r = 0;
for (int i = 0; i < COLS; ++i)
    if (random::uniform() > 0.5f)
        r |= (1ULL << i);
currentRow = r;
```

### `restart()`
Resets automaton to initial state:
```cpp
currentRow = 1ULL << (COLS / 2);
histHead = histCount = 0;
pushHistory(currentRow);
```

## 8. Mathematical Implementation

### Cellular Automaton Transition Function

The core transition function is implemented as:

$$
\text{next}[i] = \text{ruleset}\left[ \left( \text{current}[i-1] \cdot 4 \right) + \left( \text{current}[i] \cdot 2 \right) + \text{current}[i+1] \right]
$$

Where:
- Indexing wraps around using modulo arithmetic
- `ruleset` is a lookup table of 8 bits derived from the rule number

### Bit Manipulation

The implementation uses bit manipulation for efficiency:

```cpp
int left   = (currentRow >> ((i + COLS - 1) % COLS)) & 1;
int center = (currentRow >> i) & 1;
int right  = (currentRow >> ((i + 1) % COLS)) & 1;
int index  = (left << 2) | (center << 1) | right;
next |= (uint64_t) ruleset[index] << i;
```

### History Buffer

A ring buffer stores the last `ROWS` automaton states:
```cpp
std::array<uint64_t, ROWS> history{};
int histHead = 0;
int histCount = 0;
```

## 9. Performance and Optimization

### CPU Usage Considerations

- The module performs a fixed number of operations per sample (COLS * 8)
- Bit manipulation is efficient
- No floating-point arithmetic in core loop

### Memory Allocation

- Fixed-size arrays: `ruleset[8]`, `history[ROWS]`
- No dynamic allocation during processing
- Stack-based variables for temporary calculations

### Buffers

- History buffer (`std::array<uint64_t, ROWS>`) is preallocated
- Ring buffer management with head/tail indices

### Real-time Processing

- All operations are deterministic and bounded
- No blocking or time-consuming operations
- Uses `dsp::SchmittTrigger` for reliable clock detection

### Thread Safety

- History buffer access is read-only from UI thread (torn reads are cosmetic)
- No shared mutable state between audio and UI threads during rendering

### Numerical Stability

- Integer bit operations ensure deterministic behavior
- No floating-point accumulation or iteration errors

## 10. Dependencies

### Libraries/Headers

- `rack.hpp`: VCV Rack core API
- `<array>`: Standard C++ containers
- `<cmath>`, `<cstdint>`, `<cstdio>`, `<algorithm>`, `<cstring>`: Standard C++ utilities
- `dsp::SchmittTrigger`: Rack DSP utility for reliable trigger detection

### External Classes

- `Module`: Base VCV Rack module class
- `TransparentWidget`: Base widget for overlay rendering
- `Vec`, `NVGcontext`, `DrawArgs`: VCV Rack graphics and geometry classes
- `random::uniform()`: Random number generator from Rack

## 11. Summary

`CellAutoGate` is a VCV Rack module implementing a one-dimensional binary cellular automaton with gate output functionality. It simulates the evolution of a 64-column, 64-row automaton using standard Wolfram rules (0–255). The module supports external clock triggering, adjustable speed, polyphonic outputs, and visual feedback through an animated display showing the automaton's history.

The implementation uses bit manipulation for efficient state transitions, a ring buffer for history storage, and deterministic timing control. It provides both manual controls (randomize, reset) and automatic operation modes (clocked or timed). The visual representation includes a CRT-style display with scanlines and rule lookup glyphs for educational purposes.