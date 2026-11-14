// ============================================================================
// Plot — polyphonic oscilloscope / signal visualizer (24 HP Rack module)
// ----------------------------------------------------------------------------
// Overview:
//   * Two independent polyphonic inputs (A and B), up to 16 channels each, are
//     recorded into separate ring buffers. Two polyphonic thru outputs mirror
//     the inputs (channel count preserved), so the module can be inserted
//     inline in a signal chain.
//   * A real oscilloscope trigger engine (Free / Auto / Normal, rising or
//     falling slope, internal or external source) with Schmitt hysteresis and
//     sub-sample interpolation locks the trace to the waveform instead of
//     letting it drift or jitter.
//   * The display draws a 10×8 division grid with voltage and time labels,
//     the traces of both inputs (focus channel highlighted), the trigger level
//     marker, the trigger position marker, and per-window statistics for the
//     focus channel of each input (Vpp, Max, Min, RMS, Avg, frequency).
//   * The audio thread publishes the description of the window to show as a
//     single packed 64-bit word together with a generation counter, so the UI
//     thread always copies a complete, consistent snapshot. The UI then draws
//     from its own cache, immune to the audio thread wrapping the ring buffer.
//   * Controls: 6 knobs (Scale, Time, Offset, Width, Level, Focus) and 6
//     click-to-cycle selectors (View, Res, Display, Trigger, Source, Slope).
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

// ------------------------------------------------------------
// Layout constants (24 HP = 360 px wide)
// ------------------------------------------------------------
static const float PLOT_W = RACK_GRID_WIDTH * 24;   // 360
static const float PLOT_H = RACK_GRID_HEIGHT;       // 380

// Display bezel on the panel
static const float DISP_X = 10.f, DISP_Y = 38.f;
static const float DISP_W = PLOT_W - 20.f;          // 340
static const float DISP_H = 206.f;

// Plot rectangle, in display-local coordinates
static const float PX0 = 40.f, PY0 = 22.f;
static const float PW  = DISP_W - PX0 - 10.f;       // 290
static const float PH  = 130.f;
static const float PX1 = PX0 + PW;
static const float PY1 = PY0 + PH;

static const int V_DIVS = 8;     // vertical grid divisions  (full height = 2 * scale)
static const int H_DIVS = 10;    // horizontal grid divisions (full width = time/screen)

// Sample buffer (power of two so we can wrap with a mask)
static const int MAX_CHANNELS = 16;
static const int BUFFER_SIZE  = 1 << 16;
static const int BUFFER_MASK  = BUFFER_SIZE - 1;
// Longest window we allow. Keeping it below the buffer size leaves slack so the
// UI thread can safely read a latched window while the audio thread keeps writing.
static const int MAX_WINDOW   = BUFFER_SIZE * 3 / 4;

// ------------------------------------------------------------
// Small helpers
// ------------------------------------------------------------

// Format a time value with an appropriate unit (s / ms / us).
static std::string fmtTime(float t) {
    char b[32];
    float a = std::fabs(t);
    if (a < 1e-7f)       snprintf(b, sizeof(b), "0");
    else if (a >= 1.f)   snprintf(b, sizeof(b), "%.2f s", t);
    else if (a >= 1e-3f) snprintf(b, sizeof(b), "%.1f ms", t * 1e3f);
    else                 snprintf(b, sizeof(b), "%.0f us", t * 1e6f);
    return b;
}

// Format a volts-per-division value (mV/div or V/div).
static std::string fmtVoltsPerDiv(float v) {
    char b[32];
    if (v < 1.f) snprintf(b, sizeof(b), "%.0f mV/div", v * 1000.f);
    else         snprintf(b, sizeof(b), "%.2f V/div", v);
    return b;
}

// Palette of shared display colors.
static NVGcolor gridMinor() { return nvgRGBA(120, 140, 160, 26); }
static NVGcolor gridMajor() { return nvgRGBA(150, 170, 190, 52); }
static NVGcolor zeroLine()  { return nvgRGBA(180, 200, 220, 95); }
static NVGcolor okGreen()   { return nvgRGBA(120, 230, 140, 255); }
static NVGcolor warnAmber() { return nvgRGBA(255, 190, 80, 255); }
static NVGcolor alertRed()  { return nvgRGBA(255, 95, 95, 255); }
static NVGcolor extWhite()  { return nvgRGBA(225, 232, 240, 255); }

// Result of a window statistics pass (min/max/mean/RMS/frequency).
struct Stats {
    bool  valid = false;
    float vmin = 0.f, vmax = 0.f, mean = 0.f, rms = 0.f;
    bool  hasFreq = false;
    float freq = 0.f;
};

// Statistics over a window of a ring buffer (all samples, no decimation).
static Stats computeStats(const float* buf, int start, int len, float sr) {
    Stats s;
    if (len < 2) return s;
    double sum = 0.0, sum2 = 0.0;
    float mn = 1e30f, mx = -1e30f;
    for (int i = 0; i < len; ++i) {
        float v = buf[(start + i) & BUFFER_MASK];
        sum  += v;
        sum2 += (double)v * (double)v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    s.valid = true;
    s.vmin = mn;
    s.vmax = mx;
    s.mean = (float)(sum / len);
    s.rms  = (float)std::sqrt(sum2 / len);

    // Frequency from rising crossings of the mid-level, with hysteresis
    float vpp = mx - mn;
    if (vpp > 0.02f) {
        float mid = 0.5f * (mx + mn);
        float h   = 0.1f * vpp;
        bool low = false;
        int first = -1, last = -1, count = 0;
        for (int i = 0; i < len; ++i) {
            float v = buf[(start + i) & BUFFER_MASK];
            if (v < mid - h) {
                low = true;
            }
            else if (low && v >= mid + h) {
                low = false;
                if (first < 0) first = i;
                last = i;
                count++;
            }
        }
        if (count >= 2 && last > first) {
            s.freq = (float)(count - 1) * sr / (float)(last - first);
            s.hasFreq = true;
        }
    }
    return s;
}

// The audio thread describes the window to show as ONE packed 64-bit word, so the
// UI can never see a half-updated (start, length, pre-trigger) combination.
struct DispSnap { int start = 0, len = 0, pre = 0; float shift = 0.f; };

// Pack (start, len, pre, sub-sample shift) into a single 64-bit word.
static uint64_t packDisp(int start, int len, int pre, float shift) {
    uint64_t sq = (uint64_t)rack::math::clamp((int)(shift * 256.f), 0, 255);
    return (uint64_t)(start & 0xFFFF)
         | ((uint64_t)(len   & 0xFFFF) << 16)
         | ((uint64_t)(pre   & 0xFFFF) << 32)
         | (sq << 48);
}
// Inverse of packDisp().
static DispSnap unpackDisp(uint64_t p) {
    DispSnap d;
    d.start = (int)(p & 0xFFFF);
    d.len   = (int)((p >> 16) & 0xFFFF);
    d.pre   = (int)((p >> 32) & 0xFFFF);
    d.shift = (float)((p >> 48) & 0xFF) / 256.f;   // sub-sample trigger offset
    return d;
}

// ------------------------------------------------------------
// Module — DSP + state.
//
// Records both polyphonic inputs into per-channel ring buffers, runs the
// trigger engine, publishes the display window, and passes the inputs through
// to the outputs. UI-facing state is published through atomics (disp, dispId)
// and cached plain fields (channels, focus, level, mode, ...).
// ------------------------------------------------------------
struct Plot : Module {
    enum ParamId {
        PARAM_SCALE,     // vertical: volts at top/bottom edge
        PARAM_TIME,      // stored as log2(seconds)
        PARAM_OFFSET,    // vertical offset
        PARAM_LEVEL,     // trigger level
        PARAM_CHAN,      // focus channel (trigger + readouts)
        PARAM_MODE,      // 0 free, 1 auto, 2 normal
        PARAM_SRC,       // 0 A, 1 B
        PARAM_SLOPE,     // 0 rising, 1 falling
        PARAM_VIEW,      // 0 A+B, 1 A, 2 B
        PARAM_HOLD,      // 0 run, 1 hold
        PARAM_WIDTH,     // trace line width (px)
        PARAM_RES,       // plot resolution: 0 = 1x, 1 = 2x, 2 = 4x columns per pixel
        PARAMS_LEN
    };
    enum InputId {
        INPUT_A,
        INPUT_B,
        INPUT_TRIG,
        INPUTS_LEN
    };
    enum OutputId {
        OUTPUT_A,
        OUTPUT_B,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHT_TRIG,
        LIGHTS_LEN
    };

    // Two independent ring buffers, up to 16 poly channels each
    std::vector<float> wave[2][MAX_CHANNELS];
    int writePos = 0;
    int channels[2] = {0, 0};       // actual connected channel counts

    // State shared with the UI (what to draw)
    std::atomic<uint64_t> disp{0};      // packed window description
    std::atomic<uint32_t> dispId{0};    // bumps on every publish
    float dispSr    = 48000.f;
    bool  trigActive = false;

    // Cached control values (refreshed every few samples)
    int   win = 4800, pre = 1200;
    int   mode = 1, src = 0, slope = 0, focus = 0;
    float level = 0.f;
    bool  hold = false;
    dsp::ClockDivider ctrlDiv;

    // Trigger state machine
    bool armed = false, capturing = false;
    int  trigIdx = 0, capWin = 0, capPre = 0, postRemaining = 0;
    float capShift = 0.f;           // sub-sample trigger position of the pending capture
    float prevX = 0.f;              // previous trigger-domain sample (for interpolation)
    float hyst = 0.05f;             // trigger hysteresis, follows vertical scale
    int  sinceTrig = 1 << 20;

    // --------------------------------------------------------
    // Constructor — config params, inputs, outputs, buffers.
    // --------------------------------------------------------
    Plot() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(PARAM_SCALE,  0.5f, 10.f, 2.f, "Vertical scale (top/bottom edge)", " V");
        // Time is stored as log2(seconds): 1 ms ... 1 s, shown in seconds
        configParam(PARAM_TIME, -10.f, 0.f, -3.3219f, "Time per screen", " s", 2.f, 1.f);
        configParam(PARAM_OFFSET, -10.f, 10.f, 0.f, "Vertical offset", " V");
        configParam(PARAM_LEVEL,  -10.f, 10.f, 0.f, "Trigger level", " V");
        configParam(PARAM_CHAN,   1.f, 16.f, 1.f, "Focus channel (trigger & readouts)");
        paramQuantities[PARAM_CHAN]->snapEnabled = true;

        configSwitch(PARAM_MODE,  0.f, 2.f, 1.f, "Trigger mode",
                     {"Free-run", "Auto", "Normal"});
        configSwitch(PARAM_SRC,   0.f, 1.f, 0.f, "Trigger source", {"Input A", "Input B"});
        configSwitch(PARAM_SLOPE, 0.f, 1.f, 0.f, "Trigger slope", {"Rising", "Falling"});
        configSwitch(PARAM_VIEW,  0.f, 2.f, 0.f, "Traces shown", {"A + B", "A only", "B only"});
        configSwitch(PARAM_HOLD,  0.f, 1.f, 0.f, "Display", {"Run", "Hold"});
        configParam(PARAM_WIDTH, 0.5f, 4.f, 1.3f, "Trace width", " px");
        configSwitch(PARAM_RES,  0.f, 2.f, 1.f, "Plot resolution",
                     {"Standard (1x)", "High (2x)", "Ultra (4x)"});

        configInput(INPUT_A, "Input A (polyphonic)");
        configInput(INPUT_B, "Input B (polyphonic)");
        configInput(INPUT_TRIG, "External trigger (overrides source)");
        configOutput(OUTPUT_A, "Output A (polyphonic thru)");
        configOutput(OUTPUT_B, "Output B (polyphonic thru)");

        for (int k = 0; k < 2; ++k)
            for (int c = 0; c < MAX_CHANNELS; ++c)
                wave[k][c].assign(BUFFER_SIZE, 0.f);

        ctrlDiv.setDivision(16);
    }

    // --------------------------------------------------------
    // Reset — clear all buffers and invalidate the published window.
    // --------------------------------------------------------
    void onReset() override {
        for (int k = 0; k < 2; ++k)
            for (int c = 0; c < MAX_CHANNELS; ++c)
                std::fill(wave[k][c].begin(), wave[k][c].end(), 0.f);
        disp.store(0, std::memory_order_relaxed);
        dispId.fetch_add(1, std::memory_order_release);
        capturing = false;
    }

    // --------------------------------------------------------
    // Publish the window to show, atomically (one 64-bit word +
    // a generation counter so the UI can detect new frames).
    // --------------------------------------------------------
    void publish(int start, int len, int pre, float shift) {
        disp.store(packDisp(start, len, pre, shift), std::memory_order_relaxed);
        dispId.fetch_add(1, std::memory_order_release);
    }

    // --------------------------------------------------------
    // Refresh cached control values (called every few samples
    // through the ctrlDiv divider to keep the hot loop cheap).
    // --------------------------------------------------------
    void updateControls(float sr) {
        float t = std::exp2(params[PARAM_TIME].getValue());
        win   = rack::math::clamp((int)(t * sr), 32, MAX_WINDOW);
        pre   = win / 4;
        mode  = (int)std::round(params[PARAM_MODE].getValue());
        src   = (int)std::round(params[PARAM_SRC].getValue());
        slope = (int)std::round(params[PARAM_SLOPE].getValue());
        focus = (int)std::round(params[PARAM_CHAN].getValue()) - 1;
        level = params[PARAM_LEVEL].getValue();
        hold  = params[PARAM_HOLD].getValue() > 0.5f;
        // Hysteresis scales with the display so noisy signals do not double-trigger
        hyst  = std::max(0.05f, 0.02f * params[PARAM_SCALE].getValue());
        dispSr = sr;
    }

    // --------------------------------------------------------
    // Per-sample DSP:
    //   * pass-through to the outputs (always, even on HOLD)
    //   * record both inputs into the ring buffers
    //   * run the trigger state machine
    //   * publish the window to show
    // --------------------------------------------------------
    void process(const ProcessArgs& args) override {
        if (ctrlDiv.process())
            updateControls(args.sampleRate);

        // --- Pass-through (always, even when the display is held) ------
        for (int k = 0; k < 2; ++k) {
            int n = inputs[INPUT_A + k].getChannels();
            outputs[OUTPUT_A + k].setChannels(n);
            for (int ch = 0; ch < n; ++ch)
                outputs[OUTPUT_A + k].setVoltage(inputs[INPUT_A + k].getVoltage(ch), ch);
        }

        if (hold) {
            lights[LIGHT_TRIG].setBrightnessSmooth(0.f, args.sampleTime);
            return;
        }

        // --- Record --------------------------------------------------------
        for (int k = 0; k < 2; ++k) {
            int n = rack::math::clamp(inputs[INPUT_A + k].getChannels(), 0, MAX_CHANNELS);
            // Newly appearing channels start from silence, not stale history
            for (int c = channels[k]; c < n; ++c)
                std::fill(wave[k][c].begin(), wave[k][c].end(), 0.f);
            channels[k] = n;
            for (int ch = 0; ch < n; ++ch)
                wave[k][ch][writePos] = inputs[INPUT_A + k].getVoltage(ch);
        }
        const int writeIdx = writePos;
        writePos = (writePos + 1) & BUFFER_MASK;
        if (sinceTrig < (1 << 29)) sinceTrig++;

        // --- Trigger detection (Schmitt) -----------------------------------
        float tv;
        if (inputs[INPUT_TRIG].isConnected()) {
            tv = inputs[INPUT_TRIG].getVoltage();
        }
        else {
            int ch = std::min(focus, std::max(channels[src] - 1, 0));
            tv = inputs[INPUT_A + src].getVoltage(ch);
        }
        float x   = slope ? -tv : tv;
        float lvl = slope ? -level : level;
        bool edge = false;
        float frac = 1.f;               // where between prev (0) and current (1) sample we crossed
        if (x < lvl - hyst) {
            armed = true;
        }
        else if (armed && x >= lvl) {
            edge = true;
            armed = false;
            float d = x - prevX;
            frac = d > 1e-9f ? rack::math::clamp((lvl - prevX) / d, 0.f, 1.f) : 1.f;
        }
        prevX = x;

        // --- Decide which window the display shows -------------------------
        const int timeout = std::max(2 * win, (int)(0.25f * args.sampleRate));

        if (mode == 0) {
            // Free-run: continuously show the most recent window
            publish((writePos - win) & BUFFER_MASK, win, 0, 0.f);
            capturing = false;
        }
        else {
            if (capturing) {
                if (--postRemaining <= 0) {
                    publish((trigIdx - capPre) & BUFFER_MASK, capWin, capPre, capShift);
                    capturing = false;
                    sinceTrig = 0;
                }
            }
            else if (edge) {
                capturing     = true;
                trigIdx       = writeIdx;
                capWin        = win;
                capPre        = pre;
                capShift      = 1.f - frac;   // draw samples this far right so the crossing sits exactly on the trigger mark
                postRemaining = win - pre;
            }
            else if (mode == 1 && sinceTrig > timeout) {
                // Auto: no trigger for a while, fall back to free-run
                publish((writePos - win) & BUFFER_MASK, win, 0, 0.f);
            }
        }

        trigActive = (mode != 0) && (sinceTrig < timeout);
        lights[LIGHT_TRIG].setBrightnessSmooth(trigActive ? 1.f : 0.f, args.sampleTime);
    }
};

// ------------------------------------------------------------
// PlotDisplay — the oscilloscope screen widget:
//   * grid, time/voltage labels
//   * traces of both inputs (focus channel highlighted)
//   * trigger level + trigger position markers
//   * header (channel counts, focus, state, time/div, V/div)
//   * per-input measurement rows (Vpp/Max/Min/RMS/Avg/Freq)
//
// The audio thread keeps overwriting the ring buffer, so the UI copies each
// published window into a local cache and draws from that. This keeps the
// display stable even when the trigger is slow or the UI thread lags.
// ------------------------------------------------------------
struct PlotDisplay : TransparentWidget {
    Plot* module = nullptr;
    std::vector<float> demo[2];    // shown in the module browser

    // --------------------------------------------------------
    // Constructor: place the widget over the display area and
    // build the two synthetic waves used in the module browser.
    // --------------------------------------------------------
    PlotDisplay(Plot* m) : module(m) {
        box.pos  = Vec(DISP_X, DISP_Y);
        box.size = Vec(DISP_W, DISP_H);
        for (int k = 0; k < 2; ++k) demo[k].resize(1024);
        for (int i = 0; i < 1024; ++i) {
            float ph = (float)i / 1024.f * 2.f * 3.14159265f;
            demo[0][i] = 1.6f * std::sin(ph * 4.f);
            demo[1][i] = 1.0f * std::sin(ph * 8.f + 1.f) + 0.4f * std::sin(ph * 24.f);
        }
    }

    // UI-side copy of the displayed window. The audio thread keeps overwriting the
    // ring buffer, so a latched window is copied out once and drawn from here;
    // otherwise a slow trigger lets newer samples creep into the left edge.
    std::vector<float> cache[2][MAX_CHANNELS];
    Stats    cacheStats[2];
    int      cChans[2] = {0, 0};
    int      cLen = 0, cPre = 0, cFocus = -1;
    float    cShift = 0.f, cSr = 48000.f;
    uint32_t cId = 0xFFFFFFFFu;

    // --------------------------------------------------------
    // Copy the currently published window into the local cache
    // and (re)compute statistics for the focus channel when the
    // published window or the focus channel has changed.
    // --------------------------------------------------------
    void syncCache() {
        Plot* m = module;
        uint32_t id = m->dispId.load(std::memory_order_acquire);
        bool copied = (id != cId);
        if (copied) {
            DispSnap d = unpackDisp(m->disp.load(std::memory_order_relaxed));
            cId = id;
            cLen = d.len;
            cPre = d.pre;
            cShift = d.shift;
            cSr = m->dispSr;
            for (int k = 0; k < 2; ++k) {
                int n = rack::math::clamp(m->channels[k], 0, MAX_CHANNELS);
                cChans[k] = (cLen >= 2) ? n : 0;
                for (int ch = 0; ch < cChans[k]; ++ch) {
                    std::vector<float>& c = cache[k][ch];
                    if ((int)c.size() < MAX_WINDOW) c.resize(MAX_WINDOW);
                    const float* src = m->wave[k][ch].data();
                    int first = std::min(cLen, BUFFER_SIZE - d.start);
                    std::memcpy(c.data(), src + d.start, (size_t)first * sizeof(float));
                    if (first < cLen)
                        std::memcpy(c.data() + first, src, (size_t)(cLen - first) * sizeof(float));
                }
            }
        }
        if (copied || m->focus != cFocus) {
            cFocus = m->focus;
            for (int k = 0; k < 2; ++k) {
                cacheStats[k] = Stats();
                if (cLen >= 2 && cChans[k] > 0) {
                    int fch = std::min(cFocus, cChans[k] - 1);
                    cacheStats[k] = computeStats(cache[k][fch].data(), 0, cLen, cSr);
                }
            }
        }
    }

    // Draw one channel as a min/max envelope (so fast signals and narrow
    // pulses are never lost to decimation).
    void traceChannel(NVGcontext* vg, const float* buf, int mask, int start, int len,
                      float scale, float offset, NVGcolor col, bool focused,
                      float lineW, float resMul, float shiftSamples) {
        const float halfH = PH * 0.5f;
        const float yMid  = PY0 + halfH;
        auto Y = [&](float v) { return yMid - (v + offset) / scale * halfH; };

        const int cols = std::max(2, (int)(PW * resMul));
        const float xOff = (len > 1) ? shiftSamples / (float)(len - 1) * PW : 0.f;
        nvgBeginPath(vg);

        if (len <= cols * 2) {
            // Few enough samples: draw a plain polyline, one vertex per sample.
            for (int i = 0; i < len; ++i) {
                float x = PX0 + (float)i / (float)(len - 1) * PW + xOff;
                float y = Y(buf[(start + i) & mask]);
                if (i == 0) nvgMoveTo(vg, x, y);
                else        nvgLineTo(vg, x, y);
            }
        }
        else {
            // Heavy decimation: one vertical min/max segment per output column.
            float lastY = 0.f;
            for (int c = 0; c < cols; ++c) {
                int s0 = (int)((int64_t)c * len / cols);
                int s1 = (int)((int64_t)(c + 1) * len / cols);
                if (s1 <= s0) s1 = s0 + 1;
                float mn = 1e30f, mx = -1e30f;
                for (int s = s0; s < s1; ++s) {
                    float v = buf[(start + s) & mask];
                    if (v < mn) mn = v;
                    if (v > mx) mx = v;
                }
                float yHi = Y(mx), yLo = Y(mn);    // yHi <= yLo on screen
                float x = PX0 + (float)c / (float)(cols - 1) * PW + xOff;
                float y1, y2;
                if (c == 0 || std::fabs(lastY - yHi) <= std::fabs(lastY - yLo)) { y1 = yHi; y2 = yLo; }
                else                                                           { y1 = yLo; y2 = yHi; }
                if (c == 0) nvgMoveTo(vg, x, y1);
                else        nvgLineTo(vg, x, y1);
                nvgLineTo(vg, x, y2);
                lastY = y2;
            }
        }

        nvgLineJoin(vg, NVG_ROUND);
        if (focused) {
            // Glow under the focused trace for extra readability
            NVGcolor g = col; g.a *= 0.14f;
            nvgStrokeColor(vg, g);
            nvgStrokeWidth(vg, lineW * 2.6f + 1.f);
            nvgStroke(vg);
        }
        NVGcolor c = col;
        c.a *= focused ? 1.f : 0.40f;
        nvgStrokeColor(vg, c);
        nvgStrokeWidth(vg, focused ? lineW : std::max(0.6f, lineW * 0.7f));
        nvgStroke(vg);
    }

    // Draw the oscilloscope grid: vertical time lines, horizontal voltage
    // lines with labels on the left, minor ticks along the center lines.
    void drawGrid(NVGcontext* vg, float scale, float offset) {
        namespace ag = agmm;
        nvgStrokeWidth(vg, 1.f);

        // Vertical lines (time)
        for (int i = 1; i < H_DIVS; ++i) {
            float x = std::round(PX0 + PW * i / H_DIVS) + 0.5f;
            nvgBeginPath(vg);
            nvgMoveTo(vg, x, PY0);
            nvgLineTo(vg, x, PY1);
            nvgStrokeColor(vg, (i == H_DIVS / 2) ? gridMajor() : gridMinor());
            nvgStroke(vg);
        }
        // Horizontal lines (voltage) + labels
        char buf[24];
        for (int j = 0; j <= V_DIVS; ++j) {
            float y = std::round(PY0 + PH * j / V_DIVS) + 0.5f;
            bool zero = (j == V_DIVS / 2);
            if (j > 0 && j < V_DIVS) {
                nvgBeginPath(vg);
                nvgMoveTo(vg, PX0, y);
                nvgLineTo(vg, PX1, y);
                nvgStrokeColor(vg, zero ? zeroLine() : gridMinor());
                nvgStroke(vg);
            }
            // Axis label every other line. Line j is at normalized height n,
            // which equals volts v = n*scale - offset.
            if (j % 2 == 0) {
                float n = 1.f - (float)j / (V_DIVS / 2);
                float v = n * scale - offset;
                if (std::fabs(v) < 0.005f) v = 0.f;
                snprintf(buf, sizeof(buf), "%+.2f", v);
                ag::text(vg, PX0 - 4.f, y + 2.5f, buf, 6.5f,
                         zero ? ag::accent() : ag::dim(), ag::ALIGN_R, 0.f);
            }
        }
        // Minor ticks along the center lines (5 per division)
        nvgBeginPath(vg);
        float yc = std::round(PY0 + PH * 0.5f) + 0.5f;
        for (int i = 0; i < H_DIVS * 5; ++i) {
            float x = PX0 + PW * i / (H_DIVS * 5);
            nvgMoveTo(vg, x, yc - 2.f);
            nvgLineTo(vg, x, yc + 2.f);
        }
        float xc = std::round(PX0 + PW * 0.5f) + 0.5f;
        for (int j = 0; j < V_DIVS * 5; ++j) {
            float y = PY0 + PH * j / (V_DIVS * 5);
            nvgMoveTo(vg, xc - 2.f, y);
            nvgLineTo(vg, xc + 2.f, y);
        }
        nvgStrokeColor(vg, gridMajor());
        nvgStroke(vg);
    }

    // --------------------------------------------------------
    // Live drawing (layer 1). Renders the whole screen: grid,
    // traces, markers, header and measurement rows.
    // --------------------------------------------------------
    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;
        namespace ag = agmm;

        ag::screen(vg, 0.f, 0.f, DISP_W, DISP_H);
        ag::plotFrame(vg, PX0, PY0, PW, PH);

        NVGcolor colA = ag::accent();
        NVGcolor colB = ag::violet();
        NVGcolor cols[2] = {colA, colB};

        // ---- Snapshot of module state ---------------------------------
        if (module) syncCache();
        float scale  = module ? module->params[Plot::PARAM_SCALE ].getValue() : 2.f;
        float offset = module ? module->params[Plot::PARAM_OFFSET].getValue() : 0.f;
        int   mode   = module ? module->mode  : 1;
        int   view   = module ? (int)std::round(module->params[Plot::PARAM_VIEW].getValue()) : 0;
        bool  hold   = module ? module->hold  : false;
        float lineW  = module ? module->params[Plot::PARAM_WIDTH].getValue() : 1.3f;
        int   resIdx = module ? (int)std::round(module->params[Plot::PARAM_RES].getValue()) : 1;
        float resMul = (float)(1 << rack::math::clamp(resIdx, 0, 2));
        int   focus  = module ? module->focus : 0;
        float sr     = module ? cSr   : 48000.f;
        int   len    = module ? cLen  : 1024;
        int   pre    = module ? cPre  : 0;
        float shift  = module ? cShift : 0.f;
        bool  trigOk = module ? module->trigActive : false;
        bool  ext    = module ? module->inputs[Plot::INPUT_TRIG].isConnected() : false;
        int   chans[2] = {module ? cChans[0] : 1, module ? cChans[1] : 1};
        bool  haveData = len >= 2;
        float winTime = haveData ? (float)len / sr : 0.f;

        drawGrid(vg, scale, offset);

        // ---- Traces (clipped to the plot area) ---------------------------
        if (haveData) {
            nvgSave(vg);
            nvgScissor(vg, PX0, PY0, PW, PH);

            // Trigger level marker
            if (module && mode != 0) {
                float yMid = PY0 + PH * 0.5f;
                float yL = yMid - (module->level + offset) / scale * (PH * 0.5f);
                NVGcolor tc = ext ? extWhite() : cols[module->src];
                tc.a = 0.55f;
                nvgBeginPath(vg);
                for (float x = PX0; x < PX1; x += 8.f) {
                    nvgMoveTo(vg, x, yL);
                    nvgLineTo(vg, std::min(x + 3.f, PX1), yL);
                }
                nvgStrokeColor(vg, tc);
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                // Arrow at the right edge
                nvgBeginPath(vg);
                nvgMoveTo(vg, PX1 - 7.f, yL - 3.5f);
                nvgLineTo(vg, PX1,       yL);
                nvgLineTo(vg, PX1 - 7.f, yL + 3.5f);
                nvgClosePath(vg);
                nvgFillColor(vg, tc);
                nvgFill(vg);
            }

            // Draw each visible input, non-focus channels first, focus last
            for (int k = 0; k < 2; ++k) {
                if (view == 1 && k == 1) continue;
                if (view == 2 && k == 0) continue;
                if (chans[k] < 1) continue;
                if (module) {
                    int fch = std::min(focus, chans[k] - 1);
                    for (int ch = 0; ch < chans[k]; ++ch) {
                        if (ch == fch) continue;
                        traceChannel(vg, cache[k][ch].data(), BUFFER_MASK, 0, len,
                                     scale, offset, cols[k], false, lineW, resMul, shift);
                    }
                    traceChannel(vg, cache[k][fch].data(), BUFFER_MASK, 0, len,
                                 scale, offset, cols[k], true, lineW, resMul, shift);
                }
                else {
                    traceChannel(vg, demo[k].data(), 1023, 0, 1024,
                                 scale, offset, cols[k], true, lineW, resMul, 0.f);
                }
            }
            nvgRestore(vg);

            // Trigger position marker (only in a triggered window)
            if (pre > 0) {
                float xT = PX0 + PW * (float)pre / (float)len;
                nvgBeginPath(vg);
                nvgMoveTo(vg, xT, PY0);
                nvgLineTo(vg, xT, PY1);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 30));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgMoveTo(vg, xT - 3.5f, PY0 - 5.f);
                nvgLineTo(vg, xT + 3.5f, PY0 - 5.f);
                nvgLineTo(vg, xT,        PY0 + 0.5f);
                nvgClosePath(vg);
                nvgFillColor(vg, okGreen());
                nvgFill(vg);
            }
        }
        else {
            ag::text(vg, PX0 + PW * 0.5f, PY0 + PH * 0.5f + 3.f,
                     "WAITING FOR TRIGGER", 8.f, ag::dim(), ag::ALIGN_C, 1.f);
        }

        // ---- Time axis labels ---------------------------------------------
        float preTime = (haveData && pre > 0) ? (float)pre / sr : 0.f;
        for (int i = 0; i <= H_DIVS; i += 2) {
            float t = winTime * (float)i / (float)H_DIVS - preTime;
            ag::text(vg, PX0 + PW * i / H_DIVS, PY1 + 10.f, fmtTime(t).c_str(), 6.5f,
                     ag::dim(), i == 0 ? ag::ALIGN_L : (i == H_DIVS ? ag::ALIGN_R : ag::ALIGN_C), 0.f);
        }

        // ---- Header strip ----------------------------------------------------
        char buf[64];
        for (int k = 0; k < 2; ++k) {
            const char* nm = k == 0 ? "A" : "B";
            bool shown = (view == 0) || (view == 1 && k == 0) || (view == 2 && k == 1);
            if (chans[k] > 0) snprintf(buf, sizeof(buf), "%s  CH%d", nm, chans[k]);
            else              snprintf(buf, sizeof(buf), "%s  --", nm);
            NVGcolor c = (chans[k] > 0 && shown) ? cols[k] : ag::dim();
            ag::text(vg, 8.f + 50.f * k, 12.f, buf, 7.f, c, ag::ALIGN_L, 0.f);
        }
        snprintf(buf, sizeof(buf), "FOCUS %d", focus + 1);
        ag::text(vg, 108.f, 12.f, buf, 7.f, ag::dim(), ag::ALIGN_L, 0.f);

        // Trigger state (FREE / AUTO / WAIT / TRIG'D / HOLD, with EXT suffix)
        const char* stateTxt;
        NVGcolor stateCol;
        if (hold)              { stateTxt = "HOLD";   stateCol = alertRed(); }
        else if (mode == 0)    { stateTxt = "FREE";   stateCol = ag::dim(); }
        else if (trigOk)       { stateTxt = "TRIG'D"; stateCol = okGreen(); }
        else if (mode == 1)    { stateTxt = "AUTO";   stateCol = warnAmber(); }
        else                   { stateTxt = "WAIT";   stateCol = warnAmber(); }
        snprintf(buf, sizeof(buf), "%s%s", stateTxt, (ext && mode != 0 && !hold) ? " EXT" : "");
        ag::text(vg, 205.f, 12.f, buf, 7.f, stateCol, ag::ALIGN_C, 0.f);

        // Time/div and volts/div
        std::string tdiv = haveData ? fmtTime(winTime / H_DIVS) : std::string("--");
        snprintf(buf, sizeof(buf), "%s/div  %s", tdiv.c_str(),
                 fmtVoltsPerDiv(scale / (V_DIVS / 2)).c_str());
        ag::text(vg, DISP_W - 8.f, 12.f, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.f);

        // ---- Measurement rows (focus channel of each input) -------------
        for (int k = 0; k < 2; ++k) {
            float yRow = PY1 + 26.f + 14.f * k;
            NVGcolor c = cols[k];
            ag::text(vg, 8.f, yRow, k == 0 ? "A" : "B", 7.f, c, ag::ALIGN_L, 0.f);

            Stats s;
            if (module && haveData && chans[k] > 0) s = cacheStats[k];
            const char* caps[5] = {"Vpp", "Max", "Min", "RMS", "Avg"};
            float vals[5] = {s.vmax - s.vmin, s.vmax, s.vmin, s.rms, s.mean};
            const char* fmts[5] = {"%.2f", "%+.2f", "%+.2f", "%.2f", "%+.2f"};
            for (int i = 0; i < 5; ++i) {
                float x = 24.f + 56.f * i;
                ag::text(vg, x, yRow, caps[i], 6.5f, ag::dim(), ag::ALIGN_L, 0.f);
                if (s.valid) snprintf(buf, sizeof(buf), fmts[i], vals[i]);
                else         snprintf(buf, sizeof(buf), "--");
                ag::text(vg, x + 19.f, yRow, buf, 7.f, c, ag::ALIGN_L, 0.f);
            }
            // Frequency (Hz / kHz)
            if (s.hasFreq) {
                if (s.freq >= 1000.f) snprintf(buf, sizeof(buf), "%.2f kHz", s.freq / 1000.f);
                else                  snprintf(buf, sizeof(buf), "%.1f Hz", s.freq);
            }
            else snprintf(buf, sizeof(buf), "-- Hz");
            ag::text(vg, DISP_W - 8.f, yRow, buf, 7.f, c, ag::ALIGN_R, 0.f);
        }

        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// PlotPanel — static panel chrome:
//   * panel base with the module title
//   * display bezel
//   * divider between the input and output jack groups
// ------------------------------------------------------------
struct PlotPanel : TransparentWidget {
    PlotPanel() { box.size = Vec(PLOT_W, PLOT_H); }
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        agmm::panelBase(vg, PLOT_W, PLOT_H, "PLOT");
        agmm::bezel(vg, DISP_X, DISP_Y, DISP_W, DISP_H);

        // Divider between input and output jacks (clear of the AGMM mark)
        nvgStrokeWidth(vg, 1.f);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
        nvgBeginPath(vg);
        nvgMoveTo(vg, 233.f, 344.f);
        nvgLineTo(vg, 233.f, 372.f);
        nvgStroke(vg);

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// PlotLabel — small centered text label used under knobs,
// selectors and jacks.
// ------------------------------------------------------------
struct PlotLabel : TransparentWidget {
    std::string text;
    NVGcolor    color;
    PlotLabel(Vec center, const std::string& t, NVGcolor c = nvgRGBA(200, 210, 220, 220))
        : text(t), color(c) {
        box.size = Vec(56.f, 12.f);
        box.pos  = center.minus(box.size.div(2.f));
    }
    void draw(const DrawArgs& args) override {
        agmm::text(args.vg, box.size.x * 0.5f, box.size.y * 0.5f + 2.5f, text.c_str(),
                   7.f, color, agmm::ALIGN_C, 1.f);
        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// SelectorButton — click-to-cycle parameter widget.
// Renders the current state as a small rounded box with text.
// Left-click advances the state; right-click opens the standard
// parameter context menu (handled by ParamWidget).
// ------------------------------------------------------------
struct SelectorButton : app::ParamWidget {
    std::vector<std::string> labels;
    std::vector<NVGcolor>    colors;     // optional, one per state

    SelectorButton() { box.size = Vec(48.f, 14.f); }

    // Draw the rounded box and the current state label.
    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        nvgBeginPath(vg);
        nvgRoundedRect(vg, 0.5f, 0.5f, box.size.x - 1.f, box.size.y - 1.f, 3.f);
        nvgFillColor(vg, nvgRGBA(12, 16, 22, 255));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(75, 92, 108, 255));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        int idx = 0;
        engine::ParamQuantity* pq = getParamQuantity();
        if (pq) idx = (int)std::round(pq->getValue());
        if (labels.empty()) return;
        idx = rack::math::clamp(idx, 0, (int)labels.size() - 1);
        NVGcolor c = (idx < (int)colors.size()) ? colors[idx] : agmm::accent();
        agmm::text(vg, box.size.x * 0.5f, box.size.y * 0.5f + 2.5f,
                   labels[idx].c_str(), 7.f, c, agmm::ALIGN_C, 0.5f);
        Widget::draw(args);
    }

    // Left-click cycles the parameter to the next state (wrapping).
    void onButton(const ButtonEvent& e) override {
        if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            engine::ParamQuantity* pq = getParamQuantity();
            if (pq) {
                float v = std::round(pq->getValue()) + 1.f;
                if (v > pq->getMaxValue() + 0.01f) v = pq->getMinValue();
                pq->setValue(v);
            }
            e.consume(this);
            return;
        }
        ParamWidget::onButton(e);   // right-click menu, etc.
    }
};

// ------------------------------------------------------------
// PlotWidget — Rack widget.
//   * panel background + screws
//   * display (live oscilloscope)
//   * 6 knobs + 6 selectors in a shared 6-column grid
//   * 3 input jacks, 2 output jacks, plus the trigger LED
// ------------------------------------------------------------
struct PlotWidget : ModuleWidget {
    // Helper: create a SelectorButton centered at (x, y) bound to paramId,
    // with the given state labels and (optional) per-state colors.
    SelectorButton* addSelector(float x, float y, int paramId, Plot* module,
                                std::vector<std::string> labels,
                                std::vector<NVGcolor> colors = {}) {
        SelectorButton* b = createParamCentered<SelectorButton>(Vec(x, y), module, paramId);
        b->labels = labels;
        b->colors = colors;
        addParam(b);
        return b;
    }

    // --------------------------------------------------------
    // Constructor: build the module's UI.
    // --------------------------------------------------------
    PlotWidget(Plot* module) {
        setModule(module);
        box.size = Vec(PLOT_W, PLOT_H);

        addChild(new PlotPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new PlotDisplay(module));

        // Six-column grid shared by knobs, selectors and jacks.
        // Vertical bands (no overlaps): knobs 256-297, selectors 305-334, jacks 342-378.
        // The centre column of the jack row is left empty for the AGMM mark.
        const float cx[6] = {44.f, 98.f, 152.f, 206.f, 260.f, 314.f};
        const NVGcolor lbl  = nvgRGBA(200, 210, 220, 220);
        const NVGcolor dimL = nvgRGBA(150, 162, 175, 200);

        // --- Knobs -------------------------------------------------------
        const float knobY = 270.f;
        const int knobIds[6] = {Plot::PARAM_SCALE, Plot::PARAM_TIME, Plot::PARAM_OFFSET,
                                Plot::PARAM_WIDTH, Plot::PARAM_LEVEL, Plot::PARAM_CHAN};
        const char* knobNames[6] = {"SCALE", "TIME", "OFFSET", "WIDTH", "LEVEL", "FOCUS"};
        for (int i = 0; i < 6; ++i) {
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(cx[i], knobY), module, knobIds[i]));
            addChild(new PlotLabel(Vec(cx[i], knobY + 21.f), knobNames[i], i < 4 ? lbl : dimL));
        }

        // --- Selectors ----------------------------------------------------
        const float selY = 312.f;
        addSelector(cx[0], selY, Plot::PARAM_VIEW, module, {"A + B", "A ONLY", "B ONLY"},
                    {lbl, agmm::accent(), agmm::violet()});
        addSelector(cx[1], selY, Plot::PARAM_RES, module, {"STD", "HIGH", "ULTRA"},
                    {dimL, lbl, agmm::accent()});
        addSelector(cx[2], selY, Plot::PARAM_HOLD, module, {"RUN", "HOLD"},
                    {okGreen(), alertRed()});
        addSelector(cx[3], selY, Plot::PARAM_MODE, module, {"FREE", "AUTO", "NORMAL"},
                    {dimL, warnAmber(), okGreen()});
        addSelector(cx[4], selY, Plot::PARAM_SRC,  module, {"SRC  A", "SRC  B"},
                    {agmm::accent(), agmm::violet()});
        addSelector(cx[5], selY, Plot::PARAM_SLOPE, module, {"RISING", "FALLING"},
                    {lbl, lbl});
        const char* selNames[6] = {"VIEW", "RES", "DISPLAY", "TRIGGER", "SOURCE", "SLOPE"};
        for (int i = 0; i < 6; ++i)
            addChild(new PlotLabel(Vec(cx[i], selY + 16.f), selNames[i], dimL));

        // --- Jacks ---------------------------------------------------------
        const float jackY = 352.f;
        addInput (createInputCentered<DarkPJ301MPort>(Vec(cx[0], jackY), module, Plot::INPUT_A));
        addInput (createInputCentered<DarkPJ301MPort>(Vec(cx[1], jackY), module, Plot::INPUT_B));
        addInput (createInputCentered<DarkPJ301MPort>(Vec(cx[2], jackY), module, Plot::INPUT_TRIG));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(cx[4], jackY), module, Plot::OUTPUT_A));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(cx[5], jackY), module, Plot::OUTPUT_B));
        addChild(new PlotLabel(Vec(cx[0], jackY + 16.f), "IN A",    agmm::accent()));
        addChild(new PlotLabel(Vec(cx[1], jackY + 16.f), "IN B",    agmm::violet()));
        addChild(new PlotLabel(Vec(cx[2], jackY + 16.f), "TRIG IN", dimL));
        addChild(new PlotLabel(Vec(cx[4], jackY + 16.f), "OUT A",   agmm::accent()));
        addChild(new PlotLabel(Vec(cx[5], jackY + 16.f), "OUT B",   agmm::violet()));

        // --- Trigger LED (top right of panel) ---------------------------------
        addChild(createLightCentered<SmallLight<GreenLight>>(
            Vec(PLOT_W - 20.f, 22.f), module, Plot::LIGHT_TRIG));
        addChild(new PlotLabel(Vec(PLOT_W - 44.f, 22.f), "TRIG", dimL));
    }
};

Model* modelPlot = createModel<Plot, PlotWidget>("Plot");