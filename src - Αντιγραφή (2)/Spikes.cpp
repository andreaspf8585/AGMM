#include "GitLoad.hpp"
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include "Knob_arcs.hpp"

// ============================================================================
// Spikes
// ============================================================================
//
// OVERVIEW
// --------
// Spikes is a 16 HP chaotic modulation source that combines two classical
// strange attractors (Lorenz and Rössler) with a fractal-correlated noise
// generator. The two attractors are integrated in real time with RK4, blended
// together per-sample by the BLEND knob, then optionally crossfaded toward a
// 6-octave fractional Brownian motion noise (FRACTAL knob) before being scaled
// and written to X / Y / Z / MIX outputs.
//
// Controls:
//   SPEED         Attractor integration rate (with 1 V/oct CV)
//   BLEND         Lorenz ↔ Rössler crossfade
//   FRACTAL       Amount of fractal noise mixed in
//   FRC SPEED     Base frequency of the fractal noise
//   SCALE         Output voltage scaling (0.1..10 V)
//   ROTATE        Auto-rotation speed of the 3D display
//
// A 200×126 display shows a rotating 3D view of the recent trajectory of the
// blended attractor. The view can be dragged to rotate by hand and
// double-clicked to reset. Header/footer rows report speed, blend, fractal
// amount and the current X/Y/Z/MIX voltages.
//
// File structure:
//   1. Layout constants (16 HP = 240×380 px).
//   2. Spikes module: attractor states, RNG, fractal octaves, DSP helpers
//      (Lorenz / Rössler derivative, RK4, computeFractal), and process().
//   3. Anonymous-namespace UI: blockHeader helper, SPPanel (static art) and
//      SPDisplay (rotating 3D trail view).
//   4. SpikesWidget: places knobs, CV inputs and outputs.
//   5. Model registration.
// ============================================================================

// ------------------------------------------------------------
// Layout constants
// ------------------------------------------------------------
static const int SP_TRAIL = 400;                // points kept in the 3D trail
static const float SP_W = RACK_GRID_WIDTH * 16; // 240
static const float SP_H = RACK_GRID_HEIGHT;     // 380

// 3D display (200 x 126): header row, framed view, footer row
static const float SP_DX = 20.f, SP_DY = 30.f, SP_DW = 200.f, SP_DH = 126.f;
static const float SP_VX = 4.f, SP_VY = 17.f, SP_VW = 192.f, SP_VH = 92.f; // view frame

// Knob grid: 3 columns of 66 px wide cells (label at +10, knob at +32)
static const float SP_KCOL[3] = {40.f, 120.f, 200.f};
static const float SP_CELL_W = 66.f;
static const float SP_KTOP[2] = {170.f, 230.f};
static const float SP_KCELL_H = 54.f;

// Jack row: three CV inputs, a gap, four outputs (cells 30 x 50, jack at +30)
static const float SP_IOHDR = 296.f;
static const float SP_IOTOP = 306.f;
static const float SP_IOCELL_W = 30.f, SP_IOCELL_H = 50.f;
// Returns the horizontal centre of jack i (0..2 = CV inputs, 3..6 = outputs).
static float spJx(int i) { return i < 3 ? 20.f + 31.f * i : 128.f + 31.f * (i - 3); }

// ------------------------------------------------------------
// Module
// ------------------------------------------------------------
// Chaotic modulation source combining Lorenz and Rössler attractors with
// fractal-correlated noise, plus a rotatable 3D trail view.
struct Spikes : Module
{
    // New items are appended at the end so old patches keep loading.
    enum ParamId
    {
        SPEED_PARAM,
        BLEND_PARAM,
        FRACTAL_PARAM,
        FRACTAL_SPEED_PARAM,
        SCALE_PARAM,
        ROTATE_PARAM, // view only: auto-rotation speed of the 3D display
        PARAMS_LEN
    };
    enum InputId
    {
        SPEED_INPUT,
        BLEND_INPUT,
        FRACTAL_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        X_OUTPUT,
        Y_OUTPUT,
        Z_OUTPUT,
        MIX_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    // ------------------------------------------------------------------
    // Chaotic attractor states
    // ------------------------------------------------------------------
    double lorenzX = 0.1, lorenzY = 0.0, lorenzZ = 0.0;
    double rosslerX = 1.0, rosslerY = 1.0, rosslerZ = 0.0;

    // ------------------------------------------------------------------
    // Fractal correlated randomness (fractional Brownian motion)
    //   N octaves of smoothly-interpolated random values; amplitudes
    //   fall off as 2^(-H) per octave. H controls correlation length.
    // ------------------------------------------------------------------
    static const int NUM_OCTAVES = 6;
    static constexpr double HURST = 0.65;

    // One fractal octave: the current random value, the next target, and a
    // phase that smoothly advances toward the target.
    struct Octave
    {
        double current = 0.0;
        double target = 0.0;
        double phase = 0.0;
        bool initialized = false;
    };
    Octave octaves[NUM_OCTAVES];

    // xorshift32 RNG for cheap, high-quality uniform noise
    uint32_t rngState = 0x9E3779B9u;
    // Advances the xorshift32 generator and returns the next 32-bit value.
    inline uint32_t xorshift()
    {
        uint32_t x = rngState;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        rngState = x;
        return x;
    }
    // Returns a uniform double in [0, 1).
    inline double randUnipolar() { return (xorshift() & 0xFFFFFF) / 16777216.0; }
    // Returns a uniform double in [-1, 1).
    inline double randBipolar() { return randUnipolar() * 2.0 - 1.0; }

    // ------------------------------------------------------------------
    // Display data (written by the audio thread, read by the UI thread;
    // a torn read is only cosmetic)
    // ------------------------------------------------------------------
    float trailX[SP_TRAIL] = {};
    float trailY[SP_TRAIL] = {};
    float trailZ[SP_TRAIL] = {};
    int trailHead = 0;                                      // index of the newest point
    int trailCount = 0;                                     // valid points (grows until SP_TRAIL)
    double trailAcc = 0.0;                                  // attractor time since the last trail point
    float outX = 0.f, outY = 0.f, outZ = 0.f, outMix = 0.f; // volts
    float dispSpeed = 1.f, dispBlend = 0.5f, dispFractal = 0.2f;

    // Constructor: configures the six parameters (with sensible ranges and
    // units), the three CV inputs and the four outputs.
    Spikes()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(SPEED_PARAM, 0.01f, 10.f, 1.f, "Speed", "x");
        configParam(BLEND_PARAM, 0.f, 1.f, 0.5f, "Lorenz / Rössler Blend", "%", 0.f, 100.f);
        configParam(FRACTAL_PARAM, 0.f, 1.f, 0.2f, "Fractal Amount", "%", 0.f, 100.f);
        configParam(FRACTAL_SPEED_PARAM, 0.01f, 50.f, 1.f, "Fractal Speed", " Hz");
        configParam(SCALE_PARAM, 0.1f, 10.f, 5.f, "Scale", " V");
        configParam(ROTATE_PARAM, -1.f, 1.f, 0.25f, "Display rotation speed", "%", 0.f, 100.f);

        configInput(SPEED_INPUT, "Speed CV (1 V/oct)");
        configInput(BLEND_INPUT, "Blend CV (10 V = 100%)");
        configInput(FRACTAL_INPUT, "Fractal CV (10 V = 100%)");

        configOutput(X_OUTPUT, "X");
        configOutput(Y_OUTPUT, "Y");
        configOutput(Z_OUTPUT, "Z");
        configOutput(MIX_OUTPUT, "Mix");
    }

    // Rack reset hook: restores the attractor seeds, clears the fractal
    // octaves, and empties the trail buffer.
    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        lorenzX = 0.1;
        lorenzY = 0.0;
        lorenzZ = 0.0;
        rosslerX = 1.0;
        rosslerY = 1.0;
        rosslerZ = 0.0;
        for (int i = 0; i < NUM_OCTAVES; i++)
            octaves[i] = Octave();
        trailHead = 0;
        trailCount = 0;
        trailAcc = 0.0;
        for (int i = 0; i < SP_TRAIL; i++)
            trailX[i] = trailY[i] = trailZ[i] = 0.f;
    }

    // ------------------------------------------------------------------
    // Vector fields
    // ------------------------------------------------------------------
    // Writes the Lorenz derivative at state s into d.
    static void lorenzDeriv(const double s[3], double d[3])
    {
        const double sigma = 10.0, rho = 28.0, beta = 8.0 / 3.0;
        d[0] = sigma * (s[1] - s[0]);
        d[1] = s[0] * (rho - s[2]) - s[1];
        d[2] = s[0] * s[1] - beta * s[2];
    }
    // Writes the Rössler derivative at state s into d.
    static void rosslerDeriv(const double s[3], double d[3])
    {
        const double a = 0.2, b = 0.2, c = 5.7;
        d[0] = -s[1] - s[2];
        d[1] = s[0] + a * s[1];
        d[2] = b + s[2] * (s[0] - c);
    }

    // Classical RK4 step: advances state s by dt using the given derivative
    // function.
    static void rk4(double s[3], double dt, void (*deriv)(const double *, double *))
    {
        double k1[3], k2[3], k3[3], k4[3], t[3];
        deriv(s, k1);
        for (int i = 0; i < 3; i++)
            t[i] = s[i] + 0.5 * dt * k1[i];
        deriv(t, k2);
        for (int i = 0; i < 3; i++)
            t[i] = s[i] + 0.5 * dt * k2[i];
        deriv(t, k3);
        for (int i = 0; i < 3; i++)
            t[i] = s[i] + dt * k3[i];
        deriv(t, k4);
        for (int i = 0; i < 3; i++)
            s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    // ------------------------------------------------------------------
    // Fractal correlated noise: summed octaves of smoothed random values
    // ------------------------------------------------------------------
    // Advances the 6-octave fBm noise by one sample and returns a normalised
    // bipolar value in roughly [-1, 1]. Each octave uses smoothstep
    // interpolation between its current and target random values at a
    // frequency starting at baseFreq and doubling per octave; amplitudes
    // fall by 2^(-HURST) per octave (giving the classic 1/f^H spectrum).
    double computeFractal(double baseFreq, double sr)
    {
        double total = 0.0;
        double normSq = 0.0;
        double amp = 1.0;
        double freq = baseFreq;

        for (int i = 0; i < NUM_OCTAVES; i++)
        {
            Octave &o = octaves[i];
            if (!o.initialized)
            {
                o.current = randBipolar();
                o.target = randBipolar();
                o.phase = randUnipolar();
                o.initialized = true;
            }
            double inc = freq / sr;
            if (inc > 0.45)
                inc = 0.45; // keep well below Nyquist
            o.phase += inc;
            while (o.phase >= 1.0)
            {
                o.phase -= 1.0;
                o.current = o.target;
                o.target = randBipolar();
            }
            // smoothstep interpolation between current and target
            double t = o.phase;
            double sm = t * t * (3.0 - 2.0 * t);
            double v = o.current * (1.0 - sm) + o.target * sm;

            total += v * amp;
            normSq += amp * amp;
            amp *= std::pow(0.5, HURST);
            freq *= 2.0;
        }
        return total / std::sqrt(normSq);
    }

    // Main audio callback: reads parameters and CVs, integrates both
    // attractors (with RK4 substepping), computes the fractal noise, blends
    // and mixes the outputs, and samples the trajectory into the display
    // trail buffer.
    void process(const ProcessArgs &args) override
    {
        // ---------------- Parameters & CV ----------------
        float speed = params[SPEED_PARAM].getValue();
        if (inputs[SPEED_INPUT].isConnected())
        {
            float cv = clamp(inputs[SPEED_INPUT].getVoltage(), -5.f, 5.f);
            speed *= std::pow(2.f, cv);
        }
        speed = clamp(speed, 0.001f, 100.f);

        float blend = params[BLEND_PARAM].getValue();
        if (inputs[BLEND_INPUT].isConnected())
            blend = clamp(blend + inputs[BLEND_INPUT].getVoltage() * 0.1f, 0.f, 1.f);

        float fractalAmt = params[FRACTAL_PARAM].getValue();
        if (inputs[FRACTAL_INPUT].isConnected())
            fractalAmt = clamp(fractalAmt + inputs[FRACTAL_INPUT].getVoltage() * 0.1f, 0.f, 1.f);

        float fractalSpeed = params[FRACTAL_SPEED_PARAM].getValue();
        float scale = params[SCALE_PARAM].getValue();

        dispSpeed = speed;
        dispBlend = blend;
        dispFractal = fractalAmt;

        // ---------------- Integrate attractors ----------------
        // speed=1 → roughly one attractor orbit per second at 48 kHz
        double dt = speed / args.sampleRate;
        const int substeps = 2;
        double subdt = dt / substeps;
        for (int s = 0; s < substeps; s++)
        {
            double L[3] = {lorenzX, lorenzY, lorenzZ};
            rk4(L, subdt, lorenzDeriv);
            lorenzX = L[0];
            lorenzY = L[1];
            lorenzZ = L[2];

            double R[3] = {rosslerX, rosslerY, rosslerZ};
            rk4(R, subdt, rosslerDeriv);
            rosslerX = R[0];
            rosslerY = R[1];
            rosslerZ = R[2];
        }

        // Safety net: re-seed an attractor if it ever leaves the finite range
        if (!std::isfinite(lorenzX + lorenzY + lorenzZ))
        {
            lorenzX = 0.1;
            lorenzY = 0.0;
            lorenzZ = 0.0;
        }
        if (!std::isfinite(rosslerX + rosslerY + rosslerZ))
        {
            rosslerX = 1.0;
            rosslerY = 1.0;
            rosslerZ = 0.0;
        }

        // ---------------- Fractal correlated noise ----------------
        double fractal = computeFractal(fractalSpeed, args.sampleRate);

        // ---------------- Normalize attractor outputs to ~[-1, 1] -----
        double lx = lorenzX / 20.0;
        double ly = lorenzY / 25.0;
        double lz = (lorenzZ - 25.0) / 25.0;

        double rx = rosslerX / 12.0;
        double ry = rosslerY / 12.0;
        double rz = (rosslerZ - 5.0) / 15.0; // Rössler z is spiky

        // ---------------- Blend Lorenz ↔ Rössler ----------------
        double b = blend;
        double mx = (1.0 - b) * lx + b * rx;
        double my = (1.0 - b) * ly + b * ry;
        double mz = (1.0 - b) * lz + b * rz;

        // ---------------- Mix in fractal correlated randomness ----------------
        double f = fractalAmt;
        mx = mx * (1.0 - f) + fractal * f;
        my = my * (1.0 - f) + fractal * f;
        mz = mz * (1.0 - f) + fractal * f;

        // ---------------- Mix bus ----------------
        double mix = (mx + my + mz) / 3.0;

        // ---------------- Output ----------------
        float vx = clamp((float)(mx * scale), -10.f, 10.f);
        float vy = clamp((float)(my * scale), -10.f, 10.f);
        float vz = clamp((float)(mz * scale), -10.f, 10.f);
        float vmix = clamp((float)(mix * scale), -10.f, 10.f);
        outputs[X_OUTPUT].setVoltage(vx);
        outputs[Y_OUTPUT].setVoltage(vy);
        outputs[Z_OUTPUT].setVoltage(vz);
        outputs[MIX_OUTPUT].setVoltage(vmix);

        outX = vx;
        outY = vy;
        outZ = vz;
        outMix = vmix;

        // ---------------- 3D trail for the display ----------------
        // One point per 0.015 units of attractor time, so the trail shape is
        // independent of the speed setting.
        trailAcc += dt;
        if (trailAcc >= 0.015)
        {
            trailAcc = std::fmod(trailAcc, 0.015);
            int h = (trailHead + 1) % SP_TRAIL;
            trailX[h] = clamp((float)mx, -1.6f, 1.6f);
            trailY[h] = clamp((float)my, -1.6f, 1.6f);
            trailZ[h] = clamp((float)mz, -1.6f, 1.6f);
            trailHead = h;
            if (trailCount < SP_TRAIL)
                trailCount++;
        }
    }
};

// Anonymous namespace: UI helpers and the two widgets.
namespace
{
    namespace ag = agmm;

    // Block title with hairlines that stop a measured distance from the text
    // Draws a centred block header whose hairlines span from x1 to x2,
    // stopping a measured distance from the text.
    void blockHeader(NVGcontext *vg, float x1, float x2, float y, const char *txt, NVGcolor col)
    {
        const float cx = (x1 + x2) / 2.f;
        const float gap = ag::textWidth(vg, txt, 8.f, 2.f) / 2.f + 6.f;
        ag::text(vg, cx, y, txt, 8.f, col, ag::ALIGN_C, 2.f);
        nvgBeginPath(vg);
        nvgMoveTo(vg, x1, y);
        nvgLineTo(vg, cx - gap, y);
        nvgMoveTo(vg, cx + gap, y);
        nvgLineTo(vg, x2, y);
        nvgStrokeColor(vg, ag::withA(col, 0.3f));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);
    }

    // ------------------------------------------------------------
    // Panel background: title, display frame, one labelled plate per control
    // ------------------------------------------------------------
    // Static artwork: base panel, display bezel, six labelled knob cells,
    // the CV IN / OUTPUT block headers and their seven jack cells (outputs
    // tinted).
    struct SPPanel : TransparentWidget
    {
        SPPanel() { box.size = Vec(SP_W, SP_H); }

        // Draws all static panel art including section headers and cell
        // plates.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, SP_W, SP_H, "SPIKES");
            ag::bezel(vg, SP_DX, SP_DY, SP_DW, SP_DH);

            const char *names[6] = {"SPEED", "BLEND", "FRACTAL", "FRC SPEED", "SCALE", "ROTATE"};
            for (int i = 0; i < 6; ++i)
                ag::cell(vg, SP_KCOL[i % 3], SP_KTOP[i / 3], SP_CELL_W, SP_KCELL_H, names[i]);

            // jack row: 3 CV inputs | 4 outputs
            blockHeader(vg, 6.f, 98.f, SP_IOHDR, "CV IN", ag::accent());
            blockHeader(vg, 112.f, 236.f, SP_IOHDR, "OUTPUT", ag::violet());
            const char *io[7] = {"SPEED", "BLEND", "FRACT", "X", "Y", "Z", "MIX"};
            for (int i = 0; i < 7; ++i)
                ag::cell(vg, spJx(i), SP_IOTOP, SP_IOCELL_W, SP_IOCELL_H, io[i], i >= 3, 7.f);

            Widget::draw(args);
        }
    };

    // Display: rotating 3D view of the attractor trail.
    // Drag to rotate by hand, double-click to reset the view.
    // Live display: renders the recent trajectory of the blended attractor
    // from the module's trail buffer, projected and rotated into a 3D view
    // with a reference cube, axis markers and voltage readouts. Uses a
    // synthetic Lorenz preview trail in the module browser.
    struct SPDisplay : TransparentWidget
    {
        Spikes *module = nullptr;

        float yaw = 0.6f, pitch = 0.5f;
        double lastT = 0.0;

        // demo trail for the module browser
        float demoX[SP_TRAIL], demoY[SP_TRAIL], demoZ[SP_TRAIL];

        // Constructor: positions the display widget, and (in the browser)
        // precomputes a Lorenz preview trail.
        SPDisplay(Spikes *m) : module(m)
        {
            box.pos = Vec(SP_DX, SP_DY);
            box.size = Vec(SP_DW, SP_DH);

            for (int i = 0; i < SP_TRAIL; ++i)
                demoX[i] = demoY[i] = demoZ[i] = 0.f;
            if (!module)
            {
                double s[3] = {0.1, 0.0, 0.0};
                for (int i = 0; i < 2000; ++i)
                    Spikes::rk4(s, 0.005, Spikes::lorenzDeriv);
                for (int i = 0; i < SP_TRAIL; ++i)
                {
                    for (int k = 0; k < 3; ++k)
                        Spikes::rk4(s, 0.005, Spikes::lorenzDeriv);
                    demoX[i] = (float)(s[0] / 20.0);
                    demoY[i] = (float)(s[1] / 25.0);
                    demoZ[i] = (float)((s[2] - 25.0) / 25.0);
                }
            }
        }

        // Left press is consumed so the subsequent drag moves the view.
        void onButton(const ButtonEvent &e) override
        {
            if (e.button == GLFW_MOUSE_BUTTON_LEFT && e.action == GLFW_PRESS)
                e.consume(this);
        }
        // Dragging rotates the view: horizontal for yaw, vertical for pitch
        // (with pitch clamped to avoid flipping).
        void onDragMove(const DragMoveEvent &e) override
        {
            yaw += e.mouseDelta.x * 0.012f;
            pitch = clamp(pitch + e.mouseDelta.y * 0.012f, -1.45f, 1.45f);
        }
        // Double-click resets the yaw/pitch to their defaults.
        void onDoubleClick(const DoubleClickEvent &e) override
        {
            yaw = 0.6f;
            pitch = 0.5f;
        }

        // Static layer: fills the screen, draws header/footer hairlines, and
        // the framed 3D view area.
        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, SP_DW, SP_DH);

            // hairlines under the header row and above the footer row
            nvgBeginPath(vg);
            nvgMoveTo(vg, 6.f, 13.5f);
            nvgLineTo(vg, SP_DW - 6.f, 13.5f);
            nvgMoveTo(vg, 6.f, SP_DH - 13.5f);
            nvgLineTo(vg, SP_DW - 6.f, SP_DH - 13.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            ag::plotFrame(vg, SP_VX, SP_VY, SP_VW, SP_VH);
            Widget::draw(args);
        }

        // Layer-1 overlay: advances the auto-rotation, projects the trail
        // into 3D, draws the reference cube, axes, trail (with an aged
        // gradient from violet to cyan), head marker, and the header/footer
        // readouts.
        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;
            const float W = SP_DW, H = SP_DH;

            // ---- gather data ----
            static float ox[SP_TRAIL], oy[SP_TRAIL], oz[SP_TRAIL]; // ordered oldest -> newest
            int count = SP_TRAIL;
            float rot = 0.25f;
            float spd = 1.f, bld = 0.5f, frc = 0.2f;
            float vx = 0.f, vy = 0.f, vz = 0.f, vm = 0.f;

            if (module)
            {
                int head = rack::math::clamp(module->trailHead, 0, SP_TRAIL - 1);
                count = rack::math::clamp(module->trailCount, 0, SP_TRAIL);
                for (int i = 0; i < count; ++i)
                {
                    int idx = ((head - (count - 1) + i) % SP_TRAIL + SP_TRAIL) % SP_TRAIL;
                    ox[i] = module->trailX[idx];
                    oy[i] = module->trailY[idx];
                    oz[i] = module->trailZ[idx];
                }
                rot = module->params[Spikes::ROTATE_PARAM].getValue();
                spd = module->dispSpeed;
                bld = module->dispBlend;
                frc = module->dispFractal;
                vx = module->outX;
                vy = module->outY;
                vz = module->outZ;
                vm = module->outMix;
            }
            else
            {
                for (int i = 0; i < SP_TRAIL; ++i)
                {
                    ox[i] = demoX[i];
                    oy[i] = demoY[i];
                    oz[i] = demoZ[i];
                }
                vx = 3.1f;
                vy = -1.4f;
                vz = 0.6f;
                vm = 0.8f;
            }

            // ---- advance the view rotation ----
            double now = system::getTime();
            float dt = (lastT > 0.0) ? clamp((float)(now - lastT), 0.f, 0.1f) : 0.f;
            lastT = now;
            yaw += dt * rot * 1.6f;
            if (yaw > 2.f * M_PI)
                yaw -= 2.f * M_PI;
            if (yaw < -2.f * M_PI)
                yaw += 2.f * M_PI;

            // ---- projection (view frame is 192 x 92, centred) ----
            const float cx = SP_VX + SP_VW / 2.f, cy = SP_VY + SP_VH / 2.f, S = 30.f;
            const float cyw = std::cos(yaw), syw = std::sin(yaw);
            const float cp = std::cos(pitch), sp = std::sin(pitch);

            // attractor (x, y, z) -> world (x, -z up, y depth), yaw about the vertical axis, then pitch
            // Projects an attractor-space point (x, y, z) into display
            // coordinates (px, py) and a perspective scale factor k.
            auto proj = [&](float x, float y, float z, float &px, float &py, float &k)
            {
                float wx = x, wy = -z, wz = y;
                float x1 = wx * cyw + wz * syw;
                float z1 = -wx * syw + wz * cyw;
                float y2 = wy * cp - z1 * sp;
                float z2 = wy * sp + z1 * cp;
                k = 1.f / (1.f + z2 * 0.22f);
                px = cx + x1 * S * k;
                py = cy + y2 * S * k;
            };

            nvgSave(vg);
            nvgScissor(vg, SP_VX + 1.f, SP_VY + 1.f, SP_VW - 2.f, SP_VH - 2.f);
            nvgLineCap(vg, NVG_ROUND);
            nvgLineJoin(vg, NVG_ROUND);

            // ---- reference cube ----
            {
                float cxs[8], cys[8], ck;
                for (int i = 0; i < 8; ++i)
                {
                    float x = (i & 1) ? 1.f : -1.f;
                    float y = (i & 2) ? 1.f : -1.f;
                    float z = (i & 4) ? 1.f : -1.f;
                    proj(x, y, z, cxs[i], cys[i], ck);
                }
                nvgBeginPath(vg);
                for (int i = 0; i < 8; ++i)
                {
                    for (int bit = 1; bit <= 4; bit <<= 1)
                    {
                        int j = i ^ bit;
                        if (j > i)
                        {
                            nvgMoveTo(vg, cxs[i], cys[i]);
                            nvgLineTo(vg, cxs[j], cys[j]);
                        }
                    }
                }
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 26));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);

                // floor grid (z = -1 plane, the bottom of the cube)
                nvgBeginPath(vg);
                for (int g = -1; g <= 1; ++g)
                {
                    float ax, ay, bx, by, k;
                    proj((float)g, -1.f, -1.f, ax, ay, k);
                    proj((float)g, 1.f, -1.f, bx, by, k);
                    nvgMoveTo(vg, ax, ay);
                    nvgLineTo(vg, bx, by);
                    proj(-1.f, (float)g, -1.f, ax, ay, k);
                    proj(1.f, (float)g, -1.f, bx, by, k);
                    nvgMoveTo(vg, ax, ay);
                    nvgLineTo(vg, bx, by);
                }
                nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 14));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
            }

            // ---- axes (labels are kept inside the view frame) ----
            {
                const float ax[3][3] = {{1.3f, 0.f, 0.f}, {0.f, 1.3f, 0.f}, {0.f, 0.f, 1.3f}};
                const char *an[3] = {"X", "Y", "Z"};
                NVGcolor ac[3] = {ag::accent(), ag::violet(), nvgRGB(0xe6, 0xe9, 0xef)};
                float ox0, oy0, k0;
                proj(0.f, 0.f, 0.f, ox0, oy0, k0);
                for (int a = 0; a < 3; ++a)
                {
                    float px, py, k;
                    proj(ax[a][0], ax[a][1], ax[a][2], px, py, k);
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, ox0, oy0);
                    nvgLineTo(vg, px, py);
                    nvgStrokeColor(vg, ag::withA(ac[a], 0.35f));
                    nvgStrokeWidth(vg, 1.f);
                    nvgStroke(vg);
                    float lx = clamp(px, SP_VX + 6.f, SP_VX + SP_VW - 6.f);
                    float ly = clamp(py, SP_VY + 6.f, SP_VY + SP_VH - 6.f);
                    ag::text(vg, lx, ly, an[a], 7.f, ag::withA(ac[a], 0.9f), ag::ALIGN_C, 0.f);
                }
            }

            // ---- trail (flat segments, fading with age) ----
            if (count >= 2)
            {
                float px0, py0, k0;
                proj(ox[0], oy[0], oz[0], px0, py0, k0);
                NVGcolor cOld = nvgRGBA(0xb0, 0x5a, 0xff, 255);
                NVGcolor cNew = nvgRGBA(0x8a, 0xee, 0xff, 255);
                for (int i = 1; i < count; ++i)
                {
                    float px1, py1, k1;
                    proj(ox[i], oy[i], oz[i], px1, py1, k1);
                    float t = (float)i / (float)(count - 1); // 0 old .. 1 new
                    float a = std::pow(t, 1.6f);
                    NVGcolor c = nvgLerpRGBA(cOld, cNew, t);

                    nvgBeginPath(vg);
                    nvgMoveTo(vg, px0, py0);
                    nvgLineTo(vg, px1, py1);
                    nvgStrokeColor(vg, ag::withA(c, 0.08f + a * 0.92f));
                    nvgStrokeWidth(vg, (0.5f + 1.1f * t) * k1);
                    nvgStroke(vg);

                    px0 = px1;
                    py0 = py1;
                }

                // head marker: ring and dot
                nvgBeginPath(vg);
                nvgCircle(vg, px0, py0, 4.f);
                nvgStrokeColor(vg, nvgRGBA(0x8a, 0xee, 0xff, 160));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, px0, py0, 1.8f);
                nvgFillColor(vg, nvgRGB(255, 255, 255));
                nvgFill(vg);
            }

            nvgRestore(vg);

            // ---- readouts: header and footer rows, outside the view frame ----
            char buf[64];
            NVGcolor c1 = ag::withA(ag::accent(), 0.85f);
            NVGcolor cd = ag::dim();

            snprintf(buf, sizeof(buf), "SPD %.2fX", spd);
            ag::text(vg, 6.f, 7.f, buf, 7.f, c1, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "L/R %3.0f%%", bld * 100.f);
            ag::text(vg, W / 2.f, 7.f, buf, 7.f, c1, ag::ALIGN_C, 0.5f);
            snprintf(buf, sizeof(buf), "FRC %3.0f%%", frc * 100.f);
            ag::text(vg, W - 6.f, 7.f, buf, 7.f, c1, ag::ALIGN_R, 0.5f);

            snprintf(buf, sizeof(buf), "X%+.1f", vx);
            ag::text(vg, 6.f, H - 6.f, buf, 7.f, cd, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "Y%+.1f", vy);
            ag::text(vg, 50.f, H - 6.f, buf, 7.f, cd, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "Z%+.1f", vz);
            ag::text(vg, 94.f, H - 6.f, buf, 7.f, cd, ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "MIX %+.1fV", vm);
            ag::text(vg, W - 6.f, H - 6.f, buf, 7.f, ag::withA(ag::violet(), 0.95f), ag::ALIGN_R, 0.5f);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ------------------------------------------------------------
// Module widget
// ------------------------------------------------------------
// Places the static panel, screws, live 3D display, the six knobs, three CV
// inputs and four outputs.
struct SpikesWidget : ModuleWidget
{
    // Builds the widget tree: static panel, screws, 3D display, the 3×2 knob
    // grid, and the CV input / output jack row.
    SpikesWidget(Spikes *module)
    {
        setModule(module);
        box.size = Vec(SP_W, SP_H);

        addChild(new SPPanel());
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addChild(new SPDisplay(module));

        // ---------------- Param knobs (3 x 2 grid, knob at +32 inside its cell) ----------------
        const int ids[6] = {Spikes::SPEED_PARAM, Spikes::BLEND_PARAM, Spikes::FRACTAL_PARAM,
                            Spikes::FRACTAL_SPEED_PARAM, Spikes::SCALE_PARAM, Spikes::ROTATE_PARAM};
        for (int i = 0; i < 6; ++i)
            addParam(createParamCentered<RoundSmallBlackKnob>(Vec(SP_KCOL[i % 3], SP_KTOP[i / 3] + 32.f), module, ids[i]));

        agmm::addKnobArcs(this, 2.f);
            // ---------------- jack row (jack at +30 inside its cell) ----------------
        const float jy = SP_IOTOP + 30.f;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(spJx(0), jy), module, Spikes::SPEED_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(spJx(1), jy), module, Spikes::BLEND_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(spJx(2), jy), module, Spikes::FRACTAL_INPUT));

        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(spJx(3), jy), module, Spikes::X_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(spJx(4), jy), module, Spikes::Y_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(spJx(5), jy), module, Spikes::Z_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(spJx(6), jy), module, Spikes::MIX_OUTPUT));
    }

    void appendContextMenu(Menu *menu) override
    {
        agmm::addGitLink(menu); // adds "Visit my git repo!"
        // ... your other menu items ...
    }
};

Model *modelSpikes = createModel<Spikes, SpikesWidget>("Spikes");