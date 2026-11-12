// ============================================================================
// Galton — interactive Galton board / normal-distribution simulator (40 HP)
// ----------------------------------------------------------------------------
// A physically motivated triangle of pegs. Each ball makes `slots-1` random
// left/right choices (50/50) as it falls, so the landing slot follows
// Binomial(slots-1, 0.5) — a live demonstration of the CLT.
//
//   * SPEED (0.2x..3x) scales the fall rate and auto-spawn rate.
//   * SLOTS (4..16, snapped) sets the number of bins / LEDs / poly channels.
//   * DROP button + DROP IN trigger jack add a ball on demand.
//   * CLEAR empties the board and resets all statistics.
//   * GATES poly output: channel i = gate for slot i (fires on landing).
//   * One panel LED per slot flashes on each landing; the display also
//     shows a small bin indicator at the bottom of the peg board.
//   * Right-click → "Gate duration" (5 ms .. 1 s), saved with the patch.
//
// Left display  : peg board, falling balls, bin dividers, indicators.
// Right display : live histogram with expected-count ticks and a
//                 binomial normal-fit curve; statistics 2×2 grid below.
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <vector>

namespace ag = agmm;

// ------------------------------------------------------------
// Layout constants (40 HP = 600 px wide)
// ------------------------------------------------------------
static const float GL_W = RACK_GRID_WIDTH * 40;   // 600
static const float GL_H = RACK_GRID_HEIGHT;       // 380

static const int GL_N         = 16;   // max slots / LEDs (= PORT_MAX_CHANNELS)
static const int GL_MAX_BALLS = 48;   // concurrent balls

static const float GL_PI = 3.14159265358979f;

// Left display — peg board
static const float GL_LDX = 20.f, GL_LDY = 26.f, GL_LDW = 280.f, GL_LDH = 200.f;
// Right display — histogram + stats
static const float GL_RDX = 320.f, GL_RDY = 26.f, GL_RDW = 260.f, GL_RDH = 200.f;

// Section headers
static const float GL_LED_HDR  = 232.f;
static const float GL_CTRL_HDR = 262.f;

// LED row (16 panel LEDs)
static const float GL_LED_Y  = 246.f;
static const float GL_LED_X0 = 40.f;
static const float GL_LED_X1 = 560.f;

// Control cells: SPEED, SLOTS, DROP, CLEAR, TRIG (jack), GATES (jack)
static const float GL_CTRL_CELL_Y  = 272.f;
static const float GL_CTRL_CELL_H  =  50.f;
static const float GL_CTRL_CELL_W  =  80.f;
static const float GL_CTRL_Y       = GL_CTRL_CELL_Y + 32.f;   // 304
static const float GL_CTRL_X[6]    = {75.f, 165.f, 255.f, 345.f, 435.f, 525.f};

// ------------------------------------------------------------
// Gate-duration presets (right-click menu)
// ------------------------------------------------------------
static const int NUM_GATE_PRESETS = 10;
static const float GATE_PRESET_VALUES[NUM_GATE_PRESETS] = {
    0.005f, 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.25f, 0.5f, 0.75f, 1.0f
};
static const char* GATE_PRESET_NAMES[NUM_GATE_PRESETS] = {
    "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms", "250 ms", "500 ms", "750 ms", "1 s"
};

// ============================================================================
// Module
// ============================================================================
struct Galton : Module {
    enum ParamId { SPEED_PARAM, SLOTS_PARAM, DROP_PARAM, CLEAR_PARAM, PARAMS_LEN };
    enum InputId { DROP_INPUT, INPUTS_LEN };
    enum OutputId { GATES_OUTPUT, OUTPUTS_LEN };
    enum LightId  { ENUMS(SLOT_LIGHT, GL_N), LIGHTS_LEN };

    // --------------------------------------------------------
    // A ball, tracked as a sequence of kinematic segments:
    //   (x0,y0) -> (x1,y1) with progress t in [0,1].
    // `row` counts peg rows arrived at so far (0 before any bounce).
    // --------------------------------------------------------
    struct Ball {
        float x0, y0;
        float x1, y1;
        float t;
        int   row;
        bool  active;
    };
    Ball balls[GL_MAX_BALLS];

    // --------------------------------------------------------
    // Board geometry (rebuilt when slots change).
    // --------------------------------------------------------
    int   numRows    = 0;         // slots - 1
    float pegSpacing = 0.06f;     // 1 / numRows
    std::vector<float> rowY;      // y of each peg row (unit square)

    // --------------------------------------------------------
    // User state
    // --------------------------------------------------------
    int   slots        = 8;
    float speed        = 1.f;
    float gateDuration = 0.05f;
    int   gateIdx      = 3;

    float segmentTime = 0.08f;    // seconds per segment at 1x speed
    float spawnTimer  = 0.f;

    // --------------------------------------------------------
    // Gate / LED / histogram state
    // --------------------------------------------------------
    dsp::PulseGenerator gatePulses[GL_N];
    float ledFlash[GL_N]  = {};
    int   binCounts[GL_N] = {};
    bool  statsDirty      = true;
    float binomPMF[GL_N]  = {};   // cached Binomial(slots-1, 0.5) PMF

    dsp::SchmittTrigger dropBtnTrig, dropJackTrig, clearTrig;

    // --------------------------------------------------------
    // Display snapshots (written in process(), read by the UI).
    // --------------------------------------------------------
    float dispBallX[GL_MAX_BALLS] = {};
    float dispBallY[GL_MAX_BALLS] = {};
    int   dispBallCount = 0;
    int   dispSlots     = 8;
    float dispLed[GL_N]     = {};
    int   dispCounts[GL_N]  = {};

    // --------------------------------------------------------
    // Statistics (updated when `statsDirty`).
    // --------------------------------------------------------
    int   statBalls   = 0;
    float statMean    = 0.f;
    float statStdDev  = 0.f;
    float statChiSq   = 0.f;
    int   statDof     = 0;
    float statExpMean = 4.5f;
    float statExpStd  = 1.32f;

    // --------------------------------------------------------
    // Constructor
    // --------------------------------------------------------
    Galton() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(SPEED_PARAM, 0.2f, 3.f, 1.f, "Fall speed", "x");
        configParam(SLOTS_PARAM, 4.f, 16.f, 8.f, "Number of slots");
        paramQuantities[SLOTS_PARAM]->snapEnabled = true;
        configButton(DROP_PARAM,  "Drop ball");
        configButton(CLEAR_PARAM, "Clear");

        configInput(DROP_INPUT, "Drop trigger");
        configOutput(GATES_OUTPUT, "Slot gates (polyphonic)");

        for (int i = 0; i < GL_N; i++)
            configLight(SLOT_LIGHT + i, string::f("Slot %d", i + 1));

        // Set the gate output channel count once; it will be updated if
        // SLOTS changes. Assigning `channels` directly (rather than
        // setChannels(GL_N)) avoids the compiler's array-bounds
        // false-positive on the inlined zero-out loop at the 16-channel
        // boundary (PORT_MAX_CHANNELS).
        outputs[GATES_OUTPUT].channels = (uint8_t)slots;

        rebuildPegs();
        for (int i = 0; i < 2; i++) spawnBall();
    }

    // --------------------------------------------------------
    // Binomial PMF P(k | n, p).
    // --------------------------------------------------------
    static float binomialPMF(int k, int n, float p) {
        if (k < 0 || k > n) return 0.f;
        double c = 1.0;
        for (int i = 0; i < k; i++) c = c * (double)(n - i) / (double)(i + 1);
        return (float)(c * std::pow(p, k) * std::pow(1.f - p, n - k));
    }

    // --------------------------------------------------------
    // Rebuild peg layout for the current slot count.
    //   Row i has i+1 pegs at x = 0.5 + (k - i/2) * pegSpacing.
    // --------------------------------------------------------
    void rebuildPegs() {
        numRows    = std::max(1, slots - 1);
        pegSpacing = 1.f / (float)numRows;

        const float yTop = 0.15f, yBot = 0.75f;
        const float yStep = (numRows > 1) ? (yBot - yTop) / (numRows - 1) : 0.f;

        rowY.resize(numRows);
        for (int row = 0; row < numRows; row++)
            rowY[row] = yTop + yStep * (float)row;

        // Cache the Binomial(slots-1, 0.5) PMF for the histogram + stats.
        int trials = slots - 1;
        for (int i = 0; i < GL_N; i++) binomPMF[i] = 0.f;
        for (int i = 0; i < slots; i++) binomPMF[i] = binomialPMF(i, trials, 0.5f);

        statsDirty = true;
    }

    // --------------------------------------------------------
    // Spawn a ball at the top centre if a slot is free.
    // --------------------------------------------------------
    void spawnBall() {
        for (int i = 0; i < GL_MAX_BALLS; i++) {
            if (!balls[i].active) {
                Ball& b = balls[i];
                b.active = true;
                b.x0 = b.x1 = 0.5f;
                b.y0 = 0.05f;
                b.y1 = rowY[0];
                b.t   = 0.f;
                b.row = 0;
                return;
            }
        }
    }

    // --------------------------------------------------------
    // Advance one ball by dt.
    //
    //   * Integrates `t` by dt/(segmentTime/speed).
    //   * Each completed segment arrives at the target peg row
    //     (`row <= numRows`). The ball then bounces left or right
    //     (±pegSpacing/2) and heads to the next row. On the very
    //     last peg row it heads instead to the bin floor (0.9),
    //     with the post-bounce x — so exactly (slots-1) bounces
    //     happen before landing.
    //   * On landing the ball fires a gate, flashes the LED and
    //     increments the bin count; the process() loop then
    //     refreshes the statistics.
    // --------------------------------------------------------
    void advanceBall(Ball& b, float dt) {
        if (!b.active) return;

        const float segT = segmentTime / speed;
        b.t += dt / segT;

        while (b.t >= 1.f && b.active) {
            b.t -= 1.f;

            const float arriveX = b.x1;
            const float arriveY = b.y1;
            b.row++;

            if (b.row <= numRows) {
                // Arrived at peg row (row-1): bounce.
                float dir = (rack::random::uniform() < 0.5f) ? -1.f : 1.f;
                b.x0 = arriveX;
                b.y0 = arriveY;
                b.x1 = rack::math::clamp(arriveX + dir * pegSpacing * 0.5f, 0.01f, 0.99f);
                b.y1 = (b.row < numRows) ? rowY[b.row] : 0.9f;
            } else {
                // Reached the bin floor: land.
                int slot = (int)rack::math::clamp(b.x1 * (float)slots, 0.f, (float)(slots - 1));
                if (slot >= 0 && slot < GL_N) {
                    gatePulses[slot].trigger(gateDuration);
                    ledFlash[slot] = 1.f;
                    binCounts[slot]++;
                    statsDirty = true;
                }
                b.active = false;
            }
        }
    }

    // --------------------------------------------------------
    // Recompute summary statistics against the current counts:
    //   Balls, mean slot, population std dev, chi-squared vs
    //   the expected Binomial(slots-1, 0.5) counts.
    // --------------------------------------------------------
    void computeStats() {
        int n = 0;
        for (int i = 0; i < slots; i++) n += binCounts[i];
        statBalls  = n;
        statDof    = slots - 1;
        statExpMean = (float)(slots + 1) * 0.5f;
        statExpStd  = std::sqrt((float)(slots - 1) * 0.25f);

        if (n < 1) {
            statMean = statStdDev = statChiSq = 0.f;
            return;
        }

        float sum = 0.f;
        for (int i = 0; i < slots; i++) sum += (float)(i + 1) * binCounts[i];
        statMean = sum / (float)n;

        float varSum = 0.f;
        for (int i = 0; i < slots; i++) {
            float d = (float)(i + 1) - statMean;
            varSum += d * d * binCounts[i];
        }
        statStdDev = std::sqrt(varSum / (float)n);

        float chi = 0.f;
        for (int i = 0; i < slots; i++) {
            float expected = n * binomPMF[i];
            if (expected >= 1.f) {
                float d = (float)binCounts[i] - expected;
                chi += d * d / expected;
            }
        }
        statChiSq = chi;
    }

    // --------------------------------------------------------
    // Per-sample entry point.
    // --------------------------------------------------------
    void process(const ProcessArgs& args) override {
        const float dt = args.sampleTime;

        // --- params ---
        speed = params[SPEED_PARAM].getValue();
        int newSlots = rack::math::clamp((int)std::round(params[SLOTS_PARAM].getValue()), 4, GL_N);
        if (newSlots != slots) {
            slots = newSlots;
            rebuildPegs();
            for (int i = 0; i < GL_N; i++) binCounts[i] = 0;
            for (int i = 0; i < GL_MAX_BALLS; i++) balls[i].active = false;
        }

        // --- advance every ball ---
        for (int i = 0; i < GL_MAX_BALLS; i++) advanceBall(balls[i], dt);

        // --- manual triggers ---
        if (dropBtnTrig.process(params[DROP_PARAM].getValue(), 0.1f, 1.f) ||
            dropJackTrig.process(inputs[DROP_INPUT].getVoltage(), 0.1f, 2.f)) {
            spawnBall();
        }
        if (clearTrig.process(params[CLEAR_PARAM].getValue(), 0.1f, 1.f)) {
            for (int i = 0; i < GL_MAX_BALLS; i++) balls[i].active = false;
            for (int i = 0; i < GL_N; i++) binCounts[i] = 0;
            statsDirty = true;
        }

        // --- auto-spawn (~4 balls/s at 1x) ---
        spawnTimer += dt * speed;
        if (spawnTimer >= 0.25f) {
            spawnTimer -= 0.25f;
            spawnBall();
        }

        // --- gates + panel LEDs ---
        // Set the channel count directly and write the voltages ourselves.
        // Avoiding `setChannels(GL_N)` sidesteps a GCC array-bounds
        // false-positive on the inlined zero-out loop when the requested
        // count equals PORT_MAX_CHANNELS.
        auto& gatePort = outputs[GATES_OUTPUT];
        gatePort.channels = (uint8_t)slots;
        for (int i = 0; i < GL_N; i++) {
            bool g = gatePulses[i].process(dt);
            float v = (i < slots && g) ? 10.f : 0.f;
            gatePort.setVoltage(v, i);
            ledFlash[i] = std::max(0.f, ledFlash[i] - dt * 3.f);
            lights[SLOT_LIGHT + i].setBrightness(ledFlash[i]);
        }

        // --- refresh statistics (only when bins changed) ---
        if (statsDirty) {
            computeStats();
            statsDirty = false;
        }

        // --- snapshot for the UI ---
        dispBallCount = 0;
        for (int i = 0; i < GL_MAX_BALLS; i++) {
            if (balls[i].active) {
                Ball& b = balls[i];
                float tt = b.t;
                float yEase = tt * tt * 0.4f + tt * 0.6f;   // mild acceleration
                dispBallX[dispBallCount] = b.x0 + (b.x1 - b.x0) * tt;
                dispBallY[dispBallCount] = b.y0 + (b.y1 - b.y0) * yEase;
                dispBallCount++;
            }
        }
        dispSlots = slots;
        for (int i = 0; i < GL_N; i++) {
            dispLed[i]    = ledFlash[i];
            dispCounts[i] = binCounts[i];
        }
    }

    // --------------------------------------------------------
    // Persist / restore the gate preset index.
    // --------------------------------------------------------
    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "gateIdx", json_integer(gateIdx));
        return root;
    }
    void dataFromJson(json_t* root) override {
        json_t* j = json_object_get(root, "gateIdx");
        if (j) {
            gateIdx = rack::math::clamp((int)json_integer_value(j), 0, NUM_GATE_PRESETS - 1);
            gateDuration = GATE_PRESET_VALUES[gateIdx];
        }
    }

    void onReset(const ResetEvent& e) override {
        Module::onReset(e);
        for (int i = 0; i < GL_MAX_BALLS; i++) balls[i].active = false;
        for (int i = 0; i < GL_N; i++) { binCounts[i] = 0; ledFlash[i] = 0.f; }
        spawnTimer = 0.f;
        statsDirty = true;
    }
};

// ============================================================================
// UI
// ============================================================================
namespace {

// ------------------------------------------------------------
// GLPanel — static panel chrome.
// ------------------------------------------------------------
struct GLPanel : TransparentWidget {
    GLPanel() { box.size = Vec(GL_W, GL_H); }

    void draw(const DrawArgs& args) override {
        NVGcontext* vg = args.vg;
        ag::panelBase(vg, GL_W, GL_H, "GALTON");
        ag::bezel(vg, GL_LDX, GL_LDY, GL_LDW, GL_LDH);
        ag::bezel(vg, GL_RDX, GL_RDY, GL_RDW, GL_RDH);

        ag::sectionHeader(vg, GL_W, GL_LED_HDR,  "SLOT LEDS");
        ag::sectionHeader(vg, GL_W, GL_CTRL_HDR, "CONTROLS");

        ag::cell(vg, GL_CTRL_X[0], GL_CTRL_CELL_Y, GL_CTRL_CELL_W, GL_CTRL_CELL_H, "SPEED");
        ag::cell(vg, GL_CTRL_X[1], GL_CTRL_CELL_Y, GL_CTRL_CELL_W, GL_CTRL_CELL_H, "SLOTS");
        ag::cell(vg, GL_CTRL_X[2], GL_CTRL_CELL_Y, GL_CTRL_CELL_W, GL_CTRL_CELL_H, "DROP");
        ag::cell(vg, GL_CTRL_X[3], GL_CTRL_CELL_Y, GL_CTRL_CELL_W, GL_CTRL_CELL_H, "CLEAR");
        ag::cell(vg, GL_CTRL_X[4], GL_CTRL_CELL_Y, GL_CTRL_CELL_W, GL_CTRL_CELL_H, "TRIG");
        ag::cell(vg, GL_CTRL_X[5], GL_CTRL_CELL_Y, GL_CTRL_CELL_W, GL_CTRL_CELL_H, "GATES", true);

        ag::text(vg, GL_W * 0.5f, 350.f, "GALTON BOARD  -  BINOMIAL DISTRIBUTION",
                 6.f, ag::dim(), ag::ALIGN_C, 1.5f);

        Widget::draw(args);
    }
};

// ------------------------------------------------------------
// GLBoardDisplay — live peg-board rendering.
// ------------------------------------------------------------
struct GLBoardDisplay : TransparentWidget {
    Galton* module = nullptr;
    GLBoardDisplay(Galton* m) : module(m) {
        box.pos  = Vec(GL_LDX, GL_LDY);
        box.size = Vec(GL_LDW, GL_LDH);
    }

    void draw(const DrawArgs& args) override {
        ag::screen(args.vg, 0.f, 0.f, GL_LDW, GL_LDH);
        Widget::draw(args);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;

        int slots = 8, ballCount = 0;
        float ballX[GL_MAX_BALLS] = {}, ballY[GL_MAX_BALLS] = {};
        float led[GL_N] = {};

        if (module) {
            slots     = module->dispSlots;
            ballCount = module->dispBallCount;
            for (int i = 0; i < ballCount; i++) {
                ballX[i] = module->dispBallX[i];
                ballY[i] = module->dispBallY[i];
            }
            for (int i = 0; i < GL_N; i++) led[i] = module->dispLed[i];
        } else {
            slots = 8; ballCount = 3;
            ballX[0] = 0.50f; ballY[0] = 0.30f;
            ballX[1] = 0.45f; ballY[1] = 0.55f;
            ballX[2] = 0.55f; ballY[2] = 0.75f;
            led[4] = 0.5f;
        }

        // Header
        char buf[48];
        snprintf(buf, sizeof(buf), "BOARD  SLOTS %d   BALLS %d", slots, ballCount);
        ag::text(vg, 8.f, 10.f, buf, 7.f, ag::accent(), ag::ALIGN_L, 1.f);

        // Plot area
        const float padX    = 14.f;
        const float playTop = 22.f;
        const float playBot = GL_LDH - 28.f;
        const float playW   = GL_LDW - 2.f * padX;

        auto mapX = [&](float nx) { return padX + nx * playW; };
        auto mapY = [&](float ny) { return playTop + ny * (playBot - playTop); };

        nvgSave(vg);
        nvgScissor(vg, 0.f, 0.f, GL_LDW, GL_LDH);

        // Pegs
        int   numRows    = std::max(1, slots - 1);
        float pegSpacing = 1.f / (float)numRows;
        const float yTop = 0.15f, yBot = 0.75f;
        const float yStep = (numRows > 1) ? (yBot - yTop) / (numRows - 1) : 0.f;

        nvgBeginPath(vg);
        for (int row = 0; row < numRows; row++) {
            float dy = mapY(yTop + yStep * row);
            for (int k = 0; k <= row; k++) {
                float px = 0.5f + ((float)k - (float)row * 0.5f) * pegSpacing;
                nvgCircle(vg, mapX(px), dy, 2.8f);
            }
        }
        nvgFillColor(vg, nvgRGBA(0x60, 0x70, 0x80, 200));
        nvgFill(vg);

        // Bin dividers
        for (int i = 1; i < slots; i++) {
            float bx = mapX((float)i / (float)slots);
            nvgBeginPath(vg);
            nvgMoveTo(vg, bx, mapY(0.78f));
            nvgLineTo(vg, bx, playBot);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
        }

        // Falling balls (glow + core)
        for (int i = 0; i < ballCount; i++) {
            float dx = mapX(ballX[i]);
            float dy = mapY(ballY[i]);

            nvgBeginPath(vg);
            nvgCircle(vg, dx, dy, 7.f);
            nvgFillColor(vg, ag::withA(ag::accent(), 0.20f));
            nvgFill(vg);

            nvgBeginPath(vg);
            nvgCircle(vg, dx, dy, 3.f);
            nvgFillColor(vg, ag::bright());
            nvgFill(vg);
        }

        // In-display slot indicators
        for (int i = 0; i < slots && i < GL_N; i++) {
            float cx = mapX(((float)i + 0.5f) / (float)slots);
            float cy = playBot - 8.f;
            if (led[i] > 0.01f) {
                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, 5.f);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.30f * led[i]));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, 3.f);
                nvgFillColor(vg, ag::withA(ag::accent(), 0.4f + 0.6f * led[i]));
                nvgFill(vg);
            } else {
                nvgBeginPath(vg);
                nvgCircle(vg, cx, cy, 2.5f);
                nvgFillColor(vg, nvgRGBA(255, 255, 255, 30));
                nvgFill(vg);
            }
        }

        nvgRestore(vg);
        Widget::drawLayer(args, layer);
    }
};

// ------------------------------------------------------------
// GLHistDisplay — live histogram + 2×2 statistics grid.
// ------------------------------------------------------------
struct GLHistDisplay : TransparentWidget {
    Galton* module = nullptr;
    GLHistDisplay(Galton* m) : module(m) {
        box.pos  = Vec(GL_RDX, GL_RDY);
        box.size = Vec(GL_RDW, GL_RDH);
    }

    void draw(const DrawArgs& args) override {
        ag::screen(args.vg, 0.f, 0.f, GL_RDW, GL_RDH);
        Widget::draw(args);
    }

    // Helper — one bordered stats cell.
    void drawStatCell(NVGcontext* vg, float x, float y, float w, float h,
                      const char* label, const char* value, const char* sub) {
        nvgBeginPath(vg);
        nvgRoundedRect(vg, x, y, w, h, 4.f);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 8));
        nvgFill(vg);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 28));
        nvgStrokeWidth(vg, 1.f);
        nvgStroke(vg);

        ag::text(vg, x + 5.f, y + 5.f,  label, 5.5f, ag::dim(),  ag::ALIGN_L, 0.4f);
        ag::text(vg, x + 5.f, y + 15.f, value, 9.f,   nvgRGB(255, 255, 255), ag::ALIGN_L, 0.4f);
        if (sub)
            ag::text(vg, x + 5.f, y + h - 4.f, sub, 5.f, ag::dim(), ag::ALIGN_L, 0.2f);
    }

    void drawLayer(const DrawArgs& args, int layer) override {
        if (layer != 1) { Widget::drawLayer(args, layer); return; }
        NVGcontext* vg = args.vg;

        // -------- snapshot --------
        int slots = 8;
        int counts[GL_N] = {};
        int   statBalls   = 0;
        float statMean    = 0.f, statStdDev = 0.f, statChiSq = 0.f;
        int   statDof     = 0;
        float statExpMean = 4.5f, statExpStd = 1.32f;
        float binomPMF[GL_N] = {};

        if (module) {
            slots = module->dispSlots;
            for (int i = 0; i < GL_N; i++) {
                counts[i]   = module->dispCounts[i];
                binomPMF[i] = module->binomPMF[i];
            }
            statBalls   = module->statBalls;
            statMean    = module->statMean;
            statStdDev  = module->statStdDev;
            statChiSq   = module->statChiSq;
            statDof     = module->statDof;
            statExpMean = module->statExpMean;
            statExpStd  = module->statExpStd;
        } else {
            slots = 8;
            counts[2] = 5; counts[3] = 12; counts[4] = 15;
            counts[5] = 10; counts[6] = 5; counts[7] = 2;
            statBalls = 49;
            for (int i = 0; i < slots; i++) binomPMF[i] = Galton::binomialPMF(i, slots - 1, 0.5f);
            statMean   = 4.6f; statStdDev = 1.5f; statChiSq = 3.2f;
            statDof    = slots - 1;
            statExpMean = (float)(slots + 1) * 0.5f;
            statExpStd  = std::sqrt((float)(slots - 1) * 0.25f);
        }

        // -------- header + legend --------
        ag::text(vg, 8.f, 10.f, "HISTOGRAM", 7.f, ag::accent(), ag::ALIGN_L, 1.f);

        const float ly = 10.f;
        nvgBeginPath(vg); nvgRect(vg, 118.f, 6.5f, 6.f, 6.f);
        nvgFillColor(vg, ag::withA(ag::accent(), 0.65f)); nvgFill(vg);
        ag::text(vg, 127.f, ly, "BALLS", 5.f, ag::dim(), ag::ALIGN_L, 0.2f);

        nvgBeginPath(vg); nvgMoveTo(vg, 160.f, ly); nvgLineTo(vg, 168.f, ly);
        nvgStrokeColor(vg, nvgRGB(255, 255, 255)); nvgStrokeWidth(vg, 1.5f); nvgStroke(vg);
        ag::text(vg, 171.f, ly, "EXP", 5.f, ag::dim(), ag::ALIGN_L, 0.2f);

        nvgBeginPath(vg); nvgMoveTo(vg, 198.f, ly); nvgLineTo(vg, 206.f, ly);
        nvgStrokeColor(vg, ag::warm()); nvgStrokeWidth(vg, 1.5f); nvgStroke(vg);
        ag::text(vg, 209.f, ly, "FIT", 5.f, ag::dim(), ag::ALIGN_L, 0.2f);

        // -------- plot area --------
        const float plotL = 26.f;
        const float plotR = GL_RDW - 8.f;
        const float plotT = 22.f;
        const float plotB = 118.f;
        const float plotW = plotR - plotL;
        const float plotH = plotB - plotT;

        int maxCount = 0;
        for (int i = 0; i < slots; i++) maxCount = std::max(maxCount, counts[i]);
        int yMax = std::max(10, (int)std::ceil((float)maxCount * 1.2f / 5.f) * 5);

        auto mapY = [&](float v) { return plotB - (v / (float)yMax) * plotH; };

        // Y-axis grid + labels
        for (int k = 0; k <= 3; k++) {
            float v = yMax * (float)k / 3.f;
            float y = mapY(v);
            nvgBeginPath(vg);
            nvgMoveTo(vg, plotL, y + 0.5f);
            nvgLineTo(vg, plotR, y + 0.5f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, k == 0 ? 30 : 16));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", (int)std::round(v));
            ag::text(vg, plotL - 4.f, y, buf, 5.5f, ag::dim(), ag::ALIGN_R, 0.2f);
        }

        // Bars
        float binW = plotW / (float)slots;
        for (int i = 0; i < slots; i++) {
            float bx = plotL + i * binW;
            float h  = (float)counts[i] / (float)yMax * plotH;
            if (h < 0.3f) continue;
            nvgBeginPath(vg);
            nvgRect(vg, bx + 1.f, plotB - h, binW - 2.f, h);
            nvgFillColor(vg, ag::withA(ag::accent(), 0.60f));
            nvgFill(vg);
            nvgBeginPath(vg);
            nvgRect(vg, bx + 1.f, plotB - h, binW - 2.f, 1.5f);
            nvgFillColor(vg, ag::withA(ag::bright(), 0.85f));
            nvgFill(vg);
        }

        // Expected ticks
        if (statBalls > 0) {
            for (int i = 0; i < slots; i++) {
                float exp = statBalls * binomPMF[i];
                if (exp < 0.1f) continue;
                float y = mapY(exp);
                float bx = plotL + i * binW;
                nvgBeginPath(vg);
                nvgMoveTo(vg, bx + 1.f,        y);
                nvgLineTo(vg, bx + binW - 1.f, y);
                nvgStrokeColor(vg, nvgRGB(255, 255, 255));
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }
        }

        // Normal fit curve
        if (statBalls > 0 && statExpStd > 0.01f) {
            float mu    = statExpMean;
            float sigma = statExpStd;
            float norm  = 1.f / (sigma * std::sqrt(2.f * GL_PI));
            nvgBeginPath(vg);
            bool started = false;
            const int STEPS = 120;
            for (int s = 0; s <= STEPS; s++) {
                float v = 0.5f + (float)s / (float)STEPS * ((float)slots + 0.5f);
                float pdf = norm * std::exp(-0.5f * std::pow((v - mu) / sigma, 2.f));
                float y = mapY(statBalls * pdf);
                float x = plotL + (v - 0.5f) / (float)slots * plotW;
                if (!started) { nvgMoveTo(vg, x, y); started = true; }
                else nvgLineTo(vg, x, y);
            }
            nvgStrokeColor(vg, ag::warm());
            nvgStrokeWidth(vg, 2.f);
            nvgStroke(vg);
        }

        // X-axis labels
        for (int i = 0; i < slots; i++) {
            float cx = plotL + ((float)i + 0.5f) * binW;
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", i + 1);
            ag::text(vg, cx, plotB + 8.f, buf, 5.5f, ag::dim(), ag::ALIGN_C, 0.f);
        }

        // -------- statistics 2×2 --------
        const float cellW = (GL_RDW - 3.f * 8.f) / 2.f;
        const float cellH = 26.f;
        const float rowY1 = plotB + 18.f;
        const float rowY2 = rowY1 + cellH + 4.f;
        const float colX1 = 8.f;
        const float colX2 = colX1 + cellW + 8.f;

        char v1[32], v2[40], sub[40];

        snprintf(v1, sizeof(v1), "%d", statBalls);
        drawStatCell(vg, colX1, rowY1, cellW, cellH, "Balls", v1, nullptr);

        snprintf(v1, sizeof(v1), "%.2f", statMean);
        snprintf(sub, sizeof(sub), "expected %.2f", statExpMean);
        drawStatCell(vg, colX2, rowY1, cellW, cellH, "Mean slot", v1, sub);

        snprintf(v1, sizeof(v1), "%.2f", statStdDev);
        snprintf(sub, sizeof(sub), "expected %.2f", statExpStd);
        drawStatCell(vg, colX1, rowY2, cellW, cellH, "Std. deviation", v1, sub);

        snprintf(v1, sizeof(v1), "%.2f", statChiSq);
        snprintf(v2, sizeof(v2), "%d degrees of freedom", statDof);
        drawStatCell(vg, colX2, rowY2, cellW, cellH, "Chi-squared", v1, v2);

        Widget::drawLayer(args, layer);
    }
};

} // namespace

// ============================================================================
// ModuleWidget
// ============================================================================
struct GaltonWidget : ModuleWidget {
    GaltonWidget(Galton* module) {
        setModule(module);
        box.size = Vec(GL_W, GL_H);

        addChild(new GLPanel());
        addChild(new GLBoardDisplay(module));
        addChild(new GLHistDisplay(module));

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(GL_CTRL_X[0], GL_CTRL_Y), module, Galton::SPEED_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(
            Vec(GL_CTRL_X[1], GL_CTRL_Y), module, Galton::SLOTS_PARAM));
        addParam(createParamCentered<LEDButton>(
            Vec(GL_CTRL_X[2], GL_CTRL_Y), module, Galton::DROP_PARAM));
        addParam(createParamCentered<LEDButton>(
            Vec(GL_CTRL_X[3], GL_CTRL_Y), module, Galton::CLEAR_PARAM));
        addInput(createInputCentered<DarkPJ301MPort>(
            Vec(GL_CTRL_X[4], GL_CTRL_Y), module, Galton::DROP_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(
            Vec(GL_CTRL_X[5], GL_CTRL_Y), module, Galton::GATES_OUTPUT));

        for (int i = 0; i < GL_N; i++) {
            float x = GL_LED_X0 + (GL_LED_X1 - GL_LED_X0) * (float)i / (float)(GL_N - 1);
            addChild(createLightCentered<SmallLight<ag::CyanLight>>(
                Vec(x, GL_LED_Y), module, Galton::SLOT_LIGHT + i));
        }
    }

    void appendContextMenu(Menu* menu) override {
        Galton* module = dynamic_cast<Galton*>(this->module);
        if (!module) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createSubmenuItem("Gate duration", "", [=](Menu* sub) {
            for (int i = 0; i < NUM_GATE_PRESETS; i++) {
                sub->addChild(createCheckMenuItem(GATE_PRESET_NAMES[i], "",
                    [=]() { return module->gateIdx == i; },
                    [=]() {
                        module->gateIdx = i;
                        module->gateDuration = GATE_PRESET_VALUES[i];
                    }));
            }
        }));
    }
};

Model* modelGalton = createModel<Galton, GaltonWidget>("Galton");