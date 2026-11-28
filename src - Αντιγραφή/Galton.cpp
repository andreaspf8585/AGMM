// ============================================================================
// Galton - interactive Galton board with real ball physics (40 HP)
// ----------------------------------------------------------------------------
// Balls fall under gravity and bounce off a triangular lattice of pegs, the
// bin dividers and the side walls. Every bounce is a real collision, so the
// landing slots form a bell curve (central limit theorem) without any dice
// being rolled for the left/right decision.
//
//   SPEED   time scale of the simulation (0.2x .. 3x)
//   SLOTS   number of bins / LEDs / poly channels (4 .. 16)
//   RATE    automatic drop rate in balls per second (0 = manual only)
//   BOUNCE  peg restitution: how lively the balls are
//   JITTER  surface roughness of the pegs: how random each bounce is
//   TILT    tilts the board sideways and skews the distribution
//   ROOT    root note (C2 .. C6, default C4): played by the middle slot
//   SCALE   scale of the other slots (default Major); slots above and below
//           the middle climb and descend the scale across octaves
//   POLY    polyphony channels of the V/OCT and V GATE outputs (default 1)
//   DROP    drops a ball now          CLEAR   empties the board and the stats
//
//   Inputs   DROP IN, CLEAR IN, TILT CV (+-5 V = full tilt)
//   Outputs  GATES   poly gate, channel i fires when a ball lands in slot i
//            SLOT CV slot of the last landing, 0..10 V (or -5..+5 V)
//            LAND    trigger on every landing
//            HIT     trigger on every peg bounce
//            ROW CV  peg row of the last bounce, 0..10 V (or -5..+5 V)
//            V/OCT   pitch of the landing slot, 1 V/oct with C4 = 0 V. Every
//                    landing takes the next of POLY voices (round robin)
//            V GATE  gate for each V/OCT voice
//   Menu     gate length, bipolar CV outputs
//
// Left display  : the board, with pegs lighting up on impact and the bins
//                 filling up with the running count.
// Right display : live histogram against the ideal binomial counts and its
//                 normal fit, with mean, standard deviation and chi-squared.
// ============================================================================
#include "plugin.hpp"
#include "agmm_ui.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include "Knob_arcs.hpp"

namespace ag = agmm;

// ============================================================================
// Ball physics (no Rack dependencies)
// ----------------------------------------------------------------------------
// World units: one slot is 1.0 wide. Pegs sit on a triangular lattice with
// horizontal spacing 1.0 and row spacing 0.5; row r has r+1 pegs, so a board
// with N slots has N-1 rows and a ball passes N-1 pegs before it lands.
// y points down. Each ball is a disc that is integrated with fixed sub-steps
// and bounces off pegs (circles), bin dividers (capsules), walls and floor.
// ============================================================================
struct GLWorld
{
    enum
    {
        MAXB = 48,
        MAXN = 16
    };

    struct Ball
    {
        float x = 0.f, y = 0.f, vx = 0.f, vy = 0.f;
        float age = 0.f;   // seconds since landing
        float still = 0.f; // seconds spent almost motionless
        float life = 0.f;  // seconds since the drop
        bool active = false;
        bool landed = false;
    };

    Ball balls[MAXB];
    float pegFlash[MAXN * MAXN] = {};

    // Geometry
    int slots = 8, rows = 7;
    float pegTop = 0.8f, rowStep = 0.5f, binTop = 4.9f, worldH = 5.9f;
    static constexpr float BALL_R = 0.19f;
    static constexpr float PEG_R = 0.08f;
    static constexpr float DIV_R = 0.035f;
    static constexpr float GRAVITY = 40.f;
    static constexpr float MAX_SPEED = 30.f;
    static constexpr float DRAG = 1.2f; // air drag (1/s), damps sideways drift
    static constexpr float LAND_FADE = 0.7f;

    // Events produced by step(); the caller consumes and resets them.
    int landCount = 0;
    int landSlot[MAXB] = {};
    int hitCount = 0;
    int lastHitRow = 0;
    float lastHitImpact = 0.f;

    uint32_t rng = 2463534242u;

    float rnd()
    {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return (float)(rng >> 8) * (1.f / 16777216.f);
    }

    // Geometry helpers shared with the display.
    static float binTopOf(int s) { return 0.8f + (float)(s - 2) * 0.5f + 0.6f; }
    static float worldHeight(int s) { return binTopOf(s) + 1.0f; }

    void setSlots(int s)
    {
        slots = s < 4 ? 4 : (s > (int)MAXN ? (int)MAXN : s);
        rows = slots - 1;
        pegTop = 0.8f;
        rowStep = 0.5f;
        binTop = binTopOf(slots);
        worldH = worldHeight(slots);
        clearBalls();
    }

    void clearBalls()
    {
        for (int i = 0; i < MAXB; ++i)
            balls[i] = Ball();
        for (int i = 0; i < MAXN * MAXN; ++i)
            pegFlash[i] = 0.f;
        landCount = 0;
        hitCount = 0;
    }

    float pegX(int r, int k) const { return (float)slots * 0.5f + ((float)k - 0.5f * (float)r); }
    float pegY(int r) const { return pegTop + rowStep * (float)r; }

    int activeCount() const
    {
        int n = 0;
        for (int i = 0; i < MAXB; ++i)
            if (balls[i].active && !balls[i].landed)
                ++n;
        return n;
    }

    // Drop a ball slightly off-centre above the top peg.
    bool spawn()
    {
        for (int i = 0; i < MAXB; ++i)
        {
            if (!balls[i].active)
            {
                Ball &b = balls[i];
                b = Ball();
                b.active = true;
                b.x = (float)slots * 0.5f + (rnd() - 0.5f) * 0.30f;
                b.y = 0.30f;
                b.vx = (rnd() - 0.5f) * 0.6f;
                b.vy = 0.f;
                return true;
            }
        }
        return false;
    }

    // Circle-circle contact between a ball and a fixed disc. Returns true on
    // an impact (the normal speed before the bounce is written to `impact`).
    bool collideDisc(Ball &b, float cx, float cy, float rr, float e, float jitter, float &impact)
    {
        const float rad = BALL_R + rr;
        float dx = b.x - cx, dy = b.y - cy;
        float d2 = dx * dx + dy * dy;
        if (d2 >= rad * rad)
            return false;

        float d = std::sqrt(d2), nx, ny;
        if (d < 1e-5f)
        {
            nx = 0.f;
            ny = -1.f;
        }
        else
        {
            nx = dx / d;
            ny = dy / d;
        }
        b.x = cx + nx * rad;
        b.y = cy + ny * rad;

        float vn = b.vx * nx + b.vy * ny;
        if (vn >= 0.f)
            return false;
        impact = -vn;

        // Rough surface: reflect about a slightly rotated normal.
        float ang = jitter * 1.2f * (rnd() * 2.f - 1.f);
        float c = std::cos(ang), s = std::sin(ang);
        float mx = nx * c - ny * s, my = nx * s + ny * c;
        float vm = b.vx * mx + b.vy * my;
        if (vm < 0.f)
        {
            b.vx -= (1.f + e) * vm * mx;
            b.vy -= (1.f + e) * vm * my;
        }
        // Never leave the contact moving into the surface.
        float vn2 = b.vx * nx + b.vy * ny;
        if (vn2 < 0.f)
        {
            b.vx -= vn2 * nx;
            b.vy -= vn2 * ny;
        }
        // Slight tangential friction.
        float vt = b.vx * (-ny) + b.vy * nx;
        b.vx -= 0.08f * vt * (-ny);
        b.vy -= 0.08f * vt * nx;
        return true;
    }

    // Advance the simulation by h seconds. `gx` is the sideways gravity
    // (tilt), `e` the peg restitution and `jitter` the surface roughness.
    void step(float h, float gx, float e, float jitter)
    {
        for (int i = 0; i < MAXN * MAXN; ++i)
            pegFlash[i] = std::max(0.f, pegFlash[i] - h * 3.f);

        for (int bi = 0; bi < MAXB; ++bi)
        {
            Ball &b = balls[bi];
            if (!b.active)
                continue;

            b.vy += GRAVITY * h;
            b.vx += gx * h;
            {
                float k = 1.f - DRAG * h;
                b.vx *= k;
                b.vy *= k;
            }
            float sp2 = b.vx * b.vx + b.vy * b.vy;
            if (sp2 > MAX_SPEED * MAX_SPEED)
            {
                float k = MAX_SPEED / std::sqrt(sp2);
                b.vx *= k;
                b.vy *= k;
            }
            b.x += b.vx * h;
            b.y += b.vy * h;

            // Pegs: only the rows near the ball are tested.
            if (!b.landed && b.y < binTop)
            {
                int r0 = (int)std::floor((b.y - pegTop) / rowStep + 0.5f);
                for (int r = r0 - 1; r <= r0 + 1; ++r)
                {
                    if (r < 0 || r >= rows)
                        continue;
                    int kc = (int)std::floor(b.x - (float)slots * 0.5f + 0.5f * (float)r + 0.5f);
                    for (int k = kc - 1; k <= kc + 1; ++k)
                    {
                        if (k < 0 || k > r)
                            continue;
                        float impact = 0.f;
                        if (collideDisc(b, pegX(r, k), pegY(r), PEG_R, e, jitter, impact))
                        {
                            pegFlash[r * MAXN + k] = 1.f;
                            ++hitCount;
                            lastHitRow = r;
                            lastHitImpact = impact;
                        }
                    }
                }
            }

            // Bin dividers (vertical capsules between the bins).
            if (b.y > binTop - BALL_R - DIV_R)
            {
                int i0 = (int)std::floor(b.x);
                for (int d = i0; d <= i0 + 1; ++d)
                {
                    if (d < 1 || d > slots - 1)
                        continue;
                    float cy = std::min(std::max(b.y, binTop), worldH);
                    float impact = 0.f;
                    collideDisc(b, (float)d, cy, DIV_R, 0.25f, 0.f, impact);
                }
            }

            // Side walls and floor
            if (b.x < BALL_R)
            {
                b.x = BALL_R;
                if (b.vx < 0.f)
                    b.vx = -b.vx * 0.3f;
            }
            if (b.x > (float)slots - BALL_R)
            {
                b.x = (float)slots - BALL_R;
                if (b.vx > 0.f)
                    b.vx = -b.vx * 0.3f;
            }
            if (b.y > worldH - BALL_R)
            {
                b.y = worldH - BALL_R;
                if (b.vy > 0.f)
                    b.vy = -b.vy * 0.2f;
                b.vx *= 0.9f;
            }

            if (!b.landed)
            {
                // Landed once the ball is well inside a bin.
                if (b.y > binTop + 0.4f)
                {
                    b.landed = true;
                    b.age = 0.f;
                    int slot = (int)std::floor(b.x);
                    slot = std::min(std::max(slot, 0), slots - 1);
                    if (landCount < MAXB)
                        landSlot[landCount++] = slot;
                }
                else
                {
                    // A ball balanced on a peg gets a small shove.
                    float sp = std::sqrt(b.vx * b.vx + b.vy * b.vy);
                    if (sp < 0.15f)
                        b.still += h;
                    else
                        b.still = 0.f;
                    if (b.still > 0.3f)
                    {
                        b.vx += (rnd() - 0.5f) * 1.5f;
                        b.still = 0.f;
                    }
                    // A ball that dawdles for too long is shaken loose.
                    b.life += h;
                    if (b.life > 5.f)
                    {
                        b.vx += (rnd() - 0.5f) * 6.f;
                        b.vy += 4.f;
                        b.life = 4.f;
                    }
                }
            }
            else
            {
                b.age += h;
                if (b.age > LAND_FADE)
                    b.active = false;
            }
        }
    }
};

// ------------------------------------------------------------
// Layout constants (40 HP = 600 px wide)
// ------------------------------------------------------------
static const float GL_W = RACK_GRID_WIDTH * 40; // 600
static const float GL_H = RACK_GRID_HEIGHT;     // 380

static const int GL_N = GLWorld::MAXN; // slots / LEDs / poly channels
static const int GL_MAX_BALLS = GLWorld::MAXB;

// Displays (bezel adds 4 px on every side)
static const float GL_LDX = 20.f, GL_LDY = 24.f, GL_LDW = 300.f, GL_LDH = 160.f;  // board
static const float GL_RDX = 340.f, GL_RDY = 24.f, GL_RDW = 240.f, GL_RDH = 160.f; // histogram

// Slot LED strip
static const float GL_STRIP_TOP = 194.f;
static const float GL_STRIP_H = 18.f;
static const float GL_LED_Y = GL_STRIP_TOP + GL_STRIP_H * 0.5f;
static const float GL_LED_X0 = 64.f;
static const float GL_LED_X1 = 560.f;

// Control rows: equal cells spanning x = 10 .. 590 (11 in row A, 10 in row B)
static const float GL_ROW_X0 = 10.f;
static const float GL_ROW_W = 580.f;
static const float GL_CELL_WA = 49.f; // row A cell width
static const float GL_CELL_WB = 54.f; // row B cell width
static const float GL_CELL_H = 52.f;
static const float GL_CTRL_DY = 33.f; // control centre below the cell top

static const float GL_RA_HDR = 222.f; // CONTROLS
static const float GL_RA_TOP = 230.f;
static const float GL_RB_HDR = 290.f; // I / O
static const float GL_RB_TOP = 298.f;

// Centre of cell i of n equal cells in a row.
static inline float glCellX(int i, int n)
{
    return GL_ROW_X0 + GL_ROW_W / (float)n * ((float)i + 0.5f);
}

// ------------------------------------------------------------
// Gate-duration presets (right-click menu)
// ------------------------------------------------------------
static const int NUM_GATE_PRESETS = 10;
static const float GATE_PRESET_VALUES[NUM_GATE_PRESETS] = {
    0.005f, 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.25f, 0.5f, 0.75f, 1.0f};
static const char *GATE_PRESET_NAMES[NUM_GATE_PRESETS] = {
    "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms", "250 ms", "500 ms", "750 ms", "1 s"};

static const float GL_PI = 3.14159265358979f;

// ------------------------------------------------------------
// Scales: the middle slot is the root, slots above/below it climb/descend the
// scale (and wrap into the next octave).
// ------------------------------------------------------------
static const int GL_NUM_SCALES = 16;
static const int GL_SCALE_LEN[GL_NUM_SCALES] = {7, 7, 7, 7, 7, 7, 7, 7, 7, 5, 5, 6, 6, 8, 7, 12};
static const int GL_SCALE[GL_NUM_SCALES][12] = {
    {0, 2, 4, 5, 7, 9, 11},                // Major
    {0, 2, 3, 5, 7, 8, 10},                // Natural minor
    {0, 2, 3, 5, 7, 8, 11},                // Harmonic minor
    {0, 2, 3, 5, 7, 9, 11},                // Melodic minor
    {0, 2, 3, 5, 7, 9, 10},                // Dorian
    {0, 1, 3, 5, 7, 8, 10},                // Phrygian
    {0, 2, 4, 6, 7, 9, 11},                // Lydian
    {0, 2, 4, 5, 7, 9, 10},                // Mixolydian
    {0, 1, 3, 5, 6, 8, 10},                // Locrian
    {0, 2, 4, 7, 9},                       // Major pentatonic
    {0, 3, 5, 7, 10},                      // Minor pentatonic
    {0, 3, 5, 6, 7, 10},                   // Blues
    {0, 2, 4, 6, 8, 10},                   // Whole tone
    {0, 2, 3, 5, 6, 8, 9, 11},             // Diminished (whole-half)
    {0, 1, 4, 5, 7, 8, 10},                // Phrygian dominant
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11} // Chromatic
};
static const char *GL_SCALE_NAMES[GL_NUM_SCALES] = {
    "Major", "Natural minor", "Harmonic minor", "Melodic minor", "Dorian", "Phrygian",
    "Lydian", "Mixolydian", "Locrian", "Major pentatonic", "Minor pentatonic", "Blues",
    "Whole tone", "Diminished", "Phrygian dominant", "Chromatic"};
static const char *GL_SCALE_SHORT[GL_NUM_SCALES] = {
    "MAJOR", "MINOR", "HARM MIN", "MEL MIN", "DORIAN", "PHRYGIAN", "LYDIAN", "MIXOLYD",
    "LOCRIAN", "MAJ PENT", "MIN PENT", "BLUES", "WHOLE", "DIMIN", "PHRYG DOM", "CHROMATIC"};

static inline int glFloorDiv(int a, int b)
{
    int q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0)))
        q--;
    return q;
}

// The middle slot is the root; with an even number of slots it is the lower
// of the two centre slots.
static inline int glMidSlot(int slots) { return (slots - 1) / 2; }

// Semitones above C4 of a slot, for the given root (semitones above C4) and scale.
static inline int glSlotSemis(int slot, int slots, int root, int scale)
{
    scale = rack::math::clamp(scale, 0, GL_NUM_SCALES - 1);
    const int n = GL_SCALE_LEN[scale];
    const int d = slot - glMidSlot(slots);
    const int oct = glFloorDiv(d, n);
    const int idx = d - oct * n;
    return root + oct * 12 + GL_SCALE[scale][idx];
}

// Note name for semitones above C4 (0 = C4).
static inline void glNoteName(int semis, bool withOctave, char *buf, size_t n)
{
    static const char *names[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    const int midi = 60 + semis;
    const int pc = ((midi % 12) + 12) % 12;
    const int oct = glFloorDiv(midi, 12) - 1;
    if (withOctave)
        snprintf(buf, n, "%s%d", names[pc], oct);
    else
        snprintf(buf, n, "%s", names[pc]);
}

// ============================================================================
// Module
// ============================================================================
struct Galton : Module
{
    // New parameters, inputs and outputs are appended after the original ones
    // so that existing patches keep loading correctly.
    enum ParamId
    {
        SPEED_PARAM,
        SLOTS_PARAM,
        DROP_PARAM,
        CLEAR_PARAM,
        RATE_PARAM,
        BOUNCE_PARAM,
        JITTER_PARAM,
        TILT_PARAM,
        ROOT_PARAM,
        SCALE_PARAM,
        POLY_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        DROP_INPUT,
        CLEAR_INPUT,
        TILT_INPUT,
        INPUTS_LEN
    };
    enum OutputId
    {
        GATES_OUTPUT,
        CV_OUTPUT,
        LAND_OUTPUT,
        HIT_OUTPUT,
        ROW_OUTPUT,
        VOCT_OUTPUT,
        VGATE_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        ENUMS(SLOT_LIGHT, GLWorld::MAXN),
        LIGHTS_LEN
    };

    static constexpr float SIM_STEP = 1.f / 480.f; // physics sub-step (s)

    // Root note shown as a note name (the value is semitones above C4).
    struct RootQuantity : ParamQuantity
    {
        std::string getDisplayValueString() override
        {
            char buf[16];
            glNoteName((int)std::round(getValue()), true, buf, sizeof(buf));
            return buf;
        }
    };

    GLWorld world;
    float simAcc = 0.f;
    float spawnAcc = 0.f;

    // --------------------------------------------------------
    // User state
    // --------------------------------------------------------
    int slots = 8;
    float gateDuration = 0.05f;
    int gateIdx = 3;
    bool cvBipolar = false;

    // --------------------------------------------------------
    // Gate / LED / histogram state
    // --------------------------------------------------------
    dsp::PulseGenerator gatePulses[GLWorld::MAXN];
    dsp::PulseGenerator landPulse, hitPulse;
    float ledFlash[GLWorld::MAXN] = {};
    int binCounts[GLWorld::MAXN] = {};
    bool statsDirty = true;
    float binomPMF[GLWorld::MAXN] = {}; // Binomial(slots-1, 0.5) PMF
    int lastSlot = 0;
    int lastRow = 0;

    // Polyphonic pitch: every landing takes the next voice (round robin).
    float voct[GLWorld::MAXN] = {};
    dsp::PulseGenerator vGate[GLWorld::MAXN];
    int nextVoice = 0;

    dsp::SchmittTrigger dropBtnTrig, dropJackTrig, clearBtnTrig, clearJackTrig;

    // --------------------------------------------------------
    // Display snapshots (written in process(), read by the UI).
    // --------------------------------------------------------
    int snapCount = 0;
    float dispBallX[GLWorld::MAXB] = {};
    float dispBallY[GLWorld::MAXB] = {};
    float dispBallVX[GLWorld::MAXB] = {};
    float dispBallVY[GLWorld::MAXB] = {};
    float dispBallA[GLWorld::MAXB] = {};
    int dispBallCount = 0;
    int dispInPlay = 0;
    int dispSlots = 8;
    float dispTilt = 0.f;
    float dispLed[GLWorld::MAXN] = {};
    int dispCounts[GLWorld::MAXN] = {};
    int dispLast = -1;
    int dispRoot = 0;
    int dispScale = 0;
    float dispPegFlash[GLWorld::MAXN * GLWorld::MAXN] = {};

    // --------------------------------------------------------
    // Statistics (updated when `statsDirty`).
    // --------------------------------------------------------
    int statBalls = 0;
    float statMean = 0.f;
    float statStdDev = 0.f;
    float statChiSq = 0.f;
    int statDof = 0;
    float statExpMean = 4.5f;
    float statExpStd = 1.32f;

    Galton()
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(SPEED_PARAM, 0.2f, 3.f, 1.f, "Simulation speed", "x");
        configParam(SLOTS_PARAM, 4.f, 16.f, 8.f, "Number of slots");
        paramQuantities[SLOTS_PARAM]->snapEnabled = true;
        configButton(DROP_PARAM, "Drop ball");
        configButton(CLEAR_PARAM, "Clear");
        configParam(RATE_PARAM, 0.f, 16.f, 4.f, "Auto drop rate", " balls/s");
        configParam(BOUNCE_PARAM, 0.1f, 0.9f, 0.4f, "Bounce", "%", 0.f, 100.f);
        configParam(JITTER_PARAM, 0.f, 1.f, 0.5f, "Peg jitter", "%", 0.f, 100.f);
        configParam(TILT_PARAM, -1.f, 1.f, 0.f, "Board tilt", "%", 0.f, 100.f);
        configParam<RootQuantity>(ROOT_PARAM, -24.f, 24.f, 0.f, "Root note (middle slot)");
        paramQuantities[ROOT_PARAM]->snapEnabled = true;
        {
            std::vector<std::string> names;
            for (int i = 0; i < GL_NUM_SCALES; i++)
                names.push_back(GL_SCALE_NAMES[i]);
            configSwitch(SCALE_PARAM, 0.f, (float)(GL_NUM_SCALES - 1), 0.f, "Scale", names);
        }
        configParam(POLY_PARAM, 1.f, 16.f, 1.f, "Polyphony channels");
        paramQuantities[POLY_PARAM]->snapEnabled = true;

        configInput(DROP_INPUT, "Drop trigger");
        configInput(CLEAR_INPUT, "Clear trigger");
        configInput(TILT_INPUT, "Tilt CV (+-5 V = full tilt)");
        configOutput(GATES_OUTPUT, "Slot gates (polyphonic)");
        configOutput(CV_OUTPUT, "Slot CV (last landing)");
        configOutput(LAND_OUTPUT, "Landing trigger");
        configOutput(HIT_OUTPUT, "Peg hit trigger");
        configOutput(ROW_OUTPUT, "Peg row CV (last hit)");
        configOutput(VOCT_OUTPUT, "Pitch (1 V/oct, polyphonic)");
        configOutput(VGATE_OUTPUT, "Voice gates (polyphonic)");

        for (int i = 0; i < GL_N; i++)
            configLight(SLOT_LIGHT + i, string::f("Slot %d", i + 1));

        // Assigning `channels` directly avoids a GCC array-bounds false
        // positive in setChannels() at the 16-channel boundary.
        outputs[GATES_OUTPUT].channels = (uint8_t)slots;

        outputs[VOCT_OUTPUT].channels = 1;
        outputs[VGATE_OUTPUT].channels = 1;

        world.rng = random::u32() | 1u;
        applySlots();
        for (int i = 0; i < 2; i++)
            world.spawn();
    }

    // --------------------------------------------------------
    // Binomial PMF P(k | n, p).
    // --------------------------------------------------------
    static float binomialPMF(int k, int n, float p)
    {
        if (k < 0 || k > n)
            return 0.f;
        double c = 1.0;
        for (int i = 0; i < k; i++)
            c = c * (double)(n - i) / (double)(i + 1);
        return (float)(c * std::pow((double)p, k) * std::pow((double)(1.f - p), n - k));
    }

    // Rebuild the board for the current slot count and reset the statistics.
    void applySlots()
    {
        world.setSlots(slots);
        for (int i = 0; i < GL_N; i++)
            binomPMF[i] = 0.f;
        for (int i = 0; i < slots; i++)
            binomPMF[i] = binomialPMF(i, slots - 1, 0.5f);
        for (int i = 0; i < GL_N; i++)
            binCounts[i] = 0;
        lastSlot = 0;
        lastRow = 0;
        statsDirty = true;
    }

    void clearAll()
    {
        world.clearBalls();
        for (int i = 0; i < GL_N; i++)
            binCounts[i] = 0;
        statsDirty = true;
    }

    // --------------------------------------------------------
    // Summary statistics against the current counts.
    // --------------------------------------------------------
    void computeStats()
    {
        int n = 0;
        for (int i = 0; i < slots; i++)
            n += binCounts[i];
        statBalls = n;
        statDof = slots - 1;
        statExpMean = (float)(slots + 1) * 0.5f;
        statExpStd = std::sqrt((float)(slots - 1) * 0.25f);

        if (n < 1)
        {
            statMean = statStdDev = statChiSq = 0.f;
            return;
        }

        float sum = 0.f;
        for (int i = 0; i < slots; i++)
            sum += (float)(i + 1) * binCounts[i];
        statMean = sum / (float)n;

        float varSum = 0.f;
        for (int i = 0; i < slots; i++)
        {
            float d = (float)(i + 1) - statMean;
            varSum += d * d * binCounts[i];
        }
        statStdDev = std::sqrt(varSum / (float)n);

        float chi = 0.f;
        for (int i = 0; i < slots; i++)
        {
            float expected = n * binomPMF[i];
            if (expected >= 1.f)
            {
                float d = (float)binCounts[i] - expected;
                chi += d * d / expected;
            }
        }
        statChiSq = chi;
    }

    void makeSnapshot(float tilt)
    {
        dispBallCount = 0;
        for (int i = 0; i < GLWorld::MAXB; i++)
        {
            const GLWorld::Ball &b = world.balls[i];
            if (!b.active)
                continue;
            dispBallX[dispBallCount] = b.x;
            dispBallY[dispBallCount] = b.y;
            dispBallVX[dispBallCount] = b.vx;
            dispBallVY[dispBallCount] = b.vy;
            dispBallA[dispBallCount] = b.landed ? std::max(0.f, 1.f - b.age / GLWorld::LAND_FADE) : 1.f;
            dispBallCount++;
        }
        dispInPlay = world.activeCount();
        dispSlots = slots;
        dispTilt = tilt;
        dispLast = statBalls > 0 ? lastSlot : -1;
        dispRoot = (int)std::round(params[ROOT_PARAM].getValue());
        dispScale = rack::math::clamp((int)std::round(params[SCALE_PARAM].getValue()), 0, GL_NUM_SCALES - 1);
        for (int i = 0; i < GL_N; i++)
        {
            dispLed[i] = ledFlash[i];
            dispCounts[i] = binCounts[i];
        }
        for (int i = 0; i < GLWorld::MAXN * GLWorld::MAXN; i++)
            dispPegFlash[i] = world.pegFlash[i];
    }

    // --------------------------------------------------------
    // Per-sample entry point.
    // --------------------------------------------------------
    void process(const ProcessArgs &args) override
    {
        const float dt = args.sampleTime;

        // --- params ---
        const float speed = params[SPEED_PARAM].getValue();
        const float rate = params[RATE_PARAM].getValue();
        const float bounce = params[BOUNCE_PARAM].getValue();
        const float jitter = params[JITTER_PARAM].getValue();
        float tilt = params[TILT_PARAM].getValue();
        if (inputs[TILT_INPUT].isConnected())
            tilt += inputs[TILT_INPUT].getVoltage() * 0.2f;
        tilt = rack::math::clamp(tilt, -1.f, 1.f);

        const int root = (int)std::round(params[ROOT_PARAM].getValue());
        const int scale = rack::math::clamp((int)std::round(params[SCALE_PARAM].getValue()), 0, GL_NUM_SCALES - 1);
        const int voices = rack::math::clamp((int)std::round(params[POLY_PARAM].getValue()), 1, GL_N);
        if (nextVoice >= voices)
            nextVoice = 0;

        int newSlots = rack::math::clamp((int)std::round(params[SLOTS_PARAM].getValue()), 4, GL_N);
        if (newSlots != slots)
        {
            slots = newSlots;
            applySlots();
        }

        // --- manual triggers ---
        if (dropBtnTrig.process(params[DROP_PARAM].getValue(), 0.1f, 1.f) ||
            dropJackTrig.process(inputs[DROP_INPUT].getVoltage(), 0.1f, 2.f))
        {
            world.spawn();
        }
        if (clearBtnTrig.process(params[CLEAR_PARAM].getValue(), 0.1f, 1.f) ||
            clearJackTrig.process(inputs[CLEAR_INPUT].getVoltage(), 0.1f, 2.f))
        {
            clearAll();
        }

        // --- automatic drops ---
        if (rate > 0.01f)
        {
            spawnAcc += dt * speed * rate;
            if (spawnAcc >= 1.f)
            {
                spawnAcc = (spawnAcc >= 2.f) ? 0.f : spawnAcc - 1.f;
                world.spawn();
            }
        }
        else
        {
            spawnAcc = 0.f;
        }

        // --- physics: fixed sub-steps so the result does not depend on the
        // sample rate ---
        simAcc += dt * speed;
        int steps = 0;
        const float sideGravity = tilt * GLWorld::GRAVITY * 0.12f;
        while (simAcc >= SIM_STEP && steps < 64)
        {
            world.step(SIM_STEP, sideGravity, bounce, jitter);
            simAcc -= SIM_STEP;
            steps++;
        }
        if (simAcc >= SIM_STEP)
            simAcc = 0.f;

        // --- events from the physics ---
        for (int i = 0; i < world.landCount; i++)
        {
            int s = world.landSlot[i];
            if (s >= 0 && s < GL_N)
            {
                gatePulses[s].trigger(gateDuration);
                ledFlash[s] = 1.f;
                binCounts[s]++;
                lastSlot = s;
                statsDirty = true;
                landPulse.trigger(2e-3f);

                // Pitch: the landing slot picks a scale note for the next voice.
                voct[nextVoice] = (float)glSlotSemis(s, slots, root, scale) / 12.f;
                vGate[nextVoice].trigger(gateDuration);
                nextVoice = (nextVoice + 1) % voices;
            }
        }
        world.landCount = 0;
        if (world.hitCount > 0)
        {
            hitPulse.trigger(2e-3f);
            lastRow = world.lastHitRow;
            world.hitCount = 0;
        }

        // --- gates + panel LEDs ---
        auto &gatePort = outputs[GATES_OUTPUT];
        gatePort.channels = (uint8_t)slots;
        for (int i = 0; i < GL_N; i++)
        {
            bool g = gatePulses[i].process(dt);
            float v = (i < slots && g) ? 10.f : 0.f;
            gatePort.setVoltage(v, i);
            ledFlash[i] = std::max(0.f, ledFlash[i] - dt * 3.f);
            lights[SLOT_LIGHT + i].setBrightness(ledFlash[i]);
        }

        // --- CV and trigger outputs ---
        float slotNorm = (slots > 1) ? (float)lastSlot / (float)(slots - 1) : 0.f;
        float rowNorm = (world.rows > 1) ? (float)lastRow / (float)(world.rows - 1) : 0.f;
        outputs[CV_OUTPUT].setVoltage(cvBipolar ? slotNorm * 10.f - 5.f : slotNorm * 10.f);
        outputs[ROW_OUTPUT].setVoltage(cvBipolar ? rowNorm * 10.f - 5.f : rowNorm * 10.f);
        outputs[VOCT_OUTPUT].channels = (uint8_t)voices;
        outputs[VGATE_OUTPUT].channels = (uint8_t)voices;
        for (int i = 0; i < GL_N; i++)
        {
            bool g = vGate[i].process(dt);
            if (i < voices)
            {
                outputs[VOCT_OUTPUT].setVoltage(voct[i], i);
                outputs[VGATE_OUTPUT].setVoltage(g ? 10.f : 0.f, i);
            }
        }
        outputs[LAND_OUTPUT].setVoltage(landPulse.process(dt) ? 10.f : 0.f);
        outputs[HIT_OUTPUT].setVoltage(hitPulse.process(dt) ? 10.f : 0.f);

        // --- statistics (only when bins changed) ---
        if (statsDirty)
        {
            computeStats();
            statsDirty = false;
        }

        // --- snapshot for the UI (a few hundred times per second) ---
        if (--snapCount <= 0)
        {
            snapCount = 128;
            makeSnapshot(tilt);
        }
    }

    // --------------------------------------------------------
    // Persist / restore options.
    // --------------------------------------------------------
    json_t *dataToJson() override
    {
        json_t *root = json_object();
        json_object_set_new(root, "gateIdx", json_integer(gateIdx));
        json_object_set_new(root, "cvBipolar", json_boolean(cvBipolar));
        return root;
    }
    void dataFromJson(json_t *root) override
    {
        json_t *j = json_object_get(root, "gateIdx");
        if (j)
        {
            gateIdx = rack::math::clamp((int)json_integer_value(j), 0, NUM_GATE_PRESETS - 1);
            gateDuration = GATE_PRESET_VALUES[gateIdx];
        }
        json_t *c = json_object_get(root, "cvBipolar");
        if (c)
            cvBipolar = json_is_true(c);
    }

    void onReset(const ResetEvent &e) override
    {
        Module::onReset(e);
        for (int i = 0; i < GL_N; i++)
            ledFlash[i] = 0.f;
        spawnAcc = 0.f;
        simAcc = 0.f;
        slots = 8;
        nextVoice = 0;
        for (int i = 0; i < GL_N; i++)
            voct[i] = 0.f;
        applySlots();
    }
};

// ============================================================================
// UI
// ============================================================================
namespace
{

    // ------------------------------------------------------------
    // GLPanel - static panel chrome.
    // ------------------------------------------------------------
    struct GLPanel : TransparentWidget
    {
        GLPanel() { box.size = Vec(GL_W, GL_H); }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::panelBase(vg, GL_W, GL_H, "GALTON");
            ag::bezel(vg, GL_LDX, GL_LDY, GL_LDW, GL_LDH);
            ag::bezel(vg, GL_RDX, GL_RDY, GL_RDW, GL_RDH);

            // Slot LED strip
            ag::plate(vg, GL_W * 0.5f, GL_STRIP_TOP, 580.f, GL_STRIP_H);
            ag::text(vg, 18.f, GL_LED_Y, "SLOTS", 7.f, ag::dim(), ag::ALIGN_L, 0.5f);

            // Row A: simulation and drop controls
            ag::sectionHeader(vg, GL_W, GL_RA_HDR, "BOARD, PITCH & POLY", 10.f);
            const char *rowA[11] = {"SPEED", "SLOTS", "RATE", "BOUNCE", "JITTER", "TILT",
                                    "ROOT", "SCALE", "POLY", "DROP", "CLEAR"};
            for (int i = 0; i < 11; i++)
                ag::cell(vg, glCellX(i, 11), GL_RA_TOP, GL_CELL_WA, GL_CELL_H, rowA[i]);

            // Row B: inputs on the left, outputs (tinted) on the right
            ag::sectionHeader(vg, GL_W, GL_RB_HDR, "I / O", 10.f);
            const char *rowB[10] = {"DROP IN", "CLEAR IN", "TILT CV", "GATES", "SLOT CV", "LAND",
                                    "HIT", "ROW CV", "V/OCT", "V GATE"};
            for (int i = 0; i < 10; i++)
                ag::cell(vg, glCellX(i, 10), GL_RB_TOP, GL_CELL_WB, GL_CELL_H, rowB[i], i >= 3);

            Widget::draw(args);
        }
    };

    // ------------------------------------------------------------
    // GLBoardDisplay - the board: pegs, balls, bins.
    // ------------------------------------------------------------
    struct GLBoardDisplay : TransparentWidget
    {
        Galton *module = nullptr;
        GLBoardDisplay(Galton *m) : module(m)
        {
            box.pos = Vec(GL_LDX, GL_LDY);
            box.size = Vec(GL_LDW, GL_LDH);
        }

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, GL_LDW, GL_LDH);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, 18.5f);
            nvgLineTo(vg, GL_LDW - 8.f, 18.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            // -------- snapshot --------
            int slots = 8, ballCount = 0, inPlay = 0, last = -1;
            int root = 0, scaleIdx = 0;
            float tilt = 0.f;
            float bx[GL_MAX_BALLS] = {}, by[GL_MAX_BALLS] = {};
            float bvx[GL_MAX_BALLS] = {}, bvy[GL_MAX_BALLS] = {}, ba[GL_MAX_BALLS] = {};
            float led[GL_N] = {};
            int counts[GL_N] = {};
            float flash[GL_N * GL_N] = {};

            if (module)
            {
                slots = rack::math::clamp(module->dispSlots, 4, GL_N);
                ballCount = rack::math::clamp(module->dispBallCount, 0, GL_MAX_BALLS);
                inPlay = module->dispInPlay;
                tilt = module->dispTilt;
                last = module->dispLast;
                root = module->dispRoot;
                scaleIdx = module->dispScale;
                for (int i = 0; i < ballCount; i++)
                {
                    bx[i] = module->dispBallX[i];
                    by[i] = module->dispBallY[i];
                    bvx[i] = module->dispBallVX[i];
                    bvy[i] = module->dispBallVY[i];
                    ba[i] = module->dispBallA[i];
                }
                for (int i = 0; i < GL_N; i++)
                {
                    led[i] = module->dispLed[i];
                    counts[i] = module->dispCounts[i];
                }
                for (int i = 0; i < GL_N * GL_N; i++)
                    flash[i] = module->dispPegFlash[i];
            }
            else
            {
                // Static preview for the module browser
                ballCount = 3;
                inPlay = 3;
                bx[0] = 4.0f;
                by[0] = 1.6f;
                ba[0] = 1.f;
                bx[1] = 3.3f;
                by[1] = 3.1f;
                ba[1] = 1.f;
                bx[2] = 5.1f;
                by[2] = 4.4f;
                ba[2] = 1.f;
                const int demo[8] = {2, 6, 11, 15, 14, 9, 5, 1};
                for (int i = 0; i < 8; i++)
                    counts[i] = demo[i];
                led[4] = 0.6f;
            }

            // -------- header --------
            char buf[48];
            char rootName[8];
            glNoteName(root, true, rootName, sizeof(rootName));
            snprintf(buf, sizeof(buf), "%s %s  %d SLOTS", rootName, GL_SCALE_SHORT[scaleIdx], slots);
            ag::text(vg, 8.f, 9.f, buf, 8.f, ag::accent(), ag::ALIGN_L, 0.5f);
            snprintf(buf, sizeof(buf), "%d IN PLAY", inPlay);
            ag::text(vg, GL_LDW - 8.f, 9.f, buf, 8.f, ag::dim(), ag::ALIGN_R, 0.5f);
            if (std::fabs(tilt) > 0.005f)
            {
                snprintf(buf, sizeof(buf), "TILT %+.0f%%", tilt * 100.f);
                ag::text(vg, GL_LDW * 0.5f + 6.f, 9.f, buf, 8.f, ag::warm(), ag::ALIGN_C, 0.5f);
            }

            // -------- board geometry --------
            const float playL = 10.f, playR = GL_LDW - 10.f, playT = 24.f, playB = 146.f;
            const float worldH = GLWorld::worldHeight(slots);
            const float binTop = GLWorld::binTopOf(slots);
            const float scale = std::min((playR - playL) / (float)slots, (playB - playT) / worldH);
            const float ox = (GL_LDW - (float)slots * scale) * 0.5f;
            const float oy = playT;
            auto sx = [&](float x)
            { return ox + x * scale; };
            auto sy = [&](float y)
            { return oy + y * scale; };

            nvgSave(vg);
            nvgScissor(vg, 0.f, 19.f, GL_LDW, GL_LDH - 19.f);

            // Bins: flash on landing, running count as a flat fill
            int maxCount = 1;
            for (int i = 0; i < slots; i++)
                maxCount = std::max(maxCount, counts[i]);
            const float binH = (worldH - binTop) * scale;
            for (int i = 0; i < slots; i++)
            {
                float x = sx((float)i);
                float fh = (float)counts[i] / (float)maxCount * binH * 0.85f;
                if (fh > 0.5f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 1.f, sy(worldH) - fh, scale - 2.f, fh);
                    nvgFillColor(vg, ag::withA(ag::accent(), i == last ? 0.30f : 0.18f));
                    nvgFill(vg);
                }
                if (led[i] > 0.01f)
                {
                    nvgBeginPath(vg);
                    nvgRect(vg, x + 1.f, sy(binTop), scale - 2.f, binH);
                    nvgFillColor(vg, ag::withA(ag::accent(), 0.30f * led[i]));
                    nvgFill(vg);
                }
            }

            // Walls, floor and dividers
            nvgBeginPath(vg);
            nvgMoveTo(vg, sx(0.f) + 0.5f, sy(0.f));
            nvgLineTo(vg, sx(0.f) + 0.5f, sy(worldH) - 0.5f);
            nvgLineTo(vg, sx((float)slots) - 0.5f, sy(worldH) - 0.5f);
            nvgLineTo(vg, sx((float)slots) - 0.5f, sy(0.f));
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 45));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            for (int i = 1; i < slots; i++)
            {
                float x = std::floor(sx((float)i)) + 0.5f;
                nvgMoveTo(vg, x, sy(binTop));
                nvgLineTo(vg, x, sy(worldH));
            }
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 80));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            // Drop point marker
            nvgBeginPath(vg);
            nvgMoveTo(vg, sx((float)slots * 0.5f) - 3.f, sy(0.02f));
            nvgLineTo(vg, sx((float)slots * 0.5f) + 3.f, sy(0.02f));
            nvgLineTo(vg, sx((float)slots * 0.5f), sy(0.02f) + 4.f);
            nvgClosePath(vg);
            nvgFillColor(vg, ag::withA(ag::dim(), 0.7f));
            nvgFill(vg);

            // Pegs: dim, turning cyan for a moment when hit
            const int numRows = std::max(1, slots - 1);
            const float pegPx = std::max(GLWorld::PEG_R * scale, 1.6f);
            const NVGcolor pegBase = nvgRGB(0x5f, 0x6b, 0x7c);
            for (int r = 0; r < numRows; r++)
            {
                float py = sy(0.8f + 0.5f * (float)r);
                for (int k = 0; k <= r; k++)
                {
                    float px = sx((float)slots * 0.5f + ((float)k - 0.5f * (float)r));
                    float f = flash[r * GL_N + k];
                    nvgBeginPath(vg);
                    nvgCircle(vg, px, py, pegPx + 0.8f * f);
                    nvgFillColor(vg, nvgLerpRGBA(pegBase, ag::accent(), rack::math::clamp(f, 0.f, 1.f)));
                    nvgFill(vg);
                }
            }

            // Balls: short velocity streak plus a solid disc
            const float ballPx = std::max(GLWorld::BALL_R * scale, 2.2f);
            for (int i = 0; i < ballCount; i++)
            {
                float x = sx(bx[i]), y = sy(by[i]);
                float a = rack::math::clamp(ba[i], 0.f, 1.f);
                float sxv = x - bvx[i] * scale * 0.022f;
                float syv = y - bvy[i] * scale * 0.022f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, sxv, syv);
                nvgLineTo(vg, x, y);
                nvgStrokeColor(vg, ag::withA(ag::accent(), 0.35f * a));
                nvgStrokeWidth(vg, ballPx * 1.6f);
                nvgLineCap(vg, NVG_ROUND);
                nvgStroke(vg);

                nvgBeginPath(vg);
                nvgCircle(vg, x, y, ballPx);
                nvgFillColor(vg, ag::withA(ag::bright(), a));
                nvgFill(vg);
            }

            nvgRestore(vg);

            // Note names under the floor (the root, in the middle, is orange). The
            // octave is dropped when the bins get too narrow for it.
            const bool withOct = scale >= 16.f;
            for (int i = 0; i < slots; i++)
            {
                glNoteName(glSlotSemis(i, slots, root, scaleIdx), withOct, buf, sizeof(buf));
                NVGcolor col = (i == last) ? ag::bright() : (i == glMidSlot(slots) ? ag::warm() : ag::dim());
                ag::text(vg, sx((float)i + 0.5f), playB + 8.f, buf, 7.f, col, ag::ALIGN_C, 0.f);
            }

            Widget::drawLayer(args, layer);
        }
    };

    // ------------------------------------------------------------
    // GLHistDisplay - histogram, ideal counts, normal fit and statistics.
    // ------------------------------------------------------------
    struct GLHistDisplay : TransparentWidget
    {
        Galton *module = nullptr;
        GLHistDisplay(Galton *m) : module(m)
        {
            box.pos = Vec(GL_RDX, GL_RDY);
            box.size = Vec(GL_RDW, GL_RDH);
        }

        static constexpr float PLOT_L = 28.f;
        static constexpr float PLOT_R = 232.f; // GL_RDW - 8
        static constexpr float PLOT_T = 26.f;
        static constexpr float PLOT_B = 100.f;
        static constexpr float STAT_T = 118.f; // top of the statistics block

        void draw(const DrawArgs &args) override
        {
            NVGcontext *vg = args.vg;
            ag::screen(vg, 0.f, 0.f, GL_RDW, GL_RDH);

            // Hairlines: under the header, above the statistics, between cells
            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, 18.5f);
            nvgLineTo(vg, GL_RDW - 8.f, 18.5f);
            nvgStrokeColor(vg, nvgRGBA(0x3d, 0xe0, 0xff, 40));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 8.f, STAT_T - 0.5f);
            nvgLineTo(vg, GL_RDW - 8.f, STAT_T - 0.5f);
            nvgMoveTo(vg, 8.f, STAT_T + 19.5f);
            nvgLineTo(vg, GL_RDW - 8.f, STAT_T + 19.5f);
            nvgMoveTo(vg, GL_RDW * 0.5f + 0.5f, STAT_T + 3.f);
            nvgLineTo(vg, GL_RDW * 0.5f + 0.5f, STAT_T + 37.f);
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 22));
            nvgStrokeWidth(vg, 1.f);
            nvgStroke(vg);

            Widget::draw(args);
        }

        // One statistic: label on the left, value on the right.
        void statCell(NVGcontext *vg, float x, float y, float w, const char *label, const char *value)
        {
            ag::text(vg, x + 6.f, y, label, 7.f, ag::dim(), ag::ALIGN_L, 0.5f);
            ag::text(vg, x + w - 6.f, y, value, 8.f, ag::textCol(), ag::ALIGN_R, 0.3f);
        }

        void drawLayer(const DrawArgs &args, int layer) override
        {
            if (layer != 1)
            {
                Widget::drawLayer(args, layer);
                return;
            }
            NVGcontext *vg = args.vg;

            // -------- snapshot --------
            int slots = 8, last = -1;
            int counts[GL_N] = {};
            int statBalls = 0, statDof = 0;
            float statMean = 0.f, statStdDev = 0.f, statChiSq = 0.f;
            float statExpMean = 4.5f, statExpStd = 1.32f;
            float binomPMF[GL_N] = {};

            if (module)
            {
                slots = rack::math::clamp(module->dispSlots, 4, GL_N);
                last = module->dispLast;
                for (int i = 0; i < GL_N; i++)
                {
                    counts[i] = module->dispCounts[i];
                    binomPMF[i] = module->binomPMF[i];
                }
                statBalls = module->statBalls;
                statMean = module->statMean;
                statStdDev = module->statStdDev;
                statChiSq = module->statChiSq;
                statDof = module->statDof;
                statExpMean = module->statExpMean;
                statExpStd = module->statExpStd;
            }
            else
            {
                const int demo[8] = {2, 6, 11, 15, 14, 9, 5, 1};
                for (int i = 0; i < 8; i++)
                    counts[i] = demo[i];
                statBalls = 63;
                for (int i = 0; i < slots; i++)
                    binomPMF[i] = Galton::binomialPMF(i, slots - 1, 0.5f);
                statMean = 4.6f;
                statStdDev = 1.5f;
                statChiSq = 3.2f;
                statDof = slots - 1;
                statExpMean = (float)(slots + 1) * 0.5f;
                statExpStd = std::sqrt((float)(slots - 1) * 0.25f);
            }

            // -------- header + legend --------
            ag::text(vg, 8.f, 9.f, "HISTOGRAM", 8.f, ag::accent(), ag::ALIGN_L, 0.8f);

            const float ly = 9.f;
            nvgBeginPath(vg);
            nvgRect(vg, 112.f, ly - 3.f, 6.f, 6.f);
            nvgFillColor(vg, ag::withA(ag::accent(), 0.65f));
            nvgFill(vg);
            ag::text(vg, 122.f, ly, "BALLS", 7.f, ag::dim(), ag::ALIGN_L, 0.3f);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 158.f, ly);
            nvgLineTo(vg, 168.f, ly);
            nvgStrokeColor(vg, nvgRGB(255, 255, 255));
            nvgStrokeWidth(vg, 1.5f);
            nvgStroke(vg);
            ag::text(vg, 172.f, ly, "EXP", 7.f, ag::dim(), ag::ALIGN_L, 0.3f);

            nvgBeginPath(vg);
            nvgMoveTo(vg, 198.f, ly);
            nvgLineTo(vg, 208.f, ly);
            nvgStrokeColor(vg, ag::warm());
            nvgStrokeWidth(vg, 1.5f);
            nvgStroke(vg);
            ag::text(vg, 212.f, ly, "FIT", 7.f, ag::dim(), ag::ALIGN_L, 0.3f);

            // -------- plot --------
            const float plotW = PLOT_R - PLOT_L;
            const float plotH = PLOT_B - PLOT_T;

            int maxCount = 0;
            for (int i = 0; i < slots; i++)
                maxCount = std::max(maxCount, counts[i]);
            int yMax = std::max(10, (int)std::ceil((float)maxCount * 1.2f / 5.f) * 5);
            // Keep the expected counts on scale as well
            for (int i = 0; i < slots; i++)
                yMax = std::max(yMax, (int)std::ceil(statBalls * binomPMF[i] * 1.1f / 5.f) * 5);

            auto mapY = [&](float v)
            { return PLOT_B - (v / (float)yMax) * plotH; };

            // Y grid and labels
            for (int k = 0; k <= 3; k++)
            {
                float v = (float)yMax * (float)k / 3.f;
                float y = std::floor(mapY(v)) + 0.5f;
                nvgBeginPath(vg);
                nvgMoveTo(vg, PLOT_L, y);
                nvgLineTo(vg, PLOT_R, y);
                nvgStrokeColor(vg, nvgRGBA(255, 255, 255, k == 0 ? 40 : 16));
                nvgStrokeWidth(vg, 1.f);
                nvgStroke(vg);
                char buf[12];
                snprintf(buf, sizeof(buf), "%d", (int)std::round(v));
                ag::text(vg, PLOT_L - 4.f, y, buf, 7.f, ag::dim(), ag::ALIGN_R, 0.f);
            }

            nvgSave(vg);
            nvgScissor(vg, PLOT_L - 1.f, PLOT_T - 1.f, plotW + 2.f, plotH + 2.f);

            // Bars
            float binW = plotW / (float)slots;
            for (int i = 0; i < slots; i++)
            {
                float x = PLOT_L + (float)i * binW;
                float h = (float)counts[i] / (float)yMax * plotH;
                if (h < 0.3f)
                    continue;
                nvgBeginPath(vg);
                nvgRect(vg, x + 1.f, PLOT_B - h, binW - 2.f, h);
                nvgFillColor(vg, ag::withA(ag::accent(), i == last ? 0.85f : 0.55f));
                nvgFill(vg);
                nvgBeginPath(vg);
                nvgRect(vg, x + 1.f, PLOT_B - h, binW - 2.f, 1.5f);
                nvgFillColor(vg, ag::withA(ag::bright(), 0.9f));
                nvgFill(vg);
            }

            // Expected counts
            if (statBalls > 0)
            {
                for (int i = 0; i < slots; i++)
                {
                    float e = (float)statBalls * binomPMF[i];
                    if (e < 0.1f)
                        continue;
                    float y = mapY(e);
                    float x = PLOT_L + (float)i * binW;
                    nvgBeginPath(vg);
                    nvgMoveTo(vg, x + 1.f, y);
                    nvgLineTo(vg, x + binW - 1.f, y);
                    nvgStrokeColor(vg, nvgRGB(255, 255, 255));
                    nvgStrokeWidth(vg, 1.5f);
                    nvgStroke(vg);
                }
            }

            // Normal fit
            if (statBalls > 0 && statExpStd > 0.01f)
            {
                const float norm = 1.f / (statExpStd * std::sqrt(2.f * GL_PI));
                nvgBeginPath(vg);
                const int STEPS = 120;
                for (int s = 0; s <= STEPS; s++)
                {
                    float v = 0.5f + (float)s / (float)STEPS * (float)slots; // slot coordinate
                    float z = (v - statExpMean) / statExpStd;
                    float y = mapY((float)statBalls * norm * std::exp(-0.5f * z * z));
                    float x = PLOT_L + (v - 0.5f) / (float)slots * plotW;
                    if (s == 0)
                        nvgMoveTo(vg, x, y);
                    else
                        nvgLineTo(vg, x, y);
                }
                nvgStrokeColor(vg, ag::warm());
                nvgStrokeWidth(vg, 1.5f);
                nvgStroke(vg);
            }
            nvgRestore(vg);

            // X labels
            for (int i = 0; i < slots; i++)
            {
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", i + 1);
                NVGcolor col = (i == last) ? ag::bright() : (i == glMidSlot(slots) ? ag::warm() : ag::dim());
                ag::text(vg, PLOT_L + ((float)i + 0.5f) * binW, PLOT_B + 8.f, buf, 7.f, col, ag::ALIGN_C, 0.f);
            }

            // -------- statistics (2 x 2) --------
            const float colW = (GL_RDW - 16.f) * 0.5f;
            const float c1 = 8.f, c2 = 8.f + colW;
            const float r1 = STAT_T + 10.f, r2 = STAT_T + 30.f;
            char v[40];

            snprintf(v, sizeof(v), "%d", statBalls);
            statCell(vg, c1, r1, colW, "BALLS", v);

            snprintf(v, sizeof(v), "%.2f / %.2f", statMean, statExpMean);
            statCell(vg, c2, r1, colW, "MEAN", v);

            snprintf(v, sizeof(v), "%.2f / %.2f", statStdDev, statExpStd);
            statCell(vg, c1, r2, colW, "STD", v);

            snprintf(v, sizeof(v), "%.1f  DF %d", statChiSq, statDof);
            statCell(vg, c2, r2, colW, "CHI2", v);

            Widget::drawLayer(args, layer);
        }
    };

} // namespace

// ============================================================================
// ModuleWidget
// ============================================================================
struct GaltonWidget : ModuleWidget
{
    GaltonWidget(Galton *module)
    {
        setModule(module);
        box.size = Vec(GL_W, GL_H);

        addChild(new GLPanel());
        addChild(new GLBoardDisplay(module));
        addChild(new GLHistDisplay(module));

        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, box.size.y - RACK_GRID_WIDTH)));

        // Row A: SPEED, SLOTS, RATE, BOUNCE, JITTER, TILT, ROOT, SCALE, POLY, DROP, CLEAR
        const float ya = GL_RA_TOP + GL_CTRL_DY;
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(0, 11), ya), module, Galton::SPEED_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(1, 11), ya), module, Galton::SLOTS_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(2, 11), ya), module, Galton::RATE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(3, 11), ya), module, Galton::BOUNCE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(4, 11), ya), module, Galton::JITTER_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(5, 11), ya), module, Galton::TILT_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(6, 11), ya), module, Galton::ROOT_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(7, 11), ya), module, Galton::SCALE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(Vec(glCellX(8, 11), ya), module, Galton::POLY_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(glCellX(9, 11), ya), module, Galton::DROP_PARAM));
        addParam(createParamCentered<LEDButton>(Vec(glCellX(10, 11), ya), module, Galton::CLEAR_PARAM));

        // Row B: DROP IN, CLEAR IN, TILT CV | GATES, SLOT CV, LAND, HIT, ROW CV, V/OCT, V GATE
        const float yb = GL_RB_TOP + GL_CTRL_DY;
        addInput(createInputCentered<DarkPJ301MPort>(Vec(glCellX(0, 10), yb), module, Galton::DROP_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(glCellX(1, 10), yb), module, Galton::CLEAR_INPUT));
        addInput(createInputCentered<DarkPJ301MPort>(Vec(glCellX(2, 10), yb), module, Galton::TILT_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(glCellX(3, 10), yb), module, Galton::GATES_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(glCellX(4, 10), yb), module, Galton::CV_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(glCellX(5, 10), yb), module, Galton::LAND_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(glCellX(6, 10), yb), module, Galton::HIT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(glCellX(7, 10), yb), module, Galton::ROW_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(glCellX(8, 10), yb), module, Galton::VOCT_OUTPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(Vec(glCellX(9, 10), yb), module, Galton::VGATE_OUTPUT));
        agmm::addKnobArcs(this, 2.f);
        // Slot LEDs
        for (int i = 0; i < GL_N; i++)
        {
            float x = GL_LED_X0 + (GL_LED_X1 - GL_LED_X0) * (float)i / (float)(GL_N - 1);
            addChild(createLightCentered<SmallLight<ag::CyanLight>>(
                Vec(x, GL_LED_Y), module, Galton::SLOT_LIGHT + i));
        }
    }

    void appendContextMenu(Menu *menu) override
    {
        ModuleWidget::appendContextMenu(menu);
        Galton *module = dynamic_cast<Galton *>(this->module);
        if (!module)
            return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createSubmenuItem("Gate duration", "", [=](Menu *sub)
                                         {
            for (int i = 0; i < NUM_GATE_PRESETS; i++) {
                sub->addChild(createCheckMenuItem(GATE_PRESET_NAMES[i], "",
                    [=]() { return module->gateIdx == i; },
                    [=]() {
                        module->gateIdx = i;
                        module->gateDuration = GATE_PRESET_VALUES[i];
                    }));
            } }));
        menu->addChild(createBoolPtrMenuItem("Bipolar CV outputs (-5 to +5 V)", "", &module->cvBipolar));
    }
};

Model *modelGalton = createModel<Galton, GaltonWidget>("Galton");