#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Display module — round 1.28" LCD + capacitive touch (Waveshare GC9A01)
//
// ALL screen code lives here. When ENABLE_SCREEN == 0 every function is an
// empty inline no-op, so callers never need their own #if guard.
//
// PARALLEL / NON-BLOCKING + ATOMIC FRAMES: the frame is rendered into an
// off-screen framebuffer one horizontal band at a time (so no single loop pass
// blocks the pads/audio), all bands drawn from ONE per-frame snapshot of the UI
// state. Only when the frame is complete is it streamed to the LCD in a single
// background SPI-DMA transfer — the panel flips at once instead of updating in
// visible strips. Two framebuffers: render into one while the other streams.
// The SPI bus is dedicated to the LCD (audio = I2S, LEDs/touch = I2C), so the
// DMA never competes with them.
//
// Look: white-on-black; the background glows from black toward white as the
// average pad intensity rises (all pads maxed → white). Arrows and (debug) tap
// boxes are rendered straight into the strip buffer too, so there's zero
// per-pixel SPI anywhere in the steady state.
//
// Layout (top→bottom): title · MODE · SCALE (tone set, centred) · TIMBRE.
// ─────────────────────────────────────────────────────────────────────────────

#include "config.h"

// Set 1 to overlay the tap zones as coloured boxes and print every touch x/y to
// Serial (for aligning the touch coordinates with the on-screen press areas).
#define SCREEN_TOUCH_DEBUG 0

// Set 1 to render the screen rotated 180° (panel mounted upside down). Touch
// coordinates are flipped to match.
#define SCREEN_FLIP 1

enum DisplayGesture : uint8_t {
    GESTURE_NONE = 0, GESTURE_UP, GESTURE_DOWN, GESTURE_LEFT, GESTURE_RIGHT,
    GESTURE_TAP,        // a single tap (tapX/tapY out-params are set)
    GESTURE_LONGPRESS,  // a long press → back to the atlas picker (harmonic mode)
    GESTURE_MODE_PREV,  // ‹ on the mode line → previous play mode
    GESTURE_MODE_NEXT   // › on the mode line → next play mode
};

#if ENABLE_SCREEN

#include <Arduino.h>
#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include <EventResponder.h>
#include "LCD_Driver.h"
#include "Touch_Driver.h"
#include "GUI_Paint.h"

// ── Layout: title(/param) / mode / scale (centred) / timbre / stats ──────────
static constexpr uint16_t TITLE_T  = 10,  TITLE_H  = 26;
static constexpr uint16_t MODE_T   = 54,  MODE_H   = 30;
static constexpr uint16_t SCALE_T  = 92,  SCALE_H  = 52;  // centred on ~118 ≈ screen mid
static constexpr uint16_t TIMBRE_T = 156, TIMBRE_H = 48;
static constexpr uint16_t STATS_T  = 206, STATS_H  = 18;  // small cpu/mem readout, bottom middle
enum { LINE_TITLE = 0, LINE_MODE, LINE_SCALE, LINE_TIMBRE, LINE_STATS, LINE_COUNT };

static constexpr uint16_t BAND_ROWS    = 20;           // rows rendered per render step (bounds CPU per pass)
static constexpr uint32_t SCREEN_SPI_HZ = 40000000;    // GC9A01 SPI clock. DO NOT OVERCLOCK: 60 MHz
                                                       // permanently froze the I2S/SAI1 audio output within
                                                       // ~1.5 s of the first full-frame DMA flips (audio update
                                                       // ISR never fires again; loop/sensors/screen keep
                                                       // running). Verified by RMS-tap bisection 2026-07-03:
                                                       // 60 MHz → dead, 40 MHz → stable. 40 MHz is the
                                                       // proven-safe-alongside-audio value.

namespace {
static const uint16_t LINE_Y[LINE_COUNT] = { TITLE_T, MODE_T, SCALE_T, TIMBRE_T, STATS_T };
static const uint16_t LINE_H[LINE_COUNT] = { TITLE_H, MODE_H, SCALE_H, TIMBRE_H, STATS_H };
inline sFONT* lineFont(uint8_t i) {
    if (i == LINE_STATS) return &Font16;   // (the lib ships no Font12 definition)
    return (i == LINE_TITLE || i == LINE_SCALE) ? &Font20 : &Font16;
}

// Tap targets, one box per arrow — sized to where presses actually land on this
// round panel (left/right at the sides, up/down stacked in the centre-lower
// area). Edges = scale, centre column = timbre, so they never overlap.
struct TapBox { uint16_t x0, y0, x1, y1; DisplayGesture act; UWORD dbg; };
static const TapBox TAP_BOXES[6] = {
    {   0,  40,  58,  83, GESTURE_MODE_PREV, CYAN    }, // ‹ mode-  (up by the mode text)
    { 182,  40, 239,  83, GESTURE_MODE_NEXT, MAGENTA }, // › mode+
    {   0,  85,  58, 155, GESTURE_LEFT,  RED    }, // ‹ scale-
    { 182,  85, 239, 155, GESTURE_RIGHT, GREEN  }, // › scale+
    {  60, 118, 179, 193, GESTURE_UP,    BLUE   }, // ▲ timbre-
    {  60, 194, 179, 239, GESTURE_DOWN,  YELLOW }, // ▼ timbre+
};

static char    g_text[LINE_COUNT][20] = { {0} };
static uint8_t g_bgLevel = 0; // background grey 0 (black) … 255 (white)
static bool    g_timbreUsed = true; // false → hide the timbre line + ▲▼ (mode ignores it)

// Lock / visualiser mode: no text, just the glow + a fuzzy 12-pad pie.
static bool  g_locked = false;
static float g_pad[NUM_SENSORS]      = { 0 }; // per-pad intensity (0..1)
static float g_padAngle[NUM_SENSORS] = { 0 }; // each pad's wedge centre angle

// ── Harmonic Journey mode ────────────────────────────────────────────────────
// A picker of four "atlases" (see config.h HARMONIC_JOURNEYS), then an
// interactive network of chord nodes for the chosen atlas. Tap a node to retune
// the pads; the bottom strip shows the move's emotional label. Node positions
// are computed per-journey: node 0 at the centre (home) with the rest on a ring,
// or all nodes on a ring. Concept: William R Thomas / atlas protocols (2025).
static bool    g_harmonicMode  = false;
static bool    g_picking       = false; // true → show the atlas picker, not a map
static uint8_t g_harmonicChord = 0;     // currently selected node (0 … nNodes-1)

static uint8_t g_hjIdx   = 0;           // active journey index
static uint8_t g_hNodes  = 7;           // node count for the active journey
static bool    g_hCenter = true;        // node 0 centred (home) vs. all-on-ring
static int16_t g_hnodeX[HARMONIC_MAX_NODES] = { 0 };
static int16_t g_hnodeY[HARMONIC_MAX_NODES] = { 0 };

static constexpr int16_t HNODE_R  = 20; // node circle radius (px)
// Tap hit-test radius — wider than the drawn node for an easier target. Capped
// by the tightest ring spacing: Jazz has 7 ring nodes (8 total, centerFirst) at
// R=72, giving ~62.5 px between neighbours, so two 30 px hit-circles (60 px
// apart) still don't overlap. Don't raise this past ~31 without re-checking
// HARMONIC_JOURNEYS for a denser atlas.
static constexpr int16_t HTOUCH_R = 30;
static constexpr int16_t HTOP_BAND = 36; // tap above this y in a map → back to picker

// Picker row geometry (5 atlas names stacked under a title).
static constexpr int16_t HPICK_Y0 = 70;  // centre-y of the first row
static constexpr int16_t HPICK_DY = 32;  // row spacing

// Atomic-frame repaint state. Bands render top-to-bottom into the off-screen
// buffer; nothing reaches the panel until the whole frame is done, so the order
// no longer shows. Double-buffered: render into g_fb[g_fbRender] while the
// other buffer streams out by DMA.
static constexpr uint8_t  N_BANDS  = (uint8_t)((LCD_HEIGHT + BAND_ROWS - 1) / BAND_ROWS);
static constexpr uint32_t FB_BYTES = (uint32_t)LCD_WIDTH * LCD_HEIGHT * 2;
DMAMEM static uint8_t g_fb[2][FB_BYTES]; // OCRAM so the SPI DMA can reach it
static uint8_t  g_fbRender = 0;      // buffer being rendered into
static uint8_t  g_band     = 0;      // next band to render (0 = idle / frame start)
static bool     g_drawn    = true;   // screen content matches the target → idle

// Per-frame snapshot of everything the pixel composers read, taken once at the
// top of each frame so every band comes from the same instant — no intra-frame
// shear (that shear is what used to read as "blocks").
static uint8_t g_snapLevel = 0;
static float   g_snapPad[NUM_SENSORS] = { 0 };
static char    g_snapText[LINE_COUNT][20] = { {0} };

static EventResponder g_spiEvent;
static volatile bool  g_dmaBusy = false;

inline UWORD grey565(uint8_t v) { return (UWORD)(((v >> 3) << 11) | ((v >> 2) << 5) | (v >> 3)); }

inline bool glyphPixel(uint16_t x, uint16_t y, uint16_t y0, uint16_t h, const char* s, sFONT* f)
{
    uint16_t len = (uint16_t)strlen(s);
    if (!len) return false;
    uint16_t textW = (uint16_t)(len * f->Width);
    uint16_t x0    = (LCD_WIDTH > textW) ? (uint16_t)((LCD_WIDTH - textW) / 2) : 0;
    uint16_t textY = (h > f->Height) ? (uint16_t)(y0 + (h - f->Height) / 2) : y0;
    if (y < textY || y >= textY + f->Height) return false;
    if (x < x0    || x >= x0 + textW)        return false;
    uint16_t col = (uint16_t)((x - x0) % f->Width);
    uint16_t bpr = (uint16_t)(f->Width / 8 + (f->Width % 8 ? 1 : 0));
    const uint8_t* gp = &f->table[(s[(x - x0) / f->Width] - ' ') * f->Height * bpr
                                  + (y - textY) * bpr + col / 8];
    return (*gp & (0x80 >> (col % 8))) != 0;
}

// Filled triangle arrows (rendered straight into the buffer — no SPI calls).
inline bool inArrows(uint16_t x, uint16_t y)
{
    const int r = 7;
    int scy = SCALE_T + SCALE_H / 2;          // scale band centre row
    // LEFT arrow at x≈14, RIGHT at x≈W-14.
    auto tri = [&](int cx, int cy, char dir)->bool {
        int dx = (int)x - cx, dy = (int)y - cy;
        switch (dir) {
            case 'L': return (dx >= -r && dx <= r && (dy<0?-dy:dy) * 2 <= (dx + r));
            case 'R': return (dx >= -r && dx <= r && (dy<0?-dy:dy) * 2 <= (r - dx));
            case 'U': return (dy >= -r && dy <= r && (dx<0?-dx:dx) * 2 <= (dy + r));
            case 'D': return (dy >= -r && dy <= r && (dx<0?-dx:dx) * 2 <= (r - dy));
        } return false;
    };
    int mcy = MODE_T + MODE_H / 2;            // mode band centre row
    if (tri(14, mcy, 'L') || tri(LCD_WIDTH - 14, mcy, 'R')) return true; // mode ‹ ›
    if (tri(14, scy, 'L') || tri(LCD_WIDTH - 14, scy, 'R')) return true; // scale ‹ ›
    if (g_timbreUsed) {
        if (tri(LCD_WIDTH / 2, TIMBRE_T + 9, 'U'))            return true;
        if (tri(LCD_WIDTH / 2, TIMBRE_T + TIMBRE_H - 9, 'D')) return true;
    }
    return false;
}

#if SCREEN_TOUCH_DEBUG
// Outline each tap box (debug). Returns true + colour if (x,y) is on a box edge.
inline bool debugBoxEdge(uint16_t x, uint16_t y, UWORD& c)
{
    for (const TapBox& b : TAP_BOXES)
        if (((x == b.x0 || x == b.x1) && y >= b.y0 && y <= b.y1) ||
            ((y == b.y0 || y == b.y1) && x >= b.x0 && x <= b.x1)) { c = b.dbg; return true; }
    return false;
}
#endif

// Text centered at an arbitrary pixel point (used by the harmonic node labels).
inline bool glyphPixelAt(uint16_t x, uint16_t y, int16_t cx, int16_t cy,
                         const char* s, sFONT* f)
{
    uint16_t len = (uint16_t)strlen(s);
    if (!len) return false;
    int16_t tw = (int16_t)((uint16_t)len * f->Width);
    int16_t x0 = cx - tw / 2;
    int16_t y0 = cy - (int16_t)f->Height / 2;
    if ((int16_t)x < x0 || (int16_t)x >= x0 + tw)                  return false;
    if ((int16_t)y < y0 || (int16_t)y >= y0 + (int16_t)f->Height)  return false;
    uint16_t col = (uint16_t)((int16_t)x - x0) % f->Width;
    uint16_t bpr = f->Width / 8 + (f->Width % 8 ? 1 : 0);
    const uint8_t* gp = &f->table[
        (s[((int16_t)x - x0) / f->Width] - ' ') * f->Height * bpr
        + (uint16_t)((int16_t)y - y0) * bpr + col / 8];
    return (*gp & (0x80 >> (col % 8))) != 0;
}

// Point-in-regular-polygon test: n sides (3=triangle … 6=hexagon), circumradius
// R, first vertex pointing straight up (same convention as the ring layout in
// displaySetHarmonicJourney). Convex polygon → inside iff on the interior side
// of every edge's half-plane.
inline bool insidePolygon(float dx, float dy, uint8_t n, float R)
{
    float a0 = -(float)M_PI / 2.0f;
    float x0 = R * cosf(a0), y0 = R * sinf(a0);
    for (uint8_t k = 1; k <= n; k++) {
        float a1 = -(float)M_PI / 2.0f + (float)k * 2.0f * (float)M_PI / (float)n;
        float x1 = R * cosf(a1), y1 = R * sinf(a1);
        float ex = x1 - x0, ey = y1 - y0;
        float px = dx - x0, py = dy - y0;
        if (ex * py - ey * px < 0.0f) return false;
        x0 = x1; y0 = y1;
    }
    return true;
}

// Is (dx,dy) — relative to a node's centre — inside the icon for harmonic
// function `fn`, drawn at "radius" R? Replaces the old chord-name glyph: the
// SHAPE now carries the function, colour (moodColor/tmColor) carries the mood.
inline bool insideFunctionShape(float dx, float dy, uint8_t fn, float R)
{
    switch (fn) {
        case HFN_TONIC:       return dx * dx + dy * dy <= R * R;        // circle
        case HFN_SUBDOMINANT: return insidePolygon(dx, dy, 3, R * 1.15f); // triangle
        case HFN_DOMINANT:    return insidePolygon(dx, dy, 4, R * 0.95f); // square
        case HFN_CHROMATIC:   return insidePolygon(dx, dy, 5, R);         // pentagon
        case HFN_SYMMETRIC:   return insidePolygon(dx, dy, 6, R * 1.05f); // hexagon
        default:              return false;
    }
}

// Per-mood node colour (see HarmonicMood in config.h), scaled by brightness b
// (0…1) — same pattern as tmColor's per-type palette below.
inline UWORD moodColor(uint8_t mood, float b)
{
    static const float RGB[5][3] = {
        { 1.00f, 0.70f, 0.25f },  // HMOOD_WARM       — amber
        { 1.00f, 0.25f, 0.25f },  // HMOOD_TENSION    — red
        { 0.30f, 0.45f, 1.00f },  // HMOOD_MELANCHOLY — blue
        { 0.75f, 0.35f, 1.00f },  // HMOOD_MYSTERY    — purple
        { 0.35f, 0.90f, 0.55f },  // HMOOD_GROUNDED   — green
    };
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    uint8_t r  = (uint8_t)(RGB[mood][0] * b * 255.0f);
    uint8_t g  = (uint8_t)(RGB[mood][1] * b * 255.0f);
    uint8_t bl = (uint8_t)(RGB[mood][2] * b * 255.0f);
    return (UWORD)(((r >> 3) << 11) | ((g >> 2) << 5) | (bl >> 3));
}

// Distance from pixel (x,y) to line segment (ax,ay)→(bx,by), returns true if ≤ tol.
inline bool nearSeg(int16_t x, int16_t y,
                    int16_t ax, int16_t ay, int16_t bx, int16_t by, float tol)
{
    float dx=(float)(bx-ax), dy=(float)(by-ay), l2=dx*dx+dy*dy;
    if (l2 < 1.0f) return false;
    float t = ((float)(x-ax)*dx + (float)(y-ay)*dy) / l2;
    if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
    float px=(float)ax+t*dx-(float)x, py=(float)ay+t*dy-(float)y;
    return px*px+py*py <= tol*tol;
}

inline void markDirty(); // defined below (after the pixel composers)

// ── Tonal Map (the dynamic-graph journey, index JOURNEY_TONAL_MAP) ───────────
// The current chord sits at the centre; its legal moves (see tonalmap.h) form
// an option ring, and each option's own moves show as a dim labelled preview
// fan further out. Tapping an option re-centres the map: shared nodes glide to
// their new spots, new ones fade in, dropped ones fade out in place.
static constexpr uint8_t  TM_MAX_VIS = 48;    // layout targets (≤21) + fade-outs
static constexpr uint32_t TM_ANIM_MS = 400;   // relayout glide time
static constexpr float    TM_R1      = 64.0f; // option ring radius
static constexpr float    TM_R2      = 97.0f; // preview fan radius
static constexpr float    TM_CR      = 22.0f; // centre node circle radius
static constexpr float    TM_OR      = 17.0f; // option node circle radius

struct TMVisNode {
    uint8_t node;          // tonal map node id (0…47)
    float x0, y0, s0, b0;  // animation start: position, circle radius, brightness
    float x1, y1, s1, b1;  // animation target
    float cx, cy, cs, cb;  // current (interpolated) values — what gets drawn
};
static TMVisNode g_tmVis[TM_MAX_VIS];
static uint8_t   g_tmNVis  = 0;  // total entries (targets first, then fade-outs)
static uint8_t   g_tmNKeep = 0;  // target entries that survive after the anim

struct TMVisEdge { uint8_t a, b; bool dotted; bool dim; };
static TMVisEdge g_tmEdge[TM_MAX_VIS];
static uint8_t   g_tmNEdge = 0;

static uint32_t  g_tmAnimStart  = 0; // millis() when the glide began; 0 = idle
static uint8_t   g_tmCenterNode = 0;
static uint8_t   g_tmOptNode[TONAL_MAX_MOVES];  // option-ring tap targets
static int16_t   g_tmOptX[TONAL_MAX_MOVES], g_tmOptY[TONAL_MAX_MOVES];
static uint8_t   g_tmNOpt = 0;

// Per-type node colour (major pink / minor blue / dom7 amber / aug green, as
// on the source diagram), scaled by brightness b (0…1).
inline UWORD tmColor(uint8_t type, float b)
{
    static const float RGB[4][3] = {
        { 1.00f, 0.35f, 0.55f },  // TM_MAJ  — pink
        { 0.35f, 0.65f, 1.00f },  // TM_MIN  — blue
        { 1.00f, 0.80f, 0.20f },  // TM_DOM7 — amber
        { 0.40f, 1.00f, 0.45f },  // TM_AUG  — green
    };
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    uint8_t r  = (uint8_t)(RGB[type][0] * b * 255.0f);
    uint8_t g  = (uint8_t)(RGB[type][1] * b * 255.0f);
    uint8_t bl = (uint8_t)(RGB[type][2] * b * 255.0f);
    return (UWORD)(((r >> 3) << 11) | ((g >> 2) << 5) | (bl >> 3));
}

// The Tonal Map has no per-node function/mood data (it's a generated 56-chord
// graph, not a hand-authored atlas) — so its icon shape is approximated from
// the chord TYPE alone: major reads as the stable/tonic shape, minor as the
// departure shape, dominant 7th as the tension/resolving shape, and augmented
// as the symmetric shape. Colour still comes from tmColor's per-type palette.
inline uint8_t tmFunction(uint8_t type)
{
    switch (type) {
        case TM_MIN:  return HFN_SUBDOMINANT;
        case TM_DOM7: return HFN_DOMINANT;
        case TM_AUG:  return HFN_SYMMETRIC;
        default:      return HFN_TONIC; // TM_MAJ
    }
}

// nearSeg with a bounding-box early-out plus the hit's parameter t and the
// segment length — needed to dash dotted edges and clip them at the nodes.
inline bool nearSegT(uint16_t x, uint16_t y, float ax, float ay,
                     float bx, float by, float tol, float& tOut, float& lenOut)
{
    float lo = (ax < bx ? ax : bx) - tol, hi = (ax > bx ? ax : bx) + tol;
    if ((float)x < lo || (float)x > hi) return false;
    lo = (ay < by ? ay : by) - tol; hi = (ay > by ? ay : by) + tol;
    if ((float)y < lo || (float)y > hi) return false;
    float dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    if (l2 < 1.0f) return false;
    float t = (((float)x - ax) * dx + ((float)y - ay) * dy) / l2;
    if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
    float px = ax + t * dx - (float)x, py = ay + t * dy - (float)y;
    if (px * px + py * py > tol * tol) return false;
    tOut = t; lenOut = sqrtf(l2);
    return true;
}

// Re-centre the map on `node` and (optionally) start the relayout glide.
inline void tmLayout(uint8_t node, bool animate)
{
    // Snapshot the outgoing visual set at its current interpolated state.
    static TMVisNode old[TM_MAX_VIS];
    uint8_t nOld = g_tmNVis;
    memcpy(old, g_tmVis, sizeof(TMVisNode) * nOld);
    bool used[TM_MAX_VIS] = { false };

    g_tmCenterNode = node;
    g_tmNVis = 0; g_tmNEdge = 0; g_tmNOpt = 0;

    // Add a target node: glide from its old spot if it was already visible,
    // otherwise fade in at its destination.
    auto place = [&](uint8_t id, float x, float y, float s, float b) -> uint8_t {
        TMVisNode &v = g_tmVis[g_tmNVis];
        v.node = id; v.x1 = x; v.y1 = y; v.s1 = s; v.b1 = b;
        v.x0 = x; v.y0 = y; v.s0 = s * 0.4f; v.b0 = 0.0f;
        for (uint8_t i = 0; i < nOld; i++)
            if (!used[i] && old[i].node == id) {
                used[i] = true;
                v.x0 = old[i].cx; v.y0 = old[i].cy;
                v.s0 = old[i].cs; v.b0 = old[i].cb;
                break;
            }
        v.cx = v.x0; v.cy = v.y0; v.cs = v.s0; v.cb = v.b0;
        return g_tmNVis++;
    };

    const float cx = 120.0f, cy = 120.0f;
    uint8_t centreIdx = place(node, cx, cy, TM_CR, 1.0f);

    TonalMove mv[TONAL_MAX_MOVES];
    uint8_t n = tonalMapMoves(node, mv);
    for (uint8_t i = 0; i < n; i++) {
        float a  = -(float)M_PI / 2.0f + (float)i * 2.0f * (float)M_PI / (float)n;
        float ox = cx + TM_R1 * cosf(a), oy = cy + TM_R1 * sinf(a);
        uint8_t oi = place(mv[i].node, ox, oy, TM_OR, 1.0f);
        g_tmEdge[g_tmNEdge++] = { centreIdx, oi, mv[i].dotted, false };
        g_tmOptNode[g_tmNOpt] = mv[i].node;
        g_tmOptX[g_tmNOpt]    = (int16_t)(ox + 0.5f);
        g_tmOptY[g_tmNOpt]    = (int16_t)(oy + 0.5f);
        g_tmNOpt++;

        // Preview fan: the option's own moves, dim, spread around its angle.
        // The move pointing straight back at the new centre is skipped.
        TonalMove cm[TONAL_MAX_MOVES];
        uint8_t m = tonalMapMoves(mv[i].node, cm), total = 0, shown = 0;
        for (uint8_t j = 0; j < m; j++) if (cm[j].node != node) total++;
        for (uint8_t j = 0; j < m; j++) {
            if (cm[j].node == node) continue;
            float ca = a + ((float)shown - (float)(total - 1) / 2.0f) * 0.42f;
            uint8_t ci = place(cm[j].node, cx + TM_R2 * cosf(ca),
                               cy + TM_R2 * sinf(ca), 0.0f, 0.45f);
            g_tmEdge[g_tmNEdge++] = { oi, ci, cm[j].dotted, true };
            shown++;
        }
    }
    g_tmNKeep = g_tmNVis;

    if (animate) {
        // Nodes that vanished from the layout fade out where they are.
        for (uint8_t i = 0; i < nOld && g_tmNVis < TM_MAX_VIS; i++) {
            if (used[i] || old[i].cb <= 0.02f) continue;
            TMVisNode &v = g_tmVis[g_tmNVis++];
            v.node = old[i].node;
            v.x0 = v.x1 = v.cx = old[i].cx;
            v.y0 = v.y1 = v.cy = old[i].cy;
            v.s0 = v.cs = old[i].cs; v.s1 = old[i].cs * 0.4f;
            v.b0 = v.cb = old[i].cb; v.b1 = 0.0f;
        }
        g_tmAnimStart = millis();
    } else {
        for (uint8_t i = 0; i < g_tmNVis; i++) {
            TMVisNode &v = g_tmVis[i];
            v.x0 = v.cx = v.x1; v.y0 = v.cy = v.y1;
            v.s0 = v.cs = v.s1; v.b0 = v.cb = v.b1;
        }
        g_tmAnimStart = 0;
    }
    markDirty();
}

// Advance the relayout glide (called at the top of each render step). Keeps
// the screen repainting until the glide lands, then drops the faded nodes.
inline void tmAnimTick()
{
    if (g_tmAnimStart == 0) return;
    float t = (float)(millis() - g_tmAnimStart) / (float)TM_ANIM_MS;
    if (t >= 1.0f) {
        for (uint8_t i = 0; i < g_tmNKeep; i++) {
            TMVisNode &v = g_tmVis[i];
            v.cx = v.x1; v.cy = v.y1; v.cs = v.s1; v.cb = v.b1;
        }
        g_tmNVis = g_tmNKeep;  // fade-outs live at the tail — safe to drop
        g_tmAnimStart = 0;
    } else {
        float e = t * t * (3.0f - 2.0f * t); // smoothstep ease
        for (uint8_t i = 0; i < g_tmNVis; i++) {
            TMVisNode &v = g_tmVis[i];
            v.cx = v.x0 + (v.x1 - v.x0) * e;
            v.cy = v.y0 + (v.y1 - v.y0) * e;
            v.cs = v.s0 + (v.s1 - v.s0) * e;
            v.cb = v.b0 + (v.b1 - v.b0) * e;
        }
    }
    markDirty();
}

// Tonal Map pixel: coloured chord circles + solid/dotted spokes + dim preview
// labels. Draw order (priority): circled nodes › title › preview labels ›
// edges › timbre strip › background.
inline UWORD tonalMapPixel(uint16_t x, uint16_t y)
{
    // Circled nodes (centre + option ring).
    for (uint8_t i = 0; i < g_tmNVis; i++) {
        const TMVisNode &v = g_tmVis[i];
        if (v.cs < 6.0f || v.cb <= 0.02f) continue;
        float dx = (float)x - v.cx, dy = (float)y - v.cy;
        if (dx < -v.cs || dx > v.cs || dy < -v.cs || dy > v.cs) continue;
        float r2 = dx * dx + dy * dy;
        if (r2 > v.cs * v.cs) continue;
        uint8_t ty  = tmType(v.node);
        bool    ctr = (v.s1 >= TM_CR - 0.5f);      // this node is (becoming) the centre
        if (r2 >= (v.cs - 2.0f) * (v.cs - 2.0f)) return tmColor(ty, v.cb);
        if (insideFunctionShape((float)x - v.cx, (float)y - v.cy,
                                 tmFunction(ty), v.cs * 0.55f))
            return ctr ? BLACK : grey565((uint8_t)(230.0f * v.cb));
        return ctr ? tmColor(ty, 0.72f * v.cb) : tmColor(ty, 0.13f * v.cb);
    }

    // Title (tap the top band / long-press → back to the atlas picker).
    if (glyphPixelAt(x, y, 120, 16, "TONAL MAP", &Font16)) return grey565(110);

    // Preview icons (circle-less nodes) — dim, in their type colour. These
    // have no drawn background disc (s1 = 0), so the icon is drawn straight
    // at a small fixed radius rather than scaled off v.cs.
    for (uint8_t i = 0; i < g_tmNVis; i++) {
        const TMVisNode &v = g_tmVis[i];
        if (v.cs >= 6.0f || v.cb <= 0.02f) continue;
        if (insideFunctionShape((float)x - v.cx, (float)y - v.cy,
                                 tmFunction(tmType(v.node)), 6.0f))
            return tmColor(tmType(v.node), 0.85f * v.cb);
    }

    // Edges (clipped at the node circles/labels; dotted = 4 px dashes).
    for (uint8_t i = 0; i < g_tmNEdge; i++) {
        const TMVisEdge &E = g_tmEdge[i];
        const TMVisNode &A = g_tmVis[E.a], &B = g_tmVis[E.b];
        float t, len;
        if (!nearSegT(x, y, A.cx, A.cy, B.cx, B.cy, 1.2f, t, len)) continue;
        float d  = t * len;
        float bs = (B.cs > 9.0f) ? B.cs : 9.0f;  // labels get a 9 px clear zone
        if (d < A.cs + 3.0f || len - d < bs + 3.0f) continue;
        if (E.dotted && (((int)d / 4) & 1)) continue;
        float b = (A.cb < B.cb ? A.cb : B.cb) * (E.dim ? 0.5f : 1.0f);
        return grey565((uint8_t)(80.0f * b));
    }

    // Current TIMBRE at the bottom, flanked by ‹ › swipe hints (same as the
    // fixed-atlas maps).
    {
        const int16_t ty = 222, ar = 6;
        const int16_t lax = 64, rax = 176;
        int16_t dl = (int16_t)x - lax, dr = (int16_t)x - rax, dy = (int16_t)y - ty;
        int16_t ay = dy < 0 ? (int16_t)-dy : dy;
        if (dl >= -ar && dl <= ar && ay * 2 <= (dl + ar)) return grey565(150); // ‹
        if (dr >= -ar && dr <= ar && ay * 2 <= (ar - dr)) return grey565(150); // ›
        if (g_snapText[LINE_TIMBRE][0] &&
            glyphPixelAt(x, y, 120, ty, g_snapText[LINE_TIMBRE], &Font16))
            return grey565(170);
    }

    return grey565(8);
}

// Is the edge a→b part of the active journey's network? Center-first journeys
// draw spokes from node 0 to every ring node plus the ring; ring journeys draw
// only the ring. (Generated from g_hCenter — no per-journey edge table.)
inline bool harmonicEdge(uint8_t a, uint8_t b)
{
    uint8_t start = g_hCenter ? 1 : 0;       // first ring node index
    uint8_t ringN = (uint8_t)(g_hNodes - start);
    if (g_hCenter && (a == 0 || b == 0))     // spoke to the centre
        return true;
    if (a < start || b < start) return false;
    // Adjacent on the ring (consecutive, with wrap)?
    uint8_t ia = (uint8_t)(a - start), ib = (uint8_t)(b - start);
    uint8_t d = (uint8_t)((ia + ringN - ib) % ringN);
    return (d == 1 || d == ringN - 1);
}

// Atlas picker: a title plus the four journey names stacked vertically; the
// active journey is shown bright, the others dim.
inline UWORD harmonicPickerPixel(uint16_t x, uint16_t y)
{
    int16_t dx0=(int16_t)x-120, dy0=(int16_t)y-120;
    if ((int32_t)dx0*dx0+(int32_t)dy0*dy0 > 119*119) return 0; // outside round face

    if (glyphPixelAt(x, y, 120, 40, "JOURNEY", &Font16)) return grey565(120);
    for (uint8_t i = 0; i < NUM_HARMONIC_JOURNEYS; i++) {
        int16_t cy = (int16_t)(HPICK_Y0 + i * HPICK_DY);
        if (glyphPixelAt(x, y, 120, cy, HARMONIC_JOURNEYS[i].name, &Font16))
            return (i == g_hjIdx) ? WHITE : grey565(90);
    }
    return grey565(8);
}

// Harmonic Journey pixel: the active atlas's chord nodes in a star/ring network
// on a dark background; selected node highlighted (bright fill, black text).
// Journey title at top (tap to return to the picker), move label at the bottom.
// Inspired by the tonal map diagram from William R Thomas / atlas protocols.
inline UWORD harmonicPixel(uint16_t x, uint16_t y)
{
    if (g_picking) return harmonicPickerPixel(x, y);

    int16_t dx0=(int16_t)x-120, dy0=(int16_t)y-120;
    if ((int32_t)dx0*dx0+(int32_t)dy0*dy0 > 119*119) return 0; // outside round face

    if (g_hjIdx == JOURNEY_TONAL_MAP) return tonalMapPixel(x, y);

    const HarmonicJourney &J = HARMONIC_JOURNEYS[g_hjIdx];

    // Nodes (highest priority). Each node is a mood-coloured disc (see
    // config.h HarmonicMood) with a small function icon (HarmonicFunction —
    // circle/triangle/square/pentagon/hexagon) instead of the chord's name.
    for (uint8_t n = 0; n < g_hNodes; n++) {
        int16_t nx=(int16_t)x-g_hnodeX[n], ny=(int16_t)y-g_hnodeY[n];
        int32_t r2=(int32_t)nx*nx+(int32_t)ny*ny;
        if (r2 <= (int32_t)HNODE_R*HNODE_R) {
            bool sel = (n == g_harmonicChord);
            if (r2 >= (int32_t)(HNODE_R-2)*(HNODE_R-2))
                return sel ? WHITE : grey565(110);   // ring border
            const HarmonicChord &C = J.chords[n];
            if (insideFunctionShape((float)nx, (float)ny, C.function, (float)HNODE_R * 0.55f))
                return sel ? BLACK : WHITE;           // function icon (inverted on selected)
            return moodColor(C.mood, sel ? 1.0f : 0.45f); // mood fill
        }
    }

#if SCREEN_TOUCH_DEBUG
    // Debug: thin red ring at HTOUCH_R (the actual tap hit-test radius, wider
    // than the drawn node) so the real hit area can be checked by eye instead
    // of guessed at.
    for (uint8_t n = 0; n < g_hNodes; n++) {
        int16_t nx=(int16_t)x-g_hnodeX[n], ny=(int16_t)y-g_hnodeY[n];
        int32_t r2=(int32_t)nx*nx+(int32_t)ny*ny;
        if (r2 <= (int32_t)HTOUCH_R*HTOUCH_R && r2 >= (int32_t)(HTOUCH_R-1)*(HTOUCH_R-1))
            return RED;
    }
#endif

    // Connection lines (dim grey network)
    for (uint8_t a = 0; a < g_hNodes; a++)
        for (uint8_t b = (uint8_t)(a+1); b < g_hNodes; b++)
            if (harmonicEdge(a, b) &&
                nearSeg((int16_t)x,(int16_t)y, g_hnodeX[a],g_hnodeY[a],
                        g_hnodeX[b],g_hnodeY[b], 1.2f))
                return grey565(50);

    // Journey title at top (long-press anywhere → back to the atlas picker)
    if (glyphPixelAt(x, y, 120, 16, J.name, &Font16)) return grey565(110);

    // Current TIMBRE at the bottom, flanked by ‹ › arrows so it's clear a
    // left/right swipe changes it. The emotional move-labels live only in the
    // config.h comments now (they're no longer drawn here).
    {
        const int16_t ty = 222, ar = 6;
        const int16_t lax = 64, rax = 176;          // arrow centres
        int16_t dl = (int16_t)x - lax, dr = (int16_t)x - rax, dy = (int16_t)y - ty;
        int16_t ay = dy < 0 ? (int16_t)-dy : dy;
        if (dl >= -ar && dl <= ar && ay * 2 <= (dl + ar)) return grey565(150); // ‹
        if (dr >= -ar && dr <= ar && ay * 2 <= (ar - dr)) return grey565(150); // ›
        if (g_snapText[LINE_TIMBRE][0] &&
            glyphPixelAt(x, y, 120, ty, g_snapText[LINE_TIMBRE], &Font16))
            return grey565(170);
    }

    return grey565(8); // near-black background
}

// Compose one pixel: white glyph/arrow > (debug box) > grey background.
inline UWORD composePixel(uint16_t x, uint16_t y, int8_t li)
{
    if (li == LINE_TIMBRE && !g_timbreUsed) li = -1; // mode ignores timbre → hide its line
    if (li >= 0 && glyphPixel(x, y, LINE_Y[li], LINE_H[li], g_snapText[li], lineFont(li))) return WHITE;
    if (inArrows(x, y)) return WHITE;
#if SCREEN_TOUCH_DEBUG
    UWORD bc; if (debugBoxEdge(x, y, bc)) return bc;
#endif
    return grey565(g_snapLevel);
}

// Lock-mode pixel: a dim base from the overall glow plus a soft ("fuzzy") wedge
// per pad that brightens with that pad's intensity. Pad 3 sits at the bottom,
// the rest go counter-clockwise (see g_padAngle, set in displayInit).
static constexpr float PIE_CENTER_R = 96.0f; // soft black hole radius (px) — big soft centre
inline UWORD lockPixel(uint16_t x, uint16_t y)
{
    float dx = (float)x - (LCD_WIDTH / 2), dy = (float)y - (LCD_HEIGHT / 2);
    float r2 = dx * dx + dy * dy;
    if (r2 > 119.0f * 119.0f) return 0;            // outside the round face

    float ang  = atan2f(dy, dx);
    float v    = (float)g_snapLevel / 255.0f * 0.45f; // dim base from overall touch
    const float half = (float)(M_PI / 6.0) * 1.5f;  // wedge half-width (~1.5 slices → fuzzy)
    for (uint8_t p = 0; p < NUM_SENSORS; p++)
    {
        if (g_snapPad[p] <= 0.001f) continue;
        float d = ang - g_padAngle[p];
        while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
        while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
        if (d < 0) d = -d;
        if (d < half) { float w = 1.0f - d / half; v += g_snapPad[p] * w * w; }
    }
    if (v > 1.0f) v = 1.0f;

    // Fuzzy black hole in the centre so the wedges don't meet in a bright point.
    float hole = r2 / (PIE_CENTER_R * PIE_CENTER_R);
    if (hole < 1.0f) v *= hole * hole * (3.0f - 2.0f * hole); // smoothstep 0→1
    return grey565((uint8_t)(v * 255.0f));
}

// SPI-DMA completion (runs from the transfer ISR): release CS + bus.
inline void onFrameDone(EventResponderRef)
{
    DEV_Digital_Write(DEV_CS_PIN, 1);
    SPI.endTransaction();
    g_dmaBusy = false;
}

// Set the LCD window to the full screen, then kick ONE background DMA stream of
// the completed framebuffer. The SPI library auto-chunks transfers > 32 KB in
// its ISR and flushes the cache for OCRAM buffers; the EventResponder fires
// only after the LAST chunk, so CS stays low for the whole frame.
inline void startFrameDMA(uint8_t fbNum)
{
    SPI.beginTransaction(SPISettings(SCREEN_SPI_HZ, MSBFIRST, SPI_MODE3));
    DEV_Digital_Write(DEV_CS_PIN, 0);
    auto cmd  = [&](uint8_t c){ DEV_Digital_Write(DEV_DC_PIN, 0); SPI.transfer(c); };
    auto data = [&](uint8_t d){ DEV_Digital_Write(DEV_DC_PIN, 1); SPI.transfer(d); };
    cmd(0x2A); data(0); data(0); data(0); data(LCD_WIDTH  - 1);  // columns
    cmd(0x2B); data(0); data(0); data(0); data(LCD_HEIGHT - 1);  // rows
    cmd(0x2C);                                                   // write to RAM
    DEV_Digital_Write(DEV_DC_PIN, 1);
    g_dmaBusy = true;
    SPI.transfer(g_fb[fbNum], nullptr, FB_BYTES, g_spiEvent); // async DMA; CS up in callback
}
} // namespace

// Mark the screen for a fresh repaint (picked up at the next frame start).
namespace { inline void markDirty() { g_drawn = false; } }

// Enter or leave the Harmonic Journey screen (the third long-press state).
// Entering clears the locked-visualiser flag; leaving marks the screen dirty
// so the normal text UI repaints.
inline void displaySetHarmonicMode(bool on)
{
    g_harmonicMode = on;
    if (on) g_locked = false;
    markDirty();
}

// Show/hide the atlas picker (vs. the chord map) within Harmonic Journey.
inline void displaySetHarmonicPicking(bool on) { g_picking = on; markDirty(); }

// Select the active journey (atlas) and compute its node layout. Node 0 sits at
// the centre (home) for centre-first journeys, with the rest evenly spaced on a
// ring starting at the top; ring journeys put every node on the ring.
inline void displaySetHarmonicJourney(uint8_t j)
{
    if (j >= NUM_HARMONIC_JOURNEYS) return;
    const HarmonicJourney &J = HARMONIC_JOURNEYS[j];
    g_hjIdx   = j;
    g_hNodes  = J.nNodes;
    g_hCenter = J.centerFirst;

    // The dynamic Tonal Map lays itself out (displaySetTonalMapCenter).
    if (J.nNodes == 0) { markDirty(); return; }

    const float R  = 72.0f; // pulled in so ring nodes clear the title/label bands
    const int16_t cx = 120, cy = 120;
    uint8_t start = g_hCenter ? 1 : 0;
    uint8_t ringN = (uint8_t)(g_hNodes - start);
    if (g_hCenter) { g_hnodeX[0] = cx; g_hnodeY[0] = cy; }
    // Offset the ring by half a slice so a gap (not a node) sits at the very top
    // and bottom — that's where the journey title and move-label are drawn.
    float a0 = -(float)M_PI/2.0f + (float)M_PI / (float)ringN;
    for (uint8_t k = 0; k < ringN; k++) {
        float a = a0 + (float)k * 2.0f*(float)M_PI / (float)ringN;
        g_hnodeX[start+k] = (int16_t)(cx + R * cosf(a) + 0.5f);
        g_hnodeY[start+k] = (int16_t)(cy + R * sinf(a) + 0.5f);
    }
    markDirty();
}

// Update the selected chord node (the bottom label shows the live timbre, not a
// per-chord message, so nothing else is needed here).
inline void displaySetHarmonicChord(uint8_t chord)
{
    g_harmonicChord = (chord < g_hNodes) ? chord : 0;
    markDirty();
}

// Tonal Map: re-centre the graph on a node (0…47). animate=false snaps the
// layout into place (used at boot / when opening the map from the picker).
inline void displaySetTonalMapCenter(uint8_t node, bool animate)
{
    if (node < TONAL_NODES) tmLayout(node, animate);
}

// Hit-test a tap against the Tonal Map's option ring. Returns the tapped
// node id (0…47), or 0xFF if the tap missed every option.
inline uint8_t displayTapToTonalMapNode(uint16_t x, uint16_t y)
{
    for (uint8_t i = 0; i < g_tmNOpt; i++) {
        int16_t dx = (int16_t)x - g_tmOptX[i], dy = (int16_t)y - g_tmOptY[i];
        if ((int32_t)dx*dx + (int32_t)dy*dy <= (int32_t)HTOUCH_R*HTOUCH_R)
            return g_tmOptNode[i];
    }
    return 0xFF;
}

// Hit-test a tap against the active journey's chord nodes.
// Returns the node index, or 0xFF if no node was hit.
inline uint8_t displayTapToHarmonicChord(uint16_t x, uint16_t y)
{
    for (uint8_t n = 0; n < g_hNodes; n++) {
        int16_t nx=(int16_t)x-g_hnodeX[n], ny=(int16_t)y-g_hnodeY[n];
        if ((int32_t)nx*nx+(int32_t)ny*ny <= (int32_t)HTOUCH_R*HTOUCH_R) return n;
    }
    return 0xFF;
}

// Hit-test a tap against the atlas picker rows. Returns the journey index, or
// 0xFF if the tap missed every row.
inline uint8_t displayTapToHarmonicJourney(uint16_t x, uint16_t y)
{
    (void)x;
    for (uint8_t i = 0; i < NUM_HARMONIC_JOURNEYS; i++) {
        int16_t cy = (int16_t)(HPICK_Y0 + i * HPICK_DY);
        if ((int16_t)y >= cy - HPICK_DY/2 && (int16_t)y < cy + HPICK_DY/2) return i;
    }
    return 0xFF;
}

// True if a tap landed in the title band atop a chord map (→ back to picker).
inline bool displayTapIsHarmonicTitle(uint16_t x, uint16_t y)
{
    (void)x; return (int16_t)y < HTOP_BAND;
}

inline void displayInit()
{
    Touch_1IN28_XY XY;
    XY.mode = 2; // mixed: swipe gestures + single taps/long-press + x/y points

    Config_Init();
    analogWriteFrequency(DEV_BL_PIN, 60000); // push backlight PWM out of the audio band
    LCD_Init();
    LCD_SetBacklight(200);
    g_spiEvent.attachImmediate(&onFrameDone); // SPI-DMA done → drop CS

    if (Touch_1IN28_init(XY.mode)) Serial.println(F("Touchscreen OK"));
    else                           Serial.println(F("Touchscreen not found"));

    // Keep the CST816S reporting through a long, motionless 10 s hold: disable
    // auto-reset (0xFB), long-press-reset (0xFC), and auto-sleep (0xFE = 1).
    DEV_I2C_Write_Byte(touchAddress, 0xFB, 0x00);
    DEV_I2C_Write_Byte(touchAddress, 0xFC, 0x00);
    DEV_I2C_Write_Byte(touchAddress, 0xFE, 0x01); // DisAutoSleep

    // Pie-slice angles, one 30° slice per pad. PIE_BASE = pad-3 wedge centre,
    // PIE_DIR = +1 counter-clockwise / -1 clockwise. (Flipped 180° from bottom
    // so it reads right with SCREEN_FLIP; tweak these two if it's still off.)
    const float PIE_BASE = (float)(M_PI * 1.5); // 3π/2 (was π/2; +π = 180° flip)
    const float PIE_DIR  = 1.0f;
    for (uint8_t p = 0; p < NUM_SENSORS; p++) {
        uint8_t slot = (uint8_t)((p + NUM_SENSORS - 3) % NUM_SENSORS);
        g_padAngle[p] = PIE_BASE + PIE_DIR * (float)slot * (float)(2.0 * M_PI / NUM_SENSORS);
    }

}

inline void displayBeginCanvas()
{
    Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, BLACK);
    g_bgLevel = 0;
    strncpy(g_text[LINE_TITLE], "OMNIPHONE", sizeof(g_text[0]) - 1);
    LCD_Clear(BLACK); // one-time blocking clear at boot
    markDirty();
}

inline void displayShowScale(const char* s)
{ strncpy(g_text[LINE_SCALE], s, sizeof(g_text[0]) - 1); g_text[LINE_SCALE][sizeof(g_text[0]) - 1] = 0; markDirty(); }
inline void displayShowMode(const char* s)
{ strncpy(g_text[LINE_MODE], s, sizeof(g_text[0]) - 1); g_text[LINE_MODE][sizeof(g_text[0]) - 1] = 0; markDirty(); }
inline void displayShowTimbre(const char* s)
{ strncpy(g_text[LINE_TIMBRE], s, sizeof(g_text[0]) - 1); g_text[LINE_TIMBRE][sizeof(g_text[0]) - 1] = 0; markDirty(); }

// Show/hide the timbre line + ▲▼ (call false for modes that ignore the timbre).
inline void displaySetTimbreUsed(bool used) { if (used != g_timbreUsed) { g_timbreUsed = used; markDirty(); } }

// Small cpu/mem readout at the bottom middle (call only when the text changes).
inline void displayShowStats(const char* s)
{ strncpy(g_text[LINE_STATS], s, sizeof(g_text[0]) - 1); g_text[LINE_STATS][sizeof(g_text[0]) - 1] = 0; markDirty(); }

// Set the glow target from the average pad intensity (0..1). Cheap — just flags
// a repaint; the work happens a strip at a time in displayRenderStep().
inline void displaySetGlow(float avg)
{
    if (avg < 0.0f) avg = 0.0f; else if (avg > 1.0f) avg = 1.0f;
    uint8_t lvl = (uint8_t)(avg * 255.0f + 0.5f) & 0xF0; // 16 steps
    if (lvl != g_bgLevel) { g_bgLevel = lvl; markDirty(); }
}

// Enter/leave the locked visualiser (no text, just glow + pie). Leaving forces
// a full normal repaint so the text/arrows come back.
inline void displaySetLocked(bool on) { g_locked = on; if (!on) markDirty(); }

// Feed the per-pad intensities for the lock-mode pie (call each frame).
inline void displaySetPads(const float* p)
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++) g_pad[i] = p[i];
}

// Advance the repaint. Renders ONE band of the frame into the off-screen
// framebuffer and returns immediately; when the last band is done the whole
// frame is flipped to the panel in a single background DMA. Call at least once
// per sensor frame — extra calls in loop idle time render extra bands and just
// raise the frame rate. Returns true while more work remains.
inline bool displayRenderStep()
{
    if (g_band == 0)  // idle, or about to start a new frame
    {
        // Tonal Map relayout glide: advance it once PER FRAME (never between
        // bands) so every band sees the same animation state. While gliding it
        // marks dirty, which keeps the frames coming until the glide lands.
        if (g_harmonicMode && !g_picking && g_hjIdx == JOURNEY_TONAL_MAP)
            tmAnimTick();

        if (g_drawn && !g_locked) return g_dmaBusy; // up to date (locked never idles)

        // Frame start: snapshot everything the pixel composers read, so the
        // whole frame comes from this one instant. Changes that land during
        // the render mark dirty again and are picked up by the NEXT frame.
        memcpy(g_snapText, g_text, sizeof(g_snapText));
        memcpy(g_snapPad,  g_pad,  sizeof(g_snapPad));
        g_snapLevel = g_bgLevel;
        g_drawn = true;
    }

    if (g_band < N_BANDS)
    {
        uint16_t y0 = (uint16_t)(g_band * BAND_ROWS);
        uint16_t h  = BAND_ROWS;
        if (y0 + h > LCD_HEIGHT) h = (uint16_t)(LCD_HEIGHT - y0);

        // Render the band into the framebuffer (big-endian 16-bit for SPI).
        // SCREEN_FLIP renders the panel rotated 180°: panel pixel (x,y) shows
        // the logical content at (W-1-x, H-1-y), so a screen mounted upside
        // down reads upright. lx/ly are the logical (content) coordinates.
        uint8_t* dst = &g_fb[g_fbRender][(uint32_t)y0 * LCD_WIDTH * 2];
        uint32_t idx = 0;
        for (uint16_t y = y0; y < y0 + h; y++)
        {
#if SCREEN_FLIP
            uint16_t ly = (uint16_t)(LCD_HEIGHT - 1 - y);
#else
            uint16_t ly = y;
#endif
            int8_t li = -1;
            for (uint8_t k = 0; k < LINE_COUNT; k++)
                if (ly >= LINE_Y[k] && ly < LINE_Y[k] + LINE_H[k]) { li = (int8_t)k; break; }
            for (uint16_t x = 0; x < LCD_WIDTH; x++)
            {
#if SCREEN_FLIP
                uint16_t lx = (uint16_t)(LCD_WIDTH - 1 - x);
#else
                uint16_t lx = x;
#endif
                UWORD c = g_locked       ? lockPixel(lx, ly)      :
                          g_harmonicMode ? harmonicPixel(lx, ly)  :
                                           composePixel(lx, ly, li);
                dst[idx++] = (uint8_t)(c >> 8);
                dst[idx++] = (uint8_t)(c & 0xFF);
            }
        }
        if (++g_band < N_BANDS) return true;
    }

    // Frame complete → flip it to the panel as soon as the previous transfer
    // has finished (with two buffers that's usually immediately).
    if (g_dmaBusy) return true;
    startFrameDMA(g_fbRender);
    g_fbRender ^= 1;
    g_band = 0;
    return !g_drawn || g_locked;
}

// Map a tap position to a control: which arrow box (if any) it landed in.
inline DisplayGesture displayTapToGesture(uint16_t x, uint16_t y)
{
    for (const TapBox& b : TAP_BOXES) {
        if (!g_timbreUsed && (b.act == GESTURE_UP || b.act == GESTURE_DOWN)) continue; // timbre hidden
        if (x >= b.x0 && x <= b.x1 && y >= b.y0 && y <= b.y1) return b.act;
    }
    return GESTURE_NONE;
}

namespace {
static uint16_t g_downX = 0, g_downY = 0; // last raw position seen while finger was DOWN
static bool     g_haveDown = false;

// Read the touch point CORRECTLY (the vendor DEV_I2C_Read_nByte forgot its
// endTransmission, so the register pointer was never set → garbage X). Reads
// finger-count + X + Y in one transaction with a proper repeated start. Returns
// false if no finger is down.
inline bool readTouchRaw(uint16_t& x, uint16_t& y)
{
    Wire.beginTransmission(touchAddress);
    Wire.write((uint8_t)0x02);                 // start at the finger-count register
    if (Wire.endTransmission(false) != 0) return false; // repeated start (keep bus)
    if (Wire.requestFrom((int)touchAddress, 5) < 5) return false;
    uint8_t f  = (uint8_t)Wire.read();         // 0x02 finger count
    uint8_t xh = (uint8_t)Wire.read();         // 0x03
    uint8_t xl = (uint8_t)Wire.read();         // 0x04
    uint8_t yh = (uint8_t)Wire.read();         // 0x05
    uint8_t yl = (uint8_t)Wire.read();         // 0x06
    if (f == 0) return false;
    x = (uint16_t)(((xh & 0x0f) << 8) | xl);
    y = (uint16_t)(((yh & 0x0f) << 8) | yl);
    return true;
}

// This panel reports touch coordinates in native display pixels (0..239), so
// the map is identity + clamp. Flip the SWAP/FLIP knobs if a unit ever comes
// up rotated or mirrored.
static constexpr bool TOUCH_SWAP_XY = false;
static constexpr bool TOUCH_FLIP_X  = SCREEN_FLIP; // match the 180° display flip
static constexpr bool TOUCH_FLIP_Y  = SCREEN_FLIP;
inline void touchMap(uint16_t rx, uint16_t ry, uint16_t& dx, uint16_t& dy)
{
    uint16_t sx = (rx > LCD_WIDTH  - 1) ? (LCD_WIDTH  - 1) : rx;
    uint16_t sy = (ry > LCD_HEIGHT - 1) ? (LCD_HEIGHT - 1) : ry;
    if (TOUCH_SWAP_XY) { uint16_t t = sx; sx = sy; sy = t; }
    if (TOUCH_FLIP_X)  sx = (uint16_t)(LCD_WIDTH  - 1 - sx);
    if (TOUCH_FLIP_Y)  sy = (uint16_t)(LCD_HEIGHT - 1 - sy);
    dx = sx; dy = sy;
}
} // namespace

inline DisplayGesture displayPollGesture(uint16_t& tapX, uint16_t& tapY)
{
    // Swipe plausibility check: the CST816S's internal swipe threshold is tiny,
    // so a slightly rough PRESS (finger rolls a few px) gets reported as a
    // swipe — surprise timbre changes. We track the finger's total travel for
    // the current press ourselves and defer any chip-reported swipe until the
    // finger lifts: travel ≥ SWIPE_MIN_PX → real swipe; less → it was a press,
    // reclassified as a TAP at the press-start position (so a rough tap on an
    // arrow box still does what the user meant). Costs one 30 ms poll of
    // latency on real swipes — imperceptible for scale/timbre changes.
    static bool           pressActive  = false;
    static uint16_t       startX = 0, startY = 0;
    static DisplayGesture pendingSwipe = GESTURE_NONE;

    // Track the finger position WHILE it's down, so a tap (which the CST816S
    // only reports on release, when the coordinate registers are stale) can use
    // the last good down-position instead.
    bool fingerDown = false;
    { uint16_t rx, ry;
      if (readTouchRaw(rx, ry)) {
          fingerDown = true;
          if (!pressActive) { pressActive = true; startX = rx; startY = ry; }
          g_downX = rx; g_downY = ry; g_haveDown = true;
      } }

    static uint8_t shown = 0;
    uint8_t raw = DEV_I2C_Read_Byte(touchAddress, GESTUREID);
    bool isEvent = (raw == UP || raw == Down || raw == LEFT || raw == RIGHT
                 || raw == CLICK || raw == LONG_PRESS);
    if (!isEvent) shown = 0;

    // Latch a newly reported gesture (deduped via `shown`, like before).
    // Swipes aren't returned here — they park in pendingSwipe until the finger
    // lifts, so the travel check below can rule on them.
    if (isEvent && raw != shown) {
        shown = raw;
        switch (raw) {
            case UP:    pendingSwipe = GESTURE_UP;    break;
            case Down:  pendingSwipe = GESTURE_DOWN;  break;
            case LEFT:  pendingSwipe = GESTURE_LEFT;  break;
            case RIGHT: pendingSwipe = GESTURE_RIGHT; break;
            case LONG_PRESS:
                pendingSwipe = GESTURE_NONE;
                pressActive  = false;
                return GESTURE_LONGPRESS;
            case CLICK: {
                pendingSwipe = GESTURE_NONE;
                pressActive  = false;
                if (!g_haveDown) return GESTURE_NONE;
                touchMap(g_downX, g_downY, tapX, tapY);
                g_haveDown = false;
#if SCREEN_TOUCH_DEBUG
                Serial.print(F("# TAP raw=")); Serial.print(g_downX); Serial.print(',');
                Serial.print(g_downY);         Serial.print(F("  disp="));
                Serial.print(tapX); Serial.print(','); Serial.println(tapY);
#endif
                return GESTURE_TAP;
            }
            default: break;
        }
    }

    // Finger up with a parked swipe → resolve it against the actual travel of
    // the press that produced it (g_downX/Y = last position seen while down).
    if (!fingerDown && pendingSwipe != GESTURE_NONE) {
        DisplayGesture g = pendingSwipe;
        pendingSwipe = GESTURE_NONE;
        pressActive  = false;
        int16_t dx = (int16_t)g_downX - (int16_t)startX;
        int16_t dy = (int16_t)g_downY - (int16_t)startY;
        if ((int32_t)dx * dx + (int32_t)dy * dy >=
            (int32_t)SWIPE_MIN_PX * SWIPE_MIN_PX)
            return g;                          // genuine swipe
        touchMap(startX, startY, tapX, tapY);  // rough press → act as a tap
        g_haveDown = false;
        return GESTURE_TAP;
    }
    if (!fingerDown) pressActive = false;

    return GESTURE_NONE;
}

// Raw finger position (debug): true + raw x/y while a finger is down.
inline bool displayPollTouch(uint16_t& x, uint16_t& y) { return readTouchRaw(x, y); }

// Raw finger position, mapped to display pixel space, while a finger is down
// THIS poll — unlike GESTURE_TAP, this isn't gated behind the touch chip's
// internal gesture-commit delay (it only reports CLICK after a few hundred ms,
// once it has ruled out the touch turning into a long-press or swipe). Callers
// that want an instant response (e.g. Harmonic Journey node selection) should
// act on the down-edge of this instead of waiting for GESTURE_TAP.
inline bool displayPollTouchXY(uint16_t& x, uint16_t& y)
{
    uint16_t rx, ry;
    if (!readTouchRaw(rx, ry)) return false;
    touchMap(rx, ry, x, y);
    return true;
}

#else  // ENABLE_SCREEN == 0 — no-op stubs

inline void displayInit() {}
inline void displayBeginCanvas() {}
inline void displayShowScale(const char*) {}
inline void displayShowMode(const char*) {}
inline void displayShowTimbre(const char*) {}
inline void displaySetTimbreUsed(bool) {}
inline void displayShowStats(const char*) {}
inline void displaySetGlow(float) {}
inline void displaySetLocked(bool) {}
inline void displaySetPads(const float*) {}
inline bool displayRenderStep() { return false; }
inline DisplayGesture displayTapToGesture(uint16_t, uint16_t) { return GESTURE_NONE; }
inline DisplayGesture displayPollGesture(uint16_t&, uint16_t&) { return GESTURE_NONE; }
inline bool displayPollTouch(uint16_t&, uint16_t&) { return false; }
inline bool displayPollTouchXY(uint16_t&, uint16_t&) { return false; }
inline void displaySetHarmonicMode(bool) {}
inline void displaySetHarmonicPicking(bool) {}
inline void displaySetHarmonicJourney(uint8_t) {}
inline void displaySetHarmonicChord(uint8_t) {}
inline uint8_t displayTapToHarmonicChord(uint16_t, uint16_t) { return 0xFF; }
inline uint8_t displayTapToHarmonicJourney(uint16_t, uint16_t) { return 0xFF; }
inline bool displayTapIsHarmonicTitle(uint16_t, uint16_t) { return false; }
inline void displaySetTonalMapCenter(uint8_t, bool) {}
inline uint8_t displayTapToTonalMapNode(uint16_t, uint16_t) { return 0xFF; }

#endif // ENABLE_SCREEN
