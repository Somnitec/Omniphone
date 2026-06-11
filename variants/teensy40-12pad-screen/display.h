#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Display module — round 1.28" LCD + capacitive touch (Waveshare GC9A01)
//
// ALL screen code lives here. When ENABLE_SCREEN == 0 every function is an
// empty inline no-op, so callers never need their own #if guard.
//
// PARALLEL / NON-BLOCKING: the screen is repainted one horizontal strip at a
// time. Each strip is rendered into a RAM buffer (cheap CPU) and streamed to the
// LCD by SPI DMA — a background transfer — so the loop returns immediately to
// the audio and LEDs and never blocks on the screen. The SPI bus is dedicated to
// the LCD (audio = I2S, LEDs/touch = I2C), so the DMA never competes with them.
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
    GESTURE_TAP,       // a single tap (tapX/tapY out-params are set)
    GESTURE_LONGPRESS  // a long press → cycle play mode
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

// ── Layout: title / mode / scale (centred) / timbre ──────────────────────────
static constexpr uint16_t TITLE_T  = 10,  TITLE_H  = 26;
static constexpr uint16_t MODE_T   = 54,  MODE_H   = 30;
static constexpr uint16_t SCALE_T  = 92,  SCALE_H  = 52;  // centred on ~118 ≈ screen mid
static constexpr uint16_t TIMBRE_T = 156, TIMBRE_H = 60;
enum { LINE_TITLE = 0, LINE_MODE, LINE_SCALE, LINE_TIMBRE, LINE_COUNT };

static constexpr uint16_t GLOW_STRIP   = 30;            // rows streamed per render step
static constexpr uint32_t SCREEN_SPI_HZ = 40000000;    // GC9A01 SPI clock (proven-safe alongside audio)

namespace {
static const uint16_t LINE_Y[LINE_COUNT] = { TITLE_T, MODE_T, SCALE_T, TIMBRE_T };
static const uint16_t LINE_H[LINE_COUNT] = { TITLE_H, MODE_H, SCALE_H, TIMBRE_H };
inline sFONT* lineFont(uint8_t i) { return (i == LINE_TITLE || i == LINE_SCALE) ? &Font20 : &Font16; }

// Tap targets, one box per arrow — sized to where presses actually land on this
// round panel (left/right at the sides, up/down stacked in the centre-lower
// area). Edges = scale, centre column = timbre, so they never overlap.
struct TapBox { uint16_t x0, y0, x1, y1; DisplayGesture act; UWORD dbg; };
static const TapBox TAP_BOXES[4] = {
    {   0,  85,  58, 155, GESTURE_LEFT,  RED    }, // ‹ scale-
    { 182,  85, 239, 155, GESTURE_RIGHT, GREEN  }, // › scale+
    {  60, 118, 179, 193, GESTURE_UP,    BLUE   }, // ▲ timbre-
    {  60, 194, 179, 239, GESTURE_DOWN,  YELLOW }, // ▼ timbre+
};

static char    g_text[LINE_COUNT][20] = { {0}, {0}, {0}, {0} };
static uint8_t g_bgLevel = 0; // background grey 0 (black) … 255 (white)

// Lock / visualiser mode: no text, just the glow + a fuzzy 12-pad pie.
static bool  g_locked = false;
static float g_pad[NUM_SENSORS]      = { 0 }; // per-pad intensity (0..1)
static float g_padAngle[NUM_SENSORS] = { 0 }; // each pad's wedge centre angle

// Incremental-repaint state. The screen is painted in strips, in an INSIDE-OUT
// order (centre strip first, then outward) so a refresh radiates from the middle
// rather than rolling top-to-bottom — smoother, especially in lock mode.
static uint8_t  g_stripOrder[16] = { 0 }; // strip indices, centre-out
static uint8_t  g_nStrips    = 0;     // number of strips (set in displayInit)
static uint8_t  g_sweepStep  = 0;     // index into g_stripOrder for the next strip
static uint8_t  g_sweepLevel = 0;     // bg level captured at the top of this sweep
static bool     g_drawn      = true;  // whole screen matches the target → idle

// DMA transfer state (the strip buffer lives in OCRAM so DMA can reach it).
DMAMEM static uint8_t g_strip[LCD_WIDTH * GLOW_STRIP * 2];
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
    if (tri(14, scy, 'L') || tri(LCD_WIDTH - 14, scy, 'R')) return true;
    if (tri(LCD_WIDTH / 2, TIMBRE_T + 9, 'U'))              return true;
    if (tri(LCD_WIDTH / 2, TIMBRE_T + TIMBRE_H - 9, 'D'))   return true;
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

// Compose one pixel: white glyph/arrow > (debug box) > grey background.
inline UWORD composePixel(uint16_t x, uint16_t y, int8_t li)
{
    if (li >= 0 && glyphPixel(x, y, LINE_Y[li], LINE_H[li], g_text[li], lineFont(li))) return WHITE;
    if (inArrows(x, y)) return WHITE;
#if SCREEN_TOUCH_DEBUG
    UWORD bc; if (debugBoxEdge(x, y, bc)) return bc;
#endif
    return grey565(g_bgLevel);
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
    float v    = (float)g_bgLevel / 255.0f * 0.45f; // dim base from overall touch
    const float half = (float)(M_PI / 6.0) * 1.5f;  // wedge half-width (~1.5 slices → fuzzy)
    for (uint8_t p = 0; p < NUM_SENSORS; p++)
    {
        if (g_pad[p] <= 0.001f) continue;
        float d = ang - g_padAngle[p];
        while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
        while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
        if (d < 0) d = -d;
        if (d < half) { float w = 1.0f - d / half; v += g_pad[p] * w * w; }
    }
    if (v > 1.0f) v = 1.0f;

    // Fuzzy black hole in the centre so the wedges don't meet in a bright point.
    float hole = r2 / (PIE_CENTER_R * PIE_CENTER_R);
    if (hole < 1.0f) v *= hole * hole * (3.0f - 2.0f * hole); // smoothstep 0→1
    return grey565((uint8_t)(v * 255.0f));
}

// SPI-DMA completion (runs from the transfer ISR): release CS + bus.
inline void onStripDone(EventResponderRef)
{
    DEV_Digital_Write(DEV_CS_PIN, 1);
    SPI.endTransaction();
    g_dmaBusy = false;
}

// Set the LCD address window, then kick the background DMA stream of g_strip.
inline void startStripDMA(uint16_t y0, uint16_t h)
{
    SPI.beginTransaction(SPISettings(SCREEN_SPI_HZ, MSBFIRST, SPI_MODE3));
    DEV_Digital_Write(DEV_CS_PIN, 0);
    auto cmd  = [&](uint8_t c){ DEV_Digital_Write(DEV_DC_PIN, 0); SPI.transfer(c); };
    auto data = [&](uint8_t d){ DEV_Digital_Write(DEV_DC_PIN, 1); SPI.transfer(d); };
    cmd(0x2A); data(0); data(0);            data(0); data(LCD_WIDTH - 1);          // columns
    cmd(0x2B); data(0); data((uint8_t)y0);  data(0); data((uint8_t)(y0 + h - 1));  // rows
    cmd(0x2C);                                                                     // write to RAM
    DEV_Digital_Write(DEV_DC_PIN, 1);
    g_dmaBusy = true;
    SPI.transfer(g_strip, nullptr, (size_t)h * LCD_WIDTH * 2, g_spiEvent); // async DMA; CS up in callback
}
} // namespace

inline void displayInit()
{
    Touch_1IN28_XY XY;
    XY.mode = 2; // mixed: swipe gestures + single taps/long-press + x/y points

    Config_Init();
    analogWriteFrequency(DEV_BL_PIN, 60000); // push backlight PWM out of the audio band
    LCD_Init();
    LCD_SetBacklight(200);
    g_spiEvent.attachImmediate(&onStripDone); // SPI-DMA done → drop CS

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

    // Inside-out strip order: centre strip first, then alternately outward.
    g_nStrips = (uint8_t)((LCD_HEIGHT + GLOW_STRIP - 1) / GLOW_STRIP);
    int mid = (g_nStrips - 1) / 2, k = 0;
    g_stripOrder[k++] = (uint8_t)mid;
    for (int d = 1; d < g_nStrips && k < g_nStrips; d++) {
        if (mid + d < g_nStrips) g_stripOrder[k++] = (uint8_t)(mid + d);
        if (mid - d >= 0)        g_stripOrder[k++] = (uint8_t)(mid - d);
    }
}

// Mark the screen for a fresh incremental repaint (from the centre out).
namespace { inline void markDirty() { g_drawn = false; g_sweepStep = 0; } }

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

// Advance the repaint by ONE strip. Renders to RAM and kicks a DMA stream, then
// returns immediately (the transfer finishes in the background). Call once per
// frame. Returns true while more work remains.
inline bool displayRenderStep()
{
    if (g_dmaBusy) return true;             // a strip is still streaming
    if (g_drawn && !g_locked) return false; // up to date (locked mode never idles)

    if (g_sweepStep == 0) g_sweepLevel = g_bgLevel;
    uint16_t y0 = (uint16_t)(g_stripOrder[g_sweepStep] * GLOW_STRIP); // inside-out
    uint16_t h  = GLOW_STRIP;
    if (y0 + h > LCD_HEIGHT) h = (uint16_t)(LCD_HEIGHT - y0);

    // Render the strip into the byte buffer (big-endian 16-bit colour for SPI).
    // SCREEN_FLIP renders the panel rotated 180°: panel pixel (x,y) shows the
    // logical content at (W-1-x, H-1-y), so a screen mounted upside down reads
    // upright. lx/ly are the logical (content) coordinates used everywhere.
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
            UWORD c = g_locked ? lockPixel(lx, ly) : composePixel(lx, ly, li);
            g_strip[idx++] = (uint8_t)(c >> 8);
            g_strip[idx++] = (uint8_t)(c & 0xFF);
        }
    }

    startStripDMA(y0, h);
    if (++g_sweepStep >= g_nStrips)
    {
        g_sweepStep = 0;
        if (!g_locked && g_bgLevel == g_sweepLevel) g_drawn = true; // steady → done
    }
    return true;
}

// Map a tap position to a control: which arrow box (if any) it landed in.
inline DisplayGesture displayTapToGesture(uint16_t x, uint16_t y)
{
    for (const TapBox& b : TAP_BOXES)
        if (x >= b.x0 && x <= b.x1 && y >= b.y0 && y <= b.y1) return b.act;
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
    // Track the finger position WHILE it's down, so a tap (which the CST816S
    // only reports on release, when the coordinate registers are stale) can use
    // the last good down-position instead.
    { uint16_t rx, ry; if (readTouchRaw(rx, ry)) { g_downX = rx; g_downY = ry; g_haveDown = true; } }

    static uint8_t shown = 0;
    uint8_t raw = DEV_I2C_Read_Byte(touchAddress, GESTUREID);
    bool isEvent = (raw == UP || raw == Down || raw == LEFT || raw == RIGHT
                 || raw == CLICK || raw == LONG_PRESS);
    if (!isEvent)     { shown = 0; return GESTURE_NONE; }
    if (raw == shown) return GESTURE_NONE;
    shown = raw;
    switch (raw) {
        case UP:    return GESTURE_UP;
        case Down:  return GESTURE_DOWN;
        case LEFT:  return GESTURE_LEFT;
        case RIGHT: return GESTURE_RIGHT;
        case LONG_PRESS: return GESTURE_LONGPRESS;
        case CLICK: {
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
        default: return GESTURE_NONE;
    }
}

// Raw finger position (debug): true + raw x/y while a finger is down.
inline bool displayPollTouch(uint16_t& x, uint16_t& y) { return readTouchRaw(x, y); }

#else  // ENABLE_SCREEN == 0 — no-op stubs

inline void displayInit() {}
inline void displayBeginCanvas() {}
inline void displayShowScale(const char*) {}
inline void displayShowMode(const char*) {}
inline void displayShowTimbre(const char*) {}
inline void displaySetGlow(float) {}
inline void displaySetLocked(bool) {}
inline void displaySetPads(const float*) {}
inline bool displayRenderStep() { return false; }
inline DisplayGesture displayTapToGesture(uint16_t, uint16_t) { return GESTURE_NONE; }
inline DisplayGesture displayPollGesture(uint16_t&, uint16_t&) { return GESTURE_NONE; }
inline bool displayPollTouch(uint16_t&, uint16_t&) { return false; }

#endif // ENABLE_SCREEN
