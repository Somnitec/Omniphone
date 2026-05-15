#include <Arduino.h>
#include <Wire.h>
#include <MPR121.h>
#include "config.h"            // SENSORS, BOARD_ADDRESSES, BELL_* curve consts
#include "proximity_engine.h"  // the REAL algorithm we're tuning

// ─────────────────────────────────────────────────────────────────────────────
// Strike-dynamics & sensor tuning rig
//
// Build:  pio run -e strike_tuning -t upload      (serial @115200)
//
// Works on ONE pad at a time (the one you tap), using the exact proximity
// engine + bell-dynamics maths from the firmware, so any value we settle on
// here can be pasted straight into config.h / proximity_engine.h.
//
// ── WORKFLOW ─────────────────────────────────────────────────────────────────
// STEP 1 — sensor range (do this first, per pad you care about):
//   • `p<n>`  select pad n (serial index, 0..10). Default 0.
//   • `n`     NOISE scan: take your hand fully away, wait ~2 s. It reports the
//             idle noise band → a good `proxDeadband` is ~1.5× that.
//   • `h`     HOVER scan: hold your hand at the *closest distance you want to
//             still count as "far/quiet"* … then at the *loudest hover* for
//             ~3 s. Reports the fast-EMA span → sets `proxMax`.
//   • Adjust live and re-test:  d<val>=deadband  x<val>=proxMax
//             C<0-63>=charge current (CDC)  T<0-7>=charge time (CDT)
//             (raise CDC/CDT for more distance & a bigger swing; watch noise.)
//   • `s` toggles a raw telemetry stream so you can watch the numbers move.
//
// STEP 2 — strike dynamics:
//   • Set the label, then tap that pad 3×:
//        `1` → label "soft"   `2` → "med"   `3` → "hard"
//     Suggested protocol:  press `1`, tap 3× soft;  `2`, tap 3× medium;
//     `3`, tap 3× hard.
//   • Each detected contact prints ONE row with every signal we could base
//     dynamics on (pre-contact level, peak jump, peak velocity, etc.) plus the
//     `strike`/`amp` the CURRENT firmware formula would produce.
//   • Copy the whole block of rows and hand it back — that's the data to
//     retune the algorithm with.
//
//   `j<val>` live-set jumpThreshold,  `?` print help+config,  `c` print config.
// ─────────────────────────────────────────────────────────────────────────────

// MPR121 timing registers (FFI|CDC, CDT|SFI|ESI) — same fields the lib uses.
static constexpr uint8_t REG_CDC_CFG = 0x5C;
static constexpr uint8_t REG_CDT_CFG = 0x5D;
static constexpr uint8_t REG_ECR     = 0x5E;

static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]),
    MPR121(BOARD_ADDRESSES[1]),
};

static ProximityConfig cfg;          // live-editable copy of the engine config
static SensorState     st;           // state for the selected pad

static uint8_t  padIdx   = 0;        // selected serial pad index
static uint8_t  cdc      = SENSOR_CDC; // charge current 0–63 (matches firmware)
static uint8_t  cdt      = SENSOR_CDT; // charge time 0–7
static bool     stream   = false;    // raw telemetry stream on/off
static const char* label = "-";      // current strike label (soft/med/hard)

static uint32_t lastUpdMs  = 0;
static uint32_t lastStrmMs = 0;

// ── Per-contact capture (peak-hold over a short window after the strike) ─────
static bool     capturing   = false;
static uint32_t capEndMs    = 0;
static float    preFast, peakJump, peakVel, peakRaw, rawAtContact;
static constexpr uint32_t CAPTURE_MS = 150;

// ── Calibration scans ────────────────────────────────────────────────────────
enum Scan : uint8_t { SCAN_NONE, SCAN_NOISE, SCAN_HOVER };
static Scan     scan = SCAN_NONE;
static uint32_t scanEndMs = 0;
static float    scanMin, scanMax;

// ── MPR121 helpers ───────────────────────────────────────────────────────────
static uint8_t boardOf(uint8_t pad)     { return SENSORS[pad].boardIndex; }
static uint8_t electrodeOf(uint8_t pad) { return SENSORS[pad].electrode; }

// Re-apply charge timing on every board, then reload the baseline.
static void applyTiming()
{
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        uint8_t n = SENSE_ELECTRODES[b];
        boards[b].write(REG_ECR, 0x00);                 // stop
        boards[b].write(REG_CDC_CFG, (uint8_t)(cdc & 0x3F));          // FFI=0, CDC
        boards[b].write(REG_CDT_CFG, (uint8_t)(((cdt & 0x07) << 5) | 0b001)); // CDT,SFI=0,ESI=2ms
        boards[b].write(REG_ECR, (uint8_t)(0b11000000 | n)); // CL=11 full reload, run
        delay(60);
    }
}

static float readRawDelta(uint8_t pad)
{
    uint8_t  b = boardOf(pad), e = electrodeOf(pad);
    uint16_t filt = boards[b].filteredData(e);
    uint16_t base = boards[b].baselineData(e);
    int16_t  d    = (int16_t)base - (int16_t)filt;
    return d < 0 ? 0.0f : (float)d;
}

static void reseed()
{
    seedSensorState(st, readRawDelta(padIdx));
    capturing = false;
    scan = SCAN_NONE;
}

// ── Serial ───────────────────────────────────────────────────────────────────
static void printConfig()
{
    Serial.println(F("# ── config ──"));
    Serial.print(F("# pad="));      Serial.print(padIdx);
    Serial.print(F(" (board "));    Serial.print(boardOf(padIdx));
    Serial.print(F(" ELE"));        Serial.print(electrodeOf(padIdx));
    Serial.println(F(")"));
    Serial.print(F("# CDC="));      Serial.print(cdc);
    Serial.print(F(" CDT="));       Serial.println(cdt);
    Serial.print(F("# deadband=")); Serial.print(cfg.proxDeadband, 2);
    Serial.print(F(" proxMax="));   Serial.print(cfg.proxMax, 2);
    Serial.print(F(" jumpThr="));   Serial.print(cfg.jumpThreshold, 2);
    Serial.print(F(" aRise="));     Serial.print(cfg.fastAlphaRise, 2);
    Serial.print(F(" aFall="));     Serial.print(cfg.fastAlphaFall, 2);
    Serial.print(F(" aSlow="));     Serial.println(cfg.slowAlpha, 3);
    Serial.print(F("# bell: FLOOR="));Serial.print(BELL_FLOOR, 3);
    Serial.print(F(" CURVE="));     Serial.print(BELL_CURVE, 2);
    Serial.print(F(" GAIN="));      Serial.println(BELL_STRIKE_GAIN, 2);
}

static void printHelp()
{
    Serial.println(F("# keys: p<n> pad | s stream | 1/2/3 label soft/med/hard"));
    Serial.println(F("#       n noise-scan | h hover-scan | b reseed baseline"));
    Serial.println(F("#       d<v> deadband | x<v> proxMax | j<v> jumpThr"));
    Serial.println(F("#       C<0-63> CDC | T<0-7> CDT | c config | ? help"));
}

static float readNum() { return Serial.parseFloat(); }

static void handleSerial()
{
    if (!Serial.available()) return;
    char c = Serial.read();
    switch (c)
    {
        case 'p': { int v = (int)readNum(); if (v >= 0 && v < NUM_SENSORS) { padIdx = (uint8_t)v; reseed(); } printConfig(); break; }
        case 's':  stream = !stream; Serial.print(F("# stream ")); Serial.println(stream ? F("ON") : F("OFF")); break;
        case '1':  label = "soft"; Serial.println(F("# label = soft"));  break;
        case '2':  label = "med";  Serial.println(F("# label = med"));   break;
        case '3':  label = "hard"; Serial.println(F("# label = hard"));  break;
        case 'b':  reseed(); Serial.println(F("# baseline reseeded"));    break;
        case 'd':  cfg.proxDeadband  = readNum(); printConfig(); break;
        case 'x':  cfg.proxMax       = readNum(); printConfig(); break;
        case 'j':  cfg.jumpThreshold = readNum(); printConfig(); break;
        case 'C':  cdc = (uint8_t)readNum(); applyTiming(); reseed(); printConfig(); break;
        case 'T':  cdt = (uint8_t)readNum(); applyTiming(); reseed(); printConfig(); break;
        case 'n':  scan = SCAN_NOISE; scanEndMs = millis() + 2000; scanMin = 1e9f; scanMax = -1e9f; Serial.println(F("# NOISE scan 2s — hand AWAY")); break;
        case 'h':  scan = SCAN_HOVER; scanEndMs = millis() + 3000; scanMin = 1e9f; scanMax = -1e9f; Serial.println(F("# HOVER scan 3s — hold at loudest hover")); break;
        case 'c':  printConfig(); break;
        case '?':  printHelp(); printConfig(); break;
        default: break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    Wire.begin();
    Wire.setClock(400000);

    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (!boards[b].begin(SENSE_ELECTRODES[b]))
        {
            Serial.print(F("ERROR: MPR121 board "));
            Serial.print(b);
            Serial.println(F(" not found"));
        }
    }
    applyTiming();
    reseed();

    Serial.println(F("# Strike-tuning rig"));
    printHelp();
    printConfig();
    Serial.println(F("# CAPTURE columns:"));
    Serial.println(F("# label  t_ms  preFast  preInt  peakJump  peakVel  peakRaw  rawAtContact  strike  amp"));
}

// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    handleSerial();

    uint32_t now = millis();
    if (now - lastUpdMs < UPDATE_MS) return;
    lastUpdMs = now;

    float rawDelta = readRawDelta(padIdx);

    float intensity;
    bool  isTouch;
    updateProximity(rawDelta, now, cfg, st, intensity, isTouch);

    float jump = st.fast - st.slow;

    // ── Calibration scans ────────────────────────────────────────────────────
    if (scan != SCAN_NONE)
    {
        float v = (scan == SCAN_NOISE) ? rawDelta : st.fast;
        if (v < scanMin) scanMin = v;
        if (v > scanMax) scanMax = v;
        if ((int32_t)(now - scanEndMs) >= 0)
        {
            if (scan == SCAN_NOISE)
            {
                Serial.print(F("# NOISE: min=")); Serial.print(scanMin, 2);
                Serial.print(F(" max="));         Serial.print(scanMax, 2);
                Serial.print(F("  → suggest proxDeadband ≈ "));
                Serial.println(scanMax * 1.5f, 2);
            }
            else
            {
                Serial.print(F("# HOVER: fast min=")); Serial.print(scanMin, 2);
                Serial.print(F(" max="));              Serial.print(scanMax, 2);
                Serial.print(F("  → suggest proxMax ≈ "));
                Serial.println(scanMax, 2);
            }
            scan = SCAN_NONE;
        }
    }

    // ── Contact capture (peak-hold window) ───────────────────────────────────
    if (isTouch && !capturing)
    {
        capturing    = true;
        capEndMs     = now + CAPTURE_MS;
        preFast      = st.prevFast;          // level just before the spike
        rawAtContact = rawDelta;
        peakJump     = jump;
        peakVel      = st.velocity;
        peakRaw      = rawDelta;
    }
    else if (capturing)
    {
        if (jump        > peakJump) peakJump = jump;
        if (st.velocity > peakVel)  peakVel  = st.velocity;
        if (rawDelta    > peakRaw)  peakRaw  = rawDelta;

        if ((int32_t)(now - capEndMs) >= 0)
        {
            // Replicate the firmware's velocity→amp maths exactly.
            float preN = (preFast - cfg.proxDeadband)
                       / (cfg.proxMax - cfg.proxDeadband);
            preN = preN < 0.0f ? 0.0f : (preN > 1.0f ? 1.0f : preN);
            float strike = (peakVel - BELL_VEL_MIN) / (BELL_VEL_MAX - BELL_VEL_MIN);
            strike *= BELL_STRIKE_GAIN;
            strike = strike < 0.0f ? 0.0f : (strike > 1.0f ? 1.0f : strike);
            float amp = BELL_AMP * (BELL_FLOOR + (1.0f - BELL_FLOOR) * powf(strike, BELL_CURVE));

            Serial.print(label);          Serial.print('\t');
            Serial.print(now);            Serial.print('\t');
            Serial.print(preFast, 2);     Serial.print('\t');
            Serial.print(preN, 3);        Serial.print('\t');
            Serial.print(peakJump, 2);    Serial.print('\t');
            Serial.print(peakVel, 2);     Serial.print('\t');
            Serial.print(peakRaw, 2);     Serial.print('\t');
            Serial.print(rawAtContact, 2);Serial.print('\t');
            Serial.print(strike, 3);      Serial.print('\t');
            Serial.println(amp, 3);
            capturing = false;
        }
    }

    // ── Optional raw telemetry stream (~30 Hz) ───────────────────────────────
    if (stream && (now - lastStrmMs >= 33))
    {
        lastStrmMs = now;
        Serial.print(F("raw:"));  Serial.print(rawDelta, 1);
        Serial.print(F(" fast:"));Serial.print(st.fast, 1);
        Serial.print(F(" slow:"));Serial.print(st.slow, 1);
        Serial.print(F(" jump:"));Serial.print(jump, 1);
        Serial.print(F(" int:")); Serial.print(intensity, 2);
        Serial.print(F(" inC:")); Serial.println(st.inContact ? 1 : 0);
    }
}
