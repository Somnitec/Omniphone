#include <Arduino.h>
#include <Wire.h>
#include <MPR121.h>
#include "../variants/teensy40-11pad/config.h"            // SENSORS, BOARD_ADDRESSES, SENSOR_CDC/CDT
#include "../variants/teensy40-11pad/proximity_engine.h"  // the REAL algorithm (fast EMA + intensity)

// ─────────────────────────────────────────────────────────────────────────────
// Proximity tuning rig
//
// Build:  pio run -e proximity_tuning -t upload      (serial @115200)
//
// Goal: find the sensor gain (CDC) that gives the biggest, cleanest spread of
// raw delta across distance — 5 cm → 3 cm → 1 cm → plastic → metal — with the
// idle noise still low. Uses the real proximity engine so `fast`/`int` columns
// match the firmware exactly.
//
// ── PROTOCOL ─────────────────────────────────────────────────────────────────
// `p<n>`  select pad n (serial index, default 0).
//
// For each CDC value in: 6, 8, 10, 12, 16   (10 = current)
//   1. Hand FULLY AWAY, then send `C<val>` (baseline recalibrates clean).
//   2. Hand still away → `n`  (2 s idle-noise scan).
//   3. Hold a finger ~5 cm above the pad, steady → press `5`
//      (waits ~1.2 s, prints one row). Repeat for:
//        `3` = 3 cm   `1` = 1 cm   `L` = finger on the plastic   `M` = on metal
//   → 1 noise line + 5 rows per CDC.
//
// Then paste the whole log back. I'll pick the CDC with the widest monotonic
// 5cm→metal spread and lowest noise, and set proxDeadband / proxMax from it.
//
// `s` toggles a live stream while you position the finger.  `c` config  `?` help
// (CDT is left at the firmware value — one variable at a time.)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t REG_CDC_CFG = 0x5C;
static constexpr uint8_t REG_CDT_CFG = 0x5D;
static constexpr uint8_t REG_ECR     = 0x5E;

static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]),
    MPR121(BOARD_ADDRESSES[1]),
};

static ProximityConfig cfg;
static SensorState     st;

static uint8_t  padIdx = 0;
static uint8_t  cdc    = SENSOR_CDC;
static uint8_t  cdt    = SENSOR_CDT;
static bool     stream = false;

static uint32_t lastUpdMs  = 0;
static uint32_t lastStrmMs = 0;

// ── Steady-distance capture (averaged window) ────────────────────────────────
static bool        capturing = false;
static uint32_t    capEndMs  = 0;
static const char* capLabel  = "-";
static double      sumRaw, sumFast, sumInt;
static float       minRaw, maxRaw;
static uint32_t    capN;
static constexpr uint32_t CAPTURE_MS = 1200;

// ── Noise scan ───────────────────────────────────────────────────────────────
static bool     noiseScan = false;
static uint32_t noiseEndMs = 0;
static float    noiseMin, noiseMax;

static uint8_t boardOf(uint8_t p)     { return SENSORS[p].boardIndex; }
static uint8_t electrodeOf(uint8_t p) { return SENSORS[p].electrode; }

static void applyTiming()
{
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        uint8_t n = SENSE_ELECTRODES[b];
        boards[b].write(REG_ECR, 0x00);
        boards[b].write(REG_CDC_CFG, (uint8_t)(cdc & 0x3F));
        boards[b].write(REG_CDT_CFG, (uint8_t)(((cdt & 0x07) << 5) | 0b001));
        boards[b].write(REG_ECR, (uint8_t)(0b11000000 | n));
        delay(60);
    }
}

static float readRawDelta(uint8_t p)
{
    uint8_t  b = boardOf(p), e = electrodeOf(p);
    int16_t  d = (int16_t)boards[b].baselineData(e) - (int16_t)boards[b].filteredData(e);
    return d < 0 ? 0.0f : (float)d;
}

static void reseed()
{
    seedSensorState(st, readRawDelta(padIdx));
    capturing = false;
    noiseScan = false;
}

static void startCapture(const char* lbl)
{
    capturing = true;
    capLabel  = lbl;
    capEndMs  = millis() + CAPTURE_MS;
    sumRaw = sumFast = sumInt = 0.0;
    capN   = 0;
    minRaw = 1e9f; maxRaw = -1e9f;
    Serial.print(F("# hold steady (")); Serial.print(lbl); Serial.println(F(") ..."));
}

static void printConfig()
{
    Serial.print(F("# pad="));   Serial.print(padIdx);
    Serial.print(F(" (board ")); Serial.print(boardOf(padIdx));
    Serial.print(F(" ELE"));     Serial.print(electrodeOf(padIdx));
    Serial.print(F(")  CDC="));  Serial.print(cdc);
    Serial.print(F(" CDT="));    Serial.print(cdt);
    Serial.print(F("  deadband="));Serial.print(cfg.proxDeadband, 2);
    Serial.print(F(" proxMax=")); Serial.println(cfg.proxMax, 2);
}

static void printHelp()
{
    Serial.println(F("# keys: p<n> pad | C<v> CDC | T<v> CDT | n noise | s stream"));
    Serial.println(F("#       5/3/1 = 5cm/3cm/1cm | L plastic | M metal | c cfg | ?"));
}

static void handleSerial()
{
    if (!Serial.available()) return;
    char c = Serial.read();
    switch (c)
    {
        case 'p': { int v=(int)Serial.parseFloat(); if(v>=0&&v<NUM_SENSORS){padIdx=(uint8_t)v; reseed();} printConfig(); break; }
        case 'C':  cdc=(uint8_t)Serial.parseFloat(); applyTiming(); reseed(); printConfig(); break;
        case 'T':  cdt=(uint8_t)Serial.parseFloat(); applyTiming(); reseed(); printConfig(); break;
        case 's':  stream=!stream; Serial.print(F("# stream ")); Serial.println(stream?F("ON"):F("OFF")); break;
        case 'n':  noiseScan=true; noiseEndMs=millis()+2000; noiseMin=1e9f; noiseMax=-1e9f; Serial.println(F("# NOISE 2s — hand AWAY")); break;
        case '5':  startCapture("5cm");     break;
        case '3':  startCapture("3cm");     break;
        case '1':  startCapture("1cm");     break;
        case 'L': case 'l': startCapture("plastic"); break;
        case 'M': case 'm': startCapture("metal");   break;
        case 'd':  cfg.proxDeadband=Serial.parseFloat(); printConfig(); break;
        case 'x':  cfg.proxMax     =Serial.parseFloat(); printConfig(); break;
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
        if (!boards[b].begin(SENSE_ELECTRODES[b]))
        {
            Serial.print(F("ERROR: MPR121 board ")); Serial.print(b);
            Serial.println(F(" not found"));
        }
    applyTiming();
    reseed();

    Serial.println(F("# Proximity-tuning rig"));
    printHelp();
    printConfig();
    Serial.println(F("# columns:"));
    Serial.println(F("# label  CDC  CDT  meanRaw  minRaw  maxRaw  noise  meanFast  meanInt"));
}

// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    handleSerial();

    uint32_t now = millis();
    if (now - lastUpdMs < UPDATE_MS) return;
    lastUpdMs = now;

    float rawDelta = readRawDelta(padIdx);
    float intensity; bool isTouch;
    updateProximity(rawDelta, now, cfg, st, intensity, isTouch);

    if (noiseScan)
    {
        if (rawDelta < noiseMin) noiseMin = rawDelta;
        if (rawDelta > noiseMax) noiseMax = rawDelta;
        if ((int32_t)(now - noiseEndMs) >= 0)
        {
            Serial.print(F("# NOISE min=")); Serial.print(noiseMin,2);
            Serial.print(F(" max="));        Serial.print(noiseMax,2);
            Serial.print(F("  → proxDeadband ≈ ")); Serial.println(noiseMax*1.5f,2);
            noiseScan = false;
        }
    }

    if (capturing)
    {
        sumRaw  += rawDelta;
        sumFast += st.fast;
        sumInt  += intensity;
        capN++;
        if (rawDelta < minRaw) minRaw = rawDelta;
        if (rawDelta > maxRaw) maxRaw = rawDelta;

        if ((int32_t)(now - capEndMs) >= 0)
        {
            float mR = capN ? (float)(sumRaw  / capN) : 0.0f;
            float mF = capN ? (float)(sumFast / capN) : 0.0f;
            float mI = capN ? (float)(sumInt  / capN) : 0.0f;
            Serial.print(capLabel);          Serial.print('\t');
            Serial.print(cdc);               Serial.print('\t');
            Serial.print(cdt);               Serial.print('\t');
            Serial.print(mR, 2);             Serial.print('\t');
            Serial.print(minRaw, 2);         Serial.print('\t');
            Serial.print(maxRaw, 2);         Serial.print('\t');
            Serial.print(maxRaw - minRaw, 2);Serial.print('\t');
            Serial.print(mF, 2);             Serial.print('\t');
            Serial.println(mI, 3);
            capturing = false;
        }
    }

    if (stream && (now - lastStrmMs >= 50))
    {
        lastStrmMs = now;
        Serial.print(F("raw:"));  Serial.print(rawDelta, 1);
        Serial.print(F(" fast:"));Serial.print(st.fast, 1);
        Serial.print(F(" int:")); Serial.println(intensity, 2);
    }
}
