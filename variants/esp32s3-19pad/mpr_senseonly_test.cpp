// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32-S3 · MPR121 SENSE-ONLY readout (no LEDs, all 12 electrodes)
//
// Diagnostic: every chip is initialised with ALL 12 electrodes (ELE0–ELE11) as
// capacitive inputs and the LED/GPIO driver is NEVER engaged (no beginLEDs). It
// streams the raw delta (baseline − filtered) for every electrode on every chip
// to Teleplot, named <chip><ele> — A0..A11, B0..B11, C0..C11, D0..D11.
//
// Use this to isolate whether the LED driving is disturbing sensing: if an
// electrode that read flat in the LED test now responds here, the LED/GPIO config
// was the cause; if it's still flat, that electrode is a wiring/hardware issue.
//
// Build:  pio run -e esp32s3-mpr-senseonly -t upload   (OTA: …-senseonly-ota)
// Plot:   Teleplot @115200 or socket://omniphone-19pad.local:23 (A0..D11).
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <MPR121.h>

#include "config.h"
#include "net_console.h"   // WiFi + OTA + serial-over-WiFi (Console)

static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]), MPR121(BOARD_ADDRESSES[1]),
    MPR121(BOARD_ADDRESSES[2]), MPR121(BOARD_ADDRESSES[3]),
};
static bool boardOk[NUM_BOARDS] = { false };

static constexpr uint8_t  ALL_ELE  = 12;   // enable every electrode as a sense input
static constexpr uint32_t SENSE_MS = 80;   // ~12 Hz (48 series is a lot for Teleplot)

// ── I2C presence ──────────────────────────────────────────────────────────────
static bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}
static void scanI2C() {
    Serial.println(F("# I2C scan:"));
    uint8_t found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++)
        if (i2cProbe(a)) { Serial.printf("#   found device @ 0x%02X\n", a); found++; }
    if (!found)
        Serial.printf("#   NONE — check SDA=%u/SCL=%u (config.h), 3V3 power, GND, pull-ups.\n",
                      PIN_I2C_SDA, PIN_I2C_SCL);
}

// ── Read all 12 electrodes of every present chip → Teleplot ───────────────────
static void streamAll() {
    static uint8_t filt[NUM_BOARDS][24];
    static uint8_t base[NUM_BOARDS][12];
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (!boardOk[b]) continue;
        boards[b].burstRead(MPR121Reg::FILT_0L, filt[b], 24);
        boards[b].burstRead(MPR121Reg::BASE_0,  base[b], 12);
    }

    Console.teleBegin();
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (!boardOk[b]) continue;
        for (uint8_t e = 0; e < ALL_ELE; e++) {
            int16_t filtered = (int16_t)((uint16_t)filt[b][2 * e]
                              | ((uint16_t)(filt[b][2 * e + 1] & 0x03) << 8));
            uint16_t baseline = (uint16_t)base[b][e] << 2;
            int16_t d = (int16_t)baseline - filtered;
            char nm[5];
            snprintf(nm, sizeof(nm), "%c%u", 'A' + b, e);   // A0 … D11
            Console.tele(nm, d < 0 ? 0 : d, 0);
        }
    }
    Console.teleEnd();
}

// Dump filtered + baseline for all 12 electrodes of each chip to the console.
// Reading the actual values tells dead electrodes apart:
//   • two ADJACENT electrodes with nearly IDENTICAL filtered values that move
//     together when you touch either  → solder bridge / shorted pads.
//   • filtered railed near 0 or ~1023 (and not moving)  → open pin, short to
//     rail, or an electrode loaded with too much capacitance for the fixed CDC.
//   • mid-range filtered but baseline == filtered and no delta on touch → that
//     electrode just isn't coupled to a pad (wiring).
static void printTable() {
    uint8_t filt[24], base[12];
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (!boardOk[b]) continue;
        boards[b].burstRead(MPR121Reg::FILT_0L, filt, 24);
        boards[b].burstRead(MPR121Reg::BASE_0,  base, 12);
        Console.printf("# %c filt:", 'A' + b);
        for (uint8_t e = 0; e < 12; e++)
            Console.printf(" %4u", (uint16_t)filt[2 * e] | ((uint16_t)(filt[2 * e + 1] & 0x03) << 8));
        Console.printf("\n# %c base:", 'A' + b);
        for (uint8_t e = 0; e < 12; e++)
            Console.printf(" %4u", (uint16_t)base[e] << 2);
        Console.println();
    }
    Console.println(F("#   (ELE:    0    1    2    3    4    5    6    7    8    9   10   11)"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone (ESP32-S3) MPR121 SENSE-ONLY (no LEDs, all 12 ELE) ──"));

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK);
    Serial.printf("# I2C on SDA=%u SCL=%u\n", PIN_I2C_SDA, PIN_I2C_SCL);
    scanI2C();

    uint8_t present = 0;
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        boardOk[b] = i2cProbe(BOARD_ADDRESSES[b]);
        if (boardOk[b]) {
            // ALL 12 electrodes as sense inputs; LED/GPIO driver left untouched.
            boards[b].begin(ALL_ELE, DEFAULT_TOUCH_TH, DEFAULT_RELEASE_TH,
                            DEFAULT_CDC, DEFAULT_CDT);
            present++;
        }
        Serial.printf("#   chip %c @ 0x%02X  %s\n", 'A' + b, BOARD_ADDRESSES[b],
                      boardOk[b] ? "OK (12 ELE sense, no LEDs)" : "*** NOT FOUND");
    }
    Serial.printf("# %u/%u chips responding; streaming A0..D11 raw delta\n",
                  present, NUM_BOARDS);

    Console.begin();   // WiFi + OTA + remote serial (non-fatal headless)
}

void loop() {
    Console.handle();

    static uint32_t lastSense = 0;
    if (millis() - lastSense >= SENSE_MS) {
        lastSense = millis();
        streamAll();
    }

    // Periodic raw filtered/baseline table → spot bridged/railed electrodes.
    static uint32_t lastTable = 0;
    if (millis() - lastTable >= 2000) {
        lastTable = millis();
        printTable();
    }
}
