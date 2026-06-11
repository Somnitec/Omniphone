// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32-S3 (T-Display-S3) · MPR121 capsense (raw) + LEDs on + screen
//
//   • Capsense — reads the raw delta (baseline − filtered) for each sense pad
//                (SENSE_PADS) and streams ONLY that to Teleplot as d<pad>.
//   • LEDs     — every pad's mapped LED (SENSE_PADS ledBoard/ledEle) is held ON.
//   • Screen   — the T-Display-S3 panel is powered and filled white.
//
// Build:  pio run -e esp32s3-mpr-test -t upload   (OTA: …-mpr-test-ota)
// Plot:   Teleplot @115200 or socket://omniphone-19pad.local:23 (d<pad>).
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MPR121.h>

#include "config.h"
#include "net_console.h"   // WiFi + OTA + serial-over-WiFi (Console)
#include "screen_status.h" // on-screen WiFi status bar (drawNetStatus)

// T-Display-S3 panel pins not covered by the TFT_eSPI build flags.
static constexpr uint8_t PIN_LCD_POWER = 15;
static constexpr uint8_t PIN_LCD_BL    = 38;
static TFT_eSPI tft;

static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]), MPR121(BOARD_ADDRESSES[1]),
    MPR121(BOARD_ADDRESSES[2]), MPR121(BOARD_ADDRESSES[3]),
};
static bool boardOk[NUM_BOARDS] = { false };

static constexpr uint32_t SENSE_MS = 50;   // capsense stream cadence (~20 Hz)

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

// True if electrode `ele` on this chip is a capacitive-SENSE electrode.
static inline bool isSense(uint8_t board, uint8_t ele) {
    return ele < SENSE_ELECTRODES[board];
}

// Hold every pad's mapped LED ON (full brightness). ELE9/ELE10 are coupled, so
// when both are free they light together; an LED on a sense electrode or ELE0–3
// is skipped (can't be driven). LED_GPIO LEDs are driven straight on the MCU pin.
static void allLedsOn() {
    uint8_t arr[NUM_BOARDS][8] = {};   // per-chip GPIO brightness
    for (uint8_t p = 0; p < NUM_SENSORS; p++) {
        uint8_t lb = SENSE_PADS[p].ledBoard, le = SENSE_PADS[p].ledEle;
        if (le == NO_LED) continue;
        if (lb == LED_GPIO) { analogWrite(le, 255); continue; }
        if (lb >= NUM_BOARDS || !boardOk[lb]) continue;
        if (le == 9 || le == 10) {                      // coupled pair
            if (!isSense(lb, 9))  arr[lb][9 - 4]  = 15;
            if (!isSense(lb, 10)) arr[lb][10 - 4] = 15;
        } else if (le >= 4 && le <= 11 && !isSense(lb, le)) {
            arr[lb][le - 4] = 15;
        }
    }
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
        if (boardOk[b]) boards[b].setLEDs8(arr[b]);
}

// ── Capsense → Teleplot (raw delta only) ──────────────────────────────────────
static void streamSense() {
    static uint8_t filt[NUM_BOARDS][24];
    static uint8_t base[NUM_BOARDS][12];
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (!boardOk[b] || SENSE_ELECTRODES[b] == 0) continue;
        uint8_t n = SENSE_ELECTRODES[b];
        boards[b].burstRead(MPR121Reg::FILT_0L, filt[b], (uint8_t)(2 * n));
        boards[b].burstRead(MPR121Reg::BASE_0,  base[b], n);
    }

    Console.teleBegin();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const SenseMap& m = SENSE_PADS[i];
        int16_t delta = 0;
        if (boardOk[m.board]) {
            uint8_t e = m.electrode;
            int16_t filtered = (int16_t)((uint16_t)filt[m.board][2 * e]
                              | ((uint16_t)(filt[m.board][2 * e + 1] & 0x03) << 8));
            uint16_t baseline = (uint16_t)base[m.board][e] << 2;
            int16_t d = (int16_t)baseline - filtered;
            delta = d < 0 ? 0 : d;
        }
        char nm[8];
        snprintf(nm, sizeof(nm), "d%u", i);
        Console.tele(nm, delta, 0);
    }
    Console.teleEnd();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone (T-Display-S3) MPR121 raw + LEDs on + screen ──"));

    // Screen: power the panel, fill white, backlight on.
    pinMode(PIN_LCD_POWER, OUTPUT);
    digitalWrite(PIN_LCD_POWER, HIGH);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_WHITE);
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK);
    Serial.printf("# I2C on SDA=%u SCL=%u\n", PIN_I2C_SDA, PIN_I2C_SCL);
    scanI2C();

    uint8_t present = 0;
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        boardOk[b] = i2cProbe(BOARD_ADDRESSES[b]);
        if (boardOk[b]) {
            boards[b].begin(SENSE_ELECTRODES[b], DEFAULT_TOUCH_TH, DEFAULT_RELEASE_TH,
                            DEFAULT_CDC, DEFAULT_CDT);
            boards[b].beginLEDs();
            present++;
        }
        Serial.printf("#   chip %u @ 0x%02X  %s  (%u sense)\n", b, BOARD_ADDRESSES[b],
                      boardOk[b] ? "OK" : "*** NOT FOUND", SENSE_ELECTRODES[b]);
    }
    Serial.printf("# %u/%u chips responding; %u pads\n", present, NUM_BOARDS, NUM_SENSORS);

    allLedsOn();        // hold every pad's LED on

    Console.begin();    // WiFi + OTA + remote serial (non-fatal headless)
    Serial.println(F("# screen white; LEDs held on; streaming raw d<pad>"));
}

void loop() {
    Console.handle();    // OTA + remote serial
    drawNetStatus(tft);  // WiFi status bar (self-throttled ~1 Hz)

    static uint32_t lastSense = 0;
    if (millis() - lastSense >= SENSE_MS) {
        lastSense = millis();
        streamSense();
    }
}
