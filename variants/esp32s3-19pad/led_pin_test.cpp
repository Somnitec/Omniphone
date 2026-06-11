// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32-S3 · MPR121 ELE9/ELE10 LED PIN-LINK TEST (all 4 boards)
//
// The Teensy/Pico builds hit a fault where the ELE9 and ELE10 open-drain LED
// drivers misbehaved / tracked each other (see the project notes: the fix was a
// HW rework + an ELE9←ELE10 firmware mirror). This sketch checks whether the four
// MPR121s on the ESP32-S3 bus show the same coupling, by driving those two pins
// in isolation and together on EVERY board and announcing each step:
//
//   1. fade ELE9  up→down   (ELE10 off)
//   2. fade ELE10 up→down   (ELE9  off)
//   3. fade ELE9 + ELE10 together
//
// Watch the actual LEDs: if pin 9 lights when only 10 is driven (or vice-versa),
// or one never lights, the pins are linked / the driver is faulty on that board.
// Output goes to USB serial AND the remote serial monitor (socket://...:23).
//
// Build:  pio run -e esp32s3-led-test -t upload
//
// NOTE: ELE9/ELE10 are driven as LED (GPIO) outputs here, so each board is inited
// with only ELE0–ELE4 as touch electrodes (frees ELE5–ELE11 for the LED driver).
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

// All four MPR121s on the bus (one uniform list — every chip can sense and drive
// LEDs). This test drives the LED pins on every board at once.
static constexpr uint8_t NUM_ALL_BOARDS = NUM_BOARDS;
static MPR121 boards[NUM_ALL_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]), MPR121(BOARD_ADDRESSES[1]),
    MPR121(BOARD_ADDRESSES[2]), MPR121(BOARD_ADDRESSES[3]),
};
static bool boardOk[NUM_ALL_BOARDS] = { false };

// Real presence check — the MPR121 has no WHO_AM_I, so begin() can't tell us a
// board is there. A bare address write ACKs (0) only if a chip answers.
static bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// List every address that ACKs. If this finds NOTHING the bus itself is the
// problem — wrong SDA/SCL pins (config.h), no power, or missing pull-ups. If it
// finds chips at unexpected addresses, fix the ADDR straps / *_ADDRESSES[].
static void scanI2C() {
    Serial.println(F("# I2C scan:"));
    uint8_t found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++)
        if (i2cProbe(a)) { Serial.printf("#   found device @ 0x%02X\n", a); found++; }
    if (!found)
        Serial.printf("#   NONE — check SDA=%u/SCL=%u (config.h), 3V3 power, GND, pull-ups.\n",
                      PIN_I2C_SDA, PIN_I2C_SCL);
}

// setLEDs8() is indexed by GPIO bit g, driving ELE(g+4). So ELE9 → g=5, ELE10 → g=6.
static constexpr uint8_t G_ELE9  = 5;
static constexpr uint8_t G_ELE10 = 6;
static constexpr uint8_t FADE_STEP_MS = 70;   // per brightness step (0..15)

// Drive ELE9/ELE10 to the given brightness (0..15) on ALL boards at once.
static void setPinsAll(uint8_t bri9, uint8_t bri10) {
    uint8_t bri[8] = { 0 };
    bri[G_ELE9]  = bri9;
    bri[G_ELE10] = bri10;
    for (uint8_t b = 0; b < NUM_ALL_BOARDS; b++)
        if (boardOk[b]) boards[b].setLEDs8(bri);   // skip absent boards (no error spam)
}

// Fade one or both pins 0→15→0. `which`: 9, 10, or 0 (= both together).
static void fadePins(uint8_t which) {
    Console.printf("# fading %s on all %u boards\n",
                   which == 9 ? "ELE9" : which == 10 ? "ELE10" : "ELE9 + ELE10 (together)",
                   NUM_ALL_BOARDS);
    for (int dir = 0; dir < 2; dir++) {
        for (int s = 0; s <= 15; s++) {
            uint8_t v = (uint8_t)(dir == 0 ? s : 15 - s);
            uint8_t b9  = (which == 9  || which == 0) ? v : 0;
            uint8_t b10 = (which == 10 || which == 0) ? v : 0;
            setPinsAll(b9, b10);
            Console.printf(">ELE9:%u\n>ELE10:%u\n", b9, b10);  // Teleplot trace
            Console.handle(); drawNetStatus(tft);                                  // keep OTA/monitor alive
            delay(FADE_STEP_MS);
        }
    }
    setPinsAll(0, 0);
    delay(300);
}

// ── Per-pad LED sweep (uses the SENSE_PADS LED map) ───────────────────────────
static constexpr uint8_t PAD_FADE_STEP_MS = 20;

// Set one pad's mapped LED to brightness 0..15 (all other LEDs on its chip off).
static void setPadLed(uint8_t pad, uint8_t bri) {
    uint8_t lb = SENSE_PADS[pad].ledBoard;
    uint8_t le = SENSE_PADS[pad].ledEle;
    if (le == NO_LED) return;
    if (lb == LED_GPIO) {                       // LED on a direct MCU GPIO pin
        analogWrite(le, (int)bri * 17);         // 0..15 → 0..255
        return;
    }
    if (lb >= NUM_BOARDS || !boardOk[lb]) return;
    uint8_t arr[8] = { 0 };
    if (le >= 4 && le <= 11) arr[le - 4] = bri; // ELE0–3 have no LED driver
    if (le == 9 || le == 10) arr[9 - 4] = arr[10 - 4] = bri; // coupled pair
    boards[lb].setLEDs8(arr);
}

// Fade each pad's LED up then down, pad 0 → NUM_SENSORS-1, one at a time.
static void fadeAllPads() {
    Console.println(F("# sweep: fade each pad LED 0..18 up/down (per-pad LED map)"));
    for (uint8_t p = 0; p < NUM_SENSORS; p++) {
        if (SENSE_PADS[p].ledEle == NO_LED) continue;
        Console.printf("# pad %u LED  (chip %u ELE%u)\n",
                       p, SENSE_PADS[p].ledBoard, SENSE_PADS[p].ledEle);
        for (int v = 0;  v <= 15; v++) { setPadLed(p, (uint8_t)v); Console.handle(); drawNetStatus(tft); delay(PAD_FADE_STEP_MS); }
        for (int v = 15; v >= 0; v--) { setPadLed(p, (uint8_t)v); Console.handle(); drawNetStatus(tft); delay(PAD_FADE_STEP_MS); }
        setPadLed(p, 0);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone (ESP32-S3) ELE9/ELE10 LED pin-link test ──"));

    // Screen: power the panel, fill white, backlight on (for the WiFi status bar).
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

    // Probe each board for an ACK; only init/drive the ones that actually answer.
    // Init with only ELE0–ELE4 as touch so ELE5–ELE11 (incl. 9/10) are free for the
    // LED driver, then enable the open-drain LED outputs.
    uint8_t present = 0;
    for (uint8_t b = 0; b < NUM_ALL_BOARDS; b++) {
        boardOk[b] = i2cProbe(BOARD_ADDRESSES[b]);
        if (boardOk[b]) {
            boards[b].begin(/*numElectrodes=*/4, DEFAULT_TOUCH_TH, DEFAULT_RELEASE_TH,
                            DEFAULT_CDC, DEFAULT_CDT);
            boards[b].beginLEDs();
            present++;
        }
        Serial.printf("#   board %u @ 0x%02X  %s\n", b, BOARD_ADDRESSES[b],
                      boardOk[b] ? "OK" : "*** NOT FOUND — check wiring/addr/power");
    }
    Serial.printf("# %u/%u boards responding\n", present, NUM_ALL_BOARDS);

    Console.begin();   // WiFi + OTA + remote serial (non-fatal if headless)
    Serial.println(F("# starting fade cycle: ELE9, then ELE10, then both…"));
}

void loop() {
    Console.handle(); drawNetStatus(tft);
    fadeAllPads();    // first: sweep every pad's LED 0..18 in turn
    fadePins(9);      // then: the ELE9/ELE10 coupling test
    fadePins(10);
    fadePins(0);   // both together
    Console.println(F("# --- cycle complete, repeating ---"));
}
