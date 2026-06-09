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
#include <MPR121.h>

#include "config.h"
#include "net_console.h"   // WiFi + OTA + serial-over-WiFi (Console)

// All four boards on the bus: the two sense addresses + the two LED addresses.
static constexpr uint8_t NUM_ALL_BOARDS = NUM_SENSE_BOARDS + NUM_LED_BOARDS;
static constexpr uint8_t ALL_ADDRESSES[NUM_ALL_BOARDS] = {
    SENSE_ADDRESSES[0], SENSE_ADDRESSES[1], LED_ADDRESSES[0], LED_ADDRESSES[1],
};
static MPR121 boards[NUM_ALL_BOARDS] = {
    MPR121(ALL_ADDRESSES[0]), MPR121(ALL_ADDRESSES[1]),
    MPR121(ALL_ADDRESSES[2]), MPR121(ALL_ADDRESSES[3]),
};

// setLEDs8() is indexed by GPIO bit g, driving ELE(g+4). So ELE9 → g=5, ELE10 → g=6.
static constexpr uint8_t G_ELE9  = 5;
static constexpr uint8_t G_ELE10 = 6;
static constexpr uint8_t FADE_STEP_MS = 70;   // per brightness step (0..15)

// Drive ELE9/ELE10 to the given brightness (0..15) on ALL boards at once.
static void setPinsAll(uint8_t bri9, uint8_t bri10) {
    uint8_t bri[8] = { 0 };
    bri[G_ELE9]  = bri9;
    bri[G_ELE10] = bri10;
    for (uint8_t b = 0; b < NUM_ALL_BOARDS; b++) boards[b].setLEDs8(bri);
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
            Console.handle();                                  // keep OTA/monitor alive
            delay(FADE_STEP_MS);
        }
    }
    setPinsAll(0, 0);
    delay(300);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone (ESP32-S3) ELE9/ELE10 LED pin-link test ──"));

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK);

    // Init every board with only ELE0–ELE4 as touch so ELE5–ELE11 (incl. 9/10) are
    // free for the LED driver, then enable the open-drain LED outputs.
    for (uint8_t b = 0; b < NUM_ALL_BOARDS; b++) {
        bool ok = boards[b].begin(/*numElectrodes=*/4,
                                  DEFAULT_TOUCH_TH, DEFAULT_RELEASE_TH,
                                  DEFAULT_CDC, DEFAULT_CDT);
        boards[b].beginLEDs();
        Serial.printf("#   board %u @ 0x%02X  %s\n",
                      b, ALL_ADDRESSES[b],
                      ok ? "init OK" : "*** NOT FOUND — check wiring/addr/power");
    }

    Console.begin();   // WiFi + OTA + remote serial (non-fatal if headless)
    Serial.println(F("# starting fade cycle: ELE9, then ELE10, then both…"));
}

void loop() {
    Console.handle();
    fadePins(9);
    fadePins(10);
    fadePins(0);   // both together
    Console.println(F("# --- cycle complete, repeating ---"));
}
