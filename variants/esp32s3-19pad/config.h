#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32-S3 (LilyGO T8-S3) · 19-pad · 4× MPR121
//
// STATUS: bring-up. main.cpp is currently a CAPACITIVE-SENSE TEST — it inits the
// two sense boards, streams raw + filtered values to Teleplot, runs WiFi + OTA,
// and exposes a small HTTP/JSON endpoint so settings can be tweaked live from a
// browser or a companion app. The sound engine (Mozzi/I2S), LED driving and the
// scale/timbre system get layered on once sensing is dialled in.
//
// Hardware target:
//   • LilyGO T8-S3  — ESP32-S3, 16 MB flash, 8 MB PSRAM, no screen
//   • 4× MPR121 on ONE I²C bus:
//       0x5A, 0x5B  → SENSE  (12 + 7 = 19 capacitive pads)
//       0x5C, 0x5D  → LED    (driven later; ignored by the capsense test)
//
// Edit this file to match your wiring. The test reads SENSE_ADDRESSES[],
// SENSE_ELECTRODES[], SENSE_PADS[] and the SensorSettings defaults below.
// ─────────────────────────────────────────────────────────────────────────────

// ── I²C bus (LilyGO T8-S3) ────────────────────────────────────────────────────
// >>> CONFIRM AGAINST YOUR BOARD <<<  The ESP32-S3 can route I²C to almost any
// GPIO; 8/9 are the Arduino-ESP32 defaults and are broken out on the T8-S3. Avoid
// GPIO 26–37 (SPI flash / octal PSRAM) and the strapping pins (0, 3, 45, 46).
static constexpr uint8_t PIN_I2C_SDA = 8;
static constexpr uint8_t PIN_I2C_SCL = 9;
static constexpr uint32_t I2C_CLOCK  = 400000; // 400 kHz fast-mode

// ── I²S audio out (PCM5102A) — FUTURE (synth port) ───────────────────────────
// Not used by the capsense/LED tests; recorded here as the wiring target. The
// ESP32-S3 routes I²S through its GPIO matrix, so ANY free GPIOs work — there is
// no "WS = BCK + 1" constraint like the RP2040 has. MCLK is optional: strap the
// PCM5102A SCK pin to GND and leave PIN_I2S_MCLK unused. Avoid GPIO 26–37 (flash/
// octal PSRAM), 0/3/45/46 (strapping) and 19/20 (native USB).
// static constexpr uint8_t PIN_I2S_BCK  = 5;   // bit clock  → PCM5102A BCK
// static constexpr uint8_t PIN_I2S_WS   = 6;   // word/LR clock → PCM5102A LRCK
// static constexpr uint8_t PIN_I2S_DATA = 7;   // data       → PCM5102A DIN

// ── Addressable LEDs (WS2812 / SK6812) — FUTURE ──────────────────────────────
// One data line, driven by the S3's RMT peripheral — any free GPIO works. Note
// the data line is 3.3 V: short runs usually clock fine, but a 3.3→5 V level
// shifter (or the "sacrificial first pixel" trick) is the robust fix for flaky
// strips. Keep it clear of the I²C (8/9) and I²S (5/6/7) pins above.
// static constexpr uint8_t PIN_LED_DATA = 21;  // → WS2812 DIN

// ── MPR121 boards ─────────────────────────────────────────────────────────────
// Sense boards first, then LED boards. ADDR strap → address:
//   0x5A = GND   0x5B = VCC   0x5C = SDA   0x5D = SCL
static constexpr uint8_t NUM_SENSE_BOARDS = 2;
static constexpr uint8_t SENSE_ADDRESSES[NUM_SENSE_BOARDS] = { 0x5A, 0x5B };

static constexpr uint8_t NUM_LED_BOARDS = 2;                         // future
static constexpr uint8_t LED_ADDRESSES[NUM_LED_BOARDS] = { 0x5C, 0x5D };

// Sense-electrode count per sense board (contiguous from ELE0). 12 + 7 = 19.
static constexpr uint8_t SENSE_ELECTRODES[NUM_SENSE_BOARDS] = { 12, 7 };

// ── Pad → (sense board, electrode) map ───────────────────────────────────────
// 19 pads. The capsense test reports values in this pad order, so reorder these
// rows to match the physical layout once the shell is wired — the indices you see
// in Teleplot / the web view then line up with the real pads.
struct SenseMap { uint8_t board; uint8_t electrode; };
static constexpr SenseMap SENSE_PADS[] = {
    // board 0 (0x5A) — ELE0–ELE11
    { 0,  0 }, { 0,  1 }, { 0,  2 }, { 0,  3 }, { 0,  4 }, { 0,  5 },
    { 0,  6 }, { 0,  7 }, { 0,  8 }, { 0,  9 }, { 0, 10 }, { 0, 11 },
    // board 1 (0x5B) — ELE0–ELE6
    { 1,  0 }, { 1,  1 }, { 1,  2 }, { 1,  3 }, { 1,  4 }, { 1,  5 }, { 1, 6 },
};
static constexpr uint8_t NUM_SENSORS =
    static_cast<uint8_t>(sizeof(SENSE_PADS) / sizeof(SENSE_PADS[0]));

// ── Sensor tuning defaults ────────────────────────────────────────────────────
// These seed the runtime SensorSettings (below) at boot. CDC=14 is the measured
// sweet spot on the existing builds (near-field/touch instrument — see the Pico
// variant's config notes); re-tune here or live over HTTP once you A/B on the
// real shell. CDT keeps ESI at ~2 ms so future LED PWM stays flicker-free.
static constexpr uint8_t  DEFAULT_CDC        = 14; // charge current 0–63 (gain)
static constexpr uint8_t  DEFAULT_CDT        = 3;  // charge time 0–7
static constexpr uint8_t  DEFAULT_TOUCH_TH   = 40; // hardware touch threshold
static constexpr uint8_t  DEFAULT_RELEASE_TH = 20; // must be < touch threshold

// Mutable settings — one runtime copy lives in main.cpp. This is the seam the web
// interface / companion app writes to; changing a field and re-initialising the
// chips is how a tuning change is applied live. Keep it POD so it can be dumped /
// loaded as JSON (and later persisted to NVS/Preferences) without ceremony.
struct SensorSettings {
    uint8_t cdc;
    uint8_t cdt;
    uint8_t touchTh;
    uint8_t releaseTh;
};
static constexpr SensorSettings DEFAULT_SETTINGS = {
    DEFAULT_CDC, DEFAULT_CDT, DEFAULT_TOUCH_TH, DEFAULT_RELEASE_TH
};

// ── WiFi / OTA / Teleplot host ────────────────────────────────────────────────
// SECRETS (WiFi SSID + password, your Teleplot host IP) live in secrets.h, which
// is git-ignored — copy secrets.example.h → secrets.h and fill it in. If secrets.h
// is absent (fresh clone) the placeholders below apply and WiFi just won't connect
// (sensing + USB Teleplot still work). Keep real creds out of version control.
#if __has_include("secrets.h")
  #include "secrets.h"
#endif

#ifndef WIFI_SSID
  #define WIFI_SSID     "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASSWORD
  #define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif
#ifndef OTA_HOSTNAME
  #define OTA_HOSTNAME  "omniphone-esp32"   // → omniphone-esp32.local (mDNS)
#endif
#ifndef OTA_PASSWORD
  #define OTA_PASSWORD  ""                  // "" = no OTA auth (set one on a shared net)
#endif
// Teleplot over WiFi (UDP): TELEPLOT_HOST = IP of the MACHINE running Teleplot
// (your computer, not the ESP32); "" = off. Port must match Teleplot's listener
// (standalone = 47269). Both also settable live via `host=<ip>` / `host=<ip>:<port>`.
#ifndef TELEPLOT_HOST
  #define TELEPLOT_HOST ""
#endif
#ifndef TELEPLOT_PORT
  #define TELEPLOT_PORT 47269
#endif

// ── Test loop timing ──────────────────────────────────────────────────────────
static constexpr uint32_t SENSE_PERIOD_MS = 20; // I²C read cadence (~50 Hz)
static constexpr uint32_t TELE_PERIOD_MS  = 50; // Teleplot stream cadence (~20 Hz)
