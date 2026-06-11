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
//   • 4× MPR121 on ONE I²C bus — chips A–D = 0x5A/0x5B/0x5C/0x5D. Every chip both
//     senses and drives LEDs (its 12 electrodes split per chip) — see SENSE_PADS.
//
// Edit this file to match your wiring. The test reads BOARD_ADDRESSES[],
// SENSE_ELECTRODES[], SENSE_PADS[] and the SensorSettings defaults below.
// ─────────────────────────────────────────────────────────────────────────────

// ── I²C bus (LilyGO T8-S3) ────────────────────────────────────────────────────
// >>> CONFIRM AGAINST YOUR BOARD <<<  The ESP32-S3 can route I²C to almost any
// GPIO; 8/9 are the Arduino-ESP32 defaults and are broken out on the T8-S3. Avoid
// GPIO 26–37 (SPI flash / octal PSRAM) and the strapping pins (0, 3, 45, 46).
static constexpr uint8_t PIN_I2C_SDA = 18;
static constexpr uint8_t PIN_I2C_SCL = 17;
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

// ── Battery voltage sensing ──────────────────────────────────────────────────
// Read pack voltage on an ADC1 pin. ADC1 (GPIO1–10 on the S3) keeps working while
// WiFi is on; ADC2 does NOT — so use an ADC1 pin. GPIO4 = ADC1_CH3. Wire the
// battery through a resistor divider to this pin; BATTERY_DIVIDER = Vbat / Vpin
// (2.0 for two equal resistors). analogReadMilliVolts() applies the chip's eFuse
// ADC calibration, so the reading is already in millivolts at the pin.
// IMPORTANT: this pin must NOT appear in the native-touch pin list (no capsense).
#define BATTERY_ADC_PIN  4
#define BATTERY_DIVIDER  2.0f

// ── MPR121 boards ─────────────────────────────────────────────────────────────
// 4× MPR121, all identical and interchangeable. Every chip has 12 electrodes
// (ELE0–ELE11); on each chip the electrodes are split between capacitive SENSING
// and LED driving as needed — there is NO fixed "sense board" vs "LED board".
// ADDR strap → address:  0x5A = GND   0x5B = VCC   0x5C = SDA   0x5D = SCL
static constexpr uint8_t NUM_BOARDS = 4;
enum Chip : uint8_t { A, B, C, D };   // chip letters → indices (A=0x5A … D=0x5D)
static constexpr uint8_t BOARD_ADDRESSES[NUM_BOARDS] = { 0x5A, 0x5B, 0x5C, 0x5D };

// (How many electrodes to enable per chip is DERIVED from the pad map below —
//  see SENSE_ELECTRODES just after the array. Edit only SENSE_PADS.)

// ── Pad map: SENSE electrode + LED electrode per pad ─────────────────────────
// 19 pads in a symmetrical hex grid (rows of 3-4-5-4-3). The array is laid out in
// the same shape so it reads as the physical board. Each pad has a capacitive
// SENSE electrode and an LED electrode, each on some chip A–D (chip.ELE).
//
//   pad index            sense chip.ELE          LED chip.ELE
//     00 01 02            A0 A1 A2                A5 A6 A7
//    03 04 05 06         A3 A4 B0 B1             A8 A9 B5 B6
//   07 08 09 10 11      B2 B3 B4 C0 C1          B7 B8 B9 C5 C6
//    12 13 14 15         C2 C3 C4 D0             C7 C8 C9 D4
//     16 17 18            D1 D2 D3                D5 D6 D7
//
// Entry = { senseChip, senseEle,  ledChip, ledEle }.
//   • ledBoard = a chip (A–D)  → ledEle is an MPR121 electrode on that chip.
//   • ledBoard = LED_GPIO      → ledEle is a direct ESP32-S3 GPIO pin (LEDC PWM /
//                                 digital out), not an electrode.
//   • ledEle  = NO_LED         → this pad has no LED.
// >>> CONFIRM AGAINST PCB <<< — placeholder allocation; retarget to the real
// wiring (the grids above stay the at-a-glance overview).
// NOTE: ELE9 is hardware-coupled to ELE10 (driver fault) — an LED on ELE9 drives
// the 9/10 pair together; see the LED test. Pads 04/09/14 use ELE9 here.
static constexpr uint8_t NO_LED   = 0xFF;
static constexpr uint8_t LED_GPIO = 0xFE; // ledBoard sentinel: LED is on an MCU GPIO pin
struct SenseMap { uint8_t board; uint8_t electrode; uint8_t ledBoard; uint8_t ledEle; };
static constexpr SenseMap SENSE_PADS[] = {
    /* 00 */ { B,0, C,8 }, /* 01 */ { C,1, C,5 }, /* 02 */ { C,0, C,4 },
    /* 03 */ { B,2, C,10 }, /* 04 */ { B,1, C,7 }, /* 05 */ { D,6, D,11 }, /* 06 */ { D,7, D,8 },
    /* 07 */ { B,5, C,11 }, /* 08 */ { B,4, B,10 }, /* 09 */ { B,3, C,6 }, /* 10 */ { D,1, D,9 }, /* 11 */ { D,0, A,8 },
    /* 12 */ { B,7, B,11 }, /* 13 */ { B,6, A,4 }, /* 14 */ { D,5, A,6 }, /* 15 */ { D,4, A,11 },
    /* 16 */ { B,8, A,7 }, /* 17 */ { D,3, A,5 }, /* 18 */ { D,2, A,9 },
};
static constexpr uint8_t NUM_SENSORS =
    static_cast<uint8_t>(sizeof(SENSE_PADS) / sizeof(SENSE_PADS[0]));

// ── Electrodes to enable per chip — DERIVED from SENSE_PADS ───────────────────
// The MPR121 enables electrodes contiguously from ELE0, so the count to turn on
// for a chip = (highest sense electrode used on it) + 1, or 0 if it senses
// nothing. Computed at compile time from the pad map, so the firmware only ever
// enables exactly the electrodes the wiring uses — edit SENSE_PADS and this
// follows automatically. (Recursive form = valid constexpr on any C++ standard.)
constexpr uint8_t senseEnabledCount(uint8_t chip, uint8_t i = 0, uint8_t hi = 0) {
    return i >= NUM_SENSORS ? hi
         : senseEnabledCount(chip, (uint8_t)(i + 1),
               (SENSE_PADS[i].board == chip && (uint8_t)(SENSE_PADS[i].electrode + 1) > hi)
                   ? (uint8_t)(SENSE_PADS[i].electrode + 1) : hi);
}
static constexpr uint8_t SENSE_ELECTRODES[NUM_BOARDS] = {
    senseEnabledCount(A), senseEnabledCount(B), senseEnabledCount(C), senseEnabledCount(D),
};

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
