// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32-S3 (T-Display-S3) · 19-pad instrument
//
//   • Sensing  — 4× MPR121 (SENSE_PADS map), per-pad proximity → voice amplitude,
//                with per-pad LED feedback.
//   • Synth    — 19 voices over I²S (PCM5102A), 32-bit / 44.1 kHz (the S3 needs
//                32-bit frames or the DAC stays silent). Selectable SOUND SET
//                (note layout) and TIMBRE (waveform).
//   • Screen   — left: sound set  < name > ; right: timbre  < name >  (tap the
//                arrows on the touch screen to change). Plus CPU / MEM / BAT / the
//                last serial line, and an optional WiFi bar at the bottom.
//   • WiFi/OTA — shared net_console.
//
// I²S → PCM5102A: BCK=1, LRCK=2, DIN=10 (SCK→GND).
// Build:  pio run -e esp32s3-19pad -t upload   (OTA: esp32s3-19pad-ota)
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <MPR121.h>
#include "driver/i2s.h"

#include "config.h"
#include "net_console.h"
#include "screen_status.h"

// ── Feature flags ─────────────────────────────────────────────────────────────
#define SHOW_WIFI_BAR 1   // 0 = hide the WiFi diag bar at the bottom
#define DRIVE_LEDS    1   // 0 = don't drive LEDs (if they disturb sensing)

// Performance mode: a 5-second hold anywhere on the touch screen toggles it —
// WiFi off + all diagnostics hidden (just the instrument). Persisted in NVS, so it
// survives reboots. Held again → everything back on.
static Preferences prefs;
static bool perfMode = false;

// ── I²S → PCM5102A pins (BCK/LRCK/DIN labels) ────────────────────────────────
static constexpr int PIN_I2S_BCK  = 1;
static constexpr int PIN_I2S_LRCK = 2;
static constexpr int PIN_I2S_DIN  = 10;

// ── T-Display-S3 panel + touch pins ──────────────────────────────────────────
static constexpr uint8_t PIN_LCD_POWER = 15;
static constexpr uint8_t PIN_LCD_BL    = 38;
static constexpr uint8_t PIN_TOUCH_RST = 21;
static constexpr uint8_t CST816_ADDR   = 0x15;   // touch ctrl, shares the I²C bus
// Safeguard: jumper this pin to GND at boot to FORCE diagnostics/WiFi on this
// boot, ignoring a saved performance mode — your OTA escape hatch if the board
// ever boots into WiFi-off. GPIO16 is broken out (it's the touch INT line, which
// we don't use — touch is read over I²C — so it's free to borrow here).
static constexpr uint8_t PIN_SAFEGUARD = 16;
static constexpr bool    TOUCH_FLIP    = false;  // flip the arrow zones if mirrored
static TFT_eSPI tft;

// ── Audio engine ─────────────────────────────────────────────────────────────
static constexpr int      SAMPLE_RATE = 44100;
static constexpr int      BLOCK       = 256;
static constexpr uint16_t TABLE_SIZE  = 1024;
static constexpr int16_t  VOICE_AMP   = 9000;   // per-voice peak
static constexpr int      AMP_SLEW_SHIFT = 6;   // amp one-pole (larger = slower/smoother)
// Soft limiter on the voice sum: a single voice stays ~linear/clean, while many
// voices saturate smoothly (tanh) instead of hard-clipping into harsh distortion.
static constexpr float    MIX_DRIVE   = 1.0f / 26000.0f;  // pre-saturation gain
static constexpr float    MIX_OUT     = 30000.0f;         // post-saturation level
static int16_t  WAVE[4][TABLE_SIZE];            // 0 sine, 1 saw, 2 square, 3 triangle

static uint32_t phase[NUM_SENSORS]    = {0};
static uint32_t phaseInc[NUM_SENSORS] = {0};
// Amplitude in Q16 (0..65536 = 0..1.0). High-res so the per-sample slew has no
// integer dead-zone → no zipper/stepping as the volume changes.
static int32_t  ampF[NUM_SENSORS]    = {0};
static volatile int32_t targetF[NUM_SENSORS] = {0};
static volatile uint8_t audioLoadPct = 0;
static bool i2sOk = false;

// ── Sound sets (note layouts) and timbres (waveforms) ────────────────────────
// Note layouts in hex-grid order (rows 3-4-5-4-3); pad 9 = the centre.
//   00 01 02 / 03 04 05 06 / 07 08 09 10 11 / 12 13 14 15 / 16 17 18
struct SoundSet { const char* name; uint8_t midi[NUM_SENSORS]; };
static const SoundSet SOUND_SETS[] = {
    // Accordion / Wicki-Hayden isomorphic layout (one octave lower)
    { "Accordion",  { 62,64,66, 55,57,59,61, 48,50,52,54,56, 41,43,45,47, 34,36,38 } },
    // Chromatic, ascending (one octave lower)
    { "Chromatic",  { 48,49,50, 51,52,53,54, 55,56,57,58,59, 60,61,62,63, 64,65,66 } },
    // C-major pentatonic, ascending (one octave lower)
    { "Pentatonic", { 36,38,40, 43,45,48,50, 52,55,57,60,62, 64,67,69,72, 74,76,79 } },
    // Overtones — every note is a harmonic of C2(36) at the centre, so all
    // combinations fuse consonantly. Lower harmonics in/near the centre.
    { "Overtones",  { 76,78,79, 67,70,72,74, 55,60,36,48,64, 67,70,72,80, 82,84,86 } },
    // Hangdrum — central low "ding" (D2=38) ringed by a D-minor tone field.
    { "Hangdrum",   { 60,62,65, 53,55,57,58, 48,50,38,45,46, 52,53,55,57, 60,62,65 } },
    // Triads — stacked thirds ascending across the grid: any 3 in a row spell a
    // triad, and it transposes low corner → high corner.
    { "Triads",     { 24,28,31, 35,38,41,45, 48,52,55,59,62, 66,69,72,76, 79,83,86 } },
};
static constexpr uint8_t NUM_SOUND_SETS = sizeof(SOUND_SETS) / sizeof(SOUND_SETS[0]);

struct Timbre { const char* name; uint8_t wave; };
static const Timbre TIMBRES[] = {
    { "Sine", 0 }, { "Saw", 1 }, { "Square", 2 }, { "Triangle", 3 },
};
static constexpr uint8_t NUM_TIMBRES = sizeof(TIMBRES) / sizeof(TIMBRES[0]);

static volatile uint8_t activeSet    = 0;
static volatile uint8_t activeTimbre = 0;

// ── Proximity tuning ──────────────────────────────────────────────────────────
static constexpr float PROX_DEADBAND = 4.0f;
static constexpr float PROX_MAX      = 25.0f;
static constexpr float PROX_SMOOTH_K = 0.40f;
static float proxSmooth[NUM_SENSORS] = {0};

// ── Audio task (32-bit stereo) ────────────────────────────────────────────────
static void audioTask(void*) {
    static int32_t tx[BLOCK * 2];
    const uint32_t blockUs = (uint32_t)((uint64_t)BLOCK * 1000000ULL / SAMPLE_RATE);
    for (;;) {
        uint32_t t0 = micros();
        const int16_t* wt = WAVE[TIMBRES[activeTimbre].wave];
        for (int f = 0; f < BLOCK; f++) {
            int32_t acc = 0;
            for (uint8_t v = 0; v < NUM_SENSORS; v++) {
                ampF[v] += (targetF[v] - ampF[v]) >> AMP_SLEW_SHIFT;   // Q16 slew
                int32_t a = ampF[v];
                if (a > 0) acc += ((int32_t)wt[phase[v] >> 22] * a) >> 16;
                phase[v] += phaseInc[v];
            }
            int32_t out = (int32_t)(tanhf((float)acc * MIX_DRIVE) * MIX_OUT); // soft limit
            tx[2 * f] = tx[2 * f + 1] = out << 16;                // → top of 32-bit frame
        }
        uint32_t genUs = micros() - t0;
        audioLoadPct = (uint8_t)(genUs >= blockUs ? 100 : (genUs * 100) / blockUs);
        size_t wrote;
        i2s_write(I2S_NUM_0, tx, sizeof(tx), &wrote, portMAX_DELAY);
    }
}

static inline uint32_t midiToInc(uint8_t m) {
    float hz = 440.0f * powf(2.0f, (m - 69) / 12.0f);
    return (uint32_t)((double)hz * 4294967296.0 / SAMPLE_RATE);
}
static void applySoundSet(uint8_t idx) {
    activeSet = idx;
    for (uint8_t v = 0; v < NUM_SENSORS; v++)
        phaseInc[v] = midiToInc(SOUND_SETS[idx].midi[v]);
}

static void audioBegin() {
    for (uint16_t i = 0; i < TABLE_SIZE; i++) {
        float t = (float)i / TABLE_SIZE;
        WAVE[0][i] = (int16_t)(sinf(2.0f * (float)M_PI * t) * VOICE_AMP);    // sine
        WAVE[1][i] = (int16_t)((2.0f * t - 1.0f) * VOICE_AMP);               // saw
        WAVE[2][i] = (t < 0.5f) ? VOICE_AMP : -VOICE_AMP;                    // square
        WAVE[3][i] = (int16_t)((t < 0.5f ? (4.0f * t - 1.0f)                 // triangle
                                         : (3.0f - 4.0f * t)) * VOICE_AMP);
    }
    applySoundSet(0);

    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;   // 64fs — the S3/DAC fix
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = 0;
    cfg.dma_buf_count        = 8;     // more headroom so a hiccup can't underrun
    cfg.dma_buf_len          = BLOCK;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;
    i2sOk = (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) == ESP_OK);
    Console.printf("# I2S install: %s (32-bit %d Hz, BCK=%d LRCK=%d DIN=%d)\n",
                   i2sOk ? "OK" : "FAIL", SAMPLE_RATE, PIN_I2S_BCK, PIN_I2S_LRCK, PIN_I2S_DIN);

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = I2S_PIN_NO_CHANGE;
    pins.bck_io_num   = PIN_I2S_BCK;
    pins.ws_io_num    = PIN_I2S_LRCK;
    pins.data_out_num = PIN_I2S_DIN;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    i2s_set_pin(I2S_NUM_0, &pins);

    xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, 3, nullptr, 1);
}

// ── Sensing (MPR121) ──────────────────────────────────────────────────────────
static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]), MPR121(BOARD_ADDRESSES[1]),
    MPR121(BOARD_ADDRESSES[2]), MPR121(BOARD_ADDRESSES[3]),
};
static bool boardOk[NUM_BOARDS] = { false };

static bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}
static inline bool isSense(uint8_t board, uint8_t ele) { return ele < SENSE_ELECTRODES[board]; }

static void sensorsBegin() {
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        boardOk[b] = i2cProbe(BOARD_ADDRESSES[b]);
        if (boardOk[b]) {
            // Fast init: config + start scanning (CL=10 auto-baseline). Skip the
            // ~200 ms lockBaseline — the slow falling-filter keeps proximity deltas.
            boards[b].beginConfig(SENSE_ELECTRODES[b], DEFAULT_TOUCH_TH, DEFAULT_RELEASE_TH,
                                  DEFAULT_CDC, DEFAULT_CDT);
            boards[b].startScanning(SENSE_ELECTRODES[b], 0b10);
            boards[b].beginLEDs();
        }
        Console.printf("#   chip %c @ 0x%02X %s\n", 'A' + b, BOARD_ADDRESSES[b],
                       boardOk[b] ? "OK" : "NOT FOUND");
    }
}

static void readSensors() {
    static uint8_t filt[NUM_BOARDS][24];
    static uint8_t base[NUM_BOARDS][12];
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (!boardOk[b] || SENSE_ELECTRODES[b] == 0) continue;
        uint8_t n = SENSE_ELECTRODES[b];
        boards[b].burstRead(MPR121Reg::FILT_0L, filt[b], (uint8_t)(2 * n));
        boards[b].burstRead(MPR121Reg::BASE_0,  base[b], n);
    }
    uint8_t led[NUM_BOARDS][8];
    memset(led, 0, sizeof(led));

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const SenseMap& m = SENSE_PADS[i];
        float delta = 0.0f;
        if (boardOk[m.board] && m.electrode < SENSE_ELECTRODES[m.board]) {
            uint8_t e = m.electrode;
            uint16_t f = (uint16_t)filt[m.board][2 * e]
                       | ((uint16_t)(filt[m.board][2 * e + 1] & 0x03) << 8);
            uint16_t bl = (uint16_t)base[m.board][e] << 2;
            delta = (float)((int)bl - (int)f);
        }
        float norm = (delta - PROX_DEADBAND) / (PROX_MAX - PROX_DEADBAND);
        if (norm < 0.0f) norm = 0.0f; else if (norm > 1.0f) norm = 1.0f;
        proxSmooth[i] += (norm - proxSmooth[i]) * PROX_SMOOTH_K;
        targetF[i] = (int32_t)(proxSmooth[i] * 65536.0f);   // Q16

#if DRIVE_LEDS
        uint8_t lb = SENSE_PADS[i].ledBoard, le = SENSE_PADS[i].ledEle;
        uint8_t bri = (uint8_t)(proxSmooth[i] * 15.0f + 0.5f);
        if (le == NO_LED) {
        } else if (lb == LED_GPIO) {
            analogWrite(le, (int)(proxSmooth[i] * 255.0f + 0.5f));
        } else if (lb < NUM_BOARDS && boardOk[lb]) {
            if (le == 9 || le == 10) {
                if (!isSense(lb, 9))  led[lb][9 - 4]  = bri;
                if (!isSense(lb, 10)) led[lb][10 - 4] = bri;
            } else if (le >= 4 && le <= 11 && !isSense(lb, le)) {
                led[lb][le - 4] = bri;
            }
        }
#endif
    }
#if DRIVE_LEDS
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
        if (boardOk[b]) boards[b].setLEDs8(led[b]);
#endif
}

// ── Touch selectors (CST816, on the same I²C bus) ─────────────────────────────
static bool touchOk = false;
static void touchWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(CST816_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static bool touchRead6(uint8_t* d) {
    Wire.beginTransmission(CST816_ADDR); Wire.write(0x01);
    if (Wire.endTransmission(true) != 0) return false;
    Wire.requestFrom(CST816_ADDR, (uint8_t)6);
    if (Wire.available() < 6) { while (Wire.available()) Wire.read(); return false; }
    for (uint8_t i = 0; i < 6; i++) d[i] = Wire.read();
    return true;
}
static void touchBegin() {
    pinMode(PIN_TOUCH_RST, OUTPUT);
    digitalWrite(PIN_TOUCH_RST, LOW);  delay(10);
    digitalWrite(PIN_TOUCH_RST, HIGH); delay(50);
    touchOk = i2cProbe(CST816_ADDR);
    if (touchOk) touchWrite(0xFE, 0x01);   // disable auto-sleep so it keeps ACKing
    Console.printf("# touch (CST816): %s\n", touchOk ? "OK" : "not found");
}
static bool dirty = true;   // screen selectors need a redraw

static void togglePerfMode() {
    perfMode = !perfMode;
    prefs.putBool("perf", perfMode);                 // persist (no reboot)
    Console.printf("# performance mode %s\n", perfMode ? "ON (WiFi off)" : "OFF");
    if (perfMode) Console.disable();                 // WiFi off, hide diagnostics
    else          Console.begin();                   // re-enable (non-blocking)
    tft.fillScreen(TFT_WHITE);                        // clear the old layout
    dirty = true;                                     // redraw selectors
}

// A short TAP selects (the screen splits into 4 arrow zones along the long axis:
// set< | set> | timbre< | timbre>). A 5-SECOND HOLD anywhere toggles perf mode.
static constexpr uint32_t HOLD_MS = 5000;
static void pollTouch() {
    if (!touchOk) return;
    uint8_t d[6];
    bool ok = touchRead6(d);
    bool touched = ok && (d[1] & 0x0F) > 0;
    uint16_t y = ((uint16_t)(d[4] & 0x0F) << 8) | d[5];   // long axis 0..319

    static bool     was = false;
    static uint32_t downMs = 0;
    static uint8_t  downZone = 0;
    static bool     held = false;

    if (touched && !was) {                       // finger down
        downMs = millis(); held = false;
        uint8_t z = y / (320 / 4); if (z > 3) z = 3;
        downZone = TOUCH_FLIP ? (3 - z) : z;
    } else if (touched && !held && millis() - downMs >= HOLD_MS) {
        held = true;                             // 5-s hold reached
        togglePerfMode();                        // (reboots)
    } else if (!touched && was) {                // finger up
        if (!held && millis() - downMs < 800) {  // short tap → select
            switch (downZone) {
                case 0: applySoundSet((activeSet + NUM_SOUND_SETS - 1) % NUM_SOUND_SETS); break;
                case 1: applySoundSet((activeSet + 1) % NUM_SOUND_SETS); break;
                case 2: activeTimbre = (activeTimbre + NUM_TIMBRES - 1) % NUM_TIMBRES; break;
                case 3: activeTimbre = (activeTimbre + 1) % NUM_TIMBRES; break;
            }
            dirty = true;
            Console.printf("# set=%s  timbre=%s\n",
                           SOUND_SETS[activeSet].name, TIMBRES[activeTimbre].name);
        }
    }
    was = touched;
}

// ── Screen ────────────────────────────────────────────────────────────────────
static float readBattery() {
    return (float)analogReadMilliVolts(BATTERY_ADC_PIN) * BATTERY_DIVIDER / 1000.0f;
}

// Bottom of the stats block (WiFi bar, or screen edge). Stats rows stack up from
// here; the selectors sit just above the top stats row (BAT).
static inline bool wifiBarShown() { return SHOW_WIFI_BAR && !perfMode; }
static inline int16_t statsBase() { return wifiBarShown() ? (tft.height() - 20) : tft.height(); }

// Two selectors centred above the battery row: left = sound set, right = timbre.
static void drawSelectors() {
    int16_t halfW  = tft.width() / 2;
    int16_t batY   = statsBase() - 18 * 3;   // BAT row (drawStats idx 2)
    int16_t nameY  = batY - 30;
    int16_t labelY = batY - 48;
    tft.fillRect(0, labelY - 2, tft.width(), batY - (labelY - 2), TFT_WHITE);
    tft.drawFastVLine(halfW, labelY, (nameY + 26) - labelY, TFT_LIGHTGREY);
    tft.setTextDatum(TC_DATUM);

    tft.setTextFont(2);
    tft.setTextColor(TFT_DARKGREY, TFT_WHITE);
    tft.drawString("SOUND",  halfW / 2,         labelY);
    tft.drawString("TIMBRE", halfW + halfW / 2, labelY);

    char s[24];
    tft.setTextFont(4);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    snprintf(s, sizeof(s), "<%s>", SOUND_SETS[activeSet].name);   tft.drawString(s, halfW / 2,         nameY);
    snprintf(s, sizeof(s), "<%s>", TIMBRES[activeTimbre].name);   tft.drawString(s, halfW + halfW / 2, nameY);
    tft.setTextDatum(TL_DATUM);
}

static void drawRow(int idx, const char* s) {
    const int16_t lh = 18;
    int16_t base = statsBase();
    int16_t y = base - lh * (idx + 1);
    tft.fillRect(0, y, tft.width(), lh, TFT_WHITE);
    tft.setTextFont(2);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.drawString(s, 6, y);
}

static void drawStats() {
    static uint32_t last = 0;
    if (millis() - last < 300) return;
    last = millis();
    char line[80];
    drawRow(0, Console.lastLine());
    snprintf(line, sizeof(line), "CPU %u%%  MEM %uKB",
             audioLoadPct, (unsigned)(ESP.getFreeHeap() / 1024));   drawRow(1, line);
    snprintf(line, sizeof(line), "BAT %.2f V", readBattery());      drawRow(2, line);
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);   // no wait for USB — boot fast (history replays to telnet)
    Serial.println(F("\n# ── Omniphone (T-Display-S3) 19-pad instrument ──"));

    pinMode(PIN_LCD_POWER, OUTPUT); digitalWrite(PIN_LCD_POWER, HIGH);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_WHITE);
    pinMode(PIN_LCD_BL, OUTPUT); digitalWrite(PIN_LCD_BL, HIGH);

    prefs.begin("omni", false);
    pinMode(PIN_SAFEGUARD, INPUT_PULLUP);
    bool forceDiag = (digitalRead(PIN_SAFEGUARD) == LOW);   // held/grounded at boot
    perfMode = forceDiag ? false : prefs.getBool("perf", false);
    if (forceDiag) Serial.println(F("# safeguard pin held → forcing WiFi/diagnostics ON"));

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK);
    sensorsBegin();
    touchBegin();
    audioBegin();

    if (!perfMode) Console.begin();          // performance mode = no WiFi/OTA
    else { WiFi.mode(WIFI_OFF); Serial.println(F("# performance mode (WiFi off)")); }

    drawSelectors();
    Serial.printf("# running: set=%s timbre=%s  perf=%d\n",
                  SOUND_SETS[activeSet].name, TIMBRES[activeTimbre].name, perfMode);
}

void loop() {
    if (!perfMode) {
        Console.handle();
        if (wifiBarShown()) drawNetStatus(tft);
        drawStats();
    }
    if (dirty) { drawSelectors(); dirty = false; }

    static uint32_t lastSense = 0, lastTouch = 0;
    if (millis() - lastSense >= 15) { lastSense = millis(); readSensors(); }
    if (millis() - lastTouch >= 30) { lastTouch = millis(); pollTouch(); }
}
