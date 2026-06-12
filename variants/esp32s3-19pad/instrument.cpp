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
#define SHOW_WIFI_BAR   1 // 0 = hide the WiFi diag bar at the bottom
#define DRIVE_LEDS      1 // 0 = don't drive LEDs (if they disturb sensing)
#define STREAM_SENSORS  1 // 1 = stream per-pad raw delta to Teleplot (d0..d18)

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

// Teleplot streaming is a runtime toggle (the on-screen DIAG button, on the
// CPU/MEM row) and always starts OFF: every UDP burst pulses the radio, which
// couples audibly into the audio path — so it's opt-in while tuning only.
// The button claims a long-axis slice (where the long axis is known-good);
// the short axis only exempts the middle band where the selector text lives,
// because its orientation is unverified (taps are logged to settle it).
static bool streamOn = false;
static constexpr int16_t  BTN_X = 200, BTN_W = 48;   // screen px along long axis
static void drawDiagButton();
static TFT_eSPI tft;

// ── Audio engine ─────────────────────────────────────────────────────────────
static constexpr int      SAMPLE_RATE = 44100;
static constexpr int      BLOCK       = 256;
static constexpr uint16_t TABLE_SIZE  = 1024;
static constexpr int16_t  VOICE_AMP   = 9000;   // per-voice peak
static constexpr int      AMP_SLEW_SHIFT = 9;   // amp one-pole (~12 ms): ramps through
                                                // the coarse proximity steps → no stepping
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
static TaskHandle_t audioTaskHandle = nullptr;

// OTA hooks: suspend the audio task during the flash write — flash operations stall
// code/cache and contend with the upload (audio glitches AND can fail the upload).
static void otaBefore() { if (audioTaskHandle) vTaskSuspend(audioTaskHandle); }
static void otaAfter()  { if (audioTaskHandle) vTaskResume(audioTaskHandle); }

// ── Sound sets (note layouts) and timbres (waveforms) ────────────────────────
// Laid out in the hex grid (rows 3-4-5-4-3); pad 9 = the CENTRE. Each pad is a
// UNIQUE note. MIDI note names (C4 = 60); 's' = sharp (Cs = C#, As = A#/Bb).
//   00 01 02 / 03 04 05 06 / 07 08 09 10 11 / 12 13 14 15 / 16 17 18
// (Arduino's binary.h defines B0/B1 = 0/1 — undef so we can use B-octave names.)
#ifdef B0
#undef B0
#endif
#ifdef B1
#undef B1
#endif
// Note names + sets live INSIDE namespace nt, so bare names (A3, D4…) resolve to
// the enum and don't clash with the core's analog-pin constants (A0–A5).
namespace nt {
enum : uint8_t {
    C1=24,Cs1,D1,Ds1,E1,F1,Fs1,G1,Gs1,A1,As1,B1,
    C2,Cs2,D2,Ds2,E2,F2,Fs2,G2,Gs2,A2,As2,B2,
    C3,Cs3,D3,Ds3,E3,F3,Fs3,G3,Gs3,A3,As3,B3,
    C4,Cs4,D4,Ds4,E4,F4,Fs4,G4,Gs4,A4,As4,B4,
    C5,Cs5,D5,Ds5,E5,F5,Fs5,G5,Gs5,A5,As5,B5,
    C6,Cs6,D6,Ds6,E6,F6,Fs6,G6,Gs6,A6,As6,B6,
};

static const uint8_t SET_ACCORDION[NUM_SENSORS] = {  // Wicki-Hayden isomorphic
        D4, E4, Fs4,
      G3, A3, B3, Cs4,
    C3, D3, E3, Fs3, Gs3,
      F2, G2, A2, B2,
        As1, C2, D2,
};
static const uint8_t SET_CHROMATIC[NUM_SENSORS] = {  // chromatic, ascending
        C3, Cs3, D3,
      Ds3, E3, F3, Fs3,
    G3, Gs3, A3, As3, B3,
      C4, Cs4, D4, Ds4,
        E4, F4, Fs4,
};
static const uint8_t SET_PENTATONIC[NUM_SENSORS] = { // C major pentatonic, wide
        C2, D2, E2,
      G2, A2, C3, D3,
    E3, G3, A3, C4, D4,
      E4, G4, A4, C5,
        D5, E5, G5,
};
// Hangdrum — low "ding" in the CENTRE, pitch rising toward the OUTER ring.
static const uint8_t SET_HANGDRUM[NUM_SENSORS] = {
        A3, As3, C4,
      D4, A2, C3, E4,
    F4, D3, D2, E3, G4,
      A4, F3, G3, As4,
        C5, D5, E5,
};
// Triads — root in the CENTRE, stacked thirds expanding OUTWARD (inside→out), so
// the centre + two outward neighbours spell a chord.
static const uint8_t SET_TRIADS[NUM_SENSORS] = {
        C4, D4, E4,
      F4, E2, G2, G4,
    A4, B2, C2, D3, B4,
      C5, F3, A3, D5,
        E5, G5, B5,
};
// Microtonal: true harmonics (float Hz) of C2 ≈ 65.41; centre = the fundamental.
static const float SET_OVERTONES_HZ[NUM_SENSORS] = {
        65.41f*2,  65.41f*3,  65.41f*4,
      65.41f*5,  65.41f*6,  65.41f*7,  65.41f*8,
    65.41f*9,  65.41f*10, 65.41f*1,  65.41f*11, 65.41f*12,
      65.41f*13, 65.41f*14, 65.41f*15, 65.41f*16,
        65.41f*17, 65.41f*18, 65.41f*19,
};
}  // namespace nt

// A set is either MIDI-note based (midi) or microtonal float-Hz based (hz).
struct SoundSet { const char* name; const uint8_t* midi; const float* hz; };
static const SoundSet SOUND_SETS[] = {
    { "Accordion",  nt::SET_ACCORDION,  nullptr },
    { "Chromatic",  nt::SET_CHROMATIC,  nullptr },
    { "Pentatonic", nt::SET_PENTATONIC, nullptr },
    { "Overtones",  nullptr,            nt::SET_OVERTONES_HZ },
    { "Hangdrum",   nt::SET_HANGDRUM,   nullptr },
    { "Triads",     nt::SET_TRIADS,     nullptr },
};
static constexpr uint8_t NUM_SOUND_SETS = sizeof(SOUND_SETS) / sizeof(SOUND_SETS[0]);

struct Timbre { const char* name; uint8_t wave; };
static const Timbre TIMBRES[] = {
    { "Sine", 0 }, { "Saw", 1 }, { "Square", 2 }, { "Triangle", 3 },
};
static constexpr uint8_t NUM_TIMBRES = sizeof(TIMBRES) / sizeof(TIMBRES[0]);

static volatile uint8_t activeSet    = 0;
static volatile uint8_t activeTimbre = 0;

// ── Proximity tuning / calibration ────────────────────────────────────────────
// The chip's own baseline registers are NOT used: their falling-tracking config
// makes them effectively never come down (any downward signal drift became a
// permanent positive delta = hanging notes / the boot swell), and they're
// quantized to 4 counts. Instead a software baseline tracks the raw 10-bit
// filtered value directly (touch pulls filtered DOWN, so delta = base − filt).
// Touch vs interference can NOT be told apart by amplitude here (a real touch
// reaches delta 150–700, and so do EM bursts) — only by DURATION:
//   • EM bursts are short → median-of-3 kills single-scan spikes, and a voice
//     only opens after CONFIRM_SCANS consecutive scans above the deadband
//     (~20 ms onset gate — bursts shorter than that stay silent).
//   • Touches are seconds → the baseline FREEZES while a pad is held, so held
//     notes sustain at full level. Only after HOLD_MAX_SCANS (~8 s, longer than
//     any musical hold) does it conclude "stuck offset" and re-zero — that is
//     the self-heal path for drifting channels.
//   • Idle (delta < deadband): normal drift tracking absorbs slow EM change.
// For the first SEED_MS the baseline hard-follows (and pads stay muted): the
// MPR121's filtered output is still settling right after init.
static constexpr float PROX_DEADBAND  = 2.0f;   // delta below this = silent
static constexpr float PROX_MAX       = 22.0f;  // delta at full volume
static constexpr float PROX_ATTACK_K  = 0.50f;  // rising smoothing (fast attack)
static constexpr float PROX_RELEASE_K = 0.10f;  // falling smoothing (calm release)
static constexpr float SLOW_BASE_K    = 0.0067f; // idle drift tracking (200 Hz scan)
static constexpr float HEAL_K         = 0.01f;   // re-zero after hold timeout
static constexpr uint16_t HOLD_MAX_SCANS = 1600; // 8 s @ 200 Hz → stuck, heal
// Onset gate: strong deltas (a strike) confirm fast; weak ones (the EM events
// that caused soft phantom blips hover just above the deadband) must persist
// ~60 ms — a slow soft approach by hand is much longer than that anyway.
static constexpr uint8_t  CONFIRM_SCANS      = 3;    // strong onset (~15 ms)
static constexpr uint8_t  CONFIRM_WEAK_SCANS = 12;   // weak onset  (~60 ms)
static constexpr float    STRONG_DELTA       = 8.0f; // strong/weak boundary
static constexpr uint32_t SEED_MS     = 1500;
static float proxSmooth[NUM_SENSORS]  = {0};
static float slowBase[NUM_SENSORS]    = {0};
static float padDelta[NUM_SENSORS]    = {0};   // drift-corrected delta (for Teleplot)
static uint16_t fPrev1[NUM_SENSORS]   = {0};   // median-of-3 history
static uint16_t fPrev2[NUM_SENSORS]   = {0};
static uint16_t holdScans[NUM_SENSORS] = {0};  // consecutive scans above deadband
static uint32_t seedUntil = 0;                 // hard-seed window end (set in setup)

static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    uint16_t lo = min(a, b), hi = max(a, b);
    return max(lo, min(hi, c));
}

// ── Audio task (32-bit stereo) ────────────────────────────────────────────────
static void audioTask(void*) {
    static int32_t tx[BLOCK * 2];
    const uint32_t blockUs = (uint32_t)((uint64_t)BLOCK * 1000000ULL / SAMPLE_RATE);
    int32_t lp = 0;   // one-pole output LPF state
    for (;;) {
        uint32_t t0 = micros();
        const int16_t* wt = WAVE[TIMBRES[activeTimbre].wave];
        for (int f = 0; f < BLOCK; f++) {
            int32_t acc = 0;
            for (uint8_t v = 0; v < NUM_SENSORS; v++) {
                ampF[v] += (targetF[v] - ampF[v]) >> AMP_SLEW_SHIFT;   // Q16 slew
                int32_t a = ampF[v];
                if (a > 0) {
                    // Linear-interpolated table read — the bare lookup's step
                    // between entries is broadband HF grit (same fix as Pico).
                    uint32_t ph  = phase[v];
                    uint16_t i0  = ph >> 22;
                    int32_t  s0  = wt[i0];
                    int32_t  s1  = wt[(i0 + 1) & (TABLE_SIZE - 1)];
                    int32_t  fr  = (ph >> 14) & 0xFF;
                    acc += ((s0 + (((s1 - s0) * fr) >> 8)) * a) >> 16;
                }
                phase[v] += phaseInc[v];
            }
            int32_t out = (int32_t)(tanhf((float)acc * MIX_DRIVE) * MIX_OUT); // soft limit
            lp += (out - lp) >> 1;            // ~5 kHz LPF: keep tone, kill HF hash
            tx[2 * f] = tx[2 * f + 1] = lp << 16;             // → top of 32-bit frame
        }
        uint32_t genUs = micros() - t0;
        audioLoadPct = (uint8_t)(genUs >= blockUs ? 100 : (genUs * 100) / blockUs);
        size_t wrote;
        i2s_write(I2S_NUM_0, tx, sizeof(tx), &wrote, portMAX_DELAY);
    }
}

static inline uint32_t hzToInc(float hz) {
    return (uint32_t)((double)hz * 4294967296.0 / SAMPLE_RATE);
}
static inline uint32_t midiToInc(uint8_t m) {
    return hzToInc(440.0f * powf(2.0f, (m - 69) / 12.0f));
}
static void applySoundSet(uint8_t idx) {
    activeSet = idx;
    const SoundSet& s = SOUND_SETS[idx];
    for (uint8_t v = 0; v < NUM_SENSORS; v++)
        phaseInc[v] = s.hz ? hzToInc(s.hz[v]) : midiToInc(s.midi[v]);
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

    // Priority well above loopTask(1) and anything WiFi adds to core 1, so the
    // synth can only be paused by the DMA wait — never preempted into a glitch.
    xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, 18, &audioTaskHandle, 1);
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
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (!boardOk[b] || SENSE_ELECTRODES[b] == 0) continue;
        boards[b].burstRead(MPR121Reg::FILT_0L, filt[b],
                            (uint8_t)(2 * SENSE_ELECTRODES[b]));
    }
    uint8_t led[NUM_BOARDS][8];
    memset(led, 0, sizeof(led));

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const SenseMap& m = SENSE_PADS[i];
        float fm = 0.0f;
        if (boardOk[m.board] && m.electrode < SENSE_ELECTRODES[m.board]) {
            uint8_t e = m.electrode;
            uint16_t f = (uint16_t)filt[m.board][2 * e]
                       | ((uint16_t)(filt[m.board][2 * e + 1] & 0x03) << 8);
            fm = (float)median3(f, fPrev1[i], fPrev2[i]);   // spike rejection
            fPrev2[i] = fPrev1[i];
            fPrev1[i] = f;
        }
        float c = slowBase[i] - fm;                    // touch pulls filtered DOWN
        bool seeding = millis() < seedUntil;
        float k;
        if (seeding)                k = 1.0f;          // settling: hard-follow
        else if (c < PROX_DEADBAND) k = SLOW_BASE_K;   // idle: track drift
        else if (holdScans[i] > HOLD_MAX_SCANS) k = HEAL_K;  // stuck: re-zero
        else                        k = 0.0f;          // held note: freeze
        slowBase[i] += k * (fm - slowBase[i]);
        if (!seeding && c >= PROX_DEADBAND) {
            if (holdScans[i] < 0xFFFF) holdScans[i]++;
        } else {
            holdScans[i] = 0;
        }
        if (seeding || c < 0.0f) c = 0.0f;             // mute during settling
        padDelta[i] = c;
        float norm = (c - PROX_DEADBAND) / (PROX_MAX - PROX_DEADBAND);
        if (norm < 0.0f) norm = 0.0f; else if (norm > 1.0f) norm = 1.0f;
        if (holdScans[i] < (c >= STRONG_DELTA ? CONFIRM_SCANS : CONFIRM_WEAK_SCANS))
            norm = 0.0f;                               // onset gate: EM bursts
        proxSmooth[i] += (norm - proxSmooth[i]) *
                         (norm > proxSmooth[i] ? PROX_ATTACK_K : PROX_RELEASE_K);
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
    // Only touch the bus when a value actually changed — at 200 Hz the constant
    // writes are needless traffic (and audible coupling into the audio path).
    static uint8_t lastLed[NUM_BOARDS][8];
    static bool ledInit = false;
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (!boardOk[b]) continue;
        if (ledInit && memcmp(led[b], lastLed[b], 8) == 0) continue;
        boards[b].setLEDs8(led[b]);
        memcpy(lastLed[b], led[b], 8);
    }
    ledInit = true;
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
    uint16_t x = ((uint16_t)(d[2] & 0x0F) << 8) | d[3];   // short axis 0..169

    static bool     was = false;
    static uint32_t downMs = 0;
    static uint8_t  downZone = 0;
    static bool     held = false;
    static bool     downBtn = false;
    static uint16_t downX = 0, downY = 0;

    if (touched && !was) {                       // finger down
        downMs = millis(); held = false;
        downX = x; downY = y;
        uint8_t z = y / (320 / 4); if (z > 3) z = 3;
        downZone = TOUCH_FLIP ? (3 - z) : z;
        // DIAG button: its long-axis slice (with margin), any short-axis
        // position except the middle band (selector text area).
        downBtn = (y >= BTN_X - 8 && y <= BTN_X + BTN_W + 8) && (x < 70 || x > 100);
    } else if (touched && !held && millis() - downMs >= HOLD_MS) {
        held = true;                             // 5-s hold reached
        togglePerfMode();                        // live toggle, no reboot
    } else if (!touched && was) {                // finger up
        Console.printf("# tap long=%u short=%u zone=%u btn=%d\n",
                       downY, downX, downZone, (int)downBtn);
        if (!held && millis() - downMs < 800 && downBtn) {
            streamOn = !streamOn;                // DIAG tap → toggle Teleplot
            Console.printf("# diagnostics streaming %s\n", streamOn ? "ON" : "OFF");
            if (!perfMode) drawDiagButton();     // instant visual feedback
        } else if (!held && millis() - downMs < 800) {  // short tap → select
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

// Teleplot on/off button, on the CPU/MEM row right after the text. Redrawn by
// drawStats (whose drawRow(1) wipes that row) and on toggle for instant feedback.
static void drawDiagButton() {
    const int16_t h = 18;
    int16_t y = statsBase() - 2 * h;             // CPU/MEM row (drawStats idx 1)
    uint16_t bg = streamOn ? TFT_DARKGREEN : TFT_LIGHTGREY;
    tft.fillRoundRect(BTN_X, y, BTN_W, h, 4, bg);
    tft.setTextFont(2);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(streamOn ? TFT_WHITE : TFT_DARKGREY, bg);
    tft.drawString("DIAG", BTN_X + BTN_W / 2, y + 1);
    tft.setTextDatum(TL_DATUM);
}

#if STREAM_SENSORS
// Stream per-pad corrected delta to Teleplot (d0..d18). One UDP packet per frame
// to TELEPLOT_HOST (secrets.h); falls back to USB-serial/telnet echo if unset.
static void streamSensors() {
    if (!streamOn) return;              // opt-in via the on-screen DIAG button
    static uint32_t last = 0;
    if (millis() - last < 50) return;   // ~20 Hz
    last = millis();
    Console.teleBegin();
    char nm[8];
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        snprintf(nm, sizeof(nm), "d%u", i);
        Console.tele(nm, padDelta[i], 1);   // software baseline → sub-count resolution
    }
    Console.teleEnd();
}
#endif

static void drawStats() {
    static uint32_t last = 0;
    if (millis() - last < 300) return;
    last = millis();
    char line[80];
    drawRow(0, Console.lastLine());
    snprintf(line, sizeof(line), "CPU %u%%  MEM %uKB",
             audioLoadPct, (unsigned)(ESP.getFreeHeap() / 1024));   drawRow(1, line);
    // Battery: above ~4.30 V means external/USB power; otherwise show a LiPo
    // state-of-charge estimate (sigmoid fit) + the voltage.
    float v = readBattery();
    if (v > 4.30f) {
        snprintf(line, sizeof(line), "USB power  %.2fV", v);
    } else {
        float pct = 123.0f - 123.0f / powf(1.0f + powf(v / 3.7f, 80.0f), 0.165f);
        if (pct < 0) pct = 0; else if (pct > 100) pct = 100;
        snprintf(line, sizeof(line), "BAT %d%%  %.2fV", (int)(pct + 0.5f), v);
    }
    drawRow(2, line);
    drawDiagButton();   // drawRow(0) wiped its row — repaint last
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
    seedUntil = millis() + SEED_MS;          // baseline hard-follows while MPR121 settles
    touchBegin();
    audioBegin();

    Console.onOta(otaBefore, otaAfter);      // mute audio during OTA flash writes
    if (!perfMode) {
        Console.begin();                     // performance mode = no WiFi/OTA
#if STREAM_SENSORS
        if (strlen(TELEPLOT_HOST) > 0) Console.teleplotHost(TELEPLOT_HOST);
#endif
    }
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
#if STREAM_SENSORS
        streamSensors();
#endif
    }
    if (dirty) { drawSelectors(); dirty = false; }

    static uint32_t lastSense = 0, lastTouch = 0;
    if (millis() - lastSense >= 5) { lastSense = millis(); readSensors(); }
    if (millis() - lastTouch >= 30) { lastTouch = millis(); pollTouch(); }
}
