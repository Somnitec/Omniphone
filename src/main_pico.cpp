// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — Raspberry Pi Pico (RP2040) firmware, Mozzi audio backend
//
// 13-pad, no screen. Shares config.h (scales/layout), proximity_engine.h and the
// MPR121 driver with the Teensy build; the synthesis is Mozzi (sound_engine_
// mozzi.h) because the Teensy Audio Library can't run on the RP2040.
//
// Built only for the [env:pico] / [env:esp32s3] PlatformIO environments
// (build_src_filter excludes the Teensy main.cpp there, and excludes this file
// from the Teensy build).
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>

// ── BOARD PIN MAP (RP2040) ────────────────────────────────────────────────────
// This PCB was laid out for a Teensy 4.0. The Pico has a different pinout, so
// you re-point each signal *net* on the board to the Pico GPIO below. Change a
// number here and re-flash — no other file needs editing.
//
//   Signal        Teensy 4.0 pin (old)     →  Pico GPIO (new)   Notes
//   ───────────   ─────────────────────       ───────────────   ─────────────────
//   I2C SDA       18                          GP4               I2C0; any valid SDA
//   I2C SCL       19                          GP5               I2C0; SCL = SDA+1
//   I2S BCLK      21                          GP20              PIO I2S
//   I2S LRCLK/WS  20                          GP21              MUST be BCLK+1
//   I2S DIN(data) 7                           GP22              to PCM5102A DIN
//   I2S MCLK/SCK  23                          (none)            tie PCM5102A SCK→GND
//   LED top pad   (free GPIO net)             GP14              PWM, see SENSORS[]
//   LED upperring (free GPIO net)             GP15              PWM, see SENSORS[]
//
// RP2040 I2S note: the PIO driver requires LRCLK = BCLK + 1 (consecutive GPIOs);
// DATA is independent. PCM5102A runs fine with no master clock — strap its SCK
// pin to GND so it uses the internal PLL.
#define PIN_I2C_SDA   4
#define PIN_I2C_SCL   5

// ── Mozzi configuration — MUST be defined before <Mozzi.h> ────────────────────
// (Macro names follow Mozzi 2.x / MozziConfigValues.h. If you pin Mozzi to a
//  1.x release these become MOZZI_* in MozziGuts.h instead.)
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_CHANNELS MOZZI_MONO         // mono — stereo doubles CPU
#define MOZZI_CONTROL_RATE   256                // sensor/control update Hz
#define MOZZI_AUDIO_RATE     32768              // RP2040 handles 32k comfortably
#define MOZZI_AUDIO_MODE     MOZZI_OUTPUT_I2S_DAC // external PCM5102A over I2S
#if defined(OMNIPHONE_PICO)
  #define MOZZI_I2S_PIN_BCK  20
  #define MOZZI_I2S_PIN_WS   21                 // = BCK + 1 (PIO requirement)
  #define MOZZI_I2S_PIN_DATA 22
#elif defined(OMNIPHONE_ESP32S3)
  // TODO(esp32-s3): set the LilyGO T-FPGA's PCM5102A I2S pins here. The ESP32
  //   I2S peripheral has no BCK=WS+1 constraint; pick any free GPIOs the board
  //   routes to the DAC (and the FPGA, if it sits on the audio path).
  #define MOZZI_I2S_PIN_BCK  5
  #define MOZZI_I2S_PIN_WS   6
  #define MOZZI_I2S_PIN_DATA 7
#endif
#include <Mozzi.h>

#include "config.h"
#include "proximity_engine.h"
#include "sound_engine_mozzi.h"
#include <MPR121.h>

// ─────────────────────────────────────────────────────────────────────────────
// Hardware
// ─────────────────────────────────────────────────────────────────────────────
static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]),
    MPR121(BOARD_ADDRESSES[1]),
};

static SensorState   sensorState[NUM_SENSORS];
static ProximityConfig proxCfg;
static bool          boardOk[NUM_BOARDS] = { false }; // MPR121 connection status

// LFO — per-voice pitch drift for analogue feel
static float lfoPhase[NUM_SENSORS];
static float lfoRate[NUM_SENSORS];

static uint8_t activeSet = DEFAULT_SOUND_SET;

// ── Diagnostics ───────────────────────────────────────────────────────────────
// Periodic serial readout of every pad's proximity intensity (+ touch events
// and idle recals). Set to 0 for silent running. ~10 Hz so it never starves the
// Mozzi audio buffer.
#define PICO_DIAG 1
static constexpr uint32_t DIAG_PERIOD_MS = 100;
static float diagIntensity[NUM_SENSORS]; // last intensity per pad, for the readout

// Audio-path bring-up: set to 1 to ignore the sensors and play a steady 440 Hz
// tone. If you hear it → the I2S/DAC wiring is good and the problem is sensing;
// if not → the problem is the I2S wiring (see the PCM5102A checklist / SCK→GND).
#define PICO_AUDIO_TEST 0

// Onboard LED (GP25 on a standard Pico): blinks while booting, solid while running.
#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

// ─────────────────────────────────────────────────────────────────────────────
// LED helpers
// ─────────────────────────────────────────────────────────────────────────────
// Chip LEDs are batched per board (setLEDs8); GPIO LEDs (LED_GPIO sentinel) are
// driven directly with analogWrite. Collect a frame, then flush.
static uint8_t chipLedBri[NUM_BOARDS][8]; // index = GPIO bit (ledEle-4)

static void ledFrameBegin() { memset(chipLedBri, 0, sizeof(chipLedBri)); }

static void ledSet(const SensorConfig& sc, float intensity) {
    if (sc.ledEle == NO_PIN) return;
    if (sc.ledBoard == LED_GPIO) {
        // Direct RP2040 GPIO PWM (8-bit). pinMode set once in setup().
        analogWrite(sc.ledEle, (uint8_t)(intensity * 255.0f + 0.5f));
    } else {
        chipLedBri[sc.ledBoard][sc.ledEle - 4] =
            (intensity < 0.001f) ? 0 : (uint8_t)(1.0f + intensity * 14.0f + 0.5f);
    }
}

static void ledFrameFlush() {
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
        boards[b].setLEDs8(chipLedBri[b]);
}

static void loadSoundSet(uint8_t index) {
    if (index >= NUM_SOUND_SETS) return;
    activeSet = index;
    mozzisynth::loadSoundSet(SOUND_SETS[index]);
    Serial.print(F("# Sound set: "));
    Serial.println(SOUND_SETS[index].name);
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // ── Onboard LED: blink through boot ──────────────────────────────────────
    pinMode(LED_BUILTIN, OUTPUT);
    for (uint8_t i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(80);
        digitalWrite(LED_BUILTIN, LOW);  delay(80);
    }
    // Give a USB serial monitor a moment to attach so the boot banner is seen
    // (bounded — never hangs if running headless).
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone (Pico/Mozzi) boot ──"));

    // ── I2C bus (re-pointed to the GPIOs above) ──────────────────────────────
#if defined(OMNIPHONE_PICO)
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();
#else // ESP32-S3: SDA/SCL are arguments to begin()
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
#endif
    Wire.setClock(400000);

    // ── MPR121 touch boards ──────────────────────────────────────────────────
    uint8_t boardsFound = 0;
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        boardOk[b] = boards[b].begin(SENSE_ELECTRODES[b], 40, 20, SENSOR_CDC, SENSOR_CDT);
        Serial.print(F("#   MPR121 board "));
        Serial.print(b);
        Serial.print(F(" @ 0x"));
        Serial.print(BOARD_ADDRESSES[b], HEX);
        if (boardOk[b]) {
            Serial.print(F("  OK  ("));
            Serial.print(SENSE_ELECTRODES[b]);
            Serial.println(F(" sense electrodes)"));
            boardsFound++;
        } else {
            Serial.println(F("  *** NOT FOUND — check SDA/SCL wiring, address strap, power"));
        }
        boards[b].beginLEDs();
    }
    Serial.print(F("# touch boards: "));
    Serial.print(boardsFound); Serial.print('/'); Serial.print(NUM_BOARDS);
    Serial.println(boardsFound == NUM_BOARDS ? F(" connected")
                                             : F(" connected  <<< SOME MISSING"));

    // ── GPIO LED pins (LED_GPIO entries in SENSORS[]) ────────────────────────
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (SENSORS[i].ledBoard == LED_GPIO && SENSORS[i].ledEle != NO_PIN)
            pinMode(SENSORS[i].ledEle, OUTPUT);
    }

    // ── Seed per-sensor EMA state ────────────────────────────────────────────
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const SensorConfig& sc = SENSORS[i];
        if (sc.electrode == NO_PIN) { seedSensorState(sensorState[i], 0.0f); continue; }
        uint16_t filt = boards[sc.boardIndex].filteredData(sc.electrode);
        uint16_t base = boards[sc.boardIndex].baselineData(sc.electrode);
        int16_t raw = (int16_t)base - (int16_t)filt;
        seedSensorState(sensorState[i], raw < 0 ? 0.0f : (float)raw);
    }

    // ── LFO phases / rates ────────────────────────────────────────────────────
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        lfoPhase[i] = (float)i / (float)NUM_SENSORS;
        lfoRate[i]  = LFO_RATE_HZ + LFO_RATE_SPREAD * ((float)i / NUM_SENSORS - 0.5f);
    }

    // ── Proximity tuning ──────────────────────────────────────────────────────
    proxCfg.jumpThreshold     = TOUCH_JUMP_THRESHOLD;
    proxCfg.releaseRatio      = TOUCH_RELEASE_RATIO;
    proxCfg.cooldownMs        = TOUCH_COOLDOWN_MS;
    proxCfg.confirmFrames     = TOUCH_CONFIRM_FRAMES;
    proxCfg.proxConfirmFrames = PROX_CONFIRM_FRAMES;
    proxCfg.proxDeadband      = PROX_DEADBAND;
    proxCfg.proxMax           = PROX_MAX;

    // ── Synth ─────────────────────────────────────────────────────────────────
    mozzisynth::init();
    startMozzi(); // control rate comes from MOZZI_CONTROL_RATE
    loadSoundSet(DEFAULT_SOUND_SET);

    // ── Onboard LED: solid ON = running ──────────────────────────────────────
    digitalWrite(LED_BUILTIN, HIGH);

    // ── I2S audio out (PCM5102A) ─────────────────────────────────────────────
    // The DAC is write-only over I2S — there is no ACK/readback, so it cannot be
    // probed like the MPR121s. We echo the pins instead: if there's no sound,
    // verify these against your wiring (and that PCM5102A SCK is tied to GND).
    Serial.print(F("# I2S out (PCM5102A, no ACK to verify): BCK="));
    Serial.print(MOZZI_I2S_PIN_BCK);
    Serial.print(F(" WS="));   Serial.print(MOZZI_I2S_PIN_WS);
    Serial.print(F(" DATA=")); Serial.print(MOZZI_I2S_PIN_DATA);
    Serial.println(F("  (audio engine started)"));

    Serial.println(F("# Omniphone (Pico/Mozzi) started"));
    Serial.print(F("# pads="));   Serial.print(NUM_SENSORS);
    Serial.print(F(" audioRate=")); Serial.print(MOZZI_AUDIO_RATE);
    Serial.print(F(" controlRate=")); Serial.println(MOZZI_CONTROL_RATE);
    Serial.println(F("# send '0'..'6' to switch sound sets"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Mozzi control-rate callback (MOZZI_CONTROL_RATE Hz)
// ─────────────────────────────────────────────────────────────────────────────
void updateControl() {
    // Serial: switch sound sets 0–6
    if (Serial.available()) {
        char c = Serial.read();
        if (c >= '0' && c <= '6') loadSoundSet(c - '0');
    }

    uint32_t now = millis();

#if PICO_AUDIO_TEST
    // Steady 440 Hz on voice 0, sensors ignored — isolates the audio path.
    for (uint8_t i = 0; i < NUM_SENSORS; i++) mozzisynth::setVoiceIntensity(i, 0.0f);
    mozzisynth::setVoiceFreq(0, 440.0f);
    mozzisynth::setVoiceIntensity(0, 0.9f);
    mozzisynth::updateControl();
    return;
#endif

    // ── Burst-read both boards ────────────────────────────────────────────────
    // Read ALL 12 electrodes (ELE0–ELE11): filtered = 24 bytes (2 per electrode),
    // baseline = 12 bytes (1 per electrode). Sizing this for only 6 electrodes is
    // what made any pad on ELE6+ read out-of-bounds garbage.
    struct BoardData { uint8_t filt[24]; uint8_t base[12]; uint16_t touch; };
    static BoardData bd[NUM_BOARDS];
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        boards[b].burstRead(MPR121Reg::FILT_0L, bd[b].filt, 24);
        boards[b].burstRead(MPR121Reg::BASE_0,  bd[b].base, 12);
        bd[b].touch = boards[b].touchStatus();
    }

    ledFrameBegin();

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const SensorConfig& sc = SENSORS[i];
        if (sc.electrode == NO_PIN) { mozzisynth::setVoiceIntensity(i, 0.0f); continue; }

        const BoardData& b = bd[sc.boardIndex];
        uint8_t e = sc.electrode;
        if (e > 11) { mozzisynth::setVoiceIntensity(i, 0.0f); continue; } // ELE0–ELE11 only
        uint16_t filtered = (uint16_t)b.filt[2 * e]
                          | ((uint16_t)(b.filt[2 * e + 1] & 0x03) << 8);
        uint16_t baseline = (uint16_t)b.base[e] << 2;
        float rawDelta = (float)((int16_t)baseline - (int16_t)filtered);
        if (rawDelta < 0.0f) rawDelta = 0.0f;

        float intensity; bool isTouch;
        updateProximity(rawDelta, now, proxCfg, sensorState[i], intensity, isTouch);
        // TODO(bell): isTouch marks a metal-contact strike — wire a Mozzi bell
        //             voice + USB-MIDI/MPE here once the core synth is dialed in.

#if PICO_DIAG
        diagIntensity[i] = intensity;
        if (isTouch) {
            Serial.print(F("# touch pad ")); Serial.print(i);
            Serial.print(F(" vel "));        Serial.println(sensorState[i].velocity, 1);
        }
#endif

        mozzisynth::setVoiceIntensity(i, intensity);

        // LFO pitch drift
        lfoPhase[i] += lfoRate[i] * (1.0f / (float)MOZZI_CONTROL_RATE);
        if (lfoPhase[i] >= 1.0f) lfoPhase[i] -= 1.0f;
        float freq = SOUND_SETS[activeSet].freqs[i]
                   * (1.0f + LFO_AMOUNT * sinf(2.0f * (float)M_PI * lfoPhase[i]));
        mozzisynth::setVoiceFreq(i, freq);

        ledSet(sc, intensity);
    }

    ledFrameFlush();
    mozzisynth::updateControl();

#if PICO_DIAG
    // ── Throttled per-pad intensity readout (~10 Hz) ─────────────────────────
    static uint32_t lastDiag = 0;
    if (now - lastDiag >= DIAG_PERIOD_MS) {
        lastDiag = now;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            Serial.print(i);
            Serial.print(':');
            Serial.print(diagIntensity[i], 2);
            Serial.print(' ');
        }
        Serial.print(F("| "));
        Serial.println(SOUND_SETS[activeSet].name);
    }
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// Mozzi audio-rate callback (MOZZI_AUDIO_RATE Hz)
// ─────────────────────────────────────────────────────────────────────────────
AudioOutput updateAudio() {
    // 13-bit headroom: a full 13-voice chord peaks near this, while a single pad
    // is still clearly audible. fromAlmostNBit clips safely on the loudest chords.
    return MonoOutput::fromAlmostNBit(13, mozzisynth::updateAudio());
}

void loop() {
    audioHook();
}
