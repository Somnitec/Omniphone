#include <Arduino.h>
#include <Wire.h>
#include <Audio.h>
#include <math.h>

#include "LCD_Driver.h"
#include "Touch_Driver.h"
#include "GUI_Paint.h"

#include "config.h"
#include "proximity_engine.h"
#include <MPR121.h>

// ─────────────────────────────────────────────────────────────────────────────
// Audio graph — Teensy Audio Library
//
// 12 independent sine-wave voices (one per sensor pad), combined through a
// three-stage mixer tree to a stereo I²S output (PCM5102A DAC).
//
// Mixer gain structure (prevents clipping when many pads are played at once):
//   Each voice amplitude: 0.0 – 0.45
//   Stage-1 mixers (4 voices each): gain 0.25 → max out = 4 × 0.45 × 0.25 = 0.45
//   Master mixer (3 inputs):        gain 0.8  → max out ≈ 0.45 × 0.8 = 0.36
// ─────────────────────────────────────────────────────────────────────────────

AudioSynthWaveformSine voice0, voice1, voice2,  voice3,
                       voice4, voice5, voice6,  voice7,
                       voice8, voice9, voice10, voice11;
AudioMixer4            mix1, mix2, mix3; // stage 1: 4 voices each
AudioMixer4            masterMix;
AudioOutputI2S         i2sOut;

// Stage 1 connections
AudioConnection pv0(voice0,  0, mix1, 0);
AudioConnection pv1(voice1,  0, mix1, 1);
AudioConnection pv2(voice2,  0, mix1, 2);
AudioConnection pv3(voice3,  0, mix1, 3);
AudioConnection pv4(voice4,  0, mix2, 0);
AudioConnection pv5(voice5,  0, mix2, 1);
AudioConnection pv6(voice6,  0, mix2, 2);
AudioConnection pv7(voice7,  0, mix2, 3);
AudioConnection pv8(voice8,  0, mix3, 0);
AudioConnection pv9(voice9,  0, mix3, 1);
AudioConnection pv10(voice10, 0, mix3, 2);
AudioConnection pv11(voice11, 0, mix3, 3);
// Master bus
AudioConnection pm1(mix1, 0, masterMix, 0);
AudioConnection pm2(mix2, 0, masterMix, 1);
AudioConnection pm3(mix3, 0, masterMix, 2);
// Stereo output (same signal on both channels)
AudioConnection pmL(masterMix, 0, i2sOut, 0);
AudioConnection pmR(masterMix, 0, i2sOut, 1);

// Indexed pointer array for clean per-voice access in the update loop.
AudioSynthWaveformSine* const VOICES[12] = {
    &voice0, &voice1, &voice2,  &voice3,
    &voice4, &voice5, &voice6,  &voice7,
    &voice8, &voice9, &voice10, &voice11
};

// ─────────────────────────────────────────────────────────────────────────────
// Hardware
// ─────────────────────────────────────────────────────────────────────────────

// One MPR121 instance per board defined in config.h.
// The array is initialised in setup() because MPR121() takes a reference.
static MPR121 boards[2] = {
    MPR121(BOARD_ADDRESSES[0]),
    MPR121(BOARD_ADDRESSES[1]),
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-sensor state
// ─────────────────────────────────────────────────────────────────────────────

static SensorState   sensorState[NUM_SENSORS];
static ProximityConfig proxCfg; // uses struct default values from proximity_engine.h

// ─────────────────────────────────────────────────────────────────────────────
// LFO — per-voice pitch drift
//
// Each voice has an independent LFO phase so adjacent notes drift out of sync,
// giving the instrument a slightly unstable, analogue feel.
//
// LFO_RATE_HZ  : how many drift cycles per second (0.1–1 Hz sounds natural)
// LFO_AMOUNT   : peak frequency deviation as a fraction of the base pitch
//                (0.003 = ±0.3% → roughly ±5 cents at A4, subtle but audible)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr float LFO_RATE_HZ = 0.25f;  // slow drift cycle
static constexpr float LFO_AMOUNT  = 0.003f; // ±0.3% pitch drift

static float lfoPhase[NUM_SENSORS]; // phase accumulator per voice, 0.0 – 1.0

// ── Amplitude smoothing ───────────────────────────────────────────────────────
// AudioSynthWaveformSine::amplitude() takes effect at the next audio buffer
// boundary (~2.9 ms). A sudden jump from 0 → full creates a waveform
// discontinuity at that boundary → audible click.
// Smoothing the target with an EMA limits the per-frame step to ≈9% of full
// scale (α=0.2 at 125 Hz), which is below the audible click threshold.
// Time to fade from silence to full: ~1/(0.2×125) ≈ 40 ms — fast enough to
// feel immediate, smooth enough to be click-free.
static constexpr float AMP_SMOOTH_ALPHA = 0.4f;
static float smoothAmp[NUM_SENSORS]; // current smoothed amplitude per voice

// ─────────────────────────────────────────────────────────────────────────────
// Update timing
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint32_t UPDATE_MS  = 8; // ~125 Hz sensor update rate
static uint32_t lastUpdateMs         = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Instrument callbacks
// ─────────────────────────────────────────────────────────────────────────────

// Called every frame for each pad.
// intensity 0.0 = hand far away, 1.0 = hand very close.
// The actual amplitude write is done in the main loop after EMA smoothing
// to avoid waveform clicks — add other continuous controls here.
static void onProximity(uint8_t sensorIndex, float intensity) {
    // Target amplitude stored for smoothing in the loop.
    smoothAmp[sensorIndex] = smoothAmp[sensorIndex]
        + AMP_SMOOTH_ALPHA * (intensity * 0.45f - smoothAmp[sensorIndex]);
    VOICES[sensorIndex]->amplitude(smoothAmp[sensorIndex]);
}

// Called once on the frame a contact event is detected.
// velocity is the smoothed approach speed (arbitrary units; larger = faster touch).
// Reserved for future use: percussive attack envelopes, triggering samples, etc.
static void onTouch(uint8_t sensorIndex, float velocity) {
    (void)sensorIndex;
    (void)velocity;
}

// ─────────────────────────────────────────────────────────────────────────────
// Startup LED animation
//
// Sequential per-board fade followed by a simultaneous all-LEDs sweep.
// Runs after MPR121 is initialised, using the new MPR121::setAllLEDs API.
// ─────────────────────────────────────────────────────────────────────────────

static void playStartupAnimation() {
    uint8_t bri[6];
    // All LEDs on all boards fade up then back down together (~300 ms total).
    for (uint8_t v = 0; v <= 15; v++) {
        memset(bri, v, sizeof(bri));
        for (uint8_t b = 0; b < NUM_BOARDS; b++) boards[b].setAllLEDs(bri);
        delay(10);
    }
    for (uint8_t v = 15; v > 0; v--) {
        memset(bri, v, sizeof(bri));
        for (uint8_t b = 0; b < NUM_BOARDS; b++) boards[b].setAllLEDs(bri);
        delay(10);
    }
    memset(bri, 0, sizeof(bri));
    for (uint8_t b = 0; b < NUM_BOARDS; b++) boards[b].setAllLEDs(bri);
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    // ── I²C bus ───────────────────────────────────────────────────────────────
    Wire.begin();
    Wire.setClock(400000); // 400 kHz — ensure 4.7 kΩ pull-ups on SDA/SCL

    // ── MPR121 boards ─────────────────────────────────────────────────────────
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (!boards[b].begin()) {
            Serial.print(F("ERROR: MPR121 board "));
            Serial.print(b);
            Serial.print(F(" at 0x"));
            Serial.print(BOARD_ADDRESSES[b], HEX);
            Serial.println(F(" not found — check address and wiring"));
        } else {
            Serial.print(F("MPR121 board "));
            Serial.print(b);
            Serial.println(F(" OK"));
        }
        boards[b].beginLEDs();
    }

    // ── Seed per-sensor EMA state ─────────────────────────────────────────────
    // Read the actual initial delta for each pad so the jump detector
    // has a realistic baseline and does not fire spuriously on the first frame.
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const SensorConfig& sc = SENSORS[i];
        uint16_t filt = boards[sc.boardIndex].filteredData(sc.electrode);
        uint16_t base = boards[sc.boardIndex].baselineData(sc.electrode);
        int16_t  raw  = (int16_t)base - (int16_t)filt;
        seedSensorState(sensorState[i], raw < 0 ? 0.0f : (float)raw);
    }

    // ── LFO phases ────────────────────────────────────────────────────────────
    // Spread initial phases evenly so voices drift independently from startup.
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        lfoPhase[i] = (float)i / (float)NUM_SENSORS;
    }

    // ── Teensy Audio ──────────────────────────────────────────────────────────
    AudioMemory(64);

    // Stage-1 mixer gain: 4 voices × 0.25 → peak = 1.0 per sub-bus
    for (int ch = 0; ch < 4; ch++) {
        mix1.gain(ch, 0.25f);
        mix2.gain(ch, 0.25f);
        mix3.gain(ch, 0.25f);
    }
    // Master mixer: headroom for 3 sub-buses (0.8 leaves a safety margin)
    for (int ch = 0; ch < 3; ch++) {
        masterMix.gain(ch, 0.8f);
    }

    // Assign base frequencies; all voices start silent.
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        VOICES[i]->frequency(SENSORS[i].noteFreq);
        VOICES[i]->amplitude(0.0f);
    }

    // ── LCD / touch screen ────────────────────────────────────────────────────
    Touch_1IN28_XY XY;
    XY.mode = 1; // point mode (not gesture mode)

    Config_Init();
    LCD_Init();
    LCD_SetBacklight(200);

    if (Touch_1IN28_init(XY.mode)) {
        Serial.println(F("Touchscreen OK"));
    } else {
        Serial.println(F("Touchscreen not found"));
    }

    Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, BLACK);
    Paint_Clear(BLACK);
    Paint_DrawString_EN(35, 90, "OMNIPHONE", &Font20, BLACK, WHITE);
    Paint_DrawString_EN(55, 115, "ready", &Font16, BLACK, WHITE);

    // ── Startup animation (runs after all init is complete) ───────────────────
    playStartupAnimation();

    Serial.println(F("# Omniphone started"));
    Serial.println(F("# sensor | intensity | freq_hz | touch"));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    uint32_t now = millis();
    if (now - lastUpdateMs < UPDATE_MS) return;
    lastUpdateMs = now;

    // ── Burst-read all boards ─────────────────────────────────────────────────
    // Reading filtered, baseline and touch registers in three back-to-back burst
    // reads per board minimises I²C time and keeps all electrode reads coherent
    // within the same scan cycle.
    struct BoardData {
        uint8_t  filt[12]; // ELE0–ELE5 filtered: 2 bytes each (10-bit little-endian)
        uint8_t  base[6];  // ELE0–ELE5 baseline: 1 byte each (8 MSBs)
        uint16_t touch;    // bitmask: bit i set if electrode i is touched
    };
    static BoardData bd[NUM_BOARDS];

    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        boards[b].burstRead(MPR121Reg::FILT_0L, bd[b].filt, 12);
        boards[b].burstRead(MPR121Reg::BASE_0,  bd[b].base, 6);
        bd[b].touch = boards[b].touchStatus();
    }

    // ── Per-sensor update ─────────────────────────────────────────────────────
    // LED brightness values are accumulated here; a single setAllLEDs() call
    // per board is issued after the loop. This cuts LED I²C transactions from
    // 3 per sensor (read-modify-write) down to ~5 per board regardless of how
    // many sensors are active, which is the main cause of audio crackling.
    static uint8_t ledBri[NUM_BOARDS][6];
    memset(ledBri, 0, sizeof(ledBri));

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const SensorConfig& sc = SENSORS[i];
        const BoardData&    b  = bd[sc.boardIndex];
        uint8_t             e  = sc.electrode;

        // Reconstruct 10-bit values from the burst buffers.
        uint16_t filtered  = (uint16_t)b.filt[2 * e]
                             | ((uint16_t)(b.filt[2 * e + 1] & 0x03) << 8);
        uint16_t baseline  = (uint16_t)b.base[e] << 2;

        float rawDelta = (float)((int16_t)baseline - (int16_t)filtered);
        if (rawDelta < 0.0f) rawDelta = 0.0f;

        float intensity;
        bool  isTouch;
        updateProximity(rawDelta, now, proxCfg, sensorState[i], intensity, isTouch);

        if (isTouch) onTouch(i, sensorState[i].velocity);
        onProximity(i, intensity);

        // ── LFO pitch drift ───────────────────────────────────────────────────
        // Phase advances at LFO_RATE_HZ per second, wrapping at 1.0.
        // The resulting ±LFO_AMOUNT frequency deviation makes each voice
        // sound slightly unstable, like an analogue oscillator.
        lfoPhase[i] += LFO_RATE_HZ * (UPDATE_MS / 1000.0f);
        if (lfoPhase[i] >= 1.0f) lfoPhase[i] -= 1.0f;

        float freq = sc.noteFreq * (1.0f + LFO_AMOUNT * sinf(2.0f * (float)M_PI * lfoPhase[i]));
        VOICES[i]->frequency(freq);

        // Accumulate LED brightness (no I²C yet).
        // Map intensity 0–1 → PWM 0–15; value 1 is the minimum non-zero
        // brightness to avoid the invisible bottom of the PWM range.
        ledBri[sc.boardIndex][e] = (intensity < 0.001f) ? 0
                         : static_cast<uint8_t>(1.0f + intensity * 14.0f + 0.5f);
    }

    // ── Batch LED update — one setAllLEDs() per board ─────────────────────────
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        boards[b].setAllLEDs(ledBri[b]);
    }

    // ── Diagnostic output (20 Hz) ─────────────────────────────────────────────
    static uint32_t lastDiagMs = 0;
    if (now - lastDiagMs >= 50) {
        lastDiagMs = now;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            // Re-derive intensity from current EMA state for display.
            float norm = (sensorState[i].fast - proxCfg.proxDeadband)
                         / (proxCfg.proxMax - proxCfg.proxDeadband);
            float intensity = norm < 0.0f ? 0.0f : (norm > 1.0f ? 1.0f : norm);

            Serial.print(i);
            Serial.print(F(" "));
            Serial.print(intensity, 3);
            Serial.print(F(" "));
            Serial.print(SENSORS[i].noteFreq, 1);
            Serial.print(F("  "));
        }
        Serial.println();
    }
}
