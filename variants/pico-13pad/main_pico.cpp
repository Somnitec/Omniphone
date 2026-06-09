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
#define MOZZI_AUDIO_CHANNELS MOZZI_STEREO       // mono synth duplicated to L+R (see updateAudio)
#define MOZZI_CONTROL_RATE   128                // sensor/control update Hz
#define MOZZI_AUDIO_RATE     32768              // RP2040 handles 32k comfortably
#define MOZZI_AUDIO_MODE     MOZZI_OUTPUT_I2S_DAC // external PCM5102A over I2S
// Bigger RP2040 I2S DMA buffer (~512 samples ≈ 15 ms) so a brief I2C sensor read
// can't drain it → no buzz/rattle even reading every control tick. (Set directly
// to bypass Mozzi's generic 256-sample cap on MOZZI_OUTPUT_BUFFER_SIZE.)
#define MOZZI_RP2040_BUFFER_SIZE 512
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
// Fixed output low-pass (2-pole, 12 dB/oct) to roll off hiss — bass needs no
// treble. Cutoff ≈ audioRate/(2π·2^N): 1≈2.6 kHz, 2≈1.3 kHz, 3≈650 Hz, 4≈325 Hz.
// Set to 0 to bypass (diagnostic: if the noise survives this, it's low-frequency
// AM from the sensors, not high-frequency synthesis grit).
#define OMNI_OUTPUT_LPF_SHIFT 1
#include "sound_engine_mozzi.h"
#include <MPR121.h>
#include <EEPROM.h>   // flash-backed emulation (RP2040 last sector / ESP32 NVS)

// ── Persistent selection (survives power-off) ─────────────────────────────────
// Stored in emulated EEPROM. A magic byte tags a valid record; bump it to
// invalidate old layouts (e.g. if NUM_SCALES/NUM_TIMBRES change meaning).
static constexpr int     EE_SIZE  = 16;
static constexpr int     EE_ADDR  = 0;
static constexpr uint8_t EE_MAGIC = 0xA7;
struct StoredSel { uint8_t magic; uint8_t scale; uint8_t timbre; };

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

static uint8_t activeScale  = DEFAULT_SCALE;
static uint8_t activeTimbre = DEFAULT_TIMBRE;

// ── Diagnostics ───────────────────────────────────────────────────────────────
// Periodic serial readout of every pad's proximity intensity (+ touch events
// and idle recals). Set to 0 for silent running. ~10 Hz so it never starves the
// Mozzi audio buffer.
#define PICO_DIAG 0  // TEST: serial off — if the ~5 Hz rattle stops, the diagnostic print was the cause.
#define DIAG_ONELINE 0  // 0 = one value per line (Teleplot). 1 = all on one tab-separated line (plain monitor, NOT Teleplot-parseable).
static constexpr uint32_t DIAG_PERIOD_MS = 100;
// Sensor I2C read period. Reading every control tick (128 Hz) starves Mozzi's
// audio buffer (→ buzz); ~30 Hz lets the buffer recover between the ~2 ms reads.
// Lower = snappier touch but more audio risk; raise if the audio still buzzes.
static constexpr uint32_t SENSE_PERIOD_MS = 0;   // 0 = read every control tick (responsive); the big DMA buffer now absorbs the I2C blocks
static float diagIntensity[NUM_SENSORS]; // last intensity per pad, for the readout
static float proxSmooth[NUM_SENSORS];    // de-jittered amplitude (symmetric one-pole)

// Amplitude de-jitter: the near-field MPR121 delta jitters ~±1 LSB ≈ ±6% of
// full gain; sent straight to amplitude that is audible AM ("white noise" that
// grows with proximity). A *symmetric* one-pole at control rate (128 Hz) makes a
// steady hand give a steady level. Larger = smoother but slower attack:
//   0.20 → ~4 Hz corner / ~40 ms   (very smooth, soft attack)
//   0.30 → ~6 Hz corner / ~25 ms   (good default)
//   0.45 → ~10 Hz corner / ~16 ms  (snappier, lets a little jitter back in)
static constexpr float PROX_SMOOTH_K = 0.30f;

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

// Drive EVERY LED (chip-driven + GPIO) to one level. Used as the boot/calibration
// indicator: solid while calibrating, faded off once the instrument is running.
// level is the 0–15 chip-LED scale; GPIO LEDs are scaled to 0–255.
static void allLeds(uint8_t level) {
    uint8_t bri[8];
    memset(bri, level, sizeof(bri));
    for (uint8_t b = 0; b < NUM_BOARDS; b++) boards[b].setLEDs8(bri);
    uint8_t pwm = (uint8_t)((uint16_t)level * 255u / 15u);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
        if (SENSORS[i].ledBoard == LED_GPIO && SENSORS[i].ledEle != NO_PIN)
            analogWrite(SENSORS[i].ledEle, pwm);
}

static void loadScale(uint8_t index) {
    if (index >= NUM_SCALES) return;
    activeScale = index;
    mozzisynth::loadScale(SCALES[index]);
    Serial.print(F("# Scale: "));
    Serial.println(SCALES[index].name);
}

static void loadTimbre(uint8_t index) {
    if (index >= NUM_TIMBRES) return;
    activeTimbre = index;
    mozzisynth::loadTimbre(TIMBRES[index]);
    Serial.print(F("# Timbre: "));
    Serial.println(TIMBRES[index].name);
}

// Persist the current scale+timbre to EEPROM so it survives a power-cycle.
static void saveSelection() {
    StoredSel s{ EE_MAGIC, activeScale, activeTimbre };
    EEPROM.put(EE_ADDR, s);
    EEPROM.commit();
    Serial.println(F("# selection saved"));
}

// Apply a pad number exactly as if it were held at boot: even pad → scale,
// odd pad → timbre. Then persist. Used by the serial console.
static void selectByPad(uint8_t pad) {
    uint8_t scI = scaleForPad(pad);
    uint8_t tbI = timbreForPad(pad);
    if      (scI != 0xFF) loadScale(scI);
    else if (tbI != 0xFF) loadTimbre(tbI);
    else { Serial.print(F("# pad ")); Serial.print(pad); Serial.println(F(" selects nothing")); return; }
    saveSelection();
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

    // ── MPR121 staged init ────────────────────────────────────────────────────
    // Configure + start scanning with the baseline FROZEN (CL=00) so a finger
    // held during boot shows as a sharp DROP in filteredData (used for the
    // scale/timbre selection below) instead of being absorbed into the baseline.
    uint8_t boardsFound = 0;
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        boardOk[b] = boards[b].beginConfig(SENSE_ELECTRODES[b], 40, 20, SENSOR_CDC, SENSOR_CDT);
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
        boards[b].startScanning(SENSE_ELECTRODES[b], 0b00); // CL=00 → baseline frozen
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

    // ── Calibration / selection window ───────────────────────────────────────
    // All LEDs light up = "calibrating — hold a pad to choose". Hold an EVEN pad
    // to pick the scale, an ODD pad to pick the timbre; pad 0 (top) is the
    // untouched reference. LEDs fade off once calibration locks → running.
    allLeds(15);
    delay(250); // let frozen-baseline filtered data populate

    // Last saved selection becomes the default; a held pad overrides it below.
    EEPROM.begin(EE_SIZE);
    StoredSel stored; EEPROM.get(EE_ADDR, stored);
    bool stUsable = (stored.magic == EE_MAGIC);
    activeScale  = (stUsable && stored.scale  < NUM_SCALES)  ? stored.scale  : DEFAULT_SCALE;
    activeTimbre = (stUsable && stored.timbre < NUM_TIMBRES) ? stored.timbre : DEFAULT_TIMBRE;
    {
        const SensorConfig& refS = SENSORS[STARTUP_REF_PAD];
        uint16_t refF = boards[refS.boardIndex].filteredData(refS.electrode);
        int8_t heldScalePad = -1, heldTimbrePad = -1;
        for (uint8_t p = 0; p < NUM_SENSORS; p++) {
            if (p == STARTUP_REF_PAD) continue;
            const SensorConfig& s = SENSORS[p];
            if (s.electrode == NO_PIN || s.electrode > 11) continue;
            uint16_t padF = boards[s.boardIndex].filteredData(s.electrode);
            if ((int16_t)refF - (int16_t)padF <= STARTUP_HOLD_THRESHOLD) continue;
            uint8_t scI = scaleForPad(p);
            uint8_t tbI = timbreForPad(p);
            if (scI != 0xFF && heldScalePad  < 0) { activeScale  = scI; heldScalePad  = (int8_t)p; }
            if (tbI != 0xFF && heldTimbrePad < 0) { activeTimbre = tbI; heldTimbrePad = (int8_t)p; }
        }
        if (heldScalePad >= 0) {
            Serial.print(F("# held pad ")); Serial.print(heldScalePad);
            Serial.print(F(" → scale "));   Serial.println(SCALES[activeScale].name);
        }
        if (heldTimbrePad >= 0) {
            Serial.print(F("# held pad ")); Serial.print(heldTimbrePad);
            Serial.print(F(" → timbre "));  Serial.println(TIMBRES[activeTimbre].name);
        }
    }

    // Persist if a hold changed the selection (or the store was empty/stale).
    if (!stUsable || stored.scale != activeScale || stored.timbre != activeTimbre)
        saveSelection();

    // Keep the LEDs solid a moment (visible confirmation), then lock the baseline
    // and fade the LEDs out once the finger is lifting.
    delay(700);
    for (uint8_t b = 0; b < NUM_BOARDS; b++) boards[b].lockBaseline(SENSE_ELECTRODES[b]);
    for (int v = 15; v >= 0; v--) { allLeds((uint8_t)v); delay(20); }
    allLeds(0);

    // ── Seed per-sensor EMA state from the freshly locked baseline ───────────
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
    loadScale(activeScale);
    loadTimbre(activeTimbre);
    startMozzi(); // control rate comes from MOZZI_CONTROL_RATE

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
    Serial.print(F("# scale=")); Serial.print(SCALES[activeScale].name);
    Serial.print(F(" timbre=")); Serial.println(TIMBRES[activeTimbre].name);
    Serial.println(F("# boot: hold EVEN pad=scale, ODD pad=timbre (saved across resets)."
                     " Serial: send pad # 0-12 + Enter (same mapping)"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Mozzi control-rate callback (MOZZI_CONTROL_RATE Hz)
// ─────────────────────────────────────────────────────────────────────────────
void updateControl() {
    // Serial (testing): send a PAD NUMBER (0–12) + Enter — same mapping as the
    // boot hold (even pad → scale, odd pad → timbre). Persisted like a hold.
    static int serialNum = -1; // -1 = no digits buffered yet
    while (Serial.available()) {
        char c = Serial.read();
        if (c >= '0' && c <= '9') {
            serialNum = (serialNum < 0 ? 0 : serialNum * 10) + (c - '0');
        } else if (serialNum >= 0) {            // terminator (newline/space/etc.)
            if (serialNum < NUM_SENSORS) selectByPad((uint8_t)serialNum);
            serialNum = -1;
        }
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

    // ── Throttled sensor read (decoupled from the control rate) ──────────────
    // The I2C burst-reads block ~2 ms; doing them every control tick starves the
    // Mozzi audio buffer → buzz. At SENSE_PERIOD_MS the buffer recovers between
    // reads. Voice/LED values simply hold between reads.
    static uint32_t lastSense = 0;
    if (now - lastSense >= SENSE_PERIOD_MS) {
    lastSense = now;

    // Burst-read both boards: ALL 12 electrodes (ELE0–ELE11) — filtered = 24 bytes
    // (2 per electrode), baseline = 12 bytes (1 per electrode).
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

        // Symmetric low-pass kills sensor-jitter AM (the "white noise" that grew
        // with proximity) while keeping a few-ms attack. See PROX_SMOOTH_K.
        proxSmooth[i] += (intensity - proxSmooth[i]) * PROX_SMOOTH_K;
        mozzisynth::setVoiceIntensity(i, proxSmooth[i]);

        // Pitch drift (LFO) removed for this version — frequency stays at the
        // scale base set by loadScale(), so steady notes are dead steady.

        ledSet(sc, proxSmooth[i]);
    }

    ledFrameFlush();
    }  // end throttled sensor read

    mozzisynth::updateControl();

#if PICO_DIAG
    // ── Teleplot per-pad intensity (~10 Hz) ──────────────────────────────────
    // Teleplot reads lines of the form ">name:value". Each pad becomes a series
    // p0..p12. In VS Code: open the Teleplot panel, connect to this serial port
    // (115200), and close any other serial monitor first (it owns the port).
    static uint32_t lastDiag = 0;
    if (now - lastDiag >= DIAG_PERIOD_MS) {
        lastDiag = now;
        // Teleplot needs each ">name:value" on its OWN line. DIAG_ONELINE=1 packs
        // them tab-separated onto one line (readable in a plain monitor) but
        // Teleplot can't parse that — use 0 (default) when plotting in Teleplot.
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            Serial.print(F(">p"));
            Serial.print(i);
            Serial.print(':');
            Serial.print(diagIntensity[i], 3);
#if DIAG_ONELINE
            Serial.print('\t');
        }
        Serial.println();
#else
            Serial.println();
        }
#endif
    }
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// Mozzi audio-rate callback (MOZZI_AUDIO_RATE Hz)
// ─────────────────────────────────────────────────────────────────────────────
AudioOutput updateAudio() {
    // 13-bit headroom: a full 13-voice chord peaks near this, while a single pad
    // is still clearly audible. fromAlmostNBit clips safely on the loudest chords.
    // Mono synth duplicated to both I2S channels (confirmed: stereo fills L+R).
    int32_t out = mozzisynth::updateAudio();
    return StereoOutput::fromAlmostNBit(19, out, out);   // 19 keeps today's loudness with full-precision voices (~8-voice headroom)
}

void loop() {
    audioHook();
}
