// Teensy 3.2 — 13-pad touch synth (standalone test build).
//
// Thirteen sine voices laid out as a C-major diatonic from C3 to A4, sensed
// by 2× MPR121 over I²C. LEDs use as many MPR121 PWM channels as the
// touch layout leaves free, with the remaining 4 on Teensy native PWM pins.
//
// Built to be flashed independently of the main firmware — see the
// `[env:teensy32_thirteen]` block in platformio.ini.
//
// ── PCM5102A wiring per Teensy target ────────────────────────────────────────
// AudioOutputI2S picks the pins automatically per build target; the table
// below is just for soldering reference.
//
//                 Teensy 4.0     Teensy 3.2 / LC
//   BCK / BCLK    pin 21         pin 9
//   LCK / LRCLK   pin 20         pin 23
//   DIN / TX      pin 7          pin 22
//   (MCLK)        pin 23         pin 11   (only if your DAC needs it)
//
// ── Teensy 3.2 pin budget ────────────────────────────────────────────────────
//   I²S audio (taken):   9, 11, 22, 23
//   I²C to MPR121s:      18 (SDA), 19 (SCL)
//   PWM pins available:  3, 4, 5, 6, 10, 20, 21  (back-pad: 25, 32)
//   Onboard LED:         13 (digital only)
//
// ── LED layout (9 on MPR121s + 4 on Teensy = 13) ─────────────────────────────
//   Board A (0x5A): touches ELE0–ELE9 (10) · LEDs ELE10, 11           (2)
//   Board C (0x5C): touches ELE0–ELE2 (3)  · LEDs ELE4–ELE9, ELE11    (7)
//   Teensy native:  pins 3, 4, 5, 6                                   (4)
//
// ⚠ Known-fragile LED slots (per project_led_ele9_ele10_wiring memory):
//   • Board A ELE10 (pad 0): needs ELE9 driven as mirror, but ELE9 is now a
//     touch electrode → ELE10 LED almost certainly won't light. Move pad 0's
//     LED to a Teensy PWM pin (10/20/21 are free) if you need it to work.
//   • Board C ELE9  (pad 8): ELE9 had the original open-drain driver fault.
//     May light, but inconsistently — move to Teensy PWM if reliability matters.
// flushLEDs() still applies the ELE9 ← ELE10 mirror in case the conflict is
// resolved on a future board revision.

#include <Audio.h>
#include <Wire.h>
#include <MPR121.h>

// ── Note table (equal temperament, A4 = 440 Hz) ──────────────────────────────
namespace Note {
    // Octave 2
    static constexpr float C2  =  65.41f;
    static constexpr float D2  =  73.42f;
    static constexpr float E2  =  82.41f;
    static constexpr float F2  =  87.31f;
    static constexpr float G2  =  98.00f;
    static constexpr float A2  = 110.00f;
    static constexpr float B2  = 123.47f;
    // Octave 3
    static constexpr float C3  = 130.81f;
    static constexpr float D3  = 146.83f;
    static constexpr float E3  = 164.81f;
    static constexpr float F3  = 174.61f;
    static constexpr float G3  = 196.00f;
    static constexpr float A3  = 220.00f;
    static constexpr float B3  = 246.94f;
    // Octave 4
    static constexpr float C4  = 261.63f;
    static constexpr float D4  = 293.66f;
    static constexpr float E4  = 329.63f;
    static constexpr float F4  = 349.23f;
    static constexpr float G4  = 392.00f;
    static constexpr float A4  = 440.00f;
    static constexpr float B4  = 493.88f;
    // Octave 5
    static constexpr float C5  = 523.25f;
    static constexpr float D5  = 587.33f;
    static constexpr float E5  = 659.25f;
    static constexpr float G5  = 783.99f;
    static constexpr float A5  = 880.00f;
}

// ── Pad descriptor ───────────────────────────────────────────────────────────
// One row per pad. LED routing is universal: ledBoard tells setPadLED where
// to write, ledLoc is the MPR121 electrode (6–11) or Teensy PWM pin number.
static constexpr uint8_t NO_PIN     = 0xFF;
static constexpr uint8_t LED_TEENSY = 0xFE;

struct PadConfig {
    uint8_t senseBoard;   // 0 = 0x5A ("A"), 1 = 0x5C ("C")
    uint8_t senseEle;     // MPR121 electrode 0–11
    uint8_t ledBoard;     // 0, 1, LED_TEENSY, or NO_PIN
    uint8_t ledLoc;       // MPR121 electrode (6–11) OR Teensy PWM pin
    float   freq;         // tone frequency in Hz
};

//                 sense           LED                  note
//                 brd  ele       brd          loc
static constexpr PadConfig PADS[] = {
    {  0,   0,    0,           10,  Note::C3 },  //  0  (⚠ ELE10 needs ELE9 mirror — won't light while ELE9 is touch)
    {  0,   1,    0,           11,  Note::D3 },  //  1
    {  0,   2,    LED_TEENSY,           3,  Note::E3 },  //  2
    {  0,   3,    1,          4,  Note::F3 },  //  3
    {  0,   4,    1,          5,  Note::G3 },  //  4
    {  0,   5,    1,          6,  Note::A3 },  //  5
    {  0,   6,    1,           7,  Note::B3 },  //  6
    {  0,   7,    1,           8,  Note::C4 },  //  7
    {  0,   8,    1,          9,  Note::D4 },  //  8  (⚠ ELE9 may be unreliable)
    {  0,   9,    LED_TEENSY,          4,  Note::E4 },  //  9
    {  1,   0,    1,            11,  Note::F4 },  // 10
    {  1,   1,    LED_TEENSY,  5,  Note::G4 },  // 11
    {  1,   2,    LED_TEENSY,  6,  Note::A4 },  // 12
};
static constexpr uint8_t NUM_PADS = sizeof(PADS) / sizeof(PADS[0]);

// How many touch electrodes are active per board (contiguous from ELE0).
// Board A uses ELE0–ELE9 (10); Board C uses ELE0–ELE2 (3). The MPR121's
// ELE_EN field enables ELE0..ELE(N-1) as touch — anything ≥ N is free for GPIO.
static constexpr uint8_t BOARD_TOUCH_COUNT[2] = { 10, 3 };

// ── MPR121 boards ────────────────────────────────────────────────────────────
static MPR121 boards[2] = { MPR121(0x5A), MPR121(0x5C) };

// ── Audio graph — 13 sine voices → 4 stage mixers → master → I²S ────────────
AudioSynthWaveformSine sine[NUM_PADS];
AudioMixer4            stageMix[4];   // 4 + 4 + 4 + 1 voices
AudioMixer4            masterMix;
AudioOutputI2S         i2s;

AudioConnection v0 (sine[ 0], 0, stageMix[0], 0);
AudioConnection v1 (sine[ 1], 0, stageMix[0], 1);
AudioConnection v2 (sine[ 2], 0, stageMix[0], 2);
AudioConnection v3 (sine[ 3], 0, stageMix[0], 3);
AudioConnection v4 (sine[ 4], 0, stageMix[1], 0);
AudioConnection v5 (sine[ 5], 0, stageMix[1], 1);
AudioConnection v6 (sine[ 6], 0, stageMix[1], 2);
AudioConnection v7 (sine[ 7], 0, stageMix[1], 3);
AudioConnection v8 (sine[ 8], 0, stageMix[2], 0);
AudioConnection v9 (sine[ 9], 0, stageMix[2], 1);
AudioConnection v10(sine[10], 0, stageMix[2], 2);
AudioConnection v11(sine[11], 0, stageMix[2], 3);
AudioConnection v12(sine[12], 0, stageMix[3], 0);

AudioConnection s0(stageMix[0], 0, masterMix, 0);
AudioConnection s1(stageMix[1], 0, masterMix, 1);
AudioConnection s2(stageMix[2], 0, masterMix, 2);
AudioConnection s3(stageMix[3], 0, masterMix, 3);

AudioConnection oL(masterMix, 0, i2s, 0);
AudioConnection oR(masterMix, 0, i2s, 1);

// ── Per-voice state ──────────────────────────────────────────────────────────
int   touchedMax[NUM_PADS];   // strongest delta seen — auto-learns dynamics
float level[NUM_PADS];        // smoothed amplitude

// 13 voices × 0.07 ≈ 0.91 peak (worst-case all-pressed). Sines at random
// phase usually sum much lower than that.
static constexpr float MAX_VOICE_AMP = 0.07f;
static constexpr float SMOOTH        = 0.15f;

// ── LED output ───────────────────────────────────────────────────────────────
// MPR121 PWM is 4-bit per channel (0–15). Teensy analogWrite is 8-bit. The
// public API takes 0–255 and scales; per-board MPR121 brightness is staged in
// boardBri and pushed in one I²C burst per board via flushLEDs().
static uint8_t boardBri[2][8] = { {0}, {0} };

void setPadLED(uint8_t pad, uint8_t pwm /* 0–255 */) {
    const PadConfig& p = PADS[pad];
    if (p.ledBoard == NO_PIN) return;
    if (p.ledBoard == LED_TEENSY) {
        analogWrite(p.ledLoc, pwm);
        return;
    }
    // MPR121: ledLoc is an electrode (6–11) → setLEDs8 index = ledLoc - 4
    uint8_t bri4 = pwm >> 4;
    uint8_t idx  = p.ledLoc - 4;
    boardBri[p.ledBoard][idx] = bri4;
    if (p.ledLoc == 10) {
        // ELE9 must mirror ELE10 or ELE10 won't light.
        boardBri[p.ledBoard][5] = bri4;
    }
}

void flushLEDs() {
    boards[0].setLEDs8(boardBri[0]);
    boards[1].setLEDs8(boardBri[1]);
}

// Build the GPIO/LED-output mask for `boardIdx` directly from PADS[]. Each bit
// of the returned mask = ELE(bit+4); ELE10 implies setting ELE9 mirror too.
// Bits inside the chip's touch range (ELE_EN) are silently ignored by the
// hardware, so it's safe to leave the mirror bit set even when ELE9 is touch.
uint8_t computeLEDMask(uint8_t boardIdx) {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < NUM_PADS; i++) {
        const PadConfig& p = PADS[i];
        if (p.ledBoard == boardIdx) {
            mask |= (uint8_t)(1u << (p.ledLoc - 4));
            if (p.ledLoc == 10) mask |= (uint8_t)(1u << 5); // ELE9 mirror
        }
    }
    return mask;
}

// ── Sensor table snapshot (filled in loop, drawn in printSensors) ───────────
static uint16_t lastFilt[NUM_PADS];
static uint16_t lastBase[NUM_PADS];
static int      lastDelta[NUM_PADS];
static float    lastT[NUM_PADS];

void printSensors() {
    Serial.println(F("─── pad  ELE   filt  base    Δ      t  LED        ──"));
    for (uint8_t i = 0; i < NUM_PADS; i++) {
        const PadConfig& p = PADS[i];
        char ledDesc[14];
        if (p.ledBoard == NO_PIN) {
            strcpy(ledDesc, "none");
        } else if (p.ledBoard == LED_TEENSY) {
            snprintf(ledDesc, sizeof(ledDesc), "T pin %u", p.ledLoc);
        } else {
            snprintf(ledDesc, sizeof(ledDesc), "%c ELE%u",
                     p.ledBoard == 0 ? 'A' : 'C', p.ledLoc);
        }
        char activity = (lastT[i] < 0.05f) ? '.' :
                        (lastT[i] < 0.5f  ? 'o' : '#');
        Serial.printf("    %2u   %c E%u  %4u  %4u  %4d   %4.2f  %-10s %c\n",
                      i,
                      p.senseBoard == 0 ? 'A' : 'C',
                      p.senseEle,
                      lastFilt[i], lastBase[i], lastDelta[i], lastT[i],
                      ledDesc, activity);
    }
}

// ── Setup / loop ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(300);
    AudioMemory(32);

    for (uint8_t i = 0; i < NUM_PADS; i++) {
        sine[i].frequency(PADS[i].freq);
        sine[i].amplitude(0.0f);
        level[i]      = 0.0f;
        touchedMax[i] = 1;
    }

    Wire.begin();
    for (uint8_t b = 0; b < 2; b++) {
        if (!boards[b].begin(BOARD_TOUCH_COUNT[b])) {
            Serial.print(F("ERROR: MPR121 board "));
            Serial.print(b);
            Serial.println(F(" failed to init"));
            while (1) { delay(1000); }
        }
        boards[b].beginLEDs(computeLEDMask(b));
    }

    for (uint8_t i = 0; i < NUM_PADS; i++) {
        if (PADS[i].ledBoard == LED_TEENSY) {
            pinMode(PADS[i].ledLoc, OUTPUT);
            analogWrite(PADS[i].ledLoc, 0);
        }
    }

    Serial.println(F("─── 13-pad Teensy 3.2 — touch + LED test ───"));
    Serial.printf("Touch electrodes:  A=%u (ELE0-ELE%u),  C=%u (ELE0-ELE%u)\n",
                  BOARD_TOUCH_COUNT[0], BOARD_TOUCH_COUNT[0] - 1,
                  BOARD_TOUCH_COUNT[1], BOARD_TOUCH_COUNT[1] - 1);
    Serial.printf("LED GPIO masks:    A=0x%02X,  C=0x%02X\n",
                  computeLEDMask(0), computeLEDMask(1));
    Serial.println(F("Warnings:"));
    Serial.println(F("  • pad 0 LED on A ELE10 — ELE9 mirror unavailable (ELE9 is touch)"));
    Serial.println(F("  • pad 8 LED on C ELE9 — original ELE9 driver fault, may be unreliable"));
    Serial.println(F("Sensor table prints 4×/sec below. Touch a pad → t > 0 + activity '#'."));
    Serial.println();
}

void loop() {
    for (uint8_t i = 0; i < NUM_PADS; i++) {
        const PadConfig& p = PADS[i];
        uint16_t filt = boards[p.senseBoard].filteredData(p.senseEle);
        uint16_t base = boards[p.senseBoard].baselineData(p.senseEle);

        // Touch makes filtered drop below baseline; delta > 0 → contact strength.
        int delta = (int)base - (int)filt;
        if (delta < 0) delta = 0;
        if (delta > touchedMax[i]) touchedMax[i] = delta;

        float span = (float)touchedMax[i];
        float t = (span > 1.0f) ? (float)delta / span : 0.0f;
        if (t > 1.0f) t = 1.0f;
        if (t < 0.05f) t = 0.0f;

        float target = t * MAX_VOICE_AMP;
        level[i] += (target - level[i]) * SMOOTH;
        sine[i].amplitude(level[i]);

        setPadLED(i, (uint8_t)(t * 255.0f));

        lastFilt[i]  = filt;
        lastBase[i]  = base;
        lastDelta[i] = delta;
        lastT[i]     = t;
    }
    flushLEDs();

    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint >= 250) {
        printSensors();
        lastPrint = now;
    }

    delay(2);
}
