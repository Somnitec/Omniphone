// Teensy LC — 7-voice touch synth (standalone test build).
//
// Seven sine voices voicing a C major 7 chord across two octaves, driven by
// the onboard TSI touch peripheral. Audio out via PCM5102A on the standard
// Teensy LC I2S pins (BCLK=9, MCLK=11, LRCLK=23, TX=22).
//
// Built to be flashed independently of the main firmware — see the
// `[env:lc_seven_voice]` block in platformio.ini.

#include <Audio.h>

// ── Diatonic note table (equal temperament, A4 = 440 Hz) ─────────────────────
// Use these in the `freq[]` array below to retune without hunting for Hz
// values. Add sharps/flats here if you need them.
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

// ── Voice / pad configuration ────────────────────────────────────────────────
// LC touch-capable pins clear of I2S (22/23 are used by audio out): 0, 1, 3,
// 4, 15, 16, 17, 18. Pick any 7.
//
// Tuning: C major 7 (root, 3rd, 5th, 7th) extended into the next octave so
// every adjacent pair is a consonant chord tone. Swap any entry for another
// Note::… constant to retune that pad.
static constexpr int   NVOICES = 7;
static constexpr int   touchPin[NVOICES] = { 0, 1, 3, 4, 15, 16, 17 };
static constexpr float freq[NVOICES] = {
    Note::C3, Note::E3, Note::G3, Note::B3,   // Cmaj7
    Note::C4, Note::E4, Note::G4,             // upper octave
};

// ── Audio graph ──────────────────────────────────────────────────────────────
// AudioMixer4 takes 4 inputs, so two stage mixers cascade into a master mixer
// (4 + 3 voices). Unity gain everywhere — never call .gain() on the LC.
AudioSynthWaveformSine sine[NVOICES];
AudioMixer4            stageMix[2];
AudioMixer4            masterMix;
AudioOutputI2S         i2s;

AudioConnection c0(sine[0], 0, stageMix[0], 0);
AudioConnection c1(sine[1], 0, stageMix[0], 1);
AudioConnection c2(sine[2], 0, stageMix[0], 2);
AudioConnection c3(sine[3], 0, stageMix[0], 3);
AudioConnection c4(sine[4], 0, stageMix[1], 0);
AudioConnection c5(sine[5], 0, stageMix[1], 1);
AudioConnection c6(sine[6], 0, stageMix[1], 2);
AudioConnection s0(stageMix[0], 0, masterMix, 0);
AudioConnection s1(stageMix[1], 0, masterMix, 1);
AudioConnection oL(masterMix, 0, i2s, 0);
AudioConnection oR(masterMix, 0, i2s, 1);

// ── Touch state ──────────────────────────────────────────────────────────────
int   baseline[NVOICES];
int   touchedMax[NVOICES];
float level[NVOICES];

// 7 voices × 0.13 ≈ 0.91 peak — keeps headroom if every pad is pressed at once.
static constexpr float MAX_VOICE_AMP = 0.13f;
static constexpr float SMOOTH        = 0.15f;

void setup() {
    Serial.begin(115200);
    AudioMemory(24);

    for (int i = 0; i < NVOICES; i++) {
        sine[i].frequency(freq[i]);
        sine[i].amplitude(0.0f);
    }

    Serial.println("Calibrating baseline. Do NOT touch the pads...");
    delay(500);
    for (int i = 0; i < NVOICES; i++) {
        long sum = 0;
        for (int n = 0; n < 32; n++) { sum += touchRead(touchPin[i]); delay(5); }
        baseline[i]   = sum / 32;
        touchedMax[i] = baseline[i] + 1;
        level[i]      = 0.0f;
    }
    Serial.println("Baseline set. Press each pad firmly once to learn its range.");
}

void loop() {
    for (int i = 0; i < NVOICES; i++) {
        int raw = touchRead(touchPin[i]);

        if (raw > touchedMax[i]) touchedMax[i] = raw;

        float span = (float)(touchedMax[i] - baseline[i]);
        float t = (span > 1.0f) ? (raw - baseline[i]) / span : 0.0f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        if (t < 0.05f) t = 0.0f;

        float target = t * MAX_VOICE_AMP;
        level[i] += (target - level[i]) * SMOOTH;
        sine[i].amplitude(level[i]);
    }
    delay(2);
}
