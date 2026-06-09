// ─────────────────────────────────────────────────────────────────────────────
// Pico / Mozzi audio-path isolation test — sine Cmaj7 chord, no sensors.
//
// Minimal standalone sketch: four sine oscillators (C-E-G-B) summed straight to
// the PCM5102A over I2S, with the SAME Mozzi config as the pico-13pad firmware.
// Purpose: decide whether "digital/noisy, one channel" is the SYNTH/firmware or
// the I2S OUTPUT path itself.
//   • Clean chord here  → output path is fine; the problem is in the full
//                         firmware's summing/scaling (or the sensors).
//   • Noisy/one-channel → it's the I2S output config / wiring (see notes below).
//
// Build:  pio run -e pico-chord-test -t upload
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>

// ── Mozzi config — MUST precede <Mozzi.h>; mirrors variants/pico-13pad ────────
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_CHANNELS MOZZI_STEREO   // duplicate mono → both L+R of the I2S frame
#define MOZZI_CONTROL_RATE   256
#define MOZZI_AUDIO_RATE     32768
#define MOZZI_AUDIO_MODE     MOZZI_OUTPUT_I2S_DAC
#define MOZZI_I2S_PIN_BCK    20
#define MOZZI_I2S_PIN_WS     21            // = BCK + 1 (RP2040 PIO requirement)
#define MOZZI_I2S_PIN_DATA   22
#include <Mozzi.h>

#include <Oscil.h>
#include <tables/sin2048_int8.h>

static constexpr int NVOICES = 4;
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc[NVOICES];

// Cmaj7: C4, E4, G4, B4 (equal temperament)
static const float freqs[NVOICES] = { 261.63f, 329.63f, 392.00f, 493.88f };

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < NVOICES; i++) {
        osc[i].setTable(SIN2048_DATA);
        osc[i].setFreq(freqs[i]);
    }
    startMozzi();
    Serial.println(F("# pico_mozzi_chord: sine Cmaj7 on I2S DAC (BCK=20 WS=21 DATA=22)"));
}

void updateControl() {}

AudioOutput updateAudio() {
    int32_t acc = 0;
    for (int i = 0; i < NVOICES; i++) acc += osc[i].next();   // 4 × ±127 ≈ ±508
    return StereoOutput::fromAlmostNBit(10, acc, acc);        // same on L and R
}

void loop() { audioHook(); }
