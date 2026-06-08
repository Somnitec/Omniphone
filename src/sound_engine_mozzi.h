#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Sound engine — Mozzi backend (Pico / ESP32-S3)
//
// The Teensy build uses the Teensy Audio Library (sound_engine.h), which is
// DMA/hardware-specific and does NOT run on the RP2040 or ESP32-S3. This file
// reimplements the same musical idea with Mozzi, which DOES support all three
// MCUs, so the rest of the firmware (config.h scales, proximity_engine.h) is
// shared unchanged.
//
// Architecture (one shared global instance — Mozzi is a singleton anyway):
//
//   per voice:  mainOsc (wavetable) ─┐
//               subOsc  (sine, −1oct)─┴─ × voiceGain ─┐
//                                                      ├─ Σ ─ global LP filter ─► out
//   13 voices summed ───────────────────────────────-─┘
//
// Differences from the Teensy engine, by design, to fit the RP2040 CPU budget:
//   • ONE global resonant low-pass filter (cutoff follows the loudest pad's
//     proximity) instead of one filter per voice. Still gives "brighter when
//     your hand is closer", just shared across the chord.
//   • Mozzi's cheap LowPassFilter cutoff is an 8-bit value, not Hz, so the
//     config's filterBaseHz/MaxHz are mapped approximately (see hzToCutoff()).
//   • Voice gain is smoothed at control rate (updateControl), held constant per
//     audio block — no per-sample ramp, which keeps updateAudio() tight.
//
// Build-time switch:  define MOZZI_NO_SUB to drop the 13 sub-oscillators if the
// RP2040 runs out of audio-rate headroom (halves the oscillator count).
// ─────────────────────────────────────────────────────────────────────────────

#include <Oscil.h>
#include <LowPassFilter.h>
#include <tables/sin2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/saw2048_int8.h>

#include "config.h"

#define OMNI_OSC_CELLS SIN2048_NUM_CELLS // all three tables are 2048 cells

namespace mozzisynth {

// ── Per-voice oscillators ─────────────────────────────────────────────────────
static Oscil<OMNI_OSC_CELLS, MOZZI_AUDIO_RATE> mainOsc[NUM_SENSORS];
#ifndef MOZZI_NO_SUB
static Oscil<OMNI_OSC_CELLS, MOZZI_AUDIO_RATE> subOsc[NUM_SENSORS]; // sine, −1 oct
#endif

// ── Per-voice control state (set at control rate, read in updateAudio) ─────────
static uint8_t voiceGain[NUM_SENSORS]; // 0–255, smoothed amplitude
static uint8_t voiceTarget[NUM_SENSORS]; // 0–255, target from proximity
static uint8_t subGain;                // global sub level (0–255), from soundset

// ── Global filter ─────────────────────────────────────────────────────────────
static LowPassFilter lpf;
static uint8_t  filterCutoffByte = 200; // current cutoff (smoothed)
static uint8_t  filterBaseByte   = 60;  // cutoff when all pads are far (dark)
static uint8_t  filterMaxByte    = 230; // cutoff when a pad is fully close (bright)

// Map a config cutoff in Hz to Mozzi LowPassFilter's 0–255 byte. Rough: the
// cheap filter's corner ≈ (byte/255) of a fraction of the audio rate, so this is
// a perceptual approximation, not a calibrated Hz. Tune the two endpoints by ear.
static inline uint8_t hzToCutoff(float hz) {
    float b = hz / 38.0f;             // ~9700 Hz → 255
    if (b < 12.0f)  b = 12.0f;        // floor so it never fully closes
    if (b > 255.0f) b = 255.0f;
    return (uint8_t)b;
}

// Pick the Mozzi wavetable that best matches a Teensy WAVEFORM_* tag.
static inline const int8_t* tableForWaveform(short w) {
    switch (w) {
        case WAVEFORM_SINE:     return SIN2048_DATA;
        case WAVEFORM_TRIANGLE: return TRIANGLE2048_DATA;
        default:                return SAW2048_DATA; // saw / bandlimit-saw / square
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

// One-time oscillator setup. Call from setup() before startMozzi().
inline void init() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        mainOsc[i].setTable(SAW2048_DATA);
        mainOsc[i].setFreq(220.0f);
        voiceGain[i]   = 0;
        voiceTarget[i] = 0;
#ifndef MOZZI_NO_SUB
        subOsc[i].setTable(SIN2048_DATA);
        subOsc[i].setFreq(110.0f);
#endif
    }
    subGain = 90;
    lpf.setResonance(120);
    lpf.setCutoffFreq(filterCutoffByte);
}

// Load a sound set: per-voice frequencies, the oscillator waveform, the sub
// level, and the filter sweep endpoints. Pads beyond the set's note count (the
// zero-filled tail of freqs[]) are left silent.
inline void loadSoundSet(const SoundSet& ss) {
    const int8_t* table = tableForWaveform(ss.waveformType);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        mainOsc[i].setTable(table);
        if (ss.freqs[i] > 1.0f) mainOsc[i].setFreq(ss.freqs[i]);
#ifndef MOZZI_NO_SUB
        if (ss.freqs[i] > 1.0f) subOsc[i].setFreq(ss.freqs[i] * 0.5f);
#endif
    }
    subGain = (uint8_t)(ss.subMix * 255.0f);
    filterBaseByte = hzToCutoff(ss.filterBaseHz);
    filterMaxByte  = hzToCutoff(ss.filterMaxHz);
    // Map filterQ (≈0.7 flat … 3.0 aggressive) to the 8-bit resonance.
    float q = ss.filterQ;
    if (q < 0.7f) q = 0.7f;
    if (q > 3.0f) q = 3.0f;
    lpf.setResonance((uint8_t)(60.0f + (q - 0.7f) * 70.0f)); // 60…220
}

// Live pitch update for one voice (LFO drift). Cheap — just reprograms phase inc.
inline void setVoiceFreq(uint8_t i, float hz) {
    if (hz <= 1.0f) return;
    mainOsc[i].setFreq(hz);
#ifndef MOZZI_NO_SUB
    subOsc[i].setFreq(hz * 0.5f);
#endif
}

// Set a voice's target amplitude from its proximity intensity (0–1). Smoothed
// toward in updateControl().
inline void setVoiceIntensity(uint8_t i, float intensity) {
    if (intensity < 0.0f) intensity = 0.0f;
    if (intensity > 1.0f) intensity = 1.0f;
    voiceTarget[i] = (uint8_t)(intensity * 255.0f);
}

// Control-rate housekeeping: smooth each voice gain toward its target and slew
// the shared filter cutoff toward the loudest pad's brightness. Call once per
// updateControl().
inline void updateControl() {
    uint8_t loudest = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        // 1-pole-ish smoothing (asymmetric: snappier up, gentler down).
        int g = voiceGain[i];
        int t = voiceTarget[i];
        if (t > g)      g += (t - g + 1) >> 1;   // fast attack
        else            g += (t - g) >> 2;       // slower release (anti-click)
        voiceGain[i] = (uint8_t)g;
        if (voiceGain[i] > loudest) loudest = voiceGain[i];
    }
    // Filter brightness follows the loudest pad (loudest 0–255 → base..max).
    uint16_t target = filterBaseByte +
                      ((uint16_t)(filterMaxByte - filterBaseByte) * loudest) / 255;
    if (target > filterCutoffByte)      filterCutoffByte += (target - filterCutoffByte + 1) >> 1;
    else if (target < filterCutoffByte) filterCutoffByte -= (filterCutoffByte - target + 1) >> 2;
    lpf.setCutoffFreq(filterCutoffByte);
}

// Audio-rate render: sum the gated voices and run them through the global LP
// filter. Returns a signed sample (~14-bit) for MonoOutput::fromAlmostNBit(14).
inline int32_t updateAudio() {
    int32_t acc = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint8_t g = voiceGain[i];
        if (g == 0) {
#ifndef MOZZI_NO_SUB
            // still advance phase so the osc doesn't click when it re-opens
            mainOsc[i].next();
            subOsc[i].next();
#else
            mainOsc[i].next();
#endif
            continue;
        }
        acc += ((int)mainOsc[i].next() * g) >> 8;        // ±~126
#ifndef MOZZI_NO_SUB
        acc += (((int)subOsc[i].next() * g) >> 8) * subGain >> 8; // sub × voice × subMix
#endif
    }
    return lpf.next(acc);
}

} // namespace mozzisynth
