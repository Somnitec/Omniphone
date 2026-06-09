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
// Scale and timbre are independent (see config.h). A SCALE supplies the 13
// per-pad frequencies; a TIMBRE supplies the waveform, a 2nd oscillator (sub or
// detune), and the resonant low-pass settings.
//
// Architecture (one shared global instance — Mozzi is a singleton anyway):
//
//   per voice:  mainOsc ─┐
//               osc2   ──┴─ blend(secondMix) ─ × voiceGain ─┐
//                                                            ├─ Σ ─ resonant LP ─ output LP ─►
//   13 voices summed ─────────────────────────────────────-─┘
//
// Notes for the RP2040 CPU budget:
//   • The 2nd oscillator is a *bounded blend* with the main osc (weighted avg),
//     not an additive layer — so the peak per voice is unchanged and the overall
//     level stays constant whatever the timbre (no clipping, fixed loudness).
//   • ONE global resonant low-pass (cutoff follows the loudest pad's proximity)
//     instead of one per voice. Mozzi's cheap LowPassFilter cutoff is an 8-bit
//     value, not Hz, so config Hz are mapped approximately (hzToCutoff()).
//   • A fixed output low-pass (OMNI_OUTPUT_LPF_SHIFT, set in main) rolls off the
//     high-frequency synthesis grit — appropriate for a bass instrument.
// ─────────────────────────────────────────────────────────────────────────────

#include <Oscil.h>
#include <LowPassFilter.h>
#include <tables/sin2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include "config.h"

#define OMNI_OSC_CELLS SIN2048_NUM_CELLS // all four tables are 2048 cells

namespace mozzisynth {

// ── Per-voice oscillators ─────────────────────────────────────────────────────
static Oscil<OMNI_OSC_CELLS, MOZZI_AUDIO_RATE> mainOsc[NUM_SENSORS];
static Oscil<OMNI_OSC_CELLS, MOZZI_AUDIO_RATE> osc2[NUM_SENSORS];   // sub or detune

// ── Per-voice control state ───────────────────────────────────────────────────
static float   baseFreq[NUM_SENSORS];   // current scale frequency per pad
static uint8_t voiceGain[NUM_SENSORS];   // 0–255, smoothed amplitude (control rate)
static uint8_t voiceTarget[NUM_SENSORS]; // 0–255, target from proximity
static int32_t gainSmooth[NUM_SENSORS];  // Q8 per-SAMPLE smoothed gain (click-free fade)

// ── Timbre state ──────────────────────────────────────────────────────────────
static float   osc2Ratio = 0.5f;  // osc2 frequency ÷ main (0.5 sub, ~1.006 detune)
static uint8_t osc2Mix    = 0;     // 0–255 blend weight of osc2 (0 = main only)

// ── Global filter ─────────────────────────────────────────────────────────────
static LowPassFilter lpf;
static uint8_t filterCutoffByte = 200; // current cutoff (smoothed)
static uint8_t filterBaseByte   = 200; // cutoff when all pads are far (dark)
static uint8_t filterMaxByte    = 230; // cutoff when a pad is fully close (bright)

// Map a config cutoff in Hz to Mozzi LowPassFilter's 0–255 byte (perceptual
// approximation, not calibrated Hz). ~9700 Hz → 255 ≈ wide open.
static inline uint8_t hzToCutoff(float hz) {
    float b = hz / 38.0f;
    if (b < 12.0f)  b = 12.0f;        // floor so it never fully closes
    if (b > 255.0f) b = 255.0f;
    return (uint8_t)b;
}

// Pick the Mozzi wavetable for a WAVEFORM_* tag.
static inline const int8_t* tableForWaveform(short w) {
    switch (w) {
        case WAVEFORM_SINE:     return SIN2048_DATA;
        case WAVEFORM_TRIANGLE: return TRIANGLE2048_DATA;
        case WAVEFORM_SQUARE:   return SQUARE_NO_ALIAS_2048_DATA;
        default:                return SAW2048_DATA; // saw / bandlimit-saw
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

// One-time oscillator setup. Call from setup() before startMozzi().
inline void init() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        mainOsc[i].setTable(SIN2048_DATA);
        osc2[i].setTable(SIN2048_DATA);
        baseFreq[i]    = 0.0f;
        voiceGain[i]   = 0;
        voiceTarget[i] = 0;
        gainSmooth[i]  = 0;
    }
    lpf.setResonance(120);
    lpf.setCutoffFreq(filterCutoffByte);
}

// Load a SCALE: the 13 per-pad frequencies. Reapplies the current osc2 ratio.
inline void loadScale(const ScaleSet& sc) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        baseFreq[i] = sc.freqs[i];
        if (baseFreq[i] > 1.0f) {
            mainOsc[i].setFreq(baseFreq[i]);
            osc2[i].setFreq(baseFreq[i] * osc2Ratio);
        }
    }
}

// Load a TIMBRE: waveform, 2nd-oscillator (ratio + mix) and filter settings.
inline void loadTimbre(const TimbreSet& t) {
    const int8_t* table = tableForWaveform(t.waveformType);
    osc2Ratio = t.secondRatio;
    float m = t.secondMix; if (m < 0.0f) m = 0.0f; if (m > 1.0f) m = 1.0f;
    osc2Mix = (uint8_t)(m * 255.0f + 0.5f);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        mainOsc[i].setTable(table);
        osc2[i].setTable(table);
        if (baseFreq[i] > 1.0f) osc2[i].setFreq(baseFreq[i] * osc2Ratio);
    }
    filterBaseByte = hzToCutoff(t.filterBaseHz);
    filterMaxByte  = hzToCutoff(t.filterMaxHz);
    // Map filterQ (≈0.7 flat … 3.0 aggressive) to the 8-bit resonance (60…220).
    float q = t.filterQ; if (q < 0.7f) q = 0.7f; if (q > 3.0f) q = 3.0f;
    lpf.setResonance((uint8_t)(60.0f + (q - 0.7f) * 70.0f));
}

// Live pitch update for one voice (e.g. LFO drift). Cheap — just reprograms inc.
inline void setVoiceFreq(uint8_t i, float hz) {
    if (hz <= 1.0f) return;
    baseFreq[i] = hz;
    mainOsc[i].setFreq(hz);
    osc2[i].setFreq(hz * osc2Ratio);
}

// Set a voice's target amplitude from its proximity intensity (0–1).
inline void setVoiceIntensity(uint8_t i, float intensity) {
    if (intensity < 0.0f) intensity = 0.0f;
    if (intensity > 1.0f) intensity = 1.0f;
    voiceTarget[i] = (uint8_t)(intensity * 255.0f);
}

// Control-rate housekeeping: smooth each voice gain toward its target and slew
// the shared filter cutoff toward the loudest pad's brightness.
inline void updateControl() {
    uint8_t loudest = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        int g = voiceGain[i];
        int t = voiceTarget[i];
        if (t > g) g += (t - g + 1) >> 1;   // fast attack
        else       g += (t - g) >> 2;       // slower release (anti-click)
        voiceGain[i] = (uint8_t)g;
        if (voiceGain[i] > loudest) loudest = voiceGain[i];
    }
    uint16_t target = filterBaseByte +
                      ((uint16_t)(filterMaxByte - filterBaseByte) * loudest) / 255;
    if (target > filterCutoffByte)      filterCutoffByte += (target - filterCutoffByte + 1) >> 1;
    else if (target < filterCutoffByte) filterCutoffByte -= (filterCutoffByte - target + 1) >> 2;
    lpf.setCutoffFreq(filterCutoffByte);
}

// Audio-rate render: blend each voice's two oscillators, sum the gated voices,
// run through the resonant LP, then the fixed output LP. Returns a signed sample.
inline int32_t updateAudio() {
    int32_t acc = 0;
    const int mix2 = osc2Mix;          // 0..255 blend weight of osc2
    const int mix1 = 256 - osc2Mix;    // complementary weight of main osc
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        // Per-SAMPLE gain smoothing: ramps the control-rate steps out at audio
        // rate → click-free proximity fade, no zipper/rattle.
        gainSmooth[i] += (((int32_t)voiceGain[i] << 8) - gainSmooth[i]) >> 5;
        int32_t g = gainSmooth[i] >> 8;             // 0..255
        if (g == 0 && voiceGain[i] == 0) {          // fully closed — just advance phase
            mainOsc[i].next();
            osc2[i].next();
            continue;
        }
        // Bounded blend of the two oscillators → peak stays ±127 (constant level).
        int32_t s = ((int)mainOsc[i].next() * mix1 + (int)osc2[i].next() * mix2) >> 8;
        acc += s * g;                                // full precision (no >>8 grit)
    }
    int32_t out = lpf.next(acc);                     // resonant scale/timbre filter
#if defined(OMNI_OUTPUT_LPF_SHIFT) && (OMNI_OUTPUT_LPF_SHIFT > 0)
    // Fixed output low-pass (two cascaded one-poles = 12 dB/oct) — kills HF grit.
    // Cutoff ≈ audioRate/(2π·2^SHIFT): 1≈2.6 kHz, 2≈1.3 kHz, 3≈650 Hz, 4≈325 Hz.
    static int32_t lp1 = 0, lp2 = 0;
    lp1 += (out - lp1) >> OMNI_OUTPUT_LPF_SHIFT;
    lp2 += (lp1 - lp2) >> OMNI_OUTPUT_LPF_SHIFT;
    out = lp2;
#endif
    return out;
}

} // namespace mozzisynth
