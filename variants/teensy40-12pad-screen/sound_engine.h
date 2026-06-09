#pragma once
#include <Audio.h>
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// Sound engine — per-voice subtractive synth architecture
//
// Signal chain per voice:
//
//   mainOsc ─┐
//             ├─ voiceMix ─ filter (LP) ─ ampGate ─► to stage mixer
//   subOsc  ─┘                             dcAmp ─┘
//
// mainOsc  : selectable waveform (triangle, bandlimited saw, sine)
// subOsc   : sine one octave below for bass weight
// voiceMix : blends main + sub oscillator levels
// filter   : state-variable filter, lowpass output, proximity-modulated cutoff
// dcAmp    : sample-accurate linear ramp for click-free amplitude control
// ampGate  : multiplies filtered audio by DC envelope
// ─────────────────────────────────────────────────────────────────────────────

struct Voice {
    AudioSynthWaveformModulated mainOsc;  // modulated → enables per-voice FM (FM Poly)
    AudioSynthWaveformSine      modOsc;   // FM modulator (silent unless FM Poly)
    AudioSynthWaveformSine      subOsc;
    AudioMixer4                 voiceMix;
    AudioFilterStateVariable    filter;
    AudioSynthWaveformDc        dcAmp;
    AudioEffectMultiply         ampGate;
};

// ── Voice wiring macro ───────────────────────────────────────────────────────
// Declares the 6 AudioConnection objects that wire up one voice internally
// plus the connection from ampGate to a stage mixer channel.
//
// Usage: VOICE_WIRING(0, stageMix[0], 0)
//        VOICE_WIRING(1, stageMix[0], 1)
//        ...
#define VOICE_WIRING(N, STAGE, CH)                                              \
    AudioConnection vc##N##_modToOsc  (voices[N].modOsc,   0, voices[N].mainOsc,  0); \
    AudioConnection vc##N##_mainToMix (voices[N].mainOsc,  0, voices[N].voiceMix, 0); \
    AudioConnection vc##N##_subToMix  (voices[N].subOsc,   0, voices[N].voiceMix, 1); \
    AudioConnection vc##N##_mixToFilt (voices[N].voiceMix, 0, voices[N].filter,   0); \
    AudioConnection vc##N##_filtToGate(voices[N].filter,   0, voices[N].ampGate,  0); \
    AudioConnection vc##N##_dcToGate  (voices[N].dcAmp,    0, voices[N].ampGate,  1); \
    AudioConnection vc##N##_gateOut   (voices[N].ampGate,  0, STAGE,             CH);

// ── Voice control functions ──────────────────────────────────────────────────

// Initialise a voice with a frequency and a timbre. Full setup — used at boot
// (starts silent; proximity ramps it up).
inline void initVoice(Voice& v, const TimbreSet& t, float freq) {
    // Main oscillator (modulated; FM is off until FM Poly turns it on)
    v.mainOsc.begin(MAIN_OSC_AMPLITUDE, freq, t.waveformType);
    v.mainOsc.frequencyModulation(0.0f);

    // FM modulator — silent in every mode except FM Poly
    v.modOsc.frequency(freq * FM_POLY_RATIO);
    v.modOsc.amplitude(0.0f);

    // Sub oscillator — one octave below
    v.subOsc.frequency(freq * 0.5f);
    v.subOsc.amplitude(SUB_OSC_AMPLITUDE);

    // Voice mixer: channel 0 = main, channel 1 = sub
    v.voiceMix.gain(0, 1.0f);
    v.voiceMix.gain(1, t.subMix);
    v.voiceMix.gain(2, 0.0f);
    v.voiceMix.gain(3, 0.0f);

    // Filter — lowpass, start dark
    v.filter.frequency(t.filterBaseHz);
    v.filter.resonance(t.filterQ);
    v.filter.octaveControl(1.5f);

    // Amplitude gate — start silent
    v.dcAmp.amplitude(0.0f);
}

// Switch only the oscillator waveform of a running voice (keeps amplitude,
// phase, frequency). Used on a live timbre change — the waveform is discrete so
// it can't crossfade, but the rest of the timbre is morphed (setVoiceMorph).
inline void setVoiceWaveform(Voice& v, short waveformType) {
    v.mainOsc.begin(waveformType);
}

// Apply the (continuously morphing) timbre params to a running voice: sub-bass
// blend and filter resonance. Called every frame with interpolated values so a
// timbre change slides instead of jumping. The filter cutoff is handled
// separately by setVoiceFilter() (proximity-driven).
inline void setVoiceMorph(Voice& v, float subMix, float filterQ) {
    v.voiceMix.gain(1, subMix);
    v.filter.resonance(filterQ);
}

// Set voice amplitude with sample-accurate linear ramp (click-free).
inline void setVoiceAmplitude(Voice& v, float amplitude) {
    v.dcAmp.amplitude(amplitude, AMP_RAMP_MS);
}

// Set voice frequency (main oscillator, FM modulator, and sub).
inline void setVoiceFrequency(Voice& v, float freq) {
    v.mainOsc.frequency(freq);
    v.modOsc.frequency(freq * FM_POLY_RATIO);
    v.subOsc.frequency(freq * 0.5f);
}

// Per-voice FM (FM Poly mode): set the modulation depth (octaves) and modulator
// level. depth=0/amp=0 makes the voice a plain oscillator again (all other modes).
inline void setVoiceFM(Voice& v, float depthOctaves, float modAmp) {
    v.mainOsc.frequencyModulation(depthOctaves);
    v.modOsc.amplitude(modAmp);
}

// Map proximity intensity (0–1) to filter cutoff.
inline void setVoiceFilter(Voice& v, float intensity, float baseHz, float maxHz) {
    float cutoff = baseHz + intensity * (maxHz - baseHz);
    v.filter.frequency(cutoff);
}
