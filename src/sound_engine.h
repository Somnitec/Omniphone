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
    AudioSynthWaveform       mainOsc;
    AudioSynthWaveformSine   subOsc;
    AudioMixer4              voiceMix;
    AudioFilterStateVariable filter;
    AudioSynthWaveformDc     dcAmp;
    AudioEffectMultiply      ampGate;
};

// ── Voice wiring macro ───────────────────────────────────────────────────────
// Declares the 6 AudioConnection objects that wire up one voice internally
// plus the connection from ampGate to a stage mixer channel.
//
// Usage: VOICE_WIRING(0, stageMix[0], 0)
//        VOICE_WIRING(1, stageMix[0], 1)
//        ...
#define VOICE_WIRING(N, STAGE, CH)                                              \
    AudioConnection vc##N##_mainToMix (voices[N].mainOsc,  0, voices[N].voiceMix, 0); \
    AudioConnection vc##N##_subToMix  (voices[N].subOsc,   0, voices[N].voiceMix, 1); \
    AudioConnection vc##N##_mixToFilt (voices[N].voiceMix, 0, voices[N].filter,   0); \
    AudioConnection vc##N##_filtToGate(voices[N].filter,   0, voices[N].ampGate,  0); \
    AudioConnection vc##N##_dcToGate  (voices[N].dcAmp,    0, voices[N].ampGate,  1); \
    AudioConnection vc##N##_gateOut   (voices[N].ampGate,  0, STAGE,             CH);

// ── Voice control functions ──────────────────────────────────────────────────

// Initialise a voice with a waveform type, frequency, and sound set parameters.
inline void initVoice(Voice& v, const SoundSet& ss, float freq) {
    // Main oscillator
    v.mainOsc.begin(MAIN_OSC_AMPLITUDE, freq, ss.waveformType);

    // Sub oscillator — one octave below
    v.subOsc.frequency(freq * 0.5f);
    v.subOsc.amplitude(SUB_OSC_AMPLITUDE);

    // Voice mixer: channel 0 = main, channel 1 = sub
    v.voiceMix.gain(0, 1.0f);
    v.voiceMix.gain(1, ss.subMix);
    v.voiceMix.gain(2, 0.0f);
    v.voiceMix.gain(3, 0.0f);

    // Filter — lowpass, start dark
    v.filter.frequency(ss.filterBaseHz);
    v.filter.resonance(ss.filterQ);
    v.filter.octaveControl(1.5f);

    // Amplitude gate — start silent
    v.dcAmp.amplitude(0.0f);
}

// Set voice amplitude with sample-accurate linear ramp (click-free).
inline void setVoiceAmplitude(Voice& v, float amplitude) {
    v.dcAmp.amplitude(amplitude, AMP_RAMP_MS);
}

// Set voice frequency (both main oscillator and sub).
inline void setVoiceFrequency(Voice& v, float freq) {
    v.mainOsc.frequency(freq);
    v.subOsc.frequency(freq * 0.5f);
}

// Map proximity intensity (0–1) to filter cutoff.
inline void setVoiceFilter(Voice& v, float intensity, float baseHz, float maxHz) {
    float cutoff = baseHz + intensity * (maxHz - baseHz);
    v.filter.frequency(cutoff);
}
