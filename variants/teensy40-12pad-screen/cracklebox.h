#pragma once
#include <Audio.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// AudioCracklebox — chaotic logic-oscillator network (à la Michel Waisvisz's
// Kraakdoos / Cracklebox).
//
// One square oscillator per HELD pad (set via setActive()). With a single pad
// it's just that clean square wave; each extra pad adds another oscillator whose
// square is XOR'd into the logic output — the harsh digital layering of the real
// box. The active oscillators also cross-couple (each one's frequency bent by
// the others' squares — the "bridging" your fingers do), with strength set by
// proximity, so pressing harder pushes it from clean tones into squealing chaos.
// ─────────────────────────────────────────────────────────────────────────────
#define CRACKLE_MAX 12

class AudioCracklebox : public AudioStream {
public:
    AudioCracklebox() : AudioStream(0, nullptr) {}

    void setActive(uint8_t n)         { _n = (n > CRACKLE_MAX) ? CRACKLE_MAX : n; }
    void setBase(uint8_t i, float hz) { if (i < CRACKLE_MAX) _base[i] = hz; }
    void setCouple(float c)           { _couple = c; }     // 0..~1 cross-coupling
    void setAmp(float a)              { _ampTarget = a; }  // output level (smoothed)

    virtual void update();

private:
    uint8_t _n = 0;                      // active oscillators = held pads
    float   _phase[CRACKLE_MAX] = { 0 };
    float   _base [CRACKLE_MAX] = { 0 };
    float   _sq   [CRACKLE_MAX] = { 0 };
    float   _couple = 0.4f, _amp = 0.0f, _ampTarget = 0.0f;

    static inline float wrap01(float p) { p -= (float)(int)p; return p < 0.0f ? p + 1.0f : p; }
};

inline void AudioCracklebox::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    if (!isfinite(_amp)) _amp = 0; // never let a divergence NaN the whole mix

    // Silent (not the active mode) → emit zeros and skip the per-sample work.
    if (_n == 0 && _amp < 0.0003f && _ampTarget < 0.0003f) {
        memset(out->data, 0, sizeof(out->data));
        transmit(out); release(out); return;
    }

    const float fs = AUDIO_SAMPLE_RATE_EXACT;

    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        _amp += (_ampTarget > _amp ? 0.02f : 0.004f) * (_ampTarget - _amp); // fast attack, smooth release

        if (_n == 0) { out->data[s] = 0; continue; }

        // Advance each active oscillator; cross-couple it to the others (no
        // coupling when only one is active → a clean square wave).
        float sum = 0.0f;
        for (uint8_t i = 0; i < _n; i++)
        {
            float mod = 0.0f;
            for (uint8_t j = 0; j < _n; j++) if (j != i) mod += _sq[j];
            float f = _base[i] * (1.0f + _couple * mod);
            if (f < 1.0f) f = 1.0f;
            if (f > fs * 0.45f) f = fs * 0.45f;
            _phase[i] = wrap01(_phase[i] + f / fs);
            _sq[i] = (_phase[i] < 0.5f) ? 1.0f : -1.0f;
            sum += _sq[i];
        }

        // XOR the active squares for the digital crackle; with one pad this is
        // just that square. Blend a little of the analog sum for body.
        bool b = false;
        for (uint8_t i = 0; i < _n; i++) b ^= (_sq[i] > 0.0f);
        float logic  = b ? 1.0f : -1.0f;
        float analog = sum / (float)_n;
        float v = 0.65f * logic + 0.35f * analog;
        float o = _amp * v;
        if (!isfinite(o)) o = 0.0f; else if (o > 1.0f) o = 1.0f; else if (o < -1.0f) o = -1.0f;
        out->data[s] = (int16_t)(o * 9000.0f);
    }

    transmit(out);
    release(out);
}
