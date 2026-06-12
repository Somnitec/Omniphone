#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"

// ─────────────────────────────────────────────────────────────────────────────
// AudioBreathFlute — digital-waveguide wind instrument (shakuhachi / pan flute).
//
// A different physical-modeling family from the modal hang/bowls: a true
// waveguide. Each pad is an air column (delay line) excited by a nonlinear
// jet (the classic STK flute: jet delay + x·(x²−1) sigmoid). Proximity =
// breath pressure — hover for a barely-there airy whisper, lean in to make the
// note speak, push hard and it overblows. Breath noise and a slow vibrato make
// it feel alive. All 12 pads are independent columns → fully polyphonic wind.
// ─────────────────────────────────────────────────────────────────────────────
#define FLUTE_VOICES   12
#define FLUTE_BORE_LEN 1024   // supports notes down to ~45 Hz
#define FLUTE_JET_LEN  512

static constexpr float FLUTE_JET_RATIO  = 0.50f;  // jet delay / bore delay (0.5 = octave-friendly)
static constexpr float FLUTE_JET_REFL   = 0.50f;  // bore→jet feedback
static constexpr float FLUTE_END_REFL   = 0.50f;  // open-end reflection
// Breath character vs distance: far away it's mostly air (soft hiss, the note
// barely speaking); touching, the turbulence drops away and the tone is pure.
static constexpr float FLUTE_NOISE_FAR  = 0.30f;  // jet turbulence at a whisper
static constexpr float FLUTE_NOISE_NEAR = 0.02f;  // …at full touch (pure tone)
static constexpr float FLUTE_HISS       = 1.10f;  // direct breath-hiss level at distance
static constexpr float FLUTE_VIB_HZ     = 4.8f;   // slow breath vibrato
static constexpr float FLUTE_VIB_AMT    = 0.015f; // vibrato depth on the breath
static constexpr float FLUTE_BREATH_MAX = 1.10f;  // pressure at full force (≳1 overblows)
static constexpr float FLUTE_ATTACK_S   = 0.10f;  // breath swell
static constexpr float FLUTE_RELEASE_S  = 0.25f;
static constexpr float FLUTE_LOSS       = 0.998f; // bore loop loss

class AudioBreathFlute : public AudioStream {
public:
    AudioBreathFlute() : AudioStream(0, nullptr)
    {
        for (uint8_t v = 0; v < FLUTE_VOICES; v++) {
            _bore[v].init(_boreBuf[v], FLUTE_BORE_LEN);
            _jet [v].init(_jetBuf [v], FLUTE_JET_LEN);
        }
    }

    void setFreq(uint8_t v, float hz)
    {
        if (v >= FLUTE_VOICES || !(hz > 45.0f)) return;
        float L = AUDIO_SAMPLE_RATE_EXACT / hz - 2.0f;
        if (L > FLUTE_BORE_LEN - 2) L = FLUTE_BORE_LEN - 2;
        if (L < 4.0f) L = 4.0f;
        _boreD[v] = L;
        float J = L * FLUTE_JET_RATIO;
        if (J > FLUTE_JET_LEN - 2) J = FLUTE_JET_LEN - 2;
        if (J < 2.0f) J = 2.0f;
        _jetD[v] = J;
    }

    void setForce(uint8_t v, float f)
    { if (v < FLUTE_VOICES) _force[v] = (f < 0.0f) ? 0.0f : (f > 1.0f ? 1.0f : f); }

    virtual void update();

private:
    float     _boreBuf[FLUTE_VOICES][FLUTE_BORE_LEN];
    float     _jetBuf [FLUTE_VOICES][FLUTE_JET_LEN];
    DelayLine _bore[FLUTE_VOICES], _jet[FLUTE_VOICES];
    float     _boreD[FLUTE_VOICES] = { 0 }, _jetD[FLUTE_VOICES] = { 0 };
    float     _force[FLUTE_VOICES] = { 0 };
    float     _env  [FLUTE_VOICES] = { 0 };
    float     _lp   [FLUTE_VOICES] = { 0 };  // bore loop filter state
    float     _hiss [FLUTE_VOICES] = { 0 };  // direct breath-hiss low-pass state
    float     _vibPh = 0.0f;
    NoiseGen  _rng;
};

inline void AudioBreathFlute::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    float act = 0.0f;
    for (uint8_t v = 0; v < FLUTE_VOICES; v++) {
        if (!isfinite(_env[v]) || !isfinite(_lp[v])) { _env[v] = _lp[v] = 0.0f; _bore[v].clear(); _jet[v].clear(); }
        if (_force[v] > act) act = _force[v];
        if (_env[v]   > act) act = _env[v];
    }
    if (act < 1.0e-4f) { memset(out->data, 0, sizeof(out->data)); transmit(out); release(out); return; }

    const float fs = AUDIO_SAMPLE_RATE_EXACT;
    const float up = 1.0f - expf(-1.0f / (FLUTE_ATTACK_S  * fs));
    const float dn = 1.0f - expf(-1.0f / (FLUTE_RELEASE_S * fs));

    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        _vibPh += FLUTE_VIB_HZ / fs;
        if (_vibPh >= 1.0f) _vibPh -= 1.0f;
        float vib = 1.0f + FLUTE_VIB_AMT * sinLut01(_vibPh);

        float mix = 0.0f;
        for (uint8_t v = 0; v < FLUTE_VOICES; v++)
        {
            _env[v] += (_force[v] > _env[v] ? up : dn) * (_force[v] - _env[v]);
            float e = _env[v];
            if (e < 1.0e-4f || _boreD[v] < 4.0f) continue;

            // Turbulence fades out as the hand comes in: airy far, pure close.
            float n  = _rng.next();
            float na = FLUTE_NOISE_FAR + (FLUTE_NOISE_NEAR - FLUTE_NOISE_FAR) * e;
            float breath = e * FLUTE_BREATH_MAX * vib * (1.0f + na * n);

            float boreOut = _bore[v].readF(_boreD[v]);
            // jet: pressure differential travels down the jet delay…
            _jet[v].push(breath - FLUTE_JET_REFL * boreOut);
            float jx = _jet[v].readF(_jetD[v]);
            // …through the sigmoid jet nonlinearity x(x²−1), clamped
            float nl = jx * (jx * jx - 1.0f);
            if (nl > 1.0f) nl = 1.0f; else if (nl < -1.0f) nl = -1.0f;
            // back into the bore with the open-end reflection
            float boreIn = nl + FLUTE_END_REFL * boreOut;
            _lp[v] += 0.55f * (boreIn - _lp[v]);          // loop low-pass (losses)
            _bore[v].push(_lp[v] * FLUTE_LOSS);

            // Direct breath hiss, strongest at a distance, gone at full touch:
            // soft filtered air that the (quiet) tone floats inside.
            _hiss[v] += 0.09f * (n - _hiss[v]);
            float far = (1.0f - e);
            mix += _hiss[v] * FLUTE_HISS * e * far * far;
            mix += boreOut * e * 0.6f;
        }
        if (!isfinite(mix)) mix = 0.0f; else if (mix > 2.0f) mix = 2.0f; else if (mix < -2.0f) mix = -2.0f;
        out->data[s] = (int16_t)(mix * 7500.0f);
    }

    transmit(out);
    release(out);
}
