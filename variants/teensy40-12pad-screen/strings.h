#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"
#include "sample_data.h"   // STRINGS_SAMPLE / STRINGS_LEN / STRINGS_ROOT_HZ / STRINGS_RATE

// ─────────────────────────────────────────────────────────────────────────────
// AudioStringPad — "Aurora" orchestral string-section pads (samples/).
//
// Per pad, three detuned readers loop the flash string sample, transposed from
// the recording's root to the pad's note. The detunes slowly drift, so twelve
// pads × three readers behave like a big slightly-out-of-phase section.
// Proximity = bow pressure: a slow swell in (≈200 ms) and a long release
// (≈600 ms), so notes bloom and overlap like a real section breathing.
//
// Replace samples/strings_<rootHz>.wav (a clean seamless sustain loop) + rerun
// tools/wav2header.py to recast the whole section (choir, organ, glass…).
// ─────────────────────────────────────────────────────────────────────────────
#define STR_VOICES  12
#define STR_READERS 3

static constexpr float STR_DETUNE[STR_READERS] = { -0.004f, 0.0f, 0.004f }; // ±0.4 %
static constexpr float STR_DRIFT      = 0.0012f; // extra slow wander per reader
static constexpr float STR_ATTACK_S   = 0.20f;   // swell-in
static constexpr float STR_RELEASE_S  = 0.60f;   // bow lift
static constexpr float STR_FORCE_EXP  = 1.3f;    // force→level curve

class AudioStringPad : public AudioStream {
public:
    AudioStringPad() : AudioStream(0, nullptr) {}

    void setFreq(uint8_t v, float hz) { if (v < STR_VOICES && hz > 0.0f) _freq[v] = hz; }
    void setForce(uint8_t v, float f)
    { if (v < STR_VOICES) _force[v] = (f < 0.0f) ? 0.0f : (f > 1.0f ? 1.0f : f); }
    // Release length, seconds (the screen's top ‹ › adjusts this in Strings mode).
    void setRelease(float s) { _relS = (s < 0.1f) ? 0.1f : (s > 10.0f ? 10.0f : s); }

    virtual void update();

private:
    float _freq [STR_VOICES] = { 0 };
    float _force[STR_VOICES] = { 0 };
    float _env  [STR_VOICES] = { 0 };
    float _pos  [STR_VOICES][STR_READERS] = {{0}};
    float _drift[STR_VOICES][STR_READERS] = {{0}};
    float _dtgt [STR_VOICES][STR_READERS] = {{0}};
    float _relS = STR_RELEASE_S;
    NoiseGen _rng;
    bool  _seeded = false;
};

inline void AudioStringPad::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    if (!_seeded) { // scatter the reader phases so voices don't comb-filter
        for (uint8_t v = 0; v < STR_VOICES; v++)
            for (uint8_t r = 0; r < STR_READERS; r++)
                _pos[v][r] = (0.5f + 0.5f * _rng.next()) * (float)(STRINGS_LEN - 4);
        _seeded = true;
    }

    float act = 0.0f;
    for (uint8_t v = 0; v < STR_VOICES; v++) {
        if (!isfinite(_env[v])) _env[v] = 0.0f;
        if (_force[v] > act) act = _force[v];
        if (_env[v]   > act) act = _env[v];
    }
    if (act < 1.0e-4f) { memset(out->data, 0, sizeof(out->data)); transmit(out); release(out); return; }

    const float fs = AUDIO_SAMPLE_RATE_EXACT;
    const float up = 1.0f - expf(-1.0f / (STR_ATTACK_S * fs));
    const float dn = 1.0f - expf(-1.0f / (_relS * fs));

    // Block rate: retarget the slow per-reader drift now and then.
    for (uint8_t v = 0; v < STR_VOICES; v++)
        for (uint8_t r = 0; r < STR_READERS; r++) {
            if (((_rng.bits() >> 10) & 0x3FF) == 0)        // occasionally
                _dtgt[v][r] = STR_DRIFT * _rng.next();
            _drift[v][r] += 0.01f * (_dtgt[v][r] - _drift[v][r]);
        }

    const float inv = 1.0f / 32768.0f;
    const float lenF = (float)(STRINGS_LEN);
    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        float mix = 0.0f;
        for (uint8_t v = 0; v < STR_VOICES; v++)
        {
            float tgt = powf(_force[v], STR_FORCE_EXP);
            _env[v] += (tgt > _env[v] ? up : dn) * (tgt - _env[v]);
            float e = _env[v];
            if (e < 1.0e-5f || _freq[v] <= 0.0f) continue;

            float baseRate = (_freq[v] / STRINGS_ROOT_HZ) * (STRINGS_RATE / fs);
            for (uint8_t r = 0; r < STR_READERS; r++)
            {
                float p = _pos[v][r];
                uint32_t i = (uint32_t)p;
                uint32_t i2 = i + 1; if (i2 >= STRINGS_LEN) i2 = 0;
                float fr = p - (float)i;
                float sm = (float)STRINGS_SAMPLE[i] + ((float)STRINGS_SAMPLE[i2] - (float)STRINGS_SAMPLE[i]) * fr;
                mix += sm * inv * e;
                p += baseRate * (1.0f + STR_DETUNE[r] + _drift[v][r]);
                while (p >= lenF) p -= lenF;     // the sample is a seamless loop
                _pos[v][r] = p;
            }
        }
        if (!isfinite(mix)) mix = 0.0f; else if (mix > 2.0f) mix = 2.0f; else if (mix < -2.0f) mix = -2.0f;
        out->data[s] = (int16_t)(mix * 5500.0f);
    }

    transmit(out);
    release(out);
}
