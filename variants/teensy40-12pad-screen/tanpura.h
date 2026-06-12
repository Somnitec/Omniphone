#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"

// ─────────────────────────────────────────────────────────────────────────────
// AudioTanpura — Indian drone strings with the buzzing "jawari" bridge.
//
// A pad PLUCKS its Karplus–Strong string the moment it reaches full touch
// (the edge detect lives in the main loop; approach velocity sets the pluck
// strength). Four string slots round-robin so quick rolls overlap and ring
// together. The signature jawari shimmer comes from a soft nonlinearity in the
// string feedback (the grazing bridge contact that makes a real tanpura's
// harmonics bloom upward after the pluck). A foundation layer to improvise over.
// ─────────────────────────────────────────────────────────────────────────────
#define TAN_STRINGS  4
#define TAN_LINE_LEN 1024   // supports notes down to ~45 Hz

static constexpr float TAN_LOSS      = 0.9992f; // string decay (closer to 1 = longer ring)
static constexpr float TAN_JAWARI    = 0.30f;   // bridge nonlinearity (the buzz)
static constexpr float TAN_BRIGHT    = 0.14f;   // high-frequency lift at the bridge
static constexpr float TAN_IDLE_S    = 5.0f;    // hard-silence this long after last pluck

class AudioTanpura : public AudioStream {
public:
    AudioTanpura() : AudioStream(0, nullptr) {}

    // Pluck a string NOW (called from the main loop on a full-touch edge).
    void pluck(float hz, float strength)
    {
        if (!(hz > 45.0f)) return;
        const float fs = AUDIO_SAMPLE_RATE_EXACT;
        uint16_t L = (uint16_t)(fs / hz);
        if (L >= TAN_LINE_LEN) L = TAN_LINE_LEN - 1;
        uint8_t s = _slot;
        _slot = (uint8_t)((_slot + 1) % TAN_STRINGS);
        _len[s] = L; _idx[s] = 0;
        if (strength < 0.0f) strength = 0.0f; else if (strength > 1.0f) strength = 1.0f;
        float amp = 0.25f + 0.75f * strength;
        for (uint16_t i = 0; i < L; i++) _line[s][i] = amp * _rng.next();
        _quiet = 0;
    }

    virtual void update();

private:
    float    _line[TAN_STRINGS][TAN_LINE_LEN] = {{0}};
    uint16_t _len [TAN_STRINGS] = { 0 };   // active loop length per string
    uint16_t _idx [TAN_STRINGS] = { 0 };
    uint8_t  _slot = 0;                    // next string slot (round robin)
    uint32_t _quiet  = 0xFFFFFF;           // samples since the last pluck
    float    _hpX = 0.0f, _hpY = 0.0f;     // output DC blocker
    NoiseGen _rng;
};

inline void AudioTanpura::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    const float fs = AUDIO_SAMPLE_RATE_EXACT;
    if (_quiet > (uint32_t)(TAN_IDLE_S * fs)) { // rung out → idle
        memset(out->data, 0, sizeof(out->data));
        transmit(out); release(out); return;
    }
    _quiet += AUDIO_BLOCK_SAMPLES;

    for (int sN = 0; sN < AUDIO_BLOCK_SAMPLES; sN++)
    {
        float mix = 0.0f;
        for (uint8_t s = 0; s < TAN_STRINGS; s++)
        {
            uint16_t L = _len[s];
            if (L < 2) continue;
            uint16_t i  = _idx[s];
            uint16_t i2 = (uint16_t)((i + 1) % L);
            float a = _line[s][i], b = _line[s][i2];
            float y = 0.5f * (a + b) * TAN_LOSS;       // KS average + loss
            y += TAN_BRIGHT * (a - b);                 // bridge brightness lift
            y -= TAN_JAWARI * y * fabsf(y);            // jawari: soft graze nonlinearity
            if (!isfinite(y)) y = 0.0f; else if (y > 1.5f) y = 1.5f; else if (y < -1.5f) y = -1.5f;
            _line[s][i] = y;
            _idx[s] = i2;
            mix += y;
        }
        // DC blocker (the even-order jawari term creates a little DC)
        float o = mix - _hpX + 0.995f * _hpY;
        _hpX = mix; _hpY = o;
        if (!isfinite(o)) { o = 0.0f; _hpX = _hpY = 0.0f; }
        else if (o > 1.5f) o = 1.5f; else if (o < -1.5f) o = -1.5f;
        out->data[sN] = (int16_t)(o * 13000.0f);
    }

    transmit(out);
    release(out);
}
