#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"

// ─────────────────────────────────────────────────────────────────────────────
// AudioFreezeSmear — click-free spectral-style freeze for the Freeze mode.
//
// Replaces AudioEffectGranular's freeze, which loops one raw grain with no
// window → an audible tick at every wrap. This engine continuously records the
// bus into a circular buffer; on freeze() it stops recording and plays a cloud
// of overlapping Hann-windowed grains from random positions in the captured
// audio, each with a hair of detune. The window overlap means there is never a
// discontinuity, and the random positions smear the capture into a soft,
// stationary "held in the air" pad rather than an obvious loop.
//
// Output is ONLY the frozen layer (silent while just recording); the dry path
// stays on fxMix ch0 as usual.
// ─────────────────────────────────────────────────────────────────────────────
#define FRZ_CAP    16384  // capture buffer, samples (~371 ms)
#define FRZ_GRAINS 3      // simultaneous windowed grains

static constexpr float FRZ_GRAIN_S  = 0.16f;   // grain length, seconds
static constexpr float FRZ_DETUNE   = 0.004f;  // ±0.4% per-grain rate wobble (shimmer)

class AudioFreezeSmear : public AudioStream {
public:
    AudioFreezeSmear() : AudioStream(1, _inq) {}

    void freeze(bool on)
    {
        if (on && !_frozen)
            for (uint8_t g = 0; g < FRZ_GRAINS; g++) {   // stagger the cloud so the
                respawn(_g[g]);                          // windows overlap from the start
                uint32_t adv = (uint32_t)((float)g / FRZ_GRAINS * (float)_g[g].len);
                _g[g].age = adv;
                _g[g].pos += (float)adv * _g[g].rate;
                if (_g[g].pos >= (float)FRZ_CAP) _g[g].pos -= (float)FRZ_CAP;
            }
        _frozen = on;
    }
    bool frozen() const { return _frozen; }

    virtual void update();

private:
    struct Grain { float pos = 0, rate = 1; uint32_t age = 0, len = 0; bool on = false; };
    audio_block_t* _inq[1];
    int16_t  _cap[FRZ_CAP] = { 0 };
    uint32_t _w = 0;          // write head
    bool     _frozen = false, _filled = false;
    Grain    _g[FRZ_GRAINS];
    NoiseGen _rng;

    void respawn(Grain& g)
    {
        g.len  = (uint32_t)(FRZ_GRAIN_S * AUDIO_SAMPLE_RATE_EXACT * (0.85f + 0.3f * (0.5f + 0.5f * _rng.next())));
        g.rate = 1.0f + FRZ_DETUNE * _rng.next();
        // Start anywhere that lets the whole grain fit without crossing the
        // write head (the seam between newest and oldest audio).
        float span = (float)FRZ_CAP - (float)g.len * g.rate - 4.0f;
        float off  = (0.5f + 0.5f * _rng.next()) * span;
        g.pos = (float)_w + 1.0f + off;
        if (g.pos >= (float)FRZ_CAP) g.pos -= (float)FRZ_CAP;
        g.age = 0;
        g.on  = true;
    }
};

inline void AudioFreezeSmear::update()
{
    audio_block_t* in = receiveReadOnly(0);

    if (!_frozen)
    {
        // Record the bus into the ring; output silence (dry is on fxMix ch0).
        if (in) {
            for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++) {
                _cap[_w] = in->data[s];
                if (++_w >= FRZ_CAP) { _w = 0; _filled = true; }
            }
            release(in);
        }
        return; // no output block → mixer reads silence
    }
    if (in) release(in); // frozen: ignore (and free) the live input

    audio_block_t* out = allocate();
    if (!out) return;
    if (!_filled && _w < 4096) { // froze before anything was captured
        memset(out->data, 0, sizeof(out->data));
        transmit(out); release(out); return;
    }

    const float inv = 1.0f / 32768.0f;
    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        float mix = 0.0f;
        for (uint8_t k = 0; k < FRZ_GRAINS; k++)
        {
            Grain& g = _g[k];
            if (!g.on || g.age >= g.len) { respawn(g); }
            uint32_t i  = (uint32_t)g.pos;
            uint32_t i2 = (i + 1 >= FRZ_CAP) ? 0 : i + 1;
            float fr = g.pos - (float)i;
            float sm = (float)_cap[i] + ((float)_cap[i2] - (float)_cap[i]) * fr;
            float w  = sinLut01(0.5f * (float)g.age / (float)g.len); // Hann
            mix += sm * inv * w * w;
            g.pos += g.rate;
            if (g.pos >= (float)FRZ_CAP) g.pos -= (float)FRZ_CAP;
            g.age++;
        }
        mix *= 0.9f; // 3 half-overlapped Hann² grains ≈ unity; trim a touch
        if (!isfinite(mix)) mix = 0.0f; else if (mix > 1.0f) mix = 1.0f; else if (mix < -1.0f) mix = -1.0f;
        out->data[s] = (int16_t)(mix * 32000.0f);
    }

    transmit(out);
    release(out);
}
