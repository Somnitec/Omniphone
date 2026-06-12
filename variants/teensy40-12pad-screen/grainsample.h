#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"
#include "sample_data.h"   // HANG_SAMPLE / HANG_LEN / HANG_ROOT_HZ / HANG_RATE

// ─────────────────────────────────────────────────────────────────────────────
// AudioGrainHang — granular synthesis of a real hang-drum strike (samples/).
//
// Per pad, overlapping Hann-windowed grains read the flash sample at a rate
// that transposes the recording's root to the pad's note. Proximity is the
// "bow": while you stay near, grains keep respawning → the percussive sample
// becomes an endlessly sustaining, breathing texture with the authentic timbre
// of the real instrument. Force also scrubs the spawn point: a soft hover
// feeds on the smooth tail, pressing close digs into the attack transient.
//
// Replace samples/hang_<rootHz>.wav + rerun tools/wav2header.py to change the
// source sound entirely (a voice, a gong, water…) — the engine doesn't care.
// ─────────────────────────────────────────────────────────────────────────────
#define GRAIN_VOICES 12
#define GRAIN_SLOTS  3   // overlapping grains per pad

static constexpr float GRAIN_LEN_MS   = 170.0f; // nominal grain length (longer = smoother)
static constexpr float GRAIN_LEN_JIT  = 0.30f;  // ±30% length jitter
static constexpr float GRAIN_DETUNE   = 0.003f; // ±0.3% per-grain detune (chorus shimmer)
static constexpr float GRAIN_FORCE_EXP = 1.4f;  // force→loudness curve
static constexpr float GRAIN_ENV_UP   = 0.30f;  // per-block envelope slew (attack — keeps it snappy)
static constexpr float GRAIN_ENV_DN   = 0.10f;  // per-block envelope slew (release)
static constexpr float GRAIN_OUT_LP   = 0.38f;  // output low-pass — rounds the texture off

class AudioGrainHang : public AudioStream {
public:
    AudioGrainHang() : AudioStream(0, nullptr) {}

    void setFreq(uint8_t v, float hz) { if (v < GRAIN_VOICES && hz > 0.0f) _freq[v] = hz; }
    void setForce(uint8_t v, float f)
    { if (v < GRAIN_VOICES) _force[v] = (f < 0.0f) ? 0.0f : (f > 1.0f ? 1.0f : f); }

    virtual void update();

private:
    struct Grain {
        float    pos = 0, rate = 0, amp = 0;
        uint32_t age = 0, len = 0;
        bool     on = false;
    };
    float   _freq [GRAIN_VOICES] = { 0 };
    float   _force[GRAIN_VOICES] = { 0 };
    float   _env  [GRAIN_VOICES] = { 0 }; // live loudness (tracks force NOW, not at spawn)
    float   _lpOut = 0.0f;
    Grain   _g[GRAIN_VOICES][GRAIN_SLOTS];
    int32_t _spawnIn[GRAIN_VOICES] = { 0 }; // samples until next spawn
    NoiseGen _rng;

    void spawn(uint8_t v)
    {
        for (uint8_t k = 0; k < GRAIN_SLOTS; k++)
        {
            Grain& g = _g[v][k];
            if (g.on) continue;
            float f   = _force[v];
            float fs  = AUDIO_SAMPLE_RATE_EXACT;
            g.len  = (uint32_t)(GRAIN_LEN_MS * 0.001f * fs *
                                (1.0f + GRAIN_LEN_JIT * _rng.next()));
            if (g.len < 256) g.len = 256;
            g.rate = (_freq[v] / HANG_ROOT_HZ) * (HANG_RATE / AUDIO_SAMPLE_RATE_EXACT)
                     * (1.0f + GRAIN_DETUNE * _rng.next());
            // Spawn point: soft hover feeds on the tail, hard press digs into the
            // attack. Centre ± jitter, clamped so the grain fits the sample.
            float centre = (0.06f + (1.0f - f) * 0.45f) * (float)HANG_LEN;
            float jit    = 0.10f * (float)HANG_LEN * _rng.next();
            float maxPos = (float)HANG_LEN - (float)g.len * g.rate - 4.0f;
            float p      = centre + jit;
            if (p < 0.0f) p = 0.0f; else if (p > maxPos) p = maxPos;
            g.pos  = p;
            g.amp  = 0.85f + 0.15f * _rng.next(); // loudness is live (_env), not frozen at spawn
            g.age  = 0;
            g.on   = true;
            _spawnIn[v] = (int32_t)(g.len / 2);   // 50% overlap cadence
            return;
        }
    }
};

inline void AudioGrainHang::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    // Idle check: silent and nothing held → skip all work.
    bool busy = false;
    for (uint8_t v = 0; v < GRAIN_VOICES && !busy; v++) {
        if (_force[v] > 0.01f) busy = true;
        for (uint8_t k = 0; k < GRAIN_SLOTS; k++) if (_g[v][k].on) busy = true;
    }
    if (!busy) { memset(out->data, 0, sizeof(out->data)); transmit(out); release(out); return; }

    // Spawn scheduling + live loudness envelope (block rate is plenty: the env
    // tracking force NOW — not frozen at grain spawn — is what makes the mode
    // respond as fast as the LEDs).
    for (uint8_t v = 0; v < GRAIN_VOICES; v++)
    {
        if (!isfinite(_env[v])) _env[v] = 0.0f;
        float tgt = powf(_force[v], GRAIN_FORCE_EXP);
        _env[v] += (tgt > _env[v] ? GRAIN_ENV_UP : GRAIN_ENV_DN) * (tgt - _env[v]);
        _spawnIn[v] -= AUDIO_BLOCK_SAMPLES;
        if (_force[v] > 0.02f && _spawnIn[v] <= 0) spawn(v);
    }

    const float inv = 1.0f / 32768.0f;
    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        float mix = 0.0f;
        for (uint8_t v = 0; v < GRAIN_VOICES; v++)
        {
            float vsum = 0.0f;
            for (uint8_t k = 0; k < GRAIN_SLOTS; k++)
            {
                Grain& g = _g[v][k];
                if (!g.on) continue;
                uint32_t i = (uint32_t)g.pos;
                if (i + 1 >= HANG_LEN || g.age >= g.len) { g.on = false; continue; }
                float fr = g.pos - (float)i;
                float sm = (float)HANG_SAMPLE[i] + ((float)HANG_SAMPLE[i + 1] - (float)HANG_SAMPLE[i]) * fr;
                // Hann window: sin²(π·age/len)
                float w = sinLut01(0.5f * (float)g.age / (float)g.len);
                vsum += sm * inv * w * w * g.amp;
                g.pos += g.rate;
                g.age++;
            }
            mix += vsum * _env[v];
        }
        // Gentle low-pass rounds off grain edges → softer, smoother texture.
        _lpOut += GRAIN_OUT_LP * (mix - _lpOut);
        float o = _lpOut;
        if (!isfinite(o)) { o = 0.0f; _lpOut = 0.0f; }
        else if (o > 1.5f) o = 1.5f; else if (o < -1.5f) o = -1.5f;
        out->data[s] = (int16_t)(o * 12000.0f);
    }

    transmit(out);
    release(out);
}
