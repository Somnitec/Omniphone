#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"

// ─────────────────────────────────────────────────────────────────────────────
// AudioPhaseChimes — generative phasing mallet loops (Steve Reich style).
//
// The mallet strikes at the EXACT moment a pad reaches full touch (edge detect
// in the main loop), and the pattern's repetition speed is set by how fast the
// hand approached — dive in for fast ostinati, settle in slowly for sparse
// tolling. Each layer keeps looping (its four-note pattern built from the pad's
// scale degrees) while held, then lingers and fades after release. Different
// pads at different speeds slide in and out of phase, Reich-style.
// ─────────────────────────────────────────────────────────────────────────────
#define PC_VOICES 12
#define PC_POLY   2     // overlapping ring voices per pad
#define PC_STEPS  4

static constexpr uint8_t PC_DEG[PC_STEPS] = { 0, 2, 4, 7 };  // scale-degree offsets
static constexpr float PC_BASE_PERIOD_S = 0.50f;   // default step period (overridden per trigger)
static constexpr float PC_DECAY_S       = 1.4f;    // mallet ring-out
static constexpr float PC_LAYER_FALL    = 0.00001f;// ≈2.3 s release — layers linger
static constexpr float PC_OCTAVE        = 2.0f;    // chime register (×2 = up an octave)

class AudioPhaseChimes : public AudioStream {
public:
    AudioPhaseChimes() : AudioStream(0, nullptr) {}

    void setScale(const float* freqs, uint8_t n) { _scale = freqs; _n = n; }
    void setForce(uint8_t v, float f)
    { if (v < PC_VOICES) _force[v] = (f < 0.0f) ? 0.0f : (f > 1.0f ? 1.0f : f); }

    // Full-touch edge: hit the mallet NOW and (re)set this layer's repetition
    // speed (periodS comes from the approach velocity in the main loop).
    void trigger(uint8_t v, float periodS)
    {
        if (v >= PC_VOICES) return;
        if (periodS < 0.10f) periodS = 0.10f; else if (periodS > 2.0f) periodS = 2.0f;
        _periodS[v] = periodS;
        _layer[v]   = 1.0f;
        _step[v]    = 0;
        _stepIn[v]  = 0;   // strike on the next block (≈3 ms)
    }

    virtual void update();

private:
    struct Ring { float ph1 = 0, ph2 = 0, f = 0, amp = 0; };
    const float* _scale = nullptr;
    uint8_t _n = 0;
    float   _force[PC_VOICES] = { 0 };
    float   _layer[PC_VOICES] = { 0 };
    float   _periodS[PC_VOICES] = { 0 };
    Ring    _ring [PC_VOICES][PC_POLY];
    uint8_t _next [PC_VOICES] = { 0 };  // ring-voice round robin
    uint8_t _step [PC_VOICES] = { 0 };  // pattern position
    int32_t _stepIn[PC_VOICES] = { 0 }; // samples to next step
};

inline void AudioPhaseChimes::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    float act = 0.0f;
    for (uint8_t v = 0; v < PC_VOICES; v++) {
        if (!isfinite(_layer[v])) _layer[v] = 0.0f;
        if (_force[v] > act) act = _force[v];
        if (_layer[v] > act) act = _layer[v];
        for (uint8_t k = 0; k < PC_POLY; k++) {
            if (!isfinite(_ring[v][k].amp)) _ring[v][k].amp = 0.0f;
            if (_ring[v][k].amp > act) act = _ring[v][k].amp;
        }
    }
    if (act < 1.0e-4f || !_scale || _n == 0)
    { memset(out->data, 0, sizeof(out->data)); transmit(out); release(out); return; }

    const float fs = AUDIO_SAMPLE_RATE_EXACT;
    const float decay = expf(-(float)AUDIO_BLOCK_SAMPLES / (PC_DECAY_S * fs));

    // Block rate: step the patterns and strike mallet hits.
    for (uint8_t v = 0; v < PC_VOICES; v++)
    {
        for (uint8_t k = 0; k < PC_POLY; k++) _ring[v][k].amp *= decay;
        if (_periodS[v] <= 0.0f) _periodS[v] = PC_BASE_PERIOD_S;
        _stepIn[v] -= AUDIO_BLOCK_SAMPLES;
        if (_stepIn[v] > 0) continue;
        _stepIn[v] += (int32_t)(_periodS[v] * fs);
        if (_layer[v] < 0.02f) { _step[v] = 0; continue; } // silent layer → hold pattern at start
        uint8_t st = _step[v];
        _step[v] = (uint8_t)((st + 1) % PC_STEPS);
        Ring& r = _ring[v][_next[v]];
        _next[v] = (uint8_t)((_next[v] + 1) % PC_POLY);
        r.f   = _scale[(v + PC_DEG[st]) % _n] * PC_OCTAVE;
        r.amp = _layer[v] * (st == 0 ? 1.0f : 0.78f);     // accent the downbeat
        r.ph1 = r.ph2 = 0.0f;
    }

    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        float mix = 0.0f;
        for (uint8_t v = 0; v < PC_VOICES; v++)
        {
            // Layer envelope: a trigger() sets it to 1; while the pad stays held
            // it holds, after release it lingers for seconds. Hovering without a
            // full touch never starts a layer (the trigger is the only way in).
            float tgt = (_force[v] > 0.5f && _layer[v] > 0.3f) ? 1.0f : 0.0f;
            if (tgt < _layer[v]) _layer[v] += PC_LAYER_FALL * (tgt - _layer[v]);
            for (uint8_t k = 0; k < PC_POLY; k++)
            {
                Ring& r = _ring[v][k];
                if (r.amp < 1.0e-5f || r.f <= 0.0f) continue;
                r.ph1 += r.f / fs;          r.ph1 -= (float)(int)r.ph1;
                r.ph2 += r.f * 2.76f / fs;  r.ph2 -= (float)(int)r.ph2;
                mix += r.amp * (sinLut01(r.ph1) + 0.22f * sinLut01(r.ph2)); // mallet + shimmer partial
            }
        }
        if (!isfinite(mix)) mix = 0.0f; else if (mix > 2.0f) mix = 2.0f; else if (mix < -2.0f) mix = -2.0f;
        out->data[s] = (int16_t)(mix * 2200.0f);
    }

    transmit(out);
    release(out);
}
