#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"

// ─────────────────────────────────────────────────────────────────────────────
// AudioChimeSwarm — "magical moving chimes" additive swarm.
//
// Each pad owns a little cluster of sine partials whose pitches slowly wander
// (a gentle random walk around chime-like ratios). Moving a hand through the
// proximity field is like brushing a hanging mobile: partials get randomly
// re-struck (Poisson, rate ∝ force) and ring out for seconds, plus a soft
// continuous shimmer floor while you stay near. Pure synthesis — the kind of
// thing only a fast FPU lets you do (72 wandering oscillators).
// ─────────────────────────────────────────────────────────────────────────────
#define SWARM_VOICES   12
#define SWARM_PARTIALS 6

static constexpr float SWARM_RATIO[SWARM_PARTIALS] = { 1.00f, 2.00f, 3.01f, 4.16f, 5.43f, 6.79f };
static constexpr float SWARM_PGAIN[SWARM_PARTIALS] = { 1.00f, 0.62f, 0.40f, 0.30f, 0.22f, 0.16f };
static constexpr float SWARM_DECAY_S   = 1.2f;   // default ring-out (runtime: setDecay / screen ‹ ›)
static constexpr float SWARM_RATE_HZ   = 9.0f;   // strikes/sec per pad at full force
static constexpr float SWARM_DRIFT     = 0.006f; // ±0.6% slow pitch wander
static constexpr float SWARM_HOVER     = 0.16f;  // continuous shimmer floor while touching
static constexpr float SWARM_RISE      = 0.004f; // per-sample attack slew of a strike

class AudioChimeSwarm : public AudioStream {
public:
    AudioChimeSwarm() : AudioStream(0, nullptr) {}

    void setFreq(uint8_t v, float hz) { if (v < SWARM_VOICES && hz > 0.0f) _freq[v] = hz; }
    void setForce(uint8_t v, float f)
    { if (v < SWARM_VOICES) _force[v] = (f < 0.0f) ? 0.0f : (f > 1.0f ? 1.0f : f); }
    // Ring-out length, seconds (the screen's top ‹ › adjusts this in Swarm mode).
    void setDecay(float s) { _decayS = (s < 0.2f) ? 0.2f : (s > 10.0f ? 10.0f : s); }

    virtual void update();

private:
    float _freq [SWARM_VOICES] = { 0 };
    float _force[SWARM_VOICES] = { 0 };
    float _phase[SWARM_VOICES][SWARM_PARTIALS] = {{0}};
    float _fmul [SWARM_VOICES][SWARM_PARTIALS] = {{0}}; // current drift multiplier
    float _ftgt [SWARM_VOICES][SWARM_PARTIALS] = {{0}}; // drift target
    float _amp  [SWARM_VOICES][SWARM_PARTIALS] = {{0}}; // current (slewed) amplitude
    float _atgt [SWARM_VOICES][SWARM_PARTIALS] = {{0}}; // ringing target (decays)
    float _decayS = SWARM_DECAY_S;
    NoiseGen _rng;
    bool  _seeded = false;
};

inline void AudioChimeSwarm::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    if (!_seeded) { // start every partial exactly on its ratio
        for (uint8_t v = 0; v < SWARM_VOICES; v++)
            for (uint8_t p = 0; p < SWARM_PARTIALS; p++)
                { _fmul[v][p] = _ftgt[v][p] = 1.0f; _phase[v][p] = _rng.next() * 0.5f + 0.5f; }
        _seeded = true;
    }

    // Idle check (+ NaN guard: divergences silently reset).
    float act = 0.0f;
    for (uint8_t v = 0; v < SWARM_VOICES; v++) {
        if (_force[v] > act) act = _force[v];
        for (uint8_t p = 0; p < SWARM_PARTIALS; p++) {
            if (!isfinite(_amp[v][p]) || !isfinite(_atgt[v][p])) _amp[v][p] = _atgt[v][p] = 0.0f;
            if (_atgt[v][p] > act) act = _atgt[v][p];
            if (_amp [v][p] > act) act = _amp[v][p];
        }
    }
    if (act < 1.0e-4f) { memset(out->data, 0, sizeof(out->data)); transmit(out); release(out); return; }

    const float fs = AUDIO_SAMPLE_RATE_EXACT;
    const float blockDur = (float)AUDIO_BLOCK_SAMPLES / fs;
    const float decayBlk = expf(-blockDur / _decayS);

    // Block-rate: strikes, drift retargeting, target decay.
    for (uint8_t v = 0; v < SWARM_VOICES; v++)
    {
        float f = _force[v];
        for (uint8_t p = 0; p < SWARM_PARTIALS; p++)
        {
            _atgt[v][p] *= decayBlk;
            // hover shimmer floor
            float floorAmp = f * SWARM_HOVER;
            if (_atgt[v][p] < floorAmp) _atgt[v][p] = floorAmp;
            // random re-strike, more likely (and harder) the closer the hand
            if (f > 0.01f && ((_rng.bits() >> 8) & 0xFFFF) <
                (uint32_t)(f * SWARM_RATE_HZ / SWARM_PARTIALS * blockDur * 65536.0f))
            {
                float hit = f * (0.5f + 0.5f * (0.5f + 0.5f * _rng.next()));
                if (_atgt[v][p] < hit) _atgt[v][p] = hit;
                _ftgt[v][p] = 1.0f + SWARM_DRIFT * _rng.next(); // wander on each strike
            }
            // slow drift toward the wander target
            _fmul[v][p] += 0.02f * (_ftgt[v][p] - _fmul[v][p]);
        }
    }

    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        float mix = 0.0f;
        for (uint8_t v = 0; v < SWARM_VOICES; v++)
        {
            if (_freq[v] <= 0.0f) continue;
            for (uint8_t p = 0; p < SWARM_PARTIALS; p++)
            {
                float a = _amp[v][p];
                float t = _atgt[v][p];
                if (a < 1.0e-5f && t < 1.0e-5f) continue;
                a += SWARM_RISE * (t - a);
                _amp[v][p] = a;
                float ph = _phase[v][p] + _freq[v] * SWARM_RATIO[p] * _fmul[v][p] / fs;
                ph -= (float)(int)ph;
                _phase[v][p] = ph;
                mix += a * SWARM_PGAIN[p] * sinLut01(ph);
            }
        }
        // Soft saturation instead of a hard clamp: brushing all 12 pads compresses
        // gently rather than overdriving.
        if (!isfinite(mix)) mix = 0.0f;
        float o = mix * 0.55f;
        o = o / (1.0f + 0.35f * fabsf(o));
        out->data[s] = (int16_t)(o * 11000.0f);
    }

    transmit(out);
    release(out);
}
