#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"

// ─────────────────────────────────────────────────────────────────────────────
// AudioSingingBowls — rubbed singing bowls / glass harmonica.
//
// Same modal family as hangbow.h but a different instrument: each pad is a bowl
// whose partials come in SLIGHTLY DETUNED PAIRS — the two members beat against
// each other at ~0.3–1 Hz, which is the hypnotic "wah-wah-wah" shimmer of a real
// rubbed bowl. T60s are very long (up to ~11 s), the friction exciter is darker
// and gentler than the hang bow, and there's no over-pressure squeal — pressing
// harder just makes the bowl sing louder and rounder. A small sympathetic
// cross-feed lets a loud bowl gently wake its harmonic neighbours.
// ─────────────────────────────────────────────────────────────────────────────
#define BOWL_VOICES 12
#define BOWL_MODES  6

// Detuned pairs: (1.0, 1.0035) (2.74, 2.748) (4.95, 4.97) — bowl-like ratios.
static constexpr float BOWL_RATIO[BOWL_MODES] = { 1.000f, 1.0035f, 2.740f, 2.748f, 4.950f, 4.970f };
static constexpr float BOWL_MGAIN[BOWL_MODES] = { 1.00f,  0.95f,   0.42f,  0.40f,  0.16f,  0.15f };
static constexpr float BOWL_T60  [BOWL_MODES] = { 11.0f,  11.0f,   7.0f,   7.0f,   4.0f,   4.0f };

static constexpr float BOWL_FORCE_SMOOTH = 0.0025f; // slow force slew — bowls take time to sing
static constexpr float BOWL_FORCE_EXP    = 1.6f;
static constexpr float BOWL_LP_MIN       = 0.005f;  // very dark friction noise
static constexpr float BOWL_LP_MAX       = 0.024f;  // (lowered: hiss complaints)
static constexpr float BOWL_GAIN         = 1.70f;   // compensates the darker bow
static constexpr float BOWL_CROSS_GAIN   = 0.16f;   // gentle sympathetic bleed
static constexpr float BOWL_PITCH_AMT    = 0.80f;
static constexpr float BOWL_NEIGH_AMT    = 0.20f;
static constexpr float BOWL_COUPLE_WIDTH = 35.0f;   // cents
static constexpr float BOWL_OUT_LP       = 0.30f;

class AudioSingingBowls : public AudioStream {
public:
    AudioSingingBowls() : AudioStream(0, nullptr) {}

    void setFreq(uint8_t v, float hz)
    {
        if (v >= BOWL_VOICES || !(hz > 0.0f)) return;
        _freq[v] = hz;
        const float fs = AUDIO_SAMPLE_RATE_EXACT;
        for (uint8_t m = 0; m < BOWL_MODES; m++) {
            float fm = hz * BOWL_RATIO[m];
            if (fm > fs * 0.45f) fm = fs * 0.45f;
            float r = expf(-6.9078f / (BOWL_T60[m] * fs));
            float w = 2.0f * (float)M_PI * fm / fs;
            _a1[v][m]   = 2.0f * r * cosf(w);
            _r2[v][m]   = r * r;
            _norm[v][m] = (1.0f - r) * BOWL_MGAIN[m];
        }
    }

    void setForce(uint8_t v, float f)
    {
        if (v >= BOWL_VOICES) return;
        if (f < 0.0f) f = 0.0f; else if (f > 1.0f) f = 1.0f;
        _force[v] = powf(f, BOWL_FORCE_EXP);
    }

    void recalcCoupling()
    {
        for (uint8_t i = 0; i < BOWL_VOICES; i++)
            for (uint8_t j = 0; j < BOWL_VOICES; j++) {
                if (i == j) { _coup[i][j] = 0.0f; continue; }
                float pitchW = overlap(_freq[i], _freq[j]);
                int   d = (i > j) ? (i - j) : (j - i);
                float neighW = (d == 1) ? 1.0f : (d == 2 ? 0.35f : 0.0f);
                _coup[i][j] = BOWL_PITCH_AMT * pitchW + BOWL_NEIGH_AMT * neighW;
            }
    }

    virtual void update();

private:
    float _freq  [BOWL_VOICES] = { 0 };
    float _force [BOWL_VOICES] = { 0 };
    float _forceS[BOWL_VOICES] = { 0 };
    float _y1 [BOWL_VOICES][BOWL_MODES] = {{0}};
    float _y2 [BOWL_VOICES][BOWL_MODES] = {{0}};
    float _a1 [BOWL_VOICES][BOWL_MODES] = {{0}};
    float _r2 [BOWL_VOICES][BOWL_MODES] = {{0}};
    float _norm[BOWL_VOICES][BOWL_MODES] = {{0}};
    float _vout[BOWL_VOICES] = { 0 };
    float _lp  [BOWL_VOICES] = { 0 };
    float _coup[BOWL_VOICES][BOWL_VOICES] = {{0}};
    float _outLp = 0.0f;
    NoiseGen _rng;

    static float overlap(float fi, float fj)
    {
        if (!(fi > 0.0f) || !(fj > 0.0f)) return 0.0f;
        float best = 0.0f;
        for (int a = 1; a <= 4; a++)
            for (int b = 1; b <= 4; b++) {
                float cents = 1731.234f * logf((a * fi) / (b * fj));
                float w = expf(-(cents * cents) / (2.0f * BOWL_COUPLE_WIDTH * BOWL_COUPLE_WIDTH));
                float s = w / (float)(a * b);
                if (s > best) best = s;
            }
        return best;
    }
};

inline void AudioSingingBowls::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    float act = 0.0f;
    for (uint8_t v = 0; v < BOWL_VOICES; v++) {
        if (!isfinite(_vout[v]) || !isfinite(_forceS[v])) {
            _vout[v] = _forceS[v] = _lp[v] = 0.0f;
            for (uint8_t m = 0; m < BOWL_MODES; m++) _y1[v][m] = _y2[v][m] = 0.0f;
        }
        float a = fabsf(_vout[v]); if (a > act) act = a;
        if (_force[v] > act) act = _force[v];
    }
    if (act < 1.0e-4f) { memset(out->data, 0, sizeof(out->data)); transmit(out); release(out); return; }

    float nv[BOWL_VOICES];
    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        float mix = 0.0f;
        for (uint8_t v = 0; v < BOWL_VOICES; v++)
        {
            float tgt = _force[v];
            _forceS[v] += (tgt > _forceS[v] ? 0.004f : BOWL_FORCE_SMOOTH) * (tgt - _forceS[v]);
            float fs_ = _forceS[v];

            float n = _rng.next();
            _lp[v] += (BOWL_LP_MIN + BOWL_LP_MAX * fs_) * (n - _lp[v]);
            float exc = fs_ * _lp[v] * BOWL_GAIN;

            float cross = 0.0f;
            const float* cr = _coup[v];
            for (uint8_t j = 0; j < BOWL_VOICES; j++) cross += cr[j] * _vout[j];
            exc += cross * BOWL_CROSS_GAIN;

            float vsum = 0.0f;
            for (uint8_t m = 0; m < BOWL_MODES; m++) {
                float y = exc + _a1[v][m] * _y1[v][m] - _r2[v][m] * _y2[v][m];
                _y2[v][m] = _y1[v][m];
                _y1[v][m] = y;
                vsum += _norm[v][m] * y;
            }
            if (!isfinite(vsum)) vsum = 0.0f;
            else if (vsum > 4.0f) vsum = 4.0f; else if (vsum < -4.0f) vsum = -4.0f;
            nv[v] = vsum;
            mix  += vsum;
        }
        for (uint8_t v = 0; v < BOWL_VOICES; v++) _vout[v] = nv[v];

        _outLp += BOWL_OUT_LP * (mix - _outLp);
        float o = _outLp;
        if (!isfinite(o)) o = 0.0f; else if (o > 1.0f) o = 1.0f; else if (o < -1.0f) o = -1.0f;
        out->data[s] = (int16_t)(o * 20000.0f);
    }

    transmit(out);
    release(out);
}
