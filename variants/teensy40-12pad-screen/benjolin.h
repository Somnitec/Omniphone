#pragma once
#include <Audio.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// AudioBenjolin — a Benjolin-inspired chaos source (Rob Hordijk).
//
// Two triangle oscillators cross-modulate each other at audio rate. An 8-bit
// shift register — the "rungler" — is clocked on osc B's rising zero-crossings
// and fed the sign of osc A, producing a stepped, semi-random voltage that bends
// osc A's pitch. The output is osc A mixed with the rungler's raw bit, giving the
// classic Benjolin blend of pitched tone and glitchy stepped chaos.
//
// Self-oscillating: it always makes sound while _amp > 0. Parameters are set
// from the main loop (no audio inputs). Optionally the rungler pitch is
// quantised to a scale note table (the "Benjolin" mode) instead of bending
// freely (the "Benjolin Raw" mode).
// ─────────────────────────────────────────────────────────────────────────────
class AudioBenjolin : public AudioStream {
public:
    AudioBenjolin() : AudioStream(0, nullptr) {}

    void setFreqA(float hz)  { _freqA = hz; }
    void setFreqB(float hz)  { _freqB = hz; }
    void setCross(float d)   { _cross = d; }   // cross-mod depth (0..~1)
    void setRungle(float d)  { _rungle = d; }  // rungler→pitch depth (raw mode)
    void setAmp(float a)     { _ampTarget = a; } // output level (0..1), smoothed
    void setCutoff(float c)  { _cut = c < 0 ? 0 : (c > 1 ? 1 : c); } // tone 0=dark..1=open
    // Pass a scale note table to quantise the rungler pitch, or (nullptr,0) for raw.
    void setQuantize(const float* notes, uint8_t n) { _qnotes = notes; _qn = n; }

    virtual void update();

private:
    float   _phaseA = 0.0f, _phaseB = 0.0f;
    float   _freqA = 110.0f, _freqB = 70.0f;
    float   _cross = 0.5f, _rungle = 0.6f;
    float   _amp = 0.0f, _ampTarget = 0.0f;
    float   _cut = 1.0f, _lp = 0.0f; // tone (1-pole low-pass) state
    uint8_t _sr = 0x2D;          // shift register (nonzero seed so it isn't stuck)
    bool    _lastClk = false;
    float   _rungleVal = 0.0f;   // current stepped value 0..1
    const float* _qnotes = nullptr;
    uint8_t _qn = 0;

    static inline float tri(float ph) {            // -1..1 triangle from phase 0..1
        return (ph < 0.5f) ? (4.0f * ph - 1.0f) : (3.0f - 4.0f * ph);
    }
    static inline float wrap01(float p) {
        p -= (float)(int)p; return p < 0.0f ? p + 1.0f : p;
    }
};

inline void AudioBenjolin::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    // Sanitise state so a divergence can never propagate NaN into the mix (which
    // would silence ALL audio via 0×NaN, even at gain 0).
    if (!isfinite(_amp) || !isfinite(_phaseA) || !isfinite(_phaseB) || !isfinite(_rungleVal))
        { _amp = 0; _phaseA = 0; _phaseB = 0; _rungleVal = 0; _sr = 0x2D; }

    // Silent (not the active mode) → emit zeros and skip the per-sample work.
    if (_amp < 0.0003f && _ampTarget < 0.0003f) {
        memset(out->data, 0, sizeof(out->data));
        transmit(out); release(out); return;
    }

    const float fs = AUDIO_SAMPLE_RATE_EXACT;

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
    {
        _amp += (_ampTarget > _amp ? 0.02f : 0.004f) * (_ampTarget - _amp); // fast attack, smooth release

        float a = tri(_phaseA);
        float b = tri(_phaseB);

        // Rungler clock: osc B rising through zero. Shift in sign(A).
        bool clk = (b > 0.0f);
        if (clk && !_lastClk) {
            _sr = (uint8_t)((_sr << 1) | (a > 0.0f ? 1u : 0u));
            _rungleVal = (float)(_sr & 0x3F) / 63.0f; // 6-bit → 0..1
        }
        _lastClk = clk;

        // Osc A pitch: either snapped to a scale note (quantised) or bent freely.
        float fa;
        if (_qnotes && _qn) {
            uint8_t idx = (uint8_t)(_rungleVal * (float)(_qn - 1) + 0.5f);
            fa = _qnotes[idx] * (1.0f + _cross * b);
        } else {
            fa = _freqA * (1.0f + _cross * b) * (1.0f + _rungle * _rungleVal * 3.0f);
        }
        float fb = _freqB * (1.0f + _cross * a);

        _phaseA = wrap01(_phaseA + fa / fs);
        _phaseB = wrap01(_phaseB + fb / fs);

        // Output: osc A blended with the rungler's raw bit (stepped square).
        float bit = (_sr & 1) ? 1.0f : -1.0f;
        float s   = 0.55f * a + 0.45f * bit;
        // Tone: 1-pole low-pass (cut 0 = dark, 1 = open).
        float coef = 0.04f + _cut * 0.9f;
        _lp += coef * (s - _lp);
        float o = _amp * _lp;
        if (!isfinite(o)) o = 0.0f; else if (o > 1.0f) o = 1.0f; else if (o < -1.0f) o = -1.0f;
        out->data[i] = (int16_t)(o * 11000.0f);
    }

    transmit(out);
    release(out);
}
