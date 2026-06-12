#pragma once
#include <Audio.h>
#include <math.h>
#include "dsp_util.h"

// ─────────────────────────────────────────────────────────────────────────────
// AudioBowedString — bowed-string digital waveguide (cello / viol family).
//
// The McIntyre–Woodhouse / STK "Bowed" model: each pad is a string split at the
// bow point into two delay lines (bow→bridge, bow→nut). Every sample the bow's
// velocity is compared with the string's; a friction curve decides stick or
// slip, and the resulting force wave is injected both ways. That stick-slip
// cycle IS the bowed-string sound — no oscillator anywhere. Proximity = bow
// velocity AND pressure: hover for a whispery sul-tasto haze, lean in for a
// full singing arco. 12 independent strings → bowed chords.
// ─────────────────────────────────────────────────────────────────────────────
#define CELLO_VOICES     12
#define CELLO_BRIDGE_LEN 256
#define CELLO_NECK_LEN   768

static constexpr float CELLO_BOW_POS    = 0.252f;  // bow position along the string
static constexpr float CELLO_VEL_MIN    = 0.03f;   // bow velocity at zero force
static constexpr float CELLO_VEL_MAX    = 0.22f;   // …at full force
static constexpr float CELLO_PRESS_MIN  = 0.35f;   // bow pressure range (force-mapped)
static constexpr float CELLO_PRESS_MAX  = 0.92f;
static constexpr float CELLO_STRING_LP  = 0.55f;   // bridge-side loop low-pass (darker = warmer)
static constexpr float CELLO_BRIDGE_REFL = 0.985f; // bridge reflection (higher = string rings on after the bow lifts)
static constexpr float CELLO_ATTACK_S   = 0.18f;   // slow bow start — no sudden first impulse
static constexpr float CELLO_RELEASE_S  = 0.45f;
static constexpr float CELLO_BODY_LP    = 0.30f;   // output body warmth

class AudioBowedString : public AudioStream {
public:
    AudioBowedString() : AudioStream(0, nullptr)
    {
        for (uint8_t v = 0; v < CELLO_VOICES; v++) {
            _bridge[v].init(_bridgeBuf[v], CELLO_BRIDGE_LEN);
            _neck  [v].init(_neckBuf  [v], CELLO_NECK_LEN);
        }
    }

    void setFreq(uint8_t v, float hz)
    {
        if (v >= CELLO_VOICES || !(hz > 45.0f)) return;
        float L = AUDIO_SAMPLE_RATE_EXACT / hz - 4.0f;
        float b = L * CELLO_BOW_POS, n = L - b;
        if (b > CELLO_BRIDGE_LEN - 2) b = CELLO_BRIDGE_LEN - 2;
        if (n > CELLO_NECK_LEN   - 2) n = CELLO_NECK_LEN   - 2;
        if (b < 2.0f) b = 2.0f;
        if (n < 2.0f) n = 2.0f;
        _bridgeD[v] = b; _neckD[v] = n;
    }

    void setForce(uint8_t v, float f)
    { if (v < CELLO_VOICES) _force[v] = (f < 0.0f) ? 0.0f : (f > 1.0f ? 1.0f : f); }

    virtual void update();

private:
    float     _bridgeBuf[CELLO_VOICES][CELLO_BRIDGE_LEN];
    float     _neckBuf  [CELLO_VOICES][CELLO_NECK_LEN];
    DelayLine _bridge[CELLO_VOICES], _neck[CELLO_VOICES];
    float     _bridgeD[CELLO_VOICES] = { 0 }, _neckD[CELLO_VOICES] = { 0 };
    float     _force[CELLO_VOICES] = { 0 };
    float     _env  [CELLO_VOICES] = { 0 };
    float     _lp   [CELLO_VOICES] = { 0 };  // bridge-side string filter state
    float     _bodyLp = 0.0f;
    float     _energy = 0.0f;                // smoothed output level (for idling)

    // STK BowTable friction: reflection coefficient from the velocity differential.
    static inline float bowTable(float dv, float pressure)
    {
        float slope = 5.0f - 4.0f * pressure;          // harder press = wider stick
        float x = fabsf(dv * slope) + 0.75f;
        float r = 1.0f / (x * x * x * x);              // x^-4
        return (r > 1.0f) ? 1.0f : r;
    }
};

inline void AudioBowedString::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    float act = _energy;
    for (uint8_t v = 0; v < CELLO_VOICES; v++) {
        if (!isfinite(_env[v]) || !isfinite(_lp[v])) {
            _env[v] = _lp[v] = 0.0f; _bridge[v].clear(); _neck[v].clear();
        }
        if (_force[v] > act) act = _force[v];
    }
    if (act < 1.0e-4f) { memset(out->data, 0, sizeof(out->data)); transmit(out); release(out); return; }

    const float fs = AUDIO_SAMPLE_RATE_EXACT;
    const float up = 1.0f - expf(-1.0f / (CELLO_ATTACK_S  * fs));
    const float dn = 1.0f - expf(-1.0f / (CELLO_RELEASE_S * fs));

    float peak = 0.0f;
    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        float mix = 0.0f;
        for (uint8_t v = 0; v < CELLO_VOICES; v++)
        {
            _env[v] += (_force[v] > _env[v] ? up : dn) * (_force[v] - _env[v]);
            float e = _env[v];
            if (_bridgeD[v] < 2.0f) continue;
            // NOTE: the string keeps being processed even at e≈0 so it can ring
            // out and FADE after the bow lifts (the abrupt cut-off was this skip).

            float bridgeOut = _bridge[v].readF(_bridgeD[v]);
            float neckOut   = _neck  [v].readF(_neckD[v]);

            // reflections: lossy low-passed bridge, rigid nut
            _lp[v] += CELLO_STRING_LP * (bridgeOut - _lp[v]);
            float bridgeRefl = -CELLO_BRIDGE_REFL * _lp[v];
            float nutRefl    = -neckOut;

            // bow junction: stick-slip friction injection. Bow velocity scales
            // smoothly from zero with the envelope — no sudden first impulse.
            float bowVel   = e * (CELLO_VEL_MIN + (CELLO_VEL_MAX - CELLO_VEL_MIN) * e);
            float pressure = CELLO_PRESS_MIN + (CELLO_PRESS_MAX - CELLO_PRESS_MIN) * e;
            float dv  = bowVel - (bridgeRefl + nutRefl);
            float inj = dv * bowTable(dv, pressure);

            _neck  [v].push(bridgeRefl + inj);
            _bridge[v].push(nutRefl    + inj);

            mix += bridgeOut;
        }
        // body warmth: gentle low-pass blended with the raw string
        _bodyLp += CELLO_BODY_LP * (mix - _bodyLp);
        float o = 0.45f * mix + 0.55f * _bodyLp;
        if (!isfinite(o)) o = 0.0f; else if (o > 2.0f) o = 2.0f; else if (o < -2.0f) o = -2.0f;
        float ab = fabsf(o); if (ab > peak) peak = ab;
        out->data[s] = (int16_t)(o * 9000.0f);
    }
    _energy = peak;

    transmit(out);
    release(out);
}
