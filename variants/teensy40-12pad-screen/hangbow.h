#pragma once
#include <Audio.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// AudioHangBow — a bowed handpan / hang drum modal physical model.
//
// Each of the 12 pads is a little resonator: a bank of HANG_MODES high-Q two-pole
// resonators tuned to a handpan's partials (fundamental : octave : twelfth = 1:2:3).
// Instead of being struck by an impulse, every resonator is BOWED — driven by a
// friction-noise exciter whose force comes from proximity (closer = harder bow).
// Soft bowing = a pure singing tone; pressing past OVER_THRESH the bow "over-
// presses" into a scratchy, squealing multiphonic (a nonlinear feedback term).
//
// ALL 12 resonators run all the time, even un-bowed ones. Every sample the voices
// cross-feed each other through a coupling matrix (_coup), so a loud bowed tone
// pours energy into its neighbours; because each resonator is sharply tuned it
// only "catches" energy near its own partials → real sympathetic resonance. The
// coupling blends harmonic overlap (octaves/fifths ring most) with physical pad
// adjacency, so an un-touched neighbour can bloom on its own.
//
// Self-contained like benjolin.h / cracklebox.h: no audio inputs, all parameters
// pushed from the main loop via setFreq()/setForce(). NaN-guarded and clamped so a
// divergence (the cross-feedback is a real feedback loop) can never wedge the mix.
// Tune by ear with the constants just below.
// ─────────────────────────────────────────────────────────────────────────────
#define HANG_VOICES 12
#define HANG_MODES   3

// ── Tuning ───────────────────────────────────────────────────────────────────
// Handpan partial ratios and their relative loudness + ring time (T60, seconds).
static constexpr float HANG_RATIO[HANG_MODES] = { 1.00f, 2.00f, 3.00f };
static constexpr float HANG_MGAIN[HANG_MODES] = { 1.00f, 0.55f, 0.30f };
static constexpr float HANG_T60  [HANG_MODES] = { 2.20f, 1.40f, 0.90f }; // higher = longer ring/sharper Q (cleaner tone, less hiss)

static constexpr float HANG_FORCE_SMOOTH = 0.006f; // per-sample slew of bow force (lower = smoother)
static constexpr float HANG_FORCE_EXP    = 2.2f;   // >1 = steep/responsive force curve (closer bites harder)
static constexpr float HANG_BOW_LP_MIN   = 0.012f; // bow-noise brightness when barely bowing (dark)
static constexpr float HANG_BOW_LP_MAX   = 0.090f; // extra brightness at full force (lower = less hiss)
static constexpr float HANG_BOW_GAIN     = 1.05f;  // overall bow excitation level
static constexpr float HANG_OUT_LP       = 0.42f;  // output 1-pole low-pass (lower = darker, tames residual hiss)

static constexpr float HANG_OVER_THRESH  = 0.74f;  // force above this starts to over-press
static constexpr float HANG_OVER_FB       = 6.0f;  // nonlinear feedback drive (squeal sharpness)
static constexpr float HANG_OVER_GAIN     = 0.55f; // how loud the scratch/multiphonic gets

static constexpr float HANG_CROSS_GAIN   = 0.50f;  // sympathetic bleed strength (keep <1 for stability)
static constexpr float HANG_PITCH_AMT    = 0.65f;  // weight of harmonic-overlap coupling
static constexpr float HANG_NEIGH_AMT    = 0.35f;  // weight of physical pad-adjacency coupling
static constexpr float HANG_COUPLE_WIDTH = 45.0f;  // cents window for "shared partial" (wider = more bleed)

class AudioHangBow : public AudioStream {
public:
    AudioHangBow() : AudioStream(0, nullptr) { recalcCoupling(); }

    // Note for a pad (Hz). Cheap — just this voice's resonator coeffs, safe to call
    // every frame for live pitch/glide. The (heavier) coupling matrix is recomputed
    // separately via recalcCoupling() only when the note set actually changes.
    void setFreq(uint8_t v, float hz)
    {
        if (v >= HANG_VOICES || !(hz > 0.0f)) return;
        _freq[v] = hz;
        const float fs = AUDIO_SAMPLE_RATE_EXACT;
        for (uint8_t m = 0; m < HANG_MODES; m++) {
            float fm = hz * HANG_RATIO[m];
            if (fm > fs * 0.45f) fm = fs * 0.45f;
            float r = expf(-6.9078f / (HANG_T60[m] * fs));      // pole radius for the T60
            float w = 2.0f * (float)M_PI * fm / fs;
            _a1[v][m]   = 2.0f * r * cosf(w);
            _r2[v][m]   = r * r;
            _norm[v][m] = (1.0f - r) * HANG_MGAIN[m];           // normalise so Q doesn't change loudness
        }
    }

    // Rebuild the sympathetic-coupling matrix from the current notes (harmonic
    // overlap blended with pad adjacency). Call after the notes settle.
    void recalcCoupling()
    {
        for (uint8_t i = 0; i < HANG_VOICES; i++)
            for (uint8_t j = 0; j < HANG_VOICES; j++) {
                if (i == j) { _coup[i][j] = 0.0f; continue; }
                float pitchW = harmonicOverlap(_freq[i], _freq[j]);
                int   d = (i > j) ? (i - j) : (j - i);
                float neighW = (d == 1) ? 1.0f : (d == 2 ? 0.35f : 0.0f);
                _coup[i][j] = HANG_PITCH_AMT * pitchW + HANG_NEIGH_AMT * neighW;
            }
    }

    // Bow force for a pad (0..1 from proximity). 0 = not bowed (it can still ring
    // sympathetically). Steepened by HANG_FORCE_EXP for a responsive feel.
    void setForce(uint8_t v, float f)
    {
        if (v >= HANG_VOICES) return;
        if (f < 0.0f) f = 0.0f; else if (f > 1.0f) f = 1.0f;
        _force[v] = powf(f, HANG_FORCE_EXP);
    }

    void setLevel(float l) { _level = l; }  // master output level (0 mutes the engine)

    virtual void update();

private:
    float _freq  [HANG_VOICES] = { 0 };
    float _force [HANG_VOICES] = { 0 };   // target (already steepened)
    float _forceS[HANG_VOICES] = { 0 };   // slewed
    float _y1 [HANG_VOICES][HANG_MODES] = {{0}};
    float _y2 [HANG_VOICES][HANG_MODES] = {{0}};
    float _a1 [HANG_VOICES][HANG_MODES] = {{0}};
    float _r2 [HANG_VOICES][HANG_MODES] = {{0}};
    float _norm[HANG_VOICES][HANG_MODES] = {{0}};
    float _vout[HANG_VOICES] = { 0 };     // last sample's per-voice output (cross-coupling)
    float _lp  [HANG_VOICES] = { 0 };     // per-voice bow-noise low-pass (pole 1)
    float _lp2 [HANG_VOICES] = { 0 };     // bow-noise low-pass (pole 2 — steeper, less hiss)
    float _coup[HANG_VOICES][HANG_VOICES] = {{0}}; // coupling matrix (diag = 0)
    float _level = 0.5f;
    float _outLp = 0.0f;      // output low-pass state
    uint32_t _rng = 0x9E3779B9u;

    inline float noise() { _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
                           return (int32_t)_rng * (1.0f / 2147483648.0f); }

    // Harmonic overlap: 1.0 when a low partial of one note lands on a partial of
    // the other (octave/fifth/etc.), falling off over HANG_COUPLE_WIDTH cents.
    static float harmonicOverlap(float fi, float fj)
    {
        if (!(fi > 0.0f) || !(fj > 0.0f)) return 0.0f;
        float best = 0.0f;
        for (int a = 1; a <= 4; a++)
            for (int b = 1; b <= 4; b++) {
                float cents = 1731.234f * logf((a * fi) / (b * fj)); // 1200/ln2
                float w = expf(-(cents * cents) / (2.0f * HANG_COUPLE_WIDTH * HANG_COUPLE_WIDTH));
                float s = w / (float)(a * b);   // higher partials couple weaker
                if (s > best) best = s;
            }
        return best;
    }
};

inline void AudioHangBow::update()
{
    audio_block_t* out = allocate();
    if (!out) return;

    // Sanitise: the cross-feedback is a true feedback loop — never let it NaN/blow
    // the whole mix. Reset any voice that has gone non-finite.
    float act = 0.0f;
    for (uint8_t v = 0; v < HANG_VOICES; v++) {
        if (!isfinite(_vout[v]) || !isfinite(_forceS[v])) {
            _vout[v] = _forceS[v] = _lp[v] = _lp2[v] = 0.0f;
            for (uint8_t m = 0; m < HANG_MODES; m++) _y1[v][m] = _y2[v][m] = 0.0f;
        }
        float a = fabsf(_vout[v]); if (a > act) act = a;
        if (_force[v] > act) act = _force[v];
    }
    // Fully idle (not the active mode and everything rung out) → emit silence.
    if (act < 1.0e-4f) {
        memset(out->data, 0, sizeof(out->data));
        transmit(out); release(out); return;
    }

    float nv[HANG_VOICES];
    for (int s = 0; s < AUDIO_BLOCK_SAMPLES; s++)
    {
        float mix = 0.0f;
        for (uint8_t v = 0; v < HANG_VOICES; v++)
        {
            // Bow force slew (fast attack, gentle release — feels like leaning in).
            float tgt = _force[v];
            _forceS[v] += (tgt > _forceS[v] ? 0.020f : HANG_FORCE_SMOOTH) * (tgt - _forceS[v]);
            float fs_ = _forceS[v];

            // Friction exciter: band-limited noise, brighter as you bow harder.
            // Two cascaded poles → 12 dB/oct rolloff, so far less broadband hiss.
            float n = noise();
            float c = HANG_BOW_LP_MIN + HANG_BOW_LP_MAX * fs_;
            _lp[v]  += c * (n       - _lp[v]);
            _lp2[v] += c * (_lp[v]  - _lp2[v]);
            float exc = fs_ * _lp2[v] * HANG_BOW_GAIN;

            // Over-pressure: past the threshold, a nonlinear feedback of the
            // voice's own output squeals into a scratchy multiphonic.
            float over = fs_ - HANG_OVER_THRESH;
            if (over > 0.0f) {
                over *= 1.0f / (1.0f - HANG_OVER_THRESH);     // → 0..1 across the top
                exc += over * tanhf(HANG_OVER_FB * _vout[v]) * HANG_OVER_GAIN;
            }

            // Sympathetic bleed from every other voice (1-sample delayed → stable).
            float cross = 0.0f;
            const float* cr = _coup[v];
            for (uint8_t j = 0; j < HANG_VOICES; j++) cross += cr[j] * _vout[j];
            exc += cross * HANG_CROSS_GAIN;

            // Drive the resonator bank.
            float vsum = 0.0f;
            for (uint8_t m = 0; m < HANG_MODES; m++) {
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
        for (uint8_t v = 0; v < HANG_VOICES; v++) _vout[v] = nv[v];

        _outLp += HANG_OUT_LP * (mix - _outLp); // tame residual broadband hiss
        float o = _outLp * _level;
        if (!isfinite(o)) o = 0.0f; else if (o > 1.0f) o = 1.0f; else if (o < -1.0f) o = -1.0f;
        out->data[s] = (int16_t)(o * 20000.0f);
    }

    transmit(out);
    release(out);
}
