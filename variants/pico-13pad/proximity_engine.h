#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// Proximity engine — pure C++, no Arduino dependencies
//
// Converts raw MPR121 delta values (baseline − filtered) into:
//   outIntensity  0.0 (hand far) → 1.0 (very close), used for amplitude + LED
//   outTouch      true on the frame a contact event is detected
//
// Signal model:
//   delta = baseline − filtered  (positive when hand is near)
//
//   fast EMA (α_rise / α_fall)   follows hand movement; drives amplitude + LEDs
//   slow EMA (α_slow)            tracks only slow environmental drift
//
//   jump = fast − slow           stays near zero during a gradual approach,
//                                spikes suddenly when metal contact occurs
//                                (the slow EMA can't keep up) → touch event
//
// This file is safe to include in native unit tests: no hardware I/O is used.
// ─────────────────────────────────────────────────────────────────────────────

// ── Tuning parameters (one shared instance per instrument is fine) ────────────
struct ProximityConfig {
    float    proxDeadband  = 2.5f;   // delta below this → intensity = 0
    float    proxMax       = 18.0f;  // delta at which intensity = 1.0
    float    jumpThreshold = 40.0f;  // fast−slow spike that flags metal contact
    float    fastAlphaRise = 0.50f;  // EMA coefficient when hand is approaching
    float    fastAlphaFall = 0.35f;  // EMA coefficient when hand is retreating
    float    slowAlpha     = 0.01f;  // slow EMA coefficient (ambient drift)
    float    velAlpha      = 0.30f;  // smoothing for the velocity estimate
    float    releaseRatio  = 0.5f;   // re-arm once jump falls below this × jumpThreshold
    uint32_t cooldownMs    = 30;     // edge debounce — min ms between events per pad
    uint8_t  confirmFrames = 1;      // consecutive over-threshold frames before firing
                                     // (1 = instant; >1 rejects single-sample glitches)
    uint8_t  proxConfirmFrames = 1;  // fast must be over deadband this many frames
                                     // in a row before intensity is allowed to rise
                                     // (rejects short proximity blips, ~8 ms each)
};

// ── Per-sensor mutable state ─────────────────────────────────────────────────
struct SensorState {
    float    fast        = 0.0f; // fast EMA → drives amplitude + LED brightness
    float    slow        = 0.0f; // slow EMA → ambient drift reference
    float    velocity    = 0.0f; // smoothed d(fast)/dt → approach speed at contact
    float    prevFast    = 0.0f;
    uint32_t lastTouchMs = 0;     // timestamp of last touch event (ms)
    bool     inContact   = false; // edge latch: true between a strike and its release
    uint8_t  overCount   = 0;     // consecutive frames jump has been over-threshold
    uint8_t  aboveCount  = 0;     // consecutive frames fast has been over the deadband
};

// ── Seed state from an initial reading ───────────────────────────────────────
// Call once after startup reads so the jump detector does not fire spuriously
// on the first loop iteration.
inline void seedSensorState(SensorState& s, float initialDelta) {
    s.fast        = initialDelta;
    s.slow        = initialDelta;
    s.prevFast    = initialDelta;
    s.velocity    = 0.0f;
    s.lastTouchMs = 0;
    s.inContact   = false;
    s.overCount   = 0;
    s.aboveCount  = 0;
}

// ── Update one sensor for the current frame ───────────────────────────────────
// rawDelta   : baseline − filtered, clamped to ≥ 0 before calling
// nowMs      : current time in milliseconds (from millis())
// cfg        : tuning parameters
// state      : per-sensor mutable state (updated in place)
// outIntensity : normalised proximity in [0.0, 1.0]
// outTouch     : set to true if a contact event fired this frame
inline void updateProximity(float rawDelta, uint32_t nowMs,
                            const ProximityConfig& cfg, SensorState& state,
                            float& outIntensity, bool& outTouch) {
    // ── Asymmetric EMA ────────────────────────────────────────────────────────
    // Rises quickly (catches incoming hand) but falls slowly (sustains notes).
    float alpha   = (rawDelta > state.fast) ? cfg.fastAlphaRise : cfg.fastAlphaFall;
    float newFast = alpha * rawDelta + (1.0f - alpha) * state.fast;

    // Smoothed velocity: the rate of change of the fast EMA.
    // Positive when hand is approaching; used as touch "velocity" for expression.
    float rawVel   = newFast - state.fast;
    state.velocity = cfg.velAlpha * rawVel + (1.0f - cfg.velAlpha) * state.velocity;

    state.prevFast = state.fast;
    state.fast     = newFast;
    state.slow     = cfg.slowAlpha * rawDelta + (1.0f - cfg.slowAlpha) * state.slow;

    // ── Normalised intensity ──────────────────────────────────────────────────
    float norm     = (state.fast - cfg.proxDeadband) / (cfg.proxMax - cfg.proxDeadband);
    outIntensity   = norm < 0.0f ? 0.0f : (norm > 1.0f ? 1.0f : norm);

    // Hold intensity at 0 until fast has stayed above the deadband for
    // proxConfirmFrames in a row → kills short single-frame transient blips.
    if (state.fast > cfg.proxDeadband) {
        if (state.aboveCount < 255) state.aboveCount++;
    } else {
        state.aboveCount = 0;
    }
    if (state.aboveCount < cfg.proxConfirmFrames) outIntensity = 0.0f;

    // ── Contact detection (edge-triggered, with hysteresis) ───────────────────
    // jump spikes when the hand suddenly contacts metal (fast catches the step,
    // slow cannot keep up). Fire ONCE on the rising edge — not continuously
    // while the pad is held. The latch re-arms only after jump falls back below
    // releaseRatio × jumpThreshold (the hand has lifted), so a sustained hold
    // never re-triggers and successive taps each register as their own edge.
    float jump = state.fast - state.slow;
    outTouch   = false;

    if (!state.inContact) {
        if (jump > cfg.jumpThreshold &&
            (nowMs - state.lastTouchMs) > cfg.cooldownMs) {
            // Require N consecutive over-threshold frames so a single-sample
            // electrical glitch can't fire a phantom touch.
            if (++state.overCount >= cfg.confirmFrames) {
                state.inContact   = true;
                state.lastTouchMs = nowMs;
                state.overCount   = 0;
                outTouch          = true;
            }
        } else {
            state.overCount = 0; // streak broken — reset
        }
    } else if (jump < cfg.jumpThreshold * cfg.releaseRatio) {
        state.inContact = false; // released — armed for the next strike
        state.overCount = 0;
    }
}
