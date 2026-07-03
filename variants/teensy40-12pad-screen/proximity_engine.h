#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// Proximity engine — pure C++, no Arduino dependencies
//
// Converts the raw MPR121 FILTERED value (10-bit; a hand pulls it DOWN) into:
//   outIntensity  0.0 (hand far) → 1.0 (very close), used for amplitude + LED
//   outTouch      true on the frame a contact event is detected
//
// Signal model:
//   delta = softwareBaseline − filtered   (positive when hand is near)
//
// The chip's own baseline registers are NOT read — they quantize to 4 counts
// and their falling filter chases slow approaches (both measured, 2026-07-03).
// The software baseline in SensorState.base is the only zero reference; it
// idles/tracks/freezes/heals per the state machine in updateProximity.
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
    float    fastAlphaRise = 0.50f;  // base EMA coefficient on a slow approach
    float    fastJumpRef    = 5.0f;   // jump (counts/frame) that gives full-speed
                                      // attack — bigger sudden change ⇒ snappier
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
    float    proxReleaseDelta = 0.0f; // hysteresis: once open, the gate only closes
                                      // when fast falls this far BELOW the deadband
                                      // (0 = no hysteresis; stops edge strobing)
    float    smoothAlpha   = 1.0f;   // 1-pole low-pass on the output intensity when
                                     // FALLING (1 = none; lower = smoother, less flicker)
    float    smoothAlphaRise = 1.0f; // 1-pole when RISING (1 = instant attack)
    float    smoothFallRef = 0.5f;   // intensity drop (per frame) that unlocks a
                                     // full-speed fall — big drops (hand yanked
                                     // away) fade fast, small dips stay smoothed
    float    fallJumpRef   = 8.0f;   // downward delta step (counts/frame) that gives
                                     // full-speed release tracking; small dips keep
                                     // fastAlphaFall (anti-strobe), a real lift snaps
    // ── Software baseline (full port of the esp32s3-19pad design) ─────────────
    // The delta is computed against a per-pad SOFTWARE baseline on the raw
    // filtered value — the chip's baseline registers are not used at all. Two
    // measured reasons (see the 2026-07-03 characterization capture):
    //   • the register exposes only the top 8 of the baseline's 10 bits, so it
    //     moves in jumps of 4 counts — with a deadband of 4, every jump was a
    //     ghost note / flutter until the heal absorbed it;
    //   • the chip's falling baseline filter chases an approaching hand (2×
    //     faster at ESI=1 ms), eating the delta on slow approaches = lost reach.
    // The software baseline moves ONLY per the state machine below:
    //   idle  (eff < deadband)      → base slowly TRACKS the filtered value
    //                                 (absorbs EM drift, rotation, temperature)
    //   negative delta (filt > base)→ base tracks FAST — a hand can only lower
    //                                 the filtered value, so a negative delta is
    //                                 unambiguously a stale baseline (pad was
    //                                 desensitised; recover in ~0.1 s, not ~2 s)
    //   held  (eff ≥ deadband)      → base FREEZES (sustained notes don't fade)
    //   stuck (held > holdMaxFrames)→ base HEALS toward filtered (re-zeroes a
    //                                 pad no musical hold is this long)
    float    baseIdleAlpha = 0.003f; // idle drift tracking (~2 s tau @ 6 ms frames;
                                     // slower = more reach on very slow approaches,
                                     // faster = quicker recovery from EM change)
    float    baseNegAlpha  = 0.05f;  // negative-delta recovery (~0.12 s tau)
    float    healAlpha     = 0.02f;  // stuck-pad re-zero rate (~0.4 s tau)
    uint16_t holdMaxFrames = 1000;   // frames above deadband before "stuck" verdict
                                     // (1000 ≈ 8 s @ 125 Hz — longer than any hold)
    // Low-residual fast heal: with several pads pressed, a RELEASED pad still
    // sees a small coupling delta from the other fingers/body. If that residual
    // parks just above the deadband the pad glows dim/drones quietly until the
    // 8 s stuck timeout. A value that sits INSIDE the low band (deadband …
    // deadband+lowHoldBand) for lowHoldFrames straight is such a residual —
    // real playing rises above the band (resets the counter) or drops below
    // the deadband. Heals at healAlpha, i.e. fades out over ~0.5 s.
    float    lowHoldBand   = 3.0f;   // counts above deadband = "low" (≈ intensity <0.25)
    uint16_t lowHoldFrames = 250;    // ≈ 2 s @ 125 Hz before a parked low value heals
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
    bool     gateOpen    = false; // proximity gate latch (hysteresis, anti-strobe)
    float    intensitySm = 0.0f;  // smoothed output intensity (1-pole)
    float    base        = 0.0f;  // software baseline of the raw filtered value
    uint16_t holdFrames  = 0;     // consecutive frames the pad has been above deadband
    uint16_t lowFrames   = 0;     // consecutive frames parked in the low band
};

// ── Seed state from an initial reading ───────────────────────────────────────
// Call once after startup reads so the jump detector does not fire spuriously
// on the first loop iteration. Pass the pad's current RAW FILTERED value — it
// becomes the initial software baseline. (Nothing should be touching; a finger
// present at seed time self-heals via the fast negative-delta recovery once it
// lifts.)
inline void seedSensorState(SensorState& s, float initialFiltered) {
    s.fast        = 0.0f;
    s.slow        = 0.0f;
    s.prevFast    = 0.0f;
    s.velocity    = 0.0f;
    s.lastTouchMs = 0;
    s.inContact   = false;
    s.overCount   = 0;
    s.aboveCount  = 0;
    s.gateOpen    = false;
    s.intensitySm = 0.0f;
    s.base        = initialFiltered;
    s.holdFrames  = 0;
    s.lowFrames   = 0;
}

// ── Update one sensor for the current frame ───────────────────────────────────
// rawFiltered : the pad's raw 10-bit FILTERED value straight from the chip. A
//               hand near the pad pulls it DOWN. The chip's baseline registers
//               are deliberately not used (4-count quantization jumps + their
//               falling filter chases slow approaches) — the software baseline
//               below is the only zero reference, at sub-count resolution.
// nowMs      : current time in milliseconds (from millis())
// cfg        : tuning parameters
// state      : per-sensor mutable state (updated in place)
// outIntensity : normalised proximity in [0.0, 1.0]
// outTouch     : set to true if a contact event fired this frame
inline void updateProximity(float rawFiltered, uint32_t nowMs,
                            const ProximityConfig& cfg, SensorState& state,
                            float& outIntensity, bool& outTouch) {
    // ── Software baseline ──────────────────────────────────────────────────────
    // effDelta = base − filtered is what everything below runs on. The baseline
    // tracks slow environmental change while the pad is idle, snaps up quickly
    // on a negative delta (unambiguously stale — a hand only pulls filtered
    // DOWN), freezes during a musical hold, and heals a pad "held" longer than
    // any musical hold — an object set down nearby, or a rotation shift.
    float effDelta = state.base - rawFiltered;
    if (effDelta < cfg.proxDeadband) {
        state.holdFrames = 0;
        state.lowFrames  = 0;
        float k = (effDelta < 0.0f) ? cfg.baseNegAlpha : cfg.baseIdleAlpha;
        state.base += k * (rawFiltered - state.base);
        if (effDelta < 0.0f) effDelta = 0.0f;
    } else {
        if (state.holdFrames < 0xFFFF) state.holdFrames++;
        // Low-residual detector: parked just above the deadband (dim LED /
        // quiet drone that isn't a hand) — see lowHoldBand in the config.
        if (effDelta < cfg.proxDeadband + cfg.lowHoldBand) {
            if (state.lowFrames < 0xFFFF) state.lowFrames++;
        } else {
            state.lowFrames = 0;         // real playing level — not a residual
        }
        if (state.holdFrames > cfg.holdMaxFrames ||
            state.lowFrames  > cfg.lowHoldFrames)
            state.base += cfg.healAlpha * (rawFiltered - state.base);
        // else: frozen — a held note sustains at full level
    }

    // ── Velocity-adaptive EMA ──────────────────────────────────────────────────
    // Rising: the bigger the sudden jump, the faster we track it — a hand flying
    // in (large step) snaps almost instantly, while a slow hover near the edge
    // (tiny steps) keeps the gentle smoothing. Falling mirrors it: a real lift
    // (large downward step) tracks near-instantly, noise dips stay damped.
    float riseStep = effDelta - state.fast; // this frame's change in the signal
    float alpha;
    if (riseStep > 0.0f) {
        float t = riseStep / cfg.fastJumpRef;      // 0 (creep) … 1 (big jump)
        if (t > 1.0f) t = 1.0f;
        alpha = cfg.fastAlphaRise + (1.0f - cfg.fastAlphaRise) * t; // base … 1.0
    } else {
        float t = -riseStep / cfg.fallJumpRef;     // 0 (dip) … 1 (hand yanked away)
        if (t > 1.0f) t = 1.0f;
        alpha = cfg.fastAlphaFall + (1.0f - cfg.fastAlphaFall) * t;
    }
    float newFast = alpha * effDelta + (1.0f - alpha) * state.fast;

    // Smoothed velocity: the rate of change of the fast EMA.
    // Positive when hand is approaching; used as touch "velocity" for expression.
    float rawVel   = newFast - state.fast;
    state.velocity = cfg.velAlpha * rawVel + (1.0f - cfg.velAlpha) * state.velocity;

    state.prevFast = state.fast;
    state.fast     = newFast;
    state.slow     = cfg.slowAlpha * effDelta + (1.0f - cfg.slowAlpha) * state.slow;

    // ── Normalised intensity ──────────────────────────────────────────────────
    float norm     = (state.fast - cfg.proxDeadband) / (cfg.proxMax - cfg.proxDeadband);
    norm           = norm < 0.0f ? 0.0f : (norm > 1.0f ? 1.0f : norm);

    // ── Anti-strobe gate (hysteresis) ────────────────────────────────────────
    // Opening still needs proxConfirmFrames over the deadband (rejects blips and
    // sets the playable distance). Once open, the gate only closes when fast
    // drops proxReleaseDelta BELOW the deadband — so a single noisy frame at the
    // far edge can't slam intensity to 0 and strobe the tone/LED.
    if (state.fast > cfg.proxDeadband) {
        if (state.aboveCount < 255) state.aboveCount++;
        if (state.aboveCount >= cfg.proxConfirmFrames) state.gateOpen = true;
        // A strong, sudden approach snaps the gate open now — skip the confirm
        // latency (the confirm frames only exist to reject tiny slow blips).
        if (riseStep > cfg.fastJumpRef) state.gateOpen = true;
    } else {
        state.aboveCount = 0;
        if (state.fast < cfg.proxDeadband - cfg.proxReleaseDelta) state.gateOpen = false;
    }
    float target = state.gateOpen ? norm : 0.0f;

    // 1-pole low-pass on the output → removes residual frame-to-frame flicker.
    // Falling is drop-adaptive: a big drop (real release) fades at full speed,
    // small dips (edge jitter) keep the gentle smoothing.
    float sa;
    if (target > state.intensitySm) {
        sa = cfg.smoothAlphaRise;
    } else {
        float d = (state.intensitySm - target) / cfg.smoothFallRef;
        if (d > 1.0f) d = 1.0f;
        sa = cfg.smoothAlpha + (1.0f - cfg.smoothAlpha) * d;
    }
    state.intensitySm += sa * (target - state.intensitySm);
    outIntensity = state.intensitySm;

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
