// Unit tests for the proximity engine (EMA filters + touch detection).
//
// Run with:  pio test -e native
//
// These tests exercise proximity_engine.h in isolation — no hardware needed.

#include <unity.h>
#include <math.h>
#include "../../src/proximity_engine.h"

static ProximityConfig cfg;

void setUp() {
    cfg = ProximityConfig{}; // reset to defaults before each test
}
void tearDown() {}

// ─────────────────────────────────────────────────────────────────────────────

// seedSensorState should initialise both EMAs and prevFast to the given delta.
void test_seed_initialises_emas() {
    SensorState s;
    seedSensorState(s, 15.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, s.fast);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, s.slow);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, s.prevFast);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,  0.0f, s.velocity);
    TEST_ASSERT_EQUAL_UINT32(0, s.lastTouchMs);
}

// Zero delta → zero intensity, no touch.
void test_zero_delta_gives_zero_intensity() {
    SensorState s;
    seedSensorState(s, 0.0f);
    float intensity; bool touch;
    updateProximity(0.0f, 1000, cfg, s, intensity, touch);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, intensity);
    TEST_ASSERT_FALSE(touch);
}

// Delta just below the deadband → intensity must stay at 0.
void test_intensity_clamped_to_zero_below_deadband() {
    SensorState s;
    float below = cfg.proxDeadband - 0.5f;
    seedSensorState(s, below);
    float intensity; bool touch;
    updateProximity(below, 1000, cfg, s, intensity, touch);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, intensity);
}

// Delta at or above proxMax → intensity must be exactly 1.0.
void test_intensity_at_proxmax_is_one() {
    SensorState s;
    seedSensorState(s, cfg.proxMax);
    float intensity; bool touch;
    updateProximity(cfg.proxMax, 1000, cfg, s, intensity, touch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, intensity);
}

// Delta well above proxMax → intensity must be clamped to 1.0.
void test_intensity_above_proxmax_clamped_to_one() {
    SensorState s;
    seedSensorState(s, 1000.0f);
    float intensity; bool touch;
    updateProximity(1000.0f, 1000, cfg, s, intensity, touch);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, intensity);
}

// After a single large step, fast EMA should have moved much more than slow EMA.
void test_fast_ema_rises_faster_than_slow() {
    SensorState s;
    seedSensorState(s, 0.0f);
    float intensity; bool touch;
    updateProximity(50.0f, 1000, cfg, s, intensity, touch);
    TEST_ASSERT_TRUE(s.fast > s.slow);
    // fast should be: alpha_rise * 50 = 0.50 * 50 = 25.0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, cfg.fastAlphaRise * 50.0f, s.fast);
    // slow should be: alpha_slow * 50 = 0.01 * 50 = 0.5
    TEST_ASSERT_FLOAT_WITHIN(0.001f, cfg.slowAlpha * 50.0f, s.slow);
}

// When delta drops to 0 from a non-zero fast EMA, decay uses fastAlphaFall.
void test_fast_ema_decays_with_fall_alpha() {
    SensorState s;
    seedSensorState(s, 20.0f);
    float intensity; bool touch;
    updateProximity(0.0f, 1000, cfg, s, intensity, touch);
    float expected = (1.0f - cfg.fastAlphaFall) * 20.0f; // = 0.65 * 20 = 13.0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, s.fast);
}

// A touch event fires once, then the cooldown prevents re-firing.
// After the cooldown expires a second event can fire again.
void test_touch_fires_once_then_cooldown() {
    cfg.jumpThreshold = 5.0f;
    cfg.cooldownMs    = 400;

    SensorState s;
    // Pre-set state so fast >> slow, guaranteeing jump > threshold immediately.
    s.fast        = 20.0f;
    s.slow        = 0.0f;
    s.prevFast    = 20.0f;
    s.velocity    = 0.0f;
    s.lastTouchMs = 0;

    float intensity; bool touch;

    // Frame 1 at t=1000 ms: cooldown not active, jump = ~19.8 > 5 → fires.
    updateProximity(20.0f, 1000, cfg, s, intensity, touch);
    TEST_ASSERT_TRUE(touch);
    TEST_ASSERT_EQUAL_UINT32(1000, s.lastTouchMs);

    // Frame 2 at t=1200 ms: within cooldown window → must not fire.
    updateProximity(20.0f, 1200, cfg, s, intensity, touch);
    TEST_ASSERT_FALSE(touch);

    // Frame 3 at t=1500 ms: cooldown has expired (500 > 400 ms).
    // Reset slow to 0 so the jump stays large enough to trigger again.
    s.slow = 0.0f;
    updateProximity(20.0f, 1500, cfg, s, intensity, touch);
    TEST_ASSERT_TRUE(touch);
    TEST_ASSERT_EQUAL_UINT32(1500, s.lastTouchMs);
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_seed_initialises_emas);
    RUN_TEST(test_zero_delta_gives_zero_intensity);
    RUN_TEST(test_intensity_clamped_to_zero_below_deadband);
    RUN_TEST(test_intensity_at_proxmax_is_one);
    RUN_TEST(test_intensity_above_proxmax_clamped_to_one);
    RUN_TEST(test_fast_ema_rises_faster_than_slow);
    RUN_TEST(test_fast_ema_decays_with_fall_alpha);
    RUN_TEST(test_touch_fires_once_then_cooldown);
    return UNITY_END();
}
