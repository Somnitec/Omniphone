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
    TEST_ASSERT_FALSE(s.inContact);
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

// Touch is edge-triggered: fires once on the rising edge, does NOT re-fire
// while the pad is held (even long after the debounce window), and re-arms
// only after the jump collapses (hand lifted) — then a new strike fires again.
void test_touch_edge_triggered_with_hysteresis() {
    cfg.jumpThreshold = 10.0f;
    cfg.releaseRatio  = 0.5f;   // re-arm once jump < 5.0
    cfg.cooldownMs    = 30;

    SensorState s;
    seedSensorState(s, 0.0f);
    s.fast = 20.0f; s.slow = 0.0f; s.prevFast = 20.0f; // jump ≈ 20 > 10

    float intensity; bool touch;

    // Rising edge → fires once, latches.
    updateProximity(20.0f, 1000, cfg, s, intensity, touch);
    TEST_ASSERT_TRUE(touch);
    TEST_ASSERT_TRUE(s.inContact);

    // Held high → must NOT re-fire, even far past the debounce window.
    s.slow = 0.0f; // keep jump large (simulates a sustained hold)
    updateProximity(20.0f, 2000, cfg, s, intensity, touch);
    TEST_ASSERT_FALSE(touch);
    s.slow = 0.0f;
    updateProximity(20.0f, 5000, cfg, s, intensity, touch);
    TEST_ASSERT_FALSE(touch);

    // Hand lifts: jump collapses below releaseRatio×threshold → re-arm.
    s.fast = 0.0f; s.slow = 0.0f;
    updateProximity(0.0f, 6000, cfg, s, intensity, touch);
    TEST_ASSERT_FALSE(touch);
    TEST_ASSERT_FALSE(s.inContact);

    // Next strike fires again.
    s.fast = 20.0f; s.slow = 0.0f;
    updateProximity(20.0f, 7000, cfg, s, intensity, touch);
    TEST_ASSERT_TRUE(touch);
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
    RUN_TEST(test_touch_edge_triggered_with_hysteresis);
    return UNITY_END();
}
