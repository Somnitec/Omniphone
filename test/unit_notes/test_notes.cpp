// Unit tests for sensor configuration and note frequency correctness.
//
// Run with:  pio test -e native

#include <unity.h>
#include "../../src/config.h"

void setUp() {}
void tearDown() {}

// ─────────────────────────────────────────────────────────────────────────────

// Equal temperament: each octave must double the frequency.
void test_octave_doubles_frequency() {
    TEST_ASSERT_FLOAT_WITHIN(0.1f, Note::C4, Note::C3 * 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, Note::C5, Note::C4 * 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, Note::A4, Note::A3 * 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, Note::A5, Note::A4 * 2.0f);
}

// All sensor frequencies must be positive (a zero or negative Hz would silence
// or crash the audio engine).
void test_all_sensor_frequencies_positive() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        TEST_ASSERT_TRUE_MESSAGE(SENSORS[i].noteFreq > 0.0f,
                                 "sensor noteFreq must be > 0");
    }
}

// All sensor frequencies should be in a musically sensible range.
// Sub-20 Hz and above 20 kHz are outside human hearing.
void test_all_sensor_frequencies_in_audible_range() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        TEST_ASSERT_TRUE_MESSAGE(SENSORS[i].noteFreq >= 20.0f,
                                 "noteFreq below 20 Hz (inaudible)");
        TEST_ASSERT_TRUE_MESSAGE(SENSORS[i].noteFreq <= 20000.0f,
                                 "noteFreq above 20 kHz (inaudible)");
    }
}

// Every sensor must reference a board index within the declared board count.
void test_all_sensor_board_indices_valid() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        TEST_ASSERT_TRUE_MESSAGE(SENSORS[i].boardIndex < NUM_BOARDS,
                                 "boardIndex out of range");
    }
}

// Every sensor electrode must be 0–5 (the six touch electrodes per board).
void test_all_sensor_electrodes_valid() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        TEST_ASSERT_TRUE_MESSAGE(SENSORS[i].electrode <= 5,
                                 "electrode out of range (0–5)");
    }
}

// Each (boardIndex, electrode) pair must be unique — two sensors cannot share
// the same physical pad.
void test_no_duplicate_board_electrode_pairs() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        for (uint8_t j = i + 1; j < NUM_SENSORS; j++) {
            bool samePad = (SENSORS[i].boardIndex == SENSORS[j].boardIndex)
                        && (SENSORS[i].electrode  == SENSORS[j].electrode);
            TEST_ASSERT_FALSE_MESSAGE(samePad,
                                      "two sensors share the same board+electrode");
        }
    }
}

// NUM_SENSORS must not exceed the total electrode capacity of the defined boards.
void test_sensor_count_within_board_capacity() {
    uint8_t maxSensors = NUM_BOARDS * 6; // 6 electrodes per board
    TEST_ASSERT_TRUE_MESSAGE(NUM_SENSORS <= maxSensors,
                             "NUM_SENSORS exceeds available electrodes");
}

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_octave_doubles_frequency);
    RUN_TEST(test_all_sensor_frequencies_positive);
    RUN_TEST(test_all_sensor_frequencies_in_audible_range);
    RUN_TEST(test_all_sensor_board_indices_valid);
    RUN_TEST(test_all_sensor_electrodes_valid);
    RUN_TEST(test_no_duplicate_board_electrode_pairs);
    RUN_TEST(test_sensor_count_within_board_capacity);
    return UNITY_END();
}
