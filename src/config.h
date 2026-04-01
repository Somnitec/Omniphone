#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// Instrument configuration
//
// Edit this file to match your physical wiring.  The rest of the firmware
// reads from SENSORS[] and BOARD_ADDRESSES[] at compile time — no other
// files need to change when you remap pads or retune frequencies.
//
// NUM_BOARDS   : how many MPR121 boards are connected (1–4).
// BOARD_ADDRESSES : I²C addresses for boards 0, 1, 2, 3.
//                   Unused slots can be 0x00.
//
// Each SensorConfig entry binds one physical pad to:
//   boardIndex  → which entry in BOARD_ADDRESSES[]
//   electrode   → which of ELE0–ELE5 on that board (0–5)
//   noteFreq    → base pitch for the sine-wave voice (Hz)
// ─────────────────────────────────────────────────────────────────────────────

// ── Note frequencies (equal temperament, A4 = 440 Hz) ────────────────────────
namespace Note {
    static constexpr float C3  = 130.81f;
    static constexpr float D3  = 146.83f;
    static constexpr float E3  = 164.81f;
    static constexpr float G3  = 196.00f;
    static constexpr float A3  = 220.00f;

    static constexpr float C4  = 261.63f; // middle C
    static constexpr float D4  = 293.66f;
    static constexpr float E4  = 329.63f;
    static constexpr float G4  = 392.00f;
    static constexpr float A4  = 440.00f;

    static constexpr float C5  = 523.25f;
    static constexpr float D5  = 587.33f;
    static constexpr float E5  = 659.25f;
    static constexpr float G5  = 783.99f;
    static constexpr float A5  = 880.00f;
}

// ── Board setup ───────────────────────────────────────────────────────────────
static constexpr uint8_t NUM_BOARDS        = 2;
static constexpr uint8_t BOARD_ADDRESSES[] = { 0x5B, 0x5D, 0x00, 0x00 };

// ── Sensor descriptor ─────────────────────────────────────────────────────────
struct SensorConfig {
    uint8_t boardIndex; // Index into BOARD_ADDRESSES[]
    uint8_t electrode;  // ELE0–ELE5 on that board (0–5)
    float   noteFreq;   // Base pitch in Hz
};

// ── Sensor list ───────────────────────────────────────────────────────────────
// Two boards × 6 electrodes = 12 sensors mapped to two octaves of C pentatonic.
// Change noteFreq values here to retune individual pads.
static constexpr SensorConfig SENSORS[] = {
    // Board 0 , ELE0–ELE5 → low octave
    { 0, 0, Note::C3 },
    { 0, 1, Note::D3 },
    { 0, 2, Note::E3 },
    { 0, 3, Note::G3 },
    { 0, 4, Note::A3 },
    { 0, 5, Note::C4 },
    
    // Board 1 , ELE0–ELE5 → high octave
    { 1, 0, Note::D4 },
    { 1, 1, Note::E4 },
    { 1, 2, Note::G4 },
    { 1, 3, Note::A4 },
    { 1, 4, Note::C5 },
    { 1, 5, Note::D5 },
};

static constexpr uint8_t NUM_SENSORS =
    static_cast<uint8_t>(sizeof(SENSORS) / sizeof(SENSORS[0]));
