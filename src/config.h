#pragma once
#include <stdint.h>
#include <Audio.h> // for WAVEFORM_* constants

// ─────────────────────────────────────────────────────────────────────────────
// Instrument configuration
//
// Edit this file to match your physical wiring and musical preferences.
// The rest of the firmware reads from SENSORS[], SOUND_SETS[], and the
// synth parameter constants — no other files need to change when you
// remap pads, retune frequencies, or add new sound sets.
// ─────────────────────────────────────────────────────────────────────────────

// ── Board setup ──────────────────────────────────────────────────────────────
static constexpr uint8_t NUM_BOARDS        = 2;
static constexpr uint8_t BOARD_ADDRESSES[] = { 0x5A, 0x5C, 0x00, 0x00 };

// ── Sensor descriptor ────────────────────────────────────────────────────────
static constexpr uint8_t NO_PIN = 0xFF; // disabled pad / no LED

struct SensorConfig {
    uint8_t boardIndex; // SENSE board: index into BOARD_ADDRESSES[]
    uint8_t electrode;  // ELE0–ELE5 touch input, or NO_PIN if this pad is disabled
    uint8_t ledBoard;   // LED board (may differ from boardIndex — see idx1)
    uint8_t ledEle;     // ELE5–ELE11 LED pin, or NO_PIN if no LED
};

// Touch electrodes per board, indexed by board. ELE_EN is a contiguous count
// from ELE0, so LED pins can only live above the last sense electrode.
//   A (0x5A): ELE0–ELE4 (5)  — ELE5 freed as a GPIO LED pin (idx1's lamp)
//   C (0x5C): ELE0–ELE5 (6)  — idx5 senses on ELE5, LEDs limited to ELE6–ELE11
static constexpr uint8_t SENSE_ELECTRODES[NUM_BOARDS] = { 5, 6 };

// ── Physical sensor layout (post ELE9/ELE10 rework) ──────────────────────────
// 11 pad slots. Boards: 0 = 0x5A ("A"), 1 = 0x5C ("C").
// LED pin AND LED board are explicit per pad. Rework summary:
//   • A: the LED that was on A-ELE9 is now on A-ELE6   (idx8)
//   • C: the LED that was on C-ELE9 is now on A-ELE5   (idx1, cross-board!)
//   • A: idx7's touch electrode moved A-ELE5 → A-ELE0  (freed A-ELE5 for idx1's LED)
//   • idx5 (upper ring 4) still senses on C-ELE5; its LED stays C-ELE11
//   • ELE10 LEDs (idx3 A, idx10 C) only light if ELE9 is driven identically;
//     nothing is wired to ELE9 — handled in main (mirror ELE9 ← ELE10 per board).
//
// Hangdrum "ding" layout: TOP is the central low fundamental, upper ring is
// the melodic tone fields ascending around it, lower ring is bass voicings.
//                       sense                LED
//                       board, ele,          board, ele
static constexpr SensorConfig SENSORS[] = {
        // Top sensor — the "Ding" (central low fundamental)
    { 1, 0,  1, 6  },  //0  top                  — sense C ELE0, LED C ELE6

        // Upper concentric ring — tone fields
    { 1, 3,  0, 5  },  //1  upper ring 0 (front) — sense C ELE3, LED A ELE5 (cross-board, was C ELE9)
    { 1, 1,  1, 7  },  //2  upper ring 1         — sense C ELE1, LED C ELE7
    { 0, 4,  0, 10 },  //3  upper ring 2         — sense A ELE4, LED A ELE10 (+ELE9 mirror)
    { 0, 2,  0, 8  },  //4  upper ring 3         — sense A ELE2, LED A ELE8
    { 1, 5,  1, 11 },  //5  upper ring 4         — sense C ELE5, LED C ELE11

    // Lower concentric ring — bass register
    { 1, 2,  1, 8  },  //6  lower ring 0 (front) — sense C ELE2, LED C ELE8
    { 0, 0,  0, 11 },  //7  lower ring 1         — sense A ELE0, LED A ELE11 (sense was A ELE5)
    { 0, 3,  0, 6  },  //8  lower ring 2         — sense A ELE3, LED A ELE6 (was A ELE9)
    { 0, 1,  0, 7  },  //9  lower ring 3         — sense A ELE1, LED A ELE7
    { 1, 4,  1, 10 },  //10 lower ring 4         — sense C ELE4, LED C ELE10 (+ELE9 mirror)
};

static constexpr uint8_t NUM_SENSORS =
    static_cast<uint8_t>(sizeof(SENSORS) / sizeof(SENSORS[0]));

// ── Sound set definition ─────────────────────────────────────────────────────
struct SoundSet {
    const char*  name;
    float        freqs[11];       // one per sensor, in SENSORS[] order
    short        waveformType;    // WAVEFORM_TRIANGLE, WAVEFORM_BANDLIMIT_SAWTOOTH, etc.
    float        subMix;          // sub-oscillator level (0.0–1.0)
    float        filterBaseHz;    // filter cutoff when hand is far (dark)
    float        filterMaxHz;     // filter cutoff when hand is close (bright)
    float        filterQ;         // resonance (0.7 = flat, 1.5 = warm, 3.0 = aggressive)
};

// ── Note frequencies ─────────────────────────────────────────────────────────
// Equal temperament, A4 = 440 Hz
namespace Note {
    // Octave 2
    static constexpr float C2  =  65.41f;
    static constexpr float D2  =  73.42f;
    static constexpr float Eb2 =  77.78f;
    static constexpr float E2  =  82.41f;
    static constexpr float F2  =  87.31f;
    static constexpr float Gb2 =  92.50f;
    static constexpr float G2  =  98.00f;
    static constexpr float Ab2 = 103.83f;
    static constexpr float A2  = 110.00f;
    static constexpr float Bb2 = 116.54f;
    static constexpr float B2  = 123.47f;

    // Octave 3
    static constexpr float C3  = 130.81f;
    static constexpr float Db3 = 138.59f;
    static constexpr float D3  = 146.83f;
    static constexpr float Eb3 = 155.56f;
    static constexpr float E3  = 164.81f;
    static constexpr float F3  = 174.61f;
    static constexpr float Gb3 = 185.00f;
    static constexpr float G3  = 196.00f;
    static constexpr float Ab3 = 207.65f;
    static constexpr float A3  = 220.00f;
    static constexpr float Bb3 = 233.08f;
    static constexpr float B3  = 246.94f;

    // Octave 4
    static constexpr float C4  = 261.63f;
    static constexpr float Db4 = 277.18f;
    static constexpr float D4  = 293.66f;
    static constexpr float Eb4 = 311.13f;
    static constexpr float E4  = 329.63f;
    static constexpr float F4  = 349.23f;
    static constexpr float Gb4 = 369.99f;
    static constexpr float G4  = 392.00f;
    static constexpr float Ab4 = 415.30f;
    static constexpr float A4  = 440.00f;
    static constexpr float Bb4 = 466.16f;
    static constexpr float B4  = 493.88f;

    // Octave 5
    static constexpr float C5  = 523.25f;
    static constexpr float D5  = 587.33f;
    static constexpr float E5  = 659.25f;
    static constexpr float F5  = 698.46f;
    static constexpr float G5  = 783.99f;
    static constexpr float A5  = 880.00f;
    static constexpr float Bb5 = 932.33f;
}

// ── Just intonation helpers (ratios relative to D3) ──────────────────────────
namespace JI {
    static constexpr float ROOT = 146.83f; // D3

    // Bass octave (one octave below D3)
    static constexpr float D2  = ROOT * 0.5f;
    static constexpr float F2  = ROOT * 0.5f * 6.0f / 5.0f;   // minor 3rd in bass
    static constexpr float A2  = ROOT * 0.5f * 3.0f / 2.0f;   // 5th in bass
    static constexpr float Bb2 = ROOT * 0.5f * 8.0f / 5.0f;   // minor 6th in bass
    static constexpr float C3  = ROOT * 8.0f / 9.0f;          // major 2nd below D3 (descending 9/8)

    // Main octave
    static constexpr float D3  = ROOT;
    static constexpr float E3  = ROOT * 9.0f / 8.0f;    // major 2nd
    static constexpr float F3  = ROOT * 6.0f / 5.0f;    // minor 3rd
    static constexpr float G3  = ROOT * 4.0f / 3.0f;    // perfect 4th
    static constexpr float A3  = ROOT * 3.0f / 2.0f;    // perfect 5th
    static constexpr float Bb3 = ROOT * 8.0f / 5.0f;    // minor 6th
    static constexpr float C4  = ROOT * 9.0f / 5.0f;    // minor 7th
    static constexpr float D4  = ROOT * 2.0f;            // octave
    static constexpr float E4  = ROOT * 9.0f / 4.0f;    // major 9th
    static constexpr float F4  = ROOT * 12.0f / 5.0f;   // minor 10th
    static constexpr float A4  = ROOT * 3.0f;            // perfect 12th
    static constexpr float D5  = ROOT * 4.0f;            // double octave
}

// ── Sound sets ───────────────────────────────────────────────────────────────
static constexpr uint8_t NUM_SOUND_SETS = 5;

static const SoundSet SOUND_SETS[NUM_SOUND_SETS] = {
    // ── 0: Hangdrum / D Kurd ─────────────────────────────────────────────────
    // The classic handpan scale — warm, meditative, immediately musical.
    {
        "D Kurd",
        //  Lower ring                          Upper ring                          Top
        { Note::D3, Note::A3, Note::Bb3, Note::C4, Note::D4,
          Note::E4, Note::F4, Note::G4,  Note::A4, Note::Bb4,
          Note::D5 },
        WAVEFORM_TRIANGLE,
        0.35f,          // subMix — warm bass layer
        200.0f,         // filterBaseHz — dark when far
        5000.0f,        // filterMaxHz — opens up when close
        1.5f,           // filterQ — warm resonance
    },

    // ── 1: Just Intonation (D root) ──────────────────────────────────────────
    // Pure harmonic ratios — beatless intervals, crystalline blend.
    {
        "Just Intonation",
        { JI::D3, JI::A3, JI::Bb3, JI::C4, JI::D4,
          JI::E4, JI::F4, JI::A4, JI::D5, JI::D5 * 5.0f/4.0f,
          JI::D5 },
        WAVEFORM_TRIANGLE,
        0.30f,
        250.0f,
        6000.0f,
        1.2f,           // lower Q to let the pure intervals shine
    },

    // ── 2: Chromatic ─────────────────────────────────────────────────────────
    // 11 consecutive semitones from D3 — every interval available.
    {
        "Chromatic",
        { Note::D3, Note::Eb3, Note::E3, Note::F3, Note::Gb3,
          Note::G3, Note::Ab3, Note::A3, Note::Bb3, Note::B3,
          Note::C4 },
        WAVEFORM_BANDLIMIT_SAWTOOTH,
        0.20f,          // less sub for dense intervals
        300.0f,
        7000.0f,
        1.8f,           // brighter, edgier character
    },

    // ── 3: Sad Vibes / D Dorian ──────────────────────────────────────────────
    // Minor feel with a bright 6th — melancholic but not dark.
    {
        "Sad Vibes",
        { Note::D3, Note::E3, Note::F3, Note::G3, Note::A3,
          Note::Bb3, Note::C4, Note::D4, Note::E4, Note::F4,
          Note::D5 },
        WAVEFORM_TRIANGLE,
        0.40f,          // deeper sub for weight
        180.0f,         // darker base
        4000.0f,        // doesn't open as far
        2.0f,           // more resonance for moodiness
    },

    // ── 4: Happy Vibes / C Major Pentatonic ──────────────────────────────────
    // Bright, joyful, impossible to play a wrong note.
    {
        "Happy Vibes",
        { Note::C3, Note::D3, Note::E3, Note::G3, Note::A3,
          Note::C4, Note::D4, Note::E4, Note::G4, Note::A4,
          Note::C5 },
        WAVEFORM_SINE,
        0.25f,
        350.0f,         // brighter starting point
        8000.0f,        // opens wide
        1.0f,           // clean, minimal resonance
    },
};

// ── Synth parameters ─────────────────────────────────────────────────────────
// Voice architecture
static constexpr float MAIN_OSC_AMPLITUDE = 1.0f;  // main osc always full (DC controls level)
static constexpr float SUB_OSC_AMPLITUDE  = 1.0f;  // sub osc always full (voiceMix controls blend)
static constexpr float VOICE_MAX_AMP      = 0.45f; // max DC level per voice (prevents clipping)

// Mixer gain structure
static constexpr float STAGE_GAIN  = 0.25f; // per-channel gain in stage mixers
static constexpr float MASTER_GAIN = 0.7f;  // per-channel gain in master mixer

// Amplitude envelope (DC ramp)
static constexpr float AMP_RAMP_MS = 8.0f;  // matches update interval — smooth linear ramp

// LFO — per-voice pitch drift for analog feel
static constexpr float LFO_RATE_HZ   = 1.25f;  // slow drift cycle
static constexpr float LFO_AMOUNT    = 0.003f;  // +/-0.3% pitch deviation (~5 cents at A4)
static constexpr float LFO_RATE_SPREAD = 0.15f; // each voice's LFO rate offset to prevent phase-locking

// Update timing
static constexpr uint32_t UPDATE_MS = 8; // ~125 Hz sensor update rate
