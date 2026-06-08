#pragma once
#include <stdint.h>

// ── Waveform tags ─────────────────────────────────────────────────────────────
// SoundSet.waveformType uses the Teensy Audio Library's WAVEFORM_* numbering so
// the Teensy build can pass it straight into AudioSynthWaveform::begin(). On the
// non-Teensy builds (Pico / ESP32-S3, which use Mozzi instead of the Teensy
// Audio Library) <Audio.h> doesn't exist, so we define the same tags ourselves
// and the Mozzi engine maps them to wavetables. Keep the values in sync with
// Teensy's synth_waveform.h.
#if defined(OMNIPHONE_PICO) || defined(OMNIPHONE_ESP32S3)
  #ifndef WAVEFORM_SINE
    #define WAVEFORM_SINE                0
    #define WAVEFORM_SAWTOOTH            1
    #define WAVEFORM_SQUARE              2
    #define WAVEFORM_TRIANGLE            3
    #define WAVEFORM_SAWTOOTH_REVERSE    6
    #define WAVEFORM_BANDLIMIT_SAWTOOTH 11
  #endif
#else
  #include <Audio.h> // Teensy: real WAVEFORM_* constants
#endif

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

// Set to 1 only if a round LCD/touch screen is attached. 0 skips all LCD and
// touchscreen init (and the GUI draws) — noticeably faster startup.
#define ENABLE_SCREEN 0

// ── Sensor descriptor ────────────────────────────────────────────────────────
static constexpr uint8_t NO_PIN = 0xFF; // disabled pad / no LED

// ledBoard sentinel: the LED is wired straight to a microcontroller GPIO pin
// (PWM), not to an MPR121 LED driver. Use this when the chip's LED pins run out
// (13 pads need 13 LEDs but two MPR121s only free up ~11 LED pins once 13
// electrodes are spent on sensing). Set ledBoard = LED_GPIO and put the GPIO
// pin number in ledEle. On the Pico build these are RP2040 GPIO pins.
static constexpr uint8_t LED_GPIO = 0xFE;

struct SensorConfig {
    uint8_t boardIndex; // SENSE board: index into BOARD_ADDRESSES[]
    uint8_t electrode;  // ELE0–ELE6 touch input, or NO_PIN if this pad is disabled
    uint8_t ledBoard;   // LED board index, or LED_GPIO for a direct MCU GPIO LED
    uint8_t ledEle;     // MPR121 LED pin (ELE6–ELE11), OR an MCU pin # when
                        // ledBoard == LED_GPIO, or NO_PIN if no LED
};

// Touch electrodes per board, indexed by board. ELE_EN is a contiguous count
// from ELE0, so LED pins can only live above the last sense electrode.
#if defined(OMNIPHONE_PICO) || defined(OMNIPHONE_ESP32S3)
// ── 13-pad split (Pico / ESP32-S3) ───────────────────────────────────────────
//   A (0x5A, idx0): ELE0–ELE5 (6 sense)  → LED pins ELE6–ELE11 (6 free)
//   C (0x5C, idx1): ELE0–ELE6 (7 sense)  → LED pins ELE7–ELE11 (5 free)
// 13 sense electrodes total; 11 chip LED pins free → the remaining 2 LEDs live
// on MCU GPIO pins (LED_GPIO). See the 13-pad SENSORS[] table below.
static constexpr uint8_t SENSE_ELECTRODES[NUM_BOARDS] = { 6, 7 };
#else
// ── 11-pad legacy split (Teensy) ──────────────────────────────────────────────
//   A (0x5A): ELE0–ELE4 (5)  — ELE5 freed as a GPIO LED pin (idx1's lamp)
//   C (0x5C): ELE0–ELE5 (6)  — idx5 senses on ELE5, LEDs limited to ELE6–ELE11
static constexpr uint8_t SENSE_ELECTRODES[NUM_BOARDS] = { 5, 6 };
#endif

// MPR121 charge current / time (sensor gain). Controlled measurement
// (test/proximity_tuning) shows the electrode does NOT couple beyond ~1 cm at
// ANY CDC — this is a near-field/touch instrument, not a distance theremin
// (electrode geometry limit, not firmware). Within the usable zone
// (~1 cm → plastic → metal) CDC=16 gave the strongest, cleanest spread with
// idle noise still ~1 (1 cm/plastic ≈ 5/22 vs 3/14 at CDC=10). CDC=48 was
// over-saturated and felt dead — 16 is the measured sweet spot, not extreme.
static constexpr uint8_t SENSOR_CDC = 14; // 0–63 (try 10 to A/B; both 1 line)
static constexpr uint8_t SENSOR_CDT = 3;  // 0–7  (ESI stays 2 ms → flicker-free LEDs)

// ── Physical sensor layout ────────────────────────────────────────────────────
// Hangdrum "ding" layout: TOP is the central low fundamental, the upper ring is
// the melodic tone fields ascending around it, the lower ring is bass voicings.
//                       sense                LED
//                       board, ele,          board, ele
#if defined(OMNIPHONE_PICO) || defined(OMNIPHONE_ESP32S3)
// ── 13-pad layout (Pico / ESP32-S3) ──────────────────────────────────────────
// 13 pad slots. Boards: 0 = 0x5A ("A"), 1 = 0x5C ("C").
//   • Board C (idx1): top + 6-pad upper ring  → sense ELE0–ELE6 (7 electrodes)
//   • Board A (idx0): 6-pad lower/bass ring    → sense ELE0–ELE5 (6 electrodes)
// LED budget: A frees ELE6–ELE11 (6 pins), C frees ELE7–ELE11 (5 pins) = 11 chip
// LED pins for 13 LEDs, so the top pad and the last upper-ring pad drive their
// LEDs from MCU GPIO instead (LED_GPIO + a Pico GP pin).
//
// >>> CONFIRM AGAINST YOUR PCB <<< electrode↔pad and LED assignments below are a
// clean default for a 6 (A) + 7 (C) sense split; reorder rows to match the
// physical ring positions. ELE9/ELE10 LED drivers were flaky on the old MPR121
// modules — if yours are too, move those LEDs (idx3,idx4,idx10,idx11) to GPIO.
static constexpr SensorConfig SENSORS[] = {
        // Top sensor — the "Ding" (central low fundamental), board C
    { 0, 0,  LED_GPIO, 28 }, //0  top              — sense C ELE0, LED GPIO pin 14

        // Upper concentric ring — tone fields (board C)
    { 0, 1,  LED_GPIO, 27 },        //1  upper ring 0      — sense C ELE1, LED C ELE7
    { 0, 3,  LED_GPIO, 8  },        //2  upper ring 1      — sense C ELE2, LED C ELE8
    { 0, 5,  LED_GPIO, 9  },        //3  upper ring 2      — sense C ELE3, LED C ELE9
    { 0, 7,  LED_GPIO, 10 },        //4  upper ring 3      — sense C ELE4, LED C ELE10
    { 0, 9,  LED_GPIO, 1  },        //5  upper ring 4      — LED GP1  (was GP11: collided with GP27)
    { 1, 1,  LED_GPIO, 15 }, //6  upper ring 5      — sense C ELE6, LED GPIO pin 15

        // Lower concentric ring — bass register (board A)
    { 0, 2,  LED_GPIO, 6  },        //7  lower ring 0      — sense A ELE0, LED A ELE6
    { 0, 4,  LED_GPIO, 7  },        //8  lower ring 1      — sense A ELE1, LED A ELE7
    { 0, 6,  LED_GPIO, 2  },        //9  lower ring 2      — LED GP2  (was GP8: duplicate of idx2)
    { 0, 3,  LED_GPIO, 3  },        //10 lower ring 3      — LED GP3  (was GP9: duplicate of idx3)
    { 1, 0,  LED_GPIO, 0 },        //11 lower ring 4      — sense A ELE4, LED A ELE10
    { 1, 2,  LED_GPIO, 13 },        //12 lower ring 5      — LED GP13 (was GP11: duplicate/collision)
};
#else
// ── 11-pad legacy layout (Teensy, post ELE9/ELE10 rework) ─────────────────────
// 11 pad slots. Boards: 0 = 0x5A ("A"), 1 = 0x5C ("C").
// LED pin AND LED board are explicit per pad. Rework summary:
//   • A: the LED that was on A-ELE9 is now on A-ELE6   (idx8)
//   • C: the LED that was on C-ELE9 is now on A-ELE5   (idx1, cross-board!)
//   • A: idx7's touch electrode moved A-ELE5 → A-ELE0  (freed A-ELE5 for idx1's LED)
//   • idx5 (upper ring 4) still senses on C-ELE5; its LED stays C-ELE11
//   • ELE10 LEDs (idx3 A, idx10 C) only light if ELE9 is driven identically;
//     nothing is wired to ELE9 — handled in main (mirror ELE9 ← ELE10 per board).
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
#endif

static constexpr uint8_t NUM_SENSORS =
    static_cast<uint8_t>(sizeof(SENSORS) / sizeof(SENSORS[0]));

// Largest pad count any layout uses — sizes the per-pad arrays in SoundSet so
// the same table is valid for both the 11-pad (Teensy) and 13-pad builds.
static constexpr uint8_t MAX_PADS = 13;

// ── Sound set definition ─────────────────────────────────────────────────────
struct SoundSet {
    const char*  name;
    float        freqs[MAX_PADS]; // one per sensor, in SENSORS[] order. Sets that
                                  // list fewer than MAX_PADS notes leave the
                                  // trailing pads silent (zero-filled).
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
// NOTE: sets 0–5 below predate the 13-pad build and only list 11 notes; on the
// 13-pad (Pico/ESP32-S3) layout their last two pads stay silent (zero-filled).
// Set 6 "Juno Harmonic Minor" lists all 13 and is the 13-pad default.
static constexpr uint8_t NUM_SOUND_SETS = 7;

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

    // ── 5: Bright Pentatonic (default) ───────────────────────────────────────
    // Based on set 4 (sine, C major pentatonic) but voiced an octave up and
    // re-mapped per pad so every touch-combination is consonant. Indices are
    // serial-monitor pad numbers; the chord groups requested all resolve
    // because a major pentatonic has no clashing intervals. {8,4,3} lands on
    // A-minor (A C E) for a melancholic contrast among the bright voicings.
    {
        "Bright Pentatonic",
        //  0=top   1     2     3     4     5         (upper ring / root)
        { Note::C3, Note::E4, Note::G4, Note::A4, Note::C4, Note::D4,
        //  6     7     8     9     10                (lower ring)
          Note::C5, Note::D5, Note::E5, Note::G5, Note::A5 },
        WAVEFORM_SINE,
        0.20f,          // light sub — keep it clear, not boomy
        400.0f,         // bright base
        8500.0f,        // opens wide and shimmery
        0.9f,           // clean, minimal resonance (no clip colouring)
    },

    // ── 6: Juno Harmonic Minor (13-pad default) ──────────────────────────────
    // A harmonic minor (A B C D E F G#) voiced ascending across all 13 pads —
    // two octaves, top pad = low root. The "Juno" timbre: deep saw pad, heavy
    // sub, low-ish round filter and gentle resonance (Men-I-Trust dreamy).
    {
        "Juno Harmonic Minor",
        //  0=top  1     2     3     4     5     6        (top + upper ring)
        { Note::A3,  Note::B3,  Note::C4,  Note::D4,  Note::E4,  Note::F4, Note::Ab4,
        //  7     8     9     10    11    12               (lower ring)
          Note::A4,  Note::B4,  Note::C5,  Note::D5,  Note::E5,  Note::F5 },
        WAVEFORM_BANDLIMIT_SAWTOOTH,
        0.42f,          // heavy sub — deep Juno weight
        220.0f,         // low/round base cutoff
        4200.0f,        // opens gently, never harsh
        1.1f,           // soft resonance
    },
};

// Default sound set loaded at boot. The 13-pad build starts on Juno Harmonic
// Minor (set 6); the legacy 11-pad Teensy build keeps Bright Pentatonic (set 5).
#if defined(OMNIPHONE_PICO) || defined(OMNIPHONE_ESP32S3)
static constexpr uint8_t DEFAULT_SOUND_SET = 6;
#else
static constexpr uint8_t DEFAULT_SOUND_SET = 5;
#endif

// ── Synth parameters ─────────────────────────────────────────────────────────
// Voice architecture
// 0.6 (not 1.0): leaves headroom so main+sub and the resonant filter boost
// don't clip before the dcAmp stage — clipping is what made sines sound buzzy.
static constexpr float MAIN_OSC_AMPLITUDE = 0.6f;  // DC stage still sets final level
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

// Bell — short metallic transient fired on a metal-contact (touch) event,
// i.e. when the delta spikes after the proximity volume is already maxed.
static constexpr float BELL_MIX        = 0.01f;  // bell level in the master mixer (ch 3)
static constexpr float BELL_AMP        = 0.55f;  // bell oscillator peak amplitude
static constexpr float BELL_PARTIAL    = 2.76f;  // inharmonic 2nd partial → bell timbre
static constexpr float BELL_OCTAVES    = 2.0f;   // bell pitch = note × this (sparkle above)
static constexpr float BELL_FLOOR      = 0.01f;  // min bell level (softest tap)
// ADSR: attack → decay to the sustain level (held while the pad is touched) →
// release when the finger lifts. BELL_SUSTAIN = 0 makes it a pure pluck again.
// NOTE: attack is intentionally ~the strike window so the velocity-derived
// level locks in smoothly while the envelope is still ramping up (no click).
static constexpr float BELL_ATTACK_MS   = 20.0f;
static constexpr float BELL_DECAY_MS    = 280.0f;
static constexpr float BELL_SUSTAIN     = 0.35f;  // ring level while held (0–1 of peak)
static constexpr float BELL_RELEASE_MS  = 400.0f; // tail after the finger lifts
static constexpr float BELL_HOLD_RELEASE = 0.15f; // proximity below this = "lifted" → note-off

// Bell pressure aftertouch — while a pad is HELD, the live contact strength
// (raw fast value, not the saturated 0–1 intensity) modulates the bell's
// loudness AND a per-voice low-pass filter, like polyphonic aftertouch.
// From CDC=16 capture: light contact ≈ 22, firm press on metal ≈ 800.
static constexpr float BELL_AFTER_MIN   = 30.0f;   // fast at/below → quiet & dark
static constexpr float BELL_AFTER_MAX   = 600.0f;  // fast at/above → loud & bright
static constexpr float BELL_FILT_MIN_HZ = 350.0f;  // cutoff at min pressure
static constexpr float BELL_FILT_MAX_HZ = 6500.0f; // cutoff at max pressure
static constexpr float BELL_FILT_Q      = 1.1f;    // bell filter resonance
static constexpr float BELL_PRESS_SMOOTH = 0.20f;  // 1-pole smoothing on pressure (anti-zipper)

// Bell dynamics — driven by CONTACT VELOCITY (how fast the hand was moving
// when it hit the metal). A capacitive pad can't sense press force; approach
// speed is the only real expressive axis (gentle vs committed strike).
//
//   strike = clamp01( (peakVel − VEL_MIN) / (VEL_MAX − VEL_MIN) )
//   level  = BELL_AMP × (BELL_FLOOR + (1−BELL_FLOOR) × (strike×GAIN)^CURVE)
//
// Starting point from captured taps (pad0, CDC10/CDT3): soft peakVel ≈ 63–80,
// firm ≈ 86–99. Re-run test/strike_tuning per pad if pads differ a lot.
//   VEL_MIN  velocity mapped to silence-floor (just below softest tap)
//   VEL_MAX  velocity mapped to full volume   (≈ a firm strike)
//   CURVE>1 expander (only committed hits get loud) · <1 more uniform
//   GAIN scales strike before the curve (result clamped to 1)
static constexpr float BELL_VEL_MIN     = 60.0f;  // CDC=10 capture: soft ≈ 63–81,
static constexpr float BELL_VEL_MAX     = 100.0f; //   firm ≈ 86–99 (weak split — expected)
static constexpr float BELL_CURVE       = 2.0f;
static constexpr float BELL_STRIKE_GAIN = 1.0f;
static constexpr uint32_t BELL_STRIKE_WIN_MS = 25; // peak-velocity capture window

// ── Touch (metal-contact) trigger — algorithm lives in proximity_engine.h ─────
// Tune here if triggering is inconsistent or repeated taps get missed:
//   JUMP_THRESHOLD  contact spike needed to fire. LOWER = more sensitive and
//                   more consistent, but too low = false fires on a fast
//                   no-contact approach.
//   RELEASE_RATIO   re-arms once the spike falls below ratio×threshold. HIGHER
//                   (→1) re-arms sooner → fast repeated taps register better;
//                   too high can double-fire one strike.
//   COOLDOWN_MS     minimum ms between events per pad (edge debounce).
static constexpr float    TOUCH_JUMP_THRESHOLD = 250.0f; // metal contact jumps ≫ this
                                                          // (700+ at CDC=16); near-field
                                                          // ≪ this → bell only on real touch
static constexpr float    TOUCH_RELEASE_RATIO  = 0.5f;
static constexpr uint32_t TOUCH_COOLDOWN_MS    = 10;
// Consecutive frames the jump must stay over-threshold before a touch fires.
// 2 rejects single-sample electrical glitches (the idle "blip") at the cost of
// one extra 8 ms frame of latency. 1 = original instant behaviour.
static constexpr uint8_t  TOUCH_CONFIRM_FRAMES = 2;
// Same idea for the proximity intensity: hold at 0 until the fast EMA has
// stayed above PROX_DEADBAND for this many frames → kills the brief 0.05-ish
// monitor blips. ~8 ms of added onset latency per extra frame.
static constexpr uint8_t  PROX_CONFIRM_FRAMES  = 2;

// ── Proximity → volume mapping (intensity = (fast−deadband)/(proxMax−deadband))
// From the CDC=16 capture: idle noise ≈ 1, ~1 cm ≈ 5, on plastic ≈ 22. So the
// playable swell lives between a light near-touch and resting on the shell;
// deadband sits just above idle noise, proxMax ≈ firm-plastic so the sound is
// full by the time you rest on it (metal contact then adds the bell).
static constexpr float PROX_DEADBAND = 4.0f;  // ↑ from 2: rejects small idle transients
                                              // (1 cm ≈ 5 at CDC=16 still registers)
static constexpr float PROX_MAX      = 18.0f;

// ── Idle baseline recalibration ──────────────────────────────────────────────
// If nothing is happening for IDLE_RECAL_MS, force a full MPR121 baseline
// reload + re-seed the EMAs to kill drift-induced phantom blips. Skipped if a
// recal already ran within RECAL_COOLDOWN_MS (avoid hammering).
static constexpr uint32_t IDLE_RECAL_MS      = 5000;
static constexpr uint32_t RECAL_COOLDOWN_MS  = 10000;
static constexpr float    IDLE_INTENSITY     = 0.02f; // any pad above this counts as active

// ── MPE (USB-MIDI) output ────────────────────────────────────────────────────
// Each pad gets its own member channel for polyphonic aftertouch (channel
// pressure). Notes are sent at the nearest semitone — DAW handles any further
// tuning. The Teensy keeps producing audio at the same time (dual mode);
// unplug the audio jack if you only want the MIDI.
static constexpr bool     MPE_ENABLE              = true;
static constexpr uint8_t  MPE_MASTER_CH           = 1;   // 1-indexed MIDI channel
static constexpr uint8_t  MPE_MEMBER_BASE_CH      = 2;   // pad 0 → ch 2, pad 10 → ch 12
static constexpr uint32_t MPE_PRESSURE_THROTTLE_MS = 20; // ≤50 Hz per pad
static constexpr uint8_t  MPE_PRESSURE_MIN_DELTA  = 2;   // skip if change < this (0–127)

// Update timing
static constexpr uint32_t UPDATE_MS = 8; // ~125 Hz sensor update rate
