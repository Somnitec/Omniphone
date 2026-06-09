#pragma once
#include <stdint.h>

// ── Waveform IDs ──────────────────────────────────────────────────────────────
// TimbreSet.waveformType uses these IDs. The values follow the Teensy Audio
// Library's WAVEFORM_* numbering (the project's tables have always used it); the
// Mozzi engine maps each ID to a wavetable in tableForWaveform() — see
// sound_engine_mozzi.h. Add a new waveform by adding an ID here and a case there.
#define WAVEFORM_SINE                0
#define WAVEFORM_SAWTOOTH            1
#define WAVEFORM_SQUARE              2
#define WAVEFORM_TRIANGLE            3
#define WAVEFORM_BANDLIMIT_SAWTOOTH 11

// ─────────────────────────────────────────────────────────────────────────────
// Instrument configuration
//
// Edit this file to match your physical wiring and musical preferences.
// The rest of the firmware reads from SENSORS[], SCALES[]/TIMBRES[], and the
// synth parameter constants — no other files need to change when you
// remap pads, retune frequencies, or add new scales/timbres.
// ─────────────────────────────────────────────────────────────────────────────

// ── Board setup ──────────────────────────────────────────────────────────────
static constexpr uint8_t NUM_BOARDS        = 2;
static constexpr uint8_t BOARD_ADDRESSES[] = { 0x5A, 0x5C };

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

// Sense-electrode count per board. ELE_EN is a contiguous count from ELE0, so any
// LED pins must live above the last sense electrode — here all LEDs are on MCU
// GPIO (LED_GPIO), so every chip electrode is free to sense. MUST match SENSORS[]:
//   board 0 (0x5A, idx0): ELE0–ELE9 (10 sense) → pads 0–9
//   board 1 (0x5C, idx1): ELE0–ELE2 (3 sense)  → pads 10–12
static constexpr uint8_t SENSE_ELECTRODES[NUM_BOARDS] = { 10, 3 };

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
// 13 pad slots. Boards: 0 = 0x5A ("A"), 1 = 0x5C ("C"). All LEDs are driven from
// MCU GPIO (ledBoard = LED_GPIO, ledEle = a Pico GP pin), leaving every chip
// electrode free to sense.
//
// >>> CONFIRM AGAINST YOUR PCB <<< the electrode↔pad and LED-pin assignments are
// specific to this build's wiring. Reorder rows to match your physical ring
// positions, and pick LED GPIOs that don't share an RP2040 PWM slice (GPIOs that
// differ by 16 share a slice → their LEDs would track each other; that's why
// pads 10–12 use GP6/7/8 rather than GP12/3/2).
static constexpr SensorConfig SENSORS[] = {
    // Top sensor — the "Ding" (central low fundamental)
    { 0, 0,  LED_GPIO, 28 },  //0  top          — sense A ELE0, LED GP28

    // Upper concentric ring — tone fields
    { 0, 1,  LED_GPIO, 27 },  //1               — sense A ELE1, LED GP27
    { 0, 2,  LED_GPIO, 26 },  //2               — sense A ELE2, LED GP26
    { 0, 3,  LED_GPIO, 19 },  //3               — sense A ELE3, LED GP19
    { 0, 4,  LED_GPIO, 18 },  //4               — sense A ELE4, LED GP18
    { 0, 5,  LED_GPIO, 17 },  //5               — sense A ELE5, LED GP17
    { 0, 6,  LED_GPIO, 16 },  //6               — sense A ELE6, LED GP16

    // Lower concentric ring — bass register
    { 0, 7,  LED_GPIO, 15 },  //7               — sense A ELE7, LED GP15
    { 0, 8,  LED_GPIO, 14 },  //8               — sense A ELE8, LED GP14
    { 0, 9,  LED_GPIO, 13 },  //9               — sense A ELE9, LED GP13
    { 1, 0,  LED_GPIO,  6 },  //10              — sense C ELE0, LED GP6
    { 1, 1,  LED_GPIO,  7 },  //11              — sense C ELE1, LED GP7
    { 1, 2,  LED_GPIO,  8 },  //12              — sense C ELE2, LED GP8
};

static constexpr uint8_t NUM_SENSORS =
    static_cast<uint8_t>(sizeof(SENSORS) / sizeof(SENSORS[0]));

// Sizes the per-pad freqs[] in ScaleSet. Equals NUM_SENSORS for this 13-pad build.
static constexpr uint8_t MAX_PADS = 13;

// ── Scale & timbre definitions ───────────────────────────────────────────────
// Scale (which notes) and timbre (how they sound) are now INDEPENDENT. At boot,
// holding an EVEN pad picks the scale, holding an ODD pad picks the timbre; the
// played sound is scale × timbre. See SCALES[] / TIMBRES[] and the pad maps below.
struct ScaleSet {
    const char*  name;
    float        freqs[MAX_PADS]; // one per sensor, in SENSORS[] order (pad 0 = top/root)
};

struct TimbreSet {
    const char*  name;
    short        waveformType;    // WAVEFORM_SINE / TRIANGLE / SQUARE / SAWTOOTH(+BANDLIMIT)
    float        secondRatio;     // 2nd osc freq ÷ main: 0.5 = sub-octave, ~1.006 = detune, 1 = unison
    float        secondMix;       // 2nd osc level 0.0–1.0 (0 = off → 2nd osc silent)
    float        filterBaseHz;    // LP cutoff when hand is far (dark). base==max → static (no sweep)
    float        filterMaxHz;     // LP cutoff when hand is close (bright)
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
    static constexpr float G4  = ROOT * 8.0f / 3.0f;    // perfect 4th, two octaves up
    static constexpr float A4  = ROOT * 3.0f;            // perfect 12th
    static constexpr float D5  = ROOT * 4.0f;            // double octave
}

// ── SCALES (selected by holding an EVEN pad at boot) ─────────────────────────
// Each scale lists 13 distinct notes, voiced low for a bass instrument. Pad 0 =
// top/root. Layout follows the physical rings so the natural touch-combinations
// are consonant:
//   • Pentatonic: ascending — a pentatonic has no clashing intervals, so EVERY
//     pad combination is consonant.
//   • Stacked-thirds scales (Cmaj7-JI / Dorian / harmonic-minor): any 3 adjacent
//     pads (1+2+3, 2+3+4 …) form a triad and pad 0 + the ring skips (0+1+3, 0+3+5
//     …) stay consonant. Octaves are folded into the bass register (folded =
//     inverted triads; "wide" = unfolded, spanning ~3 octaves).
//   • Chromatic is left ascending — it exists to make every interval available.
//
// Pad → scale:  2→Pentatonic  4→Cmaj7-JI  6→Overtones  8→Wide-Dorian
//               10→Chromatic   12→Harmonic-minor
static constexpr uint8_t NUM_SCALES = 6;

static const ScaleSet SCALES[NUM_SCALES] = {
    // 0 (pad 2) — C major pentatonic, ascending. Every combination consonant.
    { "Pentatonic",
      { Note::C2, Note::D2, Note::E2, Note::G2, Note::A2, Note::C3, Note::D3,
        Note::E3, Note::G3, Note::A3, Note::C4, Note::D4, Note::E4 } },

    // 1 (pad 4) — Just-intonation C major in STACKED THIRDS, so triples spell
    // C/Em/G/Bø7 fragments → a rich "Cmaj7" colour. Pure ratios = beatless.
    { "Cmaj7 Just Intonation",
      { 65.41f, 81.76f, 98.11f, 122.64f, 147.16f, 174.41f, 218.02f,
        261.63f, 163.51f, 196.22f, 245.27f, 294.33f, 348.83f } },

    // 2 (pad 6) — Overtone series: harmonics 1–13 of C2 (65.41 Hz). All notes
    // share one fundamental, so every combination fuses consonantly (and a touch
    // microtonal up top — the natural 7th/11th/13th).
    { "Overtones",
      { 65.41f, 130.81f, 196.22f, 261.63f, 327.03f, 392.44f, 457.84f,
        523.25f, 588.65f, 654.06f, 719.46f, 784.87f, 850.27f } },

    // 3 (pad 8) — D Dorian stacked thirds UNFOLDED → wide, open voicing spanning
    // ~3 octaves (D2..G5). Same triad-per-triple property, just spacious.
    { "Wide Dorian",
      { Note::D2, Note::F2, Note::A2, Note::C3, Note::E3, Note::G3, Note::B3,
        Note::D4, Note::F4, Note::A4, Note::C5, Note::E5, Note::G5 } },

    // 4 (pad 10) — 13 chromatic semitones from D2. Deliberately ascending.
    { "Chromatic",
      { Note::D2, Note::Eb2, Note::E2, Note::F2, Note::Gb2, Note::G2, Note::Ab2,
        Note::A2, Note::Bb2, Note::B2, Note::C3, Note::Db3, Note::D3 } },

    // 5 (pad 12) — A harmonic minor stacked thirds (A C E G# B D F …). The aug/dim
    // triads give the exotic colour. Octaves folded low.
    { "Harmonic Minor",
      { Note::A2, Note::C3, Note::E3, Note::Ab3, Note::B3, Note::D4, Note::F4,
        Note::A4, Note::C4, Note::E4, Note::Ab4, Note::B4, Note::D3 } },
};

// ── TIMBRES (selected by holding an ODD pad at boot) ─────────────────────────
// waveform · 2nd-osc(ratio,mix) · filter(baseHz,maxHz,Q). When baseHz==maxHz the
// filter is static (no proximity sweep); secondMix 0 disables the 2nd oscillator.
//
// Pad → timbre:  1→Sines  3→Triangles  5→Supersaw  7→Video-game  9→Moog  11→Juno
static constexpr uint8_t NUM_TIMBRES = 6;

static const TimbreSet TIMBRES[NUM_TIMBRES] = {
    //  name             waveform                  ratio  mix   baseHz   maxHz    Q
    { "Sines",        WAVEFORM_SINE,                1.000f, 0.0f, 9000.f, 9000.f, 0.7f }, // pad 1 — pure
    { "Triangles",    WAVEFORM_TRIANGLE,            0.500f, 0.3f, 9000.f, 9000.f, 0.7f }, // pad 3 — + sub octave
    { "Supersaw",     WAVEFORM_BANDLIMIT_SAWTOOTH,  1.008f, 0.7f, 1200.f, 9000.f, 0.9f }, // pad 5 — detuned fat saws
    { "Video Game",   WAVEFORM_SQUARE,              1.000f, 0.0f, 9000.f, 9000.f, 0.7f }, // pad 7 — chiptune square
    { "Moog",         WAVEFORM_SAWTOOTH,            0.500f, 0.4f,  220.f, 4500.f, 2.3f }, // pad 9 — resonant LP sweep + sub
    { "Juno",         WAVEFORM_BANDLIMIT_SAWTOOTH,  1.006f, 0.5f,  400.f, 5000.f, 1.0f }, // pad 11 — detune + gentle filter
};

// Defaults when no pad is held at boot: foolproof + clean.
static constexpr uint8_t DEFAULT_SCALE  = 0; // Pentatonic
static constexpr uint8_t DEFAULT_TIMBRE = 0; // Sines

// ── Startup pad-hold selection ────────────────────────────────────────────────
// At boot the baseline is frozen (CL=00) so a held finger is visible as a sharp
// DROP in filteredData vs an untouched reference pad. Hold an even pad → scale,
// an odd pad → timbre. Pad 0 (top) is the reference — leave it untouched.
static constexpr uint8_t STARTUP_REF_PAD        = 0;
static constexpr int16_t STARTUP_HOLD_THRESHOLD = 40; // filtered-count drop = "held"

// Even pad ≥2 → scale index; else 0xFF. Odd pad → timbre index; else 0xFF.
static inline uint8_t scaleForPad(uint8_t pad) {
    return (pad >= 2 && (pad & 1u) == 0u && (uint8_t)(pad / 2u - 1u) < NUM_SCALES)
               ? (uint8_t)(pad / 2u - 1u) : 0xFF;
}
static inline uint8_t timbreForPad(uint8_t pad) {
    return ((pad & 1u) && (uint8_t)((pad - 1u) / 2u) < NUM_TIMBRES)
               ? (uint8_t)((pad - 1u) / 2u) : 0xFF;
}

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
                                                          // proximity stays well below it
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

// ── Proximity → volume mapping ───────────────────────────────────────────────
// intensity = (fast − DEADBAND) / (PROX_MAX − DEADBAND), clamped 0..1, then
// smoothed in main_pico.cpp (PROX_SMOOTH_K) before it drives voice amplitude.
// These pads are near-field/touch only (see SENSOR_CDC): from a CDC≈14–16 capture
// idle noise ≈ 1, ~1 cm ≈ 5, resting on the shell ≈ 22 — so the playable swell
// lives between a light near-touch and resting on the surface. DEADBAND sits just
// above idle noise; PROX_MAX ≈ firm contact so the note is full as you rest on it.
static constexpr float PROX_DEADBAND = 4.0f;
static constexpr float PROX_MAX      = 18.0f;
