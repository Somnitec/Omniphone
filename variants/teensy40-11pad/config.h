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

// Set to 1 only if a round LCD/touch screen is attached. 0 skips all LCD and
// touchscreen init (and the GUI draws) — noticeably faster startup.
#define ENABLE_SCREEN 0

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

// MPR121 charge current / time (sensor gain). Controlled measurement
// (test/proximity_tuning) shows the electrode does NOT couple beyond ~1 cm at
// ANY CDC — this is a near-field/touch instrument, not a distance theremin
// (electrode geometry limit, not firmware). Within the usable zone
// (~1 cm → plastic → metal) CDC=16 gave the strongest, cleanest spread with
// idle noise still ~1 (1 cm/plastic ≈ 5/22 vs 3/14 at CDC=10). CDC=48 was
// over-saturated and felt dead — 16 is the measured sweet spot, not extreme.
static constexpr uint8_t SENSOR_CDC = 14; // 0–63 (try 10 to A/B; both 1 line)
static constexpr uint8_t SENSOR_CDT = 3;  // 0–7  (ESI stays 2 ms → flicker-free LEDs)

// ── Startup sound-set picker (hold a pad while booting) ──────────────────────
// Read with the MPR121 in CL=00 mode (baseline frozen at 0) so the raw
// filtered value reflects whether a finger is on the pad — bypassing the
// usual baseline contamination. A held pad reads MUCH lower than an idle one.
// REF pad is something the user is unlikely to be touching (idx 0 = top Ding).
// Threshold is in raw 10-bit ADC counts; finger on metal usually drops the
// reading by hundreds, so 80 has wide margin against noise.
static constexpr uint8_t STARTUP_SET_DEFAULT    = 5; // Bright Pentatonic
static constexpr uint8_t STARTUP_REF_PAD        = 0; // assumed-untouched reference
static constexpr int16_t STARTUP_HOLD_THRESHOLD = 80;

// Hold one of these pads at boot to load its sound set instead of the default.
// First match wins, so list order = priority. Add or remove rows freely — the
// firmware iterates the array, so any number of bindings works (up to all 11).
struct StartupBinding { uint8_t pad; uint8_t soundSet; };
static constexpr StartupBinding STARTUP_BINDINGS[] = {
    { 10, 6 }, //  pad 10 → Harmonic Minor
    {  6, 3 }, //  pad  6 → Sad Vibes
    {  1, 2 }, //  pad  1 → Chromatic
    // add more here as you decide them, e.g.:
    // {  2, 1 }, //  pad  2 → Just Intonation
    // {  3, 4 }, //  pad  3 → Happy Vibes
    // {  4, 0 }, //  pad  4 → D Kurd
};
static constexpr uint8_t NUM_STARTUP_BINDINGS =
    static_cast<uint8_t>(sizeof(STARTUP_BINDINGS) / sizeof(STARTUP_BINDINGS[0]));

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
    float        bellMix;         // per-set bell loudness in master ch 3 (0 = no bell)
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
        0.04f,          // bellMix — subtle, meditative
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
        0.06f,          // bellMix — modest crystalline sparkle
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
        0.03f,          // bellMix — keep low, the chromatic content is busy
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
        0.08f,          // bellMix — a little glint in the dark
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
        0.05,          // bellMix — joyful sparkle
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
        0.05f,          // bellMix — sparkly bright
    },

    // ── 6: Harmonic Minor (A) — default ──────────────────────────────────────
    // Melancholic, slightly Eastern. The augmented 2nd between F and G♯ is the
    // signature interval — placed at idx10 so it sings when the lower-ring
    // front is combined with the upper ring. Low Ding on A3, ascending tones
    // around the rings, doubled octaves for handpan-style thickness.
    {
        "Harmonic Minor",
        //  0=Ding  1     2      3      4      5         (top + upper ring)
        { Note::A3, Note::E4, Note::A4, Note::C5, Note::D5, Note::E5,
        //  6     7      8      9      10                (lower ring)
          Note::B3, Note::C4, Note::D4, Note::F4, Note::Ab4 },
        WAVEFORM_TRIANGLE,
        0.30f,          // warmer sub for the melancholic weight
        220.0f,         // darker base — broody when the hand is far
        5500.0f,        // opens to a soft, present body
        1.5f,           // warm resonance — gives the minor a hint of cry
        0.05,          // bellMix — most prominent: brightens the dark minor
    },
};

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
// Recal fires when EVERY pad's intensity is below IDLE_INTENSITY for
// IDLE_RECAL_MS in a row (and a sustained bell on any pad blocks it too —
// see `anyActive` in main). A pad sitting at saturated 1.0 (firm grip on the
// plastic) keeps anyActive true and prevents recal; metal contact keeps the
// bell sustaining, same effect. RECAL_COOLDOWN_MS prevents hammering.
static constexpr uint32_t IDLE_RECAL_MS      = 1000;
static constexpr uint32_t RECAL_COOLDOWN_MS  = 2000;
static constexpr float    IDLE_INTENSITY     = 0.02f; // any pad above this counts as active

// ── Baseline outlier rewrite ─────────────────────────────────────────────────
// After every baseline lock (startup + recal), pads on the same board should
// land at similar baseline values — they share the same chip, supply, and
// environment. A pad whose baseline ends up much lower than its neighbours
// almost always means a hand was hovering over it at lock time, contaminating
// its filtered reading. Symptom: that pad is "barely sensitive" afterwards
// because rawDelta = baseline − filtered clamps to 0 until you actually touch
// the metal. Fix: detect outliers (baseline > this many 10-bit counts below
// the per-board median) and rewrite them to the median.
static constexpr uint16_t BASELINE_OUTLIER_DELTA = 30;

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
