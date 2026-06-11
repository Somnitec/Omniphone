#pragma once
#include <stdint.h>
#include <Audio.h> // for WAVEFORM_* constants

// ─────────────────────────────────────────────────────────────────────────────
// Instrument configuration
//
// Edit this file to match your physical wiring and musical preferences.
// The rest of the firmware reads from SENSORS[], SCALE_SETS[], TIMBRE_SETS[],
// and the synth parameter constants — no other files need to change when you
// remap pads, retune frequencies, or add new scales/timbres.
// ─────────────────────────────────────────────────────────────────────────────

// ── Board setup ──────────────────────────────────────────────────────────────
static constexpr uint8_t NUM_BOARDS        = 2;
static constexpr uint8_t BOARD_ADDRESSES[] = { 0x5D, 0x5B, 0x00, 0x00 };

// Set to 1 only if a round LCD/touch screen is attached. 0 skips all LCD and
// touchscreen init (and the GUI draws) — noticeably faster startup.
// This is the 12-pad + screen build, so it ships ON. (All screen code is
// isolated in display.h; flipping this to 0 turns every display*() call into
// a no-op.)
#define ENABLE_SCREEN 1

// Serial diagnostics. The periodic 10 Hz readout streams over USB constantly,
// and that USB bus activity can couple a faint high-pitched whine into the
// PCM5102A audio. Keep at 0 for silent running; set to 1 only when you need the
// live per-pad / CPU readout. (Startup banner + sound-change lines still print.)
#define SERIAL_DEBUG 0

// ── Sensor descriptor ────────────────────────────────────────────────────────
static constexpr uint8_t NO_PIN = 0xFF; // disabled pad / no LED

// ledBoard sentinel: the LED is wired straight to a Teensy GPIO pin (PWM), not
// to an MPR121 LED driver. Use this to escape the chip's flaky ELE9/ELE10
// drivers — set ledBoard = LED_TEENSY and put the Teensy pin number in ledEle.
static constexpr uint8_t LED_TEENSY = 0xFE;

struct SensorConfig {
    uint8_t boardIndex; // SENSE board: index into BOARD_ADDRESSES[]
    uint8_t electrode;  // ELE0–ELE5 touch input, or NO_PIN if this pad is disabled
    uint8_t ledBoard;   // LED board index, or LED_TEENSY for a direct Teensy GPIO LED
    uint8_t ledEle;     // MPR121 LED pin (ELE6–ELE11), OR a Teensy pin # when
                        // ledBoard == LED_TEENSY, or NO_PIN if no LED
};

// Touch electrodes per board, indexed by board. ELE_EN is a contiguous count
// from ELE0, so to sense ELE6 the chip must enable ELE0–ELE6 (7 electrodes).
//   • Board A (idx 0, 0x5D) = 7: senses ELE0–ELE6. The bust A-ELE0 is
//     abandoned; pad 6 relocated to A-ELE6. A-ELE6 is a sense electrode, so the
//     chip won't drive an LED on it → pad 6's LED is on Teensy GPIO instead.
//   • Board C (idx 1, 0x5B) = 6: senses ELE0–ELE5 only, leaving ELE6 free as a
//     GPIO/LED pin — that's where pad 0's LED lives.
// GOTCHA (this exact bug bit us): the MPR121 ignores GPIO/LED on any
// sense-enabled electrode. If you ever bump board C to 7, pad 0's LED on
// C-ELE6 goes dark — move that LED to Teensy first.
static constexpr uint8_t SENSE_ELECTRODES[NUM_BOARDS] = { 7, 6 };

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
// Defaults used on a fresh chip (no stored selection) — scale and timbre are
// remembered independently across power-off.
static constexpr uint8_t STARTUP_SCALE_DEFAULT  = 6; // Bright Pentatonic
static constexpr uint8_t STARTUP_TIMBRE_DEFAULT = 5; // Shimmer
static constexpr uint8_t STARTUP_REF_PAD        = 0; // assumed-untouched reference
static constexpr int16_t STARTUP_HOLD_THRESHOLD = 80;

// Hold one of these pads at boot to load its SCALE instead of the default
// (overrides the stored scale too). First match wins, so list order = priority.
// Timbre is not pad-pickable at boot — swipe up/down to change it live.
struct StartupBinding { uint8_t pad; uint8_t scale; };
static constexpr StartupBinding STARTUP_BINDINGS[] = {
    { 10, 7 }, //  pad 10 → Harmonic Minor
    {  6, 4 }, //  pad  6 → Sad Vibes
    {  1, 3 }, //  pad  1 → Chromatic
    // add more here as you decide them, e.g.:
    // {  2, 1 }, //  pad  2 → Just Intonation
    // {  3, 4 }, //  pad  3 → Happy Vibes
    // {  4, 0 }, //  pad  4 → D Kurd
};
static constexpr uint8_t NUM_STARTUP_BINDINGS =
    static_cast<uint8_t>(sizeof(STARTUP_BINDINGS) / sizeof(STARTUP_BINDINGS[0]));

// ── Physical sensor layout (12-pad symmetric build) ──────────────────────────
// 12 pad slots. Boards: 0 = 0x5A ("A"), 1 = 0x5C ("C").
// Clean symmetric wiring — no cross-board LEDs, no ELE9/ELE10 mirror hack:
//   • Board C (1) carries the top "Ding" + the 5-pad upper ring (idx 0–5).
//   • Board A (0) carries the 6-pad lower/bass ring               (idx 6–11).
//   • Each pad's LED is on the SAME board, ELE = sense electrode + 6
//     (sense ELE0→LED ELE6 … sense ELE5→LED ELE11).
//
// ELE9/ELE10 DRIVER FAULT: these MPR121 boards drive ELE9 and ELE10 LEDs
// unreliably (the old mirror hack only worked when ELE9 was unused — it can't
// when ELE9 and ELE10 each carry an independent LED, as a 6/6 split needs). So
// the four LEDs that would land on ELE9/ELE10 — pads 3, 4, 9, 10 — are driven
// straight from Teensy GPIO pins instead (ledBoard = LED_TEENSY). Teensy LED
// pins used: 2, 3, 14, 15 (all free PWM pins; clear of LCD/I2S/I2C — see
// the GPIO LED setup in main.cpp). The sense electrodes stay on the MPR121.
//
// CONFIRM AGAINST YOUR PCB: the sense/LED electrodes below are the clean
// default for a 6+6 split. Reorder the rows to match the physical pad ring
// positions, but keep the LED = sense + 6 relationship unless you rewire.
//
// Hangdrum "ding" layout: TOP is the central low fundamental, upper ring is
// the melodic tone fields ascending around it, lower ring is bass voicings.
//                       sense                LED
//                       board, ele,          board, ele
static constexpr SensorConfig SENSORS[] = {

    

            // Upper concentric ring — tone fields (board C)
    { 1, 0,  1, 6  },           //0                    — sense C ELE0, LED C ELE6
    { 1, 1,  1, 7  },           //1  upper ring 0 (front) — sense C ELE1, LED C ELE7
    { 1, 2,  1, 8  },           //2  upper ring 1         — sense C ELE2, LED C ELE8
    { 1, 3,  LED_TEENSY, 3  },  //3  upper ring 2         — sense C ELE3, LED Teensy pin 2  (ELE9 driver faulty)
    { 1, 4,  1, 10  },  //4  upper ring 3         — sense C ELE4, LED Teensy pin 3  (ELE10 driver faulty)
    { 1, 5,  1, 11 },           //5  upper ring 4         — sense C ELE5, LED C ELE11

    // Lower concentric ring — bass register (board A)
    { 0, 6,  LED_TEENSY, 12 },  //6  lower ring 0 (front) — sense A ELE6 (A ELE0 BUST/noisy → abandoned), LED Teensy pin 22 (pin 12 is SPI MISO — claimed by the LCD, won't drive)
    { 0, 1,  0, 7  },           //7  lower ring 1         — sense A ELE1, LED A ELE7
    { 0, 2,  0, 8  },           //8  lower ring 2         — sense A ELE2, LED A ELE8
    { 0, 3,  LED_TEENSY, 14 },  //9  lower ring 3         — sense A ELE3, LED Teensy pin 14 (ELE9 driver faulty)
    { 0, 4,  0, 10 },  //10 lower ring 4         — sense A ELE4, LED Teensy pin 15 (ELE10 driver faulty)
    { 0, 5,  0, 11 },           //11 lower ring 5         — sense A ELE5, LED A ELE11
};

static constexpr uint8_t NUM_SENSORS =
    static_cast<uint8_t>(sizeof(SENSORS) / sizeof(SENSORS[0]));

// ── Sound definition (decoupled: scale × timbre) ─────────────────────────────
// The instrument's sound is the combination of a SCALE (which note each pad
// plays — cycled by swipe left/right) and a TIMBRE (the sound character —
// cycled by swipe up/down). They're independent: any scale can be played with
// any timbre, and each is remembered separately across power-off.
struct ScaleSet {
    const char* name;
    float       freqs[12];        // one per sensor, in SENSORS[] order
};

struct TimbreSet {
    const char* name;
    short       waveformType;     // WAVEFORM_TRIANGLE, WAVEFORM_BANDLIMIT_SAWTOOTH, etc.
    float       subMix;           // sub-oscillator level (0.0–1.0)
    float       filterBaseHz;     // filter cutoff when hand is far (dark)
    float       filterMaxHz;      // filter cutoff when hand is close (bright)
    float       filterQ;          // resonance (0.7 = flat, 1.5 = warm, 3.0 = aggressive)
    float       bellMix;          // bell loudness in master ch 3 (0 = no bell)
    bool        chorus;           // true → route through the chorus send (Juno);
                                  // defaults false for every dry timbre
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

    // Octave 6
    static constexpr float C6  = 1046.50f;
    static constexpr float D6  = 1174.66f;
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

// ── Scale sets (swipe LEFT/RIGHT to cycle) ───────────────────────────────────
// What note each pad plays. Pitch only — no timbre. freqs[] is in SENSORS[]
// order: idx11 is the 12th pad.
static constexpr uint8_t NUM_SCALE_SETS = 9;

static const ScaleSet SCALE_SETS[NUM_SCALE_SETS] = {
    // 0: Hangdrum / D Kurd — D minor, voiced as STACKED THIRDS so any 3 adjacent
    // pads form a chord (e.g. {3,4,5}=C major, {0,1,2}=D minor, {5,6,7}=G minor).
    { "D Kurd",
      { Note::D3, Note::F3, Note::A3, Note::C4, Note::E4, Note::G4,
        Note::Bb4, Note::D5, Note::F4, Note::A4, Note::C5, Note::E5 } },

    // 1: Just Intonation (D root) — pure harmonic ratios, beatless intervals.
    { "Just Intonation",
      { JI::D3, JI::A3, JI::Bb3, JI::C4, JI::D4,
        JI::E4, JI::F4, JI::A4, JI::D5, JI::D5 * 5.0f/4.0f,
        JI::D5, JI::D5 * 3.0f/2.0f } },

    // 2: Overtones — MICROTONAL. The natural harmonic series, harmonics 8…19 of
    // A0 (27.5 Hz). Pads ascend the overtones, so the 11th (302.5) and 13th
    // (357.5) land in the cracks between the piano notes — that's the cool
    // "xenharmonic" colour. Beautifully consonant since it IS one harmonic stack.
    { "Overtones",
      { 220.0f, 247.5f, 275.0f, 302.5f, 330.0f, 357.5f,
        385.0f, 412.5f, 440.0f, 467.5f, 495.0f, 522.5f } },

    // 3: Chromatic — consecutive semitones from D3, every interval available.
    { "Chromatic",
      { Note::D3, Note::Eb3, Note::E3, Note::F3, Note::Gb3,
        Note::G3, Note::Ab3, Note::A3, Note::Bb3, Note::B3,
        Note::C4, Note::Db4 } },

    // 4: Sad Vibes / D Dorian — stacked thirds (natural 6th = B), so adjacent
    // triples are Dm / C / Em / F etc.
    { "Sad Vibes",
      { Note::D3, Note::F3, Note::A3, Note::C4, Note::E4, Note::G4,
        Note::B4, Note::D5, Note::F4, Note::A4, Note::C5, Note::E5 } },

    // 5: Happy Vibes / C Major Pentatonic — C-E-G-A stacked up the pads, so
    // every 3 adjacent are an Am or C-major voicing (pentatonic = no wrong notes).
    { "Happy Vibes",
      { Note::C3, Note::E3, Note::G3, Note::A3, Note::C4, Note::E4,
        Note::G4, Note::A4, Note::C5, Note::E5, Note::G5, Note::A5 } },

    // 6: Bright Pentatonic — same C-E-G-A stack as Happy but starting a third
    // higher, so it sits brighter and tops out on C6.
    { "Bright Pentatonic",
      { Note::E3, Note::G3, Note::A3, Note::C4, Note::E4, Note::G4,
        Note::A4, Note::C5, Note::E5, Note::G5, Note::A5, Note::C6 } },

    // 7: Harmonic Minor (A) — stacked thirds A C E G# B D F, so adjacent triples
    // give Am, C, E major (the G# leading tone), G#dim, etc. — melancholic/Eastern.
    { "Harmonic Minor",
      { Note::A3, Note::C4, Note::E4, Note::Ab4, Note::B4, Note::D5,
        Note::F4, Note::A4, Note::C5, Note::E5, Note::Ab4, Note::B4 } },

    // 8: Accordion — chromatic, laid out like a button-accordion keyboard. Each
    // ring is a whole-tone scale; the two rings are offset by a semitone, so
    // moving along a ring steps by whole tones and crossing rings (pad n ↔ n+6)
    // steps by a semitone. All 12 chromatic notes, evenly under the hands.
    { "Accordion",
      //  upper ring (0–5): whole-tone from C
      { Note::C4, Note::D4, Note::E4, Note::Gb4, Note::Ab4, Note::Bb4,
      //  lower ring (6–11): the other whole-tone (semitone-shifted)
        Note::Db4, Note::Eb4, Note::F4, Note::G4, Note::A4, Note::B4 } },
};

// ── Timbre sets (swipe UP/DOWN to cycle) ─────────────────────────────────────
// The sound character: oscillator waveform, sub-bass blend, the proximity →
// filter sweep (dark when far → bright when close), resonance, and bell level.
// Independent of the scale — any timbre plays any scale. The arpeggiator is no
// longer a timbre (it's a play MODE now, see below), so any timbre can be
// arpeggiated. Tuned so neighbours are clearly distinct. (True Juno chorus is a
// real effect — see the chorus send in main; Moog's ladder is approximated.)
static constexpr uint8_t NUM_TIMBRE_SETS = 9;

static const TimbreSet TIMBRE_SETS[NUM_TIMBRE_SETS] = {
    //  name           waveform                    sub    fbase   fmax    Q     bell  [chorus]
    // Warm — mellow triangle pad, dark and rounded.
    { "Warm",     WAVEFORM_TRIANGLE,          0.35f, 200.0f, 4500.0f, 1.4f, 0.03f }, // 0
    // Crystal — glassy, bright sine bell; almost no sub, sparkly, wide-open.
    { "Crystal",  WAVEFORM_SINE,              0.12f, 600.0f, 9000.0f, 0.8f, 0.09f }, // 1
    // Edgy — bright buzzy saw, the cutting one.
    { "Edgy",     WAVEFORM_BANDLIMIT_SAWTOOTH,0.20f, 300.0f, 7000.0f, 1.8f, 0.03f }, // 2
    // Dark — heavy, brooding triangle; lots of sub, low cutoff, resonant.
    { "Dark",     WAVEFORM_TRIANGLE,          0.55f, 170.0f, 3600.0f, 2.1f, 0.06f }, // 3
    // Soft — warm rounded triangle pad, mellow (clearly darker than Shimmer).
    { "Soft",     WAVEFORM_TRIANGLE,          0.30f, 300.0f, 5500.0f, 1.0f, 0.04f }, // 4
    // Shimmer — airy sine, very bright and open, tiny sub, lots of sparkle.
    { "Shimmer",  WAVEFORM_SINE,              0.12f, 500.0f, 9500.0f, 0.8f, 0.08f }, // 5
    // Cry — warm triangle with a singing resonance (harmonic-minor character).
    { "Cry",      WAVEFORM_TRIANGLE,          0.30f, 220.0f, 5500.0f, 1.6f, 0.05f }, // 6
    // Juno — deep, smooth, dreamy saw pad (Men I Trust). Low/round filter, heavy
    // sub, gentle resonance, AND the chorus send for that wide ensemble shimmer.
    { "Juno",     WAVEFORM_BANDLIMIT_SAWTOOTH,0.42f, 220.0f, 4200.0f, 1.1f, 0.02f, true }, // 7
    // Moog — fat, dark, resonant ladder-ish lead/bass.
    { "Moog",     WAVEFORM_BANDLIMIT_SAWTOOTH,0.45f, 150.0f, 3500.0f, 2.6f, 0.02f }, // 8
};

// ── Play modes (third axis — LONG-PRESS the screen to cycle) ─────────────────
// Proximity = normal (all held pads sound together). The Arp modes gate the
// held pads one at a time at a fixed step rate. FM = mono FM (first held pad is
// the carrier, the rest modulate it). Benjolin = two cross-modulating
// oscillators + rungler chaos, gated/pitched by the held pads; the plain
// version snaps the rungler to the active scale, "Raw" runs free.
enum PlayMode : uint8_t {
    MODE_PROX = 0, MODE_ARP_SLOW, MODE_ARP_MED, MODE_ARP_FAST,
    MODE_FM, MODE_FM_POLY, MODE_BENJOLIN, MODE_BENJOLIN_RAW, MODE_CRACKLE, MODE_HANG
};
static constexpr uint8_t NUM_MODES = 10;
static const char* const MODE_NAMES[NUM_MODES] =
    { "Proximity", "Arp Slow", "Arp Med", "Arp Fast",
      "FM", "FM Poly", "Benjolin", "Benjolin Raw", "Cracklebox", "Hang Bow" };
static constexpr float MODE_ARP_PERIOD_MS[NUM_MODES] =
    { 0.0f, 360.0f, 180.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
static constexpr float ARP_HOLD_INTENSITY = 0.10f; // pad counts as "held" above this

// ── FM modes ─────────────────────────────────────────────────────────────────
// FM (mono): first-held pad = carrier; every other held pad's oscillator
// modulates it, scaled by that pad's proximity (hover = shimmer, press = clang).
static constexpr float FM_DEPTH_OCTAVES = 3.0f;  // carrier frequency-modulation range
static constexpr float FM_MOD_SCALE     = 0.6f;  // per-modulator amount (× proximity)
static constexpr float FM_LEVEL         = 0.30f; // mono-FM output level (was hot at 1.0)
// FM Poly: every held pad sounds AND self-FMs (its own carrier + a fixed-ratio
// modulator); proximity sets the FM depth per pad. Plays like Proximity but metallic.
static constexpr float FM_POLY_RATIO  = 2.0f;    // modulator freq = note × this
static constexpr float FM_POLY_DEPTH  = 1.5f;    // per-voice FM range (octaves)
static constexpr float FM_POLY_AMT    = 1.0f;    // modulator amplitude (× proximity)

// ── Benjolin modes ───────────────────────────────────────────────────────────
// Two oscillators cross-modulate; an 8-bit shift register (clocked by osc B,
// data = sign of osc A) makes the stepped "rungler" that bends the pitch. Held
// pads layer in parameters by PRESS ORDER (see the loop): 1=osc A pitch+gate,
// 2=osc B pitch, 3=cross-mod, 4=rungler, 5=tone — and WHICH pad you use tints
// each (pad index biases the parameter).
static constexpr float BENJOLIN_CROSS  = 0.5f;   // base cross-modulation depth
static constexpr float BENJOLIN_RUNGLE = 0.6f;   // base rungler → pitch depth (raw mode)
static constexpr float BENJOLIN_LEVEL  = 0.38f;  // output level (was hot)

// ── Cracklebox mode ──────────────────────────────────────────────────────────
// One square oscillator per held pad (1 pad = clean square; each extra pad XORs
// its tone in). Held pads also cross-couple (bridge) with strength ∝ proximity.
static constexpr float CRACKLE_LEVEL  = 0.30f;
static constexpr float CRACKLE_OCT    = 0.25f; // pitch multiplier (0.25 = two octaves down)
static constexpr float CRACKLE_COUPLE = 0.9f;  // max cross-coupling (× proximity) → chaos

// ── Hang Bow mode ────────────────────────────────────────────────────────────
// Bowed handpan physical model (see hangbow.h for the per-voice tuning). All 12
// pads are sympathetic resonators; proximity bows them. This is just the bus level.
static constexpr float HANG_LEVEL = 1.0f;

// ── Touch-screen press timing ────────────────────────────────────────────────
static constexpr uint32_t MODE_PRESS_MIN_MS = 550;   // (unused — mode is on the ‹ › arrows now)
static constexpr uint32_t LOCK_HOLD_MS      = 5000;  // hold this long → toggle lock/visualiser

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
static constexpr float AMP_RAMP_MS = 8.0f;  // matches UPDATE_MS — smooth ramp

// Sound-set change: portamento. On a live scale switch the sustained pitches
// glide to the new notes over EXACTLY this long, then stop — a fixed-time log
// (constant cents/sec) sweep, so it both feels musical and finishes promptly.
// (The old 1-pole had a long tail that made even 200 ms feel like ~2 s.)
// Amplitude is left untouched, so the change has no pause/click. 0 = instant.
static constexpr float SET_GLIDE_MS = 250.0f;

// Timbre change: the filter (cutoff/resonance) and sub-bass blend morph to the
// new timbre over this long instead of jumping. The oscillator waveform still
// switches instantly (a single oscillator can't crossfade waveforms), but
// morphing the filter + sub makes the change feel smooth rather than abrupt.
static constexpr float TIMBRE_MORPH_MS = 400.0f;

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

// ── Edge de-strobe (smooth response at the far/quiet end) ─────────────────────
// At maximum playing distance the fast EMA hovers right at the deadband, so the
// gate used to chatter (one frame below → intensity slammed to 0) and the tone
// + LED strobed. Two fixes, neither sacrifices reach:
//   • Hysteresis: once a pad has "opened", it stays open until the fast EMA
//     falls PROX_RELEASE_DELTA *below* the deadband — so brief dips don't slam
//     it shut. (Open threshold is still exactly the deadband: same distance.)
//   • Output smoothing: a 1-pole low-pass on the final intensity removes the
//     residual frame-to-frame jitter. Higher = snappier but more flicker.
static constexpr float PROX_RELEASE_DELTA = 1.5f; // gate-close hysteresis (counts below deadband)
// Asymmetric 1-pole on intensity: instant on the way UP (snappy attack) but
// smoothed on the way DOWN — the strobe was dropouts (downward dips), so only
// the fall needs damping. 1.0 = no smoothing in that direction.
static constexpr float PROX_SMOOTH_RISE  = 1.0f;  // attack (1.0 = instant)
static constexpr float PROX_SMOOTH_ALPHA = 0.35f; // release/dropout damping
// Velocity-adaptive attack: a one-frame jump of this many counts gives full,
// instant tracking (and snaps the gate open); smaller/slower changes keep the
// gentle smoothing. Lower = more things feel "fast". (idle noise ≈ 1, 1 cm ≈ 5.)
static constexpr float PROX_FAST_JUMP_REF = 5.0f;

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
// environment. A pad more than this many 10-bit counts from the per-board
// median (in EITHER direction) is rewritten to the median:
//   • baseline ≪ median → pad is "barely sensitive" (rawDelta = baseline −
//     filtered clamps to 0 until a hard touch) — a hand hovered at lock time.
//   • baseline ≫ median → pad rests with a positive rawDelta and plays a
//     CONSTANT tone (voice never closes), yet its bell still fires on a real
//     touch because the jump detector still spikes.
static constexpr uint16_t BASELINE_OUTLIER_DELTA = 30;

// ── MPE (USB-MIDI) output ────────────────────────────────────────────────────
// Each pad gets its own member channel for polyphonic aftertouch (channel
// pressure). Notes are sent at the nearest semitone — DAW handles any further
// tuning. The Teensy keeps producing audio at the same time (dual mode);
// unplug the audio jack if you only want the MIDI.
static constexpr bool     MPE_ENABLE              = true;
static constexpr uint8_t  MPE_MASTER_CH           = 1;   // 1-indexed MIDI channel
static constexpr uint8_t  MPE_MEMBER_BASE_CH      = 2;   // pad 0 → ch 2, pad 11 → ch 13
static constexpr uint32_t MPE_PRESSURE_THROTTLE_MS = 20; // ≤50 Hz per pad
static constexpr uint8_t  MPE_PRESSURE_MIN_DELTA  = 2;   // skip if change < this (0–127)

// Update timing
static constexpr uint32_t UPDATE_MS = 8; // ~125 Hz; the burst reads fill most of this at 100 kHz I2C
