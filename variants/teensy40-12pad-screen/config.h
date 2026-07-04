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

// First-filter iterations: 0–3 → 6/10/18/34 samples averaged per measurement.
// More averaging = lower idle-noise floor at the SAME output rate, which is
// what lets the deadband sit lower = more proximity reach. Bench-measured
// (sensor_characterization FFI sweep, 2 s per value): FFI=0 → idle wobble up
// to 2 counts (std 0.25); FFI≥1 → dead flat, zero deviation over 2298 samples.
// 1 (=10 samples) is the sweet spot; higher showed no further gain.
static constexpr uint8_t SENSOR_FFI = 1;

// Electrode sample interval: 2^ESI ms. The chip's filtered output updates
// every SFI×ESI = 4×2^ESI ms, so ESI=1 (2 ms) → new data every 8 ms, ESI=0
// (1 ms) → every 4 ms. Bench capture showed the release tail is mostly this
// chip-side filter (~6 output frames to settle), so halving the cadence
// halves both attack and release lag at the source. REVERT TO 1 IF the
// MPR121-driven LEDs (pads 0,1,2,5,7,8,11) visibly flicker — the old comment
// claimed ESI=2 ms mattered for that, never verified with data.
static constexpr uint8_t SENSOR_ESI = 0;

// Median-of-3 spike filter on the raw reads. Bench-measured COST: exactly one
// frame of onset latency on every tap (raw delta hit 211 while the median
// still reported 10; it caught up the following frame). Bench-measured
// BENEFIT at FFI≥1: none — zero single-sample spikes in 2298-sample scans
// (the chip's first filter already averages 10 samples). OFF by default;
// set to 1 if phantom touches/blips reappear in the full instrument.
static constexpr uint8_t SENSOR_MEDIAN3 = 0;

// Sensor supply voltage — used to compute the auto-configuration charge limits
// (USL/LSL/TL per the MPR121 datasheet). The Teensy 4.0 powers the MPR121
// boards at 3.3 V.
static constexpr float SENSOR_VDD = 3.3f;

// Per-electrode auto-configuration: at boot the chip binary-searches CDC/CDT for
// each electrode so it charges to ~0.7·Vdd — equalising the mismatched pads and
// maximising sensitivity/reach. Runs once on the Stop→Run transition during the
// startup baseline lock (so nothing should be touching at power-on). When on,
// SENSOR_CDC/CDT above are only the pre-config fallback.
static constexpr bool SENSOR_AUTOCONFIG = true;

// Chip-side idle recalibration (the old full ECR baseline reload). Superseded by
// the software baseline; kept as a flag so it's easy to A/B. Leaving it on while
// auto-config is enabled would re-trigger auto-config every couple of seconds.
static constexpr bool RECAL_ENABLE = false;

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
static constexpr uint8_t NUM_SCALE_SETS = 11;

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

    // 9: Chords I IV V (D major) — chord FUNCTIONS side by side: pads 0–3 are
    // the I chord (D), 4–7 the IV (G), 8–11 the V (A). Sweep a region with a
    // flat hand for a full chord; walking around the circle is I → IV → V → I.
    { "Chords I IV V",
      { Note::D3, Note::Gb3, Note::A3, Note::D4,     // I:  D  F# A  D
        Note::G3, Note::B3,  Note::D4, Note::G4,     // IV: G  B  D  G
        Note::A3, Note::Db4, Note::E4, Note::A4 } }, // V:  A  C# E  A

    // 10: Jazz ii V I (D major, 7th chords) — pads 0–3 = Dmaj7 (I), 4–7 = Em7
    // (ii), 8–11 = A7 (V); circling gives the endless ii–V–I turnaround.
    { "Jazz ii V I",
      { Note::D3, Note::Gb3, Note::A3, Note::Db4,    // I:  Dmaj7
        Note::E3, Note::G3,  Note::B3, Note::D4,     // ii: Em7
        Note::A3, Note::Db4, Note::E4, Note::G4 } }, // V:  A7
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
    MODE_FM, MODE_FM_POLY, MODE_BENJOLIN, MODE_BENJOLIN_RAW, MODE_CRACKLE, MODE_HANG,
    // The "calming/beautiful" batch (each has its own engine header):
    MODE_SWARM,      // additive chime swarm — brushing a magical mobile
    MODE_GRAIN,      // granular hang-drum sample, proximity-bowed (samples/)
    MODE_STRINGS,    // looped string-section sample, ensemble pads (samples/)
    MODE_CATHEDRAL,  // normal voices into a huge reverb space
    MODE_FREEZE,     // normal voices; press hard to freeze the sound in the air
    MODE_TANPURA,    // jawari drone strings cycling the held notes
    MODE_BOWLS,      // rubbed singing bowls with beating partials
    MODE_FLUTE,      // breath-pressure waveguide wind
    MODE_CELLO,      // stick-slip bowed-string waveguide
    MODE_PHASE       // Reich-ish phasing mallet loops (generative)
};
static constexpr uint8_t NUM_MODES = 20;
static const char* const MODE_NAMES[NUM_MODES] =
    { "Proximity", "Arp Slow", "Arp Med", "Arp Fast",
      "FM", "FM Poly", "Benjolin", "Benjolin Raw", "Cracklebox", "Hang Bow",
      "Swarm", "Grain Hang", "Strings", "Cathedral", "Freeze",
      "Tanpura", "Bowls", "Flute", "Cello", "Phase Chimes" };
static constexpr float MODE_ARP_PERIOD_MS[NUM_MODES] =
    { 0.0f, 360.0f, 180.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
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

// ── Engine-mode bus levels ───────────────────────────────────────────────────
// One knob per new mode — the engines' voicing constants live in their headers
// (swarm.h, grainsample.h, strings.h, bowls.h, tanpura.h, flute.h, cello.h,
// phasechimes.h); these only set how loud each sits on the final bus.
static constexpr float SWARM_LEVEL    = 0.85f;
static constexpr float GRAIN_LEVEL    = 0.95f;
static constexpr float STRINGS_LEVEL  = 0.85f;
static constexpr float TANPURA_LEVEL  = 0.90f;
static constexpr float BOWLS_LEVEL    = 1.00f;
static constexpr float FLUTE_LEVEL    = 0.55f;
static constexpr float CELLO_LEVEL    = 0.70f;
static constexpr float PHASE_LEVEL    = 0.35f;
// Cathedral: normal voices + a huge reverb. Freeze: normal voices + a frozen
// spectral layer captured when you press hard (release lets it fade).
static constexpr float CATHEDRAL_WET      = 0.55f; // reverb level on the bus
static constexpr float CATHEDRAL_ROOMSIZE = 0.89f; // 0..1 — bigger = longer tail
static constexpr float CATHEDRAL_DAMPING  = 0.45f; // 0..1 — higher = darker tail (also less hiss)
static constexpr float FREEZE_LAYER       = 0.85f; // frozen-layer level on the bus
static constexpr float FREEZE_TRIGGER     = 0.75f; // press intensity that captures

// Engines ignore intensities below this — keeps MPR121 baseline wobble from
// randomly striking ghost notes in the swarm/chime/pluck modes.
static constexpr float INTENSITY_GATE = 0.06f;

// ── Touch-screen press timing ────────────────────────────────────────────────
static constexpr uint32_t MODE_PRESS_MIN_MS = 550;   // (unused — mode is on the ‹ › arrows now)
static constexpr uint32_t LOCK_HOLD_MS      = 5000;  // hold this long → toggle lock/visualiser
// Minimum finger travel (display px, 240 px panel) for a chip-reported swipe to
// count as a swipe. The CST816S's internal threshold is a few px, so a rough
// press used to fire timbre/scale changes; below this travel the touch is
// reclassified as a tap at the press position. Raise if it still mis-swipes,
// lower if deliberate swipes get eaten.
static constexpr int32_t SWIPE_MIN_PX = 45;

// ── Synth parameters ─────────────────────────────────────────────────────────
// Voice architecture
// 0.6 (not 1.0): leaves headroom so main+sub and the resonant filter boost
// don't clip before the dcAmp stage — clipping is what made sines sound buzzy.
static constexpr float MAIN_OSC_AMPLITUDE = 0.6f;  // DC stage still sets final level
static constexpr float SUB_OSC_AMPLITUDE  = 1.0f;  // sub osc always full (voiceMix controls blend)
static constexpr float VOICE_MAX_AMP      = 0.45f; // max DC level per voice (prevents clipping)

// Mixer gain structure
static constexpr float STAGE_GAIN  = 0.25f; // per-channel gain in stage mixers
static constexpr float MASTER_GAIN = 0.7f;  // per-channel gain in master mixer (voice path only)

// ONE global volume for the whole instrument — every mode, voices and engines
// alike (a final amplifier just before the output). Raise above 1.0 with care:
// many simultaneous voices can clip.
static constexpr float MASTER_VOLUME = 1.0f;

// Output polarity. 1 = balanced/differential (R = inverted L; for a true
// balanced DI/line input — but it CANCELS TO SILENCE if L+R are summed to mono
// or played on normal stereo gear). 0 = normal unbalanced stereo (same polarity
// both channels) — use this with ordinary mixers/amps/speakers.
#define OUTPUT_BALANCED 0

// Amplitude envelope (DC ramp)
static constexpr float AMP_RAMP_MS = 6.0f;  // matches UPDATE_MS — smooth ramp

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
// Back to 1 (instant): single-sample electrical glitches are now killed by the
// median-of-3 filter on the raw reads, so the extra 8 ms confirm frame that
// used to reject them is pure latency.
static constexpr uint8_t  TOUCH_CONFIRM_FRAMES = 1;
// Proximity onset gate: hold intensity at 0 until the fast EMA has stayed above
// PROX_DEADBAND this many frames. This is the WEAK-onset path only — a strong
// approach (one-frame jump > PROX_FAST_JUMP_REF) snaps the gate open instantly,
// so real playing feels immediate. Weak signals hovering just above the
// deadband (EM events, marginal drift) must persist ~24 ms, which is what buys
// back the lower deadband below without phantom notes. A genuinely slow hand
// swell doesn't notice 24 ms at its very first frame. (History 6→4→3; hardware
// at 4 was phantom-free. If soft phantom blips reappear, go back up before
// touching anything else.)
static constexpr uint8_t  PROX_CONFIRM_FRAMES  = 3;

// ── Proximity → volume mapping (intensity = (fast−deadband)/(proxMax−deadband))
// From the CDC=16 capture: idle noise ≈ 1, ~1 cm ≈ 5, on plastic ≈ 22. So the
// playable swell lives between a light near-touch and resting on the shell;
// deadband sits just above idle noise, proxMax ≈ firm-plastic so the sound is
// full by the time you rest on it (metal contact then adds the bell).
static constexpr float PROX_DEADBAND = 5.0f;  // 8→6→5→4→5. The drop to 4 was justified
                                              // by BENCH noise (±1 count, MPR121-only
                                              // rig) — but in-instrument noise with the
                                              // full-frame screen DMA + audio running
                                              // was never re-measured, and 4 left zero
                                              // margin (startup flicker on several
                                              // pads). 5 buys one count of margin;
                                              // in-instrument reach is still better
                                              // than the old chip-baseline builds
                                              // because nothing eats slow approaches
                                              // any more. Re-drop to 4 only after a
                                              // Teleplot idle capture IN the full
                                              // instrument shows it's clean.
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
static constexpr float PROX_SMOOTH_RISE  = 0.6f;  // attack smoothing. Was 1.0 (instant),
                                                  // but the usable delta range is only
                                                  // ~12 integer counts, so instant rise
                                                  // passed the 10-bit quantization
                                                  // straight to the amplitude = audible
                                                  // volume STEPPING on swells. 0.6
                                                  // reaches ~94% in 3 frames (24 ms) —
                                                  // still a snappy attack, steps blurred.
static constexpr float PROX_SMOOTH_ALPHA = 0.35f; // release damping for SMALL dips only
// Drop-adaptive release: an intensity drop of this size in one frame unlocks a
// full-speed fall. A real hand-lift collapses in 2–3 frames (~20 ms) instead of
// dragging through the 0.35 smoothing (~80 ms tail); edge jitter (small dips)
// still gets the full damping, so the anti-strobe fix is preserved.
static constexpr float PROX_SMOOTH_FALL_REF = 0.5f;
// Velocity-adaptive attack: a one-frame jump of this many counts gives full,
// instant tracking (and snaps the gate open, skipping PROX_CONFIRM_FRAMES);
// smaller/slower changes keep the gentle smoothing. Lower = more approaches
// feel "fast". (idle noise ≈ 1, 1 cm ≈ 5.)
static constexpr float PROX_FAST_JUMP_REF = 3.0f; // ↓ from 4 — more taps snap instantly
// Base rise-tracking speed when the jump is small (slow approaches). Higher =
// faster swells at the cost of slightly more noise reaching the volume (the
// PROX_SMOOTH_RISE stage now handles the smoothing).
static constexpr float PROX_ATTACK_ALPHA = 0.75f; // 0.5→0.65→0.75 ("still a bit slow"
                                                  // on hardware at 0.65, idle clean)
// Same idea on release: a one-frame FALL of this many counts tracks the lift
// at full speed (fast fade), small dips keep the slow fall EMA (no strobe).
static constexpr float PROX_FALL_JUMP_REF = 8.0f;

// ── Software baseline (full port of the esp32s3-19pad design) ────────────────
// The delta is computed ONLY against a per-pad software baseline on the raw
// filtered value; the chip's baseline registers are not read at all. Why (both
// measured): the register moves in 4-count jumps (only the top 8 of its 10
// bits are exposed) — with a deadband of 4, every internal boundary crossing
// was a phantom +4 delta: pads "popping out of calibration", fluttering, and
// reading stuck until the heal caught it (pad 7 = A-ELE1, harness neighbour of
// the abandoned noisy A-ELE0, was hit most often); and the chip's falling
// baseline filter chases a slowly approaching hand (2× faster at ESI=1 ms),
// eating the delta before it accumulates = lost reach.
// The software baseline moves only per this state machine:
//   • idle      → tracks slow EM/temperature/rotation drift (~2 s tau)
//   • negative  → filtered ABOVE base is unambiguously a stale baseline (a
//     delta       hand only pulls filtered DOWN) → track fast (~0.1 s) — a
//                 desensitised pad recovers almost immediately
//   • held      → frozen, so sustained notes never fade
//   • stuck     → after PROX_HOLD_MAX_FRAMES (~8 s, longer than any musical
//                 hold) the pad re-zeroes over ~1 s; low-parked residuals go
//                 sooner via PROX_LOW_HOLD_* below.
static constexpr float    PROX_BASE_IDLE_ALPHA = 0.003f; // ↓ from 0.004: restores the
                                                         // ~2 s wall-clock tau at the
                                                         // 6 ms frames (absorbs slow
                                                         // approaches less = reach)
static constexpr float    PROX_BASE_NEG_ALPHA  = 0.05f;
static constexpr float    PROX_HEAL_ALPHA      = 0.02f;
static constexpr uint16_t PROX_HOLD_MAX_FRAMES = 1333; // 8 s @ 167 Hz (6 ms) frames
// Settle window after every (re)seed: the baseline hard-follows the filtered
// value and the pads stay MUTED for this many frames (~1.5 s @ 6 ms). The
// chip's filtered output keeps converging for up to ~1 s after init — auto-
// config has just rewritten every electrode's CDC/CDT — so a single-instant
// seed left phantom deltas on the pads that settle furthest = a few pads
// flickering right at startup. (Same cure as the esp32s3 SEED_MS window.)
// Raise if any boot flicker remains.
static constexpr uint16_t PROX_SEED_FRAMES     = 250;
// Warm-up after the seed window: idle tracking runs at PROX_BASE_WARM_ALPHA
// (~0.5 s tau) instead of the slow steady-state alpha for this many frames
// (~30 s @ 6 ms). Electrodes keep drifting for tens of seconds after power-on
// (thermal equilibration); at the slow alpha that drift could reach the
// deadband and flicker before being absorbed. Holds still freeze the baseline,
// so playing during warm-up is unaffected — only sub-deadband drift-chasing
// is faster. (Cost: very slow far-hovers during the first 30 s absorb sooner.)
static constexpr uint16_t PROX_WARM_FRAMES     = 5000;
static constexpr float    PROX_BASE_WARM_ALPHA = 0.012f;

// Per-pad deadband trim (counts, added to PROX_DEADBAND). Gives one physically
// noisier pad a higher gate without desensitising the others. Pad 7 (A-ELE1)
// starts at +1: it shares the harness with the abandoned noisy A-ELE0 wire and
// was the pad reported popping out of calibration most. Set back to 0 if the
// software-baseline fix alone cures it; raise further (+2) if it still misfires.
static constexpr float PAD_DEADBAND_TRIM[NUM_SENSORS] = {
    0, 0, 0, 0, 0, 0, 0, 1.0f, 0, 0, 0, 0
};
// Low-residual fast heal. Observed on hardware: with many notes pressed, the
// RELEASED pads' LEDs hang at a dim value — coupling from the still-down
// fingers/body leaves a small delta parked just above the deadband, and the
// 8 s stuck timeout is way too slow for that. A pad sitting INSIDE the low
// band (deadband … deadband+PROX_LOW_HOLD_BAND ≈ intensity < ~0.25) for
// PROX_LOW_HOLD_FRAMES straight is treated as residual and healed (~0.5 s
// fade). Real playing exits the band in either direction and resets the
// counter. Cost: an intentional ultra-quiet hover held statically for >2 s
// fades out — push slightly closer and it re-opens instantly.
static constexpr float    PROX_LOW_HOLD_BAND   = 3.0f;
static constexpr uint16_t PROX_LOW_HOLD_FRAMES = 333; // ≈2 s @ 167 Hz (6 ms frames)

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

// ── Harmonic Journey screen mode ─────────────────────────────────────────────
// An interactive tonal map displayed on the round screen. The long-press
// screen-state cycle includes this mode:
//   normal → harmonic journey → locked visualiser → normal (repeat).
//
// Entering Harmonic Journey first shows a PICKER of four "atlases", each a
// different theory of harmonic motion. Choosing one opens its network of chord
// nodes — tap a node to retune all 12 pads to that chord (with glide) and read
// the emotional/functional label for the move. The active play mode keeps
// running. Tap the journey title at the top to return to the picker.
//
//   1 Diatonic  — the seven diatonic chords of C major / A minor.
//   2 Flamenco  — Spanish / flamenco Phrygian-dominant motion (Andalusian
//                 cadence, the ♭II "Spanish" colour).
//   3 Jazz      — chromatic jazz devices: ii–V–I, tritone substitution,
//                 secondary dominants, modal mixture, Coltrane changes,
//                 augmented / whole-tone motion.
//   4 Cinematic — Neo-Riemannian chromatic mediants: a hexatonic P/L cycle of
//                 major & minor triads (the "magic / awe" sound).
//
// The conceptual framework for atlas 1 (and the whole idea of labelling every
// harmonic move with an emotion) is from:
//   "Tonal Map of Chord Sequences: Harmonic Analysis and Practical
//    Applications" — William R Thomas, atlas protocols, 2025-03-01.
//   Inspired by a tonal map diagram by @purpasteur. Atlases 2–4 extend that
//   premise into the modal-interchange, jazz-substitution and Neo-Riemannian
//   territories Thomas discusses as the natural next layers of the map.
// ─────────────────────────────────────────────────────────────────────────────

// Node SHAPE on screen: harmonic function (role the chord plays in the key),
// not chord quality. Tonic/subdominant/dominant are the three real functional
// categories; chromatic and symmetric cover chords that sit outside that
// three-way split (borrowed/substitute chords, and augmented/whole-tone/
// equal-division chords that have no real tonal "home").
enum HarmonicFunction : uint8_t {
    HFN_TONIC = 0,     // stable "home" chord            → circle
    HFN_SUBDOMINANT,   // pre-dominant / departure chord  → triangle
    HFN_DOMINANT,      // tension-resolving chord         → square
    HFN_CHROMATIC,     // borrowed / secondary / sub      → pentagon
    HFN_SYMMETRIC,     // augmented / whole-tone / hexatonic → hexagon
};

// Node COLOUR on screen: the emotional character from `feeling`, bucketed
// into a small fixed palette so the same mood always reads the same colour.
enum HarmonicMood : uint8_t {
    HMOOD_WARM = 0,    // positive, restful             → amber
    HMOOD_TENSION,     // drama, alarm, drive            → red
    HMOOD_MELANCHOLY,  // sadness, longing               → blue
    HMOOD_MYSTERY,     // wonder, uncertainty            → purple
    HMOOD_GROUNDED,    // exploration, lift, warmth      → green
};

struct HarmonicChord {
    const char* name;    // short chord label (kept for reference/debug — no
                         // longer drawn; the node shows `function`/`mood` instead)
    const char* feeling; // emotional quality / device — shown when no transition
                         // matrix is supplied for this journey; also the source
                         // for `mood`'s colour bucket
    uint8_t function;    // HarmonicFunction — the node's on-screen SHAPE
    uint8_t mood;        // HarmonicMood — the node's on-screen COLOUR
    float freqs[12];     // pad 0-5 = upper ring, 6-11 = lower ring
};

static constexpr uint8_t NUM_HARMONIC_CHORDS = 7;     // atlas 1 (diatonic) node count
static constexpr uint8_t HARMONIC_MAX_NODES  = 8;     // largest atlas (jazz)
static const float H_B5 = 987.77f; // B5 (not in Note:: namespace — B4 * 2)

static const HarmonicChord HARMONIC_CHORDS[NUM_HARMONIC_CHORDS] = {
    // 0: I — C major   "Pure positive" — the tonic home
    { "C",  "Pure positive", HFN_TONIC, HMOOD_WARM,
      { Note::C4, Note::E4, Note::G4, Note::C5, Note::E5, Note::G5,
        Note::C3, Note::E3, Note::G3, Note::C4, Note::E4, Note::G4 } },

    // 1: ii — D minor  "Hardship, faltering" — subdominant tension
    { "Dm", "Hardship", HFN_SUBDOMINANT, HMOOD_MELANCHOLY,
      { Note::D4, Note::F4, Note::A4, Note::D5, Note::F5, Note::A5,
        Note::D3, Note::F3, Note::A3, Note::D4, Note::F4, Note::A4 } },

    // 2: iii — E minor  "Mystery, complexity" — mediant (tonic-function substitute)
    { "Em", "Mystery", HFN_TONIC, HMOOD_MYSTERY,
      { Note::E4, Note::G4, Note::B4, Note::E5, Note::G5,     H_B5,
        Note::E3, Note::G3, Note::B3, Note::E4, Note::G4, Note::B4 } },

    // 3: IV — F major  "Exploration, warmth" — subdominant
    { "F",  "Warmth", HFN_SUBDOMINANT, HMOOD_GROUNDED,
      { Note::F4, Note::A4, Note::C5, Note::F5, Note::A5, Note::C6,
        Note::F3, Note::A3, Note::C4, Note::F4, Note::A4, Note::C5 } },

    // 4: V — G major  "Tension, drama, alarm" — dominant
    { "G",  "Tension", HFN_DOMINANT, HMOOD_TENSION,
      { Note::G4, Note::B4, Note::D5, Note::G5,     H_B5, Note::D6,
        Note::G3, Note::B3, Note::D4, Note::G4, Note::B4, Note::D5 } },

    // 5: vi — A minor  "Melancholy, pure negative" — relative minor (tonic-function substitute)
    { "Am", "Melancholy", HFN_TONIC, HMOOD_MELANCHOLY,
      { Note::A3, Note::C4, Note::E4, Note::A4, Note::C5, Note::E5,
        Note::A2, Note::C3, Note::E3, Note::A3, Note::C4, Note::E4 } },

    // 6: vii° — B diminished  "Alarm, destabilization" — leading-tone chord
    // ("Bo" is used as a screen-safe approximation of B°)
    { "Bo", "Alarm", HFN_DOMINANT, HMOOD_TENSION,
      { Note::B3, Note::D4, Note::F4, Note::B4, Note::D5, Note::F5,
        Note::B2, Note::D3, Note::F3, Note::B3, Note::D4, Note::F4 } },
};

// Emotional transition labels for each chord → chord move [from][to].
// Derived from William R Thomas's analysis of the tonal map transition
// annotations: each edge in the network carries an emotional descriptor
// that reflects the harmonic function and voice-leading character of the move.
static const char* const HARMONIC_TRANSITIONS[7][7] = {
    // from I (C major)
    { "Home",        "Unease",      "Opening up",  "Exploration", "Rising",       "Darkening",   "Into dark"    },
    // from ii (Dm)
    { "Lifting",     "Stillness",   "Lingering",   "Gathering",   "Building",     "Deepening",   "Destabilize"  },
    // from iii (Em)
    { "Hope rises",  "Faltering",   "Stillness",   "Magic lift",  "Expectation",  "Kindred",     "Uncertainty"  },
    // from IV (F)
    { "Soft return", "Darker",      "Meditation",  "Stillness",   "Drama",        "Consolation", "Destabilize"  },
    // from V (G)
    { "Resolution",  "Setback",     "Surprise",    "Retreat",     "Stillness",    "Bittersweet", "Into chaos"   },
    // from vi (Am)
    { "Redemption",  "Grief",       "Soul bond",   "Support",     "Setback",      "Stillness",   "Descent"      },
    // from vii° (Bdim)
    { "Release",     "Unresolved",  "Unstable",    "Relief",      "More tension", "Dark rest",   "Stillness"    },
};

// ── Atlas 2: Flamenco / Spanish (E Phrygian dominant) ────────────────────────
// Built around the Andalusian cadence (Am–G–F–E) and the Phrygian-dominant
// tonic E (with its raised G#). Node 0 = E sits at the centre as the cadential
// home; the ♭II (F) is the iconic "Spanish" colour. Triads voiced oct3/oct4.
static const HarmonicChord FLAMENCO_CHORDS[7] = {
    { "E",  "Cante home", HFN_TONIC, HMOOD_WARM,       // Phrygian-dominant tonic (E G# B)
      { Note::E4, Note::Ab4, Note::B4, Note::E4, Note::Ab4, Note::B4,
        Note::E3, Note::Ab3, Note::B3, Note::E3, Note::Ab3, Note::B3 } },
    { "F",  "Spanish bII", HFN_CHROMATIC, HMOOD_MYSTERY, // the flat-two colour (F A C)
      { Note::F4, Note::A4, Note::C4, Note::F4, Note::A4, Note::C4,
        Note::F3, Note::A3, Note::C3, Note::F3, Note::A3, Note::C3 } },
    { "G",  "Lift", HFN_SUBDOMINANT, HMOOD_GROUNDED,   // (G B D)
      { Note::G4, Note::B4, Note::D4, Note::G4, Note::B4, Note::D4,
        Note::G3, Note::B3, Note::D3, Note::G3, Note::B3, Note::D3 } },
    { "Am", "Lament", HFN_SUBDOMINANT, HMOOD_MELANCHOLY, // iv (A C E)
      { Note::A4, Note::C4, Note::E4, Note::A4, Note::C4, Note::E4,
        Note::A3, Note::C3, Note::E3, Note::A3, Note::C3, Note::E3 } },
    { "Dm", "Deepening", HFN_SUBDOMINANT, HMOOD_MELANCHOLY, // (D F A)
      { Note::D4, Note::F4, Note::A4, Note::D4, Note::F4, Note::A4,
        Note::D3, Note::F3, Note::A3, Note::D3, Note::F3, Note::A3 } },
    { "C",  "Brightening", HFN_TONIC, HMOOD_GROUNDED,  // (C E G)
      { Note::C4, Note::E4, Note::G4, Note::C4, Note::E4, Note::G4,
        Note::C3, Note::E3, Note::G3, Note::C3, Note::E3, Note::G3 } },
    { "B7", "Tension", HFN_DOMINANT, HMOOD_TENSION,    // dominant pull (B D# F# A)
      { Note::B4, Note::Eb4, Note::Gb4, Note::A4, Note::B4, Note::Eb4,
        Note::B3, Note::Eb3, Note::Gb3, Note::A3, Note::B3, Note::Eb3 } },
};

// ── Atlas 3: Jazz (chromatic devices, motion through keys) ───────────────────
// Eight 7th-chords; node 0 = Cmaj7 at the centre. Each outer node is a classic
// way jazz leaves the key: tritone sub, secondary dominant, modal mixture,
// Coltrane (major-third) change, augmented / whole-tone. The feeling string
// names the DEVICE (no per-pair matrix — the move itself is the point).
static const HarmonicChord JAZZ_CHORDS[8] = {
    { "CM7","Tonic rest", HFN_TONIC, HMOOD_WARM,       // Imaj7 (C E G B)
      { Note::C4, Note::E4, Note::G4, Note::B4, Note::C4, Note::E4,
        Note::C3, Note::E3, Note::G3, Note::B3, Note::C3, Note::E3 } },
    { "Dm7","ii — set out", HFN_SUBDOMINANT, HMOOD_GROUNDED, // (D F A C)
      { Note::D4, Note::F4, Note::A4, Note::C4, Note::D4, Note::F4,
        Note::D3, Note::F3, Note::A3, Note::C3, Note::D3, Note::F3 } },
    { "G7", "V — drive", HFN_DOMINANT, HMOOD_TENSION,  // dominant (G B D F)
      { Note::G4, Note::B4, Note::D4, Note::F4, Note::G4, Note::B4,
        Note::G3, Note::B3, Note::D3, Note::F3, Note::G3, Note::B3 } },
    { "Db7","Tritone sub", HFN_CHROMATIC, HMOOD_TENSION, // bII7 for G7 (Db F Ab B)
      { Note::Db4, Note::F4, Note::Ab4, Note::B4, Note::Db4, Note::F4,
        Note::Db3, Note::F3, Note::Ab3, Note::B3, Note::Db3, Note::F3 } },
    { "A7", "Secondary V", HFN_CHROMATIC, HMOOD_TENSION, // V7/ii (A C# E G)
      { Note::A4, Note::Db4, Note::E4, Note::G4, Note::A4, Note::Db4,
        Note::A3, Note::Db3, Note::E3, Note::G3, Note::A3, Note::Db3 } },
    { "AbM","Modal mix", HFN_CHROMATIC, HMOOD_MYSTERY, // bVI borrowed (Ab C Eb G)
      { Note::Ab4, Note::C4, Note::Eb4, Note::G4, Note::Ab4, Note::C4,
        Note::Ab3, Note::C3, Note::Eb3, Note::G3, Note::Ab3, Note::C3 } },
    { "EbM","Coltrane", HFN_SYMMETRIC, HMOOD_MYSTERY,  // bIII major-third change (Eb G Bb D)
      { Note::Eb4, Note::G4, Note::Bb4, Note::D4, Note::Eb4, Note::G4,
        Note::Eb3, Note::G3, Note::Bb3, Note::D3, Note::Eb3, Note::G3 } },
    { "F#+","Whole-tone", HFN_SYMMETRIC, HMOOD_MYSTERY, // augmented / side-slip (F# A# D)
      { Note::Gb4, Note::Bb4, Note::D4, Note::Gb4, Note::Bb4, Note::D4,
        Note::Gb3, Note::Bb3, Note::D3, Note::Gb3, Note::Bb3, Note::D3 } },
};

// ── Atlas 4a: Cinematic Hexatonic (Neo-Riemannian hexatonic cycle) ───────────
// Six triads on a ring (no centre): C, Cm, Ab, Abm, E, Em. Each step to a
// neighbour is a single-semitone voice-leading move (P = parallel major/minor,
// L = leading-tone exchange) — Cohn's hexatonic cycle, the chromatic-mediant
// "magic / awe" sound the article ties to Neo-Riemannian transformations.
// All six sit outside a tonal centre (that's the point of a symmetric
// hexatonic cycle), so every node here shares the SYMMETRIC shape (hexagon);
// only the mood colour distinguishes them.
static const HarmonicChord CINEMATIC_HEXATONIC_CHORDS[6] = {
    { "C",  "Radiance", HFN_SYMMETRIC, HMOOD_WARM,       // C major (C E G)
      { Note::C4, Note::E4, Note::G4, Note::C4, Note::E4, Note::G4,
        Note::C3, Note::E3, Note::G3, Note::C3, Note::E3, Note::G3 } },
    { "Cm", "Shadow", HFN_SYMMETRIC, HMOOD_MELANCHOLY,   // P: C minor (C Eb G)
      { Note::C4, Note::Eb4, Note::G4, Note::C4, Note::Eb4, Note::G4,
        Note::C3, Note::Eb3, Note::G3, Note::C3, Note::Eb3, Note::G3 } },
    { "Ab", "Wonder", HFN_SYMMETRIC, HMOOD_MYSTERY,      // L: Ab major (Ab C Eb)
      { Note::Ab4, Note::C4, Note::Eb4, Note::Ab4, Note::C4, Note::Eb4,
        Note::Ab3, Note::C3, Note::Eb3, Note::Ab3, Note::C3, Note::Eb3 } },
    { "Abm","Mystery", HFN_SYMMETRIC, HMOOD_MYSTERY,     // P: Ab minor (Ab Cb=B Eb)
      { Note::Ab4, Note::B4, Note::Eb4, Note::Ab4, Note::B4, Note::Eb4,
        Note::Ab3, Note::B3, Note::Eb3, Note::Ab3, Note::B3, Note::Eb3 } },
    { "E",  "Awe", HFN_SYMMETRIC, HMOOD_MYSTERY,         // L: E major (E G# B)
      { Note::E4, Note::Ab4, Note::B4, Note::E4, Note::Ab4, Note::B4,
        Note::E3, Note::Ab3, Note::B3, Note::E3, Note::Ab3, Note::B3 } },
    { "Em", "Reverie", HFN_SYMMETRIC, HMOOD_GROUNDED,    // P: E minor (E G B); ring closes L→C
      { Note::E4, Note::G4, Note::B4, Note::E4, Note::G4, Note::B4,
        Note::E3, Note::G3, Note::B3, Note::E3, Note::G3, Note::B3 } },
};

// ── Atlas 4b: Cinematic (augmented-triad hub) ────────────────────────────────
// C+ (the augmented triad C-E-Ab) sits at the centre — its three roots are a
// major third apart, so it's equidistant from three chromatic-mediant
// neighbourhoods. The ring holds a major/minor pair from each of those three
// regions, clockwise from the top: Cm, F, Em, A, Abm, Db. Coloured by chord
// type (pink/blue/amber/green), matching the Tonal Map — see typeColor below.
// No ring-to-ring edges (see ringEdges below): only the 6 hub spokes are
// legal moves, so voice leading only ever has to be smooth hub<->ring.
//
// Voicings: each ring chord keeps the hub's exact pad layout (pad%3 = which
// of the hub's three tones — C/E/Ab — that pad "descends from") and moves
// every pad the minimal distance to the nearest tone of the new chord. Since
// each ring chord shares exactly one tone with C+, and an augmented triad is
// equidistant (a major third) from its neighbours, this always works out to
// 0 semitones on the shared tone and exactly ±1 semitone (uniformly) on the
// other two — every pad glides by at most a half-step on every hub<->ring
// move. (Verified by exhaustive per-chord-tone search, not hand-tuned.)
static const HarmonicChord CINEMATIC_CHORDS[7] = {
    { "C+",  "Suspended",                              // centre: C augmented (C E Ab)
      { Note::C4, Note::E4, Note::Ab4, Note::C4, Note::E4, Note::Ab4,
        Note::C3, Note::E3, Note::Ab3, Note::C3, Note::E3, Note::Ab3 } },
    { "Cm",  "Shadow",                                 // C minor (C Eb G); E,Ab slots -1
      { Note::C4, Note::Eb4, Note::G4, Note::C4, Note::Eb4, Note::G4,
        Note::C3, Note::Eb3, Note::G3, Note::C3, Note::Eb3, Note::G3 } },
    { "F",   "Warmth",                                 // F major (F A C); E,Ab slots +1
      { Note::C4, Note::F4, Note::A4, Note::C4, Note::F4, Note::A4,
        Note::C3, Note::F3, Note::A3, Note::C3, Note::F3, Note::A3 } },
    { "Em",  "Longing",                                // E minor (E G B); C,Ab slots -1
      { Note::B3, Note::E4, Note::G4, Note::B3, Note::E4, Note::G4,
        Note::B2, Note::E3, Note::G3, Note::B2, Note::E3, Note::G3 } },
    { "A",   "Radiance",                                // A major (A C# E); C,Ab slots +1
      { Note::Db4, Note::E4, Note::A4, Note::Db4, Note::E4, Note::A4,
        Note::Db3, Note::E3, Note::A3, Note::Db3, Note::E3, Note::A3 } },
    { "Abm", "Mystery",                                 // Ab minor (Ab Cb=B Eb); C,E slots -1
      { Note::B3, Note::Eb4, Note::Ab4, Note::B3, Note::Eb4, Note::Ab4,
        Note::B2, Note::Eb3, Note::Ab3, Note::B2, Note::Eb3, Note::Ab3 } },
    { "Db",  "Wonder",                                  // Db major (Db F Ab); C,E slots +1
      { Note::Db4, Note::F4, Note::Ab4, Note::Db4, Note::F4, Note::Ab4,
        Note::Db3, Note::F3, Note::Ab3, Note::Db3, Note::F3, Note::Ab3 } },
};

// ── Atlas 5: Tonal Map (navigable 48-chord graph) ────────────────────────────
// Unlike the fixed atlases above, this journey is a dynamic graph: the current
// chord sits at the centre and its legal moves radiate outward, re-centring on
// each tap. Nodes, edge rules and voicings live in tonalmap.h; the journey
// entry below is a sentinel (nNodes = 0) that display.h and main.cpp detect.
#include "tonalmap.h"

// ── Journey table ────────────────────────────────────────────────────────────
// transitions: flattened [from*nNodes + to] move-labels, or nullptr to fall
// back to each chord's `feeling`. centerFirst: node 0 sits at the screen centre
// with the rest on a ring (a "home + satellites" map); false = a plain ring.
struct HarmonicJourney {
    const char*          name;        // shown in the picker and atop the map
    uint8_t              nNodes;      // 0 = dynamic Tonal Map (see tonalmap.h)
    bool                 centerFirst; // node 0 centred (home) vs. all on a ring
    const HarmonicChord* chords;
    const char* const*   transitions; // nullptr → use chord.feeling
    bool                 typeColor;   // colour nodes by chord-type suffix
                                       // (m/7/+/plain), Tonal-Map style, instead
                                       // of the plain white/grey scheme
    bool                 ringEdges;   // draw the ring-to-ring lines between
                                       // adjacent outer nodes, in addition to
                                       // the centre spokes (centerFirst only)
};

static constexpr uint8_t NUM_HARMONIC_JOURNEYS = 6;
static constexpr uint8_t JOURNEY_TONAL_MAP     = 5; // index of the dynamic map
static const HarmonicJourney HARMONIC_JOURNEYS[NUM_HARMONIC_JOURNEYS] = {
    { "Diatonic",   7, true,  HARMONIC_CHORDS,            &HARMONIC_TRANSITIONS[0][0], false, true  },
    { "Flamenco",   7, true,  FLAMENCO_CHORDS,            nullptr, false, true  },
    { "Jazz",       8, true,  JAZZ_CHORDS,                nullptr, false, true  },
    { "Cinematic",  7, true,  CINEMATIC_CHORDS,           nullptr, true,  false }, // spokes only, no hexagon
    { "Hexatonic",  6, false, CINEMATIC_HEXATONIC_CHORDS, nullptr, false, true  }, // ring only, no centre
    { "Tonal Map",  0, false, nullptr,                    nullptr, false, false },
};

// Update timing. Bench-measured (sensor_characterization `u`): the production
// burst reads take 4.71 ms/frame at 100 kHz I2C — that was 59% of the old 8 ms
// budget. Dropping the unused touch-status read (−2 transactions) and moving
// the chip to ESI=1 ms (fresh data every 4 ms) makes a 6 ms frame both
// feasible and worthwhile: ~167 Hz sensing, ~25% less scan-quantization lag.
// Don't go below ~6 without shrinking the reads further (e.g. caching the
// baseline registers) — the bus physically can't do the full read set faster.
static constexpr uint32_t UPDATE_MS = 6;
