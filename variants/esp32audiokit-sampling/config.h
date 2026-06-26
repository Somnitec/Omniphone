#pragma once
#include <stdint.h>
#include "driver/i2s.h"

// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32 Audio Kit (v2.2 / A247) SAMPLER · configuration
//
// Board: AI-Thinker ESP32-A1S Audio Kit, ESP32-WROVER (8 MB PSRAM), ES8388 codec.
// This is the live-sampling variant: record short sounds off the onboard mic and
// re-trigger them, either as one-shots (Mode A) or as an endless granular drone
// (Mode B).
//
// PHASE 1 (this build): six onboard buttons stand in for the eventual cap pads —
// no MPR121 yet. KEY1/KEY2 are "mode" modifiers; KEY3–KEY6 are the four sample
// slots. See sampler.cpp for the interaction model.
//
// Edit this file to retune mic gain, sample length, silence trimming, and the
// granular engine. Pins are fixed by the Audio Kit hardware (see table below).
// ─────────────────────────────────────────────────────────────────────────────

// ── Hardware pins (fixed by the Audio Kit — do not change without rewiring) ───
//
//   I²S to/from the ES8388 codec:
//     MCLK  GPIO0   master clock the ESP32 feeds the codec (256·fs)
//     BCLK  GPIO27  bit clock
//     LRCK  GPIO25  word-select / frame clock
//     DOUT  GPIO26  ESP32 → codec DAC (playback to speaker/headphone)
//     DIN   GPIO35  codec ADC → ESP32 (record from mics; input-only pin)
//   I²C to the codec (and, later, to MPR121 + screen touch — same bus):
//     SDA   GPIO33   SCL  GPIO32
//   Speaker amplifier enable (drive HIGH or the speaker stays silent):
//     PA_EN GPIO21
static constexpr int PIN_I2S_MCLK = 0;
static constexpr int PIN_I2S_BCLK = 27;
static constexpr int PIN_I2S_LRCK = 25;
static constexpr int PIN_I2S_DOUT = 26;   // to codec DAC
static constexpr int PIN_I2S_DIN  = 35;   // from codec ADC (mics)

static constexpr int PIN_I2C_SDA  = 33;
static constexpr int PIN_I2C_SCL  = 32;
static constexpr uint32_t I2C_HZ  = 400000;   // on-board codec → short traces

static constexpr int PIN_PA_EN    = 21;   // speaker amp enable (active HIGH)

// Onboard LED lit while a slot is recording. GPIO22 is the freely-usable status
// LED (often silk-screened D4). If your "D3" is a different LED, set its GPIO
// here — but avoid 19 (that's KEY3, a button input).
static constexpr int PIN_REC_LED  = 22;

// ── Onboard buttons (Phase-1 stand-ins for cap pads) ─────────────────────────
// KEY1 (GPIO36) and KEY2 (GPIO39/35 vary by rev — A247 uses 13) are mode
// modifiers; KEY3–KEY6 are the sample slots. KEY1 = IO36 is INPUT-ONLY with NO
// internal pull-up, so it relies on the board's external pull-up. The rest get
// INPUT_PULLUP for safety. All buttons are active-LOW (pressed = 0).
static constexpr int PIN_KEY1 = 36;   // modifier → Mode A (one-shot)
static constexpr int PIN_KEY2 = 13;   // modifier → Mode B (granular)
static constexpr int PIN_KEY_SLOT[] = { 19, 23, 18, 5 };   // KEY3..KEY6
static constexpr uint8_t NUM_SLOTS = 4;

static constexpr uint16_t BUTTON_DEBOUNCE_MS = 20;

// ── Audio format ─────────────────────────────────────────────────────────────
static constexpr int      SAMPLE_RATE = 44100;     // Hz, mono record / stereo out
static constexpr uint16_t AUDIO_BLOCK = 128;       // frames per I²S read/write
static constexpr uint8_t  MIC_GAIN    = 0x88;      // ES8388 ADCCONTROL1 (+24 dB)
// Which ADC input the mic is wired to (ADCCONTROL2 LINSEL/RINSEL):
//   0x00 = LINPUT1/RINPUT1   0x50 = LINPUT2/RINPUT2   0xF0 = differential
// Use AUDIO_SELFTEST 2 (it auto-cycles inputs) to find the one with real level.
static constexpr uint8_t  MIC_INPUT   = 0x50;
static constexpr uint8_t  OUTPUT_VOLUME = 75;      // 0–100 analog out level
// The onboard mics are quiet; ALC adds make-up gain BUT pumps the noise floor up
// during quiet moments ("breathing" hiss). We default it OFF and use per-sample
// normalization instead (constant gain → no pumping). Turn ON only for very
// quiet sources where you can tolerate the artifact.
#define MIC_USE_ALC 0
// ALC make-up gain 0–7 (≈6 dB/step, 7 = +35.5 dB). Lower this if idle hiss is
// loud; raise it if quiet sources don't record loud enough.
static constexpr uint8_t MIC_ALC_MAXGAIN = 5;
// Gate idle hiss between sounds. ON = quieter silences but can clip soft tails.
#define MIC_NOISE_GATE 0

// ── Sample storage (PSRAM) ───────────────────────────────────────────────────
// One mono 16-bit buffer per slot, pre-allocated in PSRAM at boot.
//   44.1 kHz mono = 88.2 KB/s, so 6 s = ~529 KB/slot × 4 = ~2.1 MB (of 8 MB).
// Bump MAX_SAMPLE_SEC freely — the WROVER has the headroom.
static constexpr float    MAX_SAMPLE_SEC = 6.0f;
static constexpr uint32_t MAX_SAMPLE_FRAMES =
    (uint32_t)(SAMPLE_RATE * MAX_SAMPLE_SEC);

// ── Silence trimming ─────────────────────────────────────────────────────────
// Leading silence is trimmed: playback starts at the first frame whose |level|
// exceeds SILENCE_LEVEL. A recording must cross the gate at least once or it is
// discarded as empty.
static constexpr int16_t  SILENCE_LEVEL = 1000;    // above the IN2 mic noise floor (~400)
static constexpr uint32_t TRIM_PAD_FRAMES = 64;    // keep a few frames around onset

// ── Per-recording normalization ──────────────────────────────────────────────
// After trimming, each sample is scaled by a CONSTANT gain so its peak reaches
// NORM_TARGET — consistent playback level without ALC's pumping. The gain is
// capped so a near-silent (noise-only) recording isn't blasted up to pure hiss;
// loud, close-mic recordings get the cleanest result (least boost needed).
static constexpr float    NORM_TARGET   = 26000.0f;  // ~0.8 of full scale
static constexpr float    NORM_MAX_GAIN = 16.0f;     // ceiling on the boost

// ── Playback / granular engine ───────────────────────────────────────────────
// Mode A (one-shot): play start→end once at unity pitch, short fade at the tail.
// Mode B (granular):  overlapping windowed grains sustain a tone while the key is
// held. The grain "playhead" scans through the whole sample and LOOPS back to the
// start, so the timbre keeps evolving (never freezes). Per-grain position scatter
// and slight detune break up the periodic "robotic" comb artifact.
static constexpr uint8_t  GRAINS_PER_VOICE   = 4;       // overlap count (smoother)
static constexpr float    GRAIN_MS           = 80.0f;   // grain length
static constexpr float    PLAYHEAD_ADVANCE   = 0.25f;   // playhead scan speed (×realtime)
static constexpr float    GRAIN_SCATTER_MS   = 35.0f;   // ± random spawn-position spread
static constexpr float    GRAIN_PITCH_JITTER = 0.04f;   // ± per-grain detune (4% ≈ ⅔ semitone)
static constexpr float    ENV_ATTACK_MS      = 8.0f;    // gate-on ramp
static constexpr float    ENV_RELEASE_MS     = 180.0f;  // gate-off tail
static constexpr float    ONESHOT_FADE_MS    = 5.0f;    // anti-click tail (Mode A)

static constexpr float    VOICE_GAIN  = 0.7f;          // per-voice headroom
static constexpr float    MASTER_GAIN = 0.9f;          // final mix trim

// ── Audio path self-test ─────────────────────────────────────────────────────
// Bypass the sampler logic to isolate a fault. Modes:
//   0 = normal sampler (real instrument)
//   1 = sine tone → speaker. Confirms OUTPUT path (DAC, I²S, PA, speaker).
//   2 = mic → speaker live passthrough + level meter. Confirms INPUT path
//       (ADC, mic, full-duplex I²S read). Talk/tap near the mic; you should
//       hear yourself and see "mic peak=…" climb in the serial log.
// Workflow: 1 first (proves output), then 2 (proves input), then 0.
// (Mode 2 now AUTO-CYCLES the ADC input every ~3 s to find where the mic is.)
#define AUDIO_SELFTEST 0
static constexpr float SELFTEST_HZ = 440.0f;   // tone frequency (mode 1)

// ── Button discovery ─────────────────────────────────────────────────────────
// Set to 1 to find which GPIO a physical button is really on (the Audio Kit's
// KEY2–KEY6 share pins with JTAG/SD and vary by board rev). It scans a list of
// candidate GPIOs and prints any that go LOW — press a button and read off its
// real pin, then fix PIN_KEY* above and set this back to 0. Normal sampler /
// gesture logic is suspended while this is 1.
#define BUTTON_DISCOVERY 0

// ── Serial diagnostics ───────────────────────────────────────────────────────
#define SERIAL_DEBUG 1
static constexpr uint32_t SERIAL_BAUD = 115200;

// Slot playback mode, remembered per slot from how it was recorded.
enum SlotMode : uint8_t { MODE_ONESHOT = 0, MODE_GRANULAR = 1 };
