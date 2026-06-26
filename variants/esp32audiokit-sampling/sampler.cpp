// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32 Audio Kit (v2.2 / A247) live SAMPLER
//
// Record short sounds off the onboard mic and re-trigger them in one of two
// modes. Phase 1 uses the six onboard buttons as stand-ins for the eventual
// MPR121 cap pads.
//
// ── Interaction model ────────────────────────────────────────────────────────
//   KEY1 + KEY3–6   hold KEY1 and hold a slot key → RECORD into that slot.
//                   Release the slot key to finish. The slot becomes a one-shot:
//                   later, a plain TAP plays it start→end once.   (Mode A)
//
//   KEY2 + KEY3–6   same gesture, but the slot becomes a granular drone: later,
//                   HOLDING the slot key sustains an endless tone (starting at
//                   the sample's start, easing into the tail), and releasing
//                   tails it off.                                  (Mode B)
//
//   KEY3–6 alone    play the slot according to the mode it was recorded in.
//
// Leading silence is trimmed off every recording automatically.
//
// ── Architecture ─────────────────────────────────────────────────────────────
//   Core 1: audioTask — full-duplex I²S. Each block it (1) reads a mic block and
//           appends to the recording slot if any, (2) mixes the four voices,
//           (3) writes the block to the codec. Owns all DSP state.
//   Core 0: loop() — debounces the buttons and posts commands to the audio task
//           via per-slot volatile control flags. No DSP on this core.
//
// Build:  pio run -e esp32audiokit-sampling -t upload
//         pio device monitor -e esp32audiokit-sampling
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "driver/i2s.h"

#include "config.h"
#include "es8388.h"

#if SERIAL_DEBUG
  #define LOG(...)  Serial.printf(__VA_ARGS__)
#else
  #define LOG(...)  do {} while (0)
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Sample slots (PSRAM) + cross-core control
// ─────────────────────────────────────────────────────────────────────────────
struct Slot {
    int16_t* buf      = nullptr;   // mono PSRAM buffer, MAX_SAMPLE_FRAMES
    uint32_t length   = 0;         // valid frames (after trim)
    bool     recorded = false;
    SlotMode mode     = MODE_ONESHOT;
    float    gain     = 1.0f;      // per-sample normalization gain

    // Cross-core commands (set by loop(), consumed by audioTask).
    volatile bool     cmdRecStart = false;   // begin recording
    volatile bool     cmdRecStop  = false;   // finish recording
    volatile SlotMode cmdRecMode  = MODE_ONESHOT;
    volatile bool     cmdTrigger  = false;   // one-shot play (Mode A)
    volatile bool     gate        = false;   // held state (Mode B)
};
static Slot slots[NUM_SLOTS];

// Which slot is currently capturing the mic (-1 = none). Audio-task-owned.
static int       recSlot      = -1;
static uint32_t  recWritePos  = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Voices — one per slot. A voice can be a one-shot player or a granular cloud.
// ─────────────────────────────────────────────────────────────────────────────
struct Grain {
    float    phase;     // 0..grainLen, position within the grain window
    float    srcPos;    // source frame this grain started reading from
    float    rate;      // per-grain playback rate (slight detune for richness)
    bool     active;
};

struct Voice {
    bool     active   = false;
    bool     granular = false;

    // one-shot
    float    pos      = 0.0f;     // read index (unity pitch ⇒ +1/frame)

    // granular
    float    playhead = 0.0f;     // source position the grains spawn around
    Grain    grains[GRAINS_PER_VOICE];
    uint16_t spawnTimer = 0;      // frames until next grain launch

    // shared envelope
    float    env      = 0.0f;     // current amplitude 0..1
    bool     gateOn   = false;
};
static Voice voices[NUM_SLOTS];

// Precomputed engine constants (frames).
static float    grainLenF;        // grain length in frames
static uint16_t grainHop;         // frames between grain launches
static float    grainScatterF;    // ± spawn-position scatter in frames
static float    envAtkStep;       // per-frame attack increment
static float    envRelStep;       // per-frame release decrement
static float    oneshotFadeFrames;

// Fast signed random in [-1, 1) — cheap xorshift, fine at grain-spawn rate.
static inline float randSigned() {
    static uint32_t st = 0x1234567u;
    st ^= st << 13; st ^= st >> 17; st ^= st << 5;
    return (float)(int32_t)st * (1.0f / 2147483648.0f);
}

// Hann window lookup (one grain envelope).
static constexpr uint16_t WIN_SIZE = 512;
static float winTable[WIN_SIZE];

static ES8388 codec;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline float windowAt(float phase01) {
    // phase01 in 0..1 → Hann amplitude.
    int idx = (int)(phase01 * (WIN_SIZE - 1));
    if (idx < 0) idx = 0; else if (idx >= WIN_SIZE) idx = WIN_SIZE - 1;
    return winTable[idx];
}

static inline float readSampleLinear(const int16_t* b, uint32_t len, float pos) {
    if (pos < 0) pos = 0;
    uint32_t i = (uint32_t)pos;
    if (i + 1 >= len) return (len ? (float)b[len - 1] : 0.0f);
    float frac = pos - (float)i;
    return (1.0f - frac) * (float)b[i] + frac * (float)b[i + 1];
}

// ─────────────────────────────────────────────────────────────────────────────
// Recording: trim leading silence, then commit the slot.
// ─────────────────────────────────────────────────────────────────────────────
static void finalizeRecording(int s, uint32_t rawLen) {
    Slot& slot = slots[s];

    // Find first frame above the silence gate (leading trim).
    uint32_t onset = 0;
    while (onset < rawLen && abs(slot.buf[onset]) < SILENCE_LEVEL) onset++;

    if (onset >= rawLen) {                 // never crossed the gate → empty
        slot.recorded = false;
        slot.length   = 0;
        LOG("# slot %d: recording was silent, discarded\n", s + 1);
        return;
    }

    // Find last frame above the gate (trailing trim) so the tail isn't silence.
    uint32_t tail = rawLen;
    while (tail > onset && abs(slot.buf[tail - 1]) < SILENCE_LEVEL) tail--;

    // Keep a little padding around onset and tail.
    uint32_t start = (onset > TRIM_PAD_FRAMES) ? onset - TRIM_PAD_FRAMES : 0;
    uint32_t end   = tail + TRIM_PAD_FRAMES;
    if (end > rawLen) end = rawLen;
    uint32_t trimmed = end - start;

    // Shift the kept region to the front of the buffer (in place).
    if (start > 0)
        memmove(slot.buf, slot.buf + start, trimmed * sizeof(int16_t));

    // Peak-normalize: constant gain so the loudest frame hits NORM_TARGET, capped.
    int32_t peak = 1;
    for (uint32_t i = 0; i < trimmed; i++) {
        int32_t a = abs(slot.buf[i]);
        if (a > peak) peak = a;
    }
    float g = NORM_TARGET / (float)peak;
    if (g > NORM_MAX_GAIN) g = NORM_MAX_GAIN;   // don't blast near-silent takes
    slot.gain = g;

    slot.length   = trimmed;
    slot.recorded = true;
    LOG("# slot %d: recorded %.2fs (peak=%ld, norm×%.1f), mode=%s\n",
        s + 1, trimmed / (float)SAMPLE_RATE, (long)peak, g,
        slot.mode == MODE_GRANULAR ? "granular" : "one-shot");
}

// ─────────────────────────────────────────────────────────────────────────────
// Voice triggering
// ─────────────────────────────────────────────────────────────────────────────
static void triggerOneShot(int s) {
    if (!slots[s].recorded) return;
    Voice& v = voices[s];
    v.active   = true;
    v.granular = false;
    v.pos      = 0.0f;
    v.env      = 1.0f;
    v.gateOn   = false;   // one-shots free-run to the end
}

static void startGranular(int s) {
    if (!slots[s].recorded) return;
    Voice& v = voices[s];
    v.active     = true;
    v.granular   = true;
    v.playhead   = 0.0f;
    v.gateOn     = true;
    v.spawnTimer = 0;
    for (auto& g : v.grains) g.active = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-block voice rendering → accumulates into `mix` (length AUDIO_BLOCK).
// ─────────────────────────────────────────────────────────────────────────────
static void renderVoice(int s, float* mix) {
    Voice& v = voices[s];
    if (!v.active) return;
    const Slot& slot = slots[s];
    const int16_t* b = slot.buf;
    const uint32_t len = slot.length;

    for (uint16_t n = 0; n < AUDIO_BLOCK; n++) {
        // ── envelope ─────────────────────────────────────────────────────────
        if (v.gateOn) {
            v.env += envAtkStep;  if (v.env > 1.0f) v.env = 1.0f;
        } else if (v.granular) {
            v.env -= envRelStep;  if (v.env < 0.0f) v.env = 0.0f;
        }

        float out = 0.0f;

        if (!v.granular) {
            // ── one-shot ─────────────────────────────────────────────────────
            if (v.pos >= (float)len) { v.active = false; break; }
            out = readSampleLinear(b, len, v.pos) * v.env;
            v.pos += 1.0f;
            // anti-click fade over the last few ms
            float remaining = (float)len - v.pos;
            if (remaining < oneshotFadeFrames)
                out *= (remaining / oneshotFadeFrames);
        } else {
            // ── granular cloud ───────────────────────────────────────────────
            // Scan the playhead through the whole sample and LOOP — the timbre
            // keeps evolving instead of freezing at the end.
            float scanEnd = (len > (uint32_t)(2 * grainLenF))
                            ? (float)len - 2 * grainLenF : (float)len;
            v.playhead += PLAYHEAD_ADVANCE;
            if (v.playhead >= scanEnd) v.playhead -= scanEnd;   // wrap to start

            // Launch a new grain every grainHop frames, scattered around the
            // playhead and slightly detuned so it doesn't comb into a robotic buzz.
            if (v.spawnTimer == 0) {
                for (auto& g : v.grains) {
                    if (!g.active) {
                        g.active = true;
                        g.phase  = 0.0f;
                        float p  = v.playhead + grainScatterF * randSigned();
                        if (p < 0) p = 0;
                        if (p > (float)len - 2) p = (float)len - 2;
                        g.srcPos = p;
                        g.rate   = 1.0f + GRAIN_PITCH_JITTER * randSigned();
                        break;
                    }
                }
                v.spawnTimer = grainHop;
            }
            v.spawnTimer--;

            // Sum active grains (each advances its source at its own rate).
            for (auto& g : v.grains) {
                if (!g.active) continue;
                float w = windowAt(g.phase / grainLenF);
                out += readSampleLinear(b, len, g.srcPos + g.phase * g.rate) * w;
                g.phase += 1.0f;
                if (g.phase >= grainLenF) g.active = false;
            }
            out *= v.env;

            // Voice ends only after the release has fully decayed.
            if (!v.gateOn && v.env <= 0.0001f) { v.active = false; break; }
        }

        mix[n] += out * VOICE_GAIN * slot.gain;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Audio task (core 1): full-duplex I²S — read mic, record, mix, write out.
// ─────────────────────────────────────────────────────────────────────────────
static void audioTask(void*) {
    static int16_t rx[AUDIO_BLOCK * 2];   // stereo mic frames (L,R interleaved)
    static int16_t tx[AUDIO_BLOCK * 2];   // stereo out
    static float   mix[AUDIO_BLOCK];

#if AUDIO_SELFTEST == 1
    // ── self-test 1: stream a sine to the codec (proves OUTPUT path) ──────────
    float phase = 0.0f;
    const float phaseInc = 2.0f * (float)M_PI * SELFTEST_HZ / SAMPLE_RATE;
    uint32_t blocks = 0;
    for (;;) {
        for (uint16_t n = 0; n < AUDIO_BLOCK; n++) {
            int16_t s = (int16_t)(sinf(phase) * 18000.0f);   // ~0.55 full scale
            tx[2 * n] = tx[2 * n + 1] = s;
            phase += phaseInc;
            if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
        }
        size_t wrote = 0;
        i2s_write(I2S_NUM_0, tx, sizeof(tx), &wrote, portMAX_DELAY);
        // blocks should climb ~344/s (44100/128). If STUCK, I²S isn't draining.
        if ((++blocks % 344) == 0)
            LOG("# selftest1: %.0f Hz sine, blocks=%lu (wrote=%u)\n",
                SELFTEST_HZ, (unsigned long)blocks, (unsigned)wrote);
    }
#elif AUDIO_SELFTEST == 2
    // ── self-test 2: passthrough + peak meter, AUTO-CYCLING the ADC input ─────
    // Every ~3 s it switches LINPUT1 → LINPUT2 → differential and reports the
    // per-channel peak. Make a steady noise (talk/tap) and watch which input
    // shows a big peak (and sounds loud) — that's where the mic is. Put that
    // value in MIC_INPUT above.
    const uint8_t  inputs[]   = { 0x00, 0x50, 0xF0 };
    const char*    inputNames[] = { "IN1 (LIN1/RIN1)", "IN2 (LIN2/RIN2)", "DIFFERENTIAL" };
    int inIdx = 0;
    codec.setAdcInput(inputs[inIdx]);
    LOG("# selftest2: testing ADC input %s — make continuous noise\n", inputNames[inIdx]);

    uint32_t blocks = 0;
    int16_t  peakL = 0, peakR = 0;
    for (;;) {
        size_t got = 0;
        i2s_read(I2S_NUM_0, rx, sizeof(rx), &got, portMAX_DELAY);
        uint16_t frames = got / (2 * sizeof(int16_t));
        for (uint16_t n = 0; n < frames; n++) {
            int16_t l = rx[2 * n];                 // left  ADC channel
            int16_t r = rx[2 * n + 1];             // right ADC channel
            int16_t al = l < 0 ? -l : l;  if (al > peakL) peakL = al;
            int16_t ar = r < 0 ? -r : r;  if (ar > peakR) peakR = ar;
            tx[2 * n] = tx[2 * n + 1] = l;         // monitor the left channel
        }
        size_t wrote = 0;
        i2s_write(I2S_NUM_0, tx, frames * 2 * sizeof(int16_t), &wrote, portMAX_DELAY);

        if ((++blocks % 344) == 0) {               // ~once/sec
            LOG("#   %s: peak L=%d R=%d\n", inputNames[inIdx], peakL, peakR);
            peakL = peakR = 0;
        }
        if (blocks % (344 * 3) == 0) {             // ~every 3 s: next input
            inIdx = (inIdx + 1) % 3;
            codec.setAdcInput(inputs[inIdx]);
            LOG("# → switching to ADC input %s\n", inputNames[inIdx]);
        }
    }
#endif

    for (;;) {
        // ── 1. read a mic block ──────────────────────────────────────────────
        size_t got = 0;
        i2s_read(I2S_NUM_0, rx, sizeof(rx), &got, portMAX_DELAY);
        uint16_t frames = got / (2 * sizeof(int16_t));

        // ── 2. service record commands ───────────────────────────────────────
        for (int s = 0; s < NUM_SLOTS; s++) {
            if (slots[s].cmdRecStart) {
                slots[s].cmdRecStart = false;
                if (recSlot < 0) {                       // ignore if one is busy
                    recSlot      = s;
                    recWritePos  = 0;
                    slots[s].mode = slots[s].cmdRecMode;
                    LOG("# slot %d: recording...\n", s + 1);
                }
            }
            if (slots[s].cmdRecStop && recSlot == s) {
                slots[s].cmdRecStop = false;
                finalizeRecording(s, recWritePos);
                recSlot = -1;
            }
        }

        // ── 3. capture into the active record slot ───────────────────────────
        // Average the two adjacent mics (both on input 2, L & R) — same sound,
        // partly independent noise → a few dB cleaner than either alone.
        if (recSlot >= 0) {
            int16_t* dst = slots[recSlot].buf;
            for (uint16_t f = 0; f < frames && recWritePos < MAX_SAMPLE_FRAMES; f++)
                dst[recWritePos++] = (int16_t)(((int32_t)rx[2 * f] + rx[2 * f + 1]) / 2);
            if (recWritePos >= MAX_SAMPLE_FRAMES) {       // hit the ceiling
                finalizeRecording(recSlot, recWritePos);
                recSlot = -1;
            }
        }

        // ── 4. service playback commands ─────────────────────────────────────
        for (int s = 0; s < NUM_SLOTS; s++) {
            if (slots[s].cmdTrigger) {
                slots[s].cmdTrigger = false;
                triggerOneShot(s);
            }
            Voice& v = voices[s];
            if (slots[s].gate && !(v.active && v.granular)) {
                startGranular(s);                         // (re)start granular
            } else if (slots[s].gate) {
                v.gateOn = true;                          // keep sustaining
            } else if (v.active && v.granular) {
                v.gateOn = false;                         // release → tail off
            }
        }

        // ── 5. mix all voices ────────────────────────────────────────────────
        for (uint16_t n = 0; n < AUDIO_BLOCK; n++) mix[n] = 0.0f;
        for (int s = 0; s < NUM_SLOTS; s++) renderVoice(s, mix);

        // ── 6. soft-limit + write stereo out ─────────────────────────────────
        for (uint16_t n = 0; n < AUDIO_BLOCK; n++) {
            float x = mix[n] * MASTER_GAIN;
            x = tanhf(x * (1.0f / 32768.0f)) * 32767.0f;  // gentle saturation
            int16_t s16 = (int16_t)x;
            tx[2 * n]     = s16;     // L
            tx[2 * n + 1] = s16;     // R
        }
        size_t wrote = 0;
        i2s_write(I2S_NUM_0, tx, sizeof(tx), &wrote, portMAX_DELAY);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// I²S full-duplex bring-up
// ─────────────────────────────────────────────────────────────────────────────
static bool i2sBegin() {
    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;   // stereo
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = 0;
    cfg.dma_buf_count        = 8;
    cfg.dma_buf_len          = AUDIO_BLOCK;
    cfg.use_apll             = true;     // cleaner MCLK for the codec
    cfg.tx_desc_auto_clear   = true;
    if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) return false;

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = PIN_I2S_MCLK;    // GPIO0 supplies 256·fs to the codec
    pins.bck_io_num   = PIN_I2S_BCLK;
    pins.ws_io_num    = PIN_I2S_LRCK;
    pins.data_out_num = PIN_I2S_DOUT;
    pins.data_in_num  = PIN_I2S_DIN;
    if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) return false;

    i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Engine constant setup
// ─────────────────────────────────────────────────────────────────────────────
static void engineBegin() {
    grainLenF         = GRAIN_MS * 0.001f * SAMPLE_RATE;
    grainHop          = (uint16_t)(grainLenF / GRAINS_PER_VOICE);
    if (grainHop < 1) grainHop = 1;
    grainScatterF     = GRAIN_SCATTER_MS * 0.001f * SAMPLE_RATE;
    envAtkStep        = 1.0f / (ENV_ATTACK_MS  * 0.001f * SAMPLE_RATE);
    envRelStep        = 1.0f / (ENV_RELEASE_MS * 0.001f * SAMPLE_RATE);
    oneshotFadeFrames = ONESHOT_FADE_MS * 0.001f * SAMPLE_RATE;

    for (uint16_t i = 0; i < WIN_SIZE; i++)
        winTable[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (WIN_SIZE - 1)));
}

// ─────────────────────────────────────────────────────────────────────────────
// PSRAM slot allocation
// ─────────────────────────────────────────────────────────────────────────────
static bool allocSlots() {
    for (int s = 0; s < NUM_SLOTS; s++) {
        slots[s].buf = (int16_t*)ps_malloc(MAX_SAMPLE_FRAMES * sizeof(int16_t));
        if (!slots[s].buf) {
            LOG("# FATAL: ps_malloc failed for slot %d\n", s + 1);
            return false;
        }
    }
    LOG("# allocated %d × %.2fs slots in PSRAM (%.1f KB each)\n",
        NUM_SLOTS, MAX_SAMPLE_SEC,
        MAX_SAMPLE_FRAMES * sizeof(int16_t) / 1024.0f);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Button handling (core 0)
// ─────────────────────────────────────────────────────────────────────────────
struct Button {
    int      pin;
    bool     stable    = true;    // debounced level (true = released, active-LOW)
    bool     lastRead  = true;
    uint32_t lastEdge  = 0;
};
static Button bKey1, bKey2, bSlot[NUM_SLOTS];

static void buttonInit() {
    pinMode(PIN_KEY1, INPUT);          // GPIO36: input-only, board pull-up
    pinMode(PIN_KEY2, INPUT_PULLUP);
    bKey1.pin = PIN_KEY1;
    bKey2.pin = PIN_KEY2;
    for (int s = 0; s < NUM_SLOTS; s++) {
        pinMode(PIN_KEY_SLOT[s], INPUT_PULLUP);
        bSlot[s].pin = PIN_KEY_SLOT[s];
    }
}

// Debounce one button; returns true while held (pressed = LOW).
static bool buttonPressed(Button& b) {
    bool raw = (digitalRead(b.pin) == LOW);   // active-LOW
    if (raw != b.lastRead) { b.lastRead = raw; b.lastEdge = millis(); }
    if (millis() - b.lastEdge >= BUTTON_DEBOUNCE_MS) b.stable = raw;
    return b.stable;
}

void setup() {
#if SERIAL_DEBUG
    Serial.begin(SERIAL_BAUD);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 1200) {}
    Serial.println(F("\n# ── Omniphone ESP32 Audio Kit SAMPLER ──"));
#endif

    // Speaker amp on.
    pinMode(PIN_PA_EN, OUTPUT);
    digitalWrite(PIN_PA_EN, HIGH);

    // Recording-indicator LED.
    pinMode(PIN_REC_LED, OUTPUT);
    digitalWrite(PIN_REC_LED, LOW);

    // I²C → codec.
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ);
    bool codecOk = codec.begin(MIC_GAIN);
    codec.setAdcInput(MIC_INPUT);
#if MIC_USE_ALC
    codec.enableMicALC(MIC_ALC_MAXGAIN, MIC_NOISE_GATE);
#endif
    codec.setOutputVolume(OUTPUT_VOLUME);
    LOG("# ES8388 init: %s%s\n", codecOk ? "OK" : "FAIL (check I2C 33/32)",
        MIC_USE_ALC ? " (mic ALC on)" : "");

    bool i2sOk = i2sBegin();
    LOG("# I2S full-duplex: %s (MCLK%d BCK%d LRCK%d DOUT%d DIN%d)\n",
        i2sOk ? "OK" : "FAIL",
        PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_LRCK, PIN_I2S_DOUT, PIN_I2S_DIN);

    engineBegin();
    if (!allocSlots()) { LOG("# halted.\n"); for (;;) delay(1000); }

    buttonInit();

    xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 4, nullptr, 1);
    LOG("# ready. KEY1+slot = record one-shot · KEY2+slot = record granular · "
        "slot = play\n");
}

// Log a button's press/release edges (confirms wiring + mapping).
static void logButtonEdge(const char* name, int gpio, bool was, bool now) {
    if (now && !was) LOG("# BTN %s (GPIO%d): PRESS\n",   name, gpio);
    if (!now && was) LOG("# BTN %s (GPIO%d): RELEASE\n", name, gpio);
}

#if BUTTON_DISCOVERY
// Candidate input-capable GPIOs NOT used by I²S (0/27/25/26/35), I²C (33/32),
// or PA (21). 34/36/39 are input-only (no pull-up — rely on board pull-ups).
static const int DISCOVERY_PINS[] = { 36, 39, 34, 13, 19, 23, 18, 5,
                                      4, 2, 15, 12, 14, 22 };
static void buttonDiscoveryLoop() {
    static bool inited = false;
    static bool last[sizeof(DISCOVERY_PINS) / sizeof(int)];
    if (!inited) {
        for (size_t i = 0; i < sizeof(DISCOVERY_PINS) / sizeof(int); i++) {
            int p = DISCOVERY_PINS[i];
            pinMode(p, (p == 34 || p == 36 || p == 39) ? INPUT : INPUT_PULLUP);
            last[i] = (digitalRead(p) == LOW);   // seed with the real idle level
        }
        inited = true;
        LOG("# BUTTON DISCOVERY: press each physical key; the GPIO that goes "
            "LOW is its pin.\n");
    }
    for (size_t i = 0; i < sizeof(DISCOVERY_PINS) / sizeof(int); i++) {
        bool low = (digitalRead(DISCOVERY_PINS[i]) == LOW);
        if (low != last[i]) {
            LOG("# GPIO%-2d : %s\n", DISCOVERY_PINS[i], low ? "LOW  (pressed)" : "HIGH (released)");
            last[i] = low;
        }
    }
    delay(8);
}
#endif

void loop() {
#if BUTTON_DISCOVERY
    buttonDiscoveryLoop();
    return;
#endif
    bool key1Was = bKey1.stable;
    bool mod1 = buttonPressed(bKey1);     // → record as one-shot
    logButtonEdge("KEY1", PIN_KEY1, key1Was, mod1);
    if (mod1 && !key1Was) LOG("#   ↳ ARMED for ONE-SHOT record — now hold a slot key (KEY3-6)\n");
    if (!mod1 && key1Was) LOG("#   ↳ disarmed (KEY1 released)\n");

    bool key2Was = bKey2.stable;
    bool mod2 = buttonPressed(bKey2);     // → record as granular
    logButtonEdge("KEY2", PIN_KEY2, key2Was, mod2);
    if (mod2 && !key2Was) LOG("#   ↳ ARMED for GRANULAR record — now hold a slot key (KEY3-6)\n");
    if (!mod2 && key2Was) LOG("#   ↳ disarmed (KEY2 released)\n");

    bool modifier = mod1 || mod2;

    for (int s = 0; s < NUM_SLOTS; s++) {
        Button& bs = bSlot[s];
        bool wasPressed = bs.stable;      // debounced level BEFORE this read
        bool nowPressed = buttonPressed(bs);
        bool pressEdge   =  nowPressed && !wasPressed;
        bool releaseEdge = !nowPressed &&  wasPressed;

        char nm[8];
        snprintf(nm, sizeof(nm), "KEY%d", s + 3);   // slots are KEY3..KEY6
        logButtonEdge(nm, PIN_KEY_SLOT[s], wasPressed, nowPressed);

        if (modifier) {
            // ── recording gesture ────────────────────────────────────────────
            if (pressEdge) {
                slots[s].cmdRecMode  = mod1 ? MODE_ONESHOT : MODE_GRANULAR;
                slots[s].cmdRecStart = true;
                LOG("#   ↳ RECORDING slot %d (%s) — release this key to stop\n",
                    s + 1, mod1 ? "one-shot" : "granular");
            }
            if (releaseEdge && recSlot == s) {
                slots[s].cmdRecStop = true;
                LOG("#   ↳ stop slot %d — trimming leading silence...\n", s + 1);
            }
            slots[s].gate = false;        // never sound while arming
        } else {
            // ── playback gesture ─────────────────────────────────────────────
            if (!slots[s].recorded) {
                if (pressEdge)
                    LOG("#   ↳ slot %d is EMPTY — record it first (hold KEY1 or KEY2 + this key)\n",
                        s + 1);
                slots[s].gate = false;
            } else if (slots[s].mode == MODE_ONESHOT) {
                if (pressEdge) {
                    slots[s].cmdTrigger = true;
                    LOG("#   ↳ PLAY slot %d (one-shot)\n", s + 1);
                }
                slots[s].gate = false;
            } else {                       // granular: gate follows the held key
                if (pressEdge)   LOG("#   ↳ GRANULAR slot %d: drone ON (hold)\n", s + 1);
                if (releaseEdge) LOG("#   ↳ GRANULAR slot %d: release → tail off\n", s + 1);
                slots[s].gate = nowPressed;
            }
        }
    }

    // LED on while NOT recording (idle indicator).
    digitalWrite(PIN_REC_LED, recSlot < 0 ? HIGH : LOW);

    delay(2);   // ~500 Hz button scan; audio runs independently on core 1
}
