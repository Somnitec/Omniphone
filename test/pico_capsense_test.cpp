// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — Pico (RP2040) · BARE-PIN CAPACITIVE SENSING comparison test
//
// Two pin-only capsense methods on the SAME three consecutive GPIOs so they can
// be A/B'd back-to-back. Neither needs an external resistor — both lean on the
// RP2040's internal ~50 kΩ pull-up and the GPIO Schmitt-trigger input:
//
//   METHOD 1 — "jtouch": internal pull-up charge-time, self-timed CPU loop.
//     Discharge the pad (drive LOW), switch to INPUT_PULLUP, then spin-count how
//     many digitalRead() iterations pass before the pad charges past the input
//     threshold. Sum N samples. CPU-bound, blocking, jittery — the baseline.
//
//   METHOD 2 — relaxation oscillator + PIO frequency counter.
//     A PIO state machine free-runs as a relaxation oscillator on the pin
//     (discharge → let the pull-up recharge the pad → fire at the Schmitt
//     threshold → repeat) and reports the charge time, in PIO clock cycles, of
//     each cycle. Touch adds capacitance → longer charge → lower oscillation
//     frequency. Deterministic clock timing, runs in hardware, non-blocking.
//     All three pins oscillate at once → a hovering hand couples across them.
//
//   METHOD 3 — PIO sequential scan with grounded-guard neighbours.
//     Same PIO oscillator, but only ONE pad runs at a time while the other two
//     are driven LOW so they shield it (and the hand) instead of coupling to it.
//     The structural cure for mode-2's cross-pad bleed; trades simultaneity for
//     per-pad isolation.
//
// Both register touch as an INCREASE in the streamed value (charge count rises).
// A per-channel EMA (SMOOTH) and N-period averaging (M2_AVG) tame the small,
// noisy hover delta; a common-mode (channel-mean) subtraction is streamed too.
//
// Pins:  GP10, GP11, GP12 — three free, consecutive GPIOs on the 13-pad build
//        (I2S is on 20–22, LEDs on 6–8/13–19/26–28, MPR121 I2C on the Wire pins).
//
// Run ONE method at a time; switch live over USB serial:
//     1   → Method 1 (jtouch CPU charge-time)
//     2   → Method 2 (PIO relaxation oscillator, free-running on all 3)
//     3   → Method 3 (PIO sequential scan + grounded guards)
//     r   → re-snapshot the untouched baselines (keep hands off)
//
// Build:  pio run -e pico-capsense-test -t upload
// Plot:   Teleplot @115200. Per pin <gp>, prefixed by the active mode m<1|2|3>:
//           m<n>_p<gp>  smoothed value (charge cycles)
//           m<n>_d<gp>  delta above the untouched baseline
//           m<n>_c<gp>  delta with the common-mode (channel mean) removed
//           m<n>_f<gp>  oscillator frequency, kHz (PIO modes 2 & 3)
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

// ── Charge resistor: internal pull-up (touch only) vs external R (hover) ─────
// 0 = RP2040 internal ~50 kΩ pull-up. No external parts, but only a direct metal
//     TOUCH registers — 50 kΩ charges too fast to resolve the few pF of a hover.
// 1 = EXTERNAL high-value resistor (1 MΩ–10 MΩ) from each pin to 3V3, pad on the
//     pin. Disables the internal pull-up so it can't shunt the external R.
//     Sensitivity ∝ R:  1M = touch · 4.7M = ~1 cm · 10M = few cm · 40M = hand-wave.
//        3V3 ──[ R ]── GP10 ──── pad   (one resistor per pin)
#define EXTERNAL_PULLUP 1

// ── Pins under test ──────────────────────────────────────────────────────────
static const uint8_t SENSE_PINS[] = { 10, 11, 12 };
static constexpr uint8_t NUM_PINS = sizeof(SENSE_PINS) / sizeof(SENSE_PINS[0]);

static constexpr uint32_t TELE_PERIOD_MS = 30;   // ~33 Hz stream

// ── Method 1 (jtouch CPU charge-time) tuning ─────────────────────────────────
static constexpr uint8_t  M1_SAMPLES     = 30;    // cycles summed per reading (↑ = resolution, slower)
static constexpr uint32_t M1_TIMEOUT     = 50000; // per-sample spin cap (guards a floating pin).
                                                  // High-value external R charges slowly → big
                                                  // counts; keep this well above an idle reading.
static constexpr uint16_t M1_DISCHARGE_US = 10;   // settle time to fully drain the pad

// ── Method 2 (PIO relaxation oscillator) tuning ──────────────────────────────
// More periods averaged → finer FRACTIONAL resolution on the small hover delta
// (a hover only shifts the count a few PIO cycles; averaging recovers the bits
// between integers). Costs a little latency. 64 is a good noise/lag balance.
static constexpr uint8_t M2_AVG = 64;             // FIFO reads averaged per reading

// Per-channel EMA on the per-frame reading — extra smoothing on top of the
// averaging (lower = smoother/laggier). 1.0 = off.
static constexpr float SMOOTH = 0.40f;

// Mode 3 only: how many pads to oscillate AT ONCE per bank. The scan re-points a
// pool of SMs across all pads, so pad count is NOT capped by SM count — only by
// free GPIOs. 1 = pure sequential (one SM, slowest, max isolation). 2–4 = banked
// parallel: pads in a bank are spaced far apart so they don't couple, while the
// rest are grounded guards → ~×SCAN_PARALLEL faster full-scan. Capped at 4 (one
// PIO block's worth of SMs; leaves the other block free for Mozzi's I2S).
static constexpr uint8_t SCAN_PARALLEL = 1;
static_assert(SCAN_PARALLEL >= 1 && SCAN_PARALLEL <= 4, "SCAN_PARALLEL must be 1..4");

// ═══════════════════════ PIO PROGRAM — relaxation oscillator ════════════════
// Hand-assembled. Per cycle: drive the pin LOW to discharge, switch to input so
// the internal pull-up recharges the pad, count PIO cycles until the pin reads
// HIGH (Schmitt threshold), push the count, repeat (→ self-oscillating).
//
//   .wrap_target
//       set  pindirs, 1      ; pin = output
//       set  pins, 0         ; drive LOW → discharge
//       set  y, 31           ; short discharge-settle loop
//   disc:
//       jmp  y--, disc
//       set  pindirs, 0      ; pin = input → pull-up recharges the pad
//       mov  x, ~null        ; x = 0xFFFFFFFF
//   chg:
//       jmp  pin, done       ; pin past threshold? stop counting
//       jmp  x--, chg        ; else keep counting (2 PIO cycles / iteration)
//   done:
//       mov  isr, ~x         ; charge cycles = 0xFFFFFFFF - x
//       push noblock         ; hand it to the RX FIFO (drop if full)
//   .wrap
static const uint16_t capsense_program_instructions[] = {
    0xE081, // 0: set pindirs, 1
    0xE000, // 1: set pins, 0
    0xE05F, // 2: set y, 31
    0x0083, // 3: jmp y--, 3
    0xE080, // 4: set pindirs, 0
    0xA02B, // 5: mov x, ~null
    0x00C8, // 6: jmp pin, 8
    0x0046, // 7: jmp x--, 6
    0xA0C9, // 8: mov isr, ~x
    0x8000, // 9: push noblock
};
static const struct pio_program capsense_program = {
    .instructions = capsense_program_instructions,
    .length       = 10,
    .origin       = -1,
};

static PIO  pio = pio0;
static uint progOffset = 0;

// The pad's charge path: internal pull-up, or nothing (external R does the job).
// With an external resistor the internal pull-up MUST be off or its ~50 kΩ shunts
// the high-value R and you lose all the hover sensitivity.
static inline void setPadPull(uint pin) {
#if EXTERNAL_PULLUP
    gpio_disable_pulls(pin);
#else
    gpio_pull_up(pin);                                  // internal ~50 kΩ = the charge resistor
#endif
}

// Point state machine `sm` at `pin` and (re)start it oscillating. One SM can be
// re-pointed at any pad between readings — that's what lets Mode 3 scan an
// arbitrary number of pads with a small fixed pool of SMs.
static void activatePio(uint8_t sm, uint pin) {
    pio_gpio_init(pio, pin);
    setPadPull(pin);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, progOffset + 0, progOffset + 9);
    sm_config_set_set_pins(&c, pin, 1);                 // SET pins/pindirs act on our pin
    sm_config_set_jmp_pin(&c, pin);                     // `jmp pin` tests our pin
    sm_config_set_clkdiv(&c, 1.0f);                     // full sysclk → finest time resolution

    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_init(pio, sm, progOffset, &c);               // disables, clears FIFO, loads cfg, pc=offset
    pio_sm_set_enabled(pio, sm, true);
}

// ── Active method + baselines ────────────────────────────────────────────────
static uint8_t mode = 1;                 // 1 = jtouch CPU · 2 = PIO free-run · 3 = PIO sequential
static float   baseline[NUM_PINS] = {0};
static float   ema[NUM_PINS]      = {0}; // smoothed reading per channel

// One Method-1 reading: sum of charge-loop iterations over M1_SAMPLES discharges.
static uint32_t measure1(uint8_t i) {
    const uint8_t pin = SENSE_PINS[i];
    uint32_t total = 0;
    for (uint8_t s = 0; s < M1_SAMPLES; s++) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        delayMicroseconds(M1_DISCHARGE_US);     // drain the pad

        noInterrupts();
#if EXTERNAL_PULLUP
        pinMode(pin, INPUT);                     // external R to 3V3 recharges it
#else
        pinMode(pin, INPUT_PULLUP);              // internal pull-up recharges it
#endif
        uint32_t c = 0;
        while (digitalRead(pin) == LOW && c < M1_TIMEOUT) c++;
        interrupts();
        total += c;
    }
    return total;
}

// Average M2_AVG fresh charge-cycle counts from one SM.
static float readAvg(uint8_t sm) {
    uint64_t sum = 0;
    int got = 0;
    for (uint8_t k = 0; k < M2_AVG; k++) {
        uint32_t guard = 0;
        while (pio_sm_is_rx_fifo_empty(pio, sm) && guard++ < 200000) {}
        if (pio_sm_is_rx_fifo_empty(pio, sm)) break;    // SM stalled — bail safely
        sum += pio_sm_get(pio, sm);
        got++;
    }
    return got ? (float)sum / got : 0.0f;
}

// Drive a pin LOW as a plain SIO output — a grounded GUARD for inactive pads.
static void guardLow(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

// Average k SMs CONCURRENTLY (they free-run in parallel, so interleaving the
// reads averages all of them within one M2_AVG window → ~×k faster than reading
// them one after another). sms[]/out[] are length k.
static void readAvgConcurrent(const uint8_t* sms, uint8_t k, float* out) {
    uint64_t sum[SCAN_PARALLEL] = {0};
    uint16_t got[SCAN_PARALLEL] = {0};
    for (uint8_t s = 0; s < M2_AVG; s++) {
        for (uint8_t m = 0; m < k; m++) {
            uint32_t guard = 0;
            while (pio_sm_is_rx_fifo_empty(pio, sms[m]) && guard++ < 200000) {}
            if (pio_sm_is_rx_fifo_empty(pio, sms[m])) continue;
            sum[m] += pio_sm_get(pio, sms[m]);
            got[m]++;
        }
    }
    for (uint8_t m = 0; m < k; m++) out[m] = got[m] ? (float)sum[m] / got[m] : 0.0f;
}

// Mode 3 — sequential/banked scan with grounded guards. Pads are split into banks
// whose members sit SCAN-stride apart (far → no mutual coupling); every pad NOT in
// the active bank is driven LOW as a guard. Only a pool of ≤SCAN_PARALLEL SMs is
// used, re-pointed bank to bank, so pad count is bounded by GPIOs, not SMs.
static void scanMode3(float* out) {
    const uint8_t banks = (NUM_PINS + SCAN_PARALLEL - 1) / SCAN_PARALLEL;
    for (uint8_t b = 0; b < banks; b++) {
        for (uint8_t sm = 0; sm < SCAN_PARALLEL; sm++) pio_sm_set_enabled(pio, sm, false);
        for (uint8_t j = 0; j < NUM_PINS; j++) guardLow(SENSE_PINS[j]);   // all low first

        uint8_t idx[SCAN_PARALLEL], sms[SCAN_PARALLEL], k = 0;
        for (uint8_t p = b; p < NUM_PINS; p += banks) {  // bank members, spaced `banks` apart
            activatePio(k, SENSE_PINS[p]);
            idx[k] = p; sms[k] = k; k++;
        }
        float avg[SCAN_PARALLEL];
        readAvgConcurrent(sms, k, avg);
        for (uint8_t m = 0; m < k; m++) out[idx[m]] = avg[m];
    }
}

// Fill out[] with one reading per channel, per the active mode.
static void scanAll(float* out) {
    if (mode == 3) { scanMode3(out); return; }
    for (uint8_t i = 0; i < NUM_PINS; i++)
        out[i] = (mode == 1) ? (float)measure1(i) : readAvg(i); // mode 2: SM i ↔ channel i
}

// PIO charge cycles → oscillator frequency (kHz). The loop body is 2 PIO cycles
// per counted iteration; ~38 cycles of fixed discharge/overhead bracket each
// period. freq = sysclk / period_cycles.
static float method2FreqKHz(float cycles) {
    float period = cycles * 2.0f + 38.0f;
    if (period < 1.0f) period = 1.0f;
    return (float)clock_get_hz(clk_sys) / period / 1000.0f;
}

// ── Method switching: reconfigure the pins for the chosen method ─────────────
// Mode 2 free-runs ONE SM PER PAD simultaneously, so it's capped at 4 pads (one
// PIO block's SMs). It's only the naive comparison baseline — Mode 3 is the one
// that scales to many pads. (Mode 1 and Mode 3 have no such cap.)
static void startMethod2() {
    uint8_t n = NUM_PINS < 4 ? NUM_PINS : 4;
    for (uint8_t i = 0; i < n; i++) activatePio(i, SENSE_PINS[i]);
}

static void startMethod3() {                             // park all as grounded guards;
    for (uint8_t sm = 0; sm < SCAN_PARALLEL; sm++) pio_sm_set_enabled(pio, sm, false);
    for (uint8_t i = 0; i < NUM_PINS; i++) guardLow(SENSE_PINS[i]); // scanMode3 activates per bank
}

static void startMethod1() {
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        pio_sm_set_enabled(pio, i, false);               // free the pin from PIO
#if EXTERNAL_PULLUP
        pinMode(SENSE_PINS[i], INPUT);                   // back to SIO (external R)
#else
        pinMode(SENSE_PINS[i], INPUT_PULLUP);            // back to SIO
#endif
    }
}

static void recalibrate() {
    float acc[NUM_PINS] = {0};
    for (uint8_t s = 0; s < 16; s++) {
        float tmp[NUM_PINS];
        scanAll(tmp);
        for (uint8_t i = 0; i < NUM_PINS; i++) acc[i] += tmp[i];
        delay(2);
    }
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        baseline[i] = acc[i] / 16.0f;
        ema[i]      = baseline[i];        // seed the smoother so it doesn't ramp in
    }
}

static void setMode(uint8_t m) {
    mode = m;
    if      (m == 1) startMethod1();
    else if (m == 2) startMethod2();
    else             startMethod3();
    delay(20);
    recalibrate();
    const char* name = (m == 1) ? "jtouch CPU charge-time"
                     : (m == 2) ? "PIO relaxation oscillator (free-run, all 3)"
                                : "PIO sequential scan + grounded guards";
    Serial.printf("# mode %u — %s\n", mode, name);
}

static void pollCommands() {
    while (Serial.available()) {
        int c = Serial.read();
        if      (c == '1') setMode(1);
        else if (c == '2') setMode(2);
        else if (c == '3') setMode(3);
        else if (c == 'r' || c == 'R') { recalibrate(); Serial.println(F("# recalibrated")); }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone (Pico) bare-pin capsense comparison ──"));
    Serial.printf("# pins: GP%u GP%u GP%u | sysclk %lu Hz\n",
                  SENSE_PINS[0], SENSE_PINS[1], SENSE_PINS[2],
                  (unsigned long)clock_get_hz(clk_sys));

    // Load the PIO program once; SMs are (re)configured on demand by activatePio().
    progOffset = pio_add_program(pio, &capsense_program);

    Serial.println(F("# keys: 1=jtouch  2=PIO free-run  3=PIO sequential+guards  r=recalibrate"));
    setMode(1);              // default: Method 1
    delay(1500);
}

void loop() {
    pollCommands();

    static uint32_t lastTele = 0;
    uint32_t now = millis();
    if (now - lastTele < TELE_PERIOD_MS) return;
    lastTele = now;

    // Measure all channels first, then derive the common-mode (the hand couples to
    // every closely-spaced pad at once → a shared signal on all three). Subtracting
    // the channel mean rejects that and leaves the PER-PAD component — the no-extra-
    // hardware way to undo the "hover bleeds across all pins" coupling.
    float raw[NUM_PINS], v[NUM_PINS], d[NUM_PINS];
    scanAll(raw);
    float sumd = 0;
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        ema[i] += SMOOTH * (raw[i] - ema[i]);      // smooth the small, noisy delta
        v[i] = ema[i];
        d[i] = v[i] - baseline[i];
        if (d[i] < 0) d[i] = 0;
        sumd += d[i];
    }
    float meanD = sumd / NUM_PINS;

    for (uint8_t i = 0; i < NUM_PINS; i++) {
        uint8_t gp = SENSE_PINS[i];
        // Teleplot over SERIAL needs the leading '>' or it treats the line as a
        // plain log (shows in the list, never graphs). Over UDP — like the ESP32
        // build — the prefix isn't required, which is why that one worked without.
        // Decimals matter: the hover delta is only a few counts, so print fractions.
        Serial.printf(">m%u_p%u:%.2f\n", mode, gp, v[i]);
        Serial.printf(">m%u_d%u:%.2f\n", mode, gp, d[i]);
        Serial.printf(">m%u_c%u:%.2f\n", mode, gp, d[i] - meanD); // common-mode removed
        if (mode != 1) Serial.printf(">m%u_f%u:%.2f\n", mode, gp, method2FreqKHz(v[i]));
    }
}
