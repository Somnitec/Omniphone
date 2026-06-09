// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32-S3 · NATIVE TOUCH capsense test (no MPR121) + live tuning
//
// Uses the ESP32-S3's built-in capacitive-touch peripheral. Streams, per channel,
// the RAW touch value AND a CALIBRATED 0..1 intensity (the value the instrument
// would actually play from), and lets you tune every relevant knob LIVE — over
// USB serial or over WiFi (telnet) — without reflashing.
//
// Tunable parameters (type `params` to print, `key=value` to set):
//   LEVEL 1 — Arduino touch cycles:
//     measure   charge/discharge cycles integrated per reading. ↑ = more range/
//               resolution & lower relative noise, but slower sampling.
//     sleep     idle cycles between readings (rate/power trade-off).
//   LEVEL 2 — IDF hardware knobs:
//     slope     charge-current slope 0–7 (7 = fastest/most sensitive).
//     denoise   1/0 — internal noise-reference channel subtracted from all pads.
//   CALIBRATION (raw → 0..1, the "as-played" value):
//     deadband  delta below this reads 0 (sits just above idle jitter).
//     max       delta that maps to full 1.0 (≈ firm touch).
//     gamma     response curve (1 = linear, >1 = softer near-touch, <1 = hotter).
//     smooth    fast-EMA factor 0..1 (lower = smoother/laggier).
//   Actions:  recal  (re-snapshot baselines)   params  (print current values)
//
//   On the S3 touchRead() RISES with touch (opposite of the classic ESP32); a
//   slow baseline tracker absorbs drift so the calibrated value self-zeroes.
//   NOTE: this inline calibration mirrors what proximity_engine.h does on the
//   other builds — that header is the eventual shared home for it.
//
// Build:  pio run -e esp32s3-native-touch -t upload
// Plot:   Teleplot @115200 (or socket://omniphone-esp32.local:23). Per pad:
//         t<gpio>=raw,  d<gpio>=delta above baseline,  c<gpio>=calibrated 0..1.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "driver/touch_sensor.h"   // LEVEL-2 knobs: cnt_mode, denoise, fsm

#include "config.h"
#include "net_console.h"           // WiFi + OTA + serial-over-WiFi (Console)

// Touch GPIOs to scan — any subset of GPIO1–GPIO14. (Currently TOUCH1–TOUCH3.)
static const uint8_t TOUCH_PINS[] = { 1, 2, 3 };
static constexpr uint8_t NUM_TOUCH = sizeof(TOUCH_PINS) / sizeof(TOUCH_PINS[0]);

static constexpr uint32_t TOUCH_TELE_PERIOD_MS = 30; // ~33 Hz stream

// ── Live-tunable parameters ──────────────────────────────────────────────────
struct TouchTune {
    // Level 1 (Arduino cycles) — default to the framework defaults (what felt good)
    uint16_t measure = TOUCH_PAD_MEASURE_CYCLE_DEFAULT;
    uint16_t sleep   = TOUCH_PAD_SLEEP_CYCLE_DEFAULT;
    // Level 2 (IDF hardware)
    uint8_t  slope   = 7;     // 0–7
    bool     denoise = true;
    // Calibration (raw delta → 0..1)
    float    deadband = 50.0f;
    float    max      = 2000.0f;
    float    gamma    = 1.0f;
    float    smooth   = 0.40f; // fast-EMA factor
};
static TouchTune tune;
static constexpr float BASE_ALPHA = 0.005f; // slow baseline drift tracking

// ── Per-channel processing state ─────────────────────────────────────────────
struct ChanState { float base; float fast; };
static ChanState ch[NUM_TOUCH];

// Push level-1 + level-2 settings to the hardware. Touch must already be inited
// (i.e. after at least one touchRead). Stops the FSM for the level-2 writes.
static void applyHardware() {
    touchSetCycles(tune.measure, tune.sleep);     // level 1 (live-safe)

    touch_pad_fsm_stop();
    for (uint8_t i = 0; i < NUM_TOUCH; i++)
        touch_pad_set_cnt_mode((touch_pad_t)TOUCH_PINS[i],
                               (touch_cnt_slope_t)tune.slope, TOUCH_PAD_TIE_OPT_DEFAULT);
    if (tune.denoise) {
        touch_pad_denoise_t dn;
        dn.grade     = TOUCH_PAD_DENOISE_BIT4;
        dn.cap_level = TOUCH_PAD_DENOISE_CAP_L4;
        touch_pad_denoise_set_config(&dn);
        touch_pad_denoise_enable();
    } else {
        touch_pad_denoise_disable();
    }
    touch_pad_fsm_start();
    delay(20);
}

// Snapshot the untouched baseline per channel (hands off). Reset the fast EMA too.
static void recalibrate() {
    for (uint8_t i = 0; i < NUM_TOUCH; i++) {
        uint32_t acc = 0;
        for (uint8_t s = 0; s < 16; s++) { acc += touchRead(TOUCH_PINS[i]); delay(5); }
        ch[i].base = (float)(acc / 16);
        ch[i].fast = ch[i].base;
    }
}

static void printParams() {
    Console.printf("# params: measure=%u sleep=%u slope=%u denoise=%u | "
                   "deadband=%.0f max=%.0f gamma=%.2f smooth=%.2f\n",
                   tune.measure, tune.sleep, tune.slope, tune.denoise ? 1 : 0,
                   tune.deadband, tune.max, tune.gamma, tune.smooth);
}

// Parse one command line: `params`, `recal`, or `key=value`.
static void handleCommand(char* s) {
    while (*s == ' ') s++;
    if (!*s) return;
    if (!strcmp(s, "params")) { printParams(); return; }
    if (!strcmp(s, "recal"))  { recalibrate(); Console.println(F("# recalibrated")); return; }

    char* eq = strchr(s, '=');
    if (!eq) { Console.printf("# ? '%s' — try: params, recal, host=<ip>, key=value\n", s); return; }
    *eq = '\0';
    const char* k = s;
    if (!strcmp(k, "host")) { Console.teleplotHost(eq + 1); return; } // IP, not a number
    float v = atof(eq + 1);

    bool hw = false;
    if      (!strcmp(k, "measure"))  { tune.measure = (uint16_t)v; hw = true; }
    else if (!strcmp(k, "sleep"))    { tune.sleep   = (uint16_t)v; hw = true; }
    else if (!strcmp(k, "slope"))    { tune.slope   = (uint8_t)constrain((int)v, 0, 7); hw = true; }
    else if (!strcmp(k, "denoise"))  { tune.denoise = (v != 0.0f); hw = true; }
    else if (!strcmp(k, "deadband")) { tune.deadband = v; }
    else if (!strcmp(k, "max"))      { tune.max = v; }
    else if (!strcmp(k, "gamma"))    { tune.gamma = v; }
    else if (!strcmp(k, "smooth"))   { tune.smooth = constrain(v, 0.01f, 1.0f); }
    else { Console.printf("# unknown key '%s'\n", k); return; }

    if (hw) { applyHardware(); recalibrate(); } // raw scale changed → re-baseline
    printParams();
}

// Accumulate command lines from BOTH USB serial and the WiFi (telnet) console.
static void pollCommands() {
    static char buf[64];
    static uint8_t len = 0;
    int c;
    while ((c = Serial.available() ? Serial.read()
                                   : Console.available() ? Console.read() : -1) >= 0) {
        if (c == '\n' || c == '\r') {
            if (len) { buf[len] = '\0'; handleCommand(buf); len = 0; }
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = (char)c;
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone (ESP32-S3) NATIVE TOUCH + live tuning ──"));

    // Force the touch peripheral to initialise (needed before the level-2 IDF
    // calls), then push our settings and snapshot baselines.
    for (uint8_t i = 0; i < NUM_TOUCH; i++) touchRead(TOUCH_PINS[i]);
    delay(50);
    applyHardware();

    Serial.println(F("# calibrating baselines — keep hands off…"));
    recalibrate();
    printParams();

    Console.begin();   // WiFi + OTA + remote serial (non-fatal if headless)
    if (strlen(TELEPLOT_HOST) > 0) Console.teleplotHost(TELEPLOT_HOST);
    Serial.printf("# streaming %u channels: t=raw, d=delta, c=calibrated(0..1). "
                  "Type `params`/`key=value`/`host=<ip>` (USB or telnet) to tune.\n", NUM_TOUCH);
    delay(2000);       // pause so the boot/WiFi log is readable before the stream
}

void loop() {
    Console.handle();  // OTA + remote serial
    pollCommands();    // live tuning over USB or WiFi

    static uint32_t lastTele = 0;
    uint32_t now = millis();
    if (now - lastTele >= TOUCH_TELE_PERIOD_MS) {
        lastTele = now;
        Console.teleBegin();   // batches one UDP packet; also streams serial + telnet
        for (uint8_t i = 0; i < NUM_TOUCH; i++) {
            uint32_t raw = touchRead(TOUCH_PINS[i]);

            // Fast EMA for a clean signal; slow baseline tracks drift but freezes
            // under touch (only updates when the delta is below the deadband).
            ch[i].fast += tune.smooth * ((float)raw - ch[i].fast);
            float delta = ch[i].fast - ch[i].base;
            if (delta < 0.0f) delta = 0.0f;
            if (delta < tune.deadband)
                ch[i].base += BASE_ALPHA * ((float)raw - ch[i].base);

            // Calibrated 0..1: deadband..max → 0..1, then the response curve.
            float norm = (delta - tune.deadband) / (tune.max - tune.deadband);
            if (norm < 0.0f) norm = 0.0f; else if (norm > 1.0f) norm = 1.0f;
            float cal = (tune.gamma == 1.0f) ? norm : powf(norm, tune.gamma);

            char nm[8];
            snprintf(nm, sizeof(nm), "t%u", TOUCH_PINS[i]); Console.tele(nm, (double)raw, 0);
            snprintf(nm, sizeof(nm), "d%u", TOUCH_PINS[i]); Console.tele(nm, delta, 0);
            snprintf(nm, sizeof(nm), "c%u", TOUCH_PINS[i]); Console.tele(nm, cal, 3);
        }
        Console.teleEnd();
    }
}
