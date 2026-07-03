#include <Arduino.h>
#include <Wire.h>
#include <MPR121.h>
#include <math.h>
#include "../variants/teensy40-12pad-screen/config.h"            // THIS variant's real electrode map + MPR121 defaults
#include "../variants/teensy40-12pad-screen/proximity_engine.h"  // the REAL algorithm (envRef, median-friendly EMA, gate)

// ─────────────────────────────────────────────────────────────────────────────
// Sensor characterization rig — teensy40-12pad-screen
//
// Build:  pio run -e sensor_characterization -t upload      (serial @115200)
//
// Ground-truth capture for tuning the MPR121 registers and the proximity
// algorithm: idle noise + real sample interval, an automatic FFI/CDC sweep,
// raw time-series waveforms (works for both a slow approach and a fast tap —
// the label is just what you tell me you did while it was recording), a
// crosstalk capture across all pads while you press one, and real I2C
// acquisition timing. Uses THIS variant's actual config.h (SENSOR_CDC/CDT/FFI,
// electrode map) and the real proximity_engine.h, so anything captured here
// maps straight onto the firmware — copy a block of output and hand it back.
//
// This rig is MPR121-only: no screen, no audio, no other I2C/SPI activity
// competing for the bus or coupling noise in. If a noise scan here reads
// cleaner than the same pad's Teleplot stream in the full firmware, the
// difference is coming from the screen SPI / I2S audio / USB-serial print
// load, not from the sensor itself — a useful diagnostic split on its own.
//
// ── COMMANDS ──────────────────────────────────────────────────────────────
//   p<n>     select pad n (0..NUM_SENSORS-1, serial index — see config.h)
//   n<sec>   NOISE scan: hand fully AWAY. Reports min/max/mean/stddev of the
//            raw filtered value, plus the actual per-sample interval (this
//            loop reads as fast as the bus allows — see `u` for the real
//            production frame time, which is different).
//   f        AUTO FFI SWEEP: cycles FFI 0..3 at the current CDC/CDT. Hand
//            away ONCE at the prompt — fully automatic after that, one
//            noise-scan row per FFI value.
//   g        AUTO CDC SWEEP: cycles CDC 6/8/10/12/14/16/20 at the current
//            FFI/CDT. Hand away ONCE — one row per CDC value.
//   t<sec>   RAW TIME-SERIES: streams every sample (raw filtered, chip
//            baseline + delta for reference, software baseline, effDelta,
//            fast/slow EMA, intensity, touch) as a timestamped CSV for <sec>
//            seconds. Start it, then either hold a slow steady approach or do
//            a quick tap — same capture, the difference is what you did.
//   x<sec>   CROSSTALK: streams ALL pads' raw delta in one row per sample for
//            <sec> seconds while you press one or more pads OTHER than the
//            selected one — quantifies how much delta leaks into idle pads.
//   u        UPDATE-TIME: times 200 real burst-read frames (both boards, the
//            exact read pattern main.cpp uses) → min/mean/max microseconds.
//   I<khz>   set the I2C clock (100/400/1000). Default 100 — matches the
//            production wiring limit (see main.cpp's Wire.setClock note).
//            Compare against 400 to see what headroom is being left on the
//            table by the long wiring.
//   c        print current config       ?  help
// ─────────────────────────────────────────────────────────────────────────────

static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]), MPR121(BOARD_ADDRESSES[1]),
};

static ProximityConfig cfg; // engine defaults; not fed from config.h — this rig
                             // characterizes the SENSOR, the live firmware's
                             // proxCfg values are set in main.cpp's setup()
static SensorState st[NUM_SENSORS];

static uint8_t padIdx = 0;
static uint8_t curFFI = SENSOR_FFI, curCDC = SENSOR_CDC, curCDT = SENSOR_CDT;

static uint8_t boardOf(uint8_t p)     { return SENSORS[p].boardIndex; }
static uint8_t electrodeOf(uint8_t p) { return SENSORS[p].electrode; }

// Re-init every populated board with the given timing and commit a fresh
// baseline. Mirrors main.cpp's non-autoconfig path (beginConfig → CL=10 scan
// → lockBaseline) so results are comparable to the real firmware.
static void applyTiming(uint8_t ffi, uint8_t cdc, uint8_t cdt)
{
    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (BOARD_ADDRESSES[b] == 0x00) continue;
        boards[b].beginConfig(SENSE_ELECTRODES[b], 40, 20, cdc, cdt, ffi, SENSOR_ESI);
        boards[b].startScanning(SENSE_ELECTRODES[b], 0b10); // CL=10: track from first sample
        boards[b].lockBaseline(SENSE_ELECTRODES[b]);
    }
    curFFI = ffi; curCDC = cdc; curCDT = cdt;
}

static float readFilt(uint8_t p)
{
    return (float)boards[boardOf(p)].filteredData(electrodeOf(p));
}

static void reseedAll()
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (SENSORS[i].electrode == NO_PIN) continue;
        seedSensorState(st[i], readFilt(i)); // filtered value = software baseline
    }
}

// UNCLAMPED on purpose: a persistent negative idle delta is the baseline
// register's 8-bit quantization offset (up to −3 counts) — real, useful data.
static float readRawDelta(uint8_t p)
{
    uint16_t filt = boards[boardOf(p)].filteredData(electrodeOf(p));
    uint16_t base = boards[boardOf(p)].baselineData(electrodeOf(p));
    return (float)((int16_t)base - (int16_t)filt);
}

static void printConfig()
{
    Serial.print(F("# pad="));   Serial.print(padIdx);
    Serial.print(F(" board="));  Serial.print(boardOf(padIdx));
    Serial.print(F(" ELE"));     Serial.print(electrodeOf(padIdx));
    Serial.print(F("  FFI="));   Serial.print(curFFI);
    Serial.print(F(" CDC="));    Serial.print(curCDC);
    Serial.print(F(" CDT="));    Serial.println(curCDT);
}

static void printHelp()
{
    Serial.println(F("# p<n> n<sec> f g t<sec> x<sec> u I<khz> c ?  (see file header for full docs)"));
}

// ── Noise/jitter scan: hand away, `secDur` seconds, unthrottled ─────────────
// meanDtUs is the per-SINGLE-ELECTRODE read interval (two 1-byte/2-byte I2C
// transactions back to back) — NOT the production frame time; see `u` for that.
static void noiseScan(uint8_t p, float secDur, float& outMin, float& outMax,
                      float& outMean, float& outStd, float& meanDtUs, uint32_t& outN)
{
    uint32_t t0 = micros();
    uint32_t durUs = (uint32_t)(secDur * 1.0e6f);
    double sum = 0.0, sumSq = 0.0, sumDt = 0.0;
    uint32_t n = 0, lastT = t0;
    float mn = 1e9f, mx = -1e9f;
    while ((uint32_t)(micros() - t0) < durUs) {
        uint32_t t = micros();
        float v = readRawDelta(p);
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v; sumSq += (double)v * v;
        if (n > 0) sumDt += (double)(uint32_t)(t - lastT);
        lastT = t; n++;
    }
    outN     = n;
    outMin   = mn;
    outMax   = mx;
    outMean  = n ? (float)(sum / n) : 0.0f;
    double var = n ? (sumSq / n - (sum / n) * (sum / n)) : 0.0;
    outStd   = var > 0.0 ? sqrtf((float)var) : 0.0f;
    meanDtUs = n > 1 ? (float)(sumDt / (double)(n - 1)) : 0.0f;
}

static void promptHandAway(const char* what)
{
    Serial.print(F("# ")); Serial.print(what);
    Serial.println(F(" — move your hand fully AWAY from all pads now."));
    for (int i = 3; i > 0; i--) { Serial.print(i); Serial.print(' '); delay(500); }
    Serial.println(F("go"));
}

static void runFfiSweep()
{
    promptHandAway("AUTO FFI SWEEP (2 s per value)");
    Serial.println(F("# ffi  cdc  cdt  n  min  max  mean  std  dt_us"));
    for (uint8_t ffi = 0; ffi <= 3; ffi++) {
        applyTiming(ffi, curCDC, curCDT);
        float mn, mx, me, sd, dt; uint32_t n;
        noiseScan(padIdx, 2.0f, mn, mx, me, sd, dt, n);
        Serial.print(ffi);      Serial.print('\t');
        Serial.print(curCDC);   Serial.print('\t');
        Serial.print(curCDT);   Serial.print('\t');
        Serial.print(n);        Serial.print('\t');
        Serial.print(mn, 2);    Serial.print('\t');
        Serial.print(mx, 2);    Serial.print('\t');
        Serial.print(me, 3);    Serial.print('\t');
        Serial.print(sd, 3);    Serial.print('\t');
        Serial.println(dt, 1);
    }
    reseedAll();
    Serial.println(F("# FFI sweep done — chip left at the last swept value; re-run a sweep or power-cycle to restore"));
}

static void runCdcSweep()
{
    static const uint8_t CDCS[] = { 6, 8, 10, 12, 14, 16, 20 };
    promptHandAway("AUTO CDC SWEEP (2 s per value)");
    Serial.println(F("# cdc  ffi  cdt  n  min  max  mean  std  dt_us"));
    for (uint8_t i = 0; i < sizeof(CDCS); i++) {
        applyTiming(curFFI, CDCS[i], curCDT);
        float mn, mx, me, sd, dt; uint32_t n;
        noiseScan(padIdx, 2.0f, mn, mx, me, sd, dt, n);
        Serial.print(CDCS[i]);  Serial.print('\t');
        Serial.print(curFFI);   Serial.print('\t');
        Serial.print(curCDT);   Serial.print('\t');
        Serial.print(n);        Serial.print('\t');
        Serial.print(mn, 2);    Serial.print('\t');
        Serial.print(mx, 2);    Serial.print('\t');
        Serial.print(me, 3);    Serial.print('\t');
        Serial.print(sd, 3);    Serial.print('\t');
        Serial.println(dt, 1);
    }
    reseedAll();
    Serial.println(F("# CDC sweep done"));
}

// ── Raw time-series capture (slow-approach AND fast-tap tests share this) ───
static void runTimeSeries(float secDur)
{
    Serial.print(F("# TIME-SERIES pad ")); Serial.print(padIdx);
    Serial.print(F(" for ")); Serial.print(secDur, 1); Serial.println(F(" s — go"));
    // chipBase is logged for reference only (it shows the 4-count register
    // quantization); the engine runs purely on filt vs its software baseline.
    Serial.println(F("# t_ms  filt  chipBase  chipDelta  softBase  effDelta  fast  slow  intensity  touch"));

    seedSensorState(st[padIdx], readFilt(padIdx));

    uint32_t t0 = millis();
    uint32_t lastUpd = 0;
    while (millis() - t0 < (uint32_t)(secDur * 1000.0f)) {
        uint32_t now = millis();
        if (now - lastUpd < UPDATE_MS) continue;
        lastUpd = now;

        uint16_t filt = boards[boardOf(padIdx)].filteredData(electrodeOf(padIdx));
        uint16_t base = boards[boardOf(padIdx)].baselineData(electrodeOf(padIdx));
        float chipDelta = (float)((int16_t)base - (int16_t)filt);   // unclamped, reference

        float intensity; bool touch;
        updateProximity((float)filt, now, cfg, st[padIdx], intensity, touch);
        float effDelta = st[padIdx].base - (float)filt;
        if (effDelta < 0.0f) effDelta = 0.0f;

        Serial.print(now - t0);          Serial.print('\t');
        Serial.print(filt);              Serial.print('\t');
        Serial.print(base);              Serial.print('\t');
        Serial.print(chipDelta, 1);      Serial.print('\t');
        Serial.print(st[padIdx].base, 2); Serial.print('\t');
        Serial.print(effDelta, 2);       Serial.print('\t');
        Serial.print(st[padIdx].fast, 2);Serial.print('\t');
        Serial.print(st[padIdx].slow, 2);Serial.print('\t');
        Serial.print(intensity, 3);      Serial.print('\t');
        Serial.println(touch ? 1 : 0);
    }
    Serial.println(F("# TIME-SERIES done"));
}

// ── Crosstalk capture: all pads at once, production-style burst reads ───────
static void runCrosstalk(float secDur)
{
    Serial.print(F("# CROSSTALK for ")); Serial.print(secDur, 1);
    Serial.println(F(" s — press one or more OTHER pads now, go"));
    Serial.print(F("# t_ms"));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(F("\tpad")); Serial.print(i); }
    Serial.println();

    struct BoardData { uint8_t filt[24]; uint8_t base[12]; };
    static BoardData bd[NUM_BOARDS];

    uint32_t t0 = millis(), lastUpd = 0;
    while (millis() - t0 < (uint32_t)(secDur * 1000.0f)) {
        uint32_t now = millis();
        if (now - lastUpd < UPDATE_MS) continue;
        lastUpd = now;

        for (uint8_t b = 0; b < NUM_BOARDS; b++) {
            if (BOARD_ADDRESSES[b] == 0x00 || SENSE_ELECTRODES[b] == 0) continue;
            uint8_t n = SENSE_ELECTRODES[b];
            boards[b].burstRead(MPR121Reg::FILT_0L, bd[b].filt, (uint8_t)(n * 2));
            boards[b].burstRead(MPR121Reg::BASE_0,  bd[b].base, n);
        }

        Serial.print(now - t0);
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            if (SENSORS[i].electrode == NO_PIN) { Serial.print(F("\t0")); continue; }
            uint8_t b = boardOf(i), e = electrodeOf(i);
            uint16_t filt = (uint16_t)bd[b].filt[2 * e] | ((uint16_t)(bd[b].filt[2 * e + 1] & 0x03) << 8);
            uint16_t base = (uint16_t)bd[b].base[e] << 2;
            float d = (float)((int16_t)base - (int16_t)filt);
            Serial.print('\t'); Serial.print(d < 0.0f ? 0.0f : d, 1);
        }
        Serial.println();
    }
    Serial.println(F("# CROSSTALK done"));
}

// ── Update-time: real per-frame I2C acquisition cost, production pattern ────
static void runUpdateTime()
{
    struct BoardData { uint8_t filt[24]; uint8_t base[12]; };
    static BoardData bd[NUM_BOARDS];
    static constexpr uint16_t ITERS = 200;

    uint32_t mn = 0xFFFFFFFF, mx = 0; double sum = 0.0;
    for (uint16_t k = 0; k < ITERS; k++) {
        uint32_t t0 = micros();
        for (uint8_t b = 0; b < NUM_BOARDS; b++) {
            if (BOARD_ADDRESSES[b] == 0x00 || SENSE_ELECTRODES[b] == 0) continue;
            uint8_t n = SENSE_ELECTRODES[b];
            boards[b].burstRead(MPR121Reg::FILT_0L, bd[b].filt, (uint8_t)(n * 2));
            boards[b].burstRead(MPR121Reg::BASE_0,  bd[b].base, n);
        }
        uint32_t dt = micros() - t0;
        if (dt < mn) mn = dt;
        if (dt > mx) mx = dt;
        sum += dt;
    }
    Serial.print(F("# UPDATE-TIME (")); Serial.print(ITERS);
    Serial.print(F(" frames, both boards, production burst pattern): min="));
    Serial.print(mn); Serial.print(F("us mean="));
    Serial.print((float)(sum / ITERS), 1); Serial.print(F("us max="));
    Serial.print(mx); Serial.print(F("us   (UPDATE_MS budget = "));
    Serial.print(UPDATE_MS * 1000); Serial.println(F("us)"));
}

static void handleSerial()
{
    if (!Serial.available()) return;
    char c = Serial.read();
    switch (c) {
        case 'p': { int v = (int)Serial.parseFloat();
                    if (v >= 0 && v < NUM_SENSORS) { padIdx = (uint8_t)v; seedSensorState(st[padIdx], readFilt(padIdx)); }
                    printConfig(); break; }
        case 'n': { float sec = Serial.parseFloat(); if (sec <= 0.0f) sec = 2.0f;
                    Serial.println(F("# NOISE scan — hand AWAY"));
                    float mn, mx, me, sd, dt; uint32_t n;
                    noiseScan(padIdx, sec, mn, mx, me, sd, dt, n);
                    Serial.print(F("# n=")); Serial.print(n);
                    Serial.print(F(" min="));  Serial.print(mn, 2);
                    Serial.print(F(" max="));  Serial.print(mx, 2);
                    Serial.print(F(" mean=")); Serial.print(me, 3);
                    Serial.print(F(" std="));  Serial.print(sd, 3);
                    Serial.print(F(" dt_us=")); Serial.println(dt, 1);
                    break; }
        case 'f':  runFfiSweep(); break;
        case 'g':  runCdcSweep(); break;
        case 't': { float sec = Serial.parseFloat(); if (sec <= 0.0f) sec = 3.0f; runTimeSeries(sec); break; }
        case 'x': { float sec = Serial.parseFloat(); if (sec <= 0.0f) sec = 3.0f; runCrosstalk(sec); break; }
        case 'u':  runUpdateTime(); break;
        case 'I': { long khz = (long)Serial.parseFloat(); if (khz > 0) { Wire.setClock((uint32_t)khz * 1000);
                    Serial.print(F("# I2C clock = ")); Serial.print(khz); Serial.println(F(" kHz")); } break; }
        case 'c':  printConfig(); break;
        case '?':  printHelp(); printConfig(); break;
        default: break;
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    Wire.begin();
    Wire.setClock(100000); // matches production (long wires can't sustain 400 kHz — see main.cpp)

    for (uint8_t b = 0; b < NUM_BOARDS; b++) {
        if (BOARD_ADDRESSES[b] == 0x00) continue;
        bool ok = boards[b].beginConfig(SENSE_ELECTRODES[b], 40, 20, SENSOR_CDC, SENSOR_CDT, SENSOR_FFI, SENSOR_ESI);
        boards[b].startScanning(SENSE_ELECTRODES[b], 0b10);
        boards[b].lockBaseline(SENSE_ELECTRODES[b]);
        Serial.print(F("# board ")); Serial.print(b);
        Serial.println(ok ? F(" OK") : F(" NOT FOUND"));
    }
    reseedAll();

    Serial.println(F("# Sensor characterization rig — teensy40-12pad-screen"));
    printHelp();
    printConfig();
}

void loop()
{
    handleSerial();
}
