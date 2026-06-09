// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32-S3 (LilyGO T8-S3) · 19-pad · CAPACITIVE-SENSE BRING-UP TEST
//
// First light for the ESP32-S3 build: prove the capacitive sensing before the
// synth/LED/scale machinery is ported over. This sketch
//   • inits the two SENSE MPR121s (0x5A/0x5B → 19 electrodes) on one I²C bus,
//   • streams per-pad raw delta + filtered capacitance to Teleplot over USB,
//   • joins WiFi and enables ArduinoOTA (so later flashes can go over the air),
//   • serves a tiny HTTP/JSON endpoint (live values + live settings) — the seam
//     a web interface or companion app will drive.
//
// Everything degrades gracefully with no WiFi: sensing + Teleplot-over-USB still
// work, OTA and the web view just stay off. The two LED boards (0x5C/0x5D) are
// left alone here — they come online with the LED/synth port.
//
// Build:  pio run -e esp32s3-19pad -t upload          (USB, first flash)
// Plot:   open Teleplot, connect the serial port @115200 (close other monitors)
// View:   http://<device-ip>/   or   http://omniphone-esp32.local/
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <WebServer.h>
#include <MPR121.h>

#include "config.h"
#include "net_console.h"   // WiFi + OTA + serial-over-WiFi (Console)

// ── Hardware ──────────────────────────────────────────────────────────────────
static MPR121 sense[NUM_SENSE_BOARDS] = {
    MPR121(SENSE_ADDRESSES[0]),
    MPR121(SENSE_ADDRESSES[1]),
};
static bool senseOk[NUM_SENSE_BOARDS] = { false };

// Live, web-tweakable copy of the sensor tuning (seeded from config defaults).
static SensorSettings settings = DEFAULT_SETTINGS;

// Latest readings (shared by Teleplot + the web view). delta = baseline − filtered
// (positive = hand near); filtered is the chip's 10-bit value (lower = more cap).
static int16_t  padFiltered[NUM_SENSORS] = { 0 };
static int16_t  padDelta[NUM_SENSORS]    = { 0 };

static WebServer server(80);

// ─────────────────────────────────────────────────────────────────────────────
// Sensors
// ─────────────────────────────────────────────────────────────────────────────
// Real presence check: the MPR121 has no WHO_AM_I, so begin() can't tell us if a
// chip is there — but a bare address write either ACKs (0) or doesn't. Use this
// so we don't spam I2C reads at boards that aren't on the bus.
static bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;   // 0 = ACK received
}

// Scan the whole bus and list every address that ACKs. If this finds NOTHING the
// problem is the bus itself — wrong SDA/SCL pins, no power, or missing pull-ups —
// not the addresses. If it finds chips at unexpected addresses, fix the ADDR
// straps / SENSE_ADDRESSES[] to match.
static void scanI2C() {
    Serial.println(F("# I2C scan:"));
    uint8_t found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2cProbe(a)) { Serial.printf("#   found device @ 0x%02X\n", a); found++; }
    }
    if (!found)
        Serial.println(F("#   NONE — check SDA/SCL pins (config.h), 3V3 power, GND,"
                         " and that the bus has pull-ups."));
}

// (Re)initialise every sense board with the current settings. Called at boot and
// again whenever the web endpoint changes a tuning value. Probes for the chip
// first so a missing board is reported (and skipped) instead of flooding errors.
static void initSensors() {
    for (uint8_t b = 0; b < NUM_SENSE_BOARDS; b++) {
        senseOk[b] = i2cProbe(SENSE_ADDRESSES[b]);
        if (senseOk[b])
            sense[b].begin(SENSE_ELECTRODES[b], settings.touchTh, settings.releaseTh,
                           settings.cdc, settings.cdt);
        Serial.printf("#   MPR121 sense %u @ 0x%02X  %s  (%u electrodes)\n",
                      b, SENSE_ADDRESSES[b],
                      senseOk[b] ? "OK" : "*** NOT FOUND — check wiring/addr/power",
                      SENSE_ELECTRODES[b]);
    }
}

// Burst-read both boards and update padFiltered[] / padDelta[].
static void readSensors() {
    // filtered = 2 bytes/electrode (FILT_0L+2i), baseline = 1 byte/electrode (BASE_0+i)
    static uint8_t filtBuf[NUM_SENSE_BOARDS][24];
    static uint8_t baseBuf[NUM_SENSE_BOARDS][12];
    for (uint8_t b = 0; b < NUM_SENSE_BOARDS; b++) {
        if (!senseOk[b]) continue;          // skip absent boards (no error flood)
        const uint8_t n = SENSE_ELECTRODES[b];
        sense[b].burstRead(MPR121Reg::FILT_0L, filtBuf[b], (uint8_t)(2 * n));
        sense[b].burstRead(MPR121Reg::BASE_0,  baseBuf[b], n);
    }

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const SenseMap& m = SENSE_PADS[i];
        if (!senseOk[m.board]) { padFiltered[i] = 0; padDelta[i] = 0; continue; }
        uint8_t e = m.electrode;
        uint16_t filtered = (uint16_t)filtBuf[m.board][2 * e]
                          | ((uint16_t)(filtBuf[m.board][2 * e + 1] & 0x03) << 8);
        uint16_t baseline = (uint16_t)baseBuf[m.board][e] << 2;
        int16_t  delta    = (int16_t)baseline - (int16_t)filtered;
        padFiltered[i] = (int16_t)filtered;
        padDelta[i]    = delta < 0 ? 0 : delta;
    }
}

// Stream the latest readings to Teleplot (one ">name:value" per line). Two series
// per pad: f<i> = filtered capacitance, d<i> = raw delta (proximity signal).
static void streamTeleplot() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Console.printf(">f%u:%d\n", i, padFiltered[i]);
        Console.printf(">d%u:%d\n", i, padDelta[i]);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Web interface (prep for a companion app) — minimal JSON + a live HTML view.
//   GET /                → auto-refreshing dashboard
//   GET /api/values      → { "pads": [ {"f":..,"d":..}, ... ] }
//   GET /api/settings    → current tuning
//   GET /api/settings?cdc=&cdt=&touchTh=&releaseTh=  → update + re-init chips
// ─────────────────────────────────────────────────────────────────────────────
static void handleValues() {
    String json = "{\"pads\":[";
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (i) json += ',';
        json += "{\"f\":" + String(padFiltered[i]) + ",\"d\":" + String(padDelta[i]) + "}";
    }
    json += "]}";
    server.send(200, "application/json", json);
}

static String settingsJson() {
    return "{\"cdc\":" + String(settings.cdc) +
           ",\"cdt\":" + String(settings.cdt) +
           ",\"touchTh\":" + String(settings.touchTh) +
           ",\"releaseTh\":" + String(settings.releaseTh) + "}";
}

static void handleSettings() {
    bool changed = false;
    if (server.hasArg("cdc"))       { settings.cdc       = (uint8_t)server.arg("cdc").toInt() & 0x3F; changed = true; }
    if (server.hasArg("cdt"))       { settings.cdt       = (uint8_t)server.arg("cdt").toInt() & 0x07; changed = true; }
    if (server.hasArg("touchTh"))   { settings.touchTh   = (uint8_t)server.arg("touchTh").toInt();    changed = true; }
    if (server.hasArg("releaseTh")) { settings.releaseTh = (uint8_t)server.arg("releaseTh").toInt();  changed = true; }
    if (changed) {
        Serial.printf("# settings update via web: cdc=%u cdt=%u touchTh=%u releaseTh=%u\n",
                      settings.cdc, settings.cdt, settings.touchTh, settings.releaseTh);
        initSensors(); // re-apply to the chips (re-locks the baseline)
    }
    server.send(200, "application/json", settingsJson());
}

static void handleRoot() {
    // Tiny self-contained dashboard: polls /api/values and shows a bar per pad.
    static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Omniphone ESP32-S3 capsense</title>
<style>body{font:14px system-ui;margin:1rem;background:#111;color:#eee}
.pad{margin:2px 0}.bar{display:inline-block;height:14px;background:#4cf;vertical-align:middle}
b{display:inline-block;width:3rem}</style></head><body>
<h3>Omniphone · ESP32-S3 · capsense</h3><div id=v></div>
<script>
async function tick(){let r=await fetch('/api/values');let j=await r.json();
let h='';j.pads.forEach((p,i)=>{h+=`<div class=pad><b>p${i}</b>`+
`<span class=bar style="width:${Math.min(p.d,200)}px"></span> d=${p.d} f=${p.f}</div>`;});
document.getElementById('v').innerHTML=h;}
setInterval(tick,100);tick();
</script></body></html>)HTML";
    server.send_P(200, "text/html", PAGE);
}

static void startWebServer() {
    server.on("/", handleRoot);
    server.on("/api/values", handleValues);
    server.on("/api/settings", handleSettings);
    server.begin();
    Serial.println(F("# web view started on :80  (GET / · /api/values · /api/settings)"));
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone (ESP32-S3) capsense test ──"));

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK);
    Serial.printf("# I2C on SDA=%u SCL=%u @ %lu Hz\n",
                  PIN_I2C_SDA, PIN_I2C_SCL, (unsigned long)I2C_CLOCK);

    scanI2C();
    initSensors();
    Serial.printf("# pads=%u (sense boards: %u)\n", NUM_SENSORS, NUM_SENSE_BOARDS);

    Console.begin();                      // WiFi + OTA + remote serial (non-fatal)
    if (Console.wifiUp) startWebServer();

    Serial.println(F("# streaming Teleplot: f<pad>=filtered, d<pad>=raw delta"));
}

void loop() {
    Console.handle();                     // OTA + remote serial monitor
    if (Console.wifiUp) server.handleClient();

    uint32_t now = millis();

    static uint32_t lastSense = 0;
    if (now - lastSense >= SENSE_PERIOD_MS) {
        lastSense = now;
        readSensors();
    }

    static uint32_t lastTele = 0;
    if (now - lastTele >= TELE_PERIOD_MS) {
        lastTele = now;
        streamTeleplot();
    }
}
