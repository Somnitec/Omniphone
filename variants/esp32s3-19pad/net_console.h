#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// NetConsole — WiFi + OTA + serial-over-WiFi for the ESP32-S3 bring-up sketches.
//
// It is a Print, so anything you'd send to Serial you send to `Console` instead
// and it fans the bytes out to BOTH the USB serial port AND a connected TCP
// client on port 23 — i.e. a remote serial monitor that works over the same
// network as OTA. Connect it with PlatformIO's socket:// transport:
//
//   pio device monitor --port socket://omniphone-esp32.local:23
//   (or just:  nc omniphone-esp32.local 23  /  telnet omniphone-esp32.local)
//
// Teleplot works over it too (point Teleplot at the socket). One client at a time.
//
// Everything is non-fatal when headless: if WiFi never joins, output still goes
// to USB Serial and wifiUp stays false (callers gate the web server / OTA on it).
// ─────────────────────────────────────────────────────────────────────────────
// OTA hooks — the app sets these (via onOta) to e.g. suspend the audio task during
// a flash write (flash ops stall code execution and contend with the upload).
inline void (*g_otaBefore)() = nullptr;
inline void (*g_otaAfter)()  = nullptr;

class NetConsole : public Print {
public:
    bool wifiUp = false;

    // Register callbacks run at OTA start / end (or error).
    void onOta(void (*before)(), void (*after)()) { g_otaBefore = before; g_otaAfter = after; }

    // Kick off WiFi WITHOUT blocking — the connection finishes in the background
    // (handle()), so the instrument boots/plays instantly instead of waiting up to
    // ~15 s for the link. OTA + telnet come up once connected.
    void begin() {
        WiFi.persistent(false);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);            // no modem sleep (OTA reliability)
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        WiFi.setAutoReconnect(true);
        WiFi.setHostname(OTA_HOSTNAME);
        _started = true;
        connectAP(0);
        Serial.println(F("# WiFi: connecting in background…"));
    }

    // Pump OTA + telnet; drive the background connect. Call every loop.
    void handle() {
        if (!_started) return;

        if (!wifiUp) {
            if (WiFi.status() == WL_CONNECTED) {
                finalizeConnect();
            } else if (millis() - _apTryMs > 6000) {
                connectAP(_apIdx + 1);       // give the other known AP a turn
            }
            return;                          // OTA/telnet not up until connected
        }

        // Connected: watchdog (non-blocking) — if the link drops, reconnect.
        static uint32_t lastCheck = 0;
        if (millis() - lastCheck > 5000) {
            lastCheck = millis();
            if (WiFi.status() != WL_CONNECTED) { wifiUp = false; connectAP(_apIdx); return; }
        }

        ArduinoOTA.handle();
        WiFiClient incoming = _telnet.accept();
        if (incoming) {
            if (_client && _client.connected()) {
                incoming.stop();
            } else {
                _client = incoming;
                if (_histWrap) _client.write((const uint8_t*)_hist + _histLen, sizeof(_hist) - _histLen);
                _client.write((const uint8_t*)_hist, _histLen);
                _client.println(F("# --- remote serial connected (history above) ---"));
            }
        }
    }

    // Turn WiFi/OTA off at runtime (performance mode). begin() re-enables it; both
    // are non-blocking, so no reboot is needed to toggle.
    void disable() {
        if (_otaInit) { ArduinoOTA.end(); _telnet.end(); }
        if (_client) _client.stop();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        wifiUp = false;
        _started = false;
        _otaInit = false;
    }

    // Input from the telnet client, so commands can be typed over WiFi too.
    int available() { return (_client && _client.connected()) ? _client.available() : 0; }
    int read()      { return (_client && _client.connected()) ? _client.read() : -1; }

    // ── Teleplot over WiFi (UDP) ──────────────────────────────────────────────
    // Teleplot listens on UDP :47269. Point it at the machine running Teleplot;
    // until set, UDP is off and teleplot still goes out USB serial + telnet.
    // Accepts "ip" or "ip:port" (port defaults to TELEPLOT_PORT).
    bool teleplotHost(const char* spec) {
        char tmp[40];
        strncpy(tmp, spec, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = '\0';
        uint16_t port = TELEPLOT_PORT;
        char* colon = strchr(tmp, ':');
        if (colon) { *colon = '\0'; port = (uint16_t)atoi(colon + 1); }
        if (!_tpHost.fromString(tmp)) { Serial.printf("# bad teleplot host '%s'\n", spec); return false; }
        _tpPort = port ? port : TELEPLOT_PORT;
        _tpOn = true;
        Serial.printf("# teleplot UDP → %s:%u\n", tmp, _tpPort);
        return true;
    }

    // One Teleplot frame: teleBegin(), one tele() per series, teleEnd().
    // When a UDP host is configured, values go ONLY over UDP — echoing every
    // value to USB serial + telnet too means hundreds of tiny blocking writes
    // per second (the telnet ones stall the whole loop on a weak link). Frames
    // are batched TP_BATCH-deep into one UDP packet: every radio TX burst is a
    // current spike that couples into the audio path, so fewer, bigger packets
    // = less crackle for the same data. Without a UDP host, the ">"-prefixed
    // serial/telnet echo is the fallback so Teleplot-over-serial still works.
    static constexpr uint8_t TP_BATCH = 4;
    void teleBegin() {}
    void tele(const char* name, double value, uint8_t decimals = 3) {
        char line[40];
        int n = snprintf(line, sizeof(line), "%s:%.*f\n", name, decimals, value);
        if (n <= 0) return;
        if (_tpOn) {
            if ((size_t)(_tpLen + n) < sizeof(_tpBuf)) { memcpy(_tpBuf + _tpLen, line, n); _tpLen += n; }
            return;
        }
        Serial.write('>'); Serial.write((const uint8_t*)line, n);
        if (_client && _client.connected()) { _client.write('>'); _client.write((const uint8_t*)line, n); }
    }
    void teleEnd() {
        if (!_tpOn) return;
        if (!wifiUp) { _tpLen = 0; _tpFrames = 0; return; }
        if (++_tpFrames < TP_BATCH && _tpLen < sizeof(_tpBuf) - 256) return;
        if (_tpLen) {
            _udp.beginPacket(_tpHost, _tpPort);
            _udp.write((const uint8_t*)_tpBuf, _tpLen);
            _udp.endPacket();
        }
        _tpLen = 0;
        _tpFrames = 0;
    }

    size_t write(uint8_t c) override {
        Serial.write(c);
        if (_client && _client.connected()) _client.write(c);
        capture(c);
        return 1;
    }
    size_t write(const uint8_t* buf, size_t n) override {
        Serial.write(buf, n);
        if (_client && _client.connected()) _client.write(buf, n);
        for (size_t i = 0; i < n; i++) capture(buf[i]);
        return n;
    }

    // Last complete line written (no newline) — for showing on a screen.
    const char* lastLine() const { return _lastLine; }

private:
    // Start a (non-blocking) connection to known AP `idx` (alternates if a 2nd is
    // defined). WiFi.begin() returns immediately; status is polled in handle().
    void connectAP(uint8_t idx) {
        const char* ssid = WIFI_SSID; const char* pass = WIFI_PASSWORD;
#if defined(WIFI_SSID2)
        _apIdx = idx & 1;
        if (_apIdx) { ssid = WIFI_SSID2; pass = WIFI_PASSWORD2; }
#else
        _apIdx = 0;
#endif
        WiFi.begin(ssid, pass);
        _apTryMs = millis();
    }

    void finalizeConnect() {
        wifiUp = true;
        Serial.print(F("# WiFi: connected, IP "));
        Serial.print(WiFi.localIP());
        Serial.printf("  RSSI %d dBm  heap %u\n", WiFi.RSSI(), (unsigned)ESP.getFreeHeap());
        if (!_otaInit) {            // one-time OTA + telnet setup
            ArduinoOTA.setHostname(OTA_HOSTNAME);
            if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
            ArduinoOTA
                .onStart([]()  { Serial.println(F("# OTA: update starting")); if (g_otaBefore) g_otaBefore(); })
                .onEnd([]()    { Serial.println(F("\n# OTA: done")); if (g_otaAfter) g_otaAfter(); })
                .onProgress([](unsigned int p, unsigned int t) {
                    Serial.printf("\r# OTA: %u%%", t ? (p * 100u / t) : 0u);
                })
                .onError([](ota_error_t e) { Serial.printf("\n# OTA error %u\n", e); if (g_otaAfter) g_otaAfter(); });
            ArduinoOTA.begin();
            _telnet.begin();
            _telnet.setNoDelay(true);
            _otaInit = true;
        }
        Serial.printf("# OTA + remote serial ready ('%s.local', socket://%s.local:23)\n",
                      OTA_HOSTNAME, OTA_HOSTNAME);
    }

    bool       _started = false;
    bool       _otaInit = false;
    uint8_t    _apIdx   = 0;
    uint32_t   _apTryMs = 0;
    WiFiServer _telnet{23};
    WiFiClient _client;

    WiFiUDP    _udp;
    IPAddress  _tpHost;
    uint16_t   _tpPort = TELEPLOT_PORT;
    bool       _tpOn = false;
    char       _tpBuf[1024];     // holds TP_BATCH frames of ~19 series each
    uint16_t   _tpLen = 0;
    uint8_t    _tpFrames = 0;

    // Output history (replayed to a new telnet client) + last line (for the screen).
    char     _hist[1024];
    uint16_t _histLen  = 0;
    bool     _histWrap = false;
    char     _line[96];
    uint8_t  _lineLen  = 0;
    char     _lastLine[96] = "";

    void capture(uint8_t c) {
        _hist[_histLen++] = (char)c;                 // ring buffer
        if (_histLen >= sizeof(_hist)) { _histLen = 0; _histWrap = true; }
        if (c == '\n' || c == '\r') {                // finalise a line
            if (_lineLen) { _line[_lineLen] = '\0'; memcpy(_lastLine, _line, _lineLen + 1); _lineLen = 0; }
        } else if (_lineLen < sizeof(_line) - 1) {
            _line[_lineLen++] = (char)c;
        }
    }
};

inline NetConsole Console;
