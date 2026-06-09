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
class NetConsole : public Print {
public:
    bool wifiUp = false;

    void begin() {
        WiFi.mode(WIFI_STA);
        WiFi.setHostname(OTA_HOSTNAME);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print(F("# WiFi: connecting"));
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
            Serial.print('.');
            delay(200);
        }
        Serial.println();

        if (WiFi.status() != WL_CONNECTED) {
            Serial.println(F("# WiFi: NOT connected — USB serial only. Set WIFI_SSID/"
                             "WIFI_PASSWORD in config.h for OTA + remote monitor."));
            return;
        }
        wifiUp = true;
        Serial.print(F("# WiFi: connected, IP "));
        Serial.println(WiFi.localIP());

        ArduinoOTA.setHostname(OTA_HOSTNAME);
        if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA
            .onStart([]()  { Serial.println(F("# OTA: update starting")); })
            .onEnd([]()    { Serial.println(F("\n# OTA: done")); })
            .onProgress([](unsigned int p, unsigned int t) {
                Serial.printf("\r# OTA: %u%%", t ? (p * 100u / t) : 0u);
            })
            .onError([](ota_error_t e) { Serial.printf("\n# OTA error %u\n", e); });
        ArduinoOTA.begin();

        _telnet.begin();
        _telnet.setNoDelay(true);
        Serial.printf("# OTA + remote serial ready as '%s.local'  "
                      "(monitor: socket://%s.local:23)\n", OTA_HOSTNAME, OTA_HOSTNAME);
    }

    // Pump OTA and accept/maintain a single telnet client. Call every loop.
    void handle() {
        if (!wifiUp) return;
        ArduinoOTA.handle();
        WiFiClient incoming = _telnet.accept();
        if (incoming) {
            if (_client && _client.connected()) {
                incoming.stop();                 // one remote monitor at a time
            } else {
                _client = incoming;
                _client.println(F("# Omniphone ESP32-S3 — remote serial connected"));
            }
        }
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

    // One Teleplot frame: teleBegin(), one tele() per series, teleEnd(). Each
    // value goes to USB serial + telnet (with the ">" serial prefix) immediately,
    // and is batched into a single UDP packet flushed by teleEnd().
    void teleBegin() { _tpLen = 0; }
    void tele(const char* name, double value, uint8_t decimals = 3) {
        char line[40];
        int n = snprintf(line, sizeof(line), "%s:%.*f\n", name, decimals, value);
        if (n <= 0) return;
        Serial.write('>'); Serial.write((const uint8_t*)line, n);
        if (_client && _client.connected()) { _client.write('>'); _client.write((const uint8_t*)line, n); }
        if (_tpOn && (size_t)(_tpLen + n) < sizeof(_tpBuf)) { memcpy(_tpBuf + _tpLen, line, n); _tpLen += n; }
    }
    void teleEnd() {
        if (_tpOn && wifiUp && _tpLen) {
            _udp.beginPacket(_tpHost, _tpPort);
            _udp.write((const uint8_t*)_tpBuf, _tpLen);
            _udp.endPacket();
        }
    }

    size_t write(uint8_t c) override {
        Serial.write(c);
        if (_client && _client.connected()) _client.write(c);
        return 1;
    }
    size_t write(const uint8_t* buf, size_t n) override {
        Serial.write(buf, n);
        if (_client && _client.connected()) _client.write(buf, n);
        return n;
    }

private:
    WiFiServer _telnet{23};
    WiFiClient _client;

    WiFiUDP    _udp;
    IPAddress  _tpHost;
    uint16_t   _tpPort = TELEPLOT_PORT;
    bool       _tpOn = false;
    char       _tpBuf[512];
    uint16_t   _tpLen = 0;
};

inline NetConsole Console;
