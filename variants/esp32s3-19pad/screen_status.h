#pragma once
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <WiFi.h>

#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// On-screen WiFi status bar for the T-Display-S3 test sketches.
//
// Draws a bar at the top of the screen: hostname · RSSI · IP when connected
// (green), or "disconnected" (red). Self-throttles to ~1 Hz, so it's safe to call
// every loop iteration. Needs the screen already initialised (tft.init()).
// ─────────────────────────────────────────────────────────────────────────────
inline void drawNetStatus(TFT_eSPI& tft, bool force = false) {
    static uint32_t last = 0;
    if (!force && millis() - last < 1000) return;
    last = millis();

    bool up = (WiFi.status() == WL_CONNECTED);
    uint16_t bg = up ? TFT_DARKGREEN : TFT_RED;
    tft.fillRect(0, 0, tft.width(), 20, bg);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, bg);
    tft.setTextFont(2);

    char line[80];
    if (up)
        snprintf(line, sizeof(line), "%s  %d dBm  %s",
                 OTA_HOSTNAME, (int)WiFi.RSSI(), WiFi.localIP().toString().c_str());
    else
        snprintf(line, sizeof(line), "WiFi disconnected  (%s)", OTA_HOSTNAME);
    tft.drawString(line, 4, 2);
}
