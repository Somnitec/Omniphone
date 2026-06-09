#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Template for secrets.h — copy this file to `secrets.h` (which is git-ignored)
// and fill in your real values:
//
//     cp secrets.example.h secrets.h     # then edit secrets.h
//
// config.h includes secrets.h automatically if present; if it's absent (e.g. a
// fresh clone) the placeholder defaults in config.h are used and WiFi simply
// won't connect (sensing + USB Teleplot still work). Never commit secrets.h.
// ─────────────────────────────────────────────────────────────────────────────
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD"
#define TELEPLOT_HOST  ""              // IP of the machine running Teleplot, e.g. "192.168.1.20"

// Optional overrides (otherwise config.h defaults apply):
// #define OTA_HOSTNAME  "omniphone-esp32"
// #define OTA_PASSWORD  ""
// #define TELEPLOT_PORT 47269
