// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — LilyGO T-Display-S3 (Touch) · WHITE-SCREEN + TOUCH + OTA test
//
// Display check for the T-Display-S3 (ESP32-S3, ST7789 320×170, 8-bit parallel):
// power the panel, backlight on, fill the screen white. Plus a CST816 touch
// readout (the Touch variant's controller) printed to the console: finger
// down/up, live X/Y coordinates, and gestures (taps, swipes, long-press). WiFi +
// OTA come up so the next iteration can flash over the air.
//
// Pins handled in code (not in the TFT_eSPI build flags):
//   • GPIO15  LCD/peripheral power rail — HIGH before tft.init()
//   • GPIO38  backlight
//   • CST816 touch on I²C: SDA=18, SCL=17, RST=21, INT=16 (addr 0x15)
//
// Build:  pio run -e esp32s3-tdisplay-white -t upload   (OTA: …-tdisplay-white-ota)
// Touch:  watch the console (USB @115200, or socket://omniphone-19pad.local:23);
//         X/Y also stream to Teleplot as tx/ty.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>

#include "net_console.h"   // WiFi + OTA + serial-over-WiFi (Console) + secrets

// ── T-Display-S3 board pins ───────────────────────────────────────────────────
static constexpr uint8_t PIN_LCD_POWER = 15;
static constexpr uint8_t PIN_LCD_BL    = 38;
static constexpr uint8_t PIN_TOUCH_SDA = 18;
static constexpr uint8_t PIN_TOUCH_SCL = 17;
static constexpr uint8_t PIN_TOUCH_RST = 21;
static constexpr uint8_t PIN_TOUCH_INT = 16;
static constexpr uint8_t CST816_ADDR   = 0x15;

static TFT_eSPI tft;

// ── CST816 register access ────────────────────────────────────────────────────
static void touchWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(CST816_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Returns false (silently) if the chip doesn't ACK — i.e. it's asleep with no
// finger present. Probing with endTransmission() first avoids the Wire requestFrom
// error spam that polling a sleeping CST816 produces.
static bool touchReadBuf(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(CST816_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return false;   // no ACK → asleep/absent
    Wire.requestFrom(CST816_ADDR, n);
    if (Wire.available() < n) { while (Wire.available()) Wire.read(); return false; }
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

static const char* gestureName(uint8_t g) {
    switch (g) {
        case 0x01: return "swipe-up";
        case 0x02: return "swipe-down";
        case 0x03: return "swipe-left";
        case 0x04: return "swipe-right";
        case 0x05: return "tap";
        case 0x0B: return "double-tap";
        case 0x0C: return "long-press";
        default:   return "none";
    }
}

// Poll the controller and report ONLY on change: finger down/up, a real move
// (beyond a small jitter deadband), or a new gesture. Nothing is printed while a
// finger is held still.
static constexpr int TOUCH_MOVE_PX = 3; // ignore jitter smaller than this

static void pollTouch() {
    uint8_t d[6] = { 0 };
    bool     ok = touchReadBuf(0x01, d, 6); // [gesture, fingers, xh, xl, yh, yl]
    uint8_t  gesture = ok ? d[0] : 0;       // no ACK = chip asleep = no finger
    bool     touched = ok && (d[1] & 0x0F) > 0;
    uint16_t x = ((uint16_t)(d[2] & 0x0F) << 8) | d[3];
    uint16_t y = ((uint16_t)(d[4] & 0x0F) << 8) | d[5];

    static bool     wasTouched  = false;
    static uint16_t lastX = 0, lastY = 0;
    static uint8_t  lastGesture = 0;

    bool moved = touched && (abs((int)x - (int)lastX) > TOUCH_MOVE_PX ||
                             abs((int)y - (int)lastY) > TOUCH_MOVE_PX);
    bool changed = false;

    if (touched && !wasTouched) {
        Console.printf("# touch DOWN  x=%u y=%u\n", x, y);
        lastX = x; lastY = y; changed = true;
    } else if (!touched && wasTouched) {
        Console.println(F("# touch UP"));
        lastGesture = 0;               // re-arm gesture reporting
        changed = true;
    } else if (moved) {
        Console.printf("# touch MOVE  x=%u y=%u\n", x, y);
        lastX = x; lastY = y; changed = true;
    }

    if (gesture && gesture != lastGesture) {
        lastGesture = gesture;
        Console.printf("# gesture: %s\n", gestureName(gesture));
        changed = true;
    }

    // Push to Teleplot only on the same change events (0 once released).
    if (changed) {
        Console.teleBegin();
        Console.tele("tx", touched ? x : 0, 0);
        Console.tele("ty", touched ? y : 0, 0);
        Console.tele("touched", touched ? 1 : 0, 0);
        Console.teleEnd();
    }

    wasTouched = touched;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println(F("\n# ── Omniphone T-Display-S3 white-screen + touch + OTA ──"));

    // Power the LCD rail BEFORE init, then bring up the panel and fill it white.
    pinMode(PIN_LCD_POWER, OUTPUT);
    digitalWrite(PIN_LCD_POWER, HIGH);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_WHITE);

    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);

    // CST816 touch controller: reset, then start the I²C bus it shares.
    pinMode(PIN_TOUCH_RST, OUTPUT);
    digitalWrite(PIN_TOUCH_RST, LOW);  delay(10);
    digitalWrite(PIN_TOUCH_RST, HIGH); delay(50);
    pinMode(PIN_TOUCH_INT, INPUT);
    Wire.begin(PIN_TOUCH_SDA, PIN_TOUCH_SCL);
    Wire.setClock(400000);
    touchWrite(0xFE, 0x01);   // CST816 DisAutoSleep = 1 → stay awake, keep ACKing

    Console.begin();   // WiFi + OTA + remote serial (non-fatal headless)
    Serial.println(F("# screen white; touch readout live (down/up · x/y · gestures); OTA ready"));
}

void loop() {
    Console.handle();  // service OTA + remote serial

    static uint32_t lastPoll = 0;
    if (millis() - lastPoll >= 15) {   // ~66 Hz touch poll
        lastPoll = millis();
        pollTouch();
    }
}
