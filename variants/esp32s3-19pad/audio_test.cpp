// ─────────────────────────────────────────────────────────────────────────────
// Omniphone — ESP32-S3 · AUDIO test (I²S / PCM5102A) with on-screen feedback
//
// The audio path only: plays a looping A-major arpeggio so you can immediately
// hear whether I²S → DAC works (and the pitch confirms the clocking). The screen
// shows the I²S init result, the current tone, the pins, and the WiFi bar — so you
// get the diagnosis visually even if you miss the serial log.
//
// I²S → PCM5102A: BCK=1, LRCK=2, DIN=10 (SCK→GND).
//   • HEAR the arpeggio → audio path good; instrument's "no sound" is elsewhere.
//   • SILENT but screen says "I2S: OK" → DAC wiring (LRCK pin, power, FMT→GND).
//
// Build:  pio run -e esp32s3-test -t upload    (OTA: esp32s3-test-ota)
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <math.h>
#include <TFT_eSPI.h>
#include "driver/i2s.h"

#include "config.h"
#include "net_console.h"    // WiFi + OTA + serial-over-WiFi (Console)
#include "screen_status.h"  // WiFi status bar (drawNetStatus)

static constexpr int PIN_I2S_BCK  = 1;
static constexpr int PIN_I2S_LRCK = 2;
static constexpr int PIN_I2S_DIN  = 10;

static constexpr uint8_t PIN_LCD_POWER = 15;
static constexpr uint8_t PIN_LCD_BL    = 38;
static TFT_eSPI tft;

static constexpr int      SAMPLE_RATE = 44100;
static constexpr int      BLOCK       = 256;
static constexpr uint16_t TABLE_SIZE  = 1024;
static int16_t  sineTable[TABLE_SIZE];
static volatile uint32_t phaseInc = 0;
static bool i2sOk = false;
static volatile uint32_t g_blocks = 0;   // blocks actually written to I²S
static volatile uint32_t g_bytes  = 0;   // bytes the last write accepted

static void audioTask(void*) {
    static int32_t tx[BLOCK * 2];   // 32-bit stereo (PCM5102A likes 64fs frames)
    uint32_t phase = 0;
    for (;;) {
        uint32_t inc = phaseInc;
        for (int f = 0; f < BLOCK; f++) {
            int32_t s = (int32_t)sineTable[phase >> 22] << 16;   // 16-bit → top of 32
            tx[2 * f] = tx[2 * f + 1] = s;
            phase += inc;
        }
        size_t wrote = 0;
        i2s_write(I2S_NUM_0, tx, sizeof(tx), &wrote, portMAX_DELAY);
        g_bytes = wrote;
        g_blocks++;
    }
}

static void audioBegin() {
    // Near full-scale so even a marginal DAC connection is clearly audible.
    for (uint16_t i = 0; i < TABLE_SIZE; i++)
        sineTable[i] = (int16_t)(sinf(2.0f * (float)M_PI * i / TABLE_SIZE) * 28000.0f);

    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;   // 64fs frame
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = 0;
    cfg.dma_buf_count        = 6;
    cfg.dma_buf_len          = BLOCK;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    i2sOk = (err == ESP_OK);
    Console.printf("# I2S driver install: %s (BCK=%d LRCK=%d DIN=%d)\n",
                   i2sOk ? "OK" : "FAIL", PIN_I2S_BCK, PIN_I2S_LRCK, PIN_I2S_DIN);

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = I2S_PIN_NO_CHANGE;
    pins.bck_io_num   = PIN_I2S_BCK;
    pins.ws_io_num    = PIN_I2S_LRCK;
    pins.data_out_num = PIN_I2S_DIN;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    esp_err_t pe = i2s_set_pin(I2S_NUM_0, &pins);
    Console.printf("# i2s_set_pin: %s\n", pe == ESP_OK ? "OK" : "FAIL");

    BaseType_t t = xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, 3, nullptr, 1);
    Console.printf("# audio task create: %s\n", t == pdPASS ? "OK" : "FAIL");
}

static void drawStaticScreen() {
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setTextFont(4);
    tft.drawString("AUDIO TEST", 8, 8);

    // I2S status — green OK / red FAIL.
    tft.setTextColor(i2sOk ? TFT_DARKGREEN : TFT_RED, TFT_WHITE);
    tft.drawString(i2sOk ? "I2S: OK" : "I2S: FAIL", 8, 44);

    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setTextFont(2);
    char p[40];
    snprintf(p, sizeof(p), "BCK%d  LRCK%d  DIN%d  (SCK->GND)",
             PIN_I2S_BCK, PIN_I2S_LRCK, PIN_I2S_DIN);
    tft.drawString(p, 8, 84);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 1500) {}
    Serial.println(F("\n# ── Omniphone AUDIO test (I2S/PCM5102A) ──"));

    pinMode(PIN_LCD_POWER, OUTPUT); digitalWrite(PIN_LCD_POWER, HIGH);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_WHITE);
    pinMode(PIN_LCD_BL, OUTPUT); digitalWrite(PIN_LCD_BL, HIGH);

    Console.begin();   // WiFi/OTA up FIRST, so the I2S result lands in the history
    audioBegin();      // …that a late-connecting remote serial replays
    drawStaticScreen();
    Serial.println(F("# playing arpeggio — you should hear it"));
}

void loop() {
    Console.handle();
    drawNetStatus(tft);

    static const float notes[] = { 220.0f, 277.18f, 329.63f, 440.0f };
    static uint8_t n = 0;
    static uint32_t last = 0;
    if (millis() - last >= 350) {
        last = millis();
        float hz = notes[n];
        phaseInc = (uint32_t)((double)hz * 4294967296.0 / SAMPLE_RATE);
        // blocks should climb ~86/s (22050/256). If it's STUCK, the audio task is
        // blocked in i2s_write → the I²S peripheral isn't draining DMA (no clock).
        Console.printf("# tone %.0f Hz  blocks=%lu (bytes=%lu)\n",
                       hz, (unsigned long)g_blocks, (unsigned long)g_bytes);
        char t[48];
        snprintf(t, sizeof(t), "Tone %.0f Hz   blocks %lu", hz, (unsigned long)g_blocks);
        tft.fillRect(0, 118, tft.width(), 18, TFT_WHITE);
        tft.setTextFont(2);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.drawString(t, 8, 120);
        n = (n + 1) % 4;
    }
}
