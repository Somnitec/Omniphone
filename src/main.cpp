#include <Arduino.h>
#include <Wire.h>
#include <Audio.h>
#include <math.h>

#include "LCD_Driver.h"
#include "Touch_Driver.h"
#include "GUI_Paint.h"

#include "config.h"
#include "proximity_engine.h"
#include "sound_engine.h"
#include <MPR121.h>

// ─────────────────────────────────────────────────────────────────────────────
// Audio graph — Subtractive synth architecture
//
// 11 independent voices, each with:
//   mainOsc (triangle/saw) + subOsc (sine, -1 oct) -> voiceMix -> filter (LP)
//   -> ampGate (multiplied by dcAmp for click-free level control) -> stage mixer
//
// Three-stage mixer tree to stereo I2S output (PCM5102A DAC).
// ─────────────────────────────────────────────────────────────────────────────

// ── Voice instances ──────────────────────────────────────────────────────────
static Voice voices[NUM_SENSORS];

// ── Mixer tree ───────────────────────────────────────────────────────────────
AudioMixer4    stageMix[3];  // stage 1: up to 4 voices each
AudioMixer4    masterMix;    // stage 2: combines 3 stage mixers
AudioOutputI2S i2sOut;

// ── Per-voice wiring (macro expands to 6 internal + 1 output connection) ─────
//                      voice index, stage mixer,  channel
VOICE_WIRING( 0, stageMix[0], 0)
VOICE_WIRING( 1, stageMix[0], 1)
VOICE_WIRING( 2, stageMix[0], 2)
VOICE_WIRING( 3, stageMix[0], 3)
VOICE_WIRING( 4, stageMix[1], 0)
VOICE_WIRING( 5, stageMix[1], 1)
VOICE_WIRING( 6, stageMix[1], 2)
VOICE_WIRING( 7, stageMix[1], 3)
VOICE_WIRING( 8, stageMix[2], 0)
VOICE_WIRING( 9, stageMix[2], 1)
VOICE_WIRING(10, stageMix[2], 2)

// ── Mixer tree wiring ────────────────────────────────────────────────────────
AudioConnection pm0(stageMix[0], 0, masterMix, 0);
AudioConnection pm1(stageMix[1], 0, masterMix, 1);
AudioConnection pm2(stageMix[2], 0, masterMix, 2);
AudioConnection pmL(masterMix, 0, i2sOut, 0);
AudioConnection pmR(masterMix, 0, i2sOut, 1);

// ─────────────────────────────────────────────────────────────────────────────
// Hardware
// ─────────────────────────────────────────────────────────────────────────────

static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]),
    MPR121(BOARD_ADDRESSES[1]),
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-sensor state
// ─────────────────────────────────────────────────────────────────────────────

static SensorState sensorState[NUM_SENSORS];
static ProximityConfig proxCfg;

// ─────────────────────────────────────────────────────────────────────────────
// LFO — per-voice pitch drift
//
// Each voice has an independent LFO with a slightly different rate
// (spread by LFO_RATE_SPREAD) so adjacent notes drift out of sync,
// giving the instrument a slightly unstable, analogue feel.
// ─────────────────────────────────────────────────────────────────────────────

static float lfoPhase[NUM_SENSORS];
static float lfoRate[NUM_SENSORS];

// ─────────────────────────────────────────────────────────────────────────────
// Sound set state
// ─────────────────────────────────────────────────────────────────────────────

static uint8_t activeSet = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Update timing
// ─────────────────────────────────────────────────────────────────────────────

static uint32_t lastUpdateMs = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Load a sound set — configures all voices
// ─────────────────────────────────────────────────────────────────────────────

static void loadSoundSet(uint8_t index)
{
    if (index >= NUM_SOUND_SETS) return;
    activeSet = index;
    const SoundSet &ss = SOUND_SETS[index];

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        initVoice(voices[i], ss, ss.freqs[i]);
    }

    Paint_Clear(BLACK);
    Paint_DrawString_EN(35, 90, "OMNIPHONE", &Font20, BLACK, WHITE);
    Paint_DrawString_EN(30, 115, ss.name, &Font16, BLACK, WHITE);

    Serial.print(F("# Sound set: "));
    Serial.println(ss.name);
}

// ─────────────────────────────────────────────────────────────────────────────
// Instrument callbacks
// ─────────────────────────────────────────────────────────────────────────────

// Called every frame for each pad.
// intensity 0.0 = hand far away, 1.0 = hand very close.
static void onProximity(uint8_t sensorIndex, float intensity)
{
    const SoundSet &ss = SOUND_SETS[activeSet];

    // Click-free amplitude via DC ramp (sample-accurate interpolation)
    float targetAmp = intensity * VOICE_MAX_AMP;
    setVoiceAmplitude(voices[sensorIndex], targetAmp);

    // Proximity-modulated filter cutoff (brighter when closer)
    setVoiceFilter(voices[sensorIndex], intensity, ss.filterBaseHz, ss.filterMaxHz);
}

// Called once on the frame a contact event is detected.
static void onTouch(uint8_t sensorIndex, float velocity)
{
    (void)sensorIndex;
    (void)velocity;
}

// ─────────────────────────────────────────────────────────────────────────────
// Startup LED animation
// ─────────────────────────────────────────────────────────────────────────────

static void playStartupAnimation()
{
    uint8_t bri[8];
    for (uint8_t v = 0; v <= 15; v++)
    {
        memset(bri, v, sizeof(bri));
        for (uint8_t b = 0; b < NUM_BOARDS; b++)
            boards[b].setLEDs8(bri);
        delay(10);
    }
    for (uint8_t v = 15; v > 0; v--)
    {
        memset(bri, v, sizeof(bri));
        for (uint8_t b = 0; b < NUM_BOARDS; b++)
            boards[b].setLEDs8(bri);
        delay(10);
    }
    memset(bri, 0, sizeof(bri));
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
        boards[b].setLEDs8(bri);
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────

// Force USB Full-Speed (12 Mbps) instead of High-Speed (480 Mbps).
// Runs immediately after usb_init() via weak-symbol override, before the host
// has time to enumerate — so the HS handshake never happens.
extern "C" void startup_late_hook(void) {
    USB1_PORTSC1 |= USB_PORTSC1_PFSC;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    // ── I2C bus ──────────────────────────────────────────────────────────────
    Wire.begin();
    Wire.setClock(400000);

    // ── MPR121 boards ────────────────────────────────────────────────────────
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (!boards[b].begin(SENSE_ELECTRODES[b]))
        {
            Serial.print(F("ERROR: MPR121 board "));
            Serial.print(b);
            Serial.print(F(" at 0x"));
            Serial.print(BOARD_ADDRESSES[b], HEX);
            Serial.println(F(" not found"));
        }
        else
        {
            Serial.print(F("MPR121 board "));
            Serial.print(b);
            Serial.println(F(" OK"));
        }
        boards[b].beginLEDs();
    }

    // ── Seed per-sensor EMA state ────────────────────────────────────────────
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];
        if (sc.electrode == NO_PIN) { seedSensorState(sensorState[i], 0.0f); continue; }
        uint16_t filt = boards[sc.boardIndex].filteredData(sc.electrode);
        uint16_t base = boards[sc.boardIndex].baselineData(sc.electrode);
        int16_t raw = (int16_t)base - (int16_t)filt;
        seedSensorState(sensorState[i], raw < 0 ? 0.0f : (float)raw);
    }

    // ── LFO phases — spread evenly, rates slightly different per voice ───────
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        lfoPhase[i] = (float)i / (float)NUM_SENSORS;
        lfoRate[i]  = LFO_RATE_HZ + LFO_RATE_SPREAD * ((float)i / (float)NUM_SENSORS - 0.5f);
    }

    // ── Teensy Audio ─────────────────────────────────────────────────────────
    AudioMemory(150);

    // Stage mixer gains
    for (int ch = 0; ch < 4; ch++)
    {
        stageMix[0].gain(ch, STAGE_GAIN);
        stageMix[1].gain(ch, STAGE_GAIN);
        stageMix[2].gain(ch, STAGE_GAIN);
    }
    // Master mixer
    for (int ch = 0; ch < 3; ch++)
        masterMix.gain(ch, MASTER_GAIN);
    masterMix.gain(3, 0.0f);

    // ── LCD / touch screen ───────────────────────────────────────────────────
    Touch_1IN28_XY XY;
    XY.mode = 1;

    Config_Init();
    LCD_Init();
    LCD_SetBacklight(200);

    if (Touch_1IN28_init(XY.mode))
        Serial.println(F("Touchscreen OK"));
    else
        Serial.println(F("Touchscreen not found"));

    // ── Startup animation ────────────────────────────────────────────────────
    playStartupAnimation();

    // ── Load default sound set ───────────────────────────────────────────────
    Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, BLACK);
    loadSoundSet(0);

    Serial.println(F("# Omniphone started"));
    Serial.println(F("# Send 0-4 to switch sound sets:"));
    Serial.println(F("#   0=D Kurd  1=Just Intonation  2=Chromatic  3=Sad  4=Happy"));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────

void loop()
{
    uint32_t now = millis();

    // ── Serial command: sound set selection ──────────────────────────────────
    if (Serial.available())
    {
        char c = Serial.read();
        if (c >= '0' && c <= '4')
            loadSoundSet(c - '0');
    }

    // ── Rate-limited sensor update ───────────────────────────────────────────
    if (now - lastUpdateMs < UPDATE_MS)
        return;
    lastUpdateMs = now;

    // ── Burst-read all boards ────────────────────────────────────────────────
    struct BoardData {
        uint8_t  filt[12];
        uint8_t  base[6];
        uint16_t touch;
    };
    static BoardData bd[NUM_BOARDS];

    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        boards[b].burstRead(MPR121Reg::FILT_0L, bd[b].filt, 12);
        boards[b].burstRead(MPR121Reg::BASE_0,  bd[b].base, 6);
        bd[b].touch = boards[b].touchStatus();
    }

    // ── Per-sensor update ────────────────────────────────────────────────────
    // ledBri is indexed by GPIO bit (g → ELE g+4); g=1..7 = ELE5..ELE11.
    static uint8_t ledBri[NUM_BOARDS][8];
    memset(ledBri, 0, sizeof(ledBri));

    const SoundSet &ss = SOUND_SETS[activeSet];

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];

        // Disabled pad (e.g. dropped idx5): keep its voice silent, no LED.
        if (sc.electrode == NO_PIN) { onProximity(i, 0.0f); continue; }

        const BoardData &b = bd[sc.boardIndex];
        uint8_t e = sc.electrode;

        // Reconstruct 10-bit values from burst buffers
        uint16_t filtered = (uint16_t)b.filt[2 * e]
                          | ((uint16_t)(b.filt[2 * e + 1] & 0x03) << 8);
        uint16_t baseline = (uint16_t)b.base[e] << 2;

        float rawDelta = (float)((int16_t)baseline - (int16_t)filtered);
        if (rawDelta < 0.0f) rawDelta = 0.0f;

        float intensity;
        bool  isTouch;
        updateProximity(rawDelta, now, proxCfg, sensorState[i], intensity, isTouch);

        if (isTouch) onTouch(i, sensorState[i].velocity);
        onProximity(i, intensity);

        // ── LFO pitch drift ──────────────────────────────────────────────────
        lfoPhase[i] += lfoRate[i] * (UPDATE_MS / 1000.0f);
        if (lfoPhase[i] >= 1.0f) lfoPhase[i] -= 1.0f;

        float freq = ss.freqs[i] * (1.0f + LFO_AMOUNT * sinf(2.0f * (float)M_PI * lfoPhase[i]));
        setVoiceFrequency(voices[i], freq);

        // ── LED brightness ───────────────────────────────────────────────────
        // Explicit LED board + pin; GPIO bit = ledEle - 4 (ELE5→1 … ELE11→7).
        // ledBoard may differ from the sense board (idx1: sense C, LED A).
        if (sc.ledEle != NO_PIN)
        {
            ledBri[sc.ledBoard][sc.ledEle - 4] = (intensity < 0.001f) ? 0
                : static_cast<uint8_t>(1.0f + intensity * 14.0f + 0.5f);
        }
    }

    // ── Batch LED update ─────────────────────────────────────────────────────
    // ELE9 (GPIO bit 5) has nothing wired to it, but the chip only lights the
    // ELE10 LED (bit 6) when ELE9 is driven the same — mirror it per board.
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        ledBri[b][5] = ledBri[b][6]; // ELE9 ← ELE10
        boards[b].setLEDs8(ledBri[b]);
    }

    // ── Diagnostic output (10 Hz) ────────────────────────────────────────────
    static uint32_t lastDiagMs = 0;
    if (now - lastDiagMs >= 100)
    {
        lastDiagMs = now;
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            float norm = (sensorState[i].fast - proxCfg.proxDeadband)
                       / (proxCfg.proxMax - proxCfg.proxDeadband);
            float intensity = norm < 0.0f ? 0.0f : (norm > 1.0f ? 1.0f : norm);

            Serial.print(i);
            Serial.print(F(":"));
            Serial.print(intensity, 2);
            Serial.print(F(" "));
        }
        Serial.print(F(" CPU:"));
        Serial.print(AudioProcessorUsage(), 1);
        Serial.print(F("% MEM:"));
        Serial.print(AudioMemoryUsage());
        Serial.println();
    }
}
