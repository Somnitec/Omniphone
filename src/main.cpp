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
AudioMixer4    masterMix;    // stage 2: combines 3 stage mixers + bell on ch 3
AudioOutputI2S i2sOut;

// ── Bell — one touch-transient voice per pad ─────────────────────────────────
// One voice per pad (indexed by sensor) so several players striking different
// pads at once never steal each other. Each voice: two sine partials
// (fundamental + inharmonic) → fast envelope → 3-mixer tree → master ch 3.
static constexpr uint8_t NUM_BELLS = NUM_SENSORS;
static_assert(NUM_BELLS == 11, "BELL_WIRING below is hand-wired for 11 voices");

AudioSynthWaveformSine   bellOscA[NUM_BELLS];
AudioSynthWaveformSine   bellOscB[NUM_BELLS];
AudioMixer4              bellVoiceMix[NUM_BELLS];
AudioFilterStateVariable bellFilt[NUM_BELLS]; // per-voice LP for pressure aftertouch
AudioEffectEnvelope      bellEnv[NUM_BELLS];
AudioMixer4              bellGroup[3]; // 4+4+3 voices
AudioMixer4              bellSum;      // groups → master ch 3

#define BELL_WIRING(K, G, CH)                                                \
    AudioConnection bw##K##a(bellOscA[K],     0, bellVoiceMix[K], 0);        \
    AudioConnection bw##K##b(bellOscB[K],     0, bellVoiceMix[K], 1);        \
    AudioConnection bw##K##f(bellVoiceMix[K], 0, bellFilt[K],     0);        \
    AudioConnection bw##K##m(bellFilt[K],     0, bellEnv[K],      0);        \
    AudioConnection bw##K##e(bellEnv[K],      0, bellGroup[G],    CH);
BELL_WIRING( 0, 0, 0)
BELL_WIRING( 1, 0, 1)
BELL_WIRING( 2, 0, 2)
BELL_WIRING( 3, 0, 3)
BELL_WIRING( 4, 1, 0)
BELL_WIRING( 5, 1, 1)
BELL_WIRING( 6, 1, 2)
BELL_WIRING( 7, 1, 3)
BELL_WIRING( 8, 2, 0)
BELL_WIRING( 9, 2, 1)
BELL_WIRING(10, 2, 2)
AudioConnection bgs0(bellGroup[0], 0, bellSum, 0);
AudioConnection bgs1(bellGroup[1], 0, bellSum, 1);
AudioConnection bgs2(bellGroup[2], 0, bellSum, 2);
AudioConnection bellOut(bellSum, 0, masterMix, 3);

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
// Balanced output: I2S L = +master, R = −master (180° inverted) so the two
// channels drive a balanced differential line out (L−R = 2× signal, common-
// mode noise cancels).
AudioAmplifier  rInv;
AudioConnection pmL (masterMix, 0, i2sOut, 0);
AudioConnection pmRa(masterMix, 0, rInv,   0);
AudioConnection pmR (rInv,      0, i2sOut, 1);

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

// Idle-recalibration state — kicks a full MPR121 baseline reload after a
// long quiet period so slow drift can't produce phantom blips.
static uint32_t lastActivityMs = 0;
static uint32_t lastRecalMs    = 0;

// Per-pad MPE / MIDI state.
static uint8_t  midiNote     [NUM_BELLS] = { 0 };
static uint8_t  lastPress127 [NUM_BELLS] = { 0 };
static uint32_t lastPressMs  [NUM_BELLS] = { 0 };

static inline uint8_t midiNoteFromFreq(float hz)
{
    int n = (int)lroundf(69.0f + 12.0f * log2f(hz / 440.0f));
    if (n < 0)   n = 0;
    if (n > 127) n = 127;
    return (uint8_t)n;
}

static inline uint8_t mpeChannelFor(uint8_t pad)
{
    return (uint8_t)(MPE_MEMBER_BASE_CH + pad);
}

// Force a full MPR121 baseline reload (CL=11) on every board and re-seed the
// proximity EMAs. ~60 ms of sensor pause; only ever called during a quiet
// period so it isn't audible.
static void recalibrateBaselines()
{
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        boards[b].write(MPR121Reg::ECR, 0x00);
        delay(10);
        boards[b].write(MPR121Reg::ECR, (uint8_t)(0b11000000 | SENSE_ELECTRODES[b]));
        delay(50);
    }
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];
        if (sc.electrode == NO_PIN) { seedSensorState(sensorState[i], 0.0f); continue; }
        uint16_t filt = boards[sc.boardIndex].filteredData(sc.electrode);
        uint16_t base = boards[sc.boardIndex].baselineData(sc.electrode);
        int16_t raw = (int16_t)base - (int16_t)filt;
        seedSensorState(sensorState[i], raw < 0 ? 0.0f : (float)raw);
    }
    Serial.println(F("# baseline recalibrated (idle)"));
}

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

#if ENABLE_SCREEN
    Paint_Clear(BLACK);
    Paint_DrawString_EN(35, 90, "OMNIPHONE", &Font20, BLACK, WHITE);
    Paint_DrawString_EN(30, 115, ss.name, &Font16, BLACK, WHITE);
#endif

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

// Per-pad bell state.
static bool     bellHeld[NUM_BELLS]   = { false }; // sounding, awaiting lift → release
static uint32_t bellArmMs[NUM_BELLS]  = { 0 };     // peak-velocity window deadline
static float    bellPeakV[NUM_BELLS]  = { 0.0f };  // peak contact velocity so far
static float    bellPress[NUM_BELLS]  = { 0.0f };  // smoothed aftertouch pressure 0–1

// While a pad is held, the live contact strength modulates the bell's loudness
// and its low-pass cutoff (polyphonic-aftertouch feel). `fast` is the raw
// proximity EMA (hundreds when pressing metal), not the saturated intensity.
static void bellApplyPressure(uint8_t k, float fast)
{
    float p = (fast - BELL_AFTER_MIN) / (BELL_AFTER_MAX - BELL_AFTER_MIN);
    p = p < 0.0f ? 0.0f : (p > 1.0f ? 1.0f : p);
    bellPress[k] += BELL_PRESS_SMOOTH * (p - bellPress[k]); // 1-pole anti-zipper
    float ps = bellPress[k];

    float amp = BELL_AMP * (BELL_FLOOR + (1.0f - BELL_FLOOR) * ps);
    bellOscA[k].amplitude(amp);
    bellOscB[k].amplitude(amp * 0.5f);
    bellFilt[k].frequency(BELL_FILT_MIN_HZ + ps * (BELL_FILT_MAX_HZ - BELL_FILT_MIN_HZ));
}

// Contact velocity → bell oscillator amplitude (the dynamics curve).
static float bellAmpFromVel(float vel)
{
    float n = (vel - BELL_VEL_MIN) / (BELL_VEL_MAX - BELL_VEL_MIN);
    n *= BELL_STRIKE_GAIN;
    n = n < 0.0f ? 0.0f : (n > 1.0f ? 1.0f : n);
    return BELL_AMP * (BELL_FLOOR + (1.0f - BELL_FLOOR) * powf(n, BELL_CURVE));
}

static void bellSetAmp(uint8_t k, float vel)
{
    float amp = bellAmpFromVel(vel);
    bellOscA[k].amplitude(amp);
    bellOscB[k].amplitude(amp * 0.5f);
}

// Called once on the frame a contact event is detected (rising edge). Triggers
// the bell immediately at the provisional velocity, then opens a short window
// during which loop() tracks the PEAK velocity and re-levels — the long-ish
// attack means that final level lands smoothly with no click or latency.
// One dedicated bell voice per pad (k = sensorIndex).
static void onTouch(uint8_t sensorIndex, float velocity, uint32_t nowMs)
{
    const SoundSet &ss = SOUND_SETS[activeSet];
    float fund = ss.freqs[sensorIndex] * BELL_OCTAVES;

    uint8_t k = sensorIndex; // one bell per pad

    bellOscA[k].frequency(fund);
    bellOscB[k].frequency(fund * BELL_PARTIAL);
    bellSetAmp(k, velocity);
    bellEnv[k].noteOn();

    bellHeld[k]  = true;                          // sustain until the finger lifts
    bellArmMs[k] = nowMs + BELL_STRIKE_WIN_MS;    // refine level for this long
    bellPeakV[k] = velocity;

    // Seed the aftertouch pressure from the hit strength so the sustain
    // doesn't dip before the live pressure tracking eases in.
    float n = (velocity - BELL_VEL_MIN) / (BELL_VEL_MAX - BELL_VEL_MIN);
    bellPress[k] = n < 0.0f ? 0.0f : (n > 1.0f ? 1.0f : n);

    // ── MPE: noteOn on the pad's member channel ──
    if (MPE_ENABLE)
    {
        midiNote[k] = midiNoteFromFreq(ss.freqs[sensorIndex]);
        int vel = (int)lroundf(n * 127.0f);
        if (vel < 1)   vel = 1;     // 0 = noteOff in MIDI
        if (vel > 127) vel = 127;
        usbMIDI.sendNoteOn(midiNote[k], (uint8_t)vel, mpeChannelFor(k));
        lastPress127[k] = 0;
        lastPressMs[k]  = 0;
    }
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
    while (!Serial && millis() < 200) {}

    // ── I2C bus ──────────────────────────────────────────────────────────────
    Wire.begin();
    Wire.setClock(100000);

    // ── MPR121 boards ────────────────────────────────────────────────────────
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (!boards[b].begin(SENSE_ELECTRODES[b], 40, 20, SENSOR_CDC, SENSOR_CDT))
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
    AudioMemory(100); // headroom for 11 voices + 11 filtered bell voices (peak polyphony)

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
    masterMix.gain(3, BELL_MIX); // ch 3 = summed bell voices
    rInv.gain(-1.0f);            // I2S right = inverted master → balanced output

    // Bell voices
    for (uint8_t k = 0; k < NUM_BELLS; k++)
    {
        bellVoiceMix[k].gain(0, 0.65f); // fundamental partial
        bellVoiceMix[k].gain(1, 0.35f); // inharmonic partial
        bellVoiceMix[k].gain(2, 0.0f);
        bellVoiceMix[k].gain(3, 0.0f);
        bellEnv[k].attack(BELL_ATTACK_MS);
        bellEnv[k].hold(0.0f);
        bellEnv[k].decay(BELL_DECAY_MS);
        bellEnv[k].sustain(BELL_SUSTAIN); // rings while the pad is held
        bellEnv[k].release(BELL_RELEASE_MS);
        bellFilt[k].resonance(BELL_FILT_Q);
        bellFilt[k].frequency(BELL_FILT_MIN_HZ);
    }
    for (uint8_t g = 0; g < 3; g++)
        for (uint8_t ch = 0; ch < 4; ch++)
            bellGroup[g].gain(ch, 0.6f); // headroom for many simultaneous bells
    bellSum.gain(0, 1.0f);
    bellSum.gain(1, 1.0f);
    bellSum.gain(2, 1.0f);
    bellSum.gain(3, 0.0f);

    // Touch trigger tuning (see config.h)
    proxCfg.jumpThreshold     = TOUCH_JUMP_THRESHOLD;
    proxCfg.releaseRatio      = TOUCH_RELEASE_RATIO;
    proxCfg.cooldownMs        = TOUCH_COOLDOWN_MS;
    proxCfg.confirmFrames     = TOUCH_CONFIRM_FRAMES;
    proxCfg.proxConfirmFrames = PROX_CONFIRM_FRAMES;
    proxCfg.proxDeadband      = PROX_DEADBAND;
    proxCfg.proxMax           = PROX_MAX;

    // ── LCD / touch screen ───────────────────────────────────────────────────
#if ENABLE_SCREEN
    Touch_1IN28_XY XY;
    XY.mode = 1;

    Config_Init();
    LCD_Init();
    LCD_SetBacklight(200);

    if (Touch_1IN28_init(XY.mode))
        Serial.println(F("Touchscreen OK"));
    else
        Serial.println(F("Touchscreen not found"));
#endif

    // ── Startup animation ────────────────────────────────────────────────────
    playStartupAnimation();

    // ── Load default sound set ───────────────────────────────────────────────
#if ENABLE_SCREEN
    Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, BLACK);
#endif
    loadSoundSet(5); // start on the new Bright Pentatonic set

    // ── MPE Configuration Message ────────────────────────────────────────────
    // RPN 6 on the master channel sets up MPE Zone 1: 11 member channels
    // (pad 0 → ch 2, … pad 10 → ch 12).
    if (MPE_ENABLE)
    {
        const uint8_t mch = MPE_MASTER_CH;
        usbMIDI.sendControlChange(101, 0,            mch); // RPN MSB
        usbMIDI.sendControlChange(100, 6,            mch); // RPN LSB = 6 (MCM)
        usbMIDI.sendControlChange(  6, NUM_BELLS,    mch); // # of member channels
        usbMIDI.sendControlChange(101, 127,          mch); // RPN null (MSB)
        usbMIDI.sendControlChange(100, 127,          mch); // RPN null (LSB)
    }

    lastActivityMs = millis();

    Serial.println(F("# Omniphone started"));
    Serial.println(F("# Send 0-5 to switch sound sets:"));
    Serial.println(F("#   0=D Kurd 1=Just 2=Chromatic 3=Sad 4=Happy 5=Bright Pentatonic"));
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
        if (c >= '0' && c <= '5')
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

    bool anyActive = false;

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

        if (intensity > IDLE_INTENSITY || bellHeld[i]) anyActive = true;

        if (isTouch)
        {
            // Contact velocity drives the dynamics (provisional level now;
            // refined to the peak over the next BELL_STRIKE_WIN_MS).
            onTouch(i, sensorState[i].velocity, now);
        }
        else if (now < bellArmMs[i])
        {
            if (sensorState[i].velocity > bellPeakV[i])
            {
                bellPeakV[i] = sensorState[i].velocity;
                bellSetAmp(i, bellPeakV[i]); // re-level during the attack
            }
        }
        else if (bellHeld[i])
        {
            if (intensity < BELL_HOLD_RELEASE)
            {
                // Finger lifted → ADSR release (the ring tails out).
                bellEnv[i].noteOff();
                bellHeld[i] = false;
                if (MPE_ENABLE)
                    usbMIDI.sendNoteOff(midiNote[i], 0, mpeChannelFor(i));
            }
            else
            {
                // Held: live contact strength → bell loudness + filter.
                bellApplyPressure(i, sensorState[i].fast);

                // MPE: per-channel pressure (poly aftertouch), throttled.
                if (MPE_ENABLE && (now - lastPressMs[i]) >= MPE_PRESSURE_THROTTLE_MS)
                {
                    int p = (int)lroundf(bellPress[i] * 127.0f);
                    if (p < 0)   p = 0;
                    if (p > 127) p = 127;
                    int d = p - (int)lastPress127[i];
                    if (d < 0) d = -d;
                    if (d >= (int)MPE_PRESSURE_MIN_DELTA)
                    {
                        usbMIDI.sendAfterTouch((uint8_t)p, mpeChannelFor(i));
                        lastPress127[i] = (uint8_t)p;
                        lastPressMs[i]  = now;
                    }
                }
            }
        }
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

    // ── Idle baseline recalibration ──────────────────────────────────────────
    if (anyActive)
    {
        lastActivityMs = now;
    }
    else if ((now - lastActivityMs) > IDLE_RECAL_MS &&
             (now - lastRecalMs)    > RECAL_COOLDOWN_MS)
    {
        recalibrateBaselines();
        lastRecalMs    = now;
        lastActivityMs = now;
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
