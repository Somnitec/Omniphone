#include <Arduino.h>
#include <Wire.h>
#include <Audio.h>
#include <EEPROM.h>
#include <math.h>

#include "config.h"
#include "display.h"   // all LCD/touch/GUI code (no-op when ENABLE_SCREEN == 0)
#include "proximity_engine.h"
#include "sound_engine.h"
#include "benjolin.h"    // Benjolin-mode chaos source
#include "cracklebox.h"  // Cracklebox-mode chaos source
#include "hangbow.h"     // Hang Bow mode: bowed handpan physical model
#include "swarm.h"       // Swarm mode: additive chime cloud
#include "grainsample.h" // Grain Hang mode: granular hang-drum sample
#include "strings.h"     // Strings mode: looped string-section sample
#include "bowls.h"       // Bowls mode: rubbed singing bowls
#include "tanpura.h"     // Tanpura mode: jawari drone strings
#include "flute.h"       // Flute mode: breath waveguide wind
#include "cello.h"       // Cello mode: bowed-string waveguide
#include "phasechimes.h" // Phase Chimes mode: generative phasing mallets
#include "freezer.h"     // Freeze mode: click-free windowed freeze layer
#include <MPR121.h>

// ─────────────────────────────────────────────────────────────────────────────
// Audio graph — Subtractive synth architecture
//
// 12 independent voices, each with:
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
static_assert(NUM_BELLS == 12, "BELL_WIRING below is hand-wired for 12 voices");

AudioSynthWaveformSine   bellOscA[NUM_BELLS];
AudioSynthWaveformSine   bellOscB[NUM_BELLS];
AudioMixer4              bellVoiceMix[NUM_BELLS];
AudioFilterStateVariable bellFilt[NUM_BELLS]; // per-voice LP for pressure aftertouch
AudioEffectEnvelope      bellEnv[NUM_BELLS];
AudioMixer4              bellGroup[3]; // 4+4+4 voices
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
BELL_WIRING(11, 2, 3)
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
VOICE_WIRING(11, stageMix[2], 3)

// ── Mixer tree wiring ────────────────────────────────────────────────────────
AudioConnection pm0(stageMix[0], 0, masterMix, 0);
AudioConnection pm1(stageMix[1], 0, masterMix, 1);
AudioConnection pm2(stageMix[2], 0, masterMix, 2);

// ── FM mode: mono FM carrier + a modulator bus that taps the 12 voice oscs ───
// In FM mode the normal voices are muted but their oscillators keep running at
// the held-pad pitches; we sum the modulator pads into the carrier's FM input.
AudioSynthWaveformModulated fmCarrier;
AudioMixer4          fmModMix[3]; // 12 mainOsc taps → 3 mixers
AudioMixer4          fmModSum;    // 3 → 1 → carrier FM input
AudioSynthWaveformDc fmDc;        // carrier amplitude (click-free gate)
AudioEffectMultiply  fmGate;
#define FM_MOD_WIRING(N, MIX, CH) AudioConnection fmmod##N(voices[N].mainOsc, 0, fmModMix[MIX], CH);
FM_MOD_WIRING( 0, 0, 0) FM_MOD_WIRING( 1, 0, 1) FM_MOD_WIRING( 2, 0, 2) FM_MOD_WIRING( 3, 0, 3)
FM_MOD_WIRING( 4, 1, 0) FM_MOD_WIRING( 5, 1, 1) FM_MOD_WIRING( 6, 1, 2) FM_MOD_WIRING( 7, 1, 3)
FM_MOD_WIRING( 8, 2, 0) FM_MOD_WIRING( 9, 2, 1) FM_MOD_WIRING(10, 2, 2) FM_MOD_WIRING(11, 2, 3)
AudioConnection fms0(fmModMix[0], 0, fmModSum,  0);
AudioConnection fms1(fmModMix[1], 0, fmModSum,  1);
AudioConnection fms2(fmModMix[2], 0, fmModSum,  2);
AudioConnection fmci(fmModSum,    0, fmCarrier, 0); // FM input
AudioConnection fmg0(fmCarrier,   0, fmGate,    0);
AudioConnection fmg1(fmDc,        0, fmGate,    1);

// ── Benjolin / Cracklebox modes: chaos sources ──────────────────────────────
AudioBenjolin   benjolin;
AudioCracklebox crackle;
AudioHangBow    hangBow;

// ── Final bus: normal / FM / Benjolin / Cracklebox → chorus → output ─────────
// finalMix ch0 = normal voice+bell master (also FM Poly), ch1 = mono FM,
// ch2 = Benjolin, ch3 = Cracklebox. The active mode raises the right one(s).
// finalMix is full (4ch), so a second mixer folds in the Hang Bow engine.
AudioMixer4 finalMix;
AudioConnection fx0(masterMix, 0, finalMix, 0);
AudioConnection fx1(fmGate,    0, finalMix, 1);
AudioConnection fx2(benjolin,  0, finalMix, 2);
AudioConnection fx3(crackle,   0, finalMix, 3);

// ── The calming batch: eight more engines on two sub-mixers ─────────────────
AudioChimeSwarm  swarm;
AudioGrainHang   grainHang;
AudioStringPad   stringPad;
AudioSingingBowls bowls;
AudioTanpura     tanpura;
AudioBreathFlute flutes;
AudioBowedString cello;
AudioPhaseChimes phaseChimes;
AudioMixer4 engMixA;   // ch0 swarm, ch1 grain, ch2 strings, ch3 bowls
AudioMixer4 engMixB;   // ch0 tanpura, ch1 flute, ch2 cello, ch3 phase chimes
AudioConnection emA0(swarm,      0, engMixA, 0);
AudioConnection emA1(grainHang,  0, engMixA, 1);
AudioConnection emA2(stringPad,  0, engMixA, 2);
AudioConnection emA3(bowls,      0, engMixA, 3);
AudioConnection emB0(tanpura,    0, engMixB, 0);
AudioConnection emB1(flutes,     0, engMixB, 1);
AudioConnection emB2(cello,      0, engMixB, 2);
AudioConnection emB3(phaseChimes,0, engMixB, 3);

// finalMix2 ch0 = everything above, ch1 = Hang Bow, ch2/ch3 = the engine
// sub-mixers (each engine's bus level is set on engMixA/B by loadMode).
AudioMixer4 finalMix2;
AudioConnection fx2a(finalMix, 0, finalMix2, 0);
AudioConnection fx2b(hangBow,  0, finalMix2, 1);
AudioConnection fx2c(engMixA,  0, finalMix2, 2);
AudioConnection fx2d(engMixB,  0, finalMix2, 3);

// ── Cathedral / Freeze: stock effects on the summed bus ─────────────────────
// fxMix ch0 = dry (always 1), ch1 = freeverb wet (Cathedral), ch2 = granular
// frozen layer (Freeze). The dry path stays untouched in every other mode.
AudioEffectFreeverb  freeverb;
AudioFreezeSmear     freezer; // windowed grain-cloud freeze (no loop ticking)
AudioMixer4 fxMix;
AudioConnection fxd(finalMix2, 0, fxMix,    0);
AudioConnection fxv(finalMix2, 0, freeverb, 0);
AudioConnection fxw(freeverb,  0, fxMix,    1);
AudioConnection fxg(finalMix2, 0, freezer,  0);
AudioConnection fxz(freezer,   0, fxMix,    2);

// Chorus send on the final bus (for the Juno timbre's ensemble shimmer).
AudioEffectChorus chorus;
#define CHORUS_DELAY_LEN (16 * AUDIO_BLOCK_SAMPLES)
static short chorusDelayLine[CHORUS_DELAY_LEN];

// Balanced output: I2S L = +master, R = −master (180° inverted) so the two
// channels drive a balanced differential line out (L−R = 2× signal, common-
// mode noise cancels). The chorus sits between the final mix and the split.
AudioAmplifier  rInv;
AudioAmplifier  masterVol;  // ONE global volume for everything (MASTER_VOLUME)
AudioConnection pmV (fxMix,     0, masterVol, 0);
AudioConnection pmC (masterVol, 0, chorus,    0);
AudioConnection pmL (chorus,   0, i2sOut, 0);
AudioConnection pmRa(chorus,   0, rInv,   0);
AudioConnection pmR (rInv,     0, i2sOut, 1);

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

// Per-voice glided base frequency (pre-LFO). On a live scale change this slides
// toward the new note over SET_GLIDE_MS (fixed-time log sweep), then stops.
static float    glideFreq[NUM_SENSORS];  // current (live) pitch per voice
static float    glideStart[NUM_SENSORS]; // pitch at the moment the glide began
static uint32_t glideStartMs = 0;        // millis() when the current glide began

// Live (morphing) timbre params. On a live timbre change the filter cutoff
// range, resonance and sub blend slide from the start snapshot to the target
// over TIMBRE_MORPH_MS; the waveform switches instantly. cur* are what the
// voices actually use each frame (onProximity reads cur cutoffs).
static float    curSub  = 0, curBaseHz = 0, curMaxHz = 0, curQ = 0;
static float    morphStartSub = 0, morphStartBase = 0, morphStartMax = 0, morphStartQ = 0;
static float    morphTgtSub   = 0, morphTgtBase   = 0, morphTgtMax   = 0, morphTgtQ   = 0;
static uint32_t timbreMorphStartMs = 0;

// ── Play mode + arpeggiator state ────────────────────────────────────────────
static uint8_t  activeMode    = MODE_PROX;             // play mode (see PlayMode)
static uint8_t  arpCurrentPad = 0xFF;                  // pad sounding now (0xFF = none)
static uint32_t arpNextTickMs = 0;                     // when to step to the next held pad
static float    lastIntensity[NUM_SENSORS] = { 0.0f }; // prev-frame intensity → held detect
static uint32_t holdStartMs[NUM_SENSORS]   = { 0 };    // when each pad became held (0 = released)
                                                       // → press order for FM carrier / Benjolin

// Screen state: 0 = normal, 1 = harmonic journey, 2 = locked visualiser.
// Cycles on each 5-second hold: normal → harmonic → locked → normal.
static uint8_t  screenState = 0;
static bool     rawDumpOn   = false;       // serial 'r': stream raw 10-bit filtered CSV

// Harmonic Journey: override the pad pitches with the selected chord's tones.
// harmonicOverride is true once a chord has been chosen; loadHarmonicChord()
// sets harmonicFreqs[] and starts a glide, and the loop reads harmonicFreqs[]
// as the glide target. activeJourney selects which atlas (see HARMONIC_JOURNEYS);
// harmonicPicking is true while the atlas picker is showing (no chord chosen yet).
static uint8_t  activeJourney       = 0;
static uint8_t  activeHarmonicChord = 0;
static uint8_t  activeTonalNode     = 0;   // Tonal Map centre chord (0…47; 0 = C)
static float    harmonicFreqs[NUM_SENSORS] = { 0.0f };
static bool     harmonicOverride    = false;
static bool     harmonicPicking     = false;
// Harmonic journey always plays in Proximity; this remembers the play mode in
// effect before entering so it can be restored on the way out.
static uint8_t  modeBeforeHarmonic  = MODE_PROX;
static bool     teleplotOn  = false;
static uint16_t tpFilt[NUM_SENSORS]  = { 0 }; // last MPR121 filtered reading per pad
static float    tpDelta[NUM_SENSORS] = { 0 }; // last baseline−filtered delta per pad

// ─────────────────────────────────────────────────────────────────────────────
// Sound state — scale (notes) and timbre (character) are independent
// ─────────────────────────────────────────────────────────────────────────────

static uint8_t activeScale  = 0; // index into SCALE_SETS  (swipe left/right)
static uint8_t activeTimbre = 0; // index into TIMBRE_SETS (swipe up/down)

// ── Persisted selection (survives power-off) ─────────────────────────────────
// Teensy 4.0 emulated EEPROM. A magic byte validates the slots so a blank or
// previously-unused chip falls back to the compile-time defaults rather than
// reading whatever garbage happens to be there. Scale and timbre live in
// separate bytes so each is remembered on its own.
static constexpr int     EEPROM_ADDR_MAGIC  = 0;
static constexpr int     EEPROM_ADDR_SCALE  = 1;
static constexpr int     EEPROM_ADDR_TIMBRE = 2;
static constexpr int     EEPROM_ADDR_MODE   = 3;
static constexpr int     EEPROM_ADDR_SCREEN  = 4; // 0=normal, 1=harmonic, 2=locked
static constexpr int     EEPROM_ADDR_HARM    = 5; // last selected harmonic chord
static constexpr int     EEPROM_ADDR_JOURNEY = 6; // last selected harmonic atlas
static constexpr int     EEPROM_ADDR_TONAL   = 7; // last Tonal Map centre chord
static constexpr uint8_t EEPROM_MAGIC        = 0xAD; // bumped: added Tonal Map slot

static uint8_t loadStoredScale(uint8_t fallback)
{
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) return fallback;
    uint8_t v = EEPROM.read(EEPROM_ADDR_SCALE);
    return (v < NUM_SCALE_SETS) ? v : fallback;
}

static uint8_t loadStoredTimbre(uint8_t fallback)
{
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) return fallback;
    uint8_t v = EEPROM.read(EEPROM_ADDR_TIMBRE);
    return (v < NUM_TIMBRE_SETS) ? v : fallback;
}

static void storeScale(uint8_t index)
{
    // .update() only writes when the byte differs → avoids needless EEPROM wear.
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_SCALE, index);
}

static void storeTimbre(uint8_t index)
{
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_TIMBRE, index);
}

static uint8_t loadStoredMode(uint8_t fallback)
{
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) return fallback;
    uint8_t v = EEPROM.read(EEPROM_ADDR_MODE);
    return (v < NUM_MODES) ? v : fallback;
}

static void storeMode(uint8_t index)
{
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_MODE, index);
}

static uint8_t loadStoredScreen(uint8_t fallback)
{
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) return fallback;
    uint8_t v = EEPROM.read(EEPROM_ADDR_SCREEN);
    return (v < 3) ? v : fallback;
}

static void storeScreen(uint8_t s)
{
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_SCREEN, s);
}

static uint8_t loadStoredHarmChord(uint8_t fallback)
{
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) return fallback;
    uint8_t v = EEPROM.read(EEPROM_ADDR_HARM);
    return (v < HARMONIC_MAX_NODES) ? v : fallback;
}

static void storeHarmChord(uint8_t c)
{
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_HARM, c);
}

static uint8_t loadStoredJourney(uint8_t fallback)
{
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) return fallback;
    uint8_t v = EEPROM.read(EEPROM_ADDR_JOURNEY);
    return (v < NUM_HARMONIC_JOURNEYS) ? v : fallback;
}

static void storeJourney(uint8_t j)
{
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_JOURNEY, j);
}

static uint8_t loadStoredTonalNode(uint8_t fallback)
{
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) return fallback;
    uint8_t v = EEPROM.read(EEPROM_ADDR_TONAL);
    return (v < TONAL_NODES) ? v : fallback;
}

static void storeTonalNode(uint8_t n)
{
    EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.update(EEPROM_ADDR_TONAL, n);
}

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

// Forward decl — definition is below the bell state (uses bellHeld / bellPress).
static void recalibrateBaselines();

// ─────────────────────────────────────────────────────────────────────────────
// Load scale / timbre — independent. Both redraw the screen and persist.
// ─────────────────────────────────────────────────────────────────────────────

// Swipe LEFT/RIGHT — change the SCALE (which note each pad plays). Pitch only;
// the voices keep sounding and the loop glides each note to its new target
// (glide=true). At boot glide=false snaps the glide frequencies into place.
static void loadScale(uint8_t index, bool glide)
{
    if (index >= NUM_SCALE_SETS) return;
    activeScale = index;
    storeScale(index); // remember across power-off
    const ScaleSet &sc = SCALE_SETS[index];

    if (glide)
    {
        // Snapshot the current pitches and start a fixed-time glide; the loop
        // sweeps glideFreq[] from glideStart[] to sc.freqs[] over SET_GLIDE_MS.
        glideStartMs = millis();
        for (uint8_t i = 0; i < NUM_SENSORS; i++) glideStart[i] = glideFreq[i];
    }
    else
    {
        for (uint8_t i = 0; i < NUM_SENSORS; i++) glideFreq[i] = glideStart[i] = sc.freqs[i];
        glideStartMs = 0; // already-elapsed → no glide at boot
    }

    if (activeMode == MODE_BENJOLIN) // keep the chaos quantiser on the new scale
        benjolin.setQuantize(sc.freqs, NUM_SENSORS);
    phaseChimes.setScale(sc.freqs, NUM_SENSORS); // pattern notes follow the scale

    displayShowScale(sc.name); // incremental redraw of just the scale line
    Serial.print(F("# Scale: "));
    Serial.println(sc.name);
}

// Swipe UP/DOWN — change the TIMBRE (sound character). At boot glide=false does
// the full voice init; live (glide=true) updates timbre only, leaving amplitude
// and phase untouched so held notes keep sounding with no pause.
static void loadTimbre(uint8_t index, bool glide)
{
    if (index >= NUM_TIMBRE_SETS) return;
    activeTimbre = index;
    storeTimbre(index); // remember across power-off
    const TimbreSet &t = TIMBRE_SETS[index];

    morphTgtSub  = t.subMix;
    morphTgtBase = t.filterBaseHz;
    morphTgtMax  = t.filterMaxHz;
    morphTgtQ    = t.filterQ;

    if (glide)
    {
        // Waveform can't crossfade on a single oscillator, so switch it now;
        // the filter/sub MORPH from the current live values to the target over
        // TIMBRE_MORPH_MS (the loop interpolates cur* and applies them).
        for (uint8_t i = 0; i < NUM_SENSORS; i++) setVoiceWaveform(voices[i], t.waveformType);
        morphStartSub  = curSub;
        morphStartBase = curBaseHz;
        morphStartMax  = curMaxHz;
        morphStartQ    = curQ;
        timbreMorphStartMs = millis();
    }
    else
    {
        // Boot: full voice init + snap the live timbre to the target.
        for (uint8_t i = 0; i < NUM_SENSORS; i++) initVoice(voices[i], t, glideFreq[i]);
        curSub = morphStartSub = morphTgtSub;
        curBaseHz = morphStartBase = morphTgtBase;
        curMaxHz = morphStartMax = morphTgtMax;
        curQ = morphStartQ = morphTgtQ;
        timbreMorphStartMs = 0;
    }
    masterMix.gain(3, t.bellMix);          // per-timbre bell loudness (instant)
    chorus.voices(t.chorus ? 3 : 1);       // wet ensemble for Juno, dry otherwise

    displayShowTimbre(t.name); // incremental redraw of just the timbre line
    Serial.print(F("# Timbre: "));
    Serial.println(t.name);
}

// Play mode — Proximity / Arp Slow / Med / Fast. Tap the screen centre (or
// serial 'm') to cycle. Persisted across power-off.
// Freeze-mode state: whether a frozen layer is currently captured, and its
// fade envelope (slewed each frame, drives fxMix ch2).
static bool  frzFrozen = false;
static float frzEnv    = 0.0f;

// Decay setting for Swarm/Strings. In those modes it takes over the TIMBRE
// line (which they don't use): the line reads "Decay 1.2s" and the same ▲▼
// arrows adjust it instead of cycling timbres.
static float swarmDecay = 1.2f;   // Swarm ring-out, seconds
static float stringsRel = 0.6f;   // Strings release, seconds

static void showDecayParam()
{
    char buf[20];
    float v = (activeMode == MODE_SWARM) ? swarmDecay : stringsRel;
    snprintf(buf, sizeof(buf), "Decay %.1fs", (double)v);
    displayShowTimbre(buf);
}

static void adjustDecay(bool up)
{
    float mul = up ? 1.3f : (1.0f / 1.3f);
    if (activeMode == MODE_SWARM) {
        swarmDecay = constrain(swarmDecay * mul, 0.2f, 8.0f);
        swarm.setDecay(swarmDecay);
    } else {
        stringsRel = constrain(stringsRel * mul, 0.15f, 6.0f);
        stringPad.setRelease(stringsRel);
    }
    showDecayParam();
}

static void loadMode(uint8_t index)
{
    if (index >= NUM_MODES) return;
    activeMode = index;
    storeMode(index);

    const bool fmMono = (index == MODE_FM);
    const bool ben    = (index == MODE_BENJOLIN || index == MODE_BENJOLIN_RAW);
    const bool crk    = (index == MODE_CRACKLE);
    const bool hang   = (index == MODE_HANG);
    const bool swm = (index == MODE_SWARM),   grn = (index == MODE_GRAIN);
    const bool str = (index == MODE_STRINGS), tan = (index == MODE_TANPURA);
    const bool bwl = (index == MODE_BOWLS),   flu = (index == MODE_FLUTE);
    const bool cel = (index == MODE_CELLO),   pch = (index == MODE_PHASE);
    const bool cat = (index == MODE_CATHEDRAL), frz = (index == MODE_FREEZE);
    // Normal voice path is used by Proximity, Arp, FM Poly (which self-FMs the
    // voices) AND Cathedral/Freeze (which post-process it). The engine modes
    // replace it entirely.
    const bool normalPath = !(fmMono || ben || crk || hang ||
                              swm || grn || str || tan || bwl || flu || cel || pch);

    finalMix.gain(0, normalPath ? 1.0f : 0.0f);     // normal voices (+ FM Poly, Cathedral, Freeze)
    finalMix.gain(1, fmMono ? FM_LEVEL       : 0.0f); // mono FM carrier
    finalMix.gain(2, ben    ? BENJOLIN_LEVEL : 0.0f); // Benjolin
    finalMix.gain(3, crk    ? CRACKLE_LEVEL  : 0.0f); // Cracklebox
    finalMix2.gain(1, hang  ? HANG_LEVEL     : 0.0f); // Hang Bow
    engMixA.gain(0, swm ? SWARM_LEVEL   : 0.0f);
    engMixA.gain(1, grn ? GRAIN_LEVEL   : 0.0f);
    engMixA.gain(2, str ? STRINGS_LEVEL : 0.0f);
    engMixA.gain(3, bwl ? BOWLS_LEVEL   : 0.0f);
    engMixB.gain(0, tan ? TANPURA_LEVEL : 0.0f);
    engMixB.gain(1, flu ? FLUTE_LEVEL   : 0.0f);
    engMixB.gain(2, cel ? CELLO_LEVEL   : 0.0f);
    engMixB.gain(3, pch ? PHASE_LEVEL   : 0.0f);
    fxMix.gain(1, cat ? CATHEDRAL_WET : 0.0f);      // reverb wet (Cathedral only)
    fxMix.gain(2, 0.0f);                            // frozen layer (driven per-frame)

    arpCurrentPad = 0xFF; arpNextTickMs = millis();
    if (!fmMono) fmDc.amplitude(0.0f, 8.0f); // hush the mono carrier when leaving FM
    if (!ben)    benjolin.setAmp(0.0f);      // hush the chaos when leaving Benjolin
    if (!crk)    crackle.setAmp(0.0f);
    // Drop every inactive engine's forces to zero — they ring out, then idle.
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (!hang) hangBow.setForce(i, 0.0f);
        if (!swm)  swarm.setForce(i, 0.0f);
        if (!grn)  grainHang.setForce(i, 0.0f);
        if (!str)  stringPad.setForce(i, 0.0f);
        if (!bwl)  bowls.setForce(i, 0.0f);
        if (!flu)  flutes.setForce(i, 0.0f);
        if (!cel)  cello.setForce(i, 0.0f);
        if (!pch)  phaseChimes.setForce(i, 0.0f);
    }
    if (!frz) { freezer.freeze(false); frzFrozen = false; frzEnv = 0.0f; }
    if (ben)     benjolin.setQuantize(index == MODE_BENJOLIN ? SCALE_SETS[activeScale].freqs : nullptr,
                                      index == MODE_BENJOLIN ? NUM_SENSORS : 0);

    // The timbre line shows the timbre where it applies, and doubles as the
    // Decay control (same ▲▼) in Swarm/Strings. Other engine modes hide it.
    displaySetTimbreUsed(normalPath || cat || frz || swm || str);
    if      (swm) { swarm.setDecay(swarmDecay);       showDecayParam(); }
    else if (str) { stringPad.setRelease(stringsRel); showDecayParam(); }
    else if (normalPath || cat || frz) displayShowTimbre(TIMBRE_SETS[activeTimbre].name);
    displayShowMode(MODE_NAMES[index]);
    Serial.print(F("# Mode: "));
    Serial.println(MODE_NAMES[index]);
}

// Harmonic Journey — retune all 12 pads to the selected chord of the active
// atlas with a smooth glide. The active play mode keeps running; only pitches
// shift. The screen's bottom label shows the live timbre (changeable with a
// left/right swipe); the emotional character of each move is documented in the
// config.h comments / HARMONIC_TRANSITIONS data rather than drawn on screen.
static void loadHarmonicChord(uint8_t chord)
{
    const HarmonicJourney &J = HARMONIC_JOURNEYS[activeJourney];
    if (chord >= J.nNodes) return;

    activeHarmonicChord = chord;
    storeHarmChord(chord);

    glideStartMs = millis();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        glideStart[i]    = glideFreq[i];
        harmonicFreqs[i] = J.chords[chord].freqs[i];
    }
    harmonicOverride = true;

    displaySetHarmonicChord(chord);
    Serial.print(F("# Harmonic: ")); Serial.print(J.name);
    Serial.print(' ');               Serial.println(J.chords[chord].name);
}

// Tonal Map — glide all 12 pads to a chord of the 48-node graph (tonalmap.h)
// and re-centre the on-screen map on it (the display animates the relayout).
static void loadTonalMapNode(uint8_t node, bool animate)
{
    if (node >= TONAL_NODES) return;

    activeTonalNode = node;
    storeTonalNode(node);

    glideStartMs = millis();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) glideStart[i] = glideFreq[i];
    tonalMapFreqs(node, harmonicFreqs);
    harmonicOverride = true;

    displaySetTonalMapCenter(node, animate);
    Serial.print(F("# Harmonic: Tonal Map "));
    Serial.println(TONAL_NAMES[node]);
}

// Select the active atlas (journey) and lay out its node network. Clamps the
// remembered chord to the new atlas's node count.
static void setActiveJourney(uint8_t j)
{
    if (j >= NUM_HARMONIC_JOURNEYS) return;
    activeJourney = j;
    storeJourney(j);
    if (activeHarmonicChord >= HARMONIC_JOURNEYS[j].nNodes) activeHarmonicChord = 0;
    displaySetHarmonicJourney(j);
}

// ─────────────────────────────────────────────────────────────────────────────
// Instrument callbacks
// ─────────────────────────────────────────────────────────────────────────────

// Called every frame for each pad.
// intensity 0.0 = hand far away, 1.0 = hand very close.
static void onProximity(uint8_t sensorIndex, float intensity)
{
    // Click-free amplitude via DC ramp (sample-accurate interpolation)
    float targetAmp = intensity * VOICE_MAX_AMP;
    setVoiceAmplitude(voices[sensorIndex], targetAmp);

    // Proximity-modulated filter cutoff (brighter when closer). Uses the live
    // (morphing) cutoff range so a timbre change sweeps smoothly.
    setVoiceFilter(voices[sensorIndex], intensity, curBaseHz, curMaxHz);
}

// Per-pad bell state.
static bool     bellHeld[NUM_BELLS]   = { false }; // sounding, awaiting lift → release
static uint32_t bellArmMs[NUM_BELLS]  = { 0 };     // peak-velocity window deadline
static float    bellPeakV[NUM_BELLS]  = { 0.0f };  // peak contact velocity so far
static float    bellPress[NUM_BELLS]  = { 0.0f };  // smoothed aftertouch pressure 0–1

// Per-board outlier check on the freshly committed baselines. A hand hovering
// over a single pad at lock time leaves that pad's baseline much lower than
// its neighbours' — afterwards the pad is "barely sensitive" because the
// baseline ≈ near-touch reading. Compares each sense pad's baseline against
// the per-board median and rewrites any that fall too far below it. Runs
// while the chip is in normal scan mode; the next baseline-tracking tick will
// continue from the rewritten value.
static void fixOutlierBaselines()
{
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        uint8_t n = SENSE_ELECTRODES[b];
        if (n == 0) continue;

        uint16_t bases[12];
        for (uint8_t e = 0; e < n; e++) bases[e] = boards[b].baselineData(e);

        // Insertion sort to find the median (n ≤ 6).
        uint16_t sorted[12];
        for (uint8_t e = 0; e < n; e++) sorted[e] = bases[e];
        for (uint8_t i = 1; i < n; i++)
        {
            uint16_t v = sorted[i];
            int j = (int)i - 1;
            while (j >= 0 && sorted[j] > v) { sorted[j + 1] = sorted[j]; j--; }
            sorted[j + 1] = v;
        }
        uint16_t median = sorted[n / 2];

        for (uint8_t e = 0; e < n; e++)
        {
            // Rewrite either-direction outliers to the per-board median:
            //   • baseline ≫ median  → pad rests with a positive rawDelta and
            //     plays a constant tone (the voice never closes) even though a
            //     real touch still spikes the jump detector. THIS is the case
            //     that makes a pad "always sound" while its bell still fires.
            //   • baseline ≪ median  → pad is "barely sensitive" (rawDelta
            //     clamps to 0 until a hard touch) — a hand hovering at lock time.
            int16_t diff = (int16_t)bases[e] - (int16_t)median;
            if (diff < 0) diff = (int16_t)-diff;
            if (diff > (int16_t)BASELINE_OUTLIER_DELTA)
            {
                // BASE register holds the upper 8 of the 10-bit baseline.
                boards[b].write((uint8_t)(MPR121Reg::BASE_0 + e),
                                (uint8_t)(median >> 2));
                Serial.print(F("#   outlier fix: board "));
                Serial.print(b);
                Serial.print(F(" ELE"));
                Serial.print(e);
                Serial.print(F(" base "));
                Serial.print(bases[e]);
                Serial.print(F(" -> median "));
                Serial.println(median);
            }
        }
    }
}

// Force a full MPR121 baseline reload (CL=11) on every board, release any
// sustained bells, and re-seed the proximity EMAs. ~60 ms of sensor pause;
// only called during a quiet period, so the existing 8 ms voice ramp +
// 400 ms bell release handle the fade.
static void recalibrateBaselines()
{
    for (uint8_t i = 0; i < NUM_BELLS; i++)
    {
        if (bellHeld[i])
        {
            bellEnv[i].noteOff();
            bellHeld[i] = false;
            if (MPE_ENABLE)
                usbMIDI.sendNoteOff(midiNote[i], 0, mpeChannelFor(i));
        }
        bellPress[i] = 0.0f;
    }

    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        boards[b].write(MPR121Reg::ECR, 0x00);
        delay(10);
        boards[b].write(MPR121Reg::ECR, (uint8_t)(0b11000000 | SENSE_ELECTRODES[b]));
        delay(50);
    }
    // NOTE: fixOutlierBaselines() is deliberately NOT called here. The full ECR
    // reload above already sets every pad's baseline ≈ its current filtered
    // value (raw ≈ 0). Pads on this build legitimately differ from the per-board
    // median (relocated ELE6, the abandoned noisy ELE0, mixed trace lengths), so
    // forcing them to the median rewrites a CLEAN baseline into an offset one —
    // the exact "phantom moves between pads on every recal" bug. The outlier fix
    // is only meaningful at startup (a finger may contaminate lockBaseline), so
    // it stays in setup() and is kept out of the steady-state recal loop.

    // Re-seed each pad's EMA from the fresh baseline, and print the per-pad raw
    // delta (baseline − filtered) so calibration can be debugged: right after a
    // clean recal every pad should read ~0. A pad left with a large +raw will
    // sit "warm" and phantom-trigger; a large −raw means it's been desensitised.
    Serial.print(F("# RECAL @")); Serial.print(millis()); Serial.print(F("ms raw:"));
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];
        if (sc.electrode == NO_PIN) { seedSensorState(sensorState[i], 0.0f);
                                      Serial.print(F(" -")); continue; }
        uint16_t filt = boards[sc.boardIndex].filteredData(sc.electrode);
        seedSensorState(sensorState[i], (float)filt); // filtered = new software baseline
        Serial.print(' '); Serial.print(i); Serial.print(':'); Serial.print(filt);
    }
    Serial.println();
}

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
    const ScaleSet &sc = SCALE_SETS[activeScale];
    float fund = sc.freqs[sensorIndex] * BELL_OCTAVES;

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
        midiNote[k] = midiNoteFromFreq(sc.freqs[sensorIndex]);
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
        delay(5);
    }
    for (uint8_t v = 15; v > 0; v--)
    {
        memset(bri, v, sizeof(bri));
        for (uint8_t b = 0; b < NUM_BOARDS; b++)
            boards[b].setLEDs8(bri);
        delay(5);
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
    Wire.setClock(100000); // 100 kHz — the MPR121 pad boards' long wiring can't sustain
                           // faster (200/400 kHz → corrupted reads → no intensity →
                           // silence). See the note in loop()'s burst read for the fix
                           // if you ever want higher (stronger I2C pull-ups / shorter wires).

    // ── MPR121 boards (staged init so we can read pads BEFORE baseline lock) ─
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (!boards[b].beginConfig(SENSE_ELECTRODES[b], 40, 20, SENSOR_CDC, SENSOR_CDT, SENSOR_FFI, SENSOR_ESI))
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
        // CL=00 = baseline frozen at 0 → raw filtered reads. Lets us see if a
        // finger is on a pad before we commit a baseline that would hide it.
        boards[b].startScanning(SENSE_ELECTRODES[b], 0b00);
        boards[b].beginLEDs();
    }

    // ── Pick the startup scale/timbre: pad-held overrides the stored scale ───
    delay(25); // let filtered data populate
    uint8_t startupScale  = loadStoredScale(STARTUP_SCALE_DEFAULT);   // last saved, else default
    uint8_t startupTimbre = loadStoredTimbre(STARTUP_TIMBRE_DEFAULT); // (timbre isn't pad-pickable)
    uint8_t startupMode   = loadStoredMode(MODE_PROX);                // play mode
    uint8_t heldPad       = 0xFF; // 0xFF = none
    {
        const SensorConfig &refS = SENSORS[STARTUP_REF_PAD];
        uint16_t refF = boards[refS.boardIndex].filteredData(refS.electrode);

        // Finger lowers the filtered value sharply. List order = priority.
        for (uint8_t bi = 0; bi < NUM_STARTUP_BINDINGS; bi++)
        {
            const StartupBinding &b = STARTUP_BINDINGS[bi];
            const SensorConfig   &s = SENSORS[b.pad];
            uint16_t padF = boards[s.boardIndex].filteredData(s.electrode);
            if ((int16_t)refF - (int16_t)padF > STARTUP_HOLD_THRESHOLD)
            {
                startupScale = b.scale;
                heldPad      = b.pad;
                break;
            }
        }
        if (heldPad != 0xFF)
        {
            Serial.print(F("# hold detected on pad ")); Serial.print(heldPad);
            Serial.print(F(" → scale "));               Serial.println(startupScale);
        }
    }

    // ── LFO phases — spread evenly, rates slightly different per voice ───────
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        lfoPhase[i] = (float)i / (float)NUM_SENSORS;
        lfoRate[i]  = LFO_RATE_HZ + LFO_RATE_SPREAD * ((float)i / (float)NUM_SENSORS - 0.5f);
    }

    // ── Teensy Audio ─────────────────────────────────────────────────────────
    AudioMemory(230); // 12 voices (+FM mod osc) + 12 bells + FM bus + all engines + fx

    // Chorus send — starts dry (voices(1) = pass-through); loadTimbre() bumps
    // the voice count for chorused timbres (Juno).
    chorus.begin(chorusDelayLine, CHORUS_DELAY_LEN, 1);

    // FM carrier + modulator bus (silent until FM mode drives them).
    fmCarrier.begin(WAVEFORM_SINE);
    fmCarrier.amplitude(1.0f);
    fmCarrier.frequencyModulation(FM_DEPTH_OCTAVES);
    for (uint8_t m = 0; m < 3; m++)
        for (uint8_t ch = 0; ch < 4; ch++) fmModMix[m].gain(ch, 0.0f);
    fmModSum.gain(0, 1.0f); fmModSum.gain(1, 1.0f); fmModSum.gain(2, 1.0f); fmModSum.gain(3, 0.0f);
    fmDc.amplitude(0.0f);

    // Benjolin / Cracklebox defaults (silent until their mode gates them).
    benjolin.setCross(BENJOLIN_CROSS);
    benjolin.setRungle(BENJOLIN_RUNGLE);
    benjolin.setAmp(0.0f);
    crackle.setAmp(0.0f);

    // Hang Bow / Bowls defaults: seed every resonator with its pad's note (silent
    // until their modes gate them; forces are 0 so nothing is bowed yet).
    hangBow.setLevel(1.0f);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        float f0 = SCALE_SETS[startupScale].freqs[i]; // current scale's note per pad
        hangBow.setFreq(i, f0);
        hangBow.setForce(i, 0.0f);
        bowls.setFreq(i, f0);
    }
    hangBow.recalcCoupling();
    bowls.recalcCoupling();

    // Cathedral / Freeze effects (silent until their modes raise the fx gains).
    freeverb.roomsize(CATHEDRAL_ROOMSIZE);
    freeverb.damping(CATHEDRAL_DAMPING);

    // Final bus — start on the normal voice path (Proximity mode).
    finalMix.gain(0, 1.0f); finalMix.gain(1, 0.0f); finalMix.gain(2, 0.0f); finalMix.gain(3, 0.0f);
    finalMix2.gain(0, 1.0f); finalMix2.gain(1, 0.0f); // ch1 (Hang Bow) raised by its mode
    finalMix2.gain(2, 1.0f); finalMix2.gain(3, 1.0f); // engine sub-mixers (levels set there)
    for (uint8_t ch = 0; ch < 4; ch++) { engMixA.gain(ch, 0.0f); engMixB.gain(ch, 0.0f); }
    fxMix.gain(0, 1.0f); fxMix.gain(1, 0.0f); fxMix.gain(2, 0.0f); fxMix.gain(3, 0.0f);
    masterVol.gain(MASTER_VOLUME);

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
    // Output polarity (config.h OUTPUT_BALANCED): -1 = differential/balanced
    // pair, +1 = normal unbalanced stereo. Unbalanced is the default — the
    // inverted right channel cancels (≈silence) when summed on ordinary gear.
    rInv.gain(OUTPUT_BALANCED ? -1.0f : 1.0f);

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
    proxCfg.proxReleaseDelta  = PROX_RELEASE_DELTA;
    proxCfg.smoothAlpha       = PROX_SMOOTH_ALPHA;
    proxCfg.smoothAlphaRise   = PROX_SMOOTH_RISE;
    proxCfg.smoothFallRef     = PROX_SMOOTH_FALL_REF;
    proxCfg.fastJumpRef       = PROX_FAST_JUMP_REF;
    proxCfg.fastAlphaRise     = PROX_ATTACK_ALPHA;
    proxCfg.fallJumpRef       = PROX_FALL_JUMP_REF;
    proxCfg.baseIdleAlpha     = PROX_BASE_IDLE_ALPHA;
    proxCfg.baseNegAlpha      = PROX_BASE_NEG_ALPHA;
    proxCfg.healAlpha         = PROX_HEAL_ALPHA;
    proxCfg.holdMaxFrames     = PROX_HOLD_MAX_FRAMES;
    proxCfg.lowHoldBand       = PROX_LOW_HOLD_BAND;
    proxCfg.lowHoldFrames     = PROX_LOW_HOLD_FRAMES;

    // ── Teensy-GPIO LEDs ─────────────────────────────────────────────────────
    // LEDs offloaded from the MPR121's faulty ELE9/ELE10 drivers (pads 3,4,9,10)
    // are wired to Teensy pins. Set them up as PWM outputs at an inaudible
    // frequency so they neither whine nor couple into the audio.
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];
        if (sc.ledBoard == LED_TEENSY && sc.ledEle != NO_PIN)
        {
            pinMode(sc.ledEle, OUTPUT);
            analogWriteFrequency(sc.ledEle, 30000); // 30 kHz — inaudible
            analogWrite(sc.ledEle, 0);
        }
    }

    // ── LCD / touch screen ───────────────────────────────────────────────────
    displayInit(); // no-op when ENABLE_SCREEN == 0

    // ── Startup animation / hold-confirmation sequence ──────────────────────
    if (heldPad == 0xFF)
    {
        // Normal boot — quick LED ramp is also the "settle" window.
        playStartupAnimation();
    }
    else
    {
        // Hold-override boot — keep ALL LEDs solid while the user is pressing
        // (visual confirmation the override took), fade them after release,
        // then wait 1 s before locking the baseline so the chip sees the
        // fully-released state.
        const SensorConfig &refS  = SENSORS[STARTUP_REF_PAD];
        const SensorConfig &heldS = SENSORS[heldPad];

        uint8_t fullBri[8];
        memset(fullBri, 15, sizeof(fullBri));
        uint32_t holdStart = millis();
        while (true)
        {
            for (uint8_t b = 0; b < NUM_BOARDS; b++) boards[b].setLEDs8(fullBri);
            delay(20);
            uint16_t refF = boards[refS.boardIndex ].filteredData(refS.electrode);
            uint16_t padF = boards[heldS.boardIndex].filteredData(heldS.electrode);
            // Hysteresis on release: drop below half-threshold = released.
            if ((int16_t)refF - (int16_t)padF < STARTUP_HOLD_THRESHOLD / 2) break;
            // Safety: don't hang forever (30 s max).
            if (millis() - holdStart > 30000) break;
        }

        // Fade LEDs down.
        uint8_t bri[8];
        for (int v = 15; v >= 0; v--)
        {
            memset(bri, (uint8_t)v, sizeof(bri));
            for (uint8_t b = 0; b < NUM_BOARDS; b++) boards[b].setLEDs8(bri);
            delay(25);
        }

        // 1 s pause so any residual finger capacitance settles.
        delay(1000);
    }

    // ── Per-electrode auto-config, then commit the baseline ──────────────────
    // autoConfig() arms the binary search; lockBaseline()'s Stop→Run transition
    // is what actually triggers it (and reloads the baseline). Nothing should be
    // touching now — the hold/settle sequence above guarantees that.
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (SENSOR_AUTOCONFIG) boards[b].autoConfig(SENSOR_VDD, SENSOR_FFI);
        boards[b].lockBaseline(SENSE_ELECTRODES[b]);
    }
    fixOutlierBaselines();

    // Report any electrodes auto-config could not tune (disconnected/abnormal).
    if (SENSOR_AUTOCONFIG)
        for (uint8_t b = 0; b < NUM_BOARDS; b++)
        {
            uint16_t oor = boards[b].autoConfigOOR();
            Serial.print(F("# autoconfig OOR board ")); Serial.print(b);
            Serial.print(F(" = 0x")); Serial.println(oor, HEX);
        }

    // ── Seed per-sensor EMA state from the freshly locked baseline ──────────
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];
        if (sc.electrode == NO_PIN) { seedSensorState(sensorState[i], 0.0f); continue; }
        uint16_t filt = boards[sc.boardIndex].filteredData(sc.electrode);
        seedSensorState(sensorState[i], (float)filt); // filtered = new software baseline
    }

    // ── Load chosen scale + timbre (scale may be overridden by a held pad) ───
    displayBeginCanvas();                // no-op when ENABLE_SCREEN == 0
    loadScale(startupScale, false);      // snaps glideFreq[] to the scale's notes
    loadTimbre(startupTimbre, false);    // full voice init using those freqs
    loadMode(startupMode);               // restore the saved play mode

    // Restore the screen state (normal / harmonic journey / locked) and atlas.
    screenState         = loadStoredScreen(0);
    activeJourney       = loadStoredJourney(0);
    activeHarmonicChord = loadStoredHarmChord(0);
    activeTonalNode     = loadStoredTonalNode(0);
    setActiveJourney(activeJourney); // clamp chord + lay out the node network
    displaySetLocked(screenState == 2);
    displaySetHarmonicMode(screenState == 1);
    if (screenState == 1) {
        // Resume straight into the remembered atlas's map (skip the picker) with
        // the last-selected chord active. Harmonic journey always plays in
        // Proximity; remember the restored mode for when the user leaves.
        modeBeforeHarmonic = activeMode;
        if (activeMode != MODE_PROX) loadMode(MODE_PROX);
        harmonicPicking = false;
        displaySetHarmonicPicking(false);
        if (activeJourney == JOURNEY_TONAL_MAP) {
            tonalMapFreqs(activeTonalNode, harmonicFreqs);
            harmonicOverride = true;
            displaySetTonalMapCenter(activeTonalNode, false);
        } else {
            const HarmonicJourney &J = HARMONIC_JOURNEYS[activeJourney];
            for (uint8_t i = 0; i < NUM_SENSORS; i++)
                harmonicFreqs[i] = J.chords[activeHarmonicChord].freqs[i];
            harmonicOverride = true;
            displaySetHarmonicChord(activeHarmonicChord);
        }
    }

    // ── MPE Configuration Message ────────────────────────────────────────────
    // RPN 6 on the master channel sets up MPE Zone 1: 12 member channels
    // (pad 0 → ch 2, … pad 11 → ch 13).
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
    Serial.println(F("# Scale  (0-8, swipe L/R):  0=DKurd 1=Just 2=Overtones 3=Chromatic 4=Sad 5=Happy 6=Pentatonic 7=HarmMinor 8=Accordion"));
    Serial.println(F("# Timbre (a-i, swipe U/D):  a=Warm b=Crystal c=Edgy d=Dark e=Soft f=Shimmer g=Cry h=Juno i=Moog"));
    Serial.println(F("# Mode   (m, mode arrows):  Proximity / Arp x3 / FM / FM Poly / Benjolin x2 / Cracklebox / Hang Bow"));
    Serial.println(F("#                           Swarm / Grain Hang / Strings / Cathedral / Freeze / Tanpura / Bowls / Flute / Cello / Phase Chimes"));
    Serial.println(F("# Diag   (t/r):             t=Teleplot readout   r=raw filtered CSV dump (R,ms,f0..f11)"));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────

void loop()
{
    uint32_t now = millis();

    // ── Serial command: 0-6 = scale, a-g = timbre (both glide live) ──────────
    if (Serial.available())
    {
        char c = Serial.read();
        if (c >= '0' && c <= '8')      loadScale((uint8_t)(c - '0'), true);
        else if (c >= 'a' && c <= 'i') loadTimbre((uint8_t)(c - 'a'), true);
        else if (c >= 'A' && c <= 'I') loadTimbre((uint8_t)(c - 'A'), true);
        else if (c == 'm' || c == 'M') loadMode((uint8_t)((activeMode + 1) % NUM_MODES));
        else if (c == 't' || c == 'T') { teleplotOn = !teleplotOn;
            Serial.print(F("# Teleplot readout ")); Serial.println(teleplotOn ? F("ON") : F("OFF")); }
        else if (c == 'r' || c == 'R') { rawDumpOn = !rawDumpOn;
            Serial.print(F("# Raw filtered dump ")); Serial.println(rawDumpOn ? F("ON (R,ms,f0..f11)") : F("OFF")); }
    }

    // ── High-rate pitch glide (decoupled from the 8 ms sensor cadence) ───────
    // The portamento + LFO pitch update runs ~1 kHz (once per millis() tick) so
    // large retunes — e.g. a whole-chord change in Harmonic Journey — glide
    // smoothly instead of stepping in audible 8 ms jumps. The sensor + voice
    // logic below still runs every UPDATE_MS; this only maintains glideFreq[]
    // and the oscillator pitch, which the slower block then reads.
    static uint32_t lastPitchMs = 0;
    if (now != lastPitchMs)
    {
        float dtP = (lastPitchMs == 0) ? (UPDATE_MS / 1000.0f)
                                       : ((float)(now - lastPitchMs) / 1000.0f);
        lastPitchMs = now;

        float gT = (SET_GLIDE_MS > 0.0f)
                 ? ((float)(now - glideStartMs) / SET_GLIDE_MS) : 1.0f;
        if (gT > 1.0f) gT = 1.0f;

        const ScaleSet &scP = SCALE_SETS[activeScale];
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            if (SENSORS[i].electrode == NO_PIN) continue;
            float tgt = harmonicOverride ? harmonicFreqs[i] : scP.freqs[i];
            glideFreq[i] = (gT >= 1.0f) ? tgt
                         : glideStart[i] * powf(tgt / glideStart[i], gT);

            lfoPhase[i] += lfoRate[i] * dtP;
            if (lfoPhase[i] >= 1.0f) lfoPhase[i] -= 1.0f;

            float freq = glideFreq[i] * (1.0f + LFO_AMOUNT * sinf(2.0f * (float)M_PI * lfoPhase[i]));
            setVoiceFrequency(voices[i], freq);
        }
    }

    // ── Rate-limited sensor update ───────────────────────────────────────────
    if (now - lastUpdateMs < UPDATE_MS)
    {
        // Idle time before the next sensor frame → render extra screen bands
        // into the off-screen framebuffer. Gated so a band (≤~2.5 ms worst
        // case) always finishes well before the sensor frame is due: the pads'
        // 8 ms cadence is untouched, the screen just completes frames sooner.
        if (UPDATE_MS - (now - lastUpdateMs) >= 5)
            displayRenderStep();
        return;
    }
    lastUpdateMs = now;

    // ── Burst-read all boards ────────────────────────────────────────────────
    // ONLY the filtered registers are read (2 bytes/electrode). The chip's
    // baseline registers are deliberately not read — the delta is computed
    // against the engine's software baseline (see proximity_engine.h; the
    // register's 4-count quantization caused ghost flutter, and its falling
    // filter ate slow approaches). The touch-status register is skipped too:
    // nothing uses it (contact detection is the jump detector). Net frame cost
    // is now one burst per board, ~2.6 ms of the 6 ms frame at 100 kHz.
    struct BoardData {
        uint8_t  filt[24];
    };
    static BoardData bd[NUM_BOARDS];

    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        uint8_t n = SENSE_ELECTRODES[b];
        boards[b].burstRead(MPR121Reg::FILT_0L, bd[b].filt, (uint8_t)(n * 2));
    }

    // ── Per-sensor update ────────────────────────────────────────────────────
    // ledBri is indexed by GPIO bit (g → ELE g+4); g=2..7 = ELE6..ELE11 here.
    static uint8_t ledBri[NUM_BOARDS][8];
    memset(ledBri, 0, sizeof(ledBri));

    // ── Per-frame interpolation progress (shared by all voices) ──────────────
    // (Pitch glide + LFO now run in the high-rate tick above; only the timbre
    // morph is interpolated here, at the 8 ms voice cadence.)
    // Timbre morph: fixed-time linear, 0→1 over TIMBRE_MORPH_MS.
    float morphT = (TIMBRE_MORPH_MS > 0.0f)
                 ? ((float)(now - timbreMorphStartMs) / TIMBRE_MORPH_MS) : 1.0f;
    if (morphT > 1.0f) morphT = 1.0f;
    curSub    = morphStartSub  + morphT * (morphTgtSub  - morphStartSub);
    curBaseHz = morphStartBase + morphT * (morphTgtBase - morphStartBase);
    curMaxHz  = morphStartMax  + morphT * (morphTgtMax  - morphStartMax);
    curQ      = morphStartQ    + morphT * (morphTgtQ    - morphStartQ);

    // ── Mode flags ───────────────────────────────────────────────────────────
    const bool arpActive = (activeMode >= MODE_ARP_SLOW && activeMode <= MODE_ARP_FAST);
    const bool fmMono    = (activeMode == MODE_FM);
    const bool fmPoly    = (activeMode == MODE_FM_POLY);
    const bool benActive = (activeMode == MODE_BENJOLIN || activeMode == MODE_BENJOLIN_RAW);
    const bool crkActive = (activeMode == MODE_CRACKLE);
    const bool hangActive = (activeMode == MODE_HANG);
    const bool swmActive = (activeMode == MODE_SWARM);
    const bool grnActive = (activeMode == MODE_GRAIN);
    const bool strActive = (activeMode == MODE_STRINGS);
    const bool tanActive = (activeMode == MODE_TANPURA);
    const bool bwlActive = (activeMode == MODE_BOWLS);
    const bool fluActive = (activeMode == MODE_FLUTE);
    const bool celActive = (activeMode == MODE_CELLO);
    const bool pchActive = (activeMode == MODE_PHASE);
    const bool frzActive = (activeMode == MODE_FREEZE);
    // Normal voices are audible in Proximity, Arp, FM Poly, Cathedral and
    // Freeze; the engine modes replace the voice path entirely.
    const bool normalOut = !(fmMono || benActive || crkActive || hangActive ||
                             swmActive || grnActive || strActive || tanActive ||
                             bwlActive || fluActive || celActive || pchActive);

    // ── Arpeggiator step ─────────────────────────────────────────────────────
    // In an Arp mode, advance to the next HELD pad (ascending pad index,
    // wrapping) every MODE_ARP_PERIOD_MS. Held = last frame's intensity over the
    // threshold. arpCurrentPad gates which voice is allowed to sound below.
    if (arpActive)
    {
        if ((int32_t)(now - arpNextTickMs) >= 0)
        {
            uint8_t start = (arpCurrentPad == 0xFF) ? (uint8_t)(NUM_SENSORS - 1) : arpCurrentPad;
            uint8_t next  = 0xFF;
            for (uint8_t k = 1; k <= NUM_SENSORS; k++)
            {
                uint8_t p = (uint8_t)((start + k) % NUM_SENSORS);
                if (lastIntensity[p] > ARP_HOLD_INTENSITY) { next = p; break; }
            }
            arpCurrentPad = next;                       // 0xFF if nothing held
            arpNextTickMs = now + (uint32_t)MODE_ARP_PERIOD_MS[activeMode];
        }
    }
    else
    {
        arpCurrentPad = 0xFF;
    }

    bool anyActive = false;

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];

        // Disabled pad (e.g. dropped idx5): keep its voice silent, no LED.
        if (sc.electrode == NO_PIN) { onProximity(i, 0.0f); lastIntensity[i] = 0.0f; continue; }

        const BoardData &b = bd[sc.boardIndex];
        uint8_t e = sc.electrode;

        // Reconstruct the 10-bit filtered value from the burst buffer
        uint16_t filtered = (uint16_t)b.filt[2 * e]
                          | ((uint16_t)(b.filt[2 * e + 1] & 0x03) << 8);

        // Optional median-of-3 (SENSOR_MEDIAN3 in config.h — bench data showed
        // it costs a full frame of onset latency and FFI≥1 already leaves zero
        // single-sample spikes, so it ships OFF; the machinery stays for A/B).
        uint16_t fMed = filtered;
        if (SENSOR_MEDIAN3) {
            static uint16_t fPrev1[NUM_SENSORS] = { 0 }, fPrev2[NUM_SENSORS] = { 0 };
            if (fPrev1[i] == 0 && fPrev2[i] == 0) fPrev1[i] = fPrev2[i] = filtered; // first-frame seed
            uint16_t lo = min(filtered, fPrev1[i]), hi = max(filtered, fPrev1[i]);
            fMed = max(lo, min(hi, fPrev2[i]));
            fPrev2[i] = fPrev1[i];
            fPrev1[i] = filtered;
        }

        tpFilt[i] = fMed; // for the Teleplot readout

        // The engine takes the raw filtered value directly — the delta is
        // formed inside against the per-pad software baseline (state.base).
        float intensity;
        bool  isTouch;
        updateProximity((float)fMed, now, proxCfg, sensorState[i], intensity, isTouch);

        // Teleplot shows the drift-corrected delta the engine actually runs on.
        float effDelta = sensorState[i].base - (float)fMed;
        tpDelta[i] = effDelta < 0.0f ? 0.0f : effDelta;

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
        // Voice amplitude gating by mode:
        //  • arp → only the current step's pad sounds
        //  • FM / Benjolin → normal voices muted (their oscillators still run and
        //    feed the FM modulator bus; pitches feed the Benjolin)
        float ampInt = intensity;
        if (arpActive && i != arpCurrentPad) ampInt = 0.0f;
        if (!normalOut)                      ampInt = 0.0f;
        onProximity(i, ampInt);
        lastIntensity[i] = intensity; // raw (held) value for arp/FM/benjolin

        // Press-order tracking (FM carrier = first held; Benjolin osc pitches).
        if (intensity > ARP_HOLD_INTENSITY) { if (holdStartMs[i] == 0) holdStartMs[i] = now; }
        else                                holdStartMs[i] = 0;

        // ── Timbre morph — apply the live sub/resonance to this voice ────────
        // (Pitch — portamento + LFO — is maintained in the high-rate tick above
        // so big retunes glide smoothly; here we only morph the timbre.)
        setVoiceMorph(voices[i], curSub, curQ);

        // FM Poly: each voice self-FMs, depth ∝ its proximity. Off otherwise.
        if (fmPoly) setVoiceFM(voices[i], FM_POLY_DEPTH, intensity * FM_POLY_AMT);
        else        setVoiceFM(voices[i], 0.0f, 0.0f);

        // ── LED brightness ───────────────────────────────────────────────────
        // Two LED paths:
        //   • LED_TEENSY → direct Teensy PWM pin (pads 3,4,9,10 — see config).
        //     8-bit analogWrite, 0–255.
        //   • MPR121 LED → packed into ledBri for the batch setLEDs8 below.
        //     4-bit driver, 0–15; GPIO bit = ledEle - 4 (ELE6→2 … ELE11→7).
        if (sc.ledEle != NO_PIN)
        {
            if (sc.ledBoard == LED_TEENSY)
            {
                analogWrite(sc.ledEle, (intensity < 0.001f) ? 0
                    : static_cast<uint8_t>(intensity * 255.0f + 0.5f));
            }
            else
            {
                ledBri[sc.ledBoard][sc.ledEle - 4] = (intensity < 0.001f) ? 0
                    : static_cast<uint8_t>(1.0f + intensity * 14.0f + 0.5f);
            }
        }
    }

    // Per-pad history for the engine modes: previous (gated) intensity for
    // approach-velocity & full-touch edges, plus the Strings dynamics memory.
    static float prevInt[NUM_SENSORS] = { 0 };
    static float strDyn [NUM_SENSORS] = { 0 };

    // ── Held pads, sorted by press order (for FM / Benjolin / Cracklebox) ────
    uint8_t order[NUM_SENSORS];
    uint8_t nHeld = 0;
    float   maxI  = 0.0f;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (lastIntensity[i] > maxI) maxI = lastIntensity[i];
        if (holdStartMs[i]) order[nHeld++] = i;
    }
    for (uint8_t a = 1; a < nHeld; a++) // insertion sort by hold-start time
    {
        uint8_t v = order[a]; int b = (int)a - 1;
        while (b >= 0 && holdStartMs[order[b]] > holdStartMs[v]) { order[b + 1] = order[b]; b--; }
        order[b + 1] = v;
    }

    // ── FM (mono): first-held = carrier, the rest modulate it ────────────────
    if (fmMono)
    {
        uint8_t carrier = (nHeld > 0) ? order[0] : 0xFF;
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            float g = (carrier != 0xFF && i != carrier && holdStartMs[i])
                    ? lastIntensity[i] * FM_MOD_SCALE : 0.0f;
            fmModMix[i / 4].gain(i % 4, g);
        }
        if (carrier != 0xFF)
        {
            fmCarrier.frequency(glideFreq[carrier]);
            fmDc.amplitude(lastIntensity[carrier] * VOICE_MAX_AMP, AMP_RAMP_MS);
        }
        else fmDc.amplitude(0.0f, AMP_RAMP_MS);
    }
    // ── Benjolin: each touch (in press order) layers in a parameter, and WHICH
    // pad you use biases it (pad index → 0..1). 1=oscA pitch+gate, 2=oscB pitch,
    // 3=cross-mod, 4=rungler, 5=tone. More touches → more going on. ────────────
    else if (benActive)
    {
        if (nHeld >= 1)
        {
            benjolin.setFreqA(glideFreq[order[0]]);
            float fB     = (nHeld >= 2) ? glideFreq[order[1]] : glideFreq[order[0]] * 0.75f;
            float cross  = BENJOLIN_CROSS;
            float rungle = BENJOLIN_RUNGLE;
            float cut    = 0.6f;
            auto bias = [&](uint8_t k){ return (float)order[k] / (float)(NUM_SENSORS - 1); }; // 0..1
            if (nHeld >= 3) cross  = 0.15f + lastIntensity[order[2]] * (0.3f + 0.6f * bias(2));
            if (nHeld >= 4) rungle = lastIntensity[order[3]] * (0.4f + 0.9f * bias(3));
            if (nHeld >= 5) cut    = 0.2f + lastIntensity[order[4]] * 0.8f * (0.4f + 0.6f * bias(4));
            benjolin.setFreqB(fB);
            benjolin.setCross(cross);
            benjolin.setRungle(rungle);
            benjolin.setCutoff(cut);
            benjolin.setAmp(maxI);
        }
        else benjolin.setAmp(0.0f);
    }
    // ── Cracklebox: one square oscillator per held pad (1 = clean square, each
    // extra pad XORs its tone in), two octaves down; proximity adds coupling. ─
    else if (crkActive)
    {
        uint8_t n = (nHeld > CRACKLE_MAX) ? CRACKLE_MAX : nHeld;
        for (uint8_t k = 0; k < n; k++)
            crackle.setBase(k, glideFreq[order[k]] * CRACKLE_OCT);
        crackle.setActive(n);
        crackle.setCouple(CRACKLE_COUPLE * maxI);
        crackle.setAmp(n > 0 ? (0.3f + maxI * 0.7f) : 0.0f);
    }
    // ── Hang Bow: every pad is a sympathetic resonator; proximity bows it. Pitch
    // is live (cheap per-frame); the coupling matrix is rebuilt only while notes
    // are still gliding to a new scale, then left alone. ────────────────────────
    else if (hangActive)
    {
        static float hangFreqSum = -1.0f;
        float sum = 0.0f;
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            hangBow.setFreq(i, glideFreq[i]);
            hangBow.setForce(i, lastIntensity[i] < INTENSITY_GATE ? 0.0f : lastIntensity[i]);
            sum += glideFreq[i];
        }
        if (fabsf(sum - hangFreqSum) > 0.05f) { hangBow.recalcCoupling(); hangFreqSum = sum; }
    }
    // ── Bowls: same shape as Hang Bow (modal + coupling matrix), bowl voicing ─
    else if (bwlActive)
    {
        static float bowlFreqSum = -1.0f;
        float sum = 0.0f;
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            bowls.setFreq(i, glideFreq[i]);
            bowls.setForce(i, lastIntensity[i] < INTENSITY_GATE ? 0.0f : lastIntensity[i]);
            sum += glideFreq[i];
        }
        if (fabsf(sum - bowlFreqSum) > 0.05f) { bowls.recalcCoupling(); bowlFreqSum = sum; }
    }
    // ── Swarm / Grain / Strings / Flute / Cello / Phase / Tanpura ────────────
    // All proximity-per-pad, with a noise gate (no ghost notes from baseline
    // wobble) and approach-velocity tracking: Strings turn it into dynamics,
    // Phase Chimes & Tanpura fire on the full-touch edge.
    else if (swmActive || grnActive || strActive || fluActive || celActive ||
             pchActive || tanActive)
    {
        const float dtS = (float)UPDATE_MS / 1000.0f;
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            float f   = glideFreq[i];
            float in  = (lastIntensity[i] < INTENSITY_GATE) ? 0.0f : lastIntensity[i];
            float vel = (in - prevInt[i]) / dtS;             // intensity per second
            bool  fullTouch = (in >= 0.97f && prevInt[i] < 0.97f);
            if      (swmActive) { swarm.setFreq(i, f);     swarm.setForce(i, in); }
            else if (grnActive) { grainHang.setFreq(i, f); grainHang.setForce(i, in); }
            else if (strActive)
            {
                // Approach velocity → dynamics: dive in fast for a full-voiced
                // accent, drift in slowly for a hushed swell.
                if (vel > 0.0f) { float a = vel / 6.0f; if (a > 1.0f) a = 1.0f;
                                  if (a > strDyn[i]) strDyn[i] = a; }
                if (in <= 0.0f) strDyn[i] *= 0.985f;          // forget between notes
                stringPad.setFreq(i, f);
                stringPad.setForce(i, in * (0.35f + 0.65f * strDyn[i]));
            }
            else if (fluActive) { flutes.setFreq(i, f); flutes.setForce(i, in); }
            else if (celActive) { cello.setFreq(i, f);  cello.setForce(i, in); }
            else if (pchActive)
            {
                if (fullTouch) {                              // mallet strikes NOW;
                    float a = vel / 8.0f;                     // approach speed sets
                    if (a > 1.0f) a = 1.0f; if (a < 0.0f) a = 0.0f; // the loop tempo
                    phaseChimes.trigger(i, 0.15f + 1.05f * (1.0f - a));
                }
                phaseChimes.setForce(i, in);                  // hold/release the layer
            }
            else // tanpura: pluck the string the moment the pad reaches full touch
            {
                if (fullTouch) {
                    float a = vel / 8.0f;
                    if (a > 1.0f) a = 1.0f; if (a < 0.15f) a = 0.15f;
                    tanpura.pluck(f, a);
                }
            }
        }
    }
    // ── Freeze: press hard to capture the sound; the layer fades after release ─
    else if (frzActive)
    {
        if (!frzFrozen && maxI > FREEZE_TRIGGER)
        {
            freezer.freeze(true);
            frzFrozen = true;
        }
        if (frzFrozen)
        {
            float tgt = (nHeld > 0) ? 1.0f : 0.0f;     // hold to sustain, release to fade
            frzEnv += (tgt > frzEnv ? 0.15f : 0.012f) * (tgt - frzEnv);
            fxMix.gain(2, FREEZE_LAYER * frzEnv);
            if (frzEnv < 0.01f && nHeld == 0) { freezer.freeze(false); frzFrozen = false; }
        }
    }

    // Gated intensities become last frame's reference for velocity/edge detection.
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
        prevInt[i] = (lastIntensity[i] < INTENSITY_GATE) ? 0.0f : lastIntensity[i];

    // ── Batch LED update ─────────────────────────────────────────────────────
    // Current wiring (see config.h SENSORS): the ELE9 LEDs (pads 3 & 9) are on
    // Teensy GPIO, so the chip's ELE9 driver bit is otherwise unused — and the
    // chip only lights an ELE10 LED (pads 4 & 10) when ELE9 is driven the same.
    // Mirror ELE9 (GPIO bit 5) ← ELE10 (GPIO bit 6) per board so ELE10 lights.
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        ledBri[b][5] = ledBri[b][6]; // ELE9 ← ELE10
        boards[b].setLEDs8(ledBri[b]);
    }

    // ── Screen data: background glow + per-pad pie (every frame, so the
    // visualiser is responsive) ──────────────────────────────────────────────
    {
        float sum = 0.0f;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) sum += lastIntensity[i];
        displaySetGlow(sum / (float)NUM_SENSORS);
        displaySetPads(lastIntensity);
    }

    // ── Touch screen (rate-limited; shares the I2C bus) ──────────────────────
    // Swipe or tap an arrow: L/R = scale, U/D = timbre. A medium press (release
    // ≥ MODE_PRESS_MIN_MS) cycles the play MODE; a long 10 s hold toggles the
    // locked visualiser. Press timing is done in firmware (off the raw finger),
    // so the 10 s hold can't also trip a mode change.
    static uint32_t lastGestureMs = 0;
    static uint32_t holdAccum     = 0;     // cumulative down-time (lock; tolerates gaps)
    static uint32_t lastTouchMs   = 0;     // last poll the finger was seen down
    static uint32_t lastPollMs    = 0;     // for the per-poll dt
    static bool     pressActed    = false; // lock already toggled for this hold
    static bool     fingerWasDown = false; // for edge-triggered harmonic taps
    if (now - lastGestureMs >= 30)
    {
        lastGestureMs = now;
        uint16_t tapX = 0, tapY = 0;
        DisplayGesture g = displayPollGesture(tapX, tapY); // NONE when screen off
        bool wasTap = (g == GESTURE_TAP);

        // Raw finger position (display space), read once per poll and shared by
        // the immediate harmonic-tap trigger below and the hold-tracking further
        // down. This is the touch chip's live position, not GESTURE_TAP — CLICK
        // is only reported a few hundred ms after release (the chip waits to
        // rule out a long-press/swipe first), which read as sluggish chord
        // selection in Harmonic Journey.
        uint16_t fx = 0, fy = 0;
        bool fingerDown     = displayPollTouchXY(fx, fy);
        bool fingerDownEdge = fingerDown && !fingerWasDown;

        if (screenState == 0) // normal screen: swipes/arrows change scale/timbre/mode
        {
            // A tap maps to whichever arrow box it landed in (harmonic mode
            // instead consumes the raw tap to pick atlases / chord nodes).
            if (wasTap) g = displayTapToGesture(tapX, tapY);
            switch (g)
            {
                case GESTURE_RIGHT: loadScale((uint8_t)((activeScale + 1) % NUM_SCALE_SETS), true); break;
                case GESTURE_LEFT:  loadScale((uint8_t)((activeScale + NUM_SCALE_SETS - 1) % NUM_SCALE_SETS), true); break;
                case GESTURE_DOWN:  // ▼: timbre+ — or shorter Decay in Swarm/Strings
                    if (activeMode == MODE_SWARM || activeMode == MODE_STRINGS) adjustDecay(false);
                    else loadTimbre((uint8_t)((activeTimbre + 1) % NUM_TIMBRE_SETS), true);
                    break;
                case GESTURE_UP:    // ▲: timbre- — or longer Decay in Swarm/Strings
                    if (activeMode == MODE_SWARM || activeMode == MODE_STRINGS) adjustDecay(true);
                    else loadTimbre((uint8_t)((activeTimbre + NUM_TIMBRE_SETS - 1) % NUM_TIMBRE_SETS), true);
                    break;
                case GESTURE_MODE_NEXT: loadMode((uint8_t)((activeMode + 1) % NUM_MODES)); break;
                case GESTURE_MODE_PREV: loadMode((uint8_t)((activeMode + NUM_MODES - 1) % NUM_MODES)); break;
                default: break;
            }
        }
        else if (screenState == 1) // harmonic journey
        {
            // Long-press while on a chord map → back to the atlas picker. (The
            // 5 s screen-state hold still cycles to the locked visualiser.)
            if (g == GESTURE_LONGPRESS && !harmonicPicking)
            {
                harmonicPicking = true;
                displaySetHarmonicPicking(true);
            }
            // Swipe left/right cycles the TIMBRE (the scale is overridden by the
            // chord nodes here, so left/right is free to do this instead).
            else if (g == GESTURE_RIGHT)
                loadTimbre((uint8_t)((activeTimbre + 1) % NUM_TIMBRE_SETS), true);
            else if (g == GESTURE_LEFT)
                loadTimbre((uint8_t)((activeTimbre + NUM_TIMBRE_SETS - 1) % NUM_TIMBRE_SETS), true);
            else if (fingerDownEdge) // act on touch-DOWN, not the chip's delayed CLICK
            {
                if (harmonicPicking)
                {
                    // Picker: touch an atlas name to open its chord map.
                    uint8_t j = displayTapToHarmonicJourney(fx, fy);
                    if (j != 0xFF)
                    {
                        setActiveJourney(j);
                        harmonicPicking = false;
                        displaySetHarmonicPicking(false);
                        if (j == JOURNEY_TONAL_MAP)
                            loadTonalMapNode(activeTonalNode, false); // snap layout
                        else
                            loadHarmonicChord(activeHarmonicChord); // apply pitches + label
                    }
                }
                else if (activeJourney == JOURNEY_TONAL_MAP)
                {
                    // Tonal Map: touch an option to glide there — that chord
                    // slides to the centre and its own options fan out.
                    uint8_t node = displayTapToTonalMapNode(fx, fy);
                    if (node != 0xFF) loadTonalMapNode(node, true);
                }
                else
                {
                    // Map: touch a chord node to retune the pads immediately
                    // (loadHarmonicChord still glides the pitch over SET_GLIDE_MS).
                    uint8_t chord = displayTapToHarmonicChord(fx, fy);
                    if (chord != 0xFF) loadHarmonicChord(chord);
                }
            }
        }

        // Lock hold: holdAccum is cumulative down-time, tolerating gaps up to
        // 600 ms (touch reads drop out at the round edges) so an edge dropout
        // doesn't reset the long hold. Mode is on the ‹ › arrows now, not a press.
        uint32_t dt  = (lastPollMs  == 0) ? 0 : (now - lastPollMs);
        uint32_t gap = (lastTouchMs == 0) ? 99999u : (now - lastTouchMs);
        lastPollMs = now;
        fingerWasDown = fingerDown;
        if (fingerDown) // finger down this poll
        {
            holdAccum += dt;
            lastTouchMs = now;
            if (!pressActed && holdAccum >= LOCK_HOLD_MS)
            {
                // Cycle: normal (0) → harmonic journey (1) → locked (2) → normal
                screenState = (uint8_t)((screenState + 1) % 3);
                bool inHarm = (screenState == 1);
                bool inLock = (screenState == 2);
                if (inHarm) {
                    // Entering harmonic mode always opens the atlas picker first;
                    // pitches stay on the current scale until an atlas is chosen.
                    // Harmonic journey always plays in Proximity — remember the
                    // current mode and switch, restoring it again on exit.
                    modeBeforeHarmonic = activeMode;
                    if (activeMode != MODE_PROX) loadMode(MODE_PROX);
                    harmonicPicking = true;
                    setActiveJourney(activeJourney);   // lay out the (remembered) atlas
                    displaySetHarmonicPicking(true);
                    displaySetHarmonicMode(true);
                } else {
                    displaySetHarmonicMode(false);
                    harmonicPicking  = false;
                    harmonicOverride = false; // back to scale when leaving harmonic
                    if (activeMode != modeBeforeHarmonic) loadMode(modeBeforeHarmonic);
                }
                displaySetLocked(inLock);
                storeScreen(screenState);
                static const char* const sNames[] = { "normal", "harmonic", "locked" };
                Serial.print(F("# screen ")); Serial.println(sNames[screenState]);
                pressActed = true;
            }
        }
        else // finger up this poll
        {
            if (holdAccum != 0 && gap > 600) { holdAccum = 0; pressActed = false; } // hold ended
        }
    }

    // Advance the screen by one band per sensor frame (plus extra bands in the
    // loop's idle time, gated above). The completed frame flips to the panel in
    // a single background DMA — atomic on screen, and the audio I2S DMA was
    // never bothered by the SPI stream (I2C was the real culprit back when).
    displayRenderStep();

    // ── Tiny cpu/mem readout, bottom middle of the screen (1 Hz) ─────────────
    {
        static uint32_t lastStatsMs = 0;
        static char     lastStats[16] = "";
        if (now - lastStatsMs >= 1000)
        {
            lastStatsMs = now;
            char sb[16];
            snprintf(sb, sizeof(sb), "%d%% %d",
                     (int)(AudioProcessorUsage() + 0.5f), AudioMemoryUsage());
            if (strcmp(sb, lastStats) != 0) { strcpy(lastStats, sb); displayShowStats(sb); }
        }
    }

    // ── Teleplot readout (toggle with serial 't') ───────────────────────────
    // First the engine load (cpu %, mem blocks), then per pad: raw (MPR121
    // filtered), delta (baseline−filtered), int (final 0..1). Teleplot @115200.
    // ── Raw filtered dump (toggle with serial 'r') ───────────────────────────
    // One CSV line per sensor frame: "R,<ms>,f0,f1,…,f11" — the full 10-bit
    // filtered value per pad (NO baseline applied). Feed a capture of this to the
    // calibration redesign: idle noise, environmental drift, approach curve and
    // tap signatures all read straight off these numbers.
    if (rawDumpOn)
    {
        Serial.print(F("R,")); Serial.print(now);
        for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(','); Serial.print(tpFilt[i]); }
        Serial.println();
    }

    if (teleplotOn)
    {
        static uint32_t lastTpMs = 0;
        if (now - lastTpMs >= 40)
        {
            lastTpMs = now;
            Serial.print(F(">cpu:")); Serial.println(AudioProcessorUsage(), 1);
            Serial.print(F(">mem:")); Serial.println(AudioMemoryUsage());
            for (uint8_t i = 0; i < NUM_SENSORS; i++)
            {
                Serial.print(F(">p")); Serial.print(i); Serial.print(F("raw:"));   Serial.println(tpFilt[i]);
                Serial.print(F(">p")); Serial.print(i); Serial.print(F("delta:")); Serial.println(tpDelta[i], 1);
                Serial.print(F(">p")); Serial.print(i); Serial.print(F("int:"));   Serial.println(lastIntensity[i], 3);
            }
        }
    }

    // ── Idle baseline recalibration ──────────────────────────────────────────
    // Disabled by default (RECAL_ENABLE) — superseded by the software baseline,
    // and it would re-trigger auto-config every couple of seconds (each recal is
    // a Stop→Run transition). Kept behind the flag for A/B comparison.
    if (RECAL_ENABLE)
    {
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
    }

    // ── Diagnostic output (10 Hz) ────────────────────────────────────────────
    // Off by default — the constant USB traffic whines into the audio (see
    // SERIAL_DEBUG in config.h).
#if SERIAL_DEBUG
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
#endif
}
