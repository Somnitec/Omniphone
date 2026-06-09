// Teensy LC — 7-pad proximity synth with per-pad PWM LEDs + 3 WS2812 groups.
//
// Each of the 7 TSI touch pads drives (a) a sine voice whose level tracks
// proximity/press and (b) its own PWM indicator LED. Three addressable
// WS2812B LEDs each glow with the average proximity of the four pads nearest
// to them. Audio out via the LC's onboard 12-bit DAC on pin 26 (A12).
//
// ── Pin map (Teensy LC) ──────────────────────────────────────────────────────
//   Touch pads (TSI):  pad 0..6  -> pins 0, 1, 15, 16, 17, 18, 19
//   PWM LEDs:          pad 0..6  -> pins 3, 4, 6, 9, 10, 20, 22
//   WS2812 data:       pin 5     (WS2812Serial / DMA — audio-safe)
//   Audio out (DAC):   pin 26 (A12)  -> 100ohm -> 470uF(+ to DAC) -> jack/amp
//   WS2812 power:      VUSB (5V), common ground, ~330ohm on data
//
// Built to be flashed independently of the main firmware — see the
// `[env:lc_seven_voice]` block in platformio.ini.

#include <Audio.h>
#include <WS2812Serial.h>

// ── Diatonic note table (equal temperament, A4 = 440 Hz) ─────────────────────
namespace Note {
    static constexpr float C3  = 130.81f;
    static constexpr float E3  = 164.81f;
    static constexpr float G3  = 196.00f;
    static constexpr float B3  = 246.94f;
    static constexpr float C4  = 261.63f;
    static constexpr float E4  = 329.63f;
    static constexpr float G4  = 392.00f;
}

// ── Pad configuration ────────────────────────────────────────────────────────
// touchPin / pwmPin / freq are indexed by pad number 0..6. Pad 0 is the centre
// pad (it appears in all three WS2812 groups below). Rewire freely by editing
// these arrays — everything downstream follows the index.
static constexpr int   NVOICES = 7;
static constexpr int   touchPin[NVOICES] = {  19, 0,1,3,4,18,15 };
static constexpr int   pwmPin[NVOICES]   = {   20, 22, 6 , 9 , 10, 16 , 17 };
static constexpr float freq[NVOICES] = {
    Note::C3, Note::E3, Note::G3, Note::B3,   // Cmaj7
    Note::C4, Note::E4, Note::G4,             // upper octave
};

// ── Full-scale calibration (fraction of baseline) ────────────────────────────
// The touch-induced count change is ~proportional to the resting baseline (both
// scale with TSI gain), so a single FRACTION auto-tracks any NSCN/PS/REFCHRG
// retune — no recalibration when you change sensor settings. Volume hits 1.0 at
// baseline + baseline*TOUCH_FRACTION; firm plastic touch = full volume, bare
// metal spikes past and clamps at 1.0. Measure on the plotter: (touch−rest)/rest.
// Per-pad full-scale fraction (plastic-touch delta ÷ baseline), from the sweep:
//   pad   0     1     2     3     4     5      6
//   base  1987  2036  2094  2073  2040  2200   2394
//   Δ    ~160  ~135  ~124  ~133  ~135  noisy  ~50   (counts)
// Pads 0–4 are clean ~6–8%. Pad 6 (pin23) couples weakly (~2%) so it needs a
// small fraction to reach full. Pad 5 (pin19) is grounding-noisy (Δ≈noise) — no
// fraction fixes that; it needs the ground fix. Tune any entry to taste.
static constexpr float TOUCH_FRACTION[NVOICES] =
    { 0.065f, 0.065f, 0.065f, 0.065f, 0.065f, 0.065f, 0.065f };   // UNIFORM for the software/hardware test
static constexpr float REJECT_FRACTION = 0.20f;    // discard jumps > 20% of baseline (noise spikes)

// ── WS2812 groups ────────────────────────────────────────────────────────────
// Each addressable LED shows the average proximity of its four nearest pads.
static constexpr int    NLEDS = 3;
static constexpr int    LED_PIN = 5;
static constexpr uint8_t ledGroup[NLEDS][4] = {
    { 0, 1, 2, 3 },
    { 0, 3, 4, 5 },
    { 0, 5, 6, 1 },
};
// Group-average rise per LED tick (~33 ms) that maps to the FULL "fast" colour.
// Lower = fast colour triggers on a gentler approach; higher = only a quick jab.
static constexpr float SPEED_FULL_BLUE = 0.25f;

// LED colour endpoints (R,G,B 0..255), blended by approach speed and scaled by
// proximity. SLOW = resting/gentle-touch colour, FAST = quick-approach colour.
// Tune these freely — e.g. pink and purple instead of white and blue.
static constexpr uint8_t COLOR_SLOW[3] = { 255,  100,  200 };   // pink
static constexpr uint8_t COLOR_FAST[3] = { 150,   50, 255 };   // purple

byte ledDraw[NLEDS * 3];
DMAMEM byte ledDisp[NLEDS * 12];
WS2812Serial leds(NLEDS, ledDisp, ledDraw, LED_PIN, WS2812_GRB);  // WS2812B is GRB

// ── Per-sample interpolating gain ─────────────────────────────────────────────
// Ramps gain LINEARLY across every 128-sample block, so a gain change is a
// smooth slope, not a step at the block boundary — that block-boundary step is
// the amplitude "stair". Fixed-point; the M0+ multiply is single-cycle so 7 of
// these cost only a few % CPU. gain() is 0..1; set it from level[].
class AudioRampGain : public AudioStream {
public:
    AudioRampGain() : AudioStream(1, inputQueueArray), target(0), current(0) {}
    void gain(float g) {
        if (g < 0.0f) g = 0.0f; else if (g > 1.0f) g = 1.0f;
        target = (int32_t)(g * 65536.0f);
    }
    virtual void update(void) {
        audio_block_t *blk = receiveWritable();
        if (!blk) return;
        int32_t g = current;
        int32_t step = (target - current) / AUDIO_BLOCK_SAMPLES;
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            g += step;
            blk->data[i] = (int16_t)(((int32_t)blk->data[i] * g) >> 16);
        }
        current = target;
        transmit(blk);
        release(blk);
    }
private:
    audio_block_t *inputQueueArray[1];
    volatile int32_t target;
    int32_t current;
};

// ── Audio graph ──────────────────────────────────────────────────────────────
// sine (constant amplitude) → per-voice ramp gain → two stage mixers → master
// → DAC. Volume is the ramp gain (smooth), NOT the sine amplitude (which steps).
AudioSynthWaveformSine sine[NVOICES];
AudioRampGain          vgain[NVOICES];
AudioMixer4            stageMix[2];
AudioMixer4            masterMix;
AudioOutputAnalog      dac;

AudioConnection v0(sine[0], 0, vgain[0], 0);
AudioConnection v1(sine[1], 0, vgain[1], 0);
AudioConnection v2(sine[2], 0, vgain[2], 0);
AudioConnection v3(sine[3], 0, vgain[3], 0);
AudioConnection v4(sine[4], 0, vgain[4], 0);
AudioConnection v5(sine[5], 0, vgain[5], 0);
AudioConnection v6(sine[6], 0, vgain[6], 0);
AudioConnection c0(vgain[0], 0, stageMix[0], 0);
AudioConnection c1(vgain[1], 0, stageMix[0], 1);
AudioConnection c2(vgain[2], 0, stageMix[0], 2);
AudioConnection c3(vgain[3], 0, stageMix[0], 3);
AudioConnection c4(vgain[4], 0, stageMix[1], 0);
AudioConnection c5(vgain[5], 0, stageMix[1], 1);
AudioConnection c6(vgain[6], 0, stageMix[1], 2);
AudioConnection s0(stageMix[0], 0, masterMix, 0);
AudioConnection s1(stageMix[1], 0, masterMix, 1);
AudioConnection oD(masterMix, 0, dac, 0);

// ── Touch state ──────────────────────────────────────────────────────────────
int   baseline[NVOICES];
float level[NVOICES];        // smoothed output level per voice
float prevAvg[NLEDS];        // previous LED-group average (for approach speed)

// 7 voices × 0.12 ≈ 0.84 worst-case peak — headroom against in-phase clipping.
static constexpr float VOICE_AMP = 0.12f;

// Per-pad EMA smoothing (α at the 1 kHz control tick; higher = snappier). The
// per-sample gain ramp now removes clicks, so a smooth exponential glide here is
// ideal — it rounds the discrete sensor updates into a continuous volume contour
// (less "steppy") instead of the linear slew's straight-segment feel.
static constexpr float ATTACK[NVOICES]  = { 0.18f, 0.18f, 0.18f, 0.18f, 0.18f, 0.18f, 0.18f };
static constexpr float RELEASE[NVOICES] = { 0.08f, 0.08f, 0.08f, 0.08f, 0.08f, 0.08f, 0.08f };
static constexpr float GATE[NVOICES]    = { 0.10f,  0.10f,  0.10f,  0.10f,  0.10f,  0.10f,  0.10f };

// Per-pad enable. Pad 5 (pin19) false-triggers from grounding noise the stopgap
// can't tame — muted for now. Set its entry back to 1 once its wiring/ground is
// fixed (pin18 on the same side is clean, so suspect pin19's specific lead).
static constexpr bool PAD_ENABLED[NVOICES] = { 1, 1, 1, 1, 1, 1, 1 };

// Onboard LED (pin 13): fast blink during setup, slow blink while running.
static constexpr int   STATUS_LED    = 13;
// Temporal median window: median of the last N reads PER PAD over time. Rejects
// single-sample glitches for the cost of just ONE TSI read per loop — spatial
// oversampling (N reads every loop) is what made it slow. Odd values: 3, 5.
static constexpr int   MEDIAN_WIN = 5;   // catches spikes up to 2 samples wide with no lag

// Spike rejector: hold the last accepted value through any read that jumps more
// than baseline*REJECT_FRACTION (transient grounding noise), but after MAX_REJECT
// consecutive rejects accept anyway so a genuinely sustained change gets through.
static constexpr int   MAX_REJECT  = 8;

// 1 = stream raw TSI counts (tab-separated, Arduino Serial-Plotter friendly)
// for calibration; 0 = print the CPU/memory status line instead.
#define CALIBRATE 0

// 1 = drive the per-pad PWM LEDs. Set 0 to TEST whether PWM switching on pins
// physically adjacent to the touch pins (15↔16, 17↔18, 19↔20) is what couples
// noise into pads 0/5/6. If turning this off makes them clean → confirmed.
#define ENABLE_PWM 0

// ⚠ Teensy LC: AudioOutputAnalog commandeers FTM1 (pins 3 & 4) for its 44 kHz
// sample clock. So: do NOT use pins 3/4 for PWM, and do NOT call
// analogWriteFrequency() — touching an FTM can kill the audio clock (silence).
// PWM stays at the default frequency; coupling is handled by pin PLACEMENT.

// ── TSI backend ──────────────────────────────────────────────────────────────
// 0 = Teensy's stock touchRead() (REFCHRG4/EXTCHRG3/PS2/NSCN9).
// 1 = our own tsiRead() using the fields below. Same scan sequence as the core
// (see cores/teensy3/touch.c), just with knobs exposed for max range/resolution:
//   • higher NSCN  → more scans accumulated → bigger counts (more resolution)
//   • higher PS    → longer count window     → bigger counts (slower scan)
//   • lower EXTCHRG→ slower electrode osc     → more counts per ΔC (sensitivity)
// If the raw stream rails at 65535 (16-bit overflow), back off PS then NSCN.
#define USE_CUSTOM_TSI 1
// Scan time ≈ (NSCN+1) × 2^PS reference periods — big values mean high
// resolution but SLOW reads, which makes the loop crawl and the audio step.
// Keep their product modest; prefer raising NSCN over PS.
// A/B test result: noise was our aggressive EXTCHRG=1/DVOLT=1, NOT the ISR.
// Keep EXTCHRG=3/DVOLT=0 (robust), but raise NSCN for range — more scans = bigger
// counts AND fewer interrupts (slower scans → less audio jitter/dithering).
static constexpr int TSI_NSCN_FIELD    = 31;  // 32 scans — span/SNR + lowest ISR rate (9 = too few counts, flickers)
static constexpr int TSI_PS_FIELD      = 2;
static constexpr int TSI_EXTCHRG_FIELD = 3;   // keep robust (1 = max sensitivity AND max noise)
static constexpr int TSI_REFCHRG_FIELD = 4;
// ΔV hysteresis (0..3). Datasheet trick: the COUNT is independent of ΔV (it
// cancels in the electrode/reference frequency ratio) but the scan TIME ∝ ΔV.
// So raising this speeds up reads with NO recalibration: 0=1.03V (slowest),
// 1=0.59V (~1.7×), 2=0.34V (~3×), 3=0.20V (~5×). Higher = faster but noisier.
static constexpr int TSI_DVOLT_FIELD   = 0;   // was 1 (matches stock for the A/B test)

int  rawNow[NVOICES];                 // filtered count per pad (for the dump)
int  rawHist[NVOICES][MEDIAN_WIN];    // recent raw reads for the temporal median
int  lastGood[NVOICES];               // last accepted raw per pad (spike rejector)
int  rejectRun[NVOICES];              // consecutive rejections per pad
int  touchSpan[NVOICES];              // baseline*TOUCH_FRACTION  (full-scale count span)
int  rejThresh[NVOICES];              // baseline*REJECT_FRACTION (spike-reject threshold)
unsigned long loopCount = 0;          // loops since last status print (loop-rate diag)
bool blinkState = false;
elapsedMillis sinceLED;
elapsedMillis sinceStatus;
elapsedMillis sinceBlink;
elapsedMillis sinceRaw;
elapsedMicros  sinceCtrl;     // fixed-rate control tick (regular amplitude updates = less zipper)

// ── TSI background scanner ───────────────────────────────────────────────────
#if USE_CUSTOM_TSI
// Pin → TSI channel for Teensy LC (MKL26Z64), from the core's touch.c pin2tsi[].
static const uint8_t pin2tsiLC[] = {
      9, 10, 255,   2,   3, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255,  13,   0,   6,   8,   7,
    255, 255,  14,  15, 255, 255, 255
};

// Scan ORDER (pad indices). Interleaving physically-distant pads means a touched
// pad never lands right before its neighbour, so any residual carry-over hits an
// unrelated pad instead of the adjacent one. Must be a permutation of 0..NVOICES-1.
static constexpr uint8_t scanOrder[NVOICES] = { 0, 2, 5, 3, 6, 4, 1 };
// Discarded "settle" scans after each channel switch (flush the previous
// channel's charge from the shared front-end). Raise if a touch still bleeds.
static constexpr int     SETTLE_SCANS = 2;

volatile uint16_t rawScan[NVOICES];   // latest count per pad, written by the ISR
volatile uint8_t  scanPos = 0;        // index into scanOrder[]
volatile int      settleLeft = SETTLE_SCANS;
uint8_t           scanCh[NVOICES];    // TSI channel per pad (precomputed)

// End-of-scan interrupt. For each channel we discard SETTLE_SCANS priming reads
// (so the previous channel can't bleed in — the pad-to-pad crosstalk), then keep
// the next one. Scans follow scanOrder[] so neighbours aren't adjacent in time.
void tsi0_isr(void) {
    uint8_t  pad   = scanOrder[scanPos];
    uint16_t count = TSI0_DATA & 0xFFFF;              // read count BEFORE clearing EOSF
    TSI0_GENCS |= TSI_GENCS_EOSF;                     // clear end-of-scan flag (w1c)
    if (settleLeft > 0) {
        settleLeft--;                                 // discard this priming scan
    } else {
        rawScan[pad] = count;                         // clean read
        scanPos = (scanPos + 1) % NVOICES;            // advance to next pad in the order
        settleLeft = SETTLE_SCANS;
        pad = scanOrder[scanPos];
    }
    TSI0_DATA = TSI_DATA_TSICH(scanCh[pad]) | TSI_DATA_SWTS;
}

// Configure TSI for continuous, interrupt-driven scanning of all pads.
static void startTsiScanner() {
    SIM_SCGC5 |= SIM_SCGC5_TSI;
    for (int i = 0; i < NVOICES; i++) {
        *portConfigRegister(touchPin[i]) = PORT_PCR_MUX(0);
        scanCh[i] = pin2tsiLC[touchPin[i]];
    }
    TSI0_GENCS = TSI_GENCS_REFCHRG(TSI_REFCHRG_FIELD)
               | TSI_GENCS_EXTCHRG(TSI_EXTCHRG_FIELD)
               | TSI_GENCS_DVOLT(TSI_DVOLT_FIELD)
               | TSI_GENCS_PS(TSI_PS_FIELD)
               | TSI_GENCS_NSCN(TSI_NSCN_FIELD)
               | TSI_GENCS_TSIEN | TSI_GENCS_TSIIEN | TSI_GENCS_ESOR;
    NVIC_SET_PRIORITY(IRQ_TSI, 255);   // lowest priority — never preempt the audio update
    NVIC_ENABLE_IRQ(IRQ_TSI);
    scanPos = 0;
    settleLeft = SETTLE_SCANS;
    TSI0_DATA = TSI_DATA_TSICH(scanCh[scanOrder[0]]) | TSI_DATA_SWTS;   // kick off the chain
}
#endif

// Latest raw count for pad i, from whichever backend is active.
static inline int readPadIdx(int i) {
#if USE_CUSTOM_TSI
    return rawScan[i];                 // cached by the background ISR — never blocks
#else
    return touchRead(touchPin[i]);     // stock: blocking read each call
#endif
}

// Median of a small sample set (insertion sort — n is tiny). Unlike a mean,
// this fully rejects a single outlier sample (the glitch noise we're seeing).
static int medianOfN(int *a, int n) {
    for (int i = 1; i < n; i++) {
        int key = a[i], j = i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
    return a[n / 2];
}

// Smoothly fade all WS2812s (white) from one brightness to another over durMs.
static void fadeLeds(int from, int to, int durMs) {
    const int steps = 24;
    for (int s = 0; s <= steps; s++) {
        int v = from + (to - from) * s / steps;
        for (int g = 0; g < NLEDS; g++) leds.setPixel(g, v, v, v);
        leds.show();
        delay(durMs / steps);
    }
}

void setup() {
    Serial.begin(115200);
    AudioMemory(8);
    leds.begin();
    //pinMode(STATUS_LED, OUTPUT);

    /*
    // Colour-order check: should flash RED, then GREEN, then BLUE in turn.
    // If the order is wrong, change WS2812_GRB above to match what you see.
    const uint8_t probe[3][3] = { {120, 0, 0}, {0, 120, 0}, {0, 0, 120} };
    for (int c = 0; c < 3; c++) {
        for (int g = 0; g < NLEDS; g++) leds.setPixel(g, probe[c][0], probe[c][1], probe[c][2]);
        leds.show();
        delay(400);
    }
*/
    for (int i = 0; i < NVOICES; i++) {
        sine[i].frequency(freq[i]);
        sine[i].amplitude(VOICE_AMP);   // constant; volume is the smooth ramp gain
        vgain[i].gain(0.0f);
        pinMode(pwmPin[i], OUTPUT);
        analogWrite(pwmPin[i], 0);
    }

#if USE_CUSTOM_TSI
    startTsiScanner();        // begin continuous background scanning
    delay(100);               // let every pad get scanned a few times first
#endif

    // Fade the LEDs up while calibrating (a visible "booting" cue)...
    fadeLeds(0, 160, 600);

    // Sample the baseline from the (now background-scanned) counts.
    Serial.println("Calibrating baseline. Do NOT touch the pads...");
    delay(300);
    for (int i = 0; i < NVOICES; i++) {
        long sum = 0;
        for (int n = 0; n < 32; n++) { sum += readPadIdx(i); delay(5); }
        baseline[i]  = sum / 32;
        level[i]     = 0.0f;
        lastGood[i]  = baseline[i];
        rejectRun[i] = 0;
        touchSpan[i] = (int)(baseline[i] * TOUCH_FRACTION[i]);
        rejThresh[i] = (int)(baseline[i] * REJECT_FRACTION);
        for (int k = 0; k < MEDIAN_WIN; k++) rawHist[i][k] = baseline[i];
    }

    // ...then fade off — calibration done, ready to play.
    fadeLeds(160, 0, 600);
    Serial.println("Baseline set. Ready.");
}

void loop() {
    loopCount++;

    // ── Control tick: process pads + update audio at a STEADY 1 kHz ──────────
    // Doing this every (jittery) loop pass made amplitude steps land unevenly →
    // zipper. A fixed cadence makes the steps regular and quiet. Reads are free
    // (the ISR scans in hardware), so this only paces the math + amplitude write.
    if (sinceCtrl >= 1000) {
    sinceCtrl -= 1000;
    for (int i = 0; i < NVOICES; i++) {
        int raw = readPadIdx(i);

        // Spike rejector: hold the last good value through implausible jumps.
        int dev = raw - lastGood[i];
        if (dev < 0) dev = -dev;
        if (dev > rejThresh[i] && rejectRun[i] < MAX_REJECT) {
            raw = lastGood[i];
            rejectRun[i]++;
        } else {
            lastGood[i]  = raw;
            rejectRun[i] = 0;
        }

        for (int k = MEDIAN_WIN - 1; k > 0; k--) rawHist[i][k] = rawHist[i][k - 1];
        rawHist[i][0] = raw;
        int tmp[MEDIAN_WIN];
        for (int k = 0; k < MEDIAN_WIN; k++) tmp[k] = rawHist[i][k];
        int med = medianOfN(tmp, MEDIAN_WIN);
        rawNow[i] = med;

        // baseline + touchSpan maps to 1.0; a metal spike past that clamps here.
        float t = (med - baseline[i]) / (float)touchSpan[i];
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;     // plastic = full scale; metal can't push further
        if (t < GATE[i]) t = 0.0f;
        if (!PAD_ENABLED[i]) t = 0.0f;   // muted pads stay silent

        // EMA glide toward target (smooth exponential). Clicks are handled by the
        // per-sample gain ramp, so this just shapes a continuous volume contour.
        float a = (t > level[i]) ? ATTACK[i] : RELEASE[i];
        level[i] += (t - level[i]) * a;
        vgain[i].gain(level[i]);   // per-sample-interpolated → click-free
#if ENABLE_PWM
        analogWrite(pwmPin[i], (int)(level[i] * 255.0f));
#endif
    }
    }  // end 1 kHz control tick

    // ── WS2812 groups: average of four nearest pads, throttled to ~30 Hz ─────
    if (sinceLED >= 33) {
        sinceLED = 0;
        for (int g = 0; g < NLEDS; g++) {
            float avg = 0.0f;
            for (int k = 0; k < 4; k++) avg += level[ledGroup[g][k]];
            avg *= 0.25f;

            // Approach speed = how fast the group average rose since last tick.
            // Approach speed blends SLOW→FAST colour; brightness = proximity.
            float s = (avg - prevAvg[g]) / SPEED_FULL_BLUE;
            prevAvg[g] = avg;
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;

            int r  = (int)(avg * ((1.0f - s) * COLOR_SLOW[0] + s * COLOR_FAST[0]));
            int gr = (int)(avg * ((1.0f - s) * COLOR_SLOW[1] + s * COLOR_FAST[1]));
            int b  = (int)(avg * ((1.0f - s) * COLOR_SLOW[2] + s * COLOR_FAST[2]));
            leds.setPixel(g, r, gr, b);
        }
        leds.show();
    }
/*
    // ── Status LED: slow blink (~1 Hz) while running ─────────────────────────
    if (sinceBlink >= 500) {
        sinceBlink = 0;
        blinkState = !blinkState;
        digitalWrite(STATUS_LED, blinkState);
    }
*/
#if CALIBRATE
    // ── Per pad: raw count then the final smoothed level (0..1) that drives the
    //    sine amplitude and feeds the LED groups. Tab-separated: raw0 lvl0 raw1 lvl1 …
    if (sinceRaw >= 100) {
        sinceRaw = 0;
        for (int i = 0; i < NVOICES; i++) {
            Serial.print(rawNow[i]);   Serial.print('\t');
            Serial.print(level[i], 2); Serial.print('\t');
        }
        Serial.println();
    }
#else
    // ── Status (no printf %f — the LC can't format floats that way) ──────────
    if (sinceStatus >= 2000) {
        Serial.print("loop Hz: ");
        Serial.print(loopCount / 2);                 // loops/sec over the 2s window
        Serial.print("   CPU max %: ");
        Serial.print(AudioProcessorUsageMax(), 1);
        Serial.print("   audio blocks max: ");
        Serial.println(AudioMemoryUsageMax());
        loopCount   = 0;
        sinceStatus = 0;
    }
#endif
}
