/*
 * mpr121_instrument.ino  –  Teensy 4.0 + MPR121 @ 0x5C
 *
 * ELE0–ELE5  : proximity + touch inputs  (pins 8–13 on the IC)
 * ELE6–ELE11 : LEDs, brightness = proximity (GPIO2–GPIO7, bits 2–7)
 *
 * ── FLICKER FIX ──────────────────────────────────────────────────────────────
 * MPR121 freezes GPIO/PWM output during each electrode scan. Default ESI=16ms
 * means one large freeze per 8ms PWM period → alternating bright/dim → flicker.
 * ESI=2ms → 4 short evenly-spread pauses per PWM period → uniform dim reduction,
 * barely visible. CDT=2μs keeps each scan brief (~250μs total for 6 electrodes).
 *
 * ── PROXIMITY vs CONTACT SEPARATION ─────────────────────────────────────────
 * delta    = (baseline<<2) – filtered_data   (positive = hand is near)
 * fast EMA (α=0.25) follows hand movement → drives LED brightness + volume
 * slow EMA (α=0.02) tracks only slow drift → ambient reference
 * jump     = fast – slow  → stays small during gradual approach, spikes on
 *            sudden metal contact. This is the contact detector.
 *
 * ── INSTRUMENT MODEL ─────────────────────────────────────────────────────────
 * onProximity(ch, 0.0–1.0) → called every frame → modulate sine volume
 * onTouch(ch, velocity)    → called once on contact → trigger attack sound,
 *                            velocity = how fast hand was approaching
 *
 * ── SERIAL PLOTTER ───────────────────────────────────────────────────────────
 * Output: "s0:N s1:N ... j0:N j1:N ..." (Arduino IDE 2.x label:value format)
 * s = smoothed proximity (0–PROX_MAX scale)
 * j = jump detector (fast–slow, spikes on contact)
 */

#include <Arduino.h>
#include <Wire.h>

// ── I²C ───────────────────────────────────────────────────────────────────────
static constexpr uint8_t MPR_ADDR = 0x5C;

// ── Registers ─────────────────────────────────────────────────────────────────
static constexpr uint8_t
  REG_TOUCH_L = 0x00,
  REG_FILT_0L = 0x04,    // ELE0 filtered LSB; ELEi → 0x04+2i, 0x05+2i
  REG_BASE_0 = 0x1E,     // ELE0 baseline (8 MSBs of 10-bit); ELEi → 0x1E+i
  REG_TOUCH_TH0 = 0x41,  // ELE0 touch threshold; ELEi → 0x41+2i
  REG_REL_TH0 = 0x42,    // ELE0 release threshold
  REG_CDC_CFG = 0x5C,    // FFI[1:0] | CDC[5:0]
  REG_CDT_CFG = 0x5D,    // CDT[2:0] | SFI[1:0] | ESI[2:0]
  REG_ECR = 0x5E,
  REG_SRST = 0x80,
  REG_GPIOCTL0 = 0x73,
  REG_GPIOCTL1 = 0x74,
  REG_GPIODIR = 0x76,
  REG_GPIOEN = 0x77,
  REG_GPIOSET = 0x78,
  REG_GPIOCLR = 0x79,
  // PWM register layout (AN3894 Table 2):
  REG_PWM0 = 0x81,  // GPIO0 [3:0], GPIO1 [7:4]  — not our LEDs
  REG_PWM1 = 0x82,  // GPIO2 [3:0], GPIO3 [7:4]  ← LED 0, 1
  REG_PWM2 = 0x83,  // GPIO4 [3:0], GPIO5 [7:4]  ← LED 2, 3
  REG_PWM3 = 0x84;  // GPIO6 [3:0], GPIO7 [7:4]  ← LED 4, 5

// ── Tuning — adjust these for your physical pads ──────────────────────────────
// Proximity thresholds
static constexpr float PROX_DEADBAND = 2.5f;  // below this delta, LED stays off
static constexpr float PROX_MAX = 18.0f;      // delta that gives full LED brightness
                                              // start with 80, lower = more sensitive

// MPR121 hardware touch thresholds (written to chip)
// Set relatively low so proximity registers early.
// The JUMP detector (not these) separates contact from approach.
static constexpr uint8_t TOUCH_TH = 40;
static constexpr uint8_t RELEASE_TH = 20;

// Contact (jump) detection
// jump = fast_ema – slow_ema, spikes on sudden metal contact
static constexpr float JUMP_THRESHOLD = 40.0f;      // tune: lower = triggers earlier
static constexpr uint32_t TOUCH_COOLDOWN_MS = 400;  // min ms between touch events per ch

// EMA smoothing factors
static constexpr float FAST_ALPHA = 0.15f;  // hand tracking speed (larger = snappier)
static constexpr float SLOW_ALPHA = 0.01f;  // drift reference (leave small)
static constexpr float VEL_ALPHA = 0.30f;   // velocity (d(fast)/dt) smoothing

// ── LED / GPIO constants ──────────────────────────────────────────────────────
static constexpr uint8_t LED_COUNT = 6;
static constexpr uint8_t LED_MASK = 0xFC;  // ELE6–ELE11 = GPIO2–GPIO7 = bits 2–7
static inline uint8_t ledBit(uint8_t i) {
  return 1u << (i + 2);
}

// ── Per-electrode state ───────────────────────────────────────────────────────
struct Electrode {
  float fast;      // fast EMA of rawDelta → LED + volume
  float slow;      // slow EMA           → ambient drift reference
  float velocity;  // smoothed d(fast)/dt → approach speed
  float prevFast;
  bool hwTouch;  // hardware touch flag from MPR121
  bool prevHwTouch;
  uint32_t lastTouchMs;  // timestamp of last onTouch() call (for cooldown)
};
static Electrode el[6] = {};

// ── I²C helpers ───────────────────────────────────────────────────────────────
static void mprWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPR_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
static uint8_t mprRead(uint8_t reg) {
  Wire.beginTransmission(MPR_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPR_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}
// Burst read n bytes from reg (MPR121 auto-increments address)
static void mprBurst(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPR_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPR_ADDR, n);
  for (uint8_t i = 0; i < n; i++)
    buf[i] = Wire.available() ? Wire.read() : 0;
}

// ── PWM update ────────────────────────────────────────────────────────────────
// Writes REG_PWM1/2/3 and GPIOSET/CLR from el[].fast values.
// LED i uses GPIO (i+2). Even gpioIdx → low nibble, odd → high nibble.
// pwm[0]=REG_PWM1(GPIO2,3), pwm[1]=REG_PWM2(GPIO4,5), pwm[2]=REG_PWM3(GPIO6,7)
static void updateLEDs() {
  uint8_t setMask = 0, clrMask = 0;
  uint8_t pwm[3] = {};

  for (uint8_t i = 0; i < LED_COUNT; i++) {
    float norm = (el[i].fast - PROX_DEADBAND) / (PROX_MAX - PROX_DEADBAND);
    norm = constrain(norm, 0.0f, 1.0f);
    uint8_t bri = (norm < 0.001f) ? 0 : (uint8_t)(1.0f + norm * 14.0f + 0.5f);  // 0 or 1–15

    uint8_t gpioIdx = i + 2;
    uint8_t regIdx = (gpioIdx - 2) / 2;  // 0,0,1,1,2,2
    bool hiNib = (gpioIdx & 1);          // false,true,false,true,false,true

    if (bri == 0) {
      clrMask |= ledBit(i);
      // nibble stays 0 in pwm[] — DAT=0 makes PWM irrelevant anyway
    } else {
      setMask |= ledBit(i);
      if (hiNib) pwm[regIdx] |= (bri & 0x0F) << 4;
      else pwm[regIdx] |= (bri & 0x0F);
    }
  }

  if (setMask) mprWrite(REG_GPIOSET, setMask);
  if (clrMask) mprWrite(REG_GPIOCLR, clrMask);
  mprWrite(REG_PWM1, pwm[0]);
  mprWrite(REG_PWM2, pwm[1]);
  mprWrite(REG_PWM3, pwm[2]);
}

// ── Instrument callbacks ──────────────────────────────────────────────────────
// ch = 0–5, intensity = 0.0 (far) → 1.0 (very close)
// Called every frame — modulate sinewave amplitude here.
void onProximity(uint8_t ch, float intensity) {
  (void)ch;
  (void)intensity;
  // TODO: oscillator[ch].setAmplitude(intensity);
}

// ch = 0–5, velocity = approach speed at contact (arbitrary units, larger = faster)
// Called once per contact event — trigger attack / percussive sound here.
void onTouch(uint8_t ch, float velocity) {
  // Will also show up in serial output below, but kept as a separate hook
  // for when audio code is added.
  (void)ch;
  (void)velocity;
  // TODO: sampler[ch].trigger(velocity);
}

// ── Setup LED fade animation ──────────────────────────────────────────────────
static void setLedPWMNibble(uint8_t i, uint8_t bri) {
  // Write just the nibble for LED i without touching the other nibble
  uint8_t gpioIdx = i + 2;
  uint8_t regAddr = REG_PWM1 + (gpioIdx - 2) / 2;
  bool hiNib = (gpioIdx & 1);
  uint8_t cur = mprRead(regAddr);
  if (hiNib) cur = (cur & 0x0F) | ((bri & 0x0F) << 4);
  else cur = (cur & 0xF0) | (bri & 0x0F);
  mprWrite(regAddr, cur);
}

static void ledFadeSetup() {
  // Sequential: each LED fades 1→15→1, then off
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    mprWrite(REG_GPIOSET, ledBit(i));
    for (uint8_t b = 1; b <= 15; b++) {
      setLedPWMNibble(i, b);
      delay(20);
    }
    for (uint8_t b = 14; b >= 1; b--) {
      setLedPWMNibble(i, b);
      delay(20);
    }
    mprWrite(REG_GPIOCLR, ledBit(i));
    delay(30);
  }

  // All together: uniform fade up then down
  for (uint8_t b = 1; b <= 15; b++) {
    uint8_t r = b | (b << 4);
    mprWrite(REG_PWM1, r);
    mprWrite(REG_PWM2, r);
    mprWrite(REG_PWM3, r);
    mprWrite(REG_GPIOSET, LED_MASK);
    delay(30);
  }
  for (uint8_t b = 14; b >= 1; b--) {
    uint8_t r = b | (b << 4);
    mprWrite(REG_PWM1, r);
    mprWrite(REG_PWM2, r);
    mprWrite(REG_PWM3, r);
    delay(30);
  }
  mprWrite(REG_GPIOCLR, LED_MASK);
  delay(150);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin();
  Wire.setClock(400000);  // 400kHz; ensure 4.7kΩ pull-ups and short wires

  // ① Soft reset
  mprWrite(REG_SRST, 0x63);
  delay(100);

  // ② Stop mode
  mprWrite(REG_ECR, 0x00);

  // ③ Timing registers — this is the main flicker fix
  // FFI back to 6 samples (was 34) — scan time drops from 816μs to 144μs
  // PWM freeze fraction: 4 × 144μs / 8ms = 7% (invisible) vs 82% before
  mprWrite(REG_CDC_CFG, 0b00001010);  // FFI=00 (6 samples), CDC=10μA

  // ESI back to 2ms, SFI=4 — was ESI=1ms with SFI=18
  // The software EMA replaces the hardware oversampling
  mprWrite(REG_CDT_CFG, 0b01100001);  // CDT=2μs, SFI=4, ESI=2ms

  for (uint8_t i = 0; i < 6; i++) {
    mprWrite(REG_TOUCH_TH0 + 2 * i, TOUCH_TH);
    mprWrite(REG_REL_TH0 + 2 * i, RELEASE_TH);
  }

  // ⑤ GPIO: ELE6–ELE11 as high-side open-drain LED outputs (CTL0=CTL1=1)
  mprWrite(REG_GPIOEN, LED_MASK);
  mprWrite(REG_GPIODIR, LED_MASK);
  mprWrite(REG_GPIOCTL0, LED_MASK);
  mprWrite(REG_GPIOCTL1, LED_MASK);
  mprWrite(REG_GPIOCLR, LED_MASK);

  // ── Baseline tracking — slow it down so approach registers as delta ───────────
  // MHD = max half delta: limits how fast baseline can track rising data
  // NHD = noise half delta: incremental step when drift detected
  // NCL = noise count limit: how many consecutive samples before tracking
  // FDL = filter delay: how slowly the filter operates
  //
  // Default (all 0) = baseline tracking disabled, which causes unstable behaviour.
  // These values let baseline track slow environmental drift (seconds)
  // but NOT fast hand approaches (tens of ms).
  //
  // Rising = electrode data going UP (baseline chasing approach → we slow this down hard)
  // Falling = electrode data going DOWN (baseline falling after hand leaves → let it follow)
  // ── CORRECTED baseline filter direction ──────────────────────────────────────
  // Rising  = filt > baseline: at rest and after hand leaves. Make this FAST
  //           so baseline stays matched to filt when nobody is near.
  // Falling = filt < baseline: hand approaching. Make this VERY SLOW
  //           so baseline doesn't chase the hand and cancel the signal.
  mprWrite(0x2B, 4);    // MHD Rising  — up to 4 counts per sample step
  mprWrite(0x2C, 2);    // NHD Rising  — 2-count increments
  mprWrite(0x2D, 5);    // NCL Rising  — only 5 consecutive samples needed
  mprWrite(0x2E, 5);    // FDL Rising  — fast filter delay → baseline recovers quickly
  mprWrite(0x2F, 1);    // MHD Falling — limit max step (cap rapid drops)
  mprWrite(0x30, 1);    // NHD Falling — 1-count increments
  mprWrite(0x31, 200);  // NCL Falling — 200 consecutive samples before baseline moves
  mprWrite(0x32, 255);  // FDL Falling — maximum delay → baseline almost frozen during approach
  // ⑥ Run mode: CL=10 (load baseline from first measurement), ELE_EN=6 (ELE0–ELE5)
  mprWrite(REG_ECR, 0b10000110);
  delay(100);  // let baseline settle before reading initial values
  // After mprWrite(REG_ECR, 0b10000110) and delay(100):
  // Force baseline to reload from current filt (CL=11 = full 10-bit load)
  delay(400);               // let filt settle with new CDC
  mprWrite(REG_ECR, 0x00);  // stop
  delay(10);
  mprWrite(REG_ECR, 0b11000110);  // CL=11 (full reload), ELE_EN=6
  delay(100);
  // ⑦ Seed EMA state from real readings so fast=slow=current_delta at t=0.
  //    This prevents jump detector from firing spuriously on startup.
  {
    uint8_t filtBuf[12], baseBuf[6];
    mprBurst(REG_FILT_0L, filtBuf, 12);
    mprBurst(REG_BASE_0, baseBuf, 6);
    for (uint8_t i = 0; i < 6; i++) {
      uint16_t filt = (uint16_t)filtBuf[2 * i] | ((uint16_t)(filtBuf[2 * i + 1] & 0x03) << 8);
      uint16_t baseline = (uint16_t)baseBuf[i] << 2;
      float d = (float)max(0, (int16_t)baseline - (int16_t)filt);
      el[i].fast = el[i].slow = el[i].prevFast = d;
    }
  }

  // ⑧ Startup LED animation (runs in Run mode, GPIO writes work at any time)
  ledFadeSetup();

  // ⑨ Serial plotter header (lines starting with '#' are ignored by plotter)
  Serial.println(F("# s0..s5 = smoothed proximity (counts), j0..j5 = jump detector"));
  Serial.println(F("# Scale: PROX_MAX counts full scale; jump spikes on metal contact"));
}

// ── Loop ──────────────────────────────────────────────────────────────────────
static uint32_t lastUpdate = 0;

void loop() {
  uint32_t now = millis();
  if (now - lastUpdate < 8) return;  // ~60 Hz
  lastUpdate = now;

  // ── Burst-read all data in 3 transactions ─────────────────────────────────
  uint8_t filtBuf[12];  // ELE0–ELE5 filtered data (2 bytes each)
  uint8_t baseBuf[6];   // ELE0–ELE5 baseline (1 byte each, 8 MSBs)
  uint8_t touchBuf[2];  // touch status registers 0x00 and 0x01
  mprBurst(REG_FILT_0L, filtBuf, 12);
  mprBurst(REG_BASE_0, baseBuf, 6);
  mprBurst(REG_TOUCH_L, touchBuf, 2);

  uint16_t touchStatus = touchBuf[0] | ((uint16_t)touchBuf[1] << 8);

  // ── Update electrode state ────────────────────────────────────────────────
  for (uint8_t i = 0; i < 6; i++) {
    // 10-bit filtered value (inversely proportional to capacitance)
    uint16_t filtered = (uint16_t)filtBuf[2 * i]
                        | ((uint16_t)(filtBuf[2 * i + 1] & 0x03) << 8);
    // 10-bit baseline (8 MSBs left-shifted, per datasheet §5.4)
    uint16_t baseline = (uint16_t)baseBuf[i] << 2;

    // delta: positive = hand is near.
    // Clamped to 0 to avoid negative noise confusing the EMA.
    int16_t raw = (int16_t)baseline - (int16_t)filtered;
    if (raw < 0) raw = 0;

    // EMA update
    el[i].prevFast = el[i].fast;
    el[i].fast = FAST_ALPHA * raw + (1.0f - FAST_ALPHA) * el[i].fast;
    el[i].slow = SLOW_ALPHA * raw + (1.0f - SLOW_ALPHA) * el[i].slow;

    // Smoothed velocity: rate of change of fast EMA
    float rawVel = el[i].fast - el[i].prevFast;
    el[i].velocity = VEL_ALPHA * rawVel + (1.0f - VEL_ALPHA) * el[i].velocity;

    // Hardware touch flag
    el[i].prevHwTouch = el[i].hwTouch;
    el[i].hwTouch = (touchStatus >> i) & 1;

    // ── Contact (jump) detection ──────────────────────────────────────────
    // jump stays near 0 during slow approach (both EMAs move together),
    // spikes when metal contact causes rawDelta to jump suddenly
    // (fast catches it, slow doesn't).
    float jump = el[i].fast - el[i].slow;

    if (jump > JUMP_THRESHOLD && (now - el[i].lastTouchMs) > TOUCH_COOLDOWN_MS) {
      el[i].lastTouchMs = now;
      onTouch(i, el[i].velocity);
      // Flag in serial output below
    }

    // ── Continuous proximity callback ─────────────────────────────────────
    float intensity = constrain(
      (el[i].fast - PROX_DEADBAND) / (PROX_MAX - PROX_DEADBAND),
      0.0f, 1.0f);
    onProximity(i, intensity);
  }

  // ── Update LEDs ───────────────────────────────────────────────────────────
  updateLEDs();

  // ── Diagnostic serial output  ────────────
  // Shows raw chip values so you can see what's happening before any software processing.
  // Columns: ch | filtered(10bit) | baseline(10bit) | hw_delta | sw_fast | sw_jump
  //
  // What to look for:
  //   • filtered should be ~550-900 at rest (mid-range of 0–1023)
  //     If it's 0 or 1023 the CDC/CDT needs tuning for your electrode geometry
  //   • baseline should closely track filtered at rest (they drift together)
  //   • hw_delta = baseline - filtered: this should rise as hand approaches,
  //     even without touching. If it only moves on metal contact, baseline is
  //     chasing your hand — slow it down by increasing NCL/FDL rising registers
  //   • sw_fast is your smoothed version; sw_jump should spike on metal contact

  static uint32_t lastDiagPrint = 0;
  if (millis() - lastDiagPrint >= 50) {  // 20Hz — readable in serial monitor
    lastDiagPrint = millis();

    uint8_t filtBuf[12], baseBuf[6];
    mprBurst(REG_FILT_0L, filtBuf, 12);
    mprBurst(REG_BASE_0, baseBuf, 6);
    Serial.println(F("ch  filt  base  hw_d  noise_ceil  headroom"));
    for (uint8_t i = 0; i < 6; i++) {
      uint16_t filt = (uint16_t)filtBuf[2 * i]
                      | ((uint16_t)(filtBuf[2 * i + 1] & 0x03) << 8);
      uint16_t base = (uint16_t)baseBuf[i] << 2;
      int16_t hwd = (int16_t)base - (int16_t)filt;
      // headroom = how far filt can drop before hitting 0
      // noise_ceil = rough estimate, ideally you want signal >> 6 counts
      Serial.print(i);
      Serial.print(F("   "));
      Serial.print(filt);
      Serial.print(F("  "));
      Serial.print(base);
      Serial.print(F("  "));
      Serial.print(hwd);
      Serial.print(F("     ±6 est    "));
      Serial.println(filt);  // headroom = filt value itself (distance to 0)
    }
  }
}