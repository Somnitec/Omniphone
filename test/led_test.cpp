#include <Arduino.h>
#include <Wire.h>
#include <MPR121.h>
#include "config.h" // NUM_BOARDS, BOARD_ADDRESSES

// ─────────────────────────────────────────────────────────────────────────────
// LED Test  —  isolate the "ELE9/ELE10 won't light alone, work together" fault
//
// Build:  pio run -e led_test -t upload   (then open serial @115200)
//
// LEDs are on ELE6–ELE11 = GPIO2–GPIO7 = GPIO data bits 2–7 (LED index 0–5).
// The suspect pair is LED3 = ELE9 and LED4 = ELE10.
//
// Three drive modes, selectable live over serial. Each runs the same A/B/C
// pattern so you can compare them directly:
//
//   p  PROD   — production path: setAllLEDs() with real PWM dimming (bri=10).
//               Open-drain high-side LED driver. Reproduces the field bug.
//   o  ODFULL — same open-drain LED-driver mode, but PWM forced full (0x0F).
//               Removes PWM *modulation* as a variable, keeps the driver path.
//   d  DCCMOS — CMOS push-pull output, NO PWM at all. Pure digital on/off.
//               Tests the LED + wiring electrically, bypassing the MPR121
//               LED-driver/PWM subsystem entirely.
//
// Decision tree (watch LED3-alone / LED4-alone in Phase B):
//   fails in p  but OK in o   → it's the PWM duty/modulation interaction
//   fails in o  but OK in d   → it's the open-drain LED-driver mode itself
//   fails even in d           → genuine per-channel hardware (LED/resistor/pin)
//
// Phase A — each LED 0..5 alone (control: every lamp must light by itself).
// Phase B — LED3 alone, LED4 alone, LED3+LED4 (the suspect pair).
// Phase C — all on, then all off.
// Drives all boards from config.h in parallel.
// ─────────────────────────────────────────────────────────────────────────────

// LED channels: ELE6..ELE11 → GPIO bits 2..7
static constexpr uint8_t LED_MASK = 0xFC;
static inline uint8_t ledBit(uint8_t i) { return (uint8_t)(1u << (i + 2)); }

static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]),
    MPR121(BOARD_ADDRESSES[1]),
};

enum Mode : uint8_t { MODE_PROD, MODE_ODFULL, MODE_DCCMOS };
static Mode mode = MODE_DCCMOS; // default = the no-PWM electrical test

// Which board(s) are driven: 0 = board0 only, 1 = board1 only, 2 = both.
static uint8_t boardSel = 2;
static inline bool boardActive(uint8_t b) { return boardSel == 2 || boardSel == b; }

// Stepping: manual=false auto-advances; manual=true waits for 'n' between steps.
static bool    manual   = false;
static bool    stepNext = false;
static uint8_t stepIdx  = 0;

// 11-step sequence: A LED0..5 alone, B LED3/LED4/LED3+4, C all-on/all-off.
static constexpr uint8_t STEP_COUNT = 11;

static uint8_t stepMask(uint8_t i)
{
    if (i < 6)  return (uint8_t)(1u << i);            // A: LEDi alone
    if (i == 6) return 1u << 3;                       // B: LED3 alone (ELE9)
    if (i == 7) return 1u << 4;                       // B: LED4 alone (ELE10)
    if (i == 8) return (1u << 3) | (1u << 4);         // B: LED3 + LED4
    if (i == 9) return 0x3F;                          // C: all on
    return 0x00;                                      // C: all off
}

static uint16_t stepDur(uint8_t i) { return (i < 6) ? 800 : (i < 9) ? 1500 : 1000; }

static void printStep(uint8_t i)
{
    Serial.print(F("# ["));
    Serial.print(boardSel == 2 ? F("both") : boardSel == 0 ? F("brd0") : F("brd1"));
    Serial.print(manual ? F(" man] ") : F(" auto] "));
    if (i < 6)
    {
        Serial.print(F("A: LED "));
        Serial.print(i);
        Serial.print(F(" alone (ELE"));
        Serial.print(i + 6);
        Serial.println(F(")"));
    }
    else if (i == 6) Serial.println(F("B: LED3 alone (ELE9)"));
    else if (i == 7) Serial.println(F("B: LED4 alone (ELE10)"));
    else if (i == 8) Serial.println(F("B: LED3 + LED4 (ELE9+ELE10)"));
    else if (i == 9) Serial.println(F("C: ALL on"));
    else             Serial.println(F("C: ALL off"));
}

// ── GPIO configuration ───────────────────────────────────────────────────────
// LED-driver mode: high-side open drain (DIR=1, CTL0=1, CTL1=1) — production.
static void cfgOpenDrain(MPR121 &b)
{
    b.write(MPR121Reg::GPIOEN,   LED_MASK);
    b.write(MPR121Reg::GPIODIR,  LED_MASK);
    b.write(MPR121Reg::GPIOCTL0, LED_MASK);
    b.write(MPR121Reg::GPIOCTL1, LED_MASK);
    b.write(MPR121Reg::GPIOCLR,  LED_MASK);
}
// CMOS push-pull output (DIR=1, CTL0=0, CTL1=0). No open-drain, no PWM.
static void cfgCMOS(MPR121 &b)
{
    b.write(MPR121Reg::GPIOEN,   LED_MASK);
    b.write(MPR121Reg::GPIODIR,  LED_MASK);
    b.write(MPR121Reg::GPIOCTL0, 0x00);
    b.write(MPR121Reg::GPIOCTL1, 0x00);
    b.write(MPR121Reg::GPIOCLR,  LED_MASK);
}

static void applyMode()
{
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (mode == MODE_DCCMOS) cfgCMOS(boards[b]);
        else                     cfgOpenDrain(boards[b]);
    }
    Serial.print(F("# Mode = "));
    Serial.println(mode == MODE_PROD   ? F("PROD  (open-drain + PWM dimming, bri=10)")
                  : mode == MODE_ODFULL ? F("ODFULL (open-drain LED driver, PWM full)")
                                        : F("DCCMOS (CMOS push-pull, NO PWM)"));
}

// ── Drive a 6-bit LED pattern on every board, honouring the active mode ───────
static void applyPattern(uint8_t onMask6)
{
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        MPR121 &brd = boards[b];
        // Inactive boards are forced fully off so only the selected board lights.
        uint8_t m = boardActive(b) ? onMask6 : 0;

        if (mode == MODE_PROD)
        {
            // Exact production path: setAllLEDs() with real dimming.
            uint8_t bri[6];
            for (uint8_t i = 0; i < 6; i++)
                bri[i] = (m & (1u << i)) ? 10 : 0;
            brd.setAllLEDs(bri);
            continue;
        }

        // ODFULL / DCCMOS: raw GPIO, full brightness, no dimming.
        uint8_t gset = 0, gclr = 0;
        uint8_t pwm[3] = { 0, 0, 0 }; // 0x82, 0x83, 0x84
        for (uint8_t i = 0; i < 6; i++)
        {
            bool on = m & (1u << i);
            if (on) gset |= ledBit(i);
            else    gclr |= ledBit(i);

            if (on && mode == MODE_ODFULL)
            {
                uint8_t reg = i / 2;          // 0,0,1,1,2,2
                bool    hi  = ((i + 2) & 1);  // GPIO(i+2) odd → high nibble
                if (hi) pwm[reg] |= 0x0F << 4;
                else    pwm[reg] |= 0x0F;
            }
        }
        if (gset) brd.write(MPR121Reg::GPIOSET, gset);
        if (gclr) brd.write(MPR121Reg::GPIOCLR, gclr);
        if (mode == MODE_ODFULL)
        {
            brd.write(MPR121Reg::PWM1, pwm[0]);
            brd.write(MPR121Reg::PWM2, pwm[1]);
            brd.write(MPR121Reg::PWM3, pwm[2]);
        }
    }
}

// ── Serial control ───────────────────────────────────────────────────────────
// p/o/d = drive mode   0/1/2 = board0 / board1 / both   m = manual toggle
// n = next step (works in manual; also skips ahead in auto)
static void handleSerial()
{
    if (!Serial.available()) return;
    char c = Serial.read();

    bool modeKey = false, restate = false;
    switch (c)
    {
        case 'p': case 'P': mode = MODE_PROD;   modeKey = true; break;
        case 'o': case 'O': mode = MODE_ODFULL; modeKey = true; break;
        case 'd': case 'D': mode = MODE_DCCMOS; modeKey = true; break;
        case '0':           boardSel = 0; restate = true;       break;
        case '1':           boardSel = 1; restate = true;       break;
        case '2':           boardSel = 2; restate = true;       break;
        case 'm': case 'M': manual = !manual; restate = true;   break;
        case 'n': case 'N': stepNext = true;                    break;
        default: break;
    }

    if (modeKey) { applyMode(); restate = true; } // applyMode() reconfigures GPIO
    if (restate)                                  // re-show current step now
    {
        applyPattern(stepMask(stepIdx));
        printStep(stepIdx);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    Wire.begin();
    Wire.setClock(400000);

    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (!boards[b].begin()) // default 6 electrodes → ELE6–ELE11 free for GPIO
        {
            Serial.print(F("ERROR: MPR121 board "));
            Serial.print(b);
            Serial.println(F(" not found"));
        }
        boards[b].beginLEDs();
    }

    Serial.println(F("# LED test"));
    Serial.println(F("#   mode:  p=PROD  o=ODFULL  d=DCCMOS"));
    Serial.println(F("#   board: 0=brd0 only  1=brd1 only  2=both"));
    Serial.println(F("#   step:  m=toggle manual/auto  n=next step"));
    applyMode();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    printStep(stepIdx);
    applyPattern(stepMask(stepIdx));

    // Hold this step: auto = until stepDur elapses; manual = until 'n'.
    // 'n' always advances; mode/board/manual keys re-show the step live.
    stepNext = false;
    uint32_t t0 = millis();
    for (;;)
    {
        handleSerial();
        if (stepNext) break;
        if (!manual && (millis() - t0 >= stepDur(stepIdx))) break;
        delay(5);
    }

    stepIdx = (uint8_t)((stepIdx + 1) % STEP_COUNT);
}
