#include <Arduino.h>
#include <Wire.h>
#include <MPR121.h>
#include "config.h" // NUM_BOARDS, BOARD_ADDRESSES

// ─────────────────────────────────────────────────────────────────────────────
// LED PWM walk  —  every LED-capable pin, open-drain + hardware PWM
//
// Build:  pio run -e electrode_relocation -t upload   (serial @115200)
//
// Steps through ALL 8 GPIO/LED pins in ELE order — ELE4..ELE11 (GPIO0..GPIO7)
// — one at a time, in the SAME high-side open-drain + hardware-PWM mode that
// ELE9/ELE10 fail in. A working pin visibly pulses 0→full→0; a broken one
// stays dark (or only lights when a neighbour is also driven). Compare pins,
// and compare the two boards.
//
// begin(4) makes ELE0–3 nominal sense and frees ELE4–ELE11 as GPIO. This is a
// per-pin health probe, NOT a production config (ELE_EN is a contiguous count
// from ELE0, so all 8 can't be LEDs while you still have 5–6 sense channels).
//
// Pin map:  ELEx → GPIO(x-4) → data bit (x-4)
//           PWM duty reg 0x81+((x-4)/2);  even GPIO = low nibble, odd = high.
//
// NOTE: the ELE6 step intentionally drives ELE5 too, at the same value
// (diagnostic pairing — does ELE6 only work alongside its neighbour?).
//
// Serial:  0/1 = board 0 (0x5A) / board 1 (0x5C)
//          n = next LED   p = prev LED   m = manual/auto   r = ramp/steady
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t REG_PWM_BASE = 0x81; // 0x81..0x84 cover GPIO0..GPIO7
static constexpr uint8_t ALL_GPIO     = 0xFF; // GPIO0..GPIO7 = ELE4..ELE11
static constexpr uint8_t LED_COUNT    = 8;    // ELE4..ELE11

static MPR121 boards[NUM_BOARDS] = {
    MPR121(BOARD_ADDRESSES[0]),
    MPR121(BOARD_ADDRESSES[1]),
};

static uint8_t boardSel = 0;       // active board index
static uint8_t ledIdx   = 0;       // 0..7  → ELE(ledIdx+4)
static bool    manual   = true;    // default: step manually with 'n'
static bool    steady   = false;   // false = ramp, true = steady full
static bool    stepNext = false;

static inline uint8_t eleOf(uint8_t led) { return led + 4; }

static void printStep()
{
    uint8_t reg = REG_PWM_BASE + (ledIdx / 2);
    Serial.print(F("# ["));
    Serial.print(F("brd"));
    Serial.print(boardSel);
    Serial.print(F(" 0x"));
    Serial.print(BOARD_ADDRESSES[boardSel], HEX);
    Serial.print(manual ? F(" man "  ) : F(" auto "));
    Serial.print(steady ? F("steady]") : F("ramp]" ));
    Serial.print(F(" ELE"));
    Serial.print(eleOf(ledIdx));
    Serial.print(F(" (GPIO"));
    Serial.print(ledIdx);
    Serial.print(F(", PWM 0x"));
    Serial.print(reg, HEX);
    Serial.print((ledIdx & 1) ? F(" hi)") : F(" lo)"));
    if (ledIdx == 2) Serial.print(F("  +ELE5 (paired)"));
    Serial.println();
}

// ── High-side open-drain LED-driver config on all 8 GPIO (production mode) ────
static void cfgOpenDrain(MPR121 &b)
{
    b.write(MPR121Reg::GPIOEN,   ALL_GPIO);
    b.write(MPR121Reg::GPIODIR,  ALL_GPIO);
    b.write(MPR121Reg::GPIOCTL0, ALL_GPIO);
    b.write(MPR121Reg::GPIOCTL1, ALL_GPIO);
    b.write(MPR121Reg::GPIOCLR,  ALL_GPIO);
}

// Which GPIO bits a given step lights. Normally just that LED — but the ELE6
// step (ledIdx 2) also lights ELE5 (ledIdx 1) at the same value.
static uint8_t ledMask(uint8_t led)
{
    if (led == 2) return (uint8_t)((1u << 5) | (1u << 6)); // ELE9/GPIO5 + ELE10/GPIO6
    return (uint8_t)(1u << led);
}

// Light every LED in `mask` at `duty` on board `b`; all others forced off.
// mask = 0 → all off.
static void driveMask(uint8_t b, uint8_t mask, uint8_t duty)
{
    MPR121 &brd = boards[b];
    uint8_t gset = 0, gclr = 0;
    uint8_t pwm[4] = { 0, 0, 0, 0 }; // 0x81..0x84

    for (uint8_t n = 0; n < 8; n++)
    {
        if ((mask & (1u << n)) && duty)
        {
            gset |= (uint8_t)(1u << n);
            uint8_t reg = n / 2;
            if (n & 1) pwm[reg] |= (uint8_t)((duty & 0x0F) << 4);
            else       pwm[reg] |= (uint8_t)(duty & 0x0F);
        }
        else
        {
            gclr |= (uint8_t)(1u << n);
        }
    }
    if (gset) brd.write(MPR121Reg::GPIOSET, gset);
    if (gclr) brd.write(MPR121Reg::GPIOCLR, gclr);
    for (uint8_t r = 0; r < 4; r++)
        brd.write((uint8_t)(REG_PWM_BASE + r), pwm[r]);
}

// Triangle 0→15→0 over ~2 s, from millis().
static uint8_t rampDuty()
{
    uint32_t ph = millis() % 2000;
    uint32_t up = ph < 1000 ? ph : 2000 - ph;
    return (uint8_t)((up * 15) / 1000);
}

static void handleSerial()
{
    if (!Serial.available()) return;
    char c = Serial.read();
    bool restate = true;
    switch (c)
    {
        case '0':
        case '1':
        {
            uint8_t want = (uint8_t)(c - '0');
            if (want < NUM_BOARDS && want != boardSel)
            {
                driveMask(boardSel, 0, 0); // blank the board we're leaving
                boardSel = want;
            }
            break;
        }
        case 'n': case 'N': ledIdx = (uint8_t)((ledIdx + 1) % LED_COUNT); restate = false; break;
        case 'p': case 'P': ledIdx = (uint8_t)((ledIdx + LED_COUNT - 1) % LED_COUNT); restate = false; break;
        case 'm': case 'M': manual = !manual; break;
        case 'r': case 'R': steady = !steady; break;
        default: restate = false; break;
    }
    if (c=='n'||c=='N'||c=='p'||c=='P') stepNext = true;
    if (restate) printStep();
}

// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    Wire.begin();
    Wire.setClock(400000);

    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (!boards[b].begin(4)) // ELE0–3 nominal sense, ELE4–ELE11 free as GPIO
        {
            Serial.print(F("ERROR: MPR121 board "));
            Serial.print(b);
            Serial.println(F(" not found"));
        }
        cfgOpenDrain(boards[b]);
        driveMask(b, 0, 0); // start blank
    }

    Serial.println(F("# LED PWM walk — ELE4..ELE11, open-drain + PWM"));
    Serial.println(F("#   0/1=board  n=next  p=prev  m=manual/auto  r=ramp/steady"));
    printStep();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    printStep();

    stepNext = false;
    uint32_t t0 = millis();
    for (;;)
    {
        handleSerial();
        driveMask(boardSel, ledMask(ledIdx), steady ? 15 : rampDuty());

        if (stepNext) break;                                 // n / p pressed
        if (!manual && (millis() - t0 >= 3000)) break;       // auto: 3 s/LED
        delay(5);
    }

    if (!manual && !stepNext)
        ledIdx = (uint8_t)((ledIdx + 1) % LED_COUNT);        // auto-advance
}
