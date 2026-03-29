#include "MPR121.h"
using namespace MPR121Reg;

MPR121::MPR121(uint8_t addr, TwoWire& wire)
    : _addr(addr), _wire(wire) {}

// ── Low-level I²C helpers ─────────────────────────────────────────────────────

void MPR121::write(uint8_t reg, uint8_t val) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(val);
    _wire.endTransmission();
}

uint8_t MPR121::read(uint8_t reg) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.endTransmission(false);
    _wire.requestFrom(_addr, (uint8_t)1);
    return _wire.available() ? _wire.read() : 0;
}

// The MPR121 auto-increments its internal address pointer on multi-byte reads,
// so a single requestFrom call reads a contiguous block of registers.
void MPR121::burstRead(uint8_t reg, uint8_t* buf, uint8_t n) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.endTransmission(false);
    _wire.requestFrom(_addr, n);
    for (uint8_t i = 0; i < n; i++)
        buf[i] = _wire.available() ? _wire.read() : 0;
}

// ── Initialisation ────────────────────────────────────────────────────────────

bool MPR121::begin(uint8_t numElectrodes, uint8_t touchTh, uint8_t releaseTh) {
    // ① Soft reset — all registers return to power-on defaults.
    write(SRST, 0x63);
    delay(50);

    // ② Stop mode — most registers can only be written while the chip is stopped.
    write(ECR, 0x00);

    // ── Scan timing (flicker fix) ──────────────────────────────────────────
    // Default ESI=16ms causes one long PWM freeze per 8ms PWM period → flicker.
    // ESI=2ms spreads four short freezes (~144µs each) evenly across the 8ms
    // period, making the dimming nearly invisible.
    //   CDC_CFG: FFI=00 (6 samples, fast scan), CDC=10µA charge current
    //   CDT_CFG: CDT=2µs, SFI=4, ESI=2ms
    write(CDC_CFG, 0b00001010);
    write(CDT_CFG, 0b01100001);

    // ── Touch / release thresholds ────────────────────────────────────────
    for (uint8_t i = 0; i < numElectrodes; i++) {
        write(TOUCH_TH0 + 2 * i, touchTh);
        write(REL_TH0   + 2 * i, releaseTh);
    }

    // ── Baseline filter tuning ────────────────────────────────────────────
    // Rising path (filt > baseline): happens at rest and after hand leaves.
    // Track quickly so the baseline re-locks between interactions.
    write(MHD_R,  4);   // Up to 4 counts per sample step
    write(NHD_R,  2);   // 2-count increments when drifting
    write(NCL_R,  5);   // Only 5 consecutive samples needed to update
    write(FDL_R,  5);   // Short filter delay → fast recovery
    //
    // Falling path (filt < baseline): happens when hand approaches.
    // Track very slowly so the baseline does NOT chase the hand signal
    // and cancel the measurement.
    write(MHD_F,   1);
    write(NHD_F,   1);
    write(NCL_F, 200);  // 200 consecutive samples before baseline moves
    write(FDL_F, 255);  // Maximum delay → baseline essentially frozen during approach

    // ── Start mode ────────────────────────────────────────────────────────
    // CL=10: load baseline from the first measurement (clean instrument start).
    // ELE_EN: enable the requested number of electrodes.
    write(ECR, static_cast<uint8_t>(0b10000000 | (numElectrodes & 0x0F)));
    delay(50);

    // Force a clean full-precision baseline reload:
    // wait for filtered data to settle, then re-enter with CL=11 (full 10-bit load).
    delay(200);
    write(ECR, 0x00);
    delay(10);
    write(ECR, static_cast<uint8_t>(0b11000000 | (numElectrodes & 0x0F)));
    delay(50);

    return true; // The chip has no WHO_AM_I register; connectivity is assumed OK.
}

// ── Sensor reads ──────────────────────────────────────────────────────────────

uint16_t MPR121::filteredData(uint8_t electrode) {
    uint8_t buf[2];
    burstRead(FILT_0L + 2 * electrode, buf, 2);
    // Bytes are [LSB, MSB] where the MSB only uses its two lowest bits.
    return (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x03) << 8);
}

uint16_t MPR121::baselineData(uint8_t electrode) {
    // The chip stores only the 8 most significant bits of the 10-bit baseline.
    // Left-shifting by 2 gives a usable approximation.
    return (uint16_t)read(BASE_0 + electrode) << 2;
}

uint16_t MPR121::touchStatus() {
    uint8_t buf[2];
    burstRead(TOUCH_L, buf, 2);
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

// ── LED helpers ───────────────────────────────────────────────────────────────

void MPR121::beginLEDs() {
    // Configure ELE6–ELE11 as high-side open-drain outputs.
    // CTL0=CTL1=1 sets the push-pull drive needed for the PWM dimming feature.
    write(GPIOEN,   LED_MASK);
    write(GPIODIR,  LED_MASK);
    write(GPIOCTL0, LED_MASK);
    write(GPIOCTL1, LED_MASK);
    write(GPIOCLR,  LED_MASK); // All LEDs off on startup
}

void MPR121::setLED(uint8_t ledIndex, uint8_t bri) {
    // Logical LED index 0–5 maps to GPIO pins 2–7.
    // Each GPIO pin is controlled by one nibble in the PWM registers:
    //   GPIO2 (LED0) → PWM1[3:0]   GPIO3 (LED1) → PWM1[7:4]
    //   GPIO4 (LED2) → PWM2[3:0]   GPIO5 (LED3) → PWM2[7:4]
    //   GPIO6 (LED4) → PWM3[3:0]   GPIO7 (LED5) → PWM3[7:4]
    uint8_t gpioIdx = ledIndex + 2;
    uint8_t regAddr = PWM1 + (gpioIdx - 2) / 2;
    bool    hiNib   = (gpioIdx & 1);

    if (bri == 0) {
        write(GPIOCLR, ledBit(ledIndex));
    } else {
        write(GPIOSET, ledBit(ledIndex));
        uint8_t cur = read(regAddr);
        if (hiNib)  cur = (cur & 0x0F) | ((bri & 0x0F) << 4);
        else        cur = (cur & 0xF0) | (bri & 0x0F);
        write(regAddr, cur);
    }
}

void MPR121::setAllLEDs(const uint8_t bri[6]) {
    uint8_t setMask = 0, clrMask = 0;
    uint8_t pwm[3] = {0, 0, 0}; // maps to PWM1, PWM2, PWM3

    for (uint8_t i = 0; i < 6; i++) {
        if (bri[i] == 0) {
            clrMask |= ledBit(i);
        } else {
            setMask |= ledBit(i);
            uint8_t gpioIdx = i + 2;
            uint8_t regIdx  = (gpioIdx - 2) / 2; // 0, 0, 1, 1, 2, 2
            bool    hiNib   = (gpioIdx & 1);
            if (hiNib)  pwm[regIdx] |= (bri[i] & 0x0F) << 4;
            else        pwm[regIdx] |= (bri[i] & 0x0F);
        }
    }

    // Batch the GPIO enable/disable + PWM writes to minimise I²C round trips.
    if (setMask) write(GPIOSET, setMask);
    if (clrMask) write(GPIOCLR, clrMask);
    write(PWM1, pwm[0]);
    write(PWM2, pwm[1]);
    write(PWM3, pwm[2]);
}
