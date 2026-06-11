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

// Configure the chip but leave it stopped — no scanning yet. Use this when
// you want to read raw filtered data with the baseline frozen (CL=00) before
// committing the final baseline lock via lockBaseline().
bool MPR121::beginConfig(uint8_t numElectrodes, uint8_t touchTh, uint8_t releaseTh,
                         uint8_t cdc, uint8_t cdt) {
    write(SRST, 0x63);
    delay(50);
    write(ECR, 0x00);

    write(CDC_CFG, (uint8_t)(cdc & 0x3F));
    write(CDT_CFG, (uint8_t)(((cdt & 0x07) << 5) | 0b001));

    for (uint8_t i = 0; i < numElectrodes; i++) {
        write(TOUCH_TH0 + 2 * i, touchTh);
        write(REL_TH0   + 2 * i, releaseTh);
    }

    write(MHD_R,  4);  write(NHD_R,  2);  write(NCL_R,   5); write(FDL_R,   5);
    write(MHD_F,  1);  write(NHD_F,  1);  write(NCL_F, 200); write(FDL_F, 255);
    return true;
}

// Start scanning. baselineMode is the CL field (00=no init/frozen,
// 10=load from 5 MSBs, 11=load from full 10 bits then track).
void MPR121::startScanning(uint8_t numElectrodes, uint8_t baselineMode) {
    write(ECR, (uint8_t)(((baselineMode & 0x03) << 6) | (numElectrodes & 0x0F)));
    delay(50);
}

// Force the chip to take the current filtered data as the new baseline (CL=11
// reload). Use after a quiet period — anything sitting under the hand becomes
// the new zero. Must be in run mode already (this stops, settles, restarts).
void MPR121::lockBaseline(uint8_t numElectrodes) {
    // 200 ms ≈ 100 scan cycles at ESI=2 ms — plenty for filtered EMA to settle.
    delay(200);
    write(ECR, 0x00);
    delay(10);
    write(ECR, (uint8_t)(0b11000000 | (numElectrodes & 0x0F)));
    delay(50);
}

bool MPR121::begin(uint8_t numElectrodes, uint8_t touchTh, uint8_t releaseTh,
                   uint8_t cdc, uint8_t cdt) {
    if (!beginConfig(numElectrodes, touchTh, releaseTh, cdc, cdt)) return false;
    startScanning(numElectrodes, 0b10); // CL=10: track from first sample
    lockBaseline(numElectrodes);         // CL=11: full reload after settle

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
    // Configure ELE4–ELE11 (GPIO0–GPIO7) as high-side open-drain LED outputs.
    // CTL0=CTL1=1 is the open-drain LED-driver mode the hardware PWM needs.
    // The MPR121 ignores these bits for any electrode currently enabled as a touch
    // input (ELEx where x < ELE_EN), so it's safe to set them all — ELE4 only acts
    // as an LED (GPIO0/LED0) when it isn't an active sense electrode.
    constexpr uint8_t mask = 0xFF; // bits 0..7 = ELE4..ELE11
    write(GPIOEN,   mask);
    write(GPIODIR,  mask);
    write(GPIOCTL0, mask);
    write(GPIOCTL1, mask);
    write(GPIOCLR,  mask); // All LEDs off on startup
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

void MPR121::setLEDs8(const uint8_t bri[8]) {
    uint8_t setMask = 0, clrMask = 0;
    uint8_t pwm[4] = {0, 0, 0, 0}; // PWM0..PWM3 = 0x81..0x84

    // g = GPIO bit; drives ELE(g+4). g=0 → ELE4 (GPIO0/LED0), g=7 → ELE11.
    for (uint8_t g = 0; g < 8; g++) {
        if (bri[g] == 0) {
            clrMask |= (uint8_t)(1u << g);
        } else {
            setMask |= (uint8_t)(1u << g);
            uint8_t reg = g / 2;                 // 0,1,1,2,2,3,3
            if (g & 1) pwm[reg] |= (uint8_t)((bri[g] & 0x0F) << 4);
            else       pwm[reg] |= (uint8_t)(bri[g] & 0x0F);
        }
    }

    if (setMask) write(GPIOSET, setMask);
    if (clrMask) write(GPIOCLR, clrMask);
    write(PWM0, pwm[0]);
    write(PWM1, pwm[1]);
    write(PWM2, pwm[2]);
    write(PWM3, pwm[3]);
}
