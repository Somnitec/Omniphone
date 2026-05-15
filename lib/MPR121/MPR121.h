#pragma once
#include <Arduino.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────────────────────
// Minimal MPR121 capacitive sensor driver
//
// Supports up to 4 boards on the same I²C bus via different addresses:
//   0x5A  ADDR pin = GND
//   0x5B  ADDR pin = VCC
//   0x5C  ADDR pin = SDA
//   0x5D  ADDR pin = SCL
//
// Hardware layout assumed by this driver:
//   ELE0–ELE5  : capacitive-touch / proximity inputs
//   ELE6–ELE11 : configured as GPIO2–GPIO7 → PWM-dimmed LED outputs
//
// Calling begin() performs a full chip reset, configures timing for
// flicker-free LED output, sets baseline tracking, and enters run mode.
// ─────────────────────────────────────────────────────────────────────────────

// ── Register map ─────────────────────────────────────────────────────────────
namespace MPR121Reg {
    static constexpr uint8_t TOUCH_L   = 0x00; // Touch status, low byte
    static constexpr uint8_t TOUCH_H   = 0x01; // Touch status, high byte
    static constexpr uint8_t FILT_0L   = 0x04; // ELE0 filtered data LSB; ELEi → 0x04+2i
    static constexpr uint8_t BASE_0    = 0x1E; // ELE0 baseline (8 MSBs); ELEi → 0x1E+i

    // Baseline filter – rising path (filt > baseline: hand leaving, at rest)
    static constexpr uint8_t MHD_R     = 0x2B; // Max half-delta rising
    static constexpr uint8_t NHD_R     = 0x2C; // Noise half-delta rising
    static constexpr uint8_t NCL_R     = 0x2D; // Noise count limit rising
    static constexpr uint8_t FDL_R     = 0x2E; // Filter delay rising

    // Baseline filter – falling path (filt < baseline: hand approaching)
    static constexpr uint8_t MHD_F     = 0x2F; // Max half-delta falling
    static constexpr uint8_t NHD_F     = 0x30; // Noise half-delta falling
    static constexpr uint8_t NCL_F     = 0x31; // Noise count limit falling
    static constexpr uint8_t FDL_F     = 0x32; // Filter delay falling

    // Touch / release thresholds (2 bytes per electrode, interleaved)
    static constexpr uint8_t TOUCH_TH0 = 0x41; // ELE0 touch threshold; ELEi → 0x41+2i
    static constexpr uint8_t REL_TH0   = 0x42; // ELE0 release threshold

    // Scan timing
    static constexpr uint8_t CDC_CFG   = 0x5C; // FFI[1:0] | CDC[5:0] – charge current & filter
    static constexpr uint8_t CDT_CFG   = 0x5D; // CDT[2:0] | SFI[1:0] | ESI[2:0] – timing
    static constexpr uint8_t ECR       = 0x5E; // Electrode control / run mode
    static constexpr uint8_t SRST      = 0x80; // Soft reset (write 0x63)

    // GPIO / PWM registers (for ELE6–ELE11 used as LEDs)
    static constexpr uint8_t GPIOCTL0  = 0x73; // GPIO control register 0
    static constexpr uint8_t GPIOCTL1  = 0x74; // GPIO control register 1
    static constexpr uint8_t GPIODIR   = 0x76; // GPIO direction (1 = output)
    static constexpr uint8_t GPIOEN    = 0x77; // GPIO enable (1 = GPIO mode)
    static constexpr uint8_t GPIOSET   = 0x78; // Write 1 to turn pin on
    static constexpr uint8_t GPIOCLR   = 0x79; // Write 1 to turn pin off

    // PWM brightness: each nibble is one GPIO pin, 0 = off, 15 = full
    static constexpr uint8_t PWM0      = 0x81; // GPIO0 [3:0], GPIO1 [7:4]  ← ELE4, ELE5
    static constexpr uint8_t PWM1      = 0x82; // GPIO2 [3:0], GPIO3 [7:4]  ← LED 0, 1
    static constexpr uint8_t PWM2      = 0x83; // GPIO4 [3:0], GPIO5 [7:4]  ← LED 2, 3
    static constexpr uint8_t PWM3      = 0x84; // GPIO6 [3:0], GPIO7 [7:4]  ← LED 4, 5
}

// ─────────────────────────────────────────────────────────────────────────────

class MPR121 {
public:
    // addr : I²C address of this board (0x5A–0x5D)
    // wire : I²C bus instance (default Wire)
    explicit MPR121(uint8_t addr, TwoWire& wire = Wire);

    // Initialise the chip. Returns false if the device cannot be reached.
    //   numElectrodes : how many of ELE0–ELE5 to use as touch inputs (1–6)
    //   touchTh       : raw hardware touch threshold (lower = more sensitive)
    //   releaseTh     : raw hardware release threshold (must be < touchTh)
    bool begin(uint8_t numElectrodes = 6,
               uint8_t touchTh = 40, uint8_t releaseTh = 20);

    // ── Sensor reads ─────────────────────────────────────────────────────────

    // 10-bit filtered capacitance value. Lower value = more capacitance = hand close.
    uint16_t filteredData(uint8_t electrode);

    // 10-bit effective baseline. The chip stores only the 8 MSBs; we left-shift
    // by 2 to reconstruct an approximate 10-bit value.
    uint16_t baselineData(uint8_t electrode);

    // Bitmask of hardware-detected touches. Bit i is set when electrode i is touched.
    uint16_t touchStatus();

    // ── LED helpers (ELE6–ELE11 wired as open-drain PWM LED outputs) ─────────

    // Call once after begin() to configure ELE6–ELE11 as LED outputs.
    void beginLEDs();

    // Set brightness of one LED: ledIndex 0–5, bri 0 (off) or 1–15 (dim–full).
    void setLED(uint8_t ledIndex, uint8_t bri);

    // Write all 6 LED brightnesses in a minimal burst of I²C writes.
    // (Legacy ELE6–ELE11 map; used by the test sketches.)
    void setAllLEDs(const uint8_t bri[6]);

    // Generalised path: brightness indexed by GPIO bit, where index g drives
    // ELE(g+4). g=0 (ELE4) is ignored (it's a touch electrode); g=1..7 cover
    // ELE5–ELE11. 0 = off, 1–15 = dim–full. Used by the main firmware.
    void setLEDs8(const uint8_t bri[8]);

    // ── Raw register access ──────────────────────────────────────────────────
    void    write(uint8_t reg, uint8_t val);
    uint8_t read(uint8_t reg);
    // Burst read n bytes starting at reg. MPR121 auto-increments the address.
    void    burstRead(uint8_t reg, uint8_t* buf, uint8_t n);

private:
    uint8_t  _addr;
    TwoWire& _wire;

    // Bits 2–7 of the GPIO port correspond to ELE6–ELE11 (our 6 LEDs).
    static constexpr uint8_t LED_MASK = 0xFC;
    // Logical LED index 0–5 maps to GPIO pins 2–7 (bit = 1 << (index + 2)).
    static uint8_t ledBit(uint8_t i) { return static_cast<uint8_t>(1u << (i + 2)); }
};
