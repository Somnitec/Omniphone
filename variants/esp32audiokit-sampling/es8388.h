#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// ES8388 codec driver — minimal, self-contained (Wire / I²C)
//
// The AI-Thinker ESP32 Audio Kit (v2.2, A247) carries an ES8388 stereo codec.
// Unlike the PCM5102A used on the S3 variants (which needs zero config), the
// ES8388 MUST be programmed over I²C before any audio flows — both the DAC
// (→ speaker/headphone) and the ADC (← onboard mics) are powered down at reset.
//
// This is a compact port of the canonical ESP-ADF / pschatzmann init sequence,
// trimmed to the one mode this instrument needs: BOTH (record + playback) at
// 16-bit I²S, codec as I²S SLAVE (the ESP32 is master and supplies MCLK on
// GPIO0). Input = LINPUT1/RINPUT1 (the board's analog mics, MIC1 = left).
// Output = LOUT1/ROUT1 (headphone) + LOUT2/ROUT2 (speaker amp, gated by GPIO21).
//
// Register map and values: see ESP-ADF es8388.c. Addresses are the raw codec
// register numbers (CONTROL1=0x00 … DACCONTROL30=0x34).
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>

class ES8388 {
public:
    static constexpr uint8_t ADDR = 0x10;   // 7-bit I²C address (fixed)

    // ── Register addresses ───────────────────────────────────────────────────
    enum : uint8_t {
        CONTROL1    = 0x00, CONTROL2   = 0x01, CHIPPOWER  = 0x02,
        ADCPOWER    = 0x03, DACPOWER   = 0x04,
        MASTERMODE  = 0x08,
        ADCCONTROL1 = 0x09, ADCCONTROL2 = 0x0a, ADCCONTROL3 = 0x0b,
        ADCCONTROL4 = 0x0c, ADCCONTROL5 = 0x0d,
        ADCCONTROL8 = 0x10, ADCCONTROL9 = 0x11, ADCCONTROL10 = 0x12,
        ADCCONTROL11 = 0x13, ADCCONTROL12 = 0x14, ADCCONTROL13 = 0x15,
        ADCCONTROL14 = 0x16,
        DACCONTROL1 = 0x17, DACCONTROL2 = 0x18, DACCONTROL3 = 0x19,
        DACCONTROL4 = 0x1a, DACCONTROL5 = 0x1b,
        DACCONTROL16 = 0x26, DACCONTROL17 = 0x27, DACCONTROL20 = 0x2a,
        DACCONTROL21 = 0x2b, DACCONTROL23 = 0x2d,
        DACCONTROL24 = 0x2e, DACCONTROL25 = 0x2f,
        DACCONTROL26 = 0x30, DACCONTROL27 = 0x31,
    };

    bool writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(ADDR);
        Wire.write(reg);
        Wire.write(val);
        return Wire.endTransmission() == 0;
    }

    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom((int)ADDR, 1);
        return Wire.available() ? Wire.read() : 0;
    }

    // True if the codec ACKs on the bus (sanity check before init).
    bool probe() {
        Wire.beginTransmission(ADDR);
        return Wire.endTransmission() == 0;
    }

    // micGain: ADCCONTROL1 PGA, both channels. 0x00 = 0 dB … 0x88 = +24 dB …
    //          0xbb = +33 dB. Electret mics usually want 0x77–0xbb.
    bool begin(uint8_t micGain = 0x88) {
        if (!probe()) return false;
        bool ok = true;

        // ── Power-up / clock ────────────────────────────────────────────────
        ok &= writeReg(DACCONTROL3, 0x04);   // DAC muted while we configure
        ok &= writeReg(CONTROL2,    0x50);   // power up analog, low-power off
        ok &= writeReg(CHIPPOWER,   0x00);   // normal — power up all blocks
        ok &= writeReg(0x35,        0xA0);   // (ESP-ADF magic — bias/ref trim)
        ok &= writeReg(0x37,        0xD0);
        ok &= writeReg(0x39,        0xD0);
        ok &= writeReg(MASTERMODE,  0x00);   // codec = I²S SLAVE (ESP32 master)

        // ── DAC path (→ outputs) ────────────────────────────────────────────
        ok &= writeReg(DACPOWER,    0xC0);   // outputs off until configured
        ok &= writeReg(CONTROL1,    0x12);   // play & record mode, no DACL/R swap
        ok &= writeReg(DACCONTROL1, 0x18);   // 16-bit I²S
        ok &= writeReg(DACCONTROL2, 0x02);   // single-speed, 256fs
        ok &= writeReg(DACCONTROL16, 0x00);  // mixer source = LIN1/RIN1
        ok &= writeReg(DACCONTROL17, 0x90);  // L DAC → L mixer, 0 dB
        ok &= writeReg(DACCONTROL20, 0x90);  // R DAC → R mixer, 0 dB
        ok &= writeReg(DACCONTROL21, 0x80);  // ADC & DAC share one LRCK
        ok &= writeReg(DACCONTROL23, 0x00);  // VROI = 0
        setDacVolume(0);                     // 0 dB digital DAC volume
        ok &= writeReg(DACPOWER,    0x3C);   // enable LOUT1/ROUT1 + LOUT2/ROUT2

        // ── ADC path (← onboard mics on LIN1/RIN1) ──────────────────────────
        ok &= writeReg(ADCPOWER,    0xFF);   // power down ADC while configuring
        ok &= writeReg(ADCCONTROL1, micGain);// mic PGA gain (L+R)
        ok &= writeReg(ADCCONTROL2, 0x00);   // LIN1/RIN1 as ADC input
        ok &= writeReg(ADCCONTROL3, 0x02);   // ADC mono/stereo, mic bias normal
        ok &= writeReg(ADCCONTROL4, 0x0d);   // 16-bit I²S, left/right justified
        ok &= writeReg(ADCCONTROL5, 0x02);   // single-speed, 256fs
        setAdcVolume(0);                     // 0 dB digital ADC volume
        // Full ADC power-up: bit3 PdnMICB = 0 → MIC BIAS ON (electret mics need
        // it; 0x09 leaves bias OFF, giving only faint leakage). ESP-ADF does this
        // in start(); we fold it into begin().
        ok &= writeReg(ADCPOWER,    0x00);   // ADC + LIN/RIN + mic bias all on

        ok &= writeReg(DACCONTROL3, 0x00);   // un-mute the DAC
        return ok;
    }

    // Select which analog pins feed the ADC (ADCCONTROL2 LINSEL/RINSEL, high
    // nibble). 0x00 = LINPUT1/RINPUT1, 0x50 = LINPUT2/RINPUT2, 0xF0 = differential.
    // The onboard mic isn't always on input 1 — use this to find it.
    void setAdcInput(uint8_t linRinSel) {
        uint8_t cur = readReg(ADCCONTROL2);
        writeReg(ADCCONTROL2, (uint8_t)((cur & 0x0F) | (linRinSel & 0xF0)));
    }

    // Enable the ADC ALC (automatic level control / mic AGC). The onboard analog
    // mics on the Audio Kit are quiet; the +24 dB PGA alone leaves the signal far
    // below full scale. ALC adds make-up gain so quiet sources record usefully.
    //   maxGain 0–7 → roughly -6.5 dB … +35.5 dB of extra gain (≈ 6 dB/step).
    //                 Higher = louder quiet sounds but more amplified idle hiss.
    //   noiseGate     when true, mutes the ADC below a low threshold to kill
    //                 idle hiss between sounds (can clip very soft tails).
    void enableMicALC(uint8_t maxGain = 5, bool noiseGate = false) {
        if (maxGain > 7) maxGain = 7;
        uint8_t reg10 = 0xC0 | (uint8_t)(maxGain << 3) | 0x02;  // ALCSEL=stereo, MINGAIN=-12dB
        writeReg(ADCCONTROL10, reg10);
        writeReg(ADCCONTROL11, 0xF0);  // ALC target ≈ -1.5 dB, hold = 0
        writeReg(ADCCONTROL12, 0x05);  // attack / decay times
        writeReg(ADCCONTROL13, 0x06);  // ALC mode on, zero-cross
        writeReg(ADCCONTROL14, noiseGate ? 0x1B : 0x00);  // noise gate thr/type
    }

    // 0 dB … -96 dB. arg is dB attenuation as a positive number (0 = loudest).
    void setDacVolume(int dbAtten) {
        uint8_t v = clampVol(dbAtten);
        writeReg(DACCONTROL4, v);
        writeReg(DACCONTROL5, v);
    }
    void setAdcVolume(int dbAtten) {
        uint8_t v = clampVol(dbAtten);
        writeReg(ADCCONTROL8, v);
        writeReg(ADCCONTROL9, v);
    }

    // Analog output level 0–100 → LOUT1/ROUT1 (headphone) + LOUT2/ROUT2 (spk).
    void setOutputVolume(uint8_t pct) {
        if (pct > 100) pct = 100;
        uint8_t v = (uint8_t)((pct * 0x21) / 100);   // 0x21 ≈ full scale
        writeReg(DACCONTROL24, v);   // LOUT1
        writeReg(DACCONTROL25, v);   // ROUT1
        writeReg(DACCONTROL26, v);   // LOUT2 (speaker)
        writeReg(DACCONTROL27, v);   // ROUT2 (speaker)
    }

private:
    static uint8_t clampVol(int dbAtten) {
        if (dbAtten < 0)  dbAtten = 0;
        if (dbAtten > 96) dbAtten = 96;
        return (uint8_t)(dbAtten << 1);   // 0.5 dB steps
    }
};
