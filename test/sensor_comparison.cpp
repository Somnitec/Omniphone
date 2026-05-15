#include <Arduino.h>
#include <Wire.h>
#include <MPR121.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sensor Comparison Test
//
// Compares 4 physical sensor layouts on a single MPR121 board to find which
// gives the strongest proximity response at the greatest distance.
//
// Electrodes:
//   ELE0 — wire + copper tape around housingg
//   ELE1 — copper tape only
//   ELE2 — wire soldered on metal LED housin
//   ELE3 — conductive PLA pad
//
// Each electrode can have its own CDC (charge current) and CDT (charge time)
// via per-electrode registers. The test starts with identical settings and
// lets you tweak each one via serial commands.
//
// Output: tab-separated columns at ~20 Hz for easy plotting (Arduino Serial
// Plotter or external tool). Shows raw delta (baseline − filtered) and the
// EMA-smoothed delta for each sensor.
//
// Serial commands:
//   c<electrode><value>  — set per-electrode CDC (charge current 0–63)
//                          e.g. "c032" sets ELE0 CDC to 32
//   t<electrode><value>  — set per-electrode CDT (charge time 0–7)
//                          e.g. "t13"  sets ELE1 CDT to 3
//   r                    — reset all to defaults and re-init
//   p                    — print current per-electrode settings
// ─────────────────────────────────────────────────────────────────────────────

// ── Configuration ────────────────────────────────────────────────────────────

static constexpr uint8_t BOARD_ADDR     = 0x5C;
static constexpr uint8_t NUM_ELECTRODES = 4;

// Per-electrode register addresses (MPR121 datasheet section 5.4)
// CDC: 0x5F + electrode (one byte each, 6-bit value)
// CDT: 0x6C + electrode/2 (packed, 3-bit value per nibble)
static constexpr uint8_t REG_CDC0 = 0x5F; // per-electrode charge current
static constexpr uint8_t REG_CDT0 = 0x6C; // per-electrode charge time (packed pairs)

static const char* SENSOR_NAMES[NUM_ELECTRODES] = {
    "Wire+LED",
    "CuTape",
    "Wire+LED+Cu",
    "ConductPLA",
};

// ── Per-electrode tuning (modifiable at runtime via serial) ──────────────────

static uint8_t eleCDC[NUM_ELECTRODES] = { 10, 10, 10, 10 }; // 0–63 (µA)
static uint8_t eleCDT[NUM_ELECTRODES] = { 2,  2,  2,  2  }; // 0–7 (encoding)

// ── Baseline tracking mode ───────────────────────────────────────────────────
// false = MPR121 auto-tracks baseline (drifts toward hand over time)
// true  = baseline frozen after initial load (stable but no drift compensation)
static bool baselineFrozen = false;

// ── State ────────────────────────────────────────────────────────────────────

static MPR121 board(BOARD_ADDR);

static float emaFast[NUM_ELECTRODES] = {0};
static float emaSlow[NUM_ELECTRODES] = {0};
static float peakDelta[NUM_ELECTRODES] = {0};
static constexpr float EMA_ALPHA_FAST = 0.3f;
static constexpr float EMA_ALPHA_SLOW = 0.01f;

static constexpr uint32_t UPDATE_MS = 8;   // 125 Hz read rate
static constexpr uint32_t PRINT_MS  = 50;  // 20 Hz output rate
static uint32_t lastUpdateMs = 0;
static uint32_t lastPrintMs  = 0;

// ── ECR helper — restarts with correct baseline tracking mode ────────────────
// CL bits in ECR register (bits 7:6):
//   00 = baseline tracking disabled (frozen)
//   01 = baseline initialized from first measurement, then tracks
//   10 = load from initial measurement (same as 01 in practice)
//   11 = load from current filtered data, then tracks
static void restartECR()
{
    uint8_t cl = baselineFrozen ? 0b00000000 : 0b11000000;
    board.write(0x5E, static_cast<uint8_t>(cl | NUM_ELECTRODES));
}

// ── Per-electrode CDC/CDT register writes ────────────────────────────────────

static void writePerElectrodeCDC()
{
    // Must stop the chip to write config registers
    board.write(0x5E, 0x00); // ECR = stop
    delay(5);

    for (uint8_t i = 0; i < NUM_ELECTRODES; i++)
    {
        board.write(REG_CDC0 + i, eleCDC[i] & 0x3F);
    }

    // Re-enable with per-electrode CDC active:
    // CDC_CFG bit 7:6 = FFI, bit 5:0 = global CDC (set to 0 to use per-electrode)
    // When global CDC=0 and per-electrode CDC>0, per-electrode takes precedence.
    board.write(0x5C, 0b00000000); // FFI=00, global CDC=0 → per-electrode used
    restartECR();
    delay(50);
}

static void writePerElectrodeCDT()
{
    board.write(0x5E, 0x00); // stop
    delay(5);

    // CDT registers pack two electrodes per byte: ELE(2n) in low nibble, ELE(2n+1) in high
    for (uint8_t i = 0; i < NUM_ELECTRODES; i += 2)
    {
        uint8_t lo = eleCDT[i] & 0x07;
        uint8_t hi = (i + 1 < NUM_ELECTRODES) ? (eleCDT[i + 1] & 0x07) : 0;
        board.write(REG_CDT0 + i / 2, (hi << 4) | lo);
    }

    // CDT_CFG: set global CDT=0 so per-electrode values take effect
    // Keep SFI=4 (01), ESI=2ms (001)
    board.write(0x5D, 0b00000001); // CDT=0, SFI=00(4), ESI=001(2ms)
    restartECR();
    delay(50);
}

// ── Init ─────────────────────────────────────────────────────────────────────

static void initBoard()
{
    board.begin(NUM_ELECTRODES, 40, 20);

    // Reset EMA state
    for (uint8_t i = 0; i < NUM_ELECTRODES; i++)
    {
        uint16_t filt = board.filteredData(i);
        uint16_t base = board.baselineData(i);
        int16_t d = (int16_t)base - (int16_t)filt;
        float v = d < 0 ? 0.0f : (float)d;
        emaFast[i] = v;
        emaSlow[i] = v;
        peakDelta[i] = 0.0f;
    }

    // Apply per-electrode tuning
    writePerElectrodeCDC();
    writePerElectrodeCDT();
}

// ── Serial command parser ────────────────────────────────────────────────────

static void handleSerial()
{
    if (!Serial.available()) return;

    char cmd = Serial.read();
    delay(10); // let rest of command arrive

    if (cmd == 'r' || cmd == 'R')
    {
        // Reset all to defaults
        for (uint8_t i = 0; i < NUM_ELECTRODES; i++)
        {
            eleCDC[i] = 10;
            eleCDT[i] = 2;
            peakDelta[i] = 0.0f;
        }
        initBoard();
        Serial.println(F("# Reset to defaults"));
        return;
    }

    if (cmd == 'b' || cmd == 'B')
    {
        // Force baseline reload — re-locks baseline to current filtered value,
        // then restarts in current mode (frozen or tracking).
        board.write(0x5E, 0x00); // stop
        delay(10);
        // First load with CL=11 to capture current filtered data as baseline
        board.write(0x5E, static_cast<uint8_t>(0b11000000 | NUM_ELECTRODES));
        delay(100);
        // Then restart in the active mode (frozen keeps it locked, tracking lets it drift)
        if (baselineFrozen) {
            board.write(0x5E, 0x00);
            delay(5);
            restartECR();
            delay(50);
        }
        Serial.println(F("# Baseline reloaded"));
        return;
    }

    if (cmd == 'f' || cmd == 'F')
    {
        baselineFrozen = !baselineFrozen;
        // Reload baseline from current filtered, then apply the new mode
        board.write(0x5E, 0x00);
        delay(10);
        board.write(0x5E, static_cast<uint8_t>(0b11000000 | NUM_ELECTRODES));
        delay(100);
        board.write(0x5E, 0x00);
        delay(5);
        restartECR();
        delay(50);
        Serial.print(F("# Baseline tracking: "));
        Serial.println(baselineFrozen ? F("FROZEN (stable, no drift compensation)")
                                      : F("AUTO (drifts but compensates environment)"));
        return;
    }

    if (cmd == 'p' || cmd == 'P')
    {
        Serial.println(F("# Current settings:"));
        for (uint8_t i = 0; i < NUM_ELECTRODES; i++)
        {
            Serial.print(F("#   ELE"));
            Serial.print(i);
            Serial.print(F(" ("));
            Serial.print(SENSOR_NAMES[i]);
            Serial.print(F(") CDC="));
            Serial.print(eleCDC[i]);
            Serial.print(F(" CDT="));
            Serial.print(eleCDT[i]);
            Serial.print(F(" peak="));
            Serial.println(peakDelta[i], 1);
        }
        return;
    }

    if (cmd == '0')
    {
        // Reset peak tracking
        for (uint8_t i = 0; i < NUM_ELECTRODES; i++)
            peakDelta[i] = 0.0f;
        Serial.println(F("# Peaks reset"));
        return;
    }

    if (cmd == 'c' || cmd == 'C' || cmd == 't' || cmd == 'T')
    {
        // Read electrode index (single digit)
        while (!Serial.available()) {}
        char eChar = Serial.read();
        uint8_t ele = eChar - '0';
        if (ele >= NUM_ELECTRODES) {
            Serial.println(F("# Invalid electrode"));
            return;
        }

        // Read value (up to 3 digits)
        String valStr = "";
        while (Serial.available()) {
            char c = Serial.read();
            if (c >= '0' && c <= '9') valStr += c;
            delay(2);
        }
        uint8_t val = valStr.toInt();

        if (cmd == 'c' || cmd == 'C') {
            if (val > 63) val = 63;
            eleCDC[ele] = val;
            writePerElectrodeCDC();
            Serial.print(F("# ELE"));
            Serial.print(ele);
            Serial.print(F(" CDC="));
            Serial.println(val);
        } else {
            if (val > 7) val = 7;
            eleCDT[ele] = val;
            writePerElectrodeCDT();
            Serial.print(F("# ELE"));
            Serial.print(ele);
            Serial.print(F(" CDT="));
            Serial.println(val);
        }
        return;
    }

    Serial.println(F("# Commands: c<ele><val> t<ele><val> f b r p 0"));
}

// ── Setup ────────────────────────────────────────────────────────────────────

// Force USB Full-Speed (12 Mbps) instead of High-Speed (480 Mbps).
// Runs immediately after usb_init() via weak-symbol override, before the host
// has time to enumerate — so the HS handshake never happens.
extern "C" void startup_late_hook(void) {
    USB1_PORTSC1 |= USB_PORTSC1_PFSC;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Wire.begin();
    Wire.setClock(400000);

    Serial.println(F("# ─── Sensor Comparison Test ───"));
    Serial.println(F("# ELE0: Wire+LED  ELE1: CuTape   ELE2: Wire+LED+CuTape ELE3: ConductPLA"));
    Serial.println(F("# Commands: c<ele><val>(CDC) t<ele><val>(CDT) f(freeze) b(baseline) r(reset) p(print) 0(peaks)"));
    Serial.println(F("#"));

    initBoard();

    // Print header for serial plotter / CSV
    Serial.println(F("# filt0\tbase0\tdelta0\tema0\tfilt1\tbase1\tdelta1\tema1\tfilt2\tbase2\tdelta2\tema2\tfilt3\tbase3\tdelta3\tema3"));
}

// ── Loop ─────────────────────────────────────────────────────────────────────

void loop()
{
    uint32_t now = millis();

    handleSerial();

    if (now - lastUpdateMs < UPDATE_MS)
        return;
    lastUpdateMs = now;

    // Burst-read filtered + baseline
    uint8_t filtBuf[8]; // 4 electrodes × 2 bytes
    uint8_t baseBuf[4]; // 4 electrodes × 1 byte
    board.burstRead(MPR121Reg::FILT_0L, filtBuf, 8);
    board.burstRead(MPR121Reg::BASE_0,  baseBuf, 4);

    static float rawDelta[NUM_ELECTRODES];
    static uint16_t rawFilt[NUM_ELECTRODES];
    static uint16_t rawBase[NUM_ELECTRODES];

    for (uint8_t i = 0; i < NUM_ELECTRODES; i++)
    {
        rawFilt[i] = (uint16_t)filtBuf[2 * i]
                   | ((uint16_t)(filtBuf[2 * i + 1] & 0x03) << 8);
        rawBase[i] = (uint16_t)baseBuf[i] << 2;

        float d = (float)((int16_t)rawBase[i] - (int16_t)rawFilt[i]);
        if (d < 0.0f) d = 0.0f;
        rawDelta[i] = d;

        // EMA tracking
        float alpha = (d > emaFast[i]) ? EMA_ALPHA_FAST : 0.15f;
        emaFast[i] = alpha * d + (1.0f - alpha) * emaFast[i];
        emaSlow[i] = EMA_ALPHA_SLOW * d + (1.0f - EMA_ALPHA_SLOW) * emaSlow[i];

        // Peak tracking
        if (emaFast[i] > peakDelta[i])
            peakDelta[i] = emaFast[i];
    }

    // ── Print at 20 Hz ───────────────────────────────────────────────────────
    if (now - lastPrintMs >= PRINT_MS)
    {
        lastPrintMs = now;

        for (uint8_t i = 0; i < NUM_ELECTRODES; i++)
        {
            Serial.print(rawFilt[i]);
            Serial.print(F("\t"));
            Serial.print(rawBase[i]);
            Serial.print(F("\t"));
            Serial.print(rawDelta[i], 1);
            Serial.print(F("\t"));
            Serial.print(emaFast[i], 2);
            if (i < NUM_ELECTRODES - 1) Serial.print(F("\t"));
        }
        Serial.println();

        // Periodic peak summary every 5 seconds
        static uint32_t lastSummaryMs = 0;
        if (now - lastSummaryMs >= 5000)
        {
            lastSummaryMs = now;
            Serial.print(F("# PEAKS: "));
            for (uint8_t i = 0; i < NUM_ELECTRODES; i++)
            {
                Serial.print(SENSOR_NAMES[i]);
                Serial.print(F("="));
                Serial.print(peakDelta[i], 1);
                if (i < NUM_ELECTRODES - 1) Serial.print(F("  "));
            }
            Serial.println();
        }
    }
}
