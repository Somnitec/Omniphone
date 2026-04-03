
#include "config.h"
#include "proximity_engine.h"
#include <MPR121.h>

// ─────────────────────────────────────────────────────────────────────────────
// Hardware
// ─────────────────────────────────────────────────────────────────────────────

// One MPR121 instance per board defined in config.h.
// The array is initialised in setup() because MPR121() takes a reference.
static MPR121 boards[2] = {
    MPR121(BOARD_ADDRESSES[0]),
    MPR121(BOARD_ADDRESSES[1]),
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-sensor state
// ─────────────────────────────────────────────────────────────────────────────

static SensorState sensorState[NUM_SENSORS];
static ProximityConfig proxCfg; // uses struct default values from proximity_engine.h

//already defined in config.h static constexpr uint32_t UPDATE_MS = 8; // ~125 Hz sensor update rate
static uint32_t lastUpdateMs = 0;



// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000)
    {
    }

    // ── I²C bus ───────────────────────────────────────────────────────────────
    Wire.begin();
    Wire.setClock(400000); // 400 kHz — ensure 4.7 kΩ pull-ups on SDA/SCL

    // ── MPR121 boards ─────────────────────────────────────────────────────────
    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        if (!boards[b].begin())
        {
            Serial.print(F("ERROR: MPR121 board "));
            Serial.print(b);
            Serial.print(F(" at 0x"));
            Serial.print(BOARD_ADDRESSES[b], HEX);
            Serial.println(F(" not found — check address and wiring"));
        }
        else
        {
            Serial.print(F("MPR121 board "));
            Serial.print(b);
            Serial.println(F(" OK"));
        }
        boards[b].beginLEDs();
    }

    // ── Seed per-sensor EMA state ─────────────────────────────────────────────
    // Read the actual initial delta for each pad so the jump detector
    // has a realistic baseline and does not fire spuriously on the first frame.
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];
        uint16_t filt = boards[sc.boardIndex].filteredData(sc.electrode);
        uint16_t base = boards[sc.boardIndex].baselineData(sc.electrode);
        int16_t raw = (int16_t)base - (int16_t)filt;
        seedSensorState(sensorState[i], raw < 0 ? 0.0f : (float)raw);
    }

    Serial.println(F("# Sensor test started"));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────

void loop()
{
    uint32_t now = millis();
    if (now - lastUpdateMs < UPDATE_MS)
        return;
    lastUpdateMs = now;

    // ── Burst-read all boards ─────────────────────────────────────────────────
    // Reading filtered, baseline and touch registers in three back-to-back burst
    // reads per board minimises I²C time and keeps all electrode reads coherent
    // within the same scan cycle.
    struct BoardData
    {
        uint8_t filt[12]; // ELE0–ELE5 filtered: 2 bytes each (10-bit little-endian)
        uint8_t base[6];  // ELE0–ELE5 baseline: 1 byte each (8 MSBs)
        uint16_t touch;   // bitmask: bit i set if electrode i is touched
    };
    static BoardData bd[NUM_BOARDS];

    for (uint8_t b = 0; b < NUM_BOARDS; b++)
    {
        boards[b].burstRead(MPR121Reg::FILT_0L, bd[b].filt, 12);
        boards[b].burstRead(MPR121Reg::BASE_0, bd[b].base, 6);
        bd[b].touch = boards[b].touchStatus();
    }

    // ── Per-sensor update ─────────────────────────────────────────────────────
    // LED brightness values are accumulated here; a single setAllLEDs() call
    // per board is issued after the loop. This cuts LED I²C transactions from
    // 3 per sensor (read-modify-write) down to ~5 per board regardless of how
    // many sensors are active, which is the main cause of audio crackling.

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        const SensorConfig &sc = SENSORS[i];
        const BoardData &b = bd[sc.boardIndex];
        uint8_t e = sc.electrode;

        // Reconstruct 10-bit values from the burst buffers.
        uint16_t filtered = (uint16_t)b.filt[2 * e] | ((uint16_t)(b.filt[2 * e + 1] & 0x03) << 8);
        uint16_t baseline = (uint16_t)b.base[e] << 2;

        Serial.print(i);
        Serial.print(F("filt:"));
        Serial.print(filtered);
        Serial.print(F(" "));
        Serial.print(i);
        Serial.print(F("base:"));
        Serial.print(baseline);
        Serial.print(F(" "));
    }
    Serial.println();

}
