# Omniphone — Raspberry Pi Pico port (branch `13-pad-raspi-pico`)

13-pad, no touch screen, Raspberry Pi Pico (RP2040), **Mozzi** audio backend.
Default sound: **Juno Harmonic Minor**. A future ESP32-S3 (LilyGO T-FPGA) build
shares this same firmware.

## Why a new audio engine
The original firmware's synthesis is built entirely on the **Teensy Audio
Library** (`AudioSynthWaveformModulated`, `AudioFilterStateVariable`,
`AudioMixer4`, `AudioOutputI2S`, …). That library is DMA/hardware-specific to
Teensy and **does not run on the RP2040 or ESP32-S3**. So the synth + audio
output layer is re-implemented with **Mozzi**, which supports Teensy, RP2040 and
ESP32-S3 from one codebase. Everything else is shared unchanged:
- `config.h` — scales, sound sets, 13-pad layout, tuning constants
- `proximity_engine.h` — capacitive proximity/touch math (pure C++)
- `lib/MPR121` — the sensor driver (I2C, portable)

| File | Role |
| --- | --- |
| `src/main_pico.cpp` | Pico/ESP32-S3 firmware (Mozzi). Built by `env:omniphone-pico` / `env:omniphone-esp32-s3`. |
| `src/sound_engine_mozzi.h` | Mozzi polyphonic synth voices + global filter. |
| `src/main.cpp` | Legacy Teensy 4.0 firmware (Teensy Audio Lib). Built by `env:omniphone-teensy40`. |
| `src/config.h` | Shared. 13-pad layout is `#if OMNIPHONE_PICO/ESP32S3`, else legacy 11-pad. |

## Build & flash
```bash
pio run -e omniphone-pico              # build (first run downloads the RP2040 core + Mozzi)
pio run -e omniphone-pico -t upload    # hold BOOTSEL on first flash, then it's automatic
pio device monitor -e omniphone-pico   # serial; send '0'..'6' to switch sound sets
```
`default_envs = omniphone-pico`, so a bare `pio run` builds the Pico. Teensy is
still `pio run -e omniphone-teensy40`.

## Pin map — dropping a Pico onto the Teensy 4.0 PCB
The PCB was laid out for a Teensy 4.0; the Pico has a different pinout, so you
re-point each **signal net** to a Pico GPIO. All of these live at the top of
`src/main_pico.cpp` (I2C + the `MOZZI_I2S_PIN_*` block) and in `SENSORS[]` in
`config.h` (the LED GPIOs). Change a number, re-flash — nothing else.

| Signal | Teensy 4.0 pin (old trace) | Pico GPIO (new) | Notes |
| --- | --- | --- | --- |
| I2C SDA | 18 | **GP4** | `PIN_I2C_SDA` |
| I2C SCL | 19 | **GP5** | `PIN_I2C_SCL` (must be a valid SCL for the SDA's I2C block) |
| I2S BCLK | 21 | **GP20** | `MOZZI_I2S_PIN_BCK` |
| I2S LRCLK/WS | 20 | **GP21** | `MOZZI_I2S_PIN_WS` — **must be BCLK+1** on RP2040 PIO |
| I2S DIN (data) | 7 | **GP22** | `MOZZI_I2S_PIN_DATA` → PCM5102A DIN |
| I2S MCLK/SCK | 23 | *(none)* | RP2040 PIO I2S sends no MCLK — strap PCM5102A SCK to GND |
| LED: top pad | a free GPIO net | **GP14** | `SENSORS[0].ledEle`, `LED_GPIO` |
| LED: upper ring 5 | a free GPIO net | **GP15** | `SENSORS[6].ledEle`, `LED_GPIO` |

Notes / gotchas:
- **The Pico and Teensy 4.0 footprints differ** (pin count and order), so this is
  not a mechanical drop-in — you map nets to the GPIOs above, you don't expect
  the same physical holes to line up. Double-check power: feed the Pico 5V on
  **VSYS** (not the Teensy VIN position) and share grounds.
- Both MCUs are **3.3 V** logic — the MPR121s and PCM5102A wiring is unchanged.
- The 11 LEDs on MPR121 driver pins are unchanged; only the 2 `LED_GPIO` lamps
  move to Pico GP14/GP15. Reassign in `SENSORS[]` if your board routes them
  elsewhere.
- Pick the I2C/I2S GPIOs to match whatever your PCB actually exposes near the
  Pico — the numbers above are a valid, conflict-free default, not a constraint.

## 13-pad layout (confirm against your PCB)
`config.h` → `SENSORS[]` (the `OMNIPHONE_PICO` branch). 13 sense electrodes:
- **Board C (0x5C):** top + 6 upper-ring pads → ELE0–ELE6 (7 sense)
- **Board A (0x5A):** 6 lower-ring pads → ELE0–ELE5 (6 sense)

LEDs: 11 on MPR121 driver pins (A: ELE6–11, C: ELE7–11) + 2 on Pico GPIO. Reorder
rows to match physical ring positions; keep the electrode↔board split.

## Tuning the Mozzi engine
- `sound_engine_mozzi.h` uses **one global** resonant low-pass (cutoff follows
  the loudest pad) instead of per-voice filters, to fit the RP2040 CPU budget.
  Mozzi's cheap `LowPassFilter` cutoff is an 8-bit value, so `filterBaseHz` /
  `filterMaxHz` from `config.h` are mapped approximately by `hzToCutoff()` — tune
  the two endpoints by ear.
- If audio glitches (CPU starved), define `MOZZI_NO_SUB` in the env's
  `build_flags` to drop the 13 sub-oscillators (halves the oscillator count).
- `MOZZI_AUDIO_RATE` / `MOZZI_CONTROL_RATE` are at the top of `main_pico.cpp`.

## No audio? (Teensy→Pico DAC bring-up)
The Mozzi I2S config is verified correct (I2S_DAC on GP20/21/22, overriding the
RP2040 PWM default), so silence is almost always wiring. Isolate it first:
- Set `#define PICO_AUDIO_TEST 1` in `main_pico.cpp`, reflash. This plays a steady
  440 Hz tone and ignores the sensors.
  - **Hear it** → audio path is good; the problem is *sensing* (electrode /
    `SENSE_ELECTRODES` config — voices only open when a pad reads proximity).
  - **Silent** → it's the I2S/DAC wiring below.

PCM5102A checklist (the usual Teensy→Pico gotchas):
1. **SCK → GND.** The Teensy fed the DAC a master clock (MCLK) on Teensy pin 23.
   The RP2040 PIO I2S sends **no MCLK**, so the PCM5102A's SCK pin must be tied to
   GND to use its internal PLL. On the old PCB that trace is now dead → DAC silent.
   This is the #1 cause.
2. **Pins must match the traces.** BCK→GP20, LRCK/WS→GP21, DIN→GP22. If the board
   routes them to other Pico pins, change `MOZZI_I2S_PIN_BCK` / `MOZZI_I2S_PIN_DATA`
   (WS is auto = BCK+1). **BCK and LRCK must be consecutive GPIOs** (PIO constraint).
3. **XSMT (soft-mute) high.** If the DAC's XSMT pin is low it's muted — it needs
   ~3.3 V (often via a resistor). **FMT → GND** selects I2S format.

## Not yet ported (clearly marked TODOs)
- **Metal-contact "bell" voice** — `updateProximity()` still returns the
  `isTouch` strike flag; `main_pico.cpp` has a `TODO(bell)` where a Mozzi bell
  voice + velocity dynamics would hook in.
- **USB-MIDI / MPE output** — the Teensy build sends MPE via `usbMIDI`. On the
  Pico this needs Adafruit TinyUSB MIDI; not wired yet.
- **ESP32-S3 (LilyGO T-FPGA)** — `env:esp32s3` builds the same `main_pico.cpp`;
  finish the `OMNIPHONE_ESP32S3` I2S pin block and confirm the board variant.

## Status
Structure, config, engine and build setup are in place. The Pico/ESP32 targets
have **not been hardware-compiled or flashed here** (no RP2040 toolchain in this
session) — the first `pio run -e pico` on your machine is the real compile pass.
The Teensy build was kept intact.
