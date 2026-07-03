# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ✦ Project Overview

**Omniphone** is a modular, open electronic instrument platform using capacitive touch sensing (via MPR121 chips) for expressive, accessible music-making. The firmware is architecture-agnostic, supporting multiple microcontroller platforms and configurations ("variants") from a single monorepo.

**Status:** Active development — hardware/software in flux. Multiple hardware variants (RP2040 Pico, ESP32-S3, Teensy boards) coexist on `main` as tags; branches are for in-progress work.

---

## ✦ Monorepo Structure

```
.
├── platformio.ini              # Build targets (each [env:...] = one variant)
├── lib/                        # Shared libraries (MPR121, LCD drivers)
├── variants/                   # Per-instrument firmware & config
│   ├── pico-13pad/            # Raspberry Pi Pico (default, Mozzi)
│   ├── esp32s3-19pad/         # ESP32-S3 (I²S + MPR121)
│   ├── teensy40-11pad/        # Teensy 4.0 (original, Teensy Audio)
│   ├── teensy40-12pad-screen/ # Teensy 4.0 + 1.28" LCD + granular sampler
│   ├── teensy32-13pad/        # Teensy 3.2
│   └── teensy-lc-7voice/      # Teensy LC (standalone, internal DAC)
├── test/                       # Diagnostic/bring-up sketches (one .cpp each)
├── tools/                      # Build utilities (sample → header converter)
├── datasheets/                 # Hardware reference docs
└── src/                        # Placeholder (builds select variant from lib/)
```

### Key Principle

**Each variant is self-contained.** Every `variants/<name>/` folder contains a `config.h` (pin map, pad count, sensor setup) and at least one `.cpp` file (the instrument logic). The variant is selected via `platformio.ini` — its `build_src_filter` tells PlatformIO which `.cpp` to compile. Shared code lives in `lib/` and auto-compiles into every build.

---

## ✦ Build Commands

All commands use **PlatformIO** (`pio` or `platformio`). Environments are defined in `platformio.ini`.

### Build & Flash

```bash
# Build (no flash)
pio run -e pico-13pad
pio run -e esp32s3-19pad
pio run -e teensy40-11pad

# Build & flash over USB
pio run -e pico-13pad -t upload
pio run -e esp32s3-19pad -t upload

# OTA flash (WiFi, ESP32-S3 only; requires OMNIPHONE_OTA_PASSWORD env var)
pio run -e esp32s3-19pad-ota -t upload
```

### Monitor Serial Output

```bash
# While connected via USB (blocks; Ctrl+C to exit)
pio device monitor -e pico-13pad

# All output: building, flashing, serial
pio run -e pico-13pad -t upload && pio device monitor
```

### Device Info

```bash
# List available platforms/boards
pio platform list

# Show library dependency tree
pio run -e pico-13pad --verbose
```

---

## ✦ Variant Anatomy

Every variant folder (`variants/<name>/`) contains:

| File | Purpose |
|------|---------|
| `config.h` | **MANDATORY.** Pin mappings, pad count, sensor addresses (MPR121 I²C), sensor electrode → LED electrode map (SENSE_PADS), calibration defaults. Edit this to retarget a variant to new hardware. |
| `<name>.cpp` or `instrument.cpp` | Main firmware (sensor reading loop, audio/LED updates, WiFi/OTA if applicable). May have multiple `.cpp` files (diagnostic tests). |
| Other `.cpp` | Diagnostic sketches (e.g., `audio_test.cpp`, `mpr_sense_led_test.cpp`). Swapped in via `build_src_filter` for testing without touching the main instrument. |
| `.h` utilities | WiFi console, screen drivers, net utilities (e.g., `net_console.h`, `screen_status.h`). |

### Creating a New Variant

1. **Copy** the closest existing variant: `cp -r variants/pico-13pad variants/my-new-board`
2. **Edit** `config.h`:
   - Update pin constants (I²C, I²S, battery sensing, LEDs)
   - Adjust `SENSE_PADS[]` (which MPR121 electrode senses/drives each pad)
   - Set `NUM_SENSORS`, pad count, I²C addresses
3. **Add** `[env:my-new-board]` to `platformio.ini`:
   ```ini
   [env:my-new-board]
   platform = espressif32         # or: raspberrypi, teensy, etc.
   board = esp32-s3-devkitc-1     # or: pico, teensy40, etc.
   framework = arduino
   build_src_filter = -<*> +<../variants/my-new-board/>
   ```
4. **Build:** `pio run -e my-new-board`

---

## ✦ Audio & Sensing Architecture

### Sensing Loop (Firmware Core)

The typical variant's main loop:

1. **Read sensors** (I²C → MPR121 boards) — ~50–200 Hz depending on `SENSE_PERIOD_MS`
2. **Compute proximity** (raw filtered value → amplitude 0–1, with hysteresis/smoothing)
3. **Update voice amplitude** — per-pad envelope (fast attack, slow release to avoid zipper noise)
4. **Drive LEDs** (brightness = proximity, color = note/scale)
5. **Stream diagnostics** (Teleplot over UDP if enabled; CPU/memory overhead is high — opt-in)

### Audio Tasks

- **Teensy/Pico**: Interrupt-driven synth (Teensy Audio Library or Mozzi)
- **ESP32-S3**: FreeRTOS task, dual-core (sensing on Core 0, audio on Core 1), I²S DMA to DAC
- Most engines: **wavetable oscillators** (sine/saw/square/triangle) with linear interpolation + soft limiter (tanh) to avoid clipping on many-voice saturations

### Proximity Tuning

See `config.h` constants:
- `PROX_DEADBAND` — touches below this amplitude stay silent
- `PROX_MAX` — amplitude plateau (full volume)
- `PROX_ATTACK_K` / `PROX_RELEASE_K` — smoothing coefficients (avoid stepping artifacts)
- `CONFIRM_SCANS` — onset gate (strong deltas confirm fast; weak ones wait longer to reject EM spikes)

Tuning these live: most variants expose a web/JSON endpoint or Teleplot stream. See variant's `config.h` for `SensorSettings`.

---

## ✦ Common Development Tasks

### Adding a Note Layout or Timbre

**Note layouts** are in `config.h` as `static const uint8_t SET_*[NUM_SENSORS]` — MIDI note arrays laid out in the hex-grid shape.

**Timbres** are waveforms: edit the `TIMBRES[]` array and the waveform generator. For most variants, waveforms live in the audio task (sine/saw/square/triangle tables).

### Working with Samples (Teensy 12-pad-screen Only)

Sample-based playback (granular synthesis) is currently Teensy-only:

1. **Place** `.wav` files (mono or stereo, 44.1 kHz recommended) in `variants/teensy40-12pad-screen/samples/` with naming `<name>_<rootHz>.wav` (e.g., `hang_146.83.wav`)
2. **Run** `python3 tools/wav2header.py` — generates `variants/teensy40-12pad-screen/sample_data.h` (C arrays + metadata)
3. **Rebuild & flash**: `pio run -e teensy40-12pad-screen -t upload`

The root Hz lets the engine transpose samples in real-time.

### Testing a Diagnostic Sketch

Most variants have test sketches (e.g., `audio_test.cpp`, `mpr_sense_led_test.cpp`) that swap in via `build_src_filter` without editing `platformio.ini`:

```bash
# Temporarily build the audio test (same as its env: in platformio.ini)
pio run -e esp32s3-test -t upload
pio device monitor -e esp32s3-test

# Back to the main instrument
pio run -e esp32s3-19pad -t upload
```

### Debugging Capacitive Sensing

- **Teleplot** (free UDP-based plotter): Variants stream per-pad raw/filtered/baseline values. Set `TELEPLOT_HOST=<your-PC-IP>` in `config.h` or over HTTP.
- **Serial monitor**: Most variants log sensor state on USB (115200 baud default).
- **Web endpoints** (ESP32-only): JSON `/api/sensors` shows live readings; `POST /api/settings` tweaks CDC/CDT on-the-fly.

### Running Bring-up Tests

Diagnostic sketches in `test/` (e.g., `strike_tuning.cpp`, `proximity_tuning.cpp`, `sensor_comparison.cpp`) are built via `platformio.ini` entries that point `build_src_filter` at them:

```bash
pio run -e strike_tuning -t upload
pio run -e proximity_tuning -t upload
```

---

## ✦ Hardware & Pinouts

### Standard I²C (Sensing)

- **ESP32-S3**: GPIO 18 (SDA), 17 (SCL), 400 kHz (fast-mode). ⚠ Don't exceed 400 kHz with long wires; add pull-ups if needed.
- **Teensy 4.0**: I2C0 (SDA=18, SCL=19) or I2C1 (SDA=16, SCL=17). Fast-mode (400 kHz) typical.
- **Pico (RP2040)**: I2C0 (SDA=4, SCL=5) or I2C1 (SDA=26, SCL=27). 400 kHz.

### Standard I²S (Audio Out)

- **ESP32-S3**: BCK, LRCK, DIN routable to any GPIO (no fixed pins). Check variant's `config.h`.
- **Teensy 4.0**: I2S1 (BCK=21, LRCK=20, DATA=7 to codec) — fixed by Teensy Audio Library.
- **Pico**: PIO-driven (flexible pins); see variant's audio driver.

### MPR121 Addressing

All I²C slaves, addressed via hardware strap pins:
- `0x5A` = ADDR→GND
- `0x5B` = ADDR→VCC
- `0x5C` = ADDR→SDA
- `0x5D` = ADDR→SCL

Every variant can have up to 4 chips (A–D). The `config.h` `SENSE_PADS[]` map specifies which chip's electrode senses/drives each pad.

---

## ✦ Key Files & Conventions

| File | What It Tells You |
|------|-------------------|
| `platformio.ini` | All build environments & their source/flags. Start here to find which `.cpp` file builds for which board. |
| `variants/*/config.h` | Entire hardware definition (pins, pad topology, MPR121 addresses, calibration). Editing this retargets the firmware to new hardware. |
| `lib/MPR121/MPR121.h` | I²C register access, initialization, baseline/delta reads. Used by all variants. |
| `README.md` | Overview, variant table, philosophy. |
| `.gitignore` | Secrets (WiFi, OTA password) in `secrets.h` are NOT committed. Copy `secrets.example.h` → `secrets.h`. |
| `.claude/settings.local.json` | Project-level Claude Code permissions (allows `pio run`, `git` ops, `Read` on project). |

---

## ✦ Workflow Notes

### Branches & Releases

- **Main**: Always stable; ready-to-run. Never park work branches here.
- **Branches**: For in-progress features/experiments. Merge back when ready.
- **Tags**: Freeze released instruments (e.g., `v1.0-pico-13pad`, `v2.0-esp32s3-19pad`). Use tags instead of branches to archive finished configs.

### Backporting Improvements

If you fix a bug in `lib/MPR121/` or `lib/1.28inch_Touch_LCD/`, it applies to **all** variants automatically on next build. Test at least one variant per platform (e.g., Pico, Teensy, ESP32-S3) if the change is sensing/audio related.

### Working with Shared Code

Refactoring question: **"Should this live in `lib/` or `variants/<name>/`?"**
- **Move to `lib/`** if: multiple variants need it, or it's a reusable driver/utility (MPR121, LCD, WiFi console).
- **Keep in `variants/<name>/`** if: board-specific (pinout, calibration, one-off UI).

---

## ✦ Quick Reference: Add a Pad or Change a Pin

1. **New hardware config**: Edit `variants/<name>/config.h`
   - `NUM_SENSORS` — pad count
   - `SENSE_PADS[]` — which chip & electrode per pad
   - Pin constants (I²C, I²S, LED, battery)
2. **New note layout**: Add `SET_<name>[NUM_SENSORS]` array (MIDI notes or Hz)
3. **Rebuild**: `pio run -e <variant> -t upload`

---

## ✦ Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| **No audio** | I²S wiring (LRCK pin, DAC power), or I²S not started | Check I²S init in variant's `.cpp`; see `audio_test.cpp` |
| **Serial output @ wrong baud** | Variant uses non-standard speed | Check `monitor_speed` in `platformio.ini` for that env |
| **Pads unresponsive** | MPR121 not initialized, or I²C clock too fast (>400 kHz) | Verify I²C clock in `config.h`; add pull-ups if long wires |
| **Stuck/hanging tone** | Baseline drift or hysteresis tuning too tight | Adjust `PROX_RELEASE_K`, `HEAL_K`, `HOLD_MAX_SCANS` in `config.h` |
| **Build fails (lib not found)** | `lib/` deps not indexed by PlatformIO | Run `pio run -e <variant>` (not just `compile`) to regenerate cache |
| **OTA upload hangs** | Password mismatch or wrong hostname | Check `OMNIPHONE_OTA_PASSWORD` env var & `OTA_HOSTNAME` in `platformio.ini` |

---

## ✦ Related Resources

- **PlatformIO**: https://docs.platformio.org (build, monitor, OTA)
- **Datasheets**: `datasheets/` folder (MPR121, DAC, boards)
- **Memory usage**: Variants with many features (WiFi, LCD, audio) may approach flash/RAM limits. Monitor with `pio run --verbose` and check `.pio/build/<env>/size_report_*.txt`
