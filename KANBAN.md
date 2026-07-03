# Omniphone Kanban & Status Board

Single comprehensive file: all tasks, per-instrument status, and experimental lines.

---

## Quick Reference: Instruments & Experimental Lines

### Instruments by Shell

| Shell | Board | Pads | Status | Active Work | Next |
|-------|-------|------|--------|-------------|------|
| 🎋 Bamboo 12-pad screen | Teensy 4.0 | 12 | **🔥 In Progress** | Sensor tuning, harmonic journey | Test with speaker? |
| ⚫ Black 11-pad | Teensy 4.0 | 11 | ⏸ Backlog | Replace board decision | Decide: Pico or ESP32? |
| ⚫ Black 13-pad | Pico | 13 | 📋 Todo | Get cap-touch resistors | Test tuning |
| ⚪ White 7-pad | Teensy LC | 7 | 📋 Todo | Test pad LEDs | Verify feasibility |
| ⚪ White 19-pad screen | ESP32-S3 | 19 | 📋 Todo | Sound design options | Implement granular? |
| 🎵 ESP32 Audio Kit | ESP32-S3 | ? | 📋 Todo | Max voice count test, case design | Finalize audio engine |
| ✨ Textured white shell | Pico | 13 | 📋 Todo | Install + cap-sense tuning | Screen parity? |
| 🛡️ Teensy 3.2 + Shield | Teensy 3.2 | 13+ | 📋 Todo | Speaker + enclosure design | CAD finalize |
| 🪵 Printed wooden 12-pad screen | TBD | 12 | ⏸ Backlog | Screen fit, board choice | Architecture decision |

### Experimental Lines (Cross-Cutting)

| Line | Instruments | Status | Next |
|------|-------------|--------|------|
| 🔊 **Speaker Design** | Teensy 3.2 + Shield, ESP32 Audio Kit | Design phase | Sketch enclosures, test drivers |
| 📍 **Pico Cap-Sense Tuning** | Black 13-pad, Textured white | 🔌 Resistors pending | Install + tune, compare vs MPR121 |
| 🎛️ **ESP32-S3 Native Touch** | Black 13-pad, White 19-pad | Design phase | Validate on black-13pad first |
| 🎹 **Many-Pad MPR121** | Teensy 4.0 (future) | Planned | Awaits board consolidation |
| 🥁 **Hang Drum Sound Design** | All (primary: Bamboo, White 19-pad, Audio Kit) | Early | Test on multiple boards, refine timbre |

---

## 🔥 In Progress

### Bamboo 12-Pad Screen (Teensy 4.0)
**Board:** Teensy 4.0 | **Pads:** 12 (MPR121) | **Screen:** 1.28" LCD  
**Epics:** Hang drum sound design

**Current Work:**
- Tweak MPR121 sensor code — Refine `PROX_DEADBAND`, `PROX_ATTACK_K`, `PROX_RELEASE_K` tuning. [config.h](variants/teensy40-12pad-screen/config.h) | [firmware](variants/teensy40-12pad-screen/teensy40-12pad-screen.cpp)
- Harmonic journey sound design — Sample playback + timbre morphing on screen. [sample_data.h](variants/teensy40-12pad-screen/sample_data.h)

**Build & Flash:**
```bash
pio run -e teensy40-12pad-screen -t upload
pio device monitor -e teensy40-12pad-screen
```

**Next Steps:**
- [ ] Finalize sensor tuning on bamboo shell
- [ ] Test harmonic journey with samples
- [ ] Evaluate hang drum sound for other shells
- [ ] Consider speaker add-on

---

## 📋 Todo

### Black 13-Pad (Pico) — Cap-Sense Validation
**Board:** Pico (RP2040) | **Pads:** 13 (bare-pin PIO cap-sense, no MPR121)  
**Epics:** Pico cap-sense tuning, ESP32-S3 native touch (comparison)  
**Status:** 🔌 Hardware pending (resistors ordered)

**Current Work:**
- Get resistors for cap-touch — Procure ~1M resistors (exact value TBD). [config.h](variants/pico-13pad/config.h) | Memory: [[Pico PIO capsense]] Mode 3 PIO guarded scan
- Install Pico + tune cap-sense — Install in shell, refine CDC/CDT on resistive surface. Measure latency vs MPR121.
- Test on textured surfaces — Validate consistency across shell texture variations.

**Build & Flash:**
```bash
pio run -e pico-13pad -t upload
pio device monitor -e pico-13pad
```

**Next Steps:**
- [ ] Procure 1M resistors
- [ ] Install Pico in black-13pad shell
- [ ] Tune CDC/CDT on resistive surface
- [ ] Measure latency (touch → ADC)
- [ ] Compare vs MPR121 baseline + ESP32-S3 native
- [ ] Document final tuning in config.h

---

### White 7-Pad (Teensy LC) — LED Testing
**Board:** Teensy LC | **Pads:** 7 (TSI capacitive touch, internal DAC)  
**Epics:** —

**Current Work:**
- Test pad LEDs — Investigate GPIO availability for LED control. Check conflict with TSI scanner. [config.h](variants/teensy-lc-7voice/config.h) | Memory: [[Teensy LC 7-voice build]]

**Build & Flash:**
```bash
pio run -e teensy-lc-7voice -t upload
pio device monitor -e teensy-lc-7voice
```

**Next Steps:**
- [ ] Test LED control on available GPIO
- [ ] Verify LED brightness doesn't interfere with TSI
- [ ] Document GPIO allocation in config.h
- [ ] Prototype LED feedback (brightness = proximity)

**Notes:**
- Pins 15/23 side of board known electrically noisy
- Simplest variant: no I²C, no I²S, just TSI + DAC

---

### White 19-Pad Screen (ESP32-S3) — Sound Design
**Board:** ESP32-S3 | **Pads:** 19 (MPR121 I²C) | **Screen:** 1.28" LCD  
**Audio:** I²S DMA to DAC, dual-core (Core 0=sensing, Core 1=audio)  
**Epics:** Hang drum sound design, ESP32-S3 native touch (future alternative)

**Current Work:**
- Implement other sound design options — Explore granular, wavetable morphing, FM, sample playback. [firmware](variants/esp32s3-19pad/esp32s3-19pad.cpp) | [config.h](variants/esp32s3-19pad/config.h)
- Touch-responsive timbre control — Screen UI for parameter tweaking.

**Build & Flash:**
```bash
pio run -e esp32s3-19pad -t upload
pio device monitor -e esp32s3-19pad
# OTA (WiFi): OMNIPHONE_OTA_PASSWORD=<pwd> pio run -e esp32s3-19pad-ota -t upload
```

**Next Steps:**
- [ ] Decide on primary sound engine (granular vs FM vs wavetable)
- [ ] Implement + test selected option
- [ ] Compare hang drum character to Bamboo
- [ ] Iterate timbre library
- [ ] Evaluate native touch as MPR121 alternative (future)

**Notes:**
- High-end variant: dual-core, screen, WiFi-capable, OTA updates
- I²S DMA = smooth polyphony, good CPU headroom

---

### ESP32 Audio Kit — Granular Synthesis & Case Design
**Board:** ESP32-S3 on Audio Kit carrier | **Codec:** ES8388 (professional audio) | **PSRAM:** 8MB  
**Pads:** Native touch (14 max) or MPR121 (experimental)  
**Epics:** Hang drum sound design (granular), speaker design (secondary)

**Current Work:**
- Test max granular voices — Determine polyphony limit; profile CPU + memory; measure latency. [config.h](variants/esp32audiokit-sampling/config.h) | Memory: [[ESP32 Audio Kit sampler]]
- Design printable case — CAD enclosure for Audio Kit + pad layout. Account for screen (future), audio I/O access.

**Build & Flash:**
```bash
pio run -e esp32audiokit-sampling -t upload
pio device monitor -e esp32audiokit-sampling
```

**Hardware:**
- ES8388 codec: Balanced XLR out + headphone jack
- 8MB PSRAM: ~45 sec mono / ~22 sec stereo at 44.1 kHz
- I²S: Full-duplex capable
- Native touch: 14 channels on-chip

**Next Steps:**
- [ ] Set up granular synthesis engine (Karplus-Strong or sample playback)
- [ ] Measure voice count at various buffer sizes
- [ ] Profile CPU (FreeRTOS task priority, Core allocation)
- [ ] Test latency with multiple voices
- [ ] Finalize voice ceiling
- [ ] Sketch case design (I/O access, thermal)
- [ ] CAD printable enclosure

**Notes:**
- Audio quality champion: professional codec, balanced out, low noise
- Granular + sample storage = strong differentiation
- Could become standalone sampler/synth product

---

### Textured White Shell (Pico) — Cap-Sense Tuning & Screen Parity
**Board:** Pico (RP2040) | **Pads:** 13 (bare-pin PIO cap-sense)  
**Epics:** Pico cap-sense tuning, screen parity decision

**Current Work:**
- Attach Pico + tune cap-sense — Install in shell, refine CDC/CDT tuning. Test on textured surface (may affect baseline). Validate LED performance.
- Screen parity decision — Can Pico + screen match ESP32-S3 features? Research screen driver compatibility with PIO cap-sense.

**Next Steps:**
- [ ] Install Pico in shell
- [ ] Tune cap-sense on textured surface
- [ ] Research screen + I2C compatibility with PIO
- [ ] Decide: add screen or keep bare?
- [ ] Test feature/UX parity if screen added

**Notes:**
- Surface texture may affect capacitance baseline; plan for retrim
- GPIO constraints if screen added (I2C/SPI conflicts possible)

---

### Teensy 3.2 + Audio Shield — Speaker Design & Enclosure
**Board:** Teensy 3.2 + Audio Shield | **Pads:** 13 (MPR121)  
**Audio:** I2S to Audio Shield codec  
**Epics:** Speaker design (primary focus)

**Current Work:**
- Speaker design + enclosure — Acoustic principles (driver impedance, ports, resonance). CAD design for integrated speaker. [config.h](variants/teensy32-13pad/config.h) | [firmware](variants/teensy32-13pad/teensy32-13pad.cpp)

**Build & Flash:**
```bash
pio run -e teensy32-13pad -t upload
pio device monitor -e teensy32-13pad
```

**Next Steps:**
- [ ] Sketch speaker + enclosure designs (ported vs sealed)
- [ ] Research compact driver options (2–4")
- [ ] CAD prototype mount for driver
- [ ] Test acoustic response
- [ ] Finalize enclosure design
- [ ] 3D print + assemble prototype

**Notes:**
- Teensy 3.2 retired from primary variants; repurposed for speaker focus
- Audio Shield codec = clean audio path
- 13 pads good for hand-held + integrated speaker
- Could be "Omniphone portable speaker variant"

---

## ⏸ Backlog (Waiting on Decisions)

### Black 11-Pad (Teensy 4.0) — Board Replacement Decision
**Board:** Teensy 4.0 | **Pads:** 11 (MPR121)  
**Status:** Awaiting architecture decision

**Decision Point:**
- Replace Teensy 4.0 with **Pico** (PIO cap-sense) or **ESP32-S3** (native touch)?
- Teensy 4.0 is over-spec'd for 11 pads
- Freeing it up enables many-pad experiments

**Related:** [[Architecture decision: board consolidation]]

**Next Steps:**
- [ ] Decide: Pico or ESP32 replacement?
- [ ] If Pico: procure resistors, test cap-sense
- [ ] If ESP32: validate native touch on 11 pads
- [ ] Free Teensy 4.0 for many-pad experiments

**Notes:**
- Pico fits 11-pad easily with PIO cap-sense
- ESP32-S3 also viable but overkill
- Simpler board = simpler firmware maintenance

---

### Printed Wooden 12-Pad + Screen — Board & Screen Integration
**Status:** Awaiting architecture decision

**Decision Points:**
1. Which board (Teensy, Pico, ESP32)?
2. How to fit 1.28" screen into wooden enclosure?

**Current Blockers:**
- Architecture decision (board allocation)
- Screen mechanical integration (CAD needed)

**Next Steps:**
- [ ] Finalize board consolidation decision
- [ ] Sketch screen mounting options
- [ ] CAD mechanical fit
- [ ] Route I2C/SDA/SCL wiring
- [ ] Prototype fit check

---

### ESP32-S3 Native Touch Design — 13-Pad Variant
**Decision Point:** Use on-chip 14-channel cap-sensing instead of MPR121?

**Proposed Allocation:**
- **ESP32-S3** (native touch) → Black 13-pad (replaces Pico)
- **Pico** (PIO cap-sense) → Black 11-pad (replaces Teensy 4.0)
- **Teensy 4.0** → freed for many-pad experiments

**Rationale:**
- ESP32-S3 has 14 on-die channels (eliminate MPR121 for lower pad counts)
- Pico proven PIO cap-sense works well
- Teensy 4.0 = highest-performance board for high-pad testbed

**Subtasks:**
- [ ] Validate ESP32-S3 native touch on 13 pads
- [ ] Tune CDC/CDT (different params vs MPR121)
- [ ] Create variant: `esp32s3-13pad-native-touch`
- [ ] Compare responsiveness: Pico vs native touch vs MPR121
- [ ] Decide if 19-pad switches to native (14-channel ceiling)

**Related:** [[Architecture decision: board consolidation]]

---

## 🧪 Experimental Epics (Waiting on Board Decision)

### EPIC: Pico Capacitive Sensing Tuning
**Instruments:** Black 13-pad (primary), Textured white (secondary)  
**Current Status:** 🔌 Hardware pending (resistors ordered)

**Overview:** Perfecting bare-pin PIO-based cap-sense (no MPR121). Proven "keeper" design: Mode 3 PIO guarded scan. Target: validate across shells + compare vs MPR121 + ESP32-S3 native.

**Subtasks:**
- [ ] Procure resistors (~1M, exact TBD)
- [ ] Install in black-13pad, tune CDC/CDT
- [ ] Test on textured white surface
- [ ] Measure latency vs MPR121 baseline
- [ ] Measure latency vs ESP32-S3 native
- [ ] Document tuning constants (config.h)

**Memory Reference:** [[Pico PIO capsense]] — Mode 3 PIO guarded scan proven

---

### EPIC: ESP32-S3 Native Touch Sensing
**Instruments:** Black 13-pad (primary), White 19-pad (secondary)  
**Current Status:** Design phase (blocked by architecture decision)

**Overview:** On-chip 14-channel capacitive sensing. Eliminates MPR121 for lower pad counts; simpler wiring + firmware.

**Subtasks:**
- [ ] Validate native touch on 13 pads (black-13pad)
- [ ] Tune CDC/CDT (different parameters vs MPR121)
- [ ] Measure latency + consistency
- [ ] Create variant: `esp32s3-13pad-native-touch`
- [ ] Compare vs Pico cap-sense vs MPR121
- [ ] Decide if 19-pad switches (14-channel ceiling)

**Memory Reference:** [[ESP32-S3 T8-S3 bring-up]]

---

### EPIC: Many-Pad MPR121 Experimental Version
**Instruments:** Teensy 4.0 (repurposed after board consolidation)  
**Current Status:** Planned (blocked by architecture decision)

**Overview:** High-pad-count testbed using 2–4 MPR121 boards on one I²C bus. Determine practical pad ceiling + firmware complexity.

**Subtasks:**
- [ ] Finalize board consolidation (frees Teensy 4.0)
- [ ] Design firmware for multi-chip MPR121 (2 chips = 24+ pads)
- [ ] Test I²C scanning latency with 2 chips
- [ ] Scale to 3 chips (36+ pads) if feasible
- [ ] Profile CPU + scanning performance
- [ ] Document scanning strategy (SENSE_PERIOD_MS, I²C clock, addressing)
- [ ] CAD shell design for many-pad layout
- [ ] Evaluate synth polyphony limits (can handle 24+ voices?)

**Memory Reference:** [[Teensy12 I2C ceiling]] — 100 kHz max with long wires + bus capacitance

---

### EPIC: Speaker Design
**Instruments:** Teensy 3.2 + Shield (primary), ESP32 Audio Kit (secondary)  
**Current Status:** Early (design phase)

**Overview:** Integrated speaker options for natural, resonant output. Explore driver matching + enclosure tuning.

**Subtasks:**
- [ ] Research driver impedance + power amp matching
- [ ] Sketch enclosure designs (ported vs sealed)
- [ ] Test acoustic principles (bass response, resonance)
- [ ] CAD enclosure for Teensy 3.2 + Shield
- [ ] CAD enclosure for ESP32 Audio Kit
- [ ] Prototype + measure frequency response

**Goal:** Warm, organic, acoustic-like sound (not high-fidelity). Small form factor (2–4" drivers).

---

### EPIC: Hang Drum Sound Design (Cross-All)
**Instruments:** Bamboo 12-pad (primary), White 19-pad (secondary), ESP32 Audio Kit (granular variant), all others benefit  
**Current Status:** Early (harmonic journey in progress on Bamboo)

**Overview:** Rich, resonant physical modeling synth capturing hang drum aesthetic. Warm, long sustains, modal resonance—not just wavetables.

**Subtasks:**
- [ ] Refine "harmonic journey" on Bamboo (current in-progress)
- [ ] Test harmonic journey on White 19-pad
- [ ] Explore sample-based modal synthesis (granular on Audio Kit)
- [ ] Design envelope shaping for hang drum sustain (slow release, no zipper)
- [ ] Experiment with layered wavetables (saw + filtered noise for shimmer)
- [ ] Record/analyze real hang drum for harmonic content
- [ ] Implement touch-responsive timbre morphing (brightness/decay with pressure)
- [ ] Build timbre library: "warm hang", "bright hang", "metallic hang"

**Audio Reference:**
- Physical modeling: Karplus-Strong (plucked strings), modal resonators
- Inspiration: Hang, steel tongue drum, vibraphone
- Techniques: Wavetable morphing, FM, granular synthesis, convolution

**Notes:**
- Hang drum sustain is very long (3–10 sec); CPU cost of tails is real
- Touch = brightness/attack (more expressive than volume)
- Natural sounding = avoid aliasing (interpolation in wavetables) + soft limiting (polyphonic peaks)

---

## ✅ Done

(Track completions here as tasks finish)

---

## Quick Status Summary

| Instrument | Status | Board | Pads | Epics | Key Task |
|---|---|---|---|---|---|
| 🎋 Bamboo 12-pad | 🔥 In Prog | Teensy 4.0 | 12 | Hang drum | Harmonic journey |
| ⚫ Black 11-pad | ⏸ Backlog | Teensy 4.0 | 11 | — | Board decision |
| ⚫ Black 13-pad | 📋 Todo | Pico | 13 | Pico cap-sense | Get resistors |
| ⚪ White 7-pad | 📋 Todo | Teensy LC | 7 | — | Test LEDs |
| ⚪ White 19-pad | 📋 Todo | ESP32-S3 | 19 | Hang drum | Sound design |
| 🎵 ESP32 Audio Kit | 📋 Todo | ESP32-S3 | ? | Hang drum, Speaker | Granular voices |
| ✨ Textured white | 📋 Todo | Pico | 13 | Pico cap-sense | Cap-sense + screen? |
| 🛡️ Teensy 3.2+Shield | 📋 Todo | Teensy 3.2 | 13 | Speaker | Enclosure CAD |
| 🪵 Printed wooden 12-pad | ⏸ Backlog | TBD | 12 | — | Board + screen |

---

## How to Use

**Daily work:** Scan "In Progress" + "Todo" sections for your current tasks.

**Check per-instrument status:** Find the instrument name (e.g., "Bamboo 12-Pad") to see:
- Board + pad count
- Current work
- Build commands
- Next steps
- Related code links

**Follow an experimental line:** Search for "EPIC:" to find multi-instrument initiatives (speaker design, hang drum sound, cap-sense tuning).

**Make progress:** Move tasks between sections (In Progress → Todo → Backlog), edit status notes.

**Add links:** Use `[text](path)` for code, memory refs as `[[name]]`.

---

## Legend

- 🔥 In Progress — actively being worked on
- 📋 Todo — ready to start
- ⏸ Backlog — waiting on decision or dependency
- ✅ Done — completed
- 🔌 Note marker — specific blocker or resource (e.g., "resistors ordered")
- (HIGH/MEDIUM/LOW) — priority when noted
- Memory ref: `[[name]]` — links to auto-memory from prior sessions
