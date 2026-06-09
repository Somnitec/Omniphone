# ◉ Omniphone
 
> *An instrument for everyone. Any shape. Any sound. Any player.*
 
---
 
The **Omniphone** is an open, tactile electronic instrument — built to be as expressive for a seasoned performer as it is immediate and joyful for someone touching it for the first time.
 
It started as a single idea. It grew into a collaboration. Now it's becoming a platform.
 
---
 
## ✦ Origin
 
The Omniphone was conceived and first built by **Arvid Jense**, as an experiment in what an electronic instrument could be if it didn't assume what a musician looks like.
 
It was developed further in close creative partnership with **Marie Caye** — shaping its philosophy of radical accessibility and shape language.
 
→ *Read the origin story and the thinking behind inclusive instrument design on [Create Digital Music](https://cdm.link/omniphone-inclusive-design-music-therapy/)*
 
---
 
## ✦ Now
 
The project continues under **Arvid Jense**, in active collaboration with **[SoundLab at Muziekgebouw aan 't IJ](https://www.muziekgebouw.nl)**.
 
This phase is about turning the Omniphone from a singular instrument into a **flexible platform**:
 
- 🔷 **Many shapes** — modular physical configurations for different bodies, contexts, and settings
- 🔊 **Many sounds** — a sonic engine that adapts to the musical world of each player
- 🎓 **Many players** — from children in workshops to professional performers on stage; from music therapists to noise artists
 
---
 
## ✦ What It Is
 
The Omniphone uses **capacitive touch sensing** to turn almost any surface into a playable instrument. No keys. No buttons. No prior musical knowledge required.
 
It is simultaneously:
- A **workshop instrument** — robust, inviting, immediately rewarding
- A **performance instrument** — expressive, precise, deeply configurable
- A **research platform** — for inclusive design, music therapy, and expanded musical interfaces
 
---
 
## ✦ Status
 
> 🚧 **Active development** — hardware and software in flux
 
Current focus:
- Capacitive sensing via MPR121
- LED feedback and expressive output mapping
- Platform architecture for multi-configuration support
 
---
 
## ✦ Code & Building

The firmware is a **PlatformIO monorepo**: every hardware configuration ("variant") lives in its own folder under [`variants/`](variants/) and is selected by a matching `[env:…]` in [`platformio.ini`](platformio.ini). Shared libraries (MPR121 driver, LCD driver) live in [`lib/`](lib/) and compile into every build; bring-up/diagnostic sketches live in [`test/`](test/).

### Variants

| `pio run -e …`           | Board       | Pads | Sensor | Audio          | Notes |
|--------------------------|-------------|:----:|--------|----------------|-------|
| `pico-13pad` *(default)* | RPi Pico / RP2040 | 13 | MPR121 | Mozzi (I²S)    | earlephilhower core |
| `esp32s3-19pad`          | ESP32-S3 (LilyGO T8-S3) | 19 | 4× MPR121 | TBD | WiFi/OTA — capsense bring-up test (Teleplot + web view) |
| `teensy40-11pad`         | Teensy 4.0  | 11   | MPR121 | Teensy Audio   | original firmware |
| `teensy40-12pad-screen`  | Teensy 4.0  | 12   | MPR121 | Teensy Audio   | + 1.28" round LCD |
| `teensy32-13pad`         | Teensy 3.2  | 13   | MPR121 | Teensy Audio   | ⚠ WIP — uses a Teensy-4 USB register, won't build for 3.2 yet |
| `teensy-lc-7voice`       | Teensy LC   | 7    | TSI    | internal DAC   | standalone; WS2812 + PWM LEDs |

```bash
pio run -e pico-13pad              # build
pio run -e pico-13pad -t upload    # build + flash
```

### Working on / adding a variant

- **Each instrument is a folder, not a branch.** To work on one, edit its folder in `variants/` directly on `main`. To freeze a finished instrument, **tag a release** rather than parking a branch.
- **Backporting** improvements is just editing shared code in `lib/` — every variant picks it up. (Variant-specific code that proves reusable can be promoted into `lib/` over time.)
- **New variant:** copy the closest `variants/<x>/` folder, adjust its `config.h` (pin map / pad count / layout), and add a `[env:…]` pointing `build_src_filter` at the new folder.
- **Branches** are for in-progress features/experiments that merge back into `main`.

---

## ✦ Contributing
 
This repository is the working codebase for the Omniphone platform. Development is led by Arvid Jense with SoundLab at Muziekgebouw aan 't IJ.
 
If you're a developer, instrument builder, music therapist, or curious human — get in touch before diving in.
 
---
 
## ✦ Credits
 
| Role | Person |
|---|---|
| Concept & original design | Arvid Jense |
| Co-development | Arvid Jense & Marie Caye |
| Ongoing development | Arvid Jense |
| Institutional partner | SoundLab, Muziekgebouw aan 't IJ |
 
---
 
*The Omniphone exists because music shouldn't have gatekeepers.*