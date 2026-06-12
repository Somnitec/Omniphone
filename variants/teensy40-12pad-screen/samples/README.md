# Samples (Grain Hang + Strings modes)

The wavs in this folder get baked into the Teensy's flash. **The current files are
synthesized placeholders — replace them with real recordings for the good stuff.**

## How to swap a sound
1. Drop your replacement wav here, keeping the naming convention
   `<name>_<rootHz>.wav` — the firmware needs `hang_*.wav` and `strings_*.wav`:
   - `hang_146.83.wav` → one clean handpan/hang strike, the number = its pitch in Hz
   - `strings_130.67.wav` → a sustained string-section note that loops cleanly
2. Delete the old file (only one of each name prefix may exist).
3. Run `python3 tools/wav2header.py` from the repo root.
4. Rebuild + upload.

## Guidelines
- Mono 16-bit 44100 Hz is ideal (stereo gets downmixed, other rates are handled).
- Keep it short: a hang strike 2–4 s, a string loop 1–3 s. Flash budget is ~1.9 MB
  total; the converter prints sizes.
- The **root Hz in the filename must match the recorded pitch** or every pad will
  play out of tune. (A4=440 reference: D3=146.83, C3=130.81, A3=220 …)
- Strings: trim to a clean loop (no attack, no fade) — the engine loops the whole
  file. The hang strike should include its attack.

## Where to find good ones (CC0 / free)
- freesound.org — search "handpan single note", "hang drum D3", "string ensemble
  sustain C3"; filter by license CC0.
- philharmonia.co.uk/explore/sound_samples — free orchestral single notes.
- vis.versilstudios.com (VSCO2 CE) — public-domain orchestral samples.
