#!/usr/bin/env python3
"""Synthesize license-free PLACEHOLDER samples for the Grain Hang and Strings
modes. Replace them with real recordings whenever you like (see samples/README.md),
then re-run tools/wav2header.py and rebuild.

Pure stdlib — no numpy needed."""
import math, os, random, struct, wave

SR = 44100
OUT = os.path.join(os.path.dirname(__file__), "..",
                   "variants", "teensy40-12pad-screen", "samples")


def write_wav(path, samples):
    peak = max(1e-9, max(abs(s) for s in samples))
    norm = 0.85 / peak
    with wave.open(path, "w") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(b"".join(
            struct.pack("<h", int(max(-1.0, min(1.0, s * norm)) * 32767))
            for s in samples))
    print(f"wrote {path} ({len(samples)} samples, {len(samples)/SR:.2f}s)")


def make_hang(f0=146.83, dur=2.6):
    """One handpan-ish strike: harmonic partials with per-partial decay,
    a soft mallet thump and a few ms of attack noise."""
    n = int(SR * dur)
    #          ratio  amp   tau(s)
    partials = [(1.00, 1.00, 1.90),
                (2.00, 0.55, 1.10),
                (3.00, 0.32, 0.80),
                (3.98, 0.12, 0.55),
                (5.04, 0.07, 0.40)]
    rng = random.Random(7)
    phases = [rng.random() * 2 * math.pi for _ in partials]
    out = []
    for i in range(n):
        t = i / SR
        s = 0.0
        for (r, a, tau), ph in zip(partials, phases):
            s += a * math.exp(-t / tau) * math.sin(2 * math.pi * f0 * r * t + ph)
        # mallet thump (low sine, fast decay) + a touch of attack noise
        s += 0.40 * math.exp(-t / 0.060) * math.sin(2 * math.pi * 90.0 * t)
        if t < 0.008:
            s += 0.25 * (1.0 - t / 0.008) * (rng.random() * 2 - 1)
        out.append(s)
    return out


def make_strings(dur=1.5):
    """A seamlessly-loopable string-section tone: harmonics of f0 (integer cycles
    over the loop) with slow integer-cycle amplitude movement per harmonic.
    Ensemble detune/width is added at playback by the engine's detuned readers."""
    cycles = 196                      # integer cycles → perfect loop
    f0 = cycles / dur                 # ≈130.67 Hz (~C3)
    n = int(SR * dur)
    rng = random.Random(3)
    harms = []
    for h in range(1, 19):
        amp = (1.0 / h) ** 1.15 / (1.0 + (h * f0 / 2500.0) ** 2)  # saw-ish + LP rolloff
        am_rate = rng.randint(2, 7)   # integer cycles per loop → loops cleanly
        am_ph = rng.random() * 2 * math.pi
        ph = rng.random() * 2 * math.pi
        harms.append((h, amp, am_rate, am_ph, ph))
    out = []
    for i in range(n):
        t = i / SR
        s = 0.0
        for h, amp, am_rate, am_ph, ph in harms:
            am = 1.0 + 0.18 * math.sin(2 * math.pi * am_rate * t / dur + am_ph)
            s += amp * am * math.sin(2 * math.pi * f0 * h * t + ph)
        out.append(s)
    return out, f0


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    write_wav(os.path.join(OUT, "hang_146.83.wav"), make_hang())
    strings, f0 = make_strings()
    write_wav(os.path.join(OUT, f"strings_{f0:.2f}.wav"), strings)
