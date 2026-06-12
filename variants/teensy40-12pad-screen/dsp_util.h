#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>

// Small shared DSP helpers for the synthesis engines (header-only; everything
// here is only ever included into main.cpp's translation unit).

// ── Sine lookup, phase in 0..1 (cheap enough for hundreds of oscillators) ────
inline float sinLut01(float p)
{
    static float tbl[1026];
    static bool  init = false;
    if (!init) { for (int i = 0; i <= 1025; i++) tbl[i] = sinf(2.0f * (float)M_PI * (float)i / 1024.0f); init = true; }
    p -= (float)(int)p; if (p < 0.0f) p += 1.0f;
    float x = p * 1024.0f;
    int   i = (int)x;
    return tbl[i] + (tbl[i + 1] - tbl[i]) * (x - (float)i);
}

// ── Tiny xorshift noise, -1..1 ───────────────────────────────────────────────
struct NoiseGen {
    uint32_t s = 0x6C8E944Du;
    inline float next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5;
                          return (int32_t)s * (1.0f / 2147483648.0f); }
    inline uint32_t bits() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
};

// ── Fractional-read delay line (for the waveguide engines) ───────────────────
struct DelayLine {
    float*   buf = nullptr;
    uint16_t size = 0, w = 0;
    void init(float* b, uint16_t s) { buf = b; size = s; w = 0; memset(b, 0, s * sizeof(float)); }
    inline void push(float x) { buf[w] = x; if (++w >= size) w = 0; }
    // Read `d` samples into the past (1 <= d <= size-2), linear interp.
    inline float readF(float d) const {
        float r = (float)w - d;
        while (r < 0.0f) r += (float)size;
        uint16_t i = (uint16_t)r, i2 = (uint16_t)(i + 1 >= size ? 0 : i + 1);
        float fr = r - (float)i;
        return buf[i] + (buf[i2] - buf[i]) * fr;
    }
    inline void clear() { if (buf) memset(buf, 0, size * sizeof(float)); }
};
