#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Tonal Map — a navigable chord-movement graph (journey 5 in Harmonic Journey).
//
// The chord-flower diagram and concept: @briancalli.music.
//
// 12 roots × 4 chord types (major, minor, dominant 7, augmented) = 48 nodes.
// The screen shows the CURRENT chord at the centre with its legal moves
// radiating outward (plus a dim preview of each option's own moves). Tapping
// an option glides the pads to that chord and re-centres the map on it.
//
// Edges: solid = functional resolution (always-good move), dotted = optional
// "departure" move (raises tension / changes region). The graph is NOT
// uniform across the 12 roots — it was hand-drawn, node by node, in a
// purpose-built visual editor (a radial graph tool with drag/zoom/rename,
// built for this transcription) and pasted back as JSON, so
// tonalMapMoves() below is a straight lookup table rather than a formula.
// ─────────────────────────────────────────────────────────────────────────────

#include <stdint.h>
#include <math.h>

// Node id = root*4 + type for the main 48-chord flower. Roots 0…11 = C Db D
// Eb E F Gb G Ab A Bb B. Ids 48…55 are a second, smaller ring — 8 chords
// (dominant 7ths and majors) that don't follow the root*4+type packing, so
// their root/type live in CYCLE8_CHORD instead of being bit-unpacked from
// the id. A cycle-8 node can be musically identical to a flower node (e.g.
// "inner Eb" id 55 and "outer Eb" id 12 are both plain Eb major) but is a
// distinct, separately-addressable graph node with its own move list.
enum : uint8_t { TM_MAJ = 0, TM_MIN = 1, TM_DOM7 = 2, TM_AUG = 3 };
static constexpr uint8_t TONAL_FLOWER_NODES = 48;
static constexpr uint8_t TONAL_NODES        = 56;
static constexpr uint8_t TONAL_MAX_MOVES    = 4;    // most options any node offers
static constexpr uint8_t TM_NONE            = 0xFF; // unused move-table slot

struct TonalCycle8Chord { uint8_t root; uint8_t type; };
// Order matches TONAL_NAMES[48..55]: G7 C E7 A Db7 Gb Bb7 Eb, clockwise from
// the top of the inner ring (see tonal-map-editor.html).
static const TonalCycle8Chord CYCLE8_CHORD[8] = {
    { 7, TM_DOM7 }, { 0, TM_MAJ }, { 4, TM_DOM7 }, { 9, TM_MAJ },
    { 1, TM_DOM7 }, { 6, TM_MAJ }, { 10, TM_DOM7 }, { 3, TM_MAJ },
};

inline uint8_t tmRoot(uint8_t n)
{ return (n < TONAL_FLOWER_NODES) ? (uint8_t)(n >> 2) : CYCLE8_CHORD[n - TONAL_FLOWER_NODES].root; }
inline uint8_t tmType(uint8_t n)
{ return (n < TONAL_FLOWER_NODES) ? (uint8_t)(n & 3) : CYCLE8_CHORD[n - TONAL_FLOWER_NODES].type; }

static const char* const TONAL_NAMES[TONAL_NODES] = {
    "C",  "Cm",  "C7",  "C+",
    "Db", "Dbm", "Db7", "Db+",
    "D",  "Dm",  "D7",  "D+",
    "Eb", "Ebm", "Eb7", "Eb+",
    "E",  "Em",  "E7",  "E+",
    "F",  "Fm",  "F7",  "F+",
    "Gb", "Gbm", "Gb7", "Gb+",
    "G",  "Gm",  "G7",  "G+",
    "Ab", "Abm", "Ab7", "Ab+",
    "A",  "Am",  "A7",  "A+",
    "Bb", "Bbm", "Bb7", "Bb+",
    "B",  "Bm",  "B7",  "B+",
    // Inner 8-chord ring (ids 48-55): alternating dominant 7ths and majors.
    "G7", "C", "E7", "A", "Db7", "Gb", "Bb7", "Eb",
};

struct TonalMove {
    uint8_t node;   // destination
    bool    dotted; // dotted (optional/tension) vs solid (functional) edge
};

struct TonalMoveDef { uint8_t node; bool dotted; };

// Hand-drawn move table, one row per node (TONAL_NAMES order), padded to
// TONAL_MAX_MOVES with TM_NONE. Transcribed from the visual editor export,
// 2026-07-03 — see the file header for how this table came to be.
static const TonalMoveDef TONAL_MOVE_TABLE[TONAL_NODES][TONAL_MAX_MOVES] = {
    /* C    */ { { 37, false }, {  9, false }, { 18, false }, { 17, false } },
    /* Cm   */ { { 12, false }, { 22, false }, { 29, false }, { 55, false } },
    /* C7   */ { {  3, false }, { 22, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* C+   */ { { 43, false }, { 11, false }, { 20, false }, { TM_NONE, false } },
    /* Db   */ { { 41, false }, {  7, true  }, {  6, true  }, { TM_NONE, false } },
    /* Dbm  */ { { 36, false }, { 33, false }, { 16, false }, { 26, false } },
    /* Db7  */ { { 24, false }, { 26, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Db+  */ { { 47, false }, { 15, false }, { 24, false }, { TM_NONE, false } },
    /* D    */ { { 45, false }, { 11, true  }, { 10, true  }, { TM_NONE, false } },
    /* Dm   */ { { 37, false }, {  0, false }, { 20, false }, { 30, false } },
    /* D7   */ { { 11, false }, { 30, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* D+   */ { {  3, false }, { 19, false }, { 28, false }, { TM_NONE, false } },
    /* Eb   */ { {  1, false }, { 21, false }, { 30, false }, { 29, false } },
    /* Ebm  */ { { 34, false }, { 41, false }, { 24, false }, { 53, false } },
    /* Eb7  */ { { 34, false }, { 15, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Eb+  */ { { 32, false }, {  7, false }, { 23, false }, { TM_NONE, false } },
    /* E    */ { {  5, false }, { 19, true  }, { 18, true  }, { TM_NONE, false } },
    /* Em   */ { { 38, false }, { 45, false }, {  0, false }, { 28, false } },
    /* E7   */ { { 36, false }, { 38, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* E+   */ { { 36, false }, { 11, false }, { 27, false }, { TM_NONE, false } },
    /* F    */ { {  9, false }, { 23, true  }, { 22, true  }, { TM_NONE, false } },
    /* Fm   */ { { 32, false }, { 42, false }, {  1, false }, { 12, false } },
    /* F7   */ { { 42, false }, { 23, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* F+   */ { { 40, false }, { 15, false }, { 31, false }, { TM_NONE, false } },
    /* Gb   */ { { 33, false }, { 42, false }, { 41, false }, { 13, false } },
    /* Gbm  */ { { 36, false }, { 46, false }, {  5, false }, { 51, false } },
    /* Gb7  */ { { 46, false }, { 27, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Gb+  */ { { 35, false }, { 44, false }, { 19, false }, { TM_NONE, false } },
    /* G    */ { { 17, false }, { 31, true  }, { 30, true  }, { TM_NONE, false } },
    /* Gm   */ { { 40, false }, {  2, false }, {  9, false }, { 12, false } },
    /* G7   */ { {  0, false }, {  2, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* G+   */ { { 39, false }, {  0, false }, { 23, false }, { TM_NONE, false } },
    /* Ab   */ { { 35, true  }, { 34, true  }, { 21, false }, { TM_NONE, false } },
    /* Abm  */ { { 44, false }, {  6, false }, { 13, false }, { 24, false } },
    /* Ab7  */ { { 35, false }, {  6, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Ab+  */ { { 43, false }, {  4, false }, { 27, false }, { TM_NONE, false } },
    /* A    */ { { 45, false }, {  6, false }, {  5, false }, { 25, false } },
    /* Am   */ { {  0, false }, { 10, false }, { 17, false }, { 49, false } },
    /* A7   */ { { 39, false }, { 10, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* A+   */ { { 47, false }, {  8, false }, { 31, false }, { TM_NONE, false } },
    /* Bb   */ { { 43, true  }, { 42, true  }, { 29, false }, { TM_NONE, false } },
    /* Bbm  */ { {  4, false }, { 14, false }, { 21, false }, { 24, false } },
    /* Bb7  */ { { 12, false }, { 14, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Bb+  */ { { 35, false }, {  3, false }, { 12, false }, { TM_NONE, false } },
    /* B    */ { { 33, false }, { 47, true  }, { 46, true  }, { TM_NONE, false } },
    /* Bm   */ { { 36, false }, {  8, false }, { 18, false }, { 25, false } },
    /* B7   */ { { 47, false }, { 18, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* B+   */ { { 39, false }, {  7, false }, { 16, false }, { TM_NONE, false } },

    // Inner 8-chord ring (ids 48-55). The four "red" majors that share a name
    // with a flower minor chord connect back to that minor (bidirectional —
    // see the matching 4th slot added to Cm/Ebm/Gbm/Am above), and ALL eight
    // ring members additionally chain clockwise: G7→C→E7→A→Db7→Gb→Bb7→Eb→G7.
    /* G7 (inner)  */ { { 49, false }, { TM_NONE, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* C  (inner)  */ { { 37, false }, { 50, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* E7 (inner)  */ { { 51, false }, { TM_NONE, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* A  (inner)  */ { { 25, false }, { 52, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Db7 (inner) */ { { 53, false }, { TM_NONE, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Gb (inner)  */ { { 13, false }, { 54, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Bb7 (inner) */ { { 55, false }, { TM_NONE, false }, { TM_NONE, false }, { TM_NONE, false } },
    /* Eb (inner)  */ { {  1, false }, { 48, false }, { TM_NONE, false }, { TM_NONE, false } },
};

// Outgoing moves for a node — a straight lookup into TONAL_MOVE_TABLE.
// Returns the count (≤ TONAL_MAX_MOVES); out[] is filled in table order.
inline uint8_t tonalMapMoves(uint8_t node, TonalMove* out)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < TONAL_MAX_MOVES; i++) {
        const TonalMoveDef &m = TONAL_MOVE_TABLE[node][i];
        if (m.node == TM_NONE) break;
        out[n].node = m.node; out[n].dotted = m.dotted; n++;
    }
    return n;
}

// Fill the 12 pad frequencies for a node — a Tonal-Map-specific voicing (the
// fixed atlases use their own pad 0-5 / 6-11 split; this one pairs pads by
// index parity instead):
//
//   • Pads pair up (0,1) (2,3) (4,5) (6,7) (8,9) (10,11). Each pair shares one
//     "core" pitch class: the EVEN pad plays it a row lower (octave down),
//     the ODD pad plays it a row higher — so repeats are always octaves, never
//     unison duplicates.
//   • The 6 core pitches climb: the chord tones (root, 3rd, 5th[, 7th]) plus
//     ONE passing (non-chord) tone dropped into the widest gap between them,
//     repeating up an octave once the stack is exhausted. Walking the pads in
//     order traces a smooth rising two-octave-ish line instead of a flat
//     repeating arpeggio, which is friendlier to play melodies on.
//   • Any 3 pads spanning at most 2 adjacent pairs are chord tones (or the
//     one passing tone) a third/second apart — always a consonant cluster,
//     with the passing tone as the sole deliberate exception for melodic
//     interest.
inline void tonalMapFreqs(uint8_t node, float* out)
{
    static const uint8_t IV_MAJ[3]  = { 0, 4, 7 };
    static const uint8_t IV_MIN[3]  = { 0, 3, 7 };
    static const uint8_t IV_DOM7[4] = { 0, 4, 7, 10 };
    static const uint8_t IV_AUG[3]  = { 0, 4, 8 };
    const uint8_t* iv; uint8_t nt;
    switch (tmType(node)) {
        case TM_MIN:  iv = IV_MIN;  nt = 3; break;
        case TM_DOM7: iv = IV_DOM7; nt = 4; break;
        case TM_AUG:  iv = IV_AUG;  nt = 3; break;
        default:      iv = IV_MAJ;  nt = 3; break;
    }

    // Find the widest gap between consecutive chord tones (wrapping to the
    // octave above the root) and drop one passing tone at its midpoint.
    uint8_t widest = 0, gap = 0;
    for (uint8_t i = 0; i < nt; i++) {
        uint8_t next = (uint8_t)((i + 1 < nt) ? iv[i + 1] : 12);
        uint8_t g = (uint8_t)(next - iv[i]);
        if (g > gap) { gap = g; widest = i; }
    }

    uint8_t seq[5]; uint8_t ns = 0; // chord tones (+ passing tone), ascending
    for (uint8_t i = 0; i < nt; i++) {
        seq[ns++] = iv[i];
        if (i == widest && gap >= 2) seq[ns++] = (uint8_t)(iv[i] + gap / 2);
    }

    // Six ascending "core" pitches: cycle seq[], bumping up an octave each
    // time it wraps, so the sequence only ever climbs.
    const float C4 = 261.6256f;
    float core[6];
    uint8_t octUp = 0, idx = 0;
    for (uint8_t k = 0; k < 6; k++) {
        if (idx == ns) { idx = 0; octUp++; }
        float semis = (float)(tmRoot(node) + seq[idx] + 12 * octUp);
        core[k] = C4 * powf(2.0f, semis / 12.0f);
        idx++;
    }

    for (uint8_t k = 0; k < 6; k++) {
        out[2 * k]     = core[k] * 0.5f; // even pad → lower row
        out[2 * k + 1] = core[k];        // odd  pad → upper row
    }
}
