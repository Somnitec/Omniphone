# cactus.py  —  procedural cactus enclosure generator for the Omniphone
#
# Run:   python3 cactus.py               (needs numpy — system-wide is fine, NO build123d venv)
# Out:   cactus.stl                       -> send straight to the slicer / 3D printer,
#                                            or import into Fusion 360 / FreeCAD as a mesh body.
#
# Shape: a little potted-cactus body that houses the Omniphone (a capacitive touch
# instrument). One generator grows three species from a shared, watertight UV-sphere
# mesh — a deformed ellipsoid, poles capped by fans, its bottom sliced flat into a
# stand-on-your-desk FOOT (exactly the dome-base trick from coral.py). The species
# only differ in how the surface radius is warped:
#
#   "echinopsis" — a baby Echinopsis: a squat, slightly flattened ribbed globe with
#                  ~12 rounded ribs running pole-to-pole (fading at the crown & foot)
#                  and little areole bumps studded along every rib crest. Sunken crown.
#   "bishopcap"  — Astrophytum myriostigma (Bishop's Cap): a smooth globe with a few
#                  (default 5, 3-8) DEEP smooth ribs — a rounded n-pointed-star section,
#                  no spines. The iconic clean shape.
#   "powderpuff" — Mammillaria bocasana: a globe densely paved in rounded TUBERCLES
#                  laid out on a golden-angle (phyllotaxis) spiral, optionally a small
#                  cluster of 1-3 heads (overlapping closed spheres the slicer unions).
#
# The capacitive PADS (copper-tape seats) are laid onto the body two ways: PAD_MODE
# "features" turns the plant's own features into seats (each RIB, or a subset of the
# TUBERCLES); PAD_MODE "uniform" ignores the features and drops N round flat seats by
# phyllotaxis, like coral.py's raised pads. A round SCREEN recess is carved into the
# crown (top) or the foot (base). The result is a watertight solid — hollow it in the
# slicer / Fusion after import, exactly like coral.py and thistle.py. STL now; a STEP
# path can follow later.

import math
import os
import struct

try:
    import numpy as np  # noqa: F401  (kept for parity with the toolchain; math is pure-Python here)
except ImportError:
    raise SystemExit(
        "cactus.py expects numpy to be importable (project convention).\n"
        "  Fedora:  sudo dnf install python3-numpy      (system-wide, no venv)\n"
        "  or:      pip install --user numpy\n"
        "Unlike thistle.py this does NOT need the build123d venv."
    )

HERE = os.path.dirname(os.path.abspath(__file__))  # write next to this script

# ---------- PARAMETERS (edit these) ----------
SPECIES       = "echinopsis"   # "echinopsis" | "bishopcap" | "powderpuff"
AUTO_PRESET   = True           # True = pull the shape knobs below from PRESETS[SPECIES]
                               #        so just changing SPECIES gives a good-looking plant.
                               #        False = use the raw constants below verbatim.

# ---- body envelope (target ~ 150-200 mm wide) ----
BODY_RADIUS   = 80             # horizontal radius (x,y) of the globe (mm) -> width = 2*R
BODY_HEIGHT   = 110            # full height of the globe before the foot is sliced (mm)
BASE_HEIGHT   = 6              # thickness of the flat FOOT disc the plant stands on (mm)
BASE_CUT      = 0.12           # slice the globe flat below this height-fraction (0 = full sphere)
CROWN_SINK    = 6              # how far the crown (top pole) is dished in (mm) — echinopsis look

# ---- pads : the capacitive copper-tape seats (= your electrode count) ----
N_PADS        = 12             # how many pads. "features": how many ribs/tubercles become seats.
PAD_MODE      = "features"     # "features" = pads ARE the plant's ribs/tubercles ·
                               # "uniform"  = ignore features, drop N round seats by phyllotaxis
PAD_DIA       = 18             # flat seat diameter (mm)
PAD_HEIGHT    = 4              # how far each seat is raised above the local surface (mm)
PAD_SKIRT     = 7              # radial blend from the seat edge back down to the body (mm)
PAD_BAND_LO   = 0.18           # "uniform": seats live between these polar-fractions of the body
PAD_BAND_HI   = 0.88           #            (0 = crown, 1 = foot). Keeps them off pole & rim.

# ---- screen (round recess) ----
SCREEN_MODE   = "top"          # "top" = pocket in the crown · "base" = pocket in the foot · "none"
SCREEN_DIA    = 32             # screen bore diameter (mm)
SCREEN_MARGIN = 5              # flat bezel ring around the screen (mm) — top mode only
SCREEN_DEPTH  = 8              # how deep the pocket sinks (mm)

# ---- species-specific shape knobs (overridden by PRESETS when AUTO_PRESET) ----
N_RIBS        = 12             # echinopsis/bishopcap: number of ribs (bishopcap: 3-8, iconic 5)
RIB_DEPTH     = 0.12           # rib bulge as a fraction of radius (bishopcap deep ~0.30)
TUBERCLE_COUNT= 95             # powderpuff: number of tubercle bumps over the globe (60-140)
TUBERCLE_AMP  = 0.12           # powderpuff: tubercle height as a fraction of radius
N_HEADS       = 1              # powderpuff: cluster of 1-3 overlapping heads
AREOLE_AMP    = 0.030          # echinopsis: little areole bump height (fraction of radius)
AREOLE_ROWS   = 6              # echinopsis: areole bumps per rib crest, pole-to-pole

# ---- mesh resolution (modest so it runs in a few seconds) ----
RES_ANGULAR   = 140            # samples around the globe (longitude)
RES_VERTICAL  = 84             # rings from crown to foot (latitude)

# Recommended per-species settings, applied at build() when AUTO_PRESET is True.
PRESETS = {
    "echinopsis": dict(BODY_RADIUS=80, BODY_HEIGHT=110, BASE_CUT=0.12, CROWN_SINK=7,
                       N_RIBS=12, RIB_DEPTH=0.13, AREOLE_AMP=0.030, AREOLE_ROWS=6),
    "bishopcap":  dict(BODY_RADIUS=72, BODY_HEIGHT=150, BASE_CUT=0.10, CROWN_SINK=4,
                       N_RIBS=5,  RIB_DEPTH=0.30),
    "powderpuff": dict(BODY_RADIUS=80, BODY_HEIGHT=150, BASE_CUT=0.10, CROWN_SINK=3,
                       TUBERCLE_COUNT=95, TUBERCLE_AMP=0.12, N_HEADS=1),
}

RUN_VALIDATION = True          # print the watertightness / bbox report after writing the STL
# ---------------------------------------------

GOLDEN = math.pi * (3.0 - math.sqrt(5.0))   # golden angle for phyllotaxis spirals


# ============================================================================
#  Shared helpers  (smoothstep / vnoise / frame — same voice as coral.py, copied
#  in so this file stays self-contained: pure Python + the numpy import above)
# ============================================================================
def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def vnoise(x, y):
    """A cheap, smooth, deterministic value field in [0,1] (sum of incommensurate
    sines). Same helper as coral.py — handy for jittering feature placement so the
    plant never looks machine-perfect."""
    s = (math.sin(x * 1.00 + 1.7) * math.cos(y * 1.13 - 0.9)
         + math.sin(x * 1.97 - 2.3) * math.cos(y * 2.09 + 1.4) * 0.6
         + math.sin(x * 3.71 + 0.5) * math.cos(y * 3.31 - 1.9) * 0.35)
    return 0.5 + 0.5 * s / 1.95


def frame(zdir):
    """An orthonormal (U, V, Z) frame with Z = zdir. Used to seat cluster heads on
    the body surface without gimbal surprises."""
    L = math.sqrt(sum(v * v for v in zdir)) or 1.0
    Z = (zdir[0] / L, zdir[1] / L, zdir[2] / L)
    ax = (0.0, 0.0, 1.0) if abs(Z[2]) < 0.9 else (1.0, 0.0, 0.0)
    ux = ax[1] * Z[2] - ax[2] * Z[1]
    uy = ax[2] * Z[0] - ax[0] * Z[2]
    uz = ax[0] * Z[1] - ax[1] * Z[0]
    ul = math.sqrt(ux * ux + uy * uy + uz * uz) or 1.0
    U = (ux / ul, uy / ul, uz / ul)
    V = (Z[1] * U[2] - Z[2] * U[1], Z[2] * U[0] - Z[0] * U[2], Z[0] * U[1] - Z[1] * U[0])
    return U, V, Z


def write_stl_binary(path, verts, tris, label):
    def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    with open(path, "wb") as f:
        f.write(label.encode("ascii", "ignore").ljust(80, b" ")[:80])
        f.write(struct.pack("<I", len(tris)))
        for (ia, ib, ic) in tris:
            a, b, c = verts[ia], verts[ib], verts[ic]
            u, v = sub(b, a), sub(c, a)
            nx = u[1] * v[2] - u[2] * v[1]
            ny = u[2] * v[0] - u[0] * v[2]
            nz = u[0] * v[1] - u[1] * v[0]
            L = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(struct.pack("<3f", nx / L, ny / L, nz / L))
            for p in (a, b, c):
                f.write(struct.pack("<3f", *p))
            f.write(struct.pack("<H", 0))


def _ang(d, c):
    """Great-circle angle (radians) between two unit direction tuples."""
    dot = d[0] * c[0] + d[1] * c[1] + d[2] * c[2]
    return math.acos(max(-1.0, min(1.0, dot)))


# ============================================================================
#  Feature placement — direction vectors on the unit sphere for tubercles,
#  areoles and pad seats. All in the same (x,y,z) unit-direction convention as
#  the body: dir = (sin phi cos th, sin phi sin th, cos phi), phi from the crown.
# ============================================================================
def tubercle_dirs(n):
    """`n` tubercle centres spread by golden-angle (spherical Fibonacci)."""
    out = []
    for i in range(int(n)):
        z = 1.0 - 2.0 * (i + 0.5) / n            # even area from crown (+z) to foot (-z)
        r = math.sqrt(max(0.0, 1.0 - z * z))
        th = i * GOLDEN
        out.append((r * math.cos(th), r * math.sin(th), z))
    return out


def areole_dirs(n_ribs, rows, lat_lo):
    """Areole bump centres: `rows` bumps marching down each of the `n_ribs` crests.
    Crests sit at theta = 2*pi*k/n_ribs; rows are stepped in polar angle phi."""
    out = []
    for k in range(int(n_ribs)):
        th = 2.0 * math.pi * k / max(1, int(n_ribs))
        for r in range(int(rows)):
            frac = (r + 0.5) / rows
            phi = lat_lo * (0.10 + 0.80 * frac)   # keep them off the very crown & foot
            sp = math.sin(phi)
            out.append((sp * math.cos(th), sp * math.sin(th), math.cos(phi)))
    return out


def seat_dirs(lat_lo, tub_dirs):
    """Pad-seat centres as unit directions. Depends on PAD_MODE and SPECIES."""
    n = int(N_PADS)
    if PAD_MODE == "uniform":
        # phyllotaxis over a polar band of the body — even, off pole and rim.
        out = []
        for i in range(n):
            frac = (i + 0.5) / n
            phi = lat_lo * (PAD_BAND_LO + (PAD_BAND_HI - PAD_BAND_LO) * frac)
            th = i * GOLDEN
            sp = math.sin(phi)
            out.append((sp * math.cos(th), sp * math.sin(th), math.cos(phi)))
        return out
    # PAD_MODE == "features"
    if SPECIES == "powderpuff":
        return list(tub_dirs[:n])                 # a subset of tubercles become seats
    # echinopsis / bishopcap: seats ride the rib crests (wrap around ribs, step in phi)
    out = []
    nr = max(1, int(N_RIBS))
    for i in range(n):
        rib = i % nr
        layer = i // nr
        th = 2.0 * math.pi * rib / nr
        # one row centred mid-body; extra layers fan above/below it
        phi = lat_lo * (0.5 + 0.22 * (layer - (n // nr) * 0.5) / max(1, n // nr + 1))
        phi = min(max(phi, 0.15 * lat_lo), 0.85 * lat_lo)
        sp = math.sin(phi)
        out.append((sp * math.cos(th), sp * math.sin(th), math.cos(phi)))
    return out


# ============================================================================
#  The body — a deformed UV ellipsoid, poles capped, bottom sliced into a foot.
#  Watertight by construction: one closed longitude cylinder of quad rings, a
#  fan (or screen pocket) at the crown, a wall + disc (or screen pocket) at the
#  foot. Vertex *positions* are freely warped by the species deformation; the
#  triangle *topology* never changes, so the mesh stays 2-manifold.
# ============================================================================
def build_body():
    A = float(BODY_RADIUS)
    Cz = float(BODY_HEIGHT) / 2.0
    z_cut = -Cz * (1.0 - 2.0 * float(BASE_CUT))            # ellipsoid-z of the flat slice
    lat_lo = math.acos(max(-1.0, min(1.0, z_cut / Cz)))    # polar angle at the slice
    na, nlat = int(RES_ANGULAR), max(3, int(RES_VERTICAL))

    # ---- feature centres ----
    tubs = tubercle_dirs(TUBERCLE_COUNT) if SPECIES == "powderpuff" else []
    arls = (areole_dirs(N_RIBS, AREOLE_ROWS, lat_lo)
            if SPECIES == "echinopsis" else [])
    seats = seat_dirs(lat_lo, tubs)

    ribbed = SPECIES in ("echinopsis", "bishopcap")
    top_screen = (SCREEN_MODE == "top" and SCREEN_DIA > 0)
    base_screen = (SCREEN_MODE == "base" and SCREEN_DIA > 0)

    # screen geometry (top mode): bezel outer radius -> polar angle where the body starts
    r_scr = SCREEN_DIA / 2.0
    r_bez = min(SCREEN_DIA / 2.0 + SCREEN_MARGIN, A * 0.9)
    phi_start = math.asin(max(0.0, min(0.999, r_bez / A))) if top_screen else 0.0

    # feature amplitudes in mm, and their angular footprints
    tub_amp = TUBERCLE_AMP * A
    tub_sig = 2.35 / math.sqrt(max(1.0, TUBERCLE_COUNT))   # gaussian width ~ tubercle spacing
    arl_amp = AREOLE_AMP * A
    arl_sig = (math.pi / max(1, N_RIBS)) * 0.45            # areole a touch narrower than a rib gap
    seat_ra = (PAD_DIA / 2.0) / A                          # seat plateau angular radius
    seat_sk = PAD_SKIRT / A                                # seat skirt angular width

    def fade(phi):
        """Deformation envelope: 0 at both poles so ribs/tubercles vanish at the
        crown and the foot rim (keeps both circular). In top-screen mode it is also
        pinned to 0 out to the bezel so the screen sits on flat, round material."""
        t = phi / lat_lo
        f = smoothstep(t / 0.12) * smoothstep((1.0 - t) / 0.12)
        if top_screen:
            f *= smoothstep((phi - phi_start) / (0.6 * phi_start + 1e-6))
        return f

    def crown_dip(phi):
        if top_screen or CROWN_SINK <= 0:
            return 0.0
        t = phi / lat_lo
        return CROWN_SINK * (1.0 - smoothstep(t / 0.22))

    def seat_bump(d):
        h = 0.0
        for c in seats:
            a = _ang(d, c)
            if a <= seat_ra:
                h = max(h, PAD_HEIGHT)
            elif a < seat_ra + seat_sk:
                h = max(h, PAD_HEIGHT * (1.0 - smoothstep((a - seat_ra) / seat_sk)))
        return h

    def tex_bump(d):
        """Species surface texture (mm along the radial direction), pre-fade."""
        b = 0.0
        if SPECIES == "powderpuff":
            for c in tubs:
                a = _ang(d, c)
                b = max(b, tub_amp * math.exp(-(a / tub_sig) ** 2))
        elif SPECIES == "echinopsis":
            for c in arls:
                a = _ang(d, c)
                b = max(b, arl_amp * math.exp(-(a / arl_sig) ** 2))
        return b

    def surf_pt(theta, phi):
        f = fade(phi)
        sp, cp = math.sin(phi), math.cos(phi)
        ct, st = math.cos(theta), math.sin(theta)
        d = (sp * ct, sp * st, cp)                        # unit radial direction
        m = 1.0 + f * RIB_DEPTH * math.cos(N_RIBS * theta) if ribbed else 1.0
        bump = tex_bump(d) * f + seat_bump(d)             # mm outward along d
        rh = A * m * sp                                   # horizontal radius
        x = rh * ct + bump * d[0]
        y = rh * st + bump * d[1]
        z = Cz * cp + bump * d[2]
        z = BASE_HEIGHT + (z - z_cut) - crown_dip(phi)
        return (x, y, z)

    def surf_ring(phi):
        return [surf_pt(2.0 * math.pi * i / na, phi) for i in range(na)]

    verts, tris = [], []

    def add_ring(ring):
        base = len(verts)
        verts.extend(ring)
        return base

    def strip(a, b):                                       # quad strip between two rings
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((a + i, a + i1, b + i1))
            tris.append((a + i, b + i1, b + i))

    # ---- body rings: from the crown down to the foot slice ----
    if top_screen:
        phis = [phi_start + (lat_lo - phi_start) * j / nlat for j in range(nlat + 1)]
    else:
        phis = [lat_lo * j / nlat for j in range(1, nlat + 1)]   # skip the degenerate pole ring
    ring_bases = [add_ring(surf_ring(p)) for p in phis]
    for kk in range(len(ring_bases) - 1):
        strip(ring_bases[kk], ring_bases[kk + 1])

    # ---- crown cap: screen pocket, or a fan to a single (possibly sunk) pole vertex ----
    top_ring = ring_bases[0]
    if top_screen:
        bezel_z = verts[top_ring][2]                       # ring0 is flat & round (fade==0 there)
        floor_z = bezel_z - SCREEN_DEPTH
        inner = add_ring([(r_scr * math.cos(2 * math.pi * i / na),
                           r_scr * math.sin(2 * math.pi * i / na), bezel_z) for i in range(na)])
        floor = add_ring([(r_scr * math.cos(2 * math.pi * i / na),
                           r_scr * math.sin(2 * math.pi * i / na), floor_z) for i in range(na)])
        for i in range(na):                                # flat bezel annulus (crown rim -> bore)
            i1 = (i + 1) % na
            tris.append((top_ring + i, inner + i1, top_ring + i1))
            tris.append((top_ring + i, inner + i, inner + i1))
        for i in range(na):                                # bore wall down
            i1 = (i + 1) % na
            tris.append((inner + i, floor + i1, inner + i1))
            tris.append((inner + i, floor + i, floor + i1))
        fcen = len(verts); verts.append((0.0, 0.0, floor_z))
        for i in range(na):                                # pocket floor
            i1 = (i + 1) % na
            tris.append((fcen, floor + i, floor + i1))
    else:
        pole = surf_pt(0.0, 0.0)
        pc = len(verts); verts.append(pole)
        for i in range(na):
            tris.append((pc, top_ring + (i + 1) % na, top_ring + i))

    # ---- foot: drop the slice rim to z=0 and close it (optionally a base screen pocket) ----
    rim = ring_bases[-1]
    bot = add_ring([(verts[rim + i][0], verts[rim + i][1], 0.0) for i in range(na)])
    for i in range(na):                                    # foot wall (rim -> z=0)
        i1 = (i + 1) % na
        tris.append((rim + i, bot + i, bot + i1))
        tris.append((rim + i, bot + i1, rim + i1))

    if base_screen:
        rs = min(SCREEN_DIA / 2.0, A * 0.6)
        inner = add_ring([(rs * math.cos(2 * math.pi * i / na),
                           rs * math.sin(2 * math.pi * i / na), 0.0) for i in range(na)])
        floor = add_ring([(rs * math.cos(2 * math.pi * i / na),
                           rs * math.sin(2 * math.pi * i / na), SCREEN_DEPTH) for i in range(na)])
        for i in range(na):                                # flat underside annulus (rim -> bore)
            i1 = (i + 1) % na
            tris.append((bot + i, bot + i1, inner + i1))
            tris.append((bot + i, inner + i1, inner + i))
        for i in range(na):                                # bore wall up into the body
            i1 = (i + 1) % na
            tris.append((inner + i, inner + i1, floor + i1))
            tris.append((inner + i, floor + i1, floor + i))
        fcen = len(verts); verts.append((0.0, 0.0, SCREEN_DEPTH))
        for i in range(na):                                # pocket ceiling
            i1 = (i + 1) % na
            tris.append((fcen, floor + i1, floor + i))
    else:
        bc = len(verts); verts.append((0.0, 0.0, 0.0))     # plain flat foot disc
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((bc, bot + i1, bot + i))

    return verts, tris, (A, Cz, lat_lo, tubs, tub_amp, tub_sig)


# ============================================================================
#  Cluster heads (powderpuff) — extra fully-closed tuberculed spheres that
#  overlap the main body near the crown. Each is independently watertight; the
#  slicer/Fusion unions the overlap on import (same trick as coral.py branches).
# ============================================================================
def add_head(verts, tris, center, radius, tubs, tub_amp, tub_sig):
    na, nlat = int(RES_ANGULAR), max(3, int(RES_VERTICAL) // 2)
    cx, cy, cz = center

    def r_at(d):
        b = radius
        for c in tubs:
            a = _ang(d, c)
            b = max(b, radius + tub_amp * math.exp(-(a / tub_sig) ** 2))
        return b

    def pt(theta, phi):
        d = (math.sin(phi) * math.cos(theta), math.sin(phi) * math.sin(theta), math.cos(phi))
        r = r_at(d)
        return (cx + r * d[0], cy + r * d[1], cz + r * d[2])

    bases = []
    for j in range(1, nlat):                               # interior latitude rings only
        phi = math.pi * j / nlat
        b = len(verts)
        verts.extend([pt(2.0 * math.pi * i / na, phi) for i in range(na)])
        bases.append(b)
    for kk in range(len(bases) - 1):
        a, b = bases[kk], bases[kk + 1]
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((a + i, a + i1, b + i1))
            tris.append((a + i, b + i1, b + i))
    npole = len(verts); verts.append(pt(0.0, 0.0))         # north cap
    for i in range(na):
        tris.append((npole, bases[0] + (i + 1) % na, bases[0] + i))
    spole = len(verts); verts.append(pt(0.0, math.pi))     # south cap
    last = bases[-1]
    for i in range(na):
        tris.append((spole, last + i, last + (i + 1) % na))


# ============================================================================
#  Driver
# ============================================================================
def build():
    if AUTO_PRESET and SPECIES in PRESETS:                 # pull good per-species shape knobs
        g = globals()
        for k, v in PRESETS[SPECIES].items():
            g[k] = v

    verts, tris, info = build_body()
    A, Cz, lat_lo, tubs, tub_amp, tub_sig = info

    if SPECIES == "powderpuff" and int(N_HEADS) > 1:
        # seat (N_HEADS-1) smaller heads around the crown, half-buried in the body
        r_head = A * 0.55
        for h in range(int(N_HEADS) - 1):
            th = 2.0 * math.pi * h / max(1, int(N_HEADS) - 1)
            phi = 0.42 * lat_lo
            d = (math.sin(phi) * math.cos(th), math.sin(phi) * math.sin(th), math.cos(phi))
            cz = BASE_HEIGHT + (Cz * d[2] - (-Cz * (1.0 - 2.0 * BASE_CUT)))
            center = (A * 0.72 * d[0], A * 0.72 * d[1], cz)
            add_head(verts, tris, center, r_head, tubs, tub_amp, tub_sig)

    return verts, tris


# ============================================================================
#  Validation — watertightness (every undirected edge shared by exactly 2 tris),
#  finiteness, and bounding box. Run per species x pad-mode before shipping.
# ============================================================================
def check_mesh(verts, tris):
    edges = {}
    for (a, b, c) in tris:
        for e in ((a, b), (b, c), (c, a)):
            key = (e[0], e[1]) if e[0] < e[1] else (e[1], e[0])
            edges[key] = edges.get(key, 0) + 1
    nonmanifold = sum(1 for v in edges.values() if v != 2)
    bad = 0
    xs = ys = zs = None
    lo = [1e30, 1e30, 1e30]
    hi = [-1e30, -1e30, -1e30]
    for p in verts:
        for k in range(3):
            if not math.isfinite(p[k]):
                bad += 1
            else:
                lo[k] = min(lo[k], p[k]); hi[k] = max(hi[k], p[k])
    bbox = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    return nonmanifold, bad, bbox


def validate():
    print("\n=== validation (watertight = every edge shared by exactly 2 triangles) ===")
    g = globals()
    saved = (g["SPECIES"], g["PAD_MODE"])
    for sp in ("echinopsis", "bishopcap", "powderpuff"):
        for mode in ("features", "uniform"):
            g["SPECIES"], g["PAD_MODE"] = sp, mode
            verts, tris = build()
            nm, bad, bb = check_mesh(verts, tris)
            print(f"  {sp:11s} {mode:8s}: {len(verts):6d} verts  {len(tris):6d} tris  "
                  f"non-manifold edges={nm:<4d} nan/inf={bad}  "
                  f"bbox = {bb[0]:.1f} x {bb[1]:.1f} x {bb[2]:.1f} mm")
    g["SPECIES"], g["PAD_MODE"] = saved


if __name__ == "__main__":
    verts, tris = build()
    stl_path = os.path.join(HERE, "cactus.stl")
    write_stl_binary(stl_path, verts, tris, f"cactus-{SPECIES} (Omniphone) enclosure")
    print(f"wrote {stl_path}  (species={SPECIES}, pads={PAD_MODE}, "
          f"{len(verts)} verts, {len(tris)} triangles)")
    if RUN_VALIDATION:
        validate()
