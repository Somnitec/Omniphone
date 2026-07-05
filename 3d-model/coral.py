# coral.py  —  procedural coral / reaction-diffusion enclosure generator
#
# Run:   python3 coral.py               (needs numpy — system-wide is fine, NO build123d venv)
# Out:   coral.stl                       -> send straight to the slicer / 3D printer,
#                                           or import into Fusion 360 / FreeCAD as a mesh body.
#
# Shape: a coral-like body whose surface is sculpted by a *reaction-diffusion* field
# (Gray-Scott) — the same maths that grows the labyrinthine ridges of brain coral,
# the spots of a pufferfish, and Turing patterns generally. Each of the N capacitive
# PADS ("loops") is a raised, flat-topped protrusion that seats copper tape; the RD
# simulation is SEEDED at the pad centres, so the wavy coral ridges radiate out from
# every pad (RD *grows* the protrusions) while the very same field also displaces the
# whole surface into brain-coral grooves (RD *textures* it).
#
# Three forms share one generator (set FORM):
#   "dome"    — a brain-coral dome (default): a low dome carpeted in RD ridges, pads
#               are flat seats raised on the bumps. This is the primary look.
#   "bulb"    — an ellipsoid body (thistle-style) with N wavy coral BRANCHES = pads.
#   "fingers" — a base plate with N upright wavy finger-coral tubes = pads.
#
# The result is a watertight solid (or a union of overlapping watertight solids, which
# the slicer/Fusion merges on import). Hollow it in the slicer / Fusion after import,
# exactly like thistle.py and cymatics.py. STL now; a STEP path can follow later.

import math
import os
import struct

try:
    import numpy as np
except ImportError:
    raise SystemExit(
        "coral.py needs numpy for the reaction-diffusion step.\n"
        "  Fedora:  sudo dnf install python3-numpy      (system-wide, no venv)\n"
        "  or:      pip install --user numpy\n"
        "Unlike thistle.py this does NOT need the build123d venv."
    )

HERE = os.path.dirname(os.path.abspath(__file__))  # write next to this script

# ---------- PARAMETERS (edit these) ----------
FORM          = "dome"      # RD forms: "dome" | "bulb" | "fingers"
                            # sculpted species: "bubble" | "cauliflower" | "greatstar"

# ---- body envelope (target ~ 180 x 180 x 60 mm for the dome) ----
BODY_RADIUS   = 90          # dome/bulb radius (x,y); base plate radius for "fingers"
BODY_HEIGHT   = 55          # dome/bulb height (z)
DOME_PROFILE  = "ellipse"   # dome cross-section: "ellipse" | "parabola" | "cosine" | "flat"
BULB_BOT_CUT  = 0.28        # "bulb": flatten the body below this height-fraction for a stand
BASE_HEIGHT   = 4           # thickness of the flat base disc under the body
PLATE_HEIGHT  = 8           # "fingers": thickness of the base plate

# ---- pads ("loops") : one raised, flat, copper-tape seat per capacitive electrode ----
N_PADS        = 12          # number of pads / protrusions (= your electrode count)
PAD_DIA       = 20          # flat seat diameter on top of each protrusion (mm)
PAD_HEIGHT    = 12          # how far each protrusion rises above the body (mm) — the "length"
PAD_SKIRT     = 10          # radial blend from seat edge back down to the body (mm)
PAD_BAND_LO   = 0.10        # pads live between these radius-fractions of the body (disc: r/R;
PAD_BAND_HI   = 0.92        #   bulb: latitude band u = z/c). Keeps them off the rim / pole.
PAD_LAYOUT    = "phyllotaxis"  # "phyllotaxis" (golden-angle) | "ring" (single circle)
PAD_STYLE     = "raised"    # DOME only: "raised" = flat copper seats on bumps ·
                            #   "flat" = pure brain-coral surface, pads are just marked
                            #   positions (RD still seeds them; stick copper on the ridges)
PAD_MODE      = "features"  # sculpted species: "features" = pads ARE the natural features
                            #   (bubbles / knobs / corallite cups) · "uniform" = N flat seats

# ---- reaction-diffusion (Gray-Scott) : the coral pattern ----
RD_AMOUNT     = 6.0         # amplitude of the coral ridges (mm). 0 = smooth body, just pads
RD_FEED       = 0.055       # feed rate  F  }  classic presets:
RD_KILL       = 0.062       # kill rate  k  }   coral/worms F.055 k.062 · spots F.030 k.062
RD_STEPS      = 5000        #                   maze F.029 k.057 · mitosis F.0367 k.0649
RD_GRID       = 200         # RD simulation grid resolution (200 good; 260 finer, slower)
RD_SEED_AT_PADS = True      # seed the pattern at pad centres so ridges radiate from pads
RD_NOISE      = 0.06        # random seed noise (breaks symmetry so ridges actually form)
RD_SEED       = 7           # RNG seed for the noise — change for a different coral
RD_RECTIFY    = "signed"    # "signed" = ridges + grooves · "absolute" = all-positive welts

# ---- broken-ridge shaping (post-process on the RD field; all 0 = smooth continuous coral) ----
RIDGE_BREAK   = 0.0         # 0 = continuous ridges · higher = ridges chopped into segments
RIDGE_LENGTH  = 22          # approximate segment / loop length (mm) — the break spacing
RIDGE_WIGGLE  = 0.0         # 0 = smooth ridges · higher = wigglier, meandering ridges

# ---- screen ----
SCREEN_MODE   = "center"    # "center" = round recess in the top · "pad" = recess in one pad · "none"
SCREEN_DIA    = 32
SCREEN_MARGIN = 4           # ring of flat material around the screen (mm)
SCREEN_DEPTH  = 6
SCREEN_PAD_IDX = 0          # which pad carries the screen when SCREEN_MODE = "pad"

# ---- mesh resolution ----
RES_RADIAL    = 150         # dome: rings from centre to rim
RES_ANGULAR   = 260         # dome/bulb/branch: samples around the circle
BRANCH_SEG    = 15          # bulb/fingers: rings up each branch

# ---- sculpted coral species (FORM = bubble / cauliflower / greatstar) ----
BUBBLE_COUNT  = 46          # "bubble": number of vesicles piled on the mound
BUBBLE_MIN    = 12          # smallest vesicle radius (mm)
BUBBLE_MAX    = 26          # largest vesicle radius (mm)
KNOB_AMP      = 18          # "cauliflower": lump depth (mm) of the knobby crust
KNOB_FREQ     = 3.4         # "cauliflower": lump frequency (higher = finer florets)
CORALLITE_DIA = 26          # "greatstar": outer diameter of each corallite cup (mm)
CORALLITE_H   = 7           # "greatstar": how far each cup rim stands off the boulder (mm)
CORALLITE_PIT = 5           # "greatstar": depth of the central pit below the rim (mm)
# ---------------------------------------------

R, C = float(BODY_RADIUS), float(BODY_HEIGHT)


# ============================================================================
#  Reaction-diffusion (Gray-Scott) — grows the coral pattern on a square grid
#  covering the unit disc [-1,1]^2.  Returns a normalised field in [0,1].
# ============================================================================
def run_reaction_diffusion(seeds_xy):
    """Gray-Scott on an N x N grid. `seeds_xy` are pad centres in disc coords
    (each component in [-1,1]); the pattern is nucleated there so ridges grow
    outward from the pads. Returns the V (activator) field, min-max normalised."""
    N = int(RD_GRID)
    U = np.ones((N, N), dtype=np.float64)
    V = np.zeros((N, N), dtype=np.float64)

    def stamp(cx, cy, rad, u_val, v_val):
        gi = (cx * 0.5 + 0.5) * (N - 1)      # column (x)
        gj = (cy * 0.5 + 0.5) * (N - 1)      # row    (y)
        r = max(1, int(rad * N))
        c0, c1 = max(0, int(gi) - r), min(N, int(gi) + r + 1)
        r0, r1 = max(0, int(gj) - r), min(N, int(gj) + r + 1)
        U[r0:r1, c0:c1] = u_val
        V[r0:r1, c0:c1] = v_val

    if RD_SEED_AT_PADS and seeds_xy:
        for (sx, sy) in seeds_xy:
            stamp(sx, sy, 0.045, 0.25, 0.5)
    else:                                    # a few central spots
        for (sx, sy) in [(0, 0), (0.3, 0.2), (-0.25, 0.3), (0.1, -0.35)]:
            stamp(sx, sy, 0.05, 0.25, 0.5)

    if RD_NOISE > 0:
        rs = np.random.RandomState(int(RD_SEED))
        V += RD_NOISE * rs.random_sample((N, N))
        np.clip(V, 0.0, 1.0, out=V)

    Du, Dv, F, k, dt = 0.16, 0.08, float(RD_FEED), float(RD_KILL), 1.0
    for _ in range(int(RD_STEPS)):
        lapU = (np.roll(U, 1, 0) + np.roll(U, -1, 0)
                + np.roll(U, 1, 1) + np.roll(U, -1, 1) - 4.0 * U)
        lapV = (np.roll(V, 1, 0) + np.roll(V, -1, 0)
                + np.roll(V, 1, 1) + np.roll(V, -1, 1) - 4.0 * V)
        uvv = U * V * V
        U += (Du * lapU - uvv + F * (1.0 - U)) * dt
        V += (Dv * lapV + uvv - (F + k) * V) * dt

    vmin, vmax = float(V.min()), float(V.max())
    if vmax - vmin < 1e-9:
        return np.zeros_like(V)
    return (V - vmin) / (vmax - vmin)


def make_sampler(field):
    """Bilinear sampler of an RD field over disc coords (x,y in [-1,1]) -> [0,1]."""
    N = field.shape[0]
    def rd_at(x, y):
        col = min(N - 1.0, max(0.0, (x * 0.5 + 0.5) * (N - 1)))
        row = min(N - 1.0, max(0.0, (y * 0.5 + 0.5) * (N - 1)))
        c0, r0 = int(col), int(row)
        c1, r1 = min(N - 1, c0 + 1), min(N - 1, r0 + 1)
        fc, fr = col - c0, row - r0
        return float(field[r0, c0] * (1 - fc) * (1 - fr) + field[r0, c1] * fc * (1 - fr)
                     + field[r1, c0] * (1 - fc) * fr + field[r1, c1] * fc * fr)
    return rd_at


# ============================================================================
#  Pad layout — where the N protrusions sit (disc coords for dome/fingers,
#  latitude band for bulb).  Returns unit positions in [-1,1] on the disc.
# ============================================================================
def pad_disc_positions():
    n = int(N_PADS)
    out = []
    lo, hi = PAD_BAND_LO, PAD_BAND_HI
    if PAD_LAYOUT == "ring":
        u = 0.5 * (lo + hi)
        for i in range(n):
            th = 2.0 * math.pi * i / max(1, n)
            out.append((u * math.cos(th), u * math.sin(th)))
    else:                                            # phyllotaxis (golden angle)
        ga = math.pi * (3.0 - math.sqrt(5.0))
        for i in range(n):
            u = math.sqrt((i + 0.5) / n)             # even area spread on the disc
            u = lo + (hi - lo) * u
            th = i * ga
            out.append((u * math.cos(th), u * math.sin(th)))
    return out


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def vnoise(x, y):
    """A cheap, smooth, deterministic value field in [0,1] (sum of incommensurate
    sines). Identical in coral.py and coral.html so the preview matches the STL.
    Used to break ridges into segments and to wiggle them."""
    s = (math.sin(x * 1.00 + 1.7) * math.cos(y * 1.13 - 0.9)
         + math.sin(x * 1.97 - 2.3) * math.cos(y * 2.09 + 1.4) * 0.6
         + math.sin(x * 3.71 + 0.5) * math.cos(y * 3.31 - 1.9) * 0.35)
    return 0.5 + 0.5 * s / 1.95


def apply_ridge_effects(base):
    """Wrap an RD sampler with broken-ridge + wiggle post-processing. `base(x,y)`
    returns the raw ridge field in [0,1]; the returned sampler warps the sample
    point (wiggle) and multiplies by a break-mask (segments of ~RIDGE_LENGTH)."""
    if RIDGE_BREAK <= 0.0 and RIDGE_WIGGLE <= 0.0:
        return base
    two_pi = 2.0 * math.pi
    gb = two_pi * R / max(1e-6, RIDGE_LENGTH)          # break-noise frequency
    gw = two_pi * R / max(1e-6, RIDGE_LENGTH * 1.6)    # wiggle-warp frequency (a bit coarser)
    thr = 0.35 + 0.45 * RIDGE_BREAK                    # more break -> higher cut threshold

    def sampler(x, y):
        if RIDGE_WIGGLE > 0.0:
            x += 0.18 * RIDGE_WIGGLE * (vnoise(x * gw + 11.2, y * gw + 3.7) - 0.5)
            y += 0.18 * RIDGE_WIGGLE * (vnoise(x * gw - 5.1, y * gw + 8.9) - 0.5)
        v = base(x, y)
        if RIDGE_BREAK > 0.0:
            n = vnoise(x * gb + 2.3, y * gb - 1.1)
            v *= smoothstep((n - (thr - 0.15)) / 0.30)  # cut the ridge where the mask is low
        return v
    return sampler


# ============================================================================
#  Shared mesh utilities
# ============================================================================
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


def frame(zdir):
    """An orthonormal (U, V, Z) frame with Z = zdir."""
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


# ============================================================================
#  FORM "dome" — brain-coral dome (cymatics-style displaced dome + pad field)
# ============================================================================
def dome_profile(u):
    if DOME_PROFILE == "ellipse":
        return C * math.sqrt(max(0.0, 1.0 - u * u))
    if DOME_PROFILE == "parabola":
        return C * (1.0 - u * u)
    if DOME_PROFILE == "cosine":
        return C * math.cos(0.5 * math.pi * u)
    return 0.0  # flat


def build_dome(rd_at, pads):
    na, nr = int(RES_ANGULAR), int(RES_RADIAL)
    rect = RD_RECTIFY == "absolute"
    delta = 1e-3 * R
    thetas = [2.0 * math.pi * i / na for i in range(na)]
    scr_pad = int(SCREEN_PAD_IDX) % max(1, len(pads))
    r_scr = SCREEN_DIA / 2.0

    raised = PAD_STYLE == "raised"

    def pad_field(x, y):
        """Return (extra height mm, seat-flatness 0..1) from the nearest pad(s).
        In "flat" style there are no raised seats — the surface stays pure brain
        coral and pads are only marked positions (a screen-in-pad still recesses)."""
        padH, seat, recess = 0.0, 0.0, 0.0
        rs = PAD_DIA / 2.0
        rf = PAD_DIA / 2.0 + PAD_SKIRT
        for idx, (px, py) in enumerate(pads):
            d = math.hypot(x - px, y - py) * R           # disc units -> mm
            if d >= rf:
                continue
            if raised:
                pl = 1.0 if d <= rs else (1.0 - smoothstep((d - rs) / (rf - rs)))
                if PAD_HEIGHT * pl > padH:
                    padH = PAD_HEIGHT * pl
                if d <= rs:
                    seat = 1.0                            # flat only on the seat
            if SCREEN_MODE == "pad" and idx == scr_pad and d <= r_scr:
                recess = -SCREEN_DEPTH                    # carve the screen recess in
                seat = 1.0
        return padH + recess, seat

    def field_mm(u, theta):
        x, y = u * math.cos(theta), u * math.sin(theta)
        padH, seat = pad_field(x, y)
        v = rd_at(x, y)
        ripple = RD_AMOUNT * v if rect else RD_AMOUNT * (v - 0.5) * 2.0
        return padH + ripple * (1.0 - seat)

    def surf_xyz(r, theta):
        u = r / R
        disp = field_mm(u, theta)
        h0 = dome_profile(u)
        dh = (dome_profile((r + delta) / R) - dome_profile((r - delta) / R)) / (2.0 * delta)
        inv = 1.0 / math.hypot(1.0, dh)
        rr = r + disp * (-dh) * inv
        z = BASE_HEIGHT + h0 + disp * inv
        return (rr * math.cos(theta), rr * math.sin(theta), max(z, 0.4))

    def disp_ring(r):
        return [surf_xyz(r, t) for t in thetas]

    def flat_ring(r, z):
        return [(r * math.cos(t), r * math.sin(t), z) for t in thetas]

    # ----- centre screen recess (carved into the polar top), else plain apex -----
    screen = SCREEN_MODE if (SCREEN_DIA > 0 and SCREEN_MODE == "center") else "none"
    r_bez = min(SCREEN_DIA / 2.0 + SCREEN_MARGIN, R * 0.9)
    r_scr_c = min(SCREEN_DIA / 2.0, r_bez - 0.5)
    shelf_z = BASE_HEIGHT + dome_profile(r_bez / R)
    floor_z = max(BASE_HEIGHT + 0.5, shelf_z - SCREEN_DEPTH)

    if screen == "center":
        center_pt = (0.0, 0.0, floor_z)
        stations = [flat_ring(r_scr_c, floor_z), flat_ring(r_scr_c, shelf_z),
                    flat_ring(r_bez, shelf_z), disp_ring(r_bez)]
        for j in range(1, nr + 1):
            r = R * j / nr
            if r > r_bez + 1e-6:
                stations.append(disp_ring(r))
        if math.hypot(*stations[-1][0][:2]) < R - 1e-6:
            stations.append(disp_ring(R))
    else:
        center_pt = surf_xyz(0.0, 0.0)
        stations = [disp_ring(R * j / nr) for j in range(1, nr + 1)]

    verts, tris = [], []

    def add_ring(ring):
        base = len(verts)
        verts.extend(ring)
        return base

    verts.append(center_pt)
    ring_bases = [add_ring(st) for st in stations]

    b0 = ring_bases[0]
    for i in range(na):
        tris.append((0, b0 + i, b0 + (i + 1) % na))
    for kk in range(len(ring_bases) - 1):
        a, b = ring_bases[kk], ring_bases[kk + 1]
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((a + i, a + i1, b + i1))
            tris.append((a + i, b + i1, b + i))

    rim = ring_bases[-1]                        # foot wall down to the flat base disc
    bot = len(verts)
    for i in range(na):
        x, y, _ = verts[rim + i]
        verts.append((x, y, 0.0))
    bc = len(verts)
    verts.append((0.0, 0.0, 0.0))
    for i in range(na):
        i1 = (i + 1) % na
        tris.append((rim + i, bot + i, bot + i1))
        tris.append((rim + i, bot + i1, rim + i1))
    for i in range(na):
        i1 = (i + 1) % na
        tris.append((bc, bot + i1, bot + i))
    return verts, tris


# ============================================================================
#  Wavy branch tube — shared by "bulb" and "fingers".  A tapered tube whose
#  radius wobbles with the RD field (coral grooves), a flat copper seat on top,
#  a buried flared skirt for a clean union, closed top & bottom (watertight).
# ============================================================================
def add_wavy_branch(verts, tris, origin, zdir, pad_xy, rd_at, recess=False):
    U, Vv, Z = frame(zdir)
    na = int(RES_ANGULAR)
    nl = int(BRANCH_SEG)
    L = float(PAD_HEIGHT)
    r_base = PAD_DIA / 2.0 + PAD_SKIRT * 0.5     # footprint where it meets the body
    r_top = PAD_DIA / 2.0                         # flat seat radius
    sink = 5.0                                    # skirt buried below the surface
    px, py = pad_xy
    disc_r = (r_base / R)                          # branch footprint in disc units
    r_scr = SCREEN_DIA / 2.0

    def wob(theta, t):
        q = disc_r * (0.35 + 0.65 * t)            # sample circle grows up the branch
        v = rd_at(px + q * math.cos(theta), py + q * math.sin(theta))
        amp = RD_AMOUNT * (1.0 - smoothstep((t - 0.75) / 0.25))  # fade out toward the seat
        return amp * (v - 0.5) * 2.0

    def ring(h, rad, wavy, t):
        pts = []
        for i in range(na):
            th = 2.0 * math.pi * i / na
            rr = rad + (wob(th, t) if wavy else 0.0)
            cx, sy = rr * math.cos(th), rr * math.sin(th)
            pts.append((origin[0] + cx * U[0] + sy * Vv[0] + h * Z[0],
                        origin[1] + cx * U[1] + sy * Vv[1] + h * Z[1],
                        origin[2] + cx * U[2] + sy * Vv[2] + h * Z[2]))
        return pts

    stations = []                                 # (heights, radii, wavy?) bottom -> top
    stations.append((-sink, r_base + 2.0, False, 0.0))     # buried skirt rim
    stations.append((0.0, r_base, True, 0.0))              # meets the surface
    for l in range(1, nl):
        t = l / float(nl)
        rad = r_base + (r_top - r_base) * smoothstep(t)
        stations.append((L * t, rad, True, t))
    stations.append((L, r_top, False, 1.0))               # flat seat rim

    verts_base = []
    for (h, rad, wavy, t) in stations:
        b = len(verts)
        verts.extend(ring(h, rad, wavy, t))
        verts_base.append(b)

    for kk in range(len(verts_base) - 1):
        a, b = verts_base[kk], verts_base[kk + 1]
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((a + i, a + i1, b + i1))
            tris.append((a + i, b + i1, b + i))

    # bottom cap (buried, closes the tube) — fan to a centre point
    bcen = len(verts)
    verts.append((origin[0] - sink * Z[0], origin[1] - sink * Z[1], origin[2] - sink * Z[2]))
    b = verts_base[0]
    for i in range(na):
        i1 = (i + 1) % na
        tris.append((bcen, b + i1, b + i))

    # top seat — flat disc, optionally with a screen recess
    top = verts_base[-1]
    if recess and r_scr > 0.5:
        depth = min(SCREEN_DEPTH, L - 1.0)
        inner = []                                # recess-wall ring at seat level
        floor = []                                # recess-floor ring, sunk by `depth`
        for i in range(na):
            th = 2.0 * math.pi * i / na
            cx, sy = r_scr * math.cos(th), r_scr * math.sin(th)
            top_pt = (origin[0] + cx * U[0] + sy * Vv[0] + L * Z[0],
                      origin[1] + cx * U[1] + sy * Vv[1] + L * Z[1],
                      origin[2] + cx * U[2] + sy * Vv[2] + L * Z[2])
            fl_pt = (top_pt[0] - depth * Z[0], top_pt[1] - depth * Z[1], top_pt[2] - depth * Z[2])
            inner.append(top_pt); floor.append(fl_pt)
        ib = len(verts); verts.extend(inner)
        fb = len(verts); verts.extend(floor)
        for i in range(na):                       # seat annulus  top rim -> recess rim
            i1 = (i + 1) % na
            tris.append((top + i, top + i1, ib + i1))
            tris.append((top + i, ib + i1, ib + i))
        for i in range(na):                       # recess wall down
            i1 = (i + 1) % na
            tris.append((ib + i, ib + i1, fb + i1))
            tris.append((ib + i, fb + i1, fb + i))
        fcen = len(verts)                          # recess floor
        verts.append((origin[0] + (L - depth) * Z[0], origin[1] + (L - depth) * Z[1],
                      origin[2] + (L - depth) * Z[2]))
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((fcen, fb + i, fb + i1))
    else:
        tcen = len(verts)
        verts.append((origin[0] + L * Z[0], origin[1] + L * Z[1], origin[2] + L * Z[2]))
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((tcen, top + i, top + i1))


# ============================================================================
#  FORM "bulb" — ellipsoid body + wavy coral branches (thistle-style)
# ============================================================================
def build_bulb(rd_at, pads):
    a, c = R, C
    na = int(RES_ANGULAR)
    nlat = max(24, int(RES_RADIAL) // 3)
    z_cut = -c * (1.0 - 2.0 * BULB_BOT_CUT)          # flat cut plane (ellipsoid z)
    verts, tris = [], []

    # RD texture on the bulb surface (mild, so the branches read as the pads)
    def disp_at(nx, ny):                              # sample RD by the x,y unit-normal
        return 0.45 * RD_AMOUNT * (rd_at(nx, ny) - 0.5) * 2.0

    lat_lo = math.acos(max(-1.0, min(1.0, z_cut / c)))  # polar angle at the cut (from +z)
    top = (0.0, 0.0, BASE_HEIGHT + 2 * c * 1.0)         # placeholder, replaced below

    rings = []
    for j in range(nlat + 1):
        phi = lat_lo * j / nlat                       # 0 = north pole -> lat_lo = cut
        ring = []
        for i in range(na):
            th = 2.0 * math.pi * i / na
            sx, sy, sz = math.sin(phi) * math.cos(th), math.sin(phi) * math.sin(th), math.cos(phi)
            x, y, z = a * sx, a * sy, c * sz
            nx, ny, nz = x / (a * a), y / (a * a), z / (c * c)
            nl = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            nx, ny, nz = nx / nl, ny / nl, nz / nl
            d = disp_at(sx, sy)
            ring.append((x + nx * d, y + ny * d, BASE_HEIGHT + (z - z_cut) + nz * d))
        rings.append(ring)

    # north pole point
    pole = (0.0, 0.0, BASE_HEIGHT + (c - z_cut) + disp_at(0.0, 0.0))
    verts.append(pole)
    ring_bases = []
    for ring in rings:
        b = len(verts); verts.extend(ring); ring_bases.append(b)
    b0 = ring_bases[0]
    for i in range(na):
        tris.append((0, b0 + i, b0 + (i + 1) % na))
    for kk in range(len(ring_bases) - 1):
        a2, b2 = ring_bases[kk], ring_bases[kk + 1]
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((a2 + i, a2 + i1, b2 + i1))
            tris.append((a2 + i, b2 + i1, b2 + i))

    # flat base disc at z = 0 under the cut rim
    rim = ring_bases[-1]
    bot = len(verts)
    for i in range(na):
        x, y, _ = verts[rim + i]
        verts.append((x, y, 0.0))
    bc = len(verts); verts.append((0.0, 0.0, 0.0))
    for i in range(na):
        i1 = (i + 1) % na
        tris.append((rim + i, bot + i, bot + i1))
        tris.append((rim + i, bot + i1, rim + i1))
    for i in range(na):
        i1 = (i + 1) % na
        tris.append((bc, bot + i1, bot + i))

    # branches: map each disc pad position to a latitude band on the bulb
    lo, hi = PAD_BAND_LO, PAD_BAND_HI
    for idx, (px, py) in enumerate(pads):
        rr = math.hypot(px, py)                       # 0..1 -> latitude fraction
        frac = 0.0 if rr < 1e-9 else rr
        u = hi - (hi - lo) * frac                      # centre pad -> high on the pole
        phi = lat_lo * (1.0 - u)                       # small phi = near pole
        th = math.atan2(py, px)
        sx, sy, sz = math.sin(phi) * math.cos(th), math.sin(phi) * math.sin(th), math.cos(phi)
        x, y, z = a * sx, a * sy, c * sz
        nx, ny, nz = x / (a * a), y / (a * a), z / (c * c)
        nl = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx, ny, nz = nx / nl, ny / nl, nz / nl
        origin = (x, y, BASE_HEIGHT + (z - z_cut))
        recess = (SCREEN_MODE == "pad" and idx == int(SCREEN_PAD_IDX) % max(1, len(pads)))
        add_wavy_branch(verts, tris, origin, (nx, ny, nz), (px, py), rd_at, recess)

    return verts, tris


# ============================================================================
#  FORM "fingers" — base plate + upright wavy finger-coral tubes
# ============================================================================
def build_fingers(rd_at, pads):
    na = int(RES_ANGULAR)
    verts, tris = [], []
    ph = float(PLATE_HEIGHT)

    # round base plate: top disc + rim wall + bottom disc
    topc = len(verts); verts.append((0.0, 0.0, ph))
    tb = len(verts)
    for i in range(na):
        th = 2.0 * math.pi * i / na
        verts.append((R * math.cos(th), R * math.sin(th), ph))
    botc = len(verts); verts.append((0.0, 0.0, 0.0))
    bb = len(verts)
    for i in range(na):
        th = 2.0 * math.pi * i / na
        verts.append((R * math.cos(th), R * math.sin(th), 0.0))
    for i in range(na):
        i1 = (i + 1) % na
        tris.append((topc, tb + i, tb + i1))          # plate top
        tris.append((botc, bb + i1, bb + i))          # plate bottom
        tris.append((tb + i, bb + i, bb + i1))        # rim wall
        tris.append((tb + i, bb + i1, tb + i1))

    for idx, (px, py) in enumerate(pads):
        origin = (px * R, py * R, ph)
        recess = (SCREEN_MODE == "pad" and idx == int(SCREEN_PAD_IDX) % max(1, len(pads)))
        add_wavy_branch(verts, tris, origin, (0.0, 0.0, 1.0), (px, py), rd_at, recess)

    return verts, tris


# ============================================================================
#  Driver
# ============================================================================
# ============================================================================
#  Sculpted (non-RD) coral species — bubble, cauliflower, great-star.
#  These share a small mesh toolkit: closed UV spheres, a flat-bottomed dome
#  boulder, a revolve-a-profile routine (corallite cups & pad seats).
# ============================================================================
def add_uv_sphere(verts, tris, cx, cy, cz, r, na=48, nlat=24, disp=None):
    """A closed, watertight UV sphere. disp(theta, phi) -> extra radius (mm)."""
    north = len(verts); verts.append((cx, cy, cz + r + (disp(0.0, 0.0) if disp else 0.0)))
    ring_bases = []
    for i in range(1, nlat):
        phi = math.pi * i / nlat
        sp, cp = math.sin(phi), math.cos(phi)
        b = len(verts)
        for j in range(na):
            th = 2.0 * math.pi * j / na
            rr = r + (disp(th, phi) if disp else 0.0)
            verts.append((cx + rr * sp * math.cos(th), cy + rr * sp * math.sin(th), cz + rr * cp))
        ring_bases.append(b)
    south = len(verts); verts.append((cx, cy, cz - r - (disp(0.0, math.pi) if disp else 0.0)))
    b0 = ring_bases[0]
    for j in range(na):
        tris.append((north, b0 + j, b0 + (j + 1) % na))
    for k in range(len(ring_bases) - 1):
        a, b = ring_bases[k], ring_bases[k + 1]
        for j in range(na):
            j1 = (j + 1) % na
            tris.append((a + j, a + j1, b + j1)); tris.append((a + j, b + j1, b + j))
    bl = ring_bases[-1]
    for j in range(na):
        tris.append((south, bl + (j + 1) % na, bl + j))


def add_dome_body(verts, tris, R, H, na, nrings, base_z, disp=None):
    """Upper half-ellipsoid boulder, closed with a foot wall + flat base disc at
    z=0. disp(theta, phi) adds mm outward (fade it near the rim to keep the foot
    circular). Returns nothing; appends a watertight solid."""
    apex = len(verts); verts.append((0.0, 0.0, base_z + H + (disp(0.0, 0.0) if disp else 0.0)))
    ring_bases = []
    for i in range(1, nrings + 1):
        phi = 0.5 * math.pi * i / nrings
        sp, cp = math.sin(phi), math.cos(phi)
        b = len(verts)
        for j in range(na):
            th = 2.0 * math.pi * j / na
            d = disp(th, phi) if disp else 0.0
            rr = R * sp + d * sp
            verts.append((rr * math.cos(th), rr * math.sin(th), base_z + H * cp + d * cp))
        ring_bases.append(b)
    b0 = ring_bases[0]
    for j in range(na):
        tris.append((apex, b0 + j, b0 + (j + 1) % na))
    for k in range(len(ring_bases) - 1):
        a, b = ring_bases[k], ring_bases[k + 1]
        for j in range(na):
            j1 = (j + 1) % na
            tris.append((a + j, a + j1, b + j1)); tris.append((a + j, b + j1, b + j))
    rim = ring_bases[-1]                              # phi=pi/2 rim sits at z=base_z
    bot = len(verts)
    for j in range(na):
        x, y, _ = verts[rim + j]; verts.append((x, y, 0.0))
    bc = len(verts); verts.append((0.0, 0.0, 0.0))
    for j in range(na):
        j1 = (j + 1) % na
        tris.append((rim + j, bot + j, bot + j1)); tris.append((rim + j, bot + j1, rim + j1))
    for j in range(na):
        tris.append((bc, bot + (j + 1) % na, bot + j))


def revolve_into(verts, tris, profile, origin, zdir, seg=44):
    """Revolve a (radius, height) profile about zdir at origin; radius-0 points
    become axis apices. Appends a closed solid (same idea as cymatics.revolve)."""
    def norm(v):
        L = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) or 1.0
        return (v[0] / L, v[1] / L, v[2] / L)
    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    Z = norm(zdir); ax = (0.0, 0.0, 1.0) if abs(Z[2]) < 0.9 else (1.0, 0.0, 0.0)
    U = norm(cross(ax, Z)); V = cross(Z, U)
    def pt(rad, h, phi):
        cr, sr = rad * math.cos(phi), rad * math.sin(phi)
        return (origin[0] + cr * U[0] + sr * V[0] + h * Z[0],
                origin[1] + cr * U[1] + sr * V[1] + h * Z[1],
                origin[2] + cr * U[2] + sr * V[2] + h * Z[2])
    rows = []
    for (rad, h) in profile:
        if rad < 1e-6:
            rows.append(("apex", len(verts))); verts.append(pt(0.0, h, 0.0))
        else:
            base = len(verts)
            for k in range(seg):
                verts.append(pt(rad, h, 2.0 * math.pi * k / seg))
            rows.append(("ring", base))
    for k in range(len(rows) - 1):
        (ka, ia), (kb, ib) = rows[k], rows[k + 1]
        if ka == "apex" and kb == "ring":
            for i in range(seg):
                tris.append((ia, ib + i, ib + (i + 1) % seg))
        elif ka == "ring" and kb == "apex":
            for i in range(seg):
                tris.append((ia + i, ib, ia + (i + 1) % seg))
        else:
            for i in range(seg):
                i1 = (i + 1) % seg
                tris.append((ia + i, ia + i1, ib + i1)); tris.append((ia + i, ib + i1, ib + i))


def dome_point_normal(u, theta, R, H, base_z):
    """Surface point + outward unit normal of the dome boulder at (u in [0,1], theta)."""
    phi = 0.5 * math.pi * u
    sp, cp = math.sin(phi), math.cos(phi)
    x, y, z = R * sp * math.cos(theta), R * sp * math.sin(theta), base_z + H * cp
    nx, ny, nz = x / (R * R), y / (R * R), (z - base_z) / (H * H) if H > 0 else 1.0
    L = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return (x, y, z), (nx / L, ny / L, nz / L)


def add_pad_seat(verts, tris, point, normal, dia, height, seg=44, sink=3.0):
    """A raised, flat-topped round copper seat on the surface (uniform-pad mode)."""
    r = dia / 2.0
    revolve_into(verts, tris, [(0.0, -sink), (r + 2.0, -sink), (r + 2.0, 0.0),
                               (r, height), (0.0, height)], point, normal, seg)


def build_bubble(pads):
    """Bubble coral: vesicles piled on a low mound. Pads (features) = the biggest
    bubbles; (uniform) = flat seats on the mound. Screen recesses the base."""
    na_b = max(28, RES_ANGULAR // 8); nlat_b = max(16, RES_RADIAL // 8)
    verts, tris = [], []
    Rm, Hm = R * 0.86, C * 0.55
    add_dome_body(verts, tris, Rm, Hm, max(56, RES_ANGULAR // 4), max(28, RES_RADIAL // 4), BASE_HEIGHT)
    rng = np.random.RandomState(int(RD_SEED))
    n = int(BUBBLE_COUNT); ga = math.pi * (3.0 - math.sqrt(5.0))
    for i in range(n):
        u = math.sqrt((i + 0.5) / n); th = i * ga
        (px, py, pz), _ = dome_point_normal(u, th, Rm, Hm, BASE_HEIGHT)
        br = BUBBLE_MIN + (BUBBLE_MAX - BUBBLE_MIN) * (1.0 - u) * rng.uniform(0.7, 1.12)
        add_uv_sphere(verts, tris, px, py, pz + br * 0.45, br, na_b, nlat_b)
    if PAD_MODE == "uniform":
        for (dx, dy) in pads:
            u = min(0.98, math.hypot(dx, dy)); th = math.atan2(dy, dx)
            pt, nrm = dome_point_normal(u, th, Rm, Hm, BASE_HEIGHT)
            add_pad_seat(verts, tris, pt, nrm, PAD_DIA, PAD_HEIGHT)
    _carve_base_screen(verts, tris)
    return verts, tris


def build_cauliflower(pads):
    """Cauliflower coral: a lumpy knobby mound. Pads = raised flat seats on the
    florets (both modes place N_PADS seats; features clusters them on the crown)."""
    na = max(96, RES_ANGULAR // 2); nrings = max(60, RES_RADIAL // 2)
    verts, tris = [], []
    def disp(th, phi):
        sx, sy, sz = math.sin(phi) * math.cos(th), math.sin(phi) * math.sin(th), math.cos(phi)
        f = KNOB_FREQ
        n = (vnoise(sx * f * 3.0 + 1.1, sy * f * 3.0 - 0.7) * 0.6
             + vnoise(sy * f * 6.0 + 3.2, sz * f * 6.0 + 1.4) * 0.3
             + vnoise(sz * f * 11.0 - 2.1, sx * f * 11.0 + 0.5) * 0.1)
        fade = smoothstep((0.5 * math.pi - phi) / 0.30)      # ribs vanish at the foot rim
        return KNOB_AMP * n * fade
    add_dome_body(verts, tris, R * 0.9, C, na, nrings, BASE_HEIGHT, disp)
    for (dx, dy) in pads:
        u = min(0.9, math.hypot(dx, dy)); th = math.atan2(dy, dx)
        pt, nrm = dome_point_normal(u, th, R * 0.9, C, BASE_HEIGHT)
        pt = (pt[0] + nrm[0] * KNOB_AMP * 0.5, pt[1] + nrm[1] * KNOB_AMP * 0.5, pt[2] + nrm[2] * KNOB_AMP * 0.5)
        add_pad_seat(verts, tris, pt, nrm, PAD_DIA, PAD_HEIGHT * 0.7)
    _carve_top_screen(verts, tris, R * 0.9, C)
    return verts, tris


def build_greatstar(pads):
    """Great star coral: a boulder studded with corallite cups (raised rings with
    a central pit). Pads (features) = the cups; (uniform) = flat seats."""
    na = max(80, RES_ANGULAR // 3); nrings = max(50, RES_RADIAL // 3)
    Hb = C * 0.72
    verts, tris = [], []
    add_dome_body(verts, tris, R, Hb, na, nrings, BASE_HEIGHT)
    ro = CORALLITE_DIA / 2.0; wall = max(2.0, ro * 0.28); ri = ro - wall
    sink = 4.0
    if PAD_MODE == "features":
        for (dx, dy) in pads:
            u = min(0.9, math.hypot(dx, dy)); th = math.atan2(dy, dx)
            pt, nrm = dome_point_normal(u, th, R, Hb, BASE_HEIGHT)
            revolve_into(verts, tris, [(0.0, CORALLITE_H - CORALLITE_PIT), (ri, CORALLITE_H - CORALLITE_PIT),
                                       (ri, CORALLITE_H), (ro, CORALLITE_H), (ro, -sink), (0.0, -sink)],
                         pt, nrm, max(28, RES_ANGULAR // 6))
    else:
        for (dx, dy) in pads:
            u = min(0.9, math.hypot(dx, dy)); th = math.atan2(dy, dx)
            pt, nrm = dome_point_normal(u, th, R, Hb, BASE_HEIGHT)
            add_pad_seat(verts, tris, pt, nrm, PAD_DIA, PAD_HEIGHT * 0.6)
    _carve_top_screen(verts, tris, R, Hb)
    return verts, tris


def _carve_top_screen(verts, tris, R, H):
    """A round screen bezel + recess standing on the crown (a separate solid)."""
    if SCREEN_MODE not in ("top", "center") or SCREEN_DIA <= 0:
        return
    ro = SCREEN_DIA / 2.0 + SCREEN_MARGIN; ri = SCREEN_DIA / 2.0
    hr = max(4.0, SCREEN_DEPTH + 2.0)
    revolve_into(verts, tris, [(0.0, hr - SCREEN_DEPTH), (ri, hr - SCREEN_DEPTH), (ri, hr),
                               (ro, hr), (ro, -3.0), (0.0, -3.0)],
                 (0.0, 0.0, BASE_HEIGHT + H), (0.0, 0.0, 1.0), max(36, RES_ANGULAR // 5))


def _carve_base_screen(verts, tris):
    """A round screen recess pushed up into the flat base underside."""
    if SCREEN_MODE not in ("base",) or SCREEN_DIA <= 0:
        return
    ri = SCREEN_DIA / 2.0; ro = SCREEN_DIA / 2.0 + SCREEN_MARGIN
    revolve_into(verts, tris, [(0.0, SCREEN_DEPTH), (ri, SCREEN_DEPTH), (ri, 0.0),
                               (ro, 0.0), (ro, -0.1), (0.0, -0.1)],
                 (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), max(36, RES_ANGULAR // 5))


def build():
    pads = pad_disc_positions()
    if FORM == "bubble":
        return build_bubble(pads)
    if FORM == "cauliflower":
        return build_cauliflower(pads)
    if FORM == "greatstar":
        return build_greatstar(pads)
    print(f"  {N_PADS} pads, running reaction-diffusion "
          f"({RD_GRID}^2 x {RD_STEPS} steps, F={RD_FEED} k={RD_KILL})...")
    field = run_reaction_diffusion(pads)
    rd_at = apply_ridge_effects(make_sampler(field))
    if FORM == "bulb":
        return build_bulb(rd_at, pads)
    if FORM == "fingers":
        return build_fingers(rd_at, pads)
    return build_dome(rd_at, pads)


if __name__ == "__main__":
    verts, tris = build()
    stl_path = os.path.join(HERE, "coral.stl")
    write_stl_binary(stl_path, verts, tris, f"coral-{FORM} (Omniphone) reaction-diffusion")
    print(f"wrote {stl_path}  (form={FORM}, {len(verts)} verts, {len(tris)} triangles)")
