# mushroom.py  —  procedural mushroom enclosure generator for the Omniphone
#
# Run:   python3 mushroom.py             (needs numpy — system-wide is fine, NO build123d venv)
# Out:   mushroom.stl                     -> send straight to the slicer / 3D printer,
#                                            or import into Fusion 360 / FreeCAD as a mesh body.
#
# Shape: a fungus-shaped enclosure for the Omniphone capacitive touch instrument. One SPECIES
# parameter picks one of five fungal bodies, each carrying DISTINCT PADS (copper-tape touch
# zones) and a round SCREEN recess:
#
#   "shimeji"        — a bunched CLUSTER of ~10-16 small mushrooms: long slender tapered stems
#                      splaying out of a common base clump, each topped by a small squashed
#                      cap. The caps are the pads.
#   "amanita"        — the classic single fly-agaric: a bulbous-based stem with a flared ring
#                      and a wide domed cap; the cap top carries scattered WARTS (golden-angle
#                      bumps) and radial gill ridges under the rim. Cap seats are the pads.
#   "chestnutcluster"— shimeji's fatter cousin: fewer, thicker, tightly-packed convex caps on
#                      short stubby stems (a honey / Pholiota clump). Caps are the pads.
#   "morel"          — an elongated ovoid cap wrapped in a HONEYCOMB of ridges and pits (a
#                      cellular / Voronoi field from golden-angle seeds), on a short stem. The
#                      strangely-shaped cells are the pads.
#   "lionsmane"      — Hericium: a rounded body whose lower flank bristles with downward-hanging
#                      tapered SPINES (icicle teeth); the smooth crown carries the pads.
#
# Geometry approach (robust + watertight): every part is a *closed* solid of revolution or a
# closed displaced UV-dome — stems/cones are tapered revolutions, caps are ellipsoid domes, the
# honeycomb / warts / gills are surface displacements on a dome. Clusters are a base clump plus
# N stem+cap units, each its own closed solid; overlapping separate closed solids are fine — the
# slicer / Fusion unions them on import, exactly like coral.py's overlapping branch solids. A
# flat base disc under every body lets it stand. Screen recesses are meshed straight into the
# surface (bezel ring + pocket wall + floor) so the result stays watertight.
#
# The manifold check in __main__ verifies every part: each undirected edge shared by exactly two
# triangles. Hollow it in the slicer / Fusion after import, exactly like thistle.py / coral.py.

import math
import os
import struct

try:
    import numpy as np
except ImportError:
    raise SystemExit(
        "mushroom.py needs numpy for the honeycomb / wart displacement fields.\n"
        "  Fedora:  sudo dnf install python3-numpy      (system-wide, no venv)\n"
        "  or:      pip install --user numpy\n"
        "Unlike thistle.py this does NOT need the build123d venv."
    )

HERE = os.path.dirname(os.path.abspath(__file__))  # write next to this script

# ---------- PARAMETERS (edit these) ----------
SPECIES       = "amanita"   # "shimeji" | "amanita" | "chestnutcluster" | "morel" | "lionsmane"

# ---- body envelope (target span ~ 120-200 mm) ----
BODY_RADIUS   = 70          # main body / cap / clump radius (mm)  -> ~140 mm span
BODY_HEIGHT   = 60          # main body / cap height (mm)
BASE_HEIGHT   = 6           # nominal flat-base thickness (kept for the shared schema)

# ---- pads : the capacitive copper-tape touch zones ----
N_PADS        = 12          # number of pads (= your electrode count)
PAD_MODE      = "features"  # "features" = pads ARE the natural features (caps / warts / cells /
                            #   crown seats) · "uniform" = ignore features, lay N flat round seats
PAD_DIA       = 16          # pad seat diameter (mm)
PAD_HEIGHT    = 6           # how far a "uniform" / seat puck rises above the surface (mm)

# ---- screen : one round recess ----
SCREEN_MODE   = "top"       # "top" = recess in the crown · "base" = recess in the underside · "none"
SCREEN_DIA    = 34          # screen diameter (mm)
SCREEN_MARGIN = 4           # flat bezel ring around the screen (mm)
SCREEN_DEPTH  = 6           # recess depth (mm)

# ---- mesh resolution ----
RES_ANGULAR   = 96          # samples around a body / cap (the big domes)
RES_VERTICAL  = 44          # rings up a body / cap dome

# ---- species-specific ----
CLUSTER_COUNT = 13          # shimeji / chestnut: number of mushrooms in the clump
STEM_LEN      = 55          # nominal stem length (mm)
STEM_DIA      = 16          # nominal stem diameter (mm)
CAP_DIA       = 34          # nominal small-cap diameter (cluster caps, morel width factor)
CAP_RISE      = 16          # nominal small-cap height (mm)
WART_COUNT    = 24          # amanita: scattered cap warts (golden angle)
GILL_COUNT    = 28          # amanita: radial gill ridges under the cap
CELL_COUNT    = 60          # morel: honeycomb seed cells
RIDGE_AMP     = 5           # morel: honeycomb ridge height / pit depth (mm)
SPINE_COUNT   = 110         # lionsmane: hanging spines
SPINE_LEN     = 22          # lionsmane: spine length (mm)
SPINE_DIA     = 10          # lionsmane: spine base diameter (mm)
# ---------------------------------------------

GA = math.pi * (3.0 - math.sqrt(5.0))   # golden angle
RES_PAD   = 24                          # facets around a pad puck
RES_STEM  = 40                          # facets around a stem / cone
RES_SPINE = 12                          # facets around a spine


# ============================================================================
#  Shared helpers (self-contained copies, in coral.py's voice)
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


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def vnoise(x, y):
    """A cheap, smooth, deterministic value field in [0,1] (sum of incommensurate sines)."""
    s = (math.sin(x * 1.00 + 1.7) * math.cos(y * 1.13 - 0.9)
         + math.sin(x * 1.97 - 2.3) * math.cos(y * 2.09 + 1.4) * 0.6
         + math.sin(x * 3.71 + 0.5) * math.cos(y * 3.31 - 1.9) * 0.35)
    return 0.5 + 0.5 * s / 1.95


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


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def to_world(o, U, V, Z, p):
    """Local (x,y,z) in the (U,V,Z) frame at origin `o` -> world coordinates."""
    return (o[0] + p[0] * U[0] + p[1] * V[0] + p[2] * Z[0],
            o[1] + p[0] * U[1] + p[1] * V[1] + p[2] * Z[1],
            o[2] + p[0] * U[2] + p[1] * V[2] + p[2] * Z[2])


def normalize(v):
    L = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if L < 1e-12:
        return None
    return (v[0] / L, v[1] / L, v[2] / L)


# ============================================================================
#  Low-level mesh primitives — each APPENDS a single *closed* (2-manifold) solid
#  into shared verts/tris lists.  Overlapping closed solids union in the slicer.
#  Winding is not enforced; watertightness only needs every edge shared by two
#  triangles, and the slicer / Fusion fixes normals on import.
# ============================================================================
def _strip(tris, a, b, na):
    """Quad strip between two rings of `na` verts (bases a and b)."""
    for i in range(na):
        i1 = (i + 1) % na
        tris.append((a + i, a + i1, b + i1))
        tris.append((a + i, b + i1, b + i))


def _fan(verts, tris, ring_base, na, center_pt):
    """Triangle-fan cap from a ring to a single centre point."""
    c = len(verts)
    verts.append(center_pt)
    for i in range(na):
        i1 = (i + 1) % na
        tris.append((c, ring_base + i, ring_base + i1))


def _pocket(verts, tris, origin, U, V, Z, ring_base, na, z0, r_scr, depth, into, r_outer):
    """Carve a round screen pocket, closing an outer boundary ring (`ring_base`) that sits at
    local height z0 and radius ~r_outer. A flat bezel annulus runs inward to a clean r_scr
    circle, then a wall goes `into*depth` deep, then a floor disc closes it. Watertight."""
    r_scr = min(r_scr, max(1.0, r_outer * 0.8))          # keep the mouth inside the boundary
    mouth = len(verts)                                   # recess mouth (bezel inner edge)
    for i in range(na):
        th = 2.0 * math.pi * i / na
        verts.append(to_world(origin, U, V, Z, (r_scr * math.cos(th), r_scr * math.sin(th), z0)))
    floor = len(verts)                                   # recess floor rim
    zf = z0 + into * depth
    for i in range(na):
        th = 2.0 * math.pi * i / na
        verts.append(to_world(origin, U, V, Z, (r_scr * math.cos(th), r_scr * math.sin(th), zf)))
    _strip(tris, ring_base, mouth, na)                   # flat bezel / base annulus
    _strip(tris, mouth, floor, na)                       # pocket wall
    _fan(verts, tris, floor, na, to_world(origin, U, V, Z, (0.0, 0.0, zf)))  # floor disc


def add_revolution(verts, tris, origin, zdir, profile, na,
                   bottom_screen=None, top_screen=None):
    """A closed solid of revolution. `profile` is a list of (radius, height) from bottom to top;
    an endpoint radius of 0 becomes a pole. Optional round screen pockets carve the flat bottom
    (into +Z) or the flat top (into -Z). Used for stems, cones, rings, pads, spines."""
    U, V, Z = frame(zdir)
    eps = 1e-6
    bases = []                                           # ("ring", idx) or ("pole", height)
    for (r, h) in profile:
        if r < eps:
            bases.append(("pole", h))
        else:
            b = len(verts)
            for i in range(na):
                th = 2.0 * math.pi * i / na
                verts.append(to_world(origin, U, V, Z, (r * math.cos(th), r * math.sin(th), h)))
            bases.append(("ring", b, h, r))

    # connect consecutive profile stations
    for k in range(len(profile) - 1):
        a, b = bases[k], bases[k + 1]
        if a[0] == "ring" and b[0] == "ring":
            _strip(tris, a[1], b[1], na)
        elif a[0] == "pole" and b[0] == "ring":
            _fan(verts, tris, b[1], na, to_world(origin, U, V, Z, (0.0, 0.0, a[1])))
        elif a[0] == "ring" and b[0] == "pole":
            _fan(verts, tris, a[1], na, to_world(origin, U, V, Z, (0.0, 0.0, b[1])))

    # close the bottom end (profile[0]) if it is a ring
    if bases[0][0] == "ring":
        _, rb, h0, r0 = bases[0]
        if bottom_screen is not None:
            _pocket(verts, tris, origin, U, V, Z, rb, na, h0,
                    bottom_screen[0], bottom_screen[1], +1.0, r0)
        else:
            _fan(verts, tris, rb, na, to_world(origin, U, V, Z, (0.0, 0.0, h0)))
    # close the top end (profile[-1]) if it is a ring
    if bases[-1][0] == "ring":
        _, rt, hT, rT = bases[-1]
        if top_screen is not None:
            _pocket(verts, tris, origin, U, V, Z, rt, na, hT,
                    top_screen[0], top_screen[1], -1.0, rT)
        else:
            _fan(verts, tris, rt, na, to_world(origin, U, V, Z, (0.0, 0.0, hT)))


def dome_pn(origin, zdir, rad, height, u, theta):
    """Point + outward unit normal on an ellipsoid dome, parameterised by u in [0,1]
    (u=0 crown pole, u=1 equatorial rim) and angle theta. Used to seat pads / roots / spines."""
    U, V, Z = frame(zdir)
    hp = math.pi / 2.0
    r = rad * math.sin(u * hp)
    z = height * math.cos(u * hp)
    pt = to_world(origin, U, V, Z, (r * math.cos(theta), r * math.sin(theta), z))
    if u < 1e-4:
        return pt, Z
    # meridian normal: horizontal comp -z'(u)*r, vertical comp r'(u)*r  (see coral.py disp maths)
    nh = height * hp * math.sin(u * hp) * r
    nz = rad * hp * math.cos(u * hp) * r
    nl = math.hypot(nh, nz) or 1.0
    nh, nz = nh / nl, nz / nl
    nrm = (nh * math.cos(theta) * U[0] + nh * math.sin(theta) * V[0] + nz * Z[0],
           nh * math.cos(theta) * U[1] + nh * math.sin(theta) * V[1] + nz * Z[1],
           nh * math.cos(theta) * U[2] + nh * math.sin(theta) * V[2] + nz * Z[2])
    return pt, (normalize(nrm) or Z)


def add_dome(verts, tris, origin, zdir, rad, height, na, nrings,
             top="pole", bottom="disc", disp=None, seat_r=0.0, screen=None, fade=0.12):
    """A closed, optionally surface-displaced ellipsoid dome (crown at +Z, rim at the origin
    plane). `disp(u, theta)` returns an outward displacement in mm (warts, honeycomb, gills);
    it is faded to zero at the top/bottom edges so closures stay clean.

      top    : "pole"   crown closes to a point
               "seat"   crown truncated to a flat copper seat of radius seat_r
               "screen" crown truncated to a bezel + round pocket  (screen = (r_bez, r_scr, depth))
      bottom : "disc"   flat underside disc (a stand)
               "screen" flat underside with a round pocket carved up into the body
    """
    U, V, Z = frame(zdir)
    hp = math.pi / 2.0

    def rho(u):  return rad * math.sin(u * hp)
    def zz(u):   return height * math.cos(u * hp)
    def rhod(u): return rad * hp * math.cos(u * hp)
    def zzd(u):  return -height * hp * math.sin(u * hp)

    u_bot = 1.0
    if top == "seat":
        u_top = math.asin(clamp(seat_r / rad, 0.0, 1.0)) / hp
    elif top == "screen":
        r_bez = screen[0]
        u_top = math.asin(clamp(r_bez / rad, 0.0, 1.0)) / hp
    else:
        u_top = 0.0

    def ringpt(u, theta):
        r, z = rho(u), zz(u)
        if disp is not None:
            d = disp(u, theta) * smoothstep((u - u_top) / fade) * smoothstep((u_bot - u) / fade)
            if d != 0.0:
                nh, nz = -zzd(u) * r, rhod(u) * r        # outward normal (rho,z) components
                nl = math.hypot(nh, nz) or 1.0
                r += d * nh / nl
                z += d * nz / nl
        return to_world(origin, U, V, Z, (r * math.cos(theta), r * math.sin(theta), z))

    us = [u_top + (u_bot - u_top) * j / nrings for j in range(nrings + 1)]
    start = 1 if top == "pole" else 0                    # drop the degenerate rho=0 crown ring
    ring_bases = []
    for j in range(start, nrings + 1):
        b = len(verts)
        for i in range(na):
            verts.append(ringpt(us[j], 2.0 * math.pi * i / na))
        ring_bases.append(b)
    for k in range(len(ring_bases) - 1):
        _strip(tris, ring_bases[k], ring_bases[k + 1], na)

    # ---- crown closure ----
    if top == "pole":
        _fan(verts, tris, ring_bases[0], na, to_world(origin, U, V, Z, (0.0, 0.0, zz(0.0))))
    elif top == "seat":
        _fan(verts, tris, ring_bases[0], na, to_world(origin, U, V, Z, (0.0, 0.0, zz(u_top))))
    elif top == "screen":
        r_bez, r_scr, depth = screen
        _pocket(verts, tris, origin, U, V, Z, ring_bases[0], na, zz(u_top), r_scr, depth, -1.0, r_bez)

    # ---- underside closure ----
    if bottom == "disc":
        _fan(verts, tris, ring_bases[-1], na, to_world(origin, U, V, Z, (0.0, 0.0, zz(u_bot))))
    elif bottom == "screen":
        _, r_scr, depth = screen
        _pocket(verts, tris, origin, U, V, Z, ring_bases[-1], na, zz(u_bot), r_scr, depth, +1.0, rad)


def add_puck(verts, tris, center, zdir, dia, height, na=RES_PAD, sink=3.0):
    """A short flat-topped cylinder (a raised copper-tape seat) rooted at `center`, its base
    sunk `sink` mm below the surface so it unions cleanly with the body it sits on."""
    r = dia / 2.0
    add_revolution(verts, tris, center, zdir, [(r, -sink), (r, height)], na)


# ============================================================================
#  Pad / seed layout helpers
# ============================================================================
def phyllo_area(n, lo, hi):
    """Golden-angle points spread by equal AREA over a disc/dome band (u in [lo,hi])."""
    return [(lo + (hi - lo) * math.sqrt((i + 0.5) / max(1, n)), i * GA) for i in range(n)]


def phyllo_lin(n, lo, hi):
    """Golden-angle points spread LINEARLY in u (for warts / honeycomb / spines up a flank)."""
    return [(lo + (hi - lo) * (i + 0.5) / max(1, n), i * GA) for i in range(n)]


def add_uniform_pads(verts, tris, origin, zdir, rad, height, band=(0.08, 0.72)):
    """PAD_MODE = 'uniform': N_PADS flat round seats laid by phyllotaxis over a primary dome."""
    for (u, th) in phyllo_area(int(N_PADS), band[0], band[1]):
        pt, nrm = dome_pn(origin, zdir, rad, height, u, th)
        add_puck(verts, tris, pt, nrm, PAD_DIA, PAD_HEIGHT)


def screen_top_tuple(rad):
    """(r_bez, r_scr, depth) for a crown screen, clamped to the dome radius."""
    r_scr = SCREEN_DIA / 2.0
    r_bez = min(r_scr + SCREEN_MARGIN, rad * 0.85)
    return (r_bez, r_scr, SCREEN_DEPTH)


# ============================================================================
#  Displacement fields (numpy) — amanita warts+gills, morel honeycomb
# ============================================================================
def make_amanita_disp(cap_rad, cap_h):
    seeds = phyllo_lin(int(WART_COUNT), 0.08, 0.62)
    su = np.array([u for (u, _) in seeds])
    sth = np.array([t % (2.0 * math.pi) for (_, t) in seeds])
    wart_amp, sig = 3.5, 6.0
    gill_amp = 2.5

    def disp(u, theta):
        dth = np.abs((theta % (2.0 * math.pi)) - sth)
        dth = np.minimum(dth, 2.0 * math.pi - dth)
        horiz = dth * cap_rad
        vert = (u - su) * cap_h
        d2 = horiz * horiz + vert * vert
        bump = wart_amp * float(np.sum(np.exp(-d2 / (2.0 * sig * sig))))
        if u > 0.72:                                      # radial gill ridges under the cap rim
            bump += gill_amp * 0.4 * math.cos(GILL_COUNT * theta) * smoothstep((u - 0.72) / 0.08)
        return bump
    return disp


def make_morel_disp(cap_rad, cap_h):
    """Honeycomb: for each surface point find the two nearest cell seeds; raise a ridge where
    they are near-equidistant (a cell boundary) and sink a pit inside the cell."""
    seeds = phyllo_lin(int(CELL_COUNT), 0.05, 0.95)
    su = np.array([u for (u, _) in seeds])
    sth = np.array([t % (2.0 * math.pi) for (_, t) in seeds])
    ridge_w = 3.0                                         # ridge half-width (mm)
    pit = RIDGE_AMP * 0.7

    def disp(u, theta):
        dth = np.abs((theta % (2.0 * math.pi)) - sth)
        dth = np.minimum(dth, 2.0 * math.pi - dth)
        horiz = dth * cap_rad
        vert = (u - su) * cap_h
        dist = np.sqrt(horiz * horiz + vert * vert)
        two = np.partition(dist, 1)[:2]
        edge = abs(float(two[1]) - float(two[0]))         # ~0 on a cell boundary
        mask = smoothstep((ridge_w - edge) / ridge_w)     # 1 on the ridge, 0 deep in a cell
        return RIDGE_AMP * mask - pit * (1.0 - mask)
    return disp


# ============================================================================
#  SPECIES: amanita — bulbous stem + ring + wide warty cap
# ============================================================================
def build_amanita():
    verts, tris = [], []
    sd = STEM_DIA / 2.0
    stem_len = float(STEM_LEN)
    cap_rad = float(BODY_RADIUS)
    cap_h = BODY_HEIGHT * 0.5
    cap_z = stem_len - 4.0                                # sink the cap into the stem top

    # bulbous stem with a flattened volva base (the stand)
    prof = [(sd * 1.5, 0.0), (sd * 1.75, sd * 0.7), (sd * 1.15, sd * 1.6),
            (sd * 0.72, stem_len * 0.5), (sd * 0.78, stem_len * 0.85), (sd * 0.98, stem_len)]
    bscr = (SCREEN_DIA / 2.0, SCREEN_DEPTH) if SCREEN_MODE == "base" else None
    add_revolution(verts, tris, (0, 0, 0), (0, 0, 1), prof, RES_STEM, bottom_screen=bscr)

    # flared ring (annulus skirt) partway up the stem
    rh = stem_len * 0.62
    add_revolution(verts, tris, (0, 0, 0), (0, 0, 1),
                   [(sd * 0.85, rh + 3), (cap_rad * 0.32, rh - 1), (sd * 0.85, rh - 6)], RES_STEM)

    # warty, gilled cap
    disp = make_amanita_disp(cap_rad, cap_h)
    if SCREEN_MODE == "top":
        add_dome(verts, tris, (0, 0, cap_z), (0, 0, 1), cap_rad, cap_h, RES_ANGULAR, RES_VERTICAL,
                 top="screen", screen=screen_top_tuple(cap_rad), disp=disp, bottom="disc")
    else:
        add_dome(verts, tris, (0, 0, cap_z), (0, 0, 1), cap_rad, cap_h, RES_ANGULAR, RES_VERTICAL,
                 top="pole", disp=disp, bottom="disc")

    # pads
    if PAD_MODE == "uniform":
        add_uniform_pads(verts, tris, (0, 0, cap_z), (0, 0, 1), cap_rad, cap_h)
    else:                                                 # feature seats scattered among the warts
        for (u, th) in phyllo_area(int(N_PADS), 0.15, 0.6):
            pt, nrm = dome_pn((0, 0, cap_z), (0, 0, 1), cap_rad, cap_h, u, th)
            add_puck(verts, tris, pt, nrm, PAD_DIA, PAD_HEIGHT)
    return verts, tris


# ============================================================================
#  SPECIES: morel — short stem + tall ovoid honeycombed cap
# ============================================================================
def build_morel():
    verts, tris = [], []
    stem_len = BODY_HEIGHT * 0.3
    sd = CAP_DIA / 2.0 * 0.8
    cap_rad = BODY_RADIUS * 0.72
    cap_h = float(BODY_HEIGHT)
    cap_z = stem_len - 3.0

    bscr = (SCREEN_DIA / 2.0, SCREEN_DEPTH) if SCREEN_MODE == "base" else None
    add_revolution(verts, tris, (0, 0, 0), (0, 0, 1),
                   [(max(sd * 1.1, 8.0), 0.0), (sd, stem_len * 0.5), (cap_rad * 0.5, stem_len)],
                   RES_STEM, bottom_screen=bscr)

    disp = make_morel_disp(cap_rad, cap_h)
    if SCREEN_MODE == "top":
        add_dome(verts, tris, (0, 0, cap_z), (0, 0, 1), cap_rad, cap_h, RES_ANGULAR, RES_VERTICAL,
                 top="screen", screen=screen_top_tuple(cap_rad), disp=disp, bottom="disc")
    else:
        add_dome(verts, tris, (0, 0, cap_z), (0, 0, 1), cap_rad, cap_h, RES_ANGULAR, RES_VERTICAL,
                 top="pole", disp=disp, bottom="disc")

    if PAD_MODE == "uniform":
        add_uniform_pads(verts, tris, (0, 0, cap_z), (0, 0, 1), cap_rad, cap_h)
    # PAD_MODE == "features": the honeycomb cells ARE the pads — no extra geometry.
    return verts, tris


# ============================================================================
#  SPECIES: lionsmane — rounded body + hanging spines, pads on the smooth crown
# ============================================================================
def build_lionsmane():
    verts, tris = [], []
    rad, h = float(BODY_RADIUS), float(BODY_HEIGHT)

    if SCREEN_MODE == "top":
        add_dome(verts, tris, (0, 0, 0), (0, 0, 1), rad, h, RES_ANGULAR, RES_VERTICAL,
                 top="screen", screen=screen_top_tuple(rad), bottom="disc")
    elif SCREEN_MODE == "base":
        add_dome(verts, tris, (0, 0, 0), (0, 0, 1), rad, h, RES_ANGULAR, RES_VERTICAL,
                 top="pole", bottom="screen", screen=(0.0, SCREEN_DIA / 2.0, SCREEN_DEPTH))
    else:
        add_dome(verts, tris, (0, 0, 0), (0, 0, 1), rad, h, RES_ANGULAR, RES_VERTICAL,
                 top="pole", bottom="disc")

    # downward-hanging tapered spines over the lower flank
    for (u, th) in phyllo_lin(int(SPINE_COUNT), 0.5, 0.82):
        pt, nrm = dome_pn((0, 0, 0), (0, 0, 1), rad, h, u, th)
        outward = normalize((pt[0], pt[1], 0.0)) or nrm
        zd = normalize((0.8 * outward[0] + 0.4 * nrm[0],
                        0.8 * outward[1] + 0.4 * nrm[1],
                        0.8 * outward[2] + 0.4 * nrm[2] - 0.5)) or (0, 0, -1)
        sr = SPINE_DIA / 2.0
        add_revolution(verts, tris, pt, zd,
                       [(sr * 1.05, -3.0), (sr, 0.0), (0.0, SPINE_LEN)], RES_SPINE)

    # pads on the smooth crown (spines are decorative)
    if PAD_MODE == "uniform":
        add_uniform_pads(verts, tris, (0, 0, 0), (0, 0, 1), rad, h)
    else:
        for (u, th) in phyllo_area(int(N_PADS), 0.05, 0.4):
            pt, nrm = dome_pn((0, 0, 0), (0, 0, 1), rad, h, u, th)
            add_puck(verts, tris, pt, nrm, PAD_DIA, PAD_HEIGHT)
    return verts, tris


# ============================================================================
#  SPECIES: shimeji / chestnutcluster — base clump + N stem+cap units
# ============================================================================
def build_cluster(species):
    verts, tris = [], []
    fat = (species == "chestnutcluster")
    clump_r = BODY_RADIUS * (0.80 if fat else 0.72)
    clump_h = BODY_HEIGHT * (0.34 if fat else 0.28)

    # base clump (a low dome with a flat underside = the stand)
    if SCREEN_MODE == "top":
        add_dome(verts, tris, (0, 0, 0), (0, 0, 1), clump_r, clump_h, RES_ANGULAR, RES_VERTICAL // 2,
                 top="screen", screen=screen_top_tuple(clump_r), bottom="disc")
    elif SCREEN_MODE == "base":
        add_dome(verts, tris, (0, 0, 0), (0, 0, 1), clump_r, clump_h, RES_ANGULAR, RES_VERTICAL // 2,
                 top="pole", bottom="screen", screen=(0.0, SCREEN_DIA / 2.0, SCREEN_DEPTH))
    else:
        add_dome(verts, tris, (0, 0, 0), (0, 0, 1), clump_r, clump_h, RES_ANGULAR, RES_VERTICAL // 2,
                 top="pole", bottom="disc")

    count = int(CLUSTER_COUNT)
    n_pad = min(int(N_PADS), count) if PAD_MODE == "features" else 0

    if fat:                                               # short, fat, tightly-packed, convex caps
        stem_len, sr = STEM_LEN * 0.42, STEM_DIA / 2.0 * 1.45
        cap_r, cap_rise, splay, band = CAP_DIA / 2.0 * 1.05, CAP_RISE * 0.85, 0.35, (0.10, 0.66)
    else:                                                 # long, slender, splaying, small caps
        stem_len, sr = float(STEM_LEN), STEM_DIA / 2.0 * 0.7
        cap_r, cap_rise, splay, band = CAP_DIA / 2.0 * 0.68, CAP_RISE * 0.8, 0.9, (0.14, 0.82)

    na_cap = max(28, RES_ANGULAR // 2)
    nr_cap = max(8, RES_VERTICAL // 3)
    for i, (u, th) in enumerate(phyllo_area(count, band[0], band[1])):
        pt, nrm = dome_pn((0, 0, 0), (0, 0, 1), clump_r, clump_h, u, th)
        outward = normalize((pt[0], pt[1], 0.0)) or nrm
        zd = normalize((nrm[0] + splay * outward[0],
                        nrm[1] + splay * outward[1],
                        nrm[2] + splay * outward[2])) or (0, 0, 1)
        # slender/stubby tapered stem, base sunk into the clump
        add_revolution(verts, tris, pt, zd,
                       [(sr * 1.1, -5.0), (sr, 0.0), (sr * 0.72, stem_len * 0.55), (sr * 0.82, stem_len)],
                       RES_STEM)
        cap_o = (pt[0] + zd[0] * stem_len, pt[1] + zd[1] * stem_len, pt[2] + zd[2] * stem_len)
        if i < n_pad:                                     # this cap is a pad -> flat copper seat
            add_dome(verts, tris, cap_o, zd, cap_r, cap_rise, na_cap, nr_cap,
                     top="seat", seat_r=min(PAD_DIA / 2.0, cap_r * 0.6), bottom="disc")
        else:
            add_dome(verts, tris, cap_o, zd, cap_r, cap_rise, na_cap, nr_cap,
                     top="pole", bottom="disc")

    if PAD_MODE == "uniform":
        add_uniform_pads(verts, tris, (0, 0, 0), (0, 0, 1), clump_r, clump_h, band=(0.08, 0.62))
    return verts, tris


# ============================================================================
#  Driver
# ============================================================================
def build():
    if SPECIES == "amanita":
        return build_amanita()
    if SPECIES == "morel":
        return build_morel()
    if SPECIES == "lionsmane":
        return build_lionsmane()
    if SPECIES in ("shimeji", "chestnutcluster"):
        return build_cluster(SPECIES)
    raise SystemExit(f"unknown SPECIES {SPECIES!r}")


# ============================================================================
#  Watertightness / sanity check
# ============================================================================
def check_mesh(verts, tris):
    edges = {}
    for (a, b, c) in tris:
        for e in ((a, b), (b, c), (c, a)):
            k = (e[0], e[1]) if e[0] < e[1] else (e[1], e[0])
            edges[k] = edges.get(k, 0) + 1
    nonman = sum(1 for v in edges.values() if v != 2)
    bad = 0
    xs = [p[0] for p in verts]; ys = [p[1] for p in verts]; zs = [p[2] for p in verts]
    for p in verts:
        if not all(math.isfinite(c) for c in p):
            bad += 1
    bbox = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    return nonman, bad, bbox


if __name__ == "__main__":
    default_species, default_mode = SPECIES, PAD_MODE
    print("validating all species x pad modes ...")
    for sp in ["shimeji", "amanita", "chestnutcluster", "morel", "lionsmane"]:
        for pm in ["features", "uniform"]:
            globals()["SPECIES"], globals()["PAD_MODE"] = sp, pm
            v, t = build()
            nonman, bad, bbox = check_mesh(v, t)
            flag = "OK" if (nonman == 0 and bad == 0) else "**FAIL**"
            print(f"  {sp:15s} {pm:8s}  verts={len(v):6d}  tris={len(t):6d}  "
                  f"nonmanifold={nonman:3d}  nan/inf={bad}  "
                  f"bbox={bbox[0]:.0f}x{bbox[1]:.0f}x{bbox[2]:.0f}mm  {flag}")

    globals()["SPECIES"], globals()["PAD_MODE"] = default_species, default_mode
    verts, tris = build()
    stl_path = os.path.join(HERE, "mushroom.stl")
    write_stl_binary(stl_path, verts, tris, f"mushroom-{SPECIES} (Omniphone)")
    print(f"\nwrote {stl_path}  (species={SPECIES}, mode={PAD_MODE}, "
          f"{len(verts)} verts, {len(tris)} triangles)")
