# thistle_lloyd.py  —  procedural thistle/teasel-bulb generator, LLOYD-RELAXED spikes
#
# Run:   python thistle_lloyd.py     (in a venv with `pip install build123d`)
# Out:   thistle_lloyd.step -> open/edit in Fusion 360 or FreeCAD (editable solid)
#        thistle_lloyd.stl  -> send to the slicer / 3D printer
#
# Same bulb + needle geometry as thistle.py, but the spikes are placed by LLOYD'S
# RELAXATION instead of the golden-angle (phyllotaxis) spiral. Lloyd's algorithm
# (a centroidal Voronoi relaxation) starts from random seed points on the bulb's
# surface band and repeatedly nudges each one to the centroid of its Voronoi cell,
# so the points spread into an even but *organic, non-lattice* tiling — the look of
# a real teasel / burr head rather than the machined sunflower spiral. Set RELAX_SEED
# to reshuffle the arrangement; set RELAX_ITERS = 0 to see the raw random start.
#
# Each needle is identical to thistle.py: a solid of revolution with a flat filleted
# tip and a concave, flared, filleted foot that fuses with its neighbours. See
# thistle.py's header for the needle-profile and merge-fillet notes — they apply here
# verbatim. Only the placement differs.

from build123d import *
from math import sqrt, sin, cos, pi
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))  # write next to this script

# ---------- PARAMETERS (edit these) ----------
# Target envelope ~ 220 x 220 x 100 mm (tune with viewer_lloyd.html).
BULB_RADIUS       = 108
BULB_HEIGHT       = 78.5
TOP_CUT_FRAC      = 1
BOT_CUT_FRAC      = 0.05
RIM_FILLET_TOP    = 0
RIM_FILLET_BOT    = 10
SCREEN_MODE       = "bulb"  # "bulb" = recess in the top; "needle" = central screen pillar
SCREEN_DIA        = 0
SCREEN_MARGIN     = 3        # ring of material around the screen on the pillar top (mm)
SCREEN_DEPTH      = 10
SCREEN_NEEDLE_LEN = 22       # height of the screen pillar (used when SCREEN_MODE = "needle")
SCREEN_NEEDLE_BASE_DIA = 44  # base width of the screen needle where it meets the bulb (mm)
N_NEEDLES         = 14
NEEDLE_BASE_DIA   = 80
NEEDLE_LEN        = 27
NEEDLE_TIP_DIA    = 34
NEEDLE_TIP_FILLET = 2
NEEDLE_BASE_FILLET= 15
NEEDLE_MERGE_FILLET = 6
NEEDLE_EMBED      = 15
BAND_LOW          = 0.63     # spikes live between these height fractions of the bulb (u = z/c)
BAND_HIGH         = 1        # 1 = up to the top pole
NEEDLE_SIZE_START = 1.0      # radial size x of the "lowest" needle (ramps by height to _END)
NEEDLE_SIZE_END   = 1.0      # radial size x of the "highest" needle
# ---- Lloyd's-relaxation controls ----
RELAX_ITERS       = 40       # relaxation passes (more = more even; 0 = raw random seeding)
RELAX_SEED        = 7        # RNG seed — change it to get a different organic arrangement
RELAX_SAMPLES     = 20000    # Monte-Carlo samples used to estimate each Voronoi centroid
# ---------------------------------------------

a, c = BULB_RADIUS, BULB_HEIGHT

# ============================================================================
#  Bulb + needle geometry  (identical to thistle.py)
# ============================================================================

# 1) ellipsoid  = unit sphere scaled
bulb = scale(Sphere(1), by=(a, a, c))

# 2) flat top & bottom (a cut at/beyond the pole is skipped -> that end stays a dome)
if TOP_CUT_FRAC < 1.0:
    bulb = split(bulb, Plane.XY.offset(TOP_CUT_FRAC * c), keep=Keep.BOTTOM)
if BOT_CUT_FRAC > -1.0:
    bulb = split(bulb, Plane.XY.offset(BOT_CUT_FRAC * c), keep=Keep.TOP)

# 3) smooth the two rims (skip if the fillet is 0, the end is a dome, or no rim edge)
top_z, bot_z = TOP_CUT_FRAC * c, BOT_CUT_FRAC * c
if RIM_FILLET_TOP > 0 and TOP_CUT_FRAC < 1.0:
    top_edges = bulb.edges().filter_by_position(Axis.Z, top_z - 0.1, top_z + 0.1)
    if top_edges:
        bulb = fillet(top_edges, RIM_FILLET_TOP)
if RIM_FILLET_BOT > 0 and BOT_CUT_FRAC > -1.0:
    bot_edges = bulb.edges().filter_by_position(Axis.Z, bot_z - 0.1, bot_z + 0.1)
    if bot_edges:
        bulb = fillet(bot_edges, RIM_FILLET_BOT)

# 4) screen recess in the top (only in "bulb" mode; skip if there is no screen)
if SCREEN_MODE == "bulb" and SCREEN_DIA > 0 and SCREEN_DEPTH > 0:
    pocket = Pos(0, 0, top_z - SCREEN_DEPTH) * Cylinder(
        SCREEN_DIA / 2, SCREEN_DEPTH + 5, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )
    bulb = bulb - pocket


# 5) one needle, built pointing +Z, surface plane at z = 0, foot buried below.
def spike():
    rb = NEEDLE_BASE_DIA / 2.0
    rt = NEEDLE_TIP_DIA / 2.0
    L  = NEEDLE_LEN
    tf = max(0.0, min(NEEDLE_TIP_FILLET, rt - 0.05, L * 0.45))
    bf = max(0.0, NEEDLE_BASE_FILLET)
    mf = max(0.0, NEEDLE_MERGE_FILLET)      # extra reach so neighbouring feet fuse
    rfoot = rb + bf + mf                     # outer radius of the flared foot
    sink = NEEDLE_EMBED + bf + 0.5          # foot disc sits this far below surface

    raw = [
        (0.0,     L),             # tip centre
        (rt,      L),             # flat tip rim   <- tip fillet
        (rb,      0.0),           # side meets surface
        (rfoot,  -bf),            # flare outward  <- base fillet (concave)
        (rfoot,  -sink),          # skirt buried in bulb
        (0.0,    -sink),          # foot floor -> axis
    ]
    prof = [raw[0]]               # drop points a zero fillet/tip/flare collapses together
    for q in raw[1:]:
        if abs(q[0] - prof[-1][0]) > 1e-6 or abs(q[1] - prof[-1][1]) > 1e-6:
            prof.append(q)

    with BuildPart() as p:
        with BuildSketch(Plane.XZ) as sk:
            with BuildLine():
                Polyline(*prof)
                Line(prof[-1], prof[0])       # close up the axis
            make_face()
            if tf > 0:
                v_tip = sk.vertices().filter_by_position(
                    Axis.Y, L - 0.01, L + 0.01
                ).filter_by_position(Axis.X, rt - 0.01, rt + 0.01)
                fillet(v_tip, tf)
            if bf > 0:
                v_base = sk.vertices().filter_by_position(
                    Axis.Y, -0.01, 0.01
                ).filter_by_position(Axis.X, rb - 0.01, rb + 0.01)
                fillet(v_base, bf)
        revolve(axis=Axis.Z)
    return p.part

needle = spike()


# 5b) the screen-mount pillar: a flat-topped needle with a recess for the screen
def screen_spike():
    rTop = SCREEN_DIA / 2.0 + SCREEN_MARGIN     # flat top outer radius (screen + margin)
    rScr = SCREEN_DIA / 2.0                       # recess radius (the screen itself)
    rBase = SCREEN_NEEDLE_BASE_DIA / 2.0          # base width where it meets the bulb
    L = SCREEN_NEEDLE_LEN
    depth = min(SCREEN_DEPTH, L - 1.0)
    if rTop > rBase and rScr > rBase:            # mushroom: keep the recess inside the tapered body
        depth = min(depth, L - L * (rScr - rBase) / (rTop - rBase) - 0.5)
    depth = max(0.0, depth)
    tf = max(0.0, min(NEEDLE_TIP_FILLET, SCREEN_MARGIN * 0.9, L * 0.3))
    bf = max(0.0, min(NEEDLE_BASE_FILLET, rBase * 0.9))   # keep the base cove off the axis
    mf = max(0.0, NEEDLE_MERGE_FILLET)
    rfoot = rBase + bf + mf
    sink = NEEDLE_EMBED + bf + 0.5

    raw = [
        (0.0,   -sink),           # foot floor
        (rfoot, -sink),           # skirt out
        (rfoot, -bf),             # flare up
        (rBase,  0.0),            # base meets surface  <- base fillet
        (rTop,   L),              # flat top rim        <- tip fillet
        (rScr,   L),              # top annulus -> screen edge
        (rScr,   L - depth),      # recess wall down
        (0.0,    L - depth),      # recess floor -> axis
    ]
    prof = [raw[0]]
    for q in raw[1:]:
        if abs(q[0] - prof[-1][0]) > 1e-6 or abs(q[1] - prof[-1][1]) > 1e-6:
            prof.append(q)

    with BuildPart() as p:
        with BuildSketch(Plane.XZ) as sk:
            with BuildLine():
                Polyline(*prof)
                Line(prof[-1], prof[0])
            make_face()
            if tf > 0:
                v_tip = sk.vertices().filter_by_position(
                    Axis.Y, L - 0.01, L + 0.01
                ).filter_by_position(Axis.X, rTop - 0.01, rTop + 0.01)
                fillet(v_tip, tf)
            if bf > 0:
                v_base = sk.vertices().filter_by_position(
                    Axis.Y, -0.01, 0.01
                ).filter_by_position(Axis.X, rBase - 0.01, rBase + 0.01)
                fillet(v_base, bf)
        revolve(axis=Axis.Z)
    return p.part


# ============================================================================
#  Lloyd's relaxation placement  (this is what differs from thistle.py)
# ============================================================================
def lloyd_points(n, u_lo, u_hi, iters, seed, samples):
    """Return n unit-sphere points, spread evenly over the latitude band u in
    [u_lo, u_hi] by Lloyd's algorithm (a centroidal Voronoi relaxation).

    The band is sampled with a uniform point cloud (uniform in u = z gives
    area-uniform coverage of a sphere). Each pass assigns every cloud sample to
    its nearest seed, then moves each seed to the mean of its samples and projects
    it back onto the sphere — the fixed point is a centroidal Voronoi tessellation,
    i.e. an even, organic spacing. RNG is seeded, so the layout is reproducible."""
    rng = random.Random(seed)

    def rand_on_band():
        u = u_lo + (u_hi - u_lo) * rng.random()      # z, uniform over the band's area
        t = 2.0 * pi * rng.random()
        r = sqrt(max(0.0, 1.0 - u * u))
        return (r * cos(t), r * sin(t), u)

    def project(p):
        x, y, z = p
        z = min(u_hi, max(u_lo, z))                  # keep the seed inside the band
        r = sqrt(x * x + y * y)
        s = sqrt(max(0.0, 1.0 - z * z))
        if r < 1e-9:                                  # degenerate -> random azimuth
            t = 2.0 * pi * rng.random()
            return (s * cos(t), s * sin(t), z)
        return (x / r * s, y / r * s, z)

    seeds = [rand_on_band() for _ in range(n)]

    for _ in range(int(iters)):
        acc = [[0.0, 0.0, 0.0, 0] for _ in range(n)]  # sum x,y,z + count per cell
        for _s in range(int(samples)):
            px, py, pz = rand_on_band()
            best, bestd = 0, 1e18
            for i, (sx, sy, sz) in enumerate(seeds):
                d = (px - sx) ** 2 + (py - sy) ** 2 + (pz - sz) ** 2
                if d < bestd:
                    bestd, best = d, i
            a_ = acc[best]
            a_[0] += px; a_[1] += py; a_[2] += pz; a_[3] += 1
        for i in range(n):
            cnt = acc[i][3]
            if cnt == 0:                              # starved cell -> reseed randomly
                seeds[i] = rand_on_band()
            else:
                seeds[i] = project((acc[i][0] / cnt, acc[i][1] / cnt, acc[i][2] / cnt))

    seeds.sort(key=lambda p: p[2])                    # order low -> high for the size ramp
    return seeds


# In "needle" mode the band reaches the pole and the TOP-most relaxed point carries
# the screen pillar; otherwise a plain flat-top recess is used (handled on the bulb).
screen_on_needle = SCREEN_MODE == "needle" and SCREEN_DIA > 0
band_high = 1.0 if screen_on_needle else BAND_HIGH
screen_needle = screen_spike() if screen_on_needle else None

pts = lloyd_points(N_NEEDLES, BAND_LOW, band_high, RELAX_ITERS, RELAX_SEED, RELAX_SAMPLES)
screen_i = N_NEEDLES - 1 if screen_on_needle else -1   # topmost point after the sort

spikes = []
for i, (sx, sy, sz) in enumerate(pts):
    x, y, z = a * sx, a * sy, c * sz                   # unit sphere -> ellipsoid
    # outward ellipsoid normal = gradient (x/a^2, y/a^2, z/c^2)
    nx, ny, nz = x / a**2, y / a**2, z / c**2
    Ln = sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / Ln, ny / Ln, nz / Ln
    place = Plane(origin=(x, y, z), z_dir=(nx, ny, nz))  # surface plane; foot buried below
    if i == screen_i:                                    # this point carries the screen
        spikes.append(place * screen_needle)
        continue
    # per-needle radial size ramp (lowest -> highest)
    frac = i / (N_NEEDLES - 1) if N_NEEDLES > 1 else 0.0
    sc = NEEDLE_SIZE_START + (NEEDLE_SIZE_END - NEEDLE_SIZE_START) * frac
    part = needle if abs(sc - 1.0) < 1e-9 else scale(needle, by=(sc, sc, 1))
    spikes.append(place * part)

model = bulb
for s in spikes:
    model += s

if __name__ == "__main__":
    step_path = os.path.join(HERE, "thistle_lloyd.step")
    stl_path = os.path.join(HERE, "thistle_lloyd.stl")
    export_step(model, step_path)   # <- open THIS in Fusion 360 / FreeCAD
    export_stl(model, stl_path)      # <- send THIS to the slicer
    print(f"wrote {step_path} and {stl_path}")
