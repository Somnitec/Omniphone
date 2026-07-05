# thistle.py  —  procedural thistle-bulb enclosure generator
#
# Run:   python thistle.py        (in a venv with `pip install build123d`)
# Out:   thistle.step  -> open/edit in Fusion 360 or FreeCAD (editable solid)
#        thistle.stl   -> send to the slicer / 3D printer
#
# Shape: an ellipsoid with the top and bottom sliced flat, both rims smoothed,
# a screen recess in the top, and N spikes ("needles") placed in a sunflower
# (golden-angle / phyllotaxis) pattern over the curved surface. Each needle is
# where copper tape goes for capacitive sensing.
#
# Each needle is a solid of revolution: a tapered cone with a FLAT, filleted tip
# (set NEEDLE_TIP_FILLET) and a concave, flared, filleted foot that blends into
# the bulb wall (set NEEDLE_BASE_FILLET). The foot flares out by an extra
# NEEDLE_MERGE_FILLET so that when neighbouring needles are close enough their
# feet overlap and fuse into a smooth, merged base. The foot sinks NEEDLE_EMBED
# below the surface so every needle truly protrudes into the bulb.
#
# Note: a single true CAD fillet across all 144 mutual needle-needle seams is not
# something OpenCASCADE will resolve in one pass (it fails at any radius), so the
# merge here is geometric (flared, coved feet). For a crisp CAD blend, import the
# STEP into Fusion 360 and run one Fillet/Blend on the fused base edges.

from build123d import *
from math import sqrt, sin, cos, pi
import os

HERE = os.path.dirname(os.path.abspath(__file__))  # write next to this script

# ---------- PARAMETERS (edit these) ----------
# Target envelope ~ 220 x 220 x 100 mm (tune with viewer.html).
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
SCREEN_NEEDLE_IDX = 0        # which needle holds the screen, counted from the TOP (0 = topmost)
N_NEEDLES         = 14
NEEDLE_BASE_DIA   = 80
NEEDLE_LEN        = 27
NEEDLE_TIP_DIA    = 34
NEEDLE_TIP_FILLET = 2
NEEDLE_BASE_FILLET= 15
NEEDLE_MERGE_FILLET = 6
NEEDLE_EMBED      = 15
BAND_LOW          = 0.63
BAND_HIGH         = 1
NEEDLE_SIZE_START = 1.0      # radial size x of the FIRST needle (ramps linearly to _END)
NEEDLE_SIZE_END   = 1.0      # radial size x of the LAST needle
# ---------------------------------------------

a, c = BULB_RADIUS, BULB_HEIGHT

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
#    Profile (radius, z) revolved about Z -> flat filleted tip + concave foot.
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
            # round the flat tip rim
            if tf > 0:
                v_tip = sk.vertices().filter_by_position(
                    Axis.Y, L - 0.01, L + 0.01
                ).filter_by_position(Axis.X, rt - 0.01, rt + 0.01)
                fillet(v_tip, tf)
            # concave blend at the foot
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

# 6) phyllotaxis placement (golden angle = sunflower pattern)
# In "needle" mode the band reaches the pole and ONE needle (counted from the top)
# becomes the screen mount, placed by the same algorithm as the rest.
screen_on_needle = SCREEN_MODE == "needle" and SCREEN_DIA > 0
band_high = 1.0 if screen_on_needle else BAND_HIGH
screen_i = (N_NEEDLES - 1 - min(max(0, int(SCREEN_NEEDLE_IDX)), N_NEEDLES - 1)
            if screen_on_needle else -1)
screen_needle = screen_spike() if screen_on_needle else None

GA = pi * (3 - sqrt(5))
spikes = []
for i in range(N_NEEDLES):
    u = BAND_LOW + (band_high - BAND_LOW) * (i + 0.5) / N_NEEDLES  # height fraction
    theta = i * GA
    r = sqrt(max(0.0, 1 - u * u))
    x, y, z = a * r * cos(theta), a * r * sin(theta), c * u
    # outward ellipsoid normal = gradient (x/a^2, y/a^2, z/c^2)
    nx, ny, nz = x / a**2, y / a**2, z / c**2
    L = sqrt(nx * nx + ny * ny + nz * nz)
    nx, ny, nz = nx / L, ny / L, nz / L
    place = Plane(origin=(x, y, z), z_dir=(nx, ny, nz))  # surface plane; foot buried below
    if i == screen_i:                                    # this needle carries the screen
        spikes.append(place * screen_needle)
        continue
    # per-needle radial size ramp (first -> last); scale dia, keep length
    frac = i / (N_NEEDLES - 1) if N_NEEDLES > 1 else 0.0
    sc = NEEDLE_SIZE_START + (NEEDLE_SIZE_END - NEEDLE_SIZE_START) * frac
    part = needle if abs(sc - 1.0) < 1e-9 else scale(needle, by=(sc, sc, 1))
    spikes.append(place * part)

model = bulb
for s in spikes:
    model += s

step_path = os.path.join(HERE, "thistle.step")
stl_path = os.path.join(HERE, "thistle.stl")
export_step(model, step_path)   # <- open THIS in Fusion 360 / FreeCAD
export_stl(model, stl_path)      # <- send THIS to the slicer
print(f"wrote {step_path} and {stl_path}")
