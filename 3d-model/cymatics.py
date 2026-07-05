# cymatics.py  —  procedural cymatics-dome generator
#
# Run:   python3 cymatics.py          (pure standard library — no venv / build123d needed)
# Out:   cymatics.stl                  -> send straight to the slicer / 3D printer,
#                                          or import into Fusion 360 / FreeCAD as a mesh body.
#
# Shape: a dome (ellipsoid / parabola / bell / flat plate) whose surface is
# displaced by a *cymatics* standing-wave field — the vibration modes of a round
# drum membrane. Each mode is a Bessel function J_m(k_mn·r/R)·cos(m·θ): m angular
# lobes ("petals") and n concentric rings, exactly the ridge-and-valley patterns
# sand forms on a driven Chladni plate. Two modes can be blended, the field can
# be signed (peaks + troughs) or rectified (all-positive welts), and it rides
# either straight up (+Z) or along the dome's surface normal.
#
# SWIRL + WARP break the pattern's tidy n-fold symmetry (twist + organic wobble).
# A screen can be recessed into the dome centre, or mounted on a raised pad over
# one of the pattern's peaks — the same two choices as the thistle enclosure.
#
# The result is a watertight solid: displaced top + short cylindrical foot + flat
# base disc (+ an overlapping screen pad in "peak" mode, unioned by the slicer).
# Hollow it in the slicer or Fusion after import, exactly like thistle.py.
#
# All math here is mirrored 1:1 in cymatics.html, so the live viewer is faithful.

import math
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))  # write next to this script

# ---------- PARAMETERS (edit these — or dial them in cymatics.html and paste) ----------
DOME_RADIUS   = 90
DOME_HEIGHT   = 55
DOME_PROFILE  = "ellipse"
MODE_M        = 5
MODE_N        = 3
AMPLITUDE     = 10
MODE2_M       = 4
MODE2_N       = 4
MODE2_MIX     = 0.63
RECTIFY       = "absolute"
DISPLACE      = "normal"
ANGLE_OFFSET  = 0
SWIRL         = 0
WARP          = 0
SCREEN_MODE   = "center"
SCREEN_DIA    = 32
SCREEN_MARGIN = 4
SCREEN_DEPTH  = 6
SCREEN_PEAK_IDX = 0
SCREEN_PEAK_H = 10
BASE_HEIGHT   = 4
RES_RADIAL    = 130
RES_ANGULAR   = 240
# ---------------------------------------------------------------------------------------


# ---------- shared cymatics math (identical in cymatics.html) ----------
def bessel_J(m, x):
    """J_m(x) via Bessel's integral  (1/pi) * integral_0^pi cos(m t - x sin t) dt.
    Simpson's rule, robust for the small x we sample (x up to ~30)."""
    K = 48                       # even number of Simpson intervals
    h = math.pi / K
    s = 0.0
    for i in range(K + 1):
        t = i * h
        w = 1.0 if (i == 0 or i == K) else (4.0 if i % 2 else 2.0)
        s += w * math.cos(m * t - x * math.sin(t))
    return (s * h / 3.0) / math.pi


def bessel_zero(m, n):
    """The n-th positive root of J_m (n >= 1), by scan + bisection."""
    prev_x = 1e-4
    prev_f = bessel_J(m, prev_x)
    found = 0
    x = prev_x
    step = 0.05
    while x < 200.0:
        x += step
        f = bessel_J(m, x)
        if prev_f == 0.0 or (f < 0.0) != (prev_f < 0.0):   # sign change -> a root in (prev_x, x)
            lo, hi, flo = prev_x, x, prev_f
            for _ in range(60):                            # bisect to convergence
                mid = 0.5 * (lo + hi)
                fm = bessel_J(m, mid)
                if (fm < 0.0) == (flo < 0.0):
                    lo, flo = mid, fm
                else:
                    hi = mid
            found += 1
            if found == n:
                return 0.5 * (lo + hi)
        prev_x, prev_f = x, f
    return x  # fallback (should not happen for sane n)


def mode_scale(m, z):
    """1 / peak|J_m(z·u)| for u in [0,1], so every mode normalises to unit amplitude."""
    peak = 1e-9
    for i in range(201):
        peak = max(peak, abs(bessel_J(m, z * (i / 200.0))))
    return 1.0 / peak


def dome_profile(u):
    """Base dome height at radius fraction u = r/R, in [0,1]."""
    if DOME_PROFILE == "ellipse":
        return DOME_HEIGHT * math.sqrt(max(0.0, 1.0 - u * u))
    if DOME_PROFILE == "parabola":
        return DOME_HEIGHT * (1.0 - u * u)
    if DOME_PROFILE == "cosine":
        return DOME_HEIGHT * math.cos(0.5 * math.pi * u)
    return 0.0  # "flat"


# ---------- build the displaced-dome solid ----------
def build():
    R, na, nr = DOME_RADIUS, int(RES_ANGULAR), int(RES_RADIAL)
    off = math.radians(ANGLE_OFFSET)
    swirl = math.radians(SWIRL)
    m1, m2 = int(MODE_M), int(MODE2_M)
    z1, z2 = bessel_zero(m1, int(MODE_N)), bessel_zero(m2, int(MODE2_N))
    s1, s2 = mode_scale(m1, z1), mode_scale(m2, z2)
    mix = max(0.0, min(1.0, MODE2_MIX))
    rect = RECTIFY == "absolute"
    delta = 1e-3 * R  # step for the numeric surface-normal derivative
    thetas = [2.0 * math.pi * i / na for i in range(na)]

    def warp_coords(u, theta):
        """Distort the sampling coordinates so the pattern loses its clean symmetry:
        an angular twist (swirl, growing with radius) + a fixed multi-harmonic wobble."""
        ph = theta + swirl * u + WARP * (0.6 * math.sin(2 * theta + 1.3)
                                         + 0.4 * math.sin(3 * theta - 0.7)
                                         + 0.25 * math.sin(5 * theta + 2.1))
        ue = u * (1.0 + WARP * (0.22 * math.sin(3 * theta + 0.9)
                                + 0.14 * math.sin(2 * theta - 1.7)))
        return ue, ph

    def field(u, theta):
        ue, ph = warp_coords(u, theta)
        v = (1.0 - mix) * bessel_J(m1, z1 * ue) * s1 * math.cos(m1 * ph + off) \
            + mix * bessel_J(m2, z2 * ue) * s2 * math.cos(m2 * ph + off)
        return abs(v) if rect else v

    def surf_xyz(r, theta):
        """(x, y, z) of the displaced dome surface at polar (r, theta)."""
        u = r / R
        disp = AMPLITUDE * field(u, theta)
        h0 = dome_profile(u)
        if DISPLACE == "normal":
            # outward normal of the surface-of-revolution z = h0(r): n = (-h0', 1)/|.|
            dh = (dome_profile((r + delta) / R) - dome_profile((r - delta) / R)) / (2.0 * delta)
            inv = 1.0 / math.hypot(1.0, dh)
            rr = r + disp * (-dh) * inv
            z = BASE_HEIGHT + h0 + disp * inv
        else:  # "vertical"
            rr = r
            z = BASE_HEIGHT + h0 + disp
        z = max(z, 0.4)  # never poke below the bottom disc
        return (rr * math.cos(theta), rr * math.sin(theta), z)

    def disp_ring(r):
        return [surf_xyz(r, t) for t in thetas]

    def flat_ring(r, z):
        return [(r * math.cos(t), r * math.sin(t), z) for t in thetas]

    # ----- screen geometry (centre recess is carved straight into the polar top) -----
    screen = SCREEN_MODE if SCREEN_DIA > 0 else "none"
    r_bez = min(SCREEN_DIA / 2.0 + SCREEN_MARGIN, R * 0.92)
    r_scr = min(SCREEN_DIA / 2.0, r_bez - 0.5)
    shelf_z = BASE_HEIGHT + dome_profile(r_bez / R)          # flat bezel seat height
    floor_z = max(BASE_HEIGHT + 0.5, shelf_z - SCREEN_DEPTH)  # recess floor

    # ordered inner->outer "stations" (each an na-vertex ring) for the top surface
    if screen == "center":
        center_pt = (0.0, 0.0, floor_z)
        stations = [flat_ring(r_scr, floor_z),   # recess floor edge
                    flat_ring(r_scr, shelf_z),    # recess wall up to the flat seat
                    flat_ring(r_bez, shelf_z),     # flat bezel seat
                    disp_ring(r_bez)]              # step out onto the rippled dome
        for j in range(1, nr + 1):
            r = R * j / nr
            if r > r_bez + 1e-6:
                stations.append(disp_ring(r))
        if math.hypot(*stations[-1][0][:2]) < R - 1e-6:
            stations.append(disp_ring(R))          # guarantee a rim ring exactly at R
    else:
        center_pt = surf_xyz(0.0, 0.0)             # apex
        stations = [disp_ring(R * j / nr) for j in range(1, nr + 1)]

    verts, tris = [], []

    def add_ring(ring):
        base = len(verts)
        verts.extend(ring)
        return base

    verts.append(center_pt)                        # 0 = inner cap point
    ring_bases = [add_ring(st) for st in stations]

    # faces -------------------------------------------------------------
    b0 = ring_bases[0]
    for i in range(na):                            # cap fan: centre point -> first ring
        tris.append((0, b0 + i, b0 + (i + 1) % na))
    for k in range(len(ring_bases) - 1):           # quad strips between consecutive rings
        a, b = ring_bases[k], ring_bases[k + 1]
        for i in range(na):
            i1 = (i + 1) % na
            tris.append((a + i, a + i1, b + i1))
            tris.append((a + i, b + i1, b + i))

    rim = ring_bases[-1]                            # cylindrical foot wall -> flat bottom disc
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

    # ----- peaks: strict local maxima of the ripple over the (ring, angle) grid -----
    grid = [[field(j / nr, thetas[i]) for i in range(na)] for j in range(nr + 1)]
    apex_v = field(0.0, 0.0)
    peaks_rt = []
    if all(apex_v > grid[1][i] for i in range(na)):          # centre bump (e.g. m = 0)
        peaks_rt.append((0.0, 0.0))
    for j in range(1, nr + 1):
        for i in range(na):
            d = grid[j][i]
            ok = (d > grid[j][(i + 1) % na] and d > grid[j][(i - 1) % na] and d > grid[j - 1][i])
            if ok and j < nr:
                ok = d > grid[j + 1][i]
            if ok:
                peaks_rt.append((R * j / nr, thetas[i]))
    peaks = len(peaks_rt)

    # ----- screen pad standing on a chosen peak (a separate, overlapping solid) -----
    if screen == "peak" and peaks_rt:
        order = sorted(peaks_rt, key=lambda p: (p[0], p[1]))  # innermost first, then by angle
        rp, tp = order[int(SCREEN_PEAK_IDX) % len(order)]
        origin = surf_xyz(rp, tp)
        dh = (dome_profile((rp + delta) / R) - dome_profile((rp - delta) / R)) / (2.0 * delta)
        inv = 1.0 / math.hypot(1.0, dh)
        nrm = (-dh * inv * math.cos(tp), -dh * inv * math.sin(tp), inv)   # base dome normal
        H = max(2.0, SCREEN_PEAK_H)
        sink = 4.0
        depth = max(0.0, min(SCREEN_DEPTH, H + sink - 1.0))
        profile = [(0.0, -sink), (r_bez, -sink), (r_bez, H),
                   (r_scr, H), (r_scr, H - depth), (0.0, H - depth)]
        revolve(profile, origin, nrm, max(24, na // 4), verts, tris)

    return verts, tris, peaks


def revolve(profile, origin, zdir, seg, verts, tris):
    """Revolve a (radius, height) profile about `zdir` at `origin`, appending a
    closed solid to verts/tris. Profile points at radius 0 become axis apices."""
    def norm(v):
        L = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) or 1.0
        return (v[0] / L, v[1] / L, v[2] / L)

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    Z = norm(zdir)
    ax = (0.0, 0.0, 1.0) if abs(Z[2]) < 0.9 else (1.0, 0.0, 0.0)
    U = norm(cross(ax, Z))
    V = cross(Z, U)
    phis = [2.0 * math.pi * k / seg for k in range(seg)]

    def pt(rad, h, phi):
        cr, sr = rad * math.cos(phi), rad * math.sin(phi)
        return (origin[0] + cr * U[0] + sr * V[0] + h * Z[0],
                origin[1] + cr * U[1] + sr * V[1] + h * Z[1],
                origin[2] + cr * U[2] + sr * V[2] + h * Z[2])

    rows = []                    # per profile point: ("apex", idx) or ("ring", base_idx)
    for (rad, h) in profile:
        if rad < 1e-6:
            rows.append(("apex", len(verts))); verts.append(pt(0.0, h, 0.0))
        else:
            base = len(verts)
            for phi in phis:
                verts.append(pt(rad, h, phi))
            rows.append(("ring", base))
    for k in range(len(rows) - 1):
        (ka, ia), (kb, ib) = rows[k], rows[k + 1]
        if ka == "apex" and kb == "ring":
            for i in range(seg):
                tris.append((ia, ib + i, ib + (i + 1) % seg))
        elif ka == "ring" and kb == "apex":
            for i in range(seg):
                tris.append((ia + i, ib, ia + (i + 1) % seg))
        elif ka == "ring" and kb == "ring":
            for i in range(seg):
                i1 = (i + 1) % seg
                tris.append((ia + i, ia + i1, ib + i1))
                tris.append((ia + i, ib + i1, ib + i))


def write_stl_binary(path, verts, tris):
    def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    with open(path, "wb") as f:
        f.write(b"cymatics-dome (Omniphone) - solid, watertight".ljust(80, b" ")[:80])
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


if __name__ == "__main__":
    verts, tris, peaks = build()
    stl_path = os.path.join(HERE, "cymatics.stl")
    write_stl_binary(stl_path, verts, tris)
    print(f"wrote {stl_path}  ({len(verts)} verts, {len(tris)} triangles, {peaks} peaks)")
