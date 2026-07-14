# mandelbulb.py  —  procedural simplified-mandelbulb enclosure blank
#
# Run:   python3 mandelbulb.py        (pure standard library — no venv / build123d needed)
# Out:   mandelbulb.stl                -> import into Fusion 360 / FreeCAD as a MESH body,
#                                          then cut the screen pocket + power/USB holes and
#                                          shell it. Or send straight to the slicer.
#
# Shape: the classic power-8 Mandelbulb (a 3D escape-time fractal) taken at a LOW
# iteration count so it comes out as a smooth, rounded, lobed solid — the
# characteristic bulb with radial "petals" — rather than an un-printable, infinitely
# crenellated fractal. That is the "simplified" bit: fewer iterations + a generous
# iso-level inflate the surface into a clean blank you can actually machine features
# into. Turn ITERATIONS up (and LEVEL toward it) for more fractal crust.
#
# A screen recess (SCREEN_MODE = "bulb", like thistle.py) can be carved flat into the
# top lobe for the round LCD — or leave it "none" and cut the screen + power/USB holes
# yourself in Fusion after import, which is the more flexible route for the connectors.
#
# Method: the smooth (fractional) escape-time is sampled on a 3D grid, then an
# iso-surface is extracted with MARCHING TETRAHEDRA. Tetrahedra (not cubes) keep the
# mesh guaranteed watertight — every interior edge is shared by exactly two triangles,
# with bit-identical interpolated vertices — so Fusion's "Mesh -> Convert to BRep"
# and any slicer accept it as one closed solid.
#
# The math here is mirrored 1:1 in mandelbulb.html, so the live viewer is faithful.

import math
import os
import struct
from array import array

HERE = os.path.dirname(os.path.abspath(__file__))  # write next to this script

# ---------- PARAMETERS (edit these — or dial them in mandelbulb.html and paste) ----------
POWER        = 7
ITERATIONS   = 3
BAILOUT      = 1.95
LEVEL        = 1.8
GRID         = 96
BOX          = 3.0          # must clear the natural iso-surface radius (~2.7 at these POWER/LEVEL) on every side
SIZE_MM      = 120
FLATTEN_ALL  = 0.78        # uniform vertical squash of the whole ball (1 = untouched sphere)
FLATTEN_BASE = 0.4         # extra squash at the very bottom pole, on top of FLATTEN_ALL (1 = no extra squash)
FLATTEN_ZONE = 0.35        # fraction of the lower hemisphere's height over which that extra squash ramps in
                            #   (small = only the base itself flattens; 1.0 = ramps across the whole lower half)
SMOOTH_ITERS = 2
SCREEN_MODE  = "none"
SCREEN_DIA   = 34.5
SCREEN_DEPTH = 0
# ----------------------------------------------------------------------------------------


def squash_verts(verts):
    """Post-process vertical squash: FLATTEN_ALL contracts every vertex toward the
    mesh's own mid-height; within FLATTEN_ZONE of the lowest point, that contraction
    blends smoothly (smoothstep, zero slope at both ends of the ramp) into the
    stronger FLATTEN_BASE squash, so just the base flattens more than the rest.
    Keeping the ramp confined near the pole (rather than spanning the whole lower
    hemisphere) matters — a sphere's cross-section stays wide for most of its
    height and only tapers sharply near the pole, so ramping the extra squash in
    too early over-compresses those still-wide cross-sections into a thin band
    and reads as a flat shelf, the same "plate" look this is meant to avoid.
    Applied to the already-extracted, already-watertight mesh (not the sampled
    field) — moving vertices can only change their positions, never the mesh's
    edge topology, so this can't open up the surface the way clipping the field
    with a cutting plane did. The result is one continuous skin (no base-plate
    seam), so overlays like the RD coral ridge field can wrap all the way around
    it."""
    if not verts:
        return
    zmin = min(v[2] for v in verts)
    zmax = max(v[2] for v in verts)
    zmid = 0.5 * (zmin + zmax)
    ramp = max(1e-9, (zmid - zmin) * FLATTEN_ZONE)
    for v in verts:
        z = v[2]
        if z >= zmid:
            s = FLATTEN_ALL
        else:
            t = min(1.0, (zmid - z) / ramp)          # 0 above the ramp zone -> 1 at the lowest point
            t = t * t * (3.0 - 2.0 * t)               # smoothstep
            s = FLATTEN_ALL * (1.0 - (1.0 - FLATTEN_BASE) * t)
        v[2] = zmid + (z - zmid) * s


# ---------- shared mandelbulb math (identical in mandelbulb.html) ----------
def escape_field(x, y, z):
    """Smooth (fractional) escape-time of point (x,y,z) under z -> z^POWER + c, z0 = 0.
    Small value  = escaped early (well outside the set);
    large value  = survived many iterations (inside the set).
    The LEVEL iso-surface of this scalar field is the solid's skin."""
    cx, cy, cz = x, y, z
    zx = zy = zz = 0.0
    logB = math.log(BAILOUT)
    logP = math.log(POWER)
    for i in range(ITERATIONS):
        r2 = zx * zx + zy * zy + zz * zz
        if r2 > BAILOUT * BAILOUT:
            r = math.sqrt(r2)
            # continuous escape count -> a smooth field, no iteration banding
            return i + 1.0 - math.log(math.log(r) / logB) / logP
        r = math.sqrt(r2)
        if r < 1e-12:                       # z0 = 0 -> 0^n + c = c (first step)
            zx, zy, zz = cx, cy, cz
            continue
        theta = math.acos(max(-1.0, min(1.0, zz / r)))   # polar angle
        phi = math.atan2(zy, zx)                          # azimuth
        rn = r ** POWER
        st = math.sin(POWER * theta)
        ct = math.cos(POWER * theta)
        zx = rn * st * math.cos(POWER * phi) + cx
        zy = rn * st * math.sin(POWER * phi) + cy
        zz = rn * ct + cz
    return ITERATIONS + 10.0                # never escaped -> deep inside


# ---------- marching tetrahedra (watertight iso-surface extraction) ----------
# Cube corner offsets, indexed 0..7.
_CORNER = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
           (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
# Six tetrahedra sharing the 0-6 main diagonal — a decomposition that tiles space
# consistently, so neighbouring cubes agree on every shared face.
_TETS = [(0, 1, 2, 6), (0, 2, 3, 6), (0, 3, 7, 6),
         (0, 7, 4, 6), (0, 4, 5, 6), (0, 5, 1, 6)]


def carve_screen(field, N, h, L):
    """Cut a round, flat-bottomed screen recess down into the top lobe (fractal units).
    The pocket is voxel-resolution (raise GRID for crisper walls, or true it up in
    Fusion) — it seats the round LCD and marks the bezel. Keeps the solid watertight."""
    def coord(i): return -BOX + i * h
    # approximate fractal-units -> mm scale from the iso-surface's grid bounding span
    lo = [1e9, 1e9, 1e9]; hi = [-1e9, -1e9, -1e9]
    for k in range(N):
        base = k * N * N
        for j in range(N):
            row = base + j * N
            for i in range(N):
                if field[row + i] >= L:
                    p = (coord(i), coord(j), coord(k))
                    for d in range(3):
                        if p[d] < lo[d]: lo[d] = p[d]
                        if p[d] > hi[d]: hi[d] = p[d]
    span = max(hi[d] - lo[d] for d in range(3)) or 1.0
    scale = float(SIZE_MM) / span                 # mm per fractal unit
    R = (SCREEN_DIA * 0.5) / scale                # pocket radius, fractal units
    depth = SCREEN_DEPTH / scale
    R2 = R * R
    # on-axis top of the bulb (highest inside point within radius R of the axis)
    ztop = -1e9
    for k in range(N):
        z = coord(k); base = k * N * N
        for j in range(N):
            y = coord(j); row = base + j * N
            for i in range(N):
                x = coord(i)
                if x * x + y * y <= R2 and field[row + i] >= L and z > ztop:
                    ztop = z
    zfloor = ztop - depth
    for k in range(N):
        if coord(k) < zfloor:
            continue
        base = k * N * N
        for j in range(N):
            y = coord(j); row = base + j * N
            for i in range(N):
                x = coord(i)
                if x * x + y * y <= R2:
                    field[row + i] = -1.0e9        # remove the cylinder -> open recess
    print(f"  screen recess: dia {SCREEN_DIA}mm x depth {SCREEN_DEPTH}mm on the top lobe")


def build():
    N = int(GRID)
    L = float(LEVEL)
    h = 2.0 * BOX / (N - 1)                       # grid spacing in fractal units

    # 1) sample the scalar field on the full N^3 grid (this is the slow part) --------
    field = array('d', bytes(8 * N * N * N))
    def IDX(i, j, k): return (k * N + j) * N + i
    for k in range(N):
        zc = -BOX + k * h
        base = k * N * N
        for j in range(N):
            yc = -BOX + j * h
            row = base + j * N
            for i in range(N):
                xc = -BOX + i * h
                field[row + i] = escape_field(xc, yc, zc)
        if k % 8 == 0 or k == N - 1:
            print(f"  field {k + 1}/{N} slabs")

    # optional screen recess in the top lobe (like thistle.py's "bulb" mode)
    if SCREEN_MODE == "bulb" and SCREEN_DIA > 0 and SCREEN_DEPTH > 0:
        carve_screen(field, N, h, L)

    verts = []
    tris = []
    vcache = {}                                   # global edge-key -> vertex index (weld)

    def pos(gi):                                  # grid-corner index -> (x,y,z)
        i = gi % N; j = (gi // N) % N; k = gi // (N * N)
        return (-BOX + i * h, -BOX + j * h, -BOX + k * h)

    def edge_vertex(ga, gb):
        """Interpolated iso-crossing on grid edge (ga,gb), welded & order-independent."""
        if gb < ga:
            ga, gb = gb, ga                       # canonical order -> bit-identical result
        key = ga * (N * N * N) + gb
        hit = vcache.get(key)
        if hit is not None:
            return hit
        va, vb = field[ga], field[gb]
        t = (L - va) / (vb - va) if vb != va else 0.5
        pa, pb = pos(ga), pos(gb)
        p = (pa[0] + t * (pb[0] - pa[0]),
             pa[1] + t * (pb[1] - pa[1]),
             pa[2] + t * (pb[2] - pa[2]))
        vi = len(verts)
        verts.append(list(p))
        vcache[key] = vi
        return vi

    # 2) march every cube's six tetrahedra ------------------------------------------
    for k in range(N - 1):
        for j in range(N - 1):
            for i in range(N - 1):
                # global grid index of each of the 8 cube corners
                g = [IDX(i + dx, j + dy, k + dz) for (dx, dy, dz) in _CORNER]
                for (a, b, c, d) in _TETS:
                    ga, gb, gc, gd = g[a], g[b], g[c], g[d]
                    ins = []
                    out = []
                    for gv in (ga, gb, gc, gd):
                        (ins if field[gv] >= L else out).append(gv)
                    n = len(ins)
                    if n == 0 or n == 4:
                        continue
                    if n == 1:
                        a0 = ins[0]
                        tris.append((edge_vertex(a0, out[0]),
                                     edge_vertex(a0, out[1]),
                                     edge_vertex(a0, out[2])))
                    elif n == 3:
                        d0 = out[0]
                        tris.append((edge_vertex(d0, ins[0]),
                                     edge_vertex(d0, ins[2]),
                                     edge_vertex(d0, ins[1])))
                    else:  # n == 2 -> quad
                        a0, b0 = ins
                        c0, d0 = out
                        p_ac = edge_vertex(a0, c0)
                        p_ad = edge_vertex(a0, d0)
                        p_bd = edge_vertex(b0, d0)
                        p_bc = edge_vertex(b0, c0)
                        tris.append((p_ac, p_ad, p_bd))
                        tris.append((p_ac, p_bd, p_bc))
        if k % 8 == 0 or k == N - 2:
            print(f"  mesh  {k + 1}/{N - 1} slabs  ({len(tris)} tris)")

    # 3) vertical squash (continuous — see squash_verts docstring) ------------------
    squash_verts(verts)

    # 4) optional Laplacian smoothing (softens the tetra faceting) ------------------
    for _ in range(int(SMOOTH_ITERS)):
        smooth(verts, tris)

    # 5) scale to millimetres --------------------------------------------------------
    scale_to_mm(verts)
    return verts, tris


def smooth(verts, tris):
    """One umbrella (uniform Laplacian) pass: move each vertex toward its neighbours' mean."""
    nb = [set() for _ in verts]
    for (a, b, c) in tris:
        nb[a].add(b); nb[a].add(c)
        nb[b].add(a); nb[b].add(c)
        nb[c].add(a); nb[c].add(b)
    moved = [v[:] for v in verts]
    for i, ns in enumerate(nb):
        if not ns:
            continue
        sx = sy = sz = 0.0
        for j in ns:
            sx += verts[j][0]; sy += verts[j][1]; sz += verts[j][2]
        n = len(ns)
        cx, cy, cz = sx / n, sy / n, sz / n
        w = 0.5                                   # blend factor toward the neighbour centroid
        moved[i][0] = verts[i][0] + w * (cx - verts[i][0])
        moved[i][1] = verts[i][1] + w * (cy - verts[i][1])
        moved[i][2] = verts[i][2] + w * (cz - verts[i][2])
    for i in range(len(verts)):
        verts[i] = moved[i]


def scale_to_mm(verts):
    if not verts:
        return
    lo = [min(v[d] for v in verts) for d in range(3)]
    hi = [max(v[d] for v in verts) for d in range(3)]
    span = max(hi[d] - lo[d] for d in range(3)) or 1.0
    s = float(SIZE_MM) / span
    cx = 0.5 * (lo[0] + hi[0])
    cy = 0.5 * (lo[1] + hi[1])
    for v in verts:                               # centre in XY, sit the base on z = 0
        v[0] = (v[0] - cx) * s
        v[1] = (v[1] - cy) * s
        v[2] = (v[2] - lo[2]) * s


def write_stl_binary(path, verts, tris):
    def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    with open(path, "wb") as f:
        f.write(b"mandelbulb (Omniphone) - solid, watertight".ljust(80, b" ")[:80])
        f.write(struct.pack("<I", len(tris)))
        for (ia, ib, ic) in tris:
            a, b, c = verts[ia], verts[ib], verts[ic]
            u, v = sub(b, a), sub(c, a)
            nx = u[1] * v[2] - u[2] * v[1]
            ny = u[2] * v[0] - u[0] * v[2]
            nz = u[0] * v[1] - u[1] * v[0]
            Ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(struct.pack("<3f", nx / Ln, ny / Ln, nz / Ln))
            for p in (a, b, c):
                f.write(struct.pack("<3f", *p))
            f.write(struct.pack("<H", 0))


def manifold_report(verts, tris):
    """Quick watertightness check: every edge should be shared by exactly two triangles."""
    from collections import Counter
    edges = Counter()
    for (a, b, c) in tris:
        for u, w in ((a, b), (b, c), (c, a)):
            edges[(u, w) if u < w else (w, u)] += 1
    bad = sum(1 for n in edges.values() if n != 2)
    print(f"  manifold check: {len(edges)} edges, {bad} non-manifold "
          f"({'WATERTIGHT' if bad == 0 else 'OPEN — check params'})")


if __name__ == "__main__":
    verts, tris = build()
    manifold_report(verts, tris)
    stl_path = os.path.join(HERE, "mandelbulb.stl")
    write_stl_binary(stl_path, verts, tris)
    print(f"wrote {stl_path}  ({len(verts)} verts, {len(tris)} triangles)")
