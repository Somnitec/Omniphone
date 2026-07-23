# Omniphone Sculptor — Blender port, slices 1+2: body, brain-coral pads,
# screen platform, USB/jack panel flats — with live parameter updates.
#
# Pipeline:  icosphere ball → squish → gradient-flatten underside → flat-facet
# attractors (screen platform + USB + jack: vertices pulled onto a disc plane
# with a settable merge shoulder) → Gray-Scott reaction-diffusion grown on the
# mesh graph, seeded at N pad centres (phyllotaxis / random / Lloyd, repelled
# from the screen cone) → displace along normals (ridges out / grooves in),
# faded below the coverage line and masked off the facets.
#
# Live updates: parameters rebuild only the stages they invalidate.
#   shape & displacement sliders  → tens of ms, update while dragging
#   pad/coral/RD parameters       → the sim restarts and GROWS in the viewport
#     (time-sliced on a timer, ~5 fps), settling at the exact final pattern
#   RD seeds live in unit-sphere space, so body-proportion sliders re-pose the
#   body with the coral pattern riding along stably instead of re-rolling it.
#
# Coming in later slices: flat bottom cut + 2 mm push-in / 4 mm shrink rabbet,
# 3 mm shell, screen/port apertures, parametric PCB holder trimmed to shell,
# bottom plate + 3× M3 self-tap bosses, pad seats/recesses.
#
# Install:  Edit ▸ Preferences ▸ Add-ons ▸ Install… → this file, enable it.
#           Panel lives in the 3D view sidebar (N key) → "Omniphone" tab.
# Headless: blender -b --factory-startup --python-exit-code 1 \
#               --python omniphone_sculptor.py -- --selftest
#
# Everything is millimetres; generate sets the scene units to match.

bl_info = {
    "name": "Omniphone Sculptor",
    "author": "Omniphone project",
    "version": (0, 2, 0),
    "blender": (4, 2, 0),
    "location": "3D View ▸ Sidebar ▸ Omniphone",
    "description": "Generative Omniphone enclosure bodies (coral + facets slice)",
    "category": "Add Mesh",
}

import json
import sys
import time
import random as pyrandom
from math import asin, ceil, cos, pi, radians, sin, sqrt

import bpy
import bmesh
import numpy as np
from bpy.props import (BoolProperty, EnumProperty, FloatProperty, IntProperty,
                       PointerProperty)
from mathutils import Vector

GOLDEN_ANGLE = pi * (3.0 - sqrt(5.0))
BODY_NAME = "Omniphone"
COLL_NAME = "Omniphone"
PADS_COLL_NAME = "Omniphone Pads"
MARKER_NAMES = ("Screen", "USB", "Jack")


# ---------------------------------------------------------------------------
# numpy geometry core
# ---------------------------------------------------------------------------

def make_icosphere(subdiv):
    """Unit icosphere as (verts float64 (n,3), faces int64 (m,3))."""
    bm = bmesh.new()
    bmesh.ops.create_icosphere(bm, subdivisions=int(subdiv), radius=1.0)
    bm.verts.ensure_lookup_table()
    verts = np.array([v.co[:] for v in bm.verts], dtype=np.float64)
    faces = np.array([[v.index for v in f.verts] for f in bm.faces], dtype=np.int64)
    bm.free()
    return verts, faces


def neighbor_table(n, faces):
    """Padded neighbour indices for vectorised graph ops: (nbr (n,maxdeg)
    padded with self, deg (n,) f32, pad (n,) f32 = paddings per row)."""
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    e.sort(axis=1)
    e = np.unique(e, axis=0)
    ed = np.concatenate([e, e[:, ::-1]])
    ed = ed[np.argsort(ed[:, 0], kind="stable")]
    deg = np.bincount(ed[:, 0], minlength=n)
    maxdeg = int(deg.max())
    start = np.concatenate([[0], np.cumsum(deg)[:-1]]).astype(np.int64)
    col = np.arange(len(ed)) - start[ed[:, 0]]
    nbr = np.tile(np.arange(n, dtype=np.int64)[:, None], (1, maxdeg))
    nbr[ed[:, 0], col] = ed[:, 1]
    return nbr, deg.astype(np.float32), (maxdeg - deg).astype(np.float32)


def nb_mean(f, nbr, deg, pad):
    return (f[nbr].sum(axis=1) - pad * f) / deg


def vertex_normals(verts, faces):
    fv = verts[faces]
    fn = np.cross(fv[:, 1] - fv[:, 0], fv[:, 2] - fv[:, 0])  # area-weighted
    vn = np.zeros_like(verts)
    for c in range(3):
        np.add.at(vn, faces[:, c], fn)
    L = np.linalg.norm(vn, axis=1, keepdims=True)
    L[L == 0] = 1.0
    return vn / L


def smoothstep(a, b, x):
    t = np.clip((x - a) / max(b - a, 1e-9), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def squish_and_flatten(sphere_mm, diameter, squish_h, final_h, zone):
    """Scale z so the ball is squish_h tall, then compress everything below a
    blend line with a monotone Hermite map so the body ends final_h tall with a
    progressively flatter underside. Output rests on z = 0."""
    v = sphere_mm.copy()
    v[:, 2] *= squish_h / diameter
    z = v[:, 2]
    z_min = -squish_h / 2.0
    z_floor = squish_h / 2.0 - final_h
    if z_floor <= z_min + 1e-6:
        v[:, 2] = z - z_min
        return v
    z_blend = min(z_floor + zone, squish_h * 0.45)
    span_in, span_out = z_blend - z_min, z_blend - z_floor
    m1 = min(span_in, 3.0 * span_out)  # Fritsch–Carlson clamp keeps it monotone;
    #                                    slight crease only at extreme flatten + tiny zone
    t = np.clip((z - z_min) / span_in, 0.0, 1.0)
    zH = (2 * t**3 - 3 * t**2 + 1) * z_floor + (-2 * t**3 + 3 * t**2) * z_blend + (t**3 - t**2) * m1
    v[:, 2] = np.where(z < z_blend, zH, z) - z_floor
    return v


def flatten_facet(verts, unit_dirs, d, c0, disc_r, merge, strength, plane_offset=0.0,
                  t_plane_force=None):
    """Pull vertices onto a disc plane ⟂ d (axis through c0): the user's
    'circular flat surface that attracts vertices around it'. Inside disc_r the
    pull is total (a truly flat seat); outside it falls off over `merge` mm,
    scaled by `strength`. plane_offset pushes the plane out (+) / sinks it (−)
    along d. Participation is gated three ways: front side only (unit_dir·d),
    radially from the axis, AND axially near the outer surface — without the
    axial window the pull cylinder passes through the whole body and drags a
    trench where it grazes the far wall/underside (side-mounted ports!).
    Returns (new_verts, coral_suppression 0..1, t_plane)."""
    rel = verts - c0
    t = rel @ d
    r = np.linalg.norm(rel - t[:, None] * d, axis=1)
    front = (unit_dirs @ d) > 0.15
    gate = front & (r < disc_r) & (t > 0)
    if not gate.any():                                   # disc smaller than mesh res
        gate = front & (r < disc_r * 2) & (t > 0)
    if not gate.any():
        return verts, np.ones(len(verts)), 0.0
    # anchor the plane at the *mean* height of the outer surface inside the
    # disc (not the max): on a sloped wall the facet then half-recesses like a
    # real panel mount instead of standing fully proud. Only verts near the
    # outer extremity count — the gate tube may also clip other body parts.
    tg = t[gate]
    t_surf = float(tg[tg > tg.max() - 1.6 * disc_r].mean())
    # t_plane_force pins the plane to a shared value (coplanar ports) instead of
    # tracking the local surface; t_surf still anchors the locality ball below.
    t_plane = t_surf + plane_offset if t_plane_force is None else float(t_plane_force)
    # locality: full pull within a ball around the seat anchor (covers the whole
    # disc even on a steeply sloped wall, where seat verts sit well below the
    # plane), zero beyond 1.5× — kills the far wall/underside inside the tube.
    anchor = c0 + d * t_surf
    dist = np.linalg.norm(verts - anchor, axis=1)
    # 2·disc_r term: on a steep wall the disc's own lower edge sits ~disc-widths
    # below the anchor and must still get the full pull
    reach_full = max(disc_r + merge + 4.0, 2.0 * disc_r)
    w_ball = 1.0 - smoothstep(reach_full, 1.5 * reach_full, dist)
    w = np.where(r < disc_r, 1.0,
                 strength * (1.0 - smoothstep(disc_r, disc_r + max(merge, 1e-3), r)))
    w *= front * w_ball
    out = verts + d[None, :] * ((t_plane - t) * w)[:, None]
    # coral fades out approaching the seat so ridges never wrinkle the flat
    supp_r = smoothstep(disc_r + 2.0, disc_r + 2.0 + max(merge * 0.6, 3.0), r)
    supp = 1.0 - (1.0 - supp_r) * front * w_ball
    return out, supp, t_plane


def pad_layout(p, rng, screen_d, min_ang):
    """N unit directions on the band [band_lo, band_hi] (z of the unit sphere).
    Layouts: phyllotaxis / uniform random / random + repulsion relaxation.
    Pads inside the screen cone are rotated out to its rim (sculptor v2)."""
    n = int(p.pad_count)
    if n == 0:
        return np.zeros((0, 3))
    lo = min(p.band_lo, p.band_hi - 1e-3)
    hi = p.band_hi
    if p.pad_layout in ("PHYLLO", "EVEN"):
        i = np.arange(n)
        z = hi - (hi - lo) * ((i + 0.5) / n)
        th = i * GOLDEN_ANGLE
    else:
        z = rng.uniform(lo, hi, n)
        th = rng.uniform(0.0, 2 * pi, n)
    r = np.sqrt(np.clip(1 - z * z, 0, None))
    dirs = np.stack([r * np.cos(th), r * np.sin(th), z], axis=1)

    if p.pad_layout == "EVEN":
        iters = 800                                      # run to convergence: maximally
    elif p.pad_layout == "LLOYD":                        # even spacing → equal-size pads
        iters = max(int(p.relax_iters), 15)
    else:
        iters = int(p.relax_iters)
    prev = dirs.copy()
    for it in range(iters):
        dd = dirs[:, None, :] - dirs[None, :, :]
        d2 = (dd * dd).sum(-1) + 1e-6
        w = 1.0 / (d2 * np.sqrt(d2))
        np.fill_diagonal(w, 0.0)
        dirs = dirs + (dd * w[:, :, None]).sum(axis=1) * (0.35 / n)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        z = np.clip(dirs[:, 2], lo, hi)
        r = np.sqrt(np.clip(1 - z * z, 0, None))
        xy = np.linalg.norm(dirs[:, :2], axis=1)
        xy[xy < 1e-9] = 1.0
        dirs = np.stack([dirs[:, 0] / xy * r, dirs[:, 1] / xy * r, z], axis=1)
        if p.pad_layout == "EVEN":                       # stop once settled
            if float(np.abs(dirs - prev).max()) < 1e-5:
                break
            prev = dirs.copy()

    if screen_d is not None and min_ang > 0:
        for i in range(n):
            c = float(np.clip(dirs[i] @ screen_d, -1, 1))
            ang = np.arccos(c)
            if ang < min_ang:                    # rotate out to the cone rim
                perp = dirs[i] - c * screen_d
                L = np.linalg.norm(perp)
                if L < 1e-6:
                    perp = np.array([screen_d[2], 0.0, -screen_d[0]])
                    L = np.linalg.norm(perp)
                perp /= L
                dirs[i] = screen_d * cos(min_ang) + perp * sin(min_ang)
    return dirs


def _pad_dirs_for(p):
    """The pad layout (same rng, same screen repel) every consumer shares —
    coral seeds, Voronoi cells, crystal facets and pad markers all coincide."""
    rng = np.random.default_rng(int(p.rng_seed))
    R = p.diameter / 2.0
    sd, min_ang = None, 0.0
    if p.screen_on:
        sd = _screen_dir(p)
        min_ang = asin(min((p.screen_dia / 2.0 + p.seed_bulb_r + 2.0) / (R * 0.85), 0.95))
    return pad_layout(p, rng, sd, min_ang)


def pads_apply(p):
    """Do pads mean anything right now? The coral seeds on them, a Voronoi
    form grows a cell per pad, a Crystal form turns each into a flat facet.
    Cymatics/Mandelbulb forms and the texture pass ignore them."""
    return p.body_algo in ("VORONOI", "CRYSTAL") or p.coral_on


def coral_active(p):
    return p.coral_on


def rd_setup(p, geo):
    """Stamp seed bulbs at the pad layout (in unit-sphere angle space, so the
    field is independent of squish/flatten), init Gray-Scott state.
    Deterministic per rng_seed. Returns a state dict that rd_run() advances."""
    rng = np.random.default_rng(int(p.rng_seed) + 7919)  # noise stream, layout has its own
    dirs = stage_pads(p, geo)["dirs"]
    R = p.diameter / 2.0
    n = len(geo["unit"])
    seed_mask = np.zeros(n, dtype=bool)
    centers = np.empty(len(dirs), dtype=np.int64)
    cos_bulb = cos(min(p.seed_bulb_r / R, 1.2))
    for i, d in enumerate(dirs):
        dots = geo["unit"] @ d
        centers[i] = int(np.argmax(dots))
        seed_mask |= dots > cos_bulb
    U = np.ones(n, dtype=np.float32)
    V = np.zeros(n, dtype=np.float32)
    if p.rd_noise > 0:
        V += (p.rd_noise * rng.random(n)).astype(np.float32)
    U[seed_mask] = 0.25
    V[seed_mask] = 0.5
    scale = float(p.rd_scale)
    nsub = max(1, int(ceil(scale * scale)))
    return {"U": U, "V": V, "centers": centers,
            "nbr": geo["nbr"], "deg": geo["deg"], "pad": geo["pad"],
            "nsub": nsub, "dt": np.float32(1.0 / nsub),
            "Du4": np.float32(4.0 * 0.16 * scale * scale),
            "Dv4": np.float32(4.0 * 0.08 * scale * scale),
            "F": np.float32(p.rd_feed), "k": np.float32(p.rd_kill),
            "step": 0, "total": int(p.rd_steps)}


def rd_run(st, max_steps):
    """Advance Gray-Scott on the mesh graph by up to max_steps; True = finished.
    Laplacian = 4·(neighbour-mean − u) reduces to the classic 4-neighbour grid
    stencil, so familiar F/k pairs carry over. Pattern scale s runs diffusion at
    s² with ⌈s²⌉ substeps (wavelength ∝ √D, cost ∝ s²)."""
    U, V = st["U"], st["V"]
    nbr, deg, pad = st["nbr"], st["deg"], st["pad"]
    dt, Du4, Dv4, F, k = st["dt"], st["Du4"], st["Dv4"], st["F"], st["k"]
    one = np.float32(1.0)
    n = min(int(max_steps), st["total"] - st["step"])
    for _ in range(n):
        for _ in range(st["nsub"]):
            lu = (U[nbr].sum(axis=1) - pad * U) / deg - U
            lv = (V[nbr].sum(axis=1) - pad * V) / deg - V
            uvv = U * V * V
            U += dt * (Du4 * lu - uvv + F * (one - U))
            V += dt * (Dv4 * lv + uvv - (F + k) * V)
    st["step"] += n
    return st["step"] >= st["total"]


# ---------------------------------------------------------------------------
# staged pipeline with per-stage cache — the heart of live updates.
# A stage rebuilds only when its key changes; expensive stages (geo, rd) are
# only *allowed* to rebuild from debounced/full runs, so fast sliders reuse
# them stale-but-stable.
# ---------------------------------------------------------------------------

_cache = {}
_suspend = False          # guard while operators set props programmatically


def _stage(name, key, allow_slow, is_slow, builder):
    ent = _cache.get(name)
    if ent is not None and ent[0] == key:
        return ent[1]
    if ent is not None and is_slow and not allow_slow:
        return ent[1]                                    # stale but stable
    val = builder()
    _cache[name] = (key, val)
    return val


def _key_geo(p):
    return (p.resolution,)


def _pads_part(p):
    """Everything the pad layout depends on — shared by every key that
    consumes stage_pads."""
    return tuple(round(float(x), 4) for x in (
        p.diameter, p.pad_count, hash(p.pad_layout) % 10**6, p.relax_iters,
        p.band_lo, p.band_hi, p.seed_bulb_r, p.rng_seed,
        p.screen_on, p.screen_tilt, p.screen_az, p.screen_dia))


def _key_pads(p):
    return _key_geo(p) + _pads_part(p) + (pads_apply(p),)


def _key_body(p):
    key = _key_geo(p) + (p.body_algo, round(float(p.diameter), 3))
    if p.body_algo == "VORONOI":
        key += _pads_part(p) + tuple(round(float(x), 4)
                                     for x in (p.cell_amp, p.cell_groove))
    elif p.body_algo == "CRYSTAL":
        key += _pads_part(p) + tuple(round(float(x), 4)
                                     for x in (p.facet_dist, p.facet_jitter))
    elif p.body_algo == "CYMATICS":
        key += tuple(round(float(x), 4) for x in (p.cy_m, p.cy_n, p.cy_amp))
    elif p.body_algo == "MANDELBULB":
        key += tuple(round(float(x), 4) for x in (p.mb_power, p.mb_iter, p.mb_level))
    return key


def _key_tex(p):
    key = _key_geo(p) + (p.tex_algo, round(float(p.tex_amount), 3))
    if p.tex_algo in ("VORONOI", "CRYSTAL"):
        key += tuple(round(float(x), 4) for x in
                     (p.tex_count, p.tex_seed, p.tex_jitter, p.tex_groove))
    elif p.tex_algo == "CYMATICS":
        key += (int(p.tex_cy_m), int(p.tex_cy_n))
    elif p.tex_algo == "MANDELBULB":
        key += (int(p.tex_mb_power), int(p.tex_mb_iter))
    return key


def _key_shape(p):
    return _key_body(p) + tuple(round(float(x), 4) for x in (
        p.diameter, p.squish_height, p.final_height, p.flatten_zone,
        p.screen_on, p.screen_tilt, p.screen_az, p.screen_dia, p.screen_height,
        p.screen_merge, p.screen_strength, p.screen_pitch,
        p.ports_coplanar, p.ports_az, p.ports_z, p.ports_pitch, p.ports_height, p.ports_gap,
        p.ports_merge, p.ports_strength,
        p.usb_on, p.usb_az, p.usb_z, p.usb_height, p.usb_dia, p.usb_merge, p.usb_pitch,
        p.jack_on, p.jack_az, p.jack_z, p.jack_height, p.jack_dia, p.jack_merge, p.jack_pitch))


def _key_rd(p):
    return _key_geo(p) + _pads_part(p) + tuple(round(float(x), 4) for x in (
        p.rd_feed, p.rd_kill, p.rd_steps, p.rd_scale, p.rd_noise))


def _screen_dir(p):
    st, sa = radians(p.screen_tilt), radians(p.screen_az)
    return np.array([sin(st) * cos(sa), sin(st) * sin(sa), cos(st)])


def _facet_frame(verts, n_pre, unit_dirs, d_pos, az_rad, pitch_deg, disc_r, z_slab=None,
                 absolute=False):
    """Where a facet sits and which way it faces. d_pos (a spoke direction or a
    horizontal ray) picks the LOCATION; the facet NORMAL defaults to the local
    smooth surface normal there (disc lies tangent, like a coin on the ball)
    and pitch_deg rotates it in the azimuth's vertical plane (+ = face up).
    absolute=True: pitch is measured from horizontal instead (0° = the face is
    vertical / perpendicular to the ground — panel-mount ports).
    Returns (anchor_point, face_normal)."""
    dots = unit_dirs @ d_pos
    cone = dots > 0.94
    if z_slab is not None:
        cone = (dots > 0.80) & (np.abs(verts[:, 2] - z_slab) < max(disc_r * 0.7, 6.0))
    if cone.sum() < 16:
        cone = dots > 0.80
    cv = verts[cone]
    anchor = cv[np.argmax(cv @ d_pos)]
    if absolute:
        phi = pi / 2 - radians(pitch_deg)
    else:
        near = cone & (((verts - anchor) ** 2).sum(axis=1) < disc_r * disc_r * 4)
        ns = n_pre[near].mean(axis=0)
        ns /= np.linalg.norm(ns) or 1.0
        # pre-facet body is axisymmetric, so ns lies in the azimuth plane: work
        # in polar angle from +z there
        phi = np.arctan2(np.hypot(ns[0], ns[1]), ns[2]) - radians(pitch_deg)
    return anchor, np.array([sin(phi) * cos(az_rad), sin(phi) * sin(az_rad), cos(phi)])


def stage_geo(p, allow_slow):
    def build():
        unit, faces = make_icosphere(p.resolution)
        nbr, deg, pad = neighbor_table(len(unit), faces)
        return {"unit": unit, "faces": faces, "nbr": nbr, "deg": deg, "pad": pad}
    return _stage("geo", _key_geo(p), allow_slow, True, build)


def stage_rd(p, geo, allow_slow, log=print):
    def build():
        st = rd_setup(p, geo)
        log(f"  RD: {st['total']} steps × {st['nsub']} substeps on {len(st['U'])} verts…")
        rd_run(st, st["total"])
        return {"field": st["V"], "centers": st["centers"]}
    return _stage("rd", _key_rd(p), allow_slow, True, build)


def _fib_sphere(n):
    i = np.arange(n)
    z = 1.0 - 2.0 * (i + 0.5) / n
    r = np.sqrt(np.clip(1 - z * z, 0, None))
    th = i * GOLDEN_ANGLE
    return np.stack([r * np.cos(th), r * np.sin(th), z], axis=1)


_bessel_cache = {}


def _bessel_profile(m, n_zero):
    """Normalised J_m radial profile out to its n-th zero (257 samples)."""
    key = (int(m), int(n_zero))
    if key in _bessel_cache:
        return _bessel_cache[key]
    K = 48
    t = np.linspace(0, pi, K + 1)
    w = np.ones(K + 1)
    w[1:-1:2], w[2:-1:2] = 4.0, 2.0

    def J(x):
        x = np.asarray(x, dtype=np.float64).reshape(-1)
        return (np.cos(m * t[None, :] - x[:, None] * np.sin(t)[None, :])
                * w[None, :]).sum(axis=1) * (pi / K / 3.0) / pi

    xs = np.arange(1e-4, 200.0, 0.05)
    vals = J(xs)
    flips = np.where(np.sign(vals[1:]) * np.sign(vals[:-1]) < 0)[0]
    if len(flips) >= n_zero:
        lo, hi = xs[flips[n_zero - 1]], xs[flips[n_zero - 1] + 1]
        flo = J([lo])[0]
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            fm = J([mid])[0]
            if (fm < 0) == (flo < 0):
                lo, flo = mid, fm
            else:
                hi = mid
        z1 = 0.5 * (lo + hi)
    else:
        z1 = 30.0
    prof = J(np.linspace(0, 1, 257) * z1)
    prof /= max(np.abs(prof).max(), 1e-9)
    _bessel_cache[key] = (prof, int(m))
    return _bessel_cache[key]


def _mb_f(q, power, iters):
    """Mandelbulb distance estimator (solid-positive, bulb units), vectorised."""
    zx, zy, zz = q[:, 0].copy(), q[:, 1].copy(), q[:, 2].copy()
    dr = np.ones(len(q))
    for _ in range(iters):
        r = np.sqrt(zx * zx + zy * zy + zz * zz)
        act = r < 2.0
        rs = np.maximum(r, 1e-9)
        dr = np.where(act, power * rs ** (power - 1) * dr + 1.0, dr)
        th = np.arccos(np.clip(zz / rs, -1, 1)) * power
        ph = np.arctan2(zy, zx) * power
        zr = rs ** power
        st = np.sin(th)
        zx = np.where(act, q[:, 0] + zr * st * np.cos(ph), zx)
        zy = np.where(act, q[:, 1] + zr * st * np.sin(ph), zy)
        zz = np.where(act, q[:, 2] + zr * np.cos(th), zz)
    rs = np.maximum(np.sqrt(zx * zx + zy * zy + zz * zz), 1e-9)
    return -0.5 * np.log(rs) * rs / dr


def _mandelbulb_radii(u, power, iters, level_mm, R):
    """Outermost bulb crossing along each unit direction (lockstep march +
    bisection), normalised so the widest span equals the ball diameter."""
    lvl = level_mm / max(R, 1e-6)
    ts = np.linspace(1.45, 0.03, 52)
    t_hit = np.full(len(u), np.nan)
    for k, t in enumerate(ts):
        rem = np.isnan(t_hit)
        if not rem.any():
            break
        f = _mb_f(u[rem] * (t * 1.2), power, iters) + lvl
        idx = np.where(rem)[0][f >= 0]
        t_hit[idx] = t
    t_hit = np.where(np.isnan(t_hit), 0.25, t_hit)
    lo = t_hit
    hi = np.minimum(t_hit + (ts[0] - ts[1]), 1.5)
    for _ in range(9):
        mid = 0.5 * (lo + hi)
        ins = _mb_f(u * (mid[:, None] * 1.2), power, iters) + lvl >= 0
        lo = np.where(ins, mid, lo)
        hi = np.where(ins, hi, mid)
    t = 0.5 * (lo + hi)
    return t / max(float(t.max()), 1e-9) * R


def stage_pads(p, geo):
    """Pad centre directions + nearest mesh vertex per pad — the single source
    every consumer (coral seeds, Voronoi cells, crystal facets, pad markers)
    shares. Empty when the current algorithm has no use for pads."""
    def build():
        if not pads_apply(p) or p.pad_count < 1:
            return {"dirs": np.zeros((0, 3)), "centers": np.zeros(0, dtype=np.int64)}
        dirs = np.asarray(_pad_dirs_for(p), dtype=np.float64)
        centers = (geo["unit"] @ dirs.T).argmax(axis=0).astype(np.int64)
        return {"dirs": dirs, "centers": centers}
    return _stage("pads", _key_pads(p), True, False, build)


def stage_body(p, geo):
    """Base radial shape on the unit sphere, before squish/flatten/facets/coral.
    Cached separately so facet or coral slider drags never recompute it."""
    def build():
        R = p.diameter / 2.0
        u = geo["unit"]
        if p.body_algo == "VORONOI":
            dirs = stage_pads(p, geo)["dirs"]
            if len(dirs) < 4:
                dirs = _fib_sphere(12)
            dots = u @ dirs.T
            part = np.partition(dots, -2, axis=1)
            rad = R + p.cell_amp * np.tanh((part[:, -1] - part[:, -2]) / p.cell_groove)
        elif p.body_algo == "CRYSTAL":
            # every pad becomes a flat facet; a crown facet covers the
            # screen-repelled bald spot (the platform lands on a real face);
            # fibonacci filler facets below the pad band close the underside
            pdirs = stage_pads(p, geo)["dirs"]
            fill = _fib_sphere(max(12, int(p.pad_count * 2.2)))
            crown = (_screen_dir(p) if p.screen_on else np.array([0.0, 0.0, 1.0]))[None, :]
            if len(pdirs):
                dirs = np.concatenate([pdirs, crown, fill[fill[:, 2] < p.band_lo - 0.05]])
            else:
                dirs = fill
            rng = np.random.default_rng(int(p.rng_seed) + 31)
            jit = p.facet_jitter * (rng.random(len(dirs)) - 0.5) * 2
            if len(pdirs):
                jit[len(pdirs)] = 0.0                     # crown stays put
            dd = R * p.facet_dist * (1 + jit)
            dots = u @ dirs.T
            rr = np.where(dots > 0.05, dd[None, :] / np.maximum(dots, 0.05), np.inf)
            rad = np.minimum(rr.min(axis=1), R * 1.05)
        elif p.body_algo == "CYMATICS":
            prof, m = _bessel_profile(p.cy_m, p.cy_n)
            phi = np.arccos(np.clip(u[:, 2], -1, 1))
            uu = np.clip(phi / (pi / 2), 0.0, 1.0)
            fade = 1.0 - smoothstep(pi * 0.5, pi * 0.67, phi)  # quiet below the equator
            rad = R + p.cy_amp * np.interp(uu, np.linspace(0, 1, 257), prof) \
                * np.cos(m * np.arctan2(u[:, 1], u[:, 0])) * fade
        elif p.body_algo == "MANDELBULB":
            rad = _mandelbulb_radii(u, float(p.mb_power), int(p.mb_iter), p.mb_level, R)
        else:                                            # BALL: plain canvas
            rad = np.full(len(u), R)
        return {"verts": u * rad[:, None]}
    return _stage("body", _key_body(p), True, False, build)


def _tex_dirs(p):
    """Feature directions for the texture pass — fibonacci coverage of the
    whole sphere, randomly rotated per seed, decoupled from the pad layout
    (texture is decoration, pads are the instrument)."""
    rng = np.random.default_rng(int(p.tex_seed) + 101)
    n = max(4, int(p.tex_count))
    dirs = _fib_sphere(n)
    ax = rng.normal(size=3)
    ax /= np.linalg.norm(ax) or 1.0
    dirs = dirs @ _rot_axis(ax, rng.uniform(0, 2 * pi)).T
    if p.tex_jitter > 0 and p.tex_algo == "VORONOI":
        dirs = dirs + rng.normal(0.0, p.tex_jitter * 1.8 / sqrt(n), dirs.shape)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    return dirs


def stage_tex(p, geo):
    """Second-pass texture: any algorithm as a normalised field × amount (mm),
    displaced along the shaped surface's normals (same slot the coral uses, so
    both stack). None = no texture layer."""
    def build():
        if p.tex_algo == "NONE" or abs(p.tex_amount) < 1e-4:
            return None
        u = geo["unit"]
        if p.tex_algo == "VORONOI":
            dots = u @ _tex_dirs(p).T
            part = np.partition(dots, -2, axis=1)
            gap = part[:, -1] - part[:, -2]
            m = max(float(np.percentile(gap, 96.0)), 1e-6)
            f = np.tanh(gap / (p.tex_groove * m))        # groove ∝ cell size → plateaus at any count
            f /= max(float(f.max()), 1e-6)
        elif p.tex_algo == "CRYSTAL":
            return None                                  # grind pass in stage_displace
        elif p.tex_algo == "CYMATICS":
            prof, m = _bessel_profile(p.tex_cy_m, p.tex_cy_n)
            phi = np.arccos(np.clip(u[:, 2], -1, 1))
            uu = np.clip(phi / (pi / 2), 0.0, 1.0)
            fade = 1.0 - smoothstep(pi * 0.5, pi * 0.67, phi)
            f = np.interp(uu, np.linspace(0, 1, 257), prof) \
                * np.cos(m * np.arctan2(u[:, 1], u[:, 0])) * fade
        elif p.tex_algo == "MANDELBULB":
            t = _mandelbulb_radii(u, float(p.tex_mb_power), int(p.tex_mb_iter), 0.0, 1.0)
            f = t - t.mean()
            f /= max(np.abs(f).max(), 1e-6)
        return f * p.tex_amount
    return _stage("tex", _key_tex(p), True, False, build)


def stage_shape(p, geo):
    def build():
        final_h = min(p.final_height, p.squish_height)
        verts = squish_and_flatten(stage_body(p, geo)["verts"], p.diameter,
                                   p.squish_height, final_h, p.flatten_zone)
        n_pre = vertex_normals(verts, geo["faces"])      # pre-facet, for tangent frames
        supp = np.ones(len(verts))
        facets = []
        if p.screen_on:
            anchor, d = _facet_frame(verts, n_pre, geo["unit"], _screen_dir(p),
                                     radians(p.screen_az), p.screen_pitch,
                                     p.screen_dia / 2.0)
            c0 = anchor - d * 40.0
            verts, s, t_plane = flatten_facet(verts, geo["unit"], d, c0,
                                              p.screen_dia / 2.0, p.screen_merge,
                                              p.screen_strength,
                                              plane_offset=p.screen_height)
            supp = np.minimum(supp, s)
            facets.append(("Screen", d, c0, t_plane, p.screen_dia / 2.0))
        if p.ports_coplanar and (p.usb_on or p.jack_on):
            # TWO separate flats (one per connector, sized to it) forced onto ONE
            # shared plane so the cables exit parallel. `ports_*` face/place the
            # plane; each seat sits ±gap/2 along the in-plane horizontal axis.
            a = radians(p.ports_az)
            d_pos = np.array([cos(a), sin(a), 0.0])
            ref = max(p.usb_dia, p.jack_dia) / 2.0
            anchor, d = _facet_frame(verts, n_pre, geo["unit"], d_pos, a, p.ports_pitch,
                                     ref, z_slab=float(p.ports_z), absolute=True)
            plane_D = float(np.dot(anchor, d)) + p.ports_height   # shared plane: x·d = plane_D
            lat = np.array([-d[1], d[0], 0.0])
            lat = lat / (np.linalg.norm(lat) or 1.0)     # in-plane horizontal axis
            for nm, on, dia, sign in (("USB", p.usb_on, p.usb_dia, -1.0),
                                      ("Jack", p.jack_on, p.jack_dia, 1.0)):
                if not on:
                    continue
                c0 = anchor + lat * (sign * p.ports_gap / 2.0) - d * 40.0
                tpf = plane_D - float(np.dot(c0, d))     # this seat lands on the shared plane
                verts, s, t_plane = flatten_facet(verts, geo["unit"], d, c0, dia / 2.0,
                                                  p.ports_merge, p.ports_strength, t_plane_force=tpf)
                supp = np.minimum(supp, s)
                facets.append((nm, d, c0, t_plane, dia / 2.0))
        else:
            for name, on, az, z0, dia, merge, pitch, height in (
                    ("USB", p.usb_on, p.usb_az, p.usb_z, p.usb_dia, p.usb_merge, p.usb_pitch, p.usb_height),
                    ("Jack", p.jack_on, p.jack_az, p.jack_z, p.jack_dia, p.jack_merge, p.jack_pitch, p.jack_height)):
                if not on:
                    continue
                a = radians(az)
                d_pos = np.array([cos(a), sin(a), 0.0])
                anchor, d = _facet_frame(verts, n_pre, geo["unit"], d_pos, a, pitch,
                                         dia / 2.0, z_slab=float(z0), absolute=True)
                c0 = anchor - d * 40.0
                verts, s, t_plane = flatten_facet(verts, geo["unit"], d, c0, dia / 2.0,
                                                  merge, 1.0, plane_offset=height)
                supp = np.minimum(supp, s)
                facets.append((name, d, c0, t_plane, dia / 2.0))
        normals = vertex_normals(verts, geo["faces"])
        return {"verts": verts, "normals": normals, "supp": supp,
                "facets": facets, "final_h": final_h}
    return _stage("shape", _key_shape(p), True, False, build)


def _grind_facets(p, geo, verts, normals, supp):
    """CRYSTAL texture = gem grinder: within each texture cell the local crest
    is cut flat by up to Amount mm — sharp planar chips (a soft radial field
    never reads as 'faceted'). supp fades the cut out around the seats."""
    dirs = _tex_dirs(p)
    rng = np.random.default_rng(int(p.tex_seed) + 131)
    depth = abs(p.tex_amount) * (1 + p.tex_jitter * (rng.random(len(dirs)) - 0.5))
    cell = np.argmax(geo["unit"] @ dirs.T, axis=1)
    out = verts.copy()
    for i in range(len(dirs)):
        own = np.where(cell == i)[0]
        if len(own) < 8:
            continue
        nv = normals[own].mean(axis=0)
        nv /= np.linalg.norm(nv) or 1.0
        proj = out[own] @ nv
        over = proj - (proj.max() - depth[i])            # cut plane below the crest
        sel = over > 0
        if sel.any():
            idx = own[sel]
            out[idx] -= nv[None, :] * (over[sel] * supp[idx])[:, None]
    return out


def stage_displace(p, geo, shape, rd, tex):
    """Cheap final stage — never cached. Texture and coral both displace
    along the shaped surface's normals and stack; rd/tex=None = layer off."""
    amp = None
    if rd is not None:
        f01 = rd["field"].astype(np.float64)
        span = f01.max() - f01.min()
        f01 = (f01 - f01.min()) / (span if span > 1e-9 else 1.0)
        for _ in range(int(p.field_smooth)):
            f01 = 0.5 * f01 + 0.5 * nb_mean(f01, geo["nbr"], geo["deg"], geo["pad"])
        d = (f01 - 0.5) * 2.0
        amp = np.where(d > 0, d * p.ridge_height, d * p.groove_depth)
        if p.coverage < 0.999:
            z_thr = shape["final_h"] * (1.0 - p.coverage)
            amp *= smoothstep(z_thr, z_thr + p.cover_fade, shape["verts"][:, 2])
    if tex is not None:
        amp = tex if amp is None else amp + tex
    if amp is not None and p.seam_smooth > 0.01:
        # keep the plate-mating band smooth. The recess/rim/plate are all built
        # from the SMOOTH silhouette (silhouette_r on shape["verts"]); coral or
        # texture here displaces the *actual* wall off that silhouette — inward
        # grooves breach the rim (gaps), outward ridges thicken it (ridges), and
        # the ledge comes out ragged. Fade displacement to 0 below the ledge so
        # the wall the recess cuts equals the silhouette it was measured from.
        zl = float(p.bottom_cut) + float(p.plate_recess)
        amp = amp * smoothstep(zl, zl + float(p.seam_smooth), shape["verts"][:, 2])
    if amp is None:
        verts = shape["verts"]
    else:
        amp = amp * shape["supp"]                        # seats stay flat (new array)
        verts = shape["verts"] + shape["normals"] * amp[:, None]
    if p.tex_algo == "CRYSTAL" and abs(p.tex_amount) > 1e-4:
        verts = _grind_facets(p, geo, verts, shape["normals"], shape["supp"])
    return verts


# ---------------------------------------------------------------------------
# Blender writeback
# ---------------------------------------------------------------------------

def ensure_collection(name, parent=None):
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
    root = parent if parent is not None else bpy.context.scene.collection
    if name not in root.children:
        root.children.link(coll)
    return coll


def set_units_mm(scene):
    u = scene.unit_settings
    u.system = "METRIC"
    u.scale_length = 0.001
    u.length_unit = "MILLIMETERS"


def write_body(verts, faces, coll):
    """Fast path: same topology → foreach_set coords. Else rebuild datablock."""
    obj = bpy.data.objects.get(BODY_NAME)
    if obj is not None and obj.type == "MESH" and len(obj.data.vertices) == len(verts):
        me = obj.data
        me.vertices.foreach_set("co", np.ascontiguousarray(verts, dtype=np.float32).ravel())
        me.update()
        return obj
    me = bpy.data.meshes.new(BODY_NAME)
    me.from_pydata(verts.tolist(), [], faces.tolist())
    me.validate()
    me.update()
    me.polygons.foreach_set("use_smooth", np.ones(len(me.polygons), dtype=bool))
    old = None
    if obj is None or obj.type != "MESH":
        obj = bpy.data.objects.new(BODY_NAME, me)
        coll.objects.link(obj)
    else:
        old = obj.data
        obj.data = me
    if old is not None and old.users == 0:
        bpy.data.meshes.remove(old)
    return obj


def write_pad_empties(coll, positions, size):
    have = [o for o in coll.objects if o.name.startswith("Pad")]
    if len(have) != len(positions):
        for o in have:
            bpy.data.objects.remove(o, do_unlink=True)
        have = []
        for i in range(len(positions)):
            e = bpy.data.objects.new(f"Pad.{i:02d}", None)
            e.empty_display_type = "SPHERE"
            coll.objects.link(e)
            have.append(e)
    have.sort(key=lambda o: o.name)
    for e, pos in zip(have, positions):
        e.empty_display_size = size
        e.location = pos


def write_markers(coll, facets):
    """Circle empties marking each facet plane (screen/usb/jack) — these carry
    the axes the aperture-cutting slice will use."""
    active = {f[0] for f in facets}
    for name in MARKER_NAMES:
        obj = bpy.data.objects.get(name)
        if name not in active:
            if obj is not None:
                bpy.data.objects.remove(obj, do_unlink=True)
            continue
        _, d, c0, t_plane, radius = next(f for f in facets if f[0] == name)
        if obj is None:
            obj = bpy.data.objects.new(name, None)
            coll.objects.link(obj)
        obj.empty_display_type = "CIRCLE"
        obj.empty_display_size = radius
        obj.location = Vector(c0 + d * t_plane)
        # circle empties draw in their local XY; aim local Z along the facet axis
        obj.rotation_euler = Vector(d).to_track_quat("Z", "Y").to_euler()


def props_as_dict(p):
    out = {}
    for name in p.__class__.__annotations__.keys():
        v = getattr(p, name)
        out[name] = round(v, 6) if isinstance(v, float) else (v if isinstance(v, (int, str)) else bool(v))
    return out


def rebuild(scene, allow_slow, log=print):
    """Run the (cached) pipeline and write every output object."""
    p = scene.omni_sculpt
    t0 = time.time()
    geo = stage_geo(p, allow_slow)
    pads = stage_pads(p, geo)
    rd = None
    if coral_active(p) and (allow_slow or _cache.get("rd") is not None):
        # fast path with an empty rd cache (coral just toggled on): skip —
        # the scheduled growth job publishes the field in a moment
        rd = stage_rd(p, geo, allow_slow, log=log)
    shape = stage_shape(p, geo)
    verts = stage_displace(p, geo, shape, rd, stage_tex(p, geo))
    set_units_mm(scene)
    coll = ensure_collection(COLL_NAME)
    obj = write_body(verts, geo["faces"], coll)
    try:
        obj.hide_set(False)                              # editing the body un-hides it
    except Exception:
        pass
    pads_coll = ensure_collection(PADS_COLL_NAME, parent=coll)
    write_pad_empties(pads_coll, verts[pads["centers"]], p.seed_bulb_r)
    write_markers(coll, shape["facets"])
    obj["omniphone_params"] = json.dumps(props_as_dict(p))
    obj["omniphone_pads"] = json.dumps([list(map(float, verts[i])) for i in pads["centers"]])
    obj["omniphone_facets"] = json.dumps(
        [{"name": n, "dir": list(map(float, d)), "origin": list(map(float, c0)),
          "plane_t": float(tp), "radius": float(r)} for n, d, c0, tp, r in shape["facets"]])
    try:
        cutplane_sync(scene)
    except Exception:
        pass
    try:
        pcb_preview_sync(scene)
    except Exception:
        pass
    return obj, verts, time.time() - t0


CUTPLANE_NAME = "Omniphone Cut Plane"


def cutplane_sync(scene, show=True):
    """Wire circle marking where Build will cut the bottom flat. Rides the
    bottom_cut slider live (it's a unit circle, only scale/z change)."""
    p = scene.omni_sculpt
    obj = bpy.data.objects.get(CUTPLANE_NAME)
    if obj is None:
        n = 64
        thc = np.linspace(0, 2 * pi, n, endpoint=False)
        v = np.column_stack([np.cos(thc), np.sin(thc), np.zeros(n)])
        edges = [[i, (i + 1) % n] for i in range(n)] + \
                [[0, n // 2], [n // 4, 3 * n // 4]]      # cross hair
        me = bpy.data.meshes.new(CUTPLANE_NAME)
        me.from_pydata(v.tolist(), edges, [])
        obj = bpy.data.objects.new(CUTPLANE_NAME, me)
        obj.display_type = "WIRE"
        obj.hide_select = True
        ensure_collection(COLL_NAME).objects.link(obj)
    r = p.diameter * 0.5 + 10
    obj.scale = (r, r, 1.0)
    obj.location = (0.0, 0.0, p.bottom_cut)
    try:
        obj.hide_set(not show)
    except Exception:
        pass


PCB_PREVIEW_NAME = "PCB Mount Preview"


def _pcb_mount_solids(p, zl):
    """(mount_parts, lattice_holes) as world-space (verts, faces) solids. Base spine
    + centre pillar, lifted `pcb_plate_offset` above the ledge; pillar square or round;
    optional grid of square through-holes in the spine leaving `pcb_lattice_wall`-thick
    struts (uniform, edges included). Shared by the live preview (parts only, shown
    un-clipped) and the Build pass (parts − holes, then clipped), so they never drift."""
    az = radians(p.pcb_az)
    sp = np.array([cos(az), sin(az), 0.0])
    pp = np.array([-sin(az), cos(az), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    ctr = np.array([p.pcb_x, p.pcb_y, 0.0])
    base = zl + float(p.pcb_plate_offset)                # mount floor, lifted off the plate
    seat, span, sh_h = float(p.pcb_seat_z), float(p.pcb_span), float(p.pcb_spine_h)
    parts = [solid_obox(ctr + up * (base + sh_h / 2.0), sp, pp, up,     # base spine
                        span / 2.0, p.pcb_spine_w / 2.0, sh_h / 2.0)]
    ptop = max(seat, base + sh_h + 1.0)                  # pillar top = seat plane
    if p.pcb_post_shape == "ROUND":
        parts.append(solid_frustum(ctr, up, p.pcb_post_w / 2.0, p.pcb_post_w / 2.0, base, ptop))
    else:
        parts.append(solid_obox(ctr + up * ((base + ptop) / 2.0), sp, pp, up,
                                p.pcb_post_len / 2.0, p.pcb_post_w / 2.0, (ptop - base) / 2.0))
    holes = []
    if p.pcb_lattice_on:
        t = float(p.pcb_lattice_wall)
        L, Wd = span, float(p.pcb_spine_w)              # grid over span (sp) × width (pp)…
        thru = sh_h + 2.0                                # …punched vertically (top→bottom, z)
        s0 = Wd - 2.0 * t                                # square target from the spine width
        # keep the spine solid directly under the pillar so it stays fused —
        # skip any cell whose footprint touches the pillar's (AABB test; round
        # pillar uses its Ø bounding box)
        px = (p.pcb_post_len if p.pcb_post_shape == "SQUARE" else p.pcb_post_w) / 2.0
        pw = float(p.pcb_post_w) / 2.0
        if s0 > 1.0:
            nx = max(1, int(round((L - t) / (s0 + t))))
            nw = max(1, int((Wd - t) // (s0 + t)))
            sx = (L - (nx + 1) * t) / nx                 # exact strut t along the span…
            sw = (Wd - (nw + 1) * t) / nw                # …and across the width
            if sx > 0.5 and sw > 0.5:
                cz = base + sh_h / 2.0
                for i in range(nx):
                    cx = -L / 2.0 + t + sx / 2.0 + i * (sx + t)
                    for j in range(nw):
                        cw = -Wd / 2.0 + t + sw / 2.0 + j * (sw + t)
                        if abs(cx) < px + sx / 2.0 and abs(cw) < pw + sw / 2.0:
                            continue                     # under the pillar — leave solid
                        holes.append(solid_obox(ctr + sp * cx + pp * cw + up * cz, sp, pp, up,
                                                sx / 2.0, sw / 2.0, thru / 2.0))
    return parts, holes


def pcb_preview_sync(scene, show=True):
    """Live solid preview of the PCB mount, riding the pcb_* sliders so it can be
    placed before Build. Shown UN-clipped — it may poke through the shell; Build
    shaves the overhang. Hidden once the real mount is baked into the shell."""
    p = scene.omni_sculpt
    obj = bpy.data.objects.get(PCB_PREVIEW_NAME)
    if not p.pcb_mount_on or not show:
        if obj is not None:
            try:
                obj.hide_set(True)
            except Exception:
                pass
        return
    zl = float(p.bottom_cut) + float(p.plate_recess)
    v, f = merge_solids(_pcb_mount_solids(p, zl)[0])      # parts only; lattice cut at Build
    me = bpy.data.meshes.new(PCB_PREVIEW_NAME)
    me.from_pydata(v.tolist(), [], f.tolist())
    me.update()
    if obj is None:
        obj = bpy.data.objects.new(PCB_PREVIEW_NAME, me)
        obj.hide_select = True                           # placed by sliders, not by grab
        obj.display_type = "SOLID"
        ensure_collection(COLL_NAME).objects.link(obj)
    else:
        old = obj.data
        obj.data = me
        if old.users == 0:
            bpy.data.meshes.remove(old)
    try:
        obj.hide_set(False)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# shell & plate build (slice 3) — engineering pass, boolean-based, not live.
# Bottom construction: the underside is a shallow dome, so the rabbet is built
# from z-specific silhouettes: cavity = 3 mm offset skin + polar core cut,
# a solid foot ring (with local pads under the screw bosses) fills the lip,
# then the recess prism carves the step the plate drops into.
# ---------------------------------------------------------------------------

def solid_polar_prism(r_theta, z0, z1):
    """Watertight star-shaped prism: cross-section radius r(θ), capped fans."""
    n = len(r_theta)
    th = np.linspace(0, 2 * pi, n, endpoint=False)
    ring = np.stack([r_theta * np.cos(th), r_theta * np.sin(th)], axis=1)
    vb = np.column_stack([ring, np.full(n, z0)])
    vt = np.column_stack([ring, np.full(n, z1)])
    verts = np.concatenate([vb, vt, [[0, 0, z0]], [[0, 0, z1]]])
    cb, ct = 2 * n, 2 * n + 1
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [[i, j, n + j], [i, n + j, n + i],       # side
                  [cb, j, i], [ct, n + i, n + j]]          # caps
    return verts, np.array(faces, dtype=np.int64)


def solid_annular_prism(r_in_theta, r_out, z0, z1):
    """Watertight ring prism between an inner polar profile and an outer radius
    (scalar or polar profile)."""
    n = len(r_in_theta)
    r_out = np.broadcast_to(np.asarray(r_out, dtype=np.float64), (n,))
    th = np.linspace(0, 2 * pi, n, endpoint=False)
    ci, si = np.cos(th), np.sin(th)
    vi0 = np.column_stack([r_in_theta * ci, r_in_theta * si, np.full(n, z0)])
    vi1 = np.column_stack([r_in_theta * ci, r_in_theta * si, np.full(n, z1)])
    vo0 = np.column_stack([r_out * ci, r_out * si, np.full(n, z0)])
    vo1 = np.column_stack([r_out * ci, r_out * si, np.full(n, z1)])
    verts = np.concatenate([vi0, vi1, vo0, vo1])
    I0, I1, O0, O1 = 0, n, 2 * n, 3 * n
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [[I0 + i, I1 + i, I1 + j], [I0 + i, I1 + j, I0 + j],   # inner wall
                  [O0 + i, O1 + j, O1 + i], [O0 + i, O0 + j, O1 + j],   # outer wall
                  [I0 + i, O0 + j, O0 + i], [I0 + i, I0 + j, O0 + j],   # bottom ring
                  [I1 + i, O1 + i, O1 + j], [I1 + i, O1 + j, I1 + j]]   # top ring
    return verts, np.array(faces, dtype=np.int64)


def solid_frustum(p0, d, r0, r1, t0, t1, n=48):
    """Watertight cone frustum (r0 at t0 → r1 at t1) along axis d through p0.
    r0 == r1 gives a cylinder."""
    d = np.asarray(d, dtype=np.float64)
    d = d / np.linalg.norm(d)
    a = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(a, d)
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    th = np.linspace(0, 2 * pi, n, endpoint=False)
    circ = np.outer(np.cos(th), u) + np.outer(np.sin(th), v)
    v0 = p0 + d * t0 + circ * max(r0, 1e-3)
    v1 = p0 + d * t1 + circ * max(r1, 1e-3)
    verts = np.concatenate([v0, v1, [p0 + d * t0], [p0 + d * t1]])
    c0, c1 = 2 * n, 2 * n + 1
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [[i, j, n + j], [i, n + j, n + i],
                  [c0, j, i], [c1, n + i, n + j]]
    return verts, np.array(faces, dtype=np.int64)


def _axes(d):
    """Orthonormal (u, v) perpendicular to unit axis d."""
    d = np.asarray(d, dtype=np.float64)
    a = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(a, d)
    u /= np.linalg.norm(u)
    return u, np.cross(d, u)


_BOX_F = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                   [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]],
                  dtype=np.int64)


def solid_obox(center, u, v, w, hu, hv, hw):
    """Watertight box with local frame (u,v,w) and half-extents (hu,hv,hw)."""
    c = np.asarray(center, dtype=np.float64)
    corners = []
    for sw in (-1, 1):
        for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            corners.append(c + u * (su * hu) + v * (sv * hv) + w * (sw * hw))
    return np.array(corners), _BOX_F.copy()


def solid_prism3(p1, p2, p3, d, depth):
    """Watertight triangular prism: triangle (p1,p2,p3) extruded depth along −d."""
    d = np.asarray(d, dtype=np.float64)
    if np.dot(np.cross(p2 - p1, p3 - p1), d) < 0:        # normalize winding
        p2, p3 = p3, p2
    lo = [p1 - d * depth, p2 - d * depth, p3 - d * depth]
    verts = np.array(lo + [p1, p2, p3])
    faces = np.array([[0, 2, 1], [3, 4, 5],
                      [0, 1, 4], [0, 4, 3], [1, 2, 5], [1, 5, 4], [2, 0, 3], [2, 3, 5]],
                     dtype=np.int64)
    return verts, faces


def merge_solids(parts):
    """Concatenate disjoint (verts, faces) solids into one mesh."""
    vs, fs, off = [], [], 0
    for v, f in parts:
        vs.append(v)
        fs.append(f + off)
        off += len(v)
    return np.concatenate(vs), np.concatenate(fs)


def silhouette_r(verts, z_center, half_band=2.0, n_theta=192, smooth=3):
    """Max-radius polar profile of the mesh in a z band (the body is
    star-shaped around z at the bottom). Gaps interpolated, circularly smoothed."""
    band = np.abs(verts[:, 2] - z_center) < half_band
    if band.sum() < 32:
        band = np.abs(verts[:, 2] - z_center) < half_band * 3
    v = verts[band]
    th = (np.arctan2(v[:, 1], v[:, 0])) % (2 * pi)
    r = np.hypot(v[:, 0], v[:, 1])
    bins = np.minimum((th / (2 * pi) * n_theta).astype(int), n_theta - 1)
    prof = np.full(n_theta, -1.0)
    np.maximum.at(prof, bins, r)
    missing = prof < 0
    if missing.any():                                    # circular interpolation
        idx = np.arange(n_theta)
        good = ~missing
        prof[missing] = np.interp((idx[missing]), np.concatenate([idx[good], idx[good] + n_theta]),
                                  np.concatenate([prof[good], prof[good]]), period=n_theta)
    k = np.array([1, 4, 6, 4, 1], dtype=np.float64)
    k /= k.sum()
    for _ in range(smooth):
        prof = sum(np.roll(prof, s) * w for s, w in zip(range(-2, 3), k))
    return prof


def _tmp_obj(name, verts, faces):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts.tolist(), [], faces.tolist())
    me.validate()
    me.update()
    obj = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _kill_obj(obj):
    me = obj.data
    bpy.data.objects.remove(obj, do_unlink=True)
    if me.users == 0:
        bpy.data.meshes.remove(me)


def remesh_voxel(obj, size):
    """VDB voxel remesh in place — resolves self-intersections by construction."""
    m = obj.modifiers.new("r", "REMESH")
    m.mode = "VOXEL"
    m.voxel_size = size
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(obj.evaluated_get(dg))
    obj.modifiers.remove(m)
    old = obj.data
    obj.data = me
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return obj


def bool_apply(obj, tool, op, use_self=False, label="", log=None, solver="MANIFOLD"):
    """obj = obj (op) tool; tool is consumed. Default is 4.5's manifold solver —
    all our operands are closed manifolds and the exact solver misclassifies
    inside/outside on the large hollowed shell. use_self implies EXACT."""
    m = obj.modifiers.new("b", "BOOLEAN")
    m.operation = op
    if use_self:
        solver = "EXACT"
    try:
        m.solver = solver
    except TypeError:                                    # Blender < 4.5: no MANIFOLD
        m.solver = "EXACT"
    m.use_self = use_self
    m.object = tool
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(obj.evaluated_get(dg))
    obj.modifiers.remove(m)
    old = obj.data
    obj.data = me
    if old.users == 0:
        bpy.data.meshes.remove(old)
    _kill_obj(tool)
    if log is not None and label:
        loops = np.zeros(len(me.loops), dtype=np.int64)
        me.loops.foreach_get("vertex_index", loops)
        starts = np.zeros(len(me.polygons), dtype=np.int64)
        totals = np.zeros(len(me.polygons), dtype=np.int64)
        me.polygons.foreach_get("loop_start", starts)
        me.polygons.foreach_get("loop_total", totals)
        e = np.concatenate([np.stack([loops[s:s + t], np.roll(loops[s:s + t], -1)], axis=1)
                            for s, t in zip(starts, totals)])
        e.sort(axis=1)
        uniq, counts = np.unique(e, axis=0, return_counts=True)
        bad = int((counts != 2).sum())
        vco = np.zeros(len(me.vertices) * 3, dtype=np.float64)
        me.vertices.foreach_get("co", vco)
        vco = vco.reshape(-1, 3)
        ext = vco.max(axis=0) - vco.min(axis=0) if len(vco) else np.zeros(3)
        msg = (f"    {label}: {len(me.polygons)} faces  "
               f"bbox {ext[0]:.0f}×{ext[1]:.0f}×{ext[2]:.0f}")
        if bad:
            mids = vco[uniq[counts != 2]].mean(axis=1)[:3]
            locs = " ".join(f"({m[0]:.1f},{m[1]:.1f},{m[2]:.1f})" for m in mids)
            msg += f"  ⚠ {bad} non-manifold edges near {locs}"
        log(msg)
    return obj


def _load_screen_cutout():
    """User-modelled screen cutter (screencutout.stl beside the addon): welded,
    scaled to mm, centred on its top-lip circle, top face at z=0, tab at +x.
    Returns (verts, faces, depth, max_radius, tab_angle) or None."""
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "screencutout.stl")
    if not os.path.exists(path):
        return None
    raw = np.fromfile(path, dtype=np.uint8)
    n = int(np.frombuffer(raw[80:84].tobytes(), dtype=np.uint32)[0])
    tri = raw[84:84 + n * 50].reshape(n, 50)
    v = tri[:, 12:48].copy().view(np.float32).reshape(n, 3, 3).astype(np.float64)
    if (v.reshape(-1, 3).max(axis=0) - v.reshape(-1, 3).min(axis=0)).max() < 1.0:
        v *= 1000.0                                      # exported in metres → mm
    flat = np.round(v.reshape(-1, 3), 4)
    uniq, inv = np.unique(flat, axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
                  & (faces[:, 0] != faces[:, 2])]
    verts = uniq.copy()
    top = verts[:, 2].max()
    ring = verts[verts[:, 2] > top - 0.12][:, :2]        # top lip outline ONLY (the next
    #                                                      vertex level holds the tab)
    A = np.column_stack([ring, np.ones(len(ring))])      # least-squares circle centre —
    b = (ring ** 2).sum(axis=1)                          # vertex-density independent
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    ctr = sol[:2] / 2.0
    verts -= [ctr[0], ctr[1], top]
    r = np.hypot(verts[:, 0], verts[:, 1])
    far = verts[r > r.max() - 2.0]
    tab0 = float(np.arctan2(far[:, 1].mean(), far[:, 0].mean())) if len(far) else 0.0
    depth = float(-verts[:, 2].min())
    low = verts[verts[:, 2] < -depth + 0.3]              # bottom outline: pocket Ø + tab
    rl = np.hypot(low[:, 0], low[:, 1])
    arc = rl[np.abs(rl - np.median(rl)) < 0.5]           # circle points, tab excluded
    r_pocket = float(arc.mean()) if len(arc) else 19.5
    return verts, faces.astype(np.int64), depth, float(r.max()), tab0, r_pocket


def build_shell_and_plate(p, log=print):
    """The engineering pass: hollow shell with rabbet lip, apertures, bosses —
    plus the matching bottom plate. Returns (shell_obj, plate_obj)."""
    t0 = time.time()
    geo = stage_geo(p, allow_slow=True)
    rd = stage_rd(p, geo, allow_slow=True) if coral_active(p) else None
    shape = stage_shape(p, geo)
    verts = stage_displace(p, geo, shape, rd, stage_tex(p, geo))
    faces = geo["faces"]
    wall, cut = float(p.wall), float(p.bottom_cut)
    recess, inset = float(p.plate_recess), float(p.plate_inset)
    zl = cut + recess                                    # ledge / plate-top plane

    # z-specific silhouette at the cut plane drives every bottom radius
    r_cut = silhouette_r(shape["verts"], cut)
    n_th = len(r_cut)
    th = np.linspace(0, 2 * pi, n_th, endpoint=False)
    rR = r_cut - inset                                   # recess / step wall

    boss_az = np.radians([p.boss_az, p.boss_az + 120, p.boss_az + 240])
    r_boss = np.interp(boss_az, th, rR, period=2 * pi) - p.boss_dia / 2.0 - 1.5
    rC = rR - wall - 0.5                                 # cavity core (ledge stays `wall` wide)

    # The cavity tool is assembled FIRST (offset skin ∪ bottom core, minus the
    # solid lip band with boss pads), then subtracted from the body in ONE pass.
    # Sequential unions near the underside kept breeding sliver geometry; one
    # coherent subtraction keeps every delicate intersection in a single solve.
    log("  shell: cavity tool…")
    ns = vertex_normals(verts, faces)
    for _ in range(3):                                   # smoothed normals cross less in grooves
        ns = (ns[geo["nbr"]].sum(axis=1) - geo["pad"][:, None] * ns) / geo["deg"][:, None]
    ns /= np.linalg.norm(ns, axis=1, keepdims=True)
    cav = _tmp_obj("omni_cavity", verts - ns * wall, faces)
    # the inward offset self-intersects wherever concave curvature is tighter
    # than the wall (rim shoulder, deep grooves) — voxel-remesh resolves that;
    # cavity precision is invisible, wall comes out `wall` ± half a voxel
    remesh_voxel(cav, 0.8)
    core = _tmp_obj("omni_core", *solid_polar_prism(rC, -10.0, zl + 10.0))
    bool_apply(cav, core, "UNION", label="cavity∪core", log=log)
    pad_bump = np.zeros(n_th)
    for a in boss_az:                                    # local inward lobes: solid pads
        dth = np.angle(np.exp(1j * (th - a)))            # under the screw bosses
        pad_bump = np.maximum(pad_bump, (p.boss_dia + 4.0) * np.exp(-(dth / 0.13) ** 2))
    big = float(np.max(r_cut)) + 60
    lip_band = _tmp_obj("omni_lip", *solid_annular_prism(rC + 0.5 - pad_bump, big,
                                                         cut - 20.0, zl + wall))
    bool_apply(cav, lip_band, "DIFFERENCE", label="cavity−lip", log=log)

    log("  shell: hollow + bottom…")
    shell = _tmp_obj("Omniphone Shell", verts, faces)
    bool_apply(shell, cav, "DIFFERENCE", label="hollow", log=log)
    box_v = np.array([[-big, -big, -20], [big, -big, -20], [big, big, -20], [-big, big, -20],
                      [-big, -big, cut], [big, -big, cut], [big, big, cut], [-big, big, cut]])
    box_f = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                      [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]], dtype=np.int64)
    bool_apply(shell, _tmp_obj("omni_cut", box_v, box_f), "DIFFERENCE", label="bottom cut", log=log)
    recess_tool = _tmp_obj("omni_recess", *solid_polar_prism(rR, cut - 4.0, zl))
    bool_apply(shell, recess_tool, "DIFFERENCE", label="recess", log=log)

    log("  shell: bosses + apertures…")
    facets = {f[0]: f for f in shape["facets"]}
    up = np.array([0.0, 0.0, 1.0])
    unions, holes = [], []
    prongs = []
    screen_stl_cut = None                                # cut LAST (post-remesh) so the
    #                                                      voxel pass can't soften the pocket
    scr_info = None
    if "Screen" in facets:
        # real 1.28″ pocket (per user mockup), outer surface inward: Ø36 bore
        # `screen_bore` deep (the glass stack fills it, touch face flush) → PCB
        # seat plane → Ø39 pocket + flat-edged tab corner (`tab_reach` across),
        # open into the cavity — the module drops in from inside. Heat-stake
        # prongs at the four square-tangent points fold over the PCB back.
        _, d, c0, tp, _ = facets["Screen"]
        u, v = _axes(d)
        e = lambda ang: u * cos(ang) + v * sin(ang)
        Rp = p.screen_pocket_dia / 2.0
        tab = radians(p.screen_tab_az)
        cutout = _load_screen_cutout()
        if cutout is not None:
            # user-modelled cutter: top face on the platform surface, centred on
            # the screen axis, tab rotated to screen_tab_az. +0.05 lift avoids a
            # coplanar top face; boss floor 0.5 above the cutter bottom so the
            # pocket opens into the cavity.
            cvv, cff, depth_c, rmax, tab0, r_pocket = cutout
            Rp = r_pocket                                # prongs hug the file's actual bore
            T = tab - tab0
            x = cvv[:, 0] * cos(T) - cvv[:, 1] * sin(T)
            y = cvv[:, 0] * sin(T) + cvv[:, 1] * cos(T)
            world = (c0[None, :] + u[None, :] * x[:, None] + v[None, :] * y[:, None]
                     + d[None, :] * (tp + 0.05 + cvv[:, 2])[:, None])
            t_floor = tp + 0.05 - depth_c + 0.5
            boss_r = rmax + 2.0
            scr_mode = "stl"
            unions.append(solid_frustum(c0, d, boss_r, boss_r, t_floor, tp - 1.0))
            screen_stl_cut = (world, cff)                # deferred until after the remesh
        else:
            t_seat = tp - p.screen_bore                  # PCB seat plane
            t_floor = t_seat - p.screen_pcb_depth        # boss inner face / prong base
            chord = Rp * cos(pi / 4)                     # square side touches here
            r_tab = max(p.screen_tab_reach - Rp, chord + 2.0)
            boss_r = r_tab + 2.0
            scr_mode = "param"
            unions.append(solid_frustum(c0, d, boss_r, boss_r, t_floor, tp - 1.0))
            holes.append(solid_frustum(c0, d, p.screen_ap_dia / 2.0, p.screen_ap_dia / 2.0,
                                       t_seat, tp + 60))                      # bore
            holes.append(solid_frustum(c0, d, Rp, Rp, t_floor - 8, t_seat))   # PCB pocket
            tab_depth = (t_seat - (t_floor - 8)) / 2.0   # corner: chord → flat edge
            holes.append(solid_obox(c0 + d * (t_seat - tab_depth) + e(tab) * ((chord - 0.8 + r_tab) / 2.0),
                                    e(tab), e(tab + pi / 2), d,
                                    (r_tab - chord + 0.8) / 2.0, 11.0, tab_depth))
        if boss_r + 1.0 > p.screen_dia / 2.0:
            log(f"  ⚠ platform Ø{p.screen_dia:.0f} too small for the pocket — "
                f"set Platform Ø ≥ {2 * (boss_r + 1):.0f}")
        scr_info = {"mode": scr_mode, "t_floor": float(t_floor), "boss_r": float(boss_r)}
        for k in range(4):                               # melt prongs past the pocket
            ang = tab + pi / 4 + k * pi / 2
            er = e(ang)
            pc = (c0 + er * (Rp + 1.1)
                  + d * (t_floor + (1.0 - p.screen_prong_len) / 2.0))
            prongs.append(solid_obox(pc, er, e(ang + pi / 2), d,
                                     1.0, p.screen_prong_w / 2.0,
                                     (1.0 + p.screen_prong_len) / 2.0))
    for name, hole in (("USB", p.usb_hole), ("Jack", p.jack_hole)):
        if name in facets:
            _, d, c0, tp, _ = facets[name]
            holes.append(solid_frustum(c0, d, hole / 2.0, hole / 2.0, tp - 40, tp + 40))
    for a, rb in zip(boss_az, r_boss):
        c = np.array([rb * cos(a), rb * sin(a), 0.0])
        # boss base 0.2 above the recess plane (embedded in its pad): the plate
        # clamps against the pad ring, so the gap is mechanically irrelevant
        unions.append(solid_frustum(c, up, p.boss_dia / 2.0, p.boss_dia / 2.0,
                                    zl + 0.2, zl + p.boss_h))
        holes.append(solid_frustum(c, up, p.pilot_dia / 2.0, p.pilot_dia / 2.0,
                                   cut - 2, zl + p.boss_h - 1.0))
    if p.pcb_mount_on:
        # spine + centre pillar, built in a local frame (sp along the spine, pp
        # across, up vertical), offset by (pcb_x, pcb_y). Clipped to the body so
        # anything past the coral wall is shaved — this is the trim you did by
        # hand. Union happens before the bosses; pilots drill with the other holes.
        log("  shell: PCB mount…")
        az = radians(p.pcb_az)
        sp = np.array([cos(az), sin(az), 0.0])
        ctr = np.array([p.pcb_x, p.pcb_y, 0.0])
        base = zl + float(p.pcb_plate_offset)
        seat = max(float(p.pcb_seat_z), base + float(p.pcb_spine_h) + 1.0)   # actual pillar top
        parts, lat = _pcb_mount_solids(p, zl)
        mount = _tmp_obj("omni_pcbmount", *merge_solids(parts))
        if lat:                                          # punch the spine lattice first
            bool_apply(mount, _tmp_obj("omni_pcblattice", *merge_solids(lat)),
                       "DIFFERENCE", label="pcb lattice", log=log)
        bool_apply(mount, _tmp_obj("omni_body_clip", verts, faces), "INTERSECT",
                   label="pcb clip", log=log)                             # shave protrusions
        bool_apply(shell, mount, "UNION", label="pcb mount", log=log)
        hs = p.pcb_hole_span / 2.0
        seats = [ctr + sp * hs, ctr - sp * hs] if p.pcb_hole_span > 0.1 else [ctr]
        for s in seats:
            holes.append(solid_frustum(s, up, p.pcb_pilot_dia / 2.0, p.pcb_pilot_dia / 2.0,
                                       seat - p.pcb_pilot_depth, seat + 2.0))
    if unions:
        bool_apply(shell, _tmp_obj("omni_unions", *merge_solids(unions)), "UNION",
                   label="bosses", log=log)
    for i, h in enumerate(holes):                        # one op per hole: several
        bool_apply(shell, _tmp_obj(f"omni_hole{i}", *h), "DIFFERENCE",  # overlap, and the
                   label=f"hole {i}", log=log)           # manifold solver wants
    #                                                      disjoint operands

    if p.print_remesh:
        # The manifold solver leaves self-intersecting / coincident faces along the
        # coral detail; slicers read those as inside-out patches and print the shell
        # hollow. A voxel remesh rebuilds it from a distance field — one watertight
        # manifold, no self-intersection possible. This is the actual hollow fix.
        log("  shell: print-clean remesh…")
        remesh_voxel(shell, float(p.print_voxel))
        log(f"    remeshed → {len(shell.data.polygons)} faces @ {p.print_voxel} mm voxel")

    # Screen pocket + prongs go in AFTER the remesh so the voxel pass never touches
    # them — the STL cutter fitted properly, so we want its crisp edges preserved.
    # Both are clean welded solids meeting a now-watertight shell, so the MANIFOLD
    # booleans weld cleanly without reintroducing the coral self-intersections.
    if screen_stl_cut is not None:
        log("  shell: screen pocket (post-remesh, crisp)…")
        bool_apply(shell, _tmp_obj("omni_screencut", *screen_stl_cut), "DIFFERENCE",
                   label="screen pocket", log=log)
    if prongs:                                           # last, so nothing shaves them
        bool_apply(shell, _tmp_obj("omni_prongs", *merge_solids(prongs)),
                   "UNION", label="screen prongs", log=log)

    bot = {"rR": np.round(rR, 3).tolist(),               # exact bottom profile: the
           "lip_inner": np.round(rC + 0.5 - pad_bump, 3).tolist(),  # plate rebuilds from
           "z_ledge": float(zl), "cut": float(cut),      # this, live, without the shell
           "boss_az_deg": [float(np.degrees(a)) for a in boss_az],
           "boss_r": [float(r) for r in r_boss]}
    shell["omniphone_bottom"] = json.dumps(bot)
    if scr_info is not None:
        shell["omniphone_screen"] = json.dumps(scr_info)
    plate = build_plate(p, bot, log=log)
    log(f"  build done in {time.time() - t0:.1f}s")
    return shell, plate


def _rot_axis(axis, ang):
    x, y, z = axis
    c, s = cos(ang), sin(ang)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _zbox(z0, z1, r=400.0):
    v = np.array([[-r, -r, z0], [r, -r, z0], [r, r, z0], [-r, r, z0],
                  [-r, -r, z1], [r, -r, z1], [r, r, z1], [-r, r, z1]])
    return v, _BOX_F.copy()


def _leg_cove(cx, cy, aa, z0, f, n=48, k=10):
    """Revolved concave fillet ring at a foot base: a closed profile (inside the leg
    → concave quarter-ellipse flare → out onto the plate face at z0) swept 360°.
    Overlaps both the leg and the plate so a plain UNION fairs the junction."""
    prof = [(aa - f, z0), (aa - f, z0 - f)]
    for i in range(1, k + 1):
        a = pi - (pi / 2.0) * (i / k)                    # concave arc, pi → pi/2
        prof.append((aa + f + 2.0 * f * cos(a), z0 - f + f * sin(a)))
    P = len(prof)
    th = np.linspace(0, 2 * pi, n, endpoint=False)
    verts = np.concatenate([np.column_stack([cx + r * np.cos(th), cy + r * np.sin(th),
                                             np.full(n, z)]) for r, z in prof])
    faces = []
    for j in range(P):
        j2 = (j + 1) % P
        for i in range(n):
            i2 = (i + 1) % n
            a_, b_, c_, d_ = j * n + i, j * n + i2, j2 * n + i2, j2 * n + i
            faces += [[a_, b_, c_], [a_, c_, d_]]
    return verts, np.array(faces, dtype=np.int64)


def build_plate(p, bot, log=print):
    """Bottom plate from the stored bottom profile: base disc + countersunk screw
    holes + key ridge, then the outfit — legs, speaker bay, transducer zone,
    PAM8403 recess, DPST switch. Rebuildable live, independent of the shell."""
    rR = np.asarray(bot["rR"], dtype=np.float64)
    lip_inner = np.asarray(bot["lip_inner"], dtype=np.float64)
    cut, zl = float(bot["cut"]), float(bot["z_ledge"])
    boss_az = np.radians(bot["boss_az_deg"])
    boss_r = bot["boss_r"]
    up = np.array([0.0, 0.0, 1.0])

    def flat_frame(az_deg, r_off, rot_deg):
        a = radians(az_deg)
        c = np.array([r_off * cos(a), r_off * sin(a), 0.0])
        rot = a + pi / 2 + radians(rot_deg)              # long axis tangential by default
        lu = np.array([cos(rot), sin(rot), 0.0])
        return c, lu, np.array([-lu[1], lu[0], 0.0])

    plate = _tmp_obj("Omniphone Plate", *solid_polar_prism(rR - p.plate_clear, cut, zl))
    holes, sinks = [], []
    for a, rb in zip(boss_az, boss_r):
        c = np.array([rb * cos(a), rb * sin(a), 0.0])
        holes.append(solid_frustum(c, up, 1.7, 1.7, cut - 2, zl + 2))
        sinks.append(solid_frustum(c, up, 3.3, 1.7, cut - 0.01, cut + 1.6))
    bool_apply(plate, _tmp_obj("omni_pholes", *merge_solids(holes)), "DIFFERENCE",
               label="plate holes", log=log)
    bool_apply(plate, _tmp_obj("omni_psinks", *merge_solids(sinks)), "DIFFERENCE",
               label="plate countersinks", log=log)
    if p.plate_ridge_h > 0.05:
        # key ridge: traces the lip/boss-pad profile minus clearance — the plate
        # can neither slide nor rotate
        ridge_out = lip_inner - p.plate_clear
        bool_apply(plate, _tmp_obj("omni_pridge", *solid_annular_prism(
            ridge_out - max(2.0, p.wall * 0.7), ridge_out,
            zl - 0.2, zl + p.plate_ridge_h)), "UNION", label="plate key ridge", log=log)

    # ---- cuts ----
    spk_ridge = []
    if p.spk_on:
        c, lu, lv = flat_frame(p.spk_az, p.spk_r, p.spk_rot)
        if p.spk_type == "ROUND":
            rr = p.spk_dia / 2.0 + 0.3                   # recess: speaker + fit
            bool_apply(plate, _tmp_obj("omni_spkrec", *solid_frustum(
                c, up, rr, rr, zl - 1.0, zl + 2)), "DIFFERENCE",
                label="speaker recess", log=log)
            ro = max(p.spk_dia / 2.0 - min(p.spk_lip, p.spk_dia / 4), 4.0)
            bool_apply(plate, _tmp_obj("omni_spkopen", *solid_frustum(
                c, up, ro, ro, cut - 2, zl + 2)), "DIFFERENCE",
                label="speaker opening", log=log)        # fires through the plate
            if p.spk_ridge_h > 0.05:                     # melt ring around the rim
                rv, rf = solid_annular_prism(np.full(96, rr + 0.2), rr + 0.2 + 1.2,
                                             zl - 0.1, zl + p.spk_ridge_h)
                spk_ridge.append((rv + c, rf))           # ring at the speaker, not origin
        else:
            bool_apply(plate, _tmp_obj("omni_spkrec", *solid_obox(
                c + up * (zl + 0.5), lu, lv, up, p.spk_w / 2 + 0.25, p.spk_l / 2 + 0.25,
                1.5)), "DIFFERENCE", label="speaker recess", log=log)  # 1 mm deep
            lip = min(p.spk_lip, p.spk_w / 2 - 2, p.spk_l / 2 - 2)
            bool_apply(plate, _tmp_obj("omni_spkopen", *solid_obox(
                c + up * (zl - 1), lu, lv, up, p.spk_w / 2 - lip, p.spk_l / 2 - lip,
                zl)), "DIFFERENCE", label="speaker opening", log=log)  # fires through
    if p.trans_on and p.trans_thin > 0.02:
        c, _, _ = flat_frame(p.trans_az, p.trans_r, 0)
        bool_apply(plate, _tmp_obj("omni_trrec", *solid_frustum(
            c, up, p.trans_dia / 2 + 4, p.trans_dia / 2 + 4,
            zl - p.trans_thin, zl + 2)), "DIFFERENCE", label="transducer zone", log=log)
    pam_posts, sw_posts = [], []
    if p.pam_on:
        c, lu, lv = flat_frame(p.pam_az, p.pam_r, p.pam_rot)
        hw, hl = p.pam_w / 2 + 0.2, p.pam_l / 2 + 0.2
        bool_apply(plate, _tmp_obj("omni_pamrec", *solid_obox(
            c + up * (zl + 0.5), lu, lv, up, hw, hl, 1.5)), "DIFFERENCE",
            label="PAM8403 recess", log=log)             # 1 mm deep from top
        for axis, h in ((lu, hw), (lv, hl)):             # melt posts at side midpoints
            for s in (-1, 1):
                pc = c + axis * (s * (h + 0.9))
                pam_posts.append(solid_frustum(pc, up, 0.8, 0.8, zl - 1.0, zl + 1.6))
    if p.sw_on:
        c, lu, lv = flat_frame(p.sw_az, p.sw_r, p.sw_rot)
        bool_apply(plate, _tmp_obj("omni_swhole", *solid_obox(
            c + up * (cut + 1), lu, lv, up, 11.0 / 2, 5.8 / 2, zl)), "DIFFERENCE",
            label="switch hole", log=log)                # 5.8×11 through
        for s in (-1, 1):                                # prong tubes, 15 mm apart on
            pc = c + lu * (s * 7.5)                      # the long axis
            sw_posts.append(solid_frustum(pc, up, p.sw_prong_d / 2, p.sw_prong_d / 2,
                                          zl - 0.5, zl + 2.0))

    # ---- unions ----
    corral = []
    if p.bat_on:
        c, lu, lv = flat_frame(p.bat_az, p.bat_r, p.bat_rot)
        wt, gap = 1.6, 14.0                              # wall thickness, side-mid gaps
        zc = zl + p.bat_wall_h / 2 - 0.1                 # embedded 0.1 into the top
        hh = p.bat_wall_h / 2 + 0.1
        for axis, other, half_a, half_o in ((lu, lv, p.bat_w / 2, p.bat_l / 2),
                                            (lv, lu, p.bat_l / 2, p.bat_w / 2)):
            seg = (half_a - gap / 2) / 2                 # wall piece between corner and gap
            if seg < 2.0:
                continue
            for sa in (-1, 1):
                for so in (-1, 1):
                    cc_ = (c + up * zc
                           + axis * (sa * (gap / 2 + seg))
                           + other * (so * (half_o + 0.2 + wt / 2)))
                    corral.append(solid_obox(cc_, axis, other, up, seg, wt / 2, hh))
    if pam_posts or sw_posts or corral or spk_ridge:
        bool_apply(plate, _tmp_obj("omni_posts",
                                   *merge_solids(pam_posts + sw_posts + corral + spk_ridge)),
                   "UNION", label="melt posts + rings", log=log)
    if p.leg_count > 0:
        a_dia = max(p.leg_dia, p.leg_pad_dia + 1.0)
        aa = a_dia / 2.0
        frac = min(p.leg_pad_dia / a_dia, 0.95)
        cc = p.leg_h / sqrt(1.0 - frac * frac)           # ellipsoid depth so the flat
        sph_v, sph_f = make_icosphere(4)                 # foot comes out leg_pad_dia
        legs = []
        for k in range(int(p.leg_count)):
            ang = radians(p.leg_az0) + k * 2 * pi / int(p.leg_count)
            tang = np.array([-sin(ang), cos(ang), 0.0])
            R = _rot_axis(tang, -radians(p.leg_angle))   # foot swings outward
            v = (sph_v * np.array([aa, aa, cc])) @ R.T
            v += np.array([p.leg_r * cos(ang), p.leg_r * sin(ang), cut])
            legs.append((v, sph_f.copy()))
        legs_obj = _tmp_obj("omni_legs", *merge_solids(legs))
        bool_apply(legs_obj, _tmp_obj("omni_legtrim", *_zbox(cut + 0.8, cut + 60)),
                   "DIFFERENCE", label="leg trim", log=log)
        bool_apply(plate, legs_obj, "UNION", label="legs", log=log)
        if p.leg_fillet > 0.01:                          # fair each foot into the plate
            ff = min(float(p.leg_fillet), p.leg_h - 1.0, aa - 1.0)
            if ff > 0.05:
                coves = []
                for k in range(int(p.leg_count)):
                    ang = radians(p.leg_az0) + k * 2 * pi / int(p.leg_count)
                    coves.append(_leg_cove(p.leg_r * cos(ang), p.leg_r * sin(ang),
                                           aa, cut, ff))
                bool_apply(plate, _tmp_obj("omni_legfillet", *merge_solids(coves)),
                           "UNION", label="foot fillets", log=log)
        bool_apply(plate, _tmp_obj("omni_feet", *_zbox(cut - p.leg_h - 60, cut - p.leg_h)),
                   "DIFFERENCE", label="flat feet", log=log)
    return plate


# ---------------------------------------------------------------------------
# live-update machinery.
# Fast stages rebuild right inside the property callback (tens of ms).
# Slow changes (RD / topology) start a progressive *growth job*: the sim runs
# in ~0.12 s chunks on a timer and each chunk is written to the mesh, so the
# coral visibly grows in the viewport and settles at the exact final result.
# ---------------------------------------------------------------------------

_growth = None            # {"key", "st", "not_before"} or None


def _publish_growth(scene, g, done):
    # partial fields go into the cache under a sentinel key (so nothing treats
    # them as final); the last chunk publishes under the true key
    key = g["key"] if done else ("growing",) + g["key"]
    _cache["rd"] = (key, {"field": g["st"]["V"], "centers": g["st"]["centers"]})
    rebuild(scene, allow_slow=False)


def _growth_tick():
    global _growth
    g = _growth
    if g is None:
        return None
    if time.time() < g["not_before"]:                    # debounce while dragging
        return 0.05
    scene = bpy.context.scene
    if scene is None or not hasattr(scene, "omni_sculpt"):
        return 0.1
    p = scene.omni_sculpt
    if _key_rd(p) != g["key"]:                           # superseded mid-flight
        _growth = None
        return None
    try:
        if g["st"] is None:
            g["st"] = rd_setup(p, stage_geo(p, allow_slow=True))
        t0 = time.time()
        done = False
        while not done and time.time() - t0 < 0.12:
            done = rd_run(g["st"], 120)
        # live_growth: write every chunk (watch it grow); otherwise stay quiet
        # and write once at the end — same background chunking either way
        if done or p.live_growth:
            _publish_growth(scene, g, done)
    except Exception as exc:
        print("omniphone: growth failed:", exc)
        _growth = None
        return None
    if done:
        _growth = None
        return None
    return 0.01



def _schedule_slow(p):
    global _growth
    if not coral_active(p):
        _growth = None                                   # nothing to grow (and cancel any in-flight)
        return
    key = _key_rd(p)
    if _growth is not None and _growth["key"] == key:
        return                                           # already growing there
    _growth = {"key": key, "st": None, "not_before": time.time() + 0.25}
    if bpy.app.background:
        return                                           # selftest flushes manually
    if not bpy.app.timers.is_registered(_growth_tick):
        bpy.app.timers.register(_growth_tick, first_interval=0.05)


def _flush_pending():
    """Synchronously finish any scheduled growth (headless / selftest path)."""
    global _growth
    g = _growth
    if g is None:
        return
    scene = bpy.context.scene
    p = scene.omni_sculpt
    g["key"] = _key_rd(p)
    if g["st"] is None:
        g["st"] = rd_setup(p, stage_geo(p, allow_slow=True))
    rd_run(g["st"], 10 ** 9)
    _publish_growth(scene, g, True)
    _growth = None


def _mk_update(fast=False, slow=False):
    def cb(self, context):
        if _suspend or not self.auto_update:
            return
        if bpy.data.objects.get(BODY_NAME) is None and not slow:
            return                                       # nothing generated yet
        try:
            if slow and not coral_active(self):
                _schedule_slow(self)                     # cancels any in-flight growth
                rebuild(context.scene, allow_slow=True)  # no RD to grow — rest is quick
                return
            if fast:
                rebuild(context.scene, allow_slow=False)
            if slow:
                _schedule_slow(self)
        except Exception as exc:
            print("omniphone: live update failed:", exc)
    return cb


def _upd_auto(self, context):
    if self.auto_update and not _suspend:
        try:
            rebuild(context.scene, allow_slow=True)
        except Exception as exc:
            print("omniphone: sync on auto-update failed:", exc)


UPD_FAST = _mk_update(fast=True)                 # shape / displacement: instant
UPD_SLOW = _mk_update(slow=True)                 # RD / topology: grows live
UPD_BOTH = _mk_update(fast=True, slow=True)      # facet moves now, pattern follows


def _upd_cutplane(self, context):
    if _suspend:
        return
    try:
        cutplane_sync(context.scene)
    except Exception as exc:
        print("omniphone: cut plane sync failed:", exc)


def _upd_pcb(self, context):
    if _suspend:
        return
    try:
        pcb_preview_sync(context.scene)
    except Exception as exc:
        print("omniphone: pcb preview sync failed:", exc)


def _upd_plate(self, context):
    """Plate-only live rebuild — works as soon as a shell (with its stored
    bottom profile) exists; ~1 s, no shell rebuild."""
    if _suspend:
        return
    shell = bpy.data.objects.get("Omniphone Shell")
    if shell is None or "omniphone_bottom" not in shell:
        return
    try:
        t0 = time.time()
        rebuild_plate(context.scene)
        print(f"omniphone: plate rebuilt in {time.time() - t0:.1f}s")
    except Exception as exc:
        print("omniphone: plate update failed:", exc)


def rebuild_plate(scene):
    p = scene.omni_sculpt
    shell = bpy.data.objects["Omniphone Shell"]
    bot = json.loads(shell["omniphone_bottom"])
    old = bpy.data.objects.get("Omniphone Plate")
    if old is not None:
        _kill_obj(old)
    plate = build_plate(p, bot, log=lambda *a: None)
    coll = ensure_collection(COLL_NAME)
    for c in list(plate.users_collection):
        c.objects.unlink(plate)
    coll.objects.link(plate)
    return plate


# ---------------------------------------------------------------------------
# properties / operators / panels
# ---------------------------------------------------------------------------

FK_HINT = "Classic pairs — coral 0.055/0.062 · spots 0.030/0.062 · maze 0.029/0.057 · mitosis 0.0367/0.0649"


class OmniSculptProps(bpy.types.PropertyGroup):
    live_growth: BoolProperty(name="Grow in viewport", default=False,
                              description="Show the coral simulation growing (~5 fps) when a "
                                          "pattern parameter changes. Off = compute in the "
                                          "background and show only the final shape (the UI "
                                          "stays responsive either way)")
    auto_update: BoolProperty(name="Auto update", default=True, update=_upd_auto,
                              description="Rebuild live as parameters change. Shape sliders are "
                                          "instant; pad/coral changes make the pattern re-grow "
                                          "before your eyes and settle in a couple of seconds")
    # 1 · ball
    diameter: FloatProperty(name="Diameter", default=220.0, min=60.0, max=320.0,
                            update=UPD_BOTH, description="Ball diameter (mm)")
    squish_height: FloatProperty(name="Squish to", default=120.0, min=40.0, max=280.0,
                                 update=UPD_FAST, description="Height after squashing the ball (mm)")
    resolution: IntProperty(name="Mesh detail", default=7, min=5, max=8, update=UPD_SLOW,
                            description="Icosphere subdivisions: 5 ≈ 2.6k verts, 6 ≈ 10k, "
                                        "7 ≈ 41k (~1.8 mm edges), 8 ≈ 164k (4× slower, final export)")
    body_algo: EnumProperty(name="Form", default="BALL", update=UPD_BOTH, items=[
        ("BALL", "Squished ball", "Plain canvas — the texture pass and the coral "
                                  "do the talking"),
        ("VORONOI", "Puffed Voronoi cells", "One puffy cell per pad — pads ARE the "
                                            "cells (the Even layout gives equal-size "
                                            "pads)"),
        ("CRYSTAL", "Crystal / faceted", "Wulff-style faceted solid — every pad "
                                         "becomes a flat facet (Even layout = the "
                                         "cleanest crystal)"),
        ("CYMATICS", "Cymatics dome", "Bessel standing-wave ripples, like a vibrating "
                                      "plate mode. Pads don't shape this one"),
        ("MANDELBULB", "Mandelbulb", "Low-iteration fractal blob (~1–2 s per slider "
                                     "change). Pads don't shape this one")])
    coral_on: BoolProperty(name="Brain coral", default=True, update=UPD_BOTH,
                           description="Grow the Gray-Scott coral relief over the "
                                       "textured form — pads are its seeds")
    # texture pass — a second algorithm displaced along the shaped surface,
    # with its own feature count/seed (decoration; pads belong to the form)
    tex_algo: EnumProperty(name="Texture", default="NONE", update=UPD_FAST, items=[
        ("NONE", "None", "No second-pass texture"),
        ("VORONOI", "Voronoi cells", "Small puffed cells over the form"),
        ("CRYSTAL", "Crystal facets", "Gem grinder — the crest inside each texture "
                                      "cell is cut flat: sharp planar chips"),
        ("CYMATICS", "Cymatics ripple", "Standing-wave ripple over the form"),
        ("MANDELBULB", "Mandelbulb crust", "Fractal tier relief (~1 s to compute, "
                                           "then cached)")])
    tex_amount: FloatProperty(name="Amount", default=3.5, min=-12.0, max=12.0,
                              update=UPD_FAST,
                              description="Texture relief height (mm); negative flips "
                                          "it inward")
    tex_count: IntProperty(name="Features", default=48, min=4, max=400, update=UPD_FAST,
                           description="Cells / facets in the texture (Voronoi & "
                                       "Crystal only)")
    tex_seed: IntProperty(name="Seed", default=0, min=0, max=99999, update=UPD_FAST,
                          description="Rotates and re-rolls the texture independently "
                                      "of the pad seed")
    tex_jitter: FloatProperty(name="Irregularity", default=0.35, min=0.0, max=1.0,
                              subtype="FACTOR", update=UPD_FAST,
                              description="Voronoi: uneven cell sizes · Crystal: uneven "
                                          "carve depths")
    tex_groove: FloatProperty(name="Groove width", default=0.10, min=0.02, max=0.4,
                              precision=3, update=UPD_FAST,
                              description="Valley width between texture cells, as a "
                                          "fraction of the cell size")
    tex_cy_m: IntProperty(name="Angular m", default=8, min=0, max=16, update=UPD_FAST)
    tex_cy_n: IntProperty(name="Radial n", default=5, min=1, max=9, update=UPD_FAST)
    tex_mb_power: IntProperty(name="Power", default=8, min=3, max=10, update=UPD_FAST)
    tex_mb_iter: IntProperty(name="Iterations", default=8, min=3, max=12, update=UPD_FAST)
    cell_amp: FloatProperty(name="Cell puff", default=12.0, min=2.0, max=26.0, update=UPD_FAST,
                            description="How far each cell bulges out (mm)")
    cell_groove: FloatProperty(name="Groove width", default=0.13, min=0.02, max=0.4,
                               precision=3, update=UPD_FAST,
                               description="Width of the valleys between cells")
    facet_dist: FloatProperty(name="Facet dist", default=0.85, min=0.5, max=1.1, update=UPD_FAST,
                              description="Facet plane distance (fraction of radius) — "
                                          "lower = more truncated")
    facet_jitter: FloatProperty(name="Jitter", default=0.09, min=0.0, max=0.4, update=UPD_FAST,
                                description="Random per-facet distance variation")
    cy_m: IntProperty(name="Angular m", default=5, min=0, max=10, update=UPD_FAST,
                      description="Angular mode count (lobes around the axis)")
    cy_n: IntProperty(name="Radial n", default=3, min=1, max=6, update=UPD_FAST,
                      description="Radial mode count (rings from pole to equator)")
    cy_amp: FloatProperty(name="Ripple height", default=12.0, min=2.0, max=30.0, update=UPD_FAST)
    mb_power: IntProperty(name="Power", default=6, min=3, max=10, update=UPD_FAST)
    mb_iter: IntProperty(name="Iterations", default=6, min=3, max=12, update=UPD_FAST,
                         description="Low = smooth simplified blob, high = fractal crust")
    mb_level: FloatProperty(name="Inflate", default=4.0, min=0.0, max=14.0, update=UPD_FAST,
                            description="Fattens the surface (mm, pre-normalisation)")
    # 2 · bottom flatten
    final_height: FloatProperty(name="Final height", default=100.0, min=30.0, max=280.0,
                                update=UPD_FAST,
                                description="Total height after gradient-flattening the underside (mm)")
    flatten_zone: FloatProperty(name="Blend zone", default=30.0, min=5.0, max=80.0,
                                update=UPD_FAST,
                                description="How far above the floor the flattening blends out (mm)")
    # 2 · pads & layout (UPD_BOTH: Voronoi/Crystal bodies read these on the
    # fast path; the coral re-grows in the background when it's active)
    pad_count: IntProperty(name="Pad count", default=12, min=0, max=96, update=UPD_BOTH,
                           description="Number of touch pads (coral seeds / Voronoi "
                                       "cells / crystal facets)")
    pad_layout: EnumProperty(name="Layout", default="PHYLLO", update=UPD_BOTH, items=[
        ("PHYLLO", "Phyllotaxis", "Golden-angle spiral over the band"),
        ("RANDOM", "Random", "Uniform random over the band"),
        ("LLOYD", "Lloyd (relaxed)", "Random start + repulsion relaxation "
                                     "(uses at least 15 relax iterations)"),
        ("EVEN", "Even (max spacing)", "Phyllotaxis start relaxed to convergence — "
                                       "the tidiest possible distribution, most-equal "
                                       "distances → pads come out the same size")])
    relax_iters: IntProperty(name="Relax", default=0, min=0, max=60, update=UPD_BOTH,
                             description="Repulsion-relaxation iterations applied to the layout")
    band_lo: FloatProperty(name="Band low", default=0.05, min=-0.6, max=0.9, update=UPD_BOTH,
                           description="Lowest pad position (z on the unit ball: 1 = top pole, 0 = equator)")
    band_hi: FloatProperty(name="Band high", default=0.92, min=0.0, max=1.0, update=UPD_BOTH,
                           description="Highest pad position (keep < 1.0 to leave the crown free)")
    seed_bulb_r: FloatProperty(name="Seed bulb Ø/2", default=5.0, min=2.0, max=15.0, update=UPD_BOTH,
                               description="Radius of the RD seed disc at each pad centre (mm)")
    rng_seed: IntProperty(name="Random seed", default=3, min=0, max=99999, update=UPD_BOTH,
                          description="Every seed is a different instrument")
    # 3 · brain coral (the CORAL algorithm, or the overlay on any other body)
    rd_feed: FloatProperty(name="Feed F", default=0.055, min=0.02, max=0.09,
                           precision=4, step=0.05, update=UPD_SLOW, description=FK_HINT)
    rd_kill: FloatProperty(name="Kill k", default=0.062, min=0.03, max=0.075,
                           precision=4, step=0.05, update=UPD_SLOW, description=FK_HINT)
    rd_steps: IntProperty(name="Sim steps", default=2600, min=200, max=6000, update=UPD_SLOW,
                          description="More steps = pattern spreads further from the seeds and matures")
    rd_scale: FloatProperty(name="Pattern scale", default=0.75, min=0.5, max=2.5, update=UPD_SLOW,
                            description="Ridge wavelength multiplier (cost grows with scale²)")
    rd_noise: FloatProperty(name="Background noise", default=0.0, min=0.0, max=0.1,
                            precision=3, step=0.5, update=UPD_SLOW,
                            description="0 = pattern grows only from the pad seeds; "
                                        ">0 = spontaneous coral everywhere")
    field_smooth: IntProperty(name="Field smoothing", default=1, min=0, max=10, update=UPD_FAST,
                              description="Graph-smoothing passes on the RD field before displacing "
                                          "(more = softer, meltier ridges)")
    ridge_height: FloatProperty(name="Ridge height", default=3.5, min=-15.0, max=15.0, update=UPD_FAST,
                                description="How far ridges stick out of the ball (mm); "
                                            "negative dents them in")
    groove_depth: FloatProperty(name="Groove depth", default=3.5, min=-15.0, max=15.0, update=UPD_FAST,
                                description="How deep grooves cut into the ball (mm); "
                                            "negative bulges them out")
    coverage: FloatProperty(name="Coverage", default=1.0, min=0.05, max=1.0, subtype="FACTOR",
                            update=UPD_FAST,
                            description="Fraction of the body height (from the top) the coral covers — "
                                        "0.2 = only the top 20% is brain")
    cover_fade: FloatProperty(name="Coverage fade", default=12.0, min=2.0, max=40.0, update=UPD_FAST,
                              description="Width of the smooth fade at the coverage line (mm)")
    # 5 · screen platform (1.28″ round: module Ø37.5, view Ø32.4, seat Ø38.5)
    screen_on: BoolProperty(name="Screen platform", default=True, update=UPD_BOTH)
    screen_tilt: FloatProperty(name="Tilt", default=0.0, min=0.0, max=80.0, update=UPD_BOTH,
                               description="Degrees from straight up")
    screen_az: FloatProperty(name="Azimuth", default=0.0, min=0.0, max=360.0, update=UPD_BOTH,
                             description="Compass direction of the tilt")
    screen_pitch: FloatProperty(name="Pitch", default=0.0, min=-60.0, max=60.0, update=UPD_FAST,
                                description="Face angle relative to the smooth body surface: "
                                            "0° = lies tangent (like a coin on the ball), "
                                            "+ pitches the face up toward the crown, − down")
    screen_dia: FloatProperty(name="Platform Ø", default=56.0, min=34.0, max=70.0, update=UPD_BOTH,
                              description="Flat platform diameter (module seat Ø38.5 + margin; "
                                          "aperture is cut in the shell slice)")
    screen_height: FloatProperty(name="Height", default=0.0, min=-15.0, max=15.0, update=UPD_FAST,
                                 description="Push the platform out (+) or sink it in (−) along its axis (mm)")
    screen_merge: FloatProperty(name="Merge reach", default=14.0, min=0.0, max=50.0, update=UPD_FAST,
                                description="How far around the platform vertices are attracted (mm)")
    screen_strength: FloatProperty(name="Merge strength", default=0.7, min=0.0, max=1.0,
                                   subtype="FACTOR", update=UPD_FAST,
                                   description="How strongly the surroundings are pulled toward the "
                                               "platform plane (the seat itself is always fully flat)")
    # 6 · panel-mount ports
    ports_coplanar: BoolProperty(name="USB + Jack coplanar", default=False, update=UPD_BOTH,
                                 description="One shared flat panel holds BOTH connectors so their "
                                             "cables exit exactly parallel. Placed/faced by the "
                                             "shared controls below; the two individual "
                                             "position/angle settings are replaced by one Gap")
    ports_az: FloatProperty(name="Azimuth", default=180.0, min=0.0, max=360.0, update=UPD_BOTH,
                            description="Direction the shared USB+Jack panel faces")
    ports_z: FloatProperty(name="Position", default=22.0, min=4.0, max=60.0, update=UPD_BOTH,
                           description="Centre height of the shared panel above the floor (mm)")
    ports_pitch: FloatProperty(name="Pitch", default=0.0, min=-45.0, max=45.0, update=UPD_BOTH,
                               description="Absolute: 0° = panel perpendicular to the ground; "
                                           "+ tilts up, − down")
    ports_height: FloatProperty(name="Height", default=0.0, min=-15.0, max=15.0, update=UPD_BOTH,
                                description="Proud (+) / recessed (−) offset of the panel along "
                                            "its normal")
    ports_gap: FloatProperty(name="Gap", default=30.0, min=8.0, max=90.0, update=UPD_BOTH,
                             description="Centre-to-centre distance between the USB and jack flats "
                                         "(two separate seats on one shared plane), mm")
    ports_merge: FloatProperty(name="Merge reach", default=8.0, min=0.0, max=30.0, update=UPD_BOTH,
                               description="Falloff width around each shared-panel flat (mm)")
    ports_strength: FloatProperty(name="Merge strength", default=1.0, min=0.0, max=1.0,
                                  update=UPD_BOTH, subtype="FACTOR",
                                  description="How hard verts outside the disc are pulled toward "
                                              "the shared plane (0 = only the disc flattens)")
    usb_on: BoolProperty(name="USB flat", default=True, update=UPD_FAST)
    usb_az: FloatProperty(name="Azimuth", default=180.0, min=0.0, max=360.0, update=UPD_FAST)
    usb_z: FloatProperty(name="Position", default=22.0, min=4.0, max=60.0, update=UPD_FAST,
                         description="Centre height of the USB flat above the floor (mm). "
                                     "Keep the disc above the rounded floor rim: "
                                     "position − Ø/2 ≳ 8 mm")
    usb_height: FloatProperty(name="Height", default=0.0, min=-15.0, max=15.0, update=UPD_FAST,
                              description="Proud (+) / recessed (−) offset of the flat along its "
                                          "normal — the same 'Height' the screen platform has")
    usb_dia: FloatProperty(name="Flat Ø", default=26.0, min=12.0, max=44.0, update=UPD_FAST,
                           description="Flat spot for a panel-mount USB-C (flange + nut clearance)")
    usb_merge: FloatProperty(name="Merge reach", default=8.0, min=0.0, max=30.0, update=UPD_FAST)
    usb_pitch: FloatProperty(name="Pitch", default=0.0, min=-45.0, max=45.0, update=UPD_FAST,
                             description="Absolute: 0° = face perpendicular to the ground; "
                                         "+ tilts the face up, − down")
    jack_on: BoolProperty(name="Jack flat", default=True, update=UPD_FAST)
    jack_az: FloatProperty(name="Azimuth", default=210.0, min=0.0, max=360.0, update=UPD_FAST)
    jack_z: FloatProperty(name="Position", default=22.0, min=4.0, max=60.0, update=UPD_FAST,
                          description="Centre height of the jack flat above the floor (mm)")
    jack_height: FloatProperty(name="Height", default=0.0, min=-15.0, max=15.0, update=UPD_FAST,
                               description="Proud (+) / recessed (−) offset along the normal "
                                           "(ignored when coplanar — the jack takes the USB plane)")
    jack_dia: FloatProperty(name="Flat Ø", default=18.0, min=10.0, max=36.0, update=UPD_FAST,
                            description="Flat spot for a panel-mount 6.3/3.5 mm jack")
    jack_merge: FloatProperty(name="Merge reach", default=8.0, min=0.0, max=30.0, update=UPD_FAST)
    jack_pitch: FloatProperty(name="Pitch", default=0.0, min=-45.0, max=45.0, update=UPD_FAST,
                              description="Absolute: 0° = face perpendicular to the ground; "
                                          "+ tilts the face up, − down")
    # 7 · shell & plate (build step — no live update, press Build)
    wall: FloatProperty(name="Wall", default=3.0, min=1.5, max=6.0,
                        description="Shell wall thickness (mm)")
    bottom_cut: FloatProperty(name="Bottom cut", default=3.0, min=0.5, max=120.0,
                              soft_max=40.0, update=_upd_cutplane,
                              description="Flat-cut height in mm above the resting plane "
                                          "(z=0, where the flattened body touches the table); "
                                          "everything below is removed. The wire circle in "
                                          "the viewport shows where it will land")
    plate_recess: FloatProperty(name="Plate recess", default=2.0, min=1.0, max=5.0,
                                description="How far the bottom plate sits pushed in — plate thickness (mm)")
    seam_smooth: FloatProperty(name="Seam smoothing", default=6.0, min=0.0, max=25.0,
                               update=UPD_FAST,
                               description="Height above the plate ledge over which coral/texture "
                                           "fades in (mm). Keeps the plate-mating rim smooth so the "
                                           "cut seats flat — 0 lets coral run right down to the seam")
    plate_inset: FloatProperty(name="Plate inset", default=4.0, min=2.5, max=12.0,
                               description="How far the plate outline shrinks inside the footprint (mm)")
    plate_clear: FloatProperty(name="Plate clearance", default=0.25, min=0.05, max=0.8,
                               update=_upd_plate,
                               description="Radial print-fit clearance between plate and recess (mm)")
    plate_ridge_h: FloatProperty(name="Key ridge", default=2.5, min=0.0, max=8.0,
                                 update=_upd_plate,
                                 description="Standing ridge on the plate that keys into the "
                                             "shell's bottom opening (follows the boss-pad "
                                             "profile, so it locks rotation too). 0 = off")
    # plate outfit — all live once a shell exists (plate-only rebuild, ~1 s)
    leg_count: IntProperty(name="Legs", default=4, min=0, max=8, update=_upd_plate,
                           description="Smooth bump feet on the plate underside")
    leg_dia: FloatProperty(name="Leg Ø", default=18.0, min=8.0, max=40.0, update=_upd_plate,
                           description="Bump thickness at the plate")
    leg_h: FloatProperty(name="Leg height", default=14.0, min=4.0, max=30.0, update=_upd_plate,
                         description="Foot protrusion below the plate. Acoustics: keep ≥ the "
                                     "bottom port Ø (breathing gap rule) — ≥15 for a Ø15 port; "
                                     "8–10 is enough if the bottom only decouples")
    leg_pad_dia: FloatProperty(name="Foot pad Ø", default=10.0, min=4.0, max=20.0,
                               update=_upd_plate,
                               description="Flat cut at the foot tip, sized for a stick-on "
                                           "rubber/felt pad")
    leg_angle: FloatProperty(name="Leg splay", default=10.0, min=0.0, max=30.0,
                             update=_upd_plate, description="Outward tilt of each bump")
    leg_r: FloatProperty(name="Leg radius", default=55.0, min=20.0, max=95.0,
                         update=_upd_plate, description="Distance from the plate centre")
    leg_az0: FloatProperty(name="Leg angle 0", default=45.0, min=0.0, max=360.0,
                           update=_upd_plate, description="Rotation of the whole leg ring")
    leg_fillet: FloatProperty(name="Foot fillet", default=0.0, min=0.0, max=8.0,
                              update=_upd_plate,
                              description="Concave fillet blending each foot into the plate "
                                          "underside — smoother look, stronger junction, "
                                          "less stress riser. 0 = off (hard bump)")
    spk_on: BoolProperty(name="Speaker bay", default=True, update=_upd_plate)
    spk_type: EnumProperty(name="Type", default="ROUND", update=_upd_plate, items=[
        ("ROUND", "Round speaker", "Small circular speaker in a recess, locked by a "
                                   "meltable ridge around its rim"),
        ("RECT", "Rectangular box", "Enclosed box (e.g. Waveshare 5W 100×45×21) resting "
                                    "on a rim over a through-opening")])
    spk_dia: FloatProperty(name="Speaker Ø", default=40.0, min=15.0, max=80.0,
                           update=_upd_plate, description="Round speaker outer diameter")
    spk_ridge_h: FloatProperty(name="Melt ridge", default=2.5, min=0.0, max=6.0,
                               update=_upd_plate,
                               description="Standing ring around the round speaker's rim — "
                                           "melt it inward to lock the speaker in place")
    spk_az: FloatProperty(name="Azimuth", default=90.0, min=0.0, max=360.0, update=_upd_plate)
    spk_r: FloatProperty(name="Offset", default=30.0, min=0.0, max=60.0, update=_upd_plate)
    spk_rot: FloatProperty(name="Rotate", default=0.0, min=-90.0, max=90.0, update=_upd_plate)
    spk_w: FloatProperty(name="Bay width", default=100.5, min=20.0, max=120.0,
                         update=_upd_plate, description="Waveshare 5W box is 100×45×21")
    spk_l: FloatProperty(name="Bay length", default=45.5, min=15.0, max=80.0, update=_upd_plate)
    spk_lip: FloatProperty(name="Bay lip", default=6.0, min=3.0, max=15.0, update=_upd_plate,
                           description="Rim the speaker rests on; the rest opens through "
                                       "the plate (it fires downward)")
    trans_on: BoolProperty(name="Transducer zone", default=True, update=_upd_plate)
    trans_az: FloatProperty(name="Azimuth", default=270.0, min=0.0, max=360.0, update=_upd_plate)
    trans_r: FloatProperty(name="Offset", default=26.0, min=0.0, max=60.0, update=_upd_plate,
                           description="Keep it off-centre (~1/3–2/5 of the plate radius): "
                                       "centre placement sits on every mode node")
    trans_dia: FloatProperty(name="Transducer Ø", default=35.0, min=20.0, max=60.0,
                             update=_upd_plate)
    trans_thin: FloatProperty(name="Thinning", default=0.6, min=0.0, max=1.5,
                              update=_upd_plate,
                              description="Local plate thinning under the transducer glue "
                                          "zone: more = louder soundboard, weaker tactile "
                                          "coupling. 0 = off")
    pam_on: BoolProperty(name="PAM8403 mount", default=True, update=_upd_plate)
    pam_az: FloatProperty(name="Azimuth", default=180.0, min=0.0, max=360.0, update=_upd_plate)
    pam_r: FloatProperty(name="Offset", default=48.0, min=0.0, max=65.0, update=_upd_plate)
    pam_rot: FloatProperty(name="Rotate", default=0.0, min=-90.0, max=90.0, update=_upd_plate)
    pam_w: FloatProperty(name="Board W", default=21.5, min=15.0, max=30.0, update=_upd_plate)
    pam_l: FloatProperty(name="Board L", default=18.5, min=12.0, max=30.0, update=_upd_plate)
    sw_on: BoolProperty(name="DPST switch", default=True, update=_upd_plate)
    sw_az: FloatProperty(name="Azimuth", default=0.0, min=0.0, max=360.0, update=_upd_plate)
    sw_r: FloatProperty(name="Offset", default=48.0, min=0.0, max=65.0, update=_upd_plate)
    sw_rot: FloatProperty(name="Rotate", default=0.0, min=-90.0, max=90.0, update=_upd_plate)
    sw_prong_d: FloatProperty(name="Prong Ø", default=1.0, min=0.8, max=2.5,
                              update=_upd_plate,
                              description="Melt-rivet posts through the switch flange holes, "
                                          "15 mm apart on the long axis")
    bat_on: BoolProperty(name="Battery corral", default=False, update=_upd_plate,
                         description="Low perimeter wall around the battery footprint, with "
                                     "gaps at the side midpoints for a strap/tape and for "
                                     "lifting the cell out. Fits a LiPo pouch or an 18650 "
                                     "holder dropped inside. (No melt prongs — never "
                                     "heat-stake against a LiPo)")
    bat_az: FloatProperty(name="Azimuth", default=315.0, min=0.0, max=360.0, update=_upd_plate)
    bat_r: FloatProperty(name="Offset", default=34.0, min=0.0, max=65.0, update=_upd_plate)
    bat_rot: FloatProperty(name="Rotate", default=0.0, min=-90.0, max=90.0, update=_upd_plate)
    bat_w: FloatProperty(name="Battery W", default=62.0, min=20.0, max=110.0, update=_upd_plate,
                         description="Footprint incl. fit clearance (e.g. 603462 LiPo ≈ 62×36; "
                                     "single 18650 holder ≈ 77×21)")
    bat_l: FloatProperty(name="Battery L", default=37.0, min=15.0, max=80.0, update=_upd_plate)
    bat_wall_h: FloatProperty(name="Wall height", default=2.5, min=1.0, max=6.0,
                              update=_upd_plate)
    screen_ap_dia: FloatProperty(name="Aperture Ø", default=36.0, min=24.0, max=42.0,
                                 description="The bore the glass/touch stack fills; visible "
                                             "opening at the surface")
    screen_bore: FloatProperty(name="Bore depth", default=0.5, min=0.1, max=10.0,
                               description="Ø36 bore from the outer surface down to the PCB "
                                           "seat — match the glass stack height so the touch "
                                           "face sits flush (down to 0.1 for a near-flush bezel)")
    screen_pocket_dia: FloatProperty(name="Pocket Ø", default=39.0, min=30.0, max=48.0,
                                     description="PCB pocket below the bore; the module "
                                                 "drops in from inside the shell")
    screen_pcb_depth: FloatProperty(name="PCB pocket depth", default=2.0, min=1.2, max=6.0,
                                    description="Below the seat: PCB thickness (1.2) + "
                                                "fold-over room for the prongs")
    screen_tab_reach: FloatProperty(name="Tab reach", default=41.0, min=36.0, max=48.0,
                                    description="Distance from the tab's flat edge, across "
                                                "the centre, to the opposite pocket edge")
    screen_tab_az: FloatProperty(name="Tab corner angle", default=0.0, min=0.0, max=360.0,
                                 description="Rotation of the flat-edged tab corner around "
                                             "the screen axis")
    screen_prong_len: FloatProperty(name="Prong length", default=3.0, min=1.0, max=8.0,
                                    description="Heat-stake prongs past the pocket, at the "
                                                "four square-tangent points — fold over the "
                                                "PCB back")
    screen_prong_w: FloatProperty(name="Prong width", default=6.0, min=3.0, max=12.0)
    usb_hole: FloatProperty(name="USB hole Ø", default=10.0, min=4.0, max=30.0,
                            description="Round hole in the USB flat (size to your panel-mount part)")
    jack_hole: FloatProperty(name="Jack hole Ø", default=6.5, min=4.0, max=14.0,
                             description="Round hole in the jack flat (6.5 fits a 3.5 mm panel jack)")
    boss_az: FloatProperty(name="Boss azimuth", default=30.0, min=0.0, max=120.0,
                           description="First screw boss direction; the others sit at +120°/+240°")
    boss_dia: FloatProperty(name="Boss Ø", default=8.0, min=5.0, max=12.0)
    boss_h: FloatProperty(name="Boss height", default=8.0, min=3.0, max=20.0)
    pilot_dia: FloatProperty(name="Pilot Ø", default=2.8, min=2.0, max=4.0,
                             description="Pilot hole for M3 self-tapping screws")
    # 8b · PCB mount — parametric standoff unioned into the shell, auto-trimmed to
    # the cavity so the spine/pillar can't poke through the coral wall.
    pcb_mount_on: BoolProperty(name="PCB mount", default=False, update=_upd_pcb,
                               description="Build a PCB standoff into the shell: a base spine "
                                           "across the floor + a centre pillar rising to a screw "
                                           "seat. Clipped to the body so nothing pokes out. "
                                           "Shows a live solid preview before you Build")
    pcb_x: FloatProperty(name="Offset X", default=0.0, min=-90.0, max=90.0, update=_upd_pcb,
                         description="Slide the whole mount in X (mm)")
    pcb_y: FloatProperty(name="Offset Y", default=0.0, min=-90.0, max=90.0, update=_upd_pcb,
                         description="Slide the whole mount in Y (mm)")
    pcb_az: FloatProperty(name="Rotate", default=90.0, min=0.0, max=360.0, update=_upd_pcb,
                          description="Spin the mount about vertical — the spine runs along this "
                                      "direction (90° = along Y, matching the hand-built mount)")
    pcb_seat_z: FloatProperty(name="Seat height", default=95.0, min=20.0, max=180.0, update=_upd_pcb,
                              description="Top of the centre pillar = PCB seat plane (mm above the "
                                          "table). Keep it below the screen pocket floor")
    pcb_span: FloatProperty(name="Spine span", default=200.0, min=20.0, max=300.0, update=_upd_pcb,
                            description="End-to-end spine length before trimming — it gets clipped "
                                        "to whatever the cavity allows, so over-size is fine")
    pcb_spine_w: FloatProperty(name="Spine width", default=20.0, min=4.0, max=50.0, update=_upd_pcb,
                               description="Base spine width across its run")
    pcb_spine_h: FloatProperty(name="Spine height", default=8.0, min=2.0, max=50.0, update=_upd_pcb,
                               description="Base spine standing height off the ledge")
    pcb_post_len: FloatProperty(name="Pillar length", default=30.0, min=6.0, max=80.0, update=_upd_pcb,
                                description="Centre pillar footprint along the spine")
    pcb_post_w: FloatProperty(name="Pillar width", default=27.0, min=6.0, max=80.0, update=_upd_pcb,
                              description="Centre pillar footprint across the spine (Ø when round)")
    pcb_plate_offset: FloatProperty(name="Offset from plate", default=20.0, min=0.0, max=90.0,
                                    update=_upd_pcb,
                                    description="Gap between the plate top and the underside of "
                                                "the mount — lifts the whole spine/pillar this far "
                                                "off the plate (room for battery/speaker below)")
    pcb_post_shape: EnumProperty(name="Pillar", default="SQUARE", update=_upd_pcb,
                                 items=[("SQUARE", "Square", "Rectangular pillar (length × width)"),
                                        ("ROUND", "Round", "Cylindrical pillar (Ø = pillar width)")])
    pcb_lattice_on: BoolProperty(name="Spine lattice", default=False, update=_upd_pcb,
                                 description="Punch a grid of square through-holes in the spine to "
                                             "save material/weight (previews solid; cut at Build)")
    pcb_lattice_wall: FloatProperty(name="Strut", default=3.0, min=1.0, max=12.0, update=_upd_pcb,
                                    description="Material left between the lattice holes and to the "
                                                "spine edges — uniform everywhere (mm)")
    pcb_hole_span: FloatProperty(name="Screw span", default=20.0, min=0.0, max=60.0,
                                 description="Gap between the two pilot holes on the pillar top "
                                             "(0 = a single central hole)")
    pcb_pilot_dia: FloatProperty(name="Pilot Ø", default=2.8, min=1.5, max=4.0,
                                 description="PCB pilot hole (2.8 = M3 self-tap)")
    pcb_pilot_depth: FloatProperty(name="Pilot depth", default=10.0, min=3.0, max=40.0,
                                   description="How deep the pilot holes bore down into the pillar")
    # shell print finish — the fix for hollow prints
    print_remesh: BoolProperty(name="Print-clean remesh", default=True,
                               description="Final voxel remesh of the finished shell — rebuilds it "
                                           "as one watertight manifold so self-intersections from "
                                           "the coral booleans can't fool slicers into printing it "
                                           "hollow. Slower + big file + softens sub-voxel detail")
    print_voxel: FloatProperty(name="Voxel", default=0.9, min=0.25, max=1.5,
                               description="Remesh voxel size (mm) — the triangle-count lever. "
                                           "0.9 ≈ 650k tris (under Creality's 1M limit); 0.5 ≈ 2M "
                                           "(crisper walls, but Creality chokes). Bigger = fewer "
                                           "tris + softer coral")


class OMNI_OT_generate(bpy.types.Operator):
    bl_idname = "omni_sculpt.generate"
    bl_label = "Generate Omniphone"
    bl_description = "Full rebuild (clears every cached stage)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        global _growth
        _growth = None
        _cache.clear()
        try:
            obj, verts, dt = rebuild(context.scene, allow_slow=True)
        except Exception as exc:
            self.report({"ERROR"}, f"generation failed: {exc}")
            raise
        p = context.scene.omni_sculpt
        cop = p.ports_coplanar
        for name, on, z0, dia in (("USB", p.usb_on, p.ports_z if cop else p.usb_z, p.usb_dia),
                                  ("Jack", p.jack_on, p.ports_z if cop else p.jack_z, p.jack_dia)):
            if on and z0 - dia / 2.0 < 8.0:
                self.report({"WARNING"}, f"{name} flat dips into the floor rim — "
                                         f"raise its height or shrink its Ø")
        d = obj.dimensions
        self.report({"INFO"}, f"{len(verts)} verts · {d.x:.0f}×{d.y:.0f}×{d.z:.0f} mm · {dt:.1f}s")
        return {"FINISHED"}


class OMNI_OT_build(bpy.types.Operator):
    bl_idname = "omni_sculpt.build"
    bl_label = "Build Shell + Plate"
    bl_description = ("Engineering pass on the current body: bottom cut, rabbet lip, "
                      "hollow shell, screen/port apertures, screw bosses, bottom plate. "
                      "Takes a few seconds")
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        p = context.scene.omni_sculpt
        t0 = time.time()
        for name in ("Omniphone Shell", "Omniphone Plate"):
            old = bpy.data.objects.get(name)
            if old is not None:
                _kill_obj(old)
        try:
            shell, plate = build_shell_and_plate(p, log=print)
        except Exception as exc:
            self.report({"ERROR"}, f"build failed: {exc}")
            raise
        coll = ensure_collection(COLL_NAME)
        for o in (shell, plate):
            for c in list(o.users_collection):
                c.objects.unlink(o)
            coll.objects.link(o)
        shell.data.polygons.foreach_set("use_smooth",
                                        np.ones(len(shell.data.polygons), dtype=bool))
        shell.data.update()                              # plate stays flat-shaded
        body = bpy.data.objects.get(BODY_NAME)
        if body is not None:
            body.hide_set(True)                          # shell replaces it visually
        try:
            cutplane_sync(context.scene, show=False)     # the cut has happened
            pcb_preview_sync(context.scene, show=False)   # real mount is in the shell now
        except Exception:
            pass
        context.view_layer.objects.active = shell
        d = shell.dimensions
        self.report({"INFO"}, f"shell {d.x:.0f}×{d.y:.0f}×{d.z:.0f} mm · "
                              f"plate Ø{plate.dimensions.x:.0f} · {time.time() - t0:.1f}s")
        return {"FINISHED"}


class OMNI_OT_unbuild(bpy.types.Operator):
    bl_idname = "omni_sculpt.unbuild"
    bl_label = "Undo Build"
    bl_description = ("Delete the built shell + plate and bring the editable body back "
                      "(with the cut-plane / PCB-mount previews), so you can tweak and "
                      "Build again")
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return (bpy.data.objects.get("Omniphone Shell") is not None
                or bpy.data.objects.get("Omniphone Plate") is not None)

    def execute(self, context):
        for name in ("Omniphone Shell", "Omniphone Plate"):
            old = bpy.data.objects.get(name)
            if old is not None:
                _kill_obj(old)
        body = bpy.data.objects.get(BODY_NAME)
        if body is not None:
            body.hide_set(False)                         # editable body returns
            context.view_layer.objects.active = body
        try:
            cutplane_sync(context.scene, show=True)      # previews ride the sliders again
            pcb_preview_sync(context.scene, show=True)
        except Exception:
            pass
        self.report({"INFO"}, "shell + plate removed — body is editable again")
        return {"FINISHED"}


class OMNI_OT_shuffle(bpy.types.Operator):
    bl_idname = "omni_sculpt.shuffle"
    bl_label = "Shuffle"
    bl_description = "New random seed + regenerate (a different instrument every press)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        global _suspend
        _suspend = True
        try:
            context.scene.omni_sculpt.rng_seed = pyrandom.randint(0, 99999)
        finally:
            _suspend = False
        return bpy.ops.omni_sculpt.generate()


def _reload_self():
    """Deferred body of the Reload button: re-read this .py and re-register."""
    import importlib
    mod = sys.modules.get(__name__)
    if mod is None or __name__ == "__main__":
        print("omniphone: cannot self-reload when run as __main__ — re-run the script instead")
        return None
    try:
        unregister()
    except Exception:
        pass
    try:
        newmod = importlib.reload(mod)
        newmod.register()
        print("omniphone: addon reloaded from", getattr(newmod, "__file__", "?"))
    except Exception as exc:
        print("omniphone: reload FAILED (addon may be unregistered):", exc)
    return None


class OMNI_OT_reload(bpy.types.Operator):
    bl_idname = "omni_sculpt.reload_addon"
    bl_label = "Reload Addon"
    bl_description = ("Re-read omniphone_sculptor.py from disk and re-register — the "
                      "dev loop button. Sliders keep their values")

    def execute(self, context):
        # deferred: let this operator finish before its own class is unregistered
        bpy.app.timers.register(_reload_self, first_interval=0.05)
        self.report({"INFO"}, "reloading omniphone_sculptor…")
        return {"FINISHED"}


class OMNI_PT_main(bpy.types.Panel):
    bl_idname = "OMNI_PT_main"
    bl_label = "Omniphone Sculptor"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Omniphone"

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        row = col.row(align=True)
        row.scale_y = 1.4
        row.operator("omni_sculpt.generate", icon="MOD_OCEAN")
        row.operator("omni_sculpt.shuffle", text="", icon="FILE_REFRESH")
        row.operator("omni_sculpt.reload_addon", text="", icon="PLUGIN")
        row = col.row(align=True)
        row.prop(p, "auto_update")
        row.prop(p, "live_growth")


class _OmniSub:
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Omniphone"
    bl_parent_id = "OMNI_PT_main"


class OMNI_PT_ball(_OmniSub, bpy.types.Panel):
    bl_label = "1 · Shape & form"

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.prop(p, "body_algo")
        col.separator()
        col.prop(p, "diameter")
        col.prop(p, "squish_height")
        col.prop(p, "final_height")
        col.prop(p, "flatten_zone")
        col.prop(p, "resolution")


class OMNI_PT_pads(_OmniSub, bpy.types.Panel):
    bl_label = "2 · Pads & layout"

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        if not pads_apply(p):
            col.label(text="Unused: this form ignores pads and the coral is off",
                      icon="INFO")
        sub = col.column()
        sub.enabled = pads_apply(p)
        sub.prop(p, "pad_count")
        sub.prop(p, "pad_layout")
        sub.prop(p, "relax_iters")
        row = sub.row(align=True)
        row.prop(p, "band_lo")
        row.prop(p, "band_hi")
        sub.prop(p, "seed_bulb_r")
        sub.prop(p, "rng_seed")


class OMNI_PT_algo(_OmniSub, bpy.types.Panel):
    bl_label = "3 · Form settings"

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        if p.body_algo == "BALL":
            col.label(text="Plain ball — no settings", icon="MESH_UVSPHERE")
        elif p.body_algo == "VORONOI":
            row = col.row(align=True)
            row.prop(p, "cell_amp")
            row.prop(p, "cell_groove")
        elif p.body_algo == "CRYSTAL":
            row = col.row(align=True)
            row.prop(p, "facet_dist")
            row.prop(p, "facet_jitter")
        elif p.body_algo == "CYMATICS":
            row = col.row(align=True)
            row.prop(p, "cy_m")
            row.prop(p, "cy_n")
            col.prop(p, "cy_amp")
        elif p.body_algo == "MANDELBULB":
            row = col.row(align=True)
            row.prop(p, "mb_power")
            row.prop(p, "mb_iter")
            col.prop(p, "mb_level")


class OMNI_PT_tex(_OmniSub, bpy.types.Panel):
    bl_label = "4 · Texture pass"

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.prop(p, "tex_algo")
        sub = col.column()
        sub.enabled = p.tex_algo != "NONE"
        sub.prop(p, "tex_amount")
        if p.tex_algo in ("VORONOI", "CRYSTAL", "NONE"):
            row = sub.row(align=True)
            row.prop(p, "tex_count")
            row.prop(p, "tex_seed")
            sub.prop(p, "tex_jitter")
            if p.tex_algo != "CRYSTAL":
                sub.prop(p, "tex_groove")
        elif p.tex_algo == "CYMATICS":
            row = sub.row(align=True)
            row.prop(p, "tex_cy_m")
            row.prop(p, "tex_cy_n")
        elif p.tex_algo == "MANDELBULB":
            row = sub.row(align=True)
            row.prop(p, "tex_mb_power")
            row.prop(p, "tex_mb_iter")


class OMNI_PT_coral(_OmniSub, bpy.types.Panel):
    bl_label = "5 · Brain coral"

    def draw_header(self, context):
        self.layout.prop(context.scene.omni_sculpt, "coral_on", text="")

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.enabled = p.coral_on
        row = col.row(align=True)
        row.prop(p, "rd_feed")
        row.prop(p, "rd_kill")
        col.prop(p, "rd_steps")
        col.prop(p, "rd_scale")
        col.prop(p, "rd_noise")
        col.prop(p, "field_smooth")
        row = col.row(align=True)
        row.prop(p, "ridge_height")
        row.prop(p, "groove_depth")
        col.prop(p, "coverage")
        col.prop(p, "cover_fade")


class OMNI_PT_screen(_OmniSub, bpy.types.Panel):
    bl_label = "6 · Screen platform"

    def draw_header(self, context):
        self.layout.prop(context.scene.omni_sculpt, "screen_on", text="")

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.enabled = p.screen_on
        row = col.row(align=True)
        row.prop(p, "screen_tilt")
        row.prop(p, "screen_az")
        col.prop(p, "screen_pitch")
        col.prop(p, "screen_dia")
        col.prop(p, "screen_height")
        col.prop(p, "screen_merge")
        col.prop(p, "screen_strength")


class OMNI_PT_ports(_OmniSub, bpy.types.Panel):
    bl_label = "7 · Panel ports"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.prop(p, "ports_coplanar")
        if p.ports_coplanar:
            box = col.box()
            box.label(text="Shared USB + Jack panel", icon="LINKED")
            row = box.row(align=True)
            row.prop(p, "ports_az")
            row.prop(p, "ports_z")
            row = box.row(align=True)
            row.prop(p, "ports_pitch")
            row.prop(p, "ports_height")
            box.prop(p, "ports_gap")
            row = box.row(align=True)
            row.prop(p, "ports_merge")
            row.prop(p, "ports_strength")
            row = box.row(align=True)
            row.prop(p, "usb_on", text="USB")
            row.prop(p, "usb_dia")
            row = box.row(align=True)
            row.prop(p, "jack_on", text="Jack")
            row.prop(p, "jack_dia")
            return
        col.separator()
        col.prop(p, "usb_on")
        sub = col.column()
        sub.enabled = p.usb_on
        row = sub.row(align=True)
        row.prop(p, "usb_az")
        row.prop(p, "usb_z")
        row = sub.row(align=True)
        row.prop(p, "usb_dia")
        row.prop(p, "usb_height")
        row = sub.row(align=True)
        row.prop(p, "usb_merge")
        row.prop(p, "usb_pitch")
        col.separator()
        col.prop(p, "jack_on")
        sub = col.column()
        sub.enabled = p.jack_on
        row = sub.row(align=True)
        row.prop(p, "jack_az")
        row.prop(p, "jack_z")
        row = sub.row(align=True)
        row.prop(p, "jack_dia")
        row.prop(p, "jack_height")
        row = sub.row(align=True)
        row.prop(p, "jack_merge")
        row.prop(p, "jack_pitch")


class OMNI_PT_shell(_OmniSub, bpy.types.Panel):
    bl_label = "8 · Shell & plate"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        row = col.row(align=True)
        row.scale_y = 1.3
        row.operator("omni_sculpt.build", icon="MOD_SOLIDIFY")
        row.operator("omni_sculpt.unbuild", text="", icon="LOOP_BACK")
        col.prop(p, "wall")
        row = col.row(align=True)
        row.prop(p, "bottom_cut")
        row.prop(p, "plate_recess")
        row = col.row(align=True)
        row.prop(p, "plate_inset")
        row.prop(p, "plate_clear")
        col.prop(p, "plate_ridge_h")
        col.prop(p, "seam_smooth")
        col.separator()
        col.label(text="Screen pocket (module drops in from inside)", icon="RESTRICT_VIEW_OFF")
        import os as _os
        has_stl = _os.path.exists(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                                "screencutout.stl"))
        if has_stl:
            col.label(text="Cutter: screencutout.stl (edit the file to change)",
                      icon="FILE_TICK")
        else:
            row = col.row(align=True)
            row.prop(p, "screen_ap_dia")
            row.prop(p, "screen_bore")
            row = col.row(align=True)
            row.prop(p, "screen_pocket_dia")
            row.prop(p, "screen_pcb_depth")
            col.prop(p, "screen_tab_reach")
        row = col.row(align=True)
        row.prop(p, "screen_tab_az")
        row = col.row(align=True)
        row.prop(p, "screen_prong_len")
        row.prop(p, "screen_prong_w")
        col.separator()
        col.label(text="Port holes", icon="PLUGIN")
        row = col.row(align=True)
        row.prop(p, "usb_hole")
        row.prop(p, "jack_hole")
        col.separator()
        col.label(text="Plate screws (M3 self-tap)", icon="TOOL_SETTINGS")
        row = col.row(align=True)
        row.prop(p, "boss_az")
        row.prop(p, "boss_dia")
        row = col.row(align=True)
        row.prop(p, "boss_h")
        row.prop(p, "pilot_dia")
        col.separator()
        box = col.box()
        box.prop(p, "pcb_mount_on", icon="LIBRARY_DATA_DIRECT")
        if p.pcb_mount_on:
            row = box.row(align=True)
            row.prop(p, "pcb_x")
            row.prop(p, "pcb_y")
            row = box.row(align=True)
            row.prop(p, "pcb_az")
            row.prop(p, "pcb_seat_z")
            box.prop(p, "pcb_plate_offset")
            row = box.row(align=True)
            row.prop(p, "pcb_span")
            row.prop(p, "pcb_post_shape")
            row = box.row(align=True)
            row.prop(p, "pcb_spine_w")
            row.prop(p, "pcb_spine_h")
            row = box.row(align=True)
            row.prop(p, "pcb_post_len")
            row.prop(p, "pcb_post_w")
            row = box.row(align=True)
            row.prop(p, "pcb_lattice_on")
            if p.pcb_lattice_on:
                row.prop(p, "pcb_lattice_wall")
            row = box.row(align=True)
            row.prop(p, "pcb_hole_span")
            row.prop(p, "pcb_pilot_dia")
            box.prop(p, "pcb_pilot_depth")
        col.separator()
        box = col.box()
        box.prop(p, "print_remesh", icon="MOD_REMESH")
        if p.print_remesh:
            box.prop(p, "print_voxel")


class OMNI_PT_plate(_OmniSub, bpy.types.Panel):
    bl_label = "9 · Plate outfit (live after Build)"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.label(text="Legs", icon="SORTBYEXT")
        row = col.row(align=True)
        row.prop(p, "leg_count")
        row.prop(p, "leg_h")
        row = col.row(align=True)
        row.prop(p, "leg_dia")
        row.prop(p, "leg_pad_dia")
        row = col.row(align=True)
        row.prop(p, "leg_r")
        row.prop(p, "leg_angle")
        row = col.row(align=True)
        row.prop(p, "leg_az0")
        row.prop(p, "leg_fillet")
        col.separator()
        col.prop(p, "spk_on", icon="SPEAKER")
        sub = col.column()
        sub.enabled = p.spk_on
        sub.prop(p, "spk_type")
        row = sub.row(align=True)
        row.prop(p, "spk_az")
        row.prop(p, "spk_r")
        row.prop(p, "spk_rot")
        if p.spk_type == "ROUND":
            row = sub.row(align=True)
            row.prop(p, "spk_dia")
            row.prop(p, "spk_ridge_h")
            sub.prop(p, "spk_lip")
        else:
            row = sub.row(align=True)
            row.prop(p, "spk_w")
            row.prop(p, "spk_l")
            row.prop(p, "spk_lip")
        col.separator()
        col.prop(p, "trans_on", icon="FORCE_HARMONIC")
        sub = col.column()
        sub.enabled = p.trans_on
        row = sub.row(align=True)
        row.prop(p, "trans_az")
        row.prop(p, "trans_r")
        row = sub.row(align=True)
        row.prop(p, "trans_dia")
        row.prop(p, "trans_thin")
        col.separator()
        col.prop(p, "pam_on", icon="MOD_BUILD")
        sub = col.column()
        sub.enabled = p.pam_on
        row = sub.row(align=True)
        row.prop(p, "pam_az")
        row.prop(p, "pam_r")
        row.prop(p, "pam_rot")
        row = sub.row(align=True)
        row.prop(p, "pam_w")
        row.prop(p, "pam_l")
        col.separator()
        col.prop(p, "sw_on", icon="CHECKBOX_HLT")
        sub = col.column()
        sub.enabled = p.sw_on
        row = sub.row(align=True)
        row.prop(p, "sw_az")
        row.prop(p, "sw_r")
        row.prop(p, "sw_rot")
        sub.prop(p, "sw_prong_d")
        col.separator()
        col.prop(p, "bat_on", icon="SNAP_VOLUME")
        sub = col.column()
        sub.enabled = p.bat_on
        row = sub.row(align=True)
        row.prop(p, "bat_az")
        row.prop(p, "bat_r")
        row.prop(p, "bat_rot")
        row = sub.row(align=True)
        row.prop(p, "bat_w")
        row.prop(p, "bat_l")
        row.prop(p, "bat_wall_h")


CLASSES = (OmniSculptProps, OMNI_OT_generate, OMNI_OT_build, OMNI_OT_unbuild,
           OMNI_OT_shuffle, OMNI_OT_reload,
           OMNI_PT_main, OMNI_PT_ball, OMNI_PT_pads, OMNI_PT_algo,
           OMNI_PT_tex, OMNI_PT_coral,
           OMNI_PT_screen, OMNI_PT_ports, OMNI_PT_shell, OMNI_PT_plate)


def register():
    for c in CLASSES:
        bpy.utils.register_class(c)
    bpy.types.Scene.omni_sculpt = PointerProperty(type=OmniSculptProps)


def unregister():
    global _growth
    _growth = None
    if bpy.app.timers.is_registered(_growth_tick):
        bpy.app.timers.unregister(_growth_tick)
    del bpy.types.Scene.omni_sculpt
    for c in reversed(CLASSES):
        bpy.utils.unregister_class(c)


# ---------------------------------------------------------------------------
# headless selftest
# ---------------------------------------------------------------------------

def _mesh_verts_np(me):
    out = np.zeros(len(me.vertices) * 3, dtype=np.float32)
    me.vertices.foreach_get("co", out)
    return out.reshape(-1, 3)


def _mesh_closed(me):
    loops = np.zeros(len(me.loops), dtype=np.int64)
    me.loops.foreach_get("vertex_index", loops)
    starts = np.zeros(len(me.polygons), dtype=np.int64)
    totals = np.zeros(len(me.polygons), dtype=np.int64)
    me.polygons.foreach_get("loop_start", starts)
    me.polygons.foreach_get("loop_total", totals)
    edges = []
    for s, t in zip(starts, totals):                     # booleans emit ngons too
        poly = loops[s:s + t]
        edges.append(np.stack([poly, np.roll(poly, -1)], axis=1))
    e = np.concatenate(edges)
    e.sort(axis=1)
    _, counts = np.unique(e, axis=0, return_counts=True)
    return bool((counts == 2).all())


def _point_inside(obj, pt):
    """Ray-parity inside test on an evaluated mesh object."""
    from mathutils import Vector as V
    d = V((0.0173, 0.031, 0.9971)).normalized()
    o = V(pt)
    n = 0
    for _ in range(500):
        hit, loc, _, _ = obj.ray_cast(o, d, distance=1e4)
        if not hit:
            break
        n += 1
        o = loc + d * 1e-3
    return n % 2 == 1


def _facet_flatness(verts, d, c0, disc_r, plane_t):
    rel = verts - c0
    t = rel @ d
    r = np.linalg.norm(rel - t[:, None] * d, axis=1)
    gate = (r < disc_r - 1.5) & (t > plane_t - 2.5)      # the seat, not body parts
    assert gate.sum() > 10, "facet has too few vertices"  # crossing the axis deeper
    return float(t[gate].max() - t[gate].min())


def _selftest():
    import os
    scene = bpy.context.scene
    p = scene.omni_sculpt

    res = bpy.ops.omni_sculpt.generate()
    assert res == {"FINISHED"}, f"generate returned {res}"
    obj = bpy.data.objects[BODY_NAME]
    me = obj.data

    loops = np.zeros(len(me.loops), dtype=np.int64)
    me.loops.foreach_get("vertex_index", loops)
    faces = loops.reshape(-1, 3)
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    e.sort(axis=1)
    _, counts = np.unique(e, axis=0, return_counts=True)
    closed = bool((counts == 2).all())
    d = obj.dimensions
    n_pads = len(bpy.data.collections[PADS_COLL_NAME].objects)
    print(f"SELFTEST verts={len(me.vertices)} faces={len(me.polygons)} closed={closed} "
          f"dims={d.x:.1f}x{d.y:.1f}x{d.z:.1f} pads={n_pads}")
    assert closed, "mesh is not a closed 2-manifold"
    assert p.final_height - p.groove_depth - 1 < d.z < p.final_height + p.ridge_height + 1
    assert n_pads == p.pad_count, "pad empty count mismatch"

    # facet flatness on the real output mesh
    verts = _mesh_verts_np(me).astype(np.float64)
    facets = json.loads(obj["omniphone_facets"])
    assert {f["name"] for f in facets} == {"Screen", "USB", "Jack"}
    for f in facets:
        flat = _facet_flatness(verts, np.array(f["dir"]), np.array(f["origin"]),
                               f["radius"], f["plane_t"])
        print(f"SELFTEST facet {f['name']}: flatness spread {flat:.3f} mm")
        assert flat < 0.35, f"{f['name']} facet not flat ({flat:.3f} mm)"
    for name in MARKER_NAMES:
        assert bpy.data.objects.get(name) is not None, f"{name} marker missing"

    # live-update fast path: displacement slider rebuilds in-place, quickly
    before = _mesh_verts_np(me).sum()
    t0 = time.time()
    p.ridge_height = p.ridge_height + 2.0                # fires UPD_FAST synchronously
    dt_fast = time.time() - t0
    after = _mesh_verts_np(obj.data).sum()
    print(f"SELFTEST fast path: {dt_fast * 1000:.0f} ms (mesh changed: {before != after})")
    assert before != after, "fast update did not modify the mesh"
    assert dt_fast < 0.8, f"fast path too slow ({dt_fast:.2f}s)"

    # debounced slow path: RD param marks pending; flush runs the sim
    before = after
    p.rd_kill = 0.058                                    # schedules, no immediate rebuild
    mid = _mesh_verts_np(obj.data).sum()
    assert mid == before, "slow param should not rebuild synchronously"
    _flush_pending()
    after = _mesh_verts_np(obj.data).sum()
    print(f"SELFTEST debounce flush: mesh changed: {before != after}")
    assert before != after, "debounce flush did not rebuild"

    # progressive growth (live mode): drive the timer callback like the GUI does
    global _growth
    before = after
    p.live_growth = True
    p.rd_feed = 0.052                                    # schedules a growth job
    assert _growth is not None, "slow param should schedule growth"
    _growth["not_before"] = 0.0                          # skip the drag debounce
    ticks, changed_mid = 0, False
    while _growth is not None:
        _growth_tick()
        ticks += 1
        if _growth is not None and not changed_mid:
            changed_mid = _mesh_verts_np(obj.data).sum() != before
        assert ticks < 500, "growth never finished"
    print(f"SELFTEST growth: {ticks} chunks, mesh updated mid-flight: {changed_mid}")
    assert ticks >= 3, "growth should take multiple chunks"
    assert changed_mid, "growth should update the mesh while running"

    # skip mode (default): background chunks, mesh only written at the end
    p.live_growth = False
    before = _mesh_verts_np(obj.data).sum()
    p.rd_feed = 0.054
    assert _growth is not None
    _growth["not_before"] = 0.0
    ticks, changed_mid = 0, False
    while _growth is not None:
        _growth_tick()
        ticks += 1
        if _growth is not None and not changed_mid:
            changed_mid = _mesh_verts_np(obj.data).sum() != before
        assert ticks < 500, "skip-mode growth never finished"
    changed_end = _mesh_verts_np(obj.data).sum() != before
    print(f"SELFTEST skip mode: {ticks} chunks, mid-flight write: {changed_mid}, "
          f"final write: {changed_end}")
    assert not changed_mid and changed_end, "skip mode should write only at the end"

    # engineering pass: shell + plate
    res = bpy.ops.omni_sculpt.build()
    assert res == {"FINISHED"}, f"build returned {res}"
    shell = bpy.data.objects["Omniphone Shell"]
    plate = bpy.data.objects["Omniphone Plate"]
    assert _mesh_closed(shell.data), "shell is not closed"
    assert _mesh_closed(plate.data), "plate is not closed"
    sd, pd = shell.dimensions, plate.dimensions
    print(f"SELFTEST shell dims={sd.x:.1f}x{sd.y:.1f}x{sd.z:.1f} "
          f"verts={len(shell.data.vertices)}")
    print(f"SELFTEST plate Ø≈{pd.x:.1f}x{pd.y:.1f} t={pd.z:.2f}")
    top_extra = max(p.plate_ridge_h if p.plate_ridge_h > 0.05 else 0.0,
                    2.0 if p.sw_on else 0.0, 1.6 if p.pam_on else 0.0)
    plate_t = p.plate_recess + top_extra + (p.leg_h if p.leg_count > 0 else 0.0)
    assert abs(pd.z - plate_t) < 0.1, f"plate stack height wrong ({pd.z:.2f} vs {plate_t:.2f})"
    assert abs(shell.dimensions.z - (d.z - p.bottom_cut)) < 4.0, "shell height unexpected"

    # probes: cavity hollow, screen pocket open+through, wall ring, corner, prong
    fx = {ff["name"]: ff for ff in json.loads(obj["omniphone_facets"])}
    sc = fx["Screen"]
    dvec = np.array(sc["dir"])
    org = np.array(sc["origin"])
    tp = sc["plane_t"]
    u_ax, v_ax = _axes(dvec)
    e_pl = lambda ang: u_ax * cos(ang) + v_ax * sin(ang)
    Rp = p.screen_pocket_dia / 2.0
    tab = radians(p.screen_tab_az)
    scr = json.loads(shell["omniphone_screen"])
    t_floor, boss_r = scr["t_floor"], scr["boss_r"]
    print(f"SELFTEST screen mode={scr['mode']} t_floor={tp - t_floor:.2f} below surface")
    assert not _point_inside(shell, org + dvec * (tp - 2.0)), "screen bore not open"
    assert not _point_inside(shell, org + dvec * (t_floor - 3.0)), "pocket not cut through"
    assert _point_inside(shell, org + dvec * (tp - 2.5) +
                         e_pl(tab + radians(157.5)) * ((Rp + boss_r) / 2.0)), \
        "pocket wall (boss ring) missing"
    r_tab_probe = (Rp + boss_r - 2.0) / 2.0              # between pocket Ø and tab reach
    assert _point_inside(shell, org + dvec * (t_floor - p.screen_prong_len / 2.0) +
                         e_pl(tab + pi / 4) * (Rp + 1.1)), "melt prong missing"
    if scr["mode"] == "stl":
        assert not _point_inside(shell, org + dvec * (tp - 3.0) + e_pl(tab) * r_tab_probe), \
            "tab region not cut"
    else:
        t_seat = tp - p.screen_bore
        r_wall = (p.screen_ap_dia / 2.0 + Rp) / 2.0
        assert not _point_inside(shell, org + dvec * (t_seat - 0.6) + e_pl(tab) * r_tab_probe), \
            "tab region not cut"
        assert _point_inside(shell, org + dvec * (tp - 2.0) + e_pl(tab + pi) * r_wall), \
            "bore wall missing"
        assert not _point_inside(shell, org + dvec * (t_seat - 0.6) + e_pl(tab + pi) * r_wall), \
            "PCB pocket not carved below the seat"
    assert not _point_inside(shell, [0, 0, p.final_height * 0.5]), "cavity not hollow"
    bot = json.loads(shell["omniphone_bottom"])          # the build's own numbers
    rRa = np.array(bot["rR"])
    r_probe = float(np.interp(pi / 2, np.linspace(0, 2 * pi, len(rRa), endpoint=False),
                              rRa, period=2 * pi)) - 2.0
    zl = bot["z_ledge"]
    assert _point_inside(shell, [0, r_probe, zl + p.wall * 0.4]), \
        "no ledge material above the plate plane"
    assert not _point_inside(shell, [0, r_probe, zl - p.plate_recess * 0.5]), \
        "recess not carved (plate space occupied)"

    out = os.path.join(os.getcwd(), "omniphone_selftest.stl")
    for o in bpy.context.view_layer.objects:
        o.select_set(o is shell)
    bpy.context.view_layer.objects.active = shell
    bpy.ops.wm.stl_export(filepath=out, export_selected_objects=True)
    assert os.path.getsize(out) > 500_000, "STL suspiciously small"
    print(f"SELFTEST stl={out} ({os.path.getsize(out) // 1024} kB)")

    # layer stack: pure forms never run the RD sim; coral/texture layer back
    # on in any combination; pad markers exist exactly when pads apply
    p.rd_steps = 400
    for form, coral, tex in (("VORONOI", False, "NONE"), ("CRYSTAL", False, "NONE"),
                             ("CYMATICS", False, "NONE"), ("MANDELBULB", False, "NONE"),
                             ("VORONOI", False, "CRYSTAL"),   # cells + crystal texture
                             ("BALL", True, "VORONOI"),       # brain + cells
                             ("CYMATICS", True, "NONE")):     # cymatics + brain
        p.body_algo = form
        p.coral_on = coral
        p.tex_algo = tex
        t0a = time.time()
        bpy.ops.omni_sculpt.generate()
        body_obj = bpy.data.objects[BODY_NAME]
        loops2 = np.zeros(len(body_obj.data.loops), dtype=np.int64)
        body_obj.data.loops.foreach_get("vertex_index", loops2)
        f2 = loops2.reshape(-1, 3)
        e2 = np.concatenate([f2[:, [0, 1]], f2[:, [1, 2]], f2[:, [2, 0]]])
        e2.sort(axis=1)
        _, c2 = np.unique(e2, axis=0, return_counts=True)
        dd = body_obj.dimensions
        n_pads2 = len(bpy.data.collections[PADS_COLL_NAME].objects)
        ran_rd = _cache.get("rd") is not None              # generate() cleared the cache
        tag = form + ("+coral" if coral else "") + (f"+tex{tex}" if tex != "NONE" else "")
        print(f"SELFTEST body {tag}: closed={bool((c2 == 2).all())} "
              f"dims={dd.x:.0f}x{dd.y:.0f}x{dd.z:.0f} "
              f"pads={n_pads2} rd={ran_rd} ({time.time() - t0a:.1f}s)")
        assert bool((c2 == 2).all()), f"{tag} body not closed"
        assert ran_rd == coral_active(p), f"{tag}: RD {'ran' if ran_rd else 'skipped'} unexpectedly"
        exp_pads = p.pad_count if pads_apply(p) else 0
        assert n_pads2 == exp_pads, f"{tag}: {n_pads2} pad markers, expected {exp_pads}"

    # texture pass is a live fast-path layer
    p.body_algo, p.coral_on, p.tex_algo = "BALL", False, "NONE"
    bpy.ops.omni_sculpt.generate()
    s0 = _mesh_verts_np(bpy.data.objects[BODY_NAME].data).sum()
    p.tex_algo = "VORONOI"                               # UPD_FAST → instant texture
    s1 = _mesh_verts_np(bpy.data.objects[BODY_NAME].data).sum()
    print(f"SELFTEST texture toggle: mesh changed: {s0 != s1}")
    assert s0 != s1, "texture pass did not displace"
    print("SELFTEST PASS")


if __name__ == "__main__":
    try:
        unregister()
    except Exception:
        pass
    register()
    if "--selftest" in sys.argv:
        _selftest()
