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


def flatten_facet(verts, unit_dirs, d, c0, disc_r, merge, strength, plane_offset=0.0):
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
    t_plane = t_surf + plane_offset
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
    if p.pad_layout == "PHYLLO":
        i = np.arange(n)
        z = hi - (hi - lo) * ((i + 0.5) / n)
        th = i * GOLDEN_ANGLE
    else:
        z = rng.uniform(lo, hi, n)
        th = rng.uniform(0.0, 2 * pi, n)
    r = np.sqrt(np.clip(1 - z * z, 0, None))
    dirs = np.stack([r * np.cos(th), r * np.sin(th), z], axis=1)

    iters = int(p.relax_iters) if p.pad_layout != "LLOYD" else max(int(p.relax_iters), 15)
    for _ in range(iters):
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


def rd_setup(p, geo):
    """Lay out pads, stamp seed bulbs (in unit-sphere angle space, so the field
    is independent of squish/flatten), init Gray-Scott state. Deterministic per
    rng_seed. Returns a state dict that rd_run() advances."""
    rng = np.random.default_rng(int(p.rng_seed))
    R = p.diameter / 2.0
    sd, min_ang = None, 0.0
    if p.screen_on:
        sd = _screen_dir(p)
        min_ang = asin(min((p.screen_dia / 2.0 + p.seed_bulb_r + 2.0) / (R * 0.85), 0.95))
    dirs = pad_layout(p, rng, sd, min_ang)
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


def _key_shape(p):
    return _key_geo(p) + tuple(round(float(x), 4) for x in (
        p.diameter, p.squish_height, p.final_height, p.flatten_zone,
        p.screen_on, p.screen_tilt, p.screen_az, p.screen_dia, p.screen_height,
        p.screen_merge, p.screen_strength, p.screen_pitch,
        p.usb_on, p.usb_az, p.usb_z, p.usb_dia, p.usb_merge, p.usb_pitch,
        p.jack_on, p.jack_az, p.jack_z, p.jack_dia, p.jack_merge, p.jack_pitch))


def _key_rd(p):
    return _key_geo(p) + tuple(round(float(x), 4) for x in (
        p.diameter, p.pad_count, hash(p.pad_layout) % 10**6, p.relax_iters,
        p.band_lo, p.band_hi, p.seed_bulb_r, p.rng_seed,
        p.rd_feed, p.rd_kill, p.rd_steps, p.rd_scale, p.rd_noise,
        p.screen_on, p.screen_tilt, p.screen_az, p.screen_dia))


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


def stage_shape(p, geo):
    def build():
        R = p.diameter / 2.0
        final_h = min(p.final_height, p.squish_height)
        verts = squish_and_flatten(geo["unit"] * R, p.diameter,
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
        for name, on, az, z0, dia, merge, pitch in (
                ("USB", p.usb_on, p.usb_az, p.usb_z, p.usb_dia, p.usb_merge, p.usb_pitch),
                ("Jack", p.jack_on, p.jack_az, p.jack_z, p.jack_dia, p.jack_merge, p.jack_pitch)):
            if not on:
                continue
            a = radians(az)
            d_pos = np.array([cos(a), sin(a), 0.0])
            anchor, d = _facet_frame(verts, n_pre, geo["unit"], d_pos, a, pitch,
                                     dia / 2.0, z_slab=float(z0), absolute=True)
            c0 = anchor - d * 40.0
            verts, s, t_plane = flatten_facet(verts, geo["unit"], d, c0,
                                              dia / 2.0, merge, 1.0)
            supp = np.minimum(supp, s)
            facets.append((name, d, c0, t_plane, dia / 2.0))
        normals = vertex_normals(verts, geo["faces"])
        return {"verts": verts, "normals": normals, "supp": supp,
                "facets": facets, "final_h": final_h}
    return _stage("shape", _key_shape(p), True, False, build)


def stage_displace(p, geo, shape, rd):
    """Cheap final stage — never cached."""
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
    amp *= shape["supp"]
    return shape["verts"] + shape["normals"] * amp[:, None]


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
    rd = stage_rd(p, geo, allow_slow, log=log)
    shape = stage_shape(p, geo)
    verts = stage_displace(p, geo, shape, rd)
    set_units_mm(scene)
    coll = ensure_collection(COLL_NAME)
    obj = write_body(verts, geo["faces"], coll)
    try:
        obj.hide_set(False)                              # editing the body un-hides it
    except Exception:
        pass
    pads_coll = ensure_collection(PADS_COLL_NAME, parent=coll)
    write_pad_empties(pads_coll, verts[rd["centers"]], p.seed_bulb_r)
    write_markers(coll, shape["facets"])
    obj["omniphone_params"] = json.dumps(props_as_dict(p))
    obj["omniphone_pads"] = json.dumps([list(map(float, verts[i])) for i in rd["centers"]])
    obj["omniphone_facets"] = json.dumps(
        [{"name": n, "dir": list(map(float, d)), "origin": list(map(float, c0)),
          "plane_t": float(tp), "radius": float(r)} for n, d, c0, tp, r in shape["facets"]])
    try:
        cutplane_sync(scene)
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


def build_shell_and_plate(p, log=print):
    """The engineering pass: hollow shell with rabbet lip, apertures, bosses —
    plus the matching bottom plate. Returns (shell_obj, plate_obj)."""
    t0 = time.time()
    geo = stage_geo(p, allow_slow=True)
    rd = stage_rd(p, geo, allow_slow=True)
    shape = stage_shape(p, geo)
    verts = stage_displace(p, geo, shape, rd)
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
    unions, holes, plate_holes, plate_sinks = [], [], [], []
    if "Screen" in facets:
        _, d, c0, tp, _ = facets["Screen"]
        boss_r = p.seat_dia / 2.0 + 2.5
        unions.append(solid_frustum(c0, d, boss_r, boss_r, tp - p.seat_depth - wall, tp - 1.0))
        holes.append(solid_frustum(c0, d, p.seat_dia / 2.0, p.seat_dia / 2.0,
                                   tp - p.seat_depth, tp + 60))
        holes.append(solid_frustum(c0, d, p.aperture_dia / 2.0, p.aperture_dia / 2.0,
                                   tp - p.seat_depth - wall - 60, tp + 60))
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
        plate_holes.append(solid_frustum(c, up, 1.7, 1.7, cut - 2, zl + 2))
        plate_sinks.append(solid_frustum(c, up, 3.3, 1.7, cut - 0.01, cut + 1.6))  # countersink
    if unions:
        bool_apply(shell, _tmp_obj("omni_unions", *merge_solids(unions)), "UNION",
                   label="bosses", log=log)
    for i, h in enumerate(holes):                        # one op per hole: the seat
        bool_apply(shell, _tmp_obj(f"omni_hole{i}", *h), "DIFFERENCE",  # and aperture
                   label=f"hole {i}", log=log)           # overlap, manifold solver
    #                                                      wants disjoint operands

    log("  plate…")
    plate = _tmp_obj("Omniphone Plate", *solid_polar_prism(rR - p.plate_clear, cut, zl))
    bool_apply(plate, _tmp_obj("omni_pholes", *merge_solids(plate_holes)), "DIFFERENCE",
               label="plate holes", log=log)
    bool_apply(plate, _tmp_obj("omni_psinks", *merge_solids(plate_sinks)), "DIFFERENCE",
               label="plate countersinks", log=log)
    if p.plate_ridge_h > 0.05:
        # key ridge: rises into the shell's bottom opening, tracing the same
        # lip/boss-pad profile the cavity was carved with (minus clearance) —
        # the plate can neither slide nor rotate
        lip_inner = rC + 0.5 - pad_bump
        ridge_out = lip_inner - p.plate_clear
        ridge_in = ridge_out - max(2.0, wall * 0.7)
        bool_apply(plate, _tmp_obj("omni_pridge", *solid_annular_prism(
            ridge_in, ridge_out, zl - 0.2, zl + p.plate_ridge_h)), "UNION",
            label="plate key ridge", log=log)

    shell["omniphone_bottom"] = json.dumps({           # exact bottom profile, for
        "rR": np.round(rR, 3).tolist(),                # checks & later slices
        "z_ledge": float(zl), "cut": float(cut),
        "boss_az_deg": [float(np.degrees(a)) for a in boss_az],
        "boss_r": [float(r) for r in r_boss]})
    log(f"  build done in {time.time() - t0:.1f}s")
    return shell, plate


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
    # 2 · bottom flatten
    final_height: FloatProperty(name="Final height", default=100.0, min=30.0, max=280.0,
                                update=UPD_FAST,
                                description="Total height after gradient-flattening the underside (mm)")
    flatten_zone: FloatProperty(name="Blend zone", default=30.0, min=5.0, max=80.0,
                                update=UPD_FAST,
                                description="How far above the floor the flattening blends out (mm)")
    # 3 · pad seeds
    pad_count: IntProperty(name="Pad count", default=12, min=0, max=64, update=UPD_SLOW,
                           description="Number of touch pads = coral seed bulbs")
    pad_layout: EnumProperty(name="Layout", default="PHYLLO", update=UPD_SLOW, items=[
        ("PHYLLO", "Phyllotaxis", "Golden-angle spiral over the band"),
        ("RANDOM", "Random", "Uniform random over the band"),
        ("LLOYD", "Lloyd (relaxed)", "Random start + repulsion relaxation "
                                     "(uses at least 15 relax iterations)")])
    relax_iters: IntProperty(name="Relax", default=0, min=0, max=60, update=UPD_SLOW,
                             description="Repulsion-relaxation iterations applied to the layout")
    band_lo: FloatProperty(name="Band low", default=0.05, min=-0.6, max=0.9, update=UPD_SLOW,
                           description="Lowest pad position (z on the unit ball: 1 = top pole, 0 = equator)")
    band_hi: FloatProperty(name="Band high", default=0.92, min=0.0, max=1.0, update=UPD_SLOW,
                           description="Highest pad position (keep < 1.0 to leave the crown free)")
    seed_bulb_r: FloatProperty(name="Seed bulb Ø/2", default=5.0, min=2.0, max=15.0, update=UPD_SLOW,
                               description="Radius of the RD seed disc at each pad centre (mm)")
    rng_seed: IntProperty(name="Random seed", default=3, min=0, max=99999, update=UPD_SLOW,
                          description="Every seed is a different instrument")
    # 4 · brain coral
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
    screen_dia: FloatProperty(name="Platform Ø", default=44.0, min=34.0, max=70.0, update=UPD_BOTH,
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
    usb_on: BoolProperty(name="USB flat", default=True, update=UPD_FAST)
    usb_az: FloatProperty(name="Azimuth", default=180.0, min=0.0, max=360.0, update=UPD_FAST)
    usb_z: FloatProperty(name="Height", default=22.0, min=4.0, max=60.0, update=UPD_FAST,
                         description="Centre height of the USB flat above the floor (mm). "
                                     "Keep the disc above the rounded floor rim: "
                                     "height − Ø/2 ≳ 8 mm")
    usb_dia: FloatProperty(name="Flat Ø", default=26.0, min=12.0, max=44.0, update=UPD_FAST,
                           description="Flat spot for a panel-mount USB-C (flange + nut clearance)")
    usb_merge: FloatProperty(name="Merge reach", default=8.0, min=0.0, max=30.0, update=UPD_FAST)
    usb_pitch: FloatProperty(name="Pitch", default=0.0, min=-45.0, max=45.0, update=UPD_FAST,
                             description="Absolute: 0° = face perpendicular to the ground; "
                                         "+ tilts the face up, − down")
    jack_on: BoolProperty(name="Jack flat", default=True, update=UPD_FAST)
    jack_az: FloatProperty(name="Azimuth", default=210.0, min=0.0, max=360.0, update=UPD_FAST)
    jack_z: FloatProperty(name="Height", default=22.0, min=4.0, max=60.0, update=UPD_FAST)
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
    plate_inset: FloatProperty(name="Plate inset", default=4.0, min=2.5, max=12.0,
                               description="How far the plate outline shrinks inside the footprint (mm)")
    plate_clear: FloatProperty(name="Plate clearance", default=0.25, min=0.05, max=0.8,
                               description="Radial print-fit clearance between plate and recess (mm)")
    plate_ridge_h: FloatProperty(name="Key ridge", default=2.5, min=0.0, max=8.0,
                                 description="Standing ridge on the plate that keys into the "
                                             "shell's bottom opening (follows the boss-pad "
                                             "profile, so it locks rotation too). 0 = off")
    seat_dia: FloatProperty(name="Screen seat Ø", default=38.5, min=30.0, max=46.0,
                            description="Counterbore for the 1.28″ module (Ø37.5 PCB + fit)")
    seat_depth: FloatProperty(name="Seat depth", default=4.5, min=1.0, max=12.0)
    aperture_dia: FloatProperty(name="Screen aperture Ø", default=33.5, min=24.0, max=40.0,
                                description="Through-hole for the visible display (view Ø32.4)")
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
        for name, on, z0, dia in (("USB", p.usb_on, p.usb_z, p.usb_dia),
                                  ("Jack", p.jack_on, p.jack_z, p.jack_dia)):
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
        except Exception:
            pass
        context.view_layer.objects.active = shell
        d = shell.dimensions
        self.report({"INFO"}, f"shell {d.x:.0f}×{d.y:.0f}×{d.z:.0f} mm · "
                              f"plate Ø{plate.dimensions.x:.0f} · {time.time() - t0:.1f}s")
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
    bl_label = "1 · Ball & bottom"

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.prop(p, "diameter")
        col.prop(p, "squish_height")
        col.prop(p, "final_height")
        col.prop(p, "flatten_zone")
        col.prop(p, "resolution")


class OMNI_PT_pads(_OmniSub, bpy.types.Panel):
    bl_label = "2 · Pad seeds"

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.prop(p, "pad_count")
        col.prop(p, "pad_layout")
        col.prop(p, "relax_iters")
        row = col.row(align=True)
        row.prop(p, "band_lo")
        row.prop(p, "band_hi")
        col.prop(p, "seed_bulb_r")
        col.prop(p, "rng_seed")


class OMNI_PT_coral(_OmniSub, bpy.types.Panel):
    bl_label = "3 · Brain coral"

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
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
    bl_label = "4 · Screen platform"

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
    bl_label = "5 · Panel ports"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        col.prop(p, "usb_on")
        sub = col.column()
        sub.enabled = p.usb_on
        row = sub.row(align=True)
        row.prop(p, "usb_az")
        row.prop(p, "usb_z")
        row = sub.row(align=True)
        row.prop(p, "usb_dia")
        row.prop(p, "usb_merge")
        sub.prop(p, "usb_pitch")
        col.separator()
        col.prop(p, "jack_on")
        sub = col.column()
        sub.enabled = p.jack_on
        row = sub.row(align=True)
        row.prop(p, "jack_az")
        row.prop(p, "jack_z")
        row = sub.row(align=True)
        row.prop(p, "jack_dia")
        row.prop(p, "jack_merge")
        sub.prop(p, "jack_pitch")


class OMNI_PT_shell(_OmniSub, bpy.types.Panel):
    bl_label = "6 · Shell & plate"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        p = context.scene.omni_sculpt
        col = self.layout.column()
        row = col.row()
        row.scale_y = 1.3
        row.operator("omni_sculpt.build", icon="MOD_SOLIDIFY")
        col.prop(p, "wall")
        row = col.row(align=True)
        row.prop(p, "bottom_cut")
        row.prop(p, "plate_recess")
        row = col.row(align=True)
        row.prop(p, "plate_inset")
        row.prop(p, "plate_clear")
        col.prop(p, "plate_ridge_h")
        col.separator()
        col.label(text="Screen module", icon="RESTRICT_VIEW_OFF")
        row = col.row(align=True)
        row.prop(p, "seat_dia")
        row.prop(p, "seat_depth")
        col.prop(p, "aperture_dia")
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


CLASSES = (OmniSculptProps, OMNI_OT_generate, OMNI_OT_build, OMNI_OT_shuffle,
           OMNI_OT_reload,
           OMNI_PT_main, OMNI_PT_ball, OMNI_PT_pads, OMNI_PT_coral,
           OMNI_PT_screen, OMNI_PT_ports, OMNI_PT_shell)


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
    plate_t = p.plate_recess + (p.plate_ridge_h if p.plate_ridge_h > 0.05 else 0.0)
    assert abs(pd.z - plate_t) < 0.05, "plate thickness (incl. key ridge) wrong"
    assert abs(shell.dimensions.z - (d.z - p.bottom_cut)) < 4.0, "shell height unexpected"

    # probes: cavity hollow, screen aperture open, seat floor solid, ledge solid
    fx = {ff["name"]: ff for ff in json.loads(obj["omniphone_facets"])}
    sc = fx["Screen"]
    dvec = np.array(sc["dir"])
    org = np.array(sc["origin"])
    tp = sc["plane_t"]
    assert not _point_inside(shell, org + dvec * (tp - 1.0)), "screen seat not open"
    assert not _point_inside(shell, org + dvec * (tp - p.seat_depth - p.wall - 3)), \
        "screen aperture not cut through"
    side = np.array([dvec[2], 0, -dvec[0]])
    side /= np.linalg.norm(side)
    assert _point_inside(shell, org + dvec * (tp - p.seat_depth - p.wall / 2) +
                         side * (p.seat_dia / 2 + 1.5)), "seat floor missing"
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
    print("SELFTEST PASS")


if __name__ == "__main__":
    try:
        unregister()
    except Exception:
        pass
    register()
    if "--selftest" in sys.argv:
        _selftest()
