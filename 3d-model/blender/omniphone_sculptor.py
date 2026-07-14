# Omniphone Sculptor — Blender port, slice 1: body + brain-coral pads.
#
# Pipeline (this slice):  icosphere ball → squish to height → gradient-flatten
# the underside to a final height → seed N pad centres on the surface
# (phyllotaxis / random / Lloyd-relaxed) → grow a Gray-Scott reaction-diffusion
# field over the mesh graph, seeded at the pads → displace vertices along their
# normals (ridges out, grooves in), faded below a coverage line.
#
# Coming in later slices: screen platform (vertex attract), USB/jack mounts,
# flat bottom cut + rabbet lip, shell, PCB-holder insert + trim, bottom plate,
# M3 self-tap bosses.
#
# Install:  Edit ▸ Preferences ▸ Add-ons ▸ Install… → this file, enable it.
#           Panel lives in the 3D view sidebar (N key) → "Omniphone" tab.
# Dev:      edit file, then F3 → "Reload Scripts" (or re-run in Text Editor).
# Headless: blender -b --factory-startup --python-exit-code 1 \
#               --python omniphone_sculptor.py -- --selftest
#
# Everything is millimetres; generate() sets the scene units to match.

bl_info = {
    "name": "Omniphone Sculptor",
    "author": "Omniphone project",
    "version": (0, 1, 0),
    "blender": (4, 2, 0),
    "location": "3D View ▸ Sidebar ▸ Omniphone",
    "description": "Generative Omniphone enclosure bodies (brain-coral slice)",
    "category": "Add Mesh",
}

import json
import sys
import time
import random as pyrandom
from math import ceil, pi, sqrt

import bpy
import bmesh
import numpy as np
from bpy.props import EnumProperty, FloatProperty, IntProperty, PointerProperty

GOLDEN_ANGLE = pi * (3.0 - sqrt(5.0))
BODY_NAME = "Omniphone"
COLL_NAME = "Omniphone"
PADS_COLL_NAME = "Omniphone Pads"


# ---------------------------------------------------------------------------
# numpy geometry core (no bpy below this line except make_icosphere)
# ---------------------------------------------------------------------------

def make_icosphere(subdiv, radius):
    """Icosphere as (verts float64 (n,3), faces int64 (m,3)). bmesh only as
    a generator; all downstream work is numpy."""
    bm = bmesh.new()
    bmesh.ops.create_icosphere(bm, subdivisions=int(subdiv), radius=float(radius))
    bm.verts.ensure_lookup_table()
    verts = np.array([v.co[:] for v in bm.verts], dtype=np.float64)
    faces = np.array([[v.index for v in f.verts] for f in bm.faces], dtype=np.int64)
    bm.free()
    return verts, faces


def neighbor_table(n, faces):
    """Padded neighbour indices for fast vectorised graph ops.
    Returns (nbr (n,maxdeg) int64 — padded with self, deg (n,) f32,
    pad (n,) f32 = number of self-paddings per row)."""
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
    """Mean over true neighbours (self-padding contributes f itself, so it is
    subtracted back out via pad counts)."""
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


def squish_and_flatten(sphere_verts, diameter, squish_h, final_h, zone):
    """Scale z so the ball is squish_h tall, then compress everything below a
    blend line with a monotone Hermite map so the body ends final_h tall with
    a progressively flatter underside (slope→0 at the old pole ⇒ the bottom
    cap collapses toward one plane — the 'gradient select and flatten').
    Output is shifted so the underside rests at z = 0."""
    v = sphere_verts.copy()
    v[:, 2] *= squish_h / diameter
    z = v[:, 2]
    z_min = -squish_h / 2.0
    z_floor = squish_h / 2.0 - final_h
    if z_floor <= z_min + 1e-6:                       # final_h ≥ squish_h: nothing to flatten
        v[:, 2] = z - z_min
        return v
    z_blend = min(z_floor + zone, squish_h * 0.45)     # keep the blend below the equator
    span_in, span_out = z_blend - z_min, z_blend - z_floor
    m1 = min(span_in, 3.0 * span_out)  # Fritsch–Carlson clamp keeps the map monotone;
    #                                    when it kicks in there is a slight crease at the
    #                                    blend line (only at extreme flatten + tiny zone)
    t = np.clip((z - z_min) / span_in, 0.0, 1.0)
    zH = (2 * t**3 - 3 * t**2 + 1) * z_floor + (-2 * t**3 + 3 * t**2) * z_blend + (t**3 - t**2) * m1
    v[:, 2] = np.where(z < z_blend, zH, z) - z_floor
    return v


def pad_layout(p, rng):
    """N unit directions on the sphere band [band_lo, band_hi] (z of the unit
    sphere). Layouts: golden-angle phyllotaxis, uniform random (uniform in z =
    uniform per area), or random + repulsion relaxation ('Lloyd', same organic
    spread as sculptor v2's padDirs)."""
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
        d = dirs[:, None, :] - dirs[None, :, :]
        d2 = (d * d).sum(-1) + 1e-6
        w = 1.0 / (d2 * np.sqrt(d2))
        np.fill_diagonal(w, 0.0)
        dirs = dirs + (d * w[:, :, None]).sum(axis=1) * (0.35 / n)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        z = np.clip(dirs[:, 2], lo, hi)
        r = np.sqrt(np.clip(1 - z * z, 0, None))
        xy = np.linalg.norm(dirs[:, :2], axis=1)
        xy[xy < 1e-9] = 1.0
        dirs = np.stack([dirs[:, 0] / xy * r, dirs[:, 1] / xy * r, z], axis=1)
    return dirs


def seed_pads(body_verts, unit_dirs, dirs, bulb_r):
    """Map each layout direction to its nearest mesh vertex (angular nearest on
    the original sphere — stable across squish/flatten), then mark a disc of
    bulb_r mm (euclidean, on the shaped body) around it as RD seed."""
    mask = np.zeros(len(body_verts), dtype=bool)
    centers = np.empty(len(dirs), dtype=np.int64)
    for k, d in enumerate(dirs):
        i = int(np.argmax(unit_dirs @ d))
        centers[k] = i
        c = body_verts[i]
        mask |= ((body_verts - c) ** 2).sum(axis=1) < bulb_r * bulb_r
    return mask, centers


def gray_scott_mesh(nbr, deg, pad, seed_mask, p, rng, log=print):
    """Gray-Scott on the mesh graph. The Laplacian is 4·(neighbour-mean − u),
    which reduces to the classic 4-neighbour grid stencil on a regular lattice,
    so the familiar F/k parameter space (coral 0.055/0.062 …) carries over.
    Pattern scale s runs diffusion at s² with ⌈s²⌉ substeps to stay stable —
    wavelength ∝ √D, cost ∝ s²."""
    n = len(deg)
    U = np.ones(n, dtype=np.float32)
    V = np.zeros(n, dtype=np.float32)
    if p.rd_noise > 0:
        V += (p.rd_noise * rng.random(n)).astype(np.float32)
    U[seed_mask] = 0.25
    V[seed_mask] = 0.5
    scale = float(p.rd_scale)
    nsub = max(1, int(ceil(scale * scale)))
    dt = np.float32(1.0 / nsub)
    Du4 = np.float32(4.0 * 0.16 * scale * scale)
    Dv4 = np.float32(4.0 * 0.08 * scale * scale)
    F = np.float32(p.rd_feed)
    k = np.float32(p.rd_kill)
    one = np.float32(1.0)
    steps = int(p.rd_steps)
    t0 = time.time()
    for s in range(steps):
        for _ in range(nsub):
            lu = (U[nbr].sum(axis=1) - pad * U) / deg - U
            lv = (V[nbr].sum(axis=1) - pad * V) / deg - V
            uvv = U * V * V
            U += dt * (Du4 * lu - uvv + F * (one - U))
            V += dt * (Dv4 * lv + uvv - (F + k) * V)
        if (s + 1) % 500 == 0:
            log(f"  RD {s + 1}/{steps}  ({time.time() - t0:.1f}s)")
    return V


def coral_displace(verts, faces, field, nbr, deg, pad, p, final_h):
    """Normalise the RD field to [0,1], optionally smooth it on the graph, then
    push vertices out on ridges (field>0.5) and in on grooves, faded to zero
    below the coverage line (coverage = fraction of body height, from the top)."""
    f01 = field.astype(np.float64)
    span = f01.max() - f01.min()
    f01 = (f01 - f01.min()) / (span if span > 1e-9 else 1.0)
    for _ in range(int(p.field_smooth)):
        f01 = 0.5 * f01 + 0.5 * nb_mean(f01, nbr, deg, pad)
    d = (f01 - 0.5) * 2.0
    amp = np.where(d > 0, d * p.ridge_height, d * p.groove_depth)
    if p.coverage < 0.999:
        z_thr = final_h * (1.0 - p.coverage)
        amp *= smoothstep(z_thr, z_thr + p.cover_fade, verts[:, 2])
    return verts + vertex_normals(verts, faces) * amp[:, None]


def generate_body(p, log=print):
    """Full slice-1 pipeline. Returns (verts, faces, pad_center_indices, warnings)."""
    t0 = time.time()
    rng = np.random.default_rng(int(p.rng_seed))
    warnings = []

    radius = p.diameter / 2.0
    sphere, faces = make_icosphere(p.resolution, radius)
    unit_dirs = sphere / radius
    log(f"  ball: {len(sphere)} verts, {len(faces)} faces")

    final_h = min(p.final_height, p.squish_height)
    if p.final_height > p.squish_height:
        warnings.append("Final height > squish height — clamped to squish height")
    verts = squish_and_flatten(sphere, p.diameter, p.squish_height, final_h, p.flatten_zone)

    nbr, deg, pad = neighbor_table(len(verts), faces)
    dirs = pad_layout(p, rng)
    if len(dirs):
        seed_mask, centers = seed_pads(verts, unit_dirs, dirs, p.seed_bulb_r)
    else:
        seed_mask, centers = np.zeros(len(verts), dtype=bool), np.empty(0, dtype=np.int64)
    if len(dirs) == 0 and p.rd_noise <= 0:
        warnings.append("0 pads and 0 background noise → coral has nothing to grow from")

    log(f"  RD: {int(p.rd_steps)} steps × {max(1, int(ceil(p.rd_scale**2)))} substeps "
        f"on {len(verts)} verts…")
    field = gray_scott_mesh(nbr, deg, pad, seed_mask, p, rng, log=log)
    out_verts = coral_displace(verts, faces, field, nbr, deg, pad, p, final_h)

    if len(centers) and p.coverage < 0.999:
        z_thr = final_h * (1.0 - p.coverage)
        low = int((verts[centers, 2] < z_thr).sum())
        if low:
            warnings.append(f"{low} pad(s) sit below the coral coverage line — "
                            "raise Band low or Coverage")
    log(f"  done in {time.time() - t0:.1f}s")
    return out_verts, faces, centers, warnings


# ---------------------------------------------------------------------------
# Blender glue
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


def replace_body_object(verts, faces, coll):
    me = bpy.data.meshes.new(BODY_NAME)
    me.from_pydata(verts.tolist(), [], faces.tolist())
    me.validate()
    me.update()
    me.polygons.foreach_set("use_smooth", np.ones(len(me.polygons), dtype=bool))
    obj = bpy.data.objects.get(BODY_NAME)
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


def rebuild_pad_empties(coll, positions, size):
    for o in list(coll.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for i, pos in enumerate(positions):
        e = bpy.data.objects.new(f"Pad.{i:02d}", None)
        e.empty_display_type = "SPHERE"
        e.empty_display_size = size
        e.location = pos
        coll.objects.link(e)


def props_as_dict(p):
    out = {}
    for name in p.__class__.__annotations__.keys():
        v = getattr(p, name)
        out[name] = round(v, 6) if isinstance(v, float) else v
    return out


# ---------------------------------------------------------------------------
# Properties / operators / panel
# ---------------------------------------------------------------------------

FK_HINT = "Classic pairs — coral 0.055/0.062 · spots 0.030/0.062 · maze 0.029/0.057 · mitosis 0.0367/0.0649"


class OmniSculptProps(bpy.types.PropertyGroup):
    # 1 · ball
    diameter: FloatProperty(name="Diameter", default=220.0, min=60.0, max=320.0,
                            description="Ball diameter (mm)")
    squish_height: FloatProperty(name="Squish to", default=120.0, min=40.0, max=280.0,
                                 description="Height after squashing the ball (mm)")
    resolution: IntProperty(name="Mesh detail", default=7, min=5, max=8,
                            description="Icosphere subdivisions: 5 ≈ 2.6k verts, 6 ≈ 10k, "
                                        "7 ≈ 41k (~1.8 mm edges), 8 ≈ 164k (4× slower, final export)")
    # 2 · bottom flatten
    final_height: FloatProperty(name="Final height", default=100.0, min=30.0, max=280.0,
                                description="Total height after gradient-flattening the underside (mm)")
    flatten_zone: FloatProperty(name="Blend zone", default=30.0, min=5.0, max=80.0,
                                description="How far above the floor the flattening blends out (mm)")
    # 3 · pad seeds
    pad_count: IntProperty(name="Pad count", default=12, min=0, max=64,
                           description="Number of touch pads = coral seed bulbs")
    pad_layout: EnumProperty(name="Layout", default="PHYLLO", items=[
        ("PHYLLO", "Phyllotaxis", "Golden-angle spiral over the band"),
        ("RANDOM", "Random", "Uniform random over the band"),
        ("LLOYD", "Lloyd (relaxed)", "Random start + repulsion relaxation "
                                     "(uses at least 15 relax iterations)")])
    relax_iters: IntProperty(name="Relax", default=0, min=0, max=60,
                             description="Repulsion-relaxation iterations applied to the layout "
                                         "(evens out spacing while staying organic)")
    band_lo: FloatProperty(name="Band low", default=0.05, min=-0.6, max=0.9,
                           description="Lowest pad position (z on the unit ball: 1 = top pole, 0 = equator)")
    band_hi: FloatProperty(name="Band high", default=0.92, min=0.0, max=1.0,
                           description="Highest pad position (keep < 1.0 to leave the crown free)")
    seed_bulb_r: FloatProperty(name="Seed bulb Ø/2", default=5.0, min=2.0, max=15.0,
                               description="Radius of the RD seed disc at each pad centre (mm)")
    rng_seed: IntProperty(name="Random seed", default=3, min=0, max=99999,
                          description="Every seed is a different instrument")
    # 4 · brain coral
    rd_feed: FloatProperty(name="Feed F", default=0.055, min=0.02, max=0.09,
                           precision=4, step=0.05, description=FK_HINT)
    rd_kill: FloatProperty(name="Kill k", default=0.062, min=0.03, max=0.075,
                           precision=4, step=0.05, description=FK_HINT)
    rd_steps: IntProperty(name="Sim steps", default=2600, min=200, max=6000,
                          description="More steps = pattern spreads further from the seeds "
                                      "and matures (slower)")
    rd_scale: FloatProperty(name="Pattern scale", default=0.75, min=0.5, max=2.5,
                            description="Ridge wavelength multiplier (cost grows with scale²)")
    rd_noise: FloatProperty(name="Background noise", default=0.0, min=0.0, max=0.1,
                            precision=3, step=0.5,
                            description="0 = pattern grows only from the pad seeds; "
                                        ">0 = spontaneous coral everywhere")
    field_smooth: IntProperty(name="Field smoothing", default=1, min=0, max=10,
                              description="Graph-smoothing passes on the RD field before displacing "
                                          "(more = softer, meltier ridges)")
    ridge_height: FloatProperty(name="Ridge height", default=3.5, min=0.0, max=15.0,
                                description="How far ridges stick out of the ball (mm)")
    groove_depth: FloatProperty(name="Groove depth", default=3.5, min=0.0, max=15.0,
                                description="How deep grooves cut into the ball (mm)")
    coverage: FloatProperty(name="Coverage", default=1.0, min=0.05, max=1.0, subtype="FACTOR",
                            description="Fraction of the body height (from the top) the coral covers — "
                                        "0.2 = only the top 20% is brain")
    cover_fade: FloatProperty(name="Coverage fade", default=12.0, min=2.0, max=40.0,
                              description="Width of the smooth fade at the coverage line (mm)")


class OMNI_OT_generate(bpy.types.Operator):
    bl_idname = "omni_sculpt.generate"
    bl_label = "Generate Omniphone"
    bl_description = "Run the pipeline and (re)build the Omniphone body"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        p = context.scene.omni_sculpt
        t0 = time.time()
        try:
            verts, faces, centers, warnings = generate_body(p, log=print)
        except Exception as exc:
            self.report({"ERROR"}, f"generation failed: {exc}")
            raise
        set_units_mm(context.scene)
        coll = ensure_collection(COLL_NAME)
        obj = replace_body_object(verts, faces, coll)
        pads_coll = ensure_collection(PADS_COLL_NAME, parent=coll)
        rebuild_pad_empties(pads_coll, verts[centers], p.seed_bulb_r)
        obj["omniphone_params"] = json.dumps(props_as_dict(p))
        obj["omniphone_pads"] = json.dumps([list(map(float, verts[i])) for i in centers])
        for o in context.view_layer.objects:
            o.select_set(o is obj)
        context.view_layer.objects.active = obj
        for w in warnings:
            self.report({"WARNING"}, w)
        d = obj.dimensions
        self.report({"INFO"}, f"{len(verts)} verts · {d.x:.0f}×{d.y:.0f}×{d.z:.0f} mm · "
                              f"{time.time() - t0:.1f}s")
        return {"FINISHED"}


class OMNI_OT_shuffle(bpy.types.Operator):
    bl_idname = "omni_sculpt.shuffle"
    bl_label = "Shuffle"
    bl_description = "New random seed + regenerate (a different instrument every press)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        context.scene.omni_sculpt.rng_seed = pyrandom.randint(0, 99999)
        return bpy.ops.omni_sculpt.generate()


class OMNI_PT_panel(bpy.types.Panel):
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

        box = col.box()
        box.label(text="1 · Ball", icon="MESH_UVSPHERE")
        box.prop(p, "diameter")
        box.prop(p, "squish_height")
        box.prop(p, "resolution")

        box = col.box()
        box.label(text="2 · Bottom flatten", icon="AXIS_TOP")
        box.prop(p, "final_height")
        box.prop(p, "flatten_zone")

        box = col.box()
        box.label(text="3 · Pad seeds", icon="PROP_ON")
        box.prop(p, "pad_count")
        box.prop(p, "pad_layout")
        box.prop(p, "relax_iters")
        row = box.row(align=True)
        row.prop(p, "band_lo")
        row.prop(p, "band_hi")
        box.prop(p, "seed_bulb_r")
        box.prop(p, "rng_seed")

        box = col.box()
        box.label(text="4 · Brain coral", icon="RNDCURVE")
        row = box.row(align=True)
        row.prop(p, "rd_feed")
        row.prop(p, "rd_kill")
        box.prop(p, "rd_steps")
        box.prop(p, "rd_scale")
        box.prop(p, "rd_noise")
        box.prop(p, "field_smooth")
        row = box.row(align=True)
        row.prop(p, "ridge_height")
        row.prop(p, "groove_depth")
        box.prop(p, "coverage")
        box.prop(p, "cover_fade")

        col.label(text="Slice 1 — screen, ports, shell & plate come next.", icon="INFO")


CLASSES = (OmniSculptProps, OMNI_OT_generate, OMNI_OT_shuffle, OMNI_PT_panel)


def register():
    for c in CLASSES:
        bpy.utils.register_class(c)
    bpy.types.Scene.omni_sculpt = PointerProperty(type=OmniSculptProps)


def unregister():
    del bpy.types.Scene.omni_sculpt
    for c in reversed(CLASSES):
        bpy.utils.unregister_class(c)


# ---------------------------------------------------------------------------
# headless selftest
# ---------------------------------------------------------------------------

def _selftest():
    import os
    scene = bpy.context.scene
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
    p = scene.omni_sculpt
    n_pads = len(bpy.data.collections[PADS_COLL_NAME].objects)
    print(f"SELFTEST verts={len(me.vertices)} faces={len(me.polygons)} closed={closed} "
          f"dims={d.x:.1f}x{d.y:.1f}x{d.z:.1f} pads={n_pads}")
    assert closed, "mesh is not a closed 2-manifold"
    # top pole may land on a ridge (+) or in a groove (−); bottom is pinned at 0
    assert p.final_height - p.groove_depth - 1 < d.z < p.final_height + p.ridge_height + 1, \
        f"unexpected height {d.z:.1f}"
    assert n_pads == p.pad_count, "pad empty count mismatch"

    out = os.path.join(os.getcwd(), "omniphone_selftest.stl")
    for o in bpy.context.view_layer.objects:
        o.select_set(o is obj)
    bpy.context.view_layer.objects.active = obj
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
