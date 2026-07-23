# make_omniphone_template.py — build the "Omniphone" Blender app template.
#
# Produces a startup.blend that File ▸ New ▸ Omniphone loads: mm units, a
# representative generated body, a 3D-print material, and a tabletop render
# rig (ground + key/fill lights + framed camera). Also mm-friendly viewport
# clipping and 1 mm-increment snapping.
#
# Run headless:
#   blender -b --factory-startup --python make_omniphone_template.py
#     [-- --preview]          # also render a preview PNG, skip install
#     [-- --install]          # write into ~/.config/blender/<ver>/scripts/...
#
# The generator is the source of truth; re-run to regenerate the template.

import os
import sys
import math

import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

TEMPLATE_NAME = "Omniphone"
SCRATCH = "/tmp/claude-1000/-home-memo-Documents-code-Omniphone/scratchpad"


def _argv_after_ddash():
    return sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


# --------------------------------------------------------------------------- #
# scene construction
# --------------------------------------------------------------------------- #

def generate_body():
    """Register the sculptor and run a default generate. Returns the body obj."""
    import omniphone_sculptor as omni
    try:
        omni.register()
    except Exception:
        pass  # already registered
    scene = bpy.context.scene
    res = bpy.ops.omni_sculpt.generate()
    assert res == {"FINISHED"}, f"generate returned {res}"
    omni.set_units_mm(scene)
    return bpy.data.objects[omni.BODY_NAME]


def clear_starter_junk():
    """Drop the factory cube; keep nothing else we don't place ourselves."""
    for name in ("Cube",):
        o = bpy.data.objects.get(name)
        if o is not None:
            bpy.data.objects.remove(o, do_unlink=True)
    # factory camera/light get replaced by ours below
    for o in [o for o in bpy.data.objects if o.type in {"CAMERA", "LIGHT"}]:
        bpy.data.objects.remove(o, do_unlink=True)


def bbox_world(obj):
    """World-space min/center/half-diagonal of an object's bounding box."""
    corners = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    mn = Vector((min(c.x for c in corners),
                 min(c.y for c in corners),
                 min(c.z for c in corners)))
    mx = Vector((max(c.x for c in corners),
                 max(c.y for c in corners),
                 max(c.z for c in corners)))
    center = (mn + mx) * 0.5
    radius = (mx - mn).length * 0.5
    return mn, center, mx, radius


def print_material():
    """Matte prototype-print plastic: light warm gray, mid roughness, a whisper
    of subsurface so it reads as filament/resin rather than hard plastic."""
    mat = bpy.data.materials.new("Omniphone Print")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (0.62, 0.60, 0.56, 1.0)
    bsdf.inputs["Roughness"].default_value = 0.58
    # Blender 4.x renamed specular; set whichever exists.
    for key in ("Specular IOR Level", "Specular"):
        if key in bsdf.inputs:
            bsdf.inputs[key].default_value = 0.35
            break
    for key in ("Subsurface Weight", "Subsurface"):
        if key in bsdf.inputs:
            bsdf.inputs[key].default_value = 0.08
            break
    if "Subsurface Radius" in bsdf.inputs:
        bsdf.inputs["Subsurface Radius"].default_value = (2.0, 1.6, 1.2)
    return mat


def table_material():
    """Soft neutral tabletop — slightly darker than the print so the object pops."""
    mat = bpy.data.materials.new("Tabletop")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (0.20, 0.20, 0.22, 1.0)
    bsdf.inputs["Roughness"].default_value = 0.85
    return mat


def add_ground(z, span):
    bpy.ops.mesh.primitive_plane_add(size=span, location=(0, 0, z))
    plane = bpy.context.active_object
    plane.name = "Tabletop"
    plane.data.materials.append(table_material())
    return plane


def add_lights(center, radius):
    """Key + fill soft area lights for a tabletop-studio look. Everything is in
    raw Blender units where the body is ~radius BU (scale_length=0.001). Area
    power must scale with distance² — hence energies keyed off R²."""
    def area(name, loc, size, power, warm=1.0):
        d = bpy.data.lights.new(name, "AREA")
        d.shape = "DISK"
        d.size = size
        d.energy = power
        d.color = (1.0, warm, warm * 0.9)
        o = bpy.data.objects.new(name, d)
        bpy.context.collection.objects.link(o)
        o.location = loc
        direction = (center - Vector(loc))
        o.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
        return o

    R = radius
    R2 = R * R
    area("Key",  (center.x + 1.6 * R, center.y - 2.2 * R, center.z + 2.6 * R),
         size=2.5 * R, power=260.0 * R2, warm=0.96)
    area("Fill", (center.x - 2.4 * R, center.y - 1.2 * R, center.z + 1.2 * R),
         size=3.5 * R, power=90.0 * R2, warm=1.0)
    area("Rim",  (center.x - 0.6 * R, center.y + 2.6 * R, center.z + 2.0 * R),
         size=2.0 * R, power=140.0 * R2, warm=1.02)


def add_camera(center, radius):
    cam_data = bpy.data.cameras.new("Camera")
    cam_data.lens = 65.0                      # gentle compression, product-shot feel
    cam_data.clip_start = 0.01                # 0.01 mm
    cam_data.clip_end = radius * 40.0
    cam = bpy.data.objects.new("Camera", cam_data)
    bpy.context.collection.objects.link(cam)
    view_dir = Vector((0.75, -1.0, 0.5)).normalized()
    cam.location = center + view_dir * (radius * 4.0)
    look = (center - cam.location)
    cam.rotation_euler = look.to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def setup_world():
    world = bpy.context.scene.world or bpy.data.worlds.new("World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs["Color"].default_value = (0.05, 0.05, 0.06, 1.0)
        bg.inputs["Strength"].default_value = 0.6


def setup_render():
    scene = bpy.context.scene
    for eng in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE"):
        try:
            scene.render.engine = eng
            break
        except TypeError:
            continue
    scene.render.resolution_x = 1200
    scene.render.resolution_y = 1200
    scene.render.film_transparent = False
    ee = getattr(scene, "eevee", None)
    if ee is not None:
        for attr, val in (("use_shadows", True), ("use_raytracing", True),
                          ("use_gtao", True), ("taa_render_samples", 64)):
            if hasattr(ee, attr):
                setattr(ee, attr, val)


def setup_snapping():
    ts = bpy.context.scene.tool_settings
    for attr, val in (("use_snap", True),
                      ("use_snap_grid_absolute", True)):
        try:
            setattr(ts, attr, val)
        except Exception:
            pass
    for attr in ("snap_elements", "snap_elements_base"):
        try:
            setattr(ts, attr, {"INCREMENT"})
        except Exception:
            pass


def setup_viewports(center, radius):
    """mm-scale clipping, shaded viewport, and a framed 3/4 orbit in every
    saved 3D view so File ▸ New ▸ Omniphone opens looking right at the body."""
    view_dir = Vector((0.75, -1.0, 0.5)).normalized()
    view_rot = (center - (center + view_dir)).to_track_quat("-Z", "Y")
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type != "VIEW_3D":
                continue
            for space in area.spaces:
                if space.type != "VIEW_3D":
                    continue
                space.clip_start = 0.01
                space.clip_end = 100000.0
                try:
                    space.shading.type = "MATERIAL"
                except Exception:
                    pass
                r3d = getattr(space, "region_3d", None)
                if r3d is not None:
                    r3d.view_perspective = "PERSP"
                    r3d.view_location = center
                    r3d.view_distance = radius * 3.2
                    r3d.view_rotation = view_rot


# --------------------------------------------------------------------------- #
# install
# --------------------------------------------------------------------------- #

INIT_PY = '''\
# Omniphone application template. Enables the sculptor add-on so the body
# generated in this file stays live-editable (3D View ▸ Sidebar ▸ Omniphone).
import addon_utils


def register():
    try:
        addon_utils.enable("omniphone_sculptor", default_set=False,
                            persistent=True)
    except Exception:
        pass


def unregister():
    pass
'''


def template_dir():
    ver = "%d.%d" % (bpy.app.version[0], bpy.app.version[1])
    base = os.path.expanduser("~/.config/blender/%s/scripts/startup"
                              "/bl_app_templates_user/%s" % (ver, TEMPLATE_NAME))
    return base


def install(save_blend):
    d = template_dir()
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "__init__.py"), "w") as f:
        f.write(INIT_PY)
    dst = os.path.join(d, "startup.blend")
    save_blend(dst)
    print("[omniphone-template] installed →", d)
    return d


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def build():
    body = generate_body()
    clear_starter_junk()
    body.data.materials.clear()
    body.data.materials.append(print_material())

    mn, center, mx, radius = bbox_world(body)
    add_ground(mn.z, span=radius * 12.0)
    add_lights(center, radius)
    add_camera(center, radius)
    setup_world()
    setup_render()
    setup_snapping()
    setup_viewports(center, radius)
    return body, center, radius


def save_startup(path):
    bpy.ops.wm.save_as_mainfile(filepath=path, compress=True)


def main():
    args = _argv_after_ddash()
    build()

    if "--preview" in args or not ("--install" in args):
        os.makedirs(SCRATCH, exist_ok=True)
        bpy.context.scene.render.filepath = os.path.join(SCRATCH, "template_preview.png")
        bpy.ops.render.render(write_still=True)
        print("[omniphone-template] preview →", bpy.context.scene.render.filepath)

    if "--install" in args:
        install(save_startup)
    else:
        # stash the blend in scratch so we can inspect/iterate before installing
        os.makedirs(SCRATCH, exist_ok=True)
        save_startup(os.path.join(SCRATCH, "omniphone_startup.blend"))
        print("[omniphone-template] blend →",
              os.path.join(SCRATCH, "omniphone_startup.blend"))


if __name__ == "__main__":
    main()
