# 3d-model — enclosures

Procedural generators for Omniphone enclosures, each paired with a live
browser viewer (three.js, vendored offline) that emits a copy-pasteable
parameter block:

| Generator | Viewer | Shape |
|-----------|--------|-------|
| [`thistle.py`](thistle.py) | [`viewer.html`](viewer.html) | thistle-flower bulb with phyllotaxis needles |
| [`thistle_lloyd.py`](thistle_lloyd.py) | [`viewer_lloyd.html`](viewer_lloyd.html) | same bulb, needles spread by **Lloyd's relaxation** (organic teasel/burr) |
| [`cymatics.py`](cymatics.py) | [`cymatics.html`](cymatics.html) | dome sculpted by a cymatics standing-wave field |
| [`mandelbulb.py`](mandelbulb.py) | [`mandelbulb.html`](mandelbulb.html) | simplified power-8 Mandelbulb fractal solid |

> **Note (venv):** the local `.venv` was built against Python 3.13; a system
> upgrade to 3.14 orphaned it, so `build123d` (used by `thistle.py` and
> `thistle_lloyd.py`) currently won't import. Rebuild it with `./setup.sh`
> before running those two. `cymatics.py` and `mandelbulb.py` are pure standard
> library and run under any `python3` with no venv.

---

## thistle-bulb enclosure

Procedural generator for the Omniphone thistle-flower enclosure: an ellipsoid
bulb with a flat, rounded base, a screen recess in the top, and N spikes
("needles") in a sunflower / golden-angle pattern. Each needle carries copper
tape for capacitive sensing.

## Setup (Fedora)

```bash
python3 -m venv ~/cad && source ~/cad/bin/activate
pip install build123d          # pulls OpenCASCADE bindings (~few hundred MB, once)
```

## Interactive preview (sliders, real-time)

Open [`viewer.html`](viewer.html) in a browser (double-click, or right-click →
"Open with Live Server" in VS Code). Drag the sliders to reshape the bulb and
needles in real time, orbit with the mouse, then hit **Copy params** and paste
the block into [`thistle.py`](thistle.py) to generate the real solid.

The preview is a fast, faithful approximation: needle cones simply *overlap* the
bulb instead of being boolean-unioned, and rim fillets aren't shown — those only
happen in the real build123d export. three.js is vendored in `vendor/` so it
works offline.

## Generate the real model

```bash
cd 3d-model
python thistle.py              # -> thistle.step + thistle.stl
```

## Files

| File | Use |
|------|-----|
| `thistle.step` | Open/edit in **Fusion 360** or **FreeCAD** — imports as an editable solid. Do shell/wall thickness, mounting bosses, and the exact screen bezel here. |
| `thistle.stl`  | Send straight to the slicer / printer. |

`sudo dnf install freecad` gives you a native STEP viewer on Fedora for quick
iteration before touching Fusion 360.

## Tuning

Every dimension is a variable at the top of [`thistle.py`](thistle.py):

- **Size / elongation** — `BULB_RADIUS`, `BULB_HEIGHT` (target envelope ~ 220 × 220 × 100 mm)
- **Flat cuts + rounding** — `TOP_CUT_FRAC`, `BOT_CUT_FRAC`, `RIM_FILLET_TOP`, `RIM_FILLET_BOT`
- **Screen** — `SCREEN_MODE` (`"bulb"` = recess in the top, `"needle"` = the screen
  rides on one of the phyllotaxis needles), `SCREEN_DIA`, `SCREEN_MARGIN` (ring of
  material around the screen), `SCREEN_DEPTH`, `SCREEN_NEEDLE_LEN`, and
  `SCREEN_NEEDLE_IDX` (which needle carries it, counted from the top; 0 = topmost).
  In `"needle"` mode the band automatically extends to the pole (`BAND_HIGH` is forced
  to 1 and greyed out in the viewer). `SCREEN_DIA = 0` disables the screen entirely.
- **Needles** — `N_NEEDLES`, `NEEDLE_BASE_DIA`, `NEEDLE_LEN`, `NEEDLE_TIP_DIA`, `BAND_LOW`, `BAND_HIGH`
- **Needle size ramp** — `NEEDLE_SIZE_START` / `NEEDLE_SIZE_END` scale each needle's
  diameter linearly from the first to the last (e.g. 1.0 → 1.6 makes them fatten toward the end).
- **Needle fillets** — flat tips with a settable edge fillet (`NEEDLE_TIP_FILLET`), a
  concave foot blending into the bulb (`NEEDLE_BASE_FILLET`), and a merge flare so
  neighbouring needles fuse when they touch (`NEEDLE_MERGE_FILLET`). `NEEDLE_EMBED`
  sets how far each foot protrudes into the bulb.

`viewer.html` shows the live W × D × H envelope and its number fields accept typed
values, so you can dial the enclosure straight to 220 × 220 × 100 mm.

## Notes / next steps

- The print is currently a **solid**. Shell it (hollow wall + electronics
  cavity) in Fusion after import, or add it to the script.
- Spikes point straight out along the surface normal; a real thistle sweeps
  them up toward the crown — a small tweak (blend normal with +Z) if wanted.
- Needle tips are now flat (with a settable `NEEDLE_TIP_FILLET`), so copper tape
  seats on a real pad. Consider a keep-out ring around the screen hole so no
  spike intrudes on the bezel.
- The needle-to-needle *merge* is geometric (flared, coved feet), not a true CAD
  fillet — OpenCASCADE won't fillet all ~200 mutual seams in one pass. For a crisp
  blend, import the STEP into Fusion 360 and run one Fillet/Blend on the fused base.

---

## thistle (Lloyd-relaxed) — organic spike spread

[`thistle_lloyd.py`](thistle_lloyd.py) is `thistle.py` with **one thing changed**:
the needles are positioned by **Lloyd's relaxation** (a centroidal Voronoi
relaxation) instead of the golden-angle phyllotaxis spiral. Random seed points on
the bulb's surface band are repeatedly nudged to the centroid of their Voronoi
cell, so they settle into an *even but non-lattice, organic* spacing — the look of
a real teasel / burr head rather than a machined sunflower. Everything else (bulb,
needle profile, fillets, screen options, STEP/STL export) is identical to
`thistle.py`, so re-read its tuning notes above; they all apply.

```bash
cd 3d-model
./setup.sh                     # first, if the venv is orphaned (see note at top)
.venv/bin/python thistle_lloyd.py   # -> thistle_lloyd.step + thistle_lloyd.stl
```

Extra parameters (on top of every `thistle.py` knob):

- **`RELAX_ITERS`** — number of relaxation passes. More = more even; `0` = the raw
  random start (clumpy). ~15+ is fully settled.
- **`RELAX_SEED`** — RNG seed. Change it to reshuffle into a *different* organic
  arrangement; the result is fully reproducible per seed.
- **`RELAX_SAMPLES`** — Monte-Carlo samples used to estimate each Voronoi centroid
  (higher = smoother relaxation, slower).

**Screen** is supported exactly as in `thistle.py`: `SCREEN_MODE = "bulb"` (recess
in the top) or `"needle"` (the screen rides the *top-most* relaxed spike).

*Boundary note:* Lloyd's relaxation naturally pulls points ~0.1 (in height
fraction) away from the band edges, so spikes don't quite reach `BAND_LOW`. Lower
`BAND_LOW` a touch if you want them further down.

---

## mandelbulb — simplified fractal solid

[`mandelbulb.py`](mandelbulb.py) generates the iconic **power-8 Mandelbulb** as a
watertight solid, taken at a **low iteration count** so it comes out as a smooth,
rounded, lobed blank you can actually print and machine features into — not an
un-printable, infinitely crenellated fractal. Pure standard library, no venv.

```bash
cd 3d-model
python3 mandelbulb.py          # -> mandelbulb.stl   (~3 s at GRID=96)
```

**Editing in Fusion 360.** The output is a **mesh** (the fractal has no clean BREP
form), so import `mandelbulb.stl` as a *mesh body*, then:

- add the **screen pocket** and **power/USB cutouts** as parametric solids and
  `Combine`/subtract them, or use the built-in `SCREEN_MODE = "bulb"` recess (see
  below) as a starting seat;
- `Mesh → Convert Mesh → Faceted/Prismatic` to get a BRep if you need one — or drop
  `GRID` (fewer triangles: 96 ≈ 193 k tris; 64 ≈ 85 k) to stay under Fusion's
  mesh-conversion limits;
- `Mesh → Reduce` also thins the triangle count for easier editing.

The surface is extracted with **marching tetrahedra**, which guarantees a
watertight, manifold mesh (the script prints a `WATERTIGHT` check on every run).

Key parameters:

- **`POWER`** — fractal order (8 = the classic Mandelbulb).
- **`ITERATIONS`** — escape depth. **Low = smooth simplified blob; high = fractal
  crust.** This is the main "simplify" dial.
- **`LEVEL`** — iso-surface threshold. Lower = puffier/rounder; higher (toward
  `ITERATIONS`) = tighter with more surface detail.
- **`GRID`** — samples per axis (64 quick, 96 good, 128+ fine/slow). Sets mesh
  density.
- **`SIZE_MM`** — scales the finished solid so its largest span is this many mm.
- **`BASE_FLATTEN`** — slice a fraction off the bottom for a flat, standable base.
- **`SMOOTH_ITERS`** — Laplacian smoothing passes to soften the tetra faceting.
- **Screen** — `SCREEN_MODE = "bulb"` + `SCREEN_DIA` / `SCREEN_DEPTH` carves a round
  flat recess into the top lobe (voxel-resolution — raise `GRID` for crisper walls,
  or true it up in Fusion). `"none"` leaves the raw bulb for you to cut the screen
  and connectors in Fusion, which is the more flexible route.

Dial `POWER` / `ITERATIONS` / `LEVEL` live in [`mandelbulb.html`](mandelbulb.html)
(a raymarched preview), then **Copy params** into the script.
