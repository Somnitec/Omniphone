# 3d-model — enclosures

Procedural generators for Omniphone enclosures, each paired with a live
browser viewer (three.js, vendored offline) that emits a copy-pasteable
parameter block:

> **Blender port (in progress):** [`blender/omniphone_sculptor.py`](blender/omniphone_sculptor.py)
> is moving the *whole* modelling workflow into Blender (replacing the
> sculptor.html → Fusion 360 hand-off). Done so far: ball → squish →
> gradient-flatten underside → **screen platform + USB/jack panel flats**
> (disc attractors: settable tilt/azimuth/Ø/height, merge reach & strength;
> circle-empty markers carry the axes for the aperture slice) → N pad seeds
> (phyllotaxis / random / Lloyd, repelled from the screen cone) → Gray-Scott
> brain coral grown **on the mesh graph** (no pole distortion), ridges out /
> grooves in, coverage fade, auto-masked off the facets. Watertight, mm units.
>
> - **Live editing:** shape/displacement sliders update **while dragging**
>   (~5 ms; the coral field rides along stably). Pad/coral/RD changes make the
>   pattern visibly **re-grow in the viewport** (~5 fps chunks, settling in a
>   couple of seconds at the exact final state). Toggle with *Auto update*;
>   *Generate* always forces a full clean rebuild.
> - **Facet orientation:** screen/port discs lie **tangent to the smooth body
>   surface** by default (tilt/azimuth choose *where*, like a spoke); a *Pitch*
>   slider rotates the face off the tangent in the azimuth plane.
> - **Build step (slice 3):** the **Build Shell + Plate** button (section 6)
>   runs the engineering pass on the current body: flat bottom cut → plate
>   recess (2 mm in / 4 mm inset rabbet, solid lip) → ~3 mm hollow shell →
>   1.28″ screen counterbore + through-aperture, USB/jack holes → 3× M3
>   self-tap bosses on solid pads → a countersunk bottom plate that drops in
>   flush, with a **key ridge** tracing the lip/boss-pad profile so it can't
>   slide or rotate. ~30 s; output is two closed manifolds, checked per boolean
>   step in the console. The screen gets the real 1.28″ pocket: Ø36 aperture
>   lip (0.5) → Ø39 drop-in pocket with a truncated tab-corner cutout (starts
>   2.5 mm behind the lip, capped at the platform edge so it never holes the
>   visible shoulder) + four heat-stake prongs at the square-tangent points.
>   **Plate outfit** (panel 7, live ~0.3 s once a shell exists): dome legs
>   (count/Ø/height/splay/radius, flat feet sized for stick-on pads), a
>   speaker bay (recess + through-firing opening w/ rim), transducer glue zone
>   w/ optional thinning, PAM8403 recess + melt posts, DPST switch hole + Ø1
>   melt rivets @15 mm, battery corral (walls + strap gaps — no prongs against
>   a LiPo). The speaker bay is **round by default** (recess + through-opening
>   + meltable lock ridge around the rim) with the rectangular box bay as the
>   alternate type. Pad layouts gained **Even (max spacing)** — relaxation run
>   to convergence → most-equal pad sizes; pad count goes to 96.
>   **Layer stack** (three stackable passes, any combination): panel 1 picks
>   the **Form** — *Squished ball* · *Puffed Voronoi cells* (one cell per pad,
>   Even layout ⇒ equal pads) · *Crystal/faceted* (every pad becomes a flat
>   facet + a crown facet under the screen; filler facets close the underside)
>   · *Cymatics dome* (Bessel standing wave) · *Mandelbulb* (radially-captured
>   fractal, ~1 s per slider change, cached otherwise). Panel 2 = pad count &
>   layout (greyed out when nothing uses pads). Panel 3 = the form's settings.
>   Panel 4 = the **Texture pass**: a second algorithm displaced over the
>   shaped surface with its own feature count/seed/amount, decoupled from the
>   pads — Voronoi cells (grooves scale with cell size), Crystal facets (a
>   *gem grinder*: the crest inside each texture cell is cut flat — sharp
>   planar chips), Cymatics ripple, or Mandelbulb crust. Panel 5 = **Brain
>   coral** (header toggle), the RD relief grown on top; pads are its seeds.
>   So *cells + crystal texture*, *cymatics + coral*, *coral + small cells*,
>   or the same algorithm twice at two scales all compose. Only the coral
>   costs sim time — when it's off, no RD ever runs and everything is instant.
>   If the screen cutter `blender/screencutout.stl` exists it is
>   used verbatim (scaled to mm, centred on its top-lip circle, depth/bore/tab
>   re-measured from the file on every build); prongs and boss stay parametric.
>   [`screen_pocket_test.stl`](screen_pocket_test.stl) is a cut-out puck of
>   just the screen pocket for fit-testing (print it screen-face down).
>   ⚠ Don't reinstall the addon via the Add-ons menu — Install *copies* the
>   file, replacing the repo symlink, and edits stop arriving. Use the 🔌
>   Reload button (or restore the symlink). *Bottom cut* is mm above the resting plane (z=0) and
>   shows as a live wire circle in the viewport while you drag. Port *Pitch* is
>   absolute (0° = face perpendicular to the ground); the screen's is relative
>   to the surface tangent. Coral changes compute in the background and show
>   the finished shape by default — flip *Grow in viewport* on to watch the
>   pattern grow instead. Ridge height / groove depth accept negative values
>   (dented ridges, bulging grooves). Implementation notes: the offset cavity is voxel-remeshed
>   (its self-intersections in tight concave regions otherwise poison every
>   later boolean), the lip is carved *out of the cavity tool* so the delicate
>   underside resolves in a single subtraction, and everything runs on 4.5's
>   manifold solver (the exact solver misclassified the large hollowed shell).
> - **Install:** Blender ≥ 4.2 (4.5+ recommended for the manifold solver) →
>   Edit ▸ Preferences ▸ Add-ons ▸ Install… → pick the file → enable. Panel:
>   3D view sidebar (N) → **Omniphone** tab. Hit **Generate Omniphone**; the
>   🔄 button rolls a new random instrument. The installed addon is a symlink
>   to this repo file; after edits press the 🔌 **Reload** button in the panel
>   header — it re-reads the file and re-registers in place (slider values
>   survive). More reliable than F3 → "Reload Scripts" in 4.5.
> - **Headless check:** `blender -b --factory-startup --python-exit-code 1
>   --python blender/omniphone_sculptor.py -- --selftest`
> - **Roadmap** (next slices): parametric PCB-holder placeholder trimmed to the
>   shell interior; pad flat seats/recesses; more pad algorithms after coral.
> - **Known rough edges:** port flats low on the body sit on the rounded floor
>   rim and look grafted-on — keep `height − Ø/2 ≳ 8 mm` (Generate warns) and
>   tune Merge reach/strength by eye. Wall thickness is `wall` ± half the
>   0.8 mm cavity voxel. The USB hole is a plain round hole — size it to the
>   panel-mount part you pick.

> **Start here for exploring shapes:** [`sculptor.html`](sculptor.html) is a
> unified playground — one page, ~24 shapes in a dropdown grouped by family:
> **Classic** (thistle, Lloyd-relaxed thistle, cymatics, mandelbulb),
> **Coral** (brain-coral/RD, bubble, cauliflower, great-star),
> **Cactus**, **Mushroom**, **Fractals** (mandelbox, quaternion-Julia,
> juliabulb, Menger sponge, Sierpiński tetra, Apollonian packing, fBm blob,
> romanesco), and natural-shape *algorithms* (metaball lobes ≈ the real device,
> puffed Voronoi cells, pollen/geodesic bumps, gourd lobing, superformula,
> crystal facets). Shared knobs (size, feature count, screen) persist as you
> switch shapes; shape-specific knobs swap in/out. Every shape is one connected
> shell you can wall-hollow, with distinct convex touch pads.
>
> - **★ Save current** stores the shape + all its params as a named **favourite**
>   (browser localStorage); reload it any time from the *Saved* dropdown, or 🗑 delete it.
> - **Copy params** emits the settings; **⬇ Download STL** exports the current
>   shape from the browser (binary STL, z-up, base down).
>
> The per-family `.py` scripts below remain the higher-fidelity source for
> production STLs (the fractal/algorithm families are viewer-only for now).

| Generator | Viewer | Shape |
|-----------|--------|-------|
| [`thistle.py`](thistle.py) | [`viewer.html`](viewer.html) | thistle-flower bulb with phyllotaxis needles |
| [`thistle_lloyd.py`](thistle_lloyd.py) | [`viewer_lloyd.html`](viewer_lloyd.html) | same bulb, needles spread by **Lloyd's relaxation** (organic teasel/burr) |
| [`cymatics.py`](cymatics.py) | [`cymatics.html`](cymatics.html) | dome sculpted by a cymatics standing-wave field |
| [`mandelbulb.py`](mandelbulb.py) | [`mandelbulb.html`](mandelbulb.html) | simplified power-8 Mandelbulb fractal solid |
| [`coral.py`](coral.py) | [`coral.html`](coral.html) | coral body grown by a **reaction-diffusion** field; three forms |

> **Note (venv):** the local `.venv` was built against Python 3.13; a system
> upgrade to 3.14 orphaned it, so `build123d` (used by `thistle.py` and
> `thistle_lloyd.py`) currently won't import. Rebuild it with `./setup.sh`
> before running those two. `cymatics.py` and `mandelbulb.py` are pure standard
> library and run under any `python3` with no venv. `coral.py` needs only
> **numpy** (already installed system-wide — `python3 coral.py`, no venv).

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
- **`BOX`** — half-width of the sampling box (fractal units). Must clear the
  iso-surface's natural radius on every side, or the shape gets clipped flat by
  the box wall.
- **`FLATTEN_ALL`** / **`FLATTEN_BASE`** / **`FLATTEN_ZONE`** — vertical squash for
  a standable base, applied as a smooth post-process warp (not a cutting plane) so
  the surface stays one continuous skin. `FLATTEN_ALL` compresses the whole shape;
  `FLATTEN_BASE` adds extra compression near the bottom pole, ramped in over the
  bottom `FLATTEN_ZONE` fraction of the lower half.
- **`SMOOTH_ITERS`** — Laplacian smoothing passes to soften the tetra faceting.
- **Screen** — `SCREEN_MODE = "bulb"` + `SCREEN_DIA` / `SCREEN_DEPTH` carves a round
  flat recess into the top lobe (voxel-resolution — raise `GRID` for crisper walls,
  or true it up in Fusion). `"none"` leaves the raw bulb for you to cut the screen
  and connectors in Fusion, which is the more flexible route.

Dial `POWER` / `ITERATIONS` / `LEVEL` live in [`mandelbulb.html`](mandelbulb.html)
(a raymarched preview), then **Copy params** into the script.

---

## coral — reaction-diffusion enclosure

[`coral.py`](coral.py) grows a coral-like body from a **Gray-Scott
reaction-diffusion** field — the same maths behind the labyrinthine ridges of
brain coral and Turing patterns generally. The simulation is **seeded at the pad
centres**, so the wavy ridges radiate out from every pad (RD *grows* the
protrusions) while the same field displaces the whole surface into coral grooves
(RD *textures* it). Each of the `N_PADS` "loops" is a raised, **flat-topped seat**
sized for copper tape.

```bash
cd 3d-model
python3 coral.py               # -> coral.stl   (~2 s; needs numpy, no venv)
```

Output is a mesh STL — import into Fusion 360 / FreeCAD as a **mesh body** (like
`mandelbulb.stl`) and shell it / add connector cutouts there. A STEP path can
follow later.

**Three forms**, all from the one script — set `FORM`:

| `FORM` | Look |
|--------|------|
| `"dome"` (default) | a low **brain-coral dome** carpeted in ridges; pads are flat seats raised on the bumps. One continuous watertight mesh. |
| `"bulb"` | an ellipsoid body (thistle-style) with `N_PADS` **wavy coral branches** = pads. |
| `"fingers"` | a base plate with `N_PADS` **upright wavy finger-coral tubes** = pads. |

For `bulb` / `fingers` the branches are separate watertight solids that *overlap*
the body (the slicer / Fusion unions them on import) — the same trick as thistle's
needles.

Key parameters:

- **Pads ("loops")** — `N_PADS` (= your electrode count), `PAD_DIA` (flat seat
  size), `PAD_HEIGHT` (how far each protrusion rises — the "length"), `PAD_SKIRT`
  (blend back to the body), `PAD_BAND_LO` / `PAD_BAND_HI` (keep pads off the
  rim/pole), `PAD_LAYOUT` (`"phyllotaxis"` or `"ring"`).
- **`PAD_STYLE`** (dome only) — `"raised"` puts a flat copper seat on a bump per
  pad; `"flat"` gives a **pure brain-coral dome** with no raised seats — the
  surface is unbroken ridges and the pads are just marked positions (RD still
  seeds them, so you get N organized coral zones — stick the copper on the ridges).
- **Coral pattern** — `RD_AMOUNT` (ridge height in mm; `0` = smooth body + bare
  pads), `RD_FEED` / `RD_KILL` (the pattern character — presets in the script
  header: coral/worms/spots/maze/mitosis), `RD_STEPS` / `RD_GRID` (sim
  quality/speed), `RD_SEED` (reshuffle into a different coral), `RD_RECTIFY`
  (`"signed"` ridges+grooves vs `"absolute"` all-positive welts).
- **Broken ridges** — `RIDGE_BREAK` (`0` = continuous coral, higher chops the
  ridges into segments), `RIDGE_LENGTH` (approximate segment / loop length in mm),
  `RIDGE_WIGGLE` (`0` = smooth, higher makes the ridges meander / wiggle). These
  are a cheap post-process on the field — in the viewer they update instantly
  without re-running the simulation, and they apply to all three forms.
- **Body** — `BODY_RADIUS`, `BODY_HEIGHT`, `DOME_PROFILE` (`ellipse` / `parabola`
  / `cosine` / `flat`), `BULB_BOT_CUT` (bulb stand), `PLATE_HEIGHT` (fingers base).
- **Screen** — `SCREEN_MODE = "center"` (round recess in the top), `"pad"` (recess
  carved into pad `SCREEN_PAD_IDX`), or `"none"`; `SCREEN_DIA` / `SCREEN_MARGIN` /
  `SCREEN_DEPTH`.

Print is a **solid** — hollow it after import, like the others.
