# Printed models — reference archive

This folder is the record of models that were **actually physically printed**.
Unlike the loose `.stl`/`.step` exports elsewhere in `3d-model/` (which are
git-ignored because they're large and regenerable from the `.py` scripts), the
contents of *this* folder **are committed** — they're the source of truth for
"what did I print, how did it come out, and with what settings."

## How to add a print

1. Copy the template folder and rename it for your print:

   ```bash
   cp -r "3d-model/printed/_template" "3d-model/printed/2026-07-05_thistle-lloyd"
   ```

   Suggested naming: `YYYY-MM-DD_<model-name>` (date you printed it).

2. Drop the exported mesh into the folder — the exact `.stl` (and/or `.step`)
   you sliced. These are un-ignored inside `printed/`, so they'll commit.

3. Add a photo (or several) of the finished print — `photo.jpg`, `photo-2.jpg`,
   etc. Any image format is fine.

4. Fill in `print-notes.md` with the slicer/printer settings and how it turned
   out.

## Creality slicer profiles

Exported Creality Print / Creality Slicer configuration profiles live in
`creality-profiles/`. Save reusable profiles there so a good print can be
reproduced exactly. See that folder's README for how to export them.
