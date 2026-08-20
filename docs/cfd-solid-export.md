# CFD solid STEP export

This record describes the exterior-only `lep-solid.step` export introduced for
CFD workflows. Read it before changing NURBS panel capture, sewing tolerances,
wingtip handling, or engine output orchestration.

## Output contract

Normal non-preview calculations write two distinct STEP models:

- `lep-3d.step` remains the named engineering assembly, including internal
  ribs, mini-ribs, diagonals, suspension lines, and intentionally open vents.
- `lep-solid.step` contains exactly one named exterior solid and no internal
  structure or construction curves. Its intake/vent regions and any genuine
  exterior openings are closed so CAD and CFD programs can import a manifold
  body.

The solid uses the same exact analytical NURBS surfaces captured for the
ordinary wing model. It is not reconstructed from the legacy display mesh.
Vent faces that are omitted from the open engineering assembly are retained in
a separate capture collection and used only by the CFD writer.

## Conditional closures

The writer first sews the mirrored upper skin, lower skin, and vent faces. It
then adds only the closures demanded by the remaining topology:

- A wingtip cap is created from the exact upper and combined vent/lower
  boundary curves only when the preliminary sew reports an open physical tip.
  An already closed or collapsed tip is not capped again.
- Distinct upper and lower trailing-edge curves receive a ruled exterior
  closure. Coincident trailing edges are reused without another face.
- A mirrored innermost profile may receive narrow region-by-region centreline
  bridges only when the two boundaries are noncoincident but remain within the
  existing 0.5 mm symmetry tolerance. A materially off-centre boundary is not
  bridged.

The final shape must have zero free edges, exactly one shell, valid OCCT
topology, and exactly one `TopoDS_Solid`. The writer orients the closed solid,
applies a bounded healing/SameParameter pass, and exports it as AP242. Failure
is nonfatal to the traditional calculation outputs and is reported as a
`CFD solid warning`.

Single-skin designs are currently unsupported because their exterior does not
define the same closed double-surface volume. They keep all traditional
exports and receive the nonfatal warning instead of an invalid solid file.

## Ownership and compatibility constraints

The odd-centre-cell rib fix shipped with this work is related but independent:
a self-mirrored centre cell uses the real innermost rib and its mirror, not a
synthetic solid `Rib 0`. Both STEP and Playground hole tables follow that
ownership. Preserve this rule when changing centre-cell capture.

Do not add ribs or other internal parts to `lep-solid.step`; CFD users asked for
the closed exterior fluid boundary only. Do not close the vents in
`lep-3d.step`, whose open-intake assembly remains the engineering model.

## Verification state

The independent `solid-step-test` reimports the written STEP through OCCT and
requires one valid solid containing one closed shell. Engine integration tests
also reject internal assembly product names in the CFD file.

Known verified examples as of 2026-08-21:

- Plan B Parakite: 69 exterior faces, no generated tip, trailing-edge, or
  centreline closures, and zero free edges. It was also imported by the user
  into an independent CAD system and recognized as a proper solid.
- gnuA7 odd-centre preset: two required wingtip caps and zero free edges.
- SoftWingStudio fixture: two wingtip caps, three bounded centreline closures,
  and zero free edges.

The Release build and complete Windows test suite passed 28/28 after the
implementation. Relevant focused checks are:

```powershell
cmake --build --preset release --parallel
ctest --test-dir build -C Release --output-on-failure -R "preset_gnua7|softwingstudio_export"
ctest --preset release
```

Generated `lep-solid.step` files are calculation output and must not be added
as source fixtures unless a future test deliberately introduces a reviewed
binary oracle.
