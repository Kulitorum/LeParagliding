# Editor backlog — ideas parked for later

Deferred items from the section-editor work (2026-07-26). Roughly ordered by
how often the missing piece has already caused confusion.

## Preview performance (follow-ups to the 0.2.0 speedup)

The 0.2.0 rebuild optimizations (single rib validation, parallel rib
building, binary XCAF preview handoff) left the legacy core as the
dominant preview cost (~1.4 s of ~1.75 s). Remaining levers, measured
2026-07-26 on gnuA7/24:

- **Gate the legacy exports behind `--preview`**: every preview run still
  writes ~28 MB of DXF/STL/SCAD text the viewport never reads. Needs a
  flag plumbed into the f2c core's WRITE units — the biggest remaining
  win (rough estimate 0.5–1 s).
- `ParagliderView::triangulateShape` calls `BRepTools::Clean` before
  meshing, so unchanged shapes re-tessellate; mostly felt in the
  resolution slider.
- Per-section caching was examined and rejected: the engine is a fresh
  process per rebuild and the translated Fortran core is one monolithic
  pass, so nothing survives to reuse (see the 0dffb52 commit message for
  the stage timings).

## Reinvent the value grid (v1 retired from the UI)

The generic QTableWidget grid was worse than raw text and got removed from
the section pages (the code — section_grid_panel.{h,cpp} — is still built
and tested, just never instantiated). Observed failures: alternating rows
rendered light-on-light in the dark theme; the row label repeated the
section banner for every record instead of saying something useful; nested
sections (holes, line plan) became ragged cells with read-only holes; and
cell-by-cell editing was slower than typing in the text. Ideas for v2:
only show a grid where a genuinely uniform table exists; align/highlight
the TEXT editor itself (elastic tab stops, column coloring, hover hints
per token) instead of mirroring it into a widget; bespoke editors per
structured section (see below). The retired grid's rib-count cross-check
(sections 2/3 vs Section 1 cells) should return in whatever replaces it.

## Make engine-ignored values obvious

- **Orphaned anchors:** grey out / annotate Section 3 anchor curves that no
  Section 9 line actually ends on (Hegala-v2's D anchors exist but carry no
  lines — two debugging sessions started this way). Requires reading the
  line plan's final (anchor row, rib) pairs.
- **Per-point gating:** the current gates disable a whole curve; on wings
  where the Anchors count varies per row (0/4/0/4…), also dim and lock the
  individual points on rows whose count doesn't reach that column.
- General principle worth pursuing everywhere: if the engine will not read
  a value, the editor should say so before the user edits it.

## Curve/spline coverage

- B-spline truth mode for the other curve sections (3 anchors, 30
  thickness) — reuse the Section 1 mechanism; the trailer JSON is already
  namespaced per section ("section1" → "sectionN").
- Promote more columns to curves: Section 2 intake in/out, Section 10
  brake distribution rows, Section 26 vent percentages.
- **Free rib/cell count change** (the end goal): once per-rib data is
  spline-backed, changing the cell count becomes "resample every spline at
  the new stations and regenerate the per-rib rows in every section".
  Watch the centre-cell subtlety: rib 1's x-rib is half the centre cell
  width, so station placement at u=0 isn't strictly resolution-independent.

## Richer editors for structured sections

- Section 9 suspension lines: a tree/cascade editor instead of the raw
  grid (levels → branches → anchors).
- Section 12 H/V/VH ribs: per-type sub-schemas (type code in column 2
  decides the row layout: 1, 3, 6, 11, 13, 15, 16).
- Sections 15/16 colors: visual chordwise color-region editor.
- Section 31 skin-tension groups: group-aware editor (counts maintained
  automatically). Section 4 holes got a graphical editor (2026-07-26);
  still deferred there: adding/removing holes and groups from the GUI
  (counts live in the text), and rotated/rounded triangle rendering
  fidelity.
- Add/remove-record support in the grids, updating the structural count
  fields automatically (currently directed to the text editor on purpose).

## Validation

- Extend cross-section row-count checks beyond Sections 2/3: flag-gated
  per-rib tables (26 vents, 30 thickness), nested counts (4, 15, 16, 31),
  and Section 9 line-type ids existing in the Section 34 catalogue.
- A whole-design "pre-flight" summary: run all section validators and list
  every problem in one place before Build.

## Fabric orientation of flat parts

The exported flat parts carry no fabric-grain information — the core lays each
part out to fill its drawing boxes and never records which way the weave should
run, so the Print tab's nester is preserving an orientation that means nothing.
Ripstop stretches on the bias, so this is an aerodynamic issue, not a finish
one. Deliberately not solved for now; the nester assumes the input is oriented
correctly. Requirements for the C++ port, the options for deriving the
direction, and a proposed "draw the fibre vector on each piece" editor are
written up in `docs/flat-part-orientation.md`.

## Packed footprint misses some cut geometry

`outerBoundary()` chains a piece's cut-role polylines into one closed boundary,
and that boundary is what the nester rasterises, positions and measures. On
gnuA7/24 four parts carry cut points the boundary does not cover, so they sit
2–8 mm outside the packed canvas. The Print tab now says so after a pack ("N
part(s) have cut geometry reaching past the sheets", via
`flatparts::clippedPlacements`) rather than letting the PDF clip it away in
silence, but the fix belongs in the chaining: either make the boundary cover
every cut polyline, or pack against the union of them. Note it changes packing
results, so the nesting bench numbers move with it.

Related, and already handled in the writers: the engine's text anchors are
positions in the plan's drawing box, not on the part — every rib's number is
anchored 635 mm to the *left* of the rib. `sheet_export.cpp` re-anchors any
label that does not land on its own part to the part's centre. If flat-part
capture ever records label positions relative to the part, that workaround can
go.

## Smaller ideas

- Optional auto-rebuild of the 3D preview after a curve commit (debounced),
  instead of requiring Enter/Build.
- Generate the "?" dialog field-reference tables from section_specs so
  help and editors share one source (Section 1 already does this).
- Version restore keeps current B-splines and relies on staleness detection;
  storing splines per revision would restore them together.
- Sections 22/23 (nose mylars, tab reinforcements) are disable-flags in
  every shipped design; their enabled layouts are undocumented here — dig
  into the original Fortran sources if ever needed.
- The Studio trailer still says "HISTORY V1" although it now also carries
  the B-spline definitions; an older Studio build saving such a file
  rebuilds the trailer and silently drops the splines (the sampled matrix
  text survives). Consider a version bump / preserve-unknown-keys rule.
