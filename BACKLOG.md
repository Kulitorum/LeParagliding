# Editor backlog — ideas parked for later

Deferred items from the section-editor work (2026-07-26). Roughly ordered by
how often the missing piece has already caused confusion.

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
- Section 4 holes and Section 31 skin-tension groups: group-aware editors
  (counts maintained automatically).
- Add/remove-record support in the grids, updating the structural count
  fields automatically (currently directed to the text editor on purpose).

## Validation

- Extend cross-section row-count checks beyond Sections 2/3: flag-gated
  per-rib tables (26 vents, 30 thickness), nested counts (4, 15, 16, 31),
  and Section 9 line-type ids existing in the Section 34 catalogue.
- A whole-design "pre-flight" summary: run all section validators and list
  every problem in one place before Build.

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
