# LEparagliding Studio repository guide

This is the repository-level working guide for Codex and other coding agents.
It records where behavior lives, how the pieces communicate, which compatibility
contracts are load-bearing, and how to verify changes. Keep it current when a
change moves ownership or invalidates a command or invariant.

## Project identity

- LEparagliding Studio is a GPL-3.0 C++20/Qt 6 desktop port of Pere Casellas'
  LEparagliding 3.28 engineering program.
- `project(... VERSION ...)` in `CMakeLists.txt` is the Studio/release version
  (currently 0.4.1). "3.28" is the calculation core and input-format version;
  it is intentionally a different number.
- Qt Widgets provides the application shell and editors. Open CASCADE (OCCT)
  8.0 owns exact NURBS construction, STEP/XCAF I/O, tessellation, selection,
  and the primary 3D viewport.
- The numerical/drawing oracle remains the translated Fortran core. New code
  wraps it; it does not casually replace or "clean up" it.
- XFLR5 v6.62 and the SoftWingLab-derived XPBD core are vendored. Both have
  local changes and are no longer drop-in upstream copies.

## Start every task here

1. Run `git status --short --branch` and preserve all user changes, including
   untracked design/airfoil files. Generated build directories may coexist with
   a running Studio instance.
2. Read the subsystem-specific design record before changing that subsystem:
   `CONTINUE.md` for current Playground work, `BACKLOG.md` for deliberately
   deferred editor/Print work, and the files under `docs/` listed below.
3. Inspect the relevant public header and its test before editing the large
   implementation file. Several `.cpp` files are thousands of lines long but
   have deliberately small boundaries.
4. Do not kill a `LEparagliding.exe` process you did not start. It may contain
   unsaved user work and it locks the link target. See the build-lock note under
   Verification.

## Build, test, and run

Windows is the primary local environment:

```powershell
cmake --preset windows
cmake --build --preset release --parallel
ctest --preset release
```

The preset uses Visual Studio 2022 x64 and writes executables to
`build/bin/Release`. CMake auto-detects the newest compatible
`C:/Qt/6.*/msvc2022_64` and checks the usual local OCCT roots. Override OCCT
with `-DLEP_OCCT_ROOT=C:\path\to\occt` when needed. Required packages are Qt
6.5+ components Core/Widgets/OpenGL/OpenGLWidgets/XML and OCCT 8.0 toolkits.

Useful narrower commands:

```powershell
cmake --build build --config Release --target leparagliding-engine
ctest --test-dir build -C Release --output-on-failure -R "engine|preset"
ctest --test-dir build -C Release --output-on-failure -R "playground"
```

Run the products with:

```powershell
.\build\bin\Release\LEparagliding.exe
.\build\bin\Release\leparagliding-engine.exe <design-file> <output-directory>
.\build\bin\Release\LEparagliding.exe --headless <design-file> <output-directory>
```

Engine flags are `--preview`, `--no-construction-curves`, and
`--resource-dir <directory>`. GUI/developer entry points include
`--smoke-test`, `--studio-self-test <design>`, `--validate-presets <dir>`,
`--xflr5`, and `--playground <lep-sim.json>`.

Linux/macOS use a Ninja build and an OCCT 8.0 install prefix; the exact recipe
is in `README.md`. `CLAUDE.md` additionally documents this machine's WSL setup,
case-sensitive verification, release procedure, and signed Windows installer.

## System map

```text
design.txt + relative airfoil files
        |
        +--> Studio: DesignDocument -> per-section editors
        |       |                         |
        |       +---- assembled current editor text -----+
        |                                                  |
        +--------------------------------------------------v
                                    temporary input + resource directory
                                                      |
                                            QProcess child engine
                                                      |
                       PreparedInput -> path/C ABI -> translated 3.28 core
                                                      |
             +----------------------+-----------------+------------------+
             |                      |                                    |
       legacy files          NURBS capture callbacks             2D capture hooks
   DXF/reports/lines        -> OCCT XCAF or AP242 STEP          -> flat-part JSON
                                    |                                    |
                              lep-sim.json                         Print/Cut tab
                                    |
                              Playground XPBD

Preview: temporary output + binary `lep-3d.xbf`; the directory is discarded
after the GUI has loaded XCAF/simulation/flat-part data.
Export: user output directory + `lep-3d.step`; files remain on disk.
```

The GUI never calls the translated core in-process. The child-process boundary
keeps the UI responsive and contains legacy parser aborts/access violations.

## Build targets and dependencies

- `f2c_runtime`: C90 runtime from `third_party/libf2c`.
- `lep_occt`: interface target collecting the required OCCT headers/toolkits.
- `leparagliding-engine`: engine boundary, flat capture, translated core, and
  exact model builder.
- `flatparts`: structured part model, deterministic nester, PDF/DXF writers.
- `softwing_core`: dependency-free XPBD/cloth/contact/pneumatics/suspension
  core under `src/softwing`.
- `playground_contact`: Qt-free bounded Playground cloth-contact features,
  topology exclusions, projection and coverage diagnostics.
- `playground_sim`: widget-independent mesh parser/body builder/step and shape
  metrics; linked by both GUI and headless tests/tools.
- `xfoil` and `xflr5core`: vendored aerodynamic application libraries.
- `LEparagliding`: Qt GUI, linked to OCCT, XFLR5, Playground, and flat parts;
  it depends on the engine executable because previews launch it.
- `softwing-bench` and `nesting-bench`: developer tools, not shipped.
- The remaining small executables in `CMakeLists.txt` are test drivers.

When adding a production source file, add it to the appropriate explicit CMake
target. XFLR5 is the exception: its source list is generated from upstream
qmake manifests into `cmake/xflr5_sources.cmake` by
`cmake/gen_xflr5_sources.py`; do not replace that list with a glob.

## Directory and ownership map

### Calculation engine

- `src/engine/main.cpp`: CLI parsing, path validation, output cleanup, engine
  orchestration, OCCT model write, and companion `lep-sim.json` write.
- `src/engine/input_migration.{h,cpp}`: non-destructive calculation copy. It
  removes the Studio history trailer and blank records, and appends disabled
  sections 33-37 to an older design that has section 32. It rejects gaps.
- `src/engine/engine_paths.{h,cpp}`: UTF-8/native path bridge, safe output path
  handling, Fortran-shaped string helpers, `fopen` wrappers, and the planform
  row parser used by the translated core.
- `src/legacy/leparagliding_core.cpp`: mechanically translated 3.28 numerical
  and drawing core (about 49k lines). It is compiled as permissive C++ by MSVC
  and as C on GCC/Clang. Treat Fortran indexing, formatted I/O, execution order,
  and C89 declaration rules as compatibility constraints.
- Root `leparagliding3.28.f` is the active original-source oracle;
  `leparagliding3.17.f` is retained historical input. `Data/Originals` contains
  upstream/source artifacts, not application code.

The core produces the traditional calculations and drawings, but modern
outputs enter through narrow C callbacks. Keep those callbacks C-clean because
the core is a C translation unit on non-MSVC platforms.

### Exact model and viewport

- `src/model/nurbs_model.{h,cpp}` owns the capture state and all OCCT model
  construction. Legacy callbacks provide transformed profiles, analytical
  ballooning data, legacy tessellation for comparison, ribs/holes, diagonals,
  mini-ribs, and labelled line segments.
- `NurbsModel` constructs exact rational span curves and lofted
  Extrados/Vents/Intrados faces, mirrors the half wing, sews topology while
  preserving designed intake openings, adds ribs/holes/diagonals/lines to a
  named XCAF assembly, validates source/legacy agreement, and writes AP242 STEP
  or binary XCAF.
- `writeSimMesh` serializes a deliberately coarse full-wing JSON mesh in
  millimetres, Z-up: welded skin quads with surface tags, rib/hole loops,
  diagonal straps, and labelled suspension lines.
- `src/gui/paraglider_view.{h,cpp}` imports STEP or XCAF, triangulates with
  OCCT, hosts the native OCCT OpenGL viewer, maps XCAF labels to stable colour
  roles, and owns camera/navigation, selection, visibility, clipping, X-ray,
  and measurement. There is no second application-side 3D mesh builder.
- `ParagliderView::ColorRole` values are persisted by ordinal. Append roles;
  never reorder existing values or their `colorSettingsKeys` in
  `mainwindow.cpp`.

Model units are millimetres. The Playground converts the simulation JSON to
metres at its boundary.

### Studio shell, document, and editors

- `src/gui/main.cpp`: application bootstrap and command-line/self-test modes.
  `--headless` is detected before creating the full GUI application path and
  delegates to the sibling engine executable.
- `src/gui/mainwindow.{h,cpp}`: application composition and workflow. It owns
  file/preset opening, editor pages, QProcess runs, preview/export/XFLR5 modes,
  model and output refresh, part-to-source navigation, preferences, settings,
  and the major tabs.
- `src/gui/design_document.{h,cpp}`: parsing and persistence of numbered
  design sections plus embedded version history and Studio-only spline data.
- `src/gui/section_help.*` reads generated manual chapters; `section_specs.*`
  describes the dominant tables and the few generic curve-enabled sections.
- `src/gui/section1_curves.*`, `spline_fit.*`, and `section_grid.*` are pure
  C++ parser/math helpers with focused tests. Keep reusable parsing out of
  widgets when practical.
- `curve_editor.*` is the generic interactive plot. `section1_curve_panel.*`
  handles section 1 B-spline truth; `airfoil_panel.*` handles section 2 airfoil
  files/splines; `holes_panel.*` handles section 4 hole geometry;
  `grid_curve_panel.*` currently exposes spec-declared curves (section 3 and
  section 30). `section_grid_panel.*` remains built/tested but is intentionally
  not instantiated: the generic value-grid UI was retired.
- `geometry_preprocessor.*` is a pure C++ port of the separate v1.6 Canigo
  preprocessor. `geometry_preprocessor_dialog.*` is its UI and patches the
  resulting rib rows into section 1.

Each section has its own `QPlainTextEdit` undo stack. Once that live stack is
exhausted, MainWindow continues through the saved per-section history. Enter
builds the current in-memory design; Shift+Enter inserts a record. Preview and
export do not implicitly save.

`DesignDocument` recognizes headers with `* <number>. <title>`, preserves the
preamble and final-newline choice, rejects duplicate sections and interior
blank records, and assembles text in section order. Save uses `QSaveFile` and
appends a comment trailer delimited by:

```text
* >>> LEPARAGLIDING STUDIO HISTORY V1 >>>
...
* <<< LEPARAGLIDING STUDIO HISTORY V1 <<<
```

The trailer is base64 compact JSON. It contains a SHA-256-linked chain of
qCompressed full-wing snapshots (UTC time, parent, changed sections, summary)
and optional Studio spline JSON. Restoring an old version does not delete newer
history; saving the restored payload appends a new latest revision. The engine
always strips this trailer from its temporary calculation input.

Shipped presets are read-only masters. Saving one asks for a new design path
outside `resources/presets` and copies adjacent resource/airfoil files with it.

### Preview, export, and XFLR5 flow

`MainWindow::startCalculation` always writes the current editors to a temporary
`leparagliding.txt` and passes the real design directory as `--resource-dir`, so
relative airfoil references resolve correctly.

- Preview writes to a temporary output directory and adds `--preview`; the
  engine writes `lep-3d.xbf`, which the viewport loads faster than STEP.
- Export writes to the selected output directory and keeps the standard STEP.
  The construction-curve preference affects export only.
- XFLR5 transfer creates a temporary design with section 36 forced on, runs the
  engine, and calls the locally added `MainFrame::lepImportLepWing` on the
  generated `xflr5` folder. Transfers are hash-deduplicated and queued behind
  an in-flight preview.
- After a successful run, `lep-sim.json` is copied into Playground state and
  `lep-2d-parts.json` is loaded into Print state before the temporary directory
  disappears.

The six user-facing export files are:

- `leparagliding.dxf`: 2D manufacturing plans.
- `lep-3d.step`: exact named OCCT model.
- `lep-3d.dxf`: legacy reference wireframe.
- `lep-out.txt`: calculation report.
- `lines.txt`: suspension data.
- `run-log.txt`: calculation progress/diagnostics.

Optional legacy settings can additionally create `stl/`, `xflr5/`, and surface
DXFs. `lep-3d.xbf`, `lep-sim.json`, and `lep-2d-parts.json` are internal Studio
handoffs, although they are useful developer fixtures.

### Flat parts and Print/Cut

- Hooks declared in `src/flatparts/flat_capture.h` are inserted around the
  translated core's 2D drawing loops. `flat_capture.cpp` attributes otherwise
  anonymous DXF primitives to stable categories/pieces, chains line segments,
  classifies cut/seam/mark roles, converts to local millimetres/y-up, and writes
  `lep-2d-parts.json`.
- `flat_parts.*` parses that JSON into `FlatPartSet`/`FlatPiece`.
- `nesting.*` performs deterministic irregular strip packing. It rasterizes
  concave outlines to bitmasks, tries rotations and orders, minimizes tiled
  page count, reports only improving layouts, and cooperatively checks cancel.
- `nest_worker.*` runs nesting on a `QThread` with generation IDs so stale
  queued results cannot overwrite a newer run.
- `flat_parts_view.*` draws either the selected parts or the exact packed
  layout. Preview and exporters share `frameFor`; do not independently
  reimplement placement transforms.
- `sheet_export.*` tiles paper with overlap/furniture into a true-size PDF or
  writes R12 DXF (continuous canvas or one machine-bed file per sheet).
- `print_page.*` owns selection, scale/target area, paper/bed/rotation options,
  the live pack, and export controls. Export uses the captured packed options,
  not whatever widgets have changed since the pack.

Known Print correctness gaps are documented in `BACKLOG.md` and
`docs/flat-part-orientation.md`: `grainAngleDeg=90` with
`grainSource=assumed-from-plan-layout` is a placeholder, not engineering data;
and `outerBoundary()` misses small extents of some cut geometry. Quarter-turns
avoid bias but do not prove the intended grain direction. Never enable free
rotation for direct fabric cutting. Seam allowances remain fixed millimetres
when the wing itself is scaled.

### Playground and soft-wing simulation

- `src/gui/playground_sim.{h,cpp}` is the widget-independent adapter: parse and
  refine `lep-sim.json`, build the XPBD body/constraints/ribs/lines/pilot/cells,
  stamp pressure, impose the wing polar, step, contact, brakes, grabbing, and
  free-flight recentring.
- `playground_pressure_solve.*` is the Qt-free deterministic bounded weighted
  equality projection for the final exterior-Cp field. Force, pitch and the
  two half-differential rows enter in priority stages; it reports physical
  saturation separately from numerical failure.
- `playground_metrics.*` captures the rest-shape baseline and measures section
  deviation, span/area/volume, slack, asymmetry, dents/twist, line-row loads,
  cell/kink/collapse diagnostics, settling, heatmap fields, and CSV.
- `playground_page.*` is the lazy-created OpenGL tab, controls, live solver,
  HUD/session log, settle run, and analysis entry point.
- `playground_analysis.*` runs alpha sweeps on a worker using a fresh body per
  point and the same settle/measure path as the bench.
- `tools/softwing_bench.cpp` is the authoritative headless driver for the same
  `playground_sim` code. `tools/softwing_gpu.*` is an experimental benchmark
  backend, not the GUI simulation path.
- `src/softwing` supplies generic XPBD distance/cable constraints,
  deterministic coloured parallel sweeps, membrane/contact/pneumatics/canopy/
  suspension machinery. The Playground defaults to its calibrated legacy
  distance-truss skin and also offers an explicitly experimental orthotropic
  membrane skin with compression softening and four-node dihedral hinges.
  Free flight additionally requests three deterministic cable-only
  reverse/forward sweep pairs to propagate payload load through the deep
  suspension graph; zero is the exact historical/tunnel path.

The Playground is an instrumented relative/structural design sandbox, not a
certified absolute aerodynamic solver. Wind-tunnel mode is the default claim;
free flight is explicitly experimental. Simulation time is fixed at 60 Hz and
can advance more slowly than wall time. Worker count is explicit because
reproducibility is per worker count.

Before changing Playground forces, pressure, flight trim, or controls, read
all of `CONTINUE.md` and `docs/playground-shape-analysis.md`. The essential
invariants are:

1. System-level forces enter through the actual suspension/load path; prior
   point/body/couple shortcuts visibly damaged the canopy.
2. The wing polar fully cancels the pressure field's resultant. A new pressure
   request belongs in `wingForce`, or cancellation erases it. The one explicit
   post-solve exception is bounded-free-flight viscous skin traction. Below
   separation it closes the positive `q*S*Cd_polar - achievedPressureDrag`
   deficit. Above separation the live exterior-Cp field owns flat-plate form
   drag and its moment; traction may close only the remaining viscous/profile
   target, never missing form drag, lift, or the excess-frontal heuristic.
3. Do not feed per-node breathing velocity back into pressure. Rotational
   feedback uses the canopy rigid-body fit.
4. Verify angle-of-attack sign dynamically: sinking must raise alpha.
5. Free-flight damping is relative to bulk velocity or it becomes fake drag.
6. Drag must decelerate, never propel.
7. Solver left is negative mesh X (the viewer's right). The UI crossover lives
   in `setBrakePull`; bench flags and `SimBody::ribHalf` use solver sense.
8. Constructor-time widget values do not guarantee controls reached the view;
   add new defaults to `ensureView`'s explicit push list.
9. Production retrim solves the FINAL exterior Cp inside `[-3, 1]`, then
   reconstructs `p_inside-q*Cp`; never reintroduce an unbounded increment plus
   a post-solve clamp. q≈0 faces have no authority and stay at their prior.
10. The objective prior is the same-frame legacy 4x4 load-path proposal, but
    every intermediate pressure is clamped through the per-face physical
    interval `[p_inside-q, p_inside+3q]`. Never reuse the legacy absolute clamp
    here: its interval can invert during the pre-inflated soft start.
11. Active faces stay within +/-0.05 Cp of that preferred topology. Production
    authority is cached and re-verified on live geometry: halve an infeasible
    hint, then try one +0.02 probe. Force starts at 0; normally feasible pitch
    and differential start at 1. Preserve the cached normalised dual warm
    starts. Exact bisection belongs to pure tests/offline audits, not every
    full-canopy frame.
12. Force rows freeze before pitch, which freezes before the common L-R
    lift/drag authority. Preserve the prior half difference and the explicit
    factor two for +D/-D half loading.
13. Pinned bounded lift/drag telemetry reports the achieved pressure force.
    Free-flight telemetry adds separately diagnosed polar-only skin traction
    and pilot drag. Traction must be q-gated, area-weighted, dissipative, zero
    in pinned/legacy modes, and must never reinject `lastFabricDragNewtons` or
    a separated-flow form-drag shortfall. The old symmetric tunnel row is an
    explicit regression oracle. Use `--legacy-pressure`; never make the legacy
    path the production default.
14. TrimmedGlide q comes from achieved vertical wing support versus total
    dynamic-node weight, followed by eight bounded-only co-moving structural
    relaxation frames and a state-clean recalibration. DropFromRest and legacy
    launch do not receive this relaxation. Keep the 30/90/250 kg support/cap
    regression and report calibration residuals/iterations.
15. Three free-flight cable sweep pairs are targeted load-path conditioning,
    interleaved before general structural sweeps. Tunnel and zero-pair paths
    preserve historical arithmetic exactly; do not replace this with hidden
    line ballast or global extra cloth iterations.

Weathercock incidence and the bounded final-Cp retrim are now implemented.
Remaining free-flight stability and material-model limitations are recorded in
`CONTINUE.md`; do not infer that bounded authority makes this a certified
aerodynamic solver.

### Vendored and generated content

- `third_party/libf2c`: vendored runtime. Avoid edits unless fixing a proven
  compatibility/runtime issue; preserve `Notice`.
- `third_party/xflr5`: vendored v6.62 application and XFoil. Read
  `third_party/xflr5/PROVENANCE.md` before changing it and record any new local
  embedding patch there. The upstream app bootstrap is intentionally excluded.
- `src/softwing`: originated from SoftWingLab but has intentionally diverged.
  Read `src/softwing/README.md`; future syncs are merges, never blind copies.
- `resources/manual`: generated offline manual and qrc. Do not hand-edit its
  HTML/images; run `python tools/extract_manual.py`. Delete the gitignored
  `tools/manual.en.html` first only when an upstream refresh is intended.
- `resources/presets`: shipped, self-contained designs plus
  `presets.json`. Update its README when adapting upstream files. All presets
  must pass Studio validation, not just the more tolerant engine parser.
- `cmake/xflr5_sources.cmake`, `installer/version.iss`, build trees, deployed
  DLLs, installer output, and most calculation output are generated. The first
  has a generator script; `installer/version.iss` is regenerated at configure
  time and is gitignored.

## Verification matrix

There are 25 configured tests on Windows. The Fortran-reference test is
Windows-only; local `gui_smoke` and `studio_model_smoke` exercise display/model
paths that release CI deliberately excludes from its offscreen test command.

| Change area | Minimum relevant checks |
|---|---|
| engine paths/migration/legacy I/O | `engine_help`, `engine_rejects_missing_input`, `engine_tolerates_blank_lines`, `engine_matches_fortran_reference` |
| translated numerical/drawing core | full suite; the Windows Fortran reference is mandatory, plus `preset_gnua7_calculates` |
| design parsing/history/editor orchestration | `studio_model_smoke`, `presets_validate_in_studio`, `gui_headless_help` |
| Section 1/splines/airfoils/grid helpers | `section1_curves`, `spline_fit`, `airfoil_file`, `section_grid` as applicable |
| geometry preprocessor | all four `preprocessor_matches_fortran_*` tests |
| NURBS/XCAF/viewport/engine callback changes | `engine_matches_fortran_reference`, `studio_model_smoke`, and `gui_smoke`; inspect engine model statistics/warnings |
| flat capture/nesting/PDF/DXF | `flatparts_export`; run `nesting-bench` when placement quality/performance can move |
| Playground body/pressure/cells/material | `playground_pressure_solve`, `playground_cells`, `playground_metrics`, `playground_material`, `softwing_material`, and the deterministic bench guards below |
| Playground contact | `playground_contact`, `playground_contact_integration`, plus a relevant bench/GUI scenario |
| packaging/resources/CMake | configure from clean metadata, build Release, and run the full suite |

The Windows reference fixture is `tests/fixtures/3.28/leparagliding.txt` with
adjacent `gnuC2.txt`; expected reports are under `tests/reference/3.28`.
`leparagliding.dxf` and `lep-3d.dxf` must match the native gfortran run by SHA-256;
reports require identical structure/fields with only the documented 0.00015
display tolerance. Do not regenerate the oracle from the translated engine.

Geometry-preprocessor fixtures are under `tests/fixtures/pre-1.6` and compare
against native Fortran output within 0.03 cm/0.06 degrees.

The current deterministic Playground guards (details and interpretation in
`CONTINUE.md`) are:

```powershell
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --shape --csv --legacy-pressure
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --pressure-acceptance
.\build\bin\Release\softwing-bench.exe build\aero\gnuC2\lep-sim.json --glide 300
.\build\bin\Release\softwing-bench.exe build\aero\Swoop\lep-sim.json --tuck 250 --substeps 60 --iterations 4
```

The `--dive` scenario is chaotic and non-monotonic; never call one run a
regression. Sweep disturbances and compare distributions. The tunnel CSV guard
is the cheap bit-comparable signal.

### Verifying GUI changes without disturbing the user

A running Studio locks `LEparagliding.exe`. Do not terminate it. Windows allows
renaming a running executable aside before relinking, but preserve and report
any existing renamed copies. Prefer a narrower non-GUI target when sufficient.

For screenshots, do not steal focus or inject global input. Background Qt
windows can be captured with `PrintWindow(..., PW_RENDERFULLCONTENT)`, but a
native/OpenGL child can still be blank or flaky. A blank capture is not proof
that a widget failed to paint. Use an offscreen Qt harness and `widget->grab()`
for ordinary widgets; verify native GL behavior through its status/metrics and
an explicitly launched instance. The detailed machine recipe lives in
`.claude/skills/verify/SKILL.md` even though it is not a global Codex skill.

## Code and change conventions

- Match the existing four-space style and local brace/Qt conventions; there is
  no repository-wide formatter configuration to run blindly.
- Use `QStringLiteral`/`QLatin1String` for fixed Qt text. Keep reusable math and
  parsers in standard C++ when that permits small non-widget tests.
- Preserve exact byte layout where a helper promises format-preserving token
  replacement or Fortran parity. Do not normalize whole design sections as a
  side effect of one edit.
- Prefer atomic file writes (`QSaveFile`) for user documents/exports where the
  existing layer supports them.
- Keep long calculations, nesting, and sweeps off the GUI thread. Worker
  callbacks execute on the worker thread unless explicitly queued; never touch
  widgets from them.
- Preserve deterministic results. The nester deliberately fixes tie-breaking;
  XPBD parallel sweeps deliberately define reproducibility by worker count.
- Comments in the model, flat-part, and Playground code often record rejected
  physical/geometric approaches with measurements. Treat them as design
  constraints, not cleanup targets. Update the matching design document when
  changing the conclusion.
- The app embeds XFLR5 in its own `QApplication`. Do not reintroduce XFLR5's
  bootstrap or application-wide style mutations.
- New outputs loaded from a temporary preview must be consumed before
  `calculationDirectory_` is reset.

## Documentation and release map

- `README.md`: user-visible features, output contract, portable build recipe,
  architecture summary, and release overview.
- `CLAUDE.md`: machine-specific Windows/WSL verification, generated manual,
  GitHub release notes requirements, and SafeNet/Inno Setup signing recipe.
- `CONTINUE.md`: current Playground free-flight handoff, measured guards,
  diagnosed failures, and expensive invariants. It is active technical state,
  not general marketing documentation.
- `BACKLOG.md`: consciously deferred editor, validation, Print, and preview
  work. Check it before "fixing" behavior that may be intentionally parked.
- `docs/playground-shape-analysis.md`: Playground model/measurement rationale
  and claim boundary.
- `docs/xpbd-performance.md`: solver profiling, parallel/GPU experiments,
  load-field evolution, and free-flight architecture.
- `docs/flat-part-orientation.md`: unresolved fabric-grain problem and proposed
  data/UI solution.
- `third_party/xflr5/PROVENANCE.md` and `src/softwing/README.md`: vendor origins
  and local divergence contracts.

Release version comes only from `CMakeLists.txt`. GitHub CI runs on `v*` tags
or manual dispatch, builds Windows/Linux/macOS, and packages a Windows zip,
Linux AppImage, and macOS DMG. Every release needs a hand-written description
with a first-visitor summary, version changes, and platform install notes; see
`CLAUDE.md`. The signed Windows installer is built locally with
`installer/build_installer.ps1`; commit first because configure bakes the HEAD
hash into `installer/version.iss` and the installer filename.
