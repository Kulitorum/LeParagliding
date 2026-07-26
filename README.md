# LEparagliding C++ / Qt

[![CI](https://github.com/Kulitorum/LeParagliding/actions/workflows/ci.yml/badge.svg)](https://github.com/Kulitorum/LeParagliding/actions/workflows/ci.yml)

This project is a C++ port of Pere Casellas' LEparagliding 3.28 “Jardins”
engineering program with a Qt 6 desktop interface. It is free software under
the GNU GPL 3.0, like the
[original program](https://www.laboratoridenvol.com/leparagliding/lep.en.html)
it derives from.

The application accepts the same design text file as the original program.
Airfoil file references remain relative to the selected design file. Generated
files can be written to a separate folder:

- `leparagliding.dxf` — 2D manufacturing plans
- `lep-3d.step` — exact OCCT NURBS wing model
- `lep-3d.dxf` — legacy 3D wireframe retained as a reference
- `lep-out.txt` — calculated design data
- `lines.txt` — suspension line data
- `run-log.txt` — calculation progress and diagnostics

Version 3.28 can also create `stl` and `xflr5` subdirectories when the
corresponding design options are enabled.

The desktop application is a complete design studio:

- every numbered block in the selected design has its own syntax-highlighted
  editor and independent Undo/Redo history;
- every Save embeds the wing's complete version history in the design file, so
  Undo/Redo can continue across restarts and the `Versions...` window
  can restore the whole wing to any saved state;
- the `?` button on a section opens format guidance and a link to the full
  manual;
- opening a design automatically calculates a fresh OCCT NURBS model in a
  temporary folder, preventing an older exported model from being shown;
- **Build paraglider** validates the current editors and refreshes that
  temporary preview without saving the design or writing user output files;
- **Export files...** writes the manufacturing plans, 3D geometry, reports,
  and line data to the selected Output folder;
- the viewport reads `lep-3d.step` with OCCT, triangulates its exact NURBS
  surfaces with OCCT, and renders them with the native OCCT OpenGL viewer;
  isometric, front, back, left, right, top, and bottom views are available in
  perspective or orthographic projection;
- **Preferences…** adjusts the viewport triangulation resolution from very
  coarse to ultra fine; the setting applies immediately, is remembered between
  sessions, and never affects exported files.

Viewport navigation follows the slicer convention: drag with the left mouse
button to orbit, drag with the right or middle mouse button to pan, and use the
wheel to zoom. `Shift` + left drag also pans; double-click or press `F` to fit
the model. Number keys `0`–`6` select the preset views and `P` toggles the
projection. Double-click a completed output file to open it in its associated
viewer.

While editing a section, press `Enter` to build and refresh the temporary 3D
preview. Use `Shift+Enter` when you intentionally need to insert another input
record. Preview builds and exports use the current editors but do not save them;
use **Save** when the change should become a new embedded version.
`Ctrl+Z`/`Ctrl+Y` and the section's Undo/Redo buttons affect only the currently
visible section; switching sections does not merge or clear their histories.
Once the live editor history is exhausted, Undo continues through that
section's saved versions. Restoring an older whole-wing version does not delete
newer versions; the restored state becomes a new latest version when it is next
saved.

Version history is stored as a marked comment trailer at the end of the same
design file. Each entry is a compressed full-wing snapshot with a UTC
timestamp, changed-section list, parent identifier, and SHA-256 identifier.
The calculation engine removes this trailer in a temporary input copy before
calling the strict Fortran-compatible parser; the editable design file and its
history remain intact. A design without embedded history is treated as version
1, preserving the wing exactly as it was first opened.

The original
[LEparagliding user manual](https://www.laboratoridenvol.com/leparagliding/manual.en.html)
documents every input section. In particular, its record order is strict and
blank lines are not valid records.

## Build on Windows

The CMake preset mirrors the compiler, Qt, and Open CASCADE setup used by
`C:\CODE\cobod-slicer`: Visual Studio 2022, the newest compatible Qt 6 MSVC kit
under `C:\Qt`, and OCCT under `C:\OpenCASCADE-8.0\build2`. CMake auto-detects
that OCCT installation. A different compatible build can be selected with
`-DLEP_OCCT_ROOT=C:\path\to\occt`.

```powershell
cmake --preset windows
cmake --build --preset release --parallel
ctest --preset release
```

Run:

```powershell
.\build\bin\Release\LEparagliding.exe
```

`windeployqt` and the OCCT runtime deployment run after the build, so the build
output is directly runnable. The install target includes the required OCCT
toolkit and third-party DLLs as well.

The calculation engine can also be used without the GUI:

```powershell
.\build\bin\Release\leparagliding-engine.exe <design-file> <output-directory>
```

The main Qt executable exposes the same operation in headless mode:

```powershell
.\build\bin\Release\LEparagliding.exe --headless <design-file> <output-directory>
```

Both commands return the engine's exit code and generate `leparagliding.dxf`,
`lep-3d.step`, the reference `lep-3d.dxf`, `lep-out.txt`, `lines.txt`, and
`run-log.txt` in the selected output directory. Relative airfoil paths are
resolved from the design file's directory.

The 3.28 input format adds sections 33–37 for detailed risers, line
characteristics, equilibrium calculations, XFLR5 export, and special
parameters. Older 3.17 designs that end at section 32 remain usable: the
command-line boundary appends disabled defaults for the five new sections to a
temporary input file. It never rewrites the selected design.

## Build on Linux and macOS

The same CMake project builds with GCC or Clang. Two dependencies are
required:

- **Qt 6.5+** (Widgets) — distro packages, the Qt online installer, or
  [aqtinstall](https://github.com/miurahr/aqtinstall).
- **Open CASCADE 8.0** — newer than any distro or Homebrew package today, so
  build it from source once and install it to a prefix:

```sh
curl -fsSL https://github.com/Open-Cascade-SAS/OCCT/archive/refs/tags/V8_0_0_p1.tar.gz | tar xz
cmake -S OCCT-* -B occt-build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DINSTALL_DIR="$HOME/occt-install" \
    -DCMAKE_INSTALL_RPATH='$ORIGIN;@loader_path' \
    -DBUILD_MODULE_Draw=OFF -DBUILD_MODULE_DETools=OFF \
    -DUSE_TK=OFF -DUSE_TCL=OFF -DBUILD_DOC_Overview=OFF
cmake --build occt-build --target install
```

On Debian/Ubuntu the OCCT build needs
`ninja-build libfreetype-dev libfontconfig1-dev libx11-dev libxext-dev
libxi-dev libgl1-mesa-dev libglu1-mesa-dev libxkbcommon-dev`; on macOS
`brew install ninja freetype fontconfig`.

Then configure the project with both prefixes visible:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$HOME/occt-install;/path/to/Qt/6.x.y/gcc_64"
cmake --build build
ctest --test-dir build --output-on-failure
```

The release builds in `.github/workflows/ci.yml` run exactly this on
Ubuntu 22.04 (so AppImages run on any distro with glibc 2.35+) and macOS
15, caching the OCCT install tree between runs. A
third job builds with Ninja/MSVC on Windows against the official OCCT
prebuilt vc14-64 binaries (with their third-party runtime DLLs merged in)
and runs the Fortran byte-compare reference test.

## Releasing

The application version has a single source of truth: the `project(...
VERSION x.y.z)` line at the top of `CMakeLists.txt` (the ported calculation
core remains LEparagliding 3.28 regardless). CI reads it and stamps every
artifact name with it. To cut a release:

1. Bump the version in `CMakeLists.txt`, commit, and push.
2. `git tag vX.Y.Z && git push origin vX.Y.Z`

The tag build packages all three platforms — Windows zip (self-contained
Release folder), Linux AppImage, macOS DMG — creates the GitHub release,
and attaches the downloads. CI runs only on `v*` tags; for a build
without a release, use the Actions tab ("Run workflow", optionally with
"Upload artifacts to the GitHub release").

The macOS app is signed with the Developer ID certificate and notarized
when the repository's signing secrets are configured
(`MACOS_DEVELOPER_ID_P12_BASE64` and friends); without them CI produces an
unsigned DMG (right-click → Open on first launch). The signed Windows
installer remains the local `installer/build_installer.ps1` flow described
in `CLAUDE.md`.

## Port architecture

- `src/legacy/leparagliding_core.cpp` is the mechanically translated numerical
  and drawing core. It is built as C++, with Fortran indexing and I/O behavior
  retained for compatibility.
- `src/engine` supplies a small typed C++ boundary, input/output path handling,
  validation, and a command-line entry point.
- `src/model` interprets the fully transformed airfoil stations and analytical
  circular ballooning law directly. It converts each span arc to an exact
  rational B-spline, lofts semantic upper/vent/lower panel faces with OCCT,
  mirrors the calculated half wing, sews matching faces into shared shell
  topology while preserving designed intake openings, and writes an AP242
  STEP model in millimetres. The old tessellation is used only as a numerical
  regression oracle and remains available to the legacy DXF/STL exporters.
- `src/gui` is the Qt Widgets application. It runs the engine in a child
  process so the interface stays responsive and legacy input failures are
  isolated. Its viewport uses OCCT for STEP import, triangulation, and OpenGL
  presentation; it contains no application-side polygon model builder.
- `third_party/libf2c` is the portable runtime required by the translated I/O
  statements. Its original notice is included in that directory.

The translation boundary handles the Fortran features that `f2c` cannot
translate directly:

1. the Fortran 90 array-based word count was expressed as a character scan;
2. two whole-array negations were expanded to loops;
3. whole-array assignments used by the equilibrium solver were expanded to
   loops;
4. dynamic XFLR5/STL paths and directory creation were routed through the C++
   output boundary;
5. `kini=1` in `datair` was made explicit as `kini(i)=1`, matching how the
   array is subsequently consumed.

The C++ compatibility boundary also supplies reliable Windows `BACKSPACE`
record handling for LF and CRLF inputs and preserves GNU Fortran's formatted
output conventions. The regression fixture was generated by compiling
`leparagliding3.28.f` with native Windows `gfortran`: both DXFs, `lines.txt`,
and `run-log.txt` match byte-for-byte. The calculation report is compared
field-by-field with a 0.00015 display tolerance to accommodate signed zero and
two last-decimal rounding differences.

The active reference implementation is `leparagliding3.28.f` at the repository
root; the previous source is retained as `leparagliding3.17.f`. The original
source identifies itself as GNU GPL 3.0 software; the translated core is a
derivative under the same terms.
