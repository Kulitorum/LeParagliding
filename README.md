# LEparagliding C++ / Qt

This project is a C++ port of Pere Casellas' LEparagliding 3.17 “Z”
engineering program with a Qt 6 desktop interface.

The application accepts the same design text file as the original program.
Airfoil file references remain relative to the selected design file. Generated
files can be written to a separate folder:

- `leparagliding.dxf` — 2D manufacturing plans
- `lep-3d.dxf` — 3D wing geometry
- `lep-out.txt` — calculated design data
- `lines.txt` — suspension line data

The desktop application is a complete design studio:

- every numbered block in the selected design has its own syntax-highlighted
  editor;
- the `?` button on a section opens format guidance and a link to the full
  manual;
- **Build paraglider** validates and saves the design, runs the compatible C++
  engine, and reloads the generated 3D geometry;
- the viewport displays all `LINE` entities from `lep-3d.dxf`, with isometric,
  front, back, left, right, top, and bottom views plus perspective and
  orthographic projection.

Viewport navigation follows the slicer convention: drag with the left mouse
button to orbit, drag with the right or middle mouse button to pan, and use the
wheel to zoom. `Shift` + left drag also pans; double-click or press `F` to fit
the model. Number keys `0`–`6` select the preset views and `P` toggles the
projection. Double-click a completed output file to open it in its associated
viewer.

The original
[LEparagliding user manual](https://www.laboratoridenvol.com/leparagliding/manual.en.html)
documents every input section. In particular, its record order is strict and
blank lines are not valid records.

## Build on Windows

The CMake preset mirrors the compiler and Qt setup used by
`C:\CODE\cobod-slicer`: Visual Studio 2022 and the newest compatible Qt 6
MSVC kit under `C:\Qt`.

```powershell
cmake --preset windows
cmake --build --preset release --parallel
ctest --preset release
```

Run:

```powershell
.\build\bin\Release\LEparagliding.exe
```

`windeployqt` runs after the GUI build, so the build output is directly
runnable on the development machine.

The calculation engine can also be used without the GUI:

```powershell
.\build\bin\Release\leparagliding-engine.exe <design-file> <output-directory>
```

The main Qt executable exposes the same operation in headless mode:

```powershell
.\build\bin\Release\LEparagliding.exe --headless <design-file> <output-directory>
```

Both commands return the engine's exit code and generate `leparagliding.dxf`,
`lep-3d.dxf`, `lep-out.txt`, and `lines.txt` in the selected output directory.
Relative airfoil paths are resolved from the design file's directory.

## Port architecture

- `src/legacy/leparagliding_core.cpp` is the mechanically translated numerical
  and drawing core. It is built as C++, with Fortran indexing and I/O behavior
  retained for compatibility.
- `src/engine` supplies a small typed C++ boundary, input/output path handling,
  validation, and a command-line entry point.
- `src/gui` is the Qt Widgets application. It runs the engine in a child
  process so the interface stays responsive and legacy input failures are
  isolated.
- `third_party/libf2c` is the portable runtime required by the translated I/O
  statements. Its original notice is included in that directory.

Three translation-only compatibility edits were made before conversion:

1. the Fortran 90 array-based word count was expressed as a character scan;
2. two whole-array negations were expanded to loops;
3. `kini=1` in `datair` was made explicit as `kini(i)=1`, matching how the
   array is subsequently consumed.

The C++ compatibility boundary also restores the omitted `* 100` in the
flattening percentage expression, supplies reliable Windows `BACKSPACE`
record handling for LF and CRLF inputs, and matches GNU Fortran's numeric
output conventions for reproducible reference comparisons.

The original `leparagliding.f` remains at the repository root as the reference
implementation. The original source identifies itself as GNU GPL 3.0 software;
the translated core is a derivative under the same terms.
