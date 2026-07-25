# LEparagliding C++ / Qt

This project is a C++ port of Pere Casellas' LEparagliding 3.28 “Jardins”
engineering program with a Qt 6 desktop interface.

The application accepts the same design text file as the original program.
Airfoil file references remain relative to the selected design file. Generated
files can be written to a separate folder:

- `leparagliding.dxf` — 2D manufacturing plans
- `lep-3d.dxf` — 3D wing geometry
- `lep-out.txt` — calculated design data
- `lines.txt` — suspension line data
- `run-log.txt` — calculation progress and diagnostics

Version 3.28 can also create `stl` and `xflr5` subdirectories when the
corresponding design options are enabled.

The desktop application is a complete design studio:

- every numbered block in the selected design has its own syntax-highlighted
  editor and independent Undo/Redo history;
- every Save or Build embeds the wing's complete version history in the design
  file, so Undo/Redo can continue across restarts and the `Versions...` window
  can restore the whole wing to any saved state;
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

While editing a section, press `Enter` to save, build, and refresh the 3D
viewport. Use `Shift+Enter` when you intentionally need to insert another input
record. `Ctrl+Z`/`Ctrl+Y` and the section's Undo/Redo buttons affect only the
currently visible section; switching sections does not merge or clear their
histories. Once the live editor history is exhausted, Undo continues through
that section's saved versions. Restoring an older whole-wing version does not
delete newer versions; the restored state becomes a new latest version when it
is next saved.

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
`lep-3d.dxf`, `lep-out.txt`, `lines.txt`, and `run-log.txt` in the selected
output directory. Relative airfoil paths are resolved from the design file's
directory.

The 3.28 input format adds sections 33–37 for detailed risers, line
characteristics, equilibrium calculations, XFLR5 export, and special
parameters. Older 3.17 designs that end at section 32 remain usable: the
command-line boundary appends disabled defaults for the five new sections to a
temporary input file. It never rewrites the selected design.

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
