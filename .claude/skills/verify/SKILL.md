---
name: verify
description: Build, launch, and capture runtime evidence for LEparagliding Studio GUI changes on this machine
---

# Verifying LEparagliding Studio GUI changes

Build & launch:

```powershell
cmake --preset windows
cmake --build build --config Release --target LEparagliding
Start-Process build\bin\Release\LEparagliding.exe   # restores last design from QSettings (paths/input)
```

Gotchas learned the hard way:

- **LNK1104 on LEparagliding.exe** → a running instance locks it. Do NOT
  kill an instance you didn't start (it may hold the user's unsaved work);
  `Rename-Item` the exe aside — Windows lets a running process's file be
  renamed — then relink.
- **Screenshots: never `SetForegroundWindow` + `CopyFromScreen`.** The user
  works interactively on this machine; focus stealing is blocked and the
  capture photographs whatever window happens to be on top. Use
  `PrintWindow(hwnd, hdc, 2)` (PW_RENDERFULLCONTENT — required for Qt's
  composited windows); it renders background windows without touching focus.
- **Never inject global mouse/keyboard input** — clicks land in the user's
  live session.
- **`PrintWindow` cannot capture the Playground's 3D view.** That widget sets
  `WA_NativeWindow` and owns a native swapchain, so it comes back blank while
  the surrounding chrome captures fine. Verify the Playground from its status
  line (it carries the node/quad counts, and any solver exception replaces it
  with the error) plus process metrics, and say plainly that the wing itself
  was not photographed. Per-launch it is FLAKY, not consistent: some launches
  capture the GL content and miss a custom-painted sibling (the legend strip)
  instead, and one launch in several comes back as a white page with widget
  islands (the GL-composition flip; relaunch and recapture). Never conclude a
  widget does not render from a PrintWindow capture alone — prove widget
  painting with `widget->grab()` in the offscreen harness (works for plain
  widgets, needs no GL), and treat the capture as corroboration only.
- **Prefer a command-line entry point over driving the UI.** `--playground
  <lep-sim.json>` opens straight onto the tab with a mesh loaded, `--xflr5`
  onto the aerodynamics tab; no preview run or clicking needed.
- **`Add-Type` with System.Drawing differs across PowerShell editions.** On
  PS7/.NET 9 a C# block using `Bitmap`/`Graphics` needs
  `System.Private.Windows.Core`, and on 5.1 the `-ReferencedAssemblies` list
  that fixes PS7 fails instead. Keep the C# to the P/Invoke signatures and do
  the bitmap work in PowerShell after `Add-Type -AssemblyName System.Drawing`.

Interaction testing without the user's session: build an offscreen harness —
a tiny CMake project compiling the widget sources from `src/gui` plus a
`main.cpp`, linked against `Qt6::Widgets`
(`-DCMAKE_PREFIX_PATH=C:/Qt/6.11.1/msvc2022_64`, `CMAKE_AUTOMOC ON`), run
with `QT_QPA_PLATFORM=offscreen`. Drive widgets with
`QApplication::sendEvent` QMouseEvents; locate hit targets by hover-scanning
and reading `widget->cursor().shape()`; capture pixels with
`widget->grab().save(...)` (offscreen has no fonts — glyphs render as boxes,
structure is still verifiable). Working example from the Section 1 curve
editor verification: press → 8×5px move steps → release exercised the full
drag/commit path.
