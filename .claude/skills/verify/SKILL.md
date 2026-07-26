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
