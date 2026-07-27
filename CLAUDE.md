# LEparagliding Studio

Qt 6 / Open CASCADE (OCCT) port of LEparagliding 3.28. GUI in `src/gui`,
CLI engine in `src/engine` + translated Fortran core in `src/legacy`, exact
NURBS/STEP model builder in `src/model`.

## Build & test

```powershell
cmake --preset windows          # configure (VS 2022 x64, auto-detects Qt + OCCT)
cmake --build --preset release  # build; output in build/bin/Release (self-contained)
ctest --preset release          # run the test suite
```

The Release output folder is directly runnable: post-build steps run
windeployqt and copy the OCCT runtime DLLs next to the executables.

### Linux verification via WSL

The WSL Ubuntu distro on this machine keeps a persistent Linux build
environment: OCCT 8.0.0p1 installed at `~/occt-install` (built with
`-DCMAKE_INSTALL_RPATH='$ORIGIN'` so transitively loaded toolkits resolve)
and Qt 6.8.3 at `~/Qt/6.8.3/gcc_64`. Configure with
`-DCMAKE_PREFIX_PATH="$HOME/occt-install;$HOME/Qt/6.8.3/gcc_64"`.

Building from `/mnt/c` works for a quick check, but `/mnt/c` is
case-insensitive and masks case bugs that break the CI runners (a stray
`RESOURCES` source entry once configured fine locally and failed on ext4).
For a faithful check, copy the tracked tree to ext4 first:
`git ls-files -z | tar --null -T - -cf - | tar -xf - -C ~/lep-src`.

Run multi-line WSL commands as a script piped through `tr -d '\r'`
(`wsl.exe -- bash -c "tr -d '\r' < /mnt/c/...sh | bash"`): wsl.exe
re-evaluates inline command strings, mangling quotes and `$variables`,
and CRLF line endings break bash. Package installs: `wsl -u root -- apt-get
install -y ...` (no password needed, unlike sudo).

## Bundled LEparagliding manual

`resources/manual/` (compiled in via `manual.qrc`) holds the official
manual converted for the in-app help: per-section chapters shown in the
section help dialogs (`SectionHelp::manual`) and `manual_full.html` for
the Manual button's offline popup. It is **generated** — do not hand-edit;
rerun `python tools/extract_manual.py` when the upstream manual changes
(caches the download as the gitignored `tools/manual.en.html`; delete that
to re-fetch). The converter whitelists Qt-rich-text-safe markup, remaps
colors for the dark theme, and gives `<a name>` anchors zero-width-space
content because Qt's rich-text parser drops empty anchors (breaking
in-popup TOC navigation). When testing anchor navigation offscreen, note
`scrollToAnchor` silently defers until the widget is visible.

## GitHub releases

Version source of truth is `project(... VERSION x.y.z)` in CMakeLists.txt.
To release: bump it, commit, push, then `git tag vX.Y.Z && git push origin
vX.Y.Z` — CI builds Windows/Linux/macOS and attaches the downloads (builds
run only on tags or manual dispatch; the `platforms` input limits which
platforms rebuild).

**Every GitHub release must get a proper hand-written description** — never
leave bare auto-generated notes. It must contain: a one-paragraph
what-is-this for first-time visitors, the changes in this version, and a
per-platform install table (Windows: unzip & run; Linux: `chmod +x` the
AppImage; macOS: unsigned — right-click → Open). Apply it after the CI
uploads finish:

```powershell
gh release edit vX.Y.Z --title "LEparagliding Studio X.Y.Z" --notes-file notes.md
```

## Building the signed installer

One command (from anywhere):

```powershell
pwsh C:\CODE\LeParagliding\installer\build_installer.ps1
```

It reconfigures CMake (regenerates `installer/version.iss` with the HEAD
commit hash), builds Release, compiles `installer/LEparagliding-installer.iss`
with Inno Setup, signs everything, and verifies the result with
`Get-AuthenticodeSignature`. Output: `installer/Output/
LEparagliding_v<version>-<githash>_winx64_installer.exe`.

**Commit and push BEFORE building an installer you intend to distribute** —
the commit hash is baked into the filename at CMake configure time, so
building from a dirty tree stamps a hash that doesn't match the content.

What gets signed (COBOD International A/S certificate, SafeNet USB token):
`LEparagliding.exe` and `leparagliding-engine.exe` (via `Flags: sign` in the
.iss, each listed once and excluded from the wildcard) plus the installer
itself (`SignTool=SafeNet $f`).

### Manual recipe / how signing works (learned from C:\CODE\cobod-slicer)

ISCC.exe does **not** read the Inno Setup IDE's sign-tool registry config
(`HKCU:\Software\Jordan Russell\Inno Setup\SignTools`), so the SafeNet
definition must be passed via `/S`. Embedded quotes in the `/S` argument get
mangled by PowerShell native-arg passing (symptom: "You may not specify more
than one script filename"), so the recipe is quote-free: 8.3 short path for
signtool and certificate selected by thumbprint:

```powershell
$sign = '/SSafeNet=C:\PROGRA~2\WI3CF2~1\10\bin\100261~1.0\x86\signtool.exe sign /sha1 C8CD08A6B254958D769848CC047F1C7E79FC3A84 /tr http://timestamp.sectigo.com /td sha256 /fd sha256 $p'
& "C:\Program Files (x86)\Inno Setup 6\ISCC.exe" $sign "C:\CODE\LeParagliding\installer\LEparagliding-installer.iss"
```

- Certificate: COBOD International A/S (SafeNet token, `Cert:\CurrentUser\My`),
  thumbprint `C8CD08A6B254958D769848CC047F1C7E79FC3A84`, expires 2028-04-07.
  If signing fails, re-query: `Get-ChildItem Cert:\CurrentUser\My | ? Subject
  -like '*COBOD*'`. A SafeNet PIN dialog may pop up (usually cached) — if a
  headless build hangs inside signtool, that dialog is why.
- Verify: `Get-AuthenticodeSignature installer\Output\<file>.exe` → `Valid`.

### Machine-specific paths (override with ISCC `/D` if they differ)

- `AppBuildDir` — defaults to `<repo>\build\bin\Release`
- `MsvcRedist_Dir` — defaults to VS 2022 Professional's
  `VC\Redist\MSVC\v143` (the installer bundles and runs `vc_redist.x64.exe`)
- Inno Setup 6: `C:\Program Files (x86)\Inno Setup 6\ISCC.exe`
- signtool 8.3 path above = Windows SDK 10.0.26100 x86 signtool

`installer/version.iss` and `installer/Output/` are generated and gitignored;
`installer/version.iss.in` is the template.
