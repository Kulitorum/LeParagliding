# XFLR5 (vendored)

Source: SourceForge SVN, `https://svn.code.sf.net/p/xflr5/code/tags/v6.62/xflr5`
(tag v6.62, released 2026-03-24; repository head r1506). Mirrored over the
SVN DAV HTTP interface on 2026-07-26.

Copyright (C) André Deperrois and contributors, GPL v3 (see `License.txt`) —
the same license as this project. Upstream is no longer maintained
(SourceForge project status: Abandoned), so this copy is expected to diverge
via local patches rather than track upstream.

## What is vendored

- `XFoil-lib/` — the XFoil solver library (C++ translation of Mark Drela's
  XFoil), one translation unit.
- `xflr5v6/` — the complete XFLR5 v6 application: xdirect (2D foil
  analysis), miarex (3D wing/plane analysis), xinverse, afoil design,
  xflobjects data model, xfl3d OpenGL views, plus their `.qrc` resources.
- `License.txt` — GPL v3.

Omitted from upstream `tags/v6.62/xflr5`: `doc/`, `doxygen/`, `linux/`,
`mac/`, `win/`, `res/`, `translations/` (packaging and docs for the
standalone application; not needed to embed).

The upstream qmake files (`*.pro`, `*.pri`) are kept for reference but the
build is driven by the root `CMakeLists.txt` (`xfoil` and `xflr5core` static
libraries). `xflr5v6/globals/main.cpp` and `globals/xflr5app.*` are excluded
from the build: LEparagliding Studio provides the QApplication and hosts
`MainFrame` as an embedded widget.

## Local patches (embedding support)

1. `XFoil-lib/xfoil-lib_global.h` — added `XFOILLIB_STATIC` branch so the
   export macro expands to nothing in a static build.
2. `xflr5v6/globals/mainframe.cpp` — `MainFrame::MainFrame` assigns `_self`
   as its first statement (internal `MainFrame::self()` calls during
   construction must find the instance being built when it is constructed
   directly with a parent) and only sets `Qt::WA_DeleteOnClose` when
   top-level (deleting the embedded singleton would leave the host tab
   dangling).
3. `xflr5v6/misc/options/settingswt.cpp` — XFLR5's Preferences dialog
   applied styles application-wide (`qApp->setStyle`,
   `qApp->setStyleSheet`); when `MainFrame` is embedded these are scoped to
   the `MainFrame` subtree so they cannot restyle the host application.
4. `xflr5v6/globals/mainframe.{h,cpp}` — added
   `MainFrame::lepImportLepWing(dir, error)`: imports a LEparagliding
   Section-36 export (rib `*.dat` foils via `Objects2d::insertThisFoil`,
   the `*.xwimp` wing via `Wing::importDefinition` as a wing-only `Plane`),
   silently replacing a previously imported plane of the same name, then
   selects it in the Miarex module. Mirrors `Miarex::onNewPlane`'s accept
   path; `Objects3d::setModPlane` is not used because it prompts.
