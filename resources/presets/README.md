# Wing presets

Complete LEparagliding design files (with their airfoil files) for the wings
published by Pere Casellas on [Laboratori d'envol](https://www.laboratoridenvol.com/pindex.en.html#ales),
retrieved 2026-07-25. The designs are published by their author under the
GNU General Public License; brand and wing names belong to their authors.

`presets.json` is the catalog the GUI reads: one entry per wing with its
category, the URL of the wing's project web page, and one sub-entry per size
variant pointing at `<wing>/<variant>/leparagliding.txt`. Every design file
ships together with the airfoil `.txt` files its section 2 references, so a
copied preset folder is self-contained input for the engine.

Every file in this tree has been validated against the bundled 3.28 engine
(a full calculation completes without errors).

## Adaptations relative to the published originals

The published files span input-format versions 2.35 through 3.25; the
following mechanical adaptations were applied so every preset loads in the
3.28 engine. Nothing aerodynamic was altered.

- Old-format files (BHL and gnuA series among others): the missing trailing
  sections up to 37 were appended in the disabled state (`0`), exactly like
  the engine's own automatic migration does for sections 33-37.
- `BHL7/10`, `BHL7/22`: the published files contain section 32 twice
  (a copy/paste slip); the duplicated block was removed.
- `BHL7-19-lp`: the section 31 table used Unicode "en space" characters,
  which Fortran list-directed input cannot read; replaced with ASCII spaces
  (all files were normalised the same way).
- `Chooca-15`: the published file starts directly at "* Brand name" without
  the banner/section-1 header block the reader skips; a standard header was
  prepended.
- `gnuPSF-evo-19/v4`: section 28 uses an experimental calage syntax unknown
  to 3.28; the section was set to disabled. Its airfoil `gnuPSF.txt` is
  linked as a dead URL in the v4 folder and was taken from the sibling
  `evo/` folder instead.
- `gnuA7/26-3r`: the "Kit three risers" folder does not carry its own
  `gnua.txt`; taken from the parent `data26/` folder. The published file also
  ends with a "BELOW COMMENTS (not read)" block (blank lines, a stray copy of
  the section 11 header, and size-scale notes) that the Fortran reader never
  reaches but that the Studio editor rejects; the block was removed.
- `Swoop/original`: after section 37 the published file appends an
  alternative enabled copy of sections 35-37 (equilibrium + XFLR5 template).
  The Fortran reader stops at the first copy, so the appended block never
  took part in the calculation, but its duplicated section numbers made the
  Studio editor reject the file; the appended block was removed.

## Not included

- **BHL5-16** (projects/BHL5-SA): the data-file link is dead on the site
  (HTTP 404).
- **gnuA13-M / gnuA13-HA**: their pages link the base gnuA13 data files;
  there is no distinct design file to package.
- **gnuLAB2 / gnuLAB3** and the remaining wings of the index (gnuLAB1,
  Barretina 611, Ghost 1/2, gnuEASY, Snow Owl, Accuracy, Macaw, hang
  gliders, ...): no LEparagliding design file is published for them, only
  drawings, airfoil files, or nothing.
