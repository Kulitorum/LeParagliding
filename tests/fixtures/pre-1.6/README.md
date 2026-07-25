# Geometry pre-processor reference fixtures

Each directory holds a `pre-data.txt` input and the `geometry-out.txt` that the
original Fortran pre-processor v1.6 "Canigó" (`pre-processor.f`, GPL-3.0,
https://www.laboratoridenvol.com/leparagliding/pre.en.html) produces for it,
compiled with gfortran 15.2 on Windows x64.

- `basic` — the sample input shipped with v1.6 (vault type 2, cell
  distribution 3, 50 cells); its `geometry-out.txt` is byte-identical to the
  one published on the website.
- `vault1-cells1-odd` — vault type 1 (ellipse + cosine), uniform cells, odd
  count (45). The reference wingtip rib prints y-LE/y-TE as 0.00 — a rounding
  artifact of the original that the C++ port intentionally does not reproduce
  (the comparator skips the affected fields).
- `vault2-cells2-even` — vault type 2, linear cell distribution, 40 cells.
- `vault2-cells4` — vault type 2, explicit cell widths (10 cells).

`preprocessor_reference_compare.cpp` runs the C++ port on each `pre-data.txt`
and checks the rib matrix against the reference within 0.03 cm / 0.06°.
