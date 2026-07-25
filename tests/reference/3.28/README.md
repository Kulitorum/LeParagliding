# LEparagliding 3.28 native Fortran reference

These text files and the DXF hashes in
`cmake/CompareReferenceRun.cmake` were generated on Windows with:

```powershell
gfortran -O0 -o lep328.exe leparagliding3.28.f
.\lep328.exe
```

The input was `tests/fixtures/3.28/leparagliding.txt` with the adjacent
`gnuC2.txt` profile.

The translated C++ engine produces byte-identical 2D and 3D DXFs,
`lines.txt`, and `run-log.txt`. Its `lep-out.txt` has the same records and
numeric results; six displayed signed zeroes and two values rounded to four
decimals differ in their last character. The report comparator therefore uses
an absolute tolerance of 0.00015 while still requiring the same line structure
and field count.
