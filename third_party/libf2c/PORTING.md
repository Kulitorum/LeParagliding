# Local portability changes

This is the Netlib `libf2c` runtime distributed under the terms in `Notice`.
The source list follows its Visual C++ makefile.

One change is platform-width related: `f2c.h` pins `integer`, `logical`,
`flag`, `ftnlen`, and `ftnint` to plain `int` instead of `long int`, so LP64
platforms (Linux/macOS) keep the 4-byte Fortran INTEGER layout the reference
outputs were validated with on Windows.

The remaining changes are deliberately limited to the Windows boundary:

- the Linux-specific custom `ctype.h` shim is not used (`NO_My_ctype`);
- the five single-complex math sources undefine modern MSVC's `complex`
  compatibility macro after including the C math header;
- existing formatted input is opened in binary mode and treats both CR and LF
  as separators, so formatted `BACKSPACE` has stable record positions for
  Unix-LF and Windows-CRLF files;
- formatted `BACKSPACE` seeks only to positions returned by `FTELL`;
- `FOPEN` and `FREOPEN` route through UTF-8-to-wide Windows adapters supplied
  by the C++ engine, allowing non-ASCII design and output paths.
