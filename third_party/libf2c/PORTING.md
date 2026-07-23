# Local portability changes

This is the Netlib `libf2c` runtime distributed under the terms in `Notice`.
The source list follows its Visual C++ makefile.

Local changes are deliberately limited to the Windows boundary:

- the Linux-specific custom `ctype.h` shim is not used (`NO_My_ctype`);
- the five single-complex math sources undefine modern MSVC's `complex`
  compatibility macro after including the C math header;
- existing formatted input is opened in binary mode and treats both CR and LF
  as separators, so formatted `BACKSPACE` has stable record positions for
  Unix-LF and Windows-CRLF files;
- formatted `BACKSPACE` seeks only to positions returned by `FTELL`;
- `FOPEN` and `FREOPEN` route through UTF-8-to-wide Windows adapters supplied
  by the C++ engine, allowing non-ASCII design and output paths.
