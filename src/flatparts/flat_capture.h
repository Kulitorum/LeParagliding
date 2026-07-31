#pragma once

/*
 * 2D flat-part capture for the translated Fortran core.
 *
 * The core draws every 2D manufacturing plan through line_/circle_/txt_ into
 * one flat DXF (unit 20) laid out as an 8x4 grid of 1260 x 890.95 boxes. The
 * DXF has no grouping — no polylines, one layer, integer-only labels — so the
 * parts are not recoverable from it afterwards. These hooks tag the drawing
 * while it happens: each part-drawing loop brackets itself with
 * lep_flat_begin_part/lep_flat_end_part, and the drawing primitives report
 * every entity they emit. Nothing here changes a numerical result.
 *
 * Declared C-clean: the core compiles as C on non-MSVC toolchains.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Design-wide context, set once the input file has been read. The drawing
   scale is section 1's xkf: all box and drawing coordinates are centimetres
   multiplied by it, so the writer divides it back out. */
void lep_flat_set_design(double drawingScale,
                         const char *wingName,
                         int wingNameLength);

/* Flat and projected area in m2, for the Print tab's "scale to target area"
   control. Reported once they have been computed, well after the design is
   set. */
void lep_flat_set_areas(double flatArea, double projectedArea);

/* Opens a part. Geometry emitted until lep_flat_end_part is attributed to it,
   re-origined to (originX, originY) and clipped to box (boxRow, boxColumn) —
   several drawing loops interleave a reference copy of the same part one box
   row away, and the box is what tells them apart. Category is a stable token
   ("rib", "extrados-panel", ...); index is the part number shown on the plan,
   subIndex distinguishes multi-piece parts (0 when there is only one). */
void lep_flat_begin_part(const char *category,
                         int categoryLength,
                         int index,
                         int subIndex,
                         double originX,
                         double originY,
                         int boxRow,
                         int boxColumn);
void lep_flat_end_part(void);

/* Reopens the already-captured part of `category` whose origin is at
   (originX, originY), for the many later passes that add marks to a rib or
   panel drawn earlier — anchor points, air-intake marks, V-rib positions,
   equidistant seam marks. Matching on the origin rather than the part index is
   deliberate: those passes recompute the layout slot from expressions that
   differ by one between sites, so an index would be easy to get subtly wrong
   and would silently mark the neighbouring part. Opens nothing when there is
   no match, so unrecognised geometry is dropped rather than misattributed. */
void lep_flat_resume_at(const char *category,
                        int categoryLength,
                        double originX,
                        double originY);

/* Drawing primitives. Coordinates are in the line_() argument frame — that is,
   before line_ negates y on its way into the DXF — because the box grid in
   section 5.3 is expressed in that same frame. */
void lep_flat_capture_line(double x1,
                           double y1,
                           double x2,
                           double y2,
                           int color);
void lep_flat_capture_circle(double x,
                             double y,
                             double radius,
                             int color);
void lep_flat_capture_text(double x,
                           double y,
                           double height,
                           const char *text,
                           int textLength);
/* Separate entry point for itxt_/itxt2_, whose label is an integer: formatting
   it here keeps the call sites free of local declarations, which the core's
   C89 body cannot introduce mid-function. */
void lep_flat_capture_integer_text(double x,
                                   double y,
                                   double height,
                                   int value);

/* Chains the captured segments into polylines and writes the part file.
   Called once, after the 2D DXF is closed. */
void lep_flat_write(const char *path);

#ifdef __cplusplus
}
#endif
