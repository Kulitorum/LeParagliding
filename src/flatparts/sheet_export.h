#pragma once

#include <QRectF>
#include <QString>
#include <QStringList>
#include <QVector>

#include "flat_parts.h"
#include "nesting.h"

// Writing a packed layout out: tiled PDF for paper, DXF for a cutter.
//
// The nester produces one continuous canvas; both writers turn that into
// something you can put on a table. The PDF cuts it into sheets and prints them
// at 1:1 with alignment furniture, because a wing is metres long and a printer
// is not. The DXF hands the same geometry to a machine, where the layers carry
// the cut/stitch/mark distinction the operator needs and nothing is scaled.
//
// Both live below the GUI so the layout that gets written is the one that got
// packed — the exporters read the same NestResult and use the same
// flatparts::frameFor() as the on-screen preview, rather than re-deriving where
// a part sits and disagreeing about rotation sign or origin.
namespace flatparts {

struct ExportOptions
{
    // Physical sheet, before margins. The nester works in printable area, so
    // this is the only place the paper's real size is known; the printable
    // rectangle is centred inside it.
    double sheetWidthMm = 210.0;
    double sheetHeightMm = 297.0;

    // Printed in the sheet header so a stack of paper on a workshop bench can
    // still be identified. Empty entries are left out.
    QString title;
    QString subtitle;
    // Supplied by the caller rather than read from the clock, so a test can
    // compare two exports byte for byte.
    QString stamp;

    bool drawSeamLines = true;
    bool drawMarks = true;
    bool drawLabels = true;
    // Border, registration crosses, overlap guides, sheet header and the
    // calibration ruler. Off for a cutting bed, where none of it means
    // anything and all of it would be cut out.
    bool drawFurniture = true;
    // Sheets no part reaches. Off by default: a nested canvas routinely leaves
    // a corner empty and printing that corner wastes a sheet of paper.
    bool includeEmptySheets = false;

    // PDF raster resolution. Only affects the precision of the vector
    // coordinates the writer emits, not the file's structure.
    int resolutionDpi = 1200;

    // Write one DXF per sheet instead of one file for the whole canvas. What a
    // cutting bed wants: each file is one bed load, in bed coordinates.
    bool splitSheets = false;
};

// One sheet of the tiled canvas. Sheets advance by (page - overlap), so
// neighbours share a band; on a bed the overlap is zero and they tile exactly.
struct Sheet
{
    int column = 0; // 0-based, left to right
    int row = 0;    // 0-based, counted from the top of the canvas
    // Bottom-left corner of the region this sheet shows, in canvas mm, y-up.
    QPointF originMm;
    double widthMm = 0.0;
    double heightMm = 0.0;
    // Indices into NestResult::placements that reach this sheet.
    QVector<int> placements;

    bool isEmpty() const { return placements.isEmpty(); }
};

// Every sheet the canvas tiles into, in printing order (left to right, top to
// bottom), each carrying the placements that reach it. Includes empty sheets;
// the writers drop them unless asked not to.
QVector<Sheet> enumerateSheets(const FlatPartSet &set,
                               const NestResult &result,
                               const NestOptions &options);

// Canvas-space bounding box of a placed piece, including its marks and labels —
// not just the cut outline, so nothing a sheet should have shown gets clipped
// away by the sheet-assignment test.
QRectF placementBounds(const FlatPartSet &set,
                       const Placement &placement,
                       double scale);

// Placements whose drawn geometry reaches outside the sheet grid, and so would
// be clipped away by the writers.
//
// The nester positions a part by its chained outer boundary, and on real
// exports a few parts carry cut geometry that boundary does not cover — a
// handful of millimetres, but millimetres of cut line that would go missing
// from the printed sheet without a word. Labels are excluded from the test:
// their drawn width is only ever estimated here, and a label overhanging its
// part is cosmetic where a cut line is not.
QVector<int> clippedPlacements(const FlatPartSet &set,
                               const NestResult &result,
                               const NestOptions &options);

// Tiled PDF, one page per non-empty sheet, drawn at true size.
//
// Returns false and sets errorMessage on any write failure. sheetsWritten, when
// given, receives the number of pages actually produced, which is below
// NestResult::pageCount whenever empty sheets were skipped.
bool exportPdf(const QString &path,
               const FlatPartSet &set,
               const NestResult &result,
               const NestOptions &options,
               const ExportOptions &exportOptions,
               QString *errorMessage,
               int *sheetsWritten = nullptr);

// DXF R12 of the packed canvas. Layers: CUT, SEAM, MARK, TEXT, SHEET.
//
// R12 with POLYLINE/VERTEX rather than anything newer because plotters and
// cutting tables are the least forgiving DXF readers in the trade, and R12 is
// the dialect all of them accept.
//
// With ExportOptions::splitSheets the path is used as a stem and one file is
// written per sheet in that sheet's own coordinates. filesWritten, when given,
// receives every path written.
bool exportDxf(const QString &path,
               const FlatPartSet &set,
               const NestResult &result,
               const NestOptions &options,
               const ExportOptions &exportOptions,
               QString *errorMessage,
               QStringList *filesWritten = nullptr);

} // namespace flatparts
