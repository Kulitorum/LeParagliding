#pragma once

#include <QMetaType>
#include <QPolygonF>
#include <QVector>

#define _USE_MATH_DEFINES
#include <cmath>
#include <functional>

#include "flat_parts.h"

// Irregular nesting for the Print tab.
//
// The objective is page count, not raw density: the packed canvas is tiled into
// sheets, so both canvas dimensions get rounded up and a placement that raises
// the bounding rectangle costs whole pages even when it looks locally tight.
// That makes this fixed-width strip packing — minimise height for a given
// width — evaluated over a few candidate canvas widths.
//
// Placement works on rasterised outlines rather than polygons. A bitmask has no
// degenerate cases, handles concave parts and holes for free, and collision is
// a word-wise AND; the cost is that accuracy is bounded by the cell size, which
// at 1 mm against parts metres long is not a real limit. The quality comes from
// searching over placement order and rotation — mainly 180° flips, since the
// panels are near-parallelograms and the ribs tapered airfoils, and both
// interlock nose-to-tail.
namespace flatparts {

struct NestOptions
{
    // Printable area of one sheet, after margins.
    double pageWidthMm = 190.0;
    double pageHeightMm = 277.0;
    // Band each sheet repeats from its neighbour; sheets advance by
    // (page - overlap), so it changes how many sheets a canvas needs.
    double overlapMm = 10.0;
    // Clearance between parts, so there is somewhere to cut.
    double gapMm = 8.0;
    // Export scale applied to every part before packing.
    double scale = 1.0;
    // Raster cell. Smaller nests tighter and costs memory and time roughly
    // quadratically.
    double resolutionMm = 1.0;
    // Angular granularity of the rotations tried, in degrees.
    //
    // 90 is the value that matters: quarter turns keep the weave orthogonal to
    // the part, so the fabric behaves the same whichever way round the part is
    // cut. Any other angle lays the part on the bias, where a woven fabric
    // stretches — fine for a paper template you will reposition on the fabric
    // yourself, wrong for cutting fabric directly.
    //
    // Finer steps nest closer but multiply the rasterised masks, so the
    // resolution may be coarsened to stay inside the memory budget; the value
    // actually used comes back in NestResult::resolutionMm.
    int rotationStepDeg = 90;
    // Ceiling on total rasterised mask memory, in megabytes. Free rotation over
    // a whole wing would otherwise want most of a gigabyte.
    int maskBudgetMb = 512;
    // Machine-bed mode: no part may cross a sheet boundary. A bed load is cut
    // in one pass, so a part spanning two loads is cut in half — the opposite
    // of paper, where straddling a sheet edge is the whole point and the join
    // is taped. Implies sheets tile exactly, so the overlap is ignored.
    bool partsWithinOneSheet = false;
    // Widest canvas to consider, in sheets.
    int maxSheetsAcross = 10;
    // Ceiling on search effort. Zero or less runs until stopped, which is the
    // Print tab's default — packing keeps finding better layouts for as long as
    // it is given, so the sensible budget is the user's patience.
    int timeBudgetMs = 0;
};

struct NestResult;

// How the caller watches and stops a run. Both are called on whichever thread
// nest() runs on, so a GUI must hop to its own thread before touching widgets.
struct NestCallbacks
{
    // A new best layout. Never reports a worse one, so a live preview does not
    // flicker backwards.
    std::function<void(const NestResult &)> improved;
    // Checked every iteration, so stopping is prompt even during a long plateau
    // where nothing improves. Must be cheap.
    std::function<bool()> stopRequested;
};

struct Placement
{
    int pieceIndex = 0;     // index into FlatPartSet::pieces
    double x = 0.0;         // mm, left edge of the rotated part's bounding box
    double y = 0.0;         // mm, bottom edge, y-up from the canvas origin
    double rotationDeg = 0; // counter-clockwise
};

struct NestResult
{
    QVector<Placement> placements;
    // Pieces whose outline does not fit the canvas at any rotation. Non-empty
    // means the paper is too small for the design at this scale.
    QVector<int> unplaced;

    double canvasWidthMm = 0.0;
    double canvasHeightMm = 0.0;
    int sheetsAcross = 0;
    int sheetsDown = 0;
    int pageCount = 0;
    // Sum of part outline areas over canvas area. Reported, not optimised —
    // page count is the objective.
    double utilisation = 0.0;
    int iterations = 0;
    qint64 elapsedMs = 0;
    // Raster cell actually used. Free rotation may force this coarser than
    // requested to keep the rotated masks inside the memory budget.
    double resolutionMm = 1.0;
};

// Packs the given pieces (indices into set.pieces). Deterministic: the same
// inputs always produce the same layout, so a re-run does not silently
// reshuffle a layout the user has already started cutting from.
NestResult nest(const FlatPartSet &set,
                const QVector<int> &pieceIndices,
                const NestOptions &options,
                const NestCallbacks &callbacks = {});

// The outline a piece is packed against: its cut-role geometry chained into one
// closed boundary. Exposed for the preview, which draws exactly what was
// packed, and for tests.
QPolygonF outerBoundary(const FlatPiece &piece);

// Where a placed piece's own coordinates land on the canvas. The preview and
// the PDF both need this, and both must agree with the packer exactly — a
// preview drawn by a second, independently written transform would disagree
// about rotation sign or origin and show a layout that is not the one packed.
struct PlacementFrame
{
    double rotationDeg = 0.0;
    double scale = 1.0;
    QPointF offset;

    QPointF map(const QPointF &local) const
    {
        const double radians = rotationDeg * M_PI / 180.0;
        const double c = std::cos(radians);
        const double s = std::sin(radians);
        const double x = local.x() * scale;
        const double y = local.y() * scale;
        return QPointF(x * c - y * s, x * s + y * c) + offset;
    }
};

PlacementFrame frameFor(const FlatPiece &piece,
                        const Placement &placement,
                        double scale);

} // namespace flatparts

// Packing runs on a worker thread and reports back by signal, which needs the
// result to be a registered metatype.
Q_DECLARE_METATYPE(flatparts::NestResult)
