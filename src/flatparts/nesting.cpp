#include "nesting.h"

#include <QElapsedTimer>
#include <QLineF>
#include <QPolygonF>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

namespace flatparts {
namespace {

// ---------------------------------------------------------------- outlines

double polygonArea(const QPolygonF &polygon)
{
    double twice = 0.0;
    for (int i = 0, n = polygon.size(); i < n; ++i) {
        const QPointF &a = polygon.at(i);
        const QPointF &b = polygon.at((i + 1) % n);
        twice += a.x() * b.y() - b.x() * a.y();
    }
    return std::abs(twice) * 0.5;
}

// Explicit extents. QRectF names its edges for Qt's y-grows-down convention,
// so on the y-up geometry here top() is the minimum and bottom() the maximum —
// reversed from how they read. Naming them plainly avoids the confusion.
struct Extent
{
    double minX = 0.0;
    double minY = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
};

Extent extentOf(const QPolygonF &polygon)
{
    Extent extent;
    if (polygon.isEmpty()) {
        return extent;
    }
    extent.minX = extent.maxX = polygon.first().x();
    extent.minY = extent.maxY = polygon.first().y();
    for (const QPointF &point : polygon) {
        extent.minX = std::min(extent.minX, point.x());
        extent.maxX = std::max(extent.maxX, point.x());
        extent.minY = std::min(extent.minY, point.y());
        extent.maxY = std::max(extent.maxY, point.y());
    }
    return extent;
}

QPolygonF convexHull(QVector<QPointF> points)
{
    if (points.size() < 3) {
        return QPolygonF(points);
    }
    std::sort(points.begin(), points.end(), [](const QPointF &a, const QPointF &b) {
        return a.x() != b.x() ? a.x() < b.x() : a.y() < b.y();
    });
    const auto cross = [](const QPointF &o, const QPointF &a, const QPointF &b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
    QVector<QPointF> hull(2 * points.size());
    int k = 0;
    for (const QPointF &point : points) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], point) <= 0) {
            --k;
        }
        hull[k++] = point;
    }
    const int lower = k + 1;
    for (int i = points.size() - 2; i >= 0; --i) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], points[i]) <= 0) {
            --k;
        }
        hull[k++] = points[i];
    }
    hull.resize(k - 1);
    return QPolygonF(hull);
}

} // namespace

QPolygonF outerBoundary(const FlatPiece &piece)
{
    QVector<Polyline> fragments = piece.cutOutline();
    QVector<QPointF> allPoints;
    for (const Polyline &fragment : fragments) {
        allPoints.append(fragment.points);
    }
    if (allPoints.isEmpty()) {
        // No cut role at all: fall back to everything drawn, so a part with an
        // unrecognised colour still packs as something sensible.
        for (const Polyline &polyline : piece.polylines) {
            allPoints.append(polyline.points);
        }
    }
    if (allPoints.size() < 3) {
        return QPolygonF(QRectF(QPointF(0.0, 0.0), piece.size));
    }

    // The convex hull is the safe answer: never smaller than the part, so a
    // pack built on it can never overlap. Everything below tries to beat it and
    // falls back to it on any doubt.
    const QPolygonF hull = convexHull(allPoints);
    const double hullArea = polygonArea(hull);

    // The boundary arrives as open chains, and the trick is telling them from
    // everything else sharing the cut role.
    //
    // It is not a cycle, so neither spur-pruning nor outer-face tracing
    // reconstructs it: a rib's contour is one long open run, and a panel's is
    // two — dpanelcc_ omits the corner segments that would join its lateral
    // edges whenever the leading or trailing edge is suppressed. Mixed in are
    // the surface tick marks (short open runs) and, on ribs, the lightening
    // holes (closed runs).
    //
    // Length separates them cleanly: a rib's contour is metres long and its
    // tick marks tens of millimetres, so anything far shorter than the longest
    // run is a mark. Holes are excluded by being closed. What survives is the
    // boundary and only the boundary, which is then safe to join end to end —
    // the same greedy join that wandered off into self-intersection when the
    // marks were still in the pool.
    QVector<QVector<QPointF>> chains;
    double longest = 0.0;
    const auto pathLength = [](const QVector<QPointF> &points) {
        double length = 0.0;
        for (int i = 1; i < points.size(); ++i) {
            length += QLineF(points.at(i - 1), points.at(i)).length();
        }
        return length;
    };
    for (const Polyline &fragment : fragments) {
        if (fragment.closed || fragment.points.size() < 2) {
            continue;
        }
        longest = std::max(longest, pathLength(fragment.points));
    }
    if (longest > 0.0) {
        for (const Polyline &fragment : fragments) {
            if (fragment.closed || fragment.points.size() < 2) {
                continue;
            }
            if (pathLength(fragment.points) >= longest * 0.2) {
                chains.append(fragment.points);
            }
        }
    }

    if (!chains.isEmpty()) {
        QVector<QPointF> loop = chains.takeFirst();
        while (!chains.isEmpty()) {
            int bestIndex = 0;
            bool bestReversed = false;
            double bestDistance = std::numeric_limits<double>::max();
            const QPointF tail = loop.last();
            for (int i = 0; i < chains.size(); ++i) {
                const double toFront = QLineF(tail, chains.at(i).first()).length();
                const double toBack = QLineF(tail, chains.at(i).last()).length();
                if (toFront < bestDistance) {
                    bestDistance = toFront;
                    bestIndex = i;
                    bestReversed = false;
                }
                if (toBack < bestDistance) {
                    bestDistance = toBack;
                    bestIndex = i;
                    bestReversed = true;
                }
            }
            QVector<QPointF> next = chains.takeAt(bestIndex);
            if (bestReversed) {
                std::reverse(next.begin(), next.end());
            }
            loop.append(next);
        }
        const QPolygonF candidate(loop);
        const double area = polygonArea(candidate);
        // Accept only if it encloses a believable share of the hull. A join that
        // went wrong self-intersects, and a self-intersecting loop reports a
        // wrong area and rasterises into stripes, so this check is what keeps a
        // bad outline from silently corrupting the pack.
        if (hullArea > 0.0 && area >= hullArea * 0.55 && area <= hullArea * 1.02) {
            return candidate;
        }
    }
    return hull;
}

namespace {

// ------------------------------------------------------------------ raster

using Word = std::uint64_t;
constexpr int wordBits = 64;

// A part at one rotation, as a bitmask plus the per-column profile the
// placement scan needs.
struct Mask
{
    int width = 0;   // cells
    int height = 0;  // cells
    int stride = 0;  // words per row
    QVector<Word> bits;
    // Lowest and highest filled cell in each column, -1 when the column is
    // empty. bottom drives where the part can rest; top drives what it leaves
    // behind for the next part.
    QVector<int> bottom;
    QVector<int> top;
    int filled = 0;

    bool test(int x, int y) const
    {
        return (bits[y * stride + (x >> 6)] >> (x & 63)) & 1u;
    }
};

// Scanline-fills a polygon into a bitmask, growing it by `growCells` so the
// configured gap is baked into the shape and collision stays a plain AND.
Mask rasterise(const QPolygonF &polygon, double cellMm, int growCells)
{
    Mask mask;
    if (polygon.size() < 3) {
        return mask;
    }
    const Extent extent = extentOf(polygon);

    const int originX =
        static_cast<int>(std::floor(extent.minX / cellMm)) - growCells;
    const int originY =
        static_cast<int>(std::floor(extent.minY / cellMm)) - growCells;
    mask.width =
        static_cast<int>(std::ceil(extent.maxX / cellMm)) + growCells - originX + 1;
    mask.height =
        static_cast<int>(std::ceil(extent.maxY / cellMm)) + growCells - originY + 1;
    if (mask.width <= 0 || mask.height <= 0) {
        return mask;
    }
    mask.stride = (mask.width + wordBits - 1) / wordBits;
    mask.bits.assign(static_cast<qsizetype>(mask.stride) * mask.height, 0);

    // Even-odd scanline fill at cell centres.
    QVector<double> crossings;
    for (int row = 0; row < mask.height; ++row) {
        const double y = (originY + row + 0.5) * cellMm;
        crossings.clear();
        for (int i = 0, n = polygon.size(); i < n; ++i) {
            const QPointF &a = polygon.at(i);
            const QPointF &b = polygon.at((i + 1) % n);
            if ((a.y() <= y) == (b.y() <= y)) {
                continue;
            }
            const double t = (y - a.y()) / (b.y() - a.y());
            crossings.append(a.x() + t * (b.x() - a.x()));
        }
        if (crossings.size() < 2) {
            continue;
        }
        std::sort(crossings.begin(), crossings.end());
        for (int i = 0; i + 1 < crossings.size(); i += 2) {
            int from = static_cast<int>(
                std::floor(crossings[i] / cellMm - 0.5)) - originX;
            int to = static_cast<int>(
                std::ceil(crossings[i + 1] / cellMm - 0.5)) - originX;
            from = std::max(from, 0);
            to = std::min(to, mask.width - 1);
            for (int x = from; x <= to; ++x) {
                mask.bits[static_cast<qsizetype>(row) * mask.stride + (x >> 6)] |=
                    Word(1) << (x & 63);
            }
        }
    }

    // Grow by the gap. A square dilation is close enough at these radii and far
    // cheaper than a true offset; it errs towards separation, never overlap.
    for (int pass = 0; pass < growCells; ++pass) {
        QVector<Word> grown = mask.bits;
        for (int row = 0; row < mask.height; ++row) {
            const qsizetype base = static_cast<qsizetype>(row) * mask.stride;
            for (int word = 0; word < mask.stride; ++word) {
                Word value = mask.bits[base + word];
                Word spread = value | (value << 1) | (value >> 1);
                if (word > 0 && (mask.bits[base + word - 1] >> 63)) {
                    spread |= Word(1);
                }
                if (word + 1 < mask.stride && (mask.bits[base + word + 1] & 1u)) {
                    spread |= Word(1) << 63;
                }
                grown[base + word] |= spread;
                if (row > 0) {
                    grown[base - mask.stride + word] |= value;
                }
                if (row + 1 < mask.height) {
                    grown[base + mask.stride + word] |= value;
                }
            }
        }
        mask.bits = grown;
    }

    mask.bottom.assign(mask.width, -1);
    mask.top.assign(mask.width, -1);
    for (int row = 0; row < mask.height; ++row) {
        for (int x = 0; x < mask.width; ++x) {
            if (mask.test(x, row)) {
                if (mask.bottom[x] < 0) {
                    mask.bottom[x] = row;
                }
                mask.top[x] = row;
                ++mask.filled;
            }
        }
    }
    return mask;
}

QPolygonF transformed(const QPolygonF &polygon, double rotationDeg, double scale)
{
    // Must match PlacementFrame::map exactly, or the preview and the PDF will
    // draw a layout the packer did not produce.
    const double radians = rotationDeg * M_PI / 180.0;
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    QPolygonF result;
    result.reserve(polygon.size());
    for (const QPointF &point : polygon) {
        const double x = point.x() * scale;
        const double y = point.y() * scale;
        result.append(QPointF(x * c - y * s, x * s + y * c));
    }
    // Re-origin so every mask starts at (0, 0).
    const Extent extent = extentOf(result);
    result.translate(-extent.minX, -extent.minY);
    return result;
}

// ---------------------------------------------------------------- canvas

// Growable occupancy bitmap with a per-column skyline. The skyline gives a
// position that is guaranteed collision-free (every part cell then sits above
// everything already filled in its column); a gravity drop from there recovers
// the tuck-ins that a pure skyline packer misses on concave parts.
class Canvas
{
public:
    Canvas(int width, int reserveHeight)
        : width_(width),
          stride_((width + wordBits - 1) / wordBits),
          skyline_(width, 0)
    {
        rows_.assign(static_cast<qsizetype>(stride_) * std::max(reserveHeight, 1), 0);
        capacity_ = std::max(reserveHeight, 1);
    }

    int width() const { return width_; }
    int usedHeight() const { return used_; }

    bool fits(const Mask &mask, int x, int y) const
    {
        if (x < 0 || y < 0 || x + mask.width > width_) {
            return false;
        }
        for (int row = 0; row < mask.height; ++row) {
            const int canvasRow = y + row;
            if (canvasRow >= used_) {
                break; // everything above the used height is empty
            }
            const qsizetype maskBase = static_cast<qsizetype>(row) * mask.stride;
            const qsizetype canvasBase =
                static_cast<qsizetype>(canvasRow) * stride_;
            // Shift the mask row right by x across word boundaries.
            const int wordShift = x >> 6;
            const int bitShift = x & 63;
            for (int word = 0; word < mask.stride; ++word) {
                const Word value = mask.bits[maskBase + word];
                if (value == 0) {
                    continue;
                }
                const int target = wordShift + word;
                if (bitShift == 0) {
                    if (rows_[canvasBase + target] & value) {
                        return false;
                    }
                } else {
                    if (rows_[canvasBase + target] & (value << bitShift)) {
                        return false;
                    }
                    if (target + 1 < stride_
                        && (rows_[canvasBase + target + 1]
                            & (value >> (wordBits - bitShift)))) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    // Lowest y at which the mask provably cannot overlap anything.
    int restingHeight(const Mask &mask, int x) const
    {
        int y = 0;
        for (int column = 0; column < mask.width; ++column) {
            if (mask.bottom[column] < 0) {
                continue;
            }
            y = std::max(y, skyline_[x + column] - mask.bottom[column]);
        }
        return std::max(y, 0);
    }

    void place(const Mask &mask, int x, int y)
    {
        ensure(y + mask.height);
        for (int row = 0; row < mask.height; ++row) {
            const qsizetype maskBase = static_cast<qsizetype>(row) * mask.stride;
            const qsizetype canvasBase =
                static_cast<qsizetype>(y + row) * stride_;
            const int wordShift = x >> 6;
            const int bitShift = x & 63;
            for (int word = 0; word < mask.stride; ++word) {
                const Word value = mask.bits[maskBase + word];
                if (value == 0) {
                    continue;
                }
                const int target = wordShift + word;
                if (bitShift == 0) {
                    rows_[canvasBase + target] |= value;
                } else {
                    rows_[canvasBase + target] |= value << bitShift;
                    if (target + 1 < stride_) {
                        rows_[canvasBase + target + 1] |=
                            value >> (wordBits - bitShift);
                    }
                }
            }
        }
        for (int column = 0; column < mask.width; ++column) {
            if (mask.top[column] < 0) {
                continue;
            }
            skyline_[x + column] =
                std::max(skyline_[x + column], y + mask.top[column] + 1);
        }
        used_ = std::max(used_, y + mask.height);
    }

private:
    void ensure(int height)
    {
        if (height <= capacity_) {
            return;
        }
        capacity_ = std::max(height, capacity_ * 2);
        rows_.resize(static_cast<qsizetype>(stride_) * capacity_, 0);
    }

    int width_ = 0;
    int stride_ = 0;
    int capacity_ = 0;
    int used_ = 0;
    QVector<Word> rows_;
    QVector<int> skyline_;
};

struct Candidate
{
    int pieceIndex = 0;
    // One mask per allowed rotation, in the order rotations are listed.
    QVector<Mask> masks;
    QVector<double> rotations;
    double areaMm2 = 0.0;
    int longest = 0;
};

struct Layout
{
    QVector<Placement> placements;
    QVector<int> unplaced;
    int heightCells = 0;
};

// One bottom-left-fill pass in the given order. Each part is tried at every
// allowed rotation and every x, and takes the position with the lowest resting
// y — ties to the leftmost, which is what makes the result deterministic.
// sheetWidthCells / sheetHeightCells confine each part to a single sheet of the
// grid when non-zero; zero means parts may straddle sheet edges, which is what
// paper wants.
Layout runPass(const QVector<Candidate> &candidates,
               const QVector<int> &order,
               int canvasWidthCells,
               double cellMm,
               int sheetWidthCells = 0,
               int sheetHeightCells = 0)
{
    Layout layout;
    // Reserve something proportional to the work so the first placements do not
    // trigger repeated reallocation.
    double totalArea = 0.0;
    for (const Candidate &candidate : candidates) {
        totalArea += candidate.areaMm2;
    }
    const int reserve = static_cast<int>(
        totalArea / std::max(canvasWidthCells * cellMm, 1.0) / cellMm * 1.6 + 64);
    Canvas canvas(canvasWidthCells, reserve);

    for (const int index : order) {
        const Candidate &candidate = candidates.at(index);
        int bestY = std::numeric_limits<int>::max();
        int bestX = -1;
        double bestRotation = 0.0;
        const Mask *bestMask = nullptr;

        for (int r = 0; r < candidate.masks.size(); ++r) {
            const Mask &mask = candidate.masks.at(r);
            if (mask.width <= 0 || mask.width > canvasWidthCells) {
                continue;
            }
            // A part bigger than one sheet can never satisfy the constraint at
            // this rotation; another rotation may still work.
            if (sheetWidthCells > 0
                && (mask.width > sheetWidthCells
                    || mask.height > sheetHeightCells)) {
                continue;
            }
            const int span = canvasWidthCells - mask.width;

            // Two-phase x scan. Trying every column costs the width of the
            // canvas per part per pass, which the search cannot afford; a
            // coarse sweep followed by a fine sweep around the best column
            // gets full resolution where it matters for an eighth of the work.
            const auto evaluate = [&](int x) {
                int y = canvas.restingHeight(mask, x);
                // restingHeight is collision-free, and so is anything above it,
                // so lifting the part to clear a sheet seam keeps it valid.
                int floorY = 0;
                if (sheetHeightCells > 0) {
                    const int row = y / sheetHeightCells;
                    if (y + mask.height > (row + 1) * sheetHeightCells) {
                        y = (row + 1) * sheetHeightCells;
                    }
                    floorY = (y / sheetHeightCells) * sheetHeightCells;
                }
                while (y > floorY && canvas.fits(mask, x, y - 1)) {
                    --y;
                }
                if (y > bestY || !canvas.fits(mask, x, y)) {
                    return;
                }
                // Strict improvement only, so ties keep the leftmost column and
                // the result stays deterministic.
                if (y < bestY || bestMask == nullptr) {
                    bestY = y;
                    bestX = x;
                    bestRotation = candidate.rotations.at(r);
                    bestMask = &mask;
                }
            };

            // Candidate x windows. Unconstrained it is the whole span; per
            // sheet it is one window inside each sheet column, which is what
            // stops a part being offered a position across a seam at all.
            QVector<QPair<int, int>> windows;
            if (sheetWidthCells > 0) {
                for (int column = 0;
                     column * sheetWidthCells + mask.width <= canvasWidthCells;
                     ++column) {
                    const int lo = column * sheetWidthCells;
                    const int hi = std::min(lo + sheetWidthCells - mask.width,
                                            span);
                    if (hi >= lo) {
                        windows.append({lo, hi});
                    }
                }
            } else {
                windows.append({0, span});
            }

            for (const QPair<int, int> &window : windows) {
                const int reach = window.second - window.first;
                const int coarse = std::max(1, (reach + 1) / 128);
                for (int x = window.first; x <= window.second; x += coarse) {
                    evaluate(x);
                }
                if (coarse > 1 && bestX >= window.first
                    && bestX <= window.second) {
                    const int from = std::max(window.first, bestX - coarse);
                    const int to = std::min(window.second, bestX + coarse);
                    for (int x = from; x <= to; ++x) {
                        evaluate(x);
                    }
                }
            }
        }

        if (bestMask == nullptr) {
            layout.unplaced.append(candidate.pieceIndex);
            continue;
        }
        canvas.place(*bestMask, bestX, bestY);
        layout.placements.append(Placement{candidate.pieceIndex,
                                           bestX * cellMm,
                                           bestY * cellMm,
                                           bestRotation});
    }
    layout.heightCells = canvas.usedHeight();
    return layout;
}

int sheetsFor(double lengthMm, double pageMm, double overlapMm)
{
    if (lengthMm <= 0.0) {
        return 0;
    }
    const double advance = std::max(pageMm - overlapMm, 1.0);
    if (lengthMm <= pageMm) {
        return 1;
    }
    return 1 + static_cast<int>(std::ceil((lengthMm - pageMm) / advance));
}

// Effective sheet advance. Sheets overlap on paper so each repeats a band of
// its neighbour; a machine bed tiles exactly, because nothing is being taped.
double advanceFor(double pageMm, const NestOptions &options)
{
    return options.partsWithinOneSheet
        ? pageMm
        : std::max(pageMm - options.overlapMm, 1.0);
}

} // namespace

PlacementFrame frameFor(const FlatPiece &piece,
                        const Placement &placement,
                        double scale)
{
    PlacementFrame frame;
    frame.rotationDeg = placement.rotationDeg;
    frame.scale = scale;
    // The packer measures a placement from the rotated piece's bounding box, so
    // recover the same shift by rotating the outline and taking its extent.
    PlacementFrame unshifted;
    unshifted.rotationDeg = placement.rotationDeg;
    unshifted.scale = scale;
    QPolygonF rotated;
    for (const QPointF &point : outerBoundary(piece)) {
        rotated.append(unshifted.map(point));
    }
    const Extent extent = extentOf(rotated);
    frame.offset = QPointF(placement.x - extent.minX, placement.y - extent.minY);
    return frame;
}

NestResult nest(const FlatPartSet &set,
                const QVector<int> &pieceIndices,
                const NestOptions &options,
                const NestCallbacks &callbacks)
{
    QElapsedTimer timer;
    timer.start();

    NestResult result;
    if (pieceIndices.isEmpty()) {
        return result;
    }

    const int step = std::clamp(options.rotationStepDeg, 1, 180);
    QVector<double> rotations;
    for (int angle = 0; angle < 360; angle += step) {
        rotations.append(angle);
    }

    // Free rotation multiplies the rasterised masks, and a rotated mask is
    // bounded by the part's diagonal in both axes rather than its own width and
    // height — a 3 m rib at 45° covers four times the cells it does upright.
    // Rather than fail or swap to disk, coarsen the raster until the masks fit
    // the budget and report the resolution actually used.
    double cell = std::max(options.resolutionMm, 0.1);
    {
        double diagonalCellsSquared = 0.0;
        for (const int index : pieceIndices) {
            if (index < 0 || index >= set.pieces.size()) {
                continue;
            }
            const QSizeF size = set.pieces.at(index).size;
            const double diagonal =
                std::hypot(size.width(), size.height()) * options.scale;
            diagonalCellsSquared += diagonal * diagonal;
        }
        const double budgetBytes =
            static_cast<double>(std::max(options.maskBudgetMb, 16)) * 1024.0
            * 1024.0;
        // bits = (diagonal/cell)^2 per angle, so bytes = that / 8.
        const double bytesAtCell =
            diagonalCellsSquared * rotations.size() / (cell * cell) / 8.0;
        if (bytesAtCell > budgetBytes) {
            cell *= std::sqrt(bytesAtCell / budgetBytes);
        }
    }
    result.resolutionMm = cell;

    const int grow = std::max(
        0, static_cast<int>(std::round(options.gapMm / 2.0 / cell)));

    QVector<Candidate> candidates;
    candidates.reserve(pieceIndices.size());
    double partArea = 0.0;
    for (const int index : pieceIndices) {
        if (index < 0 || index >= set.pieces.size()) {
            continue;
        }
        const FlatPiece &piece = set.pieces.at(index);
        const QPolygonF boundary = outerBoundary(piece);
        Candidate candidate;
        candidate.pieceIndex = index;
        for (const double rotation : rotations) {
            Mask mask =
                rasterise(transformed(boundary, rotation, options.scale), cell,
                          grow);
            if (mask.width > 0 && mask.height > 0) {
                candidate.masks.append(std::move(mask));
                candidate.rotations.append(rotation);
            }
        }
        if (candidate.masks.isEmpty()) {
            result.unplaced.append(index);
            continue;
        }
        candidate.areaMm2 =
            polygonArea(boundary) * options.scale * options.scale;
        candidate.longest = std::max(candidate.masks.first().width,
                                     candidate.masks.first().height);
        partArea += candidate.areaMm2;
        candidates.append(std::move(candidate));
    }
    if (candidates.isEmpty()) {
        return result;
    }

    // Seed orders. Longest-side-first and largest-area-first are the two
    // classic decreasing heuristics and between them usually bracket the good
    // solutions; the search perturbs from whichever wins.
    QVector<int> byArea(candidates.size());
    for (int i = 0; i < candidates.size(); ++i) {
        byArea[i] = i;
    }
    QVector<int> byLength = byArea;
    std::sort(byArea.begin(), byArea.end(), [&](int a, int b) {
        return candidates[a].areaMm2 > candidates[b].areaMm2;
    });
    std::sort(byLength.begin(), byLength.end(), [&](int a, int b) {
        return candidates[a].longest > candidates[b].longest;
    });

    const double advanceX = advanceFor(options.pageWidthMm, options);
    // Sheet cells in raster cells; zero when parts may straddle sheet edges.
    const int sheetWidthCells = options.partsWithinOneSheet
        ? static_cast<int>(std::floor(options.pageWidthMm / cell))
        : 0;
    const int sheetHeightCells = options.partsWithinOneSheet
        ? static_cast<int>(std::floor(options.pageHeightMm / cell))
        : 0;
    int iterations = 0;

    bool cancelled = false;
    // Checked every iteration so a stop is honoured promptly even while the
    // search is plateauing and nothing is being reported.
    const auto keepGoing = [&] {
        if (cancelled) {
            return false;
        }
        if (callbacks.stopRequested && callbacks.stopRequested()) {
            cancelled = true;
            return false;
        }
        if (options.timeBudgetMs > 0 && timer.elapsed() > options.timeBudgetMs) {
            return false;
        }
        return true;
    };

    qint64 lastReportMs = 0;
    const auto report = [&](const Layout &layout,
                            double widthMm,
                            double heightMm,
                            int sheets,
                            int pages,
                            double partAreaMm2) {
        if (!callbacks.improved || cancelled) {
            return;
        }
        lastReportMs = timer.elapsed();
        NestResult snapshot;
        snapshot.placements = layout.placements;
        snapshot.unplaced = layout.unplaced;
        snapshot.canvasWidthMm = widthMm;
        snapshot.canvasHeightMm = heightMm;
        snapshot.sheetsAcross = sheets;
        snapshot.sheetsDown =
            sheetsFor(heightMm, options.pageHeightMm, options.partsWithinOneSheet ? 0.0 : options.overlapMm);
        snapshot.pageCount = pages;
        snapshot.utilisation = widthMm * heightMm > 0.0
            ? partAreaMm2 / (widthMm * heightMm)
            : 0.0;
        snapshot.iterations = iterations;
        snapshot.elapsedMs = timer.elapsed();
        callbacks.improved(snapshot);
    };

    int bestPages = std::numeric_limits<int>::max();
    double bestHeight = std::numeric_limits<double>::max();
    Layout bestLayout;
    double bestCanvasWidth = 0.0;
    int bestWidthCells = 0;
    int bestSheets = 0;

    // One search state per candidate canvas width. Narrower canvases waste less
    // on the last column but force long parts upright; wider ones do the
    // reverse, and which wins is not predictable from the part mix. Each width
    // keeps its own incumbent order so the refinement below can revisit any of
    // them without losing the progress it made there.
    struct Width
    {
        int sheets = 0;
        int cells = 0;
        double widthMm = 0.0;
        QVector<int> incumbent;
        double incumbentHeight = std::numeric_limits<double>::max();
        int sinceImprovement = 0;
        // Best page count reached at this width, used to concentrate the search
        // on widths that are still in contention.
        int bestPagesHere = std::numeric_limits<int>::max();
    };
    QVector<Width> widths;

    for (int sheets = 1; sheets <= options.maxSheetsAcross; ++sheets) {
        const double canvasWidthMm =
            options.pageWidthMm + advanceX * (sheets - 1);
        const int widthCells = static_cast<int>(std::floor(canvasWidthMm / cell));
        if (widthCells <= 0) {
            continue;
        }
        // Skip widths that cannot hold the bulkiest part in any orientation.
        bool feasible = true;
        for (const Candidate &candidate : candidates) {
            bool anyFits = false;
            for (const Mask &mask : candidate.masks) {
                if (mask.width <= widthCells) {
                    anyFits = true;
                    break;
                }
            }
            if (!anyFits) {
                feasible = false;
                break;
            }
        }
        if (feasible) {
            widths.append(Width{sheets, widthCells, canvasWidthMm, {},
                                std::numeric_limits<double>::max(), 0});
        }
    }

    const auto consider = [&](const Layout &layout, Width &width) {
        const double heightMm = layout.heightCells * cell;
        const int pages = width.sheets
            * sheetsFor(heightMm, options.pageHeightMm, options.partsWithinOneSheet ? 0.0 : options.overlapMm);
        width.bestPagesHere = std::min(width.bestPagesHere, pages);
        if (heightMm < width.incumbentHeight) {
            width.incumbentHeight = heightMm;
            width.sinceImprovement = 0;
        } else {
            ++width.sinceImprovement;
        }
        if (pages < bestPages || (pages == bestPages && heightMm < bestHeight)) {
            bestPages = pages;
            bestHeight = heightMm;
            bestLayout = layout;
            bestCanvasWidth = width.widthMm;
            bestWidthCells = width.cells;
            bestSheets = width.sheets;
            report(bestLayout, bestCanvasWidth, bestHeight, bestSheets,
                   bestPages, partArea);
            return true;
        }
        return false;
    };

    // Opening sweep: both decreasing heuristics at every width. This settles the
    // canvas width, which moves the page count far more than any ordering does.
    for (Width &width : widths) {
        for (const QVector<int> *order : {&byArea, &byLength}) {
            if (!keepGoing()) {
                break;
            }
            ++iterations;
            Layout layout = runPass(candidates, *order, width.cells, cell,
                                    sheetWidthCells, sheetHeightCells);
            if (layout.heightCells * cell < width.incumbentHeight) {
                width.incumbent = *order;
            }
            consider(layout, width);
        }
        if (!keepGoing()) {
            break;
        }
    }

    // Then refine indefinitely, round-robin across the widths. Cycling rather
    // than committing to the opening winner matters on a long run: a width that
    // opened second-best often overtakes once its ordering is tuned, and with no
    // time limit there is no reason not to find that out.
    //
    // Perturbation escalates with stagnation and restarts from a fresh shuffle
    // after a long plateau — a plain hill-climb converges within seconds and
    // then burns the rest of the run rediscovering the same local minimum.
    if (!widths.isEmpty()) {
        // The width that won the opening sweep gets the first few sweeps to
        // itself. Committing early is what a short run needs; opening up after
        // is what a long one needs, and the two are not in conflict as long as
        // the order is right.
        int leader = 0;
        for (int i = 0; i < widths.size(); ++i) {
            if (widths[i].bestPagesHere < widths[leader].bestPagesHere) {
                leader = i;
            }
        }

        std::mt19937 rng(12345u);
        int cursor = 0;
        while (keepGoing()) {
            const int slot = cursor % widths.size();
            const int sweep = cursor / widths.size();
            ++cursor;
            if (sweep < 4 && slot != leader) {
                continue;
            }
            // Concentrate on widths still within reach of the best. A canvas
            // needing 15% more pages will not be rescued by reordering, and
            // spending a tenth of the run on each of them is what made the
            // round-robin worse than committing to one width on a short run.
            // Every twenty-fifth sweep visits everything anyway, so a width is
            // never written off for good.
            if (sweep % 25 != 0
                && widths[slot].bestPagesHere
                    > static_cast<int>(bestPages * 1.1)) {
                continue;
            }
            Width &width = widths[slot];
            if (width.incumbent.isEmpty()) {
                width.incumbent = byArea;
            }

            QVector<int> trial = width.incumbent;
            if (width.sinceImprovement > 400) {
                std::shuffle(trial.begin(), trial.end(), rng);
                width.sinceImprovement = 0;
                width.incumbentHeight = std::numeric_limits<double>::max();
            } else {
                const int reach = width.sinceImprovement > 60 ? 10 : 4;
                const int swaps = 1 + static_cast<int>(rng() % reach);
                for (int s = 0; s < swaps && trial.size() > 1; ++s) {
                    const int a = static_cast<int>(rng() % trial.size());
                    const int b = static_cast<int>(rng() % trial.size());
                    std::swap(trial[a], trial[b]);
                }
            }

            ++iterations;
            Layout layout = runPass(candidates, trial, width.cells, cell,
                                    sheetWidthCells, sheetHeightCells);
            if (layout.heightCells * cell < width.incumbentHeight) {
                width.incumbent = trial;
            }
            consider(layout, width);

            // Re-report the unchanged best during a plateau. Without this the
            // counters freeze between improvements, and a search that is
            // working hard looks indistinguishable from one that has hung —
            // which matters a great deal now that a run has no end of its own.
            if (timer.elapsed() - lastReportMs > 500) {
                report(bestLayout, bestCanvasWidth, bestHeight, bestSheets,
                       bestPages, partArea);
            }
        }
    }

    if (bestLayout.placements.isEmpty()) {
        result.unplaced = pieceIndices;
        return result;
    }

    result.placements = bestLayout.placements;
    result.unplaced += bestLayout.unplaced;
    result.canvasWidthMm = bestCanvasWidth;
    result.canvasHeightMm = bestHeight;
    // Set after all searching: the refinement can win at a different width
    // than the opening sweep did, and reporting the opening width alongside the
    // final height gives a sheet grid that does not multiply out to the page
    // count.
    result.sheetsAcross = bestSheets;
    result.sheetsDown =
        sheetsFor(bestHeight, options.pageHeightMm, options.partsWithinOneSheet ? 0.0 : options.overlapMm);
    result.pageCount = bestPages;
    result.utilisation = bestCanvasWidth * bestHeight > 0.0
        ? partArea / (bestCanvasWidth * bestHeight)
        : 0.0;
    result.iterations = iterations;
    result.elapsedMs = timer.elapsed();
    return result;
}

} // namespace flatparts
