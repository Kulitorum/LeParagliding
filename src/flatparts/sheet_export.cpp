#include "sheet_export.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QPageLayout>
#include <QPageSize>
#include <QPainter>
#include <QPainterPath>
#include <QPdfWriter>
#include <QTextStream>

#include <algorithm>
#include <cmath>

namespace flatparts {
namespace {

// Registration grid pitch, in canvas millimetres. Crosses land on the same
// canvas coordinates on every sheet that covers them, so two sheets sharing an
// overlap band show the same crosses and are aligned by superimposing them.
constexpr double registrationPitchMm = 100.0;

// Line weights, in millimetres of ink.
constexpr double cutLineMm = 0.35;
constexpr double seamLineMm = 0.22;
constexpr double markLineMm = 0.18;
constexpr double furnitureLineMm = 0.20;

// A margin band narrower than this cannot hold legible text far enough from the
// paper edge for a desktop printer to reach it, so the header moves inside the
// printable area instead.
constexpr double minimumHeaderBandMm = 6.0;
constexpr double minimumRulerBandMm = 8.0;

constexpr double headerTextMm = 2.6;
constexpr double markerTextMm = 2.0;

// Smallest label that survives being printed. The plan's own text heights were
// chosen for its drawing scale, not for paper.
constexpr double minimumLabelMm = 2.2;

double advanceFor(double pageMm, const NestOptions &options)
{
    // Must match the nester: sheets overlap on paper, and a bed tiles exactly
    // because nothing is being taped.
    return options.partsWithinOneSheet
        ? std::max(pageMm, 1.0)
        : std::max(pageMm - options.overlapMm, 1.0);
}

// Rectangle overlap, inclusive of touching edges and tolerant of degenerate
// rectangles. QRectF::intersects() reports false for anything with zero width
// or height, which would silently drop a part that happens to be a straight
// line — a strap, say — from every sheet.
bool overlaps(const QRectF &a, const QRectF &b)
{
    return a.left() <= b.right() && b.left() <= a.right()
        && a.top() <= b.bottom() && b.top() <= a.bottom();
}

// Maps canvas millimetres onto one sheet's page, in painter device units.
//
// The canvas is y-up with its origin at the bottom-left; a page is y-down from
// its top-left corner. Everything is mapped through here rather than by a
// painter transform, because a flipping transform also mirrors text and the
// labels have to stay readable.
struct SheetFrame
{
    double k = 1.0; // millimetres to device units
    double marginX = 0.0;
    double marginY = 0.0;
    double pageWidthMm = 0.0;
    double pageHeightMm = 0.0;
    QPointF originMm;

    QPointF map(const QPointF &canvasMm) const
    {
        return QPointF((marginX + canvasMm.x() - originMm.x()) * k,
                       (marginY + pageHeightMm - (canvasMm.y() - originMm.y()))
                           * k);
    }

    // A point given in page millimetres, measured from the paper's top-left.
    QPointF paper(double xMm, double yMm) const
    {
        return QPointF(xMm * k, yMm * k);
    }

    QRectF printableRect() const
    {
        return QRectF(paper(marginX, marginY),
                      QSizeF(pageWidthMm * k, pageHeightMm * k));
    }
};

QPen strokePen(const QColor &colour,
               double widthMm,
               double k,
               Qt::PenStyle style)
{
    QPen pen(colour);
    // Never below one device unit: a hairline that rounds to zero disappears
    // from some PDF viewers entirely.
    pen.setWidthF(std::max(widthMm * k, 1.0));
    pen.setStyle(style);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    return pen;
}

QFont sheetFont(double heightMm, double k, bool bold = false)
{
    QFont font(QStringLiteral("Helvetica"));
    font.setPixelSize(std::max(1, static_cast<int>(std::lround(heightMm * k))));
    font.setBold(bold);
    return font;
}

// Colour per role. These prints are often run on a monochrome laser, so the
// three roles are also separated by weight and dash pattern rather than colour
// alone: near-black solid to cut, mid-grey dashed to stitch, light for marks.
QColor roleColour(Role role)
{
    switch (role) {
    case Role::Cut:
        return QColor(0, 0, 0);
    case Role::Seam:
        return QColor(105, 105, 115);
    case Role::Mark:
        break;
    }
    return QColor(140, 140, 148);
}

// Where each label actually goes on a nested sheet.
//
// The engine's anchors are positions in the plan's drawing box, not on the
// part: in the shipped presets every rib's number sits 635 mm to the LEFT of
// the rib it names, in the plan's margin. That is meaningless once the parts
// are nested — the number would be printed over whichever unrelated part
// happens to be packed there, which is worse than no number at all. So an
// anchor that does not fall on its own part is replaced by the part's centre,
// and only an anchor already on the part is taken at face value.
QVector<Label> placedLabels(const FlatPiece &piece)
{
    QVector<Label> labels;
    const QRectF box = piece.bounds();
    int stacked = 0;
    for (const Label &label : piece.labels) {
        if (label.text.isEmpty()) {
            continue;
        }
        Label placed = label;
        if (!box.contains(label.anchor)) {
            // Piece coordinates here; the export scale is applied later. The
            // anchor is clamped onto the part, because a label wider than the
            // part it names would otherwise be centred so far out that it
            // starts beyond the packed canvas and is clipped away entirely.
            const double height = std::max(label.height, 1.0);
            const double width = height * 0.6 * label.text.size();
            placed.anchor = QPointF(
                std::clamp(box.center().x() - width * 0.5, box.left(),
                           box.right()),
                std::clamp(box.center().y() - height * 0.5
                               - stacked * height * 1.4,
                           box.top(), box.bottom()));
            ++stacked;
        }
        labels.append(placed);
    }
    return labels;
}

void drawPolyline(QPainter &painter,
                  const SheetFrame &sheetFrame,
                  const PlacementFrame &frame,
                  const Polyline &polyline)
{
    QPainterPath path;
    path.moveTo(sheetFrame.map(frame.map(polyline.points.first())));
    for (int index = 1; index < polyline.points.size(); ++index) {
        path.lineTo(sheetFrame.map(frame.map(polyline.points.at(index))));
    }
    if (polyline.closed) {
        path.closeSubpath();
    }

    const double widthMm = polyline.role == Role::Cut ? cutLineMm
        : polyline.role == Role::Seam                 ? seamLineMm
                                                      : markLineMm;
    QPen pen = strokePen(roleColour(polyline.role), widthMm, sheetFrame.k,
                         Qt::SolidLine);
    if (polyline.role == Role::Seam) {
        // Dashes in units of pen width, so the pattern comes out the same
        // length on paper whatever resolution the writer runs at.
        pen.setDashPattern({12.0, 8.0});
    }
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
}

void drawPiece(QPainter &painter,
               const SheetFrame &sheetFrame,
               const FlatPiece &piece,
               const Placement &placement,
               double scale,
               const ExportOptions &exportOptions)
{
    const PlacementFrame frame = frameFor(piece, placement, scale);

    for (const Polyline &polyline : piece.polylines) {
        if (polyline.role == Role::Seam && !exportOptions.drawSeamLines) {
            continue;
        }
        if (polyline.role == Role::Mark && !exportOptions.drawMarks) {
            continue;
        }
        drawPolyline(painter, sheetFrame, frame, polyline);
    }

    if (exportOptions.drawMarks) {
        painter.setPen(strokePen(roleColour(Role::Mark), markLineMm,
                                 sheetFrame.k, Qt::SolidLine));
        painter.setBrush(Qt::NoBrush);
        for (const Circle &circle : piece.circles) {
            const QPointF centre = sheetFrame.map(frame.map(circle.centre));
            const double radius = circle.radius * scale * sheetFrame.k;
            painter.drawEllipse(centre, radius, radius);
        }
    }

    if (!exportOptions.drawLabels) {
        return;
    }
    for (const Label &label : placedLabels(piece)) {
        const double heightMm = std::max(label.height * scale, minimumLabelMm);
        painter.save();
        painter.translate(sheetFrame.map(frame.map(label.anchor)));
        // The page's y axis runs the other way, which negates rotations; with
        // this the label reads along the part exactly as it does on the plan.
        painter.rotate(-frame.rotationDeg);
        painter.setPen(QPen(roleColour(Role::Cut)));
        painter.setFont(sheetFont(heightMm, sheetFrame.k));
        painter.drawText(QPointF(0.0, 0.0), label.text);
        painter.restore();
    }
}

void drawRegistrationGrid(QPainter &painter, const SheetFrame &sheetFrame)
{
    const double x0 = sheetFrame.originMm.x();
    const double y0 = sheetFrame.originMm.y();
    const double x1 = x0 + sheetFrame.pageWidthMm;
    const double y1 = y0 + sheetFrame.pageHeightMm;

    painter.setPen(strokePen(QColor(150, 150, 158), furnitureLineMm,
                             sheetFrame.k, Qt::SolidLine));
    painter.setFont(sheetFont(markerTextMm, sheetFrame.k));

    const double arm = 3.0 * sheetFrame.k;
    const long long firstX =
        static_cast<long long>(std::ceil(x0 / registrationPitchMm));
    const long long lastX =
        static_cast<long long>(std::floor(x1 / registrationPitchMm));
    const long long firstY =
        static_cast<long long>(std::ceil(y0 / registrationPitchMm));
    const long long lastY =
        static_cast<long long>(std::floor(y1 / registrationPitchMm));

    for (long long ix = firstX; ix <= lastX; ++ix) {
        for (long long iy = firstY; iy <= lastY; ++iy) {
            const double x = static_cast<double>(ix) * registrationPitchMm;
            const double y = static_cast<double>(iy) * registrationPitchMm;
            const QPointF centre = sheetFrame.map(QPointF(x, y));
            painter.drawLine(centre + QPointF(-arm, 0.0),
                             centre + QPointF(arm, 0.0));
            painter.drawLine(centre + QPointF(0.0, -arm),
                             centre + QPointF(0.0, arm));
            // The coordinate makes the cross self-identifying, so a sheet found
            // face-down on the floor can still be placed.
            painter.drawText(
                centre + QPointF(arm * 0.6, arm * 1.9),
                QStringLiteral("%1,%2").arg(x, 0, 'f', 0).arg(y, 0, 'f', 0));
        }
    }
}

// Border, overlap guides and the neighbour hints: everything that says where
// this sheet's paper ends and the next one begins.
void drawSheetEdges(QPainter &painter,
                    const SheetFrame &sheetFrame,
                    const Sheet &sheet,
                    int sheetsAcross,
                    int sheetsDown,
                    double overlapMm)
{
    painter.setBrush(Qt::NoBrush);
    painter.setPen(strokePen(QColor(120, 120, 128), furnitureLineMm,
                             sheetFrame.k, Qt::SolidLine));
    painter.drawRect(sheetFrame.printableRect());

    if (overlapMm <= 0.0) {
        return;
    }

    // A sheet's left band repeats the previous column and its top band the row
    // above. Drawing those bands says which strip is duplicated, so it is clear
    // which sheet is laid over which.
    painter.setPen(strokePen(QColor(170, 170, 178), furnitureLineMm,
                             sheetFrame.k, Qt::DashLine));
    if (sheet.column > 0) {
        const double x = sheetFrame.marginX + overlapMm;
        painter.drawLine(sheetFrame.paper(x, sheetFrame.marginY),
                         sheetFrame.paper(x, sheetFrame.marginY
                                                 + sheetFrame.pageHeightMm));
    }
    if (sheet.row > 0) {
        const double y = sheetFrame.marginY + overlapMm;
        painter.drawLine(sheetFrame.paper(sheetFrame.marginX, y),
                         sheetFrame.paper(sheetFrame.marginX
                                              + sheetFrame.pageWidthMm,
                                          y));
    }

    painter.setPen(QPen(QColor(140, 140, 148)));
    painter.setFont(sheetFont(markerTextMm, sheetFrame.k));
    if (sheet.column + 1 < sheetsAcross) {
        const double x =
            sheetFrame.marginX + sheetFrame.pageWidthMm - overlapMm;
        painter.drawLine(
            sheetFrame.paper(x, sheetFrame.marginY),
            sheetFrame.paper(x, sheetFrame.marginY + sheetFrame.pageHeightMm));
        painter.drawText(
            sheetFrame.paper(x + 1.0,
                             sheetFrame.marginY + sheetFrame.pageHeightMm
                                 - 1.5),
            QStringLiteral("sheet %1 overlaps here").arg(sheet.column + 2));
    }
    if (sheet.row + 1 < sheetsDown) {
        const double y =
            sheetFrame.marginY + sheetFrame.pageHeightMm - overlapMm;
        painter.drawLine(
            sheetFrame.paper(sheetFrame.marginX, y),
            sheetFrame.paper(sheetFrame.marginX + sheetFrame.pageWidthMm, y));
        painter.drawText(
            sheetFrame.paper(sheetFrame.marginX + 1.0, y - 1.0),
            QStringLiteral("row %1 overlaps here").arg(sheet.row + 2));
    }
}

// The 1:1 check. Printer drivers scale to fit by default and a pattern printed
// at 97% looks entirely plausible until the wing is sewn, so every sheet
// carries a bar the user can lay a ruler against.
void drawCalibrationRuler(QPainter &painter,
                          const SheetFrame &sheetFrame,
                          double bandMm)
{
    const double lengthMm = sheetFrame.pageWidthMm >= 140.0 ? 100.0 : 50.0;
    const double baseline =
        sheetFrame.marginY + sheetFrame.pageHeightMm + bandMm * 0.55;
    const double left = sheetFrame.marginX;

    painter.setPen(strokePen(QColor(90, 90, 98), furnitureLineMm, sheetFrame.k,
                             Qt::SolidLine));
    painter.drawLine(sheetFrame.paper(left, baseline),
                     sheetFrame.paper(left + lengthMm, baseline));
    for (double tick = 0.0; tick <= lengthMm + 0.001; tick += 10.0) {
        const double height = std::fmod(tick, 50.0) < 0.001 ? 2.4 : 1.4;
        painter.drawLine(sheetFrame.paper(left + tick, baseline),
                         sheetFrame.paper(left + tick, baseline - height));
    }
    painter.setPen(QPen(QColor(90, 90, 98)));
    painter.setFont(sheetFont(markerTextMm, sheetFrame.k));
    painter.drawText(sheetFrame.paper(left + lengthMm + 3.0, baseline),
                     QStringLiteral("%1 mm — if this measures short, the "
                                    "printer scaled the page")
                         .arg(lengthMm, 0, 'f', 0));
}

QString sheetHeaderText(const Sheet &sheet,
                        int page,
                        int pages,
                        const NestResult &result,
                        const NestOptions &options,
                        const ExportOptions &exportOptions)
{
    QStringList fields;
    if (!exportOptions.title.isEmpty()) {
        fields << exportOptions.title;
    }
    fields << QStringLiteral("sheet %1/%2  (col %3 of %4, row %5 of %6)")
                  .arg(page)
                  .arg(pages)
                  .arg(sheet.column + 1)
                  .arg(result.sheetsAcross)
                  .arg(sheet.row + 1)
                  .arg(result.sheetsDown);
    fields << (qFuzzyCompare(options.scale, 1.0)
                   ? QStringLiteral("1:1 — print at 100%, not fit-to-page")
                   : QStringLiteral("scaled to %1% — print at 100%, not "
                                    "fit-to-page")
                         .arg(options.scale * 100.0, 0, 'f', 1));
    if (!result.unplaced.isEmpty()) {
        // Loud, because a short part list is the kind of thing discovered after
        // the fabric has been cut.
        fields << QStringLiteral("%1 part(s) did not fit and are NOT in this "
                                 "document")
                      .arg(result.unplaced.size());
    }
    if (!exportOptions.subtitle.isEmpty()) {
        fields << exportOptions.subtitle;
    }
    if (!exportOptions.stamp.isEmpty()) {
        fields << exportOptions.stamp;
    }
    return fields.join(QStringLiteral("  ·  "));
}

// --- DXF ---------------------------------------------------------------

struct DxfLayer
{
    const char *name;
    int colour; // AutoCAD colour index
};

const QVector<DxfLayer> &dxfLayers()
{
    static const QVector<DxfLayer> layers{
        {"CUT", 1},   // red — the line the blade follows
        {"SEAM", 5},  // blue — stitch line
        {"MARK", 8},  // dark grey — vents, rod positions, ticks
        {"TEXT", 3},  // green — part numbering
        {"SHEET", 7}, // bed outline, reference only
    };
    return layers;
}

const char *layerFor(Role role)
{
    switch (role) {
    case Role::Cut:
        return "CUT";
    case Role::Seam:
        return "SEAM";
    case Role::Mark:
        break;
    }
    return "MARK";
}

void dxfPair(QTextStream &out, int code, const QString &value)
{
    out << code << '\n' << value << '\n';
}

void dxfPair(QTextStream &out, int code, int value)
{
    out << code << '\n' << value << '\n';
}

void dxfPair(QTextStream &out, int code, double value)
{
    out << code << '\n' << QString::number(value, 'f', 4) << '\n';
}

// DXF R12 has no escaping and readers differ on what they tolerate, so anything
// outside printable ASCII is dropped rather than guessed at.
QString dxfText(const QString &text)
{
    QString clean;
    clean.reserve(text.size());
    for (const QChar &character : text) {
        const ushort code = character.unicode();
        if (code >= 32 && code < 127) {
            clean.append(character);
        } else if (character.isSpace()) {
            clean.append(QLatin1Char(' '));
        }
    }
    return clean.trimmed();
}

void writeDxfHeader(QTextStream &out, const QRectF &extents)
{
    dxfPair(out, 0, QStringLiteral("SECTION"));
    dxfPair(out, 2, QStringLiteral("HEADER"));
    dxfPair(out, 9, QStringLiteral("$ACADVER"));
    dxfPair(out, 1, QStringLiteral("AC1009"));
    dxfPair(out, 9, QStringLiteral("$INSUNITS"));
    dxfPair(out, 70, 4); // millimetres
    dxfPair(out, 9, QStringLiteral("$EXTMIN"));
    dxfPair(out, 10, extents.left());
    dxfPair(out, 20, extents.top());
    dxfPair(out, 30, 0.0);
    dxfPair(out, 9, QStringLiteral("$EXTMAX"));
    dxfPair(out, 10, extents.right());
    dxfPair(out, 20, extents.bottom());
    dxfPair(out, 30, 0.0);
    dxfPair(out, 0, QStringLiteral("ENDSEC"));

    dxfPair(out, 0, QStringLiteral("SECTION"));
    dxfPair(out, 2, QStringLiteral("TABLES"));

    // A LTYPE table holding CONTINUOUS, because the layers below name it and a
    // strict reader rejects a layer whose line type it cannot resolve.
    dxfPair(out, 0, QStringLiteral("TABLE"));
    dxfPair(out, 2, QStringLiteral("LTYPE"));
    dxfPair(out, 70, 1);
    dxfPair(out, 0, QStringLiteral("LTYPE"));
    dxfPair(out, 2, QStringLiteral("CONTINUOUS"));
    dxfPair(out, 70, 64);
    dxfPair(out, 3, QStringLiteral("Solid line"));
    dxfPair(out, 72, 65);
    dxfPair(out, 73, 0);
    dxfPair(out, 40, 0.0);
    dxfPair(out, 0, QStringLiteral("ENDTAB"));

    dxfPair(out, 0, QStringLiteral("TABLE"));
    dxfPair(out, 2, QStringLiteral("LAYER"));
    dxfPair(out, 70, static_cast<int>(dxfLayers().size()));
    for (const DxfLayer &layer : dxfLayers()) {
        dxfPair(out, 0, QStringLiteral("LAYER"));
        dxfPair(out, 2, QString::fromLatin1(layer.name));
        dxfPair(out, 70, 0);
        dxfPair(out, 62, layer.colour);
        dxfPair(out, 6, QStringLiteral("CONTINUOUS"));
    }
    dxfPair(out, 0, QStringLiteral("ENDTAB"));
    dxfPair(out, 0, QStringLiteral("ENDSEC"));

    dxfPair(out, 0, QStringLiteral("SECTION"));
    dxfPair(out, 2, QStringLiteral("ENTITIES"));
}

void writeDxfFooter(QTextStream &out)
{
    dxfPair(out, 0, QStringLiteral("ENDSEC"));
    dxfPair(out, 0, QStringLiteral("EOF"));
}

void writeDxfPolyline(QTextStream &out,
                      const QString &layer,
                      const QVector<QPointF> &points,
                      bool closed)
{
    dxfPair(out, 0, QStringLiteral("POLYLINE"));
    dxfPair(out, 8, layer);
    dxfPair(out, 66, 1);
    dxfPair(out, 70, closed ? 1 : 0);
    dxfPair(out, 10, 0.0);
    dxfPair(out, 20, 0.0);
    dxfPair(out, 30, 0.0);
    for (const QPointF &point : points) {
        dxfPair(out, 0, QStringLiteral("VERTEX"));
        dxfPair(out, 8, layer);
        dxfPair(out, 10, point.x());
        dxfPair(out, 20, point.y());
        dxfPair(out, 30, 0.0);
    }
    dxfPair(out, 0, QStringLiteral("SEQEND"));
    dxfPair(out, 8, layer);
}

void writeDxfCircle(QTextStream &out,
                    const QString &layer,
                    const QPointF &centre,
                    double radius)
{
    dxfPair(out, 0, QStringLiteral("CIRCLE"));
    dxfPair(out, 8, layer);
    dxfPair(out, 10, centre.x());
    dxfPair(out, 20, centre.y());
    dxfPair(out, 30, 0.0);
    dxfPair(out, 40, radius);
}

void writeDxfText(QTextStream &out,
                  const QString &layer,
                  const QPointF &anchor,
                  double height,
                  double rotationDeg,
                  const QString &text)
{
    dxfPair(out, 0, QStringLiteral("TEXT"));
    dxfPair(out, 8, layer);
    dxfPair(out, 10, anchor.x());
    dxfPair(out, 20, anchor.y());
    dxfPair(out, 30, 0.0);
    dxfPair(out, 40, height);
    dxfPair(out, 1, text);
    dxfPair(out, 50, rotationDeg);
}

void writeDxfPlacement(QTextStream &out,
                       const FlatPiece &piece,
                       const Placement &placement,
                       double scale,
                       const QPointF &shift,
                       const ExportOptions &exportOptions)
{
    const PlacementFrame frame = frameFor(piece, placement, scale);

    for (const Polyline &polyline : piece.polylines) {
        if (polyline.role == Role::Seam && !exportOptions.drawSeamLines) {
            continue;
        }
        if (polyline.role == Role::Mark && !exportOptions.drawMarks) {
            continue;
        }
        QVector<QPointF> points;
        points.reserve(polyline.points.size());
        for (const QPointF &point : polyline.points) {
            points.append(frame.map(point) - shift);
        }
        writeDxfPolyline(out, QString::fromLatin1(layerFor(polyline.role)),
                         points, polyline.closed);
    }

    if (exportOptions.drawMarks) {
        for (const Circle &circle : piece.circles) {
            writeDxfCircle(out, QStringLiteral("MARK"),
                           frame.map(circle.centre) - shift,
                           circle.radius * scale);
        }
    }

    if (!exportOptions.drawLabels) {
        return;
    }
    for (const Label &label : placedLabels(piece)) {
        const QString text = dxfText(label.text);
        if (text.isEmpty()) {
            continue;
        }
        // DXF rotation is counter-clockwise in a y-up world, which is the
        // canvas's own convention, so the packer's angle carries straight over.
        writeDxfText(out, QStringLiteral("TEXT"),
                     frame.map(label.anchor) - shift,
                     std::max(label.height * scale, minimumLabelMm),
                     frame.rotationDeg, text);
    }
}

} // namespace

namespace {

QRectF boundsOf(const FlatPartSet &set,
                const Placement &placement,
                double scale,
                bool includeLabels)
{
    if (placement.pieceIndex < 0 || placement.pieceIndex >= set.pieces.size()) {
        return QRectF();
    }
    const FlatPiece &piece = set.pieces.at(placement.pieceIndex);
    const PlacementFrame frame = frameFor(piece, placement, scale);

    bool any = false;
    double minX = 0.0;
    double minY = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    const auto include = [&](const QPointF &point) {
        if (!any) {
            any = true;
            minX = maxX = point.x();
            minY = maxY = point.y();
            return;
        }
        minX = std::min(minX, point.x());
        maxX = std::max(maxX, point.x());
        minY = std::min(minY, point.y());
        maxY = std::max(maxY, point.y());
    };

    for (const Polyline &polyline : piece.polylines) {
        for (const QPointF &point : polyline.points) {
            include(frame.map(point));
        }
    }
    for (const Circle &circle : piece.circles) {
        const QPointF centre = frame.map(circle.centre);
        const double radius = circle.radius * scale;
        include(centre + QPointF(-radius, -radius));
        include(centre + QPointF(radius, radius));
    }
    if (includeLabels) {
        for (const Label &label : placedLabels(piece)) {
            // The text extent is unknown without a font, so this errs
            // generously: a label clipped off the sheet it belongs on is a real
            // loss, a sheet assignment one label too many costs nothing.
            //
            // The estimate is built in the piece's own coordinates and then
            // mapped, not added to the mapped anchor: text runs along the part,
            // so on a quarter-turned piece a box added along the canvas axes
            // would be wrong by the whole length of the label.
            const double height = std::max(label.height * scale, minimumLabelMm)
                / std::max(scale, 1e-9);
            const double width = height * 0.75 * label.text.size();
            include(frame.map(label.anchor + QPointF(-height, -height)));
            include(frame.map(label.anchor + QPointF(width, -height)));
            include(frame.map(label.anchor + QPointF(width, height)));
            include(frame.map(label.anchor + QPointF(-height, height)));
        }
    }
    if (!any) {
        return QRectF();
    }
    return QRectF(minX, minY, maxX - minX, maxY - minY);
}

} // namespace

QRectF placementBounds(const FlatPartSet &set,
                       const Placement &placement,
                       double scale)
{
    return boundsOf(set, placement, scale, true);
}

QVector<int> clippedPlacements(const FlatPartSet &set,
                               const NestResult &result,
                               const NestOptions &options)
{
    QVector<int> clipped;
    const int across = std::max(result.sheetsAcross, 0);
    const int down = std::max(result.sheetsDown, 0);
    if (across == 0 || down == 0) {
        return clipped;
    }
    // The paper the sheets actually cover, which reaches at least as far as the
    // packed canvas and usually a little further.
    const double gridWidth =
        (across - 1) * advanceFor(options.pageWidthMm, options)
        + options.pageWidthMm;
    const double gridHeight =
        (down - 1) * advanceFor(options.pageHeightMm, options)
        + options.pageHeightMm;
    constexpr double tolerance = 0.01;

    for (int index = 0; index < result.placements.size(); ++index) {
        const QRectF bounds =
            boundsOf(set, result.placements.at(index), options.scale, false);
        if (bounds.isNull()) {
            continue;
        }
        if (bounds.left() < -tolerance || bounds.top() < -tolerance
            || bounds.right() > gridWidth + tolerance
            || bounds.bottom() > gridHeight + tolerance) {
            clipped.append(index);
        }
    }
    return clipped;
}

QVector<Sheet> enumerateSheets(const FlatPartSet &set,
                               const NestResult &result,
                               const NestOptions &options)
{
    QVector<Sheet> sheets;
    const int across = std::max(result.sheetsAcross, 0);
    const int down = std::max(result.sheetsDown, 0);
    if (across == 0 || down == 0) {
        return sheets;
    }

    const double advanceX = advanceFor(options.pageWidthMm, options);
    const double advanceY = advanceFor(options.pageHeightMm, options);

    QVector<QRectF> bounds;
    QVector<bool> valid;
    // In bed mode, the sheet each part belongs to, as (column, band).
    QVector<QPoint> owner;
    bounds.reserve(result.placements.size());
    valid.reserve(result.placements.size());
    owner.reserve(result.placements.size());
    for (const Placement &placement : result.placements) {
        // Validity is the placement's own, not the box's: a part that happens
        // to be a straight line has a zero-height box and is still real.
        valid.append(placement.pieceIndex >= 0
                     && placement.pieceIndex < set.pieces.size());
        bounds.append(placementBounds(set, placement, options.scale));
        // The nester keeps every part inside one sheet in bed mode, so the
        // sheet holding the placement's own origin owns it outright. Deciding
        // this by overlap instead would hand a part resting exactly on a
        // boundary to both loads, and the machine would cut it twice.
        owner.append(QPoint(
            std::clamp(static_cast<int>(std::floor(placement.x / advanceX)), 0,
                       across - 1),
            std::clamp(static_cast<int>(std::floor(placement.y / advanceY)), 0,
                       down - 1)));
    }
    // On paper the opposite rule applies: sheets are taped, so a part landing
    // on a seam belongs to both sheets, and a hairline of a part missing from a
    // page is far worse than one drawn twice.
    constexpr double slack = 0.5;

    sheets.reserve(across * down);
    // Bands are measured from the canvas bottom, matching the nester and the
    // on-screen preview, but emitted from the top down so the first page is the
    // canvas's top-left corner and the stack assembles in reading order.
    for (int band = down - 1; band >= 0; --band) {
        for (int column = 0; column < across; ++column) {
            Sheet sheet;
            sheet.column = column;
            sheet.row = (down - 1) - band;
            sheet.originMm = QPointF(column * advanceX, band * advanceY);
            sheet.widthMm = options.pageWidthMm;
            sheet.heightMm = options.pageHeightMm;

            const QRectF window(sheet.originMm.x() - slack,
                                sheet.originMm.y() - slack,
                                sheet.widthMm + slack * 2.0,
                                sheet.heightMm + slack * 2.0);
            for (int index = 0; index < bounds.size(); ++index) {
                if (!valid.at(index)) {
                    continue;
                }
                const bool mine = options.partsWithinOneSheet
                    ? owner.at(index) == QPoint(column, band)
                    : overlaps(window, bounds.at(index));
                if (mine) {
                    sheet.placements.append(index);
                }
            }
            sheets.append(sheet);
        }
    }
    return sheets;
}

bool exportPdf(const QString &path,
               const FlatPartSet &set,
               const NestResult &result,
               const NestOptions &options,
               const ExportOptions &exportOptions,
               QString *errorMessage,
               int *sheetsWritten)
{
    const auto fail = [errorMessage](const QString &message) {
        if (errorMessage != nullptr) {
            *errorMessage = message;
        }
        return false;
    };

    QVector<Sheet> sheets = enumerateSheets(set, result, options);
    if (!exportOptions.includeEmptySheets) {
        sheets.erase(std::remove_if(sheets.begin(), sheets.end(),
                                    [](const Sheet &sheet) {
                                        return sheet.isEmpty();
                                    }),
                     sheets.end());
    }
    if (sheets.isEmpty()) {
        return fail(QStringLiteral("Nothing to print — pack a layout first."));
    }

    // Checked up front: QPdfWriter reports a permission problem as a painter
    // that will not begin, which tells the user nothing about what went wrong.
    QFile probe(path);
    if (!probe.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return fail(QStringLiteral("Could not write %1: %2")
                        .arg(QDir::toNativeSeparators(path),
                             probe.errorString()));
    }
    probe.close();

    // Never smaller than the printable area the nester packed into, whatever
    // the caller passed; the alternative is content silently off the page.
    const double sheetWidthMm =
        std::max(exportOptions.sheetWidthMm, options.pageWidthMm);
    const double sheetHeightMm =
        std::max(exportOptions.sheetHeightMm, options.pageHeightMm);

    {
        QPdfWriter writer(path);
        writer.setCreator(QStringLiteral("LEparagliding Studio"));
        writer.setTitle(exportOptions.title.isEmpty()
                            ? QStringLiteral("Flat parts")
                            : exportOptions.title);
        // Both must be set before the painter exists; afterwards they are
        // ignored and the document silently comes out at the defaults.
        writer.setResolution(std::clamp(exportOptions.resolutionDpi, 72, 4800));
        writer.setPageSize(QPageSize(QSizeF(sheetWidthMm, sheetHeightMm),
                                     QPageSize::Millimeter,
                                     QString(),
                                     QPageSize::ExactMatch));
        // The margin is already accounted for: the nester packs into the
        // printable area and the exporter centres that area on the sheet.
        // Leaving Qt's own margins in would inset it a second time and the
        // print would come out under scale.
        writer.setPageMargins(QMarginsF(0.0, 0.0, 0.0, 0.0),
                              QPageLayout::Millimeter);

        QPainter painter;
        if (!painter.begin(&writer)) {
            return fail(QStringLiteral("Could not start the PDF writer for %1.")
                            .arg(QDir::toNativeSeparators(path)));
        }
        painter.setRenderHint(QPainter::Antialiasing, true);

        const double k = writer.resolution() / 25.4;
        const double marginX = (sheetWidthMm - options.pageWidthMm) / 2.0;
        const double marginY = (sheetHeightMm - options.pageHeightMm) / 2.0;
        const bool headerBand =
            exportOptions.drawFurniture && marginY >= minimumHeaderBandMm;
        const bool rulerBand =
            exportOptions.drawFurniture && marginY >= minimumRulerBandMm;

        for (int page = 0; page < sheets.size(); ++page) {
            if (page > 0) {
                writer.newPage();
            }
            const Sheet &sheet = sheets.at(page);

            SheetFrame frame;
            frame.k = k;
            frame.marginX = marginX;
            frame.marginY = marginY;
            frame.pageWidthMm = sheet.widthMm;
            frame.pageHeightMm = sheet.heightMm;
            frame.originMm = sheet.originMm;

            painter.setClipRect(frame.printableRect());
            if (exportOptions.drawFurniture) {
                // Under the parts: a registration cross laid over a cut line
                // would be one more line for the scissors to follow.
                drawRegistrationGrid(painter, frame);
            }
            for (int index : sheet.placements) {
                const Placement &placement = result.placements.at(index);
                if (placement.pieceIndex < 0
                    || placement.pieceIndex >= set.pieces.size()) {
                    continue;
                }
                drawPiece(painter, frame, set.pieces.at(placement.pieceIndex),
                          placement, options.scale, exportOptions);
            }
            painter.setClipping(false);

            if (!exportOptions.drawFurniture) {
                continue;
            }
            drawSheetEdges(painter, frame, sheet, result.sheetsAcross,
                           result.sheetsDown,
                           options.partsWithinOneSheet ? 0.0
                                                       : options.overlapMm);

            const QString header = sheetHeaderText(
                sheet, page + 1, static_cast<int>(sheets.size()), result,
                options, exportOptions);
            painter.setFont(sheetFont(headerTextMm, k, true));
            if (headerBand) {
                painter.setPen(QPen(QColor(70, 70, 78)));
                painter.drawText(
                    frame.paper(marginX, marginY - headerTextMm * 0.55),
                    header);
            } else {
                // No usable margin — a cutting bed, or paper set edge to edge.
                // The header goes inside, where it may land over geometry,
                // which is still better than an unlabelled sheet.
                painter.setPen(QPen(QColor(150, 150, 158)));
                painter.drawText(
                    frame.paper(marginX + 2.0, marginY + headerTextMm + 2.0),
                    header);
            }
            if (rulerBand) {
                drawCalibrationRuler(painter, frame, marginY);
            }
        }
        painter.end();
    }

    const QFileInfo written(path);
    if (!written.exists() || written.size() == 0) {
        return fail(QStringLiteral("The PDF writer produced no output at %1.")
                        .arg(QDir::toNativeSeparators(path)));
    }
    if (sheetsWritten != nullptr) {
        *sheetsWritten = static_cast<int>(sheets.size());
    }
    return true;
}

bool exportDxf(const QString &path,
               const FlatPartSet &set,
               const NestResult &result,
               const NestOptions &options,
               const ExportOptions &exportOptions,
               QString *errorMessage,
               QStringList *filesWritten)
{
    const auto fail = [errorMessage](const QString &message) {
        if (errorMessage != nullptr) {
            *errorMessage = message;
        }
        return false;
    };

    if (result.placements.isEmpty()) {
        return fail(QStringLiteral("Nothing to export — pack a layout first."));
    }

    // One file per job. Whole canvas by default; one file per sheet when the
    // sheets are bed loads, because a bed is cut one load at a time and each
    // load wants its own origin.
    struct Job
    {
        QString path;
        QPointF shift; // subtracted from canvas coordinates
        QRectF extents;
        QVector<int> placements;
        bool drawBedOutline = false;
    };
    QVector<Job> jobs;

    if (exportOptions.splitSheets) {
        const QFileInfo info(path);
        const QString stem = info.dir().filePath(info.completeBaseName());
        const QString suffix =
            info.suffix().isEmpty() ? QStringLiteral("dxf") : info.suffix();
        int number = 0;
        for (const Sheet &sheet : enumerateSheets(set, result, options)) {
            if (sheet.isEmpty() && !exportOptions.includeEmptySheets) {
                continue;
            }
            ++number;
            Job job;
            job.path = QStringLiteral("%1-%2.%3")
                           .arg(stem)
                           .arg(number, 2, 10, QLatin1Char('0'))
                           .arg(suffix);
            job.shift = sheet.originMm;
            job.extents = QRectF(0.0, 0.0, sheet.widthMm, sheet.heightMm);
            job.placements = sheet.placements;
            job.drawBedOutline = true;
            jobs.append(job);
        }
    } else {
        Job job;
        job.path = path;
        job.extents =
            QRectF(0.0, 0.0, result.canvasWidthMm, result.canvasHeightMm);
        job.placements.reserve(result.placements.size());
        for (int index = 0; index < result.placements.size(); ++index) {
            job.placements.append(index);
        }
        jobs.append(job);
    }

    if (jobs.isEmpty()) {
        return fail(QStringLiteral("Nothing to export — every sheet is empty."));
    }

    QStringList written;
    for (const Job &job : jobs) {
        QFile file(job.path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            return fail(QStringLiteral("Could not write %1: %2")
                            .arg(QDir::toNativeSeparators(job.path),
                                 file.errorString()));
        }
        QTextStream out(&file);
        writeDxfHeader(out, job.extents);

        // The bed outline is reference geometry on its own layer, so an
        // operator can see the load it was nested for without the machine
        // treating it as something to cut.
        if (job.drawBedOutline) {
            writeDxfPolyline(out, QStringLiteral("SHEET"),
                             {QPointF(0.0, 0.0),
                              QPointF(job.extents.width(), 0.0),
                              QPointF(job.extents.width(),
                                      job.extents.height()),
                              QPointF(0.0, job.extents.height())},
                             true);
        }

        for (int index : job.placements) {
            const Placement &placement = result.placements.at(index);
            if (placement.pieceIndex < 0
                || placement.pieceIndex >= set.pieces.size()) {
                continue;
            }
            writeDxfPlacement(out, set.pieces.at(placement.pieceIndex),
                              placement, options.scale, job.shift,
                              exportOptions);
        }

        writeDxfFooter(out);
        out.flush();
        if (out.status() != QTextStream::Ok
            || file.error() != QFileDevice::NoError) {
            return fail(QStringLiteral("Could not finish writing %1: %2")
                            .arg(QDir::toNativeSeparators(job.path),
                                 file.errorString()));
        }
        file.close();
        written.append(job.path);
    }

    if (filesWritten != nullptr) {
        *filesWritten = written;
    }
    return true;
}

} // namespace flatparts
