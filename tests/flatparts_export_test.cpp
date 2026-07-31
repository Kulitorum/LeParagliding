// Tests for the Print/Cut tab's file writers (src/flatparts/sheet_export.cpp):
// how a packed canvas is cut into sheets, that the tiled PDF is produced at the
// right page count and true size, and that the DXF carries every entity on the
// right layer and inside the right extents.
//
// The invariant the machine path lives or dies by is checked here too: in
// bed mode no part may cross a sheet boundary, so every coordinate in a
// per-bed DXF must land inside that bed.

#include "flat_parts.h"
#include "nesting.h"
#include "sheet_export.h"

#include <QByteArray>
#include <QDir>
#include <QFile>
#include <QGuiApplication>
#include <QRegularExpression>
#include <QTemporaryDir>

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void check(bool condition, const char *what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

// Where the written files go. A temporary directory that is cleaned up, unless
// LEP_EXPORT_TEST_DIR names somewhere to keep them — the only practical way to
// open a PDF this produced and look at it.
class OutputDir
{
public:
    OutputDir()
    {
        const QString keep = qEnvironmentVariable("LEP_EXPORT_TEST_DIR");
        if (!keep.isEmpty() && QDir().mkpath(keep)) {
            path_ = QDir(keep).absolutePath();
            std::printf("keeping artifacts in %s\n", qPrintable(path_));
        } else if (temporary_.isValid()) {
            path_ = temporary_.path();
        }
    }

    bool isValid() const { return !path_.isEmpty(); }
    QString filePath(const QString &name) const
    {
        return QDir(path_).filePath(name);
    }

private:
    QTemporaryDir temporary_;
    QString path_;
};

// A rectangular part with a cut outline, an inset stitch line, one circular
// mark and a label — one of each thing the writers know how to draw.
flatparts::FlatPiece makePiece(const QString &id,
                               const QString &category,
                               int index,
                               double width,
                               double height)
{
    flatparts::FlatPiece piece;
    piece.id = id;
    piece.category = category;
    piece.index = index;
    piece.size = QSizeF(width, height);

    flatparts::Polyline cut;
    cut.role = flatparts::Role::Cut;
    cut.closed = true;
    cut.points = {QPointF(0.0, 0.0),
                  QPointF(width, 0.0),
                  QPointF(width, height),
                  QPointF(0.0, height)};
    piece.polylines.append(cut);

    flatparts::Polyline seam;
    seam.role = flatparts::Role::Seam;
    seam.closed = true;
    seam.points = {QPointF(12.0, 12.0),
                   QPointF(width - 12.0, 12.0),
                   QPointF(width - 12.0, height - 12.0),
                   QPointF(12.0, height - 12.0)};
    piece.polylines.append(seam);

    flatparts::Polyline mark;
    mark.role = flatparts::Role::Mark;
    mark.closed = false;
    mark.points = {QPointF(width * 0.5, 4.0), QPointF(width * 0.5, 14.0)};
    piece.polylines.append(mark);

    piece.circles.append(
        flatparts::Circle{QPointF(width * 0.5, height * 0.5), 5.0});
    piece.labels.append(
        flatparts::Label{QPointF(18.0, height * 0.5), 6.0, id});
    return piece;
}

flatparts::FlatPartSet makeSet()
{
    flatparts::FlatPartSet set;
    set.wing = QStringLiteral("Test wing");
    set.flatArea = 12.0;
    // A mix of sizes and aspect ratios, deliberately more than one A4 holds.
    const struct
    {
        const char *category;
        double width;
        double height;
    } shapes[] = {
        {"extrados-panel", 150.0, 90.0}, {"extrados-panel", 150.0, 88.0},
        {"extrados-panel", 140.0, 96.0}, {"intrados-panel", 130.0, 80.0},
        {"intrados-panel", 128.0, 84.0}, {"rib", 170.0, 60.0},
        {"rib", 168.0, 58.0},            {"rib", 166.0, 56.0},
        {"rib", 164.0, 54.0},            {"mini-rib", 70.0, 40.0},
        {"mini-rib", 68.0, 38.0},        {"mini-rib", 66.0, 36.0},
    };
    int index = 0;
    for (const auto &shape : shapes) {
        ++index;
        set.pieces.append(makePiece(
            QStringLiteral("%1-%2").arg(QString::fromLatin1(shape.category)).arg(index),
            QString::fromLatin1(shape.category), index, shape.width,
            shape.height));
    }
    return set;
}

QVector<int> allIndices(const flatparts::FlatPartSet &set)
{
    QVector<int> indices;
    for (int i = 0; i < set.pieces.size(); ++i) {
        indices.append(i);
    }
    return indices;
}

// --- a very small DXF reader, enough to interrogate what we wrote ----------

struct DxfPair
{
    int code = 0;
    QString value;
};

QVector<DxfPair> readDxf(const QString &path, bool *ok)
{
    QVector<DxfPair> pairs;
    QFile file(path);
    *ok = file.open(QIODevice::ReadOnly);
    if (!*ok) {
        return pairs;
    }
    const QList<QByteArray> lines = file.readAll().split('\n');
    for (int i = 0; i + 1 < lines.size(); i += 2) {
        bool parsed = false;
        const int code = QString::fromLatin1(lines.at(i)).trimmed().toInt(&parsed);
        if (!parsed) {
            *ok = false;
            return pairs;
        }
        QString value = QString::fromLatin1(lines.at(i + 1));
        if (value.endsWith(QLatin1Char('\r'))) {
            value.chop(1);
        }
        pairs.append(DxfPair{code, value});
    }
    return pairs;
}

int countEntities(const QVector<DxfPair> &pairs, const QString &name)
{
    int count = 0;
    for (const DxfPair &pair : pairs) {
        if (pair.code == 0 && pair.value == name) {
            ++count;
        }
    }
    return count;
}

// Layer of the entity each pair belongs to, tracked by walking the file the way
// a reader does: a code 0 starts an entity, the following code 8 names its
// layer.
int countEntitiesOnLayer(const QVector<DxfPair> &pairs,
                         const QString &name,
                         const QString &layer)
{
    int count = 0;
    bool inEntities = false;
    for (int i = 0; i < pairs.size(); ++i) {
        if (pairs.at(i).code == 2 && pairs.at(i).value == QLatin1String("ENTITIES")) {
            inEntities = true;
            continue;
        }
        if (!inEntities || pairs.at(i).code != 0 || pairs.at(i).value != name) {
            continue;
        }
        for (int j = i + 1; j < pairs.size() && pairs.at(j).code != 0; ++j) {
            if (pairs.at(j).code == 8 && pairs.at(j).value == layer) {
                ++count;
                break;
            }
        }
    }
    return count;
}

// Every vertex, circle centre and text anchor in the ENTITIES section.
QVector<QPointF> entityPoints(const QVector<DxfPair> &pairs)
{
    QVector<QPointF> points;
    bool inEntities = false;
    double x = 0.0;
    bool haveX = false;
    for (const DxfPair &pair : pairs) {
        if (pair.code == 2 && pair.value == QLatin1String("ENTITIES")) {
            inEntities = true;
            continue;
        }
        if (!inEntities) {
            continue;
        }
        if (pair.code == 10) {
            x = pair.value.toDouble();
            haveX = true;
        } else if (pair.code == 20 && haveX) {
            points.append(QPointF(x, pair.value.toDouble()));
            haveX = false;
        }
    }
    return points;
}

void testSheetEnumeration()
{
    const flatparts::FlatPartSet set = makeSet();
    flatparts::NestOptions options;
    options.pageWidthMm = 190.0;
    options.pageHeightMm = 277.0;
    options.overlapMm = 10.0;
    options.timeBudgetMs = 400;
    const flatparts::NestResult result =
        flatparts::nest(set, allIndices(set), options);
    check(result.placements.size() == set.pieces.size(),
          "every part is placed on A4");
    check(result.sheetsAcross >= 1 && result.sheetsDown >= 1,
          "the canvas covers at least one sheet");

    const QVector<flatparts::Sheet> sheets =
        flatparts::enumerateSheets(set, result, options);
    check(sheets.size() == result.sheetsAcross * result.sheetsDown,
          "one sheet per cell of the reported sheet grid");

    // Printing order is reading order: top row first, left to right.
    for (int i = 1; i < sheets.size(); ++i) {
        const flatparts::Sheet &previous = sheets.at(i - 1);
        const flatparts::Sheet &sheet = sheets.at(i);
        const bool ordered = sheet.row > previous.row
            || (sheet.row == previous.row && sheet.column > previous.column);
        check(ordered, "sheets come out in reading order");
    }

    // Sheets advance by (page - overlap) and the top row is the highest band.
    const double advanceX = options.pageWidthMm - options.overlapMm;
    const double advanceY = options.pageHeightMm - options.overlapMm;
    bool spacingOk = true;
    bool coversOrigin = false;
    for (const flatparts::Sheet &sheet : sheets) {
        const double expectedX = sheet.column * advanceX;
        const int band = (result.sheetsDown - 1) - sheet.row;
        const double expectedY = band * advanceY;
        spacingOk = spacingOk
            && std::abs(sheet.originMm.x() - expectedX) < 1e-6
            && std::abs(sheet.originMm.y() - expectedY) < 1e-6;
        if (sheet.column == 0 && band == 0) {
            coversOrigin = true;
        }
    }
    check(spacingOk, "sheet origins step by the overlapped advance");
    check(coversOrigin, "a sheet covers the canvas origin");

    // Nothing may be lost: the packer placed it, so some sheet must show it.
    QVector<bool> seen(result.placements.size(), false);
    for (const flatparts::Sheet &sheet : sheets) {
        for (int index : sheet.placements) {
            check(index >= 0 && index < seen.size(),
                  "sheet placement indices are in range");
            if (index >= 0 && index < seen.size()) {
                seen[index] = true;
            }
        }
    }
    bool everySeen = true;
    for (bool value : seen) {
        everySeen = everySeen && value;
    }
    check(everySeen, "every placement lands on at least one sheet");

    // And nothing may be invented: a placement's bounds must actually reach the
    // sheets it was assigned to.
    bool assignmentsSound = true;
    for (const flatparts::Sheet &sheet : sheets) {
        for (int index : sheet.placements) {
            const QRectF bounds = flatparts::placementBounds(
                set, result.placements.at(index), options.scale);
            const QRectF window(sheet.originMm.x(), sheet.originMm.y(),
                                sheet.widthMm, sheet.heightMm);
            assignmentsSound = assignmentsSound
                && bounds.left() <= window.right() + 1.0
                && window.left() <= bounds.right() + 1.0
                && bounds.top() <= window.bottom() + 1.0
                && window.top() <= bounds.bottom() + 1.0;
        }
    }
    check(assignmentsSound, "assigned parts really reach their sheet");

    // These parts are rectangles the packer measures exactly, so none of them
    // can reach past the sheet grid. A non-empty answer here would mean the
    // check itself is miscalibrated and its warning meaningless.
    check(flatparts::clippedPlacements(set, result, options).isEmpty(),
          "nothing is reported as clipped when nothing is");
}

void testPdf(const OutputDir &directory)
{
    const flatparts::FlatPartSet set = makeSet();
    flatparts::NestOptions options;
    options.pageWidthMm = 190.0;
    options.pageHeightMm = 277.0;
    options.overlapMm = 10.0;
    options.timeBudgetMs = 400;
    const flatparts::NestResult result =
        flatparts::nest(set, allIndices(set), options);

    flatparts::ExportOptions exportOptions;
    exportOptions.sheetWidthMm = 210.0;
    exportOptions.sheetHeightMm = 297.0;
    exportOptions.title = QStringLiteral("Test wing");
    exportOptions.stamp = QStringLiteral("2026-01-01 00:00");

    const QString path = directory.filePath(QStringLiteral("parts.pdf"));
    QString error;
    int sheetsWritten = 0;
    const bool wrote = flatparts::exportPdf(path, set, result, options,
                                            exportOptions, &error,
                                            &sheetsWritten);
    check(wrote, "the PDF is written");
    if (!wrote) {
        std::fprintf(stderr, "  reason: %s\n", qPrintable(error));
        return;
    }
    check(sheetsWritten > 0, "the PDF has pages");
    check(sheetsWritten <= result.pageCount,
          "empty sheets are dropped, never added");

    QFile file(path);
    check(file.open(QIODevice::ReadOnly), "the PDF can be reopened");
    const QByteArray bytes = file.readAll();
    check(bytes.startsWith("%PDF"), "the file really is a PDF");
    check(bytes.size() > 2000, "the PDF holds more than a header");

    // The page box, in PostScript points. This is what makes the print come out
    // at true size, so it is checked against the paper asked for rather than
    // assumed: A4 is 595.28 x 841.89 pt, and Qt rounds the box to whole points.
    // Anything further out means the document would print at the wrong scale.
    const QRegularExpression mediaBox(
        QStringLiteral("/MediaBox\\s*\\[\\s*0\\s+0\\s+([0-9.]+)\\s+([0-9.]+)"));
    const QRegularExpressionMatch box =
        mediaBox.match(QString::fromLatin1(bytes));
    check(box.hasMatch(), "the PDF declares a page box");
    if (box.hasMatch()) {
        const double points = 72.0 / 25.4;
        check(std::abs(box.captured(1).toDouble()
                       - exportOptions.sheetWidthMm * points) < 1.0,
              "the page box is as wide as the paper asked for");
        check(std::abs(box.captured(2).toDouble()
                       - exportOptions.sheetHeightMm * points) < 1.0,
              "the page box is as tall as the paper asked for");
    }

    const int pageObjects = bytes.count("/Type /Page\n")
        + bytes.count("/Type/Page\n");
    check(pageObjects > 0, "the PDF declares page objects");
    check(pageObjects == sheetsWritten,
          "the PDF's page objects match the sheets written");

    // Furniture off must still produce a valid, smaller document.
    flatparts::ExportOptions plain = exportOptions;
    plain.drawFurniture = false;
    plain.drawSeamLines = false;
    plain.drawMarks = false;
    plain.drawLabels = false;
    const QString plainPath = directory.filePath(QStringLiteral("plain.pdf"));
    int plainSheets = 0;
    check(flatparts::exportPdf(plainPath, set, result, options, plain, &error,
                               &plainSheets),
          "a cut-line-only PDF is written");
    check(plainSheets == sheetsWritten,
          "dropping the decoration does not change the sheet count");
    QFile plainFile(plainPath);
    check(plainFile.open(QIODevice::ReadOnly) && plainFile.size() > 0,
          "the cut-line-only PDF is not empty");

    // An empty layout is a refusal, not an empty file.
    const flatparts::NestResult nothing;
    check(!flatparts::exportPdf(directory.filePath(QStringLiteral("none.pdf")),
                                set, nothing, options, exportOptions, &error),
          "exporting nothing fails");
    check(!error.isEmpty(), "the refusal says why");

    // An unwritable destination is reported rather than swallowed.
    check(!flatparts::exportPdf(
              directory.filePath(QStringLiteral("no/such/dir/parts.pdf")), set,
              result, options, exportOptions, &error),
          "an unwritable path fails");
}

void testDxf(const OutputDir &directory)
{
    const flatparts::FlatPartSet set = makeSet();
    flatparts::NestOptions options;
    options.pageWidthMm = 190.0;
    options.pageHeightMm = 277.0;
    options.overlapMm = 10.0;
    options.timeBudgetMs = 400;
    const flatparts::NestResult result =
        flatparts::nest(set, allIndices(set), options);

    flatparts::ExportOptions exportOptions;
    exportOptions.sheetWidthMm = 210.0;
    exportOptions.sheetHeightMm = 297.0;

    const QString path = directory.filePath(QStringLiteral("parts.dxf"));
    QString error;
    QStringList files;
    const bool wrote = flatparts::exportDxf(path, set, result, options,
                                            exportOptions, &error, &files);
    check(wrote, "the DXF is written");
    if (!wrote) {
        std::fprintf(stderr, "  reason: %s\n", qPrintable(error));
        return;
    }
    check(files.size() == 1, "paper exports as one continuous canvas");

    bool ok = false;
    const QVector<DxfPair> pairs = readDxf(path, &ok);
    check(ok, "the DXF parses as code/value pairs");
    if (!ok) {
        return;
    }

    bool r12 = false;
    for (int i = 0; i + 1 < pairs.size(); ++i) {
        if (pairs.at(i).code == 9
            && pairs.at(i).value == QLatin1String("$ACADVER")
            && pairs.at(i + 1).value == QLatin1String("AC1009")) {
            r12 = true;
        }
    }
    check(r12, "the DXF declares R12");
    check(countEntities(pairs, QStringLiteral("LAYER")) >= 5,
          "the layer table is written");
    check(countEntities(pairs, QStringLiteral("EOF")) == 1,
          "the DXF is terminated");

    // One POLYLINE per exported polyline, one CIRCLE and one TEXT per part.
    const int placed = result.placements.size();
    check(countEntitiesOnLayer(pairs, QStringLiteral("POLYLINE"),
                               QStringLiteral("CUT")) == placed,
          "every placed part contributes its cut outline");
    check(countEntitiesOnLayer(pairs, QStringLiteral("POLYLINE"),
                               QStringLiteral("SEAM")) == placed,
          "every placed part contributes its stitch line");
    check(countEntitiesOnLayer(pairs, QStringLiteral("CIRCLE"),
                               QStringLiteral("MARK")) == placed,
          "the circular marks survive");
    check(countEntitiesOnLayer(pairs, QStringLiteral("TEXT"),
                               QStringLiteral("TEXT")) == placed,
          "every part is numbered");
    check(countEntitiesOnLayer(pairs, QStringLiteral("POLYLINE"),
                               QStringLiteral("SHEET")) == 0,
          "a paper export draws no bed outline");

    // Every coordinate sits on the canvas the result claims.
    const QVector<QPointF> points = entityPoints(pairs);
    check(!points.isEmpty(), "the DXF holds geometry");
    bool inside = true;
    for (const QPointF &point : points) {
        // The polyline elevation pair (0,0) is written for every entity, so it
        // is on the canvas by definition; everything else must be too.
        inside = inside && point.x() >= -1.0 && point.y() >= -1.0
            && point.x() <= result.canvasWidthMm + 1.0
            && point.y() <= result.canvasHeightMm + 1.0;
    }
    check(inside, "no DXF geometry escapes the packed canvas");

    // Turning the stitch line off must remove it, not hide it.
    flatparts::ExportOptions cutOnly = exportOptions;
    cutOnly.drawSeamLines = false;
    cutOnly.drawMarks = false;
    cutOnly.drawLabels = false;
    const QString cutPath = directory.filePath(QStringLiteral("cut-only.dxf"));
    check(flatparts::exportDxf(cutPath, set, result, options, cutOnly, &error),
          "a cut-only DXF is written");
    const QVector<DxfPair> cutPairs = readDxf(cutPath, &ok);
    check(ok, "the cut-only DXF parses");
    check(countEntitiesOnLayer(cutPairs, QStringLiteral("POLYLINE"),
                               QStringLiteral("SEAM")) == 0,
          "no stitch lines when they are switched off");
    check(countEntities(cutPairs, QStringLiteral("TEXT")) == 0,
          "no labels when they are switched off");
    check(countEntitiesOnLayer(cutPairs, QStringLiteral("POLYLINE"),
                               QStringLiteral("CUT")) == placed,
          "the cut outlines are still all there");
}

// The machine path. In bed mode the nester guarantees no part crosses a sheet
// boundary; the per-bed DXF is only usable if that holds, so it is checked
// against the written coordinates rather than trusted.
void testBedSplit(const OutputDir &directory)
{
    const flatparts::FlatPartSet set = makeSet();
    // A bed small enough that the parts cannot possibly fit one load, so the
    // per-load split is actually exercised.
    constexpr double bedMm = 260.0;
    flatparts::NestOptions options;
    options.pageWidthMm = bedMm;
    options.pageHeightMm = bedMm;
    options.overlapMm = 0.0;
    options.gapMm = 6.0;
    options.partsWithinOneSheet = true;
    options.maxSheetsAcross = 2;
    options.timeBudgetMs = 600;
    const flatparts::NestResult result =
        flatparts::nest(set, allIndices(set), options);
    check(result.placements.size() == set.pieces.size(),
          "every part fits the bed");

    flatparts::ExportOptions exportOptions;
    exportOptions.sheetWidthMm = bedMm;
    exportOptions.sheetHeightMm = bedMm;
    exportOptions.splitSheets = true;

    const QString path = directory.filePath(QStringLiteral("bed.dxf"));
    QString error;
    QStringList files;
    const bool wrote = flatparts::exportDxf(path, set, result, options,
                                            exportOptions, &error, &files);
    check(wrote, "the per-bed DXF set is written");
    if (!wrote) {
        std::fprintf(stderr, "  reason: %s\n", qPrintable(error));
        return;
    }
    check(files.size() > 1, "the parts need more than one bed load");
    check(files.size() <= result.pageCount,
          "empty bed loads are not written out");

    int totalCuts = 0;
    for (const QString &file : files) {
        check(QFile::exists(file), "each named bed file exists");
        bool ok = false;
        const QVector<DxfPair> pairs = readDxf(file, &ok);
        check(ok, "each bed file parses");
        if (!ok) {
            continue;
        }
        check(countEntitiesOnLayer(pairs, QStringLiteral("POLYLINE"),
                                   QStringLiteral("SHEET")) == 1,
              "each bed file outlines its bed");
        totalCuts += countEntitiesOnLayer(pairs, QStringLiteral("POLYLINE"),
                                          QStringLiteral("CUT"));

        bool insideBed = true;
        for (const QPointF &point : entityPoints(pairs)) {
            insideBed = insideBed && point.x() >= -0.5 && point.y() >= -0.5
                && point.x() <= exportOptions.sheetWidthMm + 0.5
                && point.y() <= exportOptions.sheetHeightMm + 0.5;
        }
        check(insideBed, "nothing in a bed file falls outside the bed");
    }
    // Bed mode places each part wholly inside one sheet, so the cut outlines
    // add up exactly — a part counted twice would mean it straddled a boundary
    // and would be cut in half by the machine.
    check(totalCuts == result.placements.size(),
          "each part appears in exactly one bed load");
}

} // namespace

int main(int argc, char **argv)
{
    // The PDF writer needs a QGuiApplication for its font database, and the
    // test has to run on a build machine with no display. Windows is left on
    // its default plugin, which needs no display and is the only one deployed
    // next to the executables; elsewhere the default is a plugin that wants an
    // X or Wayland server, so the offscreen one is asked for by name.
#ifndef Q_OS_WIN
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }
#endif
    QGuiApplication app(argc, argv);

    OutputDir directory;
    if (!directory.isValid()) {
        std::fprintf(stderr, "FAIL: could not create an output directory\n");
        return 1;
    }

    testSheetEnumeration();
    testPdf(directory);
    testDxf(directory);
    testBedSplit(directory);

    if (failures == 0) {
        std::printf("flatparts export tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
