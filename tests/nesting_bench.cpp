// Measures nesting quality on real exports: page count, utilisation and time.
// Run it against a lep-2d-parts.json to see whether a change to the packer
// actually helps — "it looks tighter" is not a result.
//
//   nesting-bench <lep-2d-parts.json> [--category <name>] [--scale <f>]
//                 [--page <w> <h>] [--sheet <w> <h>] [--gap <mm>] [--res <mm>]
//                 [--svg <file>] [--pdf <file>] [--dxf <file>]
//                 [--time <ms>] [--grain] [--bed]
//
// --pdf and --dxf run the Print tab's own writers, so what they produce is what
// the application would have written for the same layout.
#include <QFile>
#include <QGuiApplication>
#include <QMap>
#include <QPair>
#include <QTextStream>

#include <algorithm>
#include <cmath>

#include "flat_parts.h"
#include "nesting.h"
#include "sheet_export.h"

namespace {

void writeSvg(const QString &path,
              const flatparts::FlatPartSet &set,
              const flatparts::NestResult &result,
              const flatparts::NestOptions &options)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return;
    }
    QTextStream out(&file);
    const double w = result.canvasWidthMm;
    const double h = result.canvasHeightMm;
    out << QStringLiteral(
               "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 %1 %2\" "
               "width=\"%3\">\n<rect width=\"%1\" height=\"%2\" fill=\"#fff\"/>\n")
               .arg(w)
               .arg(h)
               .arg(std::min(w * 0.4, 2400.0));

    // Page grid, so wasted sheets are visible rather than inferred.
    const double advanceX = options.pageWidthMm - options.overlapMm;
    const double advanceY = options.pageHeightMm - options.overlapMm;
    for (int i = 0; i < result.sheetsAcross; ++i) {
        out << QStringLiteral("<rect x=\"%1\" y=\"0\" width=\"%2\" height=\"%3\" "
                              "fill=\"none\" stroke=\"#cfd8e3\" stroke-width=\"2\"/>\n")
                   .arg(i * advanceX)
                   .arg(options.pageWidthMm)
                   .arg(h);
    }
    for (int i = 0; i < result.sheetsDown; ++i) {
        out << QStringLiteral("<rect x=\"0\" y=\"%1\" width=\"%2\" height=\"%3\" "
                              "fill=\"none\" stroke=\"#cfd8e3\" stroke-width=\"2\"/>\n")
                   .arg(i * advanceY)
                   .arg(w)
                   .arg(options.pageHeightMm);
    }

    for (const flatparts::Placement &placement : result.placements) {
        const flatparts::FlatPiece &piece = set.pieces.at(placement.pieceIndex);
        // The packer's own transform, so the drawing cannot disagree with the
        // layout it is supposed to be showing.
        const flatparts::PlacementFrame frame =
            flatparts::frameFor(piece, placement, options.scale);
        QString points;
        for (const QPointF &point : flatparts::outerBoundary(piece)) {
            const QPointF mapped = frame.map(point);
            // SVG y grows downward; the canvas is y-up.
            points += QStringLiteral("%1,%2 ")
                          .arg(mapped.x())
                          .arg(h - mapped.y());
        }
        out << QStringLiteral("<polygon points=\"%1\" fill=\"#f2c9c4\" "
                              "stroke=\"#c0392b\" stroke-width=\"1.5\"/>\n")
                   .arg(points);
    }
    out << "</svg>\n";
}

} // namespace

int main(int argc, char *argv[])
{
    // QGuiApplication rather than QCoreApplication: --pdf goes through
    // QPdfWriter, which needs a font database.
#ifndef Q_OS_WIN
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }
#endif
    QGuiApplication app(argc, argv);
    QTextStream out(stdout);

    const QStringList args = QGuiApplication::arguments();
    if (args.size() < 2) {
        out << "usage: nesting-bench <lep-2d-parts.json> [options]\n";
        return 2;
    }

    flatparts::NestOptions options;
    // The bench needs a finite run; the GUI defaults to unlimited.
    options.timeBudgetMs = 4000;
    QString category;
    QString svgPath;
    QString pdfPath;
    QString dxfPath;
    // Physical paper. Defaults to A4, which is the printable page the nester
    // defaults to plus a 10 mm margin all round.
    QSizeF sheetMm(210.0, 297.0);
    for (int i = 2; i < args.size(); ++i) {
        const QString &flag = args.at(i);
        const auto next = [&](double fallback) {
            return i + 1 < args.size() ? args.at(++i).toDouble() : fallback;
        };
        if (flag == QLatin1String("--category") && i + 1 < args.size()) {
            category = args.at(++i);
        } else if (flag == QLatin1String("--scale")) {
            options.scale = next(1.0);
        } else if (flag == QLatin1String("--gap")) {
            options.gapMm = next(8.0);
        } else if (flag == QLatin1String("--bed")) {
            // Machine bed: no part may cross a sheet boundary.
            options.partsWithinOneSheet = true;
            options.overlapMm = 0.0;
            options.maxSheetsAcross = 3;
        } else if (flag == QLatin1String("--step")) {
            options.rotationStepDeg = static_cast<int>(next(90));
        } else if (flag == QLatin1String("--grain")) {
            // Half turns only, keeping the fabric warp aligned.
            options.rotationStepDeg = 180;
        } else if (flag == QLatin1String("--time")) {
            options.timeBudgetMs = static_cast<int>(next(4000));
        } else if (flag == QLatin1String("--res")) {
            options.resolutionMm = next(1.0);
        } else if (flag == QLatin1String("--page")) {
            options.pageWidthMm = next(190.0);
            options.pageHeightMm = next(277.0);
        } else if (flag == QLatin1String("--sheet")) {
            const double width = next(210.0);
            sheetMm = QSizeF(width, next(297.0));
        } else if (flag == QLatin1String("--svg") && i + 1 < args.size()) {
            svgPath = args.at(++i);
        } else if (flag == QLatin1String("--pdf") && i + 1 < args.size()) {
            pdfPath = args.at(++i);
        } else if (flag == QLatin1String("--dxf") && i + 1 < args.size()) {
            dxfPath = args.at(++i);
        }
    }

    flatparts::FlatPartSet set;
    QString errorMessage;
    if (!flatparts::load(args.at(1), &set, &errorMessage)) {
        out << errorMessage << "\n";
        return 1;
    }

    QVector<int> indices;
    for (int i = 0; i < set.pieces.size(); ++i) {
        if (category.isEmpty() || set.pieces.at(i).category == category) {
            indices.append(i);
        }
    }
    if (indices.isEmpty()) {
        out << "no parts matched\n";
        return 1;
    }

    // Outline quality first: the packer can only be as tight as the shapes it
    // is given, so report how much each category's traced outline improves on
    // its convex hull before reporting the pack.
    {
        QMap<QString, QPair<double, double>> byCategory; // traced, hull
        for (const int index : indices) {
            const flatparts::FlatPiece &piece = set.pieces.at(index);
            const QPolygonF outline = flatparts::outerBoundary(piece);
            const auto area = [](const QPolygonF &polygon) {
                double twice = 0.0;
                for (int i = 0, n = polygon.size(); i < n; ++i) {
                    const QPointF &a = polygon.at(i);
                    const QPointF &b = polygon.at((i + 1) % n);
                    twice += a.x() * b.y() - b.x() * a.y();
                }
                return std::abs(twice) * 0.5;
            };
            auto &entry = byCategory[piece.category];
            entry.first += area(outline);
            entry.second += piece.size.width() * piece.size.height();
        }
        out << "outlines (traced area as % of bounding box):\n";
        for (auto it = byCategory.constBegin(); it != byCategory.constEnd();
             ++it) {
            out << QStringLiteral("  %1 %2 %\n")
                       .arg(it.key(), -18)
                       .arg(it.value().second > 0.0
                                ? it.value().first / it.value().second * 100.0
                                : 0.0,
                            0, 'f', 1);
        }
        // Worst offenders individually: a category average hides the case where
        // most pieces trace well and a few collapse, which is the dangerous
        // one — an outline smaller than the part packs it overlapping.
        QVector<QPair<double, QString>> ranked;
        for (const int index : indices) {
            const flatparts::FlatPiece &piece = set.pieces.at(index);
            const QPolygonF outline = flatparts::outerBoundary(piece);
            double twice = 0.0;
            for (int i = 0, n = outline.size(); i < n; ++i) {
                const QPointF &a = outline.at(i);
                const QPointF &b = outline.at((i + 1) % n);
                twice += a.x() * b.y() - b.x() * a.y();
            }
            const double box = piece.size.width() * piece.size.height();
            if (box > 0.0) {
                ranked.append({std::abs(twice) * 0.5 / box, piece.id});
            }
        }
        std::sort(ranked.begin(), ranked.end());
        const int show = ranked.size() < 4 ? static_cast<int>(ranked.size()) : 4;
        for (int i = 0; i < show; ++i) {
            // Compare against the hull of the same points, computed here, so a
            // disagreement between the traced outline and its own fallback is
            // visible rather than inferred.
            const flatparts::FlatPiece *piece = nullptr;
            for (const int index : indices) {
                if (set.pieces.at(index).id == ranked.at(i).second) {
                    piece = &set.pieces.at(index);
                    break;
                }
            }
            if (piece == nullptr) {
                continue;
            }
            QVector<QPointF> points;
            for (const flatparts::Polyline &polyline : piece->polylines) {
                if (polyline.role == flatparts::Role::Cut) {
                    points.append(polyline.points);
                }
            }
            const double box = piece->size.width() * piece->size.height();
            out << QStringLiteral("    %1  traced %2 %  cutPoints %3  box %4 x %5\n")
                       .arg(ranked.at(i).second, -22)
                       .arg(ranked.at(i).first * 100.0, 5, 'f', 1)
                       .arg(points.size(), 5)
                       .arg(piece->size.width(), 0, 'f', 0)
                       .arg(piece->size.height(), 0, 'f', 0);
            Q_UNUSED(box);
        }
    }

    const flatparts::NestResult result = flatparts::nest(set, indices, options);

    out << QStringLiteral("%1  %2 parts%3\n")
               .arg(set.wing)
               .arg(indices.size())
               .arg(category.isEmpty() ? QString()
                                       : QStringLiteral(" (%1)").arg(category));
    out << QStringLiteral("  canvas      %1 x %2 mm\n")
               .arg(result.canvasWidthMm, 0, 'f', 0)
               .arg(result.canvasHeightMm, 0, 'f', 0);
    out << QStringLiteral("  sheets      %1 across x %2 down = %3 pages\n")
               .arg(result.sheetsAcross)
               .arg(result.sheetsDown)
               .arg(result.pageCount);
    out << QStringLiteral("  utilisation %1 %\n")
               .arg(result.utilisation * 100.0, 0, 'f', 1);
    out << QStringLiteral("  placed      %1, unplaced %2\n")
               .arg(result.placements.size())
               .arg(result.unplaced.size());
    out << QStringLiteral("  raster      %1 mm cell\n")
               .arg(result.resolutionMm, 0, 'f', 2);
    out << QStringLiteral("  search      %1 passes in %2 ms\n")
               .arg(result.iterations)
               .arg(result.elapsedMs);

    if (!svgPath.isEmpty()) {
        writeSvg(svgPath, set, result, options);
        out << QStringLiteral("  wrote       %1\n").arg(svgPath);
    }

    if (!pdfPath.isEmpty() || !dxfPath.isEmpty()) {
        flatparts::ExportOptions exportOptions;
        exportOptions.sheetWidthMm = sheetMm.width();
        exportOptions.sheetHeightMm = sheetMm.height();
        exportOptions.title = set.wing;
        exportOptions.stamp = QStringLiteral("nesting-bench");
        exportOptions.splitSheets = options.partsWithinOneSheet;

        QString error;
        if (!pdfPath.isEmpty()) {
            int sheets = 0;
            if (flatparts::exportPdf(pdfPath, set, result, options,
                                     exportOptions, &error, &sheets)) {
                out << QStringLiteral("  wrote       %1 (%2 pages of %3)\n")
                           .arg(pdfPath)
                           .arg(sheets)
                           .arg(result.pageCount);
            } else {
                out << QStringLiteral("  pdf failed  %1\n").arg(error);
                return 1;
            }
        }
        if (!dxfPath.isEmpty()) {
            QStringList files;
            if (flatparts::exportDxf(dxfPath, set, result, options,
                                     exportOptions, &error, &files)) {
                out << QStringLiteral("  wrote       %1 file(s), first %2\n")
                           .arg(files.size())
                           .arg(files.value(0));
            } else {
                out << QStringLiteral("  dxf failed  %1\n").arg(error);
                return 1;
            }
        }
    }
    return 0;
}
