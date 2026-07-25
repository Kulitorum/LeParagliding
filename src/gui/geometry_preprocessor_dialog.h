#pragma once

#include <QString>

#include <functional>

class QWidget;

// Modal "Pre-processor of geometry" dialog (port of the Laboratori d'envol
// standalone pre-processor). geometrySectionText is the current Section 1
// text, or empty when no design is open; applyGeometrySection receives the
// patched section text when the user applies the generated matrix, plus
// whether the cell count differs from the design's previous one — in that
// case the other sections' per-rib rows are now stale and the design cannot
// be calculated until the user updates them.
void showGeometryPreprocessorDialog(
    QWidget *parent,
    const QString &geometrySectionText,
    const std::function<void(const QString &, bool cellCountChanged)>
        &applyGeometrySection);
