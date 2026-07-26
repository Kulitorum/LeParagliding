#pragma once

#include <QString>
#include <QVector>

// Per-section metadata driving the generic grid editors: column headers and
// tooltips for the section's dominant table (applied by column index), and a
// one-line structural reminder. Sections without a spec still get the
// generic grid with numbered columns.

struct SectionColumnSpec
{
    QString header;
    QString tooltip;
};

// A column of the section's dominant table that is worth editing as a curve
// over the row order (one row per rib). The gates mark curves the engine
// ignores in the current data: such curves grey out instead of drawing.
struct SectionCurveColumn
{
    int column = 0;
    double minValue = -1e300;
    double maxValue = 1e300;
    int decimals = 2;
    // Enabled when any row's value at gateColumn is >= gateThreshold
    // (e.g. the anchor-count column reaching this anchor). -1 = no gate.
    int gateColumn = -1;
    double gateThreshold = 0.0;
    // Enabled when any row has a non-zero value in this column itself
    // (e.g. the brake anchor, active only when set).
    bool gateOwnNonZero = false;
    // Shown as the greyed chip's tooltip when the gate disables the curve.
    QString disabledNote;
};

struct SectionSpec
{
    QVector<SectionColumnSpec> columns;
    QString structure; // one-line reminder shown under the grid
    // True for always-on tables with exactly one record per half-wing rib
    // (int(cells/2)+1 rows); the grid then cross-checks its record count
    // against Section 1's declared cell count.
    bool onePerRib = false;
    // Non-empty: the section also gets a curve view of these columns,
    // taken from the uniform rows of the dominant table.
    QVector<SectionCurveColumn> curveColumns;
};

// Spec for a section, or nullptr when only the generic grid applies.
const SectionSpec *sectionSpec(int number);
