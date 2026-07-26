#pragma once

#include "panel_undo.h"
#include "section1_curves.h"
#include "spline_fit.h"

#include <QJsonObject>
#include <QWidget>

#include <functional>
#include <map>
#include <string>
#include <vector>

class CurveEditor;
class QLabel;
class QPlainTextEdit;
class QPushButton;

// Graphical editor for the Section 1 rib geometry matrix, shown below the
// section's text editor. Curves start point-by-point; "Convert to
// B-splines" fits selected curves within a tolerance (live preview), after
// which the spline is the source of truth: its control points are the
// editable handles, the matrix rows are the spline sampled at the current
// rib stations, and the text is regenerated on every commit. Spline
// definitions persist in the design file's Studio trailer via the
// load/store callbacks; the domain is normalized to [0, 1] so the same
// splines serve any future rib count. Manual text edits that drift beyond
// the text's own rounding flag the spline as stale, with re-fit /
// apply-spline offered in the status row.
class Section1CurvePanel final : public QWidget
{
    Q_OBJECT
public:
    Section1CurvePanel(QPlainTextEdit *editor,
                       std::function<QJsonObject()> loadSplines,
                       std::function<void(const QJsonObject &)> storeSplines,
                       QWidget *parent = nullptr);

private:
    // Everything a graphical edit can touch; undo restores it absolutely.
    struct UndoState
    {
        QString text;
        QJsonObject splines;
    };
    UndoState captureState() const;
    void restoreState(const UndoState &state);
    void pushUndo(UndoState before);

    void loadSplinesFromDocument();
    void persistSplines();
    void syncFromText();
    void rebuildDisplay();
    void updateToolbar();
    void commitSeries(const QString &seriesId);
    void updateDescription(const QString &seriesId);
    void showConvertDialog();
    void removeSelectedSpline();
    void refitStaleColumns();
    void applyStaleSplines();
    // Writes new column values into the matrix text; one undo step. When
    // skipUnchangedValues is set, rows whose value is bit-identical keep
    // their original formatting untouched.
    void patchRows(const std::map<int, std::vector<double>> &valuesByColumn,
                   bool skipUnchangedValues);
    const std::map<std::string, lep::BSpline> &activeSplines() const;
    double absoluteTolerance(int column, double percent) const;
    std::vector<double> columnValues(int column) const;
    int columnIndexById(const std::string &id) const;

    QPlainTextEdit *editor_;
    CurveEditor *curves_;
    QLabel *description_;
    QLabel *status_;
    QPushButton *convertButton_;
    QPushButton *removeButton_;
    QPushButton *refitButton_;
    QPushButton *applyButton_;
    std::function<QJsonObject()> loadSplines_;
    std::function<void(const QJsonObject &)> storeSplines_;
    lep::Section1Matrix matrix_;
    QStringList matrixProblems_;
    bool matrixUsable_ = false;
    std::map<std::string, lep::BSpline> splines_;
    std::map<std::string, lep::BSpline> previewSplines_;
    bool previewActive_ = false;
    std::vector<std::string> staleColumns_;
    double tolerancePercent_ = 1.0;
    bool applyingEdit_ = false;
    PanelUndoStack<UndoState> undo_;
};
