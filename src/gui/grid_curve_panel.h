#pragma once

#include "panel_undo.h"
#include "section_grid.h"

#include <QWidget>

class CurveEditor;
class QLabel;
class QPlainTextEdit;

// Curve view over a section whose dominant table has one uniform row per
// rib (declared via SectionSpec::curveColumns): each listed column becomes
// an editable curve over the row order. Edits are written back token by
// token, only for the values that actually changed, so untouched rows keep
// their formatting. Used beside the value grid in a tab pair.
class GridCurvePanel final : public QWidget
{
    Q_OBJECT
public:
    GridCurvePanel(int sectionNumber, QPlainTextEdit *editor,
                   QWidget *parent = nullptr);

private:
    void syncFromText();
    void commitSeries(const QString &seriesId);
    void updateDescription(const QString &seriesId);

    int sectionNumber_;
    QPlainTextEdit *editor_;
    CurveEditor *curves_;
    QLabel *description_;
    QLabel *status_;
    void restoreText(const QString &text);

    lep::SectionGrid grid_;
    QVector<int> uniformRows_; // indexes into grid_.rows used by the curves
    bool applyingEdit_ = false;
    PanelUndoStack<QString> undo_;
};
