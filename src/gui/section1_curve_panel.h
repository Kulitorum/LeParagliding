#pragma once

#include "section1_curves.h"

#include <QWidget>

class CurveEditor;
class QLabel;
class QPlainTextEdit;

// Graphical editor for the Section 1 rib geometry matrix, shown below the
// section's text editor. Parses the matrix out of the text on every text
// change, shows each column as a selectable/editable curve over the rib
// number, explains the selected parameter, and reports problems in plain
// language. Curve edits are written back into the text editor by patching
// only the changed matrix rows, so the editor's undo history stays usable.
class Section1CurvePanel final : public QWidget
{
    Q_OBJECT
public:
    explicit Section1CurvePanel(QPlainTextEdit *editor,
                                QWidget *parent = nullptr);

private:
    void syncFromText();
    void commitSeries(const QString &seriesId);
    void updateDescription(const QString &seriesId);

    QPlainTextEdit *editor_;
    CurveEditor *curves_;
    QLabel *description_;
    QLabel *status_;
    lep::Section1Matrix matrix_;
    bool applyingEdit_ = false;
};
