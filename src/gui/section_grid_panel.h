#pragma once

#include "section_grid.h"

#include <QWidget>

#include <functional>

class QLabel;
class QPlainTextEdit;
class QTableWidget;

// Generic table editor shown below a section's text editor: every data
// record is a row, every whitespace-separated value a cell, with the
// nearest preceding comment as the row label. Cell edits are written back
// into the exact byte span of the token, so untouched formatting survives.
// Column headers and tooltips come from the per-section specs when defined
// (section_specs.cpp); other sections still get the generic grid.
class SectionGridPanel final : public QWidget
{
    Q_OBJECT
public:
    // expectedHalfRibRows: current int(cells/2)+1 derived from Section 1,
    // or 0 when unknown; used to cross-check one-record-per-rib tables.
    SectionGridPanel(int sectionNumber, QPlainTextEdit *editor,
                     std::function<int()> expectedHalfRibRows = {},
                     QWidget *parent = nullptr);

private:
    void syncFromText();
    void applyCellEdit(int row, int column);

    int sectionNumber_;
    QPlainTextEdit *editor_;
    std::function<int()> expectedHalfRibRows_;
    QTableWidget *table_;
    QLabel *status_;
    QLabel *hint_;
    lep::SectionGrid grid_;
    bool rebuildingTable_ = false;
    bool applyingEdit_ = false;
};
