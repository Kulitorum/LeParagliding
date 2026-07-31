#pragma once

#include <QHash>
#include <QSet>
#include <QSizeF>
#include <QString>
#include <QWidget>

#include "flat_parts.h"
#include "nesting.h"
#include "sheet_export.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

class FlatPartsView;
class NestWorker;

// The Print tab: pick parts, pack them, print them at 1:1 across tiled paper.
//
// The workflow is deliberately two-stage. Selecting parts and packing them are
// separate actions because packing a full wing is expensive and the selection
// is the thing the user iterates on — you print the ribs today and the panels
// next week, and each run should nest only what it will actually print.
class PrintPage : public QWidget
{
    Q_OBJECT

public:
    explicit PrintPage(QWidget *parent = nullptr);

    // Reads the engine's lep-2d-parts.json. Called after every successful
    // build; a design with no parts file leaves the tab in its empty state.
    void setPartsPath(const QString &path);

private:
    void buildTree();
    void syncSelectionFromTree();
    void handleItemChanged(QTreeWidgetItem *item, int column);
    void setAllChecked(bool checked);
    void updateSummary();
    // Scale is edited two ways — as a percentage, or by naming the flat area
    // you want — and each has to write the other back without looping.
    void applyScaleFromPercent();
    void applyScaleFromArea();
    double scaleFactor() const;
    // Physical paper, before margins. The nester only ever sees the printable
    // area, so this is the one place the real sheet size is known — and the PDF
    // needs it to place that printable area back on the page.
    QSizeF sheetSizeMm() const;
    flatparts::NestOptions currentOptions() const;
    void startPack();
    void showPackResult(const flatparts::NestResult &result, bool finished);
    void updateExportEnabled();
    flatparts::ExportOptions exportOptions() const;
    // Both take the layout from packedResult_/packedOptions_, never from the
    // live widgets: changing paper size after a pack must not silently write a
    // file laid out for the old one.
    void exportPdf();
    void exportDxf();
    QString askForPath(const QString &caption,
                       const QString &filter,
                       const QString &suffix);

    flatparts::FlatPartSet parts_;
    QSet<QString> selected_;
    QString partsPath_;

    QTreeWidget *tree_ = nullptr;
    FlatPartsView *view_ = nullptr;
    QLabel *summary_ = nullptr;
    QLabel *wingLabel_ = nullptr;

    QDoubleSpinBox *scalePercent_ = nullptr;
    QDoubleSpinBox *targetArea_ = nullptr;
    QComboBox *allowanceMode_ = nullptr;
    QComboBox *paperSize_ = nullptr;
    QDoubleSpinBox *customWidth_ = nullptr;
    QDoubleSpinBox *customHeight_ = nullptr;
    QComboBox *rotationMode_ = nullptr;
    QCheckBox *landscape_ = nullptr;
    QCheckBox *separateCategories_ = nullptr;
    QDoubleSpinBox *partGap_ = nullptr;
    QDoubleSpinBox *margin_ = nullptr;
    QDoubleSpinBox *overlap_ = nullptr;
    QPushButton *packButton_ = nullptr;

    QCheckBox *exportSeams_ = nullptr;
    QCheckBox *exportMarks_ = nullptr;
    QCheckBox *exportFurniture_ = nullptr;
    QPushButton *pdfButton_ = nullptr;
    QPushButton *dxfButton_ = nullptr;

    NestWorker *worker_ = nullptr;
    bool packing_ = false;

    // The layout an export writes. Held separately from the widgets because the
    // widgets keep moving and a written file has to be the layout that was
    // actually packed and previewed.
    flatparts::NestResult packedResult_;
    flatparts::NestOptions packedOptions_;
    QSizeF packedSheetMm_;
    bool hasPack_ = false;

    double flatArea_ = 0.0;
    bool syncingTree_ = false;
    bool syncingScale_ = false;
};
