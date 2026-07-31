#pragma once

#include <QHash>
#include <QSet>
#include <QString>
#include <QWidget>

#include "flat_parts.h"
#include "nesting.h"

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
    flatparts::NestOptions currentOptions() const;
    void startPack();
    void showPackResult(const flatparts::NestResult &result, bool finished);

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

    NestWorker *worker_ = nullptr;
    bool packing_ = false;

    double flatArea_ = 0.0;
    bool syncingTree_ = false;
    bool syncingScale_ = false;
};
