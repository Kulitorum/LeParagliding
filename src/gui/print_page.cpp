#include "print_page.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSizeF>
#include <QSplitter>
#include <QStandardPaths>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <cmath>

#include "flat_parts_view.h"
#include "nest_worker.h"

namespace {

constexpr int idRole = Qt::UserRole + 1;

// Where the last export went. Remembered because a wing is exported many times
// over a build and the folder is never the Documents root after the first.
constexpr auto exportDirectoryKey = "print/lastExportDirectory";

// Wing names come from the design file and can hold anything; a file name
// cannot.
QString fileStem(const QString &wing)
{
    QString stem;
    stem.reserve(wing.size());
    for (const QChar &character : wing) {
        if (character.isLetterOrNumber() || character == QLatin1Char('-')
            || character == QLatin1Char('_')) {
            stem.append(character);
        } else if (character.isSpace() || character == QLatin1Char('.')) {
            stem.append(QLatin1Char('-'));
        }
    }
    while (stem.contains(QStringLiteral("--"))) {
        stem.replace(QStringLiteral("--"), QStringLiteral("-"));
    }
    stem = stem.trimmed();
    return stem.isEmpty() ? QStringLiteral("flat-parts") : stem;
}

QString describe(const flatparts::FlatPiece &piece)
{
    QString name = QStringLiteral("%1").arg(piece.index);
    if (piece.subIndex > 0) {
        name += QStringLiteral(" strip %1").arg(piece.subIndex);
    }
    if (piece.piece > 0) {
        name += QStringLiteral(" piece %1").arg(piece.piece);
    }
    return QStringLiteral("%1   %2 × %3 mm")
        .arg(name)
        .arg(piece.size.width(), 0, 'f', 0)
        .arg(piece.size.height(), 0, 'f', 0);
}

} // namespace

PrintPage::PrintPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    auto *splitter = new QSplitter(Qt::Horizontal, this);
    layout->addWidget(splitter);

    // The sidebar splits in two: the options scroll, the actions do not.
    //
    // Selection and options are taller than most windows once every group is
    // expanded, so they scroll rather than squeezing the part tree to nothing.
    // Pack and the two export buttons stay pinned below that, because they are
    // what the tab is for and a terminal action reached only by scrolling past
    // every option is one the user has to go looking for.
    auto *sidebarColumn = new QWidget(splitter);
    auto *sidebarColumnLayout = new QVBoxLayout(sidebarColumn);
    sidebarColumnLayout->setContentsMargins(0, 0, 0, 0);
    sidebarColumnLayout->setSpacing(0);

    auto *sidebarScroll = new QScrollArea(sidebarColumn);
    sidebarScroll->setWidgetResizable(true);
    sidebarScroll->setFrameShape(QFrame::NoFrame);
    sidebarScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    sidebarColumnLayout->addWidget(sidebarScroll, 1);

    auto *actions = new QWidget(sidebarColumn);
    auto *actionsLayout = new QVBoxLayout(actions);
    actionsLayout->setContentsMargins(10, 8, 10, 10);
    actionsLayout->setSpacing(8);
    sidebarColumnLayout->addWidget(actions, 0);

    auto *sidebar = new QWidget(sidebarScroll);
    auto *sidebarLayout = new QVBoxLayout(sidebar);
    sidebarLayout->setContentsMargins(10, 10, 10, 10);
    sidebarLayout->setSpacing(8);

    wingLabel_ = new QLabel(QStringLiteral("No design built yet."), sidebar);
    wingLabel_->setWordWrap(true);
    sidebarLayout->addWidget(wingLabel_);

    tree_ = new QTreeWidget(sidebar);
    tree_->setHeaderLabels({QStringLiteral("Part"), QStringLiteral("Qty")});
    tree_->setColumnWidth(0, 220);
    tree_->setUniformRowHeights(true);
    tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    sidebarLayout->addWidget(tree_, 1);

    auto *selectRow = new QHBoxLayout;
    auto *selectAll = new QPushButton(QStringLiteral("Select all"), sidebar);
    auto *selectNone = new QPushButton(QStringLiteral("Select none"), sidebar);
    selectRow->addWidget(selectAll);
    selectRow->addWidget(selectNone);
    selectRow->addStretch(1);
    sidebarLayout->addLayout(selectRow);

    auto *scaleBox = new QGroupBox(QStringLiteral("Scale"), sidebar);
    auto *scaleForm = new QFormLayout(scaleBox);
    scalePercent_ = new QDoubleSpinBox(scaleBox);
    scalePercent_->setRange(1.0, 400.0);
    scalePercent_->setDecimals(2);
    scalePercent_->setValue(100.0);
    scalePercent_->setSuffix(QStringLiteral(" %"));
    scaleForm->addRow(QStringLiteral("Print at"), scalePercent_);

    targetArea_ = new QDoubleSpinBox(scaleBox);
    targetArea_->setRange(0.1, 100.0);
    targetArea_->setDecimals(2);
    targetArea_->setSuffix(QStringLiteral(" m²"));
    scaleForm->addRow(QStringLiteral("Flat area"), targetArea_);

    allowanceMode_ = new QComboBox(scaleBox);
    allowanceMode_->addItem(QStringLiteral("Scale seam allowances too"));
    allowanceMode_->addItem(QStringLiteral("Keep allowances at true size"));
    allowanceMode_->setToolTip(QStringLiteral(
        "Seam allowances do not scale in reality — you sew a 15 mm allowance "
        "whatever the wing's size. Keeping them true rescales the stitch line "
        "and re-offsets the cut line, which is only possible for parts that "
        "have both (ribs and panels). Single-outline parts scale wholesale "
        "either way."));
    scaleForm->addRow(QStringLiteral("Allowances"), allowanceMode_);

    auto *scaleNote = new QLabel(
        QStringLiteral("For a genuinely larger wing, set <b>Wing scale</b> in "
                       "Section 1 and rebuild — the engine then recomputes "
                       "allowances, rib data and line lengths. Scaling here is "
                       "for model-size prints and test sheets."),
        scaleBox);
    scaleNote->setWordWrap(true);
    scaleForm->addRow(scaleNote);
    sidebarLayout->addWidget(scaleBox);

    auto *paperBox = new QGroupBox(QStringLiteral("Paper"), sidebar);
    auto *paperForm = new QFormLayout(paperBox);
    paperSize_ = new QComboBox(paperBox);
    paperSize_->addItems({QStringLiteral("A4"),
                          QStringLiteral("A3"),
                          QStringLiteral("A2"),
                          QStringLiteral("A1"),
                          QStringLiteral("A0"),
                          QStringLiteral("Custom / machine bed")});
    paperForm->addRow(QStringLiteral("Size"), paperSize_);

    // A cutting bed is a sheet like any other as far as nesting is concerned —
    // it just happens to be metres across — so it rides the same path rather
    // than a separate mode.
    customWidth_ = new QDoubleSpinBox(paperBox);
    customWidth_->setRange(50.0, 20000.0);
    customWidth_->setDecimals(0);
    customWidth_->setValue(3000.0);
    customWidth_->setSuffix(QStringLiteral(" mm"));
    paperForm->addRow(QStringLiteral("Bed width"), customWidth_);

    customHeight_ = new QDoubleSpinBox(paperBox);
    customHeight_->setRange(50.0, 20000.0);
    customHeight_->setDecimals(0);
    customHeight_->setValue(6000.0);
    customHeight_->setSuffix(QStringLiteral(" mm"));
    paperForm->addRow(QStringLiteral("Bed height"), customHeight_);
    // Hidden rather than greyed out for the preset sizes: a disabled "Bed
    // width: 3000 mm" sitting under "A4" invites the reader to wonder which
    // one is in force.
    paperForm->setRowVisible(customWidth_, false);
    paperForm->setRowVisible(customHeight_, false);

    landscape_ = new QCheckBox(QStringLiteral("Landscape"), paperBox);
    paperForm->addRow(QString(), landscape_);

    margin_ = new QDoubleSpinBox(paperBox);
    margin_->setRange(0.0, 40.0);
    margin_->setValue(10.0);
    margin_->setSuffix(QStringLiteral(" mm"));
    margin_->setToolTip(QStringLiteral(
        "Unprintable border. Most desktop printers cannot reach closer than "
        "about 5 mm to the edge."));
    paperForm->addRow(QStringLiteral("Margin"), margin_);

    overlap_ = new QDoubleSpinBox(paperBox);
    overlap_->setRange(0.0, 40.0);
    overlap_->setValue(10.0);
    overlap_->setSuffix(QStringLiteral(" mm"));
    overlap_->setToolTip(QStringLiteral(
        "Each sheet repeats this much of its neighbour, so sheets are aligned "
        "by overlaying printed content rather than butted edge to edge."));
    paperForm->addRow(QStringLiteral("Overlap"), overlap_);
    sidebarLayout->addWidget(paperBox);

    auto *packBox = new QGroupBox(QStringLiteral("Packing"), sidebar);
    auto *packForm = new QFormLayout(packBox);
    partGap_ = new QDoubleSpinBox(packBox);
    partGap_->setRange(0.0, 100.0);
    partGap_->setValue(8.0);
    partGap_->setSuffix(QStringLiteral(" mm"));
    partGap_->setToolTip(
        QStringLiteral("Clearance between packed parts, so scissors have "
                       "somewhere to go."));
    packForm->addRow(QStringLiteral("Gap"), partGap_);

    rotationMode_ = new QComboBox(packBox);
    rotationMode_->addItem(QStringLiteral("0°, 90°, 180°, 270° — keeps grain"));
    rotationMode_->addItem(
        QStringLiteral("Free rotation — paper templates only"));
    rotationMode_->setToolTip(QStringLiteral(
        "Quarter turns keep the weave square to the part, so the fabric behaves "
        "the same whichever way round it is cut — required when cutting fabric "
        "directly, on a laser bed or plotter. Any other angle puts the part on "
        "the bias, where a woven fabric stretches; that is only acceptable for "
        "a paper template you will lay on the fabric yourself. Free rotation "
        "nests closer but is much slower, and may coarsen the packing "
        "resolution to stay within memory."));
    packForm->addRow(QStringLiteral("Rotation"), rotationMode_);

    separateCategories_ =
        new QCheckBox(QStringLiteral("Keep categories on separate sheets"),
                      packBox);
    separateCategories_->setToolTip(QStringLiteral(
        "Off packs everything together for the fewest pages. On starts a new "
        "sheet per category, which wastes paper but keeps ribs, panels and "
        "reinforcements from being interleaved."));
    packForm->addRow(separateCategories_);
    sidebarLayout->addWidget(packBox);

    sidebarScroll->setWidget(sidebar);

    packButton_ = new QPushButton(QStringLiteral("Pack"), actions);
    packButton_->setEnabled(false);
    actionsLayout->addWidget(packButton_);

    summary_ = new QLabel(actions);
    summary_->setWordWrap(true);
    actionsLayout->addWidget(summary_);

    auto *exportBox = new QGroupBox(QStringLiteral("Export"), actions);
    auto *exportLayout = new QVBoxLayout(exportBox);

    exportSeams_ = new QCheckBox(QStringLiteral("Stitch lines"), exportBox);
    exportSeams_->setChecked(true);
    exportSeams_->setToolTip(QStringLiteral(
        "The dashed line inside the cut outline, where the seam is sewn. Off "
        "leaves only the line you cut along."));
    exportLayout->addWidget(exportSeams_);

    exportMarks_ = new QCheckBox(QStringLiteral("Marks and part numbers"),
                                 exportBox);
    exportMarks_->setChecked(true);
    exportMarks_->setToolTip(QStringLiteral(
        "Registration ticks, vent outlines, rod positions and the number each "
        "part carries on the plan. Worth keeping — an unnumbered pile of "
        "similar-looking panels is unsewable."));
    exportLayout->addWidget(exportMarks_);

    exportFurniture_ = new QCheckBox(
        QStringLiteral("Sheet border, alignment marks and ruler"), exportBox);
    exportFurniture_->setChecked(true);
    exportFurniture_->setToolTip(QStringLiteral(
        "PDF only. Adds the page border, 100 mm registration crosses shared "
        "with the neighbouring sheets, a header naming the sheet, and a "
        "measuring bar for checking the printer did not scale the page."));
    exportLayout->addWidget(exportFurniture_);

    auto *exportRow = new QHBoxLayout;
    pdfButton_ = new QPushButton(QStringLiteral("Export PDF…"), exportBox);
    pdfButton_->setToolTip(QStringLiteral(
        "Writes one page per sheet at true size, ready to print and tape."));
    dxfButton_ = new QPushButton(QStringLiteral("Export DXF…"), exportBox);
    dxfButton_->setToolTip(QStringLiteral(
        "Writes the nested layout as DXF for a plotter or cutting table, with "
        "the cut line, stitch line and marks on separate layers."));
    exportRow->addWidget(pdfButton_);
    exportRow->addWidget(dxfButton_);
    exportLayout->addLayout(exportRow);
    actionsLayout->addWidget(exportBox);

    view_ = new FlatPartsView(splitter);
    splitter->addWidget(sidebarColumn);
    splitter->addWidget(view_);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({340, 900});

    connect(tree_, &QTreeWidget::itemChanged, this,
            &PrintPage::handleItemChanged);
    connect(selectAll, &QPushButton::clicked, this,
            [this] { setAllChecked(true); });
    connect(selectNone, &QPushButton::clicked, this,
            [this] { setAllChecked(false); });
    connect(tree_, &QTreeWidget::currentItemChanged, this,
            [this](QTreeWidgetItem *current, QTreeWidgetItem *) {
                view_->setHighlighted(
                    current == nullptr
                        ? QString()
                        : current->data(0, idRole).toString());
            });
    connect(view_, &FlatPartsView::pieceClicked, this,
            [this](const QString &id) {
                for (QTreeWidgetItemIterator it(tree_); *it != nullptr; ++it) {
                    if ((*it)->data(0, idRole).toString() == id) {
                        tree_->setCurrentItem(*it);
                        tree_->scrollToItem(*it);
                        break;
                    }
                }
            });
    connect(paperSize_, &QComboBox::currentIndexChanged, this,
            [this, paperForm](int index) {
                const bool custom = index >= 5;
                paperForm->setRowVisible(customWidth_, custom);
                paperForm->setRowVisible(customHeight_, custom);
                // Overlap and margin are paper ideas: a bed is not taped
                // together and has no unprintable border.
                paperForm->setRowVisible(overlap_, !custom);
                if (custom) {
                    overlap_->setValue(0.0);
                }
            });
    connect(scalePercent_, &QDoubleSpinBox::valueChanged, this,
            &PrintPage::applyScaleFromPercent);
    connect(targetArea_, &QDoubleSpinBox::valueChanged, this,
            &PrintPage::applyScaleFromArea);

    worker_ = new NestWorker(this);
    connect(worker_, &NestWorker::progress, this,
            [this](const flatparts::NestResult &result, int generation) {
                if (generation == worker_->generation()) {
                    showPackResult(result, false);
                }
            });
    connect(worker_, &NestWorker::finished, this,
            [this](const flatparts::NestResult &result, int generation) {
                if (generation == worker_->generation()) {
                    showPackResult(result, true);
                }
            });
    connect(packButton_, &QPushButton::clicked, this, [this] {
        if (packing_) {
            // Stopping keeps the best layout found so far — the worker reports
            // it on the way out, so there is nothing to discard.
            worker_->cancel();
            return;
        }
        startPack();
    });
    connect(pdfButton_, &QPushButton::clicked, this, &PrintPage::exportPdf);
    connect(dxfButton_, &QPushButton::clicked, this, &PrintPage::exportDxf);

    updateExportEnabled();
}

QSizeF PrintPage::sheetSizeMm() const
{
    // ISO A sizes in millimetres, index-aligned with the combo; the last entry
    // is the custom bed.
    static const QVector<QSizeF> paper{QSizeF(210, 297),
                                       QSizeF(297, 420),
                                       QSizeF(420, 594),
                                       QSizeF(594, 841),
                                       QSizeF(841, 1189)};
    const bool custom = paperSize_->currentIndex() >= paper.size();
    QSizeF sheet = custom
        ? QSizeF(customWidth_->value(), customHeight_->value())
        : paper.value(paperSize_->currentIndex(), QSizeF(210, 297));
    if (landscape_->isChecked()) {
        sheet.transpose();
    }
    return sheet;
}

flatparts::NestOptions PrintPage::currentOptions() const
{
    const QSizeF sheet = sheetSizeMm();
    // Index 5 and beyond is the custom bed; anything before it is ISO A paper.
    const bool custom = paperSize_->currentIndex() >= 5;

    flatparts::NestOptions options;
    const double margin = margin_->value();
    options.pageWidthMm = std::max(sheet.width() - margin * 2.0, 10.0);
    options.pageHeightMm = std::max(sheet.height() - margin * 2.0, 10.0);
    options.overlapMm = overlap_->value();
    options.gapMm = partGap_->value();
    options.scale = scaleFactor();
    options.rotationStepDeg = rotationMode_->currentIndex() == 1 ? 15 : 90;
    // A bed cuts one load at a time, so a part crossing a bed boundary would be
    // cut in half. On paper the opposite holds: straddling a sheet edge is
    // exactly what the overlap and registration marks are for.
    options.partsWithinOneSheet = custom;
    // Candidate canvas widths are counted in sheets, so a sheet metres across
    // needs far fewer of them; ten 3 m beds side by side is a 30 m canvas
    // nobody wants and a lot of wasted search.
    options.maxSheetsAcross =
        sheet.width() >= 1000.0 ? 3 : 10;
    return options;
}

void PrintPage::startPack()
{
    QVector<int> indices;
    for (int i = 0; i < parts_.pieces.size(); ++i) {
        if (selected_.contains(parts_.pieces.at(i).id)) {
            indices.append(i);
        }
    }
    if (indices.isEmpty()) {
        summary_->setText(QStringLiteral("Select at least one part to pack."));
        return;
    }

    packing_ = true;
    hasPack_ = false;
    packButton_->setText(QStringLiteral("Stop"));
    updateExportEnabled();
    summary_->setText(
        QStringLiteral("Packing %1 parts — keeps improving until you press "
                       "Stop.")
            .arg(indices.size()));
    // No time budget: the search keeps finding better layouts for as long as it
    // runs, so how long to spend is the user's call, not a constant.
    flatparts::NestOptions options = currentOptions();
    options.timeBudgetMs = 0;
    // Frozen here, and used by the preview and both exporters from now on. The
    // sidebar stays editable while a pack runs, and a layout redrawn against a
    // paper size it was not packed for is a layout that lies.
    packedOptions_ = options;
    packedSheetMm_ = sheetSizeMm();
    worker_->start(parts_, indices, options);
}

void PrintPage::showPackResult(const flatparts::NestResult &result, bool finished)
{
    packedResult_ = result;
    hasPack_ = !result.placements.isEmpty();
    view_->setPackedLayout(result, packedOptions_);
    if (finished) {
        packing_ = false;
        packButton_->setText(QStringLiteral("Pack"));
    }
    updateExportEnabled();

    QString text =
        QStringLiteral("%1 pages · %2 x %3 sheets · %4% used · %5 layouts "
                       "tried in %6.%7 s")
            .arg(result.pageCount)
            .arg(result.sheetsAcross)
            .arg(result.sheetsDown)
            .arg(result.utilisation * 100.0, 0, 'f', 1)
            .arg(result.iterations)
            .arg(result.elapsedMs / 1000)
            .arg((result.elapsedMs % 1000) / 100);
    text.prepend(finished ? QStringLiteral("Stopped — best found: ")
                          : QStringLiteral("Searching… best so far: "));
    if (!result.unplaced.isEmpty()) {
        // Worth saying loudly: it means the paper is too small at this scale,
        // and a silently short part list is the kind of thing found after
        // cutting.
        text += QStringLiteral("\n%1 part(s) did not fit on the sheet at this "
                               "scale.")
                    .arg(result.unplaced.size());
    }
    if (finished) {
        // Only once the search has stopped: this costs a pass over every part
        // and the answer barely moves between successive best layouts.
        const QVector<int> clipped =
            flatparts::clippedPlacements(parts_, result, packedOptions_);
        if (!clipped.isEmpty()) {
            text += QStringLiteral("\n%1 part(s) have cut geometry reaching "
                                   "past the sheets; that much will be missing "
                                   "from the export.")
                        .arg(clipped.size());
        }
    }
    summary_->setText(text);
}

void PrintPage::setPartsPath(const QString &path)
{
    partsPath_ = path;
    parts_ = flatparts::FlatPartSet();
    selected_.clear();
    // A rebuild invalidates the pack: the placements index into the old part
    // list, so both the preview and the export buttons have to let go of it.
    packedResult_ = flatparts::NestResult();
    hasPack_ = false;
    view_->clearPackedLayout();
    updateExportEnabled();

    QString errorMessage;
    if (!QFileInfo::exists(path)
        || !flatparts::load(path, &parts_, &errorMessage)) {
        wingLabel_->setText(
            QStringLiteral("No flat parts available. Build the design first."));
        tree_->clear();
        view_->setParts(parts_);
        view_->setSelection({});
        packButton_->setEnabled(false);
        summary_->clear();
        return;
    }

    flatArea_ = parts_.flatArea;
    wingLabel_->setText(
        flatArea_ > 0.0
            ? QStringLiteral("<b>%1</b><br>%2 parts · %3 m² flat")
                  .arg(parts_.wing)
                  .arg(parts_.pieces.size())
                  .arg(flatArea_, 0, 'f', 2)
            : QStringLiteral("<b>%1</b><br>%2 parts")
                  .arg(parts_.wing)
                  .arg(parts_.pieces.size()));

    syncingScale_ = true;
    scalePercent_->setValue(100.0);
    if (flatArea_ > 0.0) {
        targetArea_->setValue(flatArea_);
        targetArea_->setEnabled(true);
    } else {
        targetArea_->setEnabled(false);
    }
    syncingScale_ = false;

    buildTree();
    view_->setParts(parts_);
    syncSelectionFromTree();
    packButton_->setEnabled(true);
}

void PrintPage::buildTree()
{
    syncingTree_ = true;
    tree_->clear();
    for (const QString &category : parts_.categories()) {
        auto *group = new QTreeWidgetItem(tree_);
        group->setText(0, flatparts::FlatPartSet::categoryLabel(category));
        group->setFlags(group->flags() | Qt::ItemIsUserCheckable
                        | Qt::ItemIsAutoTristate);
        group->setCheckState(0, Qt::Checked);
        group->setExpanded(false);

        int count = 0;
        for (const flatparts::FlatPiece &piece : parts_.pieces) {
            if (piece.category != category) {
                continue;
            }
            ++count;
            auto *item = new QTreeWidgetItem(group);
            item->setText(0, describe(piece));
            item->setData(0, idRole, piece.id);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(0, Qt::Checked);
        }
        group->setText(1, QString::number(count));
    }
    syncingTree_ = false;
}

void PrintPage::handleItemChanged(QTreeWidgetItem *item, int column)
{
    if (syncingTree_ || item == nullptr || column != 0) {
        return;
    }
    // Propagate a group's state down; Qt's auto-tristate handles the way back
    // up on its own.
    if (item->childCount() > 0) {
        syncingTree_ = true;
        const Qt::CheckState state = item->checkState(0);
        if (state != Qt::PartiallyChecked) {
            for (int index = 0; index < item->childCount(); ++index) {
                item->child(index)->setCheckState(0, state);
            }
        }
        syncingTree_ = false;
    }
    syncSelectionFromTree();
}

void PrintPage::setAllChecked(bool checked)
{
    syncingTree_ = true;
    const Qt::CheckState state = checked ? Qt::Checked : Qt::Unchecked;
    for (int index = 0; index < tree_->topLevelItemCount(); ++index) {
        QTreeWidgetItem *group = tree_->topLevelItem(index);
        group->setCheckState(0, state);
        for (int child = 0; child < group->childCount(); ++child) {
            group->child(child)->setCheckState(0, state);
        }
    }
    syncingTree_ = false;
    syncSelectionFromTree();
}

void PrintPage::syncSelectionFromTree()
{
    selected_.clear();
    for (QTreeWidgetItemIterator it(tree_); *it != nullptr; ++it) {
        const QString id = (*it)->data(0, idRole).toString();
        if (!id.isEmpty() && (*it)->checkState(0) == Qt::Checked) {
            selected_.insert(id);
        }
    }
    view_->setSelection(selected_);
    updateSummary();
}

double PrintPage::scaleFactor() const
{
    return scalePercent_->value() / 100.0;
}

void PrintPage::applyScaleFromPercent()
{
    if (syncingScale_) {
        return;
    }
    syncingScale_ = true;
    if (flatArea_ > 0.0) {
        // Area goes as the square of the linear scale.
        const double factor = scaleFactor();
        targetArea_->setValue(flatArea_ * factor * factor);
    }
    syncingScale_ = false;
    view_->setScale(scaleFactor());
    updateSummary();
}

void PrintPage::applyScaleFromArea()
{
    if (syncingScale_ || flatArea_ <= 0.0) {
        return;
    }
    syncingScale_ = true;
    scalePercent_->setValue(std::sqrt(targetArea_->value() / flatArea_)
                            * 100.0);
    syncingScale_ = false;
    view_->setScale(scaleFactor());
    updateSummary();
}

void PrintPage::updateExportEnabled()
{
    // Only between packs. Exporting mid-search would write whichever best
    // happened to be current, and the file would not match what the user sees
    // a second later.
    const bool ready = hasPack_ && !packing_;
    pdfButton_->setEnabled(ready);
    dxfButton_->setEnabled(ready);
    const QString reason =
        packing_ ? QStringLiteral("Stop the pack to export it.")
                 : QStringLiteral("Pack a layout first.");
    pdfButton_->setToolTip(
        ready ? QStringLiteral("Writes one page per sheet at true size, ready "
                               "to print and tape.")
              : reason);
    dxfButton_->setToolTip(
        ready ? QStringLiteral("Writes the nested layout as DXF for a plotter "
                               "or cutting table, with the cut line, stitch "
                               "line and marks on separate layers.")
              : reason);
}

flatparts::ExportOptions PrintPage::exportOptions() const
{
    flatparts::ExportOptions options;
    options.sheetWidthMm = packedSheetMm_.width();
    options.sheetHeightMm = packedSheetMm_.height();
    options.title = parts_.wing.isEmpty() ? QStringLiteral("Flat parts")
                                          : parts_.wing;
    options.subtitle = QStringLiteral("%1 parts").arg(packedResult_.placements.size());
    options.stamp =
        QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm"));
    options.drawSeamLines = exportSeams_->isChecked();
    options.drawMarks = exportMarks_->isChecked();
    options.drawLabels = exportMarks_->isChecked();
    options.drawFurniture = exportFurniture_->isChecked();
    // A bed load is cut in one pass, so each load is its own file in its own
    // coordinates; paper sheets are one continuous canvas and stay one file.
    options.splitSheets = packedOptions_.partsWithinOneSheet;
    return options;
}

QString PrintPage::askForPath(const QString &caption,
                              const QString &filter,
                              const QString &suffix)
{
    QSettings settings;
    QString directory =
        settings.value(QLatin1String(exportDirectoryKey)).toString();
    if (directory.isEmpty() || !QFileInfo::exists(directory)) {
        directory = QStandardPaths::writableLocation(
            QStandardPaths::DocumentsLocation);
    }
    const QString suggestion =
        QDir(directory).filePath(QStringLiteral("%1-parts.%2")
                                     .arg(fileStem(parts_.wing), suffix));

    QString path = QFileDialog::getSaveFileName(this, caption, suggestion,
                                                filter);
    if (path.isEmpty()) {
        return path;
    }
    // Qt only appends the suffix on platforms whose dialog does it; typing a
    // bare name on the others would otherwise produce an extensionless file.
    if (QFileInfo(path).suffix().isEmpty()) {
        path += QLatin1Char('.') + suffix;
    }
    settings.setValue(QLatin1String(exportDirectoryKey),
                      QFileInfo(path).absolutePath());
    return path;
}

void PrintPage::exportPdf()
{
    if (!hasPack_) {
        return;
    }
    const QString path = askForPath(QStringLiteral("Export printable sheets"),
                                    QStringLiteral("PDF documents (*.pdf)"),
                                    QStringLiteral("pdf"));
    if (path.isEmpty()) {
        return;
    }

    QString error;
    int sheets = 0;
    if (!flatparts::exportPdf(path, parts_, packedResult_, packedOptions_,
                              exportOptions(), &error, &sheets)) {
        QMessageBox::warning(this, QStringLiteral("Export failed"), error);
        return;
    }

    QString text = QStringLiteral("Wrote %1 page(s) to %2.")
                       .arg(sheets)
                       .arg(QDir::toNativeSeparators(path));
    const int skipped = packedResult_.pageCount - sheets;
    if (skipped > 0) {
        text += QStringLiteral(" %1 empty sheet(s) skipped.").arg(skipped);
    }
    if (!packedResult_.unplaced.isEmpty()) {
        text += QStringLiteral(" %1 part(s) did not fit and were left out.")
                    .arg(packedResult_.unplaced.size());
    }
    text += QStringLiteral("\nPrint at 100% — the sheets carry a 100 mm bar to "
                           "check against.");
    summary_->setText(text);
}

void PrintPage::exportDxf()
{
    if (!hasPack_) {
        return;
    }
    const QString path = askForPath(QStringLiteral("Export nested DXF"),
                                    QStringLiteral("DXF drawings (*.dxf)"),
                                    QStringLiteral("dxf"));
    if (path.isEmpty()) {
        return;
    }

    QString error;
    QStringList files;
    if (!flatparts::exportDxf(path, parts_, packedResult_, packedOptions_,
                              exportOptions(), &error, &files)) {
        QMessageBox::warning(this, QStringLiteral("Export failed"), error);
        return;
    }

    QString text = files.size() == 1
        ? QStringLiteral("Wrote %1.").arg(QDir::toNativeSeparators(files.first()))
        : QStringLiteral("Wrote %1 files, one per bed load, next to %2.")
              .arg(files.size())
              .arg(QDir::toNativeSeparators(QFileInfo(path).fileName()));
    if (!packedResult_.unplaced.isEmpty()) {
        text += QStringLiteral(" %1 part(s) did not fit and were left out.")
                    .arg(packedResult_.unplaced.size());
    }
    text += QStringLiteral("\nLayers: CUT, SEAM, MARK, TEXT.");
    summary_->setText(text);
}

void PrintPage::updateSummary()
{
    if (parts_.isEmpty()) {
        summary_->clear();
        return;
    }
    const double factor = scaleFactor();
    double fabricArea = 0.0;
    for (const flatparts::FlatPiece &piece : parts_.pieces) {
        if (selected_.contains(piece.id)) {
            fabricArea += piece.area() * factor * factor;
        }
    }
    // Bounding-box area, so it reads as an upper bound rather than a promise;
    // the packer will report the real sheet count.
    summary_->setText(
        QStringLiteral("%1 of %2 parts selected · about %3 m² of paper "
                       "before nesting")
            .arg(selected_.size())
            .arg(parts_.pieces.size())
            .arg(fabricArea / 1'000'000.0, 0, 'f', 1));
}
