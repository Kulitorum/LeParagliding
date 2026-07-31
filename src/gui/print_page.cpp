#include "print_page.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSizeF>
#include <QSplitter>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <cmath>

#include "flat_parts_view.h"
#include "nest_worker.h"

namespace {

constexpr int idRole = Qt::UserRole + 1;

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

    // The sidebar is taller than most windows once every option group is
    // expanded, so it scrolls rather than squeezing the part tree to nothing.
    auto *sidebarScroll = new QScrollArea(splitter);
    sidebarScroll->setWidgetResizable(true);
    sidebarScroll->setFrameShape(QFrame::NoFrame);
    sidebarScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

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

    packButton_ = new QPushButton(QStringLiteral("Pack"), sidebar);
    packButton_->setEnabled(false);
    sidebarLayout->addWidget(packButton_);

    summary_ = new QLabel(sidebar);
    summary_->setWordWrap(true);
    sidebarLayout->addWidget(summary_);

    sidebarScroll->setWidget(sidebar);

    view_ = new FlatPartsView(splitter);
    splitter->addWidget(sidebarScroll);
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
}

flatparts::NestOptions PrintPage::currentOptions() const
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
    packButton_->setText(QStringLiteral("Stop"));
    summary_->setText(
        QStringLiteral("Packing %1 parts — keeps improving until you press "
                       "Stop.")
            .arg(indices.size()));
    // No time budget: the search keeps finding better layouts for as long as it
    // runs, so how long to spend is the user's call, not a constant.
    flatparts::NestOptions options = currentOptions();
    options.timeBudgetMs = 0;
    worker_->start(parts_, indices, options);
}

void PrintPage::showPackResult(const flatparts::NestResult &result, bool finished)
{
    view_->setPackedLayout(result, currentOptions());
    if (finished) {
        packing_ = false;
        packButton_->setText(QStringLiteral("Pack"));
    }

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
    summary_->setText(text);
}

void PrintPage::setPartsPath(const QString &path)
{
    partsPath_ = path;
    parts_ = flatparts::FlatPartSet();
    selected_.clear();

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
