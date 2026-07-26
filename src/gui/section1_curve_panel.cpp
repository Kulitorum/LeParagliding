#include "section1_curve_panel.h"

#include "curve_editor.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QStringList>
#include <QTextBlock>
#include <QTextCursor>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace {

// Categorical palette validated for the app's dark surface (#0e1726) with
// scripts/validate_palette.js from the dataviz reference: all adjacent pairs
// clear the CVD and normal-vision separation floors and 3:1 contrast.
const QColor kSeriesPalette[8] = {
    QColor(0x39, 0x87, 0xe5), // blue
    QColor(0x00, 0x83, 0x00), // green
    QColor(0xd5, 0x51, 0x81), // magenta
    QColor(0xc9, 0x85, 0x00), // yellow
    QColor(0x19, 0x9e, 0x70), // aqua
    QColor(0xd9, 0x59, 0x26), // orange
    QColor(0x90, 0x85, 0xe9), // violet
    QColor(0xe6, 0x67, 0x67), // red
};

// Columns 9 and 10 (Rot_z, Pos_z) reuse the first hues with a dashed stroke
// instead of extending the palette past its validated eight slots.
void styleForColumn(int column, QColor *color, Qt::PenStyle *penStyle)
{
    if (column <= 8) {
        *color = kSeriesPalette[column - 1];
        *penStyle = Qt::SolidLine;
    } else {
        *color = kSeriesPalette[column == 9 ? 0 : 2];
        *penStyle = Qt::DashLine;
    }
}

const QString kOkColor = QStringLiteral("#93a4ba");
const QString kWarningColor = QStringLiteral("#fab219");
const QString kErrorColor = QStringLiteral("#e66767");

QJsonObject splineToJson(const lep::BSpline &spline)
{
    QJsonObject object;
    object.insert(QStringLiteral("degree"), spline.degree);
    QJsonArray knots;
    for (const double knot : spline.knots)
        knots.append(knot);
    object.insert(QStringLiteral("knots"), knots);
    QJsonArray control;
    for (const double value : spline.control)
        control.append(value);
    object.insert(QStringLiteral("control"), control);
    return object;
}

lep::BSpline splineFromJson(const QJsonObject &object)
{
    lep::BSpline spline;
    spline.degree = object.value(QStringLiteral("degree")).toInt();
    for (const QJsonValue &value :
         object.value(QStringLiteral("knots")).toArray())
        spline.knots.push_back(value.toDouble());
    for (const QJsonValue &value :
         object.value(QStringLiteral("control")).toArray())
        spline.control.push_back(value.toDouble());
    return spline;
}

// Slider position (0..300) <-> tolerance percent (0.01..10, log scale).
double sliderToPercent(int position)
{
    return std::pow(10.0, position / 100.0 - 2.0);
}

int percentToSlider(double percent)
{
    const double clamped = std::clamp(percent, 0.01, 10.0);
    return static_cast<int>(std::lround((std::log10(clamped) + 2.0) * 100.0));
}

} // namespace

Section1CurvePanel::Section1CurvePanel(
    QPlainTextEdit *editor, std::function<QJsonObject()> loadSplines,
    std::function<void(const QJsonObject &)> storeSplines, QWidget *parent)
    : QWidget(parent), editor_(editor), loadSplines_(std::move(loadSplines)),
      storeSplines_(std::move(storeSplines))
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    curves_ = new CurveEditor(this);
    curves_->setXAxisLabel(QStringLiteral("Rib"));
    layout->addWidget(curves_, 1);

    description_ = new QLabel(this);
    description_->setObjectName(QStringLiteral("hint"));
    description_->setWordWrap(true);
    layout->addWidget(description_);

    auto *statusRow = new QHBoxLayout;
    statusRow->setSpacing(6);
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    statusRow->addWidget(status_, 1);
    refitButton_ = new QPushButton(QStringLiteral("Re-fit spline"), this);
    refitButton_->setObjectName(QStringLiteral("quietButton"));
    refitButton_->setToolTip(QStringLiteral(
        "Fit new B-splines to the edited matrix values"));
    statusRow->addWidget(refitButton_);
    applyButton_ = new QPushButton(QStringLiteral("Apply spline"), this);
    applyButton_->setObjectName(QStringLiteral("quietButton"));
    applyButton_->setToolTip(QStringLiteral(
        "Overwrite the edited matrix values with the stored B-spline"));
    statusRow->addWidget(applyButton_);
    convertButton_ =
        new QPushButton(QStringLiteral("Convert to B-splines…"), this);
    convertButton_->setObjectName(QStringLiteral("quietButton"));
    convertButton_->setToolTip(QStringLiteral(
        "Fit editable curves with B-splines; the spline becomes the "
        "source of truth and the matrix its sampled output"));
    statusRow->addWidget(convertButton_);
    removeButton_ = new QPushButton(QStringLiteral("Remove spline"), this);
    removeButton_->setObjectName(QStringLiteral("quietButton"));
    removeButton_->setToolTip(QStringLiteral(
        "Return the selected curve to plain point-by-point editing"));
    statusRow->addWidget(removeButton_);
    layout->addLayout(statusRow);

    connect(curves_, &CurveEditor::selectionChanged, this,
            [this](const QString &seriesId) {
                updateDescription(seriesId);
                updateToolbar();
            });
    connect(curves_, &CurveEditor::editCommitted, this,
            [this](const QString &seriesId) { commitSeries(seriesId); });
    connect(curves_, &CurveEditor::pointMoved, this,
            [this](const QString &seriesId, int index, double value) {
                // Morph the spline curve live while a control point drags.
                const auto spline = splines_.find(seriesId.toStdString());
                if (previewActive_ || spline == splines_.end()
                    || index < 0
                    || index >= static_cast<int>(
                           spline->second.control.size())
                    || matrix_.rows.empty())
                    return;
                lep::BSpline moved = spline->second;
                moved.control[index] = value;
                const double firstRib = matrix_.rows.front().values[0];
                const double span =
                    matrix_.rows.back().values[0] - firstRib;
                QVector<QPointF> smooth;
                const int sampleCount = 150;
                smooth.reserve(sampleCount);
                for (int i = 0; i < sampleCount; ++i) {
                    const double u =
                        static_cast<double>(i) / (sampleCount - 1);
                    smooth.append(QPointF(firstRib + u * span,
                                          moved.evaluate(u)));
                }
                curves_->setSelectedSeriesSmooth(smooth);
            });
    connect(convertButton_, &QPushButton::clicked, this,
            [this] { showConvertDialog(); });
    connect(removeButton_, &QPushButton::clicked, this,
            [this] { removeSelectedSpline(); });
    connect(refitButton_, &QPushButton::clicked, this,
            [this] { refitStaleColumns(); });
    connect(applyButton_, &QPushButton::clicked, this,
            [this] { applyStaleSplines(); });
    connect(editor_, &QPlainTextEdit::textChanged, this, [this] {
        if (!applyingEdit_)
            syncFromText();
    });
    connect(curves_, &CurveEditor::undoRequested, this, [this] {
        if (const UndoState *state = undo_.undo())
            restoreState(*state);
    });
    connect(curves_, &CurveEditor::redoRequested, this, [this] {
        if (const UndoState *state = undo_.redo())
            restoreState(*state);
    });

    loadSplinesFromDocument();
    updateDescription(QString());
    syncFromText();
}

Section1CurvePanel::UndoState Section1CurvePanel::captureState() const
{
    return {editor_->toPlainText(), loadSplines_ ? loadSplines_()
                                                 : QJsonObject()};
}

void Section1CurvePanel::pushUndo(UndoState before)
{
    UndoState after = captureState();
    if (before.text == after.text && before.splines == after.splines)
        return; // the operation turned out to be a no-op
    undo_.push(std::move(before), std::move(after));
}

void Section1CurvePanel::restoreState(const UndoState &state)
{
    if (storeSplines_)
        storeSplines_(state.splines);
    loadSplinesFromDocument();
    if (editor_->toPlainText() != state.text) {
        applyingEdit_ = true;
        QTextCursor cursor(editor_->document());
        cursor.select(QTextCursor::Document);
        cursor.insertText(state.text);
        applyingEdit_ = false;
    }
    syncFromText();
}

void Section1CurvePanel::loadSplinesFromDocument()
{
    splines_.clear();
    const QJsonObject root = loadSplines_ ? loadSplines_() : QJsonObject();
    const QJsonObject section =
        root.value(QStringLiteral("section1")).toObject();
    tolerancePercent_ =
        section.value(QStringLiteral("tolerancePercent")).toDouble(1.0);
    const QJsonObject columns =
        section.value(QStringLiteral("columns")).toObject();
    for (auto it = columns.constBegin(); it != columns.constEnd(); ++it) {
        const lep::BSpline spline = splineFromJson(it.value().toObject());
        if (spline.isValid())
            splines_[it.key().toStdString()] = spline;
    }
}

void Section1CurvePanel::persistSplines()
{
    if (!storeSplines_)
        return;
    QJsonObject root = loadSplines_ ? loadSplines_() : QJsonObject();
    if (splines_.empty()) {
        root.remove(QStringLiteral("section1"));
    } else {
        QJsonObject columns;
        for (const auto &[columnId, spline] : splines_)
            columns.insert(QString::fromStdString(columnId),
                           splineToJson(spline));
        QJsonObject section;
        section.insert(QStringLiteral("version"), 1);
        section.insert(QStringLiteral("tolerancePercent"), tolerancePercent_);
        section.insert(QStringLiteral("columns"), columns);
        root.insert(QStringLiteral("section1"), section);
    }
    storeSplines_(root);
}

int Section1CurvePanel::columnIndexById(const std::string &id) const
{
    const auto &columns = lep::section1Columns();
    for (size_t c = 0; c < columns.size(); ++c) {
        if (id == columns[c].id)
            return static_cast<int>(c);
    }
    return -1;
}

std::vector<double> Section1CurvePanel::columnValues(int column) const
{
    std::vector<double> values;
    values.reserve(matrix_.rows.size());
    for (const lep::Section1Row &row : matrix_.rows)
        values.push_back(column < static_cast<int>(row.values.size())
                             ? row.values[column]
                             : 0.0);
    return values;
}

double Section1CurvePanel::absoluteTolerance(int column,
                                             double percent) const
{
    const std::vector<double> values = columnValues(column);
    double low = values.empty() ? 0.0 : values.front();
    double high = low;
    for (const double value : values) {
        low = std::min(low, value);
        high = std::max(high, value);
    }
    const int decimals = lep::section1Columns()[column].decimals;
    // Never demand more precision than the text format can express: half of
    // the last printed decimal is pure rounding noise.
    const double floor = 0.5 * std::pow(10.0, -decimals);
    return std::max(percent / 100.0 * (high - low), floor);
}

const std::map<std::string, lep::BSpline> &
Section1CurvePanel::activeSplines() const
{
    return previewActive_ ? previewSplines_ : splines_;
}

void Section1CurvePanel::syncFromText()
{
    std::vector<std::string> problems;
    matrixUsable_ = lep::parseSection1Matrix(
        editor_->toPlainText().toStdString(), &matrix_, &problems);
    matrixProblems_.clear();
    for (const std::string &problem : problems)
        matrixProblems_ << QString::fromStdString(problem);

    // Staleness: a stored spline whose sampled values no longer match the
    // matrix beyond the text's own rounding was overtaken by manual edits.
    staleColumns_.clear();
    if (matrixUsable_) {
        const std::vector<double> stations =
            lep::uniformParameters(static_cast<int>(matrix_.rows.size()));
        for (const auto &[columnId, spline] : splines_) {
            const int column = columnIndexById(columnId);
            if (column < 1 || column >= matrix_.columnCount)
                continue;
            const std::vector<double> sampled =
                lep::sampleBSpline(spline, stations);
            const std::vector<double> current = columnValues(column);
            const double roundingSlack =
                0.75
                * std::pow(10.0,
                           -lep::section1Columns()[column].decimals);
            for (size_t i = 0; i < current.size(); ++i) {
                if (std::abs(sampled[i] - current[i]) > roundingSlack) {
                    staleColumns_.push_back(columnId);
                    break;
                }
            }
        }
    }
    rebuildDisplay();
}

void Section1CurvePanel::rebuildDisplay()
{
    if (!matrixUsable_) {
        curves_->setMessage(matrixProblems_.join(QLatin1Char('\n')));
        status_->setStyleSheet(QStringLiteral("color: %1;").arg(kErrorColor));
        status_->setText(
            QStringLiteral("The rib matrix cannot be read — fix the text "
                           "above to edit it graphically."));
        updateToolbar();
        return;
    }

    const auto &columns = lep::section1Columns();
    const auto &active = activeSplines();
    const int rowCount = static_cast<int>(matrix_.rows.size());
    const double firstRib = matrix_.rows.front().values[0];
    const double lastRib = matrix_.rows.back().values[0];
    const double span = lastRib - firstRib;

    QVector<CurveSeries> series;
    const int columnCount =
        std::min<int>(matrix_.columnCount, static_cast<int>(columns.size()));
    for (int c = 1; c < columnCount; ++c) {
        const lep::Section1Column &column = columns[c];
        CurveSeries s;
        s.id = QLatin1String(column.id);
        s.label = QLatin1String(column.label);
        s.unit = QLatin1String(column.unit);
        s.description = QLatin1String(column.description);
        s.editable = column.editable;
        s.minValue = column.minValue;
        s.maxValue = column.maxValue;
        s.decimals = column.decimals;
        styleForColumn(c, &s.color, &s.penStyle);
        s.points.reserve(rowCount);
        for (const lep::Section1Row &row : matrix_.rows)
            s.points.append(QPointF(row.values[0], row.values[c]));

        const auto spline = active.find(column.id);
        if (spline != active.end() && spline->second.isValid()) {
            const std::vector<double> greville =
                spline->second.grevilleAbscissae();
            s.handles.reserve(static_cast<int>(greville.size()));
            for (size_t i = 0; i < greville.size(); ++i)
                s.handles.append(
                    QPointF(firstRib + greville[i] * span,
                            spline->second.control[i]));
            const int sampleCount = 150;
            s.smooth.reserve(sampleCount);
            for (int i = 0; i < sampleCount; ++i) {
                const double u =
                    static_cast<double>(i) / (sampleCount - 1);
                s.smooth.append(QPointF(firstRib + u * span,
                                        spline->second.evaluate(u)));
            }
        }
        series.append(s);
    }
    curves_->setMessage(QString());
    curves_->setSeriesList(series);
    if (curves_->selectedSeriesId().isEmpty() && !series.isEmpty())
        curves_->setSelectedSeriesId(series.first().id);

    QStringList notes = matrixProblems_;
    for (const std::string &columnId : staleColumns_) {
        const int column = columnIndexById(columnId);
        notes << QStringLiteral(
                     "The %1 values no longer match their B-spline — re-fit "
                     "the spline or apply it to restore the matrix.")
                     .arg(QLatin1String(
                         lep::section1Columns()[column].label));
    }
    if (previewActive_) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kWarningColor));
        status_->setText(QStringLiteral(
            "Previewing B-spline conversion — Convert applies it, Cancel "
            "restores the current curves."));
    } else if (!notes.isEmpty()) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kWarningColor));
        status_->setText(QStringLiteral("⚠ ")
                         + notes.join(QStringLiteral("\n⚠ ")));
    } else {
        status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
        const int splineCount = static_cast<int>(splines_.size());
        QString text =
            QStringLiteral("Geometry matrix OK · %1 rib rows × %2 columns")
                .arg(matrix_.rows.size())
                .arg(matrix_.columnCount);
        if (splineCount > 0) {
            text += QStringLiteral(" · %1 curve%2 B-spline backed")
                        .arg(splineCount)
                        .arg(splineCount == 1 ? QString()
                                              : QStringLiteral("s"));
        }
        text += QStringLiteral(" · drag points to edit · ↑/↓ nudges "
                               "(Shift = ×10) · Ctrl+Z here undoes the "
                               "last curve edit");
        status_->setText(text);
    }
    updateToolbar();
    updateDescription(curves_->selectedSeriesId());
}

void Section1CurvePanel::updateToolbar()
{
    const bool stale = !staleColumns_.empty() && !previewActive_;
    refitButton_->setVisible(stale);
    applyButton_->setVisible(stale);
    convertButton_->setEnabled(matrixUsable_ && !previewActive_);
    const std::string selected =
        curves_->selectedSeriesId().toStdString();
    removeButton_->setVisible(!previewActive_
                              && splines_.count(selected) > 0);
}

void Section1CurvePanel::commitSeries(const QString &seriesId)
{
    const int column = columnIndexById(seriesId.toStdString());
    if (column < 1 || !matrixUsable_ || previewActive_)
        return;
    UndoState before = captureState();
    const CurveSeries *series = nullptr;
    for (const CurveSeries &candidate : curves_->seriesList()) {
        if (candidate.id == seriesId) {
            series = &candidate;
            break;
        }
    }
    if (!series)
        return;

    const auto spline = splines_.find(seriesId.toStdString());
    if (spline != splines_.end()) {
        // Spline truth: handles are the control values; the matrix becomes
        // the spline sampled at the current stations.
        if (series->handles.size()
            != static_cast<int>(spline->second.control.size()))
            return;
        for (int i = 0; i < series->handles.size(); ++i)
            spline->second.control[i] = series->handles.at(i).y();
        const std::vector<double> sampled = lep::sampleBSpline(
            spline->second,
            lep::uniformParameters(static_cast<int>(matrix_.rows.size())));
        patchRows({{column, sampled}}, false);
        persistSplines();
        pushUndo(std::move(before));
        return;
    }

    if (series->points.size() != static_cast<int>(matrix_.rows.size()))
        return;
    std::vector<double> values;
    values.reserve(series->points.size());
    for (const QPointF &point : series->points)
        values.push_back(point.y());
    patchRows({{column, values}}, true);
    pushUndo(std::move(before));
}

void Section1CurvePanel::patchRows(
    const std::map<int, std::vector<double>> &valuesByColumn,
    bool skipUnchangedValues)
{
    QTextDocument *document = editor_->document();
    applyingEdit_ = true;
    QTextCursor cursor(document);
    cursor.beginEditBlock();
    for (size_t i = 0; i < matrix_.rows.size(); ++i) {
        lep::Section1Row &row = matrix_.rows[i];
        bool rowTouched = false;
        for (const auto &[column, values] : valuesByColumn) {
            if (column >= static_cast<int>(row.values.size())
                || i >= values.size())
                continue;
            if (skipUnchangedValues && values[i] == row.values[column])
                continue;
            row.values[column] = values[i];
            rowTouched = true;
        }
        if (!rowTouched)
            continue;
        const QString newLine =
            QString::fromStdString(lep::formatSection1Row(row));
        const QTextBlock block = document->findBlockByNumber(row.lineIndex);
        if (!block.isValid() || block.text() == newLine)
            continue;
        cursor.setPosition(block.position());
        cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
        cursor.insertText(newLine);
    }
    cursor.endEditBlock();
    applyingEdit_ = false;
    // Re-parse once so curves and staleness reflect the rounded text.
    syncFromText();
}

void Section1CurvePanel::showConvertDialog()
{
    if (!matrixUsable_)
        return;
    const auto &columns = lep::section1Columns();
    const int columnCount =
        std::min<int>(matrix_.columnCount, static_cast<int>(columns.size()));

    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("Convert curves to B-splines"));
    auto *layout = new QVBoxLayout(&dialog);
    auto *intro = new QLabel(
        QStringLiteral(
            "Enabled curves are fitted with a B-spline within the chosen "
            "tolerance and become spline-edited; the rib matrix turns into "
            "the spline sampled at the current rib stations. Disabled "
            "curves stay point-by-point — convert them later with a "
            "different tolerance if needed."),
        &dialog);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    // One checkable button per editable column.
    auto *chipRow = new QHBoxLayout;
    chipRow->setSpacing(6);
    std::vector<std::pair<int, QPushButton *>> toggles;
    const bool anyUnconverted = [&] {
        for (int c = 1; c < columnCount; ++c) {
            if (columns[c].editable && splines_.count(columns[c].id) == 0)
                return true;
        }
        return false;
    }();
    for (int c = 1; c < columnCount; ++c) {
        if (!columns[c].editable)
            continue;
        auto *toggle =
            new QPushButton(QLatin1String(columns[c].label), &dialog);
        toggle->setCheckable(true);
        QColor color;
        Qt::PenStyle penStyle = Qt::SolidLine;
        styleForColumn(c, &color, &penStyle);
        toggle->setStyleSheet(
            QStringLiteral("QPushButton { border: 1px solid #26354a; "
                           "border-radius: 5px; padding: 4px 10px; }"
                           "QPushButton:checked { border-color: %1; "
                           "background: #1f5571; }")
                .arg(color.name()));
        // Default: convert what is not yet spline backed; if everything is,
        // offer a full re-fit.
        toggle->setChecked(!anyUnconverted
                           || splines_.count(columns[c].id) == 0);
        chipRow->addWidget(toggle);
        toggles.emplace_back(c, toggle);
    }
    chipRow->addStretch();
    layout->addLayout(chipRow);

    auto *sliderRow = new QHBoxLayout;
    sliderRow->addWidget(new QLabel(QStringLiteral("Tolerance"), &dialog));
    auto *slider = new QSlider(Qt::Horizontal, &dialog);
    slider->setRange(0, 300);
    slider->setValue(percentToSlider(tolerancePercent_));
    sliderRow->addWidget(slider, 1);
    auto *sliderLabel = new QLabel(&dialog);
    sliderLabel->setMinimumWidth(150);
    sliderRow->addWidget(sliderLabel);
    layout->addLayout(sliderRow);

    auto *summary = new QLabel(&dialog);
    summary->setWordWrap(true);
    summary->setObjectName(QStringLiteral("hint"));
    layout->addWidget(summary);

    auto *buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    buttons->button(QDialogButtonBox::Ok)
        ->setText(QStringLiteral("Convert"));
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);

    const std::vector<double> stations =
        lep::uniformParameters(static_cast<int>(matrix_.rows.size()));
    const auto updatePreview = [&] {
        const double percent = sliderToPercent(slider->value());
        sliderLabel->setText(
            QStringLiteral("%1 % of curve range")
                .arg(QString::number(percent, 'f', 2)));
        previewSplines_ = splines_;
        int converted = 0;
        int worstControls = 0;
        double worstPercent = 0.0;
        for (const auto &[column, toggle] : toggles) {
            if (!toggle->isChecked()) {
                toggle->setToolTip(QString());
                continue;
            }
            const std::vector<double> values = columnValues(column);
            double maxError = 0.0;
            const lep::BSpline fitted =
                lep::fitBSpline(stations, values,
                                absoluteTolerance(column, percent),
                                &maxError);
            if (!fitted.isValid())
                continue;
            previewSplines_[lep::section1Columns()[column].id] = fitted;
            ++converted;
            worstControls = std::max(
                worstControls, static_cast<int>(fitted.control.size()));
            double low = values.front();
            double high = low;
            for (const double value : values) {
                low = std::min(low, value);
                high = std::max(high, value);
            }
            const double range = std::max(high - low, 1e-12);
            worstPercent =
                std::max(worstPercent, maxError / range * 100.0);
            toggle->setToolTip(
                QStringLiteral("%1 control points · max deviation %2 %3 "
                               "(%4 %)")
                    .arg(fitted.control.size())
                    .arg(QString::number(maxError, 'f', 3))
                    .arg(QLatin1String(
                        lep::section1Columns()[column].unit))
                    .arg(QString::number(maxError / range * 100.0, 'f',
                                         2)));
        }
        summary->setText(
            converted == 0
                ? QStringLiteral("No curves enabled.")
                : QStringLiteral("%1 curve%2 fitted · up to %3 control "
                                 "points · worst deviation %4 % of range")
                      .arg(converted)
                      .arg(converted == 1 ? QString() : QStringLiteral("s"))
                      .arg(worstControls)
                      .arg(QString::number(worstPercent, 'f', 2)));
        buttons->button(QDialogButtonBox::Ok)->setEnabled(converted > 0);
        previewActive_ = true;
        rebuildDisplay();
    };
    connect(slider, &QSlider::valueChanged, &dialog,
            [&updatePreview](int) { updatePreview(); });
    for (const auto &[column, toggle] : toggles) {
        connect(toggle, &QPushButton::toggled, &dialog,
                [&updatePreview](bool) { updatePreview(); });
    }
    updatePreview();

    const int result = dialog.exec();
    previewActive_ = false;
    if (result != QDialog::Accepted) {
        previewSplines_.clear();
        rebuildDisplay();
        return;
    }

    UndoState before = captureState();
    tolerancePercent_ = sliderToPercent(slider->value());
    std::map<int, std::vector<double>> valuesByColumn;
    for (const auto &[column, toggle] : toggles) {
        if (!toggle->isChecked())
            continue;
        const auto fitted =
            previewSplines_.find(lep::section1Columns()[column].id);
        if (fitted == previewSplines_.end())
            continue;
        splines_[fitted->first] = fitted->second;
        valuesByColumn[column] =
            lep::sampleBSpline(fitted->second, stations);
    }
    previewSplines_.clear();
    persistSplines();
    patchRows(valuesByColumn, false);
    pushUndo(std::move(before));
}

void Section1CurvePanel::removeSelectedSpline()
{
    const std::string selected = curves_->selectedSeriesId().toStdString();
    if (splines_.count(selected) == 0)
        return;
    UndoState before = captureState();
    splines_.erase(selected);
    persistSplines();
    syncFromText();
    pushUndo(std::move(before));
}

void Section1CurvePanel::refitStaleColumns()
{
    if (!matrixUsable_)
        return;
    UndoState before = captureState();
    const std::vector<double> stations =
        lep::uniformParameters(static_cast<int>(matrix_.rows.size()));
    for (const std::string &columnId : staleColumns_) {
        const int column = columnIndexById(columnId);
        if (column < 1)
            continue;
        double maxError = 0.0;
        const lep::BSpline fitted = lep::fitBSpline(
            stations, columnValues(column),
            absoluteTolerance(column, tolerancePercent_), &maxError);
        if (fitted.isValid())
            splines_[columnId] = fitted;
    }
    persistSplines();
    syncFromText();
    pushUndo(std::move(before));
}

void Section1CurvePanel::applyStaleSplines()
{
    if (!matrixUsable_)
        return;
    UndoState before = captureState();
    const std::vector<double> stations =
        lep::uniformParameters(static_cast<int>(matrix_.rows.size()));
    std::map<int, std::vector<double>> valuesByColumn;
    for (const std::string &columnId : staleColumns_) {
        const int column = columnIndexById(columnId);
        const auto spline = splines_.find(columnId);
        if (column < 1 || spline == splines_.end())
            continue;
        valuesByColumn[column] = lep::sampleBSpline(spline->second, stations);
    }
    patchRows(valuesByColumn, false);
    pushUndo(std::move(before));
}

void Section1CurvePanel::updateDescription(const QString &seriesId)
{
    for (const lep::Section1Column &column : lep::section1Columns()) {
        if (seriesId == QLatin1String(column.id)) {
            const QString unit =
                column.unit[0] == '\0'
                    ? QString()
                    : QStringLiteral(" (%1)").arg(QLatin1String(column.unit));
            QString text = QStringLiteral("<b>%1</b>%2 — %3")
                               .arg(QLatin1String(column.label), unit,
                                    QLatin1String(column.description));
            const auto spline = activeSplines().find(column.id);
            if (spline != activeSplines().end()) {
                text += QStringLiteral(
                            " · <b>B-spline</b> with %1 control points — "
                            "drag the squares; the dots are the sampled "
                            "matrix rows.")
                            .arg(spline->second.control.size());
            }
            description_->setText(text);
            return;
        }
    }
    description_->setText(
        QStringLiteral("Each curve is one column of the rib matrix, drawn "
                       "over the rib number. Click a curve or its name to "
                       "see what it does and to edit it."));
}
