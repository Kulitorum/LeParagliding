#include "grid_curve_panel.h"

#include "curve_editor.h"
#include "section_specs.h"

#include <QLabel>
#include <QPlainTextEdit>
#include <QTextBlock>
#include <QTextCursor>
#include <QVBoxLayout>

#include <cmath>
#include <cstdlib>

namespace {

// Same validated dark-surface palette as the Section 1 panel.
const QColor kSeriesPalette[8] = {
    QColor(0x39, 0x87, 0xe5), QColor(0x00, 0x83, 0x00),
    QColor(0xd5, 0x51, 0x81), QColor(0xc9, 0x85, 0x00),
    QColor(0x19, 0x9e, 0x70), QColor(0xd9, 0x59, 0x26),
    QColor(0x90, 0x85, 0xe9), QColor(0xe6, 0x67, 0x67),
};

const QString kOkColor = QStringLiteral("#93a4ba");
const QString kWarningColor = QStringLiteral("#fab219");

bool parseNumber(const std::string &token, double *value)
{
    char *end = nullptr;
    *value = std::strtod(token.c_str(), &end);
    return end && *end == '\0' && end != token.c_str();
}

} // namespace

GridCurvePanel::GridCurvePanel(int sectionNumber, QPlainTextEdit *editor,
                               QWidget *parent)
    : QWidget(parent), sectionNumber_(sectionNumber), editor_(editor)
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

    status_ = new QLabel(this);
    status_->setWordWrap(true);
    layout->addWidget(status_);

    connect(curves_, &CurveEditor::selectionChanged, this,
            [this](const QString &seriesId) { updateDescription(seriesId); });
    connect(curves_, &CurveEditor::editCommitted, this,
            [this](const QString &seriesId) { commitSeries(seriesId); });
    connect(editor_, &QPlainTextEdit::textChanged, this, [this] {
        if (!applyingEdit_)
            syncFromText();
    });
    connect(curves_, &CurveEditor::undoRequested, this, [this] {
        if (const QString *text = undo_.undo())
            restoreText(*text);
    });
    connect(curves_, &CurveEditor::redoRequested, this, [this] {
        if (const QString *text = undo_.redo())
            restoreText(*text);
    });

    updateDescription(QString());
    syncFromText();
}

void GridCurvePanel::restoreText(const QString &text)
{
    if (editor_->toPlainText() == text)
        return;
    applyingEdit_ = true;
    QTextCursor cursor(editor_->document());
    cursor.select(QTextCursor::Document);
    cursor.insertText(text);
    applyingEdit_ = false;
    syncFromText();
}

void GridCurvePanel::syncFromText()
{
    const SectionSpec *spec = sectionSpec(sectionNumber_);
    grid_ = lep::parseSectionGrid(editor_->toPlainText().toStdString());
    uniformRows_.clear();
    if (spec == nullptr || spec->curveColumns.isEmpty()) {
        curves_->setMessage(QStringLiteral("No curve columns defined."));
        return;
    }

    // The dominant table: rows carrying at least every curve column, all
    // of them numeric. Header/flag records have fewer or non-numeric
    // tokens and stay out of the curves.
    int needed = 0;
    for (const SectionCurveColumn &curveColumn : spec->curveColumns)
        needed = std::max(needed, curveColumn.column + 1);
    for (int r = 0; r < static_cast<int>(grid_.rows.size()); ++r) {
        const lep::GridRow &row = grid_.rows[static_cast<size_t>(r)];
        if (static_cast<int>(row.tokens.size()) < needed)
            continue;
        bool numeric = true;
        for (const SectionCurveColumn &curveColumn : spec->curveColumns) {
            double value = 0.0;
            if (!parseNumber(
                    row.tokens[static_cast<size_t>(curveColumn.column)]
                        .text,
                    &value)) {
                numeric = false;
                break;
            }
        }
        if (numeric)
            uniformRows_.append(r);
    }

    if (uniformRows_.size() < 2) {
        curves_->setMessage(QStringLiteral(
            "Not enough uniform data rows for curves — check the section "
            "structure in the Values tab or the text above."));
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kWarningColor));
        status_->setText(QStringLiteral(
            "The curve view needs at least two complete data rows."));
        return;
    }

    QVector<CurveSeries> series;
    for (int index = 0; index < spec->curveColumns.size(); ++index) {
        const SectionCurveColumn &curveColumn = spec->curveColumns.at(index);
        CurveSeries s;

        // Gates: grey out curves the engine ignores in the current data.
        if (curveColumn.gateColumn >= 0 || curveColumn.gateOwnNonZero) {
            bool used = false;
            for (const int r : uniformRows_) {
                const lep::GridRow &row =
                    grid_.rows[static_cast<size_t>(r)];
                double value = 0.0;
                if (curveColumn.gateColumn >= 0) {
                    if (curveColumn.gateColumn
                            < static_cast<int>(row.tokens.size())
                        && parseNumber(
                            row.tokens[static_cast<size_t>(
                                           curveColumn.gateColumn)]
                                .text,
                            &value)
                        && value >= curveColumn.gateThreshold) {
                        used = true;
                        break;
                    }
                } else if (parseNumber(
                               row.tokens[static_cast<size_t>(
                                              curveColumn.column)]
                                   .text,
                               &value)
                           && value != 0.0) {
                    used = true;
                    break;
                }
            }
            s.enabled = used;
            s.disabledNote = curveColumn.disabledNote;
        }
        s.id = QString::number(curveColumn.column);
        s.label = curveColumn.column < spec->columns.size()
                      ? spec->columns.at(curveColumn.column).header
                      : QStringLiteral("Column %1")
                            .arg(curveColumn.column + 1);
        s.description =
            curveColumn.column < spec->columns.size()
                ? spec->columns.at(curveColumn.column).tooltip
                : QString();
        s.color = kSeriesPalette[index % 8];
        s.penStyle = index < 8 ? Qt::SolidLine : Qt::DashLine;
        s.minValue = curveColumn.minValue;
        s.maxValue = curveColumn.maxValue;
        s.decimals = curveColumn.decimals;
        s.points.reserve(uniformRows_.size());
        for (int r = 0; r < uniformRows_.size(); ++r) {
            const lep::GridRow &row =
                grid_.rows[static_cast<size_t>(uniformRows_.at(r))];
            double value = 0.0;
            parseNumber(
                row.tokens[static_cast<size_t>(curveColumn.column)].text,
                &value);
            s.points.append(QPointF(r + 1, value));
        }
        series.append(s);
    }
    curves_->setMessage(QString());
    curves_->setSeriesList(series);
    if (curves_->selectedSeriesId().isEmpty()) {
        for (const CurveSeries &candidate : series) {
            if (candidate.enabled) {
                curves_->setSelectedSeriesId(candidate.id);
                break;
            }
        }
    }

    int disabledCount = 0;
    for (const CurveSeries &candidate : series) {
        if (!candidate.enabled)
            ++disabledCount;
    }
    status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
    QString text =
        QStringLiteral("%1 data rows · drag points to edit · ↑/↓ nudges "
                       "(Shift = ×10) · the Values tab edits every field")
            .arg(uniformRows_.size());
    if (disabledCount > 0) {
        text += QStringLiteral(" · %1 unused curve%2 greyed out (hover the "
                               "name to see why)")
                    .arg(disabledCount)
                    .arg(disabledCount == 1 ? QString()
                                            : QStringLiteral("s"));
    }
    status_->setText(text);
}

void GridCurvePanel::commitSeries(const QString &seriesId)
{
    const CurveSeries *series = nullptr;
    for (const CurveSeries &candidate : curves_->seriesList()) {
        if (candidate.id == seriesId) {
            series = &candidate;
            break;
        }
    }
    if (!series || series->points.size() != uniformRows_.size())
        return;
    const int column = seriesId.toInt();
    const QString before = editor_->toPlainText();

    QTextDocument *document = editor_->document();
    applyingEdit_ = true;
    QTextCursor cursor(document);
    cursor.beginEditBlock();
    for (int r = 0; r < uniformRows_.size(); ++r) {
        const lep::GridRow &row =
            grid_.rows[static_cast<size_t>(uniformRows_.at(r))];
        const lep::GridToken &token =
            row.tokens[static_cast<size_t>(column)];
        double oldValue = 0.0;
        parseNumber(token.text, &oldValue);
        const double newValue = series->points.at(r).y();
        if (newValue == oldValue)
            continue; // untouched points round-trip bit-identically
        const QTextBlock block = document->findBlockByNumber(row.lineIndex);
        if (!block.isValid())
            continue;
        const QString newText =
            QString::number(newValue, 'f', series->decimals);
        cursor.setPosition(block.position());
        cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
        cursor.insertText(QString::fromStdString(lep::lineWithReplacedToken(
            block.text().toStdString(), token, newText.toStdString())));
    }
    cursor.endEditBlock();
    applyingEdit_ = false;
    syncFromText();
    const QString after = editor_->toPlainText();
    if (after != before)
        undo_.push(before, after);
}

void GridCurvePanel::updateDescription(const QString &seriesId)
{
    for (const CurveSeries &series : curves_->seriesList()) {
        if (series.id == seriesId && !series.description.isEmpty()) {
            description_->setText(QStringLiteral("<b>%1</b> — %2")
                                      .arg(series.label, series.description));
            return;
        }
    }
    description_->setText(QStringLiteral(
        "Each curve is one column of this section's table, drawn over the "
        "rib order. Click a curve or its name to select and edit it."));
}
