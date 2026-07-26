#include "section1_curve_panel.h"

#include "curve_editor.h"

#include <QLabel>
#include <QPlainTextEdit>
#include <QStringList>
#include <QTextBlock>
#include <QTextCursor>
#include <QVBoxLayout>

#include <algorithm>

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

} // namespace

Section1CurvePanel::Section1CurvePanel(QPlainTextEdit *editor, QWidget *parent)
    : QWidget(parent), editor_(editor)
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

    updateDescription(QString());
    syncFromText();
}

void Section1CurvePanel::syncFromText()
{
    std::vector<std::string> problems;
    const bool usable = lep::parseSection1Matrix(
        editor_->toPlainText().toStdString(), &matrix_, &problems);

    QStringList problemList;
    for (const std::string &problem : problems)
        problemList << QString::fromStdString(problem);

    if (!usable) {
        curves_->setMessage(problemList.join(QLatin1Char('\n')));
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kErrorColor));
        status_->setText(
            QStringLiteral("The rib matrix cannot be read — fix the text "
                           "above to edit it graphically."));
        return;
    }

    const auto &columns = lep::section1Columns();
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
        s.points.reserve(static_cast<int>(matrix_.rows.size()));
        for (const lep::Section1Row &row : matrix_.rows)
            s.points.append(QPointF(row.values[0], row.values[c]));
        series.append(s);
    }
    curves_->setMessage(QString());
    curves_->setSeriesList(series);
    if (curves_->selectedSeriesId().isEmpty() && !series.isEmpty())
        curves_->setSelectedSeriesId(series.first().id);

    if (!problemList.isEmpty()) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kWarningColor));
        status_->setText(QStringLiteral("⚠ ")
                         + problemList.join(QStringLiteral("\n⚠ ")));
    } else {
        status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
        status_->setText(
            QStringLiteral("Geometry matrix OK · %1 rib rows × %2 columns · "
                           "drag points to edit · ↑/↓ nudges the highlighted "
                           "point (Shift = ×10)")
                .arg(matrix_.rows.size())
                .arg(matrix_.columnCount));
    }
}

void Section1CurvePanel::commitSeries(const QString &seriesId)
{
    const auto &columns = lep::section1Columns();
    int column = -1;
    for (size_t c = 0; c < columns.size(); ++c) {
        if (seriesId == QLatin1String(columns[c].id)) {
            column = static_cast<int>(c);
            break;
        }
    }
    if (column < 1)
        return;

    const CurveSeries *series = nullptr;
    for (const CurveSeries &candidate : curves_->seriesList()) {
        if (candidate.id == seriesId) {
            series = &candidate;
            break;
        }
    }
    if (!series
        || series->points.size() != static_cast<int>(matrix_.rows.size()))
        return;

    QTextDocument *document = editor_->document();
    applyingEdit_ = true;
    QTextCursor cursor(document);
    cursor.beginEditBlock();
    for (size_t i = 0; i < matrix_.rows.size(); ++i) {
        lep::Section1Row &row = matrix_.rows[i];
        if (column >= static_cast<int>(row.values.size()))
            continue;
        const double value = series->points.at(static_cast<int>(i)).y();
        // Untouched points round-trip bit-identically from the parse; only
        // genuinely edited rows get rewritten (and thereby reformatted), so
        // a drag never rewrites the whole matrix.
        if (value == row.values[column])
            continue;
        row.values[column] = value;
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
    // Re-parse once so the curves show the rounded values now in the text.
    syncFromText();
}

void Section1CurvePanel::updateDescription(const QString &seriesId)
{
    for (const lep::Section1Column &column : lep::section1Columns()) {
        if (seriesId == QLatin1String(column.id)) {
            const QString unit =
                column.unit[0] == '\0'
                    ? QString()
                    : QStringLiteral(" (%1)").arg(QLatin1String(column.unit));
            description_->setText(QStringLiteral("<b>%1</b>%2 — %3")
                                      .arg(QLatin1String(column.label), unit,
                                           QLatin1String(column.description)));
            return;
        }
    }
    description_->setText(
        QStringLiteral("Each curve is one column of the rib matrix, drawn "
                       "over the rib number. Click a curve or its name to "
                       "see what it does and to edit it."));
}
