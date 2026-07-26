#include "section_grid_panel.h"

#include "section_specs.h"

#include <QColor>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextBlock>
#include <QTextCursor>
#include <QVBoxLayout>

#include <algorithm>

namespace {

const QString kOkColor = QStringLiteral("#93a4ba");
const QString kWarningColor = QStringLiteral("#fab219");

bool containsWhitespace(const QString &text)
{
    for (const QChar ch : text) {
        if (ch.isSpace())
            return true;
    }
    return false;
}

} // namespace

SectionGridPanel::SectionGridPanel(int sectionNumber, QPlainTextEdit *editor,
                                   std::function<int()> expectedHalfRibRows,
                                   QWidget *parent)
    : QWidget(parent), sectionNumber_(sectionNumber), editor_(editor),
      expectedHalfRibRows_(std::move(expectedHalfRibRows))
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    table_ = new QTableWidget(this);
    table_->setEditTriggers(QAbstractItemView::DoubleClicked
                            | QAbstractItemView::EditKeyPressed
                            | QAbstractItemView::AnyKeyPressed);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->verticalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    table_->setAlternatingRowColors(true);
    layout->addWidget(table_, 1);

    hint_ = new QLabel(this);
    hint_->setObjectName(QStringLiteral("hint"));
    hint_->setWordWrap(true);
    if (const SectionSpec *spec = sectionSpec(sectionNumber_)) {
        hint_->setText(spec->structure);
        hint_->setVisible(!spec->structure.isEmpty());
    } else {
        hint_->setVisible(false);
    }
    layout->addWidget(hint_);

    status_ = new QLabel(this);
    status_->setWordWrap(true);
    layout->addWidget(status_);

    connect(table_, &QTableWidget::cellChanged, this,
            [this](int row, int column) {
                if (!rebuildingTable_)
                    applyCellEdit(row, column);
            });
    connect(editor_, &QPlainTextEdit::textChanged, this, [this] {
        if (!applyingEdit_)
            syncFromText();
    });

    syncFromText();
}

void SectionGridPanel::syncFromText()
{
    grid_ = lep::parseSectionGrid(editor_->toPlainText().toStdString());
    const SectionSpec *spec = sectionSpec(sectionNumber_);

    rebuildingTable_ = true;
    const int previousRow = table_->currentRow();
    const int previousColumn = table_->currentColumn();

    const int columnCount = static_cast<int>(
        std::max<size_t>(grid_.maxColumns,
                         spec ? static_cast<size_t>(spec->columns.size())
                              : 0));
    table_->clear();
    table_->setColumnCount(columnCount);
    table_->setRowCount(static_cast<int>(grid_.rows.size()));

    for (int c = 0; c < columnCount; ++c) {
        auto *header = new QTableWidgetItem(
            spec && c < spec->columns.size()
                ? spec->columns.at(c).header
                : QString::number(c + 1));
        if (spec && c < spec->columns.size())
            header->setToolTip(spec->columns.at(c).tooltip);
        table_->setHorizontalHeaderItem(c, header);
    }

    QStringList rowLabels;
    for (size_t r = 0; r < grid_.rows.size(); ++r) {
        const lep::GridRow &row = grid_.rows[r];
        QString label = QString::fromStdString(row.comment);
        if (label.size() > 30)
            label = label.left(28) + QStringLiteral("…");
        rowLabels << label;
        for (int c = 0; c < columnCount; ++c) {
            auto *item = new QTableWidgetItem;
            if (c < static_cast<int>(row.tokens.size())) {
                item->setText(
                    QString::fromStdString(row.tokens[c].text));
            } else {
                // Ragged hole: nothing to edit at this position.
                item->setFlags(Qt::ItemIsEnabled);
                item->setBackground(QColor(0x0b, 0x12, 0x1f));
            }
            item->setToolTip(QString::fromStdString(row.comment));
            table_->setItem(static_cast<int>(r), c, item);
        }
    }
    table_->setVerticalHeaderLabels(rowLabels);
    if (previousRow >= 0 && previousRow < table_->rowCount()
        && previousColumn >= 0 && previousColumn < table_->columnCount())
        table_->setCurrentCell(previousRow, previousColumn);
    rebuildingTable_ = false;

    // Cross-check one-record-per-rib tables against Section 1's counts.
    if (spec && spec->onePerRib && expectedHalfRibRows_) {
        const int expected = expectedHalfRibRows_();
        if (expected > 1
            && expected != static_cast<int>(grid_.rows.size())) {
            status_->setStyleSheet(
                QStringLiteral("color: %1;").arg(kWarningColor));
            status_->setText(
                QStringLiteral("⚠ Section 1 declares %1 half-wing ribs, "
                               "but this section has %2 record%3 — the "
                               "engine reads exactly one per rib.")
                    .arg(expected)
                    .arg(grid_.rows.size())
                    .arg(grid_.rows.size() == 1 ? QString()
                                                : QStringLiteral("s")));
            return;
        }
    }

    status_->setStyleSheet(QStringLiteral("color: %1;").arg(kOkColor));
    status_->setText(
        grid_.rows.empty()
            ? QStringLiteral("No data records in this section.")
            : QStringLiteral("%1 record%2 · double-click a cell to edit "
                             "the value in place · add or remove records "
                             "in the text above")
                  .arg(grid_.rows.size())
                  .arg(grid_.rows.size() == 1 ? QString()
                                              : QStringLiteral("s")));
}

void SectionGridPanel::applyCellEdit(int row, int column)
{
    if (row < 0 || row >= static_cast<int>(grid_.rows.size()))
        return;
    const lep::GridRow &gridRow = grid_.rows[static_cast<size_t>(row)];
    if (column < 0 || column >= static_cast<int>(gridRow.tokens.size()))
        return;
    const lep::GridToken &token = gridRow.tokens[static_cast<size_t>(column)];
    QTableWidgetItem *item = table_->item(row, column);
    if (item == nullptr)
        return;
    const QString newText = item->text().trimmed();
    const QString oldText = QString::fromStdString(token.text);
    if (newText == oldText)
        return;

    const bool quoted = newText.startsWith(QLatin1Char('"'))
                        && newText.endsWith(QLatin1Char('"'))
                        && newText.size() >= 2;
    if (newText.isEmpty() || (containsWhitespace(newText) && !quoted)) {
        status_->setStyleSheet(
            QStringLiteral("color: %1;").arg(kWarningColor));
        status_->setText(
            newText.isEmpty()
                ? QStringLiteral("A value cannot be empty — removing a "
                                 "value would change the record layout; do "
                                 "that in the text above.")
                : QStringLiteral("A value cannot contain spaces unless it "
                                 "is \"quoted\" — this format separates "
                                 "values by whitespace."));
        rebuildingTable_ = true;
        item->setText(oldText);
        rebuildingTable_ = false;
        return;
    }

    QTextDocument *document = editor_->document();
    const QTextBlock block = document->findBlockByNumber(gridRow.lineIndex);
    if (!block.isValid())
        return;
    const std::string newLine = lep::lineWithReplacedToken(
        block.text().toStdString(), token, newText.toStdString());

    applyingEdit_ = true;
    QTextCursor cursor(document);
    cursor.setPosition(block.position());
    cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
    cursor.insertText(QString::fromStdString(newLine));
    applyingEdit_ = false;
    syncFromText();
}
