#include "geometry_preprocessor_dialog.h"

#include "geometry_preprocessor.h"

#include <QClipboard>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFontDatabase>
#include <QGridLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QSettings>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <memory>

namespace {

constexpr const char *preDataSettingsKey = "preprocessor/preData";
constexpr const char *lastWorkingSettingsKey = "preprocessor/lastWorkingPreData";

QColor colorForSegment(int autocadColor)
{
    switch (autocadColor) {
    case 2:
        return QColor(0xe0, 0xb0, 0x50); // beta angle ticks
    case 3:
        return QColor(0x4f, 0xc0, 0x86); // right-hand curves, ribs
    default:
        return QColor(0xd8, 0x6a, 0x6a); // planform edges, mirrored side
    }
}

class GeometryPreviewWidget final : public QWidget
{
public:
    explicit GeometryPreviewWidget(QWidget *parent) : QWidget(parent)
    {
        setMinimumSize(420, 380);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    void setResult(const lep::PreProcessorResult &result)
    {
        segments_ = result.drawing;
        error_ = QString::fromStdString(result.error);
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(0x0d, 0x15, 0x23));
        painter.setRenderHint(QPainter::Antialiasing);

        if (!error_.isEmpty() || segments_.empty()) {
            painter.setPen(QColor(0x8f, 0xa5, 0xbd));
            painter.drawText(rect().adjusted(16, 16, -16, -16),
                             Qt::AlignCenter | Qt::TextWordWrap,
                             error_.isEmpty()
                                 ? QStringLiteral("No geometry")
                                 : error_);
            return;
        }

        double minX = segments_.front().x1;
        double maxX = minX;
        double minY = segments_.front().y1;
        double maxY = minY;
        for (const lep::PreProcessorSegment &segment : segments_) {
            for (const double x : {segment.x1, segment.x2}) {
                minX = std::min(minX, x);
                maxX = std::max(maxX, x);
            }
            for (const double y : {segment.y1, segment.y2}) {
                minY = std::min(minY, y);
                maxY = std::max(maxY, y);
            }
        }
        const double margin = 18.0;
        const double width = std::max(maxX - minX, 1.0);
        const double height = std::max(maxY - minY, 1.0);
        const double scale = std::min((this->width() - 2.0 * margin) / width,
                                      (this->height() - 2.0 * margin) / height);
        const double offsetX = (this->width() - width * scale) / 2.0 - minX * scale;
        const double offsetY = (this->height() - height * scale) / 2.0 - minY * scale;

        for (const lep::PreProcessorSegment &segment : segments_) {
            painter.setPen(QPen(colorForSegment(segment.color), 1.0));
            painter.drawLine(QLineF(segment.x1 * scale + offsetX,
                                    segment.y1 * scale + offsetY,
                                    segment.x2 * scale + offsetX,
                                    segment.y2 * scale + offsetY));
        }
    }

private:
    std::vector<lep::PreProcessorSegment> segments_;
    QString error_;
};

bool isCommentLine(const QString &line)
{
    return line.trimmed().startsWith(QLatin1Char('*'));
}

// Cell count currently declared in Section 1 text, or -1 when absent.
int declaredCellCount(const QString &sectionText)
{
    const QStringList lines = sectionText.split(QLatin1Char('\n'));
    for (qsizetype i = 0; i + 1 < lines.size(); ++i) {
        if (isCommentLine(lines.at(i))
            && lines.at(i).contains(QStringLiteral("number of cells"),
                                    Qt::CaseInsensitive)) {
            bool numeric = false;
            const int value = lines.at(i + 1).trimmed().toInt(&numeric);
            return numeric ? value : -1;
        }
    }
    return -1;
}

// Replaces the cell/rib counts and the rib matrix inside Section 1 text,
// leaving every other parameter untouched. The engine's parser is strictly
// positional: exactly two comment lines must sit between the paraglider-type
// value and the first matrix row, and a stray blank line there makes it skip
// the whole matrix silently. The matrix block is therefore rebuilt in exactly
// that canonical shape. Returns an empty string (with *error set) when the
// section has no matrix marker to anchor on.
QString patchGeometrySection(const QString &sectionText,
                             const lep::PreProcessorResult &result,
                             QString *error)
{
    QStringList lines = sectionText.split(QLatin1Char('\n'));

    const auto replaceCountAfter = [&lines](const QString &marker, int value) {
        for (qsizetype i = 0; i + 1 < lines.size(); ++i) {
            if (isCommentLine(lines.at(i))
                && lines.at(i).contains(marker, Qt::CaseInsensitive)) {
                lines[i + 1] = QStringLiteral("\t%1").arg(value);
                return;
            }
        }
    };
    replaceCountAfter(QStringLiteral("number of cells"), result.cellCount);
    replaceCountAfter(QStringLiteral("number of ribs"), result.totalRibCount);

    qsizetype markerIndex = -1;
    for (qsizetype i = 0; i < lines.size(); ++i) {
        if (isCommentLine(lines.at(i))
            && lines.at(i).contains(QStringLiteral("rib geometric parameters"),
                                    Qt::CaseInsensitive)) {
            markerIndex = i;
            break;
        }
    }
    if (markerIndex < 0) {
        *error = QStringLiteral(
            "Section 1 has no \"* Rib geometric parameters\" line, so the "
            "position of the old rib matrix is unknown. Restore a standard "
            "Section 1 (for example with Undo or Versions…) and apply again.");
        return QString();
    }

    QString rows = QString::fromStdString(lep::formatSection1Rows(result));
    while (rows.endsWith(QLatin1Char('\n')))
        rows.chop(1);

    // The document splits sections at the "* N. TITLE" line, so this section's
    // text ends with the banner line(s) that visually introduce the NEXT
    // section. Those must survive the rewrite: the engine skips them by count
    // when it starts reading the next section.
    qsizetype firstRow = markerIndex + 1;
    while (firstRow < lines.size()
           && (isCommentLine(lines.at(firstRow))
               || lines.at(firstRow).trimmed().isEmpty()))
        ++firstRow;
    qsizetype tailStart = lines.size();
    while (tailStart > firstRow
           && (isCommentLine(lines.at(tailStart - 1))
               || lines.at(tailStart - 1).trimmed().isEmpty()))
        --tailStart;
    QStringList trailing;
    for (qsizetype i = tailStart; i < lines.size(); ++i) {
        if (isCommentLine(lines.at(i)))
            trailing << lines.at(i);
    }

    const bool endedWithNewline = !lines.isEmpty() && lines.last().isEmpty();
    // Drop blank lines directly above the marker: the engine's fixed
    // comment-skip reads would swallow them instead of the comment.
    while (markerIndex > 0 && lines.at(markerIndex - 1).trimmed().isEmpty())
        --markerIndex;
    while (lines.size() > markerIndex)
        lines.removeLast();
    lines << QStringLiteral("* Rib geometric parameters");
    lines << QStringLiteral("* Rib    x-rib       y-LE       y-TE         xp"
                            "         z       beta      RP        Washin"
                            "\tRot_z  Pos_z");
    lines << rows.split(QLatin1Char('\n'));
    lines << trailing;
    if (endedWithNewline)
        lines << QString();
    return lines.join(QLatin1Char('\n'));
}

} // namespace

void showGeometryPreprocessorDialog(
    QWidget *parent,
    const QString &geometrySectionText,
    const std::function<void(const QString &, bool cellCountChanged)>
        &applyGeometrySection)
{
    lep::PreProcessorInput initial;
    {
        const QString saved =
            QSettings().value(QLatin1String(preDataSettingsKey)).toString();
        if (!saved.isEmpty()) {
            lep::PreProcessorInput restored;
            std::string ignoredError;
            if (lep::parsePreDataText(saved.toStdString(), &restored, &ignoredError))
                initial = restored;
        }
    }

    QDialog dialog(parent);
    dialog.setWindowTitle(QStringLiteral("Pre-processor of geometry"));
    dialog.setMinimumSize(1080, 660);

    auto *pageLayout = new QVBoxLayout(&dialog);
    pageLayout->setContentsMargins(14, 12, 14, 12);
    pageLayout->setSpacing(10);

    auto *intro = new QLabel(
        QStringLiteral("Generates the Section 1 rib matrix from analytic leading edge, "
                       "trailing edge, vault, and cell distribution parameters "
                       "(port of the Laboratori d'envol pre-processor v1.6 "
                       "“Canigó”). All lengths in centimetres."),
        &dialog);
    intro->setObjectName(QStringLiteral("hint"));
    intro->setWordWrap(true);
    pageLayout->addWidget(intro);

    auto *body = new QHBoxLayout;
    body->setSpacing(12);
    pageLayout->addLayout(body, 1);

    // ---- Parameter column ---------------------------------------------
    auto *parameters = new QGridLayout;
    parameters->setSpacing(10);
    body->addLayout(parameters);

    const auto makeSpin = [&dialog](double minimum, double maximum, double step,
                                    double value, int decimals = 2) {
        auto *spin = new QDoubleSpinBox(&dialog);
        spin->setRange(minimum, maximum);
        spin->setDecimals(decimals);
        spin->setSingleStep(step);
        spin->setValue(value);
        spin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        return spin;
    };
    const auto addRow = [](QGridLayout *grid, int row, const QString &text,
                           QWidget *field) {
        auto *label = new QLabel(text, field->parentWidget());
        label->setObjectName(QStringLiteral("fieldLabel"));
        grid->addWidget(label, row, 0);
        grid->addWidget(field, row, 1);
    };

    auto *generalGroup = new QGroupBox(QStringLiteral("Wing"), &dialog);
    auto *generalGrid = new QGridLayout(generalGroup);
    auto *nameEdit = new QLineEdit(QString::fromStdString(initial.wingName), generalGroup);
    auto *halfSpanSpin = makeSpin(10.0, 20000.0, 5.0, initial.halfSpan);
    addRow(generalGrid, 0, QStringLiteral("Name"), nameEdit);
    addRow(generalGrid, 1, QStringLiteral("Half span xm"), halfSpanSpin);
    parameters->addWidget(generalGroup, 0, 0);

    auto *leadingGroup = new QGroupBox(QStringLiteral("1 · Leading edge (ellipse)"), &dialog);
    auto *leadingGrid = new QGridLayout(leadingGroup);
    auto *leadingA1 = makeSpin(1.0, 20000.0, 5.0, initial.leadingA1);
    auto *leadingB1 = makeSpin(1.0, 20000.0, 5.0, initial.leadingB1);
    auto *leadingX1 = makeSpin(0.0, 20000.0, 5.0, initial.leadingX1);
    auto *leadingX2 = makeSpin(0.0, 20000.0, 5.0, initial.leadingX2);
    auto *leadingC01 = makeSpin(-5000.0, 5000.0, 1.0, initial.leadingC01);
    auto *leadingEx1 = makeSpin(0.1, 10.0, 0.1, initial.leadingEx1);
    auto *leadingC02 = makeSpin(-5000.0, 5000.0, 1.0, initial.leadingC02);
    auto *leadingEx2 = makeSpin(0.1, 10.0, 0.1, initial.leadingEx2);
    addRow(leadingGrid, 0, QStringLiteral("a1 half axis"), leadingA1);
    addRow(leadingGrid, 1, QStringLiteral("b1 half axis"), leadingB1);
    addRow(leadingGrid, 2, QStringLiteral("x1 correction start"), leadingX1);
    addRow(leadingGrid, 3, QStringLiteral("c01 deflection"), leadingC01);
    addRow(leadingGrid, 4, QStringLiteral("ex1 exponent"), leadingEx1);
    addRow(leadingGrid, 5, QStringLiteral("x2 correction start"), leadingX2);
    addRow(leadingGrid, 6, QStringLiteral("c02 deflection"), leadingC02);
    addRow(leadingGrid, 7, QStringLiteral("ex2 exponent"), leadingEx2);
    parameters->addWidget(leadingGroup, 1, 0);

    auto *trailingGroup = new QGroupBox(QStringLiteral("2 · Trailing edge (ellipse)"), &dialog);
    auto *trailingGrid = new QGridLayout(trailingGroup);
    auto *trailingA1 = makeSpin(1.0, 20000.0, 5.0, initial.trailingA1);
    auto *trailingB1 = makeSpin(1.0, 20000.0, 5.0, initial.trailingB1);
    auto *trailingX1 = makeSpin(0.0, 20000.0, 5.0, initial.trailingX1);
    auto *trailingC0 = makeSpin(-5000.0, 5000.0, 1.0, initial.trailingC0);
    auto *trailingY0 = makeSpin(0.0, 20000.0, 5.0, initial.trailingY0);
    auto *trailingExp = makeSpin(0.1, 10.0, 0.1, initial.trailingExp);
    addRow(trailingGrid, 0, QStringLiteral("a1 half axis"), trailingA1);
    addRow(trailingGrid, 1, QStringLiteral("b1 half axis"), trailingB1);
    addRow(trailingGrid, 2, QStringLiteral("x1 correction start"), trailingX1);
    addRow(trailingGrid, 3, QStringLiteral("c0 deflection"), trailingC0);
    addRow(trailingGrid, 4, QStringLiteral("y0 centre offset"), trailingY0);
    addRow(trailingGrid, 5, QStringLiteral("exp exponent"), trailingExp);
    parameters->addWidget(trailingGroup, 0, 1);

    auto *vaultGroup = new QGroupBox(QStringLiteral("3 · Vault"), &dialog);
    auto *vaultLayout = new QVBoxLayout(vaultGroup);
    auto *vaultType = new QComboBox(vaultGroup);
    vaultType->addItem(QStringLiteral("Type 1 · ellipse + cosine modification"));
    vaultType->addItem(QStringLiteral("Type 2 · four tangent circle arcs"));
    vaultType->setCurrentIndex(initial.vaultType == 1 ? 0 : 1);
    vaultLayout->addWidget(vaultType);
    auto *vaultPages = new QStackedWidget(vaultGroup);
    vaultLayout->addWidget(vaultPages);

    auto *vaultEllipsePage = new QWidget(vaultPages);
    auto *vaultEllipseGrid = new QGridLayout(vaultEllipsePage);
    vaultEllipseGrid->setContentsMargins(0, 0, 0, 0);
    auto *vaultA1 = makeSpin(1.0, 20000.0, 5.0, initial.vaultA1);
    auto *vaultB1 = makeSpin(1.0, 20000.0, 5.0, initial.vaultB1);
    auto *vaultX1 = makeSpin(0.1, 20000.0, 5.0, initial.vaultX1);
    auto *vaultC1 = makeSpin(-5000.0, 5000.0, 1.0, initial.vaultC1);
    addRow(vaultEllipseGrid, 0, QStringLiteral("a1 half axis"), vaultA1);
    addRow(vaultEllipseGrid, 1, QStringLiteral("b1 half axis"), vaultB1);
    addRow(vaultEllipseGrid, 2, QStringLiteral("x1 modification start"), vaultX1);
    addRow(vaultEllipseGrid, 3, QStringLiteral("c1 deflection"), vaultC1);
    vaultPages->addWidget(vaultEllipsePage);

    auto *vaultArcsPage = new QWidget(vaultPages);
    auto *vaultArcsGrid = new QGridLayout(vaultArcsPage);
    vaultArcsGrid->setContentsMargins(0, 0, 0, 0);
    vaultArcsGrid->addWidget(new QLabel(QStringLiteral("Radius"), vaultArcsPage), 0, 1);
    vaultArcsGrid->addWidget(new QLabel(QStringLiteral("Angle °"), vaultArcsPage), 0, 2);
    std::array<QDoubleSpinBox *, 4> vaultRadius{};
    std::array<QDoubleSpinBox *, 4> vaultAngle{};
    for (int arc = 0; arc < 4; ++arc) {
        auto *label = new QLabel(QStringLiteral("Arc %1").arg(arc + 1), vaultArcsPage);
        label->setObjectName(QStringLiteral("fieldLabel"));
        vaultArcsGrid->addWidget(label, arc + 1, 0);
        vaultRadius[arc] = makeSpin(1.0, 20000.0, 5.0, initial.vaultRadius[arc]);
        vaultAngle[arc] = makeSpin(0.1, 180.0, 0.5, initial.vaultAngle[arc]);
        vaultArcsGrid->addWidget(vaultRadius[arc], arc + 1, 1);
        vaultArcsGrid->addWidget(vaultAngle[arc], arc + 1, 2);
    }
    vaultPages->addWidget(vaultArcsPage);
    vaultPages->setCurrentIndex(vaultType->currentIndex());
    parameters->addWidget(vaultGroup, 1, 1);

    auto *cellsGroup = new QGroupBox(QStringLiteral("4 · Cell distribution"), &dialog);
    auto *cellsLayout = new QVBoxLayout(cellsGroup);
    auto *cellsType = new QComboBox(cellsGroup);
    cellsType->addItem(QStringLiteral("1 · Uniform"));
    cellsType->addItem(QStringLiteral("2 · Linear taper towards tip"));
    cellsType->addItem(QStringLiteral("3 · Proportional to chord"));
    cellsType->addItem(QStringLiteral("4 · Explicit widths"));
    cellsType->setCurrentIndex(
        std::clamp(initial.cellDistribution, 1, 4) - 1);
    cellsLayout->addWidget(cellsType);
    auto *cellsGrid = new QGridLayout;
    cellsLayout->addLayout(cellsGrid);
    auto *cellCount = new QSpinBox(cellsGroup);
    cellCount->setRange(1, 199);
    cellCount->setValue(initial.cellCount);
    cellCount->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    auto *cellCoefficient = makeSpin(0.0, 1.0, 0.05, initial.cellCoefficient);
    addRow(cellsGrid, 0, QStringLiteral("Number of cells"), cellCount);
    addRow(cellsGrid, 1, QStringLiteral("Coefficient (0–1)"), cellCoefficient);
    auto *cellWidthsEdit = new QPlainTextEdit(cellsGroup);
    cellWidthsEdit->setPlaceholderText(
        QStringLiteral("One cell width in cm per rib row.\n"
                       "First row 0 makes the cell count even (centre rib)."));
    cellWidthsEdit->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    cellWidthsEdit->setMaximumHeight(110);
    if (!initial.cellWidths.empty()) {
        QStringList widthLines;
        for (const double width : initial.cellWidths)
            widthLines << QString::number(width, 'f', 2);
        cellWidthsEdit->setPlainText(widthLines.join(QLatin1Char('\n')));
    }
    cellsLayout->addWidget(cellWidthsEdit);
    parameters->addWidget(cellsGroup, 2, 0, 1, 2);
    parameters->setRowStretch(3, 1);

    // ---- Result column -------------------------------------------------
    auto *results = new QVBoxLayout;
    results->setSpacing(8);
    body->addLayout(results, 1);

    auto *tabs = new QTabWidget(&dialog);
    auto *preview = new GeometryPreviewWidget(tabs);
    tabs->addTab(preview, QStringLiteral("Preview"));
    auto *report = new QPlainTextEdit(tabs);
    report->setReadOnly(true);
    report->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    report->setLineWrapMode(QPlainTextEdit::NoWrap);
    tabs->addTab(report, QStringLiteral("geometry-out.txt"));
    results->addWidget(tabs, 1);

    auto *summary = new QLabel(&dialog);
    summary->setObjectName(QStringLiteral("hint"));
    summary->setWordWrap(true);
    results->addWidget(summary);

    auto *buttons = new QHBoxLayout;
    buttons->setSpacing(8);
    auto *openButton = new QPushButton(QStringLiteral("Open pre-data…"), &dialog);
    auto *saveButton = new QPushButton(QStringLiteral("Save pre-data…"), &dialog);
    auto *exportButton = new QPushButton(QStringLiteral("Export results…"), &dialog);
    auto *defaultsButton = new QPushButton(QStringLiteral("Load defaults"), &dialog);
    defaultsButton->setToolTip(
        QStringLiteral("Discard the current parameters and restore the "
                       "built-in v1.6 sample wing, which always processes"));
    auto *restoreButton =
        new QPushButton(QStringLiteral("Restore last working"), &dialog);
    restoreButton->setToolTip(
        QStringLiteral("Go back to the most recent parameter set that "
                       "processed successfully (remembered across sessions)"));
    auto *copyButton = new QPushButton(QStringLiteral("Copy matrix"), &dialog);
    auto *applyButton = new QPushButton(QStringLiteral("Apply to Section 1"), &dialog);
    auto *closeButton = new QPushButton(QStringLiteral("Close"), &dialog);
    for (QPushButton *button :
         {openButton, saveButton, exportButton, defaultsButton, restoreButton,
          copyButton})
        button->setObjectName(QStringLiteral("secondaryButton"));
    applyButton->setObjectName(QStringLiteral("primaryButton"));
    closeButton->setObjectName(QStringLiteral("quietButton"));
    applyButton->setToolTip(
        QStringLiteral("Replace the cell/rib counts and the rib matrix in "
                       "Section 1 of the open design"));
    buttons->addWidget(openButton);
    buttons->addWidget(saveButton);
    buttons->addWidget(exportButton);
    buttons->addWidget(defaultsButton);
    buttons->addWidget(restoreButton);
    buttons->addStretch();
    buttons->addWidget(copyButton);
    buttons->addWidget(applyButton);
    buttons->addWidget(closeButton);
    pageLayout->addLayout(buttons);

    // ---- Behaviour -----------------------------------------------------
    auto latestInput = std::make_shared<lep::PreProcessorInput>(initial);
    auto latestResult = std::make_shared<lep::PreProcessorResult>();
    auto lastWorking = std::make_shared<QString>(
        QSettings().value(QLatin1String(lastWorkingSettingsKey)).toString());

    const auto readInput = [=] {
        lep::PreProcessorInput input;
        input.wingName = nameEdit->text().trimmed().isEmpty()
            ? std::string("wing")
            : nameEdit->text().trimmed().toStdString();
        input.halfSpan = halfSpanSpin->value();
        input.leadingA1 = leadingA1->value();
        input.leadingB1 = leadingB1->value();
        input.leadingX1 = leadingX1->value();
        input.leadingX2 = leadingX2->value();
        input.leadingC01 = leadingC01->value();
        input.leadingEx1 = leadingEx1->value();
        input.leadingC02 = leadingC02->value();
        input.leadingEx2 = leadingEx2->value();
        input.trailingA1 = trailingA1->value();
        input.trailingB1 = trailingB1->value();
        input.trailingX1 = trailingX1->value();
        input.trailingC0 = trailingC0->value();
        input.trailingY0 = trailingY0->value();
        input.trailingExp = trailingExp->value();
        input.vaultType = vaultType->currentIndex() == 0 ? 1 : 2;
        input.vaultA1 = vaultA1->value();
        input.vaultB1 = vaultB1->value();
        input.vaultX1 = vaultX1->value();
        input.vaultC1 = vaultC1->value();
        for (int arc = 0; arc < 4; ++arc) {
            input.vaultRadius[arc] = vaultRadius[arc]->value();
            input.vaultAngle[arc] = vaultAngle[arc]->value();
        }
        input.cellDistribution = cellsType->currentIndex() + 1;
        input.cellCoefficient = cellCoefficient->value();
        input.cellCount = cellCount->value();
        input.cellWidths.clear();
        const QStringList widthLines =
            cellWidthsEdit->toPlainText().split(QLatin1Char('\n'), Qt::SkipEmptyParts);
        for (const QString &line : widthLines) {
            bool numeric = false;
            const double width = line.trimmed().toDouble(&numeric);
            if (numeric)
                input.cellWidths.push_back(width);
        }
        return input;
    };

    const bool canApply = applyGeometrySection != nullptr && !geometrySectionText.isEmpty();
    const auto recompute = [=] {
        *latestInput = readInput();
        *latestResult = lep::runPreProcessor(*latestInput);
        preview->setResult(*latestResult);
        if (latestResult->ok()) {
            report->setPlainText(QString::fromStdString(
                lep::formatGeometryOutText(*latestInput, *latestResult)));
            summary->setText(
                QStringLiteral("%1 cells · %2 matrix rows (%3 ribs) · "
                               "span %4 m (proj %5 m) · area %6 m² (proj %7 m²) · "
                               "AR %8 (proj %9) · flattening %10 %")
                    .arg(latestResult->cellCount)
                    .arg(latestResult->ribs.size())
                    .arg(latestResult->totalRibCount)
                    .arg(latestResult->flatSpan, 0, 'f', 2)
                    .arg(latestResult->projectedSpan, 0, 'f', 2)
                    .arg(latestResult->flatArea, 0, 'f', 2)
                    .arg(latestResult->projectedArea, 0, 'f', 2)
                    .arg(latestResult->flatAspectRatio, 0, 'f', 2)
                    .arg(latestResult->projectedAspectRatio, 0, 'f', 2)
                    .arg(latestResult->flattening * 100.0, 0, 'f', 0));
        } else {
            report->setPlainText(QString::fromStdString(latestResult->error));
            summary->setText(QString::fromStdString(latestResult->error));
        }
        if (latestResult->ok()) {
            *lastWorking =
                QString::fromStdString(lep::formatPreDataText(*latestInput));
        }
        copyButton->setEnabled(latestResult->ok());
        exportButton->setEnabled(latestResult->ok());
        applyButton->setEnabled(latestResult->ok() && canApply);
        restoreButton->setEnabled(!lastWorking->isEmpty());
    };
    if (!canApply) {
        applyButton->setToolTip(
            QStringLiteral("Open a design first to apply the matrix to its "
                           "Section 1"));
    }

    const auto writeWidgets = [=](const lep::PreProcessorInput &input) {
        const QSignalBlocker blockers[] = {
            QSignalBlocker(nameEdit), QSignalBlocker(halfSpanSpin),
            QSignalBlocker(leadingA1), QSignalBlocker(leadingB1),
            QSignalBlocker(leadingX1), QSignalBlocker(leadingX2),
            QSignalBlocker(leadingC01), QSignalBlocker(leadingEx1),
            QSignalBlocker(leadingC02), QSignalBlocker(leadingEx2),
            QSignalBlocker(trailingA1), QSignalBlocker(trailingB1),
            QSignalBlocker(trailingX1), QSignalBlocker(trailingC0),
            QSignalBlocker(trailingY0), QSignalBlocker(trailingExp),
            QSignalBlocker(vaultType), QSignalBlocker(vaultA1),
            QSignalBlocker(vaultB1), QSignalBlocker(vaultX1),
            QSignalBlocker(vaultC1), QSignalBlocker(cellsType),
            QSignalBlocker(cellCount), QSignalBlocker(cellCoefficient),
            QSignalBlocker(cellWidthsEdit),
            QSignalBlocker(vaultRadius[0]), QSignalBlocker(vaultAngle[0]),
            QSignalBlocker(vaultRadius[1]), QSignalBlocker(vaultAngle[1]),
            QSignalBlocker(vaultRadius[2]), QSignalBlocker(vaultAngle[2]),
            QSignalBlocker(vaultRadius[3]), QSignalBlocker(vaultAngle[3]),
        };
        (void)blockers;
        nameEdit->setText(QString::fromStdString(input.wingName));
        halfSpanSpin->setValue(input.halfSpan);
        leadingA1->setValue(input.leadingA1);
        leadingB1->setValue(input.leadingB1);
        leadingX1->setValue(input.leadingX1);
        leadingX2->setValue(input.leadingX2);
        leadingC01->setValue(input.leadingC01);
        leadingEx1->setValue(input.leadingEx1);
        leadingC02->setValue(input.leadingC02);
        leadingEx2->setValue(input.leadingEx2);
        trailingA1->setValue(input.trailingA1);
        trailingB1->setValue(input.trailingB1);
        trailingX1->setValue(input.trailingX1);
        trailingC0->setValue(input.trailingC0);
        trailingY0->setValue(input.trailingY0);
        trailingExp->setValue(input.trailingExp);
        vaultType->setCurrentIndex(input.vaultType == 1 ? 0 : 1);
        vaultPages->setCurrentIndex(vaultType->currentIndex());
        vaultA1->setValue(input.vaultA1);
        vaultB1->setValue(input.vaultB1);
        vaultX1->setValue(input.vaultX1);
        vaultC1->setValue(input.vaultC1);
        for (int arc = 0; arc < 4; ++arc) {
            vaultRadius[arc]->setValue(input.vaultRadius[arc]);
            vaultAngle[arc]->setValue(input.vaultAngle[arc]);
        }
        cellsType->setCurrentIndex(std::clamp(input.cellDistribution, 1, 4) - 1);
        cellCount->setValue(input.cellCount);
        cellCoefficient->setValue(input.cellCoefficient);
        QStringList widthLines;
        for (const double width : input.cellWidths)
            widthLines << QString::number(width, 'f', 2);
        cellWidthsEdit->setPlainText(widthLines.join(QLatin1Char('\n')));
    };

    const auto updateCellControls = [=] {
        const int distribution = cellsType->currentIndex() + 1;
        cellCoefficient->setEnabled(distribution == 2 || distribution == 3);
        cellCount->setEnabled(distribution != 4);
        cellWidthsEdit->setVisible(distribution == 4);
    };
    updateCellControls();

    for (QDoubleSpinBox *spin :
         {halfSpanSpin, leadingA1, leadingB1, leadingX1, leadingX2, leadingC01,
          leadingEx1, leadingC02, leadingEx2, trailingA1, trailingB1, trailingX1,
          trailingC0, trailingY0, trailingExp, vaultA1, vaultB1, vaultX1, vaultC1,
          cellCoefficient, vaultRadius[0], vaultRadius[1], vaultRadius[2],
          vaultRadius[3], vaultAngle[0], vaultAngle[1], vaultAngle[2],
          vaultAngle[3]}) {
        QObject::connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged),
                         &dialog, [recompute](double) { recompute(); });
    }
    QObject::connect(cellCount, qOverload<int>(&QSpinBox::valueChanged),
                     &dialog, [recompute](int) { recompute(); });
    QObject::connect(vaultType, qOverload<int>(&QComboBox::currentIndexChanged),
                     &dialog, [vaultPages, recompute](int index) {
                         vaultPages->setCurrentIndex(index);
                         recompute();
                     });
    QObject::connect(cellsType, qOverload<int>(&QComboBox::currentIndexChanged),
                     &dialog, [updateCellControls, recompute](int) {
                         updateCellControls();
                         recompute();
                     });
    QObject::connect(cellWidthsEdit, &QPlainTextEdit::textChanged,
                     &dialog, recompute);

    QObject::connect(defaultsButton, &QPushButton::clicked, &dialog,
                     [writeWidgets, updateCellControls, recompute] {
        writeWidgets(lep::PreProcessorInput{});
        updateCellControls();
        recompute();
    });

    QObject::connect(restoreButton, &QPushButton::clicked, &dialog,
                     [lastWorking, writeWidgets, updateCellControls, recompute] {
        lep::PreProcessorInput parsed;
        std::string ignoredError;
        if (lastWorking->isEmpty()
            || !lep::parsePreDataText(lastWorking->toStdString(), &parsed,
                                      &ignoredError)) {
            return;
        }
        writeWidgets(parsed);
        updateCellControls();
        recompute();
    });

    QObject::connect(openButton, &QPushButton::clicked, &dialog,
                     [&dialog, writeWidgets, updateCellControls, recompute] {
        const QString path = QFileDialog::getOpenFileName(
            &dialog, QStringLiteral("Open pre-data file"), QString(),
            QStringLiteral("Pre-processor data (*.txt);;All files (*)"));
        if (path.isEmpty())
            return;
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::warning(&dialog, QStringLiteral("Open pre-data"),
                                 QStringLiteral("Could not read %1").arg(path));
            return;
        }
        lep::PreProcessorInput parsed;
        std::string error;
        if (!lep::parsePreDataText(QString::fromUtf8(file.readAll()).toStdString(),
                                   &parsed, &error)) {
            QMessageBox::warning(
                &dialog, QStringLiteral("Open pre-data"),
                QStringLiteral("%1\n\n%2").arg(path,
                                               QString::fromStdString(error)));
            return;
        }
        writeWidgets(parsed);
        updateCellControls();
        recompute();
    });

    QObject::connect(saveButton, &QPushButton::clicked, &dialog,
                     [&dialog, readInput] {
        const QString path = QFileDialog::getSaveFileName(
            &dialog, QStringLiteral("Save pre-data file"),
            QStringLiteral("pre-data.txt"),
            QStringLiteral("Pre-processor data (*.txt);;All files (*)"));
        if (path.isEmpty())
            return;
        QSaveFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(&dialog, QStringLiteral("Save pre-data"),
                                 QStringLiteral("Could not write %1").arg(path));
            return;
        }
        const std::string text = lep::formatPreDataText(readInput());
        file.write(text.data(), static_cast<qint64>(text.size()));
        file.commit();
    });

    QObject::connect(exportButton, &QPushButton::clicked, &dialog,
                     [&dialog, latestInput, latestResult] {
        if (!latestResult->ok())
            return;
        const QString directory = QFileDialog::getExistingDirectory(
            &dialog, QStringLiteral("Export geometry-out.txt, geometry.dxf, "
                                    "and pre-data.txt to"));
        if (directory.isEmpty())
            return;
        const auto writeFile = [&dialog, &directory](const QString &name,
                                                     const std::string &text) {
            QSaveFile file(QDir(directory).filePath(name));
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QMessageBox::warning(
                    &dialog, QStringLiteral("Export results"),
                    QStringLiteral("Could not write %1").arg(file.fileName()));
                return;
            }
            file.write(text.data(), static_cast<qint64>(text.size()));
            file.commit();
        };
        writeFile(QStringLiteral("pre-data.txt"),
                  lep::formatPreDataText(*latestInput));
        writeFile(QStringLiteral("geometry-out.txt"),
                  lep::formatGeometryOutText(*latestInput, *latestResult));
        writeFile(QStringLiteral("geometry.dxf"),
                  lep::formatGeometryDxfText(*latestResult));
    });

    QObject::connect(copyButton, &QPushButton::clicked, &dialog, [latestResult] {
        if (latestResult->ok()) {
            QGuiApplication::clipboard()->setText(
                QString::fromStdString(lep::formatSection1Rows(*latestResult)));
        }
    });

    QObject::connect(applyButton, &QPushButton::clicked, &dialog,
                     [&dialog, latestResult, geometrySectionText,
                      applyGeometrySection] {
        if (!latestResult->ok())
            return;
        const int wingCells = declaredCellCount(geometrySectionText);
        const bool cellCountChanged =
            wingCells >= 0 && wingCells != latestResult->cellCount;
        if (cellCountChanged) {
            QMessageBox box(&dialog);
            box.setIcon(QMessageBox::Warning);
            box.setWindowTitle(QStringLiteral("Cell count changes"));
            box.setText(
                QStringLiteral("The open design has %1 cells; the generated "
                               "geometry has %2.")
                    .arg(wingCells)
                    .arg(latestResult->cellCount));
            box.setInformativeText(QStringLiteral(
                "The other sections keep one row per rib (airfoils, anchor "
                "points, lines…). After applying, update them to the new rib "
                "count — the design cannot be calculated until they match, "
                "so the 3D preview is not rebuilt automatically."));
            auto *applyAnyway = box.addButton(QStringLiteral("Apply anyway"),
                                              QMessageBox::AcceptRole);
            box.addButton(QMessageBox::Cancel);
            box.exec();
            if (box.clickedButton() != applyAnyway)
                return;
        }
        QString patchError;
        const QString patched =
            patchGeometrySection(geometrySectionText, *latestResult, &patchError);
        if (patched.isEmpty()) {
            QMessageBox::warning(&dialog, QStringLiteral("Cannot apply geometry"),
                                 patchError);
            return;
        }
        applyGeometrySection(patched, cellCountChanged);
        dialog.accept();
    });
    QObject::connect(closeButton, &QPushButton::clicked, &dialog,
                     [&dialog] { dialog.reject(); });

    recompute();
    dialog.exec();

    QSettings settings;
    settings.setValue(QLatin1String(preDataSettingsKey),
                      QString::fromStdString(lep::formatPreDataText(readInput())));
    if (!lastWorking->isEmpty())
        settings.setValue(QLatin1String(lastWorkingSettingsKey), *lastWorking);
}
