#include "mainwindow.h"

#include "paraglider_view.h"
#include "section_help.h"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDatabase>
#include <QFontMetricsF>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QMimeData>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollBar>
#include <QSaveFile>
#include <QSettings>
#include <QSplitter>
#include <QStackedWidget>
#include <QSyntaxHighlighter>
#include <QTabWidget>
#include <QTextBrowser>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextDocument>
#include <QTemporaryDir>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QUrl>
#include <QVBoxLayout>

#include <array>
#include <functional>
#include <utility>

namespace {

struct OutputDescription
{
    const char *fileName;
    const char *description;
};

constexpr std::array<OutputDescription, 6> outputs{{
    {"leparagliding.dxf", "2D manufacturing plans"},
    {"lep-3d.step", "OCCT NURBS 3D model"},
    {"lep-3d.dxf", "Legacy 3D wireframe (reference)"},
    {"lep-out.txt", "Design calculations"},
    {"lines.txt", "Suspension line data"},
    {"run-log.txt", "Calculation progress and diagnostics"},
}};

constexpr auto manualUrl =
    "https://www.laboratoridenvol.com/leparagliding/manual.en.html";

QString humanReadableSize(qint64 bytes)
{
    constexpr qint64 kibibyte = 1024;
    constexpr qint64 mebibyte = kibibyte * 1024;

    if (bytes >= mebibyte) {
        return QStringLiteral("%1 MB")
            .arg(bytes / static_cast<double>(mebibyte), 0, 'f', 1);
    }
    if (bytes >= kibibyte) {
        return QStringLiteral("%1 KB")
            .arg(bytes / static_cast<double>(kibibyte), 0, 'f', 1);
    }
    return QStringLiteral("%1 B").arg(bytes);
}

QString glueVentRowsHtml(const QString &sectionText)
{
    QStringList records;
    for (const QString &line : sectionText.split(QLatin1Char('\n'))) {
        const QString record = line.trimmed();
        if (!record.isEmpty() && !record.startsWith(QLatin1Char('*'))) {
            records.append(record);
        }
    }
    if (records.isEmpty()) {
        return QStringLiteral(
            "<h3>Current values</h3><p>No data records were found.</p>");
    }

    bool enabledOk = false;
    const int enabled = records.constFirst().section(
        QRegularExpression(QStringLiteral("\\s+")), 0, 0).toInt(&enabledOk);
    if (!enabledOk) {
        return QStringLiteral(
            "<h3>Current values</h3><p>The first data record is not a valid "
            "<code>0</code>/<code>1</code> mode.</p>");
    }
    if (enabled == 0) {
        return QStringLiteral(
            "<h3>Current values</h3><p><code>0</code>: old automatic vent "
            "construction is active; no explicit cell rows are read.</p>");
    }

    QString html = QStringLiteral(
        "<h3>Current values</h3>"
        "<p>Explicit mode is enabled. The editor currently contains %1 cell rows.</p>"
        "<table cellspacing=\"0\" cellpadding=\"5\" border=\"1\">"
        "<tr><th>Cell</th><th>Record</th><th>Interpretation</th></tr>")
                       .arg(records.size() - 1);

    for (qsizetype row = 1; row < records.size(); ++row) {
        const QStringList fields =
            records.at(row).split(QRegularExpression(QStringLiteral("\\s+")),
                                  Qt::SkipEmptyParts);
        bool cellOk = false;
        bool typeOk = false;
        const int cell = fields.value(0).toInt(&cellOk);
        const int type = fields.value(1).toInt(&typeOk);
        QString interpretation;
        int expectedFields = 2;

        if (!cellOk || !typeOk) {
            interpretation = QStringLiteral(
                "<b>Invalid:</b> the first two fields must be integer cell and type.");
        } else {
            switch (type) {
            case 0:
                interpretation = QStringLiteral(
                    "Separate open inlet; glued to neither skin.");
                break;
            case 1:
                interpretation = QStringLiteral(
                    "Attached to upper skin (extrados).");
                break;
            case -1:
                interpretation = QStringLiteral(
                    "Attached to lower skin (intrados), commonly a closed cell.");
                break;
            case -2:
                interpretation = QStringLiteral(
                    "Fixed lower-skin diagonal, fully open at the left side.");
                break;
            case -3:
                interpretation = QStringLiteral(
                    "Fixed lower-skin diagonal, fully open at the right side.");
                break;
            case 4:
            case -4:
                expectedFields = 4;
                interpretation =
                    QStringLiteral("%1-skin straight diagonal: left %2%, right %3%.")
                        .arg(type > 0 ? QStringLiteral("Upper")
                                      : QStringLiteral("Lower"),
                             fields.value(2).toHtmlEscaped(),
                             fields.value(3).toHtmlEscaped());
                break;
            case 5:
            case -5:
                expectedFields = 5;
                interpretation =
                    QStringLiteral(
                        "%1-skin curved inlet: left %2%, right %3%, arc depth %4%.")
                        .arg(type > 0 ? QStringLiteral("Upper")
                                      : QStringLiteral("Lower"),
                             fields.value(2).toHtmlEscaped(),
                             fields.value(3).toHtmlEscaped(),
                             fields.value(4).toHtmlEscaped());
                break;
            case 6:
            case -6:
                expectedFields = 4;
                interpretation =
                    QStringLiteral("%1-skin elliptical inlet: X width %2%, Y width %3%.")
                        .arg(type > 0 ? QStringLiteral("Upper")
                                      : QStringLiteral("Lower"),
                             fields.value(2).toHtmlEscaped(),
                             fields.value(3).toHtmlEscaped());
                break;
            default:
                interpretation =
                    QStringLiteral("<b>Unknown vent type %1.</b>").arg(type);
                break;
            }
            if (cell != row) {
                interpretation.prepend(
                    QStringLiteral("<b>Expected cell label %1 here.</b> ").arg(row));
            }
            if (fields.size() != expectedFields) {
                interpretation.append(
                    QStringLiteral(
                        " <b>This type expects %1 fields, but this row has %2.</b>")
                        .arg(expectedFields)
                        .arg(fields.size()));
            }
        }

        html += QStringLiteral("<tr><td>%1</td><td><code>%2</code></td><td>%3</td></tr>")
                    .arg(cellOk ? QString::number(cell) : QStringLiteral("?"),
                         records.at(row).toHtmlEscaped(),
                         interpretation);
    }
    html += QStringLiteral("</table>");
    return html;
}

QFrame *makeCard(QWidget *parent = nullptr)
{
    auto *card = new QFrame(parent);
    card->setObjectName(QStringLiteral("card"));
    return card;
}

QToolButton *makeViewButton(
    const QString &text,
    const QString &toolTip,
    QWidget *parent)
{
    auto *button = new QToolButton(parent);
    button->setText(text);
    button->setToolTip(toolTip);
    button->setObjectName(QStringLiteral("viewButton"));
    button->setAutoRaise(false);
    return button;
}

class DesignSyntaxHighlighter final : public QSyntaxHighlighter
{
public:
    explicit DesignSyntaxHighlighter(QTextDocument *document)
        : QSyntaxHighlighter(document)
    {
        commentFormat_.setForeground(QColor(QStringLiteral("#70839b")));
        commentFormat_.setFontItalic(true);
        numberFormat_.setForeground(QColor(QStringLiteral("#78d9ff")));
        stringFormat_.setForeground(QColor(QStringLiteral("#ffd88a")));
    }

protected:
    void highlightBlock(const QString &text) override
    {
        if (text.trimmed().startsWith(QLatin1Char('*'))) {
            setFormat(0, text.size(), commentFormat_);
            return;
        }

        static const QRegularExpression quoted(QStringLiteral(R"("[^"]*")"));
        auto strings = quoted.globalMatch(text);
        while (strings.hasNext()) {
            const auto match = strings.next();
            setFormat(match.capturedStart(), match.capturedLength(), stringFormat_);
        }

        static const QRegularExpression numbers(
            QStringLiteral(R"((?<![A-Za-z_])[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[Ee][-+]?\d+)?)"));
        auto values = numbers.globalMatch(text);
        while (values.hasNext()) {
            const auto match = values.next();
            setFormat(match.capturedStart(), match.capturedLength(), numberFormat_);
        }
    }

private:
    QTextCharFormat commentFormat_;
    QTextCharFormat numberFormat_;
    QTextCharFormat stringFormat_;
};

class DesignSectionEditor final : public QPlainTextEdit
{
public:
    using QPlainTextEdit::QPlainTextEdit;

    std::function<void()> buildRequested;
    std::function<void()> persistedUndoRequested;
    std::function<void()> persistedRedoRequested;

protected:
    void keyPressEvent(QKeyEvent *event) override
    {
        if (event->matches(QKeySequence::Undo)
            && !document()->isUndoAvailable()
            && persistedUndoRequested) {
            persistedUndoRequested();
            event->accept();
            return;
        }
        if (event->matches(QKeySequence::Redo)
            && !document()->isRedoAvailable()
            && persistedRedoRequested) {
            persistedRedoRequested();
            event->accept();
            return;
        }

        const bool enter =
            event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter;
        if (enter && !event->modifiers().testFlag(Qt::ShiftModifier)) {
            if (buildRequested) {
                buildRequested();
            }
            event->accept();
            return;
        }
        QPlainTextEdit::keyPressEvent(event);
    }
};

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , process_(new QProcess(this))
{
    setAcceptDrops(true);
    setMinimumSize(1120, 720);
    resize(1540, 940);

    buildInterface();
    connectProcess();
    loadSettings();

    if (QFileInfo(inputEdit_->text()).isFile()) {
        loadDesign(inputEdit_->text(), false);
    }
    refreshOutputFiles();
    updateRunAvailability();
    updateWindowTitle();
}

MainWindow::~MainWindow()
{
    if (process_->state() != QProcess::NotRunning) {
        process_->kill();
        process_->waitForFinished(3000);
    }
    saveSettings();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (maybeSaveChanges()) {
        saveSettings();
        event->accept();
    } else {
        event->ignore();
    }
}

void MainWindow::buildInterface()
{
    auto *central = new QWidget(this);
    auto *page = new QVBoxLayout(central);
    page->setContentsMargins(18, 16, 18, 14);
    page->setSpacing(12);

    auto *hero = new QHBoxLayout;
    hero->setSpacing(12);

    auto *mark = new QLabel(QStringLiteral("LE"), central);
    mark->setObjectName(QStringLiteral("brandMark"));
    mark->setAlignment(Qt::AlignCenter);
    mark->setFixedSize(48, 48);
    hero->addWidget(mark);

    auto *titles = new QVBoxLayout;
    titles->setSpacing(0);
    auto *title = new QLabel(QStringLiteral("LEparagliding Studio"), central);
    title->setObjectName(QStringLiteral("title"));
    auto *subtitle = new QLabel(
        QStringLiteral("Section editor · compatible C++ engine · interactive 3D model"),
        central);
    subtitle->setObjectName(QStringLiteral("subtitle"));
    titles->addWidget(title);
    titles->addWidget(subtitle);
    hero->addLayout(titles);
    hero->addStretch();

    auto *manualButton = new QPushButton(QStringLiteral("Manual"), central);
    manualButton->setObjectName(QStringLiteral("quietButton"));
    hero->addWidget(manualButton);
    auto *version = new QLabel(QStringLiteral("3.28 Jardins · Qt 6"), central);
    version->setObjectName(QStringLiteral("badge"));
    hero->addWidget(version);
    page->addLayout(hero);

    auto *fileCard = makeCard(central);
    auto *files = new QGridLayout(fileCard);
    files->setContentsMargins(14, 12, 14, 12);
    files->setHorizontalSpacing(9);
    files->setVerticalSpacing(7);
    files->setColumnStretch(2, 1);
    files->setColumnStretch(5, 1);

    auto *inputLabel = new QLabel(QStringLiteral("Design"), fileCard);
    inputLabel->setObjectName(QStringLiteral("fieldLabel"));
    files->addWidget(inputLabel, 0, 0);
    inputEdit_ = new QLineEdit(fileCard);
    inputEdit_->setReadOnly(true);
    inputEdit_->setPlaceholderText(QStringLiteral("Open leparagliding.txt"));
    files->addWidget(inputEdit_, 0, 1, 1, 2);
    inputBrowseButton_ = new QPushButton(QStringLiteral("Open…"), fileCard);
    inputBrowseButton_->setObjectName(QStringLiteral("secondaryButton"));
    files->addWidget(inputBrowseButton_, 0, 3);

    auto *outputLabel = new QLabel(QStringLiteral("Export to"), fileCard);
    outputLabel->setObjectName(QStringLiteral("fieldLabel"));
    files->addWidget(outputLabel, 0, 4);
    outputEdit_ = new QLineEdit(fileCard);
    outputEdit_->setPlaceholderText(QStringLiteral("Export directory"));
    files->addWidget(outputEdit_, 0, 5);
    outputBrowseButton_ = new QPushButton(QStringLiteral("Browse…"), fileCard);
    outputBrowseButton_->setObjectName(QStringLiteral("secondaryButton"));
    files->addWidget(outputBrowseButton_, 0, 6);

    historyButton_ = new QPushButton(QStringLiteral("Versions…"), fileCard);
    historyButton_->setObjectName(QStringLiteral("secondaryButton"));
    historyButton_->setEnabled(false);
    historyButton_->setToolTip(
        QStringLiteral("Browse and restore versions embedded in this wing file"));
    files->addWidget(historyButton_, 0, 7);
    saveButton_ = new QPushButton(QStringLiteral("Save"), fileCard);
    saveButton_->setObjectName(QStringLiteral("secondaryButton"));
    saveButton_->setEnabled(false);
    files->addWidget(saveButton_, 0, 8);
    buildButton_ = new QPushButton(QStringLiteral("Build paraglider"), fileCard);
    buildButton_->setObjectName(QStringLiteral("primaryButton"));
    buildButton_->setMinimumWidth(155);
    buildButton_->setToolTip(
        QStringLiteral("Calculate the current editors and refresh the temporary 3D preview"));
    files->addWidget(buildButton_, 0, 9);
    exportButton_ = new QPushButton(QStringLiteral("Export files…"), fileCard);
    exportButton_->setObjectName(QStringLiteral("secondaryButton"));
    exportButton_->setMinimumWidth(120);
    exportButton_->setToolTip(
        QStringLiteral("Write manufacturing, 3D, report, and line files to Output"));
    files->addWidget(exportButton_, 0, 10);

    inputDetails_ = new QLabel(
        QStringLiteral("Open a design to create its section editors."),
        fileCard);
    inputDetails_->setObjectName(QStringLiteral("hint"));
    files->addWidget(inputDetails_, 1, 1, 1, 10);
    page->addWidget(fileCard);

    auto *workspaceSplitter = new QSplitter(Qt::Vertical, central);
    workspaceSplitter->setChildrenCollapsible(false);

    auto *mainSplitter = new QSplitter(Qt::Horizontal, workspaceSplitter);
    mainSplitter->setChildrenCollapsible(false);

    auto *editorCard = makeCard(mainSplitter);
    auto *editorLayout = new QHBoxLayout(editorCard);
    editorLayout->setContentsMargins(0, 0, 0, 0);
    editorLayout->setSpacing(0);

    auto *navigator = new QWidget(editorCard);
    navigator->setObjectName(QStringLiteral("sectionNavigator"));
    navigator->setMinimumWidth(190);
    navigator->setMaximumWidth(270);
    auto *navigatorLayout = new QVBoxLayout(navigator);
    navigatorLayout->setContentsMargins(12, 13, 10, 12);
    navigatorLayout->setSpacing(8);
    auto *sectionsTitle = new QLabel(QStringLiteral("Design sections"), navigator);
    sectionsTitle->setObjectName(QStringLiteral("sectionTitle"));
    navigatorLayout->addWidget(sectionsTitle);
    sectionList_ = new QListWidget(navigator);
    sectionList_->setObjectName(QStringLiteral("sectionList"));
    sectionList_->setSpacing(1);
    navigatorLayout->addWidget(sectionList_, 1);
    editorLayout->addWidget(navigator);

    sectionPages_ = new QStackedWidget(editorCard);
    auto *emptyEditor = new QLabel(
        QStringLiteral("Open a LEparagliding design file to edit its numbered sections."),
        sectionPages_);
    emptyEditor->setAlignment(Qt::AlignCenter);
    emptyEditor->setObjectName(QStringLiteral("emptyState"));
    sectionPages_->addWidget(emptyEditor);
    editorLayout->addWidget(sectionPages_, 1);
    mainSplitter->addWidget(editorCard);

    auto *viewportCard = makeCard(mainSplitter);
    auto *viewportLayout = new QVBoxLayout(viewportCard);
    viewportLayout->setContentsMargins(10, 10, 10, 10);
    viewportLayout->setSpacing(8);

    auto *viewHeader = new QHBoxLayout;
    auto *viewTitle = new QLabel(QStringLiteral("3D model"), viewportCard);
    viewTitle->setObjectName(QStringLiteral("sectionTitle"));
    viewHeader->addWidget(viewTitle);
    modelStats_ = new QLabel(QStringLiteral("No model loaded"), viewportCard);
    modelStats_->setObjectName(QStringLiteral("hint"));
    viewHeader->addWidget(modelStats_);
    viewHeader->addStretch();
    viewportLayout->addLayout(viewHeader);

    auto *fitButton = makeViewButton(QStringLiteral("Fit"), QStringLiteral("Fit all (F)"), viewportCard);
    auto *isoButton = makeViewButton(QStringLiteral("Iso"), QStringLiteral("Isometric (0)"), viewportCard);
    auto *frontButton = makeViewButton(QStringLiteral("Front"), QStringLiteral("Front (1)"), viewportCard);
    auto *backButton = makeViewButton(QStringLiteral("Back"), QStringLiteral("Back (2)"), viewportCard);
    auto *leftButton = makeViewButton(QStringLiteral("Left"), QStringLiteral("Left (3)"), viewportCard);
    auto *rightButton = makeViewButton(QStringLiteral("Right"), QStringLiteral("Right (4)"), viewportCard);
    auto *topButton = makeViewButton(QStringLiteral("Top"), QStringLiteral("Top (5)"), viewportCard);
    auto *bottomButton = makeViewButton(QStringLiteral("Bottom"), QStringLiteral("Bottom (6)"), viewportCard);
    projectionButton_ = makeViewButton(
        QStringLiteral("Perspective"),
        QStringLiteral("Toggle projection (P)"),
        viewportCard);
    auto *viewControls = new QHBoxLayout;
    viewControls->setSpacing(5);
    viewControls->addStretch();
    for (QToolButton *button :
         {fitButton, isoButton, frontButton, backButton, leftButton,
          rightButton, topButton, bottomButton, projectionButton_}) {
        viewControls->addWidget(button);
    }
    viewControls->addStretch();
    viewportLayout->addLayout(viewControls);

    viewport_ = new ParagliderView(viewportCard);
    viewportLayout->addWidget(viewport_, 1);
    mainSplitter->addWidget(viewportCard);
    mainSplitter->setStretchFactor(0, 5);
    mainSplitter->setStretchFactor(1, 6);
    mainSplitter->setSizes({660, 780});
    workspaceSplitter->addWidget(mainSplitter);

    diagnosticsTabs_ = new QTabWidget(workspaceSplitter);
    diagnosticsTabs_->setObjectName(QStringLiteral("diagnostics"));

    log_ = new QPlainTextEdit(diagnosticsTabs_);
    log_->setReadOnly(true);
    log_->setPlaceholderText(
        QStringLiteral("Preview and export progress appears here."));
    log_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    log_->setMaximumBlockCount(5000);
    diagnosticsTabs_->addTab(log_, QStringLiteral("Calculation log"));

    auto *outputsPage = new QWidget(diagnosticsTabs_);
    auto *outputsLayout = new QVBoxLayout(outputsPage);
    outputsLayout->setContentsMargins(8, 8, 8, 8);
    auto *outputsHeader = new QHBoxLayout;
    outputsHeader->addStretch();
    openFolderButton_ = new QPushButton(QStringLiteral("Open output folder"), outputsPage);
    openFolderButton_->setObjectName(QStringLiteral("quietButton"));
    outputsHeader->addWidget(openFolderButton_);
    outputsLayout->addLayout(outputsHeader);
    outputTree_ = new QTreeWidget(outputsPage);
    outputTree_->setHeaderLabels(
        {QStringLiteral("File"), QStringLiteral("Purpose"), QStringLiteral("Size"),
         QStringLiteral("Status")});
    outputTree_->setRootIsDecorated(false);
    outputTree_->setAlternatingRowColors(true);
    outputTree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    outputTree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    outputTree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    outputTree_->header()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    outputsLayout->addWidget(outputTree_);
    diagnosticsTabs_->addTab(outputsPage, QStringLiteral("Exported files"));
    workspaceSplitter->addWidget(diagnosticsTabs_);
    workspaceSplitter->setStretchFactor(0, 7);
    workspaceSplitter->setStretchFactor(1, 2);
    workspaceSplitter->setSizes({680, 190});
    page->addWidget(workspaceSplitter, 1);

    auto *footer = new QHBoxLayout;
    statusLabel_ = new QLabel(QStringLiteral("Ready"), central);
    statusLabel_->setObjectName(QStringLiteral("status"));
    footer->addWidget(statusLabel_);
    progressBar_ = new QProgressBar(central);
    progressBar_->setTextVisible(false);
    progressBar_->setFixedWidth(130);
    progressBar_->setRange(0, 1);
    progressBar_->setValue(0);
    footer->addWidget(progressBar_);
    footer->addStretch();
    page->addLayout(footer);

    setCentralWidget(central);
    setStyleSheet(QStringLiteral(R"(
        QMainWindow, QWidget {
            background: #0d1422;
            color: #e6edf7;
            font-family: "Segoe UI";
            font-size: 9.5pt;
        }
        QFrame#card {
            background: #151f30;
            border: 1px solid #26354a;
            border-radius: 10px;
        }
        QWidget#sectionNavigator {
            background: #111b2a;
            border-right: 1px solid #26354a;
            border-top-left-radius: 10px;
            border-bottom-left-radius: 10px;
        }
        QLabel#brandMark {
            background: #38bdf8;
            color: #07111e;
            border-radius: 12px;
            font-size: 15pt;
            font-weight: 800;
        }
        QLabel#title {
            font-size: 20pt;
            font-weight: 700;
            color: #f7fbff;
        }
        QLabel#subtitle, QLabel#hint, QLabel#emptyState {
            color: #93a4ba;
        }
        QLabel#editorHint {
            color: #70ccef;
            background: #102537;
            border: 1px solid #21445d;
            border-radius: 5px;
            padding: 5px 8px;
        }
        QLabel#badge {
            background: #132d3a;
            color: #67d3ff;
            border: 1px solid #24536a;
            border-radius: 8px;
            padding: 5px 9px;
            font-weight: 600;
        }
        QLabel#sectionTitle {
            color: #f7fbff;
            font-size: 11pt;
            font-weight: 650;
        }
        QLabel#fieldLabel, QLabel#status {
            color: #b9c6d8;
            font-weight: 600;
        }
        QLineEdit, QPlainTextEdit, QTreeWidget, QListWidget, QTextBrowser {
            background: #0e1726;
            border: 1px solid #2a3a50;
            border-radius: 6px;
            color: #e6edf7;
            selection-background-color: #176b91;
        }
        QLineEdit {
            padding: 7px 9px;
        }
        QLineEdit:focus, QPlainTextEdit:focus, QTreeWidget:focus, QListWidget:focus {
            border-color: #38bdf8;
        }
        QPlainTextEdit {
            padding: 8px;
            color: #d8e4f1;
        }
        QListWidget#sectionList {
            border: none;
            background: transparent;
            outline: none;
        }
        QListWidget#sectionList::item {
            color: #afbed1;
            padding: 7px 8px;
            border-radius: 5px;
        }
        QListWidget#sectionList::item:selected {
            color: #f7fbff;
            background: #1f5571;
        }
        QTreeWidget {
            alternate-background-color: #111b2a;
            outline: none;
        }
        QHeaderView::section {
            background: #1b293c;
            color: #aebdd0;
            border: none;
            border-bottom: 1px solid #304158;
            padding: 6px;
            font-weight: 600;
        }
        QTabWidget::pane {
            border: 1px solid #26354a;
            background: #111b2a;
            border-radius: 6px;
        }
        QTabBar::tab {
            background: #162236;
            color: #91a4ba;
            padding: 6px 13px;
            border: 1px solid #26354a;
        }
        QTabBar::tab:selected {
            background: #21334a;
            color: #edf5ff;
        }
        QSplitter::handle {
            background: #0d1422;
            width: 6px;
            height: 6px;
        }
        QPushButton, QToolButton {
            border-radius: 6px;
            padding: 7px 11px;
            font-weight: 600;
        }
        QPushButton#primaryButton {
            background: #38bdf8;
            color: #07111e;
            border: 1px solid #54c8f7;
            padding: 9px 15px;
        }
        QPushButton#primaryButton:hover {
            background: #63cdf8;
        }
        QPushButton#primaryButton:disabled, QPushButton#secondaryButton:disabled {
            background: #314052;
            color: #77869a;
            border-color: #3a4a5d;
        }
        QPushButton#secondaryButton {
            background: #223149;
            border: 1px solid #354a66;
            color: #dce7f5;
        }
        QPushButton#quietButton {
            background: transparent;
            border: 1px solid #354a66;
            color: #bcd0e6;
            padding: 5px 10px;
        }
        QToolButton#viewButton {
            background: #1b2a3e;
            border: 1px solid #344b67;
            color: #c5d6e9;
            padding: 4px 7px;
            min-width: 30px;
        }
        QPushButton#secondaryButton:hover, QPushButton#quietButton:hover,
        QToolButton#viewButton:hover {
            background: #2a3d59;
        }
        QProgressBar {
            background: #172235;
            border: 1px solid #2c3c51;
            border-radius: 4px;
            height: 7px;
        }
        QProgressBar::chunk {
            background: #38bdf8;
            border-radius: 3px;
        }
        QScrollBar:vertical {
            background: #111a28;
            width: 10px;
        }
        QScrollBar::handle:vertical {
            background: #33455d;
            border-radius: 5px;
            min-height: 24px;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0;
        }
    )"));

    connect(inputBrowseButton_, &QPushButton::clicked, this, [this] {
        browseForInput();
    });
    connect(manualButton, &QPushButton::clicked, this, [] {
        QDesktopServices::openUrl(QUrl(QString::fromLatin1(manualUrl)));
    });
    connect(outputBrowseButton_, &QPushButton::clicked, this, [this] {
        browseForOutput();
    });
    connect(historyButton_, &QPushButton::clicked, this, [this] {
        showVersionHistory();
    });
    connect(saveButton_, &QPushButton::clicked, this, [this] { saveDesign(); });
    connect(buildButton_, &QPushButton::clicked, this, [this] {
        startPreviewCalculation();
    });
    connect(exportButton_, &QPushButton::clicked, this, [this] {
        startExportCalculation();
    });
    connect(sectionList_, &QListWidget::currentRowChanged,
            sectionPages_, &QStackedWidget::setCurrentIndex);
    connect(outputEdit_, &QLineEdit::textChanged, this, [this] {
        refreshOutputFiles();
        updateRunAvailability();
    });
    connect(openFolderButton_, &QPushButton::clicked, this, [this] {
        if (!outputEdit_->text().isEmpty()) {
            QDesktopServices::openUrl(
                QUrl::fromLocalFile(QDir(outputEdit_->text()).absolutePath()));
        }
    });
    connect(outputTree_, &QTreeWidget::itemDoubleClicked, this,
            [this](QTreeWidgetItem *item) { openOutputItem(item); });

    connect(fitButton, &QToolButton::clicked, viewport_, &ParagliderView::fitAll);
    connect(isoButton, &QToolButton::clicked, this, [this] {
        viewport_->setView(ParagliderView::ViewPreset::Isometric);
        viewport_->fitAll();
    });
    connect(frontButton, &QToolButton::clicked, this, [this] {
        viewport_->setView(ParagliderView::ViewPreset::Front);
        viewport_->fitAll();
    });
    connect(backButton, &QToolButton::clicked, this, [this] {
        viewport_->setView(ParagliderView::ViewPreset::Back);
        viewport_->fitAll();
    });
    connect(leftButton, &QToolButton::clicked, this, [this] {
        viewport_->setView(ParagliderView::ViewPreset::Left);
        viewport_->fitAll();
    });
    connect(rightButton, &QToolButton::clicked, this, [this] {
        viewport_->setView(ParagliderView::ViewPreset::Right);
        viewport_->fitAll();
    });
    connect(topButton, &QToolButton::clicked, this, [this] {
        viewport_->setView(ParagliderView::ViewPreset::Top);
        viewport_->fitAll();
    });
    connect(bottomButton, &QToolButton::clicked, this, [this] {
        viewport_->setView(ParagliderView::ViewPreset::Bottom);
        viewport_->fitAll();
    });
    connect(projectionButton_, &QToolButton::clicked, this, [this] {
        viewport_->toggleProjection();
        projectionButton_->setText(
            viewport_->isPerspective()
                ? QStringLiteral("Perspective")
                : QStringLiteral("Orthographic"));
    });
}

void MainWindow::connectProcess()
{
    process_->setProcessChannelMode(QProcess::MergedChannels);
    connect(process_, &QProcess::readyReadStandardOutput, this,
            [this] { appendProcessOutput(); });
    connect(process_, &QProcess::started, this, [this] {
        statusLabel_->setText(
            calculationMode_ == CalculationMode::Export
                ? QStringLiteral("Writing export files…")
                : QStringLiteral("Building 3D preview…"));
    });
    connect(process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) {
            log_->appendPlainText(
                QStringLiteral("Could not start the C++ calculation engine at:\n%1")
                    .arg(enginePath()));
            diagnosticsTabs_->setCurrentWidget(log_);
            setRunning(false);
            statusLabel_->setText(QStringLiteral("Engine could not start"));
            calculationDirectory_.reset();
            calculationOutputDirectory_.clear();
            calculationMode_ = CalculationMode::None;
        }
    });
    connect(process_,
            qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this,
            [this](int exitCode, QProcess::ExitStatus exitStatus) {
                calculationFinished(exitCode, exitStatus);
            });
}

void MainWindow::browseForInput()
{
    const QString initial =
        inputEdit_->text().isEmpty()
            ? QDir::currentPath()
            : QFileInfo(inputEdit_->text()).absolutePath();
    const QString file = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("Open LEparagliding design"),
        initial,
        QStringLiteral("LEparagliding design (*.txt);;All files (*.*)"));
    if (!file.isEmpty()) {
        loadDesign(file);
    }
}

void MainWindow::browseForOutput()
{
    const QString folder = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("Select output folder"),
        outputEdit_->text());
    if (!folder.isEmpty()) {
        outputEdit_->setText(QDir::toNativeSeparators(folder));
    }
}

bool MainWindow::loadDesign(const QString &path, bool confirmUnsaved)
{
    if (process_->state() != QProcess::NotRunning) {
        QMessageBox::information(
            this,
            QStringLiteral("Calculation in progress"),
            QStringLiteral("Wait for the current preview or export to finish "
                           "before opening another design."));
        return false;
    }
    if (confirmUnsaved && !maybeSaveChanges()) {
        return false;
    }

    QString error;
    if (!document_.load(path, &error)) {
        QMessageBox::critical(
            this,
            QStringLiteral("Could not open design"),
            QStringLiteral("%1\n\n%2").arg(QDir::toNativeSeparators(path), error));
        return false;
    }

    const QFileInfo info(document_.filePath());
    inputEdit_->setText(QDir::toNativeSeparators(info.absoluteFilePath()));
    if (outputEdit_->text().trimmed().isEmpty()) {
        outputEdit_->setText(QDir::toNativeSeparators(info.absolutePath()));
    }
    refreshInputDetails();

    rebuildSectionEditors();
    documentDirty_ = false;
    dirtySections_.clear();
    refreshSectionLabels();
    saveButton_->setEnabled(false);
    historyButton_->setEnabled(true);
    viewport_->clearModel();
    modelStats_->setText(QStringLiteral("Preparing current design preview…"));
    statusLabel_->setText(QStringLiteral("Design loaded · preview queued"));
    updateWindowTitle();
    updateRunAvailability();
    saveSettings();

    const QString loadedPath = document_.filePath();
    QTimer::singleShot(0, this, [this, loadedPath] {
        if (document_.filePath() == loadedPath
            && process_->state() == QProcess::NotRunning) {
            startPreviewCalculation(true);
        }
    });
    return true;
}

void MainWindow::rebuildSectionEditors()
{
    loadingEditors_ = true;
    sectionList_->clear();
    sectionEditors_.clear();
    savedSectionTexts_.clear();
    undoButtons_.clear();
    redoButtons_.clear();
    persistedSectionHistories_.clear();
    persistedSectionHistoryPositions_.clear();
    while (sectionPages_->count() > 0) {
        QWidget *page = sectionPages_->widget(0);
        sectionPages_->removeWidget(page);
        delete page;
    }

    for (qsizetype index = 0; index < document_.sections().size(); ++index) {
        const DesignSection &section = document_.sections().at(index);
        auto *item = new QListWidgetItem(sectionList_);
        item->setData(Qt::UserRole, section.number);

        auto *sectionPage = new QWidget(sectionPages_);
        auto *layout = new QVBoxLayout(sectionPage);
        layout->setContentsMargins(16, 14, 16, 14);
        layout->setSpacing(9);

        auto *header = new QHBoxLayout;
        auto *sectionTitle = new QLabel(
            QStringLiteral("Section %1 · %2").arg(section.number).arg(section.title),
            sectionPage);
        sectionTitle->setObjectName(QStringLiteral("sectionTitle"));
        header->addWidget(sectionTitle);
        header->addStretch();
        auto *lineCount = new QLabel(sectionPage);
        lineCount->setObjectName(QStringLiteral("hint"));
        header->addWidget(lineCount);
        auto *helpButton = new QPushButton(QStringLiteral("?"), sectionPage);
        helpButton->setObjectName(QStringLiteral("quietButton"));
        helpButton->setToolTip(QStringLiteral("Explain this section"));
        helpButton->setFixedWidth(34);
        header->addWidget(helpButton);
        layout->addLayout(header);

        const SectionHelp help = helpForSection(section.number, section.title);
        auto *summary = new QLabel(help.purpose, sectionPage);
        summary->setObjectName(QStringLiteral("hint"));
        summary->setWordWrap(true);
        layout->addWidget(summary);
        auto *editorHint = new QLabel(
            QStringLiteral(
                "Enter rebuilds the 3D preview · Shift+Enter inserts a record · "
                "Undo/Redo is independent per section and survives Save/restart"),
            sectionPage);
        editorHint->setObjectName(QStringLiteral("editorHint"));
        editorHint->setWordWrap(true);
        layout->addWidget(editorHint);

        auto *editor = new DesignSectionEditor(sectionPage);
        editor->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        editor->setLineWrapMode(QPlainTextEdit::NoWrap);
        editor->setTabStopDistance(
            QFontMetricsF(editor->font()).horizontalAdvance(QLatin1Char(' ')) * 4.0);
        editor->setPlainText(section.text);
        editor->setUndoRedoEnabled(true);
        editor->document()->clearUndoRedoStacks();
        editor->setToolTip(
            QStringLiteral("Enter: rebuild 3D preview · Shift+Enter: insert a new record"));
        editor->buildRequested = [this] { startPreviewCalculation(); };
        new DesignSyntaxHighlighter(editor->document());
        layout->addWidget(editor, 1);

        auto updateLineCount = [editor, lineCount] {
            lineCount->setText(
                QStringLiteral("%1 lines").arg(editor->document()->blockCount()));
        };
        updateLineCount();

        const int editorIndex = static_cast<int>(index);
        editor->persistedUndoRequested = [this, editorIndex] {
            undoSection(editorIndex);
        };
        editor->persistedRedoRequested = [this, editorIndex] {
            redoSection(editorIndex);
        };
        connect(helpButton, &QPushButton::clicked, this,
                [this, editorIndex] { showSectionHelp(editorIndex); });
        auto *undoButton = new QPushButton(QStringLiteral("Undo"), sectionPage);
        undoButton->setObjectName(QStringLiteral("quietButton"));
        undoButton->setToolTip(
            QStringLiteral("Undo in this section only (Ctrl+Z)"));
        undoButton->setEnabled(false);
        header->insertWidget(header->count() - 1, undoButton);
        auto *redoButton = new QPushButton(QStringLiteral("Redo"), sectionPage);
        redoButton->setObjectName(QStringLiteral("quietButton"));
        redoButton->setToolTip(
            QStringLiteral("Redo in this section only (Ctrl+Y)"));
        redoButton->setEnabled(false);
        header->insertWidget(header->count() - 1, redoButton);
        connect(undoButton, &QPushButton::clicked, this,
                [this, editorIndex] { undoSection(editorIndex); });
        connect(redoButton, &QPushButton::clicked, this,
                [this, editorIndex] { redoSection(editorIndex); });
        connect(editor, &QPlainTextEdit::undoAvailable,
                this, [this, editorIndex](bool) {
                    updateUndoRedoAvailability(editorIndex);
                });
        connect(editor, &QPlainTextEdit::redoAvailable,
                this, [this, editorIndex](bool) {
                    updateUndoRedoAvailability(editorIndex);
                });
        connect(editor, &QPlainTextEdit::textChanged, this,
                [this, editor, editorIndex, updateLineCount] {
                    updateLineCount();
                    if (loadingEditors_) {
                        return;
                    }
                    const QString text = editor->toPlainText();
                    document_.setSectionText(editorIndex, text);
                    if (savedSectionTexts_.value(editorIndex) == text) {
                        dirtySections_.remove(editorIndex);
                    } else {
                        dirtySections_.insert(editorIndex);
                    }
                    documentDirty_ = !dirtySections_.isEmpty();
                    saveButton_->setEnabled(
                        documentDirty_
                        && process_->state() == QProcess::NotRunning);
                    refreshSectionLabels();
                    updateWindowTitle();
                    updateUndoRedoAvailability(editorIndex);
                });

        sectionEditors_.append(editor);
        const QString savedText = document_.savedSectionText(section.number);
        savedSectionTexts_.append(savedText.isNull() ? section.text : savedText);
        undoButtons_.append(undoButton);
        redoButtons_.append(redoButton);
        sectionPages_->addWidget(sectionPage);
    }

    loadingEditors_ = false;
    syncPersistedSectionHistories();
    dirtySections_.clear();
    for (qsizetype index = 0; index < sectionEditors_.size(); ++index) {
        if (sectionEditors_.at(index)->toPlainText()
            != savedSectionTexts_.value(index)) {
            dirtySections_.insert(static_cast<int>(index));
        }
    }
    documentDirty_ = !dirtySections_.isEmpty();
    saveButton_->setEnabled(
        documentDirty_ && process_->state() == QProcess::NotRunning);
    refreshSectionLabels();
    if (!document_.sections().isEmpty()) {
        sectionList_->setCurrentRow(0);
    }
}

bool MainWindow::saveDesign(bool showConfirmation)
{
    for (qsizetype index = 0; index < sectionEditors_.size(); ++index) {
        document_.setSectionText(index, sectionEditors_.at(index)->toPlainText());
    }

    QString error;
    if (!document_.save(&error)) {
        QMessageBox::warning(
            this,
            QStringLiteral("Design not saved"),
            error);
        return false;
    }

    documentDirty_ = false;
    dirtySections_.clear();
    savedSectionTexts_.clear();
    for (QPlainTextEdit *editor : std::as_const(sectionEditors_)) {
        savedSectionTexts_.append(editor->toPlainText());
        editor->document()->clearUndoRedoStacks();
    }
    syncPersistedSectionHistories();
    refreshSectionLabels();
    refreshInputDetails();
    saveButton_->setEnabled(false);
    historyButton_->setEnabled(true);
    updateWindowTitle();
    if (showConfirmation) {
        statusLabel_->setText(
            QStringLiteral("Design saved · version %1 embedded")
                .arg(document_.revisionCount()));
    }
    return true;
}

bool MainWindow::maybeSaveChanges()
{
    if (!documentDirty_) {
        return true;
    }

    const QMessageBox::StandardButton answer = QMessageBox::question(
        this,
        QStringLiteral("Unsaved design changes"),
        QStringLiteral("Save changes to %1?")
            .arg(QFileInfo(document_.filePath()).fileName()),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
        QMessageBox::Save);
    if (answer == QMessageBox::Cancel) {
        return false;
    }
    if (answer == QMessageBox::Save) {
        return saveDesign(false);
    }
    return true;
}

void MainWindow::showVersionHistory()
{
    if (document_.isEmpty()) {
        return;
    }

    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("Embedded wing versions"));
    dialog.resize(760, 520);
    auto *layout = new QVBoxLayout(&dialog);

    auto *explanation = new QLabel(
        QStringLiteral(
            "Every saved snapshot lives inside this design file. Restoring a version "
            "loads it into the editors without deleting later history; press Save to "
            "make the restored wing the newest version."),
        &dialog);
    explanation->setWordWrap(true);
    explanation->setObjectName(QStringLiteral("hint"));
    layout->addWidget(explanation);

    auto *versions = new QListWidget(&dialog);
    const QList<DesignRevision> revisions = document_.revisions();
    for (int index = revisions.size() - 1; index >= 0; --index) {
        const DesignRevision &revision = revisions.at(index);
        const QString timestamp =
            revision.savedAt.toLocalTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
        auto *item = new QListWidgetItem(
            QStringLiteral("v%1 · %2 · %3")
                .arg(index + 1)
                .arg(timestamp)
                .arg(revision.summary),
            versions);
        item->setData(Qt::UserRole, index);
    }
    layout->addWidget(versions, 1);

    auto *details = new QTextBrowser(&dialog);
    details->setMaximumHeight(125);
    layout->addWidget(details);

    const auto updateDetails = [details, revisions](QListWidgetItem *item) {
        if (item == nullptr) {
            details->clear();
            return;
        }
        const int index = item->data(Qt::UserRole).toInt();
        const DesignRevision &revision = revisions.at(index);
        QStringList changed;
        for (const int section : revision.changedSections) {
            changed.append(QString::number(section));
        }
        details->setHtml(
            QStringLiteral(
                "<b>Version %1</b><br>"
                "Saved: %2<br>"
                "Commit: <code>%3</code><br>"
                "Parent: <code>%4</code><br>"
                "Changed sections: %5")
                .arg(index + 1)
                .arg(revision.savedAt.toLocalTime().toString(Qt::ISODate))
                .arg(revision.id.left(16).toHtmlEscaped())
                .arg(revision.parentId.isEmpty()
                         ? QStringLiteral("root")
                         : revision.parentId.left(16).toHtmlEscaped())
                .arg(changed.isEmpty()
                         ? QStringLiteral("initial snapshot")
                         : changed.join(QStringLiteral(", "))));
    };
    connect(versions, &QListWidget::currentItemChanged, &dialog,
            [updateDetails](QListWidgetItem *current, QListWidgetItem *) {
                updateDetails(current);
            });

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    auto *restore =
        buttons->addButton(QStringLiteral("Restore selected"), QDialogButtonBox::AcceptRole);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    connect(restore, &QPushButton::clicked, &dialog, [&dialog, versions, this] {
        QListWidgetItem *item = versions->currentItem();
        if (item == nullptr) {
            return;
        }
        const int revisionIndex = item->data(Qt::UserRole).toInt();
        dialog.accept();
        restoreVersion(revisionIndex);
    });
    layout->addWidget(buttons);

    if (versions->count() > 0) {
        versions->setCurrentRow(0);
    }
    dialog.exec();
}

void MainWindow::restoreVersion(int revisionIndex)
{
    const QMessageBox::StandardButton answer = QMessageBox::question(
        this,
        QStringLiteral("Restore embedded version"),
        QStringLiteral(
            "Replace every editor with version %1?\n\n"
            "Unsaved edits will be discarded. Existing versions remain in the file, "
            "and Save will add the restored wing as a new version.")
            .arg(revisionIndex + 1),
        QMessageBox::Yes | QMessageBox::Cancel,
        QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) {
        return;
    }

    QString error;
    if (!document_.restoreRevision(revisionIndex, &error)) {
        QMessageBox::warning(
            this,
            QStringLiteral("Version could not be restored"),
            error);
        return;
    }

    rebuildSectionEditors();
    statusLabel_->setText(
        QStringLiteral("Version %1 restored · rebuilding preview")
            .arg(revisionIndex + 1));
    viewport_->clearModel();
    modelStats_->setText(QStringLiteral("Building restored version preview…"));
    updateWindowTitle();
    QTimer::singleShot(0, this, [this] {
        if (process_->state() == QProcess::NotRunning) {
            startPreviewCalculation(true);
        }
    });
}

void MainWindow::syncPersistedSectionHistories()
{
    persistedSectionHistories_.clear();
    persistedSectionHistoryPositions_.clear();
    for (qsizetype index = 0; index < document_.sections().size(); ++index) {
        int position = -1;
        QStringList history =
            document_.sectionHistory(document_.sections().at(index).number, &position);
        if (history.isEmpty()) {
            history.append(document_.sections().at(index).text);
            position = 0;
        }
        persistedSectionHistories_.append(std::move(history));
        persistedSectionHistoryPositions_.append(position);
    }
    for (qsizetype index = 0; index < sectionEditors_.size(); ++index) {
        updateUndoRedoAvailability(static_cast<int>(index));
    }
}

void MainWindow::undoSection(int index)
{
    if (index < 0 || index >= sectionEditors_.size()
        || process_->state() != QProcess::NotRunning) {
        return;
    }

    QPlainTextEdit *editor = sectionEditors_.at(index);
    if (editor->document()->isUndoAvailable()) {
        editor->undo();
        return;
    }

    const QStringList &history = persistedSectionHistories_.at(index);
    int &position = persistedSectionHistoryPositions_[index];
    if (position <= 0 || editor->toPlainText() != history.value(position)) {
        updateUndoRedoAvailability(index);
        return;
    }

    --position;
    editor->setPlainText(history.at(position));
    editor->document()->clearUndoRedoStacks();
    updateUndoRedoAvailability(index);
}

void MainWindow::redoSection(int index)
{
    if (index < 0 || index >= sectionEditors_.size()
        || process_->state() != QProcess::NotRunning) {
        return;
    }

    QPlainTextEdit *editor = sectionEditors_.at(index);
    if (editor->document()->isRedoAvailable()) {
        editor->redo();
        return;
    }

    const QStringList &history = persistedSectionHistories_.at(index);
    int &position = persistedSectionHistoryPositions_[index];
    if (editor->document()->isUndoAvailable()
        || position + 1 >= history.size()
        || editor->toPlainText() != history.value(position)) {
        updateUndoRedoAvailability(index);
        return;
    }

    ++position;
    editor->setPlainText(history.at(position));
    editor->document()->clearUndoRedoStacks();
    updateUndoRedoAvailability(index);
}

void MainWindow::updateUndoRedoAvailability(int index)
{
    if (index < 0 || index >= sectionEditors_.size()
        || index >= undoButtons_.size() || index >= redoButtons_.size()
        || index >= persistedSectionHistories_.size()
        || index >= persistedSectionHistoryPositions_.size()) {
        return;
    }

    QPlainTextEdit *editor = sectionEditors_.at(index);
    const QStringList &history = persistedSectionHistories_.at(index);
    const int position = persistedSectionHistoryPositions_.at(index);
    const bool running = process_->state() != QProcess::NotRunning;
    const bool atPersistedState =
        position >= 0 && position < history.size()
        && editor->toPlainText() == history.at(position);

    undoButtons_.at(index)->setEnabled(
        !running
        && (editor->document()->isUndoAvailable()
            || (atPersistedState && position > 0)));
    redoButtons_.at(index)->setEnabled(
        !running
        && (editor->document()->isRedoAvailable()
            || (!editor->document()->isUndoAvailable()
                && atPersistedState
                && position + 1 < history.size())));
}

void MainWindow::refreshInputDetails()
{
    if (document_.filePath().isEmpty()) {
        inputDetails_->setText(
            QStringLiteral("Open a design to create its section editors."));
        return;
    }

    const QFileInfo info(document_.filePath());
    inputDetails_->setText(
        QStringLiteral("%1 sections · %2 embedded version%3 · %4 · modified %5")
            .arg(document_.sections().size())
            .arg(document_.revisionCount())
            .arg(document_.revisionCount() == 1 ? QString() : QStringLiteral("s"))
            .arg(humanReadableSize(info.size()))
            .arg(info.lastModified().toString(QStringLiteral("yyyy-MM-dd HH:mm"))));
}

void MainWindow::showSectionHelp(int index)
{
    if (index < 0 || index >= document_.sections().size()) {
        return;
    }
    const DesignSection &section = document_.sections().at(index);
    const SectionHelp help = helpForSection(section.number, section.title);
    QString details = help.details;
    if (section.number == 26) {
        details += glueVentRowsHtml(sectionEditors_.at(index)->toPlainText());
    }

    QDialog dialog(this);
    dialog.setWindowTitle(
        QStringLiteral("Section %1 · %2").arg(section.number).arg(help.title));
    dialog.resize(620, 440);
    auto *layout = new QVBoxLayout(&dialog);

    auto *browser = new QTextBrowser(&dialog);
    browser->setOpenExternalLinks(true);
    browser->setHtml(
        QStringLiteral(
            "<h2>Section %1 · %2</h2>"
            "<h3>Purpose</h3><p>%3</p>"
            "<h3>Format rules</h3><p>%4</p>"
            "<h3>Editing notes</h3><p>%5</p>"
            "%6"
            "%7"
            "<p><a href=\"%8\">Open the complete LEparagliding manual</a></p>")
            .arg(section.number)
            .arg(help.title.toHtmlEscaped())
            .arg(help.purpose)
            .arg(help.format)
            .arg(help.notes)
            .arg(details.isEmpty()
                     ? QString()
                     : QStringLiteral("<h3>Field reference</h3>%1").arg(details))
            .arg(help.experiment.isEmpty()
                     ? QString()
                     : QStringLiteral("<h3>Try it</h3>%1").arg(help.experiment))
            .arg(QString::fromLatin1(manualUrl)));
    layout->addWidget(browser);

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);
    dialog.exec();
}

void MainWindow::startPreviewCalculation(bool automatic)
{
    startCalculation(CalculationMode::Preview, automatic);
}

void MainWindow::startExportCalculation()
{
    startCalculation(CalculationMode::Export, false);
}

void MainWindow::startCalculation(
    CalculationMode mode,
    bool automatic)
{
    if (process_->state() != QProcess::NotRunning) {
        return;
    }

    const auto reportProblem =
        [this, automatic](const QString &title, const QString &message) {
            statusLabel_->setText(title);
            log_->appendPlainText(QStringLiteral("\n%1").arg(message));
            if (!automatic) {
                QMessageBox::warning(this, title, message);
            }
        };

    if (document_.isEmpty() || !QFileInfo(document_.filePath()).isFile()) {
        reportProblem(
            QStringLiteral("No design loaded"),
            QStringLiteral("Open a LEparagliding design before building."));
        return;
    }

    const QString invalid = document_.validationError();
    if (!invalid.isEmpty()) {
        reportProblem(QStringLiteral("Design is not valid"), invalid);
        return;
    }

    QString outputDirectory;
    if (mode == CalculationMode::Export) {
        outputDirectory = QDir(outputEdit_->text()).absolutePath();
        if (outputEdit_->text().trimmed().isEmpty()
            || !QDir().mkpath(outputDirectory)) {
            reportProblem(
                QStringLiteral("Output folder unavailable"),
                QStringLiteral("Select a writable output folder before exporting."));
            return;
        }
    }

    if (!QFileInfo::exists(enginePath())) {
        reportProblem(
            QStringLiteral("Calculation engine missing"),
            QStringLiteral("The C++ engine was not found next to the application:\n%1")
                .arg(enginePath()));
        return;
    }

    auto calculationDirectory = std::make_unique<QTemporaryDir>(
        QDir::tempPath()
        + QStringLiteral("/LEparagliding-preview-XXXXXX"));
    if (!calculationDirectory->isValid()) {
        reportProblem(
            QStringLiteral("Temporary preview unavailable"),
            QStringLiteral("A temporary calculation folder could not be created."));
        return;
    }

    const QString temporaryInput =
        calculationDirectory->filePath(QStringLiteral("leparagliding.txt"));
    QSaveFile input(temporaryInput);
    const QByteArray designText = document_.assembledText().toUtf8();
    if (!input.open(QIODevice::WriteOnly)
        || input.write(designText) != designText.size()
        || !input.commit()) {
        reportProblem(
            QStringLiteral("Temporary preview unavailable"),
            QStringLiteral("The current editor state could not be prepared:\n%1")
                .arg(input.errorString()));
        return;
    }

    if (mode == CalculationMode::Preview) {
        outputDirectory =
            calculationDirectory->filePath(QStringLiteral("output"));
        if (!QDir().mkpath(outputDirectory)) {
            reportProblem(
                QStringLiteral("Temporary preview unavailable"),
                QStringLiteral("The temporary preview output folder could not be created."));
            return;
        }
        viewport_->clearModel();
        modelStats_->setText(QStringLiteral("Building current editor preview…"));
    }

    calculationMode_ = mode;
    calculationOutputDirectory_ = outputDirectory;
    calculationDirectory_ = std::move(calculationDirectory);

    saveSettings();
    log_->clear();
    log_->appendPlainText(
        QStringLiteral("%1\nDesign resources: %2\nTemporary input: %3\nOutput: %4\n")
            .arg(mode == CalculationMode::Preview
                     ? QStringLiteral("Temporary 3D preview")
                     : QStringLiteral("Explicit file export"),
                 QFileInfo(document_.filePath()).absolutePath(),
                 temporaryInput,
                 outputDirectory));
    if (!automatic) {
        diagnosticsTabs_->setCurrentWidget(log_);
    }
    setRunning(true);

    process_->setProgram(enginePath());
    process_->setArguments(
        {QStringLiteral("--resource-dir"),
         QFileInfo(document_.filePath()).absolutePath(),
         temporaryInput,
         outputDirectory});
    process_->setWorkingDirectory(QFileInfo(document_.filePath()).absolutePath());
    process_->start();
}

void MainWindow::appendProcessOutput()
{
    const QString text = QString::fromLocal8Bit(process_->readAllStandardOutput());
    log_->moveCursor(QTextCursor::End);
    log_->insertPlainText(text);
    log_->verticalScrollBar()->setValue(log_->verticalScrollBar()->maximum());
}

void MainWindow::calculationFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    appendProcessOutput();

    const CalculationMode completedMode = calculationMode_;
    const QString completedOutput = calculationOutputDirectory_;
    const QString modelPath =
        QDir(completedOutput).filePath(QStringLiteral("lep-3d.step"));
    const bool engineSucceeded =
        exitStatus == QProcess::NormalExit && exitCode == 0;

    if (completedMode == CalculationMode::Preview) {
        const bool success =
            engineSucceeded
            && QFileInfo::exists(modelPath)
            && loadViewportModel(modelPath);
        if (success) {
            statusLabel_->setText(
                QStringLiteral("Preview ready · %1%2")
                    .arg(
                        viewport_->modelSummary(),
                        documentDirty_
                            ? QStringLiteral(" · unsaved edits")
                            : QString()));
        } else {
            statusLabel_->setText(
                QStringLiteral("Preview failed · exit %1").arg(exitCode));
            log_->appendPlainText(
                QStringLiteral(
                    "\nThe preview did not complete. Check the section data, "
                    "referenced airfoil files, and last engine message above."));
            diagnosticsTabs_->setCurrentWidget(log_);
        }
    } else if (completedMode == CalculationMode::Export) {
        refreshOutputFiles();

        int generatedCount = 0;
        for (const auto &output : outputs) {
            if (QFileInfo::exists(
                    QDir(completedOutput).filePath(
                        QString::fromLatin1(output.fileName)))) {
                ++generatedCount;
            }
        }

        const bool success =
            engineSucceeded
            && generatedCount == static_cast<int>(outputs.size());
        if (success) {
            const bool modelLoaded = loadViewportModel(modelPath);
            statusLabel_->setText(
                QStringLiteral("Export completed · %1 files%2%3")
                    .arg(generatedCount)
                    .arg(documentDirty_
                             ? QStringLiteral(" · unsaved edits remain")
                             : QString())
                    .arg(modelLoaded
                             ? QString()
                             : QStringLiteral(" · preview unavailable")));
        } else {
            statusLabel_->setText(
                QStringLiteral("Export failed · exit %1 · %2/%3 files")
                    .arg(exitCode)
                    .arg(generatedCount)
                    .arg(static_cast<int>(outputs.size())));
            log_->appendPlainText(
                QStringLiteral(
                    "\nThe export did not complete. Check the section data, "
                    "referenced airfoil files, and last engine message above."));
            diagnosticsTabs_->setCurrentWidget(log_);
        }
    }

    calculationDirectory_.reset();
    calculationOutputDirectory_.clear();
    calculationMode_ = CalculationMode::None;
    setRunning(false);
}

void MainWindow::refreshOutputFiles()
{
    outputTree_->clear();
    for (const auto &output : outputs) {
        const QString fileName = QString::fromLatin1(output.fileName);
        const QFileInfo info(outputPathFor(fileName));
        auto *item = new QTreeWidgetItem(outputTree_);
        item->setText(0, fileName);
        item->setText(1, QString::fromLatin1(output.description));
        item->setText(2, info.isFile() ? humanReadableSize(info.size()) : QStringLiteral("—"));
        item->setText(3, info.isFile() ? QStringLiteral("Ready") : QStringLiteral("Pending"));
        item->setData(0, Qt::UserRole, info.absoluteFilePath());
        item->setForeground(
            3,
            info.isFile() ? QColor(QStringLiteral("#56d7a0"))
                          : QColor(QStringLiteral("#8494a9")));
    }

    openFolderButton_->setEnabled(
        !outputEdit_->text().isEmpty() && QDir(outputEdit_->text()).exists());
}

void MainWindow::openOutputItem(QTreeWidgetItem *item)
{
    if (item == nullptr) {
        return;
    }
    const QString path = item->data(0, Qt::UserRole).toString();
    if (QFileInfo::exists(path)) {
        QDesktopServices::openUrl(QUrl::fromLocalFile(path));
    }
}

bool MainWindow::loadViewportModel(const QString &path)
{
    if (!QFileInfo(path).isFile()) {
        viewport_->clearModel();
        modelStats_->setText(QStringLiteral("No model loaded"));
        return false;
    }

    QString error;
    if (!viewport_->loadStep(path, &error)) {
        viewport_->clearModel();
        modelStats_->setText(QStringLiteral("Model could not be loaded"));
        log_->appendPlainText(
            QStringLiteral("\n3D viewport: %1").arg(error));
        return false;
    }
    modelStats_->setText(viewport_->modelSummary());
    return true;
}

void MainWindow::setRunning(bool running)
{
    inputEdit_->setEnabled(!running);
    outputEdit_->setEnabled(!running);
    inputBrowseButton_->setEnabled(!running);
    outputBrowseButton_->setEnabled(!running);
    sectionList_->setEnabled(!running);
    for (qsizetype index = 0; index < sectionEditors_.size(); ++index) {
        QPlainTextEdit *editor = sectionEditors_.at(index);
        editor->setReadOnly(running);
        if (running) {
            if (index < undoButtons_.size()) {
                undoButtons_.at(index)->setEnabled(false);
            }
            if (index < redoButtons_.size()) {
                redoButtons_.at(index)->setEnabled(false);
            }
        } else {
            updateUndoRedoAvailability(static_cast<int>(index));
        }
    }
    historyButton_->setEnabled(!running && !document_.isEmpty());
    saveButton_->setEnabled(!running && documentDirty_);
    buildButton_->setEnabled(!running);
    exportButton_->setEnabled(!running);
    progressBar_->setRange(0, running ? 0 : 1);
    progressBar_->setValue(running ? 0 : 1);
    if (!running) {
        updateRunAvailability();
    }
}

void MainWindow::updateRunAvailability()
{
    const bool ready =
        !document_.isEmpty()
        && QFileInfo(document_.filePath()).isFile()
        && process_->state() == QProcess::NotRunning;
    buildButton_->setEnabled(ready);
    exportButton_->setEnabled(
        ready && !outputEdit_->text().trimmed().isEmpty());
}

void MainWindow::updateWindowTitle()
{
    const QString fileName =
        document_.filePath().isEmpty()
            ? QStringLiteral("No design")
            : QFileInfo(document_.filePath()).fileName();
    setWindowTitle(
        QStringLiteral("%1%2 — LEparagliding Studio")
            .arg(documentDirty_ ? QStringLiteral("● ") : QString())
            .arg(fileName));
}

void MainWindow::refreshSectionLabels()
{
    for (qsizetype index = 0;
         index < document_.sections().size() && index < sectionList_->count();
         ++index) {
        const DesignSection &section = document_.sections().at(index);
        sectionList_->item(index)->setText(
            QStringLiteral("%1%2 · %3")
                .arg(dirtySections_.contains(index) ? QStringLiteral("● ") : QString())
                .arg(section.number, 2, 10, QLatin1Char('0'))
                .arg(section.title));
    }
}

void MainWindow::loadSettings()
{
    QSettings settings;
    const QString input = settings.value(QStringLiteral("paths/input")).toString();
    const QString output = settings.value(QStringLiteral("paths/output")).toString();
    inputEdit_->setText(input);
    outputEdit_->setText(output);
    settings.remove(QStringLiteral("behavior/openWhenFinished"));
}

void MainWindow::saveSettings() const
{
    QSettings settings;
    settings.setValue(QStringLiteral("paths/input"), inputEdit_->text());
    settings.setValue(QStringLiteral("paths/output"), outputEdit_->text());
    settings.remove(QStringLiteral("behavior/openWhenFinished"));
}

QString MainWindow::enginePath() const
{
#ifdef Q_OS_WIN
    constexpr auto engineName = "leparagliding-engine.exe";
#else
    constexpr auto engineName = "leparagliding-engine";
#endif
    return QDir(QCoreApplication::applicationDirPath()).filePath(
        QString::fromLatin1(engineName));
}

QString MainWindow::outputPathFor(const QString &fileName) const
{
    if (outputEdit_ == nullptr || outputEdit_->text().trimmed().isEmpty()) {
        return {};
    }
    return QDir(outputEdit_->text()).absoluteFilePath(fileName);
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls() && !event->mimeData()->urls().isEmpty()
        && event->mimeData()->urls().constFirst().isLocalFile()) {
        event->acceptProposedAction();
    }
}

void MainWindow::dropEvent(QDropEvent *event)
{
    const auto urls = event->mimeData()->urls();
    if (!urls.isEmpty()) {
        const QFileInfo info(urls.constFirst().toLocalFile());
        if (info.isFile() && loadDesign(info.absoluteFilePath())) {
            event->acceptProposedAction();
        }
    }
}
