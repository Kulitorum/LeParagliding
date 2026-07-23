#include "mainwindow.h"

#include <QCheckBox>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDir>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDatabase>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMimeData>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollBar>
#include <QSettings>
#include <QStandardPaths>
#include <QTreeWidget>
#include <QUrl>
#include <QVBoxLayout>

#include <array>

namespace {

struct OutputDescription
{
    const char *fileName;
    const char *description;
};

constexpr std::array<OutputDescription, 4> outputs{{
    {"leparagliding.dxf", "2D manufacturing plans"},
    {"lep-3d.dxf", "3D wing geometry"},
    {"lep-out.txt", "Design calculations"},
    {"lines.txt", "Suspension line data"},
}};

QString humanReadableSize(qint64 bytes)
{
    constexpr qint64 kibibyte = 1024;
    constexpr qint64 mebibyte = kibibyte * 1024;

    if (bytes >= mebibyte) {
        return QStringLiteral("%1 MB").arg(bytes / static_cast<double>(mebibyte), 0, 'f', 1);
    }
    if (bytes >= kibibyte) {
        return QStringLiteral("%1 KB").arg(bytes / static_cast<double>(kibibyte), 0, 'f', 1);
    }
    return QStringLiteral("%1 B").arg(bytes);
}

QFrame *makeCard(QWidget *parent = nullptr)
{
    auto *card = new QFrame(parent);
    card->setObjectName(QStringLiteral("card"));
    return card;
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , process_(new QProcess(this))
{
    setAcceptDrops(true);
    setWindowTitle(QStringLiteral("LEparagliding 3.17"));
    setMinimumSize(900, 680);
    resize(1080, 780);

    buildInterface();
    connectProcess();
    loadSettings();
    refreshOutputFiles();
    updateRunAvailability();
}

MainWindow::~MainWindow()
{
    if (process_->state() != QProcess::NotRunning) {
        process_->kill();
        process_->waitForFinished(3000);
    }
    saveSettings();
}

void MainWindow::buildInterface()
{
    auto *central = new QWidget(this);
    auto *page = new QVBoxLayout(central);
    page->setContentsMargins(30, 26, 30, 26);
    page->setSpacing(18);

    auto *hero = new QHBoxLayout;
    hero->setSpacing(16);

    auto *mark = new QLabel(QStringLiteral("LE"), central);
    mark->setObjectName(QStringLiteral("brandMark"));
    mark->setAlignment(Qt::AlignCenter);
    mark->setFixedSize(58, 58);
    hero->addWidget(mark, 0, Qt::AlignTop);

    auto *titles = new QVBoxLayout;
    titles->setSpacing(3);
    auto *title = new QLabel(QStringLiteral("LEparagliding"), central);
    title->setObjectName(QStringLiteral("title"));
    auto *subtitle = new QLabel(
        QStringLiteral("Paraglider and parachute geometry · C++ edition of 3.17 “Z”"),
        central);
    subtitle->setObjectName(QStringLiteral("subtitle"));
    titles->addWidget(title);
    titles->addWidget(subtitle);
    hero->addLayout(titles, 1);

    auto *version = new QLabel(QStringLiteral("C++ / Qt 6"), central);
    version->setObjectName(QStringLiteral("badge"));
    auto *manualButton = new QPushButton(QStringLiteral("Open manual"), central);
    manualButton->setObjectName(QStringLiteral("quietButton"));
    hero->addWidget(manualButton, 0, Qt::AlignTop);
    hero->addWidget(version, 0, Qt::AlignTop);
    page->addLayout(hero);

    auto *setupCard = makeCard(central);
    auto *setup = new QGridLayout(setupCard);
    setup->setContentsMargins(20, 18, 20, 20);
    setup->setHorizontalSpacing(12);
    setup->setVerticalSpacing(10);
    setup->setColumnStretch(1, 1);

    auto *setupTitle = new QLabel(QStringLiteral("Design setup"), setupCard);
    setupTitle->setObjectName(QStringLiteral("sectionTitle"));
    setup->addWidget(setupTitle, 0, 0, 1, 3);

    auto *inputLabel = new QLabel(QStringLiteral("Design file"), setupCard);
    inputLabel->setObjectName(QStringLiteral("fieldLabel"));
    setup->addWidget(inputLabel, 1, 0);

    inputEdit_ = new QLineEdit(setupCard);
    inputEdit_->setPlaceholderText(QStringLiteral("Select leparagliding.txt or another design file"));
    inputEdit_->setClearButtonEnabled(true);
    setup->addWidget(inputEdit_, 1, 1);

    auto *inputBrowse = new QPushButton(QStringLiteral("Browse…"), setupCard);
    inputBrowse->setObjectName(QStringLiteral("secondaryButton"));
    setup->addWidget(inputBrowse, 1, 2);

    inputDetails_ = new QLabel(QStringLiteral("Drop a design file anywhere in this window."), setupCard);
    inputDetails_->setObjectName(QStringLiteral("hint"));
    setup->addWidget(inputDetails_, 2, 1, 1, 2);

    auto *outputLabel = new QLabel(QStringLiteral("Output folder"), setupCard);
    outputLabel->setObjectName(QStringLiteral("fieldLabel"));
    setup->addWidget(outputLabel, 3, 0);

    outputEdit_ = new QLineEdit(setupCard);
    outputEdit_->setPlaceholderText(QStringLiteral("Choose where generated files will be written"));
    outputEdit_->setClearButtonEnabled(true);
    setup->addWidget(outputEdit_, 3, 1);

    auto *outputBrowse = new QPushButton(QStringLiteral("Browse…"), setupCard);
    outputBrowse->setObjectName(QStringLiteral("secondaryButton"));
    setup->addWidget(outputBrowse, 3, 2);

    page->addWidget(setupCard);

    auto *content = new QHBoxLayout;
    content->setSpacing(18);

    auto *outputCard = makeCard(central);
    auto *outputLayout = new QVBoxLayout(outputCard);
    outputLayout->setContentsMargins(20, 18, 20, 18);
    outputLayout->setSpacing(12);

    auto *outputHeader = new QHBoxLayout;
    auto *outputTitle = new QLabel(QStringLiteral("Generated files"), outputCard);
    outputTitle->setObjectName(QStringLiteral("sectionTitle"));
    outputHeader->addWidget(outputTitle);
    outputHeader->addStretch();
    openFolderButton_ = new QPushButton(QStringLiteral("Open folder"), outputCard);
    openFolderButton_->setObjectName(QStringLiteral("quietButton"));
    outputHeader->addWidget(openFolderButton_);
    outputLayout->addLayout(outputHeader);

    outputTree_ = new QTreeWidget(outputCard);
    outputTree_->setHeaderLabels(
        {QStringLiteral("File"), QStringLiteral("Purpose"), QStringLiteral("Size"),
         QStringLiteral("Status")});
    outputTree_->setRootIsDecorated(false);
    outputTree_->setAlternatingRowColors(true);
    outputTree_->setSelectionMode(QAbstractItemView::SingleSelection);
    outputTree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    outputTree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    outputTree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    outputTree_->header()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    outputLayout->addWidget(outputTree_);
    content->addWidget(outputCard, 3);

    auto *logCard = makeCard(central);
    auto *logLayout = new QVBoxLayout(logCard);
    logLayout->setContentsMargins(20, 18, 20, 18);
    logLayout->setSpacing(12);
    auto *logTitle = new QLabel(QStringLiteral("Calculation log"), logCard);
    logTitle->setObjectName(QStringLiteral("sectionTitle"));
    logLayout->addWidget(logTitle);

    log_ = new QPlainTextEdit(logCard);
    log_->setReadOnly(true);
    log_->setPlaceholderText(QStringLiteral("Engine progress and diagnostics will appear here."));
    log_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    log_->setMaximumBlockCount(5000);
    logLayout->addWidget(log_);
    content->addWidget(logCard, 2);

    page->addLayout(content, 1);

    auto *footer = new QHBoxLayout;
    footer->setSpacing(12);
    statusLabel_ = new QLabel(QStringLiteral("Ready"), central);
    statusLabel_->setObjectName(QStringLiteral("status"));
    footer->addWidget(statusLabel_);

    progressBar_ = new QProgressBar(central);
    progressBar_->setTextVisible(false);
    progressBar_->setFixedWidth(150);
    progressBar_->setRange(0, 1);
    progressBar_->setValue(0);
    footer->addWidget(progressBar_);
    footer->addStretch();

    openWhenFinished_ = new QCheckBox(QStringLiteral("Open folder when finished"), central);
    openWhenFinished_->setChecked(true);
    footer->addWidget(openWhenFinished_);

    runButton_ = new QPushButton(QStringLiteral("Calculate wing"), central);
    runButton_->setObjectName(QStringLiteral("primaryButton"));
    runButton_->setMinimumWidth(170);
    footer->addWidget(runButton_);
    page->addLayout(footer);

    setCentralWidget(central);
    setStyleSheet(QStringLiteral(R"(
        QMainWindow, QWidget {
            background: #0d1422;
            color: #e6edf7;
            font-family: "Segoe UI";
            font-size: 10pt;
        }
        QFrame#card {
            background: #151f30;
            border: 1px solid #26354a;
            border-radius: 12px;
        }
        QLabel#brandMark {
            background: #38bdf8;
            color: #07111e;
            border-radius: 14px;
            font-size: 17pt;
            font-weight: 800;
        }
        QLabel#title {
            font-size: 23pt;
            font-weight: 700;
            color: #f7fbff;
        }
        QLabel#subtitle, QLabel#hint {
            color: #93a4ba;
        }
        QLabel#badge {
            background: #132d3a;
            color: #67d3ff;
            border: 1px solid #24536a;
            border-radius: 9px;
            padding: 6px 10px;
            font-weight: 600;
        }
        QLabel#sectionTitle {
            color: #f7fbff;
            font-size: 12pt;
            font-weight: 650;
        }
        QLabel#fieldLabel {
            color: #b9c6d8;
            font-weight: 600;
        }
        QLabel#status {
            color: #a8b6c9;
            font-weight: 600;
        }
        QLineEdit, QPlainTextEdit, QTreeWidget {
            background: #0e1726;
            border: 1px solid #2a3a50;
            border-radius: 7px;
            color: #e6edf7;
            selection-background-color: #176b91;
        }
        QLineEdit {
            padding: 8px 10px;
        }
        QLineEdit:focus, QPlainTextEdit:focus, QTreeWidget:focus {
            border-color: #38bdf8;
        }
        QPlainTextEdit {
            padding: 8px;
            color: #b8c9dc;
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
            padding: 7px;
            font-weight: 600;
        }
        QPushButton {
            border-radius: 7px;
            padding: 8px 13px;
            font-weight: 600;
        }
        QPushButton#primaryButton {
            background: #38bdf8;
            color: #07111e;
            border: 1px solid #54c8f7;
            padding: 10px 18px;
        }
        QPushButton#primaryButton:hover {
            background: #63cdf8;
        }
        QPushButton#primaryButton:disabled {
            background: #314052;
            color: #77869a;
            border-color: #3a4a5d;
        }
        QPushButton#secondaryButton {
            background: #223149;
            border: 1px solid #354a66;
            color: #dce7f5;
        }
        QPushButton#secondaryButton:hover, QPushButton#quietButton:hover {
            background: #2a3d59;
        }
        QPushButton#quietButton {
            background: transparent;
            border: 1px solid #354a66;
            color: #bcd0e6;
            padding: 5px 10px;
        }
        QCheckBox {
            color: #aebdd0;
            spacing: 7px;
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
            margin: 0;
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

    connect(inputBrowse, &QPushButton::clicked, this, [this] { browseForInput(); });
    connect(manualButton, &QPushButton::clicked, this, [] {
        QDesktopServices::openUrl(
            QUrl(QStringLiteral("https://www.laboratoridenvol.com/leparagliding/manual.en.html")));
    });
    connect(outputBrowse, &QPushButton::clicked, this, [this] { browseForOutput(); });
    connect(runButton_, &QPushButton::clicked, this, [this] { startCalculation(); });
    connect(inputEdit_, &QLineEdit::textChanged, this, [this] {
        const QFileInfo info(inputEdit_->text());
        if (info.isFile()) {
            inputDetails_->setText(
                QStringLiteral("%1 · modified %2")
                    .arg(humanReadableSize(info.size()),
                         info.lastModified().toString(QStringLiteral("yyyy-MM-dd HH:mm"))));
        } else {
            inputDetails_->setText(QStringLiteral("Select an existing design file."));
        }
        updateRunAvailability();
    });
    connect(outputEdit_, &QLineEdit::textChanged, this, [this] {
        refreshOutputFiles();
        updateRunAvailability();
    });
    connect(openFolderButton_, &QPushButton::clicked, this, [this] {
        if (!outputEdit_->text().isEmpty()) {
            QDesktopServices::openUrl(QUrl::fromLocalFile(outputEdit_->text()));
        }
    });
    connect(outputTree_, &QTreeWidget::itemDoubleClicked, this,
            [this](QTreeWidgetItem *item) { openOutputItem(item); });
}

void MainWindow::connectProcess()
{
    process_->setProcessChannelMode(QProcess::MergedChannels);
    connect(process_, &QProcess::readyReadStandardOutput, this,
            [this] { appendProcessOutput(); });
    connect(process_, &QProcess::started, this, [this] {
        statusLabel_->setText(QStringLiteral("Calculating…"));
    });
    connect(process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) {
            log_->appendPlainText(
                QStringLiteral("Could not start the C++ calculation engine at:\n%1")
                    .arg(enginePath()));
            setRunning(false);
            statusLabel_->setText(QStringLiteral("Engine could not start"));
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
    const QString initial = QFileInfo(inputEdit_->text()).absolutePath();
    const QString file = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("Select LEparagliding design"),
        initial,
        QStringLiteral("LEparagliding design (*.txt);;All files (*.*)"));
    if (!file.isEmpty()) {
        setInputPath(file);
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

void MainWindow::setInputPath(const QString &path)
{
    const QFileInfo info(path);
    inputEdit_->setText(QDir::toNativeSeparators(info.absoluteFilePath()));
    if (outputEdit_->text().isEmpty()) {
        outputEdit_->setText(QDir::toNativeSeparators(info.absolutePath()));
    }
}

void MainWindow::startCalculation()
{
    const QFileInfo input(inputEdit_->text());
    if (!input.isFile()) {
        QMessageBox::warning(this,
                             QStringLiteral("Design file not found"),
                             QStringLiteral("Select an existing LEparagliding design file."));
        return;
    }

    if (!QDir().mkpath(outputEdit_->text())) {
        QMessageBox::warning(this,
                             QStringLiteral("Output folder unavailable"),
                             QStringLiteral("The selected output folder could not be created."));
        return;
    }

    if (!QFileInfo::exists(enginePath())) {
        QMessageBox::critical(
            this,
            QStringLiteral("Calculation engine missing"),
            QStringLiteral("The C++ calculation engine was not found next to the application:\n%1")
                .arg(enginePath()));
        return;
    }

    saveSettings();
    log_->clear();
    log_->appendPlainText(
        QStringLiteral("Input:  %1\nOutput: %2\n").arg(input.absoluteFilePath(),
                                                       QDir(outputEdit_->text()).absolutePath()));
    setRunning(true);

    process_->setProgram(enginePath());
    process_->setArguments(
        {input.absoluteFilePath(), QDir(outputEdit_->text()).absolutePath()});
    process_->setWorkingDirectory(input.absolutePath());
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
    setRunning(false);
    refreshOutputFiles();

    int generatedCount = 0;
    for (const auto &output : outputs) {
        if (QFileInfo::exists(outputPathFor(QString::fromLatin1(output.fileName)))) {
            ++generatedCount;
        }
    }

    const bool success =
        exitStatus == QProcess::NormalExit && exitCode == 0
        && generatedCount == static_cast<int>(outputs.size());

    if (success) {
        statusLabel_->setText(QStringLiteral("Calculation completed · 4 files generated"));
        if (openWhenFinished_->isChecked()) {
            QDesktopServices::openUrl(
                QUrl::fromLocalFile(QDir(outputEdit_->text()).absolutePath()));
        }
    } else {
        statusLabel_->setText(
            QStringLiteral("Calculation failed · exit %1 · %2/4 files")
                .arg(exitCode)
                .arg(generatedCount));
        log_->appendPlainText(
            QStringLiteral("\nThe engine did not complete. Check the last messages above, "
                           "the design format, and referenced airfoil files."));
    }
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
        if (info.isFile()) {
            item->setForeground(3, QColor(QStringLiteral("#56d7a0")));
        } else {
            item->setForeground(3, QColor(QStringLiteral("#8494a9")));
        }
    }

    openFolderButton_->setEnabled(!outputEdit_->text().isEmpty()
                                  && QDir(outputEdit_->text()).exists());
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

void MainWindow::setRunning(bool running)
{
    inputEdit_->setEnabled(!running);
    outputEdit_->setEnabled(!running);
    runButton_->setEnabled(!running);
    progressBar_->setRange(0, running ? 0 : 1);
    progressBar_->setValue(running ? 0 : 1);
    if (!running) {
        updateRunAvailability();
    }
}

void MainWindow::updateRunAvailability()
{
    const bool ready = QFileInfo(inputEdit_->text()).isFile()
                       && !outputEdit_->text().trimmed().isEmpty()
                       && process_->state() == QProcess::NotRunning;
    runButton_->setEnabled(ready);
}

void MainWindow::loadSettings()
{
    QSettings settings;
    const QString input = settings.value(QStringLiteral("paths/input")).toString();
    const QString output = settings.value(QStringLiteral("paths/output")).toString();
    if (!input.isEmpty()) {
        inputEdit_->setText(input);
    }
    if (!output.isEmpty()) {
        outputEdit_->setText(output);
    }
    openWhenFinished_->setChecked(
        settings.value(QStringLiteral("behavior/openWhenFinished"), true).toBool());
}

void MainWindow::saveSettings() const
{
    QSettings settings;
    settings.setValue(QStringLiteral("paths/input"), inputEdit_->text());
    settings.setValue(QStringLiteral("paths/output"), outputEdit_->text());
    settings.setValue(QStringLiteral("behavior/openWhenFinished"),
                      openWhenFinished_->isChecked());
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
        if (info.isFile()) {
            setInputPath(info.absoluteFilePath());
            event->acceptProposedAction();
        }
    }
}
