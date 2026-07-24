#pragma once

#include "design_document.h"

#include <QMainWindow>
#include <QProcess>
#include <QSet>
#include <QVector>

class QCloseEvent;
class QDragEnterEvent;
class QDropEvent;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QStackedWidget;
class QTabWidget;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;
class ParagliderView;

class MainWindow final : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent *event) override;
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

private:
    void buildInterface();
    void connectProcess();
    void browseForInput();
    void browseForOutput();
    bool loadDesign(const QString &path, bool confirmUnsaved = true);
    void rebuildSectionEditors();
    bool saveDesign(bool showConfirmation = true);
    bool maybeSaveChanges();
    void showSectionHelp(int index);
    void startCalculation();
    void appendProcessOutput();
    void calculationFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void refreshOutputFiles();
    void openOutputItem(QTreeWidgetItem *item);
    void loadViewportModel();
    void setRunning(bool running);
    void updateRunAvailability();
    void updateWindowTitle();
    void refreshSectionLabels();
    void loadSettings();
    void saveSettings() const;
    QString enginePath() const;
    QString outputPathFor(const QString &fileName) const;

    DesignDocument document_;
    QVector<QPlainTextEdit *> sectionEditors_;
    QVector<QString> savedSectionTexts_;
    QVector<QPushButton *> undoButtons_;
    QVector<QPushButton *> redoButtons_;
    QSet<int> dirtySections_;
    bool documentDirty_ = false;
    bool loadingEditors_ = false;

    QLineEdit *inputEdit_ = nullptr;
    QLineEdit *outputEdit_ = nullptr;
    QLabel *inputDetails_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    QLabel *modelStats_ = nullptr;
    QListWidget *sectionList_ = nullptr;
    QStackedWidget *sectionPages_ = nullptr;
    QPushButton *saveButton_ = nullptr;
    QPushButton *buildButton_ = nullptr;
    QPushButton *openFolderButton_ = nullptr;
    QProgressBar *progressBar_ = nullptr;
    QPlainTextEdit *log_ = nullptr;
    QTreeWidget *outputTree_ = nullptr;
    QTabWidget *diagnosticsTabs_ = nullptr;
    ParagliderView *viewport_ = nullptr;
    QToolButton *projectionButton_ = nullptr;
    QProcess *process_ = nullptr;
};
