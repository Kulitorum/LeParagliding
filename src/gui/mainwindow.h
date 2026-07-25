#pragma once

#include "design_document.h"
#include "preset_catalog.h"

#include <QHash>
#include <QMainWindow>
#include <QProcess>
#include <QSet>
#include <QVector>

#include <memory>

class QCloseEvent;
class QDragEnterEvent;
class QDropEvent;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QSlider;
class QStackedWidget;
class QTabWidget;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;
class QTemporaryDir;
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
    enum class CalculationMode
    {
        None,
        Preview,
        Export
    };

    void buildInterface();
    void buildPresetsMenu(QPushButton *button);
    void connectProcess();
    void browseForInput();
    void browseForOutput();
    bool loadDesign(const QString &path, bool confirmUnsaved = true);
    void rebuildSectionEditors();
    bool saveDesign(bool showConfirmation = true);
    bool maybeSaveChanges();
    void showSectionHelp(int index);
    void showPreferences();
    void showVersionHistory();
    void restoreVersion(int revisionIndex);
    void syncPersistedSectionHistories();
    void undoSection(int index);
    void redoSection(int index);
    void updateUndoRedoAvailability(int index);
    void refreshInputDetails();
    void startPreviewCalculation(bool automatic = false);
    void startExportCalculation();
    void startCalculation(CalculationMode mode, bool automatic);
    void appendProcessOutput();
    void calculationFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void refreshOutputFiles();
    void openOutputItem(QTreeWidgetItem *item);
    bool loadViewportModel(const QString &path);
    void clearViewportModel(const QString &statusText);
    void rebuildPartsTree();
    void refreshPartsTreeIcons();
    void revealPartInTree(int partId);
    void handlePartsTreeCheck(QTreeWidgetItem *item);
    void showPartsTreeMenu(const QPoint &position);
    void syncPartsTreeChecks();
    void jumpToPartDefinition(int partId);
    void showSectionRow(int sectionNumber, int firstRow, int lastRow);
    void setRunning(bool running);
    void updateRunAvailability();
    void updateWindowTitle();
    void refreshSectionLabels();
    void loadSettings();
    void saveSettings() const;
    QString enginePath() const;
    QString outputPathFor(const QString &fileName) const;

    DesignDocument document_;
    QList<PresetWing> presetCatalog_;
    QVector<QPlainTextEdit *> sectionEditors_;
    QVector<QString> savedSectionTexts_;
    QVector<QPushButton *> undoButtons_;
    QVector<QPushButton *> redoButtons_;
    QVector<QStringList> persistedSectionHistories_;
    QVector<int> persistedSectionHistoryPositions_;
    QSet<int> dirtySections_;
    bool documentDirty_ = false;
    bool loadingEditors_ = false;
    CalculationMode calculationMode_ = CalculationMode::None;
    std::unique_ptr<QTemporaryDir> calculationDirectory_;
    QString calculationOutputDirectory_;

    QLineEdit *inputEdit_ = nullptr;
    QLineEdit *outputEdit_ = nullptr;
    QPushButton *inputBrowseButton_ = nullptr;
    QPushButton *outputBrowseButton_ = nullptr;
    QLabel *inputDetails_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    QLabel *modelStats_ = nullptr;
    QListWidget *sectionList_ = nullptr;
    QStackedWidget *sectionPages_ = nullptr;
    QPushButton *historyButton_ = nullptr;
    QPushButton *saveButton_ = nullptr;
    QPushButton *buildButton_ = nullptr;
    QPushButton *exportButton_ = nullptr;
    QPushButton *openFolderButton_ = nullptr;
    QProgressBar *progressBar_ = nullptr;
    QPlainTextEdit *log_ = nullptr;
    QTreeWidget *outputTree_ = nullptr;
    QTabWidget *diagnosticsTabs_ = nullptr;
    ParagliderView *viewport_ = nullptr;
    QToolButton *projectionButton_ = nullptr;
    QTreeWidget *partsTree_ = nullptr;
    QLabel *partHoverLabel_ = nullptr;
    QToolButton *measureButton_ = nullptr;
    QSlider *xraySlider_ = nullptr;
    QHash<int, QTreeWidgetItem *> partsTreeItems_;
    bool syncingPartsTree_ = false;
    QProcess *process_ = nullptr;
};
