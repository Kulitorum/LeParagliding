#pragma once

#include "design_document.h"
#include "preset_catalog.h"

#include <QByteArray>
#include <QHash>
#include <QMainWindow>
#include <QProcess>
#include <QSet>
#include <QVector>

#include <memory>

class QCloseEvent;
class QDialog;
class QDragEnterEvent;
class QDropEvent;
class QFrame;
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
class MainFrame; // XFLR5's main window (third_party/xflr5), hosted as a tab
class PlaygroundPage;
class PrintPage;

class MainWindow final : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

    // Switches to the Aerodynamics tab, constructing the embedded XFLR5
    // MainFrame on first use. Public for --xflr5 and the smoke test, which
    // passes transferWing = false to avoid spawning an engine run.
    void showXflr5Tab(bool transferWing = true);

    // Switches to the Playground tab with the given lep-sim.json loaded.
    // Public for the --playground flag, which exists so the live-wing view
    // can be exercised without clicking through a calculation.
    void showPlaygroundTab(const QString &simMeshPath);

protected:
    void closeEvent(QCloseEvent *event) override;
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    enum class CalculationMode
    {
        None,
        Preview,
        Export,
        Xflr5Transfer
    };

    void buildInterface();
    void initializeXflr5Tab();
    void maybeTransferWingToXflr5();
    void showXflr5Busy(const QString &text, bool busy = true);
    void hideXflr5Busy();
    void repositionXflr5Busy();
    QString designTextWithXflr5ExportForced() const;
    void buildPresetsMenu(QPushButton *button);
    void connectProcess();
    void browseForInput();
    void browseForOutput();
    bool loadDesign(const QString &path, bool confirmUnsaved = true);
    void rebuildSectionEditors();
    bool saveDesign(bool showConfirmation = true);
    bool maybeSaveChanges();
    void showSectionHelp(int index);
    void showManualDialog();
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
    bool exportConstructionCurves_ = true;
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
    QTabWidget *workspaceTabs_ = nullptr;
    PlaygroundPage *playgroundPage_ = nullptr;
    PrintPage *printPage_ = nullptr;
    QWidget *xflr5Page_ = nullptr;
    MainFrame *xflr5Frame_ = nullptr;
    QByteArray xflr5TransferredHash_;
    QByteArray xflr5RunHash_;
    bool xflr5TransferPending_ = false;
    bool previewPending_ = false;
    QFrame *xflr5Busy_ = nullptr;
    QLabel *xflr5BusyLabel_ = nullptr;
    QProgressBar *xflr5BusyBar_ = nullptr;
    bool xflr5Initializing_ = false;
    int xflr5BusyGeneration_ = 0;
    ParagliderView *viewport_ = nullptr;
    QToolButton *projectionButton_ = nullptr;
    QTreeWidget *partsTree_ = nullptr;
    QLabel *partHoverLabel_ = nullptr;
    QToolButton *measureButton_ = nullptr;
    QSlider *xraySlider_ = nullptr;
    QHash<int, QTreeWidgetItem *> partsTreeItems_;
    bool syncingPartsTree_ = false;
    QDialog *manualDialog_ = nullptr;
    QProcess *process_ = nullptr;
};
