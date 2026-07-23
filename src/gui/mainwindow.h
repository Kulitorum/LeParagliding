#pragma once

#include <QMainWindow>
#include <QProcess>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QTreeWidget;
class QTreeWidgetItem;

class MainWindow final : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

private:
    void buildInterface();
    void connectProcess();
    void browseForInput();
    void browseForOutput();
    void setInputPath(const QString &path);
    void startCalculation();
    void appendProcessOutput();
    void calculationFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void refreshOutputFiles();
    void openOutputItem(QTreeWidgetItem *item);
    void setRunning(bool running);
    void updateRunAvailability();
    void loadSettings();
    void saveSettings() const;
    QString enginePath() const;
    QString outputPathFor(const QString &fileName) const;

    QLineEdit *inputEdit_ = nullptr;
    QLineEdit *outputEdit_ = nullptr;
    QLabel *inputDetails_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    QPushButton *runButton_ = nullptr;
    QPushButton *openFolderButton_ = nullptr;
    QProgressBar *progressBar_ = nullptr;
    QPlainTextEdit *log_ = nullptr;
    QTreeWidget *outputTree_ = nullptr;
    QCheckBox *openWhenFinished_ = nullptr;
    QProcess *process_ = nullptr;
};
