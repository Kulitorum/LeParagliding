#ifndef LEP_PLAYGROUND_ANALYSIS_H
#define LEP_PLAYGROUND_ANALYSIS_H

#include "playground_metrics.h"
#include "playground_sim.h"

#include <QByteArray>
#include <QDialog>

#include <vector>

class QCloseEvent;
class QDoubleSpinBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QTableWidget;

class PlaygroundSweepPlot;
class PlaygroundSweepWorker;

// The Analyse report: sweeps the wind tunnel across an angle-of-attack
// range on a worker thread — a fresh body per point, settled to
// quiescence, then measured (settleAndMeasure, the same pass the bench
// runs, so every plotted number is reproducible headless) — and shows the
// polar a shape designer actually wants: not just L/D against alpha but
// shape integrity against it. Non-modal, so the live tunnel stays usable
// beside the report; the page pauses its own stepping via sweepRunning so
// the sweep is not fighting the GUI sim for the same worker threads.
class PlaygroundAnalysisDialog : public QDialog
{
    Q_OBJECT

public:
    PlaygroundAnalysisDialog(const QByteArray &meshData,
                             int subdivision,
                             const lep::playground::SimBuildOptions &options,
                             const lep::playground::SimControls &controls,
                             QWidget *parent = nullptr);
    ~PlaygroundAnalysisDialog() override;

signals:
    void sweepRunning(bool running);

protected:
    // A sweep in flight is cancelled and joined before the dialog goes:
    // the worker reads mesh data the dialog owns.
    void closeEvent(QCloseEvent *event) override;

private:
    void toggleRun();
    void startSweep();
    // Cancels, joins, and discards the worker without reading its results;
    // the finish path proper is sweepFinished().
    void stopWorker();
    void sweepProgress(int done, int total);
    void sweepFinished();
    void populateResults();
    void exportCsv();
    void setRunning(bool running);

    QByteArray meshData_;
    int subdivision_ = 1;
    lep::playground::SimBuildOptions buildOptions_;
    lep::playground::SimControls baseControls_;

    QDoubleSpinBox *alphaFrom_ = nullptr;
    QDoubleSpinBox *alphaTo_ = nullptr;
    QDoubleSpinBox *alphaStep_ = nullptr;
    QDoubleSpinBox *settleSeconds_ = nullptr;
    QPushButton *runButton_ = nullptr;
    QProgressBar *progress_ = nullptr;
    QPushButton *exportButton_ = nullptr;
    PlaygroundSweepPlot *retentionPlot_ = nullptr;
    PlaygroundSweepPlot *deviationPlot_ = nullptr;
    PlaygroundSweepPlot *glidePlot_ = nullptr;
    PlaygroundSweepPlot *rowLoadPlot_ = nullptr;
    QTableWidget *table_ = nullptr;
    QLabel *status_ = nullptr;

    PlaygroundSweepWorker *worker_ = nullptr;
    std::vector<lep::playground::SettleResult> results_;
    bool running_ = false;
};

#endif  // LEP_PLAYGROUND_ANALYSIS_H
