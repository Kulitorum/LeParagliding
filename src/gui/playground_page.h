#pragma once

#include <QWidget>

class QCheckBox;
class QComboBox;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;
class QVBoxLayout;

class PlaygroundAnalysisDialog;
class PlaygroundView;
class SettleWorker;

// The Playground tab: an instrumented wind tunnel for shape fidelity (see
// docs/playground-shape-analysis.md). The engine's companion mesh
// (lep-sim.json — coarse welded skin quads sampled from the exact
// ballooning law, rib loops, labelled suspension lines) is assembled into
// a softwing XPBD soft body, loaded by the tunnel's pressure field plus an
// optional wing-level flight load, and measured against its own rest pose
// — which IS the design shape. The instruments (the colour-by heatmaps,
// the live shape HUD, the grab tool, the Analyse α-sweep) report where
// the loaded wing departs from it: fabric going slack, a row unloading, a
// nose denting. The claim is relative and structural, never absolute
// aerodynamics. Free flight survives as an experimental toy mode.
class PlaygroundPage : public QWidget
{
    Q_OBJECT

public:
    explicit PlaygroundPage(QWidget *parent = nullptr);
    // Cancels and joins a running settle worker; the per-frame cancel
    // makes the join a matter of milliseconds.
    ~PlaygroundPage() override;

    // Reads the engine's lep-sim.json immediately (preview output
    // directories are temporary); the mesh is assembled into the
    // simulation when the tab is next shown, or right away if visible.
    void setSimMeshPath(const QString &path);

    // Splits each exported skin quad into factor x factor sub-quads before
    // the body is assembled: 1 is the engine's own mesh, 4 is sixteen times
    // the triangles. Rebuilds the running simulation from the retained
    // mesh, so the wing resets to its rest pose.
    void setMeshSubdivision(int factor);
    int meshSubdivision() const { return subdivision_; }
    static constexpr int maximumMeshSubdivision = 4;

    // Meshes each rib as a real holed sheet instead of a hub and spokes:
    // costlier, but the only form in which rib load means anything. Rebuilds
    // the simulation from the retained mesh.
    void setDetailedRibs(bool enabled);
    bool detailedRibs() const { return detailedRibs_; }

protected:
    void showEvent(QShowEvent *event) override;

private:
    // The GL view is created on first tab activation, not at startup: the
    // Design tab's OCCT viewport is a native child window, and putting a
    // QOpenGLWidget into the window before that native swapchain exists
    // flips the whole window into GL composition and blacks out every
    // composited GL tab (XFLR5's lazy views avoid this the same way).
    void ensureView();
    void loadIfPending();
    void setSweepActive(bool active);
    // The max-quality converge-and-stop run: Settle starts it, the same
    // button cancels it, finishSettle() adopts the result and pauses.
    void toggleSettle();
    void finishSettle();
    // Re-reads the retained mesh so a changed preference takes effect
    // without another engine run. The wing returns to its rest pose.
    void rebuildSimulation();
    // Starts the shape-HUD poll while the solver runs and stops it when it
    // does not; called beside every place the run state changes.
    void updateShapeTimer();
    // The α-sweep report, non-modal so the tunnel stays usable beside it.
    void openAnalysis();

    QVBoxLayout *layout_ = nullptr;
    PlaygroundView *view_ = nullptr;
    QLabel *status_ = nullptr;
    QSlider *pressure_ = nullptr;
    QSlider *lift_ = nullptr;
    QSlider *leftBrake_ = nullptr;
    QSlider *rightBrake_ = nullptr;
    QPushButton *runButton_ = nullptr;
    QPushButton *resetButton_ = nullptr;
    QPushButton *flyButton_ = nullptr;
    QCheckBox *showExtrados_ = nullptr;
    QCheckBox *showVent_ = nullptr;
    QCheckBox *showIntrados_ = nullptr;
    QCheckBox *showRibs_ = nullptr;
    QCheckBox *showStraps_ = nullptr;
    QCheckBox *showLines_ = nullptr;
    QComboBox *colorBy_ = nullptr;
    QSlider *stressScale_ = nullptr;
    QCheckBox *showLineTension_ = nullptr;
    QSlider *lineScale_ = nullptr;
    QComboBox *quality_ = nullptr;
    QCheckBox *freeFlight_ = nullptr;
    QLabel *flightLabel_ = nullptr;
    QTimer *flightTimer_ = nullptr;
    QLabel *shapeLabel_ = nullptr;
    QPushButton *settleButton_ = nullptr;
    // Non-null exactly while a settle worker runs; parented to the page,
    // deleteLater'd from finishSettle.
    SettleWorker *settleWorker_ = nullptr;
    // At most one analysis dialog, so two sweeps can never race each
    // other for the pause on the live solver.
    PlaygroundAnalysisDialog *analysisDialog_ = nullptr;
    // While a sweep runs, the live solver stays paused whatever else
    // happens (Run button, Reset, rebuilds): the worker owns the cores.
    bool sweepActive_ = false;
    QCheckBox *flightLoad_ = nullptr;
    QPushButton *analyseButton_ = nullptr;
    QTimer *shapeTimer_ = nullptr;
    QLabel *stressLegend_ = nullptr;
    QTimer *legendTimer_ = nullptr;
    QByteArray pendingData_;
    // Retained so a resolution change can rebuild the body without
    // re-running the engine (whose output directory is long gone).
    QByteArray meshData_;
    int subdivision_ = 1;
    bool detailedRibs_ = false;
    // Creating the view's native window pumps the event loop, which can
    // redeliver this page's show event before view_ is assigned; the flag
    // keeps that reentrant call from constructing a second view.
    bool creatingView_ = false;
};
