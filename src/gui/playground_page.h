#pragma once

#include <QWidget>

class QLabel;
class QPushButton;
class QSlider;
class QVBoxLayout;

class PlaygroundView;

// The Playground tab: a toy live-wing simulation. The engine's companion
// mesh (lep-sim.json — coarse welded skin quads sampled from the exact
// ballooning law, rib loops, labelled suspension lines) is assembled into
// a softwing XPBD soft body; internal pressure inflates it and the brake
// anchors can be pulled. Deliberately no engineering claims: this is a
// visual sandbox, not analysis.
class PlaygroundPage : public QWidget
{
    Q_OBJECT

public:
    explicit PlaygroundPage(QWidget *parent = nullptr);

    // Reads the engine's lep-sim.json immediately (preview output
    // directories are temporary); the mesh is assembled into the
    // simulation when the tab is next shown, or right away if visible.
    void setSimMeshPath(const QString &path);

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

    QVBoxLayout *layout_ = nullptr;
    PlaygroundView *view_ = nullptr;
    QLabel *status_ = nullptr;
    QSlider *pressure_ = nullptr;
    QSlider *lift_ = nullptr;
    QSlider *leftBrake_ = nullptr;
    QSlider *rightBrake_ = nullptr;
    QPushButton *runButton_ = nullptr;
    QByteArray pendingData_;
    // Creating the view's native window pumps the event loop, which can
    // redeliver this page's show event before view_ is assigned; the flag
    // keeps that reentrant call from constructing a second view.
    bool creatingView_ = false;
};
