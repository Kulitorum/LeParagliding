#pragma once

#include <QColor>
#include <QPoint>
#include <QString>
#include <QVector>
#include <QWidget>

#include <functional>
#include <memory>

class QKeyEvent;
class QMouseEvent;
class QPaintEngine;
class QPaintEvent;
class QResizeEvent;
class QShowEvent;
class QWheelEvent;

class ParagliderView final : public QWidget
{
public:
    enum class ViewPreset
    {
        Isometric,
        Front,
        Back,
        Left,
        Right,
        Top,
        Bottom,
    };

    // Display-colour roles for every registered part type. Values are
    // persisted in the settings, so only append new roles.
    enum class ColorRole
    {
        Extrados,
        Intrados,
        Vents,
        SurfaceWireframe,
        Ribs,
        PlanA,
        PlanB,
        PlanC,
        PlanD,
        PlanE,
        PlanF,
        BrakeLines,
        OtherParts,
        Diagonals,
    };
    static constexpr int colorRoleCount = 14;

    enum class ClipAxis
    {
        None,
        X,
        Y,
        Z,
    };

    struct PartInfo
    {
        int id = -1;
        int parentId = -1;
        QString name;
        bool isGroup = false;
        bool visible = true;
        ColorRole role = ColorRole::OtherParts;
    };

    explicit ParagliderView(QWidget *parent = nullptr);
    ~ParagliderView() override;

    bool loadStep(const QString &path, QString *errorMessage);
    void clearModel();
    void setTriangulationResolution(double deflectionScale);
    double triangulationResolution() const;
    void fitAll();
    // Frames the current selection; fits everything when nothing is selected.
    void fitSelection();
    void setView(ViewPreset preset, bool fit = false);
    void resetCamera();
    // The camera survives preview rebuilds; call this before loading a
    // different design so the next load starts from isometric + fit.
    void resetCameraOnNextLoad();
    void setPerspective(bool enabled);
    void toggleProjection();

    // 0 = opaque fabric, up to 0.9 = X-ray view of the internal structure.
    void setSurfaceTransparency(double transparency01);
    double surfaceTransparency() const;
    // Section view: clips the model with a capping plane along an axis;
    // position01 sweeps the plane across the model bounds.
    void setClipPlane(ClipAxis axis, bool flipped, double position01);
    // Measure mode: the next two clicks pick model points and display the
    // distance; Esc or toggling off clears the measurement.
    void setMeasureMode(bool enabled);
    bool isMeasureMode() const;

    bool isPerspective() const;
    bool hasModel() const;
    qsizetype partCount() const;
    qsizetype surfaceCount() const;
    qsizetype rationalSurfaceCount() const;
    qsizetype splineCount() const;
    qsizetype triangleCount() const;
    QString modelSummary() const;

    // Pre-order part list for building the structure tree.
    QVector<PartInfo> partTree() const;
    QString partPath(int id) const;
    void setPartVisible(int id, bool visible);
    // Shows only the given part (or group subtree), hiding everything else.
    void showOnlyPart(int id);
    void showAllParts();
    void selectPart(int id);
    void clearSelection();
    void zoomToPart(int id);

    static QString colorRoleLabel(ColorRole role);
    static QColor defaultColor(ColorRole role);
    QColor color(ColorRole role) const;
    void setColor(ColorRole role, const QColor &color);

    // Called with the picked part id (-1 for background) on click, and the
    // part id under the cursor (-1 when none) while hovering.
    std::function<void(int)> partPicked;
    std::function<void(int)> partHovered;
    // Measurement status text ("" when cleared) and measure-mode changes
    // triggered from the viewport itself (M key, Esc).
    std::function<void(const QString &)> measurementChanged;
    std::function<void(bool)> measureModeChanged;

    QSize sizeHint() const override;

protected:
    QPaintEngine *paintEngine() const override;
    void showEvent(QShowEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void leaveEvent(QEvent *event) override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    void ensureNativeWindow();
    void redraw();
    void updateCursor();
    void updateHover(const QPoint &position);
    void pickAt(const QPoint &position, bool retargetCamera);
    QPoint nativePixel(const QPoint &logicalPosition) const;
    // Runs a camera-changing operation; when animate is true the view glides
    // from the current camera to the operation's result.
    void runCameraOperation(
        const std::function<void()> &operation,
        bool animate);

    QPoint previousMousePosition_;
    QPoint pressPosition_;
    Qt::MouseButton dragButton_ = Qt::NoButton;
    bool shiftPan_ = false;
    bool dragMoved_ = false;
};
