#pragma once

#include <QPoint>
#include <QWidget>

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

    explicit ParagliderView(QWidget *parent = nullptr);
    ~ParagliderView() override;

    bool loadStep(const QString &path, QString *errorMessage);
    void clearModel();
    void setTriangulationResolution(double deflectionScale);
    double triangulationResolution() const;
    void fitAll();
    void setView(ViewPreset preset);
    void setPerspective(bool enabled);
    void toggleProjection();

    bool isPerspective() const;
    bool hasModel() const;
    qsizetype surfaceCount() const;
    qsizetype rationalSurfaceCount() const;
    qsizetype shellCount() const;
    qsizetype splineCount() const;
    qsizetype triangleCount() const;
    QString modelSummary() const;

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

private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    void ensureNativeWindow();
    void redraw();
    void updateCursor();

    QPoint previousMousePosition_;
    Qt::MouseButton dragButton_ = Qt::NoButton;
    bool shiftPan_ = false;
};
