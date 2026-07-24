#pragma once

#include <QColor>
#include <QPoint>
#include <QPointF>
#include <QVector>
#include <QVector3D>
#include <QWidget>

class QKeyEvent;
class QMatrix4x4;
class QMouseEvent;
class QPainter;
class QPaintEvent;
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

    bool loadDxf(const QString &path, QString *errorMessage);
    void clearModel();
    void fitAll();
    void setView(ViewPreset preset);
    void setPerspective(bool enabled);
    void toggleProjection();

    bool isPerspective() const;
    bool hasModel() const;
    qsizetype segmentCount() const;
    QString modelSummary() const;

    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    struct ModelLine
    {
        QVector3D start;
        QVector3D end;
        QColor color;
    };

    struct ProjectedPoint
    {
        QPointF point;
        float depth = 0.0F;
        bool visible = false;
    };

    QVector3D cameraPosition() const;
    ProjectedPoint project(const QVector3D &point, const QMatrix4x4 &matrix) const;
    QMatrix4x4 viewProjectionMatrix() const;
    void drawGrid(QPainter &painter, const QMatrix4x4 &matrix) const;
    void drawAxes(QPainter &painter, const QMatrix4x4 &matrix) const;
    void drawHud(QPainter &painter) const;
    void panByPixels(const QPoint &delta);
    void updateCursor();
    float sceneRadius() const;

    QVector<ModelLine> lines_;
    QVector3D boundsMin_;
    QVector3D boundsMax_;
    int layerCount_ = 0;
    QString modelPath_;

    QVector3D target_;
    float azimuthDegrees_ = -45.0F;
    float elevationDegrees_ = 28.0F;
    float distance_ = 1000.0F;
    float orthographicScale_ = 1000.0F;
    bool perspective_ = true;

    QPoint previousMousePosition_;
    Qt::MouseButton dragButton_ = Qt::NoButton;
    bool shiftPan_ = false;
};
