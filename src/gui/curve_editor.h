#pragma once

#include <QColor>
#include <QPointF>
#include <QRect>
#include <QString>
#include <QVector>
#include <QWidget>

// Generic multi-series curve display and editor. Every series is drawn
// normalized to its own value range over a shared x axis, so curves with very
// different units (centimetres, degrees, percentages) stay readable together.
// One series at a time is selected — via its legend chip or by clicking the
// curve — and shows the real value axis; its points drag vertically and nudge
// with the arrow keys. Nothing here is specific to one data source: hosts
// build a QVector<CurveSeries> and write values back on editCommitted().

struct CurveSeries
{
    QString id;          // stable identity, preserved across setSeriesList()
    QString label;
    QString unit;        // shown in the axis header and value bubble
    QString description; // hosts may show this for the selected series
    QColor color;
    Qt::PenStyle penStyle = Qt::SolidLine;
    QVector<QPointF> points; // x ascending
    bool editable = true;
    // Disabled series keep their legend chip (greyed, with disabledNote as
    // its tooltip) but draw no curve and cannot be selected — for columns
    // the engine ignores in the current data.
    bool enabled = true;
    QString disabledNote;
    double minValue = -1e300; // drag/nudge clamps
    double maxValue = 1e300;
    int decimals = 2; // display precision

    // Spline mode: when `handles` is non-empty they are the editable set
    // (drawn as squares joined by a thin control polygon) and `points`
    // become passive markers; `smooth` is the densely sampled curve drawn
    // instead of the point-to-point polyline. Signals still report indexes
    // into the editable set, whichever it is.
    QVector<QPointF> handles;
    QVector<QPointF> smooth;

    const QVector<QPointF> &editableSet() const
    {
        return handles.isEmpty() ? points : handles;
    }
    QVector<QPointF> &editableSet()
    {
        return handles.isEmpty() ? points : handles;
    }
};

class CurveEditor final : public QWidget
{
    Q_OBJECT
public:
    explicit CurveEditor(QWidget *parent = nullptr);

    // Replaces all series; the selection is kept when the id still exists.
    void setSeriesList(const QVector<CurveSeries> &series);
    const QVector<CurveSeries> &seriesList() const { return series_; }

    // Non-empty message replaces the plot (e.g. "row 3 is not numeric").
    void setMessage(const QString &message);
    void setXAxisLabel(const QString &label);

    QString selectedSeriesId() const;
    void setSelectedSeriesId(const QString &id);

    // Replaces the selected series' smooth polyline in place (no reset of
    // selection or drag state) — lets hosts morph a spline curve live while
    // one of its handles is being dragged.
    void setSelectedSeriesSmooth(const QVector<QPointF> &smooth);

    QSize minimumSizeHint() const override { return {380, 200}; }
    QSize sizeHint() const override { return {720, 280}; }

signals:
    void selectionChanged(const QString &seriesId);
    // Live while dragging; the value is already clamped but not yet rounded.
    void pointMoved(const QString &seriesId, int pointIndex, double value);
    // Drag released or a keyboard nudge applied: time to persist the series.
    void editCommitted(const QString &seriesId);
    // Ctrl+Z / Ctrl+Y pressed with this widget focused; the host owns the
    // undo stack for graphical edits.
    void undoRequested();
    void redoRequested();

protected:
    bool event(QEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void leaveEvent(QEvent *event) override;

private:
    struct ValueRange
    {
        double low = 0.0;
        double high = 1.0;
    };

    QRectF plotRect() const;
    ValueRange paddedRange(int seriesIndex) const;
    double xToPixel(double x, const QRectF &plot) const;
    double valueToPixel(double value, const ValueRange &range,
                        const QRectF &plot) const;
    double pixelToValue(double pixelY, const ValueRange &range,
                        const QRectF &plot) const;
    void rebuildChips();
    int chipAt(const QPoint &position) const;
    int pointNear(int seriesIndex, const QPointF &position,
                  double maxDistance) const;
    int seriesNear(const QPointF &position, double maxDistance) const;
    void selectSeries(int index);
    void nudgeActivePoint(double direction, bool large);
    void updateHover(const QPointF &position);

    QVector<CurveSeries> series_;
    QVector<QRect> chipRects_;
    int chipsHeight_ = 0;
    QString message_;
    QString xAxisLabel_;
    int selected_ = -1;
    int hoverChip_ = -1;
    int hoverPoint_ = -1;
    int activePoint_ = -1; // keyboard nudge target
    bool dragging_ = false;
    int dragPoint_ = -1;
    ValueRange dragRange_; // mapping frozen while dragging
    double xMin_ = 0.0;
    double xMax_ = 1.0;
};
