#include "curve_editor.h"

#include <QFontMetrics>
#include <QHelpEvent>
#include <QKeyEvent>
#include <QKeySequence>
#include <QMouseEvent>
#include <QPainter>
#include <QToolTip>

#include <algorithm>
#include <cmath>

namespace {

const QColor kBackground(0x0e, 0x17, 0x26);
const QColor kBorder(0x2a, 0x3a, 0x50);
const QColor kGrid(0x1b, 0x28, 0x40);
const QColor kAxisInk(0x93, 0xa4, 0xba);
const QColor kBrightInk(0xe6, 0xed, 0xf7);
const QColor kHeaderInk(0xb9, 0xc6, 0xd8);
const QColor kChipBackground(0x13, 0x1e, 0x30);
const QColor kChipBorder(0x26, 0x35, 0x4a);
const QColor kChipSelected(0x1f, 0x55, 0x71);
const QColor kDisabledInk(0x5a, 0x6b, 0x80);
const QColor kDisabledSwatch(0x3c, 0x4c, 0x62);
const QColor kBubbleBackground(0x1b, 0x29, 0x3c);
constexpr int kDimmedAlpha = 110;
// One radius for both hover feedback and press, so the cursor never
// advertises a drag the press would reject.
constexpr double kPointHitRadius = 14.0;

double niceStep(double roughStep)
{
    if (roughStep <= 0.0)
        return 1.0;
    const double power = std::pow(10.0, std::floor(std::log10(roughStep)));
    const double base = roughStep / power;
    if (base <= 1.0)
        return power;
    if (base <= 2.0)
        return 2.0 * power;
    if (base <= 5.0)
        return 5.0 * power;
    return 10.0 * power;
}

double distanceToSegment(const QPointF &p, const QPointF &a, const QPointF &b)
{
    const QPointF ab = b - a;
    const double lengthSquared = ab.x() * ab.x() + ab.y() * ab.y();
    double t = 0.0;
    if (lengthSquared > 0.0) {
        t = ((p.x() - a.x()) * ab.x() + (p.y() - a.y()) * ab.y())
            / lengthSquared;
        t = std::clamp(t, 0.0, 1.0);
    }
    const QPointF closest = a + t * ab;
    return std::hypot(p.x() - closest.x(), p.y() - closest.y());
}

QString formatValue(double value, int decimals)
{
    return QString::number(value, 'f', std::max(decimals, 0));
}

} // namespace

CurveEditor::CurveEditor(QWidget *parent) : QWidget(parent)
{
    setMouseTracking(true);
    setFocusPolicy(Qt::ClickFocus);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void CurveEditor::setSeriesList(const QVector<CurveSeries> &series)
{
    const QString previousId = selectedSeriesId();
    series_ = series;
    dragging_ = false;
    dragPoint_ = -1;
    hoverPoint_ = -1;

    selected_ = -1;
    for (int i = 0; i < series_.size(); ++i) {
        if (series_.at(i).id == previousId && series_.at(i).enabled) {
            selected_ = i;
            break;
        }
    }
    if (selected_ >= 0) {
        const int editableCount =
            static_cast<int>(series_.at(selected_).editableSet().size());
        activePoint_ = editableCount > 0
                           ? std::clamp(activePoint_, -1, editableCount - 1)
                           : -1;
    } else {
        activePoint_ = -1;
    }

    xMin_ = 0.0;
    xMax_ = 1.0;
    bool first = true;
    for (const CurveSeries &s : series_) {
        for (const QVector<QPointF> *set : {&s.points, &s.handles, &s.smooth}) {
            for (const QPointF &point : *set) {
                if (first) {
                    xMin_ = xMax_ = point.x();
                    first = false;
                } else {
                    xMin_ = std::min(xMin_, point.x());
                    xMax_ = std::max(xMax_, point.x());
                }
            }
        }
    }
    if (xMax_ - xMin_ < 1e-9)
        xMax_ = xMin_ + 1.0;

    rebuildChips();
    update();
    if (selectedSeriesId() != previousId)
        emit selectionChanged(selectedSeriesId());
}

void CurveEditor::setMessage(const QString &message)
{
    if (message_ == message)
        return;
    message_ = message;
    dragging_ = false;
    hoverPoint_ = -1;
    update();
}

void CurveEditor::setXAxisLabel(const QString &label)
{
    xAxisLabel_ = label;
    update();
}

QString CurveEditor::selectedSeriesId() const
{
    return selected_ >= 0 && selected_ < series_.size()
               ? series_.at(selected_).id
               : QString();
}

void CurveEditor::setSelectedSeriesId(const QString &id)
{
    for (int i = 0; i < series_.size(); ++i) {
        if (series_.at(i).id == id) {
            selectSeries(i);
            return;
        }
    }
}

void CurveEditor::setSelectedSeriesSmooth(const QVector<QPointF> &smooth)
{
    if (selected_ < 0 || selected_ >= series_.size())
        return;
    series_[selected_].smooth = smooth;
    update();
}

QRectF CurveEditor::plotRect() const
{
    const QFontMetrics metrics(font());
    const double left =
        12.0 + metrics.horizontalAdvance(QStringLiteral("00000.00"));
    const double top = 10.0 + chipsHeight_ + 8.0;
    return QRectF(QPointF(left, top),
                  QPointF(width() - 14.0,
                          height() - metrics.height() - 12.0));
}

CurveEditor::ValueRange CurveEditor::paddedRange(int seriesIndex) const
{
    ValueRange range;
    const CurveSeries &series = series_.at(seriesIndex);
    bool first = true;
    for (const QVector<QPointF> *set :
         {&series.points, &series.handles, &series.smooth}) {
        for (const QPointF &point : *set) {
            if (first) {
                range.low = range.high = point.y();
                first = false;
            } else {
                range.low = std::min(range.low, point.y());
                range.high = std::max(range.high, point.y());
            }
        }
    }
    if (first)
        return range;
    const double span = range.high - range.low;
    if (span < 1e-9) {
        const double pad =
            std::max(1.0, std::abs(range.low) * 0.1);
        range.low -= pad;
        range.high += pad;
    } else {
        range.low -= span * 0.08;
        range.high += span * 0.08;
    }
    return range;
}

double CurveEditor::xToPixel(double x, const QRectF &plot) const
{
    return plot.left() + (x - xMin_) / (xMax_ - xMin_) * plot.width();
}

double CurveEditor::valueToPixel(double value, const ValueRange &range,
                                 const QRectF &plot) const
{
    const double span = std::max(range.high - range.low, 1e-12);
    return plot.bottom() - (value - range.low) / span * plot.height();
}

double CurveEditor::pixelToValue(double pixelY, const ValueRange &range,
                                 const QRectF &plot) const
{
    const double span = std::max(range.high - range.low, 1e-12);
    return range.low
           + (plot.bottom() - pixelY) / std::max(plot.height(), 1.0) * span;
}

void CurveEditor::rebuildChips()
{
    chipRects_.clear();
    const QFontMetrics metrics(font());
    const int chipHeight = metrics.height() + 8;
    const int swatchWidth = 16;
    int x = 10;
    int y = 10;
    for (const CurveSeries &series : series_) {
        const int chipWidth =
            8 + swatchWidth + 6 + metrics.horizontalAdvance(series.label) + 9;
        if (x + chipWidth > width() - 10 && x > 10) {
            x = 10;
            y += chipHeight + 5;
        }
        chipRects_.append(QRect(x, y, chipWidth, chipHeight));
        x += chipWidth + 6;
    }
    chipsHeight_ = chipRects_.isEmpty()
                       ? 0
                       : chipRects_.last().bottom() - 10 + 1;
}

int CurveEditor::chipAt(const QPoint &position) const
{
    for (int i = 0; i < chipRects_.size(); ++i) {
        if (chipRects_.at(i).contains(position))
            return i;
    }
    return -1;
}

int CurveEditor::pointNear(int seriesIndex, const QPointF &position,
                           double maxDistance) const
{
    if (seriesIndex < 0 || seriesIndex >= series_.size())
        return -1;
    const QRectF plot = plotRect();
    const ValueRange range =
        dragging_ && seriesIndex == selected_ ? dragRange_
                                              : paddedRange(seriesIndex);
    const QVector<QPointF> &editable = series_.at(seriesIndex).editableSet();
    int best = -1;
    double bestDistance = maxDistance;
    for (int i = 0; i < editable.size(); ++i) {
        const QPointF pixel(xToPixel(editable.at(i).x(), plot),
                            valueToPixel(editable.at(i).y(), range, plot));
        const double distance = std::hypot(pixel.x() - position.x(),
                                           pixel.y() - position.y());
        if (distance <= bestDistance) {
            bestDistance = distance;
            best = i;
        }
    }
    return best;
}

int CurveEditor::seriesNear(const QPointF &position, double maxDistance) const
{
    const QRectF plot = plotRect();
    int best = -1;
    double bestDistance = maxDistance;
    for (int s = 0; s < series_.size(); ++s) {
        const CurveSeries &candidate = series_.at(s);
        if (!candidate.enabled)
            continue;
        const ValueRange range = paddedRange(s);
        const QVector<QPointF> &points =
            candidate.smooth.isEmpty() ? candidate.points : candidate.smooth;
        for (int i = 0; i + 1 < points.size(); ++i) {
            const QPointF a(xToPixel(points.at(i).x(), plot),
                            valueToPixel(points.at(i).y(), range, plot));
            const QPointF b(xToPixel(points.at(i + 1).x(), plot),
                            valueToPixel(points.at(i + 1).y(), range, plot));
            const double distance = distanceToSegment(position, a, b);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = s;
            }
        }
    }
    return best;
}

void CurveEditor::selectSeries(int index)
{
    if (index == selected_ || index < 0 || index >= series_.size()
        || !series_.at(index).enabled)
        return;
    selected_ = index;
    activePoint_ = -1;
    hoverPoint_ = -1;
    update();
    emit selectionChanged(series_.at(index).id);
}

void CurveEditor::nudgeActivePoint(double direction, bool large)
{
    if (selected_ < 0 || selected_ >= series_.size())
        return;
    CurveSeries &series = series_[selected_];
    QVector<QPointF> &editable = series.editableSet();
    if (!series.editable || editable.isEmpty())
        return;
    if (activePoint_ < 0)
        activePoint_ = 0;
    activePoint_ = std::clamp(activePoint_, 0,
                              static_cast<int>(editable.size()) - 1);

    const ValueRange range = paddedRange(selected_);
    double step = std::max((range.high - range.low) / 100.0,
                           std::pow(10.0, -series.decimals));
    if (large)
        step *= 10.0;
    const double value =
        std::clamp(editable.at(activePoint_).y() + direction * step,
                   series.minValue, series.maxValue);
    editable[activePoint_].setY(value);
    update();
    const QString id = series.id;
    const int index = activePoint_;
    emit pointMoved(id, index, value);
    emit editCommitted(id);
}

void CurveEditor::updateHover(const QPointF &position)
{
    const int chip = chipAt(position.toPoint());
    int point = -1;
    if (chip < 0 && selected_ >= 0 && message_.isEmpty())
        point = pointNear(selected_, position, kPointHitRadius);
    if (chip != hoverChip_ || point != hoverPoint_) {
        hoverChip_ = chip;
        hoverPoint_ = point;
        update();
    }
    if (chip >= 0 && chip < series_.size() && series_.at(chip).enabled) {
        setCursor(Qt::PointingHandCursor);
    } else if (point >= 0 && series_.at(selected_).editable) {
        setCursor(Qt::SizeVerCursor);
    } else {
        unsetCursor();
    }
}

bool CurveEditor::event(QEvent *event)
{
    if (event->type() == QEvent::ToolTip) {
        auto *helpEvent = static_cast<QHelpEvent *>(event);
        const int chip = chipAt(helpEvent->pos());
        if (chip >= 0 && chip < series_.size()) {
            const CurveSeries &series = series_.at(chip);
            QString text;
            if (!series.enabled && !series.disabledNote.isEmpty()) {
                text = series.disabledNote;
            } else if (!series.description.isEmpty()) {
                text = QStringLiteral("<b>%1</b> — %2")
                           .arg(series.label, series.description);
            }
            if (!text.isEmpty()) {
                QToolTip::showText(helpEvent->globalPos(), text, this);
                return true;
            }
        }
        QToolTip::hideText();
    }
    return QWidget::event(event);
}

void CurveEditor::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    painter.setPen(QPen(kBorder, 1.0));
    painter.setBrush(kBackground);
    painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                            6.0, 6.0);

    const QFontMetrics metrics(font());
    for (int i = 0; i < chipRects_.size() && i < series_.size(); ++i) {
        const QRect chip = chipRects_.at(i);
        const CurveSeries &series = series_.at(i);
        const bool isSelected = i == selected_;
        const bool isEnabled = series.enabled;
        painter.setPen(QPen(isSelected && isEnabled ? series.color
                                                    : kChipBorder,
                            1.0));
        painter.setBrush(!isEnabled ? kBackground
                         : isSelected
                             ? kChipSelected
                         : i == hoverChip_
                             ? kChipSelected.darker(150)
                             : kChipBackground);
        painter.drawRoundedRect(QRectF(chip).adjusted(0.5, 0.5, -0.5, -0.5),
                                5.0, 5.0);
        QPen swatchPen(isEnabled ? series.color : kDisabledSwatch, 2.0,
                       series.penStyle);
        painter.setPen(swatchPen);
        const double swatchY = chip.center().y() + 0.5;
        painter.drawLine(QPointF(chip.left() + 8, swatchY),
                         QPointF(chip.left() + 24, swatchY));
        painter.setPen(!isEnabled       ? kDisabledInk
                       : isSelected     ? kBrightInk
                                        : kAxisInk);
        painter.drawText(chip.adjusted(30, 0, -6, 0),
                         Qt::AlignVCenter | Qt::AlignLeft, series.label);
    }

    const QRectF plot = plotRect();
    if (!message_.isEmpty() || series_.isEmpty()) {
        painter.setPen(kAxisInk);
        painter.drawText(
            QRectF(12, 10.0 + chipsHeight_, width() - 24,
                   height() - chipsHeight_ - 20)
                .toRect(),
            Qt::AlignCenter | Qt::TextWordWrap,
            message_.isEmpty() ? QStringLiteral("No curves") : message_);
        return;
    }
    if (plot.width() < 40 || plot.height() < 40)
        return;

    // Grid and value axis: real values for the selected series, plain
    // quarter-height guides when nothing is selected yet.
    painter.setPen(QPen(kGrid, 1.0));
    if (selected_ >= 0) {
        const CurveSeries &series = series_.at(selected_);
        const ValueRange range =
            dragging_ ? dragRange_ : paddedRange(selected_);
        const double step = niceStep((range.high - range.low) / 4.0);
        painter.setFont(font());
        for (double tick = std::ceil(range.low / step) * step;
             tick <= range.high + step * 0.001; tick += step) {
            const double y = valueToPixel(tick, range, plot);
            if (y < plot.top() - 0.5 || y > plot.bottom() + 0.5)
                continue;
            painter.setPen(QPen(kGrid, 1.0));
            painter.drawLine(QPointF(plot.left(), y),
                             QPointF(plot.right(), y));
            painter.setPen(kAxisInk);
            const int tickDecimals =
                step >= 1.0
                    ? 0
                    : static_cast<int>(std::ceil(-std::log10(step)));
            painter.drawText(
                QRectF(2, y - metrics.height() / 2.0, plot.left() - 8,
                       metrics.height()),
                Qt::AlignRight | Qt::AlignVCenter,
                formatValue(tick, tickDecimals));
        }
        painter.setPen(kHeaderInk);
        const QString header =
            series.unit.isEmpty()
                ? series.label
                : QStringLiteral("%1 (%2)").arg(series.label, series.unit);
        painter.drawText(QPointF(plot.left() + 6, plot.top() + metrics.ascent()
                                                      + 2),
                         header);
    } else {
        for (int i = 1; i < 4; ++i) {
            const double y = plot.top() + plot.height() * i / 4.0;
            painter.drawLine(QPointF(plot.left(), y),
                             QPointF(plot.right(), y));
        }
        painter.setPen(kAxisInk);
        painter.drawText(QPointF(plot.left() + 6,
                                 plot.top() + metrics.ascent() + 2),
                         QStringLiteral(
                             "Click a curve or its name to select and edit"));
    }

    // X axis ticks.
    {
        double step = niceStep((xMax_ - xMin_) / 8.0);
        if (step < 1.0 && xMax_ - xMin_ >= 2.0)
            step = 1.0;
        painter.setPen(kAxisInk);
        for (double tick = std::ceil(xMin_ / step) * step;
             tick <= xMax_ + step * 0.001; tick += step) {
            const double x = xToPixel(tick, plot);
            painter.setPen(QPen(kGrid, 1.0));
            painter.drawLine(QPointF(x, plot.bottom()),
                             QPointF(x, plot.bottom() + 3));
            painter.setPen(kAxisInk);
            painter.drawText(QRectF(x - 30, plot.bottom() + 4, 60,
                                    metrics.height()),
                             Qt::AlignHCenter | Qt::AlignTop,
                             formatValue(tick, step < 1.0 ? 1 : 0));
        }
        if (!xAxisLabel_.isEmpty()) {
            painter.drawText(QRectF(plot.left(), plot.bottom() + 4,
                                    plot.width() - 4, metrics.height()),
                             Qt::AlignRight | Qt::AlignTop, xAxisLabel_);
        }
    }

    painter.setClipRect(plot.adjusted(-8, -8, 8, 8));

    // Unselected curves first, dimmed; the selected curve on top with its
    // point handles.
    for (int pass = 0; pass < 2; ++pass) {
        for (int s = 0; s < series_.size(); ++s) {
            const bool isSelected = s == selected_;
            if ((pass == 0) == isSelected)
                continue;
            const CurveSeries &series = series_.at(s);
            if (!series.enabled
                || (series.points.isEmpty() && series.smooth.isEmpty()))
                continue;
            const ValueRange range = dragging_ && isSelected
                                         ? dragRange_
                                         : paddedRange(s);
            const auto toPixels = [&](const QVector<QPointF> &set) {
                QPolygonF result;
                result.reserve(set.size());
                for (const QPointF &point : set) {
                    result.append(
                        QPointF(xToPixel(point.x(), plot),
                                valueToPixel(point.y(), range, plot)));
                }
                return result;
            };
            const QPolygonF polyline =
                toPixels(series.smooth.isEmpty() ? series.points
                                                 : series.smooth);
            QColor color = series.color;
            if (!isSelected)
                color.setAlpha(kDimmedAlpha);
            QPen pen(color, isSelected ? 2.2 : 1.4, series.penStyle);
            pen.setCapStyle(Qt::RoundCap);
            painter.setPen(pen);
            painter.setBrush(Qt::NoBrush);
            painter.drawPolyline(polyline);

            if (!isSelected)
                continue;

            const bool splineMode = !series.handles.isEmpty();
            if (splineMode) {
                // Passive data markers: the spline samples the matrix rows.
                painter.setPen(Qt::NoPen);
                QColor dot = series.color;
                dot.setAlpha(190);
                painter.setBrush(dot);
                for (const QPointF &pixel : toPixels(series.points))
                    painter.drawEllipse(pixel, 2.4, 2.4);
            }

            // Editable markers: control-point squares in spline mode,
            // round data handles otherwise.
            const QPolygonF editablePixels = toPixels(series.editableSet());
            if (splineMode) {
                QColor polygonColor = series.color;
                polygonColor.setAlpha(150);
                painter.setPen(QPen(polygonColor, 1.0, Qt::DashLine));
                painter.setBrush(Qt::NoBrush);
                painter.drawPolyline(editablePixels);
            }
            for (int i = 0; i < editablePixels.size(); ++i) {
                const bool emphasized =
                    i == hoverPoint_ || i == activePoint_
                    || (dragging_ && i == dragPoint_);
                painter.setPen(QPen(kBackground, 2.0));
                painter.setBrush(series.color);
                const double radius = emphasized ? 6.0 : 4.0;
                if (splineMode) {
                    painter.drawRect(QRectF(
                        editablePixels.at(i).x() - radius,
                        editablePixels.at(i).y() - radius, radius * 2.0,
                        radius * 2.0));
                } else {
                    painter.drawEllipse(editablePixels.at(i), radius,
                                        radius);
                }
            }

            // Value bubble for the hovered or dragged editable marker.
            const int bubblePoint = dragging_ ? dragPoint_ : hoverPoint_;
            if (bubblePoint >= 0
                && bubblePoint < series.editableSet().size()) {
                const QPointF anchor = editablePixels.at(bubblePoint);
                const QPointF dataPoint =
                    series.editableSet().at(bubblePoint);
                const bool integralX =
                    std::abs(dataPoint.x() - std::round(dataPoint.x()))
                    < 1e-6;
                QString text = QStringLiteral("%1 %2 · %3")
                                   .arg(xAxisLabel_.isEmpty()
                                            ? QStringLiteral("x")
                                            : xAxisLabel_)
                                   .arg(formatValue(dataPoint.x(),
                                                    integralX ? 0 : 2))
                                   .arg(formatValue(dataPoint.y(),
                                                    series.decimals));
                if (!series.unit.isEmpty())
                    text += QLatin1Char(' ') + series.unit;
                const QSizeF textSize(
                    metrics.horizontalAdvance(text) + 14.0,
                    metrics.height() + 8.0);
                QPointF corner(anchor.x() + 12.0,
                               anchor.y() - textSize.height() - 8.0);
                if (corner.x() + textSize.width() > plot.right())
                    corner.setX(anchor.x() - textSize.width() - 12.0);
                if (corner.y() < plot.top())
                    corner.setY(anchor.y() + 12.0);
                const QRectF bubble(corner, textSize);
                painter.setPen(QPen(series.color, 1.0));
                painter.setBrush(kBubbleBackground);
                painter.drawRoundedRect(bubble, 4.0, 4.0);
                painter.setPen(kBrightInk);
                painter.drawText(bubble, Qt::AlignCenter, text);
            }
        }
    }
}

void CurveEditor::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    rebuildChips();
}

void CurveEditor::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }
    const int chip = chipAt(event->position().toPoint());
    if (chip >= 0) {
        selectSeries(chip);
        return;
    }
    if (!message_.isEmpty() || series_.isEmpty())
        return;
    if (selected_ >= 0 && series_.at(selected_).editable) {
        const int point =
            pointNear(selected_, event->position(), kPointHitRadius);
        if (point >= 0) {
            dragging_ = true;
            dragPoint_ = point;
            activePoint_ = point;
            hoverPoint_ = point;
            dragRange_ = paddedRange(selected_);
            setCursor(Qt::SizeVerCursor);
            update();
            return;
        }
    }
    const int nearest = seriesNear(event->position(), 9.0);
    if (nearest >= 0)
        selectSeries(nearest);
}

void CurveEditor::mouseMoveEvent(QMouseEvent *event)
{
    if (!dragging_) {
        updateHover(event->position());
        return;
    }
    const QRectF plot = plotRect();
    CurveSeries &series = series_[selected_];
    const double pixelY =
        std::clamp(event->position().y(), plot.top(), plot.bottom());
    const double value =
        std::clamp(pixelToValue(pixelY, dragRange_, plot), series.minValue,
                   series.maxValue);
    series.editableSet()[dragPoint_].setY(value);
    update();
    emit pointMoved(series.id, dragPoint_, value);
}

void CurveEditor::mouseReleaseEvent(QMouseEvent *event)
{
    if (!dragging_ || event->button() != Qt::LeftButton) {
        QWidget::mouseReleaseEvent(event);
        return;
    }
    dragging_ = false;
    dragPoint_ = -1;
    unsetCursor();
    update();
    const QString id = selectedSeriesId();
    emit editCommitted(id); // may re-enter setSeriesList(); keep this last
}

void CurveEditor::keyPressEvent(QKeyEvent *event)
{
    if (event->matches(QKeySequence::Undo)) {
        emit undoRequested();
        return;
    }
    if (event->matches(QKeySequence::Redo)) {
        emit redoRequested();
        return;
    }
    const bool large = event->modifiers().testFlag(Qt::ShiftModifier);
    switch (event->key()) {
    case Qt::Key_Up:
        nudgeActivePoint(1.0, large);
        return;
    case Qt::Key_Down:
        nudgeActivePoint(-1.0, large);
        return;
    case Qt::Key_Left:
    case Qt::Key_Right:
        if (selected_ >= 0 && !series_.at(selected_).points.isEmpty()) {
            const int pointCount = series_.at(selected_).points.size();
            const int step = event->key() == Qt::Key_Right ? 1 : -1;
            activePoint_ = activePoint_ < 0
                               ? (step > 0 ? 0 : pointCount - 1)
                               : std::clamp(activePoint_ + step, 0,
                                            pointCount - 1);
            update();
            return;
        }
        break;
    default:
        break;
    }
    QWidget::keyPressEvent(event);
}

void CurveEditor::leaveEvent(QEvent *event)
{
    QWidget::leaveEvent(event);
    hoverChip_ = -1;
    if (!dragging_)
        hoverPoint_ = -1;
    update();
}
