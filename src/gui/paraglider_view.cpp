#include "paraglider_view.h"

#include <QFile>
#include <QFileInfo>
#include <QKeyEvent>
#include <QLinearGradient>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QSet>
#include <QStringConverter>
#include <QTextStream>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

constexpr float pi = 3.14159265358979323846F;

float radians(float degrees)
{
    return degrees * pi / 180.0F;
}

QColor colorForEntity(int colorIndex, const QString &layer)
{
    switch (std::abs(colorIndex)) {
    case 1:
        return QColor(QStringLiteral("#ff6b6b"));
    case 2:
        return QColor(QStringLiteral("#ffd166"));
    case 3:
        return QColor(QStringLiteral("#5de4a8"));
    case 4:
        return QColor(QStringLiteral("#55d8ff"));
    case 5:
        return QColor(QStringLiteral("#72a7ff"));
    case 6:
        return QColor(QStringLiteral("#d58cff"));
    case 7:
        return QColor(QStringLiteral("#e8f0fb"));
    default:
        break;
    }

    const QString lowered = layer.toLower();
    if (lowered.contains(QStringLiteral("line"))) {
        return QColor(QStringLiteral("#f6b85f"));
    }
    if (lowered.contains(QStringLiteral("rib"))) {
        return QColor(QStringLiteral("#76d6ff"));
    }
    return QColor(QStringLiteral("#9fc8ee"));
}

QVector3D displayPoint(float x, float y, float z)
{
    // LEparagliding uses Z pointing down. The viewport presents a conventional
    // Z-up scene while retaining X (span) and Y (chord) orientation.
    return QVector3D(x, y, -z);
}

float niceGridStep(float radius)
{
    const float raw = std::max(radius / 6.0F, 0.001F);
    const float magnitude = std::pow(10.0F, std::floor(std::log10(raw)));
    const float normalized = raw / magnitude;
    const float multiplier =
        normalized < 2.0F ? 1.0F : normalized < 5.0F ? 2.0F : 5.0F;
    return multiplier * magnitude;
}

} // namespace

ParagliderView::ParagliderView(QWidget *parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("paragliderViewport"));
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setMinimumSize(420, 320);
    setAttribute(Qt::WA_OpaquePaintEvent);
}

bool ParagliderView::loadDxf(const QString &path, QString *errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    struct LineRecord
    {
        float x1 = 0.0F;
        float y1 = 0.0F;
        float z1 = 0.0F;
        float x2 = 0.0F;
        float y2 = 0.0F;
        float z2 = 0.0F;
        int colorIndex = 0;
        QString layer;
        bool startX = false;
        bool startY = false;
        bool endX = false;
        bool endY = false;
    };

    QVector<ModelLine> parsedLines;
    parsedLines.reserve(8000);
    QSet<QString> layers;
    QVector3D minimum(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());
    QVector3D maximum(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest());

    bool readingLine = false;
    LineRecord record;
    auto finishRecord = [&] {
        if (!readingLine || !record.startX || !record.startY
            || !record.endX || !record.endY) {
            return;
        }

        ModelLine line;
        line.start = displayPoint(record.x1, record.y1, record.z1);
        line.end = displayPoint(record.x2, record.y2, record.z2);
        line.color = colorForEntity(record.colorIndex, record.layer);
        parsedLines.append(line);
        if (!record.layer.isEmpty()) {
            layers.insert(record.layer);
        }

        for (const QVector3D &point : {line.start, line.end}) {
            minimum.setX(std::min(minimum.x(), point.x()));
            minimum.setY(std::min(minimum.y(), point.y()));
            minimum.setZ(std::min(minimum.z(), point.z()));
            maximum.setX(std::max(maximum.x(), point.x()));
            maximum.setY(std::max(maximum.y(), point.y()));
            maximum.setZ(std::max(maximum.z(), point.z()));
        }
    };

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    while (!stream.atEnd()) {
        const QString codeText = stream.readLine().trimmed();
        if (stream.atEnd()) {
            break;
        }
        const QString valueText = stream.readLine().trimmed();
        bool codeOk = false;
        const int code = codeText.toInt(&codeOk);
        if (!codeOk) {
            continue;
        }

        if (code == 0) {
            finishRecord();
            readingLine = valueText.compare(
                              QStringLiteral("LINE"),
                              Qt::CaseInsensitive)
                          == 0;
            record = LineRecord{};
            continue;
        }
        if (!readingLine) {
            continue;
        }

        bool valueOk = false;
        switch (code) {
        case 8:
            record.layer = valueText;
            break;
        case 62:
            record.colorIndex = valueText.toInt(&valueOk);
            break;
        case 10:
            record.x1 = valueText.toFloat(&valueOk);
            record.startX = valueOk;
            break;
        case 20:
            record.y1 = valueText.toFloat(&valueOk);
            record.startY = valueOk;
            break;
        case 30:
            record.z1 = valueText.toFloat(&valueOk);
            break;
        case 11:
            record.x2 = valueText.toFloat(&valueOk);
            record.endX = valueOk;
            break;
        case 21:
            record.y2 = valueText.toFloat(&valueOk);
            record.endY = valueOk;
            break;
        case 31:
            record.z2 = valueText.toFloat(&valueOk);
            break;
        default:
            break;
        }
    }
    finishRecord();

    if (parsedLines.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                "The DXF contains no readable 3D LINE entities.");
        }
        return false;
    }

    lines_ = std::move(parsedLines);
    boundsMin_ = minimum;
    boundsMax_ = maximum;
    layerCount_ = layers.size();
    modelPath_ = QFileInfo(path).absoluteFilePath();
    setView(ViewPreset::Isometric);
    fitAll();
    update();
    return true;
}

void ParagliderView::clearModel()
{
    lines_.clear();
    layerCount_ = 0;
    modelPath_.clear();
    update();
}

void ParagliderView::fitAll()
{
    if (lines_.isEmpty()) {
        return;
    }
    target_ = (boundsMin_ + boundsMax_) * 0.5F;
    const float radius = sceneRadius();
    distance_ = std::max(radius / std::tan(radians(22.5F)) * 1.25F, 1.0F);
    orthographicScale_ = std::max(radius * 1.25F, 1.0F);
    update();
}

void ParagliderView::setView(ViewPreset preset)
{
    switch (preset) {
    case ViewPreset::Isometric:
        azimuthDegrees_ = -45.0F;
        elevationDegrees_ = 28.0F;
        break;
    case ViewPreset::Front:
        azimuthDegrees_ = -90.0F;
        elevationDegrees_ = 0.0F;
        break;
    case ViewPreset::Back:
        azimuthDegrees_ = 90.0F;
        elevationDegrees_ = 0.0F;
        break;
    case ViewPreset::Left:
        azimuthDegrees_ = 180.0F;
        elevationDegrees_ = 0.0F;
        break;
    case ViewPreset::Right:
        azimuthDegrees_ = 0.0F;
        elevationDegrees_ = 0.0F;
        break;
    case ViewPreset::Top:
        azimuthDegrees_ = -90.0F;
        elevationDegrees_ = 89.0F;
        break;
    case ViewPreset::Bottom:
        azimuthDegrees_ = -90.0F;
        elevationDegrees_ = -89.0F;
        break;
    }
    update();
}

void ParagliderView::setPerspective(bool enabled)
{
    perspective_ = enabled;
    update();
}

void ParagliderView::toggleProjection()
{
    setPerspective(!perspective_);
}

bool ParagliderView::isPerspective() const
{
    return perspective_;
}

bool ParagliderView::hasModel() const
{
    return !lines_.isEmpty();
}

qsizetype ParagliderView::segmentCount() const
{
    return lines_.size();
}

QString ParagliderView::modelSummary() const
{
    if (lines_.isEmpty()) {
        return QStringLiteral("No model loaded");
    }
    const QVector3D dimensions = boundsMax_ - boundsMin_;
    return QStringLiteral("%1 segments · %2 layers · %3 × %4 × %5 cm")
        .arg(lines_.size())
        .arg(layerCount_)
        .arg(dimensions.x(), 0, 'f', 1)
        .arg(dimensions.y(), 0, 'f', 1)
        .arg(dimensions.z(), 0, 'f', 1);
}

QSize ParagliderView::sizeHint() const
{
    return {760, 620};
}

void ParagliderView::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QLinearGradient background(0, 0, 0, height());
    background.setColorAt(0.0, QColor(QStringLiteral("#101b2b")));
    background.setColorAt(1.0, QColor(QStringLiteral("#07101c")));
    painter.fillRect(rect(), background);

    if (lines_.isEmpty()) {
        painter.setPen(QColor(QStringLiteral("#dce8f6")));
        QFont titleFont = painter.font();
        titleFont.setPointSizeF(titleFont.pointSizeF() + 3.0);
        titleFont.setBold(true);
        painter.setFont(titleFont);
        painter.drawText(
            rect().adjusted(30, 20, -30, -20),
            Qt::AlignCenter,
            QStringLiteral(
                "3D model viewport\n\nOpen a design or build the paraglider "
                "to calculate a fresh preview"));
        drawHud(painter);
        return;
    }

    const QMatrix4x4 matrix = viewProjectionMatrix();
    drawGrid(painter, matrix);

    for (const ModelLine &line : std::as_const(lines_)) {
        const ProjectedPoint start = project(line.start, matrix);
        const ProjectedPoint end = project(line.end, matrix);
        if (!start.visible || !end.visible) {
            continue;
        }
        QColor color = line.color;
        color.setAlpha(218);
        painter.setPen(QPen(color, 1.05, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(start.point, end.point);
    }

    drawAxes(painter, matrix);
    drawHud(painter);
}

void ParagliderView::mousePressEvent(QMouseEvent *event)
{
    setFocus(Qt::MouseFocusReason);
    previousMousePosition_ = event->position().toPoint();
    dragButton_ = event->button();
    shiftPan_ = event->modifiers().testFlag(Qt::ShiftModifier);
    updateCursor();
    event->accept();
}

void ParagliderView::mouseMoveEvent(QMouseEvent *event)
{
    if (dragButton_ == Qt::NoButton) {
        return;
    }

    const QPoint position = event->position().toPoint();
    const QPoint delta = position - previousMousePosition_;
    previousMousePosition_ = position;

    if (dragButton_ == Qt::RightButton || dragButton_ == Qt::MiddleButton
        || (dragButton_ == Qt::LeftButton && shiftPan_)) {
        panByPixels(delta);
    } else if (dragButton_ == Qt::LeftButton) {
        azimuthDegrees_ -= delta.x() * 0.45F;
        elevationDegrees_ = std::clamp(
            elevationDegrees_ + delta.y() * 0.45F,
            -89.0F,
            89.0F);
        update();
    }
    event->accept();
}

void ParagliderView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == dragButton_) {
        dragButton_ = Qt::NoButton;
        shiftPan_ = false;
        updateCursor();
    }
    event->accept();
}

void ParagliderView::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        fitAll();
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}

void ParagliderView::wheelEvent(QWheelEvent *event)
{
    const float steps = event->angleDelta().y() / 120.0F;
    const float factor = std::pow(0.84F, steps);
    const float radius = sceneRadius();
    if (perspective_) {
        distance_ = std::clamp(
            distance_ * factor,
            std::max(radius * 0.02F, 0.01F),
            std::max(radius * 100.0F, 100.0F));
    } else {
        orthographicScale_ = std::clamp(
            orthographicScale_ * factor,
            std::max(radius * 0.01F, 0.01F),
            std::max(radius * 100.0F, 100.0F));
    }
    update();
    event->accept();
}

void ParagliderView::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_F:
        fitAll();
        break;
    case Qt::Key_0:
        setView(ViewPreset::Isometric);
        fitAll();
        break;
    case Qt::Key_1:
        setView(ViewPreset::Front);
        fitAll();
        break;
    case Qt::Key_2:
        setView(ViewPreset::Back);
        fitAll();
        break;
    case Qt::Key_3:
        setView(ViewPreset::Left);
        fitAll();
        break;
    case Qt::Key_4:
        setView(ViewPreset::Right);
        fitAll();
        break;
    case Qt::Key_5:
        setView(ViewPreset::Top);
        fitAll();
        break;
    case Qt::Key_6:
        setView(ViewPreset::Bottom);
        fitAll();
        break;
    case Qt::Key_P:
        toggleProjection();
        break;
    default:
        QWidget::keyPressEvent(event);
        return;
    }
    event->accept();
}

QVector3D ParagliderView::cameraPosition() const
{
    const float azimuth = radians(azimuthDegrees_);
    const float elevation = radians(elevationDegrees_);
    const float horizontal = distance_ * std::cos(elevation);
    return target_
           + QVector3D(
               horizontal * std::cos(azimuth),
               horizontal * std::sin(azimuth),
               distance_ * std::sin(elevation));
}

ParagliderView::ProjectedPoint ParagliderView::project(
    const QVector3D &point,
    const QMatrix4x4 &matrix) const
{
    const QVector4D clip = matrix * QVector4D(point, 1.0F);
    if (perspective_ && clip.w() <= 0.00001F) {
        return {};
    }
    if (std::abs(clip.w()) <= 0.00001F) {
        return {};
    }

    const QVector3D normalized = clip.toVector3DAffine();
    ProjectedPoint result;
    result.point = QPointF(
        (normalized.x() * 0.5F + 0.5F) * width(),
        (0.5F - normalized.y() * 0.5F) * height());
    result.depth = normalized.z();
    result.visible = normalized.z() >= -1.2F && normalized.z() <= 1.2F;
    return result;
}

QMatrix4x4 ParagliderView::viewProjectionMatrix() const
{
    const float aspect = height() > 0 ? width() / static_cast<float>(height()) : 1.0F;
    const float radius = sceneRadius();

    QMatrix4x4 projection;
    if (perspective_) {
        const float nearPlane = std::max(radius * 0.001F, 0.01F);
        const float farPlane = std::max(distance_ + radius * 6.0F, nearPlane + 10.0F);
        projection.perspective(45.0F, aspect, nearPlane, farPlane);
    } else {
        const float vertical = std::max(orthographicScale_, 0.01F);
        projection.ortho(
            -vertical * aspect,
            vertical * aspect,
            -vertical,
            vertical,
            -radius * 20.0F - distance_,
            radius * 20.0F + distance_);
    }

    QMatrix4x4 view;
    const QVector3D position = cameraPosition();
    QVector3D up(0.0F, 0.0F, 1.0F);
    if (std::abs(elevationDegrees_) > 88.5F) {
        up = QVector3D(0.0F, 1.0F, 0.0F);
    }
    view.lookAt(position, target_, up);
    return projection * view;
}

void ParagliderView::drawGrid(QPainter &painter, const QMatrix4x4 &matrix) const
{
    const float radius = sceneRadius();
    const float step = niceGridStep(radius);
    const float extent = step * 8.0F;
    const float centerX = std::round(target_.x() / step) * step;
    const float centerY = std::round(target_.y() / step) * step;
    constexpr float gridZ = 0.0F;

    painter.setPen(QPen(QColor(77, 102, 132, 66), 1.0));
    for (int index = -8; index <= 8; ++index) {
        const float offset = index * step;
        const ProjectedPoint a = project(
            QVector3D(centerX - extent, centerY + offset, gridZ),
            matrix);
        const ProjectedPoint b = project(
            QVector3D(centerX + extent, centerY + offset, gridZ),
            matrix);
        const ProjectedPoint c = project(
            QVector3D(centerX + offset, centerY - extent, gridZ),
            matrix);
        const ProjectedPoint d = project(
            QVector3D(centerX + offset, centerY + extent, gridZ),
            matrix);
        if (a.visible && b.visible) {
            painter.drawLine(a.point, b.point);
        }
        if (c.visible && d.visible) {
            painter.drawLine(c.point, d.point);
        }
    }
}

void ParagliderView::drawAxes(QPainter &painter, const QMatrix4x4 &matrix) const
{
    const float length = std::max(sceneRadius() * 0.22F, 1.0F);
    const QVector3D origin(0.0F, 0.0F, 0.0F);
    const ProjectedPoint projectedOrigin = project(origin, matrix);
    if (!projectedOrigin.visible) {
        return;
    }

    struct Axis
    {
        QVector3D end;
        QColor color;
        QString label;
    };
    const Axis axes[] = {
        {QVector3D(length, 0.0F, 0.0F), QColor(QStringLiteral("#ff6b6b")), QStringLiteral("X")},
        {QVector3D(0.0F, length, 0.0F), QColor(QStringLiteral("#5de4a8")), QStringLiteral("Y")},
        {QVector3D(0.0F, 0.0F, length), QColor(QStringLiteral("#72a7ff")), QStringLiteral("Z")},
    };
    for (const Axis &axis : axes) {
        const ProjectedPoint end = project(axis.end, matrix);
        if (!end.visible) {
            continue;
        }
        painter.setPen(QPen(axis.color, 2.0, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(projectedOrigin.point, end.point);
        painter.drawText(end.point + QPointF(5.0, -4.0), axis.label);
    }
}

void ParagliderView::drawHud(QPainter &painter) const
{
    const QString projection =
        perspective_ ? QStringLiteral("Perspective") : QStringLiteral("Orthographic");
    const QString text =
        QStringLiteral("%1\nLMB orbit · RMB/MMB pan · Wheel zoom · Double-click fit")
            .arg(projection);
    const QRectF box(14.0, height() - 54.0, width() - 28.0, 40.0);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(4, 10, 18, 172));
    painter.drawRoundedRect(box, 6.0, 6.0);
    painter.setPen(QColor(QStringLiteral("#b8c9dc")));
    painter.drawText(box.adjusted(10.0, 4.0, -8.0, -3.0), Qt::AlignLeft | Qt::AlignVCenter, text);
}

void ParagliderView::panByPixels(const QPoint &delta)
{
    const QVector3D position = cameraPosition();
    const QVector3D forward = (target_ - position).normalized();
    QVector3D right = QVector3D::crossProduct(forward, QVector3D(0.0F, 0.0F, 1.0F));
    if (right.lengthSquared() < 0.00001F) {
        right = QVector3D(1.0F, 0.0F, 0.0F);
    } else {
        right.normalize();
    }
    const QVector3D up = QVector3D::crossProduct(right, forward).normalized();

    const float visibleHeight =
        perspective_
            ? 2.0F * distance_ * std::tan(radians(22.5F))
            : 2.0F * orthographicScale_;
    const float unitsPerPixel = visibleHeight / std::max(height(), 1);
    target_ += right * (-delta.x() * unitsPerPixel)
               + up * (delta.y() * unitsPerPixel);
    update();
}

void ParagliderView::updateCursor()
{
    if (dragButton_ == Qt::RightButton || dragButton_ == Qt::MiddleButton
        || shiftPan_) {
        setCursor(Qt::SizeAllCursor);
    } else if (dragButton_ == Qt::LeftButton) {
        setCursor(Qt::ClosedHandCursor);
    } else {
        unsetCursor();
    }
}

float ParagliderView::sceneRadius() const
{
    if (lines_.isEmpty()) {
        return 100.0F;
    }
    return std::max((boundsMax_ - boundsMin_).length() * 0.5F, 0.1F);
}
