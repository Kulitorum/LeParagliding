#include "flat_parts_view.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QTransform>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace {

// Gap between pieces in the review layout, in millimetres. Generous, because
// the point of this layout is telling neighbouring parts apart, not saving
// space — the packer is what saves space.
constexpr double reviewGap = 60.0;

constexpr double minimumZoom = 0.02;
constexpr double maximumZoom = 40.0;

QColor roleColour(flatparts::Role role, bool highlighted)
{
    switch (role) {
    case flatparts::Role::Cut:
        return highlighted ? QColor(255, 138, 128) : QColor(214, 90, 78);
    case flatparts::Role::Seam:
        return QColor(90, 160, 205);
    case flatparts::Role::Mark:
        break;
    }
    return QColor(130, 130, 138);
}

} // namespace

FlatPartsView::FlatPartsView(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(320, 240);
    setMouseTracking(true);
    setCursor(Qt::OpenHandCursor);
    setAutoFillBackground(true);
    QPalette palette = this->palette();
    palette.setColor(QPalette::Window, QColor(26, 27, 30));
    setPalette(palette);
}

void FlatPartsView::setParts(const flatparts::FlatPartSet &parts)
{
    parts_ = parts;
    rebuildReviewLayout();
    needsFit_ = true;
    update();
}

void FlatPartsView::setSelection(const QSet<QString> &selectedIds)
{
    const bool wasEmpty = layout_.isEmpty();
    selected_ = selectedIds;
    rebuildReviewLayout();
    // First real selection is what makes a fit possible.
    if (wasEmpty && !layout_.isEmpty()) {
        needsFit_ = true;
    }
    update();
}

void FlatPartsView::setHighlighted(const QString &id)
{
    if (highlighted_ != id) {
        highlighted_ = id;
        update();
    }
}

void FlatPartsView::setScale(double factor)
{
    if (factor > 0.0 && !qFuzzyCompare(exportScale_, factor)) {
        exportScale_ = factor;
        rebuildReviewLayout();
        needsFit_ = true;
        update();
    }
}

void FlatPartsView::setPackedLayout(const flatparts::NestResult &result,
                                    const flatparts::NestOptions &options)
{
    const bool wasPacked = packed_;
    packed_ = true;
    pack_ = result;
    packOptions_ = options;
    // Only fit the first time. A pack publishes a new best every few hundred
    // milliseconds, and re-fitting on each would fight the user's zoom while
    // they are trying to look at something.
    if (!wasPacked) {
        needsFit_ = true;
    }
    update();
}

void FlatPartsView::clearPackedLayout()
{
    if (!packed_) {
        return;
    }
    packed_ = false;
    pack_ = flatparts::NestResult();
    needsFit_ = true;
    update();
}

void FlatPartsView::rebuildReviewLayout()
{
    layout_.clear();
    if (parts_.isEmpty()) {
        return;
    }

    // Wrap into rows about as wide as they are tall overall, so the review
    // layout stays roughly square whatever the selection is and the fit-zoom
    // makes reasonable use of the widget.
    double totalArea = 0.0;
    double tallest = 0.0;
    for (const flatparts::FlatPiece &piece : parts_.pieces) {
        if (!selected_.contains(piece.id)) {
            continue;
        }
        totalArea += (piece.size.width() + reviewGap)
            * (piece.size.height() + reviewGap);
        tallest = std::max(tallest, piece.size.height());
    }
    if (totalArea <= 0.0) {
        return;
    }
    const double rowWidth = std::max(std::sqrt(totalArea) * 1.4, tallest);

    double x = 0.0;
    double y = 0.0;
    double rowHeight = 0.0;
    for (int index = 0; index < parts_.pieces.size(); ++index) {
        const flatparts::FlatPiece &piece = parts_.pieces.at(index);
        if (!selected_.contains(piece.id)) {
            continue;
        }
        if (x > 0.0 && x + piece.size.width() > rowWidth) {
            x = 0.0;
            y += rowHeight + reviewGap;
            rowHeight = 0.0;
        }
        layout_.append(Placement{index, QPointF(x, y)});
        x += piece.size.width() + reviewGap;
        rowHeight = std::max(rowHeight, piece.size.height());
    }
}

QRectF FlatPartsView::contentBounds() const
{
    if (packed_) {
        return QRectF(0.0, 0.0, pack_.canvasWidthMm, pack_.canvasHeightMm);
    }
    QRectF bounds;
    for (const Placement &placement : layout_) {
        const flatparts::FlatPiece &piece = parts_.pieces.at(placement.pieceIndex);
        const QRectF box(placement.offset, piece.size);
        bounds = bounds.isNull() ? box : bounds.united(box);
    }
    return bounds;
}

// The review layout holds unscaled part coordinates, so the export scale is
// applied for display; a packed canvas already has it baked in by the packer,
// and scaling again would show a layout at the square of the scale.
double FlatPartsView::contentScale() const
{
    return packed_ ? 1.0 : exportScale_;
}

QTransform FlatPartsView::viewTransform() const
{
    // Millimetres in, device pixels out, y flipped: the parts are y-up.
    QTransform transform;
    transform.translate(width() / 2.0 + pan_.x(), height() / 2.0 + pan_.y());
    transform.scale(zoom_ * contentScale(), -zoom_ * contentScale());
    const QRectF bounds = contentBounds();
    transform.translate(-bounds.center().x(), -bounds.center().y());
    return transform;
}

void FlatPartsView::zoomToFit()
{
    needsFit_ = false;
    const QRectF bounds = contentBounds();
    if (bounds.isEmpty()) {
        zoom_ = 1.0;
        pan_ = QPointF();
        update();
        return;
    }
    const double margin = 1.08;
    const double fitX = width() / (bounds.width() * contentScale() * margin);
    const double fitY = height() / (bounds.height() * contentScale() * margin);
    zoom_ = std::clamp(std::min(fitX, fitY), minimumZoom, maximumZoom);
    pan_ = QPointF();
    update();
}

void FlatPartsView::drawPiece(QPainter &painter,
                              const flatparts::FlatPiece &piece,
                              const QPointF &offset,
                              bool highlighted) const
{
    painter.save();
    painter.translate(offset);

    // Line widths are set in device pixels via a cosmetic pen, so marks stay
    // legible at any zoom instead of vanishing when zoomed out.
    for (const flatparts::Polyline &polyline : piece.polylines) {
        QPainterPath path;
        path.moveTo(polyline.points.first());
        for (int index = 1; index < polyline.points.size(); ++index) {
            path.lineTo(polyline.points.at(index));
        }
        if (polyline.closed) {
            path.closeSubpath();
        }
        QPen pen(roleColour(polyline.role, highlighted));
        pen.setCosmetic(true);
        pen.setWidthF(polyline.role == flatparts::Role::Cut ? 1.6 : 1.0);
        if (polyline.role == flatparts::Role::Seam) {
            pen.setStyle(Qt::DashLine);
        }
        painter.setPen(pen);
        painter.drawPath(path);
    }

    QPen markPen(roleColour(flatparts::Role::Mark, highlighted));
    markPen.setCosmetic(true);
    painter.setPen(markPen);
    for (const flatparts::Circle &circle : piece.circles) {
        painter.drawEllipse(circle.centre, circle.radius, circle.radius);
    }

    painter.restore();
}

void FlatPartsView::drawPackedPiece(QPainter &painter,
                                    const flatparts::Placement &placement,
                                    bool highlighted) const
{
    if (placement.pieceIndex < 0
        || placement.pieceIndex >= parts_.pieces.size()) {
        return;
    }
    const flatparts::FlatPiece &piece = parts_.pieces.at(placement.pieceIndex);
    // Reuse the packer's own frame, so what is drawn is exactly what was
    // packed rather than a second opinion about rotation and origin.
    const flatparts::PlacementFrame frame =
        flatparts::frameFor(piece, placement, packOptions_.scale);

    for (const flatparts::Polyline &polyline : piece.polylines) {
        QPainterPath path;
        path.moveTo(frame.map(polyline.points.first()));
        for (int index = 1; index < polyline.points.size(); ++index) {
            path.lineTo(frame.map(polyline.points.at(index)));
        }
        if (polyline.closed) {
            path.closeSubpath();
        }
        QPen pen(roleColour(polyline.role, highlighted));
        pen.setCosmetic(true);
        pen.setWidthF(polyline.role == flatparts::Role::Cut ? 1.6 : 1.0);
        if (polyline.role == flatparts::Role::Seam) {
            pen.setStyle(Qt::DashLine);
        }
        painter.setPen(pen);
        painter.drawPath(path);
    }
}

void FlatPartsView::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), palette().window());

    if (needsFit_ && !layout_.isEmpty() && width() > 0 && height() > 0) {
        needsFit_ = false;
        zoomToFit();
    }

    if (packed_) {
        painter.setTransform(viewTransform());

        // Sheet grid first, so parts draw over it. Sheets advance by
        // (page - overlap), which is why the lines are not a plain page pitch.
        const double advanceX =
            std::max(packOptions_.pageWidthMm - packOptions_.overlapMm, 1.0);
        const double advanceY =
            std::max(packOptions_.pageHeightMm - packOptions_.overlapMm, 1.0);
        QPen gridPen(QColor(70, 74, 84));
        gridPen.setCosmetic(true);
        painter.setPen(gridPen);
        for (int i = 0; i < pack_.sheetsAcross; ++i) {
            const double x = i * advanceX;
            painter.drawRect(QRectF(x, 0.0, packOptions_.pageWidthMm,
                                    pack_.canvasHeightMm));
        }
        for (int i = 0; i < pack_.sheetsDown; ++i) {
            const double y = i * advanceY;
            painter.drawRect(QRectF(0.0, y, pack_.canvasWidthMm,
                                    packOptions_.pageHeightMm));
        }

        for (const flatparts::Placement &placement : pack_.placements) {
            const bool highlighted =
                placement.pieceIndex >= 0
                && placement.pieceIndex < parts_.pieces.size()
                && parts_.pieces.at(placement.pieceIndex).id == highlighted_;
            drawPackedPiece(painter, placement, highlighted);
        }

        painter.resetTransform();
        painter.setPen(QColor(170, 172, 180));
        painter.drawText(
            rect().adjusted(10, 8, -10, -8),
            Qt::AlignTop | Qt::AlignLeft,
            QStringLiteral("%1 parts · %2 x %3 mm · %4 x %5 sheets = %6 pages "
                           "· %7% used")
                .arg(pack_.placements.size())
                .arg(pack_.canvasWidthMm, 0, 'f', 0)
                .arg(pack_.canvasHeightMm, 0, 'f', 0)
                .arg(pack_.sheetsAcross)
                .arg(pack_.sheetsDown)
                .arg(pack_.pageCount)
                .arg(pack_.utilisation * 100.0, 0, 'f', 1));
        return;
    }

    if (layout_.isEmpty()) {
        painter.setPen(QColor(150, 150, 158));
        painter.drawText(rect(),
                         Qt::AlignCenter,
                         parts_.isEmpty()
                             ? QStringLiteral("Build the design to generate "
                                              "flat parts.")
                             : QStringLiteral("Select parts to preview."));
        return;
    }

    painter.setTransform(viewTransform());
    for (const Placement &placement : layout_) {
        const flatparts::FlatPiece &piece =
            parts_.pieces.at(placement.pieceIndex);
        drawPiece(painter, piece, placement.offset, piece.id == highlighted_);
    }

    painter.resetTransform();
    painter.setPen(QColor(170, 172, 180));
    painter.drawText(
        rect().adjusted(10, 8, -10, -8),
        Qt::AlignTop | Qt::AlignLeft,
        QStringLiteral("%1 parts · %2")
            .arg(layout_.size())
            .arg(qFuzzyCompare(exportScale_, 1.0)
                     ? QStringLiteral("full size")
                     : QStringLiteral("scaled %1%")
                           .arg(exportScale_ * 100.0, 0, 'f', 1)));
}

void FlatPartsView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        dragOrigin_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void FlatPartsView::mouseMoveEvent(QMouseEvent *event)
{
    if (dragging_) {
        pan_ += QPointF(event->pos() - dragOrigin_);
        dragOrigin_ = event->pos();
        update();
        return;
    }

    // Hover picking, so the tree and the viewport agree on what is under the
    // cursor.
    bool inverted = false;
    const QTransform inverse = viewTransform().inverted(&inverted);
    if (!inverted) {
        return;
    }
    const QPointF model = inverse.map(QPointF(event->pos()));
    QString hit;
    for (const Placement &placement : layout_) {
        const flatparts::FlatPiece &piece =
            parts_.pieces.at(placement.pieceIndex);
        if (QRectF(placement.offset, piece.size).contains(model)) {
            hit = piece.id;
            break;
        }
    }
    setHighlighted(hit);
}

void FlatPartsView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton) {
        return;
    }
    const bool moved = (event->pos() - dragOrigin_).manhattanLength() > 3;
    dragging_ = false;
    setCursor(Qt::OpenHandCursor);
    if (!moved && !highlighted_.isEmpty()) {
        emit pieceClicked(highlighted_);
    }
}

void FlatPartsView::wheelEvent(QWheelEvent *event)
{
    const double steps = event->angleDelta().y() / 120.0;
    if (qFuzzyIsNull(steps)) {
        return;
    }
    const double factor = std::pow(1.15, steps);
    const double next = std::clamp(zoom_ * factor, minimumZoom, maximumZoom);
    // Keep the point under the cursor put while zooming.
    const QPointF cursor = event->position();
    const QPointF centre(width() / 2.0, height() / 2.0);
    pan_ = cursor - (cursor - centre - pan_) * (next / zoom_) - centre;
    zoom_ = next;
    update();
}

void FlatPartsView::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    update();
}
