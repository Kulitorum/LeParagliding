#pragma once

#include <QSet>
#include <QString>
#include <QWidget>

#include "flat_parts.h"
#include "nesting.h"

class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

// The Print tab's 2D viewport. Two things to show, so it has two modes: the
// parts the user has selected, laid out in a plain grid for review, and — once
// packed — the nested sheet with the page grid drawn over it.
class FlatPartsView : public QWidget
{
    Q_OBJECT

public:
    explicit FlatPartsView(QWidget *parent = nullptr);

    void setParts(const flatparts::FlatPartSet &parts);
    // Ids to draw; everything else is hidden. Drives both modes.
    void setSelection(const QSet<QString> &selectedIds);
    void setHighlighted(const QString &id);
    // Millimetres per drawn millimetre — the export scale, so the preview
    // shows what will actually print.
    void setScale(double factor);

    // Switches to the packed layout: parts where the nester put them, with the
    // sheet grid over the top. Called repeatedly while a pack runs, so the view
    // keeps its zoom and pan between updates rather than snapping back.
    void setPackedLayout(const flatparts::NestResult &result,
                         const flatparts::NestOptions &options);
    void clearPackedLayout();
    bool isPacked() const { return packed_; }

    void zoomToFit();

signals:
    void pieceClicked(const QString &id);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    struct Placement
    {
        int pieceIndex = 0;
        QPointF offset; // millimetres, bottom-left of the piece
    };

    // Review layout: selected pieces in reading order, wrapped into rows sized
    // to the widget. Not the packing — just something legible to check against
    // the plan before committing to a pack.
    void rebuildReviewLayout();
    QRectF contentBounds() const;
    double contentScale() const;
    QTransform viewTransform() const;
    void drawPiece(QPainter &painter,
                   const flatparts::FlatPiece &piece,
                   const QPointF &offset,
                   bool highlighted) const;

    void drawPackedPiece(QPainter &painter,
                         const flatparts::Placement &placement,
                         bool highlighted) const;

    flatparts::FlatPartSet parts_;
    QSet<QString> selected_;
    QString highlighted_;
    QVector<Placement> layout_;
    double exportScale_ = 1.0;

    bool packed_ = false;
    flatparts::NestResult pack_;
    flatparts::NestOptions packOptions_;

    // A fit is only meaningful once there is a layout to fit and the widget has
    // its real size, and neither holds when parts first arrive. So the fit is
    // deferred to the next paint that has both, and not repeated afterwards —
    // otherwise toggling a part in the tree would yank the user's zoom back.
    bool needsFit_ = true;
    double zoom_ = 1.0;
    QPointF pan_;
    QPoint dragOrigin_;
    bool dragging_ = false;
};
