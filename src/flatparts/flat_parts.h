#pragma once

#include <QPointF>
#include <QRectF>
#include <QString>
#include <QVector>

// Reader for lep-2d-parts.json, the engine's flat-pattern export (written by
// src/flatparts/flat_capture.cpp). One FlatPiece is one piece of fabric: the
// unit the nester places and the printer draws.
namespace flatparts {

enum class Role
{
    Cut,  // the outline to cut along; already includes the seam allowance
    Seam, // the stitch line inside it, where drawn
    Mark, // everything else: registration ticks, vent outlines, rod positions
};

struct Polyline
{
    Role role = Role::Mark;
    bool closed = false;
    QVector<QPointF> points;
};

struct Circle
{
    QPointF centre;
    double radius = 0.0;
};

struct Label
{
    QPointF anchor;
    double height = 0.0;
    QString text;
};

struct FlatPiece
{
    QString id;       // "extrados-panel-3-2"
    QString category; // "extrados-panel"
    int index = 0;    // the number printed on the plan
    int subIndex = 0; // strip within one record, 0 when there is only one
    int piece = 0;    // piece within one strip, 0 when there is only one
    QString box;      // plan box it was drawn in, e.g. "1-3"

    QVector<Polyline> polylines;
    QVector<Circle> circles;
    QVector<Label> labels;

    // Millimetres, y-up, origin at the piece's own bottom-left.
    QSizeF size;
    // Direction the fabric warp should run, as drawn. Rotating a piece during
    // packing rotates this with it, so the printed arrow still points along the
    // grain.
    double grainAngleDeg = 90.0;

    QRectF bounds() const { return QRectF(QPointF(0.0, 0.0), size); }
    double area() const { return size.width() * size.height(); }
    // The outline the nester packs against, and the printer strokes solid.
    QVector<Polyline> cutOutline() const;
};

struct FlatPartSet
{
    QString wing;
    double drawingScale = 1.0;
    // Square metres, 0 when the export predates the field. Lets the Print tab
    // offer "scale to this flat area" alongside a plain percentage.
    double flatArea = 0.0;
    double projectedArea = 0.0;
    QVector<FlatPiece> pieces;

    bool isEmpty() const { return pieces.isEmpty(); }
    // Categories in a stable, build-order presentation order.
    QVector<QString> categories() const;
    static QString categoryLabel(const QString &category);
};

// Returns false and sets errorMessage when the file is missing or malformed.
bool load(const QString &path, FlatPartSet *set, QString *errorMessage);

} // namespace flatparts
