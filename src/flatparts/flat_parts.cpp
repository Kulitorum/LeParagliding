#include "flat_parts.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMap>

namespace flatparts {
namespace {

Role roleFromName(const QString &name)
{
    if (name == QLatin1String("cut")) {
        return Role::Cut;
    }
    if (name == QLatin1String("seam")) {
        return Role::Seam;
    }
    return Role::Mark;
}

// Presentation order for the part tree: the big structural pieces first, then
// the small reinforcements, roughly the order they are sewn.
const QVector<QString> &categoryOrder()
{
    static const QVector<QString> order{
        QStringLiteral("extrados-panel"),
        QStringLiteral("intrados-panel"),
        QStringLiteral("rib"),
        QStringLiteral("middle-rib"),
        QStringLiteral("mini-rib"),
        QStringLiteral("v-rib-type2"),
        QStringLiteral("v-rib-type3"),
        QStringLiteral("v-rib-type5"),
        QStringLiteral("v-rib-type6"),
        QStringLiteral("vh-rib"),
        QStringLiteral("h-rib"),
        QStringLiteral("rod-pocket"),
        QStringLiteral("nose-mylar"),
    };
    return order;
}

} // namespace

QVector<Polyline> FlatPiece::cutOutline() const
{
    QVector<Polyline> outline;
    for (const Polyline &polyline : polylines) {
        if (polyline.role == Role::Cut) {
            outline.append(polyline);
        }
    }
    return outline;
}

QString FlatPartSet::categoryLabel(const QString &category)
{
    static const QMap<QString, QString> labels{
        {QStringLiteral("extrados-panel"), QStringLiteral("Extrados panels")},
        {QStringLiteral("intrados-panel"), QStringLiteral("Intrados panels")},
        {QStringLiteral("rib"), QStringLiteral("Ribs")},
        {QStringLiteral("middle-rib"), QStringLiteral("Middle unloaded ribs")},
        {QStringLiteral("mini-rib"), QStringLiteral("Mini-ribs")},
        {QStringLiteral("v-rib-type2"), QStringLiteral("V-ribs (type 2)")},
        {QStringLiteral("v-rib-type3"), QStringLiteral("V-ribs (type 3)")},
        {QStringLiteral("v-rib-type5"), QStringLiteral("V-ribs (type 5)")},
        {QStringLiteral("v-rib-type6"), QStringLiteral("V-ribs (type 6)")},
        {QStringLiteral("vh-rib"), QStringLiteral("VH-ribs")},
        {QStringLiteral("h-rib"), QStringLiteral("H-ribs / straps")},
        {QStringLiteral("rod-pocket"), QStringLiteral("Nylon rod pockets")},
        {QStringLiteral("nose-mylar"), QStringLiteral("Nose mylars")},
    };
    return labels.value(category, category);
}

QVector<QString> FlatPartSet::categories() const
{
    QVector<QString> present;
    for (const QString &category : categoryOrder()) {
        for (const FlatPiece &piece : pieces) {
            if (piece.category == category) {
                present.append(category);
                break;
            }
        }
    }
    // Anything the export gains later still shows up, just at the end.
    for (const FlatPiece &piece : pieces) {
        if (!present.contains(piece.category)) {
            present.append(piece.category);
        }
    }
    return present;
}

bool load(const QString &path, FlatPartSet *set, QString *errorMessage)
{
    const auto fail = [errorMessage](const QString &message) {
        if (errorMessage != nullptr) {
            *errorMessage = message;
        }
        return false;
    };

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return fail(QStringLiteral("Could not open %1: %2")
                        .arg(path, file.errorString()));
    }

    QJsonParseError parseError{};
    const QJsonDocument document =
        QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        return fail(QStringLiteral("%1 is not valid JSON: %2")
                        .arg(path, parseError.errorString()));
    }
    const QJsonObject root = document.object();

    set->wing = root.value(QStringLiteral("wing")).toString();
    set->drawingScale =
        root.value(QStringLiteral("drawingScale")).toDouble(1.0);
    set->flatArea = root.value(QStringLiteral("flatArea")).toDouble();
    set->projectedArea =
        root.value(QStringLiteral("projectedArea")).toDouble();
    set->pieces.clear();

    const QJsonArray parts = root.value(QStringLiteral("parts")).toArray();
    set->pieces.reserve(parts.size());
    for (const QJsonValue &value : parts) {
        const QJsonObject object = value.toObject();
        FlatPiece piece;
        piece.id = object.value(QStringLiteral("id")).toString();
        piece.category = object.value(QStringLiteral("category")).toString();
        piece.index = object.value(QStringLiteral("index")).toInt();
        piece.subIndex = object.value(QStringLiteral("subIndex")).toInt();
        piece.piece = object.value(QStringLiteral("piece")).toInt();
        piece.box = object.value(QStringLiteral("box")).toString();
        piece.size = QSizeF(object.value(QStringLiteral("width")).toDouble(),
                            object.value(QStringLiteral("height")).toDouble());
        piece.grainAngleDeg =
            object.value(QStringLiteral("grainAngleDeg")).toDouble(90.0);

        const QJsonArray polylines =
            object.value(QStringLiteral("polylines")).toArray();
        piece.polylines.reserve(polylines.size());
        for (const QJsonValue &polylineValue : polylines) {
            const QJsonObject polylineObject = polylineValue.toObject();
            Polyline polyline;
            polyline.role = roleFromName(
                polylineObject.value(QStringLiteral("role")).toString());
            polyline.closed =
                polylineObject.value(QStringLiteral("closed")).toBool();
            const QJsonArray points =
                polylineObject.value(QStringLiteral("pts")).toArray();
            polyline.points.reserve(points.size());
            for (const QJsonValue &pointValue : points) {
                const QJsonArray pair = pointValue.toArray();
                if (pair.size() == 2) {
                    polyline.points.append(
                        QPointF(pair.at(0).toDouble(), pair.at(1).toDouble()));
                }
            }
            if (polyline.points.size() >= 2) {
                piece.polylines.append(polyline);
            }
        }

        const QJsonArray circles =
            object.value(QStringLiteral("circles")).toArray();
        for (const QJsonValue &circleValue : circles) {
            const QJsonObject circleObject = circleValue.toObject();
            piece.circles.append(
                Circle{QPointF(circleObject.value(QStringLiteral("x")).toDouble(),
                               circleObject.value(QStringLiteral("y")).toDouble()),
                       circleObject.value(QStringLiteral("r")).toDouble()});
        }

        const QJsonArray labels =
            object.value(QStringLiteral("texts")).toArray();
        for (const QJsonValue &labelValue : labels) {
            const QJsonObject labelObject = labelValue.toObject();
            piece.labels.append(
                Label{QPointF(labelObject.value(QStringLiteral("x")).toDouble(),
                              labelObject.value(QStringLiteral("y")).toDouble()),
                      labelObject.value(QStringLiteral("h")).toDouble(),
                      labelObject.value(QStringLiteral("s")).toString()});
        }

        if (!piece.polylines.isEmpty()) {
            set->pieces.append(piece);
        }
    }

    if (set->pieces.isEmpty()) {
        return fail(QStringLiteral("%1 contains no parts.").arg(path));
    }
    return true;
}

} // namespace flatparts
