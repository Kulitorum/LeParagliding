#include "preset_catalog.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

QList<PresetWing> loadPresetCatalog(const QString &directory)
{
    QFile manifest(QDir(directory).filePath(QStringLiteral("presets.json")));
    if (!manifest.open(QIODevice::ReadOnly)) {
        return {};
    }

    const QJsonDocument document = QJsonDocument::fromJson(manifest.readAll());
    if (!document.isObject()) {
        return {};
    }

    QList<PresetWing> catalog;
    const QJsonArray wings =
        document.object().value(QStringLiteral("wings")).toArray();
    for (const QJsonValue &wingValue : wings) {
        const QJsonObject wingObject = wingValue.toObject();
        PresetWing wing;
        wing.name = wingObject.value(QStringLiteral("name")).toString();
        wing.category = wingObject.value(QStringLiteral("category")).toString();
        wing.pageUrl = wingObject.value(QStringLiteral("page")).toString();
        wing.year = wingObject.value(QStringLiteral("year")).toInt();

        const QJsonArray variants =
            wingObject.value(QStringLiteral("variants")).toArray();
        for (const QJsonValue &variantValue : variants) {
            const QJsonObject variantObject = variantValue.toObject();
            PresetVariant variant;
            variant.label =
                variantObject.value(QStringLiteral("label")).toString();
            variant.sourceUrl =
                variantObject.value(QStringLiteral("source")).toString();
            const QString relative =
                variantObject.value(QStringLiteral("path")).toString();
            const QFileInfo file(QDir(directory).filePath(relative));
            if (relative.isEmpty() || !file.isFile()) {
                continue;
            }
            variant.designFile = file.absoluteFilePath();
            wing.variants.append(variant);
        }
        if (!wing.name.isEmpty() && !wing.variants.isEmpty()) {
            catalog.append(wing);
        }
    }
    return catalog;
}
