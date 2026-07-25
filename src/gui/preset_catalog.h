#pragma once

#include <QList>
#include <QString>

struct PresetVariant
{
    QString label;
    QString designFile;
    QString sourceUrl;
};

struct PresetWing
{
    QString name;
    QString category;
    QString pageUrl;
    int year = 0;
    QList<PresetVariant> variants;
};

// Reads presets.json from the given directory (the "presets" folder shipped
// next to the executable). Variants whose design file is missing on disk are
// dropped; wings without remaining variants are dropped too.
QList<PresetWing> loadPresetCatalog(const QString &directory);
