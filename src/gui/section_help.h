#pragma once

#include <QString>

struct SectionHelp
{
    QString title;
    QString purpose;
    QString format;
    QString notes;
};

SectionHelp helpForSection(int number, const QString &fallbackTitle);
