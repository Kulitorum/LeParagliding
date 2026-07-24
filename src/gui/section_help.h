#pragma once

#include <QString>

struct SectionHelp
{
    QString title;
    QString purpose;
    QString format;
    QString notes;
    QString details;
    QString experiment;
};

SectionHelp helpForSection(int number, const QString &fallbackTitle);
