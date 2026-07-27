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
    // Full section explanation from the official LEparagliding manual
    // (laboratoridenvol.com), bundled as a Qt resource; empty when the
    // manual has no chapter for the section.
    QString manual;
};

SectionHelp helpForSection(int number, const QString &fallbackTitle);
