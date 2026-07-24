#pragma once

#include <QList>
#include <QString>

struct DesignSection
{
    int number = 0;
    QString title;
    QString text;
};

class DesignDocument
{
public:
    bool load(const QString &path, QString *errorMessage);
    bool save(QString *errorMessage) const;
    bool saveAs(const QString &path, QString *errorMessage);

    QString filePath() const;
    const QList<DesignSection> &sections() const;
    void setSectionText(int index, const QString &text);

    QString validationError() const;
    QString assembledText() const;
    bool isEmpty() const;

private:
    QString filePath_;
    QString preamble_;
    QList<DesignSection> sections_;
    bool finalNewline_ = true;
};
