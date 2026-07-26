#pragma once

#include <QDateTime>
#include <QJsonObject>
#include <QList>
#include <QString>
#include <QStringList>

struct DesignSection
{
    int number = 0;
    QString title;
    QString text;
};

struct DesignRevision
{
    QString id;
    QString parentId;
    QDateTime savedAt;
    QString summary;
    QList<int> changedSections;
};

class DesignDocument
{
public:
    bool load(const QString &path, QString *errorMessage);
    bool save(QString *errorMessage);
    bool saveAs(const QString &path, QString *errorMessage);

    QString filePath() const;
    const QList<DesignSection> &sections() const;
    void setSectionText(int index, const QString &text);

    int revisionCount() const;
    QList<DesignRevision> revisions() const;
    QStringList sectionHistory(int sectionNumber, int *currentIndex = nullptr) const;
    bool restoreRevision(int revisionIndex, QString *errorMessage);
    QString savedSectionText(int sectionNumber) const;

    QString validationError() const;
    QString assembledText() const;
    bool isEmpty() const;

    // Studio-only data (B-spline curve definitions and similar) carried in
    // the embedded trailer JSON next to the version history; the engine
    // never sees it. An empty object means "none".
    QJsonObject splinesData() const;
    void setSplinesData(const QJsonObject &data);
    bool splinesDirty() const;

private:
    struct StoredRevision
    {
        DesignRevision metadata;
        QString payload;
    };

    bool replacePayload(const QString &payload, QString *errorMessage);
    QString serializedText() const;

    QString filePath_;
    QString preamble_;
    QList<DesignSection> sections_;
    QList<StoredRevision> revisions_;
    QJsonObject splines_;
    bool splinesDirty_ = false;
    QString savedPayload_;
    int activeRevisionIndex_ = -1;
    bool historyPersisted_ = false;
    bool historyDirty_ = false;
    bool finalNewline_ = true;
};
