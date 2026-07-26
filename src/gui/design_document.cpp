#include "design_document.h"

#include <QCryptographicHash>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QMap>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>

#include <algorithm>
#include <utility>

namespace {

constexpr auto historyFormat = "leparagliding-studio-history";
constexpr auto historyBegin =
    "* >>> LEPARAGLIDING STUDIO HISTORY V1 >>>";
constexpr auto historyEnd =
    "* <<< LEPARAGLIDING STUDIO HISTORY V1 <<<";
constexpr qsizetype historyLineLength = 120;

const QRegularExpression &sectionHeaderPattern()
{
    static const QRegularExpression pattern(
        QStringLiteral(R"(^[ \t]*\*[ \t]*(\d{1,2})\.[ \t]*([^\r\n*]*))"),
        QRegularExpression::MultilineOption);
    return pattern;
}

QString cleanTitle(QString title)
{
    title.replace(QLatin1Char('\t'), QLatin1Char(' '));
    return title.simplified();
}

QString normalizedText(const QByteArray &encoded)
{
    QString text = QString::fromUtf8(encoded);
    if (text.startsWith(QChar::ByteOrderMark)) {
        text.remove(0, 1);
    }
    text.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
    text.replace(QLatin1Char('\r'), QLatin1Char('\n'));
    return text;
}

struct EncodedHistory
{
    bool present = false;
    QString payload;
    QJsonObject root;
};

bool splitHistory(
    const QString &text,
    EncodedHistory *result,
    QString *errorMessage)
{
    result->payload = text;

    const QString beginLine = QString::fromLatin1(historyBegin);
    const QString endLine = QString::fromLatin1(historyEnd);
    qsizetype marker = text.indexOf(QStringLiteral("\n") + beginLine);
    if (marker < 0) {
        return true;
    }
    ++marker;

    const qsizetype beginEnd = text.indexOf(QLatin1Char('\n'), marker);
    const qsizetype endMarker =
        text.indexOf(QStringLiteral("\n") + endLine, beginEnd);
    if (beginEnd < 0 || endMarker < 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                "The embedded version history trailer is incomplete.");
        }
        return false;
    }

    QByteArray encoded;
    const QStringList lines =
        text.mid(beginEnd + 1, endMarker - beginEnd - 1)
            .split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    for (const QString &line : lines) {
        if (!line.startsWith(QStringLiteral("* "))) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "The embedded version history contains an invalid record.");
            }
            return false;
        }
        encoded.append(line.mid(2).trimmed().toLatin1());
    }

    const QByteArray json = QByteArray::fromBase64(
        encoded,
        QByteArray::AbortOnBase64DecodingErrors);
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(json, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                                "The embedded version history is damaged: %1")
                                .arg(parseError.errorString());
        }
        return false;
    }

    result->present = true;
    result->root = document.object();
    result->payload = text.left(marker);
    const bool payloadHadFinalNewline =
        result->root.value(QStringLiteral("payloadFinalNewline")).toBool(true);
    if (!payloadHadFinalNewline && result->payload.endsWith(QLatin1Char('\n'))) {
        result->payload.chop(1);
    }
    return true;
}

struct SectionLocation
{
    qsizetype start = 0;
    int number = 0;
    QString title;
};

QList<SectionLocation> sectionLocations(const QString &text)
{
    QList<SectionLocation> headers;
    auto matches = sectionHeaderPattern().globalMatch(text);
    while (matches.hasNext()) {
        const QRegularExpressionMatch match = matches.next();
        SectionLocation header;
        header.start = match.capturedStart();
        header.number = match.captured(1).toInt();
        header.title = cleanTitle(match.captured(2));
        headers.append(header);
    }
    return headers;
}

QMap<int, QString> sectionsByNumber(const QString &payload)
{
    QMap<int, QString> result;
    const QList<SectionLocation> headers = sectionLocations(payload);
    for (qsizetype index = 0; index < headers.size(); ++index) {
        const qsizetype end =
            index + 1 < headers.size() ? headers.at(index + 1).start : payload.size();
        result.insert(
            headers.at(index).number,
            payload.mid(headers.at(index).start, end - headers.at(index).start));
    }
    return result;
}

QList<int> changedSectionsBetween(
    const QString &before,
    const QString &after)
{
    const QMap<int, QString> oldSections = sectionsByNumber(before);
    const QMap<int, QString> newSections = sectionsByNumber(after);
    QSet<int> sectionNumbers;
    for (auto iterator = oldSections.cbegin(); iterator != oldSections.cend(); ++iterator) {
        sectionNumbers.insert(iterator.key());
    }
    for (auto iterator = newSections.cbegin(); iterator != newSections.cend(); ++iterator) {
        sectionNumbers.insert(iterator.key());
    }

    QList<int> sorted = sectionNumbers.values();
    std::sort(sorted.begin(), sorted.end());

    QList<int> changed;
    for (const int number : sorted) {
        if (oldSections.value(number) != newSections.value(number)) {
            changed.append(number);
        }
    }
    return changed;
}

QString revisionSummary(const QList<int> &changedSections)
{
    if (changedSections.isEmpty()) {
        return QStringLiteral("Changed wing metadata");
    }

    QStringList numbers;
    for (const int section : changedSections) {
        numbers.append(QString::number(section));
    }
    return QStringLiteral("Changed section%1 %2")
        .arg(changedSections.size() == 1 ? QString() : QStringLiteral("s"))
        .arg(numbers.join(QStringLiteral(", ")));
}

QString revisionId(
    const QString &parentId,
    const QDateTime &savedAt,
    const QString &payload)
{
    QCryptographicHash hash(QCryptographicHash::Sha256);
    hash.addData(parentId.toUtf8());
    hash.addData("\0", 1);
    hash.addData(savedAt.toUTC().toString(Qt::ISODateWithMs).toUtf8());
    hash.addData("\0", 1);
    hash.addData(payload.toUtf8());
    return QString::fromLatin1(hash.result().toHex());
}

QString compressedPayload(const QString &payload)
{
    return QString::fromLatin1(
        qCompress(payload.toUtf8(), 9).toBase64());
}

bool uncompressPayload(
    const QString &encoded,
    QString *payload,
    QString *errorMessage)
{
    const QByteArray compressed = QByteArray::fromBase64(
        encoded.toLatin1(),
        QByteArray::AbortOnBase64DecodingErrors);
    const QByteArray uncompressed = qUncompress(compressed);
    if (compressed.isEmpty() || uncompressed.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                "An embedded wing snapshot could not be decompressed.");
        }
        return false;
    }
    *payload = QString::fromUtf8(uncompressed);
    return true;
}

QJsonArray changedSectionsJson(const QList<int> &sections)
{
    QJsonArray result;
    for (const int section : sections) {
        result.append(section);
    }
    return result;
}

QList<int> changedSectionsFromJson(const QJsonArray &values)
{
    QList<int> result;
    for (const QJsonValue &value : values) {
        if (value.isDouble()) {
            result.append(value.toInt());
        }
    }
    return result;
}

} // namespace

bool DesignDocument::load(const QString &path, QString *errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    EncodedHistory encodedHistory;
    if (!splitHistory(
            normalizedText(file.readAll()),
            &encodedHistory,
            errorMessage)) {
        return false;
    }
    if (!replacePayload(encodedHistory.payload, errorMessage)) {
        return false;
    }

    filePath_ = QFileInfo(path).absoluteFilePath();
    savedPayload_ = assembledText();
    revisions_.clear();
    historyPersisted_ = encodedHistory.present;
    historyDirty_ = false;
    splines_ = encodedHistory.root.value(QStringLiteral("splines")).toObject();
    splinesDirty_ = false;

    if (encodedHistory.present) {
        if (encodedHistory.root.value(QStringLiteral("format")).toString()
                != QString::fromLatin1(historyFormat)
            || encodedHistory.root.value(QStringLiteral("version")).toInt() != 1) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "This wing uses an unsupported embedded history format.");
            }
            return false;
        }

        const QJsonArray revisions =
            encodedHistory.root.value(QStringLiteral("revisions")).toArray();
        QString expectedParent;
        for (const QJsonValue &value : revisions) {
            const QJsonObject object = value.toObject();
            StoredRevision revision;
            revision.metadata.id = object.value(QStringLiteral("id")).toString();
            revision.metadata.parentId =
                object.value(QStringLiteral("parent")).toString();
            revision.metadata.savedAt = QDateTime::fromString(
                object.value(QStringLiteral("savedAt")).toString(),
                Qt::ISODateWithMs);
            revision.metadata.summary =
                object.value(QStringLiteral("summary")).toString();
            revision.metadata.changedSections = changedSectionsFromJson(
                object.value(QStringLiteral("changedSections")).toArray());
            if (!uncompressPayload(
                    object.value(QStringLiteral("content")).toString(),
                    &revision.payload,
                    errorMessage)) {
                return false;
            }

            if (!revision.metadata.savedAt.isValid()
                || revision.metadata.parentId != expectedParent
                || revision.metadata.id
                    != revisionId(
                        revision.metadata.parentId,
                        revision.metadata.savedAt,
                        revision.payload)) {
                if (errorMessage != nullptr) {
                    *errorMessage = QStringLiteral(
                        "The embedded version chain failed its integrity check.");
                }
                return false;
            }

            expectedParent = revision.metadata.id;
            revisions_.append(std::move(revision));
        }
        if (revisions_.isEmpty()) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "The embedded version history contains no wing snapshots.");
            }
            return false;
        }
    } else {
        StoredRevision original;
        original.metadata.savedAt = QFileInfo(path).lastModified().toUTC();
        if (!original.metadata.savedAt.isValid()) {
            original.metadata.savedAt = QDateTime::currentDateTimeUtc();
        }
        original.metadata.summary = QStringLiteral("Original wing");
        original.payload = savedPayload_;
        original.metadata.id =
            revisionId({}, original.metadata.savedAt, original.payload);
        revisions_.append(std::move(original));
        historyDirty_ = true;
    }

    activeRevisionIndex_ = revisions_.size() - 1;
    if (revisions_.constLast().payload != savedPayload_) {
        StoredRevision external;
        external.metadata.parentId = revisions_.constLast().metadata.id;
        external.metadata.savedAt = QFileInfo(path).lastModified().toUTC();
        if (!external.metadata.savedAt.isValid()) {
            external.metadata.savedAt = QDateTime::currentDateTimeUtc();
        }
        external.metadata.changedSections =
            changedSectionsBetween(revisions_.constLast().payload, savedPayload_);
        external.metadata.summary = QStringLiteral("External file edit");
        external.payload = savedPayload_;
        external.metadata.id = revisionId(
            external.metadata.parentId,
            external.metadata.savedAt,
            external.payload);
        revisions_.append(std::move(external));
        activeRevisionIndex_ = revisions_.size() - 1;
        historyDirty_ = true;
    }
    return true;
}

bool DesignDocument::save(QString *errorMessage)
{
    if (filePath_.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("The design does not have a file path.");
        }
        return false;
    }

    const QString invalid = validationError();
    if (!invalid.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = invalid;
        }
        return false;
    }

    const QString currentPayload = assembledText();
    const qsizetype originalRevisionCount = revisions_.size();
    const int originalActiveRevision = activeRevisionIndex_;
    const bool originalHistoryDirty = historyDirty_;

    if (currentPayload != savedPayload_) {
        StoredRevision revision;
        revision.metadata.parentId =
            revisions_.isEmpty() ? QString() : revisions_.constLast().metadata.id;
        revision.metadata.savedAt = QDateTime::currentDateTimeUtc();
        revision.metadata.changedSections =
            changedSectionsBetween(savedPayload_, currentPayload);
        revision.metadata.summary =
            revisionSummary(revision.metadata.changedSections);
        revision.payload = currentPayload;
        revision.metadata.id = revisionId(
            revision.metadata.parentId,
            revision.metadata.savedAt,
            revision.payload);
        revisions_.append(std::move(revision));
        activeRevisionIndex_ = revisions_.size() - 1;
        historyDirty_ = true;
    }

    QSaveFile file(filePath_);
    if (!file.open(QIODevice::WriteOnly)) {
        revisions_.resize(originalRevisionCount);
        activeRevisionIndex_ = originalActiveRevision;
        historyDirty_ = originalHistoryDirty;
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    const QByteArray encoded = serializedText().toUtf8();
    if (file.write(encoded) != encoded.size() || !file.commit()) {
        revisions_.resize(originalRevisionCount);
        activeRevisionIndex_ = originalActiveRevision;
        historyDirty_ = originalHistoryDirty;
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    savedPayload_ = currentPayload;
    historyPersisted_ = true;
    historyDirty_ = false;
    splinesDirty_ = false;
    return true;
}

QJsonObject DesignDocument::splinesData() const
{
    return splines_;
}

void DesignDocument::setSplinesData(const QJsonObject &data)
{
    if (splines_ == data)
        return;
    splines_ = data;
    splinesDirty_ = true;
}

bool DesignDocument::splinesDirty() const
{
    return splinesDirty_;
}

bool DesignDocument::saveAs(const QString &path, QString *errorMessage)
{
    const QString previousPath = filePath_;
    filePath_ = QFileInfo(path).absoluteFilePath();
    if (save(errorMessage)) {
        return true;
    }
    filePath_ = previousPath;
    return false;
}

QString DesignDocument::filePath() const
{
    return filePath_;
}

const QList<DesignSection> &DesignDocument::sections() const
{
    return sections_;
}

void DesignDocument::setSectionText(int index, const QString &text)
{
    if (index >= 0 && index < sections_.size()) {
        sections_[index].text = text;
    }
}

int DesignDocument::revisionCount() const
{
    return revisions_.size();
}

QList<DesignRevision> DesignDocument::revisions() const
{
    QList<DesignRevision> result;
    result.reserve(revisions_.size());
    for (const StoredRevision &revision : revisions_) {
        result.append(revision.metadata);
    }
    return result;
}

QStringList DesignDocument::sectionHistory(
    int sectionNumber,
    int *currentIndex) const
{
    QStringList result;
    int cursor = -1;
    for (qsizetype revisionIndex = 0;
         revisionIndex < revisions_.size();
         ++revisionIndex) {
        const QString text =
            sectionsByNumber(revisions_.at(revisionIndex).payload)
                .value(sectionNumber);
        if (text.isNull()) {
            continue;
        }
        if (result.isEmpty() || result.constLast() != text) {
            result.append(text);
        }
        if (revisionIndex <= activeRevisionIndex_) {
            cursor = result.size() - 1;
        }
    }

    const QMap<int, QString> currentSections =
        sectionsByNumber(assembledText());
    const QString current = currentSections.value(sectionNumber);
    if (!current.isNull()
        && (cursor < 0 || result.value(cursor) != current)) {
        result.append(current);
        cursor = result.size() - 1;
    }

    if (currentIndex != nullptr) {
        *currentIndex = cursor;
    }
    return result;
}

bool DesignDocument::restoreRevision(
    int revisionIndex,
    QString *errorMessage)
{
    if (revisionIndex < 0 || revisionIndex >= revisions_.size()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("The selected wing version does not exist.");
        }
        return false;
    }

    if (!replacePayload(revisions_.at(revisionIndex).payload, errorMessage)) {
        return false;
    }
    activeRevisionIndex_ = revisionIndex;
    return true;
}

QString DesignDocument::savedSectionText(int sectionNumber) const
{
    return sectionsByNumber(savedPayload_).value(sectionNumber);
}

QString DesignDocument::validationError() const
{
    if (sections_.isEmpty()) {
        return QStringLiteral("The design contains no editable sections.");
    }

    QSet<int> numbers;
    for (qsizetype index = 0; index < sections_.size(); ++index) {
        const DesignSection &section = sections_.at(index);
        if (numbers.contains(section.number)) {
            return QStringLiteral("Section %1 occurs more than once.").arg(section.number);
        }
        numbers.insert(section.number);

        const QRegularExpressionMatch header =
            sectionHeaderPattern().match(section.text);
        if (!header.hasMatch() || header.capturedStart() != 0
            || header.captured(1).toInt() != section.number) {
            return QStringLiteral(
                       "Section %1 must begin with its numbered '* %1.' header.")
                .arg(section.number);
        }

        const QStringList lines = section.text.split(QLatin1Char('\n'));
        qsizetype lastContentLine = lines.size() - 1;
        while (lastContentLine >= 0 && lines.at(lastContentLine).trimmed().isEmpty()) {
            --lastContentLine;
        }
        for (qsizetype line = 0; line < lines.size(); ++line) {
            const bool sectionLineTerminator =
                line + 1 == lines.size() && lines.at(line).isEmpty();
            const bool terminalFilePadding =
                index + 1 == sections_.size() && line > lastContentLine;
            if (!sectionLineTerminator
                && !terminalFilePadding
                && lines.at(line).trimmed().isEmpty()) {
                return QStringLiteral(
                           "Section %1 contains a blank line at editor line %2. "
                           "The Fortran format does not allow blank records.")
                    .arg(section.number)
                    .arg(line + 1);
            }
        }
    }
    return {};
}

QString DesignDocument::assembledText() const
{
    QString result = preamble_;
    for (qsizetype index = 0; index < sections_.size(); ++index) {
        if (!result.isEmpty() && !result.endsWith(QLatin1Char('\n'))) {
            result.append(QLatin1Char('\n'));
        }
        QString block = sections_.at(index).text;
        if (index + 1 < sections_.size()) {
            while (block.endsWith(QStringLiteral("\n\n"))) {
                block.chop(1);
            }
            if (!block.endsWith(QLatin1Char('\n'))) {
                block.append(QLatin1Char('\n'));
            }
        }
        result.append(block);
    }

    if (finalNewline_ && !result.endsWith(QLatin1Char('\n'))) {
        result.append(QLatin1Char('\n'));
    } else if (!finalNewline_ && result.endsWith(QLatin1Char('\n'))) {
        result.chop(1);
    }
    return result;
}

bool DesignDocument::isEmpty() const
{
    return sections_.isEmpty();
}

bool DesignDocument::replacePayload(
    const QString &payload,
    QString *errorMessage)
{
    const QList<SectionLocation> headers = sectionLocations(payload);
    if (headers.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                "No numbered LEparagliding section headers were found.");
        }
        return false;
    }

    finalNewline_ = payload.endsWith(QLatin1Char('\n'));
    preamble_ = payload.left(headers.constFirst().start);
    sections_.clear();
    for (qsizetype index = 0; index < headers.size(); ++index) {
        const qsizetype end =
            index + 1 < headers.size() ? headers.at(index + 1).start : payload.size();
        DesignSection section;
        section.number = headers.at(index).number;
        section.title = headers.at(index).title;
        section.text =
            payload.mid(headers.at(index).start, end - headers.at(index).start);
        sections_.append(std::move(section));
    }
    return true;
}

QString DesignDocument::serializedText() const
{
    QJsonObject root;
    root.insert(QStringLiteral("format"), QString::fromLatin1(historyFormat));
    root.insert(QStringLiteral("version"), 1);
    root.insert(
        QStringLiteral("payloadFinalNewline"),
        assembledText().endsWith(QLatin1Char('\n')));

    QJsonArray revisions;
    for (const StoredRevision &revision : revisions_) {
        QJsonObject object;
        object.insert(QStringLiteral("id"), revision.metadata.id);
        object.insert(QStringLiteral("parent"), revision.metadata.parentId);
        object.insert(
            QStringLiteral("savedAt"),
            revision.metadata.savedAt.toUTC().toString(Qt::ISODateWithMs));
        object.insert(QStringLiteral("summary"), revision.metadata.summary);
        object.insert(
            QStringLiteral("changedSections"),
            changedSectionsJson(revision.metadata.changedSections));
        object.insert(
            QStringLiteral("content"),
            compressedPayload(revision.payload));
        revisions.append(object);
    }
    root.insert(QStringLiteral("revisions"), revisions);
    if (!splines_.isEmpty()) {
        root.insert(QStringLiteral("splines"), splines_);
    }

    const QByteArray encoded =
        QJsonDocument(root).toJson(QJsonDocument::Compact).toBase64();
    QString result = assembledText();
    if (!result.endsWith(QLatin1Char('\n'))) {
        result.append(QLatin1Char('\n'));
    }
    result.append(QString::fromLatin1(historyBegin));
    result.append(QLatin1Char('\n'));
    for (qsizetype offset = 0; offset < encoded.size(); offset += historyLineLength) {
        result.append(QStringLiteral("* "));
        result.append(QString::fromLatin1(encoded.mid(offset, historyLineLength)));
        result.append(QLatin1Char('\n'));
    }
    result.append(QString::fromLatin1(historyEnd));
    result.append(QLatin1Char('\n'));
    return result;
}
