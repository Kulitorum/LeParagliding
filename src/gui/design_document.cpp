#include "design_document.h"

#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>

namespace {

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

    QString text = QString::fromUtf8(file.readAll());
    if (text.startsWith(QChar::ByteOrderMark)) {
        text.remove(0, 1);
    }
    text.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
    text.replace(QLatin1Char('\r'), QLatin1Char('\n'));

    finalNewline_ = text.endsWith(QLatin1Char('\n'));
    sections_.clear();
    preamble_.clear();

    struct Header
    {
        qsizetype start = 0;
        int number = 0;
        QString title;
    };
    QList<Header> headers;

    auto matches = sectionHeaderPattern().globalMatch(text);
    while (matches.hasNext()) {
        const QRegularExpressionMatch match = matches.next();
        Header header;
        header.start = match.capturedStart();
        header.number = match.captured(1).toInt();
        header.title = cleanTitle(match.captured(2));
        headers.append(header);
    }

    if (headers.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                "No numbered LEparagliding section headers were found.");
        }
        return false;
    }

    preamble_ = text.left(headers.constFirst().start);
    for (qsizetype index = 0; index < headers.size(); ++index) {
        const qsizetype end =
            index + 1 < headers.size() ? headers.at(index + 1).start : text.size();
        DesignSection section;
        section.number = headers.at(index).number;
        section.title = headers.at(index).title;
        section.text = text.mid(headers.at(index).start, end - headers.at(index).start);
        sections_.append(section);
    }

    filePath_ = QFileInfo(path).absoluteFilePath();
    return true;
}

bool DesignDocument::save(QString *errorMessage) const
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

    QSaveFile file(filePath_);
    if (!file.open(QIODevice::WriteOnly)) {
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }

    const QByteArray encoded = assembledText().toUtf8();
    if (file.write(encoded) != encoded.size() || !file.commit()) {
        if (errorMessage != nullptr) {
            *errorMessage = file.errorString();
        }
        return false;
    }
    return true;
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
