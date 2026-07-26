#include "section_grid.h"

#include <cctype>

namespace lep {

namespace {

bool isSpace(char ch)
{
    return std::isspace(static_cast<unsigned char>(ch)) != 0;
}

std::string trimmed(const std::string &line)
{
    size_t begin = 0;
    size_t end = line.size();
    while (begin < end && isSpace(line[begin]))
        ++begin;
    while (end > begin && isSpace(line[end - 1]))
        --end;
    return line.substr(begin, end - begin);
}

// Comment text with the leading '*' decorations and trailing banner
// asterisks stripped, for use as a row label.
std::string commentLabel(const std::string &line)
{
    std::string text = trimmed(line);
    size_t begin = 0;
    while (begin < text.size() && (text[begin] == '*' || isSpace(text[begin])))
        ++begin;
    size_t end = text.size();
    while (end > begin && (text[end - 1] == '*' || isSpace(text[end - 1])))
        --end;
    return text.substr(begin, end - begin);
}

std::vector<GridToken> tokenizeLine(const std::string &line)
{
    std::vector<GridToken> tokens;
    size_t i = 0;
    while (i < line.size()) {
        if (isSpace(line[i])) {
            ++i;
            continue;
        }
        GridToken token;
        token.offset = i;
        if (line[i] == '"') {
            // Quoted string: one token up to and including the closing
            // quote (or end of line when unterminated).
            ++i;
            while (i < line.size() && line[i] != '"')
                ++i;
            if (i < line.size())
                ++i;
        } else {
            while (i < line.size() && !isSpace(line[i]))
                ++i;
        }
        token.length = i - token.offset;
        token.text = line.substr(token.offset, token.length);
        tokens.push_back(token);
    }
    return tokens;
}

} // namespace

SectionGrid parseSectionGrid(const std::string &sectionText)
{
    SectionGrid grid;
    std::vector<std::string> lines;
    std::string current;
    for (const char ch : sectionText) {
        if (ch == '\n') {
            lines.push_back(current);
            current.clear();
        } else if (ch != '\r') {
            current += ch;
        }
    }
    lines.push_back(current);

    std::string lastComment;
    for (size_t index = 0; index < lines.size(); ++index) {
        const std::string t = trimmed(lines[index]);
        if (t.empty())
            continue;
        if (t[0] == '*') {
            const std::string label = commentLabel(lines[index]);
            // Banner separator rows of pure asterisks produce empty labels;
            // keep the previous meaningful one in that case.
            if (!label.empty())
                lastComment = label;
            continue;
        }
        GridRow row;
        row.lineIndex = static_cast<int>(index);
        row.comment = lastComment;
        row.tokens = tokenizeLine(lines[index]);
        grid.maxColumns = std::max(grid.maxColumns, row.tokens.size());
        grid.rows.push_back(std::move(row));
    }
    return grid;
}

std::string lineWithReplacedToken(const std::string &line,
                                  const GridToken &token,
                                  const std::string &newText)
{
    if (token.offset > line.size()
        || token.offset + token.length > line.size())
        return line;
    std::string result = line.substr(0, token.offset);
    result += newText;
    result += line.substr(token.offset + token.length);
    return result;
}

} // namespace lep
