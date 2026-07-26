#pragma once

#include <string>
#include <vector>

namespace lep {

// Generic tokenized view of a section's data records, for the grid editors
// that back every section without a specialized panel. Each non-comment,
// non-blank line after the section banner is one row of whitespace-separated
// tokens (double-quoted strings stay single tokens). Token offsets into the
// original line allow replacing one cell while preserving every other byte
// of the line. Pure C++ so it compiles into the Qt-free test executables.

struct GridToken
{
    std::string text;   // token as written, including any quotes
    size_t offset = 0;  // byte offset of the token within its line
    size_t length = 0;  // byte length of the token within its line
};

struct GridRow
{
    int lineIndex = -1;      // 0-based line within the section text
    std::string comment;     // nearest preceding comment text, "*" stripped
    std::vector<GridToken> tokens;
};

struct SectionGrid
{
    std::vector<GridRow> rows;
    size_t maxColumns = 0;
};

// Tokenizes every data record of a section text. The banner lines that
// introduce the section (and the next one) are comments and are skipped;
// their text becomes the row labels instead. Always succeeds; a section
// with no data records yields an empty grid.
SectionGrid parseSectionGrid(const std::string &sectionText);

// The line with one token's text replaced, whitespace and neighbours
// preserved byte for byte.
std::string lineWithReplacedToken(const std::string &line,
                                  const GridToken &token,
                                  const std::string &newText);

} // namespace lep
