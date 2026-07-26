// Unit tests for the generic section-grid tokenizer behind the table
// editors. Pure C++ (no Qt), like the other test tools.

#include "section_grid.h"

#include <cstdio>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const char *what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

const char *const kSection2 =
    "**************************************************************\n"
    "*             2. AIRFOILS                                    *\n"
    "**************************************************************\n"
    "* Airfoil name, intake in, intake out, open , disp. rrw\n"
    "1\tgnua.txt\t1.6\t6.3\t1\t0\t1\t15\t\n"
    "2\tgnua.txt\t1.6\t6.3\t1\t0\t1\t15\n"
    "* trailing banner\n";

void testTokenizesRows()
{
    const lep::SectionGrid grid = lep::parseSectionGrid(kSection2);
    check(grid.rows.size() == 2, "two data rows");
    check(grid.maxColumns == 8, "eight columns");
    check(grid.rows[0].lineIndex == 4, "row 1 line index");
    check(grid.rows[0].tokens[1].text == "gnua.txt", "file token");
    check(grid.rows[0].comment
              == "Airfoil name, intake in, intake out, open , disp. rrw",
          "row label from preceding comment");
}

void testQuotedStringsStayWhole()
{
    const lep::SectionGrid grid = lep::parseSectionGrid(
        "* Brand name\n\"LABORATORI D'ENVOL\"  extra\n");
    check(grid.rows.size() == 1, "quoted: one row");
    check(grid.rows[0].tokens.size() == 2, "quoted: two tokens");
    check(grid.rows[0].tokens[0].text == "\"LABORATORI D'ENVOL\"",
          "quoted: quotes kept, spaces inside preserved");
    check(grid.rows[0].tokens[1].text == "extra", "quoted: next token");
}

void testReplaceTokenPreservesBytes()
{
    const std::string line = " 2      67.75    \t 1.39     308.60";
    const lep::SectionGrid grid =
        lep::parseSectionGrid("* c\n" + line + "\n");
    const lep::GridToken &token = grid.rows[0].tokens[2];
    check(token.text == "1.39", "replace: found token");
    const std::string replaced =
        lep::lineWithReplacedToken(line, token, "9.99");
    check(replaced == " 2      67.75    \t 9.99     308.60",
          "replace: only the token changed");
}

void testBannerCommentsSkipped()
{
    const lep::SectionGrid grid = lep::parseSectionGrid(
        "****************\n* Real label\n****************\n42\n");
    check(grid.rows.size() == 1, "banner: one row");
    check(grid.rows[0].comment == "Real label",
          "banner: pure-asterisk lines do not become labels");
}

} // namespace

int main()
{
    testTokenizesRows();
    testQuotedStringsStayWhole();
    testReplaceTokenPreservesBytes();
    testBannerCommentsSkipped();
    if (failures > 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all section_grid checks passed\n");
    return 0;
}
