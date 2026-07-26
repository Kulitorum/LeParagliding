// Unit tests for the Section 1 rib-matrix parser/serializer that backs the
// graphical curve editor. Pure C++ (no Qt), mirroring the other test tools.

#include "section1_curves.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char *what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

bool containsText(const std::vector<std::string> &problems,
                  const std::string &needle)
{
    for (const std::string &problem : problems) {
        if (problem.find(needle) != std::string::npos)
            return true;
    }
    return false;
}

const char *const kSection1 =
    "**************************************************************\n"
    "*             1. GEOMETRY                                    *\n"
    "**************************************************************\n"
    "* Brand name\n"
    "\"LABORATORI D'ENVOL\"\n"
    "* Wing name\n"
    "\"gnu-test-3.19\"\n"
    "* Drawing scale\n"
    "1.\n"
    "* Wing scale \n"
    "0.980762046\n"
    "* Number of cells\n"
    "\t29\n"
    "* Number of ribs\n"
    "\t30\n"
    "* Alpha max and parameter \n"
    "\t4.5\t1\n"
    "* Paraglider type and parameter\n"
    "  \"ds\" \t1\n"
    "* Rib geometric parameters\n"
    "* Rib\tx-rib\t    y-LE\ty-TE\t    xp\t       z\tbeta\t   RP\t      "
    "Washin\n"
    " 1      22.65       0.16     309.25      22.64       0.40       1.95   "
    "   33.33       0.00 0.0 50.0\n"
    " 2      67.75       1.39     308.60      67.62       3.53       6.06   "
    "   33.33       0.00 0.0 50.0\n"
    " 3     112.46       3.84     307.32     111.90       9.71       9.97   "
    "   33.33       0.00 0.0 50.0\n"
    "**************************************************************\n"
    "*             2. AIRFOILS                                    *\n";

std::string withReplacedLine(const std::string &text, int lineIndex,
                             const std::string &newLine)
{
    std::vector<std::string> lines;
    std::string current;
    for (const char ch : text) {
        if (ch == '\n') {
            lines.push_back(current);
            current.clear();
        } else {
            current += ch;
        }
    }
    lines.push_back(current);
    lines[static_cast<size_t>(lineIndex)] = newLine;
    std::string joined;
    for (size_t i = 0; i < lines.size(); ++i) {
        if (i > 0)
            joined += '\n';
        joined += lines[i];
    }
    return joined;
}

void testParsesReferenceSection()
{
    lep::Section1Matrix matrix;
    std::vector<std::string> problems;
    check(lep::parseSection1Matrix(kSection1, &matrix, &problems),
          "reference section parses");
    check(matrix.columnCount == 11, "reference section has 11 columns");
    check(matrix.rows.size() == 3, "reference section has 3 rows");
    check(matrix.declaredCells == 29, "declared cells read");
    check(matrix.declaredRibs == 30, "declared ribs read");
    check(std::fabs(matrix.rows[0].values[1] - 22.65) < 1e-9,
          "row 1 x-rib value");
    check(std::fabs(matrix.rows[2].values[6] - 9.97) < 1e-9,
          "row 3 beta value");
    check(std::fabs(matrix.rows[2].values[10] - 50.0) < 1e-9,
          "row 3 Pos_z value");
    // 29 cells imply 15 rows; the trimmed fixture has 3, which must surface
    // as a human-readable warning, not a parse failure.
    check(containsText(problems, "imply 15 rib rows"),
          "row count mismatch warned");
}

void testRoundTripPreservesValues()
{
    lep::Section1Matrix matrix;
    std::vector<std::string> problems;
    check(lep::parseSection1Matrix(kSection1, &matrix, &problems),
          "round trip: initial parse");

    // Simulate the curve editor changing beta of row 2 and rewriting only
    // that line.
    lep::Section1Row edited = matrix.rows[1];
    edited.values[6] = 7.25;
    const std::string text = withReplacedLine(
        kSection1, edited.lineIndex, lep::formatSection1Row(edited));

    lep::Section1Matrix reparsed;
    check(lep::parseSection1Matrix(text, &reparsed, &problems),
          "round trip: reparse after edit");
    check(reparsed.rows.size() == matrix.rows.size(),
          "round trip: row count unchanged");
    for (size_t r = 0; r < reparsed.rows.size(); ++r) {
        for (size_t c = 0; c < reparsed.rows[r].values.size(); ++c) {
            const double expected =
                (r == 1 && c == 6) ? 7.25 : matrix.rows[r].values[c];
            char what[96];
            std::snprintf(what, sizeof what,
                          "round trip: row %zu column %zu preserved", r + 1,
                          c + 1);
            check(std::fabs(reparsed.rows[r].values[c] - expected) < 5e-3,
                  what);
        }
    }
}

void testNineColumnLegacyFormat()
{
    std::string text = kSection1;
    // Strip the two trailing columns from every data row.
    lep::Section1Matrix matrix;
    std::vector<std::string> problems;
    lep::parseSection1Matrix(text, &matrix, &problems);
    for (const lep::Section1Row &row : matrix.rows) {
        lep::Section1Row nineColumns = row;
        nineColumns.values.resize(9);
        text = withReplacedLine(text, row.lineIndex,
                                lep::formatSection1Row(nineColumns));
    }
    check(lep::parseSection1Matrix(text, &matrix, &problems),
          "9-column matrix parses");
    check(matrix.columnCount == 9, "9-column matrix column count");
}

void testProblemsAreReported()
{
    lep::Section1Matrix matrix;
    std::vector<std::string> problems;

    check(!lep::parseSection1Matrix("* Brand name\n\"x\"\n", &matrix,
                                    &problems),
          "matrix without marker rejected");
    check(containsText(problems, "Rib geometric parameters"),
          "missing marker explained");

    // Data rows sit at 0-based lines 21-23 of the fixture; line 22 is row 2.
    const std::string badToken = withReplacedLine(
        kSection1, 22,
        " 2      67.75       oops     308.60      67.62       3.53       "
        "6.06      33.33       0.00 0.0 50.0");
    check(!lep::parseSection1Matrix(badToken, &matrix, &problems),
          "non-numeric token rejected");
    check(containsText(problems, "not a number"),
          "non-numeric token explained");

    const std::string badChord = withReplacedLine(
        kSection1, 22,
        " 2      67.75     308.60       1.39      67.62       3.53       "
        "6.06      33.33       0.00 0.0 50.0");
    check(lep::parseSection1Matrix(badChord, &matrix, &problems),
          "negative chord still parses");
    check(containsText(problems, "y-TE"), "negative chord warned");

    const std::string badOrder = withReplacedLine(
        kSection1, 22,
        " 2      12.65       1.39     308.60      67.62       3.53       "
        "6.06      33.33       0.00 0.0 50.0");
    check(lep::parseSection1Matrix(badOrder, &matrix, &problems),
          "non-increasing x-rib still parses");
    check(containsText(problems, "x-rib"), "non-increasing x-rib warned");
}

void testColumnMetadata()
{
    const std::vector<lep::Section1Column> &columns = lep::section1Columns();
    check(columns.size() == 11, "11 columns documented");
    check(!columns[0].editable, "rib number not editable");
    for (const lep::Section1Column &column : columns) {
        check(column.description[0] != '\0', "column has a description");
        check(column.label[0] != '\0', "column has a label");
    }
}

} // namespace

int main()
{
    testParsesReferenceSection();
    testRoundTripPreservesValues();
    testNineColumnLegacyFormat();
    testProblemsAreReported();
    testColumnMetadata();
    if (failures > 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all section1_curves checks passed\n");
    return 0;
}
