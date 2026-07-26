#include "section1_curves.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace lep {

namespace {

constexpr double kUnbounded = 1e300;

std::vector<std::string> splitLines(const std::string &text)
{
    std::vector<std::string> lines;
    std::string current;
    for (const char ch : text) {
        if (ch == '\n') {
            lines.push_back(current);
            current.clear();
        } else if (ch != '\r') {
            current += ch;
        }
    }
    lines.push_back(current);
    return lines;
}

std::string trimmed(const std::string &line)
{
    size_t begin = 0;
    size_t end = line.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(line[begin])))
        ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(line[end - 1])))
        --end;
    return line.substr(begin, end - begin);
}

bool isCommentLine(const std::string &line)
{
    const std::string t = trimmed(line);
    return !t.empty() && t[0] == '*';
}

bool containsNoCase(const std::string &line, const char *needle)
{
    const size_t needleLength = std::strlen(needle);
    if (needleLength == 0 || line.size() < needleLength)
        return false;
    for (size_t i = 0; i + needleLength <= line.size(); ++i) {
        size_t j = 0;
        while (j < needleLength
               && std::tolower(static_cast<unsigned char>(line[i + j]))
                      == std::tolower(static_cast<unsigned char>(needle[j])))
            ++j;
        if (j == needleLength)
            return true;
    }
    return false;
}

std::vector<std::string> tokenize(const std::string &line)
{
    std::vector<std::string> tokens;
    std::string current;
    for (const char ch : line) {
        if (std::isspace(static_cast<unsigned char>(ch))) {
            if (!current.empty()) {
                tokens.push_back(current);
                current.clear();
            }
        } else {
            current += ch;
        }
    }
    if (!current.empty())
        tokens.push_back(current);
    return tokens;
}

bool parseNumber(const std::string &token, double *value)
{
    char *end = nullptr;
    *value = std::strtod(token.c_str(), &end);
    return end && *end == '\0' && end != token.c_str();
}

// Value of the first numeric line following a "* <marker>" comment, or -1.
int declaredCountAfter(const std::vector<std::string> &lines, const char *marker)
{
    for (size_t i = 0; i + 1 < lines.size(); ++i) {
        if (isCommentLine(lines[i]) && containsNoCase(lines[i], marker)) {
            double value = 0.0;
            const std::vector<std::string> tokens = tokenize(lines[i + 1]);
            if (!tokens.empty() && parseNumber(tokens[0], &value))
                return static_cast<int>(std::lround(value));
            return -1;
        }
    }
    return -1;
}

std::string describeRow(size_t rowNumber)
{
    char buffer[48];
    std::snprintf(buffer, sizeof buffer, "Rib row %zu", rowNumber);
    return buffer;
}

} // namespace

const std::vector<Section1Column> &section1Columns()
{
    static const std::vector<Section1Column> columns = {
        {"rib", "Rib", "",
         "Rib number, counted from the wing centre (1) outwards to the tip. "
         "One row per rib of the half wing; the other half is mirrored.",
         1.0, 100.0, 0, false},
        {"xrib", "x-rib", "cm",
         "Spanwise position of the rib in the flattened planform, measured "
         "from the wing centre. Must increase from row to row.",
         0.0, kUnbounded, 2, true},
        {"yle", "y-LE", "cm",
         "Leading-edge position of the rib in the planform, measured from the "
         "line through the centre rib nose. Grows as the tip sweeps back.",
         -kUnbounded, kUnbounded, 2, true},
        {"yte", "y-TE", "cm",
         "Trailing-edge position of the rib in the planform. The rib chord is "
         "y-TE minus y-LE, so this must stay above y-LE.",
         -kUnbounded, kUnbounded, 2, true},
        {"xp", "xp", "cm",
         "Spanwise position of the rib once the wing is inflated (the vault "
         "front view). Moves inboard of x-rib as the arc curves down.",
         0.0, kUnbounded, 2, true},
        {"z", "z", "cm",
         "Vertical drop of the rib below the centre chord in the inflated "
         "vault. 0 at the centre, largest at the tip.",
         -kUnbounded, kUnbounded, 2, true},
        {"beta", "beta", "deg",
         "Tilt of the rib plane away from vertical in the front view. 0 at "
         "the centre; near 90 at the tip, where ribs lie almost flat.",
         -45.0, 135.0, 2, true},
        {"rp", "RP", "%",
         "Rotation point: chordwise position (% of chord from the leading "
         "edge) around which the washin twist is applied.",
         0.0, 100.0, 2, true},
        {"washin", "Washin", "deg",
         "Twist of the rib chord relative to the centre rib (positive nose "
         "up). Usually 0 when automatic washin (Alpha max) is enabled.",
         -30.0, 30.0, 2, true},
        {"rotz", "Rot_z", "deg",
         "Extra rotation of the rib around the vertical axis, used to shape "
         "the wing tip (LEP 3.16+). 0 means no rotation.",
         -90.0, 90.0, 2, true},
        {"posz", "Pos_z", "%",
         "Chordwise position (% of chord) of the centre of the Rot_z "
         "rotation (LEP 3.16+). 50 is mid-chord.",
         0.0, 100.0, 1, true},
    };
    return columns;
}

bool parseSection1Matrix(const std::string &sectionText, Section1Matrix *matrix,
                         std::vector<std::string> *problems)
{
    *matrix = Section1Matrix();
    problems->clear();

    const std::vector<std::string> lines = splitLines(sectionText);
    matrix->declaredCells = declaredCountAfter(lines, "number of cells");
    matrix->declaredRibs = declaredCountAfter(lines, "number of ribs");

    size_t markerIndex = lines.size();
    for (size_t i = 0; i < lines.size(); ++i) {
        if (isCommentLine(lines[i])
            && containsNoCase(lines[i], "rib geometric parameters")) {
            markerIndex = i;
            break;
        }
    }
    if (markerIndex == lines.size()) {
        problems->push_back(
            "Section 1 has no \"* Rib geometric parameters\" line, so the rib "
            "matrix cannot be located. Restore a standard Section 1 (Undo or "
            "Versions) to edit it graphically.");
        return false;
    }

    // Data rows: the non-comment, non-blank lines after the marker. Comment
    // lines directly after the marker are the column header; comments after
    // the data are the banner of the next section.
    size_t index = markerIndex + 1;
    while (index < lines.size()
           && (isCommentLine(lines[index]) || trimmed(lines[index]).empty()))
        ++index;
    for (; index < lines.size(); ++index) {
        if (isCommentLine(lines[index]) || trimmed(lines[index]).empty())
            break;
        const std::vector<std::string> tokens = tokenize(lines[index]);
        Section1Row row;
        row.lineIndex = static_cast<int>(index);
        bool numeric = true;
        for (const std::string &token : tokens) {
            double value = 0.0;
            if (!parseNumber(token, &value)) {
                problems->push_back(describeRow(matrix->rows.size() + 1)
                                    + ": \"" + token + "\" is not a number.");
                numeric = false;
                break;
            }
            row.values.push_back(value);
        }
        if (!numeric)
            return false;
        matrix->rows.push_back(row);
    }

    if (matrix->rows.empty()) {
        problems->push_back(
            "No rib rows found after \"* Rib geometric parameters\". Enter "
            "one row per rib of the half wing (9 or 11 numbers each).");
        return false;
    }

    matrix->columnCount = static_cast<int>(matrix->rows.front().values.size());
    if (matrix->columnCount != 9 && matrix->columnCount != 11) {
        char buffer[160];
        std::snprintf(buffer, sizeof buffer,
                      "Rib rows have %d values each; expected 9 (LEP < 3.16) "
                      "or 11 (LEP 3.16+, with Rot_z and Pos_z).",
                      matrix->columnCount);
        problems->push_back(buffer);
        return false;
    }
    for (size_t i = 1; i < matrix->rows.size(); ++i) {
        if (static_cast<int>(matrix->rows[i].values.size())
            != matrix->columnCount) {
            char buffer[160];
            std::snprintf(buffer, sizeof buffer,
                          "%s has %zu values, but the first row has %d. All "
                          "rows must use the same column count.",
                          describeRow(i + 1).c_str(),
                          matrix->rows[i].values.size(), matrix->columnCount);
            problems->push_back(buffer);
            return false;
        }
    }

    // The matrix is usable from here on; everything below is a warning.
    char buffer[220];
    if (matrix->declaredCells > 0) {
        const int expectedRows = matrix->declaredCells / 2 + 1;
        if (static_cast<int>(matrix->rows.size()) != expectedRows) {
            std::snprintf(buffer, sizeof buffer,
                          "%d cells imply %d rib rows for the half wing, but "
                          "the matrix has %zu.",
                          matrix->declaredCells, expectedRows,
                          matrix->rows.size());
            problems->push_back(buffer);
        }
        if (matrix->declaredRibs > 0) {
            const int expectedRibs = (matrix->declaredCells % 2 != 0)
                                         ? expectedRows * 2
                                         : expectedRows * 2 - 1;
            if (matrix->declaredRibs != expectedRibs) {
                std::snprintf(buffer, sizeof buffer,
                              "\"Number of ribs\" is %d, but %d cells imply "
                              "%d ribs for the full wing.",
                              matrix->declaredRibs, matrix->declaredCells,
                              expectedRibs);
                problems->push_back(buffer);
            }
        }
    }
    for (size_t i = 0; i < matrix->rows.size(); ++i) {
        const std::vector<double> &v = matrix->rows[i].values;
        if (std::lround(v[0]) != static_cast<long>(i) + 1) {
            std::snprintf(buffer, sizeof buffer,
                          "%s is numbered %g; rows must be numbered 1, 2, "
                          "3... from the centre out.",
                          describeRow(i + 1).c_str(), v[0]);
            problems->push_back(buffer);
        }
        if (v[3] <= v[2]) {
            std::snprintf(buffer, sizeof buffer,
                          "%s: y-TE (%.2f) must be greater than y-LE (%.2f) "
                          "or the chord becomes zero or negative.",
                          describeRow(i + 1).c_str(), v[3], v[2]);
            problems->push_back(buffer);
        }
        if (i > 0 && v[1] <= matrix->rows[i - 1].values[1]) {
            std::snprintf(buffer, sizeof buffer,
                          "%s: x-rib (%.2f) must be greater than the previous "
                          "rib's (%.2f); ribs are ordered centre to tip.",
                          describeRow(i + 1).c_str(), v[1],
                          matrix->rows[i - 1].values[1]);
            problems->push_back(buffer);
        }
        if (v[7] < 0.0 || v[7] > 100.0) {
            std::snprintf(buffer, sizeof buffer,
                          "%s: RP (%.2f) is a %% of chord and should be "
                          "between 0 and 100.",
                          describeRow(i + 1).c_str(), v[7]);
            problems->push_back(buffer);
        }
        if (matrix->columnCount == 11 && (v[10] < 0.0 || v[10] > 100.0)) {
            std::snprintf(buffer, sizeof buffer,
                          "%s: Pos_z (%.1f) is a %% of chord and should be "
                          "between 0 and 100.",
                          describeRow(i + 1).c_str(), v[10]);
            problems->push_back(buffer);
        }
    }
    return true;
}

std::string formatSection1Row(const Section1Row &row)
{
    const std::vector<double> &v = row.values;
    if (v.size() < 9)
        return std::string();
    char buffer[240];
    std::snprintf(buffer, sizeof buffer,
                  "%2d %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f %9.2f %10.2f",
                  static_cast<int>(std::lround(v[0])), v[1], v[2], v[3], v[4],
                  v[5], v[6], v[7], v[8]);
    std::string line = buffer;
    if (v.size() >= 11) {
        std::snprintf(buffer, sizeof buffer, "\t%.2f\t%.1f", v[9], v[10]);
        line += buffer;
    }
    return line;
}

} // namespace lep
