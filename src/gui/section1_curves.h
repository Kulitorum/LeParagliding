#pragma once

#include <string>
#include <vector>

namespace lep {

// Parsing, validation and serialization of the Section 1 rib geometry matrix
// (the "* Rib geometric parameters" block of leparagliding.txt). Pure C++ so
// it compiles into the test executables without Qt, mirroring
// geometry_preprocessor.{h,cpp}.

struct Section1Column
{
    const char *id;          // stable identifier, e.g. "beta"
    const char *label;       // column header text, e.g. "beta"
    const char *unit;        // "cm", "deg", "%" or "" when dimensionless
    const char *description; // one-sentence explanation for users
    double minValue;         // editing clamp; +-1e300 when unconstrained
    double maxValue;
    int decimals;            // decimals used when writing the value back
    bool editable;
};

// The 11 columns of the LEP >= 3.16 matrix, in file order. Rib number first;
// 9-column files (LEP < 3.16) simply lack the last two (Rot_z, Pos_z).
const std::vector<Section1Column> &section1Columns();

struct Section1Row
{
    int lineIndex = -1;         // 0-based line within the section text
    std::vector<double> values; // 9 or 11 values, file order
};

struct Section1Matrix
{
    std::vector<Section1Row> rows;
    int columnCount = 0; // 9 or 11
    int declaredCells = -1;
    int declaredRibs = -1;
};

// Extracts the rib matrix from Section 1 text. Returns false when no usable
// matrix exists (*problems then explains why in plain language); when true,
// *problems may still carry warnings about inconsistent or suspicious values.
bool parseSection1Matrix(const std::string &sectionText, Section1Matrix *matrix,
                         std::vector<std::string> *problems);

// One matrix row in the canonical Section 1 column layout (same widths the
// geometry pre-processor emits).
std::string formatSection1Row(const Section1Row &row);

} // namespace lep
