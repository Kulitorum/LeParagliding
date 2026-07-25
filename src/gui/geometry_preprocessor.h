#pragma once

#include <array>
#include <string>
#include <vector>

namespace lep {

// Port of the standalone "GEOMETRY PRE-PROCESSOR" v1.6 "Canigo" (pre-processor.f,
// Pere Casellas, Laboratori d'envol, GPL-3.0). Generates the rib geometry matrix
// used by Section 1 of leparagliding.txt from analytic leading/trailing edge,
// vault, and cell distribution parameters. Lengths are centimetres, angles degrees.

struct PreProcessorInput
{
    std::string wingName = "wing";

    // 1. Leading edge (type 1: ellipse with one or two exponential corrections)
    double leadingA1 = 710.21;
    double leadingB1 = 243.11;
    double leadingX1 = 375.0;
    double leadingX2 = 475.0;
    double halfSpan = 575.5; // xm, shared by all sections
    double leadingC01 = 48.30;
    double leadingEx1 = 2.0;
    double leadingC02 = 0.0;
    double leadingEx2 = 2.0;

    // 2. Trailing edge (type 1: ellipse with one exponential correction)
    double trailingA1 = 903.01;
    double trailingB1 = 243.11;
    double trailingX1 = 372.50;
    double trailingC0 = -2.45;
    double trailingY0 = 215.20;
    double trailingExp = 2.0;

    // 3. Vault
    int vaultType = 2; // 1 = ellipse + cosine modification, 2 = four tangent arcs
    double vaultA1 = 447.69;
    double vaultB1 = 253.02;
    double vaultX1 = 307.61;
    double vaultC1 = 24.80;
    std::array<double, 4> vaultRadius = {746.10, 527.47, 373.09, 104.64};
    std::array<double, 4> vaultAngle = {15.33, 20.44, 18.62, 35.38};

    // 4. Cell distribution
    int cellDistribution = 3; // 1 uniform, 2 linear, 3 proportional to chord, 4 explicit
    double cellCoefficient = 0.7; // types 2 and 3
    int cellCount = 50;           // types 1 to 3
    std::vector<double> cellWidths; // type 4: width of cell i per matrix row (row 1 may be 0)
};

struct PreProcessorRib
{
    double x = 0.0;      // x-rib: flat spanwise position
    double yLeading = 0.0;
    double yTrailing = 0.0;
    double xProjected = 0.0; // xp: projected spanwise position on the vault
    double z = 0.0;          // vertical drop below the centre chord
    double beta = 0.0;       // rib inclination in degrees
    double rotationPoint = 33.33;
    double washin = 0.0;
};

struct PreProcessorSegment
{
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    int color = 1; // AutoCAD color index, as in the original DXF output
};

struct PreProcessorResult
{
    std::string error; // non-empty when the input was rejected

    std::vector<PreProcessorRib> ribs; // half wing, centre first (matrix rows)
    int cellCount = 0;
    bool evenCellCount = false;
    int totalRibCount = 0; // full wing rib count for Section 1

    double flatSpan = 0.0;      // m
    double projectedSpan = 0.0; // m
    double flatArea = 0.0;      // m2
    double projectedArea = 0.0; // m2
    double flatAspectRatio = 0.0;
    double projectedAspectRatio = 0.0;
    double flattening = 0.0;
    double maxChord = 0.0; // cm
    double midChord = 0.0; // cm
    double minChord = 0.0; // cm

    // Polylines in the drawing plane (planform y grows towards the trailing edge).
    std::vector<std::array<double, 2>> leadingEdge;
    std::vector<std::array<double, 2>> trailingEdge;
    std::vector<std::array<double, 2>> vault; // untranslated: y from vault top down

    std::vector<PreProcessorSegment> drawing; // same content as geometry.dxf

    bool ok() const { return error.empty(); }
};

PreProcessorResult runPreProcessor(const PreProcessorInput &input);

// pre-data.txt round trip (same fixed layout the Fortran program reads).
bool parsePreDataText(const std::string &text, PreProcessorInput *input, std::string *error);
std::string formatPreDataText(const PreProcessorInput &input);

// geometry-out.txt and geometry.dxf equivalents.
std::string formatGeometryOutText(const PreProcessorInput &input, const PreProcessorResult &result);
std::string formatGeometryDxfText(const PreProcessorResult &result);

// Rib rows formatted for Section 1 of leparagliding.txt (11 columns incl. Rot_z, Pos_z).
std::string formatSection1Rows(const PreProcessorResult &result);

} // namespace lep
