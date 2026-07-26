#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace lep {

struct NurbsWriteResult
{
    bool success = false;
    int surfaceCount = 0;
    int splineCount = 0;
    int partCount = 0;
    int ribCount = 0;
    int lineCount = 0;
    int sewnEdgeCount = 0;
    int freeEdgeCount = 0;
    double maximumSourceDeviationMillimetres = 0.0;
    double maximumLegacyAgreementMillimetres = 0.0;
    std::string error;
    // Non-fatal degradations, e.g. an airfoil hole whose legacy definition
    // does not fit inside its rib outline and was left uncut.
    std::vector<std::string> warnings;
};

void resetNurbsModel();
NurbsWriteResult writeNurbsStep(const std::filesystem::path &path);

} // namespace lep

// Narrow C ABI used by the mechanically translated calculation core. The
// legacy core remains the source of the transformed airfoils and analytical
// shaping law; OCCT owns the source-curve interpretation, lofting, topology,
// meshing, and STEP serialization on the other side of this boundary. The
// old tessellation is passed only to validate numerical agreement.
extern "C" {

void lep_nurbs_capture_panel(const double *u,
                             const double *v,
                             const double *w,
                             const double *shapingHeight,
                             const double *legacyTessellation,
                             int panelIndex,
                             int totalPointCount,
                             int upperPointCount,
                             int ventPointCount,
                             int segmentCount,
                             int includeVentSurface,
                             int singleSkin);

// Captures one rib station: the chord-scaled planar profile (vector 3), its
// rigidly placed 3D contour (vector 47), and this rib's airfoil-hole table
// (the legacy hol array, f2c layout). The model builder uses the exact
// planar-to-spatial correspondence to export ribs as planar faces with the
// lightening holes cut out.
void lep_nurbs_capture_rib(const double *u,
                           const double *v,
                           const double *w,
                           const double *holes,
                           double chordCentimetres,
                           int ribIndex,
                           int totalPointCount);

void lep_nurbs_set_line_capture(int enabled);

// Names the lines captured next: the 4-character label matches lines.txt
// (e.g. "3A5"), planIndex is the line plan row 1..6 (A..F), and brake
// lines form their own group regardless of plan. typeName/diameterMm come
// from the section-34 line characteristics table (Fortran blank-padded
// name, diameter in mm); an empty name or non-positive diameter means the
// line has no usable type row and the part keeps its bare label.
void lep_nurbs_set_line_tag(const char *label,
                            int labelLength,
                            int planIndex,
                            int isBrake,
                            const char *typeName,
                            int typeNameLength,
                            double diameterMm);

// Routes the lines captured next into the top-level "Diagonals" assembly
// group as one part labeled "<kind> <index>", e.g. "H-rib 7" for row 7 of
// the H/V rib table.
void lep_nurbs_tag_diagonal(const char *kind,
                            int kindLength,
                            int index);

void lep_nurbs_capture_line(double x1,
                            double y1,
                            double z1,
                            double x2,
                            double y2,
                            double z2,
                            int colorIndex);

}
