#pragma once

#include <filesystem>
#include <string>

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

void lep_nurbs_set_line_capture(int enabled);

// Names the lines captured next: the 4-character label matches lines.txt
// (e.g. "3A5"), planIndex is the line plan row 1..6 (A..F), and brake
// lines form their own group regardless of plan.
void lep_nurbs_set_line_tag(const char *label,
                            int labelLength,
                            int planIndex,
                            int isBrake);

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
