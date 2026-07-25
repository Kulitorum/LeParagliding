#pragma once

#include <filesystem>
#include <string>

namespace lep {

struct NurbsWriteResult
{
    bool success = false;
    int surfaceCount = 0;
    int splineCount = 0;
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

void lep_nurbs_capture_line(double x1,
                            double y1,
                            double z1,
                            double x2,
                            double y2,
                            double z2,
                            int colorIndex);

}
