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
// includeConstructionCurves controls the interior surface-wireframe curve
// groups ("Extrados curves", "Vent curves", "Intrados curves"); suspension
// lines and part boundary edges are always written.
NurbsWriteResult writeNurbsStep(const std::filesystem::path &path,
                                bool includeConstructionCurves = true);

// Writes the coarse Playground simulation mesh (welded full-wing skin
// quads sampled from the ballooning law, rib outline loops, labelled
// suspension lines) as JSON. Millimetres, Z up. Returns false and sets
// error if nothing was captured or the file cannot be written.
bool writeSimMesh(const std::filesystem::path &path, std::string &error);

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

// Captures one H/V/VH diagonal-rib sheet as its two boundary polylines in
// exact rung correspondence: sample j of curve A pairs with sample j of
// curve B, matching the segments the legacy 3D drawing emits. The arrays
// are strided views into the legacy point matrices (stride doubles between
// consecutive samples). The model spans an exact ruled surface between the
// interpolated boundaries and files it in the top-level "Diagonals" group
// as "<kind> <index>", e.g. "V-rib 7" for row 7 of the H/V rib table.
void lep_nurbs_capture_diagonal_strip(const char *kind,
                                      int kindLength,
                                      int index,
                                      const double *xA,
                                      const double *yA,
                                      const double *zA,
                                      const double *xB,
                                      const double *yB,
                                      const double *zB,
                                      int pointCount,
                                      int stride);

// Derives the mini-ribs (section 2 column 8: percent of chord ahead of the
// trailing edge, 1 < pct <= 100, where 100 is a complete unloaded middle
// rib) that the legacy program only ever draws as 2D patterns. x/y/z are
// the final wing coordinates (Fortran (0:100,500), station 0 = mirrored
// rib 1), np the point-count table (0:100,9), rib the parameter table
// (0:100,500), all in f2c column-major layout. Each mini-rib becomes a
// ruled face on the mid-cell section, filed under "Ribs" / "Mini-ribs".
void lep_nurbs_capture_miniribs(const double *x,
                                const double *y,
                                const double *z,
                                const int *np,
                                const double *rib,
                                int ribCount,
                                int singleSkin);

void lep_nurbs_capture_line(double x1,
                            double y1,
                            double z1,
                            double x2,
                            double y2,
                            double z2,
                            int colorIndex);

}
