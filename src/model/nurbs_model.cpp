#include "nurbs_model.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BinXCAFDrivers.hxx>
#include <BSplCLib.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_HArray1.hxx>
#include <NCollection_Sequence.hxx>
#include <OSD_Parallel.hxx>
#include <PCDM_StoreStatus.hxx>
#include <Precision.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <Standard_ConstructionError.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp_XY.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <exception>
#include <limits>
#include <list>
#include <map>
#include <numbers>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

// LEparagliding's engineering coordinates are centimetres. OCCT and the STEP
// files produced here use millimetres, which avoids an ambiguous unitless CAD
// model and matches the convention used by most STEP consumers.
constexpr double millimetresPerCentimetre = 10.0;
constexpr double pointToleranceMillimetres = 0.005;
constexpr double maximumSourceDeviationMillimetres = 0.01;
constexpr double maximumLegacyAgreementMillimetres = 0.00001;
// A curve whose points all stay this close to the symmetry plane is on the
// wing centre; its mirror copy would coincide with it.
constexpr double symmetryPlaneToleranceMillimetres = 0.5;
// Face boundaries are already exact B-spline edges. These sample counts add
// only a clear interior surface wireframe, without duplicating boundaries or
// bloating the STEP presentation.
constexpr int maximumDisplayedSpanSamples = 5;
constexpr int maximumDisplayedChordSamples = 10;

// Stable part-tree vocabulary. The viewport recognises these exact names in
// the STEP assembly, so change them only together with the GUI.
constexpr const char *wingGroupName = "Wing";
constexpr const char *extradosGroupName = "Extrados";
constexpr const char *ventsGroupName = "Vents";
constexpr const char *intradosGroupName = "Intrados";
// The interior surface-wireframe curves live in sibling groups next to the
// skin groups so CAD tools can hide them independently of the surfaces.
constexpr const char *extradosCurvesGroupName = "Extrados curves";
constexpr const char *ventCurvesGroupName = "Vent curves";
constexpr const char *intradosCurvesGroupName = "Intrados curves";
constexpr const char *ribsGroupName = "Ribs";
constexpr const char *linesGroupName = "Lines";
constexpr const char *brakeGroupName = "Brake lines";
constexpr const char *diagonalsGroupName = "Diagonals";
constexpr const char *miniribsGroupName = "Mini-ribs";
constexpr const char *otherCurvesName = "Other curves";
constexpr const char *rightSideName = "Right";
constexpr const char *leftSideName = "Left";
constexpr const char *centerSideName = "Center";

enum class Region
{
    Extrados,
    Vent,
    Intrados,
};

const char *regionGroupName(Region region)
{
    switch (region) {
    case Region::Extrados:
        return extradosGroupName;
    case Region::Vent:
        return ventsGroupName;
    case Region::Intrados:
        return intradosGroupName;
    }
    return extradosGroupName;
}

const char *regionCurvesGroupName(Region region)
{
    switch (region) {
    case Region::Extrados:
        return extradosCurvesGroupName;
    case Region::Vent:
        return ventCurvesGroupName;
    case Region::Intrados:
        return intradosCurvesGroupName;
    }
    return extradosCurvesGroupName;
}

// Region names as used in diagnostics; these match the historical engine
// messages ("upper"/"vent"/"lower").
const char *regionDiagnosticName(Region region)
{
    switch (region) {
    case Region::Extrados:
        return "upper";
    case Region::Vent:
        return "vent";
    case Region::Intrados:
        return "lower";
    }
    return "upper";
}

struct PartColor
{
    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;

    Quantity_Color color() const
    {
        return {red, green, blue, Quantity_TOC_RGB};
    }
};

// Default part colours embedded in the STEP file so external CAD tools show
// a structured wing. The Studio viewport applies its own user-configurable
// palette on top of these.
constexpr PartColor extradosColor{0.20, 0.57, 0.88};
constexpr PartColor intradosColor{0.45, 0.69, 0.90};
constexpr PartColor ventColor{0.93, 0.60, 0.23};
constexpr PartColor wireframeColor{0.37, 0.82, 1.0};
constexpr PartColor ribColor{0.78, 0.80, 0.84};
constexpr PartColor brakeColor{0.95, 0.83, 0.28};
constexpr PartColor otherCurveColor{0.62, 0.66, 0.72};
constexpr PartColor diagonalColor{0.83, 0.45, 0.74};
constexpr PartColor planColors[6] = {
    {0.89, 0.29, 0.29}, // Plan A
    {0.95, 0.62, 0.19}, // Plan B
    {0.35, 0.79, 0.42}, // Plan C
    {0.29, 0.74, 0.86}, // Plan D
    {0.72, 0.47, 0.90}, // Plan E
    {0.62, 0.66, 0.72}, // Plan F
};

PartColor regionColor(Region region)
{
    switch (region) {
    case Region::Extrados:
        return extradosColor;
    case Region::Vent:
        return ventColor;
    case Region::Intrados:
        return intradosColor;
    }
    return extradosColor;
}

struct CapturedLine
{
    gp_Pnt start;
    gp_Pnt end;
    int colorIndex = 0;
    int planIndex = 0;
    bool brake = false;
    std::string label;
};

// A diagonal-rib or mini-rib sheet, captured as its two boundary polylines.
// Matching sample counts are in exact rung correspondence: sample j of
// curveA pairs with sample j of curveB, and the writer spans an exact ruled
// surface through every rung. Mini-rib boundaries have independent counts
// and are paired proportionally instead.
struct CapturedStrip
{
    bool minirib = false;
    std::string label;
    std::vector<gp_Pnt> curveA;
    std::vector<gp_Pnt> curveB;
};

struct SourcePanel
{
    const double *u = nullptr;
    const double *v = nullptr;
    const double *w = nullptr;
    const double *shapingHeight = nullptr;
    const double *legacyTessellation = nullptr;
    int panelIndex = 0;
    int segmentCount = 0;
};

// One skin region of one panel, sampled coarsely on the exact ballooning
// law for the Playground simulation mesh: a row per kept chordwise
// station, spanColumnCount samples across the cell. Rows keep their
// legacy station index so rib loops and neighbouring panels weld on
// identical stations.
constexpr int simSpanColumnCount = 5;
struct SimRegionCapture
{
    int panelIndex = 0;
    // Which skin the rows came from, so the Playground can draw the
    // surfaces separately (hiding the extrados to look inside).
    Region surface = Region::Extrados;
    std::vector<int> stations;
    std::vector<std::array<gp_Pnt, simSpanColumnCount>> rows;

    // True when the panel spans symmetrically across x=0, so mirroring it
    // would reproduce the panel itself.
    [[nodiscard]] bool selfMirrored() const
    {
        if (rows.empty()) {
            return false;
        }
        const auto &row = rows.front();
        return std::abs(row.front().X() + row.back().X()) < 0.5;
    }
};

struct PanelSurface
{
    Region region = Region::Extrados;
    int panelIndex = 0;
    // Chordwise airfoil point range covered by this surface; rib faces use
    // it to know which parts of a rib outline are panel boundaries.
    int firstPoint = 0;
    int lastPoint = 0;
    occ::handle<Geom_BSplineSurface> surface;
    TopoDS_Face rightFace;
    TopoDS_Face leftFace;
    std::vector<TopoDS_Edge> rightWireframe;
    std::vector<TopoDS_Edge> leftWireframe;
};

// One airfoil hole from the legacy hole table, scaled to model millimetres.
// The rotation stays in the legacy convention: radians for ellipses (type 1),
// degrees for triangles and rectangles (types 3 and 4).
struct RibHole
{
    int type = 0;
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;
    double b = 0.0;
    double rotation = 0.0;
    double cornerRadius = 0.0;
};

// A rib station: the chord-scaled planar profile and its rigidly placed 3D
// contour, in exact point correspondence, plus the rib's hole table. The
// planar-to-spatial correspondence recovers the rib plane frame that places
// the hole outlines, which the legacy core only ever draws in 2D.
struct CapturedRib
{
    std::vector<gp_XY> planarPoints;
    std::vector<gp_Pnt> spatialPoints;
    std::vector<RibHole> holes;
};

// The orthonormal frame mapping the rib's planar coordinates into model
// space, fitted from the captured point correspondence and only trusted
// when the fit reproduces every contour point.
struct RibFrame
{
    gp_Pnt origin;
    gp_Vec axisX;
    gp_Vec axisY;

    gp_Pnt point(const gp_XY &planar) const
    {
        return origin.Translated(
            axisX.Multiplied(planar.X()) + axisY.Multiplied(planar.Y()));
    }

    gp_Vec direction(const gp_XY &planar) const
    {
        return axisX.Multiplied(planar.X()) + axisY.Multiplied(planar.Y());
    }
};

// One chordwise airfoil point range of a rib outline covered by a panel
// surface. The rib face splits its outline at these region boundaries.
struct RibBoundarySegment
{
    int firstPoint = 0;
    int lastPoint = 0;
};

struct QuantizedPoint
{
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const QuantizedPoint &) const = default;
};

struct QuantizedPointHash
{
    std::size_t operator()(const QuantizedPoint &point) const noexcept
    {
        std::size_t result = std::hash<std::int64_t>{}(point.x);
        result ^= std::hash<std::int64_t>{}(point.y) + 0x9e3779b9U
                  + (result << 6U) + (result >> 2U);
        result ^= std::hash<std::int64_t>{}(point.z) + 0x9e3779b9U
                  + (result << 6U) + (result >> 2U);
        return result;
    }
};

struct QuantizedSegment
{
    QuantizedPoint first;
    QuantizedPoint second;

    bool operator==(const QuantizedSegment &) const = default;
};

struct QuantizedSegmentHash
{
    std::size_t operator()(const QuantizedSegment &segment) const noexcept
    {
        QuantizedPointHash hash;
        std::size_t result = hash(segment.first);
        result ^= hash(segment.second) + 0x9e3779b9U
                  + (result << 6U) + (result >> 2U);
        return result;
    }
};

// A coincident segment may intentionally carry a different authored row or
// brake role.  Geometry alone is therefore not enough to identify a duplicate
// in the Playground handoff: keep those semantic alternatives, while folding
// repeated captures (including reversed endpoints) of the same physical line.
struct QuantizedSemanticSegment
{
    QuantizedSegment geometry;
    int planIndex = 0;
    bool brake = false;

    bool operator==(const QuantizedSemanticSegment &) const = default;
};

struct QuantizedSemanticSegmentHash
{
    std::size_t operator()(
        const QuantizedSemanticSegment &segment) const noexcept
    {
        QuantizedSegmentHash hash;
        std::size_t result = hash(segment.geometry);
        result ^= std::hash<int>{}(segment.planIndex) + 0x9e3779b9U
                  + (result << 6U) + (result >> 2U);
        result ^= std::hash<bool>{}(segment.brake) + 0x9e3779b9U
                  + (result << 6U) + (result >> 2U);
        return result;
    }
};

QuantizedPoint quantize(const gp_Pnt &point)
{
    constexpr double scale = 1.0 / pointToleranceMillimetres;
    return {
        std::llround(point.X() * scale),
        std::llround(point.Y() * scale),
        std::llround(point.Z() * scale),
    };
}

bool pointLess(const QuantizedPoint &left, const QuantizedPoint &right)
{
    if (left.x != right.x) {
        return left.x < right.x;
    }
    if (left.y != right.y) {
        return left.y < right.y;
    }
    return left.z < right.z;
}

QuantizedSegment quantizeSegment(const gp_Pnt &start, const gp_Pnt &end)
{
    QuantizedPoint first = quantize(start);
    QuantizedPoint second = quantize(end);
    if (pointLess(second, first)) {
        std::swap(first, second);
    }
    return {first, second};
}

gp_Trsf mirrorTransform()
{
    gp_Trsf transform;
    transform.SetMirror(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)));
    return transform;
}

gp_Pnt modelPoint(double x, double y, double z, bool mirrored = false)
{
    // The calculation core uses Z down; CAD and the OCCT viewport use Z up.
    return {
        (mirrored ? -x : x) * millimetresPerCentimetre,
        y * millimetresPerCentimetre,
        -z * millimetresPerCentimetre,
    };
}

bool isFinite(const gp_Pnt &point)
{
    return std::isfinite(point.X())
           && std::isfinite(point.Y())
           && std::isfinite(point.Z());
}

std::string trimmedLabel(const char *label, int labelLength)
{
    std::string result;
    for (int index = 0; index < labelLength; ++index) {
        const char character = label[index];
        if (character != ' ' && character != '\0') {
            result.push_back(character);
        }
    }
    return result;
}

TopoDS_Shape mirrored(const TopoDS_Shape &shape)
{
    BRepBuilderAPI_Transform mirror(shape, mirrorTransform(), true);
    if (!mirror.IsDone()) {
        return {};
    }
    return mirror.Shape();
}

// Recovers the rigid planar-to-spatial frame of a rib by least squares over
// the captured contour correspondence. The legacy chain from the chord-scaled
// profile to the placed airfoil is a composition of rotations and
// translations, so the fit must come back orthonormal and reproduce every
// point; anything else means the correspondence was misread.
bool fitRibFrame(const std::vector<gp_XY> &planarPoints,
                 const std::vector<gp_Pnt> &spatialPoints,
                 RibFrame &frame)
{
    if (planarPoints.size() != spatialPoints.size()
        || planarPoints.size() < 3) {
        return false;
    }
    const double count = static_cast<double>(planarPoints.size());

    gp_XY planarCentroid(0.0, 0.0);
    gp_Vec spatialCentroid(0.0, 0.0, 0.0);
    for (std::size_t index = 0; index < planarPoints.size(); ++index) {
        planarCentroid += planarPoints[index];
        spatialCentroid += gp_Vec(spatialPoints[index].XYZ());
    }
    planarCentroid /= count;
    spatialCentroid /= count;

    double sumXX = 0.0;
    double sumXY = 0.0;
    double sumYY = 0.0;
    gp_Vec sumXP(0.0, 0.0, 0.0);
    gp_Vec sumYP(0.0, 0.0, 0.0);
    for (std::size_t index = 0; index < planarPoints.size(); ++index) {
        const gp_XY planar = planarPoints[index] - planarCentroid;
        const gp_Vec spatial =
            gp_Vec(spatialPoints[index].XYZ()) - spatialCentroid;
        sumXX += planar.X() * planar.X();
        sumXY += planar.X() * planar.Y();
        sumYY += planar.Y() * planar.Y();
        sumXP += spatial.Multiplied(planar.X());
        sumYP += spatial.Multiplied(planar.Y());
    }
    const double determinant = sumXX * sumYY - sumXY * sumXY;
    if (std::abs(determinant) <= Precision::Confusion()) {
        return false;
    }
    frame.axisX = (sumXP.Multiplied(sumYY) - sumYP.Multiplied(sumXY))
                      .Divided(determinant);
    frame.axisY = (sumYP.Multiplied(sumXX) - sumXP.Multiplied(sumXY))
                      .Divided(determinant);
    frame.origin = gp_Pnt(
        (spatialCentroid
         - frame.axisX.Multiplied(planarCentroid.X())
         - frame.axisY.Multiplied(planarCentroid.Y()))
            .XYZ());

    constexpr double rigidityTolerance = 1.0e-6;
    if (std::abs(frame.axisX.Magnitude() - 1.0) > rigidityTolerance
        || std::abs(frame.axisY.Magnitude() - 1.0) > rigidityTolerance
        || std::abs(frame.axisX.Dot(frame.axisY)) > rigidityTolerance) {
        return false;
    }
    for (std::size_t index = 0; index < planarPoints.size(); ++index) {
        if (frame.point(planarPoints[index])
                .Distance(spatialPoints[index])
            > pointToleranceMillimetres) {
            return false;
        }
    }
    return true;
}

// Winding of a closed planar wire about the given plane normal, measured as
// the signed enclosed area. Used only for a sign decision, so a modest
// sampling of each edge is exact enough.
double signedAreaAlongNormal(const TopoDS_Wire &wire,
                             const gp_Pnt &origin,
                             const gp_Dir &normal)
{
    gp_Vec area(0.0, 0.0, 0.0);
    for (BRepTools_WireExplorer explorer(wire);
         explorer.More();
         explorer.Next()) {
        const TopoDS_Edge &edge = explorer.Current();
        double first = 0.0;
        double last = 0.0;
        const occ::handle<Geom_Curve> curve =
            BRep_Tool::Curve(edge, first, last);
        if (curve.IsNull()) {
            continue;
        }
        if (edge.Orientation() == TopAbs_REVERSED) {
            std::swap(first, last);
        }
        constexpr int sampleCount = 32;
        gp_Pnt previous = curve->Value(first);
        for (int sample = 1; sample <= sampleCount; ++sample) {
            const gp_Pnt current = curve->Value(
                first
                + (last - first) * static_cast<double>(sample)
                      / static_cast<double>(sampleCount));
            area += gp_Vec(origin, previous)
                        .Crossed(gp_Vec(origin, current))
                        .Multiplied(0.5);
            previous = current;
        }
    }
    return area.Dot(gp_Vec(normal));
}

gp_XY projectOntoLine(const gp_XY &lineStart,
                      const gp_XY &lineEnd,
                      const gp_XY &point)
{
    gp_XY direction = lineEnd - lineStart;
    const double length = direction.Modulus();
    if (length <= Precision::Confusion()) {
        return lineStart;
    }
    direction /= length;
    return lineStart + direction * ((point - lineStart) * direction);
}

// One leaf part of the exported assembly: a named shape with default
// surface/curve display colours.
struct AssemblyPart
{
    std::string name;
    TopoDS_Shape shape;
    PartColor faceColor;
    PartColor curveColor;
    bool hasFaces = false;
};

// A named group of leaf parts (one level of the assembly tree). Children
// live in a std::list so references and pointers to a group stay valid
// while sibling groups are still being created.
struct AssemblyGroup
{
    std::string name;
    std::list<AssemblyGroup> groups;
    std::vector<AssemblyPart> parts;

    AssemblyGroup &group(const std::string &groupName)
    {
        for (AssemblyGroup &child : groups) {
            if (child.name == groupName) {
                return child;
            }
        }
        groups.push_back({groupName, {}, {}});
        return groups.back();
    }

    bool empty() const
    {
        return parts.empty()
               && std::all_of(
                   groups.begin(),
                   groups.end(),
                   [](const AssemblyGroup &child) { return child.empty(); });
    }
};

class NurbsModel
{
public:
    void reset()
    {
        panels_.clear();
        capturedRibs_.clear();
        capturedLines_.clear();
        capturedStrips_.clear();
        simRegions_.clear();
        errors_.clear();
        warnings_.clear();
        captureLines_ = false;
        currentLineTag_ = {};
        maximumSourceDeviation_ = 0.0;
        maximumLegacyAgreement_ = 0.0;
    }

    void capturePanel(const double *u,
                      const double *v,
                      const double *w,
                      const double *shapingHeight,
                      const double *legacyTessellation,
                      int panelIndex,
                      int totalPointCount,
                      int upperPointCount,
                      int ventPointCount,
                      int segmentCount,
                      bool includeVentSurface,
                      bool singleSkin)
    {
        if (u == nullptr
            || v == nullptr
            || w == nullptr
            || shapingHeight == nullptr
            || legacyTessellation == nullptr
            || panelIndex < 1
            || panelIndex > 100
            || totalPointCount < 2
            || totalPointCount > 500
            || upperPointCount < 2
            || upperPointCount > totalPointCount
            || ventPointCount < 2
            || segmentCount < 1
            || segmentCount > 98) {
            errors_.push_back(
                "Rejected invalid source-shape dimensions for panel "
                + std::to_string(panelIndex));
            return;
        }

        const int ventLast = upperPointCount + ventPointCount - 1;
        if (ventLast > totalPointCount) {
            errors_.push_back(
                "Vent point range exceeds the airfoil range for panel "
                + std::to_string(panelIndex));
            return;
        }

        const SourcePanel source{
            u,
            v,
            w,
            shapingHeight,
            legacyTessellation,
            panelIndex,
            segmentCount,
        };
        addRegion(source,
                  panelIndex,
                  1,
                  upperPointCount,
                  Region::Extrados);
        if (includeVentSurface) {
            addRegion(source,
                      panelIndex,
                      upperPointCount,
                      ventLast,
                      Region::Vent);
        }
        if (!singleSkin && ventLast < totalPointCount) {
            addRegion(source,
                      panelIndex,
                      ventLast,
                      totalPointCount,
                      Region::Intrados);
        }

        // Coarse skin samples for the Playground simulation mesh. Unlike
        // the STEP model, the vent wall is always captured: the toy
        // stamps a uniform pressure difference per face, so open intakes
        // would both leave flapping rims and give the closed-surface
        // pressure field a spurious net thrust through the hole.
        captureSimRegion(source,
                         panelIndex,
                         1,
                         upperPointCount,
                         16,
                         Region::Extrados);
        captureSimRegion(source,
                         panelIndex,
                         upperPointCount,
                         ventLast,
                         3,
                         Region::Vent);
        if (!singleSkin && ventLast < totalPointCount) {
            captureSimRegion(source,
                             panelIndex,
                             ventLast,
                             totalPointCount,
                             12,
                             Region::Intrados);
        }
    }

    void captureRib(const double *u,
                    const double *v,
                    const double *w,
                    const double *holes,
                    double chordCentimetres,
                    int ribIndex,
                    int totalPointCount)
    {
        if (u == nullptr
            || v == nullptr
            || w == nullptr
            || holes == nullptr
            || ribIndex < 0
            || ribIndex > 100
            || totalPointCount < 3
            || totalPointCount > 500) {
            errors_.push_back(
                "Rejected invalid source-shape dimensions for rib "
                + std::to_string(ribIndex));
            return;
        }

        CapturedRib rib;
        rib.planarPoints.reserve(
            static_cast<std::size_t>(totalPointCount));
        rib.spatialPoints.reserve(
            static_cast<std::size_t>(totalPointCount));
        for (int pointIndex = 1;
             pointIndex <= totalPointCount;
             ++pointIndex) {
            const std::size_t planarIndex =
                sourceFieldIndex(ribIndex, pointIndex, 3);
            rib.planarPoints.emplace_back(
                u[planarIndex] * millimetresPerCentimetre,
                v[planarIndex] * millimetresPerCentimetre);
            const std::size_t spatialIndex =
                sourceFieldIndex(ribIndex, pointIndex, 47);
            rib.spatialPoints.push_back(
                modelPoint(u[spatialIndex],
                           v[spatialIndex],
                           w[spatialIndex]));
            if (!isFinite(rib.spatialPoints.back())) {
                errors_.push_back(
                    "Non-finite contour point in rib "
                    + std::to_string(ribIndex));
                return;
            }
        }

        const double chordMillimetres =
            chordCentimetres * millimetresPerCentimetre;
        const int holeCount = std::clamp(
            static_cast<int>(holes[ribIndex]), 0, 20);
        for (int holeIndex = 1; holeIndex <= holeCount; ++holeIndex) {
            const auto field = [&](int fieldIndex) {
                return holes[holeFieldIndex(ribIndex, holeIndex, fieldIndex)];
            };
            RibHole hole;
            hole.type = static_cast<int>(field(9));
            // Types 1, 3 and 4 are the airfoil holes the legacy core draws
            // into rib patterns; type 11 parameterizes unloaded miniribs,
            // which are not rib stations, and other values are ignored by
            // the legacy drawing code as well.
            if (hole.type != 1 && hole.type != 3 && hole.type != 4) {
                continue;
            }
            hole.x = field(2) * chordMillimetres / 100.0;
            hole.y = field(3) * chordMillimetres / 100.0;
            hole.a = field(4) * chordMillimetres / 100.0;
            hole.b = field(5) * chordMillimetres / 100.0;
            hole.rotation = field(6);
            hole.cornerRadius = field(7) * chordMillimetres / 100.0;
            rib.holes.push_back(hole);
        }

        capturedRibs_[ribIndex] = std::move(rib);
    }

    void captureLine(CapturedLine line)
    {
        if (!captureLines_
            || !isFinite(line.start)
            || !isFinite(line.end)
            || line.start.Distance(line.end) <= Precision::Confusion()) {
            return;
        }
        line.planIndex = currentLineTag_.planIndex;
        line.brake = currentLineTag_.brake;
        line.label = currentLineTag_.label;
        capturedLines_.push_back(std::move(line));
    }

    void setLineCapture(bool enabled)
    {
        captureLines_ = enabled;
        if (!enabled) {
            currentLineTag_ = {};
        }
    }

    void setLineTag(const char *label,
                    int labelLength,
                    int planIndex,
                    bool brake,
                    const char *typeName,
                    int typeNameLength,
                    double diameterMm)
    {
        std::string title =
            label != nullptr && labelLength > 0
                ? trimmedLabel(label, labelLength)
                : std::string();
        const std::string type =
            typeName != nullptr && typeNameLength > 0
                ? trimmedLabel(typeName, typeNameLength)
                : std::string();
        if (!type.empty()
            && std::isfinite(diameterMm)
            && diameterMm > 0.0) {
            std::ostringstream suffix;
            suffix << type << " " << diameterMm << " mm";
            title = title.empty()
                        ? suffix.str()
                        : title + " (" + suffix.str() + ")";
        }
        currentLineTag_.label = std::move(title);
        currentLineTag_.planIndex = std::clamp(planIndex, 0, 6);
        currentLineTag_.brake = brake;
    }

    void captureDiagonalStrip(const char *kind,
                              int kindLength,
                              int index,
                              const double *xA,
                              const double *yA,
                              const double *zA,
                              const double *xB,
                              const double *yB,
                              const double *zB,
                              int pointCount,
                              int stride)
    {
        if (xA == nullptr || yA == nullptr || zA == nullptr
            || xB == nullptr || yB == nullptr || zB == nullptr
            || pointCount < 2 || stride < 1) {
            return;
        }
        CapturedStrip strip;
        strip.label = (kind != nullptr && kindLength > 0
                           ? trimmedLabel(kind, kindLength)
                           : std::string("Diagonal"))
                      + " " + std::to_string(index);
        strip.curveA.reserve(pointCount);
        strip.curveB.reserve(pointCount);
        for (int sample = 0; sample < pointCount; ++sample) {
            const int offset = sample * stride;
            const gp_Pnt a =
                modelPoint(xA[offset], yA[offset], zA[offset]);
            const gp_Pnt b =
                modelPoint(xB[offset], yB[offset], zB[offset]);
            if (!isFinite(a) || !isFinite(b)) {
                return;
            }
            strip.curveA.push_back(a);
            strip.curveB.push_back(b);
        }
        capturedStrips_.push_back(std::move(strip));
    }

    void captureMiniribs(const double *x,
                         const double *y,
                         const double *z,
                         const int *np,
                         const double *rib,
                         int ribCount,
                         bool singleSkin)
    {
        if (x == nullptr || y == nullptr || z == nullptr || np == nullptr
            || rib == nullptr || singleSkin) {
            return;
        }
        // The legacy arrays keep their f2c layouts: x/y/z and rib are
        // Fortran (0:100,500) matrices, np is (0:100,9), all column-major
        // with 101 rows. Station 0 already holds the mirrored first rib.
        const auto midPoint = [&](int station, int j) {
            const int offset = station + (j - 1) * 101;
            return modelPoint(0.5 * (x[offset - 1] + x[offset]),
                              0.5 * (y[offset - 1] + y[offset]),
                              0.5 * (z[offset - 1] + z[offset]));
        };
        const auto ribValue = [&](int station, int column) {
            return rib[station + (column - 1) * 101];
        };
        for (int i = 1; i <= std::min(ribCount, 100); ++i) {
            const double percent = ribValue(i, 56);
            if (percent <= 1.0) {
                continue;
            }
            const int totalPoints = np[i];
            const int upperPoints = np[i + 101];
            if (totalPoints < 4 || totalPoints > 500 || upperPoints < 2
                || upperPoints >= totalPoints) {
                continue;
            }
            // Mid-cell section between ribs i-1 and i, where the legacy
            // program sews the mini-rib but never draws it in 3D.
            std::vector<gp_Pnt> section(totalPoints + 1);
            bool finite = true;
            for (int j = 1; j <= totalPoints; ++j) {
                section[j] = midPoint(i, j);
                finite = finite && isFinite(section[j]);
            }
            const gp_Pnt trailing = section[1];
            const gp_Pnt leading = section[upperPoints];
            if (!finite
                || trailing.Distance(leading) <= Precision::Confusion()) {
                continue;
            }
            const gp_Dir chordDirection(gp_Vec(trailing, leading));
            const auto chordwise = [&](const gp_Pnt &point) {
                return gp_Vec(trailing, point).Dot(chordDirection);
            };
            // The cut runs rib(i-1,5)*pct/100 centimetres ahead of the
            // trailing edge, measured on the left rib chord exactly as the
            // legacy pattern code does; 100% keeps the whole section (a
            // complete unloaded middle rib).
            double chord = ribValue(i - 1, 5);
            if (chord <= 0.0) {
                chord = ribValue(i, 5);
            }
            const double cut = chord * millimetresPerCentimetre
                               * std::min(percent, 100.0) / 100.0;
            const auto appendCut = [&](std::vector<gp_Pnt> &curve,
                                       int fromIndex,
                                       int toIndex) {
                const double before = chordwise(section[fromIndex]);
                const double after = chordwise(section[toIndex]);
                if (after <= before + Precision::Confusion()) {
                    return;
                }
                const double t =
                    std::clamp((cut - before) / (after - before), 0.0, 1.0);
                if (t <= Precision::Confusion()) {
                    return;
                }
                const gp_Pnt &from = section[fromIndex];
                const gp_Pnt &to = section[toIndex];
                curve.emplace_back(from.X() + t * (to.X() - from.X()),
                                   from.Y() + t * (to.Y() - from.Y()),
                                   from.Z() + t * (to.Z() - from.Z()));
            };

            CapturedStrip strip;
            strip.minirib = true;
            strip.label = "Mini-rib " + std::to_string(i);
            // Upper boundary: trailing edge forward along the extrados.
            int j = 1;
            strip.curveA.push_back(section[1]);
            while (j + 1 <= upperPoints
                   && chordwise(section[j + 1]) < cut) {
                ++j;
                strip.curveA.push_back(section[j]);
            }
            if (j + 1 <= upperPoints) {
                appendCut(strip.curveA, j, j + 1);
            }
            // Lower boundary: trailing edge forward along the intrados,
            // continuing across the vent contour when a 100% rib reaches
            // the leading edge.
            j = totalPoints;
            strip.curveB.push_back(section[totalPoints]);
            while (j - 1 > upperPoints
                   && chordwise(section[j - 1]) < cut) {
                --j;
                strip.curveB.push_back(section[j]);
            }
            if (j - 1 > upperPoints) {
                appendCut(strip.curveB, j, j - 1);
            }
            if (strip.curveA.size() >= 2 && strip.curveB.size() >= 2) {
                capturedStrips_.push_back(std::move(strip));
            }
        }
    }

    lep::NurbsWriteResult writeStep(const std::filesystem::path &path,
                                    bool includeConstructionCurves)
    {
        warnings_.clear();
        lep::NurbsWriteResult result;
        result.maximumSourceDeviationMillimetres =
            maximumSourceDeviation_;
        result.maximumLegacyAgreementMillimetres =
            maximumLegacyAgreement_;

        if (!errors_.empty()) {
            result.error = errors_.front();
            return result;
        }
        if (panels_.empty()) {
            result.error = "The calculation produced no NURBS surfaces.";
            return result;
        }

        try {
            validateSewing(result);
            if (!result.error.empty()) {
                return result;
            }

            AssemblyGroup wing{wingGroupName, {}, {}};
            addPanelParts(wing, result, includeConstructionCurves);
            addRibParts(wing, result);
            if (!errors_.empty()) {
                result.error = errors_.front();
                return result;
            }
            addLineParts(wing, result);
            addStripParts(wing, result);

            writeAssembly(wing, path, result);
        } catch (const Standard_Failure &failure) {
            result.error =
                std::string("OCCT STEP export failed: ")
                + (failure.GetMessageString() != nullptr
                       ? failure.GetMessageString()
                       : "unknown OCCT error");
            return result;
        } catch (const std::exception &exception) {
            result.error =
                std::string("STEP export failed: ") + exception.what();
            return result;
        }

        result.warnings = warnings_;
        result.success = result.error.empty();
        return result;
    }

private:
    struct LineTag
    {
        std::string label;
        int planIndex = 0;
        bool brake = false;
    };

    static std::size_t sourceFieldIndex(int panelIndex,
                                        int pointIndex,
                                        int fieldIndex)
    {
        // f2c layout for real*8 u/v/w(0:100,500,99), using the
        // unadjusted arrays owned by MAIN__.
        return static_cast<std::size_t>(
            panelIndex + (pointIndex + fieldIndex * 500) * 101
            - 50601);
    }

    static std::size_t holeFieldIndex(int ribIndex,
                                      int holeIndex,
                                      int fieldIndex)
    {
        // f2c layout of the legacy hol array: hole property fieldIndex
        // (2 x%, 3 y%, 4 a%, 5 b%, 6 rotation, 7 corner radius %, 9 type)
        // of hole holeIndex, encoded as hol(rib, holeIndex + fieldIndex*200).
        // The hole count itself sits at the plain rib index.
        return static_cast<std::size_t>(
            ribIndex + (holeIndex + fieldIndex * 200) * 101 - 20301);
    }

    static std::size_t shapingHeightIndex(int panelIndex,
                                          int pointIndex)
    {
        // f2c layout for real*8 hautok(0:100,500).
        return static_cast<std::size_t>(
            panelIndex + pointIndex * 101 - 101);
    }

    static std::size_t tessellationIndex(int panelIndex, int pointIndex, int segmentIndex)
    {
        // f2c layout for real*8 tesse3d(3,0:100,500,99). This is the
        // unadjusted array passed by MAIN__; X/Y/Z occupy consecutive slots.
        return static_cast<std::size_t>(
            (panelIndex + (pointIndex + segmentIndex * 500) * 101) * 3
            - 151803);
    }

    gp_Pnt tessellationPoint(const double *tessellation,
                             int panelIndex,
                             int pointIndex,
                             int segmentIndex) const
    {
        const std::size_t index =
            tessellationIndex(panelIndex, pointIndex, segmentIndex);
        return modelPoint(tessellation[index],
                          tessellation[index + 1],
                          tessellation[index + 2]);
    }

    static gp_Pnt sourceControlPoint(const SourcePanel &source,
                                     int panelIndex,
                                     int pointIndex,
                                     int fieldIndex)
    {
        const std::size_t index =
            sourceFieldIndex(panelIndex, pointIndex, fieldIndex);
        return modelPoint(source.u[index],
                          source.v[index],
                          source.w[index]);
    }

    static double sourceShapingHeight(const SourcePanel &source,
                                      int pointIndex)
    {
        return source.shapingHeight[
                   shapingHeightIndex(source.panelIndex, pointIndex)]
               * millimetresPerCentimetre;
    }

    gp_Pnt sourceShapePoint(const SourcePanel &source,
                            int pointIndex,
                            double spanParameter) const
    {
        const gp_Pnt start =
            sourceControlPoint(
                source,
                source.panelIndex,
                pointIndex,
                47);
        const gp_Pnt end =
            sourceControlPoint(
                source,
                source.panelIndex - 1,
                pointIndex,
                47);
        const gp_Vec spanVector(start, end);
        const double spanLength = spanVector.Magnitude();
        if (spanLength <= Precision::Confusion()) {
            return start;
        }

        gp_Vec spanDirection = spanVector;
        spanDirection.Normalize();

        const double height = sourceShapingHeight(source, pointIndex);
        double alongSpan = spanParameter * spanLength;
        double normalOffset = 0.0;

        // This is the analytical circular ballooning law in tessella_. It is
        // evaluated directly from the transformed airfoil stations and
        // shaping height; no STL/DXF tessellation is involved.
        if (height >= 0.01 * millimetresPerCentimetre) {
            const double q =
                spanLength * spanLength / (8.0 * height)
                - 0.5 * height;
            const double radius = q + height;
            const double theta =
                std::atan(q / (0.5 * spanLength));
            const double omega =
                std::numbers::pi - 2.0 * theta;
            const double alpha =
                theta + omega * spanParameter;
            alongSpan =
                0.5 * spanLength - radius * std::cos(alpha);
            normalOffset =
                radius * std::sin(alpha) - q;
        }

        gp_Vec offset = spanDirection.Multiplied(alongSpan);
        if (normalOffset != 0.0) {
            const gp_Pnt normalStart =
                sourceControlPoint(
                    source,
                    source.panelIndex,
                    pointIndex,
                    48);
            const gp_Pnt normalEnd =
                sourceControlPoint(
                    source,
                    source.panelIndex,
                    pointIndex,
                    49);
            gp_Vec normalDirection(normalStart, normalEnd);
            if (normalDirection.Magnitude()
                <= Precision::Confusion()) {
                throw Standard_ConstructionError(
                    "Zero source shaping direction");
            }
            normalDirection.Normalize();
            offset += normalDirection.Multiplied(normalOffset);
        }
        return start.Translated(offset);
    }

    occ::handle<Geom_BSplineCurve> makeSourceSpanCurve(
        const SourcePanel &source,
        int pointIndex) const
    {
        const gp_Pnt start =
            sourceControlPoint(
                source,
                source.panelIndex,
                pointIndex,
                47);
        const gp_Pnt end =
            sourceControlPoint(
                source,
                source.panelIndex - 1,
                pointIndex,
                47);
        const gp_Vec spanVector(start, end);
        const double spanLength = spanVector.Magnitude();
        if (spanLength <= Precision::Confusion()) {
            return {};
        }

        const double height = sourceShapingHeight(source, pointIndex);
        if (height < 0.01 * millimetresPerCentimetre) {
            return makeLinearSpline(start, end);
        }

        gp_Vec spanDirection = spanVector;
        spanDirection.Normalize();

        const gp_Pnt normalStart =
            sourceControlPoint(
                source,
                source.panelIndex,
                pointIndex,
                48);
        const gp_Pnt normalEnd =
            sourceControlPoint(
                source,
                source.panelIndex,
                pointIndex,
                49);
        gp_Vec normalDirection(normalStart, normalEnd);
        if (normalDirection.Magnitude() <= Precision::Confusion()) {
            return {};
        }
        normalDirection.Normalize();

        // A circle converted to a rational B-spline is exact. Transforming
        // its poles by the source span/shaping basis also exactly preserves
        // the source law if that local basis is slightly skewed.
        GC_MakeArcOfCircle makeArc(
            gp_Pnt(0.0, 0.0, 0.0),
            gp_Pnt(0.5 * spanLength, 0.0, height),
            gp_Pnt(spanLength, 0.0, 0.0));
        if (!makeArc.IsDone()) {
            return {};
        }
        occ::handle<Geom_BSplineCurve> curve =
            GeomConvert::CurveToBSplineCurve(
                makeArc.Value(),
                Convert_QuasiAngular);
        if (curve.IsNull()) {
            return {};
        }

        // The conversion leaves an arbitrary uniform scale on the arc's
        // weights (scaling every weight by one constant is the identity on
        // a rational curve). Pin the end weights to exactly 1 so the arc
        // sections agree with the unit-weight straight sections at the
        // strip boundaries and the loft's chordwise weight function has no
        // step where straight and ballooned spans meet.
        const double endWeight = curve->Weight(1);
        if (endWeight > 0.0) {
            for (int poleIndex = 1;
                 poleIndex <= curve->NbPoles();
                 ++poleIndex) {
                curve->SetWeight(
                    poleIndex,
                    curve->Weight(poleIndex) / endWeight);
            }
        }

        for (int poleIndex = 1;
             poleIndex <= curve->NbPoles();
             ++poleIndex) {
            const gp_Pnt localPole = curve->Pole(poleIndex);
            gp_Vec offset =
                spanDirection.Multiplied(localPole.X());
            offset += normalDirection.Multiplied(localPole.Z());
            curve->SetPole(
                poleIndex,
                start.Translated(offset));
        }
        return curve;
    }

    // Skins the span sections into a tensor-product surface that contains
    // every section exactly: the sections are merged onto one shared
    // degree and knot structure (degree elevation and knot insertion are
    // exact), and their homogeneous poles are interpolated across the
    // chord at the given parameters. GeomFill_NSections used to build
    // this surface with a global least-squares fit in homogeneous space;
    // where degree-1 straight spans meet rational ballooning arcs (at the
    // trailing edge) its fitted weight function rang by about 1.5%,
    // growing spanwise fins that scale with the distance from the model
    // origin — worst at the wing tips and invisible to sampling at the
    // stations themselves. Interpolated poles stay bounded by the pole
    // data, so no such fins can appear.
    static occ::handle<Geom_BSplineSurface> skinSections(
        const std::vector<occ::handle<Geom_BSplineCurve>> &sections,
        const NCollection_Sequence<double> &sectionParameters)
    {
        const int sectionCount = static_cast<int>(sections.size());
        if (sectionCount < 2) {
            return {};
        }

        int uDegree = 1;
        bool rational = false;
        for (const occ::handle<Geom_BSplineCurve> &section : sections) {
            uDegree = std::max(uDegree, section->Degree());
            rational = rational || section->IsRational();
        }

        // Unify to the common degree and the union of the normalized
        // interior knots at each knot's maximum multiplicity.
        constexpr double knotTolerance = 1.0e-9;
        std::vector<occ::handle<Geom_BSplineCurve>> unified;
        unified.reserve(sections.size());
        std::vector<std::pair<double, int>> interiorKnots;
        for (const occ::handle<Geom_BSplineCurve> &section : sections) {
            occ::handle<Geom_BSplineCurve> copy =
                occ::handle<Geom_BSplineCurve>::DownCast(section->Copy());
            copy->IncreaseDegree(uDegree);
            const double first = copy->FirstParameter();
            const double range = copy->LastParameter() - first;
            for (int knotIndex = 2;
                 knotIndex < copy->NbKnots();
                 ++knotIndex) {
                const double knot =
                    (copy->Knot(knotIndex) - first) / range;
                const int multiplicity = copy->Multiplicity(knotIndex);
                const auto match = std::find_if(
                    interiorKnots.begin(),
                    interiorKnots.end(),
                    [&](const std::pair<double, int> &entry) {
                        return std::abs(entry.first - knot)
                               <= knotTolerance;
                    });
                if (match == interiorKnots.end()) {
                    interiorKnots.emplace_back(knot, multiplicity);
                } else {
                    match->second =
                        std::max(match->second, multiplicity);
                }
            }
            unified.push_back(copy);
        }
        std::sort(interiorKnots.begin(), interiorKnots.end());
        for (const occ::handle<Geom_BSplineCurve> &curve : unified) {
            const double first = curve->FirstParameter();
            const double range = curve->LastParameter() - first;
            for (const auto &[knot, multiplicity] : interiorKnots) {
                curve->InsertKnot(first + knot * range,
                                  multiplicity,
                                  knotTolerance * range);
            }
        }
        const int uPoleCount = unified.front()->NbPoles();
        for (const occ::handle<Geom_BSplineCurve> &curve : unified) {
            if (curve->NbPoles() != uPoleCount) {
                return {};
            }
        }

        // Chordwise interpolation: clamped knots averaged from the
        // parameters, satisfying Schoenberg-Whitney so the system below
        // is solvable.
        const int vDegree = std::min(3, sectionCount - 1);
        NCollection_Array1<double> parameters(1, sectionCount);
        for (int index = 1; index <= sectionCount; ++index) {
            parameters.SetValue(index, sectionParameters.Value(index));
        }
        NCollection_Array1<double> flatKnots(1, sectionCount + vDegree + 1);
        for (int index = 1; index <= vDegree + 1; ++index) {
            flatKnots.SetValue(index, parameters.Value(1));
            flatKnots.SetValue(sectionCount + index,
                               parameters.Value(sectionCount));
        }
        for (int index = 1;
             index <= sectionCount - vDegree - 1;
             ++index) {
            double sum = 0.0;
            for (int offset = 1; offset <= vDegree; ++offset) {
                sum += parameters.Value(index + offset);
            }
            flatKnots.SetValue(vDegree + 1 + index, sum / vDegree);
        }

        // One interpolation solve over all pole rows at once, on
        // homogeneous (weighted) coordinates so rational sections are
        // reproduced exactly.
        const int pointDimension = rational ? 4 : 3;
        const int rowDimension = uPoleCount * pointDimension;
        NCollection_Array1<double> data(1, sectionCount * rowDimension);
        for (int sectionIndex = 1;
             sectionIndex <= sectionCount;
             ++sectionIndex) {
            const occ::handle<Geom_BSplineCurve> &curve =
                unified[static_cast<std::size_t>(sectionIndex - 1)];
            int slot = (sectionIndex - 1) * rowDimension + 1;
            for (int poleIndex = 1;
                 poleIndex <= uPoleCount;
                 ++poleIndex) {
                const gp_Pnt pole = curve->Pole(poleIndex);
                const double weight =
                    curve->IsRational() ? curve->Weight(poleIndex) : 1.0;
                data.SetValue(slot++, pole.X() * weight);
                data.SetValue(slot++, pole.Y() * weight);
                data.SetValue(slot++, pole.Z() * weight);
                if (rational) {
                    data.SetValue(slot++, weight);
                }
            }
        }
        NCollection_Array1<int> contactOrder(1, sectionCount);
        contactOrder.Init(0);
        int inversionProblem = 0;
        BSplCLib::Interpolate(vDegree,
                              flatKnots,
                              parameters,
                              contactOrder,
                              rowDimension,
                              data.ChangeValue(1),
                              inversionProblem);
        if (inversionProblem != 0) {
            return {};
        }

        NCollection_Array2<gp_Pnt> poles(1, uPoleCount, 1, sectionCount);
        NCollection_Array2<double> weights(1, uPoleCount, 1, sectionCount);
        for (int sectionIndex = 1;
             sectionIndex <= sectionCount;
             ++sectionIndex) {
            int slot = (sectionIndex - 1) * rowDimension + 1;
            for (int poleIndex = 1;
                 poleIndex <= uPoleCount;
                 ++poleIndex) {
                const double x = data.Value(slot);
                const double y = data.Value(slot + 1);
                const double z = data.Value(slot + 2);
                const double weight =
                    rational ? data.Value(slot + 3) : 1.0;
                slot += pointDimension;
                if (weight <= Precision::Confusion()) {
                    return {};
                }
                poles.SetValue(poleIndex,
                               sectionIndex,
                               gp_Pnt(x / weight, y / weight, z / weight));
                weights.SetValue(poleIndex, sectionIndex, weight);
            }
        }

        NCollection_Array1<double> uKnots(
            1, static_cast<int>(interiorKnots.size()) + 2);
        NCollection_Array1<int> uMults(
            1, static_cast<int>(interiorKnots.size()) + 2);
        uKnots.SetValue(1, 0.0);
        uMults.SetValue(1, uDegree + 1);
        for (int index = 0;
             index < static_cast<int>(interiorKnots.size());
             ++index) {
            uKnots.SetValue(index + 2, interiorKnots[index].first);
            uMults.SetValue(index + 2, interiorKnots[index].second);
        }
        uKnots.SetValue(uKnots.Upper(), 1.0);
        uMults.SetValue(uMults.Upper(), uDegree + 1);

        const int interiorVCount = sectionCount - vDegree - 1;
        NCollection_Array1<double> vKnots(1, interiorVCount + 2);
        NCollection_Array1<int> vMults(1, interiorVCount + 2);
        vKnots.SetValue(1, parameters.Value(1));
        vMults.SetValue(1, vDegree + 1);
        for (int index = 1; index <= interiorVCount; ++index) {
            vKnots.SetValue(index + 1,
                            flatKnots.Value(vDegree + 1 + index));
            vMults.SetValue(index + 1, 1);
        }
        vKnots.SetValue(vKnots.Upper(), parameters.Value(sectionCount));
        vMults.SetValue(vMults.Upper(), vDegree + 1);

        if (rational) {
            return new Geom_BSplineSurface(poles,
                                           weights,
                                           uKnots,
                                           vKnots,
                                           uMults,
                                           vMults,
                                           uDegree,
                                           vDegree);
        }
        return new Geom_BSplineSurface(poles,
                                       uKnots,
                                       vKnots,
                                       uMults,
                                       vMults,
                                       uDegree,
                                       vDegree);
    }

    void addRegion(const SourcePanel &source,
                   int panelIndex,
                   int firstPoint,
                   int lastPoint,
                   Region region)
    {
        if (lastPoint - firstPoint + 1 < 2) {
            return;
        }

        addSurface(source,
                   panelIndex,
                   firstPoint,
                   lastPoint,
                   region);
    }

    void addSurface(const SourcePanel &source,
                    int panelIndex,
                    int firstPoint,
                    int lastPoint,
                    Region region)
    {
        const char *regionName = regionDiagnosticName(region);
        const int chordPointCount = lastPoint - firstPoint + 1;

        try {
            std::vector<occ::handle<Geom_BSplineCurve>> sections;
            sections.reserve(static_cast<std::size_t>(chordPointCount));
            NCollection_Sequence<double> sectionParameters;

            for (int pointIndex = firstPoint;
                 pointIndex <= lastPoint;
                 ++pointIndex) {
                occ::handle<Geom_BSplineCurve> curve =
                    makeSourceSpanCurve(source, pointIndex);
                if (curve.IsNull()) {
                    errors_.push_back(
                        "Could not create the analytical "
                        + std::string(regionName)
                        + " span curve for panel "
                        + std::to_string(panelIndex));
                    return;
                }

                const gp_Pnt midpoint =
                    sourceShapePoint(source, pointIndex, 0.5);
                if (!isFinite(midpoint)) {
                    errors_.push_back(
                        "Non-finite " + std::string(regionName)
                        + " source point in panel "
                        + std::to_string(panelIndex));
                    return;
                }
                sections.push_back(curve);
                // Point correspondence is established by the legacy remap
                // stage. Keeping this canonical parameter on every panel
                // makes adjacent lofts share the same airfoil boundary curve,
                // allowing OCCT to sew them into real shell topology.
                sectionParameters.Append(
                    static_cast<double>(pointIndex - firstPoint));
            }

            const occ::handle<Geom_BSplineSurface> surface =
                skinSections(sections, sectionParameters);
            if (!surface.IsNull()) {
                debugDumpPanel(source,
                               surface,
                               panelIndex,
                               firstPoint,
                               lastPoint,
                               regionName);
            }
            if (surface.IsNull()) {
                errors_.push_back(
                    "Could not loft the source "
                    + std::string(regionName)
                    + " NURBS surface for panel "
                    + std::to_string(panelIndex));
                return;
            }

            validateSourceSurface(
                source,
                surface,
                sectionParameters,
                firstPoint,
                lastPoint,
                panelIndex,
                regionName);
            if (!errors_.empty()) {
                return;
            }

            BRepBuilderAPI_MakeFace makeFace(surface, pointToleranceMillimetres);
            if (!makeFace.IsDone()) {
                errors_.push_back(
                    "Could not create the "
                    + std::string(regionName)
                    + " face for panel "
                    + std::to_string(panelIndex));
                return;
            }

            const TopoDS_Face face = makeFace.Face();
            if (!BRepCheck_Analyzer(face, true).IsValid()) {
                errors_.push_back(
                    "OCCT rejected the "
                    + std::string(regionName)
                    + " face for panel "
                    + std::to_string(panelIndex));
                return;
            }

            const TopoDS_Shape mirroredFace = mirrored(face);
            if (mirroredFace.IsNull()
                || mirroredFace.ShapeType() != TopAbs_FACE) {
                errors_.push_back(
                    "Could not mirror the "
                    + std::string(regionName)
                    + " face for panel "
                    + std::to_string(panelIndex));
                return;
            }

            PanelSurface panel;
            panel.region = region;
            panel.panelIndex = panelIndex;
            panel.firstPoint = firstPoint;
            panel.lastPoint = lastPoint;
            panel.surface = surface;
            panel.rightFace = face;
            panel.leftFace = TopoDS::Face(mirroredFace);
            addIsoparametricSplines(
                panel,
                maximumDisplayedSpanSamples,
                std::min(chordPointCount, maximumDisplayedChordSamples));
            panels_.push_back(std::move(panel));
        } catch (const Standard_Failure &failure) {
            std::ostringstream message;
            message << "OCCT failed to fit the " << regionName
                    << " NURBS surface for panel " << panelIndex;
            if (failure.GetMessageString() != nullptr) {
                message << ": " << failure.GetMessageString();
            }
            errors_.push_back(message.str());
        }
    }

    // Diagnostic: set LEP_DEBUG_PANEL to a comma-separated list of panel
    // indices to dump each region's section curves (span length, shaping
    // height, pole structure, weights) and the lofted surface's rib-side
    // boundary pole row to stderr. This is how the v0.2.2 trailing-edge
    // fin artifact was isolated.
    void debugDumpPanel(const SourcePanel &source,
                        const occ::handle<Geom_BSplineSurface> &surface,
                        int panelIndex,
                        int firstPoint,
                        int lastPoint,
                        const char *regionName) const
    {
        const char *debugPanels = std::getenv("LEP_DEBUG_PANEL");
        if (debugPanels == nullptr) {
            return;
        }
        const std::string padded = "," + std::string(debugPanels) + ",";
        if (padded.find("," + std::to_string(panelIndex) + ",")
            == std::string::npos) {
            return;
        }
        std::ostringstream dump;
        dump << "=== panel " << panelIndex << " region " << regionName
             << " points " << firstPoint << ".." << lastPoint << "\n";
        for (int pointIndex = firstPoint;
             pointIndex <= lastPoint;
             ++pointIndex) {
            const gp_Pnt start =
                sourceControlPoint(source, panelIndex, pointIndex, 47);
            const gp_Pnt end =
                sourceControlPoint(source, panelIndex - 1, pointIndex, 47);
            const double spanLength = start.Distance(end);
            const double height = sourceShapingHeight(source, pointIndex);
            const occ::handle<Geom_BSplineCurve> section =
                makeSourceSpanCurve(source, pointIndex);
            dump << "j=" << pointIndex
                 << " L=" << spanLength
                 << " h=" << height;
            if (!section.IsNull()) {
                dump << " poles=" << section->NbPoles()
                     << " deg=" << section->Degree()
                     << " rational=" << (section->IsRational() ? 1 : 0)
                     << " knots=" << section->NbKnots();
                if (pointIndex - firstPoint < 8) {
                    dump << " w=[";
                    for (int poleIndex = 1;
                         poleIndex <= section->NbPoles();
                         ++poleIndex) {
                        dump << (poleIndex > 1 ? " " : "")
                             << section->Weight(poleIndex);
                    }
                    dump << "]";
                }
            }
            dump << " start=(" << start.X() << ',' << start.Y() << ','
                 << start.Z() << ")\n";
        }
        dump << "surface uPoles=" << surface->NbUPoles()
             << " vPoles=" << surface->NbVPoles()
             << " uDeg=" << surface->UDegree()
             << " vDeg=" << surface->VDegree() << "\n";
        for (int vPole = 1; vPole <= surface->NbVPoles(); ++vPole) {
            const gp_Pnt pole = surface->Pole(1, vPole);
            dump << "  bpole " << vPole << " (" << pole.X() << ','
                 << pole.Y() << ',' << pole.Z() << ") w="
                 << surface->Weight(1, vPole) << "\n";
        }
        std::cerr << dump.str();
    }

    // Samples one skin region on the exact ballooning law at a decimated
    // set of chordwise stations for the Playground simulation mesh. The
    // station selection is deterministic from the region bounds, so
    // neighbouring panels with equal point tables produce bit-identical
    // rib columns and weld in the exporter.
    void captureSimRegion(const SourcePanel &source,
                          int panelIndex,
                          int firstPoint,
                          int lastPoint,
                          int targetRows,
                          Region surface)
    {
        const int stationCount = lastPoint - firstPoint + 1;
        if (stationCount < 2) {
            return;
        }
        const int stride =
            std::max(1, stationCount / std::max(2, targetRows));

        SimRegionCapture capture;
        capture.panelIndex = panelIndex;
        capture.surface = surface;
        try {
            for (int station = firstPoint;
                 station <= lastPoint;
                 station += stride) {
                // Always land exactly on the region boundary so adjacent
                // regions share their junction station.
                const int kept =
                    station + stride > lastPoint ? lastPoint : station;
                std::array<gp_Pnt, simSpanColumnCount> row;
                for (int column = 0; column < simSpanColumnCount; ++column) {
                    const double spanParameter =
                        static_cast<double>(column)
                        / (simSpanColumnCount - 1);
                    row[column] =
                        sourceShapePoint(source, kept, spanParameter);
                    if (!isFinite(row[column])) {
                        return;
                    }
                }
                capture.stations.push_back(kept);
                capture.rows.push_back(row);
                if (kept == lastPoint) {
                    break;
                }
            }
        } catch (const Standard_Failure &) {
            return;
        }
        if (capture.rows.size() >= 2) {
            simRegions_.push_back(std::move(capture));
        }
    }

    void validateSourceSurface(
        const SourcePanel &source,
        const occ::handle<Geom_BSplineSurface> &surface,
        const NCollection_Sequence<double> &sectionParameters,
        int firstPoint,
        int lastPoint,
        int panelIndex,
        const char *regionName)
    {
        double uFirst = 0.0;
        double uLast = 0.0;
        double vFirst = 0.0;
        double vLast = 0.0;
        surface->Bounds(uFirst, uLast, vFirst, vLast);

        double regionSourceDeviation = 0.0;
        constexpr int validationIntervals = 8;
        for (int pointIndex = firstPoint;
             pointIndex <= lastPoint;
             ++pointIndex) {
            const std::size_t curveIndex =
                static_cast<std::size_t>(pointIndex - firstPoint);
            const double vParameter =
                sectionParameters.Value(
                    static_cast<int>(curveIndex) + 1);
            if (vParameter < vFirst - Precision::PConfusion()
                || vParameter > vLast + Precision::PConfusion()) {
                errors_.push_back(
                    "Invalid chord parameter in the "
                    + std::string(regionName)
                    + " source loft for panel "
                    + std::to_string(panelIndex));
                return;
            }

            for (int sample = 0;
                 sample <= validationIntervals;
                 ++sample) {
                const double spanParameter =
                    static_cast<double>(sample)
                    / static_cast<double>(validationIntervals);
                const gp_Pnt expected =
                    sourceShapePoint(
                        source,
                        pointIndex,
                        spanParameter);
                const gp_Pnt actual =
                    surface->Value(
                        uFirst
                            + (uLast - uFirst) * spanParameter,
                        vParameter);
                regionSourceDeviation =
                    std::max(
                        regionSourceDeviation,
                        expected.Distance(actual));
            }
        }
        maximumSourceDeviation_ =
            std::max(
                maximumSourceDeviation_,
                regionSourceDeviation);
        if (regionSourceDeviation
            > maximumSourceDeviationMillimetres) {
            std::ostringstream message;
            message << "The " << regionName
                    << " NURBS loft for panel " << panelIndex
                    << " deviates from its analytical source curves by "
                    << regionSourceDeviation << " mm";
            errors_.push_back(message.str());
            return;
        }

        // The loft is law-constrained only AT the chord stations; between
        // them it is free, which is exactly where an ill-behaved fit can
        // grow artifacts that station sampling cannot see. Bound the
        // mid-station surface against the midpoint of the neighbouring law
        // points: legitimate inter-station behaviour stays within one
        // station spacing plus the local ballooning-height crease (the law
        // may drop its shaping height discontinuously, e.g. at an air
        // intake lip, and a smooth loft must absorb that step locally).
        const auto effectiveHeight = [&](int pointIndex) {
            const double height = sourceShapingHeight(source, pointIndex);
            return height >= 0.01 * millimetresPerCentimetre ? height : 0.0;
        };
        for (int pointIndex = firstPoint;
             pointIndex < lastPoint;
             ++pointIndex) {
            const int curveIndex = pointIndex - firstPoint;
            const double vMid =
                0.5
                * (sectionParameters.Value(curveIndex + 1)
                   + sectionParameters.Value(curveIndex + 2));
            // A crease rings the interpolant over its neighbouring
            // intervals too, so measure the largest height step in a
            // one-interval neighbourhood.
            double heightStep = 0.0;
            for (int neighbour = std::max(firstPoint, pointIndex - 1);
                 neighbour <= std::min(lastPoint - 1, pointIndex + 1);
                 ++neighbour) {
                heightStep =
                    std::max(heightStep,
                             std::abs(effectiveHeight(neighbour)
                                      - effectiveHeight(neighbour + 1)));
            }
            for (int sample = 0; sample <= 4; ++sample) {
                const double spanParameter = sample / 4.0;
                const gp_Pnt nearLaw =
                    sourceShapePoint(source, pointIndex, spanParameter);
                const gp_Pnt farLaw =
                    sourceShapePoint(source, pointIndex + 1, spanParameter);
                const gp_Pnt midpoint(
                    (nearLaw.XYZ() + farLaw.XYZ()) * 0.5);
                const gp_Pnt actual =
                    surface->Value(
                        uFirst + (uLast - uFirst) * spanParameter,
                        vMid);
                const double allowance =
                    nearLaw.Distance(farLaw) + heightStep + 1.0;
                if (actual.Distance(midpoint) > allowance) {
                    std::ostringstream message;
                    message << "The " << regionName
                            << " NURBS loft for panel " << panelIndex
                            << " bulges "
                            << actual.Distance(midpoint)
                            << " mm between chord stations "
                            << pointIndex << " and " << pointIndex + 1;
                    errors_.push_back(message.str());
                    return;
                }
            }
        }

        double legacyDeviation = 0.0;
        for (int pointIndex = firstPoint;
             pointIndex <= lastPoint;
             ++pointIndex) {
            for (int segment = 0;
                 segment <= source.segmentCount;
                 ++segment) {
                const double spanParameter =
                    static_cast<double>(segment)
                    / static_cast<double>(source.segmentCount);
                const gp_Pnt expected =
                    sourceShapePoint(
                        source,
                        pointIndex,
                        spanParameter);
                const gp_Pnt legacy =
                    tessellationPoint(
                        source.legacyTessellation,
                        source.panelIndex,
                        pointIndex,
                        segment + 1);
                legacyDeviation =
                    std::max(
                        legacyDeviation,
                        expected.Distance(legacy));
            }
        }
        maximumLegacyAgreement_ =
            std::max(
                maximumLegacyAgreement_,
                legacyDeviation);
        if (legacyDeviation
            > maximumLegacyAgreementMillimetres) {
            std::ostringstream message;
            message << "The interpreted " << regionName
                    << " source shape for panel " << panelIndex
                    << " differs from the legacy validation grid by "
                    << legacyDeviation << " mm";
            errors_.push_back(message.str());
        }
    }

    void addIsoparametricSplines(
        PanelSurface &panel,
        int spanPointCount,
        int chordPointCount)
    {
        double uFirst = 0.0;
        double uLast = 0.0;
        double vFirst = 0.0;
        double vLast = 0.0;
        panel.surface->Bounds(uFirst, uLast, vFirst, vLast);

        auto addCurve = [&panel](const occ::handle<Geom_Curve> &curve) {
            if (curve.IsNull()) {
                return;
            }
            BRepBuilderAPI_MakeEdge makeEdge(curve);
            if (makeEdge.IsDone()) {
                const TopoDS_Edge edge = makeEdge.Edge();
                panel.rightWireframe.push_back(edge);
                const TopoDS_Shape mirroredEdge = mirrored(edge);
                if (!mirroredEdge.IsNull()
                    && mirroredEdge.ShapeType() == TopAbs_EDGE) {
                    panel.leftWireframe.push_back(
                        TopoDS::Edge(mirroredEdge));
                }
            }
        };

        // Index 0 and the final index coincide with the face boundaries.
        for (int index = 1; index + 1 < spanPointCount; ++index) {
            const double parameter =
                uFirst
                + (uLast - uFirst)
                      * static_cast<double>(index)
                      / static_cast<double>(spanPointCount - 1);
            addCurve(panel.surface->UIso(parameter));
        }
        for (int index = 1; index + 1 < chordPointCount; ++index) {
            const double parameter =
                vFirst
                + (vLast - vFirst)
                      * static_cast<double>(index)
                      / static_cast<double>(chordPointCount - 1);
            addCurve(panel.surface->VIso(parameter));
        }
    }

    static occ::handle<Geom_BSplineCurve> makeLinearSpline(
        const gp_Pnt &start,
        const gp_Pnt &end)
    {
        NCollection_Array1<gp_Pnt> poles(1, 2);
        poles.SetValue(1, start);
        poles.SetValue(2, end);

        NCollection_Array1<double> knots(1, 2);
        knots.SetValue(1, 0.0);
        knots.SetValue(2, 1.0);

        NCollection_Array1<int> multiplicities(1, 2);
        multiplicities.SetValue(1, 2);
        multiplicities.SetValue(2, 2);

        return new Geom_BSplineCurve(
            poles,
            knots,
            multiplicities,
            1,
            false);
    }

    // Sewing remains the topology-quality gate for the lofted skins even
    // though the export is now a structured assembly of individual panels:
    // adjacent panels must still meet on shared boundary curves.
    void validateSewing(lep::NurbsWriteResult &result) const
    {
        BRepBuilderAPI_Sewing sewing(
            pointToleranceMillimetres,
            true,
            true,
            true,
            false);
        for (const PanelSurface &panel : panels_) {
            sewing.Add(panel.rightFace);
            sewing.Add(panel.leftFace);
        }
        sewing.Perform();
        const TopoDS_Shape skins = sewing.SewedShape();
        if (skins.IsNull()
            || !BRepCheck_Analyzer(skins, true).IsValid()) {
            result.error =
                "OCCT could not sew the NURBS faces into valid skin topology.";
            return;
        }
        result.sewnEdgeCount = sewing.NbContigousEdges();
        result.freeEdgeCount = sewing.NbFreeEdges();
    }

    static TopoDS_Compound makeCompound(
        const std::vector<TopoDS_Shape> &shapes)
    {
        BRep_Builder builder;
        TopoDS_Compound compound;
        builder.MakeCompound(compound);
        for (const TopoDS_Shape &shape : shapes) {
            builder.Add(compound, shape);
        }
        return compound;
    }

    void addPanelParts(AssemblyGroup &wing,
                       lep::NurbsWriteResult &result,
                       bool includeConstructionCurves) const
    {
        for (const PanelSurface &panel : panels_) {
            AssemblyGroup &regionGroup =
                wing.group(regionGroupName(panel.region));
            const std::string partName =
                "Panel " + std::to_string(panel.panelIndex);
            const PartColor faceColor = regionColor(panel.region);

            regionGroup.group(rightSideName).parts.push_back(
                {partName,
                 panel.rightFace,
                 faceColor,
                 wireframeColor,
                 true});
            regionGroup.group(leftSideName).parts.push_back(
                {partName,
                 panel.leftFace,
                 faceColor,
                 wireframeColor,
                 true});
            result.surfaceCount += 2;

            if (!includeConstructionCurves) {
                continue;
            }
            AssemblyGroup &curvesGroup =
                wing.group(regionCurvesGroupName(panel.region));
            const auto addWireframePart =
                [&](const char *sideName,
                    const std::vector<TopoDS_Edge> &wireframe) {
                    if (wireframe.empty()) {
                        return;
                    }
                    const std::vector<TopoDS_Shape> shapes(
                        wireframe.begin(), wireframe.end());
                    curvesGroup.group(sideName).parts.push_back(
                        {partName,
                         makeCompound(shapes),
                         wireframeColor,
                         wireframeColor,
                         false});
                };
            addWireframePart(rightSideName, panel.rightWireframe);
            addWireframePart(leftSideName, panel.leftWireframe);
            result.splineCount += static_cast<int>(
                panel.rightWireframe.size() + panel.leftWireframe.size());
        }
    }

    // The exact chordwise rib contour over [firstPoint, lastPoint], as one
    // degree-1 B-spline through the captured station points. The legacy
    // shape definition is chordwise piecewise linear (the cut patterns and
    // the 3D reference output are polylines through these points), and a
    // smoothing interpolation can overshoot into the opposite side of a
    // thin profile, so the polyline is both the faithful and the robust
    // boundary.
    TopoDS_Edge contourEdge(const CapturedRib &rib,
                            int firstPoint,
                            int lastPoint) const
    {
        const int count = lastPoint - firstPoint + 1;
        NCollection_Array1<gp_Pnt> poles(1, count);
        NCollection_Array1<double> knots(1, count);
        NCollection_Array1<int> multiplicities(1, count);
        for (int index = 0; index < count; ++index) {
            poles.SetValue(
                index + 1,
                rib.spatialPoints[
                    static_cast<std::size_t>(firstPoint - 1 + index)]);
            multiplicities.SetValue(index + 1, 1);
        }
        multiplicities.SetValue(1, 2);
        multiplicities.SetValue(count, 2);
        knots.SetValue(1, 0.0);
        for (int index = 2; index <= count; ++index) {
            knots.SetValue(
                index,
                knots.Value(index - 1)
                    + std::max(
                        poles.Value(index - 1)
                            .Distance(poles.Value(index)),
                        Precision::Confusion()));
        }

        BRepBuilderAPI_MakeEdge makeEdge(
            occ::handle<Geom_BSplineCurve>(
                new Geom_BSplineCurve(
                    poles,
                    knots,
                    multiplicities,
                    1,
                    false)));
        if (!makeEdge.IsDone()) {
            return {};
        }
        return makeEdge.Edge();
    }

    // Builds one closed rounded-polygon hole wire (types 3 and 4) from the
    // legacy corner construction: circle centres sit on the corner
    // bisectors, so the straight sides are the common tangents between
    // consecutive corner circles.
    TopoDS_Wire roundedPolygonHoleWire(const std::vector<gp_XY> &corners,
                                       const std::vector<gp_XY> &centers,
                                       double radius,
                                       const RibFrame &frame,
                                       int &edgeCount) const
    {
        const std::size_t count = corners.size();
        BRepBuilderAPI_MakeWire makeWire;
        int added = 0;
        const auto addEdge = [&makeWire, &added](const TopoDS_Edge &edge) {
            if (edge.IsNull()) {
                return false;
            }
            makeWire.Add(edge);
            ++added;
            return makeWire.IsDone() == Standard_True;
        };
        const auto lineEdge = [](const gp_Pnt &from,
                                 const gp_Pnt &to) -> TopoDS_Edge {
            BRepBuilderAPI_MakeEdge makeEdge(makeLinearSpline(from, to));
            if (!makeEdge.IsDone()) {
                return {};
            }
            return makeEdge.Edge();
        };

        if (radius <= Precision::Confusion()) {
            for (std::size_t index = 0; index < count; ++index) {
                const gp_Pnt from = frame.point(corners[index]);
                const gp_Pnt to =
                    frame.point(corners[(index + 1) % count]);
                if (from.Distance(to) <= Precision::Confusion()
                    || !addEdge(lineEdge(from, to))) {
                    return {};
                }
            }
        } else {
            std::vector<gp_XY> tangentIn(count);
            std::vector<gp_XY> tangentOut(count);
            for (std::size_t index = 0; index < count; ++index) {
                const gp_XY &corner = corners[index];
                tangentIn[index] = projectOntoLine(
                    corners[(index + count - 1) % count],
                    corner,
                    centers[index]);
                tangentOut[index] = projectOntoLine(
                    corner,
                    corners[(index + 1) % count],
                    centers[index]);
                if (std::abs(
                        (tangentIn[index] - centers[index]).Modulus()
                        - radius) > pointToleranceMillimetres
                    || std::abs(
                        (tangentOut[index] - centers[index]).Modulus()
                        - radius) > pointToleranceMillimetres) {
                    return {};
                }
            }
            for (std::size_t index = 0; index < count; ++index) {
                if ((tangentOut[index] - tangentIn[index]).Modulus()
                    > Precision::Confusion()) {
                    const gp_XY bisector =
                        corners[index] - centers[index];
                    const double bisectorLength = bisector.Modulus();
                    if (bisectorLength <= Precision::Confusion()) {
                        return {};
                    }
                    const gp_XY arcMid =
                        centers[index]
                        + bisector * (radius / bisectorLength);
                    GC_MakeArcOfCircle makeArc(
                        frame.point(tangentIn[index]),
                        frame.point(arcMid),
                        frame.point(tangentOut[index]));
                    if (!makeArc.IsDone()) {
                        return {};
                    }
                    BRepBuilderAPI_MakeEdge makeEdge(makeArc.Value());
                    if (!makeEdge.IsDone()
                        || !addEdge(makeEdge.Edge())) {
                        return {};
                    }
                }
                const gp_Pnt from = frame.point(tangentOut[index]);
                const gp_Pnt to =
                    frame.point(tangentIn[(index + 1) % count]);
                if (from.Distance(to) > Precision::Confusion()
                    && !addEdge(lineEdge(from, to))) {
                    return {};
                }
            }
        }
        if (added < 2 || !makeWire.IsDone()) {
            return {};
        }
        const TopoDS_Wire wire = makeWire.Wire();
        if (!wire.Closed()) {
            return {};
        }
        edgeCount = added;
        return wire;
    }

    // Builds the closed wire of one airfoil hole in the rib plane. The
    // shapes reproduce the legacy 2D construction exactly, but as ellipse,
    // line and arc geometry instead of the drawing polylines.
    TopoDS_Wire ribHoleWire(const RibHole &hole,
                            const RibFrame &frame,
                            int ribIndex,
                            int holeNumber,
                            int &edgeCount,
                            std::vector<std::string> &warnings) const
    {
        // A hole that cannot be built is left uncut with a warning; the
        // legacy core also draws inconsistent hole definitions without
        // complaint.
        const auto fail =
            [&warnings, ribIndex, holeNumber](const std::string &reason) {
                warnings.push_back(
                    "Left hole " + std::to_string(holeNumber)
                    + " of rib " + std::to_string(ribIndex)
                    + " uncut: " + reason);
                return TopoDS_Wire();
            };

        if (hole.type == 1) {
            const double semiA = std::abs(hole.a);
            const double semiB = std::abs(hole.b);
            if (semiA <= Precision::Confusion()
                || semiB <= Precision::Confusion()) {
                return fail("Degenerate ellipse axes");
            }
            // The legacy core rotates ellipses in the y-flipped drawing
            // sheet, taking the file value directly as radians; in the rib
            // frame the axis directions therefore come out as below.
            const double rotation = hole.rotation;
            gp_Vec direction1 = frame.direction(
                {std::cos(rotation), -std::sin(rotation)});
            gp_Vec direction2 = frame.direction(
                {-std::sin(rotation), -std::cos(rotation)});
            double major = semiA;
            double minor = semiB;
            if (major < minor) {
                std::swap(major, minor);
                std::swap(direction1, direction2);
            }
            const gp_Ax2 axes(
                frame.point({hole.x, hole.y}),
                gp_Dir(direction1.Crossed(direction2)),
                gp_Dir(direction1));
            BRepBuilderAPI_MakeEdge makeEdge(gp_Elips(axes, major, minor));
            if (!makeEdge.IsDone()) {
                return fail("Could not build the ellipse");
            }
            BRepBuilderAPI_MakeWire makeWire(makeEdge.Edge());
            if (!makeWire.IsDone()) {
                return fail("Could not close the ellipse");
            }
            edgeCount = 1;
            return makeWire.Wire();
        }

        // Types 3 (rounded triangle) and 4 (rounded rectangle) rotate in
        // degrees and mirror through the sign of the first side length.
        const double alpha =
            hole.rotation * std::numbers::pi / 180.0;
        if (std::abs(hole.a) <= Precision::Confusion()
            || hole.b <= Precision::Confusion()
            || hole.cornerRadius < 0.0) {
            return fail("Degenerate hole sides");
        }
        const double sideSign = hole.a >= 0.0 ? 1.0 : -1.0;
        const double sideA = std::abs(hole.a);
        const double sideB = hole.b;
        const double radius = hole.cornerRadius;

        std::vector<gp_XY> corners;
        std::vector<gp_XY> centers;
        const gp_XY corner1(hole.x, hole.y);
        if (hole.type == 3) {
            const gp_XY corner2(
                hole.x + sideSign * sideA * std::cos(alpha),
                hole.y + sideA * std::sin(alpha));
            const gp_XY corner3(hole.x, hole.y + sideB);
            const double gammaC = 0.5 * std::numbers::pi - alpha;
            const double sideC = std::sqrt(std::max(
                0.0,
                sideA * sideA + sideB * sideB
                    - 2.0 * sideA * sideB * std::cos(gammaC)));
            if (sideC <= Precision::Confusion()) {
                return fail("Degenerate triangle");
            }
            const double angleA = std::acos(std::clamp(
                (sideC * sideC + sideB * sideB - sideA * sideA)
                    / (2.0 * sideB * sideC),
                -1.0,
                1.0));
            const double angleB = std::acos(std::clamp(
                (sideC * sideC + sideA * sideA - sideB * sideB)
                    / (2.0 * sideA * sideC),
                -1.0,
                1.0));
            double height1 = 0.0;
            double height2 = 0.0;
            double height3 = 0.0;
            if (radius > Precision::Confusion()) {
                const double sin1 = std::sin(0.5 * gammaC);
                const double sin2 = std::sin(0.5 * angleB);
                const double sin3 = std::sin(0.5 * angleA);
                if (sin1 <= Precision::Confusion()
                    || sin2 <= Precision::Confusion()
                    || sin3 <= Precision::Confusion()) {
                    return fail("Invalid corner rounding");
                }
                height1 = radius / sin1;
                height2 = radius / sin2;
                height3 = radius / sin3;
            }
            corners = {corner1, corner2, corner3};
            centers = {
                {hole.x
                     + sideSign * height1 * std::cos(alpha + 0.5 * gammaC),
                 hole.y + height1 * std::sin(alpha + 0.5 * gammaC)},
                {corner2.X()
                     - sideSign * height2 * std::cos(-alpha + 0.5 * angleB),
                 corner2.Y() + height2 * std::sin(-alpha + 0.5 * angleB)},
                {corner3.X() + sideSign * height3 * std::sin(0.5 * angleA),
                 corner3.Y() - height3 * std::cos(0.5 * angleA)},
            };
        } else {
            const gp_XY corner2(
                hole.x + sideSign * sideA * std::cos(alpha),
                hole.y + sideA * std::sin(alpha));
            const gp_XY corner3(
                corner2.X() - sideSign * sideB * std::sin(alpha),
                corner2.Y() + sideB * std::cos(alpha));
            const gp_XY corner4(
                hole.x - sideSign * sideB * std::sin(alpha),
                hole.y + sideB * std::cos(alpha));
            const double radiusCos = radius * std::cos(alpha);
            const double radiusSin = radius * std::sin(alpha);
            corners = {corner1, corner2, corner3, corner4};
            centers = {
                {corner1.X() + sideSign * (radiusCos - radiusSin),
                 corner1.Y() + radiusSin + radiusCos},
                {corner2.X() - sideSign * (radiusCos + radiusSin),
                 corner2.Y() - radiusSin + radiusCos},
                {corner3.X() - sideSign * (radiusCos - radiusSin),
                 corner3.Y() - radiusSin - radiusCos},
                {corner4.X() + sideSign * (radiusCos + radiusSin),
                 corner4.Y() + radiusSin - radiusCos},
            };
        }

        const TopoDS_Wire wire = roundedPolygonHoleWire(
            corners, centers, radius, frame, edgeCount);
        if (wire.IsNull()) {
            return fail("Could not build the outline");
        }
        return wire;
    }

    // Builds the closed planar face of one rib. The outer boundary reuses
    // the exact panel boundary edges wherever a panel exists, interpolates
    // the captured contour where none does, and closes the trailing edge
    // with a straight seam. Airfoil holes become inner wires.
    // Builds one rib face; runs on worker threads, so every warning and
    // error goes to the caller's sinks instead of the shared members.
    TopoDS_Face makeRibFace(int ribIndex,
                            std::vector<RibBoundarySegment> segments,
                            const CapturedRib &rib,
                            int &edgeCount,
                            TopoDS_Shape &curveFallback,
                            std::vector<std::string> &warnings,
                            std::vector<std::string> &errors) const
    {
        const auto fail = [&errors, ribIndex](const std::string &reason) {
            errors.push_back(
                reason + " for rib " + std::to_string(ribIndex));
            return TopoDS_Face();
        };

        const int totalPointCount =
            static_cast<int>(rib.spatialPoints.size());
        std::sort(segments.begin(),
                  segments.end(),
                  [](const RibBoundarySegment &left,
                     const RibBoundarySegment &right) {
                      return left.firstPoint < right.firstPoint;
                  });

        // The outline is interpolated through the exact rib station points
        // rather than reusing the panel boundary edges: the lofts are only
        // validated at the stations and their boundary curves oscillate
        // measurably out of the rib plane between them. The outline is
        // still split at the panel region boundaries (vent corners), which
        // are genuine feature points of the rib.
        std::vector<std::pair<int, int>> ranges;
        int cursor = 1;
        for (const RibBoundarySegment &segment : segments) {
            if (segment.firstPoint < cursor
                || segment.firstPoint >= segment.lastPoint
                || segment.lastPoint > totalPointCount) {
                return fail("Inconsistent panel boundary ranges");
            }
            if (segment.firstPoint > cursor) {
                ranges.emplace_back(cursor, segment.firstPoint);
            }
            ranges.emplace_back(segment.firstPoint, segment.lastPoint);
            cursor = segment.lastPoint;
        }
        if (cursor < totalPointCount) {
            ranges.emplace_back(cursor, totalPointCount);
        }
        if (ranges.empty()) {
            return fail("No outline curves");
        }

        struct BoundaryEdge
        {
            TopoDS_Edge edge;
            gp_Pnt start;
            gp_Pnt end;
        };
        std::vector<BoundaryEdge> boundary;
        for (const auto &[firstPoint, lastPoint] : ranges) {
            const TopoDS_Edge edge =
                contourEdge(rib, firstPoint, lastPoint);
            if (edge.IsNull()) {
                return fail("Could not build the outline");
            }
            boundary.push_back(
                {edge,
                 rib.spatialPoints[
                     static_cast<std::size_t>(firstPoint - 1)],
                 rib.spatialPoints[
                     static_cast<std::size_t>(lastPoint - 1)]});
        }
        // Straight trailing-edge seam whenever the airfoil is open there.
        if (boundary.back().end.Distance(boundary.front().start)
            > Precision::Confusion()) {
            BRepBuilderAPI_MakeEdge closing(
                makeLinearSpline(
                    boundary.back().end,
                    boundary.front().start));
            if (!closing.IsDone()) {
                return fail("Could not close the trailing edge");
            }
            boundary.push_back(
                {closing.Edge(),
                 boundary.back().end,
                 boundary.front().start});
        }

        BRepBuilderAPI_MakeWire makeWire;
        for (const BoundaryEdge &edge : boundary) {
            makeWire.Add(edge.edge);
            if (!makeWire.IsDone()) {
                return fail("Could not chain the outline");
            }
        }
        TopoDS_Wire outerWire = makeWire.Wire();
        if (!outerWire.Closed()) {
            return fail("The outline is not closed");
        }

        // The rib plane, with a Newell normal so the winding test below is
        // well conditioned, and validated against every contour point.
        gp_XYZ centroid(0.0, 0.0, 0.0);
        for (const gp_Pnt &point : rib.spatialPoints) {
            centroid += point.XYZ();
        }
        centroid /= static_cast<double>(totalPointCount);
        gp_XYZ normalAccumulator(0.0, 0.0, 0.0);
        for (int index = 0; index < totalPointCount; ++index) {
            const gp_XYZ current =
                rib.spatialPoints[
                    static_cast<std::size_t>(index)].XYZ() - centroid;
            const gp_XYZ next =
                rib.spatialPoints[
                    static_cast<std::size_t>(
                        (index + 1) % totalPointCount)].XYZ() - centroid;
            normalAccumulator += current.Crossed(next);
        }
        double contourExtent = 0.0;
        for (const gp_Pnt &point : rib.spatialPoints) {
            contourExtent = std::max(
                contourExtent,
                (point.XYZ() - centroid).Modulus());
        }
        // A collapsed rib encloses no area (the wingtip typically closes
        // the wing with a zero-thickness profile whose lower side retraces
        // the upper one). No face exists there; export the outline curves.
        if (normalAccumulator.Modulus()
            <= 1.0e-6 * contourExtent * contourExtent) {
            std::vector<TopoDS_Shape> outline;
            outline.reserve(boundary.size());
            for (const BoundaryEdge &edge : boundary) {
                outline.push_back(edge.edge);
            }
            curveFallback = makeCompound(outline);
            edgeCount = static_cast<int>(boundary.size());
            return {};
        }
        gp_Dir normal(normalAccumulator);
        for (const gp_Pnt &point : rib.spatialPoints) {
            if (std::abs((point.XYZ() - centroid).Dot(normal.XYZ()))
                > pointToleranceMillimetres) {
                return fail("The contour is not planar");
            }
        }

        const gp_Pnt planeOrigin(centroid);
        const double outerWinding =
            signedAreaAlongNormal(outerWire, planeOrigin, normal);
        if (std::abs(outerWinding) <= Precision::SquareConfusion()) {
            return fail("Could not orient the outline");
        }
        if (outerWinding < 0.0) {
            normal.Reverse();
        }

        BRepBuilderAPI_MakeFace makeFace(
            gp_Pln(planeOrigin, normal), outerWire);
        if (!makeFace.IsDone()) {
            return fail("Could not build the face");
        }
        TopoDS_Face face = makeFace.Face();
        edgeCount = static_cast<int>(boundary.size());

        // Every face curve is built from in-plane geometry, so any residual
        // out-of-plane deviation is numerical noise. Measure it, reject
        // anything conspicuous, and record the measured bound as the edge
        // tolerance where it exceeds the model precision.
        constexpr double maximumPlaneDeviationMillimetres = 0.05;
        BRep_Builder toleranceUpdater;
        for (TopExp_Explorer explorer(face, TopAbs_EDGE);
             explorer.More();
             explorer.Next()) {
            const TopoDS_Edge &edge = TopoDS::Edge(explorer.Current());
            double first = 0.0;
            double last = 0.0;
            const occ::handle<Geom_Curve> curve =
                BRep_Tool::Curve(edge, first, last);
            if (curve.IsNull()) {
                return fail("Could not evaluate an outline curve");
            }
            double deviation = 0.0;
            constexpr int sampleCount = 64;
            for (int sample = 0; sample <= sampleCount; ++sample) {
                const double parameter =
                    first
                    + (last - first) * static_cast<double>(sample)
                          / static_cast<double>(sampleCount);
                deviation = std::max(
                    deviation,
                    std::abs(
                        (curve->Value(parameter).XYZ() - centroid)
                            .Dot(normal.XYZ())));
            }
            if (deviation > maximumPlaneDeviationMillimetres) {
                std::ostringstream message;
                message << "A face curve leaves the rib plane by "
                        << deviation << " mm";
                return fail(message.str());
            }
            if (deviation > Precision::Confusion()) {
                toleranceUpdater.UpdateEdge(edge, deviation * 1.25);
            }
        }

        const auto invalidFaceDetail = [](const BRepCheck_Analyzer &analyzer,
                                          const TopoDS_Face &checked) {
            std::ostringstream detail;
            detail << '(';
            const auto report = [&](const TopoDS_Shape &shape,
                                    const char *kind) {
                const occ::handle<BRepCheck_Result> checkResult =
                    analyzer.Result(shape);
                if (checkResult.IsNull()) {
                    return;
                }
                for (const BRepCheck_Status status :
                     checkResult->Status()) {
                    if (status != BRepCheck_NoError) {
                        detail << ' ' << kind
                               << static_cast<int>(status);
                    }
                }
                if (!checkResult->IsStatusOnShape(checked)) {
                    return;
                }
                for (const BRepCheck_Status status :
                     checkResult->StatusOnShape(checked)) {
                    if (status != BRepCheck_NoError) {
                        detail << ' ' << kind << "ctx"
                               << static_cast<int>(status);
                    }
                }
            };
            report(checked, "face:");
            for (TopExp_Explorer explorer(checked, TopAbs_WIRE);
                 explorer.More();
                 explorer.Next()) {
                report(explorer.Current(), "wire:");
            }
            for (TopExp_Explorer explorer(checked, TopAbs_EDGE);
                 explorer.More();
                 explorer.Next()) {
                report(explorer.Current(), "edge:");
            }
            detail << " )";
            return detail.str();
        };

        const auto outlineFallback = [&](const BRepCheck_Analyzer &analyzer) {
            // Very thin profiles (single-skin ribs) can produce an
            // outline OCCT cannot classify as a face; keep the exact
            // outline curves instead of failing the whole model.
            warnings.push_back(
                "Exported rib " + std::to_string(ribIndex)
                + " as outline curves: OCCT rejected its face "
                + invalidFaceDetail(analyzer, face));
            std::vector<TopoDS_Shape> outline;
            outline.reserve(boundary.size());
            for (const BoundaryEdge &edge : boundary) {
                outline.push_back(edge.edge);
            }
            curveFallback = makeCompound(outline);
            edgeCount = static_cast<int>(boundary.size());
        };

        // Prepare every hole wire up front. Their warnings are committed
        // only when the rib keeps its face: a rib that falls back to
        // outline curves never reported hole problems.
        struct PreparedHole
        {
            std::size_t number = 0;
            TopoDS_Wire wire;
            int edgeCount = 0;
        };
        std::vector<PreparedHole> preparedHoles;
        std::vector<std::string> holeWarnings;
        if (!rib.holes.empty()) {
            RibFrame frame;
            if (!fitRibFrame(rib.planarPoints, rib.spatialPoints, frame)) {
                holeWarnings.push_back(
                    "Left the holes of rib " + std::to_string(ribIndex)
                    + " uncut: could not recover its rigid planar frame");
            } else {
                for (std::size_t holeIndex = 0;
                     holeIndex < rib.holes.size();
                     ++holeIndex) {
                    int holeEdgeCount = 0;
                    TopoDS_Wire holeWire = ribHoleWire(
                        rib.holes[holeIndex],
                        frame,
                        ribIndex,
                        static_cast<int>(holeIndex) + 1,
                        holeEdgeCount,
                        holeWarnings);
                    if (holeWire.IsNull()) {
                        // ribHoleWire already recorded why.
                        continue;
                    }
                    // Holes must wind against the outer boundary.
                    if (signedAreaAlongNormal(holeWire, planeOrigin, normal)
                        > 0.0) {
                        holeWire.Reverse();
                    }
                    preparedHoles.push_back(
                        {holeIndex + 1, holeWire, holeEdgeCount});
                }
            }
        }

        // Cut every hole, then validate the finished face once: the
        // analyzer dominates the whole model build, so the per-hole
        // diagnosis below runs only when this combined face fails.
        TopoDS_Face holedFace = face;
        int holedEdgeCount = 0;
        bool allHolesCut = true;
        for (const PreparedHole &hole : preparedHoles) {
            BRepBuilderAPI_MakeFace cut(holedFace, hole.wire);
            if (!cut.IsDone()) {
                allHolesCut = false;
                break;
            }
            holedFace = cut.Face();
            holedEdgeCount += hole.edgeCount;
        }
        if (allHolesCut) {
            const BRepCheck_Analyzer analyzer(holedFace, true);
            if (analyzer.IsValid()) {
                warnings.insert(warnings.end(),
                                holeWarnings.begin(),
                                holeWarnings.end());
                edgeCount += holedEdgeCount;
                return holedFace;
            }
        }

        // Something in the combined face is invalid. Check the bare face
        // first (it decides between the outline fallback and hole trouble),
        // then re-cut hole by hole to skip exactly the offenders.
        {
            const BRepCheck_Analyzer analyzer(face, true);
            if (!analyzer.IsValid()) {
                outlineFallback(analyzer);
                return {};
            }
        }
        warnings.insert(warnings.end(),
                        holeWarnings.begin(),
                        holeWarnings.end());
        for (const PreparedHole &hole : preparedHoles) {
            BRepBuilderAPI_MakeFace cut(face, hole.wire);
            const BRepCheck_Analyzer holedCheck(
                cut.IsDone() ? cut.Face() : face, true);
            if (!cut.IsDone() || !holedCheck.IsValid()) {
                // The legacy core draws such holes over the outline in
                // the 2D patterns; in the solid model they cannot be
                // cut, so leave them out and say so.
                warnings.push_back(
                    "Left hole " + std::to_string(hole.number)
                    + " of rib " + std::to_string(ribIndex)
                    + " uncut: it does not fit inside the rib outline");
                continue;
            }
            face = cut.Face();
            edgeCount += hole.edgeCount;
        }
        return face;
    }

    // Every captured panel spans rib i to rib i-1, so panel i declares the
    // panel-covered chord ranges of rib i, and of rib i-1 only when panel
    // i-1 was not built (which also yields the centre rib 0 from panel 1).
    // Each rib station becomes a closed planar face over its full captured
    // contour, with its airfoil holes cut out.
    void addRibParts(AssemblyGroup &wing,
                     lep::NurbsWriteResult &result) const
    {
        std::unordered_set<int> capturedPanels;
        for (const PanelSurface &panel : panels_) {
            capturedPanels.insert(panel.panelIndex);
        }

        std::map<int, std::vector<RibBoundarySegment>> ribSegments;
        for (const PanelSurface &panel : panels_) {
            ribSegments[panel.panelIndex].push_back(
                {panel.firstPoint, panel.lastPoint});
            if (!capturedPanels.contains(panel.panelIndex - 1)) {
                ribSegments[panel.panelIndex - 1].push_back(
                    {panel.firstPoint, panel.lastPoint});
            }
        }

        struct RibJob
        {
            int ribIndex = 0;
            const std::vector<RibBoundarySegment> *segments = nullptr;
            const CapturedRib *rib = nullptr;
        };
        std::vector<RibJob> jobs;
        jobs.reserve(ribSegments.size());
        for (const auto &[ribIndex, segments] : ribSegments) {
            const auto captured = capturedRibs_.find(ribIndex);
            if (captured == capturedRibs_.end()) {
                errors_.push_back(
                    "Missing captured rib station for rib "
                    + std::to_string(ribIndex));
                return;
            }
            jobs.push_back({ribIndex, &segments, &captured->second});
        }

        struct RibBuild
        {
            TopoDS_Shape shape;
            TopoDS_Shape mirroredShape;
            bool hasFace = false;
            bool onCenter = false;
            int edgeCount = 0;
            std::vector<std::string> warnings;
            std::vector<std::string> errors;
        };
        std::vector<RibBuild> builds(jobs.size());

        // Every rib face is geometrically independent, so they build on
        // OCCT's thread pool; the merge below runs in rib order to keep
        // the assembly, warnings, and errors deterministic.
        OSD_Parallel::For(
            0,
            static_cast<int>(jobs.size()),
            [this, &jobs, &builds](int jobIndex) {
                const RibJob &job =
                    jobs[static_cast<std::size_t>(jobIndex)];
                RibBuild &build =
                    builds[static_cast<std::size_t>(jobIndex)];
                try {
                    TopoDS_Shape outlineCurves;
                    const TopoDS_Face face = makeRibFace(
                        job.ribIndex,
                        *job.segments,
                        *job.rib,
                        build.edgeCount,
                        outlineCurves,
                        build.warnings,
                        build.errors);
                    if (!build.errors.empty()) {
                        return;
                    }
                    build.hasFace = !face.IsNull();
                    build.shape = build.hasFace ? TopoDS_Shape(face)
                                                : outlineCurves;
                    if (build.shape.IsNull()) {
                        build.errors.push_back(
                            "Could not build the shape of rib "
                            + std::to_string(job.ribIndex));
                        return;
                    }
                    build.onCenter = std::all_of(
                        job.rib->spatialPoints.begin(),
                        job.rib->spatialPoints.end(),
                        [](const gp_Pnt &point) {
                            return std::abs(point.X())
                                   <= symmetryPlaneToleranceMillimetres;
                        });
                    if (!build.onCenter) {
                        build.mirroredShape = mirrored(build.shape);
                        if (build.mirroredShape.IsNull()) {
                            build.errors.push_back(
                                "Could not mirror the shape of rib "
                                + std::to_string(job.ribIndex));
                        }
                    }
                } catch (const Standard_Failure &failure) {
                    build.errors.push_back(
                        "OCCT failed building rib "
                        + std::to_string(job.ribIndex) + ": "
                        + (failure.GetMessageString() != nullptr
                               ? failure.GetMessageString()
                               : "unknown OCCT error"));
                } catch (const std::exception &exception) {
                    build.errors.push_back(
                        "Failed building rib "
                        + std::to_string(job.ribIndex) + ": "
                        + exception.what());
                }
            });

        AssemblyGroup &ribs = wing.group(ribsGroupName);
        for (std::size_t index = 0; index < jobs.size(); ++index) {
            RibBuild &build = builds[index];
            warnings_.insert(warnings_.end(),
                             build.warnings.begin(),
                             build.warnings.end());
            if (!build.errors.empty()) {
                errors_.insert(errors_.end(),
                               build.errors.begin(),
                               build.errors.end());
                return;
            }

            const std::string partName =
                "Rib " + std::to_string(jobs[index].ribIndex);
            if (build.onCenter) {
                ribs.group(centerSideName).parts.push_back(
                    {partName, build.shape, ribColor, ribColor,
                     build.hasFace});
                result.surfaceCount += build.hasFace ? 1 : 0;
                result.splineCount += build.edgeCount;
            } else {
                ribs.group(rightSideName).parts.push_back(
                    {partName, build.shape, ribColor, ribColor,
                     build.hasFace});
                ribs.group(leftSideName).parts.push_back(
                    {partName, build.mirroredShape, ribColor, ribColor,
                     build.hasFace});
                result.surfaceCount += build.hasFace ? 2 : 0;
                result.splineCount += build.edgeCount * 2;
            }
            ++result.ribCount;
        }
    }

    void addLineParts(AssemblyGroup &wing,
                      lep::NurbsWriteResult &result) const
    {
        if (capturedLines_.empty()) {
            return;
        }

        AssemblyGroup &lines = wing.group(linesGroupName);
        std::unordered_set<QuantizedSegment, QuantizedSegmentHash> added;

        struct LabelPart
        {
            AssemblyGroup *group = nullptr;
            std::string label;
            PartColor color;
            std::vector<TopoDS_Shape> segments;
        };
        std::vector<LabelPart> labelParts;

        const auto labelPartFor = [&](const CapturedLine &line) -> LabelPart & {
            AssemblyGroup *group = nullptr;
            PartColor color = otherCurveColor;
            std::string label = line.label;
            if (line.brake) {
                group = &lines.group(brakeGroupName);
                color = brakeColor;
            } else if (line.planIndex >= 1 && line.planIndex <= 6) {
                group = &lines.group(
                    std::string("Plan ")
                    + static_cast<char>('A' + line.planIndex - 1));
                color = planColors[line.planIndex - 1];
            } else {
                group = &lines.group(otherCurvesName);
            }
            if (label.empty()) {
                label = "Curve";
            }
            for (LabelPart &part : labelParts) {
                if (part.group == group && part.label == label) {
                    return part;
                }
            }
            labelParts.push_back({group, label, color, {}});
            return labelParts.back();
        };

        for (const CapturedLine &line : capturedLines_) {
            const QuantizedSegment key =
                quantizeSegment(line.start, line.end);
            if (!added.insert(key).second) {
                continue;
            }
            BRepBuilderAPI_MakeEdge makeEdge(
                makeLinearSpline(line.start, line.end));
            if (!makeEdge.IsDone()) {
                continue;
            }
            labelPartFor(line).segments.push_back(makeEdge.Edge());
        }

        for (const LabelPart &part : labelParts) {
            if (part.segments.empty()) {
                continue;
            }
            part.group->parts.push_back(
                {part.label,
                 makeCompound(part.segments),
                 part.color,
                 part.color,
                 false});
            ++result.lineCount;
            result.splineCount += static_cast<int>(part.segments.size());
        }
    }

    // Interpolates one strip boundary through its samples at the given
    // parameters. Identical parameters across sibling boundaries yield
    // identical knot vectors, which makeStripFace relies on to pair poles.
    static occ::handle<Geom_BSplineCurve> interpolateBoundary(
        const std::vector<gp_Pnt> &points,
        const std::vector<double> &parameters)
    {
        const int count = static_cast<int>(points.size());
        occ::handle<NCollection_HArray1<gp_Pnt>> pointArray =
            new NCollection_HArray1<gp_Pnt>(1, count);
        occ::handle<NCollection_HArray1<double>> parameterArray =
            new NCollection_HArray1<double>(1, count);
        for (int index = 0; index < count; ++index) {
            pointArray->SetValue(index + 1, points[index]);
            parameterArray->SetValue(index + 1, parameters[index]);
        }
        GeomAPI_Interpolate interpolate(pointArray,
                                        parameterArray,
                                        false,
                                        Precision::Confusion());
        interpolate.Perform();
        if (!interpolate.IsDone()) {
            return {};
        }
        return interpolate.Curve();
    }

    static void dropCoincidentSamples(std::vector<gp_Pnt> &points)
    {
        std::vector<gp_Pnt> kept;
        kept.reserve(points.size());
        for (const gp_Pnt &point : points) {
            if (kept.empty()
                || point.Distance(kept.back()) > Precision::Confusion()) {
                kept.push_back(point);
            }
        }
        points = std::move(kept);
    }

    // Re-samples a boundary polyline to targetCount points spaced evenly
    // along its interpolated curve, so differently sampled boundaries can
    // be paired rung by rung.
    static bool resampleBoundary(std::vector<gp_Pnt> &points,
                                 int targetCount)
    {
        dropCoincidentSamples(points);
        if (points.size() < 2 || targetCount < 2) {
            return false;
        }
        std::vector<double> parameters(points.size(), 0.0);
        for (std::size_t index = 1; index < points.size(); ++index) {
            parameters[index] = parameters[index - 1]
                                + points[index].Distance(points[index - 1]);
        }
        const occ::handle<Geom_BSplineCurve> curve =
            interpolateBoundary(points, parameters);
        if (curve.IsNull()) {
            return false;
        }
        const double length = parameters.back();
        std::vector<gp_Pnt> resampled(targetCount);
        for (int index = 0; index < targetCount; ++index) {
            resampled[index] =
                curve->Value(length * index / (targetCount - 1));
        }
        points = std::move(resampled);
        return true;
    }

    // An exact ruled loft between the strip's two boundary polylines: both
    // interpolate their samples on one shared chord-length
    // parameterization, so each legacy rung (sample j to sample j) and the
    // skin-following curvature of both boundary edges lie exactly on the
    // face.
    static TopoDS_Face makeStripFace(const CapturedStrip &strip)
    {
        std::vector<gp_Pnt> a = strip.curveA;
        std::vector<gp_Pnt> b = strip.curveB;
        if (a.size() != b.size()) {
            const int targetCount =
                static_cast<int>(std::max(a.size(), b.size()));
            if (!resampleBoundary(a, targetCount)
                || !resampleBoundary(b, targetCount)) {
                return {};
            }
        }
        // Drop rungs that collapse on either boundary (piecewise legacy
        // curves repeat their junction samples).
        std::vector<gp_Pnt> firstBoundary{a.front()};
        std::vector<gp_Pnt> secondBoundary{b.front()};
        for (std::size_t index = 1; index < a.size(); ++index) {
            if (a[index].Distance(firstBoundary.back())
                    > Precision::Confusion()
                && b[index].Distance(secondBoundary.back())
                       > Precision::Confusion()) {
                firstBoundary.push_back(a[index]);
                secondBoundary.push_back(b[index]);
            }
        }
        if (firstBoundary.size() < 2) {
            return {};
        }
        std::vector<double> parameters(firstBoundary.size(), 0.0);
        for (std::size_t index = 1; index < firstBoundary.size();
             ++index) {
            parameters[index] =
                parameters[index - 1]
                + 0.5
                      * (firstBoundary[index].Distance(
                             firstBoundary[index - 1])
                         + secondBoundary[index].Distance(
                             secondBoundary[index - 1]));
        }
        const occ::handle<Geom_BSplineCurve> firstCurve =
            interpolateBoundary(firstBoundary, parameters);
        const occ::handle<Geom_BSplineCurve> secondCurve =
            interpolateBoundary(secondBoundary, parameters);
        if (firstCurve.IsNull() || secondCurve.IsNull()
            || firstCurve->NbPoles() != secondCurve->NbPoles()
            || firstCurve->NbKnots() != secondCurve->NbKnots()) {
            return {};
        }
        NCollection_Array2<gp_Pnt> poles(1, firstCurve->NbPoles(), 1, 2);
        for (int index = 1; index <= firstCurve->NbPoles(); ++index) {
            poles.SetValue(index, 1, firstCurve->Pole(index));
            poles.SetValue(index, 2, secondCurve->Pole(index));
        }
        NCollection_Array1<double> uKnots(1, firstCurve->NbKnots());
        firstCurve->Knots(uKnots);
        NCollection_Array1<int> uMultiplicities(1, firstCurve->NbKnots());
        firstCurve->Multiplicities(uMultiplicities);
        NCollection_Array1<double> vKnots(1, 2);
        vKnots.SetValue(1, 0.0);
        vKnots.SetValue(2, 1.0);
        NCollection_Array1<int> vMultiplicities(1, 2);
        vMultiplicities.SetValue(1, 2);
        vMultiplicities.SetValue(2, 2);
        const occ::handle<Geom_BSplineSurface> surface =
            new Geom_BSplineSurface(poles,
                                    uKnots,
                                    vKnots,
                                    uMultiplicities,
                                    vMultiplicities,
                                    firstCurve->Degree(),
                                    1);
        BRepBuilderAPI_MakeFace makeFace(surface, Precision::Confusion());
        if (!makeFace.IsDone()) {
            return {};
        }
        return makeFace.Face();
    }

    // The H/V/VH diagonal strips and the trailing-edge mini-ribs become
    // ruled faces; strips sharing a label (e.g. the three sheets of one
    // VH-rib row) merge into a single named part.
    void addStripParts(AssemblyGroup &wing,
                       lep::NurbsWriteResult &result) const
    {
        if (capturedStrips_.empty()) {
            return;
        }
        struct StripPart
        {
            AssemblyGroup *group = nullptr;
            std::string label;
            PartColor color;
            std::vector<TopoDS_Shape> shapes;
            bool hasFaces = false;
        };
        std::vector<StripPart> stripParts;
        for (const CapturedStrip &strip : capturedStrips_) {
            AssemblyGroup *group =
                strip.minirib
                    ? &wing.group(ribsGroupName).group(miniribsGroupName)
                    : &wing.group(diagonalsGroupName);
            StripPart *part = nullptr;
            for (StripPart &candidate : stripParts) {
                if (candidate.group == group
                    && candidate.label == strip.label) {
                    part = &candidate;
                    break;
                }
            }
            if (part == nullptr) {
                stripParts.push_back(
                    {group,
                     strip.label,
                     strip.minirib ? ribColor : diagonalColor,
                     {},
                     false});
                part = &stripParts.back();
            }
            TopoDS_Shape shape;
            try {
                shape = makeStripFace(strip);
            } catch (const Standard_Failure &) {
                shape = {};
            }
            const bool face = !shape.IsNull();
            if (!face) {
                // Keep at least the boundary outline if OCCT could not
                // rule a face between the two curves.
                std::vector<TopoDS_Shape> outline;
                for (const std::vector<gp_Pnt> *boundary :
                     {&strip.curveA, &strip.curveB}) {
                    for (std::size_t index = 1; index < boundary->size();
                         ++index) {
                        BRepBuilderAPI_MakeEdge makeEdge(
                            makeLinearSpline((*boundary)[index - 1],
                                             (*boundary)[index]));
                        if (makeEdge.IsDone()) {
                            outline.push_back(makeEdge.Edge());
                        }
                    }
                }
                if (outline.empty()) {
                    continue;
                }
                warnings_.push_back(
                    strip.label
                    + " could not be ruled into a surface and stays a "
                      "wireframe outline.");
                shape = makeCompound(outline);
            }
            // A strip whose samples all sit on the symmetry plane, or one
            // that spans the centre cell mapping onto itself, must not get
            // a mirror copy.
            const auto centered = [](const gp_Pnt &point) {
                return std::abs(point.X())
                       <= symmetryPlaneToleranceMillimetres;
            };
            const bool onCenter =
                std::all_of(strip.curveA.begin(), strip.curveA.end(),
                            centered)
                && std::all_of(strip.curveB.begin(), strip.curveB.end(),
                               centered);
            const auto mirrorsOntoItself = [&]() {
                if (strip.curveA.size() != strip.curveB.size()) {
                    return false;
                }
                for (std::size_t index = 0; index < strip.curveA.size();
                     ++index) {
                    const gp_Pnt &a = strip.curveA[index];
                    if (gp_Pnt(-a.X(), a.Y(), a.Z())
                            .Distance(strip.curveB[index])
                        > symmetryPlaneToleranceMillimetres) {
                        return false;
                    }
                }
                return true;
            };
            part->shapes.push_back(shape);
            part->hasFaces = part->hasFaces || face;
            result.surfaceCount += face ? 1 : 0;
            if (!onCenter && !mirrorsOntoItself()) {
                const TopoDS_Shape mirroredShape = mirrored(shape);
                if (!mirroredShape.IsNull()) {
                    part->shapes.push_back(mirroredShape);
                    result.surfaceCount += face ? 1 : 0;
                }
            }
        }
        for (const StripPart &part : stripParts) {
            if (part.shapes.empty()) {
                continue;
            }
            part.group->parts.push_back({part.label,
                                         makeCompound(part.shapes),
                                         part.color,
                                         part.color,
                                         part.hasFaces});
        }
    }

    void writeAssembly(const AssemblyGroup &wing,
                       const std::filesystem::path &path,
                       lep::NurbsWriteResult &result) const
    {
        // A .xbf target serializes the XCAF document in OCCT's binary
        // format: identical geometry and assembly names, but no STEP
        // entity translation, so previews write and load far faster.
        const bool binary = path.extension() == ".xbf";
        if (binary) {
            BinXCAFDrivers::DefineFormat(
                XCAFApp_Application::GetApplication());
        }
        const occ::handle<TDocStd_Document> document =
            newDocument(binary ? "BinXCAF" : "MDTV-XCAF");
        if (document.IsNull()) {
            result.error = "OCCT could not create the XCAF document.";
            return;
        }
        const occ::handle<XCAFDoc_ShapeTool> shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const occ::handle<XCAFDoc_ColorTool> colorTool =
            XCAFDoc_DocumentTool::ColorTool(document->Main());
        XCAFDoc_ShapeTool::SetAutoNaming(false);

        const TDF_Label wingLabel = shapeTool->NewShape();
        setLabelName(wingLabel, wing.name);
        addGroupContents(shapeTool, colorTool, wingLabel, wing, result);
        shapeTool->UpdateAssemblies();

        if (binary) {
            const auto encoded = path.u8string();
            const std::string encodedPath{
                reinterpret_cast<const char *>(encoded.data()),
                encoded.size()};
            if (XCAFApp_Application::GetApplication()->SaveAs(
                    document,
                    TCollection_ExtendedString(encodedPath.c_str(), true))
                != PCDM_SS_OK) {
                result.error =
                    "OCCT could not write the binary XCAF model.";
            }
            return;
        }

        // AP242 preserves exact B-spline geometry, carries the assembly
        // names and colours, and is the current STEP schema. All model
        // coordinates are already represented in mm.
        STEPCAFControl_Writer writer;
        // The writer initializes the shared STEP parameters in its
        // constructor, so select the schema only after constructing it and
        // recreate the model with those parameters.
        Interface_Static::SetIVal("write.step.schema", 5); // AP242DIS
        Interface_Static::SetCVal("write.step.unit", "MM");
        Interface_Static::SetIVal("write.surfacecurve.mode", 1);
        writer.ChangeWriter().Model(true);
        writer.ChangeWriter().SetTolerance(pointToleranceMillimetres);
        writer.SetColorMode(true);
        writer.SetNameMode(true);
        if (!writer.Transfer(document, STEPControl_AsIs)) {
            result.error = "OCCT could not transfer the NURBS model to STEP.";
            return;
        }

        const auto encoded = path.u8string();
        const std::string encodedPath{
            reinterpret_cast<const char *>(encoded.data()),
            encoded.size()};
        if (writer.Write(encodedPath.c_str()) != IFSelect_RetDone) {
            result.error = "OCCT could not write the STEP file.";
        }
    }

    static occ::handle<TDocStd_Document> newDocument(const char *format)
    {
        occ::handle<TDocStd_Document> document;
        XCAFApp_Application::GetApplication()->NewDocument(
            format,
            document);
        return document;
    }

    static void setLabelName(const TDF_Label &label, const std::string &name)
    {
        TDataStd_Name::Set(
            label,
            TCollection_ExtendedString(name.c_str(), true));
    }

    void addGroupContents(const occ::handle<XCAFDoc_ShapeTool> &shapeTool,
                          const occ::handle<XCAFDoc_ColorTool> &colorTool,
                          const TDF_Label &groupLabel,
                          const AssemblyGroup &group,
                          lep::NurbsWriteResult &result) const
    {
        for (const AssemblyGroup &child : group.groups) {
            if (child.empty()) {
                continue;
            }
            const TDF_Label childLabel = shapeTool->NewShape();
            setLabelName(childLabel, child.name);
            const TDF_Label instance =
                shapeTool->AddComponent(
                    groupLabel,
                    childLabel,
                    TopLoc_Location());
            setLabelName(instance, child.name);
            addGroupContents(shapeTool, colorTool, childLabel, child, result);
        }
        for (const AssemblyPart &part : group.parts) {
            const TDF_Label partLabel =
                shapeTool->AddShape(part.shape, false);
            setLabelName(partLabel, part.name);
            const TDF_Label instance =
                shapeTool->AddComponent(
                    groupLabel,
                    partLabel,
                    TopLoc_Location());
            setLabelName(instance, part.name);
            colorTool->SetColor(
                partLabel,
                part.hasFaces ? part.faceColor.color()
                              : part.curveColor.color(),
                XCAFDoc_ColorGen);
            if (part.hasFaces) {
                colorTool->SetColor(
                    partLabel,
                    part.faceColor.color(),
                    XCAFDoc_ColorSurf);
            }
            colorTool->SetColor(
                partLabel,
                part.curveColor.color(),
                XCAFDoc_ColorCurv);
            ++result.partCount;
        }
    }

public:
    // Writes the Playground simulation mesh: the coarse skin samples of
    // both wing halves welded into one node table, quad connectivity per
    // region, rib outline loops for internal webs, and the labelled
    // suspension lines. Everything is in millimetres, Z up, matching the
    // STEP model.
    bool writeSimMesh(const std::filesystem::path &path,
                      std::string &error) const
    {
        if (simRegions_.empty()) {
            error = "No simulation skin was captured";
            return false;
        }

        std::vector<gp_Pnt> nodes;
        std::unordered_map<QuantizedPoint, int, QuantizedPointHash> welded;
        const auto nodeIndex = [&](const gp_Pnt &point, bool mirror) {
            // The centre rib lands within float residue of the symmetry
            // plane (~0.1 mm); snap it on so the mirrored half welds to
            // the right half instead of floating 0.2 mm away from it.
            const double snappedX =
                std::abs(point.X()) < 0.5 ? 0.0 : point.X();
            const gp_Pnt placed(mirror ? -snappedX : snappedX,
                                point.Y(),
                                point.Z());
            const auto [entry, inserted] =
                welded.try_emplace(quantize(placed),
                                   static_cast<int>(nodes.size()));
            if (inserted) {
                nodes.push_back(placed);
            }
            return entry->second;
        };

        std::vector<std::array<int, 4>> quads;
        // Which skin each quad came from, parallel to `quads`, so the
        // Playground can draw the surfaces separately.
        std::vector<int> quadSurfaces;
        // Rib outline loops keyed by (ribIndex, mirrored side), assembled
        // from the regions' rib-side sample columns in station order.
        std::map<std::pair<int, bool>, std::vector<std::pair<int, int>>>
            ribColumns;

        for (const SimRegionCapture &region : simRegions_) {
            for (const bool mirror : {false, true}) {
                // A panel that straddles the centreline (the centre cell of
                // a wing whose innermost ribs sit either side of x=0) is its
                // own mirror image, and emitting the mirrored copy would lay
                // duplicate faces over it. Decided from the geometry rather
                // than from panelIndex == 1: wings whose innermost rib sits
                // *on* the centreline have a genuine second half-panel there
                // that must be mirrored.
                if (mirror && region.selfMirrored()) {
                    continue;
                }
                std::vector<std::array<int, simSpanColumnCount>> grid;
                grid.reserve(region.rows.size());
                for (const auto &row : region.rows) {
                    std::array<int, simSpanColumnCount> ids{};
                    for (int column = 0;
                         column < simSpanColumnCount;
                         ++column) {
                        ids[column] = nodeIndex(row[column], mirror);
                    }
                    grid.push_back(ids);
                }
                for (std::size_t rowIndex = 0;
                     rowIndex + 1 < grid.size();
                     ++rowIndex) {
                    for (int column = 0;
                         column + 1 < simSpanColumnCount;
                         ++column) {
                        quads.push_back({grid[rowIndex][column],
                                         grid[rowIndex][column + 1],
                                         grid[rowIndex + 1][column + 1],
                                         grid[rowIndex + 1][column]});
                        quadSurfaces.push_back(
                            static_cast<int>(region.surface));
                    }
                }
                // Span column 0 lies on rib panelIndex, the last column on
                // rib panelIndex-1. Both edges are registered: on wings
                // whose innermost rib sits on the centreline that rib is no
                // panel's column 0, and harvesting only column 0 dropped it
                // — leaving the two centre bays joined into one, which then
                // ballooned as a single double-width cell. Ribs shared by
                // neighbouring panels yield identical loops and are removed
                // by the uniqueLoops filter below.
                for (std::size_t rowIndex = 0;
                     rowIndex < grid.size();
                     ++rowIndex) {
                    ribColumns[{region.panelIndex, mirror}].emplace_back(
                        region.stations[rowIndex], grid[rowIndex][0]);
                    ribColumns[{region.panelIndex - 1, mirror}].emplace_back(
                        region.stations[rowIndex],
                        grid[rowIndex][simSpanColumnCount - 1]);
                }
            }
        }

        std::vector<std::vector<int>> ribLoops;
        // (rib index, mirrored) per emitted loop, so each one can be paired
        // with the hole outlines of the rib it actually is.
        std::vector<std::pair<int, bool>> ribLoopKeys;
        std::set<std::vector<int>> uniqueLoops;
        for (auto &[key, column] : ribColumns) {
            std::stable_sort(column.begin(),
                             column.end(),
                             [](const auto &a, const auto &b) {
                                 return a.first < b.first;
                             });
            std::vector<int> loop;
            for (const auto &[station, node] : column) {
                if (loop.empty() || loop.back() != node) {
                    loop.push_back(node);
                }
            }
            // The mirrored centre rib welds onto the unmirrored one; keep
            // one copy of each distinct loop.
            if (loop.size() >= 3 && uniqueLoops.insert(loop).second) {
                ribLoops.push_back(std::move(loop));
                ribLoopKeys.push_back(key);
            }
        }

        // Hole outlines for one rib, as polylines in model space. The wires
        // come from the same builder that cuts the STEP rib faces, so every
        // hole type — ellipse, triangle, rounded rectangle — is exact here
        // rather than reconstructed downstream. 1 mm of chord deflection is
        // far finer than the mesh that tests against them.
        const auto ribHoleOutlines =
            [this](int ribIndex,
                   bool mirror) -> std::vector<std::vector<gp_Pnt>> {
            std::vector<std::vector<gp_Pnt>> outlines;
            const auto captured = capturedRibs_.find(ribIndex);
            if (captured == capturedRibs_.end()
                || captured->second.holes.empty()) {
                return outlines;
            }
            RibFrame frame;
            if (!fitRibFrame(captured->second.planarPoints,
                             captured->second.spatialPoints,
                             frame)) {
                return outlines;
            }
            for (std::size_t holeIndex = 0;
                 holeIndex < captured->second.holes.size();
                 ++holeIndex) {
                int edgeCount = 0;
                std::vector<std::string> ignored;
                const TopoDS_Wire wire =
                    ribHoleWire(captured->second.holes[holeIndex],
                                frame,
                                ribIndex,
                                static_cast<int>(holeIndex) + 1,
                                edgeCount,
                                ignored);
                if (wire.IsNull()) {
                    continue;
                }
                std::vector<gp_Pnt> outline;
                for (BRepTools_WireExplorer explorer(wire);
                     explorer.More();
                     explorer.Next()) {
                    BRepAdaptor_Curve curve(explorer.Current());
                    GCPnts_QuasiUniformDeflection sampler(curve, 1.0);
                    if (!sampler.IsDone()) {
                        continue;
                    }
                    // The last point of one edge is the first of the next.
                    for (int point = 1; point < sampler.NbPoints(); ++point) {
                        gp_Pnt placed = sampler.Value(point);
                        if (mirror) {
                            placed.SetX(-placed.X());
                        }
                        outline.push_back(placed);
                    }
                }
                if (outline.size() >= 3) {
                    outlines.push_back(std::move(outline));
                }
            }
            return outlines;
        };

        const auto escaped = [](const std::string &text) {
            std::string result;
            for (const char character : text) {
                if (character == '"' || character == '\\') {
                    result.push_back('\\');
                }
                result.push_back(character);
            }
            return result;
        };

        std::ostringstream json;
        json.setf(std::ios::fixed);
        json.precision(3);
        json << "{\n\"version\": 1,\n\"unit\": \"mm\",\n\"nodes\": [";
        for (std::size_t index = 0; index < nodes.size(); ++index) {
            json << (index == 0 ? "\n" : ",\n")
                 << '[' << nodes[index].X() << ',' << nodes[index].Y()
                 << ',' << nodes[index].Z() << ']';
        }
        json << "\n],\n\"quads\": [";
        for (std::size_t index = 0; index < quads.size(); ++index) {
            const auto &quad = quads[index];
            json << (index == 0 ? "\n" : ",\n")
                 << '[' << quad[0] << ',' << quad[1] << ',' << quad[2]
                 << ',' << quad[3] << ']';
        }
        // Per-quad skin tag, indexing surfaceNames. Readers that predate
        // this field simply draw every quad alike.
        json << "\n],\n\"surfaceNames\": "
                "[\"extrados\",\"vent\",\"intrados\"],\n"
                "\"quadSurfaces\": [";
        for (std::size_t index = 0; index < quadSurfaces.size(); ++index) {
            json << (index == 0 ? "" : ",") << quadSurfaces[index];
        }
        json << "],\n\"ribLoops\": [";
        for (std::size_t index = 0; index < ribLoops.size(); ++index) {
            json << (index == 0 ? "\n" : ",\n") << '[';
            for (std::size_t node = 0;
                 node < ribLoops[index].size();
                 ++node) {
                json << (node == 0 ? "" : ",") << ribLoops[index][node];
            }
            json << ']';
        }
        // Hole outlines per rib loop, in the same order, so the Playground
        // can mesh the ribs with their holes cut out.
        json << "\n],\n\"ribHoles\": [";
        for (std::size_t index = 0; index < ribLoops.size(); ++index) {
            const std::vector<std::vector<gp_Pnt>> outlines =
                index < ribLoopKeys.size()
                    ? ribHoleOutlines(ribLoopKeys[index].first,
                                      ribLoopKeys[index].second)
                    : std::vector<std::vector<gp_Pnt>>{};
            json << (index == 0 ? "\n" : ",\n") << '[';
            for (std::size_t outline = 0; outline < outlines.size();
                 ++outline) {
                json << (outline == 0 ? "" : ",") << '[';
                for (std::size_t point = 0;
                     point < outlines[outline].size();
                     ++point) {
                    const gp_Pnt &placed = outlines[outline][point];
                    json << (point == 0 ? "" : ",") << '['
                         << placed.X() << ',' << placed.Y() << ','
                         << placed.Z() << ']';
                }
                json << ']';
            }
            json << ']';
        }

        // Internal diagonal/mini-rib sheets as paired sample rows: the
        // Playground ties each pair together so line load spreads across
        // ribs the way the real V-ribs make it.
        json << "\n],\n\"straps\": [";
        bool firstStrap = true;
        for (const CapturedStrip &strip : capturedStrips_) {
            if (strip.curveA.size() < 2 || strip.curveB.size() < 2) {
                continue;
            }
            constexpr int strapSamples = 6;
            const auto sampleAt = [](const std::vector<gp_Pnt> &curve,
                                     int index) {
                const std::size_t position =
                    static_cast<std::size_t>(
                        std::llround(static_cast<double>(index)
                                     * (curve.size() - 1)
                                     / (strapSamples - 1)));
                return curve[position];
            };
            // A strip lying symmetric on the centre plane is its own
            // mirror image.
            double minX = std::numeric_limits<double>::max();
            double maxX = std::numeric_limits<double>::lowest();
            for (const gp_Pnt &point : strip.curveA) {
                minX = std::min(minX, point.X());
                maxX = std::max(maxX, point.X());
            }
            const bool selfMirrored = std::abs(minX + maxX) < 0.5;
            for (const bool mirror : {false, true}) {
                if (mirror && selfMirrored) {
                    continue;
                }
                const double sign = mirror ? -1.0 : 1.0;
                json << (firstStrap ? "\n" : ",\n") << "{\"a\":[";
                for (int index = 0; index < strapSamples; ++index) {
                    const gp_Pnt point = sampleAt(strip.curveA, index);
                    json << (index == 0 ? "" : ",") << '['
                         << sign * point.X() << ',' << point.Y() << ','
                         << point.Z() << ']';
                }
                json << "],\"b\":[";
                for (int index = 0; index < strapSamples; ++index) {
                    const gp_Pnt point = sampleAt(strip.curveB, index);
                    json << (index == 0 ? "" : ",") << '['
                         << sign * point.X() << ',' << point.Y() << ','
                         << point.Z() << ']';
                }
                json << "]}";
                firstStrap = false;
            }
        }

        json << "\n],\n\"lines\": [";
        bool firstLine = true;
        std::unordered_set<QuantizedSemanticSegment,
                           QuantizedSemanticSegmentHash> emittedLines;
        for (const CapturedLine &line : capturedLines_) {
            const QuantizedSemanticSegment key{
                quantizeSegment(line.start, line.end),
                line.planIndex,
                line.brake,
            };
            if (!emittedLines.insert(key).second) {
                continue;
            }
            // The legacy drawing pass has already visited both sides of the
            // full wing. Mirroring here used to emit every physical segment
            // twice. Preserve first-seen order and metadata from that full
            // captured stream instead.
            json << (firstLine ? "\n" : ",\n")
                 << "{\"a\":[" << line.start.X() << ','
                 << line.start.Y() << ',' << line.start.Z()
                 << "],\"b\":[" << line.end.X() << ','
                 << line.end.Y() << ',' << line.end.Z()
                 << "],\"label\":\"" << escaped(line.label)
                 << "\",\"plan\":" << line.planIndex
                 << ",\"brake\":" << (line.brake ? 1 : 0) << '}';
            firstLine = false;
        }
        json << "\n]\n}\n";

        std::ofstream file(path, std::ios::binary);
        if (!file) {
            error = "Could not open " + path.string() + " for writing";
            return false;
        }
        file << json.str();
        if (!file) {
            error = "Could not write " + path.string();
            return false;
        }
        return true;
    }

private:
    std::vector<PanelSurface> panels_;
    std::map<int, CapturedRib> capturedRibs_;
    std::vector<CapturedLine> capturedLines_;
    std::vector<CapturedStrip> capturedStrips_;
    std::vector<SimRegionCapture> simRegions_;
    mutable std::vector<std::string> errors_;
    mutable std::vector<std::string> warnings_;
    bool captureLines_ = false;
    LineTag currentLineTag_;
    double maximumSourceDeviation_ = 0.0;
    double maximumLegacyAgreement_ = 0.0;
};

NurbsModel &model()
{
    static NurbsModel instance;
    return instance;
}

} // namespace

namespace lep {

void resetNurbsModel()
{
    model().reset();
}

NurbsWriteResult writeNurbsStep(const std::filesystem::path &path,
                                bool includeConstructionCurves)
{
    return model().writeStep(path, includeConstructionCurves);
}

bool writeSimMesh(const std::filesystem::path &path, std::string &error)
{
    return model().writeSimMesh(path, error);
}

} // namespace lep

extern "C" void lep_nurbs_capture_panel(const double *u,
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
                                         int singleSkin)
{
    model().capturePanel(
        u,
        v,
        w,
        shapingHeight,
        legacyTessellation,
        panelIndex,
        totalPointCount,
        upperPointCount,
        ventPointCount,
        segmentCount,
        includeVentSurface != 0,
        singleSkin != 0);
}

extern "C" void lep_nurbs_capture_rib(const double *u,
                                       const double *v,
                                       const double *w,
                                       const double *holes,
                                       double chordCentimetres,
                                       int ribIndex,
                                       int totalPointCount)
{
    model().captureRib(
        u,
        v,
        w,
        holes,
        chordCentimetres,
        ribIndex,
        totalPointCount);
}

extern "C" void lep_nurbs_set_line_capture(int enabled)
{
    model().setLineCapture(enabled != 0);
}

extern "C" void lep_nurbs_set_line_tag(const char *label,
                                        int labelLength,
                                        int planIndex,
                                        int isBrake,
                                        const char *typeName,
                                        int typeNameLength,
                                        double diameterMm)
{
    model().setLineTag(
        label,
        labelLength,
        planIndex,
        isBrake != 0,
        typeName,
        typeNameLength,
        diameterMm);
}

extern "C" void lep_nurbs_capture_diagonal_strip(const char *kind,
                                                  int kindLength,
                                                  int index,
                                                  const double *xA,
                                                  const double *yA,
                                                  const double *zA,
                                                  const double *xB,
                                                  const double *yB,
                                                  const double *zB,
                                                  int pointCount,
                                                  int stride)
{
    model().captureDiagonalStrip(
        kind, kindLength, index, xA, yA, zA, xB, yB, zB, pointCount, stride);
}

extern "C" void lep_nurbs_capture_miniribs(const double *x,
                                            const double *y,
                                            const double *z,
                                            const int *np,
                                            const double *rib,
                                            int ribCount,
                                            int singleSkin)
{
    model().captureMiniribs(x, y, z, np, rib, ribCount, singleSkin != 0);
}

extern "C" void lep_nurbs_capture_line(double x1,
                                        double y1,
                                        double z1,
                                        double x2,
                                        double y2,
                                        double z2,
                                        int colorIndex)
{
    model().captureLine({
        modelPoint(x1, y1, z1),
        modelPoint(x2, y2, z2),
        colorIndex,
    });
}
