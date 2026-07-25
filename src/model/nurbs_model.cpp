#include "nurbs_model.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GeomConvert.hxx>
#include <GeomFill_NSections.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Sequence.hxx>
#include <Precision.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <Standard_ConstructionError.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <list>
#include <map>
#include <numbers>
#include <sstream>
#include <string>
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
constexpr const char *ribsGroupName = "Ribs";
constexpr const char *linesGroupName = "Lines";
constexpr const char *brakeGroupName = "Brake lines";
constexpr const char *diagonalsGroupName = "Diagonals";
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
    std::string group;
    std::string label;
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

struct PanelSurface
{
    Region region = Region::Extrados;
    int panelIndex = 0;
    occ::handle<Geom_BSplineSurface> surface;
    TopoDS_Face rightFace;
    TopoDS_Face leftFace;
    std::vector<TopoDS_Edge> rightWireframe;
    std::vector<TopoDS_Edge> leftWireframe;
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

bool curveOnSymmetryPlane(const occ::handle<Geom_Curve> &curve)
{
    if (curve.IsNull()) {
        return false;
    }
    constexpr int sampleCount = 9;
    const double first = curve->FirstParameter();
    const double last = curve->LastParameter();
    for (int sample = 0; sample <= sampleCount; ++sample) {
        const double parameter =
            first
            + (last - first) * static_cast<double>(sample)
                  / static_cast<double>(sampleCount);
        if (std::abs(curve->Value(parameter).X())
            > symmetryPlaneToleranceMillimetres) {
            return false;
        }
    }
    return true;
}

TopoDS_Shape mirrored(const TopoDS_Shape &shape)
{
    BRepBuilderAPI_Transform mirror(shape, mirrorTransform(), true);
    if (!mirror.IsDone()) {
        return {};
    }
    return mirror.Shape();
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
        capturedLines_.clear();
        errors_.clear();
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
        line.group = currentLineTag_.group;
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
                    bool brake)
    {
        currentLineTag_.label =
            label != nullptr && labelLength > 0
                ? trimmedLabel(label, labelLength)
                : std::string();
        currentLineTag_.planIndex = std::clamp(planIndex, 0, 6);
        currentLineTag_.brake = brake;
        currentLineTag_.group.clear();
    }

    void tagDiagonal(const char *kind, int kindLength, int index)
    {
        currentLineTag_ = {};
        currentLineTag_.group = diagonalsGroupName;
        currentLineTag_.label =
            (kind != nullptr && kindLength > 0
                 ? trimmedLabel(kind, kindLength)
                 : std::string("Diagonal"))
            + " " + std::to_string(index);
    }

    lep::NurbsWriteResult writeStep(const std::filesystem::path &path)
    {
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
            addPanelParts(wing, result);
            addRibParts(wing, result);
            if (!errors_.empty()) {
                result.error = errors_.front();
                return result;
            }
            addLineParts(wing, result);

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

        result.success = result.error.empty();
        return result;
    }

private:
    struct LineTag
    {
        std::string group;
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
            NCollection_Sequence<occ::handle<Geom_Curve>> sections;
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
                sections.Append(curve);
                // Point correspondence is established by the legacy remap
                // stage. Keeping this canonical parameter on every panel
                // makes adjacent lofts share the same airfoil boundary curve,
                // allowing OCCT to sew them into real shell topology.
                sectionParameters.Append(
                    static_cast<double>(pointIndex - firstPoint));
            }

            GeomFill_NSections loft(
                sections,
                sectionParameters);
            const occ::handle<Geom_BSplineSurface> surface =
                loft.BSplineSurface();
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
                       lep::NurbsWriteResult &result) const
    {
        for (const PanelSurface &panel : panels_) {
            AssemblyGroup &regionGroup =
                wing.group(regionGroupName(panel.region));
            const std::string partName =
                "Panel " + std::to_string(panel.panelIndex);
            const PartColor faceColor = regionColor(panel.region);

            const auto makePanelShape =
                [](const TopoDS_Face &face,
                   const std::vector<TopoDS_Edge> &wireframe) {
                    std::vector<TopoDS_Shape> shapes;
                    shapes.reserve(wireframe.size() + 1);
                    shapes.push_back(face);
                    shapes.insert(
                        shapes.end(), wireframe.begin(), wireframe.end());
                    return makeCompound(shapes);
                };

            regionGroup.group(rightSideName).parts.push_back(
                {partName,
                 makePanelShape(panel.rightFace, panel.rightWireframe),
                 faceColor,
                 wireframeColor,
                 true});
            regionGroup.group(leftSideName).parts.push_back(
                {partName,
                 makePanelShape(panel.leftFace, panel.leftWireframe),
                 faceColor,
                 wireframeColor,
                 true});
            result.surfaceCount += 2;
            result.splineCount += static_cast<int>(
                panel.rightWireframe.size() + panel.leftWireframe.size());
        }
    }

    // Rib outlines are the exact chordwise boundary curves of the lofted
    // panel surfaces: panel i spans rib i (u = uFirst) to rib i-1
    // (u = uLast), so every captured panel contributes rib i, and rib i-1
    // is taken from panel i only when panel i-1 was not built (which also
    // yields the centre rib 0 from panel 1).
    void addRibParts(AssemblyGroup &wing,
                     lep::NurbsWriteResult &result) const
    {
        std::unordered_set<int> capturedPanels;
        for (const PanelSurface &panel : panels_) {
            capturedPanels.insert(panel.panelIndex);
        }

        std::map<int, std::vector<TopoDS_Edge>> ribEdges;
        for (const PanelSurface &panel : panels_) {
            double uFirst = 0.0;
            double uLast = 0.0;
            double vFirst = 0.0;
            double vLast = 0.0;
            panel.surface->Bounds(uFirst, uLast, vFirst, vLast);

            const auto addRibCurve = [this, &ribEdges](
                                         int ribIndex,
                                         const occ::handle<Geom_Curve> &curve) {
                if (curve.IsNull()) {
                    errors_.push_back(
                        "Could not extract the outline curve for rib "
                        + std::to_string(ribIndex));
                    return;
                }
                BRepBuilderAPI_MakeEdge makeEdge(curve);
                if (!makeEdge.IsDone()) {
                    errors_.push_back(
                        "Could not build the outline edge for rib "
                        + std::to_string(ribIndex));
                    return;
                }
                ribEdges[ribIndex].push_back(makeEdge.Edge());
            };

            addRibCurve(panel.panelIndex, panel.surface->UIso(uFirst));
            if (!capturedPanels.contains(panel.panelIndex - 1)) {
                addRibCurve(panel.panelIndex - 1,
                            panel.surface->UIso(uLast));
            }
        }
        if (!errors_.empty()) {
            return;
        }

        AssemblyGroup &ribs = wing.group(ribsGroupName);
        for (const auto &[ribIndex, edges] : ribEdges) {
            const std::string partName = "Rib " + std::to_string(ribIndex);
            const bool onCenter = std::all_of(
                edges.begin(),
                edges.end(),
                [](const TopoDS_Edge &edge) {
                    double first = 0.0;
                    double last = 0.0;
                    return curveOnSymmetryPlane(
                        BRep_Tool::Curve(edge, first, last));
                });

            std::vector<TopoDS_Shape> rightShapes(edges.begin(), edges.end());
            if (onCenter) {
                ribs.group(centerSideName).parts.push_back(
                    {partName,
                     makeCompound(rightShapes),
                     ribColor,
                     ribColor,
                     false});
            } else {
                std::vector<TopoDS_Shape> leftShapes;
                for (const TopoDS_Edge &edge : edges) {
                    const TopoDS_Shape mirroredEdge = mirrored(edge);
                    if (!mirroredEdge.IsNull()) {
                        leftShapes.push_back(mirroredEdge);
                    }
                }
                ribs.group(rightSideName).parts.push_back(
                    {partName,
                     makeCompound(rightShapes),
                     ribColor,
                     ribColor,
                     false});
                ribs.group(leftSideName).parts.push_back(
                    {partName,
                     makeCompound(leftShapes),
                     ribColor,
                     ribColor,
                     false});
            }
            ++result.ribCount;
            result.splineCount +=
                static_cast<int>(edges.size() * (onCenter ? 1 : 2));
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
            if (!line.group.empty()) {
                // Custom top-level groups such as the H/V rib diagonals.
                group = &wing.group(line.group);
                color = diagonalColor;
            } else if (line.brake) {
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

    void writeAssembly(const AssemblyGroup &wing,
                       const std::filesystem::path &path,
                       lep::NurbsWriteResult &result) const
    {
        const occ::handle<TDocStd_Document> document = newDocument();
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

    static occ::handle<TDocStd_Document> newDocument()
    {
        occ::handle<TDocStd_Document> document;
        XCAFApp_Application::GetApplication()->NewDocument(
            "MDTV-XCAF",
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

    std::vector<PanelSurface> panels_;
    std::vector<CapturedLine> capturedLines_;
    mutable std::vector<std::string> errors_;
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

NurbsWriteResult writeNurbsStep(const std::filesystem::path &path)
{
    return model().writeStep(path);
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

extern "C" void lep_nurbs_set_line_capture(int enabled)
{
    model().setLineCapture(enabled != 0);
}

extern "C" void lep_nurbs_set_line_tag(const char *label,
                                        int labelLength,
                                        int planIndex,
                                        int isBrake)
{
    model().setLineTag(label, labelLength, planIndex, isBrake != 0);
}

extern "C" void lep_nurbs_tag_diagonal(const char *kind,
                                        int kindLength,
                                        int index)
{
    model().tagDiagonal(kind, kindLength, index);
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
