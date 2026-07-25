#include "nurbs_model.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Builder.hxx>
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
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_ConstructionError.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
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
// Face boundaries are already exact B-spline edges. These sample counts add
// only a clear interior surface wireframe, without duplicating boundaries or
// bloating the STEP presentation.
constexpr int maximumDisplayedSpanSamples = 5;
constexpr int maximumDisplayedChordSamples = 10;

struct CapturedLine
{
    gp_Pnt start;
    gp_Pnt end;
    int colorIndex = 0;
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

class NurbsModel
{
public:
    void reset()
    {
        surfaces_.clear();
        surfaceSplines_.clear();
        capturedLines_.clear();
        errors_.clear();
        captureLines_ = false;
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
                  "upper");
        if (includeVentSurface) {
            addRegion(source,
                      panelIndex,
                      upperPointCount,
                      ventLast,
                      "vent");
        }
        if (!singleSkin && ventLast < totalPointCount) {
            addRegion(source,
                      panelIndex,
                      ventLast,
                      totalPointCount,
                      "lower");
        }
    }

    void captureLine(const CapturedLine &line)
    {
        if (!captureLines_
            || !isFinite(line.start)
            || !isFinite(line.end)
            || line.start.Distance(line.end) <= Precision::Confusion()) {
            return;
        }
        capturedLines_.push_back(line);
    }

    void setLineCapture(bool enabled)
    {
        captureLines_ = enabled;
    }

    lep::NurbsWriteResult writeStep(const std::filesystem::path &path)
    {
        lep::NurbsWriteResult result;
        result.surfaceCount = static_cast<int>(surfaces_.size());
        result.maximumSourceDeviationMillimetres =
            maximumSourceDeviation_;
        result.maximumLegacyAgreementMillimetres =
            maximumLegacyAgreement_;

        if (!errors_.empty()) {
            result.error = errors_.front();
            return result;
        }
        if (surfaces_.empty()) {
            result.error = "The calculation produced no NURBS surfaces.";
            return result;
        }

        try {
            BRep_Builder topologyBuilder;
            TopoDS_Compound model;
            TopoDS_Compound splines;
            topologyBuilder.MakeCompound(model);
            topologyBuilder.MakeCompound(splines);

            BRepBuilderAPI_Sewing sewing(
                pointToleranceMillimetres,
                true,
                true,
                true,
                false);
            for (const TopoDS_Face &surface : surfaces_) {
                sewing.Add(surface);
            }
            sewing.Perform();
            const TopoDS_Shape skins = sewing.SewedShape();
            if (skins.IsNull()
                || !BRepCheck_Analyzer(skins, true).IsValid()) {
                result.error =
                    "OCCT could not sew the NURBS faces into valid skin topology.";
                return result;
            }
            result.sewnEdgeCount = sewing.NbContigousEdges();
            result.freeEdgeCount = sewing.NbFreeEdges();

            for (const TopoDS_Edge &spline : surfaceSplines_) {
                topologyBuilder.Add(splines, spline);
            }

            const int capturedSplineCount =
                addCapturedLineSplines(topologyBuilder, splines);
            result.splineCount =
                static_cast<int>(surfaceSplines_.size())
                + capturedSplineCount;

            topologyBuilder.Add(model, skins);
            topologyBuilder.Add(model, splines);

            // AP242 preserves exact B-spline geometry and is the current STEP
            // schema. All model coordinates are already represented in mm.
            STEPControl_Writer writer;
            // STEPControl_Writer initializes the shared STEP parameters in
            // its constructor, so select the schema only after constructing
            // it and recreate the model with those parameters.
            Interface_Static::SetIVal("write.step.schema", 5); // AP242DIS
            Interface_Static::SetCVal("write.step.unit", "MM");
            Interface_Static::SetIVal("write.surfacecurve.mode", 1);
            writer.Model(true);
            writer.SetTolerance(pointToleranceMillimetres);
            if (writer.Transfer(model, STEPControl_AsIs)
                != IFSelect_RetDone) {
                result.error = "OCCT could not transfer the NURBS model to STEP.";
                return result;
            }

            const auto encoded = path.u8string();
            const std::string encodedPath{
                reinterpret_cast<const char *>(encoded.data()),
                encoded.size()};
            if (writer.Write(encodedPath.c_str()) != IFSelect_RetDone) {
                result.error = "OCCT could not write the STEP file.";
                return result;
            }
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

        result.success = true;
        return result;
    }

private:
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
                   const char *regionName)
    {
        if (lastPoint - firstPoint + 1 < 2) {
            return;
        }

        addSurface(source,
                   panelIndex,
                   firstPoint,
                   lastPoint,
                   regionName);
    }

    void addSurface(const SourcePanel &source,
                    int panelIndex,
                    int firstPoint,
                    int lastPoint,
                    const char *regionName)
    {
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
            surfaces_.push_back(face);

            BRepBuilderAPI_Transform mirror(
                face,
                mirrorTransform(),
                true);
            if (!mirror.IsDone()
                || mirror.Shape().ShapeType() != TopAbs_FACE) {
                errors_.push_back(
                    "Could not mirror the "
                    + std::string(regionName)
                    + " face for panel "
                    + std::to_string(panelIndex));
                surfaces_.pop_back();
                return;
            }
            surfaces_.push_back(TopoDS::Face(mirror.Shape()));
            addIsoparametricSplines(
                surface,
                maximumDisplayedSpanSamples,
                std::min(chordPointCount, maximumDisplayedChordSamples));
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
        const occ::handle<Geom_BSplineSurface> &surface,
        int spanPointCount,
        int chordPointCount)
    {
        double uFirst = 0.0;
        double uLast = 0.0;
        double vFirst = 0.0;
        double vLast = 0.0;
        surface->Bounds(uFirst, uLast, vFirst, vLast);

        auto addCurve = [this](const occ::handle<Geom_Curve> &curve) {
            if (curve.IsNull()) {
                return;
            }
            BRepBuilderAPI_MakeEdge makeEdge(curve);
            if (makeEdge.IsDone()) {
                const TopoDS_Edge edge = makeEdge.Edge();
                surfaceSplines_.push_back(edge);
                BRepBuilderAPI_Transform mirror(
                    edge,
                    mirrorTransform(),
                    true);
                if (mirror.IsDone()
                    && mirror.Shape().ShapeType() == TopAbs_EDGE) {
                    surfaceSplines_.push_back(
                        TopoDS::Edge(mirror.Shape()));
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
            addCurve(surface->UIso(parameter));
        }
        for (int index = 1; index + 1 < chordPointCount; ++index) {
            const double parameter =
                vFirst
                + (vLast - vFirst)
                      * static_cast<double>(index)
                      / static_cast<double>(chordPointCount - 1);
            addCurve(surface->VIso(parameter));
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

    int addCapturedLineSplines(BRep_Builder &builder, TopoDS_Compound &compound)
    {
        std::unordered_set<QuantizedSegment, QuantizedSegmentHash> added;
        int count = 0;

        for (const CapturedLine &line : capturedLines_) {
            const QuantizedSegment key =
                quantizeSegment(line.start, line.end);
            if (!added.insert(key).second) {
                continue;
            }

            BRepBuilderAPI_MakeEdge makeEdge(
                makeLinearSpline(line.start, line.end));
            if (makeEdge.IsDone()) {
                builder.Add(compound, makeEdge.Edge());
                ++count;
            }
        }
        return count;
    }

    std::vector<TopoDS_Face> surfaces_;
    std::vector<TopoDS_Edge> surfaceSplines_;
    std::vector<CapturedLine> capturedLines_;
    std::vector<std::string> errors_;
    bool captureLines_ = false;
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
