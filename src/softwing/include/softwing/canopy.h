#pragma once

#include "softwing/pneumatics.h"

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace softwing {

inline constexpr std::string_view canopyStage4ScopeLimitations =
    "no suspension, payload, controls, aerodynamics, lift/drag, flight, collapse, "
    "geometry-derived occlusion, or commercial-wing fidelity";

enum class CanopyPhase {
    Parse,
    Validation,
    Mapping,
    Topology,
    Material,
    Envelope,
    Pneumatic,
};

class CanopyError : public std::runtime_error {
public:
    CanopyError(CanopyPhase phase,
                std::string entity,
                const std::string& message);

    [[nodiscard]] CanopyPhase phase() const { return phase_; }
    [[nodiscard]] const std::string& entity() const { return entity_; }

private:
    CanopyPhase phase_;
    std::string entity_;
};

enum class CanopyPanelRole {
    UpperSkin = 0,
    LowerSkin = 1,
    LeadingEdge = 2,
    // Wire value 3 was the removed fictitious trailing-edge fabric panel.
    // It stays unassigned so old canonical input fails instead of silently
    // becoming another physical role.
    Rib = 4,
    Diagonal = 5,
};

inline constexpr std::size_t canopyPanelRoleCount = 5;

enum class CanopyApertureKind {
    Inlet,
    CrossPort,
};

enum class CanopySeamKind {
    SkinRib,
    SkinEdge,
    RibEdge,
    DiagonalAttachment,
};

enum class CanopyMaterialBoundaryKind {
    DiagonalChordEnd,
};

enum class CanopyValueProvenance {
    Prescribed,
    Generated,
    Derived,
    Virtual,
    Residual,
};

struct CanopyProvenance {
    std::string id;
    std::string source;
};

struct RibSectionSample {
    double u = 0.0;
    double upper = 0.0;
    double lower = 0.0;
};

struct CanopySpanStation {
    std::string id;
    double y = 0.0;
    double leadingEdgeX = 0.0;
    double arcZ = 0.0;
    double chord = 0.0;
    // Version-2-only rotation of the section plane about nominal +X.  Zero
    // retains the byte-for-byte and numeric version-1 mapping.
    double sectionRollRadians = 0.0;
    double twistRadians = 0.0;
    std::vector<RibSectionSample> section;
};

inline constexpr double canopySectionRollLimitRadians =
    1.57079632679489661923;

struct CanopyStationFrame {
    Vec3 chord;
    Vec3 span;
    Vec3 up;
};

struct CanopyMaterial {
    std::string id;
    double arealDensity = 0.0;
    OrthotropicMembraneMaterial membrane;
    std::string provenanceId;
};

struct CanopyPanelAssignment {
    CanopyPanelRole role = CanopyPanelRole::UpperSkin;
    std::string materialId;
};

struct CanopyApertureDefinition {
    std::string id;
    CanopyApertureKind kind = CanopyApertureKind::Inlet;
    std::size_t cellIndex = 0;
    std::size_t ribIndex = 0;
    double firstMin = 0.0;
    double firstMax = 0.0;
    double secondMin = 0.0;
    double secondMax = 0.0;
    std::string provenanceId;
};

struct CanopyDiagonalDefinition {
    std::string id;
    std::size_t cellIndex = 0;
    double chordMin = 0.0;
    double chordMax = 0.0;
    bool lowerOnFirstRib = true;
    std::string materialId;
    std::string provenanceId;
};

struct CanopyMeshSettings {
    std::size_t chordSubdivisions = 2;
    std::size_t spanSubdivisionsPerCell = 2;
    std::size_t thicknessSubdivisions = 2;
    double coordinateTolerance = 1.0e-12;
    double volumeFloor = 1.0e-12;
};

struct CanopyDefinition {
    std::size_t schemaMajor = 1;
    std::string identifier;
    std::string description;
    std::string unitsFrameTag;
    std::vector<CanopyProvenance> provenance;
    std::size_t cellCount = 0;
    std::vector<CanopySpanStation> stations;
    std::vector<CanopyMaterial> materials;
    std::vector<CanopyPanelAssignment> assignments;
    std::vector<CanopyApertureDefinition> apertures;
    std::vector<CanopyDiagonalDefinition> diagonals;
    CanopyMeshSettings mesh;
};

struct CanopyPanelRecord {
    std::string id;
    CanopyPanelRole role = CanopyPanelRole::UpperSkin;
    std::optional<std::size_t> cellIndex;
    std::optional<std::size_t> ribIndex;
    std::string materialId;
    std::string provenanceId;
    std::vector<std::size_t> triangles;
};

struct CanopyFaceRecord {
    std::size_t triangle = 0;
    std::string semanticId;
    std::string panelId;
    CanopyPanelRole panelRole = CanopyPanelRole::UpperSkin;
    std::array<int, 2> adjacentCells{-1, -1};
    std::string materialId;
    MaterialRole materialRole = MaterialRole::Bulk;
    std::array<Vec2, 3> chart{};
    bool virtualClosure = false;
    std::string apertureId;
    std::string provenanceId;
};

struct CanopyApertureRecord {
    std::string id;
    CanopyApertureKind kind = CanopyApertureKind::Inlet;
    std::array<int, 2> adjacentZones{-1, -1};
    double area = 0.0;
    Vec3 centroid;
    Vec3 normal;
    std::array<std::size_t, 4> orderedLipNodes{};
    std::vector<std::size_t> orderedBoundaryNodes;
    std::array<std::size_t, 2> closureTriangles{};
    std::string provenanceId;
    CanopyValueProvenance areaProvenance =
        CanopyValueProvenance::Derived;
};

struct CanopySeamRecord {
    std::string id;
    std::string firstPanelId;
    std::string secondPanelId;
    CanopySeamKind kind = CanopySeamKind::SkinRib;
    Vec3 referenceDirection;
    std::vector<std::size_t> orderedNodes;
    std::string provenanceId;
};

struct CanopyMaterialBoundaryRecord {
    std::string id;
    std::string panelId;
    CanopyMaterialBoundaryKind kind =
        CanopyMaterialBoundaryKind::DiagonalChordEnd;
    std::vector<std::size_t> orderedNodes;
    std::string provenanceId;
};

struct CanopyBoundaryFace {
    std::size_t triangle = 0;
    int orientation = 1;
    std::string semanticId;
    bool virtualClosure = false;
};

struct CanopyCellDiagnostics {
    SurfaceTopologyReport topology;
    double signedVolume = 0.0;
    Vec3 centroid;
    Vec3 unitPressureForce;
    Vec3 unitPressureMoment;
    double materialMass = 0.0;
    CanopyValueProvenance volumeProvenance = CanopyValueProvenance::Derived;
    CanopyValueProvenance closureProvenance = CanopyValueProvenance::Residual;
};

struct CanopyCellRecord {
    std::string id;
    std::size_t cellIndex = 0;
    std::vector<CanopyBoundaryFace> boundary;
    CanopyCellDiagnostics diagnostics;
    std::string provenanceId;
};

struct CanopyAudit {
    std::size_t unclassifiedStructuralFaces = 0;
    std::size_t multiplyClassifiedStructuralFaces = 0;
    std::size_t undeclaredBoundaryEdges = 0;
    std::size_t undeclaredNonManifoldEdges = 0;
    std::size_t degenerateFaces = 0;
    std::size_t coincidentDuplicateSeamNodes = 0;

    [[nodiscard]] bool valid() const;
};

struct CanopyMesh {
    CanopyDefinition definition;
    SoftBody body;
    std::vector<CanopyPanelRecord> panels;
    std::vector<CanopyFaceRecord> faces;
    std::vector<CanopyApertureRecord> apertures;
    std::vector<CanopySeamRecord> seams;
    std::vector<CanopyMaterialBoundaryRecord> materialBoundaries;
    std::vector<CanopyCellRecord> cells;
    std::vector<std::size_t> structuralTriangles;
    std::vector<std::size_t> virtualTriangles;
    std::vector<std::size_t> fabricContactTriangles;
    CanopyAudit audit;
    double materialArea = 0.0;
    double materialMass = 0.0;
    Vec3 centreOfMass;
    std::array<double, canopyPanelRoleCount> areaByRole{};
    std::array<double, 3> areaByMaterialRole{};
};

struct CanopyPneumaticInputs {
    double ambientPressure = 101325.0;
    double supplyPressure = 101325.0;
    double temperature = standardFixtureTemperature;
    double inletDischargeCoefficient = 0.8;
    double crossPortMassConductance = 1.0e-7;
    double openingFraction = 0.0;
};

struct CanopyPneumaticPortRecord {
    std::string id;
    CanopyApertureKind kind = CanopyApertureKind::Inlet;
    std::array<int, 2> adjacentZones{-1, -1};
    double area = 0.0;
    Vec3 centroid;
    Vec3 normal;
    std::array<std::size_t, 2> closureTriangles{};
    std::size_t networkPortIndex = 0;
    double prescribedCoefficient = 0.0;
    double prescribedOpening = 0.0;
    std::string provenanceId;
    std::string limitation;
};

struct CanopyPneumaticLayout {
    std::unique_ptr<PneumaticNetwork> network;
    std::vector<std::size_t> cellZoneIndices;
    std::vector<std::size_t> pressureInterfaceIndices;
    std::vector<CanopyPneumaticPortRecord> ports;
    std::size_t ambientReservoirIndex = 0;
    std::size_t supplyReservoirIndex = 0;
    CanopyPneumaticInputs inputs;
    double massResidual = 0.0;
    std::string limitation;
};

[[nodiscard]] const char* canopyPhaseName(CanopyPhase phase);
[[nodiscard]] const char* canopyPanelRoleName(CanopyPanelRole role);
[[nodiscard]] std::size_t canopyPanelRoleIndex(CanopyPanelRole role);
[[nodiscard]] const char* canopyApertureKindName(CanopyApertureKind kind);
[[nodiscard]] const char* canopyProvenanceName(CanopyValueProvenance value);

[[nodiscard]] CanopyDefinition validateAndNormalizeCanopyDefinition(
    const CanopyDefinition& definition);
[[nodiscard]] Vec3 mapCanopyStationPoint(const CanopySpanStation& station,
                                         double u,
                                         double zeta);
[[nodiscard]] CanopyStationFrame canopyStationFrame(
    const CanopySpanStation& station);
[[nodiscard]] std::string serializeCanopyDefinition(
    const CanopyDefinition& definition);
[[nodiscard]] CanopyDefinition parseCanopyDefinition(std::string_view text);
[[nodiscard]] CanopyDefinition makeStraightThreeCellDefinition();
[[nodiscard]] CanopyMesh buildCanopy(const CanopyDefinition& definition);
[[nodiscard]] CanopyPneumaticLayout buildCanopyPneumaticLayout(
    const CanopyMesh& mesh,
    const CanopyPneumaticInputs& inputs = {});

// How one face is signed when every cell sits at the same gauge and the
// exterior at zero. A face's stored winding points out of adjacentCells[0]
// and into adjacentCells[1], so the difference across it is the pressure on
// its positive side minus the pressure on its negative side: +1 for exterior
// skin wound outwards, -1 for exterior skin wound inwards, and exactly 0
// wherever both sides see the same pressure.
//
// That zero is the whole point. Interior partitions -- ribs and cross-port
// closures between two equally inflated cells -- cancel, and diagonal webs,
// which the pneumatic layout pins to {-1, -1} because both their faces are
// the same cell's interior, cancel with them. Sign a partition one-sidedly
// and every rib pushes the same way, giving the canopy a body-fixed thrust
// of order pressure * ribArea * ribCount plus its moment.
//
// Both prescribed-gauge paths share this rule: setUniformCellPressure() below
// stamps it onto the soft body's per-face pressure field, and the trim
// fixture's applyTrimPrescribedGaugeForces() integrates it into explicit
// nodal external forces.
[[nodiscard]] double canopyFaceGaugeSign(const CanopyFaceRecord& face);

// Inflate every cell to the same gauge pressure without the lumped pneumatic
// network -- the cheap prescribed-pressure path the studio and the probes use.
// Face-aware via canopyFaceGaugeSign(), unlike
// SoftBody::setUniformPressureDifference.
void setUniformCellPressure(CanopyMesh& mesh, double pressure);

} // namespace softwing
