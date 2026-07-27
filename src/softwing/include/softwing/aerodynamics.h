#pragma once

#include "softwing/parallel.h"
#include "softwing/suspension.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace softwing {

inline constexpr std::string_view aerodynamicStage6Confidence =
    "verified reduced-order aerodynamics";
inline constexpr std::string_view aerodynamicStage6ScopeLimitations =
    "synthetic model-level evidence; no calibration, trim, flight prediction, "
    "atmospheric fields, collapse/reinflation, CFD, certification, or safety prediction";
inline constexpr std::string_view aerodynamicFrameTag = "SI_RH_ZUP";

enum class AerodynamicPhase {
    Parse,
    Validation,
    Registration,
    Kinematics,
    Influence,
    Circulation,
    Reconstruction,
    Transfer,
    StructuralAdvance,
    Wake,
    Separation,
    Certification,
    Diagnostics,
};

enum class AerodynamicValidity {
    Unregistered,
    Disabled,
    InactiveBelowSpeed,
    InactiveCollapsedCell,
    InsideDomain,
    OutsideDomain,
    Invalid,
    Indeterminate,
};

class AerodynamicError : public std::runtime_error {
public:
    AerodynamicError(AerodynamicPhase phase,
                     std::string entity,
                     const std::string& message,
                     double residual = 0.0,
                     double budget = 0.0,
                     AerodynamicValidity validity =
                         AerodynamicValidity::Indeterminate);
    [[nodiscard]] AerodynamicPhase phase() const { return phase_; }
    [[nodiscard]] const std::string& entity() const { return entity_; }
    [[nodiscard]] double residual() const { return residual_; }
    [[nodiscard]] double budget() const { return budget_; }
    [[nodiscard]] AerodynamicValidity validity() const { return validity_; }

private:
    AerodynamicPhase phase_;
    std::string entity_;
    double residual_ = 0.0;
    double budget_ = 0.0;
    AerodynamicValidity validity_ = AerodynamicValidity::Indeterminate;
};

enum class AerodynamicSurfaceSource { RigidStructured, Stage4Canopy };
enum class WakeConvectionMode { Uniform, UniformAndInduced };
enum class VortexValidity { Regular, Regularized, Indeterminate };
enum class DragActivity { Active, InactiveBelowSpeed, InactiveParallel, Disabled };

struct AerodynamicProvenance {
    std::string id;
    std::string source;
};

struct AerodynamicLatticeVertexDefinition {
    std::string id;
    std::size_t chordIndex = 0;
    std::size_t spanIndex = 0;
    Vec3 position;
};

struct AerodynamicLatticeDefinition {
    std::size_t chordSegments = 0;
    std::size_t spanSegments = 0;
    std::vector<AerodynamicLatticeVertexDefinition> vertices;
};

struct AerodynamicSolverSettings {
    double coreRadius = 1.0e-4;
    double pivotTolerance = 1.0e-12;
    double minimumPivotRatio = 1.0e-12;
    double maximumScaledResidual = 1.0e-9;
    std::size_t maximumPanels = 4096;
    // Maximum number of private fixed-point candidates evaluated per
    // certified window. The trial commits at the first candidate whose
    // response matches its input guess within maximumOuterResidual; a trial
    // that exhausts the maximum without converging is rejected. One keeps
    // the accepted single-candidate flow exactly.
    std::size_t outerIterations = 1;
    double maximumOuterResidual = 1.0e-6;
};

struct WakeSettings {
    bool enabled = true;
    WakeConvectionMode convection = WakeConvectionMode::UniformAndInduced;
    std::size_t maximumRows = 32;
    double maximumAge = 4.0;
    double farWakeLength = 40.0;
    double releaseOffsetFactor = 0.25;
    double kelvinTolerance = 1.0e-10;
};

struct SeparationSettings {
    bool enabled = false;
    bool symmetric = true;
    bool permitExtrapolation = false;
    double attachedAngle = 0.17453292519943295;
    double onsetAngle = 0.2617993877991494;
    double fullSeparationAngle = 0.5235987755982988;
    double minimumFraction = 0.15;
    double timeConstant = 0.08;
    double maximumValidityAngle = 0.6108652381980153;
    double maximumReducedFrequency = 0.25;
    double residualNormalFraction = 0.0;
};

struct LineDragDefinition {
    std::string id;
    std::string segmentId;
    double diameter = 0.0;
    double dragCoefficient = 0.0;
    std::string provenanceId;
};

struct PayloadDragDefinition {
    std::string id;
    std::string pointId;
    Vec3 localApplicationPoint;
    double referenceArea = 0.0;
    double dragCoefficient = 0.0;
    std::string provenanceId;
};

struct AerodynamicTransferSettings {
    double maximumAbsoluteWeight = 4.0;
    double positionTolerance = 1.0e-13;
    double forceMomentTolerance = 1.0e-12;
    double workTolerance = 1.0e-10;
    double finiteStateLimit = 1.0e12;
    double impulseTolerance = 1.0e-8;
    double angularImpulseTolerance = 1.0e-7;
    double energyTolerance = 1.0e-6;
};

struct AerodynamicDefinition {
    std::size_t schemaMajor = 1;
    std::string identifier;
    std::string description;
    std::string unitsFrameTag = std::string(aerodynamicFrameTag);
    std::vector<AerodynamicProvenance> provenance;
    Vec3 referenceOrigin;
    double airDensity = 1.225;
    Vec3 freestream{20.0, 0.0, 0.0};
    double referenceArea = 0.0;
    double referenceSpan = 0.0;
    double referenceChord = 0.0;
    double velocityFloor = 1.0e-6;
    Vec3 referenceChordDirection{1.0, 0.0, 0.0};
    Vec3 referenceSpanDirection{0.0, 1.0, 0.0};
    AerodynamicSurfaceSource surfaceSource =
        AerodynamicSurfaceSource::RigidStructured;
    AerodynamicLatticeDefinition rigidLattice;
    AerodynamicSolverSettings solver;
    WakeSettings wake;
    SeparationSettings separation;
    std::vector<LineDragDefinition> lineDrag;
    std::vector<PayloadDragDefinition> payloadDrag;
    AerodynamicTransferSettings transfer;
};

struct AerodynamicContributor {
    std::size_t nodeIndex = 0;
    double weight = 0.0;
    // Stable record-local material identity used by canonical diagnostics;
    // nodeIndex remains the owner-safe runtime lookup key only.
    std::string semanticId;
};

struct AerodynamicLatticeVertex {
    std::string id;
    std::size_t chordIndex = 0;
    std::size_t spanIndex = 0;
    Vec3 referencePosition;
    Vec3 position;
    Vec3 velocity;
    std::vector<AerodynamicContributor> contributors;
};

struct AerodynamicSkinMapping {
    std::string faceId;
    std::size_t triangle = 0;
    double sign = 0.0;
    std::array<std::string, 3> contributorIds;
};

struct AerodynamicLatticePanel {
    std::string id;
    std::optional<std::size_t> canopyCellIndex;
    std::string canopyCellId;
    std::size_t chordIndex = 0;
    std::size_t spanIndex = 0;
    std::array<std::size_t, 4> corners{};
    Vec3 boundStart;
    Vec3 boundEnd;
    Vec3 collocation;
    Vec3 applicationPoint;
    Vec3 normal;
    double area = 0.0;
    bool trailingEdge = false;
    AerodynamicValidity activity = AerodynamicValidity::InsideDomain;
    // Scale-free raw-geometry diagnostic. One is planar with consistent
    // winding; zero is a right-angle-or-sharper fold or an unusable split.
    double flatnessFactor = 1.0;
    std::string inactivityReason;
    std::vector<AerodynamicContributor> applicationContributors;
    std::vector<AerodynamicSkinMapping> skins;
};

struct AerodynamicCellActivityRecord {
    std::size_t cellIndex = 0;
    std::string cellId;
    AerodynamicValidity activity = AerodynamicValidity::InsideDomain;
    double flatnessFactor = 1.0;
    double minimumFlatness = 0.0;
    double normalizedVolume = 1.0;
    double normalizedSkinSeparation = 1.0;
    double gaugePressure = 0.0;
    double minimumNormalizedVolume = 0.0;
    double minimumNormalizedSkinSeparation = 0.0;
    double minimumGaugePressure = 0.0;
    std::string firstFailurePanelId;
    std::string reason;
    std::size_t panelCount = 0;
};

struct AerodynamicSurface {
    std::string id;
    const SoftBody* owner = nullptr;
    std::size_t ownerNodeCount = 0;
    std::size_t ownerTriangleCount = 0;
    std::uint64_t ownerTopologyFingerprint = 0;
    std::size_t chordSegments = 0;
    std::size_t spanSegments = 0;
    std::vector<AerodynamicLatticeVertex> vertices;
    std::vector<AerodynamicLatticePanel> panels;
    std::vector<AerodynamicCellActivityRecord> cellActivity;
};

struct VortexInfluenceResult {
    Vec3 velocity;
    VortexValidity validity = VortexValidity::Indeterminate;
    double distanceToSegment = 0.0;
};

struct DenseSolveDiagnostics {
    std::size_t rank = 0;
    std::size_t operationCount = 0;
    double minimumScaledPivot = 0.0;
    double pivotRatio = 0.0;
    double absoluteResidual = 0.0;
    double scaledResidual = 0.0;
    AerodynamicValidity validity = AerodynamicValidity::Indeterminate;
};

struct DenseSolveResult {
    std::vector<double> solution;
    DenseSolveDiagnostics diagnostics;
};

struct TransferDiagnostics {
    double weightSumError = 0.0;
    double positionError = 0.0;
    double forceError = 0.0;
    double momentError = 0.0;
    double powerError = 0.0;
    bool certified = false;
};

struct SeparationRecord {
    std::string panelId;
    double signedAngle = 0.0;
    double reducedFrequency = 0.0;
    double staticTarget = 1.0;
    double committedFraction = 1.0;
    double trialFraction = 1.0;
    double attenuation = 1.0;
    std::string branch;
    AerodynamicValidity validity = AerodynamicValidity::InsideDomain;
};

struct PanelAerodynamicRecord {
    std::string panelId;
    std::optional<std::size_t> canopyCellIndex;
    std::string canopyCellId;
    std::string inactivityReason;
    Vec3 boundStart;
    Vec3 boundEnd;
    Vec3 collocation;
    Vec3 applicationPoint;
    Vec3 normal;
    Vec3 relativeVelocity;
    double dynamicPressure = 0.0;
    double area = 0.0;
    // Cumulative bound circulation is the local potential jump used by the
    // unsteady Bernoulli term. Quasi-steady Kutta-Joukowski loading uses its
    // chordwise difference, retained separately for inspection.
    double circulation = 0.0;
    double priorCirculation = 0.0;
    double quasiSteadyCirculation = 0.0;
    double priorQuasiSteadyCirculation = 0.0;
    double boundInfluenceNormalVelocity = 0.0;
    double prescribedNormalVelocity = 0.0;
    double noPenetrationResidual = 0.0;
    std::size_t regularizedInfluenceCount = 0;
    double quasiSteadyPressureJump = 0.0;
    double unsteadyPressureJump = 0.0;
    double pressureJump = 0.0;
    Vec3 quasiSteadyForce;
    Vec3 unsteadyForce;
    Vec3 force;
    Vec3 moment;
    double power = 0.0;
    double lift = 0.0;
    double drag = 0.0;
    double side = 0.0;
    std::vector<AerodynamicContributor> contributors;
    TransferDiagnostics transfer;
    AerodynamicValidity validity = AerodynamicValidity::Indeterminate;
};

struct SkinPressureRecord {
    std::string panelId;
    std::string faceId;
    std::size_t triangle = 0;
    double forceShare = 0.0;
    double signedPressureJump = 0.0;
    Vec3 orientedArea;
    Vec3 applicationPoint;
    Vec3 force;
    Vec3 moment;
    double power = 0.0;
    std::vector<AerodynamicContributor> contributors;
    TransferDiagnostics transfer;
    AerodynamicValidity validity = AerodynamicValidity::Indeterminate;
};

struct InactiveFabricDragRecord {
    std::string id;
    std::size_t cellIndex = 0;
    std::string cellId;
    std::string faceId;
    std::size_t triangle = 0;
    double area = 0.0;
    Vec3 normal;
    Vec3 relativeVelocity;
    double normalVelocity = 0.0;
    Vec3 applicationPoint;
    Vec3 force;
    Vec3 moment;
    double power = 0.0;
    std::vector<AerodynamicContributor> contributors;
    TransferDiagnostics transfer;
    DragActivity activity = DragActivity::Disabled;
    std::string branch;
};

struct WakeVertexRecord {
    std::string id;
    Vec3 position;
    Vec3 convectionVelocity;
    std::size_t regularizedInfluenceCount = 0;
    std::size_t incidentExclusionCount = 0;
    VortexValidity convectionValidity = VortexValidity::Regular;
};

struct WakeRowRecord {
    std::size_t id = 0;
    double age = 0.0;
    // Each retained material strip is an oriented quadrilateral. `frontVertices`
    // is its released leading edge and `vertices` is its downstream edge.
    std::vector<WakeVertexRecord> frontVertices;
    std::vector<WakeVertexRecord> vertices;
    std::vector<double> stripCirculation;
};

struct LineDragRecord {
    std::string id;
    std::string segmentId;
    Vec3 firstPoint;
    Vec3 secondPoint;
    Vec3 relativeVelocity;
    Vec3 force;
    Vec3 firstForce;
    Vec3 secondForce;
    double projectedArea = 0.0;
    double power = 0.0;
    DragActivity activity = DragActivity::Disabled;
    std::string provenanceId;
};

struct PayloadDragRecord {
    std::string id;
    std::string pointId;
    Vec3 applicationPoint;
    Vec3 relativeVelocity;
    Vec3 force;
    Vec3 moment;
    double power = 0.0;
    DragActivity activity = DragActivity::Disabled;
    std::string provenanceId;
};

struct AerodynamicDiagnostics {
    bool registered = false;
    bool converged = false;
    bool failedTrial = false;
    AerodynamicValidity validity = AerodynamicValidity::Unregistered;
    std::size_t sequence = 0;
    std::size_t panelCount = 0;
    std::size_t activePanelCount = 0;
    std::size_t collapsedPanelCount = 0;
    std::size_t activeCellCount = 0;
    std::size_t collapsedCellCount = 0;
    std::size_t wakeRowCount = 0;
    std::size_t wakeVertexCount = 0;
    std::size_t regularizedInfluenceCount = 0;
    DenseSolveDiagnostics solver;
    std::size_t outerIterationCount = 1;
    double outerResidual = 0.0;
    double outerResidualBudget = 0.0;
    bool outerConverged = true;
    std::string outerResidualEntity = "none";
    Vec3 referenceOrigin;
    Vec3 referenceChordDirection;
    Vec3 referenceSpanDirection;
    Vec3 currentChordAxis;
    Vec3 currentSpanAxis;
    Vec3 liftAxis;
    Vec3 dragAxis;
    Vec3 freestream;
    double airDensity = 0.0;
    double referenceArea = 0.0;
    double referenceSpan = 0.0;
    double referenceChord = 0.0;
    double velocityFloor = 0.0;
    double referenceDynamicPressure = 0.0;
    // Final reconstructed real-skin pressure-load aggregate. This closes the
    // committed PanelAerodynamicRecord entries and is the load transferred to
    // the structural owner.
    Vec3 canopyForce;
    // Raw mean-lattice source diagnostics. These independently expose the
    // Kutta-Joukowski and rho*dGamma/dt source model and are intentionally not
    // asserted equal to the final real-skin pressure-load aggregate above.
    Vec3 meanLatticeCanopyForce;
    Vec3 meanLatticeQuasiSteadyForce;
    Vec3 meanLatticeUnsteadyForce;
    Vec3 inactiveFabricForce;
    Vec3 lineDragForce;
    Vec3 payloadDragForce;
    Vec3 totalForce;
    // Final reconstructed real-skin PanelAerodynamicRecord moment aggregate.
    Vec3 canopyMoment;
    Vec3 meanLatticeCanopyMoment;
    Vec3 inactiveFabricMoment;
    Vec3 lineDragMoment;
    Vec3 payloadDragMoment;
    Vec3 totalMoment;
    // Final reconstructed real-skin PanelAerodynamicRecord power aggregate.
    double canopyPower = 0.0;
    double meanLatticeCanopyPower = 0.0;
    double inactiveFabricPower = 0.0;
    double lineDragPower = 0.0;
    double payloadDragPower = 0.0;
    double totalPower = 0.0;
    double liftCoefficient = 0.0;
    double inducedDragCoefficient = 0.0;
    double sideCoefficient = 0.0;
    double kelvinResidual = 0.0;
    double farWakeCirculation = 0.0;
    std::vector<double> kelvinResidualByStrip;
    std::vector<double> farWakeCirculationByStrip;
    TransferDiagnostics transfer;
    double externalImpulseResidual = 0.0;
    double externalMomentResidual = 0.0;
    double workResidual = 0.0;
    double freestreamReservoirWork = 0.0;
    double circulationEnergyProxyBefore = 0.0;
    double circulationEnergyProxyAfter = 0.0;
    double wakeEnergyProxyWork = 0.0;
    double circulationProxyDiscrepancy = 0.0;
    double coupledMechanicalEnergyBefore = 0.0;
    double coupledMechanicalEnergyAfter = 0.0;
    double aerodynamicDiscreteWork = 0.0;
    double nonAerodynamicDiscreteWork = 0.0;
    double suspensionDampingWork = 0.0;
    double contactDissipativeWork = 0.0;
    double dissipativeEnergyCreation = 0.0;
    double signedMechanicalEnergyClosure = 0.0;
    double unexplainedEnergyCreation = 0.0;
    std::string circulationEnergyProxyProvenance;
    double maximumMaterialInterpolationError = 0.0;
    std::size_t operationCount = 0;
    std::string provenance;
    AerodynamicPhase failurePhase = AerodynamicPhase::Diagnostics;
    std::string failureEntity;
};

struct AerodynamicSnapshot {
    std::vector<double> circulation;
    std::vector<double> priorCirculation;
    std::vector<double> separationFraction;
    std::vector<double> signedAngle;
    std::vector<WakeRowRecord> wakeRows;
    double farWakeCirculation = 0.0;
    std::vector<double> farWakeCirculationByStrip;
    std::size_t nextWakeRowId = 0;
    std::vector<PanelAerodynamicRecord> panels;
    std::vector<AerodynamicCellActivityRecord> cellActivity;
    std::vector<SkinPressureRecord> skinPressure;
    std::vector<InactiveFabricDragRecord> inactiveFabricDrag;
    std::vector<SeparationRecord> separation;
    std::vector<LineDragRecord> lineDrag;
    std::vector<PayloadDragRecord> payloadDrag;
    AerodynamicDiagnostics diagnostics;
};

struct AerodynamicFailure {
    AerodynamicPhase phase = AerodynamicPhase::Diagnostics;
    std::string entity;
    AerodynamicValidity validity = AerodynamicValidity::Indeterminate;
    double residual = 0.0;
    double budget = 0.0;
    bool rollbackComplete = false;
};

[[nodiscard]] const char* aerodynamicPhaseName(AerodynamicPhase phase);
[[nodiscard]] const char* aerodynamicValidityName(AerodynamicValidity validity);
[[nodiscard]] AerodynamicDefinition validateAndNormalizeAerodynamicDefinition(
    const AerodynamicDefinition& definition);
[[nodiscard]] std::string serializeAerodynamicDefinition(
    const AerodynamicDefinition& definition);
[[nodiscard]] AerodynamicDefinition parseAerodynamicDefinition(
    std::string_view text);
[[nodiscard]] AerodynamicDefinition makeRectangularWingAerodynamicDefinition(
    std::size_t chordSegments = 4,
    std::size_t spanSegments = 12);
[[nodiscard]] AerodynamicDefinition makeStage4CanopyAerodynamicDefinition(
    const CanopyMesh& canopy);

[[nodiscard]] AerodynamicSurface buildRigidAerodynamicSurface(
    const AerodynamicDefinition& definition);
struct CanopyAerodynamicSampling {
    // Runtime adapter sampling only. One preserves the canonical full semantic
    // lattice; larger values retain the exact leading/trailing coordinates
    // and every span station while grouping chordwise skin faces.
    std::size_t chordCoordinateStride = 1;
};
[[nodiscard]] AerodynamicSurface buildCanopyAerodynamicSurface(
    const CanopyMesh& canopy,
    const AerodynamicDefinition& definition,
    const CanopyAerodynamicSampling& sampling = {});
[[nodiscard]] std::uint64_t aerodynamicTopologyFingerprint(
    const SoftBody& body);
[[nodiscard]] double aerodynamicQuadFlatness(
    const std::array<Vec3, 4>& corners);
void gatherAerodynamicSurface(AerodynamicSurface& surface,
                              const SoftBody* owner = nullptr,
                              double minimumCellFlatness = 0.0);

[[nodiscard]] VortexInfluenceResult finiteVortexInducedVelocity(
    const Vec3& first,
    const Vec3& second,
    const Vec3& query,
    double coreRadius,
    double exclusionDistance = 1.0e-14);
[[nodiscard]] VortexInfluenceResult evaluateWakeInducedVelocity(
    std::span<const WakeRowRecord> rows,
    const Vec3& query,
    double coreRadius);
[[nodiscard]] DenseSolveResult solveAerodynamicDenseSystem(
    std::span<const double> matrix,
    std::span<const double> rightHandSide,
    std::size_t size,
    const AerodynamicSolverSettings& settings);

void certifyNonnegativeSettledInducedDrag(
    const Vec3& canopyForce,
    const Vec3& dragAxis,
    double referenceForce,
    double scaledTolerance);

[[nodiscard]] double separationStaticFraction(
    const SeparationSettings& settings,
    double signedAngle);
[[nodiscard]] const char* separationBranchName(
    const SeparationSettings& settings,
    double signedAngle);
[[nodiscard]] double advanceSeparationFraction(
    const SeparationSettings& settings,
    double committedFraction,
    double targetFraction,
    double timeStep);

[[nodiscard]] InactiveFabricDragRecord evaluateInactiveFabricNormalDrag(
    std::string id,
    std::size_t cellIndex,
    std::string cellId,
    std::string faceId,
    std::size_t triangle,
    const std::array<Vec3, 3>& positions,
    const std::array<Vec3, 3>& velocities,
    const std::array<std::size_t, 3>& nodeIndices,
    const std::array<std::string, 3>& contributorIds,
    const Vec3& freestream,
    double airDensity,
    double normalDragCoefficient,
    const Vec3& referenceOrigin = {});

[[nodiscard]] LineDragRecord evaluateLineDrag(
    const LineDragDefinition& definition,
    double airDensity,
    const Vec3& firstPoint,
    const Vec3& secondPoint,
    const Vec3& firstVelocity,
    const Vec3& secondVelocity,
    const Vec3& freestream,
    double velocityFloor);
[[nodiscard]] PayloadDragRecord evaluatePayloadDrag(
    const PayloadDragDefinition& definition,
    double airDensity,
    const RigidPayloadDefinition& payloadDefinition,
    const RigidPayloadState& payloadState,
    const Vec3& freestream,
    double velocityFloor,
    const Vec3& referenceOrigin = {});
[[nodiscard]] TransferDiagnostics certifyAndApplyPanelLoad(
    SoftBody& body,
    const AerodynamicSurface& surface,
    const PanelAerodynamicRecord& record,
    const AerodynamicTransferSettings& settings,
    const Vec3& origin = {});

// Opt-in Stage 6 reduced-order aerodynamic owner. Successful output is
// synthetic/model-level only; this type does not perform trim, flight-state,
// environmental-field, collapse, calibration, or safety calculations.
class AerodynamicSystem {
public:
    static AerodynamicSystem buildRigid(
        const AerodynamicDefinition& definition);
    static AerodynamicSystem buildCanopy(
        CanopyMesh& canopy,
        const AerodynamicDefinition& definition,
        const CanopyAerodynamicSampling& sampling = {});
    // buildCanopy borrows the registered CanopyMesh/SoftBody. That owner must
    // outlive this system and remain at the same address for the registration
    // lifetime; topology changes are rejected separately.

    AerodynamicSystem(AerodynamicSystem&&) noexcept = default;
    AerodynamicSystem& operator=(AerodynamicSystem&&) noexcept = default;
    AerodynamicSystem(const AerodynamicSystem&) = delete;
    AerodynamicSystem& operator=(const AerodynamicSystem&) = delete;

    [[nodiscard]] const AerodynamicDefinition& definition() const {
        return definition_;
    }
    [[nodiscard]] const AerodynamicSurface& surface() const { return surface_; }
    [[nodiscard]] const AerodynamicSnapshot& state() const { return state_; }
    [[nodiscard]] const AerodynamicDiagnostics& diagnostics() const {
        return state_.diagnostics;
    }
    [[nodiscard]] const AerodynamicDiagnostics& committedDiagnostics() const {
        return committedDiagnostics_;
    }
    [[nodiscard]] const std::optional<AerodynamicFailure>& lastFailure() const {
        return lastFailure_;
    }
    [[nodiscard]] SoftBody* owner() const { return owner_; }

    void setRigidKinematics(const Vec3& translation,
                            const Quaternion& rotation,
                            const Vec3& linearVelocity,
                            const Vec3& angularVelocity);
    void setFreestream(const Vec3& freestream);
    [[nodiscard]] const Vec3& freestream() const { return freestream_; }
    void step(double timeStep);
    void stepCoupled(const StepSettings& settings,
                     SuspensionSystem* suspension = nullptr,
                     PneumaticNetwork* pneumatics = nullptr);
    // Advances the same certified aerodynamic/structural transaction while
    // applying normal drag to every mapped canopy skin face and suppressing
    // lifting-sheet and wake evolution. This is the explicit wind-loading
    // phase used while an open ram-air canopy is still preparing for the
    // one-time lifting handoff.
    void stepInactiveFabricCoupled(
        const StepSettings& settings,
        SuspensionSystem* suspension = nullptr,
        PneumaticNetwork* pneumatics = nullptr);
    // Samples the complete cell-local aerodynamic load once for the declared
    // application interval and holds that certified load while the coupled
    // structural/pneumatic owners take their requested inner substeps. Wake
    // and separation still advance exactly once per application interval.
    void stepHeldCoupled(
        const StepSettings& settings,
        SuspensionSystem* suspension = nullptr,
        PneumaticNetwork* pneumatics = nullptr);
    void injectFailure(AerodynamicPhase phase);
    void clearInjectedFailure();
    [[nodiscard]] std::string canonicalState() const;

    // Worker count for the wake-convection and influence-assembly sweeps.
    //
    // Unlike StepSettings::workerThreads, this never selects physics: every
    // parallelised sweep writes only slots its own index owns, and no
    // floating-point accumulation crosses an iteration, so the result is
    // bit-identical for every count including 0. The count buys speed only,
    // which is why it is safe to raise on one machine and not another.
    //
    // It stays off the definition on purpose -- the definition is serialised
    // into the canonical bytes the acceptance gates compare, and a machine's
    // worker count has no business in a physics record.
    void setWorkerThreads(unsigned workerThreads);
    [[nodiscard]] unsigned workerThreads() const { return workerThreads_; }

    // Exploratory raw-sheet gate. Zero preserves the strict geometry-only
    // classifier. This setting never edits the lattice; it is applied on the
    // next gather and only masks whole canopy cells from the lifting solve.
    void setMinimumCellFlatness(double minimumFlatness);
    [[nodiscard]] double minimumCellFlatness() const {
        return minimumCellFlatness_;
    }
    void setCellInflationThresholds(double minimumNormalizedVolume,
                                    double minimumNormalizedSkinSeparation,
                                    double minimumGaugePressure);
    void setInactiveFabricNormalDragCoefficient(double coefficient);
    [[nodiscard]] double inactiveFabricNormalDragCoefficient() const {
        return inactiveFabricNormalDragCoefficient_;
    }
    // Refreshes live canopy geometry and cell-inflation activity without
    // solving or advancing time. Intended for inspection immediately after
    // a threshold or prescribed pneumatic boundary change.
    void refreshCanopyActivity(
        const PneumaticNetwork* pneumatics = nullptr);

    // Applied to every system built after the call, so a run can opt in once
    // instead of at each of its build sites. Defaults to 0, so nothing
    // changes for a caller that never asks.
    //
    // The core still never samples the machine -- an app that wants all cores
    // passes hardwareWorkerCount() in explicitly.
    static void setDefaultWorkerThreads(unsigned workerThreads);
    [[nodiscard]] static unsigned defaultWorkerThreads();

private:
    // Accepted Stage 7 flight-state restart module (production friend, not
    // test access); it restores committed advance state into this live
    // registered owner without changing registration or identity tokens.
    friend struct FlightStateAccess;
#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
    friend struct AerodynamicVerificationTestAccess;
#endif
    AerodynamicSystem() = default;
    void gather(const PneumaticNetwork* pneumatics = nullptr);
    [[nodiscard]] AerodynamicSnapshot solveTrial(
        double timeStep,
        bool evolveState = true,
        bool forceInactiveFabric = false) const;
    void stepCoupledImpl(const StepSettings& settings,
                         SuspensionSystem* suspension,
                         PneumaticNetwork* pneumatics,
                         bool forceInactiveFabric,
                         bool allowStructuralSubsteps);
    void evolveWakeTrial(AerodynamicSnapshot& trial,
                         double timeStep,
                         const Vec3& downstream) const;
    void evolveSeparationTrial(AerodynamicSnapshot& trial,
                               double timeStep) const;
    // Null below 2 workers, so the serial path stays literally the serial
    // path rather than a one-worker dispatch. Both produce the same bytes;
    // this only avoids paying for a pool nobody asked for.
    //
    // The pool itself is shared process-wide rather than owned per system:
    // a run builds many systems, the pool's workers spin while idle, and one
    // set of spinning workers per live system oversubscribes the machine
    // badly enough to eat most of the speedup. Sweeps never nest, and nothing
    // steps two systems concurrently -- that is the assumption this shares on.
    [[nodiscard]] WorkerPool* sweepPool() const;
    void recordFailure(AerodynamicPhase phase,
                       std::string entity,
                       double residual = 0.0,
                       double budget = 0.0,
                       AerodynamicValidity validity =
                           AerodynamicValidity::Indeterminate) noexcept;

    AerodynamicDefinition definition_;
    Vec3 freestream_;
    AerodynamicSurface surface_;
    SoftBody* owner_ = nullptr;
    CanopyMesh* canopy_ = nullptr;
    AerodynamicSnapshot state_;
    AerodynamicDiagnostics committedDiagnostics_;
    std::optional<AerodynamicFailure> lastFailure_;
    std::optional<AerodynamicPhase> injectedFailure_;
    bool injectNonFinitePayloadAfterAdvance_ = false;
    bool injectImpulseLedgerMismatch_ = false;
    bool injectMechanicalEnergyLedgerMismatch_ = false;
    mutable std::size_t solveTrialInvocationCount_ = 0;
    unsigned workerThreads_ = defaultWorkerThreads();
    double minimumCellFlatness_ = 0.0;
    double minimumNormalizedCellVolume_ = 0.0;
    double minimumNormalizedSkinSeparation_ = 0.0;
    double minimumCellGaugePressure_ = 0.0;
    double inactiveFabricNormalDragCoefficient_ = 0.0;
    std::vector<double> referenceCellVolumes_;
    std::vector<double> referenceCellMinimumSeparations_;
    std::vector<std::vector<std::array<std::size_t, 2>>>
        cellSkinSeparationSamples_;
    std::shared_ptr<void> registrationToken_;
    // Optional coupled owners become sticky only after their first successful
    // coherent commit. Weak logical-owner identities reject omission,
    // substitution, destruction, and same-address replacement before any
    // owner is dereferenced. Moving the same logical owner preserves identity.
    // Cell-activity partition frozen for the duration of one certified
    // coupled window (cell index -> first failing panel id, reason). Set by
    // stepCoupledImpl after the window's first gather and cleared on every
    // exit; empty outside a coupled window.
    std::optional<std::map<std::size_t,
                           std::pair<std::string, std::string>>>
        frozenWindowActivity_;
    std::weak_ptr<void> registeredSuspensionToken_;
    std::weak_ptr<void> registeredPneumaticToken_;
    bool suspensionAttached_ = false;
    bool pneumaticsAttached_ = false;
    Vec3 rigidTranslation_;
    Quaternion rigidRotation_;
    Vec3 rigidLinearVelocity_;
    Vec3 rigidAngularVelocity_;
};

} // namespace softwing
