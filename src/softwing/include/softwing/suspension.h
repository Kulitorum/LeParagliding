#pragma once

#include "softwing/canopy.h"

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

inline constexpr std::string_view suspensionStage5Confidence =
    "verified synthetic suspension and payload dynamics";
inline constexpr std::string_view suspensionStage5ScopeLimitations =
    "no calibrated line/harness behavior, airflow, line drag, lift/drag, trim, "
    "launch, flight, collapse, recovery, or safety prediction";
inline constexpr std::string_view suspensionStage5FrameTag =
    "SI right-handed +X chord +Y span +Z up";

enum class SuspensionPhase {
    Parse,
    Validation,
    AttachmentResolution,
    GraphConstruction,
    Control,
    Prediction,
    LineSolve,
    GroundSolve,
    Certification,
    Diagnostics,
};

class SuspensionError : public std::runtime_error {
public:
    SuspensionError(SuspensionPhase phase,
                    std::string entity,
                    const std::string& message);

    [[nodiscard]] SuspensionPhase phase() const { return phase_; }
    [[nodiscard]] const std::string& entity() const { return entity_; }

private:
    SuspensionPhase phase_;
    std::string entity_;
};

struct Mat3 {
    std::array<double, 9> values{1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0};

    [[nodiscard]] double& operator()(std::size_t row, std::size_t column) {
        return values[row * 3 + column];
    }
    [[nodiscard]] double operator()(std::size_t row,
                                    std::size_t column) const {
        return values[row * 3 + column];
    }
};

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

[[nodiscard]] Mat3 transpose(const Mat3& matrix);
[[nodiscard]] double determinant(const Mat3& matrix);
[[nodiscard]] Mat3 inverse(const Mat3& matrix);
[[nodiscard]] Vec3 operator*(const Mat3& matrix, const Vec3& vector);
[[nodiscard]] Mat3 operator*(const Mat3& first, const Mat3& second);
[[nodiscard]] Mat3 rotationMatrix(const Quaternion& orientation);
[[nodiscard]] Quaternion normalizedCanonical(const Quaternion& orientation);
[[nodiscard]] Quaternion operator*(const Quaternion& first,
                                   const Quaternion& second);
[[nodiscard]] Vec3 rotate(const Quaternion& orientation, const Vec3& vector);
[[nodiscard]] Quaternion rotationIncrement(const Vec3& rotationVector);

enum class SuspensionSide { Left, Right, Centre };
enum class SuspensionEndpointKind { Attachment, Junction, HangPoint };
enum class SuspensionControlKind {
    Brake,
    Accelerator,
    Riser,
    BigEar,
    WeightShift,
};
enum class SuspensionControlTargetKind { SegmentRestLength, HangPointTravel };
enum class PayloadGroundMode { Free, Anchored, SupportPlane };
enum class SuspensionProvenanceClass {
    Prescribed,
    Resolved,
    Solved,
    Diagnostic,
};

struct SuspensionProvenance {
    std::string id;
    std::string source;
};

struct SuspensionAttachmentDefinition {
    std::string id;
    std::string panelId;
    Vec2 chart;
    SuspensionSide side = SuspensionSide::Centre;
    std::string seamId;
    std::string provenanceId;
};

struct SuspensionJunctionDefinition {
    std::string id;
    Vec3 initialWorldPosition;
    double mass = 0.0;
    SuspensionSide side = SuspensionSide::Centre;
    std::string provenanceId;
};

struct PayloadPointDefinition {
    std::string id;
    Vec3 localPosition;
    SuspensionSide side = SuspensionSide::Centre;
    std::string provenanceId;
};

struct SuspensionEndpoint {
    SuspensionEndpointKind kind = SuspensionEndpointKind::Attachment;
    std::string id;
};

struct SuspensionSegmentDefinition {
    std::string id;
    SuspensionEndpoint from;
    SuspensionEndpoint to;
    double restLength = 0.0;
    double axialStiffness = 0.0;
    double axialDamping = 0.0;
    SuspensionSide side = SuspensionSide::Centre;
    std::vector<std::string> groups;
    std::string provenanceId;
};

struct SuspensionControlTarget {
    SuspensionControlTargetKind kind =
        SuspensionControlTargetKind::SegmentRestLength;
    std::string entityId;
    double travelPerCommand = 0.0;
    Vec3 localDirection;
};

struct SuspensionControlDefinition {
    std::string id;
    SuspensionControlKind kind = SuspensionControlKind::Brake;
    SuspensionSide side = SuspensionSide::Centre;
    double minimumCommand = 0.0;
    double maximumCommand = 1.0;
    double neutralCommand = 0.0;
    double maximumRate = 1.0;
    bool clampOutOfRange = false;
    std::vector<SuspensionControlTarget> targets;
    std::string provenanceId;
};

struct RigidPayloadState {
    Vec3 centreOfMassWorld;
    Quaternion orientation;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
};

struct RigidPayloadDefinition {
    double mass = 0.0;
    Vec3 centreOfMassLocal;
    Mat3 inertiaBody;
    RigidPayloadState initialState;
    std::vector<PayloadPointDefinition> hangPoints;
    std::vector<PayloadPointDefinition> supportPoints;
    std::string provenanceId;
};

struct SuspensionSolverSettings {
    int lineIterations = 16;
    double attachmentTolerance = 1.0e-10;
    double minimumLineLength = 1.0e-10;
    double maximumLineResidual = 2.0e-4;
    double maximumControlWork = 1.0e6;
};

struct PayloadGroundSettings {
    PayloadGroundMode mode = PayloadGroundMode::Free;
    Vec3 planeNormal{0.0, 0.0, 1.0};
    double planeOffset = 0.0;
    double compliance = 0.0;
    double penetrationFraction = 0.02;
};

struct SuspensionDefinition {
    std::size_t schemaMajor = 1;
    std::string identifier;
    std::string description;
    std::string unitsFrameTag;
    std::vector<SuspensionProvenance> provenance;
    std::vector<SuspensionAttachmentDefinition> attachments;
    std::vector<SuspensionJunctionDefinition> junctions;
    RigidPayloadDefinition payload;
    std::vector<SuspensionSegmentDefinition> segments;
    std::vector<SuspensionControlDefinition> controls;
    SuspensionSolverSettings solver;
    PayloadGroundSettings ground;
};

struct ResolvedSuspensionAttachment {
    std::string id;
    std::string panelId;
    Vec2 chart;
    std::size_t nodeIndex = 0;
    Vec3 worldPosition;
    std::string provenanceId;
};

struct RuntimeSuspensionEndpoint {
    SuspensionEndpoint definition;
    std::optional<std::size_t> nodeIndex;
    std::optional<std::size_t> payloadPointIndex;
};

struct RuntimeSuspensionSegment {
    SuspensionSegmentDefinition definition;
    RuntimeSuspensionEndpoint from;
    RuntimeSuspensionEndpoint to;
    std::vector<std::string> attachmentPaths;
    double commandedRestLength = 0.0;
    double accumulatedLambda = 0.0;
};

struct SuspensionControlState {
    std::string id;
    double targetCommand = 0.0;
    double actualCommand = 0.0;
    double travel = 0.0;
    double signedWork = 0.0;
};

struct Wrench {
    Vec3 force;
    Vec3 moment;
};

struct GeneralizedLineEndpoint {
    double inverseMass = 0.0;
    bool rigid = false;
    Vec3 offset;
    Mat3 inverseWorldInertia;
};

struct UnilateralLineProjection {
    double effectiveInverseMass = 0.0;
    double multiplierIncrement = 0.0;
    double accumulatedMultiplier = 0.0;
    double tension = 0.0;
    bool taut = false;
};

[[nodiscard]] double generalizedLineEffectiveInverseMass(
    const GeneralizedLineEndpoint& endpoint,
    const Vec3& gradient);
[[nodiscard]] UnilateralLineProjection projectUnilateralLine(
    double length,
    double commandedRestLength,
    double stiffness,
    double timeStep,
    double accumulatedMultiplier,
    const GeneralizedLineEndpoint& first,
    const GeneralizedLineEndpoint& second,
    const Vec3& firstGradient);
[[nodiscard]] double boundedAxialDampingImpulse(
    double relativeAxialVelocity,
    double damping,
    double timeStep,
    double effectiveInverseMass);

struct SuspensionSegmentDiagnostics {
    std::string id;
    SuspensionEndpoint from;
    SuspensionEndpoint to;
    std::vector<std::string> paths;
    std::vector<std::string> groups;
    double length = 0.0;
    double commandedRestLength = 0.0;
    double stretch = 0.0;
    double strain = 0.0;
    bool taut = false;
    double multiplier = 0.0;
    double tension = 0.0;
    double residual = 0.0;
    double elasticEnergy = 0.0;
    Vec3 dampingImpulse;
    double dampingWork = 0.0;
    double controlWork = 0.0;
    Vec3 fromImpulse;
    Vec3 toImpulse;
    Vec3 fromMoment;
    Vec3 toMoment;
};

struct PayloadDiagnostics {
    RigidPayloadState state;
    Vec3 linearMomentum;
    Vec3 angularMomentum;
    double translationalKineticEnergy = 0.0;
    double rotationalKineticEnergy = 0.0;
    double gravitationalEnergy = 0.0;
    Wrench appliedWrench;
    Wrench lineWrench;
    Wrench groundWrench;
    Wrench anchorWrench;
};

struct SuspensionDiagnostics {
    bool registered = false;
    bool converged = false;
    bool allSlack = true;
    bool anchored = false;
    bool grounded = false;
    bool failedTrial = false;
    std::size_t attachmentCount = 0;
    std::size_t junctionCount = 0;
    std::size_t segmentCount = 0;
    std::size_t tautCount = 0;
    std::size_t slackCount = 0;
    std::size_t solverIterations = 0;
    double maximumResidual = 0.0;
    double maximumGroundPenetration = 0.0;
    double elasticEnergy = 0.0;
    double dampingWork = 0.0;
    double controlWork = 0.0;
    Vec3 netInternalImpulse;
    Vec3 netInternalMoment;
    Wrench fixedSupportReaction;
    Wrench canopySupportReaction;
    Wrench groundReaction;
    std::vector<std::pair<std::string, Vec3>> attachmentLoads;
    std::vector<std::pair<std::string, Vec3>> groupLoads;
    std::string provenance;
    SuspensionPhase failurePhase = SuspensionPhase::Diagnostics;
    std::string failureEntity;
};

[[nodiscard]] const char* suspensionPhaseName(SuspensionPhase phase);
[[nodiscard]] const char* suspensionSideName(SuspensionSide side);
[[nodiscard]] const char* suspensionEndpointKindName(
    SuspensionEndpointKind kind);
[[nodiscard]] const char* suspensionControlKindName(SuspensionControlKind kind);
[[nodiscard]] const char* payloadGroundModeName(PayloadGroundMode mode);

[[nodiscard]] SuspensionDefinition validateAndNormalizeSuspensionDefinition(
    const SuspensionDefinition& definition);
[[nodiscard]] std::string serializeSuspensionDefinition(
    const SuspensionDefinition& definition);
[[nodiscard]] SuspensionDefinition parseSuspensionDefinition(
    std::string_view text);

[[nodiscard]] Vec3 payloadPointPosition(
    const RigidPayloadDefinition& definition,
    const RigidPayloadState& state,
    const Vec3& localPoint);
[[nodiscard]] Vec3 payloadPointVelocity(
    const RigidPayloadDefinition& definition,
    const RigidPayloadState& state,
    const Vec3& localPoint);
[[nodiscard]] Mat3 payloadWorldInertia(
    const RigidPayloadDefinition& definition,
    const RigidPayloadState& state);
[[nodiscard]] Vec3 payloadLinearMomentum(
    const RigidPayloadDefinition& definition,
    const RigidPayloadState& state);
[[nodiscard]] Vec3 payloadAngularMomentum(
    const RigidPayloadDefinition& definition,
    const RigidPayloadState& state);
[[nodiscard]] double payloadKineticEnergy(
    const RigidPayloadDefinition& definition,
    const RigidPayloadState& state);
void applyPayloadImpulse(const RigidPayloadDefinition& definition,
                         RigidPayloadState& state,
                         const Vec3& worldPoint,
                         const Vec3& impulse);
void integratePayloadTorqueFree(const RigidPayloadDefinition& definition,
                                RigidPayloadState& state,
                                double timeStep,
                                const Vec3& force = {},
                                const Vec3& moment = {});

class SuspensionSystem {
public:
    static SuspensionSystem build(CanopyMesh& canopy,
                                  const SuspensionDefinition& definition);

    SuspensionSystem(SuspensionSystem&&) noexcept = default;
    SuspensionSystem& operator=(SuspensionSystem&&) noexcept = default;
    SuspensionSystem(const SuspensionSystem&) = delete;
    SuspensionSystem& operator=(const SuspensionSystem&) = delete;

    [[nodiscard]] const SuspensionDefinition& definition() const {
        return definition_;
    }
    [[nodiscard]] const std::vector<ResolvedSuspensionAttachment>&
    attachments() const { return attachments_; }
    [[nodiscard]] const std::vector<RuntimeSuspensionSegment>& segments() const {
        return segments_;
    }
    [[nodiscard]] const RigidPayloadState& payloadState() const {
        return payloadState_;
    }
    [[nodiscard]] Vec3 hangPointPosition(std::size_t pointIndex) const;
    [[nodiscard]] Vec3 hangPointVelocity(std::size_t pointIndex) const;
    [[nodiscard]] const std::vector<SuspensionControlState>& controls() const {
        return controls_;
    }
    [[nodiscard]] const std::vector<SuspensionSegmentDiagnostics>&
    segmentDiagnostics() const { return segmentDiagnostics_; }
    [[nodiscard]] const PayloadDiagnostics& payloadDiagnostics() const {
        return payloadDiagnostics_;
    }
    [[nodiscard]] const SuspensionDiagnostics& diagnostics() const {
        return diagnostics_;
    }
    [[nodiscard]] const SuspensionDiagnostics& committedDiagnostics() const {
        return committedDiagnostics_;
    }
    [[nodiscard]] SoftBody& owner() const;

    void setControlTarget(std::string_view controlId, double command);
    void resetControls();
    void setPayloadState(const RigidPayloadState& state);
    void setAppliedPayloadWrench(const Wrench& wrench);
    void step(const StepSettings& settings);

private:
    friend class SoftBody;
    friend class AerodynamicSystem;
    // Accepted Stage 7 flight-state restart module (production friend, not
    // test access); it restores committed state into this live owner.
    friend struct FlightStateAccess;
#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
    friend struct AerodynamicVerificationTestAccess;
#endif

    SuspensionSystem() = default;
    void requireOwner(const SoftBody& body) const;
    void beginSubstep(SoftBody& body, double dt, const Vec3& gravity);
    void solveLineIteration(SoftBody& body, double dt);
    void solveGroundIteration(double dt);
    void certifySubstep(const SoftBody& body, double dt);
    void reconstructPayloadVelocity(double dt);
    void applyLineDamping(SoftBody& body, double dt);
    void finishSubstep(const SoftBody& body, double dt);
    void rollbackSubstep() noexcept;
    void recordFailure(SuspensionPhase phase,
                       const std::string& entity) noexcept;

    SuspensionDefinition definition_;
    SoftBody* owner_ = nullptr;
    std::vector<ResolvedSuspensionAttachment> attachments_;
    std::vector<std::size_t> junctionNodeIndices_;
    std::vector<RuntimeSuspensionSegment> segments_;
    RigidPayloadState payloadState_;
    RigidPayloadState previousPayloadState_;
    std::vector<Vec3> baseHangPointPositions_;
    std::vector<Vec3> commandedHangPointPositions_;
    std::vector<SuspensionControlState> controls_;
    std::vector<double> groundMultipliers_;
    Wrench appliedWrench_;
    Vec3 currentGravity_;
    std::vector<double> pendingSegmentControlWork_;
    std::vector<SuspensionSegmentDiagnostics> segmentDiagnostics_;
    PayloadDiagnostics payloadDiagnostics_;
    SuspensionDiagnostics diagnostics_;
    SuspensionDiagnostics committedDiagnostics_;

    RigidPayloadState snapshotPayloadState_;
    RigidPayloadState snapshotPreviousPayloadState_;
    std::vector<Vec3> snapshotCommandedHangPointPositions_;
    std::vector<RuntimeSuspensionSegment> snapshotSegments_;
    std::vector<SuspensionControlState> snapshotControls_;
    std::vector<double> snapshotGroundMultipliers_;
    Vec3 snapshotCurrentGravity_;
    std::vector<double> snapshotPendingSegmentControlWork_;
    std::vector<SuspensionSegmentDiagnostics> snapshotSegmentDiagnostics_;
    PayloadDiagnostics snapshotPayloadDiagnostics_;
    SuspensionDiagnostics snapshotDiagnostics_;
    bool hasSnapshot_ = false;
    // Private logical-owner identity. Move transfers the identity so a
    // registered logical suspension remains usable at its new address;
    // destruction or replacement expires the token observed by coupled
    // systems.
    std::shared_ptr<void> lifetimeToken_ =
        std::make_shared<unsigned char>();
};

} // namespace softwing
