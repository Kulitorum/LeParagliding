#pragma once

#include "softwing/contact.h"
#include "softwing/membrane.h"
#include "softwing/parallel.h"
#include "softwing/vec3.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace softwing {

class SoftBody;
class PneumaticNetwork;
class SuspensionSystem;
class AerodynamicSystem;
struct ContactAuditTestAccess;
// Accepted Stage 7 flight-state restart module. It is a production friend
// (not test access): the SOFTWING_FLIGHT_STATE 1 artifact restores committed
// state into existing live owners without widening any public mutator.
struct FlightStateAccess;
#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
struct AerodynamicVerificationTestAccess;
#endif

class SurfaceGroup {
public:
    [[nodiscard]] std::size_t firstTriangle() const { return firstTriangle_; }
    [[nodiscard]] std::size_t triangleCount() const { return triangleCount_; }

private:
    friend class SoftBody;

    SurfaceGroup(const SoftBody* owner,
                 std::size_t firstTriangle,
                 std::size_t triangleCount)
        : owner_(owner),
          firstTriangle_(firstTriangle),
          triangleCount_(triangleCount) {}

    const SoftBody* owner_ = nullptr;
    std::size_t firstTriangle_ = 0;
    std::size_t triangleCount_ = 0;
};

struct Node {
    Vec3 position;
    Vec3 previousPosition;
    Vec3 velocity;
    Vec3 force;
    double inverseMass = 0.0;
};

struct Triangle {
    std::size_t a = 0;
    std::size_t b = 0;
    std::size_t c = 0;
    double pressureDifference = 0.0;
};

struct SurfaceTopologyReport {
    std::size_t outOfRangeNodeReferences = 0;
    std::size_t degenerateFaces = 0;
    std::size_t boundaryEdges = 0;
    std::size_t nonManifoldEdges = 0;
    std::size_t inconsistentDirectedEdges = 0;

    [[nodiscard]] bool valid() const {
        return outOfRangeNodeReferences == 0 && degenerateFaces == 0 &&
               boundaryEdges == 0 && nonManifoldEdges == 0 &&
               inconsistentDirectedEdges == 0;
    }
};

[[nodiscard]] SurfaceTopologyReport validateSurfaceTopology(
    std::span<const Node> nodes,
    std::span<const Triangle> triangles);

void validateMembraneElementDefinitions(
    std::span<const Node> nodes,
    std::span<const Triangle> triangles,
    std::span<const MembraneElementDefinition> definitions);

enum class ConstraintKind {
    Distance,
    Cable,
};

struct DistanceConstraint {
    std::size_t a = 0;
    std::size_t b = 0;
    double restLength = 0.0;
    double compliance = 0.0;
    double accumulatedLambda = 0.0;
    ConstraintKind kind = ConstraintKind::Distance;
};

// Optional wall-clock instrumentation for locating runtime cost. This is a
// caller-owned observation sink: it is never serialized, hashed, consulted by
// physics, or populated unless StepSettings::performanceProfile is non-null.
// Inclusive fields are named explicitly so callers do not accidentally add a
// parent duration to its children.
struct StepPerformanceProfile {
    std::uint64_t pneumaticTotalNanoseconds = 0;
    std::uint64_t pneumaticTransactionSnapshotNanoseconds = 0;
    std::uint64_t pneumaticGeometryNanoseconds = 0;
    std::uint64_t pneumaticMassTransferNanoseconds = 0;
    std::uint64_t pneumaticInterfaceForceNanoseconds = 0;
    std::uint64_t pneumaticStructuralAdvanceNanoseconds = 0;
    std::uint64_t pneumaticCertificationNanoseconds = 0;
    std::uint64_t pneumaticLedgerNanoseconds = 0;

    std::uint64_t softBodyTotalNanoseconds = 0;
    std::uint64_t softBodyTransactionSnapshotNanoseconds = 0;
    std::uint64_t predictionNanoseconds = 0;
    std::uint64_t distanceConstraintNanoseconds = 0;
    std::uint64_t membraneConstraintNanoseconds = 0;
    std::uint64_t suspensionConstraintNanoseconds = 0;
    std::uint64_t contactConstraintNanoseconds = 0;
    std::uint64_t contactCertificationNanoseconds = 0;
    std::uint64_t membraneDiagnosticsNanoseconds = 0;
    std::uint64_t finalizationNanoseconds = 0;

    std::uint64_t pneumaticSubsteps = 0;
    std::uint64_t structuralSubsteps = 0;
    std::uint64_t constraintIterations = 0;
    std::uint64_t distanceConstraintVisits = 0;
    std::uint64_t membraneConstraintVisits = 0;
    std::uint64_t suspensionConstraintVisits = 0;
    std::uint64_t contactConstraintVisits = 0;

    void reset() noexcept { *this = {}; }
};

enum class ParallelMembraneMode {
    ColouredGaussSeidel,
    Jacobi,
};

struct StepSettings {
    double timeStep = 1.0 / 120.0;
    int substeps = 2;
    int constraintIterations = 12;
    Vec3 gravity{0.0, 0.0, -9.80665};
    double velocityDampingPerSecond = 0.25;
    CcdSettings contactCcd;
    // 0 keeps the single-threaded solver and its exact element ordering, which
    // is what every acceptance gate is baselined against.
    //
    // Any value >= 1 runs the explicitly selected parallel membrane mode. Both
    // modes follow a different sweep from serial index-order Gauss-Seidel, but
    // each is bit-identical at every worker count, including 1. The count buys
    // speed only; it never selects physics. Set it explicitly: the core never
    // reads the core count.
    unsigned workerThreads = 0;
    ParallelMembraneMode parallelMembraneMode =
        ParallelMembraneMode::ColouredGaussSeidel;
    StepPerformanceProfile* performanceProfile = nullptr;
};

struct RectangularPatch {
    std::size_t firstNode = 0;
    std::size_t chordNodes = 0;
    std::size_t spanNodes = 0;

    [[nodiscard]] std::size_t node(std::size_t chordIndex,
                                   std::size_t spanIndex) const;
};

struct RectangularCell {
    std::size_t firstNode;
    std::size_t chordNodes;
    std::size_t spanNodes;
    SurfaceGroup surface;

    [[nodiscard]] std::size_t lowerNode(std::size_t chordIndex,
                                        std::size_t spanIndex) const;
    [[nodiscard]] std::size_t upperNode(std::size_t chordIndex,
                                        std::size_t spanIndex) const;
};

struct RectangularMembraneCoupon {
    std::size_t firstNode;
    std::size_t lengthNodes;
    std::size_t widthNodes;
    SurfaceGroup surface;
    MembraneGroup membrane;

    [[nodiscard]] std::size_t node(std::size_t lengthIndex,
                                   std::size_t widthIndex) const;
};

class SoftBody {
public:
    std::size_t addNode(const Vec3& position, double mass);
    std::size_t addFixedNode(const Vec3& position);
    void fixNode(std::size_t nodeIndex);

    std::size_t addTriangle(std::size_t a,
                            std::size_t b,
                            std::size_t c,
                            double pressureDifference = 0.0);
    std::size_t addDistanceConstraint(std::size_t a,
                                      std::size_t b,
                                      double restLength,
                                      double compliance = 0.0);
    std::size_t addCableConstraint(std::size_t a,
                                   std::size_t b,
                                   double maximumLength,
                                   double compliance = 0.0);

    [[nodiscard]] SurfaceGroup surfaceGroup(std::size_t firstTriangle,
                                            std::size_t triangleCount) const;
    [[nodiscard]] MembraneGroup addMembraneElements(
        std::span<const MembraneElementDefinition> definitions);
    [[nodiscard]] std::span<const MembraneElement> membraneElements(
        const MembraneGroup& group) const;
    [[nodiscard]] MembraneElementDiagnostics membraneDiagnostics(
        std::size_t elementIndex,
        const Vec3& momentOrigin = {}) const;
    [[nodiscard]] MembraneGroupDiagnostics membraneDiagnostics(
        const MembraneGroup& group,
        const Vec3& momentOrigin = {}) const;

    [[nodiscard]] ContactSurfaceHandle addContactSurface(
        const SurfaceGroup& surface,
        double halfThickness);
    [[nodiscard]] ContactLineHandle addContactLine(std::size_t a,
                                                   std::size_t b,
                                                   double radius);
    [[nodiscard]] ContactPairHandle addContactPair(
        const ContactColliderHandle& first,
        const ContactColliderHandle& second,
        const ContactPairSettings& settings);
    // Bounds-checked lookup of an already registered surface collider. This
    // supports owner-safe integration without adding contact primitive pairs
    // or changing any contact physics capability.
    [[nodiscard]] ContactColliderHandle contactSurfaceCollider(
        std::size_t surfaceIndex) const;
    [[nodiscard]] const RegisteredContactSurface& contactSurface(
        const ContactSurfaceHandle& surface) const;
    [[nodiscard]] const RegisteredContactLine& contactLine(
        const ContactLineHandle& line) const;
    [[nodiscard]] const RegisteredContactPair& contactPair(
        const ContactPairHandle& pair) const;
    [[nodiscard]] ContactTopologyReport contactTopology(
        const ContactPairHandle& pair) const;
    [[nodiscard]] const std::vector<ContactRecord>& contactRecords() const {
        return contactRecords_;
    }
    [[nodiscard]] const ContactDiagnostics& contactDiagnostics() const {
        return contactDiagnostics_;
    }
    [[nodiscard]] ContactDiagnostics contactDiagnostics(
        const ContactPairHandle& pair) const;

    // Declares that this body carries faces separating two pressurised zones
    // -- the interior ribs and cross-port closures of a multi-cell canopy.
    // buildCanopy() sets it; nothing else should need to.
    void declareInteriorPressurePartitions();
    [[nodiscard]] bool hasInteriorPressurePartitions() const {
        return interiorPressurePartitions_;
    }

    // Both uniform setters treat every face they touch as an exterior wall, so
    // they are only meaningful on a body whose triangles all separate inside
    // from outside. A multi-cell canopy has interior partitions (ribs,
    // cross-port closures) that must instead see the difference across them;
    // stamping one value over those makes each one push one-sidedly and gives
    // the whole body a spurious resultant. Use setUniformCellPressure() for a
    // canopy.
    //
    // That rule used to live only in this comment, and was violated twice --
    // once in the studio path, once in the viewer -- each time costing a
    // silent body-fixed thrust of tens of newtons that only showed up as a
    // wing visibly winding itself into a rotation. So it is enforced: on a
    // body with declared interior partitions these throw rather than stamp.
    // Zero is always permitted, since it prescribes no difference across any
    // face and is how setUniformCellPressure clears the field before signing
    // it.
    void setUniformPressureDifference(double pressureDifference);
    void setUniformPressureDifference(const SurfaceGroup& surface,
                                      double pressureDifference);
    // Prescribe one face's pressure difference, signed by its stored winding
    // (positive pushes along the right-hand normal of (a, b, c)). This is what
    // lets a caller that knows which side of a face is which -- the canopy
    // layer -- zero the interior partitions the uniform setters cannot see.
    void setFacePressureDifference(std::size_t triangleIndex,
                                   double pressureDifference);
    void clearExternalForces();
    void addForce(std::size_t nodeIndex, const Vec3& force);
    void step(const StepSettings& settings);
    void stepCoupled(const StepSettings& settings,
                     SuspensionSystem& suspension);

    [[nodiscard]] RectangularPatch addRectangularPatch(
        double chord,
        double span,
        std::size_t chordSegments,
        std::size_t spanSegments,
        double arealDensity,
        double stretchCompliance,
        double shearCompliance,
        double bendCompliance);

    [[nodiscard]] RectangularCell addRectangularCell(
        double chord,
        double span,
        double thickness,
        std::size_t chordSegments,
        std::size_t spanSegments,
        double arealDensity,
        double stretchCompliance,
        double shearCompliance,
        double bendCompliance);

    [[nodiscard]] RectangularMembraneCoupon addRectangularMembraneCoupon(
        double length,
        double width,
        std::size_t lengthSegments,
        std::size_t widthSegments,
        double arealDensity,
        const OrthotropicMembraneMaterial& material,
        double materialAngleRadians = 0.0,
        bool reverseDiagonal = false);

    [[nodiscard]] const std::vector<Node>& nodes() const { return nodes_; }
    [[nodiscard]] std::vector<Node>& nodes() { return nodes_; }
    [[nodiscard]] const std::vector<Triangle>& triangles() const { return triangles_; }
    [[nodiscard]] const std::vector<DistanceConstraint>& constraints() const {
        return constraints_;
    }
    [[nodiscard]] const std::vector<MembraneElement>& membraneElements() const {
        return membraneElements_;
    }
    [[nodiscard]] const std::vector<RegisteredContactSurface>&
    contactSurfaces() const {
        return contactSurfaces_;
    }
    [[nodiscard]] const std::vector<RegisteredContactLine>& contactLines() const {
        return contactLines_;
    }
    [[nodiscard]] const std::vector<RegisteredContactPair>& contactPairs() const {
        return contactPairs_;
    }
    [[nodiscard]] bool hasAerodynamicRegistration() const {
        return !aerodynamicRegistration_.expired();
    }

    [[nodiscard]] double surfaceArea() const;
    [[nodiscard]] double surfaceArea(const SurfaceGroup& surface) const;
    [[nodiscard]] double signedVolume(const SurfaceGroup& surface) const;
    [[nodiscard]] SurfaceTopologyReport validateSurfaceTopology(
        const SurfaceGroup& surface) const;
    [[nodiscard]] Vec3 totalPressureForce() const;
    [[nodiscard]] Vec3 totalPressureForce(const SurfaceGroup& surface) const;
    [[nodiscard]] Vec3 totalPressureMoment(const Vec3& origin = {}) const;
    [[nodiscard]] Vec3 totalPressureMoment(const SurfaceGroup& surface,
                                           const Vec3& origin = {}) const;

private:
    friend class PneumaticNetwork;
    friend class SuspensionSystem;
    friend class AerodynamicSystem;
    friend struct ContactAuditTestAccess;
    friend struct FlightStateAccess;
#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
    friend struct AerodynamicVerificationTestAccess;
#endif
    struct ContactAuditState {
        // Private implementation-audit traces. The solver consumes these
        // exact streamed broadphase keys; the test friend proves iteration
        // and final certification coverage without expanding public
        // ContactDiagnostics or rebuilding the full eligible set.
        std::vector<ContactFeatureKey> iterationCandidateKeys;
        std::vector<ContactFeatureKey> iterationQueryKeys;
        std::vector<ContactFeatureKey> certificationCandidateKeys;
        std::vector<ContactFeatureKey> certificationQueryKeys;
    };
    void requireSurfaceGroup(const SurfaceGroup& surface) const;
    void requireMembraneGroup(const MembraneGroup& group) const;
    void requireContactSurface(const ContactSurfaceHandle& surface) const;
    void requireContactLine(const ContactLineHandle& line) const;
    void requireContactPair(const ContactPairHandle& pair) const;
    void integrateSubstep(double dt,
                          const StepSettings& settings,
                          SuspensionSystem* suspension = nullptr);
    void integrateSubstepTrial(double dt,
                               const StepSettings& settings,
                               SuspensionSystem* suspension = nullptr);
    void beginContactSubstep();
    void solveContactIteration(double dt, const StepSettings& settings);
    void certifyContactState(double dt, const StepSettings& settings);
    void applyContactFriction();
    void accumulatePressureForces();
    void solveConstraint(DistanceConstraint& constraint, double dt);
    [[nodiscard]] std::array<Vec3, 3> membraneElementCorrections(
        MembraneElement& element,
        double dt);
    void solveMembraneElement(MembraneElement& element, double dt);
    void updateMembraneSolverDiagnostics(MembraneElement& element,
                                         double dt);

    // Stable general parallel path: elements in one colour share no nodes, so
    // each colour is a deterministic Gauss-Seidel phase.
    struct MembraneColouring {
        std::vector<std::size_t> elements;
        std::vector<std::size_t> colourOffsets;
        std::size_t parallelColours = 0;
        std::size_t largestColour = 0;
        std::size_t builtForElementCount = 0;
        std::size_t builtForNodeCount = 0;

        [[nodiscard]] unsigned workerCap(unsigned requested) const;
    };
    [[nodiscard]] const MembraneColouring& membraneColouring() const;
    void solveMembraneColoured(double dt, WorkerPool& pool);

    // Scratch topology for the deterministic parallel Jacobi sweep. Element
    // corrections are produced independently. Each node then owns its output
    // and sums incident corrections in ascending element order, so scheduling
    // and worker count cannot reach the result.
    struct MembraneJacobiScratch {
        struct Incidence {
            std::size_t element = 0;
            std::uint8_t corner = 0;
        };

        std::vector<std::array<Vec3, 3>> elementCorrections;
        std::vector<std::size_t> nodeOffsets;
        std::vector<Incidence> incidences;
        std::size_t builtForElementCount = 0;
        std::size_t builtForNodeCount = 0;

        MembraneJacobiScratch() = default;
        // Derived scheduling data is not physics state. Body transaction
        // copies omit it, then rebuild lazily; assignments invalidate the
        // target while retaining its vector capacities for the next rebuild.
        MembraneJacobiScratch(const MembraneJacobiScratch&) noexcept {}
        MembraneJacobiScratch& operator=(
            const MembraneJacobiScratch&) noexcept {
            clear();
            return *this;
        }
        MembraneJacobiScratch(MembraneJacobiScratch&&) noexcept {}
        MembraneJacobiScratch& operator=(MembraneJacobiScratch&&) noexcept {
            clear();
            return *this;
        }

    private:
        void clear() noexcept {
            elementCorrections.clear();
            nodeOffsets.clear();
            incidences.clear();
            builtForElementCount = 0;
            builtForNodeCount = 0;
        }
    };
    [[nodiscard]] MembraneJacobiScratch& membraneJacobiScratch();
    void solveMembraneJacobi(double dt, WorkerPool& pool);
    // Null when settings.workerThreads is 0. The pool outlives the step
    // so workers are spawned once, not per sweep; it is rebuilt only when the
    // requested worker count changes.
    [[nodiscard]] WorkerPool* poolFor(const StepSettings& settings);

    std::vector<Node> nodes_;
    std::vector<Triangle> triangles_;
    std::vector<DistanceConstraint> constraints_;
    std::vector<MembraneElement> membraneElements_;
    std::vector<RegisteredContactSurface> contactSurfaces_;
    std::vector<RegisteredContactLine> contactLines_;
    std::vector<RegisteredContactPair> contactPairs_;
    std::vector<std::pair<ContactFeatureKey, double>> contactMultipliers_;
    std::vector<ContactRecord> contactRecords_;
    ContactDiagnostics contactDiagnostics_;
    std::vector<ContactDiagnostics> contactPairDiagnostics_;
    ContactAuditState contactAudit_;
    std::weak_ptr<void> aerodynamicRegistration_;
    bool interiorPressurePartitions_ = false;
    mutable MembraneColouring membraneColouring_;
    MembraneJacobiScratch membraneJacobiScratch_;
    WorkerPoolSlot workerPool_;
};

} // namespace softwing
