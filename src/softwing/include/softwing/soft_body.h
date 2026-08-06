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
    // Bilateral tie which belongs to a deep suspension/load path. It has
    // ordinary distance physics, but participates in the targeted
    // reverse/forward load-path sweeps together with unilateral cables.
    SuspensionTie,
};

struct DistanceConstraint {
    std::size_t a = 0;
    std::size_t b = 0;
    double restLength = 0.0;
    double compliance = 0.0;
    double accumulatedLambda = 0.0;
    ConstraintKind kind = ConstraintKind::Distance;
};

// Four-node discrete-shell hinge. a-b is the shared edge; c belongs to the
// oriented face (a,b,c), d to (b,a,d). The signed angle is zero when those
// face normals agree. It resists fold without adding an in-plane diagonal.
struct DihedralBendingConstraint {
    std::size_t a = 0;
    std::size_t b = 0;
    std::size_t c = 0;
    std::size_t d = 0;
    double restAngleRadians = 0.0;
    double compliance = 0.0;
    double accumulatedLambda = 0.0;
};

struct DihedralBendingDiagnostics {
    double angleRadians = 0.0;
    std::array<Vec3, 4> angleGradient{};
    bool valid = false;
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
    std::uint64_t cableConstraintNanoseconds = 0;
    std::uint64_t membraneConstraintNanoseconds = 0;
    std::uint64_t bendingConstraintNanoseconds = 0;
    std::uint64_t suspensionConstraintNanoseconds = 0;
    std::uint64_t contactConstraintNanoseconds = 0;
    std::uint64_t contactCertificationNanoseconds = 0;
    std::uint64_t membraneDiagnosticsNanoseconds = 0;
    std::uint64_t finalizationNanoseconds = 0;

    std::uint64_t pneumaticSubsteps = 0;
    std::uint64_t structuralSubsteps = 0;
    std::uint64_t constraintIterations = 0;
    std::uint64_t distanceConstraintVisits = 0;
    std::uint64_t cableConstraintVisits = 0;
    std::uint64_t membraneConstraintVisits = 0;
    std::uint64_t bendingConstraintVisits = 0;
    std::uint64_t suspensionConstraintVisits = 0;
    std::uint64_t contactConstraintVisits = 0;

    void reset() noexcept { *this = {}; }
};

// Shape of the parallel distance-constraint sweep, for tuning and reporting.
// Like StepPerformanceProfile this is an observation only: nothing in the
// solver reads it back.
struct ConstraintColouringReport {
    std::size_t colourCount = 0;
    // Colours wide enough to be worth a barrier; the rest run serially.
    std::size_t parallelColours = 0;
    std::size_t largestColour = 0;
    std::size_t parallelConstraints = 0;
    std::size_t serialConstraints = 0;
};

enum class ParallelMembraneMode {
    ColouredGaussSeidel,
    Jacobi,
};

struct StepSettings {
    double timeStep = 1.0 / 120.0;
    int substeps = 2;
    int constraintIterations = 12;
    // Extra serial reverse+forward passes over the suspension load path:
    // unilateral cables plus bilateral canopy/harness ties. A deep cascade
    // otherwise needs one whole cloth iteration per graph level before a
    // payload reaction reaches the canopy. Zero preserves the historical
    // solver byte-for-byte.
    int cableConstraintSweepPairs = 0;
    Vec3 gravity{0.0, 0.0, -9.80665};
    double velocityDampingPerSecond = 0.25;
    // What the damping decays node velocity toward. Zero — the default, and
    // the only value any acceptance gate was baselined against — damps
    // absolute velocity, which is right for a body meant to come to rest.
    // A body in free flight is not meant to come to rest: damping its
    // absolute velocity acts as a fake drag several times larger than the
    // real aerodynamic drag budget, and no glide can survive it. A host
    // flying a body sets this to the system's bulk (centre-of-mass)
    // velocity, so fabric ringing and tumbling are damped while the flight
    // itself is not. With the default zero the arithmetic reduces exactly
    // to the historical expression.
    Vec3 dampingReferenceVelocity{};
    CcdSettings contactCcd;
    // 0 keeps the single-threaded solver and its exact element ordering, which
    // is what every acceptance gate is baselined against.
    //
    // Any value >= 1 runs the parallel sweeps: the coloured distance/cable
    // constraint sweep, and the explicitly selected parallel membrane mode.
    // All of them follow a different sweep from serial index-order
    // Gauss-Seidel, but each is bit-identical at every worker count, including
    // 1. The count buys speed only; it never selects physics. Set it
    // explicitly: the core never reads the core count.
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
    std::size_t addSuspensionTieConstraint(std::size_t a,
                                           std::size_t b,
                                           double restLength,
                                           double compliance = 0.0);
    std::size_t addDihedralBendingConstraint(
        std::size_t a,
        std::size_t b,
        std::size_t c,
        std::size_t d,
        double restAngleRadians,
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
    [[nodiscard]] DihedralBendingDiagnostics dihedralDiagnostics(
        std::size_t constraintIndex) const;

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
    // Mutable for the same reason nodes() is: a host may need to retune a
    // constraint between steps -- a brake line shortening, say. Changing a
    // rest length or a compliance is safe; adding or removing constraints
    // through this reference is not, and would leave the colouring stale.
    [[nodiscard]] std::vector<DistanceConstraint>& constraints() {
        return constraints_;
    }
    [[nodiscard]] const std::vector<MembraneElement>& membraneElements() const {
        return membraneElements_;
    }
    [[nodiscard]] const std::vector<DihedralBendingConstraint>&
    dihedralConstraints() const {
        return dihedralConstraints_;
    }
    // Builds the colouring if it is not current, so it reports what a
    // parallel step would actually run.
    [[nodiscard]] ConstraintColouringReport constraintColouringReport() const;
    // The partition itself, for a backend that wants to run the same sweep
    // somewhere else (see tools/softwing_gpu.cpp). `order` lists constraint
    // indices colour-major; `colourOffsets` bounds each colour within it and
    // has one more entry than there are colours. Both are owned by the body
    // and are invalidated by any topology change.
    struct ConstraintColouringView {
        std::span<const std::size_t> order;
        std::span<const std::size_t> colourOffsets;
    };
    [[nodiscard]] ConstraintColouringView constraintColouringView() const;
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
    // A substep-heavy budget (many substeps, few iterations) converges far
    // better per unit work than an iteration-heavy one, which leaves the
    // once-per-substep passes -- pressure, prediction, finalization -- as a
    // third of the frame rather than the 4% they were. They parallelise
    // trivially: every node writes only itself.
    void accumulatePressureForces(WorkerPool* pool);
    void predictPositions(double dt,
                          const StepSettings& settings,
                          double damping,
                          WorkerPool* pool);
    void finalizeVelocities(double dt, WorkerPool* pool);

    // Node -> incident triangles. The pressure pass walks faces and scatters
    // a third of each load to its corners, which cannot be shared out without
    // atomics; gathering per node can. Each node sums its incident faces in
    // ascending triangle index, which is the order the scatter added them in,
    // so the gather is bit-identical to it and to itself at any worker count.
    struct TriangleIncidence {
        std::vector<std::size_t> nodeOffsets;
        std::vector<std::size_t> triangles;
        std::size_t builtForTriangleCount = 0;
        std::size_t builtForNodeCount = 0;
    };
    [[nodiscard]] const TriangleIncidence& triangleIncidence() const;
    // Takes 1/dt^2 rather than dt: it is loop-invariant across the whole
    // sweep and computing it per constraint is a division the solver cannot
    // afford (see projectDistanceConstraint).
    void solveConstraint(DistanceConstraint& constraint,
                         double inverseTimeStepSquared);

    // Suspension constraints form a deep, branching graph whose authored
    // insertion order is not a load-propagation order. Build a stable order
    // by graph depth from structural/fixed canopy nodes so one reverse/forward
    // pair actually travels payload-to-canopy and back. Cached topology only;
    // changing a rest length or multiplier does not invalidate it.
    [[nodiscard]] const std::vector<std::size_t>&
    loadPathConstraintOrder() const;

    // A distance/cable projection reads exactly two things per node: its
    // position and its inverse mass. A Node is 104 bytes, so a sweep over it
    // drags four times that through the cache for nothing -- and the sweep is
    // pure cache-latency work, which is where nearly all of a mass-spring
    // step goes. Packed to 32 bytes, two nodes share a line and a mid-sized
    // body's hot state fits in L2 instead of spilling to L3.
    //
    // Only usable when nothing else in the substep moves nodes; membrane,
    // contact and suspension all do, so those substeps sweep Node directly.
    // Same arithmetic on the same values either way: the packing is a
    // container change, not a physics one.
    struct SolveNode {
        Vec3 position;
        double inverseMass = 0.0;
    };
    void packConstraintSolveNodes(WorkerPool* pool);
    void unpackConstraintSolveNodes(WorkerPool* pool);
    void solveConstraintPacked(DistanceConstraint& constraint,
                               double inverseTimeStepSquared);
    [[nodiscard]] std::array<Vec3, 3> membraneElementCorrections(
        MembraneElement& element,
        double dt);
    void solveMembraneElement(MembraneElement& element, double dt);
    void updateMembraneSolverDiagnostics(MembraneElement& element,
                                         double dt);
    void solveDihedralConstraint(DihedralBendingConstraint& constraint,
                                 double inverseTimeStepSquared);

    // The same colouring idea applied to the distance/cable constraints.
    // Constraints sharing a colour share no node, so a colour is one
    // deterministic Gauss-Seidel phase whatever the worker count.
    //
    // Unlike the membrane graph this one is not degree-bounded: a mass-spring
    // rib hub can carry a hundred spokes, and every spoke needs its own
    // colour. The colouring therefore uses a wide per-node mask rather than a
    // single 64-bit word, and colours too small to be worth a barrier are run
    // serially after the parallel ones (see parallelColours).
    struct ConstraintColouring {
        std::vector<std::size_t> constraints;
        std::vector<std::size_t> colourOffsets;
        std::size_t parallelColours = 0;
        std::size_t largestColour = 0;
        std::size_t serialConstraints = 0;
        std::size_t builtForConstraintCount = 0;
        std::size_t builtForNodeCount = 0;

        [[nodiscard]] unsigned workerCap(unsigned requested) const;
    };
    [[nodiscard]] const ConstraintColouring& constraintColouring() const;
    void solveConstraintsColoured(double inverseTimeStepSquared,
                                  WorkerPool& pool,
                                  bool packed);

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
    std::vector<DihedralBendingConstraint> dihedralConstraints_;
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
    std::vector<SolveNode> constraintSolveNodes_;
    mutable TriangleIncidence triangleIncidence_;
    struct LoadPathOrdering {
        std::vector<std::size_t> constraints;
        std::size_t builtForConstraintCount = 0;
        std::size_t builtForNodeCount = 0;
        std::size_t builtForTriangleCount = 0;
        std::size_t builtForDihedralCount = 0;
    };
    mutable LoadPathOrdering loadPathOrdering_;
    mutable ConstraintColouring constraintColouring_;
    mutable MembraneColouring membraneColouring_;
    MembraneJacobiScratch membraneJacobiScratch_;
    WorkerPoolSlot workerPool_;
};

} // namespace softwing
