#include "softwing/soft_body.h"

#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
#include "aerodynamic_test_access.h"
#endif
#include "softwing/suspension.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <locale>
#include <map>
#include <queue>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace softwing {
namespace {

constexpr double kMinimumLength = 1.0e-12;

using PerformanceClock = std::chrono::steady_clock;

class PerformanceScope {
public:
    explicit PerformanceScope(std::uint64_t* destination) noexcept
        : destination_(destination),
          start_(destination ? PerformanceClock::now()
                             : PerformanceClock::time_point{}) {}

    ~PerformanceScope() {
        if (!destination_) return;
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
            PerformanceClock::now() - start_).count();
        *destination_ += static_cast<std::uint64_t>(elapsed);
    }

private:
    std::uint64_t* destination_ = nullptr;
    PerformanceClock::time_point start_;
};

[[nodiscard]] std::uint64_t* profileField(
    StepPerformanceProfile* profile,
    std::uint64_t StepPerformanceProfile::* field) noexcept {
    return profile ? &(profile->*field) : nullptr;
}

void requireNodeIndex(std::size_t index, std::size_t count) {
    if (index >= count) {
        throw std::out_of_range("SoftBody node index is out of range");
    }
}

struct EdgeIncidence {
    std::size_t count = 0;
    std::size_t ascendingCount = 0;
    std::size_t descendingCount = 0;
};

struct MembraneEvaluation {
    MembraneElementDiagnostics diagnostics;
    std::array<std::array<Vec3, 3>, 3> strainGradients{};
};

// The XPBD sweep evaluates every element once per iteration but reads only
// the Green strain and its gradients. Constitutive resultants, elastic energy
// and nodal forces/moments are reported quantities; computing them on the
// solver path is pure waste, so callers state which they need.
enum class MembraneEvaluationDetail {
    StrainOnly,
    Full,
};

struct MomentumState {
    double mass = 0.0;
    Vec3 centreOfMass;
    Vec3 linearMomentum;
    Vec3 angularMomentum;
};

MomentumState measureMomentum(std::span<const Node> nodes) {
    MomentumState result;
    for (const Node& node : nodes) {
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double mass = 1.0 / node.inverseMass;
        result.mass += mass;
        result.centreOfMass += mass * node.position;
        result.linearMomentum += mass * node.velocity;
    }
    if (!(result.mass > 0.0)) {
        return result;
    }
    result.centreOfMass /= result.mass;
    for (const Node& node : nodes) {
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double mass = 1.0 / node.inverseMass;
        result.angularMomentum +=
            mass * cross(node.position - result.centreOfMass, node.velocity);
    }
    return result;
}

void restoreMomentum(std::span<Node> nodes, const MomentumState& target) {
    if (!(target.mass > 0.0)) {
        return;
    }
    MomentumState current = measureMomentum(nodes);
    const Vec3 translationCorrection =
        (target.linearMomentum - current.linearMomentum) / target.mass;
    for (Node& node : nodes) {
        if (node.inverseMass > 0.0) {
            node.velocity += translationCorrection;
        }
    }
    current = measureMomentum(nodes);

    SymmetricMatrix3 inertia;
    for (const Node& node : nodes) {
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double mass = 1.0 / node.inverseMass;
        const Vec3 radius = node.position - current.centreOfMass;
        inertia.xx += mass * (radius.y * radius.y + radius.z * radius.z);
        inertia.yy += mass * (radius.x * radius.x + radius.z * radius.z);
        inertia.zz += mass * (radius.x * radius.x + radius.y * radius.y);
        inertia.xy -= mass * radius.x * radius.y;
        inertia.xz -= mass * radius.x * radius.z;
        inertia.yz -= mass * radius.y * radius.z;
    }
    if (!isPositiveDefinite(inertia)) {
        return;
    }
    const Vec3 angularVelocityCorrection = checkedSolve(
        inertia, target.angularMomentum - current.angularMomentum);
    for (Node& node : nodes) {
        if (node.inverseMass > 0.0) {
            node.velocity += cross(angularVelocityCorrection,
                                   node.position - current.centreOfMass);
        }
    }
}

std::size_t materialRoleIndex(MaterialRole role) {
    switch (role) {
    case MaterialRole::Bulk:
        return 0;
    case MaterialRole::Seam:
        return 1;
    case MaterialRole::Reinforcement:
        return 2;
    }
    throw std::invalid_argument("Invalid membrane material role");
}

MembraneEvaluation evaluateMembraneElement(
    std::span<const Node> nodes,
    std::span<const Triangle> triangles,
    const MembraneElement& element,
    const Vec3& momentOrigin,
    MembraneEvaluationDetail detail = MembraneEvaluationDetail::Full) {
    const Triangle& triangle = triangles[element.triangle];
    const std::array<Vec3, 3> positions{
        nodes[triangle.a].position,
        nodes[triangle.b].position,
        nodes[triangle.c].position,
    };
    const Vec3 firstSpatialEdge = positions[1] - positions[0];
    const Vec3 secondSpatialEdge = positions[2] - positions[0];
    const Matrix2& inverse = element.inverseReferenceMatrix;
    const Vec3 deformationWarp =
        inverse.m00 * firstSpatialEdge + inverse.m10 * secondSpatialEdge;
    const Vec3 deformationWeft =
        inverse.m01 * firstSpatialEdge + inverse.m11 * secondSpatialEdge;
    const Vec3 strain{
        0.5 * (dot(deformationWarp, deformationWarp) - 1.0) +
            element.material.warpPreTension,
        0.5 * (dot(deformationWeft, deformationWeft) - 1.0) +
            element.material.weftPreTension,
        dot(deformationWarp, deformationWeft),
    };
    MembraneEvaluation evaluation;
    evaluation.diagnostics.deformationWarp = deformationWarp;
    evaluation.diagnostics.deformationWeft = deformationWeft;
    evaluation.diagnostics.greenStrain = strain;
    evaluation.diagnostics.role = element.role;
    evaluation.diagnostics.solverResultantEstimate =
        element.solverResultantEstimate;
    evaluation.diagnostics.normalizedResidual = element.normalizedResidual;

    Vec3 resultant;
    if (detail == MembraneEvaluationDetail::Full) {
        resultant = effectiveMembraneStiffness(element.material, strain) * strain;
        evaluation.diagnostics.constitutiveResultant = resultant;
        evaluation.diagnostics.elasticEnergy =
            0.5 * element.referenceArea * dot(strain, resultant);
    }

    const std::array<Vec3, 3> derivativeByWarp{
        deformationWarp, Vec3{}, deformationWeft};
    const std::array<Vec3, 3> derivativeByWeft{
        Vec3{}, deformationWeft, deformationWarp};
    for (std::size_t component = 0; component < 3; ++component) {
        const Vec3 firstEdgeGradient =
            inverse.m00 * derivativeByWarp[component] +
            inverse.m01 * derivativeByWeft[component];
        const Vec3 secondEdgeGradient =
            inverse.m10 * derivativeByWarp[component] +
            inverse.m11 * derivativeByWeft[component];
        evaluation.strainGradients[component][0] =
            -(firstEdgeGradient + secondEdgeGradient);
        evaluation.strainGradients[component][1] = firstEdgeGradient;
        evaluation.strainGradients[component][2] = secondEdgeGradient;
    }

    if (detail == MembraneEvaluationDetail::Full) {
        for (std::size_t corner = 0; corner < 3; ++corner) {
            Vec3 energyGradient;
            for (std::size_t component = 0; component < 3; ++component) {
                const double componentResultant =
                    component == 0
                        ? resultant.x
                        : (component == 1 ? resultant.y : resultant.z);
                energyGradient += element.referenceArea * componentResultant *
                                  evaluation.strainGradients[component][corner];
            }
            evaluation.diagnostics.nodalForces[corner] = -energyGradient;
            evaluation.diagnostics.totalInternalForce += -energyGradient;
            evaluation.diagnostics.totalInternalMoment +=
                cross(positions[corner] - momentOrigin, -energyGradient);
        }
    }
    return evaluation;
}

} // namespace

SurfaceTopologyReport validateSurfaceTopology(
    std::span<const Node> nodes,
    std::span<const Triangle> triangles) {
    SurfaceTopologyReport report;
    std::map<std::pair<std::size_t, std::size_t>, EdgeIncidence> edges;
    constexpr double relativeAreaTolerance =
        64.0 * std::numeric_limits<double>::epsilon();

    for (const Triangle& triangle : triangles) {
        const std::size_t indices[]{triangle.a, triangle.b, triangle.c};
        bool hasInvalidIndex = false;
        for (const std::size_t index : indices) {
            if (index >= nodes.size()) {
                ++report.outOfRangeNodeReferences;
                hasInvalidIndex = true;
            }
        }
        if (hasInvalidIndex) {
            continue;
        }

        const Vec3 ab = nodes[triangle.b].position - nodes[triangle.a].position;
        const Vec3 ac = nodes[triangle.c].position - nodes[triangle.a].position;
        const Vec3 bc = nodes[triangle.c].position - nodes[triangle.b].position;
        const double maximumEdgeLengthSquared =
            std::max({lengthSquared(ab), lengthSquared(ac), lengthSquared(bc)});
        const double twiceAreaSquared = lengthSquared(cross(ab, ac));
        const double areaScale =
            maximumEdgeLengthSquared * maximumEdgeLengthSquared;
        const bool repeatedNode = triangle.a == triangle.b ||
                                  triangle.b == triangle.c ||
                                  triangle.c == triangle.a;
        if (repeatedNode || !std::isfinite(twiceAreaSquared) ||
            !(maximumEdgeLengthSquared > 0.0) ||
            twiceAreaSquared <= relativeAreaTolerance * relativeAreaTolerance *
                                    areaScale) {
            ++report.degenerateFaces;
            continue;
        }

        const auto addEdge = [&edges](std::size_t from, std::size_t to) {
            const auto key = std::minmax(from, to);
            EdgeIncidence& incidence = edges[key];
            ++incidence.count;
            if (from < to) {
                ++incidence.ascendingCount;
            } else {
                ++incidence.descendingCount;
            }
        };
        addEdge(triangle.a, triangle.b);
        addEdge(triangle.b, triangle.c);
        addEdge(triangle.c, triangle.a);
    }

    for (const auto& [edge, incidence] : edges) {
        static_cast<void>(edge);
        if (incidence.count == 1) {
            ++report.boundaryEdges;
        } else if (incidence.count > 2) {
            ++report.nonManifoldEdges;
        } else if (incidence.ascendingCount != 1 ||
                   incidence.descendingCount != 1) {
            ++report.inconsistentDirectedEdges;
        }
    }
    return report;
}

void validateMembraneElementDefinitions(
    std::span<const Node> nodes,
    std::span<const Triangle> triangles,
    std::span<const MembraneElementDefinition> definitions) {
    if (definitions.empty()) {
        throw std::invalid_argument("A membrane batch must contain an element");
    }
    constexpr double relativeAreaTolerance =
        64.0 * std::numeric_limits<double>::epsilon();
    for (const MembraneElementDefinition& definition : definitions) {
        validateOrthotropicMembraneMaterial(definition.material);
        switch (definition.role) {
        case MaterialRole::Bulk:
        case MaterialRole::Seam:
        case MaterialRole::Reinforcement:
            break;
        default:
            throw std::invalid_argument("Invalid membrane material role");
        }
        if (definition.triangle >= triangles.size()) {
            throw std::out_of_range("Membrane triangle index is out of range");
        }
        const Triangle& triangle = triangles[definition.triangle];
        if (triangle.a >= nodes.size() || triangle.b >= nodes.size() ||
            triangle.c >= nodes.size()) {
            throw std::out_of_range(
                "Membrane triangle contains an invalid node reference");
        }
        for (const Vec2& coordinate : definition.chart) {
            if (!std::isfinite(coordinate.x) || !std::isfinite(coordinate.y)) {
                throw std::invalid_argument(
                    "Membrane chart coordinates must be finite");
            }
        }
        const Vec2 edgeOne = definition.chart[1] - definition.chart[0];
        const Vec2 edgeTwo = definition.chart[2] - definition.chart[0];
        const Vec2 edgeThree = definition.chart[2] - definition.chart[1];
        const double determinant = cross(edgeOne, edgeTwo);
        const double maximumEdgeLengthSquared = std::max(
            {lengthSquared(edgeOne),
             lengthSquared(edgeTwo),
             lengthSquared(edgeThree)});
        const double tolerance =
            relativeAreaTolerance * maximumEdgeLengthSquared;
        if (!(determinant > tolerance)) {
            throw std::invalid_argument(
                "Membrane chart must have positive non-degenerate area");
        }
    }
}

std::size_t RectangularPatch::node(std::size_t chordIndex,
                                   std::size_t spanIndex) const {
    if (chordIndex >= chordNodes || spanIndex >= spanNodes) {
        throw std::out_of_range("RectangularPatch node index is out of range");
    }
    return firstNode + spanIndex * chordNodes + chordIndex;
}

std::size_t RectangularCell::lowerNode(std::size_t chordIndex,
                                      std::size_t spanIndex) const {
    if (chordIndex >= chordNodes || spanIndex >= spanNodes) {
        throw std::out_of_range("RectangularCell node index is out of range");
    }
    return firstNode + spanIndex * chordNodes + chordIndex;
}

std::size_t RectangularCell::upperNode(std::size_t chordIndex,
                                      std::size_t spanIndex) const {
    return lowerNode(chordIndex, spanIndex) + chordNodes * spanNodes;
}

std::size_t RectangularMembraneCoupon::node(std::size_t lengthIndex,
                                            std::size_t widthIndex) const {
    if (lengthIndex >= lengthNodes || widthIndex >= widthNodes) {
        throw std::out_of_range(
            "RectangularMembraneCoupon node index is out of range");
    }
    return firstNode + widthIndex * lengthNodes + lengthIndex;
}

std::size_t SoftBody::addNode(const Vec3& position, double mass) {
    if (!(mass > 0.0)) {
        throw std::invalid_argument("A dynamic node must have positive mass");
    }
    nodes_.push_back({position, position, {}, {}, 1.0 / mass});
    return nodes_.size() - 1;
}

std::size_t SoftBody::addFixedNode(const Vec3& position) {
    nodes_.push_back({position, position, {}, {}, 0.0});
    return nodes_.size() - 1;
}

void SoftBody::fixNode(std::size_t nodeIndex) {
    requireNodeIndex(nodeIndex, nodes_.size());
    Node& node = nodes_[nodeIndex];
    node.inverseMass = 0.0;
    node.velocity = {};
    node.previousPosition = node.position;
    // Fixed nodes are roots of the suspension load-path ordering.
    loadPathOrdering_.builtForNodeCount =
        std::numeric_limits<std::size_t>::max();
}

std::size_t SoftBody::addTriangle(std::size_t a,
                                  std::size_t b,
                                  std::size_t c,
                                  double pressureDifference) {
    requireNodeIndex(a, nodes_.size());
    requireNodeIndex(b, nodes_.size());
    requireNodeIndex(c, nodes_.size());
    triangles_.push_back({a, b, c, pressureDifference});
    return triangles_.size() - 1;
}

std::size_t SoftBody::addDistanceConstraint(std::size_t a,
                                            std::size_t b,
                                            double restLength,
                                            double compliance) {
    requireNodeIndex(a, nodes_.size());
    requireNodeIndex(b, nodes_.size());
    if (a == b || restLength < 0.0 || compliance < 0.0) {
        throw std::invalid_argument("Invalid distance constraint");
    }
    constraints_.push_back(
        {a, b, restLength, compliance, 0.0, ConstraintKind::Distance});
    return constraints_.size() - 1;
}

std::size_t SoftBody::addCableConstraint(std::size_t a,
                                         std::size_t b,
                                         double maximumLength,
                                         double compliance) {
    requireNodeIndex(a, nodes_.size());
    requireNodeIndex(b, nodes_.size());
    if (a == b || maximumLength < 0.0 || compliance < 0.0) {
        throw std::invalid_argument("Invalid cable constraint");
    }
    constraints_.push_back(
        {a, b, maximumLength, compliance, 0.0, ConstraintKind::Cable});
    return constraints_.size() - 1;
}

std::size_t SoftBody::addSuspensionTieConstraint(
    std::size_t a,
    std::size_t b,
    double restLength,
    double compliance) {
    requireNodeIndex(a, nodes_.size());
    requireNodeIndex(b, nodes_.size());
    if (a == b || restLength < 0.0 || compliance < 0.0) {
        throw std::invalid_argument("Invalid suspension tie constraint");
    }
    constraints_.push_back(
        {a, b, restLength, compliance, 0.0,
         ConstraintKind::SuspensionTie});
    return constraints_.size() - 1;
}

const std::vector<std::size_t>& SoftBody::loadPathConstraintOrder() const {
    if (loadPathOrdering_.builtForConstraintCount == constraints_.size()
        && loadPathOrdering_.builtForNodeCount == nodes_.size()
        && loadPathOrdering_.builtForTriangleCount == triangles_.size()
        && loadPathOrdering_.builtForDihedralCount
               == dihedralConstraints_.size()) {
        return loadPathOrdering_.constraints;
    }

    LoadPathOrdering rebuilt;
    rebuilt.builtForConstraintCount = constraints_.size();
    rebuilt.builtForNodeCount = nodes_.size();
    rebuilt.builtForTriangleCount = triangles_.size();
    rebuilt.builtForDihedralCount = dihedralConstraints_.size();

    std::vector<std::vector<std::size_t>> adjacency(nodes_.size());
    std::vector<bool> structuralNode(nodes_.size(), false);
    for (std::size_t node = 0; node < nodes_.size(); ++node) {
        structuralNode[node] = nodes_[node].inverseMass == 0.0;
    }
    for (const Triangle& triangle : triangles_) {
        structuralNode[triangle.a] = true;
        structuralNode[triangle.b] = true;
        structuralNode[triangle.c] = true;
    }
    for (std::size_t index = 0; index < constraints_.size(); ++index) {
        const DistanceConstraint& constraint = constraints_[index];
        if (constraint.kind == ConstraintKind::Distance) {
            structuralNode[constraint.a] = true;
            structuralNode[constraint.b] = true;
            continue;
        }
        rebuilt.constraints.push_back(index);
        adjacency[constraint.a].push_back(constraint.b);
        adjacency[constraint.b].push_back(constraint.a);
    }
    for (const DihedralBendingConstraint& constraint :
         dihedralConstraints_) {
        structuralNode[constraint.a] = true;
        structuralNode[constraint.b] = true;
        structuralNode[constraint.c] = true;
        structuralNode[constraint.d] = true;
    }

    constexpr std::size_t unknownDepth =
        std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> depth(nodes_.size(), unknownDepth);
    std::queue<std::size_t> pending;
    for (std::size_t node = 0; node < nodes_.size(); ++node) {
        if (structuralNode[node] && !adjacency[node].empty()) {
            depth[node] = 0;
            pending.push(node);
        }
    }
    const auto visitComponent = [&] {
        while (!pending.empty()) {
            const std::size_t node = pending.front();
            pending.pop();
            for (const std::size_t neighbour : adjacency[node]) {
                if (depth[neighbour] == unknownDepth) {
                    depth[neighbour] = depth[node] + 1;
                    pending.push(neighbour);
                }
            }
        }
    };
    visitComponent();

    // A standalone cable graph may have no structural skin or fixed anchor.
    // Root each such component at its heaviest node so its order is still
    // deterministic and better conditioned than arbitrary insertion order.
    for (;;) {
        std::size_t root = nodes_.size();
        double smallestInverseMass = std::numeric_limits<double>::infinity();
        for (std::size_t node = 0; node < nodes_.size(); ++node) {
            if (depth[node] == unknownDepth && !adjacency[node].empty()
                && nodes_[node].inverseMass < smallestInverseMass) {
                root = node;
                smallestInverseMass = nodes_[node].inverseMass;
            }
        }
        if (root == nodes_.size()) {
            break;
        }
        depth[root] = 0;
        pending.push(root);
        visitComponent();
    }

    const auto constraintDepth = [&](std::size_t index) {
        const DistanceConstraint& constraint = constraints_[index];
        return std::max(depth[constraint.a], depth[constraint.b]);
    };
    std::stable_sort(
        rebuilt.constraints.begin(), rebuilt.constraints.end(),
        [&](std::size_t left, std::size_t right) {
            return constraintDepth(left) < constraintDepth(right);
        });
    loadPathOrdering_ = std::move(rebuilt);
    return loadPathOrdering_.constraints;
}

std::size_t SoftBody::addDihedralBendingConstraint(
    std::size_t a,
    std::size_t b,
    std::size_t c,
    std::size_t d,
    double restAngleRadians,
    double compliance) {
    requireNodeIndex(a, nodes_.size());
    requireNodeIndex(b, nodes_.size());
    requireNodeIndex(c, nodes_.size());
    requireNodeIndex(d, nodes_.size());
    if (a == b || a == c || a == d || b == c || b == d || c == d ||
        !std::isfinite(restAngleRadians) || !std::isfinite(compliance) ||
        compliance < 0.0) {
        throw std::invalid_argument("Invalid dihedral bending constraint");
    }
    dihedralConstraints_.push_back(
        {a, b, c, d, restAngleRadians, compliance, 0.0});
    return dihedralConstraints_.size() - 1;
}

SurfaceGroup SoftBody::surfaceGroup(std::size_t firstTriangle,
                                    std::size_t triangleCount) const {
    if (triangleCount == 0) {
        throw std::invalid_argument("A surface group must contain a triangle");
    }
    if (firstTriangle > triangles_.size() ||
        triangleCount > triangles_.size() - firstTriangle) {
        throw std::out_of_range("Surface group triangle range is out of bounds");
    }
    return SurfaceGroup(this, firstTriangle, triangleCount);
}

MembraneGroup SoftBody::addMembraneElements(
    std::span<const MembraneElementDefinition> definitions) {
    validateMembraneElementDefinitions(
        std::span<const Node>{nodes_}, std::span<const Triangle>{triangles_}, definitions);
    std::vector<MembraneElement> pending;
    pending.reserve(definitions.size());
    for (const MembraneElementDefinition& definition : definitions) {
        const Vec2 firstEdge = definition.chart[1] - definition.chart[0];
        const Vec2 secondEdge = definition.chart[2] - definition.chart[0];
        const Matrix2 referenceMatrix{
            firstEdge.x, secondEdge.x, firstEdge.y, secondEdge.y};
        pending.push_back({definition.triangle,
                           definition.chart,
                           definition.material,
                           definition.role,
                           0.5 * referenceMatrix.determinant(),
                           checkedInverse(referenceMatrix),
                           {},
                           {},
                           0.0,
                           definition.material.complianceMatrix()});
    }
    const std::size_t firstElement = membraneElements_.size();
    membraneElements_.insert(
        membraneElements_.end(), pending.begin(), pending.end());
    return MembraneGroup(this, firstElement, pending.size());
}

std::span<const MembraneElement> SoftBody::membraneElements(
    const MembraneGroup& group) const {
    requireMembraneGroup(group);
    return std::span<const MembraneElement>{
        membraneElements_.data() + group.firstElement_, group.elementCount_};
}

MembraneElementDiagnostics SoftBody::membraneDiagnostics(
    std::size_t elementIndex,
    const Vec3& momentOrigin) const {
    if (elementIndex >= membraneElements_.size()) {
        throw std::out_of_range("Membrane element index is out of range");
    }
    return evaluateMembraneElement(std::span<const Node>{nodes_},
                                   std::span<const Triangle>{triangles_},
                                   membraneElements_[elementIndex],
                                   momentOrigin)
        .diagnostics;
}

MembraneGroupDiagnostics SoftBody::membraneDiagnostics(
    const MembraneGroup& group,
    const Vec3& momentOrigin) const {
    requireMembraneGroup(group);
    MembraneGroupDiagnostics result;
    const std::size_t last = group.firstElement_ + group.elementCount_;
    for (std::size_t elementIndex = group.firstElement_; elementIndex < last;
         ++elementIndex) {
        const MembraneElementDiagnostics diagnostics =
            membraneDiagnostics(elementIndex, momentOrigin);
        result.elasticEnergy += diagnostics.elasticEnergy;
        result.energyByRole[materialRoleIndex(diagnostics.role)] +=
            diagnostics.elasticEnergy;
        result.totalInternalForce += diagnostics.totalInternalForce;
        result.totalInternalMoment += diagnostics.totalInternalMoment;
        result.maximumAbsoluteStrain =
            std::max({result.maximumAbsoluteStrain,
                      std::abs(diagnostics.greenStrain.x),
                      std::abs(diagnostics.greenStrain.y),
                      std::abs(diagnostics.greenStrain.z)});
        result.maximumResidual =
            std::max(result.maximumResidual, diagnostics.normalizedResidual);
    }
    return result;
}

void SoftBody::requireSurfaceGroup(const SurfaceGroup& surface) const {
    if (surface.owner_ != this) {
        throw std::invalid_argument("Surface group belongs to another SoftBody");
    }
    if (surface.triangleCount_ == 0 ||
        surface.firstTriangle_ > triangles_.size() ||
        surface.triangleCount_ > triangles_.size() - surface.firstTriangle_) {
        throw std::out_of_range("Surface group triangle range is no longer valid");
    }
}

void SoftBody::requireMembraneGroup(const MembraneGroup& group) const {
    if (group.owner_ != this) {
        throw std::invalid_argument("Membrane group belongs to another SoftBody");
    }
    if (group.elementCount_ == 0 ||
        group.firstElement_ > membraneElements_.size() ||
        group.elementCount_ > membraneElements_.size() - group.firstElement_) {
        throw std::out_of_range("Membrane group range is no longer valid");
    }
}

void SoftBody::declareInteriorPressurePartitions() {
    interiorPressurePartitions_ = true;
}

namespace {

// A uniform stamp treats every face as an exterior wall. On a body whose
// faces separate two pressurised zones that silently loads each partition
// one-sidedly, which is a body-fixed resultant rather than a visible error --
// so the call is refused instead. Zero is exempt: it prescribes no difference
// across any face and is how the face-aware path clears the field first.
void requireUniformStampIsMeaningful(bool interiorPressurePartitions,
                                     double pressureDifference) {
    if (!interiorPressurePartitions || pressureDifference == 0.0) return;
    throw std::invalid_argument(
        "SoftBody uniform pressure would load the interior partitions of a "
        "multi-zone body one-sidedly; use setUniformCellPressure() instead");
}

} // namespace

void SoftBody::setUniformPressureDifference(double pressureDifference) {
    requireUniformStampIsMeaningful(interiorPressurePartitions_,
                                    pressureDifference);
    for (Triangle& triangle : triangles_) {
        triangle.pressureDifference = pressureDifference;
    }
}

void SoftBody::setUniformPressureDifference(const SurfaceGroup& surface,
                                            double pressureDifference) {
    requireUniformStampIsMeaningful(interiorPressurePartitions_,
                                    pressureDifference);
    requireSurfaceGroup(surface);
    const std::size_t last = surface.firstTriangle_ + surface.triangleCount_;
    for (std::size_t triangleIndex = surface.firstTriangle_; triangleIndex < last;
         ++triangleIndex) {
        triangles_[triangleIndex].pressureDifference = pressureDifference;
    }
}

void SoftBody::setFacePressureDifference(std::size_t triangleIndex,
                                         double pressureDifference) {
    if (triangleIndex >= triangles_.size()) {
        throw std::out_of_range("Triangle index is out of range");
    }
    triangles_[triangleIndex].pressureDifference = pressureDifference;
}

void SoftBody::clearExternalForces() {
    for (Node& node : nodes_) {
        node.force = {};
    }
}

void SoftBody::addForce(std::size_t nodeIndex, const Vec3& force) {
    requireNodeIndex(nodeIndex, nodes_.size());
    nodes_[nodeIndex].force += force;
}

void SoftBody::step(const StepSettings& settings) {
    if (!(settings.timeStep > 0.0) || !std::isfinite(settings.timeStep) ||
        settings.substeps <= 0 ||
        settings.constraintIterations < 0 ||
        settings.cableConstraintSweepPairs < 0 ||
        (!contactPairs_.empty() && settings.constraintIterations == 0) ||
        settings.velocityDampingPerSecond < 0.0 ||
        !std::isfinite(settings.velocityDampingPerSecond) ||
        !std::isfinite(settings.gravity.x) ||
        !std::isfinite(settings.gravity.y) ||
        !std::isfinite(settings.gravity.z) ||
        !std::isfinite(settings.dampingReferenceVelocity.x) ||
        !std::isfinite(settings.dampingReferenceVelocity.y) ||
        !std::isfinite(settings.dampingReferenceVelocity.z) ||
        (!contactPairs_.empty() &&
         (settings.contactCcd.conservativeIterations < 0 ||
          settings.contactCcd.intervalSubdivisions < 0 ||
          !(settings.contactCcd.timeTolerance > 0.0) ||
          !std::isfinite(settings.contactCcd.timeTolerance) ||
          !(settings.contactCcd.distanceTolerance > 0.0) ||
          !std::isfinite(settings.contactCcd.distanceTolerance)))) {
        throw std::invalid_argument("Invalid SoftBody step settings");
    }

    std::vector<Vec3> externalForces;
    externalForces.reserve(nodes_.size());
    for (const Node& node : nodes_) {
        externalForces.push_back(node.force);
    }

    const double substepTime = settings.timeStep / settings.substeps;
    for (int substep = 0; substep < settings.substeps; ++substep) {
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            nodes_[i].force = externalForces[i];
        }
        integrateSubstep(substepTime, settings, nullptr);
    }
    clearExternalForces();
}

void SoftBody::stepCoupled(const StepSettings& settings,
                           SuspensionSystem& suspension) {
    suspension.requireOwner(*this);
    if (!(settings.timeStep > 0.0) || !std::isfinite(settings.timeStep) ||
        settings.substeps <= 0 || settings.constraintIterations < 0 ||
        settings.cableConstraintSweepPairs < 0 ||
        settings.constraintIterations == 0 ||
        settings.velocityDampingPerSecond < 0.0 ||
        !std::isfinite(settings.velocityDampingPerSecond) ||
        !std::isfinite(settings.gravity.x) ||
        !std::isfinite(settings.gravity.y) ||
        !std::isfinite(settings.gravity.z) ||
        !std::isfinite(settings.dampingReferenceVelocity.x) ||
        !std::isfinite(settings.dampingReferenceVelocity.y) ||
        !std::isfinite(settings.dampingReferenceVelocity.z) ||
        (!contactPairs_.empty() &&
         (settings.contactCcd.conservativeIterations < 0 ||
          settings.contactCcd.intervalSubdivisions < 0 ||
          !(settings.contactCcd.timeTolerance > 0.0) ||
          !std::isfinite(settings.contactCcd.timeTolerance) ||
          !(settings.contactCcd.distanceTolerance > 0.0) ||
         !std::isfinite(settings.contactCcd.distanceTolerance)))) {
        throw std::invalid_argument("Invalid coupled SoftBody step settings");
    }
    if (settings.constraintIterations !=
        suspension.definition().solver.lineIterations) {
        throw std::invalid_argument(
            "Coupled structural iterations must equal the registered "
            "suspension lineIterations contract");
    }

    std::vector<Vec3> externalForces;
    externalForces.reserve(nodes_.size());
    for (const Node& node : nodes_) {
        externalForces.push_back(node.force);
    }
    const double substepTime = settings.timeStep / settings.substeps;
    for (int substep = 0; substep < settings.substeps; ++substep) {
        if (nodes_.size() != externalForces.size()) {
            throw SuspensionError(SuspensionPhase::Prediction,
                                  "body-owner",
                                  "body topology changed after suspension build");
        }
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            nodes_[i].force = externalForces[i];
        }
        integrateSubstep(substepTime, settings, &suspension);
    }
    clearExternalForces();
}

void SoftBody::integrateSubstep(double dt,
                                const StepSettings& settings,
                                SuspensionSystem* suspension) {
    StepPerformanceProfile* const profile = settings.performanceProfile;
    PerformanceScope total(profileField(
        profile, &StepPerformanceProfile::softBodyTotalNanoseconds));
    if (profile) ++profile->structuralSubsteps;
    if (contactPairs_.empty() && suspension == nullptr) {
        integrateSubstepTrial(dt, settings, nullptr);
        return;
    }

    const auto snapshotStart = profile ? PerformanceClock::now()
                                       : PerformanceClock::time_point{};
    const std::vector<Node> savedNodes = nodes_;
    const std::vector<DistanceConstraint> savedConstraints = constraints_;
    const std::vector<MembraneElement> savedMembranes = membraneElements_;
    const std::vector<DihedralBendingConstraint> savedDihedrals =
        dihedralConstraints_;
    const auto savedMultipliers = contactMultipliers_;
    const auto savedRecords = contactRecords_;
    const ContactDiagnostics savedDiagnostics = contactDiagnostics_;
    const auto savedPairDiagnostics = contactPairDiagnostics_;
    if (profile) {
        profile->softBodyTransactionSnapshotNanoseconds +=
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    PerformanceClock::now() - snapshotStart).count());
    }
    const auto rollbackFailure = [&](const ContactFeatureKey& key) {
        nodes_ = savedNodes;
        constraints_ = savedConstraints;
        membraneElements_ = savedMembranes;
        dihedralConstraints_ = savedDihedrals;
        contactMultipliers_ = savedMultipliers;
        contactRecords_ = savedRecords;
        contactDiagnostics_ = savedDiagnostics;
        contactPairDiagnostics_ = savedPairDiagnostics;
        if (contactPairDiagnostics_.size() != contactPairs_.size()) {
            contactPairDiagnostics_.assign(contactPairs_.size(), {});
        }
        if (suspension != nullptr) {
            suspension->rollbackSubstep();
        }
        if (contactPairs_.empty()) {
            return;
        }
        contactDiagnostics_.registered = true;
        contactDiagnostics_.solveSucceeded = false;
        contactDiagnostics_.indeterminateCount = 1;
        contactDiagnostics_.hasFailure = true;
        contactDiagnostics_.failureKey = key;
        for (ContactDiagnostics& pairDiagnostic : contactPairDiagnostics_) {
            pairDiagnostic.registered = true;
        }
        if (key.pair < contactPairDiagnostics_.size()) {
            ContactDiagnostics& pairDiagnostic =
                contactPairDiagnostics_[key.pair];
            pairDiagnostic.solveSucceeded = false;
            pairDiagnostic.indeterminateCount = 1;
            pairDiagnostic.hasFailure = true;
            pairDiagnostic.failureKey = key;
        }
    };
    try {
        integrateSubstepTrial(dt, settings, suspension);
    } catch (const ContactStepError& error) {
        rollbackFailure(error.key());
        if (suspension != nullptr) {
            suspension->recordFailure(SuspensionPhase::Certification,
                                      "fabric-contact-ccd");
        }
        throw;
    } catch (const SuspensionError& error) {
        rollbackFailure({});
        if (suspension != nullptr) {
            suspension->recordFailure(error.phase(), error.entity());
        }
        throw;
    } catch (const std::exception& error) {
        const ContactFeatureKey key{};
        rollbackFailure(key);
        if (suspension != nullptr) {
            suspension->recordFailure(SuspensionPhase::Certification,
                                      "unexpected-exception");
        }
        if (contactPairs_.empty()) {
            throw;
        }
        throw ContactStepError(
            key,
            std::string("Contact-enabled substep failed: ") + error.what());
    } catch (...) {
        const ContactFeatureKey key{};
        rollbackFailure(key);
        if (contactPairs_.empty()) {
            throw;
        }
        throw ContactStepError(key,
                               "Contact-enabled substep failed unexpectedly");
    }
}

void SoftBody::integrateSubstepTrial(double dt,
                                     const StepSettings& settings,
                                     SuspensionSystem* suspension) {
    StepPerformanceProfile* const profile = settings.performanceProfile;
    WorkerPool* const pool = poolFor(settings);
    bool preserveFreeMembraneMomentum = false;
    MomentumState predictedMomentum;
    {
        PerformanceScope prediction(profileField(
            profile, &StepPerformanceProfile::predictionNanoseconds));
        accumulatePressureForces(pool);
        if (suspension != nullptr) {
            suspension->beginSubstep(*this, dt, settings.gravity);
        }
        const double damping =
            std::exp(-settings.velocityDampingPerSecond * dt);
        predictPositions(dt, settings, damping, pool);
        // Solver-drift momentum restoration applies only to isolated free
        // membranes. A participating suspension applies external line
        // impulses inside the bracketed constraint loop with their reaction
        // on the payload, so the guard stands down rather than erase them.
        preserveFreeMembraneMomentum =
            suspension == nullptr
            && (!membraneElements_.empty() || !dihedralConstraints_.empty()) &&
            contactPairs_.empty() &&
            std::all_of(nodes_.begin(), nodes_.end(), [](const Node& node) {
                return node.inverseMass > 0.0;
            });
        predictedMomentum = preserveFreeMembraneMomentum
            ? measureMomentum(std::span<const Node>{nodes_})
            : MomentumState{};

        for (DistanceConstraint& constraint : constraints_) {
            constraint.accumulatedLambda = 0.0;
        }
        for (MembraneElement& element : membraneElements_) {
            element.multiplier = {};
            element.solverResultantEstimate = {};
            element.normalizedResidual = 0.0;
        }
        for (DihedralBendingConstraint& constraint : dihedralConstraints_) {
            constraint.accumulatedLambda = 0.0;
        }
        if (!contactPairs_.empty()) beginContactSubstep();
    }

    const std::vector<std::size_t>& loadPathOrder =
        loadPathConstraintOrder();
    if (profile) {
        profile->constraintIterations +=
            static_cast<std::uint64_t>(settings.constraintIterations);
        profile->distanceConstraintVisits +=
            static_cast<std::uint64_t>(settings.constraintIterations) *
            constraints_.size();
        profile->cableConstraintVisits +=
            2ULL
            * static_cast<std::uint64_t>(settings.cableConstraintSweepPairs)
            * loadPathOrder.size();
        profile->membraneConstraintVisits +=
            static_cast<std::uint64_t>(settings.constraintIterations) *
            membraneElements_.size();
        profile->bendingConstraintVisits +=
            static_cast<std::uint64_t>(settings.constraintIterations) *
            dihedralConstraints_.size();
        if (suspension != nullptr)
            profile->suspensionConstraintVisits +=
                static_cast<std::uint64_t>(settings.constraintIterations) *
                suspension->segments().size();
        if (!contactPairs_.empty())
            profile->contactConstraintVisits +=
                static_cast<std::uint64_t>(settings.constraintIterations);
    }

    // Membrane, bending, contact and suspension all move nodes inside the iteration
    // loop, so the packed constraint state is only coherent when none of them
    // is present -- which is exactly the mass-spring cloth case it is for.
    const bool packedConstraints =
        !constraints_.empty() && membraneElements_.empty()
        && dihedralConstraints_.empty() &&
        contactPairs_.empty() && suspension == nullptr;
    if (packedConstraints) {
        PerformanceScope distance(profileField(
            profile, &StepPerformanceProfile::distanceConstraintNanoseconds));
        packConstraintSolveNodes(pool);
    }

    // Loop-invariant, and a division the sweep would otherwise redo for every
    // constraint on every iteration.
    const double inverseTimeStepSquared = 1.0 / (dt * dt);

    const auto solveCablePair = [&] {
        PerformanceScope cable(profileField(
            profile, &StepPerformanceProfile::cableConstraintNanoseconds));
        const auto solveLoadPathConstraint = [&](std::size_t index) {
            DistanceConstraint& constraint = constraints_[index];
            if (packedConstraints) {
                solveConstraintPacked(constraint, inverseTimeStepSquared);
            } else {
                solveConstraint(constraint, inverseTimeStepSquared);
            }
        };
        // Reverse depth order propagates the payload reaction to the canopy;
        // forward depth order returns the corrected support to the payload.
        for (auto constraint = loadPathOrder.rbegin();
             constraint != loadPathOrder.rend(); ++constraint) {
            solveLoadPathConstraint(*constraint);
        }
        for (const std::size_t constraint : loadPathOrder) {
            solveLoadPathConstraint(constraint);
        }
    };

    // Reserve three quarters of the pairs for final load-path closure. Canopy
    // attachment and
    // harness ties are solved late in a general forward sweep; without this
    // closing passes they can move a cable endpoint after that cable's last
    // visit, leaving centimetres of apparent line stretch. One final pair
    // reduced but did not converge branching cascades; several are required
    // after the final cloth movement. The remaining quarter stays interleaved
    // before general sweeps so the cloth still reacts to suspension load
    // within the same substep.
    const int closingCablePairs =
        settings.cableConstraintSweepPairs > 0
            ? settings.cableConstraintSweepPairs
                  - settings.cableConstraintSweepPairs / 4
            : 0;
    const int interleavedCablePairs =
        settings.cableConstraintSweepPairs - closingCablePairs;
    for (int iteration = 0; iteration < settings.constraintIterations; ++iteration) {
        // Distribute the cheap graph-depth passes BEFORE the general sweep.
        // The skin/rib constraints below can then react to the line load in
        // this same iteration instead of receiving it only after their final
        // solve of the substep.
        const int cablePairs =
            interleavedCablePairs
                / std::max(1, settings.constraintIterations)
            + (iteration
                   < interleavedCablePairs
                         % std::max(1, settings.constraintIterations)
                   ? 1
                   : 0);
        for (int pair = 0; pair < cablePairs; ++pair) {
            solveCablePair();
        }
        {
            PerformanceScope distance(profileField(
                profile,
                &StepPerformanceProfile::distanceConstraintNanoseconds));
            if (pool != nullptr && !constraints_.empty()) {
                solveConstraintsColoured(
                    inverseTimeStepSquared, *pool, packedConstraints);
            } else if (packedConstraints) {
                for (DistanceConstraint& constraint : constraints_) {
                    solveConstraintPacked(constraint, inverseTimeStepSquared);
                }
            } else {
                for (DistanceConstraint& constraint : constraints_) {
                    solveConstraint(constraint, inverseTimeStepSquared);
                }
            }
        }
        {
            PerformanceScope membrane(profileField(
                profile,
                &StepPerformanceProfile::membraneConstraintNanoseconds));
            if (pool != nullptr) {
                if (settings.parallelMembraneMode ==
                    ParallelMembraneMode::Jacobi) {
                    solveMembraneJacobi(dt, *pool);
                } else {
                    solveMembraneColoured(dt, *pool);
                }
            } else {
                for (MembraneElement& element : membraneElements_) {
                    solveMembraneElement(element, dt);
                }
            }
        }
        {
            PerformanceScope bending(profileField(
                profile,
                &StepPerformanceProfile::bendingConstraintNanoseconds));
            // Deliberately serial for the initial bounded hinge path. Stable
            // insertion order is deterministic and avoids extending the
            // membrane colouring with four-node adjacency prematurely.
            for (DihedralBendingConstraint& constraint :
                 dihedralConstraints_) {
                solveDihedralConstraint(constraint,
                                        inverseTimeStepSquared);
            }
        }
        if (suspension != nullptr) {
            PerformanceScope line(profileField(
                profile,
                &StepPerformanceProfile::suspensionConstraintNanoseconds));
            suspension->solveLineIteration(*this, dt);
            suspension->solveGroundIteration(dt);
        }
        if (!contactPairs_.empty()) {
            PerformanceScope contact(profileField(
                profile,
                &StepPerformanceProfile::contactConstraintNanoseconds));
            solveContactIteration(dt, settings);
        }
    }
    if (settings.constraintIterations > 0) {
        for (int pair = 0; pair < closingCablePairs; ++pair) {
            solveCablePair();
        }
    } else if (settings.constraintIterations == 0) {
        for (int pair = 0; pair < settings.cableConstraintSweepPairs; ++pair) {
            solveCablePair();
        }
    }
    if (packedConstraints) {
        PerformanceScope distance(profileField(
            profile, &StepPerformanceProfile::distanceConstraintNanoseconds));
        unpackConstraintSolveNodes(pool);
    }
    if (!contactPairs_.empty()) {
        PerformanceScope contactCertification(profileField(
            profile,
            &StepPerformanceProfile::contactCertificationNanoseconds));
        // Preserve the accepted contact path: certification publishes the
        // records consumed and augmented by the friction phase below.
        certifyContactState(dt, settings);
    }
    // Each element reads shared node state but writes only its own diagnostic
    // slots, so this pass needs no colouring and stays bit-identical to the
    // serial one whatever the worker count.
    {
        PerformanceScope diagnostics(profileField(
            profile,
            &StepPerformanceProfile::membraneDiagnosticsNanoseconds));
        if (pool != nullptr) {
            pool->forEachRange(
                membraneElements_.size(),
                [&](std::size_t begin, std::size_t end) {
                    for (std::size_t index = begin; index < end; ++index) {
                        updateMembraneSolverDiagnostics(
                            membraneElements_[index], dt);
                    }
                });
        } else {
            for (MembraneElement& element : membraneElements_) {
                updateMembraneSolverDiagnostics(element, dt);
            }
        }
    }

    {
        PerformanceScope finalization(profileField(
            profile, &StepPerformanceProfile::finalizationNanoseconds));
        finalizeVelocities(dt, pool);
        if (preserveFreeMembraneMomentum) {
            restoreMomentum(std::span<Node>{nodes_}, predictedMomentum);
        }
        if (suspension != nullptr) {
            suspension->reconstructPayloadVelocity(dt);
            suspension->applyLineDamping(*this, dt);
        }
        if (!contactPairs_.empty()) applyContactFriction();
        if (suspension != nullptr) {
            suspension->certifySubstep(*this, dt);
            suspension->finishSubstep(*this, dt);
        }
    }

}

const SoftBody::TriangleIncidence& SoftBody::triangleIncidence() const {
    TriangleIncidence& incidence = triangleIncidence_;
    if (incidence.builtForTriangleCount == triangles_.size() &&
        incidence.builtForNodeCount == nodes_.size() &&
        !triangles_.empty()) {
        return incidence;
    }
    incidence.nodeOffsets.assign(nodes_.size() + 1, 0);
    for (const Triangle& triangle : triangles_) {
        ++incidence.nodeOffsets[triangle.a + 1];
        ++incidence.nodeOffsets[triangle.b + 1];
        ++incidence.nodeOffsets[triangle.c + 1];
    }
    for (std::size_t node = 0; node < nodes_.size(); ++node) {
        incidence.nodeOffsets[node + 1] += incidence.nodeOffsets[node];
    }
    incidence.triangles.assign(3 * triangles_.size(), 0);
    std::vector<std::size_t> cursor(incidence.nodeOffsets.begin(),
                                    incidence.nodeOffsets.end() - 1);
    for (std::size_t index = 0; index < triangles_.size(); ++index) {
        const Triangle& triangle = triangles_[index];
        for (const std::size_t node : {triangle.a, triangle.b, triangle.c}) {
            incidence.triangles[cursor[node]++] = index;
        }
    }
    incidence.builtForTriangleCount = triangles_.size();
    incidence.builtForNodeCount = nodes_.size();
    return incidence;
}

void SoftBody::accumulatePressureForces(WorkerPool* pool) {
    if (triangles_.empty()) {
        return;
    }
    if (pool == nullptr) {
        for (const Triangle& triangle : triangles_) {
            const Vec3& a = nodes_[triangle.a].position;
            const Vec3& b = nodes_[triangle.b].position;
            const Vec3& c = nodes_[triangle.c].position;
            const Vec3 areaVector = 0.5 * cross(b - a, c - a);
            const Vec3 nodalForce =
                triangle.pressureDifference * areaVector / 3.0;
            nodes_[triangle.a].force += nodalForce;
            nodes_[triangle.b].force += nodalForce;
            nodes_[triangle.c].force += nodalForce;
        }
        return;
    }

    const TriangleIncidence& incidence = triangleIncidence();
    pool->forEachRange(
        nodes_.size(), [&](std::size_t begin, std::size_t end) {
            for (std::size_t node = begin; node < end; ++node) {
                // Starts from the force already there, which is where the
                // scatter started too -- that is what makes the two agree to
                // the last bit rather than merely to the last few.
                Vec3 total = nodes_[node].force;
                const std::size_t first = incidence.nodeOffsets[node];
                const std::size_t last = incidence.nodeOffsets[node + 1];
                for (std::size_t slot = first; slot < last; ++slot) {
                    const Triangle& triangle =
                        triangles_[incidence.triangles[slot]];
                    const Vec3& a = nodes_[triangle.a].position;
                    const Vec3& b = nodes_[triangle.b].position;
                    const Vec3& c = nodes_[triangle.c].position;
                    const Vec3 areaVector = 0.5 * cross(b - a, c - a);
                    total +=
                        triangle.pressureDifference * areaVector / 3.0;
                }
                nodes_[node].force = total;
            }
        });
}

void SoftBody::predictPositions(double dt,
                                const StepSettings& settings,
                                double damping,
                                WorkerPool* pool) {
    // Damping decays velocity toward the reference, not toward zero. With
    // the default zero reference this is bit-for-bit the historical
    // `(velocity + a·dt) * damping`: `x - Vec3{}` and `Vec3{} + x` do not
    // perturb any finite double.
    const Vec3 reference = settings.dampingReferenceVelocity;
    const auto predict = [&](std::size_t begin, std::size_t end) {
        for (std::size_t index = begin; index < end; ++index) {
            Node& node = nodes_[index];
            node.previousPosition = node.position;
            if (node.inverseMass == 0.0) {
                node.velocity = {};
                continue;
            }
            const Vec3 acceleration =
                settings.gravity + node.force * node.inverseMass;
            node.velocity =
                reference +
                (node.velocity + acceleration * dt - reference) * damping;
            node.position += node.velocity * dt;
        }
    };
    if (pool == nullptr) {
        predict(0, nodes_.size());
        return;
    }
    pool->forEachRange(nodes_.size(), predict);
}

void SoftBody::finalizeVelocities(double dt, WorkerPool* pool) {
    const double inverseTimeStep = 1.0 / dt;
    const auto finalize = [&](std::size_t begin, std::size_t end) {
        for (std::size_t index = begin; index < end; ++index) {
            Node& node = nodes_[index];
            node.velocity =
                node.inverseMass == 0.0
                    ? Vec3{}
                    : (node.position - node.previousPosition) *
                          inverseTimeStep;
        }
    };
    if (pool == nullptr) {
        finalize(0, nodes_.size());
        return;
    }
    pool->forEachRange(nodes_.size(), finalize);
}

namespace {

// One XPBD distance/cable projection. Both sweeps -- over Node and over the
// packed SolveNode copy -- route through this, so there is exactly one copy
// of the physics and the two paths cannot drift apart.
// The sweep is division-latency bound rather than cache bound, so the
// divisions are counted: `alpha` takes the caller's precomputed 1/dt^2
// instead of dividing per constraint, and the unit direction is never
// materialised -- one reciprocal folded into the correction scale replaces
// the three divisions of `difference / currentLength`. That leaves two
// divisions and the square root, down from five and one.
inline void projectDistanceConstraint(DistanceConstraint& constraint,
                                      Vec3& positionA,
                                      double inverseMassA,
                                      Vec3& positionB,
                                      double inverseMassB,
                                      double inverseTimeStepSquared) {
    const Vec3 difference = positionB - positionA;
    const double currentLength = length(difference);
    if (currentLength <= kMinimumLength) {
        return;
    }

    const double constraintValue = currentLength - constraint.restLength;
    if (constraint.kind == ConstraintKind::Cable && constraintValue <= 0.0 &&
        constraint.accumulatedLambda == 0.0) {
        return;
    }

    const double inverseMassSum = inverseMassA + inverseMassB;
    const double alpha = constraint.compliance * inverseTimeStepSquared;
    if (inverseMassSum + alpha <= 0.0) {
        return;
    }

    const double deltaLambda =
        (-constraintValue - alpha * constraint.accumulatedLambda) /
        (inverseMassSum + alpha);

    double appliedDelta = deltaLambda;
    if (constraint.kind == ConstraintKind::Cable) {
        // With this gradient convention, a tensile multiplier is negative.
        const double newLambda =
            std::min(0.0, constraint.accumulatedLambda + deltaLambda);
        appliedDelta = newLambda - constraint.accumulatedLambda;
        constraint.accumulatedLambda = newLambda;
    } else {
        constraint.accumulatedLambda += deltaLambda;
    }

    const double scale = appliedDelta / currentLength;
    positionA -= (inverseMassA * scale) * difference;
    positionB += (inverseMassB * scale) * difference;
}

struct DihedralEvaluation {
    double angle = 0.0;
    std::array<Vec3, 4> gradient{};
    bool valid = false;
};

DihedralEvaluation evaluateDihedral(const Vec3& a,
                                    const Vec3& b,
                                    const Vec3& c,
                                    const Vec3& d) {
    const Vec3 edge = b - a;
    const Vec3 toC = c - a;
    const Vec3 toD = d - a;
    const Vec3 firstArea = cross(edge, toC);
    const Vec3 secondArea = cross(toD, edge);
    const double edgeLength = length(edge);
    const double firstLength = length(firstArea);
    const double secondLength = length(secondArea);
    if (!(edgeLength > kMinimumLength) ||
        !(firstLength > kMinimumLength * edgeLength) ||
        !(secondLength > kMinimumLength * edgeLength)) {
        return {};
    }

    const Vec3 edgeUnit = edge / edgeLength;
    const Vec3 firstNormal = firstArea / firstLength;
    const Vec3 secondNormal = secondArea / secondLength;
    const double cosine = std::clamp(dot(firstNormal, secondNormal), -1.0, 1.0);
    const double sine = dot(edgeUnit, cross(firstNormal, secondNormal));

    // Reverse-mode derivative of atan2(sine, cosine). Unit normals keep
    // sine^2+cosine^2 at one, so dtheta = cosine*dsine-sine*dcosine.
    const Vec3 normalOneAdjoint =
        cosine * cross(secondNormal, edgeUnit) - sine * secondNormal;
    const Vec3 normalTwoAdjoint =
        cosine * cross(edgeUnit, firstNormal) - sine * firstNormal;
    const Vec3 edgeUnitAdjoint =
        cosine * cross(firstNormal, secondNormal);
    const Vec3 areaOneAdjoint =
        (normalOneAdjoint
         - dot(normalOneAdjoint, firstNormal) * firstNormal)
        / firstLength;
    const Vec3 areaTwoAdjoint =
        (normalTwoAdjoint
         - dot(normalTwoAdjoint, secondNormal) * secondNormal)
        / secondLength;
    Vec3 edgeAdjoint =
        (edgeUnitAdjoint - dot(edgeUnitAdjoint, edgeUnit) * edgeUnit)
        / edgeLength;
    edgeAdjoint += cross(toC, areaOneAdjoint);
    const Vec3 cAdjoint = cross(areaOneAdjoint, edge);
    edgeAdjoint += cross(areaTwoAdjoint, toD);
    const Vec3 dAdjoint = cross(edge, areaTwoAdjoint);

    DihedralEvaluation result;
    result.angle = std::atan2(sine, cosine);
    result.gradient[1] = edgeAdjoint;
    result.gradient[2] = cAdjoint;
    result.gradient[3] = dAdjoint;
    result.gradient[0] = -1.0 * (edgeAdjoint + cAdjoint + dAdjoint);
    result.valid = std::isfinite(result.angle)
                   && std::all_of(result.gradient.begin(),
                                  result.gradient.end(),
                                  [](const Vec3& value) {
                                      return std::isfinite(value.x)
                                             && std::isfinite(value.y)
                                             && std::isfinite(value.z);
                                  });
    return result;
}

}  // namespace

DihedralBendingDiagnostics SoftBody::dihedralDiagnostics(
    std::size_t constraintIndex) const {
    if (constraintIndex >= dihedralConstraints_.size()) {
        throw std::out_of_range("Dihedral constraint index is out of range");
    }
    const DihedralBendingConstraint& constraint =
        dihedralConstraints_[constraintIndex];
    const DihedralEvaluation evaluation = evaluateDihedral(
        nodes_[constraint.a].position,
        nodes_[constraint.b].position,
        nodes_[constraint.c].position,
        nodes_[constraint.d].position);
    return {evaluation.angle, evaluation.gradient, evaluation.valid};
}

void SoftBody::solveConstraint(DistanceConstraint& constraint,
                               double inverseTimeStepSquared) {
    Node& a = nodes_[constraint.a];
    Node& b = nodes_[constraint.b];
    projectDistanceConstraint(constraint,
                              a.position,
                              a.inverseMass,
                              b.position,
                              b.inverseMass,
                              inverseTimeStepSquared);
}

void SoftBody::solveConstraintPacked(DistanceConstraint& constraint,
                                     double inverseTimeStepSquared) {
    SolveNode& a = constraintSolveNodes_[constraint.a];
    SolveNode& b = constraintSolveNodes_[constraint.b];
    projectDistanceConstraint(constraint,
                              a.position,
                              a.inverseMass,
                              b.position,
                              b.inverseMass,
                              inverseTimeStepSquared);
}

void SoftBody::solveDihedralConstraint(
    DihedralBendingConstraint& constraint,
    double inverseTimeStepSquared) {
    const std::array<std::size_t, 4> indices{
        constraint.a, constraint.b, constraint.c, constraint.d};
    const DihedralEvaluation evaluation = evaluateDihedral(
        nodes_[constraint.a].position,
        nodes_[constraint.b].position,
        nodes_[constraint.c].position,
        nodes_[constraint.d].position);
    if (!evaluation.valid) {
        return;
    }

    double inverseMassGradient = 0.0;
    for (std::size_t corner = 0; corner < indices.size(); ++corner) {
        inverseMassGradient += nodes_[indices[corner]].inverseMass
                               * lengthSquared(evaluation.gradient[corner]);
    }
    const double alpha = constraint.compliance * inverseTimeStepSquared;
    const double denominator = inverseMassGradient + alpha;
    if (!(denominator > 0.0) || !std::isfinite(denominator)) {
        return;
    }
    constexpr double twoPi = 6.283185307179586476925286766559;
    const double value =
        std::remainder(evaluation.angle - constraint.restAngleRadians, twoPi);
    const double deltaLambda =
        (-value - alpha * constraint.accumulatedLambda) / denominator;
    if (!std::isfinite(deltaLambda)) {
        return;
    }
    for (std::size_t corner = 0; corner < indices.size(); ++corner) {
        Node& node = nodes_[indices[corner]];
        node.position += node.inverseMass * deltaLambda
                         * evaluation.gradient[corner];
    }
    constraint.accumulatedLambda += deltaLambda;
}

void SoftBody::packConstraintSolveNodes(WorkerPool* pool) {
    constraintSolveNodes_.resize(nodes_.size());
    const auto pack = [&](std::size_t begin, std::size_t end) {
        for (std::size_t index = begin; index < end; ++index) {
            constraintSolveNodes_[index] = {nodes_[index].position,
                                            nodes_[index].inverseMass};
        }
    };
    if (pool == nullptr) {
        pack(0, nodes_.size());
        return;
    }
    pool->forEachRange(nodes_.size(), pack);
}

void SoftBody::unpackConstraintSolveNodes(WorkerPool* pool) {
    const auto unpack = [&](std::size_t begin, std::size_t end) {
        for (std::size_t index = begin; index < end; ++index) {
            nodes_[index].position = constraintSolveNodes_[index].position;
        }
    };
    if (pool == nullptr) {
        unpack(0, nodes_.size());
        return;
    }
    pool->forEachRange(nodes_.size(), unpack);
}

std::array<Vec3, 3> SoftBody::membraneElementCorrections(
    MembraneElement& element,
    double dt) {
    const MembraneEvaluation evaluation = evaluateMembraneElement(
        std::span<const Node>{nodes_},
        std::span<const Triangle>{triangles_},
        element,
        {},
        MembraneEvaluationDetail::StrainOnly);
    const Triangle& triangle = triangles_[element.triangle];
    const std::array<std::size_t, 3> nodeIndices{
        triangle.a, triangle.b, triangle.c};
    const double areaScale = std::sqrt(element.referenceArea);
    std::array<std::array<Vec3, 3>, 3> jacobian =
        evaluation.strainGradients;
    for (auto& component : jacobian) {
        for (Vec3& gradient : component) {
            gradient *= areaScale;
        }
    }

    SymmetricMatrix3 inverseMassMatrix;
    for (std::size_t corner = 0; corner < 3; ++corner) {
        const double inverseMass = nodes_[nodeIndices[corner]].inverseMass;
        inverseMassMatrix.xx +=
            inverseMass * dot(jacobian[0][corner], jacobian[0][corner]);
        inverseMassMatrix.yy +=
            inverseMass * dot(jacobian[1][corner], jacobian[1][corner]);
        inverseMassMatrix.zz +=
            inverseMass * dot(jacobian[2][corner], jacobian[2][corner]);
        inverseMassMatrix.xy +=
            inverseMass * dot(jacobian[0][corner], jacobian[1][corner]);
        inverseMassMatrix.xz +=
            inverseMass * dot(jacobian[0][corner], jacobian[2][corner]);
        inverseMassMatrix.yz +=
            inverseMass * dot(jacobian[1][corner], jacobian[2][corner]);
    }

    Vec3 dampingMotion;
    for (std::size_t corner = 0; corner < 3; ++corner) {
        const Node& node = nodes_[nodeIndices[corner]];
        const Vec3 displacement = node.position - node.previousPosition;
        dampingMotion.x += dot(jacobian[0][corner], displacement);
        dampingMotion.y += dot(jacobian[1][corner], displacement);
        dampingMotion.z += dot(jacobian[2][corner], displacement);
    }

    const double inverseTimeStepSquared = 1.0 / (dt * dt);
    const SymmetricMatrix3 stateCompliance =
        element.material.compressionStiffnessRatio == 1.0
            ? element.complianceMatrix
            : checkedInverse(effectiveMembraneStiffness(
                  element.material, evaluation.diagnostics.greenStrain));
    const SymmetricMatrix3 alphaTilde =
        inverseTimeStepSquared * stateCompliance;
    const double gamma = element.material.dampingTime / dt;
    const SymmetricMatrix3 system =
        (1.0 + gamma) * inverseMassMatrix + alphaTilde;
    const Vec3 constraintValue =
        areaScale * evaluation.diagnostics.greenStrain;
    const Vec3 rightHandSide =
        -constraintValue - alphaTilde * element.multiplier -
        gamma * dampingMotion;
    Vec3 deltaLambda;
    try {
        deltaLambda = checkedSolve(system, rightHandSide);
    } catch (const std::invalid_argument& error) {
        std::ostringstream message;
        message.imbue(std::locale::classic());
        message << error.what() << "; membrane triangle=" << element.triangle
                << " reference_area=" << std::setprecision(17)
                << element.referenceArea << " dt=" << dt
                << " system=[" << system.xx << ',' << system.yy << ','
                << system.zz << ',' << system.xy << ',' << system.xz << ','
                << system.yz << "] rhs=[" << rightHandSide.x << ','
                << rightHandSide.y << ',' << rightHandSide.z << "] nodes=";
        for (const std::size_t nodeIndex : nodeIndices) {
            const Node& node = nodes_[nodeIndex];
            message << nodeIndex << "@(" << node.position.x << ','
                    << node.position.y << ',' << node.position.z << ")/"
                    << node.inverseMass << ';';
        }
        throw std::invalid_argument(message.str());
    }

    std::array<Vec3, 3> corrections{};
    for (std::size_t corner = 0; corner < 3; ++corner) {
        const Node& node = nodes_[nodeIndices[corner]];
        const Vec3 correction =
            jacobian[0][corner] * deltaLambda.x +
            jacobian[1][corner] * deltaLambda.y +
            jacobian[2][corner] * deltaLambda.z;
        corrections[corner] = node.inverseMass * correction;
    }
    element.multiplier += deltaLambda;
    return corrections;
}

void SoftBody::solveMembraneElement(MembraneElement& element, double dt) {
    const Triangle& triangle = triangles_[element.triangle];
    const std::array<std::size_t, 3> nodeIndices{
        triangle.a, triangle.b, triangle.c};
    const std::array<Vec3, 3> corrections =
        membraneElementCorrections(element, dt);
    for (std::size_t corner = 0; corner < 3; ++corner) {
        nodes_[nodeIndices[corner]].position += corrections[corner];
    }
}

namespace {

constexpr std::size_t kElementsPerBarrier = 128;
constexpr std::size_t kMinimumElementsPerWorker = 16;
constexpr std::size_t kColouredChunkGrain = 8;
// A distance-constraint solve is a fraction of a membrane element's work, so
// a colour has to be correspondingly wider before a barrier pays for itself.
constexpr std::size_t kConstraintsPerBarrier = 1024;
constexpr std::size_t kMinimumConstraintsPerWorker = 256;
// ...and correspondingly, a chunk has to hold many more of them before the
// claim atomic stops dominating. At the membrane grain of 8 a chunk is barely
// a hundred nanoseconds of work, and the cache line holding the chunk cursor
// ping-pongs between cores faster than the constraints are solved.
constexpr std::size_t kMinimumConstraintChunk = 64;
constexpr std::size_t kConstraintClaimsPerWorker = 4;

}  // namespace

WorkerPool* SoftBody::poolFor(const StepSettings& settings) {
    // Ordering must not hinge on the worker count, or a 2-core machine and a
    // 24-core one would run different physics from the same settings. The mode
    // explicitly selects physics; workerThreads only sizes that mode's pool.
    if (settings.workerThreads == 0 ||
        (membraneElements_.empty() && constraints_.empty())) {
        return nullptr;
    }
    // One pool serves every sweep in the substep, so it is sized by the most
    // constrained of them. Each sweep is bit-identical at any worker count, so
    // sharing a count costs nothing but a little parallelism.
    unsigned cap = std::max(1u, settings.workerThreads);
    if (!membraneElements_.empty() &&
        settings.parallelMembraneMode != ParallelMembraneMode::Jacobi) {
        cap = membraneColouring().workerCap(cap);
    }
    if (!constraints_.empty()) {
        cap = constraintColouring().workerCap(cap);
    }
    return workerPool_.get(cap);
}

unsigned SoftBody::ConstraintColouring::workerCap(unsigned requested) const {
    const std::size_t affordable = std::max<std::size_t>(
        1, largestColour / kMinimumConstraintsPerWorker);
    return static_cast<unsigned>(
        std::min<std::size_t>(requested, affordable));
}

const SoftBody::ConstraintColouring& SoftBody::constraintColouring() const {
    ConstraintColouring& colouring = constraintColouring_;
    if (colouring.builtForConstraintCount == constraints_.size() &&
        colouring.builtForNodeCount == nodes_.size() &&
        !constraints_.empty()) {
        return colouring;
    }

    // Greedy first-fit over the line graph. A constraint conflicts only with
    // the others touching one of its two nodes, so the smallest free colour is
    // below degree(a) + degree(b) - 1; twice the largest degree bounds every
    // constraint at once and sizes the per-node mask up front. A single
    // 64-bit word would not do: rib hubs routinely exceed 64 spokes.
    std::vector<std::size_t> degree(nodes_.size(), 0);
    for (const DistanceConstraint& constraint : constraints_) {
        ++degree[constraint.a];
        ++degree[constraint.b];
    }
    const std::size_t maximumDegree =
        degree.empty() ? 0 : *std::max_element(degree.begin(), degree.end());
    const std::size_t colourBound = 2 * maximumDegree + 1;
    const std::size_t words = std::max<std::size_t>(1, (colourBound + 63) / 64);
    std::vector<std::uint64_t> nodeColours(nodes_.size() * words, 0);

    std::vector<std::size_t> colourOf(constraints_.size(), 0);
    std::size_t colourCount = 0;
    for (std::size_t index = 0; index < constraints_.size(); ++index) {
        const DistanceConstraint& constraint = constraints_[index];
        const std::uint64_t* const a = &nodeColours[constraint.a * words];
        const std::uint64_t* const b = &nodeColours[constraint.b * words];
        std::size_t colour = 0;
        for (std::size_t word = 0; word < words; ++word) {
            const std::uint64_t used = a[word] | b[word];
            if (used != ~std::uint64_t{0}) {
                colour = 64 * word +
                         static_cast<std::size_t>(std::countr_one(used));
                break;
            }
            colour = 64 * (word + 1);
        }
        if (colour >= colourBound) {
            throw std::logic_error(
                "Constraint colouring exceeded its degree bound");
        }
        const std::uint64_t bit = std::uint64_t{1} << (colour % 64);
        nodeColours[constraint.a * words + colour / 64] |= bit;
        nodeColours[constraint.b * words + colour / 64] |= bit;
        colourOf[index] = colour;
        colourCount = std::max(colourCount, colour + 1);
    }

    // Largest colour first, so the barrier-worthy phases run before the tail
    // of one- and two-constraint colours that a high-degree hub leaves behind.
    std::vector<std::size_t> sizes(colourCount, 0);
    for (const std::size_t colour : colourOf) ++sizes[colour];
    std::vector<std::size_t> byDescendingSize(colourCount);
    for (std::size_t colour = 0; colour < colourCount; ++colour) {
        byDescendingSize[colour] = colour;
    }
    std::sort(byDescendingSize.begin(), byDescendingSize.end(),
              [&](std::size_t left, std::size_t right) {
                  if (sizes[left] != sizes[right]) {
                      return sizes[left] > sizes[right];
                  }
                  return left < right;
              });
    std::vector<std::size_t> rankOfColour(colourCount, 0);
    for (std::size_t rank = 0; rank < colourCount; ++rank) {
        rankOfColour[byDescendingSize[rank]] = rank;
    }

    std::vector<std::size_t> offsets(colourCount + 1, 0);
    for (std::size_t rank = 0; rank < colourCount; ++rank) {
        offsets[rank + 1] = offsets[rank] + sizes[byDescendingSize[rank]];
    }
    std::vector<std::size_t> ordered(constraints_.size(), 0);
    std::vector<std::size_t> cursor(offsets.begin(), offsets.end() - 1);
    for (std::size_t index = 0; index < colourOf.size(); ++index) {
        ordered[cursor[rankOfColour[colourOf[index]]]++] = index;
    }

    std::size_t parallelColours = colourCount;
    while (parallelColours > 0 &&
           offsets[parallelColours] - offsets[parallelColours - 1] <
               kConstraintsPerBarrier) {
        --parallelColours;
    }

    colouring.largestColour = offsets.size() > 1 ? offsets[1] - offsets[0] : 0;
    colouring.serialConstraints =
        constraints_.size() - offsets[parallelColours];
    colouring.constraints = std::move(ordered);
    colouring.colourOffsets = std::move(offsets);
    colouring.parallelColours = parallelColours;
    colouring.builtForConstraintCount = constraints_.size();
    colouring.builtForNodeCount = nodes_.size();
    return colouring;
}

ConstraintColouringReport SoftBody::constraintColouringReport() const {
    ConstraintColouringReport report;
    if (constraints_.empty()) {
        return report;
    }
    const ConstraintColouring& colouring = constraintColouring();
    report.colourCount = colouring.colourOffsets.size() - 1;
    report.parallelColours = colouring.parallelColours;
    report.largestColour = colouring.largestColour;
    report.serialConstraints = colouring.serialConstraints;
    report.parallelConstraints =
        constraints_.size() - colouring.serialConstraints;
    return report;
}

SoftBody::ConstraintColouringView SoftBody::constraintColouringView() const {
    if (constraints_.empty()) {
        return {};
    }
    const ConstraintColouring& colouring = constraintColouring();
    return {std::span<const std::size_t>{colouring.constraints},
            std::span<const std::size_t>{colouring.colourOffsets}};
}

void SoftBody::solveConstraintsColoured(double inverseTimeStepSquared,
                                        WorkerPool& pool,
                                        bool packed) {
    const ConstraintColouring& colouring = constraintColouring();
    const std::size_t parallelColours = colouring.parallelColours;
    const auto colourSize = [&](std::size_t colour) {
        return colouring.colourOffsets[colour + 1] -
               colouring.colourOffsets[colour];
    };
    pool.forEachPhase(
        parallelColours,
        colourSize,
        [&](std::size_t colour) {
            // Enough chunks that a straggling worker can be helped out, few
            // enough that claiming them is not the work.
            const std::size_t claims =
                static_cast<std::size_t>(pool.workerCount()) *
                kConstraintClaimsPerWorker;
            return std::max(kMinimumConstraintChunk,
                            colourSize(colour) / claims);
        },
        [&](std::size_t colour, std::size_t first, std::size_t last) {
            const std::size_t begin = colouring.colourOffsets[colour];
            if (packed) {
                for (std::size_t slot = first; slot < last; ++slot) {
                    solveConstraintPacked(
                        constraints_[colouring.constraints[begin + slot]],
                        inverseTimeStepSquared);
                }
                return;
            }
            for (std::size_t slot = first; slot < last; ++slot) {
                solveConstraint(
                    constraints_[colouring.constraints[begin + slot]],
                    inverseTimeStepSquared);
            }
        });

    // The tail: colours too thin to pay for a barrier. Running them here in
    // colour order keeps the sweep identical to the parallel path's.
    const std::size_t colourCount = colouring.colourOffsets.size() - 1;
    for (std::size_t colour = parallelColours; colour < colourCount; ++colour) {
        for (std::size_t slot = colouring.colourOffsets[colour];
             slot < colouring.colourOffsets[colour + 1]; ++slot) {
            DistanceConstraint& constraint =
                constraints_[colouring.constraints[slot]];
            if (packed) {
                solveConstraintPacked(constraint, inverseTimeStepSquared);
            } else {
                solveConstraint(constraint, inverseTimeStepSquared);
            }
        }
    }
}

unsigned SoftBody::MembraneColouring::workerCap(unsigned requested) const {
    const std::size_t affordable =
        std::max<std::size_t>(1, largestColour / kMinimumElementsPerWorker);
    return static_cast<unsigned>(
        std::min<std::size_t>(requested, affordable));
}

const SoftBody::MembraneColouring& SoftBody::membraneColouring() const {
    MembraneColouring& colouring = membraneColouring_;
    if (colouring.builtForElementCount == membraneElements_.size() &&
        colouring.builtForNodeCount == nodes_.size() &&
        !membraneElements_.empty()) {
        return colouring;
    }

    std::vector<std::size_t> colourOf(membraneElements_.size(), 0);
    std::vector<std::uint64_t> nodeColours(nodes_.size(), 0);
    std::size_t colourCount = 0;
    for (std::size_t index = 0; index < membraneElements_.size(); ++index) {
        const Triangle& triangle = triangles_[membraneElements_[index].triangle];
        const std::uint64_t used = nodeColours[triangle.a] |
                                   nodeColours[triangle.b] |
                                   nodeColours[triangle.c];
        const int colour = std::countr_one(used);
        if (colour >= 64) {
            throw std::invalid_argument(
                "Membrane colouring exceeded 64 colours");
        }
        const std::uint64_t bit = std::uint64_t{1} << colour;
        nodeColours[triangle.a] |= bit;
        nodeColours[triangle.b] |= bit;
        nodeColours[triangle.c] |= bit;
        colourOf[index] = static_cast<std::size_t>(colour);
        colourCount =
            std::max(colourCount, static_cast<std::size_t>(colour) + 1);
    }

    std::vector<std::size_t> sizes(colourCount, 0);
    for (const std::size_t colour : colourOf) ++sizes[colour];
    std::vector<std::size_t> byDescendingSize(colourCount);
    for (std::size_t colour = 0; colour < colourCount; ++colour) {
        byDescendingSize[colour] = colour;
    }
    std::sort(byDescendingSize.begin(), byDescendingSize.end(),
              [&](std::size_t left, std::size_t right) {
                  if (sizes[left] != sizes[right]) {
                      return sizes[left] > sizes[right];
                  }
                  return left < right;
              });
    std::vector<std::size_t> rankOfColour(colourCount, 0);
    for (std::size_t rank = 0; rank < colourCount; ++rank) {
        rankOfColour[byDescendingSize[rank]] = rank;
    }

    std::vector<std::size_t> offsets(colourCount + 1, 0);
    for (std::size_t rank = 0; rank < colourCount; ++rank) {
        offsets[rank + 1] = offsets[rank] + sizes[byDescendingSize[rank]];
    }
    std::vector<std::size_t> ordered(membraneElements_.size(), 0);
    std::vector<std::size_t> cursor(offsets.begin(), offsets.end() - 1);
    for (std::size_t index = 0; index < colourOf.size(); ++index) {
        ordered[cursor[rankOfColour[colourOf[index]]]++] = index;
    }

    std::size_t parallelColours = colourCount;
    while (parallelColours > 0 &&
           offsets[parallelColours] - offsets[parallelColours - 1] <
               kElementsPerBarrier) {
        --parallelColours;
    }

    std::vector<std::size_t> lastColourAtNode(
        nodes_.size(), std::numeric_limits<std::size_t>::max());
    for (std::size_t colour = 0; colour < colourCount; ++colour) {
        for (std::size_t slot = offsets[colour]; slot < offsets[colour + 1];
             ++slot) {
            const Triangle& triangle =
                triangles_[membraneElements_[ordered[slot]].triangle];
            for (const std::size_t node : {triangle.a, triangle.b, triangle.c}) {
                if (lastColourAtNode[node] == colour) {
                    throw std::logic_error(
                        "Membrane colouring is not node-disjoint");
                }
                lastColourAtNode[node] = colour;
            }
        }
    }

    colouring.largestColour = offsets.empty() ? 0 : offsets[1] - offsets[0];
    colouring.elements = std::move(ordered);
    colouring.colourOffsets = std::move(offsets);
    colouring.parallelColours = parallelColours;
    colouring.builtForElementCount = membraneElements_.size();
    colouring.builtForNodeCount = nodes_.size();
    return colouring;
}

void SoftBody::solveMembraneColoured(double dt, WorkerPool& pool) {
    const MembraneColouring& colouring = membraneColouring();
    const std::size_t parallelColours = colouring.parallelColours;
    pool.forEachPhase(
        parallelColours,
        [&](std::size_t colour) {
            return colouring.colourOffsets[colour + 1] -
                   colouring.colourOffsets[colour];
        },
        [](std::size_t) { return kColouredChunkGrain; },
        [&](std::size_t colour, std::size_t first, std::size_t last) {
            const std::size_t begin = colouring.colourOffsets[colour];
            for (std::size_t slot = first; slot < last; ++slot) {
                solveMembraneElement(
                    membraneElements_[colouring.elements[begin + slot]], dt);
            }
        });

    const std::size_t colourCount = colouring.colourOffsets.size() - 1;
    for (std::size_t colour = parallelColours; colour < colourCount; ++colour) {
        for (std::size_t slot = colouring.colourOffsets[colour];
             slot < colouring.colourOffsets[colour + 1]; ++slot) {
            solveMembraneElement(
                membraneElements_[colouring.elements[slot]], dt);
        }
    }
}

SoftBody::MembraneJacobiScratch& SoftBody::membraneJacobiScratch() {
    MembraneJacobiScratch& scratch = membraneJacobiScratch_;
    if (scratch.builtForElementCount == membraneElements_.size() &&
        scratch.builtForNodeCount == nodes_.size() &&
        !membraneElements_.empty()) {
        return scratch;
    }

    scratch = {};
    scratch.elementCorrections.resize(membraneElements_.size());
    scratch.nodeOffsets.assign(nodes_.size() + 1, 0);
    for (std::size_t index = 0; index < membraneElements_.size(); ++index) {
        const Triangle& triangle = triangles_[membraneElements_[index].triangle];
        ++scratch.nodeOffsets[triangle.a + 1];
        ++scratch.nodeOffsets[triangle.b + 1];
        ++scratch.nodeOffsets[triangle.c + 1];
    }
    for (std::size_t node = 0; node < nodes_.size(); ++node) {
        scratch.nodeOffsets[node + 1] += scratch.nodeOffsets[node];
    }
    scratch.incidences.resize(3 * membraneElements_.size());
    std::vector<std::size_t> cursor(scratch.nodeOffsets.begin(),
                                    scratch.nodeOffsets.end() - 1);
    for (std::size_t index = 0; index < membraneElements_.size(); ++index) {
        const Triangle& triangle = triangles_[membraneElements_[index].triangle];
        const std::array<std::size_t, 3> nodeIndices{
            triangle.a, triangle.b, triangle.c};
        for (std::size_t corner = 0; corner < nodeIndices.size(); ++corner) {
            scratch.incidences[cursor[nodeIndices[corner]]++] = {
                index, static_cast<std::uint8_t>(corner)};
        }
    }
    scratch.builtForElementCount = membraneElements_.size();
    scratch.builtForNodeCount = nodes_.size();
    return scratch;
}

void SoftBody::solveMembraneJacobi(double dt, WorkerPool& pool) {
    MembraneJacobiScratch& scratch = membraneJacobiScratch();
    const auto phaseGrain = [&](std::size_t count) {
        constexpr std::size_t kClaimsPerWorker = 4;
        const std::size_t claims =
            static_cast<std::size_t>(pool.workerCount()) * kClaimsPerWorker;
        return std::max<std::size_t>(1, count / claims);
    };
    const std::array<std::size_t, 2> grains{
        phaseGrain(membraneElements_.size()), phaseGrain(nodes_.size())};
    // Phase 0 reads one common position state and writes per-element slots.
    // Phase 1 gives each node to exactly one worker and reduces its incident
    // slots in ascending element order. Two barriers replace the colouring's
    // roughly fourteen tiny phases without introducing atomics into physics.
    pool.forEachPhase(
        2,
        [&](std::size_t phase) {
            return phase == 0 ? membraneElements_.size() : nodes_.size();
        },
        [&](std::size_t phase) { return grains[phase]; },
        [&](std::size_t phase, std::size_t first, std::size_t last) {
            if (phase == 0) {
                for (std::size_t element = first; element < last; ++element) {
                    scratch.elementCorrections[element] =
                        membraneElementCorrections(
                            membraneElements_[element], dt);
                }
                return;
            }
            for (std::size_t node = first; node < last; ++node) {
                Vec3 correction;
                const std::size_t begin = scratch.nodeOffsets[node];
                const std::size_t end = scratch.nodeOffsets[node + 1];
                for (std::size_t slot = begin; slot < end; ++slot) {
                    const MembraneJacobiScratch::Incidence incidence =
                        scratch.incidences[slot];
                    correction += scratch.elementCorrections
                        [incidence.element][incidence.corner];
                }
                if (end != begin) {
                    correction *= 1.0 / static_cast<double>(end - begin);
                }
                nodes_[node].position += correction;
            }
        });
}

void SoftBody::updateMembraneSolverDiagnostics(MembraneElement& element,
                                               double dt) {
    const MembraneEvaluation evaluation = evaluateMembraneElement(
        std::span<const Node>{nodes_},
        std::span<const Triangle>{triangles_},
        element,
        {},
        MembraneEvaluationDetail::StrainOnly);
    const Triangle& triangle = triangles_[element.triangle];
    const std::array<std::size_t, 3> nodeIndices{
        triangle.a, triangle.b, triangle.c};
    const double areaScale = std::sqrt(element.referenceArea);
    const double inverseTimeStepSquared = 1.0 / (dt * dt);
    const SymmetricMatrix3 alphaTilde =
        inverseTimeStepSquared * element.complianceMatrix;
    const double gamma = element.material.dampingTime / dt;

    Vec3 dampingMotion;
    for (std::size_t corner = 0; corner < 3; ++corner) {
        const Node& node = nodes_[nodeIndices[corner]];
        const Vec3 displacement = node.position - node.previousPosition;
        dampingMotion.x += areaScale *
                           dot(evaluation.strainGradients[0][corner],
                               displacement);
        dampingMotion.y += areaScale *
                           dot(evaluation.strainGradients[1][corner],
                               displacement);
        dampingMotion.z += areaScale *
                           dot(evaluation.strainGradients[2][corner],
                               displacement);
    }

    const Vec3 residual =
        areaScale * evaluation.diagnostics.greenStrain +
        alphaTilde * element.multiplier + gamma * dampingMotion;
    element.normalizedResidual = length(residual) / areaScale;
    element.solverResultantEstimate =
        -element.multiplier * (inverseTimeStepSquared / areaScale);
}

RectangularMembraneCoupon SoftBody::addRectangularMembraneCoupon(
    double couponLength,
    double couponWidth,
    std::size_t lengthSegments,
    std::size_t widthSegments,
    double arealDensity,
    const OrthotropicMembraneMaterial& material,
    double materialAngleRadians,
    bool reverseDiagonal) {
    validateOrthotropicMembraneMaterial(material);
    if (!std::isfinite(couponLength) || !std::isfinite(couponWidth) ||
        !std::isfinite(arealDensity) ||
        !std::isfinite(materialAngleRadians) || !(couponLength > 0.0) ||
        !(couponWidth > 0.0) || !(arealDensity > 0.0) ||
        lengthSegments == 0 || widthSegments == 0) {
        throw std::invalid_argument(
            "Invalid rectangular membrane coupon parameters");
    }

    const std::size_t maximumSize = std::numeric_limits<std::size_t>::max();
    if (lengthSegments == maximumSize || widthSegments == maximumSize) {
        throw std::invalid_argument(
            "Rectangular membrane coupon segment count is too large");
    }
    const std::size_t lengthNodes = lengthSegments + 1;
    const std::size_t widthNodes = widthSegments + 1;
    if (lengthNodes > maximumSize / widthNodes ||
        lengthSegments > maximumSize / widthSegments ||
        2 > maximumSize / (lengthSegments * widthSegments)) {
        throw std::invalid_argument(
            "Rectangular membrane coupon size overflows");
    }
    const std::size_t nodeCount = lengthNodes * widthNodes;
    const std::size_t triangleCount =
        2 * lengthSegments * widthSegments;
    if (nodes_.size() > maximumSize - nodeCount ||
        triangles_.size() > maximumSize - triangleCount ||
        membraneElements_.size() > maximumSize - triangleCount) {
        throw std::invalid_argument(
            "Rectangular membrane coupon cannot fit in this body");
    }

    const double dx =
        couponLength / static_cast<double>(lengthSegments);
    const double dy = couponWidth / static_cast<double>(widthSegments);
    const double totalArea = couponLength * couponWidth;
    const double totalMass = totalArea * arealDensity;
    if (!std::isfinite(dx) || !std::isfinite(dy) || !(dx > 0.0) ||
        !(dy > 0.0) || !std::isfinite(totalArea) || !(totalArea > 0.0) ||
        !std::isfinite(totalMass) || !(totalMass > 0.0)) {
        throw std::invalid_argument(
            "Rectangular membrane coupon derived values are invalid");
    }

    const double cosine = std::cos(materialAngleRadians);
    const double sine = std::sin(materialAngleRadians);
    std::vector<Node> pendingNodes;
    std::vector<Vec2> pendingChart;
    std::vector<Triangle> pendingTriangles;
    pendingNodes.reserve(nodeCount);
    pendingChart.reserve(nodeCount);
    pendingTriangles.reserve(triangleCount);
    for (std::size_t widthIndex = 0; widthIndex < widthNodes; ++widthIndex) {
        const double y = -0.5 * couponWidth +
                         static_cast<double>(widthIndex) * dy;
        for (std::size_t lengthIndex = 0; lengthIndex < lengthNodes;
             ++lengthIndex) {
            const double x = static_cast<double>(lengthIndex) * dx;
            const Vec3 position{x, y, 0.0};
            pendingNodes.push_back({position, position, {}, {}, 0.0});
            pendingChart.push_back(
                {cosine * x + sine * y, -sine * x + cosine * y});
        }
    }
    const auto localNode = [lengthNodes](std::size_t lengthIndex,
                                         std::size_t widthIndex) {
        return widthIndex * lengthNodes + lengthIndex;
    };
    for (std::size_t widthIndex = 0; widthIndex < widthSegments;
         ++widthIndex) {
        for (std::size_t lengthIndex = 0; lengthIndex < lengthSegments;
             ++lengthIndex) {
            const std::size_t n00 = localNode(lengthIndex, widthIndex);
            const std::size_t n01 = localNode(lengthIndex + 1, widthIndex);
            const std::size_t n10 = localNode(lengthIndex, widthIndex + 1);
            const std::size_t n11 =
                localNode(lengthIndex + 1, widthIndex + 1);
            if (reverseDiagonal) {
                pendingTriangles.push_back({n00, n01, n10, 0.0});
                pendingTriangles.push_back({n01, n11, n10, 0.0});
            } else {
                pendingTriangles.push_back({n00, n01, n11, 0.0});
                pendingTriangles.push_back({n00, n11, n10, 0.0});
            }
        }
    }

    std::vector<MembraneElementDefinition> pendingDefinitions;
    pendingDefinitions.reserve(triangleCount);
    for (std::size_t triangleIndex = 0;
         triangleIndex < pendingTriangles.size();
         ++triangleIndex) {
        const Triangle& triangle = pendingTriangles[triangleIndex];
        pendingDefinitions.push_back(
            {triangleIndex,
             {pendingChart[triangle.a],
              pendingChart[triangle.b],
              pendingChart[triangle.c]},
             material,
             MaterialRole::Bulk});
        const Vec2 firstEdge =
            pendingChart[triangle.b] - pendingChart[triangle.a];
        const Vec2 secondEdge =
            pendingChart[triangle.c] - pendingChart[triangle.a];
        const double area = 0.5 * cross(firstEdge, secondEdge);
        const double cornerMass = area * arealDensity / 3.0;
        if (!std::isfinite(cornerMass) || !(cornerMass > 0.0)) {
            throw std::invalid_argument(
                "Rectangular membrane coupon nodal mass is invalid");
        }
        pendingNodes[triangle.a].inverseMass += cornerMass;
        pendingNodes[triangle.b].inverseMass += cornerMass;
        pendingNodes[triangle.c].inverseMass += cornerMass;
    }
    for (Node& node : pendingNodes) {
        const double mass = node.inverseMass;
        if (!std::isfinite(mass) || !(mass > 0.0)) {
            throw std::invalid_argument(
                "Rectangular membrane coupon has an invalid node mass");
        }
        node.inverseMass = 1.0 / mass;
    }
    validateMembraneElementDefinitions(
        std::span<const Node>{pendingNodes},
        std::span<const Triangle>{pendingTriangles},
        std::span<const MembraneElementDefinition>{pendingDefinitions});

    std::vector<MembraneElement> pendingElements;
    pendingElements.reserve(triangleCount);
    for (const MembraneElementDefinition& definition : pendingDefinitions) {
        const Vec2 firstEdge = definition.chart[1] - definition.chart[0];
        const Vec2 secondEdge = definition.chart[2] - definition.chart[0];
        const Matrix2 referenceMatrix{
            firstEdge.x, secondEdge.x, firstEdge.y, secondEdge.y};
        pendingElements.push_back({definition.triangle,
                                   definition.chart,
                                   definition.material,
                                   definition.role,
                                   0.5 * referenceMatrix.determinant(),
                                   checkedInverse(referenceMatrix),
                                   {},
                                   {},
                                   0.0,
                                   definition.material.complianceMatrix()});
    }

    nodes_.reserve(nodes_.size() + nodeCount);
    triangles_.reserve(triangles_.size() + triangleCount);
    membraneElements_.reserve(membraneElements_.size() + triangleCount);
    const std::size_t firstNode = nodes_.size();
    const std::size_t firstTriangle = triangles_.size();
    const std::size_t firstElement = membraneElements_.size();
    nodes_.insert(nodes_.end(), pendingNodes.begin(), pendingNodes.end());
    for (Triangle triangle : pendingTriangles) {
        triangle.a += firstNode;
        triangle.b += firstNode;
        triangle.c += firstNode;
        triangles_.push_back(triangle);
    }
    for (MembraneElement element : pendingElements) {
        element.triangle += firstTriangle;
        membraneElements_.push_back(element);
    }

    const SurfaceGroup surface(this, firstTriangle, triangleCount);
    const MembraneGroup membrane(this, firstElement, triangleCount);
    return {firstNode,
            lengthNodes,
            widthNodes,
            surface,
            membrane};
}

RectangularPatch SoftBody::addRectangularPatch(double chord,
                                               double span,
                                               std::size_t chordSegments,
                                               std::size_t spanSegments,
                                               double arealDensity,
                                               double stretchCompliance,
                                               double shearCompliance,
                                               double bendCompliance) {
    if (!(chord > 0.0) || !(span > 0.0) || chordSegments == 0 ||
        spanSegments == 0 || !(arealDensity > 0.0) ||
        stretchCompliance < 0.0 || shearCompliance < 0.0 ||
        bendCompliance < 0.0) {
        throw std::invalid_argument("Invalid rectangular patch parameters");
    }

    RectangularPatch patch;
    patch.firstNode = nodes_.size();
    patch.chordNodes = chordSegments + 1;
    patch.spanNodes = spanSegments + 1;

    const double dx = chord / static_cast<double>(chordSegments);
    const double dy = span / static_cast<double>(spanSegments);
    const double totalMass = chord * span * arealDensity;
    const double nodeMass = totalMass /
                            static_cast<double>(patch.chordNodes * patch.spanNodes);

    for (std::size_t spanIndex = 0; spanIndex < patch.spanNodes; ++spanIndex) {
        const double y = -0.5 * span + static_cast<double>(spanIndex) * dy;
        for (std::size_t chordIndex = 0; chordIndex < patch.chordNodes; ++chordIndex) {
            const double x = static_cast<double>(chordIndex) * dx;
            addNode({x, y, 0.0}, nodeMass);
        }
    }

    for (std::size_t spanIndex = 0; spanIndex < spanSegments; ++spanIndex) {
        for (std::size_t chordIndex = 0; chordIndex < chordSegments; ++chordIndex) {
            const std::size_t n00 = patch.node(chordIndex, spanIndex);
            const std::size_t n01 = patch.node(chordIndex + 1, spanIndex);
            const std::size_t n10 = patch.node(chordIndex, spanIndex + 1);
            const std::size_t n11 = patch.node(chordIndex + 1, spanIndex + 1);
            addTriangle(n00, n01, n11);
            addTriangle(n00, n11, n10);
        }
    }

    // Warp and weft constraints.
    for (std::size_t spanIndex = 0; spanIndex < patch.spanNodes; ++spanIndex) {
        for (std::size_t chordIndex = 0; chordIndex < chordSegments; ++chordIndex) {
            addDistanceConstraint(patch.node(chordIndex, spanIndex),
                                  patch.node(chordIndex + 1, spanIndex),
                                  dx,
                                  stretchCompliance);
        }
    }
    for (std::size_t chordIndex = 0; chordIndex < patch.chordNodes; ++chordIndex) {
        for (std::size_t spanIndex = 0; spanIndex < spanSegments; ++spanIndex) {
            addDistanceConstraint(patch.node(chordIndex, spanIndex),
                                  patch.node(chordIndex, spanIndex + 1),
                                  dy,
                                  stretchCompliance);
        }
    }

    // Diagonal constraints resist in-plane shear.
    const double diagonal = std::sqrt(dx * dx + dy * dy);
    for (std::size_t spanIndex = 0; spanIndex < spanSegments; ++spanIndex) {
        for (std::size_t chordIndex = 0; chordIndex < chordSegments; ++chordIndex) {
            addDistanceConstraint(patch.node(chordIndex, spanIndex),
                                  patch.node(chordIndex + 1, spanIndex + 1),
                                  diagonal,
                                  shearCompliance);
            addDistanceConstraint(patch.node(chordIndex + 1, spanIndex),
                                  patch.node(chordIndex, spanIndex + 1),
                                  diagonal,
                                  shearCompliance);
        }
    }

    // Two-edge constraints provide a tunable coarse bending resistance.
    for (std::size_t spanIndex = 0; spanIndex < patch.spanNodes; ++spanIndex) {
        for (std::size_t chordIndex = 0; chordIndex + 2 < patch.chordNodes;
             ++chordIndex) {
            addDistanceConstraint(patch.node(chordIndex, spanIndex),
                                  patch.node(chordIndex + 2, spanIndex),
                                  2.0 * dx,
                                  bendCompliance);
        }
    }
    for (std::size_t chordIndex = 0; chordIndex < patch.chordNodes; ++chordIndex) {
        for (std::size_t spanIndex = 0; spanIndex + 2 < patch.spanNodes;
             ++spanIndex) {
            addDistanceConstraint(patch.node(chordIndex, spanIndex),
                                  patch.node(chordIndex, spanIndex + 2),
                                  2.0 * dy,
                                  bendCompliance);
        }
    }

    return patch;
}

RectangularCell SoftBody::addRectangularCell(double chord,
                                             double span,
                                             double thickness,
                                             std::size_t chordSegments,
                                             std::size_t spanSegments,
                                             double arealDensity,
                                             double stretchCompliance,
                                             double shearCompliance,
                                             double bendCompliance) {
    if (!std::isfinite(chord) || !std::isfinite(span) ||
        !std::isfinite(thickness) || !std::isfinite(arealDensity) ||
        !std::isfinite(stretchCompliance) || !std::isfinite(shearCompliance) ||
        !std::isfinite(bendCompliance) || !(chord > 0.0) || !(span > 0.0) ||
        !(thickness > 0.0) ||
        chordSegments == 0 || spanSegments == 0 || !(arealDensity > 0.0) ||
        stretchCompliance < 0.0 || shearCompliance < 0.0 ||
        bendCompliance < 0.0) {
        throw std::invalid_argument("Invalid rectangular cell parameters");
    }

    const std::size_t firstNode = nodes_.size();
    const std::size_t chordNodes = chordSegments + 1;
    const std::size_t spanNodes = spanSegments + 1;
    const std::size_t nodesPerPanel = chordNodes * spanNodes;
    const double dx = chord / static_cast<double>(chordSegments);
    const double dy = span / static_cast<double>(spanSegments);
    const double totalArea =
        2.0 * (chord * span + chord * thickness + span * thickness);
    const double nodeMass =
        totalArea * arealDensity / static_cast<double>(2 * nodesPerPanel);

    for (int layer = 0; layer < 2; ++layer) {
        const double z = layer == 0 ? -0.5 * thickness : 0.5 * thickness;
        for (std::size_t spanIndex = 0; spanIndex < spanNodes; ++spanIndex) {
            const double y =
                -0.5 * span + static_cast<double>(spanIndex) * dy;
            for (std::size_t chordIndex = 0; chordIndex < chordNodes;
                 ++chordIndex) {
                addNode({static_cast<double>(chordIndex) * dx, y, z}, nodeMass);
            }
        }
    }

    const auto lower = [firstNode, chordNodes](std::size_t chordIndex,
                                                std::size_t spanIndex) {
        return firstNode + spanIndex * chordNodes + chordIndex;
    };
    const auto upper = [lower, nodesPerPanel](std::size_t chordIndex,
                                              std::size_t spanIndex) {
        return lower(chordIndex, spanIndex) + nodesPerPanel;
    };
    const std::size_t firstTriangle = triangles_.size();

    for (std::size_t spanIndex = 0; spanIndex < spanSegments; ++spanIndex) {
        for (std::size_t chordIndex = 0; chordIndex < chordSegments;
             ++chordIndex) {
            const std::size_t l00 = lower(chordIndex, spanIndex);
            const std::size_t l10 = lower(chordIndex + 1, spanIndex);
            const std::size_t l01 = lower(chordIndex, spanIndex + 1);
            const std::size_t l11 = lower(chordIndex + 1, spanIndex + 1);
            const std::size_t u00 = upper(chordIndex, spanIndex);
            const std::size_t u10 = upper(chordIndex + 1, spanIndex);
            const std::size_t u01 = upper(chordIndex, spanIndex + 1);
            const std::size_t u11 = upper(chordIndex + 1, spanIndex + 1);
            addTriangle(l00, l11, l10);
            addTriangle(l00, l01, l11);
            addTriangle(u00, u10, u11);
            addTriangle(u00, u11, u01);
        }
    }

    for (std::size_t chordIndex = 0; chordIndex < chordSegments;
         ++chordIndex) {
        const std::size_t l0 = lower(chordIndex, 0);
        const std::size_t l1 = lower(chordIndex + 1, 0);
        const std::size_t u0 = upper(chordIndex, 0);
        const std::size_t u1 = upper(chordIndex + 1, 0);
        addTriangle(l0, l1, u1);
        addTriangle(l0, u1, u0);

        const std::size_t farL0 = lower(chordIndex, spanSegments);
        const std::size_t farL1 = lower(chordIndex + 1, spanSegments);
        const std::size_t farU0 = upper(chordIndex, spanSegments);
        const std::size_t farU1 = upper(chordIndex + 1, spanSegments);
        addTriangle(farL0, farU0, farU1);
        addTriangle(farL0, farU1, farL1);
    }

    for (std::size_t spanIndex = 0; spanIndex < spanSegments; ++spanIndex) {
        const std::size_t l0 = lower(0, spanIndex);
        const std::size_t l1 = lower(0, spanIndex + 1);
        const std::size_t u0 = upper(0, spanIndex);
        const std::size_t u1 = upper(0, spanIndex + 1);
        addTriangle(l0, u0, u1);
        addTriangle(l0, u1, l1);

        const std::size_t farL0 = lower(chordSegments, spanIndex);
        const std::size_t farL1 = lower(chordSegments, spanIndex + 1);
        const std::size_t farU0 = upper(chordSegments, spanIndex);
        const std::size_t farU1 = upper(chordSegments, spanIndex + 1);
        addTriangle(farL0, farL1, farU1);
        addTriangle(farL0, farU1, farU0);
    }

    std::set<std::pair<std::size_t, std::size_t>> constrainedPairs;
    const auto addUniqueConstraint =
        [this, &constrainedPairs](std::size_t a,
                                  std::size_t b,
                                  double compliance) {
            const std::pair<std::size_t, std::size_t> key{
                std::min(a, b), std::max(a, b)};
            if (!constrainedPairs.insert(key).second) {
                return;
            }
            addDistanceConstraint(
                a, b, length(nodes_[b].position - nodes_[a].position), compliance);
        };

    for (int layer = 0; layer < 2; ++layer) {
        const auto node = [&lower, &upper, layer](std::size_t chordIndex,
                                                  std::size_t spanIndex) {
            return layer == 0 ? lower(chordIndex, spanIndex)
                              : upper(chordIndex, spanIndex);
        };
        for (std::size_t spanIndex = 0; spanIndex < spanNodes; ++spanIndex) {
            for (std::size_t chordIndex = 0; chordIndex < chordSegments;
                 ++chordIndex) {
                addUniqueConstraint(node(chordIndex, spanIndex),
                                    node(chordIndex + 1, spanIndex),
                                    stretchCompliance);
            }
        }
        for (std::size_t chordIndex = 0; chordIndex < chordNodes;
             ++chordIndex) {
            for (std::size_t spanIndex = 0; spanIndex < spanSegments;
                 ++spanIndex) {
                addUniqueConstraint(node(chordIndex, spanIndex),
                                    node(chordIndex, spanIndex + 1),
                                    stretchCompliance);
            }
        }
    }
    for (std::size_t spanIndex = 0; spanIndex < spanNodes; ++spanIndex) {
        for (std::size_t chordIndex = 0; chordIndex < chordNodes;
             ++chordIndex) {
            const bool perimeter = chordIndex == 0 ||
                                   chordIndex + 1 == chordNodes ||
                                   spanIndex == 0 || spanIndex + 1 == spanNodes;
            if (perimeter) {
                addUniqueConstraint(lower(chordIndex, spanIndex),
                                    upper(chordIndex, spanIndex),
                                    stretchCompliance);
            }
        }
    }

    for (int layer = 0; layer < 2; ++layer) {
        const auto node = [&lower, &upper, layer](std::size_t chordIndex,
                                                  std::size_t spanIndex) {
            return layer == 0 ? lower(chordIndex, spanIndex)
                              : upper(chordIndex, spanIndex);
        };
        for (std::size_t spanIndex = 0; spanIndex < spanSegments; ++spanIndex) {
            for (std::size_t chordIndex = 0; chordIndex < chordSegments;
                 ++chordIndex) {
                addUniqueConstraint(node(chordIndex, spanIndex),
                                    node(chordIndex + 1, spanIndex + 1),
                                    shearCompliance);
                addUniqueConstraint(node(chordIndex + 1, spanIndex),
                                    node(chordIndex, spanIndex + 1),
                                    shearCompliance);
            }
        }
    }
    for (std::size_t chordIndex = 0; chordIndex < chordSegments;
         ++chordIndex) {
        for (const std::size_t spanIndex : {std::size_t{0}, spanSegments}) {
            addUniqueConstraint(lower(chordIndex, spanIndex),
                                upper(chordIndex + 1, spanIndex),
                                shearCompliance);
            addUniqueConstraint(upper(chordIndex, spanIndex),
                                lower(chordIndex + 1, spanIndex),
                                shearCompliance);
        }
    }
    for (std::size_t spanIndex = 0; spanIndex < spanSegments; ++spanIndex) {
        for (const std::size_t chordIndex : {std::size_t{0}, chordSegments}) {
            addUniqueConstraint(lower(chordIndex, spanIndex),
                                upper(chordIndex, spanIndex + 1),
                                shearCompliance);
            addUniqueConstraint(upper(chordIndex, spanIndex),
                                lower(chordIndex, spanIndex + 1),
                                shearCompliance);
        }
    }

    for (int layer = 0; layer < 2; ++layer) {
        const auto node = [&lower, &upper, layer](std::size_t chordIndex,
                                                  std::size_t spanIndex) {
            return layer == 0 ? lower(chordIndex, spanIndex)
                              : upper(chordIndex, spanIndex);
        };
        for (std::size_t spanIndex = 0; spanIndex < spanNodes; ++spanIndex) {
            for (std::size_t chordIndex = 0; chordIndex + 2 < chordNodes;
                 ++chordIndex) {
                addUniqueConstraint(node(chordIndex, spanIndex),
                                    node(chordIndex + 2, spanIndex),
                                    bendCompliance);
            }
        }
        for (std::size_t chordIndex = 0; chordIndex < chordNodes;
             ++chordIndex) {
            for (std::size_t spanIndex = 0; spanIndex + 2 < spanNodes;
                 ++spanIndex) {
                addUniqueConstraint(node(chordIndex, spanIndex),
                                    node(chordIndex, spanIndex + 2),
                                    bendCompliance);
            }
        }
    }

    const SurfaceGroup surface =
        surfaceGroup(firstTriangle, triangles_.size() - firstTriangle);
    return {firstNode, chordNodes, spanNodes, surface};
}

double SoftBody::surfaceArea() const {
    double result = 0.0;
    for (const Triangle& triangle : triangles_) {
        const Vec3& a = nodes_[triangle.a].position;
        const Vec3& b = nodes_[triangle.b].position;
        const Vec3& c = nodes_[triangle.c].position;
        result += 0.5 * length(cross(b - a, c - a));
    }
    return result;
}

double SoftBody::surfaceArea(const SurfaceGroup& surface) const {
    requireSurfaceGroup(surface);
    double result = 0.0;
    const std::size_t last = surface.firstTriangle_ + surface.triangleCount_;
    for (std::size_t triangleIndex = surface.firstTriangle_; triangleIndex < last;
         ++triangleIndex) {
        const Triangle& triangle = triangles_[triangleIndex];
        const Vec3& a = nodes_[triangle.a].position;
        const Vec3& b = nodes_[triangle.b].position;
        const Vec3& c = nodes_[triangle.c].position;
        result += 0.5 * length(cross(b - a, c - a));
    }
    return result;
}

double SoftBody::signedVolume(const SurfaceGroup& surface) const {
    requireSurfaceGroup(surface);

    const std::size_t first = surface.firstTriangle_;
    const std::size_t last = first + surface.triangleCount_;
    const Triangle& firstTriangle = triangles_[first];
    Vec3 minimum = nodes_[firstTriangle.a].position;
    Vec3 maximum = minimum;

    const auto includeInBounds = [&minimum, &maximum](const Vec3& position) {
        minimum.x = std::min(minimum.x, position.x);
        minimum.y = std::min(minimum.y, position.y);
        minimum.z = std::min(minimum.z, position.z);
        maximum.x = std::max(maximum.x, position.x);
        maximum.y = std::max(maximum.y, position.y);
        maximum.z = std::max(maximum.z, position.z);
    };

    for (std::size_t triangleIndex = first; triangleIndex < last;
         ++triangleIndex) {
        const Triangle& triangle = triangles_[triangleIndex];
        includeInBounds(nodes_[triangle.a].position);
        includeInBounds(nodes_[triangle.b].position);
        includeInBounds(nodes_[triangle.c].position);
    }

    const Vec3 reference{
        minimum.x + 0.5 * (maximum.x - minimum.x),
        minimum.y + 0.5 * (maximum.y - minimum.y),
        minimum.z + 0.5 * (maximum.z - minimum.z),
    };

    double sixTimesVolume = 0.0;
    double compensation = 0.0;
    for (std::size_t triangleIndex = first; triangleIndex < last;
         ++triangleIndex) {
        const Triangle& triangle = triangles_[triangleIndex];
        const Vec3 a = nodes_[triangle.a].position - reference;
        const Vec3 b = nodes_[triangle.b].position - reference;
        const Vec3 c = nodes_[triangle.c].position - reference;
        const double term = dot(a, cross(b, c));
        const double correctedTerm = term - compensation;
        const double updatedSum = sixTimesVolume + correctedTerm;
        compensation = (updatedSum - sixTimesVolume) - correctedTerm;
        sixTimesVolume = updatedSum;
    }
    return sixTimesVolume / 6.0;
}

SurfaceTopologyReport SoftBody::validateSurfaceTopology(
    const SurfaceGroup& surface) const {
    requireSurfaceGroup(surface);
    return softwing::validateSurfaceTopology(
        std::span<const Node>{nodes_},
        std::span<const Triangle>{triangles_.data() + surface.firstTriangle_,
                                  surface.triangleCount_});
}

Vec3 SoftBody::totalPressureForce() const {
    Vec3 result;
    for (const Triangle& triangle : triangles_) {
        const Vec3& a = nodes_[triangle.a].position;
        const Vec3& b = nodes_[triangle.b].position;
        const Vec3& c = nodes_[triangle.c].position;
        result += triangle.pressureDifference * 0.5 * cross(b - a, c - a);
    }
    return result;
}

Vec3 SoftBody::totalPressureForce(const SurfaceGroup& surface) const {
    requireSurfaceGroup(surface);
    Vec3 result;
    const std::size_t last = surface.firstTriangle_ + surface.triangleCount_;
    for (std::size_t triangleIndex = surface.firstTriangle_; triangleIndex < last;
         ++triangleIndex) {
        const Triangle& triangle = triangles_[triangleIndex];
        const Vec3& a = nodes_[triangle.a].position;
        const Vec3& b = nodes_[triangle.b].position;
        const Vec3& c = nodes_[triangle.c].position;
        result += triangle.pressureDifference * 0.5 * cross(b - a, c - a);
    }
    return result;
}

Vec3 SoftBody::totalPressureMoment(const Vec3& origin) const {
    Vec3 result;
    for (const Triangle& triangle : triangles_) {
        const Vec3& a = nodes_[triangle.a].position;
        const Vec3& b = nodes_[triangle.b].position;
        const Vec3& c = nodes_[triangle.c].position;
        const Vec3 force =
            triangle.pressureDifference * 0.5 * cross(b - a, c - a);
        const Vec3 centroid = (a + b + c) / 3.0;
        result += cross(centroid - origin, force);
    }
    return result;
}

Vec3 SoftBody::totalPressureMoment(const SurfaceGroup& surface,
                                   const Vec3& origin) const {
    requireSurfaceGroup(surface);
    Vec3 result;
    const std::size_t last = surface.firstTriangle_ + surface.triangleCount_;
    for (std::size_t triangleIndex = surface.firstTriangle_; triangleIndex < last;
         ++triangleIndex) {
        const Triangle& triangle = triangles_[triangleIndex];
        const Vec3& a = nodes_[triangle.a].position;
        const Vec3& b = nodes_[triangle.b].position;
        const Vec3& c = nodes_[triangle.c].position;
        const Vec3 force =
            triangle.pressureDifference * 0.5 * cross(b - a, c - a);
        const Vec3 centroid = (a + b + c) / 3.0;
        result += cross(centroid - origin, force);
    }
    return result;
}

#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
void AerodynamicVerificationTestAccess::permuteSoftBodyStorage(
    SoftBody& body,
    std::span<const std::size_t> nodeMap,
    std::span<const std::size_t> triangleMap) {
    const auto requirePermutation = [](std::span<const std::size_t> map,
                                       std::size_t size,
                                       const char* label) {
        if (map.size() != size)
            throw std::invalid_argument(
                std::string(label) + " permutation has the wrong size");
        std::vector<bool> seen(size, false);
        for (const std::size_t index : map) {
            if (index >= size || seen[index])
                throw std::invalid_argument(
                    std::string(label) + " permutation is not bijective");
            seen[index] = true;
        }
    };
    requirePermutation(nodeMap, body.nodes_.size(), "node");
    requirePermutation(triangleMap, body.triangles_.size(), "triangle");
    if (!body.aerodynamicRegistration_.expired())
        throw std::invalid_argument(
            "cannot permute a live aerodynamic structural registration");
    const auto node = [&](std::size_t oldIndex) { return nodeMap[oldIndex]; };
    const auto triangle = [&](std::size_t oldIndex) {
        return triangleMap[oldIndex];
    };
    std::vector<Node> nodes(body.nodes_.size());
    for (std::size_t oldIndex = 0; oldIndex < body.nodes_.size(); ++oldIndex)
        nodes[node(oldIndex)] = body.nodes_[oldIndex];
    std::vector<Triangle> triangles(body.triangles_.size());
    for (std::size_t oldIndex = 0;
         oldIndex < body.triangles_.size(); ++oldIndex) {
        Triangle value = body.triangles_[oldIndex];
        value.a = node(value.a); value.b = node(value.b); value.c = node(value.c);
        triangles[triangle(oldIndex)] = value;
    }
    body.nodes_ = std::move(nodes);
    body.triangles_ = std::move(triangles);
    for (auto& constraint : body.constraints_) {
        constraint.a = node(constraint.a);
        constraint.b = node(constraint.b);
    }
    for (auto& element : body.membraneElements_)
        element.triangle = triangle(element.triangle);
    for (auto& constraint : body.dihedralConstraints_) {
        constraint.a = node(constraint.a);
        constraint.b = node(constraint.b);
        constraint.c = node(constraint.c);
        constraint.d = node(constraint.d);
    }
    for (auto& surface : body.contactSurfaces_) {
        if (surface.triangleCount > 0) {
            std::size_t first = body.triangles_.size();
            for (std::size_t offset = 0;
                 offset < surface.triangleCount; ++offset)
                first = std::min(
                    first, triangle(surface.firstTriangle + offset));
            surface.firstTriangle = first;
        }
        for (auto& value : surface.vertices) value = node(value);
        std::ranges::sort(surface.vertices);
        for (auto& edge : surface.edges) {
            edge.a = node(edge.a); edge.b = node(edge.b);
            if (edge.b < edge.a) std::swap(edge.a, edge.b);
        }
        std::ranges::sort(surface.edges);
    }
    for (auto& line : body.contactLines_) {
        line.a = node(line.a); line.b = node(line.b);
    }
    const auto remapKey = [&](ContactFeatureKey& key) {
        switch (key.kind) {
        case ContactFeatureKind::VertexTriangle:
            key.firstPrimitive[0] = node(key.firstPrimitive[0]);
            key.secondPrimitive[0] = triangle(key.secondPrimitive[0]);
            break;
        case ContactFeatureKind::EdgeEdge:
            key.firstPrimitive[0] = node(key.firstPrimitive[0]);
            key.firstPrimitive[1] = node(key.firstPrimitive[1]);
            key.secondPrimitive[0] = node(key.secondPrimitive[0]);
            key.secondPrimitive[1] = node(key.secondPrimitive[1]);
            break;
        case ContactFeatureKind::SegmentTriangle:
            key.firstPrimitive[0] = node(key.firstPrimitive[0]);
            key.firstPrimitive[1] = node(key.firstPrimitive[1]);
            key.secondPrimitive[0] = triangle(key.secondPrimitive[0]);
            break;
        }
    };
    for (auto& [key, multiplier] : body.contactMultipliers_) {
        static_cast<void>(multiplier);
        remapKey(key);
    }
    std::ranges::sort(body.contactMultipliers_, {},
        [](const auto& value) -> const ContactFeatureKey& {
            return value.first;
        });
    for (auto& record : body.contactRecords_) {
        remapKey(record.key);
        for (std::size_t i = 0; i < record.firstNodeCount; ++i)
            record.firstNodes[i] = node(record.firstNodes[i]);
        for (std::size_t i = 0; i < record.secondNodeCount; ++i)
            record.secondNodes[i] = node(record.secondNodes[i]);
    }
    std::ranges::sort(body.contactRecords_, {}, &ContactRecord::key);
    if (body.contactDiagnostics_.hasFailure)
        remapKey(body.contactDiagnostics_.failureKey);
    for (auto& value : body.contactPairDiagnostics_)
        if (value.hasFailure) remapKey(value.failureKey);
    const auto remapKeys = [&](auto& values) {
        for (auto& value : values) remapKey(value);
        std::ranges::sort(values);
    };
    remapKeys(body.contactAudit_.iterationCandidateKeys);
    remapKeys(body.contactAudit_.iterationQueryKeys);
    remapKeys(body.contactAudit_.certificationCandidateKeys);
    remapKeys(body.contactAudit_.certificationQueryKeys);
}

std::string AerodynamicVerificationTestAccess::softBodyState(
    const SoftBody& body) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    const auto vec = [&](const Vec3& value) {
        out << value.x << ' ' << value.y << ' ' << value.z << ' ';
    };
    const auto key = [&](const ContactFeatureKey& value) {
        out << value.pair << ' ' << static_cast<int>(value.kind) << ' ';
        for (const auto index : value.firstPrimitive) out << index << ' ';
        for (const auto index : value.secondPrimitive) out << index << ' ';
    };
    const auto diagnostics = [&](const ContactDiagnostics& value) {
        out << value.registered << ' ' << value.solveSucceeded << ' '
            << value.possibleCount << ' ' << value.candidateCount << ' '
            << value.excludedCount << ' ' << value.queryCount << ' '
            << value.activeCount << ' ' << value.indeterminateCount << ' '
            << value.minimumGap << ' ' << value.maximumPenetration << ' '
            << value.maximumNormalResidual << ' '
            << value.maximumFrictionResidual << ' ';
        vec(value.firstImpulse); vec(value.secondImpulse);
        vec(value.netInternalImpulse); vec(value.firstMoment);
        vec(value.secondMoment); vec(value.netInternalMoment);
        out << value.frictionWork << ' ' << value.maximumCcdIterations << ' '
            << value.maximumIntervalSubdivisions << ' ' << value.hasFailure
            << ' ';
        key(value.failureKey);
    };
    out << "SOFT " << body.nodes_.size() << ' ' << body.triangles_.size()
        << ' ' << body.constraints_.size() << ' '
        << body.membraneElements_.size() << ' '
        << body.dihedralConstraints_.size() << '\n';
    for (const Node& node : body.nodes_) {
        vec(node.position); vec(node.previousPosition); vec(node.velocity);
        vec(node.force); out << node.inverseMass << '\n';
    }
    for (const Triangle& triangle : body.triangles_)
        out << triangle.a << ' ' << triangle.b << ' ' << triangle.c << ' '
            << triangle.pressureDifference << '\n';
    for (const DistanceConstraint& constraint : body.constraints_)
        out << constraint.a << ' ' << constraint.b << ' '
            << constraint.restLength << ' ' << constraint.compliance << ' '
            << constraint.accumulatedLambda << ' '
            << static_cast<int>(constraint.kind) << '\n';
    for (const MembraneElement& element : body.membraneElements_) {
        out << element.triangle << ' ';
        for (const Vec2& chart : element.chart)
            out << chart.x << ' ' << chart.y << ' ';
        const auto& material = element.material;
        out << material.warpStiffness << ' ' << material.weftStiffness << ' '
            << material.couplingStiffness << ' ' << material.shearStiffness
            << ' ' << material.warpPreTension << ' '
            << material.weftPreTension << ' ' << material.dampingTime << ' '
            << material.compressionStiffnessRatio << ' '
            << static_cast<int>(element.role) << ' ' << element.referenceArea
            << ' ' << element.inverseReferenceMatrix.m00 << ' '
            << element.inverseReferenceMatrix.m01 << ' '
            << element.inverseReferenceMatrix.m10 << ' '
            << element.inverseReferenceMatrix.m11 << ' ';
        vec(element.multiplier); vec(element.solverResultantEstimate);
        out << element.normalizedResidual << '\n';
    }
    for (const DihedralBendingConstraint& constraint :
         body.dihedralConstraints_) {
        out << constraint.a << ' ' << constraint.b << ' ' << constraint.c
            << ' ' << constraint.d << ' ' << constraint.restAngleRadians
            << ' ' << constraint.compliance << ' '
            << constraint.accumulatedLambda << '\n';
    }
    out << "SURFACES " << body.contactSurfaces_.size() << '\n';
    for (const auto& surface : body.contactSurfaces_) {
        out << surface.firstTriangle << ' ' << surface.triangleCount << ' '
            << surface.halfThickness << ' ' << surface.vertices.size() << ' ';
        for (const auto node : surface.vertices) out << node << ' ';
        out << surface.edges.size() << ' ';
        for (const auto& edge : surface.edges)
            out << edge.a << ' ' << edge.b << ' ';
        out << '\n';
    }
    out << "LINES " << body.contactLines_.size() << '\n';
    for (const auto& line : body.contactLines_)
        out << line.a << ' ' << line.b << ' ' << line.radius << '\n';
    out << "PAIRS " << body.contactPairs_.size() << '\n';
    for (const auto& pair : body.contactPairs_)
        out << static_cast<int>(pair.kind) << ' '
            << static_cast<int>(pair.firstKind) << ' ' << pair.first << ' '
            << static_cast<int>(pair.secondKind) << ' ' << pair.second << ' '
            << pair.settings.normalCompliance << ' '
            << pair.settings.staticFriction << ' '
            << pair.settings.dynamicFriction << '\n';
    out << "MULTIPLIERS " << body.contactMultipliers_.size() << '\n';
    for (const auto& [feature, multiplier] : body.contactMultipliers_) {
        key(feature); out << multiplier << '\n';
    }
    out << "RECORDS " << body.contactRecords_.size() << '\n';
    for (const auto& record : body.contactRecords_) {
        key(record.key);
        out << static_cast<int>(record.kind) << ' '
            << static_cast<int>(record.ccdState) << ' ' << record.timeOfImpact
            << ' ' << record.bracketLower << ' ' << record.bracketUpper << ' '
            << record.ccdIterations << ' ' << record.intervalSubdivisions << ' '
            << record.usedIntervalFallback << ' ';
        for (const auto node : record.firstNodes) out << node << ' ';
        for (const auto node : record.secondNodes) out << node << ' ';
        out << record.firstNodeCount << ' ' << record.secondNodeCount << ' ';
        for (const auto weight : record.firstWeights) out << weight << ' ';
        for (const auto weight : record.secondWeights) out << weight << ' ';
        vec(record.firstPoint); vec(record.secondPoint); vec(record.normal);
        out << record.pairSeparation << ' ' << record.gap << ' '
            << record.penetration << ' ' << record.normalMultiplier << ' '
            << record.normalForceEstimate << ' '
            << record.normalImpulseMagnitude << ' ';
        vec(record.tangentialImpulse);
        out << static_cast<int>(record.frictionState) << ' '
            << record.frictionConeRatio << ' ' << record.normalResidual << ' '
            << record.frictionResidual << ' ' << record.frictionWork << ' '
            << record.tangentSpeedBefore << ' ' << record.tangentSpeedAfter
            << ' ';
        vec(record.firstImpulse); vec(record.secondImpulse);
        vec(record.firstMoment); vec(record.secondMoment);
        for (const auto& impulse : record.firstNodeImpulses) vec(impulse);
        for (const auto& impulse : record.secondNodeImpulses) vec(impulse);
        out << record.solverVisits << '\n';
    }
    out << "CONTACT_DIAGNOSTICS "; diagnostics(body.contactDiagnostics_);
    out << "\nPAIR_DIAGNOSTICS " << body.contactPairDiagnostics_.size()
        << '\n';
    for (const auto& value : body.contactPairDiagnostics_) {
        diagnostics(value); out << '\n';
    }
    const auto auditKeys = [&](const auto& values) {
        out << values.size() << ' ';
        for (const auto& value : values) key(value);
        out << '\n';
    };
    auditKeys(body.contactAudit_.iterationCandidateKeys);
    auditKeys(body.contactAudit_.iterationQueryKeys);
    auditKeys(body.contactAudit_.certificationCandidateKeys);
    auditKeys(body.contactAudit_.certificationQueryKeys);
    out << "AERO_REGISTRATION " << !body.aerodynamicRegistration_.expired()
        << '\n';
    return out.str();
}
#endif

} // namespace softwing
