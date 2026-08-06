#include "playground_metrics.h"

#include <QLatin1Char>
#include <QString>
#include <QStringList>

#include <algorithm>
#include <numeric>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace lep::playground {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kAirDensity = 1.225;   // kg/m^3, for the CSV airspeed
// A riser segment carrying less than this is hanging slack: with line
// junction masses of 50 g even an unloaded cascade reads a few tenths of
// a newton of its own weight, so zero is the wrong cut.
constexpr double kSlackRiserNewtons = 0.5;
// Nose nodes for the leading-edge dent: everything forward of 10% chord.
constexpr double kNoseChordFraction = 0.10;
// Strain is only meaningful on fabric-scale edges. The rib webs pin
// interpolated nodes onto the skin with ties whose rest length can be
// sub-micron (a station landing a hair away from an outline node), and
// (length - rest)/rest on those reads astronomically — a peak of 6e15%
// reached a legend before this floor existed. Mesh-welded fabric edges
// are at least half a millimetre by construction, so anything shorter
// is a pin, not cloth.
constexpr double kMinimumStrainRestLength = 5.0e-4;
// The mesh-relative strain floor: edges shorter than this fraction of
// the median drawn edge are excluded from the strain fields (see
// nodeStrainFields). Tracks subdivision, where a fixed floor cannot.
constexpr float kStrainRestMedianFraction = 0.3F;

using softwing::Vec3;

// Row-major 3x3, kept as plain arrays: the fits below are the only place
// the Playground needs a rotation as a matrix, and a full linear-algebra
// dependency for one 4x4 eigenproblem is not worth its weight.
struct Rotation
{
    std::array<std::array<double, 3>, 3> m{{{1.0, 0.0, 0.0},
                                            {0.0, 1.0, 0.0},
                                            {0.0, 0.0, 1.0}}};

    [[nodiscard]] Vec3 apply(const Vec3 &v) const
    {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    // The inverse of a rotation is its transpose; no solve needed.
    [[nodiscard]] Vec3 applyTransposed(const Vec3 &v) const
    {
        return {m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
                m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
                m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z};
    }
};

struct RigidFit
{
    Rotation rotation;
    Vec3 translation;

    [[nodiscard]] Vec3 apply(const Vec3 &rest) const
    {
        return rotation.apply(rest) + translation;
    }

    [[nodiscard]] Vec3 toRestFrame(const Vec3 &live) const
    {
        return rotation.applyTransposed(live - translation);
    }
};

// Largest eigenvector of a symmetric 4x4 by cyclic Jacobi rotations. The
// sweep count is fixed — convergence is quadratic and 32 sweeps is far
// past machine precision for these well-scaled matrices — so the result
// is deterministic: same body, same fit, bit for bit.
std::array<double, 4> largestEigenvector(
    std::array<std::array<double, 4>, 4> n)
{
    std::array<std::array<double, 4>, 4> v{{{1.0, 0.0, 0.0, 0.0},
                                            {0.0, 1.0, 0.0, 0.0},
                                            {0.0, 0.0, 1.0, 0.0},
                                            {0.0, 0.0, 0.0, 1.0}}};
    for (int sweep = 0; sweep < 32; ++sweep) {
        for (int p = 0; p < 3; ++p) {
            for (int q = p + 1; q < 4; ++q) {
                if (n[p][q] == 0.0) {
                    continue;
                }
                const double tau =
                    (n[q][q] - n[p][p]) / (2.0 * n[p][q]);
                // The smaller root: the rotation closest to identity,
                // which is what keeps the sweep numerically tame.
                const double t =
                    (tau >= 0.0 ? 1.0 : -1.0)
                    / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
                const double c = 1.0 / std::sqrt(1.0 + t * t);
                const double s = t * c;
                for (int k = 0; k < 4; ++k) {
                    const double nkp = n[k][p];
                    const double nkq = n[k][q];
                    n[k][p] = c * nkp - s * nkq;
                    n[k][q] = s * nkp + c * nkq;
                }
                for (int k = 0; k < 4; ++k) {
                    const double npk = n[p][k];
                    const double nqk = n[q][k];
                    n[p][k] = c * npk - s * nqk;
                    n[q][k] = s * npk + c * nqk;
                }
                for (int k = 0; k < 4; ++k) {
                    const double vkp = v[k][p];
                    const double vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    int best = 0;
    for (int index = 1; index < 4; ++index) {
        if (n[index][index] > n[best][best]) {
            best = index;
        }
    }
    return {v[0][best], v[1][best], v[2][best], v[3][best]};
}

// Horn's closed-form absolute orientation, rotation and translation only.
// No scale on purpose — that is the whole point of the alignment: trim
// rotation and translation must not count as shape error, while stretch
// and distortion must survive the fit and show up in the residuals.
RigidFit fitRigid(std::span<const Vec3> rest, std::span<const Vec3> live)
{
    RigidFit fit;
    const std::size_t count = std::min(rest.size(), live.size());
    if (count == 0) {
        return fit;
    }
    Vec3 restCentroid;
    Vec3 liveCentroid;
    for (std::size_t index = 0; index < count; ++index) {
        restCentroid += rest[index];
        liveCentroid += live[index];
    }
    restCentroid /= static_cast<double>(count);
    liveCentroid /= static_cast<double>(count);
    // Provisional identity-rotation fit; every degenerate exit below
    // keeps it, which is the honest answer when the rotation is
    // under-determined.
    fit.translation = liveCentroid - restCentroid;

    double longest = 0.0;
    Vec3 axis;
    for (std::size_t index = 0; index < count; ++index) {
        const Vec3 offset = rest[index] - restCentroid;
        if (lengthSquared(offset) > longest) {
            longest = lengthSquared(offset);
            axis = offset;
        }
    }
    if (longest <= 1.0e-24) {
        return fit;   // all rest points coincident
    }
    double offAxis = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        offAxis = std::max(
            offAxis,
            lengthSquared(cross(rest[index] - restCentroid, axis)));
    }
    if (offAxis <= 1.0e-16 * longest * longest) {
        return fit;   // collinear: rotation about the line is arbitrary
    }

    // Cross-covariance, rest indexing rows. Note the skew differences in
    // the first row of N: for identical point sets the paired sums are
    // bitwise equal, so N decouples exactly and the fit returns exact
    // identity — the fresh-body report owes its clean zeros to this.
    double s[3][3] = {};
    for (std::size_t index = 0; index < count; ++index) {
        const Vec3 r = rest[index] - restCentroid;
        const Vec3 l = live[index] - liveCentroid;
        const double rc[3] = {r.x, r.y, r.z};
        const double lc[3] = {l.x, l.y, l.z};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                s[row][column] += rc[row] * lc[column];
            }
        }
    }
    std::array<std::array<double, 4>, 4> n{};
    n[0][0] = s[0][0] + s[1][1] + s[2][2];
    n[0][1] = s[1][2] - s[2][1];
    n[0][2] = s[2][0] - s[0][2];
    n[0][3] = s[0][1] - s[1][0];
    n[1][1] = s[0][0] - s[1][1] - s[2][2];
    n[1][2] = s[0][1] + s[1][0];
    n[1][3] = s[2][0] + s[0][2];
    n[2][2] = -s[0][0] + s[1][1] - s[2][2];
    n[2][3] = s[1][2] + s[2][1];
    n[3][3] = -s[0][0] - s[1][1] + s[2][2];
    n[1][0] = n[0][1];
    n[2][0] = n[0][2];
    n[3][0] = n[0][3];
    n[2][1] = n[1][2];
    n[3][1] = n[1][3];
    n[3][2] = n[2][3];

    const std::array<double, 4> q = largestEigenvector(n);
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1]
                                  + q[2] * q[2] + q[3] * q[3]);
    if (norm <= 1.0e-12) {
        return fit;
    }
    const double w = q[0] / norm;
    const double x = q[1] / norm;
    const double y = q[2] / norm;
    const double z = q[3] / norm;
    fit.rotation.m = {{{1.0 - 2.0 * (y * y + z * z),
                        2.0 * (x * y - w * z),
                        2.0 * (x * z + w * y)},
                       {2.0 * (x * y + w * z),
                        1.0 - 2.0 * (x * x + z * z),
                        2.0 * (y * z - w * x)},
                       {2.0 * (x * z - w * y),
                        2.0 * (y * z + w * x),
                        1.0 - 2.0 * (x * x + y * y)}}};
    fit.translation = liveCentroid - fit.rotation.apply(restCentroid);
    return fit;
}

// The canopy-only fit every deviation is measured against. The live
// positions land in the caller's scratch so the per-frame heatmap path
// reuses its capacity instead of churning allocations.
RigidFit globalCanopyFit(const SimBody &sim,
                         const ShapeBaseline &baseline,
                         std::vector<Vec3> &liveScratch,
                         std::size_t &canopyCountOut)
{
    const auto &nodes = sim.body->nodes();
    const std::size_t count = std::min(
        {baseline.canopyNodeCount, baseline.restPositions.size(),
         nodes.size()});
    canopyCountOut = count;
    liveScratch.resize(count);
    for (std::size_t index = 0; index < count; ++index) {
        liveScratch[index] = nodes[index].position;
    }
    return fitRigid(
        std::span<const Vec3>(baseline.restPositions.data(), count),
        std::span<const Vec3>(liveScratch.data(), count));
}

// Mass-weighted RMS canopy velocity relative to the canopy's own bulk
// motion. Shared by measureShape and the settle loop, which polls it far
// too often to pay for a full report each time.
double agitationOf(const SimBody &sim)
{
    if (!sim.body) {
        return 0.0;
    }
    const auto &nodes = sim.body->nodes();
    const std::size_t count =
        sim.canopyNodeCount > 0
            ? std::min(sim.canopyNodeCount, nodes.size())
            : nodes.size();
    Vec3 momentum;
    double mass = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const softwing::Node &node = nodes[index];
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double nodeMass = 1.0 / node.inverseMass;
        momentum += nodeMass * node.velocity;
        mass += nodeMass;
    }
    if (!(mass > 0.0)) {
        return 0.0;
    }
    const Vec3 bulk = momentum / mass;
    double sum = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const softwing::Node &node = nodes[index];
        if (node.inverseMass <= 0.0) {
            continue;
        }
        sum += lengthSquared(node.velocity - bulk) / node.inverseMass;
    }
    return std::sqrt(sum / mass);
}

[[nodiscard]] bool isSkinSurface(SimSurface surface)
{
    return static_cast<int>(surface) < simExportedSurfaceCount;
}

}  // namespace

QString shapeFlagName(ShapeFlag flag)
{
    switch (flag) {
    case ShapeFlag::FrontTuckRisk:
        return QStringLiteral("FrontTuckRisk");
    case ShapeFlag::ProfileDistortion:
        return QStringLiteral("ProfileDistortion");
    case ShapeFlag::WashoutChange:
        return QStringLiteral("WashoutChange");
    case ShapeFlag::SlackFabric:
        return QStringLiteral("SlackFabric");
    case ShapeFlag::SpanLoss:
        return QStringLiteral("SpanLoss");
    case ShapeFlag::UnderInflated:
        return QStringLiteral("UnderInflated");
    case ShapeFlag::Asymmetry:
        return QStringLiteral("Asymmetry");
    case ShapeFlag::SlackRow:
        return QStringLiteral("SlackRow");
    case ShapeFlag::Unsettled:
        return QStringLiteral("Unsettled");
    }
    return QString();
}

ShapeBaseline captureShapeBaseline(const SimBody &sim)
{
    ShapeBaseline baseline;
    if (!sim.body) {
        return baseline;
    }
    const auto &nodes = sim.body->nodes();
    baseline.restPositions.reserve(nodes.size());
    for (const softwing::Node &node : nodes) {
        baseline.restPositions.push_back(node.position);
    }
    baseline.canopyNodeCount =
        std::min(sim.canopyNodeCount, nodes.size());

    // Area-weighted outward normals: the raw cross product is twice the
    // face's area vector, so summing it per corner weights by area for
    // free; the skin's right-hand winding is outward by construction
    // (buildSimBody orients it before adding triangles).
    baseline.restNormals.assign(nodes.size(), Vec3{});
    const auto &triangles = sim.body->triangles();
    const std::size_t skinFaces =
        std::min(sim.skinTriangleCount, triangles.size());
    for (std::size_t face = 0; face < skinFaces; ++face) {
        const softwing::Triangle &tri = triangles[face];
        const Vec3 normal =
            cross(nodes[tri.b].position - nodes[tri.a].position,
                  nodes[tri.c].position - nodes[tri.a].position);
        baseline.restNormals[tri.a] += normal;
        baseline.restNormals[tri.b] += normal;
        baseline.restNormals[tri.c] += normal;
    }
    for (Vec3 &normal : baseline.restNormals) {
        const double magnitude = length(normal);
        if (magnitude > 0.0) {
            normal /= magnitude;
        }
    }

    baseline.restRibLoops.reserve(sim.ribLoopNodes.size());
    for (const std::vector<std::size_t> &loop : sim.ribLoopNodes) {
        std::vector<Vec3> points;
        points.reserve(loop.size());
        for (const std::size_t node : loop) {
            points.push_back(node < nodes.size() ? nodes[node].position
                                                 : Vec3{});
        }
        baseline.restRibLoops.push_back(std::move(points));
    }

    // Mirror pairing off the rest span stations of the leading edges. The
    // tolerance is a quarter of the mean rib spacing: an asymmetric rib
    // layout should pair nothing rather than pair wrongly, and a designed
    // wing's mirror partner sits within a fraction of a bay of its
    // reflected station.
    const std::size_t ribCount = sim.ribChords.size();
    baseline.mirrorRib.resize(ribCount);
    std::vector<double> stations(ribCount, 0.0);
    for (std::size_t rib = 0; rib < ribCount; ++rib) {
        const std::size_t leading = sim.ribChords[rib].leadingNode;
        stations[rib] =
            leading < nodes.size()
                ? dot(nodes[leading].position, sim.restSpanAxis)
                : 0.0;
    }
    double spacing = 0.0;
    if (ribCount >= 2) {
        const auto [lowIt, highIt] =
            std::minmax_element(stations.begin(), stations.end());
        spacing = (*highIt - *lowIt) / static_cast<double>(ribCount - 1);
    }
    for (std::size_t rib = 0; rib < ribCount; ++rib) {
        std::size_t best = rib;
        double bestError = std::numeric_limits<double>::max();
        for (std::size_t other = 0; other < ribCount; ++other) {
            const double error =
                std::abs(stations[other] + stations[rib]);
            if (error < bestError) {
                bestError = error;
                best = other;
            }
        }
        baseline.mirrorRib[rib] =
            (ribCount >= 2 && bestError <= 0.25 * spacing) ? best : rib;
    }

    baseline.restVolume = sim.body->signedVolume(
        sim.body->surfaceGroup(0, sim.skinTriangleCount));
    double spanLow = std::numeric_limits<double>::max();
    double spanHigh = std::numeric_limits<double>::lowest();
    for (std::size_t index = 0; index < baseline.canopyNodeCount;
         ++index) {
        const double station =
            dot(nodes[index].position, sim.restSpanAxis);
        spanLow = std::min(spanLow, station);
        spanHigh = std::max(spanHigh, station);
    }
    baseline.restSpan =
        baseline.canopyNodeCount > 0 ? spanHigh - spanLow : 0.0;
    baseline.restArea = sim.planformArea;
    return baseline;
}

double constraintTensionNewtons(const SimBody &sim,
                                const SimControls &controls,
                                std::size_t constraint)
{
    if (!sim.body || constraint >= sim.body->constraints().size()) {
        return 0.0;
    }
    const softwing::DistanceConstraint &tie =
        sim.body->constraints()[constraint];
    // λ of the last substep; force = -λ/h² with h the SUBSTEP, not the
    // frame — the accumulated multiplier converged against the substep's
    // own compliance scaling.
    const double substepRate = controls.substeps / simulationTimeStep;
    const double tension =
        -tie.accumulatedLambda * substepRate * substepRate;
    return tie.kind == softwing::ConstraintKind::Cable
               ? std::max(0.0, tension)
               : tension;
}

WeakCellReport weakestCell(const SimBody &sim)
{
    WeakCellReport worst;
    if (!sim.body) {
        return worst;
    }
    worst.sectionRatio = std::numeric_limits<double>::infinity();
    const auto &nodes = sim.body->nodes();
    for (std::size_t index = 0; index < sim.cells.size(); ++index) {
        const SimCell &cell = sim.cells[index];
        if (cell.restSectionArea <= 0.0) {
            continue;
        }
        double area = 0.0;
        for (const std::size_t rib : cell.ribs) {
            softwing::Vec3 sum;
            const auto &loop = sim.ribLoopNodes[rib];
            for (std::size_t node = 0; node < loop.size(); ++node) {
                sum += cross(nodes[loop[node]].position,
                             nodes[loop[(node + 1) % loop.size()]].position);
            }
            area += 0.5 * length(sum);
        }
        const double ratio = 0.5 * area / cell.restSectionArea;
        if (ratio < worst.sectionRatio) {
            worst.index = index;
            worst.sectionRatio = ratio;
            worst.x =
                0.5
                * (nodes[sim.ribChords[cell.ribs[0]].leadingNode].position.x
                   + nodes[sim.ribChords[cell.ribs[1]].leadingNode]
                         .position.x);
            worst.pressurePascal = index < sim.cellPressure.size()
                                       ? sim.cellPressure[index]
                                       : 0.0;
            worst.volumeRatio = index < sim.cellVolumeRatio.size()
                                    ? sim.cellVolumeRatio[index]
                                    : 1.0;
            worst.ramPressurePascal = index < sim.cellRamPressure.size()
                                          ? sim.cellRamPressure[index]
                                          : 0.0;
            worst.intakeOpening = index < sim.cellIntakeOpening.size()
                                      ? sim.cellIntakeOpening[index]
                                      : 0.0;
        }
    }
    if (!std::isfinite(worst.sectionRatio)) {
        worst.sectionRatio = 1.0;
    }
    return worst;
}

KinkReport sharpestKink(const SimBody &sim)
{
    KinkReport worst;
    if (!sim.body || sim.cells.empty()) {
        return worst;
    }
    // Span order comes off the cells: they were built from ribs sorted
    // along the rest span axis, adjacent pairs in order.
    std::vector<std::size_t> order;
    order.push_back(sim.cells.front().ribs[0]);
    for (const SimCell &cell : sim.cells) {
        order.push_back(cell.ribs[1]);
    }
    const auto &nodes = sim.body->nodes();
    for (std::size_t index = 1; index + 1 < order.size(); ++index) {
        const softwing::Vec3 before =
            nodes[sim.ribChords[order[index]].leadingNode].position
            - nodes[sim.ribChords[order[index - 1]].leadingNode].position;
        const softwing::Vec3 after =
            nodes[sim.ribChords[order[index + 1]].leadingNode].position
            - nodes[sim.ribChords[order[index]].leadingNode].position;
        if (length(before) <= 0.0 || length(after) <= 0.0) {
            continue;
        }
        const double cosine =
            std::clamp(dot(before, after) / (length(before) * length(after)),
                       -1.0,
                       1.0);
        const double degrees = std::acos(cosine) * 180.0 / kPi;
        if (degrees > worst.degrees) {
            worst.rib = order[index];
            worst.degrees = degrees;
            worst.x =
                nodes[sim.ribChords[order[index]].leadingNode].position.x;
            worst.spanFraction = static_cast<double>(index)
                                 / static_cast<double>(order.size() - 1);
        }
    }
    return worst;
}

namespace {

bool riserReaction(const SimBody &sim,
                   const LineSegment &segment,
                   double tension,
                   Vec3 &reaction)
{
    if (!segment.suspension) {
        return false;
    }
    const auto carabiner = [&sim](std::size_t node) {
        return std::find(sim.carabinerNodes.begin(),
                         sim.carabinerNodes.end(),
                         node)
               != sim.carabinerNodes.end();
    };
    const bool aIsCarabiner = carabiner(segment.a);
    const bool bIsCarabiner = carabiner(segment.b);
    if (aIsCarabiner == bIsCarabiner) {
        return false;
    }
    const std::size_t lower = aIsCarabiner ? segment.a : segment.b;
    const std::size_t upper = aIsCarabiner ? segment.b : segment.a;
    const auto &nodes = sim.body->nodes();
    if (lower >= nodes.size() || upper >= nodes.size()) {
        return false;
    }
    const Vec3 direction = nodes[upper].position - nodes[lower].position;
    if (lengthSquared(direction) <= 0.0) {
        return false;
    }
    reaction = tension * normalized(direction);
    return true;
}

}  // namespace

LineLoadReport lineLoads(const SimBody &sim, const SimControls &controls)
{
    LineLoadReport report;
    if (!sim.body) {
        return report;
    }
    for (const LineSegment &segment : sim.lineSegments) {
        if (!segment.suspension) {
            continue;
        }
        ++report.totalSegments;
        if (segment.constraint >= sim.body->constraints().size()
            || segment.a >= sim.body->nodes().size()
            || segment.b >= sim.body->nodes().size()) {
            continue;
        }
        const softwing::DistanceConstraint &constraint =
            sim.body->constraints()[segment.constraint];
        const double extension = std::max(
            0.0,
            length(sim.body->nodes()[segment.b].position
                   - sim.body->nodes()[segment.a].position)
                - constraint.restLength);
        report.maximumExtensionMetres = std::max(
            report.maximumExtensionMetres, extension);
        if (constraint.restLength > 1.0e-12) {
            report.maximumExtensionFraction = std::max(
                report.maximumExtensionFraction,
                extension / constraint.restLength);
        }
        const double tension =
            constraintTensionNewtons(sim, controls, segment.constraint);
        report.maximumTensionNewtons = std::max(
            report.maximumTensionNewtons, tension);
        if (tension < 1.0) {
            ++report.slackSegments;
        }
        Vec3 reaction;
        if (riserReaction(sim, segment, tension, reaction)) {
            report.riserForce += reaction;
        }
    }
    report.riserNewtons = length(report.riserForce);
    return report;
}

double simulatedMassKilograms(const SimBody &sim)
{
    if (!sim.body) {
        return 0.0;
    }
    double mass = 0.0;
    for (const softwing::Node &node : sim.body->nodes()) {
        if (node.inverseMass > 0.0) {
            mass += 1.0 / node.inverseMass;
        }
    }
    return std::max(0.0, mass - sim.virtualAddedAirMassKg);
}

ShapeReport measureShape(const SimBody &sim,
                         const SimControls &controls,
                         const ShapeBaseline &baseline)
{
    ShapeReport report;
    report.alphaDegrees = controls.angleOfAttackDegrees;
    report.dynamicPressurePascal = controls.pressurePascal;
    report.pressureSolve = sim.pressureSolve;
    if (!sim.body || baseline.restPositions.empty()) {
        return report;
    }
    const auto &nodes = sim.body->nodes();

    std::vector<Vec3> livePositions;
    std::size_t canopyCount = 0;
    const RigidFit fit =
        globalCanopyFit(sim, baseline, livePositions, canopyCount);
    if (canopyCount == 0) {
        return report;
    }
    std::vector<Vec3> deviations(canopyCount);
    for (std::size_t index = 0; index < canopyCount; ++index) {
        deviations[index] =
            livePositions[index] - fit.apply(baseline.restPositions[index]);
    }

    // Node -> (chord fraction, majority rib), rebuilt from the face table
    // each call. Linear in faces; the natural place to cache it would be
    // the baseline, but ShapeBaseline is a frozen contract, and a body
    // this size rebuilds it in well under a millisecond.
    const std::size_t ribCount = std::min(
        {sim.ribChords.size(), sim.ribLoopNodes.size(),
         baseline.restRibLoops.size()});
    std::vector<double> nodeAreaSum(canopyCount, 0.0);
    std::vector<double> nodeFractionSum(canopyCount, 0.0);
    std::vector<double> nodeUpperArea(canopyCount, 0.0);
    std::vector<std::uint32_t> nodeRib(canopyCount, 0);
    const bool haveAero =
        ribCount > 0 && sim.faceAero.size() >= sim.skinTriangleCount
        && sim.renderFaces.size() >= sim.skinTriangleCount;
    if (haveAero) {
        std::vector<std::pair<std::uint64_t, double>> votes;
        votes.reserve(sim.skinTriangleCount * 3);
        for (std::size_t face = 0; face < sim.skinTriangleCount;
             ++face) {
            const RenderFace &drawn = sim.renderFaces[face];
            const std::size_t a = drawn.nodes[0];
            const std::size_t b = drawn.nodes[1];
            const std::size_t c = drawn.nodes[2];
            if (a >= canopyCount || b >= canopyCount
                || c >= canopyCount) {
                continue;
            }
            // Rest areas, so a fluttering face cannot re-vote its nodes
            // between frames.
            const double area =
                0.5
                * length(cross(baseline.restPositions[b]
                                   - baseline.restPositions[a],
                               baseline.restPositions[c]
                                   - baseline.restPositions[a]));
            const FaceAero &aero = sim.faceAero[face];
            if (aero.rib >= ribCount) {
                continue;
            }
            for (const std::size_t node : {a, b, c}) {
                nodeAreaSum[node] += area;
                nodeFractionSum[node] += area * aero.chordFraction;
                if (aero.upperSurface) {
                    nodeUpperArea[node] += area;
                }
                votes.emplace_back(
                    static_cast<std::uint64_t>(node) * ribCount
                        + aero.rib,
                    area);
            }
        }
        std::sort(votes.begin(), votes.end());
        std::vector<double> bestArea(canopyCount, 0.0);
        std::size_t index = 0;
        while (index < votes.size()) {
            const std::uint64_t key = votes[index].first;
            double area = 0.0;
            while (index < votes.size() && votes[index].first == key) {
                area += votes[index].second;
                ++index;
            }
            const auto node = static_cast<std::size_t>(key / ribCount);
            // Strictly greater keeps the lowest rib on an exact tie,
            // which the sort order makes deterministic.
            if (area > bestArea[node]) {
                bestArea[node] = area;
                nodeRib[node] = static_cast<std::uint32_t>(key % ribCount);
            }
        }
    }

    // Per-rib section fits.
    report.ribs.resize(ribCount);
    std::vector<Vec3> liveLoop;
    std::vector<double> ribDent(ribCount, 0.0);
    for (std::size_t rib = 0; rib < ribCount; ++rib) {
        RibShape &shape = report.ribs[rib];
        const std::vector<std::size_t> &loopNodes = sim.ribLoopNodes[rib];
        const std::vector<Vec3> &restLoop = baseline.restRibLoops[rib];
        const std::size_t loopCount =
            std::min(loopNodes.size(), restLoop.size());
        liveLoop.resize(loopCount);
        bool loopValid = loopCount >= 3;
        for (std::size_t point = 0; point < loopCount; ++point) {
            if (loopNodes[point] >= nodes.size()) {
                loopValid = false;
                break;
            }
            liveLoop[point] = nodes[loopNodes[point]].position;
        }
        if (loopValid) {
            const RigidFit section = fitRigid(
                std::span<const Vec3>(restLoop.data(), loopCount),
                std::span<const Vec3>(liveLoop.data(), loopCount));
            double sumSquared = 0.0;
            double worst = 0.0;
            for (std::size_t point = 0; point < loopCount; ++point) {
                const double residual = length(
                    liveLoop[point] - section.apply(restLoop[point]));
                sumSquared += residual * residual;
                worst = std::max(worst, residual);
            }
            shape.rmsMetres =
                std::sqrt(sumSquared / static_cast<double>(loopCount));
            shape.maxMetres = worst;
        }

        const RibChord &chord = sim.ribChords[rib];
        if (chord.leadingNode >= nodes.size()
            || chord.trailingNode >= nodes.size()
            || chord.leadingNode >= baseline.restPositions.size()
            || chord.trailingNode >= baseline.restPositions.size()) {
            continue;
        }
        const Vec3 liveChord = nodes[chord.trailingNode].position
                               - nodes[chord.leadingNode].position;
        if (chord.restChordLength > 0.0) {
            shape.chordRatio =
                length(liveChord) / chord.restChordLength;
        }

        // Twist: the angle from the rigidly carried rest chord to the
        // live one, about the rib's live span axis, both flattened into
        // the plane perpendicular to it so arc and sweep stay out of the
        // pitch measurement. atan2 of the axis-signed cross measures the
        // right-hand rotation about the axis; in the mesh frame (span
        // +x, chord LE->TE +y, up +z) a right-hand rotation about the
        // span axis carries the trailing edge UP — nose-DOWN — so the
        // nose-up convention is its negation. The unit test pins this
        // with a synthetic rotation; do not "simplify" the sign.
        const Vec3 axis = normalized(fit.rotation.apply(chord.spanAxis));
        const Vec3 restChord =
            baseline.restPositions[chord.trailingNode]
            - baseline.restPositions[chord.leadingNode];
        const Vec3 carried = fit.rotation.apply(restChord);
        const Vec3 a = carried - dot(carried, axis) * axis;
        const Vec3 b = liveChord - dot(liveChord, axis) * axis;
        if (length(a) > 1.0e-9 && length(b) > 1.0e-9) {
            shape.twistDegrees =
                -std::atan2(dot(cross(a, b), axis), dot(a, b))
                * kRadiansToDegrees;
        }
    }

    // Leading-edge dent: worst inward normal displacement over each rib's
    // EXTRADOS nose nodes, against the globally aligned rest pose.
    // Extrados only, because the vents and the intrados stagnation region
    // carry no pressure load (Cp sits at stagnation there), so their
    // fabric flexes freely and read a standing ~80 mm on a perfectly
    // healthy wing. Measuring against each rib's own section fit was
    // tried and is WORSE as a collapse detector: the loop fit chases the
    // fold, compressing a 12-90x clean/collapsed separation down to 2-4x.
    // A healthy loaded wing still carries a standing 60-130 mm here (the
    // unloaded mid-cell nose relaxing inward from its designed ballooning
    // — the reason real wings grew nose rods), which is why the flag
    // threshold sits at 200 mm: nose TRAVEL past that is a fold, however
    // attitude and deformation share it.
    if (haveAero) {
        for (std::size_t node = 0; node < canopyCount; ++node) {
            if (nodeAreaSum[node] <= 0.0
                || node >= baseline.restNormals.size()) {
                continue;
            }
            // Any extrados contact keeps the node: the nose ring itself
            // sits exactly on the extrados/intrados boundary, and a
            // majority test would exclude the very line a tuck folds.
            if (nodeUpperArea[node] <= 0.0) {
                continue;
            }
            const double fraction =
                nodeFractionSum[node] / nodeAreaSum[node];
            if (fraction >= kNoseChordFraction) {
                continue;
            }
            const Vec3 &restNormal = baseline.restNormals[node];
            if (lengthSquared(restNormal) <= 0.0) {
                continue;
            }
            const std::size_t rib = nodeRib[node];
            if (rib >= ribCount) {
                continue;
            }
            const double dent = -dot(deviations[node],
                                     fit.rotation.apply(restNormal));
            ribDent[rib] = std::max(ribDent[rib], dent);
        }
        for (std::size_t rib = 0; rib < ribCount; ++rib) {
            report.ribs[rib].leadingEdgeDentMetres =
                std::max(0.0, ribDent[rib]);
        }
    }

    // Wing-level ratios.
    const Vec3 liveSpanAxis =
        normalized(fit.rotation.apply(sim.restSpanAxis));
    if (baseline.restSpan > 0.0 && lengthSquared(liveSpanAxis) > 0.0) {
        double low = std::numeric_limits<double>::max();
        double high = std::numeric_limits<double>::lowest();
        for (std::size_t index = 0; index < canopyCount; ++index) {
            const double station =
                dot(livePositions[index], liveSpanAxis);
            low = std::min(low, station);
            high = std::max(high, station);
        }
        report.spanRatio = (high - low) / baseline.restSpan;
    }
    if (haveAero && baseline.restArea > 0.0) {
        // The same projected-planform sum buildSimBody seeds restArea
        // with: upper-surface faces, upward component only.
        double projected = 0.0;
        for (std::size_t face = 0; face < sim.skinTriangleCount;
             ++face) {
            if (!sim.faceAero[face].upperSurface) {
                continue;
            }
            const RenderFace &drawn = sim.renderFaces[face];
            const Vec3 &a = nodes[drawn.nodes[0]].position;
            const Vec3 &b = nodes[drawn.nodes[1]].position;
            const Vec3 &c = nodes[drawn.nodes[2]].position;
            projected += std::max(0.0, 0.5 * cross(b - a, c - a).z);
        }
        report.areaRatio = projected / baseline.restArea;
    }
    if (std::abs(baseline.restVolume) > 0.0) {
        report.volumeRatio =
            sim.body->signedVolume(
                sim.body->surfaceGroup(0, sim.skinTriangleCount))
            / baseline.restVolume;
    }

    // Slack fabric comes from the selected skin physics. The membrane path
    // counts its two normal material strains (warp/weft) directly; the legacy
    // path keeps its unique exported-skin edge calculation unchanged.
    const auto &constraints = sim.body->constraints();
    {
        std::size_t materialDirections = 0;
        std::size_t slackDirections = 0;
        for (const RenderFace &face : sim.renderFaces) {
            if (!isSkinSurface(face.surface) || !face.membraneElement) {
                continue;
            }
            const auto diagnostics =
                sim.body->membraneDiagnostics(*face.membraneElement);
            materialDirections += 2;
            slackDirections +=
                diagnostics.greenStrain.x < slackStrainThreshold ? 1 : 0;
            slackDirections +=
                diagnostics.greenStrain.y < slackStrainThreshold ? 1 : 0;
        }
        if (materialDirections > 0) {
            report.slackFraction =
                static_cast<double>(slackDirections)
                / static_cast<double>(materialDirections);
        } else {
        std::vector<char> counted(constraints.size(), 0);
        std::size_t skinEdges = 0;
        std::size_t slackEdges = 0;
        for (const RenderFace &face : sim.renderFaces) {
            if (!isSkinSurface(face.surface)) {
                continue;
            }
            for (const std::size_t edge : face.edges) {
                if (edge == noConstraint || edge >= constraints.size()
                    || counted[edge] != 0) {
                    continue;
                }
                counted[edge] = 1;
                const softwing::DistanceConstraint &tie =
                    constraints[edge];
                if (tie.restLength < kMinimumStrainRestLength
                    || tie.a >= nodes.size() || tie.b >= nodes.size()) {
                    continue;
                }
                const double strain =
                    (length(nodes[tie.b].position
                            - nodes[tie.a].position)
                     - tie.restLength)
                    / tie.restLength;
                ++skinEdges;
                if (strain < slackStrainThreshold) {
                    ++slackEdges;
                }
            }
        }
        report.slackFraction =
            skinEdges > 0 ? static_cast<double>(slackEdges)
                                / static_cast<double>(skinEdges)
                          : 0.0;
        }
    }

    // Asymmetry: live sections carried back into the rest frame and
    // mirrored across the rest symmetry plane, against the partner's own
    // rest-frame position. Under symmetric input this is zero whatever
    // the wing is doing globally; a value here is the model reporting a
    // symmetry it was given and lost.
    {
        Vec3 restCentroid;
        for (std::size_t index = 0; index < canopyCount; ++index) {
            restCentroid += baseline.restPositions[index];
        }
        restCentroid /= static_cast<double>(canopyCount);
        const Vec3 mirrorAxis = normalized(sim.restSpanAxis);
        double sumSquared = 0.0;
        std::size_t samples = 0;
        const std::size_t pairCount =
            std::min(ribCount, baseline.mirrorRib.size());
        for (std::size_t rib = 0; rib < pairCount; ++rib) {
            const std::size_t partner = baseline.mirrorRib[rib];
            // Each unordered pair once; self-paired centre ribs measure
            // nothing.
            if (partner <= rib || partner >= ribCount) {
                continue;
            }
            const RibChord &own = sim.ribChords[rib];
            const RibChord &other = sim.ribChords[partner];
            for (const auto [ownNode, otherNode] :
                 {std::pair{own.leadingNode, other.leadingNode},
                  std::pair{own.trailingNode, other.trailingNode}}) {
                if (ownNode >= nodes.size()
                    || otherNode >= nodes.size()) {
                    continue;
                }
                const Vec3 ownRest =
                    fit.toRestFrame(nodes[ownNode].position);
                const Vec3 otherRest =
                    fit.toRestFrame(nodes[otherNode].position);
                const Vec3 mirrored =
                    ownRest
                    - 2.0 * dot(ownRest - restCentroid, mirrorAxis)
                          * mirrorAxis;
                sumSquared += lengthSquared(mirrored - otherRest);
                ++samples;
            }
        }
        report.asymmetryMetres =
            samples > 0
                ? std::sqrt(sumSquared / static_cast<double>(samples))
                : 0.0;
    }

    report.agitationMetresPerSecond = agitationOf(sim);

    // Row loads at the authored suspension cut immediately above each
    // carabiner. Harness ties below and synthesized brake-control cables are
    // drawn as lines too, but counting either would measure the same load path
    // again rather than another canopy reaction.
    struct RowTally
    {
        Vec3 left;
        Vec3 right;
        int leftSegments = 0;
        int rightSegments = 0;
        int leftSlack = 0;
        int rightSlack = 0;
        bool brake = false;
        bool present = false;
    };
    std::array<RowTally, 6> tallies{};
    // Row loads only when the mesh carries authored plan tags (plans 1-5;
    // plan 6 alone is the engine's own brake synthesis and proves
    // nothing). On an old mesh every A-E segment reads plan 0 and would
    // be skipped, so the tally would present ~1 kN of unmeasured riser
    // load as a measured near-zero brake row — an empty rows table is
    // the honest report, and it is what lets the bench print its
    // mesh-predates-the-tags notice.
    const bool meshHasPlans = std::any_of(
        sim.lineSegments.begin(),
        sim.lineSegments.end(),
        [](const LineSegment &segment) {
            return segment.plan >= 1 && segment.plan <= 5;
        });
    if (meshHasPlans) {
        const Vec3 spanAxis = normalized(sim.restSpanAxis);
        Vec3 totalReaction;
        for (const LineSegment &segment : sim.lineSegments) {
            const double tension = constraintTensionNewtons(
                sim, controls, segment.constraint);
            Vec3 reaction;
            if (segment.plan <= 0
                || !riserReaction(sim, segment, tension, reaction)) {
                continue;
            }
            int row;
            bool brake = false;
            if (segment.plan == 6 || segment.brake) {
                row = 5;
                brake = true;
            } else if (segment.plan >= 1 && segment.plan <= 5) {
                row = segment.plan - 1;
            } else {
                continue;
            }
            // Side from the REST midpoint: a collapsed tip that crosses
            // the centreline must not migrate its row to the other side.
            const Vec3 restA =
                segment.a < baseline.restPositions.size()
                    ? baseline.restPositions[segment.a]
                    : nodes[segment.a].position;
            const Vec3 restB =
                segment.b < baseline.restPositions.size()
                    ? baseline.restPositions[segment.b]
                    : nodes[segment.b].position;
            const double station =
                dot((restA + restB) * 0.5, spanAxis);
            const bool slack = tension < kSlackRiserNewtons;
            RowTally &tally = tallies[static_cast<std::size_t>(row)];
            tally.present = true;
            tally.brake = tally.brake || brake;
            if (station < 0.0) {
                tally.left += reaction;
                ++tally.leftSegments;
                tally.leftSlack += slack ? 1 : 0;
            } else {
                tally.right += reaction;
                ++tally.rightSegments;
                tally.rightSlack += slack ? 1 : 0;
            }
            totalReaction += reaction;
            report.slackRiserSegments += slack ? 1 : 0;
        }
        report.lineLoadNewtons = length(totalReaction);
        for (std::size_t row = 0; row < tallies.size(); ++row) {
            const RowTally &tally = tallies[row];
            if (!tally.present) {
                continue;
            }
            RowLoad load;
            load.row = QLatin1Char(static_cast<char>('A' + row));
            load.brake = tally.brake;
            load.leftNewtons = length(tally.left);
            load.rightNewtons = length(tally.right);
            load.segments = tally.leftSegments + tally.rightSegments;
            load.slackSegments = tally.leftSlack + tally.rightSlack;
            report.rows.push_back(load);
        }
    }

    // The polar's numbers only when its pass actually ran this mode;
    // last* fields survive a mode switch and stale flight numbers must
    // not leak into a tunnel report.
    if (controls.freeFlight || controls.flightLoad) {
        report.liftNewtons = sim.lastLift;
        report.dragNewtons = sim.lastDrag;
        report.glideRatio = sim.lastGlideRatio;
    }

    for (std::size_t rib = 0; rib < ribCount; ++rib) {
        const RibShape &shape = report.ribs[rib];
        if (shape.rmsMetres > report.worstDeviationMetres) {
            report.worstDeviationMetres = shape.rmsMetres;
            report.worstDeviationRib = rib;
        }
        if (shape.leadingEdgeDentMetres
            > report.worstLeadingEdgeDentMetres) {
            report.worstLeadingEdgeDentMetres =
                shape.leadingEdgeDentMetres;
            report.worstLeadingEdgeDentRib = rib;
        }
        if (std::abs(shape.twistDegrees)
            > std::abs(report.worstTwistDegrees)) {
            report.worstTwistDegrees = shape.twistDegrees;
            report.worstTwistRib = rib;
        }
    }

    // Flags: heuristic tripwires, one per finding, worst offender named.
    // Both per-rib flags scale by the wing's own chords, with the floors
    // in units of the MEAN chord: a fixed metre threshold calibrated on
    // one wing would be blind on a tandem and trigger-happy on a mini,
    // and the stubby tip and stabilo sections are exactly the ones that
    // are floppy while healthy.
    double meanChord = 0.0;
    for (std::size_t rib = 0; rib < ribCount; ++rib) {
        meanChord += sim.ribChords[rib].restChordLength;
    }
    meanChord /= static_cast<double>(std::max<std::size_t>(1, ribCount));
    {
        double worstRatio = 0.0;
        std::size_t worstRib = 0;
        for (std::size_t rib = 0; rib < ribCount; ++rib) {
            const double threshold = std::max(
                flagLeadingEdgeDentMeanChordFloor * meanChord,
                flagLeadingEdgeDentChordFraction
                    * sim.ribChords[rib].restChordLength);
            if (threshold <= 0.0) {
                continue;
            }
            const double ratio =
                report.ribs[rib].leadingEdgeDentMetres / threshold;
            if (ratio > worstRatio) {
                worstRatio = ratio;
                worstRib = rib;
            }
        }
        if (worstRatio > 1.0) {
            report.flags.push_back(
                {ShapeFlag::FrontTuckRisk,
                 QStringLiteral("LE dent %1 mm at rib %2")
                     .arg(report.ribs[worstRib].leadingEdgeDentMetres
                              * 1000.0,
                          0, 'f', 0)
                     .arg(static_cast<qulonglong>(worstRib))});
        }
    }
    {
        // Against each rib's own chord: a 10 mm ripple is noise on a
        // 3 m centre section and a buckle on a 0.5 m tip. Judged against
        // no less than half the mean chord, so healthy-but-floppy tip
        // sections do not flag the wing.
        double worstRatio = 0.0;
        std::size_t worstRib = 0;
        for (std::size_t rib = 0; rib < ribCount; ++rib) {
            const double effectiveChord = std::max(
                sim.ribChords[rib].restChordLength, 0.5 * meanChord);
            if (effectiveChord <= 0.0) {
                continue;
            }
            const double ratio =
                report.ribs[rib].rmsMetres
                / (flagSectionRmsChordFraction * effectiveChord);
            if (ratio > worstRatio) {
                worstRatio = ratio;
                worstRib = rib;
            }
        }
        if (worstRatio > 1.0) {
            report.flags.push_back(
                {ShapeFlag::ProfileDistortion,
                 QStringLiteral("section RMS %1 mm at rib %2")
                     .arg(report.ribs[worstRib].rmsMetres * 1000.0, 0,
                          'f', 0)
                     .arg(static_cast<qulonglong>(worstRib))});
        }
    }
    if (std::abs(report.worstTwistDegrees) > flagTwistChangeDegrees) {
        report.flags.push_back(
            {ShapeFlag::WashoutChange,
             QStringLiteral("twist %1 deg at rib %2")
                 .arg(report.worstTwistDegrees, 0, 'f', 1)
                 .arg(static_cast<qulonglong>(report.worstTwistRib))});
    }
    if (report.slackFraction > flagSlackFabricFraction) {
        report.flags.push_back(
            {ShapeFlag::SlackFabric,
             QStringLiteral("%1% of skin edges slack")
                 .arg(report.slackFraction * 100.0, 0, 'f', 0)});
    }
    if (report.spanRatio < flagSpanLossRatio) {
        report.flags.push_back(
            {ShapeFlag::SpanLoss,
             QStringLiteral("span at %1% of rest")
                 .arg(report.spanRatio * 100.0, 0, 'f', 0)});
    }
    // A loaded wing balloons well past its rest volume (the Swoop harness
    // settles near +23%), so sitting under +5% under airflow means the
    // cells never took ram pressure. With no airflow at all the wing is
    // simply uninflated, which is a state, not a warning.
    if (controls.pressurePascal > 0.0
        && report.volumeRatio < flagUnderInflatedVolumeRatio) {
        report.flags.push_back(
            {ShapeFlag::UnderInflated,
             QStringLiteral("volume at %1% of rest")
                 .arg(report.volumeRatio * 100.0, 0, 'f', 0)});
    }
    if (baseline.restSpan > 0.0
        && report.asymmetryMetres
               > flagAsymmetrySpanFraction * baseline.restSpan) {
        report.flags.push_back(
            {ShapeFlag::Asymmetry,
             QStringLiteral("mirror error %1 mm")
                 .arg(report.asymmetryMetres * 1000.0, 0, 'f', 0)});
    }
    {
        // One side of a row shedding load while its mirror still carries
        // is the precursor the per-wing totals average away. Judged by
        // FORCE ratio, not slack-segment counts: rows commonly run
        // redundant parallel riser cables, and which of two equal-length
        // cables ends up carrying the load is solver bookkeeping, not a
        // structural signal. Brake rows are exempt — the tunnel rigs them
        // with a deliberate gap, so they are slack by design.
        constexpr double kLoadedRowNewtons = 10.0;
        double worstRatio = flagSlackRowFraction;
        std::size_t worstRow = 0;
        bool worstLeft = false;
        bool found = false;
        for (std::size_t row = 0; row < tallies.size(); ++row) {
            const RowTally &tally = tallies[row];
            if (!tally.present || tally.brake) {
                continue;
            }
            const double left = length(tally.left);
            const double right = length(tally.right);
            const double low = std::min(left, right);
            const double high = std::max(left, right);
            if (high <= kLoadedRowNewtons) {
                continue;
            }
            const double ratio = low / high;
            if (ratio < worstRatio) {
                worstRatio = ratio;
                worstRow = row;
                worstLeft = left < right;
                found = true;
            }
        }
        if (found) {
            report.flags.push_back(
                {ShapeFlag::SlackRow,
                 QStringLiteral("row %1 unloaded on %2")
                     .arg(QLatin1Char(
                         static_cast<char>('A' + worstRow)))
                     .arg(worstLeft ? QStringLiteral("left")
                                    : QStringLiteral("right"))});
        }
    }

    return report;
}

void nodeDeviationField(const SimBody &sim,
                        const ShapeBaseline &baseline,
                        std::vector<float> &metresOut)
{
    if (!sim.body) {
        metresOut.clear();
        return;
    }
    metresOut.assign(sim.body->nodes().size(), 0.0F);
    if (baseline.restPositions.empty()) {
        return;
    }
    std::vector<Vec3> livePositions;
    std::size_t canopyCount = 0;
    const RigidFit fit =
        globalCanopyFit(sim, baseline, livePositions, canopyCount);
    for (std::size_t index = 0; index < canopyCount; ++index) {
        metresOut[index] = static_cast<float>(
            length(livePositions[index]
                   - fit.apply(baseline.restPositions[index])));
    }
}

void faceSlackField(const SimBody &sim, std::vector<float> &strainOut)
{
    strainOut.assign(sim.renderFaces.size(), 0.0F);
    if (!sim.body) {
        return;
    }
    const auto &nodes = sim.body->nodes();
    const auto &constraints = sim.body->constraints();
    for (std::size_t face = 0; face < sim.renderFaces.size(); ++face) {
        const RenderFace &drawn = sim.renderFaces[face];
        if (!isSkinSurface(drawn.surface)) {
            continue;
        }
        if (drawn.membraneElement) {
            const auto diagnostics =
                sim.body->membraneDiagnostics(*drawn.membraneElement);
            strainOut[face] = static_cast<float>(
                std::min({0.0,
                          diagnostics.greenStrain.x,
                          diagnostics.greenStrain.y}));
            continue;
        }
        double worst = 0.0;
        for (const std::size_t edge : drawn.edges) {
            if (edge == noConstraint || edge >= constraints.size()) {
                continue;
            }
            const softwing::DistanceConstraint &tie = constraints[edge];
            if (tie.restLength < kMinimumStrainRestLength
                || tie.a >= nodes.size() || tie.b >= nodes.size()) {
                continue;
            }
            const double strain =
                (length(nodes[tie.b].position - nodes[tie.a].position)
                 - tie.restLength)
                / tie.restLength;
            worst = std::min(worst, strain);
        }
        strainOut[face] = static_cast<float>(worst);
    }
}

void faceInteriorPressureField(const SimBody &sim,
                               std::vector<float> &pascalOut)
{
    pascalOut.assign(sim.renderFaces.size(), 0.0F);
    const std::size_t skinFaces = std::min(
        {sim.skinTriangleCount, sim.faceInteriorPressure.size(),
         sim.renderFaces.size()});
    for (std::size_t face = 0; face < skinFaces; ++face) {
        pascalOut[face] =
            static_cast<float>(sim.faceInteriorPressure[face]);
    }
}

void faceExteriorPressureCoefficientField(
    const SimBody &sim,
    std::vector<float> &coefficientOut)
{
    coefficientOut.assign(sim.renderFaces.size(), 0.0F);
    const std::size_t skinFaces = std::min(
        {sim.skinTriangleCount, sim.faceAppliedExternalCp.size(),
         sim.renderFaces.size()});
    for (std::size_t face = 0; face < skinFaces; ++face) {
        coefficientOut[face] =
            static_cast<float>(sim.faceAppliedExternalCp[face]);
    }
}

void facePressureDifferenceField(const SimBody &sim,
                                 std::vector<float> &pascalOut)
{
    pascalOut.assign(sim.renderFaces.size(), 0.0F);
    if (!sim.body) {
        return;
    }
    const auto &triangles = sim.body->triangles();
    const std::size_t skinFaces = std::min(
        {sim.skinTriangleCount, triangles.size(), sim.renderFaces.size()});
    for (std::size_t face = 0; face < skinFaces; ++face) {
        pascalOut[face] =
            static_cast<float>(triangles[face].pressureDifference);
    }
}

void nodeStrainFields(const SimBody &sim,
                      bool detailedRibs,
                      std::vector<float> &tensileOut,
                      std::vector<float> &slackOut)
{
    if (!sim.body) {
        tensileOut.clear();
        slackOut.clear();
        return;
    }
    const auto &nodes = sim.body->nodes();
    const auto &constraints = sim.body->constraints();
    // LENGTH-WEIGHTED means per node, not maxima, and tension and
    // compression aggregated separately. The weighting is what makes
    // the number fabric rather than numerics: XPBD leaves an absolute
    // position residual per edge, and a millimetre of residual on a
    // 4 mm vent-rim edge masquerades as 25% strain while the same
    // residual on a 40 mm skin edge is an honest 2.5% — an unweighted
    // max put a 30% "peak" on the legend of a healthy wing. Weighted
    // by rest length, total elongation over total rest, the short-edge
    // noise carries only its own few millimetres of weight. Tension
    // and compression stay separate because a wrinkled panel is BOTH —
    // taut along the load, slack across it — and netting them to zero
    // would hide exactly the state the maps exist to show.
    tensileOut.assign(nodes.size(), 0.0F);
    slackOut.assign(nodes.size(), 0.0F);
    std::vector<float> membraneTensileWeight(nodes.size(), 0.0F);
    std::vector<float> membraneSlackWeight(nodes.size(), 0.0F);
    for (const RenderFace &drawn : sim.renderFaces) {
        if (!isSkinSurface(drawn.surface) || !drawn.membraneElement) {
            continue;
        }
        const std::size_t elementIndex = *drawn.membraneElement;
        if (elementIndex >= sim.body->membraneElements().size()) {
            continue;
        }
        const auto diagnostics = sim.body->membraneDiagnostics(elementIndex);
        const float area = static_cast<float>(
            sim.body->membraneElements()[elementIndex].referenceArea);
        const std::array<float, 2> normalStrain{
            static_cast<float>(diagnostics.greenStrain.x),
            static_cast<float>(diagnostics.greenStrain.y)};
        for (const std::size_t node : drawn.nodes) {
            for (const float strain : normalStrain) {
                if (strain >= 0.0F) {
                    tensileOut[node] += area * strain;
                    membraneTensileWeight[node] += area;
                } else {
                    slackOut[node] += area * strain;
                    membraneSlackWeight[node] += area;
                }
            }
        }
    }
    for (std::size_t node = 0; node < nodes.size(); ++node) {
        if (membraneTensileWeight[node] > 0.0F) {
            tensileOut[node] /= membraneTensileWeight[node];
        }
        if (membraneSlackWeight[node] > 0.0F) {
            slackOut[node] /= membraneSlackWeight[node];
        }
    }
    // First pass: the drawn, deduplicated edge set and its rest
    // lengths, for the mesh-relative floor below. Regions meshed much
    // finer than the wing's norm — the vent rims, the tip web struts —
    // amplify the solver residual into fake tens-of-percent strain
    // over the WHOLE region, so a weighted mean inside such a region
    // is still noise; they have to be excluded outright, and the only
    // floor that survives a resolution change is one relative to the
    // wing's own median edge.
    std::vector<std::size_t> edges;
    std::vector<float> rests;
    std::vector<char> seen(constraints.size(), 0);
    for (const RenderFace &drawn : sim.renderFaces) {
        if (isSkinSurface(drawn.surface) && drawn.membraneElement) {
            continue;
        }
        if (drawn.surface == SimSurface::Rib && !detailedRibs) {
            continue;
        }
        for (const std::size_t edge : drawn.edges) {
            if (edge == noConstraint || edge >= constraints.size()
                || seen[edge] != 0) {
                continue;
            }
            seen[edge] = 1;
            const softwing::DistanceConstraint &tie = constraints[edge];
            if (tie.restLength < kMinimumStrainRestLength
                || tie.a >= nodes.size() || tie.b >= nodes.size()) {
                continue;
            }
            edges.push_back(edge);
            rests.push_back(static_cast<float>(tie.restLength));
        }
    }
    if (edges.empty()) {
        return;
    }
    std::vector<float> sorted = rests;
    std::nth_element(sorted.begin(),
                     sorted.begin()
                         + static_cast<std::ptrdiff_t>(sorted.size() / 2),
                     sorted.end());
    const float medianRest = sorted[sorted.size() / 2];
    const float restFloor =
        std::max(static_cast<float>(kMinimumStrainRestLength),
                 kStrainRestMedianFraction * medianRest);

    std::vector<float> tensileWeight(nodes.size(), 0.0F);
    std::vector<float> slackWeight(nodes.size(), 0.0F);
    for (std::size_t index = 0; index < edges.size(); ++index) {
        const float rest = rests[index];
        if (rest < restFloor) {
            continue;
        }
        const softwing::DistanceConstraint &tie =
            constraints[edges[index]];
        const auto strain = static_cast<float>(
            (length(nodes[tie.b].position - nodes[tie.a].position)
             - tie.restLength)
            / tie.restLength);
        for (const std::size_t node : {tie.a, tie.b}) {
            if (membraneTensileWeight[node] > 0.0F
                || membraneSlackWeight[node] > 0.0F) {
                continue;
            }
            if (strain >= 0.0F) {
                tensileOut[node] += strain * rest;
                tensileWeight[node] += rest;
            } else {
                slackOut[node] += strain * rest;
                slackWeight[node] += rest;
            }
        }
    }
    for (std::size_t node = 0; node < nodes.size(); ++node) {
        if (tensileWeight[node] > 0.0F) {
            tensileOut[node] /= tensileWeight[node];
        }
        if (slackWeight[node] > 0.0F) {
            slackOut[node] /= slackWeight[node];
        }
    }
}

SettleMonitor::SettleMonitor(double maxSeconds) : maxSeconds_(maxSeconds)
{
}

bool SettleMonitor::frameStepped(const SimBody &sim,
                                 double pressurePascal)
{
    seconds_ += simulationTimeStep;
    ++frame_;
    if (frame_ % 15 != 0) {
        return seconds_ >= maxSeconds_;
    }
    agitation_ = agitationOf(sim);
    // A wing that reads quiet in the first moments has not yet answered
    // the tighter solve the settle exists for; give the higher quality
    // two simulated seconds of authority before any verdict.
    if (seconds_ < 2.0) {
        return false;
    }
    if (agitation_ < settleQuiescenceTarget(pressurePascal)) {
        settled_ = true;
        return true;
    }
    constexpr std::size_t probeWindow = 8;
    agitationProbes_.push_back(agitation_);
    forceProbes_.push_back(length(sim.lastAeroForce));
    if (agitationProbes_.size() > probeWindow) {
        agitationProbes_.erase(agitationProbes_.begin());
        forceProbes_.erase(forceProbes_.begin());
    }
    if (agitationProbes_.size() == probeWindow) {
        const auto spreadOf = [](const std::vector<double> &probes) {
            const auto [low, high] =
                std::minmax_element(probes.begin(), probes.end());
            const double mean =
                std::accumulate(probes.begin(), probes.end(), 0.0)
                / static_cast<double>(probes.size());
            return mean > 0.0 ? (*high - *low) / mean : 0.0;
        };
        if (spreadOf(agitationProbes_) < settleStationarySpread
            && spreadOf(forceProbes_) < settleStationaryForceSpread) {
            settled_ = true;
            return true;
        }
    }
    return seconds_ >= maxSeconds_;
}

double settleQuiescenceTarget(double pressurePascal)
{
    const double airspeed =
        std::sqrt(2.0 * std::max(0.0, pressurePascal) / kAirDensity);
    return std::max(settleAgitationFloorMetresPerSecond,
                    settleAgitationAirspeedFraction * airspeed);
}

SettleResult settleAndMeasure(
    SimBody &sim,
    const SimControls &controls,
    const ShapeBaseline &baseline,
    double maxSeconds,
    const std::atomic<bool> *cancelled,
    const std::function<void(double, double)> *progress)
{
    SettleResult result;
    int frame = 0;
    // Soft tunnel start: ramp the dynamic pressure over the first two
    // seconds instead of slamming the full load onto the rest pose. The
    // hard start snapped the canopy onto its one-sided cables, and which
    // equilibrium it bounced into depended on the transient — with heavy
    // damping it parked nose-down on taut A-lines at a tenth of its
    // lift, a real state (a front-tucked wing) but not the one a tunnel
    // measurement is about. Loading up gently at the rigged attitude
    // lands it in the flying equilibrium every time, which is exactly
    // why real tunnels soft-start too.
    constexpr double rampSeconds = 2.0;
    SimControls ramped = controls;
    // Quiescence scales with the tunnel speed: fabric micro-flutter is
    // driven by the airflow, so what counts as "still" at 60 km/h would
    // be a storm at 15.
    const double quiescence =
        settleQuiescenceTarget(controls.pressurePascal);
    // Stationarity window: eight quarter-second probes of agitation and
    // the aerodynamic resultant. Wings differ in how loudly they flutter
    // while perfectly stationary, so "converged" is the honest criterion
    // when "quiet" never comes.
    constexpr std::size_t probeWindow = 8;
    std::vector<double> agitationProbes;
    std::vector<double> forceProbes;
    while (result.simulatedSeconds + 0.5 * simulationTimeStep
           < maxSeconds) {
        if (cancelled != nullptr
            && cancelled->load(std::memory_order_relaxed)) {
            break;
        }
        const double rise =
            std::min(1.0, result.simulatedSeconds / rampSeconds);
        // Smoothstep: no q-rate jump at either end of the ramp.
        ramped.pressurePascal =
            controls.pressurePascal * rise * rise * (3.0 - 2.0 * rise);
        stepSimulation(sim, ramped);
        result.simulatedSeconds += simulationTimeStep;
        ++frame;
        // Quiescence is not a per-frame question; a quarter second
        // between probes keeps the settle loop's cost in the steps.
        if (frame % 15 != 0) {
            continue;
        }
        const double agitation = agitationOf(sim);
        if (progress != nullptr && *progress) {
            (*progress)(result.simulatedSeconds, agitation);
        }
        // The ramp itself is motion, so settling only counts after it —
        // but progress above reports from the first probe, so a watcher
        // sees life immediately.
        if (result.simulatedSeconds <= rampSeconds + 1.0) {
            continue;
        }
        if (agitation < quiescence) {
            result.settled = true;
            break;
        }
        agitationProbes.push_back(agitation);
        forceProbes.push_back(length(sim.lastAeroForce));
        if (agitationProbes.size() > probeWindow) {
            agitationProbes.erase(agitationProbes.begin());
            forceProbes.erase(forceProbes.begin());
        }
        if (agitationProbes.size() == probeWindow) {
            const auto spreadOf = [](const std::vector<double> &probes) {
                const auto [low, high] = std::minmax_element(
                    probes.begin(), probes.end());
                const double mean =
                    std::accumulate(probes.begin(), probes.end(), 0.0)
                    / static_cast<double>(probes.size());
                return mean > 0.0 ? (*high - *low) / mean : 0.0;
            };
            // A standing flutter has steady agitation AND a steady
            // resultant; a pitch oscillation carries its lift with it
            // and fails the force test, which is the discrimination
            // that matters.
            if (spreadOf(agitationProbes) < settleStationarySpread
                && spreadOf(forceProbes) < settleStationaryForceSpread) {
                result.settled = true;
                break;
            }
        }
    }
    // Measured under `ramped`, not `controls`: if the budget (or a
    // cancel) ended inside the soft start, the honest q to echo is the
    // one actually applied, not the nominal one the wing never saw.
    result.report = measureShape(sim, ramped, baseline);
    if (!result.settled) {
        result.report.flags.push_back(
            {ShapeFlag::Unsettled,
             QStringLiteral("agitation %1 mm/s after %2 s")
                 .arg(agitationOf(sim) * 1000.0, 0, 'f', 0)
                 .arg(result.simulatedSeconds, 0, 'f', 1)});
    }
    return result;
}

QString shapeReportCsvHeader()
{
    return QStringLiteral(
        "alpha_deg,q_pa,airspeed_kmh,span_ratio,area_ratio,"
        "volume_ratio,slack_fraction,asymmetry_mm,agitation_mms,"
        "worst_dev_mm,worst_dev_rib,le_dent_mm,le_dent_rib,twist_deg,"
        "twist_rib,line_load_n,slack_risers,lift_n,drag_n,glide,"
        "rowA_left_n,rowA_right_n,rowB_left_n,rowB_right_n,"
        "rowC_left_n,rowC_right_n,rowD_left_n,rowD_right_n,"
        "rowE_left_n,rowE_right_n,rowF_left_n,rowF_right_n,flags");
}

QString shapeReportCsvRow(const ShapeReport &report)
{
    const double q = std::max(0.0, report.dynamicPressurePascal);
    const double airspeedKmh =
        std::sqrt(2.0 * q / kAirDensity) * 3.6;
    QStringList fields;
    fields << QString::number(report.alphaDegrees, 'f', 2)
           << QString::number(report.dynamicPressurePascal, 'f', 1)
           << QString::number(airspeedKmh, 'f', 1)
           << QString::number(report.spanRatio, 'f', 4)
           << QString::number(report.areaRatio, 'f', 4)
           << QString::number(report.volumeRatio, 'f', 4)
           << QString::number(report.slackFraction, 'f', 4)
           << QString::number(report.asymmetryMetres * 1000.0, 'f', 1)
           << QString::number(
                  report.agitationMetresPerSecond * 1000.0, 'f', 1)
           << QString::number(report.worstDeviationMetres * 1000.0,
                              'f', 1)
           << QString::number(static_cast<qulonglong>(
                  report.worstDeviationRib))
           << QString::number(
                  report.worstLeadingEdgeDentMetres * 1000.0, 'f', 1)
           << QString::number(static_cast<qulonglong>(
                  report.worstLeadingEdgeDentRib))
           << QString::number(report.worstTwistDegrees, 'f', 2)
           << QString::number(
                  static_cast<qulonglong>(report.worstTwistRib))
           << QString::number(report.lineLoadNewtons, 'f', 1)
           << QString::number(report.slackRiserSegments)
           << QString::number(report.liftNewtons, 'f', 1)
           << QString::number(report.dragNewtons, 'f', 1)
           << QString::number(report.glideRatio, 'f', 2);
    // Fixed A..F columns whatever rows the wing actually has, so every
    // row of a sweep file has the same shape.
    double rowLoads[6][2] = {};
    for (const RowLoad &row : report.rows) {
        const int index = row.row.toLatin1() - 'A';
        if (index >= 0 && index < 6) {
            rowLoads[index][0] = row.leftNewtons;
            rowLoads[index][1] = row.rightNewtons;
        }
    }
    for (const auto &load : rowLoads) {
        fields << QString::number(load[0], 'f', 1)
               << QString::number(load[1], 'f', 1);
    }
    QStringList flagNames;
    for (const ShapeFlagInfo &info : report.flags) {
        flagNames << shapeFlagName(info.flag);
    }
    fields << flagNames.join(QLatin1Char(';'));
    return fields.join(QLatin1Char(','));
}

}  // namespace lep::playground
