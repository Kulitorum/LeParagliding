#include "playground_sim.h"

#include "playground_pressure_solve.h"

#include <softwing/cell_volume.h>
#include <softwing/parallel.h>

#include <QByteArray>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QLatin1String>
#include <QtGlobal>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <utility>

namespace lep::playground {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kAirDensity = 1.225;   // kg/m^3
constexpr double kMaximumDynamicPressureRatio = 4.0;
// Fixed-temperature, finite-mass cell air. Pneumatics are refreshed between
// the outer XPBD substeps; this inner count only refines each orifice update.
constexpr double kCellFlowDischarge = 0.6;
constexpr double kAtmosphericPressure = 101325.0;
constexpr double kCellAirTemperature =
    softwing::standardFixtureTemperature;
constexpr int kCellAirSubsteps = 1;
constexpr double kMinimumCellVolumeRatio = 0.01;
constexpr double kCellMinimumPressureEnvelopePascal = 80.0;
constexpr double kCellMinimumGaugePressureRatio = 0.0;
constexpr double kCellMaximumGaugePressureRatio = 1.5;
void resetCellAirState(SimBody &sim);
// Bluff-body coefficient on the frontal area a deformed canopy presents
// beyond its designed shape. A flat plate broadside runs 1.1-1.2; 1.0
// allows for the layers a fold stacks in each other's wake, which the
// area sum counts twice and the air does not. On a fully collapsed wing
// this lands the terminal descent at 13-15 m/s, which is where a real
// collapsed canopy falls — free fall, the model's old answer, is 3x that.
constexpr double kFabricDragCoefficient = 1.0;
// Deadband, as a multiple of the rest frontal area. A loaded canopy
// balloons, so its live silhouette runs a little over the drawing's even
// when nothing is wrong — measured at 7-20% on the Swoop — and charging
// that as bluff-body drag cost the trimmed glide a sixth of its L/D.
// Below this the term is exactly zero and the calibration is exactly the
// old one; a fold clears it several times over.
constexpr double kFabricDragOnset = 1.25;
// Ceiling on the wind a section's own rotation may add to or subtract
// from the wing's, as a multiple of the wing's airspeed. At 1.0 a section
// can have its wind doubled or cancelled but never reversed, which keeps
// a tumbling transient from inventing loads nothing in the model bounds.
constexpr double kMaximumSpinWindRatio = 1.0;
// Chord station of the per-rib attitude reference node (see
// RibChord::referenceNode). Must match the station used when the node is
// picked at build time: the measured line is scaled by 1/this so the
// wing-level angle keeps reading in whole-chord terms.
constexpr double kAttitudeReferenceStation = 0.40;
// The pinned measurement path retains its prescribed half-polar brake model.
// Free flight does not use these additions: its live deflected fabric already
// carries the brake and adding this pair again was the reported double count.
constexpr double kTunnelBrakeCamberRadians = 8.0 * kDegreesToRadians;
constexpr double kTunnelBrakeFullPullMetres = 0.35;
constexpr double kTunnelBrakeDragCoefficient = 0.12;
// Zero-lift drag referred to the projected planform: canopy profile drag
// with its cell openings and seams, plus the line cascade. Together with
// the induced term below this puts the polar's best glide around 7–8,
// which is where this class of wing actually flies.
constexpr double kParasiticDragCoefficient = 0.055;
// The pilot and harness as a bluff body: drag area C_D·A in m², applied
// at the pilot node against the pilot's OWN relative wind, not the
// canopy's. The distinction is the pendulum damping: a pilot swinging
// under the wing moves through the air, and the drag of that motion is a
// real part of why the swing dies out.
constexpr double kPilotDragArea = 0.35;
// Oswald efficiency; a paraglider's elliptical-ish planform is decent.
constexpr double kSpanEfficiency = 0.9;
// A fabric wing cannot be flown at real negative lift — pushed from
// above it front-tucks and deflates rather than pulling downward. The
// thin-aerofoil polar does not know that, and its full-authority negative
// lift at a transiently negative angle of attack was what turned an
// aft-swing excursion into a powered dive. Floored just under zero, a
// low-alpha excursion means "no lift, sink, let the wind from below
// restore the angle" — which is a recovery, not a tumble.
constexpr double kMinimumLiftCoefficient = -0.10;
// How the imposed resultant's chordwise anchor travels with angle of
// attack, in chord fractions per radian of deviation from trim. At trim
// the anchor sits exactly on the designed hang line, so there is no
// standing moment — a fixed aft offset was tried and turned out to be a
// constant nose-down torque that wound the wing over in a second. Away
// from trim the anchor moves aft of the hang line for higher angles and
// forward for lower ones, which is a restoring moment in both directions:
// static pitch stability with the trim angle the designer rigged.
constexpr double kAnchorTravelPerRadian = 1.0;
// Pitch-rate term on the same anchor: the resultant shifts aft while the
// angle of attack is still rising, which is the pressure-native form of
// the Cmq damping a real canopy has and this model otherwise lacks. It is
// what keeps the pendulum's swing from pumping the wing into stall. Its
// contribution is clamped: an unbounded rate term slammed the anchor to
// its stops on launch transients and stalled the wing it was meant to
// protect.
constexpr double kAnchorRateSeconds = 0.15;
constexpr double kAnchorRateLimit = 0.08;
// Past the stall the section law's lift dies away, but a fabric wing at a
// silly angle is not force-free — it is a parachute. Flat-plate normal
// force: C_N = kFlatPlateNormal·sin(α), split into lift and drag by the
// angle. This is what turns a stall into a braked, recoverable descent
// instead of an accelerating free fall, and it is why a deep-stalled
// canopy noses back down at all.
constexpr double kFlatPlateNormal = 1.2;
// Fastest a hand moves a brake, metres per second of SIMULATED time (see
// SimBody::brakeApplied). Full travel in a little over half a second: a
// pilot can snatch a brake faster than that, but a pilot flying does
// not, and every input above this rate was an artefact of the controls
// being sampled on the wall clock while the wing runs on its own.
constexpr double kBrakeHandSpeed = 0.6;
// Ceiling on how far one half-span's measured angle of attack may depart
// from the wing's, for the per-half polar. The departure is real and is
// where the wing's roll damping comes from — a rolling wing genuinely has
// one half descending into the air and the other rising out of it — but
// it is read off live fabric, and a tip that has folded can report almost
// anything. Bounded, a fold costs that half its damping; unbounded it
// hands a force with kilonewtons of authority an angle nothing measured.
constexpr double kHalfAlphaLimitRadians = 10.0 * kDegreesToRadians;
// Free travel before a tunnel brake engages (see stepSimulation). Sized
// to cover the trailing edge's excursion across the sweep's attitude
// range; real wings rig 10-20 cm for the same reason.
constexpr double kTunnelBrakeGapMetres = 0.20;
// Absolute damping of the flight-loaded tunnel (see stepSimulation).
constexpr double tunnelDampingPerSecond = 8.0;

// Defined in the aerodynamics section further down; buildSimBody needs it
// for the trimmed-glide launch estimate.
double wingLiftCoefficient(double angleRadians);

// Rodrigues, for tilting the airflow off the rest chord by the angle of
// attack. The axis must be unit length.
softwing::Vec3 rotateAbout(const softwing::Vec3 &value,
                           const softwing::Vec3 &axis,
                           double angleRadians)
{
    const double c = std::cos(angleRadians);
    const double s = std::sin(angleRadians);
    return value * c + cross(axis, value) * s
           + axis * (dot(axis, value) * (1.0 - c));
}

using PlanarPolygon = std::vector<std::pair<double, double>>;

// Crossing-number test; the outlines are closed and non-self-intersecting.
bool insidePolygon(const PlanarPolygon &polygon, double x, double y)
{
    bool inside = false;
    for (std::size_t index = 0, previous = polygon.size() - 1;
         index < polygon.size();
         previous = index++) {
        const auto [xi, yi] = polygon[index];
        const auto [xj, yj] = polygon[previous];
        if ((yi > y) != (yj > y)
            && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
    }
    return inside;
}

// A rib is planar, so its holes and its mesh are worked out in the rib's
// own plane and mapped back.
struct PlanarFrame
{
    softwing::Vec3 origin;
    softwing::Vec3 u;
    softwing::Vec3 v;

    [[nodiscard]] std::pair<double, double> project(
        const softwing::Vec3 &point) const
    {
        const softwing::Vec3 offset = point - origin;
        return {dot(offset, u), dot(offset, v)};
    }
};

// Newell's normal, which is stable for the near-planar many-sided loops a
// rib outline produces, plus an arbitrary in-plane basis.
PlanarFrame fitPlane(const std::vector<softwing::Vec3> &points)
{
    PlanarFrame frame;
    for (const softwing::Vec3 &point : points) {
        frame.origin += point;
    }
    frame.origin /= static_cast<double>(std::max<std::size_t>(
        points.size(), 1));

    softwing::Vec3 normal;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const softwing::Vec3 &current = points[index];
        const softwing::Vec3 &next = points[(index + 1) % points.size()];
        normal.x += (current.y - next.y) * (current.z + next.z);
        normal.y += (current.z - next.z) * (current.x + next.x);
        normal.z += (current.x - next.x) * (current.y + next.y);
    }
    if (length(normal) <= 0.0) {
        normal = {0.0, 0.0, 1.0};
    }
    normal = normalized(normal);

    // Any axis not parallel to the normal seeds the in-plane basis.
    const softwing::Vec3 seed = std::abs(normal.x) < 0.9
                                    ? softwing::Vec3{1.0, 0.0, 0.0}
                                    : softwing::Vec3{0.0, 1.0, 0.0};
    frame.u = normalized(cross(normal, seed));
    frame.v = cross(normal, frame.u);
    return frame;
}

// Welds by position at millimetre resolution: the mesh exporter, the
// refinement below, and the suspension-line junctions all rely on points
// that are meant to coincide landing on one node.
std::uint64_t quantizedKey(const softwing::Vec3 &point)
{
    const auto component = [](double value) {
        return static_cast<std::uint64_t>(
                   static_cast<std::int64_t>(std::llround(value * 1000.0))
                   & 0x1FFFFF);
    };
    return component(point.x) | (component(point.y) << 21)
           | (component(point.z) << 42);
}

double finiteClamped(double value, double fallback, double low, double high)
{
    return std::clamp(std::isfinite(value) ? value : fallback, low, high);
}

softwing::OrthotropicMembraneMaterial prototypeMaterial(
    const SimControls &controls)
{
    softwing::OrthotropicMembraneMaterial material;
    material.warpStiffness = finiteClamped(
        controls.warpStiffness, prototypeWarpStiffness, 1.0, 1.0e7);
    material.weftStiffness = finiteClamped(
        controls.weftStiffness, prototypeWeftStiffness, 1.0, 1.0e7);
    material.shearStiffness = finiteClamped(
        controls.shearStiffness, prototypeShearStiffness, 1.0, 1.0e7);
    const double maximumCoupling =
        0.99 * std::sqrt(material.warpStiffness * material.weftStiffness);
    material.couplingStiffness = finiteClamped(
        controls.couplingStiffness,
        prototypeCouplingStiffness,
        0.0,
        maximumCoupling);
    material.dampingTime = finiteClamped(
        controls.membraneDampingSeconds,
        prototypeMembraneDampingSeconds,
        0.0,
        1.0);
    material.compressionStiffnessRatio = finiteClamped(
        controls.compressionStiffnessRatio,
        prototypeCompressionStiffnessRatio,
        1.0e-4,
        1.0);
    softwing::validateOrthotropicMembraneMaterial(material);
    return material;
}

std::optional<double> restDihedralAngle(const softwing::Vec3 &a,
                                        const softwing::Vec3 &b,
                                        const softwing::Vec3 &c,
                                        const softwing::Vec3 &d)
{
    const softwing::Vec3 edge = b - a;
    const softwing::Vec3 firstArea = cross(edge, c - a);
    const softwing::Vec3 secondArea = cross(d - a, edge);
    const double edgeLength = length(edge);
    const double firstLength = length(firstArea);
    const double secondLength = length(secondArea);
    if (!(edgeLength > 1.0e-12)
        || !(firstLength > 1.0e-12 * edgeLength)
        || !(secondLength > 1.0e-12 * edgeLength)) {
        return std::nullopt;
    }
    const softwing::Vec3 edgeUnit = edge / edgeLength;
    const softwing::Vec3 firstNormal = firstArea / firstLength;
    const softwing::Vec3 secondNormal = secondArea / secondLength;
    return std::atan2(dot(edgeUnit, cross(firstNormal, secondNormal)),
                      std::clamp(dot(firstNormal, secondNormal), -1.0, 1.0));
}

struct SemanticLineKey
{
    std::uint64_t first = 0;
    std::uint64_t second = 0;
    int plan = 0;
    bool brake = false;

    bool operator<(const SemanticLineKey &other) const
    {
        if (first != other.first) {
            return first < other.first;
        }
        if (second != other.second) {
            return second < other.second;
        }
        if (plan != other.plan) {
            return plan < other.plan;
        }
        return brake < other.brake;
    }
};

SemanticLineKey semanticLineKey(const SimLine &line)
{
    std::uint64_t first = quantizedKey(line.a);
    std::uint64_t second = quantizedKey(line.b);
    if (second < first) {
        std::swap(first, second);
    }
    return {first, second, line.plan, line.brake};
}

// Consistently orients the skin triangles (flood fill over shared edges),
// then flips the whole skin outward by signed volume, so a positive
// uniform pressure inflates the wing instead of crushing it.
void orientOutward(const std::vector<softwing::Vec3> &nodes,
                   std::vector<std::array<int, 3>> &triangles)
{
    std::map<std::pair<int, int>, std::vector<int>> edgeFaces;
    for (int face = 0; face < static_cast<int>(triangles.size()); ++face) {
        const auto &tri = triangles[face];
        for (int corner = 0; corner < 3; ++corner) {
            const int a = tri[corner];
            const int b = tri[(corner + 1) % 3];
            edgeFaces[{std::min(a, b), std::max(a, b)}].push_back(face);
        }
    }
    const auto hasDirectedEdge = [&](const std::array<int, 3> &tri,
                                     int from,
                                     int to) {
        for (int corner = 0; corner < 3; ++corner) {
            if (tri[corner] == from && tri[(corner + 1) % 3] == to) {
                return true;
            }
        }
        return false;
    };

    std::vector<char> visited(triangles.size(), 0);
    for (int seed = 0; seed < static_cast<int>(triangles.size()); ++seed) {
        if (visited[seed]) {
            continue;
        }
        std::queue<int> frontier;
        frontier.push(seed);
        visited[seed] = 1;
        while (!frontier.empty()) {
            const int face = frontier.front();
            frontier.pop();
            const auto tri = triangles[face];
            for (int corner = 0; corner < 3; ++corner) {
                const int a = tri[corner];
                const int b = tri[(corner + 1) % 3];
                for (const int neighbour :
                     edgeFaces[{std::min(a, b), std::max(a, b)}]) {
                    if (neighbour == face || visited[neighbour]) {
                        continue;
                    }
                    // A consistently wound neighbour traverses the shared
                    // edge in the opposite direction.
                    if (hasDirectedEdge(triangles[neighbour], a, b)) {
                        std::swap(triangles[neighbour][1],
                                  triangles[neighbour][2]);
                    }
                    visited[neighbour] = 1;
                    frontier.push(neighbour);
                }
            }
        }
    }

    double signedVolume = 0.0;
    for (const auto &tri : triangles) {
        signedVolume += dot(nodes[tri[0]],
                            cross(nodes[tri[1]], nodes[tri[2]]))
                        / 6.0;
    }
    if (signedVolume < 0.0) {
        for (auto &tri : triangles) {
            std::swap(tri[1], tri[2]);
        }
    }
}

}  // namespace

const char *launchModeName(LaunchMode mode)
{
    switch (mode) {
    case LaunchMode::TrimmedGlide:
        return "trimmed glide";
    case LaunchMode::DropFromRest:
        return "drop from rest";
    }
    return "unknown";
}

const char *skinModelName(SkinModel model)
{
    switch (model) {
    case SkinModel::LegacyDistanceTruss:
        return "legacy distance truss";
    case SkinModel::OrthotropicMembrane:
        return "orthotropic membrane prototype";
    }
    return "unknown";
}

std::optional<SimMesh> parseSimMesh(const QByteArray &data, QString &error)
{
    QJsonParseError parseError{};
    const QJsonDocument document =
        QJsonDocument::fromJson(data, &parseError);
    if (document.isNull() || !document.isObject()) {
        error = QStringLiteral("Not a simulation mesh: %1")
                    .arg(parseError.errorString());
        return std::nullopt;
    }
    const QJsonObject root = document.object();

    SimMesh mesh;
    const auto vec = [](const QJsonArray &array) {
        return softwing::Vec3{array.at(0).toDouble() * metresPerMillimetre,
                              array.at(1).toDouble() * metresPerMillimetre,
                              array.at(2).toDouble() * metresPerMillimetre};
    };
    for (const QJsonValue &value : root.value(QLatin1String("nodes")).toArray()) {
        mesh.nodes.push_back(vec(value.toArray()));
    }
    for (const QJsonValue &value : root.value(QLatin1String("quads")).toArray()) {
        const QJsonArray quad = value.toArray();
        mesh.quads.push_back({quad.at(0).toInt(),
                              quad.at(1).toInt(),
                              quad.at(2).toInt(),
                              quad.at(3).toInt()});
    }
    for (const QJsonValue &value :
         root.value(QLatin1String("quadSurfaces")).toArray()) {
        const int tag = value.toInt();
        mesh.quadSurfaces.push_back(
            tag >= 0 && tag < simExportedSurfaceCount
                ? static_cast<SimSurface>(tag)
                : SimSurface::Extrados);
    }
    // Older meshes carry no tags; treating the whole skin as one surface
    // keeps them loadable, just without the per-surface toggles.
    mesh.quadSurfaces.resize(mesh.quads.size(), SimSurface::Extrados);
    for (const QJsonValue &value :
         root.value(QLatin1String("ribLoops")).toArray()) {
        std::vector<int> loop;
        for (const QJsonValue &node : value.toArray()) {
            loop.push_back(node.toInt());
        }
        mesh.ribLoops.push_back(std::move(loop));
    }
    for (const QJsonValue &value :
         root.value(QLatin1String("ribHoles")).toArray()) {
        std::vector<std::vector<softwing::Vec3>> outlines;
        for (const QJsonValue &outline : value.toArray()) {
            std::vector<softwing::Vec3> points;
            for (const QJsonValue &point : outline.toArray()) {
                points.push_back(vec(point.toArray()));
            }
            if (points.size() >= 3) {
                outlines.push_back(std::move(points));
            }
        }
        mesh.ribHoles.push_back(std::move(outlines));
    }
    // Meshes written before holes were exported simply have none.
    mesh.ribHoles.resize(mesh.ribLoops.size());
    for (const QJsonValue &value :
         root.value(QLatin1String("straps")).toArray()) {
        const QJsonObject strapObject = value.toObject();
        SimStrap strap;
        for (const QJsonValue &point :
             strapObject.value(QLatin1String("a")).toArray()) {
            strap.a.push_back(vec(point.toArray()));
        }
        for (const QJsonValue &point :
             strapObject.value(QLatin1String("b")).toArray()) {
            strap.b.push_back(vec(point.toArray()));
        }
        if (strap.a.size() == strap.b.size() && !strap.a.empty()) {
            mesh.straps.push_back(std::move(strap));
        }
    }
    std::set<SemanticLineKey> lineKeys;
    for (const QJsonValue &value : root.value(QLatin1String("lines")).toArray()) {
        const QJsonObject line = value.toObject();
        // Row plan (1..6 = A..F) is a late addition; meshes written before
        // it default to 0, which the row-load instrumentation treats as
        // "unknown" rather than a row of its own.
        SimLine parsed{
            vec(line.value(QLatin1String("a")).toArray()),
            vec(line.value(QLatin1String("b")).toArray()),
            line.value(QLatin1String("brake")).toInt() != 0,
            line.value(QLatin1String("plan")).toInt(0),
        };
        if (!lineKeys.insert(semanticLineKey(parsed)).second) {
            ++mesh.duplicateLineCount;
            continue;
        }
        mesh.lines.push_back(std::move(parsed));
    }

    const int nodeCount = static_cast<int>(mesh.nodes.size());
    const auto inRange = [nodeCount](int index) {
        return index >= 0 && index < nodeCount;
    };
    for (const auto &quad : mesh.quads) {
        if (!std::all_of(quad.begin(), quad.end(), inRange)) {
            error = QStringLiteral("Mesh references nodes out of range");
            return std::nullopt;
        }
    }
    for (const auto &loop : mesh.ribLoops) {
        if (!std::all_of(loop.begin(), loop.end(), inRange)) {
            error = QStringLiteral("Rib loop references nodes out of range");
            return std::nullopt;
        }
    }
    if (mesh.nodes.size() < 4 || mesh.quads.empty()) {
        error = QStringLiteral("Simulation mesh is empty");
        return std::nullopt;
    }
    return mesh;
}

// Sub-quad corners are welded by quantized position rather than by index
// arithmetic: two quads sharing an edge parameterize it in opposite
// directions, and j/factor versus (factor-j)/factor are not bit-identical,
// so only position welding keeps the refined skin a closed surface. The
// pressure field depends on that closure.
//
// Straps and lines are stored as positions and bind to the skin by
// proximity when the body is assembled, so they carry over untouched and
// simply find the nearer refined nodes.
SimMesh refineSimMesh(const SimMesh &mesh, int factor)
{
    if (factor <= 1) {
        return mesh;
    }

    SimMesh refined;
    refined.straps = mesh.straps;
    refined.lines = mesh.lines;
    refined.nodes.reserve(mesh.nodes.size()
                          * static_cast<std::size_t>(factor) * factor);
    refined.quads.reserve(mesh.quads.size()
                          * static_cast<std::size_t>(factor) * factor);
    refined.quadSurfaces.reserve(refined.quads.capacity());

    std::map<std::uint64_t, int> welded;
    const auto nodeAt = [&](const softwing::Vec3 &point) {
        const auto [entry, inserted] =
            welded.try_emplace(quantizedKey(point), 0);
        if (inserted) {
            entry->second = static_cast<int>(refined.nodes.size());
            refined.nodes.push_back(point);
        }
        return entry->second;
    };

    const double span = static_cast<double>(factor);
    for (std::size_t quadIndex = 0; quadIndex < mesh.quads.size();
         ++quadIndex) {
        const auto &quad = mesh.quads[quadIndex];
        const softwing::Vec3 &corner0 =
            mesh.nodes[static_cast<std::size_t>(quad[0])];
        const softwing::Vec3 &corner1 =
            mesh.nodes[static_cast<std::size_t>(quad[1])];
        const softwing::Vec3 &corner2 =
            mesh.nodes[static_cast<std::size_t>(quad[2])];
        const softwing::Vec3 &corner3 =
            mesh.nodes[static_cast<std::size_t>(quad[3])];

        // Grid of (factor + 1)^2 corners; u runs 0->1, v runs 0->3.
        std::vector<int> grid(static_cast<std::size_t>(factor + 1)
                              * (factor + 1));
        for (int v = 0; v <= factor; ++v) {
            const double t = v / span;
            for (int u = 0; u <= factor; ++u) {
                const double s = u / span;
                const softwing::Vec3 front =
                    corner0 * (1.0 - s) + corner1 * s;
                const softwing::Vec3 back =
                    corner3 * (1.0 - s) + corner2 * s;
                grid[static_cast<std::size_t>(v) * (factor + 1) + u] =
                    nodeAt(front * (1.0 - t) + back * t);
            }
        }
        for (int v = 0; v < factor; ++v) {
            for (int u = 0; u < factor; ++u) {
                const auto at = [&](int row, int column) {
                    return grid[static_cast<std::size_t>(row) * (factor + 1)
                                + column];
                };
                const std::array<int, 4> cell{at(v, u),
                                              at(v, u + 1),
                                              at(v + 1, u + 1),
                                              at(v + 1, u)};
                // A degenerate source quad (collapsed trailing edge) can
                // weld a whole sub-quad onto one or two nodes; those carry
                // no area and would only feed zero-length constraints.
                if (cell[0] != cell[1] && cell[1] != cell[2]
                    && cell[2] != cell[3] && cell[3] != cell[0]) {
                    refined.quads.push_back(cell);
                    refined.quadSurfaces.push_back(
                        mesh.quadSurfaces[quadIndex]);
                }
            }
        }
    }

    // Rib loops run along quad edges, so their refined points land on the
    // sub-quad corners already welded above and reuse those nodes.
    refined.ribLoops.reserve(mesh.ribLoops.size());
    refined.ribHoles.reserve(mesh.ribHoles.size());
    for (std::size_t loopIndex = 0; loopIndex < mesh.ribLoops.size();
         ++loopIndex) {
        const auto &loop = mesh.ribLoops[loopIndex];
        std::vector<int> refinedLoop;
        refinedLoop.reserve(loop.size() * static_cast<std::size_t>(factor));
        for (std::size_t index = 0; index < loop.size(); ++index) {
            const softwing::Vec3 &from =
                mesh.nodes[static_cast<std::size_t>(loop[index])];
            const softwing::Vec3 &to =
                mesh.nodes[static_cast<std::size_t>(
                    loop[(index + 1) % loop.size()])];
            for (int step = 0; step < factor; ++step) {
                const double t = step / span;
                const int node = nodeAt(from * (1.0 - t) + to * t);
                if (refinedLoop.empty() || refinedLoop.back() != node) {
                    refinedLoop.push_back(node);
                }
            }
        }
        if (refinedLoop.size() >= 3 && refinedLoop.front() == refinedLoop.back()) {
            refinedLoop.pop_back();
        }
        if (refinedLoop.size() >= 3) {
            refined.ribLoops.push_back(std::move(refinedLoop));
            refined.ribHoles.push_back(mesh.ribHoles[loopIndex]);
        }
    }

    return refined;
}

// Defined next to the step adapter below. Build-time preparation records only
// skin topology and authored suspension attachments; geometric proximity in
// the designed pose is deliberately not an exclusion.
void prepareContact(SimBody &sim);

SimBody buildSimBody(const SimMesh &mesh,
                     const SimBuildOptions &options,
                     const SimControls &controls)
{
    SimBody sim;
    sim.skinModel = controls.skinModel;
    sim.skinMaterial = prototypeMaterial(controls);
    sim.skinBendCompliance = finiteClamped(
        controls.bendCompliance, prototypeBendCompliance, 0.0, 1.0);
    sim.duplicateLineCount = mesh.duplicateLineCount;
    auto body = std::make_unique<softwing::SoftBody>();
    const int ribLayers = std::max(1, options.ribLayers);
    const int ribStationSplit = std::max(1, options.ribStationSplit);

    // Skin triangles, oriented outward for the pressure field. The
    // surface tag is recorded per triangle in the same order, so the
    // renderer can drop whole skins without disturbing the solver.
    std::vector<std::array<int, 3>> triangles;
    std::vector<SimSurface> triangleSurfaces;
    triangles.reserve(mesh.quads.size() * 2);
    triangleSurfaces.reserve(mesh.quads.size() * 2);
    for (std::size_t quadIndex = 0; quadIndex < mesh.quads.size();
         ++quadIndex) {
        const auto &quad = mesh.quads[quadIndex];
        triangles.push_back({quad[0], quad[1], quad[2]});
        triangles.push_back({quad[0], quad[2], quad[3]});
        triangleSurfaces.push_back(mesh.quadSurfaces[quadIndex]);
        triangleSurfaces.push_back(mesh.quadSurfaces[quadIndex]);
    }
    orientOutward(mesh.nodes, triangles);

    // Area-lumped node masses.
    std::vector<double> masses(mesh.nodes.size(), 0.0);
    for (const auto &tri : triangles) {
        const double area =
            0.5
            * length(cross(mesh.nodes[tri[1]] - mesh.nodes[tri[0]],
                           mesh.nodes[tri[2]] - mesh.nodes[tri[0]]));
        for (const int node : tri) {
            masses[static_cast<std::size_t>(node)] +=
                fabricArealDensity * area / 3.0;
        }
    }
    for (std::size_t index = 0; index < mesh.nodes.size(); ++index) {
        body->addNode(mesh.nodes[index], std::max(masses[index], 5.0e-4));
    }
    for (const auto &tri : triangles) {
        body->addTriangle(static_cast<std::size_t>(tri[0]),
                          static_cast<std::size_t>(tri[1]),
                          static_cast<std::size_t>(tri[2]));
    }
    sim.skinTriangleCount = triangles.size();

    // Optional material skin. Each ordered source quad carries one neutral
    // chart orientation: q0->q3 is chord/warp and q0->q1 is span/weft (the
    // same u/v convention refineSimMesh documents). The two triangles may be
    // mildly non-planar, so each receives an isometric chart in its own rest
    // tangent plane while sharing the quad's transported warp direction.
    // This keeps the designed pose exactly strain-free instead of flattening
    // a curved quad and preloading it before the first step.
    std::vector<std::size_t> membraneForTriangle(
        triangles.size(), noConstraint);
    if (sim.skinModel == SkinModel::OrthotropicMembrane) {
        std::vector<softwing::MembraneElementDefinition> definitions;
        std::vector<std::size_t> definitionFaces;
        definitions.reserve(triangles.size());
        definitionFaces.reserve(triangles.size());
        for (std::size_t face = 0; face < triangles.size(); ++face) {
            const auto &quad = mesh.quads[face / 2];
            const softwing::Vec3 &q0 =
                mesh.nodes[static_cast<std::size_t>(quad[0])];
            const softwing::Vec3 &q1 =
                mesh.nodes[static_cast<std::size_t>(quad[1])];
            const softwing::Vec3 &q2 =
                mesh.nodes[static_cast<std::size_t>(quad[2])];
            const softwing::Vec3 &q3 =
                mesh.nodes[static_cast<std::size_t>(quad[3])];
            const softwing::Vec3 warpHint = (q3 - q0) + (q2 - q1);
            const softwing::Vec3 weftHint = (q1 - q0) + (q2 - q3);
            const auto &tri = triangles[face];
            const softwing::Vec3 &p0 =
                mesh.nodes[static_cast<std::size_t>(tri[0])];
            const softwing::Vec3 &p1 =
                mesh.nodes[static_cast<std::size_t>(tri[1])];
            const softwing::Vec3 &p2 =
                mesh.nodes[static_cast<std::size_t>(tri[2])];
            const softwing::Vec3 normal = cross(p1 - p0, p2 - p0);
            const double normalSquared = lengthSquared(normal);
            if (!(normalSquared > 1.0e-24)
                || !(lengthSquared(warpHint) > 1.0e-24)) {
                ++sim.skippedMembraneElements;
                continue;
            }
            const softwing::Vec3 normalUnit =
                normal / std::sqrt(normalSquared);
            softwing::Vec3 warp =
                warpHint - dot(warpHint, normalUnit) * normalUnit;
            if (!(lengthSquared(warp) > 1.0e-24)) {
                warp = cross(weftHint, normalUnit);
            }
            if (!(lengthSquared(warp) > 1.0e-24)) {
                ++sim.skippedMembraneElements;
                continue;
            }
            warp = normalized(warp);
            if (dot(warp, warpHint) < 0.0) {
                warp *= -1.0;
            }
            const softwing::Vec3 weft = cross(normalUnit, warp);
            std::array<softwing::Vec2, 3> chart;
            const std::array<softwing::Vec3, 3> points{p0, p1, p2};
            for (std::size_t corner = 0; corner < points.size(); ++corner) {
                const softwing::Vec3 offset = points[corner] - p0;
                chart[corner] = {dot(offset, warp), dot(offset, weft)};
            }
            const softwing::Vec2 edgeOne = chart[1] - chart[0];
            const softwing::Vec2 edgeTwo = chart[2] - chart[0];
            const double chartArea = softwing::cross(edgeOne, edgeTwo);
            const double chartScale = std::max(
                softwing::lengthSquared(edgeOne),
                softwing::lengthSquared(edgeTwo));
            if (!(chartArea > 64.0 * std::numeric_limits<double>::epsilon()
                                  * chartScale)) {
                ++sim.skippedMembraneElements;
                continue;
            }
            definitions.push_back(
                {face, chart, sim.skinMaterial, softwing::MaterialRole::Bulk});
            definitionFaces.push_back(face);
        }
        if (!definitions.empty()) {
            const std::size_t firstElement = body->membraneElements().size();
            static_cast<void>(body->addMembraneElements(definitions));
            for (std::size_t index = 0; index < definitionFaces.size();
                 ++index) {
                membraneForTriangle[definitionFaces[index]] =
                    firstElement + index;
            }
        }

        struct EdgeUse {
            std::size_t from = 0;
            std::size_t to = 0;
            std::size_t opposite = 0;
            std::size_t face = 0;
        };
        std::map<std::pair<std::size_t, std::size_t>,
                 std::vector<EdgeUse>> skinEdges;
        for (std::size_t face = 0; face < triangles.size(); ++face) {
            const auto &tri = triangles[face];
            for (int edge = 0; edge < 3; ++edge) {
                const std::size_t from = static_cast<std::size_t>(tri[edge]);
                const std::size_t to =
                    static_cast<std::size_t>(tri[(edge + 1) % 3]);
                const std::size_t opposite =
                    static_cast<std::size_t>(tri[(edge + 2) % 3]);
                const auto key = std::minmax(from, to);
                skinEdges[key].push_back({from, to, opposite, face});
            }
        }
        // Decide only after the complete incidence is known. Adding on the
        // second use and discovering a third later would leave a hinge on a
        // non-manifold edge.
        for (const auto &[key, uses] : skinEdges) {
            Q_UNUSED(key);
            if (uses.size() == 1) {
                continue; // genuine skin boundary/intake opening
            }
            if (uses.size() != 2) {
                ++sim.skippedDihedralHinges;
                continue;
            }
            const EdgeUse &first = uses[0];
            const EdgeUse &second = uses[1];
            if (membraneForTriangle[first.face] == noConstraint
                || membraneForTriangle[second.face] == noConstraint
                || first.from != second.to || first.to != second.from) {
                ++sim.skippedDihedralHinges;
                continue;
            }
            const std::optional<double> rest = restDihedralAngle(
                mesh.nodes[first.from],
                mesh.nodes[first.to],
                mesh.nodes[first.opposite],
                mesh.nodes[second.opposite]);
            if (!rest) {
                ++sim.skippedDihedralHinges;
                continue;
            }
            body->addDihedralBendingConstraint(
                first.from,
                first.to,
                first.opposite,
                second.opposite,
                *rest,
                sim.skinBendCompliance);
        }
    }

    // The designed skin as area vectors, kept for the fabric-drag
    // reference: half the sum of |A.w| over a closed surface is its
    // frontal area along w, so these give the frontal area the wing WOULD
    // have if it still had its designed shape, for any wind direction.
    // The live wing's excess over that is the bluff-body area its
    // deformation has created — which is the whole of the term.
    sim.restFaceAreas.clear();
    sim.restFaceAreas.reserve(triangles.size());
    for (const auto &tri : triangles) {
        const softwing::Vec3 &a = mesh.nodes[static_cast<std::size_t>(tri[0])];
        const softwing::Vec3 &b = mesh.nodes[static_cast<std::size_t>(tri[1])];
        const softwing::Vec3 &c = mesh.nodes[static_cast<std::size_t>(tri[2])];
        sim.restFaceAreas.push_back(0.5 * cross(b - a, c - a));
    }

    // Upward-facing faces in the rest pose form the "top surface":
    // fake lift is applied there as extra outward pressure, mimicking
    // upper-surface suction. The cosine falloff toward the tips comes
    // free from the orientation test.
    for (std::size_t face = 0; face < triangles.size(); ++face) {
        const auto &tri = triangles[face];
        const softwing::Vec3 normal =
            cross(mesh.nodes[static_cast<std::size_t>(tri[1])]
                      - mesh.nodes[static_cast<std::size_t>(tri[0])],
                  mesh.nodes[static_cast<std::size_t>(tri[2])]
                      - mesh.nodes[static_cast<std::size_t>(tri[0])]);
        if (normal.z > 0.0) {
            sim.topFaces.push_back(face);
        }
    }

    // Stretch constraints on every unique edge, plus the second quad
    // diagonal for shear.
    // Node pair -> constraint index, so any drawn face can report the
    // stretch of its sides when the view colours by stress. Rib spokes
    // and strap ties register here too, not just skin edges.
    std::map<std::pair<std::size_t, std::size_t>, std::size_t>
        edgeConstraints;
    const auto constraintKey = [](std::size_t a, std::size_t b) {
        return std::pair<std::size_t, std::size_t>{std::min(a, b),
                                                   std::max(a, b)};
    };
    const auto sideConstraint =
        [&](std::size_t a, std::size_t b) -> std::size_t {
        const auto found = edgeConstraints.find(constraintKey(a, b));
        return found == edgeConstraints.end() ? noConstraint
                                              : found->second;
    };
    // Adds a constraint between two body nodes unless the pair is
    // already tied, and remembers which constraint it is.
    const auto tie = [&](std::size_t a,
                         std::size_t b,
                         double restLength,
                         double compliance) {
        if (a == b || edgeConstraints.count(constraintKey(a, b)) != 0) {
            return;
        }
        edgeConstraints.emplace(
            constraintKey(a, b),
            body->addDistanceConstraint(a, b, restLength, compliance));
    };
    const auto suspensionTie = [&](std::size_t a,
                                   std::size_t b,
                                   double restLength,
                                   double compliance) {
        if (a == b || edgeConstraints.count(constraintKey(a, b)) != 0) {
            return;
        }
        edgeConstraints.emplace(
            constraintKey(a, b),
            body->addSuspensionTieConstraint(
                a, b, restLength, compliance));
    };
    const auto addEdge = [&](int a, int b) {
        if (a == b) {
            return;
        }
        tie(static_cast<std::size_t>(a),
            static_cast<std::size_t>(b),
            length(mesh.nodes[static_cast<std::size_t>(b)]
                   - mesh.nodes[static_cast<std::size_t>(a)]),
            skinCompliance);
    };
    if (sim.skinModel == SkinModel::LegacyDistanceTruss) {
        for (const auto &tri : triangles) {
            addEdge(tri[0], tri[1]);
            addEdge(tri[1], tri[2]);
            addEdge(tri[2], tri[0]);
        }
        for (const auto &quad : mesh.quads) {
            addEdge(quad[1], quad[3]);
        }
    }

    // Register the skin faces for drawing. Rib webs and V/H sheets are
    // appended further down, once their nodes exist.
    sim.renderFaces.reserve(triangles.size());
    for (std::size_t face = 0; face < triangles.size(); ++face) {
        const auto &tri = triangles[face];
        RenderFace drawn;
        drawn.surface = triangleSurfaces[face];
        for (int corner = 0; corner < 3; ++corner) {
            drawn.nodes[static_cast<std::size_t>(corner)] =
                static_cast<std::size_t>(tri[corner]);
        }
        for (int corner = 0; corner < 3; ++corner) {
            drawn.edges[static_cast<std::size_t>(corner)] =
                sideConstraint(
                    static_cast<std::size_t>(tri[corner]),
                    static_cast<std::size_t>(tri[(corner + 1) % 3]));
        }
        if (face < membraneForTriangle.size()
            && membraneForTriangle[face] != noConstraint) {
            drawn.membraneElement = membraneForTriangle[face];
        }
        sim.renderFaces.push_back(drawn);
    }

    // Which mesh rib loop each recorded RibChord came from, so the cell
    // construction below can look up that rib's hole outlines. Parallel to
    // ribChords (degenerate loops are skipped from both).
    std::vector<std::size_t> ribMeshIndex;

    // Rib webs. Both models are the same cross-section ladder; the simple
    // one is that ladder at its coarsest — one bay deep, one station per
    // outline segment, no holes.
    //
    // It used to be a centroid hub with a spoke to every loop node, which
    // was cheap to write and expensive in every other way. One node ended up
    // carrying a hundred-odd constraints, and since constraints meeting at a
    // node cannot be solved in parallel, that one hub forced a hundred
    // colours: on gnuC2, 29 of the solver's 38 colours were hub spokes
    // holding a few dozen constraints each. Those tiny colours cost a full
    // barrier apiece on the CPU and a full dispatch apiece on the GPU while
    // doing almost no work. The ladder spreads the same job over low-degree
    // nodes, and it is the better model besides — a rib's job is to hold the
    // two skins apart, which is what a strut across the section does and
    // what a spoke to the middle only approximates.
    for (std::size_t ribIndex = 0; ribIndex < mesh.ribLoops.size();
         ++ribIndex) {
        const auto &loop = mesh.ribLoops[ribIndex];
        const int layers = options.detailedRibs ? ribLayers : 1;
        const int stationSplit =
            options.detailedRibs ? ribStationSplit : 1;

        // The loop perimeter is skin either way.
        for (std::size_t index = 0; index < loop.size(); ++index) {
            addEdge(loop[index], loop[(index + 1) % loop.size()]);
        }

        // A ladder from the upper surface to the lower one, every strut
        // running straight across the section. Rings were tried and are
        // wrong here: routing upper-to-lower the long way round leaves the
        // rib slack, and a slack planar truss has nothing resisting
        // out-of-plane folding, so it crumples. Under tension a ladder
        // stays taut and flat.
        std::vector<softwing::Vec3> loopPoints;
        loopPoints.reserve(loop.size());
        for (const int node : loop) {
            loopPoints.push_back(mesh.nodes[static_cast<std::size_t>(node)]);
        }
        const PlanarFrame frame = fitPlane(loopPoints);
        // Holes are a detailed-model feature. A one-bay ladder tests each
        // cell by its middle, and at one bay deep that middle is the centre
        // of the section — which is exactly where an airfoil hole is, so
        // honouring holes here would delete most of the struts and let the
        // rib fold up.
        std::vector<PlanarPolygon> holes;
        if (options.detailedRibs) {
            for (const auto &outline : mesh.ribHoles[ribIndex]) {
                PlanarPolygon polygon;
                polygon.reserve(outline.size());
                for (const softwing::Vec3 &point : outline) {
                    polygon.push_back(frame.project(point));
                }
                holes.push_back(std::move(polygon));
            }
        }
        const auto inHole = [&holes](const softwing::Vec3 &point,
                                     const PlanarFrame &plane) {
            const auto [x, y] = plane.project(point);
            return std::any_of(holes.begin(),
                               holes.end(),
                               [x, y](const PlanarPolygon &polygon) {
                                   return insidePolygon(polygon, x, y);
                               });
        };

        // Rib fabric weighed by its own area, shared over the interior
        // nodes; the loop nodes already carry their skin mass.
        double area = 0.0;
        for (std::size_t index = 0; index < loopPoints.size(); ++index) {
            const auto [x0, y0] = frame.project(loopPoints[index]);
            const auto [x1, y1] = frame.project(
                loopPoints[(index + 1) % loopPoints.size()]);
            area += x0 * y1 - x1 * y0;
        }
        area = std::abs(area) * 0.5;
        const std::size_t interiorCount =
            loop.size() * static_cast<std::size_t>(layers) / 2 + 1;
        const double interiorMass =
            std::max(fabricArealDensity * area
                         / static_cast<double>(interiorCount),
                     1.0e-4);

        // Chord axis: the two outline points furthest apart are the
        // leading and trailing edge, and they split the outline into
        // its upper and lower surfaces.
        std::vector<std::pair<double, double>> flat;
        flat.reserve(loopPoints.size());
        for (const softwing::Vec3 &point : loopPoints) {
            flat.push_back(frame.project(point));
        }
        std::size_t front = 0;
        std::size_t back = 0;
        double longest = -1.0;
        for (std::size_t a = 0; a < flat.size(); ++a) {
            for (std::size_t b = a + 1; b < flat.size(); ++b) {
                const double dx = flat[a].first - flat[b].first;
                const double dy = flat[a].second - flat[b].second;
                const double distance = dx * dx + dy * dy;
                if (distance > longest) {
                    longest = distance;
                    front = a;
                    back = b;
                }
            }
        }
        if (longest <= 0.0) {
            continue;
        }

        // Record the section's chord for the load model. The two furthest
        // outline points are the leading and trailing edge; which is which
        // comes from the mesh's own convention, where the chord runs along
        // +y from the leading edge (the vents sit at the low-y end of every
        // section). The rib plane normal is the local span direction.
        {
            RibChord chord;
            const auto nodeA = static_cast<std::size_t>(loop[front]);
            const auto nodeB = static_cast<std::size_t>(loop[back]);
            const bool aIsLeading =
                mesh.nodes[nodeA].y <= mesh.nodes[nodeB].y;
            chord.leadingNode = aIsLeading ? nodeA : nodeB;
            chord.trailingNode = aIsLeading ? nodeB : nodeA;
            // Aligned to +x for every rib. The sign of this axis is the sign
            // of the section's angle of attack, so a rib whose plane normal
            // happened to come out reversed would fly upside down while its
            // neighbours flew right way up.
            const softwing::Vec3 planeNormal =
                normalized(cross(frame.u, frame.v));
            chord.spanAxis =
                planeNormal.x < 0.0 ? -1.0 * planeNormal : planeNormal;
            chord.restChordLength = length(mesh.nodes[chord.trailingNode]
                                           - mesh.nodes[chord.leadingNode]);

            // The attitude reference: the outline node nearest 40% chord,
            // on the side the leading-edge-to-trailing-edge line puts the
            // extrados. Pulling a brake rotates the LE->TE line, and
            // measuring the wing's angle of attack off that line makes a
            // brake pull read as the whole wing pitching up — which the
            // polar answers with more lift, more induced drag, less speed,
            // and therefore MORE angle. That loop is what stalled the wing
            // a few seconds after a 20 cm pull (session log, 2026-07-31:
            // alpha kept climbing 20.9 -> 23.1 -> 29.4 -> 76 with the hand
            // held still). Forward of the flap the fabric cannot be moved
            // by the brake, so LE->this node carries the wing's real
            // attitude and nothing of the pilot's input.
            const softwing::Vec3 leading = mesh.nodes[chord.leadingNode];
            const softwing::Vec3 chordVector =
                mesh.nodes[chord.trailingNode] - leading;
            const double chordSquared = lengthSquared(chordVector);
            if (chordSquared > 0.0) {
                const softwing::Vec3 up =
                    normalized(cross(chord.spanAxis, chordVector));
                // The same station sampleWingAero scales the measured
                // line back by; one constant, so the two cannot drift.
                constexpr double kReferenceStation =
                    kAttitudeReferenceStation;
                double best = std::numeric_limits<double>::max();
                std::size_t bestNode = chord.leadingNode;
                for (const int index : loop) {
                    const auto node = static_cast<std::size_t>(index);
                    const softwing::Vec3 offset = mesh.nodes[node] - leading;
                    const double station =
                        dot(offset, chordVector) / chordSquared;
                    if (dot(offset, up) < 0.0) {
                        continue;
                    }
                    const double distance =
                        std::abs(station - kReferenceStation);
                    if (distance < best) {
                        best = distance;
                        bestNode = node;
                    }
                }
                chord.referenceNode = bestNode;
                const softwing::Vec3 attitude =
                    mesh.nodes[chord.referenceNode] - leading;
                const softwing::Vec3 chordFlat = normalized(
                    chordVector
                    - dot(chordVector, chord.spanAxis) * chord.spanAxis);
                const softwing::Vec3 attitudeFlat = normalized(
                    attitude
                    - dot(attitude, chord.spanAxis) * chord.spanAxis);
                if (length(chordFlat) > 0.0
                    && length(attitudeFlat) > 0.0) {
                    chord.attitudeOffsetRadians = std::atan2(
                        dot(cross(attitudeFlat, chordFlat), chord.spanAxis),
                        dot(attitudeFlat, chordFlat));
                }
            } else {
                chord.referenceNode = chord.leadingNode;
            }
            sim.ribChords.push_back(chord);
            ribMeshIndex.push_back(ribIndex);
            // The shape instrumentation fits each rest section onto the
            // live one through this loop, indexed parallel to ribChords —
            // so it is recorded in the exact scope that records the chord.
            // A degenerate loop that bailed earlier skips both.
            std::vector<std::size_t> loopNodes;
            loopNodes.reserve(loop.size());
            for (const int node : loop) {
                loopNodes.push_back(static_cast<std::size_t>(node));
            }
            sim.ribLoopNodes.push_back(std::move(loopNodes));
        }

        const double axisX =
            (flat[back].first - flat[front].first) / std::sqrt(longest);
        const double axisY =
            (flat[back].second - flat[front].second) / std::sqrt(longest);
        // Distance along the chord, measured from the leading edge.
        const auto chordAt = [&](std::size_t index) {
            return (flat[index].first - flat[front].first) * axisX
                   + (flat[index].second - flat[front].second) * axisY;
        };

        // Walking the closed outline from the leading edge to the
        // trailing edge covers one surface; continuing covers the other.
        std::vector<std::size_t> upper;
        std::vector<std::size_t> lower;
        for (std::size_t step = 0; step <= flat.size(); ++step) {
            const std::size_t index = (front + step) % flat.size();
            (upper.empty() || upper.back() != back ? upper : lower)
                .push_back(index);
            if (index == front && step > 0) {
                break;
            }
        }
        // The split leaves the trailing edge as the last upper node; the
        // lower surface has to start there too or its aftmost segment
        // is missing and struts near the trailing edge find no foot.
        lower.insert(lower.begin(), back);
        if (upper.size() < 2 || lower.size() < 2) {
            continue;
        }

        // Where a chord station meets one of the two surfaces, as an
        // interpolation between two outline nodes so the strut end is
        // carried by the skin whether or not it lands on a node.
        struct SurfacePoint
        {
            std::size_t a = 0;
            std::size_t b = 0;
            double blend = 0.0;
        };
        const auto meets = [&](const std::vector<std::size_t> &chain,
                               double chord) {
            SurfacePoint found{chain.front(), chain.front(), 0.0};
            for (std::size_t step = 0; step + 1 < chain.size(); ++step) {
                const double from = chordAt(chain[step]);
                const double to = chordAt(chain[step + 1]);
                if ((chord >= std::min(from, to))
                    && (chord <= std::max(from, to))) {
                    const double span = to - from;
                    found = {chain[step],
                             chain[step + 1],
                             std::abs(span) < 1.0e-9
                                 ? 0.0
                                 : (chord - from) / span};
                    break;
                }
            }
            return found;
        };
        const auto placeOf = [&](const SurfacePoint &point) {
            return loopPoints[point.a] * (1.0 - point.blend)
                   + loopPoints[point.b] * point.blend;
        };
        // Reuse the outline node when the station lands on one, so the
        // rib keeps its exact grip on the skin; otherwise pin a new node
        // onto that outline segment, which the skin still carries.
        const auto nodeOf = [&](const SurfacePoint &point) {
            if (point.blend <= 1.0e-6) {
                return static_cast<std::size_t>(loop[point.a]);
            }
            if (point.blend >= 1.0 - 1.0e-6) {
                return static_cast<std::size_t>(loop[point.b]);
            }
            const softwing::Vec3 place = placeOf(point);
            const std::size_t created = body->addNode(place, interiorMass);
            for (const std::size_t anchor :
                 {static_cast<std::size_t>(loop[point.a]),
                  static_cast<std::size_t>(loop[point.b])}) {
                tie(created,
                    anchor,
                    length(mesh.nodes[anchor] - place),
                    skinCompliance);
            }
            return created;
        };

        // Stations are spaced by the mesh the holes need, not by the
        // outline's own vertex count: with cells bigger than a hole the
        // middle almost never lands inside one and the holes vanish.
        std::vector<double> stations;
        for (std::size_t step = 0; step + 1 < upper.size(); ++step) {
            const double from = chordAt(upper[step]);
            const double to = chordAt(upper[step + 1]);
            for (int split = 0; split < stationSplit; ++split) {
                stations.push_back(
                    from + (to - from) * split / stationSplit);
            }
        }
        stations.push_back(chordAt(upper.back()));

        std::vector<std::vector<std::size_t>> struts;
        std::vector<std::vector<softwing::Vec3>> strutPoints;
        struts.reserve(stations.size());
        strutPoints.reserve(stations.size());
        for (const double chord : stations) {
            const SurfacePoint crest = meets(upper, chord);
            const SurfacePoint foot = meets(lower, chord);
            const softwing::Vec3 top = placeOf(crest);
            const softwing::Vec3 base = placeOf(foot);
            std::vector<softwing::Vec3> points;
            points.reserve(static_cast<std::size_t>(layers) + 1);
            for (int layer = 0; layer <= layers; ++layer) {
                const double blend =
                    static_cast<double>(layer) / layers;
                points.push_back(top * (1.0 - blend) + base * blend);
            }
            strutPoints.push_back(std::move(points));

            // Interior node ids are filled lazily below.
            std::vector<std::size_t> ids(
                static_cast<std::size_t>(layers) + 1, noConstraint);
            ids.front() = nodeOf(crest);
            ids.back() = nodeOf(foot);
            struts.push_back(std::move(ids));
        }

        const auto strutNode = [&](std::size_t strut,
                                   std::size_t layer) -> std::size_t {
            std::size_t &id = struts[strut][layer];
            if (id == noConstraint) {
                id = body->addNode(strutPoints[strut][layer], interiorMass);
            }
            return id;
        };

        for (std::size_t strut = 0; strut + 1 < struts.size(); ++strut) {
            for (std::size_t layer = 0;
                 layer < static_cast<std::size_t>(layers);
                 ++layer) {
                const softwing::Vec3 middle =
                    (strutPoints[strut][layer]
                     + strutPoints[strut][layer + 1]
                     + strutPoints[strut + 1][layer]
                     + strutPoints[strut + 1][layer + 1])
                    * 0.25;
                if (inHole(middle, frame)) {
                    continue;
                }
                const std::size_t topA = strutNode(strut, layer);
                const std::size_t lowA = strutNode(strut, layer + 1);
                const std::size_t topB = strutNode(strut + 1, layer);
                const std::size_t lowB = strutNode(strut + 1, layer + 1);

                const auto &positions = body->nodes();
                const auto span = [&](std::size_t a, std::size_t b) {
                    return length(positions[b].position
                                  - positions[a].position);
                };
                // Across the section, along it, and one diagonal so the
                // bay carries shear instead of folding over. A second
                // diagonal was tried on the theory that XPBD's residual
                // would be biased along a single brace; it moved the
                // settled volume by 0.5% and cost 680 constraints, so the
                // bay stays singly braced.
                tie(topA, lowA, span(topA, lowA), skinCompliance);
                tie(topB, lowB, span(topB, lowB), skinCompliance);
                tie(topA, topB, span(topA, topB), skinCompliance);
                tie(lowA, lowB, span(lowA, lowB), skinCompliance);
                tie(topA, lowB, span(topA, lowB), skinCompliance);

                const auto addRibFace = [&](std::size_t a,
                                            std::size_t b,
                                            std::size_t c) {
                    if (a == b || b == c || c == a) {
                        return;
                    }
                    RenderFace drawn;
                    drawn.surface = SimSurface::Rib;
                    drawn.nodes = {a, b, c};
                    drawn.edges = {sideConstraint(a, b),
                                   sideConstraint(b, c),
                                   sideConstraint(c, a)};
                    sim.renderFaces.push_back(drawn);
                };
                addRibFace(topA, topB, lowB);
                addRibFace(topA, lowB, lowA);
            }
        }
    }
    // No pin here any more: the aerodynamic load keeps every line taut
    // against the fixed pilot-end anchors, and the wing hangs in its
    // lines like the real thing.

    // Where each skin triangle sits on the wing, so the load model can
    // evaluate a chordwise pressure distribution for it. The nearest rib is
    // the one whose plane the face is closest to, which follows sweep and
    // arc correctly where a plain spanwise coordinate would not.
    if (!sim.ribChords.empty()) {
        sim.faceAero.resize(triangles.size());
        for (std::size_t face = 0; face < triangles.size(); ++face) {
            const auto &tri = triangles[face];
            const softwing::Vec3 centroid =
                (mesh.nodes[static_cast<std::size_t>(tri[0])]
                 + mesh.nodes[static_cast<std::size_t>(tri[1])]
                 + mesh.nodes[static_cast<std::size_t>(tri[2])])
                / 3.0;
            std::size_t best = 0;
            double bestDistance = std::numeric_limits<double>::max();
            for (std::size_t index = 0; index < sim.ribChords.size();
                 ++index) {
                const RibChord &rib = sim.ribChords[index];
                const double distance = std::abs(dot(
                    centroid - mesh.nodes[rib.leadingNode], rib.spanAxis));
                if (distance < bestDistance) {
                    bestDistance = distance;
                    best = index;
                }
            }
            const RibChord &rib = sim.ribChords[best];
            const softwing::Vec3 chord = mesh.nodes[rib.trailingNode]
                                         - mesh.nodes[rib.leadingNode];
            const double chordLengthSquared = lengthSquared(chord);
            const double station =
                chordLengthSquared > 0.0
                    ? dot(centroid - mesh.nodes[rib.leadingNode], chord)
                          / chordLengthSquared
                    : 0.0;
            FaceAero aero;
            aero.rib = static_cast<std::uint32_t>(best);
            aero.chordFraction =
                static_cast<float>(std::clamp(station, 0.0, 1.0));
            // Vents sit at the leading edge underside, where the flow
            // stagnates, so they belong with the lower surface.
            aero.upperSurface =
                triangleSurfaces[face] == SimSurface::Extrados;
            sim.faceAero[face] = aero;
        }

        // PROJECTED planform: the upper skin's shadow on the ground plane,
        // i.e. only the upward component of each face's area vector. The
        // wetted area was tried first and is wrong twice over — it counts
        // the arc's curled tips at full value, under-reading the aspect
        // ratio by a third, and it over-reads the lift reference area the
        // same way. Projected span over projected area is the pair the
        // induced-drag law is written for.
        double projectedArea = 0.0;
        for (std::size_t face = 0; face < triangles.size(); ++face) {
            if (!sim.faceAero[face].upperSurface) {
                continue;
            }
            const auto &tri = triangles[face];
            const softwing::Vec3 &a = mesh.nodes[static_cast<std::size_t>(tri[0])];
            const softwing::Vec3 &b = mesh.nodes[static_cast<std::size_t>(tri[1])];
            const softwing::Vec3 &c = mesh.nodes[static_cast<std::size_t>(tri[2])];
            projectedArea += std::max(0.0, 0.5 * cross(b - a, c - a).z);
        }
        sim.planformArea = projectedArea;
        double spanLow = std::numeric_limits<double>::max();
        double spanHigh = std::numeric_limits<double>::lowest();
        for (const softwing::Vec3 &node : mesh.nodes) {
            spanLow = std::min(spanLow, node.x);
            spanHigh = std::max(spanHigh, node.x);
        }
        const double span = spanHigh - spanLow;
        if (projectedArea > 0.0 && span > 0.0) {
            sim.aspectRatio = span * span / projectedArea;
        }

        softwing::Vec3 meanChord;
        softwing::Vec3 meanSpan;
        softwing::Vec3 meanAttitude;
        for (const RibChord &rib : sim.ribChords) {
            meanChord += normalized(mesh.nodes[rib.trailingNode]
                                    - mesh.nodes[rib.leadingNode]);
            meanSpan += rib.spanAxis;
            const softwing::Vec3 attitude =
                mesh.nodes[rib.referenceNode] - mesh.nodes[rib.leadingNode];
            if (length(attitude) > 0.0) {
                meanAttitude += normalized(attitude);
            }
        }
        if (length(meanChord) > 0.0) {
            sim.restChordDirection = normalized(meanChord);
        }
        if (length(meanSpan) > 0.0) {
            sim.restSpanAxis = normalized(meanSpan);
        }

        // Calibrate the attitude line against the chord it stands in for.
        // Both flattened into the plane the pitch is measured in, and the
        // angle between them stored so the live measurement can be rotated
        // back onto the chord: the reference node rides on the extrados,
        // tens of degrees above the chord line, and without this the whole
        // stability stack would be reading an angle offset by the
        // aerofoil's thickness.
        if (length(meanAttitude) > 0.0 && length(meanChord) > 0.0) {
            const softwing::Vec3 axis = sim.restSpanAxis;
            const softwing::Vec3 chordFlat =
                normalized(sim.restChordDirection
                           - dot(sim.restChordDirection, axis) * axis);
            const softwing::Vec3 attitudeFlat = normalized(
                meanAttitude - dot(meanAttitude, axis) * axis);
            if (length(chordFlat) > 0.0 && length(attitudeFlat) > 0.0) {
                sim.attitudeOffsetRadians = std::atan2(
                    dot(cross(attitudeFlat, chordFlat), axis),
                    dot(attitudeFlat, chordFlat));
            }
        }

        // The half-span partition, taken once here from the REST pose and
        // never recomputed. Everything the free-flight force pass does per
        // half — its angle of attack, its brake, its polar, its anchor,
        // its share of the planform — indexes through this, so it has to
        // be a property of the design rather than of the current pose: a
        // face wandering across the centreline as the wing deforms would
        // change which brake's polar loads it, frame by frame.
        //
        // The centre is the mean of the two tip ribs' stations rather than
        // zero, so a mesh drawn off the origin still splits down its own
        // middle.
        {
            double stationLow = std::numeric_limits<double>::max();
            double stationHigh = std::numeric_limits<double>::lowest();
            std::vector<double> ribStation(sim.ribChords.size(), 0.0);
            for (std::size_t index = 0; index < sim.ribChords.size();
                 ++index) {
                ribStation[index] =
                    dot(mesh.nodes[sim.ribChords[index].leadingNode],
                        sim.restSpanAxis);
                stationLow = std::min(stationLow, ribStation[index]);
                stationHigh = std::max(stationHigh, ribStation[index]);
            }
            const double middle = 0.5 * (stationLow + stationHigh);
            sim.ribHalf.assign(sim.ribChords.size(), 0);
            for (std::size_t index = 0; index < sim.ribChords.size();
                 ++index) {
                sim.ribHalf[index] =
                    ribStation[index] < middle ? 0 : 1;
            }
            // Each half's share of the projected planform, from the same
            // upper-surface shadow the wing-level figure is summed from.
            sim.halfPlanformArea = {0.0, 0.0};
            sim.ribPlanformArea.assign(sim.ribChords.size(), 0.0);
            for (std::size_t face = 0; face < triangles.size(); ++face) {
                if (!sim.faceAero[face].upperSurface) {
                    continue;
                }
                const auto &tri = triangles[face];
                const softwing::Vec3 &a =
                    mesh.nodes[static_cast<std::size_t>(tri[0])];
                const softwing::Vec3 &b =
                    mesh.nodes[static_cast<std::size_t>(tri[1])];
                const softwing::Vec3 &c =
                    mesh.nodes[static_cast<std::size_t>(tri[2])];
                const double facePlanform =
                    std::max(0.0, 0.5 * cross(b - a, c - a).z);
                const std::size_t rib = sim.faceAero[face].rib;
                sim.halfPlanformArea[sim.ribHalf[rib]] += facePlanform;
                sim.ribPlanformArea[rib] += facePlanform;
            }
            // Rescaled to sum to the wing-level figure exactly, so two
            // half-passes at equal coefficients impose the same total
            // force the single pass did rather than one a rounding apart.
            const double halfSum =
                sim.halfPlanformArea[0] + sim.halfPlanformArea[1];
            if (halfSum > 0.0) {
                const double scale = sim.planformArea / halfSum;
                sim.halfPlanformArea[0] *= scale;
                sim.halfPlanformArea[1] *= scale;
                for (double &ribArea : sim.ribPlanformArea) {
                    ribArea *= scale;
                }
            }
        }

        // The two ribs furthest out along the rest span axis. The live span
        // axis is read between their leading edges each frame, which is
        // what lets the wing-level angle of attack follow a rolled or
        // yawed wing instead of its rest pose.
        double tipLow = std::numeric_limits<double>::max();
        double tipHigh = std::numeric_limits<double>::lowest();
        for (std::size_t index = 0; index < sim.ribChords.size(); ++index) {
            const double station = dot(
                mesh.nodes[sim.ribChords[index].leadingNode],
                sim.restSpanAxis);
            if (station < tipLow) {
                tipLow = station;
                sim.spanTipRibs[0] = index;
            }
            if (station > tipHigh) {
                tipHigh = station;
                sim.spanTipRibs[1] = index;
            }
        }

        // The pneumatic cells: one per bay between spanwise-adjacent ribs.
        // Adjacency is geometric — ribs sorted by their leading edge's
        // station along the rest span axis — because the mesh file's loop
        // order carries no such promise.
        if (sim.ribChords.size() >= 2) {
            std::vector<std::size_t> order(sim.ribChords.size());
            for (std::size_t index = 0; index < order.size(); ++index) {
                order[index] = index;
            }
            std::sort(order.begin(), order.end(),
                      [&](std::size_t a, std::size_t b) {
                          return dot(mesh.nodes[sim.ribChords[a].leadingNode],
                                     sim.restSpanAxis)
                                 < dot(mesh.nodes[sim.ribChords[b].leadingNode],
                                       sim.restSpanAxis);
                      });
            // Rank of each rib in span order, for the face → cell map.
            std::vector<std::size_t> rank(order.size());
            for (std::size_t position = 0; position < order.size();
                 ++position) {
                rank[order[position]] = position;
            }

            // Rest section area per rib as the loop's VECTOR area — the
            // same quantity the live collapse signal reads each frame, so
            // the ratio of the two is meaningful. A folded loop's edge
            // cross products cancel, which is exactly what makes it a
            // collapse signal; a plane fit would chase the fold instead.
            std::vector<double> ribArea(sim.ribChords.size(), 0.0);
            for (std::size_t rib = 0; rib < sim.ribLoopNodes.size();
                 ++rib) {
                softwing::Vec3 sum;
                const auto &loop = sim.ribLoopNodes[rib];
                for (std::size_t index = 0; index < loop.size(); ++index) {
                    sum += cross(
                        mesh.nodes[loop[index]],
                        mesh.nodes[loop[(index + 1) % loop.size()]]);
                }
                ribArea[rib] = 0.5 * length(sum);
            }

            sim.cells.resize(sim.ribChords.size() - 1);
            for (std::size_t cell = 0; cell < sim.cells.size(); ++cell) {
                SimCell &record = sim.cells[cell];
                record.ribs = {order[cell], order[cell + 1]};
                record.restSectionArea = 0.5
                                         * (ribArea[order[cell]]
                                            + ribArea[order[cell + 1]]);
                const double spacing = length(
                    mesh.nodes[sim.ribChords[order[cell + 1]].leadingNode]
                    - mesh.nodes[sim.ribChords[order[cell]].leadingNode]);
                const double proxyVolume = record.restSectionArea * spacing;
                record.restProxyVolume =
                    proxyVolume > 0.0 && std::isfinite(proxyVolume)
                        ? proxyVolume
                        : 1.0e-6;
                record.restVolume = record.restProxyVolume;
                // Cross-port to the next cell: the hole outlines of the
                // rib the two bays share. Closed loops, so the vector
                // area is origin-independent.
                if (cell + 1 < sim.cells.size()) {
                    const std::size_t shared = ribMeshIndex[order[cell + 1]];
                    double portArea = 0.0;
                    for (const auto &outline : mesh.ribHoles[shared]) {
                        softwing::Vec3 sum;
                        for (std::size_t index = 0; index < outline.size();
                             ++index) {
                            sum += cross(
                                outline[index],
                                outline[(index + 1) % outline.size()]);
                        }
                        portArea += 0.5 * length(sum);
                    }
                    record.portAreaToNext = portArea;
                }
            }

            // Face → cell: the nearest rib's rank, stepped one bay back
            // when the face sits on that rib's low-span side. The span
            // axes are all oriented +x, so a positive signed distance to
            // the rib plane means the high-span side.
            // Same convention as applyPressure: free flight ignores the
            // slider and flies in level air, so its opening fraction must
            // be normalised against the wind it will actually measure.
            // Unit design-flow direction. Keep this expression independent
            // of speed (and bit-identical for the zero-ambient tunnel): it
            // normalises the mouth geometry, not the surrounding air mass.
            const softwing::Vec3 buildWind = rotateAbout(
                sim.restChordDirection, sim.restSpanAxis,
                (controls.freeFlight ? 0.0
                                     : controls.angleOfAttackDegrees)
                    * kDegreesToRadians);
            std::vector<softwing::Vec3> restMouth(sim.cells.size());
            for (std::size_t face = 0; face < triangles.size(); ++face) {
                FaceAero &aero = sim.faceAero[face];
                const RibChord &rib = sim.ribChords[aero.rib];
                const auto &tri = triangles[face];
                const softwing::Vec3 centroid =
                    (mesh.nodes[static_cast<std::size_t>(tri[0])]
                     + mesh.nodes[static_cast<std::size_t>(tri[1])]
                     + mesh.nodes[static_cast<std::size_t>(tri[2])])
                    / 3.0;
                const double side = dot(
                    centroid - mesh.nodes[rib.leadingNode], rib.spanAxis);
                const std::size_t position = rank[aero.rib];
                std::size_t cell =
                    side >= 0.0 ? position
                                : (position == 0 ? 0 : position - 1);
                cell = std::min(cell, sim.cells.size() - 1);
                aero.cell = static_cast<std::uint32_t>(cell);
                SimCell &record = sim.cells[cell];
                const auto entirelyOnLoop = [&](const auto &loop) {
                    return std::all_of(
                        tri.begin(), tri.end(), [&](int node) {
                            return std::find(
                                       loop.begin(), loop.end(),
                                       static_cast<std::size_t>(node))
                                   != loop.end();
                        });
                };
                // Imported wing tips can already contain authored caps.
                // Exclude them because the volume helper supplies one
                // virtual cap at each bounding rib for every bay.
                if (!entirelyOnLoop(sim.ribLoopNodes[record.ribs[0]])
                    && !entirelyOnLoop(
                        sim.ribLoopNodes[record.ribs[1]])) {
                    record.skinFaces.push_back(face);
                }
                if (triangleSurfaces[face] == SimSurface::Vent) {
                    record.ventFaces.push_back(face);
                    const softwing::Vec3 &a =
                        mesh.nodes[static_cast<std::size_t>(tri[0])];
                    const softwing::Vec3 &b =
                        mesh.nodes[static_cast<std::size_t>(tri[1])];
                    const softwing::Vec3 &c =
                        mesh.nodes[static_cast<std::size_t>(tri[2])];
                    const softwing::Vec3 area = 0.5 * cross(b - a, c - a);
                    record.restVentArea += length(area);
                    // Vent faces are wound outward, so at rest the area
                    // vector opposes the build-time airflow; the dot
                    // against -wind is the scoop the live one is
                    // normalised by. The vector itself accumulates
                    // separately — its length is the mouth's opening,
                    // which is what a fold takes away.
                    record.restVentProjection +=
                        dot(area, -1.0 * buildWind);
                    restMouth[cell] += area;
                }
            }
            for (std::size_t cell = 0; cell < sim.cells.size(); ++cell) {
                SimCell &record = sim.cells[cell];
                if (!(record.restVentProjection > 0.0)
                    || !std::isfinite(record.restVentProjection)) {
                    record.restVentProjection = 1.0e-9;
                }
                const double aperture = length(restMouth[cell]);
                record.restVentAperture =
                    aperture > 0.0 && std::isfinite(aperture)
                        ? aperture
                        : 1.0e-9;
                const softwing::ClosedCellVolumeEstimate volume =
                    softwing::estimateClosedCellVolume(
                        body->nodes(), body->triangles(), record.skinFaces,
                        sim.ribLoopNodes[record.ribs[0]],
                        sim.ribLoopNodes[record.ribs[1]]);
                if (volume.valid && volume.volume > 0.0
                    && std::isfinite(volume.volume)) {
                    record.restVolume = volume.volume;
                }
            }
        }
    }

    // Internal V/H/VH-rib and mini-rib sheets: tie each sample pair of
    // a strap together through the nearest mesh nodes, so line load
    // spreads across neighbouring ribs like the real diagonals do.
    const auto nearestMeshNode = [&](const softwing::Vec3 &point) {
        double bestDistance = 0.08;
        int bestNode = -1;
        for (std::size_t index = 0; index < mesh.nodes.size(); ++index) {
            const double distance = length(mesh.nodes[index] - point);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestNode = static_cast<int>(index);
            }
        }
        return bestNode;
    };
    for (const SimStrap &strap : mesh.straps) {
        // Sample pairs that resolved to real nodes, kept in order so
        // the sheet between them can be drawn as a ribbon.
        std::vector<std::pair<std::size_t, std::size_t>> rungs;
        for (std::size_t sample = 0; sample < strap.a.size(); ++sample) {
            const int nodeA = nearestMeshNode(strap.a[sample]);
            const int nodeB = nearestMeshNode(strap.b[sample]);
            if (nodeA < 0 || nodeB < 0 || nodeA == nodeB) {
                continue;
            }
            tie(static_cast<std::size_t>(nodeA),
                static_cast<std::size_t>(nodeB),
                length(mesh.nodes[static_cast<std::size_t>(nodeB)]
                       - mesh.nodes[static_cast<std::size_t>(nodeA)]),
                skinCompliance);
            rungs.emplace_back(static_cast<std::size_t>(nodeA),
                               static_cast<std::size_t>(nodeB));
        }
        // Two triangles per gap between consecutive rungs. The rails
        // run along the ribs and are usually not constrained, so a
        // sheet's colour comes from the ties that hold it.
        for (std::size_t rung = 0; rung + 1 < rungs.size(); ++rung) {
            const auto [a0, b0] = rungs[rung];
            const auto [a1, b1] = rungs[rung + 1];
            for (const std::array<std::size_t, 3> corners :
                 {std::array<std::size_t, 3>{a0, b0, b1},
                  std::array<std::size_t, 3>{a0, b1, a1}}) {
                if (corners[0] == corners[1] || corners[1] == corners[2]
                    || corners[2] == corners[0]) {
                    continue;
                }
                RenderFace drawn;
                drawn.surface = SimSurface::Strap;
                drawn.nodes = corners;
                for (int corner = 0; corner < 3; ++corner) {
                    drawn.edges[static_cast<std::size_t>(corner)] =
                        sideConstraint(
                            corners[static_cast<std::size_t>(corner)],
                            corners[static_cast<std::size_t>(
                                (corner + 1) % 3)]);
                }
                sim.renderFaces.push_back(drawn);
            }
        }
    }

    // Everything added so far is canopy; lines and pilot follow.
    sim.canopyNodeCount = body->nodes().size();

    // An inflated wing does not accelerate only its few kilograms of cloth:
    // it also accelerates a substantial surrounding/entrained air volume.
    // Omitting that added mass gave the canopy roughly a 20:1 payload mass
    // ratio, so a pressure change flung it ahead of the pilot, unloaded every
    // line, and the later re-catch produced a multi-kN pulse. Use the
    // classical flat-planform normal added-mass estimate as scalar solver
    // inertia. It is distributed by skin area and its gravity is cancelled
    // in applyAerodynamicForces, so launch weight and static line loads remain
    // those of the actual fabric/lines/payload.
    if (controls.freeFlight && sim.planformArea > 0.0
        && sim.aspectRatio > 0.0) {
        const double projectedSpan =
            std::sqrt(sim.aspectRatio * sim.planformArea);
        const double chordSquaredIntegral =
            projectedSpan > 0.0
                ? sim.planformArea * sim.planformArea / projectedSpan
                : 0.0;
        sim.virtualAddedAirMassKg =
            canopyAddedMassCoefficient * (kPi / 4.0) * kAirDensity
            * chordSquaredIntegral;
        sim.virtualAddedAirMassByNode.assign(body->nodes().size(), 0.0);
        const double skinMass = std::accumulate(
            masses.begin(), masses.end(), 0.0);
        if (skinMass > 0.0 && sim.virtualAddedAirMassKg > 0.0) {
            for (std::size_t node = 0; node < masses.size(); ++node) {
                const double share =
                    sim.virtualAddedAirMassKg * masses[node] / skinMass;
                sim.virtualAddedAirMassByNode[node] = share;
                softwing::Node &bodyNode = body->nodes()[node];
                if (bodyNode.inverseMass > 0.0) {
                    const double physicalMass = 1.0 / bodyNode.inverseMass;
                    bodyNode.inverseMass = 1.0 / (physicalMass + share);
                }
            }
        }
    }

    // Suspension lines: reject legacy duplicates, lump the physical mass of
    // each authored segment equally onto its welded endpoints, then create
    // the cable graph. The old 50 g-per-junction rule made line mass depend
    // on how an exporter happened to split the graph; length-lumping makes
    // it depend on how much line is actually present. The small positive
    // node floor is retained and diagnosed separately as numerical mass.
    std::vector<const SimLine *> authoredLines;
    std::map<std::uint64_t, double> junctionPhysicalMass;
    std::set<SemanticLineKey> builtLineKeys;
    double lowestZ = std::numeric_limits<double>::max();
    for (const SimLine &line : mesh.lines) {
        if (!builtLineKeys.insert(semanticLineKey(line)).second) {
            ++sim.duplicateLineCount;
            continue;
        }
        const std::uint64_t aKey = quantizedKey(line.a);
        const std::uint64_t bKey = quantizedKey(line.b);
        if (aKey == bKey) {
            continue;
        }
        const double segmentMass =
            lineLinearDensityKgPerMetre * length(line.b - line.a);
        junctionPhysicalMass[aKey] += 0.5 * segmentMass;
        junctionPhysicalMass[bKey] += 0.5 * segmentMass;
        sim.authoredLineMassKg += segmentMass;
        lowestZ = std::min({lowestZ, line.a.z, line.b.z});
        authoredLines.push_back(&line);
    }

    std::map<std::uint64_t, std::size_t> junctions;
    const auto lineNode = [&](const softwing::Vec3 &point) {
        const std::uint64_t key = quantizedKey(point);
        const auto [entry, inserted] =
            junctions.try_emplace(key, 0);
        if (inserted) {
            const double physicalMass = junctionPhysicalMass[key];
            if (controls.freeFlight) {
                entry->second = body->addNode(
                    point, physicalMass + lineJunctionNumericalMassKg);
                sim.lineJunctionFloorMassKg +=
                    lineJunctionNumericalMassKg;
            } else {
                // A pinned, zero-gravity tunnel is a static relaxation
                // problem. Retain the old 50 g/junction inertia there so the
                // light cable cascade converges at 30x2, but name and account
                // for it as nonphysical solver ballast. Free flight never
                // takes this branch and carries physical length-lumped mass.
                entry->second = body->addNode(
                    point, tunnelLineJunctionRelaxationMassKg);
                sim.tunnelLineSolverBallastKg +=
                    tunnelLineJunctionRelaxationMassKg;
            }
        }
        return entry->second;
    };
    for (const SimLine *linePointer : authoredLines) {
        const SimLine &line = *linePointer;
        const std::size_t a = lineNode(line.a);
        const std::size_t b = lineNode(line.b);
        sim.lineSegments.push_back(
            {a,
             b,
             line.brake,
             body->addCableConstraint(
                 a, b, length(line.b - line.a), lineCompliance),
             line.plan,
             true});
    }
    std::vector<std::size_t> carabiners;
    for (const auto &[key, node] : junctions) {
        const softwing::Vec3 position = body->nodes()[node].position;
        if (position.z < lowestZ + anchorBandMetres) {
            // The pilot end. Nothing is pinned any more: the whole system
            // flies, and these become the carabiners the harness hangs on.
            carabiners.push_back(node);
            continue;
        }
        // Tie upper junctions to the canopy when they sit on it.
        double bestDistance = lineAttachRadiusMetres;
        int bestSkinNode = -1;
        for (std::size_t skin = 0; skin < mesh.nodes.size(); ++skin) {
            const double distance = length(mesh.nodes[skin] - position);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestSkinNode = static_cast<int>(skin);
            }
        }
        if (bestSkinNode >= 0) {
            body->addSuspensionTieConstraint(
                node,
                static_cast<std::size_t>(bestSkinNode),
                bestDistance,
                lineCompliance);
            sim.lineAttachmentNodes.push_back(
                static_cast<std::size_t>(bestSkinNode));
        }
    }
    std::sort(sim.lineAttachmentNodes.begin(),
              sim.lineAttachmentNodes.end());
    sim.lineAttachmentNodes.erase(
        std::unique(sim.lineAttachmentNodes.begin(),
                    sim.lineAttachmentNodes.end()),
        sim.lineAttachmentNodes.end());

    // The riser level, published for the row-load instrumentation: the
    // same set of junctions whether they end up fixed to the world
    // (pinned) or tied to the pilot (free flight) below.
    sim.carabinerNodes = carabiners;

    // Where the designed lines hang the wing: the mean carabiner position
    // projected onto the mean rest chord. The imposed aerodynamic
    // resultant is anchored a small margin aft of this, which is what
    // gives the canopy a stable pitch trim at the angle the designer
    // rigged the lines for.
    if (!carabiners.empty() && !sim.ribChords.empty()) {
        softwing::Vec3 carabinerMean;
        for (const std::size_t node : carabiners) {
            carabinerMean += body->nodes()[node].position;
        }
        carabinerMean /= static_cast<double>(carabiners.size());
        softwing::Vec3 leadingMean;
        softwing::Vec3 trailingMean;
        for (const RibChord &rib : sim.ribChords) {
            leadingMean += mesh.nodes[rib.leadingNode];
            trailingMean += mesh.nodes[rib.trailingNode];
        }
        leadingMean /= static_cast<double>(sim.ribChords.size());
        trailingMean /= static_cast<double>(sim.ribChords.size());
        const softwing::Vec3 chord = trailingMean - leadingMean;
        if (lengthSquared(chord) > 0.0) {
            const double hangFraction =
                dot(carabinerMean - leadingMean, chord)
                / lengthSquared(chord);
            sim.resultantChordFraction =
                std::clamp(hangFraction, 0.10, 0.60);
        }
    }

    // The pilot: one mass slung under the risers, free like everything
    // else. This is what makes the wing behave like a wing rather than a
    // kite on a stick — the pilot carries almost all the system's inertia,
    // so braking the canopy lets him swing forward under it, and letting
    // the canopy surge lets him swing back. That is a real pendulum with a
    // real mass ratio, not an effect painted on afterwards.
    // Only in free flight. Pinned, the carabiners are nailed down exactly
    // as they always were and no pilot exists: hanging them off a single
    // mass instead changes the riser geometry, and the canopy notices.
    softwing::Vec3 pilotPlace;
    if (!carabiners.empty() && controls.freeFlight) {
        for (const std::size_t node : carabiners) {
            pilotPlace += body->nodes()[node].position;
        }
        pilotPlace /= static_cast<double>(carabiners.size());
        pilotPlace.z -= pilotDropMetres;
        // Pilot plus harness/instruments is an explicit input. Clamping at
        // this structural boundary keeps programmatic and CLI callers as
        // safe as the GUI control, without fitting the payload to the polar
        // that is supposed to carry it.
        const double requestedPilotMass =
            std::isfinite(controls.pilotMassKg)
                ? controls.pilotMassKg
                : defaultPilotMassKg;
        sim.pilotMass = std::clamp(requestedPilotMass,
                                   minimumPilotMassKg,
                                   maximumPilotMassKg);
        sim.pilotNode = body->addNode(pilotPlace, sim.pilotMass);
        for (const std::size_t node : carabiners) {
            suspensionTie(
                sim.pilotNode,
                node,
                length(body->nodes()[node].position - pilotPlace),
                lineCompliance);
            // Registered as drawable segments so the pilot is visible:
            // without them the risers end mid-air and the mass the whole
            // pendulum hangs on cannot be seen.
            sim.lineSegments.push_back(
                {sim.pilotNode,
                 node,
                 false,
                 sideConstraint(sim.pilotNode, node),
                 0,
                 false});
        }
    } else {
        for (const std::size_t node : carabiners) {
            body->fixNode(node);
        }
    }

    // Brakes. The handle is the pilot's own hand, so rather than synthesize
    // a node and drag it about, the brake line runs from the pilot to the
    // top of the brake cascade and pulling the brake shortens it. The pull
    // is then a real force between two real masses: the canopy's trailing
    // edge comes down and the pilot feels it, which is the coupling the
    // whole pendulum depends on.
    std::vector<std::size_t> brakeHandles;
    for (const double side : {-1.0, 1.0}) {
        std::size_t lowestBrake = 0;
        double lowestBrakeZ = std::numeric_limits<double>::max();
        softwing::Vec3 carabiner;
        double carabinerZ = std::numeric_limits<double>::max();
        bool sawBrake = false;
        for (const auto &[key, node] : junctions) {
            const softwing::Vec3 position = body->nodes()[node].position;
            if (position.x * side <= 0.0) {
                continue;
            }
            if (position.z < carabinerZ) {
                carabinerZ = position.z;
                carabiner = position;
            }
            for (const LineSegment &segment : sim.lineSegments) {
                if (segment.brake
                    && (segment.a == node || segment.b == node)
                    && position.z < lowestBrakeZ) {
                    lowestBrakeZ = position.z;
                    lowestBrake = node;
                    sawBrake = true;
                    break;
                }
            }
        }
        if (!sawBrake) {
            continue;
        }
        std::vector<std::size_t> brakeTops;
        for (const LineSegment &segment : sim.lineSegments) {
            if (!segment.brake) {
                continue;
            }
            if (segment.a == lowestBrake) {
                brakeTops.push_back(segment.b);
            } else if (segment.b == lowestBrake) {
                brakeTops.push_back(segment.a);
            }
        }
        // One handle per side, where that side's carabiner is. Running both
        // brakes off a single central point instead pulls the two tips
        // toward each other: the cables are sized to the rest geometry, so
        // they go taut as the wing spreads and hold it in. That cost gnuC2
        // three metres of span before it was spotted.
        softwing::Vec3 handlePosition = carabiner;
        handlePosition.z -= 0.3;
        // This is a solver/control point standing in for the pilot's hand,
        // whose physical mass is already inside pilotMass. It therefore gets
        // only the explicit numerical floor and is not counted as authored
        // suspension-line mass.
        const std::size_t handle =
            body->addNode(handlePosition, controlNodeNumericalMassKg);
        sim.controlNodeFloorMassKg += controlNodeNumericalMassKg;
        // In free flight the pilot holds the handle, so a brake pull reacts
        // into his mass. Pinned, the handle is nailed down instead.
        if (sim.pilotNode != noConstraint) {
            suspensionTie(handle,
                          sim.pilotNode,
                          length(handlePosition - pilotPlace),
                          lineCompliance);
        }
        brakeHandles.push_back(handle);
        for (const std::size_t top : brakeTops) {
            const double rest =
                length(body->nodes()[top].position - handlePosition);
            const std::size_t constraint = body->addCableConstraint(
                handle, top, rest, lineCompliance);
            // Plan 6: the engine hard-codes brakes to the F row.
            sim.lineSegments.push_back(
                {handle, top, true, constraint, 6, false});
            sim.brakeLines.push_back({constraint, rest, side < 0.0});
        }
    }

    sim.body = std::move(body);
    prepareContact(sim);

    // Calibrate the rest field against the q-derived apparent flow. In free
    // flight the actual atmosphere is ambient only, so give every component
    // the matching common ground velocity temporarily. This initializes the
    // vents, cells and trim from the reference condition without inventing a
    // moving free-flight atmosphere. DropFromRest is reset to zero below but
    // deliberately keeps this pre-inflated state.
    if (controls.freeFlight) {
        const softwing::Vec3 calibrationVelocity =
            controls.ambientAirVelocityWorld
            - referenceFlowVelocity(sim, controls);
        for (softwing::Node &node : sim.body->nodes()) {
            node.velocity = calibrationVelocity;
        }
    }
    applyPressure(sim, controls);

    // The angle the designed line geometry rigs the wing to fly at: the
    // wing-level angle of attack of the rest pose under the build-time
    // airflow. The pitch-trim anchor holds the wing here.
    {
        const WingAeroSample rest = sampleWingAero(sim, controls);
        if (rest.valid) {
            sim.alphaTrimRadians = rest.alphaRadians;
        }
        sim.builtAngleOfAttackDegrees = controls.angleOfAttackDegrees;
    }

    // Estimate the glide-path angle used by the smart launch. Payload mass is
    // absent from the angle calculation: it changes the speed needed to carry
    // the system, not the polar's L/D at a given incidence. The achieved-load
    // calibration below supplies that weight-aware speed separately.
    if (sim.pilotNode != noConstraint) {
        // In flight the path descends at the glide angle, so the wing sees
        // its rest incidence plus that angle. Iterate the polar pair to the
        // same fixed point the previous smart launch used.
        const WingAeroSample aero = sampleWingAero(sim, controls);
        if (aero.valid && sim.planformArea > 0.0) {
            const double aspectRatio = std::max(1.0, sim.aspectRatio);
            const double finiteWing =
                1.0 / (1.0 + 2.0 / (aspectRatio * kSpanEfficiency));
            double alphaFly = aero.alphaRadians;
            for (int iteration = 0; iteration < 4; ++iteration) {
                const double liftCoefficient =
                    finiteWing * wingLiftCoefficient(alphaFly);
                const double dragCoefficient =
                    kParasiticDragCoefficient
                    + liftCoefficient * liftCoefficient
                          / (kPi * aspectRatio * kSpanEfficiency);
                const double lift = aero.dynamicPressure * sim.planformArea
                                    * liftCoefficient;
                const double drag = aero.dynamicPressure
                                    * (sim.planformArea * dragCoefficient
                                       + kPilotDragArea);
                sim.glideAngleRadians = std::atan2(drag, lift);
                alphaFly = aero.alphaRadians + sim.glideAngleRadians;
            }
            sim.alphaTrimRadians = alphaFly;
        }
    }
    for (const std::size_t node : brakeHandles) {
        if (!controls.freeFlight) {
            sim.body->fixNode(node);
        }
    }

    // TrimmedGlide launches on the estimated flight path. A short deterministic
    // scalar calibration on the untouched rest geometry sizes q so the
    // bounded field's ACHIEVED vertical wing force carries the complete
    // simulated mass. This is intentionally not the requested polar lift: Cp
    // saturation can make that unavailable. Starting the bounded force-stage
    // cache near its measured rest authority avoids calibrating against the
    // production path's deliberately conservative 0.02 first probe; the final
    // verified authority and dual are retained for the first stepped frame.
    //
    // DropFromRest deliberately leaves every node's default velocity at zero:
    // the already stamped pressure makes it a pre-inflated release experiment,
    // useful for watching gravity tension the dynamic point-payload suspension
    // without pretending to be a ground inflation or a solved trim state.
    if (controls.freeFlight && sim.pilotNode != noConstraint
        && controls.launchMode == LaunchMode::TrimmedGlide) {
        const WingAeroSample rest = sampleWingAero(sim, controls);
        const softwing::Vec3 launchAxis =
            rest.valid ? rest.spanAxis : sim.restSpanAxis;
        const softwing::Vec3 referenceApparentFlow =
            rotateAbout(referenceFlowVelocity(sim, controls),
                        launchAxis,
                        sim.glideAngleRadians);
        const double referenceQ = std::max(0.0, controls.pressurePascal);
        double launchQ = referenceQ;
        double achievedVertical = 0.0;
        int calibrationIterations = 0;
        if (controls.pressureSolveMode
                == PressureSolveMode::BoundedExteriorCp
            && sim.pressureAuthorityHint[0] <= 0.0) {
            // The full-q undeformed gnuC2 audit measured 0.1608 force-ray
            // authority. This is only a starting hint: every calibration pass
            // still projects it and deterministically backs off if its own
            // mesh cannot carry it.
            sim.pressureAuthorityHint[0] = 0.16;
        }
        const auto setCommonVelocityAt = [&](double dynamicPressure) {
            const double scale =
                referenceQ > 1.0e-12
                    ? std::sqrt(std::max(0.0, dynamicPressure) / referenceQ)
                    : 0.0;
            const softwing::Vec3 descent =
                controls.ambientAirVelocityWorld
                - scale * referenceApparentFlow;
            for (softwing::Node &node : sim.body->nodes()) {
                node.velocity = descent;
            }
        };
        const auto calibrateAt = [&](double dynamicPressure) {
            ++calibrationIterations;
            setCommonVelocityAt(dynamicPressure);
            // Each unstepped probe is a fresh steady rest condition; do not
            // let the intake relaxation from the previous scalar q bias it.
            resetCellAirState(sim);
            applyPressure(sim, controls);
            applyAerodynamicForces(sim, controls);
            const softwing::Vec3 achievedWing =
                sim.pressureSolve.achievedForce
                + sim.lastPolarDragTractionForce;
            achievedVertical = achievedWing.z;
        };

        double systemMass = 0.0;
        for (const softwing::Node &node : sim.body->nodes()) {
            if (node.inverseMass > 0.0) {
                systemMass += 1.0 / node.inverseMass;
            }
        }
        systemMass = std::max(
            0.0, systemMass - sim.virtualAddedAirMassKg);
        const double systemWeight =
            systemMass * gravityMetresPerSecondSquared;
        const double maximumQ =
            kMaximumDynamicPressureRatio * referenceQ;
        const auto resetLaunchAeroState = [&]() {
            sim.alphaFilteredRadians =
                std::numeric_limits<double>::quiet_NaN();
            sim.alphaRateRadiansPerSecond = 0.0;
            sim.alphaHalfDeviationRadians = {0.0, 0.0};
            sim.halfDynamicPressureRatio = {1.0, 1.0};
            sim.brakeApplied = {0.0, 0.0};
            sim.brakeFilteredMetres = {0.0, 0.0};
            resetCellAirState(sim);
            sim.pressureAuthorityHint = {0.16, 1.0, 1.0};
            for (auto &multipliers : sim.pressureMultiplierHint) {
                multipliers.clear();
            }
        };
        for (int iteration = 0; iteration < 2; ++iteration) {
            calibrateAt(launchQ);
            if (!(launchQ > 1.0e-12) || !(achievedVertical > 1.0e-6)
                || !(systemWeight > 0.0)) {
                break;
            }
            launchQ = std::clamp(
                launchQ * systemWeight / achievedVertical,
                0.0,
                maximumQ);
        }
        // Leave the pressure field, cell state and bounded warm start at the
        // exact q the co-moving structural relaxation starts from.
        calibrateAt(launchQ);

        // Establish the gravity/aero reaction through the real suspension
        // graph before release. Replacing every node velocity with the common
        // apparent-flow velocity erases relative vibration — this is a
        // quasi-static relaxation, not hidden flight time. Eight otherwise
        // ordinary XPBD frames let the pilot, risers and canopy find a
        // compatible loaded geometry. No travel or time-filter/control state
        // is retained, and DropFromRest plus the legacy oracle never enter
        // this path. Contact is also excluded: it is a runtime fold feature,
        // not part of trim construction.
        if (controls.pressureSolveMode
            == PressureSolveMode::BoundedExteriorCp) {
            constexpr int relaxationFrames = 8;
            SimControls relaxationControls = controls;
            relaxationControls.brakeLeft = 0.0;
            relaxationControls.brakeRight = 0.0;
            relaxationControls.fabricContact = false;
            relaxationControls.performanceProfile = nullptr;
            for (int frame = 0; frame < relaxationFrames; ++frame) {
                setCommonVelocityAt(launchQ);
                stepSimulation(sim, relaxationControls);
            }
            sim.airTravel = {};
            sim.trimmedLaunchRelaxationFrames = relaxationFrames;

            // Only geometry and constraint-compatible position survive the
            // build settle. Re-seed every wake/control state, cell state and
            // bounded warm start so the final calibration below is the first
            // stateful aerodynamic observation of the release geometry.
            resetLaunchAeroState();
        }

        // The loaded geometry can carry a different vertical coefficient
        // than the untouched rest pose. Re-size q on that retained geometry,
        // then leave the final field/cache and common launch velocity ready
        // for the first user-visible frame.
        for (int iteration = 0; iteration < 2; ++iteration) {
            calibrateAt(launchQ);
            if (!(launchQ > 1.0e-12) || !(achievedVertical > 1.0e-6)
                || !(systemWeight > 0.0)) {
                break;
            }
            launchQ = std::clamp(
                launchQ * systemWeight / achievedVertical,
                0.0,
                maximumQ);
        }
        calibrateAt(launchQ);
        sim.trimmedLaunchDynamicPressure = launchQ;
        sim.trimmedLaunchAirspeed =
            std::sqrt(2.0 * launchQ / kAirDensity);
        sim.trimmedLaunchEffectiveLiftCoefficient =
            launchQ > 1.0e-12 && sim.planformArea > 1.0e-12
                ? achievedVertical / (launchQ * sim.planformArea)
                : 0.0;
        sim.trimmedLaunchHorizontalResidualNewtons = std::hypot(
            sim.lastAeroForce.x, sim.lastAeroForce.y);
        sim.trimmedLaunchVerticalResidualNewtons =
            sim.lastAeroForce.z - systemWeight;
        sim.trimmedLaunchCalibrationIterations = calibrationIterations;
    } else if (controls.freeFlight
               && controls.launchMode == LaunchMode::DropFromRest) {
        for (softwing::Node &node : sim.body->nodes()) {
            node.velocity = {};
        }
    }

    softwing::Vec3 low{1e9, 1e9, 1e9};
    softwing::Vec3 high{-1e9, -1e9, -1e9};
    for (const softwing::Vec3 &node : mesh.nodes) {
        low = {std::min(low.x, node.x),
               std::min(low.y, node.y),
               std::min(low.z, node.z)};
        high = {std::max(high.x, node.x),
                std::max(high.y, node.y),
                std::max(high.z, node.z)};
    }
    sim.boundsLow = low;
    sim.boundsHigh = high;
    return sim;
}

namespace {

// Section lift coefficient. Thin-aerofoil slope near the working range,
// rolled off by a Gaussian past the stall so that a section which ends up
// at a silly angle stops pulling instead of pulling harder. That roll-off
// is not decoration: it is what makes the attitude stable. The old model
// faked the same effect by fading the load out as a face stopped pointing
// up, which is why it needed a fake force in the first place.
double sectionLiftCoefficient(double angleRadians)
{
    constexpr double kCamberOffset = 4.0 * kDegreesToRadians;
    constexpr double kStallAngle = 15.0 * kDegreesToRadians;
    constexpr double kStallWidth = 12.0 * kDegreesToRadians;
    const double effective = angleRadians + kCamberOffset;
    const double linear =
        2.0 * kPi * std::sin(effective) * std::cos(effective);
    const double excess = std::max(0.0, std::abs(effective) - kStallAngle);
    const double fade = excess / kStallWidth;
    return linear * std::exp(-fade * fade);
}

// The wing-level version for the imposed polar. Same shape, but the stall
// sits at the angle a whole paraglider actually stalls at (~20° with
// camber) rather than the section law's deliberately early roll-off — the
// line rigging holds the wing near 9–12°, and a law already fading there
// starved the polar of lift exactly where the wing flies.
double wingLiftCoefficient(double angleRadians)
{
    constexpr double kCamberOffset = 4.0 * kDegreesToRadians;
    constexpr double kStallAngle = 20.0 * kDegreesToRadians;
    constexpr double kStallWidth = 10.0 * kDegreesToRadians;
    const double effective = angleRadians + kCamberOffset;
    const double linear =
        2.0 * kPi * std::sin(effective) * std::cos(effective);
    const double excess = std::max(0.0, std::abs(effective) - kStallAngle);
    const double fade = excess / kStallWidth;
    return linear * std::exp(-fade * fade);
}

// Pressure coefficient on the outside of the skin, as a function of chord
// fraction. Crude but the right shape, which is all the load field needs:
//
//   upper surface  a suction peak just behind the leading edge, decaying to
//                  zero at the trailing edge, scaling with lift
//   lower surface  stagnation at the leading edge (Cp = 1, so the fabric
//                  there carries no load at all), easing to zero aft
//
// Cp is capped at 1 because nothing in a subsonic flow exceeds stagnation
// pressure, and a Cp above 1 would push the lower skin inward.
// Defined further down, next to the freestream helpers.
softwing::Vec3 canopyVelocityOf(const SimBody &sim);
softwing::Vec3 canopySpinOf(const SimBody &sim, softwing::Vec3 &centre);
double wingLiftCoefficient(double angleRadians);

double externalPressureCoefficient(double chordFraction,
                                   bool upperSurface,
                                   double liftCoefficient)
{
    const double station = std::clamp(chordFraction, 0.0, 1.0);
    const double aft = 1.0 - station;
    if (upperSurface) {
        // The peak sits just behind the leading edge, not on it. That
        // detail matters more than it looks: right at the leading edge the
        // skin faces forward, so suction placed there drags the whole wing
        // along its chord instead of lifting it. A shape that rises from
        // the stagnation line, peaks near 10% chord and trails off keeps
        // the load on surfaces that actually face up.
        constexpr double kPeakStation = 0.10;
        constexpr double kSuctionScale = 2.4;
        const double ratio = station / kPeakStation;
        const double peak = 0.75 * ratio * std::exp(1.0 - ratio);
        const double tail = 0.35 * aft;
        return -kSuctionScale * liftCoefficient * (peak + tail);
    }
    const double stagnation = std::pow(aft, 4.0);
    return std::min(1.0, stagnation + 0.3 * liftCoefficient * aft * aft);
}


void resetCellAirState(SimBody &sim)
{
    sim.cellAir = {};
    sim.cellAirDiagnostics = {};
    sim.cellPressure.clear();
    sim.cellRawPressure.clear();
    sim.cellLiveVolume.clear();
    sim.cellVolumeRatio.clear();
    sim.cellIntakeOpening.clear();
    sim.cellRamPressure.clear();
}

std::vector<CellAirVolume> measureCellVolumes(SimBody &sim)
{
    const std::size_t count = sim.cells.size();
    std::vector<CellAirVolume> volumes(count);
    sim.cellLiveVolume.assign(count, 0.0);
    sim.cellVolumeRatio.assign(count, 1.0);

    const auto &nodes = sim.body->nodes();
    const auto &triangles = sim.body->triangles();
    std::vector<double> ribArea(sim.ribLoopNodes.size(), 0.0);
    for (std::size_t rib = 0; rib < sim.ribLoopNodes.size(); ++rib) {
        softwing::Vec3 sum;
        const auto &loop = sim.ribLoopNodes[rib];
        for (std::size_t index = 0; index < loop.size(); ++index) {
            sum += cross(nodes[loop[index]].position,
                         nodes[loop[(index + 1) % loop.size()]].position);
        }
        ribArea[rib] = 0.5 * length(sum);
    }

    for (std::size_t cell = 0; cell < count; ++cell) {
        const SimCell &record = sim.cells[cell];
        const double section =
            0.5 * (ribArea[record.ribs[0]] + ribArea[record.ribs[1]]);
        const double spacing = length(
            nodes[sim.ribChords[record.ribs[1]].leadingNode].position
            - nodes[sim.ribChords[record.ribs[0]].leadingNode].position);
        const double rawProxy = section * spacing;
        const double fallbackProxy =
            record.restProxyVolume > 0.0
                    && std::isfinite(record.restProxyVolume)
                ? record.restProxyVolume
                : 1.0e-9;
        const double proxy = rawProxy > 0.0 && std::isfinite(rawProxy)
                                 ? rawProxy
                                 : fallbackProxy;
        const softwing::ClosedCellVolumeEstimate closed =
            softwing::estimateClosedCellVolume(
                nodes, triangles, record.skinFaces,
                sim.ribLoopNodes[record.ribs[0]],
                sim.ribLoopNodes[record.ribs[1]]);
        const double measured =
            closed.valid && closed.volume > 0.0
                    && std::isfinite(closed.volume)
                ? closed.volume
                : proxy;
        const double rest =
            record.restVolume > 0.0 && std::isfinite(record.restVolume)
                ? record.restVolume
                : fallbackProxy;
        const double floor = std::max(
            1.0e-9, kMinimumCellVolumeRatio * rest);
        const double live = std::max(measured, floor);
        volumes[cell].cubicMetres = live;
        sim.cellLiveVolume[cell] = live;
        sim.cellVolumeRatio[cell] =
            live / rest;
    }
    return volumes;
}

// One isothermal control volume per bay. Pressure is derived from finite air
// mass and the live closed-skin volume; the moving mouth and rib cross-ports
// are ordinary bidirectional orifices. A sealed squeeze therefore raises
// pressure, an expanding sealed cell loses pressure, and every internal
// transfer is equal-and-opposite. No visual-collapse pressure deletion or
// synthetic squeeze stamp is involved.
std::vector<double> advanceCellAirPressures(
    SimBody &sim,
    const std::vector<double> &ribPressure,
    const softwing::Vec3 &airVelocity,
    double crossPortGain,
    double timeStep)
{
    const std::size_t count = sim.cells.size();
    const std::vector<double> previousVolume = sim.cellLiveVolume;
    const bool hadFiniteMass = sim.cellAir.massKg.size() == count;
    const std::vector<CellAirVolume> volumes = measureCellVolumes(sim);
    std::vector<double> initialGauge(count, 0.0);
    for (std::size_t cell = 0; cell < count; ++cell) {
        const SimCell &record = sim.cells[cell];
        initialGauge[cell] =
            0.5 * (ribPressure[record.ribs[0]]
                   + ribPressure[record.ribs[1]]);
    }

    CellAirSettings settings;
    settings.cellTemperatureKelvin = kCellAirTemperature;
    settings.ambientAbsolutePressurePascal = kAtmosphericPressure;
    settings.substeps = kCellAirSubsteps;
    if (sim.cellAir.massKg.size() != count) {
        if (sim.cellPressure.size() == count) {
            initialGauge = sim.cellPressure;
        }
        initializeCellAirMass(sim.cellAir, volumes, initialGauge, settings);
    }
    const std::vector<double> massAtStepStart = sim.cellAir.massKg;
    double boundaryReservoirInflow = 0.0;

    sim.cellIntakeOpening.assign(count, 0.0);
    sim.cellRamPressure.assign(count, 0.0);
    std::vector<CellAirReservoirPort> intakes;
    intakes.reserve(count);
    const auto &nodes = sim.body->nodes();
    const auto &triangles = sim.body->triangles();
    for (std::size_t cell = 0; cell < count; ++cell) {
        const SimCell &record = sim.cells[cell];
        if (record.ventFaces.empty() || record.restVentAperture <= 0.0
            || record.restVentProjection <= 0.0) {
            continue;
        }
        softwing::Vec3 mouth;
        softwing::Vec3 weightedRelativeWind;
        double mouthArea = 0.0;
        double scoop = 0.0;
        for (const std::size_t face : record.ventFaces) {
            const auto &tri = triangles[face];
            const softwing::Vec3 area =
                0.5
                * cross(nodes[tri.b].position - nodes[tri.a].position,
                        nodes[tri.c].position - nodes[tri.a].position);
            const softwing::Vec3 relative =
                airVelocity
                - (nodes[tri.a].velocity + nodes[tri.b].velocity
                   + nodes[tri.c].velocity)
                      / 3.0;
            const double magnitude = length(area);
            mouth += area;
            weightedRelativeWind += magnitude * relative;
            mouthArea += magnitude;
            scoop -= dot(relative, area);
        }
        const double rawApproach =
            mouthArea > 0.0 ? length(weightedRelativeWind) / mouthArea : 0.0;
        const double approach =
            rawApproach >= 0.0 && std::isfinite(rawApproach)
                ? rawApproach
                : 0.0;
        const double rawIntakeSpeed =
            std::max(0.0, scoop) / record.restVentProjection;
        const double intakeSpeed =
            std::isfinite(rawIntakeSpeed)
                ? std::min(rawIntakeSpeed, approach)
                : 0.0;
        const double rawOpening = length(mouth) / record.restVentAperture;
        const double opening = std::isfinite(rawOpening)
                                   ? std::clamp(rawOpening, 0.0, 1.0)
                                   : 0.0;
        const double target = initialGauge[cell];
        const double ram = std::min(
            0.5 * kAirDensity * intakeSpeed * intakeSpeed,
            std::max(0.0, target));
        sim.cellIntakeOpening[cell] = opening;
        sim.cellRamPressure[cell] = ram;
        intakes.push_back({cell,
                           kAtmosphericPressure + ram,
                           kCellAirTemperature,
                           record.restVentAperture,
                           opening,
                           kCellFlowDischarge});
    }

    // A moving boundary pumps the bulk, ambient-density part of the air
    // through an open mouth even at zero pressure difference. The orifice
    // law below handles only pressure-driven flow; omitting this swept-volume
    // term makes a normal one-substep ballooning motion look like expansion
    // of a sealed vessel and invents kilopascal suction one frame late.
    // Closed mouths get no such correction: their finite mass is genuinely
    // trapped, so compression/expansion follows the gas law and resists the
    // fold.
    if (timeStep > 0.0 && hadFiniteMass
        && previousVolume.size() == count) {
        const double gasScale = settings.air.specificGasConstant
                                * settings.cellTemperatureKelvin;
        double sweptReservoirMass = 0.0;
        for (std::size_t cell = 0; cell < count; ++cell) {
            const double opening = sim.cellIntakeOpening[cell];
            if (!(opening > 0.0)) {
                continue;
            }
            const double reservoirDensity =
                (kAtmosphericPressure + sim.cellRamPressure[cell])
                / gasScale;
            double transfer = opening * reservoirDensity
                              * (volumes[cell].cubicMetres
                                 - previousVolume[cell]);
            transfer = std::max(
                transfer,
                settings.massFloorKg - sim.cellAir.massKg[cell]);
            sim.cellAir.massKg[cell] += transfer;
            sweptReservoirMass += transfer;
        }
        sim.cellAir.cumulativeReservoirInflowKg += sweptReservoirMass;
        boundaryReservoirInflow += sweptReservoirMass;
    }

    // The model does not resolve acoustic waves, fluttering mouth lips or
    // seam/fabric leakage. It therefore cannot let a one-frame geometric
    // impulse trap tens of kilopascals in a low-pressure ram-air canopy.
    // Treat a live, non-pinched intake as a pressure-relief path only outside
    // the aerodynamic envelope; ordinary pressure is still governed by the
    // measured aperture/orifice above. A pinched mouth or a test cell with no
    // atmosphere boundary remains genuinely sealed.
    if (timeStep > 0.0) {
        const double gasScale = settings.air.specificGasConstant
                                * settings.cellTemperatureKelvin;
        double reliefMass = 0.0;
        for (std::size_t cell = 0; cell < count; ++cell) {
            if (sim.cells[cell].ventFaces.empty()
                || !(sim.cellIntakeOpening[cell] > 0.0)) {
                continue;
            }
            const double scale = std::max(
                initialGauge[cell], kCellMinimumPressureEnvelopePascal);
            const double currentGauge =
                sim.cellAir.massKg[cell] * gasScale
                    / volumes[cell].cubicMetres
                - kAtmosphericPressure;
            const double boundedGauge = std::clamp(
                currentGauge, kCellMinimumGaugePressureRatio * scale,
                kCellMaximumGaugePressureRatio * scale);
            // The unresolved fast-relief authority vanishes continuously as
            // the live mouth pinches shut; an epsilon-sized aperture must not
            // behave like a fully open intake.
            const double relievedGauge =
                currentGauge
                + sim.cellIntakeOpening[cell]
                      * (boundedGauge - currentGauge);
            if (relievedGauge == currentGauge) {
                continue;
            }
            const double relievedMass =
                (kAtmosphericPressure + relievedGauge)
                * volumes[cell].cubicMetres / gasScale;
            reliefMass += relievedMass - sim.cellAir.massKg[cell];
            sim.cellAir.massKg[cell] = relievedMass;
        }
        sim.cellAir.cumulativeReservoirInflowKg += reliefMass;
        boundaryReservoirInflow += reliefMass;
    }

    if (timeStep > 0.0) {
        std::vector<CellAirCrossPort> crossPorts;
        crossPorts.reserve(count > 0 ? count - 1 : 0);
        for (std::size_t cell = 0; cell + 1 < count; ++cell) {
            const double area = std::max(0.0, crossPortGain)
                                * sim.cells[cell].portAreaToNext;
            if (area > 0.0 && std::isfinite(area)) {
                crossPorts.push_back(
                    {cell, cell + 1, area, 1.0, kCellFlowDischarge});
            }
        }

        sim.cellAirDiagnostics = advanceCellAirMass(
            sim.cellAir, volumes, intakes, crossPorts,
            timeStep, settings);
        // The generic orifice solver owns its own reservoir diagnostics.
        // Include moving-boundary and pressure-envelope exchange so this
        // public report covers the complete pneumatic update.
        sim.cellAirDiagnostics.reservoirInflowKg +=
            boundaryReservoirInflow;
        sim.cellAirDiagnostics.cumulativeReservoirInflowKg =
            sim.cellAir.cumulativeReservoirInflowKg;
        for (std::size_t cell = 0; cell < count; ++cell) {
            sim.cellAirDiagnostics.cells[cell].netMassRateKgPerSecond =
                (sim.cellAir.massKg[cell] - massAtStepStart[cell])
                / timeStep;
        }
    } else {
        // A zero-duration call synchronizes geometry-dependent diagnostics
        // and the pressure stamp without advancing air mass. stepSimulation
        // uses it before the first XPBD substep; each completed substep then
        // receives one real dt/N pneumatic update, including the final one.
        const std::vector<double> raw =
            cellAirGaugePressures(sim.cellAir, volumes, settings);
        sim.cellAirDiagnostics = {};
        sim.cellAirDiagnostics.cells.resize(count);
        sim.cellAirDiagnostics.finiteCellMassKg =
            std::accumulate(sim.cellAir.massKg.begin(),
                            sim.cellAir.massKg.end(), 0.0);
        sim.cellAirDiagnostics.cumulativeReservoirInflowKg =
            sim.cellAir.cumulativeReservoirInflowKg;
        sim.cellAirDiagnostics.massResidualKg =
            sim.cellAirDiagnostics.finiteCellMassKg
            - sim.cellAir.referenceFiniteMassKg
            - sim.cellAir.cumulativeReservoirInflowKg;
        for (std::size_t cell = 0; cell < count; ++cell) {
            CellAirCellDiagnostics &diagnostic =
                sim.cellAirDiagnostics.cells[cell];
            diagnostic.massKg = sim.cellAir.massKg[cell];
            diagnostic.volumeCubicMetres = volumes[cell].cubicMetres;
            diagnostic.gaugePressurePascal = raw[cell];
            diagnostic.absolutePressurePascal =
                kAtmosphericPressure + raw[cell];
        }
    }
    sim.cellPressure.resize(count);
    sim.cellRawPressure.resize(count);
    for (std::size_t cell = 0; cell < count; ++cell) {
        const double raw =
            sim.cellAirDiagnostics.cells[cell].gaugePressurePascal;
        sim.cellRawPressure[cell] = raw;
        const double volumeDistress = std::clamp(
            (0.95 - sim.cellVolumeRatio[cell]) / 0.20, 0.0, 1.0);
        const double openingDistress = std::clamp(
            (0.80 - sim.cellIntakeOpening[cell]) / 0.50, 0.0, 1.0);
        const double target = initialGauge[cell];
        const double ramDistress =
            target > 1.0e-9
                ? std::clamp(
                      1.0 - sim.cellRamPressure[cell] / target,
                      0.0, 1.0)
                : 0.0;
        const double distress = std::max(
            {volumeDistress, openingDistress, ramDistress});
        // Preserve the calibrated ram field while the bay is healthy. The
        // finite-mass result takes authority continuously as true volume,
        // aperture or ram recovery is lost; this keeps ordinary fabric
        // breathing out of the stiff gas/cloth feedback without hiding a
        // real collapse.
        sim.cellPressure[cell] =
            (1.0 - distress) * target + distress * raw;
    }
    return sim.cellPressure;
}

}  // namespace

softwing::Vec3 referenceFlowVelocity(const SimBody &sim,
                                     const SimControls &controls)
{
    const double airspeed = std::sqrt(
        2.0 * std::max(0.0, controls.pressurePascal) / kAirDensity);
    const double angle =
        (controls.freeFlight ? 0.0 : controls.angleOfAttackDegrees)
        * kDegreesToRadians;
    return airspeed
           * rotateAbout(sim.restChordDirection, sim.restSpanAxis, angle);
}

softwing::Vec3 airVelocityWorld(const SimBody &sim,
                                const SimControls &controls)
{
    if (controls.freeFlight) {
        return controls.ambientAirVelocityWorld;
    }
    return controls.ambientAirVelocityWorld
           + referenceFlowVelocity(sim, controls);
}

softwing::Vec3 relativeAirVelocity(
    const softwing::Vec3 &airVelocity,
    const softwing::Vec3 &surfaceVelocity)
{
    return airVelocity - surfaceVelocity;
}

void applyPressureForDuration(SimBody &sim,
                              const SimControls &controls,
                              double cellTimeStep)
{
    if (!sim.body) {
        return;
    }
    const double referenceDynamicPressure = controls.pressurePascal;
    const softwing::Vec3 airVelocity = airVelocityWorld(sim, controls);
    const softwing::Vec3 bulkSurfaceVelocity =
        controls.freeFlight ? canopyVelocityOf(sim) : softwing::Vec3{};
    const softwing::Vec3 bulkRelative =
        relativeAirVelocity(airVelocity, bulkSurfaceVelocity);
    const bool legacyTunnelAir =
        !controls.freeFlight
        && lengthSquared(controls.ambientAirVelocityWorld) == 0.0;
    const double dynamicPressure =
        legacyTunnelAir
            ? referenceDynamicPressure
            : std::min(0.5 * kAirDensity * lengthSquared(bulkRelative),
                       kMaximumDynamicPressureRatio
                           * std::max(0.0, referenceDynamicPressure));

    // No rib loops, no chord to hang a distribution off: fall back to the
    // uniform field, which is what this used to be everywhere.
    if (sim.faceAero.empty() || sim.ribChords.empty()) {
        sim.body->setUniformPressureDifference(
            sim.body->surfaceGroup(0, sim.skinTriangleCount),
            dynamicPressure);
        // Nothing to derive a physical floor from; the retrim falls back
        // to its own bound.
        sim.facePressureFloor.clear();
        sim.faceInteriorPressure.clear();
        sim.faceDynamicPressure.clear();
        sim.facePriorExternalCp.clear();
        sim.faceRetrimPreferredCp.clear();
        sim.faceAppliedExternalCp.clear();
        return;
    }

    // The air the wing is flying through, as a velocity rather than a
    // direction. That distinction is the difference between a wing and a
    // tumbling bag: a section that is moving meets the air at a different
    // angle and a different speed than one that is not, so every load here
    // depends on how the wing is moving. Without it there is no
    // aerodynamic damping anywhere in the model — nothing resists a pitch
    // rate — and the system pendulums until it goes over the top.
    // Positive angle of attack means air from BELOW the chord: the
    // downstream direction is the rest chord rotated UP by the slider
    // angle. The convention ran the other way for a long time and was
    // statically self-consistent — but dynamically it inverted the
    // fundamental feedback (a sinking wing lost lift instead of gaining
    // it), which made every free-flight attempt diverge.
    // In free flight the slider supplies only the reference load/launch
    // speed. The air mass itself is ambientAirVelocityWorld; angle of attack
    // comes from the wing's motion and attitude. The tunnel adds its
    // prescribed q-derived flow at the slider angle.

    const auto &nodes = sim.body->nodes();

    // The system's bulk velocity, not each section's own. Using the local
    // node velocity here looks more refined and is catastrophic: pressure
    // accelerates the fabric, fabric moving downwind sees less relative
    // wind, less relative wind means less pressure, and the canopy talks
    // itself flat. Measured on gnuC2 it took the span from 10.4 m to 5.2 m.
    // Bulk motion carries the damping that free flight needs without
    // closing that loop.
    //
    // Pinned, there is no bulk motion to account for and the canopy's own
    // sloshing is not it: feeding fabric velocity back into the load
    // modulates the pressure that is causing the sloshing. So this is a
    // free-flight term only, and with it off the field is exactly the fixed
    // freestream it was verified against.
    softwing::Vec3 systemVelocity;
    softwing::Vec3 spinRate;
    softwing::Vec3 spinCentre;
    if (controls.freeFlight) {
        systemVelocity = canopyVelocityOf(sim);
        spinRate = canopySpinOf(sim, spinCentre);
    }

    sim.ribLiftCoefficient.assign(sim.ribChords.size(), 0.0);
    std::vector<double> &ribLift = sim.ribLiftCoefficient;
    std::vector<double> ribPressure(sim.ribChords.size(), dynamicPressure);

    // RibChord::spanAxis is authored in the rest frame. Carry it with the
    // canopy before projecting the live chord and wind into a section plane;
    // using the stored world-space vector after a yaw made a rigidly rotated
    // wing aerodynamically different from itself, which manufactured
    // crossflow as a turn developed.
    const WingAeroSample wingFrame =
        controls.freeFlight ? sampleWingAero(sim, controls)
                            : WingAeroSample{};
    const softwing::Vec3 restSpan = normalized(sim.restSpanAxis);
    const softwing::Vec3 restChord = normalized(
        sim.restChordDirection
        - dot(sim.restChordDirection, restSpan) * restSpan);
    const softwing::Vec3 restUp = normalized(cross(restSpan, restChord));
    const softwing::Vec3 liveSpan =
        wingFrame.valid ? wingFrame.spanAxis : restSpan;
    const softwing::Vec3 liveChord =
        wingFrame.valid ? wingFrame.chordDirection : restChord;
    const softwing::Vec3 liveUp = normalized(cross(liveSpan, liveChord));
    for (std::size_t index = 0; index < sim.ribChords.size(); ++index) {
        const RibChord &rib = sim.ribChords[index];
        const softwing::Vec3 chord = nodes[rib.trailingNode].position
                                     - nodes[rib.leadingNode].position;
        if (length(chord) <= 0.0) {
            continue;
        }
        // The wind THIS section meets, not the wind the wing as a whole
        // meets. A rolling wing has one tip descending into the air and
        // the other rising out of it; a yawing one has a tip running
        // forward and a tip running back. Both give the outer sections a
        // different angle AND a different speed from the inner ones, and
        // that difference is the entirety of a wing's roll and yaw
        // damping — without it an asymmetric input diverges instead of
        // settling into a turn, which is what folded the wing at the same
        // station every time a brake was held.
        //
        // Taken from the canopy's rigid-body spin (see canopySpinOf), so
        // it carries the wing's rotation and nothing of the fabric's own
        // motion. The pressure field's net force and pitch moment are
        // both cancelled downstream by the polar pass; its ROLL and YAW
        // moments are deliberately not — so this reaches the wing as
        // exactly the damping it was missing and cannot disturb the
        // trimmed force balance.
        const softwing::Vec3 station =
            nodes[rib.leadingNode].position + 0.25 * chord;
        softwing::Vec3 spin = cross(spinRate, station - spinCentre);
        // A tumbling transient must not hand a section a wind of its own
        // invention: capped at the airspeed the wing is flying at, so a
        // section can at most double or cancel its own wind.
        const double spinSpeed = length(spin);
        const double spinLimit =
            kMaximumSpinWindRatio
            * length(relativeAirVelocity(airVelocity, systemVelocity));
        if (spinSpeed > spinLimit && spinSpeed > 0.0) {
            spin = (spinLimit / spinSpeed) * spin;
        }
        const softwing::Vec3 relativeWind =
            relativeAirVelocity(airVelocity, systemVelocity + spin);

        // Both vectors flattened into the section's own plane, so the angle
        // measured is pitch and not some part of the wing's sweep or arc.
        softwing::Vec3 axis = rib.spanAxis;
        if (controls.freeFlight
            && length(restSpan) > 0.0 && length(restChord) > 0.0
            && length(restUp) > 0.0 && length(liveSpan) > 0.0
            && length(liveChord) > 0.0 && length(liveUp) > 0.0) {
            axis = normalized(
                dot(rib.spanAxis, restSpan) * liveSpan
                + dot(rib.spanAxis, restChord) * liveChord
                + dot(rib.spanAxis, restUp) * liveUp);
        }
        const softwing::Vec3 attitude =
            nodes[rib.referenceNode].position
            - nodes[rib.leadingNode].position;
        const softwing::Vec3 attitudeInPlane =
            attitude - dot(attitude, axis) * axis;
        const softwing::Vec3 fullChordInPlane =
            chord - dot(chord, axis) * axis;
        const softwing::Vec3 chordInPlane = normalized(
            controls.freeFlight && length(attitudeInPlane) > 0.0
                ? rotateAbout(attitudeInPlane, axis,
                              rib.attitudeOffsetRadians)
                : fullChordInPlane);
        const softwing::Vec3 windInPlane =
            relativeWind - dot(relativeWind, axis) * axis;
        const double windSpeed = length(windInPlane);
        if (length(chordInPlane) <= 0.0 || windSpeed <= 1.0e-6) {
            ribPressure[index] = 0.0;
            continue;
        }
        const softwing::Vec3 windDirection = windInPlane / windSpeed;
        const double alongWind = dot(chordInPlane, windDirection);
        // Positive when the wind comes from below the section's chord.
        const double acrossWind =
            dot(cross(chordInPlane, windDirection), axis);
        ribLift[index] = sectionLiftCoefficient(
            std::atan2(acrossWind, alongWind));
        // The pressure scales with the FULL relative wind, not the part of
        // it lying in the section's plane. The in-plane component sets the
        // angle the section flies at and nothing else; the cell behind it
        // is fed by a ram intake that does not care which way the air came
        // from. Using the in-plane speed here charges an arced wing's tips
        // -- whose section planes are tilted well out of the flow -- a
        // fraction of the pressure they should carry, and the wing loses
        // most of its lift and a third of its span.
        //
        // Capped so a section flung about during a transient cannot answer
        // with an unbounded load.
        const double relativeSpeed = length(relativeWind);
        ribPressure[index] =
            std::min(0.5 * kAirDensity * relativeSpeed * relativeSpeed,
                     kMaximumDynamicPressureRatio
                         * referenceDynamicPressure);
    }

    // The interior side of every face. With the cell model on, finite mass
    // and live volume produce the raw gas pressure. The pressure applied to
    // the cloth retains the calibrated ram field as a healthy-flight prior,
    // then continuously hands authority to that gas state as volume, mouth
    // opening or ram recovery is lost. With the model off — or when the mesh
    // gave us no cells — the interior uses the old blanket ram pressure.
    const bool cellsActive =
        controls.cellPressureModel && !sim.cells.empty();
    std::vector<double> interior;
    if (cellsActive) {
        // The air itself, not the bulk relative wind: the intakes subtract
        // each vent face's own velocity, which is how a nose sweeping
        // backwards through a pitch-up still rams itself full. That is a
        // mass flow, not an aerodynamic load, so it does not re-open the
        // per-node feedback the pressure field has to stay clear of.
        interior = advanceCellAirPressures(
            sim,
            ribPressure,
            airVelocity,
            std::max(0.0, controls.crossPortGain),
            cellTimeStep);
    }
    sim.facePressureFloor.assign(sim.skinTriangleCount, 0.0);
    sim.faceInteriorPressure.assign(sim.skinTriangleCount, 0.0);
    sim.faceDynamicPressure.assign(sim.skinTriangleCount, 0.0);
    sim.facePriorExternalCp.assign(sim.skinTriangleCount, 0.0);
    sim.faceRetrimPreferredCp.assign(sim.skinTriangleCount, 0.0);
    sim.faceAppliedExternalCp.assign(sim.skinTriangleCount, 0.0);
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const FaceAero &aero = sim.faceAero[face];
        const double coefficient = externalPressureCoefficient(
            aero.chordFraction, aero.upperSurface, ribLift[aero.rib]);
        const double faceInterior =
            cellsActive ? interior[aero.cell] : ribPressure[aero.rib];
        const double faceDynamic = ribPressure[aero.rib];
        sim.faceInteriorPressure[face] = faceInterior;
        sim.faceDynamicPressure[face] = faceDynamic;
        sim.facePriorExternalCp[face] = coefficient;
        sim.faceAppliedExternalCp[face] = coefficient;
        // The interior this face has, minus the most the air outside it
        // can ever push back with. Cp is capped at 1 just above, so the
        // stamped value below is always at or over this; it is the retrim
        // that has to be held to it.
        sim.facePressureFloor[face] = faceInterior - faceDynamic;
        // The outside of the face is q·Cp; only the difference across the
        // fabric loads it, which is why there is no separate ambient
        // anywhere here. The legacy expression is kept verbatim on the
        // legacy path — q·(1−Cp) and q − q·Cp round differently in IEEE
        // double, and the bench's pose checksums notice.
        sim.body->setFacePressureDifference(
            face,
            cellsActive
                ? interior[aero.cell]
                      - ribPressure[aero.rib] * coefficient
                : ribPressure[aero.rib] * (1.0 - coefficient));
    }
}

void applyPressure(SimBody &sim, const SimControls &controls)
{
    applyPressureForDuration(sim, controls, simulationTimeStep);
}

void advanceAndRestampCellInterior(SimBody &sim,
                                   const SimControls &controls,
                                   double timeStep)
{
    if (!sim.body || !controls.cellPressureModel || sim.cells.empty()
        || sim.faceDynamicPressure.size() != sim.skinTriangleCount
        || sim.faceAppliedExternalCp.size() != sim.skinTriangleCount) {
        return;
    }
    std::vector<double> ribPressure(sim.ribChords.size(), 0.0);
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        ribPressure[sim.faceAero[face].rib] =
            sim.faceDynamicPressure[face];
    }
    const std::vector<double> interior = advanceCellAirPressures(
        sim, ribPressure, airVelocityWorld(sim, controls),
        std::max(0.0, controls.crossPortGain), timeStep);
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const FaceAero &aero = sim.faceAero[face];
        const double faceInterior = interior[aero.cell];
        const double faceDynamic = sim.faceDynamicPressure[face];
        sim.faceInteriorPressure[face] = faceInterior;
        sim.facePressureFloor[face] = faceInterior - faceDynamic;
        sim.body->setFacePressureDifference(
            face,
            faceInterior
                - faceDynamic * sim.faceAppliedExternalCp[face]);
    }
}

namespace {

softwing::Vec3 systemVelocityOf(const SimBody &sim)
{
    softwing::Vec3 velocity;
    double mass = 0.0;
    for (const softwing::Node &node : sim.body->nodes()) {
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double nodeMass = 1.0 / node.inverseMass;
        velocity += nodeMass * node.velocity;
        mass += nodeMass;
    }
    return mass > 0.0 ? velocity / mass : velocity;
}

// The canopy's own bulk velocity. This — not the system's — is what the
// air meets: measured against the system mean (mostly the pilot), the
// canopy's pendulum swing was invisible to the aerodynamics, which
// removed the damping that keeps a real canopy from whipping over on its
// lines.
softwing::Vec3 canopyVelocityOf(const SimBody &sim)
{
    const auto &nodes = sim.body->nodes();
    const std::size_t count =
        sim.canopyNodeCount > 0 ? sim.canopyNodeCount : nodes.size();
    softwing::Vec3 velocity;
    double mass = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const softwing::Node &node = nodes[index];
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double nodeMass = 1.0 / node.inverseMass;
        velocity += nodeMass * node.velocity;
        mass += nodeMass;
    }
    return mass > 0.0 ? velocity / mass : velocity;
}

// The canopy's rotation rate, fitted as if it were a RIGID body: solve
// I·omega = L for the angular momentum L and inertia I of the canopy
// nodes about their own centroid. That the fit is rigid is the whole
// safety argument for using it: a rigid body has no breathing mode, so
// none of the fabric's own motion reaches the pressure field through it,
// and the per-node feedback that once talked the canopy flat (span 10.4 m
// to 5.2 m, measured) cannot come back this way. What DOES reach it is
// the part a wing must have — a rolling wing's tips move opposite ways
// through the air, and that is where roll and yaw damping come from.
softwing::Vec3 canopySpinOf(const SimBody &sim, softwing::Vec3 &centre)
{
    const auto &nodes = sim.body->nodes();
    const std::size_t count =
        sim.canopyNodeCount > 0 ? sim.canopyNodeCount : nodes.size();
    softwing::Vec3 middle;
    softwing::Vec3 mean;
    double mass = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const softwing::Node &node = nodes[index];
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double nodeMass = 1.0 / node.inverseMass;
        middle += nodeMass * node.position;
        mean += nodeMass * node.velocity;
        mass += nodeMass;
    }
    if (mass <= 0.0) {
        centre = {};
        return {};
    }
    middle = middle / mass;
    mean = mean / mass;
    centre = middle;

    softwing::Vec3 momentum;
    double inertia[3][3] = {};
    for (std::size_t index = 0; index < count; ++index) {
        const softwing::Node &node = nodes[index];
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double nodeMass = 1.0 / node.inverseMass;
        const softwing::Vec3 arm = node.position - middle;
        momentum += nodeMass * cross(arm, node.velocity - mean);
        const double armSquared = lengthSquared(arm);
        const double component[3] = {arm.x, arm.y, arm.z};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                inertia[row][column] +=
                    nodeMass
                    * ((row == column ? armSquared : 0.0)
                       - component[row] * component[column]);
            }
        }
    }

    // Cramer, with a determinant guard: a canopy squashed into a plane
    // has a singular inertia tensor and no defined rotation about the
    // degenerate axis.
    const double determinant =
        inertia[0][0]
            * (inertia[1][1] * inertia[2][2] - inertia[1][2] * inertia[2][1])
        - inertia[0][1]
              * (inertia[1][0] * inertia[2][2] - inertia[1][2] * inertia[2][0])
        + inertia[0][2]
              * (inertia[1][0] * inertia[2][1] - inertia[1][1] * inertia[2][0]);
    const double scale = std::abs(inertia[0][0]) + std::abs(inertia[1][1])
                         + std::abs(inertia[2][2]);
    if (std::abs(determinant) <= 1.0e-9 * scale * scale * scale) {
        return {};
    }
    const double right[3] = {momentum.x, momentum.y, momentum.z};
    double solution[3] = {};
    for (int axis = 0; axis < 3; ++axis) {
        double swapped[3][3];
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                swapped[row][column] = column == axis ? right[row]
                                                      : inertia[row][column];
            }
        }
        solution[axis] =
            (swapped[0][0]
                 * (swapped[1][1] * swapped[2][2]
                    - swapped[1][2] * swapped[2][1])
             - swapped[0][1]
                   * (swapped[1][0] * swapped[2][2]
                      - swapped[1][2] * swapped[2][0])
             + swapped[0][2]
                   * (swapped[1][0] * swapped[2][1]
                      - swapped[1][1] * swapped[2][0]))
            / determinant;
    }
    return {solution[0], solution[1], solution[2]};
}

}  // namespace

WingAeroSample sampleWingAero(const SimBody &sim,
                              const SimControls &controls)
{
    WingAeroSample sample;
    if (!sim.body || sim.ribChords.empty() || sim.faceAero.empty()
        || !(sim.planformArea > 0.0)) {
        return sample;
    }

    // The polar must see the canopy's own motion whenever it is the one
    // applying system-level force — free flight AND the tunnel's flight
    // load. A tethered canopy still swings on its lines, and a ±2 kN
    // force that follows the swinging angle of attack through a 0.25 s
    // lag while ignoring the swing velocity is a pumped oscillator: with
    // the subtraction missing, the tunnel wing wound itself up to 8 m/s
    // of agitation and pitched right over. At equilibrium the canopy is
    // still and the term vanishes, so the tunnel condition itself is
    // untouched.
    const softwing::Vec3 surfaceVelocity =
        controls.freeFlight || controls.flightLoad
            ? canopyVelocityOf(sim)
            : softwing::Vec3{};
    const softwing::Vec3 relative = relativeAirVelocity(
        airVelocityWorld(sim, controls), surfaceVelocity);
    const double speed = length(relative);
    if (speed <= 1.0e-6) {
        return sample;
    }
    sample.airspeed = speed;
    sample.windDirection = relative / speed;
    sample.dynamicPressure =
        std::min(0.5 * kAirDensity * speed * speed,
                 kMaximumDynamicPressureRatio
                     * std::max(0.0, controls.pressurePascal));

    const auto &nodes = sim.body->nodes();

    // Live span axis between the tip ribs' leading edges. spanTipRibs keeps
    // the low-to-high material ordering established in the rest pose, so the
    // vector already has a stable sign and must be allowed to rotate through
    // every world heading. Reorienting it against the fixed rest/world axis
    // flips span, lift and alpha at 90 degrees and makes a 360-degree turn
    // impossible.
    softwing::Vec3 spanAxis =
        nodes[sim.ribChords[sim.spanTipRibs[1]].leadingNode].position
        - nodes[sim.ribChords[sim.spanTipRibs[0]].leadingNode].position;
    if (length(spanAxis) <= 1.0e-6) {
        spanAxis = sim.restSpanAxis;
    }
    spanAxis = normalized(spanAxis);
    if (!controls.freeFlight
        && dot(spanAxis, sim.restSpanAxis) < 0.0) {
        // Preserve the pinned/tunnel path's historical orientation exactly.
        spanAxis = -1.0 * spanAxis;
    }
    sample.spanAxis = spanAxis;

    // Mean live attitude line, length-weighted so a tip rib flapping about
    // cannot steer the whole wing's angle, flattened into the plane normal
    // to the span so sweep and arc do not contaminate the pitch
    // measurement.
    //
    // Measured leading edge to the 40%-chord reference node, NOT to the
    // trailing edge. A brake pulls the trailing edge down; off the full
    // chord that reads as the whole wing pitching up, and the polar
    // answers with more lift, more induced drag, less airspeed and
    // therefore a still higher angle — a loop that stalled the wing a few
    // seconds after a 20 cm pull with the pilot's hand held still. The
    // forward 40% is fabric the brake cannot move, so this line carries
    // the wing's attitude and none of the input. In free flight the brake's
    // real effect remains in the pressure force on the deflected live skin;
    // the pinned compatibility path separately retains its prescribed polar.
    softwing::Vec3 chordSum;
    for (const RibChord &rib : sim.ribChords) {
        // Scaled back to a full chord so the angle is unchanged but the
        // length weighting still favours the big central ribs.
        chordSum += (nodes[rib.referenceNode].position
                     - nodes[rib.leadingNode].position)
                    / kAttitudeReferenceStation;
    }
    // Rotated back onto the chord by the rest-pose offset between the two
    // lines, so this reads the same angle the full chord read in the rest
    // pose — and keeps reading the wing's attitude, not the pilot's hand,
    // once a brake is pulled.
    const softwing::Vec3 chordInPlane = rotateAbout(
        chordSum - dot(chordSum, spanAxis) * spanAxis,
        spanAxis,
        sim.attitudeOffsetRadians);
    const softwing::Vec3 windInPlane =
        relative - dot(relative, spanAxis) * spanAxis;
    if (length(chordInPlane) <= 1.0e-6 || length(windInPlane) <= 1.0e-6) {
        return sample;
    }
    const softwing::Vec3 chordDirection = normalized(chordInPlane);
    sample.chordDirection = chordDirection;
    const softwing::Vec3 windPlaneDirection = normalized(windInPlane);
    const double alongWind = dot(chordDirection, windPlaneDirection);
    // Positive when the wind comes from below the chord — the physical
    // convention, and the one that makes sinking raise the angle of
    // attack (negative feedback) rather than lower it.
    const double acrossWind =
        dot(cross(chordDirection, windPlaneDirection), spanAxis);
    sample.alphaRadians = std::atan2(acrossWind, alongWind);

    // The same section law the pressure field uses — camber offset, stall
    // roll-off and all — knocked down by the finite-wing factor, so the
    // wing-level lift slope is the three-dimensional one. The stall
    // roll-off doubles as pitch stability: a wing pitched to a silly angle
    // stops pulling instead of pulling harder.
    const double aspectRatio = std::max(1.0, sim.aspectRatio);
    const double finiteWing =
        1.0 / (1.0 + 2.0 / (aspectRatio * kSpanEfficiency));
    sample.liftCoefficient =
        finiteWing * wingLiftCoefficient(sample.alphaRadians);
    sample.dragCoefficient =
        kParasiticDragCoefficient
        + sample.liftCoefficient * sample.liftCoefficient
              / (kPi * aspectRatio * kSpanEfficiency);

    // Lift is normal to the relative wind, in the plane the wind and the
    // wing's up direction share. With the span oriented +x and the wind
    // running chordwise +y this is +z, and the sign of the lift comes from
    // the coefficient, not from flipping this axis.
    const softwing::Vec3 lift = cross(spanAxis, sample.windDirection);
    if (length(lift) <= 1.0e-6) {
        return sample;
    }
    sample.liftDirection = normalized(lift);
    sample.valid = true;
    return sample;
}

FlightFrameSample sampleFlightFrame(const SimBody &sim,
                                    const SimControls &controls)
{
    FlightFrameSample frame;
    const WingAeroSample aero = sampleWingAero(sim, controls);
    if (!aero.valid) {
        return frame;
    }

    // The mesh chord and relative-air vectors both run downstream. Physical
    // forward and travel through the air point the other way. Keeping this
    // conversion here prevents callers from silently treating the historical
    // +Y mesh convention as the nose direction.
    frame.forwardDirection = -1.0 * aero.chordDirection;
    frame.travelVelocity = -aero.airspeed * aero.windDirection;
    frame.spanAxis = aero.spanAxis;
    frame.upDirection = normalized(
        cross(aero.spanAxis, aero.chordDirection));
    if (length(frame.upDirection) <= 1.0e-6) {
        return frame;
    }

    frame.forwardSpeed = dot(frame.travelVelocity,
                             frame.forwardDirection);
    frame.spanwiseSpeed = dot(frame.travelVelocity, frame.spanAxis);
    frame.sideslipRadians = std::atan2(frame.spanwiseSpeed,
                                       frame.forwardSpeed);

    const auto heading = [](const softwing::Vec3 &direction) {
        // The unrotated canopy flies toward -Y. Positive heading turns from
        // there toward solver +X, matching the planform's span convention.
        return std::atan2(direction.x, -direction.y);
    };
    frame.noseHeadingRadians = heading(frame.forwardDirection);
    frame.courseHeadingRadians = heading(frame.travelVelocity);

    // Rotating the +X span about the downstream +Y chord lowers the +X tip
    // and tilts lift toward +X. That is a positive coordinated turn.
    frame.bankRadians = std::atan2(-frame.spanAxis.z,
                                   frame.upDirection.z);
    frame.valid = true;
    return frame;
}

HalfAeroKinematics sampleHalfAeroKinematics(
    const SimBody &sim, const WingAeroSample &sample)
{
    HalfAeroKinematics result;
    if (!sim.body || !sample.valid || !(sample.airspeed > 1.0e-6)
        || sim.ribHalf.size() != sim.ribChords.size()
        || sim.ribPlanformArea.size() != sim.ribChords.size()
        || !(sim.halfPlanformArea[0] > 0.0)
        || !(sim.halfPlanformArea[1] > 0.0)) {
        return result;
    }

    const softwing::Vec3 restSpan = normalized(sim.restSpanAxis);
    const softwing::Vec3 restChord = normalized(
        sim.restChordDirection
        - dot(sim.restChordDirection, restSpan) * restSpan);
    const softwing::Vec3 restUp = normalized(cross(restSpan, restChord));
    const softwing::Vec3 liveSpan = normalized(sample.spanAxis);
    const softwing::Vec3 liveChord = normalized(
        sample.chordDirection
        - dot(sample.chordDirection, liveSpan) * liveSpan);
    const softwing::Vec3 liveUp = normalized(cross(liveSpan, liveChord));
    if (length(restSpan) <= 0.0 || length(restChord) <= 0.0
        || length(restUp) <= 0.0 || length(liveSpan) <= 0.0
        || length(liveChord) <= 0.0 || length(liveUp) <= 0.0) {
        return result;
    }

    const auto &nodes = sim.body->nodes();
    softwing::Vec3 spinCentre;
    const softwing::Vec3 spinRate = canopySpinOf(sim, spinCentre);
    const softwing::Vec3 commonFlow =
        sample.airspeed
        * normalized(sample.windDirection
                     - dot(sample.windDirection, liveSpan) * liveSpan);
    if (length(commonFlow) <= 1.0e-6) {
        return result;
    }
    double halfAlpha[2] = {0.0, 0.0};
    double measuredArea[2] = {0.0, 0.0};
    for (int half = 0; half < 2; ++half) {
        softwing::Vec3 centre;
        double centreWeight = 0.0;
        for (std::size_t index = 0; index < sim.ribChords.size(); ++index) {
            if (sim.ribHalf[index] != static_cast<std::uint8_t>(half)) {
                continue;
            }
            const RibChord &rib = sim.ribChords[index];
            const double weight = sim.ribPlanformArea[index];
            centre += weight
                      * (nodes[rib.leadingNode].position
                         + 0.25
                               * (nodes[rib.trailingNode].position
                                  - nodes[rib.leadingNode].position));
            centreWeight += weight;
        }
        if (!(centreWeight > 0.0)) {
            return result;
        }
        centre /= centreWeight;

        softwing::Vec3 localVelocity =
            cross(spinRate, centre - spinCentre);
        const double localSpeed = length(localVelocity);
        const double localLimit = kMaximumSpinWindRatio * sample.airspeed;
        if (localSpeed > localLimit && localSpeed > 0.0) {
            localVelocity = (localLimit / localSpeed) * localVelocity;
        }
        const softwing::Vec3 halfFlow =
            sample.airspeed * sample.windDirection - localVelocity;
        result.dynamicPressureRatio[half] =
            lengthSquared(halfFlow) / (sample.airspeed * sample.airspeed);

        for (std::size_t index = 0; index < sim.ribChords.size(); ++index) {
            if (sim.ribHalf[index] != static_cast<std::uint8_t>(half)) {
                continue;
            }
            const double weight = sim.ribPlanformArea[index];
            if (!(weight > 0.0)) {
                continue;
            }
            const softwing::Vec3 restSection =
                normalized(sim.ribChords[index].spanAxis);
            softwing::Vec3 liveSection =
                dot(restSection, restSpan) * liveSpan
                + dot(restSection, restChord) * liveChord
                + dot(restSection, restUp) * liveUp;
            if (length(liveSection) <= 1.0e-6) {
                continue;
            }
            liveSection = normalized(liveSection);
            const softwing::Vec3 sectionChord =
                liveChord - dot(liveChord, liveSection) * liveSection;
            const softwing::Vec3 sectionFlow =
                halfFlow - dot(halfFlow, liveSection) * liveSection;
            const softwing::Vec3 baselineFlow =
                commonFlow - dot(commonFlow, liveSection) * liveSection;
            if (length(sectionChord) <= 1.0e-6
                || length(sectionFlow) <= 1.0e-6
                || length(baselineFlow) <= 1.0e-6) {
                continue;
            }
            const softwing::Vec3 chord = normalized(sectionChord);
            const softwing::Vec3 flow = normalized(sectionFlow);
            const softwing::Vec3 baseline = normalized(baselineFlow);
            const double alpha = std::atan2(
                dot(cross(chord, flow), liveSection), dot(chord, flow));
            const double baselineAlpha = std::atan2(
                dot(cross(chord, baseline), liveSection),
                dot(chord, baseline));
            halfAlpha[half] += weight * (alpha - baselineAlpha);
            measuredArea[half] += weight;
        }
        if (!(measuredArea[half] > 0.0)) {
            return result;
        }
        halfAlpha[half] /= measuredArea[half];
    }

    // Remove the exact planform-weighted common mode. This machinery is a
    // differential weathercock/rotation correction only; the wing-level
    // sample remains the sole owner of common incidence and trim.
    const double common =
        (sim.halfPlanformArea[0] * halfAlpha[0]
         + sim.halfPlanformArea[1] * halfAlpha[1])
        / (sim.halfPlanformArea[0] + sim.halfPlanformArea[1]);
    result.alphaDeviationRadians = {
        halfAlpha[0] - common, halfAlpha[1] - common};
    result.valid = true;
    return result;
}

namespace {

// One half-span, as the differential pass sees it: the point on its OWN
// mean chord where its share of the turning couple acts, and the 4x4
// system that lays that couple onto its own fabric as pressure.
struct HalfPass
{
    bool valid = false;
    // The half's measured angle, WITHOUT the brake's camber: the anchor
    // travel models where the centre of pressure sits at a given
    // attitude, and a pulled brake is an input, not an attitude.
    double alphaRadians = 0.0;
    softwing::Vec3 anchor;
    softwing::Vec3 chordDirection;
    double matrix[4][5] = {};
};

}  // namespace

void applyAerodynamicForces(SimBody &sim, const SimControls &controls)
{
    if (!sim.body) {
        return;
    }
    sim.body->clearExternalForces();

    // The canopy's added-air term is solver inertia, not structural mass.
    // SoftBody applies gravity through inverse mass, so cancel exactly the
    // gravity that it would otherwise apply to the virtual share. Do this
    // before sampling the flow so a drop from rest still has ordinary g and
    // all physical parts of the system remain in the same free-fall frame.
    if (controls.freeFlight && sim.virtualAddedAirMassKg > 0.0) {
        const std::size_t count = std::min(
            sim.virtualAddedAirMassByNode.size(), sim.body->nodes().size());
        for (std::size_t node = 0; node < count; ++node) {
            const double virtualMass = sim.virtualAddedAirMassByNode[node];
            if (virtualMass > 0.0) {
                sim.body->addForce(
                    node,
                    {0.0, 0.0,
                     virtualMass * gravityMetresPerSecondSquared});
            }
        }
    }
    sim.lastAeroForce = {};
    sim.lastLift = 0.0;
    sim.lastDrag = 0.0;
    sim.lastPolarDragTractionForce = {};
    sim.lastPolarDragTargetNewtons = 0.0;
    sim.lastPolarDragTractionNewtons = 0.0;
    sim.lastPolarDragTractionPowerWatts = 0.0;
    sim.lastGlideRatio = 0.0;
    sim.lastAlphaDegrees = 0.0;
    sim.lastAirspeed = 0.0;

    WingAeroSample sample = sampleWingAero(sim, controls);
    if (!sample.valid) {
        return;
    }

    // Low-pass the angle of attack before it reaches the polar. The raw
    // angle is read off the live fabric, and fabric has pitch modes; a
    // force with ±2 kN of authority that follows those modes instantly is
    // a feedback loop, and it was measured tearing the canopy apart inside
    // half a second. The filtered angle responds on the wake's timescale
    // instead of the fabric's.
    if (!std::isfinite(sim.alphaFilteredRadians)) {
        sim.alphaFilteredRadians = sample.alphaRadians;
        sim.alphaRateRadiansPerSecond = 0.0;
    } else {
        const double blend = std::min(
            1.0, simulationTimeStep / alphaFilterSeconds);
        const double previous = sim.alphaFilteredRadians;
        sim.alphaFilteredRadians +=
            (sample.alphaRadians - previous) * blend;
        sim.alphaRateRadiansPerSecond =
            (sim.alphaFilteredRadians - previous) / simulationTimeStep;
    }
    const double aspectRatio = std::max(1.0, sim.aspectRatio);
    const double finiteWing =
        1.0 / (1.0 + 2.0 / (aspectRatio * kSpanEfficiency));
    // Which angle drives the polar is the deepest difference between the
    // two modes. Free flight closes the loop: the wing's own measured,
    // filtered angle feeds CL and CD, and the whole calibrated stability
    // stack exists to keep that loop from diverging. The tunnel does NOT
    // close it: the polar is evaluated at the PRESCRIBED angle — the
    // rigged rest angle shifted degree-for-degree with the slider — so
    // the load is a dead load along the current airflow. Every
    // closed-loop tunnel variant tried (fast filter, slow filter, split
    // anchor) found a way to pump an oscillation or slide off trim onto
    // slack rows over tens of seconds; open-loop, the bridle geometry
    // alone is statically stable (a nose-down excursion slackens the C
    // rows and the still-taut A rows restore it, and vice versa), and a
    // measurement instrument WANTS the load prescribed: the wing's
    // actual attitude under it is an output, not an input. The measured
    // angle still goes to the HUD via lastAlphaDegrees.
    sample.alphaRadians =
        controls.freeFlight
            ? sim.alphaFilteredRadians
            : sim.alphaTrimRadians
                  + (controls.angleOfAttackDegrees
                     - sim.builtAngleOfAttackDegrees)
                        * kDegreesToRadians;

    // ------------------------------------------------------------------
    // The half-span split. Everything below runs twice, once per half,
    // over that half's own fabric — and the wing's roll and yaw moments
    // then EMERGE from two correctly placed resultants instead of being
    // imposed on top of one. Two earlier attempts bolted a couple onto a
    // wing-level resultant and both failed: as a fifth row in the solve
    // it rebuilt the whole increment field and wrecked the symmetric
    // glide (airspeed 9 -> 14.5 m/s with zero brake), and layered on
    // afterwards as a spanwise pressure gradient it loaded the tips
    // hardest and folded the wing at 4 s against a 9 s baseline. A
    // brake's moment has to arrive where the brake acts: on the aft
    // fabric of its own half.
    //
    // The split is off when the mesh gives no usable partition, and then
    // half 0 is the whole wing at the wing-level mean pair — which is
    // exactly the single pass this replaced.
    // ------------------------------------------------------------------
    const bool split = sim.ribHalf.size() == sim.ribChords.size()
                       && sim.halfPlanformArea[0] > 0.0
                       && sim.halfPlanformArea[1] > 0.0;
    const int halfCount = split ? 2 : 1;

    // Each half's angle, as a low-passed DEPARTURE from the wing's. Rigid
    // spin supplies rate damping, while sideslip projected through the
    // live images of the rest section planes supplies the arc's static
    // weathercock response. The wing-level polar cannot see either after
    // it removes the spanwise component to measure common incidence.
    //
    // Rotation is taken from the canopy's RIGID-body spin, for exactly the
    // reason the per-rib pressure flow is (see canopySpinOf): a rigid fit
    // has no breathing mode, so none of the fabric's own motion can reach a
    // force with kilonewtons of authority. Section-plane incidence is
    // measured relative to its own no-beta baseline, then its exact weighted
    // common mode is removed; a no-spin/no-sideslip wing therefore gets zero
    // even when the mesh or arc is not perfectly mirror-symmetric.
    //
    // Measuring each half's own CHORD line instead was tried first and is
    // wrong twice over: a rigid roll does not move the chords relative to
    // each other, so it produced no damping at all, and what it did
    // measure — differential twist — is positive feedback, since a half
    // that has twisted nose-up is handed more lift and twists further.
    // Measured on the Swoop it took the glide's sideways drift from 5 to
    // 9 m/s and brought the departure forward from 17 s to 13.
    //
    // The tunnel does not close this loop at all: its load is prescribed
    // open-loop for the reasons above, so both halves fly at the
    // prescribed angle there and the split changes nothing but where each
    // half's share of the force is placed.
    if (!split || !controls.freeFlight || !(sample.airspeed > 1.0e-6)) {
        sim.alphaHalfDeviationRadians = {0.0, 0.0};
        sim.halfDynamicPressureRatio = {1.0, 1.0};
    } else {
        const HalfAeroKinematics raw =
            sampleHalfAeroKinematics(sim, sample);
        if (raw.valid) {
            // Filtered on the same wake timescale as the wing-level
            // angle, so it damps a roll without becoming a fast loop of
            // its own. Sideslip now also reaches this differential only:
            // the rest arc's mirrored section planes turn beta into
            // opposite incidences, while the exact weighted common-mode
            // subtraction keeps wing-level trim untouched.
            const double blend =
                std::min(1.0, simulationTimeStep / alphaFilterSeconds);
            for (int half = 0; half < 2; ++half) {
                double &state = sim.alphaHalfDeviationRadians[half];
                state +=
                    (raw.alphaDeviationRadians[half] - state) * blend;
                state = std::clamp(state,
                                   -kHalfAlphaLimitRadians,
                                   kHalfAlphaLimitRadians);
                double &ratio = sim.halfDynamicPressureRatio[half];
                ratio +=
                    (raw.dynamicPressureRatio[half] - ratio) * blend;
                // The spin wind is already capped at the wing's own
                // airspeed, which bounds this at 4; the clamp is the
                // belt to that brace.
                ratio = std::clamp(ratio, 0.0, 4.0);
            }
        }
    }
    const std::array<double, 2> halfAlpha{
        sample.alphaRadians + sim.alphaHalfDeviationRadians[0],
        sample.alphaRadians + sim.alphaHalfDeviationRadians[1]};

    // The polar, evaluated for one half of the wing at its measured angle.
    // In free flight brake input is deliberately absent here. The cable has
    // already moved the live trailing-edge fabric before applyPressure runs,
    // so the base pressure resultant contains the brake's camber, lift and
    // drag at the correct span/chord location. Adding another effective
    // camber and drag coefficient counted the same flap twice: the canopy
    // yawed much faster than its course, met 20-30 degrees of crossflow and
    // folded. The pinned measurement path retains its prescribed polar brake
    // arithmetic for compatibility.
    struct SidePolar
    {
        double lift = 0.0;
        double drag = 0.0;
    };
    const auto polarFor = [&](double brakeMetres, double alphaBase) {
        const double pull = controls.freeFlight
                                ? 0.0
                                : std::clamp(
                                      brakeMetres
                                          / kTunnelBrakeFullPullMetres,
                                      0.0, 1.0);
        const double alpha =
            alphaBase + pull * kTunnelBrakeCamberRadians;
        SidePolar side;
        const double attachedLift = std::max(
            kMinimumLiftCoefficient,
            finiteWing * wingLiftCoefficient(alpha));
        // Blend toward flat-plate normal force past the stall: the
        // attached polar between ±20°, the parachute beyond ±40°, mixed
        // in between.
        const double sinAlpha = std::sin(alpha);
        const double cosAlpha = std::cos(alpha);
        const double postStall =
            std::clamp((std::abs(alpha) - 20.0 * kDegreesToRadians)
                           / (20.0 * kDegreesToRadians),
                       0.0,
                       1.0);
        const double plateNormal = kFlatPlateNormal * sinAlpha;
        side.lift = (1.0 - postStall) * attachedLift
                    + postStall * plateNormal * cosAlpha;
        side.drag = kParasiticDragCoefficient
                    + kTunnelBrakeDragCoefficient * pull
                    + side.lift * side.lift
                          / (kPi * aspectRatio * kSpanEfficiency)
                    + postStall * plateNormal * sinAlpha;
        return side;
    };
    {
        const double blend =
            std::min(1.0, simulationTimeStep / alphaFilterSeconds);
        for (int side = 0; side < 2; ++side) {
            sim.brakeFilteredMetres[side] +=
                (sim.brakeApplied[side] - sim.brakeFilteredMetres[side])
                * blend;
        }
    }
    // "Left" is the solver's left — the negative-span-station half, the
    // same convention SimBody::brakeLines and SimBody::ribHalf use.
    const SidePolar leftSide =
        polarFor(sim.brakeFilteredMetres[0], halfAlpha[0]);
    const SidePolar rightSide =
        polarFor(sim.brakeFilteredMetres[1], halfAlpha[1]);
    // The wing-level pair stays the mean, so every reported number and
    // every symmetric case is bit-for-bit what it was.
    sample.liftCoefficient = 0.5 * (leftSide.lift + rightSide.lift);
    sample.dragCoefficient = 0.5 * (leftSide.drag + rightSide.drag);

    const double q = sample.dynamicPressure;
    const double area = sim.planformArea;

    const auto &nodes = sim.body->nodes();
    const auto &triangles = sim.body->triangles();

    const auto halfOf = [&](std::size_t face) {
        return halfCount == 2
                   ? static_cast<int>(sim.ribHalf[sim.faceAero[face].rib])
                   : 0;
    };

    // FABRIC DRAG. The polar above is the force of a WING: one angle of
    // attack, the rest planform, a lift and a drag coefficient drawn from
    // an aerofoil. A canopy that has folded is not a wing and has no
    // meaningful angle — but it is still several square metres of cloth
    // being dragged through the air edge-on to nothing, and without this
    // term the model gives it none of that. Measured on the Swoop: after
    // an asymmetric fold the risers carried 321 N of a 927 N system, the
    // whole machine fell at two thirds of g, and in a fall that steep the
    // pilot has no apparent weight left to tension the lines with — so
    // nothing pulled the wing back into shape and the collapse was
    // permanent by construction.
    //
    // Half the sum of |A.w| over a closed surface is its frontal area
    // along w. Taking the LIVE surface's frontal area minus the frontal
    // area the DESIGNED surface would present at the same attitude leaves
    // exactly the bluff-body area the deformation created: identically
    // zero on a wing holding its shape, so the tunnel calibration and the
    // trimmed glide are untouched, and square metres once it is a bag.
    // Directed along the wind, so it is pure dissipation — it can slow
    // the system down and can never drive it, which is what keeps it out
    // of the velocity loops the rest of the stability stack avoids.
    //
    // Deliberately NOT split per half, though the machinery below would
    // take it. It is a wing-level bluff-body estimate, and on a healthy
    // wing the two halves differ only by the mesh's own asymmetry — which
    // is millimetres, and which turning into a yawing moment moved the
    // tunnel calibration for no physical reason. A one-sided collapse's
    // yaw has to come from something better conditioned than the
    // difference of two deadbanded area sums.
    double fabricDrag = 0.0;
    if (!sim.restFaceAreas.empty()
        && sim.restFaceAreas.size() >= sim.skinTriangleCount) {
        double liveFrontal = 0.0;
        for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
            const softwing::Triangle &tri = triangles[face];
            const softwing::Vec3 areaVector =
                0.5
                * cross(nodes[tri.b].position - nodes[tri.a].position,
                        nodes[tri.c].position - nodes[tri.a].position);
            liveFrontal +=
                std::abs(dot(areaVector, sample.windDirection));
        }
        liveFrontal *= 0.5;

        // The same wind, written in the rest pose's own frame, so the
        // reference follows the wing's attitude instead of being frozen
        // at the angle it was built at.
        double restFrontal = 0.0;
        const softwing::Vec3 liveChordAxis = normalized(
            sample.windDirection
            - dot(sample.windDirection, sample.spanAxis) * sample.spanAxis);
        if (length(liveChordAxis) > 0.0) {
            const softwing::Vec3 restChordAxis = normalized(
                sim.restChordDirection
                - dot(sim.restChordDirection, sim.restSpanAxis)
                      * sim.restSpanAxis);
            const softwing::Vec3 liveUp =
                cross(sample.spanAxis, liveChordAxis);
            const softwing::Vec3 restUp =
                cross(sim.restSpanAxis, restChordAxis);
            const softwing::Vec3 restWind =
                dot(sample.windDirection, sample.spanAxis) * sim.restSpanAxis
                + dot(sample.windDirection, liveChordAxis) * restChordAxis
                + dot(sample.windDirection, liveUp) * restUp;
            for (const softwing::Vec3 &restArea : sim.restFaceAreas) {
                restFrontal += std::abs(dot(restArea, restWind));
            }
            restFrontal *= 0.5;
        }
        // A fold stacks fabric in its own wake, and the area sum counts
        // every layer where the air only meets the first — measured at
        // 9.1 m2 on a wing whose entire planform is 15. Capped at the
        // planform, which is the largest silhouette a canopy has.
        liveFrontal = std::min(liveFrontal, area);
        sim.lastExcessFrontalArea =
            std::max(0.0, liveFrontal - kFabricDragOnset * restFrontal);
        fabricDrag =
            q * kFabricDragCoefficient * sim.lastExcessFrontalArea;

        // Drag decelerates; it does not propel. Bounded by the impulse
        // that would bring the system to rest against the air within this
        // frame, so however wrong the area estimate goes, the force can
        // null the relative motion and never reverse it. Without this the
        // term pushed a collapsed wing UPWARD — 2900 N of "drag" against
        // 927 N of weight, which is not a drag any more.
        double systemMass = 0.0;
        for (const softwing::Node &node : nodes) {
            if (node.inverseMass > 0.0) {
                systemMass += 1.0 / node.inverseMass;
            }
        }
        const double stoppingForce =
            systemMass * sample.airspeed / simulationTimeStep;
        if (fabricDrag > stoppingForce) {
            fabricDrag = stoppingForce;
            sim.lastExcessFrontalArea =
                q > 0.0 ? fabricDrag / (q * kFabricDragCoefficient) : 0.0;
        }
    }
    sim.lastFabricDragNewtons = fabricDrag;

    const softwing::Vec3 wingForce =
        q * area
              * (sample.liftCoefficient * sample.liftDirection
                 + sample.dragCoefficient * sample.windDirection)
        + fabricDrag * sample.windDirection;

    // What the polar requests minus what the pressure field already made.
    // The tunnel uses this common-mode correction as its prescribed force
    // target. Free flight deliberately preserves the calibrated live pressure
    // force below; only missing positive polar drag is later added as
    // dissipative skin traction.
    const softwing::Vec3 correction = wingForce - aerodynamicForce(sim);

    // THE HALVES DISAGREE, and that disagreement is the turn. Each half
    // has its own brake, its own measured angle and therefore its own
    // coefficients; half their difference, carried on one half and taken
    // off the other, is a pure couple. Referred to the MEAN half planform
    // rather than to each half's own, so a mesh that is not perfectly
    // mirror-symmetric contributes its asymmetry to the shared pass
    // (which has always carried it) and not to the couple — with equal
    // brakes and equal angles this is then exactly zero, and a symmetric
    // case stays bit-for-bit what it was.
    const double halfArea =
        halfCount == 2
            ? 0.5 * (sim.halfPlanformArea[0] + sim.halfPlanformArea[1])
            : 0.0;
    // Each half at its OWN dynamic pressure, so the difference carries
    // both the coefficients the brakes and the angles set and the speeds
    // the wing's rotation gives the two halves. On a wing that is neither
    // rotating nor braked the ratios are 1, the coefficients are equal,
    // and this is exactly the zero vector — which is what keeps the
    // tunnel calibration and the symmetric glide untouched.
    const double leftQ = q * sim.halfDynamicPressureRatio[0];
    const double rightQ = q * sim.halfDynamicPressureRatio[1];
    const softwing::Vec3 sideDifference =
        halfArea * 0.5
        * ((leftQ * leftSide.lift - rightQ * rightSide.lift)
               * sample.liftDirection
           + (leftQ * leftSide.drag - rightQ * rightSide.drag)
                 * sample.windDirection);

    // The tunnel's common-mode target acts at this hang-line-derived fraction
    // of the live mean chord. Free flight keeps the pressure field's natural
    // centre, but the half-wing steering solve still uses local anchors.
    softwing::Vec3 leadingMean;
    softwing::Vec3 trailingMean;
    for (const RibChord &rib : sim.ribChords) {
        leadingMean += nodes[rib.leadingNode].position;
        trailingMean += nodes[rib.trailingNode].position;
    }
    leadingMean /= static_cast<double>(sim.ribChords.size());
    trailingMean /= static_cast<double>(sim.ribChords.size());
    // On the hang line at the in-flight trim, travelling aft/forward
    // with angle-of-attack deviations from it and with the
    // angle-of-attack rate. The static term matters as much as the
    // damping: with a pure damper the wing had nothing restoring it
    // against SLOW drifts, and it mushed itself into stall over ten
    // quiet seconds. The target is the build-time fixed point — the
    // angle the line rigging itself settles at — so the anchor and the
    // lines pull the same way; targeting the rest-pose angle instead
    // put the two in a standing fight.
    // sample.alphaRadians is the polar's driving angle in both modes
    // (filtered-measured in free flight, prescribed in the tunnel), so
    // the anchor travels with the same angle the force was computed at:
    // in the tunnel that makes the centre-of-pressure travel a static,
    // prescribed offset per operating point rather than a feedback path.
    const double anchorFraction = std::clamp(
        sim.resultantChordFraction
            + kAnchorTravelPerRadian
                  * (sample.alphaRadians - sim.alphaTrimRadians)
            + std::clamp(
                  kAnchorRateSeconds * sim.alphaRateRadiansPerSecond,
                  -kAnchorRateLimit,
                  kAnchorRateLimit),
        0.10,
        0.70);
    const softwing::Vec3 anchor =
        leadingMean
        + anchorFraction * (trailingMean - leadingMean);
    const softwing::Vec3 liveChord = trailingMean - leadingMean;
    if (lengthSquared(liveChord) <= 1.0e-9) {
        return;
    }
    const softwing::Vec3 chordDirection = normalized(liveChord);

    // The pressure field's pitch moment about the anchor. Its span-axis
    // component gets cancelled below; roll and yaw are left alone on
    // purpose — they are how asymmetric brake input steers, and they
    // belong to the pressure distribution.
    softwing::Vec3 pressureMoment;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Triangle &tri = triangles[face];
        const softwing::Vec3 &a = nodes[tri.a].position;
        const softwing::Vec3 &b = nodes[tri.b].position;
        const softwing::Vec3 &c = nodes[tri.c].position;
        const softwing::Vec3 pressureForce =
            triangles[face].pressureDifference * 0.5
            * cross(b - a, c - a);
        pressureMoment +=
            cross((a + b + c) / 3.0 - anchor, pressureForce);
    }

    // Each half's own anchor, on its own mean chord at the same
    // hang-line fraction. This is where the couple lands, and placing it
    // here is the whole of the change: the two anchors sit at the halves'
    // real span stations, so the couple has the wing's real lever arm and
    // arrives on the fabric of the half whose brake made it. It costs
    // nothing in pitch — the offset between the two anchors is spanwise,
    // and a spanwise arm crossed with any force has no span-axis
    // component — so the trim is exactly where the shared pass puts it.
    //
    // The angle each anchor travels with is that half's MEASURED angle,
    // without the brake's camber. Adding the camber — the flap's
    // centre-of-pressure travel, which is real — was tried: it halves the
    // wrong-way bank discussed below without fixing its sign, and it
    // brought the departure under a one-sided pull forward by a second
    // and a half. It is not worth having for that. The rate term stays
    // wing-level, like the filter it comes from.
    std::array<HalfPass, 2> halves;
    if (halfCount == 2) {
        for (int half = 0; half < 2; ++half) {
            HalfPass &pass = halves[half];
            pass.alphaRadians = halfAlpha[half];
            softwing::Vec3 halfLeading;
            softwing::Vec3 halfTrailing;
            std::size_t counted = 0;
            for (std::size_t index = 0; index < sim.ribChords.size();
                 ++index) {
                if (sim.ribHalf[index] != static_cast<std::uint8_t>(half)) {
                    continue;
                }
                halfLeading +=
                    nodes[sim.ribChords[index].leadingNode].position;
                halfTrailing +=
                    nodes[sim.ribChords[index].trailingNode].position;
                ++counted;
            }
            if (counted == 0) {
                continue;
            }
            halfLeading /= static_cast<double>(counted);
            halfTrailing /= static_cast<double>(counted);
            const softwing::Vec3 halfChord = halfTrailing - halfLeading;
            if (lengthSquared(halfChord) <= 1.0e-9) {
                continue;
            }
            const double fraction = std::clamp(
                sim.resultantChordFraction
                    + kAnchorTravelPerRadian
                          * (pass.alphaRadians - sim.alphaTrimRadians)
                    + std::clamp(
                          kAnchorRateSeconds
                              * sim.alphaRateRadiansPerSecond,
                          -kAnchorRateLimit,
                          kAnchorRateLimit),
                0.10,
                0.70);
            pass.anchor = halfLeading + fraction * halfChord;
            pass.chordDirection = normalized(halfChord);
            pass.valid = true;
        }
    }

    // The bounded path solves FINAL exterior Cp directly, within its physical
    // [-3, 1] envelope, and reconstructs every face as p_inside - q*Cp. A
    // deterministic weighted projection preserves or introduces force, pitch,
    // then L-R differential equalities in that order; a later stage freezes
    // what the earlier one physically achieved. The old 4x4 increment plus
    // post-clamp path is retained only as an explicit regression mode.
    // applyPressure already evaluates a section aerodynamic law from the
    // canopy's motion through the air. In free flight, replacing that lift a
    // second time with the wing-level polar and forcing its centre onto a
    // synthetic pitch anchor created two competing feedback controllers:
    // pressure, line load and slackness alternated until the wing folded.
    // Keep the section pressure's force and natural centre of pressure in
    // flight. The polar still supplies only its missing dissipative drag
    // below, and its asymmetric half-span terms still steer. The tunnel keeps
    // the prescribed polar correction used by its measurement acceptance
    // path; the environment switches remain useful regression oracles there.
    static const bool disablePolarPitchCorrection =
        qEnvironmentVariableIsSet("LEP_AERO_NO_COUPLE");
    static const bool disablePolarForceCorrection =
        qEnvironmentVariableIsSet("LEP_AERO_NO_CORRECTION");
    const bool preserveNaturalPressurePitch =
        controls.freeFlight || disablePolarPitchCorrection;
    const bool preserveNaturalPressureForce =
        controls.freeFlight || disablePolarForceCorrection;

    std::vector<softwing::Vec3> areaVector(sim.skinTriangleCount);
    std::vector<softwing::Vec3> faceCentre(sim.skinTriangleCount);
    std::vector<double> station(sim.skinTriangleCount, 0.0);
    std::vector<double> halfStation(sim.skinTriangleCount, 0.0);
    std::vector<std::uint8_t> faceHalf(sim.skinTriangleCount, 0);
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Triangle &tri = triangles[face];
        const softwing::Vec3 &a = nodes[tri.a].position;
        const softwing::Vec3 &b = nodes[tri.b].position;
        const softwing::Vec3 &c = nodes[tri.c].position;
        areaVector[face] = 0.5 * cross(b - a, c - a);
        faceCentre[face] = (a + b + c) / 3.0;
        station[face] = dot(faceCentre[face] - anchor, chordDirection);
        faceHalf[face] = static_cast<std::uint8_t>(halfOf(face));
    }

    PressureSolveDiagnostics pressureDiagnostics;
    pressureDiagnostics.requestedForce =
        preserveNaturalPressureForce ? aerodynamicForce(sim) : wingForce;
    pressureDiagnostics.requestedLiftNewtons =
        dot(pressureDiagnostics.requestedForce, sample.liftDirection);
    pressureDiagnostics.requestedDragNewtons =
        dot(pressureDiagnostics.requestedForce, sample.windDirection);
    pressureDiagnostics.requestedPitchMoment =
        preserveNaturalPressurePitch
            ? dot(pressureMoment, sample.spanAxis)
            : 0.0;
    pressureDiagnostics.variableCount = sim.skinTriangleCount;

    softwing::Vec3 priorHalfDifference;
    if (halfCount == 2) {
        for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
            const double sign = faceHalf[face] == 0 ? 1.0 : -1.0;
            priorHalfDifference +=
                sign * triangles[face].pressureDifference * areaVector[face];
        }
    }
    // sideDifference is the force added to the left half and removed from
    // the right. The L-R observable therefore changes by twice that value.
    // Keeping the prior L-R field in the target preserves the canopy arc's
    // authored lateral pressure bracing instead of independently rebuilding
    // each half.
    const softwing::Vec3 requestedHalfDifference =
        priorHalfDifference + 2.0 * sideDifference;
    pressureDiagnostics.requestedHalfDifference = {
        dot(requestedHalfDifference, sample.liftDirection),
        dot(requestedHalfDifference, sample.windDirection)};

    if (controls.pressureSolveMode
        == PressureSolveMode::BoundedExteriorCp) {
        pressureDiagnostics.attempted = true;
        const auto boundedStart = std::chrono::steady_clock::now();

        const bool havePressureState =
            sim.faceInteriorPressure.size() == sim.skinTriangleCount
            && sim.faceDynamicPressure.size() == sim.skinTriangleCount
            && sim.facePriorExternalCp.size() == sim.skinTriangleCount;
        // Near zero q the Cp columns have vanishing physical leverage, so
        // every stage would spend dozens of bounded projections resolving a
        // load smaller than the displayed residual precision. Preserve the
        // stamped field and report zero authority until the polar has at
        // least 10% of its reference dynamic pressure.
        const bool enoughPressureAuthority =
            q >= 0.10 * std::max(0.0, controls.pressurePascal);
        if (havePressureState && enoughPressureAuthority) {
            // Preserve the calibrated load-path shape as the objective
            // prior. The historical 4x4 machinery supplies its proposal
            // (shared force/pitch plus the two half differentials), with each
            // intermediate pressure clipped through that face's physical Cp
            // interval. The bounded solve changes that field only where a
            // higher-priority equality requires it. The unclamped proposal
            // was tested first, but projected 10,748 faces onto Cp=1 and
            // drove an asymmetric limit cycle. Starting again from the base
            // section Cp instead made the minimum buy the polar force with
            // new leading-edge suction, producing a 125 mm dent.
            std::vector<double> preferredCp = sim.facePriorExternalCp;
            std::vector<double> proposedPressure(sim.skinTriangleCount);
            for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
                proposedPressure[face] =
                    triangles[face].pressureDifference;
            }

            double proposalSystem[4][5] = {};
            double proposalHalfSystem[2][4][5] = {};
            for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
                const double faceArea = length(areaVector[face]);
                if (faceArea <= 0.0) {
                    continue;
                }
                const softwing::Vec3 normal = areaVector[face] / faceArea;
                const softwing::Vec3 momentArm =
                    cross(faceCentre[face] - anchor, areaVector[face]);
                const double basis[4] = {
                    normal.x, normal.y, normal.z, station[face]};
                for (int column = 0; column < 4; ++column) {
                    proposalSystem[0][column] +=
                        basis[column] * areaVector[face].x;
                    proposalSystem[1][column] +=
                        basis[column] * areaVector[face].y;
                    proposalSystem[2][column] +=
                        basis[column] * areaVector[face].z;
                    proposalSystem[3][column] +=
                        basis[column]
                        * dot(momentArm, sample.spanAxis);
                }
                const int half = faceHalf[face];
                if (half < 0 || half > 1 || !halves[half].valid) {
                    continue;
                }
                halfStation[face] = dot(
                    faceCentre[face] - halves[half].anchor,
                    halves[half].chordDirection);
                const softwing::Vec3 halfArm = cross(
                    faceCentre[face] - halves[half].anchor,
                    areaVector[face]);
                const double halfBasis[4] = {
                    normal.x, normal.y, normal.z, halfStation[face]};
                for (int column = 0; column < 4; ++column) {
                    proposalHalfSystem[half][0][column] +=
                        halfBasis[column] * areaVector[face].x;
                    proposalHalfSystem[half][1][column] +=
                        halfBasis[column] * areaVector[face].y;
                    proposalHalfSystem[half][2][column] +=
                        halfBasis[column] * areaVector[face].z;
                    proposalHalfSystem[half][3][column] +=
                        halfBasis[column]
                        * dot(halfArm, sample.spanAxis);
                }
            }
            proposalSystem[0][4] =
                preserveNaturalPressureForce ? 0.0 : correction.x;
            proposalSystem[1][4] =
                preserveNaturalPressureForce ? 0.0 : correction.y;
            proposalSystem[2][4] =
                preserveNaturalPressureForce ? 0.0 : correction.z;
            proposalSystem[3][4] =
                preserveNaturalPressurePitch
                    ? 0.0
                    : -dot(pressureMoment, sample.spanAxis);
            const auto solveProposal4x4 = [](
                double matrix[4][5], double solution[4]) {
                for (int pivot = 0; pivot < 4; ++pivot) {
                    int best = pivot;
                    for (int row = pivot + 1; row < 4; ++row) {
                        if (std::abs(matrix[row][pivot])
                            > std::abs(matrix[best][pivot])) {
                            best = row;
                        }
                    }
                    if (std::abs(matrix[best][pivot]) < 1.0e-9) {
                        return false;
                    }
                    if (best != pivot) {
                        for (int column = 0; column < 5; ++column) {
                            std::swap(matrix[pivot][column],
                                      matrix[best][column]);
                        }
                    }
                    for (int row = pivot + 1; row < 4; ++row) {
                        const double factor =
                            matrix[row][pivot] / matrix[pivot][pivot];
                        for (int column = pivot; column < 5; ++column) {
                            matrix[row][column] -=
                                factor * matrix[pivot][column];
                        }
                    }
                }
                for (int row = 3; row >= 0; --row) {
                    double value = matrix[row][4];
                    for (int column = row + 1; column < 4; ++column) {
                        value -= matrix[row][column] * solution[column];
                    }
                    solution[row] = value / matrix[row][row];
                }
                return true;
            };
            double proposalSolution[4] = {};
            const auto clampProposalPressure =
                [&](std::size_t face, double pressure) {
                const double faceDynamic =
                    std::max(0.0, sim.faceDynamicPressure[face]);
                const double faceInterior = sim.faceInteriorPressure[face];
                // Cp in [-3,1], expressed without dividing by a possibly
                // tiny q. Unlike the legacy absolute ceiling this interval
                // remains ordered during the preinflated soft start, where
                // p_inside is near 80 Pa while local q can be below 1 Pa.
                return std::clamp(
                    pressure,
                    faceInterior
                        - maximumExteriorPressureCoefficient * faceDynamic,
                    faceInterior
                        - minimumExteriorPressureCoefficient * faceDynamic);
            };
            if (solveProposal4x4(proposalSystem, proposalSolution)) {
                const softwing::Vec3 gradient{
                    proposalSolution[0], proposalSolution[1],
                    proposalSolution[2]};
                for (std::size_t face = 0;
                     face < sim.skinTriangleCount; ++face) {
                    const double faceArea = length(areaVector[face]);
                    if (faceArea <= 0.0) {
                        continue;
                    }
                    const softwing::Vec3 normal =
                        areaVector[face] / faceArea;
                    proposedPressure[face] = clampProposalPressure(
                        face,
                        proposedPressure[face]
                            + dot(normal, gradient)
                            + proposalSolution[3] * station[face]);
                }
            }
            if (lengthSquared(sideDifference) > 0.0
                && halves[0].valid && halves[1].valid) {
                for (int half = 0; half < 2; ++half) {
                    const softwing::Vec3 want =
                        half == 0 ? sideDifference
                                  : -1.0 * sideDifference;
                    proposalHalfSystem[half][0][4] = want.x;
                    proposalHalfSystem[half][1][4] = want.y;
                    proposalHalfSystem[half][2][4] = want.z;
                    proposalHalfSystem[half][3][4] = 0.0;
                    double halfSolution[4] = {};
                    if (!solveProposal4x4(
                            proposalHalfSystem[half], halfSolution)) {
                        continue;
                    }
                    const softwing::Vec3 gradient{
                        halfSolution[0], halfSolution[1],
                        halfSolution[2]};
                    for (std::size_t face = 0;
                         face < sim.skinTriangleCount; ++face) {
                        if (faceHalf[face]
                            != static_cast<std::uint8_t>(half)) {
                            continue;
                        }
                        const double faceArea = length(areaVector[face]);
                        if (faceArea <= 0.0) {
                            continue;
                        }
                        const softwing::Vec3 normal =
                            areaVector[face] / faceArea;
                        proposedPressure[face] = clampProposalPressure(
                            face,
                            proposedPressure[face]
                                + dot(normal, gradient)
                                + halfSolution[3] * halfStation[face]);
                    }
                }
            }
            for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
                const double faceDynamic = sim.faceDynamicPressure[face];
                if (faceDynamic <= 1.0e-9) {
                    continue;
                }
                const double proposal =
                    (sim.faceInteriorPressure[face]
                     - proposedPressure[face])
                    / faceDynamic;
                if (std::isfinite(proposal)) {
                    preferredCp[face] = proposal;
                }
            }

            BoundedEqualityProblem problem;
            problem.variableCount = sim.skinTriangleCount;
            problem.rowCount = 6;
            problem.stageRowCounts = {3, 1, 2};
            problem.authorityMode = AuthoritySolveMode::CachedProbe;
            problem.authorityHint = sim.pressureAuthorityHint;
            problem.multiplierHint = sim.pressureMultiplierHint;
            problem.authorityProbeStep = 0.02;
            problem.feasibilityTolerance = 1.0e-7;
            problem.prior.resize(problem.variableCount);
            problem.lower.resize(problem.variableCount);
            problem.upper.resize(problem.variableCount);
            problem.mobility.resize(problem.variableCount);
            problem.matrix.assign(
                problem.rowCount * problem.variableCount, 0.0);
            problem.target.assign(problem.rowCount, 0.0);

            softwing::Vec3 interiorForce;
            softwing::Vec3 interiorHalfDifference;
            double interiorPitch = 0.0;
            for (std::size_t face = 0; face < problem.variableCount;
                 ++face) {
                const double faceDynamic =
                    std::max(0.0, sim.faceDynamicPressure[face]);
                const double faceInterior = sim.faceInteriorPressure[face];
                const double stampedCp = std::clamp(
                    sim.facePriorExternalCp[face],
                    minimumExteriorPressureCoefficient,
                    maximumExteriorPressureCoefficient);
                const double priorCp = std::clamp(
                    preferredCp[face],
                    minimumExteriorPressureCoefficient,
                    maximumExteriorPressureCoefficient);
                problem.prior[face] = priorCp;
                sim.faceRetrimPreferredCp[face] = priorCp;

                // A still-air face has no Cp authority: changing Cp while
                // q=0 is mathematically invisible and only manufactures a
                // condition number. Fix it to the physical prior.
                if (faceDynamic <= 1.0e-9) {
                    problem.lower[face] = stampedCp;
                    problem.upper[face] = stampedCp;
                } else {
                    // A local Cp trust region keeps exact global moments
                    // from rebuilding the structurally calibrated proposal
                    // into a different pressure topology. Five hundredths
                    // Cp is 4 Pa at the standard q=80 Pa: enough to repair
                    // small clipped resultant/pitch errors, bounded enough that
                    // missing authority is reported instead of purchased by
                    // concentrating load at a nose or trailing edge.
                    // The tunnel is a shape instrument, so its narrow local
                    // window protects the calibrated pressure topology.
                    constexpr double cpTrustRadius = 0.05;
                    problem.lower[face] = std::max(
                        minimumExteriorPressureCoefficient,
                        priorCp - cpTrustRadius);
                    problem.upper[face] = std::min(
                        maximumExteriorPressureCoefficient,
                        priorCp + cpTrustRadius);
                }

                // Integral squared change from the calibrated proposal.
                // All faces get equal Cp mobility per unit area; the
                // structurally meaningful chordwise shape is in the prior,
                // not an arbitrary preference for the leading edge.
                problem.mobility[face] =
                    1.0 / std::max(1.0e-9, length(areaVector[face]));

                const softwing::Vec3 cpForce =
                    -faceDynamic * areaVector[face];
                problem.matrix[face] = cpForce.x;
                problem.matrix[problem.variableCount + face] = cpForce.y;
                problem.matrix[2 * problem.variableCount + face] =
                    cpForce.z;
                problem.matrix[3 * problem.variableCount + face] =
                    dot(cross(faceCentre[face] - anchor, cpForce),
                        sample.spanAxis);
                const double halfSign =
                    halfCount == 2 && faceHalf[face] == 0 ? 1.0
                    : halfCount == 2                       ? -1.0
                                                         : 0.0;
                problem.matrix[4 * problem.variableCount + face] =
                    halfSign * dot(cpForce, sample.liftDirection);
                problem.matrix[5 * problem.variableCount + face] =
                    halfSign * dot(cpForce, sample.windDirection);

                const softwing::Vec3 inside =
                    faceInterior * areaVector[face];
                interiorForce += inside;
                interiorPitch +=
                    dot(cross(faceCentre[face] - anchor, inside),
                        sample.spanAxis);
                interiorHalfDifference += halfSign * inside;
            }

            const softwing::Vec3 forceTarget =
                pressureDiagnostics.requestedForce - interiorForce;
            problem.target[0] = forceTarget.x;
            problem.target[1] = forceTarget.y;
            problem.target[2] = forceTarget.z;
            problem.target[3] =
                pressureDiagnostics.requestedPitchMoment - interiorPitch;
            problem.target[4] =
                pressureDiagnostics.requestedHalfDifference[0]
                - dot(interiorHalfDifference, sample.liftDirection);
            problem.target[5] =
                pressureDiagnostics.requestedHalfDifference[1]
                - dot(interiorHalfDifference, sample.windDirection);

            const BoundedEqualityResult result =
                solveBoundedEqualityHierarchy(problem);
            pressureDiagnostics.valid = result.valid;
            pressureDiagnostics.numericalFailure =
                result.numericalFailure;
            pressureDiagnostics.authority = result.authority;
            pressureDiagnostics.authorityHint = result.authorityHint;
            pressureDiagnostics.authorityBackoffs =
                result.authorityBackoffs;
            pressureDiagnostics.authorityProbeAccepted =
                result.authorityProbeAccepted;
            pressureDiagnostics.rank = result.rank;
            pressureDiagnostics.conditionEstimate =
                result.conditionEstimate;
            pressureDiagnostics.activeLower = result.activeLower;
            pressureDiagnostics.activeUpper = result.activeUpper;
            pressureDiagnostics.projectionCalls = result.projectionCalls;
            pressureDiagnostics.projectionIterations =
                result.projectionIterations;
            if (result.valid
                && result.value.size() == sim.skinTriangleCount) {
                sim.pressureAuthorityHint = result.authority;
                sim.pressureMultiplierHint = result.multiplierHint;
                sim.faceAppliedExternalCp = result.value;
                for (std::size_t face = 0;
                     face < sim.skinTriangleCount; ++face) {
                    const double finalPressure =
                        sim.faceInteriorPressure[face]
                        - sim.faceDynamicPressure[face]
                              * sim.faceAppliedExternalCp[face];
                    sim.body->setFacePressureDifference(
                        face, finalPressure);
                }
            }
        } else if (!havePressureState) {
            pressureDiagnostics.numericalFailure = true;
        } else {
            pressureDiagnostics.valid = true;
        }
        pressureDiagnostics.solveMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - boundedStart)
                .count();
    } else {
        pressureDiagnostics.legacy = true;

    // Assemble the legacy 4x4 increment system: columns are
    // (v.x, v.y, v.z, μ), rows are
    // the three force components and the span-axis moment (pitch). The
    // same geometry is assembled for each half about its own anchor and
    // its own chord, ready for the differential pass.
    double system[4][5] = {};
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Vec3 &areaN = areaVector[face];
        const double areaLength = length(areaN);
        if (areaLength <= 0.0) {
            continue;
        }
        const softwing::Vec3 normal = areaN / areaLength;
        const softwing::Vec3 momentArm =
            cross(faceCentre[face] - anchor, areaN);
        const double basis[4] = {normal.x, normal.y, normal.z,
                                 station[face]};
        for (int column = 0; column < 4; ++column) {
            system[0][column] += basis[column] * areaN.x;
            system[1][column] += basis[column] * areaN.y;
            system[2][column] += basis[column] * areaN.z;
            system[3][column] +=
                basis[column] * dot(momentArm, sample.spanAxis);
        }
        HalfPass &pass = halves[faceHalf[face]];
        if (!pass.valid) {
            continue;
        }
        halfStation[face] =
            dot(faceCentre[face] - pass.anchor, pass.chordDirection);
        const softwing::Vec3 halfArm =
            cross(faceCentre[face] - pass.anchor, areaN);
        const double halfBasis[4] = {normal.x, normal.y, normal.z,
                                     halfStation[face]};
        for (int column = 0; column < 4; ++column) {
            pass.matrix[0][column] += halfBasis[column] * areaN.x;
            pass.matrix[1][column] += halfBasis[column] * areaN.y;
            pass.matrix[2][column] += halfBasis[column] * areaN.z;
            pass.matrix[3][column] +=
                halfBasis[column] * dot(halfArm, sample.spanAxis);
        }
    }
    system[0][4] =
        disablePolarForceCorrection ? 0.0 : correction.x;
    system[1][4] =
        disablePolarForceCorrection ? 0.0 : correction.y;
    system[2][4] =
        disablePolarForceCorrection ? 0.0 : correction.z;
    system[3][4] =
        disablePolarPitchCorrection
            ? 0.0
            : -dot(pressureMoment, sample.spanAxis);

    // Gaussian elimination with partial pivoting; a singular system (a
    // degenerate skin) just skips the retrim.
    const auto solve4x4 = [](double matrix[4][5], double solution[4]) {
        for (int pivot = 0; pivot < 4; ++pivot) {
            int best = pivot;
            for (int row = pivot + 1; row < 4; ++row) {
                if (std::abs(matrix[row][pivot])
                    > std::abs(matrix[best][pivot])) {
                    best = row;
                }
            }
            if (std::abs(matrix[best][pivot]) < 1.0e-9) {
                return false;
            }
            if (best != pivot) {
                for (int column = 0; column < 5; ++column) {
                    std::swap(matrix[pivot][column], matrix[best][column]);
                }
            }
            for (int row = pivot + 1; row < 4; ++row) {
                const double factor =
                    matrix[row][pivot] / matrix[pivot][pivot];
                for (int column = pivot; column < 5; ++column) {
                    matrix[row][column] -= factor * matrix[pivot][column];
                }
            }
        }
        for (int row = 3; row >= 0; --row) {
            double value = matrix[row][4];
            for (int column = row + 1; column < 4; ++column) {
                value -= matrix[row][column] * solution[column];
            }
            solution[row] = value / matrix[row][row];
        }
        return true;
    };

    const double ceiling =
        kMaximumDynamicPressureRatio
        * std::max(q, std::max(0.0, controls.pressurePascal));
    // The floor the retrim may not go under, per face: the interior this
    // face has, less the most the air outside it can push with (see
    // SimBody::facePressureFloor). A flat -0.5*q was used before, and it
    // let the pitch gradient drive the trailing edge to Cp 1.2-1.5 — a
    // pressure above stagnation, which the stamped field forbids itself
    // one line at a time and the retrim then walked straight through.
    const auto floorFor = [&](std::size_t face) {
        return face < sim.facePressureFloor.size()
                   ? sim.facePressureFloor[face]
                   : -0.5 * q;
    };
    double solution[4] = {};
    if (solve4x4(system, solution)) {
        const softwing::Vec3 gradient{solution[0], solution[1],
                                      solution[2]};
        const double couple = solution[3];
        for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
            const double areaLength = length(areaVector[face]);
            if (areaLength <= 0.0) {
                continue;
            }
            const softwing::Vec3 normal = areaVector[face] / areaLength;
            const double increment = dot(normal, gradient)
                                     + couple * station[face];
            // The base field never pulls a face inward (Cp is capped at
            // stagnation); the retrim may, a little, but a face sucked
            // hard into the cell is how the intrados got dented before,
            // so the combined field is floored just below zero and
            // capped like the base field.
            const double base = triangles[face].pressureDifference;
            const double combined =
                std::clamp(base + increment, floorFor(face), ceiling);
            sim.body->setFacePressureDifference(face, combined);
        }
    }

    // THE DIFFERENTIAL PASS: the couple, laid on each half's own fabric
    // through the same machinery, about that half's own anchor and its
    // own chord. One half is asked for +sideDifference and the other for
    // -sideDifference, each with no pitch moment of its own, so the
    // wing's total force and pitch are untouched and what is added is
    // exactly a roll-and-yaw couple.
    //
    // Two earlier attempts put this couple somewhere else and both
    // failed. As a FIFTH ROW inside the solve above it wrecked the
    // symmetric glide (airspeed 9 -> 14.5 m/s, sink -1.3 -> -3.2, with
    // zero brake): constraining that increment's own roll moment is not
    // a no-op, it rebuilds the entire increment field. Layered on
    // afterwards as a SPANWISE PRESSURE GRADIENT it was symmetric-safe
    // but folded the wing at 4 s against a 9 s baseline (span 8.4 -> 5.4
    // m in one second), because a gradient across the span loads the tips
    // hardest and the tips are where this fabric is weakest. Running the
    // whole force pass per half instead — each half cancelling its own
    // pressure resultant — was tried third and takes the arc's lateral
    // bracing out of the fabric: the tunnel wing lost 8% of its span and
    // 18% of its volume and never settled. What is left, and what this
    // is, is the shared pass exactly as it was plus a couple that lands
    // where the brake acts.
    if (lengthSquared(sideDifference) > 0.0 && halves[0].valid
        && halves[1].valid) {
        for (int half = 0; half < 2; ++half) {
            HalfPass &pass = halves[half];
            const softwing::Vec3 want =
                half == 0 ? sideDifference : -1.0 * sideDifference;
            pass.matrix[0][4] = want.x;
            pass.matrix[1][4] = want.y;
            pass.matrix[2][4] = want.z;
            pass.matrix[3][4] = 0.0;
            double halfSolution[4] = {};
            if (!solve4x4(pass.matrix, halfSolution)) {
                continue;
            }
            const softwing::Vec3 gradient{halfSolution[0], halfSolution[1],
                                          halfSolution[2]};
            const double couple = halfSolution[3];
            for (std::size_t face = 0; face < sim.skinTriangleCount;
                 ++face) {
                if (faceHalf[face] != static_cast<std::uint8_t>(half)) {
                    continue;
                }
                const double areaLength = length(areaVector[face]);
                if (areaLength <= 0.0) {
                    continue;
                }
                const softwing::Vec3 normal =
                    areaVector[face] / areaLength;
                const double increment =
                    dot(normal, gradient) + couple * halfStation[face];
                const double base = triangles[face].pressureDifference;
                const double combined =
                    std::clamp(base + increment, floorFor(face), ceiling);
                sim.body->setFacePressureDifference(face, combined);
            }
        }
    }
        pressureDiagnostics.valid = true;
        if (sim.faceInteriorPressure.size() == sim.skinTriangleCount
            && sim.faceDynamicPressure.size() == sim.skinTriangleCount
            && sim.facePriorExternalCp.size() == sim.skinTriangleCount) {
            sim.faceAppliedExternalCp.resize(sim.skinTriangleCount);
            for (std::size_t face = 0; face < sim.skinTriangleCount;
                 ++face) {
                const double faceDynamic = sim.faceDynamicPressure[face];
                sim.faceAppliedExternalCp[face] =
                    faceDynamic > 1.0e-9
                        ? (sim.faceInteriorPressure[face]
                           - triangles[face].pressureDifference)
                              / faceDynamic
                        : sim.facePriorExternalCp[face];
            }
        }
    }

    // What actually reached the fabric. For the bounded path these are
    // physical-unit residuals after authority saturation; for the legacy
    // oracle they expose what its post-solve clamps changed.
    softwing::Vec3 achieved;
    softwing::Vec3 achievedMoment;
    softwing::Vec3 achievedHalfDifference;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Vec3 force =
            triangles[face].pressureDifference * areaVector[face];
        achieved += force;
        achievedMoment += cross(faceCentre[face] - anchor, force);
        if (halfCount == 2) {
            achievedHalfDifference +=
                (faceHalf[face] == 0 ? 1.0 : -1.0) * force;
        }
    }
    pressureDiagnostics.achievedForce = achieved;
    pressureDiagnostics.achievedLiftNewtons =
        dot(achieved, sample.liftDirection);
    pressureDiagnostics.achievedDragNewtons =
        dot(achieved, sample.windDirection);
    pressureDiagnostics.forceResidual =
        achieved - pressureDiagnostics.requestedForce;
    pressureDiagnostics.achievedPitchMoment =
        dot(achievedMoment, sample.spanAxis);
    pressureDiagnostics.pitchResidual =
        pressureDiagnostics.achievedPitchMoment
        - pressureDiagnostics.requestedPitchMoment;
    pressureDiagnostics.achievedHalfDifference = {
        dot(achievedHalfDifference, sample.liftDirection),
        dot(achievedHalfDifference, sample.windDirection)};
    for (std::size_t axis = 0; axis < 2; ++axis) {
        pressureDiagnostics.halfDifferenceResidual[axis] =
            pressureDiagnostics.achievedHalfDifference[axis]
            - pressureDiagnostics.requestedHalfDifference[axis];
    }
    if (!sim.faceAppliedExternalCp.empty()) {
        const auto [minimumCp, maximumCp] = std::minmax_element(
            sim.faceAppliedExternalCp.begin(),
            sim.faceAppliedExternalCp.end());
        pressureDiagnostics.minimumCp = *minimumCp;
        pressureDiagnostics.maximumCp = *maximumCp;
    }
    sim.pressureSolve = pressureDiagnostics;
    sim.lastForceResidual = pressureDiagnostics.forceResidual;
    sim.lastPitchResidual = pressureDiagnostics.pitchResidual;

    // A pressure-only inviscid field can saturate at the physical Cp bounds
    // before it realizes the polar's viscous drag. Add only that positive
    // deficit as skin traction: no missing lift is manufactured here. The
    // force is distributed by current wetted area and points with the
    // relative wind, so its work against the canopy's air-relative bulk
    // velocity is necessarily dissipative. With no dynamic pressure the
    // requested and applied traction are exactly zero.
    softwing::Vec3 polarDragTraction;
    if (controls.pressureSolveMode
            == PressureSolveMode::BoundedExteriorCp
        && controls.freeFlight && q > 1.0e-9) {
        // Close only the finite-wing polar's viscous drag deficit. The
        // deformation/frontal-area heuristic remains diagnostic: applying its
        // full residual as shear made harmless early cloth breathing become
        // hundreds of newtons of extra drag and a self-amplifying collapse.
        sim.lastPolarDragTargetNewtons =
            q * area * sample.dragCoefficient;
        const double missingDrag = std::max(
            0.0,
            sim.lastPolarDragTargetNewtons
                - pressureDiagnostics.achievedDragNewtons);
        double skinArea = 0.0;
        for (const softwing::Vec3 &faceArea : areaVector) {
            skinArea += length(faceArea);
        }
        if (missingDrag > 0.0 && skinArea > 1.0e-12) {
            polarDragTraction = missingDrag * sample.windDirection;
            for (std::size_t face = 0; face < sim.skinTriangleCount;
                 ++face) {
                const softwing::Triangle &triangle = triangles[face];
                const softwing::Vec3 share =
                    polarDragTraction
                    * (length(areaVector[face]) / skinArea / 3.0);
                sim.body->addForce(triangle.a, share);
                sim.body->addForce(triangle.b, share);
                sim.body->addForce(triangle.c, share);
            }
        }
    }
    sim.lastPolarDragTractionForce = polarDragTraction;
    sim.lastPolarDragTractionNewtons =
        dot(polarDragTraction, sample.windDirection);
    sim.lastPolarDragTractionPowerWatts = dot(
        polarDragTraction,
        canopyVelocityOf(sim) - airVelocityWorld(sim, controls));

    // The pilot as a bluff body, dragged where the mass hangs, against
    // the pilot's own relative wind. Beyond trimming the pendulum lean,
    // this is the pendulum's damper: a swinging pilot moves through the
    // air and pays for it.
    softwing::Vec3 pilotDrag;
    if (sim.pilotNode != noConstraint) {
        const softwing::Vec3 pilotWind = relativeAirVelocity(
            airVelocityWorld(sim, controls),
            nodes[sim.pilotNode].velocity);
        const double pilotSpeed = std::min(length(pilotWind), 40.0);
        pilotDrag = 0.5 * kAirDensity * kPilotDragArea * pilotSpeed
                    * pilotWind;
        sim.body->addForce(sim.pilotNode, pilotDrag);
    }

    if (controls.pressureSolveMode
        == PressureSolveMode::BoundedExteriorCp) {
        // Production telemetry reports the force that survived the physical
        // Cp/trust bounds, not the polar request. Dynamics already receives
        // this achieved pressure through the triangles; reporting the request
        // here made an authority-zero frame still claim 1280 N of lift.
        sim.lastAeroForce = achieved + polarDragTraction + pilotDrag;
        sim.lastLift = dot(sim.lastAeroForce, sample.liftDirection);
        sim.lastDrag = dot(sim.lastAeroForce, sample.windDirection);
    } else {
        // Explicit regression mode preserves its historical requested-polar
        // CSV/readout. pressureSolve still records the actual residual.
        sim.lastAeroForce = wingForce + pilotDrag;
        sim.lastLift = q * area * sample.liftCoefficient;
        sim.lastDrag =
            q * area * sample.dragCoefficient + length(pilotDrag);
    }
    sim.lastGlideRatio =
        sim.lastDrag > 1.0e-6 ? sim.lastLift / sim.lastDrag : 0.0;
    // The MEASURED attitude, in both modes. In the tunnel the polar ran
    // at the prescribed angle, but what the HUD should report is where
    // the rigging actually put the wing under that load.
    sim.lastAlphaDegrees = sim.alphaFilteredRadians / kDegreesToRadians;
    sim.lastAirspeed = sample.airspeed;
}

AeroSummary aerodynamicSummary(const SimBody &sim,
                               const SimControls &controls)
{
    AeroSummary summary;
    if (!sim.body) {
        return summary;
    }
    if (controls.freeFlight || controls.flightLoad) {
        // applyAerodynamicForces records what its selected pressure path
        // actually reports. Production is the force that survived the
        // bounded Cp/trust solve; the explicit legacy oracle preserves its
        // historical requested-polar readout.
        summary.force = sim.lastAeroForce;
        summary.lift = sim.lastLift;
        summary.drag = sim.lastDrag;
        summary.glideRatio = sim.lastGlideRatio;
        return summary;
    }
    // Pinned, the pressure field is all there is. It carries no drag
    // model, so resolve it against the airflow for what it is worth.
    summary.force = aerodynamicForce(sim);
    const softwing::Vec3 relative = airVelocityWorld(sim, controls);
    if (length(relative) <= 1.0e-6) {
        return summary;
    }
    const softwing::Vec3 windDirection = normalized(relative);
    summary.drag = dot(summary.force, windDirection);
    summary.lift = length(summary.force - summary.drag * windDirection);
    summary.glideRatio =
        summary.drag > 1.0e-6 ? summary.lift / summary.drag : 0.0;
    return summary;
}

softwing::Vec3 aerodynamicForce(const SimBody &sim)
{
    if (!sim.body) {
        return {};
    }
    const auto &nodes = sim.body->nodes();
    const auto &triangles = sim.body->triangles();
    softwing::Vec3 total;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Triangle &tri = triangles[face];
        const softwing::Vec3 &a = nodes[tri.a].position;
        const softwing::Vec3 &b = nodes[tri.b].position;
        const softwing::Vec3 &c = nodes[tri.c].position;
        total += tri.pressureDifference * 0.5 * cross(b - a, c - a);
    }
    return total;
}

unsigned playgroundWorkerThreads()
{
    const unsigned cores = softwing::hardwarePhysicalCoreCount();
    return cores > 3 ? cores - 2 : 1;
}

void recentreSystem(SimBody &sim)
{
    if (!sim.body) {
        return;
    }
    auto &nodes = sim.body->nodes();
    softwing::Vec3 centre;
    double mass = 0.0;
    for (const softwing::Node &node : nodes) {
        if (node.inverseMass <= 0.0) {
            continue;
        }
        const double nodeMass = 1.0 / node.inverseMass;
        centre += nodeMass * node.position;
        mass += nodeMass;
    }
    if (!(mass > 0.0)) {
        return;
    }
    const softwing::Vec3 shift = -1.0 * (centre / mass);
    for (softwing::Node &node : nodes) {
        node.position += shift;
        // Moved by the same amount, so the velocity XPBD reconstructs from
        // the pair is unchanged. Shifting only the position would silently
        // brake the whole system every frame.
        node.previousPosition += shift;
    }
}

// --- Fabric/line contact -------------------------------------------------

void prepareContact(SimBody &sim)
{
    std::vector<PlaygroundContactLine> authoredLines;
    authoredLines.reserve(sim.lineSegments.size());
    for (const LineSegment &line : sim.lineSegments) {
        if (line.suspension) {
            authoredLines.push_back(
                {static_cast<std::uint32_t>(line.a),
                 static_cast<std::uint32_t>(line.b)});
        }
    }
    preparePlaygroundContact(sim.contact, sim.body->nodes(),
                             sim.body->triangles(), sim.skinTriangleCount,
                             authoredLines);
}

void stepSimulation(SimBody &sim, const SimControls &controls)
{
    if (!sim.body) {
        return;
    }
    // Braking is a shorter line, not a hand placed somewhere. Cables are
    // one-sided, so letting the brake off simply restores the slack.
    //
    // Under flight load the tunnel adds a brake GAP, exactly as real wings
    // rig one: the cascades are sized to the rest pose, but a loaded
    // canopy pitches into its tether cone and the trailing edge moves
    // decimetres relative to the fixed handles. Without the gap the
    // brakes went spuriously taut at zero input — 80 N on one side —
    // and the asymmetric snatch wound the wing up over tens of seconds.
    // Free flight keeps its calibrated zero-gap rigging: there the
    // handles ride on the pilot, who moves with the canopy.
    const double brakeGap =
        !controls.freeFlight && controls.flightLoad
            ? kTunnelBrakeGapMetres
            : 0.0;
    // The pull the wing actually gets, chasing the pull the pilot asked
    // for at no more than a hand's speed IN SIMULATED TIME. See
    // SimBody::brakeApplied: the controls are sampled on the wall clock
    // and the wing runs on its own, so without this a hand movement made
    // over three quarters of a second arrives as a third of a second of
    // input. It applies to letting go as much as to pulling — a release
    // is the half of a brake input that surges the wing, and a step
    // release is not something a hand can do.
    const double reach = kBrakeHandSpeed * simulationTimeStep;
    const std::array<double, 2> wanted{controls.brakeLeft,
                                       controls.brakeRight};
    for (int side = 0; side < 2; ++side) {
        sim.brakeApplied[side] +=
            std::clamp(wanted[side] - sim.brakeApplied[side],
                       -reach,
                       reach);
    }
    auto &constraints = sim.body->constraints();
    for (const BrakeLine &brake : sim.brakeLines) {
        if (brake.constraint >= constraints.size()) {
            continue;
        }
        const double pull =
            brake.left ? sim.brakeApplied[0] : sim.brakeApplied[1];
        constraints[brake.constraint].restLength =
            std::max(0.05, brake.restLength + brakeGap - pull);
    }
    const int coupledSubsteps = std::max(1, controls.substeps);
    const double coupledTimeStep =
        simulationTimeStep / static_cast<double>(coupledSubsteps);
    const bool pneumaticActive = controls.cellPressureModel
                                 && !sim.cells.empty();
    // The prior frame already ended with a complete pneumatic update. At
    // this frame boundary only refresh the aerodynamic target and pressure
    // stamp; each completed XPBD substep advances exactly one dt/N of flow.
    applyPressureForDuration(
        sim, controls, pneumaticActive ? 0.0 : coupledTimeStep);
    // The polar force pass runs in free flight always, and pinned when the
    // wind tunnel asks for flight load. Pinned without it, the canopy
    // carries only the pressure field's own resultant, and an inviscid
    // pressure field under-reads lift badly (d'Alembert: full leading-edge
    // suction, no viscous loss) — so every line-load number read off the
    // tethered wing is fiction. With flightLoad the same wing-level polar
    // is spread over the skin as pressure; the carabiners stay fixed, so
    // the ~1 kN resultant reacts into the tether exactly as a tunnel
    // model's load reacts into its balance.
    if (controls.freeFlight || controls.flightLoad) {
        applyAerodynamicForces(sim, controls);
    }

    softwing::StepSettings settings;
    settings.timeStep = simulationTimeStep;
    settings.substeps = controls.substeps;
    settings.constraintIterations = controls.constraintIterations;
    settings.cableConstraintSweepPairs =
        controls.freeFlight
            ? std::max(0, controls.freeFlightCableSweepPairs)
            : 0;
    settings.gravity =
        controls.freeFlight
            ? softwing::Vec3{0.0, 0.0, -gravityMetresPerSecondSquared}
            : softwing::Vec3{0.0, 0.0, 0.0};
    // Free flight damps velocity RELATIVE to the system's bulk motion.
    // Damping absolute velocity at glide speed is a fake drag several
    // times the whole real drag budget — it, not the aerodynamics, would
    // set the trim speed. Referred to the bulk velocity, the damping
    // quiets fabric ringing and resists tumbling while leaving the glide
    // itself untouched; the reference is a whole-frame constant, so within
    // the step it stays a pure change of frame. Pinned, nothing glides and
    // the old heavy absolute damping keeps the fabric quiet.
    // Under flight load the tunnel damps harder still. The imposed polar
    // follows the angle of attack through a deliberate 0.25 s lag, and on
    // the tether's one-sided cables — which catch and release as rows
    // load and unload — that lag sustains a limit cycle at ~2 m/s of
    // agitation that no measurement can be read through. A tunnel mount
    // is allowed to be heavily damped: the tunnel measures statics, and
    // the dynamics it would distort are free flight's job.
    settings.velocityDampingPerSecond =
        controls.freeFlight ? systemDampingPerSecond
        : controls.flightLoad ? tunnelDampingPerSecond
                              : 3.0;
    if (controls.freeFlight) {
        settings.dampingReferenceVelocity = systemVelocityOf(sim);
    }
    settings.workerThreads = controls.workerThreads;
    settings.performanceProfile = controls.performanceProfile;
    const bool contactActive = controls.fabricContact && sim.contact.prepared
                               && sim.skinTriangleCount > 0;
    if (contactActive || pneumaticActive) {
        // Contact projection has to interleave with the solve — at
        // collapse speeds the fabric crosses its own thickness many
        // times inside one frame, so a single end-of-frame fix would
        // resolve against the wrong side. The frame is stepped as N
        // single-substep calls with the projection after each one; the
        // arithmetic (dt/N per substep, damping per substep) is the same
        // as the engine's own internal loop. With the option off this
        // branch is never taken and the step is exactly the old one.
        const int substeps = coupledSubsteps;
        softwing::StepSettings sub = settings;
        sub.timeStep = simulationTimeStep / substeps;
        sub.substeps = 1;
        if (contactActive) {
            beginPlaygroundContactFrame(sim.contact, sim.body->nodes(),
                                        sim.body->triangles(),
                                        sim.skinTriangleCount,
                                        sub.timeStep);
        }
        // step() consumes the external-force channel at the end of every
        // call (it snapshots node.force, replays it per substep, then
        // clears it), so the frame's forces — the whole polar flight
        // load — must be re-seeded before each single-substep call or
        // they would act for one thirtieth of the frame.
        std::vector<softwing::Vec3> externalForces;
        externalForces.reserve(sim.body->nodes().size());
        for (const softwing::Node &node : sim.body->nodes()) {
            externalForces.push_back(node.force);
        }
        for (int substep = 0; substep < substeps; ++substep) {
            auto &liveNodes = sim.body->nodes();
            for (std::size_t index = 0; index < liveNodes.size();
                 ++index) {
                liveNodes[index].force = externalForces[index];
            }
            sim.body->step(sub);
            if (contactActive) {
                // Project candidates captured BEFORE this substep first:
                // their retained normal is the side a fast crossing returns
                // to.
                projectPlaygroundContact(sim.contact, sim.body->nodes(),
                                         sim.body->triangles());
            }
            if (pneumaticActive) {
                // Gas stiffness is far too high for a once-per-frame
                // explicit force. Re-measuring V and flowing mass after every
                // XPBD substep makes the pressure response act with the
                // changing geometry. Including the final substep leaves the
                // HUD and next frame synchronized with the live mesh.
                advanceAndRestampCellInterior(
                    sim, controls, sub.timeStep);
            }
            if (substep + 1 < substeps) {
                if (contactActive
                    && playgroundContactEnvelopeEscaped(
                        sim.contact, sim.body->nodes())) {
                    refreshPlaygroundContact(
                        sim.contact, sim.body->nodes(),
                        sim.body->triangles(), sim.skinTriangleCount,
                        sub.timeStep);
                }
            }
        }
    } else {
        sim.body->step(settings);
    }

    if (controls.freeFlight) {
        recentreSystem(sim);
    }

    // Carry the air past the wing. In the wing's own frame — the one the
    // camera shows — a parcel of air moves at the relative wind, so this
    // integrates exactly that: the glide and the sink in free flight, the
    // tunnel's own airflow when the wing is pinned. Nothing else in the
    // model records that the wing is travelling, because both modes keep
    // it at the origin.
    sim.airTravel += simulationTimeStep
                     * relativeAirVelocity(
                         airVelocityWorld(sim, controls),
                         controls.freeFlight ? canopyVelocityOf(sim)
                                             : softwing::Vec3{});
}

bool beginGrab(SimBody &sim, std::size_t junctionNode)
{
    if (!sim.body || junctionNode >= sim.body->nodes().size()) {
        return false;
    }
    const softwing::Vec3 place =
        sim.body->nodes()[junctionNode].position;
    // Re-grabbing a junction that already has a cable wakes that cable
    // instead of adding another: constraints cannot be removed, so
    // repeated grabs must not accumulate. The scan covers EVERY previous
    // grab cable, not just the latest — alternating between two
    // junctions with a last-cable-only check grew the constraint table
    // by one dead cable per switch, and each grew colouring rebuild.
    // Grab cables are the only constraints whose endpoint a is the
    // anchor, so the scan cannot confuse anything else.
    std::size_t existing = noConstraint;
    if (sim.grabAnchorNode != noConstraint) {
        const auto &constraints = sim.body->constraints();
        for (std::size_t index = 0; index < constraints.size(); ++index) {
            if (constraints[index].a == sim.grabAnchorNode
                && constraints[index].b == junctionNode) {
                existing = index;
                break;
            }
        }
    }
    if (sim.grabConstraint != noConstraint
        && sim.grabConstraint != existing) {
        sim.body->constraints()[sim.grabConstraint].restLength = 1.0e6;
    }
    if (existing != noConstraint) {
        sim.body->constraints()[existing].restLength = 0.01;
        sim.grabConstraint = existing;
    } else {
        if (sim.grabAnchorNode == noConstraint) {
            sim.grabAnchorNode = sim.body->addFixedNode(place);
        }
        // Adding a constraint after build is safe: the colouring rebuilds
        // lazily off the count change.
        sim.grabConstraint = sim.body->addCableConstraint(
            sim.grabAnchorNode, junctionNode, 0.01, grabCompliance);
    }
    // Both positions, so the anchor arrives with no reconstructed
    // velocity; constraints never move a fixed node, so this is the only
    // thing that ever places it.
    softwing::Node &anchor = sim.body->nodes()[sim.grabAnchorNode];
    anchor.position = place;
    anchor.previousPosition = place;
    sim.grabbedNode = junctionNode;
    return true;
}

void moveGrab(SimBody &sim, const softwing::Vec3 &target)
{
    if (!sim.body || !grabActive(sim)
        || sim.grabAnchorNode == noConstraint) {
        return;
    }
    softwing::Node &anchor = sim.body->nodes()[sim.grabAnchorNode];
    anchor.position = target;
    anchor.previousPosition = target;
}

void endGrab(SimBody &sim)
{
    if (sim.body && sim.grabConstraint != noConstraint) {
        // Slack, not gone: a cable longer than any wing is a cable that
        // never engages, and the constraint stays available for the next
        // grab of the same junction.
        sim.body->constraints()[sim.grabConstraint].restLength = 1.0e6;
    }
    sim.grabbedNode = noConstraint;
}

bool grabActive(const SimBody &sim)
{
    return sim.grabbedNode != noConstraint;
}

double grabForceNewtons(const SimBody &sim, const SimControls &controls)
{
    if (!sim.body || !grabActive(sim)
        || sim.grabConstraint == noConstraint
        || sim.grabConstraint >= sim.body->constraints().size()) {
        return 0.0;
    }
    // λ of the last substep; force = -λ/h². Cable λ is clamped <= 0, so
    // the floor only guards round-off.
    const double substepRate = controls.substeps / simulationTimeStep;
    return std::max(0.0,
                    -sim.body->constraints()[sim.grabConstraint]
                            .accumulatedLambda
                        * substepRate * substepRate);
}

}  // namespace lep::playground
