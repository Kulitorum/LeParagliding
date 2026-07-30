#include "playground_sim.h"

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
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <utility>

namespace lep::playground {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kAirDensity = 1.225;   // kg/m^3
constexpr double kMaximumDynamicPressureRatio = 4.0;
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
// The anchor's static reference tracks the wing's own slow-average angle
// of attack rather than a fixed target: the line rigging plus the
// pendulum are already statically stable in pitch, and an anchor that
// pulled toward its own idea of trim just fought them — raising its gain
// produced a limit cycle, not stability. Washed out this way, the anchor
// carries no standing moment and acts purely as a damper on the
// timescales the fabric and pendulum oscillate at.
constexpr double kAnchorWashoutSeconds = 3.0;
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
// Brake input as seen by the polar. The geometric side of a brake pull is
// already real — the trailing edge comes down, the measured chord
// rotates, the lift follows — but the drag of a deflected trailing edge
// is not in the pressure field, and without it braking ADDED energy
// (lift with no penalty) and pumped the surge mode instead of damping
// it. Full pull is taken as the swing test's 35 cm.
constexpr double kBrakeFullPullMetres = 0.35;
constexpr double kBrakeDragCoefficient = 0.12;
// Free travel before a tunnel brake engages (see stepSimulation). Sized
// to cover the trailing edge's excursion across the sweep's attitude
// range; real wings rig 10-20 cm for the same reason.
constexpr double kTunnelBrakeGapMetres = 0.20;
// Absolute damping of the flight-loaded tunnel (see stepSimulation).
constexpr double tunnelDampingPerSecond = 8.0;

// Defined in the aerodynamics section further down; buildSimBody needs
// them for pilot sizing and the glide launch.
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
    for (const QJsonValue &value : root.value(QLatin1String("lines")).toArray()) {
        const QJsonObject line = value.toObject();
        // Row plan (1..6 = A..F) is a late addition; meshes written before
        // it default to 0, which the row-load instrumentation treats as
        // "unknown" rather than a row of its own.
        mesh.lines.push_back(
            {vec(line.value(QLatin1String("a")).toArray()),
             vec(line.value(QLatin1String("b")).toArray()),
             line.value(QLatin1String("brake")).toInt() != 0,
             line.value(QLatin1String("plan")).toInt(0)});
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

SimBody buildSimBody(const SimMesh &mesh,
                     const SimBuildOptions &options,
                     const SimControls &controls)
{
    SimBody sim;
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
    for (const auto &tri : triangles) {
        addEdge(tri[0], tri[1]);
        addEdge(tri[1], tri[2]);
        addEdge(tri[2], tri[0]);
    }
    for (const auto &quad : mesh.quads) {
        addEdge(quad[1], quad[3]);
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
        sim.renderFaces.push_back(drawn);
    }

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
            sim.ribChords.push_back(chord);
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
        for (const RibChord &rib : sim.ribChords) {
            meanChord += normalized(mesh.nodes[rib.trailingNode]
                                    - mesh.nodes[rib.leadingNode]);
            meanSpan += rib.spanAxis;
        }
        if (length(meanChord) > 0.0) {
            sim.restChordDirection = normalized(meanChord);
        }
        if (length(meanSpan) > 0.0) {
            sim.restSpanAxis = normalized(meanSpan);
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

    // Suspension lines: weld junctions, cable constraints per segment,
    // attach top ends to the nearest skin node, fix the pilot band.
    std::map<std::uint64_t, std::size_t> junctions;
    const auto lineNode = [&](const softwing::Vec3 &point) {
        const auto [entry, inserted] =
            junctions.try_emplace(quantizedKey(point), 0);
        if (inserted) {
            entry->second = body->addNode(point, lineJunctionMass);
        }
        return entry->second;
    };
    double lowestZ = std::numeric_limits<double>::max();
    for (const SimLine &line : mesh.lines) {
        lowestZ = std::min({lowestZ, line.a.z, line.b.z});
    }
    for (const SimLine &line : mesh.lines) {
        const std::size_t a = lineNode(line.a);
        const std::size_t b = lineNode(line.b);
        if (a == b) {
            continue;
        }
        sim.lineSegments.push_back(
            {a,
             b,
             line.brake,
             body->addCableConstraint(
                 a, b, length(line.b - line.a), lineCompliance),
             line.plan});
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
            body->addDistanceConstraint(
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
        // Provisional mass; the real one is set once the wing's lift can be
        // integrated, further down.
        sim.pilotNode = body->addNode(pilotPlace, 1.0);
        for (const std::size_t node : carabiners) {
            tie(sim.pilotNode,
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
                 sideConstraint(sim.pilotNode, node)});
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
        const std::size_t handle =
            body->addNode(handlePosition, lineJunctionMass);
        // In free flight the pilot holds the handle, so a brake pull reacts
        // into his mass. Pinned, the handle is nailed down instead.
        if (sim.pilotNode != noConstraint) {
            tie(handle,
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
            sim.lineSegments.push_back({handle, top, true, constraint, 6});
            sim.brakeLines.push_back({constraint, rest, side < 0.0});
        }
    }

    sim.body = std::move(body);
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

    // Trim the pilot to the wing rather than the other way round. The load
    // model is crude enough that its absolute lift is not a number to hang a
    // wing loading off, but the pendulum only behaves if weight and lift are
    // in the same place: too heavy and the system falls with the pilot
    // weightless in his harness, too light and it climbs and the risers go
    // slack. Sizing the pilot from the wing's own lift at its default
    // setting makes every preset hang properly, and the slider still lets
    // the system sink or climb from there.
    if (sim.pilotNode != noConstraint) {
        auto &nodes = sim.body->nodes();
        double bodyMass = 0.0;
        for (std::size_t index = 0; index < nodes.size(); ++index) {
            if (index != sim.pilotNode && nodes[index].inverseMass > 0.0) {
                bodyMass += 1.0 / nodes[index].inverseMass;
            }
        }
        // Sized so the system is in vertical balance at its rest attitude:
        // in a steady glide the weight equals the full aerodynamic
        // resultant, sqrt(L² + D²), of the polar the force pass imposes.
        // Sizing from the polar rather than from the integrated pressure
        // field is what lets the pilot be a realistic weight — the
        // pressure field's own lift under-reads badly, and a pilot sized
        // from it hung the wing at a fraction of its real wing loading.
        // In flight the pendulum hangs vertical and the path descends at
        // the glide angle, so the wing flies at its rest angle PLUS the
        // glide angle. Iterate that pair to a fixed point, then size the
        // pilot so the weight matches the polar's resultant there — that
        // is what lets the launched system hang at its own trim instead
        // of climbing away or folding under a starved dynamic pressure.
        const WingAeroSample aero = sampleWingAero(sim, controls);
        double wanted = 90.0;
        if (aero.valid && sim.planformArea > 0.0) {
            const double aspectRatio = std::max(1.0, sim.aspectRatio);
            const double finiteWing =
                1.0 / (1.0 + 2.0 / (aspectRatio * kSpanEfficiency));
            double alphaFly = aero.alphaRadians;
            double lift = 0.0;
            double drag = 0.0;
            for (int iteration = 0; iteration < 4; ++iteration) {
                const double liftCoefficient =
                    finiteWing * wingLiftCoefficient(alphaFly);
                const double dragCoefficient =
                    kParasiticDragCoefficient
                    + liftCoefficient * liftCoefficient
                          / (kPi * aspectRatio * kSpanEfficiency);
                lift = aero.dynamicPressure * sim.planformArea
                       * liftCoefficient;
                drag = aero.dynamicPressure
                       * (sim.planformArea * dragCoefficient
                          + kPilotDragArea);
                sim.glideAngleRadians = std::atan2(drag, lift);
                alphaFly = aero.alphaRadians + sim.glideAngleRadians;
            }
            sim.alphaTrimRadians = alphaFly;
            // A little under the exact balance. Flying a few percent
            // light trims a degree or two below the fixed point, which
            // is the margin that keeps the arc-tilted tip sections out
            // of their own stall — sized exactly, the slow trim creep
            // walked the tips into stall at about seven seconds and the
            // span folded.
            wanted = 0.90 * std::hypot(lift, drag)
                         / gravityMetresPerSecondSquared
                     - bodyMass;
        }
        sim.pilotMass = std::clamp(wanted, 30.0, 250.0);
        nodes[sim.pilotNode].inverseMass = 1.0 / sim.pilotMass;
    }
    for (const std::size_t node : brakeHandles) {
        if (!controls.freeFlight) {
            sim.body->fixNode(node);
        }
    }

    // Launch ON the glide, not at a dead stop in a horizontal wind. From
    // rest, the imposed drag has no counterpart until the system develops
    // its sink, and in that transient the light canopy gets yanked aft
    // around the heavy pilot, loses its angle of attack, and pendulums
    // over — measured every time, with every force scheme. So every node
    // starts with the steady descent velocity the polar predicts: the
    // relative wind then meets the canopy climbing at the glide angle,
    // which is exactly the in-flight trim the pilot was sized for. The
    // attitude needs no adjustment — the rest pose already hangs the
    // pendulum vertical, which is its in-flight attitude too.
    if (controls.freeFlight && sim.pilotNode != noConstraint
        && sim.glideAngleRadians > 0.0) {
        const WingAeroSample rest = sampleWingAero(sim, controls);
        if (rest.valid) {
            const softwing::Vec3 freestream =
                rest.airspeed * rest.windDirection;
            const softwing::Vec3 descent =
                freestream
                - rotateAbout(freestream,
                              rest.spanAxis,
                              sim.glideAngleRadians);
            for (softwing::Node &node : sim.body->nodes()) {
                node.velocity = descent;
            }
            applyPressure(sim, controls);
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

}  // namespace

void applyPressure(SimBody &sim, const SimControls &controls)
{
    if (!sim.body) {
        return;
    }
    const double dynamicPressure = controls.pressurePascal;

    // No rib loops, no chord to hang a distribution off: fall back to the
    // uniform field, which is what this used to be everywhere.
    if (sim.faceAero.empty() || sim.ribChords.empty()) {
        sim.body->setUniformPressureDifference(
            sim.body->surfaceGroup(0, sim.skinTriangleCount),
            dynamicPressure);
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
    // In free flight the slider is ignored and the air is level: the wing
    // finds its own trim, and tilting the oncoming air is a hillside, not
    // an angle-of-attack control — at the pinned default of 6° it trimmed
    // the flying wing into its stall.
    const double airspeed =
        std::sqrt(2.0 * std::max(0.0, dynamicPressure) / kAirDensity);
    const double angle =
        (controls.freeFlight ? 0.0 : controls.angleOfAttackDegrees)
        * kDegreesToRadians;
    const softwing::Vec3 freestream =
        airspeed
        * rotateAbout(sim.restChordDirection, sim.restSpanAxis, angle);

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
    if (controls.freeFlight) {
        systemVelocity = canopyVelocityOf(sim);
    }

    sim.ribLiftCoefficient.assign(sim.ribChords.size(), 0.0);
    std::vector<double> &ribLift = sim.ribLiftCoefficient;
    std::vector<double> ribPressure(sim.ribChords.size(), dynamicPressure);
    for (std::size_t index = 0; index < sim.ribChords.size(); ++index) {
        const RibChord &rib = sim.ribChords[index];
        const softwing::Vec3 chord = nodes[rib.trailingNode].position
                                     - nodes[rib.leadingNode].position;
        if (length(chord) <= 0.0) {
            continue;
        }
        const softwing::Vec3 relativeWind = freestream - systemVelocity;

        // Both vectors flattened into the section's own plane, so the angle
        // measured is pitch and not some part of the wing's sweep or arc.
        const softwing::Vec3 axis = rib.spanAxis;
        const softwing::Vec3 chordInPlane =
            normalized(chord - dot(chord, axis) * axis);
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
                     kMaximumDynamicPressureRatio * dynamicPressure);
    }

    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const FaceAero &aero = sim.faceAero[face];
        const double coefficient = externalPressureCoefficient(
            aero.chordFraction, aero.upperSurface, ribLift[aero.rib]);
        // The cell interior sits at ram pressure, so its gauge pressure is
        // q; the outside of the face is q·Cp. Only the difference loads the
        // fabric, which is why there is no separate ambient anywhere here.
        sim.body->setFacePressureDifference(
            face, ribPressure[aero.rib] * (1.0 - coefficient));
    }
}

namespace {

// The airflow the whole wing sees, shared by the load and drag passes.
softwing::Vec3 freestreamVelocity(const SimBody &sim,
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
    const softwing::Vec3 relative =
        freestreamVelocity(sim, controls)
        - (controls.freeFlight || controls.flightLoad
               ? canopyVelocityOf(sim)
               : softwing::Vec3{});
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

    // Live span axis between the tip ribs' leading edges, oriented like the
    // rest one so a settled wing and its rest pose agree on signs.
    softwing::Vec3 spanAxis =
        nodes[sim.ribChords[sim.spanTipRibs[1]].leadingNode].position
        - nodes[sim.ribChords[sim.spanTipRibs[0]].leadingNode].position;
    if (length(spanAxis) <= 1.0e-6) {
        spanAxis = sim.restSpanAxis;
    }
    spanAxis = normalized(spanAxis);
    if (dot(spanAxis, sim.restSpanAxis) < 0.0) {
        spanAxis = -1.0 * spanAxis;
    }
    sample.spanAxis = spanAxis;

    // Mean live chord, length-weighted so a tip rib flapping about cannot
    // steer the whole wing's angle, flattened into the plane normal to the
    // span so sweep and arc do not contaminate the pitch measurement.
    softwing::Vec3 chordSum;
    for (const RibChord &rib : sim.ribChords) {
        chordSum += nodes[rib.trailingNode].position
                    - nodes[rib.leadingNode].position;
    }
    const softwing::Vec3 chordInPlane =
        chordSum - dot(chordSum, spanAxis) * spanAxis;
    const softwing::Vec3 windInPlane =
        relative - dot(relative, spanAxis) * spanAxis;
    if (length(chordInPlane) <= 1.0e-6 || length(windInPlane) <= 1.0e-6) {
        return sample;
    }
    const softwing::Vec3 chordDirection = normalized(chordInPlane);
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

void applyAerodynamicForces(SimBody &sim, const SimControls &controls)
{
    if (!sim.body) {
        return;
    }
    sim.body->clearExternalForces();
    sim.lastAeroForce = {};
    sim.lastLift = 0.0;
    sim.lastDrag = 0.0;
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
        sim.alphaSlowRadians = sample.alphaRadians;
        sim.alphaRateRadiansPerSecond = 0.0;
    } else {
        const double blend = std::min(
            1.0, simulationTimeStep / alphaFilterSeconds);
        const double previous = sim.alphaFilteredRadians;
        sim.alphaFilteredRadians +=
            (sample.alphaRadians - previous) * blend;
        sim.alphaRateRadiansPerSecond =
            (sim.alphaFilteredRadians - previous) / simulationTimeStep;
        sim.alphaSlowRadians +=
            (sim.alphaFilteredRadians - sim.alphaSlowRadians)
            * std::min(1.0, simulationTimeStep / kAnchorWashoutSeconds);
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
    const double attachedLift = std::max(
        kMinimumLiftCoefficient,
        finiteWing * wingLiftCoefficient(sample.alphaRadians));

    // Blend toward flat-plate normal force past the stall: the attached
    // polar between ±20°, the parachute beyond ±40°, mixed in between.
    const double sinAlpha = std::sin(sample.alphaRadians);
    const double cosAlpha = std::cos(sample.alphaRadians);
    const double postStall = std::clamp(
        (std::abs(sample.alphaRadians) - 20.0 * kDegreesToRadians)
            / (20.0 * kDegreesToRadians),
        0.0,
        1.0);
    const double plateNormal = kFlatPlateNormal * sinAlpha;
    sample.liftCoefficient =
        (1.0 - postStall) * attachedLift
        + postStall * plateNormal * cosAlpha;
    const double brakeFraction = std::clamp(
        (controls.brakeLeft + controls.brakeRight)
            / (2.0 * kBrakeFullPullMetres),
        0.0,
        1.0);
    sample.dragCoefficient =
        kParasiticDragCoefficient
        + kBrakeDragCoefficient * brakeFraction
        + sample.liftCoefficient * sample.liftCoefficient
              / (kPi * aspectRatio * kSpanEfficiency)
        + postStall * plateNormal * sinAlpha;

    const double q = sample.dynamicPressure;
    const double area = sim.planformArea;
    const softwing::Vec3 wingForce =
        q * area
        * (sample.liftCoefficient * sample.liftDirection
           + sample.dragCoefficient * sample.windDirection);

    // What the polar wants minus what the pressure field already made: the
    // pressure resultant is cancelled in full — its lift is unrealistically
    // small and its along-wind component is spurious thrust, and the two
    // cannot be fixed independently — and the polar's force imposed in its
    // place. Spread by area so the local fabric loads, which are the
    // pressure field's actual job, are disturbed as little as possible.
    const softwing::Vec3 correction = wingForce - aerodynamicForce(sim);

    const auto &nodes = sim.body->nodes();
    const auto &triangles = sim.body->triangles();

    // Where the total resultant must act: the hang-line-derived fraction
    // of the live mean chord. The line geometry was drawn for a wing whose
    // resultant sits there; making that true here is what lets the
    // designed lines set the trim angle.
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

    // The correction enters the canopy the only way fabric carries load
    // gracefully: as pressure. Every other application was measured
    // failing structurally — point loads at the line attachments dented
    // the intrados into the cells, an area-spread body force leaned the
    // canopy over, a fabric-distributed couple crushed the nose. Here a
    // per-face pressure increment δp_i = n̂_i·v + μ·s_i is added to the
    // stamped field, with (v, μ) solved from a 4x4 linear system so that
    // the increment's resultant equals the correction exactly and the
    // total pitch moment about the anchor is zero. s_i is the chordwise
    // station, so μ is a linear chordwise pressure gradient — the
    // pressure-native form of a pitch couple. Roll and yaw moments are
    // deliberately left to the base pressure field: they are how brakes
    // steer.
    static const bool noCouple =
        qEnvironmentVariableIsSet("LEP_AERO_NO_COUPLE");
    static const bool noCorrection =
        qEnvironmentVariableIsSet("LEP_AERO_NO_CORRECTION");

    std::vector<softwing::Vec3> areaVector(sim.skinTriangleCount);
    std::vector<softwing::Vec3> faceCentre(sim.skinTriangleCount);
    std::vector<double> station(sim.skinTriangleCount, 0.0);
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Triangle &tri = triangles[face];
        const softwing::Vec3 &a = nodes[tri.a].position;
        const softwing::Vec3 &b = nodes[tri.b].position;
        const softwing::Vec3 &c = nodes[tri.c].position;
        areaVector[face] = 0.5 * cross(b - a, c - a);
        faceCentre[face] = (a + b + c) / 3.0;
        station[face] = dot(faceCentre[face] - anchor, chordDirection);
    }

    // Assemble the 4x4 system: columns are (v.x, v.y, v.z, μ), rows are
    // the three force components and the span-axis moment.
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
    }
    system[0][4] = noCorrection ? 0.0 : correction.x;
    system[1][4] = noCorrection ? 0.0 : correction.y;
    system[2][4] = noCorrection ? 0.0 : correction.z;
    system[3][4] =
        noCouple ? 0.0 : -dot(pressureMoment, sample.spanAxis);

    // Gaussian elimination with partial pivoting; a singular system (a
    // degenerate skin) just skips the retrim.
    bool solved = true;
    for (int pivot = 0; pivot < 4 && solved; ++pivot) {
        int best = pivot;
        for (int row = pivot + 1; row < 4; ++row) {
            if (std::abs(system[row][pivot])
                > std::abs(system[best][pivot])) {
                best = row;
            }
        }
        if (std::abs(system[best][pivot]) < 1.0e-9) {
            solved = false;
            break;
        }
        if (best != pivot) {
            for (int column = 0; column < 5; ++column) {
                std::swap(system[pivot][column], system[best][column]);
            }
        }
        for (int row = pivot + 1; row < 4; ++row) {
            const double factor =
                system[row][pivot] / system[pivot][pivot];
            for (int column = pivot; column < 5; ++column) {
                system[row][column] -= factor * system[pivot][column];
            }
        }
    }
    if (solved) {
        double solution[4] = {};
        for (int row = 3; row >= 0; --row) {
            double value = system[row][4];
            for (int column = row + 1; column < 4; ++column) {
                value -= system[row][column] * solution[column];
            }
            solution[row] = value / system[row][row];
        }
        const softwing::Vec3 gradient{solution[0], solution[1],
                                      solution[2]};
        const double couple = solution[3];
        for (std::size_t face = 0; face < sim.skinTriangleCount;
             ++face) {
            const double areaLength = length(areaVector[face]);
            if (areaLength <= 0.0) {
                continue;
            }
            const softwing::Vec3 normal = areaVector[face] / areaLength;
            const double increment =
                dot(normal, gradient) + couple * station[face];
            // The base field never pulls a face inward (Cp is capped at
            // stagnation); the retrim may, a little, but a face sucked
            // hard into the cell is how the intrados got dented before,
            // so the combined field is floored just below zero and
            // capped like the base field.
            const double base = triangles[face].pressureDifference;
            const double combined = std::clamp(
                base + increment,
                -0.5 * q,
                kMaximumDynamicPressureRatio
                    * std::max(q, std::max(0.0,
                                           controls.pressurePascal)));
            sim.body->setFacePressureDifference(face, combined);
        }

        // What actually got applied, after clamping: the residuals say
        // whether the solve's promise survived contact with the floor
        // and the cap.
        softwing::Vec3 achieved;
        softwing::Vec3 achievedMoment;
        for (std::size_t face = 0; face < sim.skinTriangleCount;
             ++face) {
            const softwing::Vec3 force =
                triangles[face].pressureDifference * areaVector[face];
            achieved += force;
            achievedMoment += cross(faceCentre[face] - anchor, force);
        }
        sim.lastForceResidual = achieved - wingForce;
        sim.lastPitchResidual = dot(achievedMoment, sample.spanAxis);
    }

    // The pilot as a bluff body, dragged where the mass hangs, against
    // the pilot's own relative wind. Beyond trimming the pendulum lean,
    // this is the pendulum's damper: a swinging pilot moves through the
    // air and pays for it.
    softwing::Vec3 pilotDrag;
    if (sim.pilotNode != noConstraint) {
        const softwing::Vec3 pilotWind =
            freestreamVelocity(sim, controls)
            - nodes[sim.pilotNode].velocity;
        const double pilotSpeed = std::min(length(pilotWind), 40.0);
        pilotDrag = 0.5 * kAirDensity * kPilotDragArea * pilotSpeed
                    * pilotWind;
        sim.body->addForce(sim.pilotNode, pilotDrag);
    }

    sim.lastAeroForce = wingForce + pilotDrag;
    sim.lastLift = q * area * sample.liftCoefficient;
    sim.lastDrag =
        q * area * sample.dragCoefficient + length(pilotDrag);
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
        // The imposed polar is the whole aerodynamic force whenever its
        // pass runs — free flight, or pinned with flight load; the
        // pressure resultant is cancelled against it by construction.
        summary.force = sim.lastAeroForce;
        summary.lift = sim.lastLift;
        summary.drag = sim.lastDrag;
        summary.glideRatio = sim.lastGlideRatio;
        return summary;
    }
    // Pinned, the pressure field is all there is. It carries no drag
    // model, so resolve it against the airflow for what it is worth.
    summary.force = aerodynamicForce(sim);
    const softwing::Vec3 relative = freestreamVelocity(sim, controls);
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
    auto &constraints = sim.body->constraints();
    for (const BrakeLine &brake : sim.brakeLines) {
        if (brake.constraint >= constraints.size()) {
            continue;
        }
        const double pull =
            brake.left ? controls.brakeLeft : controls.brakeRight;
        constraints[brake.constraint].restLength =
            std::max(0.05, brake.restLength + brakeGap - pull);
    }
    applyPressure(sim, controls);
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
    sim.body->step(settings);

    if (controls.freeFlight) {
        recentreSystem(sim);
    }
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
