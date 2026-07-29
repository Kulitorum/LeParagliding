#include "playground_sim.h"

#include <softwing/parallel.h>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QLatin1String>

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
// Per unit of wetted area. Covers the fabric, the seams and the lines,
// which on a paraglider are a large slice of the total -- this is well
// above a clean aerofoil's skin friction for that reason.
constexpr double kFrictionCoefficient = 0.015;
// Oswald efficiency; a paraglider's elliptical-ish planform is decent.
constexpr double kSpanEfficiency = 0.9;

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
        mesh.lines.push_back(
            {vec(line.value(QLatin1String("a")).toArray()),
             vec(line.value(QLatin1String("b")).toArray()),
             line.value(QLatin1String("brake")).toInt() != 0});
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

        // Planform, taken as the upper surface's area — for a thin canopy
        // that is the reference area to within a few percent, and it is the
        // one the induced-drag term wants.
        double upperArea = 0.0;
        for (std::size_t face = 0; face < triangles.size(); ++face) {
            if (!sim.faceAero[face].upperSurface) {
                continue;
            }
            const auto &tri = triangles[face];
            const softwing::Vec3 &a = mesh.nodes[static_cast<std::size_t>(tri[0])];
            const softwing::Vec3 &b = mesh.nodes[static_cast<std::size_t>(tri[1])];
            const softwing::Vec3 &c = mesh.nodes[static_cast<std::size_t>(tri[2])];
            upperArea += length(0.5 * cross(b - a, c - a));
        }
        sim.planformArea = upperArea;
        double spanLow = std::numeric_limits<double>::max();
        double spanHigh = std::numeric_limits<double>::lowest();
        for (const softwing::Vec3 &node : mesh.nodes) {
            spanLow = std::min(spanLow, node.x);
            spanHigh = std::max(spanHigh, node.x);
        }
        const double span = spanHigh - spanLow;
        if (upperArea > 0.0 && span > 0.0) {
            sim.aspectRatio = span * span / upperArea;
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
                 a, b, length(line.b - line.a), lineCompliance)});
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
            sim.lineSegments.push_back({handle, top, true, constraint});
            sim.brakeLines.push_back({constraint, rest, side < 0.0});
        }
    }

    sim.body = std::move(body);
    applyPressure(sim, controls);

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
        // Sized off the lift the wing actually makes at its rest pose, and
        // then held well below it. Two reasons for the margin: that rest
        // pose over-reads, because the canopy is at its designed shape with
        // no fabric slack anywhere; and this is a toy load model whose
        // integrated lift comes out well under a real wing's. Hang a
        // realistic pilot on it and the canopy simply folds. Better a light
        // pilot on a wing that flies than a correct one on a wing that
        // collapses -- and the mass ratio, which is what the pendulum
        // actually cares about, is unaffected.
        const double lift = std::abs(aerodynamicForce(sim).z);
        const double wanted =
            0.55 * lift / gravityMetresPerSecondSquared - bodyMass;
        sim.pilotMass = std::clamp(wanted, 12.0, 60.0);
        nodes[sim.pilotNode].inverseMass = 1.0 / sim.pilotMass;
    }
    for (const std::size_t node : brakeHandles) {
        if (!controls.freeFlight) {
            sim.body->fixNode(node);
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
    const double airspeed =
        std::sqrt(2.0 * std::max(0.0, dynamicPressure) / kAirDensity);
    const double angle = controls.angleOfAttackDegrees * kDegreesToRadians;
    const softwing::Vec3 freestream =
        airspeed
        * rotateAbout(sim.restChordDirection, sim.restSpanAxis, -angle);

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
        double mass = 0.0;
        for (const softwing::Node &node : nodes) {
            if (node.inverseMass <= 0.0) {
                continue;
            }
            const double nodeMass = 1.0 / node.inverseMass;
            systemVelocity += nodeMass * node.velocity;
            mass += nodeMass;
        }
        if (mass > 0.0) {
            systemVelocity /= mass;
        }
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
        const double acrossWind =
            dot(cross(windDirection, chordInPlane), axis);
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
    const double angle = controls.angleOfAttackDegrees * kDegreesToRadians;
    return airspeed
           * rotateAbout(sim.restChordDirection, sim.restSpanAxis, -angle);
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

}  // namespace

void applyAerodynamicDrag(SimBody &sim, const SimControls &controls)
{
    if (!sim.body) {
        return;
    }
    sim.body->clearExternalForces();
    sim.lastDragForce = {};
    if (sim.faceAero.empty()) {
        return;
    }

    const softwing::Vec3 relative =
        freestreamVelocity(sim, controls)
        - (controls.freeFlight ? systemVelocityOf(sim) : softwing::Vec3{});
    const double speed = length(relative);
    if (speed <= 1.0e-6) {
        return;
    }
    const softwing::Vec3 windDirection = relative / speed;
    const double dynamicPressure =
        std::min(0.5 * kAirDensity * speed * speed,
                 kMaximumDynamicPressureRatio
                     * std::max(0.0, controls.pressurePascal));

    // Two terms per unit of wetted area. Friction is what the fabric and the
    // lines cost just by being there; the induced term is the price of
    // making lift at all, and it is the one that rewards a long thin wing.
    // Wetted area is about twice the planform, which is where the halving
    // in the induced factor comes from.
    const double induced =
        1.0 / (2.0 * kPi * std::max(1.0, sim.aspectRatio)
               * kSpanEfficiency);

    // The pressure field's own along-wind resultant is thrust, and a large
    // one: an inviscid Cp integrated over a curved leading edge recovers
    // the full leading-edge suction that a real wing loses to viscosity.
    // Raising the suction until the lift is realistic only makes it worse,
    // because both scale together. So the along-wind direction is taken off
    // the pressure field entirely — cancelled here and replaced by a drag
    // model. The pressure field keeps its job of shaping the fabric, which
    // is what it is good at; it was never going to set a glide ratio.
    const auto &nodes = sim.body->nodes();
    const auto &triangles = sim.body->triangles();
    const double pressureAlongWind =
        dot(aerodynamicForce(sim), windDirection);

    double totalArea = 0.0;
    std::vector<double> faceArea(sim.skinTriangleCount, 0.0);
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Triangle &tri = triangles[face];
        const softwing::Vec3 &a = nodes[tri.a].position;
        const softwing::Vec3 &b = nodes[tri.b].position;
        const softwing::Vec3 &c = nodes[tri.c].position;
        faceArea[face] = length(0.5 * cross(b - a, c - a));
        totalArea += faceArea[face];
    }
    if (!(totalArea > 0.0)) {
        return;
    }

    softwing::Vec3 total;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Triangle &tri = triangles[face];
        const double area = faceArea[face];
        const std::size_t rib = sim.faceAero[face].rib;
        const double lift = rib < sim.ribLiftCoefficient.size()
                                ? sim.ribLiftCoefficient[rib]
                                : 0.0;
        const double coefficient =
            kFrictionCoefficient + induced * lift * lift;
        // Spread the cancellation by area, so it does not distort the
        // spanwise load the way lumping it anywhere would.
        const double magnitude = dynamicPressure * coefficient * area
                                 - pressureAlongWind * area / totalArea;
        const softwing::Vec3 force = magnitude * windDirection;
        total += force;
        const softwing::Vec3 share = force / 3.0;
        sim.body->addForce(tri.a, share);
        sim.body->addForce(tri.b, share);
        sim.body->addForce(tri.c, share);
    }
    sim.lastDragForce = total;
}

AeroSummary aerodynamicSummary(const SimBody &sim,
                               const SimControls &controls)
{
    AeroSummary summary;
    if (!sim.body) {
        return summary;
    }
    summary.force = aerodynamicForce(sim) + sim.lastDragForce;
    const softwing::Vec3 relative =
        freestreamVelocity(sim, controls)
        - (controls.freeFlight ? systemVelocityOf(sim) : softwing::Vec3{});
    if (length(relative) <= 1.0e-6) {
        return summary;
    }
    const softwing::Vec3 windDirection = normalized(relative);
    summary.drag = dot(summary.force, windDirection);
    summary.lift =
        length(summary.force - summary.drag * windDirection);
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
    auto &constraints = sim.body->constraints();
    for (const BrakeLine &brake : sim.brakeLines) {
        if (brake.constraint >= constraints.size()) {
            continue;
        }
        const double pull =
            brake.left ? controls.brakeLeft : controls.brakeRight;
        constraints[brake.constraint].restLength =
            std::max(0.05, brake.restLength - pull);
    }
    applyPressure(sim, controls);
    // Drag is part of the free-flight experiment, not of the pinned tab.
    // Pinned, the canopy hangs on fixed carabiners and an along-wind force
    // just leans it against its lines; it also lands on a lift model that
    // is not yet calibrated to carry it (see docs/xpbd-performance.md).
    if (controls.freeFlight) {
        applyAerodynamicDrag(sim, controls);
    }

    softwing::StepSettings settings;
    settings.timeStep = simulationTimeStep;
    settings.substeps = controls.substeps;
    settings.constraintIterations = controls.constraintIterations;
    // Free flight puts real gravity on an unpinned system, so the damping
    // has to come down too or it would eat the pendulum it exists to show.
    // Pinned, there is nothing to fall and the old heavy damping keeps the
    // fabric quiet.
    settings.gravity =
        controls.freeFlight
            ? softwing::Vec3{0.0, 0.0, -gravityMetresPerSecondSquared}
            : softwing::Vec3{0.0, 0.0, 0.0};
    settings.velocityDampingPerSecond =
        controls.freeFlight ? systemDampingPerSecond : 3.0;
    settings.workerThreads = controls.workerThreads;
    settings.performanceProfile = controls.performanceProfile;
    sim.body->step(settings);

    if (controls.freeFlight) {
        recentreSystem(sim);
    }
}

}  // namespace lep::playground
