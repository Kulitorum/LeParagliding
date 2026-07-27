#include "playground_page.h"

#include "softwing/soft_body.h"

#include <QDebug>
#include <QFile>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QSurfaceFormat>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector3D>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <utility>
#include <vector>

namespace {

constexpr double metresPerMillimetre = 0.001;
constexpr double fabricArealDensity = 0.045;   // kg/m^2
// Tuned on the Swoop harness: at 1e-8/1e-9 with 4 substeps and 25
// iterations the wing settles at +23% volume under 80 Pa instead of
// creeping past its own fabric.
constexpr double skinCompliance = 1.0e-8;      // XPBD, m/N
constexpr double lineCompliance = 1.0e-9;
// Heavy enough that ~1 kN of lift through the cascade keeps a workable
// mass ratio for the solver (5 g junctions let the wing creep skyward).
constexpr double lineJunctionMass = 0.05;      // kg
constexpr double ribCentroidMass = 0.002;
constexpr double anchorBandMetres = 0.3;
constexpr double lineAttachRadiusMetres = 0.12;
constexpr double maximumBrakeTravelMetres = 0.6;

struct SimLine
{
    softwing::Vec3 a;
    softwing::Vec3 b;
    bool brake = false;
};

struct SimStrap
{
    std::vector<softwing::Vec3> a;
    std::vector<softwing::Vec3> b;
};

struct SimMesh
{
    std::vector<softwing::Vec3> nodes;
    std::vector<std::array<int, 4>> quads;
    std::vector<std::vector<int>> ribLoops;
    std::vector<SimStrap> straps;
    std::vector<SimLine> lines;
};

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
         root.value(QLatin1String("ribLoops")).toArray()) {
        std::vector<int> loop;
        for (const QJsonValue &node : value.toArray()) {
            loop.push_back(node.toInt());
        }
        mesh.ribLoops.push_back(std::move(loop));
    }
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

} // namespace

// A minimal orbit-camera OpenGL view running the XPBD body on a timer.
// No signals or slots of its own, so no Q_OBJECT / moc involvement.
class PlaygroundView : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    explicit PlaygroundView(QWidget *parent = nullptr)
        : QOpenGLWidget(parent)
    {
        // Render into an own native window instead of joining Qt's
        // whole-window GL composition: the main window mixes raster
        // widgets with the OCCT viewport's native swapchain, and letting
        // this widget flip the window into GL composition both blacked
        // out the other GL tabs and left a ghost copy of this view at its
        // pre-layout geometry.
        setAttribute(Qt::WA_NativeWindow, true);
        timer_ = new QTimer(this);
        timer_->setInterval(16);
        connect(timer_, &QTimer::timeout, this, [this] {
            stepSimulation();
            update();
        });
    }

    QString buildFromMesh(const SimMesh &mesh)
    {
        auto body = std::make_unique<softwing::SoftBody>();
        topFaces_.clear();
        lineSegments_.clear();
        anchors_.clear();

        // Skin triangles, oriented outward for the pressure field.
        std::vector<std::array<int, 3>> triangles;
        triangles.reserve(mesh.quads.size() * 2);
        for (const auto &quad : mesh.quads) {
            triangles.push_back({quad[0], quad[1], quad[2]});
            triangles.push_back({quad[0], quad[2], quad[3]});
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
            body->addNode(mesh.nodes[index],
                          std::max(masses[index], 5.0e-4));
        }
        for (const auto &tri : triangles) {
            body->addTriangle(static_cast<std::size_t>(tri[0]),
                              static_cast<std::size_t>(tri[1]),
                              static_cast<std::size_t>(tri[2]));
        }
        skinTriangleCount_ = triangles.size();

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
                topFaces_.push_back(face);
            }
        }

        // Stretch constraints on every unique edge, plus the second quad
        // diagonal for shear.
        std::set<std::pair<int, int>> edges;
        const auto addEdge = [&](int a, int b) {
            if (a == b) {
                return;
            }
            if (edges.insert({std::min(a, b), std::max(a, b)}).second) {
                body->addDistanceConstraint(
                    static_cast<std::size_t>(a),
                    static_cast<std::size_t>(b),
                    length(mesh.nodes[static_cast<std::size_t>(b)]
                           - mesh.nodes[static_cast<std::size_t>(a)]),
                    skinCompliance);
            }
        };
        for (const auto &tri : triangles) {
            addEdge(tri[0], tri[1]);
            addEdge(tri[1], tri[2]);
            addEdge(tri[2], tri[0]);
        }
        for (const auto &quad : mesh.quads) {
            addEdge(quad[1], quad[3]);
        }

        // Rib webs: perimeter plus spokes to a centroid node. Constraints
        // only — no faces, so the uniform skin pressure stays valid.
        std::size_t centrePin = 0;
        double centrePinAbsX = std::numeric_limits<double>::max();
        for (const auto &loop : mesh.ribLoops) {
            softwing::Vec3 centroid;
            for (const int node : loop) {
                centroid += mesh.nodes[static_cast<std::size_t>(node)];
            }
            centroid /= static_cast<double>(loop.size());
            const std::size_t centre =
                body->addNode(centroid, ribCentroidMass);
            if (std::abs(centroid.x) < centrePinAbsX) {
                centrePinAbsX = std::abs(centroid.x);
                centrePin = centre;
            }
            for (std::size_t index = 0; index < loop.size(); ++index) {
                const int node = loop[index];
                const int next =
                    loop[(index + 1) % loop.size()];
                addEdge(node, next);
                body->addDistanceConstraint(
                    static_cast<std::size_t>(node),
                    centre,
                    length(mesh.nodes[static_cast<std::size_t>(node)]
                           - centroid),
                    skinCompliance);
            }
        }
        // No pin here any more: the lift pressure keeps every line taut
        // against the fixed pilot-end anchors, and the wing hangs in its
        // lines like the real thing.
        (void)centrePin;

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
            for (std::size_t sample = 0; sample < strap.a.size(); ++sample) {
                const int nodeA = nearestMeshNode(strap.a[sample]);
                const int nodeB = nearestMeshNode(strap.b[sample]);
                if (nodeA < 0 || nodeB < 0 || nodeA == nodeB) {
                    continue;
                }
                body->addDistanceConstraint(
                    static_cast<std::size_t>(nodeA),
                    static_cast<std::size_t>(nodeB),
                    length(mesh.nodes[static_cast<std::size_t>(nodeB)]
                           - mesh.nodes[static_cast<std::size_t>(nodeA)]),
                    skinCompliance);
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
            body->addCableConstraint(
                a, b, length(line.b - line.a), lineCompliance);
            lineSegments_.push_back({a, b, line.brake});
        }
        for (const auto &[key, node] : junctions) {
            const softwing::Vec3 position = body->nodes()[node].position;
            if (position.z < lowestZ + anchorBandMetres) {
                // Pilot-end anchor (the carabiners); they stay put.
                body->fixNode(node);
                anchors_.push_back({node, position, false});
                continue;
            }
            // Tie upper junctions to the canopy when they sit on it.
            double bestDistance = lineAttachRadiusMetres;
            int bestSkinNode = -1;
            for (std::size_t skin = 0; skin < mesh.nodes.size(); ++skin) {
                const double distance =
                    length(mesh.nodes[skin] - position);
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

        // The captured line set ends at the carabiners — there is no brake
        // handle node. Synthesize one per side, hanging below the
        // carabiner and cabled to the tops of the brake segments that
        // leave the lowest brake junction, so pulling it hauls the brake
        // cascade like the real handle would.
        for (const double side : {-1.0, 1.0}) {
            std::size_t lowestBrake = 0;
            double lowestBrakeZ = std::numeric_limits<double>::max();
            softwing::Vec3 carabiner;
            double carabinerZ = std::numeric_limits<double>::max();
            bool sawBrake = false;
            for (const auto &[key, node] : junctions) {
                const softwing::Vec3 position =
                    body->nodes()[node].position;
                if (position.x * side <= 0.0) {
                    continue;
                }
                if (position.z < carabinerZ) {
                    carabinerZ = position.z;
                    carabiner = position;
                }
                for (const LineSegment &segment : lineSegments_) {
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
            for (const LineSegment &segment : lineSegments_) {
                if (!segment.brake) {
                    continue;
                }
                if (segment.a == lowestBrake) {
                    brakeTops.push_back(segment.b);
                } else if (segment.b == lowestBrake) {
                    brakeTops.push_back(segment.a);
                }
            }
            softwing::Vec3 handlePosition = carabiner;
            handlePosition.z -= 0.3;
            const std::size_t handle =
                body->addNode(handlePosition, lineJunctionMass);
            body->fixNode(handle);
            for (const std::size_t top : brakeTops) {
                body->addCableConstraint(
                    handle,
                    top,
                    length(body->nodes()[top].position - handlePosition),
                    lineCompliance);
                lineSegments_.push_back({handle, top, true});
            }
            anchors_.push_back({handle, handlePosition, true});
        }

        body_ = std::move(body);
        applyPressure();

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
        target_ = QVector3D(static_cast<float>((low.x + high.x) / 2),
                            static_cast<float>((low.y + high.y) / 2),
                            static_cast<float>((low.z + high.z) / 2));
        distance_ =
            static_cast<float>(2.0 * length(high - low));

        setRunning(true);
        return {};
    }

    void setPressurePascal(double pressure)
    {
        pressurePascal_ = pressure;
        applyPressure();
    }

    void setLiftPascal(double lift)
    {
        liftPascal_ = lift;
        applyPressure();
    }

    void setBrakePull(double leftMetres, double rightMetres)
    {
        brakeLeft_ = leftMetres;
        brakeRight_ = rightMetres;
    }

    void setRunning(bool running)
    {
        if (running && body_) {
            timer_->start();
        } else {
            timer_->stop();
        }
    }

    bool isRunning() const { return timer_->isActive(); }
    bool hasBody() const { return body_ != nullptr; }
    QString lastSimError() const { return simError_; }
    QString lastGlError() const { return glError_; }

protected:
    void initializeGL() override
    {
        initializeOpenGLFunctions();
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.10F, 0.11F, 0.13F, 1.0F);

        // The application's default surface format is XFLR5's, which is a
        // 3.3 core profile on most machines: core GLSL syntax and a bound
        // VAO are mandatory there, while older configurations fall back
        // to a compatibility context and 1.10 syntax.
        const bool coreProfile =
            context()->format().profile() == QSurfaceFormat::CoreProfile;
        const QString vertexSource =
            coreProfile
                ? QStringLiteral(
                      "#version 330 core\n"
                      "in vec3 position;\n"
                      "in vec3 normal;\n"
                      "uniform mat4 mvp;\n"
                      "out vec3 vNormal;\n"
                      "void main() {\n"
                      "    vNormal = normal;\n"
                      "    gl_Position = mvp * vec4(position, 1.0);\n"
                      "}\n")
                : QStringLiteral(
                      "attribute vec3 position;\n"
                      "attribute vec3 normal;\n"
                      "uniform mat4 mvp;\n"
                      "varying vec3 vNormal;\n"
                      "void main() {\n"
                      "    vNormal = normal;\n"
                      "    gl_Position = mvp * vec4(position, 1.0);\n"
                      "}\n");
        const QString shading =
            QStringLiteral(
                "    float shade = lit\n"
                "        ? 0.25 + 0.75 * abs(dot(normalize(vNormal),\n"
                "                                normalize(vec3(0.3, -0.5, "
                "0.8))))\n"
                "        : 1.0;\n");
        const QString fragmentSource =
            coreProfile
                ? QStringLiteral(
                      "#version 330 core\n"
                      "uniform vec4 color;\n"
                      "uniform bool lit;\n"
                      "in vec3 vNormal;\n"
                      "out vec4 fragColor;\n"
                      "void main() {\n")
                      + shading
                      + QStringLiteral(
                          "    fragColor = vec4(color.rgb * shade, "
                          "color.a);\n"
                          "}\n")
                : QStringLiteral(
                      "uniform vec4 color;\n"
                      "uniform bool lit;\n"
                      "varying vec3 vNormal;\n"
                      "void main() {\n")
                      + shading
                      + QStringLiteral(
                          "    gl_FragColor = vec4(color.rgb * shade, "
                          "color.a);\n"
                          "}\n");

        program_ = new QOpenGLShaderProgram(this);
        if (!program_->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                               vertexSource)
            || !program_->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                                  fragmentSource)
            || !program_->link()) {
            glError_ = QStringLiteral("OpenGL shader error: %1")
                           .arg(program_->log().trimmed());
            return;
        }
        vao_.create();
        buffer_.create();
    }

    void resizeGL(int, int) override {}

    void paintGL() override
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (!body_ || !glError_.isEmpty()) {
            return;
        }

        QMatrix4x4 projection;
        projection.perspective(
            40.0F,
            width() > 0 ? float(width()) / float(std::max(height(), 1))
                        : 1.0F,
            0.02F,
            500.0F);
        QMatrix4x4 view;
        view.translate(0, 0, -distance_);
        view.rotate(pitch_, 1, 0, 0);
        view.rotate(yaw_, 0, 0, 1);
        view.translate(-target_);
        const QMatrix4x4 mvp = projection * view;

        const auto &nodes = body_->nodes();
        const auto &triangles = body_->triangles();

        vertexScratch_.clear();
        vertexScratch_.reserve(skinTriangleCount_ * 18);
        for (std::size_t face = 0; face < skinTriangleCount_; ++face) {
            const auto &tri = triangles[face];
            const softwing::Vec3 &a = nodes[tri.a].position;
            const softwing::Vec3 &b = nodes[tri.b].position;
            const softwing::Vec3 &c = nodes[tri.c].position;
            const softwing::Vec3 normal = normalized(cross(b - a, c - a));
            for (const softwing::Vec3 *point : {&a, &b, &c}) {
                vertexScratch_.push_back(static_cast<float>(point->x));
                vertexScratch_.push_back(static_cast<float>(point->y));
                vertexScratch_.push_back(static_cast<float>(point->z));
                vertexScratch_.push_back(static_cast<float>(normal.x));
                vertexScratch_.push_back(static_cast<float>(normal.y));
                vertexScratch_.push_back(static_cast<float>(normal.z));
            }
        }
        const int skinFloats = static_cast<int>(vertexScratch_.size());
        for (const LineSegment &segment : lineSegments_) {
            for (const std::size_t node : {segment.a, segment.b}) {
                const softwing::Vec3 &point = nodes[node].position;
                vertexScratch_.push_back(static_cast<float>(point.x));
                vertexScratch_.push_back(static_cast<float>(point.y));
                vertexScratch_.push_back(static_cast<float>(point.z));
                vertexScratch_.push_back(segment.brake ? 1.0F : 0.0F);
                vertexScratch_.push_back(0.0F);
                vertexScratch_.push_back(0.0F);
            }
        }

        program_->bind();
        QOpenGLVertexArrayObject::Binder vaoBinder(&vao_);
        program_->setUniformValue("mvp", mvp);
        buffer_.bind();
        buffer_.allocate(
            vertexScratch_.data(),
            static_cast<int>(vertexScratch_.size() * sizeof(float)));
        program_->enableAttributeArray("position");
        program_->setAttributeBuffer(
            "position", GL_FLOAT, 0, 3, 6 * sizeof(float));
        program_->enableAttributeArray("normal");
        program_->setAttributeBuffer(
            "normal", GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));

        program_->setUniformValue("lit", true);
        program_->setUniformValue(
            "color", QVector4D(0.72F, 0.78F, 0.88F, 1.0F));
        glDrawArrays(GL_TRIANGLES, 0, skinFloats / 6);

        program_->setUniformValue("lit", false);
        program_->setUniformValue(
            "color", QVector4D(0.55F, 0.62F, 0.55F, 1.0F));
        glDrawArrays(GL_LINES,
                     skinFloats / 6,
                     (static_cast<int>(vertexScratch_.size()) - skinFloats)
                         / 6);
        buffer_.release();
        program_->release();
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        lastMouse_ = event->position();
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        const QPointF delta = event->position() - lastMouse_;
        lastMouse_ = event->position();
        if (event->buttons() & Qt::LeftButton) {
            yaw_ += static_cast<float>(delta.x()) * 0.4F;
            pitch_ += static_cast<float>(delta.y()) * 0.4F;
            pitch_ = std::clamp(pitch_, -90.0F, 90.0F);
            update();
        }
    }

    void wheelEvent(QWheelEvent *event) override
    {
        distance_ *=
            event->angleDelta().y() > 0 ? 1.0F / 1.15F : 1.15F;
        update();
    }

private:
    struct LineSegment
    {
        std::size_t a = 0;
        std::size_t b = 0;
        bool brake = false;
    };
    struct Anchor
    {
        std::size_t node = 0;
        softwing::Vec3 restPosition;
        bool brakeHandle = false;
    };

    void applyPressure()
    {
        if (!body_) {
            return;
        }
        body_->setUniformPressureDifference(
            body_->surfaceGroup(0, skinTriangleCount_), pressurePascal_);
        stampLift();
    }

    // Orientation-aware lift, restamped every frame: each top face gets
    // extra pressure along its live normal, weighted by how much that
    // normal still points up. A pure follower force rotating with the
    // wing has no preferred attitude and winds the wing around its span
    // axis forever; the weighting gives it a restoring pitch moment and
    // a stable upright equilibrium.
    void stampLift()
    {
        const auto &nodes = body_->nodes();
        const auto &triangles = body_->triangles();
        for (const std::size_t face : topFaces_) {
            const auto &tri = triangles[face];
            const softwing::Vec3 normal = normalized(
                cross(nodes[tri.b].position - nodes[tri.a].position,
                      nodes[tri.c].position - nodes[tri.a].position));
            body_->setFacePressureDifference(
                face,
                pressurePascal_ + liftPascal_ * std::max(0.0, normal.z));
        }
    }

    void stepSimulation()
    {
        if (!body_) {
            return;
        }
        // Pull brake handles straight down; other anchors stay put.
        for (const Anchor &anchor : anchors_) {
            softwing::Vec3 position = anchor.restPosition;
            if (anchor.brakeHandle) {
                position.z -= anchor.restPosition.x < 0.0 ? brakeLeft_
                                                          : brakeRight_;
            }
            body_->nodes()[anchor.node].position = position;
            body_->nodes()[anchor.node].previousPosition = position;
            body_->nodes()[anchor.node].velocity = {};
        }
        stampLift();

        softwing::StepSettings settings;
        settings.timeStep = 1.0 / 60.0;
        settings.substeps = 4;
        settings.constraintIterations = 30;
        settings.gravity = {0.0, 0.0, 0.0};
        settings.velocityDampingPerSecond = 3.0;
        try {
            body_->step(settings);
        } catch (const std::exception &exception) {
            simError_ = QString::fromUtf8(exception.what());
            setRunning(false);
        }
    }

    std::unique_ptr<softwing::SoftBody> body_;
    std::size_t skinTriangleCount_ = 0;
    std::vector<std::size_t> topFaces_;
    std::vector<LineSegment> lineSegments_;
    std::vector<Anchor> anchors_;
    std::vector<float> vertexScratch_;
    double pressurePascal_ = 80.0;
    double liftPascal_ = 30.0;
    double brakeLeft_ = 0.0;
    double brakeRight_ = 0.0;
    QString simError_;
    QString glError_;

    QTimer *timer_ = nullptr;
    QOpenGLShaderProgram *program_ = nullptr;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer buffer_{QOpenGLBuffer::VertexBuffer};
    QVector3D target_;
    float distance_ = 10.0F;
    float yaw_ = 30.0F;
    float pitch_ = -60.0F;
    QPointF lastMouse_;
};

PlaygroundPage::PlaygroundPage(QWidget *parent)
    : QWidget(parent)
{
    if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_DEBUG")) {
        qWarning() << "PlaygroundPage constructed"
                   << static_cast<void *>(this);
    }
    status_ = new QLabel(
        QStringLiteral("Run a preview or export first — the Playground "
                       "loads the calculated wing."),
        this);
    status_->setWordWrap(true);

    const auto makeSlider = [this](int maximum, int value) {
        auto *slider = new QSlider(Qt::Horizontal, this);
        slider->setRange(0, maximum);
        slider->setValue(value);
        return slider;
    };
    pressure_ = makeSlider(300, 80);
    lift_ = makeSlider(200, 30);
    leftBrake_ = makeSlider(100, 0);
    rightBrake_ = makeSlider(100, 0);
    runButton_ = new QPushButton(QStringLiteral("Pause"), this);
    runButton_->setCheckable(true);

    auto *controls = new QHBoxLayout;
    controls->addWidget(new QLabel(QStringLiteral("Pressure"), this));
    controls->addWidget(pressure_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Lift"), this));
    controls->addWidget(lift_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Left brake"), this));
    controls->addWidget(leftBrake_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Right brake"), this));
    controls->addWidget(rightBrake_, 1);
    controls->addWidget(runButton_);

    layout_ = new QVBoxLayout(this);
    layout_->addWidget(status_);
    layout_->addLayout(controls);

    connect(pressure_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setPressurePascal(static_cast<double>(value));
        }
    });
    connect(lift_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setLiftPascal(static_cast<double>(value));
        }
    });
    const auto pushBrakes = [this] {
        if (view_ != nullptr) {
            view_->setBrakePull(
                leftBrake_->value() / 100.0 * maximumBrakeTravelMetres,
                rightBrake_->value() / 100.0 * maximumBrakeTravelMetres);
        }
    };
    connect(leftBrake_, &QSlider::valueChanged, this, pushBrakes);
    connect(rightBrake_, &QSlider::valueChanged, this, pushBrakes);
    connect(runButton_, &QPushButton::toggled, this, [this](bool paused) {
        if (view_ != nullptr) {
            view_->setRunning(!paused);
        }
        runButton_->setText(paused ? QStringLiteral("Run")
                                   : QStringLiteral("Pause"));
    });
}

void PlaygroundPage::ensureView()
{
    if (view_ != nullptr || creatingView_) {
        return;
    }
    creatingView_ = true;
    if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_DEBUG")) {
        qWarning() << "ensureView creating view on page"
                   << static_cast<void *>(this);
    }
    view_ = new PlaygroundView(this);
    layout_->addWidget(view_, 1);
    view_->setPressurePascal(static_cast<double>(pressure_->value()));
    view_->setLiftPascal(static_cast<double>(lift_->value()));
    view_->setBrakePull(
        leftBrake_->value() / 100.0 * maximumBrakeTravelMetres,
        rightBrake_->value() / 100.0 * maximumBrakeTravelMetres);
    view_->show();
    creatingView_ = false;
}

void PlaygroundPage::setSimMeshPath(const QString &path)
{
    // Previews run the engine in a temporary directory that is removed
    // right after the model is handed over, so the mesh must be read here
    // and now — by the time the tab is first opened the file is gone.
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        status_->setText(
            QStringLiteral("Could not read the simulation mesh %1")
                .arg(path));
        return;
    }
    pendingData_ = file.readAll();
    if (isVisible()) {
        loadIfPending();
    }
}

void PlaygroundPage::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    ensureView();
    loadIfPending();
    if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_DEBUG")) {
        QTimer::singleShot(4000, this, [this] {
            qWarning() << "PlaygroundPage geometry" << geometry();
            const QList<QWidget *> children =
                findChildren<QWidget *>(Qt::FindDirectChildrenOnly);
            for (QWidget *child : children) {
                qWarning() << " child" << child->metaObject()->className()
                           << child->geometry() << "visible"
                           << child->isVisible();
            }
            qWarning() << " view_" << static_cast<void *>(view_)
                       << (view_ != nullptr ? view_->geometry() : QRect());
        });
    }
}

void PlaygroundPage::loadIfPending()
{
    if (view_ == nullptr || pendingData_.isEmpty()) {
        return;
    }
    QString error;
    const std::optional<SimMesh> mesh = parseSimMesh(pendingData_, error);
    if (!mesh) {
        status_->setText(error);
        pendingData_.clear();
        return;
    }
    const QString buildError = view_->buildFromMesh(*mesh);
    if (!buildError.isEmpty()) {
        status_->setText(buildError);
        pendingData_.clear();
        return;
    }
    pendingData_.clear();
    status_->setText(
        QStringLiteral("Toy simulation · %1 nodes, %2 skin quads, %3 line "
                       "segments · drag to orbit, wheel to zoom. Not "
                       "engineering — a sandbox.")
            .arg(mesh->nodes.size())
            .arg(mesh->quads.size())
            .arg(mesh->lines.size()));
    // Shader problems only surface once the first frame renders; a silent
    // black view is undiagnosable, so report them here.
    QTimer::singleShot(500, this, [this] {
        if (!view_->lastGlError().isEmpty()) {
            status_->setText(view_->lastGlError());
        }
    });
}
