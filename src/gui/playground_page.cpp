#include "playground_page.h"

#include "playground_analysis.h"
#include "playground_metrics.h"
#include "playground_sim.h"

#include "softwing/soft_body.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QFile>
#include <QGuiApplication>
#include <QHBoxLayout>
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
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector3D>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

// The wing itself — mesh parsing, refinement, body assembly and the step —
// lives in playground_sim.{h,cpp} so the headless solver benchmark can drive
// exactly the same simulation. What is left here is the view: camera,
// shading, colour ramps and the control panel.
using lep::playground::LineSegment;
using lep::playground::RenderFace;
using lep::playground::SimBody;
using lep::playground::SimBuildOptions;
using lep::playground::SimControls;
using lep::playground::SimMesh;
using lep::playground::SimSurface;
using lep::playground::buildSimBody;
using lep::playground::defaultRibLayers;
using lep::playground::defaultRibStationSplit;
using lep::playground::noConstraint;
using lep::playground::parseSimMesh;
using lep::playground::refineSimMesh;
using lep::playground::simSurfaceCount;

namespace {

constexpr float cameraFieldOfViewDegrees = 40.0F;
constexpr float degreesToRadians = 3.14159265358979323846F / 180.0F;
constexpr double maximumBrakeTravelMetres = 0.6;
// Stretch that saturates the stress ramp, adjustable so slack fabric and
// hard-loaded seams can each be examined at a useful contrast. The legend
// reports the live peak next to it.
constexpr double defaultStressFullScaleStrain = 0.01;   // 1% of rest length
constexpr double maximumStressFullScaleStrain = 0.05;   // slider top, 5%
// Suspension lines are nearly inextensible (see lineCompliance), so stretch
// tells you nothing about them: their load is read from the solver's
// multiplier instead, which is a force. Newtons per line.
constexpr double defaultLineFullScaleNewtons = 100.0;
constexpr double maximumLineFullScaleNewtons = 500.0;
// Cadence of the deviation/slack field refresh, in simulation steps. The
// fields are a measurement pass over the whole body — cheap enough for a
// few hertz, pointless (and allocating) every frame.
constexpr int fieldRefreshSteps = 15;
// How close a Ctrl-click must land to a projected line junction to grab it.
constexpr double grabPickRadiusPixels = 14.0;
// Grey for faces that are drawn while stress colouring is on but carry no
// meaningful stress of their own — the simple rib web.
const QVector3D uncolouredTint(0.58F, 0.60F, 0.63F);
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
        controls_.workerThreads = lep::playground::playgroundWorkerThreads();
        // The GUI tunnel imposes the wing-level polar load by default so
        // the line-load numbers mean something (the bench keeps the raw
        // pressure field for its baselines). Must match the page's
        // Flight-load checkbox default.
        controls_.flightLoad = true;
        timer_ = new QTimer(this);
        timer_->setInterval(16);
        connect(timer_, &QTimer::timeout, this, [this] {
            stepSimulation();
            update();
        });
    }

    QString buildFromMesh(const SimMesh &mesh)
    {
        // Retained so free flight can be toggled without another engine
        // run: the pilot only exists on a body built for free flight, so
        // the toggle is a rebuild.
        mesh_ = mesh;
        rebuildBody();
        return {};
    }

    void rebuildBody()
    {
        // A grab holds a node of the body being discarded; drop it rather
        // than let a stale index pull on the fresh wing.
        if (grabbing_) {
            grabbing_ = false;
            setCursor(flyMode_ ? Qt::CrossCursor : Qt::ArrowCursor);
        }
        sim_ = buildSimBody(mesh_, buildOptions_, controls_);
        // The baseline is the design shape itself: buildSimBody leaves
        // every node at the mesh's rest pose (the free-flight launch only
        // stamps velocities), so capturing here reads exactly the designed
        // geometry the instruments compare against.
        baseline_ = lep::playground::captureShapeBaseline(sim_);
        deviationField_.clear();
        nodeTensile_.clear();
        nodeSlack_.clear();
        refreshColourField();

        const softwing::Vec3 low = sim_.boundsLow;
        const softwing::Vec3 high = sim_.boundsHigh;
        softwing::Vec3 focus = 0.5 * (low + high);
        double extent = length(high - low);
        if (controls_.freeFlight
            && sim_.pilotNode != noConstraint) {
            // The flying system is re-centred on its mass centre every
            // step, which sits close to the pilot. Frame the whole
            // pendulum — canopy above, pilot below — in that frame.
            softwing::Vec3 centreOfMass;
            double mass = 0.0;
            const auto &nodes = sim_.body->nodes();
            for (const softwing::Node &node : nodes) {
                if (node.inverseMass <= 0.0) {
                    continue;
                }
                const double nodeMass = 1.0 / node.inverseMass;
                centreOfMass += nodeMass * node.position;
                mass += nodeMass;
            }
            if (mass > 0.0) {
                centreOfMass /= mass;
            }
            const softwing::Vec3 pilot =
                nodes[sim_.pilotNode].position;
            focus = 0.5 * (0.5 * (low + high) + pilot) - centreOfMass;
            extent = std::max(extent,
                              1.4 * length(0.5 * (low + high) - pilot));
        }
        target_ = QVector3D(static_cast<float>(focus.x),
                            static_cast<float>(focus.y),
                            static_cast<float>(focus.z));
        distance_ = static_cast<float>(2.0 * extent);

        setRunning(true);
    }

    // Free flight rebuilds the body: pinned and flying wings differ in
    // structure (pilot mass, fixed anchors), not just in settings.
    void setFreeFlight(bool enabled)
    {
        if (controls_.freeFlight == enabled) {
            return;
        }
        controls_.freeFlight = enabled;
        if (!mesh_.nodes.empty()) {
            rebuildBody();
        }
        update();
    }

    bool freeFlight() const { return controls_.freeFlight; }

    // Back to the rest pose (and, in free flight, a fresh launch on the
    // glide), from the retained mesh — no engine run needed.
    void resetSimulation()
    {
        if (!mesh_.nodes.empty()) {
            rebuildBody();
        }
    }

    // The refined mesh the body was built from, for a worker that wants
    // to build an identical twin without re-refining.
    const SimMesh &simMesh() const { return mesh_; }

    // Takes over the pose, motion and aerodynamic bookkeeping of a body
    // settled elsewhere — the max-quality settle worker's result. The
    // twin was built from the same mesh and options, so its node,
    // constraint and triangle tables are a prefix of the live ones (a
    // grab may have appended an anchor node and cables past the prefix;
    // those stay untouched, and the grab cable is slack when no grab is
    // held). Copying the constraint multipliers keeps the line-tension
    // readouts honest while the sim sits paused for review; copying the
    // alpha filter state keeps the HUD's angle honest too. Returns an
    // error when the tables do not line up (the live body was rebuilt
    // to a different mesh while the worker ran).
    QString adoptSettledState(const SimBody &settled)
    {
        if (!sim_.body || !settled.body) {
            return QStringLiteral("no body to adopt into");
        }
        auto &nodes = sim_.body->nodes();
        const auto &settledNodes = settled.body->nodes();
        if (settled.canopyNodeCount != sim_.canopyNodeCount
            || settledNodes.size() > nodes.size()) {
            return QStringLiteral(
                "the wing changed while settling — result discarded");
        }
        for (std::size_t index = 0; index < settledNodes.size(); ++index) {
            nodes[index].position = settledNodes[index].position;
            nodes[index].previousPosition =
                settledNodes[index].previousPosition;
            nodes[index].velocity = settledNodes[index].velocity;
        }
        auto &constraints = sim_.body->constraints();
        const auto &settledConstraints = settled.body->constraints();
        const std::size_t constraintCount =
            std::min(constraints.size(), settledConstraints.size());
        for (std::size_t index = 0; index < constraintCount; ++index) {
            constraints[index].accumulatedLambda =
                settledConstraints[index].accumulatedLambda;
        }
        const auto &settledTriangles = settled.body->triangles();
        const std::size_t triangleCount = std::min(
            sim_.skinTriangleCount, settledTriangles.size());
        for (std::size_t face = 0; face < triangleCount; ++face) {
            sim_.body->setFacePressureDifference(
                face, settledTriangles[face].pressureDifference);
        }
        sim_.ribLiftCoefficient = settled.ribLiftCoefficient;
        sim_.alphaFilteredRadians = settled.alphaFilteredRadians;
        sim_.alphaSlowRadians = settled.alphaSlowRadians;
        sim_.alphaRateRadiansPerSecond = settled.alphaRateRadiansPerSecond;
        sim_.lastForceResidual = settled.lastForceResidual;
        sim_.lastPitchResidual = settled.lastPitchResidual;
        sim_.lastAeroForce = settled.lastAeroForce;
        sim_.lastLift = settled.lastLift;
        sim_.lastDrag = settled.lastDrag;
        sim_.lastGlideRatio = settled.lastGlideRatio;
        sim_.lastAlphaDegrees = settled.lastAlphaDegrees;
        sim_.lastAirspeed = settled.lastAirspeed;
        refreshColourField();
        update();
        return {};
    }

    // Fly mode: the cursor's position over the view IS the brake input.
    // Top centre is hands-up; straight down pulls both brakes; moving
    // toward a side releases the opposite brake, so the pair gives full
    // two-brake control in real time. Esc leaves.
    void setFlyMode(bool enabled)
    {
        if (flyMode_ == enabled) {
            return;
        }
        flyMode_ = enabled;
        setMouseTracking(enabled);
        setCursor(enabled ? Qt::CrossCursor : Qt::ArrowCursor);
        if (enabled) {
            setFocus(Qt::OtherFocusReason);
            // The keyboard grab is what makes Esc reliable regardless of
            // which control happens to have focus; released on exit.
            grabKeyboard();
        } else {
            releaseKeyboard();
        }
    }

    bool flyMode() const { return flyMode_; }

    // The page mirrors fly-mode brake input back onto its sliders, and
    // needs to know when Esc ended the mode.
    void setFlyModeCallbacks(std::function<void(double, double)> brakes,
                             std::function<void()> exited)
    {
        flyBrakesChanged_ = std::move(brakes);
        flyModeExited_ = std::move(exited);
    }

    // One line for the flight label: what the wing is doing, in units a
    // pilot would use.
    QString flightReadout() const
    {
        if (!controls_.freeFlight || !sim_.body
            || sim_.lastAirspeed <= 0.0) {
            return {};
        }
        softwing::Vec3 velocity;
        double mass = 0.0;
        for (const softwing::Node &node : sim_.body->nodes()) {
            if (node.inverseMass <= 0.0) {
                continue;
            }
            const double nodeMass = 1.0 / node.inverseMass;
            velocity += nodeMass * node.velocity;
            mass += nodeMass;
        }
        if (mass > 0.0) {
            velocity /= mass;
        }
        return QStringLiteral(
                   "%1 km/h · sink %2 m/s · glide %3 · α %4° · pilot %5 kg")
            .arg(sim_.lastAirspeed * 3.6, 0, 'f', 0)
            .arg(-velocity.z, 0, 'f', 1)
            .arg(sim_.lastGlideRatio, 0, 'f', 1)
            .arg(sim_.lastAlphaDegrees, 0, 'f', 1)
            .arg(sim_.pilotMass, 0, 'f', 0);
    }

    void setPressurePascal(double pressure)
    {
        controls_.pressurePascal = pressure;
        applyPressure();
    }

    // Degrees between the airflow and the wing's rest chord. The load falls
    // out of the pressure field now, so this is the only handle the
    // aerodynamics needs — it replaces a slider that dialled in a fake force.
    void setAngleOfAttack(double degrees)
    {
        controls_.angleOfAttackDegrees = degrees;
        applyPressure();
    }

    // Takes the VIEWER's left and right. The solver's "left" cascade sits
    // at negative mesh x, which the default camera shows on the viewer's
    // right — so the two cross over here, in one place, rather than in
    // every caller. Before this the Left brake slider pulled the wing's
    // right side.
    void setBrakePull(double leftMetres, double rightMetres)
    {
        controls_.brakeLeft = rightMetres;
        controls_.brakeRight = leftMetres;
    }

    void setSurfaceVisible(SimSurface surface, bool visible)
    {
        surfaceVisible_[static_cast<std::size_t>(surface)] = visible;
        update();
    }

    void setLinesVisible(bool visible)
    {
        linesVisible_ = visible;
        update();
    }

    // Index into lep::playground::solverQualities. Applies from the next
    // frame; the body is untouched, only how hard it is solved.
    void setSolverQuality(int index)
    {
        const auto &table = lep::playground::solverQualities;
        const int count = static_cast<int>(std::size(table));
        const lep::playground::SolverQuality &chosen =
            table[std::clamp(index, 0, count - 1)];
        controls_.substeps = chosen.substeps;
        controls_.constraintIterations = chosen.iterations;
    }

    // Skin heatmap source. The page's combo items are in this order.
    enum class ColorMode
    {
        Plain,
        Stress,
        Deviation,
        Slack,
    };

    void setColorMode(ColorMode mode)
    {
        if (colorMode_ == mode) {
            return;
        }
        colorMode_ = mode;
        // A paused tunnel must colour immediately too; the step-count
        // cadence only refreshes while the solver runs.
        refreshColourField();
        update();
    }

    // Wind-tunnel loading: impose the wing-level polar pass in pinned mode
    // so line loads are realistic. A per-step control, no rebuild.
    void setFlightLoad(bool enabled) { controls_.flightLoad = enabled; }

    // Snapshot for the analysis dialog, which drives its own bodies with
    // the tunnel's exact settings.
    SimControls controls() const { return controls_; }

    // One line for the shape HUD: the live wing measured against its
    // design shape, in the units a designer reads. Empty until a body and
    // its baseline exist.
    QString shapeReadout() const
    {
        if (!sim_.body || baseline_.restPositions.empty()) {
            return {};
        }
        const lep::playground::ShapeReport report =
            lep::playground::measureShape(sim_, controls_, baseline_);
        QStringList parts;
        // The polar's numbers only exist when a polar pass runs; the raw
        // pinned pressure field carries no drag model worth quoting.
        if (controls_.flightLoad || controls_.freeFlight) {
            parts << QStringLiteral("L/D %1")
                         .arg(report.glideRatio, 0, 'f', 1)
                  << QStringLiteral("α %1°")
                         .arg(sim_.lastAlphaDegrees, 0, 'f', 1);
        }
        parts << QStringLiteral("span %1%")
                     .arg(report.spanRatio * 100.0, 0, 'f', 0);
        const double volume = (report.volumeRatio - 1.0) * 100.0;
        parts << QStringLiteral("vol %1%2%")
                     .arg(volume >= 0.0 ? QStringLiteral("+") : QString())
                     .arg(volume, 0, 'f', 0);
        parts << QStringLiteral("dev %1 mm @ rib %2")
                     .arg(report.worstDeviationMetres * 1000.0, 0, 'f', 0)
                     .arg(static_cast<qulonglong>(report.worstDeviationRib));
        parts << QStringLiteral("slack %1%")
                     .arg(report.slackFraction * 100.0, 0, 'f', 0);
        parts << QStringLiteral("LE %1 mm")
                     .arg(report.worstLeadingEdgeDentMetres * 1000.0,
                          0, 'f', 0);
        QStringList rows;
        for (const lep::playground::RowLoad &row : report.rows) {
            if (row.segments == 0) {
                continue;
            }
            rows << QStringLiteral("%1 %2")
                        .arg(row.row)
                        .arg(row.leftNewtons + row.rightNewtons, 0, 'f', 0);
        }
        if (!rows.isEmpty()) {
            parts << rows.join(QLatin1Char(' ')) + QStringLiteral(" N");
        }
        for (const lep::playground::ShapeFlagInfo &flag : report.flags) {
            parts << QStringLiteral("⚠ %1")
                         .arg(lep::playground::shapeFlagName(flag.flag));
        }
        if (lep::playground::grabActive(sim_)) {
            const auto &nodes = sim_.body->nodes();
            if (sim_.grabAnchorNode < nodes.size()
                && sim_.grabbedNode < nodes.size()) {
                const double pull =
                    length(nodes[sim_.grabAnchorNode].position
                           - nodes[sim_.grabbedNode].position);
                parts << QStringLiteral("pull %1 m %2 N")
                             .arg(pull, 0, 'f', 2)
                             .arg(lep::playground::grabForceNewtons(
                                      sim_, controls_),
                                  0, 'f', 0);
            }
        }
        return parts.join(QStringLiteral(" · "));
    }

    // Stretch at which the ramp saturates, as a fraction of rest length.
    void setStressFullScale(double strain)
    {
        stressFullScale_ =
            std::clamp(strain, 1.0e-4, maximumStressFullScaleStrain);
        update();
    }

    // Takes effect on the next build; the page rebuilds when it changes.
    void setDetailedRibs(bool enabled, int layers, int stationSplit)
    {
        buildOptions_.detailedRibs = enabled;
        buildOptions_.ribLayers = std::max(1, layers);
        buildOptions_.ribStationSplit = std::max(1, stationSplit);
    }

    bool detailedRibs() const { return buildOptions_.detailedRibs; }

    void setLineTensionColoring(bool enabled)
    {
        lineTensionColoring_ = enabled;
        update();
    }

    void setLineFullScale(double newtons)
    {
        lineFullScaleNewtons_ =
            std::clamp(newtons, 1.0, maximumLineFullScaleNewtons);
        update();
    }

    // Peak stretch across the wing right now, for the legend.
    // Highest cable load in the wing right now, for the legend.
    double peakLineTension() const
    {
        if (!sim_.body) {
            return 0.0;
        }
        const auto &constraints = sim_.body->constraints();
        double peak = 0.0;
        for (const LineSegment &segment : sim_.lineSegments) {
            peak = std::max(peak, lineTension(segment, constraints));
        }
        return peak;
    }

    // Legend peaks over the cached fields — fresh only while their mode is
    // active, which is the only time the legend quotes them.
    double peakStrain() const
    {
        float peak = 0.0F;
        for (const float strain : nodeTensile_) {
            peak = std::max(peak, strain);
        }
        return peak;
    }

    double peakDeviation() const
    {
        float peak = 0.0F;
        for (const float deviation : deviationField_) {
            peak = std::max(peak, deviation);
        }
        return peak;
    }

    // Positive compression fraction; the field stores strain (negative
    // when compressed, 0 where taut).
    double peakSlackCompression() const
    {
        float worst = 0.0F;
        for (const float strain : nodeSlack_) {
            worst = std::min(worst, strain);
        }
        return -worst;
    }

    void setRunning(bool running)
    {
        if (running && sim_.body) {
            timer_->start();
        } else {
            timer_->stop();
        }
    }

    bool isRunning() const { return timer_->isActive(); }
    bool hasBody() const { return sim_.body != nullptr; }
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
                      "in vec3 tint;\n"
                      "uniform mat4 mvp;\n"
                      "out vec3 vNormal;\n"
                      "out vec3 vTint;\n"
                      "void main() {\n"
                      "    vNormal = normal;\n"
                      "    vTint = tint;\n"
                      "    gl_Position = mvp * vec4(position, 1.0);\n"
                      "}\n")
                : QStringLiteral(
                      "attribute vec3 position;\n"
                      "attribute vec3 normal;\n"
                      "attribute vec3 tint;\n"
                      "uniform mat4 mvp;\n"
                      "varying vec3 vNormal;\n"
                      "varying vec3 vTint;\n"
                      "void main() {\n"
                      "    vNormal = normal;\n"
                      "    vTint = tint;\n"
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
                      "uniform bool useTint;\n"
                      "in vec3 vNormal;\n"
                      "in vec3 vTint;\n"
                      "out vec4 fragColor;\n"
                      "void main() {\n")
                      + shading
                      + QStringLiteral(
                          "    vec3 base = useTint ? vTint : color.rgb;\n"
                          "    fragColor = vec4(base * shade, color.a);\n"
                          "}\n")
                : QStringLiteral(
                      "uniform vec4 color;\n"
                      "uniform bool lit;\n"
                      "uniform bool useTint;\n"
                      "varying vec3 vNormal;\n"
                      "varying vec3 vTint;\n"
                      "void main() {\n")
                      + shading
                      + QStringLiteral(
                          "    vec3 base = useTint ? vTint : color.rgb;\n"
                          "    gl_FragColor = vec4(base * shade, color.a);\n"
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
        if (!sim_.body || !glError_.isEmpty()) {
            return;
        }

        const QMatrix4x4 mvp = viewProjection();

        const auto &nodes = sim_.body->nodes();

        vertexScratch_.clear();
        vertexScratch_.reserve(sim_.renderFaces.size() * 27);
        const auto &constraints = sim_.body->constraints();
        for (const RenderFace &face : sim_.renderFaces) {
            if (!surfaceVisible_[static_cast<std::size_t>(face.surface)]) {
                continue;
            }
            const softwing::Vec3 &a = nodes[face.nodes[0]].position;
            const softwing::Vec3 &b = nodes[face.nodes[1]].position;
            const softwing::Vec3 &c = nodes[face.nodes[2]].position;
            const softwing::Vec3 normal = normalized(cross(b - a, c - a));
            // useTint is a per-draw uniform while colourability is per
            // face, so an uncoloured face still needs a colour of its own
            // here — leaving it zeroed painted the simple ribs black.
            // Stress and Slack keep the colourable() gate (a simple rib's
            // colour would be spoke tension dressed up as rib stress);
            // Deviation is per node and honest everywhere the baseline
            // reaches, so it tints ungated. All three modes tint per
            // VERTEX from per-node fields: a face-flat colour renders the
            // skin as facets, the node-scattered same data shades
            // smoothly across them.
            const bool faceColoured =
                colorMode_ == ColorMode::Deviation
                || (colorMode_ != ColorMode::Plain && colourable(face));
            for (int corner = 0; corner < 3; ++corner) {
                const std::size_t node = face.nodes[
                    static_cast<std::size_t>(corner)];
                QVector3D tint = uncolouredTint;
                if (faceColoured) {
                    switch (colorMode_) {
                    case ColorMode::Stress:
                        tint = stressTint(node < nodeTensile_.size()
                                              ? nodeTensile_[node]
                                              : 0.0F);
                        break;
                    case ColorMode::Slack:
                        tint = rampTint(
                            (node < nodeSlack_.size() ? -nodeSlack_[node]
                                                      : 0.0F)
                            / std::max(stressFullScale_, 1.0e-6));
                        break;
                    case ColorMode::Deviation:
                        tint = rampTint((node < deviationField_.size()
                                             ? deviationField_[node]
                                             : 0.0F)
                                        / deviationFullScaleMetres());
                        break;
                    case ColorMode::Plain:
                        break;
                    }
                }
                const softwing::Vec3 &point = nodes[node].position;
                vertexScratch_.push_back(static_cast<float>(point.x));
                vertexScratch_.push_back(static_cast<float>(point.y));
                vertexScratch_.push_back(static_cast<float>(point.z));
                vertexScratch_.push_back(static_cast<float>(normal.x));
                vertexScratch_.push_back(static_cast<float>(normal.y));
                vertexScratch_.push_back(static_cast<float>(normal.z));
                vertexScratch_.push_back(tint.x());
                vertexScratch_.push_back(tint.y());
                vertexScratch_.push_back(tint.z());
            }
        }
        const int skinFloats = static_cast<int>(vertexScratch_.size());
        if (linesVisible_) {
            for (const LineSegment &segment : sim_.lineSegments) {
                QVector3D tint;
                if (lineTensionColoring_) {
                    tint = rampTint(
                        lineTension(segment, constraints)
                        / std::max(lineFullScaleNewtons_, 1.0e-6));
                }
                for (const std::size_t node : {segment.a, segment.b}) {
                    const softwing::Vec3 &point = nodes[node].position;
                    vertexScratch_.push_back(static_cast<float>(point.x));
                    vertexScratch_.push_back(static_cast<float>(point.y));
                    vertexScratch_.push_back(static_cast<float>(point.z));
                    vertexScratch_.push_back(segment.brake ? 1.0F : 0.0F);
                    vertexScratch_.push_back(0.0F);
                    vertexScratch_.push_back(0.0F);
                    vertexScratch_.push_back(tint.x());
                    vertexScratch_.push_back(tint.y());
                    vertexScratch_.push_back(tint.z());
                }
            }
        }

        program_->bind();
        QOpenGLVertexArrayObject::Binder vaoBinder(&vao_);
        program_->setUniformValue("mvp", mvp);
        buffer_.bind();
        buffer_.allocate(
            vertexScratch_.data(),
            static_cast<int>(vertexScratch_.size() * sizeof(float)));
        constexpr int stride = 9 * sizeof(float);
        program_->enableAttributeArray("position");
        program_->setAttributeBuffer("position", GL_FLOAT, 0, 3, stride);
        program_->enableAttributeArray("normal");
        program_->setAttributeBuffer(
            "normal", GL_FLOAT, 3 * sizeof(float), 3, stride);
        program_->enableAttributeArray("tint");
        program_->setAttributeBuffer(
            "tint", GL_FLOAT, 6 * sizeof(float), 3, stride);

        program_->setUniformValue("lit", true);
        program_->setUniformValue("useTint",
                                  colorMode_ != ColorMode::Plain);
        program_->setUniformValue(
            "color", QVector4D(0.72F, 0.78F, 0.88F, 1.0F));
        glDrawArrays(GL_TRIANGLES, 0, skinFloats / 9);

        program_->setUniformValue("lit", false);
        program_->setUniformValue("useTint", lineTensionColoring_);
        program_->setUniformValue(
            "color", QVector4D(0.55F, 0.62F, 0.55F, 1.0F));
        glDrawArrays(GL_LINES,
                     skinFloats / 9,
                     (static_cast<int>(vertexScratch_.size()) - skinFloats)
                         / 9);
        buffer_.release();
        program_->release();
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        lastMouse_ = event->position();
        // The grab tool is a tunnel instrument only: in fly mode the mouse
        // is the brake input, and in free flight a world-anchored pull on
        // a flying frame reads as nonsense.
        if (event->button() == Qt::LeftButton
            && event->modifiers().testFlag(Qt::ControlModifier)
            && !flyMode_ && !controls_.freeFlight) {
            tryBeginGrab(event->position());
        }
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (grabbing_ && event->button() == Qt::LeftButton) {
            lep::playground::endGrab(sim_);
            grabbing_ = false;
            setCursor(flyMode_ ? Qt::CrossCursor : Qt::ArrowCursor);
            update();
        }
        QOpenGLWidget::mouseReleaseEvent(event);
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        if (flyMode_ && event->key() == Qt::Key_Escape) {
            setFlyMode(false);
            if (flyModeExited_) {
                flyModeExited_();
            }
            return;
        }
        QOpenGLWidget::keyPressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (grabbing_) {
            // The cursor drags the grab anchor across the camera's screen
            // plane at the picked node's depth — the same right/up and
            // metres-per-pixel construction the pan branch uses, with the
            // node's depth in place of the orbit distance so the anchor
            // tracks the cursor exactly wherever the junction sits.
            const QPointF delta = event->position() - lastMouse_;
            lastMouse_ = event->position();
            QMatrix4x4 rotation;
            rotation.rotate(pitch_, 1, 0, 0);
            rotation.rotate(yaw_, 0, 0, 1);
            const QVector3D right = rotation.row(0).toVector3D();
            const QVector3D up = rotation.row(1).toVector3D();
            const float metresPerPixel =
                2.0F * grabDepth_
                * std::tan(cameraFieldOfViewDegrees * 0.5F
                           * degreesToRadians)
                / static_cast<float>(std::max(height(), 1));
            grabWorld_ +=
                softwing::Vec3{right.x(), right.y(), right.z()}
                    * (delta.x() * metresPerPixel)
                - softwing::Vec3{up.x(), up.y(), up.z()}
                      * (delta.y() * metresPerPixel);
            lep::playground::moveGrab(sim_, grabWorld_);
            update();
            return;
        }
        if (flyMode_) {
            // Horizontal position steers, vertical position is the pull:
            // -1 at the left edge, +1 at the right, 0 pull at the top,
            // full travel at the bottom.
            const double across = std::clamp(
                event->position().x() / std::max(1, width()) * 2.0 - 1.0,
                -1.0,
                1.0);
            const double pull =
                std::clamp(event->position().y() / std::max(1, height()),
                           0.0,
                           1.0)
                * maximumBrakeTravelMetres;
            const double screenLeft =
                pull * std::clamp(1.0 - across, 0.0, 1.0);
            const double screenRight =
                pull * std::clamp(1.0 + across, 0.0, 1.0);
            setBrakePull(screenLeft, screenRight);
            if (flyBrakesChanged_) {
                flyBrakesChanged_(screenLeft, screenRight);
            }
        }
        const QPointF delta = event->position() - lastMouse_;
        lastMouse_ = event->position();
        if (event->buttons() & Qt::LeftButton) {
            yaw_ += static_cast<float>(delta.x()) * 0.4F;
            pitch_ += static_cast<float>(delta.y()) * 0.4F;
            pitch_ = std::clamp(pitch_, -90.0F, 90.0F);
            update();
        } else if (event->buttons() & Qt::RightButton) {
            // Slide the orbit target across the camera's screen plane. The
            // view matrix is a pure rotation about the target followed by a
            // pull-back, so the camera's world-space right/up axes are the
            // first two rows of that rotation. Scaling by the view height at
            // the target's depth makes the wing track the cursor exactly.
            QMatrix4x4 rotation;
            rotation.rotate(pitch_, 1, 0, 0);
            rotation.rotate(yaw_, 0, 0, 1);
            const QVector3D right = rotation.row(0).toVector3D();
            const QVector3D up = rotation.row(1).toVector3D();
            const float metresPerPixel =
                2.0F * distance_
                * std::tan(cameraFieldOfViewDegrees * 0.5F * degreesToRadians)
                / static_cast<float>(std::max(height(), 1));
            target_ -= right * static_cast<float>(delta.x()) * metresPerPixel;
            target_ += up * static_cast<float>(delta.y()) * metresPerPixel;
            update();
        }
    }

    void wheelEvent(QWheelEvent *event) override
    {
        distance_ *=
            event->angleDelta().y() > 0 ? 1.0F / 1.15F : 1.15F;
        // Zooming changes the camera the grab depth was measured in;
        // re-read it from the anchor's current place or the next drag
        // step scales its cursor motion with a stale metres-per-pixel
        // and the pulled node runs away from the cursor.
        if (grabbing_ && sim_.body != nullptr
            && sim_.grabAnchorNode < sim_.body->nodes().size()) {
            const softwing::Vec3 &anchor =
                sim_.body->nodes()[sim_.grabAnchorNode].position;
            const QVector3D eye = viewMatrix().map(
                QVector3D(static_cast<float>(anchor.x),
                          static_cast<float>(anchor.y),
                          static_cast<float>(anchor.z)));
            grabDepth_ = std::max(-eye.z(), 0.05F);
        }
        update();
    }

private:
    // The skin is a mass-spring cloth, not a membrane element, so there is
    // no stress tensor to read: the honest per-face measure is how far its
    // three sides are stretched past their rest length. Because every skin
    // edge shares one compliance, tension is proportional to this, so the
    // picture reads the same as a tension plot.
    // Without the detailed model a rib is a hub and spokes, so its colour
    // would be spoke tension dressed up as rib stress. Leave those faces
    // plain rather than show a number that means nothing.
    bool colourable(const RenderFace &face) const
    {
        return face.surface != SimSurface::Rib || buildOptions_.detailedRibs;
    }

    // Cable tension in newtons. XPBD's multiplier is an impulse over the
    // substep, so dividing by the substep squared recovers the force; the
    // sign convention makes a tensile multiplier negative, hence the
    // magnitude. A slack cable holds zero, which is what we want to show.
    // The substep length is whatever the solver quality currently asks for,
    // so it is read from the live controls rather than from a constant --
    // otherwise switching quality would silently rescale every line load.
    double lineTension(
        const LineSegment &segment,
        const std::vector<softwing::DistanceConstraint> &constraints) const
    {
        if (segment.constraint == noConstraint
            || segment.constraint >= constraints.size()) {
            return 0.0;
        }
        const double substep =
            lep::playground::simulationTimeStep
            / std::max(1, controls_.substeps);
        return std::abs(constraints[segment.constraint].accumulatedLambda)
               / (substep * substep);
    }

    // Unloaded blue -> teal -> green -> amber -> red at full scale.
    static QVector3D rampTint(double loadFraction)
    {
        static const std::array<QVector3D, 5> ramp{
            QVector3D(0.16F, 0.29F, 0.62F),
            QVector3D(0.16F, 0.60F, 0.62F),
            QVector3D(0.30F, 0.68F, 0.33F),
            QVector3D(0.90F, 0.68F, 0.20F),
            QVector3D(0.83F, 0.24F, 0.20F)};
        const double position =
            std::clamp(loadFraction, 0.0, 1.0) * (ramp.size() - 1);
        const auto stop =
            std::min(static_cast<std::size_t>(position), ramp.size() - 2);
        const float blend =
            static_cast<float>(position - static_cast<double>(stop));
        return ramp[stop] * (1.0F - blend) + ramp[stop + 1] * blend;
    }

    QVector3D stressTint(double strain) const
    {
        return rampTint(strain / std::max(stressFullScale_, 1.0e-6));
    }

    // One scale slider serves three ramps. Its integer value is hundredths
    // of a percent for the strain modes (10..500 -> 0.1%..5% strain) and
    // read as millimetres for deviation (10..500 mm). stressFullScale_
    // stores value/10000, so the metre scale is that times ten.
    double deviationFullScaleMetres() const
    {
        return std::max(stressFullScale_ * 10.0, 1.0e-4);
    }

    QMatrix4x4 viewMatrix() const
    {
        QMatrix4x4 view;
        view.translate(0, 0, -distance_);
        view.rotate(pitch_, 1, 0, 0);
        view.rotate(yaw_, 0, 0, 1);
        view.translate(-target_);
        return view;
    }

    // Shared by the draw and the grab pick, so what is clicked is exactly
    // what is seen.
    QMatrix4x4 viewProjection() const
    {
        QMatrix4x4 projection;
        projection.perspective(
            cameraFieldOfViewDegrees,
            width() > 0 ? float(width()) / float(std::max(height(), 1))
                        : 1.0F,
            0.02F,
            500.0F);
        return projection * viewMatrix();
    }

    // Refills the cached colour field for the active mode, reusing the
    // vector. Called on the step cadence, on mode changes (so a paused
    // wing colours too) and after a rebuild.
    void refreshColourField()
    {
        if (!sim_.body) {
            return;
        }
        if (colorMode_ == ColorMode::Deviation
            && !baseline_.restPositions.empty()) {
            lep::playground::nodeDeviationField(sim_, baseline_,
                                                deviationField_);
        } else if (colorMode_ == ColorMode::Slack
                   || colorMode_ == ColorMode::Stress) {
            lep::playground::nodeStrainFields(sim_,
                                              buildOptions_.detailedRibs,
                                              nodeTensile_,
                                              nodeSlack_);
        }
        fieldSteps_ = 0;
    }

    // Ctrl-click picking: every unique line-junction endpoint projected to
    // widget pixels, nearest within grabPickRadiusPixels wins. Carabiners
    // are fixed nodes, so grabbing one merely parks the anchor on it — not
    // worth excluding.
    void tryBeginGrab(const QPointF &cursor)
    {
        if (!sim_.body) {
            return;
        }
        const QMatrix4x4 mvp = viewProjection();
        const auto &nodes = sim_.body->nodes();
        std::set<std::size_t> junctions;
        for (const LineSegment &segment : sim_.lineSegments) {
            junctions.insert(segment.a);
            junctions.insert(segment.b);
        }
        double bestDistance = grabPickRadiusPixels;
        std::size_t bestNode = noConstraint;
        for (const std::size_t node : junctions) {
            if (node >= nodes.size()) {
                continue;
            }
            const softwing::Vec3 &position = nodes[node].position;
            const QVector4D clip =
                mvp
                * QVector4D(static_cast<float>(position.x),
                            static_cast<float>(position.y),
                            static_cast<float>(position.z),
                            1.0F);
            if (clip.w() <= 0.0F) {
                continue;
            }
            const double px =
                (clip.x() / clip.w() * 0.5 + 0.5) * width();
            const double py =
                (1.0 - (clip.y() / clip.w() * 0.5 + 0.5)) * height();
            const double distance =
                std::hypot(px - cursor.x(), py - cursor.y());
            if (distance < bestDistance) {
                bestDistance = distance;
                bestNode = node;
            }
        }
        if (bestNode == noConstraint) {
            return;
        }
        // Read before beginGrab: creating the anchor node can grow the
        // node table and move it.
        const softwing::Vec3 picked = nodes[bestNode].position;
        if (!lep::playground::beginGrab(sim_, bestNode)) {
            return;
        }
        grabWorld_ = picked;
        const QVector3D eye = viewMatrix().map(
            QVector3D(static_cast<float>(picked.x),
                      static_cast<float>(picked.y),
                      static_cast<float>(picked.z)));
        // Camera looks down -z; the depth scales the cursor's
        // metres-per-pixel during the drag.
        grabDepth_ = std::max(-eye.z(), 0.05F);
        grabbing_ = true;
        setCursor(Qt::ClosedHandCursor);
    }

    void applyPressure()
    {
        lep::playground::applyPressure(sim_, controls_);
    }

    void stepSimulation()
    {
        try {
            lep::playground::stepSimulation(sim_, controls_);
            if (colorMode_ != ColorMode::Plain
                && ++fieldSteps_ >= fieldRefreshSteps) {
                refreshColourField();
            }
        } catch (const std::exception &exception) {
            simError_ = QString::fromUtf8(exception.what());
            setRunning(false);
        }
    }

    SimBody sim_;
    SimMesh mesh_;
    SimControls controls_;
    SimBuildOptions buildOptions_;
    std::array<bool, simSurfaceCount> surfaceVisible_{
        true, true, true, true, true};
    bool linesVisible_ = true;
    bool flyMode_ = false;
    std::function<void(double, double)> flyBrakesChanged_;
    std::function<void()> flyModeExited_;
    ColorMode colorMode_ = ColorMode::Plain;
    double stressFullScale_ = defaultStressFullScaleStrain;
    bool lineTensionColoring_ = false;
    double lineFullScaleNewtons_ = defaultLineFullScaleNewtons;
    // The design shape and the cached heatmap fields measured against it.
    // Fields are refreshed on the fieldRefreshSteps cadence, never
    // reallocated per frame.
    lep::playground::ShapeBaseline baseline_;
    std::vector<float> deviationField_;
    std::vector<float> nodeTensile_;
    std::vector<float> nodeSlack_;
    int fieldSteps_ = 0;
    // The interactive grab: anchor position accumulated in doubles so a
    // long drag does not drift, depth fixed at pick time.
    bool grabbing_ = false;
    float grabDepth_ = 1.0F;
    softwing::Vec3 grabWorld_;
    std::vector<float> vertexScratch_;
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

// Settles an identical twin of the live wing to convergence at the
// Accurate solver setting on a worker thread, so the user can start a
// long computation and review the settled result later. A twin rather
// than the live body: the live body belongs to the GUI thread and its
// 16 ms timer, and stealing it would either block the UI or race it.
// QThread's inherited finished() signal is the only signalling needed,
// so the class carries no Q_OBJECT.
class SettleWorker : public QThread
{
public:
    SettleWorker(lep::playground::SimMesh mesh,
                 const SimBuildOptions &options,
                 const SimControls &controls,
                 QObject *parent)
        : QThread(parent),
          mesh_(std::move(mesh)),
          options_(options),
          controls_(controls)
    {
    }

    // Polled every settle frame, so a cancel lands in milliseconds.
    void requestCancel() { cancelled_.store(true, std::memory_order_relaxed); }
    [[nodiscard]] bool wasCancelled() const
    {
        return cancelled_.load(std::memory_order_relaxed);
    }

    // Live progress for the status ticker, written from the worker at
    // every quiescence probe.
    [[nodiscard]] double progressSimSeconds() const
    {
        return progressSimSeconds_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] double progressAgitation() const
    {
        return progressAgitation_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] double quiescenceTarget() const
    {
        return lep::playground::settleQuiescenceTarget(
            controls_.pressurePascal);
    }

    // GUI thread, after finished() only.
    [[nodiscard]] const SimBody &settledBody() const { return sim_; }
    [[nodiscard]] const lep::playground::SettleResult &result() const
    {
        return result_;
    }
    [[nodiscard]] const QString &error() const { return error_; }

    void run() override
    {
        try {
            sim_ = buildSimBody(mesh_, options_, controls_);
            const lep::playground::ShapeBaseline baseline =
                lep::playground::captureShapeBaseline(sim_);
            const std::function<void(double, double)> progress =
                [this](double seconds, double agitation) {
                    progressSimSeconds_.store(seconds,
                                              std::memory_order_relaxed);
                    progressAgitation_.store(agitation,
                                             std::memory_order_relaxed);
                };
            result_ = lep::playground::settleAndMeasure(
                sim_, controls_, baseline, settleBudgetSeconds,
                &cancelled_, &progress);
        } catch (const std::exception &failure) {
            error_ = QString::fromUtf8(failure.what());
        }
    }

    // Generous: convergence stops the run long before this on a healthy
    // wing (gnuC2 settles in ~3 s simulated); the budget only bounds a
    // wing that never converges.
    static constexpr double settleBudgetSeconds = 30.0;

private:
    lep::playground::SimMesh mesh_;
    SimBuildOptions options_;
    SimControls controls_;
    std::atomic<bool> cancelled_{false};
    std::atomic<double> progressSimSeconds_{0.0};
    std::atomic<double> progressAgitation_{0.0};
    SimBody sim_;
    lep::playground::SettleResult result_;
    QString error_;
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
    // Capped at a third of what the slider used to reach: past roughly this
    // the skin inflates further than the fabric would really allow and the
    // wing stops looking like one, so the extra travel was only misleading.
    pressure_ = makeSlider(100, 80);
    lift_ = makeSlider(15, 6);
    leftBrake_ = makeSlider(100, 0);
    rightBrake_ = makeSlider(100, 0);
    runButton_ = new QPushButton(QStringLiteral("Pause"), this);
    runButton_->setCheckable(true);
    resetButton_ = new QPushButton(QStringLiteral("Reset"), this);
    resetButton_->setToolTip(QStringLiteral(
        "Rebuild the wing at its rest pose; in free flight it launches "
        "again on its glide."));
    flyButton_ = new QPushButton(QStringLiteral("Fly mode"), this);
    flyButton_->setCheckable(true);
    flyButton_->setToolTip(QStringLiteral(
        "Steer with the mouse over the wing: top centre is hands-up, "
        "straight down pulls both brakes, and moving toward a side "
        "releases the opposite brake. Esc leaves fly mode."));

    // Display filters: the solver always sees the whole wing, these only
    // decide what is drawn, so hiding the extrados looks inside a wing that
    // is still inflating normally.
    const auto makeCheck = [this](const QString &text, bool checked) {
        auto *check = new QCheckBox(text, this);
        check->setChecked(checked);
        return check;
    };
    showExtrados_ = makeCheck(QStringLiteral("Top"), true);
    showVent_ = makeCheck(QStringLiteral("Vent"), true);
    showIntrados_ = makeCheck(QStringLiteral("Bottom"), true);
    showRibs_ = makeCheck(QStringLiteral("Ribs"), true);
    showStraps_ = makeCheck(QStringLiteral("V/H ribs"), true);
    showLines_ = makeCheck(QStringLiteral("Lines"), true);

    // Skin heatmap source; item order matches PlaygroundView::ColorMode.
    colorBy_ = new QComboBox(this);
    colorBy_->addItem(QStringLiteral("Plain"));
    colorBy_->addItem(QStringLiteral("Stress"));
    colorBy_->addItem(QStringLiteral("Deviation"));
    colorBy_->addItem(QStringLiteral("Slack"));
    colorBy_->setToolTip(QStringLiteral(
        "Colour the skin by edge stretch (Stress), by distance from the "
        "designed shape (Deviation), or by compressed — wrinkled — fabric "
        "(Slack)."));

    // Full-scale for the ramp. Read as hundredths of a percent strain for
    // Stress and Slack (so the low end, where fabric actually works, still
    // has resolution) and as millimetres for Deviation.
    stressScale_ = new QSlider(Qt::Horizontal, this);
    stressScale_->setRange(
        10, static_cast<int>(maximumStressFullScaleStrain * 10000.0));
    stressScale_->setValue(
        static_cast<int>(defaultStressFullScaleStrain * 10000.0));
    stressScale_->setMaximumWidth(140);
    stressScale_->setVisible(false);
    showLineTension_ =
        makeCheck(QStringLiteral("Colour lines by tension"), false);
    lineScale_ = new QSlider(Qt::Horizontal, this);
    lineScale_->setRange(
        5, static_cast<int>(maximumLineFullScaleNewtons));
    lineScale_->setValue(static_cast<int>(defaultLineFullScaleNewtons));
    lineScale_->setMaximumWidth(140);
    lineScale_->setVisible(false);

    stressLegend_ = new QLabel(this);
    stressLegend_->setVisible(false);

    // How much solving each frame gets. This is a sandbox, so the default
    // is the setting that keeps a mid-sized wing interactive; the higher
    // ones are there for when the shape matters more than the frame rate.
    // Takes effect on the next frame — no rebuild.
    quality_ = new QComboBox(this);
    for (const lep::playground::SolverQuality &entry :
         lep::playground::solverQualities) {
        quality_->addItem(
            QStringLiteral("%1 (%2x%3)")
                .arg(QString::fromLatin1(entry.label))
                .arg(entry.substeps)
                .arg(entry.iterations));
    }
    quality_->setCurrentIndex(lep::playground::defaultSolverQuality);
    quality_->setToolTip(QStringLiteral(
        "Substeps x iterations per frame. More substeps hold the fabric "
        "closer to its designed length; fewer keep the wing interactive."));

    auto *view = new QHBoxLayout;
    view->addWidget(new QLabel(QStringLiteral("Show"), this));
    view->addWidget(showExtrados_);
    view->addWidget(showVent_);
    view->addWidget(showIntrados_);
    view->addWidget(showRibs_);
    view->addWidget(showStraps_);
    view->addWidget(showLines_);
    view->addSpacing(16);
    view->addWidget(showLineTension_);
    view->addWidget(lineScale_);
    view->addWidget(stressLegend_, 1);
    view->addStretch();

    // Its own row. The other two are already full edge to edge — the filter
    // row runs to the legend and the wing row's four stretching sliders will
    // grow until whatever follows them is pushed off the window — so a
    // widget added to either disappears rather than wraps.
    freeFlight_ = makeCheck(QStringLiteral("Free flight"), false);
    freeFlight_->setToolTip(QStringLiteral(
        "Unpin the wing: gravity on, a pilot slung under the risers, the "
        "whole system flying and re-centred each frame. Steer with the "
        "brakes; a little symmetric brake steadies it."));
    flightLabel_ = new QLabel(this);

    settleButton_ = new QPushButton(QStringLiteral("Settle"), this);
    settleButton_->setToolTip(QStringLiteral(
        "Run a twin of this wing at the Accurate solver setting (60×"
        "4) on a worker thread until it converges, then adopt the "
        "settled pose and pause for review. Start it, do something "
        "else, come back to the answer."));

    auto *solver = new QHBoxLayout;
    solver->addWidget(new QLabel(QStringLiteral("Solver"), this));
    solver->addWidget(quality_);
    solver->addWidget(settleButton_);
    solver->addSpacing(16);
    solver->addWidget(freeFlight_);
    solver->addWidget(flightLabel_, 1);
    solver->addStretch();

    // The shape instruments get a row of their own for the same
    // row-fullness reason free flight did (see above).
    shapeLabel_ = new QLabel(this);
    // The HUD line can outgrow the window (row loads, flags, a grab
    // readout); Ignored lets the layout clip it rather than push the
    // Flight load and Analyse controls off the right edge. The full
    // text rides in the tooltip.
    shapeLabel_->setSizePolicy(QSizePolicy::Ignored,
                               QSizePolicy::Preferred);
    flightLoad_ = makeCheck(QStringLiteral("Flight load"), true);
    flightLoad_->setToolTip(QStringLiteral(
        "Impose the wing-level polar load in the tunnel so line loads are "
        "realistic"));
    analyseButton_ = new QPushButton(QStringLiteral("Analyse…"), this);
    analyseButton_->setToolTip(QStringLiteral(
        "Sweep the tunnel across an angle-of-attack range on a worker "
        "thread and report shape integrity vs α."));

    auto *shape = new QHBoxLayout;
    shape->addWidget(new QLabel(QStringLiteral("Colour"), this));
    shape->addWidget(colorBy_);
    shape->addWidget(stressScale_);
    shape->addWidget(shapeLabel_, 1);
    shape->addWidget(flightLoad_);
    shape->addWidget(analyseButton_);

    auto *controls = new QHBoxLayout;
    controls->addWidget(new QLabel(QStringLiteral("Pressure"), this));
    controls->addWidget(pressure_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Angle"), this));
    controls->addWidget(lift_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Left brake"), this));
    controls->addWidget(leftBrake_, 1);
    controls->addWidget(new QLabel(QStringLiteral("Right brake"), this));
    controls->addWidget(rightBrake_, 1);
    controls->addWidget(runButton_);
    controls->addWidget(resetButton_);
    controls->addWidget(flyButton_);

    layout_ = new QVBoxLayout(this);
    layout_->addWidget(status_);
    layout_->addLayout(view);
    layout_->addLayout(solver);
    layout_->addLayout(shape);
    layout_->addLayout(controls);

    const auto bindSurface = [this](QCheckBox *check, SimSurface surface) {
        connect(check, &QCheckBox::toggled, this,
                [this, surface](bool visible) {
                    if (view_ != nullptr) {
                        view_->setSurfaceVisible(surface, visible);
                    }
                });
    };
    bindSurface(showExtrados_, SimSurface::Extrados);
    bindSurface(showVent_, SimSurface::Vent);
    bindSurface(showIntrados_, SimSurface::Intrados);
    bindSurface(showRibs_, SimSurface::Rib);
    bindSurface(showStraps_, SimSurface::Strap);
    connect(showLines_, &QCheckBox::toggled, this, [this](bool visible) {
        if (view_ != nullptr) {
            view_->setLinesVisible(visible);
        }
    });
    connect(quality_, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (view_ != nullptr) {
            view_->setSolverQuality(index);
        }
    });

    // The flight readout only means anything while the wing is flying.
    flightTimer_ = new QTimer(this);
    flightTimer_->setInterval(250);
    connect(flightTimer_, &QTimer::timeout, this, [this] {
        flightLabel_->setText(view_ != nullptr ? view_->flightReadout()
                                               : QString());
    });
    connect(freeFlight_, &QCheckBox::toggled, this, [this](bool enabled) {
        if (view_ != nullptr) {
            view_->setFreeFlight(enabled);
        }
        if (enabled) {
            flightTimer_->start();
        } else {
            flightTimer_->stop();
            flightLabel_->clear();
        }
        // Settling to convergence is a tunnel idea; a free-flying wing
        // glides, it does not converge.
        settleButton_->setEnabled(!enabled && !sweepActive_);
        // The toggle rebuilt the body, which leaves the solver running.
        updateShapeTimer();
    });
    connect(settleButton_, &QPushButton::clicked, this,
            [this] { toggleSettle(); });

    // The shape HUD runs whenever the solver does — tunnel or free flight
    // alike — so slider changes answer in real time. 500 ms: measureShape
    // is a full instrumentation pass, cheap at a few hertz, not per frame.
    shapeTimer_ = new QTimer(this);
    shapeTimer_->setInterval(500);
    connect(shapeTimer_, &QTimer::timeout, this, [this] {
        const QString readout =
            view_ != nullptr ? view_->shapeReadout() : QString();
        shapeLabel_->setText(readout);
        // The label clips (see its size policy); the tooltip carries
        // whatever fell off the edge.
        shapeLabel_->setToolTip(readout);
    });

    connect(flightLoad_, &QCheckBox::toggled, this, [this](bool enabled) {
        if (view_ != nullptr) {
            view_->setFlightLoad(enabled);
        }
    });
    connect(analyseButton_, &QPushButton::clicked, this,
            [this] { openAnalysis(); });

    // The legend needs the live peak stretch, which only exists while the
    // solver runs; polling a few times a second keeps it readable instead
    // of flickering with every frame.
    legendTimer_ = new QTimer(this);
    legendTimer_->setInterval(250);
    connect(legendTimer_, &QTimer::timeout, this, [this] {
        if (view_ == nullptr) {
            return;
        }
        QStringList parts;
        // Index order matches PlaygroundView::ColorMode; Plain contributes
        // nothing. The scale slider's integer value is hundredths of a
        // percent for the strain modes and millimetres for deviation.
        switch (colorBy_->currentIndex()) {
        case 1:
            parts << QStringLiteral("fabric slack → %1% stretch (peak %2%)")
                         .arg(stressScale_->value() / 100.0, 0, 'f', 2)
                         .arg(view_->peakStrain() * 100.0, 0, 'f', 2);
            break;
        case 2:
            parts << QStringLiteral("deviation 0 → %1 mm (peak %2 mm)")
                         .arg(stressScale_->value())
                         .arg(view_->peakDeviation() * 1000.0, 0, 'f', 0);
            break;
        case 3:
            parts << QStringLiteral("taut → %1% compressed (peak %2%)")
                         .arg(stressScale_->value() / 100.0, 0, 'f', 2)
                         .arg(view_->peakSlackCompression() * 100.0,
                              0, 'f', 2);
            break;
        default:
            break;
        }
        if (showLineTension_->isChecked()) {
            parts << QStringLiteral("lines 0 → %1 N (peak %2 N)")
                         .arg(lineScale_->value())
                         .arg(view_->peakLineTension(), 0, 'f', 1);
        }
        stressLegend_->setText(parts.join(QStringLiteral("  ·  ")));
    });
    connect(stressScale_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setStressFullScale(value / 10000.0);
        }
    });
    // One legend serves every colour mode plus line tension, so it lives
    // as long as any of them is on.
    const auto refreshLegendTimer = [this] {
        const bool wanted = colorBy_->currentIndex() != 0
                            || showLineTension_->isChecked();
        stressLegend_->setVisible(wanted);
        if (wanted) {
            legendTimer_->start();
        } else {
            legendTimer_->stop();
        }
    };
    connect(colorBy_, &QComboBox::currentIndexChanged, this,
            [this, refreshLegendTimer](int index) {
                if (view_ != nullptr) {
                    view_->setColorMode(
                        static_cast<PlaygroundView::ColorMode>(index));
                }
                stressScale_->setVisible(index != 0);
                refreshLegendTimer();
            });
    connect(lineScale_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setLineFullScale(static_cast<double>(value));
        }
    });
    connect(showLineTension_, &QCheckBox::toggled, this,
            [this, refreshLegendTimer](bool enabled) {
                if (view_ != nullptr) {
                    view_->setLineTensionColoring(enabled);
                }
                lineScale_->setVisible(enabled);
                refreshLegendTimer();
            });

    connect(pressure_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setPressurePascal(static_cast<double>(value));
        }
    });
    connect(lift_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setAngleOfAttack(static_cast<double>(value));
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
        updateShapeTimer();
    });
    connect(resetButton_, &QPushButton::clicked, this, [this] {
        if (view_ != nullptr) {
            view_->resetSimulation();
        }
        // The rebuilt body is running; the Pause button must say so.
        runButton_->setChecked(false);
        updateShapeTimer();
    });
    connect(flyButton_, &QPushButton::toggled, this, [this](bool enabled) {
        if (view_ != nullptr) {
            view_->setFlyMode(enabled);
        }
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
    view_->setAngleOfAttack(static_cast<double>(lift_->value()));
    view_->setBrakePull(
        leftBrake_->value() / 100.0 * maximumBrakeTravelMetres,
        rightBrake_->value() / 100.0 * maximumBrakeTravelMetres);
    view_->setSurfaceVisible(SimSurface::Extrados,
                             showExtrados_->isChecked());
    view_->setSurfaceVisible(SimSurface::Vent, showVent_->isChecked());
    view_->setSurfaceVisible(SimSurface::Intrados,
                             showIntrados_->isChecked());
    view_->setSurfaceVisible(SimSurface::Rib, showRibs_->isChecked());
    view_->setSurfaceVisible(SimSurface::Strap, showStraps_->isChecked());
    view_->setLinesVisible(showLines_->isChecked());
    view_->setStressFullScale(stressScale_->value() / 10000.0);
    view_->setColorMode(static_cast<PlaygroundView::ColorMode>(
        colorBy_->currentIndex()));
    view_->setLineFullScale(static_cast<double>(lineScale_->value()));
    view_->setLineTensionColoring(showLineTension_->isChecked());
    view_->setFlightLoad(flightLoad_->isChecked());
    view_->setFreeFlight(freeFlight_->isChecked());
    view_->setFlyModeCallbacks(
        // Mirror the live brake input onto the sliders. Signals stay
        // blocked: the view has already applied the pull, and letting the
        // sliders re-apply their integer-rounded copy would fight it.
        [this](double leftMetres, double rightMetres) {
            const auto mirror = [](QSlider *slider, double metres) {
                const QSignalBlocker blocker(slider);
                slider->setValue(static_cast<int>(
                    std::lround(metres / maximumBrakeTravelMetres
                                * 100.0)));
            };
            mirror(leftBrake_, leftMetres);
            mirror(rightBrake_, rightMetres);
        },
        [this] { flyButton_->setChecked(false); });
    view_->setFlyMode(flyButton_->isChecked());
    view_->show();
    creatingView_ = false;
}

void PlaygroundPage::updateShapeTimer()
{
    if (view_ != nullptr && view_->isRunning()) {
        shapeTimer_->start();
    } else {
        // The last readout stays up: a paused tunnel's numbers still
        // describe the frozen pose.
        shapeTimer_->stop();
    }
}

void PlaygroundPage::openAnalysis()
{
    if (meshData_.isEmpty() || view_ == nullptr) {
        status_->setText(QStringLiteral(
            "Run a preview or export first — the α sweep needs the "
            "calculated wing mesh."));
        return;
    }
    // The sweep builds its own bodies from the same mesh, refinement and
    // rib options rebuildSimulation would use, driven by the tunnel's
    // current controls, so its numbers match what the live view shows.
    SimBuildOptions options;
    options.detailedRibs = detailedRibs_;
    options.ribLayers = defaultRibLayers + 2 * (subdivision_ - 1);
    options.ribStationSplit = defaultRibStationSplit + subdivision_ - 1;
    // One dialog only: a second Analyse click raises the existing one.
    // Two dialogs meant two sweeps racing each other for the pause on
    // the live solver — whichever finished first resumed the tunnel
    // under the other's still-running sweep.
    if (analysisDialog_ != nullptr) {
        analysisDialog_->raise();
        analysisDialog_->activateWindow();
        return;
    }
    auto *dialog = new PlaygroundAnalysisDialog(
        meshData_, subdivision_, options, view_->controls(), this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    analysisDialog_ = dialog;
    connect(dialog, &QObject::destroyed, this, [this] {
        analysisDialog_ = nullptr;
        if (sweepActive_) {
            // A dialog killed mid-sweep still ends the sweep; its
            // sweepRunning(false) may arrive during teardown when this
            // connection is already severed, so the release lives here
            // too.
            setSweepActive(false);
        }
    });
    // The sweep worker and the live solver would fight over the same
    // cores; pause the tunnel for the sweep and restore whatever the run
    // button says afterwards (a sim the user paused stays paused).
    connect(dialog, &PlaygroundAnalysisDialog::sweepRunning, this,
            [this](bool running) { setSweepActive(running); });
    dialog->show();
}

// The single gate for "a sweep owns the machine". While active the Run
// and Reset buttons are disabled — not merely ignored, so the user can
// see why — and every rebuild path re-asserts the pause; on release the
// run button's own state decides, so a sim the user had paused stays
// paused.
void PlaygroundPage::setSweepActive(bool active)
{
    if (sweepActive_ == active) {
        return;
    }
    sweepActive_ = active;
    runButton_->setEnabled(!active);
    resetButton_->setEnabled(!active);
    // One background job at a time: the Analyse and Settle entries close
    // while either kind runs — except the Settle button itself when the
    // settle job is the owner, because that button is also its Cancel.
    analyseButton_->setEnabled(!active);
    settleButton_->setEnabled(
        (!active && !freeFlight_->isChecked()) || settleWorker_ != nullptr);
    // A free-flight toggle rebuilds the live body into a different
    // structure; adopting a tunnel twin's pose into it would be nonsense,
    // so the toggle waits the job out.
    freeFlight_->setEnabled(!active);
    if (view_ != nullptr) {
        view_->setRunning(!active && !runButton_->isChecked());
    }
    updateShapeTimer();
}

PlaygroundPage::~PlaygroundPage()
{
    if (settleWorker_ != nullptr) {
        settleWorker_->requestCancel();
        settleWorker_->wait();
    }
}

void PlaygroundPage::toggleSettle()
{
    if (settleWorker_ != nullptr) {
        settleWorker_->requestCancel();
        return;
    }
    if (view_ == nullptr || !view_->hasBody() || sweepActive_) {
        return;
    }
    // The twin gets the live controls with the solver budget forced to
    // the Accurate setting — the whole point of settling offline is to
    // afford the quality the interactive frame rate cannot.
    SimControls controls = view_->controls();
    controls.substeps = lep::playground::solverQualities[2].substeps;
    controls.constraintIterations =
        lep::playground::solverQualities[2].iterations;
    controls.workerThreads = lep::playground::playgroundWorkerThreads();
    controls.performanceProfile = nullptr;
    SimBuildOptions options;
    options.detailedRibs = detailedRibs_;
    options.ribLayers = defaultRibLayers + 2 * (subdivision_ - 1);
    options.ribStationSplit = defaultRibStationSplit + subdivision_ - 1;
    settleWorker_ =
        new SettleWorker(view_->simMesh(), options, controls, this);
    connect(settleWorker_, &QThread::finished, this,
            [this] { finishSettle(); });
    setSweepActive(true);
    settleButton_->setText(QStringLiteral("Cancel"));
    status_->setText(QStringLiteral(
        "Settling in the background at 60×4…"));
    // Live progress in the status line. The convergence-relevant number
    // is the agitation falling toward its target, so that is the
    // "progress bar"; the simulated clock against the budget bounds it.
    // The ticker is parented to the worker and dies with it.
    auto *ticker = new QTimer(settleWorker_);
    ticker->setInterval(500);
    connect(ticker, &QTimer::timeout, this, [this] {
        if (settleWorker_ == nullptr) {
            return;
        }
        const double seconds = settleWorker_->progressSimSeconds();
        if (seconds <= 0.0) {
            return;   // still building the twin
        }
        status_->setText(
            QStringLiteral("Settling in the background at 60×4… %1 of "
                           "max %2 s simulated · agitation %3 mm/s "
                           "(quiet below %4)")
                .arg(seconds, 0, 'f', 1)
                .arg(SettleWorker::settleBudgetSeconds, 0, 'f', 0)
                .arg(settleWorker_->progressAgitation() * 1000.0, 0, 'f',
                     0)
                .arg(settleWorker_->quiescenceTarget() * 1000.0, 0, 'f',
                     0));
    });
    ticker->start();
    settleWorker_->start();
}

void PlaygroundPage::finishSettle()
{
    SettleWorker *worker = settleWorker_;
    settleWorker_ = nullptr;
    settleButton_->setText(QStringLiteral("Settle"));
    if (worker == nullptr) {
        return;
    }
    QString outcome;
    if (worker->wasCancelled()) {
        outcome = QStringLiteral("Settle cancelled.");
    } else if (!worker->error().isEmpty()) {
        outcome = QStringLiteral("Settle failed: %1").arg(worker->error());
    } else if (view_ != nullptr) {
        const QString adoptError =
            view_->adoptSettledState(worker->settledBody());
        if (!adoptError.isEmpty()) {
            outcome = QStringLiteral("Settle: %1").arg(adoptError);
        } else {
            // The Done message earns its place with the numbers a
            // designer would ask for first; the HUD below carries the
            // full instrument line.
            const lep::playground::SettleResult &result = worker->result();
            const lep::playground::ShapeReport &report = result.report;
            QString flagText;
            if (report.flags.empty()) {
                flagText = QStringLiteral("no flags");
            } else {
                QStringList names;
                for (const auto &flag : report.flags) {
                    names << lep::playground::shapeFlagName(flag.flag);
                }
                flagText = QStringLiteral("⚠ ") + names.join(
                               QStringLiteral(", "));
            }
            const QString headline =
                result.settled
                    ? QStringLiteral("Done: settled at 60×4 in %1 s "
                                     "simulated")
                          .arg(result.simulatedSeconds, 0, 'f', 1)
                    : QStringLiteral("Done: did not converge within %1 s "
                                     "simulated (final pose adopted)")
                          .arg(SettleWorker::settleBudgetSeconds, 0, 'f',
                               0);
            outcome =
                QStringLiteral(
                    "%1 · L/D %2 · lift %3 N · worst deviation %4 mm @ "
                    "rib %5 · %6 — paused for review, Run resumes.")
                    .arg(headline)
                    .arg(report.glideRatio, 0, 'f', 2)
                    .arg(report.liftNewtons, 0, 'f', 0)
                    .arg(report.worstDeviationMetres * 1000.0, 0, 'f', 0)
                    .arg(static_cast<qulonglong>(
                        report.worstDeviationRib))
                    .arg(flagText);
            // Paused ON PURPOSE: the settled pose is the deliverable.
            // Checked == paused, so the run button reads "Run" and one
            // click resumes.
            runButton_->setChecked(true);
        }
    }
    worker->deleteLater();
    setSweepActive(false);
    status_->setText(outcome);
    // The HUD timer is stopped while paused; one explicit refresh shows
    // the settled numbers immediately.
    if (view_ != nullptr) {
        const QString readout = view_->shapeReadout();
        shapeLabel_->setText(readout);
        shapeLabel_->setToolTip(readout);
    }
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
    meshData_ = pendingData_;
    if (isVisible()) {
        loadIfPending();
    }
}

void PlaygroundPage::setMeshSubdivision(int factor)
{
    const int clamped = std::clamp(factor, 1, maximumMeshSubdivision);
    if (clamped == subdivision_) {
        return;
    }
    subdivision_ = clamped;
    rebuildSimulation();
}

void PlaygroundPage::setDetailedRibs(bool enabled)
{
    if (enabled == detailedRibs_) {
        return;
    }
    detailedRibs_ = enabled;
    rebuildSimulation();
}

void PlaygroundPage::rebuildSimulation()
{
    if (meshData_.isEmpty()) {
        return;
    }
    pendingData_ = meshData_;
    if (view_ != nullptr && isVisible()) {
        loadIfPending();
    }
}

void PlaygroundPage::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);
    ensureView();
    loadIfPending();
    updateShapeTimer();
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
    // Refining is quadratic in the factor and can take a moment at 4x.
    QGuiApplication::setOverrideCursor(Qt::WaitCursor);
    // Ribs gain a ring per resolution step so they densify with the skin.
    view_->setDetailedRibs(detailedRibs_,
                           defaultRibLayers + 2 * (subdivision_ - 1),
                           defaultRibStationSplit + subdivision_ - 1);
    const SimMesh simulated = refineSimMesh(*mesh, subdivision_);
    const QString buildError = view_->buildFromMesh(simulated);
    QGuiApplication::restoreOverrideCursor();
    if (!buildError.isEmpty()) {
        status_->setText(buildError);
        pendingData_.clear();
        return;
    }
    pendingData_.clear();
    QString resolution;
    if (subdivision_ > 1) {
        resolution = QStringLiteral(" · %1x resolution")
                         .arg(subdivision_ * subdivision_);
    }
    status_->setText(
        QStringLiteral("Wind tunnel · %1 nodes, %2 skin quads, %3 line "
                       "segments%4 · drag to orbit, right-drag to pan, "
                       "wheel to zoom, Ctrl-click a line junction to pull "
                       "it. Relative shape signal, not absolute "
                       "aerodynamics.")
            .arg(simulated.nodes.size())
            .arg(simulated.quads.size())
            .arg(simulated.lines.size())
            .arg(resolution));
    // The fresh body is running whatever the run button said before —
    // unless a sweep owns the machine, in which case the pause is
    // re-asserted over the rebuild's auto-start.
    if (sweepActive_) {
        view_->setRunning(false);
    }
    updateShapeTimer();
    // Shader problems only surface once the first frame renders; a silent
    // black view is undiagnosable, so report them here.
    QTimer::singleShot(500, this, [this] {
        if (!view_->lastGlError().isEmpty()) {
            status_->setText(view_->lastGlError());
        }
    });
}
