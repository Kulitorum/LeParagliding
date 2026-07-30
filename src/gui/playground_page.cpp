#include "playground_page.h"

#include "playground_analysis.h"
#include "playground_metrics.h"
#include "playground_sim.h"

#include "softwing/soft_body.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QFile>
#include <QGridLayout>
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
#include <QPainter>
#include <QPainterPath>
#include <QSurfaceFormat>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QToolButton>
#include <QElapsedTimer>
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
// Cadence of the heatmap field refresh, in wall-clock milliseconds. The
// fields are a measurement pass over the whole body — cheap enough for a
// few hertz, pointless every frame — and the cadence must be wall time:
// counted in steps, a heavily subdivided wing at hundreds of
// milliseconds per frame showed its rest pose for seconds.
constexpr int fieldRefreshMilliseconds = 400;
// How close a Ctrl-click must land to a projected line junction to grab it.
constexpr double grabPickRadiusPixels = 14.0;
// Simulated-time bound on the foreground settle; convergence normally
// stops it far earlier.
constexpr double kSettleBudgetSeconds = 30.0;
// Grey for faces that are drawn while stress colouring is on but carry no
// meaningful stress of their own — the simple rib web.
const QVector3D uncolouredTint(0.58F, 0.60F, 0.63F);
// The one heat ramp every coloured mode uses: unloaded blue -> teal ->
// green -> amber -> red at full scale. Shared by the per-vertex tints
// and the legend's colour bar, so the bar IS the calibration of the
// picture rather than an approximation of it.
const std::array<QVector3D, 5> kRampStops{
    QVector3D(0.16F, 0.29F, 0.62F), QVector3D(0.16F, 0.60F, 0.62F),
    QVector3D(0.30F, 0.68F, 0.33F), QVector3D(0.90F, 0.68F, 0.20F),
    QVector3D(0.83F, 0.24F, 0.20F)};

// One calibrated colour bar of the legend: what is plotted, the ramp's
// full-scale value, the live peak, and how to print a value in the
// bar's own unit.
struct LegendBar
{
    QString title;
    double fullScale = 1.0;
    double peak = 0.0;
    std::function<QString(double)> format;
};
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

        fitView();

        setRunning(true);
    }

    // Frame the wing (and, flying, the whole pendulum): target and
    // distance only, so the current view direction survives a re-fit.
    void fitView()
    {
        if (!sim_.body) {
            return;
        }
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
        update();
    }

    // The same named views the Design tab's viewport offers. Mesh
    // convention: span +x, chord +y (leading edge at low y), z up; the
    // camera at pitch 0 / yaw 0 looks straight down, so Top is the
    // identity and the others rotate off it.
    enum class ViewPreset
    {
        Iso,
        Front,
        Back,
        Left,
        Right,
        Top,
        Bottom,
    };

    void setViewPreset(ViewPreset preset)
    {
        switch (preset) {
        case ViewPreset::Iso:
            yaw_ = 30.0F;
            pitch_ = -60.0F;
            break;
        case ViewPreset::Front:
            yaw_ = 0.0F;
            pitch_ = -90.0F;
            break;
        case ViewPreset::Back:
            yaw_ = 180.0F;
            pitch_ = -90.0F;
            break;
        case ViewPreset::Left:
            yaw_ = 90.0F;
            pitch_ = -90.0F;
            break;
        case ViewPreset::Right:
            yaw_ = -90.0F;
            pitch_ = -90.0F;
            break;
        case ViewPreset::Top:
            yaw_ = 0.0F;
            pitch_ = 0.0F;
            break;
        case ViewPreset::Bottom:
            yaw_ = 0.0F;
            pitch_ = 180.0F;
            break;
        }
        update();
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

    // Debug only: what the display fields actually hold.
    QString debugFieldSummary() const
    {
        float devPeak = 0.0F;
        for (const float value : deviationField_) {
            devPeak = std::max(devPeak, value);
        }
        return QStringLiteral("mode %1 dev[%2] peak %3")
            .arg(static_cast<int>(colorMode_))
            .arg(deviationField_.size())
            .arg(devPeak);
    }

    // For the angle dial: the polar pass's live numbers. Lift is 0 and
    // the angle stale when no polar pass runs (pinned without flight
    // load), which the dial states rather than hides.
    bool polarActive() const
    {
        return controls_.flightLoad || controls_.freeFlight;
    }
    double liveLiftNewtons() const { return sim_.lastLift; }
    double liveAlphaDegrees() const { return sim_.lastAlphaDegrees; }

    // For the page's foreground settle: one simulation frame, outside
    // the 16 ms timer's pacing. False when the solver threw (the error
    // is in lastSimError and the run must stop).
    bool stepOnce()
    {
        stepSimulation();
        return simError_.isEmpty();
    }

    // The live body against sim state for the settle verdict: the
    // monitor reads agitation and the resultant off the body itself.
    const SimBody &simBody() const { return sim_; }

    // A full instrument pass on the current pose, for the settle's Done
    // line. Empty-report when no body or baseline exists.
    lep::playground::ShapeReport currentShapeReport() const
    {
        if (!sim_.body || baseline_.restPositions.empty()) {
            return {};
        }
        return lep::playground::measureShape(sim_, controls_, baseline_);
    }

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

    // What the legend must show right now: the active face mode's bar
    // and, when on, the line-tension bar. Empty in Plain mode with the
    // tension colouring off — no colours, no legend.
    std::vector<LegendBar> legendBars() const
    {
        const auto percent = [](double value) {
            return QStringLiteral("%1%").arg(value * 100.0, 0, 'f',
                                             value * 100.0 < 1.0 ? 2 : 1);
        };
        std::vector<LegendBar> bars;
        switch (colorMode_) {
        case ColorMode::Stress:
            bars.push_back({QStringLiteral("edge stretch"),
                            std::max(stressFullScale_, 1.0e-6),
                            peakStrain(), percent});
            break;
        case ColorMode::Deviation:
            bars.push_back(
                {QStringLiteral("deviation"), deviationFullScaleMetres(),
                 peakDeviation(), [](double value) {
                     return QStringLiteral("%1 mm").arg(value * 1000.0, 0,
                                                        'f', 0);
                 }});
            break;
        case ColorMode::Slack:
            bars.push_back({QStringLiteral("compression"),
                            std::max(stressFullScale_, 1.0e-6),
                            peakSlackCompression(), percent});
            break;
        case ColorMode::Plain:
            break;
        }
        if (lineTensionColoring_) {
            bars.push_back({QStringLiteral("line tension"),
                            std::max(lineFullScaleNewtons_, 1.0e-6),
                            peakLineTension(), [](double value) {
                                return QStringLiteral("%1 N").arg(
                                    value, 0, 'f', 0);
                            }});
        }
        return bars;
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
        if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_DEBUG")) {
            qWarning() << "GL context: stencil"
                       << context()->format().stencilBufferSize()
                       << "samples" << context()->format().samples()
                       << "profile"
                       << int(context()->format().profile());
        }

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

        // The calibrated legend, painted over the scene it calibrates.
        // This needs the stencil buffer requested in the constructor —
        // without one QPainter's GL engine silently drops filled paths.
        if (colorMode_ != ColorMode::Plain || lineTensionColoring_) {
            glDisable(GL_DEPTH_TEST);
            QPainter painter(this);
            painter.setRenderHint(QPainter::Antialiasing);
            painter.setRenderHint(QPainter::TextAntialiasing);
            drawLegendOverlay(painter);
            painter.end();
            glEnable(GL_DEPTH_TEST);
        }
    }

    // One calibrated colour bar per active colouring, top-right in the
    // viewport: quantity, unit, ticks and a live peak marker — a peak
    // past full scale parks at the top with its true value printed, so
    // saturation reads as saturation.
    void drawLegendOverlay(QPainter &painter)
    {
        const std::vector<LegendBar> bars = legendBars();
        if (bars.empty()) {
            return;
        }
        const QFont titleFont(painter.font().family(), 8,
                              QFont::DemiBold);
        const QFont tickFont(painter.font().family(), 8);
        constexpr int panelWidth = 146;
        constexpr int barWidth = 14;
        constexpr int margin = 12;
        constexpr int pad = 10;
        const int barHeight =
            std::clamp(static_cast<int>(height() * 0.30), 100, 240);
        int top = margin;
        for (const LegendBar &bar : bars) {
            const QRectF panel(width() - margin - panelWidth, top,
                               panelWidth, barHeight + 3 * pad + 16);
            painter.setPen(QColor(0x26, 0x35, 0x4a));
            painter.setBrush(QColor(0x0d, 0x14, 0x22, 222));
            painter.drawRoundedRect(panel, 7.0, 7.0);

            painter.setFont(titleFont);
            painter.setPen(QColor(0xb9, 0xc6, 0xd8));
            painter.drawText(
                QRectF(panel.left() + pad, panel.top() + pad,
                       panelWidth - 2 * pad, 14),
                Qt::AlignLeft | Qt::AlignVCenter, bar.title);

            const QRectF gradient(panel.left() + pad + 8,
                                  panel.top() + 2 * pad + 14, barWidth,
                                  barHeight);
            QLinearGradient ramp(gradient.bottomLeft(),
                                 gradient.topLeft());
            for (std::size_t stop = 0; stop < kRampStops.size();
                 ++stop) {
                const QVector3D &colour = kRampStops[stop];
                ramp.setColorAt(
                    static_cast<double>(stop) / (kRampStops.size() - 1),
                    QColor::fromRgbF(colour.x(), colour.y(),
                                     colour.z()));
            }
            painter.setPen(Qt::NoPen);
            painter.setBrush(ramp);
            painter.drawRect(gradient);

            painter.setFont(tickFont);
            for (int tick = 0; tick <= 4; ++tick) {
                const double fraction = tick / 4.0;
                const double tickY = gradient.bottom()
                                     - fraction * gradient.height();
                painter.setPen(QColor(0x93, 0xa4, 0xba));
                painter.drawLine(QPointF(gradient.right(), tickY),
                                 QPointF(gradient.right() + 4, tickY));
                if (tick % 2 == 0) {
                    painter.drawText(
                        QRectF(gradient.right() + 7, tickY - 8,
                               panel.right() - gradient.right() - 9,
                               16),
                        Qt::AlignLeft | Qt::AlignVCenter,
                        bar.format(fraction * bar.fullScale));
                }
            }

            const double peakFraction =
                std::clamp(bar.peak / bar.fullScale, 0.0, 1.0);
            const double peakY =
                gradient.bottom() - peakFraction * gradient.height();
            QPolygonF marker;
            marker << QPointF(gradient.left() - 2, peakY)
                   << QPointF(gradient.left() - 8, peakY - 4)
                   << QPointF(gradient.left() - 8, peakY + 4);
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(0xf7, 0xfb, 0xff));
            painter.drawPolygon(marker);
            painter.setPen(QColor(0xf7, 0xfb, 0xff));
            const double labelY =
                std::clamp(peakY + 10.0, gradient.top(),
                           gradient.bottom() - 16.0);
            painter.drawText(
                QRectF(gradient.right() + 7, labelY,
                       panel.right() - gradient.right() - 9, 16),
                Qt::AlignLeft | Qt::AlignVCenter,
                QStringLiteral("◂ %1").arg(bar.format(bar.peak)));

            top += static_cast<int>(panel.height()) + 8;
        }
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

    static QVector3D rampTint(double loadFraction)
    {
        const double position =
            std::clamp(loadFraction, 0.0, 1.0) * (kRampStops.size() - 1);
        const auto stop = std::min(static_cast<std::size_t>(position),
                                   kRampStops.size() - 2);
        const float blend =
            static_cast<float>(position - static_cast<double>(stop));
        return kRampStops[stop] * (1.0F - blend)
               + kRampStops[stop + 1] * blend;
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
        fieldClock_.start();
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
            // Wall-clock cadence, not steps: at high subdivision one
            // frame can cost hundreds of milliseconds, and a
            // count-based refresh left the heatmap showing the rest
            // pose for the first five seconds and crawling after.
            if (colorMode_ != ColorMode::Plain
                && (!fieldClock_.isValid()
                    || fieldClock_.elapsed() >= fieldRefreshMilliseconds)) {
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
    // Fields are refreshed on the wall-clock cadence, never reallocated
    // per frame.
    lep::playground::ShapeBaseline baseline_;
    std::vector<float> deviationField_;
    std::vector<float> nodeTensile_;
    std::vector<float> nodeSlack_;
    QElapsedTimer fieldClock_;
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

// The angle-of-attack dial: a section glyph with the wind arrow at the
// SET angle (accent), a second thinner arrow at the MEASURED live angle
// (ink), and the computed lift in newtons — the picture that makes the
// Angle slider mean something. A plain painted widget for the same
// stencil-buffer reason as the legend strip.
class AngleOfAttackDial : public QWidget
{
public:
    explicit AngleOfAttackDial(QWidget *parent) : QWidget(parent)
    {
        setFixedSize(236, 40);
        setToolTip(QStringLiteral(
            "The airflow against the wing's chord. Solid arrow: the "
            "angle the sliders set. Thin arrow: the angle the rigged "
            "wing actually holds under load. Lift is the imposed "
            "polar's, in newtons."));
        auto *refresh = new QTimer(this);
        refresh->setInterval(250);
        connect(refresh, &QTimer::timeout, this,
                QOverload<>::of(&QWidget::update));
        refresh->start();
    }

    void setView(const PlaygroundView *view) { view_ = view; }
    void setSetAngleProvider(std::function<double()> provider)
    {
        setAngle_ = std::move(provider);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setRenderHint(QPainter::TextAntialiasing);
        painter.setPen(QColor(0x26, 0x35, 0x4a));
        painter.setBrush(QColor(0x11, 0x1b, 0x2a));
        painter.drawRoundedRect(QRectF(0.5, 0.5, width() - 1.0,
                                       height() - 1.0),
                                6.0, 6.0);

        const double setDegrees = setAngle_ ? setAngle_() : 0.0;
        const bool live = view_ != nullptr && view_->hasBody()
                          && view_->polarActive();
        const double liveDegrees =
            live ? view_->liveAlphaDegrees() : setDegrees;

        // The section glyph: chord horizontal, nose left; wind arrows
        // point along the flow, tilted UP toward the tail by the angle
        // of attack — air from below, the physical convention.
        const QPointF nose(14.0, height() * 0.5);
        const QPointF tail(58.0, height() * 0.5);
        QPainterPath section;
        section.moveTo(nose);
        section.cubicTo(nose + QPointF(8.0, -7.0),
                        tail + QPointF(-18.0, -6.0), tail);
        section.cubicTo(tail + QPointF(-18.0, 1.5),
                        nose + QPointF(10.0, 3.5), nose);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0x93, 0xa4, 0xba));
        painter.drawPath(section);

        const auto windArrow = [&](double degrees, const QColor &colour,
                                   double strokeWidth) {
            const double radians = degrees * 3.14159265358979 / 180.0;
            // Points downstream; positive alpha lifts the tail end.
            const QPointF direction(std::cos(radians),
                                    -std::sin(radians));
            const QPointF centre((nose.x() + tail.x()) * 0.5,
                                 height() * 0.5);
            const QPointF from = centre - direction * 26.0;
            const QPointF to = centre + direction * 26.0;
            painter.setPen(QPen(colour, strokeWidth, Qt::SolidLine,
                                Qt::RoundCap));
            painter.drawLine(from, to);
            const QPointF back = to - direction * 6.0;
            const QPointF side(direction.y() * 3.5,
                               -direction.x() * 3.5);
            painter.drawLine(to, back + side);
            painter.drawLine(to, back - side);
        };
        // Measured first, so the set arrow stays on top where they
        // nearly coincide.
        if (live) {
            windArrow(liveDegrees, QColor(0xf7, 0xfb, 0xff), 1.0);
        }
        windArrow(setDegrees, QColor(0x38, 0xbd, 0xf8), 2.0);

        const QFont textFont(painter.font().family(), 8);
        painter.setFont(textFont);
        painter.setPen(QColor(0xb9, 0xc6, 0xd8));
        const QString angleLine =
            live ? QStringLiteral("α %1° · holds %2°")
                       .arg(setDegrees, 0, 'f', 0)
                       .arg(liveDegrees, 0, 'f', 1)
                 : QStringLiteral("α %1°").arg(setDegrees, 0, 'f', 0);
        painter.drawText(QRectF(70.0, 3.0, width() - 76.0, 16.0),
                         Qt::AlignLeft | Qt::AlignVCenter, angleLine);
        painter.setPen(QColor(0xf7, 0xfb, 0xff));
        const QFont liftFont(painter.font().family(), 8,
                             QFont::DemiBold);
        painter.setFont(liftFont);
        painter.drawText(
            QRectF(70.0, 20.0, width() - 76.0, 16.0),
            Qt::AlignLeft | Qt::AlignVCenter,
            live ? QStringLiteral("lift %1 N")
                       .arg(view_->liveLiftNewtons(), 0, 'f', 0)
                 : QStringLiteral("lift — (flight load off)"));
    }

private:
    const PlaygroundView *view_ = nullptr;
    std::function<double()> setAngle_;
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

    // ---- The left panel. The wing on screen is roughly 1:1 while the
    // window is closer to 2:1, so chrome above the viewport is exactly
    // the wrong place for it: everything lives in a column on the left
    // and the viewport gets the full height. ----

    freeFlight_ = makeCheck(QStringLiteral("Free flight"), false);
    freeFlight_->setToolTip(QStringLiteral(
        "Unpin the wing: gravity on, a pilot slung under the risers, the "
        "whole system flying and re-centred each frame. Steer with the "
        "brakes; a little symmetric brake steadies it."));
    flightLabel_ = new QLabel(this);
    flightLabel_->setWordWrap(true);

    settleButton_ = new QPushButton(QStringLiteral("Settle"), this);
    settleButton_->setToolTip(QStringLiteral(
        "Step the live wing at the Accurate setting (60×4), as fast as "
        "the machine allows, until it converges — watch it happen — "
        "then pause for review."));

    shapeLabel_ = new QLabel(this);
    // In a side panel the HUD may wrap: the full instrument line beats
    // a clipped one.
    shapeLabel_->setWordWrap(true);
    flightLoad_ = makeCheck(QStringLiteral("Flight load"), true);
    flightLoad_->setToolTip(QStringLiteral(
        "Impose the wing-level polar load in the tunnel so line loads are "
        "realistic"));
    analyseButton_ = new QPushButton(QStringLiteral("Analyse…"), this);
    analyseButton_->setToolTip(QStringLiteral(
        "Sweep the tunnel across an angle-of-attack range on a worker "
        "thread and report shape integrity vs α."));

    pressureLabel_ = new QLabel(this);
    angleLabel_ = new QLabel(this);
    leftBrakeLabel_ = new QLabel(this);
    rightBrakeLabel_ = new QLabel(this);
    alphaDial_ = new AngleOfAttackDial(this);
    alphaDial_->setSetAngleProvider(
        [this] { return static_cast<double>(lift_->value()); });

    // Initial readouts before any slider moves.
    const auto refreshControlReadouts = [this] {
        const double pascal = static_cast<double>(pressure_->value());
        pressureLabel_->setText(
            QStringLiteral("Pressure %1 Pa · %2 km/h")
                .arg(pressure_->value())
                .arg(std::sqrt(2.0 * pascal / 1.225) * 3.6, 0, 'f', 0));
        angleLabel_->setText(
            QStringLiteral("Angle %1°").arg(lift_->value()));
        leftBrakeLabel_->setText(
            QStringLiteral("Left brake %1 cm")
                .arg(std::lround(leftBrake_->value() / 100.0
                                 * maximumBrakeTravelMetres * 100.0)));
        rightBrakeLabel_->setText(
            QStringLiteral("Right brake %1 cm")
                .arg(std::lround(rightBrake_->value() / 100.0
                                 * maximumBrakeTravelMetres * 100.0)));
    };
    refreshControlReadouts();
    for (QSlider *slider :
         {pressure_, lift_, leftBrake_, rightBrake_}) {
        connect(slider, &QSlider::valueChanged, this,
                [refreshControlReadouts] { refreshControlReadouts(); });
    }

    const auto sectionLabel = [this](const QString &text) {
        auto *label = new QLabel(text, this);
        label->setObjectName(QStringLiteral("fieldLabel"));
        return label;
    };

    auto *panelLayout = new QVBoxLayout;
    panelLayout->setContentsMargins(0, 0, 6, 0);
    panelLayout->setSpacing(6);

    panelLayout->addWidget(sectionLabel(QStringLiteral("Show")));
    auto *showGrid = new QGridLayout;
    showGrid->setHorizontalSpacing(10);
    showGrid->setVerticalSpacing(4);
    showGrid->addWidget(showExtrados_, 0, 0);
    showGrid->addWidget(showVent_, 0, 1);
    showGrid->addWidget(showIntrados_, 1, 0);
    showGrid->addWidget(showRibs_, 1, 1);
    showGrid->addWidget(showStraps_, 2, 0);
    showGrid->addWidget(showLines_, 2, 1);
    panelLayout->addLayout(showGrid);
    panelLayout->addWidget(showLineTension_);
    panelLayout->addWidget(lineScale_);

    panelLayout->addSpacing(8);
    panelLayout->addWidget(sectionLabel(QStringLiteral("Solver")));
    auto *solverRow = new QHBoxLayout;
    solverRow->addWidget(quality_, 1);
    solverRow->addWidget(settleButton_);
    panelLayout->addLayout(solverRow);
    panelLayout->addWidget(freeFlight_);
    panelLayout->addWidget(flightLabel_);

    panelLayout->addSpacing(8);
    panelLayout->addWidget(sectionLabel(QStringLiteral("Colour")));
    auto *colourRow = new QHBoxLayout;
    colourRow->addWidget(colorBy_, 1);
    colourRow->addWidget(stressScale_, 1);
    panelLayout->addLayout(colourRow);
    auto *analysisRow = new QHBoxLayout;
    analysisRow->addWidget(flightLoad_);
    analysisRow->addWidget(analyseButton_, 1);
    panelLayout->addLayout(analysisRow);

    panelLayout->addSpacing(8);
    panelLayout->addWidget(sectionLabel(QStringLiteral("Tunnel")));
    panelLayout->addWidget(pressureLabel_);
    panelLayout->addWidget(pressure_);
    panelLayout->addWidget(angleLabel_);
    panelLayout->addWidget(lift_);
    panelLayout->addWidget(alphaDial_);
    panelLayout->addWidget(leftBrakeLabel_);
    panelLayout->addWidget(leftBrake_);
    panelLayout->addWidget(rightBrakeLabel_);
    panelLayout->addWidget(rightBrake_);
    auto *runRow = new QHBoxLayout;
    runRow->addWidget(runButton_, 1);
    runRow->addWidget(resetButton_, 1);
    runRow->addWidget(flyButton_, 1);
    panelLayout->addLayout(runRow);

    panelLayout->addSpacing(8);
    panelLayout->addWidget(shapeLabel_);
    panelLayout->addStretch();

    auto *panel = new QWidget(this);
    panel->setLayout(panelLayout);
    // Wide enough for the dial and the slider readouts; the scroll area
    // keeps a short window usable instead of crushing the sections.
    panel->setFixedWidth(272);
    auto *panelScroll = new QScrollArea(this);
    panelScroll->setWidget(panel);
    panelScroll->setWidgetResizable(true);
    panelScroll->setFrameShape(QFrame::NoFrame);
    panelScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    panelScroll->setFixedWidth(292);

    // ---- The right column: the viewport (inserted by ensureView) over
    // the Design-tab-style navigation buttons. ----
    const auto makeNav = [this](const QString &text, const QString &tip) {
        auto *button = new QToolButton(this);
        button->setObjectName(QStringLiteral("viewButton"));
        button->setText(text);
        button->setToolTip(tip);
        return button;
    };
    auto *navRow = new QHBoxLayout;
    navRow->setSpacing(5);
    navRow->addStretch();
    struct NavPreset
    {
        const char *label;
        const char *tip;
        int preset;   // -1 = fit
    };
    static constexpr NavPreset navPresets[] = {
        {"Fit", "Frame the wing", -1},
        {"Iso", "Isometric", 0},
        {"Front", "Front", 1},
        {"Back", "Back", 2},
        {"Left", "Left", 3},
        {"Right", "Right", 4},
        {"Top", "Top", 5},
        {"Bottom", "Bottom", 6},
    };
    for (const NavPreset &entry : navPresets) {
        QToolButton *button = makeNav(QLatin1String(entry.label),
                                      QLatin1String(entry.tip));
        const int preset = entry.preset;
        connect(button, &QToolButton::clicked, this, [this, preset] {
            if (view_ == nullptr) {
                return;
            }
            if (preset < 0) {
                view_->fitView();
            } else {
                view_->setViewPreset(
                    static_cast<PlaygroundView::ViewPreset>(preset));
            }
        });
        navRow->addWidget(button);
    }
    navRow->addStretch();

    layout_ = new QVBoxLayout;
    layout_->setSpacing(4);
    // ensureView() inserts the GL view at index 0 with stretch 1.
    layout_->addLayout(navRow);

    auto *contentRow = new QHBoxLayout;
    contentRow->addWidget(panelScroll);
    contentRow->addLayout(layout_, 1);

    // ---- One status line across the bottom. ----
    status_->setWordWrap(false);
    status_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    auto *pageLayout = new QVBoxLayout(this);
    pageLayout->addLayout(contentRow, 1);
    pageLayout->addWidget(status_);

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

    // The foreground settle's driver. Interval 0 = run whenever the
    // event loop is idle; each tick steps the live sim for at most ~40
    // ms then repaints, so the wing visibly converges while Cancel, the
    // camera and the rest of the UI stay responsive.
    settleTimer_ = new QTimer(this);
    settleTimer_->setInterval(0);
    connect(settleTimer_, &QTimer::timeout, this, [this] {
        if (!settleRunning_ || view_ == nullptr
            || settleMonitor_ == nullptr) {
            settleTimer_->stop();
            return;
        }
        QElapsedTimer chunk;
        chunk.start();
        bool done = false;
        bool failed = false;
        while (chunk.elapsed() < 40 && !done && !failed) {
            if (!view_->stepOnce()) {
                failed = true;
                break;
            }
            done = settleMonitor_->frameStepped(
                view_->simBody(), view_->controls().pressurePascal);
        }
        view_->update();
        ++settleStatusTick_;
        if (settleStatusTick_ % 6 == 0) {
            status_->setText(
                QStringLiteral("Settling at 60×4… %1 of max %2 s "
                               "simulated · agitation %3 mm/s (quiet "
                               "below %4)")
                    .arg(settleMonitor_->simulatedSeconds(), 0, 'f', 1)
                    .arg(kSettleBudgetSeconds, 0, 'f', 0)
                    .arg(settleMonitor_->lastAgitation() * 1000.0, 0,
                         'f', 0)
                    .arg(lep::playground::settleQuiescenceTarget(
                             view_->controls().pressurePascal)
                             * 1000.0,
                         0, 'f', 0));
        }
        // The shape HUD keeps measuring during the show — the data is
        // the point of watching.
        if (settleStatusTick_ % 12 == 0) {
            const QString readout = view_->shapeReadout();
            shapeLabel_->setText(readout);
            shapeLabel_->setToolTip(readout);
        }
        if (failed || done) {
            finishSettle(false);
        }
    });

    connect(flightLoad_, &QCheckBox::toggled, this, [this](bool enabled) {
        if (view_ != nullptr) {
            view_->setFlightLoad(enabled);
        }
    });
    connect(analyseButton_, &QPushButton::clicked, this,
            [this] { openAnalysis(); });

    // The legend is a calibrated colour bar drawn inside the view itself
    // (see PlaygroundView::drawLegendOverlay) — units, ticks and a live
    // peak marker next to the picture they explain, not a prose line in
    // the toolbar.
    connect(stressScale_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setStressFullScale(value / 10000.0);
        }
    });
    connect(colorBy_, &QComboBox::currentIndexChanged, this,
            [this](int index) {
                if (view_ != nullptr) {
                    view_->setColorMode(
                        static_cast<PlaygroundView::ColorMode>(index));
                }
                stressScale_->setVisible(index != 0);
            });
    connect(lineScale_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setLineFullScale(static_cast<double>(value));
        }
    });
    connect(showLineTension_, &QCheckBox::toggled, this,
            [this](bool enabled) {
                if (view_ != nullptr) {
                    view_->setLineTensionColoring(enabled);
                }
                lineScale_->setVisible(enabled);
            });

    // The sliders write live controls that only a STEPPING solver reads.
    // After Settle (or Pause) the sim is deliberately frozen, and a
    // slider that visibly does nothing reads as broken — say why.
    const auto notePausedControls = [this] {
        if (view_ != nullptr && view_->hasBody() && !view_->isRunning()
            && !settleRunning_ && !sweepActive_) {
            status_->setText(QStringLiteral(
                "Paused — press Run to see the new settings act."));
        }
    };
    connect(pressure_, &QSlider::valueChanged, this,
            [this, notePausedControls](int value) {
                if (view_ != nullptr) {
                    view_->setPressurePascal(static_cast<double>(value));
                }
                notePausedControls();
            });
    connect(lift_, &QSlider::valueChanged, this,
            [this, notePausedControls](int value) {
                if (view_ != nullptr) {
                    view_->setAngleOfAttack(static_cast<double>(value));
                }
                notePausedControls();
            });
    const auto pushBrakes = [this, notePausedControls] {
        if (view_ != nullptr) {
            view_->setBrakePull(
                leftBrake_->value() / 100.0 * maximumBrakeTravelMetres,
                rightBrake_->value() / 100.0 * maximumBrakeTravelMetres);
        }
        notePausedControls();
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

    // Start-up preset for scripted runs and screenshots, the same idea
    // as LEP_PLAYGROUND_DEBUG. After the connections, so the dependent
    // widgets (the scale slider's visibility) follow the preset.
    if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_COLOR")) {
        colorBy_->setCurrentIndex(
            std::clamp(qEnvironmentVariableIntValue("LEP_PLAYGROUND_COLOR"),
                       0, colorBy_->count() - 1));
    }
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
    alphaDial_->setView(view_);
    // Above the navigation buttons, taking every spare pixel: the wing
    // is the page.
    layout_->insertWidget(0, view_, 1);
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
    // One job at a time: the Analyse and Settle entries close while
    // either kind runs — except the Settle button itself when the
    // settle is the owner, because that button is also its Cancel.
    analyseButton_->setEnabled(!active);
    settleButton_->setEnabled(
        (!active && !freeFlight_->isChecked()) || settleRunning_);
    // A free-flight toggle rebuilds the live body into a different
    // structure mid-run; the toggle waits the job out.
    freeFlight_->setEnabled(!active);
    if (view_ != nullptr) {
        view_->setRunning(!active && !runButton_->isChecked());
    }
    updateShapeTimer();
}

PlaygroundPage::~PlaygroundPage() = default;

void PlaygroundPage::toggleSettle()
{
    if (settleRunning_) {
        finishSettle(true);
        return;
    }
    if (view_ == nullptr || !view_->hasBody() || sweepActive_) {
        return;
    }
    settleRunning_ = true;
    settleStatusTick_ = 0;
    settleMonitor_ = std::make_unique<lep::playground::SettleMonitor>(
        kSettleBudgetSeconds);
    // The Accurate solver budget, visibly: the combo itself moves (and
    // is restored on finish) — settling exists to afford the quality
    // the interactive frame rate cannot.
    settleRestoreQuality_ = quality_->currentIndex();
    quality_->setCurrentIndex(2);
    // setSweepActive stops the 16 ms pacing timer; the settle timer
    // then steps flat out on this thread in bounded chunks, so the user
    // WATCHES the wing converge under whatever heatmap is active
    // instead of staring at a progress line.
    setSweepActive(true);
    settleButton_->setText(QStringLiteral("Cancel"));
    status_->setText(QStringLiteral("Settling at 60×4…"));
    settleTimer_->start();
}

void PlaygroundPage::finishSettle(bool cancelled)
{
    settleTimer_->stop();
    if (!settleRunning_) {
        return;
    }
    settleRunning_ = false;
    settleButton_->setText(QStringLiteral("Settle"));
    // The solver budget returns to the user's choice; the converged
    // pose keeps its quality — that is state, not a setting.
    if (settleRestoreQuality_ >= 0) {
        quality_->setCurrentIndex(settleRestoreQuality_);
        settleRestoreQuality_ = -1;
    }
    QString outcome;
    const QString solverError =
        view_ != nullptr ? view_->lastSimError() : QString();
    if (cancelled) {
        outcome = QStringLiteral("Settle cancelled.");
    } else if (!solverError.isEmpty()) {
        outcome = QStringLiteral("Settle failed: %1").arg(solverError);
    } else if (view_ != nullptr && settleMonitor_ != nullptr) {
        // The Done message earns its place with the numbers a designer
        // would ask for first; the HUD below carries the full
        // instrument line.
        const lep::playground::ShapeReport report =
            view_->currentShapeReport();
        QString flagText;
        if (report.flags.empty()) {
            flagText = QStringLiteral("no flags");
        } else {
            QStringList names;
            for (const auto &flag : report.flags) {
                names << lep::playground::shapeFlagName(flag.flag);
            }
            flagText =
                QStringLiteral("⚠ ") + names.join(QStringLiteral(", "));
        }
        const QString headline =
            settleMonitor_->settled()
                ? QStringLiteral("Done: settled at 60×4 in %1 s "
                                 "simulated")
                      .arg(settleMonitor_->simulatedSeconds(), 0, 'f', 1)
                : QStringLiteral("Done: did not converge within %1 s "
                                 "simulated (final pose kept)")
                      .arg(kSettleBudgetSeconds, 0, 'f', 0);
        outcome =
            QStringLiteral(
                "%1 · L/D %2 · lift %3 N · worst deviation %4 mm @ "
                "rib %5 · %6 — paused for review, Run resumes.")
                .arg(headline)
                .arg(report.glideRatio, 0, 'f', 2)
                .arg(report.liftNewtons, 0, 'f', 0)
                .arg(report.worstDeviationMetres * 1000.0, 0, 'f', 0)
                .arg(static_cast<qulonglong>(report.worstDeviationRib))
                .arg(flagText);
        // Paused ON PURPOSE: the settled pose is the deliverable.
        // Checked == paused, so the run button reads "Run" and one
        // click resumes.
        runButton_->setChecked(true);
    }
    settleMonitor_.reset();
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
        // A Qt-side screenshot: QWidget::grab renders the widget tree
        // including the QOpenGLWidget's framebuffer, which PrintWindow
        // captures miss whenever native child windows are involved.
        if (qEnvironmentVariableIsSet("LEP_PLAYGROUND_SHOT")) {
            QTimer::singleShot(4500, this, [this] {
                const QString path = QString::fromLocal8Bit(
                    qgetenv("LEP_PLAYGROUND_SHOT"));
                window()->grab().save(path);
                // grab() cannot render the paintGL QPainter overlay (a
                // nested painter cannot begin); grabFramebuffer runs a
                // clean paintGL and shows the view as the screen does.
                if (view_ != nullptr) {
                    view_->grabFramebuffer().save(
                        path + QStringLiteral(".view.png"));
                }
                qWarning() << "grab saved";
            });
        }
        QTimer::singleShot(4000, this, [this] {
            qWarning() << "PlaygroundPage geometry" << geometry();
            if (view_ != nullptr) {
                qWarning() << "colour mode"
                           << colorBy_->currentIndex() << "field sizes"
                           << view_->debugFieldSummary();
            }
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
