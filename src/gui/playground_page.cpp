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
#include <QMutex>
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
#include <QThread>
#include <QToolButton>
#include <QWaitCondition>
#include <QElapsedTimer>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector3D>

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include <QTextStream>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
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
using lep::playground::LaunchMode;
using lep::playground::PressureSolveMode;
using lep::playground::RenderFace;
using lep::playground::SimBody;
using lep::playground::SimBuildOptions;
using lep::playground::SimControls;
using lep::playground::SimMesh;
using lep::playground::SimSurface;
using lep::playground::SkinModel;
using lep::playground::buildSimBody;
using lep::playground::defaultRibLayers;
using lep::playground::defaultRibStationSplit;
using lep::playground::maximumPilotMassKg;
using lep::playground::minimumPilotMassKg;
using lep::playground::noConstraint;
using lep::playground::parseSimMesh;
using lep::playground::refineSimMesh;
using lep::playground::simSurfaceCount;

namespace {

constexpr float cameraFieldOfViewDegrees = 40.0F;
constexpr float degreesToRadians = 3.14159265358979323846F / 180.0F;
constexpr double maximumBrakeTravelMetres = 0.6;
// How far out the air motes are drawn. Big enough that the field reads as
// surrounding space rather than a cloud around the wing, small enough that
// the densest lattice setting stays a few tens of thousands of points.
constexpr float kAirMoteRadiusMetres = 35.0F;
// Lattice pitch at the two ends of the Air slider, metres.
constexpr double kAirSpacingSparse = 9.0;
constexpr double kAirSpacingDense = 2.0;
// Slider position (1..100) to lattice pitch. Geometric, because the mote
// COUNT goes as the cube of the pitch: a linear map would spend most of
// the travel between "nothing there" and "solid fog".
double airSpacingFor(int sliderValue)
{
    const double fraction = std::clamp(sliderValue / 100.0, 0.0, 1.0);
    return kAirSpacingSparse
           * std::pow(kAirSpacingDense / kAirSpacingSparse, fraction);
}
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
// Signed pressure quantities use an actual diverging scale: negative Cp or
// inward fabric load is blue, zero is neutral, and positive Cp or outward
// load is red. Reusing the sequential ramp painted every negative Cp as the
// same "unloaded" blue and hid its magnitude completely.
const QVector3D kSignedNegativeTint(0.16F, 0.29F, 0.62F);
const QVector3D kSignedNeutralTint(0.66F, 0.68F, 0.66F);
const QVector3D kSignedPositiveTint(0.83F, 0.24F, 0.20F);

// One calibrated colour bar of the legend: what is plotted, the ramp's
// numeric range, the live extreme, and how to print a value in the bar's
// own unit. Diverging bars put neutral zero at its true place in the range.
struct LegendBar
{
    QString title;
    double minimum = 0.0;
    double maximum = 1.0;
    double marker = 0.0;
    std::function<QString(double)> format;
    bool diverging = false;
};
} // namespace

// A minimal orbit-camera OpenGL view running the XPBD body on a timer.
// Steps the wing on its own thread so the GUI stays fluid at any mesh
// density: at 4x subdivision one solver frame costs hundreds of
// milliseconds, and no amount of GUI-thread chunking survives that.
// Ownership is strict — the SimBody, the baseline and every metrics
// call live on this thread only. The GUI sees the simulation solely
// through SNAPSHOTS (positions, the active colour field, line
// tensions, the instrument report) exchanged through a triple-buffered
// slot: the worker fills its back buffer and swaps it in; the GUI
// swaps it out and paints at its own pace. Inputs flow the other way —
// live controls under the same mutex, structural changes (rebuild,
// grab, settle) as queued commands handled between frames.
// The session log. Everything the collapse diagnostics report, written
// from the worker thread as the wing flies, so a session that ended in a
// shape nobody can explain leaves a record instead of a memory. Truncated
// on every rebuild — Reset included — because a log that spans two
// different wings is a log nobody can read.
//
// Written by the worker and nobody else: the SimBody it measures lives on
// that thread, and the file follows it rather than needing a lock.
class SessionLog
{
public:
    static QString path()
    {
        return QStandardPaths::writableLocation(
                   QStandardPaths::AppLocalDataLocation)
               + QStringLiteral("/playground-session.log");
    }

    void restart(const QString &header)
    {
        const QString target = path();
        QDir().mkpath(QFileInfo(target).absolutePath());
        file_.setFileName(target);
        // Truncate: WriteOnly without Append does exactly that.
        open_ = file_.open(QIODevice::WriteOnly | QIODevice::Text);
        if (!open_) {
            return;
        }
        stream_.setDevice(&file_);
        stream_ << header;
        stream_.flush();
    }

    void write(const QString &line)
    {
        if (!open_) {
            return;
        }
        stream_ << line;
        // Flushed every line: the whole point is to survive the session,
        // and a session can end by being closed mid-flight.
        stream_.flush();
    }

    [[nodiscard]] bool isOpen() const { return open_; }

private:
    QFile file_;
    QTextStream stream_;
    bool open_ = false;
};

class SimWorker : public QThread
{
public:
    struct Snapshot
    {
        // Everything paintGL needs, in render precision.
        std::vector<QVector3D> positions;
        // For the mode in fieldMode: per NODE (metres for deviation,
        // strain for stress/slack) or per FACE (pascals for internal/fabric
        // pressure, dimensionless for exterior Cp); empty in Plain mode.
        // The tag lets the GUI keep interpreting
        // the field under the ramp that produced it during the window
        // between a mode switch and the worker's recomputation.
        std::vector<float> colourField;
        int fieldMode = 0;
        std::vector<float> lineTension;   // parallel to the segments
        // The instrument pass, refreshed on its own slower cadence.
        lep::playground::ShapeReport report;
        // Fast per-frame readouts for the dial and free-flight HUD.
        double liftNewtons = 0.0;
        double alphaDegrees = 0.0;
        double airspeed = 0.0;
        double glideRatio = 0.0;
        double forwardSpeed = 0.0;
        double sinkSpeed = 0.0;
        double pilotBelowMetres = 0.0;
        double pilotMassKg = 0.0;
        double polarDragTargetNewtons = 0.0;
        double polarDragTractionNewtons = 0.0;
        double polarDragTractionPowerWatts = 0.0;
        double trimmedLaunchAirspeed = 0.0;
        double trimmedLaunchDynamicPressure = 0.0;
        double trimmedLaunchEffectiveLiftCoefficient = 0.0;
        double trimmedLaunchHorizontalResidualNewtons = 0.0;
        double trimmedLaunchVerticalResidualNewtons = 0.0;
        int trimmedLaunchCalibrationIterations = 0;
        int trimmedLaunchRelaxationFrames = 0;
        lep::playground::PressureSolveDiagnostics pressureSolve;
        // Simulated time since the last build or Reset, seconds. Not wall
        // clock: the wing runs at its own 60 Hz however long a frame takes
        // to compute, and at 30x2 on a real wing that is two to three
        // times slower than the clock on the wall. Without this on screen
        // there is no way to tell a two-second surge from a five-second
        // one, and every judgement about how fast the wing answers a
        // control is made against the wrong clock.
        double simSeconds = 0.0;
        // How far the air has slid past the wing since the build. The
        // only thing in the model that knows the wing is going anywhere.
        QVector3D airTravel;
        bool polarActive = false;
        bool grabActive = false;
        double grabForceNewtons = 0.0;
        double grabPullMetres = 0.0;
        // Settle progress; done latches until the GUI acknowledges by
        // cancelling the settle.
        bool settleRunning = false;
        bool settleDone = false;
        bool settleConverged = false;
        double settleSimSeconds = 0.0;
        double settleAgitation = 0.0;
        QString simError;
    };

    // Static per-build data the renderer keys the positions by.
    struct Topology
    {
        std::vector<RenderFace> renderFaces;
        std::vector<LineSegment> lineSegments;
        std::size_t skinTriangleCount = 0;
        std::vector<std::size_t> junctions;   // for grab picking
        softwing::Vec3 boundsLow;
        softwing::Vec3 boundsHigh;
        std::size_t pilotNode = noConstraint;
        bool freeFlight = false;
        QString materialSummary;
        QString buildError;
    };

    SimWorker() { start(); }

    // Joining can block for one solver frame or one buildSimBody —
    // seconds at heavy subdivision. Accepted: teardown happens at page
    // destruction only, and aborting mid-build has no safe point.
    ~SimWorker() override
    {
        {
            QMutexLocker lock(&mutex_);
            quit_ = true;
        }
        wake_.wakeAll();
        wait();
    }

    // A full rebuild: mesh, options and controls are copied; the wing
    // returns to its rest pose. Supersedes any queued rebuild.
    void requestRebuild(SimMesh mesh,
                        const SimBuildOptions &options,
                        const SimControls &controls,
                        int colorMode)
    {
        QMutexLocker lock(&mutex_);
        pendingMesh_ = std::move(mesh);
        pendingOptions_ = options;
        rebuildRequested_ = true;
        controls_ = controls;
        colorMode_ = colorMode;
        settleRequested_ = false;
        settleCancel_ = true;
        wake_.wakeAll();
    }

    // Live inputs: cheap, applied before the next frame. displayDirty
    // makes a paused worker refresh its snapshot so a colour-mode
    // change recolours a frozen wing.
    void updateInputs(const SimControls &controls, int colorMode)
    {
        QMutexLocker lock(&mutex_);
        controls_ = controls;
        if (colorMode != colorMode_) {
            colorMode_ = colorMode;
            displayDirty_ = true;
        }
        wake_.wakeAll();
    }

    void setPaused(bool paused)
    {
        QMutexLocker lock(&mutex_);
        paused_ = paused;
        wake_.wakeAll();
    }

    void beginGrab(std::size_t junction)
    {
        QMutexLocker lock(&mutex_);
        grabBegin_ = junction;
        // A begin supersedes a queued end (release-then-regrab within
        // one worker cycle): sim-side beginGrab already releases any
        // previous grab, and replaying the stale end AFTER the new
        // begin killed the fresh grab.
        grabEnd_ = false;
        wake_.wakeAll();
    }

    void moveGrab(const softwing::Vec3 &target)
    {
        QMutexLocker lock(&mutex_);
        grabTarget_ = target;
        grabMove_ = true;
        wake_.wakeAll();
    }

    void endGrab()
    {
        QMutexLocker lock(&mutex_);
        grabEnd_ = true;
        wake_.wakeAll();
    }

    void startSettle(double budgetSeconds)
    {
        QMutexLocker lock(&mutex_);
        settleBudget_ = budgetSeconds;
        settleRequested_ = true;
        settleCancel_ = false;
        paused_ = false;
        wake_.wakeAll();
    }

    void cancelSettle()
    {
        QMutexLocker lock(&mutex_);
        settleCancel_ = true;
        // Erase a pending start too: the flags are a batch, not a
        // queue, and without this a start+cancel arriving in one worker
        // cycle replayed in inverted order and ran a phantom settle.
        // startSettle clears settleCancel_ symmetrically, so the last
        // GUI call wins in both orders.
        settleRequested_ = false;
        // So the cleared settle state reaches the GUI even while paused.
        displayDirty_ = true;
        wake_.wakeAll();
    }

    // ---- The session log, worker-thread only. ----

    void startLog(const SimBody &sim,
                  const SimControls &controls,
                  const QString &error)
    {
        QString header;
        QTextStream out(&header);
        out << "LEparagliding Playground session log\n"
            << "started        "
            << QDateTime::currentDateTime().toString(Qt::ISODate) << "\n";
        if (!error.isEmpty()) {
            out << "BUILD FAILED   " << error << "\n";
            log_.restart(header);
            return;
        }
        out << "body           " << sim.body->nodes().size() << " nodes, "
            << sim.body->triangles().size() << " triangles, "
            << sim.body->constraints().size() << " constraints\n"
            << "wing           " << QString::number(sim.planformArea, 'f', 2)
            << " m2 planform, AR "
            << QString::number(sim.aspectRatio, 'f', 2) << ", pilot "
            << QString::number(controls.pilotMassKg, 'f', 1)
            << " kg requested, "
            << QString::fromLatin1(
                   lep::playground::launchModeName(controls.launchMode))
            << "\n"
            << "skin           "
            << QString::fromLatin1(lep::playground::skinModelName(sim.skinModel))
            << ", " << sim.body->membraneElements().size()
            << " membranes, " << sim.body->dihedralConstraints().size()
            << " hinges, skipped " << sim.skippedMembraneElements << "/"
            << sim.skippedDihedralHinges << " elements/hinges\n";
        if (sim.skinModel == SkinModel::OrthotropicMembrane) {
            const auto &m = sim.skinMaterial;
            out << "material       warp/weft/coupling/shear "
                << QString::number(m.warpStiffness, 'f', 0) << "/"
                << QString::number(m.weftStiffness, 'f', 0) << "/"
                << QString::number(m.couplingStiffness, 'f', 0) << "/"
                << QString::number(m.shearStiffness, 'f', 0)
                << " N/m, damping "
                << QString::number(m.dampingTime, 'f', 3)
                << " s, compression "
                << QString::number(m.compressionStiffnessRatio, 'f', 3)
                << ", bend compliance "
                << QString::number(sim.skinBendCompliance, 'g', 3) << "\n";
            if (m.dampingTime > 0.0) {
                out << "WARNING        nonzero membrane damping is "
                       "experimental in the mixed rib/line network\n";
            }
        }
        out
            << "line mass      "
            << QString::number(sim.authoredLineMassKg, 'f', 3)
            << " kg authored + "
            << QString::number(sim.lineJunctionFloorMassKg, 'f', 4)
            << " kg junction floor + "
            << QString::number(sim.controlNodeFloorMassKg, 'f', 4)
            << " kg control floor\n";
        if (sim.virtualAddedAirMassKg > 0.0) {
            out << "added air      "
                << QString::number(sim.virtualAddedAirMassKg, 'f', 1)
                << " kg solver inertia (zero weight)\n";
        }
        if (sim.tunnelLineSolverBallastKg > 0.0) {
            out << "line relaxation "
                << QString::number(sim.tunnelLineSolverBallastKg, 'f', 3)
                << " kg nonphysical pinned-tunnel solver ballast"
                   " (zero gravity)\n";
        }
        out
            << "solver         " << controls.substeps << " substeps x "
            << controls.constraintIterations << " iterations";
        if (controls.freeFlight && controls.freeFlightCableSweepPairs > 0) {
            out << " + " << controls.freeFlightCableSweepPairs
                << " reverse/forward load-path pair"
                << (controls.freeFlightCableSweepPairs == 1 ? "" : "s");
        }
        out << "\n"
            << "pressure solve "
            << (controls.pressureSolveMode
                        == PressureSolveMode::BoundedExteriorCp
                    ? "bounded final exterior Cp [-3, 1]"
                    : "LEGACY increment + post-clamp")
            << "\n"
            << "cells          " << sim.cells.size()
            << ", cross-port gain x"
            << QString::number(controls.crossPortGain, 'f', 1) << "\n";
        if (controls.freeFlight
            && controls.launchMode == LaunchMode::TrimmedGlide) {
            const double support =
                (sim.pressureSolve.achievedForce
                 + sim.lastPolarDragTractionForce)
                    .z;
            out << "launch trim    "
                << QString::number(sim.trimmedLaunchAirspeed, 'f', 2)
                << " m/s, "
                << QString::number(sim.trimmedLaunchDynamicPressure, 'f', 1)
                << " Pa, effective CL "
                << QString::number(
                       sim.trimmedLaunchEffectiveLiftCoefficient, 'f', 3)
                << ", achieved wing support "
                << QString::number(support, 'f', 0) << " N, residual H/V "
                << QString::number(
                       sim.trimmedLaunchHorizontalResidualNewtons, 'f', 0)
                << "/"
                << QString::number(
                       sim.trimmedLaunchVerticalResidualNewtons, 'f', 0)
                << " N, " << sim.trimmedLaunchCalibrationIterations
                << " calibration solves + "
                << sim.trimmedLaunchRelaxationFrames
                << " relaxation frames\n";
        }
        std::size_t ported = 0;
        for (std::size_t cell = 0; cell + 1 < sim.cells.size(); ++cell) {
            if (sim.cells[cell].portAreaToNext > 0.0) {
                ++ported;
            }
        }
        out << "cross-ports    " << ported << " of "
            << (sim.cells.empty() ? 0 : sim.cells.size() - 1)
            << " rib crossings ported\n"
            << "\n"
            << "  Rows are 0.5 s of SIMULATED time apart. 'real' is the "
               "wall-clock cost of\n"
            << "  one frame: at 16.7 ms the wing flies in real time, at "
               "125 ms it is 7x slow.\n"
            << "\n"
            << "   sim s   real ms   q Pa  alpha    brakeL/R cm   "
               "airspeed  sink   L/D   span  vol%   resolved Pa "
               "risers N  slack   weak cell s/v/m      kink        "
               "fabric N polarD A/T/W      Cp range       authority F/P/D\n";
        log_.restart(header);
    }

    void logFrame(const SimBody &sim,
                  const SimControls &controls,
                  double frameSeconds)
    {
        if (!sim.body) {
            return;
        }
        // Counted before the log is consulted, not after: this is also
        // the HUD's clock now, and a session where the log file could not
        // be opened would otherwise report the wing frozen at zero
        // seconds while it flew.
        loggedSeconds_ += lep::playground::simulationTimeStep;
        if (!log_.isOpen()) {
            return;
        }
        // Every control change gets its own line whatever the cadence:
        // what the pilot did is the half of the record that explains the
        // other half.
        const bool changed =
            std::abs(controls.brakeLeft - loggedControls_.brakeLeft) > 0.005
            || std::abs(controls.brakeRight - loggedControls_.brakeRight)
                   > 0.005
            || std::abs(controls.pressurePascal
                        - loggedControls_.pressurePascal)
                   > 0.5
            || std::abs(controls.angleOfAttackDegrees
                        - loggedControls_.angleOfAttackDegrees)
                   > 0.05
            || controls.freeFlight != loggedControls_.freeFlight
            || controls.fabricContact != loggedControls_.fabricContact
            || std::abs(controls.crossPortGain
                        - loggedControls_.crossPortGain)
                   > 0.01
            || controls.substeps != loggedControls_.substeps;
        if (changed) {
            QString line;
            QTextStream out(&line);
            out << "  ---- " << QString::number(loggedSeconds_, 'f', 1)
                << "s  brakes "
                << QString::number(controls.brakeLeft * 100.0, 'f', 0)
                << "/"
                << QString::number(controls.brakeRight * 100.0, 'f', 0)
                << " cm, q "
                << QString::number(controls.pressurePascal, 'f', 0)
                << " Pa, angle "
                << QString::number(controls.angleOfAttackDegrees, 'f', 1)
                << " deg, "
                << (controls.freeFlight ? "free flight" : "tunnel")
                << (controls.fabricContact ? ", contact" : "")
                << ", cross-port x"
                << QString::number(controls.crossPortGain, 'f', 1) << ", "
                << controls.substeps << "x" << controls.constraintIterations
                << "\n";
            log_.write(line);
            loggedControls_ = controls;
        }
        if (++loggedFrames_ < 30) {
            return;
        }
        loggedFrames_ = 0;

        const lep::playground::WeakCellReport weak =
            lep::playground::weakestCell(sim);
        const lep::playground::KinkReport kink =
            lep::playground::sharpestKink(sim);
        const lep::playground::LineLoadReport lines =
            lep::playground::lineLoads(sim, controls);
        double cellLow = 0.0;
        double cellHigh = 0.0;
        if (!sim.cellPressure.empty()) {
            cellLow = *std::min_element(sim.cellPressure.begin(),
                                        sim.cellPressure.end());
            cellHigh = *std::max_element(sim.cellPressure.begin(),
                                         sim.cellPressure.end());
        }
        double spanLow = std::numeric_limits<double>::max();
        double spanHigh = std::numeric_limits<double>::lowest();
        for (std::size_t node = 0; node < sim.canopyNodeCount
                                   && node < sim.body->nodes().size();
             ++node) {
            const double x = sim.body->nodes()[node].position.x;
            spanLow = std::min(spanLow, x);
            spanHigh = std::max(spanHigh, x);
        }
        // Sink through the surrounding air and enclosed volume, computed
        // here rather than carried on
        // the body: neither is wanted anywhere else, and the log is the
        // one place both have to line up with the same frame.
        double sink = 0.0;
        double mass = 0.0;
        for (const softwing::Node &node : sim.body->nodes()) {
            if (node.inverseMass <= 0.0) {
                continue;
            }
            const double nodeMass = 1.0 / node.inverseMass;
            sink += nodeMass * node.velocity.z;
            mass += nodeMass;
        }
        sink = mass > 0.0
                   ? sink / mass - controls.ambientAirVelocityWorld.z
                   : 0.0;
        double volume = 0.0;
        for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
            const softwing::Triangle &tri = sim.body->triangles()[face];
            const softwing::Vec3 &a = sim.body->nodes()[tri.a].position;
            const softwing::Vec3 &b = sim.body->nodes()[tri.b].position;
            const softwing::Vec3 &c = sim.body->nodes()[tri.c].position;
            volume += dot(a, cross(b, c)) / 6.0;
        }
        const double volumePercent =
            loggedRestVolume_ > 0.0
                ? 100.0 * (std::abs(volume) - loggedRestVolume_)
                      / loggedRestVolume_
                : 0.0;

        QString line;
        QTextStream out(&line);
        const auto field = [](double value, int width, int decimals) {
            return QStringLiteral("%1").arg(
                QString::number(value, 'f', decimals), width);
        };
        out << field(loggedSeconds_, 8, 1) << field(frameSeconds * 1000.0, 10, 1)
            << field(controls.pressurePascal, 7, 0)
            << field(sim.lastAlphaDegrees, 7, 1)
            << field(controls.brakeLeft * 100.0, 9, 0) << "/"
            << field(controls.brakeRight * 100.0, 3, 0)
            << field(sim.lastAirspeed, 12, 2) << field(sink, 7, 2)
            << field(sim.lastGlideRatio, 6, 2)
            << field(spanHigh - spanLow, 7, 2)
            << field(volumePercent, 6, 1) << "  "
            << field(cellLow, 5, 0) << ".." << field(cellHigh, 4, 0)
            << field(lines.riserNewtons, 10, 0)
            << field(static_cast<double>(lines.slackSegments), 7, 0) << "  #"
            << weak.index << " x" << field(weak.x, 6, 2) << " "
            << field(100.0 * weak.sectionRatio, 3, 0) << "/"
            << field(100.0 * weak.volumeRatio, 3, 0) << "/"
            << field(100.0 * weak.intakeOpening, 3, 0) << "%  "
            << field(kink.degrees, 4, 0) << "deg@" << kink.rib << " s"
            << field(kink.spanFraction, 4, 2)
            << field(sim.lastFabricDragNewtons, 9, 0)
            << field(sim.lastPolarDragTractionNewtons, 8, 0) << "/"
            << field(sim.lastPolarDragTargetNewtons, 5, 0) << "/"
            << field(sim.lastPolarDragTractionPowerWatts, 7, 0)
            << "  " << field(sim.pressureSolve.minimumCp, 5, 2)
            << ".." << field(sim.pressureSolve.maximumCp, 5, 2)
            << "  " << field(sim.pressureSolve.authority[0], 5, 3)
            << "/" << field(sim.pressureSolve.authority[1], 5, 3)
            << "/" << field(sim.pressureSolve.authority[2], 5, 3)
            << " h" << field(sim.pressureSolve.authorityHint[0], 5, 3)
            << "/" << field(sim.pressureSolve.authorityHint[1], 5, 3)
            << "/" << field(sim.pressureSolve.authorityHint[2], 5, 3)
            << " p" << (sim.pressureSolve.authorityProbeAccepted[0] ? '+' : '-')
            << (sim.pressureSolve.authorityProbeAccepted[1] ? '+' : '-')
            << (sim.pressureSolve.authorityProbeAccepted[2] ? '+' : '-')
            << (sim.pressureSolve.numericalFailure
                    ? " NUMERICAL-FAIL"
                    : "")
            << "\n";
        log_.write(line);
    }

    // Swap the freshest snapshot/topology out, surrendering the
    // previous front buffer for reuse. False when nothing new arrived.
    bool takeSnapshot(Snapshot &front)
    {
        QMutexLocker lock(&mutex_);
        if (!snapshotFresh_) {
            return false;
        }
        std::swap(front, sharedSnapshot_);
        snapshotFresh_ = false;
        return true;
    }

    bool takeTopology(Topology &front)
    {
        QMutexLocker lock(&mutex_);
        if (!topologyFresh_) {
            return false;
        }
        std::swap(front, sharedTopology_);
        topologyFresh_ = false;
        return true;
    }

    void run() override
    {
        QElapsedTimer pace;
        pace.start();
        QElapsedTimer fieldClock;
        QElapsedTimer reportClock;
        lep::playground::ShapeBaseline baseline;
        SimBuildOptions builtOptions;
        std::unique_ptr<lep::playground::SettleMonitor> monitor;
        bool settling = false;
        bool settleDone = false;
        bool settleConverged = false;
        SimBody sim;
        QString simError;

        while (true) {
            // --- Inputs, under the lock. ---
            SimControls controls;
            int colorMode = 0;
            bool doRebuild = false;
            SimMesh mesh;
            SimBuildOptions options;
            bool paused = false;
            std::size_t grabBegin = noConstraint;
            bool grabMove = false;
            softwing::Vec3 grabTarget;
            bool grabEnd = false;
            bool startSettle = false;
            double settleBudget = 30.0;
            bool displayDirty = false;
            {
                QMutexLocker lock(&mutex_);
                if (quit_) {
                    return;
                }
                const bool idle =
                    !rebuildRequested_ && !displayDirty_
                    && grabBegin_ == noConstraint && !grabMove_
                    && !grabEnd_ && !settleRequested_
                    && (((paused_ || !simError.isEmpty()) && !settling)
                        || !sim.body);
                if (idle && !settleCancel_) {
                    wake_.wait(&mutex_);
                    if (quit_) {
                        return;
                    }
                }
                controls = controls_;
                colorMode = colorMode_;
                paused = paused_;
                displayDirty = displayDirty_;
                displayDirty_ = false;
                if (rebuildRequested_) {
                    doRebuild = true;
                    rebuildRequested_ = false;
                    mesh = std::move(pendingMesh_);
                    options = pendingOptions_;
                }
                grabBegin = grabBegin_;
                grabBegin_ = noConstraint;
                grabMove = grabMove_;
                grabMove_ = false;
                grabTarget = grabTarget_;
                grabEnd = grabEnd_;
                grabEnd_ = false;
                if (settleRequested_) {
                    startSettle = true;
                    settleRequested_ = false;
                    settleBudget = settleBudget_;
                }
                if (settleCancel_) {
                    settling = false;
                    settleDone = false;
                    settleCancel_ = false;
                    monitor.reset();
                }
            }

            // --- Structural work, off the lock. ---
            if (doRebuild) {
                simError.clear();
                settling = false;
                settleDone = false;
                monitor.reset();
                builtOptions = options;
                try {
                    sim = buildSimBody(mesh, options, controls);
                    baseline =
                        lep::playground::captureShapeBaseline(sim);
                } catch (const std::exception &failure) {
                    simError = QString::fromUtf8(failure.what());
                    sim = SimBody{};
                }
                loggedFrames_ = 0;
                loggedSeconds_ = 0.0;
                loggedControls_ = controls;
                loggedRestVolume_ = 0.0;
                if (sim.body) {
                    for (std::size_t face = 0; face < sim.skinTriangleCount;
                         ++face) {
                        const softwing::Triangle &tri =
                            sim.body->triangles()[face];
                        loggedRestVolume_ +=
                            dot(sim.body->nodes()[tri.a].position,
                                cross(sim.body->nodes()[tri.b].position,
                                      sim.body->nodes()[tri.c].position))
                            / 6.0;
                    }
                    loggedRestVolume_ = std::abs(loggedRestVolume_);
                }
                startLog(sim, controls, simError);
                publishTopology(sim, options, simError);
                fieldClock.invalidate();
                reportClock.invalidate();
                publishSnapshot(sim, baseline, builtOptions, controls,
                                colorMode, settling, settleDone,
                                settleConverged, monitor.get(),
                                simError, true, true);
                continue;
            }
            if (!sim.body) {
                continue;
            }
            if (grabBegin != noConstraint) {
                lep::playground::beginGrab(sim, grabBegin);
            }
            if (grabMove) {
                lep::playground::moveGrab(sim, grabTarget);
            }
            if (grabEnd) {
                lep::playground::endGrab(sim);
            }
            if (startSettle) {
                monitor =
                    std::make_unique<lep::playground::SettleMonitor>(
                        settleBudget);
                settling = true;
                settleDone = false;
                settleConverged = false;
            }

            const bool stepping =
                simError.isEmpty() && (!paused || settling);
            if (stepping) {
                try {
                    const auto frameStart =
                        std::chrono::steady_clock::now();
                    lep::playground::stepSimulation(sim, controls);
                    logFrame(sim,
                             controls,
                             std::chrono::duration<double>(
                                 std::chrono::steady_clock::now()
                                 - frameStart)
                                 .count());
                    if (settling && monitor != nullptr
                        && monitor->frameStepped(
                            sim, controls.pressurePascal)) {
                        settleConverged = monitor->settled();
                        settling = false;
                        settleDone = true;
                        // The Done numbers must describe the exact
                        // frozen pose, not a report from up to half a
                        // second of flat-out stepping earlier.
                        fieldClock.invalidate();
                        reportClock.invalidate();
                        // Freeze at the converged pose; the GUI
                        // acknowledges with cancelSettle and decides
                        // the run state. Guarded: a cancel or rebuild
                        // already pending owns the run state instead.
                        QMutexLocker lock(&mutex_);
                        if (!settleCancel_) {
                            paused_ = true;
                        }
                    }
                } catch (const std::exception &failure) {
                    simError = QString::fromUtf8(failure.what());
                    // settleDone only when a settle was the thing that
                    // failed: outside one, nothing ever acknowledges
                    // the latch.
                    settleDone = settling;
                    settling = false;
                    settleConverged = false;
                }
            } else if (!displayDirty) {
                continue;
            }

            const bool wantFields =
                !fieldClock.isValid()
                || fieldClock.elapsed() >= fieldRefreshMilliseconds
                || displayDirty;
            const bool wantReport = !reportClock.isValid()
                                    || reportClock.elapsed() >= 500;
            if (wantFields) {
                fieldClock.start();
            }
            if (wantReport) {
                reportClock.start();
            }
            publishSnapshot(sim, baseline, builtOptions, controls,
                            colorMode, settling, settleDone,
                            settleConverged, monitor.get(), simError,
                            wantFields, wantReport);

            // Pace to the display rate when the solver is faster than
            // it; a settle runs flat out on purpose.
            if (stepping && !settling) {
                const qint64 remaining = 16 - pace.elapsed();
                if (remaining > 0) {
                    msleep(static_cast<unsigned long>(remaining));
                }
            }
            pace.start();
        }
    }

private:
    void publishTopology(const SimBody &sim,
                         const SimBuildOptions &options,
                         const QString &error)
    {
        backTopology_.renderFaces = sim.renderFaces;
        backTopology_.lineSegments = sim.lineSegments;
        backTopology_.skinTriangleCount = sim.skinTriangleCount;
        backTopology_.boundsLow = sim.boundsLow;
        backTopology_.boundsHigh = sim.boundsHigh;
        backTopology_.pilotNode = sim.pilotNode;
        backTopology_.freeFlight = sim.pilotNode != noConstraint;
        backTopology_.materialSummary.clear();
        if (sim.body) {
            backTopology_.materialSummary = QStringLiteral(
                "%1 · %2 membranes · %3 hinges · skipped %4/%5")
                .arg(QString::fromLatin1(
                    lep::playground::skinModelName(sim.skinModel)))
                .arg(sim.body->membraneElements().size())
                .arg(sim.body->dihedralConstraints().size())
                .arg(sim.skippedMembraneElements)
                .arg(sim.skippedDihedralHinges);
            if (sim.skinModel == SkinModel::OrthotropicMembrane
                && sim.skinMaterial.dampingTime > 0.0) {
                backTopology_.materialSummary += QStringLiteral(
                    " · WARNING nonzero membrane damping is experimental");
            }
        }
        backTopology_.buildError = error;
        backTopology_.junctions.clear();
        std::set<std::size_t> unique;
        for (const LineSegment &segment : sim.lineSegments) {
            unique.insert(segment.a);
            unique.insert(segment.b);
        }
        backTopology_.junctions.assign(unique.begin(), unique.end());
        Q_UNUSED(options);
        QMutexLocker lock(&mutex_);
        std::swap(sharedTopology_, backTopology_);
        topologyFresh_ = true;
    }

    void publishSnapshot(const SimBody &sim,
                         const lep::playground::ShapeBaseline &baseline,
                         const SimBuildOptions &options,
                         const SimControls &controls,
                         int colorMode,
                         bool settling,
                         bool settleDone,
                         bool settleConverged,
                         const lep::playground::SettleMonitor *monitor,
                         const QString &error,
                         bool refreshFields,
                         bool refreshReport)
    {
        Snapshot &back = backSnapshot_;
        back.simError = error;
        if (sim.body) {
            const auto &nodes = sim.body->nodes();
            back.positions.resize(nodes.size());
            for (std::size_t index = 0; index < nodes.size(); ++index) {
                const softwing::Vec3 &p = nodes[index].position;
                back.positions[index] =
                    QVector3D(static_cast<float>(p.x),
                              static_cast<float>(p.y),
                              static_cast<float>(p.z));
            }
            back.lineTension.resize(sim.lineSegments.size());
            for (std::size_t index = 0;
                 index < sim.lineSegments.size(); ++index) {
                back.lineTension[index] = static_cast<float>(
                    lep::playground::constraintTensionNewtons(
                        sim, controls,
                        sim.lineSegments[index].constraint));
            }
            if (refreshFields) {
                currentFieldMode_ = colorMode;
                // ColorMode order matches the page's combo. Computed
                // into the PERSISTENT field and copied out per
                // snapshot: writing straight into the ping-pong buffer
                // alternated a fresh field with a stale one and the
                // heatmap flickered.
                if (colorMode == 4) {
                    // Per FACE and deliberately flat: every face belonging
                    // to one lumped pneumatic cell receives the same value.
                    lep::playground::faceInteriorPressureField(
                        sim, currentField_);
                } else if (colorMode == 5) {
                    lep::playground::faceExteriorPressureCoefficientField(
                        sim, currentField_);
                } else if (colorMode == 6) {
                    lep::playground::facePressureDifferenceField(
                        sim, currentField_);
                } else if (colorMode == 2) {
                    lep::playground::nodeDeviationField(
                        sim, baseline, currentField_);
                } else if (colorMode == 1 || colorMode == 3) {
                    lep::playground::nodeStrainFields(
                        sim, options.detailedRibs,
                        colorMode == 1 ? currentField_ : scratchField_,
                        colorMode == 3 ? currentField_
                                       : scratchField_);
                } else {
                    currentField_.clear();
                }
            }
            back.colourField = currentField_;
            back.fieldMode = currentFieldMode_;
            if (refreshReport && !baseline.restPositions.empty()) {
                currentReport_ = lep::playground::measureShape(
                    sim, controls, baseline);
            }
            back.report = currentReport_;
            back.polarActive =
                controls.flightLoad || controls.freeFlight;
            back.liftNewtons = sim.lastLift;
            back.alphaDegrees = sim.lastAlphaDegrees;
            back.airspeed = sim.lastAirspeed;
            back.airTravel = QVector3D(
                static_cast<float>(sim.airTravel.x),
                static_cast<float>(sim.airTravel.y),
                static_cast<float>(sim.airTravel.z));
            back.glideRatio = sim.lastGlideRatio;
            back.pilotMassKg = sim.pilotMass;
            back.polarDragTargetNewtons =
                sim.lastPolarDragTargetNewtons;
            back.polarDragTractionNewtons =
                sim.lastPolarDragTractionNewtons;
            back.polarDragTractionPowerWatts =
                sim.lastPolarDragTractionPowerWatts;
            back.trimmedLaunchAirspeed = sim.trimmedLaunchAirspeed;
            back.trimmedLaunchDynamicPressure =
                sim.trimmedLaunchDynamicPressure;
            back.trimmedLaunchEffectiveLiftCoefficient =
                sim.trimmedLaunchEffectiveLiftCoefficient;
            back.trimmedLaunchHorizontalResidualNewtons =
                sim.trimmedLaunchHorizontalResidualNewtons;
            back.trimmedLaunchVerticalResidualNewtons =
                sim.trimmedLaunchVerticalResidualNewtons;
            back.trimmedLaunchCalibrationIterations =
                sim.trimmedLaunchCalibrationIterations;
            back.trimmedLaunchRelaxationFrames =
                sim.trimmedLaunchRelaxationFrames;
            back.pressureSolve = sim.pressureSolve;
            // The same counter the session log's first column carries, so
            // a moment on screen and a row in the log can be lined up.
            back.simSeconds = loggedSeconds_;
            back.forwardSpeed = 0.0;
            back.sinkSpeed = 0.0;
            back.pilotBelowMetres = 0.0;
            if (controls.freeFlight
                && sim.pilotNode != noConstraint) {
                softwing::Vec3 velocity;
                double mass = 0.0;
                softwing::Vec3 canopyCentre;
                std::size_t canopyCount = 0;
                const auto &nodes = sim.body->nodes();
                for (std::size_t index = 0; index < nodes.size();
                     ++index) {
                    const softwing::Node &node = nodes[index];
                    if (node.inverseMass > 0.0) {
                        const double nodeMass =
                            1.0 / node.inverseMass;
                        velocity += nodeMass * node.velocity;
                        mass += nodeMass;
                    }
                    if (index < sim.canopyNodeCount) {
                        canopyCentre += node.position;
                        ++canopyCount;
                    }
                }
                if (mass > 0.0) {
                    velocity /= mass;
                }
                const softwing::Vec3 throughAir =
                    velocity - controls.ambientAirVelocityWorld;
                const lep::playground::FlightFrameSample flightFrame =
                    lep::playground::sampleFlightFrame(sim, controls);
                if (flightFrame.valid) {
                    back.forwardSpeed =
                        dot(throughAir, flightFrame.forwardDirection);
                }
                back.sinkSpeed = throughAir.z;
                if (canopyCount > 0) {
                    canopyCentre /= double(canopyCount);
                    back.pilotBelowMetres =
                        canopyCentre.z
                        - nodes[sim.pilotNode].position.z;
                }
            }
            back.grabActive = lep::playground::grabActive(sim);
            back.grabForceNewtons =
                lep::playground::grabForceNewtons(sim, controls);
            back.grabPullMetres = 0.0;
            if (back.grabActive
                && sim.grabAnchorNode < sim.body->nodes().size()
                && sim.grabbedNode < sim.body->nodes().size()) {
                back.grabPullMetres = length(
                    sim.body->nodes()[sim.grabAnchorNode].position
                    - sim.body->nodes()[sim.grabbedNode].position);
            }
        } else {
            back.positions.clear();
            back.colourField.clear();
            back.lineTension.clear();
        }
        back.settleRunning = settling;
        back.settleDone = settleDone;
        back.settleConverged = settleConverged;
        back.settleSimSeconds =
            monitor != nullptr ? monitor->simulatedSeconds() : 0.0;
        back.settleAgitation =
            monitor != nullptr ? monitor->lastAgitation() : 0.0;

        QMutexLocker lock(&mutex_);
        std::swap(sharedSnapshot_, backSnapshot_);
        snapshotFresh_ = true;
    }

    // Log state. Worker thread only — no lock, because nothing else
    // touches it.
    SessionLog log_;
    int loggedFrames_ = 0;
    double loggedSeconds_ = 0.0;
    double loggedRestVolume_ = 0.0;
    SimControls loggedControls_;

    QMutex mutex_;
    QWaitCondition wake_;
    bool quit_ = false;
    bool paused_ = false;
    bool rebuildRequested_ = false;
    bool displayDirty_ = false;
    SimMesh pendingMesh_;
    SimBuildOptions pendingOptions_;
    SimControls controls_;
    int colorMode_ = 0;
    std::size_t grabBegin_ = noConstraint;
    bool grabMove_ = false;
    softwing::Vec3 grabTarget_;
    bool grabEnd_ = false;
    bool settleRequested_ = false;
    bool settleCancel_ = false;
    double settleBudget_ = 30.0;
    Snapshot sharedSnapshot_;
    Snapshot backSnapshot_;
    bool snapshotFresh_ = false;
    Topology sharedTopology_;
    Topology backTopology_;
    bool topologyFresh_ = false;
    std::vector<float> scratchField_;
    std::vector<float> currentField_;
    int currentFieldMode_ = 0;
    lep::playground::ShapeReport currentReport_;
};

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
        worker_ = std::make_unique<SimWorker>();
        // The GUI's only clock: poll the worker's snapshot slot and
        // repaint when something new arrived. Stepping happens on the
        // worker; this timer never blocks on it.
        timer_ = new QTimer(this);
        timer_->setInterval(16);
        connect(timer_, &QTimer::timeout, this, [this] { pollWorker(); });
        timer_->start();
    }

    void pollWorker()
    {
        bool changed = false;
        // Snapshot FIRST: a rebuild publishes topology and snapshot in
        // that order on the worker, so taking them the other way round
        // here framed the new wing against the previous body's
        // positions.
        const bool freshSnapshot = worker_->takeSnapshot(front_);
        if (freshSnapshot) {
            changed = true;
        }
        if (worker_->takeTopology(topo_)) {
            changed = true;
            // A grab that straddled the rebuild held a node of the
            // discarded body.
            if (grabbing_) {
                grabbing_ = false;
                setCursor(flyMode_ ? Qt::CrossCursor : Qt::ArrowCursor);
            }
            // A fresh build frames itself, exactly as the synchronous
            // rebuild used to.
            fitView();
            // ...and the follow starts over from it: where the discarded
            // body had flown to says nothing about where this one begins.
            haveFollowAnchor_ = false;
            if (topologyArrived_) {
                topologyArrived_(topo_.buildError);
            }
        }
        if (freshSnapshot) {
            followSystem();
        }
        if (changed) {
            update();
        }
    }

    // The mean of the live node positions — steadier than the bounding
    // box's centre, which a single fluttering tip can shift.
    QVector3D systemCentre() const
    {
        QVector3D sum;
        for (const QVector3D &position : front_.positions) {
            sum += position;
        }
        return front_.positions.empty()
                   ? target_
                   : sum / static_cast<float>(front_.positions.size());
    }

    // A flying wing travels through the world at flying speed, so a camera
    // anchored in the world loses it within seconds — and a Reset then
    // frames the launch pose the wing has already left, which looks like
    // the reset not being framed at all. Carry the orbit target along with
    // the system's own motion. Only the DELTA is applied, so the distance,
    // orbit angles and any pan the user has dialled in all survive.
    void followSystem()
    {
        if (!topo_.freeFlight || front_.positions.empty()) {
            haveFollowAnchor_ = false;
            return;
        }
        const QVector3D centre = systemCentre();
        if (haveFollowAnchor_) {
            target_ += centre - followAnchor_;
        }
        followAnchor_ = centre;
        haveFollowAnchor_ = true;
    }

    // Page hook, invoked on the GUI thread when a build completes (the
    // error is empty on success): the async build outcome would
    // otherwise be invisible.
    void setTopologyCallback(std::function<void(const QString &)> hook)
    {
        topologyArrived_ = std::move(hook);
    }

    QString buildFromMesh(const SimMesh &mesh)
    {
        // Retained so free flight can be toggled without another engine
        // run: the pilot only exists on a body built for free flight, so
        // the toggle is a rebuild.
        mesh_ = mesh;
        sendRebuild();
        return {};
    }

    // Hands the mesh to the worker; the built topology and first
    // snapshot come back through the poll. The wing returns to its rest
    // pose and runs.
    void sendRebuild()
    {
        if (mesh_.nodes.empty() || worker_ == nullptr) {
            return;
        }
        // A grab holds a node of the body being discarded.
        if (grabbing_) {
            grabbing_ = false;
            setCursor(flyMode_ ? Qt::CrossCursor : Qt::ArrowCursor);
        }
        worker_->requestRebuild(mesh_, buildOptions_, controls_,
                                static_cast<int>(colorMode_));
        worker_->setPaused(false);
        runningRequested_ = true;
        // The worker clears its error deterministically on rebuild;
        // clearing the GUI mirror now keeps isRunning() truthful for
        // the synchronous callers between here and the next snapshot.
        front_.simError.clear();
    }

    // Frame the wing (and, flying, the whole pendulum): target and
    // distance only, so the current view direction survives a re-fit.
    void fitView()
    {
        if (topo_.renderFaces.empty()) {
            return;
        }
        // Frame the WHOLE system — canopy, cascades, carabiners, pilot
        // — from the live node positions, not the mesh bounds: those
        // cover the canopy only, and a fit that cropped the lines
        // framed half the machine. The topology bounds remain the
        // fallback for the hand-off window before the first snapshot.
        softwing::Vec3 low = topo_.boundsLow;
        softwing::Vec3 high = topo_.boundsHigh;
        if (!front_.positions.empty()) {
            low = softwing::Vec3{1e9, 1e9, 1e9};
            high = softwing::Vec3{-1e9, -1e9, -1e9};
            for (const QVector3D &p : front_.positions) {
                low.x = std::min<double>(low.x, p.x());
                low.y = std::min<double>(low.y, p.y());
                low.z = std::min<double>(low.z, p.z());
                high.x = std::max<double>(high.x, p.x());
                high.y = std::max<double>(high.y, p.y());
                high.z = std::max<double>(high.z, p.z());
            }
        }
        const softwing::Vec3 focus = 0.5 * (low + high);
        const double extent = length(high - low);
        target_ = QVector3D(static_cast<float>(focus.x),
                            static_cast<float>(focus.y),
                            static_cast<float>(focus.z));
        // extent x 1.15, not the old x 2.0: at a 40° field of view the
        // full system still fits (the line convergence included), and
        // the old framing parked the wing in distant empty space.
        distance_ = static_cast<float>(1.15 * extent);
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
            sendRebuild();
        }
        update();
    }

    bool freeFlight() const { return controls_.freeFlight; }

    void setPilotMassKg(double kilograms)
    {
        const double clamped = std::clamp(
            kilograms, minimumPilotMassKg, maximumPilotMassKg);
        if (controls_.pilotMassKg == clamped) {
            return;
        }
        controls_.pilotMassKg = clamped;
        if (!mesh_.nodes.empty()) {
            sendRebuild();
        }
    }

    void setLaunchMode(LaunchMode mode)
    {
        if (controls_.launchMode == mode) {
            return;
        }
        controls_.launchMode = mode;
        if (!mesh_.nodes.empty()) {
            sendRebuild();
        }
    }

    void setSkinModel(SkinModel model)
    {
        if (controls_.skinModel == model) {
            return;
        }
        controls_.skinModel = model;
        if (!mesh_.nodes.empty()) {
            sendRebuild();
        }
    }

    QString materialReadout() const { return topo_.materialSummary; }

    // Back to the rest pose (and, in free flight, a fresh launch on the
    // glide), from the retained mesh — no engine run needed.
    void resetSimulation()
    {
        if (!mesh_.nodes.empty()) {
            sendRebuild();
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
        if (!controls_.freeFlight || front_.airspeed <= 0.0) {
            return {};
        }
        return QStringLiteral(
                   "%1 km/h · sink %2 m/s · glide %3 · α %4° · pilot %5 kg")
            .arg(front_.airspeed * 3.6, 0, 'f', 0)
            .arg(-front_.sinkSpeed, 0, 'f', 1)
            .arg(front_.glideRatio, 0, 'f', 1)
            .arg(front_.alphaDegrees, 0, 'f', 1)
            .arg(front_.pilotMassKg, 0, 'f', 0);
    }

    void setPressurePascal(double pressure)
    {
        controls_.pressurePascal = pressure;
        pushInputs();
    }

    // Fabric/line self-contact is a pure runtime option — the pass lives
    // in stepSimulation, nothing in the body changes — so unlike free
    // flight it needs no rebuild, just the next frame's controls.
    void setFabricContact(bool enabled)
    {
        controls_.fabricContact = enabled;
        pushInputs();
    }

    bool fabricContact() const { return controls_.fabricContact; }

    // Degrees between the airflow and the wing's rest chord. The load falls
    // out of the pressure field now, so this is the only handle the
    // aerodynamics needs — it replaces a slider that dialled in a fake force.
    void setAngleOfAttack(double degrees)
    {
        controls_.angleOfAttackDegrees = degrees;
        pushInputs();
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
        pushInputs();
    }

    void setCrossPortGain(double gain)
    {
        controls_.crossPortGain = gain;
        pushInputs();
    }

    // Metres between air motes; 0 clears the field. Denser is a shorter
    // lattice pitch, and the count goes as the cube of it, so the slider
    // is mapped to the pitch rather than to a count.
    void setAirSpacing(double metres)
    {
        airSpacingMetres_ = static_cast<float>(std::max(0.0, metres));
        update();
    }

    // Every live-controls change funnels through here to the worker.
    void pushInputs()
    {
        if (worker_ != nullptr) {
            worker_->updateInputs(controls_,
                                  static_cast<int>(colorMode_));
        }
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
        pushInputs();
    }

    // Skin heatmap source. The page's combo items are in this order.
    enum class ColorMode
    {
        Plain,
        Stress,
        Deviation,
        Slack,
        CellInteriorPressure,
        ExteriorPressureCoefficient,
        FabricPressureDifference,
    };

    void setColorMode(ColorMode mode)
    {
        if (colorMode_ == mode) {
            return;
        }
        colorMode_ = mode;
        // The worker recomputes the field for the new mode — also while
        // paused, so a frozen wing recolours (displayDirty).
        pushInputs();
        update();
    }

    // Wind-tunnel loading: impose the wing-level polar pass in pinned mode
    // so line loads are realistic. A per-step control, no rebuild.
    void setFlightLoad(bool enabled)
    {
        controls_.flightLoad = enabled;
        pushInputs();
    }

    // Snapshot for the analysis dialog, which drives its own bodies with
    // the tunnel's exact settings.
    SimControls controls() const { return controls_; }

    // Debug only: what the display snapshot actually holds.
    QString debugFieldSummary() const
    {
        float peak = 0.0F;
        for (const float value : front_.colourField) {
            peak = std::max(peak, std::abs(value));
        }
        return QStringLiteral("mode %1 field[%2] peak %3")
            .arg(static_cast<int>(colorMode_))
            .arg(front_.colourField.size())
            .arg(peak);
    }

    // For the angle dial: the polar pass's live numbers. Lift is 0 and
    // the angle stale when no polar pass runs (pinned without flight
    // load), which the dial states rather than hides.
    bool polarActive() const
    {
        return controls_.flightLoad || controls_.freeFlight;
    }
    double liveLiftNewtons() const { return front_.liftNewtons; }
    double liveAlphaDegrees() const { return front_.alphaDegrees; }

    // The settle runs on the worker; the GUI watches it through the
    // snapshot and acknowledges completion.
    void startSettle(double budgetSeconds)
    {
        if (worker_ != nullptr) {
            runningRequested_ = true;
            worker_->startSettle(budgetSeconds);
        }
    }
    void acknowledgeSettle()
    {
        if (worker_ != nullptr) {
            worker_->cancelSettle();
        }
    }
    bool settleRunning() const { return front_.settleRunning; }
    bool settleDone() const { return front_.settleDone; }
    bool settleConverged() const { return front_.settleConverged; }
    double settleSimSeconds() const { return front_.settleSimSeconds; }
    double settleAgitation() const { return front_.settleAgitation; }

    lep::playground::ShapeReport currentShapeReport() const
    {
        return front_.report;
    }

    // One line for the shape HUD: the live wing measured against its
    // design shape, in the units a designer reads. Empty until a body and
    // its baseline exist.
    QString shapeReadout() const
    {
        if (front_.positions.empty()) {
            return {};
        }
        const lep::playground::ShapeReport &report = front_.report;
        QStringList parts;
        // SIMULATED seconds, and first, because everything after it is
        // read as a rate. The wing keeps its own 60 Hz clock however long
        // a frame takes to compute, so on a real wing at 30x2 this runs
        // two to three times slower than the wall clock — and without it
        // in view, a surge that took the wing two seconds looks like five
        // and every judgement about how quickly it answered a control is
        // made against the wrong one.
        parts << QStringLiteral("Time %1 s")
                     .arg(front_.simSeconds, 0, 'f', 1);
        // The polar's numbers only exist when a polar pass runs; the raw
        // pinned pressure field carries no drag model worth quoting.
        if (controls_.flightLoad || controls_.freeFlight) {
            parts << QStringLiteral("L/D %1")
                         .arg(report.glideRatio, 0, 'f', 1)
                  << QStringLiteral("α %1°")
                         .arg(front_.alphaDegrees, 0, 'f', 1);
            const auto &solve = front_.pressureSolve;
            if (solve.numericalFailure) {
                parts << QStringLiteral("Cp SOLVE FAILED");
            } else if (solve.attempted) {
                parts << QStringLiteral(
                             "Cp %1..%2 · A %3/%4/%5 · H %6/%7/%8")
                             .arg(solve.minimumCp, 0, 'f', 2)
                             .arg(solve.maximumCp, 0, 'f', 2)
                             .arg(solve.authority[0], 0, 'f', 2)
                             .arg(solve.authority[1], 0, 'f', 2)
                             .arg(solve.authority[2], 0, 'f', 2)
                             .arg(solve.authorityHint[0], 0, 'f', 2)
                             .arg(solve.authorityHint[1], 0, 'f', 2)
                             .arg(solve.authorityHint[2], 0, 'f', 2);
            }
            parts << QStringLiteral("polar skin D %1/%2 N (%3 W)")
                         .arg(front_.polarDragTractionNewtons, 0, 'f', 0)
                         .arg(front_.polarDragTargetNewtons, 0, 'f', 0)
                         .arg(front_.polarDragTractionPowerWatts, 0, 'f', 0);
            if (controls_.freeFlight
                && controls_.launchMode == LaunchMode::TrimmedGlide) {
                parts << QStringLiteral(
                             "launch %1 km/h · q %2 Pa · CL %3 · H/V %4/%5 N · %6+%7")
                             .arg(front_.trimmedLaunchAirspeed * 3.6,
                                  0, 'f', 0)
                             .arg(front_.trimmedLaunchDynamicPressure,
                                  0, 'f', 0)
                             .arg(front_.trimmedLaunchEffectiveLiftCoefficient,
                                  0, 'f', 2)
                             .arg(front_.trimmedLaunchHorizontalResidualNewtons,
                                  0, 'f', 0)
                             .arg(front_.trimmedLaunchVerticalResidualNewtons,
                                  0, 'f', 0)
                             .arg(front_.trimmedLaunchCalibrationIterations)
                             .arg(front_.trimmedLaunchRelaxationFrames);
            }
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
        if (front_.grabActive) {
            parts << QStringLiteral("pull %1 m %2 N")
                         .arg(front_.grabPullMetres, 0, 'f', 2)
                         .arg(front_.grabForceNewtons, 0, 'f', 0);
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

    // Highest cable load in the wing right now, for the legend.
    double peakLineTension() const
    {
        float peak = 0.0F;
        for (const float tension : front_.lineTension) {
            peak = std::max(peak, tension);
        }
        return peak;
    }

    // Legend peaks over the cached fields — fresh only while their mode is
    // active, which is the only time the legend quotes them.
    double peakPositiveField() const
    {
        float peak = 0.0F;
        for (const float value : front_.colourField) {
            peak = std::max(peak, value);
        }
        return peak;
    }

    double peakDeviation() const
    {
        float peak = 0.0F;
        for (const float value : front_.colourField) {
            peak = std::max(peak, value);
        }
        return peak;
    }

    // Signed pressure fields need the value furthest from zero, preserving
    // its sign so the marker lands on the correct side of the legend.
    double extremeFieldValue() const
    {
        float extreme = 0.0F;
        for (const float value : front_.colourField) {
            if (std::abs(value) > std::abs(extreme)) {
                extreme = value;
            }
        }
        return extreme;
    }

    // Positive compression fraction; the field stores strain (negative
    // when compressed, 0 where taut).
    double peakSlackCompression() const
    {
        float worst = 0.0F;
        for (const float value : front_.colourField) {
            worst = std::min(worst, value);
        }
        return -worst;
    }

    // What the legend must show right now: the active face mode's bar
    // and, when on, the line-tension bar. Empty in Plain mode with the
    // tension colouring off — no colours, no legend.
    // The mode the DELIVERED field belongs to. During a mode switch the
    // requested mode runs ahead of the worker's recomputation by up to
    // a solver frame, and interpreting the old field under the new ramp
    // showed nonsense colours; painting keeps the old calibration until
    // the matching field lands.
    ColorMode displayMode() const
    {
        if (colorMode_ == ColorMode::Plain || front_.colourField.empty()) {
            return ColorMode::Plain;
        }
        return static_cast<ColorMode>(front_.fieldMode);
    }

    std::vector<LegendBar> legendBars() const
    {
        const auto percent = [](double value) {
            return QStringLiteral("%1%").arg(value * 100.0, 0, 'f',
                                             value * 100.0 < 1.0 ? 2 : 1);
        };
        std::vector<LegendBar> bars;
        switch (displayMode()) {
        case ColorMode::Stress:
            bars.push_back({QStringLiteral("edge stretch"),
                            0.0,
                            std::max(stressFullScale_, 1.0e-6),
                            peakPositiveField(), percent});
            break;
        case ColorMode::Deviation:
            bars.push_back(
                {QStringLiteral("deviation"), 0.0,
                 deviationFullScaleMetres(),
                 peakDeviation(), [](double value) {
                     return QStringLiteral("%1 mm").arg(value * 1000.0, 0,
                                                        'f', 0);
                 }});
            break;
        case ColorMode::Slack:
            bars.push_back({QStringLiteral("compression"),
                            0.0,
                            std::max(stressFullScale_, 1.0e-6),
                            peakSlackCompression(), percent});
            break;
        case ColorMode::CellInteriorPressure:
        {
            const double scale = pressureFullScalePascal();
            bars.push_back({QStringLiteral("resolved cell p"),
                            -scale, scale, extremeFieldValue(),
                            [](double value) {
                                return QStringLiteral("%1 Pa").arg(
                                    value, 0, 'f', 0);
                            }, true});
            break;
        }
        case ColorMode::ExteriorPressureCoefficient:
            bars.push_back(
                {QStringLiteral("external Cp"),
                 lep::playground::minimumExteriorPressureCoefficient,
                 lep::playground::maximumExteriorPressureCoefficient,
                 extremeFieldValue(), [](double value) {
                     return QString::number(value, 'f', 2);
                 }, true});
            break;
        case ColorMode::FabricPressureDifference: {
            const double scale = pressureFullScalePascal();
            bars.push_back({QStringLiteral("fabric Δp (in−out)"),
                            -scale, scale, extremeFieldValue(),
                            [](double value) {
                                return QStringLiteral("%1 Pa").arg(
                                    value, 0, 'f', 0);
                            }, true});
            break;
        }
        case ColorMode::Plain:
            break;
        }
        if (lineTensionColoring_) {
            bars.push_back({QStringLiteral("line tension"),
                            0.0,
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
        // Intent is stored ungated: unpausing a worker with no body is
        // harmless (its idle predicate keeps it waiting), and gating on
        // topology silently dropped a Run pressed during the first
        // build.
        runningRequested_ = running;
        if (worker_ != nullptr) {
            worker_->setPaused(!running);
        }
    }

    bool isRunning() const
    {
        return runningRequested_ && hasBody()
               && front_.simError.isEmpty();
    }
    bool hasBody() const { return !topo_.renderFaces.empty(); }
    QString lastSimError() const { return front_.simError; }
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
        if (topo_.renderFaces.empty() || front_.positions.empty()
            || !glError_.isEmpty()) {
            return;
        }

        const QMatrix4x4 mvp = viewProjection();

        const std::vector<QVector3D> &nodes = front_.positions;
        const ColorMode paintMode = displayMode();

        vertexScratch_.clear();
        vertexScratch_.reserve(topo_.renderFaces.size() * 27);
        for (std::size_t faceIndex = 0;
             faceIndex < topo_.renderFaces.size(); ++faceIndex) {
            const RenderFace &face = topo_.renderFaces[faceIndex];
            if (!surfaceVisible_[static_cast<std::size_t>(face.surface)]) {
                continue;
            }
            // During a rebuild the topology and the latest snapshot can
            // be one hand-off apart; skip anything the positions cannot
            // back rather than read past them.
            if (face.nodes[0] >= nodes.size()
                || face.nodes[1] >= nodes.size()
                || face.nodes[2] >= nodes.size()) {
                continue;
            }
            const QVector3D &a = nodes[face.nodes[0]];
            const QVector3D &b = nodes[face.nodes[1]];
            const QVector3D &c = nodes[face.nodes[2]];
            const QVector3D normal =
                QVector3D::crossProduct(b - a, c - a).normalized();
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
            const bool pressureFaceMode =
                paintMode == ColorMode::CellInteriorPressure
                || paintMode == ColorMode::ExteriorPressureCoefficient
                || paintMode == ColorMode::FabricPressureDifference;
            const bool faceColoured =
                paintMode == ColorMode::Deviation
                || (pressureFaceMode
                        ? faceIndex < topo_.skinTriangleCount
                        : paintMode != ColorMode::Plain
                              && colourable(face));
            // Pressure fields are per FACE: every corner wears the same tint.
            // Internal pressure therefore stays exactly flat within a cell;
            // exterior Cp and fabric Δp expose their own discretisation.
            const float faceValue =
                pressureFaceMode && faceIndex < front_.colourField.size()
                    ? front_.colourField[faceIndex]
                    : 0.0F;
            for (int corner = 0; corner < 3; ++corner) {
                const std::size_t node = face.nodes[
                    static_cast<std::size_t>(corner)];
                QVector3D tint = uncolouredTint;
                const float fieldValue =
                    node < front_.colourField.size()
                        ? front_.colourField[node]
                        : 0.0F;
                if (faceColoured) {
                    switch (paintMode) {
                    case ColorMode::Stress:
                        tint = stressTint(fieldValue);
                        break;
                    case ColorMode::Slack:
                        tint = rampTint(
                            -fieldValue
                            / std::max(stressFullScale_, 1.0e-6));
                        break;
                    case ColorMode::Deviation:
                        tint = rampTint(fieldValue
                                        / deviationFullScaleMetres());
                        break;
                    case ColorMode::CellInteriorPressure:
                    {
                        const double scale = pressureFullScalePascal();
                        tint = signedPressureTint(faceValue, -scale, scale);
                        break;
                    }
                    case ColorMode::ExteriorPressureCoefficient:
                        tint = signedPressureTint(
                            faceValue,
                            lep::playground::minimumExteriorPressureCoefficient,
                            lep::playground::maximumExteriorPressureCoefficient);
                        break;
                    case ColorMode::FabricPressureDifference: {
                        const double scale = pressureFullScalePascal();
                        tint = signedPressureTint(faceValue, -scale, scale);
                        break;
                    }
                    case ColorMode::Plain:
                        break;
                    }
                }
                const QVector3D &point = nodes[node];
                vertexScratch_.push_back(point.x());
                vertexScratch_.push_back(point.y());
                vertexScratch_.push_back(point.z());
                vertexScratch_.push_back(normal.x());
                vertexScratch_.push_back(normal.y());
                vertexScratch_.push_back(normal.z());
                vertexScratch_.push_back(tint.x());
                vertexScratch_.push_back(tint.y());
                vertexScratch_.push_back(tint.z());
            }
        }
        const int skinFloats = static_cast<int>(vertexScratch_.size());
        if (linesVisible_) {
            for (std::size_t segmentIndex = 0;
                 segmentIndex < topo_.lineSegments.size();
                 ++segmentIndex) {
                const LineSegment &segment =
                    topo_.lineSegments[segmentIndex];
                if (segment.a >= nodes.size()
                    || segment.b >= nodes.size()) {
                    continue;
                }
                QVector3D tint;
                if (lineTensionColoring_) {
                    tint = rampTint(
                        (segmentIndex < front_.lineTension.size()
                             ? front_.lineTension[segmentIndex]
                             : 0.0F)
                        / std::max(lineFullScaleNewtons_, 1.0e-6));
                }
                for (const std::size_t node : {segment.a, segment.b}) {
                    const QVector3D &point = nodes[node];
                    vertexScratch_.push_back(point.x());
                    vertexScratch_.push_back(point.y());
                    vertexScratch_.push_back(point.z());
                    vertexScratch_.push_back(segment.brake ? 1.0F : 0.0F);
                    vertexScratch_.push_back(0.0F);
                    vertexScratch_.push_back(0.0F);
                    vertexScratch_.push_back(tint.x());
                    vertexScratch_.push_back(tint.y());
                    vertexScratch_.push_back(tint.z());
                }
            }
        }

        // The air itself, appended last so the wing and lines keep their
        // own vertex ranges. Centred on the pilot when there is one: he
        // is the thing that travels, and the canopy swings about him.
        const int sceneFloats = static_cast<int>(vertexScratch_.size());
        appendAirMotes(topo_.pilotNode != noConstraint
                               && topo_.pilotNode < nodes.size()
                           ? nodes[topo_.pilotNode]
                           : target_);

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
                                  paintMode != ColorMode::Plain);
        program_->setUniformValue(
            "color", QVector4D(0.72F, 0.78F, 0.88F, 1.0F));
        glDrawArrays(GL_TRIANGLES, 0, skinFloats / 9);

        program_->setUniformValue("lit", false);
        program_->setUniformValue("useTint", lineTensionColoring_);
        program_->setUniformValue(
            "color", QVector4D(0.55F, 0.62F, 0.55F, 1.0F));
        glDrawArrays(GL_LINES,
                     skinFloats / 9,
                     (sceneFloats - skinFloats) / 9);

        if (static_cast<int>(vertexScratch_.size()) > sceneFloats) {
            glPointSize(2.0F);
            program_->setUniformValue("lit", false);
            program_->setUniformValue("useTint", true);
            glDrawArrays(
                GL_POINTS,
                sceneFloats / 9,
                (static_cast<int>(vertexScratch_.size()) - sceneFloats) / 9);
        }
        buffer_.release();
        program_->release();

        // The calibrated legend, painted over the scene it calibrates.
        // This needs the stencil buffer requested in the constructor —
        // without one QPainter's GL engine silently drops filled paths.
        if (paintMode != ColorMode::Plain || lineTensionColoring_) {
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
    // viewport: quantity, unit, ticks and a live extreme marker. A value
    // outside the displayed range parks at the corresponding end with its
    // true value printed, so saturation reads as saturation.
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
            if (bar.diverging) {
                const double range = bar.maximum - bar.minimum;
                const double zero = range > 0.0
                                        ? std::clamp(-bar.minimum / range,
                                                     0.0, 1.0)
                                        : 0.5;
                ramp.setColorAt(
                    0.0, QColor::fromRgbF(kSignedNegativeTint.x(),
                                          kSignedNegativeTint.y(),
                                          kSignedNegativeTint.z()));
                ramp.setColorAt(
                    zero, QColor::fromRgbF(kSignedNeutralTint.x(),
                                           kSignedNeutralTint.y(),
                                           kSignedNeutralTint.z()));
                ramp.setColorAt(
                    1.0, QColor::fromRgbF(kSignedPositiveTint.x(),
                                          kSignedPositiveTint.y(),
                                          kSignedPositiveTint.z()));
            } else {
                for (std::size_t stop = 0; stop < kRampStops.size();
                     ++stop) {
                    const QVector3D &colour = kRampStops[stop];
                    ramp.setColorAt(
                        static_cast<double>(stop)
                            / (kRampStops.size() - 1),
                        QColor::fromRgbF(colour.x(), colour.y(),
                                         colour.z()));
                }
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
                        bar.format(bar.minimum
                                   + fraction
                                         * (bar.maximum - bar.minimum)));
                }
            }

            const double range = bar.maximum - bar.minimum;
            const double peakFraction =
                range > 0.0
                    ? std::clamp((bar.marker - bar.minimum) / range,
                                 0.0, 1.0)
                    : 0.0;
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
                QStringLiteral("◂ %1").arg(bar.format(bar.marker)));

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
            worker_->endGrab();
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
            worker_->moveGrab(grabWorld_);
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
        if (grabbing_) {
            // grabWorld_ is the GUI's own accumulated anchor target, so
            // no trip to the worker is needed.
            const QVector3D eye = viewMatrix().map(
                QVector3D(static_cast<float>(grabWorld_.x),
                          static_cast<float>(grabWorld_.y),
                          static_cast<float>(grabWorld_.z)));
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

    static QVector3D signedPressureTint(double value,
                                        double minimum,
                                        double maximum)
    {
        if (value <= 0.0) {
            const double fraction = minimum < 0.0
                                        ? std::clamp(
                                              (value - minimum) / -minimum,
                                              0.0, 1.0)
                                        : 1.0;
            return kSignedNegativeTint
                       * static_cast<float>(1.0 - fraction)
                   + kSignedNeutralTint * static_cast<float>(fraction);
        }
        const double fraction = maximum > 0.0
                                    ? std::clamp(value / maximum, 0.0, 1.0)
                                    : 1.0;
        return kSignedNeutralTint * static_cast<float>(1.0 - fraction)
               + kSignedPositiveTint * static_cast<float>(fraction);
    }

    QVector3D stressTint(double strain) const
    {
        return rampTint(strain / std::max(stressFullScale_, 1.0e-6));
    }

    // One scale slider serves five adjustable ramps. Its integer value is
    // hundredths of a percent for the strain modes (10..500 -> 0.1%..5%
    // strain), millimetres for deviation (10..500 mm), and pascals for the
    // signed internal-pressure/fabric-Δp ranges (10..500 Pa). Exterior Cp
    // keeps its fixed physical -3..1 range. stressFullScale_ stores
    // value/10000.
    double deviationFullScaleMetres() const
    {
        return std::max(stressFullScale_ * 10.0, 1.0e-4);
    }
    double pressureFullScalePascal() const
    {
        return std::max(stressFullScale_ * 1.0e4, 1.0);
    }

    // One deterministic pseudo-random number per lattice cell and channel.
    // Integer coordinates in, a repeatable fraction out: no state, no
    // sequence, no seed to carry — which is what lets the mote field be a
    // pure function of where you are rather than a thing that has to be
    // spawned, stored and aged.
    static float moteHash(int x, int y, int z, int channel)
    {
        std::uint32_t value =
            static_cast<std::uint32_t>(x) * 0x8DA6B343u
            ^ static_cast<std::uint32_t>(y) * 0xD8163841u
            ^ static_cast<std::uint32_t>(z) * 0xCB1AB31Fu
            ^ static_cast<std::uint32_t>(channel) * 0x165667B1u;
        value ^= value >> 15;
        value *= 0x2C1B3C6Du;
        value ^= value >> 12;
        value *= 0x297A2D39u;
        value ^= value >> 15;
        return static_cast<float>(value & 0xFFFFFFu)
               / static_cast<float>(0x1000000u);
    }

    // Motes of air hanging in the world, so the wing's travel through it
    // is visible. The field is an infinite lattice with one mote per cell,
    // offset inside its cell by the hash of the cell's own coordinates: a
    // mote therefore sits at exactly the same world point every frame no
    // matter how the wing moves, which is the whole trick — the eye reads
    // the wing sliding past fixed things as speed, and a field that was
    // respawned or jittered per frame would read as noise instead. Only
    // the cells near the pilot are visited, so an unbounded field costs a
    // fixed amount.
    void appendAirMotes(const QVector3D &centre)
    {
        if (airSpacingMetres_ <= 0.0F) {
            return;
        }
        const float spacing = airSpacingMetres_;
        const float radius = kAirMoteRadiusMetres;
        const int reach = std::min(
            24, static_cast<int>(std::ceil(radius / spacing)));
        // The lattice is anchored to the AIR, not to the world: both
        // flight modes hold the wing at the origin (the tunnel by
        // tethering it, free flight by re-centring the system every
        // frame), so a world-anchored field is a field that never moves.
        // Motes are laid out around where the wing sits in AIR
        // coordinates and drawn shifted back by the same travel, which is
        // what makes them stream past at the speed the wing is flying.
        const QVector3D travel = front_.airTravel;
        const QVector3D anchor = centre - travel;
        const auto cellOf = [spacing](float value) {
            return static_cast<int>(std::floor(value / spacing));
        };
        const int baseX = cellOf(anchor.x());
        const int baseY = cellOf(anchor.y());
        const int baseZ = cellOf(anchor.z());
        const QVector3D background(0.10F, 0.11F, 0.13F);
        const QVector3D lit(0.62F, 0.70F, 0.82F);
        for (int i = -reach; i <= reach; ++i) {
            for (int j = -reach; j <= reach; ++j) {
                for (int k = -reach; k <= reach; ++k) {
                    const int x = baseX + i;
                    const int y = baseY + j;
                    const int z = baseZ + k;
                    // Laid out in AIR coordinates, then carried back into
                    // the wing's frame by the travel. Both halves matter:
                    // the lattice is what makes a mote hold still in the
                    // air, and the shift is what makes it stream past the
                    // wing.
                    const QVector3D mote =
                        QVector3D(
                            (static_cast<float>(x) + moteHash(x, y, z, 0))
                                * spacing,
                            (static_cast<float>(y) + moteHash(x, y, z, 1))
                                * spacing,
                            (static_cast<float>(z) + moteHash(x, y, z, 2))
                                * spacing)
                        + travel;
                    const float distance = (mote - centre).length();
                    if (distance >= radius) {
                        continue;
                    }
                    // Faded into the background toward the edge of the
                    // sphere, so motes arrive and leave instead of
                    // popping at a hard boundary.
                    const float near = 1.0F - distance / radius;
                    const QVector3D tint =
                        background + (lit - background) * (near * near);
                    vertexScratch_.push_back(mote.x());
                    vertexScratch_.push_back(mote.y());
                    vertexScratch_.push_back(mote.z());
                    vertexScratch_.push_back(0.0F);
                    vertexScratch_.push_back(0.0F);
                    vertexScratch_.push_back(1.0F);
                    vertexScratch_.push_back(tint.x());
                    vertexScratch_.push_back(tint.y());
                    vertexScratch_.push_back(tint.z());
                }
            }
        }
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
    // Ctrl-click picking: every unique line-junction endpoint projected to
    // widget pixels, nearest within grabPickRadiusPixels wins. Carabiners
    // are fixed nodes, so grabbing one merely parks the anchor on it — not
    // worth excluding.
    void tryBeginGrab(const QPointF &cursor)
    {
        if (front_.positions.empty()) {
            return;
        }
        const QMatrix4x4 mvp = viewProjection();
        const std::vector<QVector3D> &nodes = front_.positions;
        const std::vector<std::size_t> &junctions = topo_.junctions;
        double bestDistance = grabPickRadiusPixels;
        std::size_t bestNode = noConstraint;
        for (const std::size_t node : junctions) {
            if (node >= nodes.size()) {
                continue;
            }
            const QVector3D &position = nodes[node];
            const QVector4D clip = mvp * QVector4D(position, 1.0F);
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
        const QVector3D picked = nodes[bestNode];
        worker_->beginGrab(bestNode);
        grabWorld_ = softwing::Vec3{picked.x(), picked.y(), picked.z()};
        const QVector3D eye = viewMatrix().map(picked);
        // Camera looks down -z; the depth scales the cursor's
        // metres-per-pixel during the drag.
        grabDepth_ = std::max(-eye.z(), 0.05F);
        grabbing_ = true;
        setCursor(Qt::ClosedHandCursor);
    }

    // The worker owns the simulation; the GUI owns only what it needs
    // to draw and command.
    std::unique_ptr<SimWorker> worker_;
    SimWorker::Snapshot front_;
    SimWorker::Topology topo_;
    bool runningRequested_ = false;
    std::function<void(const QString &)> topologyArrived_;
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
    // The interactive grab: anchor position accumulated in doubles so a
    // long drag does not drift, depth fixed at pick time.
    bool grabbing_ = false;
    float grabDepth_ = 1.0F;
    softwing::Vec3 grabWorld_;
    std::vector<float> vertexScratch_;
    QString glError_;

    QTimer *timer_ = nullptr;
    QOpenGLShaderProgram *program_ = nullptr;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer buffer_{QOpenGLBuffer::VertexBuffer};
    QVector3D target_;
    // Where the flying system was last frame, so the orbit target can be
    // carried along with it (see followSystem).
    QVector3D followAnchor_;
    bool haveFollowAnchor_ = false;
    // Lattice spacing of the air motes, metres; 0 turns the field off.
    float airSpacingMetres_ = 0.0F;
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
    // Default 61 Pa = 36 km/h (q = ½ρv² at 10 m/s), a typical trim speed.
    pressure_ = makeSlider(100, 61);
    pressure_->setToolTip(QStringLiteral(
        "Reference dynamic pressure and airspeed. In the tunnel this is "
        "the prescribed airflow. In free flight it calibrates the load "
        "cap and trimmed launch speed; the atmosphere itself is stationary "
        "unless an ambient-air velocity is supplied programmatically."));
    lift_ = makeSlider(15, 6);
    leftBrake_ = makeSlider(100, 0);
    rightBrake_ = makeSlider(100, 0);
    // 1 is the cross-port area the design declares; the rest of the range
    // is a deliberate lie, so the one path that re-feeds a sealed cell can
    // be turned up until its effect is visible.
    crossPortGain_ = new QSlider(Qt::Horizontal, this);
    crossPortGain_->setRange(1, 10);
    crossPortGain_->setValue(1);
    crossPortGain_->setToolTip(QStringLiteral(
        "Multiplies the flow through the rib cross-port holes — the only "
        "way air reaches a cell whose own intake has folded shut. 1 is "
        "the hole area the design actually has; above that the model is "
        "knowingly generous."));
    // Motes of air at fixed points in the world. They exist only to make
    // the wing's own travel legible: gliding, sinking and surging all
    // look alike against an empty background.
    // On at full by default: gliding, sinking and surging all look alike
    // against an empty background, and the motes are the only thing in
    // the view that shows the wing is going anywhere at all.
    airDensity_ = makeSlider(100, 100);
    airDensity_->setToolTip(QStringLiteral(
        "Draws motes of air at fixed points in space. They do not touch "
        "the physics — they are there so the wing's movement through the "
        "air can be seen, which an empty background hides."));
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
    colorBy_->addItem(QStringLiteral("Cell resolved p"));
    colorBy_->addItem(QStringLiteral("External Cp"));
    colorBy_->addItem(QStringLiteral("Fabric Δp"));
    colorBy_->setToolTip(QStringLiteral(
        "Colour the skin by edge stretch (Stress), by distance from the "
        "designed shape (Deviation), by compressed — wrinkled — fabric "
        "(Slack), or by one of three distinct pressure quantities: "
        "Cell resolved p is the uniform gauge pressure applied inside each "
        "cell. It follows the calibrated ram field in healthy flight, then "
        "the finite-mass/live-volume gas state takes over as the cell loses "
        "volume, mouth opening or ram recovery. External Cp is the signed "
        "outside aerodynamic "
        "pressure coefficient; Fabric Δp is the actual signed load across "
        "each triangle, p_inside − q·Cp. Blue/red on signed modes means "
        "negative/positive."));
    // Full-scale for the ramp. Read as hundredths of a percent strain for
    // Stress and Slack (so the low end, where fabric actually works, still
    // has resolution), as millimetres for Deviation, and as pascals for the
    // two adjustable pressure modes. External Cp uses fixed physical bounds.
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
        "closer to its designed length; fewer keep the wing interactive. "
        "Free flight also adds reverse/forward suspension load-path passes "
        "so payload load reaches the canopy without extra cloth iterations."));
    skinModel_ = new QComboBox(this);
    skinModel_->addItem(QStringLiteral("Legacy distance truss"));
    skinModel_->addItem(QStringLiteral("Orthotropic membrane (prototype)"));
    skinModel_->setToolTip(QStringLiteral(
        "Legacy preserves the calibrated bilateral distance-spring skin. "
        "The prototype replaces only skin springs with warp/weft/shear "
        "membranes, compression softening and true fold hinges; ribs, seams, "
        "straps, lines, pressure, cells and contact are unchanged. It is an "
        "engineering experiment, not a calibrated fabric certificate."));
    skinMaterialLabel_ = new QLabel(this);
    skinMaterialLabel_->setWordWrap(true);
    const auto refreshSkinMaterial = [this] {
        skinMaterialLabel_->setText(
            skinModel_->currentIndex() == 0
                ? QStringLiteral("Calibrated legacy structural guard")
                : QStringLiteral(
                      "Prototype W/T/C/S 8000/5000/1000/1500 N/m · "
                      "compression 0.05 · damping off · bend C 5e-4"));
    };
    refreshSkinMaterial();
    connect(skinModel_, &QComboBox::currentIndexChanged, this,
            [refreshSkinMaterial](int) { refreshSkinMaterial(); });

    // ---- The left panel. The wing on screen is roughly 1:1 while the
    // window is closer to 2:1, so chrome above the viewport is exactly
    // the wrong place for it: everything lives in a column on the left
    // and the viewport gets the full height. ----

    freeFlight_ = makeCheck(QStringLiteral("Free flight"), false);
    freeFlight_->setToolTip(QStringLiteral(
        "Unpin the wing: gravity on, a pilot slung under the risers, the "
        "whole system flying and re-centred each frame. Steer with the "
        "brakes; a little symmetric brake steadies it."));
    pilotMass_ = makeSlider(
        static_cast<int>(maximumPilotMassKg),
        static_cast<int>(lep::playground::defaultPilotMassKg));
    pilotMass_->setMinimum(static_cast<int>(minimumPilotMassKg));
    pilotMass_->setToolTip(QStringLiteral(
        "Pilot plus harness, instruments and carried equipment. This is an "
        "explicit free-flight mass; the simulator no longer chooses a mass "
        "that happens to balance its own aerodynamic polar."));
    pilotMassLabel_ = new QLabel(this);
    launchMode_ = new QComboBox(this);
    launchMode_->addItem(QStringLiteral("Trimmed glide"));
    launchMode_->addItem(QStringLiteral("Drop from rest"));
    launchMode_->setToolTip(QStringLiteral(
        "Trimmed glide starts every component on the estimated flight path. "
        "Drop from rest releases the pre-inflated point-payload system with "
        "zero initial velocity so its gravity transient is visible."));
    fabricContact_ = makeCheck(QStringLiteral("Fabric contact"), false);
    fabricContact_->setToolTip(QStringLiteral(
        "Experimental skin vertex/triangle, edge/edge and authored-line "
        "contact. Prevents ghosting and reports incomplete broad-phase "
        "coverage, but is currently expensive on a full wing. Friction "
        "and line/line contact are not modelled. Takes effect on the next "
        "frame — no rebuild."));
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
    crossPortLabel_ = new QLabel(this);
    airLabel_ = new QLabel(this);
    alphaDial_ = new AngleOfAttackDial(this);
    alphaDial_->setSetAngleProvider(
        [this] { return static_cast<double>(lift_->value()); });

    // Initial readouts before any slider moves.
    const auto refreshControlReadouts = [this] {
        const double pascal = static_cast<double>(pressure_->value());
        pressureLabel_->setText(
            QStringLiteral("Reference q %1 Pa · airspeed %2 km/h")
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
        pilotMassLabel_->setText(
            QStringLiteral("Pilot + harness %1 kg")
                .arg(pilotMass_->value()));
        crossPortLabel_->setText(
            crossPortGain_->value() == 1
                ? QStringLiteral("Neighbour reinflation ×1 (as designed)")
                : QStringLiteral("Neighbour reinflation ×%1")
                      .arg(crossPortGain_->value()));
        airLabel_->setText(
            airDensity_->value() == 0
                ? QStringLiteral("Air motes off")
                : QStringLiteral("Air motes · %1 m apart")
                      .arg(airSpacingFor(airDensity_->value()), 0, 'f', 1));
    };
    refreshControlReadouts();
    for (QSlider *slider :
         {pressure_, lift_, leftBrake_, rightBrake_, pilotMass_, crossPortGain_,
          airDensity_}) {
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
    panelLayout->addWidget(skinModel_);
    panelLayout->addWidget(skinMaterialLabel_);
    panelLayout->addWidget(freeFlight_);
    panelLayout->addWidget(pilotMassLabel_);
    panelLayout->addWidget(pilotMass_);
    panelLayout->addWidget(launchMode_);
    panelLayout->addWidget(fabricContact_);
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
    panelLayout->addWidget(crossPortLabel_);
    panelLayout->addWidget(crossPortGain_);
    panelLayout->addWidget(airLabel_);
    panelLayout->addWidget(airDensity_);
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
    connect(fabricContact_, &QCheckBox::toggled, this,
            [this](bool enabled) {
                if (view_ != nullptr) {
                    view_->setFabricContact(enabled);
                }
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
    connect(skinModel_, &QComboBox::currentIndexChanged,
            this, [this](int index) {
                if (view_ != nullptr) {
                    view_->setSkinModel(
                        index == 1 ? SkinModel::OrthotropicMembrane
                                   : SkinModel::LegacyDistanceTruss);
                }
            });
    connect(pilotMass_, &QSlider::valueChanged, this, [this](int kilograms) {
        if (view_ != nullptr) {
            view_->setPilotMassKg(static_cast<double>(kilograms));
        }
    });
    connect(launchMode_, &QComboBox::currentIndexChanged,
            this, [this](int index) {
                if (view_ != nullptr) {
                    view_->setLaunchMode(
                        index == 1 ? LaunchMode::DropFromRest
                                   : LaunchMode::TrimmedGlide);
                }
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
    settleTimer_->setInterval(500);
    connect(settleTimer_, &QTimer::timeout, this, [this] {
        if (!settleRunning_ || view_ == nullptr) {
            settleTimer_->stop();
            return;
        }
        if (view_->settleDone() || !view_->lastSimError().isEmpty()) {
            finishSettle(false);
            return;
        }
        status_->setText(
            QStringLiteral("Settling at 60×4… %1 of max %2 s "
                           "simulated · agitation %3 mm/s (quiet "
                           "below %4)")
                .arg(view_->settleSimSeconds(), 0, 'f', 1)
                .arg(kSettleBudgetSeconds, 0, 'f', 0)
                .arg(view_->settleAgitation() * 1000.0, 0, 'f', 0)
                .arg(lep::playground::settleQuiescenceTarget(
                         view_->controls().pressurePascal)
                         * 1000.0,
                     0, 'f', 0));
        // The shape HUD keeps measuring during the show — the data is
        // the point of watching.
        const QString readout = view_->shapeReadout();
        shapeLabel_->setText(readout);
        shapeLabel_->setToolTip(readout);
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
                // External Cp has a fixed physical -3..1 scale; all other
                // non-plain fields use this slider in their own units.
                stressScale_->setVisible(index != 0 && index != 5);
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
    connect(crossPortGain_, &QSlider::valueChanged, this,
            [this, notePausedControls](int value) {
                if (view_ != nullptr) {
                    view_->setCrossPortGain(static_cast<double>(value));
                }
                notePausedControls();
            });
    connect(airDensity_, &QSlider::valueChanged, this, [this](int value) {
        if (view_ != nullptr) {
            view_->setAirSpacing(value == 0 ? 0.0 : airSpacingFor(value));
        }
    });
    connect(runButton_, &QPushButton::toggled, this, [this](bool paused) {
        if (view_ != nullptr) {
            view_->setRunning(!paused);
        }
        runButton_->setText(paused ? QStringLiteral("Run")
                                   : QStringLiteral("Pause"));
        updateShapeTimer();
    });
    connect(resetButton_, &QPushButton::clicked, this, [this] {
        // Hands up first. The brake input outlives the body it was
        // pulling: fly mode reads the pull off the cursor, and the cursor
        // has to leave the view to reach this button, so the last pull
        // stays in the controls (and mirrored on the sliders). A wing
        // rebuilt at its rest pose with 60 cm of brake still on folds
        // again within a few frames, which reads — correctly — as "Reset
        // did not reset".
        leftBrake_->setValue(0);
        rightBrake_->setValue(0);
        if (view_ != nullptr) {
            // Directly too: the sliders only push when their integer
            // value actually changes, and a pull under half a percent of
            // travel rounds to a slider that was already at zero.
            view_->setBrakePull(0.0, 0.0);
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
    // Runs on the GUI thread from the view's poll: the async build's
    // outcome — the status line stays at "Building…" until here, and a
    // failed build says so instead of leaving a silent blank viewport.
    view_->setTopologyCallback([this](const QString &error) {
        status_->setText(
            error.isEmpty()
                ? pendingBuildStatus_ + QStringLiteral(" · ")
                      + view_->materialReadout()
                : QStringLiteral("Wind tunnel build failed: %1")
                      .arg(error));
        updateShapeTimer();
    });
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
    // The mote field is off in a fresh view and only ever heard about
    // the slider through valueChanged, so before this the slider's
    // starting position was silently ignored — motes stayed off until
    // the slider was touched, whatever it read.
    view_->setAirSpacing(airDensity_->value() == 0
                             ? 0.0
                             : airSpacingFor(airDensity_->value()));
    view_->setColorMode(static_cast<PlaygroundView::ColorMode>(
        colorBy_->currentIndex()));
    view_->setLineFullScale(static_cast<double>(lineScale_->value()));
    view_->setLineTensionColoring(showLineTension_->isChecked());
    view_->setFlightLoad(flightLoad_->isChecked());
    view_->setFabricContact(fabricContact_->isChecked());
    view_->setPilotMassKg(static_cast<double>(pilotMass_->value()));
    view_->setLaunchMode(
        launchMode_->currentIndex() == 1 ? LaunchMode::DropFromRest
                                         : LaunchMode::TrimmedGlide);
    view_->setSkinModel(
        skinModel_->currentIndex() == 1
            ? SkinModel::OrthotropicMembrane
            : SkinModel::LegacyDistanceTruss);
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
    if (view_ == nullptr || sweepActive_) {
        return;
    }
    if (!view_->hasBody()) {
        status_->setText(QStringLiteral(
            "Still building the wing — Settle will be available in a "
            "moment."));
        return;
    }
    settleRunning_ = true;
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
    view_->startSettle(kSettleBudgetSeconds);
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
    const bool converged =
        view_ != nullptr && view_->settleConverged();
    const double simSeconds =
        view_ != nullptr ? view_->settleSimSeconds() : 0.0;
    if (cancelled) {
        outcome = QStringLiteral("Settle cancelled.");
    } else if (!solverError.isEmpty()) {
        outcome = QStringLiteral("Settle failed: %1").arg(solverError);
    } else if (view_ != nullptr) {
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
            converged
                ? QStringLiteral("Done: settled at 60×4 in %1 s "
                                 "simulated")
                      .arg(simSeconds, 0, 'f', 1)
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
    if (view_ != nullptr) {
        // Acknowledge so the worker clears its latched settle state.
        view_->acknowledgeSettle();
        view_->setRunning(false);
    }
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
            QTimer::singleShot(6000, this, [this] {
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
    // A rebuild cancels a running settle on the worker; without closing
    // it here too, the page's settle UI polled for a completion that
    // could never come.
    if (settleRunning_) {
        finishSettle(true);
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
    // The build is asynchronous now; the success line waits for the
    // topology callback, and until then the truth is "building".
    pendingBuildStatus_ =
        QStringLiteral("Wind tunnel · %1 nodes, %2 skin quads, %3 line "
                       "segments%4 · drag to orbit, right-drag to pan, "
                       "wheel to zoom, Ctrl-click a line junction to pull "
                       "it. Relative shape signal, not absolute "
                       "aerodynamics.")
            .arg(simulated.nodes.size())
            .arg(simulated.quads.size())
            .arg(simulated.lines.size())
            .arg(resolution);
    status_->setText(QStringLiteral("Building the wind tunnel…"));
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
