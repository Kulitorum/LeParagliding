// Headless timing harness for the Playground's XPBD solve, and the
// headless face of its wind-tunnel instruments.
//
// Builds the same wing the Playground tab builds — same mesh, same
// refinement, same constraints, same step settings — and runs it without a
// window, so the solver can be measured and optimised without a human
// driving the GUI. Everything it reports comes either from wall clock around
// SoftBody::step or from the core's own StepPerformanceProfile. The shape
// modes go through the same settleAndMeasure() the GUI sweep uses, so any
// number in a GUI report can be reproduced here.
//
//   softwing-bench <lep-sim.json> [--subdiv N] [--detailed-ribs]
//                  [--frames N] [--warmup N] [--threads N]
//                  [--substeps N] [--iterations N] [--line-sweeps N] [--csv]
//                  [--glide N] [--brake CM] [--release SECONDS]
//                  [--pilot-mass KG] [--drop-from-rest]
//                  [--membrane] [--warp-stiffness N/M] [material flags]
//                  [--shape [SECONDS]] [--shape-sweep FROM:TO:STEP]
//                  [--tuck [PULL_CM]] [--dive [DEGREES]] [--no-cells]
//                  [--contact] [--no-flight-load] [--legacy-pressure]
//                  [--pressure-acceptance]

#include "../src/gui/playground_metrics.h"
#include "../src/gui/playground_sim.h"
#include "softwing_gpu.h"

#include <QByteArray>
#include <QFile>
#include <QGuiApplication>
#include <QString>
#include <QStringList>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <string>
#include <vector>

// Laptops with switchable graphics hand an OpenGL context to the integrated
// GPU unless the executable says otherwise, and on this machine that is the
// difference between a Radeon iGPU and an RTX 3070. Both vendors read these
// exported symbols out of the .exe at process start; they must live in the
// executable itself, not in a library it links.
#ifdef _WIN32
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace {

namespace pg = lep::playground;

struct Options
{
    std::string meshPath;
    int subdivision = 1;
    bool detailedRibs = false;
    int frames = 60;
    int warmup = 5;
    unsigned threads = 0;
    int substeps = pg::simulationSubsteps;
    int iterations = pg::simulationIterations;
    int lineSweepPairs = pg::defaultFreeFlightCableSweepPairs;
    double pressurePascal = 80.0;
    double angleOfAttackDegrees = 6.0;
    double pilotMassKg = pg::defaultPilotMassKg;
    pg::LaunchMode launchMode = pg::LaunchMode::TrimmedGlide;
    pg::SkinModel skinModel = pg::SkinModel::LegacyDistanceTruss;
    double warpStiffness = pg::prototypeWarpStiffness;
    double weftStiffness = pg::prototypeWeftStiffness;
    double couplingStiffness = pg::prototypeCouplingStiffness;
    double shearStiffness = pg::prototypeShearStiffness;
    double membraneDampingSeconds = pg::prototypeMembraneDampingSeconds;
    double compressionRatio = pg::prototypeCompressionStiffnessRatio;
    double bendCompliance = pg::prototypeBendCompliance;
    bool swing = false;
    bool polar = false;
    double brakeMetres = 0.0;
    // Per-side pulls, in the SOLVER's sense (left = negative mesh x).
    // Used instead of brakeMetres when either was given.
    double brakeLeftMetres = 0.0;
    double brakeRightMetres = 0.0;
    bool asymmetricBrake = false;
    // Frames the pull is eased on over, 0 for a step.
    int rampFrames = 0;
    // Frame the brakes come back up on, 0 for "never let go".
    int releaseFrames = 0;
    int glideFrames = 0;
    bool freeFlight = false;
    bool shape = false;
    // Settle budget for --shape, and for each sweep point when --shape is
    // given alongside --shape-sweep.
    double shapeSeconds = 6.0;
    bool noFlightLoad = false;
    // The per-cell air model, on by default like the GUI; --no-cells is
    // the A/B switch for comparing against the old blanket-ram stamp.
    bool noCells = false;
    // Fabric/line self-contact, off by default like the GUI.
    bool contact = false;
    // Explicit regression oracle for pre-P5 guards. Production/default is
    // the bounded final-exterior-Cp solve.
    bool legacyPressure = false;
    bool pressureAcceptance = false;
    // The collapse-recovery experiment: settle, yank one side's A cascade
    // down until it folds, release, and watch whether the wing recovers.
    bool tuck = false;
    double tuckPullMetres = 1.2;
    // The aerodynamic collapse-recovery experiment: settle at trim, slam
    // the airflow to a front-tuck angle for a few seconds, return to
    // trim, and watch whether the wing re-inflates.
    bool dive = false;
    double diveDegrees = -6.0;
    bool shapeSweep = false;
    double sweepFromDegrees = 0.0;
    double sweepToDegrees = 0.0;
    double sweepStepDegrees = 0.0;
    bool csv = false;
    bool gpu = false;
    pg::GpuSolveMode gpuMode = pg::GpuSolveMode::ColouredGaussSeidel;
};

[[nodiscard]] bool parseOptions(int argc, char **argv, Options &options)
{
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto value = [&](int &out) {
            if (index + 1 >= argc) return false;
            out = std::atoi(argv[++index]);
            return true;
        };
        const auto realValue = [&](double &out) {
            if (index + 1 >= argc) return false;
            char *end = nullptr;
            out = std::strtod(argv[++index], &end);
            return end != argv[index] && *end == '\0' && std::isfinite(out);
        };
        if (argument == "--subdiv") {
            if (!value(options.subdivision)) return false;
        } else if (argument == "--frames") {
            if (!value(options.frames)) return false;
        } else if (argument == "--warmup") {
            if (!value(options.warmup)) return false;
        } else if (argument == "--substeps") {
            if (!value(options.substeps)) return false;
        } else if (argument == "--iterations") {
            if (!value(options.iterations)) return false;
        } else if (argument == "--line-sweeps") {
            if (!value(options.lineSweepPairs)
                || options.lineSweepPairs < 0) return false;
        } else if (argument == "--threads") {
            int threads = 0;
            if (!value(threads)) return false;
            options.threads = static_cast<unsigned>(threads < 0 ? 0 : threads);
        } else if (argument == "--pressure") {
            int pascals = 0;
            if (!value(pascals)) return false;
            options.pressurePascal = pascals;
        } else if (argument == "--aoa") {
            int degrees = 0;
            if (!value(degrees)) return false;
            options.angleOfAttackDegrees = degrees;
        } else if (argument == "--pilot-mass") {
            if (index + 1 >= argc) return false;
            char *end = nullptr;
            options.pilotMassKg = std::strtod(argv[++index], &end);
            if (end == argv[index] || *end != '\0'
                || !std::isfinite(options.pilotMassKg)) {
                return false;
            }
        } else if (argument == "--drop-from-rest") {
            options.launchMode = pg::LaunchMode::DropFromRest;
            options.freeFlight = true;
        } else if (argument == "--membrane") {
            options.skinModel = pg::SkinModel::OrthotropicMembrane;
        } else if (argument == "--warp-stiffness") {
            if (!realValue(options.warpStiffness)) return false;
        } else if (argument == "--weft-stiffness") {
            if (!realValue(options.weftStiffness)) return false;
        } else if (argument == "--coupling-stiffness") {
            if (!realValue(options.couplingStiffness)) return false;
        } else if (argument == "--shear-stiffness") {
            if (!realValue(options.shearStiffness)) return false;
        } else if (argument == "--membrane-damping") {
            if (!realValue(options.membraneDampingSeconds)) return false;
        } else if (argument == "--compression-ratio") {
            if (!realValue(options.compressionRatio)) return false;
        } else if (argument == "--bend-compliance") {
            if (!realValue(options.bendCompliance)) return false;
        } else if (argument == "--swing") {
            options.swing = true;
            options.freeFlight = true;
        } else if (argument == "--polar") {
            options.polar = true;
        } else if (argument == "--brake") {
            int centimetres = 0;
            if (!value(centimetres)) return false;
            options.brakeMetres = centimetres / 100.0;
        } else if (argument == "--brake-left"
                   || argument == "--brake-right") {
            // One brake only: an asymmetric pull is a different animal
            // from a symmetric one, and it is the one that turns a wing.
            int centimetres = 0;
            if (!value(centimetres)) return false;
            (argument == "--brake-left" ? options.brakeLeftMetres
                                        : options.brakeRightMetres) =
                centimetres / 100.0;
            options.asymmetricBrake = true;
        } else if (argument == "--release") {
            int seconds = 0;
            if (!value(seconds)) return false;
            options.releaseFrames = seconds * 60;
        } else if (argument == "--ramp") {
            // A hand does not step a brake on. Ramping matters: a step to
            // 30 cm tumbles a wing that survives the same pull applied
            // over a few seconds, so a step cannot reproduce what a pilot
            // reports feeling.
            int seconds = 0;
            if (!value(seconds)) return false;
            options.rampFrames = seconds * 60;
        } else if (argument == "--glide") {
            options.glideFrames = 1800;
            if (index + 1 < argc && argv[index + 1][0] != '-') {
                options.glideFrames = std::atoi(argv[++index]);
            }
            options.freeFlight = true;
        } else if (argument == "--free-flight") {
            options.freeFlight = true;
        } else if (argument == "--shape") {
            options.shape = true;
            if (index + 1 < argc && argv[index + 1][0] != '-') {
                options.shapeSeconds = std::atof(argv[++index]);
            }
        } else if (argument == "--no-flight-load") {
            options.noFlightLoad = true;
        } else if (argument == "--no-cells") {
            options.noCells = true;
        } else if (argument == "--contact") {
            options.contact = true;
        } else if (argument == "--legacy-pressure") {
            options.legacyPressure = true;
        } else if (argument == "--pressure-acceptance") {
            options.pressureAcceptance = true;
            options.shape = true;
        } else if (argument == "--tuck") {
            options.tuck = true;
            // Full-string numeric parse, so a following mesh path is not
            // swallowed as a pull distance.
            if (index + 1 < argc) {
                char *end = nullptr;
                const double centimetres =
                    std::strtod(argv[index + 1], &end);
                if (end != argv[index + 1] && *end == '\0') {
                    options.tuckPullMetres = centimetres / 100.0;
                    ++index;
                }
            }
        } else if (argument == "--dive") {
            options.dive = true;
            // The angle is usually negative, so a leading '-' cannot be
            // the option-vs-value discriminator here.
            if (index + 1 < argc) {
                char *end = nullptr;
                const double degrees = std::strtod(argv[index + 1], &end);
                if (end != argv[index + 1] && *end == '\0') {
                    options.diveDegrees = degrees;
                    ++index;
                }
            }
        } else if (argument == "--shape-sweep") {
            if (index + 1 >= argc) return false;
            const QStringList parts =
                QString::fromUtf8(argv[++index]).split(u':');
            bool fromOk = false;
            bool toOk = false;
            bool stepOk = false;
            if (parts.size() == 3) {
                options.sweepFromDegrees = parts[0].toDouble(&fromOk);
                options.sweepToDegrees = parts[1].toDouble(&toOk);
                options.sweepStepDegrees = parts[2].toDouble(&stepOk);
            }
            if (!fromOk || !toOk || !stepOk
                || options.sweepStepDegrees <= 0.0) {
                std::fprintf(stderr,
                             "--shape-sweep wants FROM:TO:STEP in degrees "
                             "with a positive step, e.g. -4:24:2\n");
                return false;
            }
            options.shapeSweep = true;
        } else if (argument == "--detailed-ribs") {
            options.detailedRibs = true;
        } else if (argument == "--csv") {
            options.csv = true;
        } else if (argument == "--gpu") {
            options.gpu = true;
        } else if (argument == "--gpu-jacobi") {
            options.gpu = true;
            options.gpuMode = pg::GpuSolveMode::Jacobi;
        } else if (!argument.empty() && argument[0] == '-') {
            std::fprintf(stderr, "Unknown option: %s\n", argument.c_str());
            return false;
        } else if (options.meshPath.empty()) {
            options.meshPath = argument;
        } else {
            std::fprintf(stderr, "Unexpected argument: %s\n", argument.c_str());
            return false;
        }
    }
    return !options.meshPath.empty();
}

// Order-sensitive digest of the final pose. Two runs that should be
// bit-identical -- the same sweep at different worker counts, or the packed
// sweep against the unpacked one -- must print the same value.
std::uint64_t poseChecksum(const std::vector<softwing::Node> &nodes)
{
    std::uint64_t hash = 1469598103934665603ULL;
    const auto mix = [&hash](double value) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        hash = (hash ^ bits) * 1099511628211ULL;
    };
    for (const softwing::Node &node : nodes) {
        mix(node.position.x);
        mix(node.position.y);
        mix(node.position.z);
    }
    return hash;
}

// Enclosed volume of the closed skin, in m^3. The wing is a pressure vessel,
// so this is the single number that says whether a solver is holding it: too
// low and the fabric is being crushed, too high and it is creeping.
double enclosedVolume(const pg::SimBody &sim)
{
    const auto &nodes = sim.body->nodes();
    double volume = 0.0;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const softwing::Triangle &triangle = sim.body->triangles()[face];
        volume += dot(nodes[triangle.a].position,
                      cross(nodes[triangle.b].position,
                            nodes[triangle.c].position))
                  / 6.0;
    }
    return std::abs(volume);
}

double spanExtent(const pg::SimBody &sim)
{
    double low = 1e30;
    double high = -1e30;
    for (const softwing::Node &node : sim.body->nodes()) {
        low = std::min(low, node.position.x);
        high = std::max(high, node.position.x);
    }
    return high - low;
}

double canopyExtentAlong(const pg::SimBody &sim,
                         const softwing::Vec3 &axis)
{
    const softwing::Vec3 direction = normalized(axis);
    if (length(direction) <= 0.0) {
        return spanExtent(sim);
    }
    double low = 1e30;
    double high = -1e30;
    const std::size_t count = std::min(sim.canopyNodeCount,
                                       sim.body->nodes().size());
    for (std::size_t node = 0; node < count; ++node) {
        const double station =
            dot(sim.body->nodes()[node].position, direction);
        low = std::min(low, station);
        high = std::max(high, station);
    }
    return count > 0 ? high - low : 0.0;
}

// Which side of the centreline each cell sits on, captured ONCE before
// the collapse: a folded side's ribs cross the centreline, and a live
// classification would migrate them into the other column — corrupting
// exactly the per-side recovery signal the experiments read.
std::vector<double> cellSides(const pg::SimBody &sim)
{
    const auto &nodes = sim.body->nodes();
    std::vector<double> sides(sim.cells.size(), 0.0);
    for (std::size_t cell = 0; cell < sim.cells.size(); ++cell) {
        const double x =
            nodes[sim.ribChords[sim.cells[cell].ribs[0]].leadingNode]
                .position.x
            + nodes[sim.ribChords[sim.cells[cell].ribs[1]].leadingNode]
                  .position.x;
        sides[cell] = x > 0.0 ? 1.0 : -1.0;
    }
    return sides;
}

// Live/rest section area ratio averaged over the cells on one side of the
// centreline — the same signal the cell model's squeeze term reads, so
// the collapse experiments report the quantity the physics acts on.
double sideSectionRatio(const pg::SimBody &sim,
                        const std::vector<double> &sides,
                        double sideSign)
{
    const auto &nodes = sim.body->nodes();
    double live = 0.0;
    double rest = 0.0;
    for (std::size_t index = 0; index < sim.cells.size(); ++index) {
        if (sides[index] * sideSign <= 0.0) {
            continue;
        }
        const pg::SimCell &cell = sim.cells[index];
        double area = 0.0;
        for (const std::size_t rib : cell.ribs) {
            softwing::Vec3 sum;
            const auto &loop = sim.ribLoopNodes[rib];
            for (std::size_t node = 0; node < loop.size(); ++node) {
                sum += cross(nodes[loop[node]].position,
                             nodes[loop[(node + 1) % loop.size()]]
                                 .position);
            }
            area += 0.5 * length(sum);
        }
        live += 0.5 * area;
        rest += cell.restSectionArea;
    }
    return rest > 0.0 ? live / rest : 1.0;
}

// The single worst bay's live/rest section ratio, and how many bays are
// far enough below their rest section for the cell model's collapse vent
// to be doing anything. The side averages in the table hide both: a wing
// can lose a third of its span with every bay still near its rest
// section, and then nothing is folded in the sense the vent means.
struct WorstSection
{
    double ratio = 1.0;    // worst live/rest rib section area
    double volume = 1.0;   // worst live/rest bay volume
    int vented = 0;        // bays under the cell model's vent threshold
};
WorstSection worstSection(const pg::SimBody &sim)
{
    const auto &nodes = sim.body->nodes();
    WorstSection worst;
    for (std::size_t cellIndex = 0; cellIndex < sim.cells.size();
         ++cellIndex) {
        const pg::SimCell &cell = sim.cells[cellIndex];
        if (cell.restSectionArea <= 0.0 || cell.restVolume <= 0.0) {
            continue;
        }
        double area = 0.0;
        for (const std::size_t rib : cell.ribs) {
            softwing::Vec3 sum;
            const auto &loop = sim.ribLoopNodes[rib];
            for (std::size_t node = 0; node < loop.size(); ++node) {
                sum += cross(nodes[loop[node]].position,
                             nodes[loop[(node + 1) % loop.size()]]
                                 .position);
            }
            area += 0.5 * length(sum);
        }
        const double ratio = 0.5 * area / cell.restSectionArea;
        const double volume = cellIndex < sim.cellVolumeRatio.size()
                                  ? sim.cellVolumeRatio[cellIndex]
                                  : 1.0;
        worst.ratio = std::min(worst.ratio, ratio);
        worst.volume = std::min(worst.volume, volume);
        if (volume < 0.55) {
            ++worst.vented;
        }
    }
    return worst;
}

// One row of the collapse-experiment tables.
void printCollapseRow(const pg::SimBody &sim,
                      const std::vector<double> &sides,
                      double timeSeconds,
                      const char *phase,
                      double referenceVolume,
                      double trailing)
{
    double cellLow = 0.0;
    double cellHigh = 0.0;
    if (!sim.cellPressure.empty()) {
        cellLow = 1e30;
        cellHigh = -1e30;
        for (const double pressure : sim.cellPressure) {
            cellLow = std::min(cellLow, pressure);
            cellHigh = std::max(cellHigh, pressure);
        }
    }
    const WorstSection worst = worstSection(sim);
    std::printf("  %5.1fs   %-5s   %+6.1f   %6.2f   %+5.1f   %+5.1f"
                "   %4.0f..%-4.0f   %4.0f %4.0f %3d   %7.0f\n",
                timeSeconds,
                phase,
                referenceVolume > 0.0
                    ? 100.0 * (enclosedVolume(sim) - referenceVolume)
                          / referenceVolume
                    : 0.0,
                spanExtent(sim),
                (sideSectionRatio(sim, sides, -1.0) - 1.0) * 100.0,
                (sideSectionRatio(sim, sides, +1.0) - 1.0) * 100.0,
                cellLow,
                cellHigh,
                worst.ratio * 100.0,
                worst.volume * 100.0,
                worst.vented,
                trailing);
}

// The CSV strings come from playground_metrics; own the line framing here
// regardless of whether they carry a trailing newline.
void printCsvLine(QString line)
{
    while (line.endsWith(u'\n') || line.endsWith(u'\r')) {
        line.chop(1);
    }
    std::printf("%s\n", line.toUtf8().constData());
}

// The --shape human report. Ratios are shown as signed departures from the
// design shape — a designer reads "-3%" faster than "0.97" — and lengths in
// millimetres, the unit sail deviations are discussed in.
void printShapeReport(const pg::SimControls &controls,
                      const pg::SettleResult &result,
                      const std::string &meshPath)
{
    const pg::ShapeReport &report = result.report;
    const auto percent = [](double ratio) { return (ratio - 1.0) * 100.0; };
    std::printf("mesh            %s\n", meshPath.c_str());
    std::printf("airflow         q = %.0f Pa (%.0f km/h), alpha %+.1f deg\n",
                report.dynamicPressurePascal,
                std::sqrt(2.0 * report.dynamicPressurePascal / 1.225) * 3.6,
                report.alphaDegrees);
    std::printf("flight load     %s\n", controls.flightLoad ? "on" : "off");
    std::printf("settling        %s after %.1f s simulated\n",
                result.settled ? "settled" : "NOT settled",
                result.simulatedSeconds);
    std::printf("\n");
    std::printf("  span %+.1f%%   area %+.1f%%   volume %+.1f%%\n",
                percent(report.spanRatio),
                percent(report.areaRatio),
                percent(report.volumeRatio));
    std::printf("  slack fabric %.1f%%   asymmetry %.1f mm   "
                "agitation %.3f m/s\n",
                report.slackFraction * 100.0,
                report.asymmetryMetres * 1000.0,
                report.agitationMetresPerSecond);
    if (controls.flightLoad) {
        std::printf("  imposed polar: %.0f N lift, %.0f N drag, L/D %.2f\n",
                    report.liftNewtons,
                    report.dragNewtons,
                    report.glideRatio);
    }
    std::printf("\n");
    std::printf("  rib    rms mm   max mm   twist deg   LE dent mm"
                "   chord %%\n");
    for (std::size_t rib = 0; rib < report.ribs.size(); ++rib) {
        const pg::RibShape &shape = report.ribs[rib];
        std::printf("  %3zu   %7.1f  %7.1f     %+7.2f     %8.1f"
                    "    %+6.1f\n",
                    rib,
                    shape.rmsMetres * 1000.0,
                    shape.maxMetres * 1000.0,
                    shape.twistDegrees,
                    shape.leadingEdgeDentMetres * 1000.0,
                    percent(shape.chordRatio));
    }
    std::printf("\n");
    if (report.rows.empty()) {
        std::printf("  (no row loads: this mesh predates the line plan "
                    "tags, so segments cannot be grouped into rows)\n");
    } else {
        std::printf("  row     left N    right N   segments   slack\n");
        for (const pg::RowLoad &row : report.rows) {
            std::printf("   %c    %8.1f   %8.1f       %4d    %4d%s\n",
                        row.row.toLatin1(),
                        row.leftNewtons,
                        row.rightNewtons,
                        row.segments,
                        row.slackSegments,
                        row.brake ? "   (brake)" : "");
        }
    }
    std::printf("\n");
    if (report.flags.empty()) {
        std::printf("  no flags\n");
    } else {
        for (const pg::ShapeFlagInfo &flag : report.flags) {
            std::printf("  %s: %s\n",
                        pg::shapeFlagName(flag.flag).toUtf8().constData(),
                        flag.detail.toUtf8().constData());
        }
    }
}

double millisecondsOf(std::uint64_t nanoseconds, int frames)
{
    return static_cast<double>(nanoseconds) / 1.0e6
           / static_cast<double>(frames);
}

void reportLine(const char *label,
                std::uint64_t nanoseconds,
                std::uint64_t totalNanoseconds,
                int frames)
{
    const double share =
        totalNanoseconds == 0
            ? 0.0
            : 100.0 * static_cast<double>(nanoseconds)
                  / static_cast<double>(totalNanoseconds);
    std::printf("  %-26s %9.3f ms/frame  %5.1f%%\n",
                label,
                millisecondsOf(nanoseconds, frames),
                share);
}

void printPressureSolve(const pg::SimBody &sim)
{
    const pg::PressureSolveDiagnostics &solve = sim.pressureSolve;
    if (!solve.attempted && !solve.legacy) {
        return;
    }
    std::fprintf(
        stderr,
        "pressure %s: %s%s, authority %.4f/%.4f/%.4f, "
        "rank %zu/%zu/%zu, %zu faces, active %zu low + %zu high, "
        "%zu projections/%zu Newton iterations/%.2f ms, "
        "Cp %.4f..%.4f, residual F %.3f %.3f %.3f N, "
        "pitch %.3f N.m, half dL/dD %.3f/%.3f N\n",
        solve.legacy ? "legacy increment+clamp" : "bounded final Cp",
        solve.valid ? "valid" : "invalid",
        solve.numericalFailure ? " NUMERICAL FAILURE" : "",
        solve.authority[0], solve.authority[1], solve.authority[2],
        solve.rank[0], solve.rank[1], solve.rank[2],
        solve.variableCount,
        solve.activeLower, solve.activeUpper,
        solve.projectionCalls, solve.projectionIterations,
        solve.solveMilliseconds,
        solve.minimumCp, solve.maximumCp,
        solve.forceResidual.x, solve.forceResidual.y,
        solve.forceResidual.z, solve.pitchResidual,
        solve.halfDifferenceResidual[0],
        solve.halfDifferenceResidual[1]);
    if (!solve.legacy) {
        std::fprintf(
            stderr,
            "  authority cache hint %.4f/%.4f/%.4f, probe %c/%c/%c, "
            "backoff %zu/%zu/%zu\n",
            solve.authorityHint[0], solve.authorityHint[1],
            solve.authorityHint[2],
            solve.authorityProbeAccepted[0] ? '+' : '-',
            solve.authorityProbeAccepted[1] ? '+' : '-',
            solve.authorityProbeAccepted[2] ? '+' : '-',
            solve.authorityBackoffs[0], solve.authorityBackoffs[1],
            solve.authorityBackoffs[2]);
    }
    if (!solve.legacy && !sim.faceRetrimPreferredCp.empty()) {
        const auto [low, high] = std::minmax_element(
            sim.faceRetrimPreferredCp.begin(),
            sim.faceRetrimPreferredCp.end());
        std::fprintf(stderr,
                     "  same-frame legacy preferred Cp %.4f..%.4f\n",
                     *low, *high);
    }
}

}  // namespace

int main(int argc, char **argv)
{
    Options options;
    if (!parseOptions(argc, argv, options)) {
        std::fprintf(stderr,
                     "usage: softwing-bench <lep-sim.json> [--subdiv N] "
                     "[--detailed-ribs] [--frames N] [--warmup N] "
                     "[--threads N] [--substeps N] [--iterations N] "
                     "[--line-sweeps N] "
                     "[--gpu|--gpu-jacobi] [--csv] "
                     "[--shape [SECONDS]] [--shape-sweep FROM:TO:STEP] "
                     "[--tuck [PULL_CM]] [--dive [DEGREES]] [--no-cells] "
                     "[--contact] [--no-flight-load] [--legacy-pressure] "
                     "[--pressure-acceptance] "
                     "[--glide N] [--brake CM] [--release SECONDS] "
                     "[--pilot-mass KG] [--drop-from-rest] "
                     "[--membrane] [--warp-stiffness N/M] "
                     "[--weft-stiffness N/M] [--coupling-stiffness N/M] "
                     "[--shear-stiffness N/M] [--membrane-damping SEC] "
                     "[--compression-ratio R] [--bend-compliance C]\n");
        return 2;
    }

    // Only the GPU path needs it, but a QGuiApplication is cheap and keeping
    // one construction path avoids two ways to start the same tool.
    QGuiApplication application(argc, argv);

    QFile file(QString::fromStdString(options.meshPath));
    if (!file.open(QIODevice::ReadOnly)) {
        std::fprintf(stderr,
                     "Could not read %s\n",
                     options.meshPath.c_str());
        return 2;
    }
    const QByteArray data = file.readAll();

    QString error;
    const auto mesh = pg::parseSimMesh(data, error);
    if (!mesh) {
        std::fprintf(stderr, "%s\n", error.toUtf8().constData());
        return 2;
    }

    // The page densifies the ribs along with the skin; mirror that so a
    // --subdiv here means what the resolution control means.
    pg::SimBuildOptions build;
    build.detailedRibs = options.detailedRibs;
    build.ribLayers = pg::defaultRibLayers + 2 * (options.subdivision - 1);
    build.ribStationSplit =
        pg::defaultRibStationSplit + options.subdivision - 1;

    pg::SimControls controls;
    controls.substeps = options.substeps;
    controls.constraintIterations = options.iterations;
    controls.freeFlightCableSweepPairs = options.lineSweepPairs;
    controls.workerThreads = options.threads;
    controls.pressurePascal = options.pressurePascal;
    controls.angleOfAttackDegrees = options.angleOfAttackDegrees;
    controls.pilotMassKg = options.pilotMassKg;
    controls.launchMode = options.launchMode;
    controls.skinModel = options.skinModel;
    controls.warpStiffness = options.warpStiffness;
    controls.weftStiffness = options.weftStiffness;
    controls.couplingStiffness = options.couplingStiffness;
    controls.shearStiffness = options.shearStiffness;
    controls.membraneDampingSeconds = options.membraneDampingSeconds;
    controls.compressionStiffnessRatio = options.compressionRatio;
    controls.bendCompliance = options.bendCompliance;
    controls.pressureSolveMode =
        options.legacyPressure
            ? pg::PressureSolveMode::LegacyIncrementClamp
            : pg::PressureSolveMode::BoundedExteriorCp;
    controls.freeFlight = options.freeFlight;
    // The shape modes load the tunnel like flight by default (a tunnel
    // carrying only the pressure field's own resultant under-reads every
    // line load — see docs/playground-shape-analysis.md), and hold any
    // --brake pull from the first step. Every other mode keeps flightLoad
    // false, which is what keeps the timing baselines and pose checksums
    // bit-identical to runs that predate these flags.
    controls.cellPressureModel = !options.noCells;
    controls.fabricContact = options.contact;
    if (options.shape || options.shapeSweep || options.tuck
        || options.dive) {
        if (options.freeFlight) {
            // A free-flying wing chooses its own angle of attack, so a
            // sweep prescribing alpha would label its rows with angles
            // the wing never flew at. Refuse rather than mislabel.
            std::fprintf(stderr,
                         "--shape/--shape-sweep/--tuck/--dive are tunnel "
                         "modes; --free-flight does not combine with "
                         "them.\n");
            return 2;
        }
        controls.flightLoad = !options.noFlightLoad;
        controls.brakeLeft = options.brakeMetres;
        controls.brakeRight = options.brakeMetres;
    }

    const auto buildStart = std::chrono::steady_clock::now();
    const pg::SimMesh refined =
        pg::refineSimMesh(*mesh, options.subdivision);
    pg::SimBody sim = pg::buildSimBody(refined, build, controls);
    const double buildSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now()
                                      - buildStart)
            .count();
    std::fprintf(stderr,
                 "skin model %s: %zu membranes, %zu hinges, %zu/%zu "
                 "degenerate elements/hinges skipped\n",
                 pg::skinModelName(sim.skinModel),
                 sim.body->membraneElements().size(),
                 sim.body->dihedralConstraints().size(),
                 sim.skippedMembraneElements,
                 sim.skippedDihedralHinges);
    if (sim.skinModel == pg::SkinModel::OrthotropicMembrane) {
        const auto &m = sim.skinMaterial;
        std::fprintf(stderr,
                     "prototype material warp/weft/coupling/shear "
                     "%.0f/%.0f/%.0f/%.0f N/m, damping %.3f s, "
                     "compression %.4f, bend compliance %.3g\n",
                     m.warpStiffness,
                     m.weftStiffness,
                     m.couplingStiffness,
                     m.shearStiffness,
                     m.dampingTime,
                     m.compressionStiffnessRatio,
                     sim.skinBendCompliance);
        if (m.dampingTime > 0.0) {
            std::fprintf(stderr,
                         "WARNING: nonzero membrane damping is experimental "
                         "in the mixed membrane/rib/line network\n");
        }
    }
    if (sim.tunnelLineSolverBallastKg > 0.0) {
        std::fprintf(stderr,
                     "line relaxation %.3f kg nonphysical pinned-tunnel "
                     "solver ballast (zero gravity)\n",
                     sim.tunnelLineSolverBallastKg);
    }

    // The cross-ports: the only path back into a cell whose own mouth
    // has folded shut, so a run of bays with no port is a run that
    // cannot re-inflate from its neighbours at all.
    if (!sim.cells.empty()) {
        std::size_t ported = 0;
        double smallest = 1e30;
        double largest = 0.0;
        std::size_t run = 0;
        std::size_t worstRun = 0;
        std::size_t worstEnd = 0;
        for (std::size_t cell = 0; cell + 1 < sim.cells.size(); ++cell) {
            const double area = sim.cells[cell].portAreaToNext;
            if (area > 0.0) {
                ++ported;
                smallest = std::min(smallest, area);
                largest = std::max(largest, area);
                run = 0;
            } else {
                ++run;
                if (run > worstRun) {
                    worstRun = run;
                    worstEnd = cell;
                }
            }
        }
        std::printf("cross-ports     %zu of %zu rib crossings ported, "
                    "%.4f..%.4f m2",
                    ported,
                    sim.cells.size() - 1,
                    ported > 0 ? smallest : 0.0,
                    largest);
        if (worstRun > 0) {
            std::printf("; longest unported run %zu at bays %zu..%zu",
                        worstRun,
                        worstEnd + 1 - worstRun,
                        worstEnd);
        }
        std::printf("\n");
    }

    // Where along the span the canopy actually has lines on it. A wing
    // folds where nothing holds it, so the longest run of bays with no
    // attachment is the first thing to check against a hinge that keeps
    // appearing at the same station.
    if (!sim.cells.empty() && !sim.lineAttachmentNodes.empty()) {
        std::vector<std::size_t> order;
        order.push_back(sim.cells.front().ribs[0]);
        for (const pg::SimCell &cell : sim.cells) {
            order.push_back(cell.ribs[1]);
        }
        std::vector<double> station(order.size(), 0.0);
        for (std::size_t index = 0; index < order.size(); ++index) {
            station[index] =
                sim.body->nodes()[sim.ribChords[order[index]].leadingNode]
                    .position.x;
        }
        std::vector<bool> supported(sim.cells.size(), false);
        for (const std::size_t node : sim.lineAttachmentNodes) {
            const double x = sim.body->nodes()[node].position.x;
            for (std::size_t bay = 0; bay < sim.cells.size(); ++bay) {
                const double low = std::min(station[bay], station[bay + 1]);
                const double high = std::max(station[bay], station[bay + 1]);
                if (x >= low && x <= high) {
                    supported[bay] = true;
                }
            }
        }
        std::size_t run = 0;
        std::size_t worstRun = 0;
        std::size_t worstEnd = 0;
        for (std::size_t bay = 0; bay < supported.size(); ++bay) {
            run = supported[bay] ? 0 : run + 1;
            if (run > worstRun) {
                worstRun = run;
                worstEnd = bay;
            }
        }
        std::printf("line support    %zu of %zu bays carry an attachment; "
                    "longest gap %zu bays",
                    static_cast<std::size_t>(
                        std::count(supported.begin(), supported.end(), true)),
                    supported.size(),
                    worstRun);
        if (worstRun > 0) {
            std::printf(" at span %.2f..%.2f (x %+.2f..%+.2f m)",
                        static_cast<double>(worstEnd + 1 - worstRun)
                            / static_cast<double>(supported.size()),
                        static_cast<double>(worstEnd + 1)
                            / static_cast<double>(supported.size()),
                        station[worstEnd + 1 - worstRun],
                        station[worstEnd + 1]);
        }
        std::printf("\n");
    }

    // The designed shape, before any pressure has acted on it: the yardstick
    // for how far the solver lets the fabric balloon.
    const double designVolume = enclosedVolume(sim);
    const std::size_t nodeCount = sim.body->nodes().size();
    const std::size_t constraintCount = sim.body->constraints().size();
    const std::size_t triangleCount = sim.body->triangles().size();

    // The rigid polar: the canopy held at its design shape (no stepping at
    // all), the airflow swept over angle of attack, and the imposed
    // wing-level polar read back at each angle. This is the calibration
    // view — everything here is analytic in the rest geometry, so a wrong
    // aspect ratio, a wrong reference area or a wrong sign shows up in
    // seconds without a solver in the loop.
    if (options.polar) {
        std::printf("mesh            %s\n", options.meshPath.c_str());
        std::printf("wing            %.2f m^2 projected planform, "
                    "aspect ratio %.2f, span %.2f m\n",
                    sim.planformArea,
                    sim.aspectRatio,
                    spanExtent(sim));
        std::printf("airspeed        %.1f m/s (q = %.0f Pa)\n\n",
                    std::sqrt(2.0 * options.pressurePascal / 1.225),
                    options.pressurePascal);
        std::printf("  slider   wing alpha      CL      CD     L/D"
                    "     lift N   drag N   pressure N (z / along-wind)\n");
        for (int degrees = -4; degrees <= 14; ++degrees) {
            pg::SimControls sweep = controls;
            sweep.angleOfAttackDegrees = degrees;
            sweep.freeFlight = false;
            pg::applyPressure(sim, sweep);
            const pg::WingAeroSample sample =
                pg::sampleWingAero(sim, sweep);
            if (!sample.valid) {
                std::printf("  %+5d    (no sample)\n", degrees);
                continue;
            }
            const double lift = sample.dynamicPressure * sim.planformArea
                                * sample.liftCoefficient;
            const double drag =
                sample.dynamicPressure
                * (sim.planformArea * sample.dragCoefficient
                   + 0.35);
            const softwing::Vec3 pressure = pg::aerodynamicForce(sim);
            std::printf("  %+5d    %+7.2f deg  %6.3f  %6.4f  %6.2f"
                        "   %8.1f  %7.1f   %8.1f / %+8.1f\n",
                        degrees,
                        sample.alphaRadians * 180.0 / 3.14159265358979,
                        sample.liftCoefficient,
                        sample.dragCoefficient,
                        sample.dragCoefficient > 0.0
                            ? sample.liftCoefficient
                                  / sample.dragCoefficient
                            : 0.0,
                        lift,
                        drag,
                        pressure.z,
                        dot(pressure, sample.windDirection));
        }
        return 0;
    }

    if (options.shape || options.shapeSweep || options.tuck
        || options.dive) {
        // ShapeBaseline compares live sections against rest ones; a mesh
        // without rib loops has no sections to compare.
        if (sim.ribChords.empty()) {
            std::fprintf(stderr,
                         "This mesh has no rib chords; the shape "
                         "instruments need sections.\n");
            return 1;
        }
    }

    // The shape sweep: the wind tunnel run across an angle-of-attack range,
    // a fresh body per point so no point inherits the previous one's
    // settled pose. Always CSV on stdout — a sweep is data, not prose —
    // with a per-point progress note on stderr so a long run is watchable
    // without contaminating the data stream.
    if (options.shapeSweep) {
        printCsvLine(pg::shapeReportCsvHeader());
        for (int point = 0;; ++point) {
            const double alpha = options.sweepFromDegrees
                                 + point * options.sweepStepDegrees;
            // Inclusive endpoint; the epsilon covers representation error
            // in from + n*step, not a half-step of generosity.
            if (alpha > options.sweepToDegrees
                            + options.sweepStepDegrees * 1e-6) {
                break;
            }
            pg::SimControls at = controls;
            at.angleOfAttackDegrees = alpha;
            pg::SimBody wing = pg::buildSimBody(refined, build, at);
            const pg::ShapeBaseline baseline =
                pg::captureShapeBaseline(wing);
            const pg::SettleResult result = pg::settleAndMeasure(
                wing, at, baseline, options.shapeSeconds);
            printCsvLine(pg::shapeReportCsvRow(result.report));
            std::fflush(stdout);
            std::fprintf(stderr,
                         "alpha %+.1f: %s %.1f s, %zu flag%s\n",
                         alpha,
                         result.settled ? "settled" : "unsettled",
                         result.simulatedSeconds,
                         result.report.flags.size(),
                         result.report.flags.size() == 1 ? "" : "s");
            printPressureSolve(wing);
        }
        return 0;
    }

    // Real-wing P5 acceptance: compare the bounded final-Cp production path
    // against the explicit legacy oracle from fresh, identical bodies. This
    // guards structural distribution, not merely zero global residuals.
    if (options.pressureAcceptance) {
        if (options.legacyPressure) {
            std::fprintf(stderr,
                         "--pressure-acceptance selects both pressure "
                         "paths; do not combine it with --legacy-pressure.\n");
            return 2;
        }
        const pg::ShapeBaseline boundedBaseline =
            pg::captureShapeBaseline(sim);
        const pg::SettleResult bounded = pg::settleAndMeasure(
            sim, controls, boundedBaseline, options.shapeSeconds);

        pg::SimControls legacyControls = controls;
        legacyControls.pressureSolveMode =
            pg::PressureSolveMode::LegacyIncrementClamp;
        pg::SimBody legacy = pg::buildSimBody(
            refined, build, legacyControls);
        const pg::ShapeBaseline legacyBaseline =
            pg::captureShapeBaseline(legacy);
        const pg::SettleResult oracle = pg::settleAndMeasure(
            legacy, legacyControls, legacyBaseline, options.shapeSeconds);

        const auto finiteReport = [](const pg::ShapeReport &report) {
            return std::isfinite(report.spanRatio)
                   && std::isfinite(report.areaRatio)
                   && std::isfinite(report.volumeRatio)
                   && std::isfinite(report.worstLeadingEdgeDentMetres)
                   && std::isfinite(report.worstTwistDegrees)
                   && std::isfinite(report.agitationMetresPerSecond);
        };
        constexpr double kDentToleranceMetres = 0.010;
        constexpr double kWashoutToleranceDegrees = 0.5;
        const bool accepted =
            bounded.settled && finiteReport(bounded.report)
            && bounded.report.flags.empty()
            && sim.pressureSolve.valid
            && !sim.pressureSolve.numericalFailure
            && sim.pressureSolve.minimumCp
                   >= pg::minimumExteriorPressureCoefficient - 1.0e-9
            && sim.pressureSolve.maximumCp
                   <= pg::maximumExteriorPressureCoefficient + 1.0e-9
            && bounded.report.worstLeadingEdgeDentMetres
                   <= oracle.report.worstLeadingEdgeDentMetres
                          + kDentToleranceMetres
            && std::abs(bounded.report.worstTwistDegrees)
                   <= std::abs(oracle.report.worstTwistDegrees)
                          + kWashoutToleranceDegrees;
        std::printf(
            "pressure acceptance (%s)\n"
            "                 bounded      legacy       allowed delta\n"
            "  settled        %-8s     %-8s\n"
            "  flags          %-8zu     %-8zu\n"
            "  LE dent mm     %8.1f     %8.1f       +%.1f\n"
            "  washout deg    %8.2f     %8.2f       +%.2f abs\n"
            "  agitation mm/s %8.1f     %8.1f\n"
            "  span ratio     %8.4f     %8.4f\n"
            "  volume ratio   %8.4f     %8.4f\n",
            accepted ? "PASS" : "FAIL",
            bounded.settled ? "yes" : "no",
            oracle.settled ? "yes" : "no",
            bounded.report.flags.size(), oracle.report.flags.size(),
            1000.0 * bounded.report.worstLeadingEdgeDentMetres,
            1000.0 * oracle.report.worstLeadingEdgeDentMetres,
            1000.0 * kDentToleranceMetres,
            bounded.report.worstTwistDegrees,
            oracle.report.worstTwistDegrees,
            kWashoutToleranceDegrees,
            1000.0 * bounded.report.agitationMetresPerSecond,
            1000.0 * oracle.report.agitationMetresPerSecond,
            bounded.report.spanRatio, oracle.report.spanRatio,
            bounded.report.volumeRatio, oracle.report.volumeRatio);
        printPressureSolve(sim);
        return accepted ? 0 : 1;
    }

    // A single wind-tunnel measurement: settle at the current controls,
    // then print the full instrument report (or its CSV row, for scripts).
    if (options.shape) {
        const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
        const auto settleStart = std::chrono::steady_clock::now();
        const pg::SettleResult result = pg::settleAndMeasure(
            sim, controls, baseline, options.shapeSeconds);
        const double settleMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - settleStart)
                .count();
        if (options.csv) {
            printCsvLine(pg::shapeReportCsvHeader());
            printCsvLine(pg::shapeReportCsvRow(result.report));
        } else {
            printShapeReport(controls, result, options.meshPath);
        }
        printPressureSolve(sim);
        const double frames =
            result.simulatedSeconds / pg::simulationTimeStep;
        std::fprintf(stderr,
                     "shape stepping %.2f ms total, %.2f ms/frame "
                     "over %.0f frames (build %.2f ms excluded)\n",
                     settleMilliseconds,
                     frames > 0.0 ? settleMilliseconds / frames : 0.0,
                     frames,
                     1000.0 * buildSeconds);
        return 0;
    }

    // The collapse-recovery experiment. Settle under flight load, grab
    // the outermost A-row cascade junction on the +x side (the same
    // interactive grab the GUI has), haul it straight down until that
    // side folds, hold, let go, and watch the recovery signals: with the
    // per-cell air model the folded side seals its intake, gets re-fed
    // through the cross-ports and squeezed back open; with --no-cells the
    // old blanket stamp pins the fold shut forever.
    if (options.tuck) {
        const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
        const pg::SettleResult settled = pg::settleAndMeasure(
            sim, controls, baseline, options.shapeSeconds);
        std::printf("mesh            %s\n", options.meshPath.c_str());
        std::printf("cell model      %s\n",
                    controls.cellPressureModel ? "on" : "off (--no-cells)");
        std::printf("settle          %s after %.1f s\n",
                    settled.settled ? "settled" : "NOT settled",
                    settled.simulatedSeconds);

        // The grab target: a node used by at least two A-row segments —
        // a cascade junction, not a skin attachment — excluding the
        // carabiners, furthest out on the +x side.
        std::map<std::size_t, int> junctionUses;
        for (const pg::LineSegment &segment : sim.lineSegments) {
            if (segment.plan != 1) {
                continue;
            }
            ++junctionUses[segment.a];
            ++junctionUses[segment.b];
        }
        for (const std::size_t node : sim.carabinerNodes) {
            junctionUses.erase(node);
        }
        std::size_t grabNode = pg::noConstraint;
        double bestX = 0.0;
        for (const auto &[node, uses] : junctionUses) {
            if (uses < 2 || node < sim.canopyNodeCount) {
                continue;
            }
            const double x = sim.body->nodes()[node].position.x;
            if (grabNode == pg::noConstraint || x > bestX) {
                grabNode = node;
                bestX = x;
            }
        }
        if (grabNode == pg::noConstraint) {
            std::fprintf(stderr,
                         "No A-row cascade junction to grab; this mesh "
                         "predates the line plan tags.\n");
            return 1;
        }
        const softwing::Vec3 grabStart =
            sim.body->nodes()[grabNode].position;
        std::printf("grab            node %zu at (%.2f, %.2f, %.2f), "
                    "pulling %.2f m down\n\n",
                    grabNode,
                    grabStart.x,
                    grabStart.y,
                    grabStart.z,
                    options.tuckPullMetres);

        const double settledVolume = enclosedVolume(sim);
        const std::vector<double> sides = cellSides(sim);
        std::printf("   time   phase   volume%%   span m   secL%%   secR%%"
                    "   cells Pa   sec  vol vnt      grab N\n");
        pg::beginGrab(sim, grabNode);
        const int rampFrames = 60;
        const int holdFrames = 60;
        const int recoverFrames = 900;
        const int totalFrames = rampFrames + holdFrames + recoverFrames;
        for (int frame = 0; frame < totalFrames; ++frame) {
            const bool pulling = frame < rampFrames + holdFrames;
            if (pulling) {
                const double progress = std::min(
                    1.0,
                    static_cast<double>(frame + 1)
                        / static_cast<double>(rampFrames));
                // Down AND aft: a straight-down pull only pitches the
                // tethered wing and it springs back; dragging the A
                // cascade backwards under the wing is what folds the
                // leading edge under — the real front-tuck gesture.
                pg::moveGrab(sim,
                             grabStart
                                 + progress * options.tuckPullMetres
                                       * softwing::Vec3{0.0, 0.6, -0.8});
            } else if (frame == rampFrames + holdFrames) {
                pg::endGrab(sim);
            }
            pg::stepSimulation(sim, controls);
            if (frame % 30 != 29) {
                continue;
            }
            printCollapseRow(sim,
                             sides,
                             (frame + 1) / 60.0,
                             pulling ? (frame < rampFrames ? "pull"
                                                           : "hold")
                                     : "free",
                             settledVolume,
                             pulling
                                 ? pg::grabForceNewtons(sim, controls)
                                 : 0.0);
        }
        std::printf("\n");
        const pg::ShapeReport after =
            pg::measureShape(sim, controls, baseline);
        if (after.flags.empty()) {
            std::printf("  recovered: no flags\n");
        } else {
            for (const pg::ShapeFlagInfo &flag : after.flags) {
                std::printf("  %s: %s\n",
                            pg::shapeFlagName(flag.flag).toUtf8().constData(),
                            flag.detail.toUtf8().constData());
            }
        }
        return 0;
    }

    // The aerodynamic collapse-recovery experiment. Settle at trim, slam
    // the airflow to a front-tuck angle (the calibrated collapse boundary
    // sits at -4 degrees on gnuC2), hold it there while the nose folds,
    // return the airflow to trim, and watch whether the wing takes itself
    // back. The last column is the prescribed slider angle.
    if (options.dive) {
        const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
        const pg::SettleResult settled = pg::settleAndMeasure(
            sim, controls, baseline, options.shapeSeconds);
        std::printf("mesh            %s\n", options.meshPath.c_str());
        std::printf("cell model      %s\n",
                    controls.cellPressureModel ? "on" : "off (--no-cells)");
        std::printf("settle          %s after %.1f s\n",
                    settled.settled ? "settled" : "NOT settled",
                    settled.simulatedSeconds);
        std::printf("dive            alpha %+.1f deg for 3 s, then back "
                    "to %+.1f deg\n\n",
                    options.diveDegrees,
                    controls.angleOfAttackDegrees);
        const double settledVolume = enclosedVolume(sim);
        const std::vector<double> sides = cellSides(sim);
        std::printf("   time   phase   volume%%   span m   secL%%   secR%%"
                    "   cells Pa   sec  vol vnt  alpha deg\n");
        pg::SimControls dived = controls;
        dived.angleOfAttackDegrees = options.diveDegrees;
        const int diveFrames = 180;
        const int recoverFrames = 900;
        for (int frame = 0; frame < diveFrames + recoverFrames; ++frame) {
            const bool diving = frame < diveFrames;
            pg::stepSimulation(sim, diving ? dived : controls);
            if (frame % 30 != 29) {
                continue;
            }
            printCollapseRow(sim,
                             sides,
                             (frame + 1) / 60.0,
                             diving ? "dive" : "trim",
                             settledVolume,
                             diving ? options.diveDegrees
                                    : controls.angleOfAttackDegrees);
        }
        std::printf("\n");
        const pg::ShapeReport after =
            pg::measureShape(sim, controls, baseline);
        if (after.flags.empty()) {
            std::printf("  recovered: no flags\n");
        } else {
            for (const pg::ShapeFlagInfo &flag : after.flags) {
                std::printf("  %s: %s\n",
                            pg::shapeFlagName(flag.flag).toUtf8().constData(),
                            flag.detail.toUtf8().constData());
            }
        }
        return 0;
    }

    // The free-flight convergence run: does the whole coupled system —
    // canopy, lines, pilot, polar force pass, relative-wind feedback —
    // settle into a steady glide and keep its shape while doing it?
    if (options.glideFrames > 0) {
        if (sim.pilotNode == pg::noConstraint) {
            std::fprintf(stderr, "This mesh has no suspension lines.\n");
            return 1;
        }
        std::printf("pilot mass      %.1f kg\n", sim.pilotMass);
        std::printf("launch          %s\n",
                    pg::launchModeName(controls.launchMode));
        if (controls.launchMode == pg::LaunchMode::TrimmedGlide) {
            const double launchSupport = sim.lastAeroForce.z;
            std::printf("launch trim     %.2f m/s, %.1f Pa, effective CL %.3f,"
                        " achieved support %.0f N, residual H/V %.0f/%+.0f N,"
                        " %d calibration solves, relax %d frames\n",
                        sim.trimmedLaunchAirspeed,
                        sim.trimmedLaunchDynamicPressure,
                        sim.trimmedLaunchEffectiveLiftCoefficient,
                        launchSupport,
                        sim.trimmedLaunchHorizontalResidualNewtons,
                        sim.trimmedLaunchVerticalResidualNewtons,
                        sim.trimmedLaunchCalibrationIterations,
                        sim.trimmedLaunchRelaxationFrames);
            std::printf("flight rig      %.1f%% chord resultant, trim alpha %.2f deg,"
                        " glide path %.2f deg\n",
                        100.0 * sim.resultantChordFraction,
                        sim.alphaTrimRadians * 180.0
                            / 3.14159265358979323846,
                        sim.glideAngleRadians * 180.0
                            / 3.14159265358979323846);
        }
        std::printf("system mass     %.1f kg (%zu duplicate lines rejected)\n",
                    pg::simulatedMassKilograms(sim),
                    sim.duplicateLineCount);
        if (sim.virtualAddedAirMassKg > 0.0) {
            std::printf("added air       %.1f kg solver inertia (zero weight)\n",
                        sim.virtualAddedAirMassKg);
        }
        std::printf("line mass       %.3f kg authored + %.4f kg junction"
                    " floor + %.4f kg control floor\n",
                    sim.authoredLineMassKg,
                    sim.lineJunctionFloorMassKg,
                    sim.controlNodeFloorMassKg);
        std::printf("system          %.2f m^2 planform, AR %.2f\n\n",
                    sim.planformArea,
                    sim.aspectRatio);
        std::printf("solver          %d x %d + %d load-path sweep pair%s\n\n",
                    controls.substeps, controls.constraintIterations,
                    controls.freeFlightCableSweepPairs,
                    controls.freeFlightCableSweepPairs == 1 ? "" : "s");
        std::printf("   time    airspeed   alpha     L/D    fwd m/s"
                    "   sink m/s   span m   volume    pilot below\n");
        // Turn instrumentation. A brake's whole job is to steer, and none
        // of the columns above can see a turn: the system is re-centred on
        // the origin every frame, so heading lives only in the velocity.
        // Bank comes from the live span axis (the same one the polar
        // measures its angle of attack against), heading from the
        // horizontal flight direction, and the two together are what says
        // whether a one-sided pull produced a turn or merely a departure.
        double previousHeading = 0.0;
        double previousTime = 0.0;
        double headingTotal = 0.0;
        bool haveHeading = false;
        const auto systemVelocity = [&sim] {
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
        };
        for (int frame = 0; frame < options.glideFrames; ++frame) {
            // Brakes come on after two seconds of hands-up flight, so the
            // trim settles first and the brake response is legible, and go
            // off again at --release. Releasing is the half of a hard pull
            // that a wing has to survive: hands up, a real canopy surges
            // and flies away, so a run that never lets go cannot tell a
            // recoverable stall from a permanent one.
            const bool pulling =
                frame >= 120
                && (options.releaseFrames <= 0
                    || frame < options.releaseFrames);
            const double reach =
                options.rampFrames > 0
                    ? std::min(1.0,
                               static_cast<double>(frame - 120)
                                   / options.rampFrames)
                    : 1.0;
            controls.brakeLeft =
                pulling ? reach
                              * (options.asymmetricBrake
                                     ? options.brakeLeftMetres
                                     : options.brakeMetres)
                        : 0.0;
            controls.brakeRight =
                pulling ? reach
                              * (options.asymmetricBrake
                                     ? options.brakeRightMetres
                                     : options.brakeMetres)
                        : 0.0;
            pg::stepSimulation(sim, controls);
            // Finer cadence over the first second, where launch transients
            // live.
            if (frame < 120 ? frame % 6 != 5 : frame % 60 != 59) {
                continue;
            }
            const softwing::Vec3 velocity = systemVelocity();
            const softwing::Vec3 throughAir =
                velocity - controls.ambientAirVelocityWorld;
            const softwing::Vec3 pilot =
                sim.body->nodes()[sim.pilotNode].position;
            softwing::Vec3 canopy;
            std::size_t counted = 0;
            for (std::size_t node = 0; node < sim.body->nodes().size();
                 ++node) {
                canopy += sim.body->nodes()[node].position;
                ++counted;
            }
            canopy /= static_cast<double>(counted);
            const pg::FlightFrameSample flightFrame =
                pg::sampleFlightFrame(sim, controls);
            const double forward = flightFrame.valid
                                       ? dot(throughAir,
                                             flightFrame.forwardDirection)
                                       : 0.0;
            std::printf("  %5.1fs   %7.2f   %+6.2f  %6.2f    %+6.2f"
                        "    %+6.2f    %6.2f    %+5.1f%%     %6.2f m"
                        "   [L %5.0f N, Pz %5.0f N]\n",
                        (frame + 1) / 60.0,
                        sim.lastAirspeed,
                        sim.lastAlphaDegrees,
                        sim.lastGlideRatio,
                        forward,
                        throughAir.z,
                        flightFrame.valid
                            ? canopyExtentAlong(sim, flightFrame.spanAxis)
                            : spanExtent(sim),
                        designVolume > 0.0
                            ? 100.0 * (enclosedVolume(sim) - designVolume)
                                  / designVolume
                            : 0.0,
                        canopy.z - pilot.z,
                        sim.lastLift,
                        pg::aerodynamicForce(sim).z);
            // Is the wing's measured chord rotating rigidly, or is the
            // trailing edge just deforming under it? Compare the pitch of
            // the LE->TE chord with the pitch of centroid->LE, which no
            // TE flutter can touch.
            softwing::Vec3 leadingMean;
            softwing::Vec3 trailingMean;
            for (const pg::RibChord &rib : sim.ribChords) {
                leadingMean +=
                    sim.body->nodes()[rib.leadingNode].position;
                trailingMean +=
                    sim.body->nodes()[rib.trailingNode].position;
            }
            leadingMean /= static_cast<double>(sim.ribChords.size());
            trailingMean /= static_cast<double>(sim.ribChords.size());
            const auto pitchOf = [](const softwing::Vec3 &vec) {
                return std::atan2(vec.z, vec.y) * 180.0
                       / 3.14159265358979;
            };
            double nosePressure = 0.0;
            double tailPressure = 0.0;
            std::size_t noseCount = 0;
            std::size_t tailCount = 0;
            for (std::size_t face = 0; face < sim.skinTriangleCount;
                 ++face) {
                const double fraction = sim.faceAero[face].chordFraction;
                const double delta =
                    sim.body->triangles()[face].pressureDifference;
                if (fraction < 0.2) {
                    nosePressure += delta;
                    ++noseCount;
                } else if (fraction > 0.8) {
                    tailPressure += delta;
                    ++tailCount;
                }
            }
            // The cell states, low..high: a wing that has stopped feeding
            // its intakes shows up here long before the shape does.
            double cellLow = 0.0;
            double cellHigh = 0.0;
            if (!sim.cellPressure.empty()) {
                cellLow = 1e30;
                cellHigh = -1e30;
                for (const double pressure : sim.cellPressure) {
                    cellLow = std::min(cellLow, pressure);
                    cellHigh = std::max(cellHigh, pressure);
                }
            }
            std::printf("           residual F (%.0f %.0f %.0f) N,"
                        "  pitch M %.0f N.m, chord pitch %+.1f deg,"
                        "  LE dp %.0f Pa, TE dp %.0f Pa,"
                        "  cells %.0f..%.0f Pa\n",
                        sim.lastForceResidual.x,
                        sim.lastForceResidual.y,
                        sim.lastForceResidual.z,
                        sim.lastPitchResidual,
                        pitchOf(trailingMean - leadingMean),
                        noseCount > 0 ? nosePressure / noseCount : 0.0,
                        tailCount > 0 ? tailPressure / tailCount : 0.0,
                        cellLow,
                        cellHigh);
            printPressureSolve(sim);
            // What the lines are actually carrying. A flying wing holds
            // the pilot up, so this has to sit at his weight; anything
            // far below it means the system is falling faster than the
            // canopy can hold it and the lines have gone slack — at
            // which point nothing is pulling the wing into shape.
            const pg::LineLoadReport lineReport =
                pg::lineLoads(sim, controls);
            const double systemWeight =
                pg::simulatedMassKilograms(sim)
                * pg::gravityMetresPerSecondSquared;
            const double pilotWeight =
                sim.pilotMass * pg::gravityMetresPerSecondSquared;
            const pg::WeakCellReport weak = pg::weakestCell(sim);
            const pg::KinkReport kink = pg::sharpestKink(sim);
            std::printf("           risers %6.0f N; pilot %.0f N,"
                        " system %.0f N weight,"
                        "  %zu of %zu line segments slack,"
                        "  max cable error %.2f mm/%.2f%% at %.0f N,"
                        "  fabric drag %5.0f N over %.1f m2,"
                        "  polar skin drag %5.0f/%5.0f N (%+.0f W)\n",
                        lineReport.riserNewtons,
                        pilotWeight,
                        systemWeight,
                        lineReport.slackSegments,
                        lineReport.totalSegments,
                        1000.0 * lineReport.maximumExtensionMetres,
                        100.0 * lineReport.maximumExtensionFraction,
                        lineReport.maximumTensionNewtons,
                        sim.lastFabricDragNewtons,
                        sim.lastExcessFrontalArea,
                        sim.lastPolarDragTractionNewtons,
                        sim.lastPolarDragTargetNewtons,
                        sim.lastPolarDragTractionPowerWatts);
            std::printf("           weakest cell #%zu at x %+.2f m: "
                        "section %.0f%%, volume %.0f%%, p/ram %.0f/%.0f Pa, "
                        "mouth %.0f%%;"
                        "  kink %.0f deg at rib %zu"
                        " (span %.2f, x %+.2f m)\n",
                        weak.index,
                        weak.x,
                        100.0 * weak.sectionRatio,
                        100.0 * weak.volumeRatio,
                        weak.pressurePascal,
                        weak.ramPressurePascal,
                        100.0 * weak.intakeOpening,
                        kink.degrees,
                        kink.rib,
                        kink.spanFraction,
                        kink.x);
            // Bank, nose heading, course, turn rate and sideslip.
            //
            // Heading is taken from the wing's travel THROUGH THE AIR,
            // not from its ground track. The model flies the wing in an
            // air mass that is itself moving at the airspeed the pressure
            // slider sets, so the ground velocity is the difference of
            // two comparable vectors and its direction says almost
            // nothing about where the wing is pointing — a 10 degree yaw
            // can swing it 90.
            //
            // Signs are the SOLVER's: zero points along the physical nose
            // direction (-Y), positive heading turns toward +X, and positive
            // bank means lift is tilted toward +X. A coordinated turn has
            // bank and course rate with the same sign. Nose minus course
            // exposes the yaw/course separation that a single "heading"
            // column used to hide.
            const double time = (frame + 1) / 60.0;
            double bankDegrees = 0.0;
            double turnRate = 0.0;
            if (flightFrame.valid) {
                bankDegrees = flightFrame.bankRadians
                              * 180.0 / 3.14159265358979;
                const double horizontal = std::sqrt(
                    flightFrame.travelVelocity.x
                        * flightFrame.travelVelocity.x
                    + flightFrame.travelVelocity.y
                          * flightFrame.travelVelocity.y);
                if (horizontal > 1.0e-6) {
                    const double heading = flightFrame.courseHeadingRadians;
                    if (haveHeading && time > previousTime) {
                        double delta = heading - previousHeading;
                        while (delta > 3.14159265358979) {
                            delta -= 2.0 * 3.14159265358979;
                        }
                        while (delta < -3.14159265358979) {
                            delta += 2.0 * 3.14159265358979;
                        }
                        headingTotal += delta;
                        turnRate = delta / (time - previousTime) * 180.0
                                   / 3.14159265358979;
                    }
                    previousHeading = heading;
                    previousTime = time;
                    haveHeading = true;
                }
            }
            std::printf("           bank %+6.2f deg,  nose %+7.1f deg,"
                        "  course %+7.1f deg,  turn %+6.2f deg/s,"
                        "  beta %+6.1f deg (%+5.2f m/s)\n",
                        bankDegrees,
                        flightFrame.noseHeadingRadians
                            * 180.0 / 3.14159265358979,
                        headingTotal * 180.0 / 3.14159265358979,
                        turnRate,
                        flightFrame.sideslipRadians
                            * 180.0 / 3.14159265358979,
                        flightFrame.spanwiseSpeed);
            std::printf("           half alpha %+5.1f/%+5.1f deg,"
                        "  q ratio %.2f/%.2f,  brake line %.1f/%.1f cm,"
                        "  requested dL/dD %+.0f/%+.0f N,"
                        "  achieved %+.0f/%+.0f N\n",
                        sim.alphaHalfDeviationRadians[0]
                            * 180.0 / 3.14159265358979,
                        sim.alphaHalfDeviationRadians[1]
                            * 180.0 / 3.14159265358979,
                        sim.halfDynamicPressureRatio[0],
                        sim.halfDynamicPressureRatio[1],
                        100.0 * sim.brakeApplied[0],
                        100.0 * sim.brakeApplied[1],
                        sim.pressureSolve.requestedHalfDifference[0],
                        sim.pressureSolve.requestedHalfDifference[1],
                        sim.pressureSolve.achievedHalfDifference[0],
                        sim.pressureSolve.achievedHalfDifference[1]);
        }
        return 0;
    }

    // Does the pilot actually swing? Settle the system, then haul both
    // brakes and watch where the pilot goes relative to the canopy. A
    // pendulum shows up as an overshoot and a return; a rigid attachment
    // shows up as a step.
    if (options.swing) {
        if (sim.pilotNode == pg::noConstraint) {
            std::fprintf(stderr, "This mesh has no suspension lines.\n");
            return 1;
        }
        const auto canopyCentre = [&sim] {
            softwing::Vec3 centre;
            const auto &nodes = sim.body->nodes();
            const std::size_t counted = std::min(sim.canopyNodeCount,
                                                 nodes.size());
            for (std::size_t index = 0; index < counted; ++index) {
                centre += nodes[index].position;
            }
            return counted > 0 ? centre / static_cast<double>(counted)
                               : centre;
        };
        std::printf("pilot mass      %.1f kg (explicit point payload)\n",
                    sim.pilotMass);
        std::printf("launch          %s\n",
                    pg::launchModeName(controls.launchMode));
        std::printf("\n  frame   brake     pilot fore/aft   pilot below\n");
        for (int frame = 0; frame < 240; ++frame) {
            // 90 frames (1.5 s) to settle, then both brakes to 40 cm.
            controls.brakeLeft = controls.brakeRight = frame < 90 ? 0.0 : 0.40;
            pg::stepSimulation(sim, controls);
            if (frame % 10 == 9) {
                const softwing::Vec3 pilot =
                    sim.body->nodes()[sim.pilotNode].position;
                const softwing::Vec3 centre = canopyCentre();
                std::printf("  %5d   %.2f m    %+8.3f m      %8.3f m\n",
                            frame + 1,
                            controls.brakeLeft,
                            dot(pilot - centre, sim.restChordDirection),
                            centre.z - pilot.z);
            }
        }
        return 0;
    }

    if (options.gpu) {
        // Settle the wing on the CPU first so both backends are timed on a
        // representative pose rather than on the rest shape, then hand that
        // pose to the GPU and let it carry on from there.
        pg::GpuSoftBody gpu;
        if (!gpu.initialize(sim, options.gpuMode, error)) {
            std::fprintf(stderr,
                         "GPU backend unavailable: %s\n",
                         error.toUtf8().constData());
            return 1;
        }
        for (int frame = 0; frame < options.warmup; ++frame) {
            gpu.step(sim, controls);
        }
        const auto start = std::chrono::steady_clock::now();
        for (int frame = 0; frame < options.frames; ++frame) {
            gpu.step(sim, controls);
        }
        const double msPerFrame =
            std::chrono::duration<double>(std::chrono::steady_clock::now()
                                          - start)
                .count()
            * 1000.0 / static_cast<double>(options.frames);
        const auto readbackStart = std::chrono::steady_clock::now();
        gpu.readback(sim);
        const double readbackMs =
            std::chrono::duration<double>(std::chrono::steady_clock::now()
                                          - readbackStart)
                .count()
            * 1000.0;

        if (options.csv) {
            std::printf("%s,%d,%d,gpu,%d,%d,%zu,%zu,%zu,%.4f\n",
                        options.meshPath.c_str(),
                        options.subdivision,
                        options.detailedRibs ? 1 : 0,
                        options.substeps,
                        options.iterations,
                        nodeCount,
                        constraintCount,
                        triangleCount,
                        msPerFrame);
            return 0;
        }
        std::printf("mesh            %s\n", options.meshPath.c_str());
        std::printf("device          %s\n",
                    gpu.rendererDescription().toUtf8().constData());
        std::printf("body            %zu nodes, %zu triangles, "
                    "%zu distance/cable constraints\n",
                    nodeCount,
                    triangleCount,
                    constraintCount);
        std::printf("step            %d substeps x %d iterations, float32, "
                    "%s\n",
                    options.substeps,
                    options.iterations,
                    options.gpuMode == pg::GpuSolveMode::Jacobi
                        ? "Jacobi"
                        : "coloured Gauss-Seidel");
        std::printf("dispatches      %zu per frame over %zu colours\n",
                    gpu.dispatchesPerFrame(controls),
                    gpu.colourCount());
        std::printf("\n");
        std::printf("  %-26s %9.3f ms/frame  (%.1f fps)\n",
                    "wall clock (glFinish)",
                    msPerFrame,
                    msPerFrame > 0.0 ? 1000.0 / msPerFrame : 0.0);
        std::printf("  %-26s %9.3f ms\n", "pose readback", readbackMs);

        // Speed means nothing if the wing is not being held. Step an
        // identical body on the CPU for the same number of frames from the
        // same start and compare what the two solvers converged to.
        pg::SimBody reference = pg::buildSimBody(refined, build, controls);
        for (int frame = 0; frame < options.warmup + options.frames;
             ++frame) {
            pg::stepSimulation(reference, controls);
        }
        gpu.readback(sim);
        std::printf("\n");
        std::printf("  %-26s %9.4f m^3 (GPU)   %9.4f m^3 (CPU)\n",
                    "enclosed volume",
                    enclosedVolume(sim),
                    enclosedVolume(reference));
        std::printf("  %-26s %9.4f m     (GPU)   %9.4f m     (CPU)\n",
                    "span extent",
                    spanExtent(sim),
                    spanExtent(reference));
        double worst = 0.0;
        for (std::size_t index = 0; index < nodeCount; ++index) {
            worst = std::max(
                worst,
                length(sim.body->nodes()[index].position
                       - reference.body->nodes()[index].position));
        }
        std::printf("  %-26s %9.4f m\n", "worst node disagreement", worst);
        return 0;
    }

    try {
        for (int frame = 0; frame < options.warmup; ++frame) {
            pg::stepSimulation(sim, controls);
        }

        softwing::StepPerformanceProfile profile;
        controls.performanceProfile = &profile;
        const auto start = std::chrono::steady_clock::now();
        for (int frame = 0; frame < options.frames; ++frame) {
            pg::stepSimulation(sim, controls);
        }
        const double seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now()
                                          - start)
                .count();
        controls.performanceProfile = nullptr;

        const double msPerFrame =
            seconds * 1000.0 / static_cast<double>(options.frames);
        if (options.csv) {
            std::printf(
                "%s,%d,%d,%u,%d,%d,%zu,%zu,%zu,%.4f,%.4f,%.4f,%.4f,%.4f,"
                "%.6f,%.6f\n",
                options.meshPath.c_str(),
                options.subdivision,
                options.detailedRibs ? 1 : 0,
                options.threads,
                options.substeps,
                options.iterations,
                nodeCount,
                constraintCount,
                triangleCount,
                msPerFrame,
                millisecondsOf(profile.predictionNanoseconds, options.frames),
                millisecondsOf(profile.distanceConstraintNanoseconds,
                               options.frames),
                millisecondsOf(profile.finalizationNanoseconds,
                               options.frames),
                millisecondsOf(profile.softBodyTotalNanoseconds,
                               options.frames),
                enclosedVolume(sim),
                spanExtent(sim));
            return 0;
        }

        std::printf("mesh            %s\n", options.meshPath.c_str());
        std::printf(
            "build           subdiv %d, %s ribs, %.2f s\n",
            options.subdivision,
            options.detailedRibs ? "detailed" : "simple",
            buildSeconds);
        std::printf("body            %zu nodes, %zu triangles, "
                    "%zu distance/cable constraints\n",
                    nodeCount,
                    triangleCount,
                    constraintCount);
        std::printf("step            %d substeps x %d iterations, "
                    "%u worker threads\n",
                    options.substeps,
                    options.iterations,
                    options.threads);
        const softwing::ConstraintColouringReport colouring =
            sim.body->constraintColouringReport();
        std::printf("colouring       %zu colours, %zu run in parallel "
                    "(largest %zu), %zu constraints left serial (%.1f%%)\n",
                    colouring.colourCount,
                    colouring.parallelColours,
                    colouring.largestColour,
                    colouring.serialConstraints,
                    constraintCount == 0
                        ? 0.0
                        : 100.0 * static_cast<double>(
                                      colouring.serialConstraints)
                              / static_cast<double>(constraintCount));
        std::printf("constraint work %.2f M solves/frame\n",
                    static_cast<double>(profile.distanceConstraintVisits)
                        / static_cast<double>(options.frames) / 1.0e6);
        std::printf("pose checksum   %016llx\n",
                    static_cast<unsigned long long>(
                        poseChecksum(sim.body->nodes())));
        std::printf("\n");
        std::printf("  %-26s %9.3f ms/frame  (%.1f fps)\n",
                    "wall clock",
                    msPerFrame,
                    msPerFrame > 0.0 ? 1000.0 / msPerFrame : 0.0);
        const std::uint64_t total = profile.softBodyTotalNanoseconds;
        reportLine("solver total", total, total, options.frames);
        reportLine("  prediction + pressure",
                   profile.predictionNanoseconds,
                   total,
                   options.frames);
        reportLine("  distance constraints",
                   profile.distanceConstraintNanoseconds,
                   total,
                   options.frames);
        reportLine("  load-path sweeps",
                   profile.cableConstraintNanoseconds,
                   total,
                   options.frames);
        reportLine("  membrane constraints",
                   profile.membraneConstraintNanoseconds,
                   total,
                   options.frames);
        reportLine("  dihedral hinges",
                   profile.bendingConstraintNanoseconds,
                   total,
                   options.frames);
        reportLine("  membrane diagnostics",
                   profile.membraneDiagnosticsNanoseconds,
                   total,
                   options.frames);
        reportLine("  finalization",
                   profile.finalizationNanoseconds,
                   total,
                   options.frames);
        std::printf("\n");
        std::printf("  %-26s %9.4f m^3 (designed %.4f, %+.1f%%)\n",
                    "enclosed volume",
                    enclosedVolume(sim),
                    designVolume,
                    designVolume > 0.0
                        ? 100.0 * (enclosedVolume(sim) - designVolume)
                              / designVolume
                        : 0.0);
        std::printf("  %-26s %9.4f m\n", "span extent", spanExtent(sim));
        const pg::AeroSummary aero = pg::aerodynamicSummary(sim, controls);
        std::printf("  %-26s %9.1f N up, %.1f N fore/aft\n",
                    "aerodynamic load",
                    aero.force.z,
                    aero.force.y);
        std::printf("  %-26s %9.1f N lift, %.1f N drag  ->  L/D %.2f\n",
                    "resolved to the airflow",
                    aero.lift,
                    aero.drag,
                    aero.glideRatio);
        std::printf("  %-26s %9.2f m^2 planform, aspect ratio %.2f\n",
                    "wing",
                    sim.planformArea,
                    sim.aspectRatio);
        if (controls.fabricContact) {
            const pg::PlaygroundContactStats &contact = sim.contact.stats;
            std::printf(
                "  %-26s %9zu vertex/tri, %zu edge/edge, %zu line/tri\n",
                "contact candidates",
                contact.vertexTriangleCandidates,
                contact.edgeEdgeCandidates,
                contact.segmentTriangleCandidates);
            std::printf(
                "  %-26s %9zu active, %zu refreshes, coverage %s\n",
                "contact projection",
                contact.activeContacts,
                contact.substepRefreshes,
                contact.coverageComplete ? "complete" : "INCOMPLETE");
            std::printf(
                "  %-26s %9zu tests, %zu excluded, %zu large, %zu hits\n",
                "contact broad phase",
                contact.broadPhaseTests,
                contact.topologyExcludedPairs,
                contact.largeSweptEnvelopes,
                contact.broadPhaseBudgetHits
                    + contact.candidateBudgetHits);
            std::printf(
                "  %-26s %9.3f -> %.3f mm worst before/after\n",
                "contact penetration",
                1000.0 * contact.worstPenetrationBefore,
                1000.0 * contact.worstPenetrationAfter);
        }
    } catch (const std::exception &exception) {
        std::fprintf(stderr, "Solver failed: %s\n", exception.what());
        return 1;
    }
    return 0;
}
