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
//                  [--substeps N] [--iterations N] [--csv]
//                  [--shape [SECONDS]] [--shape-sweep FROM:TO:STEP]
//                  [--no-flight-load]

#include "../src/gui/playground_metrics.h"
#include "../src/gui/playground_sim.h"
#include "softwing_gpu.h"

#include <QByteArray>
#include <QFile>
#include <QGuiApplication>
#include <QString>
#include <QStringList>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
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
    double pressurePascal = 80.0;
    double angleOfAttackDegrees = 6.0;
    bool swing = false;
    bool polar = false;
    double brakeMetres = 0.0;
    int glideFrames = 0;
    bool freeFlight = false;
    bool shape = false;
    // Settle budget for --shape, and for each sweep point when --shape is
    // given alongside --shape-sweep.
    double shapeSeconds = 6.0;
    bool noFlightLoad = false;
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
        } else if (argument == "--swing") {
            options.swing = true;
            options.freeFlight = true;
        } else if (argument == "--polar") {
            options.polar = true;
        } else if (argument == "--brake") {
            int centimetres = 0;
            if (!value(centimetres)) return false;
            options.brakeMetres = centimetres / 100.0;
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

}  // namespace

int main(int argc, char **argv)
{
    Options options;
    if (!parseOptions(argc, argv, options)) {
        std::fprintf(stderr,
                     "usage: softwing-bench <lep-sim.json> [--subdiv N] "
                     "[--detailed-ribs] [--frames N] [--warmup N] "
                     "[--threads N] [--substeps N] [--iterations N] "
                     "[--gpu|--gpu-jacobi] [--csv] "
                     "[--shape [SECONDS]] [--shape-sweep FROM:TO:STEP] "
                     "[--no-flight-load]\n");
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
    controls.workerThreads = options.threads;
    controls.pressurePascal = options.pressurePascal;
    controls.angleOfAttackDegrees = options.angleOfAttackDegrees;
    controls.freeFlight = options.freeFlight;
    // The shape modes load the tunnel like flight by default (a tunnel
    // carrying only the pressure field's own resultant under-reads every
    // line load — see docs/playground-shape-analysis.md), and hold any
    // --brake pull from the first step. Every other mode keeps flightLoad
    // false, which is what keeps the timing baselines and pose checksums
    // bit-identical to runs that predate these flags.
    if (options.shape || options.shapeSweep) {
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

    if (options.shape || options.shapeSweep) {
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
        }
        return 0;
    }

    // A single wind-tunnel measurement: settle at the current controls,
    // then print the full instrument report (or its CSV row, for scripts).
    if (options.shape) {
        const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
        const pg::SettleResult result = pg::settleAndMeasure(
            sim, controls, baseline, options.shapeSeconds);
        if (options.csv) {
            printCsvLine(pg::shapeReportCsvHeader());
            printCsvLine(pg::shapeReportCsvRow(result.report));
        } else {
            printShapeReport(controls, result, options.meshPath);
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
        std::printf("system          %.2f m^2 planform, AR %.2f\n\n",
                    sim.planformArea,
                    sim.aspectRatio);
        std::printf("   time    airspeed   alpha     L/D    fwd m/s"
                    "   sink m/s   span m   volume    pilot below\n");
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
            // trim settles first and the brake response is legible.
            controls.brakeLeft = controls.brakeRight =
                frame >= 120 ? options.brakeMetres : 0.0;
            pg::stepSimulation(sim, controls);
            // Finer cadence over the first second, where launch transients
            // live.
            if (frame < 120 ? frame % 6 != 5 : frame % 60 != 59) {
                continue;
            }
            const softwing::Vec3 velocity = systemVelocity();
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
            const double forward = dot(velocity, sim.restChordDirection);
            std::printf("  %5.1fs   %7.2f   %+6.2f  %6.2f    %+6.2f"
                        "    %+6.2f    %6.2f    %+5.1f%%     %6.2f m"
                        "   [L %5.0f N, Pz %5.0f N]\n",
                        (frame + 1) / 60.0,
                        sim.lastAirspeed,
                        sim.lastAlphaDegrees,
                        sim.lastGlideRatio,
                        forward,
                        velocity.z,
                        spanExtent(sim),
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
            std::printf("           residual F (%.0f %.0f %.0f) N,"
                        "  pitch M %.0f N.m, chord pitch %+.1f deg,"
                        "  LE dp %.0f Pa, TE dp %.0f Pa\n",
                        sim.lastForceResidual.x,
                        sim.lastForceResidual.y,
                        sim.lastForceResidual.z,
                        sim.lastPitchResidual,
                        pitchOf(trailingMean - leadingMean),
                        noseCount > 0 ? nosePressure / noseCount : 0.0,
                        tailCount > 0 ? tailPressure / tailCount : 0.0);
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
            for (std::size_t node = 0; node < sim.skinTriangleCount; ++node) {
                static_cast<void>(node);
            }
            const auto &nodes = sim.body->nodes();
            std::size_t counted = 0;
            for (const softwing::Node &node : nodes) {
                centre += node.position;
                ++counted;
            }
            return counted > 0 ? centre / static_cast<double>(counted)
                               : centre;
        };
        std::printf("pilot mass      %.1f kg (wing trimmed to its own lift)\n",
                    sim.pilotMass);
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
        reportLine("  membrane constraints",
                   profile.membraneConstraintNanoseconds,
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
    } catch (const std::exception &exception) {
        std::fprintf(stderr, "Solver failed: %s\n", exception.what());
        return 1;
    }
    return 0;
}
