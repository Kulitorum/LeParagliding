// Unit tests for the Playground's shape instrumentation
// (playground_metrics.{h,cpp}) and the grab API, on a small synthetic
// wing built in code: three octagonal rib sections joined by skin quads,
// closed at the tips, with a two-row line cascade per side. Small enough
// to reason about exactly, rich enough to exercise the mirror pairing,
// the row grouping and the section fits.

#include "playground_metrics.h"

#include <QString>
#include <QStringList>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

namespace pg = lep::playground;
using softwing::Vec3;

namespace {

int failures = 0;

void check(bool condition, const char *what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegrees = kPi / 180.0;

Vec3 rotatedAboutX(const Vec3 &value, const Vec3 &centre,
                   double angleRadians)
{
    const Vec3 offset = value - centre;
    const double c = std::cos(angleRadians);
    const double s = std::sin(angleRadians);
    return {centre.x + offset.x,
            centre.y + offset.y * c - offset.z * s,
            centre.z + offset.y * s + offset.z * c};
}

Vec3 rotatedAboutZ(const Vec3 &value, double angleRadians)
{
    const double c = std::cos(angleRadians);
    const double s = std::sin(angleRadians);
    return {value.x * c - value.y * s, value.x * s + value.y * c,
            value.z};
}

// Three 8-node sections at x = -1, 0, +1 (chord 0.8 m along +y, LE at
// y = 0), skin quads joining adjacent sections around the profile, and
// caps closing the tips: an open tube's signed volume is not
// translation-invariant, and the rigid-motion test depends on that
// invariance. Lines: per side an A cascade (plan 1) off the nose
// underside and a B cascade (plan 2) further aft, both through a mid
// junction down to one low carabiner point.
pg::SimMesh testMesh()
{
    pg::SimMesh mesh;
    const double sections[3] = {-1.0, 0.0, 1.0};
    for (const double x : sections) {
        for (int k = 0; k < 8; ++k) {
            const double theta = 2.0 * kPi * k / 8.0;
            mesh.nodes.push_back({x,
                                  0.4 - 0.4 * std::cos(theta),
                                  0.08 * std::sin(theta)});
        }
    }
    const auto node = [](int s, int k) { return s * 8 + (k % 8); };
    for (int s = 0; s < 2; ++s) {
        for (int k = 0; k < 8; ++k) {
            mesh.quads.push_back({node(s, k), node(s, k + 1),
                                  node(s + 1, k + 1), node(s + 1, k)});
            mesh.quadSurfaces.push_back(k < 4 ? pg::SimSurface::Extrados
                                              : pg::SimSurface::Intrados);
        }
    }
    for (const int s : {0, 2}) {
        for (const std::array<int, 4> corners :
             {std::array<int, 4>{0, 1, 2, 3},
              std::array<int, 4>{3, 4, 5, 6},
              std::array<int, 4>{6, 7, 0, 3}}) {
            mesh.quads.push_back(
                {node(s, corners[0]), node(s, corners[1]),
                 node(s, corners[2]), node(s, corners[3])});
            mesh.quadSurfaces.push_back(pg::SimSurface::Intrados);
        }
    }
    for (int s = 0; s < 3; ++s) {
        std::vector<int> loop;
        for (int k = 0; k < 8; ++k) {
            loop.push_back(node(s, k));
        }
        mesh.ribLoops.push_back(std::move(loop));
    }
    mesh.ribHoles.resize(mesh.ribLoops.size());
    for (const int s : {0, 2}) {
        const double side = s == 0 ? -1.0 : 1.0;
        const Vec3 carabiner{0.25 * side, 0.4, -3.0};
        // Line tops 2 cm under the skin node, so the attachment
        // constraint gets a nonzero rest length.
        const Vec3 attachA =
            mesh.nodes[node(s, 7)] + Vec3{0.0, 0.0, -0.02};
        const Vec3 midA{0.7 * side, 0.2, -1.6};
        const Vec3 attachB =
            mesh.nodes[node(s, 5)] + Vec3{0.0, 0.0, -0.02};
        const Vec3 midB{0.7 * side, 0.55, -1.6};
        mesh.lines.push_back({attachA, midA, false, 1});
        mesh.lines.push_back({midA, carabiner, false, 1});
        mesh.lines.push_back({attachB, midB, false, 2});
        mesh.lines.push_back({midB, carabiner, false, 2});
    }
    return mesh;
}

// Pinned, no airflow: the shape tests pose the body by hand and only
// measure, so nothing should be loading the fabric.
pg::SimControls pinnedControls()
{
    pg::SimControls controls;
    controls.freeFlight = false;
    controls.flightLoad = false;
    controls.pressurePascal = 0.0;
    return controls;
}

bool hasFlag(const pg::ShapeReport &report, pg::ShapeFlag flag)
{
    return std::any_of(report.flags.begin(), report.flags.end(),
                       [flag](const pg::ShapeFlagInfo &info) {
                           return info.flag == flag;
                       });
}

void testFreshBody(const pg::SimMesh &mesh)
{
    const pg::SimControls controls = pinnedControls();
    const pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
    check(sim.ribChords.size() == 3, "fresh: three ribs");
    check(sim.ribLoopNodes.size() == sim.ribChords.size(),
          "fresh: rib loops parallel to chords");
    check(baseline.mirrorRib.size() == 3
              && baseline.mirrorRib[0] == 2 && baseline.mirrorRib[1] == 1
              && baseline.mirrorRib[2] == 0,
          "fresh: mirror pairing across the centre rib");

    const pg::ShapeReport report =
        pg::measureShape(sim, controls, baseline);
    check(std::abs(report.spanRatio - 1.0) < 1e-9, "fresh: span ratio 1");
    check(std::abs(report.areaRatio - 1.0) < 1e-9, "fresh: area ratio 1");
    check(std::abs(report.volumeRatio - 1.0) < 1e-9,
          "fresh: volume ratio 1");
    check(report.slackFraction == 0.0, "fresh: nothing slack");
    check(report.asymmetryMetres < 1e-9, "fresh: symmetric");
    check(report.agitationMetresPerSecond == 0.0, "fresh: at rest");
    for (const pg::RibShape &rib : report.ribs) {
        check(rib.rmsMetres < 1e-12, "fresh: section fit exact");
        check(std::abs(rib.chordRatio - 1.0) < 1e-12,
              "fresh: chord ratio 1");
        check(std::abs(rib.twistDegrees) < 1e-9, "fresh: no twist");
        check(rib.leadingEdgeDentMetres == 0.0, "fresh: no dent");
    }
    check(report.flags.empty(), "fresh: no flags");
    check(report.rows.size() == 2, "fresh: rows A and B");
    if (report.rows.size() == 2) {
        check(report.rows[0].row == QLatin1Char('A')
                  && report.rows[0].segments == 2,
              "fresh: row A has one riser segment per side");
        check(report.rows[1].row == QLatin1Char('B')
                  && report.rows[1].segments == 2,
              "fresh: row B has one riser segment per side");
    }
    check(report.lineLoadNewtons == 0.0 && report.slackRiserSegments == 4,
          "fresh: unloaded risers read slack");
    check(report.liftNewtons == 0.0 && report.dragNewtons == 0.0,
          "fresh: no polar numbers without flight load");
}

void testRigidMotion(const pg::SimMesh &mesh)
{
    const pg::SimControls controls = pinnedControls();
    pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
    const Vec3 shift{0.3, -0.2, 0.5};
    for (softwing::Node &node : sim.body->nodes()) {
        node.position =
            rotatedAboutZ(node.position, 10.0 * kDegrees) + shift;
        node.previousPosition = node.position;
    }
    const pg::ShapeReport report =
        pg::measureShape(sim, controls, baseline);
    check(std::abs(report.spanRatio - 1.0) < 1e-9,
          "rigid: span ratio unchanged");
    check(std::abs(report.areaRatio - 1.0) < 1e-9,
          "rigid: area ratio unchanged");
    check(std::abs(report.volumeRatio - 1.0) < 1e-9,
          "rigid: volume ratio unchanged");
    check(report.slackFraction == 0.0, "rigid: nothing slack");
    check(report.asymmetryMetres < 1e-9, "rigid: still symmetric");
    for (const pg::RibShape &rib : report.ribs) {
        check(rib.rmsMetres < 1e-9, "rigid: alignment absorbs motion");
        check(std::abs(rib.chordRatio - 1.0) < 1e-12,
              "rigid: chords unchanged");
        check(std::abs(rib.twistDegrees) < 1e-6, "rigid: no twist");
        check(rib.leadingEdgeDentMetres < 1e-9, "rigid: no dent");
    }
    check(report.worstDeviationMetres < 1e-9,
          "rigid: no worst deviation");
}

void testUniformInflation(const pg::SimMesh &mesh)
{
    const pg::SimControls controls = pinnedControls();
    pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
    auto &nodes = sim.body->nodes();
    Vec3 centroid;
    for (std::size_t index = 0; index < sim.canopyNodeCount; ++index) {
        centroid += nodes[index].position;
    }
    centroid /= static_cast<double>(sim.canopyNodeCount);
    for (softwing::Node &node : nodes) {
        node.position = centroid + 1.03 * (node.position - centroid);
        node.previousPosition = node.position;
    }
    const pg::ShapeReport report =
        pg::measureShape(sim, controls, baseline);
    for (const pg::RibShape &rib : report.ribs) {
        check(rib.rmsMetres > 1e-3,
              "inflate: scaling is not rigid, sections show residual");
        check(std::abs(rib.chordRatio - 1.03) < 1e-9,
              "inflate: chords stretched 3%");
    }
    check(std::abs(report.volumeRatio - 1.03 * 1.03 * 1.03) < 1e-9,
          "inflate: volume grows by the cube");
    check(std::abs(report.spanRatio - 1.03) < 1e-9,
          "inflate: span stretched 3%");
}

void testTwistSign(const pg::SimMesh &mesh)
{
    const pg::SimControls controls = pinnedControls();
    pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
    auto &nodes = sim.body->nodes();
    // Centre rib, nodes 8..15. Nose-up means the leading edge (low y)
    // rises and the trailing edge falls: with span +x, chord +y, up +z
    // that is a NEGATIVE right-hand rotation about +x. If this test
    // starts failing with ~-3 deg, the measurement's sign flipped.
    Vec3 centre;
    for (std::size_t index = 8; index < 16; ++index) {
        centre += nodes[index].position;
    }
    centre /= 8.0;
    for (std::size_t index = 8; index < 16; ++index) {
        nodes[index].position = rotatedAboutX(
            nodes[index].position, centre, -3.0 * kDegrees);
        nodes[index].previousPosition = nodes[index].position;
    }
    const pg::ShapeReport report =
        pg::measureShape(sim, controls, baseline);
    check(report.ribs.size() == 3, "twist: three ribs");
    if (report.ribs.size() == 3) {
        const double target = report.ribs[1].twistDegrees;
        const double neighbours = 0.5
                                  * (report.ribs[0].twistDegrees
                                     + report.ribs[2].twistDegrees);
        // The global fit absorbs the share of the rotation the moved
        // nodes carry (a third here), so the absolute value reads low;
        // the moved-minus-unmoved difference recovers the full +3.
        check(target > 1.5, "twist: nose-up reads positive");
        check(std::abs(target - neighbours - 3.0) < 0.1,
              "twist: +3 deg nose-up recovered relative to the wing");
        check(std::abs(report.ribs[1].chordRatio - 1.0) < 1e-9,
              "twist: rotation leaves the chord length alone");
    }
}

void testLeadingEdgeDent(const pg::SimMesh &mesh)
{
    const pg::SimControls controls = pinnedControls();
    pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
    // The centre section's leading-edge node, pushed 250 mm inward along
    // its rest normal — a fold-scale excursion, comfortably past the
    // collapse-calibrated flagLeadingEdgeDentMetres, so the flag check is
    // not riding the threshold edge.
    const std::size_t nose = 8;
    const Vec3 normal = baseline.restNormals[nose];
    check(std::abs(length(normal) - 1.0) < 1e-9,
          "dent: nose has a unit rest normal");
    auto &nodes = sim.body->nodes();
    nodes[nose].position -= 0.25 * normal;
    nodes[nose].previousPosition = nodes[nose].position;
    const pg::ShapeReport report =
        pg::measureShape(sim, controls, baseline);
    // The moved node shifts the global fit a little, so the read dent
    // sits somewhat under the push.
    check(std::abs(report.worstLeadingEdgeDentMetres - 0.25) < 0.05,
          "dent: ~250 mm recovered");
    check(report.worstLeadingEdgeDentRib <= 1,
          "dent: reported at the nose it was pushed into");
    check(hasFlag(report, pg::ShapeFlag::FrontTuckRisk),
          "dent: FrontTuckRisk flag raised");
}

void testTensionReadout()
{
    // A 1 kg bob on a stiff cable under gravity: the accumulated
    // multiplier must read back as the bob's weight.
    pg::SimBody sim;
    sim.body = std::make_unique<softwing::SoftBody>();
    sim.body->addFixedNode({0.0, 0.0, 0.0});
    sim.body->addNode({0.0, 0.0, -1.0}, 1.0);
    const std::size_t cable =
        sim.body->addCableConstraint(0, 1, 1.0, 1.0e-9);
    softwing::StepSettings settings;
    settings.timeStep = pg::simulationTimeStep;
    settings.substeps = pg::simulationSubsteps;
    settings.constraintIterations = pg::simulationIterations;
    settings.gravity = {0.0, 0.0, -pg::gravityMetresPerSecondSquared};
    for (int frame = 0; frame < 300; ++frame) {
        sim.body->step(settings);
    }
    const pg::SimControls controls;   // default substeps match settings
    const double tension =
        pg::constraintTensionNewtons(sim, controls, cable);
    check(std::abs(tension - pg::gravityMetresPerSecondSquared)
              < 0.05 * pg::gravityMetresPerSecondSquared,
          "tension: hanging weight within 5%");
    check(pg::constraintTensionNewtons(sim, controls, 999) == 0.0,
          "tension: out-of-range constraint reads zero");
}

void testGrab(const pg::SimMesh &mesh)
{
    // Direction matters here. Cables are one-sided and at q = 0 nothing
    // tensions the cascade, so pulling a junction DOWN just translates
    // the slack system after it — zero force is the physically correct
    // answer for that. Pulling it UP stretches the cable below it against
    // the FIXED carabiner, which resists regardless of any aerodynamic
    // state: the deterministic structural rig this test wants.
    const pg::SimControls controls = pinnedControls();
    pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    // A mid-cascade junction: a line segment endpoint that is neither a
    // carabiner nor on the canopy, sitting at the cascade's mid level.
    std::size_t junction = pg::noConstraint;
    for (const pg::LineSegment &segment : sim.lineSegments) {
        for (const std::size_t node : {segment.a, segment.b}) {
            if (node < sim.canopyNodeCount
                || std::find(sim.carabinerNodes.begin(),
                             sim.carabinerNodes.end(), node)
                       != sim.carabinerNodes.end()) {
                continue;
            }
            const double z = sim.body->nodes()[node].position.z;
            if (z < -1.0 && z > -2.5) {
                junction = node;
                break;
            }
        }
        if (junction != pg::noConstraint) {
            break;
        }
    }
    check(junction != pg::noConstraint, "grab: found a mid junction");
    if (junction == pg::noConstraint) {
        return;
    }
    check(!pg::grabActive(sim), "grab: inactive before beginGrab");
    check(pg::grabForceNewtons(sim, controls) == 0.0,
          "grab: no force before beginGrab");
    check(pg::beginGrab(sim, junction), "grab: beginGrab accepts");
    check(pg::grabActive(sim), "grab: active after beginGrab");
    const Vec3 target =
        sim.body->nodes()[junction].position + Vec3{0.0, 0.0, 0.15};
    pg::moveGrab(sim, target);
    for (int frame = 0; frame < 120; ++frame) {
        pg::stepSimulation(sim, controls);
    }
    const double pull = pg::grabForceNewtons(sim, controls);
    check(std::isfinite(pull), "grab: force is finite");
    check(pull > 1.0, "grab: pulling 15 cm develops real force");
    pg::endGrab(sim);
    check(!pg::grabActive(sim), "grab: inactive after endGrab");
    for (int frame = 0; frame < 60; ++frame) {
        pg::stepSimulation(sim, controls);
    }
    check(pg::grabForceNewtons(sim, controls) == 0.0,
          "grab: no force after endGrab");
    check(pg::beginGrab(sim, sim.body->nodes().size() + 5) == false,
          "grab: out-of-range junction rejected");
}

void testCsv(const pg::SimMesh &mesh)
{
    const pg::SimControls controls = pinnedControls();
    const pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    const pg::ShapeBaseline baseline = pg::captureShapeBaseline(sim);
    pg::ShapeReport report = pg::measureShape(sim, controls, baseline);
    report.flags.push_back({pg::ShapeFlag::SlackRow,
                            QStringLiteral("row A slack on left")});
    report.flags.push_back({pg::ShapeFlag::Unsettled,
                            QStringLiteral("agitation 90 mm/s")});
    const QString header = pg::shapeReportCsvHeader();
    const QString row = pg::shapeReportCsvRow(report);
    const auto headerColumns = header.split(QStringLiteral(","));
    const auto rowColumns = row.split(QStringLiteral(","));
    check(headerColumns.size() == rowColumns.size(),
          "csv: header and row column counts match");
    check(row.contains(pg::shapeFlagName(pg::ShapeFlag::SlackRow))
              && row.contains(
                  pg::shapeFlagName(pg::ShapeFlag::Unsettled)),
          "csv: flags serialised by name");
    check(!pg::shapeFlagName(pg::ShapeFlag::FrontTuckRisk).isEmpty(),
          "csv: flag names non-empty");
}

}  // namespace

int main()
{
    const pg::SimMesh mesh = testMesh();
    testFreshBody(mesh);
    testRigidMotion(mesh);
    testUniformInflation(mesh);
    testTwistSign(mesh);
    testLeadingEdgeDent(mesh);
    testTensionReadout();
    testGrab(mesh);
    testCsv(mesh);
    if (failures > 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all playground_metrics checks passed\n");
    return 0;
}
