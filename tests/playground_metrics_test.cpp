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
#include <numeric>
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

void testLineDeduplication()
{
    // Old lep-sim files contain both the captured full-wing segment and the
    // writer's mirrored copy. Endpoint order is immaterial, but an authored
    // row/brake distinction at coincident coordinates is not.
    const QByteArray json = R"json({
        "nodes": [[0,0,0], [1000,0,0], [1000,1000,0], [0,1000,0]],
        "quads": [[0,1,2,3]],
        "lines": [
          {"a":[0,0,0], "b":[0,0,-1000], "plan":1, "brake":0},
          {"a":[0,0,-1000], "b":[0,0,0], "plan":1, "brake":0},
          {"a":[0,0,0], "b":[0,0,-1000], "plan":1, "brake":0},
          {"a":[0,0,0], "b":[0,0,-1000], "plan":2, "brake":0},
          {"a":[0,0,0], "b":[0,0,-1000], "plan":1, "brake":1}
        ]
      })json";
    QString error;
    const std::optional<pg::SimMesh> parsed = pg::parseSimMesh(json, error);
    check(parsed.has_value(), "dedup: duplicate JSON parses");
    if (!parsed) {
        return;
    }
    check(parsed->lines.size() == 3,
          "dedup: exact and reversed copies become one line");
    check(parsed->duplicateLineCount == 2,
          "dedup: parser reports discarded copies");
    const pg::SimBody parsedBody =
        pg::buildSimBody(*parsed, {}, pinnedControls());
    check(parsedBody.lineSegments.size() == 3,
          "dedup: one cable per distinct semantic line");
    check(parsedBody.duplicateLineCount == 2,
          "dedup: parser diagnostic reaches built body");
    check(std::abs(parsedBody.tunnelLineSolverBallastKg
                   - 2.0 * pg::tunnelLineJunctionRelaxationMassKg)
              < 1.0e-12,
          "tunnel mass: unique junctions get diagnosed relaxation ballast");
    check(parsedBody.lineJunctionFloorMassKg == 0.0,
          "tunnel mass: physical free-flight floor is inactive when pinned");

    pg::SimMesh direct = testMesh();
    const pg::SimLine original = direct.lines.front();
    direct.lines.push_back(
        {original.b, original.a, original.brake, original.plan});
    const pg::SimBody directBody =
        pg::buildSimBody(direct, {}, pinnedControls());
    check(directBody.lineSegments.size() + 1 == direct.lines.size(),
          "dedup: builder defensively rejects direct duplicate");
    check(directBody.duplicateLineCount == 1,
          "dedup: builder reports direct duplicate");
    double uniqueLineMass = 0.0;
    for (const pg::SimLine &line : testMesh().lines) {
        uniqueLineMass += pg::lineLinearDensityKgPerMetre
                          * length(line.b - line.a);
    }
    check(std::abs(directBody.authoredLineMassKg - uniqueLineMass) < 1.0e-12,
          "dedup: rejected line does not duplicate physical mass");
}

void setConstraintTension(pg::SimBody &sim,
                          const pg::SimControls &controls,
                          std::size_t constraint,
                          double newtons)
{
    const double substepRate =
        controls.substeps / pg::simulationTimeStep;
    sim.body->constraints()[constraint].accumulatedLambda =
        -newtons / (substepRate * substepRate);
}

void testRiserCutAndMass()
{
    pg::SimBody sim;
    sim.body = std::make_unique<softwing::SoftBody>();
    const std::size_t carabiner =
        sim.body->addNode({0.0, 0.0, 0.0},
                          pg::lineJunctionNumericalMassKg);
    const std::size_t upperLeft =
        sim.body->addNode({-0.6, 0.0, 0.8}, 0.10);
    const std::size_t upperRight =
        sim.body->addNode({0.6, 0.0, 0.8}, 0.20);
    const std::size_t pilot =
        sim.body->addNode({0.0, 0.0, -1.0}, 80.0);
    const std::size_t leftCable =
        sim.body->addCableConstraint(carabiner, upperLeft, 1.0);
    const std::size_t rightCable =
        sim.body->addCableConstraint(carabiner, upperRight, 1.0);
    const std::size_t harness =
        sim.body->addDistanceConstraint(carabiner, pilot, 1.0);
    sim.lineSegments = {
        {carabiner, upperLeft, false, leftCable, 1, true},
        {carabiner, upperRight, false, rightCable, 1, true},
        {carabiner, pilot, false, harness, 0, false},
    };
    sim.carabinerNodes = {carabiner};

    const pg::SimControls controls;
    setConstraintTension(sim, controls, leftCable, 5.0);
    setConstraintTension(sim, controls, rightCable, 5.0);
    setConstraintTension(sim, controls, harness, 8.0);
    const pg::LineLoadReport loads = pg::lineLoads(sim, controls);
    // Each branch contributes (±3, 0, 4) N. The canopy reaction is their
    // vector sum: 8 N, not the 10 N scalar sum and not 18 N after counting
    // the harness side of the same internal cut.
    check(std::abs(loads.riserForce.x) < 1.0e-12
              && std::abs(loads.riserForce.z - 8.0) < 1.0e-12,
          "riser: branched reaction is vector-summed");
    check(std::abs(loads.riserNewtons - 8.0) < 1.0e-12,
          "riser: harness reaction is not double-counted");
    check(loads.totalSegments == 2 && loads.slackSegments == 0,
          "riser: only authored suspension segments are diagnosed");
    check(loads.maximumExtensionMetres == 0.0
              && loads.maximumExtensionFraction == 0.0
              && std::abs(loads.maximumTensionNewtons - 5.0) < 1.0e-12,
          "riser: cable solve error and peak tension are diagnosed");

    sim.body->nodes()[upperLeft].position = {-0.66, 0.0, 0.88};
    const pg::LineLoadReport stretched = pg::lineLoads(sim, controls);
    check(std::abs(stretched.maximumExtensionMetres - 0.1) < 1.0e-12
              && std::abs(stretched.maximumExtensionFraction - 0.1)
                     < 1.0e-12,
          "riser: maximum authored-cable length error is measured");

    check(std::abs(pg::simulatedMassKilograms(sim) - 80.3001) < 1.0e-12,
          "mass: actual nodes include the line junction");
}

void testPayloadMassLineMassAndLaunch(const pg::SimMesh &mesh)
{
    pg::SimControls drop;
    drop.freeFlight = true;
    drop.pressurePascal = 80.0;
    drop.pilotMassKg = 72.0;
    drop.launchMode = pg::LaunchMode::DropFromRest;
    pg::SimBody released = pg::buildSimBody(mesh, {}, drop);

    check(std::abs(released.pilotMass - 72.0) < 1.0e-12,
          "payload: explicit pilot/harness mass is used");
    check(released.pilotNode != pg::noConstraint
              && std::abs(released.body->nodes()[released.pilotNode].inverseMass
                          - 1.0 / 72.0)
                     < 1.0e-12,
          "payload: pilot is a dynamic point mass, not a kinematic anchor");

    double expectedLineMass = 0.0;
    for (const pg::SimLine &line : mesh.lines) {
        expectedLineMass += pg::lineLinearDensityKgPerMetre
                            * length(line.b - line.a);
    }
    check(std::abs(released.authoredLineMassKg - expectedLineMass) < 1.0e-12,
          "line mass: authored segment lengths set physical mass");

    std::vector<std::size_t> endpoints;
    for (const pg::LineSegment &segment : released.lineSegments) {
        if (segment.suspension) {
            endpoints.push_back(segment.a);
            endpoints.push_back(segment.b);
        }
    }
    std::sort(endpoints.begin(), endpoints.end());
    endpoints.erase(std::unique(endpoints.begin(), endpoints.end()),
                    endpoints.end());
    const double expectedFloor = endpoints.size()
                                 * pg::lineJunctionNumericalMassKg;
    check(std::abs(released.lineJunctionFloorMassKg - expectedFloor)
              < 1.0e-12,
          "line mass: numerical junction floor is separate");
    check(released.controlNodeFloorMassKg == 0.0,
          "line mass: a mesh without brakes has no generated control mass");
    check(released.tunnelLineSolverBallastKg == 0.0,
          "line mass: free flight carries no tunnel relaxation ballast");
    check(released.virtualAddedAirMassKg > 0.0,
          "added air: free flight carries canopy solver inertia");
    const double solverInertialMass = std::accumulate(
        released.body->nodes().begin(), released.body->nodes().end(), 0.0,
        [](double sum, const softwing::Node &node) {
            return sum + (node.inverseMass > 0.0
                              ? 1.0 / node.inverseMass
                              : 0.0);
        });
    check(std::abs(solverInertialMass
                   - pg::simulatedMassKilograms(released)
                   - released.virtualAddedAirMassKg)
              < 1.0e-10,
          "added air: solver inertia is excluded from physical mass");

    pg::SimControls stillAirDrop = drop;
    stillAirDrop.pressurePascal = 0.0;
    pg::SimBody falling = pg::buildSimBody(mesh, {}, stillAirDrop);
    pg::applyAerodynamicForces(falling, stillAirDrop);
    bool addedAirGravityCancelled = true;
    for (std::size_t node = 0;
         node < falling.virtualAddedAirMassByNode.size(); ++node) {
        const double virtualMass = falling.virtualAddedAirMassByNode[node];
        if (virtualMass <= 0.0) {
            continue;
        }
        const softwing::Vec3 expected{
            0.0, 0.0,
            virtualMass * pg::gravityMetresPerSecondSquared};
        addedAirGravityCancelled =
            addedAirGravityCancelled
            && length(falling.body->nodes()[node].force - expected) < 1.0e-12;
    }
    check(addedAirGravityCancelled,
          "added air: virtual gravity is cancelled before free fall");
    double endpointMass = 0.0;
    for (const std::size_t endpoint : endpoints) {
        endpointMass += 1.0 / released.body->nodes()[endpoint].inverseMass;
    }
    check(std::abs(endpointMass - expectedLineMass - expectedFloor) < 1.0e-12,
          "line mass: half-segment lumps conserve authored mass");

    pg::SimMesh brakedMesh = mesh;
    brakedMesh.lines.front().brake = true;
    const pg::SimBody braked = pg::buildSimBody(brakedMesh, {}, drop);
    check(std::abs(braked.controlNodeFloorMassKg
                   - pg::controlNodeNumericalMassKg)
              < 1.0e-12,
          "line mass: generated brake handle carries only its stated floor");
    check(std::abs(braked.authoredLineMassKg - expectedLineMass) < 1.0e-12,
          "line mass: synthesized brake cable is not double-counted");

    const bool releasedAtRest = std::all_of(
        released.body->nodes().begin(), released.body->nodes().end(),
        [](const softwing::Node &node) {
            return length(node.velocity) < 1.0e-15;
        });
    check(releasedAtRest,
          "launch: drop-from-rest leaves the pre-inflated system at rest");

    pg::SimControls trimmed = drop;
    trimmed.launchMode = pg::LaunchMode::TrimmedGlide;
    trimmed.ambientAirVelocityWorld = {2.5, -1.25, 0.75};
    pg::SimBody flying = pg::buildSimBody(mesh, {}, trimmed);
    const Vec3 referenceFlow = pg::referenceFlowVelocity(flying, trimmed);
    const double referenceSpeed = length(referenceFlow);
    const Vec3 expectedLaunch =
        trimmed.ambientAirVelocityWorld
        - rotatedAboutX(referenceFlow, {}, flying.glideAngleRadians)
              * (referenceSpeed > 0.0
                     ? flying.trimmedLaunchAirspeed / referenceSpeed
                     : 0.0);
    check(length(flying.body->nodes()[flying.pilotNode].velocity
                 - expectedLaunch)
              < 1.0e-12,
          "launch: ground velocity is ambient minus trimmed apparent flow");
    check(flying.trimmedLaunchDynamicPressure > 0.0
              && flying.trimmedLaunchAirspeed > 0.0
              && flying.trimmedLaunchEffectiveLiftCoefficient > 0.0,
          "launch: achieved rest-field calibration is diagnosed");

    std::array<double, 3> mass{30.0, 90.0, 250.0};
    std::array<double, 3> launchSpeed{};
    std::array<double, 3> launchQ{};
    for (std::size_t index = 0; index < mass.size(); ++index) {
        pg::SimControls weighted = trimmed;
        weighted.pilotMassKg = mass[index];
        const pg::SimBody launch = pg::buildSimBody(mesh, {}, weighted);
        launchSpeed[index] = launch.trimmedLaunchAirspeed;
        launchQ[index] = launch.trimmedLaunchDynamicPressure;
        check(std::isfinite(launchSpeed[index])
                  && std::isfinite(launchQ[index]),
              "launch: weight-aware speed and q remain finite");
        const double weight =
            pg::simulatedMassKilograms(launch)
            * pg::gravityMetresPerSecondSquared;
        const double achievedSupport =
            (launch.pressureSolve.achievedForce
             + launch.lastPolarDragTractionForce)
                .z;
        const bool qCapped =
            launchQ[index] >= 4.0 * weighted.pressurePascal - 1.0e-9;
        check(qCapped
                  ? achievedSupport > 0.0 && achievedSupport < weight
                  : std::abs(achievedSupport - weight) / weight < 0.05,
              "launch: achieved support matches weight unless q is capped");
    }
    const bool monotonicLaunch =
        launchSpeed[0] < launchSpeed[1]
        && launchSpeed[1] <= launchSpeed[2]
        && launchQ[0] < launchQ[1] && launchQ[1] <= launchQ[2]
        && (launchQ[1] < launchQ[2]
            || launchQ[2] >= 4.0 * trimmed.pressurePascal - 1.0e-9);
    check(monotonicLaunch,
          "launch: 30/90/250 kg produce monotonic trimmed speed and q");

    pg::SimControls tooLight = drop;
    tooLight.pilotMassKg = -5.0;
    const pg::SimBody minimum = pg::buildSimBody(mesh, {}, tooLight);
    check(std::abs(minimum.pilotMass - pg::minimumPilotMassKg) < 1.0e-12,
          "payload: programmatic mass is clamped at the lower bound");
    pg::SimControls tooHeavy = drop;
    tooHeavy.pilotMassKg = 500.0;
    const pg::SimBody maximum = pg::buildSimBody(mesh, {}, tooHeavy);
    check(std::abs(maximum.pilotMass - pg::maximumPilotMassKg) < 1.0e-12,
          "payload: programmatic mass is clamped at the upper bound");
}

void testDynamicPointPayloadPendulum(const pg::SimMesh &mesh)
{
    pg::SimControls controls;
    controls.freeFlight = true;
    controls.pressurePascal = 0.0;
    controls.pilotMassKg = 70.0;
    controls.launchMode = pg::LaunchMode::DropFromRest;
    pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    check(sim.pilotNode != pg::noConstraint,
          "pendulum: free-flight build has a point payload");
    check(!sim.carabinerNodes.empty(),
          "pendulum: payload has a carabiner-side support");
    if (sim.pilotNode == pg::noConstraint || sim.carabinerNodes.empty()) {
        return;
    }

    // Freeze the canopy/riser side to make the familiar pendulum reference
    // case, but leave the exact payload node and harness constraints created
    // by buildSimBody. Rotate the payload about the common carabiner axis so
    // every harness rest length remains satisfied at release.
    auto &nodes = sim.body->nodes();
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        if (index != sim.pilotNode) {
            sim.body->fixNode(index);
        }
    }
    Vec3 pivot;
    for (const std::size_t carabiner : sim.carabinerNodes) {
        pivot += nodes[carabiner].position;
    }
    pivot /= static_cast<double>(sim.carabinerNodes.size());
    const Vec3 bottom = nodes[sim.pilotNode].position;
    const Vec3 released = rotatedAboutX(bottom, pivot, 20.0 * kDegrees);
    nodes[sim.pilotNode].position = released;
    nodes[sim.pilotNode].previousPosition = released;
    nodes[sim.pilotNode].velocity = {};

    softwing::StepSettings settings;
    settings.timeStep = pg::simulationTimeStep;
    settings.substeps = pg::simulationSubsteps;
    settings.constraintIterations = pg::simulationIterations;
    settings.gravity = {0.0, 0.0, -pg::gravityMetresPerSecondSquared};
    settings.velocityDampingPerSecond = 0.0;
    for (int frame = 0; frame < 5; ++frame) {
        sim.body->step(settings);
    }
    const Vec3 moved = nodes[sim.pilotNode].position;
    check(std::abs(moved.y - pivot.y) < std::abs(released.y - pivot.y),
          "pendulum: gravity moves the released payload toward bottom");
    check(length(nodes[sim.pilotNode].velocity) > 1.0e-4,
          "pendulum: payload acquires dynamic velocity under gravity");
    check(moved.z < released.z,
          "pendulum: payload loses gravitational potential after release");
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

// A pinning tie with a near-zero rest length must never reach the
// strain fields: (length - rest)/rest on it is astronomical however
// healthy the wing, and one such tie once put a 6e15% peak on the
// stress legend.
void testTinyRestEdgeIgnored(const pg::SimMesh &mesh)
{
    const pg::SimControls controls = pinnedControls();
    pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    // Turn a real skin edge into a degenerate pin: rest collapses to a
    // picometre while the live geometry stays put.
    const std::size_t edge = sim.renderFaces.front().edges[0];
    check(edge != pg::noConstraint, "tinyrest: probe edge exists");
    sim.body->constraints()[edge].restLength = 1.0e-12;
    std::vector<float> tensile;
    std::vector<float> slack;
    pg::nodeStrainFields(sim, false, tensile, slack);
    float worst = 0.0F;
    for (const float strain : tensile) {
        worst = std::max(worst, strain);
    }
    // Without the rest-length floor this reads ~1e9.
    check(worst < 1.0F, "tinyrest: degenerate edge excluded from strain");
}

void testBoundedPressureRetrim(const pg::SimMesh &mesh)
{
    pg::SimControls controls;
    controls.pressurePascal = 80.0;
    controls.flightLoad = true;
    controls.cellPressureModel = false;
    pg::SimBody sim = pg::buildSimBody(mesh, {}, controls);
    pg::applyAerodynamicForces(sim, controls);

    const pg::PressureSolveDiagnostics &solve = sim.pressureSolve;
    check(solve.attempted && !solve.legacy,
          "pressure: bounded final-Cp path is default");
    check(solve.valid && !solve.numericalFailure,
          "pressure: bounded projection completes");
    check(sim.faceAppliedExternalCp.size() == sim.skinTriangleCount,
          "pressure: one final Cp per skin face");
    check(solve.minimumCp
              >= pg::minimumExteriorPressureCoefficient - 1.0e-9
              && solve.maximumCp
                     <= pg::maximumExteriorPressureCoefficient + 1.0e-9,
          "pressure: final exterior Cp respects physical bounds");

    std::vector<float> interiorField;
    std::vector<float> exteriorField;
    std::vector<float> fabricField;
    pg::faceInteriorPressureField(sim, interiorField);
    pg::faceExteriorPressureCoefficientField(sim, exteriorField);
    pg::facePressureDifferenceField(sim, fabricField);
    check(interiorField.size() == sim.renderFaces.size()
              && exteriorField.size() == sim.renderFaces.size()
              && fabricField.size() == sim.renderFaces.size(),
          "pressure fields: all display fields follow render-face topology");
    double displayFieldError = 0.0;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        displayFieldError = std::max(
            {displayFieldError,
             std::abs(static_cast<double>(interiorField[face])
                      - sim.faceInteriorPressure[face]),
             std::abs(static_cast<double>(exteriorField[face])
                      - sim.faceAppliedExternalCp[face]),
             std::abs(static_cast<double>(fabricField[face])
                      - sim.body->triangles()[face].pressureDifference)});
    }
    check(displayFieldError < 1.0e-4,
          "pressure fields: internal p, external Cp and fabric delta-p stay distinct");

    pg::SimControls cellControls = controls;
    cellControls.cellPressureModel = true;
    pg::SimBody cellSim = pg::buildSimBody(mesh, {}, cellControls);
    pg::applyAerodynamicForces(cellSim, cellControls);
    std::vector<double> firstCellValue(
        cellSim.cells.size(), std::numeric_limits<double>::quiet_NaN());
    bool cellInteriorUniform = !cellSim.cells.empty();
    for (std::size_t face = 0; face < cellSim.skinTriangleCount; ++face) {
        const std::size_t cell = cellSim.faceAero[face].cell;
        const double value = cellSim.faceInteriorPressure[face];
        if (cell >= firstCellValue.size()) {
            cellInteriorUniform = false;
            continue;
        }
        if (std::isnan(firstCellValue[cell])) {
            firstCellValue[cell] = value;
        } else if (std::abs(firstCellValue[cell] - value) > 1.0e-12) {
            cellInteriorUniform = false;
        }
    }
    check(cellInteriorUniform,
          "pressure fields: internal gauge pressure is uniform within each cell");

    Vec3 integrated;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const double reconstructed =
            sim.faceInteriorPressure[face]
            - sim.faceDynamicPressure[face]
                  * sim.faceAppliedExternalCp[face];
        check(std::abs(reconstructed
                       - sim.body->triangles()[face].pressureDifference)
                  < 1.0e-9,
              "pressure: face load is interior minus q Cp");
        const softwing::Triangle &triangle = sim.body->triangles()[face];
        const auto &nodes = sim.body->nodes();
        integrated +=
            triangle.pressureDifference * 0.5
            * cross(nodes[triangle.b].position - nodes[triangle.a].position,
                    nodes[triangle.c].position - nodes[triangle.a].position);
    }
    check(length(integrated - solve.achievedForce) < 1.0e-8,
          "pressure: achieved-force diagnostic is independently integrated");
    check(length((solve.achievedForce - solve.requestedForce)
                 - solve.forceResidual)
              < 1.0e-9,
          "pressure: force residual remains in physical newtons");
    const pg::WingAeroSample appliedSample =
        pg::sampleWingAero(sim, controls);
    check(appliedSample.valid
              && std::abs(sim.lastLift
                          - dot(sim.lastAeroForce,
                                appliedSample.liftDirection))
                     < 1.0e-8
              && std::abs(sim.lastDrag
                          - dot(sim.lastAeroForce,
                                appliedSample.windDirection))
                     < 1.0e-8,
          "pressure: bounded telemetry resolves the applied force");
    check(std::abs(solve.achievedLiftNewtons
                   - dot(solve.achievedForce,
                         appliedSample.liftDirection))
              < 1.0e-8,
          "pressure: diagnostics separate achieved from requested lift");
    check(length(sim.lastPolarDragTractionForce) < 1.0e-15
              && sim.lastPolarDragTractionNewtons == 0.0,
          "drag: pinned pressure-shape instrument receives no skin traction");

    controls.pressureSolveMode =
        pg::PressureSolveMode::LegacyIncrementClamp;
    pg::SimBody legacy = pg::buildSimBody(mesh, {}, controls);
    pg::applyAerodynamicForces(legacy, controls);
    check(legacy.pressureSolve.legacy
              && !legacy.pressureSolve.attempted,
          "pressure: legacy retrim requires explicit mode");
    check(length(legacy.lastPolarDragTractionForce) < 1.0e-15
              && legacy.lastPolarDragTractionNewtons == 0.0,
          "drag: legacy oracle receives no new skin traction");
    check(sim.faceRetrimPreferredCp.size() == sim.skinTriangleCount
              && legacy.faceAppliedExternalCp.size()
                     == legacy.skinTriangleCount,
          "pressure: preferred and legacy Cp fields are available");
    double worstPriorDifference = 0.0;
    for (std::size_t face = 0;
         face < sim.faceRetrimPreferredCp.size()
             && face < legacy.faceAppliedExternalCp.size(); ++face) {
        worstPriorDifference = std::max(
            worstPriorDifference,
            std::abs(sim.faceRetrimPreferredCp[face]
                     - legacy.faceAppliedExternalCp[face]));
    }
    check(worstPriorDifference < 1.0e-10,
          "pressure: bounded objective prior matches same-frame legacy field");

    pg::SimControls flyingControls = controls;
    flyingControls.freeFlight = true;
    flyingControls.flightLoad = false;
    flyingControls.pressureSolveMode =
        pg::PressureSolveMode::BoundedExteriorCp;
    flyingControls.launchMode = pg::LaunchMode::DropFromRest;
    pg::SimBody flying = pg::buildSimBody(mesh, {}, flyingControls);
    const Vec3 calibrationVelocity =
        flyingControls.ambientAirVelocityWorld
        - pg::referenceFlowVelocity(flying, flyingControls);
    for (softwing::Node &node : flying.body->nodes()) {
        node.velocity = calibrationVelocity;
    }
    pg::applyPressure(flying, flyingControls);
    const Vec3 stampedFreeFlightForce = pg::aerodynamicForce(flying);
    pg::applyAerodynamicForces(flying, flyingControls);
    const pg::WingAeroSample flyingSample =
        pg::sampleWingAero(flying, flyingControls);
    check(length(flying.pressureSolve.requestedForce
                 - stampedFreeFlightForce)
              < 1.0e-8,
          "pressure: free flight preserves the section-pressure lift field");
    check(flyingSample.valid
              && std::abs(flying.lastPolarDragTargetNewtons
                          - flyingSample.dynamicPressure
                                * flying.planformArea
                                * flyingSample.dragCoefficient)
                     < 1.0e-8,
          "drag: traction target is finite-wing polar drag without fabric heuristic");
    const double expectedTraction = std::max(
        0.0,
        flying.lastPolarDragTargetNewtons
            - flying.pressureSolve.achievedDragNewtons);
    check(std::abs(flying.lastPolarDragTractionNewtons
                   - expectedTraction)
              < 1.0e-6 * std::max(1.0, expectedTraction),
          "drag: free-flight skin traction supplies only missing positive drag");
    check(flying.lastPolarDragTractionPowerWatts <= 1.0e-8,
          "drag: free-flight skin traction has non-positive air-relative power");

    pg::SimControls still = controls;
    still.pressureSolveMode = pg::PressureSolveMode::BoundedExteriorCp;
    still.pressurePascal = 0.0;
    pg::SimBody noQ = pg::buildSimBody(mesh, {}, still);
    pg::applyAerodynamicForces(noQ, still);
    check(length(noQ.lastPolarDragTractionForce) < 1.0e-15
              && noQ.lastPolarDragTractionNewtons == 0.0,
          "drag: zero dynamic pressure produces zero skin traction");
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
    testLineDeduplication();
    testRiserCutAndMass();
    testPayloadMassLineMassAndLaunch(mesh);
    testDynamicPointPayloadPendulum(mesh);
    testGrab(mesh);
    testCsv(mesh);
    testTinyRestEdgeIgnored(mesh);
    testBoundedPressureRetrim(mesh);
    if (failures > 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all playground_metrics checks passed\n");
    return 0;
}
