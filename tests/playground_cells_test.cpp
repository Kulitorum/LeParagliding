// Unit tests for the Playground's per-cell air model (SimCell state in
// playground_sim.{h,cpp}): cell construction from a synthetic three-rib
// wing, the face → cell map, the healthy-wing guarantee that the stamped
// field equals the old blanket-ram one, conservative cross-port refill,
// closed-skin volume coupling, swept intake flow and sealed compression.

#include "playground_sim.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
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

bool nearVec(const Vec3 &a, const Vec3 &b, double tolerance)
{
    return length(a - b) <= tolerance;
}

constexpr double kPi = 3.14159265358979323846;

// Three 8-node sections at x = -1, 0, +1 (chord 0.8 m along +y, LE at
// y = 0), skin quads joining adjacent sections around the profile, caps
// closing the tips. The underside nose quad ring (k == 7) is tagged Vent —
// its outward normal points forward-down, into the oncoming air, like a
// real intake. The middle rib carries one rectangular cross-port hole.
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
            mesh.quadSurfaces.push_back(k < 4    ? pg::SimSurface::Extrados
                                        : k == 7 ? pg::SimSurface::Vent
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
    // One 0.2 m x 0.04 m hole in the middle rib: 0.008 m² of cross-port.
    mesh.ribHoles[1].push_back({Vec3{0.0, 0.3, 0.02},
                                Vec3{0.0, 0.5, 0.02},
                                Vec3{0.0, 0.5, -0.02},
                                Vec3{0.0, 0.3, -0.02}});
    return mesh;
}

pg::SimBody build(bool cellModel)
{
    pg::SimControls controls;
    controls.cellPressureModel = cellModel;
    return pg::buildSimBody(testMesh(), {}, controls);
}

void setCellGaugePressure(pg::SimBody &sim,
                          std::initializer_list<double> pressure)
{
    sim.cellAir = {};
    sim.cellAirDiagnostics = {};
    sim.cellPressure.assign(pressure.begin(), pressure.end());
}

void testConstruction()
{
    const pg::SimBody sim = build(true);
    check(sim.cells.size() == 2, "two ribs bays -> two cells");
    if (sim.cells.size() != 2) {
        return;
    }
    const auto ribX = [&](std::size_t rib) {
        return sim.body->nodes()[sim.ribChords[rib].leadingNode]
            .position.x;
    };
    check(std::abs(ribX(sim.cells[0].ribs[0]) + 1.0) < 1e-9
              && std::abs(ribX(sim.cells[0].ribs[1])) < 1e-9,
          "cell 0 spans the ribs at x=-1 and x=0");
    check(std::abs(ribX(sim.cells[1].ribs[0])) < 1e-9
              && std::abs(ribX(sim.cells[1].ribs[1]) - 1.0) < 1e-9,
          "cell 1 spans the ribs at x=0 and x=+1");
    check(std::abs(sim.cells[0].portAreaToNext - 0.008) < 1e-9,
          "cross-port area equals the middle rib's hole area");
    check(sim.cells[1].portAreaToNext == 0.0,
          "the last cell has no next port");
    check(sim.cells[0].restVentArea > 0.0
              && sim.cells[1].restVentArea > 0.0,
          "both cells found their vent faces");
    check(sim.cells[0].restSectionArea > 0.05
              && sim.cells[0].restSectionArea < 0.13,
          "rest section area is the octagon's");
    check(sim.cells[0].restVolume > 0.05 && sim.cells[0].restVolume < 0.13,
          "rest volume is section area times 1 m spacing");
    check(sim.cellPressure.size() == 2,
          "build-time stamp initialised the cell state");

    // The face -> cell map: everything left of the middle rib is cell 0.
    bool mapped = true;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        const auto &tri = sim.body->triangles()[face];
        const double x = (sim.body->nodes()[tri.a].position.x
                          + sim.body->nodes()[tri.b].position.x
                          + sim.body->nodes()[tri.c].position.x)
                         / 3.0;
        const std::uint32_t expected = x < 0.0 ? 0 : 1;
        if (sim.faceAero[face].cell != expected) {
            mapped = false;
        }
    }
    check(mapped, "faces map to the cell their centroid sits in");
}

void testHealthyFieldMatchesLegacy()
{
    pg::SimBody withCells = build(true);
    pg::SimBody legacy = build(false);
    pg::SimControls onControls;
    onControls.cellPressureModel = true;
    pg::SimControls offControls;
    offControls.cellPressureModel = false;

    for (int pass = 0; pass < 5; ++pass) {
        pg::applyPressure(withCells, onControls);
        pg::applyPressure(legacy, offControls);
    }
    double worst = 0.0;
    for (std::size_t face = 0; face < withCells.skinTriangleCount;
         ++face) {
        worst = std::max(
            worst,
            std::abs(withCells.body->triangles()[face].pressureDifference
                     - legacy.body->triangles()[face].pressureDifference));
    }
    check(worst < 1e-9,
          "healthy rest-pose field is identical with the model on");
}

void testCrossPortRefill()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    if (sim.cells.size() != 2) {
        check(false, "refill test needs two cells");
        return;
    }
    // Seal both intakes so the only air path is the cross-port, then
    // empty one cell against a full neighbour.
    sim.cells[0].ventFaces.clear();
    sim.cells[0].restVentArea = 0.0;
    sim.cells[1].ventFaces.clear();
    sim.cells[1].restVentArea = 0.0;
    setCellGaugePressure(sim, {0.0, controls.pressurePascal});
    pg::applyPressure(sim, controls);
    check(sim.cellPressure[0] > 0.1,
          "an empty cell refills through the cross-port");
    check(sim.cellPressure[1] < controls.pressurePascal,
          "the donor cell pays for the refill");

    // Same setup without the port: nothing moves.
    sim.cells[0].portAreaToNext = 0.0;
    setCellGaugePressure(sim, {0.0, controls.pressurePascal});
    pg::applyPressure(sim, controls);
    check(std::abs(sim.cellPressure[0]) < 1.0e-8,
          "no port means no refill");
}

void testSealedCompressionResponse()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    pg::applyPressure(sim, controls);
    const std::vector<double> charged = sim.cellPressure;
    const double initialMass = std::accumulate(
        sim.cellAir.massKg.begin(), sim.cellAir.massKg.end(), 0.0);

    for (pg::SimCell &cell : sim.cells) {
        cell.ventFaces.clear();
    }
    sim.cells[0].portAreaToNext = 0.0;
    for (std::size_t node = 0; node < sim.canopyNodeCount; ++node) {
        sim.body->nodes()[node].position.z *= 0.95;
    }
    pg::applyPressure(sim, controls);

    check(sim.cellVolumeRatio[0] < 0.951
              && sim.cellVolumeRatio[0] > 0.949,
          "closed-skin volume follows a five-percent section squeeze");
    check(sim.cellPressure[0] > charged[0] + 1000.0
              && sim.cellPressure[1] > charged[1] + 1000.0,
          "a sealed squeeze raises pressure through the gas law");
    check(std::abs(std::accumulate(sim.cellAir.massKg.begin(),
                                   sim.cellAir.massKg.end(), 0.0)
                   - initialMass)
              < 1.0e-12,
          "a sealed squeeze conserves total finite air mass");

    bool uniform = true;
    for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
        uniform = uniform
                  && std::abs(sim.faceInteriorPressure[face]
                              - sim.cellPressure[sim.faceAero[face].cell])
                         < 1.0e-9;
    }
    check(uniform, "derived internal pressure is uniform within each cell");
}

void testIntakeRelaxation()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    // Knock both cells below their ram target and let the intakes refill
    // them: the state must rise monotonically toward the target and stay
    // bounded by it.
    setCellGaugePressure(sim, {40.0, 40.0});
    double previous = 40.0;
    bool wellBehaved = true;
    for (int frame = 0; frame < 600; ++frame) {
        pg::applyPressure(sim, controls);
        if (sim.cellPressure[0] < previous - 1e-9
            || sim.cellPressure[0] > controls.pressurePascal + 1e-9) {
            wellBehaved = false;
        }
        previous = sim.cellPressure[0];
    }
    check(wellBehaved, "intake refill is monotonic and never overshoots");
    check(sim.cellPressure[0] > 0.95 * controls.pressurePascal,
          "ten seconds of intake refill reach the ram target");
}

void testVentGating()
{
    // Pitch the whole wing 180 degrees about the span axis (a rotation,
    // not a mirror — winding must survive): the vents now face dead away
    // from the wind.
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    for (softwing::Node &node : sim.body->nodes()) {
        node.position.y = 0.8 - node.position.y;
        node.position.z = -node.position.z;
    }
    // A cell below its target must stay empty: this mouth is moving
    // backwards through the air it sits in, and nothing enters that way.
    setCellGaugePressure(sim, {0.0, 0.0});
    for (int frame = 0; frame < 60; ++frame) {
        pg::applyPressure(sim, controls);
    }
    check(std::abs(sim.cellPressure[0]) < 1.0e-8
              && std::abs(sim.cellPressure[1]) < 1.0e-8,
          "a vent facing away from the wind cannot scoop ram air");
    // But a cell ABOVE its target vents whichever way the mouth points.
    setCellGaugePressure(sim, {160.0, 160.0});
    pg::applyPressure(sim, controls);
    check(sim.cellPressure[0] < 160.0,
          "an over-pressured cell exhausts even facing away");
}

// The other half of that rule: what feeds a mouth is ITS OWN travel
// through the air, whatever direction the mouth happens to point. Same
// wing pitched dead away from the airflow as above, but now the whole
// thing is being carried downwind faster than the air moves, so every
// mouth is going mouth-first through it. It must fill — a wing that has
// pitched, rolled or swung has not stopped flying, and reading its
// intakes against one bulk wind direction is what left a tilted wing
// unable to take its air back.
void testMovingMouthFeeds()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    for (softwing::Node &node : sim.body->nodes()) {
        node.position.y = 0.8 - node.position.y;
        node.position.z = -node.position.z;
    }
    const double airspeed =
        std::sqrt(2.0 * controls.pressurePascal / 1.225);
    for (softwing::Node &node : sim.body->nodes()) {
        node.velocity = Vec3{0.0, 2.0 * airspeed, 0.0};
    }
    setCellGaugePressure(sim, {0.0, 0.0});
    for (int frame = 0; frame < 600; ++frame) {
        pg::applyPressure(sim, controls);
    }
    check(sim.cellPressure[0] > 0.3 * controls.pressurePascal
              && sim.cellPressure[1] > 0.3 * controls.pressurePascal,
          "a mouth travelling mouth-first through the air fills its cell");
}

// A mouth folded shut is shut BOTH ways. Letting a cell blow its air out
// through an opening it can no longer take air in through is a ratchet:
// one slow moment empties the wing and nothing can ever refill it.
void testPinchedMouthHoldsAir()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    pg::applyPressure(sim, controls);
    const double charged = sim.cellPressure[0];
    check(charged > 1.0, "the cells start charged");

    // Every mouth shut, SECTION BY SECTION: each section's mouth lip
    // pinched onto its own centroid, so the vent faces between sections
    // degenerate to lines and no opening is left for air to cross.
    //
    // Merging every vent node in the wing onto one point also shuts the
    // mouths, but it drags all three ribs together and destroys bay volume.
    // That tests sealed gas compression, not intake closure. Pinched per
    // section, the mouths close while the bays keep their volume.
    std::vector<std::size_t> ventNodes;
    for (const pg::SimCell &cell : sim.cells) {
        for (const std::size_t face : cell.ventFaces) {
            const auto &tri = sim.body->triangles()[face];
            ventNodes.push_back(tri.a);
            ventNodes.push_back(tri.b);
            ventNodes.push_back(tri.c);
        }
    }
    std::map<long long, std::vector<std::size_t>> sections;
    for (const std::size_t node : ventNodes) {
        sections[std::llround(sim.body->nodes()[node].position.x * 1000.0)]
            .push_back(node);
    }
    for (const auto &[station, group] : sections) {
        static_cast<void>(station);
        Vec3 centre;
        for (const std::size_t node : group) {
            centre += sim.body->nodes()[node].position;
        }
        centre = centre / static_cast<double>(group.size());
        for (const std::size_t node : group) {
            sim.body->nodes()[node].position = centre;
        }
    }

    pg::applyPressure(sim, controls);
    check(sim.cellIntakeOpening[0] < 1.0e-9
              && sim.cellIntakeOpening[1] < 1.0e-9,
          "pinching the vent geometry closes both live mouths");
    setCellGaugePressure(sim, {300.0, 300.0});
    pg::applyPressure(sim, controls);
    const double sealedMass = std::accumulate(
        sim.cellAir.massKg.begin(), sim.cellAir.massKg.end(), 0.0);
    pg::applyPressure(sim, controls);
    check(std::abs(std::accumulate(sim.cellAir.massKg.begin(),
                                   sim.cellAir.massKg.end(), 0.0)
                   - sealedMass)
              < 1.0e-12
              && sim.cellRawPressure[0] > 250.0,
          "a pinched authored intake bypasses no pressure-relief mass");

    // The tunnel off would empty an open mouth (testTunnelOffDeflates);
    // a shut one has to hold what it has.
    controls.pressurePascal = 0.0;
    for (int frame = 0; frame < 300; ++frame) {
        pg::applyPressure(sim, controls);
    }
    check(sim.cellPressure[0] > 0.9 * charged
              && sim.cellPressure[1] > 0.9 * charged,
          "a mouth folded shut cannot dump the cell's air either");
}

void testTunnelOffDeflates()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    pg::applyPressure(sim, controls);
    controls.pressurePascal = 0.0;
    for (int frame = 0; frame < 300; ++frame) {
        pg::applyPressure(sim, controls);
    }
    check(sim.cellPressure[0] < 1.0 && sim.cellPressure[1] < 1.0,
          "turning the tunnel off lets the cells exhaust toward zero");
}

void testOpenVolumeChangeMassLedger()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    pg::applyPressure(sim, controls);
    const double ledgerBefore = sim.cellAir.cumulativeReservoirInflowKg;
    for (std::size_t node = 0; node < sim.canopyNodeCount; ++node) {
        sim.body->nodes()[node].position.z *= 1.01;
    }
    pg::applyPressure(sim, controls);
    check(sim.cellVolumeRatio[0] > 1.009
              && std::isfinite(sim.cellPressure[0]),
          "an open growing cell remains finite while drawing swept air");
    check(std::abs(sim.cellAirDiagnostics.massResidualKg) < 1.0e-12,
          "swept-volume and orifice flow close the atmosphere mass ledger");
    check(std::abs(sim.cellAirDiagnostics.reservoirInflowKg
                   - (sim.cellAir.cumulativeReservoirInflowKg
                      - ledgerBefore))
              < 1.0e-12,
          "step diagnostics include swept and relief boundary exchange");
}

void testCrushedSealedCellRetainsAir()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    pg::applyPressure(sim, controls);
    const double charged = sim.cellPressure[0];
    const double initialMass = sim.cellAir.massKg[0];
    check(charged > 1.0, "the cells start charged");

    // Concertina bay 0 while leaving both its mouth and cross-port sealed.
    // There is no physical path which can delete its air: volume loss must
    // therefore raise its pressure and provide a restoring load.
    for (const std::size_t node : sim.ribLoopNodes[0]) {
        sim.body->nodes()[node].position.x = -0.02;
    }
    for (pg::SimCell &cell : sim.cells) {
        cell.ventFaces.clear();
    }
    sim.cells[0].portAreaToNext = 0.0;
    pg::applyPressure(sim, controls);
    check(sim.cellVolumeRatio[0] < 0.1,
          "closed-skin volume sees a concertinaed bay");
    check(sim.cellPressure[0] > charged + 1000.0,
          "a crushed sealed bay compresses its retained air");
    check(std::abs(sim.cellAir.massKg[0] - initialMass) < 1.0e-15,
          "a sealed collapse neither creates nor deletes air mass");
}

// A sealed cell at fixed geometry has neither a flow path nor a changing
// volume, so its mass and pressure remain exactly stationary.
void testHealthyBayIsNotVented()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    pg::applyPressure(sim, controls);
    // Both intakes and the port sealed: with the vent silent there is no
    // path left, so the state must not move by a pascal in ten seconds.
    for (pg::SimCell &cell : sim.cells) {
        cell.ventFaces.clear();
        cell.restVentArea = 0.0;
    }
    sim.cells[0].portAreaToNext = 0.0;
    const double charged = sim.cellPressure[0];
    for (int frame = 0; frame < 600; ++frame) {
        pg::applyPressure(sim, controls);
    }
    check(std::abs(sim.cellPressure[0] - charged) < 1.0e-8,
          "an undeformed bay is not vented at all");
}

void testGalileanAirState()
{
    pg::SimControls baseControls;
    baseControls.freeFlight = true;
    baseControls.launchMode = pg::LaunchMode::DropFromRest;
    baseControls.cellPressureModel = true;
    pg::SimControls shiftedControls = baseControls;
    const Vec3 frameShift{2.3, -1.7, 0.6};
    shiftedControls.ambientAirVelocityWorld = frameShift;

    pg::SimBody base = pg::buildSimBody(testMesh(), {}, baseControls);
    pg::SimBody shifted =
        pg::buildSimBody(testMesh(), {}, shiftedControls);
    check(!pg::sampleWingAero(base, baseControls).valid,
          "air state: reference q is not a moving free-flight atmosphere");
    check(!base.cellPressure.empty()
              && base.cellPressure[0] > 0.9 * baseControls.pressurePascal,
          "air state: drop-from-rest is nevertheless pre-inflated");
    const Vec3 baseVelocity{0.4, -9.3, -1.2};
    for (std::size_t index = 0; index < base.body->nodes().size(); ++index) {
        base.body->nodes()[index].velocity = baseVelocity;
        shifted.body->nodes()[index].velocity = baseVelocity + frameShift;
    }

    const pg::WingAeroSample baseSample =
        pg::sampleWingAero(base, baseControls);
    const pg::WingAeroSample shiftedSample =
        pg::sampleWingAero(shifted, shiftedControls);
    check(baseSample.valid && shiftedSample.valid,
          "Galilean: both aero samples are valid");
    check(std::abs(baseSample.airspeed - shiftedSample.airspeed) < 1e-12
              && std::abs(baseSample.dynamicPressure
                          - shiftedSample.dynamicPressure)
                     < 1e-12
              && std::abs(baseSample.alphaRadians
                          - shiftedSample.alphaRadians)
                     < 1e-12
              && nearVec(baseSample.windDirection,
                         shiftedSample.windDirection,
                         1e-12),
          "Galilean: ambient and surface shifts leave polar sample unchanged");

    pg::applyPressure(base, baseControls);
    pg::applyPressure(shifted, shiftedControls);
    double pressureError = 0.0;
    for (std::size_t face = 0; face < base.skinTriangleCount; ++face) {
        pressureError = std::max(
            pressureError,
            std::abs(base.body->triangles()[face].pressureDifference
                     - shifted.body->triangles()[face].pressureDifference));
    }
    double cellError = 0.0;
    for (std::size_t cell = 0; cell < base.cellPressure.size(); ++cell) {
        cellError = std::max(
            cellError,
            std::abs(base.cellPressure[cell] - shifted.cellPressure[cell]));
    }
    check(pressureError < 1e-11 && cellError < 1e-11,
          "Galilean: rib pressure, intakes and cell state are invariant");

    pg::applyAerodynamicForces(base, baseControls);
    pg::applyAerodynamicForces(shifted, shiftedControls);
    double forceError = 0.0;
    for (std::size_t index = 0; index < base.body->nodes().size(); ++index) {
        forceError = std::max(
            forceError,
            length(base.body->nodes()[index].force
                   - shifted.body->nodes()[index].force));
    }
    check(forceError < 1e-9
              && nearVec(base.lastAeroForce, shifted.lastAeroForce, 1e-9),
          "Galilean: distributed aerodynamic forces are invariant");

    pg::stepSimulation(base, baseControls);
    pg::stepSimulation(shifted, shiftedControls);
    double positionError = 0.0;
    double velocityError = 0.0;
    for (std::size_t index = 0; index < base.body->nodes().size(); ++index) {
        positionError = std::max(
            positionError,
            length(base.body->nodes()[index].position
                   - shifted.body->nodes()[index].position));
        velocityError = std::max(
            velocityError,
            length(base.body->nodes()[index].velocity + frameShift
                   - shifted.body->nodes()[index].velocity));
    }
    check(positionError < 1e-8 && velocityError < 1e-8,
          "Galilean: one solver step preserves relative state");
}

void testFlightFrameDefinition()
{
    pg::SimControls controls;
    controls.freeFlight = true;
    controls.launchMode = pg::LaunchMode::DropFromRest;
    controls.cellPressureModel = false;
    pg::SimBody sim = pg::buildSimBody(testMesh(), {}, controls);
    const double speed = 9.0;
    for (softwing::Node &node : sim.body->nodes()) {
        node.velocity = {0.0, -speed, 0.0};
    }

    const pg::FlightFrameSample aligned =
        pg::sampleFlightFrame(sim, controls);
    check(aligned.valid,
          "flight frame: aligned free-flight sample is valid");
    check(nearVec(aligned.forwardDirection, {0.0, -1.0, 0.0}, 1e-12)
              && nearVec(aligned.travelVelocity,
                         {0.0, -speed, 0.0}, 1e-12),
          "flight frame: forward is opposite the LE-to-TE mesh chord");
    check(std::abs(aligned.forwardSpeed - speed) < 1e-12
              && std::abs(aligned.spanwiseSpeed) < 1e-12
              && std::abs(aligned.noseHeadingRadians) < 1e-12
              && std::abs(aligned.courseHeadingRadians) < 1e-12,
          "flight frame: aligned travel has positive speed and zero headings");

    // A course ten degrees toward the +X span with the nose unchanged is
    // positive sideslip, not a reversed forward velocity.
    const double beta = 10.0 * kPi / 180.0;
    for (softwing::Node &node : sim.body->nodes()) {
        node.velocity = {speed * std::sin(beta),
                         -speed * std::cos(beta),
                         0.0};
    }
    const pg::FlightFrameSample slipping =
        pg::sampleFlightFrame(sim, controls);
    check(slipping.valid
              && std::abs(slipping.courseHeadingRadians - beta) < 1e-12
              && std::abs(slipping.noseHeadingRadians) < 1e-12
              && std::abs(slipping.sideslipRadians - beta) < 1e-12,
          "flight frame: +X course offset has the documented sideslip sign");
}

void testBrakeImmuneSectionIncidence()
{
    pg::SimControls controls;
    controls.freeFlight = true;
    controls.launchMode = pg::LaunchMode::DropFromRest;
    controls.cellPressureModel = false;
    pg::SimBody base = pg::buildSimBody(testMesh(), {}, controls);
    pg::SimBody deflected = pg::buildSimBody(testMesh(), {}, controls);
    const Vec3 flightVelocity{0.0, -9.0, -1.0};
    for (softwing::Node &node : base.body->nodes()) {
        node.velocity = flightVelocity;
    }
    for (softwing::Node &node : deflected.body->nodes()) {
        node.velocity = flightVelocity;
    }
    check(deflected.ribChords.size() == 3,
          "section incidence: synthetic wing exposes three rib frames");
    if (deflected.ribChords.size() != 3) {
        return;
    }

    // Move only the aft point of one rib, as a brake does. The skin geometry
    // is allowed to change, but the forward 40% attitude reference is not.
    const std::size_t trailing = deflected.ribChords[2].trailingNode;
    deflected.body->nodes()[trailing].position.z -= 0.20;
    pg::applyPressure(base, controls);
    pg::applyPressure(deflected, controls);
    check(base.ribLiftCoefficient.size() == 3
              && deflected.ribLiftCoefficient.size() == 3,
          "section incidence: both pressure samples report rib lift");
    check(base.ribLiftCoefficient.size() == 3
              && deflected.ribLiftCoefficient.size() == 3
              && std::abs(base.ribLiftCoefficient[2]
                          - deflected.ribLiftCoefficient[2])
                     < 1e-12,
          "section incidence: trailing-edge pull is not rigid section pitch");
}

void testRigidYawPreservesSectionIncidence()
{
    pg::SimControls controls;
    controls.freeFlight = true;
    controls.launchMode = pg::LaunchMode::DropFromRest;
    controls.cellPressureModel = false;
    pg::SimBody base = pg::buildSimBody(testMesh(), {}, controls);
    pg::SimBody yawed = pg::buildSimBody(testMesh(), {}, controls);
    // Past 90 degrees catches any attempt to reorient the live material span
    // against the fixed rest/world axis.
    const double yaw = 135.0 * kPi / 180.0;
    const auto rotateZ = [&](const Vec3 &value) {
        return Vec3{std::cos(yaw) * value.x - std::sin(yaw) * value.y,
                    std::sin(yaw) * value.x + std::cos(yaw) * value.y,
                    value.z};
    };
    const Vec3 velocity{0.0, -9.0, -1.0};
    for (std::size_t index = 0; index < base.body->nodes().size(); ++index) {
        base.body->nodes()[index].velocity = velocity;
        yawed.body->nodes()[index].position =
            rotateZ(yawed.body->nodes()[index].position);
        yawed.body->nodes()[index].velocity = rotateZ(velocity);
    }
    pg::applyPressure(base, controls);
    pg::applyPressure(yawed, controls);
    bool invariant = base.ribLiftCoefficient.size()
                     == yawed.ribLiftCoefficient.size();
    if (invariant) {
        for (std::size_t rib = 0; rib < base.ribLiftCoefficient.size();
             ++rib) {
            invariant = invariant
                        && std::abs(base.ribLiftCoefficient[rib]
                                    - yawed.ribLiftCoefficient[rib])
                               < 1e-11;
        }
    }
    check(invariant,
          "section incidence: rigid yaw carries every rib plane with the wing");
    const pg::FlightFrameSample frame =
        pg::sampleFlightFrame(yawed, controls);
    check(frame.valid
              && std::abs(frame.noseHeadingRadians - yaw) < 1e-11
              && std::abs(frame.courseHeadingRadians - yaw) < 1e-11
              && std::abs(frame.forwardSpeed - 9.0) < 1e-11,
          "flight frame: material forward and span remain continuous past 90 degrees");
}

void testSectionPlaneWeathercock()
{
    pg::SimControls controls;
    controls.freeFlight = true;
    controls.launchMode = pg::LaunchMode::DropFromRest;
    controls.cellPressureModel = false;
    pg::SimBody sim = pg::buildSimBody(testMesh(), {}, controls);
    check(sim.ribChords.size() == 3,
          "weathercock: synthetic wing has three section planes");
    if (sim.ribChords.size() != 3) {
        return;
    }

    // Give both halves equal planform authority for this pure helper case.
    // The centre rib joins the low-span half; its live position still sets
    // the correct quarter-chord centre for the rigid-spin term.
    sim.ribHalf = {0, 0, 1};
    sim.ribPlanformArea = {0.5, 0.5, 1.0};
    sim.halfPlanformArea = {1.0, 1.0};
    const double referenceSpeed = length(
        pg::referenceFlowVelocity(sim, controls));
    const auto setFlow = [&](double betaSpeed) {
        for (softwing::Node &node : sim.body->nodes()) {
            node.velocity = {-betaSpeed, -referenceSpeed, 0.0};
        }
        return pg::sampleWingAero(sim, controls);
    };

    for (pg::RibChord &rib : sim.ribChords) {
        rib.spanAxis = {1.0, 0.0, 0.0};
    }
    const pg::HalfAeroKinematics flat =
        pg::sampleHalfAeroKinematics(sim, setFlow(2.0));
    check(flat.valid
              && std::abs(flat.alphaDeviationRadians[0]) < 1e-12
              && std::abs(flat.alphaDeviationRadians[1]) < 1e-12,
          "weathercock: a flat wing removes spanwise beta from every section");

    const double arc = 25.0 * kPi / 180.0;
    const Vec3 lowNormal{std::cos(arc), 0.0, std::sin(arc)};
    const Vec3 highNormal{std::cos(arc), 0.0, -std::sin(arc)};
    sim.ribChords[0].spanAxis = lowNormal;
    sim.ribChords[1].spanAxis = lowNormal;
    sim.ribChords[2].spanAxis = highNormal;
    const pg::HalfAeroKinematics noBeta =
        pg::sampleHalfAeroKinematics(sim, setFlow(0.0));
    const pg::HalfAeroKinematics positive =
        pg::sampleHalfAeroKinematics(sim, setFlow(2.0));
    const pg::HalfAeroKinematics negative =
        pg::sampleHalfAeroKinematics(sim, setFlow(-2.0));
    check(positive.valid && negative.valid
              && positive.alphaDeviationRadians[0]
                     * positive.alphaDeviationRadians[1]
                     < 0.0,
          "weathercock: mirrored arc turns beta into opposite half incidences");
    check(noBeta.valid
              && std::abs(noBeta.alphaDeviationRadians[0]) < 1e-12
              && std::abs(noBeta.alphaDeviationRadians[1]) < 1e-12,
          "weathercock: asymmetric section geometry has zero no-beta departure");
    check(std::abs(positive.alphaDeviationRadians[0]
                   + negative.alphaDeviationRadians[0])
                  < 1e-12
              && std::abs(positive.alphaDeviationRadians[1]
                          + negative.alphaDeviationRadians[1])
                     < 1e-12,
          "weathercock: reversing beta reverses the differential sign");
    check(std::abs(positive.alphaDeviationRadians[0]
                   + positive.alphaDeviationRadians[1])
                  < 1e-12,
          "weathercock: exact weighted common incidence is removed");

    const auto yawMomentFor = [&](double betaSpeed) {
        sim.alphaFilteredRadians =
            std::numeric_limits<double>::quiet_NaN();
        sim.alphaHalfDeviationRadians = {0.0, 0.0};
        sim.halfDynamicPressureRatio = {1.0, 1.0};
        setFlow(betaSpeed);
        pg::applyPressure(sim, controls);
        pg::applyAerodynamicForces(sim, controls);
        const pg::WingAeroSample sample = pg::sampleWingAero(sim, controls);
        Vec3 centre;
        for (std::size_t node = 0; node < sim.canopyNodeCount; ++node) {
            centre += sim.body->nodes()[node].position;
        }
        centre = centre / static_cast<double>(sim.canopyNodeCount);
        Vec3 moment;
        for (std::size_t face = 0; face < sim.skinTriangleCount; ++face) {
            const auto &tri = sim.body->triangles()[face];
            const Vec3 &a = sim.body->nodes()[tri.a].position;
            const Vec3 &b = sim.body->nodes()[tri.b].position;
            const Vec3 &c = sim.body->nodes()[tri.c].position;
            const Vec3 force = tri.pressureDifference
                               * 0.5 * cross(b - a, c - a);
            moment += cross((a + b + c) / 3.0 - centre, force);
        }
        const Vec3 up = normalized(
            cross(sample.spanAxis, sample.chordDirection));
        return dot(moment, up);
    };
    const double yawZero = yawMomentFor(0.0);
    const double yawPositive = yawMomentFor(2.0) - yawZero;
    const double yawNegative = yawMomentFor(-2.0) - yawZero;
    check(yawPositive < 0.0 && yawNegative > 0.0,
          "weathercock: differential pressure moment restores both beta signs");
    check(std::abs(std::abs(yawPositive) - std::abs(yawNegative))
                  < 0.08 * 0.5
                        * (std::abs(yawPositive) + std::abs(yawNegative)),
          "weathercock: mirrored beta produces near-odd yaw-moment parity");
    check(std::max(std::abs(yawPositive), std::abs(yawNegative)) < 10.0,
          "weathercock: small-beta yaw moment stays bounded");

    for (int frame = 0; frame < 120; ++frame) {
        setFlow(50.0);
        pg::applyPressure(sim, controls);
        pg::applyAerodynamicForces(sim, controls);
    }
    constexpr double halfLimit = 10.0 * kPi / 180.0;
    check(std::isfinite(sim.alphaHalfDeviationRadians[0])
              && std::isfinite(sim.alphaHalfDeviationRadians[1])
              && std::abs(sim.alphaHalfDeviationRadians[0])
                     <= halfLimit + 1e-12
              && std::abs(sim.alphaHalfDeviationRadians[1])
                     <= halfLimit + 1e-12,
          "weathercock: extreme-beta filter remains finite and clamped");
}

}  // namespace

int main()
{
    testConstruction();
    testHealthyFieldMatchesLegacy();
    testCrossPortRefill();
    testSealedCompressionResponse();
    testIntakeRelaxation();
    testVentGating();
    testMovingMouthFeeds();
    testPinchedMouthHoldsAir();
    testTunnelOffDeflates();
    testOpenVolumeChangeMassLedger();
    testCrushedSealedCellRetainsAir();
    testHealthyBayIsNotVented();
    testGalileanAirState();
    testFlightFrameDefinition();
    testBrakeImmuneSectionIncidence();
    testRigidYawPreservesSectionIncidence();
    testSectionPlaneWeathercock();
    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("playground cells: all checks passed\n");
    return 0;
}
