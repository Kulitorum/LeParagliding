// Unit tests for the Playground's per-cell air model (SimCell state in
// playground_sim.{h,cpp}): cell construction from a synthetic three-rib
// wing, the face → cell map, the healthy-wing guarantee that the stamped
// field equals the old blanket-ram one, cross-port refill of a sealed
// cell, and the squeeze response of a collapsed section.

#include "playground_sim.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
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
    sim.cellPressure = {0.0, controls.pressurePascal};
    pg::applyPressure(sim, controls);
    check(sim.cellPressure[0] > 0.1,
          "an empty cell refills through the cross-port");
    check(sim.cellPressure[1] < controls.pressurePascal,
          "the donor cell pays for the refill");

    // Same setup without the port: nothing moves.
    sim.cells[0].portAreaToNext = 0.0;
    sim.cellPressure = {0.0, controls.pressurePascal};
    pg::applyPressure(sim, controls);
    check(sim.cellPressure[0] == 0.0,
          "no port means no refill");
}

void testSqueezeResponse()
{
    pg::SimBody squeezed = build(true);
    pg::SimBody legacy = build(false);
    pg::SimControls onControls;
    pg::SimControls offControls;
    offControls.cellPressureModel = false;

    // Collapse the middle rib section in BOTH bodies: every loop node
    // pulled to 20% of its offset from the loop centroid. Both cells
    // share that rib, so both read a squeezed section.
    for (pg::SimBody *sim : {&squeezed, &legacy}) {
        const auto &loop = sim->ribLoopNodes[1];
        Vec3 centre;
        for (const std::size_t node : loop) {
            centre += sim->body->nodes()[node].position;
        }
        centre = centre / static_cast<double>(loop.size());
        for (const std::size_t node : loop) {
            Vec3 &position = sim->body->nodes()[node].position;
            position = centre + 0.2 * (position - centre);
        }
    }
    pg::applyPressure(squeezed, onControls);
    pg::applyPressure(legacy, offControls);

    // The squeeze adds the same per-cell pressure to every face of a
    // cell, on top of an exterior field both bodies share.
    double least = 1e30;
    double spread = 0.0;
    std::array<double, 2> cellDiff{1e30, 1e30};
    for (std::size_t face = 0; face < squeezed.skinTriangleCount;
         ++face) {
        const double diff =
            squeezed.body->triangles()[face].pressureDifference
            - legacy.body->triangles()[face].pressureDifference;
        least = std::min(least, diff);
        double &expected = cellDiff[squeezed.faceAero[face].cell];
        if (expected > 1e29) {
            expected = diff;
        }
        spread = std::max(spread, std::abs(diff - expected));
    }
    check(least > 20.0,
          "a squeezed cell pushes back with tens of pascals");
    check(spread < 1e-9,
          "the squeeze is uniform across each cell's faces");
}

void testIntakeRelaxation()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    // Knock both cells below their ram target and let the intakes refill
    // them: the state must rise monotonically toward the target and stay
    // bounded by it.
    sim.cellPressure = {40.0, 40.0};
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
    sim.cellPressure = {0.0, 0.0};
    for (int frame = 0; frame < 60; ++frame) {
        pg::applyPressure(sim, controls);
    }
    check(sim.cellPressure[0] == 0.0 && sim.cellPressure[1] == 0.0,
          "a vent facing away from the wind cannot scoop ram air");
    // But a cell ABOVE its target vents whichever way the mouth points.
    sim.cellPressure = {160.0, 160.0};
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
    sim.cellPressure = {0.0, 0.0};
    for (int frame = 0; frame < 600; ++frame) {
        pg::applyPressure(sim, controls);
    }
    check(sim.cellPressure[0] > 0.7 * controls.pressurePascal
              && sim.cellPressure[1] > 0.7 * controls.pressurePascal,
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
    // Merging every vent node in the wing onto one point — what this did
    // — shuts the mouths too, but it also drags the three ribs into each
    // other, and a bay with no volume left is the one case that IS meant
    // to lose its air (see testCollapseVent). Pinched per section, the
    // mouths close and the bays keep their volume, which is the case
    // this test is about.
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

void testRateClamp()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    // A near-zero volume makes the intake rate enormous; the per-frame
    // clamp must stop the step at exactly half the gap.
    sim.cells[0].restVolume = 1.0e-6;
    sim.cells[0].portAreaToNext = 0.0;   // isolate the intake term
    sim.cellPressure = {0.0, controls.pressurePascal};
    pg::applyPressure(sim, controls);
    check(std::abs(sim.cellPressure[0] - 0.5 * controls.pressurePascal)
              < 1e-6,
          "a runaway relaxation rate is clamped to half the gap");
}

void testSqueezeCap()
{
    pg::SimBody squeezed = build(true);
    pg::SimBody legacy = build(false);
    pg::SimControls onControls;
    pg::SimControls offControls;
    offControls.cellPressureModel = false;
    // Crush EVERY rib section to 10%: the area ratio lands under the
    // floor and the raw boost far past the cap, so the stamp must
    // saturate at kCellSqueezeCapRatio times the state — and the state
    // itself must not wind up (the squeeze is stamp-only).
    for (pg::SimBody *sim : {&squeezed, &legacy}) {
        for (const auto &loop : sim->ribLoopNodes) {
            Vec3 centre;
            for (const std::size_t node : loop) {
                centre += sim->body->nodes()[node].position;
            }
            centre = centre / static_cast<double>(loop.size());
            for (const std::size_t node : loop) {
                Vec3 &position = sim->body->nodes()[node].position;
                position = centre + 0.1 * (position - centre);
            }
        }
    }
    pg::applyPressure(squeezed, onControls);
    pg::applyPressure(legacy, offControls);
    // The cap is a multiple of whatever state the cell still HAS, not of
    // the ram target: a cell that has lost its air cannot push back. A
    // bay crushed this far has also lost essentially all of its volume,
    // so the collapse vent is draining it (see testCollapseVent) — which
    // is why the state is read here rather than assumed.
    const double state = squeezed.cellPressure[0];
    const double diff =
        squeezed.body->triangles()[0].pressureDifference
        - legacy.body->triangles()[0].pressureDifference;
    check(std::abs(diff - (4.0 * state - onControls.pressurePascal)) < 1e-6,
          "the squeeze saturates at the cap");
    check(state <= onControls.pressurePascal + 1e-9,
          "the squeeze is stamp-only; the state does not wind up");
}

// A bay that has lost its VOLUME must be able to lose its air. Before
// the collapse vent, a folded bay was force-fed to ram exactly like a
// flying one — its ribs read half-rho-v-squared whether they were in
// clean air or buried inside a fold — so the folded cell and its healthy
// neighbour BOTH sat at target, there was no gradient across the rib
// holes, and the cross-port re-inflation path could do nothing however
// hard it was driven. Ten times a zero gradient is still zero.
void testCollapseVent()
{
    pg::SimBody sim = build(true);
    pg::SimControls controls;
    pg::applyPressure(sim, controls);
    const double charged = sim.cellPressure[0];
    check(charged > 1.0, "the cells start charged");

    // Bay 0 concertinaed: its outboard rib slid in against the middle
    // one. It keeps its whole section — translating a loop does not
    // change its vector area — and loses its volume, which is exactly
    // how this canopy collapses in the standing front-tuck case. Bay 1
    // is untouched.
    for (const std::size_t node : sim.ribLoopNodes[0]) {
        sim.body->nodes()[node].position.x = -0.02;
    }
    // Both intakes and the cross-port sealed, so the vent is the only
    // path either cell has.
    for (pg::SimCell &cell : sim.cells) {
        cell.ventFaces.clear();
        cell.restVentArea = 0.0;
    }
    sim.cells[0].portAreaToNext = 0.0;
    for (int frame = 0; frame < 120; ++frame) {
        pg::applyPressure(sim, controls);
    }
    check(sim.cellPressure[0] < 0.2 * charged,
          "a bay crushed to a fraction of its volume vents toward ambient");
    check(sim.cellPressure[1] > 0.99 * charged,
          "its healthy neighbour keeps every pascal");
    check(sim.cellPressure[1] - sim.cellPressure[0] > 10.0,
          "so the cross-ports finally have a gradient to work on");
}

// And the deadband under it: a wing holding its designed shape must see
// the vent do nothing at all, or the calibration moves.
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
    check(sim.cellPressure[0] == charged,
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
    testSqueezeResponse();
    testIntakeRelaxation();
    testVentGating();
    testMovingMouthFeeds();
    testPinchedMouthHoldsAir();
    testTunnelOffDeflates();
    testRateClamp();
    testSqueezeCap();
    testCollapseVent();
    testHealthyBayIsNotVented();
    testGalileanAirState();
    testSectionPlaneWeathercock();
    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("playground cells: all checks passed\n");
    return 0;
}
