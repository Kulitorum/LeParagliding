// Unit tests for the Playground's per-cell air model (SimCell state in
// playground_sim.{h,cpp}): cell construction from a synthetic three-rib
// wing, the face → cell map, the healthy-wing guarantee that the stamped
// field equals the old blanket-ram one, cross-port refill of a sealed
// cell, and the squeeze response of a collapsed section.

#include "playground_sim.h"

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
    // A cell below its target must stay empty: a reversed mouth cannot
    // scoop ram air.
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
    const double diff =
        squeezed.body->triangles()[0].pressureDifference
        - legacy.body->triangles()[0].pressureDifference;
    check(std::abs(diff - 3.0 * onControls.pressurePascal) < 1e-6,
          "the squeeze saturates at the cap");
    check(std::abs(squeezed.cellPressure[0] - onControls.pressurePascal)
              < 1e-9,
          "the squeeze is stamp-only; the state does not wind up");
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
    testTunnelOffDeflates();
    testRateClamp();
    testSqueezeCap();
    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("playground cells: all checks passed\n");
    return 0;
}
