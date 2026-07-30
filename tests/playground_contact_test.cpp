// Unit tests for the Playground's fabric contact pass: two free-floating
// fabric sheets driven into each other must be stopped at the contact
// separation with the option on, pass straight through with it off, and
// sheets DESIGNED closer than the separation (rest-pose exclusions) must
// be left alone.

#include "playground_sim.h"

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

// Two 2x1-quad horizontal sheets, sheet B a gap above sheet A. No rib
// loops, so the pressure field falls back to the uniform stamp — and the
// controls run it at zero pressure, so the sheets are free cloth in still
// air with no gravity (pinned mode).
pg::SimMesh sheetsMesh(double gapMetres)
{
    pg::SimMesh mesh;
    for (const double z : {0.0, gapMetres}) {
        for (int row = 0; row < 2; ++row) {
            for (int column = 0; column < 3; ++column) {
                mesh.nodes.push_back({0.4 * column, 0.4 * row, z});
            }
        }
    }
    const auto node = [](int sheet, int row, int column) {
        return sheet * 6 + row * 3 + column;
    };
    for (int sheet = 0; sheet < 2; ++sheet) {
        for (int column = 0; column < 2; ++column) {
            mesh.quads.push_back({node(sheet, 0, column),
                                  node(sheet, 0, column + 1),
                                  node(sheet, 1, column + 1),
                                  node(sheet, 1, column)});
            mesh.quadSurfaces.push_back(pg::SimSurface::Extrados);
        }
    }
    return mesh;
}

pg::SimControls stillAir()
{
    pg::SimControls controls;
    controls.pressurePascal = 0.0;
    return controls;
}

// Mean height of one sheet's nodes.
double sheetHeight(const pg::SimBody &sim, int sheet)
{
    double total = 0.0;
    for (int index = 0; index < 6; ++index) {
        total += sim.body->nodes()[static_cast<std::size_t>(sheet * 6
                                                            + index)]
                     .position.z;
    }
    return total / 6.0;
}

void driveSheetDown(pg::SimBody &sim, double metresPerSecond)
{
    for (int index = 6; index < 12; ++index) {
        sim.body->nodes()[static_cast<std::size_t>(index)].velocity = {
            0.0, 0.0, -metresPerSecond};
    }
}

void testCrossingBlocked()
{
    pg::SimControls controls = stillAir();
    controls.fabricContact = true;
    pg::SimBody sim = pg::buildSimBody(sheetsMesh(0.05), {}, controls);
    check(sim.contact.prepared, "the build prepared the contact scratch");
    check(sim.contact.restExclusions.empty(),
          "a 5 cm gap is outside the rest-exclusion capture");
    driveSheetDown(sim, 1.0);
    for (int frame = 0; frame < 60; ++frame) {
        pg::stepSimulation(sim, controls);
    }
    const double gap = sheetHeight(sim, 1) - sheetHeight(sim, 0);
    check(gap > 0.0005,
          "contact stops the falling sheet before it crosses");
    check(gap < 0.02,
          "the falling sheet actually reached the other one");
}

void testCrossingWithoutContact()
{
    pg::SimControls controls = stillAir();
    controls.fabricContact = false;
    pg::SimBody sim = pg::buildSimBody(sheetsMesh(0.05), {}, controls);
    driveSheetDown(sim, 1.0);
    for (int frame = 0; frame < 60; ++frame) {
        pg::stepSimulation(sim, controls);
    }
    check(sheetHeight(sim, 1) < sheetHeight(sim, 0),
          "without contact the sheets pass straight through");
}

void testOverlapRecovery()
{
    // Sheets built 5 cm apart (so no rest exclusions), then sheet B is
    // teleported to 0.5 mm above sheet A with every velocity zeroed. At
    // zero approach velocity the inelastic velocity fix is a no-op, so
    // only the PBD position projection can restore the separation — this
    // is the test that fails if the position corrections are broken
    // while the velocity fix still masks it in testCrossingBlocked.
    pg::SimControls controls = stillAir();
    controls.fabricContact = true;
    pg::SimBody sim = pg::buildSimBody(sheetsMesh(0.05), {}, controls);
    check(sim.contact.restExclusions.empty(),
          "sheets built 5 cm apart produce no rest exclusions");
    const double drop = 0.05 - 0.0005;
    for (int index = 6; index < 12; ++index) {
        auto &node = sim.body->nodes()[static_cast<std::size_t>(index)];
        node.position.z -= drop;
        node.velocity = {0.0, 0.0, 0.0};
    }
    for (int frame = 0; frame < 10; ++frame) {
        pg::stepSimulation(sim, controls);
    }
    const double gap = sheetHeight(sim, 1) - sheetHeight(sim, 0);
    check(gap > 0.0009,
          "position projection recovers overlapped fabric to separation");
    check(gap < 0.002,
          "overlap recovery does not overshoot the separation");
}

void testRestAdjacencyExcluded()
{
    // Sheets designed 0.5 mm apart — inside the contact separation. The
    // rest-pose exclusion set must keep contact from pushing designed
    // geometry apart.
    pg::SimControls controls = stillAir();
    controls.fabricContact = true;
    pg::SimBody sim = pg::buildSimBody(sheetsMesh(0.0005), {}, controls);
    check(!sim.contact.restExclusions.empty(),
          "designed-adjacent sheets produce rest exclusions");
    for (int frame = 0; frame < 60; ++frame) {
        pg::stepSimulation(sim, controls);
    }
    const double gap = sheetHeight(sim, 1) - sheetHeight(sim, 0);
    check(std::abs(gap - 0.0005) < 0.00035,
          "designed-adjacent fabric is not pushed apart");
}

}  // namespace

int main()
{
    testCrossingBlocked();
    testCrossingWithoutContact();
    testOverlapRecovery();
    testRestAdjacencyExcluded();
    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("playground contact: all checks passed\n");
    return 0;
}
