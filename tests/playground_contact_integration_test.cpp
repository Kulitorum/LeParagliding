// Wiring test: the same stepSimulation path used by the GUI/bench must invoke
// Playground contact when requested and remain unchanged when it is off.

#include "playground_sim.h"

#include <cstdio>

namespace pg = lep::playground;

namespace {

pg::SimMesh sheetsMesh()
{
    pg::SimMesh mesh;
    for (const double z : {0.0, 0.05}) {
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

double sheetHeight(const pg::SimBody &sim, int sheet)
{
    double total = 0.0;
    for (int index = 0; index < 6; ++index) {
        total += sim.body->nodes()[sheet * 6 + index].position.z;
    }
    return total / 6.0;
}

double run(bool contact)
{
    pg::SimControls controls;
    controls.pressurePascal = 0.0;
    controls.fabricContact = contact;
    pg::SimBody sim = pg::buildSimBody(sheetsMesh(), {}, controls);
    for (int index = 6; index < 12; ++index) {
        sim.body->nodes()[index].velocity = {0.0, 0.0, -1.0};
    }
    for (int frame = 0; frame < 60; ++frame) {
        pg::stepSimulation(sim, controls);
    }
    if (contact && (!sim.contact.stats.coverageComplete
                    || sim.contact.stats.projectionVisits == 0)) {
        std::fprintf(stderr,
                     "FAIL: enabled integration pass did not run with full "
                     "coverage\n");
        return -100.0;
    }
    return sheetHeight(sim, 1) - sheetHeight(sim, 0);
}

}  // namespace

int main()
{
    const double blocked = run(true);
    const double ghosted = run(false);
    if (!(blocked > 0.0005 && blocked < 0.02)) {
        std::fprintf(stderr,
                     "FAIL: enabled contact did not stop the crossing "
                     "(gap %.6f m)\n",
                     blocked);
        return 1;
    }
    if (!(ghosted < 0.0)) {
        std::fprintf(stderr,
                     "FAIL: disabled contact changed the historical ghosting "
                     "path (gap %.6f m)\n",
                     ghosted);
        return 1;
    }
    std::printf("playground contact integration: all checks passed\n");
    return 0;
}
