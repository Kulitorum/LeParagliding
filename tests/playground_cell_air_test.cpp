#include "playground_cell_air.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

namespace pg = lep::playground;

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

bool near(double first, double second, double tolerance)
{
    return std::abs(first - second) <= tolerance;
}

pg::CellAirState initialized(std::span<const pg::CellAirVolume> volumes,
                             std::span<const double> gauge,
                             const pg::CellAirSettings &settings = {})
{
    pg::CellAirState state;
    pg::initializeCellAirMass(state, volumes, gauge, settings);
    return state;
}

void testEquationOfStateAndSealedCompression()
{
    std::array<pg::CellAirVolume, 1> volumes{{{0.1}}};
    const std::array<double, 1> initialGauge{80.0};
    pg::CellAirState state = initialized(volumes, initialGauge);
    const double initialMass = state.massKg[0];
    check(near(pg::cellAirGaugePressures(state, volumes)[0], 80.0, 1.0e-9),
          "equation of state reproduces initialized gauge pressure");

    volumes[0].cubicMetres *= 0.5;
    const auto compressed = pg::advanceCellAirMass(
        state, volumes, {}, {}, 1.0 / 60.0);
    const double expected = 2.0 * (101325.0 + 80.0) - 101325.0;
    check(state.massKg[0] == initialMass,
          "sealed compression conserves finite air mass exactly");
    check(near(compressed.cells[0].gaugePressurePascal, expected, 1.0e-8),
          "sealed half-volume cell doubles absolute pressure");
    check(std::abs(compressed.massResidualKg) < 1.0e-15,
          "sealed mass ledger closes");

    volumes[0].cubicMetres *= 2.0;
    check(near(pg::cellAirGaugePressures(state, volumes)[0], 80.0, 1.0e-9),
          "restoring volume restores pressure without a squeeze state");
}

void testCrossPortConservationAndEquilibriumLimit()
{
    const std::array<pg::CellAirVolume, 2> volumes{{{0.1}, {0.2}}};
    const std::array<double, 2> initialGauge{3000.0, 0.0};
    pg::CellAirState state = initialized(volumes, initialGauge);
    const double initialMass = state.massKg[0] + state.massKg[1];
    const std::array<pg::CellAirCrossPort, 1> ports{{
        {0, 1, 0.2, 1.0, 0.6},
    }};
    const auto result = pg::advanceCellAirMass(
        state, volumes, {}, ports, 1.0);

    check(result.crossPorts[0].appliedTransferKg > 0.0,
          "cross-port direction follows high pressure to low pressure");
    check(result.crossPorts[0].pressureEquilibriumLimited,
          "oversized cross-port is limited at pressure equilibrium");
    check(result.limiterActive,
          "step reports an active conservative limiter");
    check(near(result.cells[0].absolutePressurePascal,
               result.cells[1].absolutePressurePascal, 1.0e-8),
          "cross-port does not overshoot common pressure");
    check(near(state.massKg[0] + state.massKg[1], initialMass, 1.0e-15),
          "cross-port transfer conserves total finite mass");
    check(std::abs(result.reservoirInflowKg) < 1.0e-18
              && std::abs(result.massResidualKg) < 1.0e-15,
          "internal-only network has a closed mass ledger");
}

void testReservoirFillExhaustAndOpening()
{
    const std::array<pg::CellAirVolume, 1> volumes{{{0.1}}};
    const std::array<double, 1> ambientGauge{0.0};
    pg::CellAirState state = initialized(volumes, ambientGauge);
    std::array<pg::CellAirReservoirPort, 1> intake{{
        {0, 102325.0, softwing::standardFixtureTemperature,
         0.05, 1.0, 0.6},
    }};
    const auto filled = pg::advanceCellAirMass(
        state, volumes, intake, {}, 1.0);
    check(filled.reservoirPorts[0].appliedTransferKg > 0.0,
          "higher-pressure reservoir fills a cell");
    check(near(filled.cells[0].absolutePressurePascal, 102325.0, 1.0e-8),
          "reservoir fill stops at equilibrium without overshoot");
    check(near(filled.finiteCellMassKg - state.referenceFiniteMassKg,
               filled.cumulativeReservoirInflowKg, 1.0e-15)
              && std::abs(filled.massResidualKg) < 1.0e-15,
          "atmosphere ledger accounts for imported mass");

    std::array<pg::CellAirReservoirPort, 1> exhaust{{
        {0, 101325.0, softwing::standardFixtureTemperature,
         0.05, 1.0, 0.6},
    }};
    const auto emptied = pg::advanceCellAirMass(
        state, volumes, exhaust, {}, 1.0);
    check(emptied.reservoirPorts[0].appliedTransferKg < 0.0,
          "overpressured cell exhausts to a lower-pressure reservoir");
    check(near(emptied.cells[0].gaugePressurePascal, 0.0, 1.0e-8),
          "exhaust stops at ambient pressure");
    check(std::abs(emptied.massResidualKg) < 1.0e-15,
          "fill and exhaust preserve the cumulative mass ledger");

    pg::CellAirState sealed = initialized(volumes, ambientGauge);
    intake[0].openingFraction = 0.0;
    const auto closed = pg::advanceCellAirMass(
        sealed, volumes, intake, {}, 10.0);
    check(closed.reservoirPorts[0].appliedTransferKg == 0.0
              && closed.cells[0].gaugePressurePascal == 0.0,
          "zero opening blocks reservoir flow in both directions");
}

void testSimultaneousOrderIndependentTransfers()
{
    const std::array<pg::CellAirVolume, 3> volumes{{{0.1}, {0.1}, {0.1}}};
    const std::array<double, 3> initialGauge{6000.0, 0.0, 0.0};
    const std::array<pg::CellAirCrossPort, 2> forward{{
        {0, 1, 0.2, 1.0, 0.6},
        {0, 2, 0.2, 1.0, 0.6},
    }};
    const std::array<pg::CellAirCrossPort, 2> reversed{{
        {2, 0, 0.2, 1.0, 0.6},
        {1, 0, 0.2, 1.0, 0.6},
    }};
    pg::CellAirState first = initialized(volumes, initialGauge);
    pg::CellAirState second = initialized(volumes, initialGauge);
    const auto firstResult = pg::advanceCellAirMass(
        first, volumes, {}, forward, 1.0);
    const auto secondResult = pg::advanceCellAirMass(
        second, volumes, {}, reversed, 1.0);

    check(first.massKg == second.massKg,
          "port order and endpoint orientation do not change cell state");
    const double low = std::min({firstResult.cells[0].absolutePressurePascal,
                                 firstResult.cells[1].absolutePressurePascal,
                                 firstResult.cells[2].absolutePressurePascal});
    const double high = std::max({firstResult.cells[0].absolutePressurePascal,
                                  firstResult.cells[1].absolutePressurePascal,
                                  firstResult.cells[2].absolutePressurePascal});
    check(high - low < 1.0e-8,
          "simultaneous fan-out stops all ports at common pressure");
    check(std::abs(firstResult.massResidualKg) < 1.0e-15
              && std::abs(secondResult.massResidualKg) < 1.0e-15,
          "simultaneous internal transfers conserve mass");
}

void testExplicitSubstepsAreReproducible()
{
    const std::array<pg::CellAirVolume, 1> volumes{{{0.12}}};
    const std::array<double, 1> initialGauge{0.0};
    const std::array<pg::CellAirReservoirPort, 1> port{{
        {0, 101725.0, softwing::standardFixtureTemperature,
         2.0e-5, 0.8, 0.6},
    }};
    pg::CellAirSettings fourSubsteps;
    fourSubsteps.substeps = 4;
    pg::CellAirState oneCall = initialized(volumes, initialGauge,
                                           fourSubsteps);
    pg::CellAirState repeated = initialized(volumes, initialGauge);
    const auto oneResult = pg::advanceCellAirMass(
        oneCall, volumes, port, {}, 1.0 / 30.0, fourSubsteps);
    for (int i = 0; i < 4; ++i) {
        static_cast<void>(pg::advanceCellAirMass(
            repeated, volumes, port, {}, 1.0 / 120.0));
    }
    check(oneCall.massKg == repeated.massKg,
          "one four-substep call equals four explicit smaller calls");
    check(oneResult.substeps == 4,
          "diagnostics report the exact requested subdivision");

    pg::CellAirState again = initialized(volumes, initialGauge,
                                         fourSubsteps);
    const auto repeatedResult = pg::advanceCellAirMass(
        again, volumes, port, {}, 1.0 / 30.0, fourSubsteps);
    check(again.massKg == oneCall.massKg
              && repeatedResult.reservoirPorts[0].appliedTransferKg
                     == oneResult.reservoirPorts[0].appliedTransferKg,
          "identical subdivided runs are bit-repeatable");
}

void testDonorMassFloor()
{
    const std::array<pg::CellAirVolume, 1> volumes{{{0.001}}};
    const std::array<double, 1> initialGauge{0.0};
    pg::CellAirState state = initialized(volumes, initialGauge);
    const std::array<pg::CellAirReservoirPort, 1> vacuum{{
        {0, 1.0e-12, softwing::standardFixtureTemperature,
         1.0, 1.0, 1.0},
    }};
    const auto result = pg::advanceCellAirMass(
        state, volumes, vacuum, {}, 100.0);
    check(result.reservoirPorts[0].donorMassLimited,
          "extreme exhaust reports the donor mass-floor limiter");
    check(state.massKg[0] >= pg::CellAirSettings{}.massFloorKg
              && std::isfinite(result.cells[0].absolutePressurePascal),
          "mass-floor limit retains positive finite state");
    check(std::abs(result.massResidualKg) < 1.0e-15,
          "mass-floor-limited reservoir flow remains accounted");
}

}  // namespace

int main()
{
    testEquationOfStateAndSealedCompression();
    testCrossPortConservationAndEquilibriumLimit();
    testReservoirFillExhaustAndOpening();
    testSimultaneousOrderIndependentTransfers();
    testExplicitSubstepsAreReproducible();
    testDonorMassFloor();
    if (failures != 0) {
        std::fprintf(stderr, "%d cell-air check(s) failed\n", failures);
        return 1;
    }
    std::printf("playground cell air: all checks passed\n");
    return 0;
}
