#ifndef LEP_PLAYGROUND_CELL_AIR_H
#define LEP_PLAYGROUND_CELL_AIR_H

#include <softwing/pneumatics.h>

#include <cstddef>
#include <span>
#include <vector>

namespace lep::playground {

// A deliberately small pneumatic state for the Playground. Geometry and
// openings are supplied afresh for every step; only finite cell mass and the
// atmosphere ledger persist. Pressure is always derived from mRT/V.
struct CellAirState
{
    std::vector<double> massKg;
    double referenceFiniteMassKg = 0.0;
    double cumulativeReservoirInflowKg = 0.0;
    bool massReferenceInitialized = false;
};

struct CellAirVolume
{
    double cubicMetres = 0.0;
};

// A fixed-pressure source such as a ram-recovery intake or the atmosphere.
// Positive diagnostic transfer is from this reservoir into the cell.
struct CellAirReservoirPort
{
    std::size_t cell = 0;
    double absolutePressurePascal = 101325.0;
    double absoluteTemperatureKelvin =
        softwing::standardFixtureTemperature;
    double effectiveAreaSquareMetres = 0.0;
    double openingFraction = 1.0;
    double dischargeCoefficient = 0.6;
};

// Positive diagnostic transfer is from firstCell into secondCell.
struct CellAirCrossPort
{
    std::size_t firstCell = 0;
    std::size_t secondCell = 0;
    double effectiveAreaSquareMetres = 0.0;
    double openingFraction = 1.0;
    double dischargeCoefficient = 0.6;
};

struct CellAirSettings
{
    softwing::AirProperties air;
    double cellTemperatureKelvin = softwing::standardFixtureTemperature;
    double ambientAbsolutePressurePascal = 101325.0;
    double massFloorKg = 1.0e-12;
    // Exact, explicit subdivision keeps results reproducible. The
    // conservative limiter also makes one substep safe for extreme ports;
    // more substeps improve transient accuracy rather than prevent failure.
    int substeps = 1;
};

struct CellAirCellDiagnostics
{
    double massKg = 0.0;
    double volumeCubicMetres = 0.0;
    double absolutePressurePascal = 0.0;
    double gaugePressurePascal = 0.0;
    double netMassRateKgPerSecond = 0.0;
};

struct CellAirPortDiagnostics
{
    // Integrated over the complete caller step. Raw transfer is what the
    // orifice requested; applied transfer includes the shared conservative
    // limiter. Sign follows the corresponding public port type.
    double rawTransferKg = 0.0;
    double appliedTransferKg = 0.0;
    double appliedMassRateKgPerSecond = 0.0;
    double minimumLimiterScale = 1.0;
    bool choked = false;
    bool donorMassLimited = false;
    bool pressureEquilibriumLimited = false;
};

struct CellAirStepDiagnostics
{
    std::vector<CellAirCellDiagnostics> cells;
    std::vector<CellAirPortDiagnostics> reservoirPorts;
    std::vector<CellAirPortDiagnostics> crossPorts;
    double finiteCellMassKg = 0.0;
    double reservoirInflowKg = 0.0;
    double cumulativeReservoirInflowKg = 0.0;
    double massResidualKg = 0.0;
    double minimumLimiterScale = 1.0;
    bool limiterActive = false;
    int substeps = 0;
};

// Initialize finite mass from gauge pressure at the supplied live volumes.
// Also establishes the reference used by later mass-residual diagnostics.
void initializeCellAirMass(
    CellAirState &state,
    std::span<const CellAirVolume> cells,
    std::span<const double> gaugePressurePascal,
    const CellAirSettings &settings = {});

// Read pressure from the current finite mass without advancing any flow.
[[nodiscard]] std::vector<double> cellAirGaugePressures(
    const CellAirState &state,
    std::span<const CellAirVolume> cells,
    const CellAirSettings &settings = {});

// Advance all ports simultaneously in each deterministic substep. Internal
// transfers are exactly equal and opposite. Every connected component shares
// one limiter scale, so a large group of ports cannot overdraw a donor or
// cross a pressure equilibrium within a substep.
[[nodiscard]] CellAirStepDiagnostics advanceCellAirMass(
    CellAirState &state,
    std::span<const CellAirVolume> cells,
    std::span<const CellAirReservoirPort> reservoirPorts,
    std::span<const CellAirCrossPort> crossPorts,
    double timeStepSeconds,
    const CellAirSettings &settings = {});

}  // namespace lep::playground

#endif
