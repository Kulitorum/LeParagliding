#include "playground_cell_air.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace lep::playground {
namespace {

constexpr std::size_t noCell = std::numeric_limits<std::size_t>::max();

[[nodiscard]] bool finitePositive(double value)
{
    return value > 0.0 && std::isfinite(value);
}

void validateSettings(const CellAirSettings &settings)
{
    settings.air.validate();
    if (!finitePositive(settings.cellTemperatureKelvin)
        || !finitePositive(settings.ambientAbsolutePressurePascal)
        || !finitePositive(settings.massFloorKg)
        || settings.substeps <= 0 || settings.substeps > 1024) {
        throw std::invalid_argument("Invalid Playground cell-air settings");
    }
}

void validateVolumes(std::span<const CellAirVolume> cells)
{
    if (std::any_of(cells.begin(), cells.end(), [](const CellAirVolume &cell) {
            return !finitePositive(cell.cubicMetres);
        })) {
        throw std::invalid_argument(
            "Cell-air volumes must be positive and finite");
    }
}

void validateState(const CellAirState &state,
                   std::span<const CellAirVolume> cells,
                   const CellAirSettings &settings)
{
    if (state.massKg.size() != cells.size()) {
        throw std::invalid_argument(
            "Cell-air mass and volume counts must match");
    }
    if (std::any_of(state.massKg.begin(), state.massKg.end(),
                    [&](double mass) {
                        return !std::isfinite(mass)
                               || mass < settings.massFloorKg;
                    })) {
        throw std::invalid_argument(
            "Cell-air mass must be finite and at least the mass floor");
    }
    if (state.massReferenceInitialized
        && (!std::isfinite(state.referenceFiniteMassKg)
            || !std::isfinite(state.cumulativeReservoirInflowKg))) {
        throw std::invalid_argument("Cell-air mass ledger is non-finite");
    }
}

void validateOrifice(double area, double opening, double discharge)
{
    if (area < 0.0 || !std::isfinite(area)
        || opening < 0.0 || opening > 1.0 || !std::isfinite(opening)
        || !(discharge > 0.0) || discharge > 1.0
        || !std::isfinite(discharge)) {
        throw std::invalid_argument("Invalid cell-air orifice definition");
    }
}

[[nodiscard]] double absolutePressure(double mass,
                                      double volume,
                                      const CellAirSettings &settings)
{
    const double pressure = mass * settings.air.specificGasConstant
                            * settings.cellTemperatureKelvin / volume;
    if (!finitePositive(pressure)) {
        throw std::runtime_error(
            "Cell-air equation of state produced invalid pressure");
    }
    return pressure;
}

[[nodiscard]] double sumMass(const std::vector<double> &mass)
{
    return static_cast<double>(std::accumulate(
        mass.begin(), mass.end(), 0.0L,
        [](long double sum, double value) {
            return sum + static_cast<long double>(value);
        }));
}

class DisjointSet
{
public:
    explicit DisjointSet(std::size_t size)
        : parent_(size), rank_(size, 0)
    {
        std::iota(parent_.begin(), parent_.end(), std::size_t{0});
    }

    [[nodiscard]] std::size_t find(std::size_t value)
    {
        if (parent_[value] != value) {
            parent_[value] = find(parent_[value]);
        }
        return parent_[value];
    }

    void unite(std::size_t first, std::size_t second)
    {
        first = find(first);
        second = find(second);
        if (first == second) {
            return;
        }
        if (rank_[first] < rank_[second]) {
            std::swap(first, second);
        }
        parent_[second] = first;
        if (rank_[first] == rank_[second]) {
            ++rank_[first];
        }
    }

private:
    std::vector<std::size_t> parent_;
    std::vector<unsigned char> rank_;
};

enum class EdgeKind
{
    CrossPort,
    ReservoirPort,
};

struct Edge
{
    EdgeKind kind = EdgeKind::CrossPort;
    std::size_t publicIndex = 0;
    std::size_t firstCell = noCell;
    std::size_t secondCell = noCell;
    double reservoirPressure = 0.0;
    double reservoirTemperature = 0.0;
    double area = 0.0;
    double opening = 0.0;
    double discharge = 0.0;
    softwing::MassFlowEvaluation evaluation;
    double rawTransferKg = 0.0;
};

[[nodiscard]] auto edgeSortKey(const Edge &edge)
{
    const std::size_t low = edge.kind == EdgeKind::CrossPort
                                ? std::min(edge.firstCell, edge.secondCell)
                                : edge.secondCell;
    const std::size_t high = edge.kind == EdgeKind::CrossPort
                                 ? std::max(edge.firstCell, edge.secondCell)
                                 : edge.secondCell;
    return std::tuple{edge.kind,
                      low,
                      high,
                      edge.reservoirPressure,
                      edge.reservoirTemperature,
                      edge.area,
                      edge.opening,
                      edge.discharge};
}

[[nodiscard]] softwing::MassFlowEvaluation evaluatePort(
    const softwing::FlowZoneState &first,
    const softwing::FlowZoneState &second,
    double area,
    double opening,
    double discharge,
    const CellAirSettings &settings)
{
    if (area == 0.0 || opening == 0.0) {
        return {};
    }
    return softwing::evaluateOrificeMassFlow(
        first, second, settings.air,
        softwing::OrificeDefinition{discharge, area, opening});
}

void recordPort(CellAirPortDiagnostics &diagnostic,
                const Edge &edge,
                double appliedTransfer,
                double scale,
                bool donorLimited,
                bool equilibriumLimited)
{
    diagnostic.rawTransferKg += edge.rawTransferKg;
    diagnostic.appliedTransferKg += appliedTransfer;
    diagnostic.minimumLimiterScale =
        std::min(diagnostic.minimumLimiterScale, scale);
    diagnostic.choked = diagnostic.choked || edge.evaluation.choked;
    diagnostic.donorMassLimited =
        diagnostic.donorMassLimited || donorLimited;
    diagnostic.pressureEquilibriumLimited =
        diagnostic.pressureEquilibriumLimited || equilibriumLimited;
}

}  // namespace

void initializeCellAirMass(
    CellAirState &state,
    std::span<const CellAirVolume> cells,
    std::span<const double> gaugePressurePascal,
    const CellAirSettings &settings)
{
    validateSettings(settings);
    validateVolumes(cells);
    if (gaugePressurePascal.size() != cells.size()) {
        throw std::invalid_argument(
            "Cell-air pressure and volume counts must match");
    }

    state.massKg.resize(cells.size());
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const double pressure = settings.ambientAbsolutePressurePascal
                                + gaugePressurePascal[i];
        if (!finitePositive(pressure)) {
            throw std::invalid_argument(
                "Initial cell-air absolute pressure must be positive");
        }
        const double mass = pressure * cells[i].cubicMetres
                            / (settings.air.specificGasConstant
                               * settings.cellTemperatureKelvin);
        if (!std::isfinite(mass) || mass < settings.massFloorKg) {
            throw std::invalid_argument(
                "Initial cell-air mass falls below its mass floor");
        }
        state.massKg[i] = mass;
    }
    state.referenceFiniteMassKg = sumMass(state.massKg);
    state.cumulativeReservoirInflowKg = 0.0;
    state.massReferenceInitialized = true;
}

std::vector<double> cellAirGaugePressures(
    const CellAirState &state,
    std::span<const CellAirVolume> cells,
    const CellAirSettings &settings)
{
    validateSettings(settings);
    validateVolumes(cells);
    validateState(state, cells, settings);
    std::vector<double> result(cells.size());
    for (std::size_t i = 0; i < cells.size(); ++i) {
        result[i] = absolutePressure(
                        state.massKg[i], cells[i].cubicMetres, settings)
                    - settings.ambientAbsolutePressurePascal;
    }
    return result;
}

CellAirStepDiagnostics advanceCellAirMass(
    CellAirState &state,
    std::span<const CellAirVolume> cells,
    std::span<const CellAirReservoirPort> reservoirPorts,
    std::span<const CellAirCrossPort> crossPorts,
    double timeStepSeconds,
    const CellAirSettings &settings)
{
    validateSettings(settings);
    validateVolumes(cells);
    validateState(state, cells, settings);
    if (!finitePositive(timeStepSeconds)) {
        throw std::invalid_argument(
            "Cell-air timestep must be positive and finite");
    }
    for (const CellAirReservoirPort &port : reservoirPorts) {
        if (port.cell >= cells.size()
            || !finitePositive(port.absolutePressurePascal)
            || !finitePositive(port.absoluteTemperatureKelvin)) {
            throw std::invalid_argument("Invalid cell-air reservoir port");
        }
        validateOrifice(port.effectiveAreaSquareMetres,
                        port.openingFraction,
                        port.dischargeCoefficient);
    }
    for (const CellAirCrossPort &port : crossPorts) {
        if (port.firstCell >= cells.size()
            || port.secondCell >= cells.size()
            || port.firstCell == port.secondCell) {
            throw std::invalid_argument("Invalid cell-air cross-port cells");
        }
        validateOrifice(port.effectiveAreaSquareMetres,
                        port.openingFraction,
                        port.dischargeCoefficient);
    }

    if (!state.massReferenceInitialized) {
        state.referenceFiniteMassKg = sumMass(state.massKg);
        state.cumulativeReservoirInflowKg = 0.0;
        state.massReferenceInitialized = true;
    }

    const std::vector<double> initialMass = state.massKg;
    CellAirStepDiagnostics result;
    result.cells.resize(cells.size());
    result.reservoirPorts.resize(reservoirPorts.size());
    result.crossPorts.resize(crossPorts.size());
    result.substeps = settings.substeps;
    const double substepSeconds =
        timeStepSeconds / static_cast<double>(settings.substeps);

    for (int substep = 0; substep < settings.substeps; ++substep) {
        std::vector<double> pressure(cells.size());
        for (std::size_t i = 0; i < cells.size(); ++i) {
            pressure[i] = absolutePressure(
                state.massKg[i], cells[i].cubicMetres, settings);
        }

        std::vector<Edge> edges;
        edges.reserve(crossPorts.size() + reservoirPorts.size());
        for (std::size_t i = 0; i < crossPorts.size(); ++i) {
            const CellAirCrossPort &port = crossPorts[i];
            Edge edge;
            edge.kind = EdgeKind::CrossPort;
            edge.publicIndex = i;
            edge.firstCell = port.firstCell;
            edge.secondCell = port.secondCell;
            edge.area = port.effectiveAreaSquareMetres;
            edge.opening = port.openingFraction;
            edge.discharge = port.dischargeCoefficient;
            edge.evaluation = evaluatePort(
                {pressure[port.firstCell], settings.cellTemperatureKelvin},
                {pressure[port.secondCell], settings.cellTemperatureKelvin},
                edge.area, edge.opening, edge.discharge, settings);
            edge.rawTransferKg =
                edge.evaluation.signedMassRate * substepSeconds;
            edges.push_back(edge);
        }
        for (std::size_t i = 0; i < reservoirPorts.size(); ++i) {
            const CellAirReservoirPort &port = reservoirPorts[i];
            Edge edge;
            edge.kind = EdgeKind::ReservoirPort;
            edge.publicIndex = i;
            edge.secondCell = port.cell;
            edge.reservoirPressure = port.absolutePressurePascal;
            edge.reservoirTemperature = port.absoluteTemperatureKelvin;
            edge.area = port.effectiveAreaSquareMetres;
            edge.opening = port.openingFraction;
            edge.discharge = port.dischargeCoefficient;
            edge.evaluation = evaluatePort(
                {edge.reservoirPressure, edge.reservoirTemperature},
                {pressure[port.cell], settings.cellTemperatureKelvin},
                edge.area, edge.opening, edge.discharge, settings);
            edge.rawTransferKg =
                edge.evaluation.signedMassRate * substepSeconds;
            edges.push_back(edge);
        }
        std::sort(edges.begin(), edges.end(), [](const Edge &left,
                                                 const Edge &right) {
            return edgeSortKey(left) < edgeSortKey(right);
        });

        DisjointSet components(cells.size());
        for (const Edge &edge : edges) {
            if (edge.kind == EdgeKind::CrossPort) {
                components.unite(edge.firstCell, edge.secondCell);
            }
        }
        std::vector<long double> rawNetMass(cells.size(), 0.0L);
        for (const Edge &edge : edges) {
            if (edge.kind == EdgeKind::CrossPort) {
                rawNetMass[edge.firstCell] -= edge.rawTransferKg;
                rawNetMass[edge.secondCell] += edge.rawTransferKg;
            } else {
                rawNetMass[edge.secondCell] += edge.rawTransferKg;
            }
        }

        std::vector<double> componentScale(cells.size(), 1.0);
        std::vector<bool> donorLimited(cells.size(), false);
        std::vector<bool> equilibriumLimited(cells.size(), false);
        const auto applyLimit = [&](std::size_t cell, double candidate,
                                    bool donor) {
            const std::size_t root = components.find(cell);
            candidate = std::clamp(candidate, 0.0, 1.0);
            if (candidate < 1.0) {
                candidate = std::nextafter(candidate, 0.0);
            }
            if (candidate < componentScale[root]) {
                componentScale[root] = candidate;
                donorLimited[root] = donor;
                equilibriumLimited[root] = !donor;
            } else if (candidate == componentScale[root]
                       && candidate < 1.0) {
                donorLimited[root] = donorLimited[root] || donor;
                equilibriumLimited[root] =
                    equilibriumLimited[root] || !donor;
            }
        };

        for (std::size_t i = 0; i < cells.size(); ++i) {
            if (rawNetMass[i] < 0.0L) {
                const long double available =
                    static_cast<long double>(state.massKg[i]
                                             - settings.massFloorKg);
                applyLimit(i,
                           static_cast<double>(
                               available / -rawNetMass[i]),
                           true);
            }
        }
        const auto pressureSlope = [&](std::size_t cell) {
            return static_cast<double>(rawNetMass[cell])
                   * settings.air.specificGasConstant
                   * settings.cellTemperatureKelvin
                   / cells[cell].cubicMetres;
        };
        for (const Edge &edge : edges) {
            if (edge.rawTransferKg == 0.0) {
                continue;
            }
            const double firstPressure =
                edge.kind == EdgeKind::CrossPort
                    ? pressure[edge.firstCell]
                    : edge.reservoirPressure;
            const double secondPressure = pressure[edge.secondCell];
            const double firstSlope =
                edge.kind == EdgeKind::CrossPort
                    ? pressureSlope(edge.firstCell)
                    : 0.0;
            const double difference = firstPressure - secondPressure;
            const double differenceSlope =
                firstSlope - pressureSlope(edge.secondCell);
            if (difference * differenceSlope < 0.0
                && std::abs(differenceSlope) > std::abs(difference)) {
                applyLimit(edge.secondCell,
                           std::abs(difference / differenceSlope), false);
            }
        }

        std::vector<long double> appliedNetMass(cells.size(), 0.0L);
        long double reservoirInflow = 0.0L;
        for (const Edge &edge : edges) {
            const std::size_t root = components.find(edge.secondCell);
            const double scale = componentScale[root];
            const double applied = edge.rawTransferKg * scale;
            if (edge.kind == EdgeKind::CrossPort) {
                appliedNetMass[edge.firstCell] -= applied;
                appliedNetMass[edge.secondCell] += applied;
                recordPort(result.crossPorts[edge.publicIndex], edge,
                           applied, scale, donorLimited[root],
                           equilibriumLimited[root]);
            } else {
                appliedNetMass[edge.secondCell] += applied;
                reservoirInflow += applied;
                recordPort(result.reservoirPorts[edge.publicIndex], edge,
                           applied, scale, donorLimited[root],
                           equilibriumLimited[root]);
            }
            result.minimumLimiterScale =
                std::min(result.minimumLimiterScale, scale);
        }

        for (std::size_t i = 0; i < cells.size(); ++i) {
            state.massKg[i] = static_cast<double>(
                static_cast<long double>(state.massKg[i])
                + appliedNetMass[i]);
            if (!std::isfinite(state.massKg[i])
                || state.massKg[i] < settings.massFloorKg) {
                throw std::runtime_error(
                    "Conservative cell-air limiter produced invalid mass");
            }
        }
        const double reservoirDelta =
            static_cast<double>(reservoirInflow);
        state.cumulativeReservoirInflowKg += reservoirDelta;
        result.reservoirInflowKg += reservoirDelta;
    }

    result.limiterActive = result.minimumLimiterScale < 1.0;
    result.finiteCellMassKg = sumMass(state.massKg);
    result.cumulativeReservoirInflowKg =
        state.cumulativeReservoirInflowKg;
    result.massResidualKg =
        result.finiteCellMassKg - state.referenceFiniteMassKg
        - state.cumulativeReservoirInflowKg;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        CellAirCellDiagnostics &diagnostic = result.cells[i];
        diagnostic.massKg = state.massKg[i];
        diagnostic.volumeCubicMetres = cells[i].cubicMetres;
        diagnostic.absolutePressurePascal = absolutePressure(
            state.massKg[i], cells[i].cubicMetres, settings);
        diagnostic.gaugePressurePascal =
            diagnostic.absolutePressurePascal
            - settings.ambientAbsolutePressurePascal;
        diagnostic.netMassRateKgPerSecond =
            (state.massKg[i] - initialMass[i]) / timeStepSeconds;
    }
    for (CellAirPortDiagnostics &diagnostic : result.reservoirPorts) {
        diagnostic.appliedMassRateKgPerSecond =
            diagnostic.appliedTransferKg / timeStepSeconds;
    }
    for (CellAirPortDiagnostics &diagnostic : result.crossPorts) {
        diagnostic.appliedMassRateKgPerSecond =
            diagnostic.appliedTransferKg / timeStepSeconds;
    }
    return result;
}

}  // namespace lep::playground
