#include "softwing/pneumatics.h"

#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
#include "aerodynamic_test_access.h"
#endif
#include "flight_state_access.h"
#include "softwing/suspension.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <numeric>
#include <sstream>
#include <tuple>

namespace softwing {
namespace {

using PerformanceClock = std::chrono::steady_clock;

class PerformanceScope {
public:
    explicit PerformanceScope(std::uint64_t* destination) noexcept
        : destination_(destination),
          start_(destination ? PerformanceClock::now()
                             : PerformanceClock::time_point{}) {}

    ~PerformanceScope() {
        if (!destination_) return;
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
            PerformanceClock::now() - start_).count();
        *destination_ += static_cast<std::uint64_t>(elapsed);
    }

private:
    std::uint64_t* destination_ = nullptr;
    PerformanceClock::time_point start_;
};

[[nodiscard]] std::uint64_t* profileField(
    StepPerformanceProfile* profile,
    std::uint64_t StepPerformanceProfile::* field) noexcept {
    return profile ? &(profile->*field) : nullptr;
}

bool finitePositive(double value) {
    return value > 0.0 && std::isfinite(value);
}

bool finiteVector(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

void validateFlowState(const FlowZoneState& state) {
    if (!finitePositive(state.absolutePressure) ||
        !finitePositive(state.absoluteTemperature)) {
        throw std::invalid_argument(
            "Pneumatic flow state requires positive finite absolute pressure and temperature");
    }
}

void validateOpening(double opening) {
    if (!std::isfinite(opening) || opening < 0.0 || opening > 1.0) {
        throw std::invalid_argument(
            "Pneumatic port opening fraction must be finite and in [0,1]");
    }
}

double faceVolumeContribution(const Node& a,
                              const Node& b,
                              const Node& c) {
    return dot(a.position, cross(b.position, c.position)) / 6.0;
}

struct DisjointSet {
    explicit DisjointSet(std::size_t count) : parent(count), rank(count, 0) {
        std::iota(parent.begin(), parent.end(), std::size_t{0});
    }

    std::size_t find(std::size_t value) {
        if (parent[value] != value) {
            parent[value] = find(parent[value]);
        }
        return parent[value];
    }

    void unite(std::size_t first, std::size_t second) {
        first = find(first);
        second = find(second);
        if (first == second) {
            return;
        }
        if (rank[first] < rank[second]) {
            std::swap(first, second);
        }
        parent[second] = first;
        if (rank[first] == rank[second]) {
            ++rank[first];
        }
    }

    std::vector<std::size_t> parent;
    std::vector<unsigned char> rank;
};

} // namespace

void AirProperties::validate() const {
    if (!finitePositive(specificGasConstant) ||
        !(heatCapacityRatio > 1.0) ||
        !std::isfinite(heatCapacityRatio)) {
        throw std::invalid_argument(
            "Air properties require finite R>0 and gamma>1");
    }
}

double AirProperties::cv() const {
    validate();
    return specificGasConstant / (heatCapacityRatio - 1.0);
}

double AirProperties::cp() const {
    validate();
    return heatCapacityRatio * specificGasConstant /
           (heatCapacityRatio - 1.0);
}

MassFlowEvaluation evaluateOrificeMassFlow(
    const FlowZoneState& first,
    const FlowZoneState& second,
    const AirProperties& air,
    const OrificeDefinition& definition) {
    air.validate();
    validateFlowState(first);
    validateFlowState(second);
    validateOpening(definition.openingFraction);
    if (!(definition.dischargeCoefficient > 0.0) ||
        definition.dischargeCoefficient > 1.0 ||
        !std::isfinite(definition.dischargeCoefficient) ||
        !finitePositive(definition.effectiveArea)) {
        throw std::invalid_argument(
            "Orifice requires finite 0<Cd<=1 and effective area>0");
    }

    MassFlowEvaluation result;
    const double gamma = air.heatCapacityRatio;
    result.criticalPressureRatio =
        std::pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0));
    if (first.absolutePressure == second.absolutePressure ||
        definition.openingFraction == 0.0) {
        return result;
    }

    const bool firstUpstream =
        first.absolutePressure > second.absolutePressure;
    const FlowZoneState& upstream = firstUpstream ? first : second;
    const FlowZoneState& downstream = firstUpstream ? second : first;
    result.direction = firstUpstream ? FlowDirection::FirstToSecond
                                     : FlowDirection::SecondToFirst;
    result.pressureRatio =
        std::clamp(downstream.absolutePressure / upstream.absolutePressure,
                   0.0,
                   1.0);

    const double pressureTemperatureScale =
        upstream.absolutePressure /
        std::sqrt(air.specificGasConstant * upstream.absoluteTemperature);
    double flux = 0.0;
    if (result.pressureRatio <= result.criticalPressureRatio) {
        result.choked = true;
        flux = pressureTemperatureScale * std::sqrt(gamma) *
               std::pow(2.0 / (gamma + 1.0),
                        (gamma + 1.0) / (2.0 * (gamma - 1.0)));
    } else {
        const double firstPower =
            std::pow(result.pressureRatio, 2.0 / gamma);
        const double secondPower =
            std::pow(result.pressureRatio, (gamma + 1.0) / gamma);
        const double radicand =
            std::max(0.0,
                     (2.0 * gamma / (gamma - 1.0)) *
                         (firstPower - secondPower));
        flux = pressureTemperatureScale * std::sqrt(radicand);
    }
    result.magnitude = definition.dischargeCoefficient *
                       definition.effectiveArea *
                       definition.openingFraction * flux;
    if (!std::isfinite(result.magnitude)) {
        throw std::invalid_argument("Orifice flow evaluation became non-finite");
    }
    result.signedMassRate = firstUpstream ? result.magnitude : -result.magnitude;
    return result;
}

MassFlowEvaluation evaluateConductanceMassFlow(
    const FlowZoneState& first,
    const FlowZoneState& second,
    const ConductanceDefinition& definition) {
    validateFlowState(first);
    validateFlowState(second);
    validateOpening(definition.openingFraction);
    if (!finitePositive(definition.massConductance)) {
        throw std::invalid_argument(
            "Linear conductance requires finite positive kg/(s*Pa)");
    }
    MassFlowEvaluation result;
    if (first.absolutePressure == second.absolutePressure ||
        definition.openingFraction == 0.0) {
        return result;
    }
    result.signedMassRate = definition.massConductance *
                            definition.openingFraction *
                            (first.absolutePressure - second.absolutePressure);
    result.magnitude = std::abs(result.signedMassRate);
    result.direction = result.signedMassRate > 0.0
                           ? FlowDirection::FirstToSecond
                           : FlowDirection::SecondToFirst;
    const double high =
        std::max(first.absolutePressure, second.absolutePressure);
    const double low =
        std::min(first.absolutePressure, second.absolutePressure);
    result.pressureRatio = low / high;
    if (!std::isfinite(result.signedMassRate)) {
        throw std::invalid_argument(
            "Linear conductance flow evaluation became non-finite");
    }
    return result;
}

struct PneumaticNetwork::GasCell {
    GasCellDefinition definition;
    double volume = std::numeric_limits<double>::quiet_NaN();
    double absolutePressure = std::numeric_limits<double>::quiet_NaN();
    double minimumVolume = std::numeric_limits<double>::infinity();
    PneumaticTopologyStatus topology;
};

struct PneumaticNetwork::Port {
    PneumaticPortKind kind = PneumaticPortKind::Orifice;
    PneumaticZoneKind firstKind = PneumaticZoneKind::GasCell;
    std::size_t firstIndex = 0;
    PneumaticZoneKind secondKind = PneumaticZoneKind::Reservoir;
    std::size_t secondIndex = 0;
    OrificeDefinition orifice;
    ConductanceDefinition conductance;
};

PneumaticNetwork::PneumaticNetwork(const PneumaticNetwork &other)
    : air_(other.air_), bodyOwner_(other.bodyOwner_), cells_(other.cells_),
      reservoirs_(other.reservoirs_), interfaces_(other.interfaces_),
      ports_(other.ports_), cellDiagnostics_(other.cellDiagnostics_),
      portDiagnostics_(other.portDiagnostics_),
      interfaceDiagnostics_(other.interfaceDiagnostics_),
      diagnostics_(other.diagnostics_),
      initialFiniteMass_(other.initialFiniteMass_),
      massReferenceInitialized_(other.massReferenceInitialized_),
      topologyValidated_(other.topologyValidated_),
      certifiedCellTopologies_(other.certifiedCellTopologies_) {}

PneumaticNetwork &PneumaticNetwork::operator=(const PneumaticNetwork &other) {
  if (this == &other)
    return *this;
  air_ = other.air_;
  bodyOwner_ = other.bodyOwner_;
  cells_ = other.cells_;
  reservoirs_ = other.reservoirs_;
  interfaces_ = other.interfaces_;
  ports_ = other.ports_;
  cellDiagnostics_ = other.cellDiagnostics_;
  portDiagnostics_ = other.portDiagnostics_;
  interfaceDiagnostics_ = other.interfaceDiagnostics_;
  diagnostics_ = other.diagnostics_;
  initialFiniteMass_ = other.initialFiniteMass_;
  massReferenceInitialized_ = other.massReferenceInitialized_;
  topologyValidated_ = other.topologyValidated_;
  certifiedCellTopologies_ = other.certifiedCellTopologies_;
  return *this;
}

PneumaticNetwork::~PneumaticNetwork() = default;

PneumaticNetwork::PneumaticNetwork(AirProperties air) : air_(air) {
  air_.validate();
}

PneumaticZoneHandle PneumaticNetwork::addGasCell(
    const GasCellDefinition& definition) {
    if (!finitePositive(definition.mass) ||
        !finitePositive(definition.absoluteTemperature) ||
        !finitePositive(definition.massFloor) ||
        !finitePositive(definition.volumeFloor) ||
        !(definition.mass > definition.massFloor)) {
        throw std::invalid_argument(
            "Gas cell requires finite positive mass/temperature/floors and mass>floor");
    }
    const std::size_t index = cells_.size();
    cells_.push_back({definition});
    cellDiagnostics_.push_back({});
    topologyValidated_ = false;
    certifiedCellTopologies_.clear();
    massReferenceInitialized_ = false;
    return {this, PneumaticZoneKind::GasCell, index};
}

PneumaticZoneHandle PneumaticNetwork::addReservoir(
    const ReservoirDefinition& definition) {
    if (!finitePositive(definition.absolutePressure) ||
        !finitePositive(definition.absoluteTemperature)) {
        throw std::invalid_argument(
            "Reservoir requires positive finite absolute pressure and temperature");
    }
    const std::size_t index = reservoirs_.size();
    reservoirs_.push_back(definition);
    return {this, PneumaticZoneKind::Reservoir, index};
}

void PneumaticNetwork::requireZone(const PneumaticZoneHandle& zone) const {
    if (zone.owner_ != this) {
        throw std::invalid_argument(
            "Pneumatic zone belongs to another network");
    }
    const std::size_t count = zone.kind_ == PneumaticZoneKind::GasCell
                                  ? cells_.size()
                                  : reservoirs_.size();
    if (zone.index_ >= count) {
        throw std::out_of_range("Pneumatic zone handle is out of range");
    }
}

void PneumaticNetwork::requireCell(const PneumaticZoneHandle& cell) const {
    requireZone(cell);
    if (cell.kind_ != PneumaticZoneKind::GasCell) {
        throw std::invalid_argument("Pneumatic zone is not a finite gas cell");
    }
}

void PneumaticNetwork::requirePort(const PneumaticPortHandle& port) const {
    if (port.owner_ != this) {
        throw std::invalid_argument(
            "Pneumatic port belongs to another network");
    }
    if (port.index_ >= ports_.size()) {
        throw std::out_of_range("Pneumatic port handle is out of range");
    }
}

PressureInterfaceHandle PneumaticNetwork::addPressureInterface(
    const SoftBody& body,
    std::size_t triangle,
    const PneumaticZoneHandle& negativeSide,
    const PneumaticZoneHandle& positiveSide) {
    requireZone(negativeSide);
    requireZone(positiveSide);
    if (triangle >= body.triangles().size()) {
        throw std::out_of_range("Pneumatic interface triangle is out of range");
    }
    if (negativeSide.kind_ == positiveSide.kind_ &&
        negativeSide.index_ == positiveSide.index_) {
        throw std::invalid_argument(
            "Pneumatic interface sides must be distinct zones");
    }
    if (bodyOwner_ != nullptr && bodyOwner_ != &body) {
        throw std::invalid_argument(
            "Pneumatic interfaces cannot mix SoftBody owners");
    }
    if (std::any_of(interfaces_.begin(),
                    interfaces_.end(),
                    [triangle](const PressureInterface& interfaceRecord) {
                        return interfaceRecord.triangle == triangle;
                    })) {
        throw std::invalid_argument(
            "A triangle may have at most one pneumatic interface");
    }

    const std::size_t index = interfaces_.size();
    interfaces_.push_back({triangle,
                           negativeSide.kind_,
                           negativeSide.index_,
                           positiveSide.kind_,
                           positiveSide.index_});
    interfaceDiagnostics_.push_back({});
    bodyOwner_ = &body;
    topologyValidated_ = false;
    certifiedCellTopologies_.clear();
    return {this, index};
}

PneumaticPortHandle PneumaticNetwork::addOrificePort(
    const PneumaticZoneHandle& first,
    const PneumaticZoneHandle& second,
    const OrificeDefinition& definition) {
    requireZone(first);
    requireZone(second);
    static_cast<void>(evaluateOrificeMassFlow(
        {101325.0, standardFixtureTemperature},
        {101324.0, standardFixtureTemperature},
        air_,
        definition));
    if (first.kind_ == second.kind_ && first.index_ == second.index_) {
        throw std::invalid_argument("Pneumatic port endpoints must be distinct");
    }
    if (first.kind_ == PneumaticZoneKind::Reservoir &&
        second.kind_ == PneumaticZoneKind::Reservoir) {
        throw std::invalid_argument(
            "At least one pneumatic port endpoint must be a finite cell");
    }
    const std::size_t index = ports_.size();
    Port port;
    port.kind = PneumaticPortKind::Orifice;
    port.firstKind = first.kind_;
    port.firstIndex = first.index_;
    port.secondKind = second.kind_;
    port.secondIndex = second.index_;
    port.orifice = definition;
    ports_.push_back(port);
    portDiagnostics_.push_back({});
    return {this, index};
}

PneumaticPortHandle PneumaticNetwork::addConductancePort(
    const PneumaticZoneHandle& first,
    const PneumaticZoneHandle& second,
    const ConductanceDefinition& definition) {
    requireZone(first);
    requireZone(second);
    static_cast<void>(evaluateConductanceMassFlow(
        {101325.0, standardFixtureTemperature},
        {101324.0, standardFixtureTemperature},
        definition));
    if (first.kind_ == second.kind_ && first.index_ == second.index_) {
        throw std::invalid_argument("Pneumatic port endpoints must be distinct");
    }
    if (first.kind_ == PneumaticZoneKind::Reservoir &&
        second.kind_ == PneumaticZoneKind::Reservoir) {
        throw std::invalid_argument(
            "At least one pneumatic port endpoint must be a finite cell");
    }
    const std::size_t index = ports_.size();
    Port port;
    port.kind = PneumaticPortKind::LinearConductance;
    port.firstKind = first.kind_;
    port.firstIndex = first.index_;
    port.secondKind = second.kind_;
    port.secondIndex = second.index_;
    port.conductance = definition;
    ports_.push_back(port);
    portDiagnostics_.push_back({});
    return {this, index};
}

void PneumaticNetwork::setPortOpening(const PneumaticPortHandle& portHandle,
                                      double openingFraction) {
    requirePort(portHandle);
    setPortOpening(portHandle.index_, openingFraction);
}

void PneumaticNetwork::setPortOpening(std::size_t portIndex,
                                      double openingFraction) {
    validateOpening(openingFraction);
    if (portIndex >= ports_.size()) {
        throw std::out_of_range("Pneumatic port index is out of range");
    }
    Port& port = ports_[portIndex];
    if (port.kind == PneumaticPortKind::Orifice) {
        port.orifice.openingFraction = openingFraction;
    } else {
        port.conductance.openingFraction = openingFraction;
    }
}

void PneumaticNetwork::setReservoirState(
    std::size_t reservoirIndex,
    const ReservoirDefinition& definition) {
    if (!finitePositive(definition.absolutePressure) ||
        !finitePositive(definition.absoluteTemperature)) {
        throw std::invalid_argument(
            "Reservoir requires positive finite absolute pressure and temperature");
    }
    if (reservoirIndex >= reservoirs_.size()) {
        throw std::out_of_range("Pneumatic reservoir index is out of range");
    }
    reservoirs_[reservoirIndex] = definition;
}

void PneumaticNetwork::validate(const SoftBody& body) {
    if (bodyOwner_ != nullptr && bodyOwner_ != &body) {
        throw std::invalid_argument(
            "Pneumatic network belongs to another SoftBody");
    }
    if (cells_.empty()) {
        throw std::invalid_argument(
            "Pneumatic network requires at least one finite gas cell");
    }

    const std::vector<CertifiedCellGeometry> certified =
        certifyCurrentGeometry(body);
    bodyOwner_ = &body;
    for (std::size_t i = 0; i < cells_.size(); ++i) {
        GasCell& cell = cells_[i];
        cell.topology = certified[i].topology;
        cell.volume = certified[i].volume;
        cell.minimumVolume = std::min(cell.minimumVolume, cell.volume);
        cell.absolutePressure = certified[i].absolutePressure;
    }
    certifiedCellTopologies_.resize(certified.size());
    for (std::size_t i = 0; i < certified.size(); ++i)
        certifiedCellTopologies_[i] = certified[i].topology.surface;
    topologyValidated_ = true;
    if (!massReferenceInitialized_) {
        initialFiniteMass_ = 0.0;
        for (const GasCell& cell : cells_) {
            initialFiniteMass_ += cell.definition.mass;
        }
        diagnostics_.cumulativeReservoirInflow = 0.0;
        massReferenceInitialized_ = true;
    }
    refreshNetworkDiagnostics();
}

std::vector<PneumaticNetwork::CertifiedCellGeometry>
PneumaticNetwork::certifyCurrentGeometry(const SoftBody &body,
                                         WorkerPool *workerPool) const {
  if (bodyOwner_ != nullptr && bodyOwner_ != &body) {
    throw std::invalid_argument(
        "Pneumatic network belongs to another SoftBody");
  }
  // Preserve the old per-cell triangle order while replacing its
  // cells*interfaces scan with one global pass. The inventory is staged
  // before any result is published, so allocation failure is atomic.
  geometryTriangleScratch_.resize(cells_.size());
  for (std::vector<Triangle> &triangles : geometryTriangleScratch_)
    triangles.clear();
  for (const PressureInterface &interfaceRecord : interfaces_) {
    const Triangle &source = body.triangles()[interfaceRecord.triangle];
    if (interfaceRecord.negativeKind == PneumaticZoneKind::GasCell) {
      geometryTriangleScratch_[interfaceRecord.negativeIndex].push_back(source);
    }
    if (interfaceRecord.positiveKind == PneumaticZoneKind::GasCell) {
      Triangle reversed = source;
      std::swap(reversed.b, reversed.c);
      geometryTriangleScratch_[interfaceRecord.positiveIndex].push_back(
          reversed);
    }
  }

  std::vector<CertifiedCellGeometry> certified(cells_.size());
  const auto certifyRange = [&](std::size_t begin, std::size_t end) {
    for (std::size_t cellIndex = begin; cellIndex < end; ++cellIndex) {
      const std::vector<Triangle> &orientedTriangles =
          geometryTriangleScratch_[cellIndex];
      CertifiedCellGeometry &result = certified[cellIndex];
      result.topology.incidentInterfaces = orientedTriangles.size();
      const bool hasCertifiedTopology =
          topologyValidated_ &&
          certifiedCellTopologies_.size() == cells_.size();
      bool currentFacesRegular = hasCertifiedTopology;
      double volume = 0.0;
      for (const Triangle &triangle : orientedTriangles) {
        if (triangle.a >= body.nodes().size() ||
            triangle.b >= body.nodes().size() ||
            triangle.c >= body.nodes().size()) {
          currentFacesRegular = false;
          continue;
        }
        if (hasCertifiedTopology) {
          const Vec3 ab = body.nodes()[triangle.b].position -
                          body.nodes()[triangle.a].position;
          const Vec3 ac = body.nodes()[triangle.c].position -
                          body.nodes()[triangle.a].position;
          const Vec3 bc = body.nodes()[triangle.c].position -
                          body.nodes()[triangle.b].position;
          const double maximumEdgeLengthSquared = std::max(
              {lengthSquared(ab), lengthSquared(ac), lengthSquared(bc)});
          const double twiceAreaSquared = lengthSquared(cross(ab, ac));
          const double areaScale =
              maximumEdgeLengthSquared * maximumEdgeLengthSquared;
          constexpr double relativeAreaTolerance =
              64.0 * std::numeric_limits<double>::epsilon();
          const bool repeatedNode = triangle.a == triangle.b ||
                                    triangle.b == triangle.c ||
                                    triangle.c == triangle.a;
          if (repeatedNode || !std::isfinite(twiceAreaSquared) ||
              !(maximumEdgeLengthSquared > 0.0) ||
              twiceAreaSquared <=
                  relativeAreaTolerance * relativeAreaTolerance * areaScale)
            currentFacesRegular = false;
        }
        volume += faceVolumeContribution(body.nodes()[triangle.a],
                                         body.nodes()[triangle.b],
                                         body.nodes()[triangle.c]);
      }
      if (currentFacesRegular) {
        result.topology.surface = certifiedCellTopologies_[cellIndex];
      } else {
        result.topology.surface = softwing::validateSurfaceTopology(
            std::span<const Node>{body.nodes()},
            std::span<const Triangle>{orientedTriangles});
      }
      result.volume = volume;
      result.absolutePressure =
          cells_[cellIndex].definition.mass * air_.specificGasConstant *
          cells_[cellIndex].definition.absoluteTemperature / volume;
      result.topology.valid =
          result.topology.surface.valid() && !orientedTriangles.empty() &&
          std::isfinite(volume) &&
          volume > cells_[cellIndex].definition.volumeFloor &&
          finitePositive(result.absolutePressure);
      if (!result.topology.valid) {
        std::ostringstream message;
        message << "Invalid pneumatic cell boundary for cell " << cellIndex
                << ": volume=" << volume
                << ", incident=" << orientedTriangles.size()
                << ", degenerate=" << result.topology.surface.degenerateFaces
                << ", boundary_edges=" << result.topology.surface.boundaryEdges
                << ", non_manifold=" << result.topology.surface.nonManifoldEdges
                << ", directed_edge_errors="
                << result.topology.surface.inconsistentDirectedEdges;
        throw std::invalid_argument(message.str());
      }
    }
  };
  if (workerPool && cells_.size() > 1) {
    workerPool->forEachRange(cells_.size(), certifyRange);
  } else {
    certifyRange(0, cells_.size());
  }
  return certified;
}

void PneumaticNetwork::initializeCellMassFromPressure(
    const SoftBody& body,
    const PneumaticZoneHandle& cellHandle,
    double absolutePressure) {
    requireCell(cellHandle);
    if (!finitePositive(absolutePressure)) {
        throw std::invalid_argument(
            "Cell initialization pressure must be positive and finite");
    }
    validate(body);
    GasCell& cell = cells_[cellHandle.index_];
    const double mass = absolutePressure * cell.volume /
                        (air_.specificGasConstant *
                         cell.definition.absoluteTemperature);
    if (!finitePositive(mass) || !(mass > cell.definition.massFloor)) {
        throw std::invalid_argument(
            "Cell initialization pressure produces invalid mass");
    }
    cell.definition.mass = mass;
    cell.absolutePressure = absolutePressure;
    massReferenceInitialized_ = false;
    validate(body);
}

void PneumaticNetwork::initializeCellMassFromPressure(
    const SoftBody& body,
    std::size_t cellIndex,
    double absolutePressure) {
    if (cellIndex >= cells_.size()) {
        throw std::out_of_range("Pneumatic cell index out of range");
    }
    initializeCellMassFromPressure(
        body,
        PneumaticZoneHandle(this, PneumaticZoneKind::GasCell, cellIndex),
        absolutePressure);
}

double PneumaticNetwork::zonePressure(const PneumaticZoneHandle& zone) const {
    requireZone(zone);
    return zone.kind_ == PneumaticZoneKind::GasCell
               ? cells_[zone.index_].absolutePressure
               : reservoirs_[zone.index_].absolutePressure;
}

double PneumaticNetwork::zoneTemperature(const PneumaticZoneHandle& zone) const {
    requireZone(zone);
    return zone.kind_ == PneumaticZoneKind::GasCell
               ? cells_[zone.index_].definition.absoluteTemperature
               : reservoirs_[zone.index_].absoluteTemperature;
}

void PneumaticNetwork::synchronizeGeometry(const SoftBody& body,
                                           WorkerPool* workerPool) {
    if (!topologyValidated_) {
        validate(body);
        return;
    }
    std::vector<CertifiedCellGeometry> certified;
    try {
        certified = certifyCurrentGeometry(body, workerPool);
    } catch (const std::exception& error) {
        throw PneumaticStepError(
            std::string("Pneumatic geometry certification failed: ") +
            error.what());
    }
    for (std::size_t i = 0; i < cells_.size(); ++i) {
        GasCell& cell = cells_[i];
        const double oldPressure = cell.absolutePressure;
        const double deltaVolume = certified[i].volume - cell.volume;
        cell.volume = certified[i].volume;
        cell.topology = certified[i].topology;
        cell.minimumVolume = std::min(cell.minimumVolume, cell.volume);
        cell.absolutePressure = certified[i].absolutePressure;
        PneumaticCellDiagnostics& diagnostic = cellDiagnostics_[i];
        diagnostic.gasWork +=
            0.5 * (oldPressure + cell.absolutePressure) * deltaVolume;
    }
}

void PneumaticNetwork::advanceMassTransfers(double timeStep) {
    if (!finitePositive(timeStep)) {
        throw std::invalid_argument(
            "Pneumatic timestep must be positive and finite");
    }
    const std::size_t zoneCount = cells_.size() + reservoirs_.size();
    const auto flatIndex = [this](PneumaticZoneKind kind, std::size_t index) {
        return kind == PneumaticZoneKind::GasCell ? index
                                                  : cells_.size() + index;
    };
    const auto stateFor = [this](PneumaticZoneKind kind,
                                 std::size_t index) {
        if (kind == PneumaticZoneKind::GasCell) {
            return FlowZoneState{cells_[index].absolutePressure,
                                 cells_[index].definition.absoluteTemperature};
        }
        return FlowZoneState{reservoirs_[index].absolutePressure,
                             reservoirs_[index].absoluteTemperature};
    };

    DisjointSet components(zoneCount);
    std::vector<MassFlowEvaluation> evaluations(ports_.size());
    std::vector<double> rawDm(ports_.size(), 0.0);
    std::vector<long double> netDm(cells_.size(), 0.0L);
    for (std::size_t i = 0; i < ports_.size(); ++i) {
        const Port& port = ports_[i];
        const FlowZoneState first = stateFor(port.firstKind, port.firstIndex);
        const FlowZoneState second = stateFor(port.secondKind, port.secondIndex);
        evaluations[i] = port.kind == PneumaticPortKind::Orifice
                             ? evaluateOrificeMassFlow(first,
                                                       second,
                                                       air_,
                                                       port.orifice)
                             : evaluateConductanceMassFlow(first,
                                                           second,
                                                           port.conductance);
        rawDm[i] = evaluations[i].signedMassRate * timeStep;
        if (port.firstKind == PneumaticZoneKind::GasCell) {
            netDm[port.firstIndex] -= rawDm[i];
        }
        if (port.secondKind == PneumaticZoneKind::GasCell) {
            netDm[port.secondIndex] += rawDm[i];
        }
        components.unite(flatIndex(port.firstKind, port.firstIndex),
                         flatIndex(port.secondKind, port.secondIndex));
    }

    std::vector<double> componentScale(zoneCount, 1.0);
    std::vector<FlowLimiterCause> componentCause(
        zoneCount, FlowLimiterCause::None);
    const auto applyLimit = [&](std::size_t root,
                                double candidate,
                                FlowLimiterCause cause) {
        candidate = std::clamp(candidate, 0.0, 1.0);
        if (candidate < componentScale[root]) {
            componentScale[root] = candidate;
            componentCause[root] = cause;
        }
    };

    for (std::size_t i = 0; i < cells_.size(); ++i) {
        if (netDm[i] < 0.0L) {
            const double available =
                cells_[i].definition.mass - cells_[i].definition.massFloor;
            const double limit =
                available / static_cast<double>(-netDm[i]);
            applyLimit(components.find(i),
                       limit,
                       FlowLimiterCause::DonorMassFloor);
        }
    }
    for (std::size_t i = 0; i < ports_.size(); ++i) {
        if (rawDm[i] == 0.0) {
            continue;
        }
        const Port& port = ports_[i];
        const double firstPressure =
            stateFor(port.firstKind, port.firstIndex).absolutePressure;
        const double secondPressure =
            stateFor(port.secondKind, port.secondIndex).absolutePressure;
        const double initialDifference = firstPressure - secondPressure;
        const auto pressureSlope = [&](PneumaticZoneKind kind,
                                       std::size_t index) {
            if (kind == PneumaticZoneKind::Reservoir) {
                return 0.0;
            }
            const GasCell& cell = cells_[index];
            return static_cast<double>(netDm[index]) *
                   air_.specificGasConstant *
                   cell.definition.absoluteTemperature / cell.volume;
        };
        const double differenceSlope =
            pressureSlope(port.firstKind, port.firstIndex) -
            pressureSlope(port.secondKind, port.secondIndex);
        if (initialDifference * differenceSlope < 0.0 &&
            std::abs(differenceSlope) > std::abs(initialDifference)) {
            const double limit =
                std::abs(initialDifference / differenceSlope);
            applyLimit(components.find(
                           flatIndex(port.firstKind, port.firstIndex)),
                       limit,
                       FlowLimiterCause::PressureEquilibrium);
        }
    }

    std::vector<std::size_t> order(ports_.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(), [&](std::size_t left,
                                               std::size_t right) {
        const Port& a = ports_[left];
        const Port& b = ports_[right];
        return std::tuple{a.firstKind,
                          a.firstIndex,
                          a.secondKind,
                          a.secondIndex,
                          a.kind,
                          rawDm[left]} <
               std::tuple{b.firstKind,
                          b.firstIndex,
                          b.secondKind,
                          b.secondIndex,
                          b.kind,
                          rawDm[right]};
    });
    std::vector<long double> appliedNetDm(cells_.size(), 0.0L);
    std::vector<double> enthalpy(cells_.size(), 0.0);
    double reservoirInflow = 0.0;
    for (const std::size_t i : order) {
        const Port& port = ports_[i];
        const std::size_t root = components.find(
            flatIndex(port.firstKind, port.firstIndex));
        const double scale = componentScale[root];
        const double dm = rawDm[i] * scale;
        if (port.firstKind == PneumaticZoneKind::GasCell) {
            appliedNetDm[port.firstIndex] -= dm;
            const double sourceTemperature =
                dm >= 0.0
                    ? cells_[port.firstIndex].definition.absoluteTemperature
                    : stateFor(port.secondKind, port.secondIndex)
                          .absoluteTemperature;
            enthalpy[port.firstIndex] +=
                -dm * air_.cp() * sourceTemperature;
        }
        if (port.secondKind == PneumaticZoneKind::GasCell) {
            appliedNetDm[port.secondIndex] += dm;
            const double sourceTemperature =
                dm >= 0.0
                    ? stateFor(port.firstKind, port.firstIndex)
                          .absoluteTemperature
                    : cells_[port.secondIndex].definition.absoluteTemperature;
            enthalpy[port.secondIndex] +=
                dm * air_.cp() * sourceTemperature;
        }
        if (port.firstKind == PneumaticZoneKind::Reservoir &&
            port.secondKind == PneumaticZoneKind::GasCell) {
            reservoirInflow += dm;
        } else if (port.firstKind == PneumaticZoneKind::GasCell &&
                   port.secondKind == PneumaticZoneKind::Reservoir) {
            reservoirInflow -= dm;
        }

        PneumaticPortDiagnostics& diagnostic = portDiagnostics_[i];
        diagnostic.kind = port.kind;
        diagnostic.direction = evaluations[i].direction;
        diagnostic.rawMassRate = evaluations[i].signedMassRate;
        diagnostic.appliedMassRate = dm / timeStep;
        diagnostic.pressureRatio = evaluations[i].pressureRatio;
        diagnostic.choked = evaluations[i].choked;
        diagnostic.openingFraction =
            port.kind == PneumaticPortKind::Orifice
                ? port.orifice.openingFraction
                : port.conductance.openingFraction;
        diagnostic.limiterScale = scale;
        diagnostic.limiterCause = componentCause[root];
    }

    for (std::size_t i = 0; i < cells_.size(); ++i) {
        GasCell& cell = cells_[i];
        const double oldPressure = cell.absolutePressure;
        cell.definition.mass += static_cast<double>(appliedNetDm[i]);
        if (!(cell.definition.mass >= cell.definition.massFloor) ||
            !std::isfinite(cell.definition.mass)) {
            throw PneumaticStepError(
                "Conservative pneumatic limiter failed to retain valid mass");
        }
        cell.absolutePressure = cell.definition.mass *
                                air_.specificGasConstant *
                                cell.definition.absoluteTemperature / cell.volume;
        if (!finitePositive(cell.absolutePressure)) {
            throw PneumaticStepError(
                "Pneumatic transfer produced invalid pressure");
        }
        PneumaticCellDiagnostics& diagnostic = cellDiagnostics_[i];
        diagnostic.netMassRate =
            static_cast<double>(appliedNetDm[i]) / timeStep;
        diagnostic.enthalpyFlow += enthalpy[i];
        diagnostic.pressureRate =
            (cell.absolutePressure - oldPressure) / timeStep;
    }
    diagnostics_.cumulativeReservoirInflow += reservoirInflow;
}

void PneumaticNetwork::evaluatePressureDiagnostics(const SoftBody& body,
                                                    const Vec3& origin) {
    diagnostics_.pressureForce = {};
    diagnostics_.pressureMoment = {};
    for (std::size_t i = 0; i < interfaces_.size(); ++i) {
        const PressureInterface& interfaceRecord = interfaces_[i];
        const auto pressure = [this](PneumaticZoneKind kind,
                                     std::size_t index) {
            return kind == PneumaticZoneKind::GasCell
                       ? cells_[index].absolutePressure
                       : reservoirs_[index].absolutePressure;
        };
        const double difference =
            pressure(interfaceRecord.negativeKind,
                     interfaceRecord.negativeIndex) -
            pressure(interfaceRecord.positiveKind,
                     interfaceRecord.positiveIndex);
        const Triangle& triangle =
            body.triangles()[interfaceRecord.triangle];
        const Vec3& a = body.nodes()[triangle.a].position;
        const Vec3& b = body.nodes()[triangle.b].position;
        const Vec3& c = body.nodes()[triangle.c].position;
        const Vec3 force = difference * 0.5 * cross(b - a, c - a);
        const Vec3 centroid = (a + b + c) / 3.0;
        const Vec3 moment = cross(centroid - origin, force);
        PressureInterfaceDiagnostics& diagnostic = interfaceDiagnostics_[i];
        diagnostic.pressureDifference = difference;
        diagnostic.force = force;
        diagnostic.moment = moment;
        diagnostics_.pressureForce += force;
        diagnostics_.pressureMoment += moment;
    }
}

void PneumaticNetwork::refreshNetworkDiagnostics() {
    diagnostics_.topologyValid = topologyValidated_;
    diagnostics_.finiteCellMass = 0.0;
    diagnostics_.internalEnergyChange = 0.0;
    diagnostics_.advectedEnthalpy = 0.0;
    diagnostics_.gasWork = 0.0;
    diagnostics_.bathHeat = 0.0;
    diagnostics_.firstLawResidual = 0.0;
    diagnostics_.maximumFirstLawResidual = 0.0;
    diagnostics_.minimumLimiterScale = 1.0;
    diagnostics_.limiterActive = false;
    const double ambient = reservoirs_.empty()
                               ? 0.0
                               : reservoirs_.front().absolutePressure;
    for (std::size_t i = 0; i < cells_.size(); ++i) {
        const GasCell& cell = cells_[i];
        PneumaticCellDiagnostics& diagnostic = cellDiagnostics_[i];
        diagnostic.mass = cell.definition.mass;
        diagnostic.volume = cell.volume;
        diagnostic.absolutePressure = cell.absolutePressure;
        diagnostic.gaugePressure = cell.absolutePressure - ambient;
        diagnostic.absoluteTemperature = cell.definition.absoluteTemperature;
        diagnostic.internalEnergy =
            cell.definition.mass * air_.cv() *
            cell.definition.absoluteTemperature;
        diagnostic.minimumVolume = cell.minimumVolume;
        diagnostic.topology = cell.topology;
        diagnostics_.finiteCellMass += cell.definition.mass;
        diagnostics_.internalEnergyChange += diagnostic.internalEnergyChange;
        diagnostics_.advectedEnthalpy += diagnostic.enthalpyFlow;
        diagnostics_.gasWork += diagnostic.gasWork;
        diagnostics_.bathHeat += diagnostic.bathHeat;
        diagnostics_.firstLawResidual += diagnostic.firstLawResidual;
        diagnostics_.maximumFirstLawResidual =
            std::max(diagnostics_.maximumFirstLawResidual,
                     std::abs(diagnostic.firstLawResidual));
    }
    for (const PneumaticPortDiagnostics& diagnostic : portDiagnostics_) {
        diagnostics_.minimumLimiterScale =
            std::min(diagnostics_.minimumLimiterScale,
                     diagnostic.limiterScale);
        diagnostics_.limiterActive =
            diagnostics_.limiterActive || diagnostic.limiterScale < 1.0;
    }
    diagnostics_.massResidual =
        diagnostics_.finiteCellMass - initialFiniteMass_ -
        diagnostics_.cumulativeReservoirInflow;
    diagnostics_.couplingWorkMismatch =
        diagnostics_.nodalWork - diagnostics_.interfaceWork;
    diagnostics_.relativeCouplingWorkMismatch =
        std::abs(diagnostics_.couplingWorkMismatch) /
        std::max(1.0e-6, diagnostics_.cumulativeAbsoluteInterfaceWork);
}

void PneumaticNetwork::advanceRigid(const SoftBody& body, double timeStep) {
    if (!finitePositive(timeStep)) {
        throw std::invalid_argument(
            "Pneumatic timestep must be positive and finite");
    }
    const PneumaticNetwork saved = *this;
    try {
        if (!topologyValidated_) {
            validate(body);
        }
        std::vector<double> initialEnergy(cells_.size());
        for (std::size_t i = 0; i < cells_.size(); ++i) {
            initialEnergy[i] = cells_[i].definition.mass * air_.cv() *
                               cells_[i].definition.absoluteTemperature;
            cellDiagnostics_[i].netMassRate = 0.0;
            cellDiagnostics_[i].pressureRate = 0.0;
            cellDiagnostics_[i].enthalpyFlow = 0.0;
            cellDiagnostics_[i].internalEnergyChange = 0.0;
            cellDiagnostics_[i].gasWork = 0.0;
            cellDiagnostics_[i].bathHeat = 0.0;
            cellDiagnostics_[i].firstLawResidual = 0.0;
        }
        synchronizeGeometry(body);
        advanceMassTransfers(timeStep);
        for (std::size_t i = 0; i < cells_.size(); ++i) {
            PneumaticCellDiagnostics& diagnostic = cellDiagnostics_[i];
            const double finalEnergy = cells_[i].definition.mass * air_.cv() *
                                       cells_[i].definition.absoluteTemperature;
            const double deltaEnergy = finalEnergy - initialEnergy[i];
            diagnostic.internalEnergyChange = deltaEnergy;
            diagnostic.bathHeat = deltaEnergy - diagnostic.enthalpyFlow +
                                  diagnostic.gasWork;
            diagnostic.firstLawResidual =
                deltaEnergy - diagnostic.enthalpyFlow - diagnostic.bathHeat +
                diagnostic.gasWork;
        }
        diagnostics_.stepSucceeded = true;
        diagnostics_.failure.clear();
        evaluatePressureDiagnostics(body);
        refreshNetworkDiagnostics();
    } catch (const std::exception& error) {
        *this = saved;
        diagnostics_.stepSucceeded = false;
        diagnostics_.failure = error.what();
        throw;
    }
}

void PneumaticNetwork::coupledSubstep(
    SoftBody& body,
    const StepSettings& settings,
    double timeStep,
    std::span<const Vec3> persistentForces,
    SuspensionSystem* suspension) {
    StepPerformanceProfile* const profile = settings.performanceProfile;
    PerformanceScope total(profileField(
        profile, &StepPerformanceProfile::pneumaticTotalNanoseconds));
    if (profile) ++profile->pneumaticSubsteps;
    WorkerPool* geometryPool = nullptr;
    if (settings.workerThreads > 1 && cells_.size() > 1) {
        const unsigned cellWorkers = std::min(
            settings.workerThreads,
            static_cast<unsigned>(std::min<std::size_t>(
                cells_.size(), std::numeric_limits<unsigned>::max())));
        geometryPool = geometryWorkerPool_.get(cellWorkers);
    }
    const auto snapshotStart = profile ? PerformanceClock::now()
                                       : PerformanceClock::time_point{};
    const PneumaticNetwork savedNetwork = *this;
    const auto savedNodes = body.nodes_;
    const auto savedConstraints = body.constraints_;
    const auto savedMembranes = body.membraneElements_;
    const auto savedDihedrals = body.dihedralConstraints_;
    const auto savedMultipliers = body.contactMultipliers_;
    const auto savedRecords = body.contactRecords_;
    const auto savedContactDiagnostics = body.contactDiagnostics_;
    const auto savedPairDiagnostics = body.contactPairDiagnostics_;
    if (profile) {
        profile->pneumaticTransactionSnapshotNanoseconds +=
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    PerformanceClock::now() - snapshotStart).count());
    }

    try {
        std::vector<double> initialEnergy(cells_.size());
        std::vector<double> initialPressure(cells_.size());
        std::vector<double> initialEnthalpy(cells_.size());
        std::vector<double> initialGasWork(cells_.size());
        for (std::size_t i = 0; i < cells_.size(); ++i) {
            initialEnergy[i] = cells_[i].definition.mass * air_.cv() *
                               cells_[i].definition.absoluteTemperature;
            initialPressure[i] = cells_[i].absolutePressure;
            initialEnthalpy[i] = cellDiagnostics_[i].enthalpyFlow;
            initialGasWork[i] = cellDiagnostics_[i].gasWork;
        }
        std::vector<double> startVolume(cells_.size());
        {
            PerformanceScope geometry(profileField(
                profile, &StepPerformanceProfile::pneumaticGeometryNanoseconds));
            synchronizeGeometry(body, geometryPool);
            for (std::size_t i = 0; i < cells_.size(); ++i) {
                startVolume[i] = cells_[i].volume;
            }
        }
        {
            PerformanceScope massTransfer(profileField(
                profile,
                &StepPerformanceProfile::pneumaticMassTransferNanoseconds));
            advanceMassTransfers(timeStep);
        }
        std::vector<double> expansionStartPressure(cells_.size());
        for (std::size_t i = 0; i < cells_.size(); ++i) {
            expansionStartPressure[i] = cells_[i].absolutePressure;
        }

        std::vector<Node> forceGeometry;
        std::vector<double> startDifference(interfaces_.size());
        std::vector<Vec3> firstNodalForce(interfaces_.size());
        const auto pressure = [this](PneumaticZoneKind kind,
                                     std::size_t index) {
            return kind == PneumaticZoneKind::GasCell
                       ? cells_[index].absolutePressure
                       : reservoirs_[index].absolutePressure;
        };
        {
            PerformanceScope interfaceForce(profileField(
                profile,
                &StepPerformanceProfile::pneumaticInterfaceForceNanoseconds));
            for (std::size_t i = 0; i < body.nodes_.size(); ++i) {
                body.nodes_[i].force = persistentForces[i];
            }
            forceGeometry = body.nodes_;
            for (std::size_t i = 0; i < interfaces_.size(); ++i) {
                const PressureInterface& interfaceRecord = interfaces_[i];
                const Triangle& triangle =
                    body.triangles_[interfaceRecord.triangle];
                const Vec3& a = body.nodes_[triangle.a].position;
                const Vec3& b = body.nodes_[triangle.b].position;
                const Vec3& c = body.nodes_[triangle.c].position;
                startDifference[i] =
                    pressure(interfaceRecord.negativeKind,
                             interfaceRecord.negativeIndex) -
                    pressure(interfaceRecord.positiveKind,
                             interfaceRecord.positiveIndex);
                const Vec3 nodalForce =
                    startDifference[i] * 0.5 * cross(b - a, c - a) / 3.0;
                if (!finiteVector(nodalForce)) {
                    throw PneumaticStepError(
                        "Pneumatic interface force became non-finite");
                }
                firstNodalForce[i] = nodalForce;
                body.nodes_[triangle.a].force += nodalForce;
                body.nodes_[triangle.b].force += nodalForce;
                body.nodes_[triangle.c].force += nodalForce;
            }
        }

        StepSettings oneSubstep = settings;
        oneSubstep.timeStep = timeStep;
        oneSubstep.substeps = 1;
        {
            PerformanceScope structural(profileField(
                profile,
                &StepPerformanceProfile::pneumaticStructuralAdvanceNanoseconds));
            if (suspension) body.stepCoupled(oneSubstep, *suspension);
            else body.step(oneSubstep);
        }

        std::vector<CertifiedCellGeometry> certified;
        {
            PerformanceScope certification(profileField(
                profile,
                &StepPerformanceProfile::pneumaticCertificationNanoseconds));
            try {
                certified = certifyCurrentGeometry(body, geometryPool);
            } catch (const std::exception& error) {
                throw PneumaticStepError(
                    std::string(
                        "Coupled structural geometry certification failed: ") +
                    error.what());
            }
        }
        {
            PerformanceScope ledger(profileField(
                profile, &StepPerformanceProfile::pneumaticLedgerNanoseconds));
            for (std::size_t i = 0; i < cells_.size(); ++i) {
                GasCell& cell = cells_[i];
                cell.volume = certified[i].volume;
                cell.topology = certified[i].topology;
                cell.minimumVolume = std::min(cell.minimumVolume, cell.volume);
                cell.absolutePressure = certified[i].absolutePressure;
                const double deltaVolume = cell.volume - startVolume[i];
                PneumaticCellDiagnostics& diagnostic = cellDiagnostics_[i];
                diagnostic.gasWork +=
                    0.5 *
                    (expansionStartPressure[i] + cell.absolutePressure) *
                    deltaVolume;
                const double finalEnergy =
                    cell.definition.mass * air_.cv() *
                    cell.definition.absoluteTemperature;
                const double deltaEnergy = finalEnergy - initialEnergy[i];
                const double substepEnthalpy =
                    diagnostic.enthalpyFlow - initialEnthalpy[i];
                const double substepGasWork =
                    diagnostic.gasWork - initialGasWork[i];
                const double substepBath =
                    deltaEnergy - substepEnthalpy + substepGasWork;
                diagnostic.internalEnergyChange += deltaEnergy;
                diagnostic.bathHeat += substepBath;
                diagnostic.firstLawResidual +=
                    deltaEnergy - substepEnthalpy - substepBath +
                    substepGasWork;
                diagnostic.pressureRate =
                    (cell.absolutePressure - initialPressure[i]) / timeStep;
            }

            for (std::size_t i = 0; i < interfaces_.size(); ++i) {
                const PressureInterface& interfaceRecord = interfaces_[i];
                const Triangle& triangle =
                    body.triangles_[interfaceRecord.triangle];
                const double oldFaceVolume =
                    faceVolumeContribution(forceGeometry[triangle.a],
                                           forceGeometry[triangle.b],
                                           forceGeometry[triangle.c]);
                const double newFaceVolume =
                    faceVolumeContribution(body.nodes_[triangle.a],
                                           body.nodes_[triangle.b],
                                           body.nodes_[triangle.c]);
                const double sweptVolume = newFaceVolume - oldFaceVolume;
                const double endDifference =
                    pressure(interfaceRecord.negativeKind,
                             interfaceRecord.negativeIndex) -
                    pressure(interfaceRecord.positiveKind,
                             interfaceRecord.positiveIndex);
                const double interfaceWork =
                    0.5 * (startDifference[i] + endDifference) * sweptVolume;
                const Vec3 displacementA =
                    body.nodes_[triangle.a].position -
                    forceGeometry[triangle.a].position;
                const Vec3 displacementB =
                    body.nodes_[triangle.b].position -
                    forceGeometry[triangle.b].position;
                const Vec3 displacementC =
                    body.nodes_[triangle.c].position -
                    forceGeometry[triangle.c].position;
                const double nodalWork =
                    dot(firstNodalForce[i],
                        displacementA + displacementB + displacementC);
                PressureInterfaceDiagnostics& diagnostic =
                    interfaceDiagnostics_[i];
                diagnostic.sweptVolume = sweptVolume;
                diagnostic.interfaceWork = interfaceWork;
                diagnostic.nodalWork = nodalWork;
                diagnostics_.interfaceWork += interfaceWork;
                diagnostics_.nodalWork += nodalWork;
                diagnostics_.cumulativeAbsoluteInterfaceWork +=
                    std::abs(interfaceWork);
            }
            evaluatePressureDiagnostics(body);
            refreshNetworkDiagnostics();
        }
    } catch (const ContactStepError&) {
        *this = savedNetwork;
        for (std::size_t i = 0; i < body.nodes_.size(); ++i) {
            body.nodes_[i].force = persistentForces[i];
        }
        diagnostics_.stepSucceeded = false;
        diagnostics_.failure = "Structural contact substep rejected";
        throw;
    } catch (const std::exception& error) {
        *this = savedNetwork;
        body.nodes_ = savedNodes;
        body.constraints_ = savedConstraints;
        body.membraneElements_ = savedMembranes;
        body.dihedralConstraints_ = savedDihedrals;
        body.contactMultipliers_ = savedMultipliers;
        body.contactRecords_ = savedRecords;
        body.contactDiagnostics_ = savedContactDiagnostics;
        body.contactPairDiagnostics_ = savedPairDiagnostics;
        diagnostics_.stepSucceeded = false;
        diagnostics_.failure = error.what();
        throw;
    }
}

void PneumaticNetwork::stepCoupled(SoftBody& body,
                                   const StepSettings& settings) {
    stepCoupledImpl(body, settings, nullptr);
}

void PneumaticNetwork::stepCoupled(SoftBody& body,
                                   const StepSettings& settings,
                                   SuspensionSystem& suspension) {
    stepCoupledImpl(body, settings, &suspension);
}

void PneumaticNetwork::stepCoupledImpl(SoftBody& body,
                                       const StepSettings& settings,
                                       SuspensionSystem* suspension) {
    if (!(settings.timeStep > 0.0) || !std::isfinite(settings.timeStep) ||
        settings.substeps <= 0 || settings.constraintIterations < 0) {
        throw std::invalid_argument(
            "Invalid coupled pneumatic structural step settings");
    }
    if (!topologyValidated_) {
        validate(body);
    }
    std::vector<Vec3> persistentForces;
    persistentForces.reserve(body.nodes_.size());
    for (const Node& node : body.nodes_) {
        persistentForces.push_back(node.force);
    }

    diagnostics_.interfaceWork = 0.0;
    diagnostics_.nodalWork = 0.0;
    diagnostics_.cumulativeAbsoluteInterfaceWork = 0.0;
    diagnostics_.stepSucceeded = true;
    diagnostics_.failure.clear();
    for (PneumaticCellDiagnostics& diagnostic : cellDiagnostics_) {
        diagnostic.netMassRate = 0.0;
        diagnostic.pressureRate = 0.0;
        diagnostic.enthalpyFlow = 0.0;
        diagnostic.internalEnergyChange = 0.0;
        diagnostic.gasWork = 0.0;
        diagnostic.bathHeat = 0.0;
        diagnostic.firstLawResidual = 0.0;
    }

    const double substepTime = settings.timeStep / settings.substeps;
    for (int substep = 0; substep < settings.substeps; ++substep) {
        coupledSubstep(body,
                       settings,
                       substepTime,
                       std::span<const Vec3>{persistentForces},
                       suspension);
    }
    body.clearExternalForces();
    diagnostics_.stepSucceeded = true;
    refreshNetworkDiagnostics();
}

const PneumaticCellDiagnostics& PneumaticNetwork::cellDiagnostics(
    const PneumaticZoneHandle& cell) const {
    requireCell(cell);
    return cellDiagnostics_[cell.index_];
}

const ReservoirDefinition& PneumaticNetwork::reservoir(
    const PneumaticZoneHandle& reservoirHandle) const {
    requireZone(reservoirHandle);
    if (reservoirHandle.kind_ != PneumaticZoneKind::Reservoir) {
        throw std::invalid_argument("Pneumatic zone is not a reservoir");
    }
    return reservoirs_[reservoirHandle.index_];
}

const ReservoirDefinition& PneumaticNetwork::reservoir(
    std::size_t reservoirIndex) const {
    if (reservoirIndex >= reservoirs_.size()) {
        throw std::out_of_range("Pneumatic reservoir index is out of range");
    }
    return reservoirs_[reservoirIndex];
}

const PressureInterface& PneumaticNetwork::pressureInterface(
    const PressureInterfaceHandle& interfaceHandle) const {
    if (interfaceHandle.owner_ != this) {
        throw std::invalid_argument(
            "Pressure interface belongs to another network");
    }
    if (interfaceHandle.index_ >= interfaces_.size()) {
        throw std::out_of_range("Pressure interface handle is out of range");
    }
    return interfaces_[interfaceHandle.index_];
}

const PneumaticPortDiagnostics& PneumaticNetwork::portDiagnostics(
    const PneumaticPortHandle& port) const {
    requirePort(port);
    return portDiagnostics_[port.index_];
}

// ---------------------------------------------------------------------------
// SOFTWING_FLIGHT_STATE 1 access. These functions live here because the
// mutable cell/port containers are private nested types of PneumaticNetwork.
// They expose exactly the committed cross-step state identified in the
// flight-state completeness contract and never touch identity, handles,
// topology, or per-step diagnostics.

PneumaticFlightCounts FlightStateAccess::pneumaticCounts(
    const PneumaticNetwork& network) {
    return {network.cells_.size(), network.reservoirs_.size(),
            network.interfaces_.size(), network.ports_.size()};
}

std::vector<PneumaticFlightCell> FlightStateAccess::capturePneumaticCells(
    const PneumaticNetwork& network) {
    std::vector<PneumaticFlightCell> cells;
    cells.reserve(network.cells_.size());
    for (const auto& cell : network.cells_)
        cells.push_back({cell.definition.mass,
                         cell.definition.absoluteTemperature, cell.volume,
                         cell.absolutePressure, cell.minimumVolume});
    return cells;
}

std::vector<PneumaticFlightPortOpening>
FlightStateAccess::capturePneumaticPortOpenings(
    const PneumaticNetwork& network) {
    std::vector<PneumaticFlightPortOpening> openings;
    openings.reserve(network.ports_.size());
    for (const auto& port : network.ports_)
        openings.push_back({port.orifice.openingFraction,
                            port.conductance.openingFraction});
    return openings;
}

PneumaticFlightLedger FlightStateAccess::capturePneumaticLedger(
    const PneumaticNetwork& network) {
    return {network.initialFiniteMass_, network.massReferenceInitialized_,
            network.topologyValidated_,
            network.diagnostics_.cumulativeReservoirInflow};
}

std::string FlightStateAccess::pneumaticStructureText(
    const PneumaticNetwork& network) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "PNEUMATIC_STRUCTURE 1\n";
    out << "AIR " << network.air_.specificGasConstant << ' '
        << network.air_.heatCapacityRatio << '\n';
    out << "CELLS " << network.cells_.size() << '\n';
    for (const auto& cell : network.cells_)
        out << cell.definition.massFloor << ' '
            << cell.definition.volumeFloor << '\n';
    out << "RESERVOIRS " << network.reservoirs_.size() << '\n';
    for (const auto& reservoir : network.reservoirs_)
        out << reservoir.absolutePressure << ' '
            << reservoir.absoluteTemperature << ' '
            << static_cast<int>(reservoir.pressureProvenance) << ' '
            << static_cast<int>(reservoir.temperatureProvenance) << '\n';
    out << "INTERFACES " << network.interfaces_.size() << '\n';
    for (const auto& interfaceRecord : network.interfaces_)
        out << interfaceRecord.triangle << ' '
            << static_cast<int>(interfaceRecord.negativeKind) << ' '
            << interfaceRecord.negativeIndex << ' '
            << static_cast<int>(interfaceRecord.positiveKind) << ' '
            << interfaceRecord.positiveIndex << '\n';
    out << "PORTS " << network.ports_.size() << '\n';
    for (const auto& port : network.ports_)
        out << static_cast<int>(port.kind) << ' '
            << static_cast<int>(port.firstKind) << ' ' << port.firstIndex
            << ' ' << static_cast<int>(port.secondKind) << ' '
            << port.secondIndex << ' ' << port.orifice.dischargeCoefficient
            << ' ' << port.orifice.effectiveArea << ' '
            << port.conductance.massConductance << '\n';
    return out.str();
}

const SoftBody* FlightStateAccess::pneumaticBodyOwner(
    const PneumaticNetwork& network) {
    return network.bodyOwner_;
}

void FlightStateAccess::applyPneumaticState(
    PneumaticNetwork& network,
    const std::vector<PneumaticFlightCell>& cells,
    const std::vector<PneumaticFlightPortOpening>& openings,
    const PneumaticFlightLedger& ledger) noexcept {
    for (std::size_t index = 0;
         index < network.cells_.size() && index < cells.size(); ++index) {
        auto& cell = network.cells_[index];
        cell.definition.mass = cells[index].mass;
        cell.definition.absoluteTemperature =
            cells[index].absoluteTemperature;
        cell.volume = cells[index].volume;
        cell.absolutePressure = cells[index].absolutePressure;
        cell.minimumVolume = cells[index].minimumVolume;
    }
    for (std::size_t index = 0;
         index < network.ports_.size() && index < openings.size(); ++index) {
        auto& port = network.ports_[index];
        port.orifice.openingFraction = openings[index].orificeOpening;
        port.conductance.openingFraction =
            openings[index].conductanceOpening;
    }
    network.initialFiniteMass_ = ledger.initialFiniteMass;
    network.massReferenceInitialized_ = ledger.massReferenceInitialized;
    network.topologyValidated_ = ledger.topologyValidated;
    network.diagnostics_.cumulativeReservoirInflow =
        ledger.cumulativeReservoirInflow;
}

#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
std::string AerodynamicVerificationTestAccess::pneumaticState(
    const PneumaticNetwork& network) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    const auto vec = [&](const Vec3& value) {
        out << value.x << ' ' << value.y << ' ' << value.z << ' ';
    };
    const auto topology = [&](const PneumaticTopologyStatus& value) {
        out << value.valid << ' ' << value.incidentInterfaces << ' '
            << value.surface.outOfRangeNodeReferences << ' '
            << value.surface.degenerateFaces << ' '
            << value.surface.boundaryEdges << ' '
            << value.surface.nonManifoldEdges << ' '
            << value.surface.inconsistentDirectedEdges << ' ';
    };
    out << "AIR " << network.air_.specificGasConstant << ' '
        << network.air_.heatCapacityRatio << " OWNER "
        << (network.bodyOwner_ != nullptr) << " IDENTITY "
        << static_cast<bool>(network.lifetimeToken_) << '\n';
    out << "CELLS " << network.cells_.size() << '\n';
    for (const auto& cell : network.cells_) {
        out << cell.definition.mass << ' '
            << cell.definition.absoluteTemperature << ' '
            << cell.definition.massFloor << ' '
            << cell.definition.volumeFloor << ' ' << cell.volume << ' '
            << cell.absolutePressure << ' ' << cell.minimumVolume << ' ';
        topology(cell.topology); out << '\n';
    }
    out << "RESERVOIRS " << network.reservoirs_.size() << '\n';
    for (const auto& reservoir : network.reservoirs_)
        out << reservoir.absolutePressure << ' '
            << reservoir.absoluteTemperature << ' '
            << static_cast<int>(reservoir.pressureProvenance) << ' '
            << static_cast<int>(reservoir.temperatureProvenance) << '\n';
    out << "INTERFACES " << network.interfaces_.size() << '\n';
    for (const auto& interface : network.interfaces_)
        out << interface.triangle << ' '
            << static_cast<int>(interface.negativeKind) << ' '
            << interface.negativeIndex << ' '
            << static_cast<int>(interface.positiveKind) << ' '
            << interface.positiveIndex << '\n';
    out << "PORTS " << network.ports_.size() << '\n';
    for (const auto& port : network.ports_)
        out << static_cast<int>(port.kind) << ' '
            << static_cast<int>(port.firstKind) << ' ' << port.firstIndex
            << ' ' << static_cast<int>(port.secondKind) << ' '
            << port.secondIndex << ' ' << port.orifice.dischargeCoefficient
            << ' ' << port.orifice.effectiveArea << ' '
            << port.orifice.openingFraction << ' '
            << port.conductance.massConductance << ' '
            << port.conductance.openingFraction << '\n';
    out << "CELL_DIAGNOSTICS " << network.cellDiagnostics_.size() << '\n';
    for (const auto& value : network.cellDiagnostics_) {
        out << value.mass << ' ' << value.volume << ' '
            << value.absolutePressure << ' ' << value.gaugePressure << ' '
            << value.absoluteTemperature << ' ' << value.netMassRate << ' '
            << value.pressureRate << ' ' << value.internalEnergy << ' '
            << value.internalEnergyChange << ' ' << value.enthalpyFlow << ' '
            << value.gasWork << ' ' << value.bathHeat << ' '
            << value.firstLawResidual << ' ' << value.minimumVolume << ' ';
        topology(value.topology);
        out << static_cast<int>(value.massProvenance) << ' '
            << static_cast<int>(value.volumeProvenance) << ' '
            << static_cast<int>(value.pressureProvenance) << ' '
            << static_cast<int>(value.temperatureProvenance) << '\n';
    }
    out << "PORT_DIAGNOSTICS " << network.portDiagnostics_.size() << '\n';
    for (const auto& value : network.portDiagnostics_)
        out << static_cast<int>(value.kind) << ' '
            << static_cast<int>(value.direction) << ' ' << value.rawMassRate
            << ' ' << value.appliedMassRate << ' ' << value.pressureRatio
            << ' ' << value.choked << ' ' << value.openingFraction << ' '
            << value.limiterScale << ' '
            << static_cast<int>(value.limiterCause) << ' '
            << static_cast<int>(value.openingProvenance) << ' '
            << static_cast<int>(value.flowProvenance) << '\n';
    out << "INTERFACE_DIAGNOSTICS "
        << network.interfaceDiagnostics_.size() << '\n';
    for (const auto& value : network.interfaceDiagnostics_) {
        out << value.pressureDifference << ' '; vec(value.force);
        vec(value.moment);
        out << value.sweptVolume << ' ' << value.interfaceWork << ' '
            << value.nodalWork << '\n';
    }
    const auto& value = network.diagnostics_;
    out << "DIAGNOSTICS " << value.topologyValid << ' '
        << value.stepSucceeded << ' ' << std::quoted(value.failure) << ' '
        << value.finiteCellMass << ' ' << value.cumulativeReservoirInflow
        << ' ' << value.massResidual << ' ';
    vec(value.pressureForce); vec(value.pressureMoment);
    out << value.internalEnergyChange << ' ' << value.advectedEnthalpy << ' '
        << value.gasWork << ' ' << value.bathHeat << ' '
        << value.firstLawResidual << ' ' << value.interfaceWork << ' '
        << value.nodalWork << ' ' << value.cumulativeAbsoluteInterfaceWork
        << ' ' << value.couplingWorkMismatch << ' '
        << value.relativeCouplingWorkMismatch << ' '
        << value.maximumFirstLawResidual << ' ' << value.minimumLimiterScale
        << ' ' << value.limiterActive << ' '
        << static_cast<int>(value.residualProvenance) << '\n';
    out << "REFERENCES " << network.initialFiniteMass_ << ' '
        << network.massReferenceInitialized_ << ' '
        << network.topologyValidated_ << '\n';
    return out.str();
}
#endif

} // namespace softwing
