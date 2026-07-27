#pragma once

#include "softwing/soft_body.h"

#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace softwing {

inline constexpr double dryAirSpecificGasConstant = 287.05;
inline constexpr double dryAirHeatCapacityRatio = 1.4;
inline constexpr double standardFixtureTemperature = 293.15;

enum class QuantityProvenance {
    Solved,
    Prescribed,
    Diagnostic,
};

enum class PneumaticZoneKind {
    GasCell,
    Reservoir,
};

enum class PneumaticPortKind {
    Orifice,
    LinearConductance,
};

enum class FlowDirection {
    Equilibrium,
    FirstToSecond,
    SecondToFirst,
};

enum class FlowLimiterCause {
    None,
    DonorMassFloor,
    PressureEquilibrium,
};

struct AirProperties {
    double specificGasConstant = dryAirSpecificGasConstant;
    double heatCapacityRatio = dryAirHeatCapacityRatio;

    void validate() const;
    [[nodiscard]] double cv() const;
    [[nodiscard]] double cp() const;
};

struct FlowZoneState {
    double absolutePressure = 0.0;
    double absoluteTemperature = 0.0;
};

struct OrificeDefinition {
    double dischargeCoefficient = 1.0;
    double effectiveArea = 0.0;
    double openingFraction = 1.0;
};

struct ConductanceDefinition {
    double massConductance = 0.0;
    double openingFraction = 1.0;
};

struct MassFlowEvaluation {
    double signedMassRate = 0.0; // kg/s, positive first -> second
    double magnitude = 0.0;
    double pressureRatio = 1.0;
    double criticalPressureRatio = 1.0;
    FlowDirection direction = FlowDirection::Equilibrium;
    bool choked = false;
};

[[nodiscard]] MassFlowEvaluation evaluateOrificeMassFlow(
    const FlowZoneState& first,
    const FlowZoneState& second,
    const AirProperties& air,
    const OrificeDefinition& definition);

[[nodiscard]] MassFlowEvaluation evaluateConductanceMassFlow(
    const FlowZoneState& first,
    const FlowZoneState& second,
    const ConductanceDefinition& definition);

class PneumaticNetwork;

class PneumaticZoneHandle {
public:
    [[nodiscard]] PneumaticZoneKind kind() const { return kind_; }
    [[nodiscard]] std::size_t index() const { return index_; }

private:
    friend class PneumaticNetwork;
    PneumaticZoneHandle(const PneumaticNetwork* owner,
                        PneumaticZoneKind kind,
                        std::size_t index)
        : owner_(owner), kind_(kind), index_(index) {}

    const PneumaticNetwork* owner_ = nullptr;
    PneumaticZoneKind kind_ = PneumaticZoneKind::GasCell;
    std::size_t index_ = 0;
};

class PressureInterfaceHandle {
public:
    [[nodiscard]] std::size_t index() const { return index_; }

private:
    friend class PneumaticNetwork;
    PressureInterfaceHandle(const PneumaticNetwork* owner, std::size_t index)
        : owner_(owner), index_(index) {}
    const PneumaticNetwork* owner_ = nullptr;
    std::size_t index_ = 0;
};

class PneumaticPortHandle {
public:
    [[nodiscard]] std::size_t index() const { return index_; }

private:
    friend class PneumaticNetwork;
    PneumaticPortHandle(const PneumaticNetwork* owner, std::size_t index)
        : owner_(owner), index_(index) {}
    const PneumaticNetwork* owner_ = nullptr;
    std::size_t index_ = 0;
};

struct GasCellDefinition {
    double mass = 0.0;
    double absoluteTemperature = standardFixtureTemperature;
    double massFloor = 1.0e-12;
    double volumeFloor = 1.0e-12;
};

struct ReservoirDefinition {
    double absolutePressure = 101325.0;
    double absoluteTemperature = standardFixtureTemperature;
    QuantityProvenance pressureProvenance = QuantityProvenance::Prescribed;
    QuantityProvenance temperatureProvenance = QuantityProvenance::Prescribed;
};

struct PressureInterface {
    std::size_t triangle = 0;
    PneumaticZoneKind negativeKind = PneumaticZoneKind::GasCell;
    std::size_t negativeIndex = 0;
    PneumaticZoneKind positiveKind = PneumaticZoneKind::Reservoir;
    std::size_t positiveIndex = 0;
};

struct PneumaticTopologyStatus {
    bool valid = false;
    std::size_t incidentInterfaces = 0;
    SurfaceTopologyReport surface;
};

struct PneumaticCellDiagnostics {
    double mass = 0.0;
    double volume = 0.0;
    double absolutePressure = 0.0;
    double gaugePressure = 0.0;
    double absoluteTemperature = 0.0;
    double netMassRate = 0.0;
    double pressureRate = 0.0;
    double internalEnergy = 0.0;
    double internalEnergyChange = 0.0;
    double enthalpyFlow = 0.0;
    double gasWork = 0.0;
    double bathHeat = 0.0;
    double firstLawResidual = 0.0;
    double minimumVolume = 0.0;
    PneumaticTopologyStatus topology;
    QuantityProvenance massProvenance = QuantityProvenance::Solved;
    QuantityProvenance volumeProvenance = QuantityProvenance::Solved;
    QuantityProvenance pressureProvenance = QuantityProvenance::Solved;
    QuantityProvenance temperatureProvenance = QuantityProvenance::Prescribed;
};

struct PneumaticPortDiagnostics {
    PneumaticPortKind kind = PneumaticPortKind::Orifice;
    FlowDirection direction = FlowDirection::Equilibrium;
    double rawMassRate = 0.0;
    double appliedMassRate = 0.0;
    double pressureRatio = 1.0;
    bool choked = false;
    double openingFraction = 0.0;
    double limiterScale = 1.0;
    FlowLimiterCause limiterCause = FlowLimiterCause::None;
    QuantityProvenance openingProvenance = QuantityProvenance::Prescribed;
    QuantityProvenance flowProvenance = QuantityProvenance::Diagnostic;
};

struct PressureInterfaceDiagnostics {
    double pressureDifference = 0.0;
    Vec3 force;
    Vec3 moment;
    double sweptVolume = 0.0;
    double interfaceWork = 0.0;
    double nodalWork = 0.0;
};

struct PneumaticNetworkDiagnostics {
    bool topologyValid = false;
    bool stepSucceeded = true;
    std::string failure;
    double finiteCellMass = 0.0;
    double cumulativeReservoirInflow = 0.0;
    double massResidual = 0.0;
    Vec3 pressureForce;
    Vec3 pressureMoment;
    double internalEnergyChange = 0.0;
    double advectedEnthalpy = 0.0;
    double gasWork = 0.0;
    double bathHeat = 0.0;
    double firstLawResidual = 0.0;
    double interfaceWork = 0.0;
    double nodalWork = 0.0;
    double cumulativeAbsoluteInterfaceWork = 0.0;
    double couplingWorkMismatch = 0.0;
    double relativeCouplingWorkMismatch = 0.0;
    double maximumFirstLawResidual = 0.0;
    double minimumLimiterScale = 1.0;
    bool limiterActive = false;
    QuantityProvenance residualProvenance = QuantityProvenance::Diagnostic;
};

class PneumaticStepError : public std::runtime_error {
public:
    explicit PneumaticStepError(const std::string& message)
        : std::runtime_error(message) {}
};

class PneumaticNetwork {
public:
    explicit PneumaticNetwork(AirProperties air = {});
    ~PneumaticNetwork();

    [[nodiscard]] PneumaticZoneHandle addGasCell(
        const GasCellDefinition& definition);
    [[nodiscard]] PneumaticZoneHandle addReservoir(
        const ReservoirDefinition& definition);
    [[nodiscard]] PressureInterfaceHandle addPressureInterface(
        const SoftBody& body,
        std::size_t triangle,
        const PneumaticZoneHandle& negativeSide,
        const PneumaticZoneHandle& positiveSide);
    [[nodiscard]] PneumaticPortHandle addOrificePort(
        const PneumaticZoneHandle& first,
        const PneumaticZoneHandle& second,
        const OrificeDefinition& definition);
    [[nodiscard]] PneumaticPortHandle addConductancePort(
        const PneumaticZoneHandle& first,
        const PneumaticZoneHandle& second,
        const ConductanceDefinition& definition);

    void setPortOpening(const PneumaticPortHandle& port,
                        double openingFraction);
    void setPortOpening(std::size_t portIndex, double openingFraction);
    void setReservoirState(std::size_t reservoirIndex,
                           const ReservoirDefinition& definition);
    void initializeCellMassFromPressure(const SoftBody& body,
                                        const PneumaticZoneHandle& cell,
                                        double absolutePressure);
    void initializeCellMassFromPressure(const SoftBody& body,
                                        std::size_t cellIndex,
                                        double absolutePressure);
    void validate(const SoftBody& body);
    void advanceRigid(const SoftBody& body, double timeStep);
    void stepCoupled(SoftBody& body, const StepSettings& settings);
    void stepCoupled(SoftBody& body,
                     const StepSettings& settings,
                     SuspensionSystem& suspension);

    [[nodiscard]] const AirProperties& airProperties() const { return air_; }
    [[nodiscard]] const PneumaticCellDiagnostics& cellDiagnostics(
        const PneumaticZoneHandle& cell) const;
    [[nodiscard]] const ReservoirDefinition& reservoir(
        const PneumaticZoneHandle& reservoir) const;
    [[nodiscard]] const ReservoirDefinition& reservoir(
        std::size_t reservoirIndex) const;
    [[nodiscard]] const PressureInterface& pressureInterface(
        const PressureInterfaceHandle& interfaceHandle) const;
    [[nodiscard]] const PneumaticPortDiagnostics& portDiagnostics(
        const PneumaticPortHandle& port) const;
    [[nodiscard]] const std::vector<PneumaticCellDiagnostics>&
    cellDiagnostics() const { return cellDiagnostics_; }
    [[nodiscard]] const std::vector<PneumaticPortDiagnostics>&
    portDiagnostics() const { return portDiagnostics_; }
    [[nodiscard]] const std::vector<PressureInterfaceDiagnostics>&
    interfaceDiagnostics() const { return interfaceDiagnostics_; }
    [[nodiscard]] const PneumaticNetworkDiagnostics& diagnostics() const {
        return diagnostics_;
    }

private:
    friend class AerodynamicSystem;
    // Accepted Stage 7 flight-state restart module (production friend, not
    // test access). PneumaticNetwork is noncopyable/nonmovable with identity
    // semantics, so the restart artifact restores committed state into this
    // live owner instead of constructing a replacement.
    friend struct FlightStateAccess;
#ifdef SOFTWING_AERODYNAMIC_TEST_ACCESS
    friend struct AerodynamicVerificationTestAccess;
#endif
    PneumaticNetwork(const PneumaticNetwork& other);
    PneumaticNetwork& operator=(const PneumaticNetwork& other);

    struct GasCell;
    struct Port;
    struct CertifiedCellGeometry {
        double volume = 0.0;
        double absolutePressure = 0.0;
        PneumaticTopologyStatus topology;
    };

    void requireZone(const PneumaticZoneHandle& zone) const;
    void requireCell(const PneumaticZoneHandle& cell) const;
    void requirePort(const PneumaticPortHandle& port) const;
    [[nodiscard]] double zonePressure(const PneumaticZoneHandle& zone) const;
    [[nodiscard]] double zoneTemperature(const PneumaticZoneHandle& zone) const;
    [[nodiscard]] std::vector<CertifiedCellGeometry>
    certifyCurrentGeometry(const SoftBody& body,
                           WorkerPool* workerPool = nullptr) const;
    void synchronizeGeometry(const SoftBody& body,
                             WorkerPool* workerPool = nullptr);
    void evaluatePressureDiagnostics(const SoftBody& body,
                                     const Vec3& origin = {});
    void advanceMassTransfers(double timeStep);
    void refreshNetworkDiagnostics();
    void coupledSubstep(SoftBody& body,
                        const StepSettings& settings,
                        double timeStep,
                        std::span<const Vec3> persistentForces,
                        SuspensionSystem* suspension = nullptr);
    void stepCoupledImpl(SoftBody& body,
                         const StepSettings& settings,
                         SuspensionSystem* suspension);

    AirProperties air_;
    const SoftBody* bodyOwner_ = nullptr;
    std::vector<GasCell> cells_;
    std::vector<ReservoirDefinition> reservoirs_;
    std::vector<PressureInterface> interfaces_;
    std::vector<Port> ports_;
    std::vector<PneumaticCellDiagnostics> cellDiagnostics_;
    std::vector<PneumaticPortDiagnostics> portDiagnostics_;
    std::vector<PressureInterfaceDiagnostics> interfaceDiagnostics_;
    PneumaticNetworkDiagnostics diagnostics_;
    double initialFiniteMass_ = 0.0;
    bool massReferenceInitialized_ = false;
    bool topologyValidated_ = false;
    // Directed edge incidence depends only on the registered triangle
    // indices. A normal live step reuses this certified report after checking
    // every current face for the unchanged degeneracy predicate; any suspect
    // face falls back to the complete validator and its exact diagnostics.
    std::vector<SurfaceTopologyReport> certifiedCellTopologies_;
    // Reused oriented-face storage avoids two allocations per cell per live
    // substep. It is scratch only and is intentionally not copied/serialized.
    mutable std::vector<std::vector<Triangle>> geometryTriangleScratch_;
    // Geometry certification is read-only over the body and writes one result
    // slot per gas cell. The pool is scratch, not pneumatic state: copy and
    // rollback retain the destination's workers exactly like SoftBody's pool.
    mutable WorkerPoolSlot geometryWorkerPool_;
    // Private copies are transaction snapshots with a fresh identity; copy
    // assignment intentionally retains the destination identity. The network
    // remains non-movable because its public handles bind its address.
    std::shared_ptr<void> lifetimeToken_ =
        std::make_shared<unsigned char>();
};

} // namespace softwing
