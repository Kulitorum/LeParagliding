#pragma once

#include "softwing/aerodynamics.h"
#include "softwing/pneumatics.h"

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace softwing {

// Plain pneumatic state mirrors used by the SOFTWING_FLIGHT_STATE 1 module.
// PneumaticNetwork's mutable cell and port containers are private nested
// types defined in src/pneumatics.cpp, so the pneumatic capture/apply
// functions are implemented there while all formatting, parsing, and
// validation stays in src/flight_state.cpp.
struct PneumaticFlightCell {
    double mass = 0.0;
    double absoluteTemperature = 0.0;
    double volume = 0.0;
    double absolutePressure = 0.0;
    double minimumVolume = 0.0;
};

struct PneumaticFlightPortOpening {
    double orificeOpening = 0.0;
    double conductanceOpening = 0.0;
};

struct PneumaticFlightLedger {
    double initialFiniteMass = 0.0;
    bool massReferenceInitialized = false;
    bool topologyValidated = false;
    double cumulativeReservoirInflow = 0.0;
};

struct PneumaticFlightCounts {
    std::size_t cells = 0;
    std::size_t reservoirs = 0;
    std::size_t interfaces = 0;
    std::size_t ports = 0;
};

class TrimConfiguration;
struct FlightStateRestoreResult;

// Production friend of the accepted owners. It restores committed state into
// existing live owners for the Stage 7 restart artifact; it never changes
// registration, identity tokens, definitions, or topology.
struct FlightStateAccess {
    // Implemented in src/flight_state.cpp. The public writeFlightState /
    // restoreFlightState functions forward here so the entire module runs
    // with the friend access it needs while every owner keeps its accepted
    // public surface unchanged.
    static std::string write(const TrimConfiguration& configuration,
                             double time,
                             std::size_t step,
                             const SoftBody& body,
                             const SuspensionSystem* suspension,
                             const PneumaticNetwork* pneumatics,
                             const AerodynamicSystem* aerodynamics);
    static FlightStateRestoreResult restore(
        std::string_view text,
        const TrimConfiguration& configuration,
        SoftBody& body,
        SuspensionSystem* suspension,
        PneumaticNetwork* pneumatics,
        AerodynamicSystem* aerodynamics);

    // Canonical classic-locale immutable-structure text for the suspension
    // fingerprint (definition, resolved attachments, junction node indices,
    // and base hang points). Implemented in src/flight_state.cpp.
    static std::string suspensionStructureText(
        const SuspensionSystem& suspension);

    // Implemented in src/pneumatics.cpp (private nested types).
    static PneumaticFlightCounts pneumaticCounts(
        const PneumaticNetwork& network);
    static std::vector<PneumaticFlightCell> capturePneumaticCells(
        const PneumaticNetwork& network);
    static std::vector<PneumaticFlightPortOpening>
    capturePneumaticPortOpenings(const PneumaticNetwork& network);
    static PneumaticFlightLedger capturePneumaticLedger(
        const PneumaticNetwork& network);
    // Canonical classic-locale immutable-structure text for the pneumatic
    // fingerprint: air properties, cell floors, reservoirs, interfaces, and
    // static port data. Mutable opening fractions are excluded.
    static std::string pneumaticStructureText(const PneumaticNetwork& network);
    static const SoftBody* pneumaticBodyOwner(const PneumaticNetwork& network);
    // Non-throwing element-wise commit; sizes must already be validated.
    static void applyPneumaticState(
        PneumaticNetwork& network,
        const std::vector<PneumaticFlightCell>& cells,
        const std::vector<PneumaticFlightPortOpening>& openings,
        const PneumaticFlightLedger& ledger) noexcept;
};

} // namespace softwing
