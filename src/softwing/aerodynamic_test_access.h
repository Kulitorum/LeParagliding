#pragma once

#include "softwing/vec3.h"

#include <array>
#include <cstddef>
#include <span>
#include <string>

namespace softwing {

class AerodynamicSystem;
class PneumaticNetwork;
class SoftBody;
class SuspensionSystem;

struct AerodynamicVerificationTestAccess {
    static bool trianglesIntersectBeyondSharedTopology(
        std::array<Vec3, 3> first,
        std::array<Vec3, 3> second,
        std::span<const Vec3> sharedPoints);
    static std::string softBodyState(const SoftBody& body);
    static std::string suspensionState(const SuspensionSystem& suspension);
    static std::string pneumaticState(const PneumaticNetwork& pneumatics);
    static std::string aerodynamicState(const AerodynamicSystem& aerodynamics);
    static std::size_t solveTrialInvocationCount(
        const AerodynamicSystem& aerodynamics);
    static void permuteSoftBodyStorage(
        SoftBody& body,
        std::span<const std::size_t> nodeMap,
        std::span<const std::size_t> triangleMap);
    static void injectNonFinitePayloadAfterAdvance(
        AerodynamicSystem& aerodynamics,
        bool enabled);
    static void injectImpulseLedgerMismatch(
        AerodynamicSystem& aerodynamics,
        bool enabled);
    static void injectMechanicalEnergyLedgerMismatch(
        AerodynamicSystem& aerodynamics,
        bool enabled);
};

} // namespace softwing
