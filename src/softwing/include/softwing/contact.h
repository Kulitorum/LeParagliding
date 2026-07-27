#pragma once

#include "softwing/vec3.h"

#include <array>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace softwing {

class SoftBody;

enum class ContactColliderKind {
    Surface,
    Line,
};

enum class ContactPairKind {
    SurfaceSurface,
    SurfaceLine,
};

enum class ContactFeatureKind {
    VertexTriangle,
    EdgeEdge,
    SegmentTriangle,
};

enum class GeometryQueryStatus {
    Certified,
    Degenerate,
    NonFinite,
    Uncertifiable,
};

enum class ClosestFeatureClass {
    Point,
    SegmentStart,
    SegmentEnd,
    SegmentInterior,
    TriangleVertex0,
    TriangleVertex1,
    TriangleVertex2,
    TriangleEdge01,
    TriangleEdge12,
    TriangleEdge20,
    TriangleFace,
};

struct ClosestFeatureResult {
    GeometryQueryStatus status = GeometryQueryStatus::Uncertifiable;
    ClosestFeatureClass firstFeature = ClosestFeatureClass::Point;
    ClosestFeatureClass secondFeature = ClosestFeatureClass::Point;
    Vec3 firstPoint;
    Vec3 secondPoint;
    std::array<double, 3> firstWeights{};
    std::array<double, 3> secondWeights{};
    Vec3 normal;
    double distance = 0.0;
    double gap = 0.0;

    [[nodiscard]] bool certified() const {
        return status == GeometryQueryStatus::Certified;
    }
};

[[nodiscard]] ClosestFeatureResult closestVertexTriangle(
    const Vec3& vertex,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    double separation = 0.0);

[[nodiscard]] ClosestFeatureResult closestEdgeEdge(
    const Vec3& firstA,
    const Vec3& firstB,
    const Vec3& secondA,
    const Vec3& secondB,
    double separation = 0.0);

[[nodiscard]] ClosestFeatureResult closestSegmentTriangle(
    const Vec3& segmentA,
    const Vec3& segmentB,
    const Vec3& triangleA,
    const Vec3& triangleB,
    const Vec3& triangleC,
    double separation = 0.0);

struct ContactFeatureKey {
    std::size_t pair = 0;
    ContactFeatureKind kind = ContactFeatureKind::VertexTriangle;
    std::array<std::size_t, 3> firstPrimitive{};
    std::array<std::size_t, 3> secondPrimitive{};

    auto operator<=>(const ContactFeatureKey&) const = default;
};

struct ContactTopologyReport {
    std::size_t possibleCount = 0;
    std::size_t excludedCount = 0;
    std::vector<ContactFeatureKey> eligibleKeys;
};

struct SweptAabb {
    Vec3 minimum;
    Vec3 maximum;
};

enum class BroadphaseAxis {
    X,
    Y,
    Z,
};

struct BroadphaseProxy {
    std::size_t id = 0;
    SweptAabb bounds;
};

struct BroadphaseFeaturePair {
    ContactFeatureKey key;
    std::size_t firstProxy = 0;
    std::size_t secondProxy = 0;
};

struct BroadphaseResult {
    BroadphaseAxis axis = BroadphaseAxis::X;
    std::size_t possibleCount = 0;
    std::vector<ContactFeatureKey> candidateKeys;
};

struct LinearContactPrimitive {
    std::array<Vec3, 3> previous{};
    std::array<Vec3, 3> current{};
    std::size_t count = 0;
};

enum class CcdState {
    NoImpact,
    Impact,
    InitiallyProximate,
    Indeterminate,
};

struct CcdSettings {
    int conservativeIterations = 32;
    int intervalSubdivisions = 256;
    double timeTolerance = 1.0e-8;
    double distanceTolerance = 1.0e-10;
};

struct CcdResult {
    CcdState state = CcdState::Indeterminate;
    double timeOfImpact = 0.0;
    double bracketLower = 0.0;
    double bracketUpper = 1.0;
    ClosestFeatureResult geometry;
    int conservativeIterations = 0;
    int intervalSubdivisions = 0;
    bool usedIntervalFallback = false;
};

enum class ContactFrictionState {
    Inactive,
    Sticking,
    Sliding,
};

struct CoulombFrictionResult {
    ContactFrictionState state = ContactFrictionState::Inactive;
    Vec3 initialTangentialVelocity;
    Vec3 finalTangentialVelocity;
    Vec3 tangentialImpulse;
    double coneRatio = 0.0;
    double work = 0.0;
};

[[nodiscard]] CoulombFrictionResult solveCoulombFriction(
    const Vec3& relativeVelocity,
    const Vec3& normal,
    double tangentialEffectiveInverseMass,
    double normalImpulse,
    double staticFriction,
    double dynamicFriction,
    bool normalContactActive);

struct ContactRecord {
    ContactFeatureKey key;
    ContactFeatureKind kind = ContactFeatureKind::VertexTriangle;
    CcdState ccdState = CcdState::NoImpact;
    double timeOfImpact = 0.0;
    double bracketLower = 0.0;
    double bracketUpper = 0.0;
    int ccdIterations = 0;
    int intervalSubdivisions = 0;
    bool usedIntervalFallback = false;
    std::array<std::size_t, 3> firstNodes{};
    std::array<std::size_t, 3> secondNodes{};
    std::size_t firstNodeCount = 0;
    std::size_t secondNodeCount = 0;
    std::array<double, 3> firstWeights{};
    std::array<double, 3> secondWeights{};
    Vec3 firstPoint;
    Vec3 secondPoint;
    Vec3 normal;
    double pairSeparation = 0.0;
    double gap = 0.0;
    double penetration = 0.0;
    double normalMultiplier = 0.0;
    double normalForceEstimate = 0.0;
    double normalImpulseMagnitude = 0.0;
    Vec3 tangentialImpulse;
    ContactFrictionState frictionState = ContactFrictionState::Inactive;
    double frictionConeRatio = 0.0;
    double normalResidual = 0.0;
    double frictionResidual = 0.0;
    double frictionWork = 0.0;
    double tangentSpeedBefore = 0.0;
    double tangentSpeedAfter = 0.0;
    Vec3 firstImpulse;
    Vec3 secondImpulse;
    Vec3 firstMoment;
    Vec3 secondMoment;
    // Per-node ledgers retain the impulses actually distributed by the
    // positional and friction solves. Aggregate impulses and moments are
    // derived from these values at the accepted final node positions.
    std::array<Vec3, 3> firstNodeImpulses{};
    std::array<Vec3, 3> secondNodeImpulses{};
    int solverVisits = 0;
};

struct ContactDiagnostics {
    bool registered = false;
    bool solveSucceeded = true;
    std::size_t possibleCount = 0;
    std::size_t candidateCount = 0;
    std::size_t excludedCount = 0;
    std::size_t queryCount = 0;
    std::size_t activeCount = 0;
    std::size_t indeterminateCount = 0;
    double minimumGap = std::numeric_limits<double>::infinity();
    double maximumPenetration = 0.0;
    double maximumNormalResidual = 0.0;
    double maximumFrictionResidual = 0.0;
    Vec3 firstImpulse;
    Vec3 secondImpulse;
    Vec3 netInternalImpulse;
    Vec3 firstMoment;
    Vec3 secondMoment;
    Vec3 netInternalMoment;
    double frictionWork = 0.0;
    int maximumCcdIterations = 0;
    int maximumIntervalSubdivisions = 0;
    bool hasFailure = false;
    ContactFeatureKey failureKey;
};

class ContactStepError : public std::runtime_error {
public:
    ContactStepError(const ContactFeatureKey& key, const std::string& message)
        : std::runtime_error(message), key_(key) {}

    [[nodiscard]] const ContactFeatureKey& key() const { return key_; }

private:
    ContactFeatureKey key_;
};

[[nodiscard]] CcdResult sweptContact(
    ContactFeatureKind kind,
    const LinearContactPrimitive& first,
    const LinearContactPrimitive& second,
    double separation,
    const CcdSettings& settings = {});

[[nodiscard]] SweptAabb sweptExpandedAabb(
    std::span<const Vec3> previous,
    std::span<const Vec3> current,
    double expansion);

[[nodiscard]] BroadphaseResult sweepAndPruneContactPairs(
    std::span<const BroadphaseProxy> proxies,
    std::span<const BroadphaseFeaturePair> supportedPairs);

struct ContactPairSettings {
    // Normal compliance is expressed in m/N. Friction coefficients are
    // dimensionless synthetic Coulomb parameters.
    double normalCompliance = 0.0;
    double staticFriction = 0.0;
    double dynamicFriction = 0.0;
};

class ContactColliderHandle {
public:
    [[nodiscard]] ContactColliderKind kind() const { return kind_; }
    [[nodiscard]] std::size_t index() const { return index_; }

private:
    friend class SoftBody;
    friend class ContactSurfaceHandle;
    friend class ContactLineHandle;

    ContactColliderHandle(const SoftBody* owner,
                          ContactColliderKind kind,
                          std::size_t index)
        : owner_(owner), kind_(kind), index_(index) {}

    const SoftBody* owner_ = nullptr;
    ContactColliderKind kind_ = ContactColliderKind::Surface;
    std::size_t index_ = 0;
};

class ContactSurfaceHandle {
public:
    [[nodiscard]] std::size_t index() const { return index_; }
    [[nodiscard]] ContactColliderHandle collider() const {
        return ContactColliderHandle(
            owner_, ContactColliderKind::Surface, index_);
    }

private:
    friend class SoftBody;

    ContactSurfaceHandle(const SoftBody* owner, std::size_t index)
        : owner_(owner), index_(index) {}

    const SoftBody* owner_ = nullptr;
    std::size_t index_ = 0;
};

class ContactLineHandle {
public:
    [[nodiscard]] std::size_t index() const { return index_; }
    [[nodiscard]] ContactColliderHandle collider() const {
        return ContactColliderHandle(owner_, ContactColliderKind::Line, index_);
    }

private:
    friend class SoftBody;

    ContactLineHandle(const SoftBody* owner, std::size_t index)
        : owner_(owner), index_(index) {}

    const SoftBody* owner_ = nullptr;
    std::size_t index_ = 0;
};

class ContactPairHandle {
public:
    [[nodiscard]] std::size_t index() const { return index_; }

private:
    friend class SoftBody;

    ContactPairHandle(const SoftBody* owner, std::size_t index)
        : owner_(owner), index_(index) {}

    const SoftBody* owner_ = nullptr;
    std::size_t index_ = 0;
};

struct ContactEdge {
    std::size_t a = 0;
    std::size_t b = 0;

    auto operator<=>(const ContactEdge&) const = default;
};

struct RegisteredContactSurface {
    std::size_t firstTriangle = 0;
    std::size_t triangleCount = 0;
    double halfThickness = 0.0;
    std::vector<std::size_t> vertices;
    std::vector<ContactEdge> edges;
};

struct RegisteredContactLine {
    std::size_t a = 0;
    std::size_t b = 0;
    double radius = 0.0;
};

struct RegisteredContactPair {
    ContactPairKind kind = ContactPairKind::SurfaceSurface;
    ContactColliderKind firstKind = ContactColliderKind::Surface;
    std::size_t first = 0;
    ContactColliderKind secondKind = ContactColliderKind::Surface;
    std::size_t second = 0;
    ContactPairSettings settings;
};

} // namespace softwing
