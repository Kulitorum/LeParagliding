#include "softwing/soft_body.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace softwing {
namespace {

bool finite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

bool validSeparation(double separation) {
    return std::isfinite(separation) && separation >= 0.0;
}

ClosestFeatureClass segmentFeature(double parameter) {
    constexpr double tolerance =
        64.0 * std::numeric_limits<double>::epsilon();
    if (parameter <= tolerance) {
        return ClosestFeatureClass::SegmentStart;
    }
    if (parameter >= 1.0 - tolerance) {
        return ClosestFeatureClass::SegmentEnd;
    }
    return ClosestFeatureClass::SegmentInterior;
}

ClosestFeatureClass triangleFeatureFromWeights(
    const std::array<double, 3>& weights) {
    constexpr double tolerance =
        128.0 * std::numeric_limits<double>::epsilon();
    if (weights[0] >= 1.0 - tolerance) {
        return ClosestFeatureClass::TriangleVertex0;
    }
    if (weights[1] >= 1.0 - tolerance) {
        return ClosestFeatureClass::TriangleVertex1;
    }
    if (weights[2] >= 1.0 - tolerance) {
        return ClosestFeatureClass::TriangleVertex2;
    }
    if (weights[2] <= tolerance) {
        return ClosestFeatureClass::TriangleEdge01;
    }
    if (weights[0] <= tolerance) {
        return ClosestFeatureClass::TriangleEdge12;
    }
    if (weights[1] <= tolerance) {
        return ClosestFeatureClass::TriangleEdge20;
    }
    return ClosestFeatureClass::TriangleFace;
}

Vec3 deterministicPerpendicular(const Vec3& direction) {
    const Vec3 absolute{std::abs(direction.x),
                        std::abs(direction.y),
                        std::abs(direction.z)};
    const Vec3 axis = absolute.x <= absolute.y && absolute.x <= absolute.z
                          ? Vec3{1.0, 0.0, 0.0}
                      : absolute.y <= absolute.z ? Vec3{0.0, 1.0, 0.0}
                                                 : Vec3{0.0, 0.0, 1.0};
    return normalized(cross(direction, axis));
}

void finishClosestResult(ClosestFeatureResult& result,
                         const Vec3& zeroDistanceNormal,
                         double separation) {
    const Vec3 difference = result.firstPoint - result.secondPoint;
    result.distance = length(difference);
    result.gap = result.distance - separation;
    const double scale = std::max(
        {1.0, length(result.firstPoint), length(result.secondPoint)});
    const double zeroTolerance =
        64.0 * std::numeric_limits<double>::epsilon() * scale;
    result.normal = result.distance > zeroTolerance
                        ? difference / result.distance
                        : normalized(zeroDistanceNormal);
    if (!finite(result.firstPoint) || !finite(result.secondPoint) ||
        !finite(result.normal) || !std::isfinite(result.distance) ||
        !std::isfinite(result.gap) || lengthSquared(result.normal) == 0.0) {
        result.status = GeometryQueryStatus::Uncertifiable;
    } else {
        result.status = GeometryQueryStatus::Certified;
    }
}

bool triangleBarycentric(const Vec3& point,
                         const Vec3& a,
                         const Vec3& b,
                         const Vec3& c,
                         std::array<double, 3>& weights) {
    const Vec3 first = b - a;
    const Vec3 second = c - a;
    const Vec3 relative = point - a;
    const double d00 = dot(first, first);
    const double d01 = dot(first, second);
    const double d11 = dot(second, second);
    const double d20 = dot(relative, first);
    const double d21 = dot(relative, second);
    const double denominator = d00 * d11 - d01 * d01;
    const double scale = d00 * d11;
    if (!(denominator > 64.0 * std::numeric_limits<double>::epsilon() *
                              scale)) {
        return false;
    }
    const double secondWeight =
        (d11 * d20 - d01 * d21) / denominator;
    const double thirdWeight =
        (d00 * d21 - d01 * d20) / denominator;
    weights = {1.0 - secondWeight - thirdWeight,
               secondWeight,
               thirdWeight};
    constexpr double tolerance =
        256.0 * std::numeric_limits<double>::epsilon();
    return weights[0] >= -tolerance && weights[1] >= -tolerance &&
           weights[2] >= -tolerance;
}

void validatePairSettings(const ContactPairSettings& settings) {
    if (!std::isfinite(settings.normalCompliance) ||
        !std::isfinite(settings.staticFriction) ||
        !std::isfinite(settings.dynamicFriction) ||
        settings.normalCompliance < 0.0 || settings.dynamicFriction < 0.0 ||
        settings.staticFriction < settings.dynamicFriction) {
        throw std::invalid_argument(
            "Contact requires finite non-negative compliance and "
            "0 <= dynamic friction <= static friction");
    }
}

} // namespace

ClosestFeatureResult closestVertexTriangle(const Vec3& vertex,
                                           const Vec3& a,
                                           const Vec3& b,
                                           const Vec3& c,
                                           double separation) {
    ClosestFeatureResult result;
    result.firstFeature = ClosestFeatureClass::Point;
    result.firstPoint = vertex;
    result.firstWeights = {1.0, 0.0, 0.0};
    if (!finite(vertex) || !finite(a) || !finite(b) || !finite(c) ||
        !std::isfinite(separation)) {
        result.status = GeometryQueryStatus::NonFinite;
        return result;
    }
    if (!validSeparation(separation)) {
        result.status = GeometryQueryStatus::Uncertifiable;
        return result;
    }
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 bc = c - b;
    const double scaleSquared = std::max(
        {lengthSquared(ab), lengthSquared(ac), lengthSquared(bc)});
    const Vec3 areaNormal = cross(ab, ac);
    const double areaSquared = lengthSquared(areaNormal);
    const double relativeTolerance =
        64.0 * std::numeric_limits<double>::epsilon();
    if (!(scaleSquared > 0.0) ||
        !(areaSquared > relativeTolerance * relativeTolerance *
                             scaleSquared * scaleSquared)) {
        result.status = GeometryQueryStatus::Degenerate;
        return result;
    }

    const Vec3 ap = vertex - a;
    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        result.secondPoint = a;
        result.secondWeights = {1.0, 0.0, 0.0};
    } else {
        const Vec3 bp = vertex - b;
        const double d3 = dot(ab, bp);
        const double d4 = dot(ac, bp);
        if (d3 >= 0.0 && d4 <= d3) {
            result.secondPoint = b;
            result.secondWeights = {0.0, 1.0, 0.0};
        } else {
            const double vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
                const double edgeParameter = d1 / (d1 - d3);
                result.secondPoint = a + edgeParameter * ab;
                result.secondWeights =
                    {1.0 - edgeParameter, edgeParameter, 0.0};
            } else {
                const Vec3 cp = vertex - c;
                const double d5 = dot(ab, cp);
                const double d6 = dot(ac, cp);
                if (d6 >= 0.0 && d5 <= d6) {
                    result.secondPoint = c;
                    result.secondWeights = {0.0, 0.0, 1.0};
                } else {
                    const double vb = d5 * d2 - d1 * d6;
                    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
                        const double edgeParameter = d2 / (d2 - d6);
                        result.secondPoint = a + edgeParameter * ac;
                        result.secondWeights =
                            {1.0 - edgeParameter, 0.0, edgeParameter};
                    } else {
                        const double va = d3 * d6 - d5 * d4;
                        if (va <= 0.0 && d4 - d3 >= 0.0 &&
                            d5 - d6 >= 0.0) {
                            const double edgeParameter =
                                (d4 - d3) /
                                ((d4 - d3) + (d5 - d6));
                            result.secondPoint =
                                b + edgeParameter * (c - b);
                            result.secondWeights =
                                {0.0,
                                 1.0 - edgeParameter,
                                 edgeParameter};
                        } else {
                            const double denominator =
                                1.0 / (va + vb + vc);
                            const double secondWeight = vb * denominator;
                            const double thirdWeight = vc * denominator;
                            result.secondWeights =
                                {1.0 - secondWeight - thirdWeight,
                                 secondWeight,
                                 thirdWeight};
                            result.secondPoint =
                                result.secondWeights[0] * a +
                                result.secondWeights[1] * b +
                                result.secondWeights[2] * c;
                        }
                    }
                }
            }
        }
    }
    result.secondFeature =
        triangleFeatureFromWeights(result.secondWeights);
    finishClosestResult(result, areaNormal, separation);
    return result;
}

ClosestFeatureResult closestEdgeEdge(const Vec3& firstA,
                                     const Vec3& firstB,
                                     const Vec3& secondA,
                                     const Vec3& secondB,
                                     double separation) {
    ClosestFeatureResult result;
    if (!finite(firstA) || !finite(firstB) || !finite(secondA) ||
        !finite(secondB) || !std::isfinite(separation)) {
        result.status = GeometryQueryStatus::NonFinite;
        return result;
    }
    if (!validSeparation(separation)) {
        result.status = GeometryQueryStatus::Uncertifiable;
        return result;
    }
    const Vec3 firstDirection = firstB - firstA;
    const Vec3 secondDirection = secondB - secondA;
    const Vec3 relative = firstA - secondA;
    const double a = dot(firstDirection, firstDirection);
    const double b = dot(firstDirection, secondDirection);
    const double c = dot(secondDirection, secondDirection);
    const double d = dot(firstDirection, relative);
    const double e = dot(secondDirection, relative);
    if (!(a > 0.0) || !(c > 0.0)) {
        result.status = GeometryQueryStatus::Degenerate;
        return result;
    }
    const double denominator = a * c - b * b;
    const double parallelTolerance =
        64.0 * std::numeric_limits<double>::epsilon() * a * c;
    double firstParameter = 0.0;
    if (denominator > parallelTolerance) {
        firstParameter = std::clamp((b * e - c * d) / denominator,
                                    0.0,
                                    1.0);
    }
    double secondParameter = (b * firstParameter + e) / c;
    if (secondParameter < 0.0) {
        secondParameter = 0.0;
        firstParameter = std::clamp(-d / a, 0.0, 1.0);
    } else if (secondParameter > 1.0) {
        secondParameter = 1.0;
        firstParameter = std::clamp((b - d) / a, 0.0, 1.0);
    }

    result.firstPoint = firstA + firstParameter * firstDirection;
    result.secondPoint = secondA + secondParameter * secondDirection;
    result.firstWeights = {1.0 - firstParameter, firstParameter, 0.0};
    result.secondWeights = {1.0 - secondParameter, secondParameter, 0.0};
    result.firstFeature = segmentFeature(firstParameter);
    result.secondFeature = segmentFeature(secondParameter);
    Vec3 zeroNormal = cross(firstDirection, secondDirection);
    if (lengthSquared(zeroNormal) <= parallelTolerance) {
        zeroNormal = deterministicPerpendicular(firstDirection);
    }
    finishClosestResult(result, zeroNormal, separation);
    return result;
}

ClosestFeatureResult closestSegmentTriangle(const Vec3& segmentA,
                                            const Vec3& segmentB,
                                            const Vec3& triangleA,
                                            const Vec3& triangleB,
                                            const Vec3& triangleC,
                                            double separation) {
    ClosestFeatureResult result;
    if (!finite(segmentA) || !finite(segmentB) || !finite(triangleA) ||
        !finite(triangleB) || !finite(triangleC) ||
        !std::isfinite(separation)) {
        result.status = GeometryQueryStatus::NonFinite;
        return result;
    }
    if (!validSeparation(separation)) {
        result.status = GeometryQueryStatus::Uncertifiable;
        return result;
    }
    const Vec3 segmentDirection = segmentB - segmentA;
    if (!(lengthSquared(segmentDirection) > 0.0)) {
        result.status = GeometryQueryStatus::Degenerate;
        return result;
    }
    const Vec3 triangleNormal =
        cross(triangleB - triangleA, triangleC - triangleA);
    const double normalLength = length(triangleNormal);
    const double triangleScaleSquared = std::max(
        {lengthSquared(triangleB - triangleA),
         lengthSquared(triangleC - triangleA),
         lengthSquared(triangleC - triangleB)});
    if (!(triangleScaleSquared > 0.0) ||
        !(normalLength >
          64.0 * std::numeric_limits<double>::epsilon() *
              triangleScaleSquared)) {
        result.status = GeometryQueryStatus::Degenerate;
        return result;
    }

    const double planeDenominator = dot(triangleNormal, segmentDirection);
    const double planeTolerance =
        64.0 * std::numeric_limits<double>::epsilon() * normalLength *
        length(segmentDirection);
    if (std::abs(planeDenominator) > planeTolerance) {
        const double parameter =
            dot(triangleNormal, triangleA - segmentA) / planeDenominator;
        constexpr double parameterTolerance =
            256.0 * std::numeric_limits<double>::epsilon();
        if (parameter >= -parameterTolerance &&
            parameter <= 1.0 + parameterTolerance) {
            const double clampedParameter = std::clamp(parameter, 0.0, 1.0);
            const Vec3 point =
                segmentA + clampedParameter * segmentDirection;
            std::array<double, 3> triangleWeights;
            if (triangleBarycentric(
                    point, triangleA, triangleB, triangleC, triangleWeights)) {
                result.firstPoint = point;
                result.secondPoint = point;
                result.firstWeights =
                    {1.0 - clampedParameter, clampedParameter, 0.0};
                result.secondWeights = triangleWeights;
                result.firstFeature = segmentFeature(clampedParameter);
                result.secondFeature =
                    triangleFeatureFromWeights(triangleWeights);
                finishClosestResult(result, triangleNormal, separation);
                return result;
            }
        }
    }

    bool hasBest = false;
    const auto consider = [&result, &hasBest](
                              const ClosestFeatureResult& candidate) {
        if (!candidate.certified()) {
            return;
        }
        const double tolerance =
            64.0 * std::numeric_limits<double>::epsilon() *
            std::max({1.0, candidate.distance, result.distance});
        if (!hasBest || candidate.distance < result.distance - tolerance) {
            result = candidate;
            hasBest = true;
        }
    };

    for (std::size_t endpoint = 0; endpoint < 2; ++endpoint) {
        ClosestFeatureResult candidate = closestVertexTriangle(
            endpoint == 0 ? segmentA : segmentB,
            triangleA,
            triangleB,
            triangleC,
            separation);
        if (candidate.certified()) {
            candidate.firstFeature = endpoint == 0
                                         ? ClosestFeatureClass::SegmentStart
                                         : ClosestFeatureClass::SegmentEnd;
            candidate.firstWeights = endpoint == 0
                                         ? std::array<double, 3>{1.0, 0.0, 0.0}
                                         : std::array<double, 3>{0.0, 1.0, 0.0};
        }
        consider(candidate);
    }

    const std::array<Vec3, 3> triangleVertices{
        triangleA, triangleB, triangleC};
    for (std::size_t edge = 0; edge < 3; ++edge) {
        ClosestFeatureResult candidate = closestEdgeEdge(
            segmentA,
            segmentB,
            triangleVertices[edge],
            triangleVertices[(edge + 1) % 3],
            separation);
        if (!candidate.certified()) {
            continue;
        }
        const double edgeParameter = candidate.secondWeights[1];
        std::array<double, 3> triangleWeights{};
        triangleWeights[edge] = 1.0 - edgeParameter;
        triangleWeights[(edge + 1) % 3] = edgeParameter;
        candidate.secondWeights = triangleWeights;
        candidate.secondFeature =
            triangleFeatureFromWeights(triangleWeights);
        consider(candidate);
    }
    if (!hasBest) {
        result.status = GeometryQueryStatus::Uncertifiable;
    }
    return result;
}

CoulombFrictionResult solveCoulombFriction(
    const Vec3& relativeVelocity,
    const Vec3& normal,
    double tangentialEffectiveInverseMass,
    double normalImpulse,
    double staticFriction,
    double dynamicFriction,
    bool normalContactActive) {
    CoulombFrictionResult result;
    if (!finite(relativeVelocity) || !finite(normal) ||
        !std::isfinite(tangentialEffectiveInverseMass) ||
        !std::isfinite(normalImpulse) || !std::isfinite(staticFriction) ||
        !std::isfinite(dynamicFriction) ||
        !(tangentialEffectiveInverseMass > 0.0) || normalImpulse < 0.0 ||
        dynamicFriction < 0.0 || staticFriction < dynamicFriction ||
        std::abs(lengthSquared(normal) - 1.0) > 1.0e-8) {
        return result;
    }
    result.initialTangentialVelocity =
        relativeVelocity - dot(relativeVelocity, normal) * normal;
    result.finalTangentialVelocity = result.initialTangentialVelocity;
    const double tangentSpeed = length(result.initialTangentialVelocity);
    if (!normalContactActive || !(normalImpulse > 0.0) ||
        !(tangentSpeed > 0.0)) {
        return result;
    }
    const Vec3 stopImpulse =
        -result.initialTangentialVelocity /
        tangentialEffectiveInverseMass;
    const double stopMagnitude = length(stopImpulse);
    const double staticLimit = staticFriction * normalImpulse;
    if (stopMagnitude <= staticLimit) {
        result.state = ContactFrictionState::Sticking;
        result.tangentialImpulse = stopImpulse;
    } else {
        result.state = ContactFrictionState::Sliding;
        const double appliedMagnitude =
            std::min(dynamicFriction * normalImpulse, stopMagnitude);
        result.tangentialImpulse =
            -appliedMagnitude *
            (result.initialTangentialVelocity / tangentSpeed);
    }
    result.finalTangentialVelocity =
        result.initialTangentialVelocity +
        tangentialEffectiveInverseMass * result.tangentialImpulse;
    const double impulseMagnitude = length(result.tangentialImpulse);
    result.coneRatio = staticLimit > 0.0 ? impulseMagnitude / staticLimit : 0.0;
    result.work = dot(result.tangentialImpulse,
                      result.initialTangentialVelocity) +
                  0.5 * tangentialEffectiveInverseMass *
                      lengthSquared(result.tangentialImpulse);
    return result;
}

SweptAabb sweptExpandedAabb(std::span<const Vec3> previous,
                            std::span<const Vec3> current,
                            double expansion) {
    if (previous.empty() || previous.size() != current.size() ||
        !std::isfinite(expansion) || expansion < 0.0) {
        throw std::invalid_argument("Invalid swept AABB input");
    }
    SweptAabb result{previous[0], previous[0]};
    double coordinateScale = 1.0;
    const auto include = [&result, &coordinateScale](const Vec3& point) {
        if (!finite(point)) {
            throw std::invalid_argument(
                "Swept AABB positions must be finite");
        }
        result.minimum.x = std::min(result.minimum.x, point.x);
        result.minimum.y = std::min(result.minimum.y, point.y);
        result.minimum.z = std::min(result.minimum.z, point.z);
        result.maximum.x = std::max(result.maximum.x, point.x);
        result.maximum.y = std::max(result.maximum.y, point.y);
        result.maximum.z = std::max(result.maximum.z, point.z);
        coordinateScale = std::max(
            {coordinateScale,
             std::abs(point.x),
             std::abs(point.y),
             std::abs(point.z)});
    };
    for (std::size_t index = 0; index < previous.size(); ++index) {
        include(previous[index]);
        include(current[index]);
    }
    const double numericalMargin =
        64.0 * std::numeric_limits<double>::epsilon() *
        std::max(coordinateScale, expansion);
    const double totalExpansion = expansion + numericalMargin;
    const Vec3 offset{totalExpansion, totalExpansion, totalExpansion};
    result.minimum -= offset;
    result.maximum += offset;
    return result;
}

BroadphaseResult sweepAndPruneContactPairs(
    std::span<const BroadphaseProxy> proxies,
    std::span<const BroadphaseFeaturePair> supportedPairs) {
    BroadphaseResult result;
    result.possibleCount = supportedPairs.size();
    if (proxies.empty()) {
        if (!supportedPairs.empty()) {
            throw std::invalid_argument(
                "Broadphase pairs require registered proxies");
        }
        return result;
    }
    Vec3 globalMinimum = proxies[0].bounds.minimum;
    Vec3 globalMaximum = proxies[0].bounds.maximum;
    const auto validateAndInclude = [&globalMinimum, &globalMaximum](
                                        const SweptAabb& bounds) {
        if (!finite(bounds.minimum) || !finite(bounds.maximum) ||
            bounds.minimum.x > bounds.maximum.x ||
            bounds.minimum.y > bounds.maximum.y ||
            bounds.minimum.z > bounds.maximum.z) {
            throw std::invalid_argument("Invalid broadphase AABB");
        }
        globalMinimum.x = std::min(globalMinimum.x, bounds.minimum.x);
        globalMinimum.y = std::min(globalMinimum.y, bounds.minimum.y);
        globalMinimum.z = std::min(globalMinimum.z, bounds.minimum.z);
        globalMaximum.x = std::max(globalMaximum.x, bounds.maximum.x);
        globalMaximum.y = std::max(globalMaximum.y, bounds.maximum.y);
        globalMaximum.z = std::max(globalMaximum.z, bounds.maximum.z);
    };
    std::map<std::size_t, const BroadphaseProxy*> proxiesById;
    for (const BroadphaseProxy& proxy : proxies) {
        validateAndInclude(proxy.bounds);
        if (!proxiesById.emplace(proxy.id, &proxy).second) {
            throw std::invalid_argument("Duplicate broadphase proxy id");
        }
    }
    using ProxyPair = std::pair<std::size_t, std::size_t>;
    std::map<ProxyPair, std::vector<const BroadphaseFeaturePair*>> links;
    for (const BroadphaseFeaturePair& pair : supportedPairs) {
        if (!proxiesById.contains(pair.firstProxy) ||
            !proxiesById.contains(pair.secondProxy) ||
            pair.firstProxy == pair.secondProxy) {
            throw std::invalid_argument(
                "Broadphase pair references invalid proxies");
        }
        links[{std::min(pair.firstProxy, pair.secondProxy),
               std::max(pair.firstProxy, pair.secondProxy)}]
            .push_back(&pair);
    }
    const Vec3 extent = globalMaximum - globalMinimum;
    if (extent.y > extent.x && extent.y >= extent.z) {
        result.axis = BroadphaseAxis::Y;
    } else if (extent.z > extent.x && extent.z > extent.y) {
        result.axis = BroadphaseAxis::Z;
    }

    const auto component = [](const Vec3& value, BroadphaseAxis axis) {
        switch (axis) {
        case BroadphaseAxis::X:
            return value.x;
        case BroadphaseAxis::Y:
            return value.y;
        case BroadphaseAxis::Z:
            return value.z;
        }
        return value.x;
    };
    std::vector<const BroadphaseProxy*> sorted;
    sorted.reserve(proxies.size());
    for (const BroadphaseProxy& proxy : proxies) {
        sorted.push_back(&proxy);
    }
    std::sort(sorted.begin(),
              sorted.end(),
              [&component, &result](const BroadphaseProxy* first,
                                    const BroadphaseProxy* second) {
                  const double firstMinimum =
                      component(first->bounds.minimum, result.axis);
                  const double secondMinimum =
                      component(second->bounds.minimum, result.axis);
                  if (firstMinimum != secondMinimum) {
                      return firstMinimum < secondMinimum;
                  }
                  const double firstMaximum =
                      component(first->bounds.maximum, result.axis);
                  const double secondMaximum =
                      component(second->bounds.maximum, result.axis);
                  if (firstMaximum != secondMaximum) {
                      return firstMaximum < secondMaximum;
                  }
                  return first->id < second->id;
              });
    const auto overlapsAxis = [&component](const SweptAabb& first,
                                           const SweptAabb& second,
                                           BroadphaseAxis axis) {
        return component(first.minimum, axis) <=
                   component(second.maximum, axis) &&
               component(second.minimum, axis) <=
                   component(first.maximum, axis);
    };
    std::vector<const BroadphaseProxy*> active;
    for (const BroadphaseProxy* current : sorted) {
        const double currentMinimum =
            component(current->bounds.minimum, result.axis);
        std::erase_if(active,
                      [&component, &result, currentMinimum](
                          const BroadphaseProxy* proxy) {
                          return component(proxy->bounds.maximum,
                                           result.axis) < currentMinimum;
                      });
        for (const BroadphaseProxy* other : active) {
            const ProxyPair proxyPair{
                std::min(current->id, other->id),
                std::max(current->id, other->id)};
            const auto link = links.find(proxyPair);
            if (link == links.end()) {
                continue;
            }
            if (overlapsAxis(current->bounds,
                             other->bounds,
                             BroadphaseAxis::X) &&
                overlapsAxis(current->bounds,
                             other->bounds,
                             BroadphaseAxis::Y) &&
                overlapsAxis(current->bounds,
                             other->bounds,
                             BroadphaseAxis::Z)) {
                for (const BroadphaseFeaturePair* pair : link->second) {
                    result.candidateKeys.push_back(pair->key);
                }
            }
        }
        active.push_back(current);
    }
    std::sort(result.candidateKeys.begin(), result.candidateKeys.end());
    result.candidateKeys.erase(
        std::unique(result.candidateKeys.begin(), result.candidateKeys.end()),
        result.candidateKeys.end());
    return result;
}

CcdResult sweptContact(ContactFeatureKind kind,
                       const LinearContactPrimitive& first,
                       const LinearContactPrimitive& second,
                       double separation,
                       const CcdSettings& settings) {
    CcdResult result;
    const bool expectedCounts =
        (kind == ContactFeatureKind::VertexTriangle && first.count == 1 &&
         second.count == 3) ||
        (kind == ContactFeatureKind::EdgeEdge && first.count == 2 &&
         second.count == 2) ||
        (kind == ContactFeatureKind::SegmentTriangle && first.count == 2 &&
         second.count == 3);
    if (!expectedCounts || !validSeparation(separation) ||
        settings.conservativeIterations < 0 ||
        settings.intervalSubdivisions < 0 ||
        !std::isfinite(settings.timeTolerance) ||
        !(settings.timeTolerance > 0.0) ||
        !std::isfinite(settings.distanceTolerance) ||
        !(settings.distanceTolerance > 0.0)) {
        return result;
    }
    for (std::size_t index = 0; index < first.count; ++index) {
        if (!finite(first.previous[index]) || !finite(first.current[index])) {
            return result;
        }
    }
    for (std::size_t index = 0; index < second.count; ++index) {
        if (!finite(second.previous[index]) ||
            !finite(second.current[index])) {
            return result;
        }
    }

    const auto interpolate = [](const LinearContactPrimitive& primitive,
                                double time) {
        std::array<Vec3, 3> positions{};
        for (std::size_t index = 0; index < primitive.count; ++index) {
            positions[index] = primitive.previous[index] +
                               time * (primitive.current[index] -
                                       primitive.previous[index]);
        }
        return positions;
    };
    const auto evaluate = [&](double time) {
        const std::array<Vec3, 3> firstPositions = interpolate(first, time);
        const std::array<Vec3, 3> secondPositions = interpolate(second, time);
        switch (kind) {
        case ContactFeatureKind::VertexTriangle:
            return closestVertexTriangle(firstPositions[0],
                                         secondPositions[0],
                                         secondPositions[1],
                                         secondPositions[2],
                                         separation);
        case ContactFeatureKind::EdgeEdge:
            return closestEdgeEdge(firstPositions[0],
                                   firstPositions[1],
                                   secondPositions[0],
                                   secondPositions[1],
                                   separation);
        case ContactFeatureKind::SegmentTriangle:
            return closestSegmentTriangle(firstPositions[0],
                                          firstPositions[1],
                                          secondPositions[0],
                                          secondPositions[1],
                                          secondPositions[2],
                                          separation);
        }
        return ClosestFeatureResult{};
    };
    double firstSpeed = 0.0;
    for (std::size_t index = 0; index < first.count; ++index) {
        firstSpeed = std::max(
            firstSpeed, length(first.current[index] - first.previous[index]));
    }
    double secondSpeed = 0.0;
    for (std::size_t index = 0; index < second.count; ++index) {
        secondSpeed = std::max(
            secondSpeed,
            length(second.current[index] - second.previous[index]));
    }
    const double relativeSpeedBound = firstSpeed + secondSpeed;
    const auto isImpact = [&settings](const ClosestFeatureResult& geometry) {
        return geometry.certified() &&
               geometry.gap <= settings.distanceTolerance;
    };

    const ClosestFeatureResult initialGeometry = evaluate(0.0);
    if (!initialGeometry.certified()) {
        result.geometry = initialGeometry;
        return result;
    }
    if (isImpact(initialGeometry)) {
        result.state = CcdState::InitiallyProximate;
        result.timeOfImpact = 0.0;
        result.bracketLower = 0.0;
        result.bracketUpper = 0.0;
        result.geometry = initialGeometry;
        return result;
    }
    if (!(relativeSpeedBound > 0.0)) {
        result.state = CcdState::NoImpact;
        result.timeOfImpact = 1.0;
        result.bracketLower = 1.0;
        result.bracketUpper = 1.0;
        result.geometry = initialGeometry;
        return result;
    }
    if (settings.conservativeIterations == 0 &&
        settings.intervalSubdivisions == 0) {
        result.geometry = initialGeometry;
        return result;
    }

    const auto refineImpact = [&](double safeTime,
                                  double impactTime,
                                  ClosestFeatureResult impactGeometry,
                                  CcdResult& refined) {
        double lower = safeTime;
        double upper = impactTime;
        while (upper - lower > settings.timeTolerance) {
            const double middle = 0.5 * (lower + upper);
            const ClosestFeatureResult geometry = evaluate(middle);
            if (!geometry.certified()) {
                refined.state = CcdState::Indeterminate;
                refined.geometry = geometry;
                return false;
            }
            if (isImpact(geometry)) {
                upper = middle;
                impactGeometry = geometry;
            } else {
                lower = middle;
            }
        }
        refined.state = CcdState::Impact;
        refined.timeOfImpact = upper;
        refined.bracketLower = lower;
        refined.bracketUpper = upper;
        refined.geometry = impactGeometry;
        return true;
    };

    double currentTime = 0.0;
    ClosestFeatureResult currentGeometry = initialGeometry;
    bool needsFallback = false;
    for (int iteration = 0;
         iteration < settings.conservativeIterations;
         ++iteration) {
        ++result.conservativeIterations;
        const double safeGap =
            currentGeometry.gap - settings.distanceTolerance;
        const double advance = 0.9 * safeGap / relativeSpeedBound;
        if (!(advance > 0.0) || !std::isfinite(advance)) {
            needsFallback = true;
            break;
        }
        const double nextTime = currentTime + advance;
        if (nextTime >= 1.0) {
            const double lowerBound = currentGeometry.distance -
                                      relativeSpeedBound *
                                          (1.0 - currentTime);
            if (lowerBound > separation + settings.distanceTolerance) {
                result.state = CcdState::NoImpact;
                result.timeOfImpact = 1.0;
                result.bracketLower = currentTime;
                result.bracketUpper = 1.0;
                result.geometry = evaluate(1.0);
                if (!result.geometry.certified()) {
                    result.state = CcdState::Indeterminate;
                }
                return result;
            }
            const ClosestFeatureResult endpoint = evaluate(1.0);
            if (!endpoint.certified()) {
                result.geometry = endpoint;
                return result;
            }
            if (isImpact(endpoint)) {
                static_cast<void>(
                    refineImpact(currentTime, 1.0, endpoint, result));
                return result;
            }
            needsFallback = true;
            break;
        }
        const ClosestFeatureResult nextGeometry = evaluate(nextTime);
        if (!nextGeometry.certified()) {
            result.geometry = nextGeometry;
            return result;
        }
        if (isImpact(nextGeometry)) {
            static_cast<void>(refineImpact(
                currentTime, nextTime, nextGeometry, result));
            return result;
        }
        currentTime = nextTime;
        currentGeometry = nextGeometry;
    }
    if (result.conservativeIterations >=
        settings.conservativeIterations) {
        needsFallback = true;
    }
    if (!needsFallback || settings.intervalSubdivisions == 0) {
        result.geometry = currentGeometry;
        return result;
    }

    result.usedIntervalFallback = true;
    struct Interval {
        double lower = 0.0;
        double upper = 1.0;
        ClosestFeatureResult lowerGeometry;
        ClosestFeatureResult upperGeometry;
    };
    const ClosestFeatureResult endpointGeometry = evaluate(1.0);
    if (!endpointGeometry.certified()) {
        result.geometry = endpointGeometry;
        return result;
    }
    if (isImpact(endpointGeometry)) {
        static_cast<void>(
            refineImpact(0.0, 1.0, endpointGeometry, result));
        result.usedIntervalFallback = true;
        return result;
    }
    std::deque<Interval> intervals;
    intervals.push_back({0.0, 1.0, initialGeometry, endpointGeometry});
    while (!intervals.empty()) {
        const Interval interval = intervals.front();
        intervals.pop_front();
        const double width = interval.upper - interval.lower;
        const double certifiedLowerDistance =
            0.5 * (interval.lowerGeometry.distance +
                   interval.upperGeometry.distance -
                   relativeSpeedBound * width);
        if (certifiedLowerDistance >
            separation + settings.distanceTolerance) {
            continue;
        }
        if (result.intervalSubdivisions >=
            settings.intervalSubdivisions) {
            result.geometry = interval.lowerGeometry;
            return result;
        }
        const double middle = 0.5 * (interval.lower + interval.upper);
        const ClosestFeatureResult middleGeometry = evaluate(middle);
        ++result.intervalSubdivisions;
        if (!middleGeometry.certified()) {
            result.geometry = middleGeometry;
            return result;
        }
        if (isImpact(middleGeometry)) {
            static_cast<void>(refineImpact(interval.lower,
                                           middle,
                                           middleGeometry,
                                           result));
            result.usedIntervalFallback = true;
            return result;
        }
        intervals.push_back({interval.lower,
                             middle,
                             interval.lowerGeometry,
                             middleGeometry});
        intervals.push_back({middle,
                             interval.upper,
                             middleGeometry,
                             interval.upperGeometry});
    }
    result.state = CcdState::NoImpact;
    result.timeOfImpact = 1.0;
    result.bracketLower = 1.0;
    result.bracketUpper = 1.0;
    result.geometry = endpointGeometry;
    return result;
}

namespace {

struct SolverContactCandidate {
    ContactFeatureKey key;
    ContactFeatureKind kind = ContactFeatureKind::VertexTriangle;
    std::array<std::size_t, 3> firstNodes{};
    std::array<std::size_t, 3> secondNodes{};
    std::size_t firstNodeCount = 0;
    std::size_t secondNodeCount = 0;
    double separation = 0.0;
    ContactPairSettings settings;
    LinearContactPrimitive firstPrimitive;
    LinearContactPrimitive secondPrimitive;
    std::size_t firstProxy = 0;
    std::size_t secondProxy = 0;
};

struct SolverProxyKey {
    std::size_t pair = 0;
    std::array<std::size_t, 3> nodes{};
    std::size_t nodeCount = 0;

    auto operator<=>(const SolverProxyKey&) const = default;
};

LinearContactPrimitive makeLinearPrimitive(
    const std::vector<Node>& nodes,
    const std::array<std::size_t, 3>& indices,
    std::size_t count) {
    LinearContactPrimitive result;
    result.count = count;
    for (std::size_t index = 0; index < count; ++index) {
        result.previous[index] = nodes[indices[index]].previousPosition;
        result.current[index] = nodes[indices[index]].position;
    }
    return result;
}

ClosestFeatureResult evaluateCurrent(
    const SolverContactCandidate& candidate) {
    switch (candidate.kind) {
    case ContactFeatureKind::VertexTriangle:
        return closestVertexTriangle(candidate.firstPrimitive.current[0],
                                     candidate.secondPrimitive.current[0],
                                     candidate.secondPrimitive.current[1],
                                     candidate.secondPrimitive.current[2],
                                     candidate.separation);
    case ContactFeatureKind::EdgeEdge:
        return closestEdgeEdge(candidate.firstPrimitive.current[0],
                               candidate.firstPrimitive.current[1],
                               candidate.secondPrimitive.current[0],
                               candidate.secondPrimitive.current[1],
                               candidate.separation);
    case ContactFeatureKind::SegmentTriangle:
        return closestSegmentTriangle(candidate.firstPrimitive.current[0],
                                      candidate.firstPrimitive.current[1],
                                      candidate.secondPrimitive.current[0],
                                      candidate.secondPrimitive.current[1],
                                      candidate.secondPrimitive.current[2],
                                      candidate.separation);
    }
    return {};
}

Vec3 weightedPoint(const std::vector<Node>& nodes,
                   const std::array<std::size_t, 3>& indices,
                   std::size_t count,
                   const std::array<double, 3>& weights) {
    Vec3 result;
    for (std::size_t index = 0; index < count; ++index) {
        result += weights[index] * nodes[indices[index]].position;
    }
    return result;
}

void updateRecordImpulseDiagnostics(ContactRecord& record,
                                    const std::vector<Node>& nodes) {
    record.firstImpulse = {};
    record.secondImpulse = {};
    record.firstMoment = {};
    record.secondMoment = {};
    for (std::size_t index = 0; index < record.firstNodeCount; ++index) {
        const Vec3& impulse = record.firstNodeImpulses[index];
        record.firstImpulse += impulse;
        record.firstMoment +=
            cross(nodes[record.firstNodes[index]].position, impulse);
    }
    for (std::size_t index = 0; index < record.secondNodeCount; ++index) {
        const Vec3& impulse = record.secondNodeImpulses[index];
        record.secondImpulse += impulse;
        record.secondMoment +=
            cross(nodes[record.secondNodes[index]].position, impulse);
    }
}

} // namespace

void SoftBody::beginContactSubstep() {
    contactMultipliers_.clear();
    contactRecords_.clear();
    contactDiagnostics_ = {};
    contactAudit_ = {};
    contactDiagnostics_.registered = !contactPairs_.empty();
    contactDiagnostics_.solveSucceeded = true;
    contactPairDiagnostics_.assign(contactPairs_.size(), {});
    for (ContactDiagnostics& diagnostics : contactPairDiagnostics_) {
        diagnostics.registered = true;
        diagnostics.solveSucceeded = true;
    }
}

void SoftBody::solveContactIteration(double dt,
                                     const StepSettings& stepSettings) {
    std::vector<SolverContactCandidate> candidates;
    std::vector<BroadphaseProxy> broadphaseProxies;
    std::vector<BroadphaseFeaturePair> broadphasePairs;
    std::map<SolverProxyKey, std::size_t> proxyIds;
    ContactDiagnostics diagnostics;
    diagnostics.registered = true;
    diagnostics.solveSucceeded = true;
    std::vector<ContactDiagnostics> pairDiagnostics(contactPairs_.size());
    for (ContactDiagnostics& pairDiagnostic : pairDiagnostics) {
        pairDiagnostic.registered = true;
        pairDiagnostic.solveSucceeded = true;
    }

    for (std::size_t pairIndex = 0; pairIndex < contactPairs_.size();
         ++pairIndex) {
        const RegisteredContactPair& pair = contactPairs_[pairIndex];
        std::size_t possibleCount = 0;
        std::size_t excludedCount = 0;
        double separation = 0.0;
        if (pair.kind == ContactPairKind::SurfaceSurface) {
            separation = contactSurfaces_[pair.first].halfThickness +
                         contactSurfaces_[pair.second].halfThickness;
        } else {
            separation = contactSurfaces_[pair.first].halfThickness +
                         contactLines_[pair.second].radius;
        }
        const auto proxyFor = [&](const std::array<std::size_t, 3>& nodeIds,
                                  std::size_t nodeCount,
                                  const LinearContactPrimitive& primitive) {
            const SolverProxyKey proxyKey{pairIndex, nodeIds, nodeCount};
            const auto existing = proxyIds.find(proxyKey);
            if (existing != proxyIds.end()) {
                return existing->second;
            }
            const std::size_t proxyId = broadphaseProxies.size();
            const SweptAabb bounds = sweptExpandedAabb(
                std::span<const Vec3>{primitive.previous.data(), nodeCount},
                std::span<const Vec3>{primitive.current.data(), nodeCount},
                separation);
            proxyIds.emplace(proxyKey, proxyId);
            broadphaseProxies.push_back({proxyId, bounds});
            return proxyId;
        };
        const auto emitEligible = [&](const ContactFeatureKey& key) {
            SolverContactCandidate candidate;
            candidate.key = key;
            candidate.kind = key.kind;
            candidate.separation = separation;
            candidate.settings = pair.settings;
            switch (key.kind) {
            case ContactFeatureKind::VertexTriangle: {
                candidate.firstNodes = {key.firstPrimitive[0], 0, 0};
                candidate.firstNodeCount = 1;
                const Triangle& triangle =
                    triangles_[key.secondPrimitive[0]];
                candidate.secondNodes =
                    {triangle.a, triangle.b, triangle.c};
                candidate.secondNodeCount = 3;
                break;
            }
            case ContactFeatureKind::EdgeEdge:
                candidate.firstNodes =
                    {key.firstPrimitive[0], key.firstPrimitive[1], 0};
                candidate.secondNodes =
                    {key.secondPrimitive[0], key.secondPrimitive[1], 0};
                candidate.firstNodeCount = 2;
                candidate.secondNodeCount = 2;
                break;
            case ContactFeatureKind::SegmentTriangle: {
                candidate.firstNodes =
                    {key.firstPrimitive[0], key.firstPrimitive[1], 0};
                candidate.firstNodeCount = 2;
                const Triangle& triangle =
                    triangles_[key.secondPrimitive[0]];
                candidate.secondNodes =
                    {triangle.a, triangle.b, triangle.c};
                candidate.secondNodeCount = 3;
                break;
            }
            }
            try {
                candidate.firstPrimitive = makeLinearPrimitive(
                    nodes_, candidate.firstNodes, candidate.firstNodeCount);
                candidate.secondPrimitive = makeLinearPrimitive(
                    nodes_, candidate.secondNodes, candidate.secondNodeCount);
                candidate.firstProxy = proxyFor(candidate.firstNodes,
                                                candidate.firstNodeCount,
                                                candidate.firstPrimitive);
                candidate.secondProxy = proxyFor(candidate.secondNodes,
                                                 candidate.secondNodeCount,
                                                 candidate.secondPrimitive);
            } catch (const std::exception& error) {
                throw ContactStepError(
                    candidate.key,
                    std::string("Contact broadphase input failed: ") +
                        error.what());
            }
            const SweptAabb& firstBounds =
                broadphaseProxies[candidate.firstProxy].bounds;
            const SweptAabb& secondBounds =
                broadphaseProxies[candidate.secondProxy].bounds;
            const auto overlaps = [](double firstMinimum,
                                     double firstMaximum,
                                     double secondMinimum,
                                     double secondMaximum) {
                return firstMinimum <= secondMaximum &&
                       secondMinimum <= firstMaximum;
            };
            if (!overlaps(firstBounds.minimum.x, firstBounds.maximum.x,
                          secondBounds.minimum.x, secondBounds.maximum.x) ||
                !overlaps(firstBounds.minimum.y, firstBounds.maximum.y,
                          secondBounds.minimum.y, secondBounds.maximum.y) ||
                !overlaps(firstBounds.minimum.z, firstBounds.maximum.z,
                          secondBounds.minimum.z, secondBounds.maximum.z))
                return;
            broadphasePairs.push_back(
                {key, candidate.firstProxy, candidate.secondProxy});
            candidates.push_back(std::move(candidate));
        };
        const auto triangleContains = [this](std::size_t triangleIndex,
                                              std::size_t nodeIndex) {
            const Triangle& triangle = triangles_[triangleIndex];
            return triangle.a == nodeIndex || triangle.b == nodeIndex ||
                   triangle.c == nodeIndex;
        };
        const auto emitVertexTriangles = [&](
            const RegisteredContactSurface& vertices,
            const RegisteredContactSurface& triangles) {
            const std::size_t triangleLast =
                triangles.firstTriangle + triangles.triangleCount;
            for (const std::size_t vertex : vertices.vertices) {
                for (std::size_t triangleIndex = triangles.firstTriangle;
                     triangleIndex < triangleLast; ++triangleIndex) {
                    ++possibleCount;
                    if (triangleContains(triangleIndex, vertex)) {
                        ++excludedCount;
                        continue;
                    }
                    emitEligible({pairIndex,
                                  ContactFeatureKind::VertexTriangle,
                                  {vertex, 0, 0},
                                  {triangleIndex, 0, 0}});
                }
            }
        };
        const auto emitEdgePair = [&](const ContactEdge& first,
                                      const ContactEdge& second) {
            ++possibleCount;
            if (first.a == second.a || first.a == second.b ||
                first.b == second.a || first.b == second.b) {
                ++excludedCount;
                return;
            }
            ContactEdge canonicalFirst = first;
            ContactEdge canonicalSecond = second;
            if (canonicalSecond < canonicalFirst)
                std::swap(canonicalFirst, canonicalSecond);
            emitEligible({pairIndex,
                          ContactFeatureKind::EdgeEdge,
                          {canonicalFirst.a, canonicalFirst.b, 0},
                          {canonicalSecond.a, canonicalSecond.b, 0}});
        };
        if (pair.kind == ContactPairKind::SurfaceSurface) {
            const RegisteredContactSurface& first =
                contactSurfaces_[pair.first];
            const RegisteredContactSurface& second =
                contactSurfaces_[pair.second];
            emitVertexTriangles(first, second);
            if (pair.first != pair.second) {
                emitVertexTriangles(second, first);
                for (const ContactEdge& firstEdge : first.edges)
                    for (const ContactEdge& secondEdge : second.edges)
                        emitEdgePair(firstEdge, secondEdge);
            } else {
                for (std::size_t firstEdge = 0;
                     firstEdge < first.edges.size(); ++firstEdge)
                    for (std::size_t secondEdge = firstEdge + 1;
                         secondEdge < first.edges.size(); ++secondEdge)
                        emitEdgePair(first.edges[firstEdge],
                                     first.edges[secondEdge]);
            }
        } else {
            const RegisteredContactSurface& surface =
                contactSurfaces_[pair.first];
            const RegisteredContactLine& line = contactLines_[pair.second];
            const std::size_t triangleLast =
                surface.firstTriangle + surface.triangleCount;
            for (std::size_t triangleIndex = surface.firstTriangle;
                 triangleIndex < triangleLast; ++triangleIndex) {
                ++possibleCount;
                if (triangleContains(triangleIndex, line.a) ||
                    triangleContains(triangleIndex, line.b)) {
                    ++excludedCount;
                    continue;
                }
                emitEligible({pairIndex,
                              ContactFeatureKind::SegmentTriangle,
                              {line.a, line.b, pair.second},
                              {triangleIndex, 0, 0}});
            }
        }
        diagnostics.possibleCount += possibleCount;
        diagnostics.excludedCount += excludedCount;
        pairDiagnostics[pairIndex].possibleCount = possibleCount;
        pairDiagnostics[pairIndex].excludedCount = excludedCount;
    }
    std::sort(candidates.begin(),
              candidates.end(),
              [](const SolverContactCandidate& first,
                 const SolverContactCandidate& second) {
                  return first.key < second.key;
              });
    BroadphaseResult broadphase =
        sweepAndPruneContactPairs(broadphaseProxies, broadphasePairs);
    diagnostics.candidateCount = broadphase.candidateKeys.size();
    contactAudit_.iterationCandidateKeys =
        std::move(broadphase.candidateKeys);
    contactAudit_.iterationQueryKeys.clear();
    for (const ContactFeatureKey& key :
         contactAudit_.iterationCandidateKeys) {
        ++pairDiagnostics[key.pair].candidateCount;
    }

    std::vector<ContactRecord> records;
    for (const ContactFeatureKey& candidateKey :
         contactAudit_.iterationCandidateKeys) {
        const auto candidateIterator = std::lower_bound(
            candidates.begin(),
            candidates.end(),
            candidateKey,
            [](const SolverContactCandidate& candidate,
               const ContactFeatureKey& key) { return candidate.key < key; });
        if (candidateIterator == candidates.end() ||
            candidateIterator->key != candidateKey) {
            throw ContactStepError(candidateKey,
                                   "Broadphase emitted an unknown feature");
        }
        const SolverContactCandidate& candidate = *candidateIterator;
        ++diagnostics.queryCount;
        contactAudit_.iterationQueryKeys.push_back(candidate.key);
        ++pairDiagnostics[candidate.key.pair].queryCount;
        const CcdResult ccd = sweptContact(candidate.kind,
                                           candidate.firstPrimitive,
                                           candidate.secondPrimitive,
                                           candidate.separation,
                                           stepSettings.contactCcd);
        diagnostics.maximumCcdIterations = std::max(
            diagnostics.maximumCcdIterations, ccd.conservativeIterations);
        diagnostics.maximumIntervalSubdivisions =
            std::max(diagnostics.maximumIntervalSubdivisions,
                     ccd.intervalSubdivisions);
        ContactDiagnostics& pairDiagnostic =
            pairDiagnostics[candidate.key.pair];
        pairDiagnostic.maximumCcdIterations = std::max(
            pairDiagnostic.maximumCcdIterations,
            ccd.conservativeIterations);
        pairDiagnostic.maximumIntervalSubdivisions = std::max(
            pairDiagnostic.maximumIntervalSubdivisions,
            ccd.intervalSubdivisions);
        if (ccd.state == CcdState::Indeterminate) {
            diagnostics.indeterminateCount += 1;
            pairDiagnostic.indeterminateCount += 1;
            throw ContactStepError(
                candidate.key,
                "Contact CCD could not certify a supported feature");
        }
        if (ccd.state == CcdState::NoImpact) {
            continue;
        }

        const auto multiplierIterator = std::lower_bound(
            contactMultipliers_.begin(),
            contactMultipliers_.end(),
            candidate.key,
            [](const auto& entry, const ContactFeatureKey& key) {
                return entry.first < key;
            });
        const std::size_t multiplierIndex = static_cast<std::size_t>(
            multiplierIterator - contactMultipliers_.begin());
        if (multiplierIterator == contactMultipliers_.end() ||
            multiplierIterator->first != candidate.key) {
            contactMultipliers_.insert(multiplierIterator,
                                       {candidate.key, 0.0});
        }
        double& multiplier = contactMultipliers_[multiplierIndex].second;

        const ClosestFeatureResult impactGeometry = ccd.geometry;
        const ClosestFeatureResult currentGeometry =
            evaluateCurrent(candidate);
        if (!currentGeometry.certified()) {
            throw ContactStepError(
                candidate.key,
                "Contact endpoint geometry is not certifiable");
        }
        // Corrections use current closest-point weights. The historical CCD
        // normal only supplies orientation, which preserves signed crossing
        // information without applying endpoint impulses at obsolete TOI
        // weights.
        ClosestFeatureResult geometry = currentGeometry;
        if (dot(geometry.normal, impactGeometry.normal) < 0.0) {
            geometry.normal = -geometry.normal;
        }
        const Vec3 normal = geometry.normal;
        if (!finite(normal) ||
            std::abs(lengthSquared(normal) - 1.0) > 1.0e-8) {
            throw ContactStepError(candidate.key,
                                   "Contact normal is not finite/unit");
        }
        Vec3 firstPoint = weightedPoint(nodes_,
                                        candidate.firstNodes,
                                        candidate.firstNodeCount,
                                        geometry.firstWeights);
        Vec3 secondPoint = weightedPoint(nodes_,
                                         candidate.secondNodes,
                                         candidate.secondNodeCount,
                                         geometry.secondWeights);
        const double gap =
            dot(firstPoint - secondPoint, normal) - candidate.separation;
        const double persistenceMargin = std::max(
            0.1 * candidate.separation,
            8.0 * stepSettings.contactCcd.distanceTolerance);
        if (gap > persistenceMargin && multiplier == 0.0) {
            continue;
        }
        double effectiveInverseMass = 0.0;
        for (std::size_t index = 0; index < candidate.firstNodeCount;
             ++index) {
            effectiveInverseMass +=
                nodes_[candidate.firstNodes[index]].inverseMass *
                geometry.firstWeights[index] * geometry.firstWeights[index];
        }
        for (std::size_t index = 0; index < candidate.secondNodeCount;
             ++index) {
            effectiveInverseMass +=
                nodes_[candidate.secondNodes[index]].inverseMass *
                geometry.secondWeights[index] * geometry.secondWeights[index];
        }
        const double alphaTilde =
            candidate.settings.normalCompliance / (dt * dt);
        if (!(effectiveInverseMass + alphaTilde > 0.0)) {
            if (gap < 0.0) {
                throw ContactStepError(
                    candidate.key,
                    "Penetrating contact has no movable/compliant response");
            }
            continue;
        }
        const double delta =
            (-gap - alphaTilde * multiplier) /
            (effectiveInverseMass + alphaTilde);
        const double newMultiplier = std::max(0.0, multiplier + delta);
        const double appliedDelta = newMultiplier - multiplier;
        if (!std::isfinite(newMultiplier) ||
            !std::isfinite(appliedDelta)) {
            throw ContactStepError(candidate.key,
                                   "Non-finite contact projection");
        }
        multiplier = newMultiplier;
        ContactRecord record;
        const auto priorRecord = std::lower_bound(
            contactRecords_.begin(),
            contactRecords_.end(),
            candidate.key,
            [](const ContactRecord& existing, const ContactFeatureKey& key) {
                return existing.key < key;
            });
        if (priorRecord != contactRecords_.end() &&
            priorRecord->key == candidate.key) {
            record = *priorRecord;
        }
        for (std::size_t index = 0; index < candidate.firstNodeCount;
             ++index) {
            Node& node = nodes_[candidate.firstNodes[index]];
            const Vec3 impulse = geometry.firstWeights[index] *
                                 (appliedDelta / dt) * normal;
            node.position += node.inverseMass * dt * impulse;
            record.firstNodeImpulses[index] += impulse;
        }
        for (std::size_t index = 0; index < candidate.secondNodeCount;
             ++index) {
            Node& node = nodes_[candidate.secondNodes[index]];
            const Vec3 impulse = -geometry.secondWeights[index] *
                                 (appliedDelta / dt) * normal;
            node.position += node.inverseMass * dt * impulse;
            record.secondNodeImpulses[index] += impulse;
        }
        firstPoint = weightedPoint(nodes_,
                                   candidate.firstNodes,
                                   candidate.firstNodeCount,
                                   geometry.firstWeights);
        secondPoint = weightedPoint(nodes_,
                                    candidate.secondNodes,
                                    candidate.secondNodeCount,
                                    geometry.secondWeights);
        const double solvedGap =
            dot(firstPoint - secondPoint, normal) - candidate.separation;
        if (!finite(firstPoint) || !finite(secondPoint) ||
            !std::isfinite(solvedGap)) {
            throw ContactStepError(candidate.key,
                                   "Non-finite solved contact geometry");
        }

        const int priorVisits = record.solverVisits;
        record.key = candidate.key;
        record.kind = candidate.kind;
        record.ccdState = ccd.state;
        record.timeOfImpact = ccd.timeOfImpact;
        record.bracketLower = ccd.bracketLower;
        record.bracketUpper = ccd.bracketUpper;
        record.ccdIterations = ccd.conservativeIterations;
        record.intervalSubdivisions = ccd.intervalSubdivisions;
        record.usedIntervalFallback = ccd.usedIntervalFallback;
        record.firstNodes = candidate.firstNodes;
        record.secondNodes = candidate.secondNodes;
        record.firstNodeCount = candidate.firstNodeCount;
        record.secondNodeCount = candidate.secondNodeCount;
        record.firstWeights = geometry.firstWeights;
        record.secondWeights = geometry.secondWeights;
        record.firstPoint = firstPoint;
        record.secondPoint = secondPoint;
        record.normal = normal;
        record.pairSeparation = candidate.separation;
        record.gap = solvedGap;
        record.penetration = std::max(0.0, -solvedGap);
        record.normalMultiplier = multiplier;
        record.normalForceEstimate = multiplier / (dt * dt);
        record.normalImpulseMagnitude = multiplier / dt;
        record.normalResidual = std::max(0.0, -solvedGap);
        record.solverVisits = priorVisits + 1;
        updateRecordImpulseDiagnostics(record, nodes_);
        records.push_back(record);
    }
    // Keep any earlier per-visit impulse ledger even when the feature's
    // start-to-current sweep becomes NoImpact in a later iteration. Final
    // certification will refresh its geometry and decide final activity;
    // dropping it here would make actual nodal momentum changes untraceable.
    for (const ContactRecord& priorRecord : contactRecords_) {
        const bool hasAppliedImpulse =
            std::any_of(priorRecord.firstNodeImpulses.begin(),
                        priorRecord.firstNodeImpulses.end(),
                        [](const Vec3& impulse) {
                            return lengthSquared(impulse) > 0.0;
                        }) ||
            std::any_of(priorRecord.secondNodeImpulses.begin(),
                        priorRecord.secondNodeImpulses.end(),
                        [](const Vec3& impulse) {
                            return lengthSquared(impulse) > 0.0;
                        });
        if (!hasAppliedImpulse) {
            continue;
        }
        const auto current = std::find_if(
            records.begin(),
            records.end(),
            [&priorRecord](const ContactRecord& record) {
                return record.key == priorRecord.key;
            });
        if (current == records.end()) {
            ContactRecord inactive = priorRecord;
            inactive.ccdState = CcdState::NoImpact;
            inactive.tangentialImpulse = {};
            inactive.frictionState = ContactFrictionState::Inactive;
            inactive.frictionConeRatio = 0.0;
            inactive.frictionResidual = 0.0;
            inactive.frictionWork = 0.0;
            inactive.tangentSpeedBefore = 0.0;
            inactive.tangentSpeedAfter = 0.0;
            records.push_back(inactive);
        }
    }
    std::sort(records.begin(),
              records.end(),
              [](const ContactRecord& first, const ContactRecord& second) {
                  return first.key < second.key;
              });
    contactRecords_ = std::move(records);
    for (const ContactRecord& record : contactRecords_) {
        ContactDiagnostics& pairDiagnostic = pairDiagnostics[record.key.pair];
        if (record.normalMultiplier > 0.0 &&
            record.ccdState != CcdState::NoImpact) {
            ++diagnostics.activeCount;
            ++pairDiagnostic.activeCount;
        }
        diagnostics.minimumGap =
            std::min(diagnostics.minimumGap, record.gap);
        diagnostics.maximumPenetration =
            std::max(diagnostics.maximumPenetration, record.penetration);
        diagnostics.maximumNormalResidual =
            std::max(diagnostics.maximumNormalResidual,
                     record.normalResidual);
        diagnostics.firstImpulse += record.firstImpulse;
        diagnostics.secondImpulse += record.secondImpulse;
        diagnostics.firstMoment += record.firstMoment;
        diagnostics.secondMoment += record.secondMoment;
        pairDiagnostic.minimumGap =
            std::min(pairDiagnostic.minimumGap, record.gap);
        pairDiagnostic.maximumPenetration =
            std::max(pairDiagnostic.maximumPenetration, record.penetration);
        pairDiagnostic.maximumNormalResidual =
            std::max(pairDiagnostic.maximumNormalResidual,
                     record.normalResidual);
        pairDiagnostic.maximumFrictionResidual =
            std::max(pairDiagnostic.maximumFrictionResidual,
                     record.frictionResidual);
        pairDiagnostic.firstImpulse += record.firstImpulse;
        pairDiagnostic.secondImpulse += record.secondImpulse;
        pairDiagnostic.firstMoment += record.firstMoment;
        pairDiagnostic.secondMoment += record.secondMoment;
        pairDiagnostic.frictionWork += record.frictionWork;
    }
    diagnostics.netInternalImpulse =
        diagnostics.firstImpulse + diagnostics.secondImpulse;
    diagnostics.netInternalMoment =
        diagnostics.firstMoment + diagnostics.secondMoment;
    contactDiagnostics_ = diagnostics;
    for (ContactDiagnostics& pairDiagnostic : pairDiagnostics) {
        pairDiagnostic.netInternalImpulse =
            pairDiagnostic.firstImpulse + pairDiagnostic.secondImpulse;
        pairDiagnostic.netInternalMoment =
            pairDiagnostic.firstMoment + pairDiagnostic.secondMoment;
    }
    contactPairDiagnostics_ = std::move(pairDiagnostics);
}

void SoftBody::certifyContactState(double dt,
                                   const StepSettings& stepSettings) {
    // Rebuild the complete supported candidate set from the accepted trial
    // geometry. This pass is deliberately read-only: it certifies the final
    // Gauss-Seidel state rather than trusting each record's last solver visit.
    std::vector<SolverContactCandidate> candidates;
    std::vector<BroadphaseProxy> broadphaseProxies;
    std::vector<BroadphaseFeaturePair> broadphasePairs;
    std::map<SolverProxyKey, std::size_t> proxyIds;
    ContactDiagnostics diagnostics;
    diagnostics.registered = true;
    diagnostics.solveSucceeded = true;
    std::vector<ContactDiagnostics> pairDiagnostics(contactPairs_.size());
    for (ContactDiagnostics& pairDiagnostic : pairDiagnostics) {
        pairDiagnostic.registered = true;
        pairDiagnostic.solveSucceeded = true;
    }

    for (std::size_t pairIndex = 0; pairIndex < contactPairs_.size();
         ++pairIndex) {
        const RegisteredContactPair& pair = contactPairs_[pairIndex];
        std::size_t possibleCount = 0;
        std::size_t excludedCount = 0;
        const double separation =
            pair.kind == ContactPairKind::SurfaceSurface
                ? contactSurfaces_[pair.first].halfThickness +
                      contactSurfaces_[pair.second].halfThickness
                : contactSurfaces_[pair.first].halfThickness +
                      contactLines_[pair.second].radius;
        const auto proxyFor = [&](const std::array<std::size_t, 3>& nodeIds,
                                  std::size_t nodeCount,
                                  const LinearContactPrimitive& primitive) {
            const SolverProxyKey proxyKey{pairIndex, nodeIds, nodeCount};
            const auto existing = proxyIds.find(proxyKey);
            if (existing != proxyIds.end()) {
                return existing->second;
            }
            const std::size_t proxyId = broadphaseProxies.size();
            const SweptAabb bounds = sweptExpandedAabb(
                std::span<const Vec3>{primitive.previous.data(), nodeCount},
                std::span<const Vec3>{primitive.current.data(), nodeCount},
                separation);
            proxyIds.emplace(proxyKey, proxyId);
            broadphaseProxies.push_back({proxyId, bounds});
            return proxyId;
        };
        const auto emitEligible = [&](const ContactFeatureKey& key) {
            SolverContactCandidate candidate;
            candidate.key = key;
            candidate.kind = key.kind;
            candidate.separation = separation;
            candidate.settings = pair.settings;
            switch (key.kind) {
            case ContactFeatureKind::VertexTriangle: {
                candidate.firstNodes = {key.firstPrimitive[0], 0, 0};
                candidate.firstNodeCount = 1;
                const Triangle& triangle =
                    triangles_[key.secondPrimitive[0]];
                candidate.secondNodes =
                    {triangle.a, triangle.b, triangle.c};
                candidate.secondNodeCount = 3;
                break;
            }
            case ContactFeatureKind::EdgeEdge:
                candidate.firstNodes =
                    {key.firstPrimitive[0], key.firstPrimitive[1], 0};
                candidate.secondNodes =
                    {key.secondPrimitive[0], key.secondPrimitive[1], 0};
                candidate.firstNodeCount = 2;
                candidate.secondNodeCount = 2;
                break;
            case ContactFeatureKind::SegmentTriangle: {
                candidate.firstNodes =
                    {key.firstPrimitive[0], key.firstPrimitive[1], 0};
                candidate.firstNodeCount = 2;
                const Triangle& triangle =
                    triangles_[key.secondPrimitive[0]];
                candidate.secondNodes =
                    {triangle.a, triangle.b, triangle.c};
                candidate.secondNodeCount = 3;
                break;
            }
            }
            try {
                candidate.firstPrimitive = makeLinearPrimitive(
                    nodes_, candidate.firstNodes, candidate.firstNodeCount);
                candidate.secondPrimitive = makeLinearPrimitive(
                    nodes_, candidate.secondNodes, candidate.secondNodeCount);
                candidate.firstProxy = proxyFor(candidate.firstNodes,
                                                candidate.firstNodeCount,
                                                candidate.firstPrimitive);
                candidate.secondProxy = proxyFor(candidate.secondNodes,
                                                 candidate.secondNodeCount,
                                                 candidate.secondPrimitive);
            } catch (const std::exception& error) {
                throw ContactStepError(
                    candidate.key,
                    std::string("Final broadphase input failed: ") +
                        error.what());
            }
            const SweptAabb& firstBounds =
                broadphaseProxies[candidate.firstProxy].bounds;
            const SweptAabb& secondBounds =
                broadphaseProxies[candidate.secondProxy].bounds;
            const auto overlaps = [](double firstMinimum,
                                     double firstMaximum,
                                     double secondMinimum,
                                     double secondMaximum) {
                return firstMinimum <= secondMaximum &&
                       secondMinimum <= firstMaximum;
            };
            if (!overlaps(firstBounds.minimum.x, firstBounds.maximum.x,
                          secondBounds.minimum.x, secondBounds.maximum.x) ||
                !overlaps(firstBounds.minimum.y, firstBounds.maximum.y,
                          secondBounds.minimum.y, secondBounds.maximum.y) ||
                !overlaps(firstBounds.minimum.z, firstBounds.maximum.z,
                          secondBounds.minimum.z, secondBounds.maximum.z))
                return;
            broadphasePairs.push_back(
                {key, candidate.firstProxy, candidate.secondProxy});
            candidates.push_back(std::move(candidate));
        };
        const auto triangleContains = [this](std::size_t triangleIndex,
                                              std::size_t nodeIndex) {
            const Triangle& triangle = triangles_[triangleIndex];
            return triangle.a == nodeIndex || triangle.b == nodeIndex ||
                   triangle.c == nodeIndex;
        };
        const auto emitVertexTriangles = [&] (
            const RegisteredContactSurface& vertices,
            const RegisteredContactSurface& triangles) {
            const std::size_t triangleLast =
                triangles.firstTriangle + triangles.triangleCount;
            for (const std::size_t vertex : vertices.vertices) {
                for (std::size_t triangleIndex = triangles.firstTriangle;
                     triangleIndex < triangleLast; ++triangleIndex) {
                    ++possibleCount;
                    if (triangleContains(triangleIndex, vertex)) {
                        ++excludedCount;
                        continue;
                    }
                    emitEligible({pairIndex,
                                  ContactFeatureKind::VertexTriangle,
                                  {vertex, 0, 0},
                                  {triangleIndex, 0, 0}});
                }
            }
        };
        const auto emitEdgePair = [&](const ContactEdge& first,
                                      const ContactEdge& second) {
            ++possibleCount;
            if (first.a == second.a || first.a == second.b ||
                first.b == second.a || first.b == second.b) {
                ++excludedCount;
                return;
            }
            ContactEdge canonicalFirst = first;
            ContactEdge canonicalSecond = second;
            if (canonicalSecond < canonicalFirst)
                std::swap(canonicalFirst, canonicalSecond);
            emitEligible({pairIndex,
                          ContactFeatureKind::EdgeEdge,
                          {canonicalFirst.a, canonicalFirst.b, 0},
                          {canonicalSecond.a, canonicalSecond.b, 0}});
        };
        if (pair.kind == ContactPairKind::SurfaceSurface) {
            const RegisteredContactSurface& first =
                contactSurfaces_[pair.first];
            const RegisteredContactSurface& second =
                contactSurfaces_[pair.second];
            emitVertexTriangles(first, second);
            if (pair.first != pair.second) {
                emitVertexTriangles(second, first);
                for (const ContactEdge& firstEdge : first.edges)
                    for (const ContactEdge& secondEdge : second.edges)
                        emitEdgePair(firstEdge, secondEdge);
            } else {
                for (std::size_t firstEdge = 0;
                     firstEdge < first.edges.size(); ++firstEdge)
                    for (std::size_t secondEdge = firstEdge + 1;
                         secondEdge < first.edges.size(); ++secondEdge)
                        emitEdgePair(first.edges[firstEdge],
                                     first.edges[secondEdge]);
            }
        } else {
            const RegisteredContactSurface& surface =
                contactSurfaces_[pair.first];
            const RegisteredContactLine& line = contactLines_[pair.second];
            const std::size_t triangleLast =
                surface.firstTriangle + surface.triangleCount;
            for (std::size_t triangleIndex = surface.firstTriangle;
                 triangleIndex < triangleLast; ++triangleIndex) {
                ++possibleCount;
                if (triangleContains(triangleIndex, line.a) ||
                    triangleContains(triangleIndex, line.b)) {
                    ++excludedCount;
                    continue;
                }
                emitEligible({pairIndex,
                              ContactFeatureKind::SegmentTriangle,
                              {line.a, line.b, pair.second},
                              {triangleIndex, 0, 0}});
            }
        }
        diagnostics.possibleCount += possibleCount;
        diagnostics.excludedCount += excludedCount;
        pairDiagnostics[pairIndex].possibleCount = possibleCount;
        pairDiagnostics[pairIndex].excludedCount = excludedCount;
    }
    std::sort(candidates.begin(),
              candidates.end(),
              [](const SolverContactCandidate& first,
                 const SolverContactCandidate& second) {
                  return first.key < second.key;
              });
    BroadphaseResult broadphase = sweepAndPruneContactPairs(
        broadphaseProxies, broadphasePairs);
    diagnostics.candidateCount = broadphase.candidateKeys.size();
    contactAudit_.certificationCandidateKeys =
        std::move(broadphase.candidateKeys);
    contactAudit_.certificationQueryKeys.clear();
    for (const ContactFeatureKey& key :
         contactAudit_.certificationCandidateKeys) {
        ++pairDiagnostics[key.pair].candidateCount;
    }

    const std::vector<ContactRecord> solverRecords = contactRecords_;
    std::vector<ContactRecord> certifiedRecords;
    for (const ContactFeatureKey& candidateKey :
         contactAudit_.certificationCandidateKeys) {
        const auto candidateIterator = std::lower_bound(
            candidates.begin(),
            candidates.end(),
            candidateKey,
            [](const SolverContactCandidate& candidate,
               const ContactFeatureKey& key) { return candidate.key < key; });
        if (candidateIterator == candidates.end() ||
            candidateIterator->key != candidateKey) {
            throw ContactStepError(candidateKey,
                                   "Final broadphase emitted an unknown feature");
        }
        const SolverContactCandidate& candidate = *candidateIterator;
        ++diagnostics.queryCount;
        contactAudit_.certificationQueryKeys.push_back(candidate.key);
        ContactDiagnostics& pairDiagnostic =
            pairDiagnostics[candidate.key.pair];
        ++pairDiagnostic.queryCount;
        const CcdResult ccd = sweptContact(candidate.kind,
                                           candidate.firstPrimitive,
                                           candidate.secondPrimitive,
                                           candidate.separation,
                                           stepSettings.contactCcd);
        diagnostics.maximumCcdIterations = std::max(
            diagnostics.maximumCcdIterations, ccd.conservativeIterations);
        diagnostics.maximumIntervalSubdivisions = std::max(
            diagnostics.maximumIntervalSubdivisions,
            ccd.intervalSubdivisions);
        pairDiagnostic.maximumCcdIterations = std::max(
            pairDiagnostic.maximumCcdIterations, ccd.conservativeIterations);
        pairDiagnostic.maximumIntervalSubdivisions = std::max(
            pairDiagnostic.maximumIntervalSubdivisions,
            ccd.intervalSubdivisions);
        if (ccd.state == CcdState::Indeterminate) {
            ++diagnostics.indeterminateCount;
            ++pairDiagnostic.indeterminateCount;
            throw ContactStepError(
                candidate.key,
                "Final contact geometry could not be certified");
        }
        if (ccd.state == CcdState::NoImpact) {
            continue;
        }

        const ClosestFeatureResult currentGeometry =
            evaluateCurrent(candidate);
        if (!currentGeometry.certified()) {
            throw ContactStepError(
                candidate.key,
                "Final contact closest geometry is not certifiable");
        }
        ClosestFeatureResult geometry = currentGeometry;
        if (dot(geometry.normal, ccd.geometry.normal) < 0.0) {
            geometry.normal = -geometry.normal;
        }
        const Vec3 firstPoint = weightedPoint(nodes_,
                                              candidate.firstNodes,
                                              candidate.firstNodeCount,
                                              geometry.firstWeights);
        const Vec3 secondPoint = weightedPoint(nodes_,
                                               candidate.secondNodes,
                                               candidate.secondNodeCount,
                                               geometry.secondWeights);
        const double finalGap =
            dot(firstPoint - secondPoint, geometry.normal) -
            candidate.separation;
        if (!finite(firstPoint) || !finite(secondPoint) ||
            !finite(geometry.normal) || !std::isfinite(finalGap)) {
            throw ContactStepError(candidate.key,
                                   "Final contact geometry is non-finite");
        }

        ContactRecord record;
        const auto prior = std::lower_bound(
            solverRecords.begin(),
            solverRecords.end(),
            candidate.key,
            [](const ContactRecord& existing, const ContactFeatureKey& key) {
                return existing.key < key;
            });
        if (prior != solverRecords.end() && prior->key == candidate.key) {
            record = *prior;
        }
        const auto multiplier = std::lower_bound(
            contactMultipliers_.begin(),
            contactMultipliers_.end(),
            candidate.key,
            [](const auto& entry, const ContactFeatureKey& key) {
                return entry.first < key;
            });
        const double lambda =
            multiplier != contactMultipliers_.end() &&
                    multiplier->first == candidate.key
                ? multiplier->second
                : 0.0;
        record.key = candidate.key;
        record.kind = candidate.kind;
        record.ccdState = ccd.state;
        record.timeOfImpact = ccd.timeOfImpact;
        record.bracketLower = ccd.bracketLower;
        record.bracketUpper = ccd.bracketUpper;
        record.ccdIterations = ccd.conservativeIterations;
        record.intervalSubdivisions = ccd.intervalSubdivisions;
        record.usedIntervalFallback = ccd.usedIntervalFallback;
        record.firstNodes = candidate.firstNodes;
        record.secondNodes = candidate.secondNodes;
        record.firstNodeCount = candidate.firstNodeCount;
        record.secondNodeCount = candidate.secondNodeCount;
        record.firstWeights = geometry.firstWeights;
        record.secondWeights = geometry.secondWeights;
        record.firstPoint = firstPoint;
        record.secondPoint = secondPoint;
        record.normal = geometry.normal;
        record.pairSeparation = candidate.separation;
        record.gap = finalGap;
        record.penetration = std::max(0.0, -finalGap);
        record.normalMultiplier = lambda;
        record.normalForceEstimate = lambda / (dt * dt);
        record.normalImpulseMagnitude = lambda / dt;
        record.normalResidual = record.penetration;
        record.tangentialImpulse = {};
        record.frictionState = ContactFrictionState::Inactive;
        record.frictionConeRatio = 0.0;
        record.frictionResidual = 0.0;
        record.frictionWork = 0.0;
        record.tangentSpeedBefore = 0.0;
        record.tangentSpeedAfter = 0.0;
        updateRecordImpulseDiagnostics(record, nodes_);
        const double penetrationLimit =
            std::max(0.02 * record.pairSeparation, 1.0e-8);
        if (record.penetration > penetrationLimit) {
            throw ContactStepError(
                record.key,
                std::string(
                    "Final contact penetration exceeds the certified gate: ") +
                    std::to_string(record.penetration));
        }
        certifiedRecords.push_back(record);
    }
    // A Gauss-Seidel contact may have applied an impulse and then been moved
    // fully clear by a later constraint. The straight start-to-final sweep no
    // longer contains that intermediate visit, but its actual nodal impulse
    // must remain inspectable. Retain such records as final NoImpact records,
    // refresh their endpoint geometry, and keep friction inactive.
    for (const ContactRecord& solverRecord : solverRecords) {
        const auto alreadyCertified = std::lower_bound(
            certifiedRecords.begin(),
            certifiedRecords.end(),
            solverRecord.key,
            [](const ContactRecord& existing, const ContactFeatureKey& key) {
                return existing.key < key;
            });
        if (alreadyCertified != certifiedRecords.end() &&
            alreadyCertified->key == solverRecord.key) {
            continue;
        }
        const auto candidate = std::lower_bound(
            candidates.begin(),
            candidates.end(),
            solverRecord.key,
            [](const SolverContactCandidate& existing,
               const ContactFeatureKey& key) { return existing.key < key; });
        if (candidate == candidates.end() ||
            candidate->key != solverRecord.key) {
            throw ContactStepError(
                solverRecord.key,
                "Applied contact impulse lost its supported feature");
        }
        ClosestFeatureResult geometry = evaluateCurrent(*candidate);
        if (!geometry.certified()) {
            throw ContactStepError(
                solverRecord.key,
                "Cleared contact final geometry is not certifiable");
        }
        if (dot(geometry.normal, solverRecord.normal) < 0.0) {
            geometry.normal = -geometry.normal;
        }
        ContactRecord record = solverRecord;
        record.ccdState = CcdState::NoImpact;
        record.firstWeights = geometry.firstWeights;
        record.secondWeights = geometry.secondWeights;
        record.firstPoint = weightedPoint(nodes_,
                                          candidate->firstNodes,
                                          candidate->firstNodeCount,
                                          geometry.firstWeights);
        record.secondPoint = weightedPoint(nodes_,
                                           candidate->secondNodes,
                                           candidate->secondNodeCount,
                                           geometry.secondWeights);
        record.normal = geometry.normal;
        record.gap = dot(record.firstPoint - record.secondPoint,
                         record.normal) -
                     candidate->separation;
        record.penetration = std::max(0.0, -record.gap);
        record.normalResidual = record.penetration;
        record.tangentialImpulse = {};
        record.frictionState = ContactFrictionState::Inactive;
        record.frictionConeRatio = 0.0;
        record.frictionResidual = 0.0;
        record.frictionWork = 0.0;
        record.tangentSpeedBefore = 0.0;
        record.tangentSpeedAfter = 0.0;
        updateRecordImpulseDiagnostics(record, nodes_);
        const double penetrationLimit =
            std::max(0.02 * record.pairSeparation, 1.0e-8);
        if (!std::isfinite(record.gap) ||
            record.penetration > penetrationLimit) {
            throw ContactStepError(
                record.key,
                "Cleared contact failed final geometry certification");
        }
        certifiedRecords.push_back(record);
        std::sort(certifiedRecords.begin(),
                  certifiedRecords.end(),
                  [](const ContactRecord& first,
                     const ContactRecord& second) {
                      return first.key < second.key;
                  });
    }
    std::sort(certifiedRecords.begin(),
              certifiedRecords.end(),
              [](const ContactRecord& first, const ContactRecord& second) {
                  return first.key < second.key;
              });
    contactRecords_ = std::move(certifiedRecords);

    for (const ContactRecord& record : contactRecords_) {
        ContactDiagnostics& pairDiagnostic =
            pairDiagnostics[record.key.pair];
        if (record.normalMultiplier > 0.0 &&
            record.ccdState != CcdState::NoImpact) {
            ++diagnostics.activeCount;
            ++pairDiagnostic.activeCount;
        }
        diagnostics.minimumGap =
            std::min(diagnostics.minimumGap, record.gap);
        diagnostics.maximumPenetration =
            std::max(diagnostics.maximumPenetration, record.penetration);
        diagnostics.maximumNormalResidual = std::max(
            diagnostics.maximumNormalResidual, record.normalResidual);
        diagnostics.firstImpulse += record.firstImpulse;
        diagnostics.secondImpulse += record.secondImpulse;
        diagnostics.firstMoment += record.firstMoment;
        diagnostics.secondMoment += record.secondMoment;
        pairDiagnostic.minimumGap =
            std::min(pairDiagnostic.minimumGap, record.gap);
        pairDiagnostic.maximumPenetration = std::max(
            pairDiagnostic.maximumPenetration, record.penetration);
        pairDiagnostic.maximumNormalResidual = std::max(
            pairDiagnostic.maximumNormalResidual, record.normalResidual);
        pairDiagnostic.firstImpulse += record.firstImpulse;
        pairDiagnostic.secondImpulse += record.secondImpulse;
        pairDiagnostic.firstMoment += record.firstMoment;
        pairDiagnostic.secondMoment += record.secondMoment;
    }
    diagnostics.netInternalImpulse =
        diagnostics.firstImpulse + diagnostics.secondImpulse;
    diagnostics.netInternalMoment =
        diagnostics.firstMoment + diagnostics.secondMoment;
    for (ContactDiagnostics& pairDiagnostic : pairDiagnostics) {
        pairDiagnostic.netInternalImpulse =
            pairDiagnostic.firstImpulse + pairDiagnostic.secondImpulse;
        pairDiagnostic.netInternalMoment =
            pairDiagnostic.firstMoment + pairDiagnostic.secondMoment;
    }
    contactDiagnostics_ = diagnostics;
    contactPairDiagnostics_ = std::move(pairDiagnostics);
}

void SoftBody::applyContactFriction() {
    for (ContactRecord& record : contactRecords_) {
        Vec3 firstVelocity;
        Vec3 secondVelocity;
        double effectiveInverseMass = 0.0;
        for (std::size_t index = 0; index < record.firstNodeCount; ++index) {
            const Node& node = nodes_[record.firstNodes[index]];
            firstVelocity += record.firstWeights[index] * node.velocity;
            effectiveInverseMass +=
                node.inverseMass * record.firstWeights[index] *
                record.firstWeights[index];
        }
        for (std::size_t index = 0; index < record.secondNodeCount; ++index) {
            const Node& node = nodes_[record.secondNodes[index]];
            secondVelocity += record.secondWeights[index] * node.velocity;
            effectiveInverseMass +=
                node.inverseMass * record.secondWeights[index] *
                record.secondWeights[index];
        }
        const RegisteredContactPair& pair = contactPairs_[record.key.pair];
        const CoulombFrictionResult friction = solveCoulombFriction(
            firstVelocity - secondVelocity,
            record.normal,
            effectiveInverseMass,
            record.normalImpulseMagnitude,
            pair.settings.staticFriction,
            pair.settings.dynamicFriction,
            record.normalMultiplier > 0.0 &&
                record.ccdState != CcdState::NoImpact);
        record.tangentialImpulse = friction.tangentialImpulse;
        record.frictionState = friction.state;
        record.frictionConeRatio = friction.coneRatio;
        record.frictionWork = friction.work;
        record.tangentSpeedBefore =
            length(friction.initialTangentialVelocity);
        record.tangentSpeedAfter =
            length(friction.finalTangentialVelocity);
        const double frictionLimit =
            (friction.state == ContactFrictionState::Sliding
                 ? pair.settings.dynamicFriction
                 : pair.settings.staticFriction) *
            record.normalImpulseMagnitude;
        record.frictionResidual = std::max(
            0.0, length(record.tangentialImpulse) - frictionLimit);
        if (!finite(record.tangentialImpulse) ||
            !std::isfinite(record.frictionWork) ||
            record.frictionWork >
                1.0e-12 *
                    std::max(1.0,
                             lengthSquared(
                                 friction.initialTangentialVelocity))) {
            throw ContactStepError(record.key,
                                   "Non-finite/positive contact friction work");
        }
        for (std::size_t index = 0; index < record.firstNodeCount; ++index) {
            Node& node = nodes_[record.firstNodes[index]];
            const Vec3 nodalImpulse =
                record.firstWeights[index] * record.tangentialImpulse;
            node.velocity += node.inverseMass * nodalImpulse;
            record.firstNodeImpulses[index] += nodalImpulse;
        }
        for (std::size_t index = 0; index < record.secondNodeCount; ++index) {
            Node& node = nodes_[record.secondNodes[index]];
            const Vec3 nodalImpulse =
                -record.secondWeights[index] * record.tangentialImpulse;
            node.velocity += node.inverseMass * nodalImpulse;
            record.secondNodeImpulses[index] += nodalImpulse;
        }
        updateRecordImpulseDiagnostics(record, nodes_);
    }

    contactDiagnostics_.activeCount = 0;
    contactDiagnostics_.firstImpulse = {};
    contactDiagnostics_.secondImpulse = {};
    contactDiagnostics_.netInternalImpulse = {};
    contactDiagnostics_.firstMoment = {};
    contactDiagnostics_.secondMoment = {};
    contactDiagnostics_.netInternalMoment = {};
    contactDiagnostics_.frictionWork = 0.0;
    contactDiagnostics_.maximumFrictionResidual = 0.0;
    for (ContactDiagnostics& pairDiagnostic : contactPairDiagnostics_) {
        pairDiagnostic.activeCount = 0;
        pairDiagnostic.firstImpulse = {};
        pairDiagnostic.secondImpulse = {};
        pairDiagnostic.netInternalImpulse = {};
        pairDiagnostic.firstMoment = {};
        pairDiagnostic.secondMoment = {};
        pairDiagnostic.netInternalMoment = {};
        pairDiagnostic.frictionWork = 0.0;
        pairDiagnostic.maximumFrictionResidual = 0.0;
    }
    for (const ContactRecord& record : contactRecords_) {
        ContactDiagnostics& pairDiagnostic =
            contactPairDiagnostics_[record.key.pair];
        if (record.normalMultiplier > 0.0 &&
            record.ccdState != CcdState::NoImpact) {
            ++contactDiagnostics_.activeCount;
            ++pairDiagnostic.activeCount;
        }
        contactDiagnostics_.firstImpulse += record.firstImpulse;
        contactDiagnostics_.secondImpulse += record.secondImpulse;
        contactDiagnostics_.firstMoment += record.firstMoment;
        contactDiagnostics_.secondMoment += record.secondMoment;
        contactDiagnostics_.frictionWork += record.frictionWork;
        contactDiagnostics_.maximumFrictionResidual =
            std::max(contactDiagnostics_.maximumFrictionResidual,
                     record.frictionResidual);
        pairDiagnostic.firstImpulse += record.firstImpulse;
        pairDiagnostic.secondImpulse += record.secondImpulse;
        pairDiagnostic.firstMoment += record.firstMoment;
        pairDiagnostic.secondMoment += record.secondMoment;
        pairDiagnostic.frictionWork += record.frictionWork;
        pairDiagnostic.maximumFrictionResidual =
            std::max(pairDiagnostic.maximumFrictionResidual,
                     record.frictionResidual);
    }
    contactDiagnostics_.netInternalImpulse =
        contactDiagnostics_.firstImpulse +
        contactDiagnostics_.secondImpulse;
    contactDiagnostics_.netInternalMoment =
        contactDiagnostics_.firstMoment +
        contactDiagnostics_.secondMoment;
    for (ContactDiagnostics& pairDiagnostic : contactPairDiagnostics_) {
        pairDiagnostic.netInternalImpulse =
            pairDiagnostic.firstImpulse + pairDiagnostic.secondImpulse;
        pairDiagnostic.netInternalMoment =
            pairDiagnostic.firstMoment + pairDiagnostic.secondMoment;
    }
}

ContactSurfaceHandle SoftBody::addContactSurface(
    const SurfaceGroup& surface,
    double halfThickness) {
    requireSurfaceGroup(surface);
    if (!std::isfinite(halfThickness) || !(halfThickness > 0.0)) {
        throw std::invalid_argument(
            "Contact surface half-thickness must be finite and positive");
    }
    const std::size_t requestedLast =
        surface.firstTriangle_ + surface.triangleCount_;
    for (const RegisteredContactSurface& existing : contactSurfaces_) {
        const std::size_t existingLast =
            existing.firstTriangle + existing.triangleCount;
        if (surface.firstTriangle_ < existingLast &&
            existing.firstTriangle < requestedLast) {
            throw std::invalid_argument(
                "Contact surface triangle ranges must not overlap");
        }
    }

    std::set<std::size_t> vertices;
    std::set<ContactEdge> edges;
    for (std::size_t triangleIndex = surface.firstTriangle_;
         triangleIndex < requestedLast;
         ++triangleIndex) {
        const Triangle& triangle = triangles_[triangleIndex];
        if (triangle.a >= nodes_.size() || triangle.b >= nodes_.size() ||
            triangle.c >= nodes_.size()) {
            throw std::out_of_range(
                "Contact surface triangle contains an invalid node");
        }
        if (triangle.a == triangle.b || triangle.b == triangle.c ||
            triangle.c == triangle.a) {
            throw std::invalid_argument(
                "Contact surface contains a repeated-node triangle");
        }
        const Vec3& a = nodes_[triangle.a].position;
        const Vec3& b = nodes_[triangle.b].position;
        const Vec3& c = nodes_[triangle.c].position;
        if (!finite(a) || !finite(b) || !finite(c)) {
            throw std::invalid_argument(
                "Contact surface positions must be finite");
        }
        const double ab2 = lengthSquared(b - a);
        const double ac2 = lengthSquared(c - a);
        const double bc2 = lengthSquared(c - b);
        const double scale2 = std::max({ab2, ac2, bc2});
        const double twiceArea2 = lengthSquared(cross(b - a, c - a));
        const double relativeTolerance =
            64.0 * std::numeric_limits<double>::epsilon();
        if (!(scale2 > 0.0) ||
            !(twiceArea2 > relativeTolerance * relativeTolerance *
                                  scale2 * scale2)) {
            throw std::invalid_argument(
                "Contact surface contains a degenerate triangle");
        }
        const std::array<std::size_t, 3> indices{
            triangle.a, triangle.b, triangle.c};
        vertices.insert(indices.begin(), indices.end());
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const auto [first, second] = std::minmax(
                indices[edge], indices[(edge + 1) % indices.size()]);
            edges.insert({first, second});
        }
    }

    RegisteredContactSurface pending;
    pending.firstTriangle = surface.firstTriangle_;
    pending.triangleCount = surface.triangleCount_;
    pending.halfThickness = halfThickness;
    pending.vertices.assign(vertices.begin(), vertices.end());
    pending.edges.assign(edges.begin(), edges.end());
    const std::size_t index = contactSurfaces_.size();
    contactSurfaces_.push_back(std::move(pending));
    return ContactSurfaceHandle(this, index);
}

ContactLineHandle SoftBody::addContactLine(std::size_t a,
                                           std::size_t b,
                                           double radius) {
    if (a >= nodes_.size() || b >= nodes_.size()) {
        throw std::out_of_range("Contact line node index is out of range");
    }
    if (a == b || !std::isfinite(radius) || !(radius > 0.0) ||
        !finite(nodes_[a].position) || !finite(nodes_[b].position)) {
        throw std::invalid_argument(
            "Contact line requires distinct finite nodes and positive radius");
    }
    const auto requested = std::minmax(a, b);
    for (const RegisteredContactLine& existing : contactLines_) {
        if (std::minmax(existing.a, existing.b) == requested) {
            throw std::invalid_argument(
                "Duplicate contact line registration is not allowed");
        }
    }
    const std::size_t index = contactLines_.size();
    contactLines_.push_back({a, b, radius});
    return ContactLineHandle(this, index);
}

ContactPairHandle SoftBody::addContactPair(
    const ContactColliderHandle& first,
    const ContactColliderHandle& second,
    const ContactPairSettings& settings) {
    validatePairSettings(settings);
    if (first.owner_ != this || second.owner_ != this) {
        throw std::invalid_argument(
            "Contact collider belongs to another SoftBody");
    }
    const auto requireCollider = [this](const ContactColliderHandle& collider) {
        if (collider.kind_ == ContactColliderKind::Surface) {
            if (collider.index_ >= contactSurfaces_.size()) {
                throw std::out_of_range(
                    "Contact surface handle is out of range");
            }
        } else if (collider.kind_ == ContactColliderKind::Line) {
            if (collider.index_ >= contactLines_.size()) {
                throw std::out_of_range("Contact line handle is out of range");
            }
        } else {
            throw std::invalid_argument("Invalid contact collider kind");
        }
    };
    requireCollider(first);
    requireCollider(second);
    if (first.kind_ == ContactColliderKind::Line &&
        second.kind_ == ContactColliderKind::Line) {
        throw std::invalid_argument("Line-line contact is not supported");
    }

    ContactColliderKind firstKind = first.kind_;
    ContactColliderKind secondKind = second.kind_;
    std::size_t firstIndex = first.index_;
    std::size_t secondIndex = second.index_;
    if (firstKind == ContactColliderKind::Line ||
        (firstKind == secondKind && firstIndex > secondIndex)) {
        std::swap(firstKind, secondKind);
        std::swap(firstIndex, secondIndex);
    }
    const ContactPairKind kind =
        secondKind == ContactColliderKind::Line
            ? ContactPairKind::SurfaceLine
            : ContactPairKind::SurfaceSurface;
    for (const RegisteredContactPair& existing : contactPairs_) {
        if (existing.firstKind == firstKind && existing.first == firstIndex &&
            existing.secondKind == secondKind &&
            existing.second == secondIndex) {
            throw std::invalid_argument(
                "Duplicate contact pair registration is not allowed");
        }
    }

    const std::size_t index = contactPairs_.size();
    contactPairs_.push_back(
        {kind,
         firstKind,
         firstIndex,
         secondKind,
         secondIndex,
         settings});
    return ContactPairHandle(this, index);
}

ContactColliderHandle SoftBody::contactSurfaceCollider(
    std::size_t surfaceIndex) const {
    if (surfaceIndex >= contactSurfaces_.size())
        throw std::out_of_range("Contact surface index is out of range");
    return ContactColliderHandle(
        this, ContactColliderKind::Surface, surfaceIndex);
}

void SoftBody::requireContactSurface(
    const ContactSurfaceHandle& surface) const {
    if (surface.owner_ != this) {
        throw std::invalid_argument(
            "Contact surface belongs to another SoftBody");
    }
    if (surface.index_ >= contactSurfaces_.size()) {
        throw std::out_of_range("Contact surface handle is out of range");
    }
}

void SoftBody::requireContactLine(const ContactLineHandle& line) const {
    if (line.owner_ != this) {
        throw std::invalid_argument("Contact line belongs to another SoftBody");
    }
    if (line.index_ >= contactLines_.size()) {
        throw std::out_of_range("Contact line handle is out of range");
    }
}

void SoftBody::requireContactPair(const ContactPairHandle& pair) const {
    if (pair.owner_ != this) {
        throw std::invalid_argument("Contact pair belongs to another SoftBody");
    }
    if (pair.index_ >= contactPairs_.size()) {
        throw std::out_of_range("Contact pair handle is out of range");
    }
}

const RegisteredContactSurface& SoftBody::contactSurface(
    const ContactSurfaceHandle& surface) const {
    requireContactSurface(surface);
    return contactSurfaces_[surface.index_];
}

const RegisteredContactLine& SoftBody::contactLine(
    const ContactLineHandle& line) const {
    requireContactLine(line);
    return contactLines_[line.index_];
}

const RegisteredContactPair& SoftBody::contactPair(
    const ContactPairHandle& pair) const {
    requireContactPair(pair);
    return contactPairs_[pair.index_];
}

ContactDiagnostics SoftBody::contactDiagnostics(
    const ContactPairHandle& pair) const {
    requireContactPair(pair);
    if (pair.index_ >= contactPairDiagnostics_.size()) {
        ContactDiagnostics result;
        result.registered = true;
        result.solveSucceeded = true;
        return result;
    }
    return contactPairDiagnostics_[pair.index_];
}

ContactTopologyReport SoftBody::contactTopology(
    const ContactPairHandle& pairHandle) const {
    requireContactPair(pairHandle);
    const std::size_t pairIndex = pairHandle.index_;
    const RegisteredContactPair& pair = contactPairs_[pairIndex];
    ContactTopologyReport report;

    const auto triangleContains = [this](std::size_t triangleIndex,
                                         std::size_t nodeIndex) {
        const Triangle& triangle = triangles_[triangleIndex];
        return triangle.a == nodeIndex || triangle.b == nodeIndex ||
               triangle.c == nodeIndex;
    };
    const auto addVertexTriangles =
        [this, pairIndex, &report, &triangleContains](
            const RegisteredContactSurface& vertices,
            const RegisteredContactSurface& triangles) {
            const std::size_t triangleLast =
                triangles.firstTriangle + triangles.triangleCount;
            for (const std::size_t vertex : vertices.vertices) {
                for (std::size_t triangleIndex = triangles.firstTriangle;
                     triangleIndex < triangleLast;
                     ++triangleIndex) {
                    ++report.possibleCount;
                    if (triangleContains(triangleIndex, vertex)) {
                        ++report.excludedCount;
                        continue;
                    }
                    report.eligibleKeys.push_back(
                        {pairIndex,
                         ContactFeatureKind::VertexTriangle,
                         {vertex, 0, 0},
                         {triangleIndex, 0, 0}});
                }
            }
        };
    const auto shareNode = [](const ContactEdge& first,
                              const ContactEdge& second) {
        return first.a == second.a || first.a == second.b ||
               first.b == second.a || first.b == second.b;
    };
    const auto addEdgePair = [pairIndex, &report, &shareNode](
                                 const ContactEdge& first,
                                 const ContactEdge& second) {
        ++report.possibleCount;
        if (shareNode(first, second)) {
            ++report.excludedCount;
            return;
        }
        ContactEdge canonicalFirst = first;
        ContactEdge canonicalSecond = second;
        if (canonicalSecond < canonicalFirst) {
            std::swap(canonicalFirst, canonicalSecond);
        }
        report.eligibleKeys.push_back(
            {pairIndex,
             ContactFeatureKind::EdgeEdge,
             {canonicalFirst.a, canonicalFirst.b, 0},
             {canonicalSecond.a, canonicalSecond.b, 0}});
    };

    if (pair.kind == ContactPairKind::SurfaceSurface) {
        const RegisteredContactSurface& first =
            contactSurfaces_[pair.first];
        const RegisteredContactSurface& second =
            contactSurfaces_[pair.second];
        addVertexTriangles(first, second);
        if (pair.first != pair.second) {
            addVertexTriangles(second, first);
            for (const ContactEdge& firstEdge : first.edges) {
                for (const ContactEdge& secondEdge : second.edges) {
                    addEdgePair(firstEdge, secondEdge);
                }
            }
        } else {
            for (std::size_t firstEdge = 0; firstEdge < first.edges.size();
                 ++firstEdge) {
                for (std::size_t secondEdge = firstEdge + 1;
                     secondEdge < first.edges.size();
                     ++secondEdge) {
                    addEdgePair(first.edges[firstEdge],
                                first.edges[secondEdge]);
                }
            }
        }
    } else {
        const RegisteredContactSurface& surface =
            contactSurfaces_[pair.first];
        const RegisteredContactLine& line = contactLines_[pair.second];
        const std::size_t triangleLast =
            surface.firstTriangle + surface.triangleCount;
        for (std::size_t triangleIndex = surface.firstTriangle;
             triangleIndex < triangleLast;
             ++triangleIndex) {
            ++report.possibleCount;
            if (triangleContains(triangleIndex, line.a) ||
                triangleContains(triangleIndex, line.b)) {
                ++report.excludedCount;
                continue;
            }
            report.eligibleKeys.push_back(
                {pairIndex,
                 ContactFeatureKind::SegmentTriangle,
                 {line.a, line.b, pair.second},
                 {triangleIndex, 0, 0}});
        }
    }
    std::sort(report.eligibleKeys.begin(), report.eligibleKeys.end());
    return report;
}

} // namespace softwing
