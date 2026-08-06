#include "softwing/cell_volume.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace softwing {
namespace {

struct Edge {
    std::size_t low = 0;
    std::size_t high = 0;

    friend bool operator==(const Edge&, const Edge&) = default;
};

struct EdgeHash {
    std::size_t operator()(const Edge& edge) const {
        const std::size_t seed = std::hash<std::size_t>{}(edge.low);
        return seed ^ (std::hash<std::size_t>{}(edge.high)
                       + static_cast<std::size_t>(0x9e3779b9U)
                       + (seed << 6U) + (seed >> 2U));
    }
};

struct EdgeUse {
    std::size_t count = 0;
    bool soleUseRunsLowToHigh = false;
};

struct CapOrientation {
    bool forward = true;
    bool usedFallback = false;
    bool valid = false;
};

bool finite(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
           && std::isfinite(value.z);
}

Edge edgeKey(std::size_t a, std::size_t b) {
    return {std::min(a, b), std::max(a, b)};
}

void addEdge(std::unordered_map<Edge, EdgeUse, EdgeHash>& edges,
             std::size_t a,
             std::size_t b) {
    const Edge key = edgeKey(a, b);
    EdgeUse& use = edges[key];
    ++use.count;
    if (use.count == 1) {
        use.soleUseRunsLowToHigh = a == key.low;
    }
}

double tetrahedronVolume(const Vec3& a,
                         const Vec3& b,
                         const Vec3& c,
                         const Vec3& origin) {
    return dot(a - origin, cross(b - origin, c - origin)) / 6.0;
}

Vec3 polygonCentroid(std::span<const Node> nodes,
                     std::span<const std::size_t> loop) {
    Vec3 result;
    for (const std::size_t node : loop) {
        result += nodes[node].position;
    }
    return result / static_cast<double>(loop.size());
}

Vec3 polygonAreaVector(std::span<const Node> nodes,
                       std::span<const std::size_t> loop,
                       const Vec3& origin) {
    Vec3 result;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const Vec3 a = nodes[loop[i]].position - origin;
        const Vec3 b = nodes[loop[(i + 1) % loop.size()]].position - origin;
        result += cross(a, b);
    }
    return result * 0.5;
}

CapOrientation capOrientation(
    std::span<const Node> nodes,
    std::span<const std::size_t> loop,
    const std::unordered_map<Edge, EdgeUse, EdgeHash>& skinEdges,
    const Vec3& desiredNormal) {
    int forwardVotes = 0;
    int reverseVotes = 0;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const std::size_t a = loop[i];
        const std::size_t b = loop[(i + 1) % loop.size()];
        const Edge key = edgeKey(a, b);
        const auto found = skinEdges.find(key);
        if (found == skinEdges.end() || found->second.count != 1) {
            continue;
        }

        const bool loopRunsLowToHigh = a == key.low;
        // A cap closes the skin only when its boundary edge runs opposite to
        // the skin's sole boundary use.
        if (loopRunsLowToHigh != found->second.soleUseRunsLowToHigh) {
            ++forwardVotes;
        } else {
            ++reverseVotes;
        }
    }

    if (forwardVotes != reverseVotes) {
        return {forwardVotes > reverseVotes, false, true};
    }

    const Vec3 center = polygonCentroid(nodes, loop);
    const Vec3 area = polygonAreaVector(nodes, loop, center);
    const double alignment = dot(area, desiredNormal);
    const double scale = length(area) * length(desiredNormal);
    if (!std::isfinite(alignment) || !std::isfinite(scale)
        || scale <= std::numeric_limits<double>::epsilon()
        || std::abs(alignment) <= scale * 1.0e-10) {
        return {true, true, false};
    }
    return {alignment > 0.0, true, true};
}

double capVolume(std::span<const Node> nodes,
                 std::span<const std::size_t> loop,
                 bool forward,
                 const Vec3& origin) {
    const Vec3 center = polygonCentroid(nodes, loop);
    double result = 0.0;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const Vec3& a = nodes[loop[i]].position;
        const Vec3& b = nodes[loop[(i + 1) % loop.size()]].position;
        result += forward ? tetrahedronVolume(center, a, b, origin)
                          : tetrahedronVolume(center, b, a, origin);
    }
    return result;
}

bool validLoop(std::span<const Node> nodes,
               std::span<const std::size_t> loop) {
    if (loop.size() < 3) {
        return false;
    }
    std::unordered_set<std::size_t> unique;
    unique.reserve(loop.size());
    for (const std::size_t node : loop) {
        if (node >= nodes.size() || !finite(nodes[node].position)
            || !unique.insert(node).second) {
            return false;
        }
    }
    return true;
}

} // namespace

ClosedCellVolumeEstimate estimateClosedCellVolume(
    std::span<const Node> nodes,
    std::span<const Triangle> triangles,
    std::span<const std::size_t> skinFaceIndices,
    std::span<const std::size_t> lowRibLoop,
    std::span<const std::size_t> highRibLoop) {
    ClosedCellVolumeEstimate result;
    if (skinFaceIndices.empty() || !validLoop(nodes, lowRibLoop)
        || !validLoop(nodes, highRibLoop)) {
        return result;
    }

    std::unordered_map<Edge, EdgeUse, EdgeHash> skinEdges;
    skinEdges.reserve(skinFaceIndices.size() * 3);
    std::vector<const Triangle*> skinFaces;
    skinFaces.reserve(skinFaceIndices.size());
    std::unordered_set<std::size_t> uniqueFaces;
    uniqueFaces.reserve(skinFaceIndices.size());

    Vec3 origin;
    std::size_t originSampleCount = 0;
    for (const std::size_t faceIndex : skinFaceIndices) {
        if (faceIndex >= triangles.size() || !uniqueFaces.insert(faceIndex).second) {
            return result;
        }
        const Triangle& face = triangles[faceIndex];
        if (face.a >= nodes.size() || face.b >= nodes.size()
            || face.c >= nodes.size() || !finite(nodes[face.a].position)
            || !finite(nodes[face.b].position)
            || !finite(nodes[face.c].position)) {
            return result;
        }
        skinFaces.push_back(&face);
        addEdge(skinEdges, face.a, face.b);
        addEdge(skinEdges, face.b, face.c);
        addEdge(skinEdges, face.c, face.a);
        origin += nodes[face.a].position + nodes[face.b].position
                  + nodes[face.c].position;
        originSampleCount += 3;

        const Vec3 area = cross(nodes[face.b].position - nodes[face.a].position,
                                nodes[face.c].position - nodes[face.a].position);
        if (lengthSquared(area) <= std::numeric_limits<double>::epsilon()) {
            ++result.degenerateSkinFaces;
        }
    }
    for (const std::size_t node : lowRibLoop) {
        origin += nodes[node].position;
        ++originSampleCount;
    }
    for (const std::size_t node : highRibLoop) {
        origin += nodes[node].position;
        ++originSampleCount;
    }
    origin /= static_cast<double>(originSampleCount);

    for (const auto& [edge, use] : skinEdges) {
        static_cast<void>(edge);
        if (use.count == 1) {
            ++result.skinBoundaryEdges;
        } else if (use.count > 2) {
            ++result.nonManifoldSkinEdges;
        }
    }

    std::unordered_set<Edge, EdgeHash> capEdges;
    capEdges.reserve(lowRibLoop.size() + highRibLoop.size());
    bool uniqueCapEdges = true;
    const auto countCapMatches = [&](std::span<const std::size_t> loop) {
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const Edge edge = edgeKey(loop[i], loop[(i + 1) % loop.size()]);
            uniqueCapEdges = capEdges.insert(edge).second && uniqueCapEdges;
            const auto found = skinEdges.find(edge);
            if (found != skinEdges.end() && found->second.count == 1) {
                ++result.capBoundaryEdgesMatched;
            }
        }
    };
    countCapMatches(lowRibLoop);
    countCapMatches(highRibLoop);
    const std::size_t capEdgeCount = lowRibLoop.size() + highRibLoop.size();
    result.watertightAfterCapping = uniqueCapEdges
                                    && result.nonManifoldSkinEdges == 0
                                    && result.skinBoundaryEdges == capEdgeCount
                                    && result.capBoundaryEdgesMatched == capEdgeCount;

    const Vec3 lowCenter = polygonCentroid(nodes, lowRibLoop);
    const Vec3 highCenter = polygonCentroid(nodes, highRibLoop);
    const Vec3 spanDirection = highCenter - lowCenter;
    if (!finite(spanDirection)
        || lengthSquared(spanDirection) <= std::numeric_limits<double>::epsilon()) {
        return result;
    }

    double skinOrientationScore = 0.0;
    double skinOrientationScale = 0.0;
    for (const Triangle* face : skinFaces) {
        const Vec3& a = nodes[face->a].position;
        const Vec3& b = nodes[face->b].position;
        const Vec3& c = nodes[face->c].position;
        const Vec3 area = cross(b - a, c - a);
        const Vec3 faceCenter = (a + b + c) / 3.0;
        skinOrientationScore += dot(area, faceCenter - origin);
        skinOrientationScale += length(area) * length(faceCenter - origin);
    }
    const bool ambiguousSkinOrientation =
        skinOrientationScale <= std::numeric_limits<double>::epsilon()
        || std::abs(skinOrientationScore) <= skinOrientationScale * 1.0e-10;
    result.ambiguousSkinOrientation = ambiguousSkinOrientation;
    const double skinOrientationSign = skinOrientationScore < 0.0 ? -1.0 : 1.0;

    const CapOrientation lowOrientation = capOrientation(
        nodes, lowRibLoop, skinEdges, spanDirection * -skinOrientationSign);
    const CapOrientation highOrientation = capOrientation(
        nodes, highRibLoop, skinEdges, spanDirection * skinOrientationSign);
    result.usedGeometricOrientationFallback = lowOrientation.usedFallback
                                              || highOrientation.usedFallback;
    if (!lowOrientation.valid || !highOrientation.valid
        || (result.usedGeometricOrientationFallback
            && ambiguousSkinOrientation)) {
        return result;
    }

    double signedVolume = 0.0;
    for (const Triangle* face : skinFaces) {
        signedVolume += tetrahedronVolume(nodes[face->a].position,
                                          nodes[face->b].position,
                                          nodes[face->c].position,
                                          origin);
    }
    signedVolume += capVolume(nodes, lowRibLoop, lowOrientation.forward, origin);
    signedVolume += capVolume(nodes, highRibLoop, highOrientation.forward, origin);

    result.signedVolume = signedVolume;
    result.volume = std::abs(signedVolume);
    result.valid = std::isfinite(result.volume)
                   && result.volume > std::numeric_limits<double>::epsilon();
    if (!result.valid) {
        result.signedVolume = 0.0;
        result.volume = 0.0;
    }
    return result;
}

} // namespace softwing
