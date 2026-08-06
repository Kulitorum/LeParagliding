#ifndef LEP_PLAYGROUND_CONTACT_H
#define LEP_PLAYGROUND_CONTACT_H

#include <softwing/soft_body.h>

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace lep::playground {

// Playground contact is deliberately a small, switchable adapter rather than
// a registration in SoftBody's exhaustive certified contact pipeline.  This
// header and its implementation are Qt-free so geometry/contact tests do not
// need the GUI or mesh parser.
enum class PlaygroundContactFeature : std::uint8_t
{
    VertexTriangle,
    EdgeEdge,
    SegmentTriangle,
};

struct PlaygroundContactLine
{
    std::uint32_t a = 0;
    std::uint32_t b = 0;
};

struct PlaygroundContactEdge
{
    std::uint32_t a = 0;
    std::uint32_t b = 0;

    auto operator<=>(const PlaygroundContactEdge &) const = default;
};

struct PlaygroundContactCandidate
{
    PlaygroundContactFeature feature =
        PlaygroundContactFeature::VertexTriangle;
    // Vertex index / skin-edge index / authored-line index.
    std::uint32_t first = 0;
    // Triangle index / second skin-edge index / triangle index.
    std::uint32_t second = 0;
    // The separating direction from the second feature to the first at
    // capture. It is retained through all substeps so a crossing cannot flip
    // which side the correction returns to.
    softwing::Vec3 normal{0.0, 0.0, 1.0};

    auto operator<=>(const PlaygroundContactCandidate &other) const
    {
        if (feature != other.feature) {
            return feature <=> other.feature;
        }
        if (first != other.first) {
            return first <=> other.first;
        }
        return second <=> other.second;
    }
};

struct PlaygroundContactLimits
{
    // Each feature family gets this many swept-AABB overlap visits. A hit is
    // reported and makes coverageComplete false; pairs are never silently
    // discarded.
    std::size_t maxBroadPhaseTestsPerFeature = 500000;
    std::size_t maxCandidatesPerFeature = 20000;
    double maximumCorrectionMetres = 0.001;
};

struct PlaygroundContactStats
{
    std::size_t vertexTriangleCandidates = 0;
    std::size_t edgeEdgeCandidates = 0;
    std::size_t segmentTriangleCandidates = 0;
    std::size_t activeContacts = 0;
    std::size_t projectionVisits = 0;
    std::size_t topologyExcludedPairs = 0;
    std::size_t broadPhaseTests = 0;
    std::size_t broadPhaseBudgetHits = 0;
    std::size_t candidateBudgetHits = 0;
    std::size_t geometryQueryFailures = 0;
    std::size_t largeSweptEnvelopes = 0;
    std::size_t substepRefreshes = 0;
    bool coverageComplete = true;
    double worstPenetrationBefore = 0.0;
    double worstPenetrationAfter = 0.0;
};

struct PlaygroundContactScratch
{
    PlaygroundContactLimits limits;
    PlaygroundContactStats stats;
    std::vector<PlaygroundContactCandidate> candidates;
    std::vector<std::uint32_t> skinNodes;
    std::vector<PlaygroundContactEdge> skinEdges;
    std::vector<PlaygroundContactLine> authoredLines;
    // Canonical (low, high) skin vertex pairs containing self and one-ring
    // adjacency. These are the only fabric topology exclusions.
    std::vector<std::uint64_t> oneRingPairs;
    std::vector<softwing::Vec3> capturePositions;
    std::vector<double> captureAllowance;
    double meanSkinEdgeLength = 0.05;
    bool prepared = false;
};

inline constexpr double playgroundFabricContactSeparation = 0.001;
inline constexpr double playgroundLineContactSeparation = 0.0015;

void preparePlaygroundContact(
    PlaygroundContactScratch &scratch,
    std::span<const softwing::Node> nodes,
    std::span<const softwing::Triangle> triangles,
    std::size_t skinTriangleCount,
    std::span<const PlaygroundContactLine> authoredLines);

void beginPlaygroundContactFrame(
    PlaygroundContactScratch &scratch,
    std::span<const softwing::Node> nodes,
    std::span<const softwing::Triangle> triangles,
    std::size_t skinTriangleCount,
    double remainingFrameSeconds);

[[nodiscard]] bool playgroundContactEnvelopeEscaped(
    const PlaygroundContactScratch &scratch,
    std::span<const softwing::Node> nodes);

void refreshPlaygroundContact(
    PlaygroundContactScratch &scratch,
    std::span<const softwing::Node> nodes,
    std::span<const softwing::Triangle> triangles,
    std::size_t skinTriangleCount,
    double remainingFrameSeconds);

void projectPlaygroundContact(
    PlaygroundContactScratch &scratch,
    std::span<softwing::Node> nodes,
    std::span<const softwing::Triangle> triangles);

}  // namespace lep::playground

#endif
