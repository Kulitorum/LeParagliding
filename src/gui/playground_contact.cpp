#include "playground_contact.h"

#include <softwing/contact.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <numeric>
#include <utility>

namespace lep::playground {
namespace {

using softwing::ClosestFeatureResult;
using softwing::Node;
using softwing::Triangle;
using softwing::Vec3;

struct Bounds
{
    Vec3 low;
    Vec3 high;
};

struct Proxy
{
    std::uint32_t id = 0;
    Bounds bounds;
    double margin = 0.0;
};

std::uint64_t vertexPairKey(std::uint32_t first, std::uint32_t second)
{
    if (second < first) {
        std::swap(first, second);
    }
    return (static_cast<std::uint64_t>(first) << 32) | second;
}

bool adjacent(const PlaygroundContactScratch &scratch,
              std::uint32_t first,
              std::uint32_t second)
{
    return std::binary_search(scratch.oneRingPairs.begin(),
                              scratch.oneRingPairs.end(),
                              vertexPairKey(first, second));
}

Bounds pointBounds(const Vec3 &point, double expansion)
{
    return {{point.x - expansion, point.y - expansion, point.z - expansion},
            {point.x + expansion, point.y + expansion,
             point.z + expansion}};
}

Bounds primitiveBounds(std::span<const Node> nodes,
                       std::span<const std::uint32_t> indices,
                       double expansion)
{
    Bounds result{{std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity()},
                  {-std::numeric_limits<double>::infinity(),
                   -std::numeric_limits<double>::infinity(),
                   -std::numeric_limits<double>::infinity()}};
    for (const std::uint32_t index : indices) {
        const Vec3 &point = nodes[index].position;
        result.low.x = std::min(result.low.x, point.x - expansion);
        result.low.y = std::min(result.low.y, point.y - expansion);
        result.low.z = std::min(result.low.z, point.z - expansion);
        result.high.x = std::max(result.high.x, point.x + expansion);
        result.high.y = std::max(result.high.y, point.y + expansion);
        result.high.z = std::max(result.high.z, point.z + expansion);
    }
    return result;
}

double coordinate(const Vec3 &point, int axis)
{
    return axis == 0 ? point.x : axis == 1 ? point.y : point.z;
}

int extentAxis(std::span<const Proxy> first, std::span<const Proxy> second)
{
    Vec3 low{std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity()};
    Vec3 high{-std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity()};
    const auto include = [&low, &high](const Proxy &proxy) {
        low.x = std::min(low.x, proxy.bounds.low.x);
        low.y = std::min(low.y, proxy.bounds.low.y);
        low.z = std::min(low.z, proxy.bounds.low.z);
        high.x = std::max(high.x, proxy.bounds.high.x);
        high.y = std::max(high.y, proxy.bounds.high.y);
        high.z = std::max(high.z, proxy.bounds.high.z);
    };
    for (const Proxy &proxy : first) {
        include(proxy);
    }
    for (const Proxy &proxy : second) {
        include(proxy);
    }
    const Vec3 extent = high - low;
    return extent.y > extent.x && extent.y >= extent.z
               ? 1
               : extent.z > extent.x && extent.z > extent.y ? 2 : 0;
}

std::uint64_t crossIntervalOverlapEstimate(std::span<const Proxy> first,
                                           std::span<const Proxy> second,
                                           int axis)
{
    std::vector<double> starts;
    std::vector<double> ends;
    starts.reserve(second.size());
    ends.reserve(second.size());
    for (const Proxy &proxy : second) {
        starts.push_back(coordinate(proxy.bounds.low, axis));
        ends.push_back(coordinate(proxy.bounds.high, axis));
    }
    std::sort(starts.begin(), starts.end());
    std::sort(ends.begin(), ends.end());
    std::uint64_t estimate = 0;
    for (const Proxy &proxy : first) {
        const auto begun = std::upper_bound(
            starts.begin(), starts.end(),
            coordinate(proxy.bounds.high, axis));
        const auto finished = std::lower_bound(
            ends.begin(), ends.end(), coordinate(proxy.bounds.low, axis));
        estimate += static_cast<std::uint64_t>(begun - starts.begin())
                    - static_cast<std::uint64_t>(finished - ends.begin());
    }
    return estimate;
}

int crossSweepAxis(std::span<const Proxy> first,
                   std::span<const Proxy> second)
{
    int best = extentAxis(first, second);
    std::uint64_t bestEstimate =
        crossIntervalOverlapEstimate(first, second, best);
    for (int axis = 0; axis < 3; ++axis) {
        const std::uint64_t estimate =
            crossIntervalOverlapEstimate(first, second, axis);
        if (estimate < bestEstimate) {
            best = axis;
            bestEstimate = estimate;
        }
    }
    return best;
}

int selfSweepAxis(std::span<const Proxy> proxies)
{
    // The same cross estimate counts every overlap twice and each primitive
    // once. That constant factor does not affect which axis is best.
    return crossSweepAxis(proxies, proxies);
}

bool overlaps(const Bounds &first, const Bounds &second)
{
    return first.low.x <= second.high.x && second.low.x <= first.high.x
           && first.low.y <= second.high.y
           && second.low.y <= first.high.y
           && first.low.z <= second.high.z
           && second.low.z <= first.high.z;
}

std::uint64_t gridCellKey(int x, int y, int z)
{
    const auto pack = [](int value) {
        return static_cast<std::uint64_t>(value + (1 << 20)) & 0x1FFFFFULL;
    };
    return pack(x) | (pack(y) << 21) | (pack(z) << 42);
}

using GridEntry = std::pair<std::uint64_t, std::uint32_t>;

template <typename Callback>
bool crossSweep(std::span<const Proxy> first,
                std::span<const Proxy> second,
                std::size_t budget,
                std::size_t &visits,
                Callback &&callback);

template <typename Callback>
bool selfSweep(std::span<const Proxy> proxies,
               std::size_t budget,
               std::size_t &visits,
               Callback &&callback);

void insertProxyGrid(const Proxy &proxy,
                     std::uint32_t proxyIndex,
                     double cellSize,
                     std::vector<GridEntry> &entries,
                     std::vector<std::uint32_t> &oversized)
{
    const auto cell = [cellSize](double value) {
        return static_cast<int>(std::floor(value / cellSize));
    };
    const int x0 = cell(proxy.bounds.low.x);
    const int y0 = cell(proxy.bounds.low.y);
    const int z0 = cell(proxy.bounds.low.z);
    const int x1 = cell(proxy.bounds.high.x);
    const int y1 = cell(proxy.bounds.high.y);
    const int z1 = cell(proxy.bounds.high.z);
    const std::uint64_t count =
        static_cast<std::uint64_t>(x1 - x0 + 1)
        * static_cast<std::uint64_t>(y1 - y0 + 1)
        * static_cast<std::uint64_t>(z1 - z0 + 1);
    constexpr std::uint64_t maximumCellsPerPrimitive = 64;
    if (count > maximumCellsPerPrimitive) {
        oversized.push_back(proxyIndex);
        return;
    }
    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                entries.emplace_back(gridCellKey(x, y, z), proxyIndex);
            }
        }
    }
}

const Proxy *proxyWithId(std::span<const Proxy> proxies, std::uint32_t id)
{
    const auto found = std::lower_bound(
        proxies.begin(), proxies.end(), id,
        [](const Proxy &proxy, std::uint32_t wanted) {
            return proxy.id < wanted;
        });
    return found != proxies.end() && found->id == id ? &*found : nullptr;
}

bool emitPair(std::vector<std::uint64_t> &pairs,
              std::uint32_t first,
              std::uint32_t second,
              std::size_t rawBudget)
{
    if (pairs.size() >= rawBudget) {
        return false;
    }
    pairs.push_back((static_cast<std::uint64_t>(first) << 32) | second);
    return true;
}

bool finishPairs(std::vector<std::uint64_t> &pairs,
                 std::size_t budget,
                 bool complete)
{
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    if (pairs.size() > budget) {
        pairs.resize(budget);
        complete = false;
    }
    return complete;
}

// Regular primitives use a complete 3-D grid. Primitives spanning too many
// cells are put on the explicit oversized list and tested against the full
// opposite set. Dense cells use the same local interval sweep as the global
// fallback; no cell is skipped because its pair product is large.
bool gridCrossPairs(std::span<const Proxy> first,
                    std::span<const Proxy> second,
                    double cellSize,
                    std::size_t budget,
                    std::vector<std::uint64_t> &pairs,
                    std::size_t &oversizedCount,
                    const std::function<bool(std::uint32_t,
                                             std::uint32_t)> &eligible)
{
    std::vector<GridEntry> firstCells;
    std::vector<GridEntry> secondCells;
    std::vector<std::uint32_t> firstOversized;
    std::vector<std::uint32_t> secondOversized;
    for (std::uint32_t index = 0; index < first.size(); ++index) {
        insertProxyGrid(first[index], index, cellSize, firstCells,
                        firstOversized);
    }
    for (std::uint32_t index = 0; index < second.size(); ++index) {
        insertProxyGrid(second[index], index, cellSize, secondCells,
                        secondOversized);
    }
    oversizedCount += firstOversized.size() + secondOversized.size();
    std::sort(firstCells.begin(), firstCells.end());
    std::sort(secondCells.begin(), secondCells.end());
    // Cell overlap duplicates are bounded separately from the unique-pair
    // budget. A hit is returned as incomplete, never hidden.
    const std::size_t rawBudget =
        budget > std::numeric_limits<std::size_t>::max() / 8
            ? std::numeric_limits<std::size_t>::max()
            : std::max<std::size_t>(budget, 1) * 8;
    pairs.clear();
    std::size_t a = 0;
    std::size_t b = 0;
    constexpr std::size_t denseProduct = 4096;
    while (a < firstCells.size() && b < secondCells.size()) {
        if (firstCells[a].first < secondCells[b].first) {
            ++a;
            continue;
        }
        if (secondCells[b].first < firstCells[a].first) {
            ++b;
            continue;
        }
        std::size_t aEnd = a + 1;
        while (aEnd < firstCells.size()
               && firstCells[aEnd].first == firstCells[a].first) {
            ++aEnd;
        }
        std::size_t bEnd = b + 1;
        while (bEnd < secondCells.size()
               && secondCells[bEnd].first == secondCells[b].first) {
            ++bEnd;
        }
        const std::size_t product = (aEnd - a) * (bEnd - b);
        if (product <= denseProduct) {
            for (std::size_t i = a; i < aEnd; ++i) {
                const Proxy *firstProxy = &first[firstCells[i].second];
                for (std::size_t j = b; j < bEnd; ++j) {
                    const Proxy *secondProxy = &second[secondCells[j].second];
                    if (firstProxy && secondProxy
                        && overlaps(firstProxy->bounds,
                                    secondProxy->bounds)
                        && eligible(firstProxy->id, secondProxy->id)
                        && !emitPair(pairs, firstProxy->id,
                                     secondProxy->id, rawBudget)) {
                        return finishPairs(pairs, budget, false);
                    }
                }
            }
        } else {
            std::vector<Proxy> localFirst;
            std::vector<Proxy> localSecond;
            localFirst.reserve(aEnd - a);
            localSecond.reserve(bEnd - b);
            for (std::size_t i = a; i < aEnd; ++i) {
                localFirst.push_back(first[firstCells[i].second]);
            }
            for (std::size_t i = b; i < bEnd; ++i) {
                localSecond.push_back(second[secondCells[i].second]);
            }
            std::size_t visits = 0;
            const bool complete = crossSweep(
                localFirst, localSecond, rawBudget - pairs.size(), visits,
                [&pairs, rawBudget, &eligible](const Proxy &left,
                                               const Proxy &right) {
                    return !eligible(left.id, right.id)
                           || emitPair(pairs, left.id, right.id, rawBudget);
                });
            if (!complete || pairs.size() >= rawBudget) {
                return finishPairs(pairs, budget, false);
            }
        }
        a = aEnd;
        b = bEnd;
    }

    // Oversized primitives were not inserted anywhere: test them explicitly
    // against every compatible proxy. Skip the oversized/oversized duplicate
    // in the second pass.
    for (const std::uint32_t index : firstOversized) {
        const Proxy *left = &first[index];
        for (const Proxy &right : second) {
            if (left && overlaps(left->bounds, right.bounds)
                && eligible(left->id, right.id)
                && !emitPair(pairs, left->id, right.id, rawBudget)) {
                return finishPairs(pairs, budget, false);
            }
        }
    }
    for (std::uint32_t firstIndex = 0; firstIndex < first.size();
         ++firstIndex) {
        const Proxy &left = first[firstIndex];
        if (std::binary_search(firstOversized.begin(),
                               firstOversized.end(), firstIndex)) {
            continue;
        }
        for (const std::uint32_t index : secondOversized) {
            const Proxy *right = &second[index];
            if (right && overlaps(left.bounds, right->bounds)
                && eligible(left.id, right->id)
                && !emitPair(pairs, left.id, right->id, rawBudget)) {
                return finishPairs(pairs, budget, false);
            }
        }
    }
    return finishPairs(pairs, budget, true);
}

bool gridSelfPairs(std::span<const Proxy> proxies,
                   double cellSize,
                   std::size_t budget,
                   std::vector<std::uint64_t> &pairs,
                   std::size_t &oversizedCount,
                   const std::function<bool(std::uint32_t,
                                            std::uint32_t)> &eligible)
{
    std::vector<GridEntry> cells;
    std::vector<std::uint32_t> oversized;
    for (std::uint32_t index = 0; index < proxies.size(); ++index) {
        insertProxyGrid(proxies[index], index, cellSize, cells, oversized);
    }
    oversizedCount += oversized.size();
    std::sort(cells.begin(), cells.end());
    const std::size_t rawBudget =
        budget > std::numeric_limits<std::size_t>::max() / 8
            ? std::numeric_limits<std::size_t>::max()
            : std::max<std::size_t>(budget, 1) * 8;
    pairs.clear();
    std::size_t begin = 0;
    constexpr std::size_t denseProduct = 4096;
    while (begin < cells.size()) {
        std::size_t end = begin + 1;
        while (end < cells.size() && cells[end].first == cells[begin].first) {
            ++end;
        }
        const std::size_t count = end - begin;
        if (count * count <= denseProduct) {
            for (std::size_t i = begin; i < end; ++i) {
                const Proxy *left = &proxies[cells[i].second];
                for (std::size_t j = i + 1; j < end; ++j) {
                    const Proxy *right = &proxies[cells[j].second];
                    if (!left || !right || left->id == right->id
                        || !overlaps(left->bounds, right->bounds)) {
                        continue;
                    }
                    const auto [low, high] = std::minmax(left->id, right->id);
                    if (eligible(low, high)
                        && !emitPair(pairs, low, high, rawBudget)) {
                        return finishPairs(pairs, budget, false);
                    }
                }
            }
        } else {
            std::vector<Proxy> local;
            local.reserve(count);
            for (std::size_t i = begin; i < end; ++i) {
                local.push_back(proxies[cells[i].second]);
            }
            std::size_t visits = 0;
            const bool complete = selfSweep(
                local, rawBudget - pairs.size(), visits,
                [&pairs, rawBudget, &eligible](const Proxy &left,
                                               const Proxy &right) {
                    const auto [low, high] =
                        std::minmax(left.id, right.id);
                    return !eligible(low, high)
                           || emitPair(pairs, low, high, rawBudget);
                });
            if (!complete || pairs.size() >= rawBudget) {
                return finishPairs(pairs, budget, false);
            }
        }
        begin = end;
    }
    for (const std::uint32_t index : oversized) {
        const Proxy *left = &proxies[index];
        for (const Proxy &right : proxies) {
            if (!left || right.id <= left->id
                || !overlaps(left->bounds, right.bounds)) {
                continue;
            }
            if (eligible(left->id, right.id)
                && !emitPair(pairs, left->id, right.id, rawBudget)) {
                return finishPairs(pairs, budget, false);
            }
        }
    }
    return finishPairs(pairs, budget, true);
}

// A global swept-AABB sweep avoids the spatial grid's two historical holes:
// no primitive is too large to insert and there is no dense cell whose pair
// product can be silently skipped. Work is capped explicitly and diagnosed.
template <typename Callback>
bool crossSweep(std::span<const Proxy> first,
                std::span<const Proxy> second,
                std::size_t budget,
                std::size_t &visits,
                Callback &&callback)
{
    struct Event
    {
        double start = 0.0;
        std::uint32_t proxy = 0;
        bool first = false;
    };
    const int axis = crossSweepAxis(first, second);
    std::vector<Event> events;
    events.reserve(first.size() + second.size());
    for (std::uint32_t index = 0; index < first.size(); ++index) {
        events.push_back(
            {coordinate(first[index].bounds.low, axis), index, true});
    }
    for (std::uint32_t index = 0; index < second.size(); ++index) {
        events.push_back(
            {coordinate(second[index].bounds.low, axis), index, false});
    }
    std::sort(events.begin(), events.end(), [](const Event &a,
                                                const Event &b) {
        if (a.start != b.start) {
            return a.start < b.start;
        }
        if (a.first != b.first) {
            return a.first < b.first;
        }
        return a.proxy < b.proxy;
    });
    std::vector<std::uint32_t> activeFirst;
    std::vector<std::uint32_t> activeSecond;
    for (const Event &event : events) {
        const auto prune = [axis, &event](auto &active,
                                          std::span<const Proxy> proxies) {
            active.erase(
                std::remove_if(active.begin(), active.end(),
                               [axis, &event, proxies](std::uint32_t index) {
                    return coordinate(proxies[index].bounds.high, axis)
                           < event.start;
                }),
                active.end());
        };
        prune(activeFirst, first);
        prune(activeSecond, second);
        if (event.first) {
            const Proxy &a = first[event.proxy];
            for (const std::uint32_t index : activeSecond) {
                if (++visits > budget) {
                    return false;
                }
                const Proxy &b = second[index];
                if (overlaps(a.bounds, b.bounds)
                    && !callback(a, b)) {
                    return true;
                }
            }
            activeFirst.push_back(event.proxy);
        } else {
            const Proxy &b = second[event.proxy];
            for (const std::uint32_t index : activeFirst) {
                if (++visits > budget) {
                    return false;
                }
                const Proxy &a = first[index];
                if (overlaps(a.bounds, b.bounds)
                    && !callback(a, b)) {
                    return true;
                }
            }
            activeSecond.push_back(event.proxy);
        }
    }
    return true;
}

template <typename Callback>
bool selfSweep(std::span<const Proxy> proxies,
               std::size_t budget,
               std::size_t &visits,
               Callback &&callback)
{
    const int axis = selfSweepAxis(proxies);
    std::vector<std::uint32_t> order(proxies.size());
    std::iota(order.begin(), order.end(), 0U);
    std::sort(order.begin(), order.end(), [axis, proxies](std::uint32_t a,
                                                          std::uint32_t b) {
        const double first = coordinate(proxies[a].bounds.low, axis);
        const double second = coordinate(proxies[b].bounds.low, axis);
        return first != second ? first < second : a < b;
    });
    std::vector<std::uint32_t> active;
    for (const std::uint32_t current : order) {
        const double start = coordinate(proxies[current].bounds.low, axis);
        active.erase(
            std::remove_if(active.begin(), active.end(),
                           [axis, start, proxies](std::uint32_t index) {
                return coordinate(proxies[index].bounds.high, axis) < start;
            }),
            active.end());
        for (const std::uint32_t previous : active) {
            if (++visits > budget) {
                return false;
            }
            if (overlaps(proxies[previous].bounds,
                         proxies[current].bounds)
                && !callback(proxies[previous], proxies[current])) {
                return true;
            }
        }
        active.push_back(current);
    }
    return true;
}

Vec3 fallbackNormal(PlaygroundContactFeature feature,
                    std::uint32_t first,
                    std::uint32_t second,
                    const PlaygroundContactScratch &scratch,
                    std::span<const Node> nodes,
                    std::span<const Triangle> triangles)
{
    if (feature == PlaygroundContactFeature::VertexTriangle
        || feature == PlaygroundContactFeature::SegmentTriangle) {
        const Triangle &triangle = triangles[second];
        Vec3 normal = cross(nodes[triangle.b].position
                                - nodes[triangle.a].position,
                            nodes[triangle.c].position
                                - nodes[triangle.a].position);
        const double magnitude = length(normal);
        if (magnitude > 1.0e-12) {
            return normal / magnitude;
        }
    } else {
        const PlaygroundContactEdge &a = scratch.skinEdges[first];
        const PlaygroundContactEdge &b = scratch.skinEdges[second];
        Vec3 normal = cross(nodes[a.b].position - nodes[a.a].position,
                            nodes[b.b].position - nodes[b.a].position);
        const double magnitude = length(normal);
        if (magnitude > 1.0e-12) {
            return normal / magnitude;
        }
    }
    return {0.0, 0.0, 1.0};
}

ClosestFeatureResult geometry(const PlaygroundContactCandidate &candidate,
                              const PlaygroundContactScratch &scratch,
                              std::span<const Node> nodes,
                              std::span<const Triangle> triangles)
{
    if (candidate.feature
        == PlaygroundContactFeature::VertexTriangle) {
        const Triangle &triangle = triangles[candidate.second];
        return softwing::closestVertexTriangle(
            nodes[candidate.first].position,
            nodes[triangle.a].position,
            nodes[triangle.b].position,
            nodes[triangle.c].position,
            playgroundFabricContactSeparation);
    }
    if (candidate.feature == PlaygroundContactFeature::EdgeEdge) {
        const PlaygroundContactEdge &first =
            scratch.skinEdges[candidate.first];
        const PlaygroundContactEdge &second =
            scratch.skinEdges[candidate.second];
        return softwing::closestEdgeEdge(
            nodes[first.a].position, nodes[first.b].position,
            nodes[second.a].position, nodes[second.b].position,
            playgroundFabricContactSeparation);
    }
    const PlaygroundContactLine &line =
        scratch.authoredLines[candidate.first];
    const Triangle &triangle = triangles[candidate.second];
    return softwing::closestSegmentTriangle(
        nodes[line.a].position, nodes[line.b].position,
        nodes[triangle.a].position, nodes[triangle.b].position,
        nodes[triangle.c].position, playgroundLineContactSeparation);
}

bool sameKey(const PlaygroundContactCandidate &a,
             const PlaygroundContactCandidate &b)
{
    return a.feature == b.feature && a.first == b.first
           && a.second == b.second;
}

}  // namespace

void preparePlaygroundContact(
    PlaygroundContactScratch &scratch,
    std::span<const Node> nodes,
    std::span<const Triangle> triangles,
    std::size_t skinTriangleCount,
    std::span<const PlaygroundContactLine> authoredLines)
{
    scratch.candidates.clear();
    scratch.skinNodes.clear();
    scratch.skinEdges.clear();
    scratch.oneRingPairs.clear();
    scratch.authoredLines.assign(authoredLines.begin(), authoredLines.end());
    const std::size_t faceCount = std::min(skinTriangleCount,
                                           triangles.size());
    double edgeLength = 0.0;
    std::size_t edgeVisits = 0;
    for (std::size_t face = 0; face < faceCount; ++face) {
        const Triangle &triangle = triangles[face];
        const std::array<std::uint32_t, 3> vertices{
            static_cast<std::uint32_t>(triangle.a),
            static_cast<std::uint32_t>(triangle.b),
            static_cast<std::uint32_t>(triangle.c)};
        scratch.skinNodes.insert(scratch.skinNodes.end(), vertices.begin(),
                                 vertices.end());
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const auto [a, b] = std::minmax(
                vertices[edge], vertices[(edge + 1) % vertices.size()]);
            scratch.skinEdges.push_back({a, b});
            scratch.oneRingPairs.push_back(vertexPairKey(a, b));
            if (a < nodes.size() && b < nodes.size()) {
                edgeLength += length(nodes[b].position - nodes[a].position);
                ++edgeVisits;
            }
        }
    }
    std::sort(scratch.skinNodes.begin(), scratch.skinNodes.end());
    scratch.skinNodes.erase(
        std::unique(scratch.skinNodes.begin(), scratch.skinNodes.end()),
        scratch.skinNodes.end());
    std::sort(scratch.skinEdges.begin(), scratch.skinEdges.end());
    scratch.skinEdges.erase(
        std::unique(scratch.skinEdges.begin(), scratch.skinEdges.end()),
        scratch.skinEdges.end());
    for (const std::uint32_t vertex : scratch.skinNodes) {
        scratch.oneRingPairs.push_back(vertexPairKey(vertex, vertex));
    }
    std::sort(scratch.oneRingPairs.begin(), scratch.oneRingPairs.end());
    scratch.oneRingPairs.erase(
        std::unique(scratch.oneRingPairs.begin(),
                    scratch.oneRingPairs.end()),
        scratch.oneRingPairs.end());
    scratch.meanSkinEdgeLength =
        edgeVisits > 0 ? edgeLength / static_cast<double>(edgeVisits) : 0.05;
    scratch.capturePositions.assign(nodes.size(), {});
    scratch.captureAllowance.assign(nodes.size(), 0.0);
    scratch.stats = {};
    scratch.prepared = true;
}

void beginPlaygroundContactFrame(
    PlaygroundContactScratch &scratch,
    std::span<const Node> nodes,
    std::span<const Triangle> triangles,
    std::size_t skinTriangleCount,
    double remainingFrameSeconds)
{
    scratch.stats = {};
    refreshPlaygroundContact(scratch, nodes, triangles, skinTriangleCount,
                             remainingFrameSeconds);
    // The first capture is not a substep escape refresh.
    scratch.stats.substepRefreshes = 0;
}

void refreshPlaygroundContact(
    PlaygroundContactScratch &scratch,
    std::span<const Node> nodes,
    std::span<const Triangle> triangles,
    std::size_t skinTriangleCount,
    double remainingFrameSeconds)
{
    if (!scratch.prepared) {
        return;
    }
    ++scratch.stats.substepRefreshes;
    scratch.candidates.clear();
    scratch.stats.vertexTriangleCandidates = 0;
    scratch.stats.edgeEdgeCandidates = 0;
    scratch.stats.segmentTriangleCandidates = 0;
    scratch.stats.activeContacts = 0;

    Vec3 meanVelocity;
    for (const std::uint32_t node : scratch.skinNodes) {
        if (node < nodes.size()) {
            meanVelocity += nodes[node].velocity;
        }
    }
    if (!scratch.skinNodes.empty()) {
        meanVelocity = meanVelocity
                       / static_cast<double>(scratch.skinNodes.size());
    }
    const double projectionSlack = std::clamp(
        0.10 * scratch.meanSkinEdgeLength, 0.003, 0.010);
    std::vector<double> margins(nodes.size(), projectionSlack);
    const double seconds = std::max(0.0, remainingFrameSeconds);
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        margins[index] += 1.5 * seconds
                          * length(nodes[index].velocity - meanVelocity);
    }
    scratch.capturePositions.resize(nodes.size());
    scratch.captureAllowance.resize(nodes.size());
    // Frame envelopes already include the velocity-predicted travel. Allow a
    // bounded amount of XPBD projection motion before rebuilding: without
    // this, ordinary pressure relaxation refreshes contact every substep even
    // though no primitive has escaped its useful neighbourhood.
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        scratch.capturePositions[index] = nodes[index].position;
        scratch.captureAllowance[index] = margins[index];
    }

    const double largeThreshold =
        std::max(0.025, 4.0 * scratch.meanSkinEdgeLength);
    scratch.stats.largeSweptEnvelopes = 0;
    const auto noteLarge = [&scratch, largeThreshold](double margin) {
        if (margin > largeThreshold) {
            ++scratch.stats.largeSweptEnvelopes;
        }
    };

    std::vector<Proxy> nodeProxies;
    nodeProxies.reserve(scratch.skinNodes.size());
    for (const std::uint32_t node : scratch.skinNodes) {
        if (node >= nodes.size()) {
            continue;
        }
        const double expansion = 0.5 * playgroundFabricContactSeparation
                                 + margins[node];
        noteLarge(margins[node]);
        nodeProxies.push_back(
            {node, pointBounds(nodes[node].position, expansion),
             margins[node]});
    }

    const std::size_t faceCount = std::min(skinTriangleCount,
                                           triangles.size());
    std::vector<Proxy> triangleProxies;
    triangleProxies.reserve(faceCount);
    for (std::uint32_t face = 0; face < faceCount; ++face) {
        const Triangle &triangle = triangles[face];
        if (triangle.a >= nodes.size() || triangle.b >= nodes.size()
            || triangle.c >= nodes.size()) {
            scratch.stats.coverageComplete = false;
            ++scratch.stats.geometryQueryFailures;
            continue;
        }
        const double margin = std::max(
            {margins[triangle.a], margins[triangle.b], margins[triangle.c]});
        const std::array<std::uint32_t, 3> indices{
            static_cast<std::uint32_t>(triangle.a),
            static_cast<std::uint32_t>(triangle.b),
            static_cast<std::uint32_t>(triangle.c)};
        noteLarge(margin);
        triangleProxies.push_back(
            {face,
             primitiveBounds(nodes, indices,
                             0.5 * playgroundFabricContactSeparation
                                 + margin),
             margin});
    }

    std::vector<Proxy> edgeProxies;
    edgeProxies.reserve(scratch.skinEdges.size());
    for (std::uint32_t edge = 0; edge < scratch.skinEdges.size(); ++edge) {
        const PlaygroundContactEdge &skinEdge = scratch.skinEdges[edge];
        if (skinEdge.a >= nodes.size() || skinEdge.b >= nodes.size()) {
            continue;
        }
        const double margin = std::max(margins[skinEdge.a],
                                       margins[skinEdge.b]);
        const std::array<std::uint32_t, 2> indices{skinEdge.a, skinEdge.b};
        noteLarge(margin);
        edgeProxies.push_back(
            {edge,
             primitiveBounds(nodes, indices,
                             0.5 * playgroundFabricContactSeparation
                                 + margin),
             margin});
    }

    std::vector<Proxy> lineProxies;
    lineProxies.reserve(scratch.authoredLines.size());
    for (std::uint32_t line = 0; line < scratch.authoredLines.size(); ++line) {
        const PlaygroundContactLine &segment = scratch.authoredLines[line];
        if (segment.a >= nodes.size() || segment.b >= nodes.size()) {
            scratch.stats.coverageComplete = false;
            ++scratch.stats.geometryQueryFailures;
            continue;
        }
        const double margin = std::max(margins[segment.a],
                                       margins[segment.b]);
        const std::array<std::uint32_t, 2> indices{segment.a, segment.b};
        noteLarge(margin);
        lineProxies.push_back(
            {line,
             primitiveBounds(nodes, indices,
                             0.5 * playgroundLineContactSeparation
                                 + margin),
             margin});
        scratch.captureAllowance[segment.a] =
            margins[segment.a];
        scratch.captureAllowance[segment.b] =
            margins[segment.b];
    }

    const auto addCandidate = [&scratch, &nodes, &triangles](
                                  PlaygroundContactFeature feature,
                                  std::uint32_t first,
                                  std::uint32_t second,
                                  double capture,
                                  std::size_t &count) {
        PlaygroundContactCandidate candidate;
        candidate.feature = feature;
        candidate.first = first;
        candidate.second = second;
        const ClosestFeatureResult result =
            geometry(candidate, scratch, nodes, triangles);
        if (!result.certified()) {
            ++scratch.stats.geometryQueryFailures;
            scratch.stats.coverageComplete = false;
            return true;
        }
        const Vec3 delta = result.firstPoint - result.secondPoint;
        if (lengthSquared(delta) >= capture * capture) {
            return true;
        }
        if (count >= scratch.limits.maxCandidatesPerFeature) {
            ++scratch.stats.candidateBudgetHits;
            scratch.stats.coverageComplete = false;
            return false;
        }
        const double distance = length(delta);
        candidate.normal =
            distance > 1.0e-12
                ? delta / distance
                : fallbackNormal(feature, first, second, scratch, nodes,
                                 triangles);
        scratch.candidates.push_back(candidate);
        ++count;
        return true;
    };

    const double cellSize =
        std::clamp(2.0 * scratch.meanSkinEdgeLength, 0.02, 0.5);
    std::vector<std::uint64_t> pairs;
    const auto vertexTriangleEligible =
        [&scratch, &triangles](std::uint32_t node, std::uint32_t face) {
            const Triangle &triangle = triangles[face];
            const bool excluded =
                adjacent(scratch, node,
                         static_cast<std::uint32_t>(triangle.a))
                || adjacent(scratch, node,
                            static_cast<std::uint32_t>(triangle.b))
                || adjacent(scratch, node,
                            static_cast<std::uint32_t>(triangle.c));
            if (excluded) {
                ++scratch.stats.topologyExcludedPairs;
            }
            return !excluded;
        };
    bool complete = gridCrossPairs(
        nodeProxies, triangleProxies, cellSize,
        scratch.limits.maxBroadPhaseTestsPerFeature, pairs,
        scratch.stats.largeSweptEnvelopes, vertexTriangleEligible);
    scratch.stats.broadPhaseTests += pairs.size();
    if (!complete) {
        ++scratch.stats.broadPhaseBudgetHits;
        scratch.stats.coverageComplete = false;
    }
    for (const std::uint64_t pair : pairs) {
        const Proxy *node = proxyWithId(
            nodeProxies, static_cast<std::uint32_t>(pair >> 32));
        const Proxy *face = proxyWithId(
            triangleProxies, static_cast<std::uint32_t>(pair));
        if (!node || !face) {
            continue;
        }
        const Triangle &triangle = triangles[face->id];
        if (adjacent(scratch, node->id,
                         static_cast<std::uint32_t>(triangle.a))
                || adjacent(scratch, node->id,
                            static_cast<std::uint32_t>(triangle.b))
                || adjacent(scratch, node->id,
                            static_cast<std::uint32_t>(triangle.c))) {
            ++scratch.stats.topologyExcludedPairs;
            continue;
        }
        if (!addCandidate(
                PlaygroundContactFeature::VertexTriangle, node->id, face->id,
                playgroundFabricContactSeparation + node->margin
                    + face->margin,
                scratch.stats.vertexTriangleCandidates)) {
            break;
        }
    }

    const auto edgeEligible = [&scratch](std::uint32_t first,
                                         std::uint32_t second) {
        const PlaygroundContactEdge &a = scratch.skinEdges[first];
        const PlaygroundContactEdge &b = scratch.skinEdges[second];
        const bool excluded = adjacent(scratch, a.a, b.a)
                              || adjacent(scratch, a.a, b.b)
                              || adjacent(scratch, a.b, b.a)
                              || adjacent(scratch, a.b, b.b);
        if (excluded) {
            ++scratch.stats.topologyExcludedPairs;
        }
        return !excluded;
    };
    complete = gridSelfPairs(
        edgeProxies, cellSize,
        scratch.limits.maxBroadPhaseTestsPerFeature, pairs,
        scratch.stats.largeSweptEnvelopes, edgeEligible);
    scratch.stats.broadPhaseTests += pairs.size();
    if (!complete) {
        ++scratch.stats.broadPhaseBudgetHits;
        scratch.stats.coverageComplete = false;
    }
    for (const std::uint64_t pair : pairs) {
        const Proxy *first = proxyWithId(
            edgeProxies, static_cast<std::uint32_t>(pair >> 32));
        const Proxy *second = proxyWithId(
            edgeProxies, static_cast<std::uint32_t>(pair));
        if (!first || !second) {
            continue;
        }
        const PlaygroundContactEdge &a = scratch.skinEdges[first->id];
        const PlaygroundContactEdge &b = scratch.skinEdges[second->id];
        if (adjacent(scratch, a.a, b.a)
                || adjacent(scratch, a.a, b.b)
                || adjacent(scratch, a.b, b.a)
                || adjacent(scratch, a.b, b.b)) {
            ++scratch.stats.topologyExcludedPairs;
            continue;
        }
        if (!addCandidate(
                PlaygroundContactFeature::EdgeEdge, first->id, second->id,
                playgroundFabricContactSeparation + first->margin
                    + second->margin,
                scratch.stats.edgeEdgeCandidates)) {
            break;
        }
    }

    const auto lineTriangleEligible =
        [&scratch, &triangles](std::uint32_t line, std::uint32_t face) {
            const PlaygroundContactLine &segment =
                scratch.authoredLines[line];
            const Triangle &triangle = triangles[face];
            const bool excluded =
                adjacent(scratch, segment.a,
                         static_cast<std::uint32_t>(triangle.a))
                || adjacent(scratch, segment.a,
                            static_cast<std::uint32_t>(triangle.b))
                || adjacent(scratch, segment.a,
                            static_cast<std::uint32_t>(triangle.c))
                || adjacent(scratch, segment.b,
                            static_cast<std::uint32_t>(triangle.a))
                || adjacent(scratch, segment.b,
                            static_cast<std::uint32_t>(triangle.b))
                || adjacent(scratch, segment.b,
                            static_cast<std::uint32_t>(triangle.c));
            if (excluded) {
                ++scratch.stats.topologyExcludedPairs;
            }
            return !excluded;
        };
    complete = gridCrossPairs(
        lineProxies, triangleProxies, cellSize,
        scratch.limits.maxBroadPhaseTestsPerFeature, pairs,
        scratch.stats.largeSweptEnvelopes, lineTriangleEligible);
    scratch.stats.broadPhaseTests += pairs.size();
    if (!complete) {
        ++scratch.stats.broadPhaseBudgetHits;
        scratch.stats.coverageComplete = false;
    }
    for (const std::uint64_t pair : pairs) {
        const Proxy *line = proxyWithId(
            lineProxies, static_cast<std::uint32_t>(pair >> 32));
        const Proxy *face = proxyWithId(
            triangleProxies, static_cast<std::uint32_t>(pair));
        if (!line || !face) {
            continue;
        }
        const PlaygroundContactLine &segment =
            scratch.authoredLines[line->id];
        const Triangle &triangle = triangles[face->id];
        if (adjacent(scratch, segment.a,
                         static_cast<std::uint32_t>(triangle.a))
                || adjacent(scratch, segment.a,
                            static_cast<std::uint32_t>(triangle.b))
                || adjacent(scratch, segment.a,
                            static_cast<std::uint32_t>(triangle.c))
                || adjacent(scratch, segment.b,
                            static_cast<std::uint32_t>(triangle.a))
                || adjacent(scratch, segment.b,
                            static_cast<std::uint32_t>(triangle.b))
                || adjacent(scratch, segment.b,
                            static_cast<std::uint32_t>(triangle.c))) {
            ++scratch.stats.topologyExcludedPairs;
            continue;
        }
        if (!addCandidate(
                PlaygroundContactFeature::SegmentTriangle, line->id, face->id,
                playgroundLineContactSeparation + line->margin + face->margin,
                scratch.stats.segmentTriangleCandidates)) {
            break;
        }
    }

    std::sort(scratch.candidates.begin(), scratch.candidates.end());
    scratch.candidates.erase(
        std::unique(scratch.candidates.begin(), scratch.candidates.end(),
                    sameKey),
        scratch.candidates.end());
}

bool playgroundContactEnvelopeEscaped(
    const PlaygroundContactScratch &scratch,
    std::span<const Node> nodes)
{
    if (!scratch.prepared || scratch.capturePositions.size() != nodes.size()
        || scratch.captureAllowance.size() != nodes.size()) {
        return true;
    }
    const auto escaped = [&scratch, nodes](std::uint32_t node) {
        return node >= nodes.size()
               || lengthSquared(nodes[node].position
                                - scratch.capturePositions[node])
                      > scratch.captureAllowance[node]
                            * scratch.captureAllowance[node];
    };
    for (const std::uint32_t node : scratch.skinNodes) {
        if (escaped(node)) {
            return true;
        }
    }
    for (const PlaygroundContactLine &line : scratch.authoredLines) {
        if (escaped(line.a) || escaped(line.b)) {
            return true;
        }
    }
    return false;
}

void projectPlaygroundContact(
    PlaygroundContactScratch &scratch,
    std::span<Node> nodes,
    std::span<const Triangle> triangles)
{
    scratch.stats.activeContacts = 0;
    scratch.stats.worstPenetrationBefore = 0.0;
    scratch.stats.worstPenetrationAfter = 0.0;
    for (const PlaygroundContactCandidate &candidate : scratch.candidates) {
        const ClosestFeatureResult result =
            geometry(candidate, scratch, nodes, triangles);
        ++scratch.stats.projectionVisits;
        if (!result.certified()) {
            ++scratch.stats.geometryQueryFailures;
            scratch.stats.coverageComplete = false;
            continue;
        }
        Vec3 direction = candidate.normal;
        const double normalLength = length(direction);
        if (!(normalLength > 1.0e-12)) {
            continue;
        }
        direction = direction / normalLength;
        const double separation =
            candidate.feature == PlaygroundContactFeature::SegmentTriangle
                ? playgroundLineContactSeparation
                : playgroundFabricContactSeparation;
        const double signedDistance =
            dot(result.firstPoint - result.secondPoint, direction);
        const double penetration = separation - signedDistance;
        if (!(penetration > 0.0)) {
            continue;
        }
        ++scratch.stats.activeContacts;
        scratch.stats.worstPenetrationBefore =
            std::max(scratch.stats.worstPenetrationBefore, penetration);

        std::array<std::uint32_t, 3> firstNodes{};
        std::array<std::uint32_t, 3> secondNodes{};
        std::size_t firstCount = 0;
        std::size_t secondCount = 0;
        if (candidate.feature
            == PlaygroundContactFeature::VertexTriangle) {
            firstNodes[0] = candidate.first;
            firstCount = 1;
            const Triangle &triangle = triangles[candidate.second];
            secondNodes = {static_cast<std::uint32_t>(triangle.a),
                           static_cast<std::uint32_t>(triangle.b),
                           static_cast<std::uint32_t>(triangle.c)};
            secondCount = 3;
        } else if (candidate.feature
                   == PlaygroundContactFeature::EdgeEdge) {
            const PlaygroundContactEdge &first =
                scratch.skinEdges[candidate.first];
            const PlaygroundContactEdge &second =
                scratch.skinEdges[candidate.second];
            firstNodes = {first.a, first.b, 0};
            secondNodes = {second.a, second.b, 0};
            firstCount = 2;
            secondCount = 2;
        } else {
            const PlaygroundContactLine &line =
                scratch.authoredLines[candidate.first];
            const Triangle &triangle = triangles[candidate.second];
            firstNodes = {line.a, line.b, 0};
            secondNodes = {static_cast<std::uint32_t>(triangle.a),
                           static_cast<std::uint32_t>(triangle.b),
                           static_cast<std::uint32_t>(triangle.c)};
            firstCount = 2;
            secondCount = 3;
        }

        double denominator = 0.0;
        Vec3 firstVelocity;
        Vec3 secondVelocity;
        for (std::size_t index = 0; index < firstCount; ++index) {
            const double weight = result.firstWeights[index];
            denominator += nodes[firstNodes[index]].inverseMass
                           * weight * weight;
            firstVelocity += weight * nodes[firstNodes[index]].velocity;
        }
        for (std::size_t index = 0; index < secondCount; ++index) {
            const double weight = result.secondWeights[index];
            denominator += nodes[secondNodes[index]].inverseMass
                           * weight * weight;
            secondVelocity += weight * nodes[secondNodes[index]].velocity;
        }
        if (!(denominator > 0.0)) {
            continue;
        }
        // Once the retained side says the features crossed, a millimetre cap
        // would leave them on the wrong side at ordinary collapse speeds.
        // Return the full signed crossing in one projection; only a same-side
        // overlap is relaxed gradually.
        const double correction =
            signedDistance < 0.0
                ? penetration
                : std::min(penetration,
                           scratch.limits.maximumCorrectionMetres);
        const double positionScale = correction / denominator;
        for (std::size_t index = 0; index < firstCount; ++index) {
            Node &node = nodes[firstNodes[index]];
            node.position += positionScale * result.firstWeights[index]
                             * node.inverseMass * direction;
        }
        for (std::size_t index = 0; index < secondCount; ++index) {
            Node &node = nodes[secondNodes[index]];
            node.position -= positionScale * result.secondWeights[index]
                             * node.inverseMass * direction;
        }

        const double closing = dot(firstVelocity - secondVelocity, direction);
        if (closing < 0.0) {
            const double impulse = -closing / denominator;
            for (std::size_t index = 0; index < firstCount; ++index) {
                Node &node = nodes[firstNodes[index]];
                node.velocity += impulse * result.firstWeights[index]
                                 * node.inverseMass * direction;
            }
            for (std::size_t index = 0; index < secondCount; ++index) {
                Node &node = nodes[secondNodes[index]];
                node.velocity -= impulse * result.secondWeights[index]
                                 * node.inverseMass * direction;
            }
        }

        const ClosestFeatureResult after =
            geometry(candidate, scratch, nodes, triangles);
        if (after.certified()) {
            const double afterPenetration =
                std::max(0.0,
                         separation
                             - dot(after.firstPoint - after.secondPoint,
                                   direction));
            scratch.stats.worstPenetrationAfter =
                std::max(scratch.stats.worstPenetrationAfter,
                         afterPenetration);
        }
    }
}

}  // namespace lep::playground
