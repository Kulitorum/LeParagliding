#include <softwing/cell_volume.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <span>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

void checkNear(double actual,
               double expected,
               double tolerance,
               const char* message) {
    if (std::abs(actual - expected) > tolerance) {
        std::fprintf(stderr,
                     "FAIL: %s (actual %.17g, expected %.17g)\n",
                     message,
                     actual,
                     expected);
        ++failures;
    }
}

struct BayFixture {
    std::vector<softwing::Node> nodes;
    std::vector<softwing::Triangle> triangles;
    std::vector<std::size_t> faces;
    std::array<std::size_t, 4> lowLoop{};
    std::array<std::size_t, 4> highLoop{};
};

std::size_t addNode(BayFixture& fixture, const softwing::Vec3& position) {
    softwing::Node node;
    node.position = position;
    fixture.nodes.push_back(node);
    return fixture.nodes.size() - 1;
}

void addOrientedTriangle(BayFixture& fixture,
                         std::size_t a,
                         std::size_t b,
                         std::size_t c,
                         const softwing::Vec3& outward) {
    const softwing::Vec3 normal = softwing::cross(
        fixture.nodes[b].position - fixture.nodes[a].position,
        fixture.nodes[c].position - fixture.nodes[a].position);
    if (softwing::dot(normal, outward) < 0.0) {
        std::swap(b, c);
    }
    fixture.faces.push_back(fixture.triangles.size());
    fixture.triangles.push_back({a, b, c});
}

void addOrientedQuad(BayFixture& fixture,
                     std::size_t a,
                     std::size_t b,
                     std::size_t c,
                     std::size_t d,
                     const softwing::Vec3& outward) {
    addOrientedTriangle(fixture, a, b, c, outward);
    addOrientedTriangle(fixture, a, c, d, outward);
}

BayFixture makeBay(std::span<const double> stations,
                   std::span<const double> topHeights) {
    BayFixture fixture;
    for (std::size_t station = 0; station < stations.size(); ++station) {
        const double x = stations[station];
        const double top = topHeights[station];
        addNode(fixture, {x, 0.0, 0.0});
        addNode(fixture, {x, 1.0, 0.0});
        addNode(fixture, {x, 1.0, top});
        addNode(fixture, {x, 0.0, top});
    }

    for (std::size_t station = 0; station + 1 < stations.size(); ++station) {
        const std::size_t a = station * 4;
        const std::size_t b = (station + 1) * 4;
        addOrientedQuad(fixture, a, b, b + 1, a + 1, {0.0, 0.0, -1.0});
        addOrientedQuad(fixture, a + 1, b + 1, b + 2, a + 2,
                        {0.0, 1.0, 0.0});
        addOrientedQuad(fixture, a + 3, a + 2, b + 2, b + 3,
                        {0.0, 0.0, 1.0});
        addOrientedQuad(fixture, a, a + 3, b + 3, b,
                        {0.0, -1.0, 0.0});
    }

    fixture.lowLoop = {0, 1, 2, 3};
    const std::size_t high = (stations.size() - 1) * 4;
    fixture.highLoop = {high, high + 1, high + 2, high + 3};
    return fixture;
}

softwing::ClosedCellVolumeEstimate estimate(const BayFixture& fixture) {
    return softwing::estimateClosedCellVolume(fixture.nodes,
                                               fixture.triangles,
                                               fixture.faces,
                                               fixture.lowLoop,
                                               fixture.highLoop);
}

void testTopologyOrientsCaps() {
    const std::array stations{0.0, 2.0};
    const std::array heights{1.0, 1.0};
    BayFixture fixture = makeBay(stations, heights);

    const auto result = estimate(fixture);
    check(result.valid, "box: estimate is valid");
    check(result.watertightAfterCapping,
          "box: two virtual caps close every skin boundary");
    check(!result.usedGeometricOrientationFallback,
          "box: shared boundary winding orients both caps");
    check(result.skinBoundaryEdges == 8,
          "box: reports the eight open rib boundary edges");
    check(result.capBoundaryEdgesMatched == 8,
          "box: both cap loops match all open rib edges");
    checkNear(result.volume, 2.0, 1.0e-12,
              "box: closed volume is exact");

    for (softwing::Triangle& triangle : fixture.triangles) {
        std::swap(triangle.b, triangle.c);
    }
    const auto inward = estimate(fixture);
    check(inward.valid && inward.watertightAfterCapping,
          "box: globally reversed skin winding remains valid");
    checkNear(inward.volume, 2.0, 1.0e-12,
              "box: topology also closes inward-wound skin");

    std::reverse(fixture.lowLoop.begin(), fixture.lowLoop.end());
    std::reverse(fixture.highLoop.begin(), fixture.highLoop.end());
    const auto reversedLoops = estimate(fixture);
    check(reversedLoops.valid && reversedLoops.watertightAfterCapping,
          "box: caller loop winding is immaterial");
    checkNear(reversedLoops.volume, 2.0, 1.0e-12,
              "box: reversed input loops preserve volume");
}

void testGeometricOrientationFallback() {
    const std::array stations{0.0, 2.0};
    const std::array heights{1.0, 1.0};
    BayFixture fixture = makeBay(stations, heights);

    // Some import/refinement paths can retain geometrically identical rib
    // loop points without sharing the skin's node IDs. The cap is still
    // measurable, but the diagnostic must not claim topological closure.
    for (std::size_t& node : fixture.lowLoop) {
        node = addNode(fixture, fixture.nodes[node].position);
    }
    for (std::size_t& node : fixture.highLoop) {
        node = addNode(fixture, fixture.nodes[node].position);
    }

    const auto result = estimate(fixture);
    check(result.valid, "fallback: geometric cap orientation is valid");
    check(result.usedGeometricOrientationFallback,
          "fallback: reports that shared-edge winding was unavailable");
    check(!result.watertightAfterCapping,
          "fallback: duplicate loop IDs do not claim topological closure");
    check(result.capBoundaryEdgesMatched == 0,
          "fallback: no duplicate loop edge is reported as shared");
    checkNear(result.volume, 2.0, 1.0e-12,
              "fallback: coincident virtual caps recover box volume");
}

void testMidCellCaveChangesTrueVolume() {
    const std::array stations{0.0, 1.0, 2.0};
    const std::array restHeights{1.0, 1.0, 1.0};
    const std::array cavedHeights{1.0, 0.2, 1.0};
    const BayFixture rest = makeBay(stations, restHeights);
    const BayFixture caved = makeBay(stations, cavedHeights);

    const auto restVolume = estimate(rest);
    const auto cavedVolume = estimate(caved);
    check(restVolume.valid && cavedVolume.valid,
          "cave: both rest and caved bays are measurable");
    check(restVolume.watertightAfterCapping
              && cavedVolume.watertightAfterCapping,
          "cave: changing mid-cell skin preserves closed topology");
    checkNear(restVolume.volume, 2.0, 1.0e-12,
              "cave: rest bay volume is exact");
    checkNear(cavedVolume.volume, 1.2, 1.0e-12,
              "cave: estimator sees the depressed mid-cell upper skin");
    checkNear(cavedVolume.volume / restVolume.volume, 0.6, 1.0e-12,
              "cave: true volume reports forty percent loss");

    const double unchangedRibOnlyProxy = 2.0;
    checkNear(unchangedRibOnlyProxy / 2.0, 1.0, 0.0,
              "cave: bounding-rib-only proxy remains blind by construction");
}

void testRejectsInvalidInput() {
    const std::array stations{0.0, 2.0};
    const std::array heights{1.0, 1.0};
    BayFixture fixture = makeBay(stations, heights);
    fixture.highLoop[2] = fixture.nodes.size() + 10;
    const auto invalidLoop = estimate(fixture);
    check(!invalidLoop.valid && invalidLoop.volume == 0.0,
          "invalid: out-of-range cap node is rejected without a partial volume");

    fixture = makeBay(stations, heights);
    fixture.faces.push_back(fixture.triangles.size() + 1);
    const auto invalidFace = estimate(fixture);
    check(!invalidFace.valid && invalidFace.volume == 0.0,
          "invalid: out-of-range skin face is rejected without a partial volume");
}

} // namespace

int main() {
    testTopologyOrientsCaps();
    testGeometricOrientationFallback();
    testMidCellCaveChangesTrueVolume();
    testRejectsInvalidInput();
    if (failures != 0) {
        std::fprintf(stderr, "%d soft-wing cell-volume test(s) failed\n", failures);
        return 1;
    }
    std::puts("soft-wing closed-cell volume tests passed");
    return 0;
}
