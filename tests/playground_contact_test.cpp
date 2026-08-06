// Deterministic, Qt-free tests for the Playground's bounded cloth contact
// adapter. These exercise all supported feature families, topology filtering,
// envelope refresh and honest budget diagnostics without building a whole
// wing or registering SoftBody's exhaustive contact pipeline.

#include "playground_contact.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace pg = lep::playground;
using softwing::Node;
using softwing::Triangle;
using softwing::Vec3;

namespace {

int failures = 0;

void check(bool condition, const char *what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

Node node(Vec3 position, double inverseMass = 1.0)
{
    Node result;
    result.position = position;
    result.previousPosition = position;
    result.inverseMass = inverseMass;
    return result;
}

struct ContactFixture
{
    std::vector<Node> nodes;
    std::vector<Triangle> triangles;
    std::vector<pg::PlaygroundContactLine> lines;
    pg::PlaygroundContactScratch scratch;

    void prepare()
    {
        pg::preparePlaygroundContact(scratch, nodes, triangles,
                                     triangles.size(), lines);
    }

    void detect(double seconds = 1.0 / 60.0)
    {
        pg::beginPlaygroundContactFrame(scratch, nodes, triangles,
                                        triangles.size(), seconds);
    }

    void project()
    {
        pg::projectPlaygroundContact(scratch, nodes, triangles);
    }
};

ContactFixture parallelTriangles(double gap)
{
    ContactFixture fixture;
    fixture.nodes = {
        node({-0.4, -0.3, 0.0}, 0.0),
        node({0.4, -0.3, 0.0}, 0.0),
        node({0.0, 0.4, 0.0}, 0.0),
        node({-0.3, -0.2, gap}),
        node({0.3, -0.2, gap}),
        node({0.0, 0.3, gap}),
    };
    fixture.triangles = {{0, 1, 2, 0.0}, {3, 4, 5, 0.0}};
    return fixture;
}

void testVertexTriangleProjectionAndVelocity()
{
    ContactFixture fixture = parallelTriangles(0.0003);
    for (std::size_t index = 3; index < 6; ++index) {
        fixture.nodes[index].velocity = {0.0, 0.0, -2.0};
    }
    fixture.prepare();
    fixture.detect();
    check(fixture.scratch.stats.vertexTriangleCandidates > 0,
          "vertex-triangle candidates are generated");
    const double before = fixture.nodes[3].position.z;
    fixture.project();
    check(fixture.scratch.stats.activeContacts > 0,
          "vertex-triangle overlap becomes active");
    check(fixture.nodes[3].position.z > before,
          "mass-weighted projection moves the free sheet out");
    check(fixture.scratch.stats.worstPenetrationAfter
              < fixture.scratch.stats.worstPenetrationBefore,
          "projection improves the measured worst penetration");
    check(fixture.nodes[3].velocity.z > -1.0e-10,
          "normal closing velocity is removed");
}

void testUnrelatedRestCloseSheetsStillCollide()
{
    ContactFixture fixture = parallelTriangles(0.0005);
    fixture.prepare();
    fixture.detect(0.0);
    check(fixture.scratch.stats.vertexTriangleCandidates > 0,
          "unrelated sheets close in the rest pose are not excluded");
    fixture.project();
    check(fixture.scratch.stats.activeContacts > 0,
          "unrelated rest-close sheets are projected apart");
}

void testTopologyExclusions()
{
    ContactFixture fixture;
    fixture.nodes = {node({0.0, 0.0, 0.0}), node({1.0, 0.0, 0.0}),
                     node({0.0, 1.0, 0.0}), node({1.0, 1.0, 0.0})};
    fixture.triangles = {{0, 1, 2, 0.0}, {1, 3, 2, 0.0}};
    fixture.prepare();
    fixture.detect(0.0);
    check(fixture.scratch.candidates.empty(),
          "incident and one-ring skin features are excluded");
    check(fixture.scratch.stats.topologyExcludedPairs > 0,
          "topology exclusions are diagnosed");

    ContactFixture attached = parallelTriangles(0.01);
    attached.lines.push_back({0, 6});
    attached.nodes.push_back(node({0.0, 0.0, -0.2}));
    attached.prepare();
    attached.detect(0.0);
    const bool attachedPair = std::any_of(
        attached.scratch.candidates.begin(),
        attached.scratch.candidates.end(), [](const auto &candidate) {
            return candidate.feature
                       == pg::PlaygroundContactFeature::SegmentTriangle
                   && candidate.first == 0 && candidate.second == 0;
        });
    check(!attachedPair,
          "authored suspension attachment adjacency is excluded");
}

void testEdgeOnlyCrossing()
{
    ContactFixture fixture;
    // The long edges cross near the origin while all four endpoints stay far
    // outside the opposing triangle, so edge-edge is the relevant feature.
    fixture.nodes = {
        node({-1.0, 0.0, 0.0}), node({1.0, 0.0, 0.0}),
        node({-1.0, -0.8, 0.0}),
        node({0.0, -1.0, 0.0002}), node({0.0, 1.0, 0.0002}),
        node({0.8, -1.0, 0.0002}),
    };
    fixture.triangles = {{0, 1, 2, 0.0}, {3, 4, 5, 0.0}};
    fixture.prepare();
    fixture.detect(0.0);
    check(fixture.scratch.stats.edgeEdgeCandidates > 0,
          "edge-only crossing produces an edge-edge candidate");
    fixture.project();
    check(fixture.scratch.stats.activeContacts > 0,
          "edge-edge contact projects the crossing");
}

void testSuspensionSegmentTriangle()
{
    ContactFixture fixture;
    fixture.nodes = {
        node({-0.5, -0.5, 0.0}, 0.0),
        node({0.5, -0.5, 0.0}, 0.0),
        node({0.0, 0.5, 0.0}, 0.0),
        node({0.0, 0.0, -0.1}), node({0.0, 0.0, 0.1}),
    };
    fixture.triangles = {{0, 1, 2, 0.0}};
    fixture.lines = {{3, 4}};
    fixture.prepare();
    fixture.detect(0.0);
    check(fixture.scratch.stats.segmentTriangleCandidates == 1,
          "authored suspension segment through fabric is detected");
    fixture.project();
    check(fixture.scratch.stats.activeContacts == 1,
          "segment-triangle contact is projected");
}

void testHighSpeedCaptureAndEscapeRefresh()
{
    ContactFixture fixture = parallelTriangles(0.05);
    for (std::size_t index = 3; index < 6; ++index) {
        fixture.nodes[index].velocity = {0.0, 0.0, -10.0};
    }
    fixture.prepare();
    fixture.detect();
    check(fixture.scratch.stats.vertexTriangleCandidates > 0,
          "swept envelope captures a high-speed approach");
    for (std::size_t index = 3; index < 6; ++index) {
        fixture.nodes[index].position.z = -0.01;
    }
    fixture.project();
    check(fixture.nodes[3].position.z > 0.0009,
          "retained approach side returns a high-speed crossing");

    ContactFixture escaped = parallelTriangles(0.10);
    escaped.prepare();
    escaped.detect();
    check(escaped.scratch.candidates.empty(),
          "stationary distant sheets start with no candidates");
    for (std::size_t index = 3; index < 6; ++index) {
        escaped.nodes[index].position.z = 0.0004;
    }
    check(pg::playgroundContactEnvelopeEscaped(escaped.scratch,
                                                escaped.nodes),
          "unexpected substep motion escapes the cached envelope");
    pg::refreshPlaygroundContact(escaped.scratch, escaped.nodes,
                                 escaped.triangles,
                                 escaped.triangles.size(), 0.0);
    check(escaped.scratch.stats.substepRefreshes == 1,
          "escape triggers a diagnosed substep refresh");
    check(!escaped.scratch.candidates.empty(),
          "refresh captures the newly close features");
}

void testBudgetDiagnostics()
{
    ContactFixture fixture;
    for (int triangle = 0; triangle < 8; ++triangle) {
        const double offset = 1.0e-5 * triangle;
        const std::size_t first = fixture.nodes.size();
        fixture.nodes.push_back(node({-0.3, -0.3, offset}));
        fixture.nodes.push_back(node({0.3, -0.3, offset}));
        fixture.nodes.push_back(node({0.0, 0.3, offset}));
        fixture.triangles.push_back(
            {first, first + 1, first + 2, 0.0});
    }
    fixture.scratch.limits.maxBroadPhaseTestsPerFeature = 2;
    fixture.prepare();
    fixture.detect(0.0);
    check(!fixture.scratch.stats.coverageComplete,
          "a broad-phase budget hit honestly marks coverage incomplete");
    check(fixture.scratch.stats.broadPhaseBudgetHits > 0,
          "broad-phase overflow is diagnosed");

    ContactFixture candidates = parallelTriangles(0.0001);
    candidates.scratch.limits.maxCandidatesPerFeature = 1;
    candidates.prepare();
    candidates.detect(0.0);
    check(!candidates.scratch.stats.coverageComplete,
          "a candidate cap honestly marks coverage incomplete");
    check(candidates.scratch.stats.candidateBudgetHits > 0,
          "candidate overflow is diagnosed");
}

void testRepeatDeterminism()
{
    ContactFixture first = parallelTriangles(0.0004);
    ContactFixture second = parallelTriangles(0.0004);
    first.prepare();
    second.prepare();
    first.detect(0.0);
    second.detect(0.0);
    check(first.scratch.candidates.size()
              == second.scratch.candidates.size(),
          "repeat candidate counts are deterministic");
    bool same = first.scratch.candidates.size()
                == second.scratch.candidates.size();
    for (std::size_t index = 0;
         same && index < first.scratch.candidates.size(); ++index) {
        const auto &a = first.scratch.candidates[index];
        const auto &b = second.scratch.candidates[index];
        same = a.feature == b.feature && a.first == b.first
               && a.second == b.second && a.normal.x == b.normal.x
               && a.normal.y == b.normal.y && a.normal.z == b.normal.z;
    }
    check(same, "repeat candidate keys and sides are bit-identical");
    first.project();
    second.project();
    bool positionsSame = first.nodes.size() == second.nodes.size();
    for (std::size_t index = 0;
         positionsSame && index < first.nodes.size(); ++index) {
        positionsSame = first.nodes[index].position.x
                            == second.nodes[index].position.x
                        && first.nodes[index].position.y
                               == second.nodes[index].position.y
                        && first.nodes[index].position.z
                               == second.nodes[index].position.z;
    }
    check(positionsSame, "repeat projected positions are bit-identical");
}

}  // namespace

int main()
{
    testVertexTriangleProjectionAndVelocity();
    testUnrelatedRestCloseSheetsStillCollide();
    testTopologyExclusions();
    testEdgeOnlyCrossing();
    testSuspensionSegmentTriangle();
    testHighSpeedCaptureAndEscapeRefresh();
    testBudgetDiagnostics();
    testRepeatDeterminism();
    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("playground contact: all checks passed\n");
    return 0;
}
