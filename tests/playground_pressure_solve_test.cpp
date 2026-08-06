#include "playground_pressure_solve.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace pg = lep::playground;

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

bool near(double a, double b, double tolerance = 1.0e-8)
{
    return std::abs(a - b) <= tolerance;
}

pg::BoundedEqualityProblem problem(std::size_t variables,
                                   std::size_t rows)
{
    pg::BoundedEqualityProblem value;
    value.variableCount = variables;
    value.rowCount = rows;
    value.prior.assign(variables, 0.0);
    value.lower.assign(variables, -3.0);
    value.upper.assign(variables, 1.0);
    value.mobility.assign(variables, 1.0);
    value.matrix.assign(variables * rows, 0.0);
    value.target.assign(rows, 0.0);
    value.stageRowCounts = {rows, 0, 0};
    return value;
}

void testAnalyticFeasible()
{
    auto p = problem(2, 1);
    p.matrix = {1.0, 1.0};
    p.target = {1.0};
    const auto r = pg::solveBoundedEqualityHierarchy(p);
    check(r.valid && !r.numericalFailure, "analytic: valid");
    check(near(r.value[0], 0.5) && near(r.value[1], 0.5),
          "analytic: weighted projection");
    check(near(r.authority[0], 1.0), "analytic: full authority");
}

void testActiveBoundAndImpossibleAuthority()
{
    auto active = problem(2, 1);
    active.matrix = {1.0, 1.0};
    active.target = {1.0};
    active.upper[0] = 0.25;
    const auto feasible = pg::solveBoundedEqualityHierarchy(active);
    check(feasible.valid && near(feasible.value[0], 0.25)
              && near(feasible.value[1], 0.75),
          "bounds: active upper still meets equality");
    check(feasible.activeUpper == 1, "bounds: active count");

    auto impossible = problem(2, 1);
    impossible.matrix = {1.0, 1.0};
    impossible.target = {2.0};
    impossible.upper = {0.75, 0.75};
    const auto limited = pg::solveBoundedEqualityHierarchy(impossible);
    check(limited.valid && !limited.numericalFailure,
          "authority: physical saturation is not numerical failure");
    check(near(limited.achieved[0], 1.5, 1.0e-7),
          "authority: reports achieved bounded target");
    check(near(limited.authority[0], 0.75, 2.0e-5),
          "authority: reports fractional force authority");
}

void testKktReleaseAtPreferredBound()
{
    auto p = problem(2, 1);
    p.prior = {3.0, 0.0};
    p.lower = {0.0, 0.0};
    p.upper = {2.0, 2.0};
    p.matrix = {1.0, 1.0};
    p.target = {2.0};
    const auto r = pg::solveBoundedEqualityHierarchy(p);
    check(r.valid && near(r.value[0], 2.0)
              && near(r.value[1], 0.0),
          "KKT: preferred out-of-box field projects to (2,0)");
}

void testRankDeficiency()
{
    auto consistent = problem(2, 2);
    consistent.matrix = {1.0, 1.0, 2.0, 2.0};
    consistent.target = {1.0, 2.0};
    const auto solved = pg::solveBoundedEqualityHierarchy(consistent);
    check(solved.valid && solved.rank[0] == 1,
          "rank: consistent dependent rows solve at reported rank");

    auto inconsistent = consistent;
    inconsistent.target[1] = 3.0;
    const auto limited = pg::solveBoundedEqualityHierarchy(inconsistent);
    check(limited.valid && near(limited.authority[0], 0.0, 1.0e-7),
          "rank: inconsistent request receives no fabricated authority");
    check(near(limited.achieved[0], 0.0, 1.0e-7)
              && near(limited.achieved[1], 0.0, 1.0e-7),
          "rank: inconsistent stage retains feasible prior");
}

void testPriorityHierarchy()
{
    auto p = problem(2, 3);
    p.matrix = {
        1.0, 1.0,  // force
        1.0, 0.0,  // pitch proxy
        1.0, -1.0  // differential
    };
    p.target = {1.0, 0.8, 1.0};
    p.stageRowCounts = {1, 1, 1};
    p.upper[0] = 0.8;
    const auto r = pg::solveBoundedEqualityHierarchy(p);
    check(r.valid, "hierarchy: valid");
    check(near(r.achieved[0], 1.0) && near(r.achieved[1], 0.8),
          "hierarchy: force and pitch remain frozen");
    check(near(r.achieved[2], 0.6),
          "hierarchy: differential stops at remaining authority");
    check(near(r.authority[0], 1.0) && near(r.authority[1], 1.0)
              && r.authority[2] < 1.0,
          "hierarchy: only final stage saturates");
}

void testTwoRowDifferentialSharesAuthority()
{
    auto p = problem(4, 4);
    p.prior = {0.0, 0.0, 0.0, 0.0};
    p.lower = {0.0, 0.0, 0.0, 0.0};
    p.upper = {1.0, 1.0, 1.0, 1.0};
    p.matrix = {
        1.0, 1.0, 0.0, 0.0,  // force
        1.0, 0.0, 0.0, 0.0,  // pitch
        0.0, 0.0, 1.0, 0.0,  // L-R lift
        0.0, 0.0, 0.0, 1.0   // L-R drag
    };
    p.target = {1.0, 0.5, 2.0, 4.0};
    p.stageRowCounts = {1, 1, 2};
    const auto r = pg::solveBoundedEqualityHierarchy(p);
    check(r.valid, "two-row differential: valid saturated result");
    check(near(r.achieved[0], 1.0)
              && near(r.achieved[1], 0.5),
          "two-row differential: force and pitch stay frozen");
    check(near(r.authority[2], 0.25, 2.0e-5)
              && near(r.achieved[2], 0.5, 4.0e-5)
              && near(r.achieved[3], 1.0, 4.0e-5),
          "two-row differential: lift and drag use one authority");
}

void testSymmetryDeterminismAndSubdivision()
{
    auto symmetric = problem(2, 2);
    symmetric.matrix = {1.0, 1.0, 1.0, -1.0};
    symmetric.target = {1.0, 0.0};
    symmetric.stageRowCounts = {1, 0, 1};
    const auto first = pg::solveBoundedEqualityHierarchy(symmetric);
    const auto again = pg::solveBoundedEqualityHierarchy(symmetric);
    check(first.value == again.value && first.authority == again.authority,
          "determinism: repeated solve is exact");
    check(near(first.value[0], first.value[1]),
          "symmetry: zero differential stays symmetric");

    auto refined = problem(4, 1);
    refined.matrix = {0.5, 0.5, 0.5, 0.5};
    refined.target = {1.0};
    // Integral objective: half-area variables carry twice the mobility.
    refined.mobility = {2.0, 2.0, 2.0, 2.0};
    const auto split = pg::solveBoundedEqualityHierarchy(refined);
    check(split.valid && near(split.value[0], first.value[0])
              && near(split.value[3], first.value[1]),
          "subdivision: area-weighted projection is invariant");

    std::reverse(refined.prior.begin(), refined.prior.end());
    const auto reversed = pg::solveBoundedEqualityHierarchy(refined);
    check(reversed.value == split.value,
          "determinism: equivalent reversed inputs agree");
}

void testCachedAuthorityGrowthBackoffAndDeterminism()
{
    auto growing = problem(2, 1);
    growing.matrix = {1.0, 1.0};
    growing.target = {2.0};
    growing.upper = {0.75, 0.75};
    growing.authorityMode = pg::AuthoritySolveMode::CachedProbe;
    growing.authorityHint = {0.0, 0.0, 0.0};
    double priorAuthority = 0.0;
    for (int frame = 0; frame < 5; ++frame) {
        const auto result = pg::solveBoundedEqualityHierarchy(growing);
        check(result.valid && !result.numericalFailure,
              "cache: growth solve remains valid");
        check(result.authority[0] > priorAuthority
                  && near(result.authority[0], priorAuthority + 0.02),
              "cache: one verified probe grows authority monotonically");
        check(result.authorityProbeAccepted[0]
                  && result.authorityBackoffs[0] == 0,
              "cache: feasible growth records its probe");
        priorAuthority = result.authority[0];
        growing.authorityHint[0] = priorAuthority;
    }

    auto changed = problem(2, 1);
    changed.matrix = {1.0, 1.0};
    changed.target = {2.0};
    changed.upper = {0.1, 0.1};
    changed.authorityMode = pg::AuthoritySolveMode::CachedProbe;
    changed.authorityHint = {0.4, 0.0, 0.0};
    const auto backedOff = pg::solveBoundedEqualityHierarchy(changed);
    check(backedOff.valid && near(backedOff.authority[0], 0.1, 1.0e-7),
          "cache: geometry-induced infeasibility backs off to a safe value");
    check(backedOff.authorityBackoffs[0] == 2
              && !backedOff.authorityProbeAccepted[0],
          "cache: deterministic backoff and rejected probe are reported");
    check(backedOff.authority[0] <= 1.0
              && backedOff.achieved[0] <= changed.target[0] + 1.0e-9,
          "cache: authority and achieved target never overshoot request");

    const auto repeated = pg::solveBoundedEqualityHierarchy(changed);
    check(repeated.value == backedOff.value
              && repeated.authority == backedOff.authority
              && repeated.authorityBackoffs == backedOff.authorityBackoffs
              && repeated.authorityProbeAccepted
                     == backedOff.authorityProbeAccepted,
          "cache: repeated live-geometry solve is deterministic");
}

void testInvalidInputIsNumericalFailure()
{
    auto p = problem(1, 1);
    p.matrix = {std::numeric_limits<double>::quiet_NaN()};
    p.target = {1.0};
    const auto r = pg::solveBoundedEqualityHierarchy(p);
    check(!r.valid && r.numericalFailure,
          "invalid: nonfinite matrix is a numerical/input failure");
}

}  // namespace

int main()
{
    testAnalyticFeasible();
    testActiveBoundAndImpossibleAuthority();
    testKktReleaseAtPreferredBound();
    testRankDeficiency();
    testPriorityHierarchy();
    testTwoRowDifferentialSharesAuthority();
    testSymmetryDeterminismAndSubdivision();
    testCachedAuthorityGrowthBackoffAndDeterminism();
    testInvalidInputIsNumericalFailure();
    if (failures != 0) {
        std::fprintf(stderr, "%d pressure-solver checks failed\n", failures);
        return 1;
    }
    std::printf("playground pressure solver checks passed\n");
    return 0;
}
