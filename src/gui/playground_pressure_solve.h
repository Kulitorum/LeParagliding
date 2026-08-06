#ifndef LEP_PLAYGROUND_PRESSURE_SOLVE_H
#define LEP_PLAYGROUND_PRESSURE_SOLVE_H

#include <array>
#include <cstddef>
#include <vector>

namespace lep::playground {

enum class AuthoritySolveMode
{
    // Accurate but deliberately expensive; intended for small pure tests and
    // offline audits, not a full canopy on every frame.
    ExactBisection,
    // Production: verify the previous frame's authority, back off if geometry
    // invalidated it, then try one bounded increase.
    CachedProbe,
};

// Weighted projection of a preferred exterior-Cp field onto a bounded set
// of equality targets. Rows are introduced in three priority stages; a later
// stage freezes every result achieved by the earlier ones.
struct BoundedEqualityProblem
{
    std::size_t variableCount = 0;
    std::size_t rowCount = 0;
    std::vector<double> prior;
    std::vector<double> lower;
    std::vector<double> upper;
    // Inverse objective weight. Larger values let this variable move more.
    std::vector<double> mobility;
    // Row-major rowCount x variableCount matrix and absolute row targets.
    std::vector<double> matrix;
    std::vector<double> target;
    // Force, pitch, differential in the Playground integration. The pure
    // solver is deliberately generic and only requires that the sum match.
    std::array<std::size_t, 3> stageRowCounts{0, 0, 0};
    AuthoritySolveMode authorityMode = AuthoritySolveMode::ExactBisection;
    std::array<double, 3> authorityHint{0.0, 0.0, 0.0};
    // Normalised dual multipliers from the preceding live-geometry solve.
    // Empty vectors request a cold start.
    std::array<std::vector<double>, 3> multiplierHint;
    double authorityProbeStep = 0.02;
    double feasibilityTolerance = 1.0e-8;
};

struct BoundedEqualityResult
{
    bool valid = false;
    bool numericalFailure = false;
    std::vector<double> value;
    std::vector<double> requested;
    std::vector<double> achieved;
    std::vector<double> residual;
    std::array<double, 3> authority{0.0, 0.0, 0.0};
    std::array<double, 3> authorityHint{0.0, 0.0, 0.0};
    std::array<std::size_t, 3> authorityBackoffs{0, 0, 0};
    std::array<bool, 3> authorityProbeAccepted{false, false, false};
    std::array<std::vector<double>, 3> multiplierHint;
    std::array<std::size_t, 3> rank{0, 0, 0};
    std::array<double, 3> conditionEstimate{0.0, 0.0, 0.0};
    std::size_t activeLower = 0;
    std::size_t activeUpper = 0;
    std::size_t projectionCalls = 0;
    std::size_t projectionIterations = 0;
};

[[nodiscard]] BoundedEqualityResult solveBoundedEqualityHierarchy(
    const BoundedEqualityProblem &problem);

}  // namespace lep::playground

#endif
