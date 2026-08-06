#include "playground_pressure_solve.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace lep::playground {
namespace {

constexpr double kPivotTolerance = 1.0e-12;

struct DenseSolve
{
    bool consistent = false;
    std::vector<double> value;
    std::size_t rank = 0;
    double condition = 0.0;
};

DenseSolve solveDense(std::vector<double> matrix,
                      std::vector<double> right,
                      std::size_t size,
                      double tolerance)
{
    DenseSolve result;
    result.value.assign(size, 0.0);
    if (size == 0) {
        result.consistent = true;
        return result;
    }
    std::vector<std::size_t> pivotColumn;
    double largestPivot = 0.0;
    double smallestPivot = std::numeric_limits<double>::max();
    std::size_t pivotRow = 0;
    for (std::size_t column = 0; column < size && pivotRow < size;
         ++column) {
        std::size_t best = pivotRow;
        double magnitude = std::abs(matrix[pivotRow * size + column]);
        for (std::size_t row = pivotRow + 1; row < size; ++row) {
            const double candidate = std::abs(matrix[row * size + column]);
            if (candidate > magnitude) {
                magnitude = candidate;
                best = row;
            }
        }
        if (magnitude <= kPivotTolerance) {
            continue;
        }
        if (best != pivotRow) {
            for (std::size_t c = 0; c < size; ++c) {
                std::swap(matrix[pivotRow * size + c],
                          matrix[best * size + c]);
            }
            std::swap(right[pivotRow], right[best]);
        }
        const double pivot = matrix[pivotRow * size + column];
        largestPivot = std::max(largestPivot, std::abs(pivot));
        smallestPivot = std::min(smallestPivot, std::abs(pivot));
        for (std::size_t c = column; c < size; ++c) {
            matrix[pivotRow * size + c] /= pivot;
        }
        right[pivotRow] /= pivot;
        for (std::size_t row = 0; row < size; ++row) {
            if (row == pivotRow) {
                continue;
            }
            const double factor = matrix[row * size + column];
            if (factor == 0.0) {
                continue;
            }
            for (std::size_t c = column; c < size; ++c) {
                matrix[row * size + c] -=
                    factor * matrix[pivotRow * size + c];
            }
            right[row] -= factor * right[pivotRow];
        }
        pivotColumn.push_back(column);
        ++pivotRow;
    }
    result.rank = pivotRow;
    result.condition = result.rank > 0
                           ? largestPivot / smallestPivot
                           : std::numeric_limits<double>::infinity();
    for (std::size_t row = pivotRow; row < size; ++row) {
        double scale = 0.0;
        for (std::size_t column = 0; column < size; ++column) {
            scale = std::max(scale,
                             std::abs(matrix[row * size + column]));
        }
        if (scale <= kPivotTolerance
            && std::abs(right[row]) > tolerance) {
            return result;
        }
    }
    for (std::size_t row = 0; row < pivotColumn.size(); ++row) {
        result.value[pivotColumn[row]] = right[row];
    }
    result.consistent = true;
    return result;
}

double rowValue(const BoundedEqualityProblem &problem,
                std::size_t row,
                const std::vector<double> &value)
{
    double result = 0.0;
    const std::size_t first = row * problem.variableCount;
    for (std::size_t variable = 0; variable < problem.variableCount;
         ++variable) {
        result += problem.matrix[first + variable] * value[variable];
    }
    return result;
}

struct Projection
{
    bool feasible = false;
    bool numericalFailure = false;
    std::vector<double> value;
    std::size_t rank = 0;
    double condition = 0.0;
    std::size_t iterations = 0;
    std::vector<double> multiplier;
};

Projection project(const BoundedEqualityProblem &problem,
                   const std::vector<double> &preferred,
                   const std::vector<double> &target,
                   std::size_t activeRows,
                   const Projection *warmStart = nullptr)
{
    Projection result;
    const std::size_t count = problem.variableCount;
    result.value.resize(count);
    if (activeRows == 0) {
        for (std::size_t variable = 0; variable < count; ++variable) {
            result.value[variable] = std::clamp(
                preferred[variable], problem.lower[variable],
                problem.upper[variable]);
        }
        result.feasible = true;
        return result;
    }

    // The dual of the bounded weighted projection has only activeRows
    // unknowns (six at most in the Playground). For a dual multiplier λ,
    // x(λ)=clamp(prior + mobility*A^T*λ). A semismooth Newton step solves
    // the tiny Schur matrix formed by variables that are currently free.
    // Bounds therefore activate and release as a set, with no O(faces)
    // active-set sweep. Public residuals remain in physical units.
    std::vector<double> rowScale(activeRows, 1.0);
    for (std::size_t row = 0; row < activeRows; ++row) {
        double normSquared = 0.0;
        for (std::size_t variable = 0; variable < count; ++variable) {
            const double coefficient =
                problem.matrix[row * count + variable];
            normSquared += coefficient * coefficient
                           * problem.mobility[variable];
        }
        if (normSquared > kPivotTolerance * kPivotTolerance) {
            rowScale[row] = 1.0 / std::sqrt(normSquared);
        }
    }
    std::vector<double> multiplier(activeRows, 0.0);
    if (warmStart != nullptr) {
        const std::size_t shared = std::min(
            activeRows, warmStart->multiplier.size());
        std::copy_n(warmStart->multiplier.begin(), shared,
                    multiplier.begin());
    }
    std::vector<double> trialMultiplier(activeRows, 0.0);
    std::vector<double> trialValue(count, 0.0);
    const auto evaluate = [&](const std::vector<double> &lambda,
                              std::vector<double> &value,
                              std::vector<double> *residual,
                              std::vector<double> *schur) {
        if (residual != nullptr) {
            residual->resize(activeRows);
            for (std::size_t row = 0; row < activeRows; ++row) {
                (*residual)[row] = rowScale[row] * target[row];
            }
        }
        if (schur != nullptr) {
            schur->assign(activeRows * activeRows, 0.0);
        }
        for (std::size_t variable = 0; variable < count; ++variable) {
            double dual = 0.0;
            for (std::size_t row = 0; row < activeRows; ++row) {
                dual += rowScale[row]
                        * problem.matrix[row * count + variable]
                        * lambda[row];
            }
            const double unconstrained =
                preferred[variable] + problem.mobility[variable] * dual;
            value[variable] = std::clamp(
                unconstrained, problem.lower[variable],
                problem.upper[variable]);
            const bool free =
                problem.upper[variable] - problem.lower[variable]
                    > kPivotTolerance
                && unconstrained
                       >= problem.lower[variable] - 1.0e-12
                && unconstrained
                       <= problem.upper[variable] + 1.0e-12;
            for (std::size_t row = 0; row < activeRows; ++row) {
                const double coefficient =
                    rowScale[row]
                    * problem.matrix[row * count + variable];
                if (residual != nullptr) {
                    (*residual)[row] -= coefficient * value[variable];
                }
                if (free && schur != nullptr) {
                    for (std::size_t other = 0; other < activeRows;
                         ++other) {
                        (*schur)[row * activeRows + other] +=
                            coefficient * problem.mobility[variable]
                            * rowScale[other]
                            * problem.matrix[other * count + variable];
                    }
                }
            }
        }
    };
    const auto residualNorm = [](const std::vector<double> &residual) {
        double norm = 0.0;
        for (const double value : residual) {
            norm = std::max(norm, std::abs(value));
        }
        return norm;
    };

    constexpr std::size_t iterationLimit = 96;
    std::vector<double> residual;
    std::vector<double> schur;
    for (std::size_t iteration = 0; iteration < iterationLimit; ++iteration) {
        ++result.iterations;
        evaluate(multiplier, result.value, &residual, &schur);
        result.multiplier = multiplier;
        const double norm = residualNorm(residual);
        double physicalWorst = 0.0;
        double physicalTarget = 0.0;
        for (std::size_t row = 0; row < activeRows; ++row) {
            physicalWorst = std::max(
                physicalWorst,
                std::abs(rowValue(problem, row, result.value) - target[row]));
            physicalTarget = std::max(physicalTarget, std::abs(target[row]));
        }
        if (physicalWorst
            <= problem.feasibilityTolerance * (1.0 + physicalTarget)) {
            const DenseSolve rankProbe = solveDense(
                schur, std::vector<double>(activeRows, 0.0),
                activeRows, problem.feasibilityTolerance);
            result.rank = rankProbe.rank;
            result.condition = rankProbe.condition;
            result.feasible = true;
            return result;
        }

        const DenseSolve step = solveDense(
            std::move(schur), residual, activeRows,
            problem.feasibilityTolerance);
        result.rank = step.rank;
        result.condition = step.condition;
        if (!step.consistent) {
            return result;
        }
        if (!std::all_of(step.value.begin(), step.value.end(),
                         [](double value) { return std::isfinite(value); })) {
            result.numericalFailure = true;
            return result;
        }

        bool accepted = false;
        double scale = 1.0;
        for (int lineSearch = 0; lineSearch < 24; ++lineSearch) {
            for (std::size_t row = 0; row < activeRows; ++row) {
                trialMultiplier[row] =
                    multiplier[row] + scale * step.value[row];
            }
            std::vector<double> trialResidual;
            evaluate(trialMultiplier, trialValue, &trialResidual, nullptr);
            if (residualNorm(trialResidual) < norm) {
                multiplier = trialMultiplier;
                accepted = true;
                break;
            }
            scale *= 0.5;
        }
        if (!accepted) {
            return result;
        }
    }
    result.numericalFailure = true;
    return result;
}

}  // namespace

BoundedEqualityResult solveBoundedEqualityHierarchy(
    const BoundedEqualityProblem &problem)
{
    BoundedEqualityResult result;
    const std::size_t stagedRows = std::accumulate(
        problem.stageRowCounts.begin(), problem.stageRowCounts.end(),
        std::size_t{0});
    if (problem.variableCount == 0 || problem.rowCount == 0
        || stagedRows != problem.rowCount
        || problem.prior.size() != problem.variableCount
        || problem.lower.size() != problem.variableCount
        || problem.upper.size() != problem.variableCount
        || problem.mobility.size() != problem.variableCount
        || problem.matrix.size() != problem.rowCount * problem.variableCount
        || problem.target.size() != problem.rowCount
        || !(problem.feasibilityTolerance > 0.0)
        || !std::isfinite(problem.feasibilityTolerance)
        || (problem.authorityMode == AuthoritySolveMode::CachedProbe
            && (!(problem.authorityProbeStep > 0.0)
                || !std::isfinite(problem.authorityProbeStep)
                || !std::all_of(
                    problem.authorityHint.begin(),
                    problem.authorityHint.end(),
                    [](double value) { return std::isfinite(value); })))) {
        result.numericalFailure = true;
        return result;
    }
    for (std::size_t variable = 0; variable < problem.variableCount;
         ++variable) {
        if (!std::isfinite(problem.prior[variable])
            || !std::isfinite(problem.lower[variable])
            || !std::isfinite(problem.upper[variable])
            || !(problem.lower[variable] <= problem.upper[variable])
            || !(problem.mobility[variable] > 0.0)
            || !std::isfinite(problem.mobility[variable])) {
            result.numericalFailure = true;
            return result;
        }
    }
    if (!std::all_of(problem.matrix.begin(), problem.matrix.end(),
                      [](double value) { return std::isfinite(value); })
        || !std::all_of(problem.target.begin(), problem.target.end(),
                        [](double value) { return std::isfinite(value); })
        || !std::all_of(
            problem.multiplierHint.begin(), problem.multiplierHint.end(),
            [](const std::vector<double> &hint) {
                return std::all_of(
                    hint.begin(), hint.end(),
                    [](double value) { return std::isfinite(value); });
            })) {
        result.numericalFailure = true;
        return result;
    }

    result.requested = problem.target;
    result.value.resize(problem.variableCount);
    for (std::size_t variable = 0; variable < problem.variableCount;
         ++variable) {
        result.value[variable] = std::clamp(
            problem.prior[variable], problem.lower[variable],
            problem.upper[variable]);
    }
    std::vector<double> frozenTarget(problem.rowCount, 0.0);
    Projection stageState;
    std::size_t activeRows = 0;
    for (int stage = 0; stage < 3; ++stage) {
        const std::size_t added = problem.stageRowCounts[stage];
        const std::size_t nextRows = activeRows + added;
        const double suppliedHint = std::clamp(
            problem.authorityHint[stage], 0.0, 1.0);
        result.authorityHint[stage] = suppliedHint;
        if (added == 0) {
            result.authority[stage] = 1.0;
            continue;
        }
        std::vector<double> baseline(added, 0.0);
        for (std::size_t row = 0; row < added; ++row) {
            baseline[row] = rowValue(problem, activeRows + row, result.value);
            frozenTarget[activeRows + row] = problem.target[activeRows + row];
        }
        Projection accepted;
        const bool exactAuthority =
            problem.authorityMode == AuthoritySolveMode::ExactBisection;
        Projection stageWarm = stageState;
        if (!exactAuthority && !problem.multiplierHint[stage].empty()) {
            stageWarm.multiplier = problem.multiplierHint[stage];
        }
        // A cached value below one already proved the full request impossible
        // on the preceding frame. Avoid paying for that known failure. A hint
        // at one retains the cheap one-projection full-target fast path.
        const bool tryFullTarget = exactAuthority
                                   || suppliedHint >= 1.0 - 1.0e-12;
        if (tryFullTarget) {
            accepted = project(
                problem, problem.prior, frozenTarget, nextRows, &stageWarm);
            ++result.projectionCalls;
            result.projectionIterations += accepted.iterations;
            if (accepted.numericalFailure) {
                result.numericalFailure = true;
                return result;
            }
        }
        double authority = accepted.feasible ? 1.0 : 0.0;
        if (!accepted.feasible) {
            for (std::size_t row = 0; row < added; ++row) {
                frozenTarget[activeRows + row] = baseline[row];
            }
            if (exactAuthority) {
                accepted = project(problem, problem.prior,
                                   frozenTarget, nextRows, &stageState);
                ++result.projectionCalls;
                result.projectionIterations += accepted.iterations;
                if (accepted.numericalFailure || !accepted.feasible) {
                    result.rank[stage] = accepted.rank;
                    result.conditionEstimate[stage] = accepted.condition;
                    result.numericalFailure = true;
                    return result;
                }
            } else {
                // The current hierarchical value is exactly the zero-authority
                // baseline for the newly added rows; projecting it again would
                // spend one O(faces) pass per stage to rediscover the same
                // weighted optimum. Reuse it as the cached probe warm start.
                accepted = stageWarm;
                accepted.value = result.value;
                accepted.feasible = true;
                accepted.numericalFailure = false;
            }
            if (exactAuthority) {
                // Exact maximum on the requested target ray. This costs up to
                // 24 full-face projections and is reserved for pure tests and
                // explicit offline audits.
                double lowerAuthority = 0.0;
                double upperAuthority = 1.0;
                for (int iteration = 0; iteration < 24; ++iteration) {
                    const double trialAuthority =
                        0.5 * (lowerAuthority + upperAuthority);
                    for (std::size_t row = 0; row < added; ++row) {
                        frozenTarget[activeRows + row] =
                            baseline[row]
                            + trialAuthority
                                  * (problem.target[activeRows + row]
                                     - baseline[row]);
                    }
                    Projection candidate = project(
                        problem, problem.prior, frozenTarget, nextRows,
                        &accepted);
                    ++result.projectionCalls;
                    result.projectionIterations += candidate.iterations;
                    if (candidate.numericalFailure) {
                        result.numericalFailure = true;
                        return result;
                    }
                    if (candidate.feasible) {
                        lowerAuthority = trialAuthority;
                        accepted = std::move(candidate);
                    } else {
                        upperAuthority = trialAuthority;
                    }
                }
                authority = lowerAuthority;
            } else {
                // Production verifies last frame's authority against the live
                // geometry. If it no longer fits, halve deterministically
                // until it does; then make one +step probe. Thus every
                // accepted value remains an exact bounded projection, while a
                // stable saturated stage costs only two projections per frame.
                authority = suppliedHint;
                bool cachedFeasible = authority <= 1.0e-12;
                if (tryFullTarget && authority >= 1.0 - 1.0e-12) {
                    authority *= 0.5;
                    ++result.authorityBackoffs[stage];
                    cachedFeasible = false;
                }
                constexpr int kMaximumCachedBackoffs = 8;
                for (int backoff = 0;
                     !cachedFeasible && backoff < kMaximumCachedBackoffs;
                     ++backoff) {
                    for (std::size_t row = 0; row < added; ++row) {
                        frozenTarget[activeRows + row] =
                            baseline[row]
                            + authority
                                  * (problem.target[activeRows + row]
                                     - baseline[row]);
                    }
                    Projection candidate = project(
                        problem, problem.prior, frozenTarget, nextRows,
                        &accepted);
                    ++result.projectionCalls;
                    result.projectionIterations += candidate.iterations;
                    if (candidate.numericalFailure) {
                        result.numericalFailure = true;
                        return result;
                    }
                    if (candidate.feasible) {
                        accepted = std::move(candidate);
                        cachedFeasible = true;
                    } else {
                        authority *= 0.5;
                        ++result.authorityBackoffs[stage];
                        if (authority <= 1.0e-12) {
                            authority = 0.0;
                            cachedFeasible = true;
                        }
                    }
                }
                const bool exhaustedBackoff = !cachedFeasible;
                if (exhaustedBackoff) {
                    // Rank can remove all nonzero authority for a stage. The
                    // exact baseline is always feasible by construction; use
                    // it after the bounded backoff budget rather than turning
                    // physical zero-authority into a numerical failure.
                    authority = 0.0;
                    accepted = stageWarm;
                    accepted.value = result.value;
                    accepted.feasible = true;
                    accepted.numericalFailure = false;
                }

                const double probeAuthority = std::min(
                    1.0, authority + problem.authorityProbeStep);
                if (!exhaustedBackoff
                    && probeAuthority > authority + 1.0e-12) {
                    for (std::size_t row = 0; row < added; ++row) {
                        frozenTarget[activeRows + row] =
                            baseline[row]
                            + probeAuthority
                                  * (problem.target[activeRows + row]
                                     - baseline[row]);
                    }
                    Projection candidate = project(
                        problem, problem.prior, frozenTarget, nextRows,
                        &accepted);
                    ++result.projectionCalls;
                    result.projectionIterations += candidate.iterations;
                    if (candidate.numericalFailure) {
                        result.numericalFailure = true;
                        return result;
                    }
                    if (candidate.feasible) {
                        accepted = std::move(candidate);
                        authority = probeAuthority;
                        result.authorityProbeAccepted[stage] = true;
                    }
                }
            }
            for (std::size_t row = 0; row < added; ++row) {
                frozenTarget[activeRows + row] =
                    rowValue(problem, activeRows + row, accepted.value);
            }
        }
        result.value = accepted.value;
        stageState = accepted;
        result.authority[stage] = authority;
        result.rank[stage] = accepted.rank;
        result.conditionEstimate[stage] = accepted.condition;
        result.multiplierHint[stage] = accepted.multiplier;
        for (std::size_t row = 0; row < nextRows; ++row) {
            frozenTarget[row] = rowValue(problem, row, result.value);
        }
        activeRows = nextRows;
    }

    result.achieved.resize(problem.rowCount);
    result.residual.resize(problem.rowCount);
    for (std::size_t row = 0; row < problem.rowCount; ++row) {
        result.achieved[row] = rowValue(problem, row, result.value);
        result.residual[row] = result.achieved[row] - problem.target[row];
    }
    for (std::size_t variable = 0; variable < problem.variableCount;
         ++variable) {
        result.activeLower +=
            result.value[variable] <= problem.lower[variable] + 1.0e-10;
        result.activeUpper +=
            result.value[variable] >= problem.upper[variable] - 1.0e-10;
    }
    result.valid = true;
    return result;
}

}  // namespace lep::playground
