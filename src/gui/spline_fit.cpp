#include "spline_fit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace lep {

namespace {

// Index of the knot span containing u (Piegl & Tiller A2.1).
int findSpan(const BSpline &spline, double u)
{
    const int p = spline.degree;
    const int n = static_cast<int>(spline.control.size()) - 1;
    if (u >= spline.knots[n + 1])
        return n;
    if (u <= spline.knots[p])
        return p;
    int low = p;
    int high = n + 1;
    int mid = (low + high) / 2;
    while (u < spline.knots[mid] || u >= spline.knots[mid + 1]) {
        if (u < spline.knots[mid])
            high = mid;
        else
            low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

// The p+1 nonzero basis functions at u (Piegl & Tiller A2.2).
void basisFunctions(const BSpline &spline, int span, double u,
                    std::vector<double> *result)
{
    const int p = spline.degree;
    result->assign(p + 1, 0.0);
    std::vector<double> left(p + 1, 0.0);
    std::vector<double> right(p + 1, 0.0);
    (*result)[0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - spline.knots[span + 1 - j];
        right[j] = spline.knots[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            const double temp =
                (*result)[r] / (right[r + 1] + left[j - r]);
            (*result)[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        (*result)[j] = saved;
    }
}

// Clamped knot vector with no interior knots: a single polynomial segment,
// the starting point for adaptive refinement.
std::vector<double> initialKnots(int degree)
{
    std::vector<double> knots(2 * (degree + 1));
    for (int i = 0; i <= degree; ++i) {
        knots[i] = 0.0;
        knots[knots.size() - 1 - i] = 1.0;
    }
    return knots;
}

// Solves the small dense symmetric system A x = b in place (Gaussian
// elimination with partial pivoting; A is at most ~samples-2 square).
bool solveDense(std::vector<std::vector<double>> &a, std::vector<double> &b)
{
    const int n = static_cast<int>(b.size());
    for (int column = 0; column < n; ++column) {
        int pivot = column;
        for (int row = column + 1; row < n; ++row) {
            if (std::abs(a[row][column]) > std::abs(a[pivot][column]))
                pivot = row;
        }
        if (std::abs(a[pivot][column]) < 1e-12)
            return false;
        std::swap(a[column], a[pivot]);
        std::swap(b[column], b[pivot]);
        for (int row = column + 1; row < n; ++row) {
            const double factor = a[row][column] / a[column][column];
            for (int k = column; k < n; ++k)
                a[row][k] -= factor * a[column][k];
            b[row] -= factor * b[column];
        }
    }
    for (int row = n - 1; row >= 0; --row) {
        double sum = b[row];
        for (int k = row + 1; k < n; ++k)
            sum -= a[row][k] * b[k];
        b[row] = sum / a[row][row];
    }
    return true;
}

// Least-squares control values for a fixed knot vector: end controls pinned
// to the end samples, interior controls from the normal equations
// (Piegl & Tiller A9.7 shape).
bool solveControls(const std::vector<double> &parameters,
                   const std::vector<double> &values, BSpline *spline)
{
    const int sampleCount = static_cast<int>(parameters.size());
    const int controlCount = static_cast<int>(spline->control.size());
    spline->control.front() = values.front();
    spline->control.back() = values.back();
    const int unknowns = controlCount - 2;
    if (unknowns <= 0)
        return true;

    // Basis matrix rows for the interior samples.
    std::vector<std::vector<double>> normal(
        unknowns, std::vector<double>(unknowns, 0.0));
    std::vector<double> rhs(unknowns, 0.0);
    std::vector<double> basis;
    for (int i = 0; i < sampleCount; ++i) {
        const int span = findSpan(*spline, parameters[i]);
        basisFunctions(*spline, span, parameters[i], &basis);
        // Residual after subtracting the pinned end controls.
        double target = values[i];
        std::vector<std::pair<int, double>> row;
        for (int j = 0; j <= spline->degree; ++j) {
            const int controlIndex = span - spline->degree + j;
            const double weight = basis[j];
            if (controlIndex == 0)
                target -= weight * spline->control.front();
            else if (controlIndex == controlCount - 1)
                target -= weight * spline->control.back();
            else
                row.emplace_back(controlIndex - 1, weight);
        }
        for (const auto &[rowIndex, rowWeight] : row) {
            rhs[rowIndex] += rowWeight * target;
            for (const auto &[columnIndex, columnWeight] : row)
                normal[rowIndex][columnIndex] += rowWeight * columnWeight;
        }
    }
    if (!solveDense(normal, rhs))
        return false;
    for (int j = 0; j < unknowns; ++j)
        spline->control[j + 1] = rhs[j];
    return true;
}

double maxDeviation(const BSpline &spline,
                    const std::vector<double> &parameters,
                    const std::vector<double> &values, double *worstAt)
{
    double worst = -1.0;
    for (std::size_t i = 0; i < parameters.size(); ++i) {
        const double deviation =
            std::abs(spline.evaluate(parameters[i]) - values[i]);
        if (deviation > worst) {
            worst = deviation;
            if (worstAt != nullptr)
                *worstAt = parameters[i];
        }
    }
    return std::max(worst, 0.0);
}

} // namespace

bool BSpline::isValid() const
{
    return degree >= 1
           && static_cast<int>(control.size()) >= degree + 1
           && knots.size() == control.size() + degree + 1
           && std::is_sorted(knots.begin(), knots.end());
}

double BSpline::evaluate(double u) const
{
    u = std::clamp(u, knots.front(), knots.back());
    const int span = findSpan(*this, u);
    std::vector<double> basis;
    basisFunctions(*this, span, u, &basis);
    double value = 0.0;
    for (int j = 0; j <= degree; ++j)
        value += basis[j] * control[span - degree + j];
    return value;
}

std::vector<double> BSpline::grevilleAbscissae() const
{
    std::vector<double> result(control.size(), 0.0);
    for (std::size_t i = 0; i < control.size(); ++i) {
        double sum = 0.0;
        for (int j = 1; j <= degree; ++j)
            sum += knots[i + j];
        result[i] = sum / degree;
    }
    return result;
}

std::vector<double> sampleBSpline(const BSpline &spline,
                                  const std::vector<double> &parameters)
{
    std::vector<double> result;
    result.reserve(parameters.size());
    for (const double u : parameters)
        result.push_back(spline.evaluate(u));
    return result;
}

BSpline fitBSpline(const std::vector<double> &parameters,
                   const std::vector<double> &values, double tolerance,
                   double *maxError)
{
    if (maxError != nullptr)
        *maxError = 0.0;
    BSpline best;
    const int sampleCount = static_cast<int>(values.size());
    if (sampleCount < 2 || parameters.size() != values.size())
        return BSpline{0, {}, {}};

    const int degree = std::min(3, sampleCount - 1);
    // Adaptive refinement: least-squares fit, then bisect the knot span
    // holding the worst sample until the tolerance is met. Knots concentrate
    // where the data actually bends (a wingtip kink, say) and stay sparse in
    // smooth regions — a uniformly grown near-interpolating fit oscillates
    // wildly BETWEEN samples even though every sample is within tolerance.
    BSpline candidate;
    candidate.degree = degree;
    candidate.knots = initialKnots(degree);
    candidate.control.assign(degree + 1, 0.0);
    // A span narrower than half the sample spacing has no interior samples
    // left to constrain it; refining further only invites oscillation.
    const double minSpan = 0.5 / (sampleCount - 1);
    double bestDeviation = 0.0;
    while (true) {
        if (!solveControls(parameters, values, &candidate))
            break; // singular system; keep the best fit found so far
        double worstAt = 0.0;
        const double deviation =
            maxDeviation(candidate, parameters, values, &worstAt);
        if (!best.isValid() || deviation < bestDeviation) {
            best = candidate;
            bestDeviation = deviation;
            if (maxError != nullptr)
                *maxError = deviation;
        }
        if (deviation <= tolerance
            || static_cast<int>(candidate.control.size()) >= sampleCount)
            break;
        // Bisect the (non-empty) span containing the worst sample.
        double low = 0.0;
        double high = 1.0;
        for (std::size_t k = degree;
             k + 1 < candidate.knots.size() - degree; ++k) {
            if (candidate.knots[k + 1] <= candidate.knots[k])
                continue;
            if (worstAt >= candidate.knots[k]
                && (worstAt < candidate.knots[k + 1]
                    || candidate.knots[k + 1] >= 1.0)) {
                low = candidate.knots[k];
                high = candidate.knots[k + 1];
                break;
            }
        }
        if (high - low < minSpan)
            break;
        const double newKnot = 0.5 * (low + high);
        candidate.knots.insert(
            std::upper_bound(candidate.knots.begin(), candidate.knots.end(),
                             newKnot),
            newKnot);
        candidate.control.assign(candidate.control.size() + 1, 0.0);
    }
    return best;
}

std::vector<double> uniformParameters(int count)
{
    std::vector<double> result;
    if (count < 2)
        return result;
    result.reserve(count);
    for (int i = 0; i < count; ++i)
        result.push_back(static_cast<double>(i)
                         / static_cast<double>(count - 1));
    return result;
}

bool BSpline2D::isValid() const
{
    return degree >= 1 && controlX.size() == controlY.size()
           && static_cast<int>(controlX.size()) >= degree + 1
           && knots.size() == controlX.size() + degree + 1
           && std::is_sorted(knots.begin(), knots.end());
}

void BSpline2D::evaluate(double u, double *x, double *y) const
{
    const BSpline sx{degree, knots, controlX};
    const BSpline sy{degree, knots, controlY};
    *x = sx.evaluate(u);
    *y = sy.evaluate(u);
}

std::vector<double> BSpline2D::grevilleAbscissae() const
{
    return BSpline{degree, knots, controlX}.grevilleAbscissae();
}

std::vector<double> chordParameters(const std::vector<double> &xs,
                                    const std::vector<double> &ys)
{
    std::vector<double> params(xs.size(), 0.0);
    double total = 0.0;
    for (std::size_t i = 1; i < xs.size(); ++i) {
        total += std::hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
        params[i] = total;
    }
    if (total <= 0.0)
        return uniformParameters(static_cast<int>(xs.size()));
    for (double &value : params)
        value /= total;
    return params;
}

BSpline2D fitBSpline2D(const std::vector<double> &xs,
                       const std::vector<double> &ys, double tolerance,
                       double *maxError)
{
    if (maxError != nullptr)
        *maxError = 0.0;
    BSpline2D best;
    const int sampleCount = static_cast<int>(xs.size());
    if (sampleCount < 2 || xs.size() != ys.size())
        return best;

    const std::vector<double> params = chordParameters(xs, ys);
    const int degree = std::min(3, sampleCount - 1);
    const double minSpan = 0.5 / (sampleCount - 1);

    BSpline componentX;
    componentX.degree = degree;
    componentX.knots = initialKnots(degree);
    componentX.control.assign(degree + 1, 0.0);
    BSpline componentY = componentX;

    double bestDeviation = 0.0;
    while (true) {
        if (!solveControls(params, xs, &componentX)
            || !solveControls(params, ys, &componentY))
            break;
        double deviation = -1.0;
        double worstAt = 0.0;
        for (int i = 0; i < sampleCount; ++i) {
            const double d =
                std::hypot(componentX.evaluate(params[i]) - xs[i],
                           componentY.evaluate(params[i]) - ys[i]);
            if (d > deviation) {
                deviation = d;
                worstAt = params[i];
            }
        }
        if (!best.isValid() || deviation < bestDeviation) {
            best = BSpline2D{degree, componentX.knots, componentX.control,
                             componentY.control};
            bestDeviation = deviation;
            if (maxError != nullptr)
                *maxError = deviation;
        }
        if (deviation <= tolerance
            || static_cast<int>(componentX.control.size()) >= sampleCount)
            break;
        double low = 0.0;
        double high = 1.0;
        for (std::size_t k = degree;
             k + 1 < componentX.knots.size() - degree; ++k) {
            if (componentX.knots[k + 1] <= componentX.knots[k])
                continue;
            if (worstAt >= componentX.knots[k]
                && (worstAt < componentX.knots[k + 1]
                    || componentX.knots[k + 1] >= 1.0)) {
                low = componentX.knots[k];
                high = componentX.knots[k + 1];
                break;
            }
        }
        if (high - low < minSpan)
            break;
        const double newKnot = 0.5 * (low + high);
        componentX.knots.insert(
            std::upper_bound(componentX.knots.begin(),
                             componentX.knots.end(), newKnot),
            newKnot);
        componentY.knots = componentX.knots;
        componentX.control.assign(componentX.control.size() + 1, 0.0);
        componentY.control.assign(componentY.control.size() + 1, 0.0);
    }
    return best;
}

} // namespace lep
