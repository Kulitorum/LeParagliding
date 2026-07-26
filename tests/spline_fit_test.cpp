// Unit tests for the B-spline fit/sample core behind the curve editor's
// spline mode. Pure C++ (no Qt), like the other test tools.

#include "spline_fit.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char *what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

void testLineIsExact()
{
    const std::vector<double> u = lep::uniformParameters(8);
    std::vector<double> y;
    for (const double t : u)
        y.push_back(3.0 + 4.0 * t);
    double error = 1.0;
    const lep::BSpline spline = lep::fitBSpline(u, y, 0.01, &error);
    check(spline.isValid(), "line: fit is valid");
    check(error < 1e-9, "line: fit is exact");
    check(spline.control.size() == 4, "line: minimal control count");
    check(std::fabs(spline.evaluate(0.5) - 5.0) < 1e-9,
          "line: evaluates mid-span");
}

void testEndPointsPinned()
{
    const std::vector<double> u = lep::uniformParameters(15);
    std::vector<double> y;
    for (const double t : u)
        y.push_back(std::sin(3.0 * t) * 100.0 + 20.0 * t * t);
    double error = 0.0;
    const lep::BSpline spline = lep::fitBSpline(u, y, 0.5, &error);
    check(spline.isValid(), "pinned: fit is valid");
    check(std::fabs(spline.evaluate(0.0) - y.front()) < 1e-9,
          "pinned: centre sample interpolated");
    check(std::fabs(spline.evaluate(1.0) - y.back()) < 1e-9,
          "pinned: tip sample interpolated");
    check(error <= 0.5, "pinned: tolerance met");
}

void testToleranceControlsComplexity()
{
    const std::vector<double> u = lep::uniformParameters(15);
    std::vector<double> y;
    for (const double t : u)
        y.push_back(std::sin(6.28 * t) * 50.0);
    double looseError = 0.0;
    double tightError = 0.0;
    const lep::BSpline loose = lep::fitBSpline(u, y, 5.0, &looseError);
    const lep::BSpline tight = lep::fitBSpline(u, y, 0.05, &tightError);
    check(loose.isValid() && tight.isValid(), "tolerance: fits valid");
    check(loose.control.size() <= tight.control.size(),
          "tolerance: looser tolerance never needs more controls");
    check(looseError <= 5.0, "tolerance: loose tolerance met");
    check(tightError <= 0.05, "tolerance: tight tolerance met");
}

void testResolutionIndependence()
{
    // A spline fitted at 15 stations must sample cleanly at any other
    // resolution: the curve, not the sample count, is the truth.
    const std::vector<double> u15 = lep::uniformParameters(15);
    std::vector<double> y;
    for (const double t : u15)
        y.push_back(200.0 * t * t * (3.0 - 2.0 * t));
    double error = 0.0;
    const lep::BSpline spline = lep::fitBSpline(u15, y, 0.1, &error);
    check(spline.isValid(), "resolution: fit valid");

    for (const int count : {2, 7, 15, 23, 101}) {
        const std::vector<double> stations = lep::uniformParameters(count);
        const std::vector<double> samples =
            lep::sampleBSpline(spline, stations);
        check(static_cast<int>(samples.size()) == count,
              "resolution: sample count");
        // The underlying smoothstep shape should be reproduced within the
        // fit tolerance plus a small margin at every resolution.
        double worst = 0.0;
        for (int i = 0; i < count; ++i) {
            const double t = stations[i];
            const double reference = 200.0 * t * t * (3.0 - 2.0 * t);
            worst = std::max(worst, std::fabs(samples[i] - reference));
        }
        check(worst < 0.75, "resolution: shape holds at other resolutions");
    }
}

void testConstantAndTinyInputs()
{
    const std::vector<double> u = lep::uniformParameters(15);
    const std::vector<double> flat(15, 33.33);
    double error = 1.0;
    const lep::BSpline spline = lep::fitBSpline(u, flat, 0.01, &error);
    check(spline.isValid() && error < 1e-9, "flat: exact fit");
    check(std::fabs(spline.evaluate(0.37) - 33.33) < 1e-9,
          "flat: constant everywhere");

    const std::vector<double> two = {1.0, 5.0};
    double twoError = 0.0;
    const lep::BSpline line =
        lep::fitBSpline(lep::uniformParameters(2), two, 0.01, &twoError);
    check(line.isValid(), "two samples: valid degree-1 fit");
    check(std::fabs(line.evaluate(0.5) - 3.0) < 1e-9,
          "two samples: linear midpoint");

    const lep::BSpline invalid =
        lep::fitBSpline({}, {}, 0.01, nullptr);
    check(!invalid.isValid(), "empty input: invalid spline");
}

void testKinkedDataDoesNotOscillate()
{
    // Regression: a real wing's beta column (gnuC2-27, kink near the tip at
    // rib 28) fitted at a tight 0.316 % tolerance. The uniform-growth fitter
    // needed 26 of 31 controls and oscillated 3.1 deg BETWEEN ribs 1 and 2
    // while staying within tolerance AT every rib; adaptive knot insertion
    // must concentrate knots at the kink instead.
    const std::vector<double> beta = {
        0.00, 1.63, 3.17, 4.72, 6.27, 7.82, 9.37, 10.92, 12.47, 14.01,
        15.64, 17.91, 19.98, 22.25, 24.31, 26.58, 28.65, 30.71, 32.78,
        34.84, 37.37, 40.38, 43.20, 46.02, 48.65, 51.47, 54.20, 62.43,
        71.72, 80.30, 89.23};
    const int n = static_cast<int>(beta.size());
    const std::vector<double> u = lep::uniformParameters(n);
    const double tolerance = 0.00316 * 89.23;
    double error = 0.0;
    const lep::BSpline spline = lep::fitBSpline(u, beta, tolerance, &error);
    check(spline.isValid(), "kink: fit valid");
    check(error <= tolerance, "kink: tolerance met at samples");
    check(spline.control.size() <= 20, "kink: no near-interpolation blowup");

    double worstBetween = 0.0;
    for (int i = 0; i <= 3000; ++i) {
        const double uu = i / 3000.0;
        const double s = spline.evaluate(uu);
        const double position = uu * (n - 1);
        const int k = std::min(static_cast<int>(position), n - 2);
        const double linear =
            beta[k] + (position - k) * (beta[k + 1] - beta[k]);
        worstBetween = std::max(worstBetween, std::fabs(s - linear));
    }
    check(worstBetween < 1.2,
          "kink: no oscillation between samples (regression)");
}

void testGrevilleAbscissae()
{
    const std::vector<double> u = lep::uniformParameters(15);
    std::vector<double> y;
    for (const double t : u)
        y.push_back(t * t * 100.0);
    const lep::BSpline spline = lep::fitBSpline(u, y, 0.01, nullptr);
    const std::vector<double> greville = spline.grevilleAbscissae();
    check(greville.size() == spline.control.size(),
          "greville: one abscissa per control");
    check(std::fabs(greville.front()) < 1e-12
              && std::fabs(greville.back() - 1.0) < 1e-12,
          "greville: clamped ends at 0 and 1");
    for (size_t i = 1; i < greville.size(); ++i)
        check(greville[i] >= greville[i - 1] - 1e-12,
              "greville: ascending");
}

} // namespace

int main()
{
    testLineIsExact();
    testEndPointsPinned();
    testToleranceControlsComplexity();
    testResolutionIndependence();
    testConstantAndTinyInputs();
    testKinkedDataDoesNotOscillate();
    testGrevilleAbscissae();
    if (failures > 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all spline_fit checks passed\n");
    return 0;
}
