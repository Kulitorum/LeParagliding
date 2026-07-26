// Unit tests for the airfoil file parser and the parametric 2D B-spline
// fit behind the Section 2 airfoil editor. Run with the path to a real
// airfoil file (Data/gnua.txt). Pure C++ (no Qt).

#include "airfoil_file.h"
#include "spline_fit.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
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

void testRealFile(const char *path)
{
    std::ifstream stream(path, std::ios::binary);
    check(stream.good(), "real file: opens");
    std::stringstream buffer;
    buffer << stream.rdbuf();

    lep::AirfoilFile airfoil;
    std::string error;
    check(lep::parseAirfoilFile(buffer.str(), &airfoil, &error),
          "real file: parses");
    check(error.empty(), "real file: no consistency warnings");
    check(airfoil.totalPoints() > 50, "real file: has points");
    check(airfoil.segments().size() == 3, "real file: three segments");
    check(std::fabs(airfoil.xs.front() - 1.0) < 1e-6,
          "real file: starts at the trailing edge");
    check(std::fabs(airfoil.xs.back() - 1.0) < 1e-6,
          "real file: ends at the trailing edge");

    // Round trip: serialize and reparse without value drift.
    lep::AirfoilFile reparsed;
    check(lep::parseAirfoilFile(lep::formatAirfoilFile(airfoil), &reparsed,
                                &error),
          "round trip: reparses");
    check(reparsed.totalPoints() == airfoil.totalPoints(),
          "round trip: point count");
    double worst = 0.0;
    for (int i = 0; i < airfoil.totalPoints(); ++i) {
        worst = std::max(worst,
                         std::fabs(reparsed.xs[static_cast<size_t>(i)]
                                   - airfoil.xs[static_cast<size_t>(i)]));
        worst = std::max(worst,
                         std::fabs(reparsed.ys[static_cast<size_t>(i)]
                                   - airfoil.ys[static_cast<size_t>(i)]));
    }
    check(worst < 1e-6, "round trip: values preserved");

    // Fit every segment as a parametric spline within 0.2 % of chord and
    // resample at the original parameters: shape must hold and the point
    // counts (the header structure) must be untouched.
    const auto segments = airfoil.segments();
    for (const auto &segment : segments) {
        std::vector<double> xs, ys;
        for (int i = segment.first; i <= segment.last; ++i) {
            xs.push_back(airfoil.xs[static_cast<size_t>(i)]);
            ys.push_back(airfoil.ys[static_cast<size_t>(i)]);
        }
        double maxError = 0.0;
        const lep::BSpline2D spline =
            lep::fitBSpline2D(xs, ys, 0.002, &maxError);
        check(spline.isValid(), "segment fit: valid");
        check(maxError <= 0.002, "segment fit: tolerance met");
        check(spline.controlX.size() < xs.size() || xs.size() <= 4,
              "segment fit: fewer controls than points");

        const std::vector<double> params = lep::chordParameters(xs, ys);
        double x0 = 0.0;
        double y0 = 0.0;
        spline.evaluate(0.0, &x0, &y0);
        check(std::hypot(x0 - xs.front(), y0 - ys.front()) < 1e-9,
              "segment fit: start pinned");
        spline.evaluate(1.0, &x0, &y0);
        check(std::hypot(x0 - xs.back(), y0 - ys.back()) < 1e-9,
              "segment fit: end pinned");
        double resampleWorst = 0.0;
        for (size_t i = 0; i < params.size(); ++i) {
            spline.evaluate(params[i], &x0, &y0);
            resampleWorst = std::max(
                resampleWorst, std::hypot(x0 - xs[i], y0 - ys[i]));
        }
        check(resampleWorst <= 0.002,
              "segment fit: resampled points stay within tolerance");
    }
    std::printf("real airfoil: %d points, 3 segments fitted OK\n",
                airfoil.totalPoints());
}

void testBrokenInputs()
{
    lep::AirfoilFile airfoil;
    std::string error;
    check(!lep::parseAirfoilFile("name\n1\n2\n", &airfoil, &error),
          "broken: too short rejected");
    check(!lep::parseAirfoilFile(
              "name\n  3\n  2\n  2\n  1\n0 0\n0.5 oops\n1 0\n", &airfoil,
              &error),
          "broken: non-numeric point rejected");
    check(!lep::parseAirfoilFile("name\n  5\n  2\n  2\n  3\n0 0\n1 0\n",
                                 &airfoil, &error),
          "broken: missing points rejected");

    // Inconsistent header counts parse but warn, with a single segment.
    check(lep::parseAirfoilFile(
              "name\n  3\n  9\n  9\n  9\n0 0\n0.5 0.1\n1 0\n", &airfoil,
              &error),
          "inconsistent counts: still parses");
    check(!error.empty(), "inconsistent counts: warned");
    check(airfoil.segments().size() == 1,
          "inconsistent counts: one fallback segment");
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: airfoil-file-test <airfoil.txt>\n");
        return 2;
    }
    testRealFile(argv[1]);
    testBrokenInputs();
    if (failures > 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all airfoil_file checks passed\n");
    return 0;
}
