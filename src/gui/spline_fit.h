#pragma once

#include <vector>

namespace lep {

// Function-form clamped B-spline y(u) over u in [0, 1]: fixed knot vector,
// one control value per basis function (abscissae are the Greville points).
// The domain is normalized and carries no rib count: a design's rib matrix
// at any resolution is this spline sampled at that resolution's stations.
// Pure C++ so it compiles into the Qt-free test executables.
struct BSpline
{
    int degree = 3;
    std::vector<double> knots;   // clamped, normalized to [0, 1]
    std::vector<double> control; // control values, size = knots.size() - degree - 1

    bool isValid() const;
    double evaluate(double u) const;
    // Greville abscissae in u — the natural x positions of the control
    // handles in a function-form spline.
    std::vector<double> grevilleAbscissae() const;
};

// Evaluates the spline at every station in `parameters`.
std::vector<double> sampleBSpline(const BSpline &spline,
                                  const std::vector<double> &parameters);

// One-time import of existing point data into spline truth: least-squares
// fit to samples (parameters[i], values[i]), parameters ascending in [0, 1].
// The first and last samples are interpolated exactly (wing centre and tip
// must not move); interior control values minimize the squared error. The
// control count grows until every sample deviates by at most `tolerance`,
// or the spline has as many controls as samples (then it interpolates).
// Returns the fitted spline; *maxError receives the achieved maximum
// deviation. Fewer than two samples yield an invalid spline.
BSpline fitBSpline(const std::vector<double> &parameters,
                   const std::vector<double> &values, double tolerance,
                   double *maxError);

// Uniform stations 0..1 for `count` ribs (the current sampling policy for
// the rib matrix; station placement can grow smarter without touching the
// stored splines).
std::vector<double> uniformParameters(int count);

// Parametric planar B-spline (x(u), y(u)) with one shared knot vector —
// for closed-nose shapes like airfoil segments where y(x) is not a
// function. Control points are genuine 2D handles.
struct BSpline2D
{
    int degree = 3;
    std::vector<double> knots;
    std::vector<double> controlX;
    std::vector<double> controlY;

    bool isValid() const;
    void evaluate(double u, double *x, double *y) const;
    std::vector<double> grevilleAbscissae() const;
};

// Least-squares fit of a point sequence within `tolerance` (maximum
// euclidean deviation at the samples; end points interpolated exactly).
// Parameters are chord-length based, both components share the adaptively
// refined knot vector. *maxError receives the achieved deviation.
BSpline2D fitBSpline2D(const std::vector<double> &xs,
                       const std::vector<double> &ys, double tolerance,
                       double *maxError);

// Chord-length parameters in [0, 1] for a point sequence (the fit's own
// parameterization; use it to resample at the original point layout).
std::vector<double> chordParameters(const std::vector<double> &xs,
                                    const std::vector<double> &ys);

} // namespace lep
