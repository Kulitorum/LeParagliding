#include "geometry_preprocessor.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <sstream>

namespace lep {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Point
{
    double x = 0.0;
    double y = 0.0;
};

// The Fortran source walks its analytic curves as (q, p) segment pairs; the
// interpolation code later depends on the exact same pairs, so keep them.
struct CurveSegments
{
    std::vector<double> xq, yq, xp, yp;

    void add(double aq, double bq, double ap, double bp)
    {
        xq.push_back(aq);
        yq.push_back(bq);
        xp.push_back(ap);
        yp.push_back(bp);
    }

    int count() const { return static_cast<int>(xq.size()); }
};

void addMirroredSegment(std::vector<PreProcessorSegment> &drawing,
                        double x1, double y1, double x2, double y2,
                        int rightColor, int leftColor)
{
    drawing.push_back({x1, y1, x2, y2, rightColor});
    drawing.push_back({-x1, y1, -x2, y2, leftColor});
}

// eo() subroutine: rib count of the half wing for a cell count.
void splitCells(int cellCount, bool *even, int *ribCount)
{
    if (cellCount % 2 == 0) {
        *even = true;
        *ribCount = cellCount / 2 + 1;
    } else {
        *even = false;
        *ribCount = (cellCount + 1) / 2;
    }
}

// Interpolates y on the polyline at position x using the original's segment
// scan. strictLower reproduces the "xq < x" (vs "<=") variant used in some
// branches; the result is negated relative to reference height b11.
bool interpolateEdge(const CurveSegments &curve, double x, double b11,
                     bool strictLower, int extraIndex, double *out)
{
    bool found = false;
    const int limit = curve.count() + extraIndex;
    for (int k = 0; k < limit; ++k) {
        const double xq = k < curve.count() ? curve.xq[k] : 0.0;
        const double yq = k < curve.count() ? curve.yq[k] : 0.0;
        const double xp = k < curve.count() ? curve.xp[k] : 0.0;
        const double yp = k < curve.count() ? curve.yp[k] : 0.0;
        const bool lower = strictLower ? (xq < x) : (xq <= x);
        if (lower && xp >= x) {
            const double slope = (yp - yq) / (xp - xq);
            const double offset = yq - slope * xq;
            *out = -(slope * x + offset - b11);
            found = true;
        }
    }
    return found;
}

} // namespace

PreProcessorResult runPreProcessor(const PreProcessorInput &input)
{
    PreProcessorResult result;

    const double xm = input.halfSpan;
    const double span = xm * 2.0;

    if (!(xm > 0.0)) {
        result.error = "Half span xm must be positive.";
        return result;
    }
    if (input.leadingA1 <= 0.0 || input.leadingB1 <= 0.0
        || input.trailingA1 <= 0.0 || input.trailingB1 <= 0.0) {
        result.error = "Ellipse axes must be positive.";
        return result;
    }
    if (input.leadingA1 < xm || input.trailingA1 < xm) {
        result.error = "Leading and trailing edge a1 must be at least the half span xm.";
        return result;
    }

    //------------------------------------------------------------------
    // 1. Leading edge: ellipse sampled at dtheta = 0.01 with exponential
    // deflections after x1 (and x2), clamped at xm.
    //------------------------------------------------------------------
    const double leX1 = input.leadingX1;
    const double leX2 = input.leadingX2;
    const double xk1 = input.leadingC01 / std::pow(xm - leX1, input.leadingEx1);
    const double xk2 = input.leadingC02 / std::pow(xm - leX2, input.leadingEx2);
    const double b11 = input.leadingB1; // planform reference height

    CurveSegments leading;
    {
        const double a1 = input.leadingA1;
        const double b1 = input.leadingB1;
        int jcontrol = 0;
        const int steps = static_cast<int>((kPi / 2.0 - 0.01 + 0.01) / 0.01); // 157
        for (int i = 0; i < steps; ++i) {
            const double tetha = 0.01 * i;
            double xq = a1 * std::sin(tetha);
            double yq = b1 * std::cos(tetha);
            double xp = a1 * std::sin(tetha + 0.01);
            double yp = b1 * std::cos(tetha + 0.01);

            if (xq < leX1 && jcontrol == 0) {
                leading.add(xq, yq, xp, yp);
            } else if (xq >= leX1 && jcontrol < 2) {
                jcontrol = 1;
                if (xq < xm && xp >= xm) {
                    xp = xm;
                    yp = b1 * std::cos(std::asin(xp / a1));
                    jcontrol = 2;
                }
                if (xq >= leX1 && xq < leX2) {
                    yq -= xk1 * std::pow(xq - leX1, input.leadingEx1);
                    yp -= xk1 * std::pow(xp - leX1, input.leadingEx1);
                } else if (xq >= leX2) {
                    yq -= xk1 * std::pow(xq - leX1, input.leadingEx1)
                        + xk2 * std::pow(xq - leX2, input.leadingEx2);
                    yp -= xk1 * std::pow(xp - leX1, input.leadingEx1)
                        + xk2 * std::pow(xp - leX2, input.leadingEx2);
                }
                leading.add(xq, yq, xp, yp);
            }
        }
    }
    const int kmax1 = leading.count();
    if (kmax1 < 2) {
        result.error = "Leading edge parameters produced no curve; check x1 < xm <= a1.";
        return result;
    }

    for (int k = 0; k < kmax1; ++k)
        result.leadingEdge.push_back({leading.xq[k], -leading.yq[k] + b11});
    result.leadingEdge.push_back({leading.xp[kmax1 - 1], -leading.yp[kmax1 - 1] + b11});

    double chordmax = input.leadingB1;

    //------------------------------------------------------------------
    // 2. Trailing edge: inverted ellipse centred at (0, y0), one deflection.
    //------------------------------------------------------------------
    const double teX1 = input.trailingX1;
    const double xkTe = input.trailingC0 / std::pow(xm - teX1, input.trailingExp);
    chordmax += input.trailingB1 - input.trailingY0;

    CurveSegments trailing;
    {
        const double a1 = input.trailingA1;
        const double b1 = input.trailingB1;
        const double y0 = input.trailingY0;
        int jcontrol = 0;
        const int steps = static_cast<int>((kPi / 2.0 - 0.01 + 0.01) / 0.01); // 157
        for (int i = 0; i < steps; ++i) {
            const double tetha = 0.01 * i;
            double xq = a1 * std::sin(tetha);
            double yq = -b1 * std::cos(tetha) + y0;
            double xp = a1 * std::sin(tetha + 0.01);
            double yp = -b1 * std::cos(tetha + 0.01) + y0;

            if (xq < teX1 && jcontrol == 0) {
                trailing.add(xq, yq, xp, yp);
            } else if (xq >= teX1 && jcontrol < 2) {
                jcontrol = 1;
                if (xq < xm && xp >= xm) {
                    xp = xm;
                    yp = -b1 * std::cos(std::asin(xp / a1)) + y0;
                    jcontrol = 2;
                }
                yq -= xkTe * std::pow(xq - teX1, input.trailingExp);
                yp -= xkTe * std::pow(xp - teX1, input.trailingExp);
                trailing.add(xq, yq, xp, yp);
            }
        }
    }
    const int kmax2 = trailing.count();
    if (kmax2 < 2) {
        result.error = "Trailing edge parameters produced no curve; check x1 < xm <= a1.";
        return result;
    }

    for (int k = 0; k < kmax2; ++k)
        result.trailingEdge.push_back({trailing.xq[k], -trailing.yq[k] + b11});
    result.trailingEdge.push_back({trailing.xp[kmax2 - 1], -trailing.yp[kmax2 - 1] + b11});

    // Planform drawing: edges plus wingtip closing lines.
    for (int k = 0; k < kmax1; ++k)
        addMirroredSegment(result.drawing, leading.xq[k], -leading.yq[k] + b11,
                           leading.xp[k], -leading.yp[k] + b11, 1, 1);
    for (int k = 0; k < kmax2; ++k)
        addMirroredSegment(result.drawing, trailing.xq[k], -trailing.yq[k] + b11,
                           trailing.xp[k], -trailing.yp[k] + b11, 1, 1);
    addMirroredSegment(result.drawing,
                       leading.xp[kmax1 - 1], -leading.yp[kmax1 - 1] + b11,
                       trailing.xp[kmax2 - 1], -trailing.yp[kmax2 - 1] + b11, 1, 1);

    //------------------------------------------------------------------
    // 3. Vault. Both types produce a point chain whose developed length is
    // rescaled to the half span, plus one extrapolated "virtual" point used
    // by the interpolation of the last segment.
    //------------------------------------------------------------------
    std::vector<Point> vault;
    std::array<Point, 4> arcCenters{};
    double scalingLength = 0.0; // vault type 2: the original's (q, p) length sum

    if (input.vaultType == 1) {
        const double a1 = input.vaultA1;
        const double b1 = input.vaultB1;
        const double x1 = input.vaultX1;
        const double c1 = input.vaultC1;
        if (a1 <= 0.0 || b1 <= 0.0 || x1 <= 0.0 || x1 >= a1) {
            result.error = "Vault type 1 requires 0 < x1 < a1 and positive axes.";
            return result;
        }

        std::vector<Point> chain;
        int jcontrol = 0;
        for (int i = 0; i < 300; ++i) {
            const double theta1 = (kPi / 2.0) * i / 300.0;
            const double xq = a1 * std::sin(theta1);
            if (xq < x1) {
                chain.push_back({xq, b1 * std::sqrt(1.0 - xq * xq / (a1 * a1))});
            } else if (jcontrol == 0) {
                const double y1 = b1 * std::sqrt(1.0 - x1 * x1 / (a1 * a1));
                const double dy = y1 / 100.0;
                for (int j = 0; j <= 100; ++j) {
                    const double y = y1 - dy * j;
                    const double x = a1 * std::sqrt(std::max(0.0, 1.0 - y * y / (b1 * b1)))
                        + c1 * (1.0 - (std::cos((y1 - y) * kPi / y1) + 1.0) * 0.5);
                    chain.push_back({x, y});
                }
                jcontrol = 1;
            }
        }
        vault = chain;
        if (std::abs(vault.back().y) < 1.0e-4)
            vault.back().y = 0.0;
    } else if (input.vaultType == 2) {
        // Four tangent arcs: each center sits on the previous end radius.
        std::array<double, 4> startAngle{};
        double accumulated = 0.0;
        for (int c = 0; c < 4; ++c) {
            if (input.vaultRadius[c] <= 0.0 || input.vaultAngle[c] <= 0.0) {
                result.error = "Vault type 2 radii and angles must be positive.";
                return result;
            }
            startAngle[c] = accumulated;
            accumulated += input.vaultAngle[c];
        }

        arcCenters[0] = {0.0, -400.0};
        for (int c = 1; c < 4; ++c) {
            const double turn = (startAngle[c]) * kPi / 180.0;
            arcCenters[c] = {
                arcCenters[c - 1].x + (input.vaultRadius[c - 1] - input.vaultRadius[c]) * std::sin(turn),
                arcCenters[c - 1].y + (input.vaultRadius[c - 1] - input.vaultRadius[c]) * std::cos(turn)};
        }

        // The original iterates each arc with a single-precision DO loop whose
        // trip count gfortran evaluates in double from the REAL*4 bounds; an
        // arc can therefore get 98 instead of 99 samples. Its developed length
        // is summed over per-sample (q, p) pairs, so a short arc leaves the
        // chord bridging into the next arc out of the sum even though the
        // interpolation polyline contains it. Established wings were built
        // with that scaling, so reproduce both quirks exactly.
        double lastAngle = 0.0;
        float arcStartF = 0.0f;
        for (int c = 0; c < 4; ++c) {
            const float sweepF = static_cast<float>(input.vaultAngle[c]);
            const float stepF = sweepF / 99.0f;
            const float beginF = arcStartF;
            const float endF = beginF + sweepF - stepF;
            const int samples = static_cast<int>(
                (static_cast<double>(endF) - static_cast<double>(beginF)
                 + static_cast<double>(stepF)) / static_cast<double>(stepF));
            for (int j = 0; j < samples; ++j) {
                const double angle =
                    (static_cast<double>(beginF) + static_cast<double>(stepF) * j) * kPi / 180.0;
                const double next =
                    (static_cast<double>(beginF) + static_cast<double>(stepF) * (j + 1))
                    * kPi / 180.0;
                const Point q{arcCenters[c].x + input.vaultRadius[c] * std::sin(angle),
                              arcCenters[c].y + input.vaultRadius[c] * std::cos(angle)};
                const Point p{arcCenters[c].x + input.vaultRadius[c] * std::sin(next),
                              arcCenters[c].y + input.vaultRadius[c] * std::cos(next)};
                vault.push_back(q);
                scalingLength += std::hypot(q.x - p.x, q.y - p.y);
                lastAngle = next;
            }
            arcStartF += sweepF;
        }
        vault.push_back({arcCenters[3].x + input.vaultRadius[3] * std::sin(lastAngle),
                         arcCenters[3].y + input.vaultRadius[3] * std::cos(lastAngle)});
    } else {
        result.error = "Vault type must be 1 or 2.";
        return result;
    }

    if (vault.size() < 3) {
        result.error = "Vault parameters produced no curve.";
        return result;
    }

    // Rescale the developed length to the half span.
    double developed = 0.0;
    for (std::size_t k = 0; k + 1 < vault.size(); ++k)
        developed += std::hypot(vault[k].x - vault[k + 1].x, vault[k].y - vault[k + 1].y);
    if (input.vaultType == 1)
        scalingLength = developed;
    if (!(developed > 0.0) || !(scalingLength > 0.0)) {
        result.error = "Vault has zero developed length.";
        return result;
    }
    const double scale = xm / scalingLength;
    for (Point &p : vault) {
        p.x *= scale;
        p.y *= scale;
    }
    for (Point &p : arcCenters)
        p = {p.x * scale, p.y * scale};

    // Type 1 tops out at y = 0 already; type 2 is translated so the tip
    // touches y = 0, and its height is measured before the translation.
    const double vaultB1 = vault.front().y - (input.vaultType == 2 ? vault.back().y : 0.0);
    if (input.vaultType == 2) {
        const double tip = vault.back().y;
        for (Point &p : vault)
            p.y -= tip;
    }

    // Virtual point extending the last segment.
    const std::size_t last = vault.size() - 1;
    vault.push_back({vault[last].x + (vault[last].x - vault[last - 1].x),
                     vault[last].y + (vault[last].y - vault[last - 1].y)});

    const int vaultPoints = static_cast<int>(vault.size()) - 1; // without virtual point

    for (int k = 0; k < vaultPoints; ++k)
        result.vault.push_back({vault[k].x, vault[k].y});

    for (int k = 0; k + 1 < vaultPoints; ++k)
        addMirroredSegment(result.drawing, vault[k].x, -vault[k].y - 400.0,
                           vault[k + 1].x, -vault[k + 1].y - 400.0, 3, 1);

    //------------------------------------------------------------------
    // 4. Cell distribution: width of each half-wing cell (row 1 is the
    // centre rib with width 0 when the cell count is even).
    //------------------------------------------------------------------
    int cellCount = 0;
    int ribCount = 0;
    bool even = false;
    std::vector<double> width; // 1-based semantics kept 0-based here

    switch (input.cellDistribution) {
    case 1: {
        cellCount = input.cellCount;
        if (cellCount < 1) {
            result.error = "Cell count must be at least 1.";
            return result;
        }
        splitCells(cellCount, &even, &ribCount);
        width.assign(ribCount, span / cellCount);
        if (even)
            width[0] = 0.0;
        break;
    }
    case 2: {
        cellCount = input.cellCount;
        if (cellCount < 2) {
            result.error = "Cell count must be at least 2.";
            return result;
        }
        splitCells(cellCount, &even, &ribCount);
        const double xk = std::clamp(1.0 - input.cellCoefficient, 0.0, 1.0);
        const double xkCells = 2.0 * xk / cellCount;
        const double uniform = 2.0 * xm / cellCount;

        std::vector<double> x(ribCount, 0.0);
        width.assign(ribCount, 0.0);
        if (!even) {
            const double coefg = xm / (ribCount * uniform - 0.5 * uniform);
            for (int i = 0; i < ribCount; ++i) {
                const double centre = uniform * (i + 1) - 0.5 * uniform;
                width[i] = uniform * coefg - xkCells * centre;
            }
        } else {
            width[0] = 0.0;
            for (int i = 1; i < ribCount; ++i) {
                const double centre = uniform * i;
                width[i] = uniform - xkCells * centre;
            }
        }
        x[0] = width[0] / 2.0;
        for (int i = 1; i < ribCount; ++i)
            x[i] = x[i - 1] + width[i];
        const double normalize = xm / x[ribCount - 1];
        for (int i = 0; i < ribCount; ++i)
            width[i] *= normalize;
        break;
    }
    case 3: {
        cellCount = input.cellCount;
        if (cellCount < 2) {
            result.error = "Cell count must be at least 2.";
            return result;
        }
        splitCells(cellCount, &even, &ribCount);
        const double xk = input.cellCoefficient;

        width.assign(ribCount, span / cellCount);
        if (even)
            width[0] = 0.0;

        // Five fixed-point iterations, as in the original.
        for (int pass = 0; pass < 5; ++pass) {
            std::vector<double> x(ribCount, 0.0);
            x[0] = width[0] / 2.0;
            for (int i = 1; i < ribCount; ++i)
                x[i] = x[i - 1] + width[i];

            const int first = even ? 1 : 0;
            for (int i = first; i < ribCount; ++i) {
                double yLe = 0.0;
                double yTe = 0.0;
                interpolateEdge(leading, x[i], b11, !even, 1, &yLe);
                interpolateEdge(trailing, x[i], b11, !even, 0, &yTe);
                const double chord = std::abs(yLe - yTe);
                const double local = ((chordmax - chord) * xk + chord) / chordmax;
                width[i] = (span / cellCount) * local;
            }

            double s = width[0] / 2.0;
            for (int i = 1; i < ribCount; ++i)
                s += width[i];
            const double global = span / (2.0 * s);
            for (int i = 0; i < ribCount; ++i)
                width[i] *= global;
        }
        break;
    }
    case 4: {
        if (input.cellWidths.size() < 2) {
            result.error = "Explicit distribution needs at least two rib rows.";
            return result;
        }
        width = input.cellWidths;
        ribCount = static_cast<int>(width.size());
        even = width[0] < 0.01;
        cellCount = even ? (ribCount - 1) * 2 : 2 * ribCount - 1;
        double s = width[0] / 2.0;
        for (int i = 1; i < ribCount; ++i)
            s += width[i];
        if (!(s > 0.0)) {
            result.error = "Explicit cell widths must sum to a positive length.";
            return result;
        }
        const double normalize = span / (2.0 * s);
        for (double &w : width)
            w *= normalize;
        break;
    }
    default:
        result.error = "Cell distribution must be 1, 2, 3, or 4.";
        return result;
    }

    //------------------------------------------------------------------
    // 5. Rib matrix: planform positions plus interpolation along the vault.
    //------------------------------------------------------------------
    result.ribs.assign(ribCount, PreProcessorRib{});
    result.cellCount = cellCount;
    result.evenCellCount = even;
    result.totalRibCount = even ? 2 * ribCount - 1 : 2 * ribCount;

    std::vector<double> x(ribCount, 0.0);
    x[0] = width[0] / 2.0;
    for (int i = 1; i < ribCount; ++i)
        x[i] = x[i - 1] + width[i];
    // Every distribution puts the last rib at the half span exactly; pin it so
    // the wingtip never misses the final (clamped-at-xm) edge segments through
    // accumulated rounding. (The original loses the tip planform to this in
    // some inputs and prints 0.00 there.)
    x[ribCount - 1] = xm;

    for (int i = 0; i < ribCount; ++i) {
        PreProcessorRib &rib = result.ribs[i];
        rib.x = x[i];
        interpolateEdge(leading, x[i], b11, false, 0, &rib.yLeading);
        interpolateEdge(trailing, x[i], b11, false, 0, &rib.yTrailing);

        // Walk the vault by developed length; the matching segment is k+1,
        // exactly as in the original (positions inside the very first
        // segment keep xp = z = beta = 0).
        double along1 = 0.0;
        for (std::size_t k = 0; k + 2 < vault.size(); ++k) {
            along1 += std::hypot(vault[k].x - vault[k + 1].x, vault[k].y - vault[k + 1].y);
            const double along2 = along1
                + std::hypot(vault[k + 1].x - vault[k + 2].x, vault[k + 1].y - vault[k + 2].y);
            if (along1 < rib.x && along2 >= rib.x) {
                const Point &q = vault[k + 1];
                Point p = vault[k + 2];
                const double t = (rib.x - along1) / (along2 - along1);
                rib.xProjected = q.x + (p.x - q.x) * t;
                const double slope = (p.y - q.y) / (p.x - q.x);
                const double offset = q.y - slope * q.x;
                rib.z = vaultB1 - (slope * rib.xProjected + offset);
                if (std::abs(q.x - p.x) <= 1.0e-6)
                    p.x += 1.0e-5;
                rib.beta = (180.0 / kPi) * std::atan((p.y - q.y) / (q.x - p.x));
            }
        }
        if (i == ribCount - 1) {
            rib.xProjected = vault[vault.size() - 2].x;
            rib.z = vaultB1 + vault[vault.size() - 2].y;
        }

        addMirroredSegment(result.drawing, rib.x, rib.yLeading, rib.x, rib.yTrailing, 3, 1);
    }

    // Beta tick lines under the vault, as in the original drawing.
    for (int i = 0; i < ribCount; ++i) {
        const PreProcessorRib &rib = result.ribs[i];
        double beta = rib.beta == 90.0 ? rib.beta + 0.001 : rib.beta;
        const double dxv = 50.0 * std::sin(beta * kPi / 180.0);
        const double dyv = 50.0 * std::cos(beta * kPi / 180.0);
        const double sepy = -400.0 - vaultB1;
        result.drawing.push_back({rib.xProjected, rib.z + sepy,
                                  rib.xProjected, rib.z - 50.0 + sepy, 1});
        result.drawing.push_back({rib.xProjected, rib.z + sepy,
                                  rib.xProjected + dxv, rib.z - dyv + sepy, 2});
    }

    //------------------------------------------------------------------
    // 6. Summary values (metres and square metres like the original report).
    //------------------------------------------------------------------
    const PreProcessorRib &lastRib = result.ribs[ribCount - 1];
    result.flatSpan = span / 100.0;
    result.projectedSpan = lastRib.xProjected * 2.0 / 100.0;

    auto integrate = [&](auto position) {
        double area = position(result.ribs[0]) *
            (result.ribs[0].yTrailing - result.ribs[0].yLeading);
        for (int i = 0; i + 1 < ribCount; ++i) {
            const PreProcessorRib &a = result.ribs[i];
            const PreProcessorRib &b = result.ribs[i + 1];
            area += (position(b) - position(a)) * 0.5
                * (a.yTrailing - a.yLeading + b.yTrailing - b.yLeading);
        }
        return 2.0 * area / 10000.0;
    };
    result.flatArea = integrate([](const PreProcessorRib &r) { return r.x; });
    result.projectedArea = integrate([](const PreProcessorRib &r) { return r.xProjected; });

    result.flatAspectRatio = result.flatSpan * result.flatSpan / result.flatArea;
    result.projectedAspectRatio =
        result.projectedSpan * result.projectedSpan / result.projectedArea;
    result.flattening = (result.flatArea - result.projectedArea) / result.flatArea;
    result.maxChord = result.ribs[0].yTrailing - result.ribs[0].yLeading;
    result.midChord = result.flatArea / result.flatSpan * 100.0;
    result.minChord = lastRib.yTrailing - lastRib.yLeading;

    return result;
}

namespace {

class LineReader
{
public:
    explicit LineReader(const std::string &text)
    {
        std::istringstream stream(text);
        std::string line;
        while (std::getline(stream, line)) {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            lines_.push_back(line);
        }
    }

    bool next(std::string *line)
    {
        if (cursor_ >= lines_.size())
            return false;
        *line = lines_[cursor_++];
        return true;
    }

    bool skip(int count)
    {
        std::string ignored;
        for (int i = 0; i < count; ++i) {
            if (!next(&ignored))
                return false;
        }
        return true;
    }

private:
    std::vector<std::string> lines_;
    std::size_t cursor_ = 0;
};

std::vector<std::string> tokens(const std::string &line)
{
    std::vector<std::string> parts;
    std::istringstream stream(line);
    std::string part;
    while (stream >> part)
        parts.push_back(part);
    return parts;
}

// Reads a "name= value" parameter line the way the Fortran list-directed
// read does; also accepts "name=value" and a bare number.
bool namedValue(LineReader &reader, double *value)
{
    std::string line;
    if (!reader.next(&line))
        return false;
    const std::vector<std::string> parts = tokens(line);
    if (parts.empty())
        return false;
    std::string number = parts.back();
    if (parts.size() == 1) {
        const std::size_t equals = number.find('=');
        if (equals != std::string::npos)
            number = number.substr(equals + 1);
    }
    try {
        *value = std::stod(number);
    } catch (...) {
        return false;
    }
    return true;
}

bool intLine(LineReader &reader, int *value)
{
    double parsed = 0.0;
    if (!namedValue(reader, &parsed))
        return false;
    *value = static_cast<int>(parsed);
    return true;
}

void appendFormatted(std::string &out, const char *format, ...)
{
    char buffer[256];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(buffer, sizeof buffer, format, arguments);
    va_end(arguments);
    out += buffer;
}

const char *const kStars49 = "*************************************************";
const char *const kStars34 = "**********************************";

} // namespace

bool parsePreDataText(const std::string &text, PreProcessorInput *input, std::string *error)
{
    PreProcessorInput parsed;
    LineReader reader(text);
    const auto fail = [error](const char *message) {
        if (error != nullptr)
            *error = message;
        return false;
    };

    std::string line;
    if (!reader.skip(4) || !reader.next(&line))
        return fail("File is shorter than the pre-data.txt header.");
    parsed.wingName = line;
    if (!parsed.wingName.empty() && parsed.wingName.front() == '"') {
        parsed.wingName.erase(0, 1);
        if (!parsed.wingName.empty() && parsed.wingName.back() == '"')
            parsed.wingName.pop_back();
    }

    int leadingType = 0;
    if (!reader.skip(3) || !intLine(reader, &leadingType))
        return fail("Missing leading edge type.");
    if (leadingType != 1)
        return fail("Only leading edge type 1 is supported.");
    if (!namedValue(reader, &parsed.leadingA1) || !namedValue(reader, &parsed.leadingB1)
        || !namedValue(reader, &parsed.leadingX1) || !namedValue(reader, &parsed.leadingX2)
        || !namedValue(reader, &parsed.halfSpan) || !namedValue(reader, &parsed.leadingC01)
        || !namedValue(reader, &parsed.leadingEx1) || !namedValue(reader, &parsed.leadingC02)
        || !namedValue(reader, &parsed.leadingEx2))
        return fail("Malformed leading edge parameters.");

    int trailingType = 0;
    if (!reader.skip(3) || !intLine(reader, &trailingType))
        return fail("Missing trailing edge type.");
    if (trailingType != 1)
        return fail("Only trailing edge type 1 is supported.");
    double trailingHalfSpan = 0.0;
    if (!namedValue(reader, &parsed.trailingA1) || !namedValue(reader, &parsed.trailingB1)
        || !namedValue(reader, &parsed.trailingX1) || !namedValue(reader, &trailingHalfSpan)
        || !namedValue(reader, &parsed.trailingC0) || !namedValue(reader, &parsed.trailingY0)
        || !namedValue(reader, &parsed.trailingExp))
        return fail("Malformed trailing edge parameters.");

    if (!reader.skip(3) || !intLine(reader, &parsed.vaultType))
        return fail("Missing vault type.");
    if (parsed.vaultType == 1) {
        if (!namedValue(reader, &parsed.vaultA1) || !namedValue(reader, &parsed.vaultB1)
            || !namedValue(reader, &parsed.vaultX1) || !namedValue(reader, &parsed.vaultC1))
            return fail("Malformed vault type 1 parameters.");
    } else if (parsed.vaultType == 2) {
        for (int c = 0; c < 4; ++c) {
            if (!reader.next(&line))
                return fail("Vault type 2 needs four radius/angle rows.");
            const std::vector<std::string> parts = tokens(line);
            if (parts.size() < 2)
                return fail("Vault type 2 rows need a radius and an angle.");
            try {
                parsed.vaultRadius[c] = std::stod(parts[0]);
                parsed.vaultAngle[c] = std::stod(parts[1]);
            } catch (...) {
                return fail("Vault type 2 rows need numeric radius and angle.");
            }
        }
    } else {
        return fail("Vault type must be 1 or 2.");
    }

    if (!reader.skip(3) || !intLine(reader, &parsed.cellDistribution))
        return fail("Missing cell distribution type.");
    switch (parsed.cellDistribution) {
    case 1:
        if (!intLine(reader, &parsed.cellCount))
            return fail("Missing cell count.");
        break;
    case 2:
    case 3:
        if (!namedValue(reader, &parsed.cellCoefficient) || !intLine(reader, &parsed.cellCount))
            return fail("Missing cell coefficient or count.");
        break;
    case 4: {
        int rows = 0;
        if (!intLine(reader, &rows) || rows < 2 || rows > 100)
            return fail("Explicit distribution needs a rib row count between 2 and 100.");
        parsed.cellWidths.clear();
        for (int i = 0; i < rows; ++i) {
            if (!reader.next(&line))
                return fail("Missing explicit cell width row.");
            const std::vector<std::string> parts = tokens(line);
            if (parts.size() < 2)
                return fail("Explicit width rows need an index and a width.");
            try {
                parsed.cellWidths.push_back(std::stod(parts[1]));
            } catch (...) {
                return fail("Explicit width rows need numeric widths.");
            }
        }
        break;
    }
    default:
        return fail("Cell distribution must be 1, 2, 3, or 4.");
    }

    *input = parsed;
    return true;
}

std::string formatPreDataText(const PreProcessorInput &input)
{
    std::string out;
    const auto starLine = [&out] { out += kStars34; out += '\n'; };
    const auto value = [&out](const char *name, double v) {
        appendFormatted(out, "%s= %g\n", name, v);
    };

    starLine();
    out += "LEPARAGLIDING\n";
    out += "GEOMETRY PRE-PROCESSOR     v1.6\n";
    starLine();
    out += input.wingName + "\n";
    starLine();
    out += "1. Leading edge\n";
    starLine();
    out += "1\n";
    value("a1", input.leadingA1);
    value("b1", input.leadingB1);
    value("x1", input.leadingX1);
    value("x2", input.leadingX2);
    value("xm", input.halfSpan);
    value("c01", input.leadingC01);
    value("ex1", input.leadingEx1);
    value("c02", input.leadingC02);
    value("ex2", input.leadingEx2);
    starLine();
    out += "2. Trailing edge\n";
    starLine();
    out += "1\n";
    value("a1", input.trailingA1);
    value("b1", input.trailingB1);
    value("x1", input.trailingX1);
    value("xm", input.halfSpan);
    value("c0", input.trailingC0);
    value("y0", input.trailingY0);
    value("ex1", input.trailingExp);
    starLine();
    out += "3. Vault\n";
    starLine();
    appendFormatted(out, "%d\n", input.vaultType);
    if (input.vaultType == 1) {
        value("a1", input.vaultA1);
        value("b1", input.vaultB1);
        value("x1", input.vaultX1);
        value("c1", input.vaultC1);
    } else {
        for (int c = 0; c < 4; ++c)
            appendFormatted(out, "%.2f\t%.2f\n", input.vaultRadius[c], input.vaultAngle[c]);
    }
    starLine();
    out += "4. Cells\n";
    starLine();
    appendFormatted(out, "%d\n", input.cellDistribution);
    switch (input.cellDistribution) {
    case 1:
        appendFormatted(out, "%d\n", input.cellCount);
        break;
    case 2:
    case 3:
        appendFormatted(out, "%g\n%d\n", input.cellCoefficient, input.cellCount);
        break;
    case 4:
        appendFormatted(out, "%d\n", static_cast<int>(input.cellWidths.size()));
        for (std::size_t i = 0; i < input.cellWidths.size(); ++i)
            appendFormatted(out, "%d %.2f\n", static_cast<int>(i) + 1, input.cellWidths[i]);
        break;
    default:
        break;
    }
    return out;
}

std::string formatGeometryOutText(const PreProcessorInput &input,
                                  const PreProcessorResult &result)
{
    std::string out;
    const auto stars = [&out] { out += kStars49; out += '\n'; };

    stars();
    out += "LABORATORI D'ENVOL PARAGLIDING\n";
    stars();
    out += "GEOMETRY PRE-PROCESSOR\n";
    out += "Version 1.6 \"Canigo\" (2021-01-17)\n";
    stars();
    out += "Auxiliar geometry data for use with LEparagliding\n";
    stars();
    out += "1. Matrix of geometry\n";
    stars();
    out += "Rib\tx-rib       y-LE       y-TE   \t    xp         z\tbeta      RP        Washin\n";

    for (std::size_t i = 0; i < result.ribs.size(); ++i) {
        const PreProcessorRib &rib = result.ribs[i];
        appendFormatted(out, "%2d    %7.2f    %7.2f    %7.2f    %7.2f    %7.2f    %7.2f    %7.2f    %7.2f    \n",
                        static_cast<int>(i) + 1, rib.x, rib.yLeading, rib.yTrailing,
                        rib.xProjected, rib.z, rib.beta, rib.rotationPoint, rib.washin);
    }

    stars();
    out += "2. Main geometry paraglider data:\n";
    out += "***********************************************\n";
    appendFormatted(out, "Cells= %3d\n", result.cellCount);
    if (result.evenCellCount) {
        out += "0 Number of cells is even \n";
        appendFormatted(out, "Practical number of ribs %3d\n",
                        static_cast<int>(result.ribs.size()));
    } else {
        out += "1 Number of cells is odd \n";
        appendFormatted(out, "Number of ribs %3d\n", static_cast<int>(result.ribs.size()));
    }
    appendFormatted(out, "Span= %7.2f m\n", result.flatSpan);
    appendFormatted(out, "Span_proj= %7.2f m\n", result.projectedSpan);
    appendFormatted(out, "Surface= %7.2f m2\n", result.flatArea);
    appendFormatted(out, "Surface_proj= %7.2f m2\n", result.projectedArea);
    appendFormatted(out, "Aspect_Ratio= %7.2f\n", result.flatAspectRatio);
    appendFormatted(out, "Aspect_Ratio_proj= %7.2f\n", result.projectedAspectRatio);
    appendFormatted(out, "Flattening=  %5.2f\n", result.flattening);
    appendFormatted(out, "Max_chord= %7.2f cm\n", result.maxChord);
    appendFormatted(out, "Mid_chord= %7.2f cm\n", result.midChord);
    appendFormatted(out, "Min_chord= %7.2f cm\n", result.minChord);

    out += "***********************************************\n";
    out += "3. Some internal parameters:\n";
    out += "***********************************************\n";

    out += "***********************************************\n";
    out += "4. Leading edge, trailing edge, and vault coordinates:\n";
    out += "***********************************************\n";
    const auto coordinateList = [&out](const char *title,
                                       const std::vector<std::array<double, 2>> &points,
                                       double yOffset, double ySign) {
        appendFormatted(out, "%s\n%4d\n", title, static_cast<int>(points.size()));
        for (std::size_t i = 0; i < points.size(); ++i)
            appendFormatted(out, "%3d   %10.4f   %10.4f\n", static_cast<int>(i) + 1,
                            points[i][0], ySign * points[i][1] + yOffset);
    };
    coordinateList("Leading edge coordinates", result.leadingEdge, 0.0, 1.0);
    coordinateList("Trailing edge coordinates", result.trailingEdge, 0.0, 1.0);
    coordinateList("Vault coordinates", result.vault, -400.0, -1.0);

    (void)input;
    return out;
}

std::string formatGeometryDxfText(const PreProcessorResult &result)
{
    std::string out;
    out += "0\nSECTION\n2\nHEADER\n";
    out += "9\n$EXTMAX\n10\n    -900.000\n20\n      90.000\n";
    out += "9\n$EXTMIN\n10\n    5000.000\n20\n   -3000.000\n";
    out += "0\nENDSEC\n0\n";
    out += "SECTION\n2\nENTITIES\n0\n";
    for (const PreProcessorSegment &segment : result.drawing) {
        out += "LINE\n8\ndefault\n6\nCONTINUOUS\n";
        appendFormatted(out, "10\n%12.2f\n20\n%12.2f\n", segment.x1, -segment.y1);
        appendFormatted(out, "11\n%12.2f\n21\n%12.2f\n", segment.x2, -segment.y2);
        appendFormatted(out, "39\n%2d\n62\n%2d\n%2d\n", 0, segment.color, 0);
    }
    out += "ENDSEC\n0\nEOF\n";
    return out;
}

std::string formatSection1Rows(const PreProcessorResult &result)
{
    std::string out;
    for (std::size_t i = 0; i < result.ribs.size(); ++i) {
        const PreProcessorRib &rib = result.ribs[i];
        appendFormatted(out, "%2d %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f %9.2f %10.2f\t0.00\t50.0\n",
                        static_cast<int>(i) + 1, rib.x, rib.yLeading, rib.yTrailing,
                        rib.xProjected, rib.z, rib.beta, rib.rotationPoint, rib.washin);
    }
    return out;
}

} // namespace lep
