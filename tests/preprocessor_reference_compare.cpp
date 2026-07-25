// Compares the C++ port of the geometry pre-processor against reference
// geometry-out.txt files produced by the original Fortran pre-processor.f v1.6.
// Usage: preprocessor-reference-compare <pre-data.txt> <reference-geometry-out.txt>

#include "geometry_preprocessor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string readFile(const std::string &path)
{
    std::ifstream stream(path, std::ios::binary);
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

struct ReferenceData
{
    std::vector<lep::PreProcessorRib> ribs;
    int cellCount = 0;
    double flatSpan = 0.0;
    double projectedSpan = 0.0;
    double flatArea = 0.0;
    double projectedArea = 0.0;
};

bool parseReference(const std::string &text, ReferenceData *reference)
{
    std::istringstream stream(text);
    std::string line;
    bool inMatrix = false;
    while (std::getline(stream, line)) {
        if (line.find("Rib") == 0) {
            inMatrix = true;
            continue;
        }
        if (inMatrix) {
            lep::PreProcessorRib rib;
            int index = 0;
            std::istringstream row(line);
            if (row >> index >> rib.x >> rib.yLeading >> rib.yTrailing >> rib.xProjected
                    >> rib.z >> rib.beta >> rib.rotationPoint >> rib.washin) {
                reference->ribs.push_back(rib);
                continue;
            }
            inMatrix = false;
        }
        const auto numberAfter = [&line](const char *prefix, double *value) {
            const std::size_t at = line.find(prefix);
            if (at != 0)
                return false;
            *value = std::stod(line.substr(std::string(prefix).size()));
            return true;
        };
        double parsed = 0.0;
        if (numberAfter("Cells=", &parsed))
            reference->cellCount = static_cast<int>(parsed);
        else if (numberAfter("Span=", &parsed))
            reference->flatSpan = parsed;
        else if (numberAfter("Span_proj=", &parsed))
            reference->projectedSpan = parsed;
        else if (numberAfter("Surface=", &parsed))
            reference->flatArea = parsed;
        else if (numberAfter("Surface_proj=", &parsed))
            reference->projectedArea = parsed;
    }
    return !reference->ribs.empty();
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <pre-data.txt> <reference-geometry-out.txt>\n", argv[0]);
        return 2;
    }

    const std::string preData = readFile(argv[1]);
    const std::string referenceText = readFile(argv[2]);
    if (preData.empty() || referenceText.empty()) {
        std::fprintf(stderr, "failed to read input files\n");
        return 2;
    }

    lep::PreProcessorInput input;
    std::string error;
    if (!lep::parsePreDataText(preData, &input, &error)) {
        std::fprintf(stderr, "pre-data parse error: %s\n", error.c_str());
        return 1;
    }

    ReferenceData reference;
    if (!parseReference(referenceText, &reference)) {
        std::fprintf(stderr, "reference parse error\n");
        return 2;
    }

    const lep::PreProcessorResult result = lep::runPreProcessor(input);
    if (!result.ok()) {
        std::fprintf(stderr, "pre-processor error: %s\n", result.error.c_str());
        return 1;
    }

    int failures = 0;
    const auto check = [&failures](const char *what, double actual, double expected,
                                   double tolerance) {
        const double difference = std::abs(actual - expected);
        if (difference > tolerance) {
            std::fprintf(stderr, "MISMATCH %-12s actual %10.4f expected %10.4f (|d| %.4f > %.4f)\n",
                         what, actual, expected, difference, tolerance);
            ++failures;
        }
    };

    if (result.ribs.size() != reference.ribs.size()) {
        std::fprintf(stderr, "rib count mismatch: actual %zu expected %zu\n",
                     result.ribs.size(), reference.ribs.size());
        return 1;
    }
    check("cells", result.cellCount, reference.cellCount, 0.0);

    // The Fortran original sometimes loses the wingtip rib's planform to
    // accumulated rounding and prints 0.00 for its y-LE / y-TE (which also
    // shrinks the reported areas). The port always computes the tip, so skip
    // the affected fields when the reference shows that artifact.
    const lep::PreProcessorRib &referenceTip = reference.ribs.back();
    const bool referenceTipLost =
        referenceTip.x > 1.0 && referenceTip.yLeading == 0.0 && referenceTip.yTrailing == 0.0;
    if (referenceTipLost)
        std::printf("note: reference wingtip planform is 0.00/0.00 (original tip artifact); "
                    "skipping tip chord and area comparisons\n");

    double worst = 0.0;
    for (std::size_t i = 0; i < result.ribs.size(); ++i) {
        const lep::PreProcessorRib &a = result.ribs[i];
        const lep::PreProcessorRib &e = reference.ribs[i];
        const bool skipChord = referenceTipLost && i + 1 == result.ribs.size();
        char label[64];
        const auto at = [&](const char *column) {
            std::snprintf(label, sizeof label, "rib%zu.%s", i + 1, column);
            return label;
        };
        check(at("x"), a.x, e.x, 0.03);
        if (!skipChord) {
            check(at("yLE"), a.yLeading, e.yLeading, 0.03);
            check(at("yTE"), a.yTrailing, e.yTrailing, 0.03);
        }
        check(at("xp"), a.xProjected, e.xProjected, 0.03);
        check(at("z"), a.z, e.z, 0.03);
        check(at("beta"), a.beta, e.beta, 0.06);
        for (const double d : {a.x - e.x, a.xProjected - e.xProjected, a.z - e.z}) {
            worst = std::max(worst, std::abs(d));
        }
        if (!skipChord) {
            worst = std::max({worst, std::abs(a.yLeading - e.yLeading),
                              std::abs(a.yTrailing - e.yTrailing)});
        }
    }

    check("span", result.flatSpan, reference.flatSpan, 0.011);
    check("span_proj", result.projectedSpan, reference.projectedSpan, 0.011);
    if (!referenceTipLost) {
        check("area", result.flatArea, reference.flatArea, 0.011);
        check("area_proj", result.projectedArea, reference.projectedArea, 0.011);
    }

    std::printf("%s: %zu ribs compared, worst length deviation %.4f cm, %d failure(s)\n",
                argv[1], result.ribs.size(), worst, failures);
    return failures == 0 ? 0 : 1;
}
