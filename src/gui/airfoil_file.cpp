#include "airfoil_file.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>

namespace lep {

namespace {

std::vector<std::string> splitLines(const std::string &text)
{
    std::vector<std::string> lines;
    std::string current;
    for (const char ch : text) {
        if (ch == '\n') {
            lines.push_back(current);
            current.clear();
        } else if (ch != '\r') {
            current += ch;
        }
    }
    lines.push_back(current);
    return lines;
}

std::string trimmed(const std::string &line)
{
    size_t begin = 0;
    size_t end = line.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(line[begin])))
        ++begin;
    while (end > begin && std::isspace(static_cast<unsigned char>(line[end - 1])))
        --end;
    return line.substr(begin, end - begin);
}

bool parseInt(const std::string &line, int *value)
{
    char *end = nullptr;
    const std::string t = trimmed(line);
    *value = static_cast<int>(std::strtol(t.c_str(), &end, 10));
    return end && *end == '\0' && end != t.c_str();
}

} // namespace

std::vector<AirfoilFile::Segment> AirfoilFile::segments() const
{
    std::vector<Segment> result;
    const int total = totalPoints();
    if (total < 2)
        return result;
    // Boundary points are shared: extrados [0, e-1], intake [e-1, e+i-2],
    // intrados [e+i-2, total-1]. Fall back to one segment when the header
    // counts do not describe the point list.
    const int e = extradosPoints;
    const int i = intakePoints;
    if (e >= 2 && i >= 2 && intradosPoints >= 2
        && e + i + intradosPoints - 2 == total) {
        result.push_back({0, e - 1});
        result.push_back({e - 1, e + i - 2});
        result.push_back({e + i - 2, total - 1});
    } else {
        result.push_back({0, total - 1});
    }
    return result;
}

bool parseAirfoilFile(const std::string &text, AirfoilFile *airfoil,
                      std::string *error)
{
    *airfoil = AirfoilFile();
    error->clear();
    const std::vector<std::string> lines = splitLines(text);
    if (lines.size() < 6) {
        *error = "The airfoil file is too short to contain the name, the "
                 "four header counts and any points.";
        return false;
    }
    airfoil->name = trimmed(lines[0]);
    int total = 0;
    if (!parseInt(lines[1], &total)
        || !parseInt(lines[2], &airfoil->extradosPoints)
        || !parseInt(lines[3], &airfoil->intakePoints)
        || !parseInt(lines[4], &airfoil->intradosPoints)) {
        *error = "The four header counts (total, extrados, intake, "
                 "intrados points) could not be read.";
        return false;
    }
    for (size_t index = 5;
         index < lines.size()
         && static_cast<int>(airfoil->xs.size()) < total;
         ++index) {
        const std::string t = trimmed(lines[index]);
        if (t.empty())
            continue;
        double x = 0.0;
        double y = 0.0;
        if (std::sscanf(t.c_str(), "%lf %lf", &x, &y) != 2) {
            char buffer[96];
            std::snprintf(buffer, sizeof buffer,
                          "Point line %zu is not two numbers.", index + 1);
            *error = buffer;
            return false;
        }
        airfoil->xs.push_back(x);
        airfoil->ys.push_back(y);
    }
    if (airfoil->totalPoints() != total) {
        char buffer[128];
        std::snprintf(buffer, sizeof buffer,
                      "The header declares %d points but the file has %d.",
                      total, airfoil->totalPoints());
        *error = buffer;
        return false;
    }
    if (airfoil->extradosPoints + airfoil->intakePoints
            + airfoil->intradosPoints - 2
        != total) {
        char buffer[160];
        std::snprintf(buffer, sizeof buffer,
                      "Segment counts %d + %d + %d - 2 do not add up to the "
                      "%d points; treating the contour as one segment.",
                      airfoil->extradosPoints, airfoil->intakePoints,
                      airfoil->intradosPoints, total);
        *error = buffer; // informational; parsing still succeeded
    }
    return true;
}

std::string formatAirfoilFile(const AirfoilFile &airfoil)
{
    std::string result = airfoil.name + "\n";
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%4d\n", airfoil.totalPoints());
    result += buffer;
    std::snprintf(buffer, sizeof buffer, "%4d\n", airfoil.extradosPoints);
    result += buffer;
    std::snprintf(buffer, sizeof buffer, "%4d\n", airfoil.intakePoints);
    result += buffer;
    std::snprintf(buffer, sizeof buffer, "%4d\n", airfoil.intradosPoints);
    result += buffer;
    for (int i = 0; i < airfoil.totalPoints(); ++i) {
        std::snprintf(buffer, sizeof buffer, "%9.6f %12.6f\n",
                      airfoil.xs[static_cast<size_t>(i)],
                      airfoil.ys[static_cast<size_t>(i)]);
        result += buffer;
    }
    return result;
}

} // namespace lep
