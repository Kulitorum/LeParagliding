#pragma once

#include <string>
#include <vector>

namespace lep {

// LEparagliding airfoil coordinate file (.txt): a name line, the total
// point count, then the extrados / intake / intrados point counts, then
// the (x, y) rows running from the trailing edge over the top to the nose
// and back. Consecutive segments share their boundary point, so
// extrados + intake + intrados - 2 == total. Pure C++ for the Qt-free
// test executables.
struct AirfoilFile
{
    std::string name;
    int extradosPoints = 0;
    int intakePoints = 0;
    int intradosPoints = 0;
    std::vector<double> xs;
    std::vector<double> ys;

    int totalPoints() const { return static_cast<int>(xs.size()); }
    // Segment ranges as [first, last] point indexes (inclusive), derived
    // from the header counts: extrados, intake, intrados in file order.
    struct Segment
    {
        int first = 0;
        int last = 0;
    };
    std::vector<Segment> segments() const;
};

// Parses the .txt airfoil format. Returns false with *error set when the
// file cannot be read; count inconsistencies are reported in *error but
// still return true with segments derived best-effort.
bool parseAirfoilFile(const std::string &text, AirfoilFile *airfoil,
                      std::string *error);

// Serializes in the canonical layout the engine's list-directed reader
// accepts (spacing is cosmetic).
std::string formatAirfoilFile(const AirfoilFile &airfoil);

} // namespace lep
